# Real-Time Local Mapping: Implementation Plan

## 1. Introduction

### 1.1 Motivation

The Basalt SLAM system currently provides two decoupled subsystems: a real-time Visual-Inertial Odometry (VIO) thread operating on a fixed-lag sliding window, and an offline Non-linear Factor Recovery (NFR) Mapper that consumes serialized marginalization data from disk. The production integration in `basalt::Controller` wires only the VIO subsystem; no live mapping layer currently exists. This document specifies the design and implementation details of a **real-time incremental local mapping thread** (`basalt::LocalMapper`) that bridges the gap between VIO and future global mapping. The local mapper consumes `MargData` packets emitted by the VIO thread as keyframes (KFs) are marginalised, and maintains a bounded-size, optimised sub-map of the most recent covisible KFs. `basalt::Controller` can run SLAM either as a producer consumer system or an event driven system. The type of architecture implemented is determined at invocation of `Controller::initialize`. If `useProducerConsumerArchitecture` is passed as false, we use an event driven architecture, otherwise a producer consumer architecture is implemented.

The resulting system implements the first two tiers of a three-layered SLAM architecture:
- **Layer 1 — VIO** (`SqrtKeypointVioEstimator`): ~20 Hz fixed-lag smoother over the last ~10 frames + keyframes.
- **Layer 2 — Local Mapper** (`LocalMapper`): runs at the KF-marginalisation rate (~1–5 Hz) on a bounded covisibility window of ~50 keyframes; emits refined poses back to the VIO thread.
- **Layer 3 — Global Mapper / Loop Closure** (future): not dicussed yet.

### 1.2 Scope

**In scope:**
- Design and implementation of `LocalMapper` including its dedicated thread, data ingestion, incremental feature tracking, landmark management, keyframe culling, and bundle adjustment.
- Infrastructure upgrades required to support single-threaded deletion: `HashBowBase<N>` + `HashBowStl<N>` class hierarchy.
- Extensions to `TrackBuilder` / `UnionFind` for incremental tracking with persistent track IDs.
- Wiring of `VioEstimatorBase::out_marg_queue` into the Controller and a bidirectional pose-update feedback channel (`mpPosesToUpdate`) into the VIO estimator.
- `Controller::Stop()` for graceful, deadlock-free shutdown of the full SLAM pipeline.

**Out of scope (deferred to later PRs):**
- Loop closure detection and graph optimisation.
- Global map persistence (serialisation of the refined map to disk).
- Visualisation of the local map inside a live GUI.
- The parallel VO (`SqrtKeypointVoEstimator`) variant of pose-update application; this plan targets only the full-IMU VIO path. The VO path is addressed by the same `VioEstimatorBase::QueuePoseUpdates` virtual and can be implemented symmetrically.
- Optimisation of local mapping for time and space complexity reduction

Section 2 gives the high-level architecture. Sections 3–5 describe infrastructure changes (BoW, UnionFind/TrackBuilder, VIO extensions, Controller). Section 6 introduces the `LocalMapper` class. Section 7 walks through each pipeline stage in algorithmic detail. Section 8 documents concurrency guarantees. Sections 9–11 cover edge cases, the resolution of every previously-identified issue, and an implementation checklist.

---

## 2. High-Level Architecture

### 2.1 Three-Tier Data Flow

```
                  ┌──────────────┐
                  │   Camera +   │
                  │     IMU      │
                  └──────┬───────┘
                         │
                         ▼
                ┌────────────────────┐
                │ OpticalFlowBase    │    (thread #1: front-end)
                └────────┬───────────┘
                         │ vision_data_queue
                         ▼
            ┌─────────────────────────┐
            │ SqrtKeypointVioEstimator│   (thread #2: VIO back-end)
            │                         │
            │  mpPosesToUpdate ◀──────┼───────────┐
            └───┬─────────────────┬───┘           │
                │                 │               │ pose feedback
   out_state_q  │   out_marg_q    │               │ (mutex-protected)
                │                 │               │
                ▼                 ▼               │
    pose consumer        ┌────────────────┐       │
    (Controller)         │  LocalMapper   ├───────┘
                         │                │  (thread #3: local map)
                         └────────────────┘
```

### 2.2 Thread Model

| Thread | Owner | Lifetime | Communication |
|---|---|---|---|
| T1 Optical flow | `OpticalFlowBase::processing_thread` | Started on construction; joined in `OpticalFlowBase` subclass destructor | Pops `input_queue`; pushes to `output_queue` (= VIO's `vision_data_queue`) |
| T2 VIO | `SqrtKeypointVioEstimator::processing_thread` | Started by `initialize()`; joined by `maybe_join()` | Pops `vision_data_queue`/`imu_data_queue`; pushes to `out_state_queue`/`out_marg_queue` |
| T3 Local mapper | `LocalMapper::mpLocalMappingThread` | Started by `LocalMapper::Initialise()`; joined by `LocalMapper::Stop()` | Pops `local_map_input_queue_`; calls `mpVioPoseUpdateCallback` |
| T4 Pose consumer | `Controller::pose_processing_thread_` | Started by `initialize()`; joined by `Controller::Stop()`. This is only created and used when `useProducerConsumerArchitecture` is set to true at invocation of `Controller::initialize` | Pops `out_state_queue_`; updates `current_latest_pose_` |


The cascade sentinel-nullptr pattern (OF → VIO → LocalMapper) guarantees that each thread terminates only after the upstream thread has drained its queue.

### 2.3 Class Hierarchy Overview

```
VioEstimatorBase<Scalar>                    BundleAdjustmentBase<Scalar>
         △                                            △
         |                                            │
SqrtKeypointVioEstimator ────────────────►  ScBundleAdjustmentBase
    (+ mpPosesToUpdate)                               △
                                                      │
                                                  NfrMapper
                                                      △
                                                      │
                                                 LocalMapper
                                           (+ TrackBuilder, etc. as member
                                                  variables)

HashBowBase<N>
      △
      ├─── HashBow<N>    (TBB-backed, used by offline NfrMapper)
      └─── HashBowStl<N> (STL-backed, used by LocalMapper)
```

### 2.4 Pipeline Stages

A single iteration of `LocalMapper::MapLocally` processes one `MargData` packet through the following ten stages (aligned with the requirements specification):

```
 1. IngestMargData           ← new-KF detection by polling on `LocalMapper::local_map_input_queue_`, NFR factor extraction
 2. detect_keypoints         ← inherited from NfrMapper
 3. match_stereo             ← inherited; results promoted to mpLatestKeyframesMatches
 4. MatchLocal               ← BoW query restricted to new KFs
 5. build_tracks             ← LocalMapper override: Build or AddNewMatches
 6. setup_opt                ← LocalMapper override: skip-if-exists, consume mpRetiredTrackIds
 7. CullRedundantKeyframes   ← rehost + lmdb.removeFrame + prune all maps
 8. optimize(n)              ← inherited from NfrMapper
 9. filterOutliers           ← inherited from NfrMapper
10. optimize(n)              ← second pass, inherited
11. mpVioPoseUpdateCallback  ← push poses to VIO::mpPosesToUpdate
```

---

## 3. Bag-of-Words Matcher

### 3.1 Rationale

`HashBow<N>`'s inverted index is a `tbb::concurrent_unordered_map` of `tbb::concurrent_vector`. Neither TBB container supports safe element removal, which is essential for the local mapper when culled keyframes must be erased from the BoW database. Since the local mapper drives BoW from a single thread (`MapLocally` is serial), we can replace TBB with standard containers without incurring a concurrency penalty.

To avoid forking `NfrMapper` for the offline and real-time cases, we introduce a common base class and switch `NfrMapper::hash_bow_database` to a base pointer.

### 3.2 `HashBowBase<N>` (Abstract Base)

**File:** `include/basalt/hash_bow/hash_bow.h`

Hoisted from the current `HashBow<N>`:
- `num_bits` (member).
- `random_bit_permutation[512]` (`constexpr static`).
- `compute_permutation()` (`constexpr static`).
- `word_bit_permutation` (`constexpr static`).
- `compute_hash(const std::bitset<N>&)` — identical in both subclasses; depends only on the constexpr permutation.
- `compute_bow(const std::vector<std::bitset<N>>&, std::vector<FeatureHash>&, HashBowVector&)` — identical in both subclasses; it calls `compute_hash` and does per-BoW normalisation using only STL containers internally.

Declared virtual in `class HashBowBase`:
```cpp
virtual void add_to_database(const TimeCamId&, const HashBowVector&) = 0;
virtual void querry_database(const HashBowVector&, size_t,
                             std::vector<std::pair<TimeCamId, double>>&,
                             const int64_t* max_t_ns = nullptr) const = 0;
virtual ~HashBowBase() = default;
virtual void RemoveKeyframes(const std::vector<TimeCamId>& tcids){
    std::cout<<"Not Implemented"<<std::endl;
}
```

The spelling `querry_database` matches the existing Basalt API (`querry` vs `query`) — we keep it to avoid touching `nfr_mapper.cpp`.

### 3.3 `HashBow<N>` (TBB-backed, Unchanged Behaviour)

Inherits from `HashBowBase<N>`. Its `inverted_index` remains the TBB-backed structure. Only the header is updated to mark the two methods `override` and rest of the class remains the same.

### 3.4 `HashBowStl<N>` (New)

Inherits from `HashBowBase<N>`. The following containers are declared to use STL containers instead of tbb containers:
```cpp
std::unordered_map<FeatureHash,
                   std::vector<std::pair<TimeCamId, double>>,
                   std::hash<FeatureHash>> inverted_index;

std::unordered_map<TimeCamId,
                   std::vector<FeatureHash>,
                   std::hash<TimeCamId>> mpTcidHashIndex;
```

No mutex is introduced yet as the usage of the object of this class is single-threaded by construction. A comment in the header must state this explicitly.

**Method overrides:**

`add_to_database(tcid, bow_vector)`:
```
For each (hash, weight) in bow_vector:
    inverted_index[hash].emplace_back(tcid, weight)
    mpTcidHashIndex[tcid].push_back(hash)
```

`querry_database(...)`:
- Scoring formula identical to `HashBow::querry_database`.
- The only difference is container iteration; `std::unordered_map::find` and `std::vector` instead of their TBB equivalents.

The following additional method specific to `HashBowStl` for keyframe culling. A virtual method `HashBowBase::RemoveKeyframes` is defined to ensure compilation.

```cpp
void RemoveKeyframes(const std::vector<TimeCamId>& tcids);
```

Algorithm:
```
For each tcid in tcids:
    auto it = mpTcidHashIndex.find(tcid)
    if it == mpTcidHashIndex.end(): continue
    For each hash in it->second:
        auto vec_it = inverted_index.find(hash)
        if vec_it == inverted_index.end(): continue
        auto& vec = vec_it->second
        vec.erase(std::remove_if(vec.begin(), vec.end(),
                    [&](const auto& p){ return p.first == tcid; }),
                  vec.end())
        if vec.empty(): inverted_index.erase(vec_it)
    mpTcidHashIndex.erase(it)
```

Complexity is O(∑|hashes_of_tcid| × |vec|). In the local map (≤50 KFs), each vec stays bounded; this is cheap.

### 3.5 `NfrMapper` Changes

In `include/basalt/vi_estimator/nfr_mapper.h`:
```cpp
// Forward declaration BEFORE class NfrMapper:
template <size_t N> class HashBowBase;

// Member change:
std::shared_ptr<HashBowBase<256>> hash_bow_database;
```

In `src/vi_estimator/nfr_mapper.cpp::NfrMapper::NfrMapper`:
```cpp
// Unchanged — concrete type TBB version for offline mapper:
hash_bow_database.reset(new HashBow<256>(config.mapper_bow_num_bits));
```

The point: the base pointer lets `LocalMapper` inject `HashBowStl<256>` instead, without modifying any downstream code.

---

## 4. Feature tracking

`struct UnionFind` and `struct TrackBuilder` are used for feature tracking and must be updated to incorporate incremental tracking features required by `class LocalMapper`.

### 4.1 UnionFind Extensions

**File:** `include/basalt/utils/union_find.h`

Two additional methods:

```cpp
// Appends a single new independent node (self-parent).
// Returns the DSU index of the new node (== GetNumNodes() before the call).
ValueType AddIndex() {
    ValueType new_idx = static_cast<ValueType>(m_cc_size.size());
    m_cc_parent.push_back(new_idx);
    m_cc_rank.push_back(0);
    m_cc_size.push_back(1);
    return new_idx;
}

// Marks root_idx as invalid. Any subsequent Find() on nodes in this set
// will return InvalidIndex() (path compression propagates the marker).
// PRECONDITION: root_idx is a current root (Find(root_idx) == root_idx).
// CALLER must also erase root_idx from mpUFNodeIndexToTrackIdMap.
void InvalidateRoot(ValueType root_idx) {
    BASALT_ASSERT(root_idx < m_cc_parent.size());
    m_cc_parent[root_idx] = InvalidIndex();
}
```

**Rationale for the single-node `AddIndex`:** Callers typically add one node at a time as new matches arrive. A batch-variant `AddIndices(n)` can be added later if profiling shows it beneficial; the single-node variant keeps `AddNewMatches` simpler because it can resolve indices for new feature pairs on the fly.

### 4.2 TrackBuilder

#### 4.2.1 Design Motivation

The current `TrackBuilder::map_node_to_index` stores `TrackId` as the value type. This conflates two distinct concepts:

1. The **DSU array index** — an internal `uint32_t` used to reach into `UnionFind::m_cc_parent`. These are not stable: `CompactTree` will renumber them.
2. The **persistent TrackId** — the external, stable identifier that downstream consumers (the landmark database) use as a landmark key. Once assigned, a TrackId must never change.

Fusing the two works for the offline one-shot use case but breaks as soon as incremental compaction is introduced. The redesign separates them:

- `map_node_to_index : ImageFeaturePair → uint32_t` (DSU index, may be renumbered).
- `mpUFNodeIndexToTrackIdMap : uint32_t (root) → TrackId` (stable TrackId; the key is the current root).

Invariant: a TrackId value, once assigned from `mpNextTrackId`, is stable for the lifetime of the mapper; the `uint32_t` root mapping to it may change after `CompactTree`, but the TrackId value does not.

#### 4.2.2 Updated Structure

**File:** `include/basalt/utils/tracks.h`

```cpp
struct TrackBuilder {
    // TYPE CHANGE: was std::map<ImageFeaturePair, TrackId>
    std::map<ImageFeaturePair, uint32_t> map_node_to_index;

    UnionFind uf_tree;

    // NEW: stable persistent TrackId management
    std::unordered_map<uint32_t, TrackId> mpUFNodeIndexToTrackIdMap;
    TrackId mpNextTrackId = 0;

    // NEW: populated by RemoveObservations; consumed by DeleteTracksAfterCulling
    std::set<TrackId> mpTracksWithCulledObs;
    std::set<TrackId> mpTracksWithLiveObs;

    // Existing methods (updated internally):
    void Build(const Matches& map_pair_wise_matches);
    bool Filter(size_t minimumTrackLength = 2);
    void Export(FeatureTracks& tracks);
    size_t TrackCount() const;

    // New methods (see §4.4–§4.8):
    void AddNewMatches(const Matches& new_matches,
                       const LandmarkDatabase<double>& lmdb,
                       FeatureTracks& updated_tracks,
                       std::set<TrackId>& retired_ids);

    void RemoveObservations(const std::set<FrameId>& culled_frame_ids);
    void DeleteTracksAfterCulling(const std::set<FrameId>& culled_frame_ids);
    void CompactTree();
};
```

### 4.2.3`Build()` Update

For the very first batch (when `LocalMapper::mpIsTrackBuilderInitialised == false`), `Build` must assign stable TrackIds alongside the DSU graph construction instead of assigning incremental root IDs as track IDs as was previously done.

Algorithm:
```
Build(map_pair_wise_matches):
  map_node_to_index.clear()
  mpUFNodeIndexToTrackIdMap.clear()
  uf_tree = UnionFind()

  Phase 1 — collect all ImageFeaturePairs in a sorted set (determinism). Remains as is.

  Phase 2 — assign DSU indices. IDs assigned change from an incremental local variable to `uf_tree` size dependent index. This allows for incremental track building:
    for each feat in allFeatures:
        map_node_to_index.emplace(feat, uf_tree.AddIndex())

  Phase 3 — Union matches:
    for each (pair, matchData) in map_pair_wise_matches:
        for each (fi, fj) in matchData.inliers:
            uf_tree.Union(map_node_to_index[{pair.first, fi}],
                          map_node_to_index[{pair.second, fj}])

  Phase 4 — assign TrackIds to every distinct root:
    for each (_, dsu_idx) in map_node_to_index:
        uint32_t root = uf_tree.Find(dsu_idx)
        if root != InvalidIndex() && mpUFNodeIndexToTrackIdMap.count(root) == 0:
            mpUFNodeIndexToTrackIdMap[root] = mpNextTrackId++
```

### 4.2.4 `Filter()` Update

The current `Filter` has two flaws for the incremental case:
- It iterates `uf_tree.m_cc_parent` by reference and mutates array cells in place. Non-root entries get assigned `InvalidIndex()` when they shouldn't — non-root cells always point to their parent, and that parent's validity is the thing that matters.
- It does not clean `mpUFNodeIndexToTrackIdMap`, leaking retired TrackId entries.

Corrected algorithm:
```
Filter(min_track_length):
  // Build per-root TimeCamId sets, detect duplicates and too-short tracks.
  std::map<uint32_t, std::set<TimeCamId>> tracks_by_root
  std::set<uint32_t> problematic_roots

  for each (feat_pair, dsu_idx) in map_node_to_index:
      uint32_t root = uf_tree.Find(dsu_idx)
      if root == InvalidIndex(): continue
      if problematic_roots.count(root): continue
      const TimeCamId& tcid = feat_pair.first
      if tracks_by_root[root].count(tcid):
          problematic_roots.insert(root)   // same image twice → conflict
      else:
          tracks_by_root[root].insert(tcid)

  for each (root, tcid_set) in tracks_by_root:
      if tcid_set.size() < min_track_length:
          problematic_roots.insert(root)

  for each bad_root in problematic_roots:
      if uf_tree.m_cc_parent[bad_root] != InvalidIndex():
          uf_tree.InvalidateRoot(bad_root)
          mpUFNodeIndexToTrackIdMap.erase(bad_root)

  return false
```

We do NOT also erase the feature pairs themselves from `map_node_to_index`; that would break `CompactTree`, which relies on every live node being present in the map. Invalidation of the root is sufficient because `Find()` will propagate `InvalidIndex()` through the tree via path compression the next time those nodes are visited.

### 4.2.5 `Export()` Update

`TrackBuilder::Export` uses root indices for indentifying and exporting tracks. We now want to use incremental track IDs instead.

```
Export(tracks):
  tracks.clear()
  for each (feat_pair, dsu_idx) in map_node_to_index:
      uint32_t root = uf_tree.Find(dsu_idx)
      if root == InvalidIndex(): continue
      TrackId tid = mpUFNodeIndexToTrackIdMap.at(root)  // stable ID
      tracks[tid].emplace(feat_pair)
```

### 4.2.6 `AddNewMatches()` Full Algorithm

Full signature:
```cpp
void AddNewMatches(const Matches& new_matches,
                   const LandmarkDatabase<double>& lmdb,
                   FeatureTracks& updated_tracks,
                   std::set<TrackId>& retired_ids);
```

**Algorithm:**

```
AddNewMatches(new_matches, lmdb, updated_tracks, retired_ids):
  retired_ids.clear()
  updated_tracks.clear()

  // ─── Step A: collect all image-feature pairs touched by new_matches ───
  std::set<ImageFeaturePair> new_feats
  for each (pair, matchData) in new_matches:
      for each (fi, fj) in matchData.inliers:
          new_feats.insert({pair.first, fi})
          new_feats.insert({pair.second, fj})

  // ─── Step B: add new DSU nodes for unseen feature pairs ───
  for each feat in new_feats:
      if map_node_to_index.count(feat) == 0:
          map_node_to_index[feat] = uf_tree.AddIndex()

  // ─── Step C: perform Union with survivor-preference heuristic ───
  for each (pair, matchData) in new_matches:
      for each (fi, fj) in matchData.inliers:
          uint32_t idx_i = map_node_to_index.at({pair.first, fi})
          uint32_t idx_j = map_node_to_index.at({pair.second, fj})
          uint32_t root_i = uf_tree.Find(idx_i)
          uint32_t root_j = uf_tree.Find(idx_j)
          if root_i == root_j || root_i == InvalidIndex() || root_j == InvalidIndex():
              continue

          TrackId tid_i = mpUFNodeIndexToTrackIdMap.count(root_i)
                          ? mpUFNodeIndexToTrackIdMap.at(root_i) : -1
          TrackId tid_j = mpUFNodeIndexToTrackIdMap.count(root_j)
                          ? mpUFNodeIndexToTrackIdMap.at(root_j) : -1
          bool i_has_lm = (tid_i >= 0 && lmdb.landmarkExists(tid_i))
          bool j_has_lm = (tid_j >= 0 && lmdb.landmarkExists(tid_j))

          // Survivor heuristic (priority order):
          //   1) Existing landmark wins → preserve optimised geometry.
          //   2) Larger track wins     → stability under merges.
          //   3) DSU union-by-rank     → default.
          bool force_i_wins = (i_has_lm && !j_has_lm) ||
                              (i_has_lm == j_has_lm &&
                               uf_tree.m_cc_size[root_i] > uf_tree.m_cc_size[root_j])
          bool force_j_wins = (j_has_lm && !i_has_lm) ||
                              (!force_i_wins && i_has_lm == j_has_lm &&
                               uf_tree.m_cc_size[root_j] > uf_tree.m_cc_size[root_i])

          uint32_t survivor_root, retired_root
          if force_i_wins:
              uf_tree.m_cc_parent[root_j] = root_i
              uf_tree.m_cc_size[root_i]  += uf_tree.m_cc_size[root_j]
              survivor_root = root_i; retired_root = root_j
          else if force_j_wins:
              uf_tree.m_cc_parent[root_i] = root_j
              uf_tree.m_cc_size[root_j]  += uf_tree.m_cc_size[root_i]
              survivor_root = root_j; retired_root = root_i
          else:
              uf_tree.Union(idx_i, idx_j)
              uint32_t new_root = uf_tree.Find(idx_i)
              survivor_root = new_root
              retired_root  = (new_root == root_i) ? root_j : root_i

          // Retire the losing track's TrackId (if it had one).
          if mpUFNodeIndexToTrackIdMap.count(retired_root):
              retired_ids.insert(mpUFNodeIndexToTrackIdMap.at(retired_root))
              mpUFNodeIndexToTrackIdMap.erase(retired_root)
          // Survivor keeps its existing TrackId if any; else assign below in step D.

  // ─── Step D: assign TrackIds to brand-new singleton/minimal tracks ───
  for each (_, dsu_idx) in map_node_to_index:
      uint32_t root = uf_tree.Find(dsu_idx)
      if root == InvalidIndex(): continue
      if mpUFNodeIndexToTrackIdMap.count(root) == 0:
          mpUFNodeIndexToTrackIdMap[root] = mpNextTrackId++

  // ─── Step E: filter (length + conflict) — updates mpUFNodeIndexToTrackIdMap ───
  Filter(/*min_track_length=*/ 2)

  // ─── Step F: compute the set of TrackIds touched by new_matches ───
  std::set<TrackId> touched_tids
  for each feat in new_feats:
      uint32_t root = uf_tree.Find(map_node_to_index.at(feat))
      if root != InvalidIndex():
          touched_tids.insert(mpUFNodeIndexToTrackIdMap.at(root))

  // ─── Step G: export only touched tracks into updated_tracks ───
  for each (feat_pair, dsu_idx) in map_node_to_index:
      uint32_t root = uf_tree.Find(dsu_idx)
      if root == InvalidIndex(): continue
      TrackId tid = mpUFNodeIndexToTrackIdMap.at(root)
      if touched_tids.count(tid):
          updated_tracks[tid].emplace(feat_pair)

  // ─── Step H: compact the tree if the ghost + invalid fraction is high ───
  size_t ghost_invalid = uf_tree.GetNumNodes() - CountLiveNodes()
  if ghost_invalid * 2 >= uf_tree.GetNumNodes():
      CompactTree()
```

Where `CountLiveNodes()` is the number of `map_node_to_index` entries whose `Find()` returns a non-invalid root. "Ghost" nodes are those that were in `map_node_to_index` but have been removed via `RemoveObservations` (which erases from the map but not the DSU arrays). "Invalid" nodes are those whose root has been invalidated.

### 4.2.7 `RemoveObservations()` and `DeleteTracksAfterCulling()`

```cpp
void RemoveObservations(const std::set<FrameId>& culled_frame_ids);
```

Algorithm:
```
RemoveObservations(culled_frame_ids):
  // 1. Identify features belonging to culled frames; track affected TrackIds.
  std::vector<ImageFeaturePair> to_erase
  for each (feat_pair, dsu_idx) in map_node_to_index:
      if culled_frame_ids.count(feat_pair.first.frame_id):
          to_erase.push_back(feat_pair)
          uint32_t root = uf_tree.Find(dsu_idx)
          if root != InvalidIndex() && mpUFNodeIndexToTrackIdMap.count(root):
              mpTracksWithCulledObs.insert(mpUFNodeIndexToTrackIdMap.at(root))

  // 2. Erase from map_node_to_index (creates ghost nodes in DSU).
  for each pair in to_erase:
      map_node_to_index.erase(pair)

  // 3. Determine which affected tracks still have at least one live observation.
  for each (feat_pair, dsu_idx) in map_node_to_index:
      uint32_t root = uf_tree.Find(dsu_idx)
      if root == InvalidIndex(): continue
      TrackId tid = mpUFNodeIndexToTrackIdMap.at(root)
      if mpTracksWithCulledObs.count(tid):
          mpTracksWithLiveObs.insert(tid)
```

```cpp
void DeleteTracksAfterCulling(const std::set<FrameId>& culled_frame_ids);
```

Algorithm:
```
DeleteTracksAfterCulling(culled_frame_ids):
  RemoveObservations(culled_frame_ids)

  // Any track with culled obs AND no remaining live obs → invalidate root.
  std::set<TrackId> to_kill
  for each tid in mpTracksWithCulledObs:
      if mpTracksWithLiveObs.count(tid) == 0:
          to_kill.insert(tid)

  if !to_kill.empty():
      // Build the reverse {TrackId → root} map once (O(n)).
      std::unordered_map<TrackId, uint32_t> tid_to_root
      for each (root_idx, tid) in mpUFNodeIndexToTrackIdMap:
          tid_to_root[tid] = root_idx
      for each tid in to_kill:
          auto it = tid_to_root.find(tid)
          if it != tid_to_root.end():
              uf_tree.InvalidateRoot(it->second)
              mpUFNodeIndexToTrackIdMap.erase(it->second)

  mpTracksWithCulledObs.clear()
  mpTracksWithLiveObs.clear()
```

### 4.2.8 `CompactTree()`

**Goal:** rebuild `uf_tree` so that only live nodes remain (densely indexed `0..N-1`), preserving the same set-equivalence (same TrackIds map to the same groups of feature pairs).

**Algorithm (O(N log N)):**
```
CompactTree():
  // 1. Group live nodes by their current root (i.e. by track).
  std::unordered_map<uint32_t, std::vector<ImageFeaturePair>> old_root_to_feats
  std::unordered_map<uint32_t, TrackId> old_root_to_tid
  for each (feat_pair, old_idx) in map_node_to_index:
      uint32_t root = uf_tree.Find(old_idx)
      if root == InvalidIndex(): continue
      old_root_to_feats[root].push_back(feat_pair)
      old_root_to_tid[root] = mpUFNodeIndexToTrackIdMap.at(root)

  // 2. Build a fresh UnionFind with exactly enough nodes.
  UnionFind new_uf
  std::map<ImageFeaturePair, uint32_t> new_map
  std::unordered_map<uint32_t, TrackId> new_tid_map
  for each (old_root, feats) in old_root_to_feats:
      uint32_t first_new_idx = (uint32_t) new_uf.AddIndex()
      new_map[feats[0]] = first_new_idx
      for i in 1..feats.size()-1:
          uint32_t idx = new_uf.AddIndex()
          new_map[feats[i]] = idx
          new_uf.Union(first_new_idx, idx)
      uint32_t new_root = new_uf.Find(first_new_idx)
      new_tid_map[new_root] = old_root_to_tid[old_root]

  // 3. Swap in the compacted structures.
  uf_tree = std::move(new_uf)
  map_node_to_index = std::move(new_map)
  mpUFNodeIndexToTrackIdMap = std::move(new_tid_map)
```

The critical property: `mpNextTrackId` is NOT reset. TrackIds remain stable across compaction. Landmark keys in `lmdb` therefore remain valid — exactly what the consumer requires.

---

## 5. VIO Updated Pose Feedback

### 5.1 Rationale

The local mapper refines keyframe poses via BA. Those refinements must be communicated back to VIO so that it does not continue to drift relative to the corrected map. Two constraints:

1. **FEJ consistency.** VIO marginalises at fixed linearisation points (First-Estimate Jacobians). If we overwrite a pose without resetting the linearisation, the Jacobians of the marginalisation prior will have inconsistent nullspace relative to the new pose, re-introducing spurious information. We must therefore reconstruct the `PoseStateWithLin` rather than copy it, and, if the correction is large, reset the linearisation flag on `frame_states` entries.

2. **Thread safety.** The mapper thread writes; the VIO thread reads and applies. A simple mutex-protected staging map is sufficient; we do not need lock-free atomics because updates are rare (≤ few Hz) relative to VIO's frame rate.

### 5.2 Additions to `VioEstimatorBase<Scalar>`

**File:** `include/basalt/vi_estimator/vio_estimator.h`

Add a virtual method (default no-op, to be overridden by VIO and VO):
```cpp
virtual void QueuePoseUpdates(
    const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>& updates) {
    (void)updates; // default no-op; the VO path can override later.
}
```

### 5.3 Additions to `SqrtKeypointVioEstimator<Scalar>`

**File:** `include/basalt/vi_estimator/sqrt_keypoint_vio.h`

Private members:
```cpp
Eigen::aligned_unordered_map<int64_t, PoseStateWithLin<double>> mpPosesToUpdate;
mutable std::mutex mpPosesToUpdateMutex;

// Threshold above which we reset linearisation (in meters).
static constexpr Scalar kRelinThresholdTrans = Scalar(0.10);
// Threshold in radians (rotation component of SE3 log).
static constexpr Scalar kRelinThresholdRot = Scalar(0.05);
```

Override `QueuePoseUpdates`:
```cpp
void QueuePoseUpdates(
    const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>& updates) override {
    std::lock_guard<std::mutex> lock(mpPosesToUpdateMutex);
    for (const auto& kv : updates) mpPosesToUpdate[kv.first] = kv.second;
}
```

### 5.4 Pose application

**File:** `src/vi_estimator/sqrt_keypoint_vio.cpp`

At the very top of `SqrtKeypointVioEstimator<Scalar>::measure()` (before IMU preintegration and frame-states manipulation):

```cpp
{
    std::lock_guard<std::mutex> lock(mpPosesToUpdateMutex);
    if (!mpPosesToUpdate.empty()) {
        for(Eigen::aligned_unordered_map<int64_t, PoseStateWithLin<double>>::iterator it = mpPosesToUpdate.begin(); it != mpPosesToUpdate.end();) {
            int64_t t_ns = it->first;
            PoseStateWithLin<double> lm_pose = it->second;
            const SE3 T_new = lm_pose.getPose().template cast<Scalar>();

            auto it_fp = frame_poses.find(t_ns);
            if (it_fp != frame_poses.end()) {
                // Reconstruct to drop the mapper's linearisation point.
                // PoseStateWithLin(t_ns, T_w_i) defaults linearized=false.
                const SE3 T_old = it_fp->second.getPose();
                SE3 diff = T_new.inverse() * T_old;
                Scalar trans_err = diff.translation().norm();
                Scalar rot_err   = diff.so3().log().norm();
                if (trans_err > kRelinThresholdTrans ||
                    rot_err   > kRelinThresholdRot) {
                    // Large correction — reset linearization to prevent FEJ drift.
                    // Commented out for experimentation
                    // it_fp->second = PoseStateWithLin<Scalar>(t_ns, T_new, false);
                    std::cout<<"too large update in pose, not updating" <<std::endl;
                } else {
                    // Small correction — apply but keep linearised form if it was.
                    bool was_lin = it_fp->second.isLinearized();
                    it_fp->second = PoseStateWithLin<Scalar>(t_ns, T_new, was_lin);
                    it = mpPosesToUpdate.erase(it);
                    continue;
                }
            }
            it++;
            // We intentionally do NOT update frame_states here: the active
            // sliding-window states (pose+vel+bias) are still being optimised
            // by VIO and overwriting them mid-loop would violate the IMU
            // preintegration invariants. Only the already-marginalised KFs
            // carried in frame_poses are safe to overwrite.
        }
    }
}
```

 `frame_poses` entries correspond to keyframes that have already had velocity and bias marginalised out (they are pose-only). These are the entries the mapper optimises. `frame_states` entries are still live navigation states with ongoing IMU preintegration coupling them to neighbours. Overwriting a `frame_states` pose mid-integration would break the `BASALT_ASSERT` equality check between preintegration start state and `frame_states[last_state_t_ns]` at `measure()` line 352. Furthermore, due to `BASALT_ASSERT(frame_poses.at(kv.first).isLinearized());` in `src/vi_estimator/ba_base.cpp:321`, we can not set change the linearisation of the poses updated currently. Thus, in the barebones implementation we only update `frame_poses` if updates are small.

### 5.5 Lock Scope Summary

The mutex covers:
- Reads of `mpPosesToUpdate`.
- Writes of `mpPosesToUpdate.clear()` and the callback's `emplace`.

The mutex does NOT cover the subsequent optimisation or marginalisation — `frame_poses` is accessed exclusively by the VIO thread after the lock is released. The mapper never writes directly to VIO's `frame_poses`.

---

## 6. The LocalMapper Class

### 6.1 Full Header

**File:** `include/basalt/vi_estimator/local_mapper.h`

```cpp
#pragma once
#include "basalt/vi_estimator/nfr_mapper.h"
#include "basalt/utils/tracks.h"
#include "basalt/hash_bow/hash_bow.h"
#include <atomic>
#include <functional>
#include <mutex>
#include <set>
#include <thread>
#include <tbb/concurrent_queue.h>

namespace basalt {

class LocalMapper : public basalt::NfrMapper {
public:
    using Scalar = double;
    using Ptr = std::shared_ptr<LocalMapper>;
    using PoseUpdateCallback = std::function<void(
        const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>&)>;

    LocalMapper(const Calibration<double>& calib, const VioConfig& config);
    ~LocalMapper();

    // ── Lifecycle ───────────────────────────────────────────────────
    void Initialise();
    void Stop();
    void SetMarginalisationDataInputQueue(
        tbb::concurrent_bounded_queue<MargData::Ptr>* queue);
    void SetVIOPoseUpdateCallback(PoseUpdateCallback cb);

    // ── Pipeline (public for testability) ───────────────────────────
    void MapLocally();                            // thread entry point
    void IngestMargData(MargData::Ptr& data);
    void MatchLocal();
    void build_tracks();                          // shadows NfrMapper
    void setup_opt();                             // shadows NfrMapper
    void CullRedundantKeyframes();
    size_t ComputeCovisibility(int64_t tid_a, int64_t tid_b, int num_cameras);
    void CollectNewKeyframesAfterMatching();

    // ── Config ──────────────────────────────────────────────────────
    size_t mpMaxLocalMapSize          = 50;
    double mpCullCovisibilityThresh   = 0.90;
    int    mpOptIterations            = 5;
    double mpFilterOutlierThreshold   = 3.0;  // pixels * obs_std_dev scaling

    // ── State exposed for tests and introspection ───────────────────
    std::set<int64_t> mpNewKeyframesForTracking;

    std::unordered_map<
        std::pair<TimeCamId, TimeCamId>, MatchData,
        std::hash<std::pair<TimeCamId, TimeCamId>>,
        std::equal_to<std::pair<TimeCamId, TimeCamId>>,
        Eigen::aligned_allocator<
            std::pair<const std::pair<TimeCamId, TimeCamId>, MatchData>>>
        mpLatestKeyframesMatches;

    TrackBuilder mpTrackBuilder;
    bool mpIsTrackBuilderInitialised = false;
    std::set<TrackId> mpRetiredTrackIds;

    std::atomic<bool> mpStopLocalMapping{false};
    std::atomic<bool> mpIsMargDataInputQueueSet{false};
    std::thread mpLocalMappingThread;

private:
    tbb::concurrent_bounded_queue<MargData::Ptr>* mpMargInputQueue = nullptr;
    PoseUpdateCallback mpVioPoseUpdateCallback;

    // Helpers
    int64_t SelectKeyframeToCull();
    int64_t FindBestRehostKf(int64_t culled_kf, TrackId lm_id,
                             const std::set<int64_t>& candidates);
    void RehostLandmark(TrackId lm_id, int64_t culled_kf,
                        int64_t new_host_kf);
};

} // namespace basalt
```

### 6.2 Constructor

```cpp
LocalMapper::LocalMapper(const Calibration<double>& calib, const VioConfig& config)
    : NfrMapper(calib, config) {
    // Replace NfrMapper's HashBow<256> (TBB) with HashBowStl<256>.
    hash_bow_database = std::make_shared<HashBowStl<256>>(config.mapper_bow_num_bits);

    // Ensure bounded queue behaviour if Controller forgets to set capacity.
    // The actual capacity is set by Controller; this is a safety default.
}
```

### 6.3 Destructor and `Stop()`

```cpp
LocalMapper::~LocalMapper() {
    Stop();
}

void LocalMapper::Stop() {
    mpStopLocalMapping = true;
    // Do NOT push nullptr to mpMargInputQueue here — VIO is the sole owner
    // of the nullptr sentinel in that queue. Relying on VIO's sentinel keeps
    // the cascade contract deterministic.
    if (mpLocalMappingThread.joinable()) {
        mpLocalMappingThread.join();
    }
    mpNextTrackId = 0;
}
```

### 6.4 `Initialise()` and Queue Wiring

```cpp
void LocalMapper::Initialise() {
    mpStopLocalMapping = false;
    mpIsTrackBuilderInitialised = false;
    mpRetiredTrackIds.clear();
    mpNewKeyframesForTracking.clear();
    mpLatestKeyframesMatches.clear();

    // Spawn the mapping thread. It busy-waits in MapLocally until the queue
    // is wired (mpIsMargDataInputQueueSet == true).
    mpLocalMappingThread = std::thread(&LocalMapper::MapLocally, this);
}

void LocalMapper::SetMarginalisationDataInputQueue(
        tbb::concurrent_bounded_queue<MargData::Ptr>* queue) {
    mpMargInputQueue = queue;
    mpIsMargDataInputQueueSet = true;
}

void LocalMapper::SetVIOPoseUpdateCallback(PoseUpdateCallback cb) {
    mpVioPoseUpdateCallback = std::move(cb);
}
```

### 6.5 `MapLocally()` Thread Entry

```cpp
void LocalMapper::MapLocally() {
    // Wait until the queue is wired.
    while (!mpStopLocalMapping && !mpIsMargDataInputQueueSet) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    while (!mpStopLocalMapping) {
        MargData::Ptr data;
        mpMargInputQueue->pop(data);     // blocking
        if (!data) break;                // VIO's nullptr sentinel → shutdown

        IngestMargData(data);

        if (mpNewKeyframesForTracking.empty()) continue;

        detect_keypoints();          // inherited — only touches img_data (now filtered)
        match_stereo();              // inherited — writes to feature_matches
        MatchLocal();                // BoW cross-frame matching for new KFs
        CollectNewKeyframesAfterMatching();      // promote new-KF stereo into mpLatestKeyframesMatches

        build_tracks();              // LocalMapper (shadowing)
        setup_opt();                 // LocalMapper (shadowing)

        CullRedundantKeyframes();

        optimize(mpOptIterations);
        filterOutliers(mpFilterOutlierThreshold, 4);
        optimize(mpOptIterations);

        if (mpVioPoseUpdateCallback) mpVioPoseUpdateCallback(frame_poses);
    }
}
```

---

## 7. Pipeline Component Details

### 7.1 Marginalisation Data Ingestion

#### 7.1.1 Operational Requirement

`IngestMargData` must:
- Identify keyframes that are in the `MargData` window but not yet in `LocalMapper::frame_poses` ("new KFs").
- Run the inherited `processMargData` (eliminates velocity/bias, populates `img_data`).
- Run the inherited `extractNonlinearFactors` (pushes `RollPitchFactor`/`RelPoseFactor`).
- Update `frame_poses` for ALL KFs in `kfs_all` — **unconditionally on NFR validity** — so even rank-deficient cases still propagate the latest absolute poses.
- Filter `img_data` down to ONLY the new KFs (avoid redundant detection on already-mapped KFs).

We must also note the following additional requirements:
- **New KFs** must always be added; skipping them would leave them undetected and unmatched downstream.
- **Existing KFs** are updated only when NFR is valid. `extractNonlinearFactors` returning `false` means the *marginal* is rank-deficient — the pose estimate itself is still the current VIO best, but for consistency with `class NfrMapper` we update only if `valid` is `true`.

#### 7.1.2 Algorithm

```
IngestMargData(data):
  mpNewKeyframesForTracking.clear()
  mpLatestKeyframesMatches.clear()

  // Step 1 — detect new KFs.
  for each id in data->kfs_all:
      if frame_poses.count(id) == 0:
          mpNewKeyframesForTracking.emplace(id);

  // Step 2 — inherited factor extraction (mutates data).
  processMargData(*data)
  bool valid = extractNonlinearFactors(*data)

  // Step 3 — update frame_poses unconditionally (all KFs in kfs_all).
  // Note: after processMargData, KFs with POSE_VEL_BIAS state have been
  // moved from data->frame_states into data->frame_poses; non-KF POSE_VEL_BIAS
  // entries have been erased.
  for each (t_ns, pose_state) in data->frame_poses:
      // Reconstruct to drop any stale linearisation point from MargData.
      if mpNewKeyframesForTracking.count(t_ns):
          frame_poses[t_ns] = PoseStateWithLin<double>(
              pose_state.getT_ns(), pose_state.getPose());

  if (valid){
      // Update as previously done to update older keyframes in 
      for (const auto& kv : data->frame_poses) {
          if(mpNewKeyframesForTracking.count(kv.first) == 0){
              PoseStateWithLin<double> p(kv.second.getT_ns(),
                                         kv.second.getPose());

              frame_poses[kv.first] = p;
          }
      }

      for (const auto& kv : data->frame_states) {
          if(mpNewKeyframesForTracking.count(kv.first) == 0){
              if (data->kfs_all.count(kv.first) > 0) {
                  auto state = kv.second;
                  PoseStateWithLin<double> p(state.getState().t_ns,
                                             state.getState().T_w_i);
                  frame_poses[kv.first] = p;
              }
          }
      }

  }

  // Step 4 — filter img_data to keep only new KF images.
  // processMargData has already populated img_data for every KF in kfs_all;
  // for KFs already known to the local map, re-detection is wasted work.
  for (auto it = img_data.begin(); it != img_data.end(); ) {
      if (mpNewKeyframesForTracking.count(it->first) == 0) it = img_data.erase(it);
      else ++it;
  }

```

### 7.2 Keypoint Detection

`NfrMapper::detect_keypoints` is used unchanged. Because §7.1.3 filters `img_data` to new KFs only, the parallel-for loop in `detect_keypoints` automatically restricts itself to those frames. `feature_corners` accumulates the results; the entries for culled KFs are removed in §7.6 via `feature_corners.unsafe_erase(tcid)`.

**TBB safety note:** `feature_corners` is a `tbb::concurrent_unordered_map`. `detect_keypoints` launches a `tbb::parallel_for` that writes new entries concurrently. Culling (§7.6) calls `unsafe_erase` from the single mapper thread, outside of any `parallel_for`. This is safe because `unsafe_erase` is documented as non-thread-safe; we guarantee no concurrent writers at that point by only removing between pipeline stages.

### 7.3 Feature Matching

#### 7.3.1 Stereo Matching — `match_stereo` Inherited

`NfrMapper::match_stereo` iterates over all timestamps in `img_data` and adds stereo match entries into `feature_matches` (TBB concurrent map). We also need to the latest stereo matches in `mpLatestKeyframesMatches`. Thus, after `NfrMapper::match_stereo` is invoked, we promote the stereo matches that involve any new KF into `mpLatestKeyframesMatches`, through `LocalMapper::CollectNewKeyframesAfterMatching` to be run after `LocalMapper::MatchLocal` is completed.

```cpp
void LocalMapper::CollectNewKeyframesAfterMatching() {

    for (const auto& kv : feature_matches) {
        const auto& pair = kv.first;
        if (mpNewKeyframesForTracking.count(pair.first.frame_id) ||
            mpNewKeyframesForTracking.count(pair.second.frame_id)) {
            mpLatestKeyframesMatches[pair] = kv.second;
        }
    }
    // feature_matches retains these entries; they'll be cleared at the start
    // of the next iteration.
}
```

#### 7.3.3 `MatchLocal` (BoW Cross-Frame Matching)

`NfrMapper::match_all` iterates over all timestamps in `feature_corners` and because `feature_corners` contains entries for new KFs (just-detected) and the accumulated entries for older KFs (persisted across iterations), `NfrMapper::match_all` as written would attempt matching for all keyframes, redundantly. Thus, `NfrMapper::match_all` is replaced with `LocalMapper::MatchLocal`. `LocalMapper::MatchLocal` mirrors its logic but restricts the query source to new KFs.

```cpp
void LocalMapper::MatchLocal() {
    std::vector<TimeCamId> keys;
    std::unordered_map<TimeCamId, size_t> id_to_key_idx;

    for (const auto& kv : feature_corners) {
        id_to_key_idx[kv.first] = keys.size();
        keys.push_back(kv.first);
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    struct match_pair {
        size_t i;
        size_t j;
        double score;
    };

    tbb::concurrent_vector<match_pair> ids_to_match;

    tbb::blocked_range<size_t> keys_range(0, keys.size());
    auto compute_pairs = [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
            const TimeCamId& tcid = keys[i];
            if (mpNewKeyframesForTracking.count(tcid.frame_id) > 0){
                const KeypointsData& kd = feature_corners.at(tcid);

                std::vector<std::pair<TimeCamId, double>> results;

                hash_bow_database->querry_database(
                    kd.bow_vector, config.mapper_num_frames_to_match, results,
                    &tcid.frame_id);

                // std::cout << "Closest frames for " << tcid << ": ";
                for (const auto& otcid_score : results) {
                    // std::cout << otcid_score.first << "(" << otcid_score.second
                    // << ")
                    // ";
                    if (otcid_score.first.frame_id != tcid.frame_id &&
                        otcid_score.second >
                            config.mapper_frames_to_match_threshold) {
                        match_pair m;
                        m.i = i;
                        m.j = id_to_key_idx.at(otcid_score.first);
                        m.score = otcid_score.second;

                        ids_to_match.emplace_back(m);
                    }
                }
            }
            // std::cout << std::endl;
        }
    };

    tbb::parallel_for(keys_range, compute_pairs);
    // compute_pairs(keys_range);

    auto t2 = std::chrono::high_resolution_clock::now();

    std::cout << "Matching " << ids_to_match.size() << " image pairs..."
              << std::endl;

    std::atomic<int> total_matched = 0;

    tbb::blocked_range<size_t> range(0, ids_to_match.size());
    auto match_func = [&](const tbb::blocked_range<size_t>& r) {
        int matched = 0;

        for (size_t j = r.begin(); j != r.end(); ++j) {
            const TimeCamId& id1 = keys[ids_to_match[j].i];
            const TimeCamId& id2 = keys[ids_to_match[j].j];

            const KeypointsData& f1 = feature_corners[id1];
            const KeypointsData& f2 = feature_corners[id2];

            MatchData md;

            matchDescriptors(f1.corner_descriptors, f2.corner_descriptors,
                             md.matches, 70, 1.2);

            if (int(md.matches.size()) > config.mapper_min_matches) {
                matched++;

                findInliersRansac(f1, f2, config.mapper_ransac_threshold,
                                  config.mapper_min_matches, md);
            }

            if (!md.inliers.empty()){
                feature_matches[std::make_pair(id1, id2)] = md;
            }
        }
        total_matched += matched;
    };

    tbb::parallel_for(range, match_func);
    // match_func(range);

    auto t3 = std::chrono::high_resolution_clock::now();

    auto elapsed1 =
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    auto elapsed2 =
        std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);

    //
    int num_matches = 0;
    int num_inliers = 0;

    for (const auto& kv : mpLatestKeyframesMatches) {
        num_matches += kv.second.matches.size();
        num_inliers += kv.second.inliers.size();
    }

    std::cout << "Matched " << ids_to_match.size() << " image pairs with "
              << num_inliers << " inlier matches (" << num_matches << " total)."
              << std::endl;

    std::cout << "DB query " << elapsed1.count() * 1e-6 << "s. matching "
              << elapsed2.count() * 1e-6
              << "s. Geometric verification attemts: " << total_matched << "."
              << std::endl;
}
```

Note: the matching helpers (`matchDescriptors`, `findInliersRansac`) are static free functions defined in `include/basalt/utils/keypoint.h` and imported into `include/basalt/vi_estimatro/nfr_mapper.h`. These will be imported into `include/basalt/vi_estimator/local_mapper.h` with the include for `include/basalt/vi_estimator/nfr_mapper.h`.

### 7.4 Incremental Track Building

**`LocalMapper::build_tracks`:**
```cpp
void LocalMapper::build_tracks() {
    if (!mpIsTrackBuilderInitialised) {
        // Convert mpLatestKeyframesMatches (std::unordered_map) to Matches (TBB map)
        // because TrackBuilder::Build uses Matches API. A simple copy suffices.
        Matches tbb_matches;
        for (const auto& kv : mpLatestKeyframesMatches)
            tbb_matches.insert({kv.first, kv.second});

        mpTrackBuilder.Build(tbb_matches);
        mpTrackBuilder.Filter(config.mapper_min_track_length);
        feature_tracks.clear();
        mpTrackBuilder.Export(feature_tracks);
        mpIsTrackBuilderInitialised = true;
    } else {
        Matches tbb_matches;
        for (const auto& kv : mpLatestKeyframesMatches)
            tbb_matches.insert({kv.first, kv.second});

        feature_tracks.clear();
        mpTrackBuilder.AddNewMatches(
            tbb_matches, lmdb, feature_tracks, mpRetiredTrackIds);
    }
}
```

**Type conversion note:** `TrackBuilder::Build` takes a `const Matches&`. Since `Matches` is `tbb::concurrent_unordered_map`, and `mpLatestKeyframesMatches` is `std::unordered_map`, we either (a) convert at call sites, or (b) refactor `TrackBuilder` to take a concept/templated matches container. Option (a) is simpler and the conversion cost is minimal for small match sets; option (b) is a follow-up refactor deferred for later updates.

### 7.5 Landmark Database Update

#### 7.5.1 Design Intent

`LocalMapper::setup_opt` must differ from `NfrMapper::setup_opt` in two ways:

1. **Retire merged landmarks before adding new ones.** When `AddNewMatches` merges two tracks with existing landmarks, the retired track's TrackId is no longer a valid landmark key. If we call `lmdb.addLandmark` for the survivor TrackId without first removing the retired TrackId, the retired landmark's observations remain in `lmdb`, orphaned.

2. **Skip-if-exists for landmark triangulation.** Running `lmdb.addLandmark(tid, new_pos)` when `tid` already exists would overwrite the current (optimised) landmark 3D position with a freshly-triangulated one. This destroys all the optimisation progress. The correct behaviour: triangulate only if `!lmdb.landmarkExists(tid)`; always add observations (already idempotent).

#### 7.5.2 Algorithm

```cpp
void LocalMapper::setup_opt() {
    const double min_triang_dist2 =
        config.mapper_min_triangulation_dist *
        config.mapper_min_triangulation_dist;

    // Step A — retire merged landmarks.
    for (TrackId retired : mpRetiredTrackIds) {
        if (lmdb.landmarkExists(retired)) {
            lmdb.removeLandmark(retired);
        }
    }
    mpRetiredTrackIds.clear();

    // Step C — iterate updated feature tracks.
    for (const auto& kv : feature_tracks) {
        if (kv.second.size() < 2) continue;

        // // Process only tracks involving at least one new KF.
        // // This logic will be commented but added for later experimentation
        // bool touches_new_kf = false;
        // for (const auto& obs : kv.second)
        //     if (mpNewKeyframesForTracking.count(obs.first.frame_id)) { touches_new_kf = true; break; }
        // if (!touches_new_kf) continue;

        // Add landmark iff not yet known to lmdb.
        if (!lmdb.landmarkExists(kv.first)) {
            auto it_h = kv.second.begin();
            TimeCamId tcid_h = it_h->first;

            FeatureId fid_h  = it_h->second;
            Eigen::Vector2d pos_2d_h =
                feature_corners.at(tcid_h).corners[fid_h];
            Eigen::Vector4d pos_3d_h;
            calib.intrinsics[tcid_h.cam_id].unproject(pos_2d_h, pos_3d_h);

            bool triangulated = false;
            for (auto it_o = std::next(it_h); it_o != kv.second.end(); ++it_o) {
                TimeCamId tcid_o = it_o->first;
                FeatureId fid_o  = it_o->second;
                if (feature_corners.count(tcid_o) == 0) continue;
                if (!frame_poses.count(tcid_h.frame_id)) continue;
                if (!frame_poses.count(tcid_o.frame_id)) continue;

                Eigen::Vector2d pos_2d_o =
                    feature_corners.at(tcid_o).corners[fid_o];
                Eigen::Vector4d pos_3d_o;
                calib.intrinsics[tcid_o.cam_id].unproject(pos_2d_o, pos_3d_o);

                Sophus::SE3d T_w_h =
                    frame_poses.at(tcid_h.frame_id).getPose() *
                    calib.T_i_c[tcid_h.cam_id];
                Sophus::SE3d T_w_o =
                    frame_poses.at(tcid_o.frame_id).getPose() *
                    calib.T_i_c[tcid_o.cam_id];
                Sophus::SE3d T_h_o = T_w_h.inverse() * T_w_o;

                if (T_h_o.translation().squaredNorm() < min_triang_dist2)
                    continue;

                Eigen::Vector4d pos_3d = triangulate(
                    pos_3d_h.head<3>(), pos_3d_o.head<3>(), T_h_o);
                if (!pos_3d.array().isFinite().all() ||
                     pos_3d[3] <= 0 || pos_3d[3] > 2.0)
                    continue;

                Keypoint<Scalar> kpt;
                kpt.host_kf_id = tcid_h;
                kpt.direction  = StereographicParam<double>::project(pos_3d);
                kpt.inv_dist   = pos_3d[3];
                lmdb.addLandmark(kv.first, kpt);
                triangulated = true;
                break;
            }
            if (!triangulated) continue;  // no baseline good enough → defer
        }

        // Add observations (idempotent — safe to repeat).
        for (const auto& obs_kv : kv.second) {
            if (!frame_poses.count(obs_kv.first.frame_id)) continue;
            if (feature_corners.count(obs_kv.first) == 0)    continue;
            KeypointObservation<Scalar> ko;
            ko.kpt_id = kv.first;
            ko.pos    = feature_corners.at(obs_kv.first).corners[obs_kv.second];
            lmdb.addObservation(obs_kv.first, ko);
        }
    }
}
```

### 7.6 Keyframe Culling and Landmark Rehosting

#### 7.6.1 Culling Criteria

Two-stage decision (matching the requirement):

1. **Redundancy-based (preferred):** cull any KF in the "older" region (excluding the N most recent KFs, e.g. last 5) whose landmark observation set overlaps more than `mpCullCovisibilityThresh = 0.90` with any other single KF in the local map.

2. **Capacity-based (fallback):** if the map exceeds `mpMaxLocalMapSize`, cull the oldest non-recent KF, if not keyframes fulfilling criteria 1 are found.

#### 7.6.2 Covisibility Computation

`LandmarkDatabase::getObservations()` returns the inverted `host_kf → {target_kf → {kpt_id...}}` index. Covisibility of KF `a` with KF `b` (as defined in the requirements: total shared landmarks, regardless of which hosts which) is:

```
covis(a, b) =
    sum over cam_id_a, cam_id_b of:
        |observations[{a,cam_id_a}].at({b,cam_id_b})|     (landmarks hosted by a, seen by b)
      + |observations[{b,cam_id_b}].at({a,cam_id_a})|     (landmarks hosted by b, seen by a)
```

For stereo, the sum iterates over `(cam_id_a ∈ {0,1}, cam_id_b ∈ {0,1})`. Missing entries contribute 0.

For the coverage fraction (for the primary culling criterion):
```
total(a) = number of landmarks with host frame_id == a + number of observations of landmarks hosted in other frames observed by a
         = sum over cam_id of |lmdb.getLandmarksForHost({a, cam_id})| + sum over tid (tid ∈ all keyframes - {TimeCamId(a, cam_id)}) of |lmdb.getNonLandmarkObservationsCountForKeyFrame(tid, TimeCamId(a, cam_id))|
```

A KF `a` is culled if `∃ b ≠ a : covis(a, b) / total(a) ≥ 0.90`.

If `total(a) == 0` (KF hosts no landmarks), the fraction is undefined so the keyframe should not be culled based on this criterion. Such a KF is caught by a fallback capacity rule `if (total_a == 0) continue;`.

```cpp
size_t LocalMapper::ComputeCovisibility(int64_t tid_a, int64_t tid_b, int num_cameras){
    int covisibility = 0;
    for(int i = 0; i < num_cameras; i++){
        for(int j = 0; j < num_cameras; j++){
            covisibility += lmdb.getObservationsCountForPair(TimeCamId(tid_a, i), TimeCamId(tid_b, j));
            covisibility += lmdb.getObservationsCountForPair(TimeCamId(tid_b, j), TimeCamId(tid_a, i));
        }
    }
    return covisibility;
}
```

The following public methods must be added in `class LandmarkDatabase` in `include/basalt/vi_estimator/landmark_database.h` and `src/basalt/vi_estimator/landmark_database.cpp`:

```cpp
size_t LandmarkDatabase::getObservationsCountForPair(const TimeCamId& tid_a, const TimeCamId &tid_b){
    std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<KeypointId>>>::iterator it = observations.find(tid_a);
    if(it != observations.end()){
        std::map<TimeCamId, std::set<KeypointId>>::iterator jt = it->second.find(tid_b);
        if(jt != it->second.end()){
            return jt->second.size();
        }
    }
    return 0;
}
```

```cpp
size_t LandmarkDatabase::getNonLandmarkObservationsCountForKeyFrame(TimeCamId& tid){
    size_t observations_count = 0;
    for(std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<KeypointId>>>::iterator it = observations.begin(); it != observations.end(); it++){
        if(it->first == tid){
            continue;
        }
        std::map<TimeCamId, std::set<KeypointId>>::iterator jt = it->second.find(tid);
        if(jt != it->second.end()){
            observations_count += jt->second.size();
        }
    }
    return observations_count;
}
```

#### 7.6.3 `SelectKeyframeToCull()`

```cpp
int64_t LocalMapper::SelectKeyframeToCull() {
    if (frame_poses.size() <= 5) return -1;  // keep minimum map

    // Order KFs by timestamp; the last `mpNewKeyframesForTracking.size()` are "recent" and untouchable.
    std::vector<int64_t> ordered;
    for (const auto& kv : frame_poses){
        ordered.push_back(kv.first);
    }
    std::sort(ordered.begin(), ordered.end());
    size_t keep_recent = std::min<size_t>(mpNewKeyframesForTracking.size(), ordered.size());
    size_t eligible_end = ordered.size() - keep_recent;

    // Criterion 1 — redundancy.
    for (size_t i = 0; i < eligible_end; ++i) {
        int64_t a = ordered[i];
        // Compute total(a).
        size_t total_a = 0;
        for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam){
            total_a += lmdb.getLandmarksForHost(TimeCamId(a, cam)).size();
            total_a += lmdb.getNonLandmarkObservationsCountForKeyFrame(TimeCamId(a, cam));
        }
        if (total_a == 0){
            continue;
        }

        // Compute covisibility against every other KF b.
        for (size_t j = 0; j < ordered.size(); ++j) {
            if (i == j) continue;
            int64_t b = ordered[j];
            size_t covis = ComputeCovisibility(a, b, calib.intrinsics.size());
            if (double(covis) / double(total_a) >= mpCullCovisibilityThresh)
                return a;
        }
    }

    // Criterion 2 — capacity (oldest eligible KF).
    if (frame_poses.size() > mpMaxLocalMapSize) {
        return ordered.front();
    }
    return -1;   // nothing to cull
}
```

#### 7.6.4 Landmark Rehosting Algorithm

When KF `V` is to be culled, every landmark hosted by `V` must be rehosted to the non-culled KF with which `V` has the highest covisibility.

```cpp
void LocalMapper::RehostLandmark(TrackId lm_id, int64_t culled_kf,
                                 int64_t new_host_kf) {
    const Keypoint<double>& old_kpt = lmdb.getLandmark(lm_id);

    // Preserve observations before removeLandmark tears them down.
    Eigen::aligned_map<TimeCamId, Eigen::Vector2d> obs_copy(
        old_kpt.obs.begin(), old_kpt.obs.end());

    // Choose the cam_id of the new host that actually observed this landmark.
    // Prefer camera 0 for determinism when both cameras observed it.
    TimeCamId new_host_tcid(new_host_kf, 0);
    if (obs_copy.count(new_host_tcid) == 0 && calib.intrinsics.size() > 1) {
        bool found_obs = false;
        for(int i = 1; i < calib.intrinsics.size(); i++){
            TimeCamId alt(new_host_kf, i);
            if (obs_copy.count(alt)){
                new_host_tcid = alt;
                found_obs = true;
                break;
            }
        }
        if(!found_obs)
            return;
    } else if (obs_copy.count(new_host_tcid) == 0) {
        // Observation not in any of the frame in the multi cam setup at new host keyframe
        return;
    }

    // Compute new landmark parameters in new host frame.
    const Eigen::Vector2d& pos_2d_new = obs_copy.at(new_host_tcid);
    Eigen::Vector4d pos_3d_new_hom;
    if (!calib.intrinsics[new_host_tcid.cam_id].unproject(pos_2d_new,
                                                          pos_3d_new_hom))
        return;

    // Re-triangulate using an observation from another KF with sufficient
    // baseline. Prefer the KF that hosts the most covisible landmarks with V.
    // Algorithmically simple choice: the existing host — but that's being
    // culled. Fall back to any observing KF with sufficient baseline.
    Sophus::SE3d T_w_newh =
        frame_poses.at(new_host_kf).getPose() *
        calib.T_i_c[new_host_tcid.cam_id];

    Keypoint<double> new_kpt;
    new_kpt.host_kf_id = new_host_tcid;
    bool triangulated = false;
    for (const auto& o : obs_copy) {
        if (o.first == new_host_tcid) continue;
        if (o.first.frame_id == culled_kf) continue;  // don't use the dying KF
        if (!frame_poses.count(o.first.frame_id)) continue;

        Eigen::Vector4d p_o_hom;
        if (!calib.intrinsics[o.first.cam_id].unproject(o.second, p_o_hom))
            continue;
        Sophus::SE3d T_w_o = frame_poses.at(o.first.frame_id).getPose() *
                             calib.T_i_c[o.first.cam_id];
        Sophus::SE3d T_newh_o = T_w_newh.inverse() * T_w_o;
        if (T_newh_o.translation().squaredNorm() <
            config.mapper_min_triangulation_dist *
            config.mapper_min_triangulation_dist)
            continue;
        Eigen::Vector4d p_3d = triangulate(
            pos_3d_new_hom.head<3>(), p_o_hom.head<3>(), T_newh_o);
        if (!p_3d.array().isFinite().all() || p_3d[3] <= 0 || p_3d[3] > 2.0)
            continue;
        new_kpt.direction = StereographicParam<double>::project(p_3d);
        new_kpt.inv_dist  = p_3d[3];
        triangulated = true;
        break;
    }
    if (!triangulated) {
        // No good baseline: drop the landmark entirely.
        lmdb.removeLandmark(lm_id);
        return;
    }

    // Perform the swap.
    lmdb.removeLandmark(lm_id);       // strips obs[old_host] references
    lmdb.addLandmark(lm_id, new_kpt); // new host installed
    for (const auto& o : obs_copy) {
        if (o.first.frame_id == culled_kf) continue;  // excluded per spec
        if(o.first == new_host_tcid) continue;
        KeypointObservation<double> ko;
        ko.kpt_id = lm_id;
        ko.pos    = o.second;
        lmdb.addObservation(o.first, ko);
    }
    // The culled KF is removed as an observation of lm_id implicitly:
    // lmdb.removeFrame(culled_kf) will do it for all remaining landmarks
    // in the parent CullRedundantKeyframes call; we only excluded it from
    // the re-adds here.
}
```

#### 7.6.5 `FindBestRehostKf`

```cpp
int64_t LocalMapper::FindBestRehostKf(int64_t culled_kf, TrackId lm_id,
                                      const std::set<int64_t>& candidates){
    // Find the KF (∈ candidates) with highest covisibility to culled_kf that
    // also observes lm_id.
    int64_t best = -1;
    size_t best_covis = 0;
    if (lm_id < 0 || !lmdb.landmarkExists(lm_id)) return -1;
    const auto& obs_set = lmdb.getLandmark(lm_id).obs;
    for (int64_t c : candidates) {
        if (c == culled_kf) continue;
        // lm_id must be observed by c in at least one camera.
        bool observed = false;
        for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam)
            if (obs_set.count(TimeCamId(c, cam))) { observed = true; break; }
        if (!observed) continue;

        size_t cv = ComputeCovisibility(culled_kf, c, calib.intrinsics.size());
        if (cv > best_covis) { best_covis = cv; best = c; }
    }
    return best;
}
```

#### 7.6.6 `CullRedundantKeyframes` Full Sequence

```cpp
void LocalMapper::CullRedundantKeyframes() {
    std::vector<KeypointId> landmarksToRemove;
    int64_t culled = SelectKeyframeToCull();
    if (culled < 0) {
        // Still clear transient per-step data even if no culling occurred.
        img_data.clear();
        feature_tracks.clear();
        mpLatestKeyframesMatches.clear();
        return;
    }

    // ─── Step 1 — rehost landmarks hosted by culled KF ─────────────────
    std::set<int64_t> candidates;
    for (const auto& kv : frame_poses) candidates.insert(kv.first);

    // Collect landmark IDs BEFORE any removal to avoid iterator invalidation.
    std::vector<TrackId> hosted_lm_ids;
    for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam) {
        auto v = lmdb.getLandmarksForHost(TimeCamId(culled, cam));
        for (const Keypoint<double>* kpt : v) {
            // Need to find the TrackId; lmdb stores Keypoint by TrackId in kpts.
            // getLandmarksForHost returns pointers without TrackId.
            // Traverse getLandmarks() to find matching pointer — O(N);
            // acceptable for bounded local maps.
            for (const auto& lm_kv : lmdb.getLandmarks()) {
                if (&lm_kv.second == kpt) {
                    hosted_lm_ids.push_back(lm_kv.first);
                    break;
                }
            }
        }
    }

    landmarksToRemove.clear();
    for (TrackId lm : hosted_lm_ids) {
        int64_t new_host = FindBestRehostKf(culled, lm, candidates);
        if (new_host < 0) {
            // No valid rehost target → drop the landmark.
            if (lmdb.landmarkExists(lm)){
                landmarksToRemove.push_back(lm);
            }
        } else {
            RehostLandmark(lm, culled, new_host);
        }
    }
    for(KeypointId& id: landmarksToRemove){
        lmdb.removeLandmark(id);
    }

    // ─── Step 2 — remove remaining observations of culled KF ───────────
    lmdb.removeFrame(culled);

    // ─── Step 2b — drop landmarks now below min_num_obs ─────────────────
    // (lmdb.removeFrame cascades for observations, but we additionally
    //  enforce our own min-obs threshold.)
    const Eigen::aligned_unordered_map<KeypointId, Keypoint<Scalar>>& landmarks = lmdb.getLandmarks();
    landmarksToRemove.clear();
    for (Eigen::aligned_unordered_map<KeypointId, Keypoint<Scalar>>::const_iterator it = landmarks.begin(); it != landmarks.end(); ++it) {
        if (lmdb.numObservations(it->first) < 2){
            if (lmdb.landmarkExists(it->first)){
                landmarksToRemove.push_back(it->first);
            }
        }
    }
    for(KeypointId& id: landmarksToRemove){
        lmdb.removeLandmark(id);
    }

    // ─── Step 3 — prune NFR factors referencing culled KF ──────────────
    rel_pose_factors.erase(
        std::remove_if(rel_pose_factors.begin(), rel_pose_factors.end(),
            [culled](const RelPoseFactor& f){
                return f.t_i_ns == culled || f.t_j_ns == culled; }),
        rel_pose_factors.end());
    roll_pitch_factors.erase(
        std::remove_if(roll_pitch_factors.begin(), roll_pitch_factors.end(),
            [culled](const RollPitchFactor& f){
                return f.t_ns == culled; }),
        roll_pitch_factors.end());

    // ─── Step 4 — erase frame_poses and feature_corners ─────────────────
    frame_poses.erase(culled);
    for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam)
        feature_corners.unsafe_erase(TimeCamId(culled, cam));

    // feature_matches: erase any entry involving culled.
    for (auto kv = feature_matches.begin(); kv != feature_matches.end();) {
        if (kv->first.first.frame_id  == culled ||
            kv->first.second.frame_id == culled){
            kv = feature_matches.unsafe_erase(kv);
        }
        else{
            kv++;
        }
    }

    // ─── Step 5 — BoW database ──────────────────────────────────────────
    std::vector<TimeCamId> bow_to_drop;
    for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam)
        bow_to_drop.emplace_back(TimeCamId(culled, cam));
    hash_bow_database->RemoveKeyframes(bow_to_drop);

    // ─── Step 6 — TrackBuilder cleanup ──────────────────────────────────
    mpTrackBuilder.DeleteTracksAfterCulling({culled});

    // ─── Step 7 — clear transient per-step data ─────────────────────────
    img_data.clear();
    feature_tracks.clear();
    mpLatestKeyframesMatches.clear();
}
```

`getLandmarksForHost` returns pointers into the internal `kpts` map of `LandmarkDatabase`. If we were to call `lmdb.removeLandmark` while iterating those pointers, the map could rehash and the pointers would dangle. The algorithm above collects IDs into `hosted_lm_ids` BEFORE any removal, then iterates the ID list. The "find matching pointer → TrackId" inner loop in Step 1 is O(|landmarks|) per hosted landmark, giving O(|landmarks|²) worst-case per culling step. For ≤50 KFs × ~100 lms/KF = 5000 landmarks, worst-case is 25M comparisons — tolerable but not ideal.

### 7.7 Optimisation and Filtering

`optimize`, `filterOutliers`, and the second `optimize` pass are used unchanged from `NfrMapper`/`BundleAdjustmentBase`. Two considerations:

**Gauge freedom:** The local map optimises up to the 7-DoF similarity group (3 translation, 3 rotation, 1 scale), less the constraints from `RollPitchFactor` (fixes 2 rotation DoF from gravity) and the anchoring provided by the VIO marginalisation prior propagated through `RelPoseFactor`. In practice, with IMU-constrained VIO feeding this system, the scale and gravity directions are already observable; only global position and yaw are weakly constrained. For stability, we do not additionally fix a gauge explicitly. The RollPitchFactor on each IMU-backed KF and the RelPoseFactor web are sufficient for bounded drift within one local-map lifetime.

**Linearization invariants:** `NfrMapper::optimize` calls `applyInc` on `PoseStateWithLin` entries. `applyInc` operates relative to the linearisation point. Because `IngestMargData` reconstructs `PoseStateWithLin` (not `linearized`), each optimisation iteration starts with `delta = 0` and a fresh linearisation — there is no stale FEJ corruption.

### 7.8 VIO Pose Feedback

`mpVioPoseUpdateCallback` is invoked with `frame_poses` at the end of every successful mapping iteration. The Controller sets this callback to `vio_estimator_->QueuePoseUpdates`, which copies into `mpPosesToUpdate` under lock (§5.3). The VIO's `measure()` consumes and clears it (§5.4).

---

## 8. Controller Integration

### 8.1 Header Additions

**File:** `include/basalt/controller.h`

```cpp
#include <basalt/vi_estimator/local_mapper.h>

class Controller {
    // ...existing members...

    tbb::concurrent_bounded_queue<basalt::MargData::Ptr> local_map_input_queue_;
    std::shared_ptr<basalt::LocalMapper> local_mapper_;

public:
    void Stop();
};
```

### 8.2 Updated `initialize()`

Inside the `useProducerConsumerArchitecture` branch in `Controller::initialize()`:

```cpp
if (useProducerConsumerArchitecture) {
    opt_flow_ptr_->output_queue       = &vio_estimator_->vision_data_queue;
    vio_estimator_->out_state_queue   = &out_state_queue_;

    // Wire marginalisation output to local mapper input.
    local_map_input_queue_.set_capacity(10);
    vio_estimator_->out_marg_queue    = &local_map_input_queue_;

    // Create and wire local mapper.
    local_mapper_ = std::make_shared<basalt::LocalMapper>(calib_, vio_config_);
    local_mapper_->SetMarginalisationDataInputQueue(&local_map_input_queue_);
    local_mapper_->SetVIOPoseUpdateCallback(
        [this](const auto& poses) {
            vio_estimator_->QueuePoseUpdates(poses);
        });
    local_mapper_->Initialise();

    terminate_processing_thread_ = false;
    pose_processing_thread_ =
        std::thread(&Controller::process_pose_queue_loop, this);
}
```

### 8.3 `Controller::Stop()`

```cpp
void Controller::Stop() {
    // (1) Unblock optical flow — it will propagate nullptr to VIO.
    if (opt_flow_ptr_) opt_flow_ptr_->input_queue.push(nullptr);

    // (2) VIO's processing loop, on receiving nullptr, pushes nullptr to
    //     out_marg_queue (i.e. local_map_input_queue_) and out_state_queue.
    //     This happens automatically inside sqrt_keypoint_vio.cpp line 191-192.

    // (3) Local mapper will receive the nullptr from VIO → exits its loop.
    //     Stop() joins its thread.
    if (local_mapper_) local_mapper_->Stop();

    // (4) Join VIO thread explicitly.
    if (vio_estimator_) {
        vio_estimator_->maybe_join();
        vio_estimator_->drain_input_queues();
    }

    // (5) Join optical flow via destructor.
    opt_flow_ptr_.reset();

    // (6) Drain and join the pose processing thread.
    terminate_processing_thread_ = true;
    out_state_queue_.push(nullptr);
    if(mpUseProducerConsumerArchitecture){
        if (pose_processing_thread_.joinable()){
            pose_processing_thread_.join();
        }
    }
}
```

**Update existing destructor** to call `Stop()` for completeness:
```cpp
Controller::~Controller() {
    Stop();
}
```

### 8.4 Cascade Shutdown Sequence (Summary)

```
    Controller::Stop()
         │
         ▼
 (1) push(nullptr) → opt_flow_ptr_->input_queue
                                │
                                ▼
                OpticalFlow::processingLoop sees nullptr
                                │
                                ▼
       push(nullptr) → opt_flow_ptr_->output_queue (= vio->vision_data_queue)
                                │
                                ▼
                VIO::measure() observes nullptr curr_frame
                                │
                                ▼
   push(nullptr) → vio->out_state_queue
   push(nullptr) → vio->out_marg_queue (= local_map_input_queue_)
                                │
                                ▼
      LocalMapper::MapLocally() pops nullptr → exits loop
                                │
                                ▼
 (3) local_mapper_->Stop() joins LocalMapper thread
 (4) vio_estimator_->maybe_join() joins VIO thread
 (5) opt_flow_ptr_.reset() → OpticalFlow destructor joins its thread
 (6) join pose_processing_thread_
```

---

## 9. Concurrency Summary

| Resource | Writers | Readers | Synchronisation |
|---|---|---|---|
| `local_map_input_queue_`| VIO thread (push) | LocalMapper thread (pop) | TBB concurrent queue (lock-free) |
| `out_state_queue_` | VIO thread | `process_pose_queue_loop` | TBB concurrent queue (lock-free) |
| `VIO::mpPosesToUpdate` | LocalMapper (via callback) | VIO (`measure()`) | `VIO::mpPosesToUpdateMutex` |
| `LocalMapper::frame_poses` | LocalMapper (solely) | LocalMapper | None needed |
| `LocalMapper::lmdb` | LocalMapper (solely) | LocalMapper | None needed |
| `feature_corners` (TBB) | LocalMapper's `parallel_for` (writes) | LocalMapper (sequential reads + `unsafe_erase`) | TBB-internal for writes; sequential-only for erase |
| `feature_matches` (TBB) | LocalMapper's `parallel_for` (writes) | LocalMapper (sequential reads + `unsafe_erase`) | Same as feature_corners |
| `mpLatestKeyframesMatches` (std) | LocalMapper (sequential) | LocalMapper | None needed (single-thread) |
| `hash_bow_database` (`HashBowStl`) | LocalMapper (sequential) | LocalMapper | None needed (single-thread) |
| `mpStopLocalMapping` | Controller (`Stop`) | LocalMapper (`MapLocally`) | `std::atomic<bool>` |
| `mpIsMargDataInputQueueSet` | Controller (`SetMarginalisationDataInputQueue`) | LocalMapper (`MapLocally`) | `std::atomic<bool>` |

**TBB `unsafe_erase` safety contract:** we may call `unsafe_erase` on `feature_corners`/`feature_matches` only between TBB parallel stages, from the LocalMapper thread. The pipeline guarantees this: culling runs after `setup_opt`, which runs strictly after the last parallel stage (`MatchLocal`).

---

## 10. Additional Notes

- **First iteration with an empty `frame_poses`:** `IngestMargData` adds all KFs as new, `mpTrackBuilder.Build` runs (not `AddNewMatches`), `setup_opt` triangulates all landmarks. The very first optimise pass operates on a small window — acceptable.
- **Rank-deficient `MargData`:** `extractNonlinearFactors` returns `false`; no new factors are pushed. Pose addition for new keyframes still go through per §7.1.4.
- **KF present in `kfs_all` but already in `frame_poses`:** classified as existing; pose updated, no new image added, no new detection or matching.
- **Culled KF hosted no landmarks:** `hosted_lm_ids` is empty; the loop in Step 1 of `CullRedundantKeyframes` is a no-op; `lmdb.removeFrame(culled)` still strips any observations this KF made of other landmarks.
- **No valid rehost target (KF isolated):** landmark dropped via `lmdb.removeLandmark` in Step 1's fallback branch; downstream factor pruning still works.
- **All landmarks dropped by `filterOutliers` leave empty `lmdb`:** `optimize` has no visual residuals but still has `RelPoseFactor`/`RollPitchFactor` to anchor poses; it degenerates gracefully.
- **Local mapper crashes before VIO sends sentinel:** `local_map_input_queue_` is bounded (capacity 10); once full, `out_marg_queue->push(m)` blocks the VIO thread. Mitigations: set a watchdog in `Controller`; on observation of `local_mapper_ == nullptr` or exception, call `Stop()` to propagate shutdown. For this PR, the bounded-queue blocking is accepted as a fail-stop behaviour.

---

## 11. Implementation Checklist

Each sub-section in this section is to be committed together. Thus, we will implement the changes required one step at a time, commit the changes, re-analyse the code base, implement any corrections to ensure the code base builds successfully and then move onto the next step of implementation.

### 11.1 Infrastructure

- [ ] `include/basalt/hash_bow/hash_bow.h`: extract `HashBowBase<N>`; mark `HashBow<N>` virtual overrides; add `HashBowStl<N>` with `mpTcidHashIndex` + `RemoveKeyframes`.
- [ ] `include/basalt/utils/union_find.h`: add `AddIndex()`, `InvalidateRoot()`.
- [ ] `include/basalt/utils/tracks.h`: change `map_node_to_index` type; add `mpUFNodeIndexToTrackIdMap`, `mpNextTrackId`, `mpTracksWithCulledObs`, `mpTracksWithLiveObs`; revise `Build`, `Filter`, `Export`; add `AddNewMatches`, `RemoveObservations`, `DeleteTracksAfterCulling`, `CompactTree`.
- [ ] `include/basalt/vi_estimator/nfr_mapper.h`: change `hash_bow_database` type; forward-declare `HashBowBase`.
- [ ] `include/basalt/vi_estimator/landmark_database.h`: add `LandmarkDatabase::getObservationsCountForPair`, `LandmarkDatabase::getNonLandmarkObservationsCountForKeyFrame`

### 11.2 LocalMapper

- [ ] `include/basalt/vi_estimator/local_mapper.h`: full rewrite per §6.1.
- [ ] `src/vi_estimator/local_mapper.cpp`: implement constructor, `Initialise`, `Stop`, queue-setter, callback-setter, `MapLocally`, `IngestMargData`, `CollectNewKeyframesAfterMatching`, `MatchLocal`, `build_tracks`, `setup_opt`, `CullRedundantKeyframes`, helpers (`SelectKeyframeToCull`, `FindBestRehostKf`, `RehostLandmark`, `ComputeCovisibility`).

### 11.3 VIO Extensions

- [ ] `include/basalt/vi_estimator/vio_estimator.h`: add virtual `QueuePoseUpdates`.
- [ ] `include/basalt/vi_estimator/sqrt_keypoint_vio.h`: add `mpPosesToUpdate`, `mpPosesToUpdateMutex`, `kRelinThresholdTrans`, `kRelinThresholdRot`; override `QueuePoseUpdates`.
- [ ] `src/vi_estimator/sqrt_keypoint_vio.cpp`: implement `QueuePoseUpdates`; add pose-application block at top of `measure()`.

### 11.4 Controller

- [ ] `include/basalt/controller.h`: add `local_map_input_queue_`, `local_mapper_`, `Stop()`.
- [ ] `src/controller.cpp`: wire queues in `initialize()`; implement `Stop()`; update destructor to call `Stop()`.

### 11.5 Build System

- [ ] Add `src/vi_estimator/local_mapper.cpp` to the `basalt` library sources in the root `CMakeLists.txt`.

### 11.6 Testing

- [ ] Unit tests for `UnionFind::AddIndex`/`InvalidateRoot`.
- [ ] Unit tests for `TrackBuilder::AddNewMatches` covering: (a) brand-new track, (b) append to existing track, (c) merge of two tracks where one has a landmark, (d) merge where both have landmarks.
- [ ] Unit tests for `TrackBuilder::CompactTree` with partial invalidation.
- [ ] Unit test for `HashBowStl::RemoveKeyframes`.
- [ ] Integration test: feed a recorded EuRoC dataset through `Controller`; verify that `local_mapper_` produces non-empty `frame_poses` with bounded size over runtime.

---

## 12. References

1. Usenko, V., Demmel, N., Schubert, D., Stückler, J., Cremers, D. (2020). *Visual-Inertial Mapping with Non-Linear Factor Recovery*. arXiv:1904.06504v3.
2. Mazuran, M., Burgard, W., Tipaldi, G. D. (2015). *Nonlinear Factor Recovery for Long-Term SLAM*. IJRR.
3. `doc/Marginalisation.md` — VIO linearisation and marginalisation theory.
4. `doc/Mapping.md` — Offline NFR mapper pipeline.
5. `doc/MappingFeatureExtractionMatching.md` — Feature extraction/matching details.
6. `ARCHITECTURE.md` — Basalt system overview.
7. `include/basalt/vi_estimator/nfr_mapper.h`, `src/vi_estimator/nfr_mapper.cpp` — `NfrMapper` reference.
8. `include/basalt/utils/tracks.h`, `include/basalt/utils/union_find.h` — existing `TrackBuilder`/`UnionFind`.
9. `include/basalt/hash_bow/hash_bow.h` — existing `HashBow`.
10. `include/basalt/vi_estimator/ba_base.h`, `include/basalt/vi_estimator/landmark_database.h` — `BundleAdjustmentBase`, `LandmarkDatabase`.
11. `src/controller.cpp`, `include/basalt/controller.h` — `Controller`.
12. `src/vi_estimator/sqrt_keypoint_vio.cpp` — VIO marginalisation and `measure()` loop.
