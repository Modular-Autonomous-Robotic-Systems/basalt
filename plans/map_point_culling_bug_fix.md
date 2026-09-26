# Map Point Culling and Keyframe Culling Defects in `LocalMapper`

## 1. Introduction

The local map exists to substitute for a LiDAR return, so triangulated map points are the deliverable rather than a by-product. The 2026-09-08 SITL recording showed the point cloud absent for the first 64 seconds, then rebuilt and emptied three more times, and subsequent observation shows runs where the count never leaves zero at all. A map that empties is a sensor that stops reporting.

The 2026-09-08 change in [`visualiser_defects.md`](visualiser_defects.md) added a floor, restored recent-keyframe protection and added a `claimed` exclusion to `LocalMapper::SelectKeyframesToCull`. That stopped the map being emptied through wholesale keyframe removal. It did not address three deeper defects, which are the subject of this document.

The first is that keyframe selection is greedy and first-match. It culls the first keyframe that exceeds a covisibility ratio against any single partner, rather than ranking all keyframes and removing those that are genuinely redundant.

The second is that the covisibility measure the selection rests on does not measure covisibility, and the same wrong measure is reused to choose a rehost target, where it is not even a function of the landmark being rehosted.

The third is that landmarks are destroyed during culling faster than the rehost path preserves them, so the database drains even when the keyframe count is healthy.

Instrumentation for all three has been added under `mpVioDebugMode` and is described in section 4.5. The quantitative thresholds in section 4 are to be fixed from a short instrumented run.

## 2. Current Implementation Analysis

### 2.1 What the observations index actually stores

`LandmarkDatabase::observations` is keyed by host first, then by observer.

```
observations : TimeCamId(host) ──▶ TimeCamId(target) ──▶ { landmark ids }

  obs[H][T] = "landmarks HOSTED BY H that are OBSERVED IN T"

  H == T is a legitimate entry, because setup_opt adds an observation for
  every element of the feature track including the host itself
  (local_mapper.cpp, Step B inner loop → addObservation).
```

This asymmetry is the origin of most of what follows. Every derived quantity in the culling code is host-directional, while the quantity the algorithm wants is a symmetric property of a keyframe pair.

### 2.2 `ComputeCovisibility` is not covisibility

```cpp
covisibility += lmdb.getObservationsCountForPair(TimeCamId(a,i), TimeCamId(b,j));
covisibility += lmdb.getObservationsCountForPair(TimeCamId(b,j), TimeCamId(a,i));
```

This is `|hosted by a, seen in b| + |hosted by b, seen in a|`. Landmarks hosted by a third keyframe `c` and seen by both `a` and `b` contribute nothing.

```
        landmark L, hosted by C, observed in A and B

        A ────────┐
                  ├──▶  L  (host C)
        B ────────┘

        true covisibility(A,B) counts L.
        ComputeCovisibility(A,B) does not, because obs[A][B] and obs[B][A]
        are both empty for L.
```

In a sliding local map most landmarks are hosted by the keyframe that first saw them, so for two later keyframes that both observe an older landmark the measure returns zero. The measure systematically undercounts, and it undercounts most for exactly the mature, well-observed keyframes that ought to be candidates for removal.

### 2.3 The redundancy ratio has mismatched numerator and denominator

```cpp
size_t total_a = 0;
for each cam:  for each target in obs[TimeCamId(a,cam)]:  total_a += |kpt_set|
...
if (covis / total_a >= mpCullCovisibilityThresh) cull a
```

`total_a` sums over every target row of `a`, so it is the total number of observations of landmarks hosted by `a`, across all observers. The numerator is a single pair. The ratio is therefore a pair count divided by an all-observers total, which is not a fraction of anything and has no fixed range.

Two consequences follow directly.

A keyframe that hosts nothing has `total_a == 0` and hits `continue`, so it is never culled. Once a keyframe's hosted landmarks have all been rehosted or destroyed it becomes permanently immortal, and it accumulates in `frame_poses` while contributing no map points.

A keyframe hosting a small number of heavily observed landmarks has a small `total_a` and trips the threshold easily, while a keyframe hosting many landmarks each seen by few observers has a large `total_a` and rarely trips it. The criterion is biased against precisely the keyframes that anchor the map.

### 2.4 Selection is greedy and first-match

```
for i in eligible:                       ┌─ stops at the FIRST j above thresh
    if culled_enough: break              │
    if claimed(a): continue              │
    for j in all, j != i: ───────────────┘
        if covis(a,j)/total_a >= thresh:
            cull a;  claim j;  break     ── no comparison against any other
                                            candidate, no ranking
```

Nothing is ranked. The pass removes whichever keyframes happen to appear early in timestamp order and happen to clear the bar against any one partner. The worst keyframe in the map may never be examined, because `max_cull` can be reached first. Two keyframes that are only marginally redundant are treated identically to one that is entirely subsumed.

### 2.5 The cull pipeline and where landmarks leave

```
CullRedundantKeyframes(victim V)
 │
 ├─ Step 1  for each landmark L hosted by V
 │            new_host = FindBestRehostKf(V, L, candidates)
 │            new_host < 0  ──▶ removeLandmark(L)                  [loss A]
 │            else          ──▶ RehostLandmark(L, V, new_host)
 │                                 retriangulation fails ──▶ remove [loss B]
 │                                 obs_added == 0        ──▶ remove [loss C]
 │                                 else re-add obs EXCEPT the new host's own
 │
 ├─ Step 2  lmdb.removeFrame(V)
 │            strips every obs at V, then internally drops any landmark
 │            left with obs.size() < 2                             [loss D]
 │
 ├─ Step 2b sweep: drop every landmark with numObservations() < 2  [loss E]
 │            (duplicates the check removeFrame already performed)
 │
 └─ Steps 3-6  factors, frame_poses.erase, feature_corners, BoW, tracks
```

Losses A through E are unconditional. There is no floor on landmark count, no probation window for young landmarks, and no check that the map remains usable.

### 2.6 `FindBestRehostKf` ranks by the wrong quantity

```cpp
const size_t cv = ComputeCovisibility(culled_kf, c, calib.intrinsics.size());
if (cv > best_covis) { best_covis = cv; best = c; }
```

The score depends only on `culled_kf` and `c`. It does not depend on `lm_id` at all. Every landmark hosted by the same victim therefore receives the same new host, namely the single keyframe with the highest host-directional covisibility against the victim. The only per-landmark filter is the `observed` precondition above it, which merely requires that `c` saw the landmark.

Piling every rehosted landmark onto one keyframe concentrates the map on a single anchor. When that anchor is culled in a later pass, or in a later iteration of the same pass, the whole group is rehosted again or destroyed together.

### 2.7 The candidate set includes doomed keyframes

```cpp
std::set<int64_t> candidates;
for (const auto& kv : frame_poses) candidates.insert(kv.first);
```

`frame_poses.erase(culled)` happens at Step 4 of each iteration, so keyframes culled in earlier iterations are excluded, but keyframes still queued for culling later in the same pass are not. A landmark rehosted onto a doomed keyframe is processed again when that keyframe's turn arrives, and each round trip through `RehostLandmark` costs it observations for the reasons in section 3.

### 2.8 The covisibility matrix is computed twice

`SelectKeyframesToCull` evaluates `ComputeCovisibility` over pairs, then `FindBestRehostKf` evaluates it again over the same keyframes, once per landmark. For a map of `N` keyframes and `L` hosted landmarks the second phase alone is `O(L·N)` calls, each of which walks the observation sub-maps. A matrix built once in the selection phase and passed forward removes this entirely.

## 3. Root Cause of Landmark Database Vacation

Correction, 2026-09-08. This section previously ranked the rehost path's dropped host observation as the primary mechanism and the doomed-candidate routing as second. The instrumented run refutes that ordering. Both defects are real and both are corrected in section 4, but neither is the trigger, and the dropped host observation is never even reached because retriangulation fails first in every single case. The trigger is a knife-edge arithmetic identity described below.

### 3.1 What the run shows

313 mapping cycles, 163 culling events, 2891 rehost attempts.

```
publish        305 of 313 cycles          points=0  lmdb_landmarks=0  host_kfs=0
                 6 cycles                 points=36 lmdb_landmarks=16
                 1 cycle                  points=272
                 1 cycle                  points=8

setup_opt      steady state per cycle     tracks=72  new_ok=26  obs_added=52
                                          lmdb: 0 -> 26

cull           steady state               landmarks: 122 -> 0   in one victim

rehost         2891 of 2891               REMOVED reason=retriangulation_failed
                                          obs=2 in every case

cull-select    37 of 37 selections        best_ratio=0.5 exactly
               35430 keyframe scores      best_ratio=0
```

Landmarks are created every cycle and annihilated before the next. The database is not draining slowly, it is being zeroed once per keyframe.

### 3.2 C0, every track has exactly two observations

`MapLocally` logs `Procesing 1 keyframes` on all 313 cycles, and `CullRedundantKeyframes` clears `feature_tracks` at the end of every cycle, in both the early-return branch and Step 7. A track can therefore only ever span the one new keyframe of the current cycle plus whichever earlier keyframe `MatchLocal` paired it with. It can never accumulate across cycles.

The logs confirm the consequence exactly. Across all 37 Criterion 1 events, `hosted_obs / hosted_lms` is 2.000 without exception, and `obs_added` is twice `new_ok`. Every landmark in the database has precisely the minimum two observations, which is also `LandmarkDatabase::min_num_obs`. The map is permanently at the edge of the cliff.

### 3.3 C1, the redundancy ratio is identically one half

Let `a` host `N` landmarks, each observed by `a` itself and by exactly one partner `b`. Section 2.1 established that `setup_opt` records the host's own observation, so both rows exist.

```
        obs[a][a] = N        the host's own observations
        obs[a][b] = N        the partner's observations
        obs[b][a] = 0        b hosts nothing of a's

        total_a  =  |obs[a][a]| + |obs[a][b]|          =  2N
        covis(a,b) = |obs[a][b]| + |obs[b][a]| = N + 0 =   N

        ratio  =  N / 2N  =  0.5     exactly, for every host, always
```

`hosted_obs=244` against `hosted_lms=122`, and `hosted_obs=24` against `hosted_lms=12`, confirm `total_a = 2N` directly. `best_ratio=0.5` appears in all 37 selections and no other non-zero value appears more than once.

### 3.4 C2, the threshold sits exactly on the knife edge

```cpp
if (static_cast<double>(covis) / static_cast<double>(total_a) >=
    mpCullCovisibilityThresh) {          // mpCullCovisibilityThresh = 0.5
```

The comparison is `>=` and the constant is 0.5, so a ratio of exactly 0.5 fires. Every keyframe that hosts anything is therefore culled the moment it is examined. Had the constant been 0.51, or the comparison strict, Criterion 1 would never have fired at all on this data. The system is balanced on a floating-point equality.

The complementary observation is equally stark. 35430 of the 35468 scored keyframes report `best_ratio=0`, because they host nothing and hit the `total_a == 0` guard from section 2.3. They are immortal. The map saturates at 150 landmark-free keyframes while every keyframe that carries map points is removed on sight.

### 3.5 C3, the victim is always the universal host

`setup_opt` assigns the host from `kv.second.begin()`, the earliest element of the track, and `SelectKeyframesToCull` scans `ordered` in ascending timestamp order. The oldest keyframe in the current track set is therefore both the host of every freshly created landmark and the first candidate the culler examines.

```
[cull] begin, map=101 landmarks=122 victims=1
[cull] kf=493158000000 hosted_lms=122 rehost_attempted=122 landmarks: 122 -> 0
```

All 122 landmarks are hosted by the single victim. There is no partial loss anywhere in the run.

### 3.6 C4, rehosting a two-observation landmark is impossible by construction

This is the mechanism that converts a keyframe removal into total annihilation, and it is arithmetic rather than a tuning failure.

```cpp
for (const auto& o : obs_copy) {
    if (o.first == new_host_tcid) continue;      // skips K
    if (o.first.frame_id == culled_kf) continue; // skips V
    ...                                          // never reached
    triangulated = true;
}
if (!triangulated) { lmdb.removeLandmark(lm_id); return; }
```

```
    obs_copy = { V, K }          exactly two, by C0
    new_host = K                 the only candidate FindBestRehostKf can return

    iteration 1   o = V   ──▶ skipped, o.first.frame_id == culled_kf
    iteration 2   o = K   ──▶ skipped, o.first == new_host_tcid
                              ─────────────────────────────────
    loop body executes zero times   ──▶  triangulated == false
                                    ──▶  removeLandmark, unconditionally
```

A landmark with two observations has no third view to retriangulate against once its old host and its new host are both excluded. The failure rate is not high, it is exactly 100 percent, and the log bears that out with 2891 of 2891 attempts reporting `retriangulation_failed` with `obs=2`.

The dropped host observation identified earlier is downstream of this and never executes, because the function returns at the retriangulation guard before reaching the re-add loop. It remains a real defect that would bite as soon as tracks grow past two observations, and it is corrected in section 4.4.

### 3.7 The composed failure

```
  1 keyframe per cycle, feature_tracks cleared every cycle
                  │
                  ▼
  every landmark has exactly 2 observations              (C0)
                  │
                  ├─────────────────────────────┐
                  ▼                             ▼
  total_a = 2N, covis = N                rehost has zero
  ratio ≡ 0.5                            candidate views
                  │                             │
                  ▼                             │
  threshold 0.5 with >=  fires             (C4) │
                  │                             │
                  ▼                             │
  the universal host is culled  (C3)            │
                  │                             │
                  └──────────────┬──────────────┘
                                 ▼
                    every hosted landmark removed
                                 │
                                 ▼
                          lmdb = 0, points = 0
                                 │
                    next cycle rebuilds ~26 landmarks
                                 │
                                 └──▶ repeat, 313 times
```

Criterion 2 accounts for the remaining 126 of the 163 culling events, firing once the map exceeds `mpMaxLocalMapSize`. It culls `ordered.front()`, the oldest keyframe, which by C3 is again the universal host. The capacity rule reproduces the same annihilation by a different route, so fixing Criterion 1 alone would not have helped.

### 3.8 Two further defects the logs exposed

`getLandmarksForHost` emits a landmark once per observing keyframe rather than once per landmark.

```cpp
for (const auto& [k, obs] : observations.at(tcid))
    for (const auto& v : obs) res.emplace_back(&kpts.at(v));
```

The outer loop walks target rows and the inner loop walks the landmark ids in each row, so a landmark observed by two keyframes is emitted twice. This is why `[publish]` reports `points=36` against `lmdb_landmarks=16`. Every point the GUI draws is duplicated once per observation, which inflates the count and wastes the draw.

Triangulation quality is also poor independently of culling. In steady state `bad_depth` rejects 46 to 59 of 72 to 89 tracks, roughly two thirds, meaning the triangulated inverse distance is non-positive or exceeds 2.0, that is, closer than half a metre. For an aerial survey at altitude the true inverse distance should be small, so these are degenerate solutions from the two-view, short-baseline geometry that C0 forces. Lengthening tracks should improve this on its own, and the remaining rejections should be re-measured afterwards rather than tuned against now.

## 4. Proposed Solution

Correction, 2026-09-08. This section previously opened directly on the covisibility matrix and the ORB-SLAM3 redundancy criterion. Those remain correct and necessary, but the run showed they are not sufficient and not first. Two changes must land ahead of them, because with two-observation landmarks no selection criterion can be safe and no rehost can succeed. The ordering below is by dependency, and P0 is the one that must ship first.

### 4.0 P0, the two-observation ceiling is not yet localised

Correction, 2026-09-08. This subsection previously proposed pruning `feature_tracks` against culled keyframes instead of clearing it in `CullRedundantKeyframes`, on the reading that the wholesale clear capped track length. Reading `LocalMapper::build_tracks` refutes that. It calls `feature_tracks.clear()` itself on both branches and repopulates from `mpTrackBuilder`, so `feature_tracks` is a per-cycle export of the builder's state rather than an accumulator, and the Step 7 clear is harmless. The proposed prune would have been a no-op. It has not been applied.

The builder does accumulate. `TrackBuilder::AddNewMatches` Step G exports every node of each touched track, not only the new ones, and `DeleteTracksAfterCulling` invalidates a track only when it retains no live observation after the cull, which is correct. `MatchLocal` queries each new keyframe against the whole bag-of-words database, so a new keyframe can match many older ones and tracks have room to grow.

The ceiling is therefore somewhere between the matcher and `setup_opt`, and the existing logs cannot separate the candidates. Three remain. Matching may return only one partner per new keyframe, so tracks are born at length two. `Filter(config.mapper_min_track_length)` runs on the first cycle with a configured value of 5, while `AddNewMatches` filters at 2, so the two paths disagree. Or tracks are long but `setup_opt`'s `feature_corners` and `frame_poses` guards drop most elements when adding observations, which the current counters do not cover because they instrument only the triangulation loop.

Instrumentation to settle it has been added to `build_tracks` instead of a speculative fix.

```cpp
    if (mpVioDebugMode) {
        std::map<size_t, size_t> hist;
        for (const auto& kv : feature_tracks) hist[kv.second.size()]++;
        std::cout << "[Local Mapper][tracks] exported=" << feature_tracks.size()
                  << " live_in_builder=" << mpTrackBuilder.TrackCount()
                  << " matches=" << mpLatestKeyframesMatches.size()
                  << " new_kfs=" << mpNewKeyframesForTracking.size()
                  << " corners=" << feature_corners.size() << " lengths {";
        for (const auto& [len, count] : hist)
            std::cout << len << ":" << count << " ";
        std::cout << "}" << std::endl;
    }
```

A length histogram peaking at two with `live_in_builder` close to `exported` puts the ceiling in matching. A histogram with longer tracks, against `obs_added` still equal to twice `new_ok` in `[setup_opt]`, puts it in the observation-add guards. `matches` and `corners` distinguish a matcher that finds one partner from one that finds many.

Everything else in section 4 is independent of this and has been applied, because the logs prove each of those defects directly.

### 4.0b P0b, make rehosting possible for a minimal landmark

Even with longer tracks a landmark can legitimately reach a cull with exactly two observations, and today that is a guaranteed loss. The retriangulation loop must be allowed to use the new host's own observation as one of the two views, since the new host is a real camera with a real pose and excluding it is what leaves the loop empty.

```cpp
    // The new host's observation is a valid second view. Excluding it left a
    // two-observation landmark with no views at all, which is why all 2891
    // rehost attempts in the 2026-09-08 run failed with retriangulation_failed.
    // Only the culled keyframe must be excluded, because its pose is going away.
    for (const auto& o : obs_copy) {
        if (o.first.frame_id == culled_kf) continue;
        if (o.first == new_host_tcid) continue;   // still skipped as the base view
        ...
    }

    // Fall back to the old host's geometry when no second view remains. The
    // landmark was already triangulated in the culled frame, so its world point
    // is known; re-express it in the new host rather than discarding it.
    if (!triangulated) {
        const Sophus::SE3d T_w_oldh = frame_poses.at(culled_kf).getPose() *
                                      calib.T_i_c[old_kpt.host_kf_id.cam_id];
        Eigen::Vector4d p_h = StereographicParam<double>::unproject(old_kpt.direction);
        p_h[3] = old_kpt.inv_dist;
        const Eigen::Vector4d p_w = (T_w_oldh.matrix() * p_h);
        const Eigen::Vector4d p_newh = T_w_newh.inverse().matrix() * p_w;
        if (p_newh.array().isFinite().all() && p_newh[3] > 0 && p_newh[3] <= 2.0) {
            new_kpt.direction = StereographicParam<double>::project(p_newh);
            new_kpt.inv_dist = p_newh[3];
            triangulated = true;
        }
    }
```

Re-expressing the existing estimate in the new host frame is exact up to the pose difference and costs nothing. It is strictly better than destroying a landmark that the bundle adjustment has already refined, and it is the step that makes culling non-destructive.

### 4.0c P0c, remove the knife-edge and the immortality rule

Two one-line consequences of section 3.4 that must not survive even as a fallback.

The comparison `>=` against a threshold of exactly 0.5 fires on the degenerate ratio. Section 4.2 replaces the criterion entirely, so the ratio disappears, but `mpCullCovisibilityThresh` must be commented as superseded rather than retuned, because 0.5 is not a safe value in the new scheme either.

The `total_a == 0` guard makes any keyframe that hosts nothing permanently uncullable, which is why the map saturated at 150 landmark-free keyframes. Under section 4.2 a keyframe observing nothing scores zero and is exempted by `mpMinObservedForCull`, so the capacity rule must be the path that removes it. Criterion 2 already does exactly that, and with P0b it is no longer destructive.

### 4.0d P0d, publish each landmark once

`getLandmarksForHost` emits a landmark once per observing keyframe, so `get_current_points` publishes duplicates and the GUI count is inflated by the observation multiplicity. The fix is local and has no other consumer in the live path.

```cpp
template <class Scalar_>
std::vector<const Keypoint<Scalar_>*>
LandmarkDatabase<Scalar_>::getLandmarksForHost(const TimeCamId& tcid) const {
    std::vector<const Keypoint<Scalar>*> res;
    std::set<KeypointId> seen;   // a landmark appears in one row per observer
    for (const auto& [k, obs] : observations.at(tcid))
        for (const auto& v : obs)
            if (seen.insert(v).second) res.emplace_back(&kpts.at(v));
    return res;
}
```

This touches `LandmarkDatabase`, which section 4.6 previously stated would not be modified. `getLandmarksForHost` has two callers, `get_current_points` in `ba_base.cpp` and the offline `mapper.cpp` display path, and both want distinct landmarks, so the correction is safe for each.


### 4.1 P1, build the covisibility matrix once, symmetrically, over landmark sets

Replace the host-directional count with a set intersection over the landmarks each keyframe observes. Build an observer index once per pass, then intersect.

```cpp
// Landmarks observed by each keyframe, regardless of who hosts them. This is
// the inverse of the host-keyed observations index and is what covisibility
// actually needs.
std::map<int64_t, std::set<KeypointId>> LocalMapper::BuildObservedSets() const {
    std::map<int64_t, std::set<KeypointId>> seen;
    for (const auto& [host, targets] : lmdb.getObservations())
        for (const auto& [target, lms] : targets)
            seen[target.frame_id].insert(lms.begin(), lms.end());
    return seen;
}
```

```cpp
// Symmetric covisibility, |seen(a) ∩ seen(b)|. Populated once per cull pass and
// reused by both keyframe selection and rehost-target selection.
using CovisMatrix = std::map<int64_t, std::map<int64_t, size_t>>;

CovisMatrix LocalMapper::BuildCovisibilityMatrix(
    const std::map<int64_t, std::set<KeypointId>>& seen) const {
    CovisMatrix m;
    for (auto it = seen.begin(); it != seen.end(); ++it)
        for (auto jt = std::next(it); jt != seen.end(); ++jt) {
            std::vector<KeypointId> shared;
            std::set_intersection(it->second.begin(), it->second.end(),
                                  jt->second.begin(), jt->second.end(),
                                  std::back_inserter(shared));
            if (shared.empty()) continue;
            m[it->first][jt->first] = shared.size();
            m[jt->first][it->first] = shared.size();
        }
    return m;
}
```

The user's requested `std::map<int64_t, std::pair<int64_t, int>> covisibility`, mapping each keyframe to its best partner and that partner's count, is the row-wise maximum of this matrix and is derived from it rather than replacing it. The full matrix is retained because `FindBestRehostKf` needs arbitrary rows, not just the maximum.

### 4.2 P2, rank by observer redundancy rather than by a single pair

Adopt the ORB-SLAM3 criterion, which asks how well each landmark a keyframe sees is covered elsewhere, rather than how much one partner overlaps it.

```
ORB-SLAM3 KeyFrameCulling, LocalMapping.cc:964-1022

  for each map point observed by KF:
      nMPs++
      if point has more than thObs (=3) observations at compatible scale:
          nRedundantObservations++
  if nRedundantObservations > redundant_th * nMPs:   (0.9 monocular)
      SetBadFlag()
```

The transcription for this codebase, with no scale pyramid, is a plain observer count.

```cpp
// Fraction of the landmarks seen by kf that are also seen by at least
// mpMinRedundantObservers other keyframes. This is the ORB-SLAM3 criterion,
// evaluated against `alive` so a victim already chosen this pass no longer
// counts as an observer.
double LocalMapper::RedundancyScore(int64_t kf,
                                    const std::map<int64_t, std::set<KeypointId>>& seen,
                                    const std::set<int64_t>& alive,
                                    size_t& nObservedOut) const {
    const auto it = seen.find(kf);
    if (it == seen.end() || it->second.empty()) { nObservedOut = 0; return 0.0; }
    size_t redundant = 0;
    for (const KeypointId lm : it->second) {
        size_t others = 0;
        for (const int64_t k : alive) {
            if (k == kf) continue;
            const auto jt = seen.find(k);
            if (jt != seen.end() && jt->second.count(lm)) ++others;
            if (others >= mpMinRedundantObservers) break;
        }
        if (others >= mpMinRedundantObservers) ++redundant;
    }
    nObservedOut = it->second.size();
    return static_cast<double>(redundant) / static_cast<double>(nObservedOut);
}
```

Selection becomes a ranked, iterative removal rather than a greedy first-match scan.

```cpp
bool LocalMapper::SelectKeyframesToCull(std::vector<int64_t>& keyframesToCull) {
    if (frame_poses.size() <= mpMinLocalMapSize) return false;

    const auto seen = BuildObservedSets();
    mpCovisMatrix = BuildCovisibilityMatrix(seen);   // cached for the rehost phase

    std::vector<int64_t> ordered;
    for (const auto& kv : frame_poses) ordered.push_back(kv.first);
    std::sort(ordered.begin(), ordered.end());

    const size_t keep_recent =
        std::min<size_t>(mpNewKeyframesForTracking.size(), ordered.size());
    const size_t eligible_end = ordered.size() - keep_recent;
    const size_t max_cull =
        std::min(frame_poses.size() - mpMinLocalMapSize, mpMaxCullPerPass);

    std::set<int64_t> alive(ordered.begin(), ordered.end());

    // Remove one keyframe at a time, always the most redundant survivor, and
    // re-score after each removal. Re-scoring is what stops both members of a
    // mutually covisible pair being culled together, because removing the first
    // drops the observer count of every landmark they shared and the second's
    // score falls below the threshold on the next round.
    while (keyframesToCull.size() < max_cull) {
        int64_t worst = -1;
        double worstScore = mpCullRedundancyThresh;
        for (size_t i = 0; i < eligible_end; ++i) {
            const int64_t a = ordered[i];
            if (!alive.count(a)) continue;
            size_t nObserved = 0;
            const double score = RedundancyScore(a, seen, alive, nObserved);
            if (nObserved < mpMinObservedForCull) continue;   // anchors are exempt
            if (score > worstScore) { worstScore = score; worst = a; }
        }
        if (worst < 0) break;
        keyframesToCull.push_back(worst);
        alive.erase(worst);
    }
    return !keyframesToCull.empty();
}
```

### 4.3 P3, the additional conservatism check, and why it is required

The question raised was whether a further guard is needed against culling both members of a highly covisible pair. It is, and the reason is structural rather than incidental.

ORB-SLAM3 is safe from this without an explicit guard because `SetBadFlag()` takes effect immediately and `pMP->Observations()` is re-read for each keyframe as the loop proceeds. Culling the first member of a pair lowers the observation count of every landmark they shared, so the second member's redundancy falls before it is tested. The protection is a side effect of evaluating against live state.

A batch design that scores every keyframe against a snapshot loses that protection entirely.

```
  snapshot scoring, both members pass, both culled
      A and B are near-duplicates, and each landmark they share also has
      2 other observers.
      score(A) = 1.0  ✓ cull        score(B) = 1.0  ✓ cull
      after both are removed, the shared landmarks have 2 observers,
      not the 3 the test assumed, and the next removeFrame kills them.

  incremental scoring, the second member is spared
      round 1:  score(A) = 1.0  ✓ cull,  alive.erase(A)
      round 2:  score(B) recomputed against alive without A
                the shared landmarks now have 2 other observers, below
                mpMinRedundantObservers = 3, so score(B) = 0.0  ✗ keep
```

The `while` loop with `alive.erase(worst)` and re-scoring in section 4.2 is that guard. It reproduces ORB-SLAM3's live-state semantics inside a batch selection, and it makes the separate `claimed` set from the 2026-09-08 change unnecessary, since a partner that is still needed as an observer will keep its own score below the threshold automatically.

Two further guards, both cheap.

`mpMinObservedForCull` exempts a keyframe that observes very few landmarks, so a sparsely connected anchor is never removed on a score computed from a handful of points.

`mpMaxCullPerPass` bounds the edit so no single pass can restructure the map, which bounds the blast radius of any residual scoring error.

### 4.4 P4, stop the rehost path leaking landmarks

Four corrections. The 2026-09-08 run shows none of them executes today, because `RehostLandmark` returns at the retriangulation guard in 100 percent of cases, so these matter only once P0 and P0b let the function reach its second half. They are not optional for that reason. They are the defects that would replace the current failure with a slower one.

The new host's own observation must be re-added. This is the difference between a three-observer landmark surviving a rehost and dying.

```cpp
    for (const auto& o : obs_copy) {
        if (o.first.frame_id == culled_kf) continue;
        // The new host observes the landmark too. setup_opt records obs[H][H]
        // for every normally created landmark, so skipping it here left a
        // rehosted landmark one observation short and below min_num_obs.
        KeypointObservation<double> ko;
        ko.kpt_id = lm_id;
        ko.pos = o.second;
        lmdb.addObservation(o.first, ko);
        ++obs_added;
    }
```

Doomed keyframes must be excluded from the candidate set, at the call site.

```cpp
        std::set<int64_t> candidates;
        const std::set<int64_t> doomed(keyframesToCull.begin(),
                                       keyframesToCull.end());
        for (const auto& kv : frame_poses)
            if (!doomed.count(kv.first)) candidates.insert(kv.first);
```

The rehost target must be chosen per landmark, from the keyframes that actually observe it, using the cached matrix rather than a second traversal.

```cpp
int64_t LocalMapper::FindBestRehostKf(int64_t culled_kf, TrackId lm_id,
                                      const std::set<int64_t>& candidates) {
    if (lm_id < 0 || !lmdb.landmarkExists(lm_id)) return -1;
    const auto& obs_set = lmdb.getLandmark(lm_id).obs;

    int64_t best = -1;
    size_t best_covis = 0;
    for (const int64_t c : candidates) {
        if (c == culled_kf) continue;
        bool observed = false;
        for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam)
            if (obs_set.count(TimeCamId(c, cam))) { observed = true; break; }
        if (!observed) continue;

        // Rank by how well c is connected to the rest of the map, read from
        // the matrix built during selection. The previous score depended only
        // on culled_kf and c, so every landmark from one victim was routed to
        // the same host and the map collapsed onto a single anchor.
        size_t covis = 0;
        const auto row = mpCovisMatrix.find(c);
        if (row != mpCovisMatrix.end())
            for (const auto& [other, count] : row->second)
                if (candidates.count(other)) covis += count;

        if (covis > best_covis) { best_covis = covis; best = c; }
    }
    return best;
}
```

Step 2b becomes redundant once `removeFrame` is trusted to enforce `min_num_obs`, and it should be deleted rather than left duplicating the check mid-pass. Whether a probation window in the style of ORB-SLAM3's `MapPointCulling` is additionally required is the open question the logs will settle, since it is only warranted if landmarks are dying young rather than dying at rehost.

New bounds beside the existing ones in `include/basalt/vi_estimator/local_mapper.h`.

```cpp
    size_t mpMinRedundantObservers = 3;    // ORB-SLAM3 thObs
    double mpCullRedundancyThresh = 0.9;   // ORB-SLAM3 redundant_th, monocular
    size_t mpMinObservedForCull = 30;      // anchors below this are exempt
    size_t mpMaxCullPerPass = 2;           // bound the blast radius
```

`mpCullCovisibilityThresh` becomes unused by selection and is left in place, commented as superseded, because the ratio it governed is not comparable with the new redundancy fraction and a reader must not transplant the old 0.5 into `mpCullRedundancyThresh`.

### 4.5 P5, instrumentation, applied

All of the following are live under `mpVioDebugMode` and precede the changes above, so a run on the current code localises the failure before any behaviour is altered.

| Tag | Emitted from | Answers |
|---|---|---|
| `[setup_opt]` | `LocalMapper::setup_opt` | Whether landmarks are created at all, and which of the six rejection paths dominates when they are not |
| `[cull-select]` | `SelectKeyframesToCull` | Map size, eligibility, the victims chosen, and per-keyframe `hosted_obs` and `best_ratio` with the partner that produced it |
| `[rehost]` | `FindBestRehostKf` call site and `RehostLandmark` | Per landmark, the victim, the chosen host, observations before and after, and which of five exit paths was taken, including rehosting onto a keyframe queued for culling |
| `[cull]` | `CullRedundantKeyframes` | Landmark count before and after each victim, and across the whole pass |
| `[publish]` | the `out_vis_queue` block | Points and keyframes actually shipped, against `lmdb.numLandmarks()` and `getHostKfs().size()` |

The `[publish]` line is the join between this document and the GUI. A `points=0` with `lmdb_landmarks=0` is a creation or attrition failure, whereas `points=0` with a non-zero `lmdb_landmarks` would instead indicate a fault in `get_current_points` or its host lookup.

### 4.6 Backwards compatibility

`SelectKeyframesToCull`, `FindBestRehostKf`, `RehostLandmark` and the `feature_tracks` handling are non-virtual members of `LocalMapper` and shadow nothing in `NfrMapper`, so the offline paths in `src/mapper.cpp` and `src/mapper_sim.cpp` are untouched. `BuildObservedSets`, `BuildCovisibilityMatrix`, `RedundancyScore` and `PruneTracksForCulledKeyframes` are additions.

`LandmarkDatabase::getLandmarksForHost` is modified by P0d. Correction, 2026-09-08. This paragraph previously stated that `LandmarkDatabase` would not be modified at all, which was written before the log exposed the duplicate emission. Its two callers, `BundleAdjustmentBase::get_current_points` and the offline display path in `src/mapper.cpp`, both want distinct landmarks, so de-duplication is the correct behaviour for each and neither changes semantically beyond losing the duplicates.

The change is behavioural for every `basalt_slam` run. Trajectories and maps produced before it are not comparable with those after it.

### 4.7 What the logs settled, what was applied, and what remains

Settled by the 2026-09-08 instrumented run.

The attrition is not gradual, and neither R1 nor R2 as originally ranked is the trigger. The trigger is the two-observation ceiling, which makes the redundancy ratio identically 0.5 against a threshold of exactly 0.5 compared with `>=`, and simultaneously makes rehosting impossible. Section 3 carries the arithmetic and the counts.

The ORB-SLAM3 probation window is not needed. Probation guards against young points being judged too early, whereas here no point is ever judged on its own merits at all, since every point dies with its host.

Applied, with verification.

The selection criterion, `FindBestRehostKf`, `RehostLandmark`, the candidate exclusion, the duplicated Step 2b sweep and `getLandmarksForHost` are all changed as described above. The selection logic was transcribed onto stub types and compiled with `g++ -std=c++17 -Wall -Wextra`. Twelve identical keyframes yield two culls against a cap of two. Four keyframes sharing ten landmarks with `mpMinRedundantObservers` at three yield exactly one cull, confirming the pair guard of section 4.3 holds, because removing the first drops the survivors' observer count to two. A keyframe observing fewer landmarks than `mpMinObservedForCull` is exempt. A map at the floor culls nothing.

The reprojection fallback of P0b was checked numerically over 2000 random pose pairs. The reconstructed world point matches the original to 7.3e-14 metres, and the inverse distance is genuinely recomputed rather than passed through. That check caught a real error. The first draft carried the transformed homogeneous point straight into `Keypoint`, but `BundleAdjustmentBase::triangulate` documents and enforces a unit-length direction head with the inverse distance in the last component, at `ba_base.h:95-128`, and an SE3 product does not preserve that. Rescaling the homogeneous point by the norm of its head restores the invariant and leaves the 3D position unchanged.

Not applied, deliberately.

The `feature_tracks` prune originally proposed as P0 is a no-op and was not applied, for the reason in section 4.0. The two-observation ceiling remains unlocalised and is instrumented rather than guessed at.

`mpMinRedundantObservers` at 3 and `mpCullRedundancyThresh` at 0.9 are the ORB-SLAM3 values and cannot be evaluated against the current data, because no landmark in the run ever had more than two observers. A histogram of observers per landmark is now emitted by `[cull-select]` on every pass and is the distribution they must be re-derived from.

The two-thirds `bad_depth` rejection rate should be re-measured after the ceiling is found, since the short two-view baseline it forces is the most likely cause.

`ComputeCovisibility` is now unreferenced outside a comment. It is left defined because it is part of the class's public surface and removing it is out of scope here.

Next step is a second instrumented run. `[tracks]` localises the ceiling, `[cull-select] observers_per_landmark` fixes the two constants, and `[rehost]` should now show `REPROJECTED` where it previously showed 2891 identical failures.

## 5. Second Instrumented Run, 2026-09-08

### 5.1 What the first round of fixes achieved

493 cycles across two takeoff and landing sequences. The database is no longer zeroed once per keyframe.

```
                              first run          second run
  publish points              0 in 305/313       404 to 738 in steady state
  landmarks in lmdb           0                  peak 744
  track lengths               all exactly 2      2 to 17, histogram spread
  rehost outcomes             2891/2891 removed  113 REHOSTED, 100 REPROJECTED,
                                                 85 REHOSTED_FRAGILE, 31 removed
  landmarks lost per cull     all of them        0 in 170 of 194 culls
```

The two total collapses, at cycles 284 and 469, are node restarts between the two flights and not a defect. The thread-exit block in the log immediately precedes each.

The track ceiling is gone. `[tracks]` reports lengths up to 17 with `live_in_builder` reaching 1168, so the ceiling was never in the track store, consistent with section 4.0. It was in the rehost path destroying every landmark before a track could ever be observed to be long.

### 5.2 The remaining shortfall

A 150 keyframe local map holding 450 to 750 landmarks is far below what the exported tracks support. `live_in_builder` reaches 1168 while `lmdb` sits near 500, so roughly half the tracks never hold a landmark at any moment.

The balance per cycle is the diagnosis.

```
  new_ok per cycle          31 to 61 created
  loss between cycles       150 to 250 removed at cycles 477 to 484
  net                       flat, an equilibrium well below capacity
```

Culling is no longer the sink, since 170 of 194 culls lose nothing. The loss falls between the end of one `setup_opt` and the start of the next, and the only landmark-destroying step in that interval is `filterOutliers`.

### 5.3 R5, filterOutliers demands four observations from a map that has three

```cpp
filterOutliers(mpFilterOutlierThreshold, 4);        // local_mapper.cpp
...
if (num_obs - num_outliers < min_num_obs) remove = true;   // ba_base.cpp:287
```

The measured observer distribution, emitted by `[cull-select]` at the peak of the run.

```
  observers_per_landmark {2:121  3:261  4:43  5:35  6:71  7:22  8:17  9:25
                          10:11 11:17 12:21 13:10 14:13 15:22 16:8 17:2 18:45}

  382 of 744 landmarks, 51 percent, have two or three observers.
  Every one of them is deleted outright by a single outlier observation.
```

The 4 is inherited from `src/mapper.cpp:676`, the offline batch mapper, where a landmark accumulates observations from the whole sequence before any filtering. A live local map creates a landmark with two observations and grows it, so the threshold is applied to a population that has not had the chance to reach it. `LandmarkDatabase::min_num_obs` is 2, so 4 is also inconsistent with what the database itself considers viable.

`computeError` compounds this. A residual on the host's own observation is recorded as the sentinel `-2` at `ba_base.cpp:190`, and `filterOutliers` removes the landmark unconditionally on seeing it, regardless of how many other observations are healthy.

### 5.4 R6, the triangulation pair is chosen worst-first

`setup_opt` iterated candidate second views in timestamp order and broke on the first that produced a valid depth. Track observations are keyed by timestamp, so the first candidate is the nearest in time and therefore the shortest baseline that clears `mapper_min_triangulation_dist`, which at 0.07 m is negligible against a survey altitude. The best-conditioned pair available was routinely discarded in favour of the worst.

Simulated over 4000 samples with survey geometry, namely a point at 20 to 80 m, eight later keyframes at 1.5 m spacing and one pixel of bearing noise at `fx=320`.

```
  depth error, first adequate baseline    median 2.888 m    p90 11.950 m
  depth error, largest baseline first     median 0.415 m    p90  1.537 m
                                          median error down 85.6 percent
```

Success rate at creation is identical, because the loop already tried every candidate and only `continue`d on failure. The ordering therefore does not change how many landmarks are born, it changes how good they are. That is precisely what feeds R5. A landmark carrying nearly three metres of depth error reprojects outside the outlier threshold, is flagged, and with two or three observations is then deleted rather than trimmed.

R5 and R6 are one loop. Poor geometry manufactures outliers, and an over-strict survival rule converts each outlier into a deletion.

### 5.5 Changes applied

`mpFilterMinObs`, defaulting to 2, replaces the hard-coded 4 at the `filterOutliers` call site. Two matches `LandmarkDatabase::min_num_obs`, so a landmark that the database considers viable is no longer destroyed by the filter.

`setup_opt` now sorts candidate second views by descending baseline before trying any of them, so the best-conditioned pair is attempted first. The accounting for short baselines is retained rather than short-circuited, since the sort makes it a prefix property.

A parallax gate is added ahead of the solve, following ORB-SLAM3's `cosParallaxRays` test at `LocalMapping.cc:594`. Near-parallel bearing rays make the linear system ill-conditioned, and it then returns a point behind the camera or at absurd depth rather than failing cleanly. `mpMaxCosParallax` defaults to 0.9998, the ORB-SLAM3 non-inertial value, which at a 12 m baseline admits points well beyond survey altitude.

`mpMaxInvDist` replaces the literal 2.0 depth cap, so the rejection is named and tunable.

The `bad_depth` counter is split into `behind`, `too_close` and `nan`, and `low_parallax` is counted separately. The first run could not distinguish a solution landing behind the camera from one landing implausibly close, and those have different remedies.

`[filter]` logs the landmark count either side of `filterOutliers`, which is the measurement that was missing. The loss was inferred from the gap between consecutive `setup_opt` lines rather than observed directly.

### 5.6 What the next run must show

`[filter]` should show a much smaller drop, and the split `bad_depth` counters should show whether the residual rejections are behind-camera, which indicates conditioning, or too-close, which indicates a wrong cap.

If `lmdb` still plateaus well below `live_in_builder` with `[filter]` losing little, the remaining sink is elsewhere and the next candidate is the `-2` host-observation sentinel, which would warrant trimming the offending observation rather than deleting the landmark.

`observers_per_landmark` should shift right as landmarks survive long enough to accumulate observations. Once it does, `mpMinRedundantObservers` and `mpCullRedundancyThresh` become measurable for the first time, since they still cannot be evaluated against a population that never exceeded two observers in the first run and peaks at three in the second.

## 6. Third Instrumented Run, 2026-09-10, the tracking failure

### 6.1 What the run shows

66 mapping cycles, one keyframe each, across a takeoff and landing. No culling event fired at all, so every landmark loss in this run happened without a single keyframe being removed. That alone relocates the problem, because the document up to section 5 assumed culling was the sink.

Track building stops on the fifth cycle and never restarts.

```
  cyc   bow_pairs   kf_matches   exported   live_in_builder
    1        0            0           0            0
    2        1            1          72           72
    3        2            1          61          109
    4        3            1          48          142
    5        3            0           0          142      ◀── stops here
    6        1            0           0          142
    7        2            0           0          142
    …        0            0           0          142      for 59 more cycles
```

`live_in_builder` freezes at 142 while `feature_corners` grows to 66, so keypoints are still being detected and added to the bag-of-words database on every cycle. Nothing is wrong with ingestion. What stops is the supply of matches.

Two separate failures produce that, and they must not be conflated.

Bag-of-words retrieval returns no candidate pair at all on 57 of the 66 cycles. `Matching 0 image pairs` is logged 57 times, against 5 cycles with one pair, 2 with two and 2 with three.

On the 9 cycles where retrieval did return candidates, descriptor matching and RANSAC produced no surviving inliers from cycle 5 onward, so `feature_matches` gained no entry and `mpLatestKeyframesMatches` was empty.

### 6.2 R7, local matching depends entirely on place recognition

`LocalMapper::MatchLocal` builds its candidate list from one source only.

```cpp
hash_bow_database->querry_database(kd.bow_vector,
                                   config.mapper_num_frames_to_match,
                                   results, &tcid.frame_id);
for (const auto& otcid_score : results)
    if (otcid_score.second > config.mapper_frames_to_match_threshold)
        ids_to_match.emplace_back(...);
```

A bag-of-words index is a place-recognition structure. It answers "which keyframe anywhere in the map looks like this one", which is the question loop closure asks. A local mapper asks a different question, namely "which keyframes are adjacent to this one", and it already knows the answer from the timestamps. Making the adjacent pair conditional on a retrieval score means the local map is rebuilt only when place recognition happens to fire.

The hash makes that unreliable by construction. `compute_hash` selects `mapper_bow_num_bits`, 16, fixed bit positions from the 256-bit descriptor, and two descriptors share a bucket only if all 16 agree exactly. With a per-bit disagreement rate of a few percent between two views of the same point, a large fraction of true correspondences fall into different buckets, and the surviving overlap must still clear `mapper_frames_to_match_threshold`. Over homogeneous terrain seen from altitude that margin disappears.

Every one of these parameters is byte-identical to basalt's defaults in `src/utils/vio_config.cpp:94-101`, which were set for indoor handheld sequences. Nothing has been mistuned. The mechanism is simply being asked to do a job it was not built for.

ORB-SLAM3 does not make this mistake. `LocalMapping::CreateNewMapPoints` takes its neighbours from `GetBestCovisibilityKeyFrames`, and in the inertial case walks the `mPrevKF` chain explicitly to guarantee temporal neighbours are present, at `LocalMapping.cc:409-419`. Retrieval is reserved for `LoopClosing`.

### 6.3 R8, the first track filter uses the offline batch length

```cpp
mpTrackBuilder.Build(tbb_matches);
mpTrackBuilder.Filter(config.mapper_min_track_length);   // 5
...
mpTrackBuilder.AddNewMatches(...);                       // Filter(2) inside
```

A track produced by pairwise matching spans exactly two images on the cycle it is born. `TrackBuilder::Filter` invalidates every root whose distinct `TimeCamId` count is below the threshold, so filtering the first `Build` at 5 destroys every track it just created, while every later cycle filters at 2 through `AddNewMatches`. The two paths disagree about what a valid track is.

This did not fire in the 2026-09-10 run, and only by accident. The first cycle had no matches at all, so `Build` ran on an empty match set, `Filter(5)` had nothing to invalidate, and `mpIsTrackBuilderInitialised` was set. Any run whose first mapping cycle carries matches loses its entire initial track set.

### 6.4 R9, the descriptor matcher ignores its configuration

```cpp
matchDescriptors(f1.corner_descriptors, f2.corner_descriptors,
                 md.matches, 70, 1.2);
```

`config.mapper_max_hamming_distance` and `config.mapper_second_best_test_ratio` are read from the configuration file and then not used. The literals happen to equal the current defaults, so behaviour is unchanged today, but tuning either value has no effect, which would have silently defeated any attempt to loosen matching in response to this investigation.

### 6.5 The remaining observations, adjudicated

On the union-by-rank tie break ignoring timestamps. The concern does not reach landmark hosting. `setup_opt` takes the host from `kv.second.begin()`, and an exported track is an ordered set keyed by `TimeCamId`, so the host is always the earliest observation regardless of which node the disjoint-set structure chose as root. The root determines only the `TrackId`, and the consequences of that are the next paragraph.

On a track merge invalidating a landmark index. The mechanism is sound. The losing root's `TrackId` is placed in `retired_ids`, `setup_opt` Step A removes the corresponding landmark, and Step G exports the merged track with every node it now owns, so the observation-add loop re-attaches the loser's observations to the surviving landmark. Nothing is orphaned.

There is nonetheless a real defect adjacent to it, not previously recorded. `setup_opt` creates geometry only when `!lmdb.landmarkExists(kv.first)`, so a surviving landmark whose track has just absorbed a much longer baseline keeps the geometry triangulated from its original short one. The merge improves the observation set without ever improving the estimate. This is left unfixed for now because it cannot bite while tracks are not being built at all, and it should be measured once they are.

On `removeLandmark` emptying an observations row. `removeLandmarkHelper` at `src/vi_estimator/landmark_database.cpp:217-246` erases each target entry as its landmark set empties and erases the host row when that empties, so the index stays consistent and `getHostKfs` never returns a stale key. A keyframe that hosted only the removed landmark does drop out of `observations` entirely, which is correct, and the downstream consequence is that `BuildObservedSets` will not list it, `RedundancyScore` returns zero for it, and it becomes exempt from redundancy culling so that only the capacity rule can remove it. That is the documented behaviour of section 4.2 rather than a defect.

On the redundancy criterion. The run supports it. Zero culling events fired across 66 cycles, against 163 in the first run, so the replacement is no longer removing keyframes indiscriminately.

### 6.6 Changes applied

Temporal neighbours are now matched unconditionally. Every new keyframe is paired with the `mpLocalMatchNeighbours` most recent keyframes older than it, in addition to whatever retrieval returns, with the two sets deduplicated so a retrieval hit that is also a neighbour is not matched twice. Retrieval hits outside the neighbour window are preserved, so loop closure to a genuinely revisited place still works.

```cpp
    std::set<std::pair<size_t, size_t>> already;
    for (const auto& m : ids_to_match) already.emplace(m.i, m.j);

    std::vector<TimeCamId> vOlder;
    for (const auto& tcid : keys)
        if (mpNewKeyframesForTracking.count(tcid.frame_id) == 0)
            vOlder.push_back(tcid);
    std::sort(vOlder.begin(), vOlder.end(),
              [](const TimeCamId& a, const TimeCamId& b) {
                  return a.frame_id > b.frame_id;  // newest first
              });

    for (size_t i = 0; i < keys.size(); ++i) {
        if (mpNewKeyframesForTracking.count(keys[i].frame_id) == 0) continue;
        size_t taken = 0;
        for (const TimeCamId& other : vOlder) {
            if (taken >= mpLocalMatchNeighbours) break;
            if (other.frame_id >= keys[i].frame_id) continue;
            const size_t j = id_to_key_idx.at(other);
            ++taken;
            if (already.count({i, j})) continue;
            ids_to_match.emplace_back(match_pair{i, j, 0.0});
        }
    }
```

`mpLocalMatchNeighbours` defaults to 5, beside the other bounds in `include/basalt/vi_estimator/local_mapper.h`.

The initial `Build` now filters at 2 rather than `config.mapper_min_track_length`, matching what `AddNewMatches` does on every subsequent cycle. The superseded line is commented in place.

`matchDescriptors` is passed `config.mapper_max_hamming_distance` and `config.mapper_second_best_test_ratio` instead of literals.

A retrieval hit that is no longer present in `feature_corners` is now skipped rather than being looked up with `id_to_key_idx.at`, which would have thrown out of a TBB worker thread.

Two instrumentation tags are added. `[bow]` reports, per new keyframe, the keypoint count, the descriptor count, the bag-of-words bucket count, the number of database hits, the best score and how many cleared the threshold, which separates a keyframe with no features from one whose features simply do not retrieve. `[match]` reports, per candidate pair, the descriptor match count against `mapper_min_matches`, the RANSAC inlier count, and whether the pair was stored.

### 6.7 Verification

The neighbour selection was transcribed onto stub types and compiled with `g++ -std=c++17 -Wall -Wextra`. With no retrieval hits it supplies exactly five pairs and they are the five most recent keyframes older than the new one. With a retrieval hit that is also a neighbour it produces no duplicate. With a far-past retrieval hit it keeps that hit alongside the five neighbours, so loop closure survives. With fewer older keyframes than the limit it supplies all of them and does not fail.

### 6.8 What the next run must show

`[bow]` answers the question this run could not. A `keypoints` count near `mapper_detection_num_points` with `db_hits` non-zero but `best_score` below the threshold means retrieval is working and merely too strict, and the threshold is then the thing to tune. A `keypoints` count near zero means the images are too dark or too smooth for `detectKeypointsMapping` and the problem is upstream of the mapper entirely, which the very dark camera panel in the earlier screen recording makes plausible.

`[match]` says whether the neighbour pairs now being fed to the matcher survive. A `descriptor_matches` count below `mapper_min_matches` points at the matcher, while a healthy match count collapsing to zero `ransac_inliers` points at `mapper_ransac_threshold`, which at 5e-5 corresponds to roughly half a degree of bearing error.

`[tracks]` should show `exported` non-zero on every cycle rather than three, and `live_in_builder` climbing rather than frozen at 142. Only once that holds do the landmark counts, the observer histogram and therefore `mpMinRedundantObservers` and `mpCullRedundancyThresh` become measurable at all.

## 7. Incremental Covisibility Maintenance in `LandmarkDatabase`

### 7.1 Motivation, a full rebuild is paid on every keyframe

Sections 4.1 and 4.2 replaced a wrong covisibility measure with a correct one, and correctness was the only concern at the time. The correct measure is however rebuilt from nothing on every entry to `SelectKeyframesToCull`, which runs once per mapping cycle, and the cost of that rebuild grows quadratically in the size of the local map.

The repository already carries the measurement. `culling_time.log` holds 1995 timed calls to `CullRedundantKeyframes` from a SITL run.

```
  n=1995   min 1e-06s   median 0.00818s   p90 0.0199s   p99 0.0289s   max 0.035s

  chronological decile means, seconds
    0.00065  0.0147  0.0152  0.0148  0.0057  0.0067  0.0082  0.0100  0.0168  0.0051
```

The first decile runs in well under a millisecond because the map is still small, and the cost settles into the ten to fifteen millisecond band as the map fills. Most passes in the 2026-09-10 run culled nothing at all, so in a no-cull pass essentially the whole of that time is `BuildObservedSets`, `BuildCovisibilityMatrix` and the `RedundancyScore` loop, with no rehosting work to attribute it to. Eight milliseconds per keyframe is not fatal, but it is a quadratic term sitting in the path of a thread that must keep pace with keyframe insertion, and it is entirely avoidable.

The deeper objection is structural rather than numerical. Covisibility is a property of the observation set, the observation set is mutated one observation at a time through three methods of `LandmarkDatabase`, and every one of those mutations already performs index bookkeeping. Recomputing a derived quantity from scratch when the primitive that derives it is edited in place is the classic signature of a missing incremental invariant.

### 7.2 The key insight, `Keypoint::obs` is the sole mutation surface

The design rests on one verified fact. `Keypoint::obs` is never written outside `landmark_database.cpp`. A sweep of every reference to `.obs` across `src/` and `include/` returns three sites beyond the database itself, namely `local_mapper.cpp:1005`, `sqrt_keypoint_vo.cpp:686` and `landmark_block_abs_dynamic.hpp:66`, and all three bind it by const reference and only read.

Inside the database every mutation funnels through four methods, and the two removal helpers are shared by every public removal entry point.

```
   addLandmark ─────────────▶ creates kpts entry, NEVER touches obs
                              (so it needs no bookkeeping at all, see 7.7)

   addObservation ──────────▶ obs[t] = pos                    ── ADD hook

   removeFrame ─────────┐
   removeKeyframes ─────┤
   removeObservations ──┼───▶ removeLandmarkObservationHelper  ── DROP-ONE hook
   removeLandmark ──────┤          obs.erase(it2)
   (all of the above) ──┴───▶ removeLandmarkHelper             ── DROP-ALL hook
                                   kpts.erase(it)
```

Three hooks therefore cover every path by which the observation set can change, in the VIO estimator, the VO estimator, the offline `NfrMapper` and the local mapper alike. This is what makes the change small. No caller is touched, no public method changes signature, and no existing control flow is rerouted.

### 7.3 What must be counted, and the two ways to miscount it

The batch implementation defines the quantity precisely, and the incremental one must reproduce it exactly rather than approximately.

```cpp
seen[target.frame_id].insert(lms.begin(), lms.end());   // BuildObservedSets
m[a][b] = |seen(a) ∩ seen(b)|;                          // BuildCovisibilityMatrix
```

`seen` is keyed by `frame_id`, not by `TimeCamId`, and it is a set. Two consequences follow, and each is a distinct way for a naive incremental counter to be wrong.

The first is stereo multiplicity. A landmark observed by both cameras of the same keyframe appears once in `seen[f]`, because the set absorbs the duplicate. A counter that incremented on every `addObservation` would count that keyframe twice and inflate every covisibility cell it participates in. The incremental structure must therefore track, per landmark and per frame, how many cameras of that frame observe it, and move the covisibility counters only on the transitions between zero cameras and one.

The second is idempotent re-addition. `LocalMapper::setup_opt` re-adds every observation of every live feature track on every cycle, and the comment at the call site says so explicitly. The current `addObservation` absorbs this because `obs[tcid_target] = o.pos` is an assignment and `std::set::insert` is a no-op on a present element. A counter placed beside them without a novelty test would increment on every cycle and diverge without bound within seconds. The hook must fire only when the observation was genuinely absent.

Handle both and the incremental matrix is identical to the batch one, element for element, including the absence of zero-valued cells.

### 7.4 The maintained state

Five type aliases first, declared at namespace scope in `include/basalt/vi_estimator/landmark_database.h` rather than inside the class. None of them depends on `Scalar_`, so nesting them in the class template would give each instantiation its own spelling, `LandmarkDatabase<double>::CovisMatrix` and `LandmarkDatabase<float>::CovisMatrix`, for what is one type. The decisive reason is the interface. These types are the contract between `LandmarkDatabase` and `LocalMapper`, appearing in the signatures of `BuildObservedSets`, `BuildCovisibilityMatrix` and `RedundancyScore`, and at namespace scope `basalt::CovisMatrix` is exactly the type `LocalMapper::CovisMatrix` already names, so the substitution described below needs no edit at any use site. Nested, every one of those signatures would have to name a scalar parameter that the type does not depend on.

Note, 2026-09-12. This paragraph previously justified namespace scope by claiming that a class-scope alias would force `typename` on `ObserverFrameMap::const_iterator` throughout `landmark_database.cpp`. That is false and was checked rather than left standing. An alias declared inside the class template is a member of the current instantiation, and both GCC 12.2 and clang accept `ObserverFrameMap::const_iterator it = m.begin();` inside an out-of-line member definition with no qualifier, under `-Wpedantic` at C++14 and C++17 alike. Only a member of an unknown specialization requires `typename`, which is why the `typename Keypoint<Scalar>::MapIter` in section 7.6 is genuinely required, `Keypoint<Scalar>` being a dependent type distinct from the current instantiation.

```cpp
// Covisibility index types. Declared ahead of the class template and outside
// it, because none depends on the landmark scalar and a non-dependent alias
// needs no `typename` at its use sites.
using ObserverFrameMap = std::map<FrameId, uint32_t>;
using ObserverIndex = std::map<KeypointId, ObserverFrameMap>;
using ObservedByFrameMap = std::map<FrameId, std::set<KeypointId>>;
using CovisRow = std::map<FrameId, size_t>;
using CovisMatrix = std::map<FrameId, CovisRow>;
```

`FrameId` is `int64_t` at `include/basalt/utils/common_types.h:55` and `KeypointId` is `size_t` at `include/basalt/optical_flow/optical_flow.h:49`, so `basalt::CovisMatrix` is the identical type to the existing `LocalMapper::CovisMatrix`, declared `std::map<int64_t, std::map<int64_t, size_t>>` at `include/basalt/vi_estimator/local_mapper.h:60`, and `ObservedByFrameMap` is the identical type to the return of `LocalMapper::BuildObservedSets`. The class-scope alias in `LocalMapper` is therefore commented out as superseded and every unqualified use of `CovisMatrix` in that class resolves to the namespace-scope one without further edit, since `LocalMapper` is itself in namespace `basalt`.

The header gains `#include <cstdint>`, `<map>` and `<set>`, none of which it includes directly today.

Then three members, all private, all inert unless tracking is enabled.

```cpp
    // Landmarks observed by each frame, regardless of who hosts them. The
    // incremental equivalent of LocalMapper::BuildObservedSets.
    ObservedByFrameMap mpObservedByFrame;

    // Per landmark, the frames observing it and how many cameras of each frame
    // do so. The camera count is what makes a stereo observation contribute one
    // observer rather than two, and the key set is the landmark's observer
    // frame set, which RedundancyScore needs directly.
    ObserverIndex mpObserverFrames;

    // Symmetric |seen(a) ∩ seen(b)|, with zero-valued cells erased rather than
    // stored, matching BuildCovisibilityMatrix exactly.
    CovisMatrix mpCovisibility;

    bool mpCovisEnabled = false;
```

The multiplicity counter is `uint32_t` rather than `size_t` because it counts cameras of one keyframe, which is one or two in every supported configuration, and naming a 32-bit type documents that it is not an observation count.

The declarations these members and the section 7.5 helpers require, added to `class LandmarkDatabase`.

```cpp
 public:
  // Incremental covisibility, opt-in and off by default. Enabling it on one
  // instance affects no other, since lmdb is a value member of
  // BundleAdjustmentBase and each estimator owns its own.
  void EnableCovisibilityTracking(bool enable);

  const CovisMatrix& GetCovisibility() const;
  const ObservedByFrameMap& GetObservedByFrame() const;
  const ObserverFrameMap& GetObserverFrames(KeypointId lm_id) const;

 private:
  void CovisAddObserver(KeypointId lm_id, const TimeCamId& tcid);
  void CovisRemoveObserver(KeypointId lm_id, const TimeCamId& tcid);
  void CovisRemoveLandmark(KeypointId lm_id);
  void CovisDecrementPair(FrameId a, FrameId b);
  void CovisDecrementDirected(FrameId from, FrameId to);
```

`GetCovisibility` and `GetObservedByFrame` return the member directly and need no definition beyond that. `EnableCovisibilityTracking` assigns `mpCovisEnabled` and is deliberately not able to backfill an index for a database that already holds landmarks, since the only caller enables it in a constructor on an empty database. A later caller wanting to enable it mid-run would need a rebuild step, which is not written because nothing needs it.

Every type in the new code is spelled explicitly. Eight `auto` uses remain in the listings below, and none of them is a line this change writes. Four are pre-existing upstream lines quoted verbatim as context, namely the `kpts.find` lookup in `addObservation`, the `observations.find` lookup in each of the two removal helpers, and the `kpts[lm_id]` reference in `addLandmark`. Rewriting those would widen the diff into code this change does not otherwise touch. The other four are superseded lines retained as commented-out code under the rule in `CLAUDE.md` section 2, where the whole point is that they reproduce what the file used to say.

`mpObserverFrames` is the load-bearing one. It is simultaneously the inverse index that makes covisibility updates local, the multiplicity record that solves the stereo problem of section 7.3, and the observer count that section 7.8 uses to collapse the dominant cost of `RedundancyScore`. `mpObservedByFrame` and `mpCovisibility` are both derivable from it, and are materialised only because their consumers iterate them.

The relationship between the three, and the invariant that binds them.

```
  mpObserverFrames[L] = { f : some camera of frame f observes L }  with counts

        L₇ ──▶ { 100:1, 200:2, 300:1 }      frame 200 sees L₇ in both cameras

  mpObservedByFrame[f] = { L : f ∈ keys(mpObserverFrames[L]) }      the transpose

        100 ──▶ {L₇, L₉}     200 ──▶ {L₇}     300 ──▶ {L₇, L₉}

  mpCovisibility[a][b] = |{ L : a ∈ keys(mpObserverFrames[L])
                              ∧ b ∈ keys(mpObserverFrames[L]) }|

        INVARIANT, for every pair (a,b) with a ≠ b
        mpCovisibility[a][b] == |mpObservedByFrame[a] ∩ mpObservedByFrame[b]|
        and the cell is absent rather than zero when the intersection is empty
```

Section 7.10 turns that invariant into a runtime assertion.

### 7.5 The update rules

Adding an observer frame to a landmark raises the covisibility of that frame against every frame already observing it, by exactly one. Removing an observer frame lowers the same cells by one. Removing a landmark lowers every pair among its observer frames by one. That is the entire algorithm, and its correctness is the elementary identity that a set intersection is the sum over elements of the product of two membership indicators.

```
   landmark L already observed by frames {A, B, C}.  Frame D now observes it.

        before                          after
          A───B                           A───B
          │ ╲ │                           │╲ ╱│╲
          │  ╳│         add D             │ ╳ │ D        three cells raised,
          │ ╱ │        ─────────▶         │╱ ╲│╱         namely (D,A) (D,B)
          C───┘                           C───┘          and (D,C), each by 1

   the cost is O(|observers of L|), not O(|frames in the map|), and the total
   work to build L's contribution one observation at a time is k(k-1)/2, which
   is precisely what one batch intersection pass would have spent on L anyway
```

The three private helpers that implement it, new to `landmark_database.cpp`.

```cpp
// Lower one directed covisibility cell, erasing it and its row at zero so the
// structure never holds a cell BuildCovisibilityMatrix would omit. Split out of
// CovisDecrementPair as a named member rather than a lambda, because a lambda's
// closure type cannot be written explicitly.
template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisDecrementDirected(FrameId from,
                                                       FrameId to) {
    const CovisMatrix::iterator row = mpCovisibility.find(from);
    if (row == mpCovisibility.end()) return;
    const CovisRow::iterator cell = row->second.find(to);
    if (cell == row->second.end()) return;
    BASALT_ASSERT(cell->second > 0);
    if (--cell->second == 0) row->second.erase(cell);
    if (row->second.empty()) mpCovisibility.erase(row);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisDecrementPair(FrameId a, FrameId b) {
    CovisDecrementDirected(a, b);
    CovisDecrementDirected(b, a);
}

// One camera of `tcid.frame_id` has begun observing lm_id. Only the zero to one
// transition of the camera count is a new observer frame.
template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisAddObserver(KeypointId lm_id,
                                                 const TimeCamId& tcid) {
    if (!mpCovisEnabled) return;
    ObserverFrameMap& frames = mpObserverFrames[lm_id];
    if (++frames[tcid.frame_id] > 1) return;

    // The size guard is not an optimisation. operator[] materialises the row,
    // so on a landmark's first observation, where the loop below has no other
    // frame to pair with, an unguarded `mpCovisibility[tcid.frame_id]` leaves
    // an empty row that BuildCovisibilityMatrix never creates, and the parity
    // check of section 7.10 then fails. The fuzz test caught exactly this.
    if (frames.size() > 1) {
        // std::map does not invalidate references to existing elements on
        // insert, so holding `row` across the loop is safe even as the loop
        // creates other rows.
        CovisRow& row = mpCovisibility[tcid.frame_id];
        for (ObserverFrameMap::const_iterator f = frames.begin();
             f != frames.end(); ++f) {
            if (f->first == tcid.frame_id) continue;
            ++row[f->first];
            ++mpCovisibility[f->first][tcid.frame_id];
        }
    }
    mpObservedByFrame[tcid.frame_id].insert(lm_id);
}

// One camera of `tcid.frame_id` has stopped observing lm_id. The frame ceases
// to be an observer only when its last camera goes.
template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisRemoveObserver(KeypointId lm_id,
                                                    const TimeCamId& tcid) {
    if (!mpCovisEnabled) return;
    const ObserverIndex::iterator lm_it = mpObserverFrames.find(lm_id);
    if (lm_it == mpObserverFrames.end()) return;
    ObserverFrameMap& frames = lm_it->second;
    const ObserverFrameMap::iterator f_it = frames.find(tcid.frame_id);
    if (f_it == frames.end()) return;
    BASALT_ASSERT(f_it->second > 0);
    if (--f_it->second > 0) return;
    frames.erase(f_it);

    for (ObserverFrameMap::const_iterator f = frames.begin();
         f != frames.end(); ++f)
        CovisDecrementPair(tcid.frame_id, f->first);

    const ObservedByFrameMap::iterator obf =
        mpObservedByFrame.find(tcid.frame_id);
    if (obf != mpObservedByFrame.end()) {
        obf->second.erase(lm_id);
        if (obf->second.empty()) mpObservedByFrame.erase(obf);
    }
    if (frames.empty()) mpObserverFrames.erase(lm_it);
}

// The landmark is going away entirely. Driven off mpObserverFrames rather than
// off Keypoint::obs, so it is correct even where the host-keyed observations
// index has gone stale, which is the case RehostLandmark can produce.
template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisRemoveLandmark(KeypointId lm_id) {
    if (!mpCovisEnabled) return;
    const ObserverIndex::iterator lm_it = mpObserverFrames.find(lm_id);
    if (lm_it == mpObserverFrames.end()) return;

    const ObserverFrameMap& frames = lm_it->second;
    for (ObserverFrameMap::const_iterator it = frames.begin();
         it != frames.end(); ++it) {
        for (ObserverFrameMap::const_iterator jt = std::next(it);
             jt != frames.end(); ++jt)
            CovisDecrementPair(it->first, jt->first);
        const ObservedByFrameMap::iterator obf =
            mpObservedByFrame.find(it->first);
        if (obf != mpObservedByFrame.end()) {
            obf->second.erase(lm_id);
            if (obf->second.empty()) mpObservedByFrame.erase(obf);
        }
    }
    mpObserverFrames.erase(lm_it);
}

// Observer frames of one landmark, empty when the landmark is unknown. The
// static empty map is what keeps a miss from materialising an entry, which an
// operator[] accessor would do and which would corrupt the index on read.
template <class Scalar_>
const ObserverFrameMap& LandmarkDatabase<Scalar_>::GetObserverFrames(
    KeypointId lm_id) const {
    static const ObserverFrameMap kEmpty;
    const ObserverIndex::const_iterator it = mpObserverFrames.find(lm_id);
    return it == mpObserverFrames.end() ? kEmpty : it->second;
}
```

Every one of these returns immediately when tracking is disabled, so a consumer that never enables it pays one predictable branch per mutation and nothing else.

### 7.6 The three hook sites

`addObservation` is the only site that needs its existing logic touched, and only to learn whether the observation was new. The assignment on the else branch preserves the original overwrite semantics exactly, so a caller re-adding an observation with a different pixel position still updates it.

```cpp
template <class Scalar_>
void LandmarkDatabase<Scalar_>::addObservation(
    const TimeCamId& tcid_target, const KeypointObservation<Scalar>& o) {
    auto it = kpts.find(o.kpt_id);
    BASALT_ASSERT(it != kpts.end());

    // Superseded 2026-09-12. Covisibility bookkeeping must fire only on a
    // genuinely new observation, and LocalMapper::setup_opt re-adds every
    // observation of every live track on every cycle. emplace reports novelty,
    // and the else branch reproduces the assignment this line performed.
    // it->second.obs[tcid_target] = o.pos;
    const std::pair<typename Keypoint<Scalar>::MapIter, bool> ins =
        it->second.obs.emplace(tcid_target, o.pos);
    if (ins.second)
        CovisAddObserver(it->first, tcid_target);
    else
        ins.first->second = o.pos;

    observations[it->second.host_kf_id][tcid_target].insert(it->first);
}
```

The two removal helpers each take a single added line, placed first so it runs before the index surgery and, in the second case, before the early return.

```cpp
template <class Scalar_>
typename Keypoint<Scalar_>::MapIter
LandmarkDatabase<Scalar_>::removeLandmarkObservationHelper(
    LandmarkDatabase<Scalar>::MapIter it,
    typename Keypoint<Scalar>::MapIter it2) {
    CovisRemoveObserver(it->first, it2->first);

    auto host_it = observations.find(it->second.host_kf_id);
    ...
}

template <class Scalar_>
typename LandmarkDatabase<Scalar_>::MapIter
LandmarkDatabase<Scalar_>::removeLandmarkHelper(
    LandmarkDatabase<Scalar>::MapIter it) {
    // Ahead of the observations.end() early return below, which exists for
    // landmarks whose host bucket is already gone. The covisibility state is
    // keyed on frames rather than on hosts, so it must be torn down on that
    // path too.
    CovisRemoveLandmark(it->first);

    auto host_it = observations.find(it->second.host_kf_id);
    ...
}
```

Placing the teardown ahead of the early return is not cosmetic. That branch is reached when `removeLandmarkObservationHelper` has already drained the host bucket, and also when `addLandmark` produced a keypoint that never received an observation, and in the first of those the landmark may still hold observer frames. Driving the teardown from `mpObserverFrames` rather than from `Keypoint::obs` makes the helper correct on both branches without inspecting which one it is on.

No double counting arises from the interaction of the two helpers. `removeFrame`, `removeKeyframes` and `removeObservations` all erase individual observations through the first helper before deciding whether to invoke the second, and the first helper removes the corresponding entry from `mpObserverFrames` as it goes, so by the time the second runs it sees only what genuinely remains.

### 7.7 Why `addLandmark` needs no hook, and why that is the load-bearing property

```cpp
template <class Scalar_>
void LandmarkDatabase<Scalar_>::addLandmark(KeypointId lm_id,
                                            const Keypoint<Scalar>& pos) {
    auto& kpt = kpts[lm_id];
    kpt.direction = pos.direction;
    kpt.inv_dist = pos.inv_dist;
    kpt.host_kf_id = pos.host_kf_id;
}
```

The method writes geometry and host, and leaves `obs` alone. On an existing landmark it therefore changes the host without moving the entries the old host owns in the `observations` index, which is a genuine latent inconsistency in the host-keyed index. It is not reached today, because `LocalMapper::setup_opt` guards on `!landmarkExists` and `RehostLandmark` performs a full `removeLandmark` before its `addLandmark`, but it is a trap for a future caller.

The incremental covisibility state is immune to it by construction, because none of the three structures in section 7.4 is keyed by host. A host change is invisible to them, which is correct, since covisibility does not depend on who hosts a landmark. This is also why the incremental index is strictly more robust than `BuildObservedSets`, which reads through the host index and would silently lose a landmark whose host bucket had been orphaned.

### 7.8 The `LocalMapper` side

The constructor is the opt-in site, and it is the only place in the codebase where tracking is enabled.

```cpp
LocalMapper::LocalMapper(const Calibration<double>& calib,
                         const VioConfig& config)
    : NfrMapper(calib, config) {
    hash_bow_database =
        std::make_shared<HashBowStl<256>>(config.mapper_bow_num_bits);
    // Only this estimator reads covisibility. Every other LandmarkDatabase
    // instance, in SqrtKeypointVioEstimator, SqrtKeypointVoEstimator and the
    // offline NfrMapper, leaves it off and is bit-for-bit unaffected.
    lmdb.EnableCovisibilityTracking(true);
}
```

`SelectKeyframesToCull` stops building and starts reading.

```cpp
    // Superseded 2026-09-12 by the incremental index maintained inside
    // LandmarkDatabase. Both were rebuilt from nothing on every mapping cycle,
    // at a measured median of 8.2ms for the enclosing pass.
    // const auto seen = BuildObservedSets();
    // mpCovisMatrix = BuildCovisibilityMatrix(seen);
    // Bound by reference. Nothing in this function mutates lmdb, so the
    // reference stays valid for the whole selection loop.
    const ObservedByFrameMap& seen = lmdb.GetObservedByFrame();
```

The larger gain is in `RedundancyScore`, which is not what the brief asked about but falls out of the same index and dominates the pass. It currently answers "how many keyframes observe this landmark" by scanning every live keyframe and testing set membership, which is a linear search for a fact the inverse index holds directly.

```cpp
double LocalMapper::RedundancyScore(int64_t kf, const ObservedByFrameMap& seen,
                                    const std::set<int64_t>& alive,
                                    size_t& nObservedOut) const {
    const ObservedByFrameMap::const_iterator it = seen.find(kf);
    if (it == seen.end() || it->second.empty()) {
        nObservedOut = 0;
        return 0.0;
    }
    size_t redundant = 0;
    for (const KeypointId lm : it->second) {
        size_t others = 0;
        // Superseded 2026-09-12. This scanned every live keyframe to count the
        // observers of one landmark. mpObserverFrames holds exactly that set,
        // so the scan becomes an iteration over ~6 entries instead of ~150.
        // for (const int64_t k : alive) {
        //     if (k == kf) continue;
        //     const auto jt = seen.find(k);
        //     if (jt != seen.end() && jt->second.count(lm)) ++others;
        //     if (others >= mpMinRedundantObservers) break;
        // }
        const ObserverFrameMap& observers = lmdb.GetObserverFrames(lm);
        for (ObserverFrameMap::const_iterator f = observers.begin();
             f != observers.end(); ++f) {
            if (f->first == kf || alive.count(f->first) == 0) continue;
            if (++others >= mpMinRedundantObservers) break;
        }
        if (others >= mpMinRedundantObservers) ++redundant;
    }
    nObservedOut = it->second.size();
    return static_cast<double>(redundant) / static_cast<double>(nObservedOut);
}
```

The observer histogram in the debug block of `SelectKeyframesToCull`, which today rebuilds the inverse index inside an immediately-invoked lambda purely to print it, reads `mpObserverFrames` directly instead.

`FindBestRehostKf` swaps `mpCovisMatrix` for `lmdb.GetCovisibility()`, and `mpCovisMatrix` is deleted as a member.

```cpp
        size_t covis = 0;
        // Superseded 2026-09-12. mpCovisMatrix was a snapshot rebuilt once per
        // pass in SelectKeyframesToCull. The database now maintains the same
        // matrix incrementally, so the member is deleted and this reads it.
        // const auto row = mpCovisMatrix.find(c);
        // if (row != mpCovisMatrix.end())
        //     for (const auto& [other, count] : row->second)
        //         if (candidates.count(other)) covis += count;
        const CovisMatrix& covis_matrix = lmdb.GetCovisibility();
        const CovisMatrix::const_iterator row = covis_matrix.find(c);
        if (row != covis_matrix.end())
            for (CovisRow::const_iterator cell = row->second.begin();
                 cell != row->second.end(); ++cell)
                if (candidates.count(cell->first)) covis += cell->second;
```

That last substitution carries the one genuine semantic change in this section, and it must not be mistaken for a pure refactor. `FindBestRehostKf` is called from inside the cull loop, interleaved with `removeLandmark` and `addObservation`, so where it previously read a snapshot frozen before the first victim was processed, it now reads state that moves as the pass proceeds. The new behaviour is the better one and is the same argument section 4.3 made for re-scoring against `alive`, namely that ORB-SLAM3 is safe from compounding errors because `SetBadFlag` takes effect immediately and every subsequent query sees it. A rehost target whose connectivity has just been reduced by an earlier rehost in the same pass should be ranked on its reduced connectivity. It is nonetheless a behaviour change, and runs either side of it are not comparable.

### 7.9 Backwards compatibility

The blast radius is every translation unit holding a `LandmarkDatabase`, because the three hooks live in methods all of them call.

| Consumer | Enables tracking | Effect |
|---|---|---|
| `SqrtKeypointVioEstimator` (`sqrt_keypoint_vio.cpp:468,571,580,1070,1073`) | no | one predicated branch per observation add or remove |
| `SqrtKeypointVoEstimator` (`sqrt_keypoint_vo.cpp:310,422,431,550,733,927,931`) | no | as above |
| `NfrMapper` offline (`nfr_mapper.cpp:785,793`) | no | as above |
| `BundleAdjustmentBase::filterOutliers` (`ba_base.cpp:302,306`) | inherits the owner's setting | active only under `LocalMapper` |
| `LocalMapper` (`local_mapper.cpp`) | yes, in the constructor | full maintenance |
| `src/mapper.cpp`, `src/mapper_sim.cpp` display paths | no | read `getObservations()` only, untouched |

The default of `false` is what makes this safe. It follows the rule in `CLAUDE.md` section 7 that new behaviour should ride on a default that reproduces the old behaviour exactly, set explicitly only in the calling configuration. No public method changes signature, no existing method changes its observable result, and the one edited body, `addObservation`, is edited to an equivalent formulation of the same two operations.

The threading question the brief raised resolves cleanly and deserves stating, because it is easy to assume otherwise. `lmdb` is a value member of `BundleAdjustmentBase` at `ba_base.h:160`, so the VIO estimator and the local mapper each own a distinct instance. What they share is the class, not the object. No lock is therefore required for the new state, and enabling tracking on one instance cannot affect the other.

Four additions to the public surface, namely `EnableCovisibilityTracking`, `GetCovisibility`, `GetObservedByFrame` and `GetObserverFrames`, plus five private helpers and three members. Nothing is removed from `LandmarkDatabase`. The five namespace-scope aliases of section 7.4 are new names in namespace `basalt` and collide with nothing, which was checked by grepping the tree for each.

One alias is superseded rather than added. `LocalMapper::CovisMatrix` at `include/basalt/vi_estimator/local_mapper.h:60` names the identical type to the new `basalt::CovisMatrix`, so it is commented out in place with a note, and the unqualified uses of `CovisMatrix` in that class then resolve to the namespace-scope alias with no further edit. `BuildCovisibilityMatrix` keeps its signature and its meaning, since it is retained as the section 7.10 oracle. `getObservationsCountForPair` and `getNonLandmarkObservationsCountForKeyFrame` were already unreferenced once `ComputeCovisibility` was commented out in section 4, and they remain defined and unused.

One defect is left standing and recorded rather than fixed, since it is unrelated to covisibility and touching it would widen the diff. `landmarkExists(int lm_id)` takes an `int` while `KeypointId` is `size_t` and `TrackId` is `int64_t`, so every call from `LocalMapper` narrows. It is harmless at current track counts and wrong in principle.

### 7.10 Validation

The invariant of section 7.4 is checkable at runtime against the very functions this section replaces, which is why `BuildObservedSets` and `BuildCovisibilityMatrix` are retained rather than deleted. They become the oracle, and a future reader must not remove them as dead code.

```cpp
    if (mpVioDebugMode) {
        const ObservedByFrameMap ref_seen = BuildObservedSets();
        const CovisMatrix ref_covis = BuildCovisibilityMatrix(ref_seen);
        const bool ok = ref_seen == lmdb.GetObservedByFrame() &&
                        ref_covis == lmdb.GetCovisibility();
        std::cout << "[Local Mapper][covis] parity=" << (ok ? "OK" : "MISMATCH")
                  << " frames=" << lmdb.GetObservedByFrame().size()
                  << " ref_frames=" << ref_seen.size()
                  << " cells=" << lmdb.GetCovisibility().size()
                  << " ref_cells=" << ref_covis.size() << std::endl;
    }
```

A mismatch must be diagnosed before it is attributed. The two are not equal by definition, because the oracle reads through the host-keyed `observations` index while the incremental structure is driven off `Keypoint::obs`, and section 7.7 showed the host index can be left stale by an `addLandmark` on a live landmark. Where they diverge the incremental value is the one to trust, and the divergence is then evidence of a host-index defect rather than of a bookkeeping defect.

Offline, a randomised differential test is the strongest available check and is cheap to write, since the structures under test depend on nothing but `TimeCamId` and integers and transcribe onto stub types the way the section 4 and section 6 checks did.

```
  T1  one landmark, two frames                 covis(a,b) == 1
  T2  re-add the identical observation         covis(a,b) == 1 still
  T3  same frame, cam0 then cam1               covis unchanged, observers == 1
  T4  drop cam0, cam1 remains                  covis unchanged
  T5  drop the last camera of a frame          cell erased, not left at zero
  T6  removeLandmark with 4 observer frames    all 6 pairs decremented
  T7  rehost cycle, remove then add then
      re-add all observations but one frame    equals a from-scratch rebuild
  T8  fuzz, 10^4 random add and remove ops     equals a brute-force oracle
      against a brute-force recomputation      after EVERY operation
```

T8 subsumes the rest and is the one that matters. The oracle is four lines, namely rebuild the frame-to-landmark map from the stub observation sets and intersect every pair, and comparing after every single operation localises a divergence to the operation that caused it rather than to the end of the run.

Two further cases cover the accessors this change introduces.

```
  T9  GetObserverFrames on an unknown landmark   returns empty, creates no entry
  T10 tracking disabled, 200 adds and 7 removes  all three structures stay empty
```

T9 guards the one read path that could corrupt the index. An accessor written with `operator[]` would materialise an entry for every landmark queried, and `RedundancyScore` queries every landmark in the map on every pass, so the static empty map in section 7.5 is load-bearing rather than defensive. T10 is the backwards-compatibility claim of section 7.9 stated as a test.

All ten have been run. The update rules of section 7.5 were transcribed onto stub types, with every alias at namespace scope and the database itself a class template so that the `typename` question above is actually exercised, and compiled clean with `g++ -std=c++17 -Wall -Wextra -Wpedantic -O2`. T8 executed 6032 observation additions, 2930 observation removals and 1038 landmark removals over 40 landmarks and 20 frames in stereo, with both halves of the section 7.4 invariant asserted after every one of the 10000 operations. All ten pass and T8 reports no divergence.

T8 earned its place immediately by failing on its second operation against the first draft of `CovisAddObserver`, which materialised a covisibility row through `operator[]` before establishing that the landmark had a second observer frame. The empty row is invisible to every consumer, since `FindBestRehostKf` sums a row and an empty one sums to zero, so it would have survived every functional test and shown up only as a slow leak of rows for frames that are covisible with nothing. The guard in section 7.5 is the fix. The lesson generalises to the implementation, namely that a structure defined by an equality against a reference implementation should be tested by that equality and not by its consumers' tolerance of error.

In-run, the existing `CullRedundantKeyframes` timing instrumentation that produced `culling_time.log` gives a direct before and after on the same recording, with no new measurement apparatus needed.

### 7.11 Complexity

Let `N` be keyframes in the local map, `L` landmarks, `k` the mean observer frames per landmark and `m = Lk/N` the mean landmarks per frame. The 2026-09-08 observer histogram in section 5.3 fixes these empirically at `L = 744` and `k = 4476/744 = 6.0`, which at `N = 150` gives `m = 30`.

| Work | Batch, per cull pass | Incremental |
|---|---|---|
| Observed sets | `O(Lk log m)`, 4476 set inserts | maintained, `O(1)` to read |
| Covisibility matrix | `O(N²m)`, 11175 pairs × 60 ≈ 6.7e5 | `O(k)` per new observation |
| Redundancy scoring, per round | `O(N²m log m)` ≈ 6.8e5 lookups | `O(Nmk log N)` ≈ 2.7e4 |
| Per-cycle total | ≈ 2e6 operations | ≈ 3e4 operations |

The quadratic term in `N` disappears from both the matrix build and the scoring loop, which is the substantive result. The per-observation cost of `O(k)` is not additional work in any real sense, since building one landmark's contribution incrementally costs `k(k-1)/2` in total, exactly what a single batch intersection pass would have spent on that landmark, and the incremental version spends it once over the landmark's lifetime instead of once per cycle for as long as the landmark lives.

Memory is bounded by the observation count. `mpObservedByFrame` and `mpObserverFrames` are transposes of the same 4476 entries, and `mpCovisibility` holds one cell per ordered pair of frames that share a landmark, bounded by `N(N-1)` and in practice far below it. At the measured scale this is on the order of a few hundred kilobytes.

### 7.12 Not done, and why

The row-wise maximum form the brief originally proposed, `std::map<int64_t, std::pair<int64_t, int>>` mapping each keyframe to its best partner, is not maintained. It is a projection of `mpCovisibility` and would need its own invalidation logic on every decrement, since lowering the current maximum requires rescanning the row to find the new one. `FindBestRehostKf` sums a whole row rather than reading its maximum, so nothing in the current code would consume it. It is derivable on demand in `O(N)` from a row if a future consumer wants it.

Covisibility is not keyed by `TimeCamId`. The consumers all reason about keyframes, `BuildObservedSets` already collapses cameras, and keying by `TimeCamId` would produce a different and less useful number, as section 7.3 set out.

`getObservationsCountForPair` and `getNonLandmarkObservationsCountForKeyFrame` are not removed, although nothing calls them once `ComputeCovisibility` is gone. They are part of the class's public surface and their removal is a separate decision.

The stale host index that `addLandmark` can produce, described in section 7.7, is not fixed here. The incremental structure is immune to it, so fixing it is neither a prerequisite for nor a consequence of this change, and it belongs with the section 6.5 observation about merged tracks keeping stale geometry.

### 7.13 Applied, 2026-09-12

All of section 7 is in the tree across four files. `include/basalt/vi_estimator/landmark_database.h` gains the five namespace-scope aliases, the four accessors, the five private helpers and the three members. `src/vi_estimator/landmark_database.cpp` gains the helper definitions and the three hooks. `include/basalt/vi_estimator/local_mapper.h` has `LocalMapper::CovisMatrix` and `mpCovisMatrix` commented out as superseded and the three signatures repointed at the aliases. `src/vi_estimator/local_mapper.cpp` enables tracking in the constructor, reads the live index in `SelectKeyframesToCull` and `FindBestRehostKf`, and takes the observer count from `GetObserverFrames` in `RedundancyScore`.

Five deviations from the plan as written, all minor.

`src/vi_estimator/landmark_database.cpp` gains `#include <iterator>` for the `std::next` in `CovisRemoveLandmark`. The file previously included only `<algorithm>` and `<set>` and was relying on a transitive include.

The observer histogram in the `SelectKeyframesToCull` debug block, which section 7.8 described in prose only, now collects the landmark set from `seen` and reads each count from `GetObserverFrames` rather than rebuilding an inverse index inside an immediately-invoked lambda.

`FindBestRehostKf` accumulates into `cv` rather than `covis`, matching the name already in the file.

Comments are cut to what `CLAUDE.md` section 7 permits, so the rationale that section 7 of this document carries at length appears in the source only where reading the code cannot supply it, namely the stereo transition rule, the empty-row guard, the ordering requirement ahead of the early return in `removeLandmarkHelper`, and the mandated superseded blocks.

The parity check of section 7.10 is live under `mpVioDebugMode`, and it calls `BuildObservedSets` and `BuildCovisibilityMatrix` on every cull pass to do its comparison. It therefore reinstates in full the cost that this change removes. A debug run measures correctness and will not show the speedup, and the two must be measured in separate runs. Once the parity line has read `OK` across a full flight the block should be deleted, along with `BuildObservedSets` and `BuildCovisibilityMatrix` if nothing else has come to depend on them.

Nothing has been compiled. The container has neither TBB headers nor cmake, so verification remains the section 7.10 differential test on stub types, brace and paren balance on all four files, a check that no live reference to `mpCovisMatrix` survives, and a check that no code path assigns a whole `LandmarkDatabase`, which would have copied `mpCovisEnabled` across instances. Build and flight on a proper host are outstanding.
