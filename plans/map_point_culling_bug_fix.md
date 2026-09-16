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
