# Keyframe Driven Local Mapping

Reference for how the local mapper is fed by the estimator, written 2026-09-06 when the driver was moved from marginalisation to keyframe selection. The design document with the full rationale is [`../plans/kf_selection_driven_local_mapping.md`](../plans/kf_selection_driven_local_mapping.md). Related context lives in [`vio_localmapper_correction_loop.md`](vio_localmapper_correction_loop.md) and [`linearisation.md`](linearisation.md).

## The two inputs

`LocalMapper` now has two input queues with distinct jobs.

`mpKFInputQueue` carries `Keyframe::Ptr` and drives the thread. `MapLocally` blocks on it, so a mapping cycle happens once per keyframe. `LocalMapper::IngestKeyframe` is the only writer that inserts into `frame_poses`, the only writer of `img_data`, and the only writer of `mpNewKeyframesForTracking`.

`mpMargInputQueue` carries `MargData::Ptr` and is drained non-blockingly once per cycle, immediately after the keyframes of that cycle have been admitted. `LocalMapper::IngestMargData` refines poses of keyframes already held and extracts the nonlinear factors. It never inserts.

`struct Keyframe` lives in `include/basalt/utils/imu_types.h` beside `MargData` and holds `timestamp`, `pose` as `PoseStateWithLin<double>`, and `opt_flow_res`. It carries `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` because `PoseStateWithLin<double>` embeds a `Sophus::SE3d` by value, so it must be allocated with plain `new` and never with `std::make_shared`, which allocates through the allocator and bypasses the class operator new. The `std::allocate_shared` with `Eigen::aligned_allocator` form used for `MatchData` at `src/vi_estimator/local_mapper.cpp:451` is the alternative.

## Where a keyframe is published

`mpIsCurrentFrameKF` is raised inside the `if (take_kf)` commit block, at `sqrt_keypoint_vio.cpp:496` and `sqrt_keypoint_vo.cpp:347`, not at the threshold test that precedes it. This matters. `take_kf` has two writers, the threshold test and the constructor's `take_kf(true)`, and it is the constructor that makes the very first frame a keyframe. Only the commit block sees both.

`PublishKeyframe` is called from `optimize_and_marg` after `marginalize`, so the pose shipped is the jointly optimised one rather than the inertial prediction available at selection time. The flag is cleared before the queue null check so an unwired queue cannot latch it.

The published pose is rebuilt through `PoseStateWithLin<double>(t_ns, T_w_i)`, whose `linearized` argument defaults to false. This is load bearing. `NfrMapper::optimize` asserts `BASALT_ASSERT(!kv.second.isLinearized())` before applying an increment, at `src/vi_estimator/nfr_mapper.cpp:333` and `:414`, while the estimator marks marginalised states linearised via `setLinTrue` at `sqrt_keypoint_vio.cpp:1044`. A state copied verbatim from the estimator aborts the mapper's bundle adjustment.

## Why the marginalisation queue cannot deadlock the estimator

The mapper no longer blocks on the marginalisation queue, so an estimator blocked pushing to a full one while the mapper waits on an empty keyframe queue would hang the pipeline. It cannot happen.

A packet is pushed only when `kfs_to_marg` is non-empty, and `kfs_to_marg` is filled only inside the loop at `sqrt_keypoint_vio.cpp:735` whose guard is `kf_ids.size() > max_kfs`. Every iteration erases one identity from `kf_ids` at `:807`, and `kf_ids` grows only in the commit block at `:498`. The estimator can therefore emit at most one packet without an intervening keyframe selection, bounded by the excess of `kf_ids` over `max_kfs`, which is one in steady state. Reaching the capacity of ten set at `src/controller.cpp:172` is impossible.

Within a cycle the ordering is also fixed. `marginalize` pushes its packet before `PublishKeyframe` pushes the keyframe, so the mapper wakes on the keyframe, admits it, and only then drains the packet that describes it.

## Invariants the mapper now relies on

A keyframe culled by `CullRedundantKeyframes` stays culled. `IngestMargData` uses `frame_poses.find` and skips a miss, where it previously used `frame_poses[id] = p` and inserted. Before this change a culled keyframe still inside the estimator's `kfs_all` was resurrected by the next packet as a pose with no corners, landmarks or bag-of-words entry, was skipped by the redundancy criterion because its observation count was zero, and was culled again on the next cycle, giving a thrash loop.

Nonlinear factors must be pruned against the mapper's own `frame_poses`. `extractNonlinearFactors` draws endpoints from the packet, so it can name keyframes the mapper has culled, and those endpoints are read with `frame_poses.at` in `MapperLinearizeAbsReduce::operator()` at `include/basalt/vi_estimator/nfr_mapper.h:114` and `:135`, and in `computeRelPose` and `computeRollPitch` at `nfr_mapper.cpp:456` and `:469`. All four throw `std::out_of_range` on a miss, and `mapper_use_factors` defaults to true at `src/utils/vio_config.cpp:105`. The old unconditional insertion accidentally guaranteed every endpoint existed; `LocalMapper::PruneFactorsWithUnknownKeyframes` now does so deliberately, over the range appended by the packet just ingested. `CullRedundantKeyframes` prunes factors for the keyframes it removes, so the two guards cover both orderings.

## Facts about the estimator that were established while doing this

`frame_poses` inside the estimator is an invariant subset of `kf_ids`. Its only writer is the `states_to_marg_vel_bias` promotion at `sqrt_keypoint_vio.cpp:1053-1060`, which handles keyframes only, and every entry not in `kf_ids` is placed into `poses_to_marg` at `:698` and erased in the same call at `:1062`. Non-keyframe poses therefore never reached the local map, contrary to what was assumed before the investigation.

`MargData::frame_states` is empty after `processMargData` returns. Every `frame_states` entry appears in `aom` with `POSE_VEL_BIAS_SIZE` and is either moved into `m.frame_poses` when it belongs to `kfs_all` or erased outright when it does not, at `src/vi_estimator/nfr_mapper.cpp:112-119`. Any loop over `data->frame_states` after that call is dead. One such loop was removed from `LocalMapper::IngestMargData`; the identical dead loop in `NfrMapper::addMargData` was left alone because it belongs to the offline mapper's call path.

Marginalisation packets are emitted roughly once per keyframe, not once per frame. `marginalize` runs on nearly every frame because `vio_max_states` is three, but the push at `:915` is gated on `!kfs_to_marg.empty()`, and with `vio_max_kfs` at seven no packet at all is produced for the first seven keyframes. That warm-up gap was the sharpest cost of the old driver.

## The culling defect found on 2026-09-08

`LocalMapper::SelectKeyframesToCull` could empty the map outright in a single pass. Three protections were absent at once. The minimum map guard at `src/vi_estimator/local_mapper.cpp:668` was commented out, so there was no floor. `keep_recent` was commented out with `eligible_end` set to `ordered.size()`, so the newest keyframe was as eligible as the oldest. Criterion 1 never removed a keyframe from the candidate set once marked, so a mutually redundant pair marked both members and a run of high overlap keyframes was marked wholesale.

`mpCullCovisibilityThresh` is 0.5, and a nadir looking survey over homogeneous terrain produces exactly the high pairwise covisibility that drives the ratio past a half for long runs. `CullRedundantKeyframes` then removed each one, `lmdb.removeFrame` stripped the observations and Step 2b dropped every landmark left below two, so `get_current_points` returned an empty vector and the published snapshot carried no points at all.

The evidence was a screen recording rather than a log. Local map points measured zero over t in [16, 64], [224, 274] and [380, 458] of a 608 second flight and rebuilt monotonically from near zero at t equal to 66, 324 and 470, and the cloud vanished in place between two adjacent frames at t equal to 378 and 380 with the camera static. See [`visualiser_pipeline.md`](visualiser_pipeline.md) for the pixel-classification method and [`../plans/visualiser_defects.md`](../plans/visualiser_defects.md) for the fix.

The fix restores the floor as `mpMinLocalMapSize`, defaulting to 8 in `include/basalt/vi_estimator/local_mapper.h`, restores `keep_recent`, adds a `claimed` set so a keyframe that served as another's redundancy partner is not itself culled in the same pass, and adds a `max_cull` backstop equal to `frame_poses.size() - mpMinLocalMapSize`. On the pathological input of twenty mutually redundant keyframes with two new and a floor of eight, the old logic culls all eighteen eligible and the new logic culls nine.

Note that `mpMaxLocalMapSize` was raised from 50 to 150 separately, outside this work. The floor is independent of it.

## The landmark annihilation loop, measured 2026-09-08

An instrumented run of 313 mapping cycles established why the local map publishes zero points. The full analysis is [`../plans/map_point_culling_bug_fix.md`](../plans/map_point_culling_bug_fix.md). The facts worth carrying forward are these.

`MapLocally` processes exactly one keyframe per cycle in practice, on all 313 cycles, and `CullRedundantKeyframes` clears `feature_tracks` at the end of every cycle in both its branches. A track can therefore never span more than the current keyframe plus one match, so every landmark in `lmdb` has exactly two observations, which is also `LandmarkDatabase::min_num_obs`. Measured `hosted_obs / hosted_lms` was 2.000 in all 37 Criterion 1 events without exception.

That ceiling makes the redundancy ratio a constant. `setup_opt` records the host's own observation, so for a host `a` with `N` landmarks each seen by one partner `b`, `total_a` is `|obs[a][a]| + |obs[a][b]| = 2N` while `ComputeCovisibility(a,b)` is `|obs[a][b]| + |obs[b][a]| = N`. The ratio is exactly 0.5 for every host, always. `mpCullCovisibilityThresh` is 0.5 and the comparison is `>=`, so it fires on the floating-point equality. `best_ratio=0.5` appeared in all 37 selections and no other non-zero value appeared more than once. A threshold of 0.51, or a strict comparison, would have fired never.

Rehosting a two-observation landmark is impossible by construction. `RehostLandmark`'s retriangulation loop skips the culled frame and skips the new host, so with exactly two observations the loop body executes zero times, `triangulated` stays false, and the landmark is removed. All 2891 rehost attempts in the run failed this way, with `obs=2` in every case. The failure rate is 100 percent, not merely high.

The victim is always the universal host. `setup_opt` takes the host from `kv.second.begin()`, the earliest track element, and `SelectKeyframesToCull` scans in ascending timestamp order, so the oldest keyframe both hosts every fresh landmark and is examined first. A single log line reads `hosted_lms=122` against `landmarks=122`, and the count goes to zero.

A keyframe hosting nothing has `total_a == 0`, hits `continue`, and is permanently uncullable. 35430 of 35468 scored keyframes reported `best_ratio=0`, so the map saturated at 150 landmark-free keyframes while every keyframe carrying map points was removed on sight.

Criterion 2, the capacity rule, accounted for 126 of the 163 culling events and culls `ordered.front()`, which is the same universal host. Fixing Criterion 1 alone would not have helped.

Two independent defects surfaced in the same run. `LandmarkDatabase::getLandmarksForHost` emits a landmark once per observing keyframe rather than once per landmark, because it walks target rows and then the ids within each row, so `get_current_points` publishes duplicates and the GUI count is inflated by the observation multiplicity, measured as `points=36` against `lmdb_landmarks=16`. Separately, `bad_depth` rejected roughly two thirds of tracks in steady state, meaning triangulated inverse distance was non-positive or above 2.0, which is consistent with the degenerate two-view short-baseline geometry the track ceiling forces.

## How the track store actually behaves

Established 2026-09-08 while looking for the cause of the two-observation ceiling, because the obvious reading is wrong.

`feature_tracks` is not an accumulator. `LocalMapper::build_tracks` calls `feature_tracks.clear()` on both of its branches and repopulates from `mpTrackBuilder`, so the member is a per-cycle export of the builder's state. The `feature_tracks.clear()` in `CullRedundantKeyframes` Step 7 is therefore harmless, and pruning it instead of clearing it would change nothing. The accumulation lives in the `TrackBuilder`.

`TrackBuilder::AddNewMatches` Step G exports every node of each touched track, not only the observations added this cycle, so an exported track carries its full history. `DeleteTracksAfterCulling` invalidates a track only when `RemoveObservations` leaves it with no live observation, which is the correct behaviour and does not truncate surviving tracks. `MatchLocal` queries each new keyframe against the whole bag-of-words database rather than against its immediate predecessor, so tracks have room to grow.

The ceiling that forces exactly two observations per landmark is therefore somewhere between the matcher and `setup_opt`, and was not localised by the first run. Three candidates remain. Matching may return a single partner per new keyframe. `Filter(config.mapper_min_track_length)` runs on the first cycle with a configured value of 5 while `AddNewMatches` filters at 2, so the two disagree. Or tracks are long and `setup_opt`'s `feature_corners` and `frame_poses` guards drop most elements in the observation-add loop, which the original counters did not cover because they instrument only the triangulation loop. A `[tracks]` histogram in `build_tracks` now separates these.

## The inverse-depth invariant

`BundleAdjustmentBase::triangulate` at `include/basalt/vi_estimator/ba_base.h:95-128` documents and enforces that the returned homogeneous 4-vector has a unit-length direction in its first three components and the inverse distance in the last, through `worldPoint /= worldPoint.head<3>().norm()`. Every consumer relies on it, including `get_current_points`, which divides by the fourth component.

Transforming such a point into another frame with an SE3 matrix does not preserve the invariant, because the rotated direction plus the translation scaled by the inverse distance is not unit. Rescaling the whole 4-vector by the norm of its head restores it and leaves the represented 3D position unchanged, since the homogeneous point is scale-invariant. This is what `RehostLandmark`'s reprojection fallback does. Verified numerically over 2000 random pose pairs with a maximum world-point error of 7.3e-14 metres. Omitting the rescale corrupts the landmark silently rather than failing loudly, so it is worth remembering before writing any other frame change on a `Keypoint`.

## The second bottleneck, measured 2026-09-08

After the rehost and selection fixes the database holds landmarks again, peaking at 744 with track lengths spread from 2 to 17, but a 150 keyframe local map still plateaus near 500 while the track builder holds 1168 live tracks. The sink moved.

`filterOutliers` was called with a hard-coded `min_num_obs` of 4, at `src/vi_estimator/local_mapper.cpp`. `BundleAdjustmentBase::filterOutliers` at `src/vi_estimator/ba_base.cpp:287` removes a landmark outright when `num_obs - num_outliers < min_num_obs`. The measured observers-per-landmark histogram is `{2:121 3:261 4:43 5:35 6:71 ...}`, so 382 of 744 landmarks, 51 percent, sat below the threshold and were deleted by a single outlier observation. The 4 is inherited from `src/mapper.cpp:676`, the offline batch mapper, where a landmark accumulates observations over the whole sequence before any filtering is applied. It is wrong for a live map that creates landmarks with two observations and grows them. `LandmarkDatabase::min_num_obs` is 2, so 4 was also inconsistent with the database's own viability rule.

`computeError` compounds it. A residual on the host's own observation is recorded as the sentinel `-2` at `ba_base.cpp:190`, and `filterOutliers` deletes the landmark unconditionally on seeing it, however healthy its other observations are.

`setup_opt` also chose its triangulation pair worst-first. Track observations are keyed by timestamp, and the loop broke on the first candidate clearing `mapper_min_triangulation_dist`, which at 0.07 m is negligible at survey altitude. The nearest-in-time partner is the shortest baseline available, so the best-conditioned pair was routinely discarded. Simulated over 4000 samples with survey geometry, a point at 20 to 80 m with eight later keyframes at 1.5 m spacing and one pixel of bearing noise at `fx=320`, the median depth error is 2.888 m taking the first adequate baseline against 0.415 m taking the largest, an 85.6 percent reduction, with p90 falling from 11.950 m to 1.537 m.

Success rate at creation is identical under both orderings, because the loop already tried every candidate and only continued on failure. Ordering changes landmark quality, not yield. That is what makes it matter, since a landmark carrying three metres of depth error reprojects outside the outlier threshold and is then deleted by the rule above. Poor geometry manufactures outliers and an over-strict survival rule converts each one into a deletion.

Note also that the two total collapses in the run, at cycles 284 and 469, are node restarts between the two flights rather than a defect. The thread-exit block precedes each in the log.

## Why local matching stops, measured 2026-09-10

The mapper stops building tracks entirely a few keyframes into a flight, and the cause is upstream of every landmark mechanism recorded above. In a 66 cycle takeoff and landing, track export was non-zero on cycles 2, 3 and 4 and zero on the remaining 63, with `live_in_builder` frozen at 142 while `feature_corners` grew to 66. Keypoint detection and bag-of-words insertion were healthy throughout, so the failure is in candidate selection and matching, not ingestion. No culling event fired at all in that run.

`LocalMapper::MatchLocal` sources its candidate pairs from `hash_bow_database->querry_database` and from nothing else. A bag-of-words index answers the place-recognition question, which is the one loop closure asks. A local mapper needs its temporal neighbours, which it already knows from the timestamps, and making that adjacency conditional on a retrieval score means the local map is rebuilt only when place recognition happens to fire. Retrieval returned no candidate at all on 57 of the 66 cycles.

The hash makes retrieval brittle by construction. `HashBowBase::compute_hash` selects `mapper_bow_num_bits`, 16, fixed bit positions out of the 256-bit descriptor, and two descriptors share a bucket only when all 16 agree exactly, so a few percent of per-bit disagreement between two views of the same point sends a large fraction of true correspondences into different buckets. The surviving overlap must then clear `mapper_frames_to_match_threshold` at 0.04. Every relevant parameter is byte-identical to basalt's defaults at `src/utils/vio_config.cpp:94-101`, which were set for indoor handheld sequences, so nothing is mistuned and the mechanism is simply being used outside its design case.

ORB-SLAM3 does not rely on retrieval here. `LocalMapping::CreateNewMapPoints` draws neighbours from `GetBestCovisibilityKeyFrames` and, in the inertial case, walks the `mPrevKF` chain explicitly to guarantee the temporal neighbours are present, at `LocalMapping.cc:409-419`. Retrieval belongs to `LoopClosing`. The fix applied here matches every new keyframe against the `mpLocalMatchNeighbours` most recent older keyframes unconditionally, unioned with whatever retrieval returns so that a genuine revisit is still matched.

Two further defects were found in the same path. The initial `TrackBuilder::Build` filtered at `config.mapper_min_track_length`, which is 5, while `AddNewMatches` filters at 2 on every later cycle. A track from pairwise matching spans exactly two images on the cycle it is born, so the first filter destroys every track the first `Build` produces. It did not fire in the measured run only because the first cycle carried no matches, leaving the builder empty. And `matchDescriptors` was called with the literals 70 and 1.2, silently ignoring `config.mapper_max_hamming_distance` and `config.mapper_second_best_test_ratio`, so tuning either had no effect. The literals equal the current defaults, so behaviour was unchanged, but the configuration was inert.

## Three questions about the track builder, settled

Whether the union-by-rank tie break in `TrackBuilder::AddNewMatches` corrupts hosting, because it ignores timestamps when choosing a surviving root. It does not. `setup_opt` takes the host from `kv.second.begin()` and an exported track is an ordered set keyed by `TimeCamId`, so the host is always the earliest observation whatever the disjoint-set structure chose as root. The root determines only the `TrackId`.

Whether a track merge can leave the landmark database indexed by a stale `TrackId`. It cannot. The losing root's id goes into `retired_ids`, `setup_opt` Step A removes that landmark, and Step G exports the merged track with every node it now owns, so the observation-add loop re-attaches the loser's observations to the surviving landmark. There is a genuine adjacent defect though, not yet fixed. `setup_opt` creates geometry only when the landmark does not already exist, so a surviving landmark whose track has just absorbed a much longer baseline keeps the geometry triangulated from its original short one. The merge improves the observation set without ever improving the estimate.

Whether `removeLandmark` can leave a keyframe stranded in the observations index. It cannot. `removeLandmarkHelper` at `src/vi_estimator/landmark_database.cpp:217-246` erases each target entry as its landmark set empties and erases the host row when that empties, so `getHostKfs` never returns a stale key. A keyframe that hosted only the removed landmark drops out of `observations` altogether, which is correct, and the consequence downstream is that `BuildObservedSets` will not list it, `RedundancyScore` returns zero, and only the capacity rule can then remove it.

## Shutdown

The mapper's blocking wait is on the keyframe queue, so the keyframe sentinel is what releases it. `nullptr` is pushed to `mpKFOutputQueue` at four sites, `sqrt_keypoint_vio.cpp:186` and `:212`, and `sqrt_keypoint_vo.cpp:154` and `:182`. The last of these is new in substance, because the visual-only `ProcessFrame` previously returned on a null frame without pushing any sentinel at all, which meant visual-only mode in the event-driven model would hang in `LocalMapper::Stop` even before this change. See [`../BUG.md`](../BUG.md) for the original deadlock analysis.

`MapLocally`'s early exit when no keyframe was admitted checks `nullReceived` before continuing. Without that check a sentinel drained alongside real data is discarded and the next blocking pop never returns.

## Not verified

The change has not been compiled. The development container has no `cmake` and no TBB headers, and its `build/` tree predates `local_mapper.cpp` joining the library source list, so `CMakeFiles/basalt.dir/build.make` does not even list that file. Verification was a symbol audit only. Build and run on a proper host before trusting any of this in flight.

## `LandmarkDatabase` mutation surface, verified 2026-09-12

`Keypoint::obs` is never written outside `src/vi_estimator/landmark_database.cpp`. A sweep of every `.obs` reference across `src/` and `include/` finds three sites beyond the database, namely `src/vi_estimator/local_mapper.cpp:1005`, `src/vi_estimator/sqrt_keypoint_vo.cpp:686` and `include/basalt/linearization/landmark_block_abs_dynamic.hpp:66`, and all three bind it by const reference and only read. The observation set is therefore fully encapsulated, which is the precondition for maintaining any index derived from it incrementally.

Inside the class every mutation funnels through four methods. `addObservation` is the sole addition site. `removeLandmarkObservationHelper` drops one observation and `removeLandmarkHelper` drops the landmark entirely, and those two helpers are shared by all four public removal entry points, namely `removeFrame`, `removeKeyframes`, `removeObservations` and `removeLandmark`. There is no removal path that bypasses them.

`addLandmark` is the exception that matters. It writes `direction`, `inv_dist` and `host_kf_id` into `kpts[lm_id]` and never touches `obs`, so on an existing landmark it changes the host without relocating that landmark's entries in the host-keyed `observations` index, leaving the index stale. No current caller reaches that state, since `LocalMapper::setup_opt` guards on `!landmarkExists` and `RehostLandmark` performs a full `removeLandmark` before its `addLandmark`, but it is a trap for a future caller and it means `BuildObservedSets`, which reads through the host index, is less robust than a structure driven off `obs` directly.

`addObservation` is idempotent by construction, since `obs[tcid_target] = o.pos` is an assignment and `observations[host][target].insert` is a no-op on a present element. `LocalMapper::setup_opt` relies on this and re-adds every observation of every live track on every cycle. Any counter placed beside these operations must therefore test novelty explicitly, for instance by replacing the assignment with `emplace` and reading the returned bool, or it will diverge without bound.

`lmdb` is a value member of `BundleAdjustmentBase` at `include/basalt/vi_estimator/ba_base.h:160`, so the VIO estimator, the VO estimator and the local mapper each own a distinct instance. What they share is the class, not the object, and no lock is needed for per-instance state added to it.

`CullRedundantKeyframes` cost was measured over 1995 calls in `culling_time.log`, giving a median of 8.2 ms, p90 of 19.9 ms, p99 of 28.9 ms and a maximum of 35 ms, rising from a 0.65 ms first-decile mean to roughly 15 ms once the map fills. Most passes cull nothing, so in a no-cull pass that time is almost entirely `BuildObservedSets`, `BuildCovisibilityMatrix` and the `RedundancyScore` loop, whose combined cost is `O(N²m)` in the keyframe count `N` and the mean landmarks per frame `m`. The incremental replacement is planned in `plans/map_point_culling_bug_fix.md` section 7.

One naming trap in that plan's structures, found by a differential fuzz test rather than by inspection. `BuildCovisibilityMatrix` never emits a zero-valued cell, because it skips a pair with an empty intersection, so any incremental equivalent must erase a cell and its row at zero and must not let `std::map::operator[]` materialise a row for a landmark's first observation, which has no partner frame to pair with. The resulting empty row is invisible to every consumer and shows up only under a direct equality check against the batch implementation.

Applied 2026-09-12. `LandmarkDatabase` now maintains the covisibility matrix, the frame-to-landmark index and the landmark-to-observer-frame index incrementally, behind `EnableCovisibilityTracking` which defaults to off and which only `LocalMapper`'s constructor sets. `LocalMapper::BuildObservedSets` and `BuildCovisibilityMatrix` survive as the parity oracle for the `[covis]` debug line and are no longer the live source. Note that the parity check itself calls both on every cull pass, so a run with `mpVioDebugMode` set measures correctness rather than cost and cannot show the speedup.

Applied 2026-09-12. The eight tuning parameters from `mpMaxLocalMapSize` through `mpFilterOutlierThreshold` are no longer hard-coded. They are read from the `local_mapper_*` fields of `VioConfig` and assigned in `LocalMapper::LocalMapper`, with the in-class initialisers removed so that `VioConfig::VioConfig` is the sole home of each default. Every consumer named above is unaffected, because the members keep their names, types and public access. The mechanism and its one sharp edge, namely that a configuration file lacking a newly added key aborts the process with an uncaught `cereal::Exception`, are recorded in `vio_config_json.md`. `mpMinRedundantObservers` and `mpCullRedundancyThresh` can now be swept from the configuration file rather than by recompiling, which is what the `observers_per_landmark` histogram is meant to calibrate once tracking is restored.
