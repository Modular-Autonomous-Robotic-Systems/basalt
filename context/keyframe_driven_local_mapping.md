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

## Shutdown

The mapper's blocking wait is on the keyframe queue, so the keyframe sentinel is what releases it. `nullptr` is pushed to `mpKFOutputQueue` at four sites, `sqrt_keypoint_vio.cpp:186` and `:212`, and `sqrt_keypoint_vo.cpp:154` and `:182`. The last of these is new in substance, because the visual-only `ProcessFrame` previously returned on a null frame without pushing any sentinel at all, which meant visual-only mode in the event-driven model would hang in `LocalMapper::Stop` even before this change. See [`../BUG.md`](../BUG.md) for the original deadlock analysis.

`MapLocally`'s early exit when no keyframe was admitted checks `nullReceived` before continuing. Without that check a sentinel drained alongside real data is discarded and the next blocking pop never returns.

## Not verified

The change has not been compiled. The development container has no `cmake` and no TBB headers, and its `build/` tree predates `local_mapper.cpp` joining the library source list, so `CMakeFiles/basalt.dir/build.make` does not even list that file. Verification was a symbol audit only. Build and run on a proper host before trusting any of this in flight.
