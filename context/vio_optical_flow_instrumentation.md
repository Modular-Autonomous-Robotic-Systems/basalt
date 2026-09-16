# VIO and Optical Flow Pipeline, Instrumentation and Structural Facts

Recorded 2026-09-12 during the investigation of the SITL VIO divergence written up in [`../plans/vio_divergence_fix.md`](../plans/vio_divergence_fix.md). This file holds the durable facts about the estimator and front end that a future session would otherwise re-derive. It is a companion to [`vio_residuals_and_priors.md`](vio_residuals_and_priors.md), [`linearisation.md`](linearisation.md) and [`keyframe_driven_local_mapping.md`](keyframe_driven_local_mapping.md).

## The synchronous ingestion architecture

`BasaltSLAM::InitialiseSlam` at `ros_ws/src/slam/src/basalt/slam.cpp:53` passes `useProducerConsumerArchitecture = false`. Everything downstream follows from that single argument.

- `FrameToFrameOpticalFlow` starts no `processingLoop` thread, and `SqrtKeypointVioEstimator` starts no `proc_func` thread. Both are driven by direct calls.
- `Controller::TrackMonocular` at `src/controller.cpp:205` calls `opt_flow_ptr_->processFrame(...)` then `vio_estimator_->ProcessFrame(res)` inline, and `BasaltSLAMNode::GrabImage` at `ros_ws/src/slam/src/basalt/node.cpp:211` calls that from inside the ROS image callback. The entire front end, estimator, optimisation and marginalisation therefore run on the image callback thread.
- `out_state_queue` is never wired and `Controller::process_pose_queue_loop` never runs, because both are inside the `if (mpUseProducerConsumerArchitecture)` branch at `src/controller.cpp:189-201`. The pose reaches the caller as the return value of `ProcessFrame` instead.
- `SqrtKeypointVioEstimator::addIMUToQueue` and `addVisionToQueue`, which carry the only two other `[VIO]` tagged prints in the original source, are dead on this path. `Controller::GrabIMU` pushes into `imu_data_queue` directly at `src/controller.cpp:231`.
- The executor is `rclcpp::executors::MultiThreadedExecutor` at `ros_ws/src/slam/src/basalt/driver.cpp:11`, and the image and inertial subscriptions sit on two separate `MutuallyExclusive` callback groups created in the node constructor at `node.cpp:51-54`. The two callbacks can therefore run concurrently, which is what makes the non blocking inertial drain in `ProcessFrame` a race rather than a deadlock.

Queue capacities are `vision_data_queue` 10 and `imu_data_queue` 300, set at `src/vi_estimator/sqrt_keypoint_vio.cpp:124-125`, `local_map_input_queue_` 10 and `local_map_kf_queue_` 100, set at `src/controller.cpp:175,178`. Both ROS subscriptions use `rclcpp::SensorDataQoS`, which is `BEST_EFFORT`, `VOLATILE`, `KEEP_LAST(5)`. At the 167 Hz inertial rate that history depth tolerates only about 30 ms of consumer stall before the middleware discards samples.

## The inertial ingestion contract in `ProcessFrame`

The preintegration for frame `k` must span exactly `[t_{k-1}, t_k]`, and two assertions enforce that, namely `BASALT_ASSERT(meas->get_dt_ns() > 0)` and `BASALT_ASSERT(opt_flow_meas->t_ns == meas->get_dt_ns() + meas->get_start_t_ns())` at `src/vi_estimator/sqrt_keypoint_vio.cpp:508-512`.

The trap is that a zero order hold fallback satisfies both regardless of how many genuine samples were integrated. After draining the queue with `popFromImuDataQueueNonBlocking`, which `break`s the moment the queue is empty, the code closes any residual gap by forging the timestamp of whatever sample is currently held.

```cpp
if (meas->get_start_t_ns() + meas->get_dt_ns() < curr_frame->t_ns) {
    int64_t tmp = imuData->t_ns;
    imuData->t_ns = curr_frame->t_ns;   // forged
    meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
    imuData->t_ns = tmp;
}
```

If the inertial and visual streams are in different epochs, `imuData->t_ns > prev_frame->t_ns` is trivially true and `imuData->t_ns <= curr_frame->t_ns` is trivially false, so the integration loop never runs, `imuData` is never advanced past the first sample, and every frame in the entire session integrates one frozen sample stretched across its interval. Both assertions still pass. Nothing in the unmodified log distinguishes this from correct operation. See [`ap_dds_imu_stream.md`](ap_dds_imu_stream.md) for the measured epoch mismatch on this stack.

`popFromImuDataQueue`, the blocking variant used only for the very first sample and during initialisation, is itself bounded by `mpImuPopTimeoutMs` and returns `nullptr` on timeout, which makes `ProcessFrame` return `nullptr` and drop the frame silently.

## Assertions are live in this build

`BASALT_ASSERT` and friends key on `BASALT_DISABLE_ASSERTS`, not on `NDEBUG`, at `thirdparty/basalt-headers/include/basalt/utils/assert.h:90`. The only definition of that symbol is commented out at `CMakeLists.txt:407`, so every `BASALT_ASSERT` in the library is compiled in and will `std::abort` on failure even in a `-O3` build. Do not reason about a crash-free run as evidence that a debug-only invariant was skipped.

## Dead and inert machinery in the estimator

- `filterOutliers` is never called from `SqrtKeypointVioEstimator`. The call site is a standing TODO at `src/vi_estimator/sqrt_keypoint_vio.cpp:1758`, with the identical TODO in the visual only estimator at `sqrt_keypoint_vo.cpp:1375`. The estimator therefore has no outlier removal at all, and a bad landmark is moderated only by the Huber loss at `vio_obs_huber_thresh`, which is expressed in pixels rather than in standard deviations as recorded in [`linearisation.md`](linearisation.md), and is never deleted.
- `vio_outlier_threshold` and `vio_filter_iteration` are commented out of `struct VioConfig` at `include/basalt/utils/vio_config.h:68-69` and out of both the defaults and the cereal serialiser at `src/utils/vio_config.cpp:71-72` and `:189-190`. The keys present in `data/sitl_config_vo.json` and `data/euroc_config.json` are consequently inert, since unknown keys are ignored. See [`vio_config_json.md`](vio_config_json.md).
- `cam_time_offset_ns` is never applied online. The line is commented out at `sqrt_keypoint_vio.cpp:250` and at `sqrt_keypoint_vo.cpp:178`. Only the offline calibration tools honour it.
- `FrameToFrameOpticalFlow::filterPoints` returns immediately when `calib.intrinsics.size() < 2`, so in a monocular configuration the epipolar test never runs. The only geometric rejection in the whole visual front end is then the forward backward round trip gate at `optical_flow_max_recovered_dist2`.

## The `ExecutionStats` collector and its four traps

Recorded 2026-09-13 while designing the centralised logging subsystem in [`../plans/SLAM_logging.md`](../plans/SLAM_logging.md). `class ExecutionStats` at `include/basalt/utils/time_utils.hpp:108-139` is the upstream measurement collector. The estimator holds two instances, `stats_all_` and `stats_sums_` at `include/basalt/vi_estimator/sqrt_keypoint_vio.h:275-276`, with the identical pair in the visual only estimator at `sqrt_keypoint_vo.h:257-258`.

Its output is not internal, and it must not be deleted. `SqrtKeypointVioEstimator::debug_finalize` writes `stats_all.json` and `stats_sums.json` through `save_json`, which despite the name emits UBJSON because `save_as_json` is hard coded false at `src/utils/time_utils.cpp:139-140`. `python/basalt/log.py:93-95` loads `stats_all`, `stats_sums` and `stats_vio`, `python/basalt/run.py:107-112` collects all six file names, and `scripts/batch/generate-tables.py` builds the batch evaluation tables from them, the workflow documented in `doc/BatchEvaluation.md`. `debug_finalize` itself is called from exactly one place, `src/vio.cpp:634`, so none of this machinery runs on the live `Controller` path at all.

Adding an `int64_t` overload to `ExecutionStats::add` breaks the build. `int` converts to `double` and to `int64_t` at the same conversion rank, so every existing `add("num_it", it)` and `add("num_cams", size_t)` call becomes ambiguous. An integer channel needs a distinctly named method such as `add_int`.

Nanosecond timestamps do not survive the collector. Every scalar is stored as `double`, and `src/vi_estimator/sqrt_keypoint_vio.cpp:452` already passes an `int64_t` frame timestamp through it. A `double` is exact for integers only below 2^53, which is 9.007e15 ns or 104 days of clock. Simulation time near 1.4e12 ns is safe, but the UTC epoch stamps near 1.787e18 ns recorded in [`ap_dds_imu_stream.md`](ap_dds_imu_stream.md) quantise to 256 ns.

`ExecutionStats::merge_sums` at `src/utils/time_utils.cpp:91-111` crashes with `std::out_of_range` from `unordered_map::at` the first time it merges a name whose data is `std::vector<Eigen::VectorXd>`. Recorded 2026-09-14, found from a live SIGABRT during `basalt_slam_node` debug-mode operation. The function's `std::visit` has three arms, `double` and `int64_t` each call `add`/`add_int` on `this`, which inserts the name into `this->stats_` via `try_emplace`, but the `Eigen::VectorXd` arm is a deliberate no-op (`UNUSED(data); // TODO: for now no-op`, unchanged from the original ICCV'21 release commit `24325f2`), so no insertion happens for that arm. The trailing line, `stats_.at(name).set_meta(meta)`, ran unconditionally regardless of which arm fired, so it always throws for a `VectorXd` name. This is a genuinely pre-existing upstream defect, `merge_all` at `time_utils.cpp:74-89` has the equivalent `VectorXd` arm call `add(name, v)` and so never hits it, only `merge_sums` is broken, but the defect was dormant because nothing before the `Logger` refactor ever routed `VectorXd` data through `merge_sums`. `SqrtKeypointVioEstimator::logMargNullspace` (`sqrt_keypoint_vio.cpp:817-829`) calls `mpLogger->SolverScratch().add("marg_ns", margNs)` and `.add("marg_ev", margEv)`, both `Eigen::VectorXd`, into the fresh-per-cycle `mpSolverScratch`; `Logger::FinishVioOptimize` (`logger.cpp:1303-1308`) then calls `mpStatsSums.merge_sums(mpSolverScratch)` at the end of every `optimize()` call, so the crash fires on the first `optimize()` cycle that also runs `logMargNullspace()` (gated by `config.vio_debug || config.vio_extended_logging`). The pre-refactor code called `stats_sums_.add("marg_ns", checkMargNullspace())` directly, never through `merge_sums`, which is why the defect never manifested before. Fixed by guarding the trailing `set_meta` call on whether the visited arm actually inserted the name, `auto it = stats_.find(name); if (it != stats_.end()) it->second.set_meta(meta);`, which preserves the deliberate sums-side no-op for vector data instead of crashing on it. The backtrace's frame for `SqrtKeypointVioEstimator<double>::optimize` was misattributed to `/usr/include/c++/11/bits/shared_ptr_base.h:1295`, confirming gdb line numbers are unreliable in this optimized build and should not be trusted to the exact line without checking the surrounding logic.

## The offline `basalt_vio` tool passes the wrong argument to the estimator factory

Recorded 2026-09-13. `src/vio.cpp:312` passes `use_double` into the fifth parameter of `VioEstimatorFactory::getVioEstimator`, which is `useProducerConsumerArchitecture`, declared at `include/basalt/vi_estimator/vio_estimator.h:147-150`. That slot carried the scalar type selector upstream and was repurposed in this fork without updating the offline call site. The `--use-double` flag registered at `src/vio.cpp:249` therefore switches the offline tool between the synchronous and the producer consumer threading models rather than between the `float` and the `double` estimator, and the scalar type is pinned to `double` regardless because `getVioEstimator<double>` is the only exported instantiation, at `src/vi_estimator/vio_estimator.cpp:177-181`. The correction is to pass `false` and retire the flag. It is left standing as of this date, so any measurement taken with `--use-double` was taken on a threaded pipeline.


## Triangulation geometry and why the EuRoC threshold does not transfer

New landmarks are created only on keyframes, by triangulating an unconnected track against an earlier observation held in `prev_opt_flow_res`, guarded by `vio_min_triangulation_dist` and by an inverse distance acceptance window of `(0, 3.0)`, that is a depth window of `(0.33 m, infinity)`, at `sqrt_keypoint_vio.cpp:636-690`. The window bounds depth only from below.

The propagated relative depth uncertainty of a two view triangulation is

```
sigma_d / d = sigma_px * d / (f * b)
```

For the SITL mission the commanded altitude is 121 m and the camera is pitched 45 degrees below the horizon, so `d` is about `121 / sin(45 deg) = 171.1 m`. With `f = 320 px`, `sigma_px = vio_obs_std_dev = 0.5`, and a per keyframe baseline of `12.9 m/s x 0.156 s = 2.01 m`, this gives `0.133`. At the configured `vio_min_triangulation_dist` floor of `0.05 m` the same expression gives `5.35`, that is the depth is not determined by the data at all. The threshold is an EuRoC value, where depths are two to ten metres and five centimetres of baseline yields half a degree to two and a half degrees of parallax. Any aerial configuration needs it recomputed from the expected scene depth. Correction, 2026-09-12. This paragraph previously took the altitude as 150 m, giving a slant range of 212 m, a keyframe baseline of 2.36 m at a 183 ms interval, and ratios of 0.14 and 6.6. The commanded altitude of the mission is 121 m and the keyframe interval measured on the later capture is 156 ms, so the figures above supersede those.

The useful form of the threshold is a parallax rather than a distance, because `sigma_d / d = sigma_px / (f * alpha)` where `alpha = b / d` is the parallax angle, so a threshold expressed in parallax bounds the quantity that matters and transfers between scenes without retuning. At `f = 320` one pixel of parallax is `1 / 320 = 3.125e-3` rad, that is 0.179 degrees, and a ten percent depth uncertainty at `sigma_px = 0.5` needs five pixels, that is 0.9 degrees. The equivalent baselines at this slant range are 0.535 m for one pixel and 2.67 m for five. The parallax can be measured without any depth estimate, as the angle between the two bearings, which means the test can be applied before the triangulation rather than after it. The mapping layer already carries the same construction as `mpMaxCosParallax` in `include/basalt/vi_estimator/local_mapper.h`.

The corresponding window span is also small. `vio_max_kfs = 7` at the measured 156 ms keyframe interval spans 0.936 s, about 12.1 m of travel, a baseline to depth ratio of 0.071.

The estimator does search the whole window for the widest baseline. `prev_opt_flow_res` retains an entry for every frame in `frame_states` and `frame_poses` and is erased only at marginalisation, at `sqrt_keypoint_vio.cpp:1225` and `:1239`, so it holds up to `vio_max_states` plus `vio_max_kfs` frames. The candidate loop at `:617` iterates it in ascending timestamp order, because `kp_obs` is a `std::map` keyed by `TimeCamId`, and breaks at the first acceptance, so it naturally tries the oldest and therefore the widest baseline first. The problem is therefore not that a long baseline is unavailable, it is that the acceptance threshold lets a short one through.

Triangulation itself is the homogeneous direct linear transform at `include/basalt/vi_estimator/ba_base.h:99-128`, four rows built from the two bearings and the relative pose, solved by `Eigen::JacobiSVD` for the right singular vector of the smallest singular value, rescaled so the leading three components are a unit direction and the fourth is the inverse distance. Two properties follow and both matter. It minimises an algebraic residual rather than a reprojection error, which Hartley and Sturm (1997) show is biased and increasingly so as the configuration becomes ill conditioned. And as the baseline shrinks the two pairs of rows become linearly dependent, so the returned solution tends to a point at infinity, that is to a very small fourth component. Since the acceptance window at `:681-689` is `0 < inv_dist < 3.0`, it bounds depth only from below at 0.333 m and admits any large depth whatever, so the degenerate answer always passes.

## Measured characteristics of a SITL run, 2026-09-12

From run one of `/ws/log.log`, log lines 232 to 59,399, beginning `Setting up filter: t_ns 139614000000`.

| Quantity | Value |
|---|---|
| Estimator frames | 614 |
| Keyframes | 205, that is 33 percent of frames |
| Marginalisations | 610, of which 609 remove one state and 197 marginalise a keyframe |
| Keyframe span | 37.368 s |
| Keyframe interval, mean / median / p90 / max | 183.2 / 162 / 219 / 417 ms |
| Implied camera rate | about 16.4 Hz |
| Local mapper cycle, mean | 0.550 s |
| Local mapper queue wait, mean | 0.507 s |
| Local mapper net work per keyframe | about 43 ms |

The camera rate of 16.4 Hz supersedes the 7.42 Hz figure recorded in [`airsim_camera_extrinsics.md`](airsim_camera_extrinsics.md), which was measured on an earlier capture. Both figures are below the scene tick ceiling and the earlier one is not wrong for its own capture, but 16.4 Hz is what the 2026-09-12 stack delivers.

The local mapper is idle for ninety two percent of every cycle. The hundred slot keyframe queue cannot fill and the blocking `push` in `PublishKeyframe` cannot stall the estimator. Use this before suspecting the mapper of back pressure.

`too large update in pose` fires 3,388 times in 614 frames, roughly five times per frame, so the local mapper to estimator pose correction loop guarded by `kRelinThresholdTrans = 0.10 m` and `kRelinThresholdRot = 0.05 rad` in `sqrt_keypoint_vio.h:296-297` applies nothing whatever when the estimator has diverged.

## Negative `error_total` is expected, not a defect

`BundleAdjustmentBase::computeMargPriorError` at `src/vi_estimator/ba_base.cpp:477-502` deliberately drops the constant `0.5 r^T r` term of the marginalisation prior cost, because only the change in cost across an update matters to the optimiser. Its own comment records that the computed error can therefore be negative. Values of order `-10^5` in the estimator's `error_total` indicate a state far from the prior's linearisation point, which is diagnostically useful, but they are not evidence of a numerical fault.

## The instrumentation added on 2026-09-12

Every record is gated on `config.vio_debug` and carries `[VIO]` or `[Optical Flow]`. The estimator's optimiser output was previously untagged and collided with the local mapper's identical `[LINEARIZE]`, `[EVAL]`, `[ACCEPTED]` and `[REJECTED]` markers, so grep could not separate them. It now carries `[VIO]`.

| Record | Where | What it settles |
|---|---|---|
| `[VIO] ===== frame t_ns=... =====` | `ProcessFrame` | frame boundary marker |
| `[VIO][imu-ingest]` | `Controller::GrabIMU`, every 200 samples | raw inertial stamp domain, stamp rate against wall rate, maximum gap, monotonicity, queue depth |
| `[VIO] init: ...` | `ProcessFrame`, first frame | epoch offset between the two streams, the accelerometer sample the gravity alignment was taken from, seeded biases and gravity |
| `[VIO] imu frame ...` | `ProcessFrame`, per frame | `integrated` against `expected`, `coverage` of the frame interval, whether the zero order hold fallback fired and over what span, `head_minus_frame_ns` |
| `[Optical Flow] frame ...` | `processFrame`, per frame | track survival with the forward, backward and round trip failure split, detection yield, flow magnitude |
| `[Optical Flow] timing_ms` | `processFrame` | pyramid, track and detect latency, input queue depth |
| `[VIO] assoc ...` | `measure`, per frame | `connected0` against `unconnected0`, the keyframe decision and its inputs, database and window sizes |
| `[VIO] triang ...` | `measure`, per keyframe | triangulation rejection tally and the observed baseline and depth ranges, mirroring the mapper's `[setup_opt]` breakdown |
| `[VIO] state ...` | `measure`, per frame | position, velocity and bias, which is the divergence itself in metres |
| `[VIO][SPLIT] ...` | `optimize`, per inner iteration | the vision, inertial, bias and marginalisation prior terms separately, which is the first time the inertial contribution to the cost is observable |

The signature changes made to carry this are backwards compatible by construction. `FrameToFrameOpticalFlow::trackPoints` gained a trailing `OpticalFlowTrackStats*` defaulted to `nullptr`, so the stereo call site in `addPoints` keeps its previous behaviour. `addPoints` and `filterPoints` changed return type from `void` to `int` and no caller consumes the return. `class MultiscaleFrameToFrameOpticalFlow` and `class PatchOpticalFlow` derive from `class OpticalFlowBase` directly and not from `class FrameToFrameOpticalFlow`, so the blast radius is one class.

## Reducing a capture to tables and figures

Two scripts in `scripts/` perform the reduction, written on 2026-09-12 for [`../plans/vio_drift_analysis.md`](../plans/vio_drift_analysis.md) and intended to be reused for every capture.

`scripts/vio_log_extract.py` splits a capture at each `Setting up filter` line, joins every tagged record sharing a frame timestamp into one row, and writes `run<N>_frames.csv`, `run<N>_imu_ingest.csv` and `summary.txt`, with `--runs` selecting a subset. `scripts/vio_log_plot.py` reads those and writes eight figures per run, `trajectory`, `capture`, `state`, `frontend`, `triangulation`, `cost`, `inertial` and `window`, and overlays runs with `--compare`. The `trajectory` figure carries the top down track, the altitude against the commanded altitude, the cumulative path length and the distance from the takeoff point, which is the closure error. The `capture` figure bins track survival against interframe flow with the tracker capture range marked.

```bash
pip3 install --break-system-packages matplotlib      # absent from the dev container
python3 scripts/vio_log_extract.py /ws/log.log -o /tmp/vio_csv
python3 scripts/vio_log_plot.py /tmp/vio_csv -o plans/figures
```

Four parsing behaviours were forced by the data and should not be removed.

Fields are delimited by the position of the next key, not by whitespace, because Eigen prints a vector as space separated components. A vector arriving with fewer than three components is dropped rather than written into the scalar column of the same name, which is what the `|p|`, `|v|` and `|ba|` norms would otherwise collide with. Those norms are renamed to `p_norm`, `v_norm` and `ba_norm`.

A bracketed group such as `baseline_m[min=.. max=..]` names its members by the group, so `min` becomes `baseline_m_min`. Without this the `min`, `mean` and `max` of the two groups in the `[VIO] triang` record overwrite one another.

An intrusion from another thread is detected as a bracketed tag or any of `:`, `(` and `,` appearing after the record has begun writing fields, and the record is truncated there with the field the truncation landed in discarded. Separately, the script knows the final field each record type writes, so a record cut short without leaving any marker is also caught. On the 2,002 frame capture this rejects 699 records. Without it the reduction reports a maximum `opt_and_marg` of 52,034 ms where the true maximum is 18.25 ms, because a foreign write spliced digits into a truncated value.

A corrupted record can carry a timestamp with foreign digits spliced into it. The `[VIO] ===== frame` marker is short and rarely corrupted, so it arbitrates, and a record whose timestamp differs from the current marker by more than `--max-frame-gap-s`, five seconds by default, is rejected. One such record appears in the capture, an `[Optical Flow] frame` line carrying `t_ns=708360000000` against a true value near `1.08e11`.

## Measured characteristics of the 2,002 frame capture, 2026-09-12

A second capture replaced `/ws/log.log` after the one described above, beginning `Setting up filter: t_ns 48822000000`. Both are recorded because the two differ in frame rate and in length, and the later one is the reference for [`../plans/vio_drift_analysis.md`](../plans/vio_drift_analysis.md).

| Quantity | Value |
|---|---|
| Estimator frames | 2,002 over 112.935 s |
| Frame interval, p50 / p90 / max | 51 / 54 / 258 ms, that is 19.61 Hz |
| Keyframes | 667, one third of all frames |
| Keyframe interval, p50 / p90 / max | 156 / 210 / 420 ms |
| `optimize_and_marg`, p50 / p90 / max | 9.04 / 12.27 / 18.25 ms |
| Front end total, p50 / p90 / max | 1.26 / 1.92 / 3.47 ms |
| Track survival, p50 / mean | 0.867 / 0.828 |
| Tracks handed to the estimator, p50 | 264 |
| Connected track ratio, p50 | 0.568 against a threshold of 0.8 |
| Landmarks triangulated per keyframe, p50 | 50, against 75 rejected on baseline |
| Pose update rejections | 13,769 over 667 keyframe timestamps |

The keyframe decision is saturated. A connected ratio of 0.568 against a threshold of 0.8 means the ratio test wants a keyframe on essentially every frame, so the rate is set by `vio_min_frames_after_kf` alone. The counter is reset to zero on a keyframe and incremented on every frame that is not one, and the test at `sqrt_keypoint_vio.cpp:566-570` is the strict inequality `frames_after_kf > config.vio_min_frames_after_kf`, so a setting of `n` permits a keyframe every `n + 2` frames. At the configured 1 that is every third frame, and the capture confirms it exactly, since 2,002 frames divided by 667 keyframes is 3.00 and the median interval of 156 ms is three frames at the measured 51 ms frame interval. Any reasoning about the keyframe rate on this configuration must start from the floor rather than from the ratio test.

## The detection rate is capped by the grid, and the population is not

`detectKeypoints` at `src/utils/keypoints.cpp:161-233` is a hard occupancy grid. A cell containing an existing track is skipped and an empty cell receives at most `num_points_cell`, which `addPoints` always passes as 1. The loop runs `x` from `x_start` to `x_stop` inclusive in steps of `PATCH_SIZE` and likewise for `y`, which for a 640 by 480 image at `optical_flow_detection_grid_size = 40` gives sixteen columns and twelve rows, so at most 192 corners are added per frame. At the earlier setting of 20 the ceiling would be 768. The FAST threshold falls from 40 by halving until a corner is found or it drops below 5, so a textured cell almost always yields one and the ceiling rather than the texture is the operative limit on the detection rate.

That ceiling bounds the number added per frame and not the population, because several surviving tracks may drift into one cell and none of them is removed by the detector. The captures confirm both readings, with `of_detected` never exceeding 191 and `of_out` reaching 368. Correction, 2026-09-12. This section previously stated that the population is capped at 192, citing a first frame that produced 188. The first frame is the only frame on which the two coincide, because every cell is empty then, and the claim is wrong for every frame afterwards.

## The tracker capture range, and the takeoff transient that exceeds it

`trackPoint` at `include/basalt/optical_flow/frame_to_frame_optical_flow.h:369` loops from `level = config.optical_flow_levels` down to zero, so `optical_flow_levels = 3` builds four levels and the coarsest carries a scale of `2^3 = 8`. `ManagedImagePyr::setFromImage` builds `num_levels + 1` levels, so the configured value is the index of the coarsest rather than a count. The patch is `Pattern51`, defined at `include/basalt/optical_flow/patterns.h:147-155` as one half of `Pattern52`, whose raw extent is plus or minus 7, so the half extent is 3.5 px at whatever level it is applied. The capture range referred to level zero is therefore

```
r * 2^levels = 3.5 * 8 = 28 px per frame
```

which agrees with the `2^(L-1) r` rule quoted at `doc/VIO.md` §2.4.3 once `L` is read as the number of levels rather than the configured index.

The two flights of 2026-09-12 measure the degradation directly. Binning track survival against mean interframe flow gives 0.87 to 0.91 below one pixel, 0.68 to 0.84 between four and eight, 0.40 to 0.68 between sixteen and twenty, and 0.16 to 0.38 above twenty, at which point the maximum flow in those frames is 30 to 41 px. The knee therefore sits where the geometry says it should.

This matters only at takeoff, and it matters a great deal there. Cruise flow is `f v dt / d = 320 * 12.9 * 0.051 / 171.1 = 1.23` px per frame, far inside the range, but at takeoff the depth in the denominator falls to about one metre while the speed is already several metres per second, and the measured flow reaches 21.4 px mean and 41 px maximum. Survival falls to 0.34 and 0.097 and the landmark database empties completely. Raising `optical_flow_levels` to 4 doubles the range to 56 px and is the proposed remedy in [`../plans/vio_drift_analysis.md`](../plans/vio_drift_analysis.md) Phase 4.

## Conditional degeneracy, the failure mode the four direction analysis misses

`doc/Linearisation.md` §2.15 establishes that visual inertial odometry has exactly four permanently unobservable directions, the three components of global position and the rotation about gravity, and the estimator anchors precisely those four in the initial prior. That analysis is correct and it is not the whole story, because a direction can also be unobservable on account of what the vehicle happens to be doing. Such directions live in the nullspace of the Jacobian along one trajectory rather than in the nullspace of the model, they appear and vanish with the motion, no prior anchors them, and nothing in the estimator notices them. The two flights of 2026-09-12 were lost to one of them, so the distinction is worth keeping.

### The tilt and accelerometer bias confound

While the vehicle is not accelerating the accelerometer measures `a_meas = R_wb^T (-g) + b_a + n_a`, and perturbing the attitude by `delta_theta` about a horizontal axis while perturbing the bias by

```
delta_b = [delta_theta]_x R_wb^T (-g),   which for a level vehicle is  delta_b_horizontal = g * delta_theta
```

leaves the measurement exactly unchanged. One metre per second squared of horizontal bias is worth 5.85 degrees of tilt and the accelerometer cannot separate them at any averaging time. Vision breaks the tie only when there are landmarks with real parallax and when there is linear acceleration, and a vehicle standing on the ground or climbing vertically supplies neither.

Measured on the 2026-09-12 flights. The estimator settles at `b_a = (2.30, -0.60, -0.13)` from 6.35 s to 78 s, a horizontal magnitude of 2.377 against a specific force of 9.96, which is 13.8 degrees of attitude error. The pair is self consistent, which is why the trajectory stays bounded, and the cost of being on that manifold is that every real acceleration is rotated by 13.8 degrees before it is integrated.

The bias is identified at the first instant the mission supplies horizontal acceleration, at 78 s, when the speed rises from 8.52 to 16.90 m/s and the bias collapses from 2.3 to 0.5 m/s^2 within two seconds. That is the Martinelli (2014) observability result visible in a log file.

### Why clamping one member of the pair makes it worse

Raising `vio_init_bg_weight` to 1e8, which by the transposition recorded in [`linearisation.md`](linearisation.md) anchors the accelerometer bias at an implied 1e-4 m/s^2, succeeds completely at its stated aim and holds `|b_a|` below 0.233 m/s^2 for a whole flight. The confounded error does not disappear, because it is a property of the motion rather than of the parameterisation. It relocates to the attitude, and an attitude that is wrong and static still leaves the gravity reaction uncancelled, so the optimiser holds a rotated world frame by continuing to rotate it, paying with a gyroscope bias that `vio_init_ba_weight` anchors at an implied 0.316 rad/s and therefore does not resist. The measured gyroscope bias reaches 0.048 rad/s, the trajectory reaches 41.5 km and 757 m/s, and the vision cost median rises from 292 to 13,003, which is the optimiser recording in the log that it is being forced against the images.

The general rule to carry forward is that a confounded pair is broken by adding information and never by removing a degree of freedom, because removing one member relocates the error to whichever remaining direction is cheapest, and choosing that direction blindly is how a stiff prior turns a bounded error into a divergent one.

### The two biases are not symmetric

A gyroscope bias displaces the image by `f b_g T` pixels with no dependence on scene depth, so at `f = 320` and a 0.102 s window a bias of 0.015 rad/s is already worth half a pixel and the gyroscope bias is well observed at any altitude. An accelerometer bias displaces the image by `f b T^2 / (2 d)`, so it is observable only while the scene is close. Inverting gives the detection window

```
T_detect = sqrt( 2 sigma_px d / ( f b ) )
```

which at the SITL slant range of 171.1 m is `sqrt(0.5347 / b)` seconds. With `vio_max_states = 3` the active window is two inertial factors, 0.102 s at 19.6 Hz, which detects a bias of 51 m/s^2. The 2.3 m/s^2 bias the flights actually carried is worth 0.022 px inside that window. At aerial altitudes the accelerometer bias is observable through acceleration and effectively not at all through the images.

## The phantom map created before the vehicle moves

The single most damaging behaviour found on this stack, and the one with the shortest description. While the vehicle is stationary no landmark can be triangulated, because the baseline is zero and every candidate is rejected. The estimator's own position estimate nonetheless drifts, since nothing constrains it, and once that drift exceeds `vio_min_triangulation_dist` the estimator believes it has a baseline and triangulates a noise disparity against it,

```
d_phantom = f * b_believed / disparity_noise
```

Measured on both 2026-09-12 flights. The landmark database is empty for the first 1.25 and 1.42 s. The first successful triangulation occurs at a believed baseline of 0.0590 and 0.0541 m against a measured disparity of 0.296 and 0.223 px, and creates 89 and 25 landmarks at mean depths of 117.1 and 63.3 m. The true depth is under one metre, the camera being 0.1 m above the ground and pitched 45 degrees. The first map the estimator ever builds is wrong by two orders of magnitude, and it is the map against which takeoff is estimated.

The expression says what to do about it. The phantom depth is proportional to the believed baseline and inversely proportional to the tracking noise, so no tightening of the tracker removes it. Only a threshold expressed as a parallax removes it, because a stationary vehicle produces no parallax at all and no parallax threshold can be crossed by the estimator's own drift, whereas any threshold expressed in metres can.

## Measuring accuracy without a ground truth subscription

Three measurements escape the absence of a reference trajectory on the SITL mission, which takes off, climbs to a commanded 121 m, flies a spline and lands at the takeoff point. They cost nothing and should be computed for every run.

The closure error, the distance between the first and last estimated position, is drift in metres because the landing point is the takeoff point. The altitude scale factor, the estimated cruise plateau divided by the commanded altitude, is the scale error directly. The residual altitude at landing, the estimated height of a vehicle that is on the ground, isolates the frozen component of the error from the part that was later corrected.

Measured on the 2026-09-12 flights. The diverged flight closes 41,484 m from a 41,708 m path. The bounded flight closes 90.2 m from a 2,480.4 m path, that is 3.64 percent, of which 78.6 m is altitude and 44.6 m horizontal. Its plateau reads 209.0 m against a commanded 121 m, a scale factor of 1.73, while its descent covers 136.6 m of estimated altitude for a true 121 m, a factor of 1.13. The scale error was therefore acquired during the climb and largely corrected before the descent, and what remains at landing is the frozen residue rather than a continuing error. That decomposition is the clearest available statement of how marginalisation and first estimate Jacobians convert a transient error into a permanent one.

## There is no ground truth anywhere in the SLAM stack

`class BasaltSLAMNode` creates exactly two subscriptions, the image at `ros_ws/src/slam/src/basalt/node.cpp:125` and the inertial stream at `:140`. Nothing subscribes to a reference trajectory, so no statistic the stack produces is an accuracy measurement and every reduction in this file is a statement about internal consistency. ArduPilot's AP_DDS bridge publishes `/ap/geopose/filtered` and `/ap/twist/filtered`, and `/ws/mav.tlog` carries the same information offline through MAVLink `LOCAL_POSITION_NED`, so a reference is available without any change to the simulator. Until one is wired in, do not describe any run as more or less accurate than another.

## Measured characteristics of the two SITL flights of 2026-09-12, `/ws/log2.log`

A capture with three activations, of which the first two differ in exactly one parameter and are the comparison behind [`../plans/vio_drift_analysis.md`](../plans/vio_drift_analysis.md). The third raised `local_mapper_max_local_map_size` from 30 to 100 and was excluded. Flight A set `config.vio_init_bg_weight` to 1e8 and Flight B left it at 1e2, and by the transposition that parameter anchors the accelerometer bias.

| Quantity | Flight A, run 1 | Flight B, run 2 |
|---|---|---|
| Frames, duration | 1,835 over 106.9 s | 5,607 over 304.5 s |
| Frame interval, p50 / p90 / max | 54 / 54 / 3,159 ms, 18.52 Hz | 51 / 54 / 267 ms, 19.61 Hz |
| Keyframes, frames per keyframe | 612, 3.00 | 1,819, 3.08 |
| Keyframe interval, p50 / p90 | 159 / 210 ms | 156 / 207 ms |
| Initial accelerometer sample | 9.93888 m/s^2 | 9.94766 m/s^2 |
| Epoch offset at initialisation | -30 ms | -87 ms |
| Track survival, p50 | 0.902 | 0.888 |
| Tracks handed to the estimator, p50 | 269 | 277 |
| Landmarks in the database, p50 | 158 | 216 |
| Connected track ratio, p50 | 0.563 | 0.741 |
| Landmarks triangulated per keyframe, p50 | 47 | 56 |
| Cheirality rejections per keyframe, p50 / max | 214 / 786 | 1 / 489 |
| Largest baseline used, p50 | 944 m | 30.2 m |
| Mean triangulated depth, p50 | 83,341 m | 197.7 m |
| Accelerometer bias norm, p50 / max | 0.053 / 0.233 m/s^2 | 0.155 / 2.502 m/s^2 |
| Gyroscope bias, peak component | -0.048 rad/s | 0.007 rad/s |
| Position norm, p50 / max | 7,970 / 41,484 m | 222 / 471 m |
| Speed, p50 / p90 / max | 451 / 744 / 757 m/s | 6.09 / 17.5 / 21.8 m/s |
| Vision cost, p50 | 13,003 | 292 |
| Inertial cost, p50 | 55.0 | 11.7 |
| Marginalisation prior cost, p50 / min | -7,958 / -3.67e7 | -16,802 / -2.37e6 |
| Optimise and marginalise, p50 / p90 / max | 9.02 / 13.10 / 21.1 ms | 11.79 / 14.21 / 111.2 ms |
| Front end total, p50 / max | 1.29 / 3.77 ms | 1.22 / 3.90 ms |
| Local mapper corrections rejected | 2,232, median error 12.39 m | 9,498, median error 0.387 m |
| Records rejected by the reduction | 656 truncated, 0 malformed | 1,975 truncated, 1 malformed |

Several of these are reusable as thresholds rather than as history.

The inertial path is sound in both and the stamp rate is 167.08 to 167.93 Hz with no non monotonic sample, samples integrated per frame have median 9 against an expected 9, coverage has median 0.97 to 1.00 and minimum 0.882, and the zero order hold spans at most 6 ms. Treat any departure from these as a transport fault rather than an estimator fault.

The vision cost median separates a healthy run from a forced one by a factor of 45, 292 against 13,003. It is the cheapest single number that says the optimiser is overruling the images.

The cheirality rejection median separates them by a factor of 214, and is sharper still. A median in single figures is healthy and a median in the hundreds means the poses and the bearings no longer describe a consistent geometry.

The local mapper correction loop rejected every correction in both runs, but the median rejected translation error is 12.39 m in the diverged run against 0.387 m in the bounded one, the latter within a factor of four of the 0.10 m acceptance threshold. The loop is close to engaging once the drift is reduced, which supersedes nothing but is worth knowing before the threshold is next discussed.

The measured accelerometer stream is itself informative. Averaged over a whole flight the body frame reading is `(0.0013, 0.0092, -9.8017)` in the diverged run and `(-0.4903, 0.0042, -9.9360)` in the bounded one, with a body x standard deviation of 0.659 in the latter. That x component is not a bias. It is the gravity reaction resolved into a body frame that pitches nose down during forward flight, reaching -2.15 m/s^2 at 171.6 s which is 12.6 degrees of pitch. Do not read a nonzero mean body x acceleration on a multirotor as evidence of an accelerometer bias.

## The development container cannot build this library

There is no Eigen at `/usr/include/eigen3` in the dev container, and every translation unit requires it, so `g++ -fsyntax-only` against the recorded compile command fails at `#include <Eigen/Dense>`. `build/compile_commands.json` was generated on the build host, whose workspace root is `/home/shandilya/neurorobotics-ws`. Do not spend turns trying to compile here. Verification is by inspection plus a build on the host with `colcon build --packages-select slam`, and the build uses `-Wall -Wextra -Werror`, so a first host build must be checked for new diagnostics.

## Measured behaviour of the SITL estimator, capture of 2026-09-12

The instrumented capture settled the divergence investigated in [`../plans/vio_divergence_fix.md`](../plans/vio_divergence_fix.md). The numbers below are reusable, both as a healthy baseline for the paths that turned out to be sound and as the derivation of the failure for the one that did not.

### The inertial path is sound, and the epoch mismatch is gone

`imu_minus_frame_ns` at initialisation is -69,000,000, so the camera and inertial streams now share the simulation time epoch. See the correction in [`ap_dds_imu_stream.md`](ap_dds_imu_stream.md). The inertial stamp rate is 167.08 to 167.50 Hz with no non-monotonic sample in 18,800, the wall rate is 55.8 Hz because the simulation runs at one third of real time, and `max_gap_ms` is 9. Per frame, `integrated` equals `expected` within one sample on every frame of 2,001, `coverage` is 0.94 to 1.00, and `imu_queue_in` stays between 1 and 28 of a 300 slot capacity. The zero order hold fallback fires on 976 of 2,001 frames but always for `stretch_ms` of exactly 3, which is one inertial period of quantisation residual rather than a real gap. Treat a fired `stretch` as normal unless `stretch_ms` exceeds one inertial period.

### The front end is sound, and `flow_px_mean` is a scale oracle

Track survival over 1,999 frames has mean 0.828 and median 0.867. The dominant rejection is `recov_rej`, the forward backward round trip gate, which is the intended behaviour of a conservative gate.

More useful than either is the cross check that `flow_px_mean` makes possible, because it ties the images to physical units independently of anything the estimator believes.

```
flow [px/frame] = f * v * dt / d
```

For the SITL cruise, `f = 320`, `v = 12.9 m/s` from QGroundControl, `dt = 0.053 s` at the measured 19 Hz, and `d = 150 / sin(45 deg) = 212 m` gives 1.0 px per frame. The measured value through cruise is 0.52 to 0.88. Agreement to within ten percent proves the intrinsics, the frame rate and the tracker are all correct, and it means any disagreement with the estimator's own velocity is the estimator's error and not the sensor's. Use this before suspecting the camera or the front end of anything.

### The failure, accelerometer bias runaway, and why it is structural

The estimated accelerometer bias rises from zero to 2.6 m/s^2 in the first second and freezes at `ba = (-2.53128, 1.03963, 1.96361)`, `|ba| = 3.36808`, identical to five decimal places across consecutive frames. The true bias of the simulated sensor is of order 1e-4 m/s^2. Velocity and position are the integral and double integral of that constant, reaching 714 m/s and 26 km against a true 12.9 m/s and 1012 m.

Four facts make this structural rather than a tuning accident, and all four are worth keeping.

The filter is seeded with `vel_w_i = 0` at a moment when the vehicle is already climbing. `BasaltSLAM::InitialiseSlam` at `ros_ws/src/slam/src/basalt/slam.cpp:47-54` passes `Eigen::Vector3d::Zero()` unconditionally, and the lifecycle driver used to promote the node to ACTIVE only once the autopilot latched `flying`, so the seed was false by construction. Fixed 2026-09-12 by widening `SLAMDriverNode._desired_state` so that arming alone implies ACTIVE, which brings the estimator up while the vehicle is still at rest. The seed in `InitialiseSlam` is unchanged and is now true when it is read. The first accelerometer sample reads `|accel| = 11.1006 m/s^2` against a gravity magnitude of 9.81, which is 1.29 m/s^2 of net upward acceleration and proves the airframe is under power. The attitude seed is fine, since the lateral components place the measured vertical only 0.41 degrees from the body axis. It is the velocity seed that is false.

The bias random walk constrains the rate and never the level. `accel_bias_sqrt_weight` is `1/2.83e-4 = 3534` and at `dt = 0.053 s` the effective weight is 15,352 per unit of bias change between consecutive states, so a step of 0.13 between two states would cost 2.0e6. The measured `bias_a` term is 2.2e-08, which proves the bias does not step inside the window at all. It moves as a single value shared by every state, and a common shift of the whole window costs the random walk term exactly nothing. Only the marginalisation prior resists a common shift. The anchor on the accelerometer block is `sqrt(vio_init_bg_weight)`, which was 10 during the capture analysed here, an implied standard deviation of 0.1 m/s^2 against a sensor whose true bias stability is 2.83e-4. `data/sitl_config_vo.json` was raised to 1e8 on 2026-09-12, giving an effective weight of 1e4 and an implied standard deviation of 1e-4 m/s^2. Every other configuration file still carries 1e2. Remember that the accelerometer block reads `vio_init_bg_weight`, not `vio_init_ba_weight`, because of the transposition recorded in [`linearisation.md`](linearisation.md), and that `PoseVelBiasState::applyInc` in `thirdparty/basalt-headers/include/basalt/imu/imu_types.h` is the authoritative statement of the ordering, placing `bias_gyro` at 9 to 11 and `bias_accel` at 12 to 14.

The bias is observable only while the scene is close. A bias `b` displaces the predicted position over a window of duration `T` by `0.5 b T^2`, which the camera sees as `f * 0.5 b T^2 / d` pixels. With `vio_max_states = 3` the window is two inertial factors spanning 0.106 s, so a 3 m/s^2 bias is worth 7.8 px at 0.7 m depth, 0.54 px at 10 m and 0.027 px at 200 m, against `vio_obs_std_dev = 0.5` px. Inverting, detecting a bias `b` at depth `d` needs `T = sqrt(2 * sigma_px * d / (f * b))`, which for 0.5 m/s^2 at 212 m is 1.15 s, about 21 states at 19 Hz. At aerial altitudes the accelerometer bias is close to unobservable at any practical window length.

The observability window closes in about one second. The measured mean triangulated depth passes 6.4 m at 0.90 s, 10.8 m at 1.22 s and 35.8 m at 2.33 s as the vehicle climbs, while the bias reaches 2.34 m/s^2 at 0.96 s and 3.02 m/s^2 at 1.32 s. The error is acquired and locked in during precisely the interval in which it was still detectable.

The sign is diagnostic and worth recording. The specific force is close to `(0, 0, -9.7)` in the NED body frame and the estimator forms `a_world = R (accel - ba) + g`, so a positive `ba_z` increases the magnitude of the body vector and therefore the upward world acceleration. A velocity deficit must drive `ba_z` positive, and the first records of the runaway show `ba = (0.0216, -0.0384, 0.5555)`. If a future run shows the opposite sign, the mechanism is not a velocity deficit.

### The map inflates with the trajectory, which is why the vision never objects

`[VIO] triang` shows the mean triangulated depth tracking the pose divergence exactly, from 0.658 m at 0.26 s through 956 m at 5.3 s to 720,302 m at 70.9 s, with the baseline used rising from 0.054 m to 7,771 m. Because poses and map inflate together every reprojection stays satisfiable and the vision residual stays in the low thousands. Bearing only measurements cannot observe scale, so an intact looking map beside a diverged trajectory is the expected appearance of a scale runaway and is not evidence that the mapper is healthy and the estimator alone is sick.

### Two readings that look like defects and are not

A large negative `marg_prior` in `[VIO][SPLIT]`, reaching -2.0e6, is the first estimates Jacobian deviation measured against a position block weighted at `sqrt(vio_init_pose_weight) = 1e4`. A deviation of 0.14 m from the linearisation point alone gives a residual norm of 1400 and a cost of order 1e6. It is a symptom of the state moving far from its linearisation points, not a corrupt prior. `checkMargNullspace` confirms the prior is sound, reporting the three translations and yaw between 1e-13 and 1e-9 while roll and pitch carry finite response, which is the correct signature for a gravity aligned problem.

A negligible `imu` term in `[VIO][SPLIT]`, 0.033 against a vision term of 1746, does not mean the inertial measurement is missing. It means the inertial constraint is satisfied rather than binding, which is what happens once the bias has absorbed the discrepancy. Use `[VIO] imu frame` to decide whether the measurement is present and `[VIO][SPLIT]` to decide whether it is binding. They answer different questions.

### `mpPosesToUpdate` grows without bound

In `SqrtKeypointVioEstimator::measure` an entry is erased only when its correction is accepted. A rejected entry is retained, and an entry whose timestamp has left `frame_poses` is retained forever, so the staging map grows to the number of keyframes ever produced and is rescanned under its mutex every frame. The capture shows 667 entries and 13,770 rejection evaluations across 1,413 frames, with a median rejected `trans_err` of 6.996 m against a 0.10 m threshold. All 667 keyframes the mapper returned were rejected, so the correction loop applies nothing once the estimator has diverged.

### Instrumentation notes learned from the first capture

The build is clean on the host under `-Wall -Wextra -Werror` and the records are well formed. `std::cout` is not synchronised across the estimator, mapper and front end threads, so a small fraction of records interleave mid line and every reduction must filter them, typically with `grep -vE '\[Local Mapper\]|\[Mapper\]'` on the record being parsed. Guarding the estimator and front end writes with the mutex the mapper already uses would remove the need.

## SLAM lifecycle activation is driven by arm, not by takeoff

`SLAMDriverNode._desired_state` in `ros_ws/src/controllers/controllers/slam/driver_node.py` maps the vehicle phase onto the lifecycle state of whichever SLAM node the driver manages. Arming alone implies ACTIVE, so SLAM comes up while the vehicle is still on the ground. Correction, 2026-09-12. It previously promoted to ACTIVE only when the autopilot latched `flying`, which activated Basalt after takeoff and therefore after the vehicle was already moving, and that was the origin of the zero velocity seed described above. Any capture taken before this date carries the old policy, so the interval before takeoff will be missing from it.

The precedent for the correct policy was already in the tree. `VideoLoggingDriver::UpdateTargetState` at `ros_ws/src/controllers/src/logging/video_controller.cpp:177-193` targets ACTIVE on the rising edge of `armed` and never consults `flying`, so video logging has always covered the interval from arm onward while SLAM did not. Bringing SLAM into line means both stacks come up on the same event and a capture covers one interval rather than two. That alignment is what the 2026-09-12 change implemented.

Facts worth keeping about that driver.

`class SLAMDriverNode` is a base with two subclasses, `class BasaltSLAMDriver` in `controllers/slam/basalt_driver_node.py` and `class MonoDriver` in `controllers/slam/mono_driver_node.py`, the latter driving ORB-SLAM3. Anything changed in `_desired_state` reaches both, so the blast radius of a policy change is never one estimator.

The rules are numbered R1 to R5 in the controllers package's own plan and are referenced by number in the code comments and in `_phase_description`. R1 and R5 coincide, because ArduCopter forces `land_complete` and hence `flying=False` whenever disarmed, so the state `(armed=False, flying=True)` is unreachable. Since 2026-09-12 only R1, R5 and a widened R2 remain live. R3 is subsumed, because SLAM is already ACTIVE in the case it covered, and R4 is retired, because INACTIVE is no longer a resting state for an armed vehicle. Both are commented in place in `_desired_state` rather than deleted. The controllers package's own rule documentation still lists all five and must be corrected.

`TRANSITION_STEP` at lines 53 to 77 already contains `(UNCONFIGURED, ACTIVE) -> CONFIGURE`, so a desired state two steps away resolves correctly without a table edit. `_reconcile` takes at most one step per invocation and `/ap/status` publishes on change and keeps alive at 2 Hz, so the second step follows within half a second.

The Tello path has no armed-on-the-ground state at all. `_tello_status_callback` at lines 518 to 536 maps `taking_off` to `(armed=True, flying=True)` and `landed` to `(False, False)`, so the two flags flip together and any policy expressed in terms of `armed` alone behaves identically for that vehicle.
