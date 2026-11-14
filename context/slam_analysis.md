# SLAM Performance Analysis, Metrics, Visualisation and Cross Run Comparison

Recorded 2026-09-26 as a literature survey and codebase audit, undertaken to scope a dashboard that compares two or more runs of this stack under different configurations, features and hyperparameters. This document is deliberately broader than the current implementation, it lists what is already logged, what the SLAM and VIO evaluation literature considers a complete diagnostic picture, and where the two agree or diverge, so that later sessions refine it into a concrete dashboard design rather than re survey the field from scratch. Companion files are [`vio_optical_flow_instrumentation.md`](vio_optical_flow_instrumentation.md), [`linearisation.md`](linearisation.md), [`keyframe_driven_local_mapping.md`](keyframe_driven_local_mapping.md) and [`feature_detection_descriptors.md`](feature_detection_descriptors.md), each of which already documents parts of the subsystems referenced below in more depth. The full `Logger`/`AggregateStats` design that produces every CSV named here, and every concrete C++ logging change this survey's findings motivate, is recorded in [`../plans/SLAM_logging.md`](../plans/SLAM_logging.md), Change 1 through Change 9 for the original design, Change 10 through Change 15 for the yaw alignment, front end, parallax, IMU excitation and Bag of Words extensions this document's sections 3, 4 and 5 motivate, Change 16 through Change 18 for the stationarity detection, the inactive by default parallax gate, and the per run configuration and calibration snapshot a subsequent ablation cross reference against `plans/vio_drift_analysis.md` added, and Change 19 through Change 23 for the invoking timestamp this document's own tables now depend on being present in every file, closing the eight files that lacked it, `vio_solver_timing.csv`, `vio_marg_timing.csv`, `map_pair_match.csv`, `mapper_detect.csv`, `mapper_stereo.csv`, `mapper_linearise.csv`, `mapper_solver_iter.csv` and `mapper_iter_summary.csv`. This document itself carries the research grounding, the mathematics and the Python facing analysis method for that data, the plan carries the C++ that produces it. Every claim below that rests on an external source is marked `[N]`, resolved in the References section at the end, in the style of an academic paper, so a later reader can trace a design choice back to its source without re-deriving or re-searching it.

The run audited to ground this survey is `/ws/logs/2026-09-26_01-14-16/`, a live `basalt_slam_node` session against ArduPilot SITL that ended in the `merge_sums` crash fixed the same day, recorded in the fourth trap of `vio_optical_flow_instrumentation.md`. Because `AggregateStats::CommitRow` (`src/utils/logger.cpp`, guarded by `mpMtxRows`) flushes every 32 rows rather than buffering to end of run, every CSV from that crashed run is intact up to at most 31 unflushed rows, so a crash mid mission does not invalidate the analysis below.

A second run, `/ws/logs/2026-09-26_16-37-51/`, a complete flight spanning `t_ns` 180762000000 to 402528000000, about 222 seconds at roughly 18 Hz across its 4035 logged frames, is audited alongside the first to ground this feedback pass, since it is the first full run to exercise Change 10 through Change 18 end to end rather than only in isolated unit reasoning. `vio_init.csv` in this run has exactly one row, `yaw_aligned` reading 1 and `yaw_offset_rad` reading -0.695559, a value inside `(-pi, pi]` as the wrapping code requires, confirming the ground truth seeded yaw correction from `Change 10` logs correctly on a live flight. `config.json` and `calibration.json` both sit alongside the CSVs in this run's directory, confirming `Change 18`'s file copy runs correctly. `of_track_death.csv` has 161996 rows and `map_bow_candidate.csv` has 38854 rows in this run, the two largest new files `Change 11` and `Change 15` add, sizes this document's device recommendations below are calibrated against directly rather than against an estimate. This feedback pass also adds, to every section from section 2 onward, a table naming every quantity that section discusses, the plot or metric it feeds, a literature grounded description, and the concrete implementation instruction including which device, CPU or GPU, the computation belongs on, so that `prompt5.md`'s own implementation plan can be written directly from this document's tables without a further round of codebase investigation.

## 1. What the pipeline already logs

Every `Add*` method on `class Logger` (`include/basalt/utils/logger.h`, `src/utils/logger.cpp`) writes one row per call into a per record type CSV under the run's output directory, and has an independent `Print*` console counterpart gated separately. The table groups all 34 files by how often a row is written, because that grouping is what decides whether a quantity belongs on a dense time series plot, a sparse event log, or a single summary card.

### Dense, once per processed frame

| File | Emitted from | Columns | Notes |
|---|---|---|---|
| `of_frame.csv` | `Logger::AddOpticalFlowFrame`, called from `FrameToFrameOpticalFlow::processFrame` | `t_ns`, `frame`, `dt` (ms), `in`/`attempted`/`tracked`/`fwd_fail`/`bwd_fail`/`recov_rej`/`detected`/`epi_rej`/`out` (counts), `survival` (tracked/in), `flow_px_mean`/`flow_px_max` (px) | `recov_rej`, the forward backward round trip gate, dominates rejections in a healthy run. `flow_px_mean` is a scale oracle, `f v dt / d`, per `vio_optical_flow_instrumentation.md`. |
| `of_timing.csv` | `AddOpticalFlowTiming` | `pyramid`/`track`/`detect_add`/`total` (ms), `queue` (count) | Front end latency breakdown. |
| `vio_imu_frame.csv` | `AddVioImuFrame`, `ProcessFrame` when a previous frame exists | `integrated`/`expected` (count), `coverage` (ratio), `stretch` (flag), `stretch_span` (ms), `head_minus_frame` (ms) | `stretch` firing with `stretch_span` near one IMU period, about 6 ms at 167 Hz, is quantisation residue rather than a real gap. A larger `stretch_span` is the real transport fault signature. `Change 14` in `../plans/SLAM_logging.md` adds `accel_mean`/`gyro_mean` to this record, the raw material section 4 below needs. `Change 16` adds `accel_deviation_max`, `gyro_norm_max` and `is_stationary`, confirmed present in the second audited run's own header, the stationarity detection this document's excitation covariance metric in section 4 reads alongside `accel_mean`/`gyro_mean` rather than instead of them, since a stationary window is exactly the degenerate, near singular covariance case that metric exists to flag. |
| `vio_pose_update.csv` | `AddVioPoseUpdate`, `measure()` | `pending`/`applied`/`rejected` (counts), `max_trans_err`/`max_rot_err` against `limit_trans`/`limit_rot` (m, rad) | The local mapper to estimator correction channel. A run that has diverged saturates at 100 percent rejected. |
| `vio_assoc.csv` | `AddVioAssoc`, `measure()` | `connected0`/`unconnected0` (counts), `connected_ratio`, `thresh` (default 0.8), `take_kf` (flag) | The keyframe rate driver, though in practice the ratio test is usually saturated and the real keyframe cadence is set by the `vio_min_frames_after_kf` floor. |
| `vio_triang.csv` | `AddVioTriang`, `measure()` on a keyframe | rejection tally (`no_prior_obs`/`unproject_fail`/`short_baseline`/`not_finite`/`behind`/`too_close`), `baseline_m_min`/`baseline_m_max`, `depth_m_min`/`depth_m_mean`/`depth_m_max` | `depth_m_mean` is the phantom map and scale runaway detector, it should track true scene depth rather than pose divergence. `Change 13` in `../plans/SLAM_logging.md` adds `cos_parallax_min`/`cos_parallax_mean`/`cos_parallax_max`, closing the parallax gap section 3 identifies. `Change 17` adds a seventh rejection reason, `low_parallax`, and `candidates`/`added` totals confirmed present in the second audited run's own header, gated by the new `vio_min_triangulation_parallax_deg` config field, left at `0.0` and therefore inactive in every run audited so far, `low_parallax` reads 0 throughout both audited runs for this reason. |
| `vio_landmarks.csv` | `AddVioLandmarks`, `measure()` | `lost`/`lmdb` (counts), `marg_lost` (flag) | |
| `vio_state.csv` | `AddVioState`, `measure()` | `p_w_i` (m, vec3) and `p_norm`, `q_w_i` (vec4), `v_w_i` (m/s, vec3) and `v_norm`, `bg`/`ba` (vec3) and their norms | The primary state and divergence trace. |
| `vio_timing.csv` | `AddVioTiming`, tail of `measure()` | nine stage timings in ms, `pose_update`/`association`/`triangulation`/`lost_scan`/`opt_and_marg`/`publish`/`measure`/`imu_drain`/`process_frame`, plus `vision_queue`/`imu_queue` depth | The full per frame latency breakdown, the direct input to a stacked stage latency chart. |
| `vio_reproj_pre_opt.csv` | `AddVioReprojectionPreOpt`, added by `Change 12` in `../plans/SLAM_logging.md` | `t_ns`, `n` (count), `mean_px`/`median_px`/`p95_px`/`max_px` | The pre-optimisation reprojection residual distribution section 3 below identifies as missing, computed against the IMU propagated prior at association time, before the backend optimiser has touched the pose. Named `_pre_opt` so a later record logging the post optimisation residual has an unambiguous name of its own. |
| `gt_eval.csv` | `AddGtEval`, `Controller::TrackMonocular` | `p_w_i`, `q_w_i` (SLAM pose), `p_gt`, `q_gt` (ground truth pose) | New since the last audit of this codebase, see the correction in `vio_optical_flow_instrumentation.md`. Ground truth columns are blank, not zero, on a frame that matched no ground truth sample, and SLAM columns are blank before the estimator has produced a pose, because this aggregate sets `SetBlankEmptyVectors(true)`. The pair is raw and unaligned, an accuracy comparison still needs a rigid alignment step before subtraction, exactly as described in section 2. |

### Denser than per frame, once per solver inner iteration or per optimize call

| File | Emitted from | Columns | Notes |
|---|---|---|---|
| `vio_linearise.csv` | `AddVioLinearise`, `optimize()` per inner iteration | `error_total`, `lambda` (both scientific notation), `landmarks`/`states`/`poses` (counts), `numerically_valid` (flag) | |
| `vio_solver_iter.csv` | `AddVioSolverIter` | `status` (0 accepted, 1 rejected or invalid), `error_total`/`f_diff`/`l_diff`/`step_quality`/`step_size`, cost split `vision`/`imu`/`bias_g`/`bias_a`/`marg_prior`, `lambda`, `it_time`/`total_time` (ms) | The cost split answers directly whether the optimiser is being overruled by a dominant prior term rather than by the images, `vio_optical_flow_instrumentation.md` records a 45 fold difference in median vision cost between a healthy and a diverged run. |
| `vio_solver_timing.csv` | `AddVioSolverTiming` | `t_ns`, per stage ms, `allocateLMB`/`linearizeProblem`/`performQR`/`get_dense_H_b`/`solve`/`backSubstitute`/`computerError2`, `iteration` total, `resident_memory`/`resident_memory_peak` (bytes) | Mirrors the solve pipeline stages documented in `linearisation.md`. `Change 19` in `../plans/SLAM_logging.md` adds `t_ns`, `last_state_t_ns`, the same value `vio_solver_iter.csv`'s own row for this iteration already logs, closing this file's gap in the mandatory `t_ns` join key. |
| `vio_solver_summary.csv` | `AddVioSolverSummary`, tail of `optimize()` | `num_it`/`num_it_rejected`, `converged`/`terminated` (flags), `reason` (0 converged, 2 not converged and not terminated, 3 terminated without convergence), `optimize` (ms) | |
| `vio_marg.csv` | `AddVioMarg`, `marginalize()` | pose, state and keyframe counts being marginalised versus kept, `last_state_to_marg` (ns) | |
| `vio_marg_nullspace.csv` | `AddVioMargNullspace`, `logMargNullspace()`, gated on `vio_debug` or `vio_extended_logging` | `marg_ns` (vec7, per degree of freedom nullspace response), `ev_min`/`ev_max`/`ev_negative`/`ev_condition` | The eigenvalue spectrum of the marginalisation prior is an observability proxy, see section 4. This is the exact data whose `merge_sums` handling crashed the audited run. |
| `vio_marg_timing.csv` | `AddVioMargTiming` | `t_ns`, `measure` (ms, always 0.0, a documented limitation since that value is unavailable inside `marginalize()`), `marg_linearize`/`marg_helper`/`marg`/`marg_log`/`marginalize` (ms) | `Change 20` in `../plans/SLAM_logging.md` renames this file's own invoking timestamp column from `frame_id` to `t_ns`, the value logged is unchanged, only the name a join now needs to use. |

### One off or periodic

| File | Emitted from | Columns | Notes |
|---|---|---|---|
| `vio_init.csv` | `AddVioInit`, `ProcessFrame` on the first frame and again on a ground truth triggered reseed | `imu_minus_frame` (s), `accel`/`gyro` (vec3 and norm), `bg`/`ba`/`g` (vec3), `q_w_i` (vec4), `tilt_deg` | Two rows in the audited run, the initial identity seed and the ground truth seeded reinitialisation described in the corrected section of `vio_optical_flow_instrumentation.md`. `Change 10` in `../plans/SLAM_logging.md` adds `yaw_aligned`, `yaw_offset_rad`, `gt_yaw_rad` and `slam_yaw_rad`, three named columns rather than one packed vector, recording the yaw correction `ProcessFrame` already silently applies against ground truth at this seed frame, with `-999.0` as the explicit sentinel on a run without ground truth. |
| `vio_imu_ingest.csv` | `AddVioImuIngest`, `Controller::GrabIMU` every 200 samples | `n` (count), `stamp_rate_hz`, `wall_rate_hz`, `max_gap` (ms), `non_monotonic` (count), `queue` (count), `accel`/`gyro` (vec3) | A healthy baseline sits near 167 Hz stamp rate with zero non monotonic samples and a sub 10 ms max gap. |

### Per local mapper cycle

| File | Emitted from | Columns | Notes |
|---|---|---|---|
| `map_cycle_timing.csv` | `AddMapCycleTiming`, `LocalMapper::MapLocally` | 15 stage ms breakdown plus `queue_wait` | The mapper's measured 92 percent idle fraction, recorded in `vio_optical_flow_instrumentation.md`, is the ratio of `queue_wait` to total cycle time. |
| `map_bow_query.csv` | `AddMapBowQuery` | `keypoints`/`descriptors`/`bow_buckets`/`db_hits`/`above_thresh` (counts), `best_score` against `thresh` (default 0.04) | Ties to the hash collision probability analysis in `feature_detection_descriptors.md`. |
| `map_bow_candidate.csv` | `AddMapBowCandidate`, added by `Change 15` in `../plans/SLAM_logging.md` | `t_ns`, `query_kf`/`candidate_kf` (ns ids), `score` (f), `above_thresh` (flag) | One row per candidate the database returned, not only the winner, the raw material section 5 below needs for a precision recall sweep. |
| `map_pair_match.csv` | `AddMapPairMatch` | `t_ns`, `kf1`/`kf2` (ns ids), `descriptor_matches`/`min_matches`/`ransac_inliers` (counts), `stored` (flag) | Per candidate pair per match attempt, joins to `map_bow_query.csv` and `map_match_summary.csv` by keyframe id. `Change 21` in `../plans/SLAM_logging.md` adds `t_ns`, the invoking cycle's own keyframe, previously recoverable only through the implicit invariant that `kf1` always equals it, now an explicit column. |
| `map_match_summary.csv` | `AddMapMatchSummary` | `pairs`/`neighbour_pairs`/`neighbour_limit`/`verification_attempts`/`inlier_matches`/`total_matches`, `db_query`/`matching` (ms) | |
| `map_tracks.csv` | `AddMapTracks`, `build_tracks()` | `exported`/`live_in_builder`/`matches`/`new_kfs`/`corners` (counts), `len_hist` (vec10, index i is length i+2, index 9 accumulates 11 and above) | |
| `map_setup_opt.csv` | `AddMapSetupOpt` | 13 named triangulation rejection reasons, `lmdb_before`/`lmdb_after`, `min_triang_dist`/`max_cos_parallax`/`max_inv_dist` | The full rejection funnel, direct diagnostic for the worst first triangulation pair defect recorded in `keyframe_driven_local_mapping.md`. |
| `map_covis.csv` | `AddMapCovis`, `SelectKeyframesToCull` parity check, only under `mpVioDebugMode` | `parity` (flag), `frames`/`ref_frames`/`cells`/`ref_cells` | Only meaningful with the debug flag on, since the parity check runs both the incremental and the batch covisibility path and defeats the speedup it exists to verify. |
| `map_cull_select.csv` | `AddMapCullSelect` | `eligible`/`keep_recent`/`max_cull`/`min_observers`/`selected`, `capacity_rule` (flag), `observers_hist` (vec10, index i is i observers, index 9 accumulates 9 and above) | The histogram needed to calibrate `mpMinRedundantObservers` and the redundancy threshold. |
| `map_filter.csv` | `AddMapFilter` | `lm_before`/`lm_after`, `min_obs` | |
| `map_publish.csv` | `AddMapPublish` | `points`/`keyframes`/`lmdb_landmarks`/`host_kfs` (counts) | `keyframe_driven_local_mapping.md` flags a `points` versus `lmdb_landmarks` mismatch as a known duplicate emission defect. |
| `mapper_detect.csv` | `AddMapperDetect` | `t_ns`, `detection` (ms), `corners_total` (count) | The Shi Tomasi detector documented in `feature_detection_descriptors.md`, distinct from the FAST based front end detector. `Change 22` in `../plans/SLAM_logging.md` adds `t_ns`, the newest frame among the ones `detect_keypoints()` processed this call, one frame in the live `LocalMapper` path. |
| `mapper_stereo.csv` | `AddMapperStereo`, `NfrMapper::match_stereo()` | `t_ns`, (otherwise unpopulated in the audited run) | Empty because the audited dataset is monocular, `calib.intrinsics.size() < 2` short circuits stereo matching. Only populates on a stereo rig. `Change 22` adds `t_ns` alongside `mapper_detect.csv`'s own fix, derived the same way, the newest frame in `img_data` this call processed. |
| `mapper_linearise.csv` | `AddMapperLinearise`, the shared `NfrMapper` base that `LocalMapper` inherits | `t_ns`, `opt_pass`, `vision`/`rel_error`/`roll_pitch_error`/`total` cost, `landmarks`/`observations`/`rel_pose_factors`/`roll_pitch_factors` (counts) | A distinct optimiser from the VIO one, Schur complement rather than square root QR, per `linearisation.md`. `Change 23` in `../plans/SLAM_logging.md` adds `t_ns`, the invoking keyframe, and `opt_pass`, 0 or 1, since `LocalMapper::MapLocally` calls `optimize()` twice per cycle, once before landmark filtering and once after, both would otherwise share the same `t_ns` and the same `iter` sequence starting at zero. Any analysis of this file must group by `t_ns` and `opt_pass` together before treating `iter` as a fresh per call sequence. |
| `mapper_solver_iter.csv` | `AddMapperSolverIter` | `t_ns`, `opt_pass`, `status` (0 accepted, 1 rejected), `lambda`/`f_diff`/`max_inc`, cost split, `converged`/`error_increased` (flags) | Same `Change 23` addition and the same `t_ns`/`opt_pass` grouping requirement as `mapper_linearise.csv` above. |
| `mapper_iter_summary.csv` | `AddMapperIterSummary` | `t_ns`, `opt_pass`, `iteration` (ms), `num_states`/`num_poses`/`inner_steps` | `inner_steps` is 0 on a Gauss Newton step, since it has no Levenberg Marquardt inner loop. Same `Change 23` addition and grouping requirement as the two rows above. |

### Per landmark event, the densest files in the audited run

| File | Emitted from | Columns | Notes |
|---|---|---|---|
| `map_rehost.csv` | `AddMapRehost`, `RehostLandmark` and `CullRedundantKeyframes` | `outcome` (0 full multi view retriangulation, 1 success but under observed, 2 single view reprojection fallback, 3 failed before triangulation, 4 failed entirely), `reason` (0 none, 1 new host missing the observation, 2 new host unproject failed, 3 no view triangulated, 4 zero observations survived), `obs_before`/`obs_readded` | This is the mechanism behind the two observation landmark rehost defect measured in `keyframe_driven_local_mapping.md`, where rehosting fails by construction at exactly two observations. 171,353 rows in the audited run, since every hosted landmark on every culled keyframe emits one row. |
| `map_cull.csv` | `AddMapCull` | `victims`/`hosted_lms`/`rehost_attempted`/`no_host` (counts), `lm_before`/`lm_after`, `map_before`/`map_after` | Per culled keyframe summary, joins to `map_rehost.csv`. |

### Front end track lifetime, added by Change 11

| File | Emitted from | Columns | Notes |
|---|---|---|---|
| `of_track_death.csv` | `AddOpticalFlowTrackDeath`, added by `Change 11` in `../plans/SLAM_logging.md` | `t_ns`, `track_id`, `birth_t_ns`, `age_frames`, `reason` (0 forward fail, 1 backward fail, 2 recovery rejected) | One row per track death, the raw material for both the length distribution and the survival curve section 3 below describes, since a bucketed histogram alone cannot support the survival curve without the underlying per track ages. |

### Per run configuration and calibration snapshot, added by Change 18

| File | Emitted from | Contents | Notes |
|---|---|---|---|
| `config.json` | `Controller::initialize`, `Change 18` in `../plans/SLAM_logging.md`, a plain `basalt::fs::copy_file` of `config_path_`, not a `Logger` record | Every `VioConfig` field, `cereal` JSON, the same file format `VioConfig::load` reads | The one artefact that answers, without reopening the run's own launch command, exactly which hyperparameters produced this run's CSVs, essential for the cross run comparison in section 9, where two runs are compared precisely because a field here differs |
| `calibration.json` | `Controller::initialize`, `Change 18`, a plain copy of `calib_path_` | Camera intrinsics, `T_i_c` extrinsics, IMU noise parameters | Confirms which physical or simulated sensor model a run used, distinguishing a genuine algorithmic regression from a calibration change |

Both files are confirmed present, non empty and directly parseable JSON in the second audited run, `/ws/logs/2026-09-26_16-37-51/`, closing the loop between `Change 18`'s own plan entry and a live run's actual output directory.

Both `Add*` and `Print*` exist independently for all record types, console output gated by `IsConsoleEnabled()` and CSV output gated by a non empty output directory, so a run can be configured to produce either, both, or neither without touching the call sites in the estimator, front end or mapper.

## 2. Trajectory accuracy, the metrics a reference trajectory unlocks

Absolute Trajectory Error and Relative Pose Error are the two metrics that essentially every SLAM and VIO benchmark reports, defined in the TUM RGB-D benchmark paper `[1]`. Given ground truth poses `Q_1..Q_n` and estimated poses `P_1..P_n` in SE(3), ATE first finds the rigid transform `S` that best aligns the estimate onto the reference by least squares, Umeyama or Horn's closed form solution, then computes a per frame error `F_i = Q_i^{-1} S P_i` and reports the RMSE of the translational part across the whole run. ATE is a global consistency measure, it captures accumulated drift after removing the one degree of freedom that alignment cannot distinguish from genuine error.

RPE fixes a delta, either a frame count or a time or distance interval, and computes `E_i = (Q_i^{-1} Q_{i+delta})^{-1} (P_i^{-1} P_{i+delta})`, then reports RMSE across `i` `[1]`. RPE measures drift rate rather than accumulated drift, and because a single delta is noise dominated, the standard practice from Zhang and Scaramuzza's tutorial `[3]` is to sweep the delta across many sub trajectory lengths and plot error against length, a near flat line meaning no systematic drift and a rising line meaning drift proportional to distance travelled, exactly the signature that would separate a scale error from a one off event in this stack's own diverged flights.

The `evo` package `[2]` is the practical implementation of both metrics. Its alignment modes matter for this stack specifically, `--align` performs SE(3) alignment, rotation and translation only, which is the correct mode for a metric VIO whose scale is observable through the IMU, while `--align --correct_scale` performs Sim(3) alignment and would silently hide a real scale error as if it were an alignment artefact, exactly the caution that EuRoC and TUM VI apply when they report SE(3) aligned ATE RMSE rather than Sim(3) aligned `[5]`. `--align_origin`, aligning only the first pose rather than least squares fitting the whole trajectory, shows raw drift without spreading the error backward in time, useful for locating when a divergence began rather than only how large it eventually grew. `evo_ape` and `evo_rpe` report RMSE, mean, median, standard deviation, minimum, maximum and sum of squared error, and can plot an error coloured trajectory, the estimate drawn with colour proportional to per pose error magnitude, which shows where along the path the error concentrates in a single figure. `evo_res` ingests several saved result files and produces box or violin plots and a comparison table across them, which is `evo`'s own answer to comparing several configurations against each other and the closest existing template for this dashboard's cross run view, discussed further in section 9.

The alignment step itself is Umeyama's least squares solution `[2]`, reproduced here verbatim from `evo/core/geometry.py` since it is the exact function the Python analysis phase should call rather than reimplement, `with_scale=False` being the correct call for this metric VIO stack per the reasoning above.

```python
def umeyama_alignment(x: np.ndarray, y: np.ndarray, with_scale: bool = False):
    """
    Computes the least squares solution parameters of an Sim(m) matrix
    that minimizes the distance between a set of registered points.
    Umeyama, Shinji: Least-squares estimation of transformation parameters
                     between two point patterns. IEEE PAMI, 1991
    :param x: mxn matrix of points, m = dimension, n = nr. of data points
    :param y: mxn matrix of points, m = dimension, n = nr. of data points
    :param with_scale: set to True to align also the scale (default: 1.0 scale)
    :return: r, t, c - rotation matrix, translation vector and scale factor
    """
    if x.shape != y.shape:
        raise GeometryException("data matrices must have the same shape")
    m, n = x.shape
    mean_x = x.mean(axis=1)
    mean_y = y.mean(axis=1)
    sigma_x = 1.0 / n * (np.linalg.norm(x - mean_x[:, np.newaxis]) ** 2)
    outer_sum = np.zeros((m, m))
    for i in range(n):
        outer_sum += np.outer((y[:, i] - mean_y), (x[:, i] - mean_x))
    cov_xy = np.multiply(1.0 / n, outer_sum)
    u, d, v = np.linalg.svd(cov_xy)
    if np.count_nonzero(d > np.finfo(d.dtype).eps) < m - 1:
        raise GeometryException("Degenerate covariance rank, "
                                 "Umeyama alignment is not possible")
    s = np.eye(m)
    if np.linalg.det(u) * np.linalg.det(v) < 0.0:
        s[m - 1, m - 1] = -1
    r = u.dot(s).dot(v)
    c = 1 / sigma_x * np.trace(np.diag(d).dot(s)) if with_scale else 1.0
    t = mean_y - np.multiply(c, r.dot(mean_x))
    return r, t, c
```

### 2.1 Three alignment modes, one Umeyama core

A single alignment mode is not enough for this stack, since it runs three structurally different configurations, `optical_flow_type` monocular against `mapper_use_factors` disabled or enabled, and a stereo capable calibration when one is supplied, each with a different degree of freedom that alignment legitimately may absorb without hiding a real estimation error. `rpg_trajectory_evaluation` `[3]` itself ships exactly three alignment modes for this reason, `sim3`, `se3` and a yaw only mode it names `posyaw`, dispatched from `trajectory.py`'s `align_type` and implemented in `align_utils.py` and `align_trajectory.py`. Monocular VO has no metric scale reference at all, so `sim3`, full rotation, translation and a free scale factor, is the only mode that does not conflate scale error with alignment residual. A calibrated stereo rig has a metric baseline and therefore an observable scale, but has no absolute heading or gravity reference of its own, so `se3`, full rotation and translation with the scale fixed at one, is correct there, aligning all three rotation degrees of freedom since none of them is otherwise anchored. A gravity aligned VIO estimator is different again, its roll and pitch are observable directly from the accelerometer's own gravity direction, exactly the tilt this stack already logs as `vio_init.csv`'s `tilt_deg`, so absorbing roll or pitch into the alignment would silently launder a real attitude estimation error into an invisible alignment artefact. Only global yaw and global position are unobservable for a monocular gravity aligned VIO, the same four directions Hesch et al. `[10]` and OpenVINS `[11]` derive in section 4 below, so `posyaw`, translation and yaw only, scale fixed at one, is the correct mode, and is exactly `rpg_trajectory_evaluation`'s own `alignPositionYaw`.

The Umeyama core all three modes share, reproduced below from `rpg_trajectory_evaluation`'s own `align_trajectory.py` `[3]`, is structurally the same closed form SVD solution already quoted from `evo` in section 2's opening code block, with two extra keyword arguments, `known_scale` fixing `s = 1` instead of computing it from the singular values, and `yaw_only` replacing the general rotation `R = U S V^T` with a rotation constrained to the gravity axis alone, found by maximising `trace(Rz(theta) . C)` over `theta` in closed form rather than by an unconstrained SVD.

```python
import numpy as np


def get_best_yaw(C):
    """Closed form theta maximising trace(Rz(theta) . C), rpg_trajectory_evaluation's
    align_trajectory.py. C is the 3x3 cross covariance data^T . model."""
    a = C[0, 1] - C[1, 0]
    b = C[0, 0] + C[1, 1]
    return np.pi / 2 - np.arctan2(b, a)


def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def align_umeyama(model, data, known_scale=False, yaw_only=False):
    """model = s * R * data + t. model and data are Nx3 point sets, rows are
    points, adapted for internal consistency between its three keyword
    combinations from rpg_trajectory_evaluation's own align_trajectory.py and
    align_utils.py [3], itself S. Umeyama, "Least-Squares Estimation of
    Transformation Parameters Between Two Point Patterns," IEEE PAMI 13(4),
    1991. A general purpose SVD of the cross covariance is always computed
    for the scale term, and is separately overridden by the closed form
    yaw only rotation when yaw_only is set, so every one of the three
    dispatch modes below shares one code path rather than three."""
    mean_model = model.mean(0)
    mean_data = data.mean(0)
    model_zerocentered = model - mean_model
    data_zerocentered = data - mean_data
    n = np.shape(model)[0]

    cov = 1.0 / n * np.dot(model_zerocentered.T, data_zerocentered)
    U_svd, D_svd, V_svd = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U_svd) * np.linalg.det(V_svd) < 0:
        S[2, 2] = -1

    if yaw_only:
        rot_C = np.dot(data_zerocentered.T, model_zerocentered)
        theta = get_best_yaw(rot_C)
        R = rot_z(theta)
    else:
        R = np.dot(U_svd, np.dot(S, V_svd))

    if known_scale:
        s = 1.0
    else:
        sigma2 = 1.0 / n * np.multiply(data_zerocentered, data_zerocentered).sum()
        s = 1.0 / sigma2 * np.trace(np.dot(np.diag(D_svd), S))

    t = mean_model - s * np.dot(R, mean_data)
    return s, R, t


def align_sim3(p_gt, p_est):
    """Monocular VO, scale unobservable, full 7 degree of freedom similarity."""
    return align_umeyama(p_gt, p_est, known_scale=False, yaw_only=False)


def align_se3(p_gt, p_est):
    """Stereo VO, scale metric from the baseline, no gravity reference, all
    three rotation degrees of freedom still need aligning."""
    _, R, t = align_umeyama(p_gt, p_est, known_scale=True, yaw_only=False)
    return R, t


def align_posyaw(p_gt, p_est):
    """Gravity aligned VIO, roll and pitch observable via the accelerometer
    and must not be absorbed into alignment, only translation and yaw are
    genuinely unobservable, see the Hesch/OpenVINS nullspace in section 4."""
    _, R, t = align_umeyama(p_gt, p_est, known_scale=True, yaw_only=True)
    return R, t
```

`class Alignment`, per `prompt5.md`'s own naming, is this function set wrapped as a class, one method per mode, each taking the two `p_w_i`/`p_gt` position arrays already available in `gt_eval.csv` and returning the rotation and translation `class TrajectoryAnalysis` applies before computing ATE or RPE, exactly the ordering `evo` itself enforces between its alignment and its metrics modules `[2]`.

`rpg_trajectory_evaluation` `[3]` is the VIO specific complement, its signature plot is RPE against sub trajectory length averaged over every starting point at each length with a shaded standard deviation band, and the toolbox is built explicitly for multi algorithm, multi dataset batch comparison producing paper ready plots from one command, again the closest prior art to the stated goal of comparing configurations rather than evaluating one run alone.

KITTI's odometry benchmark `[4]` computes the same idea over a fixed grid of path lengths, 100 to 800 metres, and reports translation error as a percentage of path length and rotation error in degrees per metre, suited to continuous driving motion rather than a hovering platform. Tracking success is usually reported alongside accuracy rather than folded into it, as percentage of frames successfully tracked, failures per kilometre travelled, and time to recovery after a tracking loss, split into short recoveries bridged by IMU dead reckoning and long recoveries that need a full relocalisation against the existing map. None of these three appear in this stack today, since nothing currently flags a frame as lost versus tracked at the estimator level beyond the landmark loss counters in `vio_landmarks.csv`.

`gt_eval.csv`, described in section 1, is the substrate all of the above needs and did not have until now. It is raw and unaligned, so the dashboard's first new derived computation, not new logging, is the Umeyama alignment step above followed by ATE and RPE at several sub trajectory lengths, reusing `evo`'s conventions directly rather than reinventing them. The three drift only proxy measurements already recorded in `vio_optical_flow_instrumentation.md`, closure error between first and last position, altitude scale factor against the commanded altitude, and residual altitude at landing, remain useful as a cheap sanity check that needs no alignment step at all, but should now be reported alongside rather than instead of the aligned ATE and RPE.

### 2.2 Relative Pose Error as a drift rate, and why its sweep needs a GPU

RPE, defined in this section's opening paragraph, is a drift RATE measurement rather than an absolute drift measurement precisely because it is computed at a fixed delta and then swept across many deltas, `evo`'s own `RPE` class in `evo/core/metrics.py` `[2]` computes, for a chosen delta and pairing mode, `E_i = (Q_i^-1 Q_(i+delta))^-1 (P_i^-1 P_(i+delta))` for every admissible start index `i`, then reduces `E_i` to a scalar through whichever `PoseRelation` is requested, `translation_part` as `norm(E_i[:3, 3])` or `rotation_angle_rad` as the geodesic SO(3) angle of `E_i`'s rotation block. The quantity this document's ablation study needs directly is not only the RMSE `evo` itself reports for one delta, but the full per start index array of scalar errors, since that array is indexed one to one by the start frame's own timestamp and is exactly the timestamp wise drift rate series a plot can show alongside `t_ns`. `rpg_trajectory_evaluation`'s own README `[3]` states plainly that this per pair computation is time consuming enough that the toolbox caches it to disk rather than recomputing it on a second run, and the reason is structural, sweeping a grid of `L` candidate lengths against a trajectory of `n` poses, each length needing on the order of `n` pair evaluations, each pair needing two matrix inversions and three 4x4 multiplies, is `O(n L)` total small matrix operations, which for this stack's own 4035 to 30000 row per frame CSVs swept across even a modest length grid reaches the low millions of matrix operations, comfortably past the point where a Python level loop over pairs, exactly what `evo`'s own reference implementation runs, dominates wall clock time.

The fix is not a faster algorithm, the pair definition itself is already `O(n)` per length and cannot be improved without changing what RPE means, the fix is removing the Python level loop by batching every pair at a given length into one matrix operation over an `(n-delta, 4, 4)` array, which is exactly the class of workload a GPU array library accelerates well and a plain NumPy loop does not. Two general purpose GPU array libraries were evaluated for this workload, CuPy `[22]` and PyTorch `[23]`, since both are freely installable, NVIDIA GPU targeting and Python native, the two properties this stack's own analysis environment needs.

| Criterion | CuPy | PyTorch |
|---|---|---|
| API model | Near drop in NumPy and SciPy replacement, same function names and broadcasting rules, `cupy.linalg.inv`, `cupy.matmul` `[22]` | General tensor and autodiff framework, array ops exist but are wrapped in `torch.Tensor` semantics, default dtype float32, explicit `.to(device)` and `.numpy()` conversions needed `[23]` |
| Typical install size | About 150 MB per `cupy-cudaXXx` wheel `[22]` | About 2.3 GB per CUDA enabled wheel, roughly fifteen times larger, driven by bundled cuDNN, cuBLAS and NCCL and the autodiff engine, a size large enough that the PyTorch project itself tracks it as an open packaging concern `[23]` |
| numpy interoperability | Direct, `cupy.asarray(df.values)` and `cupy.asnumpy(x)` preserve dtype and shape exactly as NumPy would | Requires `torch.from_numpy` and `.numpy()`, and float64, the dtype every CSV in this stack loads as, must be cast explicitly since torch defaults new tensors to float32 |
| SE(3) and SO(3) utilities out of the box | None, batched 4x4 inverse, multiply and log map must be written from array primitives, as this section does below | Available through companion libraries, RoMa `[25]` and Kornia among them, though `kornia-slam`'s own issue tracker still lists batched trajectory ATE and RPE as an open, unimplemented item as of this survey `[24]`, so the ecosystem advantage is real but not yet a finished drop in metric |
| Autodiff and gradients | Not supported | Native, and irrelevant here, since every metric this document defines is evaluated once per run rather than optimised against, so the gradient machinery `torch.no_grad()` would otherwise need to suppress is pure overhead for this workload |
| Fit with this stack's own analysis stack | Matches `prompt5.md`'s own `DataReader` returning `dict[str, pd.DataFrame]` directly, `cupy.asarray(df.to_numpy())` is the entire adaptation needed | Would need `class Alignment` and `class TrajectoryAnalysis` written in tensor semantics from the start, a larger rewrite for the same closed form linear algebra |

The verdict is CuPy. The workload is pure batched array arithmetic on closed form linear algebra with no gradient ever taken, so PyTorch's autodiff engine and its far larger install are a cost paid for a capability this document never uses, while CuPy's near identical surface to NumPy lets `class Alignment` and `class TrajectoryAnalysis`, already committed to `pandas` and `numpy` by `prompt5.md`'s own `DataReader` design, move a computation to the GPU by changing an import and wrapping an array in `cupy.asarray` rather than by rewriting in a different framework's semantics. PyTorch's more mature SE(3) ecosystem, RoMa `[25]` and Kornia among them, would be the right choice if this pipeline ever needed to differentiate through a pose error, for instance to learn a correction, but no metric in this document does, so that advantage goes unused here. The implementation below is written against CuPy, with the same code running unmodified on a machine with no GPU by passing `xp=numpy` in place of `xp=cupy`, since every function below is written against a generic `xp` array namespace rather than hardcoding either library, `evo` and `rpg_trajectory_evaluation` both being CPU only precedents `[2][3]`, this stack's own contribution is exactly this batched, device agnostic reformulation of their shared pair definition.

```python
import numpy as np
try:
    import cupy as cp
    _HAVE_CUPY = True
except ImportError:
    cp = np
    _HAVE_CUPY = False


def _invert_se3_batch(T, xp):
    """Closed form batched SE(3) inverse, T: (N, 4, 4) -> (N, 4, 4). Cheaper
    than a general xp.linalg.inv since R is orthonormal by construction,
    T^-1 = [[R^T, -R^T t], [0, 1]]."""
    R = T[:, :3, :3]
    t = T[:, :3, 3:4]
    Rt = xp.swapaxes(R, 1, 2)
    Tinv = xp.zeros_like(T)
    Tinv[:, :3, :3] = Rt
    Tinv[:, :3, 3:4] = -xp.matmul(Rt, t)
    Tinv[:, 3, 3] = 1.0
    return Tinv


def relative_pose_error(T_est, T_gt, delta, xp=None, translation_only=True):
    """Timestamp wise Relative Pose Error at one fixed sub trajectory length,
    following evo's own per pair definition [2] and rpg_trajectory_evaluation's
    own practice of sweeping delta and averaging over every start index [3],
    batched into one array operation per delta rather than a Python loop
    over pairs. T_est, T_gt must already be ALIGNED SE(3) poses, (N, 4, 4)
    each, see class Alignment in section 2.1, this function performs no
    alignment of its own, matching evo's own separation of the two steps.

    Returns
        per_start_error : (N - delta,) array, entry i is the RPE of the sub
            trajectory starting at frame i, aligned one to one with
            t_ns[:-delta] for a drift rate over time plot.
        rmse : scalar RMSE over per_start_error, the single number each sub
            trajectory length contributes to the RPE vs length curve.
    """
    xp = xp or (cp if _HAVE_CUPY else np)
    T_est = xp.asarray(T_est)
    T_gt = xp.asarray(T_gt)
    n = T_est.shape[0]
    i0 = xp.arange(0, n - delta)
    i1 = i0 + delta
    rel_est = xp.matmul(_invert_se3_batch(T_est[i0], xp), T_est[i1])
    rel_gt = xp.matmul(_invert_se3_batch(T_gt[i0], xp), T_gt[i1])
    E = xp.matmul(_invert_se3_batch(rel_gt, xp), rel_est)
    if translation_only:
        per_start_error = xp.linalg.norm(E[:, :3, 3], axis=1)
    else:
        trace = xp.clip((E[:, 0, 0] + E[:, 1, 1] + E[:, 2, 2] - 1.0) / 2.0, -1.0, 1.0)
        per_start_error = xp.arccos(trace)
    rmse = xp.sqrt(xp.mean(per_start_error ** 2))
    return per_start_error, rmse


def relative_pose_error_sweep(T_est, T_gt, deltas, xp=None):
    """Runs relative_pose_error for every delta in deltas, returning the RPE
    vs sub trajectory length curve, section 2's headline plot, and every
    delta's own timestamp wise vector, the drift rate over time plot. The
    Python level loop here is bound by len(deltas), a few tens of lengths in
    practice, not by the trajectory length, which is the actual source of
    rpg_trajectory_evaluation's own reported slowness [3] and the reason this
    module batches every pair within one delta rather than only vectorising
    within a fixed pair."""
    xp = xp or (cp if _HAVE_CUPY else np)
    curve = []
    per_delta_series = {}
    for delta in deltas:
        per_start_error, rmse = relative_pose_error(T_est, T_gt, delta, xp=xp)
        curve.append((delta, float(rmse)))
        per_delta_series[delta] = cp.asnumpy(per_start_error) if (_HAVE_CUPY and xp is cp) else np.asarray(per_start_error)
    return curve, per_delta_series
```

Not every derived computation in this document needs this treatment. The pair count at one delta is `O(n)`, and it is only the product across a length sweep, `O(n L)`, that reaches the point where batching matters, and even that point is reached only by the RPE sweep among everything catalogued in this document, since every other derived quantity below resolves with a single sort or a single windowed reduction over one CSV's own row count rather than a cross product of two swept dimensions. The per section tables below mark each row's device accordingly, and the great majority read CPU.

### Section 2 quantities, plots and implementation instructions

| Quantity or metric | Plot or metric type | Description | Implementation instructions | Compute |
|---|---|---|---|---|
| Aligned ATE RMSE, translation | Summary scalar, one per run, feeds the section 9 cross run table | Umeyama aligned per frame position error, RMSE over the whole run, mode chosen per section 2.1 `[1][2]` | Read `gt_eval.csv`, drop rows with a blank ground truth pair, an unmatched frame per section 1's caveat, build `(N, 4, 4)` SE(3) arrays from `p_w_i_*`, `q_w_i_*` and `p_gt_*`, `q_gt_*`, call `align_sim3`, `align_se3` or `align_posyaw` matching this run's estimator mode, then RMSE the aligned translation residual | CPU, a few thousand rows even on the longest run |
| Error coloured trajectory | 3D line plot, colour mapped by per pose ATE | Localises where along the mission error concentrates, `evo`'s own signature plot `[2]` | Same aligned per frame residual as above, plotted as a 3D polyline with colour driven by residual magnitude, `matplotlib`'s `Line3DCollection` or an equivalent web plotting library primitive | CPU |
| RPE RMSE vs sub trajectory length | Line plot with a shaded standard deviation band, x axis length, y axis RMSE | Drift RATE rather than accumulated drift, `rpg_trajectory_evaluation`'s own signature plot `[3]` | `relative_pose_error_sweep` from section 2.2 over a length grid, for instance every ten keyframes up to the full mission length | GPU, CuPy, once the run exceeds a few thousand frames, see section 2.2 |
| RPE per start time, one length | Time series, x axis mission time, y axis RPE at that length | The literal drift rate over time plot the ablation study in `plans/vio_drift_analysis.md` needs, isolating WHEN drift accelerates rather than only how much accumulates by the end | `relative_pose_error` at the single length most relevant to the change under test, for instance matching `vio_max_kfs`'s own keyframe horizon | GPU, CuPy, for the sweep that produces every length, trivial to replot per length afterward on CPU |
| Yaw offset at ground truth seeded reinitialisation | Scalar card, one per run | Confirms the yaw silently corrected against ground truth at init, `Change 10` | Read `vio_init.csv`'s `yaw_offset_rad`, `-999.0` the sentinel meaning ground truth was unavailable at that seed | CPU |
| Closure error, scale factor, landing residual | Three scalar cards | Cheap drift proxies needing no alignment step at all, already reported in `vio_optical_flow_instrumentation.md` | First and last row of `gt_eval.csv` and `vio_state.csv` | CPU |
| Tracking success percentage, failures per kilometre, recovery time | Scalar and event log | KITTI style tracking robustness reporting `[4]`, distinct from geometric accuracy | Not a first class event in this stack today, open item, see section 10 | N/A |

## 3. Front end diagnostics, is the tracker seeing what it needs

Track length distribution, how many frames each feature survives before it is dropped, and the attrition curve, the fraction of tracks alive at frame N still alive at frame N plus k, are the standard first diagnostics in VINS-Mono `[6]`, VINS-Fusion and rpg_svo. This stack already computes the mapper side equivalent as the bucketed histogram in `map_tracks.csv`, and `Change 11` in `../plans/SLAM_logging.md` now adds the front end's own per track death event log, `of_track_death.csv`, giving every track's exact age at death and its birth timestamp, from which both the length distribution, ages grouped into buckets, and the survival curve, the complementary empirical CDF of age at death, follow directly without recomputing one from the other.

Reprojection error per frame, against the current motion estimate, is VINS-Mono's inlier gate `[6]`, a hard 1.5 pixel threshold in its published configuration, and tracking the full distribution rather than only the pass or fail count shows whether the front end runs comfortably under the threshold or hovers near it. The configuration values themselves, quoted from `VINS-Mono/config/euroc/euroc_config.yaml` and `vins_estimator/src/parameters.cpp` `[6]`:

```yaml
F_threshold: 1.0        # ransac threshold (pixel)
keyframe_parallax: 10.0 # keyframe selection threshold (pixel)
```

```cpp
MIN_PARALLAX = fsSettings["keyframe_parallax"];
MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;  // normalises to an angular measure
```

`F_threshold` is the fundamental matrix RANSAC inlier gate, `keyframe_parallax`, normalised by focal length into an angular quantity, is the minimum parallax a track needs before VINS-Mono considers it worth keyframing or triangulating, the same role `mpMaxCosParallax` already plays in this stack's own local mapper. Inlier to outlier ratio after the geometric verification is the earliest warning of an imminent tracking failure in VINS-Mono's own reporting `[6]`, since it degrades before ATE or RPE would show anything, because those only surface after the fact. Optical flow endpoint error against a known correspondence, once a landmark exists to reproject, is the more diagnostic version of the forward backward round trip distance this stack already gates on through `optical_flow_max_recovered_dist2`. `Change 12` in `../plans/SLAM_logging.md` now closes the reprojection error gap directly, computing the pre-optimisation residual, `vio_reproj_pre_opt.csv`, at the point a track is first matched to an existing landmark, using `BundleAdjustmentBase::computeProjections` against the IMU propagated prior rather than the post-optimisation pose, matching VINS-Mono's own before-the-solve philosophy. What this residual measures is squarely the front end's own tracking together with the VIO thread's own triangulation, since neither has yet been touched by the backend optimiser, hence the `_pre_opt` name rather than the unqualified `vio_reproj`, distinguishing it from a residual a future addition might compute once the solver has converged.

Parallax and baseline distribution at triangulation time is the direct predictor of monocular initialisation failure across the VIO literature `[6][7][12]`, low parallax produces an ill conditioned depth estimate independent of how good the tracker is. `vio_triang.csv` already logged `baseline_m_min` and `baseline_m_max` per keyframe, and `Change 13` in `../plans/SLAM_logging.md` now adds the parallax angle itself, the cosine of the angle between the two viewing rays to the same candidate, mirroring the identical formula `LocalMapper::setup_opt` already uses and cites as ORB-SLAM3's own parallax gate `[7]`, closing the gap between a linear baseline measure and a true angular parallax measure. Detection count and its spatial distribution, whether keypoints cluster in one region of the frame or spread across an occupancy grid, is the standard way to catch a degenerate all features in one corner failure mode, and this stack's own 192 track detection ceiling, recorded in `vio_optical_flow_instrumentation.md`, is produced by exactly this occupancy grid mechanism, only the grid occupancy over time is not itself visualised, only the resulting count, left as an open item in section 10.

### Section 3 quantities, plots and implementation instructions

| Quantity or metric | Plot or metric type | Description | Implementation instructions | Compute |
|---|---|---|---|---|
| Track length distribution | Histogram of age at death, buckets by frame count | The standard first front end diagnostic in VINS-Mono, VINS-Fusion and rpg_svo `[6]` | Read `of_track_death.csv`'s `age_frames` column, one row per track death, bucket directly | CPU, a single `pandas` histogram over 161996 rows in the second audited run |
| Track survival curve | Line plot, x axis frame count since birth, y axis fraction still alive | The complementary empirical CDF of age at death, shows attrition rate rather than only the final distribution `[6]` | Sort `of_track_death.csv`'s `age_frames`, compute `1 - ECDF(age_frames)` at each integer frame offset, `numpy.searchsorted` against a sorted age array is sufficient | CPU, one sort of 161996 rows |
| Death reason breakdown | Stacked bar or pie, one slice per `reason` | Separates forward backward tracking loss from a rejected recovery attempt, the two failure modes `of_frame.csv`'s own counters already distinguish at the frame level, now attributable per track | Group `of_track_death.csv` by `reason`, 0 forward fail, 1 backward fail, 2 recovery rejected | CPU |
| Reprojection error distribution, pre optimisation | Histogram or box plot per run, and a time series of `mean_px`/`p95_px` | VINS-Mono's own inlier gate quantity, a hard 1.5 pixel threshold in its published configuration `[6]`, tracked here as a full distribution rather than a pass or fail count | Read `vio_reproj_pre_opt.csv` directly, `mean_px`, `median_px`, `p95_px`, `max_px` are already the aggregated statistics, no further per point data is logged, `Change 12` | CPU |
| Inlier to outlier ratio after geometric verification | Time series, fraction | VINS-Mono's earliest warning of an imminent tracking failure, degrading before ATE or RPE would show anything `[6]` | Proxy this stack's own way, `of_frame.csv`'s `tracked / (tracked + epi_rej)` per frame, the epipolar rejection gate playing the role of VINS-Mono's fundamental matrix RANSAC inlier test | CPU |
| Optical flow endpoint error, forward backward recovery | Time series and histogram | The more diagnostic version of the forward backward round trip distance this stack already gates on through `optical_flow_max_recovered_dist2` `[6]` | `of_frame.csv`'s `recov_rej` count per frame for the gated version, `flow_px_mean`/`flow_px_max` for the underlying magnitude this stack already logs | CPU |
| Parallax angle at triangulation | Histogram and time series, x axis frame or keyframe, y axis `cos_parallax_mean` | Direct predictor of monocular initialisation failure, ill conditioned depth independent of tracker quality `[6][7][12]` | `vio_triang.csv`'s `cos_parallax_min`/`mean`/`max`, `Change 13`, plot `arccos` of the mean for a degree scale axis if preferred over the raw cosine | CPU |
| Baseline distribution at triangulation | Histogram, metres | Linear complement to the angular parallax measure above | `vio_triang.csv`'s `baseline_m_min`/`baseline_m_max` | CPU |
| Mapper side track length histogram | Bucketed bar chart, already binned at logging time | The mapper's own already existing equivalent of the front end track length distribution above, a different cadence and a different subsystem, joins as a second series on the same plot | `map_tracks.csv`'s `len_hist_0` through `len_hist_9`, index i is length i+2, index 9 accumulates 11 and above, summed across the run or shown per cycle | CPU |
| Detection count per frame | Time series | Coarse proxy for tracker health when no track is yet available to measure reprojection error against | `of_frame.csv`'s `detected` column | CPU |
| Occupancy grid over time | Heatmap animation or small multiples, one cell per detection grid bucket | Would catch a degenerate all features in one corner failure mode directly rather than only through the resulting count | Not logged today, the occupancy grid this stack's own 192 track detection ceiling is produced by is computed and discarded per frame, open item, see section 10 | N/A |

## 4. Back end diagnostics, is the optimiser healthy and is the problem observable

The cost or residual convergence curve, total cost per solver iteration, is the primary health signal in every bundle adjustment framework, and this stack already has it directly in `vio_linearise.csv`'s `error_total` column per inner iteration. The Levenberg Marquardt damping parameter, `lambda`, rises on a rejected step and falls on an accepted one, and plotting it alongside the accept or reject flag in `vio_solver_iter.csv` shows why a solve slowed, repeated rejection meaning a poor local linearisation rather than ordinary convergence. The accepted to rejected ratio across a whole run is a single comparable number between two configurations and needs no new logging, only an aggregation of the `status` column already present.

The Hessian or information matrix condition number and its eigenvalue spectrum is used as a degeneracy proxy in recent SLAM literature, and this stack already computes exactly this object for the marginalisation prior, `ev_min`, `ev_max`, `ev_negative` and `ev_condition` in `vio_marg_nullspace.csv`, described in section 1 and the subject of this session's `merge_sums` fix.

What the literature adds beyond the scalar condition number already logged is a check against the closed form visual-inertial observability result independently derived by Martinelli `[8][9]` and by Hesch, Kottas, Bowman and Roumeliotis through Lie derivative analysis `[10]`, that IMU biases, the three dimensional velocity and attitude about the two gravity perpendicular axes, roll and pitch, are observable, while global yaw and global position, four directions in total, are not, regardless of how the vehicle moves. A grounding caveat is owed here. Martinelli's own two papers `[8][9]` are paywalled by IEEE and Springer respectively, and this session could not retrieve a reproducible copy of his own closed form matrix construction, so the specific rank condition and its matrix are not reproduced below as a direct quotation, only cited by name so a reader with institutional access can verify them independently. What is reproduced below instead is the publicly documented, independently confirmed equivalent from the OpenVINS project `[11]`, whose First Estimate Jacobian documentation states the same four unobservable directions and gives their explicit nullspace basis, in the notation OpenVINS itself uses, orientation, position, velocity and feature blocks stacked as rows:

```
N = [  R_0^G g        0_{3x3}  ]
    [ -[p_0^G x] g     I_{3x3}  ]
    [ -[v_0^G x] g     0_{3x3}  ]
    [ -[p_f   x] g     I_{3x3}  ]
```

The left column is the yaw direction, the rotated gravity vector propagated through each state's own rotation, position and velocity blocks, the right 3x3 block is the three translation directions, an identity block wherever the corresponding state is position. The consistency condition an estimator's own linearisation should satisfy is `H_k Phi_(k,0) N = 0` for every timestep `k`, where `H_k` is that timestep's measurement Jacobian and `Phi_(k,0)` the state transition matrix back to the reference time `[11]`. This is directly analogous to what `vio_marg_nullspace.csv`'s `marg_ns` vector already reports numerically, and the missing analysis step, entirely Python side, is projecting `marg_ns`'s dominant response onto the yaw direction this basis predicts and confirming the remaining directions carry negligible response, a check against a known analytical answer rather than only watching a scalar condition number rise or fall.

Separately from which directions are structurally unobservable regardless of motion, Martinelli's closed form solution requires the vehicle to have undergone sufficient acceleration diversity, informally, motion that is not constant velocity, for scale, bias and the observable directions above to actually be recoverable from a finite window of data `[8][9]`. The precise rank condition on Martinelli's own matrix could not be verified this session for the reason given above, and per the explicit direction that produced this correction, Martinelli's own closed form metric is not to be implemented in this iteration at all, its description above remains in this document purely as the grounding a future session should read once his two papers `[8][9]` are obtained through institutional access, at which point his own rank condition can be reproduced and checked directly against `vio_marg_nullspace.csv`. The metric this document instead selects for actual implementation is the windowed sample covariance of the gravity compensated specific force direction, drawn from the secondary literature on closed form scale and velocity determination `[12]` and from standard VIO initialisation practice more broadly, chosen precisely because it needs no paywalled derivation to implement or to justify. Over a sliding window of typically one to two seconds, tens to a couple of hundred IMU samples at typical 100 to 200 Hz rates, compute the 3x3 covariance of the unit specific force direction after subtracting the estimated gravity vector, and check its rank or condition number, a well conditioned, full rank covariance meaning the accelerometer swept multiple directions during the window, good excitation, and a near singular one, for instance a hovering or constant velocity segment, meaning degenerate excitation regardless of how good the rest of the pipeline is. `Change 14` in `../plans/SLAM_logging.md` adds exactly the raw material this check needs, `accel_mean` and `gyro_mean` per frame in `vio_imu_frame.csv`, to be rotated into the world frame in Python using the per frame orientation already in `vio_state.csv` and the fixed gravity vector already in `vio_init.csv`, this excitation covariance check itself being explicitly this session's own synthesis grounded in the cited literature rather than a direct quotation of Martinelli's own derivation. Correction, 2026-09-26. This paragraph previously left Martinelli's closed form metric as an implied fallback pending paper access, alongside the excitation covariance proxy, phrasing since replaced because the session that produced this correction directed that Martinelli's metric is excluded from implementation entirely for now, description only, while the excitation covariance metric is the one actually queued for the implementation plan `prompt5.md` will write.

The cost split already logged per solver iteration in `vio_solver_iter.csv`, vision, imu, bias_g, bias_a and marg_prior, is the direct answer to whether the optimiser is being overruled by a dominant prior term rather than by the images, and `vio_optical_flow_instrumentation.md` already reports a 45 fold difference in median vision cost between a healthy and a diverged run using exactly this column. Solve time and inner iteration count are fully covered by `vio_solver_timing.csv`.

This stack runs two independent optimisers, the VIO estimator's own square root QR solve just discussed, and the local mapper's separate Schur complement bundle adjustment documented in `linearisation.md`, `mapper_linearise.csv`, `mapper_solver_iter.csv` and `mapper_iter_summary.csv` being its exact structural counterpart to `vio_linearise.csv`, `vio_solver_iter.csv` and `vio_solver_summary.csv`. The same health questions, is the cost decreasing, is the damping parameter well behaved, is the accept to reject ratio healthy, apply to both, and are tabled together below rather than as a second copy of this section, since the diagnostic reasoning is identical and only the subsystem producing the numbers differs.

### Section 4 quantities, plots and implementation instructions

| Quantity or metric | Plot or metric type | Description | Implementation instructions | Compute |
|---|---|---|---|---|
| VIO cost convergence curve | Line plot, x axis inner iteration, y axis `error_total` | Primary optimiser health signal in every bundle adjustment framework | `vio_linearise.csv`'s `error_total` per inner iteration, one line per `optimize()` call or overlaid across a run | CPU |
| VIO damping trace | Line plot, `lambda` alongside the `status` accept or reject flag | Rising `lambda` on repeated rejection means a poor local linearisation rather than ordinary convergence | `vio_solver_iter.csv`'s `lambda` and `status`, plotted on the same time axis | CPU |
| VIO accept to reject ratio | Scalar per run, feeds section 9 | A single comparable number between two configurations | `vio_solver_iter.csv`'s `status` column, `count(status == 0) / count(status)`, no new logging needed | CPU |
| VIO cost split by term | Stacked area or line plot, vision, imu, bias_g, bias_a, marg_prior | Answers whether the optimiser is overruled by a dominant prior term rather than by the images, `vio_optical_flow_instrumentation.md` already reports a 45 fold difference in median vision cost between a healthy and a diverged run using this column | `vio_solver_iter.csv`'s five cost columns, plotted per iteration or reduced to a per frame median | CPU |
| VIO solve outcome per frame | Categorical timeline, `converged`/`terminated`/`reason` | Whether each optimisation actually converged, and by which of the three documented reasons it stopped if not | `vio_solver_summary.csv`, `reason` 0 converged, 2 not converged and not terminated, 3 terminated without convergence | CPU |
| VIO marginalisation size | Time series, pose, state and keyframe counts marginalised versus kept | Tracks how aggressively the sliding window is being trimmed | `vio_marg.csv` | CPU |
| VIO marginalisation prior eigenspectrum | Line plot, `ev_min`/`ev_max`/`ev_condition` over time, and `marg_ns`'s 7 component vector as a stacked bar per marginalisation event | Degeneracy proxy for the marginalisation prior, this is the exact object the `merge_sums` crash this document's audited run hit while computing | `vio_marg_nullspace.csv`, gated on `vio_debug` or `vio_extended_logging` | CPU |
| VIO marginalisation prior against the analytical nullspace | Scalar per marginalisation event, projection magnitude onto each of the four predicted unobservable directions | The literature grounded consistency check beyond a bare condition number, `H_k Phi_(k,0) N = 0` `[11]`, confirming the dominant response in `marg_ns` lies along the predicted yaw and position directions and not elsewhere | Project `vio_marg_nullspace.csv`'s `marg_ns` onto the four columns of the `N` matrix quoted above, built per event from that event's own orientation in `vio_state.csv` and gravity in `vio_init.csv`, entirely new Python side derivation, no new logging needed | CPU, a 7x4 projection per event |
| VIO pose update acceptance | Time series, `applied`/`rejected` counts and `max_trans_err`/`max_rot_err` against their limits | The local mapper to estimator correction channel, a run that has diverged saturates at 100 percent rejected | `vio_pose_update.csv` | CPU |
| VIO association ratio and keyframe trigger | Time series, `connected_ratio` against `thresh`, `take_kf` as a marker overlay | The nominal keyframe rate driver, though in practice usually saturated, the real cadence set by the `vio_min_frames_after_kf` floor | `vio_assoc.csv` | CPU |
| VIO triangulation rejection funnel | Stacked bar per keyframe, seven named rejection reasons | Shows which gate, no prior observation, unproject failure, short baseline, non finite, behind camera, too close or low parallax, is actually filtering candidates on a given run | `vio_triang.csv`'s `no_prior_obs`, `unproject_fail`, `short_baseline`, `not_finite`, `behind`, `too_close`, `low_parallax` against `candidates`/`added`, `low_parallax` reads 0 throughout every run audited so far since `vio_min_triangulation_parallax_deg` is left at its inactive default | CPU |
| VIO landmark churn | Time series, `lost` count and `marg_lost` flag against `lmdb` size | The VIO side landmark database's own count, distinct from the mapper's, section 5's `map_tracks.csv`/`map_publish.csv` being the mapper side equivalent | `vio_landmarks.csv` | CPU |
| VIO initialisation sanity | Scalar card per `vio_init.csv` row, `tilt_deg` and `accel_norm`/`gyro_norm` at the seed frame | Confirms gravity alignment landed on a sane tilt rather than a degenerate one before trusting anything the optimiser subsequently produces | `vio_init.csv`, flag `tilt_deg` far from 0 as a suspect initial alignment | CPU |
| VIO state and divergence trace | Multi line time series, position norm, velocity norm, bias norms | The primary state and divergence trace, a diverged run is visible here before any derived metric confirms it | `vio_state.csv`'s `p_norm`, `v_norm`, `bg_norm`, `ba_norm` | CPU |
| Excitation covariance, gravity compensated specific force | Scalar or line plot per window, smallest eigenvalue or condition number of the windowed 3x3 covariance | This document's selected proxy for Martinelli's own acceleration diversity requirement `[8][9][12]`, Martinelli's own metric is description only in this document, not implemented, pending paper access, per the correction above | Rotate `vio_imu_frame.csv`'s `accel_mean` into the world frame using `vio_state.csv`'s per frame orientation, subtract `vio_init.csv`'s `g`, window over a sliding one to two second span, `numpy.cov` on the unit specific force direction within the window | CPU, small windows, no cross product with a swept parameter |
| Mapper cost convergence curve | Line plot, x axis inner iteration, y axis `total` cost, one line per `t_ns`/`opt_pass` group | The mapper's own Schur complement optimiser's direct counterpart to the VIO cost curve above, a structurally distinct solver per `linearisation.md` | `mapper_linearise.csv`'s `total`, `vision`, `rel_error`, `roll_pitch_error`, grouped by `t_ns` and `opt_pass`, `Change 23`, before plotting `iter` on the x axis, since `LocalMapper::MapLocally` calls `optimize()` twice per cycle and both calls restart `iter` at zero | CPU |
| Mapper damping and accept to reject ratio | Line plot and scalar, `lambda`/`f_diff`/`max_inc` and `status` | Same health question as the VIO solver, applied to the mapper's own optimiser, `inner_steps` reading 0 confirming a Gauss Newton step with no Levenberg Marquardt inner loop | `mapper_solver_iter.csv`, `mapper_iter_summary.csv`'s `inner_steps`, both grouped by `t_ns` and `opt_pass` for the same reason as the row above, an accept to reject ratio computed without that grouping would silently pool two distinct optimiser calls into one number | CPU |

## 5. Map quality and loop closure diagnostics

Landmark count and lifetime over time, and the co-visibility graph's edge weight, shared landmark count between two keyframes, and node degree, how many other keyframes a given keyframe shares points with, are the standard ORB-SLAM3 `[7]` and OpenVSLAM diagnostics, and this stack's `map_tracks.csv` and `map_covis.csv` already carry the raw material, the latter usable in full only with `mpVioDebugMode` on since the parity check that produces it runs both the incremental and the batch covisibility path.

Bag of Words retrieval precision and recall needs a fuller methodology than a single published number, since a real system's operating point comes from several stages compounding rather than from thresholding the raw score alone, the full pipeline below.

**Ground truth true match definition.** A query keyframe and a candidate keyframe are labelled a true positive if their ground truth poses, joined from `gt_eval.csv` by nearest timestamp, are within a fixed distance threshold and the pair is separated in time by more than a minimum gap, the second condition excluding the trivial case of matching a keyframe to one of its own immediate temporal neighbours, which this codebase's own adjacency based matching in `LocalMapper::MatchLocal` already handles separately and is not a loop closure at all `[14]`. The literature reports no single universal distance threshold, published values range from roughly 1.5 metres for tight indoor benchmarks to 10 to 25 metres for city scale outdoor ones, so the right number for this stack is one proportional to its own SITL flight scale, and the minimum temporal gap is naturally set to this stack's own `mpLocalMatchNeighbours` reach, since anything closer than that is already adjacency matched by construction rather than retrieved by Bag of Words at all. Precision and recall are then computed sweeping the acceptance threshold across `map_bow_candidate.csv`'s `score` column, `precision = TP / (TP + FP)`, `recall = TP / (TP + FN)`, and the threshold at which false positives first reach zero names the recall at that same threshold as the headline number a system reports.

**The multi stage pipeline real systems actually run.** DBoW2 and ORB-SLAM3's own loop closing `[7][13]` compound three independent stages rather than thresholding the raw score alone, quoted directly from `ORB_SLAM3/src/LoopClosing.cc` and `ORB_SLAM2/src/KeyFrameDatabase.cc` `[7]`. Stage one, raw retrieval, queries the top scoring candidates from the vocabulary database, with the acceptance floor itself adaptive, the lowest score among the current keyframe's own already known covisible neighbours, a self calibrating threshold rather than a fixed global constant. Stage two, temporal consistency, requires several consecutive keyframes to each independently detect a geometrically consistent match to the same candidate region before a loop is confirmed, `mnCovisibilityConsistencyTh = 3` in ORB-SLAM3's own source. Stage three, geometric verification, fits a similarity transform by RANSAC and only accepts a candidate whose inlier count clears a minimum, `nBoWInliers = 15` at 99 percent RANSAC confidence in ORB-SLAM3's own `Sim3Solver::SetRansacParameters` call. These three stages compounding, each trading recall for precision, is exactly why real systems report figures near 100 percent precision at moderate recall, raw top one DBoW2 retrieval alone sits at only 50 to 80 percent precision and recall before any of the later stages run `[13]`.

**The score itself.** `feature_detection_descriptors.md` already claims this stack's own HashBow score equals the DBoW2 L1 score, confirmed directly against DBoW2's own public source, `dorian3d/DBoW2/src/ScoringObject.cpp` `[13]`:

```cpp
// per common word index, accumulate:
score += fabs(vi - wi) - fabs(vi) - fabs(wi);
// final:
score = -score / 2.0;
```

which is algebraically `1 - 0.5 ||v - w||_L1` for L1 normalised Bag of Words vectors, scaled onto `[0, 1]`.

**What this stack now logs for it.** `map_bow_query.csv` recorded only the winning candidate's `best_score` and a count of candidates above threshold, which cannot support a threshold sweep since every rejected candidate's own score was invisible. `Change 15` in `../plans/SLAM_logging.md` adds `map_bow_candidate.csv`, one row per candidate the database returned regardless of whether it cleared the currently configured threshold, `query_kf`, `candidate_kf`, `score` and `above_thresh`, which is exactly the minimum needed for the Python analysis phase to join against `gt_eval.csv`, apply the true match rule above, and sweep `score` for the precision recall curve and the companion score histogram split by true against false, the diagnostic this document's first draft could describe only in outline. This stack's own local mapper today runs only stages one and three of the pipeline above, retrieval and RANSAC geometric verification through `map_pair_match.csv`, with no stage two temporal consistency grouping, a structural difference from ORB-SLAM3 worth stating plainly when the resulting precision recall curve is read, since it reflects a single stage pipeline's honest operating point rather than a three stage one's.

Keyframe culling rate, culled over created, and the redundancy ratio that triggers a cull are already fully logged in `map_cull.csv` and `map_cull_select.csv`, this is a plot what is already there item rather than a new instrumentation item, and directly comparable as a single number between two configurations.

### Section 5 quantities, plots and implementation instructions

| Quantity or metric | Plot or metric type | Description | Implementation instructions | Compute |
|---|---|---|---|---|
| BoW retrieval precision recall curve | Line plot, x axis recall, y axis precision, threshold swept | The full multi stage methodology this section derives, `[7][13][14]` | `map_bow_candidate.csv`'s `score` swept as threshold, true match rule joined from `gt_eval.csv` by nearest timestamp plus a minimum temporal gap set to `mpLocalMatchNeighbours`, `TP/(TP+FP)` and `TP/(TP+FN)` at each threshold | CPU, a single sort over 38854 rows in the second audited run |
| BoW score histogram, true against false | Overlaid histogram, two series | Companion diagnostic to the precision recall curve, shows separability of the score itself independent of any one threshold | Same joined table as above, split by the true match label, two `pandas` histograms | CPU |
| BoW raw retrieval yield | Time series, `db_hits`/`above_thresh` against `best_score` and `thresh` | The database's own per query summary, one row per query keyframe rather than one row per candidate | `map_bow_query.csv` | CPU |
| Pairwise match verification funnel | Stacked bar, `descriptor_matches` down to `ransac_inliers`, `stored` as the final accept flag | Shows where a candidate pair is lost, weak descriptor matching versus a failed RANSAC geometric check | `map_pair_match.csv`, joins to `map_bow_query.csv` and `map_match_summary.csv` by keyframe id, or directly on `t_ns` since `Change 21` in `../plans/SLAM_logging.md` added it as an explicit column rather than relying on `kf1` always equalling the invoking keyframe | CPU |
| Matching stage summary | Scalar per cycle, `pairs`/`neighbour_pairs`/`verification_attempts`/`inlier_matches`/`total_matches` | Per cycle roll up of the pairwise funnel above | `map_match_summary.csv`'s count columns, its `db_query`/`matching` millisecond columns belong to section 6 instead | CPU |
| Map building throughput | Time series, `exported`/`live_in_builder`/`matches`/`new_kfs`/`corners` | Distinct from the track length histogram this file also carries, discussed in section 3, this is the raw throughput of the track builder itself | `map_tracks.csv`'s non histogram columns | CPU |
| Mapper triangulation rejection funnel | Stacked bar, 13 named rejection reasons | The mapper side equivalent of section 4's VIO triangulation funnel, the direct diagnostic for the worst first triangulation pair defect in `keyframe_driven_local_mapping.md` | `map_setup_opt.csv`, `lmdb_before`/`lmdb_after` alongside the funnel for context | CPU |
| Covisibility graph | Node link graph or adjacency heatmap, edge weight shared landmark count, node degree keyframe count | Standard ORB-SLAM3 and OpenVSLAM map quality diagnostic `[7]` | `map_covis.csv`, only representative of steady state cost with `mpVioDebugMode` on, since the parity check it comes from runs both the incremental and the batch covisibility path deliberately | CPU |
| Culling redundancy calibration | Histogram, `observers_hist_0` through `9` | Calibrates `mpMinRedundantObservers` and the redundancy threshold directly against the observed distribution rather than by guessing | `map_cull_select.csv` | CPU |
| Keyframe culling rate | Scalar per run, culled over created, feeds section 9 | Already fully logged, a plot what already exists item | `map_cull.csv` and `map_cull_select.csv` | CPU |
| Landmark filter attrition | Time series, `lm_before`/`lm_after` against `min_obs` | Shows how much of the map a minimum observation count filter removes | `map_filter.csv` | CPU |
| Publish consistency | Time series, `points` against `lmdb_landmarks` | `keyframe_driven_local_mapping.md` flags a mismatch between these two as a known duplicate emission defect, this plot is the direct way to see it recur or resolve across configurations | `map_publish.csv` | CPU |
| Landmark rehost outcome funnel | Stacked bar, five outcomes and five reasons | The mechanism behind the two observation landmark rehost defect measured in `keyframe_driven_local_mapping.md`, where rehosting fails by construction at exactly two observations | `map_rehost.csv`, `outcome` and `reason` columns, 171353 rows in the first audited run and 168279 in the second, both well within a single `groupby().value_counts()` pass | CPU, a single pass over the largest file this stack produces |
| Culled keyframe summary | Time series, `victims`/`rehost_attempted`/`no_host` against `lm_before`/`lm_after` | Per culled keyframe roll up, joins to the rehost funnel above | `map_cull.csv` | CPU |
| VIO side landmark database size | Cross referenced against the mapper's own count above | Already tabled in section 4 as `vio_landmarks.csv`, repeated here only as a pointer since the two databases, VIO's and the mapper's, are easy to conflate | See section 4 | CPU |
| Stereo matching | N/A | `mapper_stereo.csv` is expected to remain empty on this stack's own monocular rig, `calib.intrinsics.size() < 2` short circuits stereo matching, a dashboard caveat rather than a logging gap, see section 10 | Skip plotting when the file is empty rather than treat it as a missing metric | N/A |

## 6. Timing and systems diagnostics

Per stage latency, already logged in `vio_timing.csv`, `of_timing.csv` and `map_cycle_timing.csv`, is best shown as a stacked bar or stacked area chart per frame, immediately showing which stage regressed when two configurations are compared. Tail latency, the 95th or 99th percentile rather than the mean, is standard robotics systems practice, for instance the ROS2 tracing and CARET line of work built on LTTng `[15]`, because a mean hides an occasional stall that still causes queue backpressure or a dropped frame, and a hyperparameter change frequently shifts the tail long before it shifts the mean. This needs no new logging at all, since every timing CSV already has one row per frame or per cycle across the whole run, percentiles are a post hoc aggregation over rows already collected. Queue depth against the sensor's nominal rate is already present as the `vision_queue` and `imu_queue` columns in `vio_timing.csv`, sustained growth over a run rather than momentary spikes is the signature that the pipeline is falling behind rather than merely busy.

### Section 6 quantities, plots and implementation instructions

| Quantity or metric | Plot or metric type | Description | Implementation instructions | Compute |
|---|---|---|---|---|
| Front end stage latency | Stacked area chart, `pyramid`/`track`/`detect_add`/`total` | Front end's own per stage breakdown | `of_timing.csv` | CPU |
| VIO stage latency | Stacked area chart, nine named stages | The full per frame latency breakdown, the direct input to a stacked stage latency chart, already the richest timing file in this stack | `vio_timing.csv` | CPU |
| VIO queue depth | Time series, `vision_queue`/`imu_queue` | Sustained growth rather than momentary spikes is the signature of the pipeline falling behind rather than merely busy | `vio_timing.csv` | CPU |
| VIO solver internal stage latency | Stacked area chart, `allocateLMB`/`linearizeProblem`/`performQR`/`get_dense_H_b`/`solve`/`backSubstitute`/`computerError2` | Mirrors the solve pipeline stages documented in `linearisation.md`, drills one level below `vio_timing.csv`'s own `opt_and_marg` stage | `vio_solver_timing.csv`, also carries `resident_memory`/`resident_memory_peak` as a companion memory time series, joinable on `t_ns` since `Change 19` added it | CPU |
| VIO marginalisation stage latency | Stacked area chart, `marg_linearize`/`marg_helper`/`marg`/`marg_log` | `measure` here is a documented always zero limitation, unavailable inside `marginalize()`, plot the remaining four stages only | `vio_marg_timing.csv`, its own invoking timestamp column renamed from `frame_id` to `t_ns` by `Change 20` for join key consistency with every other file | CPU |
| Mapper cycle latency and idle fraction | Stacked area chart, 15 named stages plus `queue_wait` | The mapper's measured 92 percent idle fraction, recorded in `vio_optical_flow_instrumentation.md`, is `queue_wait` divided by total cycle time, computed directly from this file | `map_cycle_timing.csv` | CPU |
| Mapper matching stage latency | Time series, `db_query`/`matching` milliseconds | The mapper's own retrieval and verification cost, separate from the count based summary tabled in section 5 | `map_match_summary.csv` | CPU |
| Detector latency and yield | Time series, `detection` ms against `corners_total` | The Shi Tomasi detector documented in `feature_detection_descriptors.md`, distinct from the FAST based front end detector already covered in section 3 | `mapper_detect.csv`, joinable on `t_ns` since `Change 22` added it | CPU |
| IMU ingestion health | Time series, `stamp_rate_hz`/`wall_rate_hz`/`max_gap`/`non_monotonic` | A healthy baseline sits near 167 Hz stamp rate with zero non monotonic samples and a sub 10 ms max gap, a transport level check upstream of anything the estimator itself can compensate for | `vio_imu_ingest.csv`, one row every 200 samples | CPU |
| IMU frame coverage and stretch | Time series, `coverage` ratio and `stretch`/`stretch_span` | `stretch` firing with `stretch_span` near one IMU period is quantisation residue rather than a real gap, a larger `stretch_span` is the real transport fault signature | `vio_imu_frame.csv` | CPU |
| Tail latency percentiles, every stage | Scalar table, p95 and p99 per stage, feeds section 9 | Standard robotics systems practice `[15]`, a mean hides an occasional stall that a hyperparameter change shifts before the mean moves at all | `numpy.percentile` over any of the timing CSVs above, a post hoc aggregation needing no new logging | CPU |

## 7. Non-standard, research grade diagnostics

Normalized Estimation Error Squared, NEES, `(1/(m d)) sum_i e_i^T P_i^{-1} e_i` over a d dimensional state error `e_i` against ground truth and the estimator's own reported covariance `P_i`, is the standard filter consistency check `[16]`, a well tuned estimator averages NEES near one, chi square distributed under `d` degrees of freedom, and a value far from one shows the estimator is over or under confident in a way raw error alone would not reveal. This stack has no covariance to feed it today, `linearisation.md` records that `MargLinData::H` is a square root factor rather than an inverted covariance, and nothing in the estimator currently extracts or logs a per state covariance from it, so NEES is a genuine gap rather than a plot what already exists item, and the smallest step toward it would be extracting a covariance from the already computed square root factor at the points where `vio_state.csv` is emitted. The innovation whiteness test, the Normalized Innovation Squared family, checks that the residual sequence itself behaves like white noise rather than a systematic, not yet catastrophic, bias, and shares the same covariance dependency `[16]`. Covariance ellipsoid visualisation along the trajectory is the standard companion plot to both and shares the same prerequisite.

Per explicit direction from the session that requested this survey, NEES, innovation whiteness and covariance ellipsoid visualisation are deferred, not implemented in this iteration's logging plan, and no covariance extraction is added to `plans/SLAM_logging.md`'s Change 10 through Change 15. This paragraph and the one above it remain as the grounding for whichever future session takes the deferred work up, so that session does not need to re survey the topic.

### Section 7 quantities, plots and implementation instructions, all deferred

| Quantity or metric | Plot or metric type | Description | Implementation instructions | Compute |
|---|---|---|---|---|
| Normalized Estimation Error Squared, NEES | Time series against the chi square bound for `d` degrees of freedom | Standard filter consistency check `[16]`, a well tuned estimator averages near one | Needs a per state covariance extracted from `MargLinData::H`'s square root factor, not currently extracted anywhere, deferred | N/A, deferred |
| Normalized Innovation Squared, NIS | Time series against the chi square bound | Innovation whiteness test, the residual sequence should behave like white noise `[16]` | Same covariance dependency as NEES above, deferred | N/A, deferred |
| Covariance ellipsoid along the trajectory | 3D overlay on the trajectory plot | Standard companion visualisation to both checks above | Same covariance dependency, deferred | N/A, deferred |

## 8. Visualisation tool precedents for the eventual dashboard layout

Foxglove Studio's panel taxonomy `[17]`, a 3D scene panel, an image panel with overlay, a plot panel for arbitrary numeric series, a raw message table for drill down, and a state transitions panel for a categorical timeline such as tracking status over time, is organised as a saved, shareable layout reopened by a team rather than an ad hoc one off plot, a design principle worth borrowing directly. `rerun.io` `[18]` instead organises every panel around one shared timeline cursor, so a 3D view, a time series view and an image view all scrub together, useful specifically if the dashboard should let a user jump to a frame and see the trajectory, the image and the timing panel move together. PlotJuggler `[19]` is built for dense multi series exploration of exactly this stack's own CSV output, handling thousands of series and millions of points, with a scriptable transform editor to derive a new series, a rolling RMSE for instance, from a logged one without rerunning the SLAM stack, and is worth prototyping against before committing to a bespoke dashboard. `evo`'s error coloured trajectory and its cross run box or violin plot `[2]`, both described in section 2, are the most directly reusable templates for this dashboard's own accuracy views. Pangolin, already the native live viewer for this stack per `visualiser_pipeline.md`, is best suited to a running session rather than offline cross run comparison, complementary to rather than a replacement for the dashboard this document is scoping.

### Section 8 precedents and the dashboard design choice each motivates

| Precedent | Panel or feature | Description | Design choice for this dashboard |
|---|---|---|---|
| Foxglove Studio `[17]` | Saved, shareable panel layout, one panel type per data shape | A layout is reopened by a team rather than rebuilt ad hoc | One tab per independent subsystem, per `prompt5.md`'s own requirement, saved as a static `dashboard.html` rather than a live session |
| rerun.io `[18]` | One shared timeline cursor across every panel | Scrubbing one panel moves the trajectory, image and timing panels together | Where interactivity allows it, tie every plot's x axis to `t_ns` so a zoom in one plot zooms the rest |
| PlotJuggler `[19]` | Dense multi series exploration, a scriptable transform editor | Built for exactly this stack's own CSV output, thousands of series, millions of points, derived series without rerunning SLAM | Prototype the per section tables above in PlotJuggler before committing to a bespoke web layout, `metrics.csv` and `figures/` are exactly what it would ingest |
| `evo` `[2]` | Error coloured trajectory, cross run box or violin plot | The most directly reusable templates for this dashboard's own accuracy views | Section 2's error coloured trajectory and section 9's cross run comparison both reuse these templates directly |
| Pangolin | Live 3D scene during a running session | Complementary to, not a replacement for, offline comparison | Out of scope for this dashboard, already covered by the existing live viewer |

## 9. Cross run comparison specifically

The stated goal is comparing two or more configurations, not evaluating one run in isolation, and the closest existing methodology is `evo_res`'s box or violin plot across saved results together with a comparison table `[2]`, though that stops scaling gracefully once more than a handful of configurations need comparing at once. Recent hyperparameter tuning and multi method benchmarking work, for instance the VIO-GO automatic tuning line of work `[21]` and the LaMaria unified multi method benchmark `[20]`, structures the comparison as one row per configuration and one column per metric, closer to a sortable and filterable spreadsheet than a single overlaid plot, and that structure is the recommended top level view for this dashboard, a summary table with ATE RMSE, RPE at a few lengths, tracking success rate, mean and tail latency per stage, keyframe and landmark counts, and the solver accept to reject ratio as columns, one row per run, with any single row expandable into the full per frame time series and histogram views described in sections 2 through 6.

### Section 9 quantities, plots and implementation instructions

| Quantity or metric | Plot or metric type | Description | Implementation instructions | Compute |
|---|---|---|---|---|
| Cross run summary table | Sortable, filterable table, one row per run, one column per metric | The top level view `[20][21]` recommends over a single overlaid plot once more than a handful of configurations need comparing | Written directly to `metrics.csv` per `prompt5.md`'s own output contract, one row appended per run processed by `class Analysis` | CPU |
| Cross run box or violin plot | One plot per metric, one box or violin per run | `evo_res`'s own cross run comparison style `[2]`, complements the summary table for a metric whose full distribution matters, not only its summary statistic | Any per frame column already tabled in sections 2 through 6, grouped by run | CPU |
| Experiment selection overlay | Interactive line plot, series togglable per run | `prompt5.md`'s own requirement, experiments can be selected or deselected to visualise only the runs under comparison | Any time series plot above, rendered with a legend click handler, for instance Plotly's own default legend toggle behaviour, one figure per subsystem tab | CPU |
| ATE RMSE column | Table column | Section 2 | See section 2's table | CPU |
| RPE RMSE at N lengths, columns | Table columns, one per length | Section 2 | See section 2's table, computed once via `relative_pose_error_sweep` and its `curve` return value read off at the N lengths chosen for the table | GPU for the sweep, CPU to read off the table values |
| Tracking success rate column | Table column | Open item, section 3 and section 10 | Not implementable until the tracking success event is logged | N/A |
| Mean and tail latency per stage, columns | Table columns | Section 6 | See section 6's table | CPU |
| Keyframe and landmark count columns | Table columns | Section 5 | See section 5's table | CPU |
| Solver accept to reject ratio column | Table column | Section 4 | See section 4's table | CPU |
| Configuration diff between runs | Side by side key value table, differing fields highlighted | Answers directly which hyperparameter change produced a difference seen elsewhere in the dashboard, the entire point of an ablation study | Diff each run's `config.json` and `calibration.json`, `Change 18`, against a chosen baseline run, a flat key comparison over parsed JSON | CPU |

## 10. Open items for the next iteration

Logged already, needs only a derived computation rather than new instrumentation, are aligned ATE and RPE from `gt_eval.csv` using the Umeyama function in section 2, tail latency percentiles from every timing CSV, the accept to reject ratio from `vio_solver_iter.csv`, culling and redundancy rates from `map_cull.csv` and `map_cull_select.csv`, and the cross run summary table described in section 9.

Now logged as of `plans/SLAM_logging.md`'s Change 10 through Change 18, confirmed executed and populated in the second audited run, `/ws/logs/2026-09-26_16-37-51/`, are the yaw alignment recorded at ground truth seeded initialisation, the front end's own track length and survival event log distinct from the mapper's `map_tracks.csv`, the pre-optimisation reprojection residual distribution, the VIO side triangulation parallax angle alongside the existing baseline extremes, the per frame accelerometer and gyroscope means that let the excitation covariance check in section 4 run, the per candidate Bag of Words score needed for the precision recall curve in section 5, the stationarity flag and its accelerometer and gyroscope deviation bounds from `Change 16`, the inactive by default triangulation parallax gate and its `low_parallax` rejection reason from `Change 17`, and the per run configuration and calibration snapshot from `Change 18`. Correction, 2026-09-26. This line previously read Change 10 through Change 15 and described the changes as awaiting execution, both now superseded, the full Change 10 through Change 18 range has since been implemented in the source tree and exercised end to end by the second audited run. `Change 19` through `Change 23`, the invoking timestamp on `vio_solver_timing.csv`, `vio_marg_timing.csv`, `map_pair_match.csv`, `mapper_detect.csv`, `mapper_stereo.csv`, `mapper_linearise.csv`, `mapper_solver_iter.csv` and `mapper_iter_summary.csv`, are implemented in the source tree as of this correction but not yet exercised by a live run, since both audited runs in this document predate them, the host build and a fresh flight are the outstanding verification step, the same one every prior batch of changes in this plan has needed before its own claim of correctness could be upgraded from implemented to confirmed.

Not logged and not planned in this iteration, in roughly increasing order of effort, are an occupancy grid snapshot over time rather than only its resulting count, and the per state covariance extraction that NEES, innovation whiteness and covariance ellipsoid visualisation in section 7 all depend on, explicitly deferred per the direction recorded there.

`mapper_stereo.csv` is expected to remain empty on a monocular rig and is not a defect. `map_covis.csv` is only representative of steady state cost with the debug flag on, since the parity check it comes from runs both covisibility paths deliberately. Both are dashboard caveats rather than logging gaps.

## 11. Coverage audit and the gravity and scale tab (added 2026-11-11)

An audit of `/ws/logs/slam/2026-11-11_19-54-39` against `analysis.py`, `metrics.py` and `trajectory_analysis.py` found four CSVs no tab loaded (`vio_gravity_scale`, `mapper_iter_summary`, and the empty `mapper_stereo` and `vo_stationary`) and 199 columns of loaded CSVs that no figure or scalar read, among them every column of `vio_gravity_scale.csv`, `vio_pose_update.max_trans_err` and `max_rot_err`, `vio_solver_summary.converged`, `vio_state.v_w_i` and the bias components (only `ba_norm` and `bg_norm` were drawn). `../plans/analysis_module.md` section 19 added a `gravity_scale` tab, a generic `raw_logs` tab and a lift off time reference. Three facts matter for later work.

1. The new physical metrics (`metrics.tilt_error_deg`, `yaw_error_deg`, `windowed_scale_ratio`, `position_jump`, `block_update_mask`, `bias_equivalent_tilt_deg`, `liftoff_t_ns`) use NumPy only, so they can be checked in the dev container, which on 2026-11-11 had NumPy but neither pandas nor SciPy, by executing their parsed source from `metrics.py` without importing the module. They reproduced the hand analysis of `dynamic_marginalisation.md` §11 exactly.
2. `vio_gravity_scale.tilt_step_deg` is an `acos` of a dot product, so held frames of the update cadence log about $10^{-6}$ degrees, not zero. Update frames are separated with a tolerance of $10^{-3}$ degrees on the tilt and $10^{-9}$ on `sigma_step`.
3. `gt_eval.csv` and `vio_state.csv` share frame timestamps, so they merge on `t_ns` with `metrics.align_frames_on_t_ns`. Both carry `p_w_i`, so only the needed columns are selected before the merge.
4. `AggregateStats` writes a CSV's header with its first flush (`logger.cpp:197`), so a table that never flushed a row is a zero byte file. Every run of 2026-11-11 has three (`mapper_stereo`, `vio_marg_nullspace`, `vo_stationary`), and a run still being recorded has more, for example `vio_init.csv` before its first flush. `pd.read_csv` raises `EmptyDataError` on such a file, which crashed `djinn start slam-analysis` on 2026-11-11 when the log directory held a run in progress. `DataReader._load_one` now treats a zero byte file as absent, as it already treated a missing or header only file. A scan of every CSV of the eight runs then present found no row longer than its header and no non numeric value, so no other parse failure is expected.
5. `vio_marg_nullspace.csv` is never written while the gravity and scale period is active, because `logMargNullspace` is called only under `(config.vio_debug || config.vio_extended_logging) && !mpGsActive` (`sqrt_keypoint_vio.cpp:363` and `:1584`), and with `vio_gs_duration_s` 7000 the period never ends. With the file absent in every run, `_analyse_vio_optimisation` built the `marg_ev_*` and `marg_nullspace_response` figures with no runs, and `visualisation._legend_outside` raised `IndexError` on a figure without a legend (2026-11-11). Both blocks now also require data, as the stationary tab's blocks already did, and `_legend_outside` leaves a legendless figure untouched. A static check on 2026-11-11 found no other CSV loaded by `analysis.py` that is empty in every run, and no indexed column name absent from every header.

## References

`[1]` J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, "A Benchmark for the Evaluation of RGB-D SLAM Systems," IROS 2012.

`[2]` M. Grupp, `evo`, Python package for the evaluation of odometry and SLAM, https://github.com/MichaelGrupp/evo, including its Metrics wiki page and `evo/core/geometry.py`'s `umeyama_alignment`.

`[3]` Z. Zhang, D. Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry," IROS 2018, and the accompanying `uzh-rpg/rpg_trajectory_evaluation` toolbox.

`[4]` KITTI Vision Benchmark Suite, Odometry evaluation, https://www.cvlibs.net/datasets/kitti/eval_odometry.php.

`[5]` D. Schubert, T. Goll, N. Demmel, V. Usenko, J. Stückler, D. Cremers, "The TUM VI Benchmark for Evaluating Visual-Inertial Odometry," IROS 2018, arXiv:1804.06120.

`[6]` T. Qin, P. Li, S. Shen, "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator," IEEE T-RO 34(4), 2018, and the `HKUST-Aerial-Robotics/VINS-Mono` repository, `config/euroc/euroc_config.yaml` and `vins_estimator/src/parameters.cpp`.

`[7]` C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, J. D. Tardós, "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM," IEEE T-RO 37(6), 2021, and the `UZ-SLAMLab/ORB_SLAM3` repository, `src/LoopClosing.cc`, with `ORB_SLAM2/src/KeyFrameDatabase.cc` for the adaptive minimum score.

`[8]` A. Martinelli, "Vision and IMU Data Fusion: Closed-Form Solutions for Attitude, Speed, Absolute Scale and Bias Determination," IEEE T-RO 28(1), 2012. Paywalled, not independently reproduced in this document, cited by name only, see section 4.

`[9]` A. Martinelli, "Closed-Form Solution of Visual-Inertial Structure from Motion," IJCV 106(2), 2014. Paywalled, not independently reproduced in this document, cited by name only, see section 4.

`[10]` J. A. Hesch, D. G. Kottas, S. L. Bowman, S. I. Roumeliotis, "Camera-IMU-based Localization: Observability Analysis and Consistency Improvement," IJRR 33(1), 2014.

`[11]` OpenVINS project, "First-Estimate Jacobians," https://docs.openvins.com/fej.html, cited here for the publicly reproducible nullspace basis used in section 4 in place of Martinelli's own paywalled derivation.

`[12]` L. Kneip, S. Weiss, R. Siegwart, closed-form absolute scale and velocity determination line of work, ICRA 2011, cited as secondary context for the acceleration diversity condition, not a substitute for `[8][9]`'s own rank condition.

`[13]` D. Gálvez-López, J. D. Tardós, "Bags of Binary Words for Fast Place Recognition in Image Sequences," IEEE T-RO 28(5), 2012, and the `dorian3d/DBoW2` repository, `src/ScoringObject.cpp`.

`[14]` S. Lowry, N. Sünderhauf, P. Newman, J. J. Leonard, D. Cox, P. Corke, M. J. Milford, "Visual Place Recognition: A Survey," IEEE T-RO 32(1), 2016.

`[15]` C. Bédard, P.-Y. Lajoie, G. Beltrame, M. Dagenais, "ros2_tracing: Multipurpose Low-Overhead Framework for Real-Time Tracing of ROS 2," arXiv:2201.00393, 2022.

`[16]` Kalman filter consistency and NEES/NIS based auto-tuning, arXiv:2306.07225, cited for the standard NEES/NIS formulation used in section 7.

`[17]` Foxglove Studio documentation, panel taxonomy, https://docs.foxglove.dev/docs/visualization/panels.

`[18]` rerun.io documentation, https://rerun.io/.

`[19]` PlotJuggler, `facontidavide/PlotJuggler` repository.

`[20]` LaMaria, city scale egocentric visual-inertial SLAM benchmark, arXiv:2509.26639, 2025.

`[21]` VIO-GO, automatic hyperparameter tuning for event based visual-inertial odometry, Frontiers in Robotics and AI, 2025.

`[22]` CuPy, NumPy and SciPy for GPU, https://docs.cupy.dev/, and `cupy-cuda12x` on PyPI, https://pypi.org/project/cupy-cuda12x/, cited in section 2.2 for its NumPy compatible API and wheel size.

`[23]` PyTorch, https://pytorch.org/, and `pytorch/pytorch` GitHub issue #17621, "Why torch wheel is so huge," cited in section 2.2 for its install size and tensor semantics.

`[24]` `kornia/kornia-slam` GitHub issue #2, "Trajectory evaluation metrics (ATE / RPE)," cited in section 2.2 as evidence that GPU native batched SE(3) trajectory metrics remain an open, unimplemented problem in the PyTorch robotics ecosystem rather than a solved, off the shelf one.

`[25]` RoMa, "A lightweight library to deal with 3D rotations in PyTorch," https://naver.github.io/roma/, cited in section 2.2 for PyTorch's SE(3)/SO(3) ecosystem.
