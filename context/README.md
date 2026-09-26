# Context Directory

This directory contains LLM-generated reference documents for the Basalt SLAM project.
These serve as long-term memory to avoid re-investigating the same questions.

## How to update these files (MANDATORY)

These documents are incremental. Update them by correction and addition only, never by rewriting. Every fact already recorded must survive an edit, whether or not it bears on the task in hand, because the accumulated derivations, worked examples, code references and caveats are the whole value of the store and no single session would regenerate them.

When a recorded fact turns out to be wrong, rewrite it in place so that the document states the correct value as the only reading offered, then record the correction alongside it, naming what the file previously said, when it changed and why the old value was wrong. Never leave the wrong value standing as the primary text and never tag a block as superseded, because that leaves a reader to adjudicate between two competing claims, which is the largest source of confusion in a store of this kind. The correction note is what preserves the history and lets a reader identify an artefact built from the old value, so it must name that value explicitly, but it is a short note attached to a correct statement rather than a second version of the section. After editing, diff against the previous revision and confirm that every substantive line that vanished was deliberately corrected or removed rather than accidentally lost.

## Documents

| File | Topic | Date |
|------|-------|------|
| `gt_slam_alignment.md` | GT vs SLAM alignment analysis, gravity-alignment explanation, frame geometry, two-part fix — EuRoC + TUM-VI; §8 the NED body-frame requirement for a new live GT source, cross-referenced against Project AirSim's `/actual_pose` NWU convention; §5e the SITL/ArduPilot NED mounting hitting `FromTwoVectors`' degenerate branch, with four real `T_w_i_init` captures showing an arbitrary ~160° horizontal heading spread while Z-up stays solid; §9 VO (`eSLAMType::VSLAM`, `BasaltSLAMNode`'s default mode) never gravity-aligns at all, `T_w_i_init` stays exactly identity for the estimator's whole life, so its world "up" is really just body-down on a level start; §10 the live ground truth ingestion of 2026-09-25, the six source types and their conversion to an ENU world with an FRD body, the frame gate that seeds the SLAM world from the first ground truth sample (VIO heading and position, VO the whole pose) so Z is always up, and the `gt_eval.csv` layout | 2026-07-09/11, 2026-09-19, 2026-09-25 |
| `euroc_coordinate_frames.md` | EuRoC dataset coordinate frame reference, GT column decoding | 2026-07-09 |
| `tumvi_coordinate_frames.md` | TUM-VI dataset coordinate frame reference, GT column decoding | 2026-07-11 |
| `vio_localmapper_correction_loop.md` | VIO drift vs local BA correction loop; why live trajectory is never corrected; gauge freedom in NfrMapper BA; path to fix; the 2026-08-23 conversion of the `out_state_queue` and `out_vis_queue` taps to `try_push` | 2026-07-11/2026-08-23 |
| `airsim_camera_extrinsics.md` | Project AirSim → Basalt T_imu_cam derivation; `origin`/`rpy-deg` z-y-x convention and pitch clamp; NED↔CV axis permutation; SE3 composition and inversion; front_center numerical result and quaternion check; proof the IMU sits at the parent-link origin; intrinsics; measured 7.42 Hz frame rate against a 19.6 Hz ceiling; the FLU extrinsic the bridge IMU topic would need | — |
| `imu_noise_parameters.md` | Project AirSim IMU noise params → Basalt, read directly in SI with no conversion; ARW/VRW theory and the external-source conversions; pure Wiener bias process and the σ_b/√τ mapping; current values, discrepancy table, and Part 7 on ArduPilot's added noise, 50 Hz filter and gyro drift | — |
| `ap_dds_imu_stream.md` | Live `/ap/imu/experimental/data` capture; NED frame confirmation; Basalt gravity auto-alignment path; the two ArduPilot stamp sources and why duplicate stamps are impossible at 167 Hz; the node's de-duplication filter; the inertial/visual epoch mismatch; outstanding measurements; the 2026-09-25 correction that AP_DDS writes the IMU quaternion with permuted fields, `(x, y, z, w) = (qw, qx, qy, qz)`, so it must be decoded as `Quaterniond(o.x, o.y, o.z, o.w)` | — |
| `vio_residuals_and_priors.md` | Basalt's actual IMU residual and its Jacobians read from `preintegration.h`, the non-Forster rotation convention, the left/right SO(3) Jacobian split, the initial marginalisation prior's nullspace-only anchoring and the `vio_init_ba_weight`/`vio_init_bg_weight` index swap, plus the `doc/VIO.md` and `doc/Marginalisation.md` split and renumbering | 2026-08-31 |
| `linearisation.md` | The `LinearizationBase` family read from source. Default `ABS_QR` selection, the confirmed `vio_init_ba_weight`/`vio_init_bg_weight` transposition with its authoritative index proof, why the prior needs no index remapping and where that is asserted, `MargLinData::H` being a square root factor and not a Hessian, the dense normal equation convention `H x + b = 0` with the solver negating its own increment and how to add a residual directly to it, the 2026-09-12 correction establishing that `vio_obs_huber_thresh` is in pixels and independent of `vio_obs_std_dev`, where the square root property is kept versus spent, the landmark block storage layout and the `numQ2rows` accounting subtlety, whitening as the precondition for QR, the dead and latent code inventory, the threading split, and the proof that local mapping does not use this framework | 2026-09-02/2026-09-12 |
| `visualiser_pipeline.md` | `class SlamVisualiser` internals. The two palettes and why `vis_utils.h` must not be edited, the `cam_color`/`state_color` collision that hides a current-frame highlight, `states.back()` being the live edge, the Pangolin `Plotter` default `$0..$9` series and the `GuiChanged` consume-on-read trap, the panel widget dispatch and `META_FLAG_READONLY`, latest-only cache semantics and why a vanished layer is always an empty payload rather than a drop, the frame-classification method used on the SITL recording, and the 2026-09-19 finding that the world-origin `glDrawAxis` triad is the only axes drawn and only its blue (Z) axis is a reliable reading | 2026-09-08, 2026-09-19 |
| `vio_optical_flow_instrumentation.md` | The estimator and front end read as a pipeline. The synchronous ingestion architecture that follows from `useProducerConsumerArchitecture=false`, the inertial ingestion contract in `ProcessFrame` and the zero order hold fallback that satisfies both assertions on an empty preintegration, assertions keying on `BASALT_DISABLE_ASSERTS` rather than `NDEBUG`, the dead `filterOutliers` call and the two inert configuration keys, monocular `filterPoints` being inert, the triangulation uncertainty derivation showing why the EuRoC `vio_min_triangulation_dist` does not transfer to a 212 m scene, the 2026-09-12 measured run statistics including the 16.4 Hz camera rate and the mapper's 92 percent idle fraction, why negative `error_total` is expected, the `[VIO]` and `[Optical Flow]` instrumentation record catalogue, and the instrumented capture that resolved the divergence, namely the healthy inertial and front end baselines, `flow_px_mean` as a scale oracle, the accelerometer bias runaway with its four structural conditions and its diagnostic sign, the bias observability derivation `T = sqrt(2 sigma_px d / (f b))`, why a large negative `marg_prior` and a negligible `imu` term are both expected rather than defects, the unbounded `mpPosesToUpdate` staging map, and the SLAM lifecycle activation policy with the `VideoLoggingDriver` arm-edge precedent, the two subclasses that share `_desired_state`, and the R1 to R5 rule set, the two log reduction scripts in `scripts/` with the four parsing behaviours the interleaved output forces, the 2,002 frame capture at 19.61 Hz with its keyframe and latency distributions, the parallax form of the triangulation threshold and the direct linear transform properties that make a degenerate solution pass the acceptance window, the 192 track detection ceiling the occupancy grid imposes, and the absence of any ground truth subscription in the SLAM stack, the tracker capture range `r * 2^levels = 28 px` and the takeoff transient that exceeds it, conditional degeneracy and the tilt against accelerometer bias confound with the proof that clamping one member relocates the error rather than removing it, the phantom map built from tracking noise before the vehicle moves, the three endpoint accuracy measurements that need no reference trajectory, and the two 2026-09-12 SITL flights of `/ws/log2.log` with their diagnostic thresholds, the `ExecutionStats` collector with the batch pipeline that depends on its two JSON artefacts, the `int64_t` overload that would make every existing `add` call ambiguous and the `double` ceiling that quantises UTC epoch nanosecond stamps to 256 ns, the `use_double` argument at `src/vio.cpp:312` that actually selects the threading model, and the 2026-09-14 `merge_sums` `std::out_of_range` crash, a dormant upstream defect in its `Eigen::VectorXd` no-op arm, first triggered live by the `Logger`'s scratch-and-merge design | 2026-09-12/2026-09-14 |
| `imu_static_calibration.md` | calib_accel_bias (9-param) and calib_gyro_bias (12-param) model; code usage; EuRoC vs SITL values; how to calibrate a real IMU | 2026-07-13 |
| `keyframe_driven_local_mapping.md` | The two local-mapper input queues after the 2026-09-06 driver change, `struct Keyframe` and its alignment requirement, where a keyframe is published and why the commit block rather than the threshold test, the proof that the marginalisation queue cannot deadlock the estimator, the culled-keyframe and factor-pruning invariants, the proof that estimator `frame_poses` is a subset of `kf_ids`, the dead `MargData::frame_states` loop, the four shutdown sentinel sites, the 2026-09-08 measured landmark annihilation loop with the two-observation track ceiling, the identically-0.5 redundancy ratio and the impossible rehost, the 2026-09-10 tracking failure where local matching depends solely on bag-of-words retrieval, and the 2026-09-12 `LandmarkDatabase` mutation-surface audit with the measured `CullRedundantKeyframes` cost distribution | 2026-09-06/2026-09-12 |
| `vio_config_json.md` | The cereal JSON configuration path. `CEREAL_NVP` as the origin of the `config.` key prefix, order-independent name lookup, extra keys inert and missing keys fatal with an uncaught `cereal::Exception`, the eight configuration sources including the batch `.toml` template that a JSON search misses, `size_t` overload resolution through the archive, the field naming convention, and the eight local mapper parameters exposed on 2026-09-12 | 2026-09-12 |
| `feature_detection_descriptors.md` | The mapping-path feature pipeline. Which of the two detectors in `keypoints.cpp` is actually used and why the FAST one is not, the derivation of `EDGE_THRESHOLD = 19` from the BRIEF pattern extent, the verification that the pattern tables are OpenCV's learned ORB table verbatim, the two deliberate deviations from reference ORB including the absent patch smoothing, the computed calibrations of `mapper_max_hamming_distance` and `mapper_bow_num_bits` with the word-splitting probabilities, the proof that the HashBow score is the DBoW2 L1 score, why detection is serial while retrieval is parallel, and the three instrumentable failure modes | 2026-09-12 |

## Quick Reference

### The 2026-09-12 SITL flights, in one paragraph (`vio_optical_flow_instrumentation.md`, `../plans/vio_drift_analysis.md`)
- The vehicle stands still for four seconds, no landmark can be triangulated, and the estimator's own **unconstrained position drift** crosses the 0.05 m threshold and manufactures a map at **60 to 120 m where the truth is under 1 m**
- Takeoff drives interframe flow to **41 px against a 28 px capture range**, the map is annihilated and rebuilt
- During the rebuild the estimator resolves the **tilt against bias confound** onto `b_a = 2.3 m/s^2`, that is **13.8 degrees**, and holds it for 72 s until the spline supplies horizontal acceleration at 78 s
- Marginalisation freezes the resulting error. The climb is over integrated by **1.73**, the descent is tracked at **1.13**, and the vehicle lands believing it is **78.6 m** in the air
- Clamping the accelerometer bias (`vio_init_bg_weight = 1e8`) does not help, it moves the error into the **gyroscope bias at 0.048 rad/s** and diverges to 41.5 km. Vision cost median goes 292 -> 13,003 and cheirality rejections 1 -> 214 per keyframe

### SITL Aerial Geometry, the numbers that decide everything (`vio_optical_flow_instrumentation.md`, `../plans/vio_drift_analysis.md`)
- Mission slant range is `121 / sin(45 deg) = 171.1 m`. Every depth dependent expression below uses it
- Depth uncertainty is `sigma_d / d = sigma_px * d / (f * b) = 0.2674 / b` at this range. The configured `vio_min_triangulation_dist = 0.05` m therefore gives **535 percent** depth error and 0.094 px of parallax
- A ten percent depth error needs `b = 2.67 m`, that is 5 px or 0.9 degrees of parallax. One keyframe interval at cruise gives 2.01 m, the whole seven keyframe window gives 12.1 m
- Accelerometer bias detection time is `T = sqrt(2 sigma_px d / (f b)) = sqrt(0.5348 / b)` s. At `vio_max_states = 3` the active window is **0.102 s**, which detects a bias of 51 m/s^2. The bias is unobservable at altitude, full stop
- On the ground the horizontal bias and the body tilt are **exactly confounded**, `delta_b = g * delta_theta`. The bias prior permits 0.1 m/s^2, that is 0.58 degrees, that is **179 m of position error after 60 s**
- The attitude seed is one accelerometer sample carrying `1.1768e-2 * sqrt(167) = 0.1521` m/s^2 per axis, which is **0.89 degrees** of tilt on its own
- `flow_px_mean` should equal `f v dt / d = 1.23` px per frame at cruise. It checks the camera, the intrinsics and the tracker, but it **cannot detect a scale error**, because a global rescaling changes `v` and `d` together
- Tracker capture range is `pattern_half_extent * 2^optical_flow_levels = 3.5 * 8 = 28` px. Measured survival knee sits at 16 to 20 px mean flow
- Accuracy without ground truth, since the mission lands where it took off, is the **closure error**, the **altitude scale factor** and the **residual altitude at landing**

### Live Visualiser Traps (`visualiser_pipeline.md`)
- `cam_color` and `state_color` in `include/basalt/utils/vis_utils.h` are the **same triple** `{250, 0, 26}`. A current-frame highlight drawn in one over states drawn in the other is invisible. Stock `src/vio.cpp:812-821` double-draws one frustum because of this
- `vis_utils.h` is shared with **six offline viewers**. Live-viewer colours belong in `include/basalt/visualisation/utils.h` instead
- `pangolin::Plotter`'s constructor installs ten default `$0..$9` series whenever handed a non-null log (`plotter.cpp:283-288`). Something must clear them before the first frame
- `pangolin::Var<T>::GuiChanged()` is a **consume-on-read edge detector**, not a state query. It can never establish an initial state
- A `Var<std::string>` renders as a live `TextInput` readout; `META_FLAG_READONLY` makes it display-only
- The latest-only caches are never reset, so a **vanished layer means an empty payload, never a dropped one**
- `states.back()` is the newest state, because `Eigen::aligned_map` is an ordered `std::map`

### Mapping Feature Pipeline Numbers (`feature_detection_descriptors.md`)
- `detectKeypointsMapping` is **Shi-Tomasi**, not FAST and not Harris. `goodFeaturesToTrack(img, pts, N, 0.01, 8)` leaves `useHarrisDetector=false`. The FAST detector in the same file is `detectKeypoints`, used only by the VO front end
- `EDGE_THRESHOLD = 19` is **derived**: pattern extent `[-13,12]` gives radius `sqrt(2)*13 = 18.38`, plus half a pixel of rounding. Adding a scale pyramid invalidates it
- `mapper_max_hamming_distance = 70` is 7.25 sd below the `Binomial(256,0.5)` mean of 128. `P(d_H<=70) = 1.3e-13`. Very conservative
- `mapper_bow_num_bits = 16`: a true match at Hamming distance 20 shares a word only **26%** of the time. That is why `mapper_frames_to_match_threshold` must be 0.04
- The HashBow score **is** the DBoW2 L1 score `1 - 0.5*||q-d||_1`, over an untrained LSH vocabulary. No vocabulary file needed
- `mapper_obs_std_dev = 0.25` is already **below** the `1/sqrt(12) = 0.289` pixel-quantisation floor. `cornerSubPix` is commented out at `keypoints.cpp:239-243`
- **No patch smoothing** before the BRIEF tests, which BRIEF requires. No scale pyramid anywhere. No gate on centroid magnitude, so `atan2(0,0)=0` passes silently
- Detection is **serial** since commit `5de98a1` (2026-06-25); only the `add_to_database` call was ever the race

### GT-SLAM Mismatch Fix (applies to both EuRoC and TUM-VI)
- **Root cause**: World frame origin/orientation mismatch (NOT camera-IMU extrinsics)
- **Fix**: SE(3) first-pose alignment applied in `dataset_io_euroc.h` at IO read time
- **Formula**: `T_gt_aligned[i] = T_gt[0].inverse() * T_gt[i]`

### EuRoC GT Frame
- `state_groundtruth_estimate0` → GT is `T_w_i` (body=IMU, T_BS=Identity confirmed)
- No camera-IMU extrinsics needed for this GT source
- `mocap0` (raw MoCap) → GT is in marker frame; needs `T_imu_marker` (but `T_imu_marker=I` in ds_calib)

### TUM-VI GT Frame
- GT file: `mocap0/data.csv` — **no `state_groundtruth_estimate0`** directory
- GT is `T_w_i` (already converted to IMU frame in EuRoC export)
- **Proof**: `dso/gt_imu.csv` has identical values and explicitly labels the IMU frame
- No sensor.yaml files; calibration via `tumvi_512_ds_calib.json`
- Only room sequences have full-trajectory GT
- 7-column format (no velocity/bias), ~120 Hz Vicon rate

### Downloaded Dataset
- EuRoC: `data/machine_hall/MH_01_easy/`, `data/machine_hall/MH_05_difficult/`
- TUM-VI: `data/TUM/dataset-room1_512_16/` (room1, 512×16 EuRoC export, 1.78 GB)

### Project AirSim SITL Calibration (`data/sitl_calib.json`)
- **T_imu_cam**: analytically derived and numerically verified — see `airsim_camera_extrinsics.md`
  - front_center (`"xyz": "0.5 0.0 0.1"`, `"rpy-deg": "0 -45 0"`): `qx=0.2706, qy=0.2706, qz=0.6533, qw=0.6533`
  - det=1, unit norm, optical axis 45° down from forward in NED
  - The IMU parses no `origin` and applies no lever-arm correction, so camera-to-body is camera-to-IMU exactly
- **Intrinsics**: `fx=fy=320, cx=320, cy=240` — correct for 90° HFOV at 640×480, pinhole, no distortion
- **Frame rate**: measured 7.42 Hz against a 19.6 Hz scene-tick ceiling, with intervals quantised to 51 ms and a worst gap of 1.64 s. `orb_slam3/config/Monocular/sitl.yaml:28` still declares `Camera.fps: 15` and overstates it twofold
- **IMU frame**: NED (`base_link_ned`), from ArduPilot AP_DDS, not the simulator's own IMU sensor — see `ap_dds_imu_stream.md`. No code change is needed to put the IMU in the frame Basalt expects
- **IMU noise**: three of four values are WRONG in the file — see `imu_noise_parameters.md`
  - `gyro_noise_std=5.818e-4` correct; `accel_noise_std` 1.70× too large; `gyro_bias_std` and `accel_bias_std` √2 too large
  - `imu_update_rate` is 167, correct, and equals the distinct-sample rate because no stamp ever repeats
- **calib_accel_bias / calib_gyro_bias**: ALL ZEROS for simulation — see `imu_static_calibration.md`
  - Correct because `turn-on-bias` is zero and no scale or misalignment error is modelled at all

### Project AirSim → Basalt IMU Conversions
Read from `core_sim/src/sensors/imu.cpp` and `core_sim/include/core_sim/sensors/imu.hpp`. Project AirSim reads SI directly and performs **no unit conversion whatever**, so the white-noise mappings are the identity.
- `gyro_noise_std  = gyroscope.angle-random-walk`  ← already rad/s/√Hz
- `accel_noise_std = accelerometer.velocity-random-walk`  ← already m/s²/√Hz, NOT mg and NOT m/s/√hr despite the name
- `gyro_bias_std   = gyroscope.bias-stability / √tau`  ← bias-stability already rad/s
- `accel_bias_std  = accelerometer.bias-stability / √tau`  ← bias-stability already m/s²
- The bias process is a plain Wiener random walk with coefficient σ_b/√τ. There is no mean-reversion term, so the FOGM factor √2 does not apply.
- No flag disables the noise; `ApplyNoiseModel` is called unconditionally at `imu.cpp:195`. Zero the parameters to get a clean IMU.
- Both an `accelerometer` and a `gyroscope` block must be present: `accelerometer_bias_stability_norm` is only assigned inside the loaders and is read uninitialised if the object is omitted.

### Timestamps on the ArduPilot IMU Topic
- Every `/ap/*` stamp comes from `AP_DDS_Client::update_topic(builtin_interfaces_msg_Time&)`, which uses the external `/clock` when `has_received_clock` is set and otherwise `AP::rtc().get_utc_usec()`.
- The 5 ms `AP_DDS_DELAY_IMU_TOPIC_MS` gate with a strict inequality gives a 6 ms period, exactly 166.667 Hz, measured with zero jitter across 69 consecutive stamps.
- Duplicate stamps are impossible on either branch, so the node's de-duplication filter at `node.cpp:190-211` drops nothing and `imu_update_rate` equals the message rate.
- **Open hazard**: a capture showed inertial stamps on a UTC epoch (~1.787e9 s) while camera stamps carry raw simulation time (~1.4e3 s), meaning `/clock` had not reached ArduPilot. Basalt cannot fuse streams with that offset. Verify with the epoch comparison in `ap_dds_imu_stream.md`.

### What Stands Between the Simulator and Basalt
Basalt subscribes to ArduPilot, not to the simulator. Three effects are added on the way, none configured by the simulator — see `imu_noise_parameters.md` Part 7.
- ArduPilot adds its own white noise at `AP_InertialSensor_SITL.cpp:114-120, 230-249`. Negligible, raising the densities by 1.0001 and 1.0005.
- `INS_ACCEL_FILTER` and `INS_GYRO_FILTER` are 50 Hz. Reduces visible scatter to 0.104 m/s² against the 0.152 the calibration implies, but leaves the preintegration covariance correct. Do **not** compensate.
- `gyro_drift()` at `:401-413` adds a deterministic 0.05 °/s per minute ramp on all three axes. This one matters: it exceeds the permitted bias excursion by 20× over a minute. Either set `SIM_DRIFT_SPEED` to 0 (preferred) or raise `gyro_bias_std` to 1.5586e-5.

### Basalt Gravity Convention
- World frame is Z-up, `g = (0,0,−9.81)` at `include/basalt/utils/imu_types.h:62`
- A NED (z-down) IMU is handled correctly, because `controller.cpp:163-167` routes an identity/zero init to the two-arg `initialize(bg, ba)`, leaving `initialized=false` so `sqrt_keypoint_vio.cpp:248` gravity-aligns via `FromTwoVectors(accel, UnitZ())`
- Side effect: the antiparallel case makes the initial heading arbitrary and non-reproducible between runs

### IMU Static Calibration Key Facts
- `calib_accel_bias`: 9 params `[b_x, b_y, b_z, s1–s6 (lower-triangular scale/misalignment)]`
- `calib_gyro_bias`: 12 params `[b_x, b_y, b_z, s1–s9 (full 3×3 scale/misalignment)]`
- Model: `x_calibrated = (I + S) · x_raw − b`  (applied per sample, `sqrt_keypoint_vio.cpp:226-230`
- Bias vector (first 3) seeds VIO initial dynamic bias: `basalt_slam.cpp:213-214`
- For SITL: all zero. For real hardware: run `basalt_calibrate_imu`
