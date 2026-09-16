# VIO Divergence Investigation and Fix

## Introduction

The visual inertial odometry estimate produced by `class SqrtKeypointVioEstimator` diverges from the commanded flight path within seconds of takeoff under the Project AirSim software in the loop stack. The commanded mission is a ten waypoint circuit flown at 150 m altitude over roughly 1012 m of ground track, while the estimated position reaches an order of 2x10^4 m over the same interval. The divergence is insensitive to the estimator configuration, which rules out a simple tuning deficiency and points at a structural failure in the inertial or visual constraint itself.

The local mapping thread is not implicated. Its point cloud renders as a coherent structure in the visualiser throughout the flight, and the timing evidence below shows it idle for the greater part of every cycle, so it can neither back pressure the estimator nor corrupt it. The failure is confined to the visual inertial estimator and the optical flow front end that feeds it.

The cause is established as the estimator acquireing a spurious accelerometer bias of roughly 3 m/s^2 within the first second of operation and then holding it for the remainder of the flight, so the propagated velocity and position are the single and double integral of a constant error that nothing in the problem can remove. The bias is acquired because the filter is seeded with zero velocity at an instant when the vehicle is already climbing under power, and it becomes permanent because the accelerometer bias ceases to be observable as soon as the scene depth grows beyond a few metres, which happens within about one second of takeoff. The full chain, with the measurements that establish each link, is set out under `Confirmed root cause`.

This document records what the video and the two logs establish, what the code establishes, which candidate causes survived and which were refuted, and the change that follows. Correction, 2026-09-12. The first revision of this document named an empty inertial preintegration as the primary candidate and left `Solution Implementation` empty pending instrumented logs. Those logs refute that candidate outright, since the preintegration is measurably complete, and the section is now filled.

## Evidences Analysed

### Video evidence

`/ws/evidence.webm` is a 9 m 54 s screen capture at 1920x1080, surveyed at 0.25 Hz into 150 frames. The quantitative readings are as follows.

| Survey frame | Wall time | Visualiser readout | QGroundControl readout |
|---|---|---|---|
| `f_0000` | 00:00 | SLAM window not yet open | Ready To Fly, ten waypoint mission loaded, terminal shows `djinn start sitl2 basalt > log.log` |
| `f_0024` | 01:36 | Tracked 828 points, `map_pts` 3954, `map_kfs` 30, `map_age` 0.5 s, plot range about [-10, +7] | not visible |
| `f_0030` | 01:59 | Tracked 869 points, `map_pts` 4016, `map_kfs` 32, `map_age` 0.4 s | not visible |
| `f_0035` | 02:19 | plot range 600 to 900, blue trace climbing 600 to 900 between plot time 25 s and 40 s | not visible |
| `f_0060` | 03:59 | Tracked 400 points, `map_pts` 457, `map_kfs` 29, plot range about +/-2000 | Flying, Auto, altitude 150.0 m, ground speed 12.9 m/s, distance 422.8 m, Vehicle Error "Potential Thrust Loss (2)" |
| `f_0090` | 05:59 | Tracked 69 points, `map_pts` 0, `map_kfs` 30, plot range +/-2000 with all three axes oscillating, image almost entirely dark | not visible |
| `f_0120` | 07:59 | Tracked 735 points of which nearly all are freshly created, `map_pts` 381, `map_kfs` 16, plot range reaching -20000 | not visible |
| `f_0149` | 09:54 | SLAM window closed | Flight Plan complete, total distance 1011.7 m |

Three readings carry most of the diagnostic weight.

The divergence is immediate rather than cumulative. At `f_0024`, only about eight seconds after the visualiser opens, the vertical trace has already left a plot range of ten metres, and the other two axes leave it within the first ten seconds of estimator time. Nothing in the record shows a period of correct tracking followed by decay.

The divergence is smooth and polynomial rather than noisy. Every trace is a clean monotone or single inflection curve on the scale of tens of seconds, which is the signature of an integrated constant rather than of accumulated visual error. Taking the vertical excursion of roughly 900 m at plot time 40 s gives an implied residual acceleration of `a = 2 x 900 / 40^2 = 1.1 m/s^2`, and the excursion of roughly 2x10^4 m at plot time 150 s gives `a = 2 x 2x10^4 / 150^2 = 1.8 m/s^2`. Both are of the order of ten to twenty percent of gravity, far above anything a simulated inertial sensor could contribute as bias or noise, and consistent with a gravity vector that is either misaligned or not being cancelled at all.

The map is sound while the trajectory is not. In the three dimensional panel the local map renders as a plausible spatially coherent cloud, while the estimated trajectory leaves it as a thin nearly straight ray. Structure and motion are therefore failing independently, which is only possible if the motion estimate is being driven by something other than the structure.

A secondary observation is that the tracked point count collapses from 869 to 69 around `f_0090`, where the captured image is almost entirely dark. The scene at 150 m altitude with the camera pitched 45 degrees down is low texture terrain, so the front end is operating near its limit for part of the flight.

### Log evidence, first capture

`/ws/log.log` contains 1,243,702 lines spanning six separate activations of the SLAM node, each identifiable by an `IMU Topic Name` line followed by a `Setting up filter` line. The configuration in use is `data/sitl_config_vo.json` with `config.vio_debug` set true, and the node runs as `monocular-inertial`, so `SlamMode::VIO` is selected at `ros_ws/src/slam/src/basalt/slam.cpp:35` and `use_imu` is true at `src/controller.cpp:158`.

The tag census establishes how asymmetric the existing instrumentation is.

| Tag | Occurrences |
|---|---|
| `[Local Mapper]` | 296,752 |
| `[rehost]` | 128,494 |
| `[LINEARIZE]` | 95,000 |
| `[ACCEPTED]` | 91,260 |
| `[EVAL]` | 82,498 |
| `[match]` | 52,236 |
| `[REJECTED]` | 49,394 |
| `[Mapper]` | 25,988 |
| `[VIO]` | 10,883 |
| `[cull-select]` | 10,428 |
| `[Optical Flow]` | 0 |

Before this pass the `[VIO]` tag existed at exactly three call sites in `src/vi_estimator/sqrt_keypoint_vio.cpp`, of which only `[VIO] processing new frame` ever fired, 10,880 times. The other two sit in `addIMUToQueue` and `addVisionToQueue`, which the synchronous ingestion path never calls because `Controller::GrabIMU` pushes to `imu_data_queue` directly at `src/controller.cpp:231`. The optical flow front end had no instrumentation whatever. Worse, the estimator's own optimiser output was untagged and used the same `[LINEARIZE]`, `[EVAL]`, `[ACCEPTED]` and `[REJECTED]` markers as the local mapper, so the two were indistinguishable by grep. Of the 95,000 `[LINEARIZE]` lines, 61,274 have the estimator's `Error: N num points` shape and 33,657 have the mapper's `iter N before_update_error` shape.

Run one occupies log lines 232 to 59,399 and begins with `Setting up filter: t_ns 139614000000`. Its measured characteristics are as follows.

| Quantity | Value |
|---|---|
| Estimator frames | 614 |
| Keyframes published to the mapper | 205, that is 33 percent of frames |
| Marginalisations | 610, of which 609 remove exactly one state |
| Keyframe marginalisations | 197 of 610 |
| Keyframe timestamp span | 37.368 s |
| Keyframe interval, mean | 183.2 ms |
| Keyframe interval, median | 162 ms |
| Keyframe interval, 90th percentile | 219 ms |
| Keyframe interval, maximum | 417 ms |
| Implied camera rate | about 16.4 Hz |
| Local mapper total cycle, mean | 0.550 s |
| Local mapper queue wait, mean | 0.507 s |
| Local mapper net work per keyframe | about 43 ms |
| `too large update in pose` rejections | 3,388 in run one, 72,884 across the log |

Two of these settle questions outright.

The local mapper spends 0.507 s of a 0.550 s cycle waiting on its input queue, so its net work is about 43 ms against a keyframe period of 183 ms. It is idle for ninety two percent of every cycle, the hundred slot keyframe queue set at `src/controller.cpp:178` can never fill, and the blocking `push` in `SqrtKeypointVioEstimator::PublishKeyframe` can never stall the estimator. The mapper is exonerated as a source of back pressure.

Every pose correction the mapper sends back is rejected. The `too large update in pose` branch in `SqrtKeypointVioEstimator::measure` fires 3,388 times in a run with 614 frames, which is roughly five rejections per frame, because the correction exceeds `kRelinThresholdTrans` of 0.10 m or `kRelinThresholdRot` of 0.05 rad. The estimator to mapper to estimator correction loop is therefore completely inert. This is a symptom rather than a cause, since it simply records that the two estimates disagree by more than a decimetre from the outset, but it does prove the loop cannot be the mechanism of divergence.

The landmark association statistics are the clearest visual signal in the existing log. The `connected0` and `unconnected0` counters report, per frame, how many current observations match a landmark already in the estimator's database and how many do not. Sampling every twenty fifth frame of run one gives the following sequence.

```
0/184  100/129  91/128  120/128  215/49  196/47  74/176  39/199  24/189
51/167  47/158  23/188  28/172  22/176  9/188  10/193  8/196  13/189
7/200  9/206  5/236  2/271  0/225  54/16  44/26
```

The connected count peaks at 215 early in the run and then decays into single digits while the unconnected count holds at roughly 200. The estimator is therefore receiving a steady supply of observations of which almost none attach to an existing landmark, which is to say feature tracks are not surviving long enough to be triangulated and reobserved.

Finally, the estimator's `error_total` swings between -2.8x10^5 and +1.1x10^4. Negative values are not in themselves a defect. `BundleAdjustmentBase::computeMargPriorError` at `src/vi_estimator/ba_base.cpp:477-502` deliberately drops the constant `0.5 r^T r` term from the marginalisation prior cost, and its own comment records that the result may be negative. The magnitude is nonetheless informative, because the retained term is `delta^T H^T (0.5 H delta + b)` and a value of 10^5 means the state has moved very far from the linearisation point the prior was built at.

### Instrumented log evidence, capture of 2026-09-12

The second capture of `/ws/log.log` holds 231,289 lines covering one activation, beginning `Setting up filter: t_ns 48822000000`. It carries 34,846 `[VIO]` records and 4,004 `[Optical Flow]` records against zero of either in the first capture. The measurements below are the ones that decide the case.

#### The inertial path is healthy

| Measurement | Value | Reading |
|---|---|---|
| `imu_minus_frame_ns` at initialisation | -69,000,000 | Both streams are in simulation time. The epoch mismatch recorded in `context/ap_dds_imu_stream.md` is resolved on this stack, and the residual offset is 69 ms, not 1.8e18 ns |
| `stamp_rate_hz` | 167.08 to 167.50 over 94 windows | Exactly the `AP_DDS_DELAY_IMU_TOPIC_MS` ceiling |
| `wall_rate_hz` | 55.8 | The simulation runs at one third of real time, which matches `real-time-update-rate` 9000000 against `step-ns` 3000000 |
| `non_monotonic` | 0 throughout | No duplicate or reordered stamps |
| `max_gap_ms` | 9 | One sample period of slack, no drop cascade |
| `integrated` against `expected` | 1,159 frames exact, 607 short by one, 82 long by one | The preintegration is filled from genuine samples |
| `coverage` | 0.94 to 1.00 | The frame interval is covered |
| `stretch` | 1 on 976 frames, always `stretch_ms` 3 | The zero order hold fallback closes one inertial period of quantisation residual, never a real gap |
| `imu_queue_in` | 1 to 28 | The 300 slot queue never approaches capacity |

Candidate C1 is refuted on every count it predicted.

#### The visual front end is healthy, and it independently confirms the true motion

Track survival over 1,999 frames has mean 0.828 and median 0.867, rising above 0.90 for most of the cruise. The dominant failure mode is `recov_rej`, the forward backward round trip gate, which is the intended behaviour of a conservative gate rather than a malfunction.

The decisive reading is `flow_px_mean`. Through the cruise it sits between 0.52 and 0.88 px per frame at the measured 19 Hz frame rate, that is 10 to 17 px/s. The predicted flow for the true flight is

```
flow = f * v / d = 320 * 12.9 / 212 = 19.5 px/s = 1.0 px per frame
```

using the QGroundControl ground speed of 12.9 m/s, the 212 m slant depth at 150 m altitude with the camera pitched 45 degrees down, and `fx = 320` from `data/sitl_calib.json`. Measurement and prediction agree to within ten percent. The images reaching the estimator therefore describe the real trajectory correctly, and the intrinsics are right. Candidate C3 is refuted as a cause, since a front end that is both surviving 87 percent of its tracks and reporting the physically correct flow is not the thing that is failing.

#### The estimated accelerometer bias is the divergence

| Time from initialisation | `\|p\|` (m) | `\|v\|` (m/s) | `\|ba\|` (m/s^2) |
|---|---|---|---|
| 0.00 | 0 | 0 | 0 |
| 0.31 | 0.144 | 0.734 | 0.0019 |
| 0.48 | 0.115 | 0.806 | 0.161 |
| 0.68 | 0.509 | 1.601 | 0.622 |
| 0.96 | 2.063 | 4.213 | 2.336 |
| 1.32 | 4.199 | 6.262 | 3.015 |
| 2.38 | 12.714 | 9.846 | 2.626 |
| 3.69 | 25.579 | 10.679 | 2.447 |
| 25.2 | 1,105 | 98.0 | 3.324 |
| 50.0 | 6,171 | 414.5 | 2.929 |
| 84.9 | 26,332 | 714.2 | 3.566 |

The bias rises from zero to 2.6 m/s^2 in the first second, and for the remaining eighty seconds it never leaves the band 2.4 to 3.6 m/s^2. By the end of the capture it has frozen completely, reading `ba = (-2.53128, 1.03963, 1.96361)` with `|ba| = 3.36808` identical to five decimal places across consecutive frames. The true bias of this simulated sensor is of order 1e-4 m/s^2, since `accel_bias_std` is 2.83e-4 and the flight lasts under two minutes, so the estimate is four orders of magnitude too large.

Velocity and position are the integral and double integral of that constant. The estimator reports 714 m/s where the vehicle flies at 12.9 m/s, and 26 km of travel where the mission covers 1012 m.

#### The inertial residual exerts no restoring force

The `[VIO][SPLIT]` records separate the cost for the first time. Late in the run a representative record reads

```
[VIO][SPLIT] vision=1746.56 imu=0.0330935 bias_g=4.65e-09 bias_a=2.22e-08
             marg_prior=-47326.6 total=-45580
```

Three things follow. The inertial residual is 0.033, that is numerically zero, so the diverged state satisfies the inertial measurements perfectly and the accelerometer offers no objection to it. The bias random walk term is 2.2e-08, so the bias is common mode across the window and its random walk penalty is inert. And the marginalisation prior exceeds the vision term by between 27 and 1000 times throughout the run, so the optimiser is driven by the prior rather than by the images.

The inertness of the random walk term is worth stating precisely, because it looks at first like a defect. `accel_bias_sqrt_weight` is `1/2.83e-4 = 3534`, and at `dt = 0.053 s` the effective weight is `3534/sqrt(0.053) = 15,352` per unit of bias change between consecutive states. A bias step of 0.13 between two states would cost `0.5 * (15352 * 0.13)^2 = 2.0e6`. The observed cost of 2.2e-08 therefore proves that the bias does not step between states inside the window. It moves as a single common value shared by all three states, and a common shift of the whole window costs the random walk term exactly nothing. The random walk constrains the bias rate, never its level.

#### The map inflates to match, so the vision never objects

`[VIO] triang` shows the triangulated depth tracking the pose divergence exactly.

| Time from initialisation | Max baseline used (m) | Mean triangulated depth (m) |
|---|---|---|
| 0.00 | 5.55e-17 | none created |
| 0.26 | 0.054 | 0.658 |
| 0.90 | 0.519 | 6.373 |
| 1.22 | 0.878 | 10.79 |
| 2.33 | 1.477 | 35.83 |
| 5.26 | 13.80 | 956 |
| 16.1 | 189.0 | 1,934 |
| 20.9 | 418.1 | 12,077 |
| 66.2 | 4,989 | 424,837 |
| 70.9 | 7,771 | 720,302 |

The scene is 212 m deep throughout. The estimator places it at up to 7.2e5 m. Because the poses and the map inflate together, every reprojection remains satisfiable and the vision residual stays in the low thousands rather than exploding. This is a pure scale drift, which bearing only measurements are structurally incapable of detecting.

#### The marginalisation is structurally sound

`checkMargNullspace` reports the three translations and yaw at machine zero throughout, between 1e-13 and 1e-9, while roll and pitch carry finite response, which is the correct signature for a gravity aligned visual inertial problem. The prior is not corrupt. Its large negative error is the first estimates Jacobian deviation `delta` measured against a position block weighted at `sqrt(vio_init_pose_weight) = 1e4`, so a deviation of 0.14 m from the linearisation point alone produces a residual norm of 1400 and a cost of order 1e6. The magnitude is a symptom of the state moving far from its linearisation points, not a cause.

#### The local mapper feedback path is inert, and it leaks

All 667 keyframes the mapper returned appear in the rejection list, and the rejection count is 13,770, so entries are rejected repeatedly rather than once. The median rejected `trans_err` is 6.996 m against a threshold of 0.10 m. The feedback loop applies nothing, which confirms the first revision's finding by direct measurement.

It also reveals a defect of its own. In `SqrtKeypointVioEstimator::measure`, an entry of `mpPosesToUpdate` is erased only when the correction is accepted. A rejected entry is retained, and an entry whose timestamp is no longer present in `frame_poses` is retained as well, so the map grows monotonically to the number of keyframes ever produced and is rescanned under its mutex on every frame. At 667 entries and 1,413 frames this costs little, but it is unbounded in a longer flight.

### Evidence that the local mapper changes are not responsible

The complete estimator side delta since commit `5c768c6`, which precedes both `Keyframe Selection Driven Local Mapping` and `Visualisation Bug Fixes`, is thirty four inserted lines and no deletions across `src/vi_estimator/sqrt_keypoint_vio.cpp` and `include/basalt/vi_estimator/sqrt_keypoint_vio.h`. Those lines are three `mpKFOutputQueue->push(nullptr)` shutdown sentinels, the `mpIsCurrentFrameKF` flag raised where the keyframe is committed, and `SqrtKeypointVioEstimator::PublishKeyframe`, which runs after `optimize()` and `marginalize()` inside `optimize_and_marg` and copies the already optimised pose and the stored optical flow result into the mapper's queue. It reads estimator state and writes none. The optical flow front end has no delta at all across the same range.

The one path that does write into estimator state is the `mpPosesToUpdate` block at the head of `SqrtKeypointVioEstimator::measure`, introduced by commit `286c4f3` `VIO Data Feedback Loop`, which predates the keyframe work. It writes only into `frame_poses`, never into `frame_states`, and as established above its guard rejects every update in practice, so it applies nothing.

The local mapper is therefore cleared on three independent grounds, namely that it does not back pressure, that its feedback is inert, and that it made no functional change to the estimator or the front end.

## Possible Root Causes

### Pipeline background, flow chart and studied implementation

#### Data flow

```
 ROS 2 MultiThreadedExecutor  (ros_ws/src/slam/src/basalt/driver.cpp:11)
 ├── mpImageCallbackGroup (MutuallyExclusive)        ├── mpImuCallbackGroup (MutuallyExclusive)
 │                                                   │
 │  /airsim_node/Copter/front_center_Scene/image      │  /ap/imu/experimental/data
 │  SensorDataQoS, BEST_EFFORT, KEEP_LAST(5)          │  SensorDataQoS, BEST_EFFORT, KEEP_LAST(5)
 │  stamp domain = AirSim simulation time             │  stamp domain = ArduPilot /clock or RTC
 │                                                    │
 ▼                                                    ▼
 BasaltSLAMNode::GrabImage  (node.cpp:188)            BasaltSLAMNode::GrabIMU  (node.cpp:218)
 │  cv_bridge -> MONO8, t = sec*1e9 + nsec            │  monotonicity and duplicate guard
 ▼                                                    ▼
 BasaltSLAM::TrackMonocular -> Controller::TrackMonocular (controller.cpp:205)
 │                                                    │
 │  SYNCHRONOUS. useProducerConsumerArchitecture=false │  Controller::GrabIMU (controller.cpp:229)
 │  (slam.cpp:53). The whole front end and estimator   │  pushes straight into
 │  run inside the image callback.                     │  imu_data_queue, capacity 300
 ▼                                                     ▼
 FrameToFrameOpticalFlow::processFrame                imu_data_queue (tbb bounded)
 │  build 3 level pyramid                                      │
 │  trackPoints  = inverse compositional LK on Pattern51,       │
 │                 forward then backward, keep if the round     │
 │                 trip error squared < 0.04 px^2               │
 │  addPoints    = FAST detect on a 20 px grid, 32x24 = 768     │
 │                 cells on 640x480                             │
 │  filterPoints = epipolar test, INERT in monocular            │
 ▼                                                              │
 OpticalFlowResult { t_ns, observations[0] : id -> 2x3 affine } │
 │                                                              │
 ▼                                                              ▼
 SqrtKeypointVioEstimator::ProcessFrame  (sqrt_keypoint_vio.cpp:205)
 │  (a) first call, pop one IMU sample, gravity align, seed state
 │  (b) per frame, build IntegratedImuMeasurement over
 │      [prev_frame->t_ns, curr_frame->t_ns] by draining the IMU
 │      queue NON BLOCKING, then close any residual gap with one
 │      relabelled sample
 ▼
 SqrtKeypointVioEstimator::measure  (sqrt_keypoint_vio.cpp:365)
 │  apply pending local mapper pose corrections, guarded
 │  predictState from the preintegration, append frame_state
 │  associate observations, count connected0 / unconnected0
 │  keyframe test, triangulate unconnected tracks
 │  collect lost landmarks
 ▼
 optimize_and_marg
 ├── optimize()      LM over ABS_QR, vision + IMU + marg prior
 ├── marginalize()   square root marginalisation, sliding window
 └── PublishKeyframe -> mpKFOutputQueue -> LocalMapper
 │
 ▼
 out_vis_queue -> SlamVisualiser        out_marg_queue -> LocalMapper
```

#### The inertial constraint

Basalt's inertial residual is built by `IntegratedImuMeasurement`, whose interval for frame `k` spans `[t_{k-1}, t_k]`. The linearisation at `include/basalt/linearization/imu_block.hpp:26-100` forms

```
r_imu   = sqrt_cov_inv * residual(state_{k-1}, g, state_k, bg, ba)
E_imu   = 0.5 * || r_imu ||^2
r_bg    = (gyro_bias_weight_sqrt / sqrt(dt)) * (bg_{k-1} - bg_k)
r_ba    = (accel_bias_weight_sqrt / sqrt(dt)) * (ba_{k-1} - ba_k)
```

with `dt = get_dt_ns() * 1e-9`. Two properties matter here. The bias random walk weights carry `1/sqrt(dt)`, so a degenerate interval makes them singular. More importantly, the residual is the only term in the whole problem that fixes metric scale and that cancels gravity. If the preintegration carries no real samples, the estimator becomes a monocular visual odometry system in which scale is unobservable and there is nothing to oppose an arbitrary drift of the world frame against gravity.

The residual also depends on the initial gravity alignment. `ProcessFrame` seeds the world frame from a single accelerometer sample by

```cpp
T_w_i_init.setQuaternion(
    Eigen::Quaternion<Scalar>::FromTwoVectors(imuData->accel, Vec3::UnitZ()));
```

with `constants::g = (0, 0, -9.81)` from `include/basalt/utils/imu_types.h:60`. The six initialisations recorded in the log all produce a rotation whose `(2,2)` entry is about -0.999998, so the measured specific force is close to `(0, 0, -9.8)` in the body frame, which is the expected reading for an accelerometer at rest in the ArduPilot NED body frame. `FromTwoVectors` is being asked for a rotation between two very nearly antiparallel vectors, where the rotation axis is not determined by the data, and indeed the recovered yaw differs arbitrarily between runs, taking first row first column values of 0.919, 0.652, 0.222, -0.439, -0.523 and -0.921. Yaw is a gauge freedom and this alone is harmless, but it confirms the alignment is being taken at a numerically delicate configuration from one unaveraged sample.

#### The visual constraint

New landmarks are created only on keyframes, by triangulating a currently unconnected track against an earlier observation held in `prev_opt_flow_res`. The guard is

```cpp
if (T_0_1.translation().squaredNorm() < min_triang_distance2) continue;
...
if (p0_triangulated.array().isFinite().all() &&
    p0_triangulated[3] > 0 && p0_triangulated[3] < 3.0) { ... }
```

where `min_triang_distance2` derives from `vio_min_triangulation_dist = 0.05` m and `p0_triangulated[3]` is inverse distance, so the depth acceptance window is `(0.33 m, infinity)`.

The relevant geometry is the parallax angle `alpha ~ b / d` for baseline `b` and depth `d`, and the propagated relative depth uncertainty for a two view triangulation, namely

```
sigma_d / d  =  sigma_px * d / (f * b)
```

With the flight regime read from QGroundControl and the calibration, `d` is about `150 / sin(45 deg) = 212 m`, `f = 320 px`, `sigma_px` is `vio_obs_std_dev = 0.5` px, and the per keyframe baseline is `12.9 m/s x 0.183 s = 2.36 m`. That gives `sigma_d / d = 0.5 x 212 / (320 x 2.36) = 0.14`, a fourteen percent depth error at the full keyframe baseline. At the 0.05 m floor the same expression gives a relative error of about 6.6, that is the depth is not determined at all. The threshold was tuned for EuRoC, where depths are two to ten metres and 0.05 m of baseline yields half a degree to two and a half degrees of parallax, and it does not transfer to a 212 m scene.

#### The optimisation and what removes bad data

The estimator solves a Levenberg Marquardt problem over `ABS_QR` in which `LinearizationAbsQR::linearizeProblem` at `src/linearization/linearization_abs_qr.cpp:185-265` accumulates the landmark blocks, then the IMU blocks when `imu_lin_data` is present, then the marginalisation prior. Only the Huber loss at `vio_obs_huber_thresh = 1.0` px moderates a bad observation. Nothing removes one, because `filterOutliers` is never called from the estimator, a fact recorded as a standing TODO at `src/vi_estimator/sqrt_keypoint_vio.cpp:1569`, and because `vio_outlier_threshold` and `vio_filter_iteration` are commented out of `struct VioConfig` at `include/basalt/utils/vio_config.h:68-69` and out of both the defaults and the serialiser at `src/utils/vio_config.cpp:71-72` and `:189-190`. The two corresponding keys in `data/sitl_config_vo.json` are consequently inert, since unknown keys are ignored by the cereal reader as recorded in `context/vio_config_json.md`.

The front end is equally unfiltered in this configuration. `FrameToFrameOpticalFlow::filterPoints` returns immediately when `calib.intrinsics.size() < 2`, so with one camera the epipolar test never runs and the only rejection mechanism in the entire visual path is the forward backward round trip check at `optical_flow_max_recovered_dist2 = 0.04`.

### Confirmed root cause

The divergence is a monocular scale and accelerometer bias runaway. It has one initiating error and three structural conditions that make that error permanent. Each link below is named with the measurement that establishes it.

#### The initiating error, the filter is seeded with zero velocity while the vehicle is already climbing

`basalt_slam_controller` activates the estimator only once the autopilot reports that the vehicle is airborne. The capture records the decision verbatim.

```
[basalt_controller]: ArduPilot armed=True flying=True mode=GUIDED
[basalt_controller]: Requesting ACTIVATE on 'basalt_slam_node' from INACTIVE,
                     the vehicle is FLYING, so SLAM must be ACTIVE
```

`BasaltSLAM::InitialiseSlam` at `ros_ws/src/slam/src/basalt/slam.cpp:47-54` then seeds the filter unconditionally.

```cpp
mpController->initialize(0,                        // t_ns: start at origin
                         Sophus::SE3d(),           // T_w_i: identity pose
                         Eigen::Vector3d::Zero(),  // vel_w_i: zero velocity
                         bg.cast<double>(), ba.cast<double>(),
                         false, useVisualisation);
```

`Controller::initialize` at `src/controller.cpp:167` sees a zero timestamp, an identity pose and a zero velocity, so it takes the gravity aligning branch `vio_estimator_->initialize(bg, ba)`, which hard codes `vel_w_i_init.setZero()` at `src/vi_estimator/sqrt_keypoint_vio.cpp:240`.

That the vehicle is not at rest is measured directly. The first accelerometer sample reads

```
[VIO] init: skipped_behind=12 accel= 0.0481421 -0.0627823 -11.1003 |accel|=11.1006
            gyro=-0.00237143 -0.00240529 0.00379908 |gyro|=0.00508351
```

A specific force of 11.1006 m/s^2 against a gravity magnitude of 9.81 is a net upward acceleration of 1.29 m/s^2, so the airframe is under power and gaining speed at the instant the estimator asserts that it is stationary. The direction is sound, since the lateral components place the measured vertical only 0.41 degrees from the body axis, so the gravity alignment itself is accurate and the seeded attitude is not the problem. The seeded velocity is.

#### Why the error is resolved into the bias rather than into the velocity

From the first optimisation onward the images demand more motion than an inertial chain seeded at rest predicts. The optimiser can supply that motion by raising the velocity or by raising the accelerometer bias, and the bias is the softer direction by two to four orders of magnitude. The velocity of every state in the window is tied to its neighbours by the inertial residual and to the marginalisation prior, whose position block carries `sqrt(vio_init_pose_weight) = 1e4`. The bias is tied to nothing comparable, because its random walk penalty sees only the difference between consecutive states and a common shift of the whole window is free, as the measured `bias_a` of 2.2e-08 proves.

The sign confirms the mechanism. The measured specific force is close to `(0, 0, -9.7)` in the NED body frame and the estimator forms `a_world = R (accel - ba) + g`, so a positive `ba_z` increases the magnitude of the body vector, which the alignment rotation maps to a larger upward world acceleration. If the mechanism is a velocity deficit the optimiser must therefore drive `ba_z` positive, and the first records of the runaway show exactly that, `ba = (0.0216, -0.0384, 0.5555)` at 0.63 s. The observed sign is the predicted one.

#### Structural condition one, the map scale is fixed by unconstrained dead reckoning

The first keyframe creates no landmarks at all.

```
[VIO] triang t_ns=48822000000 candidates=188 added=0 | short_baseline=188
      baseline_m[min=5.55112e-17 max=5.55112e-17]
```

The baseline is machine epsilon because the only observation available is the keyframe itself. The second keyframe falls at 49083000000, which is 261 ms later, because `vio_min_frames_after_kf` is 1 and the test is a strict inequality, so two intervening frames are required. Over that whole span `connected0` is zero, and `optimize` additionally returns immediately until `frame_states.size() > 4`. The first four frames therefore run as pure inertial dead reckoning with no visual constraint whatever, and the 80 landmarks created at the second keyframe are triangulated from a 0.054 m baseline that the dead reckoning alone produced. The metric scale of the entire map is set by that number, and the seeded velocity error is already inside it.

#### Structural condition two, the bias is not observable over the estimator's window

`vio_max_states` is 3, so the window holds two inertial factors spanning about 0.106 s at the measured 19 Hz frame rate. A bias `b` displaces the predicted position over a window of duration `T` by

```
dp = 0.5 * b * T^2 = 0.5 * 3 * 0.106^2 = 0.017 m
```

and that displacement is visible to the camera as

```
dpx = f * dp / d = 320 * 0.017 / d
```

which gives 7.8 px at a depth of 0.7 m, 0.54 px at 10 m and 0.027 px at 200 m, against an observation standard deviation of 0.5 px. The bias is therefore observable only while the scene is within roughly ten metres.

#### Structural condition three, the observability window closes in one second

The measured mean triangulated depth passes 6.4 m at 0.90 s, 10.8 m at 1.22 s and 35.8 m at 2.33 s as the vehicle climbs away from the ground. The bias reaches 2.34 m/s^2 at 0.96 s and 3.02 m/s^2 at 1.32 s. The error is therefore acquired and locked in during precisely the interval in which it is still detectable, and by the time the estimator has enough baseline to check it, the depth has grown far enough that a 3 m/s^2 bias is worth a fortieth of a pixel. Nothing afterwards can recover it, which is why the bias freezes and why the divergence is monotone rather than oscillatory.

#### The self consistency that hides the failure

Once the bias is locked, the poses inflate, new landmarks are triangulated from the inflated baselines and inherit the inflated scale, and every reprojection remains satisfiable. The vision residual stays in the low thousands while the trajectory passes 26 km. Bearing only measurements cannot observe scale, so the images never object. The `[VIO][SPLIT]` inertial residual of 0.033 confirms that the inertial measurements do not object either, because the bias has absorbed the whole discrepancy. This is the reason the reporter observes an intact map beside a diverged trajectory.

### Disposition of the candidates from the first revision

| Candidate | Disposition |
|---|---|
| C1, empty preintegration | Refuted. `coverage` 0.94 to 1.00, `integrated` equal to `expected` within one sample on every frame, queue depth 1 to 28 against a capacity of 300, stamps monotonic at 167 Hz in simulation time, `imu_minus_frame_ns` of -69 ms. The epoch mismatch that motivated this candidate has been resolved on this stack |
| C2, degenerate triangulation | Refuted as a cause and confirmed as an amplifier. Early triangulations are sound, with depths of 0.45 to 1.44 m at a 0.054 m baseline against a scene genuinely a metre away. The depths of 1e5 to 1e7 m seen later are downstream of the pose divergence, not upstream of it. `vio_min_triangulation_dist` remains mistuned for an aerial scene and admits near zero parallax pairs, which compounds the runaway once started |
| C3, track lifetime collapse | Refuted. Mean survival 0.828 and median 0.867, and `flow_px_mean` agrees with the physically predicted flow to within ten percent |
| C4, window too short | Confirmed as a necessary structural condition. Three states spanning 0.106 s is what makes the bias unobservable beyond ten metres of depth |
| C5, initialisation from one sample under acceleration | Confirmed, and identified as the initiating error, though not by the mechanism first proposed. Correction, 2026-09-12. The first revision expected a gravity alignment error of about 6.4 degrees. The measured alignment error is 0.41 degrees and is negligible. The damage is done instead by the zero velocity seed that accompanies the alignment, since the same sample proves the vehicle is accelerating at 1.29 m/s^2 and therefore already moving |
| C6, synchronous execution | Refuted as a contributor. The inertial queue never exceeds 28 of 300 slots and no sample is dropped, so the coupling this candidate predicted does not occur |

### Candidates considered and set aside

The local mapper, for the three independent reasons set out under `Evidences Analysed`.

The negative `error_total` values, which are expected behaviour of the square root marginalisation prior as documented in `src/vi_estimator/ba_base.cpp:486-490`, not a numerical defect.

A tuning deficiency as the whole explanation, since the reporter observes the same divergence across configurations and no parameter in the file changes the fact that the filter is seeded with a false velocity. Correction, 2026-09-12. This paragraph previously claimed that no parameter could account for the implied residual acceleration at all. That is too strong, because the anchor on the accelerometer bias is a parameter and it is what allows a 3 m/s^2 excursion, so Fix 2 is a parameter change that does remove the failure. The point that survives is that a parameter change alone treats the symptom, since the velocity error remains and merely has nowhere left to go.

## Instrumentation Added To Diagnose This

All new output is gated on `config.vio_debug`, carries either the `[VIO]` or the `[Optical Flow]` tag, and is written on one line per record so that the log can be reduced with `grep` and `awk`.

### Files touched

| File | Change |
|---|---|
| `include/basalt/optical_flow/frame_to_frame_optical_flow.h` | `struct OpticalFlowTrackStats`, an optional out parameter on `trackPoints`, `addPoints` and `filterPoints` now return counts, per frame and per stage logging in `processFrame` |
| `src/vi_estimator/sqrt_keypoint_vio.cpp` | inertial ingestion accounting in `ProcessFrame`, initialisation reporting, association and triangulation reporting in `measure`, state and latency reporting, error split and retagging in `optimize` and `marginalize` |
| `src/controller.cpp` and `include/basalt/controller.h` | inertial ingest rate, stamp domain and continuity reporting in `Controller::GrabIMU` |

### Backwards compatibility

`FrameToFrameOpticalFlow::trackPoints` gains a trailing `OpticalFlowTrackStats*` parameter defaulted to `nullptr`, so both existing call sites, the temporal one in `processFrame` and the stereo one in `addPoints`, keep their previous behaviour exactly. Only the temporal call passes a non null pointer, and only when `vio_debug` is set. `addPoints` and `filterPoints` change return type from `void` to `int`, which no caller consumes. `class MultiscaleFrameToFrameOpticalFlow` and `class PatchOpticalFlow` do not derive from `class FrameToFrameOpticalFlow` and are unaffected, so the blast radius is confined to the `frame_to_frame` front end that `data/sitl_config_vo.json` selects.

In `optimize`, `bool numerically_valid` and the three inertial error scalars are hoisted out of the inner blocks they were declared in so that the new lines can report them. No control flow changes. The retagged lines change text only.

`Controller::GrabIMU` gains six counters and one wall clock time point, all private, and emits one line every two hundred samples. The push into `imu_data_queue` is unchanged and still the last statement.

Every new statement is inside an `if (config.vio_debug)` or `if (vio_config_.vio_debug)` guard, so with the flag clear the only residual cost is the counter arithmetic in `Controller::GrabIMU`, which is itself guarded, and the `stats` null checks in `trackPoints`, which receive `nullptr` and are predicted trivially.

### New log records and how to read them

```
[VIO] ===== frame t_ns=<ns> prev_t_ns=<ns> initialized=<0|1> =====
```
Frame boundary marker. Everything between two of these belongs to one estimator step.

```
[VIO][imu-ingest] n=<count> t_ns=<ns> stamp_rate_hz=<f> wall_rate_hz=<f>
                  max_gap_ms=<f> non_monotonic=<count> queue=<n>
                  accel=<x y z> gyro=<x y z>
```
Emitted every two hundred inertial samples from the ROS callback. `t_ns` settles the stamp domain immediately, since a value in the low hundreds of billions is simulation time and a value near 1.79e18 is the real time clock epoch. `stamp_rate_hz` should read 167. A divergence between `stamp_rate_hz` and `wall_rate_hz` indicates the simulation is not running at real time. `max_gap_ms` above 6 indicates dropped samples, which at `KEEP_LAST(5)` means the consumer stalled.

```
[VIO] init: frame t_ns=<ns> imu t_ns=<ns> imu_minus_frame_ns=<ns> imu_queue=<n>
[VIO] init: skipped_behind=<n> accel=<x y z> |accel|=<f> gyro=<x y z> |gyro|=<f>
            bg=<x y z> ba=<x y z> g=<x y z>
```
The first line is the decisive epoch test. The second reports the sample the gravity alignment was taken from, where `|accel|` should be close to 9.81 if the airframe was at rest.

```
[VIO] imu frame t_ns=<ns> frame_dt_ms=<f> | integrated=<n> expected=<n>
      skipped_behind=<n> first_imu_ns=<ns> last_imu_ns=<ns> coverage=<f>
      | meas_dt_ms=<f> stretch=<0|1> stretch_ms=<f>
      | imu_queue_in=<n> imu_queue_out=<n> head_minus_frame_ns=<ns>
```
The single most important record. `expected` is `frame_dt * calib.imu_update_rate`. `coverage` is the fraction of the frame interval covered by genuine samples. `stretch` flags the zero order hold fallback and `stretch_ms` gives the interval it was asked to span.

```
[Optical Flow] frame=<n> t_ns=<ns> dt_ms=<f> | in=<n> attempted=<n> tracked=<n>
               fwd_fail=<n> bwd_fail=<n> recov_rej=<n> survival=<f>
               | detected=<n> epi_rej=<n> out=<n>
               | flow_px_mean=<f> flow_px_max=<f>
[Optical Flow] timing_ms pyramid=<f> track=<f> detect_add=<f> total=<f> queue=<n>
```
Track survival with the failure mode split, detection yield, and the flow magnitude that determines whether the pyramid depth is adequate.

```
[VIO] assoc t_ns=<ns> of_obs=<n> connected0=<n> unconnected0=<n>
      connected_ratio=<f> thresh=<f> frames_after_kf=<n> lmdb_landmarks=<n>
      kfs=<n> frame_states=<n> frame_poses=<n> take_kf=<0|1>
[VIO] triang t_ns=<ns> candidates=<n> added=<n> | no_prior_obs=<n>
      unproject_fail=<n> short_baseline=<n> not_finite=<n> behind=<n>
      too_close=<n> | min_triang_dist=<f> baseline_m[min=<f> max=<f>]
      depth_m[min=<f> mean=<f> max=<f>]
[VIO] lost_landmarks=<n> of lmdb=<n> marg_lost=<0|1>
```
Landmark bookkeeping. The `triang` record mirrors the local mapper's existing `[Local Mapper][setup_opt]` breakdown so the two front ends can be compared directly.

```
[VIO] state t_ns=<ns> p_w_i=<x y z> |p|=<f> v_w_i=<x y z> |v|=<f>
      bg=<x y z> ba=<x y z> |ba|=<f>
[VIO] timing_ms opt_and_marg=<f> measure=<f> vision_queue=<n> imu_queue=<n>
```
The divergence itself, per frame, in metres and metres per second, alongside the estimated biases.

```
[VIO][LINEARIZE] iter=<n> error_total=<f> lambda=<f> landmarks=<n> states=<n>
                 poses=<n> numerically_valid=<0|1>
[VIO][SPLIT] vision=<f> imu=<f> bias_g=<f> bias_a=<f> marg_prior=<f> total=<f>
[VIO]	[EVAL] ...
[VIO]	[ACCEPTED] ...
[VIO]	[REJECTED] ...
[VIO] iter=<n> backtracking
[VIO] solver terminated early ...
[VIO][marg] states_to_remove=<n> poses_to_marg=<n> states_to_marg=<n>
            state_to_marg_vel_bias=<n> kfs_to_marg=<n> kf_ids=<n>
[VIO][marg] keeping=<n> marg=<n> total=<n> last_state_to_marg=<ns>
            frame_poses=<n> frame_states=<n>
[VIO][marg] marginalisation done
[VIO] local-mapper pose update rejected t_ns=<ns> trans_err=<f> rot_err=<f>
      (limits <f>/<f>)
```
The `[VIO][SPLIT]` record is the second most important addition, because it is the first time the inertial contribution to the cost is separable from the visual one. A negligible `imu` term against `vision` means the inertial measurements are exerting no restoring force, which is what the capture shows and what identified the bias as the failing quantity. Correction, 2026-09-12. This paragraph previously read that such a reading would confirm C1, the empty preintegration hypothesis. It does not, because a preintegration built from a full complement of genuine samples is equally satisfied once the bias has absorbed the discrepancy, and the capture shows exactly that. A negligible `imu` term distinguishes a satisfied inertial constraint from a binding one, not a present measurement from an absent one, and `[VIO] imu frame` is what settles the latter. The retagging also resolves the grep collision with the local mapper, which previously shared every one of these markers.

### Reductions applied to the instrumented capture

These are the commands that produced the measurements in `Instrumented log evidence`. Note that `std::cout` is not synchronised across the estimator, mapper and front end threads, so a small fraction of records are interleaved mid line and must be filtered before parsing.

```bash
grep '\[VIO\] init'          log.log
grep '\[VIO\]\[imu-ingest\]' log.log | head -5
grep -oE 'integrated=[0-9]+ expected=[0-9]+' log.log | sort | uniq -c | sort -rn
grep -oE 'stretch=[01]'        log.log | sort | uniq -c
grep '\[VIO\] state t_ns='    log.log | grep -v '\[Local Mapper\]'
grep '\[VIO\]\[SPLIT\]'      log.log | tail -4
grep '\[VIO\] triang'         log.log | awk 'NR%30==1'
grep '\[Optical Flow\] frame=' log.log | grep -vE '\[Local Mapper\]|\[VIO\]'
grep -h -A8 'Marg nullspace'   log.log | grep 'nullspace '
```

## Solution Implementation

The fix has three parts. The first removes the initiating error, the second denies the error the soft direction it currently escapes into, and the third corrects a parameter that is mistuned for the flight regime and amplifies any scale error once one exists. The fourth item is an unrelated leak found during the investigation and is recorded so that it is not mistaken for an oversight.

| Change | State |
|---|---|
| Fix 1, activate SLAM on arm | Applied 2026-09-12 |
| Fix 2, anchor the accelerometer bias | Applied 2026-09-12 |
| Fix 3, triangulation baseline floor | Deferred, needs further investigation |
| Fix 4, `mpPosesToUpdate` leak | Deferred, needs further investigation |

Fix 1 and Fix 2 are recorded below as written. Fix 3 and Fix 4 remain specifications, and the reasoning for deferring them is that each rests on a judgement this capture cannot settle. Fix 3 needs a baseline floor that holds across takeoff, cruise and landing, and a single metric constant is the wrong shape for that, as its own section explains. Fix 4 changes behaviour in all builds, so it needs the marginalisation path re-read to confirm that a timestamp absent from `frame_poses` can never return. Neither is required for the divergence, since Fix 1 removes the initiating error and Fix 2 removes the direction it escaped into.

### Fix 1, activate SLAM on arm rather than on takeoff

The estimator asserts `vel_w_i = 0` at initialisation and takes its gravity alignment from a single accelerometer sample, and the lifecycle driver activates it only once the autopilot reports the vehicle airborne, so both assumptions are false by construction. The fix brings SLAM to ACTIVE as soon as the vehicle reports armed, which is the point at which those assumptions are true.

This is the policy the logging stack already follows. `VideoLoggingDriver::UpdateTargetState` at `ros_ws/src/controllers/src/logging/video_controller.cpp:177-193` targets ACTIVE on the rising edge of `armed` and does not consult `flying` at all.

```cpp
void VideoLoggingDriver::UpdateTargetState(bool currentArmed,
                                           bool currentFlying) {
    if (currentArmed && !this->mpPreviousArmedStatus) {
        RCLCPP_INFO(this->get_logger(),
                    "UAV is ARMED. Target for '%s' is ACTIVE.",
                    this->mpLifecycleNodeNameToManage.c_str());
        this->mpTargetState = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    } else if (currentFlying && currentArmed) {
        ...
```

SLAM is brought into line with it. The two stacks then come up together on the same event, which is also what makes a capture self consistent, since logging currently records an interval that SLAM does not cover.

#### Change 1a, widen the desired-state map

`SLAMDriverNode._desired_state` in `ros_ws/src/controllers/controllers/slam/driver_node.py`. Arming alone now implies ACTIVE. Rule R2 is widened to cover the armed-on-the-ground case, which subsumes R3 because SLAM is already ACTIVE there, and retires R4 because INACTIVE ceases to be a resting state for an armed vehicle. Rules R1 and R5 are untouched. The superseded branches are commented in place rather than deleted, because they encode the rule set a later reader will be comparing against.

```python
    def _desired_state(self) -> int:
        """Map the vehicle's phase to the lifecycle state SLAM should be in.

        The caller must hold `_state_lock`.

        ArduCopter forces land_complete (hence flying=False) whenever disarmed
        (ArduCopter/land_detector.cpp:53-55), so (armed=False, flying=True) is
        unreachable and two cases now cover the space. Rule R5 of the plan, "not
        armed and not flying implies UNCONFIGURED", is exactly rule R1 under that
        invariant and needs no branch of its own. The cleanup transition R5 calls
        for is supplied by the (INACTIVE, UNCONFIGURED) row of TRANSITION_STEP,
        which is also the only row that can ever emit a CLEANUP.

        Arming implies ACTIVE. The estimator seeds vel_w_i = 0 and takes its
        gravity alignment from one accelerometer sample, so it must come up while
        the vehicle is still at rest. This matches VideoLoggingDriver, which has
        always targeted ACTIVE on the rising edge of armed
        (controllers/src/logging/video_controller.cpp:179-183), so logging and
        SLAM now cover the same interval.
        """
        if not self._last_armed:
            return State.PRIMARY_STATE_UNCONFIGURED  # R1, and R5
        return State.PRIMARY_STATE_ACTIVE  # R2, widened 2026-09-12
```

#### Change 1b, correct the reason strings

`_phase_description` is the only record of why a transition was requested, so it must say what actually happened. The ACTIVE branch attributed an armed-on-the-ground activation to rule R3, which no longer exists. The INACTIVE branch is retained with a comment, because although `_desired_state` can no longer return INACTIVE as a target, a DEACTIVATE is still requested through the `(ACTIVE, UNCONFIGURED)` row of `TRANSITION_STEP` on the way to teardown.

```python
    def _phase_description(self, desired: int) -> str:
        """One human sentence explaining why a transition is being requested.

        The caller must hold `_state_lock`, because this reads the recorded
        vehicle phase. It performs no I/O, so holding the lock across it is free.
        """
        if desired == State.PRIMARY_STATE_ACTIVE:
            if self._last_flying:
                return "the vehicle is FLYING, so SLAM must be ACTIVE"
            return (
                "the vehicle is ARMED on the ground, so SLAM must be ACTIVE now, while the "
                "zero-velocity seed and the gravity alignment it performs at initialisation "
                "are still true (rule R2)"
            )
            # Superseded 2026-09-12 with rule R3, which no longer exists.
            # return (
            #     "the vehicle is ARMED on the ground and SLAM is already ACTIVE, so SLAM is "
            #     "held ACTIVE until disarm (rule R3)"
            # )

        # Retained although `_desired_state` no longer returns INACTIVE for an
        # armed vehicle, because a DEACTIVATE is still requested by the
        # (ACTIVE, UNCONFIGURED) row of TRANSITION_STEP on the way to teardown.
        if desired == State.PRIMARY_STATE_INACTIVE:
            return (
                "the vehicle is ARMED on the ground, so SLAM must be CONFIGURED and ready "
                "for takeoff"
            )
        if desired == State.PRIMARY_STATE_UNCONFIGURED:
            return (
                "the vehicle is DISARMED, so SLAM must be torn down and cleaned up for the "
                "next cycle"
            )
        return f"the target state is {STATE_NAMES.get(desired, desired)}"
```

#### What needs no change, and why

`TRANSITION_STEP` at lines 53 to 77 already carries the row `(PRIMARY_STATE_UNCONFIGURED, PRIMARY_STATE_ACTIVE): TRANSITION_CONFIGURE`, so a jump from UNCONFIGURED straight to a desired ACTIVE resolves to a CONFIGURE, after which `_reconcile` is re-entered by the next status message and issues the ACTIVATE from INACTIVE. `/ap/status` publishes on change and keeps alive at 2 Hz, so the second step follows within half a second. No row is added and none is removed. The `(ACTIVE, UNCONFIGURED)` row that supplies the teardown DEACTIVATE is likewise untouched, which is why the retained INACTIVE description above is still reachable as a transition reason even though it is no longer reachable as a target.

`_reconcile` at lines 611 to 668 is untouched. It takes at most one step toward whatever `_desired_state` returns, so widening the target is exactly the kind of change it was written to absorb. Its FINALIZED guard, its in-flight guard and its timeout are unaffected.

No launch file changes. The policy is unconditional, so `basalt_slam_test.launch.py` and every other launch file keep their current parameter blocks.

The Basalt C++ side needs no change at all. `BasaltSLAMNode::on_activate` at `ros_ws/src/slam/src/basalt/node.cpp:100-150` constructs the estimator and creates the subscriptions, and nothing in it depends on the vehicle being airborne.

Deliberately not done. An opt-in parameter that defaults to the old behaviour was considered, so that only the Basalt driver changed. It was rejected because the two estimators would then come up on different events for no stated reason, and because logging already activates on arm, so an arm-driven SLAM is the consistent policy for the stack rather than a Basalt-specific workaround.

#### Blast radius

`_desired_state` lives in the base `class SLAMDriverNode`, so the change reaches both subclasses.

| Consumer | Effect |
|---|---|
| `class BasaltSLAMDriver`, `controllers/slam/basalt_driver_node.py` | The intended change. SLAM becomes ACTIVE at arm instead of at takeoff |
| `class MonoDriver`, `controllers/slam/mono_driver_node.py`, ORB-SLAM3 | Also activates at arm. ORB-SLAM3 monocular has no inertial term, so a stationary start costs it nothing structurally, but its monocular initialiser requires parallax and will simply wait. This must be confirmed on a run rather than assumed |
| Tello vehicles | No effect in substance. `_tello_status_callback` at lines 518 to 536 maps `taking_off` to `(armed=True, flying=True)` and `landed` to `(False, False)`, so armed and flying flip together and no armed-on-the-ground state exists for that vehicle |
| `TRANSITION_STEP`, `_reconcile`, `_request_transition`, `_transition_event_callback`, `_query_state` | None. No row, guard or timeout is touched |
| Basalt C++ estimator, front end and ROS node | None |
| The controllers package rule documentation | Must be corrected in place. R2 is widened, R3 and R4 are retired, and a rule set that still lists them would contradict the code |

#### Consequences of a stationary start that must be checked rather than assumed

SLAM now processes images with no parallax for as long as the vehicle sits armed, which in the capture analysed here was 11.3 s between the CONFIGURE and the ACTIVATE. Four consequences follow and each is a validation item, not a claim.

Triangulation rejects everything on `short_baseline`, so `lmdb` stays empty. The first four frames of the current capture already exercise this path without incident, but eleven seconds of it is a longer exposure than anything yet observed. The `[VIO] triang` record reports it directly.

`take_kf` fires on every frame while `connected_ratio` is zero, so `kf_ids` grows at the frame rate until `vio_max_kfs` of 7 begins marginalising. Keyframes carrying no landmarks are published to the local mapper, which finds no matches and adds no factors. The mapper's `[setup_opt]` record already reports this case, so it is instrumented.

`optimize` becomes active once `frame_states.size() > 4`, four frames in, and then runs with an inertial residual and a marginalisation prior but no vision term. At rest this is the benign case, since a stationary vehicle is consistent with zero velocity and zero bias, and it gives the bias prior an interval in which it is not asked to absorb anything. This is the part of the fix that does the work.

Image ingestion begins earlier, so the `[Optical Flow] frame` counter and the keypoint identifier space start further back. Neither is bounded in a way that eleven seconds could threaten.

#### Recorded for completeness, the route not taken

Leaving the activation policy alone and supplying the missing velocity would mean `BasaltSLAM::InitialiseSlam` at `ros_ws/src/slam/src/basalt/slam.cpp:47-54` taking a measured velocity instead of a zero vector.

```cpp
    mpController->initialize(0,                        // t_ns: start at origin
                             Sophus::SE3d(),           // T_w_i: identity pose
                             Eigen::Vector3d::Zero(),  // vel_w_i: zero velocity
                             bg.cast<double>(),        // gyro bias from calibration
                             ba.cast<double>(),        // accel bias from calibration
                             false,                    // useProducerConsumerArchitecture
                             useVisualisation);
```

`BasaltSLAMNode` would have to subscribe to the autopilot twist topic, hold the most recent sample, and rotate it from the autopilot's NED world frame into the estimator's gravity-aligned world frame. That frame is defined by `FromTwoVectors(accel, UnitZ)` inside `ProcessFrame` and is therefore not known to the caller at the moment `initialize` is invoked, so the rotation cannot be applied where the velocity is supplied. It also leaves the second half of the problem standing, because the gravity alignment would still be taken from a single sample under 1.29 m/s^2 of acceleration. It is recorded so that a later reader does not reopen it as an obviously cheaper option.

### Fix 2, anchor the accelerometer bias at the strength the sensor actually warrants

The measured runaway is a common mode shift of the bias across the whole window, which the random walk term cannot see, so the only thing that resists it is the initial prior. That prior is currently `sqrt(1e2) = 10`, an implied standard deviation of 0.1 m/s^2, while the simulated sensor's true bias stability is `accel_bias_std = 2.83e-4` from `data/sitl_calib.json`. Matching the prior to the sensor makes a 3 m/s^2 excursion impossible.

```json
        "config.vio_init_pose_weight": 1e8,
        "config.vio_init_ba_weight": 1e1,
        "config.vio_init_bg_weight": 1e8,
```

JSON admits no comments and the cereal reader rejects them, so the superseded value of 1e2 cannot be recorded beside the new one in the file itself. It is recorded here instead, and the surrounding two lines are shown so that the edited line is identifiable in place.

The apparent misnaming is deliberate and must not be corrected here. The accelerometer bias occupies tangent indices 12 to 14 and the constructor at `src/vi_estimator/sqrt_keypoint_vio.cpp:94-98` applies `vio_init_bg_weight` to that block, so `vio_init_bg_weight` is the value that reaches the accelerometer. This transposition is an upstream defect confirmed twice, in `context/linearisation.md` and `context/vio_residuals_and_priors.md`, and is left standing so that comparison against historical runs stays valid. Anyone tuning these must target the block actually reached rather than the block named.

The derivation of 1e8 is that the prior is applied in square root form, so the effective weight is `sqrt(1e8) = 1e4` and the implied standard deviation is `1e-4 m/s^2`, which is the order of the true bias of this sensor over a two minute flight.

The index ordering was re-verified at source before the value was changed, rather than taken from the context store. `PoseVelBiasState::applyInc` at `thirdparty/basalt-headers/include/basalt/imu/imu_types.h` is authoritative.

```cpp
  /// @param[in] inc 15x1 increment vector [trans, rot, vel, bias_gyro,
  /// bias_accel]
  void applyInc(const VecN& inc) {
    PoseVelState<Scalar>::applyInc(inc.template head<9>());
    bias_gyro += inc.template segment<3>(9);
    bias_accel += inc.template segment<3>(12);
  }
```

`bias_accel` therefore occupies indices 12 to 14, which is the block the constructor fills from `vio_init_bg_weight`, so that key is the one that reaches the accelerometer.

Backwards compatibility. This is a value in `data/sitl_config_vo.json` only. `data/euroc_config.json`, `data/euroc_config_vo.json`, `data/euroc_config_no_factors.json`, `data/euroc_config_no_weights.json`, `data/tumvi_512_config.json`, `data/kitti_config.json` and `data/iccv21/basalt_batch_config.toml` were checked and all still carry 1e2, so every other dataset keeps the behaviour it has today. No code changes. `vio_init_bg_weight` is read at exactly two sites, the square root and squared branches of the `SqrtKeypointVioEstimator` constructor at `src/vi_estimator/sqrt_keypoint_vio.cpp:100` and `:112`, and nowhere else, so the blast radius is the initial marginalisation prior of the inertial estimator alone. `class SqrtKeypointVoEstimator` does not read it, since a visual-only estimator carries no biases. The change is specific to a simulated sensor whose bias is known to be negligible and must not be copied to a real IMU without recomputing the value from that sensor's datasheet.

### Fix 3, set the triangulation baseline floor from the scene depth (deferred)

`vio_min_triangulation_dist` is 0.05 m, an EuRoC value for a scene two to ten metres away. The propagated relative depth uncertainty of a two view triangulation is `sigma_d / d = sigma_px * d / (f * b)`, so holding that below ten percent at the cruise depth of 212 m with `f = 320` and `sigma_px = vio_obs_std_dev = 0.5` requires

```
b >= sigma_px * d / (f * 0.1) = 0.5 * 212 / (320 * 0.1) = 3.3 m
```

```json
"config.vio_min_triangulation_dist": 3.3,
```

Backwards compatibility. Again a value in `data/sitl_config_vo.json` only. It is second order relative to Fix 1 and Fix 2, since the instrumented capture shows early triangulation to be sound and the pathological depths to be downstream of the pose divergence, but it removes the amplification path by which a small scale error is converted into a large one.

Known defect left standing. A fixed metric floor is the wrong shape for the problem, because the requirement is a parallax angle and the correct floor therefore scales with scene depth. A criterion expressed in parallax would hold across takeoff, cruise and landing where a constant cannot. That is a larger change with its own blast radius across every configuration file, so it is recorded here and not taken now. A single value tuned for cruise will be too strict during the low altitude phase, where it will suppress triangulation exactly as the 0.05 m value currently suppresses nothing.

### Fix 4, stop `mpPosesToUpdate` from growing without bound (deferred)

In `SqrtKeypointVioEstimator::measure`, an entry of `mpPosesToUpdate` is erased only when its correction is accepted. A rejected entry is retained and re-examined on every subsequent frame, and an entry whose timestamp has left `frame_poses` entirely is retained forever. The capture shows 667 entries accumulating and 13,770 rejection evaluations across 1,413 frames.

```cpp
auto it_fp = frame_poses.find(t_ns);
if (it_fp == frame_poses.end()) {
    // The keyframe has left the sliding window, so no correction can ever
    // be applied to it. Dropping the entry bounds the staging map by the
    // window rather than by the flight duration.
    it = mpPosesToUpdate.erase(it);
    continue;
}
```

Backwards compatibility. The block is guarded by `config.vio_debug` for its logging only, not for its effect, so this changes behaviour in all builds. The change is nonetheless behaviour preserving in substance, because an entry whose timestamp is absent from `frame_poses` could never have been applied. `frame_poses` only loses a timestamp by marginalisation, which is irreversible, so no entry dropped here could have become applicable later.

### What is deliberately not changed

`vio_max_states` remains 3. Lengthening the window does improve bias observability, and the derivation is that detecting a bias `b` at depth `d` against a `sigma_px` noise floor needs a window of `T = sqrt(2 * sigma_px * d / (f * b))`, which for `b = 0.5 m/s^2` at `d = 212 m` is 1.15 s, or about 21 states at 19 Hz. That is a sevenfold increase in the size of the optimisation problem for a benefit that Fix 1 and Fix 2 obtain for nothing, and at 150 m altitude the accelerometer bias is close to unobservable at any practical window length. The correct answer in this regime is to avoid acquiring a bias error rather than to try to observe one.

The instrumentation added in the previous pass is retained in full. It is the only means by which this failure was distinguishable from the four other candidates, and it will be the means by which the fix is verified.

### Validation strategy

| Check | How |
|---|---|
| The bias no longer runs away | `grep '\[VIO\] state' log.log` and confirm `\|ba\|` stays below 0.01 m/s^2 for the whole flight, against 3.37 today. This is the single check that decides whether Fix 1 and Fix 2 worked |
| The velocity tracks truth | Confirm `\|v\|` settles near the 12.9 m/s QGroundControl ground speed, against 714 m/s today |
| Scale is correct | `grep '\[VIO\] triang'` and confirm `depth_m mean` stays near the 212 m slant depth, against 7.2e5 m today |
| The inertial residual is doing work | `grep '\[VIO\]\[SPLIT\]'` and confirm `imu` is comparable to `vision` rather than 0.033 against 1746 |
| The initialisation assumption holds | `grep '\[VIO\] init'` and confirm `\|accel\|` is within a few hundredths of 9.81, against 11.1006 today |
| The correction loop becomes live | Confirm the `local-mapper pose update rejected` count falls far below the 13,770 of today, which will show the estimator and the mapper agreeing |
| SLAM covers the interval logging covers | Confirm from the capture that the `Requesting ACTIVATE` line for `basalt_slam_node` now follows the `armed=True flying=False` status message rather than the `flying=True` one, and that `Setting up filter` precedes the first climb |
| The stationary interval is benign | `grep '\[VIO\] triang'` over the pre-takeoff seconds and confirm `added=0` with `short_baseline` accounting for every candidate, that `[VIO] state` holds `\|p\|`, `\|v\|` and `\|ba\|` near zero throughout, and that neither the estimator nor the mapper accumulates unbounded state while there is no parallax |
| ORB-SLAM3 still initialises | `class MonoDriver` shares the widened `_desired_state`, so run the ORB-SLAM3 monocular stack and confirm its initialiser waits for parallax rather than failing, and that the node survives being ACTIVE while stationary |
| No regression elsewhere | Re-run EuRoC MH_01_easy and TUM-VI room1 with their own unchanged configuration files and confirm the trajectories are unchanged, since Fix 2 and Fix 3 touch `data/sitl_config_vo.json` alone and Fix 1 touches no code those datasets reach |

## Validation Status

| Check | Status |
|---|---|
| Instrumentation compiles and runs | Passed. The capture of 2026-09-12 carries 34,846 `[VIO]` and 4,004 `[Optical Flow]` records from a full mission, so the build is clean under `-Wall -Wextra -Werror` on the host and the records are well formed. Correction, 2026-09-12. This row previously read "Not run", because the development container has no Eigen at `/usr/include/eigen3` and no target can be compiled there. The host build has since settled it |
| Behaviour unchanged with `vio_debug` false | Established by inspection. Every added statement is inside a `vio_debug` guard and no control flow was altered |
| Existing call sites of changed signatures | Enumerated. `trackPoints` has two, both preserved by the defaulted parameter. `addPoints` and `filterPoints` have two each, none consuming the return value. No other class derives from `class FrameToFrameOpticalFlow` |
| Records are machine readable | Partially. `std::cout` is not synchronised across the estimator, mapper and front end threads, so a small fraction of records interleave mid line. Every reduction in this document filters them out. Worth fixing if the instrumentation is kept beyond this investigation, by guarding the estimator and front end writes with the mutex the mapper already uses |
| Fix 1 applied | `ros_ws/src/controllers/controllers/slam/driver_node.py` edited in `_desired_state` and `_phase_description`. `python3 -m py_compile` passes and an AST walk confirms both functions parse with the expected statement counts. No runtime check yet, since the lifecycle transition can only be exercised against a running stack |
| Fix 2 applied | `data/sitl_config_vo.json` line 35 changed from 1e2 to 1e8. The file still parses as JSON and the value reads back as 1e8. The index ordering that makes `vio_init_bg_weight` the accelerometer key was re-verified at `thirdparty/basalt-headers/include/basalt/imu/imu_types.h` rather than taken on trust |
| Fix 2 does not reach other datasets | Verified. All seven other configuration files still carry `vio_init_bg_weight` of 1e2 |
| Fix 3 and Fix 4 | Deferred at the reviewer's direction, pending further investigation. Neither is required for the divergence |
| Python lint | Not run. `flake8` is not installed in the development container, and the package declares `ament_lint_common` as a test dependency, so the first `colcon test` on the host must be checked. Note that the superseded branches are commented out rather than left as unreachable statements, so no dead-code diagnostic is expected |
| Regression against EuRoC and TUM-VI | Not run. Required before the fix lands, although Fix 2 and Fix 3 touch `data/sitl_config_vo.json` only and cannot reach those configurations |
