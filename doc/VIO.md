# Visual-Inertial Odometry in Basalt

## 1. Introduction

Visual-Inertial Odometry (VIO) is the task of estimating the six-degree-of-freedom ego-motion of a rigid body equipped with one or more cameras and an Inertial Measurement Unit (IMU). By tightly coupling visual feature tracks with inertial measurements, VIO systems recover metric-scale trajectories in environments where neither modality alone is sufficient: cameras fail under low-texture, high dynamic range, or rapid motion; IMUs integrate noise into unbounded drift when used alone. The complementary sensing properties make visual-inertial fusion the dominant paradigm for real-time state estimation on mobile robots, drones, and head-mounted devices.

Basalt (Usenko et al., 2020) is a modular, open-source VIO/SLAM framework built around a fixed-lag smoother operating on a sliding window of keyframes and recent inertial states. The estimator solves a non-linear least squares problem whose cost aggregates three factor types: visual reprojection residuals defined on a sparse set of tracked landmarks, IMU preintegration residuals connecting consecutive states, and a marginalisation prior that compresses all information from previously evicted states. The system is implemented as a producer–consumer pipeline: an optical-flow frontend delivers feature tracks on a concurrent queue (`tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr>`), the IMU stream feeds another queue, and the estimator drains both inside a dedicated processing thread.

At the source level, the VIO estimator is realised by the template class `SqrtKeypointVioEstimator<Scalar>` (`include/basalt/vi_estimator/sqrt_keypoint_vio.h`, `src/vi_estimator/sqrt_keypoint_vio.cpp`). It inherits `VioEstimatorBase<Scalar>` for the queue/lifecycle interface and `SqrtBundleAdjustmentBase<Scalar>` for shared bundle-adjustment utilities. The square-root form (Demmel et al., 2021) maintains the marginalisation prior as a Jacobian factor `J` rather than an information matrix `J^T J`, doubling the effective numerical precision.

The rest of this document dissects the VIO pipeline as orchestrated by `SqrtKeypointVioEstimator::ProcessFrame` and `SqrtKeypointVioEstimator::measure` into six successive stages:

- **§2 IMU Preintegration** — accumulation of high-rate inertial samples between two image timestamps into a compact pseudo-measurement with propagated covariance and bias Jacobians (`IntegratedImuMeasurement<Scalar>`).
- **§3 State Prediction** — use of the preintegrated pseudo-measurement to forecast the current-frame state from the previous estimate, providing a warm start to the optimizer (`IntegratedImuMeasurement::predictState`).
- **§4 Keyframe Selection** — the feature-tracking-ratio heuristic that decides when a new keyframe is inserted into the sliding window.
- **§5 Triangulation** — geometric initialization of new landmarks from stereo or temporal baselines and the pruning of lost landmarks (`BundleAdjustmentBase::triangulate`).
- **§6 Optimisation and Marginalisation** — the Levenberg–Marquardt loop, residual and Jacobian assembly, and the marginalisation step that keeps the active state vector bounded. Detailed treatment of marginalisation is deferred to `doc/Marginalisation.md`.
- **§7 Key Classes and Interfaces** — consolidated reference of the types, methods, and fields exercised by the previous sections.

Throughout, file/line references point to the canonical implementation in this repository (`/ws/ros_ws/src/slam/ext/basalt`).

---

## 2. IMU Preintegration

Inertial Measurement Units sample angular velocity $\boldsymbol{\omega}_k$ and specific force $\mathbf{a}_k$ at frequencies (typically 200–1000 Hz) that far exceed the camera frame rate (20–60 Hz). A naïve optimizer that treated each IMU sample as an independent factor and kept its endpoints as optimization variables would incur a linear re-integration cost every time a pose, velocity, or bias changed during a non-linear iteration. Preintegration (Lupton and Sukkarieh, 2012; Forster et al., 2017) folds the raw samples between two camera timestamps into a single pseudo-measurement expressed in the reference frame of the first state, decoupling the heavy integration from the optimization variables.

The canonical reference is Forster et al. (2017), *On-Manifold Preintegration for Real-Time Visual–Inertial Odometry*, IEEE T-RO. Basalt deviates from the Forster formulation in two respects: (i) it stores the delta state as a `PoseVelState<Scalar>` with a full $SE(3) \times \mathbb{R}^3$ structure and propagates it in Euclidean body frame rather than separately accumulating $\Delta R, \Delta v, \Delta p$; (ii) it uses a mid-point (trapezoidal) integrator for the rotation update to reduce discretization error at moderate sampling rates.

### 2.1 Mathematical Formulation

**Bias-compensated samples.** Given a raw IMU measurement $(\mathbf{a}_k^{\text{raw}}, \boldsymbol{\omega}_k^{\text{raw}})$ at time $t_k$, the corrected samples relative to the linearization biases $(\mathbf{b}_a^\text{lin}, \mathbf{b}_g^\text{lin})$ are
$$ \mathbf{a}_k = \mathbf{a}_k^{\text{raw}} - \mathbf{b}_a^\text{lin}, \qquad \boldsymbol{\omega}_k = \boldsymbol{\omega}_k^{\text{raw}} - \mathbf{b}_g^\text{lin}. $$
Gravity is *not* removed at this stage; it is added at residual-evaluation time through the predicted-state equation.

**Mid-point integrator.** For a time step $\Delta t = t_{k+1} - t_k$, the body-frame state $(\mathbf{R}_k, \mathbf{v}_k, \mathbf{p}_k)$ is propagated using the mid-point rotation $\mathbf{R}_{k+\tfrac{1}{2}} = \mathbf{R}_k\, \text{Exp}\!\left(\tfrac{1}{2}\Delta t\, \boldsymbol{\omega}_k\right)$:
$$
\begin{aligned}
  \mathbf{R}_{k+1} &= \mathbf{R}_k\, \text{Exp}(\Delta t\, \boldsymbol{\omega}_k), \\
  \mathbf{v}_{k+1} &= \mathbf{v}_k + \mathbf{R}_{k+\tfrac{1}{2}}\, \mathbf{a}_k\, \Delta t, \\
  \mathbf{p}_{k+1} &= \mathbf{p}_k + \mathbf{v}_k\, \Delta t + \tfrac{1}{2}\,\mathbf{R}_{k+\tfrac{1}{2}}\, \mathbf{a}_k\, \Delta t^2.
\end{aligned}
$$
The mid-point evaluation of $\mathbf{R}$ is second-order accurate for constant $\boldsymbol{\omega}$, whereas a trapezoidal rule in $\mathbf{R}$ alone would be first-order. The gravity term is intentionally absent — the delta state is expressed *relative* to the starting body frame with zero gravity.

**Covariance and bias Jacobian propagation.** Linearising the propagation around $(\mathbf{R}_k, \mathbf{v}_k, \mathbf{p}_k)$ gives the state-transition Jacobian $\mathbf{F}$ and the noise-input Jacobians $\mathbf{A}$ (accel) and $\mathbf{G}$ (gyro):
$$
\boldsymbol{\Sigma}_{k+1} = \mathbf{F}\, \boldsymbol{\Sigma}_k\, \mathbf{F}^T + \mathbf{A}\, \boldsymbol{\Sigma}_a\, \mathbf{A}^T + \mathbf{G}\, \boldsymbol{\Sigma}_g\, \mathbf{G}^T,
$$
with $\boldsymbol{\Sigma}_a, \boldsymbol{\Sigma}_g$ the discrete-time accelerometer and gyroscope noise covariances (diagonal). The bias-correction Jacobians, which enable analytical bias updates without re-integration, are propagated in parallel:
$$
\mathbf{J}_{\Delta|b_a} \leftarrow -\mathbf{A} + \mathbf{F}\, \mathbf{J}_{\Delta|b_a}, \qquad
\mathbf{J}_{\Delta|b_g} \leftarrow -\mathbf{G} + \mathbf{F}\, \mathbf{J}_{\Delta|b_g}.
$$

**Residual.** Let $\Delta \mathbf{s} = (\Delta \mathbf{R}, \Delta \mathbf{v}, \Delta \mathbf{p})$ be the accumulated delta and $\Delta t$ the total interval. Given two candidate states $\mathbf{s}_0, \mathbf{s}_1$ with current biases $(\mathbf{b}_a, \mathbf{b}_g)$, the $9$-vector residual (with translation, rotation, velocity components) is
$$
\begin{aligned}
\mathbf{r}_p &= \mathbf{R}_0^T(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0 \Delta t - \tfrac{1}{2}\mathbf{g}\,\Delta t^2) - \left(\Delta \mathbf{p} + \mathbf{J}^p_{b_a}\Delta \mathbf{b}_a + \mathbf{J}^p_{b_g}\Delta \mathbf{b}_g\right), \\
\mathbf{r}_R &= \log\!\left(\text{Exp}(\mathbf{J}^R_{b_g}\Delta \mathbf{b}_g)\,\Delta \mathbf{R}\, \mathbf{R}_1^T \mathbf{R}_0\right), \\
\mathbf{r}_v &= \mathbf{R}_0^T(\mathbf{v}_1 - \mathbf{v}_0 - \mathbf{g}\,\Delta t) - \left(\Delta \mathbf{v} + \mathbf{J}^v_{b_a}\Delta \mathbf{b}_a + \mathbf{J}^v_{b_g}\Delta \mathbf{b}_g\right),
\end{aligned}
$$
with $\Delta \mathbf{b}_\bullet = \mathbf{b}_\bullet - \mathbf{b}_\bullet^\text{lin}$. The linear bias correction is exact only for small $\Delta \mathbf{b}$; if biases drift substantially during the interval, re-integration is required — in Basalt this is handled implicitly by marginalisation, which fixes the linearization biases for states evicted from the active window.

**Bias random walk.** Separately from preintegration, biases are constrained between consecutive states by a discrete-time Gaussian random walk with weight $\mathbf{W}^b = \boldsymbol{\sigma}_b^{-1}/\sqrt{\Delta t}$, producing two additional 3-vector residuals per IMU factor (accel and gyro bias difference).

### 2.2 Implementation Overview

The preintegration code lives in the header-only library `thirdparty/basalt-headers/include/basalt/imu/preintegration.h`; it is re-exported through `include/basalt/imu/preintegration.h` and used verbatim by the estimator.

**Class `IntegratedImuMeasurement<Scalar>`** — (`thirdparty/basalt-headers/include/basalt/imu/preintegration.h:49`).
It stores four pieces of information:
- `PoseVelState<Scalar> delta_state_` — the accumulated $\Delta \mathbf{s}$ (9-vector). Its `t_ns` field holds the elapsed nanoseconds $\Delta t_{\text{ns}}$.
- `MatNN cov_` — the $9\times 9$ measurement covariance $\boldsymbol{\Sigma}_{\Delta}$ (lazily square-root inverted in `compute_sqrt_cov_inv`).
- `MatN3 d_state_d_ba_`, `MatN3 d_state_d_bg_` — the $9\times 3$ bias-correction Jacobians.
- `Vec3 bias_gyro_lin_, bias_accel_lin_` — the biases used as the linearization point.

**Static method `propagateState`** — (`preintegration.h:76`). Given the current state and one sample, it produces the predicted state and optionally the three Jacobians $(d_\text{next}/d_\text{curr},\, d_\text{next}/d_\text{accel},\, d_\text{next}/d_\text{gyro})$. Lines 89–100 implement the mid-point integrator exactly as in §2.1.

**Instance method `integrate`** — (`preintegration.h:161`). Subtracts bias, calls `propagateState` with Jacobians, propagates covariance, and updates bias Jacobians (lines 177–183). Invalidates the cached square-root inverse covariance.

**Construction and owner.** Basalt constructs one `IntegratedImuMeasurement` per inter-frame interval. In `SqrtKeypointVioEstimator::ProcessFrame` (`src/vi_estimator/sqrt_keypoint_vio.cpp:271`):
```cpp
meas.reset(new IntegratedImuMeasurement<Scalar>(
    this->prev_frame->t_ns, last_state.getState().bias_gyro,
    last_state.getState().bias_accel));
```
After instantiation the method drains IMU samples until `imuData->t_ns > curr_frame->t_ns` and pushes each through `meas->integrate(...)` (lines 288–296). If the final IMU sample straddles the frame timestamp, a synthetic sample is created by rewriting `imuData->t_ns = curr_frame->t_ns` (lines 298–304) to close the interval exactly.

**IMU consumption loop.** IMU samples are drawn from `imu_data_queue` — a `tbb::concurrent_bounded_queue<ImuData<double>::Ptr>` defined in `VioEstimatorBase` (`include/basalt/vi_estimator/vio_estimator.h:81`, capacity 300 at `sqrt_keypoint_vio.cpp:124`). The helper `popFromImuDataQueue()` (`sqrt_keypoint_vio.cpp:326`) pops and scalar-casts. Each pop is immediately followed by accelerometer-and-gyro bias calibration via `Calibration<Scalar>::calib_accel_bias.getCalibrated(...)` (lines 282–285) — this subtracts manufacturer-calibrated biases, orthogonal to the bias state being estimated.

**Downstream use.** The preintegrated object persists in `imu_meas[start_t_ns]` (`sqrt_keypoint_vio.cpp:370`) for the lifetime of the state it connects and is consumed by:
- `measure()` for state prediction (see §3),
- `ImuBlock::linearizeImu` for residual/Jacobian assembly (see §6),
- `ScBundleAdjustmentBase::computeImuError` for cost evaluation during LM step-acceptance checks.

---

## 3. State Prediction

A good initial guess is essential for the success of any non-linear least-squares iteration. At the moment a new image arrives, the most recently optimized state is at most one keyframe-period old; the IMU has been silently accumulating delta measurements. Before surrendering the new timestamp to the optimizer, Basalt integrates those measurements analytically to forecast the new pose, velocity, and biases. The forecast is consistent with the IMU residual (a zero residual implies the forecast satisfies it exactly), so the optimizer typically only needs a few iterations to absorb visual corrections.

### 3.1 Mathematical Formulation

Given the previous state $\mathbf{s}_0 = (\mathbf{R}_0, \mathbf{p}_0, \mathbf{v}_0, \mathbf{b}_{a,0}, \mathbf{b}_{g,0})$, gravity $\mathbf{g}$, and a preintegrated measurement $\Delta \mathbf{s} = (\Delta \mathbf{R}, \Delta \mathbf{v}, \Delta \mathbf{p})$ of duration $\Delta t$, the predicted state $\mathbf{s}_1$ is
$$
\begin{aligned}
\mathbf{R}_1 &= \mathbf{R}_0\, \Delta \mathbf{R}, \\
\mathbf{v}_1 &= \mathbf{v}_0 + \mathbf{g}\,\Delta t + \mathbf{R}_0\,\Delta \mathbf{v}, \\
\mathbf{p}_1 &= \mathbf{p}_0 + \mathbf{v}_0\,\Delta t + \tfrac{1}{2}\mathbf{g}\,\Delta t^2 + \mathbf{R}_0\,\Delta \mathbf{p}.
\end{aligned}
$$
Biases are propagated by identity: $\mathbf{b}_{a,1} = \mathbf{b}_{a,0}$, $\mathbf{b}_{g,1} = \mathbf{b}_{g,0}$. This is the "zero-bias-random-walk" choice — a consistent prior for the optimizer to adjust.

Note that the delta quantities are held in the start-of-interval body frame by construction (§2.1), so $\mathbf{R}_0$ appears as a pre-multiplier on $\Delta \mathbf{v}$ and $\Delta \mathbf{p}$. Gravity enters the prediction but is absent from the preintegration, which is why the residual function of §2.1 mirrors this structure.

### 3.2 Implementation Overview

**`IntegratedImuMeasurement::predictState`** — (`preintegration.h:191`). Implements the closed-form equations above, operating on a `PoseVelState` that is subsequently widened to a `PoseVelBiasState` by the caller.

**Call site: `SqrtKeypointVioEstimator::measure`** — (`sqrt_keypoint_vio.cpp:351`). The relevant block:
```cpp
if (meas.get()) {
    PoseVelBiasState<Scalar> next_state =
        frame_states.at(last_state_t_ns).getState();

    meas->predictState(frame_states.at(last_state_t_ns).getState(), g,
                       next_state);

    last_state_t_ns = opt_flow_meas->t_ns;
    next_state.t_ns = opt_flow_meas->t_ns;

    frame_states[last_state_t_ns] =
        PoseVelBiasStateWithLin<Scalar>(next_state);

    imu_meas[meas->get_start_t_ns()] = *meas;
}
```
Three pre-conditions are asserted (lines 352–356): the delta's start time matches the previous state, the delta's end time matches the current observation, and the duration is strictly positive. These catch the common integration-before-prediction race where an IMU sample is missing, which otherwise leads to divergent pose estimates.

After prediction, the new state is wrapped in a `PoseVelBiasStateWithLin<Scalar>` (`include/basalt/utils/imu_types.h:67`). This wrapper supports the **First-Estimate Jacobians (FEJ)** mechanism: the `linearized` flag is initially `false`, meaning the state is still free to move and has no fixed linearization point. Once marginalised (see `doc/Marginalisation.md`), the flag flips to `true` via `setLinTrue()`, freezing `state_linearized` while the live estimate moves as `state_current = state_linearized ⊕ delta`.

**Initial bootstrap.** On the very first frame (`sqrt_keypoint_vio.cpp:227`, `if (!initialized)`), no previous state exists. Basalt picks the initial orientation such that the accelerometer reading aligns with $+Z$ (line 241):
```cpp
T_w_i_init.setQuaternion(Eigen::Quaternion<Scalar>::FromTwoVectors(
    imuData->accel, Vec3::UnitZ()));
```
This assumes the sensor is stationary at startup so the accelerometer measures only gravity. An identity-velocity state is then seeded.

**Cross-stage coupling.** The predicted state is the *only* information carried forward about the new frame before the visual factors are added. If the prediction is wildly off — for instance because IMU biases diverged during a long stationary period — the optical-flow frontend may lose tracks due to predicted patch offsets that no longer match the image, after which keyframe creation cascades and optimization quality degrades.

---

## 4. Keyframe Selection

Running bundle adjustment on every frame would saturate the sliding window in under a second. Basalt therefore uses a two-tier strategy: the full navigation state (pose, velocity, biases) is retained for the last `vio_max_states` frames (default 3), while longer-lived visual constraints are preserved only for a set of *keyframes* (up to `vio_max_kfs`, default 7). A keyframe is spawned when the current frame is unable to re-observe enough landmarks hosted by existing keyframes. This heuristic is a specialisation of the classical "visual redundancy" criterion used in PTAM, ORB-SLAM, and DSO.

**Rationale.** Visual constraints lose value when tracked landmarks gradually drift out of the camera's field of view. When the overlap ratio falls below a threshold, new landmarks must be created, which requires a new host frame — a new keyframe. The counter-indication is motion-blur-induced track loss, where a *temporary* drop in track count should not be promoted to a permanent keyframe. The combined rule below guards against this by also requiring a minimum number of frames to have elapsed since the previous keyframe.

### 4.1 Implementation Overview

The check is centralised in `SqrtKeypointVioEstimator::measure` (`sqrt_keypoint_vio.cpp:411`):
```cpp
if (Scalar(connected0) / (connected0 + unconnected_obs0.size()) <
        Scalar(config.vio_new_kf_keypoints_thresh) &&
    frames_after_kf > config.vio_min_frames_after_kf)
    take_kf = true;
```
Here `connected0` counts observations in camera 0 whose corresponding landmark is already in the `LandmarkDatabase` (`sqrt_keypoint_vio.cpp:386`), and `unconnected_obs0` is the set of camera-0 keypoints seen but not yet triangulated. The configuration knobs are `vio_new_kf_keypoints_thresh` (ratio threshold, default 0.7) and `vio_min_frames_after_kf` (hysteresis counter, default 5).

**Connected-landmark accounting.** Lines 380–409 iterate over every observation in every camera of the current `OpticalFlowResult`. For each (camera, keypoint) pair:
- If the keypoint already exists in `lmdb`, a `KeypointObservation<Scalar>` is created and pushed via `lmdb.addObservation(tcid_target, kobs)`, extending the landmark's observation set. A per-host counter `num_points_connected[tcid_host.frame_id]` is incremented — this map is later consumed by the marginalisation keyframe-dropping heuristic (see §4.1.2 of `doc/Marginalisation.md`).
- If the keypoint does not exist and this is camera 0, its id is placed in `unconnected_obs0`, marking it for potential triangulation on the next keyframe (§5).

**Stereo asymmetry.** Only camera 0 (`i == 0` on lines 402, 404) contributes to the ratio. Stereo correspondences in camera 1 still populate the landmark database but do not trigger keyframe creation — the estimator treats camera 0 as the privileged host.

**Outcome.** When `take_kf` is set (`sqrt_keypoint_vio.cpp:421`), three actions follow:
1. `take_kf` is reset and `frames_after_kf` is zeroed.
2. `kf_ids.emplace(last_state_t_ns)` registers the current frame as a keyframe — the set `std::set<int64_t> kf_ids` is the authoritative record consulted during marginalisation (`sqrt_keypoint_vio.cpp:426`).
3. Triangulation of all `unconnected_obs0` keypoints runs (see §5). If `take_kf` is false, the counter advances (`frames_after_kf++;`) and no landmarks are added.

**Dropping a keyframe.** Eviction from `kf_ids` is the concern of `marginalize(...)` — detailed in `doc/Marginalisation.md` §4.1.

---

## 5. Triangulation

When a new keyframe is created, Basalt promotes previously-untriangulated 2D keypoints to 3D landmarks so that subsequent frames can benefit from their visual constraints. Three sub-operations occur in sequence:

1. **Observation gathering** — for each untriangulated feature id, the estimator traverses the recent `prev_opt_flow_res` history and collects every 2D observation, keyed by `TimeCamId`.
2. **Triangulation** — the algorithm tries each gathered observation in turn as the "other" view; once a baseline exceeds `vio_min_triangulation_dist` and the DLT solution yields a positive, bounded inverse-depth, the landmark is accepted.
3. **Lost-landmark filtering** — landmarks not observed in the current frame are flagged for optional marginalisation, decoupling map growth from the active state vector.

These steps work together to balance two competing pressures: insufficient baseline produces ill-conditioned depth (ambiguous up to scale), while waiting too long delays the inclusion of a feature and may drop it entirely if tracking is lost.

### 5.1 Mathematical Formulation

**Linear triangulation.** Given unit bearing vectors $\mathbf{f}_0, \mathbf{f}_1 \in \mathbb{R}^3$ observed from camera poses with known relative transform $\mathbf{T}_{0,1} \in SE(3)$, the DLT system stacks the two cross-product constraints $\mathbf{f}_i \times (\mathbf{P}_i \mathbf{X}) = 0$ into a $4\times 4$ system $\mathbf{A}\mathbf{X} = \mathbf{0}$ where $\mathbf{X} = [\mathbf{x}^T, w]^T$ is the homogeneous world point. Basalt uses the projection matrices $\mathbf{P}_1 = [\mathbf{I}\ |\ \mathbf{0}]$ and $\mathbf{P}_2 = \mathbf{T}_{0,1}^{-1}$, and the four rows
$$
\begin{aligned}
\mathbf{A}_{0,:} &= f_{0,x}\, \mathbf{P}_{1,2,:} - f_{0,z}\, \mathbf{P}_{1,0,:}, \\
\mathbf{A}_{1,:} &= f_{0,y}\, \mathbf{P}_{1,2,:} - f_{0,z}\, \mathbf{P}_{1,1,:}, \\
\mathbf{A}_{2,:} &= f_{1,x}\, \mathbf{P}_{2,2,:} - f_{1,z}\, \mathbf{P}_{2,0,:}, \\
\mathbf{A}_{3,:} &= f_{1,y}\, \mathbf{P}_{2,2,:} - f_{1,z}\, \mathbf{P}_{2,1,:}.
\end{aligned}
$$
The null-space vector corresponding to the smallest singular value is returned by Jacobi SVD. It is then rescaled such that $\| \mathbf{X}_{0:3} \| = 1$, making the first three components the unit direction and the fourth the inverse-distance.

**Stereographic landmark parameterisation.** The resulting $(\mathbf{d}, \rho)$ pair with $\|\mathbf{d}\|=1$ lies on the unit sphere (a 2-manifold) times the positive real line. To give the direction a *minimal* two-parameter representation, Basalt uses a stereographic projection `StereographicParam::project` (`thirdparty/basalt-headers/include/basalt/camera/stereographic_param.hpp:79`):
$$
\pi(\mathbf{d}) = \left[\frac{d_x}{d+d_z},\; \frac{d_y}{d+d_z}\right]^T, \quad d = \|\mathbf{d}\|.
$$
The landmark is stored as `(Vec2 direction, Scalar inv_dist)` in `Keypoint<Scalar>` (`include/basalt/vi_estimator/landmark_database.h:53`). This gives 3 DOF per landmark instead of the 4 of a homogeneous point, eliminating the gauge freedom of the 4-vector's scale. Stereographic projection is smooth and bijective except at the antipodal point $-\mathbf{z}$ — a benign restriction because landmarks always lie in front of the host camera ($d_z > 0$).

**Baseline gate.** The triangulation attempt is rejected if
$$
\|\mathbf{T}_{0,1}.\text{translation}\|^2 < (\text{vio\_min\_triangulation\_dist})^2.
$$
This suppresses the numerical ill-conditioning of near-zero-parallax DLT solutions.

**Inverse-depth gate.** After triangulation, the homogeneous point is accepted only if $w = \rho$ is positive and bounded (`p0_triangulated[3] > 0 && p0_triangulated[3] < 3.0`). The upper bound $\rho < 3$ corresponds to a minimum depth of $\sim 33\,\text{cm}$ and rejects near-singular triangulations where tiny errors in bearing explode into meter-scale depth variations.

### 5.2 Implementation Overview

**Landmark addition pipeline** — `SqrtKeypointVioEstimator::measure`, `sqrt_keypoint_vio.cpp:421–510`.

For each `lm_id` in `unconnected_obs0` the estimator gathers observations from the stored history (`prev_opt_flow_res`, an `aligned_map<int64_t, OpticalFlowResult::Ptr>`). The inner loop walks every past frame, every camera in that frame, and collects the observation into `kp_obs` (a `map<TimeCamId, KeypointObservation<Scalar>>`). This history is windowed — `prev_opt_flow_res` is pruned by marginalisation, so only recent, actively-tracked frames contribute.

**Triangulation attempt** (`sqrt_keypoint_vio.cpp:453`). For each candidate target observation `kv_obs`:
1. Unproject the 2D observations into unit bearings via `calib.intrinsics[i].unproject` — this is a visitor over the camera model variant (double-sphere, equidistant, etc.).
2. Compute the relative camera-to-camera pose:
```cpp
SE3 T_i0_i1 = getPoseStateWithLin(tcidl.frame_id).getPose().inverse() *
              getPoseStateWithLin(tcido.frame_id).getPose();
SE3 T_0_1 = calib.T_i_c[0].inverse() * T_i0_i1 *
            calib.T_i_c[tcido.cam_id];
```
`getPoseStateWithLin` (`ba_base.h:143`) transparently looks up `frame_states` (pose+vel+bias) first, falling back to `frame_poses` (pose only) — an essential abstraction because the state record of a frame changes type when the frame ages from the recent-states window into the keyframe window.
3. Baseline check (§5.1), skip if failed.
4. Call the static method `BundleAdjustmentBase::triangulate(p0_3d, p1_3d, T_0_1)` (`ba_base.h:99`), which runs the DLT and normalises.
5. If the depth gate passes, construct a `Keypoint<Scalar>` with `host_kf_id = tcidl`, `direction = StereographicParam::project(p0_triangulated)`, `inv_dist = p0_triangulated[3]`, and insert via `lmdb.addLandmark(lm_id, kpt_pos)`.
6. Loop-break via `valid_kp = true` — the first successful baseline wins.

If triangulation succeeds, *all* gathered observations (including host-frame and future-frame views) are pushed to the landmark via `lmdb.addObservation`. If it fails, the feature is silently abandoned; the next frame sees it again and may succeed once the baseline grows.

**Lost-landmark detection** (`sqrt_keypoint_vio.cpp:515`). If `config.vio_marg_lost_landmarks` is enabled, every landmark currently in `lmdb` is tested against the current observation set; any not observed in any camera of the current frame is added to `lost_landmaks` and forwarded to `marginalize(...)`. There the landmark's residual block participates in the Schur complement and is then deleted — its information is absorbed into the marginalisation prior rather than being discarded.

**`LandmarkDatabase<Scalar>`** — (`include/basalt/vi_estimator/landmark_database.h:86`). Holds two maps: `kpts` (keypoint id $\to$ landmark) and `observations` (host `TimeCamId` $\to$ target `TimeCamId` $\to$ set of landmark ids). The split indexing allows both fast landmark look-up (§6, visual residual evaluation) and fast host-based eviction (§4.1 of `doc/Marginalisation.md`). The `addObservation` method conveniently auto-populates both structures.

---

## 6. Optimisation and Marginalisation

With the window populated (§3), the landmark set updated (§5), and the keyframe topology decided (§4), Basalt solves a local non-linear least squares problem over all active variables. The cost combines three factor families:

1. **Reprojection factors** for every (landmark, observation) pair.
2. **IMU preintegration factors** for every consecutive state pair (plus bias random-walk residuals).
3. **The marginalisation prior** — a Gaussian factor on the Markov blanket of previously evicted states, maintained in square-root form.

Marginalisation, treated thoroughly in `doc/Marginalisation.md`, is the mechanism by which older variables are analytically eliminated through a Schur-complement block-update while their information is retained as a quadratic prior. This subsection focuses on optimisation proper and cross-references the marginalisation document where appropriate.

### 6.1 Mathematical Formulation

**Total objective.** Let $\mathbf{s}$ denote the stack of active pose-only frame poses (`frame_poses`, 6-DoF each), full navigation states (`frame_states`, 15-DoF each), and landmarks (`lmdb`, 3-DoF each). Basalt minimises
$$
E(\mathbf{s}) = \underbrace{\tfrac{1}{2} \sum_{(i,j)\in\mathcal{V}} \|\mathbf{r}_{ij}(\mathbf{s})\|^2_{\boldsymbol{\Sigma}_\text{vis}^{-1}}}_{\text{reprojection}} + \underbrace{\tfrac{1}{2} \sum_{k\in\mathcal{I}} \|\mathbf{r}_k^{\text{IMU}}(\mathbf{s})\|^2_{\boldsymbol{\Sigma}_k^{-1}}}_{\text{inertial}} + \underbrace{\tfrac{1}{2} \sum_{k\in\mathcal{I}} \left(\|\mathbf{r}_k^{b_a}\|^2_{\boldsymbol{W}_{b_a}} + \|\mathbf{r}_k^{b_g}\|^2_{\boldsymbol{W}_{b_g}}\right)}_{\text{bias random walk}} + E_{\text{marg}}(\mathbf{s}),
$$
with Huber robustification applied to the visual residuals. The marginalisation prior energy $E_{\text{marg}}$ is detailed in `doc/Marginalisation.md` §3.

**Linearisation.** Each iteration replaces the objective with its Gauss–Newton quadratic model at the current estimate $\mathbf{s}$:
$$
\mathbf{r}(\mathbf{s} \oplus \boldsymbol{\xi}) \approx \mathbf{r}(\mathbf{s}) + \mathbf{J}(\mathbf{s})\, \boldsymbol{\xi}, \qquad E_{\text{quad}}(\boldsymbol{\xi}) = \tfrac{1}{2}\|\mathbf{J}\boldsymbol{\xi} + \mathbf{r}\|^2_{\boldsymbol{W}}.
$$
The normal equations are $\mathbf{H}\boldsymbol{\xi} = -\mathbf{J}^T \mathbf{W}\mathbf{r} = \mathbf{b}$, with $\mathbf{H} = \mathbf{J}^T \mathbf{W} \mathbf{J}$.

**Jacobian structure.** The Jacobian is sparse and block-structured:
- For a reprojection residual observed in frame $t$ of a landmark hosted in frame $h$, $\mathbf{J}$ has non-zero blocks in the columns corresponding to $(\mathbf{T}_h, \mathbf{T}_t, \mathbf{l})$. The chain rule runs as $\partial \mathbf{r} / \partial \mathbf{T}_{t,h}$ (via `computeRelPose` Adjoint multiplication) and $\partial \mathbf{T}_{t,h} / \partial (\mathbf{T}_h, \mathbf{T}_t)$ (via `d_rel_d_h`, `d_rel_d_t`). See `linearizePoint` (`include/basalt/utils/ba_utils.h:82`).
- For an IMU residual between states $k$ and $k+1$, non-zero blocks land on $(\mathbf{T}_k, \mathbf{v}_k, \mathbf{b}_{g,k}, \mathbf{b}_{a,k})$ and $(\mathbf{T}_{k+1}, \mathbf{v}_{k+1})$ with additional bias-diff blocks. See `ImuBlock::linearizeImu` (`include/basalt/linearization/imu_block.hpp:26`).

**Landmark elimination (Schur complement).** Because each landmark appears in only its observing frames, the Hessian has an arrowhead structure. The landmark block is eliminated in place, producing a reduced camera system of dimension $\dim(\mathbf{s}_\text{pose}) + \dim(\mathbf{s}_\text{state})$ — the pose-and-velocity subset. Basalt offers three elimination strategies (enum `LinearizationType`):
- **`ABS_SC`** — standard Schur complement with absolute-pose parameterisation. Produces $(\mathbf{H}, \mathbf{b})$ directly.
- **`REL_SC`** — relative-pose parameterisation, later projected to absolute via the adjoint Jacobian.
- **`ABS_QR`** — square-root form (Demmel et al., 2021): a Householder QR decomposition of the landmark's Jacobian block eliminates the landmark columns without ever forming $\mathbf{J}^T \mathbf{J}$. The residuals $Q_2^T \mathbf{r}$ and Jacobian rows $Q_2^T \mathbf{J}_\text{pose}$ survive.

The `ABS_QR` path doubles the effective precision and is the default (`vio_linearization_type = ABS_QR` in `vio_config.cpp`).

**Levenberg–Marquardt.** The damped Hessian $\mathbf{H} + \lambda \cdot \text{diag}(\mathbf{H})$ is solved by Cholesky (LDLT) for the increment $\boldsymbol{\xi}^*$. Step acceptance uses the Nielsen rule: if the actual cost reduction $f_\text{diff}$ and the model-predicted reduction $l_\text{diff}$ have a positive ratio, the step is accepted and $\lambda$ is shrunk; otherwise $\lambda$ grows geometrically (factor `lambda_vee`, initially 2) and the step is retried. Termination conditions are (a) `f_diff < 1e-6`, (b) $\|\boldsymbol{\xi}\|_\infty < 10^{-4}$, (c) $\lambda >$ `vio_lm_lambda_max`, or (d) iteration count exceeds `vio_max_iterations`.

**Manifold updates.** Pose increments apply via `PoseState::incPose`: $\mathbf{p} \leftarrow \mathbf{p} + \boldsymbol{\upsilon}$ and $\mathbf{R} \leftarrow \text{Exp}(\boldsymbol{\omega})\,\mathbf{R}$ (left-multiplication convention, `imu_types.h:98`). Velocity and biases are Euclidean. Landmark updates act on the 3-DoF `(direction, inv_dist)` representation.

### 6.2 Implementation Overview

The top-level entry point is `SqrtKeypointVioEstimator::optimize_and_marg` (`sqrt_keypoint_vio.cpp:1521`), which unconditionally calls `optimize()` then `marginalize(...)`.

**`optimize()`** (`sqrt_keypoint_vio.cpp:1078`).

*Bootstrapping check.* The method is a no-op until the window contains either `opt_started == true` (persistent flag once set) or more than four frame states. This prevents optimisation on a trivially under-constrained window.

*Variable ordering.* An `AbsOrderMap aom` is built by iterating `frame_poses` first (contributing `POSE_SIZE = 6` blocks) and `frame_states` second (contributing `POSE_VEL_BIAS_SIZE = 15` blocks). Each entry maps a timestamp to `(start_index, block_size)`. Consistency with the existing `marg_data.order` is asserted block-by-block (lines 1103–1118), guarding against desynchronisation between the marginalisation prior and the active state.

*Linearisation factory.* `LinearizationBase::create` (`src/linearization/linearization_base.cpp:58`) dispatches on `config.vio_linearization_type` to instantiate one of the three concrete classes. The constructor allocates one `LandmarkBlock` per landmark (`landmark_block_abs_dynamic.hpp`) and one `ImuBlock` per preintegrated measurement, wiring them to `aom`, `marg_data`, and the `ImuLinData` bundle (which packages gravity, bias weights, and the map of preintegrated measurements).

*LM loop* (lines 1164–1489).

One outer iteration executes:
```
error_total = lqr->linearizeProblem(&numerically_valid);   // J, r
lqr->performQR();                                          // eliminate lms
```
`linearizeProblem` (`src/linearization/linearization_abs_qr.cpp:185`) evaluates relative-pose Jacobians `d_rel_d_h, d_rel_d_t` at the *linearization point* (`getPoseLin()`), then — if the state is already FEJ-frozen — re-evaluates residual values at the current estimate. Landmark linearisation runs in parallel via `tbb::parallel_reduce` (line 250). IMU blocks and the marginalisation-prior error are added sequentially.

`performQR` marginalises landmarks in place — each `LandmarkBlock::performQR` applies a Householder QR to its local Jacobian, yielding the reduced-system rows.

Inside, the inner backtracking loop solves the damped system:
```cpp
lqr->get_dense_H_b(H, b);
VecX Hdiag_lambda = (H.diagonal() * lambda).cwiseMax(min_lambda);
MatX H_copy = H;
H_copy.diagonal() += Hdiag_lambda;
Eigen::LDLT<Eigen::Ref<MatX>> ldlt(H_copy);
inc = ldlt.solve(b);
```
Up to three re-solves are attempted if the result is non-finite, each time inflating $\lambda$ by `lambda_vee` (lines 1286–1302). The increment sign is flipped (line 1319) because `inc` returned by the solver is the RHS of $\mathbf{H}\boldsymbol{\xi} = \mathbf{b}$ with $\mathbf{b} = -\mathbf{J}^T \mathbf{r}$.

*Apply, evaluate, accept-or-reject.*
- `backup()` snapshots every frame state, pose, and landmark (`ba_base.h:130`).
- `lqr->backSubstitute(inc)` updates landmarks and accumulates the quadratic model cost-change `l_diff` (`linearization_abs_qr.cpp:292`).
- `applyInc` is called on each `PoseStateWithLin` / `PoseVelBiasStateWithLin` (lines 1331–1340), respecting the FEJ flag — if linearised, the increment is accumulated into `delta` and `state_current = state_linearized ⊕ delta`; otherwise `state_linearized` itself moves.
- Three error terms are re-computed (lines 1349–1366): the visual error (`computeError`), the marginalisation-prior error (`computeMargPriorError`), and the IMU error (`ScBundleAdjustmentBase::computeImuError`).
- The actual cost decrease `f_diff = error_total - after_error_total` is compared to `l_diff`. Acceptance flips $\lambda$ down and breaks out; rejection restores from `backup`, raises $\lambda$, and re-solves.

*Termination.* Convergence is declared if `f_diff < 1e-6` and positive, or if the step norm is below $10^{-4}$ (lines 1446–1451). Failure is declared at $\lambda >$ `vio_lm_lambda_max` (line 1481).

**`marginalize()`** (`sqrt_keypoint_vio.cpp:603`). Detailed in `doc/Marginalisation.md`. The salient orchestration concerns at the VIO level are:
- The `marg_data` structure is maintained across calls (§6.1 mathematical formulation), with `is_sqrt = config.vio_sqrt_marg`.
- IMU factors that span only active states are added to the marginalised system (`ild.imu_meas[kv.first] = &kv.second;` at line 787), exactly as during optimisation.
- `frame_states.at(last_state_to_marg).setLinTrue()` (line 973) is the point at which the newest-to-be-kept state has its FEJ linearisation fixed.

**Parallelism.** Every expensive loop in the linearisation pipeline is a `tbb::parallel_for` or `tbb::parallel_reduce`. The dense LM solve is serial but benefits from Eigen's vectorisation. The producer–consumer architecture (`sqrt_keypoint_vio.cpp:162`) runs the estimator in its own `std::thread`, overlapping optimisation with sensor acquisition.

---

## 7. Key Classes and Interfaces

The references below collate every non-trivial type, method, and field touched by the preceding sections. Paths are relative to the repository root `/ws/ros_ws/src/slam/ext/basalt/`.

### 7.1 `SqrtKeypointVioEstimator<Scalar>`

- **Header:** `include/basalt/vi_estimator/sqrt_keypoint_vio.h`
- **Source:** `src/vi_estimator/sqrt_keypoint_vio.cpp`
- **Purpose:** Top-level sliding-window VIO estimator. Consumes optical-flow results and IMU samples, predicts states, manages keyframes and landmarks, and runs LM optimisation with marginalisation.
- **Inheritance:** `VioEstimatorBase<Scalar>` (queue/lifecycle) $\hookleftarrow$ `SqrtBundleAdjustmentBase<Scalar>` (BA + square-root utilities).

| Field / Method | Signature | Role |
|---|---|---|
| `frame_states` | `aligned_map<int64_t, PoseVelBiasStateWithLin<Scalar>>` | 15-DoF navigation states for recent frames (from `BundleAdjustmentBase`). |
| `frame_poses` | `aligned_map<int64_t, PoseStateWithLin<Scalar>>` | 6-DoF pure poses for aged-out keyframes. |
| `lmdb` | `LandmarkDatabase<Scalar>` | Landmark storage and observation index. |
| `imu_meas` | `aligned_map<int64_t, IntegratedImuMeasurement<Scalar>>` | Preintegrated IMU between consecutive active timestamps. |
| `prev_opt_flow_res` | `aligned_map<int64_t, OpticalFlowResult::Ptr>` | Recent optical-flow outputs, windowed by marginalisation. |
| `kf_ids` | `std::set<int64_t>` | Timestamps of current keyframes. |
| `num_points_kf` | `std::map<int64_t, int>` | Initial landmark count per keyframe — denominator of the KF drop heuristic. |
| `marg_data` | `MargLinData<Scalar>` | Square-root marginalisation prior `(H, b, order)`. |
| `nullspace_marg_data` | `MargLinData<Scalar>` | Debug-only parallel prior without initial priors for nullspace checks. |
| `g` | `Vec3` (const) | World-frame gravity. |
| `lambda, lambda_vee` | `Scalar` | LM damping and backtracking factor. |
| `max_states, max_kfs` | `size_t` | Sliding-window capacities. |
| `initialize(t_ns, T_w_i, vel, bg, ba)` | override | Seed the filter with ground-truth biases/pose (typically from a calibration pipeline). |
| `initialize(bg, ba)` | override | Lazy initialisation: seeds biases only; pose derived from accelerometer alignment. |
| `addIMUToQueue`, `addVisionToQueue` | override | Enqueue sensor inputs. |
| `popFromImuDataQueue()` | `ImuData<Scalar>::Ptr` | Dequeue next IMU sample, scalar-cast if needed. |
| `ProcessFrame(curr_frame)` | `PoseVelBiasState<Scalar>::Ptr` | Integrates IMU up to `curr_frame->t_ns`, calls `measure`. |
| `measure(opt_flow_meas, meas)` | ditto | Prediction, keypoint book-keeping, KF logic, triangulation, `optimize_and_marg`. |
| `optimize()` | `void` | LM loop with parallel linearisation. |
| `marginalize(num_points_connected, lost_landmaks)` | `void` | KF selection-to-drop, Schur-complement block update; see `doc/Marginalisation.md`. |
| `optimize_and_marg(...)` | `void` | Pipeline wrapper. |
| `logMargNullspace()` | `void` | Debug-only: computes nullspace eigenvalues. |

### 7.2 `IntegratedImuMeasurement<Scalar>`

- **Header:** `thirdparty/basalt-headers/include/basalt/imu/preintegration.h`
- **Purpose:** Preintegrated IMU pseudo-measurement between two timestamps.
- **Inheritance:** None.

| Field / Method | Signature | Role |
|---|---|---|
| `delta_state_` | `PoseVelState<Scalar>` | Accumulated $(\Delta \mathbf{R}, \Delta \mathbf{v}, \Delta \mathbf{p})$; `t_ns` holds $\Delta t_\text{ns}$. |
| `cov_` | `MatNN` ($9\times 9$) | Propagated covariance. |
| `sqrt_cov_inv_` | mutable `MatNN` | Cached LDLT-derived square-root inverse. |
| `d_state_d_ba_, d_state_d_bg_` | `MatN3` ($9\times 3$) | Jacobians for first-order bias correction. |
| `bias_gyro_lin_, bias_accel_lin_` | `Vec3` | Linearisation biases. |
| `propagateState(curr, data, next, Fs...)` | static | One-step mid-point integrator with Jacobians. |
| `integrate(data, accel_cov, gyro_cov)` | member | Accumulate one sample, update cov and bias Jacobians. |
| `predictState(state0, g, state1)` | member const | Forecast of §3. |
| `residual(state0, g, state1, bg, ba, J0, J1, Jbg, Jba)` | member const | 9-vector IMU residual + optional Jacobians. |
| `get_cov_inv()`, `get_sqrt_cov_inv()` | member const | Weight matrices for residual whitening. |
| `get_dt_ns()`, `get_start_t_ns()` | member const | Interval bounds. |

### 7.3 State wrappers and layouts — `include/basalt/utils/imu_types.h`

- **`PoseVelBiasStateWithLin<Scalar>`** (`:67`). Wraps a `PoseVelBiasState` with FEJ support: fields `linearized`, `delta`, `state_linearized`, `state_current`. Methods `setLinTrue`, `applyInc`, `getState`, `getStateLin`, `backup`/`restore`.
- **`PoseStateWithLin<Scalar>`** (`:188`). Analogous 6-DoF wrapper, with conversion constructor from `PoseVelBiasStateWithLin` discarding velocity and biases.
- **`AbsOrderMap`** (`:293`). `std::map<int64_t, std::pair<int, int>> abs_order_map` + `items` + `total_size`. The canonical layout dictionary consumed by every linearisation routine.
- **`ImuLinData<Scalar>`** (`:307`). Read-only bundle `(g, gyro_bias_weight_sqrt, accel_bias_weight_sqrt, imu_meas)` passed to the IMU block.
- **`MargLinData<Scalar>`** (`:318`). Square-root or squared marginalisation prior; see `doc/Marginalisation.md`.
- **`MargData`** (`:329`). Off-thread payload for NFR; see `doc/Marginalisation.md`.

### 7.4 `BundleAdjustmentBase<Scalar>` and descendants

- **Header:** `include/basalt/vi_estimator/ba_base.h`
- **Source:** `src/vi_estimator/ba_base.cpp`
- **Purpose:** Shared bundle-adjustment utilities independent of marginalisation strategy.
- **Key methods:**
  - `triangulate(f0, f1, T_0_1)` (`:99`) — static DLT + unit-direction + inverse-distance.
  - `computeError(error, outliers, threshold)` (`:57`, impl `ba_base.cpp:141`) — parallel visual residual evaluation with Huber weights.
  - `filterOutliers(outlier_threshold, min_num_obs)` — prune landmarks whose observations exceed error threshold.
  - `computeDelta(order, delta)` — collect per-state `getDelta()` into a stacked vector used by `linearizeMargPrior`.
  - `linearizeMargPrior(mld, aom, H, b, err)` — add the marg prior's quadratic term to an external `(H, b)`.
  - `computeMargPriorError(mld, err)` — evaluate prior cost at current state.
  - `computeMargPriorModelCostChange(mld, scaling, inc)` — prior's contribution to `l_diff`.
  - `backup()`, `restore()` — snapshot across all states and landmarks.
  - `getPoseStateWithLin(t_ns)` — unified pose look-up across `frame_poses` and `frame_states`.

**Descendants.**
- `ScBundleAdjustmentBase<Scalar>` (`include/basalt/vi_estimator/sc_ba_base.h`): adds `RelLinData`, `AbsLinData`, and the Schur-complement linearisation helpers `linearizeHelperStatic`, `linearizeHelperAbsStatic`, `linearizeAbs`, `updatePoints`, `updatePointsAbs`, `computeImuError` (`sc_ba_base.cpp:738`).
- `SqrtBundleAdjustmentBase<Scalar>` (`include/basalt/vi_estimator/sqrt_ba_base.h`): thin wrapper re-exporting the SC helpers; provides static `checkNullspace` / `checkEigenvalues` for the nullspace diagnostic.

### 7.5 Linearisation classes

- **Header:** `include/basalt/linearization/linearization_base.hpp`
- **Purpose:** Abstract interface for a single-iteration linearised system.
- **Factory:** `LinearizationBase<Scalar, POSE_SIZE>::create(estimator, aom, options, marg_lin_data, imu_lin_data, used_frames, lost_landmarks, last_state_to_marg)` (`src/linearization/linearization_base.cpp:58`) dispatches on `options.linearization_type`.

| Method | Contract |
|---|---|
| `linearizeProblem(valid*)` | Compute Jacobians + residuals; return total error; set `*valid = false` on numerical failure. |
| `performQR()` | In-place landmark elimination (Schur or Householder depending on variant). |
| `get_dense_H_b(H, b)` | Assemble reduced camera system for `LDLT::solve`. |
| `get_dense_Q2Jp_Q2r(Q2Jp, Q2r)` | Square-root equivalent (only meaningful for `ABS_QR`). |
| `backSubstitute(pose_inc)` | Update landmarks from pose increment; return quadratic model cost-change `l_diff`. |
| `log_problem_stats(stats)` | Problem-size diagnostics. |

**Concrete classes.**
- `LinearizationAbsQR<Scalar, POSE_SIZE>` (`include/basalt/linearization/linearization_abs_qr.hpp`) — absolute-pose + Householder QR landmark elimination. Default (`vio_linearization_type = ABS_QR`).
- `LinearizationAbsSC<Scalar, POSE_SIZE>` (`include/basalt/linearization/linearization_abs_sc.hpp`) — absolute-pose + Schur complement.
- `LinearizationRelSC<Scalar, POSE_SIZE>` (`include/basalt/linearization/linearization_rel_sc.hpp`) — relative-pose + Schur complement.

Internal helpers used by all three:
- `LandmarkBlock<Scalar>` (`include/basalt/linearization/landmark_block.hpp`) — per-landmark Jacobian/residual storage with states `Uninitialized / Allocated / NumericalFailure / Linearized / Marginalized`. Methods `allocateLandmark`, `linearizeLandmark`, `performQR`, `backSubstitute`, `get_dense_H_b`, `get_dense_Q2Jp_Q2r`.
- `ImuBlock<Scalar>` (`include/basalt/linearization/imu_block.hpp`) — per-IMU-factor analogue. `linearizeImu(frame_states)` evaluates residual + Jacobians at the linearisation point, whitens by `get_sqrt_cov_inv()`, and also produces the bias-random-walk Jacobian rows (`:74`–`:101`).
- `RelPoseLin<Scalar>` (`landmark_block.hpp:13`) — stores the relative pose matrix `T_t_h` and the adjoint Jacobians `d_rel_d_h, d_rel_d_t` used by the chain rule.

### 7.6 `LandmarkDatabase<Scalar>`

- **Header:** `include/basalt/vi_estimator/landmark_database.h`
- **Source:** `src/vi_estimator/landmark_database.cpp`
- **Purpose:** Storage and indexing of landmarks + observations.

| Field / Method | Role |
|---|---|
| `kpts` | `aligned_unordered_map<KeypointId, Keypoint<Scalar>>`. |
| `observations` | `unordered_map<TimeCamId, map<TimeCamId, set<KeypointId>>>` — host-to-target index. |
| `min_num_obs = 2` | Static threshold: landmarks with fewer observations are removed by cleanup paths. |
| `addLandmark(lm_id, kpt)`, `addObservation(tcid, obs)` | Create/extend records. |
| `getLandmark(lm_id)`, `landmarkExists(lm_id)`, `numLandmarks`, `numObservations` | Query. |
| `getHostKfs()`, `getLandmarksForHost(tcid)` | Host-indexed iteration (used by `get_current_points`). |
| `removeFrame`, `removeKeyframes(kfs_to_marg, poses_to_marg, states_to_marg_all)` | Marginalisation clean-up. |
| `removeLandmark`, `removeObservations` | Outlier pruning. |
| `backup()`, `restore()` | LM step-level snapshot/rollback. |

**Supporting types.**
- `Keypoint<Scalar>` (`landmark_database.h:53`) — `direction (Vec2)`, `inv_dist (Scalar)`, `host_kf_id (TimeCamId)`, `obs: aligned_map<TimeCamId, Vec2>`. The 2-vector `direction` is the stereographic image of the unit bearing (§5.1).
- `KeypointObservation<Scalar>` (`:43`) — `kpt_id`, `pos (Vec2)`.

### 7.7 Auxiliary types

- **`TimeCamId`** (`include/basalt/utils/common_types.h:62`) — `(FrameId frame_id, CamId cam_id)` pair uniquely identifying an image. Hashable, orderable.
- **`KeypointId`** — `int`. Global id of a tracked keypoint, assigned by the optical-flow frontend.
- **`FrameId`** — `int64_t` timestamp.
- **`OpticalFlowResult`** (`include/basalt/optical_flow/optical_flow.h:61`) — `t_ns`, `observations` (vector over cameras of `aligned_map<KeypointId, AffineCompact2f>`), `pyramid_levels`, `input_images`. The `AffineCompact2f` permits patch-aware rotation/scale compensation of the 2D observation.
- **`StereographicParam<Scalar>`** (`thirdparty/basalt-headers/include/basalt/camera/stereographic_param.hpp`) — static `project`, `unproject` and their Jacobians.
- **`computeRelPose<Scalar>(T_w_i_h, T_i_c_h, T_w_i_t, T_i_c_t, d_rel_d_h, d_rel_d_t)`** (`include/basalt/utils/ba_utils.h:41`) — yields the target-to-host camera-to-camera transform plus adjoint Jacobians.
- **`linearizePoint<Scalar, CamT>(...)`** (`ba_utils.h:82`) — single-observation reprojection residual + Jacobians w.r.t. relative pose and 3-vector landmark.
- **`VioConfig`** (`include/basalt/utils/vio_config.h:43`) — all tunables, in particular `vio_linearization_type`, `vio_sqrt_marg`, `vio_max_states`, `vio_max_kfs`, `vio_new_kf_keypoints_thresh`, `vio_min_frames_after_kf`, `vio_min_triangulation_dist`, `vio_marg_lost_landmarks`, `vio_kf_marg_feature_ratio`, `vio_lm_lambda_*`, `vio_init_pose_weight`, `vio_init_ba_weight`, `vio_init_bg_weight`, `vio_obs_std_dev`, `vio_obs_huber_thresh`.
- **`Calibration<Scalar>`** (`thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp`) — camera intrinsics (variant over models), IMU-to-camera extrinsics `T_i_c[cam_id]`, IMU noise and bias std, IMU-intrinsic calibration `calib_accel_bias`, `calib_gyro_bias`.
- **`MargHelper<Scalar>`** (`include/basalt/vi_estimator/marg_helper.h`) — static Schur-complement and Householder QR block elimination routines. Full treatment in `doc/Marginalisation.md`.

---

## 8. References

1. Usenko, V., Demmel, N., Schubert, D., Stückler, J., & Cremers, D. (2020). *Visual-Inertial Mapping with Non-Linear Factor Recovery*. IEEE Robotics and Automation Letters, arXiv:1904.06504.
2. Demmel, N., Schubert, D., Sommer, C., Cremers, D., & Usenko, V. (2021). *Square Root Marginalization for Sliding-Window Bundle Adjustment*. ICCV.
3. Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. (2017). *On-Manifold Preintegration for Real-Time Visual–Inertial Odometry*. IEEE Transactions on Robotics, 33(1), 1–21.
4. Lupton, T., & Sukkarieh, S. (2012). *Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions*. IEEE Transactions on Robotics, 28(1), 61–76.
5. Basalt source: `src/vi_estimator/sqrt_keypoint_vio.cpp`, `include/basalt/vi_estimator/sqrt_keypoint_vio.h`, and the supporting `include/basalt/linearization/` and `thirdparty/basalt-headers/` trees.
6. `doc/Marginalisation.md` — companion document on Schur-complement marginalisation in Basalt.
