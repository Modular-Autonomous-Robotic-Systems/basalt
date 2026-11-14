# VIO optimisation deep dive, `SqrtKeypointVioEstimator::optimize`

Date of investigation, 2026-10-30. Read from the working tree of branch `shreyas/kf_driven_mapping_and_logger`, on top of `7e91ec4`. Every anchor below is a `file:line` against that tree, and nothing in this document was executed, so every statement is a reading of source. The companion files are [`linearisation.md`](linearisation.md) for the `LinearizationBase` family and its defects, [`vio_residuals_and_priors.md`](vio_residuals_and_priors.md) for the IMU residual Jacobians and the initial prior, and [`keyframe_driven_local_mapping.md`](keyframe_driven_local_mapping.md) for what happens to a keyframe after it leaves the estimator. This file is the end to end walk through one call of `optimize()` and the machinery that surrounds it, written so that a new residual, a new state or a new solver variant can be added without re-reading the source.

Notation. `x` is the stacked sliding window state, `δ` an increment in the tangent space, `p` a position, `R` a rotation, `v` a velocity, `b_g` and `b_a` the gyroscope and accelerometer biases, `ℓ` a landmark, `ρ` an inverse distance, `T_t_h` the pose of host camera expressed in target camera, `σ` the pixel standard deviation `obs_std_dev`, `w` a Huber weight. Citations `[N]` refer to the References section.

## 1. Where `optimize` sits

One camera frame enters through `ProcessFrame` (`src/vi_estimator/sqrt_keypoint_vio.cpp:222`), which drains the IMU queue into a fresh `IntegratedImuMeasurement`, runs the three stage stationarity detector (`:448-498`) and calls `measure` (`:582`). `measure` is the only caller of `optimize_and_marg` (`:953`), which is three calls in a row at `:2022-2028`.

```
ProcessFrame
  measure
    1. apply mapper pose corrections to frame_poses            (:591-651)
    2. predict the new state from the preintegration           (:653-673)
    3. associate tracks to existing landmarks                  (:678-712)
    4. keyframe decision                                       (:714-717)
    5. if keyframe, triangulate new landmarks                  (:767-928)
    6. collect lost landmarks                                  (:931-945)
    7. optimize_and_marg
         optimize()                                            (:1529-2019)
         marginalize(num_points_connected, lost_landmaks)      (:1052-1526)
         PublishKeyframe()                                     (:2033-2054)
```

Steps 2 to 6 build exactly the problem that `optimize` solves, so they are described in sections 2 and 3 before the solver. The gate at the top of `optimize` is `opt_started || frame_states.size() > 4` (`:1530`). With one state added per frame, nothing is optimised and nothing is marginalised until the fifth frame, so the first four states are pure inertial prediction. `marginalize` returns early on `!opt_started` (`:1055`). Once set, `opt_started` is never cleared.

## 2. Estimator state

### 2.1 Two containers

`BundleAdjustmentBase` owns the window (`include/basalt/vi_estimator/ba_base.h:156-160`).

| Member | Type | Holds | Tangent size |
|---|---|---|---|
| `frame_states` | `aligned_map<int64_t, PoseVelBiasStateWithLin>` | the newest frames, full inertial state | `POSE_VEL_BIAS_SIZE` = 15 |
| `frame_poses` | `aligned_map<int64_t, PoseStateWithLin>` | older keyframes whose velocity and biases were marginalised | `POSE_SIZE` = 6 |
| `lmdb` | `LandmarkDatabase<Scalar>` | landmarks, observations, host index | per landmark 3 |

Both maps are keyed by the frame timestamp in nanoseconds, so ascending iteration is ascending time. A frame lives first in `frame_states`, and if it is a keyframe it is demoted into `frame_poses` by `marginalize` (`:1428-1435`), otherwise it is erased.

The body frame is the IMU. `T_w_i` is IMU to world, and camera poses are never states. The camera extrinsic `calib.T_i_c[cam]` is composed only where a reprojection residual needs it.

### 2.2 Tangent space and ordering

`PoseVelBiasState::applyInc` (`thirdparty/basalt-headers/include/basalt/imu/imu_types.h:221-225`) fixes the ordering `[trans(3), rot(3), vel(3), bias_gyro(3), bias_accel(3)]`, and `PoseState::incPose` (`:98-101`) fixes the manifold update.

```
p  <- p + δp
R  <- Exp(δφ) R            (left multiplication, world frame increment)
v  <- v + δv
b_g <- b_g + δb_g
b_a <- b_a + δb_a
```

A pose block is the first six of those. Index 6 within a state is the velocity, 9 the gyroscope bias and 12 the accelerometer bias.

### 2.3 First estimate Jacobians, the `WithLin` wrappers

`PoseStateWithLin` and `PoseVelBiasStateWithLin` (`include/basalt/utils/imu_types.h:53-270`) hold a flag `linearized`, a frozen linearisation point `state_linearized`, an accumulated `delta` and a cached `state_current`.

| `linearized` | `applyInc(inc)` | `getState()` | `getStateLin()` |
|---|---|---|---|
| false | moves `state_linearized` directly | `state_linearized` | the same moving value |
| true | `delta += inc`, `state_current = state_linearized ⊞ delta` | `state_current` | the frozen point |

So an unfrozen state is relinearised at every outer iteration, which is ordinary Gauss Newton. A frozen state is held at its linearisation point for Jacobians while its value moves, which is first estimate Jacobians (FEJ). Residual values always use `getState()` and Jacobians always use `getStateLin()`. The freeze is applied in exactly two places, at initialisation (`:156-158`, `:327-328`, constructed with `linearized = true`) and at the end of every marginalisation for the surviving oldest state (`:1417-1419`, `setLinTrue`). Freezing is what keeps the marginalisation prior consistent, because the prior was built from Jacobians at that point and `delta` is the offset of the present estimate from it.

`backup()` and `restore()` (`ba_base.h:130-140`) copy the `delta`, the linearisation point and the current value of every state, and the `direction` and `inv_dist` of every landmark. They are the rollback used for a rejected LM step.

### 2.4 Window size

With the library defaults `vio_max_states = 3` and `vio_max_kfs = 7` (`src/utils/vio_config.cpp:61-62`), a steady state `optimize` call sees three full states and up to seven keyframe slots, so the dense system dimension is `n = 6·|frame_poses| + 15·|frame_states|`, of the order of 80. `marginalize` fires when `frame_poses.size() > max_kfs || frame_states.size() >= max_states` (`:1059`), and removes `frame_states.size() - max_states + 1` states (`:1062`), so it removes exactly one per frame in steady state and leaves `max_states - 1` states for the next frame to extend back to three. The shipped JSON files may override these defaults, so check `data/*.json` before quoting a size.

## 3. Landmarks, from birth to linearisation

### 3.1 Parametrisation

A landmark is a `Keypoint<Scalar>` (`include/basalt/vi_estimator/landmark_database.h:60-84`) holding a host `TimeCamId host_kf_id`, a two vector `direction`, a scalar `inv_dist` and an observation map `obs` from `TimeCamId` to pixel. It is an inverse distance point anchored in its host camera frame, with the bearing carried by the stereographic parametrisation (`thirdparty/basalt-headers/include/basalt/camera/stereographic_param.hpp`).

```
unit bearing   m(u) = [ 2u / (1+|u|²) ;  (1-|u|²) / (1+|u|²) ]          u = direction ∈ R²
homogeneous    p_h  = [ m(u) ; ρ ]                                       ρ = inv_dist
```

Three degrees of freedom per landmark, and the stereographic chart is singular only at one bearing, the direction opposite to the camera axis, so it is safe for any visible point. The inverse distance admits points at infinity (`ρ = 0`) without a singularity, which suits the sub-pixel parallax of the SITL scene. `backSubstitute` clamps `ρ` at zero from below (`landmark_block_abs_dynamic.hpp:334`), so a landmark never crosses behind its host.

### 3.2 Association and the keyframe decision

For every observation of every camera in the incoming `OpticalFlowResult`, `measure` looks the track id up in `lmdb`. If a landmark exists, `lmdb.addObservation(tcid_target, kobs)` is called (`:697`), `num_points_connected[host_frame]` is incremented (`:700-703`), and camera 0 hits are counted in `connected0`. Camera 0 tracks with no landmark go to `unconnected_obs0` (`:707-709`).

`take_kf` is raised when `connected0 / (connected0 + |unconnected_obs0|) < vio_new_kf_keypoints_thresh` and `frames_after_kf > vio_min_frames_after_kf` (`:714-717`), defaults 0.7 and 5. `take_kf` is initialised true (`:67`), so the very first frame is a keyframe.

### 3.3 Triangulation on a keyframe

When `take_kf` is set (`:769-928`), the current frame becomes a keyframe (`kf_ids.emplace`, `mpIsCurrentFrameKF = true`) and each `unconnected_obs0` track is considered for a new landmark hosted at `tcidl = (t_now, cam 0)`.

1. Every observation of the track in every retained frame is gathered from `prev_opt_flow_res` (`:802-817`). That map contains the current frame, so the host's own observation is part of the set.
2. For each candidate pair in ascending `TimeCamId` order, both pixels are unprojected with the camera model, the relative pose `T_0_1 = T_ic0⁻¹ (T_w_i,now⁻¹ T_w_i,other) T_ic_other` is formed from `getPose()` of the two frames (`:848-852`), and the pair is rejected if the baseline is below `vio_min_triangulation_dist` (`:867`) or, when configured, the parallax is below `vio_min_triangulation_parallax_deg` (`:872`).
3. `BundleAdjustmentBase::triangulate` (`ba_base.h:99-128`) solves the 4x4 direct linear transform system by SVD and returns `[unit direction ; ρ]`. The result is accepted when finite, `0 < ρ < 3` (`:888-889`), that is a depth above 0.33 m.
4. The first accepted pair wins (`if (valid_kp) break`). `lmdb.addLandmark` stores `direction = StereographicParam::project(p0_triangulated)` and `inv_dist = ρ` (`:896-899`), and then every observation gathered in step 1 is added (`:906-910`), not merely the two that triangulated it.

The triangulation has no refinement, so the first optimisation after a keyframe is the first time the landmark meets a statistically weighted cost. For low parallax tracks this is where `ρ` is badly conditioned, see `vio_optical_flow_instrumentation.md` for the measured consequences.

### 3.4 Lifetime rules in `LandmarkDatabase`

`min_num_obs = 2` (`landmark_database.h`, private constant). `removeKeyframes` (`src/vi_estimator/landmark_database.cpp:72-101`) deletes a landmark outright when its host keyframe is being marginalised, and otherwise strips the observations belonging to departing frames and deletes the landmark if fewer than two remain. `removeLandmark` is called for the lost landmark set when `vio_marg_lost_landmarks` is set (`sqrt_keypoint_vio.cpp:1444-1446`). A host with no remaining targets should not survive, since `LinearizationAbsQR` asserts `!target_map.empty()` (`linearization_abs_qr.cpp:88`).

`lost_landmaks` (`:931-945`) is every landmark with no observation in the current frame in any camera, and is computed only when `vio_marg_lost_landmarks` is true, the default.

The `observations` index is `host TimeCamId -> target TimeCamId -> set of landmark ids` and is what `LinearizationAbsQR` iterates to size its caches.

## 4. The absolute order map (`aom`)

`AbsOrderMap` (`include/basalt/utils/imu_types.h:291-302`) is a plain struct.

```cpp
struct AbsOrderMap {
    std::map<int64_t, std::pair<int,int>> abs_order_map;  // frame_id -> (start column, tangent size)
    size_t items;                                         // number of frames
    size_t total_size;                                    // dimension of the dense system
};
```

It is the single translation table between a frame timestamp and its columns in every dense object of the solve, namely `H`, `b`, `inc`, the pose columns of every landmark block and every IMU block. Landmarks themselves are not in `aom`. Their parameters live in `lmdb`, their three columns are private to each `LandmarkBlockAbsDynamic` at `lm_idx` and are rotated away by `performQR`, so the dense system has pose, velocity and bias columns only and its size is independent of the landmark count.

### 4.1 Construction in `optimize` (`:1543-1568`)

```cpp
AbsOrderMap aom;
for (kv : frame_poses)  { aom[kv.first] = (total_size, POSE_SIZE);          total_size += 6;  items++; }
for (kv : frame_states) { aom[kv.first] = (total_size, POSE_VEL_BIAS_SIZE); total_size += 15; items++; }
```

The layout is therefore all poses in ascending time, followed by all full states in ascending time. It is grouped by type first and by time second, so a pose is always to the left of every state even if its timestamp is later, which does not occur in practice because poses are demoted from the oldest states.

```
column 0                                                                       total_size
|  pose_0 (6) | pose_1 (6) | ... | pose_P-1 (6) | state_0 (15) | state_1 (15) | state_2 (15) |
|<------------- prior block, always a prefix ----------->|
```

### 4.2 Why the order must match the prior

`marg_data.order` is the same table for the prior. It always lists every `frame_poses` entry plus exactly one full state, the oldest surviving one (`:1448-1463`). Because every marginalisation removes the oldest items and `optimize` only ever appends newer ones, the prior's variables are a contiguous prefix of the new `aom`, and the prior can be added to the top left block with no permutation. Two assertions enforce it on construction, `:1550-1551` for each pose and `:1562-1564` for each state that lies inside the prior's range, and `BundleAdjustmentBase::linearizeMargPrior` asserts it again for every entry (`ba_base.cpp:417-422`).

### 4.3 Consumers

`LinearizationAbsQR` stores a reference to `aom` and every `LandmarkBlockAbsDynamic` and `ImuBlock` indexes into it, so `aom` must outlive `lqr`. It does, since both are locals of one `optimize` call. The post-update loops at `:1835-1844` use it to slice `inc` per frame. The zero velocity term uses `aom.abs_order_map.at(last_state_t_ns).first + 6` (`:1753`).

## 5. The objective

The cost minimised by `optimize` is (see `vio_config.cpp` defaults for the weights)

```
F(x) =  Σ_ℓ Σ_{obs of ℓ}  ρ_H(‖r_ℓ,obs‖) / σ²                                  visual, Huber
      + Σ_k  ½ ‖ L_k r^imu_k ‖²  +  ½ ‖ Σ_g^{-½}/√Δt (b_g,k − b_g,k+1) ‖²
                                 +  ½ ‖ Σ_a^{-½}/√Δt (b_a,k − b_a,k+1) ‖²       inertial and bias random walk
      + ½ ‖ H_m (δ + x) + b_m ‖²                                                 marginalisation prior
      + ½ w_0² ‖ v_newest ‖²   (only while mpIsStationary and w_0 > 0)           zero velocity
```

The factor of one half is the Ceres convention used throughout (`landmark_block_abs_dynamic.hpp:475`), and the objective therefore has no `2` in its gradient.

### 5.1 Visual residual

Host `h`, target `t`, cameras `c_h`, `c_t`, landmark `ℓ` hosted in `h`.

```
T_t_h  = T_ic_t⁻¹ · T_w_i,t⁻¹ · T_w_i,h · T_ic_h              computeRelPose,  include/basalt/utils/ba_utils.h:45-82
p_t    = T_t_h · p_h                                          4-vector, = R m + ρ t
r      = π_cam(p_t) − z                                       2-vector in pixels, linearizePoint, ba_utils.h:84-135
```

`linearizePoint` returns `false` when the camera model rejects the projection (a point behind the camera, for the models used) or the result is non finite, and `use_valid_projections_only` (default true) then zeroes the residual and Jacobian rows of that observation instead of failing (`landmark_block_abs_dynamic.hpp:162`). A host observation (`tcid_h == tcid_t`) has `T_t_h = I` and `D_h = D_t = 0`, so it constrains only the landmark bearing and contributes no pose information (`linearization_abs_qr.cpp:222-226`).

The Jacobian with respect to a left perturbation `ξ ∈ R⁶` of `T_t_h` (translation, then rotation) and with respect to the landmark are, from `ba_utils.h:112-135`,

```
∂p_t/∂ξ = [ ρ I₃ , −[p_t,xyz]× ]            (the fourth row is zero)
∂r/∂ξ   = J_π · ∂p_t/∂ξ                      2×6
∂r/∂ℓ   = J_π · [ T_t_h[0:3,0:4] · ∂p_h,m/∂u  |  T_t_h[:,3] ]       2×3, columns (u₁, u₂, ρ)
```

The chain rule from state increments to `ξ` is computed once per (host, target) pair per linearisation (`computeRelPose`, `ba_utils.h:45-82`).

```
ξ = D_h δ_h + D_t δ_t
D_h =  Ad( T_ic_t⁻¹ T_t_i_h_i ) · blkdiag(R_h^T, R_h^T)
D_t = −Ad( T_ic_t⁻¹ )           · blkdiag(R_t^T, R_t^T)
```

where `T_t_i_h_i = (R_t^T R_h , R_t^T (p_h − p_t))` is the host IMU pose in the target IMU frame. Both `D` are 6x6 and are read by every landmark of the same host and target.

Robust weighting is the Huber function on the norm of the two vector, in pixels, before whitening.

```
w      = 1                      if ‖r‖ ≤ k,       k / ‖r‖  otherwise          k = vio_obs_huber_thresh (pixels)
cost   = ½ (2 − w) w ‖r‖²                                                      = Huber loss, equal to ½‖r‖² inside k
row scale  s = √w / σ                                                          iteratively reweighted rows
```

The loss value is the true Huber loss. The Jacobian rows are the iteratively reweighted ones, which have the exact gradient of the Huber loss, because the Huber influence function is `w r`, and the Gauss Newton Hessian of the reweighted problem. This is the usual approximation and it is not the exact Huber Hessian.

### 5.2 Inertial residual

One factor `IntegratedImuMeasurement` per consecutive pair of full states, keyed in `imu_meas` by the start time (`:672`). The nine vector residual is documented in [`vio_residuals_and_priors.md`](vio_residuals_and_priors.md) and in `thirdparty/basalt-headers/include/basalt/imu/preintegration.h:221-292`.

```
r_p = R0^T (p1 − p0 − v0 Δt − ½ g Δt²) − (Δp + dg_p + da_p)
r_R = Log( Exp(dg_R) ΔR R1⁻¹ R0 )
r_v = R0^T (v1 − v0 − g Δt)           − (Δv + dg_v + da_v)
dg = J_bg (b_g − b_g,lin),     da = J_ba (b_a − b_a,lin)
```

`ImuBlock::linearizeImu` (`include/basalt/linearization/imu_block.hpp:26-108`) evaluates residual and Jacobians at the linearisation points and then recomputes only the residual value at the current state if either endpoint is frozen (`:50-55`). The nine rows are whitened by `L = get_sqrt_cov_inv()`, the square root of the inverse preintegration covariance obtained from an LDLT (`preintegration.h:337-352`). Six random walk rows for the biases are appended with weights `σ_b⁻¹/√Δt`, built from `calib.gyro_bias_std` and `calib.accel_bias_std` (`gyro_bias_sqrt_weight` and `accel_bias_sqrt_weight` at `src/vi_estimator/sqrt_keypoint_vio.cpp:134-135`, consumed at `imu_block.hpp:76-105`). The result is a 15 row residual `r` and a 15x30 Jacobian `Jp`, whose two 15x15 halves belong to the start and the end state.

```
Jp = [ L ∂r/∂x₀ , L ∂r/∂x₁ ]    rows 0..8     (the end state has no bias columns in the inertial rows)
     [ diag(w_g) ,  −diag(w_g) ] rows 9..11    on the b_g columns (9..11) of start and end
     [ diag(w_a) ,  −diag(w_a) ] rows 12..14   on the b_a columns (12..14)
```

### 5.3 Marginalisation prior

`marg_data` (`MargLinData`, `include/basalt/utils/imu_types.h:317-324`) holds an order `order`, a matrix `H` and a vector `b`. In the default configuration `is_sqrt` is true, so `H` is the square root factor `J_m` with `J_mᵀ J_m = H*`, and `b` is the residual `r_m`, as discussed in `linearisation.md`. The prior cost is

```
P(x) = ½ ‖ J_m (δ + x) + r_m ‖²        δ = current offset of the frozen states from their linearisation points
```

Its linearisation at `x = 0` has Jacobian `J_m` and residual `J_m δ + r_m`, so, from `ba_base.cpp:455-466`,

```
H += J_mᵀ J_m                       top left corner, size marg_size
b += J_mᵀ (J_m δ + r_m)
cost = δᵀ J_mᵀ (½ J_m δ + r_m)       the constant ½ r_mᵀ r_m is dropped, so the cost can be negative
```

Because the constant is dropped, `error_total` can legitimately be negative, which is the reason a negative total in the logs is not a defect.

The initial prior, before any marginalisation, anchors the four unobservable directions (global position and yaw, `:105-116`), see `vio_residuals_and_priors.md` for the weight transposition defect.

### 5.4 Zero velocity term

An experimental extra residual `r = w_0 v` on the newest state (`:1664-1672`, `:1748-1761`, `:1812-1822`, `:1884-1888`) active only when the detector sets `mpIsStationary` and `vio_zero_velocity_weight > 0`. It is injected directly into the dense system rather than through the linearisation framework, and it is not included in `marginalize`, so it never enters the prior. Section 11 uses it as the template for adding a term.

## 6. Constructing the linearisation (`:1585-1607`)

```cpp
lqr_options.lb_options.huber_parameter = huber_thresh;       // pixels
lqr_options.lb_options.obs_std_dev     = obs_std_dev;
lqr_options.linearization_type         = config.vio_linearization_type;   // ABS_QR by default
ImuLinData<Scalar> ild = { g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {} };
for (kv : imu_meas) ild.imu_meas[kv.first] = &kv.second;
lqr = LinearizationBase<Scalar,POSE_SIZE>::create(this, aom, lqr_options, &marg_data, &ild);
```

`LinearizationBase::create` (`src/linearization/linearization_base.cpp:58-91`) dispatches on the enum. `ABS_QR` builds `LinearizationAbsQR` (`src/linearization/linearization_abs_qr.cpp:48-173`), whose constructor does the following, none of it numeric.

1. It asserts that the Huber threshold and `obs_std_dev` equal the estimator's own (`:69-74`), because the estimator recomputes the cost independently through `computeError`.
2. It allocates one `RelPoseLin` per (host, target) pair in `lmdb.getObservations()` (`:77-97`), and builds `relative_pose_per_host` (`:100-112`), a per host list of iterators into that cache.
3. It selects the landmarks. With `used_frames == nullptr` and `lost_landmarks == nullptr`, as in `optimize`, every landmark is selected (`:116-127`).
4. It creates one `LandmarkBlockAbsDynamic` per landmark in parallel with `tbb::parallel_for` and calls `allocateLandmark` (`:132-150`).
5. It records the starting row of each block in the stacked reduced system, `landmark_block_idx`, and the total `num_rows_Q2r` in a sequential loop (`:154-161`). Doing this sequentially is what lets the later dense export write disjoint ranges without locking.
6. It creates one `ImuBlock` per entry of `imu_lin_data->imu_meas` (`:163-168`).

`allocateLandmark` (`include/basalt/linearization/landmark_block_abs_dynamic.hpp:36-108`) sizes the block's dense row major `storage`.

```
rows = 2·n_obs + 3                        two per observation, three reserved for landmark damping
cols = [ pose columns : aom.total_size ] [ pad to a multiple of 4 ] [ landmark : 3 ] [ residual : 1 ]
         0 .. padding_idx-1                padding_size            lm_idx .. lm_idx+2   res_idx
```

The pose columns span the whole window rather than only the frames this landmark touches, so the matrix is structurally sparse and densely stored (the TODO at `:52-55`). The benefit is that one Householder reflection acts on a whole row with no index bookkeeping. `res_idx_by_abs_pose_` records which observation rows touch which frame so that sparse reductions such as `addJp_diag2` can skip zero blocks.

For an observation whose target frame is absent from `aom`, `pose_lin_vec` receives `nullptr` and the row is skipped (`:72-78`). In `optimize` this never happens, since `lmdb` and `aom` cover the same frames, but `marginalize` relies on it (section 12).

## 7. One outer iteration, linearise and eliminate

### 7.1 `linearizeProblem` (`linearization_abs_qr.cpp:184-268`)

1. Reset damping and scaling members (`:188-190`).
2. Relative pose cache (`:193-228`, sequential, because it writes a shared hash map). For every (host, target) with `tcid_h != tcid_t`, call `computeRelPose` with `getPoseLin()` of both frames to obtain `T_t_h`, `D_h`, `D_t`. If either frame is frozen, recompute only `T_t_h` at `getPose()` so that the value sees the current estimate and the Jacobian stays at the linearisation point. Same-frame pairs get `T_t_h = I`, `D = 0`.
3. Landmarks (`:231-253`, `tbb::parallel_reduce`). Each block runs `linearizeLandmark` and the pass accumulates the visual cost and an all-valid flag.
4. IMU blocks (`:255-259`, sequential) add their cost.
5. The prior (`:261-265`) adds `computeMargPriorError`, which is the offset cost of section 5.3.

The return value is the full cost `error_total`. The estimator then asserts `numerically_valid` (`:1632-1634`) and adds the zero velocity cost when active (`:1664-1672`).

### 7.2 `linearizeLandmark` fills the block (`landmark_block_abs_dynamic.hpp:114-201`)

`storage` is zeroed, then for observation `i` of the landmark, with `abs_h_idx`, `abs_t_idx` the columns of host and target in `aom`:

```
storage[2i:2i+2, lm_idx:lm_idx+3] =  s · ∂r/∂ℓ                  2×3
storage[2i:2i+2, res_idx]         =  s · r                      2×1
storage[2i:2i+2, abs_h_idx : +6] +=  s · (∂r/∂ξ) · D_h          2×6
storage[2i:2i+2, abs_t_idx : +6] +=  s · (∂r/∂ξ) · D_t          2×6
```

with `s = √w / σ`. The accumulation operator `+=` matters for a multi camera rig, where host and target cameras differ but share a frame, so `abs_h_idx == abs_t_idx` and the two products land on the same six columns and must add. The returned value is `Σ ½(2−w) w ‖r‖² / σ²`. A non finite Jacobian marks the block `NumericalFailure` (`:163-165`), which `linearizeProblem` ANDs across all blocks.

At this point `storage` is the whitened Jacobian of one landmark stacked over its observations, `[Jp | pad | Jl | r]`.

### 7.3 `performQR`, eliminating every landmark (`:203-216`, `:456-472`, `linearization_abs_qr.cpp:270-280`)

All blocks run in parallel. With `use_householder = true`, the default, `performQRHouseholder` performs three reflections, one per landmark column `k = 0, 1, 2`. Each reflection is computed from the landmark column over rows `k .. num_rows-4` (the last three damping rows are excluded) and applied to every column of `storage`, including the pose columns and the residual column. The result is an orthogonal `Q` with

```
         [ Q1ᵀ ]                  [ Q1ᵀ Jp    R ]  } 3 rows           R is 3×3 upper triangular
Qᵀ J  =  [     ] [ Jp  Jl ]   =  [             ]
         [ Q2ᵀ ]                  [ Q2ᵀ Jp    0 ]  } 2n − 3 informative rows (plus 3 zero damping rows)

Qᵀ r  =  [ Q1ᵀ r ; Q2ᵀ r ]
```

The block now holds `Q1ᵀJp` and `Q1ᵀr` in rows 0 to 2 and `Q2ᵀJp` and `Q2ᵀr` in rows 3 onwards, in place.

The mathematics is the standard null space projection. The landmark appears only in the first three rows, so the cost splits.

```
½ ‖ J_p δp + J_ℓ δℓ + r ‖²  =  ½ ‖ Q1ᵀJp δp + R δℓ + Q1ᵀ r ‖²  +  ½ ‖ Q2ᵀJp δp + Q2ᵀ r ‖²
```

The second term is independent of `δℓ` and is the landmark's contribution to the reduced pose problem, and it equals the Schur complement term `H_pp − H_pℓ H_ℓℓ⁻¹ H_ℓp` in the normal equations, without ever forming `H_ℓℓ` or its inverse. This is the square root marginalisation of Demmel et al. [1], and it is why `performQR` is the step the solver code calls "marginalise points in place" (`:1652`). The first term is zero when `δℓ` is set by back substitution, section 9.

A landmark with fewer than two observations leaves the three landmark columns rank deficient, since each observation contributes two rows. `min_num_obs = 2` prevents this in the database. `use_valid_projections_only` may still zero the rows of an invalid projection, which removes information but does not break the triangular solve for a block that retains two valid observations.

Accounting. `numQ2rows() = num_rows − 3 = 2n`, while the informative count is `2n − 3`. The three surplus rows are the zero damping rows. They keep the exported row count independent of whether damping is applied, see `linearisation.md`.

### 7.4 What is commented out

The upstream solver optionally scaled the Jacobian columns and the landmark columns, and applied Levenberg style damping to poses and landmarks through the linearisation object. All of it is commented out here (`:1534-1535`, `:1640-1651`, `:1681-1693`, `:1705-1731`, `:1829-1832`). Consequences are that `Jl_col_scale` stays at ones, so `inc.array() *= Jl_col_scale` in `backSubstitute` is the identity, that `setLandmarkDamping(0)` inside `backSubstitute` is a no-op placeholder, and that the only damping is the pose side diagonal damping of section 8.2. Landmarks are not damped at all, which the TODO at `:1570-1574` lists as a suspect for accuracy differences against the older Schur loop.

## 8. The reduced system and its solution

### 8.1 Dense assembly (`get_dense_H_b`, `linearization_abs_qr.cpp:536-591`)

```
H = Σ_ℓ (Q2ᵀJp)_ℓᵀ (Q2ᵀJp)_ℓ        +  Σ_k Jp_kᵀ Jp_k (IMU, scattered into start and end blocks)  +  J_mᵀ J_m (prior)
b = Σ_ℓ (Q2ᵀJp)_ℓᵀ (Q2ᵀr)_ℓ        +  Σ_k Jp_kᵀ r_k                                              +  J_mᵀ (J_m δ + r_m)
```

Landmark parts run as a `tbb::parallel_reduce` with per thread dense accumulators (`:539-578`), calling `add_dense_H_b` of each block (`landmark_block_abs_dynamic.hpp:519-525`, `H += Jᵀ J`, `b += Jᵀ r` over rows 3 onwards). The IMU part (`:581`, `add_dense_H_b_imu`) goes through a `DenseAccumulator`. The prior (`:587`) goes through `linearizeMargPrior`. Pose damping (`:584`) adds nothing because `setPoseDamping` is never called.

This is the one place where the square root form is spent. The normal equations square the condition number, which is accepted because the odometry system is rebuilt at the next frame, whereas the prior persists (see `linearisation.md`).

Block structure of `H` for three full states and `P` pose slots.

```
            poses (6P)      s0 (15)    s1 (15)    s2 (15)
poses    [ vis + prior     vis+prior   vis        vis      ]
s0       [   .            vis+prior+imu(0,1)  imu(0,1)  vis ]
s1       [                         imu(0,1)+imu(1,2)  imu(1,2) ]
s2       [                                              imu(1,2) ]
```

Visual terms fill any pair of frames that co-observe a landmark, the IMU terms are block tridiagonal over states, and the prior occupies the top left prefix. Velocity and bias columns receive information only from the IMU factors and the prior, never from vision, which is why the visual system can say nothing about bias except through the pose chain.

### 8.2 Sign convention and zero velocity injection (`:1746-1761`)

The system is `H x + b = 0` with `H = JᵀJ` and `b = Jᵀr`, so the Newton increment is `−H⁻¹ b`. The code solves `inc = H_damped⁻¹ b` and negates it before back substitution (`:1779`, `:1808`). A direct term `r = w_0 v` therefore adds `+w_0² I` to the velocity diagonal block and `+w_0² v` to the velocity segment of `b`, not the negation.

```cpp
const int vi = aom.abs_order_map.at(last_state_t_ns).first + 6;       // velocity block of the newest state
H.block<3,3>(vi, vi).diagonal().array() += zeroVelocityW2;
b.segment<3>(vi)                       += zeroVelocityW2 * vel_w_i;
```

### 8.3 Damped solve (`:1768-1792`)

```cpp
VecX Hdiag_lambda = (H.diagonal() * lambda).cwiseMax(min_lambda);
MatX H_copy = H;  H_copy.diagonal() += Hdiag_lambda;
Eigen::LDLT<Eigen::Ref<MatX>> ldlt(H_copy);
inc = ldlt.solve(b);
```

This is Marquardt damping, `(H + λ diag(H)) inc = b`, with an additive floor of `min_lambda` (default 1e-6) so that zero diagonal entries remain invertible. If `inc` is not finite, `lambda` is multiplied by `lambda_vee`, `lambda_vee` is doubled and the solve is retried, at most three times (`max_num_iter = 3`). After three failures the code only prints `Still invalid inc` to `std::cerr` and continues with the non finite increment (`:1794-1797`). Nothing guards the following `applyInc`, so a persistent failure poisons the window.

`LDLT` of the damped matrix assumes positive semidefiniteness, with the damping lifting any zero eigenvalue. The gauge directions of the visual and inertial terms (global position and yaw) are held by the prior's `vio_init_pose_weight = 1e8` before the first marginalisation, and by the carried prior thereafter, see `vio_residuals_and_priors.md`.

## 9. Back substitution and the predicted decrease (`:1800-1827`)

The solver calls `backup()` (`:1801`), negates `inc`, and calls `lqr->backSubstitute(inc)` (`linearization_abs_qr.cpp:291-322`), which does three things and returns the predicted cost decrease `l_diff`.

### 9.1 Landmarks (parallel, `landmark_block_abs_dynamic.hpp:262-335`)

From the first three rows,

```
R δℓ = − ( Q1ᵀ r + Q1ᵀ Jp δp )            δℓ = −R⁻¹ (Q1ᵀ r + Q1ᵀ Jp δp)       back substitution on a 3×3 upper triangle
direction += δℓ[0:2]                        inv_dist = max(0, inv_dist + δℓ[2])
```

The landmark update is applied immediately to `lmdb`, in place (`:333-334`), which is why `backup()` must capture the landmark parameters and why `restore()` restores them for a rejected step.

### 9.2 Predicted decrease, from the quadratic model

For the whole model `L(δ) = F + δᵀ Jᵀ r + ½ δᵀ JᵀJ δ`,

```
l_diff = L(0) − L(δ) = −(J δ)ᵀ ( r + ½ J δ )
```

For a landmark, `J δ` in the rotated basis is the stack `[ Q1ᵀJp δp + R δℓ ; Q2ᵀJp δp ]`, and the code forms it as `storage.topLeftCorner(num_rows−3, padding_idx) * pose_inc` plus `Q1Jl * δℓ` on the first three rows, then `l_diff −= QJincᵀ (½ QJinc + Qr)` (`:313-320`). The same expression is evaluated for each IMU block (`imu_block.hpp:197-223`) and for the prior (`computeMargPriorModelCostChange`, `ba_base.cpp:505-548`). The zero velocity term is added by hand, `l_diff −= w_0² dvᵀ (v + ½ dv)` (`:1821`), which is the same expression for a residual `r = w_0 v` with Jacobian `w_0 I`.

### 9.3 Cost bookkeeping, the three places a new term must appear

A new residual must be added consistently to all three of

1. `error_total` before the step (`linearizeProblem` return value plus any hand-added term, `:1631`, `:1668-1672`),
2. `l_diff`, the predicted decrease (`backSubstitute` return plus any hand-added term, `:1811-1822`),
3. `after_error_total`, the cost recomputed after the step (`:1881-1888`).

If any of the three omits the term, the ratio `f_diff / l_diff` is computed against a different objective than the one solved, and acceptance becomes unreliable. The zero velocity term was added to all three for this reason, see the comment at `:1660-1663`.

## 10. Applying the step, acceptance and the LM schedule

### 10.1 Applying the increment (`:1834-1847`)

```cpp
for (auto& [id, st] : frame_poses)  st.applyInc(inc.segment<POSE_SIZE>(aom.at(id).first));
for (auto& [id, st] : frame_states) st.applyInc(inc.segment<POSE_VEL_BIAS_SIZE>(aom.at(id).first));
step_norminf = inc.array().abs().maxCoeff();
```

Frozen states accumulate `delta`, unfrozen ones move directly, as in section 2.3. The step norm is the infinity norm over all of `inc`, mixing metres, radians, metres per second and bias units.

### 10.2 Cost after the step (`:1849-1888`)

`computeError` (`ba_base.cpp:140-226`) recomputes the visual cost from `getPoseStateWithLin(...).getPose()`, in parallel over host frames. The variable names are misleading, `after_update_vision_and_inertial_error` holds only the visual error until the IMU terms are added at `:1871-1873`. `computeMargPriorError` adds the prior cost, and `ScBundleAdjustmentBase::computeImuError` (`src/vi_estimator/sc_ba_base.cpp:736-790`) adds the inertial and bias random walk costs. This path is independent of the `LandmarkBlock` code, which makes it a useful cross check, because a disagreement between `error_total` and an `after_error_total` at `inc = 0` indicates a bug in one of the two.

### 10.3 Acceptance (`:1890-1913`)

```
f_diff   = error_total − after_error_total                   actual decrease
ρ_ratio  = f_diff / l_diff                                    gain ratio
valid    = l_diff > 0
success  = valid and ρ_ratio > 0
```

The test accepts any positive actual decrease. The conventional trust region test uses a small positive threshold, so a step that decreases the cost by a numerically negligible amount is accepted here.

### 10.4 Lambda schedule and loop control

```cpp
// success
lambda *= max(1/3, 1 − (2ρ_ratio − 1)³);   lambda = max(min_lambda, lambda);   lambda_vee = 2;   it++;
if ((f_diff > 0 && f_diff < 1e-6) || step_norminf < 1e-4) { converged = true; terminated = true; }
break;                                    // leave the inner loop, relinearise
// failure
lambda *= lambda_vee;  lambda_vee *= 2;   restore();   it++;  it_rejected++;
if (lambda > max_lambda) terminated = true;
```

The success update is the one of Nielsen (1999) [5], also used by Ceres, as the source comments suggest by referring to Ceres, and the failure update is the same paper's doubling schedule. The loop structure is

```
lambda = vio_lm_lambda_initial                              once per optimize() call (:1575)
for (it = 0; it <= vio_max_iterations && !terminated; )     outer, relinearise once per accepted step
    error_total = linearizeProblem();   performQR();
    for (j = 0; ; j++)                                       inner, retries with larger lambda on rejection
        H, b = get_dense_H_b();  [zero velocity];
        inc  = LDLT(H + damping(lambda)).solve(b);
        l_diff = backSubstitute(−inc);   apply;   after_error_total = recompute();
        accept → it++, shrink lambda, break          reject → it++, grow lambda, restore(), continue
```

Three properties follow. The relinearisation and the QR run once per accepted step, while a rejected step reuses the stored `storage` and only re-solves, which is cheap. Rejected steps also consume the iteration budget, since `it++` is in both branches, so with the default `vio_max_iterations = 7` the loop runs at most eight accepted or rejected steps. The `converged` flag is set only on a successful step.

`optimize` logs `reasonCode = converged ? 0 : (terminated ? 3 : 2)` (`:2013`), so code 2 means the iteration budget ran out without convergence and code 3 means `lambda` exceeded `max_lambda`.

Note on `vio_use_lm`. The key exists in `VioConfig` and is serialised but is never read anywhere under `src/` or `include/` apart from its own default and serialisation, so the adaptive schedule above is always active regardless of the flag.

## 11. Extending the optimisation

### 11.1 Adding a residual that involves existing states only

Follow the zero velocity precedent, which touches six places in `optimize` and none in the linearisation classes.

1. Add `½ ‖r‖²` to `error_total` after `linearizeProblem` (`:1668`).
2. Add `JᵀJ` and `Jᵀr` to the right blocks of `H` and `b` immediately after `get_dense_H_b` (`:1746`), at offsets taken from `aom.abs_order_map.at(frame_id).first` plus the within state offset (translation 0, rotation 3, velocity 6, gyroscope bias 9, accelerometer bias 12).
3. Subtract `(Jδ)ᵀ(r + ½Jδ)` from `l_diff` after `backSubstitute` (`:1812`). At that point `inc` has already been negated (`:1808`), so it is the increment that will actually be applied, and it is the vector to use for `δ`.
4. Add the same cost to `after_error_total` (`:1881`).
5. Decide whether the term belongs in the prior. If it does, it must also be added in `marginalize`, otherwise its information is lost at the next marginalisation (section 12.4).
6. Log the new component, and if it is a hard constraint use a weight small enough not to ruin the conditioning of `LDLT` (the zero velocity weight enters squared).

Use the sign convention of section 8.2, so a residual `r(x)` with Jacobian `J` adds `JᵀJ` and `+Jᵀr`.

### 11.2 Adding a new factor type to the linearisation framework

Write a block class analogous to `ImuBlock` (`linearizeImu`, `add_dense_H_b`, `add_dense_Q2Jp_Q2r`, `backSubstitute`), construct it in the `LinearizationAbsQR` constructor next to the IMU blocks (`:163-168`), and call it from the five loops that touch `imu_blocks`, namely `linearizeProblem`, `backSubstitute`, `get_dense_Q2Jp_Q2r`, `add_dense_H_b_imu` and `getJp_diag2`. The `get_dense_Q2Jp_Q2r` path reserves its rows by `imu_lin_data->imu_meas.size() * POSE_VEL_BIAS_SIZE` (`:491-494`), so a factor with a different row count must add its own reservation. Doing this lets the factor enter `marginalize` for free, since `marginalize` builds the same object.

### 11.3 Adding or removing a state block

Every site that enumerates `frame_poses` and `frame_states` assumes the two block sizes 6 and 15. A new state type needs a new entry in `aom`, a new `idx_to_keep` and `idx_to_marg` rule in `marginalize` (`:1290-1319`), a rule for the order of `marg_data.order` (`:1448-1463`), an update in `computeDelta` (`ba_base.cpp:314-335`, which asserts a linearised flag), `backup`/`restore`, and the post update loops in `optimize`. The prefix property of section 4.2 must hold, or the prior needs a permutation.

### 11.4 Changing the landmark parametrisation

The parametrisation is wired into `StereographicParam` and `linearizePoint`. The block width of three is wired into `LandmarkBlockAbsDynamic` (the three Householder passes, the 3x3 `R`, the three damping rows, the clamp on `inv_dist`, and `numQ2rows`), and into the `Keypoint` update at `backSubstitute`. A different parametrisation of the same dimension needs only `linearizePoint`, the `Keypoint` fields and the update lines. A different dimension requires a rewrite of the block.

### 11.5 Reviving the commented machinery

Pose damping through `setPoseDamping`, landmark damping through `setLandmarkDamping`, and Jacobian scaling through `getJp_diag2`/`scaleJp_cols` are implemented but disconnected. Two traps were recorded in `linearisation.md`, the square root pose damping writes only `num_cameras * POSE_SIZE` diagonal entries while the dense damping writes the whole diagonal, and `add_dense_H_b_marg_prior` asserts `marg_scaling.rows() == 0`, so scaling and `get_dense_H_b` cannot be combined while a prior exists.

### 11.6 Invariants to preserve

1. The prior's variables are a prefix of `aom`, in the same order, with the same sizes.
2. Residual values use `getState()` and Jacobians use `getStateLin()` for frozen states.
3. Any state that enters the prior or `frame_poses` is frozen, because `computeDelta` asserts `isLinearized()`.
4. The three cost places of section 9.3 stay consistent.
5. `backup()` precedes the first mutation of a step and `restore()` is the only rollback.
6. Landmarks are whitened by `√w/σ` before the QR, because an orthogonal transform preserves only the Euclidean norm.

## 12. Marginalisation after the solve, `marginalize` (`:1052-1526`)

The user-facing question was how landmarks are marginalised, and there are two distinct eliminations. Section 7.3 eliminates every landmark inside each iteration, with the result discarded after the step. This section describes the permanent marginalisation that turns old states and the landmarks they host into a prior.

### 12.1 What is removed (`:1059-1186`)

1. `last_state_to_marg` is the state `states_to_remove` positions into `frame_states` (`:1062-1066`). It is kept as a full state and frozen. States older than it are removed.
2. `poses_to_marg` starts as every `frame_poses` entry that is not in `kf_ids` (`:1076`).
3. Of the older states, a keyframe state goes to `states_to_marg_vel_bias` (it keeps its pose and loses velocity and biases) and a non keyframe goes to `states_to_marg_all` (`:1091-1096`).
4. While `kf_ids.size() > max_kfs`, a keyframe is chosen to leave, first the oldest one outside the newest two whose tracked fraction `num_points_connected / num_points_kf` is below `vio_kf_marg_feature_ratio`, otherwise by a DSO style score (`:1113-1177`). It is added to `kfs_to_marg` and `poses_to_marg` (`:1182-1183`).

### 12.2 The marginalisation linearisation (`:1195-1247`)

A second `aom` is built over the same containers but truncated at `last_state_to_marg` (`:1068-1109`), so newer states are not part of the system. `LinearizationBase::create` is called with `used_frames = &kfs_to_marg` and `lost_landmarks = &lost_landmaks`, which changes the selection of step 3 in section 6.

```cpp
for (kv : lmdb.getLandmarks())
    if (used_frames->count(host frame) )          select
    else if (lost_landmarks->count(id))           select
```

Only landmarks hosted by a departing keyframe, or lost, enter the marginalisation system. Their observations in frames newer than `last_state_to_marg` are absent from the truncated `aom`, so `pose_lin_vec[i]` is `nullptr` and those rows are zero (`landmark_block_abs_dynamic.hpp:72-78`). IMU factors whose endpoints are not both in the truncated `aom` are skipped (`:1223-1225`). The prior is included, so the old prior is carried into the new one.

`lqr->linearizeProblem()` and `lqr->performQR()` eliminate the selected landmarks, and `get_dense_Q2Jp_Q2r` (`linearization_abs_qr.cpp:482-534`) exports the stacked square root system without forming `H`.

```
rows:  [ Q2ᵀJp of selected landmarks | IMU rows (15 per factor) | pose damping rows (none) | prior rows J_m ]
cols:  aom.total_size (truncated)
rhs :  [ Q2ᵀ r | r_imu | 0 | J_m δ + r_m ]
```

### 12.3 Square root marginalisation (`marg_helper.cpp`, `marginalizeHelperSqrtToSqrt`)

`idx_to_marg` and `idx_to_keep` partition the columns of the truncated system (`:1290-1319`).

| Item | Columns kept | Columns marginalised |
|---|---|---|
| pose in `poses_to_marg` | none | all 6 |
| pose otherwise | all 6 | none |
| state in `states_to_marg_all` | none | all 15 |
| state in `states_to_marg_vel_bias` | pose, 6 | velocity and both biases, 9 |
| `last_state_to_marg` | all 15 | none |

The helper permutes the columns so that marginalised ones come first, then runs Householder QR on `[Q2Jp | Q2r]` with a rank threshold of `√ε` on the Householder pivot magnitude (`marg_helper.cpp`, `rank_threshold`). A column whose pivot falls below the threshold is zeroed and does not advance the rank, and `marg_rank` is the rank reached once the last marginalised column has been processed. After the triangularisation of the marginalised columns, the rows below the marginalised rank contain no marginalised variable, and they are the new prior in square root form.

```
[ R_mm  R_mk | c_m ]            rows 0..marg_rank−1
[  0    J_new | r_new ]         rows marg_rank .. total_rank−1          = (marg_sqrt_H, marg_sqrt_b), at least one row
```

The result satisfies `J_newᵀ J_new = H_kk − H_km H_mm⁻¹ H_mk`, the Schur complement, while never squaring the condition number [1]. The squared variants `marginalizeHelperSqToSqrt` and `marginalizeHelperSqToSq` are the dense paths used when `ABS_QR` or `vio_sqrt_marg` is off, and use a pseudo inverse from a complete orthogonal decomposition (`marg_helper.cpp`).

### 12.4 Bookkeeping afterwards (`:1416-1469`)

1. `frame_states.at(last_state_to_marg).setLinTrue()` freezes the surviving oldest state (`:1419`). It asserts it was not frozen before.
2. `states_to_marg_all` leave `frame_states`, `imu_meas` and `prev_opt_flow_res`. `states_to_marg_vel_bias` become `frame_poses` entries through `PoseStateWithLin(state)` and leave `frame_states` and `imu_meas` (`:1422-1435`).
3. `poses_to_marg` leave `frame_poses` and `prev_opt_flow_res` (`:1437-1440`).
4. `lmdb.removeKeyframes(...)` deletes the landmarks hosted by departing keyframes and strips the observations of departing frames, and lost landmarks are removed (`:1442-1446`). Landmarks hosted by a surviving keyframe lose their observations in a departing non keyframe without those observations ever entering the prior. By this reading of `removeKeyframes` and of the landmark selection in section 12.2, that information is discarded rather than marginalised.
5. `marg_order_new` is rebuilt as the surviving poses followed by `last_state_to_marg` (`:1448-1463`), then `marg_data.H`, `marg_data.b` and `marg_data.order` are replaced.
6. `marg_data.b -= marg_data.H * delta` (`:1494-1496`). The helper returned a residual linearised at `x = 0`, which already includes the offset `δ`, and this subtraction restores the delta independent form `P(x) = ½‖J_new(δ + x) + (r_new − J_new δ)‖²`, derived in the comment block at `:1471-1492`.

The marginalised prior is also published to `out_marg_queue` as a `MargData` with the dense `H = Q2Jpᵀ Q2Jp` rebuilt for the mapper (`:1258-1288`).

### 12.5 After `marginalize`

`PublishKeyframe` (`:2033-2054`) ships the newest state's pose, rebuilt with `linearized = false`, and its optical flow to the local mapper through `mpKFOutputQueue`. Mapper pose corrections return through `QueuePoseUpdates` and are applied at the top of the next `measure`, only to `frame_poses`, only below `kRelinThresholdTrans = 0.10` m and `kRelinThresholdRot = 0.05` rad (`:591-651`). The replacement `PoseStateWithLin(t_ns, T_new, was_lin)` makes `T_new` the new linearisation point with `delta = 0`. The prior was built at the old point, so the correction silently re-anchors the prior. The thresholds bound the inconsistency, they do not remove it.

## 13. Reference table of default parameters

Library defaults at `src/utils/vio_config.cpp:58-104`. The shipped `data/*.json` files may override them.

| Parameter | Default | Role in this document |
|---|---|---|
| `vio_linearization_type` | `ABS_QR` | selects `LinearizationAbsQR`, the square root path |
| `vio_sqrt_marg` | true | prior stored as `J_m`, `r_m` |
| `vio_max_states` | 3 | full states in the window |
| `vio_max_kfs` | 7 | keyframes in the window |
| `vio_max_iterations` | 7 | outer limit, `it <= 7` gives up to 8 steps |
| `vio_lm_lambda_initial` / `min` / `max` | 1e-4 / 1e-6 / 1e2 | damping start, additive floor, abort threshold |
| `vio_obs_std_dev` | 0.5 | `σ` in pixels |
| `vio_obs_huber_thresh` | 1.0 | Huber `k` in pixels |
| `vio_new_kf_keypoints_thresh` | 0.7 | keyframe trigger on tracked fraction |
| `vio_min_frames_after_kf` | 5 | keyframe cool down |
| `vio_kf_marg_feature_ratio` | 0.1 | keyframe marginalisation rule |
| `vio_min_triangulation_dist` | 0.05 | metres of baseline to triangulate |
| `vio_marg_lost_landmarks` | true | marginalise landmarks with no current observation |
| `vio_init_pose_weight` | 1e8 | gauge anchor of the initial prior |
| `vio_init_ba_weight` / `bg_weight` | 1e1 / 1e2 | transposed onto the opposite bias blocks, see `vio_residuals_and_priors.md` |
| `vio_use_lm` | false | read nowhere, inert |
| `vio_scale_jacobian` | true | the scaling code is commented out, inert |
| `vio_zero_velocity_weight` | 0.0 | zero velocity term disabled |

## 14. Defects, hazards and open questions

All are readings of the source and none was reproduced.

1. A non finite increment after three retries is only printed and then applied (`:1794-1797`).
2. `lambda_vee` is not reset at the start of `optimize`, only on a successful step (`:1960`). A call that ends on a rejected step starts the next call with an inflated `lambda_vee`, which affects only how fast `lambda` escalates on the first rejection.
3. The acceptance test is `ρ_ratio > 0` (`:1912`), without a positive threshold.
4. The `numerically_valid` assertion (`:1632`) is fatal only when assertions are enabled (`BASALT_DISABLE_ASSERTS`), see `vio_optical_flow_instrumentation.md`. With assertions disabled, a non finite Jacobian proceeds into the solve.
5. `filterOutliers` is not called inside the loop (TODO at `:2009`), so robustness rests on the Huber weighting alone.
6. Landmarks are undamped, and the pose Jacobian scaling is disabled (section 7.4).
7. Observations of surviving landmarks in departing non keyframes are dropped, not marginalised (section 12.4). Observations of marginalised landmarks in frames newer than `last_state_to_marg` are dropped from the marginalisation system (section 12.2).
8. The assertion in `computeDelta` that every prior variable is frozen holds in steady state because each round demotes the previous round's `last_state_to_marg`. For the first round, which removes `frame_states.size() - max_states + 1 = 3` states at the default sizes, it appears to rest on the initial state being the only keyframe among the removed states, since `take_kf` starts true and `vio_min_frames_after_kf` is 5. This is an inference from reading, not an observed run.
9. The mapper pose correction re-anchors the prior (section 12.5).
10. The initial prior bias weights are swapped relative to their names, recorded in `vio_residuals_and_priors.md` and `linearisation.md`.

## 15. Extension notes from the gravity and scale plan (2026-11-04)

These were found while writing [`../plans/gravity_scale_optimisation.md`](../plans/gravity_scale_optimisation.md), which adds a global gravity direction and log scale block to the window. They apply to any new variable or factor, not only to that plan. Nothing here was executed.

### 15.1 Adding a variable that is not a frame

`AbsOrderMap` is keyed by `int64_t` and nothing requires a key to be a timestamp, so a global block can use a sentinel key such as −1, which `std::map` orders before every timestamp. The prefix invariant of §4.2 then decides its position. If the block is ever part of the marginalisation prior it must come first in both the optimisation order and the prior order, otherwise the prior is no longer a prefix and the assertions at `sqrt_keypoint_vio.cpp:1550`, `:1562` and `ba_base.cpp:417-422` fire. The check `aom.items < marg_data.order.abs_order_map.size()` at `:1562` and `:1098` counts items, so a block counted in both orders keeps it consistent.

Four places assume every order entry has size 6 or 15 and need a new branch. They are `computeDelta` (`ba_base.cpp:315-332`, `BASALT_ASSERT(false)` otherwise), the `idx_to_keep`/`idx_to_marg` loop of `marginalize` (`:1290-1319`, which asserts `POSE_VEL_BIAS_SIZE` in its else branch), `marg_order_new` (`:1448-1463`, rebuilt from `frame_poses` and `last_state_to_marg` only), and the increment loops of `optimize` (`:1835-1844`), which walk `frame_poses` and `frame_states` rather than `aom`. `backup()` and `restore()` (`ba_base.h:130-140`) cover frames and landmarks only. The `MargData` pushed to `out_marg_queue` (`:1249-1279`) carries `aom` to `NfrMapper`, which knows only frames.

The initial prior's order is not written in the constructor. It is written in the two `initialize` paths, `:160-162` for the five argument overload and `:330-333` for the lazy bootstrap, while the constructor (`:93-116`) writes only the 15×15 `H` and `b`. A block added to the initial prior must therefore shift the constructor's pins by its size when the order is first written.

### 15.2 Landmark blocks pick up new columns for free

`LandmarkBlockAbsDynamic` allocates `aom.total_size` pose columns (`landmark_block_abs_dynamic.hpp:85-87`), so any block placed in `aom` already has storage in every landmark block. Writing a Jacobian into those columns in `linearizeLandmark` is enough for the QR, both dense exports and the back substitution to include it, since `backSubstitute` multiplies `storage.topLeftCorner(num_rows − 3, padding_idx)` by the whole increment (`:313-320`). The whitening factor is already applied to `d_res_d_xi` at `:181`, before the pose blocks are written at `:182-185`. The chain from a variable to the visual residual belongs in `RelPoseLin` (`landmark_block.hpp:13-22`), which today holds `T_t_h`, `d_rel_d_h` and `d_rel_d_t` per host and target pair.

### 15.3 Extension seams in the linearisation

`ImuBlock` has no virtual methods (`imu_block.hpp`), and `LinearizationAbsQR` stores `std::unique_ptr<ImuBlock<Scalar>>` (`linearization_abs_qr.hpp:37`), as do `LinearizationAbsSC` and `LinearizationRelSC`. A new inertial factor type is therefore added most cheaply by making the six methods the linearisation calls virtual and deriving from `ImuBlock`, after which every existing loop over `imu_blocks` dispatches to it unchanged. `ImuBlock::add_dense_H_b` forms `Jpᵀ Jp` from the whole `Jp` and reads blocks at fixed offsets (`:135-160`), so a derived block with a wider `Jp` must not call it unaltered.

The relative pose loop sits inline in `linearizeProblem` (`linearization_abs_qr.cpp:193-228`), so changing how `T_t_h` and its Jacobians are formed requires either editing that loop or extracting it into a virtual method. `LinearizationBase::create` (`linearization_base.cpp:58-91`) dispatches on an enum, and a variant used only in some frames can be constructed directly with `std::make_unique` instead of being added to the enum.

`computeRelPose` is called at `linearization_abs_qr.cpp:208` and `:217`, at `ba_base.cpp:84` (`optimize_single_frame_pose`, VO only), `:167` (`computeError`, the after step cost) and `:378` (`computeProjections`, visualisation), and at `sc_ba_base.cpp:278`, `:287`, `:402` and `:408` (the Schur complement linearisations). `IntegratedImuMeasurement::residual` is called at `imu_block.hpp:44` and `:51` and at `sc_ba_base.cpp:761` (`computeImuError`, the after step cost), and `predictState` only at `sqrt_keypoint_vio.cpp:660`. A change to either residual must reach the after step cost as well, by §9.3.

### 15.4 A parallel prior already exists

The debug path at `sqrt_keypoint_vio.cpp:1323-1384` builds a second prior, `nullspace_marg_data`, from the same truncated `aom`, `kfs_to_marg` and `lost_landmaks` but a different starting prior, and re-centres it with its own `computeDelta` (`:1498-1502`). It is the template for any scheme that maintains several priors in step, such as the dynamic marginalisation of VI-DSO, and it shows that the `idx_to_keep` and `idx_to_marg` sets can be shared across them.

### 15.5 Global similarity variables are gauge freedoms

Any variable that applies a global similarity to the inertial residual, such as a scale on positions and velocities or a rotation of gravity, is redundant with the frame states in the joint problem. Scaling every position and velocity by `k` against a scale variable divided by `k`, or rotating the whole world about a horizontal axis together with the gravity rotation, leaves every residual invariant. The initial prior does not fix these directions, because it pins the position of a first state that sits at the origin and the rotation about the vertical only. Such a variable therefore needs an explicit gauge treatment, and it changes the estimate only through how the marginalisation prior holds old information. The derivation is §2.5 of the plan.

DSO's projector in practice (added 2026-11-05). `EnergyFunctional::orthogonalize` in DM-VIO's copy of DSO (`src/dso/OptimizationBackend/EnergyFunctional.cpp:784-829`) forms $\mathbf{N}(\mathbf{N}^\top\mathbf{N})^{-1}\mathbf{N}^\top$ through a truncated SVD, with $\mathbf{N}$ the six pose and one scale null vectors obtained by central differences of a global left perturbation of each frame's linearisation point (`src/dso/FullSystem/HessianBlocks.cpp:80-100`, assembled in `FullSystemOptimize.cpp:704-757`). Under the default `setting_solverMode = SOLVER_ORTHOGONALIZE_X_LATER` (`src/dso/util/settings.cpp:81`) only the step is projected, from iteration index 2 on (`EnergyFunctional.cpp:980-984`). Projection of $\mathbf{H}$ and $\mathbf{b}$ is the separate `SOLVER_ORTHOGONALIZE_SYSTEM` mode, applied only when the first frame and its prior have left the window (`:876-890`). Do not cite DSO as projecting the system by default. The projected step is the minimum norm member of the family of equally good steps, so it fixes how a correction is split between gauge related variables, not the correction itself. `plans/gravity_scale_optimisation.md` §2.5 derives this for the gravity and scale block.

### 15.6 Scale in the visual residual and the extrinsic lever arm

With metric extrinsics and metric inverse distances, a scale `s` on the estimator world enters the visual residual only through the IMU to IMU relative translation, `T_t_h = T_i_c_t⁻¹ (R_tᵀR_h, s R_tᵀ(p_h − p_t)) T_i_c_h`, and a gravity rotation cancels. The scale column then satisfies `∂r/∂σ = ρ ∂r/∂ρ − ρ J_π c`, with `c` the lever arm contribution to the camera translation. Without a lever arm the per landmark QR annihilates the scale column, so monocular vision gives scale no information, and back substitution keeps `ρ s` fixed to first order. A lever arm, or two cameras of one rig, makes scale visible to vision.

### 15.7 Outputs that would need a frame transform

States leave the estimator at `out_state_queue` (`:965-970`), the visualisation payload (`:972-1000`, states, frames, points from `get_current_points` and projections from `computeProjections`), the returned `PoseVelBiasState` (`:1012-1013`), the logger's `AddVioState` (`:953-960`), `PublishKeyframe` to the local mapper (`:2035-2054`) and `out_marg_queue` (`:1249-1279`). Mapper corrections come back at `:592-641`. Any change that makes the stored world differ from the published world must cover all of them.

### 15.8 What the local mapper holds in world coordinates, and the epoch contract (2026-11-04)

Any change that moves the estimator's world, such as the per frame gravity and scale re-centring of `../plans/gravity_scale_optimisation.md` §2.7, must move the local mapper's world identically. The mapper's world frame state is `frame_poses` (keyframe IMU poses, never frozen, since `NfrMapper::optimize` asserts `!isLinearized()` at `nfr_mapper.cpp:337` and `:410`), the landmarks in its own `lmdb` (host camera bearing plus metric inverse distance, so a world scaling by `s` divides `inv_dist` by `s`), `rel_pose_factors` (`RelPoseFactor`, a relative pose `T_i_j` whose translation scales with the world and whose information transforms as `D_s⁻¹ Λ D_s⁻¹` with `D_s = blkdiag(sI₃, I₃)`), and `roll_pitch_factors` (`RollPitchFactor`, an absolute attitude `R_w_i_meas` that rotates with the world). `MatchData::T_i_j` written by `match_stereo` (`nfr_mapper.cpp:562`) is the fixed metric stereo extrinsic and is unaffected, and the track store and bag of words index hold no geometry.

The mapper cycle in `LocalMapper::MapLocally` (`local_mapper.cpp:83-260`) blocks on one keyframe, ingests it, then drains every pending `MargData` packet, then runs matching, track building, bundle adjustment, culling and the pose update callback. A world correction applied right after the blocking pop therefore keeps the whole cycle in one world. Keyframes, packets and pose updates cross threads asynchronously, so each must carry the world it is expressed in. The plan does this with a cumulative correction log and an epoch stamp on `Keyframe`, `MargData` and the pose update callback.

`NfrMapper::processMargData` (`nfr_mapper.cpp:86-145`) aborts on any order entry whose size is neither 6 nor 15 (`:122-125`), and it copies pose entries into the new order with their original start index (`aom_new.abs_order_map.emplace(kv)`) while giving state entries fresh offsets. It is therefore correct only when the packet's order begins with frame entries at column 0. A packet whose order holds a leading non frame block, such as the gravity and scale block, must have that block removed by conditioning on its current value, which deletes its rows and columns, and the remaining offsets shifted before it is pushed (§15.10). Note, 2026-11-05. This sentence previously said the block must be marginalised out, which leaves the packet rank deficient and makes the mapper drop it, as §15.10 records.

## Correction history

2026-10-30. This file is new. While writing it, the claim in `linearisation.md` that `vio_use_lm` false makes the outer loop plain Gauss Newton with diagonal regularisation was found to be wrong and was corrected in that file in place. The loop in `optimize` always applies the adaptive `lambda` schedule of section 10.4.

## References

[1] N. Demmel, D. Schubert, C. Sommer, D. Cremers, V. Usenko, "Square Root Marginalization for Sliding-Window Bundle Adjustment", ICCV 2021. The ICCV 2021 paper is the origin of the `LinearizationAbsQR` square root path. Cited from memory, not re-read in this session. The repository's own narrative treatment is `doc/Linearisation.md` and `doc/Marginalisation.md`.

[2] V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers, "Visual-Inertial Mapping with Non-Linear Factor Recovery", arXiv:1904.06504v3, 2020. The VIO formulation, the IMU preintegration residual and the keyframe window are described here. The title and authors were confirmed from the first page of the repository copy `1904.06504v3.pdf`.

[3] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", IEEE Transactions on Robotics, 2017. The comparison form of the rotation residual, as discussed in `vio_residuals_and_priors.md`. Cited from memory.

[4] G. H. Golub and C. F. Van Loan, "Matrix Computations", 4th edition. Named in the source at `landmark_block_abs_dynamic.hpp:445-446` as the origin of the Givens QR loops, which the default Householder path replaces. Confirmed from the source comment only.

[5] H. B. Nielsen, "Damping Parameters in Marquardt's Method", Technical Report IMM-REP-1999-05, Technical University of Denmark, 1999. The update `lambda *= max(1/3, 1 − (2ρ−1)³)` with doubling on rejection is recognised as this schedule. Cited from memory, and the match to the paper's constants was not checked against the paper in this session.

### 15.9 VI-DSO dynamic marginalisation, as read from the paper (2026-11-05)

The text of VI-DSO [arXiv:1804.05625] was extracted with `pdftotext -layout` (present in the container at `/usr/bin/pdftotext`), after `WebFetch` of the PDF returned only compressed streams. Section III-F.3 states the motivation in one sentence, that marginalisation fixes the linearisation points of connected states, which is acceptable for every variable except scale. The update sets are $M_\text{curr}$ from $G_\text{metric} \cup G_\text{visual} \cup M_\text{curr}$, $M_\text{half}$ from $G_\text{metric} \cup G_\text{visual} \cup M_\text{half}$, and $M_\text{visual}$ from $G_\text{visual} \cup M_\text{visual}$, where $G_\text{metric}$ holds the inertial factors (eq. 12-13). The paper asserts without proof that the rule keeps the scale ratio inside $M_\text{curr}$ below $d_i^2$, and that eq. 17 prevents $M_\text{curr}$ from being left without inertial factors. Both follow in a few lines, derived in `plans/gravity_scale_optimisation.md` §2.6.6. Eq. 17 is printed with the condition $s_i/s_{i-1} < d_i$, which needs an absolute value in log space to constrain a decreasing scale. With $d_\text{min} = \sqrt{1.1}$ the bound $d_i^2 = 1.1$ limits the error of a first estimate Jacobian that contains $s$ to 10 % and the second order error of the scale dependence to about 0.5 %. The paper contains no ablation comparing dynamic marginalisation with a single prior. Its only evidence for the technique in isolation is the qualitative Fig. 4 on EuRoC V1_03.

### 15.10 The mapper rejects rank deficient packets (2026-11-05)

`NfrMapper::extractNonlinearFactors` (`src/vi_estimator/nfr_mapper.cpp:157-162`) factors `abs_H` with a full pivot QR and returns `false`, dropping the packet without factors, whenever the rank is below the number of columns. The local mapper calls it for every packet (`local_mapper.cpp:312`). Any change that removes variables from a `MargData` packet must therefore keep `abs_H` of full rank. Marginalising out a variable that is gauge related to the frames, such as a global scale or gravity direction, makes the frames' marginal singular along the frames only part of each gauge direction and loses the packet, while conditioning on the variable's current value, which for a packet linearised as increments from the current estimate is deleting its rows and columns, keeps the rank. `plans/gravity_scale_optimisation.md` §2.9.3 derives this.

### 15.11 The gravity and scale period as implemented (2026-11-05)

`../plans/gravity_scale_optimisation.md` Changes 1 to 16 landed on 2026-11-05 and pass the in container syntax check of every touched translation unit, with both `BASALT_INSTANTIATIONS_DOUBLE` and `BASALT_INSTANTIATIONS_FLOAT` instantiated. `doc/VIO.md` §3.3 summarises the formulation. The facts below are the ones a later session needs and cannot read off a single file.

The block lives in `BundleAdjustmentBase::mGsState`, an `std::optional<GravityScaleStateWithLin<Scalar>>` that is empty outside the period, and `backup` and `restore` copy it beside the frames, so a rejected Levenberg–Marquardt step also restores the block. Its key `kGravityScaleStateId = -1` sorts before every timestamp in `AbsOrderMap::abs_order_map`, so iterating an order visits the block first. `computeDelta` returns the block's frozen offset `GetDelta()`, which is zero until the first marginalisation of the period freezes it through `AnchorGs`.

`ImuBlock`'s six linearisation methods are now virtual and its bias random walk rows live in `LinearizeBiasRandomWalk`, and `LinearizationAbsQR::linearizeProblem` calls the virtual `linearizeRelPoses`. These are the two seams §15.3 found missing. `LinearizationGSAbsQR` uses both and is constructed directly in `optimize` and `LinearizeMargGS`, so `LinearizationBase::create` and its enumeration are untouched. It replaces the base's `imu_blocks` after base construction, which is safe because the base constructor reads nothing from them.

The gauge projector is applied twice per solve in `optimize`, to $\mathbf{H}$ and $\mathbf{b}$ after the zero velocity term and to the solution after the retry loop. Its null vectors use linearisation point values and unscaled coordinates, which is correct only while the Jacobi column scaling at `optimize` stays commented out.

`RecentreGravityScale` runs as the last statement of every `optimize` in the period and appends one epoch to `VioEstimatorBase::mGsCorrectionLog`, so the log grows by one entry per frame for `vio_gs_duration_s` seconds and then stops. The log outlives the period, so mapper updates stamped with an old epoch are still carried forward. Because every state is re-centred to $\mathbf{d} = \hat{\mathbf{g}}$ and $\sigma = 0$ before `optimize` returns, every later consumer in the frame, including `computeProjections`, `filterOutliers`, the published state and the keyframe sent to the mapper, sees a metric world with scale 1 and needs no change.

The debug null space prior `nullspace_marg_data` cannot hold the block, and `checkNullspace` aborts on any order entry whose size is neither 6 nor 15 (`sc_ba_base.cpp:600-602`). All four `vio_debug || vio_extended_logging` guards in `SqrtKeypointVioEstimator`, including the `logMargNullspace` call at initialisation in `ProcessFrame`, therefore carry `&& !mpGsActive`, and `FoldGravityScale` copies the folded `marg_data` into `nullspace_marg_data` so that the diagnostic resumes on a prior of matching size. After the period that diagnostic measures the prior including the initial pose and yaw information. The plan as approved suspended only the three guards in `marginalize`, and the omission was found while implementing it. `data/sitl_config_vo.json` sets `vio_debug` true, so the omission would have aborted the first SITL run with the mode enabled.


### 15.12 Revision 2 of the gravity and scale period (2026-11-06)

`../plans/gravity_scale_optimisation_2.md` adds a fourth frame local term to `optimize`, the random walk residual on the block step, which follows the zero velocity term at every one of its sites (`error_total`, the dense `H` and `b` before `ComputeGaugeProjector`, `l_diff` after the increment is negated, `after_error_total`) and keeps its accumulated increment `gsJump` in a local that only the accepted branch advances. `LinearizeMargGS` now passes inertial data to the $M_\text{visual}$ linearisation when `vio_gs_visual_prior_inertial` is set, through two trailing constructor arguments of `LinearizationGSAbsQR` that put `ImuBlockGS` into its scale free mode, and the estimator keeps a third anchor `mpAnchorVisual`, re-expressed by `RecentreGravityScale` and copied with the current scale into $M_\text{half}$ by `ResetHalfFromVisual`. `vio_gravity_scale.csv` gained `jump_cost`, `visual_imu_full` and `visual_imu_rot`.
