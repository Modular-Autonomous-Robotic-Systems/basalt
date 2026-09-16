# Visual-Inertial Odometry in Basalt

## 1. Introduction

Visual-Inertial Odometry (VIO) is the task of estimating the six-degree-of-freedom ego-motion of a rigid body equipped with one or more cameras and an Inertial Measurement Unit (IMU). By tightly coupling visual feature tracks with inertial measurements, VIO systems recover metric-scale trajectories in environments where neither modality alone is sufficient: cameras fail under low-texture, high dynamic range, or rapid motion; IMUs integrate noise into unbounded drift when used alone. The complementary sensing properties make visual-inertial fusion the dominant paradigm for real-time state estimation on mobile robots, drones, and head-mounted devices.

Basalt (Usenko et al., 2020) is a modular, open-source VIO/SLAM framework built around a fixed-lag smoother operating on a sliding window of keyframes and recent inertial states. The estimator solves a non-linear least squares problem whose cost aggregates three factor types: visual reprojection residuals defined on a sparse set of tracked landmarks, IMU preintegration residuals connecting consecutive states, and a marginalisation prior that compresses all information from previously evicted states. The system is implemented as a producer–consumer pipeline: an optical-flow frontend delivers feature tracks on a concurrent queue (`tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr>`), the IMU stream feeds another queue, and the estimator drains both inside a dedicated processing thread.

At the source level, the VIO estimator is realised by the template class `SqrtKeypointVioEstimator<Scalar>` (`include/basalt/vi_estimator/sqrt_keypoint_vio.h`, `src/vi_estimator/sqrt_keypoint_vio.cpp`). It inherits `VioEstimatorBase<Scalar>` for the queue/lifecycle interface and `SqrtBundleAdjustmentBase<Scalar>` for shared bundle-adjustment utilities. The square-root form (Demmel et al., 2021) maintains the marginalisation prior as a Jacobian factor `J` rather than an information matrix `J^T J`, doubling the effective numerical precision.

The rest of this document dissects the VIO pipeline as orchestrated by `SqrtKeypointVioEstimator::ProcessFrame` and `SqrtKeypointVioEstimator::measure` into the following successive stages:

- **§2 Bundle Adjustment (Visual Odometry)** — the visual-only foundation of the estimation pipeline: optical-flow-based feature tracking overview, the mathematical derivation of bundle adjustment from first principles, the reprojection residual, the code-level implementation of the VO estimator (`SqrtKeypointVoEstimator<Scalar>`), and a discussion of VO's inherent limitations that motivate the addition of inertial sensing.
- **§3 VIO Frame to Frame Tracking** — the inertial extension of §2: the sliding-window state vector, the IMU preintegration and bias random-walk residuals and their Jacobians, the marginalisation residual, the complete VIO objective and its normal equations, followed by a code-level walkthrough of the estimator (`SqrtKeypointVioEstimator<Scalar>`).
- **§4 IMU Preintegration** — accumulation of high-rate inertial samples between two image timestamps into a compact pseudo-measurement with propagated covariance and bias Jacobians (`IntegratedImuMeasurement<Scalar>`).
- **§5 State Prediction** — use of the preintegrated pseudo-measurement to forecast the current-frame state from the previous estimate, providing a warm start to the optimizer (`IntegratedImuMeasurement::predictState`).
- **§6 Keyframe Selection** — the feature-tracking-ratio heuristic that decides when a new keyframe is inserted into the sliding window.
- **§7 Triangulation** — geometric initialization of new landmarks from stereo or temporal baselines and the pruning of lost landmarks (`BundleAdjustmentBase::triangulate`).
- **§8 Optimisation and Marginalisation** — the Levenberg–Marquardt loop, residual and Jacobian assembly, and the marginalisation step that keeps the active state vector bounded. Detailed treatment of marginalisation is deferred to `doc/Marginalisation.md`.
- **§9 Key Classes and Interfaces** — consolidated reference of the types, methods, and fields exercised by the previous sections.

Throughout, file/line references point to the canonical implementation in this repository (`/ws/ros_ws/src/slam/ext/basalt`).

---

## 2. Bundle Adjustment (Visual Odometry)

Bundle adjustment (BA) is the joint non-linear refinement of camera poses and 3D landmark positions by minimising the sum of squared reprojection errors across all observations (Triggs et al., 2000). The name originates from the "bundles" of light rays connecting each 3D point to its image projections: BA adjusts both the ray origins (camera poses) and the ray targets (landmarks) until the rays are maximally self-consistent. In the context of real-time odometry, BA constitutes the *visual-only* estimation core — the foundation upon which IMU preintegration (§4), marginalisation (`doc/Marginalisation.md`), and the full VIO cost function are layered. Understanding BA in isolation is therefore a prerequisite for understanding the complete VIO system.

Basalt implements visual odometry as the class `SqrtKeypointVoEstimator<Scalar>` (`include/basalt/vi_estimator/sqrt_keypoint_vo.h`, `src/vi_estimator/sqrt_keypoint_vo.cpp`). It shares the same `SqrtBundleAdjustmentBase<Scalar>` infrastructure and marginalisation machinery as the VIO estimator but operates without inertial measurements: the state vector contains only 6-DoF camera poses (no velocity or biases), and motion prediction between frames reduces to a constant-position model. This section derives the BA problem from geometric first principles, describes how Basalt's optical-flow frontend provides the measurements that drive it, details the implementation, and concludes with an analysis of the structural limitations of vision-only estimation.

### 2.1 VO/VIO Front End

The *front end* of the VIO/VO pipeline is responsible for converting raw images into a set of sparse 2D feature correspondences across frames. These correspondences, combined with the camera's intrinsic calibration, provide the bearing measurements that anchor the bundle adjustment back end. Basalt's frontend is based on **patch-based KLT optical flow** (Lucas and Kanade, 1981; Shi and Tomasi, 1994; Bouguet, 2001) rather than the detect-describe-match paradigm used by systems like ORB-SLAM (Mur-Artal et al., 2015). The optical-flow approach eliminates the descriptor computation and brute-force matching overhead, trading it for a tighter coupling between tracking and the image intensity signal.

The execution flow of the optical flow frontend is implemented in `FrameToFrameOpticalFlow` (`include/basalt/optical_flow/frame_to_frame_optical_flow.h`) via `FrameToFrameOpticalFlow::processFrame`. The pipeline branches depending on whether the system is initializing the very first frame at timestamp $t = 0$ (pyramid building, feature detection, keypoint addition, stereo matching, and epipolar filtering) or processing an arbitrary subsequent frame at timestamp $t > 0$ (pyramid building, temporal multi-level tracking with forward-backward consistency, feature detection in empty grid cells, stereo matching, and epipolar filtering). The following ASCII flowchart outlines all method calls and operations executed during both stages:

```
                    FrameToFrameOpticalFlow::processFrame(curr_t_ns, new_img_vec)
                                                 │
                                                 ▼
                                      Is first frame? (t = 0)
                                                 │
                          ┌──────────────────────┴───────────────────────┐
                          │ (Yes: t = 0)                   (No: t > 0)   │
                          ▼                                              ▼
    ┌───────────────────────────────────────────┐  ┌───────────────────────────────────────────┐
    │           FIRST FRAME (t = 0)             │  │        SUBSEQUENT FRAME (t > 0)           │
    ├───────────────────────────────────────────┤  ├───────────────────────────────────────────┤
    │ 1. Image Pyramid Construction             │  │ 1. Image Pyramid Construction             │
    │    ManagedImagePyr::setFromImage()        │  │    ManagedImagePyr::setFromImage()        │
    │    - Multi-scale image pyramid for each   │  │    - Store old_pyramid = pyramid          │
    │      camera stream (tbb::parallel_for)    │  │    - Build pyramids for current frame     │
    │    - Store input_images for visualization │  │    - Store input_images for visualization │
    │                                           │  │                                           │
    │ 2. Temporal Feature Tracking (Skipped)    │  │ 2. Temporal Feature Tracking              │
    │    - No prior frame available at t = 0    │  │    FrameToFrameOpticalFlow::trackPoints() │
    │    - Skips temporal trackPoints()         │  │    ├─ trackPoint() (forward tracking)     │
    │    - Seeds new keypoint tracks from       │  │    │  - Coarse-to-fine pyramid loop       │
    │      subsequent feature detection         │  │    │  - trackPointAtLevel()               │
    │    - Initializes empty transform map      │  │    │    * PatchT::residual (SE(2) error)  │
    │      for all calibrated camera streams    │  │    │    * Gauss-Newton patch alignment    │
    │    - Prepares state for next frame t > 0  │  │    └─ trackPoint() (reverse consistency)  │
    │                                           │  │       - Reverse track (curr -> old)       │
    │ 3. Feature Detection & Addition           │  │       - Discard if distance > threshold   │
    │    FrameToFrameOpticalFlow::addPoints()   │  │                                           │
    │    ├─ detectKeypoints()                   │  │ 3. Feature Detection & Addition           │
    │    │  - FAST corner detection on grid     │  │    FrameToFrameOpticalFlow::addPoints()   │
    │    │  - Sub-pixel corner refinement       │  │    ├─ detectKeypoints()                   │
    │    │  - Assign new unique KeypointId      │  │    │  - Detect in unoccupied grid cells   │
    │    │  - Add to observations[0] (Cam 0)    │  │    │  - Assign new unique KeypointId      │
    │    └─ trackPoints() [if stereo]           │  │    │  - Add to observations[0] (Cam 0)    │
    │       - Track features Cam 0 -> Cam 1     │  │    └─ trackPoints() [if stereo]           │
    │       - Add matches to observations[1]    │  │       - Track features Cam 0 -> Cam 1     │
    │                                           │  │       - Add matches to observations[1]    │
    │ 4. Stereo Epipolar Filtering              │  │                                           │
    │    FrameToFrameOpticalFlow::filterPoints()│  │ 4. Stereo Epipolar Filtering              │
    │    ├─ CamIntrinsics::unproject()          │  │    FrameToFrameOpticalFlow::filterPoints()│
    │    │  - Back-project 2D to 3D unit rays   │  │    ├─ CamIntrinsics::unproject()          │
    │    └─ Epipolar Constraint Check           │  │    │  - Back-project 2D to 3D unit rays   │
    │       - Check |p0^T * E * p1| <= thresh   │  │    └─ Epipolar Constraint Check           │
    │       - Discard stereo outliers in Cam 1  │  │       - Check |p0^T * E * p1| <= thresh   │
    │                                           │  │       - Discard stereo outliers in Cam 1  │
    │ 5. Output Packaging                       │  │                                           │
    │    - Increment frame_counter              │  │ 5. Output Packaging                       │
    │    - Assemble OpticalFlowResult:          │  │    - Increment frame_counter              │
    │      * t_ns: frame timestamp              │  │    - Assemble OpticalFlowResult:          │
    │      * observations: keypoint poses       │  │      * Updated observations (Cam 0 & 1)   │
    │      * input_images: raw image buffer     │  │      * Forward-backward valid tracks      │
    │    - Return transforms to backend queue   │  │    - Return transforms to backend queue   │
    └───────────────────────────────────────────┘  └───────────────────────────────────────────┘
                          │                                              │
                          └──────────────────────┬───────────────────────┘
                                                 │
                                                 ▼
                                       OpticalFlowResult::Ptr
                                      (Pushed to vision queue)
```

#### 2.1.1 Feature Detection

As depicted in the flowchart above (Step 3 for both $t = 0$ and $t > 0$), new features are detected when the tracking frontend determines that the current set of active tracks is insufficient to cover the image. Detection is governed by a spatial occupancy grid of cell size `optical_flow_detection_grid_size` (default 50 px). A corner detector (FAST-like, with sub-pixel refinement) is applied independently to each empty cell, producing at most one keypoint per cell. This grid-based strategy ensures an approximately uniform spatial distribution of features across the image, which is critical for the well-conditioning of the BA problem: features clustered in a small image region produce a near-singular Jacobian block structure, degrading both numerical precision and geometric observability.

The detection pipeline is coordinated by `FrameToFrameOpticalFlow::addPoints`, which gathers existing active track locations in Camera 0 (`pts0`) and delegates extraction to `detectKeypoints` (`include/basalt/utils/keypoints.h`, `src/utils/keypoints.cpp`). Detected corners are assigned monotonically increasing `KeypointId` values that persist for the lifetime of the track, providing a global association key between the frontend and backend. For stereo rigs (`calib.intrinsics.size() > 1`), newly detected keypoints in Camera 0 are immediately tracked to Camera 1 using `FrameToFrameOpticalFlow::trackPoints`, after which stereo matches are validated against the epipolar geometry via `FrameToFrameOpticalFlow::filterPoints` (Step 4).

#### 2.1.2 Feature Tracking

As shown in Step 2 of the subsequent frame pipeline ($t > 0$) in the flowchart above, tracking is the core operation of the frontend executed on each incoming image before new features are detected. Given a patch $\mathcal{P}$ of pixel intensities centred on a keypoint in a reference image $I_\text{ref}$, the tracker seeks the 2D displacement $\mathbf{d} = [d_x, d_y]^T$ that minimises the photometric error in the current image $I_\text{cur}$:

$$
\min_{\mathbf{d}} \sum_{(u,v) \in \mathcal{P}} \left[ I_\text{cur}(u + d_x,\, v + d_y) - I_\text{ref}(u, v) \right]^2.
$$

This is a non-linear least squares problem solved by Gauss–Newton iteration over the image gradient. In `FrameToFrameOpticalFlow::trackPoints`, tracking is parallelised across keypoints using `tbb::parallel_for`. For each keypoint, `FrameToFrameOpticalFlow::trackPoint` conducts a coarse-to-fine search over an image pyramid of `optical_flow_levels` levels (typically 3–4) constructed by `ManagedImagePyr::setFromImage`. At each pyramid level, `FrameToFrameOpticalFlow::trackPointAtLevel` solves for the optimal $SE(2)$ transformation using `PatchT::residual`. To reject tracking drift and false positives, bidirectional verification is performed: the keypoint is tracked forward from `old_pyramid` to `pyramid` and then tracked backward from `pyramid` to `old_pyramid`. If the squared distance between the original and recovered keypoint position exceeds `optical_flow_max_recovered_dist2`, the track is discarded.

Basalt provides three tracking strategies, selectable via `VioConfig::optical_flow_type`:
- **`PatchOpticalFlow`** (`include/basalt/optical_flow/patch_optical_flow.h`) — tracks against the *reference keyframe* where the keypoint was first detected. The stored patch never changes, preventing template drift but shortening track lifetime under viewpoint change.
- **`FrameToFrameOpticalFlow`** (`include/basalt/optical_flow/frame_to_frame_optical_flow.h`) — tracks against the *previous frame*. The template is updated at every step, enabling longer tracks at the cost of gradual appearance drift.
- **`MultiscaleFrameToFrameOpticalFlow`** (`include/basalt/optical_flow/multiscale_frame_to_frame_optical_flow.h`) — extends the frame-to-frame strategy by detecting and tracking features at multiple pyramid levels, improving robustness to large motions and defocused regions.

For stereo configurations, newly detected keypoints in camera 0 are immediately tracked to camera 1 (Step 3). The resulting stereo matches are validated by an epipolar check in `FrameToFrameOpticalFlow::filterPoints` (Step 4): 2D points are unprojected to calibrated 3D bearing rays via `CamIntrinsics::unproject`, and the algebraic epipolar error $|\mathbf{p}_0^T \mathbf{E} \mathbf{p}_1|$ with respect to the Essential matrix $\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}$ derived from the known camera-to-camera extrinsic must be below `optical_flow_epipolar_error`. Matches violating this constraint are discarded.

#### 2.1.3 Landmark Selection

Not every tracked keypoint becomes a landmark in the BA back end. A keypoint is promoted to a 3D landmark only when two conditions are met: (i) a keyframe decision triggers triangulation (see §7), and (ii) sufficient parallax (baseline) exists between the host keyframe and at least one other observation to permit a well-conditioned depth estimate. Specifically, the squared translation between the host and observing cameras must exceed `vio_min_triangulation_dist`$^2$, and the inverse-depth must satisfy $0 < \rho < 3.0$ (see §7 for the full triangulation procedure).

The resulting 3D landmark is parameterised in the host camera frame using a minimal **stereographic direction + inverse-distance** representation (3 DOF per landmark), as detailed in §2.2.3. This representation avoids the gauge freedom of homogeneous 4-vectors and is numerically well-behaved for points at finite depth.

#### 2.1.4 Frontend Output

The frontend packages each processed frame into an `OpticalFlowResult` (`include/basalt/optical_flow/optical_flow.h`), containing:
- `t_ns` — the frame timestamp in nanoseconds;
- `observations` — a per-camera vector of maps `KeypointId → AffineCompact2f`, where the affine transform encodes the 2D position (translation component) and optional patch deformation;
- `input_images` — a pointer to the raw images for downstream visualisation.

This result is pushed to the estimator's `vision_data_queue` (`tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr>`), from which the backend consumes it for BA.

### 2.2 VO Backend

This section derives the bundle adjustment problem from projective geometry first principles, defines the reprojection residual and its Jacobians, and formulates the resulting non-linear least squares optimisation. The treatment proceeds incrementally: we first define the coordinate frames and transformations, then the projection model, then the residual, and finally the normal equations. An ASCII diagram anchors the geometric setup.

#### 2.2.1 Geometric Setup

The bundle adjustment problem involves three types of entities: camera poses, 3D landmarks, and 2D observations. A landmark hosted in one keyframe (the *host*, observed at $t_h$) is generally also observed in other keyframes (each a *target*, observed at $t_t$) as the camera moves through the scene. The geometric relationship between the host frame, the target frame, and a shared 3D landmark $X$ is depicted below:

```
                                                     3D landmark, world frame
                                                                   X
                                                ray C_h-X         / \        ray C_t-X
                                                                 /   \
                                                                /     \
                                                               /       \
                                                              /         \
                                                             /           \
                                                            /             \
                                                           /             ┌─\── Target Frame  (t = t_t) ────┐
                                                          /              │  \                              │
                                                         /               │   \                             │
                                                        /                │    \ x_t                        │
                                                       /                 │     \                           │
                                                      /                  │      \                          │
                                                     /                   │       \                         │
                                                    /                    └────────\────────────────────────┘
                                                   /                               \
                                                  /                                 \
                                                 / baseline  b = t_t,h               \
                                                /                                     \
                                               /                                       \
                   ┌───── Host Frame  (t = t_h/ ─────┐                                  \
                   │                         /       │                                   \
                   │                        /        │                                    \
                   │                       / x_h     │                                     C_t
                   │                      /          │                            (camera centre,
                   │                     /           │                                target)
                   │                    /            │
                   └───────────────────/─────────────┘
                                      /
                                     /
                                    /
                                   /
                                  /
                                 /
                                /
                               C_h
                      (camera centre,
                            host)
```

The ray from each camera centre to $X$ pierces that camera's frame at the observed 2D point, $x_h$ in the host frame and $x_t$ in the target frame. The two frames are related by the relative pose $\mathbf{T}_{t,h}$, whose translation component is the baseline $b$ shown above; this relative pose is what the reprojection residual below is built from.

**Notation.** Let $\mathbf{T}_{w,i} \in SE(3)$ denote the pose of the IMU/body frame relative to the world frame, decomposed as $\mathbf{T}_{w,i} = (\mathbf{R}_{w,i},\, \mathbf{p}_{w,i})$ with rotation $\mathbf{R}_{w,i} \in SO(3)$ and translation $\mathbf{p}_{w,i} \in \mathbb{R}^3$. The fixed extrinsic calibration between the IMU and camera $c$ is $\mathbf{T}_{i,c} \in SE(3)$. The camera pose in the world frame is therefore $\mathbf{T}_{w,c} = \mathbf{T}_{w,i} \cdot \mathbf{T}_{i,c}$.

**Relative pose.** For a landmark hosted in camera $h$ and observed in camera $t$, the relative transformation mapping points from the host camera frame to the target camera frame is

$$
\mathbf{T}_{t,h} = \mathbf{T}_{i,c_t}^{-1} \cdot \mathbf{T}_{w,i_t}^{-1} \cdot \mathbf{T}_{w,i_h} \cdot \mathbf{T}_{i,c_h}.
$$

This is computed by `computeRelPose` (`include/basalt/utils/ba_utils.h:41`). The function also returns the $6 \times 6$ Jacobians of $\mathbf{T}_{t,h}$ with respect to the host and target body poses, computed via the Adjoint representation of $SE(3)$:

$$
\frac{\partial \mathbf{T}_{t,h}}{\partial \mathbf{T}_{w,i_h}} = \text{Adj}(\mathbf{T}_{i,c_t}^{-1} \cdot \mathbf{T}_{w,i_t}^{-1} \cdot \mathbf{T}_{w,i_h}), \qquad
\frac{\partial \mathbf{T}_{t,h}}{\partial \mathbf{T}_{w,i_t}} = -\text{Adj}(\mathbf{T}_{i,c_t}^{-1}).
$$

#### 2.2.2 Camera Projection Model

A camera projects a 3D point $\mathbf{p} = [x, y, z]^T$ in its local frame to a 2D image coordinate $\mathbf{z} \in \mathbb{R}^2$ via a projection function $\pi(\cdot)$:

$$
\mathbf{z} = \pi(\mathbf{p}).
$$

Basalt supports multiple camera models through a compile-time variant (`Calibration<Scalar>::intrinsics`), including the pinhole, equidistant (Kannala–Brandt), and double-sphere (Usenko et al., 2018) models. The projection function and its Jacobian $\mathbf{J}_\pi = \partial \pi / \partial \mathbf{p} \in \mathbb{R}^{2 \times 3}$ are provided by each model's `project` method. The inverse operation — mapping a 2D point to a unit bearing vector — is provided by `unproject` and is used during triangulation and landmark initialisation.

#### 2.2.3 Landmark Parameterisation

Each landmark is stored relative to the camera frame of its *host keyframe*. As introduced in §7.1 of the Triangulation section, the landmark is parameterised by a **bearing direction** $\mathbf{d} \in \mathbb{R}^3 \setminus \{\mathbf{0}\}$ and an **inverse distance** $\rho \in \mathbb{R}_{>0}$ from the host camera. Given a true 3D point $\mathbf{p}_h = [X,Y,Z]^T \in \mathbb{R}^3$ in the host camera frame, the direction/distance decomposition underlying the parameterisation is

$$
\mathbf{d} = \frac{\mathbf{p}_h}{|\mathbf{p}_h|}, \qquad \rho = \frac{1}{|\mathbf{p}_h|},
$$

To give the direction a minimal 2-parameter representation, Basalt uses **stereographic projection** (Civera et al., 2008):

$$
\theta = \pi_\text{st}(\mathbf{d}) = \left[\frac{d_x}{(|\mathbf{d}| + d_z)},\; \frac{d_y}{(|\mathbf{d}| + d_z)}\right]^T \in \mathbb{R}^2,
$$

with inverse

$$
\pi_\text{st}^{-1}(\theta) = \frac{2}{(1 + \|\theta\|^2)} \begin{bmatrix} \theta_x \\ \theta_y \\ 1 \end{bmatrix} - \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}.
$$

The $|\mathbf{d}|$ appearing explicitly in $\pi_\text{st}$'s denominator, rather than an assumed unit norm, is what makes the map scale-invariant, $\pi_\text{st}(\lambda\mathbf{d}) = \pi_\text{st}(\mathbf{d})$ for every $\lambda > 0$: scaling $\mathbf{d}$ scales the numerator and $|\mathbf{d}|+d_z$ identically, so the ratio is unchanged. The inverse map $\pi_\text{st}^{-1}$, by contrast, always returns an exact unit vector.

The landmark parameter vector is thus $\mathbf{l} = [\theta^T,\, \rho]^T \in \mathbb{R}^3$, stored in `Keypoint<Scalar>` (`include/basalt/vi_estimator/landmark_database.h:53`). The 3D point in the host camera frame is recovered as $\mathbf{p}_h = \pi_\text{st}^{-1}(\theta) / \rho$. The stereographic projection is implemented in `StereographicParam<Scalar>` (`thirdparty/basalt-headers/include/basalt/camera/stereographic_param.hpp`), which also optionally provides the Jacobians $\partial \pi_\text{st} / \partial \mathbf{d}$ and $\partial \pi_\text{st}^{-1} / \partial \theta$.

#### 2.2.4 The Reprojection Residual

The reprojection residual measures the discrepancy between a predicted and an observed 2D feature location. For a landmark $j$ with parameters $\mathbf{l}_j = [\theta_j^T, \rho_j]^T$ hosted in keyframe $h(j)$ and observed in target frame $i$ at measured pixel location $\mathbf{z}_{ij}$, the residual is:

**Step 1: Unproject to 3D.** Recover the 3D point in the host camera frame:

$$
\mathbf{p}_h = \frac{1}{\rho_j} \pi_\text{st}^{-1}(\theta_j).
$$

**Step 2: Transform to target frame.** Apply the relative pose:

$$
\mathbf{p}_t = \mathbf{T}_{t,h} \cdot \mathbf{p}_h.
$$

**Step 3: Project and compute residual.** Project into the target camera and subtract the observation:

$$
\mathbf{r}_{ij} = \pi(\mathbf{p}_t) - \mathbf{z}_{ij} \in \mathbb{R}^2.
$$

This three-step chain is implemented by `linearizePoint` (`include/basalt/utils/ba_utils.h:82`), which additionally computes the Jacobians by the chain rule. The remainder of this subsection explicitly formulates $\mathbf{r}_{ij}$ as a function of a *local update* of the pose and landmark variables.

The aforementioned Steps 1–2 treat $\mathbf{p}_h$ as an ordinary point of $\mathbb{R}^3$, obtained by dividing the unprojected direction by $\rho_j$. Division by $\rho_j$ is undefined as $\rho_j \to 0$, precisely the regime of a landmark far enough from the host camera to be effectively at infinity, a configuration VIO frontends encounter routinely (distant structure, sky, horizon features). Basalt sidesteps the division entirely by carrying $\mathbf{p}_h$ in unnormalised homogeneous coordinates (Hartley and Zisserman, 2004, §1.1), scaled by $\rho_j$ instead of by the conventional weight 1:

$$
\tilde{\mathbf{p}}_h = \begin{bmatrix} \pi_\text{st}^{-1}(\theta_j) \\[2pt] \rho_j \end{bmatrix} \in \mathbb{R}^4, \qquad \mathbf{p}_h = \frac{\tilde{\mathbf{p}}_h[1{:}3]}{\tilde{\mathbf{p}}_h[4]}.
$$

Applying the $SE(3)$ transform to this representative in its $4\times 4$ homogeneous matrix form, $\mathbf{T}_{t,h} = \begin{bmatrix} \mathbf{R}_{t,h} & \mathbf{t}_{t,h} \\ \mathbf{0}^T & 1 \end{bmatrix}$, gives $\tilde{\mathbf{p}}_t = \mathbf{T}_{t,h}\, \tilde{\mathbf{p}}_h = [\mathbf{R}_{t,h}\,\pi_\text{st}^{-1}(\theta_j) + \rho_j \mathbf{t}_{t,h};\ \rho_j]$, which is exactly $\rho_j\, \mathbf{p}_t$ in homogeneous form, so $\tilde{\mathbf{p}}_t[1{:}3]/\tilde{\mathbf{p}}_t[4] = \mathbf{p}_t$ recovers Step 2 unchanged. Every camera projection model in `basalt-headers` (double-sphere, pinhole, Kannala–Brandt, unified) is, by construction, invariant to a positive rescaling of its 3D argument (a consequence of the projection being defined purely by a ray direction), so $\pi(\tilde{\mathbf{p}}_t[1{:}3]) = \pi(\rho_j \mathbf{p}_t) = \pi(\mathbf{p}_t)$ identically. Passing $\tilde{\mathbf{p}}_t[1{:}3]$ directly into $\pi$ therefore reproduces Step 3 exactly, without ever dividing by $\rho_j$, and the same expression remains well defined at $\rho_j = 0$. `linearizePoint` carries out steps 1–3 in exactly this homogeneous form: `p_h_3d = [StereographicParam::unproject(...); kpt_pos.inv_dist]` and `p_t_3d = T_t_h * p_h_3d` are $\tilde{\mathbf{p}}_h$ and $\tilde{\mathbf{p}}_t$ above.

Gauss–Newton iteration (§2.2.7, `doc/Marginalisation.md` §2.1) requires the derivative of $\mathbf{r}_{ij}$ with respect to a local update of $\mathbf{T}_{t,h}$ and $\mathbf{l}_j$ about the current linearisation point $(\bar{\mathbf{T}}_{t,h}, \bar{\mathbf{l}}_j)$, not with respect to these quantities directly: $SE(3)$ is a manifold rather than a vector space, so "$\partial \mathbf{r}/\partial \mathbf{T}_{t,h}$" is not otherwise defined, and $\theta_j$ enters $\tilde{\mathbf{p}}_h$ through the nonlinear stereographic chart. Two local update rules are used, one per variable group.

The relative pose is updated multiplicatively, by composing a minimal $\mathbb{R}^6$ perturbation on the left through the $SE(3)$ exponential map:

$$
\mathbf{T}_{t,h}(\boldsymbol{\xi}) = \exp(\boldsymbol{\xi}) \cdot \bar{\mathbf{T}}_{t,h}, \qquad \boldsymbol{\xi} = [\boldsymbol{\upsilon}^T,\, \boldsymbol{\omega}^T]^T \in \mathbb{R}^6.
$$

The landmark is updated additively, since $\mathbf{l}_j = [\theta_j^T, \rho_j]^T$ already lives in the flat space $\mathbb{R}^3$:

$$
\mathbf{l}_j(\delta\mathbf{l}) = \bar{\mathbf{l}}_j + \delta\mathbf{l}, \qquad \delta\mathbf{l} = [\delta\theta^T,\, \delta\rho]^T \in \mathbb{R}^3.
$$

Substituting both updates into the homogeneous chain of Steps 1–3 gives the residual as an explicit function of $(\boldsymbol{\xi}, \delta\mathbf{l})$,

$$
\mathbf{r}_{ij}(\boldsymbol{\xi}, \delta\mathbf{l}) = \pi\Big(\big[\exp(\boldsymbol{\xi}) \cdot \bar{\mathbf{T}}_{t,h} \cdot \tilde{\mathbf{p}}_h(\bar{\mathbf{l}}_j + \delta\mathbf{l})\big][1{:}3]\Big) - \mathbf{z}_{ij},
$$

and §2.2.5 differentiates exactly this expression at $(\boldsymbol{\xi}, \delta\mathbf{l}) = (\mathbf{0}, \mathbf{0})$, first with respect to $\boldsymbol{\xi}$ holding $\delta\mathbf{l} = \mathbf{0}$, then with respect to $\delta\mathbf{l}$ holding $\boldsymbol{\xi} = \mathbf{0}$, and combines the two partial results into the total first-order (Gauss–Newton) Jacobian, following the general product/chain-rule pattern of `doc/Marginalisation.md` §2.1.

The Lie algebra $\mathfrak{se}(3)$ consists of $4\times 4$ matrices $\hat{\boldsymbol{\xi}} = \begin{bmatrix} [\boldsymbol{\omega}]_\times & \boldsymbol{\upsilon} \\ \mathbf{0}^T & 0 \end{bmatrix}$ for $\boldsymbol{\xi} = [\boldsymbol{\upsilon}^T,\boldsymbol{\omega}^T]^T \in \mathbb{R}^6$, where $[\boldsymbol{\omega}]_\times$ is the skew-symmetric matrix satisfying $[\boldsymbol{\omega}]_\times \mathbf{v} = \boldsymbol{\omega} \times \mathbf{v}$ for all $\mathbf{v} \in \mathbb{R}^3$. The exponential map $\exp: \mathfrak{se}(3) \to SE(3)$ recovers a rigid transform from $\boldsymbol{\xi}$ and, near $\boldsymbol{\xi} = \mathbf{0}$, admits the first-order (matrix) expansion $\exp(\boldsymbol{\xi}) = \mathbf{I}_4 + \hat{\boldsymbol{\xi}} + O(\|\boldsymbol{\xi}\|^2)$. For any homogeneous 4-vector $\tilde{\mathbf{p}} = [\mathbf{X}^T,\, w]^T$ (an ordinary point has $w=1$; a landmark's host representative has $w = \rho_j$, per above), left action by $\exp(\boldsymbol{\xi})$ therefore satisfies, to first order,

$$
\exp(\boldsymbol{\xi}) \cdot \tilde{\mathbf{p}} = (\mathbf{I}_4 + \hat{\boldsymbol{\xi}})\, \tilde{\mathbf{p}} + O(\|\boldsymbol{\xi}\|^2) = \begin{bmatrix} \mathbf{X} + \boldsymbol{\omega} \times \mathbf{X} + w\, \boldsymbol{\upsilon} \\ w \end{bmatrix} + O(\|\boldsymbol{\xi}\|^2).
$$

This single identity, evaluated at $\tilde{\mathbf{p}} = \bar{\mathbf{T}}_{t,h}\, \tilde{\mathbf{p}}_h = \tilde{\mathbf{p}}_t$, is the exact tool §2.2.5 uses to differentiate the pose part of the residual; the same Adjoint mechanics underlie the $\partial \mathbf{T}_{t,h}/\partial \mathbf{T}_{w,i_h}$, $\partial \mathbf{T}_{t,h}/\partial \mathbf{T}_{w,i_t}$ formulas already stated in §2.2.1.

#### 2.2.5 Jacobians of the Reprojection Residual

The residual $\mathbf{r}_{ij}(\boldsymbol{\xi}, \delta\mathbf{l})$ derived in §2.2.4 depends on three groups of variables: the relative-pose perturbation $\boldsymbol{\xi}$ (which itself factors, via `computeRelPose`, into the host pose $\mathbf{T}_{w,i_h}$ and the target pose $\mathbf{T}_{w,i_t}$) and the landmark perturbation $\delta\mathbf{l}$. Differentiating $\mathbf{r}_{ij}$ requires differentiating each stage of its definition in turn and combining the results by the chain rule; this subsection performs that differentiation explicitly, one variable group at a time, holding the others fixed at $\mathbf{0}$.

Differentiating with respect to the relative-pose perturbation $\boldsymbol{\xi}$, with $\delta\mathbf{l} = \mathbf{0}$, only the pose term of $\mathbf{r}_{ij}(\boldsymbol{\xi}, \mathbf{0})$ varies with $\boldsymbol{\xi}$ as follows:

$$
\tilde{\mathbf{p}}_t(\boldsymbol{\xi}) = \exp(\boldsymbol{\xi}) \cdot \bar{\mathbf{T}}_{t,h} \cdot \tilde{\mathbf{p}}_h = \exp(\boldsymbol{\xi}) \cdot \bar{\tilde{\mathbf{p}}}_t.
$$

Applying the exponential-map identity of §2.2.4 at $\tilde{\mathbf{p}} = \bar{\tilde{\mathbf{p}}}_t = [\mathbf{p}_t^T,\, \rho_j]^T$ (so $\mathbf{X} = \mathbf{p}_t$, $w = \rho_j$) and differentiating termwise with respect to $\boldsymbol{\upsilon}$ and $\boldsymbol{\omega}$ gives

$$
\frac{\partial \tilde{\mathbf{p}}_t}{\partial \boldsymbol{\upsilon}}\bigg|_{\boldsymbol{\xi}=\mathbf{0}} = \begin{bmatrix} \rho_j \mathbf{I}_3 \\ \mathbf{0}^T \end{bmatrix}, \qquad
\frac{\partial \tilde{\mathbf{p}}_t}{\partial \boldsymbol{\omega}}\bigg|_{\boldsymbol{\xi}=\mathbf{0}} = \begin{bmatrix} -[\mathbf{p}_{t}]_\times \\ \mathbf{0}^T \end{bmatrix},
$$

using $\partial(\boldsymbol{\omega} \times \mathbf{p}_t)/\partial \boldsymbol{\omega} = -[\mathbf{p}_t]_\times$ (the standard identity for the derivative of a cross product with respect to its left argument). Stacking the two blocks gives the full $4\times 6$ Jacobian $\partial \tilde{\mathbf{p}}_t / \partial \boldsymbol{\xi}$. Every camera projection model reads only the first three components of its homogeneous argument (§2.2.4), so $\pi$'s Jacobian $\mathbf{J}_\pi = \partial \pi/\partial \mathbf{p}_t \in \mathbb{R}^{2\times 3}$, already introduced in §2.2.2, contracts only against the top three rows of $\partial \tilde{\mathbf{p}}_t/\partial \boldsymbol{\xi}$; the bottom row (the derivative of the homogeneous weight $w$, which is identically zero here) never contributes. Writing $\partial \mathbf{p}_t/\partial\boldsymbol{\xi} := (\partial \tilde{\mathbf{p}}_t/\partial\boldsymbol{\xi})[1{:}3,:]$ for this top block,

$$
\frac{\partial \mathbf{p}_t}{\partial \boldsymbol{\xi}} = \begin{bmatrix} \rho_j \mathbf{I}_3 & -[\mathbf{p}_{t}]_\times \end{bmatrix} \in \mathbb{R}^{3 \times 6},
$$

and, by the chain rule through $\mathbf{r}_{ij} = \pi(\mathbf{p}_t) - \mathbf{z}_{ij}$,

$$
\frac{\partial \mathbf{r}_{ij}}{\partial \boldsymbol{\xi}} = \mathbf{J}_\pi \cdot \frac{\partial \mathbf{p}_t}{\partial \boldsymbol{\xi}} \in \mathbb{R}^{2 \times 6}.
$$

The perturbation $\boldsymbol{\xi}$ is itself induced by perturbing the host and target body poses inside `computeRelPose`; the chain rule through that composition gives

$$
\frac{\partial \mathbf{r}_{ij}}{\partial \mathbf{T}_{w,i_h}} = \frac{\partial \mathbf{r}_{ij}}{\partial \boldsymbol{\xi}} \cdot \frac{\partial \mathbf{T}_{t,h}}{\partial \mathbf{T}_{w,i_h}}, \qquad
\frac{\partial \mathbf{r}_{ij}}{\partial \mathbf{T}_{w,i_t}} = \frac{\partial \mathbf{r}_{ij}}{\partial \boldsymbol{\xi}} \cdot \frac{\partial \mathbf{T}_{t,h}}{\partial \mathbf{T}_{w,i_t}},
$$

with $\partial \mathbf{T}_{t,h}/\partial \mathbf{T}_{w,i_h}$ and $\partial \mathbf{T}_{t,h}/\partial \mathbf{T}_{w,i_t}$ the Adjoint-representation Jacobians already derived in §2.2.1 from the same first-order exponential-map identity.

Differentiating with respect to the landmark perturbation $\delta\mathbf{l}$, with $\boldsymbol{\xi} = \mathbf{0}$, the pose $\bar{\mathbf{T}}_{t,h}$ is fixed and only $\tilde{\mathbf{p}}_h$ varies with $\delta\mathbf{l} = [\delta\theta^T, \delta\rho]^T$. Let $\mathbf{J}_\text{up} = \partial \pi_\text{st}^{-1}/\partial \theta \in \mathbb{R}^{4\times 2}$ denote the stereographic unproject Jacobian (`StereographicParam::unproject`, §2.2.3); by construction its fourth row is identically zero, since the raw unprojected 4-vector's last component is a placeholder $0$ before being overwritten by $\rho_j$ and so carries no $\theta$-dependence. Differentiating $\tilde{\mathbf{p}}_h(\delta\mathbf{l}) = [\pi_\text{st}^{-1}(\bar\theta + \delta\theta);\ \bar\rho + \delta\rho]$ termwise gives the two Jacobian blocks

$$
\frac{\partial \tilde{\mathbf{p}}_h}{\partial \theta} = \mathbf{J}_\text{up}, \qquad
\frac{\partial \tilde{\mathbf{p}}_h}{\partial \rho} = \mathbf{e}_4 := [0,0,0,1]^T,
$$

the first a genuine first-order Taylor approximation (the stereographic chart is nonlinear in $\theta$), the second exact ($\tilde{\mathbf{p}}_h$ is affine, in fact linear, in $\rho$). Stacking, $\partial \tilde{\mathbf{p}}_h/\partial \mathbf{l}_j = [\mathbf{J}_\text{up}\ \ \mathbf{e}_4] \in \mathbb{R}^{4\times 3}$. Because $\tilde{\mathbf{p}}_t = \bar{\mathbf{T}}_{t,h}\, \tilde{\mathbf{p}}_h$ is *linear* in $\tilde{\mathbf{p}}_h$ at fixed pose, the chain rule needs no further linearisation here, it applies $\bar{\mathbf{T}}_{t,h}$ directly:

$$
\frac{\partial \tilde{\mathbf{p}}_t}{\partial \mathbf{l}_j} = \bar{\mathbf{T}}_{t,h} \cdot \begin{bmatrix} \mathbf{J}_\text{up} & \mathbf{e}_4 \end{bmatrix} \in \mathbb{R}^{4 \times 3}.
$$

Unlike the pose case above, the bottom (homogeneous-weight) row of this Jacobian is *not* zero, its $\delta\rho$ column equals $1$, since $\bar{\mathbf{T}}_{t,h}\, \mathbf{e}_4$ is exactly the fourth column of $\bar{\mathbf{T}}_{t,h}$, namely $[\bar{\mathbf{t}}_{t,h}^T,\, 1]^T$. This is immaterial to the final residual Jacobian only because $\mathbf{J}_\pi$ structurally ignores that row (§2.2.4); the weight row must nonetheless be retained through this step, since dropping it before forming $\bar{\mathbf{T}}_{t,h}\cdot[\mathbf{J}_\text{up}\ \ \mathbf{e}_4]$ would also silently drop its contribution to the *top* three rows via $\bar{\mathbf{t}}_{t,h}$. Restricting to those top three rows, using $\bar{\mathbf{T}}_{t,h} = [\bar{\mathbf{R}}_{t,h} \mid \bar{\mathbf{t}}_{t,h}]$ and $\mathbf{J}_\text{up}$'s zero fourth row,

$$
\frac{\partial \mathbf{p}_t}{\partial \mathbf{l}_j} = \begin{bmatrix} \bar{\mathbf{R}}_{t,h}\, \mathbf{J}_\text{up}[1{:}3,:] & \bar{\mathbf{t}}_{t,h} \end{bmatrix} \in \mathbb{R}^{3 \times 3},
$$

and, by the chain rule,

$$
\frac{\partial \mathbf{r}_{ij}}{\partial \mathbf{l}_j} = \mathbf{J}_\pi \cdot \frac{\partial \mathbf{p}_t}{\partial \mathbf{l}_j} \in \mathbb{R}^{2 \times 3}.
$$

(Correction, 2026-08-28: an earlier draft of this Jacobian read $\mathbf{T}_{t,h}\cdot[\mathbf{J}_\text{up}\ \ \mathbf{T}_{t,h}^{-1}\mathbf{e}_4]$. The $\mathbf{T}_{t,h}^{-1}$ factor was an error, introduced by mis-stating which side of the product the fourth-column extraction belongs on; differentiating `linearizePoint`'s actual computation, `Jpp.col(2) = T_t_h.col(3)`, shows the $\delta\rho$ column is $\mathbf{T}_{t,h}\,\mathbf{e}_4$ with no inverse involved, as derived above.)

#### 2.2.6 The Bundle Adjustment Objective

Given a set of camera poses $\{\mathbf{T}_{w,i_k}\}_{k=1}^{K}$ and landmarks $\{\mathbf{l}_j\}_{j=1}^{L}$, with observations $\{(i, j, \mathbf{z}_{ij})\} \in \mathcal{V}$, the BA problem seeks to minimise:

$$
E_\text{BA}(\mathbf{s}) = \frac{1}{2} \sum_{(i,j) \in \mathcal{V}} \mathbf{r}_{ij}(\mathbf{s})^T \boldsymbol{\Sigma}_\text{vis}^{-1} \mathbf{r}_{ij}(\mathbf{s}),
$$

where $\mathbf{s} = [\mathbf{T}_1, \ldots, \mathbf{T}_K, \mathbf{l}_1, \ldots, \mathbf{l}_L]$ is the full state vector and $\boldsymbol{\Sigma}_\text{vis} = \sigma_\text{obs}^2 \mathbf{I}_2$ is the isotropic observation covariance (configured by `vio_obs_std_dev`).

To limit the influence of outlier observations, each residual is down-weighted by a Huber kernel (Huber, 1964):

$$
w_H(\mathbf{r}) = \begin{cases}
1 & \text{if } \|\mathbf{r}\| \leq \delta_H, \\
\delta_H / \|\mathbf{r}\| & \text{if } \|\mathbf{r}\| > \delta_H,
\end{cases}
$$

where $\delta_H$ = `vio_obs_huber_thresh`. The weighted cost becomes $\frac{1}{2} w_H \|\mathbf{r}\|^2 / \sigma_\text{obs}^2$.

To enable efficient online operation, the VO estimator does not optimise the entire trajectory. Instead, it maintains a sliding window of at most `vio_max_kfs` keyframes and `vio_max_states` recent frame poses. The total VO objective includes the visual cost above plus a marginalisation prior $E_\text{marg}(\mathbf{s})$ that summarises all information from evicted states (see `doc/Marginalisation.md`):

$$
E_\text{VO}(\mathbf{s}) = E_\text{BA}(\mathbf{s}) + E_\text{marg}(\mathbf{s}).
$$

#### 2.2.7 Gauss–Newton Linearisation and the Normal Equations

The non-linear cost is iteratively minimised by linearising each residual around the current estimate $\mathbf{s}$ using a first-order Taylor expansion on the manifold (see `doc/Marginalisation.md` §2 for the complete derivation of the Gauss–Newton approximation, the information matrix $\mathbf{H}$, and the information vector $\mathbf{b}$):

$$
\mathbf{r}_{ij}(\mathbf{s} \oplus \boldsymbol{\xi}) \approx \mathbf{r}_{ij}(\mathbf{s}) + \mathbf{J}_{ij} \boldsymbol{\xi},
$$

where $\boldsymbol{\xi}$ is the stacked state increment and $\mathbf{J}_{ij}$ is the Jacobian row block for observation $(i,j)$. Substituting into the cost and differentiating yields the normal equations $\mathbf{H} \boldsymbol{\xi} = \mathbf{b}$, where

$$
\mathbf{H} = \sum_{(i,j)} \mathbf{J}_{ij}^T \mathbf{W}_{ij} \mathbf{J}_{ij}, \qquad \mathbf{b} = -\sum_{(i,j)} \mathbf{J}_{ij}^T \mathbf{W}_{ij} \mathbf{r}_{ij},
$$

with $\mathbf{W}_{ij} = w_H / \sigma_\text{obs}^2 \cdot \mathbf{I}_2$.

#### 2.2.8 The Arrowhead Structure and Landmark Elimination

The key computational insight of BA is the **arrowhead sparsity** of $\mathbf{H}$. Each landmark $\mathbf{l}_j$ appears only in the observations that see it, so its off-diagonal Hessian blocks couple only the (few) frames observing it. Ordering $\boldsymbol{\xi} = [\boldsymbol{\xi}_\text{poses}^T,\, \boldsymbol{\xi}_\text{lms}^T]^T$, the Hessian takes the block form:

$$
\begin{bmatrix} \mathbf{H}_{pp} & \mathbf{H}_{pl} \\ \mathbf{H}_{lp} & \mathbf{H}_{ll} \end{bmatrix} \begin{bmatrix} \boldsymbol{\xi}_p \\ \boldsymbol{\xi}_l \end{bmatrix} = \begin{bmatrix} \mathbf{b}_p \\ \mathbf{b}_l \end{bmatrix},
$$

where $\mathbf{H}_{ll}$ is **block-diagonal** (each $3 \times 3$ block belongs to a single landmark). This permits analytic elimination of the landmarks via the Schur complement:

$$
\underbrace{(\mathbf{H}_{pp} - \mathbf{H}_{pl}\, \mathbf{H}_{ll}^{-1}\, \mathbf{H}_{lp})}_{\mathbf{H}^*_p} \boldsymbol{\xi}_p = \underbrace{\mathbf{b}_p - \mathbf{H}_{pl}\, \mathbf{H}_{ll}^{-1}\, \mathbf{b}_l}_{\mathbf{b}^*_p}.
$$

Because $\mathbf{H}_{ll}$ is block-diagonal, each $3 \times 3$ inversion is $O(1)$, and the total elimination cost is $O(L \cdot K^2)$ where $L$ is the number of landmarks and $K$ the number of poses. The reduced system $\mathbf{H}^*_p$ is dense but small ($6K \times 6K$ for VO), making it tractable for direct factorisation.

**Square-root elimination (ABS_QR).** Basalt's default linearisation strategy (`vio_linearization_type = ABS_QR`) avoids forming $\mathbf{H}_{ll} = \mathbf{J}_l^T \mathbf{J}_l$ entirely. Instead, it applies a Householder QR decomposition to the landmark columns of each landmark's stacked Jacobian (Demmel et al., 2021):

$$
\mathbf{Q}^T \begin{bmatrix} \mathbf{J}_l & \mathbf{J}_p & \mathbf{r} \end{bmatrix} = \begin{bmatrix} \mathbf{R}_l & \mathbf{Q}_1^T \mathbf{J}_p & \mathbf{Q}_1^T \mathbf{r} \\ \mathbf{0} & \mathbf{Q}_2^T \mathbf{J}_p & \mathbf{Q}_2^T \mathbf{r} \end{bmatrix}.
$$

The rows $[\mathbf{Q}_2^T \mathbf{J}_p \mid \mathbf{Q}_2^T \mathbf{r}]$ contribute directly to the reduced camera system without ever forming $\mathbf{J}_l^T \mathbf{J}_l$, effectively doubling the numerical precision. This is performed per-landmark by `LandmarkBlock::performQR` (`include/basalt/linearization/landmark_block.hpp`).

The equation above is stated here without proof. `doc/Linearisation.md` derives it in full, establishing in its §2.11 that nullspace projection and the Schur complement give algebraically identical reduced systems, and developing the storage layout, the Householder sweep and the back-substitution that realise it in §3.1.5 through §3.1.10. That document is also the reference for how the marginalisation prior enters the linearised system, in its §3.2.3, and for the `LinearizationBase` API summarised in §9.5 below.

#### 2.2.9 Levenberg–Marquardt Iteration

The Gauss–Newton system is regularised with Levenberg–Marquardt (LM) damping:

$$
(\mathbf{H}^*_p + \lambda \cdot \text{diag}(\mathbf{H}^*_p))\, \boldsymbol{\xi}_p = \mathbf{b}^*_p.
$$

The damped system is solved by LDLT factorisation. The damping parameter $\lambda$ is adjusted by the Nielsen rule: if the step reduces the actual cost and the ratio of actual to predicted reduction is positive, $\lambda$ is decreased; otherwise it is increased geometrically by `lambda_vee` (initially 2). Convergence is declared when $|f_\text{diff}| < 10^{-6}$ or $\|\boldsymbol{\xi}\|_\infty < 10^{-4}$. Failure is declared if $\lambda$ exceeds `vio_lm_lambda_max`.

After solving for $\boldsymbol{\xi}_p$, landmarks are recovered by **back-substitution**:

$$
\boldsymbol{\xi}_{l_j} = \mathbf{R}_{l_j}^{-1} (\mathbf{Q}_1^T \mathbf{r}_j - \mathbf{Q}_1^T \mathbf{J}_{p_j}\, \boldsymbol{\xi}_p),
$$

which is performed by `LandmarkBlock::backSubstitute`.

Pose updates are applied on the manifold: $\mathbf{p} \leftarrow \mathbf{p} + \boldsymbol{\upsilon}$ and $\mathbf{R} \leftarrow \text{Exp}(\boldsymbol{\omega}) \cdot \mathbf{R}$ (left-multiplication), via `PoseStateWithLin::applyInc` (`include/basalt/utils/imu_types.h`). Landmark updates act on the 3-vector $[\theta^T, \rho]^T$ in Euclidean space.

### 2.3 Implementation Overview

This section describes the code-level implementation of Visual Odometry as realised by `SqrtKeypointVoEstimator<Scalar>` (`include/basalt/vi_estimator/sqrt_keypoint_vo.h`, `src/vi_estimator/sqrt_keypoint_vo.cpp`). The implementation parallels the VIO estimator (`SqrtKeypointVioEstimator`) but is structurally simpler: there are no IMU states, no velocity or bias variables, and the state vector at each frame contains only a 6-DoF pose.

#### 2.3.1 Pipeline Overview

The VO processing pipeline is driven by the `measure` method, which is called for each incoming optical flow result. The following ASCII diagram summarises the data flow:

```
                        ┌──────────────────────────┐
                        │   Optical Flow Frontend   │
                        │  (PatchOpticalFlow, etc.) │
                        └────────────┬─────────────┘
                                     │ OpticalFlowResult::Ptr
                                     ▼
                        ┌──────────────────────────┐
                        │     ProcessFrame(curr)    │
                        │   sqrt_keypoint_vo.cpp    │
                        │   - Drain IMU queue       │
                        │   - Call measure()        │
                        └────────────┬─────────────┘
                                     │
                                     ▼
                        ┌──────────────────────────┐
                        │        measure()          │
                        ├──────────────────────────┤
                        │ 1. Add pose (constant     │
                        │    position prediction)   │
                        │                           │
                        │ 2. Landmark association:  │
                        │    - For each observation │
                        │      in OpticalFlowResult │
                        │      check lmdb           │
                        │    - Count connected /    │
                        │      unconnected obs      │
                        │                           │
                        │ 3. Single-frame pose-only │
                        │    optimisation           │
                        │    (optimize_single_      │
                        │     frame_pose)           │
                        │                           │
                        │ 4. Keyframe decision      │
                        │    (tracking ratio)       │
                        │                           │
                        │ 5. If KF: triangulate     │
                        │    unconnected obs        │
                        │                           │
                        │ 6. Lost landmark detect   │
                        │                           │
                        │ 7. optimize_and_marg()    │
                        │    ┌────────────────────┐ │
                        │    │ optimize()         │ │
                        │    │ (LM loop)          │ │
                        │    ├────────────────────┤ │
                        │    │ marginalize()      │ │
                        │    │ (Schur complement) │ │
                        │    └────────────────────┘ │
                        ├──────────────────────────┤
                        │ 8. Output state to queue  │
                        └──────────────────────────┘
```

#### 2.3.2 Initialisation

On the first frame (`!initialized`, `sqrt_keypoint_vo.cpp:185`), the estimator seeds the sliding window with a single pose at the initial transform `T_w_i_init` (identity by default, or set explicitly via `initialize(t_ns, T_w_i, ...)`). The marginalisation prior is initialised as a diagonal matrix with weight `vio_init_pose_weight` (line 83–89), anchoring the first pose to prevent gauge drift. This prior is stored in `marg_data.H` in either squared ($\mathbf{H}$) or square-root ($\mathbf{J}$) form depending on `vio_sqrt_marg`.

#### 2.3.3 Pose Prediction (Constant Position Model)

Unlike the VIO estimator, which uses IMU preintegration for state prediction (§5), the VO estimator has no inertial input. The new frame's pose is initialised as a *copy of the previous frame's pose* (line 259–261):

```cpp
PoseStateWithLin<Scalar_> next_state(opt_flow_meas->t_ns,
                                     curr_state.getPose());
```

This constant-position model assumes zero inter-frame motion, which is a poor prior during fast motion but suffices as a warm start for the optimiser when combined with the single-frame pose-only refinement (§2.3.4).

#### 2.3.4 Single-Frame Pose-Only Optimisation

Before the full sliding-window BA, the VO estimator performs a fast pose-only Gauss–Newton optimisation of the current frame against the existing landmark map (`optimize_single_frame_pose`, `src/vi_estimator/ba_base.cpp:47`). This is a $6 \times 6$ system (one pose, no landmarks) solved in 2 iterations with Levenberg–Marquardt damping. The procedure:
1. For each landmark observed in the current frame, compute the relative pose $\mathbf{T}_{t,h}$ and linearise the reprojection residual.
2. Apply Huber weighting and accumulate the $6 \times 6$ Hessian and $6 \times 1$ gradient.
3. Solve and apply the pose increment.

This step provides a substantially better initial guess for the full BA than the constant-position prediction, particularly during fast motion or after long inter-frame intervals. It is unique to the VO pipeline; the VIO estimator relies on IMU prediction instead.

#### 2.3.5 Landmark Association

For each observation in the `OpticalFlowResult` (lines 286–314), the estimator checks whether the tracked keypoint already exists as a landmark in `lmdb`. If it does, the observation is added via `lmdb.addObservation`, extending the landmark's factor graph connectivity. A counter `num_points_connected[host_frame_id]` records how many of each host keyframe's landmarks are tracked by the current frame — this is later used by the keyframe-dropping heuristic (§6, `doc/Marginalisation.md`).

Keypoints not yet in `lmdb` are collected in `unconnected_obs0` for potential triangulation upon the next keyframe decision.

#### 2.3.6 Keyframe Selection and Triangulation

The keyframe decision and triangulation logic is shared with the VIO estimator and is detailed in §6 and §7. The VO-specific note is that, because there is no IMU prediction, the VO estimator is more sensitive to the keyframe spacing: too-frequent keyframes waste computation without improving geometry, while too-sparse keyframes may fail to provide sufficient baseline for triangulation.

#### 2.3.7 Joint Optimisation

The `optimize()` method (line 995) implements the Levenberg–Marquardt loop described in §2.2.9. The VO-specific implementation details are:

- **Variable ordering.** An `AbsOrderMap aom` is constructed from the marginalisation prior and all `frame_poses`. Each entry contributes `POSE_SIZE = 6` dimensions (lines 1011–1023). `frame_states` is asserted empty (line 1026) — there are no IMU states in VO.
- **Linearisation.** `LinearizationBase<Scalar, POSE_SIZE>::create(this, aom, lqr_options, &marg_data)` instantiates the solver *without* an `ImuLinData` argument (compare with the VIO call, which passes `&ild`). The lineariser evaluates only reprojection and marginalisation-prior residuals.
- **LM loop.** Each iteration: `linearizeProblem` → `performQR` → `get_dense_H_b` → LDLT solve → `backSubstitute` → `applyInc` → cost evaluation → accept/reject (lines 1061–1358).
- **Lambda reset.** The VO estimator resets $\lambda$ to `vio_lm_lambda_initial` at the start of each frame (line 1033), as does the VIO estimator at `sqrt_keypoint_vio.cpp:1200`. (Correction, 2026-08-31: this entry previously claimed that the VIO estimator instead carries $\lambda$ across frames and that the reset was VO-specific. Both estimators reset it identically, so the asymmetry described no behaviour of either.)

#### 2.3.8 Marginalisation

The `marginalize()` method (line 519) follows the same Schur-complement procedure described in `doc/Marginalisation.md`, but operates exclusively on 6-DoF pose blocks (no velocity or bias blocks). Key differences from the VIO marginalisation:
- **Non-keyframe removal.** All `frame_poses` that are neither keyframes nor the current frame are erased immediately (lines 530–541), without entering the Schur complement.
- **Keyframe dropping.** Uses the same feature-tracking-ratio and DSO-inspired spatial-distribution scoring as VIO (lines 545–618).
- **Index partitioning.** Every entry in `aom` contributes exactly `POSE_SIZE` dimensions; the assertion at line 820 enforces this.
- **Re-centring.** After the Schur complement, the prior residual is compensated by the current delta from the linearisation point (lines 965–967), identically to VIO.

#### 2.3.9 Key Data Structures

| Structure | Type | Role |
|---|---|---|
| `frame_poses` | `aligned_map<int64_t, PoseStateWithLin<Scalar>>` | Active 6-DoF poses (keyframes + current). |
| `frame_states` | *always empty in VO* | Asserted empty; 15-DoF states exist only in VIO. |
| `lmdb` | `LandmarkDatabase<Scalar>` | Landmark storage and observation index. |
| `kf_ids` | `std::set<int64_t>` | Timestamps of active keyframes. |
| `prev_opt_flow_res` | `aligned_map<int64_t, OpticalFlowResult::Ptr>` | Recent frontend outputs for triangulation history. |
| `marg_data` | `MargLinData<Scalar>` | Square-root marginalisation prior. |
| `lambda` / `lambda_vee` | `Scalar` | LM damping state. |

### 2.4 Limitations of Visual Odometry

Visual odometry, despite its elegance, suffers from several fundamental and practical limitations that motivate the addition of inertial sensing in a VIO system. This section catalogues these limitations, derives their mathematical origins where applicable, and connects them to the design decisions in Basalt.

#### 2.4.1 Scale Ambiguity (Monocular Case)

Consider a monocular camera observing $L$ landmarks from $K$ poses. The reprojection residual for any observation $(i, j)$ depends on the relative pose $\mathbf{T}_{t,h}$ and the 3D landmark position $\mathbf{p}_h$. If we apply a uniform scaling $s > 0$ to all translations and all landmark depths simultaneously:

$$
\mathbf{p}_{w,k} \leftarrow s \cdot \mathbf{p}_{w,k}, \qquad \mathbf{l}_j \leftarrow s \cdot \mathbf{l}_j \quad (\text{i.e., } \rho_j \leftarrow \rho_j / s),
$$

then the 3D point in any camera frame scales as $\mathbf{p}_t \leftarrow s \cdot \mathbf{p}_t$, and the projection $\pi(\mathbf{p}_t) = \pi(s \cdot \mathbf{p}_t)$ is invariant because $\pi$ is a perspective projection (division by the $z$-component absorbs the scale). Consequently, the entire reprojection cost $E_\text{BA}$ is invariant under this 7-parameter similarity transformation (3 translation + 3 rotation + 1 scale), and the Hessian $\mathbf{H}$ has a 7-dimensional null space, the gauge freedom of monocular SfM (Hartley and Zisserman, 2004; Triggs et al., 2000). In particular, the absolute scale is unobservable from vision alone.

Therefore, without an external scale reference, the trajectory recovered by monocular VO is correct only up to an unknown scale factor. This scale may drift over time as landmarks are created and destroyed. An IMU resolves this ambiguity because the specific force measurement provides a direct observation of $\|\mathbf{g}\| \approx 9.81\, \text{m/s}^2$, fixing the metric scale.

In Basalt's VO mode (`SqrtKeypointVoEstimator`), the initial pose prior (`vio_init_pose_weight`) partially anchors the gauge, and the marginalisation prior propagates this anchoring forward. However, the prior is defined at a single fixed linearisation point and does not continuously inject new scale information — scale drift remains a concern over long trajectories.

#### 2.4.2 Pure-Rotation Degeneracy

When the camera undergoes pure rotation ($\mathbf{t}_{k, k+1} = \mathbf{0}$), all scene points lie at optical infinity relative to the baseline, and the epipolar geometry degenerates: the Essential matrix $\mathbf{E} = [\mathbf{t}]_\times \mathbf{R}$ becomes the zero matrix (Scaramuzza and Fraundorfer, 2011). No translation can be recovered, and no new landmarks can be triangulated because the DLT system (§7.1) becomes rank-deficient. In Basalt, the baseline gate

$$
\|\mathbf{T}_{0,1}.\text{translation}\|^2 < (\text{vio\_min\_triangulation\_dist})^2
$$

explicitly rejects triangulation attempts with insufficient parallax. During sustained pure rotation, no new landmarks are created, existing landmarks are eventually lost as they leave the field of view, and the sliding window may exhaust its feature constraints entirely.

A VO system experiencing pure rotation (e.g., a camera panning in place) will degrade rapidly: the landmark count drops to zero, the Hessian becomes rank-deficient, and the optimiser diverges. An IMU provides direct rotation-rate observations via the gyroscope, maintaining full observability of the rotation state even in the absence of visual parallax.

#### 2.4.3 Fast Motion and Motion Blur

**Problem.** Rapid camera motion causes two compounding failures:
1. **Large inter-frame displacement.** The constant-position prediction used by the VO estimator (§2.3.3) places the search window for optical flow tracking far from the true correspondence. The pyramidal coarse-to-fine KLT tracker can compensate for displacements up to approximately $2^{L-1} \times r$ pixels (where $L$ is the number of pyramid levels and $r$ is the patch radius), but larger motions exceed this capture range, and tracks are lost.
2. **Motion blur.** At exposure times typical of low-cost cameras ($\sim$5–33 ms), fast motion smears features across the image, destroying the local gradient structure that the KLT tracker relies upon. The photometric error surface becomes flat, and the Gauss–Newton iteration fails to converge.

Both effects cause a sudden drop in the number of tracked features, triggering an avalanche of keyframe decisions and poorly-conditioned triangulations. In extreme cases, the estimator enters a state of cascading failure from which it cannot recover leading to the VO estimator with no mechanism to predict the inter-frame motion. An IMU provides a high-rate motion prior that can be used to predict the search window for feature tracking (pre-rotating the image or predicting the feature displacement), greatly extending the tracker's capture range during aggressive manoeuvres. In the VIO pipeline, the IMU preintegration (§4) provides the warm start that replaces the VO's constant-position model, and the IMU residual (§3.1.1) provides rotation and velocity constraints even when visual tracking is temporarily lost.

#### 2.4.4 Rolling Shutter and Other Camera Artefacts

CMOS cameras read the sensor row by row, so different rows correspond to slightly different times. During fast motion, this causes geometric distortion: straight lines appear curved, and the effective projection model becomes time-dependent. A standard global-shutter projection model introduces systematic errors that bias the BA solution. Basalt currently assumes global-shutter cameras; handling rolling shutter would require augmenting the projection model with a per-row pose interpolation (Kerl et al., 2015).

KLT tracking assumes brightness constancy. Automatic exposure or gain adjustments between frames violate this assumption, causing the photometric residual to become biased. The affine illumination model used by some direct methods (Engel et al., 2018) can partially compensate, but Basalt's patch tracker does not currently incorporate illumination modelling.

Strong light sources can cause flare artefacts that create spurious intensity patterns, corrupting both feature detection and tracking. These are particularly problematic for outdoor robotics. No purely visual method can reliably distinguish flare from genuine scene texture; the only recourse is physical (lens hoods, coatings) or to rely on the IMU to maintain state estimates through periods of visual corruption.

#### 2.4.5 Summary of VO Limitations

The following table summarises the structural limitations of vision-only estimation and how each is addressed by the addition of inertial sensing in VIO:

| Limitation | Root Cause | VO Impact | VIO Resolution |
|---|---|---|---|
| Scale ambiguity | Perspective projection invariance | Trajectory up to unknown scale | Gravity magnitude fixes scale |
| Pure-rotation degeneracy | Zero baseline $\Rightarrow$ zero Essential matrix | No triangulation, rank-deficient Hessian | Gyroscope provides rotation directly |
| Fast motion | Large displacement exceeds KLT capture range | Track loss, cascading KF decisions | IMU predicts search window |
| Motion blur | Exposure-time smearing destroys gradients | KLT convergence failure | IMU maintains state through blur |
| Rolling shutter | Row-sequential readout | Systematic projection bias | Time-stamped IMU interpolates per-row pose |
| Illumination change | Brightness constancy violation | Biased photometric residual | IMU is illumination-invariant |

These limitations motivate the extension from VO to VIO, which is the subject of the remaining sections of this document. §3 assembles the visual-inertial estimation problem, stating the sliding-window state vector, deriving the inertial residuals and their Jacobians, and combining them with the reprojection residual of §2.2.4 and the marginalisation prior into a single objective, before walking through the estimator that solves it. The three ingredients §3 draws upon are then developed in their own right, with §4 deriving the preintegration itself, §5 the state prediction it enables, and §7 the triangulation that seeds the visual factors.

---

## 3. VIO Frame to Frame Tracking

Every entry in the table closing §2.4 has the same shape, namely a direction of the state space along which the visual cost is flat, or a regime in which the photometric assumptions underpinning the frontend cease to hold. Scale is unobservable because perspective projection is invariant to a uniform rescaling of the scene, rotation about a stationary camera centre yields no parallax and therefore no triangulation, and rapid motion drives the true inter-frame displacement beyond the capture range of the pyramidal tracker. None of these failures is a defect of the estimator. They are properties of the measurement itself, and no amount of additional visual processing removes them. Adding a second sensor whose error model is complementary is therefore the only structural remedy, and the inertial measurement unit is that sensor. The accelerometer observes specific force in metres per second squared, which fixes the metric scale through the known magnitude of gravity, the gyroscope observes angular rate directly, which keeps rotation observable when parallax vanishes, and both are sampled at a rate an order of magnitude above the camera, which supplies a motion prior over exactly the interval in which vision is blind.

The price of the second sensor is a larger state and a second residual family. Because the accelerometer measures specific force rather than position, the inertial measurement can only be related to the trajectory through two integrations, which forces linear velocity into the state vector, and because the sensor's bias drifts slowly over minutes it must be estimated online alongside the trajectory rather than calibrated once. The per-frame state therefore grows from the six degrees of freedom of §2 to fifteen, being pose, velocity, gyroscope bias and accelerometer bias, and the objective acquires an inertial residual coupling every consecutive pair of states together with a random-walk residual coupling their biases.

At the highest level the estimator is a two-input, single-thread consumer. Optical flow results arrive on one bounded queue and raw inertial samples on another, and `SqrtKeypointVioEstimator::ProcessFrame` drains the inertial queue up to the timestamp of the next image, folding the samples into a single preintegrated pseudo-measurement. That pseudo-measurement is handed to `SqrtKeypointVioEstimator::measure`, which uses it to predict the new state, associates the frame's observations against the landmark database, decides whether the frame becomes a keyframe and triangulates new landmarks if it does, and finally runs a Levenberg–Marquardt optimisation over the whole sliding window followed by a marginalisation step that evicts the oldest variables while retaining their information as a prior. The remainder of this section states that problem precisely in §3.1 and then walks through its realisation in code in §3.2. The preintegration itself is derived in §4, the prediction equations in §5, and the marginalisation machinery in `doc/Marginalisation.md`.

### 3.1 VIO Backend

The state estimation is cast as a fixed-lag smoothing, or sliding window, non-linear least squares optimisation problem. A factor graph is formed in which nodes represent state variables and edges represent measurement constraints, or factors.

Variables:
1.   Optimised Variables (Active State, $\mathbf{s}$):
    1.   $\mathbf{s}_k$: Body poses $\mathbf{T}_{w,i_k} \in SE(3)$ for a set of older, selected keyframes, held in `frame_poses` and contributing `POSE_SIZE = 6` dimensions each.
    2.   $\mathbf{s}_f$: Full navigation states for the most recent frames in the sliding window, held in `frame_states` and contributing `POSE_VEL_BIAS_SIZE = 15` dimensions each. Each state comprises the body pose $\mathbf{T}_{w,i} \in SE(3)$, the linear velocity $\mathbf{v}_i \in \mathbb{R}^3$ expressed in the world frame, and the IMU biases $\mathbf{b}_i = [\mathbf{b}_i^{g\,T}, \mathbf{b}_i^{a\,T}]^T \in \mathbb{R}^6$.
    3.   $\mathbf{s}_l$: Landmark parameters $\mathbf{l}_j$, parameterised by a two-dimensional stereographic direction and an inverse distance relative to their host frame exactly as in §2.2.3, contributing 3 dimensions each.
2.   Non-Optimised Variables (Fixed or Prior Context):
    1.   Marginalised states that are no longer actively optimised but influence the current estimate through the marginalisation prior $E_\text{marg}(\mathbf{s})$.
    2.   Fixed calibration parameters, namely the camera intrinsics and the camera-to-IMU extrinsics $\mathbf{T}_{i,c}$.
3.   Configuration Parameters:
    1.   The gravity vector $\mathbf{g}$, the noise covariances for the IMU ($\Sigma_{a}, \Sigma_{g}, \Sigma_{b_a}, \Sigma_{b_g}$) and for vision ($\Sigma_\text{vis}$), and the threshold heuristics governing keyframe selection and marginalisation.

The tangent-space ordering of a full navigation state is fixed by `PoseVelBiasState::applyInc` (`thirdparty/basalt-headers/include/basalt/imu/imu_types.h:221`) as $[\delta\mathbf{p}^T,\, \delta\boldsymbol{\varphi}^T,\, \delta\mathbf{v}^T,\, \delta\mathbf{b}_g^T,\, \delta\mathbf{b}_a^T]^T \in \mathbb{R}^{15}$, with the pose part following the same left-multiplicative convention introduced in §2.2.4, so that $\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}$ and $\mathbf{R} \leftarrow \text{Exp}(\delta\boldsymbol{\varphi})\,\mathbf{R}$. Every Jacobian in the remainder of this section is taken with respect to this ordering and this convention.

(Correction, 2026-08-31: this variable list previously described $\mathbf{s}_k$ and the pose part of $\mathbf{s}_f$ as camera poses. The estimator stores body, that is IMU, poses $\mathbf{T}_{w,i}$ and composes the fixed extrinsic $\mathbf{T}_{i,c}$ only when a residual needs a camera frame, as §2.2.1 shows, so the earlier wording named the wrong frame. It also listed the bias vector as $[\mathbf{b}^a, \mathbf{b}^g]$, whereas `applyInc` places the gyroscope bias first.)

#### 3.1.1 IMU Residual

The inertial factor relates two consecutive navigation states through the preintegrated pseudo-measurement accumulated between their timestamps. The construction of that pseudo-measurement, namely the mid-point integrator, the covariance propagation and the bias Jacobians, is derived in §4.1, and only its interface is needed here. Preintegration returns a delta state $\Delta \mathbf{s} = (\Delta \mathbf{R}, \Delta \mathbf{v}, \Delta \mathbf{p})$ of duration $\Delta t$, expressed in the body frame at the start of the interval and computed at fixed linearisation biases $(\mathbf{b}_g^\text{lin}, \mathbf{b}_a^\text{lin})$, together with the $9\times 3$ Jacobians $\mathbf{J}_{\Delta|b_g}$ and $\mathbf{J}_{\Delta|b_a}$ of that delta with respect to those biases and the $9\times 9$ covariance $\boldsymbol{\Sigma}_\Delta$.

**First-order bias correction.** Re-integrating hundreds of samples every time the optimiser nudges a bias would defeat the purpose of preintegration, so the delta is instead corrected to first order about its linearisation biases. Writing $\Delta\mathbf{b}_g = \mathbf{b}_g - \mathbf{b}_g^\text{lin}$ and $\Delta\mathbf{b}_a = \mathbf{b}_a - \mathbf{b}_a^\text{lin}$, the two correction vectors are

$$
\boldsymbol{\delta}_g = \mathbf{J}_{\Delta|b_g}\, \Delta\mathbf{b}_g \in \mathbb{R}^9, \qquad
\boldsymbol{\delta}_a = \mathbf{J}_{\Delta|b_a}\, \Delta\mathbf{b}_a \in \mathbb{R}^9,
$$

each partitioned into position, rotation and velocity segments in that order. The rotation segment of $\boldsymbol{\delta}_a$ vanishes identically, because the orientation delta is integrated from the gyroscope alone and carries no dependence on the accelerometer bias, and the implementation asserts exactly this at `preintegration.h:234`.

**Residual.** Let $\mathbf{s}_0 = (\mathbf{R}_0, \mathbf{p}_0, \mathbf{v}_0)$ and $\mathbf{s}_1 = (\mathbf{R}_1, \mathbf{p}_1, \mathbf{v}_1)$ denote the two states the factor connects. The nine-vector inertial residual, ordered as position, rotation and velocity to match the state layout above, is

$$
\begin{aligned}
\mathbf{r}_p &= \mathbf{R}_0^T\!\left(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0 \Delta t - \tfrac{1}{2}\mathbf{g}\,\Delta t^2\right) - \left(\Delta\mathbf{p} + \boldsymbol{\delta}_g[1{:}3] + \boldsymbol{\delta}_a[1{:}3]\right), \\
\mathbf{r}_R &= \text{Log}\!\left(\text{Exp}\!\left(\boldsymbol{\delta}_g[4{:}6]\right)\, \Delta\mathbf{R}\, \mathbf{R}_1^{-1} \mathbf{R}_0\right), \\
\mathbf{r}_v &= \mathbf{R}_0^T\!\left(\mathbf{v}_1 - \mathbf{v}_0 - \mathbf{g}\,\Delta t\right) - \left(\Delta\mathbf{v} + \boldsymbol{\delta}_g[7{:}9] + \boldsymbol{\delta}_a[7{:}9]\right),
\end{aligned}
$$

as computed by `IntegratedImuMeasurement::residual` (`thirdparty/basalt-headers/include/basalt/imu/preintegration.h:221`). The structure is the same in all three rows. The bracketed term on the left transports the state difference into the body frame at time $0$ and removes the contribution of gravity, which was deliberately excluded from the preintegration, leaving a quantity directly comparable with the delta. The bracketed term on the right is the bias-corrected delta. The residual vanishes precisely when the two states are consistent with the integrated inertial measurement, which is the same condition the prediction of §5.1 enforces by construction, so a freshly predicted state produces a zero inertial residual and the optimiser starts from the inertial manifold.

(Correction, 2026-08-31: this residual previously appeared, in the section of `doc/Marginalisation.md` that has since been moved here, with the rotation row written as $\text{Log}((\Delta\mathbf{R}\hat{\mathbf{R}})^T \mathbf{R}_k^T \mathbf{R}_{k+1})$, which is the convention of Forster et al. (2017) rather than the one Basalt implements. Basalt forms $\Delta\mathbf{R}\,\mathbf{R}_1^{-1}\mathbf{R}_0$, the group inverse of the Forster argument up to conjugation. The two agree in norm, and therefore in cost, but not in sign or in Jacobian structure, so the corrected form above is the one from which §3.1.2 differentiates. The position and velocity rows were already stated correctly and are unchanged.)

**Bias random walk.** The biases are not constant. They are modelled as a discrete-time Gaussian random walk between consecutive states, contributing two further three-vector residuals per inertial factor,

$$
\mathbf{r}_{b_g} = \mathbf{W}_{b_g}\left(\mathbf{b}_{g,0} - \mathbf{b}_{g,1}\right), \qquad
\mathbf{r}_{b_a} = \mathbf{W}_{b_a}\left(\mathbf{b}_{a,0} - \mathbf{b}_{a,1}\right),
$$

with the diagonal weights $\mathbf{W}_{b_g} = \text{diag}(\boldsymbol{\sigma}_{b_g})^{-1}/\sqrt{\Delta t}$ and $\mathbf{W}_{b_a} = \text{diag}(\boldsymbol{\sigma}_{b_a})^{-1}/\sqrt{\Delta t}$ formed at `imu_block.hpp:76` and `:91` from the per-axis standard deviations $\boldsymbol{\sigma}_{b_\bullet}$ supplied by the calibration. The $\Delta t^{-1/2}$ scaling is what makes the factor a random walk rather than a fixed penalty, since the variance of a Wiener increment grows linearly in the elapsed time and its inverse square root therefore weights long intervals more loosely.

**Whitening and assembly.** The nine inertial rows are whitened by the square-root inverse of the preintegrated covariance, $\tilde{\mathbf{r}} = \boldsymbol{\Sigma}_\Delta^{-1/2}\mathbf{r}$, obtained from the cached factorisation `get_sqrt_cov_inv()`, whereas the six bias rows carry their weights inside $\mathbf{W}_{b_\bullet}$ as written above. `ImuBlock::linearizeImu` (`include/basalt/linearization/imu_block.hpp:26`) stacks all fifteen whitened rows into a single residual vector $\mathbf{r} \in \mathbb{R}^{15}$ and a single Jacobian $\mathbf{J}_p \in \mathbb{R}^{15 \times 30}$ spanning the two states the factor touches, which is the form the linearisation classes of §3.1.5 consume.

#### 3.1.2 Jacobians of the IMU Residual

Differentiating the residual of §3.1.1 requires three pieces of machinery beyond the $\mathfrak{se}(3)$ expansion already established in §2.2.4, all of them concerning $SO(3)$.

**Preliminary 1, the perturbation convention.** Orientation is perturbed on the left and position, velocity and bias additively, following `applyInc`, so a state is written $\mathbf{R} = \text{Exp}(\delta\boldsymbol{\varphi})\bar{\mathbf{R}}$, $\mathbf{p} = \bar{\mathbf{p}} + \delta\mathbf{p}$, $\mathbf{v} = \bar{\mathbf{v}} + \delta\mathbf{v}$ and $\mathbf{b} = \bar{\mathbf{b}} + \delta\mathbf{b}$, and every derivative below is evaluated at zero perturbation. To first order $\text{Exp}(\delta\boldsymbol{\varphi}) = \mathbf{I} + [\delta\boldsymbol{\varphi}]_\times + O(\|\delta\boldsymbol{\varphi}\|^2)$, and consequently $\mathbf{R}^{-1} = \bar{\mathbf{R}}^{-1}\text{Exp}(-\delta\boldsymbol{\varphi}) \approx \bar{\mathbf{R}}^{-1}(\mathbf{I} - [\delta\boldsymbol{\varphi}]_\times)$.

**Preliminary 2, the conjugation identity.** For any $\mathbf{R} \in SO(3)$ and $\mathbf{a} \in \mathbb{R}^3$, $\mathbf{R}^{-1}[\mathbf{a}]_\times \mathbf{R} = [\mathbf{R}^{-1}\mathbf{a}]_\times$. This follows from the fact that a rotation is an orthogonal map preserving the cross product, $\mathbf{R}^{-1}(\mathbf{a} \times \mathbf{R}\mathbf{x}) = (\mathbf{R}^{-1}\mathbf{a}) \times \mathbf{x}$, and it is the identity that moves a perturbation from the world frame into the body frame wherever a rotation is inverted.

**Preliminary 3, the left and right Jacobians of $SO(3)$.** The logarithm is a nonlinear map, so differentiating $\text{Log}$ of a perturbed product produces a correction factor. For a fixed $\mathbf{A} \in SO(3)$ with $\boldsymbol{\phi} = \text{Log}(\mathbf{A})$,

$$
\frac{\partial}{\partial \boldsymbol{\epsilon}}\Big|_{\boldsymbol{\epsilon}=\mathbf{0}} \text{Log}\!\left(\mathbf{A}\,\text{Exp}(\boldsymbol{\epsilon})\right) = \mathbf{J}_r^{-1}(\boldsymbol{\phi}), \qquad
\frac{\partial}{\partial \boldsymbol{\epsilon}}\Big|_{\boldsymbol{\epsilon}=\mathbf{0}} \text{Log}\!\left(\text{Exp}(\boldsymbol{\epsilon})\,\mathbf{A}\right) = \mathbf{J}_l^{-1}(\boldsymbol{\phi}),
$$

where $\mathbf{J}_r$ and $\mathbf{J}_l$ are the right and left Jacobians of $SO(3)$ and satisfy $\mathbf{J}_l(\boldsymbol{\phi}) = \mathbf{J}_r(-\boldsymbol{\phi})$. Whether the right or the left inverse appears is decided entirely by which side of the product the perturbation ends up on, and both occur in this residual. Basalt provides them as `Sophus::rightJacobianInvSO3` and `Sophus::leftJacobianInvSO3`.

Two abbreviations shorten the derivation. Let

$$
\mathbf{t} = \mathbf{R}_0^T\!\left(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0 \Delta t - \tfrac{1}{2}\mathbf{g}\Delta t^2\right), \qquad
\mathbf{t}_2 = \mathbf{R}_0^T\!\left(\mathbf{v}_1 - \mathbf{v}_0 - \mathbf{g}\Delta t\right),
$$

which are precisely the local variables `tmp` and `tmp2` in the implementation, so that $\mathbf{r}_p = \mathbf{t} - (\cdots)$ and $\mathbf{r}_v = \mathbf{t}_2 - (\cdots)$ with the parenthesised terms independent of the pose and velocity.

**Derivative with respect to the first state.** The position row depends on $\mathbf{p}_0$, $\boldsymbol{\varphi}_0$ and $\mathbf{v}_0$. The two additive dependences are immediate, $\partial \mathbf{r}_p/\partial \delta\mathbf{p}_0 = -\mathbf{R}_0^T$ and $\partial \mathbf{r}_p/\partial \delta\mathbf{v}_0 = -\mathbf{R}_0^T \Delta t$. For the rotational dependence, substitute the perturbed inverse of Preliminary 1 and write $\mathbf{u} = \mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t - \tfrac{1}{2}\mathbf{g}\Delta t^2$ so that $\mathbf{t} = \mathbf{R}_0^T\mathbf{u}$,

$$
\mathbf{t}(\delta\boldsymbol{\varphi}_0) = \mathbf{R}_0^T\!\left(\mathbf{I} - [\delta\boldsymbol{\varphi}_0]_\times\right)\mathbf{u} = \mathbf{t} - \mathbf{R}_0^T [\delta\boldsymbol{\varphi}_0]_\times \mathbf{R}_0\, \mathbf{t} = \mathbf{t} - \left[\mathbf{R}_0^T \delta\boldsymbol{\varphi}_0\right]_\times \mathbf{t} = \mathbf{t} + [\mathbf{t}]_\times \mathbf{R}_0^T\, \delta\boldsymbol{\varphi}_0,
$$

where the second equality inserts $\mathbf{R}_0\mathbf{R}_0^T = \mathbf{I}$, the third applies Preliminary 2, and the fourth uses the antisymmetry $[\mathbf{a}]_\times\mathbf{b} = -[\mathbf{b}]_\times\mathbf{a}$. Hence $\partial \mathbf{r}_p/\partial \delta\boldsymbol{\varphi}_0 = [\mathbf{t}]_\times \mathbf{R}_0^T$. The velocity row is the same computation with $\mathbf{t}_2$ in place of $\mathbf{t}$, giving $\partial \mathbf{r}_v/\partial \delta\boldsymbol{\varphi}_0 = [\mathbf{t}_2]_\times \mathbf{R}_0^T$ and $\partial \mathbf{r}_v/\partial \delta\mathbf{v}_0 = -\mathbf{R}_0^T$.

For the rotation row, collect the factors standing to the left of $\mathbf{R}_0$ into $\mathbf{A} = \text{Exp}(\boldsymbol{\delta}_g[4{:}6])\,\Delta\mathbf{R}\,\mathbf{R}_1^{-1}$, so that $\mathbf{r}_R = \text{Log}(\mathbf{A}\mathbf{R}_0)$. Perturbing gives

$$
\text{Log}\!\left(\mathbf{A}\,\text{Exp}(\delta\boldsymbol{\varphi}_0)\,\mathbf{R}_0\right) = \text{Log}\!\left(\mathbf{A}\mathbf{R}_0 \cdot \mathbf{R}_0^{-1}\text{Exp}(\delta\boldsymbol{\varphi}_0)\mathbf{R}_0\right) = \text{Log}\!\left(\mathbf{A}\mathbf{R}_0\, \text{Exp}\!\left(\mathbf{R}_0^T \delta\boldsymbol{\varphi}_0\right)\right),
$$

using the adjoint property $\mathbf{R}^{-1}\text{Exp}(\boldsymbol{\epsilon})\mathbf{R} = \text{Exp}(\mathbf{R}^{-1}\boldsymbol{\epsilon})$, which is the group-level form of Preliminary 2. The perturbation now sits on the right of the product, so the right Jacobian inverse of Preliminary 3 applies and $\partial \mathbf{r}_R/\partial \delta\boldsymbol{\varphi}_0 = \mathbf{J}_r^{-1}(\mathbf{r}_R)\, \mathbf{R}_0^T$.

**Derivative with respect to the second state.** Only three blocks are non-zero. Position and velocity enter linearly and undifferentiated by any rotation of their own, giving $\partial \mathbf{r}_p/\partial \delta\mathbf{p}_1 = \mathbf{R}_0^T$ and $\partial \mathbf{r}_v/\partial \delta\mathbf{v}_1 = \mathbf{R}_0^T$. For the rotation row, set $\mathbf{B} = \text{Exp}(\boldsymbol{\delta}_g[4{:}6])\,\Delta\mathbf{R}$ and perturb $\mathbf{R}_1$, so that $\mathbf{R}_1^{-1} \to \mathbf{R}_1^{-1}\text{Exp}(-\delta\boldsymbol{\varphi}_1)$ and

$$
\text{Log}\!\left(\mathbf{B}\,\mathbf{R}_1^{-1}\text{Exp}(-\delta\boldsymbol{\varphi}_1)\,\mathbf{R}_0\right) = \text{Log}\!\left(\mathbf{B}\mathbf{R}_1^{-1}\mathbf{R}_0\, \text{Exp}\!\left(-\mathbf{R}_0^T \delta\boldsymbol{\varphi}_1\right)\right),
$$

by the same adjoint move, whence $\partial \mathbf{r}_R/\partial \delta\boldsymbol{\varphi}_1 = -\mathbf{J}_r^{-1}(\mathbf{r}_R)\, \mathbf{R}_0^T$. Note that the second state contributes nothing to the inertial residual through its biases. The biases of the interval enter only at its start, and the bias of the second state is constrained solely by the random-walk residual.

**Derivative with respect to the biases.** The corrections $\boldsymbol{\delta}_g$ and $\boldsymbol{\delta}_a$ are subtracted from the position and velocity rows, so those rows differentiate to the negated preintegration bias Jacobians, $\partial \mathbf{r}_{p,v}/\partial \mathbf{b}_\bullet = -\mathbf{J}_{\Delta|b_\bullet}[1{:}3 \text{ and } 7{:}9,\, :]$. The rotation row is different, because there the gyroscope correction enters inside an exponential standing on the extreme left of the product. Writing $\mathbf{C} = \Delta\mathbf{R}\,\mathbf{R}_1^{-1}\mathbf{R}_0$ and perturbing the gyroscope bias by $\delta\mathbf{b}_g$ gives, to first order, $\text{Log}(\text{Exp}(\mathbf{J}^R_{\Delta|b_g}\delta\mathbf{b}_g)\,\mathbf{C})$, a left perturbation, so the left Jacobian inverse applies and

$$
\frac{\partial \mathbf{r}_R}{\partial \mathbf{b}_g} = \mathbf{J}_l^{-1}(\mathbf{r}_R)\, \mathbf{J}^R_{\Delta|b_g}, \qquad \frac{\partial \mathbf{r}_R}{\partial \mathbf{b}_a} = \mathbf{0},
$$

the sign being positive here, in contrast to the negated position and velocity blocks, because the gyroscope correction is composed with the delta rather than subtracted from it. The accelerometer block vanishes for the reason already given in §3.1.1, namely that the rotation delta has no accelerometer dependence.

**Assembled blocks.** Collecting the results, with columns ordered $[\delta\mathbf{p}, \delta\boldsymbol{\varphi}, \delta\mathbf{v}, \delta\mathbf{b}_g, \delta\mathbf{b}_a]$ and rows ordered $[\mathbf{r}_p, \mathbf{r}_R, \mathbf{r}_v, \mathbf{r}_{b_g}, \mathbf{r}_{b_a}]$,

$$
\frac{\partial \mathbf{r}}{\partial \mathbf{s}_0} =
\begin{bmatrix}
-\mathbf{R}_0^T & [\mathbf{t}]_\times \mathbf{R}_0^T & -\mathbf{R}_0^T \Delta t & -\mathbf{J}^p_{\Delta|b_g} & -\mathbf{J}^p_{\Delta|b_a} \\
\mathbf{0} & \mathbf{J}_r^{-1}(\mathbf{r}_R)\mathbf{R}_0^T & \mathbf{0} & \mathbf{J}_l^{-1}(\mathbf{r}_R)\mathbf{J}^R_{\Delta|b_g} & \mathbf{0} \\
\mathbf{0} & [\mathbf{t}_2]_\times \mathbf{R}_0^T & -\mathbf{R}_0^T & -\mathbf{J}^v_{\Delta|b_g} & -\mathbf{J}^v_{\Delta|b_a} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{W}_{b_g} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{W}_{b_a}
\end{bmatrix},
$$

$$
\frac{\partial \mathbf{r}}{\partial \mathbf{s}_1} =
\begin{bmatrix}
\mathbf{R}_0^T & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & -\mathbf{J}_r^{-1}(\mathbf{r}_R)\mathbf{R}_0^T & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{R}_0^T & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & -\mathbf{W}_{b_g} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & -\mathbf{W}_{b_a}
\end{bmatrix}.
$$

The upper-left nine-by-nine corners of these two matrices are exactly the `d_res_d_state0` and `d_res_d_state1` computed at `preintegration.h:254-276`, the bias columns of the first nine rows are `d_res_d_bg` and `d_res_d_ba` at `:278-290`, and the two weight rows are written directly by `ImuBlock::linearizeImu` at `imu_block.hpp:81-99`. Every non-zero entry of the first nine rows carries a factor $\mathbf{R}_0^T$ or a preintegration bias Jacobian, which reflects that the whole residual is expressed in the body frame at the start of the interval.

**Sparsity.** The inertial factor touches exactly two states, so it contributes a $15 \times 15$ diagonal block to each of them and a $15 \times 15$ off-diagonal block coupling them. Since the factors are chained along consecutive timestamps, their aggregate contribution to the reduced camera system of §3.1.5 is block-tridiagonal, in contrast to the visual factors whose contribution couples any keyframe pair sharing a landmark.

**Evaluation point and FEJ.** `ImuBlock::linearizeImu` evaluates both the residual and the Jacobians at the linearisation point, `getStateLin()`, and then, if either endpoint has already been frozen by marginalisation, re-evaluates the residual value alone at the current estimate (`imu_block.hpp:50-55`). Holding the Jacobian at the first estimate while letting the residual follow the state is the First-Estimate Jacobians discipline discussed in §3.1.3, and it is what keeps the nullspace of the accumulated prior aligned with the genuinely unobservable directions.

#### 3.1.3 Marginalisation Residual

When variables such as old keyframes, velocities and biases are removed from the active state to cap computational complexity, the constraints connected to them must not simply be discarded. As discussed in Usenko et al. (2020) and Mazuran et al. (2015), discarding these variables would lead to a significant loss of information and rapid accumulation of drift. Instead, the information from the marginalised variables is compressed into a dense quadratic prior on the remaining coupled variables, the Markov blanket. This prior is parameterised by an information matrix $\mathbf{H}^{\ast}$ and an information vector $\mathbf{b}^{\ast}$, derived analytically by performing the Schur complement on the partitioned linearised system, as detailed in `doc/Marginalisation.md` §3.2.

For the non-linear least squares optimisation, this newly constructed prior must be incorporated into the total objective function. This yields a marginalisation penalty acting on the active state increment $\boldsymbol{\xi}$,

$$
E_\text{marg}(\boldsymbol{\xi}) = \mathbf{b}^{\ast T} \boldsymbol{\xi} + \frac{1}{2} \boldsymbol{\xi}^T \mathbf{H}^{\ast} \boldsymbol{\xi}.
$$

To optimise this objective alongside the visual and inertial residuals using Gauss-Newton or Levenberg-Marquardt solvers, the marginalisation energy is conventionally reformulated as a sum of squares. The relationship between the energy $E_\text{marg}(\boldsymbol{\xi})$ and its equivalent squared residual $\mathbf{r}_\text{marg}(\boldsymbol{\xi})$ is $E_\text{marg}(\boldsymbol{\xi}) = \frac{1}{2}\|\mathbf{r}_\text{marg}(\boldsymbol{\xi})\|^2 + C$. To find this residual, the information matrix $\mathbf{H}^{\ast}$ is decomposed by a square-root decomposition, Cholesky or eigenvalue, such that $\mathbf{J}_\text{marg}^T \mathbf{J}_\text{marg} = \mathbf{H}^{\ast}$. The corresponding residual vector is then

$$
\mathbf{r}_\text{marg}(\boldsymbol{\xi}) = \mathbf{J}_\text{marg}\, \boldsymbol{\xi} + \mathbf{r}_{\text{marg},0}.
$$

Expanding the squared norm,

$$
\frac{1}{2}\left\|\mathbf{J}_\text{marg}\boldsymbol{\xi} + \mathbf{r}_{\text{marg},0}\right\|^2 = \frac{1}{2}\boldsymbol{\xi}^T \mathbf{J}_\text{marg}^T \mathbf{J}_\text{marg} \boldsymbol{\xi} + \mathbf{r}_{\text{marg},0}^T \mathbf{J}_\text{marg} \boldsymbol{\xi} + \frac{1}{2}\mathbf{r}_{\text{marg},0}^T \mathbf{r}_{\text{marg},0},
$$

and matching the linear terms against $E_\text{marg}(\boldsymbol{\xi})$ identifies the constant offset vector $\mathbf{r}_{\text{marg},0}$. It must satisfy $\mathbf{J}_\text{marg}^T \mathbf{r}_{\text{marg},0} = \mathbf{b}^{\ast}$, whose minimum-norm solution is $\mathbf{r}_{\text{marg},0} = (\mathbf{J}_\text{marg}^T)^{+}\, \mathbf{b}^{\ast}$. Minimising the squared norm of $\mathbf{r}_\text{marg}(\boldsymbol{\xi})$ therefore reproduces the behaviour of minimising $E_\text{marg}(\boldsymbol{\xi})$ exactly. A complete step-by-step derivation of this formulation, including the required background on quadratic forms, positive semi-definite square roots and the Moore-Penrose pseudoinverse, is presented in `doc/Marginalisation.md` Appendix A, and the Schur complement that produces $\mathbf{H}^{\ast}$ and $\mathbf{b}^{\ast}$ in the first place is derived in `doc/Marginalisation.md` §3.2.

The inclusion of the marginalisation residual is vital because it anchors the actively optimised states to the historically accumulated visual-inertial constraints, significantly improving both tracking robustness and accuracy. A critical mathematical consideration when adding this residual is the proper handling of unobservable state directions, namely global position and yaw around the gravity vector. The true VIO system has a nullspace corresponding to these dimensions, meaning the system possesses no absolute information about them. When constructing $\mathbf{H}^{\ast}$, its nullspace should perfectly align with these unobservable directions. However, if the active variables were to be relinearised around new estimates in subsequent optimisation steps, the Jacobians would change. This inconsistency would alter the nullspace of the marginalisation prior relative to the current state, causing the non-linear solver to artificially introduce non-zero information, a spurious information gain, along the global position and yaw dimensions. To prevent this, the system enforces the First-Estimate Jacobians approach. As soon as a variable becomes part of the marginalisation prior, its linearisation point $\mathbf{s}_0$ is permanently fixed for all future evaluations of its associated Jacobians, which is the `getStateLin()` evaluation already noted in §3.1.2. By freezing the linearisation point, the mathematical structure of $\mathbf{H}^{\ast}$ is preserved, ensuring that the marginalisation residual only penalises deviations in the observable subspace, maintaining global consistency and preventing long-term drift.

#### 3.1.4 VIO Objective

The three residual families are now in hand, the reprojection residual of §2.2.4, the inertial and bias random-walk residuals of §3.1.1, and the marginalisation residual above. The optimal state $\mathbf{s}^{\ast}$ is the minimiser of their weighted sum of squares,

$$
E(\mathbf{s}) = \underbrace{\frac{1}{2}\sum_{(i,j) \in \mathcal{V}} \mathbf{r}_{ij}^T\, \boldsymbol{\Sigma}_\text{vis}^{-1}\, \mathbf{r}_{ij}}_{\text{reprojection}} + \underbrace{\frac{1}{2}\sum_{k \in \mathcal{I}} \mathbf{r}_k^T\, \boldsymbol{\Sigma}_k^{-1}\, \mathbf{r}_k}_{\text{inertial}} + \underbrace{\frac{1}{2}\sum_{k \in \mathcal{I}} \left(\mathbf{r}_k^{b_g\,T} \mathbf{W}_{b_g} \mathbf{r}_k^{b_g} + \mathbf{r}_k^{b_a\,T} \mathbf{W}_{b_a} \mathbf{r}_k^{b_a}\right)}_{\text{bias random walk}} + E_\text{marg}(\mathbf{s}),
$$

where $\mathcal{V}$ is the set of valid visual observations, feature $j$ observed in frame $i$, and $\mathcal{I}$ is the set of preintegrated inertial measurements connecting consecutive frames. The visual term is the bundle adjustment objective $E_\text{BA}$ of §2.2.6 unchanged, including its Huber robustification and its isotropic covariance $\boldsymbol{\Sigma}_\text{vis} = \sigma_\text{obs}^2 \mathbf{I}_2$, and the marginalisation term is the same $E_\text{marg}$ that already appeared in the vision-only objective $E_\text{VO}$ of §2.2.6. Setting the inertial and bias terms to zero and shrinking every fifteen-dimensional state to its pose recovers $E_\text{VO}$ exactly, which is the precise sense in which VIO extends rather than replaces the estimator of §2.

Two properties of this objective deserve emphasis. First, the weights are not free parameters of the optimiser. $\boldsymbol{\Sigma}_k$ is the covariance propagated during preintegration and grows with the length of the interval, $\mathbf{W}_{b_\bullet}$ derives from the manufacturer's bias-stability figures, and $\boldsymbol{\Sigma}_\text{vis}$ is a fixed pixel noise, so the relative influence of vision and inertia is set by the sensor models rather than tuned. `doc/Marginalisation.md` §2 discusses the consequences of this, including the heteroscedastic alternative used by ORB-SLAM3 for the visual term. Second, the addition of the inertial term shrinks the nullspace of the problem from the seven dimensions of monocular structure from motion, catalogued in §2.4.1, to four, namely global position and rotation about the gravity vector. Scale becomes observable through the accelerometer, and roll and pitch become observable because gravity is a fixed direction in the world frame that the accelerometer sees at every sample. This is the formal counterpart of the qualitative claims in the table of §2.4.5.

#### 3.1.5 Gauss–Newton Linearisation and the Normal Equations

The objective is non-linear in every variable group, so it is minimised iteratively by replacing it at each step with the quadratic model obtained from a first-order expansion of the residuals on the manifold. The general derivation, from $\mathbf{r}(\mathbf{s} \oplus \boldsymbol{\xi}) \approx \mathbf{r}(\mathbf{s}) + \mathbf{J}\boldsymbol{\xi}$ through to the normal equations $\mathbf{H}\boldsymbol{\xi} = \mathbf{b}$ with $\mathbf{H} = \mathbf{J}^T\mathbf{W}\mathbf{J}$ and $\mathbf{b} = -\mathbf{J}^T\mathbf{W}\mathbf{r}$, is carried out in `doc/Marginalisation.md` §2.1 and is not repeated here. What changes relative to the vision-only case of §2.2.7 is the structure of $\mathbf{J}$ and hence of $\mathbf{H}$.

The stacked increment is ordered by the `AbsOrderMap`, which assigns each active variable a start index and a block size, six for a pure keyframe pose in `frame_poses`, fifteen for a full navigation state in `frame_states` and three for a landmark. Against this ordering the Jacobian acquires rows from three sources. A reprojection residual contributes a $2 \times 3$ landmark block and two pose blocks obtained by the chain rule of §2.2.5, where for a frame held as a full state only the leading six columns of its fifteen are touched, since a landmark projection depends on neither velocity nor bias. An inertial factor contributes the fifteen rows of §3.1.2 spanning two consecutive full states. The marginalisation prior contributes its own affine rows on whatever subset of variables the previous prior retained.

The arrowhead argument of §2.2.8 survives this extension untouched. Landmarks still appear in no factor other than their own observations, so $\mathbf{H}_{ll}$ remains block-diagonal with $3\times 3$ blocks, and the landmark elimination, whether by Schur complement or by the Householder QR of the `ABS_QR` path, proceeds exactly as before. The inertial and prior factors simply never enter the landmark columns and pass through the elimination unchanged. What differs is the reduced system that survives. For VO it was a dense $6K \times 6K$ pose system, whereas for VIO it is a mixed system of dimension $6 K_\text{poses} + 15 K_\text{states}$ in which the visual factors couple keyframe pairs sharing a landmark and the inertial factors add a block-tridiagonal chain along the recent states. The damped system $(\mathbf{H} + \lambda\,\text{diag}(\mathbf{H}))\boldsymbol{\xi} = \mathbf{b}$ is then solved by LDLT and the landmarks recovered by back-substitution, with the Nielsen damping rule and the termination criteria of §2.2.9 applying verbatim.

### 3.2 Implementation Overview

This section describes the code-level realisation of the problem stated in §3.1 by `SqrtKeypointVioEstimator<Scalar>` (`include/basalt/vi_estimator/sqrt_keypoint_vio.h`, `src/vi_estimator/sqrt_keypoint_vio.cpp`). It shadows the structure of §2.3 deliberately, so that each subsection can be read against its vision-only counterpart. Landmark association, keyframe selection, triangulation, joint optimisation and marginalisation are largely shared with the VO estimator, and the treatment below concentrates on what the inertial input changes rather than restating the common machinery.

#### 3.2.1 Pipeline Overview

The estimator consumes two asynchronous streams. `ProcessFrame` (`sqrt_keypoint_vio.cpp:201`) owns the inertial stream and converts it into one preintegrated measurement per image interval, and `measure` (`:372`) owns the visual stream and runs the estimation proper. When the producer-consumer architecture is enabled the two run inside a dedicated `std::thread` launched from `initialize` (`:193`), overlapping estimation with sensor acquisition. The following diagram summarises the flow.

```
   ┌────────────────────────┐        ┌────────────────────────┐
   │  Optical Flow Frontend │        │       IMU Driver       │
   │  (§2.1, FrameToFrame…) │        │  (accel, gyro @ ~200Hz)│
   └───────────┬────────────┘        └───────────┬────────────┘
               │ OpticalFlowResult::Ptr          │ ImuData::Ptr
               ▼                                 ▼
   ┌────────────────────────┐        ┌────────────────────────┐
   │  vision_data_queue     │        │    imu_data_queue      │
   │  (capacity 10)         │        │    (capacity 300)      │
   └───────────┬────────────┘        └───────────┬────────────┘
               │                                 │
               └───────────────┬─────────────────┘
                               ▼
          ┌──────────────────────────────────────────────┐
          │  ProcessFrame(curr_frame)      (:201)        │
          ├──────────────────────────────────────────────┤
          │ A. First frame?  →  bootstrap  (:229)        │
          │    - drain IMU up to first image             │
          │    - T_w_i_init = FromTwoVectors(accel, +Z)  │
          │    - seed frame_states, marg_data.order      │
          │                                              │
          │ B. Otherwise  →  preintegrate  (:270)        │
          │    meas = IntegratedImuMeasurement(          │
          │              prev_frame->t_ns, bg, ba)       │
          │    while (imu.t_ns <= curr_frame->t_ns)      │
          │        meas->integrate(imu, Σ_a, Σ_g)   §4   │
          │    close interval with synthetic sample      │
          └───────────────────────┬──────────────────────┘
                                  │ IntegratedImuMeasurement::Ptr
                                  ▼
          ┌──────────────────────────────────────────────┐
          │  measure(opt_flow_meas, meas)   (:372)       │
          ├──────────────────────────────────────────────┤
          │ 1. Pose prediction              (:433)  §3.2.3│
          │    meas->predictState(s_prev, g, s_new)  §5  │
          │    imu_meas[start_t_ns] = *meas              │
          │                                              │
          │ 2. Landmark association         (:448)  §3.2.4│
          │    lmdb.addObservation / unconnected_obs0    │
          │    num_points_connected[host]++              │
          │                                              │
          │ 3. Keyframe decision            (:483)  §6   │
          │    connected0 / total < thresh  &&           │
          │    frames_after_kf > min_frames              │
          │                                              │
          │ 4. If KF: triangulate           (:493)  §7   │
          │    DLT + baseline gate + inv-depth gate      │
          │                                              │
          │ 5. Lost-landmark detection      (:587)       │
          │                                              │
          │ 6. optimize_and_marg()          (:1592)      │
          │    ┌────────────────────────────────────┐    │
          │    │ optimize()   (:1149)          §3.2.6│    │
          │    │  LM loop over reprojection + IMU + │    │
          │    │  bias RW + marg prior residuals    │    │
          │    ├────────────────────────────────────┤    │
          │    │ marginalize() (:674)          §3.2.7│    │
          │    │  Schur complement, FEJ freeze      │    │
          │    └────────────────────────────────────┘    │
          │                                              │
          │ 7. Publish state, visualisation payload      │
          └───────────────────────┬──────────────────────┘
                                  ▼
                    PoseVelBiasState<Scalar>::Ptr
              (out_state_queue, out_vis_queue, out_marg_queue)
```

The comparison with the VO pipeline of §2.3.1 is instructive. Two stages present there are absent here, namely the constant-position prediction and the single-frame pose-only refinement that compensated for it, and both are replaced by the single inertial prediction of step 1. The remaining stages are structurally identical, differing only in the block size of the state and in the presence of the inertial factors inside the optimisation.

#### 3.2.2 Initialisation

Two entry points seed the filter. The five-argument `initialize(t_ns, T_w_i, vel_w_i, bg, ba)` (`:129`) is used when an external source supplies a full initial state, for instance a calibration pipeline or a ground-truth prior. It writes the state directly into `frame_states`, registers a single fifteen-dimensional entry in `marg_data.order`, and sets `initialized = true`, which suppresses the bootstrap path described below. The two-argument `initialize(bg, ba)` (`:152`) seeds only the biases, caches the discrete-time noise covariances derived from the calibration through `dicrete_time_accel_noise_std()` and its gyroscope counterpart, and starts the processing thread, leaving the pose to be determined from the first inertial sample.

In that lazy case the bootstrap runs inside `ProcessFrame` on the first image (`:229`). Inertial samples older than the image are discarded, and the initial orientation is chosen so that the measured specific force points along the world $+Z$ axis,

```cpp
T_w_i_init.setQuaternion(Eigen::Quaternion<Scalar>::FromTwoVectors(
    imuData->accel, Vec3::UnitZ()));
```

which presumes the platform is stationary at startup so that the accelerometer reads gravity alone. The velocity is seeded to zero and the biases to the values passed to `initialize`. This step is what makes roll and pitch observable from the outset and is the reason the world frame is gravity-aligned rather than aligned with the first camera. A side effect worth noting is that the rotation about the gravity axis is left arbitrary, because `FromTwoVectors` fixes only two degrees of freedom, so the initial heading is not reproducible between runs.

The initial prior is not a uniform anchor on all fifteen dimensions. The constructor (`:87-111`) populates `marg_data.H` on four groups only,

```cpp
marg_data.H.diagonal().template head<3>().setConstant(          // position
    std::sqrt(Scalar(config.vio_init_pose_weight)));
marg_data.H(5, 5) = std::sqrt(Scalar(config.vio_init_pose_weight));  // yaw
marg_data.H.diagonal().template segment<3>(9).array() =
    std::sqrt(Scalar(config.vio_init_ba_weight));
marg_data.H.diagonal().template segment<3>(12).array() =
    std::sqrt(Scalar(config.vio_init_bg_weight));
```

The selection is exactly the nullspace identified in §3.1.4. Indices $0$ to $2$ are the global position and index $5$ is the rotation about the world $Z$ axis, and these four directions carry the large weight `vio_init_pose_weight`, default $10^8$, because nothing in the measurement stream determines them. Indices $3$ and $4$, being roll and pitch, and indices $6$ to $8$, being velocity, are deliberately left unweighted, because gravity and the accelerometer already observe them and an artificial prior would bias the estimate. The remaining two groups receive small weights whose role is to prevent the biases from jumping before enough inertial excitation has accumulated to identify them. When `vio_sqrt_marg` is set the square roots of the weights are stored, since the prior is then carried as a Jacobian factor rather than an information matrix.

A defect is left standing in this block. Indices $9$ to $11$ hold the gyroscope bias and indices $12$ to $14$ the accelerometer bias, per the `applyInc` ordering recorded in §3.1, yet the code applies `vio_init_ba_weight` to the former and `vio_init_bg_weight` to the latter. The two weights are therefore exchanged relative to their names, so the defaults of $10^1$ and $10^2$ act on the gyroscope and accelerometer biases respectively rather than the other way round. The behaviour is inherited from upstream Basalt and is recorded here rather than altered, because changing it would shift the initial conditioning of every existing run and invalidate comparison against historical results. Anyone tuning these weights should set them against the block they actually reach.

#### 3.2.3 Pose Prediction (IMU Measurement Model)

Prediction replaces the constant-position model of §2.3.3 and eliminates the need for the single-frame pose-only refinement of §2.3.4. It proceeds in two stages, the first in `ProcessFrame` and the second in `measure`.

In `ProcessFrame` (`:270`) a fresh `IntegratedImuMeasurement<Scalar>` is constructed at the previous frame's timestamp, linearised about the biases of the most recent state,

```cpp
meas.reset(new IntegratedImuMeasurement<Scalar>(
    this->prev_frame->t_ns, last_state.getState().bias_gyro,
    last_state.getState().bias_accel));
```

Samples are then advanced until the buffered sample is strictly ahead of the previous frame, after which every sample with `imuData->t_ns <= curr_frame->t_ns` is folded in by `meas->integrate(*imuData, mpAccelCov, mpGyroCov)`, the mid-point integrator and covariance propagation of §4.1. Each sample passes first through `calib_accel_bias.getCalibrated` and `calib_gyro_bias.getCalibrated`, which remove the manufacturer's static bias, scale and misalignment and are orthogonal to the bias states being estimated online. If the last folded sample falls short of the image timestamp the interval is closed exactly by temporarily rewriting that sample's timestamp to `curr_frame->t_ns` and integrating once more (`:307-312`), which is a zero-order hold on the final sub-interval and guarantees the precondition that the delta's duration equals the inter-frame gap.

In `measure` (`:433`) the accumulated delta is applied to the previous state through `predictState`, whose closed form is derived in §5.1, and the delta itself is retained in `imu_meas` keyed by its start timestamp so that the optimiser can rebuild the inertial factor on every iteration,

```cpp
meas->predictState(frame_states.at(last_state_t_ns).getState(), g, next_state);
last_state_t_ns = opt_flow_meas->t_ns;
next_state.t_ns = opt_flow_meas->t_ns;
frame_states[last_state_t_ns] = PoseVelBiasStateWithLin<Scalar>(next_state);
imu_meas[meas->get_start_t_ns()] = *meas;
```

Three assertions guard this block (`:424-428`), requiring that the delta starts at the previous state, ends at the current observation, and has strictly positive duration. They catch the integration-before-prediction race in which an inertial sample is missing, a failure that otherwise manifests only as a silently divergent trajectory. Because the prediction satisfies the inertial residual of §3.1.1 identically, the optimiser begins each frame with the inertial term at zero and expends its iterations absorbing the visual correction alone.

#### 3.2.4 Landmark Association

Association is unchanged from the vision-only estimator described in §2.3.5. For every observation in the incoming `OpticalFlowResult` (`:448-481`) the estimator tests whether the tracked keypoint already exists in `lmdb`. Existing keypoints yield a `KeypointObservation<Scalar>` pushed through `lmdb.addObservation(tcid_target, kobs)`, which extends the landmark's connectivity in the factor graph, and increment the per-host counter `num_points_connected[tcid_host.frame_id]` consumed later by the keyframe-dropping heuristic. Keypoints absent from `lmdb` and seen in camera $0$ are collected in `unconnected_obs0` as candidates for triangulation. Only camera $0$ contributes to the connected count, making it the privileged host as in VO.

The inertial input changes nothing here, because a landmark observation is a function of pose and landmark alone. Its indirect effect is on the quality of the association rather than its mechanism, since a better-predicted pose keeps the frontend's search windows near the true correspondences and so raises the connected ratio that drives the next stage.

#### 3.2.5 Keyframe Selection And Triangulation

Both operations are shared with the VO estimator and are derived in §6 and §7 respectively. The keyframe test (`:483`) fires when the ratio of connected to total camera-$0$ observations falls below `vio_new_kf_keypoints_thresh` and at least `vio_min_frames_after_kf` frames have elapsed since the last keyframe, and triangulation (`:493-580`) gathers each unconnected feature's history from `prev_opt_flow_res`, unprojects the observation pair to bearings, forms the camera-to-camera transform, rejects the attempt if the baseline falls below `vio_min_triangulation_dist`, and accepts the DLT solution only if the resulting inverse distance lies in $(0, 3)$.

Two VIO-specific observations are worth recording. First, the relative pose used for the baseline gate is obtained through `getPoseStateWithLin`, which resolves a timestamp against `frame_states` before falling back to `frame_poses`. This abstraction matters more in VIO than in VO because a frame genuinely changes representation as it ages, entering the window as a fifteen-dimensional state and leaving the recent window as a six-dimensional pose, and the triangulation code must be indifferent to which it currently is. Second, the inertial prediction improves triangulation indirectly but materially, because the baseline gate is evaluated against predicted poses. A poor prediction can either admit a triangulation whose true parallax is inadequate or reject one that was in fact well conditioned, and in the vision-only case that prediction was a constant-position guess.

#### 3.2.6 Joint Optimisation

`optimize()` (`:1149`) implements the Levenberg–Marquardt loop of §3.1.5. Its skeleton matches the VO implementation of §2.3.7, and the differences are the following.

The variable ordering now mixes block sizes. `frame_poses` is walked first, contributing `POSE_SIZE = 6` per entry, and `frame_states` second, contributing `POSE_VEL_BIAS_SIZE = 15` (`:1170-1193`). Because both maps are keyed by timestamp and the keyframes that have aged into `frame_poses` are always older than the states, the resulting layout is ascending in time. Consistency with `marg_data.order` is asserted entry by entry, which guards against a desynchronisation between the prior and the active state that would otherwise corrupt the prior silently.

The inertial factors are supplied to the lineariser through an `ImuLinData<Scalar>` bundle assembled immediately before construction (`:1215-1225`),

```cpp
ImuLinData<Scalar> ild = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};
for (const auto& kv : imu_meas) {
    ild.imu_meas[kv.first] = &kv.second;
}
lqr = LinearizationBase<Scalar, POSE_SIZE>::create(this, aom, lqr_options,
                                                   &marg_data, &ild);
```

The presence of the fifth argument is the whole structural difference between the VIO and VO linearisation. The VO call passes no `ImuLinData`, and the factory consequently allocates only landmark blocks, whereas here it additionally allocates one `ImuBlock` per preintegrated measurement, wired to the same `AbsOrderMap`. The bundle carries gravity and the two square-root bias weights, which are the reciprocals of the calibration's bias standard deviations, computed once in the constructor.

Cost evaluation after each trial step accordingly has three parts rather than two. Alongside the visual error from `computeError` and the prior error from `computeMargPriorError`, the loop calls `ScBundleAdjustmentBase::computeImuError`, which re-evaluates the inertial and bias residuals at the trial state. Only the sum of the three is compared against the model-predicted decrease when deciding whether to accept the step.

The damping parameter is reset to `vio_lm_lambda_initial` at the start of every frame (`:1200`), before the loop begins.

(Correction, 2026-08-31: §2.3.7 previously stated that the VO estimator resets $\lambda$ each frame unlike the VIO estimator, which was said to carry it across frames. Both estimators reset it, VIO at `sqrt_keypoint_vio.cpp:1200` and VO at the corresponding point of its own `optimize()`, and the claimed asymmetry did not exist. That sentence has been corrected in §2.3.7.)

#### 3.2.7 Marginalisation

`marginalize()` (`:674`) bounds the problem size by eliminating old variables, and its mathematics is the Schur complement of §3.1.3 with the full treatment in `doc/Marginalisation.md` §3.2 and §3.3. What distinguishes the VIO case from the pose-only elimination of §2.3.8 is that a frame does not simply leave or stay. It can also be partially demoted.

The partition is built from three disjoint sets computed at `:708-718`. A state older than `last_state_to_marg` that was never selected as a keyframe goes into `states_to_marg_all`, and all fifteen of its indices are eliminated, since it carries neither useful visual constraints nor future kinematic ones. A state that is a keyframe goes into `states_to_marg_vel_bias`, and only its trailing nine indices, the velocity and the two biases, are eliminated while its leading six pose indices are retained, which is precisely the transition of a frame from `frame_states` into `frame_poses`. The boundary state `last_state_to_marg` itself is retained whole. Keyframe eviction from `kf_ids` then proceeds by the feature-tracking-ratio test followed by the spatial-redundancy score, unchanged from VO and documented in `doc/Marginalisation.md` §3.1.

Two further VIO-specific concerns arise. Inertial factors spanning states inside the marginalised subproblem must be included in the system before elimination, which is done by populating a second `ImuLinData` at `:846` and `:969` exactly as during optimisation, since omitting them would discard the very information the prior exists to preserve. And `frame_states.at(last_state_to_marg).setLinTrue()` (`:1044`) is the moment at which the newest retained state has its linearisation point frozen, implementing the First-Estimate Jacobians condition of §3.1.3. After the complement is formed the prior is re-centred by `marg_data.b -= marg_data.H * delta`, so that the next optimisation evaluates it around a zero increment.

---

## 4. IMU Preintegration

Inertial Measurement Units sample angular velocity $\boldsymbol{\omega}_k$ and specific force $\mathbf{a}_k$ at frequencies (typically 200–1000 Hz) that far exceed the camera frame rate (20–60 Hz). A naïve optimizer that treated each IMU sample as an independent factor and kept its endpoints as optimization variables would incur a linear re-integration cost every time a pose, velocity, or bias changed during a non-linear iteration. Preintegration (Lupton and Sukkarieh, 2012; Forster et al., 2017) folds the raw samples between two camera timestamps into a single pseudo-measurement expressed in the reference frame of the first state, decoupling the heavy integration from the optimization variables.

The canonical reference is Forster et al. (2017), *On-Manifold Preintegration for Real-Time Visual–Inertial Odometry*, IEEE T-RO. Basalt deviates from the Forster formulation in two respects: (i) it stores the delta state as a `PoseVelState<Scalar>` with a full $SE(3) \times \mathbb{R}^3$ structure and propagates it in Euclidean body frame rather than separately accumulating $\Delta R, \Delta v, \Delta p$; (ii) it uses a mid-point (trapezoidal) integrator for the rotation update to reduce discretization error at moderate sampling rates.

### 4.1 Mathematical Formulation

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

**Residual.** The preintegrated delta, its covariance and its bias Jacobians are consumed by the inertial residual that couples two consecutive navigation states, together with the bias random-walk residual that constrains their biases. Both are stated in §3.1.1 and differentiated in §3.1.2 and are not repeated here. One caveat belongs with the preintegration rather than with the residual, namely that the first-order bias correction $\boldsymbol{\delta}_\bullet = \mathbf{J}_{\Delta|b_\bullet}\Delta\mathbf{b}_\bullet$ is exact only for small $\Delta\mathbf{b}$. If the biases drift substantially within a single interval the delta must be re-integrated, which in Basalt happens implicitly through marginalisation, since that fixes the linearisation biases of every state evicted from the active window and the intervals themselves are short.

### 4.2 Implementation Overview

The preintegration code lives in the header-only library `thirdparty/basalt-headers/include/basalt/imu/preintegration.h`; it is re-exported through `include/basalt/imu/preintegration.h` and used verbatim by the estimator.

**Class `IntegratedImuMeasurement<Scalar>`** — (`thirdparty/basalt-headers/include/basalt/imu/preintegration.h:49`).
It stores four pieces of information:
- `PoseVelState<Scalar> delta_state_` — the accumulated $\Delta \mathbf{s}$ (9-vector). Its `t_ns` field holds the elapsed nanoseconds $\Delta t_{\text{ns}}$.
- `MatNN cov_` — the $9\times 9$ measurement covariance $\boldsymbol{\Sigma}_{\Delta}$ (lazily square-root inverted in `compute_sqrt_cov_inv`).
- `MatN3 d_state_d_ba_`, `MatN3 d_state_d_bg_` — the $9\times 3$ bias-correction Jacobians.
- `Vec3 bias_gyro_lin_, bias_accel_lin_` — the biases used as the linearization point.

**Static method `propagateState`** — (`preintegration.h:76`). Given the current state and one sample, it produces the predicted state and optionally the three Jacobians $(d_\text{next}/d_\text{curr},\, d_\text{next}/d_\text{accel},\, d_\text{next}/d_\text{gyro})$. Lines 89–100 implement the mid-point integrator exactly as in §4.1.

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
- `measure()` for state prediction (see §5),
- `ImuBlock::linearizeImu` for residual/Jacobian assembly (see §8),
- `ScBundleAdjustmentBase::computeImuError` for cost evaluation during LM step-acceptance checks.

---

## 5. State Prediction

A good initial guess is essential for the success of any non-linear least-squares iteration. At the moment a new image arrives, the most recently optimized state is at most one keyframe-period old; the IMU has been silently accumulating delta measurements. Before surrendering the new timestamp to the optimizer, Basalt integrates those measurements analytically to forecast the new pose, velocity, and biases. The forecast is consistent with the IMU residual (a zero residual implies the forecast satisfies it exactly), so the optimizer typically only needs a few iterations to absorb visual corrections.

### 5.1 Mathematical Formulation

Given the previous state $\mathbf{s}_0 = (\mathbf{R}_0, \mathbf{p}_0, \mathbf{v}_0, \mathbf{b}_{a,0}, \mathbf{b}_{g,0})$, gravity $\mathbf{g}$, and a preintegrated measurement $\Delta \mathbf{s} = (\Delta \mathbf{R}, \Delta \mathbf{v}, \Delta \mathbf{p})$ of duration $\Delta t$, the predicted state $\mathbf{s}_1$ is

$$
\begin{aligned}
\mathbf{R}_1 &= \mathbf{R}_0\, \Delta \mathbf{R}, \\
\mathbf{v}_1 &= \mathbf{v}_0 + \mathbf{g}\,\Delta t + \mathbf{R}_0\,\Delta \mathbf{v}, \\
\mathbf{p}_1 &= \mathbf{p}_0 + \mathbf{v}_0\,\Delta t + \tfrac{1}{2}\mathbf{g}\,\Delta t^2 + \mathbf{R}_0\,\Delta \mathbf{p}.
\end{aligned}
$$

Biases are propagated by identity: $\mathbf{b}_{a,1} = \mathbf{b}_{a,0}$, $\mathbf{b}_{g,1} = \mathbf{b}_{g,0}$. This is the "zero-bias-random-walk" choice — a consistent prior for the optimizer to adjust.

Note that the delta quantities are held in the start-of-interval body frame by construction (§4.1), so $\mathbf{R}_0$ appears as a pre-multiplier on $\Delta \mathbf{v}$ and $\Delta \mathbf{p}$. Gravity enters the prediction but is absent from the preintegration, which is why the residual function of §4.1 mirrors this structure.

### 5.2 Implementation Overview

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

## 6. Keyframe Selection

Running bundle adjustment on every frame would saturate the sliding window in under a second. Basalt therefore uses a two-tier strategy: the full navigation state (pose, velocity, biases) is retained for the last `vio_max_states` frames (default 3), while longer-lived visual constraints are preserved only for a set of *keyframes* (up to `vio_max_kfs`, default 7). A keyframe is spawned when the current frame is unable to re-observe enough landmarks hosted by existing keyframes. This heuristic is a specialisation of the classical "visual redundancy" criterion used in PTAM, ORB-SLAM, and DSO.

**Rationale.** Visual constraints lose value when tracked landmarks gradually drift out of the camera's field of view. When the overlap ratio falls below a threshold, new landmarks must be created, which requires a new host frame — a new keyframe. The counter-indication is motion-blur-induced track loss, where a *temporary* drop in track count should not be promoted to a permanent keyframe. The combined rule below guards against this by also requiring a minimum number of frames to have elapsed since the previous keyframe.

### 6.1 Implementation Overview

The check is centralised in `SqrtKeypointVioEstimator::measure` (`sqrt_keypoint_vio.cpp:411`):

```cpp
if (Scalar(connected0) / (connected0 + unconnected_obs0.size()) <
        Scalar(config.vio_new_kf_keypoints_thresh) &&
    frames_after_kf > config.vio_min_frames_after_kf)
    take_kf = true;
```

Here `connected0` counts observations in camera 0 whose corresponding landmark is already in the `LandmarkDatabase` (`sqrt_keypoint_vio.cpp:386`), and `unconnected_obs0` is the set of camera-0 keypoints seen but not yet triangulated. The configuration knobs are `vio_new_kf_keypoints_thresh` (ratio threshold, default 0.7) and `vio_min_frames_after_kf` (hysteresis counter, default 5).

**Connected-landmark accounting.** Lines 380–409 iterate over every observation in every camera of the current `OpticalFlowResult`. For each (camera, keypoint) pair:
- If the keypoint already exists in `lmdb`, a `KeypointObservation<Scalar>` is created and pushed via `lmdb.addObservation(tcid_target, kobs)`, extending the landmark's observation set. A per-host counter `num_points_connected[tcid_host.frame_id]` is incremented — this map is later consumed by the marginalisation keyframe-dropping heuristic (see §3.1 of `doc/Marginalisation.md`).
- If the keypoint does not exist and this is camera 0, its id is placed in `unconnected_obs0`, marking it for potential triangulation on the next keyframe (§7).

**Stereo asymmetry.** Only camera 0 (`i == 0` on lines 402, 404) contributes to the ratio. Stereo correspondences in camera 1 still populate the landmark database but do not trigger keyframe creation — the estimator treats camera 0 as the privileged host.

**Outcome.** When `take_kf` is set (`sqrt_keypoint_vio.cpp:421`), three actions follow:
1. `take_kf` is reset and `frames_after_kf` is zeroed.
2. `kf_ids.emplace(last_state_t_ns)` registers the current frame as a keyframe — the set `std::set<int64_t> kf_ids` is the authoritative record consulted during marginalisation (`sqrt_keypoint_vio.cpp:426`).
3. Triangulation of all `unconnected_obs0` keypoints runs (see §7). If `take_kf` is false, the counter advances (`frames_after_kf++;`) and no landmarks are added.

**Dropping a keyframe.** Eviction from `kf_ids` is the concern of `marginalize(...)` — detailed in `doc/Marginalisation.md` §3.1.

---

## 7. Triangulation

When a new keyframe is created, Basalt promotes previously-untriangulated 2D keypoints to 3D landmarks so that subsequent frames can benefit from their visual constraints. Three sub-operations occur in sequence:

1. **Observation gathering** — for each untriangulated feature id, the estimator traverses the recent `prev_opt_flow_res` history and collects every 2D observation, keyed by `TimeCamId`.
2. **Triangulation** — the algorithm tries each gathered observation in turn as the "other" view; once a baseline exceeds `vio_min_triangulation_dist` and the DLT solution yields a positive, bounded inverse-depth, the landmark is accepted.
3. **Lost-landmark filtering** — landmarks not observed in the current frame are flagged for optional marginalisation, decoupling map growth from the active state vector.

These steps work together to balance two competing pressures: insufficient baseline produces ill-conditioned depth (ambiguous up to scale), while waiting too long delays the inclusion of a feature and may drop it entirely if tracking is lost.

### 7.1 Mathematical Formulation

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
\pi(\mathbf{d}) = \left[\frac{d_x}{d+d_z},\; \frac{d_y}{d+d_z}\right]^T, \quad d = |\mathbf{d}|.
$$

The landmark is stored as `(Vec2 direction, Scalar inv_dist)` in `Keypoint<Scalar>` (`include/basalt/vi_estimator/landmark_database.h:53`). This gives 3 DOF per landmark instead of the 4 of a homogeneous point, eliminating the gauge freedom of the 4-vector's scale. Stereographic projection is smooth and bijective except at the antipodal point $-\mathbf{z}$ — a benign restriction because landmarks always lie in front of the host camera ($d_z > 0$).

**Baseline gate.** The triangulation attempt is rejected if

$$
\|\mathbf{T}_{0,1}.\text{translation}\|^2 < (\text{vio\_min\_triangulation\_dist})^2.
$$

This suppresses the numerical ill-conditioning of near-zero-parallax DLT solutions.

**Inverse-depth gate.** After triangulation, the homogeneous point is accepted only if $w = \rho$ is positive and bounded (`p0_triangulated[3] > 0 && p0_triangulated[3] < 3.0`). The upper bound $\rho < 3$ corresponds to a minimum depth of $\sim 33\,\text{cm}$ and rejects near-singular triangulations where tiny errors in bearing explode into meter-scale depth variations.

### 7.2 Implementation Overview

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
3. Baseline check (§7.1), skip if failed.
4. Call the static method `BundleAdjustmentBase::triangulate(p0_3d, p1_3d, T_0_1)` (`ba_base.h:99`), which runs the DLT and normalises.
5. If the depth gate passes, construct a `Keypoint<Scalar>` with `host_kf_id = tcidl`, `direction = StereographicParam::project(p0_triangulated)`, `inv_dist = p0_triangulated[3]`, and insert via `lmdb.addLandmark(lm_id, kpt_pos)`.
6. Loop-break via `valid_kp = true` — the first successful baseline wins.

If triangulation succeeds, *all* gathered observations (including host-frame and future-frame views) are pushed to the landmark via `lmdb.addObservation`. If it fails, the feature is silently abandoned; the next frame sees it again and may succeed once the baseline grows.

**Lost-landmark detection** (`sqrt_keypoint_vio.cpp:515`). If `config.vio_marg_lost_landmarks` is enabled, every landmark currently in `lmdb` is tested against the current observation set; any not observed in any camera of the current frame is added to `lost_landmaks` and forwarded to `marginalize(...)`. There the landmark's residual block participates in the Schur complement and is then deleted — its information is absorbed into the marginalisation prior rather than being discarded.

**`LandmarkDatabase<Scalar>`** — (`include/basalt/vi_estimator/landmark_database.h:86`). Holds two maps: `kpts` (keypoint id $\to$ landmark) and `observations` (host `TimeCamId` $\to$ target `TimeCamId` $\to$ set of landmark ids). The split indexing allows both fast landmark look-up (§8, visual residual evaluation) and fast host-based eviction (§3.1 of `doc/Marginalisation.md`). The `addObservation` method conveniently auto-populates both structures.

---

## 8. Optimisation and Marginalisation

With the window populated (§5), the landmark set updated (§7), and the keyframe topology decided (§6), Basalt solves a local non-linear least squares problem over all active variables. The cost combines three factor families:

1. **Reprojection factors** for every (landmark, observation) pair.
2. **IMU preintegration factors** for every consecutive state pair (plus bias random-walk residuals).
3. **The marginalisation prior** — a Gaussian factor on the Markov blanket of previously evicted states, maintained in square-root form.

Marginalisation, treated thoroughly in `doc/Marginalisation.md`, is the mechanism by which older variables are analytically eliminated through a Schur-complement block-update while their information is retained as a quadratic prior. This subsection focuses on optimisation proper and cross-references the marginalisation document where appropriate.

### 8.1 Mathematical Formulation

**Total objective.** Let $\mathbf{s}$ denote the stack of active pose-only frame poses (`frame_poses`, 6-DoF each), full navigation states (`frame_states`, 15-DoF each), and landmarks (`lmdb`, 3-DoF each). Basalt minimises the objective assembled in §3.1.4, restated here for reference,

$$
E(\mathbf{s}) = \underbrace{\tfrac{1}{2} \sum_{(i,j)\in\mathcal{V}} \mathbf{r}_{ij}(\mathbf{s})^T \boldsymbol{\Sigma}_\text{vis}^{-1} \mathbf{r}_{ij}(\mathbf{s})}_{\text{reprojection}} + \underbrace{\tfrac{1}{2} \sum_{k\in\mathcal{I}} \mathbf{r}_k^{\text{IMU}}(\mathbf{s})^T \boldsymbol{\Sigma}_k^{-1} \mathbf{r}_k^{\text{IMU}}(\mathbf{s})}_{\text{inertial}} + \underbrace{\tfrac{1}{2} \sum_{k\in\mathcal{I}} \left(\mathbf{r}_k^{b_a\,T} \boldsymbol{W}_{b_a} \mathbf{r}_k^{b_a} + \mathbf{r}_k^{b_g\,T} \boldsymbol{W}_{b_g} \mathbf{r}_k^{b_g}\right)}_{\text{bias random walk}} + E_{\text{marg}}(\mathbf{s}),
$$

with Huber robustification applied to the visual residuals. The marginalisation prior energy $E_{\text{marg}}$ is formulated in §3.1.3 and derived in full in `doc/Marginalisation.md` Appendix A.

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

### 8.2 Implementation Overview

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
- The `marg_data` structure is maintained across calls (§8.1 mathematical formulation), with `is_sqrt = config.vio_sqrt_marg`.
- IMU factors that span only active states are added to the marginalised system (`ild.imu_meas[kv.first] = &kv.second;` at line 787), exactly as during optimisation.
- `frame_states.at(last_state_to_marg).setLinTrue()` (line 973) is the point at which the newest-to-be-kept state has its FEJ linearisation fixed.

**Parallelism.** Every expensive loop in the linearisation pipeline is a `tbb::parallel_for` or `tbb::parallel_reduce`. The dense LM solve is serial but benefits from Eigen's vectorisation. The producer–consumer architecture (`sqrt_keypoint_vio.cpp:162`) runs the estimator in its own `std::thread`, overlapping optimisation with sensor acquisition.

---

## 9. Key Classes and Interfaces

The references below collate every non-trivial type, method, and field touched by the preceding sections. Paths are relative to the repository root `/ws/ros_ws/src/slam/ext/basalt/`.

### 9.1 `SqrtKeypointVioEstimator<Scalar>`

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

### 9.2 `IntegratedImuMeasurement<Scalar>`

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
| `predictState(state0, g, state1)` | member const | Forecast of §5. |
| `residual(state0, g, state1, bg, ba, J0, J1, Jbg, Jba)` | member const | 9-vector IMU residual + optional Jacobians. |
| `get_cov_inv()`, `get_sqrt_cov_inv()` | member const | Weight matrices for residual whitening. |
| `get_dt_ns()`, `get_start_t_ns()` | member const | Interval bounds. |

### 9.3 State wrappers and layouts — `include/basalt/utils/imu_types.h`

- **`PoseVelBiasStateWithLin<Scalar>`** (`:67`). Wraps a `PoseVelBiasState` with FEJ support: fields `linearized`, `delta`, `state_linearized`, `state_current`. Methods `setLinTrue`, `applyInc`, `getState`, `getStateLin`, `backup`/`restore`.
- **`PoseStateWithLin<Scalar>`** (`:188`). Analogous 6-DoF wrapper, with conversion constructor from `PoseVelBiasStateWithLin` discarding velocity and biases.
- **`AbsOrderMap`** (`:293`). `std::map<int64_t, std::pair<int, int>> abs_order_map` + `items` + `total_size`. The canonical layout dictionary consumed by every linearisation routine.
- **`ImuLinData<Scalar>`** (`:307`). Read-only bundle `(g, gyro_bias_weight_sqrt, accel_bias_weight_sqrt, imu_meas)` passed to the IMU block.
- **`MargLinData<Scalar>`** (`:318`). Square-root or squared marginalisation prior; see `doc/Marginalisation.md`.
- **`MargData`** (`:329`). Off-thread payload for NFR; see `doc/Marginalisation.md`.

### 9.4 `BundleAdjustmentBase<Scalar>` and descendants

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

### 9.5 Linearisation classes

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

### 9.6 `LandmarkDatabase<Scalar>`

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
- `Keypoint<Scalar>` (`landmark_database.h:53`) — `direction (Vec2)`, `inv_dist (Scalar)`, `host_kf_id (TimeCamId)`, `obs: aligned_map<TimeCamId, Vec2>`. The 2-vector `direction` is the stereographic image of the unit bearing (§7.1).
- `KeypointObservation<Scalar>` (`:43`) — `kpt_id`, `pos (Vec2)`.

### 9.7 Auxiliary types

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

## 10. References

1. Usenko, V., Demmel, N., Schubert, D., Stückler, J., & Cremers, D. (2020). *Visual-Inertial Mapping with Non-Linear Factor Recovery*. IEEE Robotics and Automation Letters, arXiv:1904.06504.
2. Demmel, N., Schubert, D., Sommer, C., Cremers, D., & Usenko, V. (2021). *Square Root Marginalization for Sliding-Window Bundle Adjustment*. ICCV.
3. Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. (2017). *On-Manifold Preintegration for Real-Time Visual–Inertial Odometry*. IEEE Transactions on Robotics, 33(1), 1–21.
4. Lupton, T., & Sukkarieh, S. (2012). *Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions*. IEEE Transactions on Robotics, 28(1), 61–76.
5. Triggs, B., McLauchlan, P. F., Hartley, R. I., & Fitzgibbon, A. W. (2000). *Bundle Adjustment — A Modern Synthesis*. In Vision Algorithms: Theory and Practice, LNCS 1883, 298–372. Springer.
6. Hartley, R., & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2nd ed.). Cambridge University Press.
7. Lucas, B. D., & Kanade, T. (1981). *An Iterative Image Registration Technique with an Application to Stereo Vision*. In Proceedings of the 7th International Joint Conference on Artificial Intelligence (IJCAI), 674–679.
8. Shi, J., & Tomasi, C. (1994). *Good Features to Track*. In Proceedings of IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 593–600.
9. Bouguet, J.-Y. (2001). *Pyramidal Implementation of the Affine Lucas Kanade Feature Tracker: Description of the Algorithm*. Intel Corporation, Technical Report.
10. Mur-Artal, R., Montiel, J. M. M., & Tardós, J. D. (2015). *ORB-SLAM: A Versatile and Accurate Monocular SLAM System*. IEEE Transactions on Robotics, 31(5), 1147–1163.
11. Scaramuzza, D., & Fraundorfer, F. (2011). *Visual Odometry [Tutorial]. Part I: The First 30 Years and Fundamentals*. IEEE Robotics & Automation Magazine, 18(4), 80–92.
12. Fraundorfer, F., & Scaramuzza, D. (2012). *Visual Odometry. Part II: Matching, Robustness, Optimization, and Applications*. IEEE Robotics & Automation Magazine, 19(2), 78–90.
13. Civera, J., Davison, A. J., & Montiel, J. M. M. (2008). *Inverse Depth Parametrization for Monocular SLAM*. IEEE Transactions on Robotics, 24(5), 932–945.
14. Huber, P. J. (1964). *Robust Estimation of a Location Parameter*. The Annals of Mathematical Statistics, 35(1), 73–101.
15. Engel, J., Koltun, V., & Cremers, D. (2018). *Direct Sparse Odometry*. IEEE Transactions on Pattern Analysis and Machine Intelligence, 40(3), 611–625.
16. Kerl, C., Stückler, J., & Cremers, D. (2015). *Dense Continuous-Time Tracking and Mapping with Rolling Shutter RGB-D Cameras*. In Proceedings of IEEE International Conference on Computer Vision (ICCV), 2264–2272.
17. Usenko, V., Demmel, N., & Cremers, D. (2018). *The Double Sphere Camera Model*. In Proceedings of the International Conference on 3D Vision (3DV), 552–560.
18. Agarwal, S., Snavely, N., Seitz, S. M., & Szeliski, R. (2010). *Bundle Adjustment in the Large*. In Proceedings of the European Conference on Computer Vision (ECCV), 29–42.
19. Klein, G., & Murray, D. (2007). *Parallel Tracking and Mapping for Small AR Workspaces*. In Proceedings of IEEE/ACM International Symposium on Mixed and Augmented Reality (ISMAR), 225–234.
20. Basalt source: `src/vi_estimator/sqrt_keypoint_vio.cpp`, `src/vi_estimator/sqrt_keypoint_vo.cpp`, `include/basalt/vi_estimator/sqrt_keypoint_vio.h`, `include/basalt/vi_estimator/sqrt_keypoint_vo.h`, and the supporting `include/basalt/linearization/` and `thirdparty/basalt-headers/` trees.
21. Mazuran, M., Burgard, W., & Tipaldi, G. D. (2015). *Nonlinear Factor Recovery for Long-Term SLAM*. The International Journal of Robotics Research, 35(1–3), 50–72.
22. Sola, J., Deray, J., & Atchuthan, D. (2018). *A Micro Lie Theory for State Estimation in Robotics*. arXiv:1812.01537. (Reference for the left and right Jacobians of $SO(3)$ used in §3.1.2.)
23. `doc/Marginalisation.md` — companion document on Schur-complement marginalisation in Basalt.
