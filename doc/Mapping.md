# Visual-Inertial Mapping with Non-Linear Factor Recovery

## 1. Introduction

Visual-Inertial Odometry (VIO) thread provides real-time, locally consistent pose estimates by operating on a bounded sliding window of recent frames and keyframes. While computationally tractable, this approach fundamentally cannot correct long-term drift: once a keyframe leaves the active window it is marginalised and its pose is never revisited. A global mapping layer is therefore required to build a globally consistent 3D map over the complete trajectory of the camera setup.

The **mapping pipeline** in `basalt`  performs this role of globally consistent mapping. The output of the VIO thread — `MargData` packets emitted when keyframes are marginalised, are consumed by the mapping pipline to build a globally refined map of keyframe poses and 3D landmarks. The core insight that makes this feasible is **Non-Linear Factor Recovery (NFR)**: rather than discarding the dense information accumulated by VIO marginalisation, the mapper recovers a small set of sparse, non-linear factors that faithfully summarise the VIO's high-frequency visual-inertial constraints. These recovered factors, together with freshly detected visual features, drive a global Bundle Adjustment (BA) that can correct drift over the entire trajectory.

The mapping pipeline is tightly coupled to the VIO marginalisation output. The dense information matrix passed through `MargData` is the direct product of the Schur-complement operations described in (`doc/Marginalisation.md`)[doc/Marginalisation.md]. The quality of the recovered non-linear factors — and therefore the global map — is contingent on the VIO sliding-window optimisation having converged to a good local minimum before marginalisation. Readers unfamiliar with the VIO linearisation and marginalisation process are advised to read (`doc/Marginalisation.md`)[doc/Marginalisation.md] before proceeding.
Each `MargData` packet fed to the mapping pipeline contains:
- The dense marginal information matrix `abs_H` and vector `abs_b` over the Markov blanket of the marginalised keyframe.
- Keyframe pose estimates (`frame_poses`, `frame_states`).
- Keyframe bookkeeping sets (`kfs_all`, `kfs_to_marg`).
- The optical flow results (`opt_flow_res`) carrying the raw image data for re-detection.
- A flag `use_imu` indicating whether IMU constraints are present in the marginal.

The mapping pipeline will process the input sequence of `MargData::Ptr` to output:
- A globally refined map of keyframe poses: `Eigen::aligned_map<int64_t, PoseStateWithLin<double>> frame_poses`.
- A 3D landmark database: `LandmarkDatabase<double> lmdb` (inherited from `BundleAdjustmentBase`).

The mapping pipeline is implemented in the `NfrMapper` class (`include/basalt/vi_estimator/nfr_mapper.h` and `src/vi_estimator/nfr_mapper.cpp`). It inherits the full Bundle Adjustment implementation from `ScBundleAdjustmentBase<double>`, which in turn inherits from `BundleAdjustmentBase<double>`. In the current implementation the pipeline is **offline**: a sample driver (`src/mapper.cpp`) loads serialised `MargData` from disk using `MargDataLoader`, feeds all packets to the mapper, then executes the detection, matching, optimisation, and filtering stages sequentially.
Section 2 in the document derives the mathematics of Non-Linear Factor Recovery: how a dense marginal covariance is converted into sparse relative-pose and roll-pitch factors. Section 3 formulates the global Bundle Adjustment problem and describes the feature pipeline (detection, matching, triangulation) that supplements the recovered factors. Section 4 documents all classes, data structures, and the end-to-end execution flow in code. Section 5 concludes with a discussion of the current limitations and the path to real-time integration.

---

## 2. Non-Linear Factor Recovery

### 2.1 Motivation

As detailed in `doc/Marginalisation.md §4`, marginalising a keyframe produces a dense marginal information matrix $\mathbf{H}^*$ over its Markov blanket — the set of all remaining active keyframes that shared visual landmarks with the marginalised frame. This dense prior captures the full joint uncertainty of those keyframes as inferred from all visual-inertial measurements processed up to that point.

If one were to use $\mathbf{H}^*$ directly in a global factor graph, two problems arise:

1. **Computational intractability.** Each marginalised keyframe introduces a dense block coupling every frame in its Markov blanket to every other. As the trajectory grows, the global information matrix becomes increasingly dense, destroying the sparsity that makes graph optimisation efficient.
2. **Linearisation point fixation.** The prior $\mathbf{H}^*$ is computed at a fixed linearisation point. Reusing it across large pose changes degrades accuracy.

**Non-Linear Factor Recovery (NFR)**, introduced in Mazuran et al. [2] and applied to visual-inertial mapping in Usenko et al. [1], resolves both issues. The dense Gaussian distribution $p(\mathbf{x}) \sim \mathcal{N}(\boldsymbol{\mu}, (\mathbf{H}^*)^{-1})$ is approximated by a sparse product of non-linear factors $q(\mathbf{x})$. The optimal approximation is found by minimising the Kullback–Leibler divergence:

$$D_{\text{KL}}(p | q) = \int p(\mathbf{x}) \log (\frac{p(\mathbf{x})}{q(\mathbf{x})}) d\mathbf{x}$$

The recovered factors are standard non-linear factors and can be linearised freshly at each global optimisation iteration, eliminating the fixed-point issue. Their sparse structure preserves graph sparsity.

In the `basalt` implementation, two types of factors are recovered from each `MargData` packet:

- **`RelPoseFactor`**: A 6-DOF relative pose constraint between the marginalised keyframe and each other keyframe in its Markov blanket.
- **`RollPitchFactor`**: A 2-DOF absolute orientation constraint on the marginalised keyframe's roll and pitch, derived from the IMU gravity alignment.

### 2.2 Covariance Recovery and Propagation

The prerequisite for NFR is the marginal covariance matrix $\mathbf{C} = (\mathbf{H}^*)^{-1}$. In code, this inversion is performed in `NfrMapper::extractNonlinearFactors` (`src/vi_estimator/nfr_mapper.cpp:151`):

```cpp
Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr(m.abs_H);
if (qr.rank() != m.abs_H.cols()) return false;   // rank-deficient: skip

Eigen::MatrixXd cov_old = qr.solve(Eigen::MatrixXd::Identity(asize, asize));
```

A full-pivot Householder QR decomposition is used because the information matrix $\mathbf{H}^*$, while positive semi-definite, may be rank-deficient if the Markov blanket contains poses connected by degenerate visual constraints. A rank-deficient marginal is silently discarded (the function returns `false`) to avoid propagating ill-conditioned factors into the global graph.

At this stage the state vector indexed by `m.aom` contains only 6-DOF pose blocks (`POSE_SIZE = 6`) — the IMU velocity and bias dimensions have already been stripped by `processMargData` (see Section 4.2, Step 1).

#### 2.2.1 Law of Propagation of Uncertainty

Once $\mathbf{C}$ is recovered, it must be propagated through the non-linear measurement functions used to define the NFR factors. The mathematical basis for this is the **law of propagation of uncertainty**, derived here from first principles.

**Setup.** Let $\mathbf{x} \in \mathbb{R}^n$ be a random vector distributed as a multivariate Gaussian:

$$\mathbf{x} \sim \mathcal{N}(\boldsymbol{\mu}, \mathbf{C})$$

where $\boldsymbol{\mu} \in \mathbb{R}^n$ is the mean and $\mathbf{C} \in \mathbb{R}^{n \times n}$ is the covariance matrix (symmetric positive semi-definite). Define a linear transformation:

$$\mathbf{y} = \mathbf{J} \mathbf{x}, \quad \mathbf{J} \in \mathbb{R}^{m \times n}$$

**Step 1 — Mean of $\mathbf{y}$.** By linearity of expectation:

$$\mathbb{E}[\mathbf{y}] = \mathbb{E}[\mathbf{J}\mathbf{x}] = \mathbf{J}\,\mathbb{E}[\mathbf{x}] = \mathbf{J}\boldsymbol{\mu}$$

**Step 2 — Covariance of $\mathbf{y}$.** The covariance is defined as:

$$\mathbf{C}_y = \mathbb{E}\!\left[(\mathbf{y} - \mathbb{E}[\mathbf{y}])(\mathbf{y} - \mathbb{E}[\mathbf{y}])^T\right]$$

Substituting $\mathbf{y} - \mathbb{E}[\mathbf{y}] = \mathbf{J}(\mathbf{x} - \boldsymbol{\mu})$:

$$\mathbf{C}_y = \mathbb{E}\!\left[\mathbf{J}(\mathbf{x} - \boldsymbol{\mu})(\mathbf{x} - \boldsymbol{\mu})^T \mathbf{J}^T\right]$$

Since $\mathbf{J}$ is a constant matrix it factors out of the expectation:

$$\mathbf{C}_y = \mathbf{J}\,\underbrace{\mathbb{E}\!\left[(\mathbf{x} - \boldsymbol{\mu})(\mathbf{x} - \boldsymbol{\mu})^T\right]}_{\mathbf{C}}\,\mathbf{J}^T = \mathbf{J}\,\mathbf{C}\,\mathbf{J}^T$$

**Step 3 — Gaussianity is preserved.** A linear transformation of a Gaussian is itself Gaussian.

To prove this we use the **characteristic function** (CF). For a random vector $\mathbf{x}$, its CF is defined as the expected value of the complex exponential:

$$\phi_{\mathbf{x}}(\boldsymbol{\omega}) = \mathbb{E}\!\left[e^{i\boldsymbol{\omega}^T\mathbf{x}}\right]$$

The CF is the Fourier transform of the probability density function, and — crucially — it **uniquely identifies the distribution**: two random vectors have the same distribution if and only if their CFs are equal everywhere. This makes it an ideal tool for proving that $\mathbf{y} = \mathbf{J}\mathbf{x}$ is Gaussian without integrating a density directly.

**Deriving the Gaussian CF.** For $\mathbf{x} \sim \mathcal{N}(\boldsymbol{\mu}, \mathbf{C})$ we evaluate the expectation by completing the square in the exponent:

$$\phi_{\mathbf{x}}(\boldsymbol{\omega}) = \int \frac{1}{(2\pi)^{n/2}|\mathbf{C}|^{1/2}} \exp\!\left(-\tfrac{1}{2}(\mathbf{x}-\boldsymbol{\mu})^T\mathbf{C}^{-1}(\mathbf{x}-\boldsymbol{\mu}) + i\boldsymbol{\omega}^T\mathbf{x}\right) d\mathbf{x}$$

Combining the two exponentials and completing the square in $\mathbf{x}$ shifts the integration variable to $\mathbf{z} = \mathbf{x} - \boldsymbol{\mu} - i\mathbf{C}\boldsymbol{\omega}$, leaving a standard Gaussian integral over $\mathbf{z}$ that evaluates to 1. The remaining terms yield the **Gaussian characteristic function**:

$$\phi_{\mathbf{x}}(\boldsymbol{\omega}) = \exp\!\left(i\boldsymbol{\omega}^T\boldsymbol{\mu} - \tfrac{1}{2}\boldsymbol{\omega}^T\mathbf{C}\boldsymbol{\omega}\right)$$

The structure is always the same: a linear phase term $i\boldsymbol{\omega}^T\boldsymbol{\mu}$ encoding the mean, and a quadratic damping term $-\frac{1}{2}\boldsymbol{\omega}^T\mathbf{C}\boldsymbol{\omega}$ encoding the covariance. Any distribution whose CF has this form *must* be Gaussian.

**Applying the CF to $\mathbf{y} = \mathbf{J}\mathbf{x}$.** For $\mathbf{y} = \mathbf{J}\mathbf{x}$:

$$\phi_{\mathbf{y}}(\boldsymbol{\nu}) = \mathbb{E}[e^{i\boldsymbol{\nu}^T\mathbf{y}}] = \mathbb{E}[e^{i\boldsymbol{\nu}^T\mathbf{J}\mathbf{x}}] = \mathbb{E}[e^{i(\mathbf{J}^T\boldsymbol{\nu})^T\mathbf{x}}] = \phi_{\mathbf{x}}(\mathbf{J}^T\boldsymbol{\nu})$$

The key step is recognising that $\boldsymbol{\nu}^T(\mathbf{J}\mathbf{x}) = (\mathbf{J}^T\boldsymbol{\nu})^T\mathbf{x}$, so evaluating the CF of $\mathbf{y}$ at frequency $\boldsymbol{\nu}$ is identical to evaluating the CF of $\mathbf{x}$ at the transformed frequency $\mathbf{J}^T\boldsymbol{\nu}$. Substituting into the Gaussian CF formula:

$$\phi_{\mathbf{y}}(\boldsymbol{\nu}) = \phi_{\mathbf{x}}(\mathbf{J}^T\boldsymbol{\nu}) = \exp\!\left(i\boldsymbol{\nu}^T\mathbf{J}\boldsymbol{\mu} - \tfrac{1}{2}\boldsymbol{\nu}^T\mathbf{J}\mathbf{C}\mathbf{J}^T\boldsymbol{\nu}\right)$$

This is exactly the characteristic function of $\mathcal{N}(\mathbf{J}\boldsymbol{\mu},\, \mathbf{J}\mathbf{C}\mathbf{J}^T)$. Therefore:

$$\boxed{\mathbf{y} = \mathbf{J}\mathbf{x} \sim \mathcal{N}\!\left(\mathbf{J}\boldsymbol{\mu},\; \mathbf{J}\mathbf{C}\mathbf{J}^T\right)}$$

#### 2.2.2 Application to NFR Covariance Propagation

The NFR residuals (relative pose and roll-pitch) are non-linear functions of the joint pose state. Under a first-order linearisation around the current estimate $\boldsymbol{\mu}$, a small perturbation $\delta\mathbf{x} = \mathbf{x} - \boldsymbol{\mu} \sim \mathcal{N}(\mathbf{0}, \mathbf{C})$ induces a perturbation of any residual $\mathbf{r}(\mathbf{x})$:

$$\delta\mathbf{r} \approx \mathbf{J}\,\delta\mathbf{x}$$

where $\mathbf{J} = \frac{\partial \mathbf{r}}{\partial \mathbf{x}}\big|_{\boldsymbol{\mu}}$ is the Jacobian evaluated at the current linearisation point. By the result of Section 2.2.1, the covariance of the residual perturbation is:

$$\boldsymbol{\Sigma} = \mathbf{J}\,\mathbf{C}\,\mathbf{J}^T$$

The inverse $\boldsymbol{\Omega} = \boldsymbol{\Sigma}^{-1}$ is the **information matrix** of the recovered factor, used as its weight in the global BA objective. This is the operation performed for both factor types — `RelPoseFactor` (Section 2.3.3) and `RollPitchFactor` (Section 2.4.3).

The Jacobian $\mathbf{J} \in \mathbb{R}^{m \times n}$ is sparse by construction: it has non-zero blocks only at the columns corresponding to the specific frames involved in the residual. For the relative pose factor between frames $i$ and $j$, only the $6 \times 6$ blocks $\mathbf{J}_i$ and $\mathbf{J}_j$ are non-zero. The matrix product therefore expands as:

$$\boldsymbol{\Sigma}_{ij} = \mathbf{J}_i\,\mathbf{C}_{ii}\,\mathbf{J}_i^T + \mathbf{J}_i\,\mathbf{C}_{ij}\,\mathbf{J}_j^T + \mathbf{J}_j\,\mathbf{C}_{ji}\,\mathbf{J}_i^T + \mathbf{J}_j\,\mathbf{C}_{jj}\,\mathbf{J}_j^T$$

This expansion makes explicit the two contributions to the factor weight:
- **Individual uncertainty** ($\mathbf{C}_{ii}$, $\mathbf{C}_{jj}$): the marginal uncertainty of each frame's pose in isolation.
- **Mutual correlation** ($\mathbf{C}_{ij}$, $\mathbf{C}_{ji}$): the co-variance between the two frames' poses as jointly estimated by the VIO. If the two frames were tightly co-constrained (e.g., many shared landmarks), the off-diagonal blocks partially cancel the diagonal terms, yielding a tighter $\boldsymbol{\Sigma}_{ij}$ and therefore a higher-weight factor. If the frames were weakly coupled, $\boldsymbol{\Sigma}_{ij}$ is large and the factor is down-weighted accordingly in the global BA.

### 2.3 Relative Pose Factor

#### 2.3.1 Residual Definition

Let $\mathbf{T}_{w,i} \in SE(3)$ and $\mathbf{T}_{w,j} \in SE(3)$ be the world-frame poses of keyframe $i$ (the marginalised frame) and keyframe $j$ (any other frame in the Markov blanket), respectively. The measured relative pose is:

$$\mathbf{T}_{i,j}^{\text{meas}} = \mathbf{T}_{w,i}^{-1}  \mathbf{T}_{w,j}$$

evaluated at the current VIO estimate. The non-linear residual at perturbed poses $\tilde{\mathbf{T}}_{w,i}$, $\tilde{\mathbf{T}}_{w,j}$ is:

$$\mathbf{r}_{ij} = \text{Log}_{SE(3)}\!\left(\mathbf{T}_{i,j}^{\text{meas}} \cdot \left(\tilde{\mathbf{T}}_{w,j}^{-1} \, \tilde{\mathbf{T}}_{w,i}\right)\right) \in \mathbb{R}^6$$

where $\text{Log}_{SE(3)}(\cdot)$ is the Lie algebra logarithm on $SE(3)$, implemented as `Sophus::se3_logd`. This residual is zero when the estimated poses reproduce the measured relative transformation exactly.

Implemented in `include/basalt/utils/nfr.h::relPoseError`:

```cpp
Sophus::SE3d T_j_i = T_w_j.inverse() * T_w_i;
Sophus::Vector6d res = Sophus::se3_logd(T_i_j * T_j_i);
```

#### 2.3.2 Jacobians

The Jacobians of $\mathbf{r}_{ij}$ with respect to the left-perturbation of $\mathbf{T}_{w,i}$ and $\mathbf{T}_{w,j}$ are computed using the right inverse Jacobian of $SE(3)$ in the decoupled (translation/rotation) form, together with the adjoint representation:

$$\frac{\partial \mathbf{r}_{ij}}{\partial \delta \mathbf{T}_{w,i}} = \mathbf{J}_r^{-1}(\mathbf{r}_{ij}) \cdot \text{Adj}(\mathbf{T}_{w,i}^{-1})$$

$$\frac{\partial \mathbf{r}_{ij}}{\partial \delta \mathbf{T}_{w,j}} = -\mathbf{J}_r^{-1}(\mathbf{r}_{ij}) \cdot \text{Adj}(\mathbf{T}_{j,i}^{-1}, \mathbf{T}_{w,i}^{-1})$$

where $\mathbf{J}_r^{-1}$ is the right inverse Jacobian (`Sophus::rightJacobianInvSE3Decoupled`). The Jacobian implementation is in `nfr.h:50–69`.

In `extractNonlinearFactors`, a block-row Jacobian $\mathbf{J} \in \mathbb{R}^{6 \times n}$ is assembled placing $\frac{\partial \mathbf{r}}{\partial \delta\mathbf{T}_{w,i}}$ and $\frac{\partial \mathbf{r}}{\partial \delta\mathbf{T}_{w,j}}$ at the column blocks corresponding to frames $i$ and $j$ in the full state vector (`nfr_mapper.cpp:220–224`):

```cpp
Eigen::MatrixXd J;
J.setZero(POSE_SIZE, asize);
J.block<POSE_SIZE, POSE_SIZE>(0, kf_start_idx) = d_res_d_T_w_i;
J.block<POSE_SIZE, POSE_SIZE>(0, o_start_idx)  = d_res_d_T_w_j;
```

#### 2.3.3 Covariance Propagation

The $6 \times 6$ covariance of the relative pose factor is obtained by propagating the full marginal covariance $\mathbf{C}$ through the Jacobian:

$$\boldsymbol{\Sigma}_{ij} = \mathbf{J} \, \mathbf{C} \, \mathbf{J}^T \in \mathbb{R}^{6 \times 6}$$

The information matrix stored in the factor is the inverse:

$$\boldsymbol{\Omega}_{ij} = \boldsymbol{\Sigma}_{ij}^{-1}$$

computed via LDLT decomposition (`nfr_mapper.cpp:233`):

```cpp
Sophus::Matrix6d cov_new = J * cov_old * J.transpose();
cov_new.ldlt().solveInPlace(rpf.cov_inv);
```

If `config.mapper_no_factor_weights` is set, $\boldsymbol{\Omega}_{ij}$ is replaced by the identity matrix, treating all relative pose factors as equally weighted.

The fully constructed `RelPoseFactor` stores the timestamp pair `(t_i_ns, t_j_ns)`, the measured relative pose `T_i_j`, and the inverse covariance `cov_inv`, and is appended to `NfrMapper::rel_pose_factors`.

### 2.4 Roll-Pitch Factor

#### 2.4.1 Motivation and Observable Directions

In a VIO system without an absolute orientation reference, global position and yaw about the gravity vector are **unobservable**: no combination of visual and inertial measurements can determine them in an absolute sense. Roll and pitch, however, are directly observable from the gravity direction measured by the accelerometer. The IMU therefore anchors the roll and pitch of every keyframe, and this information is preserved in the marginal $\mathbf{H}^*$.

To extract this anchoring as a sparse factor while **not** introducing spurious information along the unobservable yaw direction, a 2-DOF roll-pitch residual is constructed rather than a full 3-DOF orientation constraint.

#### 2.4.2 Residual Definition

Let $\mathbf{R}_{w,i}^{\text{meas}} \in SO(3)$ be the measured orientation of keyframe $i$ (the marginalised frame) at the moment of marginalisation, and let $\tilde{\mathbf{R}}_{w,i}$ be the current orientation estimate. The error rotation is:

$$\Delta\mathbf{R} = \mathbf{R}_{w,i}^{\text{meas}} \cdot \tilde{\mathbf{R}}_{w,i}^{-1} \in SO(3)$$

The residual measures the deviation of the gravity direction under this error rotation. Gravity points in $-\hat{\mathbf{z}}$ in the world frame, so:

$$\mathbf{r}_{\text{rp}} = \left(\Delta\mathbf{R} \cdot (-\hat{\mathbf{z}})\right)_{x,y} \in \mathbb{R}^2$$

Only the $x$ and $y$ components are retained — these correspond to roll and pitch errors. The $z$ component, which encodes yaw, is discarded. Implemented in `include/basalt/utils/nfr.h::rollPitchError`:

```cpp
Eigen::Matrix3d R = (R_w_i_meas * T_w_i.so3().inverse()).matrix();
Eigen::Vector3d res = R * (-Eigen::Vector3d::UnitZ());
return res.head<2>();
```

#### 2.4.3 Jacobian and Covariance Propagation

The $2 \times 6$ Jacobian of $\mathbf{r}_{\text{rp}}$ with respect to the left-perturbation of the pose $\mathbf{T}_{w,i}$ is computed analytically in `nfr.h:110–118`. Only the rotation columns (indices 3–5) are non-zero:

$$\frac{\partial \mathbf{r}_{\text{rp}}}{\partial \delta\boldsymbol{\omega}} = \begin{bmatrix} 0 & -R_{01} & R_{00} \\ 0 & -R_{11} & R_{10} \end{bmatrix}$$

A full $6 \times n$ Jacobian is assembled in `extractNonlinearFactors` placing this block at the column position of the marginalised keyframe (`nfr_mapper.cpp:180–184`):

```cpp
J.block<3, POSE_SIZE>(0, kf_start_idx) = d_pos_d_T_w_i;
J.block<1, POSE_SIZE>(3, kf_start_idx) = d_yaw_d_T_w_i;
J.block<2, POSE_SIZE>(4, kf_start_idx) = d_rp_d_T_w_i;
```

The $6 \times 6$ covariance $\boldsymbol{\Sigma} = \mathbf{J} \mathbf{C} \mathbf{J}^T$ is propagated and its $2 \times 2$ roll-pitch sub-block extracted and inverted (`nfr_mapper.cpp:186–197`):

```cpp
Sophus::Matrix6d cov_new = J * cov_old * J.transpose();
rpf.cov_inv = cov_new.block<2, 2>(4, 4).inverse();
```

The $2 \times 2$ inverse covariance `cov_inv` serves as the information weight of the roll-pitch factor. Roll-pitch factors are only appended to `NfrMapper::roll_pitch_factors` when `data->use_imu == true`, since the gravity anchoring originates from the IMU preintegration.

### 2.5 IMU State Reduction

Before factor extraction can proceed, the `MargData` information matrix must be reduced to a pose-only system. The VIO marginal $\mathbf{H}^*$ is expressed over a mixed state vector containing both 6-DOF poses (`POSE_SIZE = 6`) and full 15-DOF navigation states (`POSE_VEL_BIAS_SIZE = 15`, covering pose, velocity, and IMU biases). The mapper has no use for velocity or bias dimensions, so they are eliminated via a secondary Schur complement.

`NfrMapper::processMargData` (`nfr_mapper.cpp:82`) partitions the state indices into:
- **`idx_to_keep`**: The 6 pose columns of every keyframe in `kfs_all`, plus the 6 pose columns of pure pose states.
- **`idx_to_marg`**: The 9 velocity/bias columns of every full navigation state, and all columns of non-keyframe navigation states.

The Schur complement is then applied via:

```cpp
MargHelper<Scalar>::marginalizeHelperSqToSq(
    m.abs_H, m.abs_b, idx_to_keep, idx_to_marg, marg_H_new, marg_b_new);
```

(`nfr_mapper.cpp:130–131`). The resulting `marg_H_new` is a pure $6K \times 6K$ pose-only information matrix (where $K$ is the number of keyframes), ready for the covariance inversion in `extractNonlinearFactors`.

---

## 3. Mapping

### 3.1 Problem Formulation

The global mapping problem is cast as a non-linear least squares optimisation over all keyframe poses $\{ \mathbf{T}_{w,i} \in SE(3) \}_{i=1}^{K}$ and all 3D landmark parameters $\{ \mathbf{l}_j \}_{j=1}^{L}$. The total objective function is:

$$E(\mathbf{s}) = E_{\text{vision}}(\mathbf{s}) + E_{\text{rel}}(\mathbf{s}) + E_{\text{rp}}(\mathbf{s})$$

where $\mathbf{s} = \{ \mathbf{T}_{w,i}, \mathbf{l}_j \}$ is the full state vector.

#### 3.1.1 Visual Reprojection Error

For a 3D landmark $j$ hosted in frame $h(j)$ and observed in target frame $i$ at image coordinates $\mathbf{z}_{ij} \in \mathbb{R}^2$, the reprojection residual is:

$$\mathbf{r}_{ij}^{\text{vis}} = \mathbf{z}_{ij} - \pi\!\left(\mathbf{T}_{w,i}^{-1} \, \mathbf{T}_{w,h(j)} \, \mathbf{q}_j\right)$$

where $\pi(\cdot)$ is the camera projection model and $\mathbf{q}_j$ is the 3D point reconstructed from the landmark parameters. The visual cost term is:

$$E_{\text{vision}} = \sum_{(i,j) \in \mathcal{V}} \rho\!\left(\left\|\mathbf{r}_{ij}^{\text{vis}}\right\|_{\boldsymbol{\Sigma}_{ij}^{-1}}^2\right)$$

where $\rho(\cdot)$ is the Huber loss function (threshold `mapper_obs_huber_thresh`) and $\boldsymbol{\Sigma}_{ij} = \sigma_{\text{obs}}^2 \mathbf{I}_2$ with `mapper_obs_std_dev`.

#### 3.1.2 Relative Pose Error

$$E_{\text{rel}} = \sum_{(i,j) \in \mathcal{R}} \mathbf{r}_{ij}^T \, \boldsymbol{\Omega}_{ij} \, \mathbf{r}_{ij}$$

where $\mathbf{r}_{ij} \in \mathbb{R}^6$ is the SE(3) relative pose residual defined in Section 2.3.1, and $\boldsymbol{\Omega}_{ij}$ is the recovered information matrix from Section 2.3.3.

#### 3.1.3 Roll-Pitch Error

$$E_{\text{rp}} = \sum_{k \in \mathcal{P}} \mathbf{r}_k^T \, \boldsymbol{\Omega}_k \, \mathbf{r}_k$$

where $\mathbf{r}_k \in \mathbb{R}^2$ is the roll-pitch residual defined in Section 2.4.2, and $\boldsymbol{\Omega}_k$ is the recovered $2 \times 2$ information matrix from Section 2.4.3.

### 3.2 Landmark Parameterization and Triangulation

Landmarks are parameterised identically to the VIO backend: each landmark $j$ is represented relative to its **host frame** $h(j)$ as a tuple $(u, v, \rho)$ where $(u, v) \in \mathbb{R}^2$ is the stereographic projection of the unit-sphere ray from the host camera, and $\rho > 0$ is the inverse distance (inverse depth). This minimal, singularity-free parameterisation is implemented in `StereographicParam<double>::project`.

Landmark initialisation in `setup_opt` (`nfr_mapper.cpp:697`) proceeds via Direct Linear Transform (DLT) triangulation. For each feature track, the first observation is taken as the host frame. For each subsequent observation with sufficient baseline:

$$\left\| \mathbf{T}_{h,o}.\mathbf{t} \right\|^2 \geq d_{\min}^2 \quad (\texttt{mapper\_min\_triangulation\_dist}^2)$$

the 3D point is triangulated using `BundleAdjustmentBase::triangulate` and validated:

```cpp
if (!pos_3d.array().isFinite().all() || pos_3d[3] <= 0 || pos_3d[3] > 2.0)
    continue;
```

The inverse depth is stored as `pos.inv_dist = pos_3d[3]` and the stereographic direction as `pos.direction = StereographicParam<double>::project(pos_3d)`. The landmark is then registered in the database via `lmdb.addLandmark` and all track observations are added via `lmdb.addObservation`.

### 3.3 Feature Detection

Fresh keypoint detection is performed on every keyframe image stored in `NfrMapper::img_data` (populated from `opt_flow_res` during `processMargData`). Detection runs in parallel over all frames using TBB (`nfr_mapper.cpp:465–499`):

1. **Keypoint detection**: `detectKeypointsMapping` extracts up to `mapper_detection_num_points` FAST/Harris corners per image.
2. **Angle computation**: `computeAngles` estimates the dominant orientation of each corner.
3. **Descriptor computation**: `computeDescriptors` computes binary descriptors (ORB-style) for matching.
4. **Unprojection**: Each 2D corner is unprojected to a unit-sphere ray using the calibrated camera intrinsics (`calib.intrinsics[cam_id].unproject`).
5. **BoW encoding**: `hash_bow_database->compute_bow` converts the descriptor set into a Bag-of-Words vector.
6. **Database insertion**: `hash_bow_database->add_to_database(tcid, kd.bow_vector)` registers the frame for subsequent retrieval.

Detected keypoints are stored in `NfrMapper::feature_corners` keyed by `TimeCamId` (a `(frame_id, cam_id)` pair).

### 3.4 Feature Matching

#### 3.4.1 Stereo Matching

For each stereo pair `(tcid_left, tcid_right)` at the same timestamp, `match_stereo` (`nfr_mapper.cpp:513`) uses the known stereo extrinsic calibration $\mathbf{T}_{0,1} = \mathbf{T}_{w,0}^{-1} \mathbf{T}_{w,1}$ to compute the essential matrix $\mathbf{E}$ via `computeEssential`. Descriptor matching is performed with `matchDescriptors` subject to a Hamming distance threshold `mapper_max_hamming_distance` and second-best ratio test `mapper_second_best_test_ratio`. Only geometrically verified inliers passing the essential-matrix check (`findInliersEssential`, epipolar tolerance $10^{-3}$) with at least 16 inliers are retained.

#### 3.4.2 Appearance-Based Temporal and Loop-Closure Matching

`match_all` (`nfr_mapper.cpp:555`) performs cross-frame matching for temporal and potential loop-closure connections:

1. **BoW retrieval** (parallel over all frames): For each `TimeCamId`, query the `HashBow` database for the `mapper_num_frames_to_match` most visually similar frames, excluding frames from the same timestamp. Pairs with similarity score below `mapper_frames_to_match_threshold` are discarded.
2. **Descriptor matching**: For each candidate pair, `matchDescriptors` is run with a fixed Hamming distance of 70 and ratio 1.2.
3. **Geometric verification**: `findInliersRansac` runs a RANSAC fundamental-matrix test (threshold `mapper_ransac_threshold`, minimum inliers `mapper_min_matches`) to retain only geometrically consistent matches.

All verified match pairs are stored in `NfrMapper::feature_matches`.

### 3.5 Feature Track Building

`build_tracks` (`nfr_mapper.cpp:671`) constructs multi-frame feature tracks from the pairwise matches using a `TrackBuilder` based on a Union-Find data structure:

1. **Build**: `trackBuilder.Build(feature_matches)` fuses all pairwise correspondences into consistent multi-frame tracks.
2. **Filter**: `trackBuilder.Filter(config.mapper_min_track_length)` removes any track observed in fewer than `mapper_min_track_length` frames, as well as tracks with conflicting observations (the same feature appearing twice in one frame).
3. **Export**: `trackBuilder.Export(feature_tracks)` writes the tracks as `std::map<TrackId, std::map<TimeCamId, FeatureId>>` into `NfrMapper::feature_tracks`.

### 3.6 Global Bundle Adjustment

The optimiser in `NfrMapper::optimize` (`nfr_mapper.cpp:244`) solves the objective from Section 3.1 using either Levenberg–Marquardt (LM) or Gauss-Newton (GN), controlled by `config.mapper_use_lm`.

#### 3.6.1 Linearisation

At each iteration, the system is linearised around the current state estimate:

1. **Visual linearisation**: `linearizeHelper` (`ScBundleAdjustmentBase`) linearises the visual reprojection errors for all landmarks and their host/target frame pairs. The Schur complement eliminates landmark variables analytically to yield a pose-only system (`RelLinData` contains the condensed $\mathbf{H}_{pp}$ and $\mathbf{b}_p$). This runs via TBB `parallel_reduce` over `rld_vec`.

2. **Factor linearisation**: If `config.mapper_use_factors`, the NFR factors are linearised in the same TBB reduction pass via operator overloads in `MapperLinearizeAbsReduce`:
   - `RelPoseFactor`: computes the SE(3) residual and its $6 \times 6$ Jacobians, accumulates `J^T Ω J` and `J^T Ω r` into the sparse hash accumulator.
   - `RollPitchFactor`: computes the 2D roll-pitch residual and its $2 \times 6$ Jacobian, accumulates `J^T Ω J` and `J^T Ω r`.

3. **Assembly**: After reduction, `lopt.accum` holds the full sparse Hessian approximation $\mathbf{H}$ and gradient vector $\mathbf{b}$ over all keyframe poses.

#### 3.6.2 Solve and Update

**Levenberg–Marquardt**: The diagonal damping vector is $\mathbf{d}_\lambda = \max(\text{diag}(\mathbf{H}) \cdot \lambda, \lambda_{\min}) \cdot \mathbf{1}$. The system $(\mathbf{H} + \text{diag}(\mathbf{d}_\lambda)) \, \boldsymbol{\xi} = \mathbf{b}$ is solved iteratively. If the total error decreases (`f_diff > 0`), the step is accepted and $\lambda \leftarrow \max(\lambda_{\min}, \lambda / 3)$; otherwise it is rejected, the state is restored, and $\lambda \leftarrow \min(\lambda_{\max}, \lambda_{\text{vee}} \cdot \lambda)$ with $\lambda_{\text{vee}} \leftarrow 2 \lambda_{\text{vee}}$. Parameters `mapper_lm_lambda_min` and `mapper_lm_lambda_max` bound the damping range.

**Gauss-Newton**: A single solve per outer iteration with minimal diagonal regularisation `min_lambda`.

After each accepted step, poses are updated via the exponential map on $SE(3)$:

```cpp
kv.second.applyInc(-inc.segment<POSE_SIZE>(idx));
```

Landmark inverse depths are updated by `updatePoints` (`ScBundleAdjustmentBase`), which applies the condensed landmark increment from the Schur complement.

Convergence is declared when $\|\boldsymbol{\xi}\|_\infty < 10^{-5}$.

---

## 4. Implementation in the Code

### 4a. Key Classes and Interface

The following table provides a complete reference to all classes involved in the mapping pipeline.

---

**1. `NfrMapper`**
- **Header:** `include/basalt/vi_estimator/nfr_mapper.h`
- **Source:** `src/vi_estimator/nfr_mapper.cpp`
- **Purpose:** Top-level mapper class. Orchestrates the full pipeline: ingests `MargData`, extracts NFR factors, detects features, builds tracks, and runs global BA.
- **Inheritance:** `NfrMapper` → `ScBundleAdjustmentBase<double>` → `BundleAdjustmentBase<double>`
- **Key Member Variables:**

| Variable | Type | Description |
|---|---|---|
| `rel_pose_factors` | `Eigen::aligned_vector<RelPoseFactor>` | Accumulated recovered relative pose factors from all `MargData` packets |
| `roll_pitch_factors` | `Eigen::aligned_vector<RollPitchFactor>` | Accumulated recovered roll-pitch factors |
| `img_data` | `std::unordered_map<int64_t, OpticalFlowInput::Ptr>` | Raw image data keyed by timestamp, populated from `MargData::opt_flow_res` |
| `feature_corners` | `Corners` (`tbb::concurrent_unordered_map<TimeCamId, KeypointsData>`) | Detected keypoints and descriptors per (frame, camera) |
| `feature_matches` | `Matches` | Verified pairwise feature correspondences |
| `feature_tracks` | `FeatureTracks` | Multi-frame feature tracks from Union-Find |
| `hash_bow_database` | `std::shared_ptr<HashBow<256>>` | BoW database for appearance-based frame retrieval |
| `config` | `VioConfig` | Configuration parameters (all `mapper_*` fields) |
| `lambda`, `min_lambda`, `max_lambda`, `lambda_vee` | `double` | Levenberg–Marquardt damping state |

- **Key Public Methods:**

| Method | Description |
|---|---|
| `addMargData(MargData::Ptr&)` | Entry point: calls `processMargData` then `extractNonlinearFactors`; populates `frame_poses` |
| `processMargData(MargData&)` | Strips IMU states from the marginal via Schur complement; saves image data |
| `extractNonlinearFactors(MargData&)` | Inverts pose-only H*, propagates covariance to build `RelPoseFactor` and `RollPitchFactor` |
| `detect_keypoints()` | Parallel feature detection + BoW encoding over all keyframes in `img_data` |
| `match_stereo()` | Stereo feature matching using essential matrix constraint |
| `match_all()` | BoW-guided cross-frame matching with RANSAC geometric verification |
| `build_tracks()` | Union-Find track construction and filtering |
| `setup_opt()` | Triangulates features and populates `lmdb` for optimisation |
| `optimize(int num_iterations)` | Runs global BA (LM or GN) over poses and landmarks |
| `computeRelPose(double&)` | Evaluates total relative pose factor error (for monitoring) |
| `computeRollPitch(double&)` | Evaluates total roll-pitch factor error (for monitoring) |
| `getFramePoses()` | Returns reference to the optimised keyframe pose map |

---

**2. `NfrMapper::MapperLinearizeAbsReduce<AccumT>`**
- **Header:** `include/basalt/vi_estimator/nfr_mapper.h` (inner struct)
- **Source:** N/A (template defined in header)
- **Purpose:** TBB-compatible parallel reduction functor. Accumulates the linearised contributions of visual `RelLinData`, `RollPitchFactor`, and `RelPoseFactor` into a sparse hash accumulator in parallel. Overloads `operator()` for all three range types.
- **Inheritance:** `ScBundleAdjustmentBase<Scalar>::LinearizeAbsReduce<AccumT>`
- **Key Variables:** `roll_pitch_error` (accumulated roll-pitch cost), `rel_error` (accumulated rel-pose cost), `frame_poses` (const pointer to keyframe pose map), `accum` (inherited `SparseHashAccumulator<double>`).

---

**3. `ScBundleAdjustmentBase<Scalar>`**
- **Header:** `include/basalt/vi_estimator/sc_ba_base.h`
- **Source:** `src/vi_estimator/sc_ba_base.cpp`
- **Purpose:** Provides the Schur-complement Bundle Adjustment infrastructure: relative-pose linearisation data structures, the landmark Schur complement, and the template `LinearizeAbsReduce` functor for assembling the pose-only Hessian.
- **Inheritance:** `ScBundleAdjustmentBase<Scalar>` → `BundleAdjustmentBase<Scalar>`
- **Key Data Structures:**

| Struct | Description |
|---|---|
| `RelLinData` | Per-landmark-group Schur complement data: `Hll` (landmark Hessian), `Hllinv` (inverse), `bl` (landmark gradient), `Hpppl` (pose-landmark cross terms), `d_rel_d_h`/`d_rel_d_t` (pose Jacobians) |
| `FrameRelLinData` | Per-frame contribution within a `RelLinData`: `Hpp` (pose Hessian block), `bp` (pose gradient), `Hpl` (pose-landmark cross terms) |
| `LinearizeAbsReduce<AccumT>` | Base TBB reduction functor for assembling the absolute-frame Hessian from `RelLinData` |

- **Key Static Methods:**

| Method | Description |
|---|---|
| `linearizeHelperStatic(...)` | Linearises all visual observations in parallel, producing a vector of `RelLinData` |
| `linearizeRel(rld, H, b)` | Performs the Schur complement on a single `RelLinData` to eliminate landmarks, yielding a dense pose-pose H and b |
| `linearizeAbs(rel_H, rel_b, rld, aom, accum)` | Scatters the dense pose-pose H/b block into the global sparse accumulator using `AbsOrderMap` indices |
| `updatePoints(aom, rld, inc, lmdb)` | Updates landmark inverse depths using the condensed Schur complement increment |

---

**4. `BundleAdjustmentBase<Scalar>`**
- **Header:** `include/basalt/vi_estimator/ba_base.h`
- **Source:** `src/vi_estimator/ba_base.cpp`
- **Purpose:** Core BA state and utilities. Holds the landmark database, keyframe poses, and calibration. Provides error computation, outlier filtering, triangulation, and projection utilities.
- **Inheritance:** Base class (no further inheritance).
- **Key Member Variables:**

| Variable | Type | Description |
|---|---|
| `lmdb` | `LandmarkDatabase<Scalar>` | Stores landmarks (`Keypoint`) and observations (`KeypointObservation`) |
| `frame_poses` | `Eigen::aligned_map<int64_t, PoseStateWithLin<Scalar>>` | Keyframe poses indexed by timestamp (nanoseconds) |
| `frame_states` | `Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<Scalar>>` | Full navigation states (used during IMU-active phases) |
| `calib` | `Calibration<Scalar>` | Camera intrinsics, extrinsics, and IMU-camera transform |
| `obs_std_dev` | `Scalar` | Standard deviation of visual observations |
| `huber_thresh` | `Scalar` | Huber loss threshold |

- **Key Methods:**

| Method | Description |
|---|---|
| `computeError(error, outliers, threshold)` | Computes total reprojection error; optionally collects outlier observations |
| `filterOutliers(threshold, min_num_obs)` | Removes landmarks with reprojection error above threshold or too few observations |
| `triangulate(f0, f1, T_0_1)` | DLT triangulation returning a homogeneous 4-vector `(direction, inv_dist)` |
| `get_current_points(points, ids)` | Extracts 3D world-frame landmark positions from `lmdb` |
| `computeDelta(marg_order, delta)` | Computes the state increment from the stored linearisation points |

---

**5. `MargData`**
- **Header:** `include/basalt/utils/imu_types.h:329`
- **Source:** N/A (struct defined in header)
- **Purpose:** Container for one marginalisation event emitted by the VIO thread. Carries the dense information matrix and all state/observation data needed by the mapper.
- **Key Fields:**

| Field | Type | Description |
|---|---|
| `aom` | `AbsOrderMap` | Block layout of `abs_H` / `abs_b` |
| `abs_H` | `Eigen::MatrixXd` | Dense marginal information matrix $\mathbf{H}^*$ |
| `abs_b` | `Eigen::VectorXd` | Dense marginal information vector $\mathbf{b}^*$ |
| `frame_states` | `Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<double>>` | Full 15-DOF navigation states in the window |
| `frame_poses` | `Eigen::aligned_map<int64_t, PoseStateWithLin<double>>` | Pure 6-DOF pose states |
| `kfs_all` | `std::set<int64_t>` | All keyframe timestamps in the Markov blanket |
| `kfs_to_marg` | `std::set<int64_t>` | Keyframe timestamps being marginalised (typically one) |
| `use_imu` | `bool` | True if IMU constraints are present in `abs_H` |
| `opt_flow_res` | `std::vector<OpticalFlowResult::Ptr>` | Optical flow results carrying raw images and observations |

---

**6. `RelPoseFactor`**
- **Header:** `include/basalt/utils/imu_types.h:344`
- **Purpose:** Stores a single NFR-recovered relative pose constraint between two keyframes.

| Field | Type | Description |
|---|---|
| `t_i_ns` | `int64_t` | Timestamp of the reference (marginalised) keyframe |
| `t_j_ns` | `int64_t` | Timestamp of the target keyframe |
| `T_i_j` | `Sophus::SE3d` | Measured relative transformation from $j$ to $i$ |
| `cov_inv` | `Sophus::Matrix6d` | $6 \times 6$ inverse covariance (information matrix) of the factor |

---

**7. `RollPitchFactor`**
- **Header:** `include/basalt/utils/imu_types.h:353`
- **Purpose:** Stores a single NFR-recovered 2-DOF gravity-alignment constraint on a keyframe.

| Field | Type | Description |
|---|---|
| `t_ns` | `int64_t` | Timestamp of the constrained keyframe |
| `R_w_i_meas` | `Sophus::SO3d` | Measured orientation at the moment of marginalisation |
| `cov_inv` | `Eigen::Matrix2d` | $2 \times 2$ inverse covariance of the roll-pitch residual |

---

**8. `AbsOrderMap`**
- **Header:** `include/basalt/utils/imu_types.h`
- **Purpose:** Defines the block layout of the global information matrix. Maps each state timestamp to its `(start_index, block_size)` in the Hessian.

| Field | Type | Description |
|---|---|
| `abs_order_map` | `std::map<int64_t, std::pair<int,int>>` | Timestamp → (column offset, block size) |
| `total_size` | `size_t` | Total number of scalar state dimensions |
| `items` | `size_t` | Number of state blocks |

---

**9. `MargHelper<Scalar>`**
- **Header:** `include/basalt/vi_estimator/marg_helper.h`
- **Source:** `src/vi_estimator/marg_helper.cpp`
- **Purpose:** Implements the Schur complement block operations used for both VIO marginalisation and the mapper's IMU-state reduction.
- **Key Static Methods:**

| Method | Description |
|---|---|
| `marginalizeHelperSqToSq(H, b, keep, marg, H_new, b_new)` | Standard Gaussian elimination: $\mathbf{H}^* = \mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{H}_{\beta\alpha}$ |
| `marginalizeHelperSqToSqrt(...)` | Gaussian elimination returning the square-root (Cholesky) form |
| `marginalizeHelperSqrtToSqrt(...)` | Numerically stable Householder/QR-based elimination on Jacobians |

---

**10. `LandmarkDatabase<Scalar>`**
- **Header:** `include/basalt/vi_estimator/landmark_database.h`
- **Purpose:** Stores the 3D map. Maps landmark IDs to `Keypoint<Scalar>` (host frame, stereographic direction, inverse depth) and tracks all `KeypointObservation<Scalar>` (timestamp-camera-pixel) for each landmark.
- **Key Methods:** `addLandmark`, `addObservation`, `getObservations`, `removeKeyframes`.

---

**11. `HashBow<256>`**
- **Header:** `include/basalt/hash_bow/hash_bow.h`
- **Purpose:** A hash-based Bag-of-Words image retrieval database. Computes 256-bit hash codes for ORB descriptors and supports fast nearest-neighbour retrieval for place recognition.
- **Key Methods:** `compute_bow`, `add_to_database`, `querry_database`.

---

**12. `VioEstimatorBase<Scalar>`**
- **Header:** `include/basalt/vi_estimator/vio_estimator.h`
- **Purpose:** Abstract base class of the VIO backend. The mapper's upstream data source.
- **Relevant Queue:** `tbb::concurrent_bounded_queue<MargData::Ptr>* out_marg_queue` — the mapper subscribes to this queue to receive `MargData` packets as keyframes are marginalised.

---

**13. `MargDataLoader`**
- **Header:** `include/basalt/io/marg_data_io.h`
- **Purpose:** Offline utility used in `src/mapper.cpp` to load serialised `MargData` packets from disk into a `tbb::concurrent_bounded_queue<MargData::Ptr>`. Not used during real-time operation.

---

**14. NFR Residual Functions (`include/basalt/utils/nfr.h`)**

| Function | Signature | Description |
|---|---|---|
| `relPoseError` | `(T_i_j, T_w_i, T_w_j, *Ji, *Jj) → Vector6d` | SE(3) relative pose residual and Jacobians |
| `rollPitchError` | `(T_w_i, R_w_i_meas, *J) → Vector2d` | 2-DOF roll-pitch residual and Jacobian |
| `absPositionError` | `(T_w_i, pos, *J) → Vector3d` | Absolute position residual (used internally for covariance propagation) |
| `yawError` | `(T_w_i, yaw_dir_body, *J) → double` | Yaw residual (used internally for covariance propagation) |

---

### 4b. Execution Flow & Pipeline

The full mapping pipeline proceeds in seven sequential stages. In the current offline implementation these are driven by `src/mapper.cpp`; the equivalent calls for real-time integration are noted where relevant.

---

**Stage 1: Data Ingestion**

`src/mapper.cpp:182–184`
```cpp
for (auto& kv : marg_data) {
    nrf_mapper->addMargData(kv.second);
}
```

`NfrMapper::addMargData` (`nfr_mapper.cpp:60`) is the entry point for each `MargData` packet:

```cpp
void NfrMapper::addMargData(MargData::Ptr& data) {
    processMargData(*data);
    bool valid = extractNonlinearFactors(*data);

    if (valid) {
        // register pose-only keyframes into frame_poses
        for (const auto& kv : data->frame_poses) { ... }
        for (const auto& kv : data->frame_states) {
            if (data->kfs_all.count(kv.first) > 0) { ... }
        }
    }
}
```

Only keyframes present in `kfs_all` are registered in `frame_poses`; non-keyframe navigation states are discarded after the IMU-state reduction.

---

**Stage 2: IMU State Reduction** (`processMargData`, `nfr_mapper.cpp:82`)

For each entry in `m.aom`:
- **Pure pose states** (`POSE_SIZE = 6`): all 6 columns added to `idx_to_keep`.
- **Full navigation states** (`POSE_VEL_BIAS_SIZE = 15`) that are keyframes: the 6 pose columns added to `idx_to_keep`, the 9 velocity/bias columns added to `idx_to_marg`. A `PoseStateWithLin` is constructed and moved to `m.frame_poses`.
- **Full navigation states** that are not keyframes: all 15 columns added to `idx_to_marg` and the state is erased.

If `idx_to_marg` is non-empty, `MargHelper::marginalizeHelperSqToSq` applies the Schur complement to yield a pose-only `marg_H_new` and `marg_b_new`. These replace `m.abs_H` and `m.abs_b`. Image data from `m.opt_flow_res` is saved into `img_data` keyed by timestamp.

---

**Stage 3: Non-Linear Factor Extraction** (`extractNonlinearFactors`, `nfr_mapper.cpp:151`)

1. Rank-check `m.abs_H`; return `false` if rank-deficient.
2. Invert to obtain full marginal covariance `cov_old`.
3. Identify the marginalised keyframe `kf_id = *m.kfs_to_marg.cbegin()` and its pose `T_w_i_kf`.
4. Compute Jacobians for absolute position, yaw, and roll-pitch at `T_w_i_kf`; assemble full $6 \times n$ Jacobian; propagate `cov_new = J * cov_old * J^T`; extract $2 \times 2$ roll-pitch block; store as `RollPitchFactor` in `roll_pitch_factors` (only if `use_imu`).
5. For each other keyframe `other_id` in `m.kfs_all`: compute relative pose `T_kf_o`; compute Jacobians `d_res_d_T_w_i` and `d_res_d_T_w_j`; assemble $6 \times n$ Jacobian; propagate `cov_new = J * cov_old * J^T`; invert via LDLT; store as `RelPoseFactor` in `rel_pose_factors`.

---

**Stage 4: Feature Detection** (`detect_keypoints`, `nfr_mapper.cpp:455`)

Collects all timestamps present in both `img_data` and `frame_poses`, then dispatches a TBB parallel-for over them. Each thread processes one `TimeCamId`: detects keypoints, computes descriptors, unprojects corners, computes BoW vector, and inserts into `hash_bow_database`. Results are stored in `feature_corners`.

---

**Stage 5: Feature Matching** (`match_stereo` + `match_all`, `nfr_mapper.cpp:513,555`)

`match_stereo`: Iterates over all timestamps, matches left and right camera keypoints using the stereo essential matrix. Stereo matches with $\geq 16$ inliers are stored in `feature_matches[{tcid_left, tcid_right}]`.

`match_all`:
1. Build index from `TimeCamId` to position in a flat `keys` vector.
2. Parallel BoW query: each frame queries `hash_bow_database` for `mapper_num_frames_to_match` similar frames; candidate pairs exceeding the score threshold are pushed into a concurrent `ids_to_match` vector.
3. Parallel geometric verification: for each candidate pair, run descriptor matching and `findInliersRansac`; pairs with inliers are stored in `feature_matches`.

---

**Stage 6: Track Building and Triangulation** (`build_tracks` + `setup_opt`, `nfr_mapper.cpp:671,697`)

`build_tracks`: Runs `TrackBuilder::Build`, `Filter`, and `Export` to produce `feature_tracks`.

`setup_opt`: Iterates over `feature_tracks`. For each track with $\geq 2$ observations, takes the first as host. Iterates over remaining observations; the first one with sufficient baseline is used to triangulate. A valid landmark (finite, positive, inverse-depth $\leq 2$) is added to `lmdb` with all track observations registered as `KeypointObservation`.

---

**Stage 7: Global Optimisation and Filtering** (`optimize` + `filterOutliers`, `nfr_mapper.cpp:244`)

1. Build `AbsOrderMap aom` over all entries in `frame_poses`, assigning consecutive 6-column blocks.
2. For each iteration:
   a. `linearizeHelper` produces `rld_vec` — the relative linearisation data for the visual cost.
   b. A `MapperLinearizeAbsReduce<SparseHashAccumulator<double>> lopt(aom, &frame_poses)` is constructed and reduced in parallel over `rld_vec` (vision), `roll_pitch_factors`, and `rel_pose_factors`.
   c. `lopt.accum.setup_solver()` factorises the sparse Hessian.
   d. LM or GN solve yields the pose increment vector `inc`.
   e. Poses updated via `kv.second.applyInc(-inc.segment<POSE_SIZE>(idx))`.
   f. Landmarks updated via parallel `updatePoints`.
   g. If LM: evaluate new error; accept or reject step; adjust `lambda`.
   h. Break on convergence (`max_inc < 1e-5`).
3. After first optimisation pass: `filterOutliers(outlier_threshold, 4)` removes landmarks with excessive reprojection error.
4. A second `optimize` pass refines the cleaned map.

---

## 5. Conclusion

The `NfrMapper` pipeline implements a complete global visual-inertial mapping system. Its central contribution is the use of Non-Linear Factor Recovery to bridge the VIO sliding-window estimator and the global Bundle Adjustment layer: rather than discarding the rich information accumulated during marginalisation, the mapper converts it into a sparse set of relative-pose and roll-pitch factors that faithfully summarise the high-frequency VIO constraints at a fraction of the computational cost. Combined with a fresh round of cross-frame feature matching and triangulation, the resulting global optimisation can refine keyframe poses and 3D structure over arbitrarily long trajectories.

### Current Limitation: Offline Execution

As implemented, the pipeline is entirely **offline**. The sample driver `src/mapper.cpp` reads pre-serialised `MargData` from disk, feeds all packets to the mapper at once, then executes detection, matching, and optimisation as a batch. There is no real-time thread, no incremental matching, and no live output. Integration into `src/controller.cpp` (`basalt::Controller`) is required before the mapper can operate as part of a live SLAM system.

### Path to Real-Time Integration

The following changes are required to upgrade the mapper for real-time use:

1. **Dedicated mapper thread in `Controller`**: Spawn a `std::thread` (or a managed lifecycle component in the ROS 2 context) that continuously pops from `VioEstimatorBase::out_marg_queue`. For each received `MargData::Ptr`, call `nrf_mapper->addMargData(data)` to incrementally accumulate NFR factors and keyframe poses.

2. **Incremental feature detection and matching**: Replace the current batch `detect_keypoints` / `match_all` calls with an incremental strategy. Newly arriving keyframes should be detected immediately. Matching should be performed against a recent temporal window and against a fixed-size candidate set retrieved via BoW, rather than exhaustively against all historical frames.

3. **Periodic or triggered global optimisation**: Rather than a single post-hoc `optimize` call, the mapper thread should trigger global BA at a configurable rate (e.g., after every $N$ new keyframes) or on detection of a loop-closure candidate. The current `optimize` implementation is compatible with this pattern as it operates on the accumulated `frame_poses` and `lmdb`.

4. **Output publication**: After each optimisation cycle, the refined keyframe poses should be published back to the Controller for use by downstream consumers (e.g., dense reconstruction, localisation).

5. **Loop closure integration**: The `HashBow` database already supports global place recognition. A loop-closure detector can identify candidate frame pairs from `match_all`, verify them geometrically, and inject additional `RelPoseFactor`-like constraints into the global graph to correct long-term drift.

---

## References

1. Usenko, V., Demmel, N., Schubert, D., Stückler, J., & Cremers, D. (2020). *Visual-Inertial Mapping with Non-Linear Factor Recovery*. arXiv preprint arXiv:1904.06504v3.
2. Mazuran, M., Burgard, W., & Tipaldi, G. D. (2015). *Nonlinear Factor Recovery for Long-Term SLAM*. The International Journal of Robotics Research (IJRR).
3. `basalt` Mapping Implementation: `include/basalt/vi_estimator/nfr_mapper.h` and `src/vi_estimator/nfr_mapper.cpp`.
4. `basalt` Mapper Sample Driver: `src/mapper.cpp`.
5. `basalt` NFR Utilities: `include/basalt/utils/nfr.h`.
6. `basalt` Marginalisation Documentation: `doc/Marginalisation.md`.
