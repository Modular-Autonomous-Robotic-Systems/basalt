# Visual-Inertial Mapping with Non-Linear Factor Recovery

## 1. Introduction

Visual-Inertial Odometry (VIO) thread provides real-time, locally consistent pose estimates by operating on a bounded sliding window of recent frames and keyframes. While computationally tractable, this approach fundamentally cannot correct long-term drift: once a keyframe leaves the active window it is marginalised and its pose is never revisited. A global mapping layer is therefore required to build a globally consistent 3D map over the complete trajectory of the camera setup.

The **mapping pipeline** in `basalt`  performs this role of globally consistent mapping. The output of the VIO thread — `MargData` packets emitted when keyframes are marginalised, are consumed by the mapping pipline to build a globally refined map of keyframe poses and 3D landmarks. The core insight that makes this feasible is **Non-Linear Factor Recovery (NFR)**: rather than discarding the dense information accumulated by VIO marginalisation, the mapper recovers a small set of sparse, non-linear factors that faithfully summarise the VIO's high-frequency visual-inertial constraints. These recovered factors, together with freshly detected visual features, drive a global Bundle Adjustment (BA) that can correct drift over the entire trajectory.

The mapping pipeline is tightly coupled to the VIO marginalisation output. The dense information matrix passed through `MargData` is the direct product of the Schur-complement operations described in (`doc/Marginalisation.md`)[doc/Marginalisation.md]. The quality of the recovered non-linear factors — and therefore the global map — is contingent on the VIO sliding-window optimisation having converged to a good local minimum before marginalisation. Readers unfamiliar with the VIO linearisation and marginalisation process are advised to read (`doc/Marginalisation.md`)[doc/Marginalisation.md] before proceeding.
Each `MargData` packet fed to the mapping pipeline contains:
1. The dense marginal information matrix `abs_H` and vector `abs_b` over the Markov blanket of the marginalised keyframe.
2. Keyframe pose estimates (`frame_poses`, `frame_states`).
3. Keyframe bookkeeping sets (`kfs_all`, `kfs_to_marg`).
4. The optical flow results (`opt_flow_res`) carrying the raw image data for re-detection.
4. A flag `use_imu` indicating whether IMU constraints are present in the marginal.

The mapping pipeline will process the input sequence of `MargData::Ptr` to output:
1. A globally refined map of keyframe poses: `Eigen::aligned_map<int64_t, PoseStateWithLin<double>> frame_poses`.
2. A 3D landmark database: `LandmarkDatabase<double> lmdb` (inherited from `BundleAdjustmentBase`).

The mapping pipeline is implemented in the `NfrMapper` class (`include/basalt/vi_estimator/nfr_mapper.h` and `src/vi_estimator/nfr_mapper.cpp`). It inherits the full Bundle Adjustment implementation from `ScBundleAdjustmentBase<double>`, which in turn inherits from `BundleAdjustmentBase<double>`. In the current implementation the pipeline is **offline**: a sample driver (`src/mapper.cpp`) loads serialised `MargData` from disk using `MargDataLoader`, feeds all packets to the mapper, then executes the detection, matching, optimisation, and filtering stages sequentially.
Section 2 in the document derives the mathematics of Non-Linear Factor Recovery: how a dense marginal covariance is converted into sparse relative-pose and roll-pitch factors. Section 3 formulates the global Bundle Adjustment problem and describes the feature pipeline (detection, matching, triangulation) that supplements the recovered factors. Section 4 documents all classes, data structures, and the end-to-end execution flow in code. Section 5 concludes with a discussion of the current limitations and the path to real-time integration.

---

## 2. Non-Linear Factor Recovery

### 2.1 Motivation

As detailed in `doc/Marginalisation.md §4`, marginalising a keyframe produces a dense marginal information matrix $\mathbf{H}^{\ast}$ over its Markov blanket — the set of all remaining active keyframes that shared visual landmarks with the marginalised frame. This dense prior captures the full joint uncertainty of those keyframes as inferred from all visual-inertial measurements processed up to that point.

If one were to use $\mathbf{H}^{\ast}$ directly in a global factor graph, two problems arise:
1. Computational intractability. Each marginalised keyframe introduces a dense block coupling every frame in its Markov blanket to every other. As the trajectory grows, the global information matrix becomes increasingly dense, destroying the sparsity that makes graph optimisation efficient.
2. Linearisation point fixation. The prior $\mathbf{H}^\ast$ is computed at a fixed linearisation point. Reusing it across large pose changes degrades accuracy.

Non-Linear Factor Recovery (NFR), introduced in Mazuran et al. [2] and applied to visual-inertial mapping in Usenko et al. [1], resolves both issues. The dense Gaussian distribution $p(\mathbf{x}) \sim \mathcal{N}(\boldsymbol{\mu}, (\mathbf{H}^{\ast})^{-1})$ is approximated by a sparse product of non-linear factors $q(\mathbf{x})$. The optimal approximation is found by minimising the Kullback–Leibler divergence:

$$D_{\text{KL}}(p | q) = \int p(\mathbf{x}) \log (\frac{p(\mathbf{x})}{q(\mathbf{x})}) d\mathbf{x}$$

The recovered factors are standard non-linear factors and can be linearised freshly at each global optimisation iteration, eliminating the fixed-point issue. Their sparse structure preserves graph sparsity.

In the `basalt` implementation, two types of factors are recovered from each `MargData` packet:
1. `RelPoseFactor`: A 6-DOF relative pose constraint between the marginalised keyframe and each other keyframe in its Markov blanket.
2. `RollPitchFactor`: A 2-DOF absolute orientation constraint on the marginalised keyframe's roll and pitch, derived from the IMU gravity alignment.

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

(`nfr_mapper.cpp:130–131`) to get the marginalised information matrix and information vector. The resulting `marg_H_new` is a pure $6K \times 6K$ pose-only information matrix (where $K$ is the number of keyframes), ready for the covariance inversion in `NfrMapper::extractNonlinearFactors`.

---

## 3. Mapping

### 3.1 Problem Formulation

The global mapping problem is cast as a non-linear least squares optimisation over all keyframe poses $\{ \mathbf{T}_{w,i} \in SE(3) \}_{i=1}^{K}$ and all 3D landmark parameters $\{ \mathbf{l}_j \}_{j=1}^{L}$. The total objective function to minimise is:

$$E(\mathbf{s}) = E_{\text{vision}}(\mathbf{s}) + E_{\text{rel}}(\mathbf{s}) + E_{\text{rp}}(\mathbf{s})$$

where $\mathbf{s} = \{ \mathbf{T}_{w,i}, \mathbf{l}_j \}$ is the full state vector.

#### 3.1.1 Visual Reprojection Error

For a 3D landmark $j$ hosted in frame $h(j)$ and observed in target frame $i$ at image coordinates $\mathbf{z}_{ij} \in \mathbb{R}^2$, the reprojection residual is:

$$\mathbf{r}_{ij}^{\text{vis}} = \mathbf{z}_{ij} - \pi\!\left(\mathbf{T}_{w,i}^{-1} \, \mathbf{T}_{w,h(j)} \, \mathbf{q}_j\right)$$

where $\pi(\cdot)$ is the camera projection model and $\mathbf{q}_j$ is the 3D point reconstructed from the landmark parameters. The visual cost term is:

$$E_{\text{vision}} = \sum_{(i,j)} \rho\!\left(\left\|\mathbf{r}_{ij}^{\text{vis}}\right\|_{\boldsymbol{\Sigma}_{ij}^{-1}}^2\right)$$

where $\rho(\cdot)$ is the Huber loss function (threshold `mapper_obs_huber_thresh`) and $\boldsymbol{\Sigma}_{ij} = \sigma_{\text{obs}}^2 \mathbf{I}_2$ with `mapper_obs_std_dev`.

#### 3.1.2 Relative Pose Error

$$E_{\text{rel}} = \sum_{(i,j)} \mathbf{r}_{ij}^T \, \boldsymbol{\Omega}_{ij} \, \mathbf{r}_{ij}$$

where $\mathbf{r}_{ij} \in \mathbb{R}^6$ is the SE(3) relative pose residual defined in Section 2.3.1, and $\boldsymbol{\Omega}_{ij}$ is the recovered information matrix from Section 2.3.3.

#### 3.1.3 Roll-Pitch Error

$$E_{\text{rp}} = \sum_{k \in \mathcal{P}} \mathbf{r}_k^T \, \boldsymbol{\Omega}_k \, \mathbf{r}_k$$

where $\mathbf{r}_k \in \mathbb{R}^2$ is the roll-pitch residual defined in Section 2.4.2, and $\boldsymbol{\Omega}_k$ is the recovered $2 \times 2$ information matrix from Section 2.4.3.

### 3.2 Feature Detection

Fresh keypoint detection is performed on every keyframe image stored in `NfrMapper::img_data`, populated from `opt_flow_res` during `processMargData`. The mapper does not reuse the corners that the optical-flow front end already tracked, because those corners were selected to be trackable across a short baseline and carry no descriptor, whereas the mapper needs a representation that survives an arbitrary gap in time and viewpoint. Detection is driven by `NfrMapper::detect_keypoints` (`nfr_mapper.cpp:508-539`), which walks `img_data` serially and calls `NfrMapper::detect_timestamp_keypoints` (`nfr_mapper.cpp:477-506`) once per timestamp. The per-frame routine performs six steps for each camera in the rig.

1. **Keypoint detection**: `detectKeypointsMapping` extracts up to `mapper_detection_num_points` Shi-Tomasi corners per image.
2. **Angle computation**: `computeAngles` estimates the dominant orientation of each corner from its intensity centroid.
3. **Descriptor computation**: `computeDescriptors` computes a 256 bit steered BRIEF descriptor per corner.
4. **Unprojection**: Each 2D corner is unprojected to a unit-sphere ray using the calibrated camera intrinsics (`calib.intrinsics[cam_id].unproject`).
5. **BoW encoding**: `hash_bow_database->compute_bow` converts the descriptor set into a Bag-of-Words vector.
6. **Database insertion**: `hash_bow_database->add_to_database(tcid, kd.bow_vector)` registers the frame for subsequent retrieval.

Detected keypoints are stored in `NfrMapper::feature_corners` keyed by `TimeCamId`, a `(frame_id, cam_id)` pair.

For the offline mapper detection runs in parallel over all frames using TBB as can be seen in `src/vi_estimator/nfr_mapper.cpp:465-499`. However, the `tbb::parallel_for` was removed because two threads updating `HashBowStl::inverted_index` concurrently corrupted it, and the loop at `src/vi_estimator/nfr_mapper.cpp:518-523` now carries a comment recording that. The same section also described step 1 as extracting FAST or Harris corners. Neither is correct for this path. `detectKeypointsMapping` calls `goodFeaturesToTrack` with `useHarrisDetector` left at its default of `false`, so the response is the Shi-Tomasi minimum eigenvalue. FAST is used by the sibling routine `detectKeypoints` (`keypoints.cpp:161-250`), which serves the cell-based visual odometry front end and is never called from the mapping path.

The three algorithmic steps are treated in turn below. Section 3.2.1 covers corner selection, Section 3.2.2 the orientation estimate that makes the descriptor rotation invariant, and Section 3.2.3 the descriptor itself. Section 3.2.4 completes the record with unprojection and Bag-of-Words encoding, Section 3.2.5 draws the three together into an argument for why this particular combination is the right one for the mapping thread, and Section 3.2.6 surveys the improvements the literature offers. A complementary treatment of the same pipeline with a stronger emphasis on the matching stages that consume it is given in [`doc/MappingFeatureExtractionMatching.md`](MappingFeatureExtractionMatching.md).

#### 3.2.1 Corner Detection — the Shi-Tomasi Criterion

**History.** The search for image locations that can be relocated reliably in a second view begins with Moravec's interest operator of 1980, which scored a patch by the minimum sum of squared differences over four discrete shift directions. Harris and Stephens replaced the discrete shifts with a first-order Taylor expansion in 1988, turning the score into an algebraic function of a 2 by 2 matrix and making the response isotropic rather than quantised to four directions. Shi and Tomasi refined the response function itself in 1994, and it is their criterion, not Harris's, that `basalt` uses. Rosten and Drummond's FAST detector of 2006 took the opposite path, abandoning the gradient formulation entirely in favour of a learned decision tree over a Bresenham circle of sixteen pixels, trading the well-conditioned structure tensor for roughly an order of magnitude in speed.

**Mathematical background.** Consider a weighted patch centred at $(x,y)$ and the sum of squared intensity differences induced by displacing it by $\boldsymbol{\Delta} = (\Delta x, \Delta y)^T$,

$$E(\boldsymbol{\Delta}) = \sum_{(u,v) \in \mathcal{W}} w(u,v) \left[ I(u + \Delta x,\, v + \Delta y) - I(u,v) \right]^2 .$$

A first-order Taylor expansion of the shifted intensity, $I(u + \Delta x, v + \Delta y) \approx I(u,v) + \nabla I^T \boldsymbol{\Delta}$, reduces this to a quadratic form,

$$E(\boldsymbol{\Delta}) \approx \boldsymbol{\Delta}^T \mathbf{M} \boldsymbol{\Delta}, \qquad \mathbf{M}(x,y) = \sum_{(u,v) \in \mathcal{W}} w(u,v) \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix},$$

where $\mathbf{M}$ is the second-moment matrix, also called the structure tensor. It is symmetric and positive semi-definite, so it admits the eigendecomposition $\mathbf{M} = \mathbf{R}\,\mathrm{diag}(\lambda_1, \lambda_2)\,\mathbf{R}^T$ with $\lambda_1 \geq \lambda_2 \geq 0$. The level sets of $E$ are ellipses whose semi-axes are $\lambda_1^{-1/2}$ and $\lambda_2^{-1/2}$, so over all unit displacement directions the smallest rise in $E$ is exactly $\lambda_2$. The Shi-Tomasi response is therefore

$$R = \min(\lambda_1, \lambda_2) = \tfrac{1}{2}\left[ \left(\textstyle\sum I_x^2 + \sum I_y^2\right) - \sqrt{\left(\textstyle\sum I_x^2 - \sum I_y^2\right)^2 + 4\left(\textstyle\sum I_x I_y\right)^2} \right],$$

which is the worst-case sensitivity of the patch to displacement in any direction. A large $R$ certifies that the patch changes appreciably however it is moved, which is precisely the property that makes it relocatable.

The significance of this choice runs deeper than a scalar summary of two eigenvalues. The same matrix $\mathbf{M}$ is the normal-equation matrix of the Lucas-Kanade differential tracker, whose update solves $\mathbf{M}\boldsymbol{\Delta} = \mathbf{b}$. Requiring $\lambda_{\min}$ to be large is therefore identical to requiring that the tracker's own linear system be well conditioned, which is why Shi and Tomasi titled their paper "Good Features to Track" and why OpenCV names the function `goodFeaturesToTrack`. The criterion does not merely describe a corner, it selects the features on which the downstream estimator is numerically stable.

Harris and Stephens instead used $R_H = \det \mathbf{M} - k\,(\mathrm{tr}\,\mathbf{M})^2 = \lambda_1 \lambda_2 - k(\lambda_1 + \lambda_2)^2$, which avoids the square root but introduces a free sensitivity parameter $k$ with no principled value, conventionally fixed between 0.04 and 0.06. Shi-Tomasi has no such parameter. Given that the square root above costs a single instruction on a 2 by 2 system, the Harris economy is no longer worth its tuning burden, and this is the standard modern justification for preferring the minimum eigenvalue.

**Implementation.** `detectKeypointsMapping` (`keypoints.cpp:136-159`) first converts the 16 bit image to 8 bit by a right shift of eight places,

```cpp
uint8_t* dst = image.ptr();
const uint16_t* src = img_raw.ptr;
for (size_t i = 0; i < img_raw.size(); i++) {
  dst[i] = (src[i] >> 8);
}

std::vector<cv::Point2f> points;
goodFeaturesToTrack(image, points, num_features, 0.01, 8);
```

The five-argument call leaves `blockSize` at its OpenCV default of 3, `useHarrisDetector` at `false` and the mask empty, so $\mathcal{W}$ is a 3 by 3 box window with uniform weights and the response is the minimum eigenvalue. OpenCV computes $R$ at every pixel, then applies three filters in order. A corner is kept only if $R > q \cdot \max_{(x,y)} R$ with the quality level $q = 0.01$, surviving corners are visited in descending $R$ and greedily suppressed if they lie within `minDistance` $= 8$ pixels of an already accepted corner, and the list is finally truncated to `maxCorners`, which `basalt` sets to `config.mapper_detection_num_points`. That parameter defaults to 800 (`vio_config.cpp:94`) and is raised to 1800 in `data/sitl_config_vo.json:40`.

Each surviving corner is admitted only if it clears an image border of `EDGE_THRESHOLD` $= 19$ pixels,

```cpp
if (img_raw.InBounds(points[i].x, points[i].y, EDGE_THRESHOLD)) {
  kd.corners.emplace_back(points[i].x, points[i].y);
}
```

That constant is not arbitrary. The BRIEF sampling offsets of Section 3.2.3 span the range $[-13, 12]$ in both axes, so the largest distance from the patch centre to a sample is $\sqrt{13^2 + 13^2} = 18.38$ pixels. Rotation preserves that norm and rounding to the nearest integer can add at most half a pixel per axis, so a steered sample never falls further than 19 pixels from the corner. `Image::InBounds(x, y, border)` requires `border <= x < w - border - 1` (`basalt-headers/include/basalt/image/image.h:723-727`), so a border of 19 is exactly sufficient and not one pixel more. The orientation patch of Section 3.2.2 needs only 15 pixels and is therefore covered a fortiori.

**Properties and limitations.** Shi-Tomasi corners are invariant to rotation of the image plane, because $\mathbf{M}$ transforms by conjugation under a rotation and its eigenvalues are unchanged, and they are equivariant under an affine intensity change $I \mapsto aI + b$ only up to the factor $a^2$, which the relative quality threshold partly absorbs. Three limitations bear directly on the mapping thread.

The first is the absence of scale invariance. `detectKeypointsMapping` runs on a single full-resolution image with no pyramid, whereas ORB detects over eight octaves with a scale factor of 1.2 and assigns each keypoint an octave. A corner detected at one distance therefore has no guarantee of being redetected when the vehicle has halved or doubled its range to the structure, which bounds the viewpoint change over which a loop can close.

The second is that the acceptance threshold is relative to the single strongest response in the frame. A specular highlight, a lens flare or one very high contrast structure raises $\max R$ and with it the bar for the entire image, so the detected count can collapse on a frame that is otherwise perfectly textured. This is a plausible contributor to the retrieval failure recorded in [`../context/keyframe_driven_local_mapping.md`](../context/keyframe_driven_local_mapping.md), where the 2026-09-10 run returned zero bag-of-words hits on 57 of 66 mapping cycles, and it is the reason the instrumentation planned there must separate "no keypoints at all" from "keypoints fine, retrieval too strict".

The third is that corner positions are never refined below the pixel. The `cv::cornerSubPix` call that would do so is commented out at `keypoints.cpp:239-243`. A position quantised uniformly to the pixel grid carries a standard deviation of $1/\sqrt{12} = 0.289$ pixels, and `mapper_obs_std_dev` is set to 0.25 (`vio_config.cpp:92`). The assumed observation noise in the bundle adjustment of Section 3.1.1 is thus accounted for almost entirely by quantisation, leaving no budget for detector jitter or calibration residual. Sub-pixel refinement would not merely reduce the residuals, it would make the assumed $\boldsymbol{\Sigma}_{ij}$ defensible.

#### 3.2.2 Orientation Assignment — the Intensity Centroid

**History.** Binary intensity-comparison descriptors are not rotation invariant by construction, because the sample pattern is expressed in image axes. Rotation invariance must therefore be supplied by an external orientation estimate attached to each keypoint. SIFT solved this in 2004 with a 36 bin histogram of gradient orientations weighted by gradient magnitude, taking the dominant peak. That is accurate but costs a gradient computation and a histogram per keypoint. Rosin proposed the intensity centroid in 1999 as a far cheaper alternative, and Rublee and colleagues adopted it for ORB in 2011 after measuring it to be both faster and, on their data, more stable than the gradient histogram. `basalt` uses Rosin's construction unchanged.

**Mathematical background.** Define the raw image moments of a patch $\mathcal{D}$ centred on the corner,

$$m_{pq} = \sum_{(x,y) \in \mathcal{D}} x^p y^q\, I(x,y),$$

where $x$ and $y$ are offsets from the corner, not absolute pixel indices. The intensity centroid is the first moment normalised by the zeroth,

$$\mathbf{C} = \left( \frac{m_{10}}{m_{00}},\; \frac{m_{01}}{m_{00}} \right).$$

The orientation is the direction of the vector from the patch centre to the centroid, and since $m_{00} > 0$ scales both components identically it may be dropped,

$$\theta = \mathrm{atan2}(m_{01},\, m_{10}).$$

The construction is meaningful precisely because a corner is, by the definition of Section 3.2.1, a location where intensity is not symmetric about the centre, so the centroid is displaced from it by a margin that grows with the corner's own distinctiveness.

The property that matters is equivariance. Let the patch be rotated about its centre by $\varphi$, so that the observed intensity becomes $I'(\mathbf{x}) = I(\mathbf{R}(-\varphi)\mathbf{x})$. Substituting into the moment sum and changing variables gives

$$\begin{pmatrix} m_{10}' \\ m_{01}' \end{pmatrix} = \mathbf{R}(\varphi) \begin{pmatrix} m_{10} \\ m_{01} \end{pmatrix} \quad \Longrightarrow \quad \theta' = \theta + \varphi .$$

The estimated angle therefore rotates exactly with the image content, which is what allows the descriptor of Section 3.2.3 to cancel the rotation by steering its sampling pattern by $-\theta$ in the patch frame.

Two invariances follow from the geometry of the support. Because $\mathcal{D}$ is centrally symmetric, $\sum_{\mathcal{D}} x = \sum_{\mathcal{D}} y = 0$, so an additive brightness offset $I \mapsto I + b$ leaves $m_{10}$ and $m_{01}$ unchanged. Because a multiplicative gain $I \mapsto aI$ scales both components equally, it leaves $\theta$ unchanged. The angle is thus invariant to any affine photometric change $I \mapsto aI + b$ with $a > 0$. The support must nonetheless be a disc rather than the square patch, since a square support admits different pixels as the content rotates and would break the equivariance derived above.

**Implementation.** `computeAngles` (`keypoints.cpp:252-281`) accumulates the two moments over the disc of radius `HALF_PATCH_SIZE` $= 15$, giving the 31 by 31 patch from which the descriptor pattern tables take their name.

```cpp
if (rotate_features) {
  double m01 = 0, m10 = 0;
  for (int x = -HALF_PATCH_SIZE; x <= HALF_PATCH_SIZE; x++) {
    for (int y = -HALF_PATCH_SIZE; y <= HALF_PATCH_SIZE; y++) {
      if (x * x + y * y <= HALF_PATCH_SIZE * HALF_PATCH_SIZE) {
        double val = img_raw(cx + x, cy + y);
        m01 += y * val;
        m10 += x * val;
      }
    }
  }
  angle = atan2(m01, m10);
}
```

The mapper passes `rotate_features = true` at `nfr_mapper.cpp:488`, so the branch is always taken on this path. Note that the summation reads `img_raw`, the full 16 bit image, whereas detection in Section 3.2.1 worked on the 8 bit reduction. The angle therefore carries more photometric precision than the corner position it is attached to.

**Properties and limitations.** The estimate costs one multiply-accumulate pair per pixel over roughly $\pi \cdot 15^2 \approx 707$ pixels and needs no gradient, no smoothing and no histogram, which is the whole reason it was adopted. Its weakness is that it is a single first-order statistic and offers no confidence measure. When the disc is close to radially symmetric in intensity the moment vector shrinks towards zero and the angle becomes dominated by noise, and in the exact degenerate case IEEE 754 defines $\mathrm{atan2}(0,0) = 0$, so a wholly undetermined orientation is silently reported as zero radians rather than rejected. `basalt` applies no gate on $\|(m_{10}, m_{01})\|$, so such keypoints enter the descriptor stage with an arbitrary steering angle and contribute descriptors that will not match their own counterparts in an adjacent view. A magnitude gate is among the improvements of Section 3.2.6.

#### 3.2.3 Descriptor Computation — Steered BRIEF

**History.** The descriptor problem is to summarise the neighbourhood of a keypoint in a form that is stable across viewpoint and illumination yet cheap to compare. SIFT answered it in 2004 with a 128 dimensional histogram of oriented gradients, and SURF in 2006 accelerated the same idea using integral images and Haar responses. Both produce floating-point vectors, 512 bytes for SIFT, compared under the Euclidean metric. Calonder and colleagues broke from that lineage in 2010 with BRIEF, which forms the descriptor directly as a string of binary intensity comparisons, reducing storage to 32 bytes and the metric to a Hamming distance. BRIEF as published was not rotation invariant. ORB supplied the missing invariance in 2011 by steering the sampling pattern with the intensity-centroid angle of Section 3.2.2 and, critically, by relearning the pattern so that the steered bits retained the statistical properties that make a binary code discriminative. BRISK and FREAK followed in 2011 and 2012 with hand-designed concentric sampling patterns and explicit scale handling, and the learned binary descriptors, of which BEBLID in 2020 is the most directly substitutable, later recovered accuracy close to SIFT at a cost below ORB.

**Mathematical background.** For a smoothed or raw patch $\mathbf{p}$ and an ordered pair of sample locations $(\mathbf{a}_i, \mathbf{b}_i)$, the elementary binary test is

$$\tau(\mathbf{p}; \mathbf{a}_i, \mathbf{b}_i) = \begin{cases} 1 & \text{if } I(\mathbf{a}_i) < I(\mathbf{b}_i) \\ 0 & \text{otherwise.} \end{cases}$$

The descriptor is the concatenation of $n_d = 256$ such tests, $\mathbf{d} = \left( \tau_1, \tau_2, \dots, \tau_{256} \right) \in \{0,1\}^{256}$, and similarity between two descriptors is the Hamming distance

$$d_H\!\left(\mathbf{d}^{(1)}, \mathbf{d}^{(2)}\right) = \mathrm{popcount}\!\left( \mathbf{d}^{(1)} \oplus \mathbf{d}^{(2)} \right),$$

which four XOR operations and four `POPCNT` instructions evaluate on a 64 bit machine. Because each bit is the sign of a difference, the descriptor is invariant to any monotonically increasing photometric transform, which is a stronger guarantee than the affine invariance that normalised gradient histograms provide.

Rotation invariance is obtained by steering. Collect the sample locations into $\mathbf{S} = \begin{bmatrix} \mathbf{a}_1 & \cdots & \mathbf{a}_{256} \\ \mathbf{b}_1 & \cdots & \mathbf{b}_{256} \end{bmatrix}$ and rotate them by the keypoint angle,

$$\mathbf{S}_\theta = \mathbf{R}(\theta)\, \mathbf{S}, \qquad \tilde{\mathbf{a}}_i = \left\lfloor \mathbf{R}(\theta)\, \mathbf{a}_i \right\rceil, \quad \tilde{\mathbf{b}}_i = \left\lfloor \mathbf{R}(\theta)\, \mathbf{b}_i \right\rceil,$$

with $\lfloor \cdot \rceil$ denoting rounding to the nearest integer pixel. By the equivariance established in Section 3.2.2, a rotation $\varphi$ of the scene rotates $\theta$ by $\varphi$ and hence rotates $\mathbf{S}_\theta$ by $\varphi$, so the same physical pixel pairs are compared and the descriptor is unchanged up to resampling error.

The choice of the 256 pairs is the subtle part, and it is what separates ORB's rBRIEF from plain BRIEF. A binary bit carries its maximum entropy of one bit when its mean over the data is 0.5, and a code of $n_d$ bits attains its maximum joint entropy of $n_d$ bits only when the bits are mutually uncorrelated. Calonder's isotropic Gaussian sampling gives bits with mean near 0.5 and high variance in the unsteered case, but Rublee and colleagues measured that steering destroys both properties, because the rotated tests become concentrated along the dominant gradient direction that defines $\theta$ and therefore correlated with one another. Their remedy was to learn the pattern. From roughly $205{,}000$ candidate tests evaluated on a training set of some $300{,}000$ keypoint patches, tests were ranked by $\left| \bar{\tau}_i - 0.5 \right|$ and then greedily accepted in that order, each candidate admitted only if its absolute correlation with every already-accepted test fell below a threshold, until 256 had been chosen. The resulting table is what ships in OpenCV as `bit_pattern_31_`, and the effect is that the 256 bit code approaches 256 bits of usable discriminability rather than the far smaller effective dimension a correlated code would deliver. This is what makes the Hamming distance a well-calibrated metric and, downstream, what makes the second-best ratio test of Section 3.3 meaningful.

**Implementation.** The pattern is embedded directly in `keypoints.cpp:58-134` as four `char[256]` tables, `pattern_31_x_a`, `pattern_31_y_a`, `pattern_31_x_b` and `pattern_31_y_b`. Their entries are exactly OpenCV's learned ORB table de-interleaved into four arrays. The first four quadruples are $(8,-3,9,5)$, $(4,2,7,-12)$, $(-11,9,-8,2)$ and $(7,-12,12,-13)$, matching `bit_pattern_31_` element for element, and all 1024 entries lie in $[-13, 12]$, which is the bound that fixes `EDGE_THRESHOLD` in Section 3.2.1. `basalt` therefore inherits ORB's decorrelation learning without carrying its training code.

```cpp
Eigen::Rotation2Dd rot(angle);
Eigen::Matrix2d mat_rot = rot.matrix();

for (int i = 0; i < 256; i++) {
  Eigen::Vector2d va(pattern_31_x_a[i], pattern_31_y_a[i]),
      vb(pattern_31_x_b[i], pattern_31_y_b[i]);

  Eigen::Vector2i vva = (mat_rot * va).array().round().cast<int>();
  Eigen::Vector2i vvb = (mat_rot * vb).array().round().cast<int>();

  descriptor[i] =
      img_raw(cx + vva[0], cy + vva[1]) < img_raw(cx + vvb[0], cy + vvb[1]);
}
```

Two deviations from the reference ORB implementation are visible here and both are deliberate to record. First, the rotation is applied at full precision per keypoint, whereas OpenCV quantises $\theta$ to increments of $2\pi/30$ and precomputes thirty steered pattern tables. `basalt` pays two matrix-vector products and a rounding per bit in exchange for removing a steering quantisation of up to six degrees. Second, and more consequentially, no smoothing is applied to the patch before the tests. Calonder and colleagues established that smoothing is not optional for BRIEF, since each test is effectively the sign of a derivative and an unsmoothed sign is maximally sensitive to sensor noise, and they recommended a Gaussian of standard deviation 2 over a 9 by 9 window. OpenCV's ORB achieves the same end with a 5 by 5 box filter evaluated through an integral image. `basalt` samples the raw 16 bit pixels directly, so every bit rests on a single pixel pair whose steered locations have themselves been displaced by up to half a pixel by rounding.

**Properties and limitations.** The descriptor occupies 32 bytes, against 512 for SIFT, and a comparison costs four XOR and four `POPCNT` operations against a 128 dimensional Euclidean norm. For a mapper holding tens of keyframes at 800 to 1800 features each, that ratio decides whether the descriptor store remains in cache. The Hamming metric is also what makes the threshold `mapper_max_hamming_distance` $= 70$ interpretable. Two independent random 256 bit codes have a Hamming distance distributed as $\mathrm{Binomial}(256, \tfrac{1}{2})$, with mean 128 and standard deviation 8, so the threshold sits 7.25 standard deviations into the lower tail and admits a random pair with probability $1.3 \times 10^{-13}$. The threshold is thus extremely conservative against chance agreement, and any false match that survives it is a genuine appearance ambiguity in the scene rather than a statistical accident.

Against those strengths stand three limitations. The descriptor inherits the single-scale detection of Section 3.2.1 and carries no octave, so it is not scale invariant. It is not affine invariant, since steering corrects a planar rotation but not the anisotropic distortion induced by a change in viewing angle on a slanted surface. And the absent smoothing costs real matching accuracy, which is the most tractable of the three to remedy.

#### 3.2.4 Unprojection, Bag-of-Words Encoding and Database Insertion

The remaining three steps turn the per-camera descriptor set into a record that the matching stages of Section 3.3 can consume. Unprojection lifts each 2D corner onto the unit sphere through the calibrated camera model, producing `kd.corners_3d` as homogeneous bearing vectors, and it is these rays rather than the pixel coordinates that the essential-matrix and RANSAC verifiers operate on, which is what allows a single code path to serve pinhole, Kannala-Brandt and double-sphere cameras alike.

Bag-of-Words encoding then compresses the descriptor set into a retrievable signature. `basalt` departs from the DBoW2 lineage here in a way worth stating plainly. DBoW2 trains an offline hierarchical vocabulary by $k$-means over a large descriptor corpus and ships the resulting tree as a data file. `HashBowBase::compute_hash` (`hash_bow.h:33-39`) instead forms the word by extracting `num_bits` descriptor bits at positions drawn from a fixed compile-time random permutation, which is a locality-sensitive hash rather than a trained quantiser. With `mapper_bow_num_bits` $= 16$ the vocabulary is the $2^{16} = 65536$ possible words, and no vocabulary file, no training corpus and no domain assumption is required. The bag-of-words vector is the L1 normalised histogram of word frequencies, $\mathbf{v}_w = c_w / \sum_j c_j$.

The price of that simplicity is quantifiable. Two descriptors hash to the same word if and only if they agree on all 16 selected bits, so two unrelated descriptors collide with probability $2^{-16} = 1.5 \times 10^{-5}$, which is the desired behaviour, but a genuinely matching pair separated by a Hamming distance of $h$ shares a word only with probability $\binom{256-h}{16} \big/ \binom{256}{16}$. That evaluates to 0.52 at $h = 10$, 0.26 at $h = 20$ and 0.13 at $h = 30$. A good match is therefore more likely than not to be split across two different words, and the retrieval score of a true revisit is correspondingly depressed. This is the reason `mapper_frames_to_match_threshold` must be as low as 0.04, and it is directly relevant to the zero-retrieval failure recorded in the context store.

The score itself is computed by `HashBowStl::querry_database` (`hash_bow.h:229-260`) as

```cpp
scores[v.first] += std::abs(kv.second - v.second) - std::abs(kv.second) - std::abs(v.second);
...
results.emplace_back(kv.first, -kv.second / 2.0);
```

For non-negative histogram entries the accumulated quantity is $|q_w - d_w| - q_w - d_w = -2\min(q_w, d_w)$, so the reported score is the histogram intersection $\sum_w \min(q_w, d_w)$. Since both vectors are L1 normalised this equals $1 - \tfrac{1}{2}\lVert \mathbf{q} - \mathbf{d} \rVert_1$, which is exactly the L1 score of Gálvez-López and Tardós. `basalt` reproduces the DBoW2 similarity measure over an untrained hash vocabulary, and `add_to_database` (`hash_bow.h:219-227`) maintains the inverted index that keeps the query cost proportional to the number of occupied words rather than to the database size.

#### 3.2.5 Significance — Why This Combination

The three algorithms above are not an arbitrary assembly of well-known components. Each is selected by a constraint that the mapping thread imposes, and together they resolve a tension that the optical-flow front end does not face.

That tension is the difference between tracking and recognition. The visual odometry front end matches a frame against its immediate predecessor, where displacement is small, illumination is essentially unchanged and a patch and a gradient descent suffice. The mapper must match a keyframe against any keyframe in the local map, and eventually against a keyframe from an earlier traversal of the same place, where the baseline may be metres, the viewing angle tens of degrees and the exposure entirely different. No patch-based tracker crosses that gap. A descriptor is mandatory, and once a descriptor is mandatory the question becomes which one is affordable.

Affordability is the binding constraint, because the mapper runs concurrently with the estimator on the same vehicle. Detection is invoked on every keyframe entering the local map through `LocalMapper::MapLocally`, which calls the inherited `detect_keypoints` at `local_mapper.cpp:170`, and measurements recorded in the context store put a single `CullRedundantKeyframes` pass at a median of 8.2 milliseconds, so the extraction stage has a budget of the same order and not more. That rules out SIFT and SURF at 800 to 1800 features per image and leaves the binary family.

Within those constraints each choice answers a specific need. Shi-Tomasi supplies corners whose structure tensor is well conditioned, which matters twice over, once because such corners are repeatable across the wide baselines the mapper must bridge, and once because the same conditioning governs the reprojection Jacobians that the bundle adjustment of Section 3.6 will assemble from these very observations. A corner selected by minimum eigenvalue is a corner whose image position constrains the pose, so detection and optimisation are aligned by construction rather than by coincidence. The intensity centroid supplies the rotation invariance that the rest of the pipeline cannot do without, at a cost of one accumulation pass over a 707 pixel disc and no gradients, which is the cheapest credible orientation estimate available. A gradient histogram would cost several times as much for an accuracy the wide-baseline matching cannot exploit. Steered BRIEF supplies discriminability at 32 bytes and four instructions per comparison, and because ORB's learned pattern is embedded verbatim, `basalt` receives the decorrelation and variance properties that make those 256 bits carry close to 256 bits of information, without importing a training pipeline.

The binary representation then propagates its advantage forward through the whole mapping stack. The Hamming metric makes `mapper_max_hamming_distance` calibratable against a known binomial null distribution, as computed in Section 3.2.3, so a match threshold can be set from first principles rather than by tuning. The same bit string is the input to the hash vocabulary, so place recognition needs no second representation and no offline training artefact, a property of real operational value for a vehicle deployed into an environment for which no vocabulary was ever trained. And the resulting correspondences, once verified by the essential matrix and RANSAC of Section 3.3, are exactly the inputs the track builder of Section 3.4 fuses and the triangulation of Section 3.5 converts into the landmarks over which Section 3.6 optimises. The detector, the orientation estimate and the descriptor are therefore the foundation on which the entire non-linear factor recovery formulation of this document rests, since a pose graph with recovered factors cannot correct drift it was never given the correspondences to observe.

What the combination does not provide is equally worth naming. It is not scale invariant, it is not affine invariant, and its descriptor is computed without the smoothing its own originating paper requires. These are the axes along which the pipeline can be improved, and they are taken up next.

#### 3.2.6 Possible Improvements

The improvements below are drawn from the literature and ordered by the aspect of system behaviour they address. Each names the change, the evidence for it and the expected cost.

**Improving tracking and matching robustness.**

Restoring the patch smoothing that BRIEF prescribes is the single cheapest correction available. Calonder and colleagues showed that recognition rate degrades sharply without it and that a Gaussian of standard deviation 2, or equivalently a 5 by 5 box filter evaluated through an integral image as OpenCV's ORB does, is sufficient. The change is confined to `computeDescriptors` and costs one integral-image pass per keyframe, which is negligible beside the detection it follows.

Adding a scale pyramid would remove the most restrictive of the three limitations identified in Section 3.2.3. Detecting over eight octaves at a scale factor of 1.2, as ORB does, and scaling the sampling pattern by the octave, extends the range of viewpoint change over which a landmark remains matchable and directly widens the window in which a loop can be recognised. The cost is a detection pass per octave, roughly a factor of $\sum_k 1.2^{-2k} \approx 2.2$ on the detection stage alone.

Gating the orientation estimate on $\lVert (m_{10}, m_{01}) \rVert$ would suppress the silent $\mathrm{atan2}(0,0) = 0$ degeneracy of Section 3.2.2. Keypoints whose centroid vector is short carry an arbitrary steering angle and produce descriptors that cannot match their own counterparts, so discarding them costs nothing and removes a source of unexplained match failure.

Replacing the descriptor outright is the larger step. BEBLID learns its binary tests by AdaBoost with equal weak-learner weights and reports accuracy close to SIFT at a cost below ORB, and it is available in OpenCV from version 4.5.1, where substituting it for the ORB descriptor has been measured to improve matching by around 14 percent. It is a drop-in replacement in the sense that it consumes the same keypoints and emits the same binary type, so `matchDescriptors`, the hash vocabulary and every threshold downstream continue to apply.

Beyond the binary family, learned front ends are now within embedded budget. XFeat achieves sparse inference on CPU at VGA resolution in real time and reports performance comparable to SuperPoint at a small fraction of its cost, with measurements on embedded hardware of 1.8 frames per second against 0.16 for SuperPoint and 0.58 for ALIKE. Reported integrations of ALIKED, SuperPoint and XFeat into visual-inertial pipelines find learned front ends viable in real time but not uniformly superior to classical tracking, so this warrants evaluation rather than adoption on principle.

**Improving bag-of-words matching.**

The quantified weakness of the present scheme is the word-splitting probability derived in Section 3.2.4, where a true match at a Hamming distance of 20 shares a word only 26 percent of the time. Three remedies exist, in ascending order of disruption.

The first is to hash each descriptor into several independent words rather than one, drawing $L$ disjoint bit subsets from the permutation and inserting the descriptor under all of them. The probability that a true match is missed entirely falls as $(1 - p)^L$, which at $p = 0.26$ and $L = 4$ takes the miss rate from 74 percent to 30 percent, at the cost of an $L$-fold larger inverted index. This is the classical multi-index hashing construction and it requires no training.

The second is to adopt a search structure designed for binary descriptors. HBST builds a binary search tree whose splits are chosen on individual descriptor bits, giving insertion and search in time logarithmic in the database size while retaining a bounded Hamming neighbourhood at each leaf, and it is distributed as a header-only C++ library, which suits a codebase that already avoids heavyweight dependencies. Multi-index hashing offers exact $k$ nearest neighbour search in Hamming space as an alternative with a stronger guarantee.

The third is to replace appearance retrieval with a learned global descriptor. Evaluations consistently find DBoW2-class methods outperformed by NetVLAD-derived approaches under large viewpoint change and severe perceptual aliasing, and recent work reports large retrieval-time reductions at map scales far beyond the present local map. Within the classical family, BoWG addresses perceptual aliasing specifically by grouping co-occurring words, and iBoW-LCD builds the vocabulary incrementally and online, which would preserve the present system's freedom from a trained vocabulary file.

**Improving map fidelity.**

Enabling sub-pixel corner refinement addresses the quantisation floor identified in Section 3.2.1, where the $0.289$ pixel standard deviation of pixel-grid quantisation already exceeds the $0.25$ assumed by `mapper_obs_std_dev`. The `cv::cornerSubPix` call needed is already present in commented form at `keypoints.cpp:239-243`. Restoring it would reduce reprojection residuals and, more importantly, make the observation covariance of Section 3.1.1 an honest description of the measurement rather than an optimistic one.

Enforcing a homogeneous spatial distribution of keypoints is the second significant gain. The greedy `minDistance` suppression that `goodFeaturesToTrack` performs enforces a local separation but does nothing to prevent all retained corners clustering in the most textured region of the image, and a spatially concentrated observation set conditions the pose estimate poorly whatever its cardinality. ORB-SLAM addresses this with a quadtree that subdivides until the requested count is reached. Bailo and colleagues showed that adaptive non-maximal suppression selects the strongest and best distributed subset far more cheaply, their Suppression via Square Covering variant in particular, and reported that it substantially improves motion estimation accuracy for a fixed keypoint budget. Since the mapping thread operates under exactly such a budget through `mapper_detection_num_points`, this is the change with the best ratio of accuracy gained to cost incurred.

Replacing the relative quality gate would remove the failure mode described in Section 3.2.1, where one bright structure suppresses detection across the whole frame. Applying the quality level within each cell of a coarse grid, rather than against the global maximum, decouples regions of differing contrast and composes naturally with the spatial distribution measure above.

**Improving detection speed.**

Restoring the parallelism that commit `5de98a1` removed is the largest single saving available and the one most clearly worth taking. The `tbb::parallel_for` was withdrawn because concurrent insertion corrupted `HashBowStl::inverted_index`, not because the detection work itself is serial. Splitting the loop into a parallel phase that performs detection, orientation, description, unprojection and bag-of-words computation into per-frame local storage, followed by a short serial phase that performs only the database insertions, recovers nearly all of the concurrency while removing the data race by construction. The insertion phase touches no image data and is a small fraction of the total.

Substituting FAST for Shi-Tomasi in the detection stage is the conventional speed trade. FAST evaluates a learned decision tree over a sixteen pixel Bresenham circle and is roughly an order of magnitude faster than a structure-tensor computation, and it is already implemented in this file at `keypoints.cpp:213` for the visual odometry path. The cost is the loss of the eigenvalue conditioning argument of Section 3.2.1, so it should be paired with a Harris or Shi-Tomasi re-ranking of the FAST candidates, which is what ORB itself does.

Two further reductions are available without changing any algorithm. The descriptor loop of Section 3.2.3 recomputes $\mathbf{R}(\theta)$ per keypoint and performs 512 matrix-vector products per descriptor, which ORB's thirty precomputed steered tables avoid entirely at a steering quantisation of six degrees. And detection operates on the 8 bit reduction while description reads the 16 bit original, so the shifted image is built for the detector and then discarded, where a single pass producing both representations, or description on the 8 bit image throughout, would remove one full-image traversal per camera per keyframe.

| Axis | Change | Expected benefit | Cost |
|------|--------|------------------|------|
| Matching | Patch smoothing before BRIEF tests | Restores the stability BRIEF requires | One integral-image pass |
| Matching | Scale pyramid over eight octaves | Scale invariance, wider loop-closure range | About 2.2× detection |
| Matching | Gate orientation on centroid magnitude | Removes silent `atan2(0,0)` degeneracy | Negligible |
| Matching | BEBLID in place of steered BRIEF | Around 14% better matching, faster than ORB | Drop-in, same binary type |
| Matching | Learned front end (XFeat, ALIKED) | Largest robustness gain reported | Inference budget, evaluation required |
| Retrieval | $L$ disjoint hash words per descriptor | Miss rate $(1-p)^L$ instead of $1-p$ | $L$× inverted index |
| Retrieval | HBST or multi-index hashing | Logarithmic search, exact kNN option | Header-only dependency |
| Retrieval | Learned global descriptor | Robust to viewpoint change and aliasing | Model, training data |
| Fidelity | Sub-pixel corner refinement | Residual floor below `mapper_obs_std_dev` | Already present, commented out |
| Fidelity | ANMS or quadtree distribution | Better conditioned pose for fixed budget | Small, one suppression pass |
| Fidelity | Per-cell quality threshold | Removes global-maximum suppression failure | Negligible |
| Speed | Restore TBB with serial insertion phase | Recovers the concurrency of `5de98a1` | Restructure, no race by construction |
| Speed | FAST candidates with Shi-Tomasi re-ranking | About 10× on the detection stage | Loses pure eigenvalue selection |
| Speed | Precomputed steered pattern tables | Removes 512 products per descriptor | 6° steering quantisation |
| Speed | Single-pass 8 bit and 16 bit preparation | One fewer full-image traversal | Negligible |

### 3.3 Feature Matching

#### 3.3.1 Stereo Matching

For each stereo pair `(tcid_left, tcid_right)` at the same timestamp, `match_stereo` (`nfr_mapper.cpp:541-587`) uses the known stereo extrinsic calibration $\mathbf{T}_{0,1} = \mathbf{T}_{w,0}^{-1} \mathbf{T}_{w,1}$ to compute the essential matrix $\mathbf{E}$ via `computeEssential`. Descriptor matching is performed with `matchDescriptors` subject to a Hamming distance threshold `mapper_max_hamming_distance` and second-best ratio test `mapper_second_best_test_ratio`. Only geometrically verified inliers passing the essential-matrix check (`findInliersEssential`, epipolar tolerance $10^{-3}$) with at least 16 inliers are retained.

#### 3.3.2 Appearance-Based Temporal and Loop-Closure Matching

`match_all` (`nfr_mapper.cpp:588-707`) performs cross-frame matching for temporal and potential loop-closure connections:

1. **BoW retrieval** (parallel over all frames, `nfr_mapper.cpp:638`): For each `TimeCamId`, query the `HashBow` database for the `mapper_num_frames_to_match` most visually similar frames. The query passes `&tcid.frame_id` as the `max_t_ns` bound (`nfr_mapper.cpp:612-614`), so only strictly earlier frames are ever returned, and pairs from the same timestamp or with a similarity score below `mapper_frames_to_match_threshold` are discarded. The retrieval stage retained its `tbb::parallel_for` because `querry_database` only reads the inverted index.
2. **Descriptor matching**: For each candidate pair, `matchDescriptors` is run with a fixed Hamming distance of 70 and ratio 1.2.
3. **Geometric verification**: `findInliersRansac` runs a RANSAC relative-pose test (threshold `mapper_ransac_threshold`, minimum inliers `mapper_min_matches`) to retain only geometrically consistent matches. The solver is OpenGV's `CentralRelativePoseSacProblem` in `STEWENIUS` mode (`keypoints.cpp:383-389`), which is the five-point essential-matrix minimal solver operating on the calibrated bearing vectors `kd.corners_3d`, followed by a non-linear refinement of the recovered pose. Correction, 2026-09-12. This step previously described it as a fundamental-matrix test. No fundamental matrix is estimated anywhere in `basalt`, because the intrinsics are always known and the unprojection of Section 3.2.4 has already removed them.

All verified match pairs are stored in `NfrMapper::feature_matches`.

### 3.4 Feature Track Building

`build_tracks` (`nfr_mapper.cpp:671`) constructs multi-frame feature tracks from the pairwise matches using a `TrackBuilder` based on a Union-Find data structure:

1. **Build**: `trackBuilder.Build(feature_matches)` fuses all pairwise correspondences into consistent multi-frame tracks.
2. **Filter**: `trackBuilder.Filter(config.mapper_min_track_length)` removes any track observed in fewer than `mapper_min_track_length` frames, as well as tracks with conflicting observations (the same feature appearing twice in one frame).
3. **Export**: `trackBuilder.Export(feature_tracks)` writes the tracks as `std::map<TrackId, std::map<TimeCamId, FeatureId>>` into `NfrMapper::feature_tracks`.

### 3.5 Landmark Parameterization and Triangulation

Landmarks are parameterised identically to the VIO backend: each landmark $j$ is represented relative to its **host frame** $h(j)$ as a tuple $(u, v, \rho)$ where $(u, v) \in \mathbb{R}^2$ is the stereographic projection of the unit-sphere ray from the host camera, and $\rho > 0$ is the inverse distance (inverse depth). This minimal, singularity-free parameterisation is implemented in `StereographicParam<double>::project`.

Landmark initialisation in `setup_opt` (`nfr_mapper.cpp:697`) proceeds via Direct Linear Transform (DLT) triangulation. For each feature track, the first observation is taken as the host frame. For each subsequent observation with sufficient baseline:

$$\left || \mathbf{T}_{h,o}.\mathbf{t} \right ||^2 \geq d_{\min}^2 \quad (\texttt{mapper\_min\_triangulation\_dist}^2)$$

where $\mathbf{T}_{h,o}$ refers to the transform between the host keyframe $\mathbf{h}$ and the observation frame $\mathbf{o}$ and $d_{\min}$ is the minimum baseline distance between two keyframes required to perform triangulation accurately, the 3D point is triangulated using `BundleAdjustmentBase::triangulate` and validated:

```cpp
if (!pos_3d.array().isFinite().all() || pos_3d[3] <= 0 || pos_3d[3] > 2.0)
    continue;
```

The inverse depth is stored as `pos.inv_dist = pos_3d[3]` and the stereographic direction as `pos.direction = StereographicParam<double>::project(pos_3d)`. The landmark is then registered in the database via `lmdb.addLandmark` and all track observations are added via `lmdb.addObservation`.

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

### 4a. Execution Flow & Pipeline

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
3. Identify the marginalised keyframe `kf_id = *m.kfs_to_marg.cbegin()` and its pose `T_w_i_kf`. Please note that we currently assume that only one keyframe may be marginalised at a time.
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

### 4b. Key Classes and Interface

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
| `linearizeHelper(...)` | Non-static entry point called by `NfrMapper::optimize`; dispatches to `linearizeHelperStatic` over `rld_vec` via TBB `parallel_for` |
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
| `applyInc(inc)` | Applies a 6-DOF $SE(3)$ perturbation (via exponential map) to the pose stored in `PoseStateWithLin`; called per-keyframe after each accepted solver step |

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

**15. `TrackBuilder`**
- **Header:** `include/basalt/utils/tracks.h`
- **Purpose:** Union-Find (disjoint-set) based data structure that fuses pairwise feature correspondences from `feature_matches` into multi-frame tracks stored in `feature_tracks`.
- **Key Methods:**

| Method | Description |
|---|---|
| `Build(feature_matches)` | Iterates over all pairwise matches and merges connected observations into disjoint sets |
| `Filter(min_track_length)` | Removes tracks shorter than `mapper_min_track_length` and tracks with conflicting observations (same feature appearing twice in one frame) |
| `Export(feature_tracks)` | Writes surviving tracks as `std::map<TrackId, std::map<TimeCamId, FeatureId>>` into `NfrMapper::feature_tracks` |

---

**16. Standalone Feature Processing Functions**
- **Header/Source:** `src/vi_estimator/nfr_mapper.cpp` (static free functions called inside `detect_keypoints`, `match_stereo`, `match_all`)
- **Purpose:** Low-level building blocks for keypoint detection, orientation estimation, descriptor extraction, and cross-frame matching used by the mapper's feature front-end.

| Function | Description |
|---|---|
| `detectKeypointsMapping(img, num_points)` | Extracts up to `mapper_detection_num_points` FAST/Harris corners from a single image |
| `computeAngles(img, corners)` | Estimates the dominant orientation (angle) for each detected corner, needed for rotation-invariant ORB descriptors |
| `computeDescriptors(img, corners)` | Computes binary (ORB-style) descriptors for each oriented corner |
| `computeEssential(T_0_1)` | Derives the essential matrix $\mathbf{E}$ from a known relative pose $\mathbf{T}_{0,1}$ for stereo geometric verification |
| `matchDescriptors(desc0, desc1, max_dist, ratio)` | Brute-force Hamming-distance descriptor matching with distance threshold and second-best ratio test |
| `findInliersEssential(kd0, kd1, E, tol)` | Marks matches as inliers if the epipolar constraint $\mathbf{x}_1^T \mathbf{E} \mathbf{x}_0 < \text{tol}$ is satisfied |
| `findInliersRansac(kd0, kd1, threshold, min_inliers)` | RANSAC-based fundamental-matrix estimation to geometrically verify cross-frame matches; rejects pairs below `mapper_min_matches` inliers |

---

**17. `SparseHashAccumulator<Scalar>`**
- **Header:** `include/basalt/optimization/accumulator.h`
- **Purpose:** A sparse hash-map based symmetric matrix accumulator used during linearisation. Accumulates $\mathbf{J}^T \boldsymbol{\Omega} \mathbf{J}$ and $\mathbf{J}^T \boldsymbol{\Omega} \mathbf{r}$ contributions from individual factors into a global pose-only Hessian $\mathbf{H}$ and gradient $\mathbf{b}$, using hash-indexed blocks to avoid materialising a dense matrix.
- **Key Methods:**

| Method | Description |
|---|---|
| `setup_solver()` | Converts the accumulated sparse hash entries into a factorisable sparse linear system (e.g., Cholesky or LDLT); must be called before `solve()` |
| `solve(b)` | Solves $\mathbf{H} \boldsymbol{\xi} = \mathbf{b}$ for the pose increment vector using the factorisation produced by `setup_solver` |

---

**18. `StereographicParam<Scalar>`**
- **Header:** `thirdparty/basalt-headers/include/basalt/camera/stereographic_param.hpp`
- **Purpose:** Implements the stereographic projection used to represent unit-sphere bearing directions as a minimal 2-vector $(u, v)$. This singularity-free parameterisation underpins the landmark direction storage in `Keypoint`.
- **Key Methods:**

| Method | Description |
|---|---|
| `project(bearing_3d)` | Maps a homogeneous 4-vector (or 3D unit ray) from the host camera onto the 2D stereographic plane, returning $(u, v)$ |
| `unproject(u, v)` | Lifts a 2D stereographic point back to a unit-sphere 3D bearing vector |

---

**19. `TimeCamId`**
- **Header:** `include/basalt/utils/common_types.h`
- **Purpose:** A composite key type representing a unique `(frame_id, cam_id)` pair. Used as the primary index into `feature_corners`, `feature_matches`, and `LandmarkDatabase` observations.

| Field | Type | Description |
|---|---|---|
| `frame_id` | `int64_t` | Timestamp (nanoseconds) identifying the keyframe |
| `cam_id` | `int` | Camera index within the multi-camera rig (0 = left, 1 = right) |

---

**20. `KeypointsData`**
- **Header:** `include/basalt/utils/common_types.h`
- **Purpose:** Container for all keypoints and their associated data for a single `(frame, camera)` image. Stored in `NfrMapper::feature_corners` keyed by `TimeCamId`.

| Field | Type | Description |
|---|---|---|
| `corners` | `std::vector<Eigen::Vector2d>` | Pixel coordinates of detected keypoints |
| `corner_angles` | `std::vector<double>` | Dominant orientations for each corner (set by `computeAngles`) |
| `corner_descriptors` | `std::vector<std::bitset<256>>` | Binary descriptors for each corner (set by `computeDescriptors`) |
| `bow_vector` | `HashBow<256>::BowVector` | Bag-of-Words encoding of the descriptor set |
| `pyramid` | image pyramid | Multi-scale image pyramid used during descriptor computation |

---

**21. `OpticalFlowInput` / `OpticalFlowResult`**
- **Header:** `include/basalt/optical_flow/optical_flow.h`
- **Purpose:** Data exchange types between the optical-flow front-end and the VIO/mapper back-end.
  - `OpticalFlowInput` carries raw image frames (timestamps + per-camera images). Stored in `NfrMapper::img_data` keyed by timestamp.
  - `OpticalFlowResult` carries tracked feature observations from the optical-flow thread, embedded inside `MargData::opt_flow_res`. The mapper unpacks these in `processMargData` to populate `img_data`.

---

**22. `PoseStateWithLin<Scalar>`**
- **Header:** `include/basalt/utils/imu_types.h`
- **Purpose:** Wraps a single 6-DOF keyframe pose $\mathbf{T}_{w,i} \in SE(3)$ together with its **linearisation point** $\mathbf{T}_0$. The linearisation point allows the optimiser to track the state at the start of each iteration and compute increments in the tangent space. Used in `BundleAdjustmentBase::frame_poses` and `MargData::frame_poses`.
- **Key Method:** `applyInc(xi)` — applies a 6-DOF tangent-space increment $\boldsymbol{\xi}$ via the exponential map to update the current pose.

---

**23. `PoseVelBiasStateWithLin<Scalar>`**
- **Header:** `include/basalt/utils/imu_types.h`
- **Purpose:** Extends `PoseStateWithLin` with IMU velocity and accelerometer/gyroscope bias estimates, forming a full 15-DOF navigation state. Used in `BundleAdjustmentBase::frame_states` and `MargData::frame_states` for frames that are still actively tracked by the IMU integrator.

---

**24. `Calibration<Scalar>`**
- **Header:** `thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp`
- **Purpose:** Holds the complete sensor calibration for the multi-camera (+ IMU) rig: per-camera intrinsic models, camera-to-IMU extrinsic transforms, and time-offset parameters. Accessed via `BundleAdjustmentBase::calib`. The intrinsic model provides `unproject` and `project` methods used during feature unprojection and reprojection error computation.
- **Key Field:** `intrinsics[cam_id]` — per-camera intrinsic model with `project(bearing)` and `unproject(pixel)` methods.

---

**25. Configuration Parameters (`VioConfig` — `mapper_*` fields)**
- **Header:** `include/basalt/utils/vio_config.h`
- **Purpose:** All mapper tuning parameters are grouped in the `VioConfig` struct and accessed via `NfrMapper::config`. The table below lists every `mapper_*` field referenced in this document.

| Parameter | Default context | Description |
|---|---|---|
| `mapper_detection_num_points` | Feature detection | Maximum number of FAST/Harris corners to detect per image frame |
| `mapper_max_hamming_distance` | Stereo + cross-frame matching | Maximum Hamming distance for a descriptor match to be accepted |
| `mapper_second_best_test_ratio` | Stereo + cross-frame matching | Ratio-test threshold: a match is accepted only if `best_dist / second_best_dist < ratio` |
| `mapper_num_frames_to_match` | BoW retrieval (`match_all`) | Number of nearest-neighbour frames to retrieve from the BoW database per query |
| `mapper_frames_to_match_threshold` | BoW retrieval (`match_all`) | Minimum BoW similarity score; pairs below this threshold are discarded before descriptor matching |
| `mapper_ransac_threshold` | Geometric verification (`match_all`) | RANSAC inlier threshold (pixels) for fundamental-matrix estimation |
| `mapper_min_matches` | Geometric verification (`match_all`) | Minimum number of RANSAC inliers required to retain a frame pair |
| `mapper_min_track_length` | Track filtering (`build_tracks`) | Minimum number of frames a track must span to be kept |
| `mapper_min_triangulation_dist` | Triangulation (`setup_opt`) | Minimum baseline distance $d_{\min}$ between host and observation frame for DLT triangulation |
| `mapper_obs_std_dev` | BA cost function | Standard deviation $\sigma$ of visual observations; scales the reprojection cost |
| `mapper_obs_huber_thresh` | BA cost function | Huber loss threshold; observations with error above this are down-weighted |
| `mapper_use_lm` | Optimiser | If true, use Levenberg–Marquardt; if false, use Gauss-Newton |
| `mapper_use_factors` | Optimiser | If true, include NFR relative-pose and roll-pitch factors in the BA cost |
| `mapper_lm_lambda_min` | LM damping | Lower bound on the LM damping parameter $\lambda$ |
| `mapper_lm_lambda_max` | LM damping | Upper bound on the LM damping parameter $\lambda$ |
| `mapper_no_factor_weights` | NFR factor recovery | If true, recovered NFR factors are not weighted by their information matrix |

---

**26. `FeatureId`, `FrameId`, `CamId`**
- **Header:** `include/basalt/utils/common_types.h`
- **Definition:**
  ```cpp
  using FeatureId = int;          // index of a 2D feature within a single image
  using FrameId   = int64_t;      // nanosecond timestamp identifying a frame
  using CamId     = std::size_t;  // camera index within the multi-camera rig
  ```
- **Purpose:** Fundamental scalar aliases used throughout the feature pipeline. `FeatureId` indexes into `KeypointsData::corners` for a specific image. `FrameId` is the primary key for all per-frame maps. `CamId` selects the camera (0 = left, 1 = right in a stereo rig).

---

**27. `MatchData`**
- **Header:** `include/basalt/utils/common_types.h`
- **Definition:** struct
- **Purpose:** Stores all feature correspondences and the recovered relative pose for a single image pair `(i, j)`. Produced by `match_stereo` and `match_all`; consumed by `build_tracks`.

| Field | Type | Description |
|---|---|---|
| `T_i_j` | `Sophus::SE3d` | Estimated transformation from camera `j` to camera `i` (either from stereo calibration or RANSAC homography) |
| `matches` | `std::vector<std::pair<FeatureId, FeatureId>>` | All accepted descriptor matches `(featureId_i, featureId_j)` |
| `inliers` | `std::vector<std::pair<FeatureId, FeatureId>>` | Geometrically verified inlier subset of `matches` |

---

**28. `Matches`**
- **Header:** `include/basalt/utils/common_types.h`
- **Definition:**
  ```cpp
  using Matches = tbb::concurrent_unordered_map<
      std::pair<TimeCamId, TimeCamId>, MatchData,
      std::hash<std::pair<TimeCamId, TimeCamId>>, ...>;
  ```
- **Purpose:** Thread-safe map from an image pair `(TimeCamId_i, TimeCamId_j)` to the corresponding `MatchData`. Populated concurrently by `match_stereo` and `match_all`; read sequentially by `build_tracks`. Stored in `NfrMapper::feature_matches`.

---

**29. `Corners`**
- **Header:** `include/basalt/utils/common_types.h`
- **Definition:**
  ```cpp
  using Corners = tbb::concurrent_unordered_map<TimeCamId, KeypointsData,
                                                std::hash<TimeCamId>>;
  ```
- **Purpose:** Thread-safe map from a `(frame, camera)` id to all keypoints and descriptors detected in that image. Populated concurrently by `detect_keypoints`; read by `match_stereo`, `match_all`, and `setup_opt`. Stored in `NfrMapper::feature_corners`.

---

**30. `ImageFeaturePair`**
- **Header:** `include/basalt/utils/common_types.h`
- **Definition:**
  ```cpp
  using ImageFeaturePair = std::pair<TimeCamId, FeatureId>;
  ```
- **Purpose:** Uniquely identifies a single 2D feature observation: which image (`TimeCamId`) and which index within that image (`FeatureId`). Used internally by `TrackBuilder` as the node type in the Union-Find structure.

---

**31. `FeatureTrack`, `TrackId`, `FeatureTracks`**
- **Header:** `include/basalt/utils/common_types.h`
- **Definition:**
  ```cpp
  using FeatureTrack  = std::map<TimeCamId, FeatureId>;
  using TrackId       = int64_t;
  using FeatureTracks = std::unordered_map<TrackId, FeatureTrack>;
  ```
- **Purpose:**
  - `FeatureTrack`: A single multi-frame landmark track — maps each image in which the landmark was observed to the local `FeatureId` within that image.
  - `TrackId`: Integer identifier for a track; also serves as the `KeypointId` / `LandmarkId` used as the key in `LandmarkDatabase`.
  - `FeatureTracks`: The complete collection of all tracks output by `TrackBuilder::Export`; stored in `NfrMapper::feature_tracks` and consumed by `setup_opt` to triangulate landmarks.

---

**32. `KeypointObservation<Scalar>`**
- **Header:** `include/basalt/vi_estimator/landmark_database.h`
- **Definition:** struct template
- **Purpose:** Represents a single 2D measurement of a landmark in one image. Used as the value type stored in `Keypoint::obs` and passed to `LandmarkDatabase::addObservation`.

| Field | Type | Description |
|---|---|---|
| `kpt_id` | `int` | Local `FeatureId` of this observation within its image |
| `pos` | `Eigen::Matrix<Scalar, 2, 1>` | 2D pixel coordinates (or normalised bearing in stereographic space) of the observation |

---

**33. `Keypoint<Scalar>`**
- **Header:** `include/basalt/vi_estimator/landmark_database.h`
- **Definition:** struct template
- **Purpose:** Stores a single 3D landmark in the map. The landmark is parameterised relative to a **host keyframe** using a stereographic bearing direction and inverse depth — a minimal, singularity-free representation that avoids Euclidean infinity. Stored in `LandmarkDatabase` keyed by `TrackId`.

| Field | Type | Description |
|---|---|---|
| `host_kf_id` | `TimeCamId` | The `(frame, camera)` in which this landmark was first observed and relative to which `direction` / `inv_dist` are defined |
| `direction` | `Eigen::Matrix<Scalar, 2, 1>` | 2D stereographic projection of the unit bearing vector from the host camera to the landmark |
| `inv_dist` | `Scalar` | Inverse depth along the bearing ray from the host camera |
| `obs` | `Eigen::aligned_map<TimeCamId, Eigen::Matrix<Scalar,2,1>>` | All subsequent observations of this landmark, keyed by `(frame, camera)` |

- **Key Methods:** `backup()` / `restore()` — saves and restores `direction` and `inv_dist` during LM trial steps.

---

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

## 6. References

1. Usenko, V., Demmel, N., Schubert, D., Stückler, J., & Cremers, D. (2020). *Visual-Inertial Mapping with Non-Linear Factor Recovery*. arXiv preprint arXiv:1904.06504v3.
2. Mazuran, M., Burgard, W., & Tipaldi, G. D. (2015). *Nonlinear Factor Recovery for Long-Term SLAM*. The International Journal of Robotics Research (IJRR).
3. `basalt` Mapping Implementation: `include/basalt/vi_estimator/nfr_mapper.h` and `src/vi_estimator/nfr_mapper.cpp`.
4. `basalt` Mapper Sample Driver: `src/mapper.cpp`.
5. `basalt` NFR Utilities: `include/basalt/utils/nfr.h`.
6. `basalt` Marginalisation Documentation: `doc/Marginalisation.md`.
7. `basalt` Feature Extraction and Matching Documentation: `doc/MappingFeatureExtractionMatching.md`.

### Feature Detection and Description (Section 3.2)

8. Moravec, H. P. (1980). *Obstacle Avoidance and Navigation in the Real World by a Seeing Robot Rover*. PhD thesis, Stanford University.
9. Harris, C., & Stephens, M. (1988). *A Combined Corner and Edge Detector*. Alvey Vision Conference, 147–151.
10. Shi, J., & Tomasi, C. (1994). *Good Features to Track*. IEEE CVPR, 593–600.
11. Lucas, B. D., & Kanade, T. (1981). *An Iterative Image Registration Technique with an Application to Stereo Vision*. IJCAI, 674–679.
12. Rosten, E., & Drummond, T. (2006). *Machine Learning for High-Speed Corner Detection*. ECCV, 430–443.
13. Rosin, P. L. (1999). *Measuring Corner Properties*. Computer Vision and Image Understanding, 73(2), 291–307.
14. Lowe, D. G. (2004). *Distinctive Image Features from Scale-Invariant Keypoints*. IJCV, 60(2), 91–110.
15. Bay, H., Tuytelaars, T., & Van Gool, L. (2006). *SURF: Speeded Up Robust Features*. ECCV, 404–417.
16. Calonder, M., Lepetit, V., Strecha, C., & Fua, P. (2010). *BRIEF: Binary Robust Independent Elementary Features*. ECCV, 778–792. Extended as Calonder et al. (2012), *BRIEF: Computing a Local Binary Descriptor Very Fast*, IEEE TPAMI, 34(7), 1281–1298.
17. Rublee, E., Rabaud, V., Konolige, K., & Bradski, G. (2011). *ORB: An Efficient Alternative to SIFT or SURF*. IEEE ICCV, 2564–2571.
18. Leutenegger, S., Chli, M., & Siegwart, R. (2011). *BRISK: Binary Robust Invariant Scalable Keypoints*. IEEE ICCV, 2548–2555.
19. Alahi, A., Ortiz, R., & Vandergheynst, P. (2012). *FREAK: Fast Retina Keypoint*. IEEE CVPR, 510–517.
20. Gálvez-López, D., & Tardós, J. D. (2012). *Bags of Binary Words for Fast Place Recognition in Image Sequences*. IEEE Transactions on Robotics, 28(5), 1188–1197.
21. Mur-Artal, R., Montiel, J. M. M., & Tardós, J. D. (2015). *ORB-SLAM: A Versatile and Accurate Monocular SLAM System*. IEEE Transactions on Robotics, 31(5), 1147–1163.

### Improvements Surveyed (Section 3.2.6)

22. Bailo, O., Rameau, F., Joo, K., Park, J., Bogdan, O., & Kweon, I. S. (2018). *Efficient Adaptive Non-Maximal Suppression Algorithms for Homogeneous Spatial Keypoint Distribution*. Pattern Recognition Letters, 106, 53–60. Code at https://github.com/BAILOOL/ANMS-Codes.
23. Suárez, I., Sfeir, G., Buenaposada, J. M., & Baumela, L. (2020). *BEBLID: Boosted Efficient Binary Local Image Descriptor*. Pattern Recognition Letters, 133, 366–372. arXiv:2402.04482. Available in OpenCV from 4.5.1.
24. Schlegel, D., & Grisetti, G. (2018). *HBST: A Hamming Distance Embedding Binary Search Tree for Feature-Based Visual Place Recognition*. IEEE Robotics and Automation Letters. arXiv:1802.09261.
25. Norouzi, M., Punjani, A., & Fleet, D. J. (2014). *Fast Exact Search in Hamming Space with Multi-Index Hashing*. IEEE TPAMI, 36(6), 1107–1119. arXiv:1307.2982.
26. Garcia-Fidalgo, E., & Ortiz, A. (2018). *iBoW-LCD: An Appearance-Based Loop Closure Detection Approach Using Incremental Bags of Binary Words*. IEEE Robotics and Automation Letters. arXiv:1802.05909.
27. *Bag-of-Word-Groups (BoWG): A Robust and Efficient Loop Closure Detection Method Under Perceptual Aliasing* (2025). arXiv:2510.22529.
28. Arandjelović, R., Gronat, P., Torii, A., Pajdla, T., & Sivic, J. (2016). *NetVLAD: CNN Architecture for Weakly Supervised Place Recognition*. IEEE CVPR, 5297–5307.
29. Potje, G., Cadar, F., Araujo, A., Martins, R., & Nascimento, E. R. (2024). *XFeat: Accelerated Features for Lightweight Image Matching*. IEEE CVPR. arXiv:2404.19174. Code at https://github.com/verlab/accelerated_features.
30. DeTone, D., Malisiewicz, T., & Rabinovich, A. (2018). *SuperPoint: Self-Supervised Interest Point Detection and Description*. IEEE CVPR Workshops, 224–236.
31. Zhao, X., Wu, X., Chen, W., Chen, P. C. Y., Xu, Q., & Li, Z. (2023). *ALIKED: A Lighter Keypoint and Descriptor Extraction Network via Deformable Transformation*. IEEE Transactions on Instrumentation and Measurement, 72, 1–16.
