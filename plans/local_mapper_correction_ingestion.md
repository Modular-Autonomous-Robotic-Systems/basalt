# Ingestion of Local Mapper Corrections as Window Residuals

## 1. Motivation

The local mapper refines keyframe poses over a window far larger than the estimator's, and returns them through `LocalMapper::MapLocally` (`src/vi_estimator/local_mapper.cpp:322-324`) to `SqrtKeypointVioEstimator::QueuePoseUpdates` (`include/basalt/vi_estimator/sqrt_keypoint_vio.h:252-260`). `SqrtKeypointVioEstimator::measure` (`src/vi_estimator/sqrt_keypoint_vio.cpp:637-714`) then overwrites the matching entries of `frame_poses` when the correction is below `kRelinThresholdTrans` = 0.10 m and `kRelinThresholdRot` = 0.05 rad. This has three defects.

1. An overwrite moves the current value of a frozen pose away from its first estimate without any residual asking for it, so the prior's offset $\boldsymbol{\delta}$ grows by an amount the prior never saw, and the next marginalisation linearises the departing factors at a point no optimisation produced.
2. Corrections above the threshold are rejected, so the feedback weakens exactly as drift grows (`context/vio_localmapper_correction_loop.md`, Mechanism B).
3. Only `frame_poses` are touched, so the full states, the velocities and the biases never learn of the correction, and the gravity and scale block never sees the mapper's scale.

During the delayed marginalisation period the overwrite also corrupts a readvance, which is why `vio_gs_dm_hold_pose_updates` discards every update while the store is active (`:648-655`).

The proposal replaces the overwrite with residuals that pull the window towards the mapper's keyframe constellation, optimised jointly with the visual, inertial, prior and gravity and scale terms, starting once the first delayed marginalisation has been installed. The residuals reuse Basalt's own relative pose factor, and two ways of choosing the compared keyframe pairs, every keyframe against the oldest shared one and every pair, are implemented side by side so that a replay can decide between them on the evidence of §3.5.

## 2. Literature survey

The survey first builds the machinery every surveyed method shares, namely the maximum a posteriori problem, its Gauss Newton solution, the marginalisation prior, first estimate Jacobians and inertial preintegration. Each method is then read in that machinery, from its residuals to the step that makes it distinctive, and closes with what it offers this estimator.

### 2.1 Notation and identities

A pose is $\mathbf{T} = (\mathbf{R}, \mathbf{p}) \in SE(3)$, the IMU in the world. Basalt perturbs it on the left in the world frame (`PoseState::incPose`, `thirdparty/basalt-headers/include/basalt/imu/imu_types.h:98-101`),

$$\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p},$$

$$\mathbf{R} \leftarrow \text{Exp}(\delta\boldsymbol{\varphi})\,\mathbf{R}.$$

A whitened residual is $\mathbf{r} = \boldsymbol{\Sigma}^{-\frac{1}{2}}(\mathbf{h}(\mathbf{x}) - \mathbf{z})$ for a measurement $\mathbf{z}$ with model $\mathbf{h}$ and noise covariance $\boldsymbol{\Sigma}$, and $|\mathbf{e}|^2_{\mathbf{W}} = \mathbf{e}^\top \mathbf{W} \mathbf{e}$ for an information matrix $\mathbf{W}$. VINS-Mono subscripts the norm with a covariance instead (§2.4.1). $[\mathbf{v}]_\times$ is the skew matrix with $[\mathbf{v}]_\times \mathbf{w} = \mathbf{v} \times \mathbf{w}$, `Sophus::SO3::hat` in code. The derivations rely on eleven identities.

(I1) For $\mathbf{R} \in SO(3)$ and $\boldsymbol{\phi} \in \mathbb{R}^3$, $\mathbf{R}\,\text{Exp}(\boldsymbol{\phi})\,\mathbf{R}^\top = \text{Exp}(\mathbf{R}\boldsymbol{\phi})$, the adjoint of $SO(3)$ [8].

(I2) For small $\boldsymbol{\epsilon}$, $\text{Log}\big(\text{Exp}(\boldsymbol{\epsilon})\,\text{Exp}(\boldsymbol{\phi})\big) \approx \boldsymbol{\phi} + \mathbf{J}_l^{-1}(\boldsymbol{\phi})\,\boldsymbol{\epsilon}$, and $\mathbf{J}_l^{-1}(\boldsymbol{\phi}) \approx \mathbf{I}$ for small $\boldsymbol{\phi}$ [8]. `StationaryRows` (`src/vi_estimator/sqrt_keypoint_vio.cpp:2990-3058`) uses the same approximation.

(I3) For small $\boldsymbol{\epsilon}$, $\text{Exp}(\boldsymbol{\epsilon})\,\mathbf{u} \approx \mathbf{u} + \boldsymbol{\epsilon} \times \mathbf{u} = \mathbf{u} - [\mathbf{u}]_\times \boldsymbol{\epsilon}$.

(I4) Marginal of a Gaussian in information form. For a density proportional to $\exp(-\frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H} \boldsymbol{\delta} - \mathbf{b}^\top \boldsymbol{\delta})$ partitioned over $\boldsymbol{\delta}_\alpha$ and $\boldsymbol{\delta}_\beta$, integrating out $\boldsymbol{\delta}_\beta$ leaves the quadratic obtained by minimising over $\boldsymbol{\delta}_\beta$, because the integral of the exponential of a quadratic over a block is a constant, $(2\pi)^{n_\beta/2}\,\text{det}(\mathbf{H}_{\beta\beta})^{-1/2}$, times the exponential of its minimum over that block. The minimum is derived in §2.2.3 (`context/dynamic_marginalisation.md`, (M2)).

(I5) $\text{tr}(\mathbf{A} \mathbf{B}) = \text{tr}(\mathbf{B} \mathbf{A})$ for conforming matrices, and $\text{det}(\mathbf{A} \mathbf{B}) = \text{det}\mathbf{A}\,\text{det}\mathbf{B}$ for square ones [10].

(I6) For a symmetric positive definite $\mathbf{X}$ and a symmetric $\mathbf{S}$, the derivative of $\text{tr}(\mathbf{X} \mathbf{S})$ with respect to $\mathbf{X}$ is $\mathbf{S}$, and that of $\ln\,\text{det}\mathbf{X}$ is $\mathbf{X}^{-1}$ [10].

(I7) For Gaussians $p = \mathcal{N}(\boldsymbol{\mu}_o, \boldsymbol{\Sigma}_o)$ and $p_a = \mathcal{N}(\boldsymbol{\mu}_a, \mathbf{H}_a^{-1})$ of dimension $d$ [6, eq. 27],

$$D_\text{KL}(p\,|\,p_a) = \frac{1}{2}\Big(\text{tr}(\mathbf{H}_a \boldsymbol{\Sigma}_o) - \ln\,\text{det}(\mathbf{H}_a \boldsymbol{\Sigma}_o) + |\boldsymbol{\mu}_a - \boldsymbol{\mu}_o|^2_{\mathbf{H}_a} - d\Big).$$

(I8) For $\mathbf{R} \in SO(3)$ and $\mathbf{v} \in \mathbb{R}^3$, $[\mathbf{R} \mathbf{v}]_\times = \mathbf{R} [\mathbf{v}]_\times \mathbf{R}^\top$, because both sides map any $\mathbf{w}$ to $(\mathbf{R} \mathbf{v}) \times \mathbf{w}$, using $\mathbf{R}(\mathbf{a} \times \mathbf{b}) = (\mathbf{R} \mathbf{a}) \times (\mathbf{R} \mathbf{b})$ [8].

(I9) For vectors $\mathbf{d}_1, \dots, \mathbf{d}_N$ with mean $\bar{\mathbf{d}} = \frac{1}{N}\sum_k \mathbf{d}_k$,

$$\sum_{i < j} |\mathbf{d}_j - \mathbf{d}_i|^2 = N \sum_k |\mathbf{d}_k - \bar{\mathbf{d}}|^2.$$

Proof. Every unordered pair appears twice among ordered pairs, so the left side is $\frac{1}{2}\sum_{i, j} |\mathbf{d}_j - \mathbf{d}_i|^2$. Expanding the square gives $\frac{1}{2}\big(2N \sum_k |\mathbf{d}_k|^2 - 2|\sum_k \mathbf{d}_k|^2\big) = N \sum_k |\mathbf{d}_k|^2 - N^2 |\bar{\mathbf{d}}|^2$. Expanding the right side gives $N\big(\sum_k |\mathbf{d}_k|^2 - 2\bar{\mathbf{d}}^\top \sum_k \mathbf{d}_k + N |\bar{\mathbf{d}}|^2\big) = N \sum_k |\mathbf{d}_k|^2 - N^2 |\bar{\mathbf{d}}|^2$, the same.

(I10) Sherman–Morrison [10]. Write $\boldsymbol{\iota}_n$ for the all ones vector of length $n$ and $\mathbf{K}_n = \boldsymbol{\iota}_n \boldsymbol{\iota}_n^\top$ for the $n \times n$ all ones matrix, whose every entry is $1$. Both keep these meanings throughout, the subscript giving the size, which is $N$ or $N - 1$ in §3.5. Since $\boldsymbol{\iota}_n^\top \boldsymbol{\iota}_n = n$, the all ones matrix satisfies $\mathbf{K}_n^2 = \boldsymbol{\iota}_n (\boldsymbol{\iota}_n^\top \boldsymbol{\iota}_n) \boldsymbol{\iota}_n^\top = n\mathbf{K}_n$. For the $n \times n$ identity $\mathbf{I}$,

$$(\mathbf{I} + \mathbf{K}_n)^{-1} = \mathbf{I} - \frac{\mathbf{K}_n}{1 + n},$$

because the product $(\mathbf{I} + \mathbf{K}_n)(\mathbf{I} - \frac{\mathbf{K}_n}{1 + n}) = \mathbf{I} + \mathbf{K}_n - \frac{\mathbf{K}_n + n\mathbf{K}_n}{1 + n} = \mathbf{I}$. Note, 2026-11-20. The all ones vector and matrix were previously written $\mathbf{1}$ and $\mathbf{1} \mathbf{1}^\top$, which `latex2text` renders as the scalar $1$.

(I11) Graph Laplacians [12]. A graph on $N$ nodes with unit edge weights has the Laplacian $\mathbf{L} = \mathbf{D}^\top \mathbf{D}$, with $\mathbf{D}$ its incidence matrix, one row per edge with $+1$ and $-1$ in the columns of its two nodes. The star with one centre has the eigenvalues $0$, $1$ with multiplicity $N - 2$ and $N$, the last with eigenvector $(N - 1)$ at the centre and $-1$ at every other node. The complete graph has $\mathbf{L} = N\mathbf{I} - \mathbf{K}_N$, with the eigenvalues $0$ and $N$ with multiplicity $N - 1$. Both have $\boldsymbol{\iota}_N$ as their only null vector when connected. Both spectra are derived in §3.5.2 Steps 1 to 7.

### 2.2 Common machinery

#### 2.2.1 From probability to least squares

Each measurement is modelled as $\mathbf{z}_i = \mathbf{h}_i(\mathbf{x}) + \mathbf{n}_i$ with $\mathbf{n}_i \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma}_i)$, independent across $i$. Bayes' rule gives the posterior

$$p(\mathbf{x}\,|\,\mathbf{z}) \propto p(\mathbf{x}) \prod_i \exp\Big(-\frac{1}{2}|\mathbf{h}_i(\mathbf{x}) - \mathbf{z}_i|^2_{\boldsymbol{\Sigma}_i^{-1}}\Big).$$

Taking the negative logarithm turns the product into a sum, and with the whitened residuals of §2.1 the maximum a posteriori estimate is

$$\mathbf{x}^\ast = \arg\min_\mathbf{x} \sum_i \frac{1}{2}|\mathbf{r}_i(\mathbf{x})|^2.$$

A robust loss $\rho$ replaces $\frac{1}{2}|\mathbf{r}|^2$ for terms that may hold outliers, the Huber loss in every method below.

#### 2.2.2 Gauss Newton on a manifold

States live on manifolds, so the step is a tangent increment $\boldsymbol{\delta}$ applied by $\mathbf{x} \boxplus \boldsymbol{\delta}$, the left perturbation of §2.1 in Basalt. Linearising each residual at $\boldsymbol{\delta} = \mathbf{0}$ gives $\mathbf{r}(\mathbf{x} \boxplus \boldsymbol{\delta}) \approx \mathbf{r} + \mathbf{J}\boldsymbol{\delta}$ with $\mathbf{J} = \frac{\partial\mathbf{r}}{\partial\boldsymbol{\delta}}$. Stacking all residuals, the model cost is

$$F(\boldsymbol{\delta}) = \frac{1}{2}|\mathbf{r} + \mathbf{J}\boldsymbol{\delta}|^2 = \frac{1}{2}\mathbf{r}^\top \mathbf{r} + \boldsymbol{\delta}^\top \mathbf{J}^\top \mathbf{r} + \frac{1}{2}\boldsymbol{\delta}^\top \mathbf{J}^\top \mathbf{J} \boldsymbol{\delta}.$$

Setting its gradient $\mathbf{J}^\top \mathbf{J} \boldsymbol{\delta} + \mathbf{J}^\top \mathbf{r}$ to zero gives the normal equations

$$\mathbf{H}\boldsymbol{\delta} = -\mathbf{b},$$

with $\mathbf{H} = \mathbf{J}^\top \mathbf{J}$ and $\mathbf{b} = \mathbf{J}^\top \mathbf{r}$. Levenberg–Marquardt solves $(\mathbf{H} + \lambda\,\text{diag}(\mathbf{H}))\,\boldsymbol{\delta} = -\mathbf{b}$ and adapts $\lambda$ to the ratio of actual and predicted decrease (`context/vio_opt.md` §8 to §10).

#### 2.2.3 Marginalisation by the Schur complement

A sliding window removes old variables $\boldsymbol{\delta}_\beta$ while keeping their information on the remaining $\boldsymbol{\delta}_\alpha$. Partitioning the model cost of §2.2.2, up to a constant,

$$F = \frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}_{\alpha\alpha} \boldsymbol{\delta}_\alpha + \boldsymbol{\delta}_\alpha^\top \mathbf{H}_{\alpha\beta} \boldsymbol{\delta}_\beta + \frac{1}{2}\boldsymbol{\delta}_\beta^\top \mathbf{H}_{\beta\beta} \boldsymbol{\delta}_\beta + \mathbf{b}_\alpha^\top \boldsymbol{\delta}_\alpha + \mathbf{b}_\beta^\top \boldsymbol{\delta}_\beta.$$

Step 1. The derivative with respect to $\boldsymbol{\delta}_\beta$ vanishes at the minimiser for fixed $\boldsymbol{\delta}_\alpha$,

$$\mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha + \mathbf{H}_{\beta\beta} \boldsymbol{\delta}_\beta + \mathbf{b}_\beta = \mathbf{0},$$

$$\boldsymbol{\delta}_\beta^\ast = -\mathbf{H}_{\beta\beta}^{-1}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha).$$

Step 2. Substituting $\boldsymbol{\delta}_\beta^\ast$, and using Step 1 to write the three terms in $\boldsymbol{\delta}_\beta$ as $\frac{1}{2}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)^\top \boldsymbol{\delta}_\beta^\ast$,

$$F^\ast(\boldsymbol{\delta}_\alpha) = \frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}_{\alpha\alpha} \boldsymbol{\delta}_\alpha + \mathbf{b}_\alpha^\top \boldsymbol{\delta}_\alpha - \frac{1}{2}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)^\top \mathbf{H}_{\beta\beta}^{-1}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha).$$

Step 3. Collecting the quadratic and linear terms in $\boldsymbol{\delta}_\alpha$,

$$\mathbf{H}' = \mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{H}_{\beta\alpha},$$

$$\mathbf{b}' = \mathbf{b}_\alpha - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{b}_\beta.$$

By (I4) this quadratic is also the exact marginal of the linearised Gaussian. Every surveyed method computes this prior [1, 3, 5, 6, 7]. Basalt keeps it in square root form, $\mathbf{J}_m^\top \mathbf{J}_m = \mathbf{H}'$ and $\mathbf{J}_m^\top \mathbf{r}_m = \mathbf{b}'$, so that up to a constant $F^\ast = \frac{1}{2}|\mathbf{J}_m \boldsymbol{\delta}_\alpha + \mathbf{r}_m|^2$. The prior is a function of the offset from the point $\bar{\mathbf{x}}_\alpha$ at which it was linearised, so in later solves it reads $\frac{1}{2}|\mathbf{J}_m(\boldsymbol{\delta} + \mathbf{x}) + \mathbf{r}_m|^2$, with $\boldsymbol{\delta}$ the accumulated offset and $\mathbf{x}$ the new step (`context/vio_opt.md` §5.3).

#### 2.2.4 First estimate Jacobians

Let $\mathbf{n}(\mathbf{x})$ be a direction along which the nonlinear cost is flat, for example a global translation. At its linearisation point the prior satisfies $\mathbf{J}_m \mathbf{n}(\bar{\mathbf{x}}) = \mathbf{0}$. A later factor linearised at another point $\mathbf{x}$ satisfies $\mathbf{J}\,\mathbf{n}(\mathbf{x}) = \mathbf{0}$ instead. When $\mathbf{n}(\mathbf{x}) \neq \mathbf{n}(\bar{\mathbf{x}})$ the sum $\mathbf{J}_m^\top \mathbf{J}_m + \mathbf{J}^\top \mathbf{J}$ has no common null vector, and the solver gains spurious information along a direction no measurement observes. Basalt, VI-DSO and DM-VIO therefore freeze every variable that enters a prior and evaluate all its Jacobians at $\bar{\mathbf{x}}$, moving only the offset $\boldsymbol{\delta}$ (`context/dynamic_marginalisation.md` §3.1.3). Writing a new value into a frozen state outside the solve, as the current overwrite does, moves $\boldsymbol{\delta}$ with no factor in the cost asking for it.

#### 2.2.5 Inertial preintegration

Over an interval $[i, j]$ with gyroscope and accelerometer samples $\tilde{\boldsymbol{\omega}}_k$ and $\tilde{\mathbf{a}}_k$, biases $\mathbf{b}_g$ and $\mathbf{b}_a$, sample period $\Delta t$ and gravity $\mathbf{g}$, the kinematics integrate to

$$\mathbf{R}_j = \mathbf{R}_i \prod_k \text{Exp}\big((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g)\Delta t\big),$$

$$\mathbf{v}_j = \mathbf{v}_i + \mathbf{g}\Delta t_{ij} + \sum_k \mathbf{R}_k(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t,$$

$$\mathbf{p}_j = \mathbf{p}_i + \mathbf{v}_i\Delta t_{ij} + \frac{1}{2}\mathbf{g}\Delta t_{ij}^2 + \sum_k \Big(\big(\mathbf{v}_k - \mathbf{v}_i - \mathbf{g}(t_k - t_i)\big)\Delta t + \frac{1}{2}\mathbf{R}_k(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t^2\Big).$$

Step 1. Every sum depends on the unknown $\mathbf{R}_i$ through $\mathbf{R}_k = \mathbf{R}_i \Delta\mathbf{R}_{ik}$. Multiplying each equation by $\mathbf{R}_i^\top$ and moving the state terms to the left removes that dependence and defines increments computed from samples alone [9],

$$\Delta\mathbf{R}_{ij} = \mathbf{R}_i^\top \mathbf{R}_j = \prod_k \text{Exp}\big((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g)\Delta t\big),$$

$$\Delta\mathbf{v}_{ij} = \mathbf{R}_i^\top(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\Delta t_{ij}) = \sum_k \Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t,$$

$$\Delta\mathbf{p}_{ij} = \mathbf{R}_i^\top\Big(\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i\Delta t_{ij} - \frac{1}{2}\mathbf{g}\Delta t_{ij}^2\Big) = \sum_k \Big(\Delta\mathbf{v}_{ik}\Delta t + \frac{1}{2}\Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t^2\Big).$$

Step 2. The increments depend on the biases, which change during optimisation. Re-integrating is avoided by a first order expansion about the bias $\bar{\mathbf{b}}$ of the integration, with $\delta\mathbf{b} = \mathbf{b} - \bar{\mathbf{b}}$ and Jacobians propagated sample by sample [9, 1],

$$\Delta\mathbf{R}_{ij}(\mathbf{b}_g) \approx \Delta\mathbf{R}_{ij}\,\text{Exp}(\mathbf{J}^g_{\Delta R}\,\delta\mathbf{b}_g),$$

$$\Delta\mathbf{v}_{ij}(\mathbf{b}) \approx \Delta\mathbf{v}_{ij} + \mathbf{J}^g_{\Delta v}\,\delta\mathbf{b}_g + \mathbf{J}^a_{\Delta v}\,\delta\mathbf{b}_a,$$

$$\Delta\mathbf{p}_{ij}(\mathbf{b}) \approx \Delta\mathbf{p}_{ij} + \mathbf{J}^g_{\Delta p}\,\delta\mathbf{b}_g + \mathbf{J}^a_{\Delta p}\,\delta\mathbf{b}_a.$$

Step 3. The residual compares the state side of Step 1 with the measured side of Step 2,

$$\mathbf{e}_R = \text{Log}\Big(\big(\Delta\mathbf{R}_{ij}\,\text{Exp}(\mathbf{J}^g_{\Delta R}\,\delta\mathbf{b}_g)\big)^\top \mathbf{R}_i^\top \mathbf{R}_j\Big),$$

$$\mathbf{e}_v = \mathbf{R}_i^\top(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\Delta t_{ij}) - \Delta\mathbf{v}_{ij}(\mathbf{b}),$$

$$\mathbf{e}_p = \mathbf{R}_i^\top\Big(\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i\Delta t_{ij} - \frac{1}{2}\mathbf{g}\Delta t_{ij}^2\Big) - \Delta\mathbf{p}_{ij}(\mathbf{b}),$$

whitened by the propagated preintegration covariance, and a bias random walk residual $\mathbf{e}_b = \mathbf{b}_j - \mathbf{b}_i$ whitened by the random walk density times $\Delta t_{ij}$. Visual inertial ORB-SLAM writes exactly these [1, eq. 6], VINS-Mono writes the same quantities as $\hat{\boldsymbol{\alpha}}$, $\hat{\boldsymbol{\beta}}$, $\hat{\boldsymbol{\gamma}}$ with a quaternion rotation error and its own sign convention for $\mathbf{g}^w$ [3], and Basalt's form is `doc/VIO.md` §3.1.1.

### 2.3 Map reuse in visual inertial ORB-SLAM and ORB-SLAM3

#### 2.3.1 Residuals

A map point $\mathbf{X}^k_W$ projects into frame $j$ through the body to camera transform $(\mathbf{R}_{CB}, \mathbf{p}_CB)$ [1, eq. 5],

$$\mathbf{X}^k_C = \mathbf{R}_{CB} \mathbf{R}^{j\top}_{WB}(\mathbf{X}^k_W - \mathbf{p}^j) + \mathbf{p}_CB,$$

$$E_\text{proj}(k, j) = \rho\Big(|\mathbf{x}^k - \pi(\mathbf{X}^k_C)|^2_{\boldsymbol{\Sigma}_k}\Big),$$

with $\mathbf{x}^k$ the keypoint, $\pi$ the projection and $\boldsymbol{\Sigma}_k$ an information matrix set by the keypoint's pyramid scale. The inertial term stacks the residuals of §2.2.5 [1, eq. 6],

$$E_\text{IMU}(i, j) = \rho\Big(|[\mathbf{e}_R; \mathbf{e}_v; \mathbf{e}_p]|^2_{\boldsymbol{\Sigma}_I}\Big) + \rho\Big(|\mathbf{e}_b|^2_{\boldsymbol{\Sigma}_R}\Big).$$

#### 2.3.2 Tracking after a map change

When local mapping has just changed the map, frame $j$ is optimised alone against the last keyframe $i$ [1],

$$\theta = \{\mathbf{R}^j_{WB}, \mathbf{p}^j, \mathbf{v}^j, \mathbf{b}^j_g, \mathbf{b}^j_a\},$$

$$\theta^\ast = \arg\min_\theta \Big(\sum_k E_\text{proj}(k, j) + E_\text{IMU}(i, j)\Big).$$

Keyframe $i$ and the map points are not in $\theta$, so they enter as constants, and the solution is the frame's state conditioned on the map as it now stands.

#### 2.3.3 Tracking without a map change

Otherwise frame $j+1$ is optimised together with frame $j$, linked by $E_\text{IMU}(j, j+1)$, with a prior on $j$ from the previous solve [1, eq. 8],

$$\theta^\ast = \arg\min_\theta \Big(\sum_k E_\text{proj}(k, j+1) + E_\text{IMU}(j, j+1) + E_\text{prior}(j)\Big),$$

$$E_\text{prior}(j) = \rho\Big(|[\mathbf{e}_R; \mathbf{e}_v; \mathbf{e}_p; \mathbf{e}_b]|^2_{\boldsymbol{\Sigma}_p}\Big),$$

with $\mathbf{e}_R = \text{Log}(\bar{\mathbf{R}}^{j\top}_{WB} \mathbf{R}^j_{WB})$, $\mathbf{e}_v = \bar{\mathbf{v}}^j - \mathbf{v}^j$, $\mathbf{e}_p = \bar{\mathbf{p}}^j - \mathbf{p}^j$ and $\mathbf{e}_b = \bar{\mathbf{b}}^j - \mathbf{b}^j$, the bars marking the previous estimate. After the joint solve frame $j$ is marginalised and the resulting estimate and Hessian serve as the prior of the next solve [1]. The paper does not write the formula. Applying §2.2.3 to the Hessian over $[\boldsymbol{\delta}_j; \boldsymbol{\delta}_{j+1}]$ gives

$$\boldsymbol{\Sigma}_p = \mathbf{H}_{j+1,j+1} - \mathbf{H}_{j+1,j} \mathbf{H}_{j,j}^{-1} \mathbf{H}_{j,j+1},$$

which is Step 3 of §2.2.3 with $\alpha = j+1$ and $\beta = j$. This prior then serves the next frame, and the scheme is a two state sliding window filter.

#### 2.3.4 Why the prior is discarded at a map change

$\boldsymbol{\Sigma}_p$ summarises the reprojections of the map points and the link to keyframe $i$ at the values they held when it was formed. The paper states only that the prior is not valid after the map change [1]. The reason follows from §2.2.3. Once local mapping moves those points and keyframes, keeping the prior would count the same measurements twice, once through the prior at the old map and once through $E_\text{proj}$ at the new map, at two inconsistent values. Dropping the prior and restarting from the new map, treated as exact, avoids both. ORB-SLAM3 keeps the scheme and notes that fixing states in tracking and local BA could bias the solution [2].

#### 2.3.5 Assessment

The map is authoritative and the cost is tiny. The price is that all inertial and visual history outside the map is discarded at every map change. That suits a two state tracker. It does not suit a window of tens of states whose prior holds the gravity and scale period and the stationary warm start.

### 2.4 Tightly coupled relocalisation in VINS-Mono

#### 2.4.1 Window cost

VINS-Mono estimates the state [3, eq. 21]

$$\mathcal{X} = [\mathbf{x}_0, \mathbf{x}_1, \dots, \mathbf{x}_n, \mathbf{x}^b_c, \lambda_0, \lambda_1, \dots, \lambda_m],$$

$$\mathbf{x}_k = [\mathbf{p}^w_{b_k}, \mathbf{v}^w_{b_k}, \mathbf{q}^w_{b_k}, \mathbf{b}_a, \mathbf{b}_g],$$

$$\mathbf{x}^b_c = [\mathbf{p}^b_c, \mathbf{q}^b_c],$$

and minimises [3, eq. 22]

$$\min_\mathcal{X} \Big\{ |\mathbf{r}_p - \mathbf{H}_p \mathcal{X}|^2 + \sum_{k \in \mathcal{B}} |\mathbf{r}_\mathcal{B}(\hat{\mathbf{z}}^{b_k}_{b_{k+1}}, \mathcal{X})|^2_{\mathbf{P}^{b_k}_{b_{k+1}}} + \sum_{(l, j) \in \mathcal{C}} \rho\big(|\mathbf{r}_\mathcal{C}(\hat{\mathbf{z}}^{c_j}_l, \mathcal{X})|^2_{\mathbf{P}^{c_j}_l}\big) \Big\}.$$

The notation reads as follows, in the order of the equations.

1. Superscripts and subscripts name frames. $w$ is the world, $b_k$ the IMU body when the $k$-th image of the window was taken, $c_j$ the camera of the $j$-th image, and $\mathbf{a}^w_{b_k}$ is a quantity of body $b_k$ expressed in the world. $\mathcal{X}$ is the full state, which the minimisation ranges over.
2. $\mathbf{x}_k$ is the IMU state of image $k$, namely its position $\mathbf{p}^w_{b_k}$, velocity $\mathbf{v}^w_{b_k}$ and orientation as a unit quaternion $\mathbf{q}^w_{b_k}$, with the accelerometer and gyroscope biases $\mathbf{b}_a$ and $\mathbf{b}_g$. $n$ is the number of keyframes in the window, so it holds $n + 1$ states. These correspond to Basalt's `frame_states`.
3. $\mathbf{x}^b_c$ is the camera to IMU extrinsic, translation $\mathbf{p}^b_c$ and rotation $\mathbf{q}^b_c$. VINS-Mono estimates it online, whereas Basalt holds it constant as `calib.T_i_c`.
4. $\lambda_l$ is the inverse depth of feature $l$ along its ray in the frame of its first observation, and $m$ is the number of features. Basalt parametrises a landmark the same way, a host keyframe, a bearing and an inverse distance (`Keypoint::direction`, `Keypoint::inv_dist`, `include/basalt/utils/ba_utils.h:148-149`).
5. $\{\mathbf{r}_p, \mathbf{H}_p\}$ is the prior from marginalisation [3]. It is the square root form of §2.2.3, with $\mathbf{H}_p$ playing $\mathbf{J}_m$ and $\mathbf{r}_p$ playing $-\mathbf{r}_m$, and $\mathcal{X}$ in the prior term read as the tangent offset from the prior's linearisation point, as in §2.2.3. The paper writes $\mathcal{X}$ for brevity. The term is unweighted because the prior is already whitened.
6. $\mathcal{B}$ is the set of all IMU measurements, one preintegrated interval per consecutive pair $(b_k, b_{k+1})$. $\hat{\mathbf{z}}^{b_k}_{b_{k+1}}$ is the preintegrated measurement of §2.2.5, $\hat{\boldsymbol{\alpha}}$, $\hat{\boldsymbol{\beta}}$, $\hat{\boldsymbol{\gamma}}$, and $\mathbf{r}_\mathcal{B}$ the fifteen dimensional residual of §2.2.5 with a quaternion rotation error.
7. $\mathbf{P}^{b_k}_{b_{k+1}}$ is the covariance of the preintegrated measurement, propagated from zero at $b_k$ sample by sample through the first order error dynamics $\mathbf{P} \leftarrow (\mathbf{I} + \mathbf{F}_t \delta t) \mathbf{P} (\mathbf{I} + \mathbf{F}_t \delta t)^\top + (\mathbf{G}_t \delta t) \mathbf{Q} (\mathbf{G}_t \delta t)^\top$, with $\mathbf{Q}$ the diagonal noise covariance [3, eq. 10].
8. $\mathcal{C}$ is the set of features observed at least twice in the window, enumerated as pairs $(l, j)$ of a feature and an image other than its first. $\hat{\mathbf{z}}^{c_j}_l$ is the observed pixel and $\mathbf{r}_\mathcal{C}$ the two dimensional residual below. $\mathbf{P}^{c_j}_l$ is the covariance of a fixed pixel noise mapped into the tangent plane of the unit sphere [3].
9. A norm subscripted with a covariance is the Mahalanobis norm, $|\mathbf{e}|^2_\mathbf{P} = \mathbf{e}^\top \mathbf{P}^{-1} \mathbf{e}$, the usual reading of the paper's "Mahalanobis norm" [3]. In the notation of §2.1 this is $|\mathbf{e}|^2_{\mathbf{P}^{-1}}$, so each term is the squared whitened residual of §2.2.1.
10. $\rho$ is the Huber loss on the squared norm $s$. The paper prints eq. 23 as $\rho(s) = 1$ for $s \geq 1$ and $2\sqrt{s} - 1$ for $s < 1$, which is not a Huber loss and is evidently a typesetting error. The Huber loss in this form, and the one Ceres implements, is $\rho(s) = s$ for $s \leq 1$ and $2\sqrt{s} - 1$ for $s > 1$, quadratic for inliers and linear in $|\mathbf{e}|$ for outliers. Only the visual term is robustified, because only feature associations can be gross outliers.

The significance of the formulation follows from §2.2.1. Each term is the negative logarithm of one Gaussian factor of the posterior, the prior factor from everything the window has forgotten, one inertial factor per interval and one visual factor per observation, so the minimiser is the maximum a posteriori estimate of $\mathcal{X}$. It has the same three part structure as Basalt's window cost (`doc/VIO.md` §3), which is why a constant from an external map can enter either estimator as a fourth sum without touching the other three. The visual residual places feature $l$, first seen in frame $i$ at pixel $(u^{c_i}_l, v^{c_i}_l)$, into frame $j$,

$$\mathcal{P}^{c_j}_l = \mathbf{R}^c_b\Big(\mathbf{R}^{b_j}_w\big(\mathbf{R}^w_{b_i}(\mathbf{R}^b_c \frac{1}{\lambda_l}\pi_c^{-1}(u^{c_i}_l, v^{c_i}_l) + \mathbf{p}^b_c) + \mathbf{p}^w_{b_i} - \mathbf{p}^w_{b_j}\big) - \mathbf{p}^b_c\Big),$$

and compares bearings on the unit sphere in the tangent plane of the observed ray $\hat{\bar{\mathcal{P}}}^{c_j}_l = \pi_c^{-1}(\hat{u}^{c_j}_l, \hat{v}^{c_j}_l)$, spanned by $\mathbf{b}_1$ and $\mathbf{b}_2$,

$$\mathbf{r}_\mathcal{C} = [\mathbf{b}_1\ \mathbf{b}_2]^\top \Big(\hat{\bar{\mathcal{P}}}^{c_j}_l - \frac{\mathcal{P}^{c_j}_l}{|\mathcal{P}^{c_j}_l|}\Big).$$

#### 2.4.2 Relocalisation

When a loop is detected, the features retrieved in a loop frame $v$ are added as visual residuals whose target pose is the loop frame's stored pose $(\hat{\mathbf{q}}^w_v, \hat{\mathbf{p}}^w_v)$, taken from the pose graph or, at the first relocalisation, from past odometry [3, eq. 26],

$$\min_\mathcal{X} \Big\{ \cdots + \sum_{(l, v) \in \mathcal{L}} \rho\big(|\mathbf{r}_\mathcal{C}(\hat{\mathbf{z}}^v_l, \mathcal{X}, \hat{\mathbf{q}}^w_v, \hat{\mathbf{p}}^w_v)|^2_{\mathbf{P}^{c_v}_l}\big) \Big\},$$

with the dots standing for the three terms of §2.4.1. The effect follows from §2.2.2 in three steps.

Step 1. In $\mathcal{P}^{c_v}_l$ the target pose $(\mathbf{R}^{b_j}_w, \mathbf{p}^w_{b_j})$ is replaced by the constant $(\hat{\mathbf{R}}^{b_v}_w, \hat{\mathbf{p}}^w_v)$, so the residual has Jacobian columns only for the host state $\mathbf{x}_i$, the extrinsic and $\lambda_l$. The state dimension is unchanged.

Step 2. The term adds $\mathbf{J}^\top \mathbf{J}$ to the host block. Because the target is an absolute pose in the map's world, the residual changes under a global translation or yaw of the window, so it observes the four directions the window cannot observe alone, and the solve pulls the window into the map's world.

Step 3. Several loop frames add their terms independently, which gives multi view constraints on the same window [3].

#### 2.4.3 The 4 DOF pose graph

Gravity makes roll and pitch observable to visual inertial odometry, so drift accumulates only in position and yaw [3]. The pose graph therefore optimises $(\mathbf{p}^w_i, \psi_i)$ per keyframe and holds roll $\hat{\phi}_i$ and pitch $\hat{\theta}_i$ at the odometry's values. Its edge is derived from a relative pose in three steps.

Step 1. The full relative pose of $j$ in $i$ is $\mathbf{T}_i^{-1} \mathbf{T}_j = (\mathbf{R}_i^\top \mathbf{R}_j, \mathbf{R}_i^\top(\mathbf{p}_j - \mathbf{p}_i))$.

Step 2. Restricting $\mathbf{R}_i$ to $\mathbf{R}(\hat{\phi}_i, \hat{\theta}_i, \psi_i)$ keeps the translation part a function of the free yaw, and the relative rotation reduces to the yaw difference.

Step 3. The odometry's values at the time the edge is created give the measurements, $\hat{\mathbf{p}}^i_{ij} = \hat{\mathbf{R}}^{w\top}_i(\hat{\mathbf{p}}^w_j - \hat{\mathbf{p}}^w_i)$ and $\hat{\psi}_{ij} = \hat{\psi}_j - \hat{\psi}_i$, and the residual is [3, eq. 28]

$$\mathbf{r}_{ij} = \begin{bmatrix} \mathbf{R}(\hat{\phi}_i, \hat{\theta}_i, \psi_i)^\top (\mathbf{p}^w_j - \mathbf{p}^w_i) - \hat{\mathbf{p}}^i_{ij} \\ \psi_j - \psi_i - \hat{\psi}_{ij} \end{bmatrix}.$$

Sequential edges $\mathcal{S}$ connect consecutive keyframes and loop edges $\mathcal{L}$ come from relocalisation, robustified because a wrong loop is possible [3, eq. 29],

$$\min_{\mathbf{p}, \psi} \Big\{ \sum_{(i, j) \in \mathcal{S}} |\mathbf{r}_{ij}|^2 + \sum_{(i, j) \in \mathcal{L}} \rho\big(|\mathbf{r}_{ij}|^2\big) \Big\}.$$

#### 2.4.4 Assessment

Relocalisation keeps the prior and the window's dimension, and the constants enter as ordinary residuals. It needs feature correspondences against the map and treats the map poses as exact. Its absolute target is right for VINS-Mono, whose pose graph defines a global frame, and wrong for a mapper whose absolute position and yaw are free (§2.7). The 4 DOF edge shows the other half of the lesson, a correction should act only on the directions that drift.

### 2.5 Relative pose errors from marginalised observations in OKVIS2

#### 2.5.1 Estimator cost

OKVIS2 minimises reprojection errors $\mathbf{e}_\text{r}$, IMU errors $\mathbf{e}_\text{s}$ and relative pose errors $\mathbf{e}_\text{p}$ over a bounded real time graph [5],

$$c(\mathbf{x}) = \frac{1}{2}\sum_i \sum_k \sum_j \rho\big(|\mathbf{e}^{i,j,k}_\text{r}|^2_{\mathbf{W}_\text{r}}\big) + \frac{1}{2}\sum_k |\mathbf{e}^k_\text{s}|^2_{\mathbf{W}^k_\text{s}} + \frac{1}{2}\sum_r \sum_c |\mathbf{e}^{r,c}_\text{p}|^2_{\mathbf{W}^{r,c}_\text{p}}.$$

#### 2.5.2 Derivation of the relative pose error

The edge between a reference frame $r$ and a frame $c$ summarises the landmarks they observe in common. Let $\mathbf{p}$ be the relative pose $\mathbf{T}_{S^r S^c} = \mathbf{T}_{WS^r}^{-1} \mathbf{T}_{WS^c}$, written as a translation and a quaternion, and express each common landmark $\mathbf{l}^j$ in frame $r$ [5].

Step 1. Both frames' reprojection errors depend only on $\mathbf{p}$ and the landmarks, because the landmark lives in frame $r$,

$$\mathbf{e}^{i,j,r}_\text{r} = \tilde{\mathbf{z}}^{i,j,r} - \mathbf{h}(\mathbf{T}_{SC_i}^{-1}\,{}_{S^r}\mathbf{l}^j),$$

$$\mathbf{e}^{i,j,c}_\text{r} = \tilde{\mathbf{z}}^{i,j,c} - \mathbf{h}(\mathbf{T}_{SC_i}^{-1} \mathbf{T}_{S^c S^r}\,{}_{S^r}\mathbf{l}^j).$$

Step 2. Linearising their sum at the current estimate $\tilde{\mathbf{p}}$ gives a system whose landmark blocks are mutually independent, and the Schur complement of §2.2.3 over all landmarks gives [5]

$$\mathbf{H}^\ast = \mathbf{H}_{\text{p},\text{p}} - \sum_j \mathbf{H}_{\text{p},j} \mathbf{H}_{jj}^{+} \mathbf{H}_{\text{p},j}^\top,$$

$$\mathbf{b}^\ast = \mathbf{b}_\text{p} - \sum_j \mathbf{H}_{\text{p},j} \mathbf{H}_{jj}^{+} \mathbf{b}_j.$$

Step 3. A factor of the form $\mathbf{e}_\text{p} = \mathbf{e}_{\text{p},0} + \mathbf{p} \boxminus \tilde{\mathbf{p}}$ with information $\mathbf{W}$ has, at $\boldsymbol{\delta} = \mathbf{p} \boxminus \tilde{\mathbf{p}}$, the cost

$$\frac{1}{2}(\mathbf{e}_{\text{p},0} + \boldsymbol{\delta})^\top \mathbf{W}(\mathbf{e}_{\text{p},0} + \boldsymbol{\delta}) = \frac{1}{2}\mathbf{e}_{\text{p},0}^\top \mathbf{W} \mathbf{e}_{\text{p},0} + \boldsymbol{\delta}^\top \mathbf{W} \mathbf{e}_{\text{p},0} + \frac{1}{2}\boldsymbol{\delta}^\top \mathbf{W} \boldsymbol{\delta}.$$

Step 4. Matching its quadratic and linear terms with $\frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H}^\ast \boldsymbol{\delta} + \mathbf{b}^{\ast\top} \boldsymbol{\delta}$ in the sign convention of §2.2.2 gives

$$\mathbf{W}^{r,c}_\text{p} = \mathbf{H}^\ast,$$

$$\mathbf{e}^{r,c}_{\text{p},0} = \mathbf{H}^{\ast +} \mathbf{b}^\ast.$$

OKVIS2 writes the second with a minus sign, because its normal equations are $\mathbf{H}^\ast \boldsymbol{\delta} = \mathbf{b}^\ast$ [5]. The factor is nonlinear in $\mathbf{p}$, so it can be relinearised later, and it depends only on the relative pose, so a global motion of both frames leaves it unchanged.

#### 2.5.3 Loop closure and synchronisation

At a loop closure the pose graph edges near the loop are converted back into landmarks and observations, the loop error is distributed, a full graph optimisation runs asynchronously, and its result is imported into the real time estimator with the states created meanwhile realigned [5]. The paper gives these steps in prose only.

#### 2.5.4 Assessment

OKVIS2 shows that a summary of visual information between two frames is naturally a relative pose factor with a full information matrix. Its synchronisation replaces states wholesale, which presupposes an estimator built around it.

### 2.6 Relative marginalisation in ICE-BA

This section is read from the paper's §5 with eq. 15 to 21 and Fig. 5, read on 2026-11-07 and summarised in `context/dynamic_marginalisation.md` §6.12 [4], and from the authors' released code [11], `Backend/Geometry/CameraPrior.h` and `Backend/BundleAdjustment/LocalBundleAdjustor.cpp`, `LocalBundleAdjustorGN.cpp`, `GlobalBundleAdjustor.cpp` and `GlobalBundleAdjustorGN.cpp`. The PDF returned HTTP 403 on 2026-11-17 and 2026-11-18. The code was read through a summarising fetch whose quotes are capped in length and which covered 100000 of the 165285 characters of `LocalBundleAdjustor.cpp` and of the 108397 of `GlobalBundleAdjustorGN.cpp`, so a statement below that rests on the absence of a call is marked as such. Note, 2026-11-19. This paragraph previously stated that the paper could not be read and that the section rests on the code and the abstract alone, which overlooked the reading of §5 recorded in the context store on 2026-11-07. Note, 2026-11-18. This section previously derived the principle without any ICE-BA equation and stated that the form of its relative prior was unverified. The code now settles both.

#### 2.6.1 Architecture

ICE-BA runs a local BA over a temporal sliding window of local frames and a global BA over all keyframes in parallel threads [4]. The two exchange information in both directions. The local BA sends the global BA the information of the frames it marginalises as relative priors, and the global BA sends back keyframe poses it has corrected, for example after a loop closure. The abstract states that relative marginalisation resolves the conflict between the window's marginalisation and the global loop constraints [4]. This is the closest precedent in the survey to the present problem, an inner estimator with a marginalisation prior that must accept corrections from an outer map over more keyframes.

The variables are of three kinds [11]. Local frames are the recent frames of the sliding window, at most `LBA_MAX_LOCAL_FRAMES` of them, each with a camera pose in `m_CsLF` and a motion, its velocity and biases, in `m_CmsLF`. Keyframes are a subset of the frames, with poses in `m_CsKF`. Every feature is parametrised by its inverse depth in the keyframe that first observed it, its source. Three operations remove variables from a linear system, and they must be told apart, because only one of them leaves a prior behind.

1. Elimination of the depths at every iteration. Both solvers form the reduced camera system by the Schur complement of §2.2.3 over the depths, `UpdateSchurComplement` with `FTR::Factor::Full::Marginalize` and `FTR::Factor::Full::Source::Marginalize` in `GlobalBundleAdjustorGN.cpp`, solve it, and recover each depth by back substitution. Nothing is discarded, because every depth stays a variable and its factors are relinearised at the next iteration. This is the incremental part of ICE-BA. A factor's contribution to the reduced system is recomputed only when one of its variables changed, `if (!uc) { continue; }` in `UpdateFactorsPriorCameraPose`, and otherwise updated by the difference from its stored value, the `dArr`, `dArm` and `dAmm` of `UpdateFactorsPriorCameraMotion` [11]. The abstract's "incremental" refers to this reuse [4].
2. Marginalisation of a local frame. When the window is full, `MarginalizeLocalFrame` removes its oldest local frame, its camera and its motion. Every term that involves them, its inertial factor to the next local frame, the blocks of its feature measurements and the prior it already carries, is folded by the Schur complement into a joint prior over the states that remain. This is the only operation that leaves a prior. The prior is over cameras and motion, not over landmarks. In the paper's terms it is a Gaussian over the gravity direction in the reference keyframe, the state of the next local frame relative to that keyframe and the relative poses of the keyframes the departing frame observed [4, §5, eq. 17 and 18]. §2.6.2 gives its content, its construction step by step and its use.
3. Removal of a keyframe from a prior. When the map deletes a keyframe, every prior that names it drops its rows and columns, `Pose::DeleteKeyFrame` with `Pose::Erase`. This removes a variable without leaving a prior, and §2.6.2 shows that it conditions the prior on the deleted keyframe rather than marginalising it.

Note, 2026-11-19. Item 2 previously stated that the marginalised depths are those of the features whose source is the departing frame, and item 3 that a keyframe's removal reinitialises the local prior. In ICE-BA a feature's source is a keyframe, the departing local frame only observes it, and the reinitialisation happens only when the deleted keyframe was the reference.

The local prior does not accumulate over the run. At each marginalisation it is built from the departing frame's terms and the motion prior carried from the previous step, split into a pose part and a motion part, and rebuilt from the motion part alone (§2.6.2, Steps 1 to 7). The two parts have two destinations.

1. The pose part, `m_Zp.GetPriorPose(...)` followed by `m_ZpKF.MarginalizeUninformative(...)`, is a relative prior over keyframe poses. It is sent to the global BA, `m_GBA->PushCameraPriorPose(iFrm, m_ZpKF)`, which appends it to its list, `m_Zps.push_back(Zp)`, without merging it with any earlier prior. It is not applied in the local BA.
2. The motion part, `m_Zp.GetPriorMotion(&m_ZpLF, ...)`, is a prior on the velocity and biases of the new oldest local frame. It is applied in every local iteration, `m_ZpLF.GetFactor(BA_WEIGHT_PRIOR_CAMERA_MOTION, m_CsLF[iLF], &m_ApLF, &U)` in `LocalBundleAdjustor::UpdateFactorsPriorCameraMotion`, with `iLF = m_ic2LF.front()`. It is also sent to the global BA, `m_GBA->PushCameraPriorMotion(iFrm, iKFr, m_ZpLF)`, where the latest one replaces the previous by assignment, `m_ZpLM = Zp`, after the old contribution is subtracted.

The pose part is not applied locally because the local BA holds keyframe poses constant. Its update flags are set in active code only for local frames, `m_ucsLF[iLF] |= LBA_FLAG_FRAME_UPDATE_CAMERA`, and `m_CsKF` changes only where the global map marks a keyframe, `m_marksTmp1[iKF] & GM_FLAG_FRAME_UPDATE_CAMERA`. A prior over keyframe poses therefore has nothing to act on in the local BA, and its information is spent where the keyframes are variables. The global BA applies every prior it holds at every iteration from the current poses, `Z.GetFactor(BA_WEIGHT_PRIOR_CAMERA_POSE, m_Cs, &A, &U, BA_ANGLE_EPSILON)` in `UpdateFactorsPriorCameraPose` and `m_ZpLM.GetFactor(...)` in `UpdateFactorsPriorCameraMotion`. It removes a prior only when a keyframe deletion leaves it invalid or empty (`GlobalBundleAdjustor::DeleteKeyFrame`). No marginalisation of a camera was found in `GlobalBundleAdjustor.cpp` or in the part of `GlobalBundleAdjustorGN.cpp` that was read, and keyframes leave the global BA only through `PushDeleteKeyFrame`, which is consistent with a global BA over all keyframes [4]. The global BA thus keeps no prior of its own making, only the priors the local BA hands it.

ICE-BA's design therefore rests on four facts. The incremental solver concerns the elimination of depths, which leaves no prior. The only marginalisation is that of local frames. Of its prior, only the motion part crosses from one marginalisation to the next inside the local BA, and the pose part leaves for the global BA once, in relative coordinates, which by §2.6.3 makes it indifferent to the global BA's gauge, so that a loop closure moving the world does not fight the window's history. And the local BA treats keyframe poses as constants supplied by the global BA, which is the role this plan gives the mapper's poses (§3.1). Note, 2026-11-19. This paragraph and the lists before it were added after a reading in which the incremental BA concerns only depths and no marginalisation prior survives across frames in either window. The first half of that reading holds. The second holds for the global BA, which marginalises no keyframe, and for all pose information of the local BA, but not for its motion, whose marginal is carried and applied at every iteration. Depths are eliminated from the departing frame's measurements only, so the prior that results is one on cameras and motion rather than on landmarks. A first version of this paragraph, written earlier the same day, stated that `m_Zp` persists and accumulates for the life of the run. The calls `m_Zp.Initialize(..., m_ZpLF, ...)` in both branches of `MarginalizeLocalFrame` rebuild it at every marginalisation.

#### 2.6.2 The relative prior

ICE-BA stores a camera as $\mathbf{C} = \mathbf{T}^{-1}$, the world to camera transform. In the notation of §2.1, with $\mathbf{T}_k = (\mathbf{R}_k, \mathbf{p}_k)$ the camera in the world, the prior `CameraPrior::Pose` holds the following [11].

1. A reference keyframe $r$ (`m_iKFr`), chosen at each marginalisation as the nearest keyframe of the marginalised local frame (`MarginalizeLocalFrame`, `iKFr = LF1.m_iKFNearest`).
2. A list of member keyframes $k \in \mathcal{K}_p$ (`m_iKFs`), and for each a measured relative pose $\hat{\mathbf{T}}_{rk} = \bar{\mathbf{T}}_r^{-1} \bar{\mathbf{T}}_k$ from the estimates at marginalisation (`SetPose`, `Rigid3D::ABI(Tr, Ti, m_Zps[i])`).
3. A dense information matrix and gradient over the stacked residual (`m_Arr`, `m_Arc`, `m_Acc`, `m_br`, `m_bc`), produced by the Schur complement of §2.2.3 and re-expressed in the coordinates of the residuals below.

The residuals are evaluated from the current poses (`Pose::GetError`). For each member, the position of $k$ in the frame of $r$ and the relative rotation,

$$\mathbf{e}_{p,k} = \mathbf{R}_r^\top(\mathbf{p}_k - \mathbf{p}_r) - \hat{\mathbf{p}}_{rk},$$

$$\mathbf{e}_{R,k} = \text{Log}\big(\hat{\mathbf{R}}_{rk} \mathbf{R}_k^\top \mathbf{R}_r\big),$$

and for the reference a two dimensional rotation error $\mathbf{e}_g$, the $x$ and $y$ components of a rotation vector (`GetRodriguesXY`), which constrains the reference's roll and pitch and leaves its yaw free. The prior's cost is the quadratic

$$E_p = \frac{1}{2}\mathbf{e}^\top \mathbf{A} \mathbf{e} + \mathbf{b}^\top \mathbf{e},$$

with $\mathbf{e} = [\mathbf{e}_g; \mathbf{e}_{p,k}; \mathbf{e}_{R,k}; \dots]$, and the global BA rebuilds its linearisation from the current keyframe poses at every iteration (`Zp.GetFactor(BA_WEIGHT_PRIOR_CAMERA_POSE, m_Cs, ...)` in `GlobalBundleAdjustor.cpp`). A separate `CameraPrior::Motion` carries velocity and bias information on one frame, and the local BA's working prior `m_Zp` is a `CameraPrior::Joint`, which joins a `Pose` and a `Motion` with their cross information. The general notion of a prior and its consumption in Basalt's window and mapper are set out in `context/vio_residuals_and_priors.md`, section "What a prior and a marginalisation prior are, and how the window and the mapper consume them". The rest of this section reads ICE-BA's prior in the same terms.

##### What the prior consists of

The joint prior `m_Zp` is a Gaussian over a stacked error $\mathbf{e}$ whose blocks are the following, each with its stored measurement [11].

1. The reference's tilt $\mathbf{e}_g \in \mathbb{R}^2$, from the stored reference rotation `m_Zps.Back()`, with information `m_Arr` and gradient `m_br`.
2. For every member keyframe $k$, $\mathbf{e}_{p,k}$ and $\mathbf{e}_{R,k}$ above, from the stored relative pose `m_Zps[k]`, with diagonal and cross blocks in `m_Acc`, cross blocks with the tilt in `m_Arc` and gradient in `m_bc`.
3. The current local frame $c$, held as one more member under the sentinel index `INT_MAX`, with the same two errors against its stored pose relative to the reference.
4. The motion of $c$, from the stored body frame velocity $\hat{\mathbf{v}}$ and biases $\hat{\mathbf{b}}_a$, $\hat{\mathbf{b}}_\omega$, `m_v`, `m_ba` and `m_bw`, with information `m_Amm`, gradient `m_bm`, and cross blocks `m_Arm` with the tilt and `m_Acm` with the cameras. `Motion::GetError` forms

$$\mathbf{e}_v = \mathbf{R}_c^\top \mathbf{v}_c - \hat{\mathbf{v}},$$

$$\mathbf{e}_{b_a} = \mathbf{b}_{a,c} - \hat{\mathbf{b}}_a,$$

$$\mathbf{e}_{b_\omega} = \mathbf{b}_{\omega,c} - \hat{\mathbf{b}}_\omega,$$

with $\mathbf{R}_c^\top \mathbf{v}_c$ the velocity in the frame of $c$, `C.m_T.GetAppliedRotation(C.m_v)` in ICE-BA's world to camera convention.

Every measurement is set to the estimate of the moment it enters the prior, `SetPose` by `Rigid3D::ABI(Tr, Ti, m_Zps[i])`, `Insert` by `T.GetInverse(m_Zps[i])` and `SetMotion` by `R.Apply(v, m_v)`, so every block of $\mathbf{e}$ is zero at the point where its information was computed, and the gradient $\mathbf{b}$ records how far that point lay from the prior's minimum.

##### The cost and its square root form

The prior's cost is the quadratic above, $E_p = \frac{1}{2}\mathbf{e}^\top \mathbf{A} \mathbf{e} + \mathbf{b}^\top \mathbf{e}$, scaled in code by a weight $w$ (`Pose::GetCost` accumulates $w(\mathbf{e}^\top \mathbf{A} \mathbf{e} + 2\mathbf{b}^\top \mathbf{e})$, twice this form). $\mathbf{A}$ and $\mathbf{b}$ are produced by the Schur complement, as the construction below shows. Completing the square with a factor $\mathbf{U}$ of $\mathbf{A} = \mathbf{U}^\top \mathbf{U}$,

$$E_p = \frac{1}{2}|\mathbf{U} \mathbf{e} + \mathbf{U}^{-\top} \mathbf{b}|^2 - \frac{1}{2}\mathbf{b}^\top \mathbf{A}^{-1} \mathbf{b},$$

because expanding the norm gives $\frac{1}{2}\mathbf{e}^\top \mathbf{U}^\top \mathbf{U} \mathbf{e} + \mathbf{e}^\top \mathbf{U}^\top \mathbf{U}^{-\top} \mathbf{b} + \frac{1}{2}\mathbf{b}^\top \mathbf{U}^{-1} \mathbf{U}^{-\top} \mathbf{b}$, whose middle term is $\mathbf{b}^\top \mathbf{e}$ and whose last term is the constant subtracted. The prior is therefore $\frac{1}{2}|\mathbf{r}_\text{marg}|^2$ with $\mathbf{r}_\text{marg} = \mathbf{U} \mathbf{e} + \mathbf{U}^{-\top} \mathbf{b}$, up to a constant that does not affect the solution. Basalt's prior has the same form, $\frac{1}{2}|\mathbf{J}_m \boldsymbol{\delta} + \mathbf{r}_m|^2$ with $\mathbf{J}_m^\top \mathbf{J}_m = \mathbf{H}_m$ and $\mathbf{J}_m^\top \mathbf{r}_m = \mathbf{b}_m$ (`BundleAdjustmentBase::linearizeMargPrior`, `src/vi_estimator/ba_base.cpp:419-490`). The two differ in two respects only. ICE-BA keeps the information form and factorises it by $\mathbf{L} \mathbf{D} \mathbf{L}^\top$ when needed (`MarginalizeLDL`, `SolveLDL`), whereas Basalt keeps the square root factor from a QR decomposition (`MargHelper::marginalizeHelperSqrtToSqrt`). And the coordinates differ, the relative error $\mathbf{e}(\mathbf{x})$, a nonlinear function of the current states, in ICE-BA, against the absolute tangent offset $\boldsymbol{\delta} = \mathbf{x} \boxminus \mathbf{x}_\text{lin}$ from frozen first estimates in Basalt. The second difference decides everything that follows.

##### Construction at one marginalisation

The steps follow `LocalBundleAdjustor::MarginalizeLocalFrame` and the `CameraPrior` routines it calls [11]. Let $c_1$ and $\mathbf{m}_1$ be the camera and motion of the departing local frame, and $c_2$ and $\mathbf{m}_2$ those of the next.

Step 1. The starting point. `m_Zp` holds the prior over the reference tilt, its members, $c_1$ as the current frame and $\mathbf{m}_1$, left by Step 7 of the previous marginalisation.

Step 2. The visual terms. The departing frame's feature measurements are linearised at the current estimates, and the depth of each feature is eliminated from these measurements alone by the Schur complement of §2.2.3, `FTR::Factor::FixSource::Marginalize` or `FTR::Factor::Full::Source::Marginalize`. What remains couples $c_1$ to the source keyframe $k$ of the feature, or acts on $c_1$ alone when the source is held fixed, and is added to the prior's blocks, `m_Zp.Update(ik, Nk, SAcxx, SAcxz, SAczz)` and `m_Zp.Update(Nk, SAczz)`, which add the blocks to `m_Acc` and `m_bc`. A source keyframe not yet a member first enters by `m_Zp.Insert(...)` with its relative pose at the current estimates as measurement, zero cross information, and a weak diagonal block $w\,\text{diag}(\frac{1}{s_p^2}\mathbf{I}, \frac{1}{s_R^2}\mathbf{I})$, `m_Acc[i][i].SetDiagonal(UT::Inverse(s2p, w), UT::Inverse(s2r, w))` with `BA_WEIGHT_PRIOR_CAMERA_INITIAL`. This reading of which blocks the visual terms reach rests on the arguments of the calls, since their bodies were not in the part of the code read.

Step 3. The inertial term. `Joint::PropagateLF` enlarges the system by fifteen zero rows and columns for $c_2$ and $\mathbf{m}_2$, `_A.InsertZero(ipc2, Npcm, NULL)`, and adds the Hessian and gradient of the inertial factor between $(c_1, \mathbf{m}_1)$ and $(c_2, \mathbf{m}_2)$, linearised at the current estimates, block by block, `_A.IncreaseBlockDiagonal(ipg, A.m_Agg)` and `_b.IncreaseBlock(ipg, A.m_bg)` and the ten cross blocks `A.m_A[k]`.

Step 4. The elimination. With the kept coordinates indexed by $k$ and the departing $(c_1, \mathbf{m}_1)$ by $1$, `_A.MarginalizeLDL(...)` replaces the system by its Schur complement,

$$\mathbf{A} \leftarrow \mathbf{A}_{kk} - \mathbf{A}_{k1} \mathbf{A}_{11}^{-1} \mathbf{A}_{1k},$$

$$\mathbf{b} \leftarrow \mathbf{b}_k - \mathbf{A}_{k1} \mathbf{A}_{11}^{-1} \mathbf{b}_1,$$

which by (I4) is the marginal of the Gaussian over the kept states. When the departing frame is itself the reference keyframe, `Joint::PropagateKF` does the same with the member list reduced to the new current frame, keeping the reference rotation `m_Zps.Back()`.

Step 5. The re-centring. `SetPose(Tr, Nk, C.m_T)` and `SetMotion(C.m_T, C.m_v, C.m_ba, C.m_bw)` set the measurements of $c_2$ and $\mathbf{m}_2$ to their current estimates, so their error blocks restart at zero, as in Step 2 for new members.

Step 6. The split. `GetPriorPose` eliminates the motion block, and in the branch where the departing frame is not the reference also the current frame, by `MarginalizeLDL`, which leaves a prior over the reference tilt and the member keyframes, `m_ZpKF`. `MarginalizeUninformative` then eliminates, by the Schur complement `Pose::Marginalize(i)`, every member whose marginal variance exceeds the thresholds $\frac{s_p^2}{w}$ or $\frac{s_R^2}{w}$, and the result is pushed to the global BA. `GetPriorMotion` eliminates every pose block and the tilt, `MarginalizeLDL(0, Npc, ...)`, which leaves the motion marginal `m_ZpLF` over $\mathbf{m}_2$. Both are marginals of the one Gaussian of Step 4, and applying them as two independent factors ignores their correlation.

Step 7. The rebuild. Both branches end with `m_Zp.Initialize(..., iKFr, Tr, s2g, m_ZpLF, newKF, ...)`, which calls `Pose::Initialize` and `Motion::Initialize(Zp)` and zeroes the cross blocks `m_Arm` and `m_Acm`. The pose part restarts as a tilt prior on the reference with variance $s_g^2$, `BA_VARIANCE_PRIOR_GRAVITY_NEW` in the branch where the departing frame is not the reference, which is the weak gravity prior the paper adds when the reference changes [4, Fig. 5f], together with a placeholder member for the current frame with a weak diagonal block. The motion part is copied from `m_ZpLF`. What crosses into the next marginalisation is therefore the motion marginal, and the pose information leaves the local BA once, to the global BA. The order of these calls was read from a summarising fetch of a truncated file, so the rebuild after every push is the reading most consistent with the global BA appending each prior without merging, rather than a line by line confirmation.

##### Use and relinearisation in the global BA

The global BA evaluates each prior at its current poses $\mathbf{x}$ at every iteration (`Pose::GetError`, `Pose::GetErrorJacobian(Cs, ...)`, `Pose::GetFactor`). For an increment $\boldsymbol{\xi}$, a first order expansion of the error about the current poses gives

$$\mathbf{e}(\mathbf{x} \boxplus \boldsymbol{\xi}) \approx \mathbf{e}(\mathbf{x}) + \mathbf{J}(\mathbf{x})\,\boldsymbol{\xi},$$

with $\mathbf{J}(\mathbf{x})$ the derivative of $\mathbf{e}$ at the current poses, built from `SkewSymmetricMatrix::AB(ec.m_ep, C1, Jc.m_Jpr1)` and `GetRodriguesJacobianInverse(ec.m_er, ...)`. Substituting into $E_p$,

$$E_p(\boldsymbol{\xi}) \approx \frac{1}{2}(\mathbf{e} + \mathbf{J} \boldsymbol{\xi})^\top \mathbf{A} (\mathbf{e} + \mathbf{J} \boldsymbol{\xi}) + \mathbf{b}^\top (\mathbf{e} + \mathbf{J} \boldsymbol{\xi}),$$

whose Hessian and gradient in $\boldsymbol{\xi}$ are

$$\mathbf{H}_p = \mathbf{J}^\top \mathbf{A} \mathbf{J},$$

$$\mathbf{g}_p = \mathbf{J}^\top (\mathbf{A} \mathbf{e} + \mathbf{b}),$$

the blocks `U->Get(...)` accumulates. $\mathbf{A}$ and $\mathbf{b}$ are frozen at the marginalisation, while $\mathbf{e}$ and $\mathbf{J}$ are recomputed from the current poses. This is what relinearisation of the prior at the current estimate means here. Basalt's prior offers nothing to relinearise in the same way, because its coordinate is $\boldsymbol{\delta}$, whose increment is exactly $\boldsymbol{\delta} + \boldsymbol{\xi}$, so its Jacobian is the identity and `linearizeMargPrior` adds $\mathbf{H}_m$ and $\mathbf{H}_m \boldsymbol{\delta} + \mathbf{b}_m$ at every iteration. Its nonlinearity was spent at marginalisation, in the factor Jacobians evaluated at $\mathbf{x}_\text{lin}$.

The relative coordinates make the relinearisation safe. For a rigid motion $\mathbf{G}$ of translation and yaw, §2.6.3 gives $\mathbf{e}(\mathbf{G} \mathbf{x}) = \mathbf{e}(\mathbf{x})$ for every $\mathbf{x}$. Taking $\mathbf{G} = \text{Exp}(\epsilon \mathbf{n})$ along a gauge direction and differentiating at $\epsilon = 0$ gives

$$\mathbf{J}(\mathbf{x})\,\mathbf{N}(\mathbf{x}) = \mathbf{0}$$

at every $\mathbf{x}$, with $\mathbf{N}(\mathbf{x})$ the four gauge directions at $\mathbf{x}$. The prior adds no information along the gauge wherever it is linearised, so it needs no first estimate to keep the null space that §2.2.4 obtains in Basalt by freezing Jacobians.

##### Removal of a keyframe from a prior

When the map deletes a keyframe, `GlobalBundleAdjustor::DeleteKeyFrame` calls `Zp.DeleteKeyFrame(iKF, &i)` on every prior and drops the priors left invalid or empty, and the local BA does the same on `m_Zp`. `Pose::DeleteKeyFrame` shifts the indices of later keyframes, and if the deleted keyframe is a member, erases its error block, its measurement and its rows and columns of $\mathbf{A}$ and $\mathbf{b}$ (`Pose::Erase`). Partition the error into the kept blocks $\mathbf{e}_y$ and the deleted block $\mathbf{e}_z$. Erasing keeps

$$E = \frac{1}{2}\mathbf{e}_y^\top \mathbf{A}_{yy} \mathbf{e}_y + \mathbf{b}_y^\top \mathbf{e}_y,$$

which is $E_p$ at $\mathbf{e}_z = \mathbf{0}$, the prior conditioned on the deleted keyframe sitting exactly at its measured relative pose. Marginalising it would instead leave the information

$$\mathbf{A}_{yy} - \mathbf{A}_{yz} \mathbf{A}_{zz}^{-1} \mathbf{A}_{zy},$$

which is smaller whenever $\mathbf{A}_{yz} \neq \mathbf{0}$. Removal therefore keeps the information that flowed through the deleted keyframe as if that keyframe were known exactly, an overconfident approximation that `Pose::Marginalize`, used by `MarginalizeUninformative`, avoids. If the deleted keyframe is the reference, `m_iKFr` becomes $-1$ and `Invalid()` holds, so the global BA discards the prior, and the local BA rebuilds `m_Zp` from `m_ZpLF` or from a weak prior at the new reference.

Note, 2026-11-19. The five subsections above were added to set out what the prior consists of, how it is built, carried and consumed, and what the removal of a keyframe does to it.

Two consequences matter here. The prior is a star anchored at one reference keyframe, the same topology as the anchor formulation of §3.4. Its information over the members is dense, so the choice of reference is a change of coordinates rather than a choice of weighting (§3.5.3). And the prior is nonlinear in the current poses and relinearised at the current estimate, which first estimate Jacobians (§2.2.4) forbid in Basalt.

The prior sent to the global BA, `m_GBA->PushCameraPriorPose(iFrm, m_ZpKF)`, is the pose part of `m_Zp` extracted by `GetPriorPose` (§2.6.1), of the same type and form, so ICE-BA submits a relative prior over a reference and several member keyframes with dense information, together with a motion prior through `PushCameraPriorMotion(iFrm, iKFr, m_ZpLF)`. The paper's description of a constraint between the last keyframe of the local BA and the latest keyframe [4] is the special case of a single member, which `Pose::Initialize(w, iKF1, iKF2, T, S)` also constructs. Note, 2026-11-19. This paragraph previously called the prior sent to the global BA the same object as the local prior, whereas it is the pose part extracted from it.

#### 2.6.3 Invariance of the prior under a global correction

Let the global BA correct the world by $\mathbf{G} = (\mathbf{R}_G, \mathbf{t}_G)$, so that every keyframe moves to $\mathbf{T}'_k = \mathbf{G} \mathbf{T}_k$, that is $\mathbf{R}'_k = \mathbf{R}_G \mathbf{R}_k$ and $\mathbf{p}'_k = \mathbf{R}_G \mathbf{p}_k + \mathbf{t}_G$. A visual inertial global BA leaves gravity along the world $z$ axis, so $\mathbf{R}_G$ is a rotation about $z$ and $\mathbf{R}_G^\top \mathbf{e}_z = \mathbf{e}_z$.

Step 1. The relative position is unchanged,

$$\mathbf{R}_r'^\top(\mathbf{p}'_k - \mathbf{p}'_r) = \mathbf{R}_r^\top \mathbf{R}_G^\top \big(\mathbf{R}_G \mathbf{p}_k + \mathbf{t}_G - \mathbf{R}_G \mathbf{p}_r - \mathbf{t}_G\big) = \mathbf{R}_r^\top(\mathbf{p}_k - \mathbf{p}_r),$$

so $\mathbf{e}'_{p,k} = \mathbf{e}_{p,k}$.

Step 2. The relative rotation is unchanged,

$$\mathbf{R}_k'^\top \mathbf{R}'_r = \mathbf{R}_k^\top \mathbf{R}_G^\top \mathbf{R}_G \mathbf{R}_r = \mathbf{R}_k^\top \mathbf{R}_r,$$

so $\mathbf{e}'_{R,k} = \mathbf{e}_{R,k}$.

Step 3. The direction of gravity seen by the reference is unchanged,

$$\mathbf{R}_r'^\top \mathbf{e}_z = \mathbf{R}_r^\top \mathbf{R}_G^\top \mathbf{e}_z = \mathbf{R}_r^\top \mathbf{e}_z,$$

and roll and pitch are functions of that direction alone, so $\mathbf{e}'_g = \mathbf{e}_g$.

Step 4. Every residual is unchanged, so $E_p$ is unchanged exactly, not only to first order, because $E_p$ is evaluated at the current poses rather than through a linearisation frozen at old ones.

The four directions of $\mathbf{G}$, translation and yaw, are precisely the gauge of visual inertial estimation (§2.4.3). A correction along them therefore costs the local prior nothing, while an absolute prior $\frac{1}{2}|\mathbf{J}_m(\mathbf{x}_a \boxminus \bar{\mathbf{x}}_a) + \mathbf{r}_m|^2$ evaluated as a function of absolute states would resist it, although no measurement contradicts it.

#### 2.6.4 Bringing global corrections into the local window

The global BA's keyframe poses enter the local BA by overwriting, in three steps [11].

Step 1. `LocalBundleAdjustor::UpdateCameras(const std::vector<GlobalMap::InputCamera>&)` saves each corrected keyframe's old pose, `m_CsKFBkp[iKF] = m_CsKF[iKF]`, and writes the new one, `m_CsKF[iKF] = C.m_C`.

Step 2. `SynchronizeData` then calls `UpdateCameras(m_marksTmp1, m_CsKFBkp, m_CsKF)`, which moves any local frame $C_f$ whose nearest keyframe $n$ changed by that keyframe's correction. With $\mathbf{C}^\text{old}_n$ and $\mathbf{C}^\text{new}_n$ the backup and the new pose, the code forms $\mathbf{T}_I = (\mathbf{C}^\text{new}_n)^{-1} \mathbf{C}^\text{old}_n$ and sets $\mathbf{C}_f \leftarrow \mathbf{C}_f \mathbf{T}_I^{-1}$, so

$$\mathbf{C}'_f = \mathbf{C}_f (\mathbf{C}^\text{old}_n)^{-1} \mathbf{C}^\text{new}_n,$$

$$\mathbf{C}'_f (\mathbf{C}^\text{new}_n)^{-1} = \mathbf{C}_f (\mathbf{C}^\text{old}_n)^{-1}.$$

The pose of every local frame relative to its nearest keyframe is preserved, and its velocity is rotated by the same correction, `C.m_v = TI.GetAppliedRotation(C.m_v)`. The inertial residual of §2.2.5 between two frames moved by one rigid $\mathbf{G}$ with $\mathbf{R}_G$ about gravity is unchanged by Steps 1 to 3 of §2.6.3 applied to its positions, rotations and velocities, because $\mathbf{R}_G^\top \mathbf{g} = \mathbf{g}$.

Step 3. Neither `UpdateCameras` overload touches the local prior `m_Zp`. The local solve evaluates only its motion part `m_ZpLF`, at the oldest local frame's camera, which Step 2 has carried rigidly. Its pose part is relative and is used at the current keyframe poses by the next marginalisation and, once sent, by the global BA (§2.6.1). Note, 2026-11-19. This step previously stated that `m_Zp` is re-evaluated at the new poses in the next local solve, which overlooked that the local BA holds keyframes constant and applies only the motion part.

Split the global correction into its rigid part, one $\mathbf{G}$ common to the keyframes of the window, and the remainder, which changes their relative poses. By §2.6.3 and Step 2 the rigid part leaves every local residual and the local prior unchanged. The remainder changes $\mathbf{e}_{p,k}$ and $\mathbf{e}_{R,k}$, so it appears as a cost in the prior and in the residuals between local frames attached to different keyframes, and the next local solve weighs it against the window's own measurements. The impact of the global map on the local prior is thus removed for the gauge directions, which carry the drift a loop closure corrects, and reduced to a cost the solver can arbitrate for the shape.

##### Why the overwrite leaves every prior valid

The overwrite of Steps 1 and 2 changes keyframe poses that some prior depends on, yet no prior is updated, for three reasons that together cover every prior in the system.

1. The local BA carries no pose prior. By §2.6.2 Step 7 the only prior that crosses from one marginalisation to the next inside the local BA is the motion marginal `m_ZpLF` on the oldest local frame. Step 2 moves that frame rigidly, $\mathbf{R}'_f = \mathbf{R}_\Delta \mathbf{R}_f$ with $\mathbf{R}_\Delta$ the rotation of its nearest keyframe's correction, and rotates its velocity by the same rotation, $\mathbf{v}'_f = \mathbf{R}_\Delta \mathbf{v}_f$. The body frame velocity of the motion error is then unchanged,

$$\mathbf{R}_f'^\top \mathbf{v}'_f = \mathbf{R}_f^\top \mathbf{R}_\Delta^\top \mathbf{R}_\Delta \mathbf{v}_f = \mathbf{R}_f^\top \mathbf{v}_f,$$

and the biases are untouched, so $\mathbf{e}_v$, $\mathbf{e}_{b_a}$ and $\mathbf{e}_{b_\omega}$ are unchanged by any correction, rigid or not, because each frame follows its own keyframe rigidly.
2. The keyframes are constants of the local BA. No local linear system has a column for a keyframe pose, so no local information was ever computed about a keyframe's old value, and no linearisation point of a keyframe is stored that the overwrite could contradict. The visual factors that tie local frames to keyframes are rebuilt from the current poses, and after the overwrite they are rebuilt from the new ones, exactly as they would be after a step of the local solve.
3. The pose priors live where the keyframes are variables. Each `m_ZpKF` is held by the global BA, which is the writer of the corrected poses, and is evaluated at its current poses at every iteration (§2.6.2). The rigid part of a correction leaves its errors unchanged by §2.6.3. The non rigid part changes them and appears as cost, which is the prior correctly contesting a change of shape that the marginalised measurements disagree with.

The overwrite is therefore safe in ICE-BA not because the priors are updated but because no prior is stated in coordinates that the overwrite changes, apart from the change of shape the priors should see. Note, 2026-11-19. This subsection was added to answer how ICE-BA rewrites keyframe poses without disturbing its priors.

#### 2.6.5 How ICE-BA solves the problem of this plan, and what transfers

ICE-BA updates a local window from a global map by four cooperating choices, keyframe poses that are constants of the local BA (§2.6.1), a prior on relative coordinates (§2.6.2), a rigid carry of each local frame with its nearest keyframe (§2.6.4 Step 2), and relinearisation of the prior at the current estimate, which lets an overwrite change the prior's residual without invalidating its Jacobians. The first choice is the one this plan copies most directly, since the mapper's poses enter the window as constants of its residuals (§3.1). Note, 2026-11-19. This paragraph previously listed three choices, without the constant keyframes.

The fourth choice, together with the overwrite it permits, does not transfer to Basalt, for four reasons that follow from §2.6.2 and §2.6.4.

1. Basalt's prior is stated in the coordinates an overwrite changes. Its cost $\frac{1}{2}|\mathbf{J}_m \boldsymbol{\delta} + \mathbf{r}_m|^2$ is a function of $\boldsymbol{\delta} = \mathbf{x} \boxminus \mathbf{x}_\text{lin}$, absolute offsets of the window's own states, so writing a pose moves $\boldsymbol{\delta}$ and the prior pulls back, although no measurement asks for it, which is defect 1 of §1. ICE-BA's errors are relative, and the states the global BA writes are not variables of any local prior (§2.6.4, reasons 1 and 2).
2. Its gauge consistency depends on the first estimates. The null space of $\mathbf{H}_m$ is the gauge at $\mathbf{x}_\text{lin}$, and the other factors on the same states keep it only because their Jacobians are evaluated at the same $\mathbf{x}_\text{lin}$ (§2.2.4). Relinearising those factors at the current estimate would give them the null space $\mathbf{N}(\mathbf{x})$ while the prior keeps $\mathbf{N}(\mathbf{x}_\text{lin})$, and the difference is spurious information along position and yaw. ICE-BA's prior satisfies $\mathbf{J}(\mathbf{x})\,\mathbf{N}(\mathbf{x}) = \mathbf{0}$ at every $\mathbf{x}$ (§2.6.2), so it needs no frozen point.
3. There is nothing to relinearise. The prior is linear in $\boldsymbol{\delta}$ with the identity as Jacobian, and its information was formed from factor Jacobians at $\mathbf{x}_\text{lin}$. Relinearising it would need it re-expressed in relative coordinates first, finding $\mathbf{A}$ with $\mathbf{H}_m = \mathbf{M}^\top \mathbf{A} \mathbf{M}$ for $\mathbf{M}$ the derivative of the relative errors with respect to $\boldsymbol{\delta}$ at $\mathbf{x}_\text{lin}$, which is ICE-BA's conversion [4, §5, eq. 19] and, in covariance form, what NFR does for the mapper (§2.7.2). Doing it for the window would change `marginalize`, the square root prior, the re-centring of the gravity and scale block and the mapper packets, all of which assume world frame priors (`context/dynamic_marginalisation.md` §6.12, Q6).
4. The delayed store records first estimates (§2.8.4), which any write outside the solve invalidates.

The current overwrite also moves only `frame_poses`, so it is not even rigid within the window, the opposite of ICE-BA's Step 2. Note, 2026-11-19. This paragraph previously gave the first and fourth reasons only, as a single sentence.

The first three choices transfer through the solve rather than around it. The mapper's poses enter the window's residuals as constants, as ICE-BA's keyframes enter its local BA. A residual on the window's relative poses against the mapper's relative poses is invariant to the mapper's gauge by the computation of §2.6.3 (§3.6), so the mapper's drift in position and yaw cannot enter, and its Jacobians, like ICE-BA's, satisfy $\mathbf{J}(\mathbf{x})\,\mathbf{N}(\mathbf{x}) = \mathbf{0}$ at any linearisation point. The full states and their velocities follow a corrected keyframe through the inertial residuals in the same solve, which is the effect ICE-BA obtains by its rigid carry, without writing any state. ICE-BA also shows that a star anchored at one keyframe is a sound parametrisation of relative information, provided its information is dense, which is the question §3.5 takes up.

#### 2.6.6 Assessment

ICE-BA is a two way architecture built around relative priors. Its prior and its update path were designed together, which is what lets it overwrite safely. The present estimator has an absolute first estimate prior and a delayed store, so it takes ICE-BA's relative coordinates and leaves its overwrite.

### 2.7 Non-linear factor recovery in Basalt's mapper

#### 2.7.1 Setting

Basalt's odometry minimises reprojection, inertial and prior terms and marginalises with §2.2.3 [6, eq. 22 to 24]. Every marginalisation packet gives a dense linear prior $\mathcal{N}(\boldsymbol{\mu}_o, \boldsymbol{\Sigma}_o)$ on the keyframes that leave. A dense linear prior cannot be relinearised, so the mapper replaces it by nonlinear factors whose linearisation matches it as closely as possible in Kullback–Leibler divergence [6].

#### 2.7.2 Derivation of the recovered information

Choose residuals $\mathbf{r}_i(\mathbf{s}, \mathbf{z}_i)$ with unknown informations $\mathbf{H}_i$. Stacking their Jacobians at $\boldsymbol{\mu}_o$ into $\mathbf{J}_r$ and their informations into the block diagonal $\mathbf{H}_r$, the recovered information is [6, eq. 28]

$$\mathbf{H}_a = \mathbf{J}_r^\top \mathbf{H}_r \mathbf{J}_r.$$

Step 1. The pseudo measurements $\mathbf{z}_i$ are chosen so that every residual vanishes at the mean, $\mathbf{r}_i(\boldsymbol{\mu}_o, \mathbf{z}_i) = \mathbf{0}$, so $\boldsymbol{\mu}_a = \boldsymbol{\mu}_o$ and the mean term of (I7) drops,

$$D_\text{KL} = \frac{1}{2}\Big(\text{tr}(\mathbf{H}_a \boldsymbol{\Sigma}_o) - \ln\,\text{det}(\mathbf{H}_a \boldsymbol{\Sigma}_o) - d\Big).$$

Step 2. By the cyclic trace of (I5), with $\mathbf{S} = \mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top$ and $\mathbf{H}_r$ block diagonal,

$$\text{tr}(\mathbf{J}_r^\top \mathbf{H}_r \mathbf{J}_r \boldsymbol{\Sigma}_o) = \text{tr}(\mathbf{H}_r \mathbf{S}) = \sum_i \text{tr}(\mathbf{H}_i \mathbf{S}_{ii}).$$

Step 3. For a square invertible $\mathbf{J}_r$, which [6] assumes for the closed form, the determinant product of (I5) gives

$$\ln\,\text{det}(\mathbf{J}_r^\top \mathbf{H}_r \mathbf{J}_r \boldsymbol{\Sigma}_o) = \sum_i \ln\,\text{det}\mathbf{H}_i + \ln\,\text{det}\mathbf{S}.$$

Step 4. The divergence separates into one term per factor plus terms free of the $\mathbf{H}_i$,

$$D_\text{KL} = \frac{1}{2}\sum_i \Big(\text{tr}(\mathbf{H}_i \mathbf{S}_{ii}) - \ln\,\text{det}\mathbf{H}_i\Big) + \text{const}.$$

Step 5. Setting the derivative with respect to each $\mathbf{H}_i$ to zero with (I6), $\mathbf{S}_{ii} - \mathbf{H}_i^{-1} = \mathbf{0}$, so [6, eq. 30]

$$\mathbf{H}_i = \big(\{\mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top\}_{ii}\big)^{-1}.$$

Each recovered factor carries exactly the marginal uncertainty the dense prior assigns to the quantity it measures.

#### 2.7.3 The recovered factors and the mapping cost

Basalt recovers relative pose factors and roll pitch factors, with measurements taken from the estimate at linearisation [6, eq. 31 and 32],

$$\mathbf{r}_\text{rel} = \text{Log}\big(\mathbf{z}_\text{rel}\,\mathbf{T}_j^{-1} \mathbf{T}_i\big),$$

$$\mathbf{z}_\text{rel} = \mathbf{T}_i^{-1} \mathbf{T}_j,$$

$$\mathbf{r}_\text{rp} = \big\lfloor \mathbf{z}_\text{rp}\,\mathbf{R}_i^{-1}(0, 0, -1)^\top \big\rfloor_{xy},$$

$$\mathbf{z}_\text{rp} = \mathbf{R}_i.$$

Absolute position and yaw factors are dropped because odometry cannot observe them [6]. The mapping cost adds these factors to the reprojection errors of the mapper's own matches [6, eq. 26 and 35],

$$E^G(\mathbf{s}) = \sum_{i, t} |\mathbf{r}_{it}|^2_{\boldsymbol{\Sigma}_{it}^{-1}} + \sum_{(i, j) \in \mathcal{R}} |\mathbf{r}_{ij}|^2_{\mathbf{H}_{ij}} + \sum_{i} |\mathbf{r}_i|^2_{\mathbf{H}_i}.$$

The local mapper implements these as `RelPoseFactor` and `RollPitchFactor` (`include/basalt/utils/imu_types.h:504-521`).

#### 2.7.4 The relative pose factor in code

`relPoseError` (`include/basalt/utils/nfr.h:43-73`) implements eq. 31 with the decoupled logarithm `se3_logd`, which returns the translation and the rotation vector separately (`thirdparty/basalt-headers/include/basalt/utils/sophus_utils.hpp:62-68`). With $\mathbf{T}_{ji} = \mathbf{T}_j^{-1} \mathbf{T}_i = (\mathbf{R}_j^\top \mathbf{R}_i, \mathbf{R}_j^\top(\mathbf{p}_i - \mathbf{p}_j))$ and the measurement $\hat{\mathbf{T}}_{ij} = (\hat{\mathbf{R}}_{ij}, \hat{\mathbf{t}}_{ij})$, the product $\hat{\mathbf{T}}_{ij} \mathbf{T}_{ji}$ gives

$$\mathbf{r}_t = \hat{\mathbf{R}}_{ij} \mathbf{R}_j^\top(\mathbf{p}_i - \mathbf{p}_j) + \hat{\mathbf{t}}_{ij},$$

$$\mathbf{r}_R = \text{Log}\big(\hat{\mathbf{R}}_{ij} \mathbf{R}_j^\top \mathbf{R}_i\big).$$

Both vanish when $\mathbf{T}_i^{-1} \mathbf{T}_j = \hat{\mathbf{T}}_{ij}$, since then $\mathbf{R}_j^\top \mathbf{R}_i = \hat{\mathbf{R}}_{ij}^\top$ and $\mathbf{R}_j^\top(\mathbf{p}_i - \mathbf{p}_j) = -\hat{\mathbf{R}}_{ij}^\top \hat{\mathbf{t}}_{ij}$. Write $\mathbf{E} = \hat{\mathbf{R}}_{ij} \mathbf{R}_j^\top \mathbf{R}_i = \text{Exp}(\mathbf{r}_R)$. The Jacobians on the left world increments of §2.1 follow in four steps.

Step 1. In $\delta\mathbf{p}_i$ and $\delta\mathbf{p}_j$ the translation is linear,

$$\frac{\partial\mathbf{r}_t}{\partial\delta\mathbf{p}_i} = \hat{\mathbf{R}}_{ij} \mathbf{R}_j^\top = \mathbf{E} \mathbf{R}_i^\top,$$

$$\frac{\partial\mathbf{r}_t}{\partial\delta\mathbf{p}_j} = -\mathbf{E} \mathbf{R}_i^\top,$$

and the rotation does not depend on positions.

Step 2. In $\delta\boldsymbol{\varphi}_i$, $\mathbf{R}_i$ does not enter $\mathbf{r}_t$. For $\mathbf{r}_R$, (I1) with $\mathbf{R} = \mathbf{R}_i^\top$ gives $\text{Exp}(\delta\boldsymbol{\varphi}_i) \mathbf{R}_i = \mathbf{R}_i \text{Exp}(\mathbf{R}_i^\top \delta\boldsymbol{\varphi}_i)$, so the rotation becomes $\text{Log}\big(\mathbf{E}\,\text{Exp}(\mathbf{R}_i^\top \delta\boldsymbol{\varphi}_i)\big)$. The right Jacobian form of (I2), $\text{Log}(\text{Exp}(\boldsymbol{\phi}) \text{Exp}(\boldsymbol{\epsilon})) \approx \boldsymbol{\phi} + \mathbf{J}_r^{-1}(\boldsymbol{\phi}) \boldsymbol{\epsilon}$ [8], with $\boldsymbol{\phi} = \mathbf{r}_R$ and $\boldsymbol{\epsilon} = \mathbf{R}_i^\top \delta\boldsymbol{\varphi}_i$, gives

$$\frac{\partial\mathbf{r}_R}{\partial\delta\boldsymbol{\varphi}_i} = \mathbf{J}_r^{-1}(\mathbf{r}_R) \mathbf{R}_i^\top.$$

Step 3. In $\delta\boldsymbol{\varphi}_j$, $\mathbf{R}_j^\top \leftarrow \mathbf{R}_j^\top \text{Exp}(-\delta\boldsymbol{\varphi}_j)$. For the translation, (I3) with $\boldsymbol{\epsilon} = -\delta\boldsymbol{\varphi}_j$ and $\mathbf{u} = \mathbf{p}_i - \mathbf{p}_j$ gives $\text{Exp}(-\delta\boldsymbol{\varphi}_j) \mathbf{u} \approx \mathbf{u} + [\mathbf{u}]_\times \delta\boldsymbol{\varphi}_j$, so

$$\frac{\partial\mathbf{r}_t}{\partial\delta\boldsymbol{\varphi}_j} = \hat{\mathbf{R}}_{ij} \mathbf{R}_j^\top [\mathbf{p}_i - \mathbf{p}_j]_\times = -\mathbf{E} \mathbf{R}_i^\top [\mathbf{p}_j - \mathbf{p}_i]_\times.$$

The code writes this as $-\mathbf{E}\,[\mathbf{R}_i^\top(\mathbf{p}_j - \mathbf{p}_i)]_\times \mathbf{R}_i^\top$, which is the same matrix by (I8) with $\mathbf{R} = \mathbf{R}_i^\top$ and $\mathbf{v} = \mathbf{p}_j - \mathbf{p}_i$. For the rotation, (I1) moves $\text{Exp}(-\delta\boldsymbol{\varphi}_j)$ to the right of $\mathbf{R}_i$ as $\text{Exp}(-\mathbf{R}_i^\top \delta\boldsymbol{\varphi}_j)$, and (I2) gives

$$\frac{\partial\mathbf{r}_R}{\partial\delta\boldsymbol{\varphi}_j} = -\mathbf{J}_r^{-1}(\mathbf{r}_R) \mathbf{R}_i^\top.$$

Step 4. Stacking, with $\text{blkdiag}(\mathbf{E}, \mathbf{J}_r^{-1}(\mathbf{r}_R))$ computed by `rightJacobianInvSE3Decoupled` (`sophus_utils.hpp:456-474`),

$$\frac{\partial\mathbf{r}}{\partial\delta\mathbf{x}_i} = \begin{bmatrix} \mathbf{E} & \mathbf{0} \\ \mathbf{0} & \mathbf{J}_r^{-1}(\mathbf{r}_R) \end{bmatrix} \begin{bmatrix} \mathbf{R}_i^\top & \mathbf{0} \\ \mathbf{0} & \mathbf{R}_i^\top \end{bmatrix},$$

$$\frac{\partial\mathbf{r}}{\partial\delta\mathbf{x}_j} = -\begin{bmatrix} \mathbf{E} & \mathbf{0} \\ \mathbf{0} & \mathbf{J}_r^{-1}(\mathbf{r}_R) \end{bmatrix} \begin{bmatrix} \mathbf{R}_i^\top & [\mathbf{R}_i^\top(\mathbf{p}_j - \mathbf{p}_i)]_\times \mathbf{R}_i^\top \\ \mathbf{0} & \mathbf{R}_i^\top \end{bmatrix},$$

which is the code line for line, and is checked numerically by `RelPoseTest` (`test/src/test_nfr.cpp:24-61`).

The mapper accumulates each factor into its normal equations as $\mathbf{H} \mathrel{+}= \mathbf{J}^\top \boldsymbol{\Lambda} \mathbf{J}$ and $\mathbf{b} \mathrel{+}= \mathbf{J}^\top \boldsymbol{\Lambda} \mathbf{r}$ with $\boldsymbol{\Lambda}$ = `cov_inv`, in the four blocks $(i, i)$, $(i, j)$, $(j, i)$, $(j, j)$ (`MapperLinearizeAbsReduce::operator()`, `include/basalt/vi_estimator/nfr_mapper.h:131-162`), reduced in parallel beside the visual blocks of `linearizeRel` and `linearizeAbs` (`src/vi_estimator/nfr_mapper.cpp:284-299`). It evaluates residual and Jacobians at the same current point, because the mapper has no first estimates (`NfrMapper::optimize` asserts `!isLinearized()`, `:344`). The window, which has them, follows the convention of `linearizeHelperStatic`, which calls `computeRelPose` once at `getPoseLin` for the Jacobians and again at `getPose` for the value (`src/vi_estimator/sc_ba_base.cpp:278-290`).

#### 2.7.5 The topology of the recovered factors

`NfrMapper::extractNonlinearFactors` (`src/vi_estimator/nfr_mapper.cpp:157-252`) takes the one keyframe $m$ the packet marginalises and creates a relative pose factor from $m$ to every other keyframe of the packet's `kfs_all` (`:215-249`), plus a roll pitch factor on $m$. Each packet is therefore a star centred on the departing keyframe. Over a run, every keyframe is the centre of one star whose leaves are the keyframes that shared its window, so the union approaches the complete graph over each window's keyframes, the NFR counterpart of the pairwise topology of §3.4.

The window uses the same machinery with its measurement taken from the mapper's refined poses (§3.2). The code the two share is kept in one place. The data type is `RelPoseFactor` (`include/basalt/utils/imu_types.h:504-511`), which the window constructs exactly as `extractNonlinearFactors` does. The residual and its Jacobians have one implementation, `relPoseErrorGS` in `include/basalt/utils/ba_utils.h`, of which `relPoseError` becomes the scale one case with bit identical results (Change 2). The weighting by `cov_inv` is the one `MapperLinearizeAbsReduce` applies (`include/basalt/vi_estimator/nfr_mapper.h:131-162`), written as whitened rows rather than accumulated blocks (Change 5). Note, 2026-11-19. This paragraph previously evaluated whether the mapper's recovered factors could themselves be fed to the window and found that they could not. That question was never posed, and the paragraph now records only the code the window shares with the mapper.

#### 2.7.6 Assessment

Two consequences matter here. The mapper's estimate is built partly from the odometry's own marginal information, so any feedback must not fold it back unweighted. The mapper has no factor on absolute position or yaw, so those four directions are free up to damping and the seeding from the odometry (`context/vio_localmapper_correction_loop.md`, Mechanism C). Basalt has no feedback from the mapper to the odometry [6].

### 2.8 Delayed marginalisation in DM-VIO

#### 2.8.1 Idea

Marginalisation cannot be undone, but it can be delayed [7]. Beside the main graph, DM-VIO keeps a delayed graph that marginalises the same variables in the same order $d$ keyframes later, so the original nonlinear factors of the last $d$ keyframes are still available (`context/dynamic_marginalisation.md` §3.3).

#### 2.8.2 Readvancing

A new prior is obtained by marginalising, in the delayed graph and at the current estimate, every variable the main graph has marginalised since. Successive Schur complements equal the joint one in three steps.

Step 1. Split the removed block as $\boldsymbol{\delta}_\beta = [\boldsymbol{\delta}_{\beta_1}; \boldsymbol{\delta}_{\beta_2}]$. By §2.2.3 the prior is the minimum of $F$ over $\boldsymbol{\delta}_\beta$.

Step 2. A minimum over two blocks is the minimum over one block of the minimum over the other,

$$\min_{\boldsymbol{\delta}_{\beta_1}, \boldsymbol{\delta}_{\beta_2}} F = \min_{\boldsymbol{\delta}_{\beta_2}} \Big(\min_{\boldsymbol{\delta}_{\beta_1}} F\Big).$$

Step 3. The inner minimum is the prior after removing $\boldsymbol{\delta}_{\beta_1}$, and the outer one removes $\boldsymbol{\delta}_{\beta_2}$ from it, each by §2.2.3.

The readvanced prior has new and consistent linearisation points, because every factor of the delayed window is relinearised before it is marginalised [7].

#### 2.8.3 Replacement on a scale change

DM-VIO compares the current scale $s$ with the one at which the prior was linearised, $\delta_s = \frac{\max(s, s_\text{fej})}{\min(s, s_\text{fej})}$, and readvances when $\delta_s$ exceeds a threshold [7].

#### 2.8.4 Assessment

The readvance relinearises at recorded first estimates and recorded values of every departed state. A state moved outside the solve makes the recorded values disagree with the factors that produced them, which is why `vio_gs_dm_hold_pose_updates` exists. A correction that acts only through residuals in the solve leaves the store consistent.

### 2.9 Comparison and what carries over

1. Visual inertial ORB-SLAM [1, 2]. The map is constant and the prior is discarded at a map change. Exact against the map, but it loses the window's history.
2. VINS-Mono relocalisation [3]. The map poses are constants inside the window's residuals and the prior is kept. Absolute targets fit a global frame but not a gauge free mapper.
3. VINS-Mono pose graph [3]. Relative residuals acting only on the drifting directions.
4. OKVIS2 [5]. Summaries between frames are relative pose factors with full information, and the full graph result replaces the real time states.
5. ICE-BA [4, 11]. A star prior on relative coordinates with dense information makes a global correction free along the gauge directions. An overwrite with a rigid carry of local frames is safe because the local BA holds keyframes constant and carries only a body frame motion prior, while the pose priors live in the global BA and are relinearised at its current estimate. Note, 2026-11-19. This item previously attributed the safety of the overwrite to the relinearisation alone.
6. Basalt NFR [6]. Relative and roll pitch factors recovered from the odometry's priors, so the mapper's estimate is correlated with the odometry. Each packet adds a star centred on the departing keyframe, and the stars together approach a complete graph per window.
7. DM-VIO [7]. A delayed store requires that nothing moves a recorded state outside the solve.

The estimator should keep its prior, as VINS-Mono does and ORB-SLAM does not. It should treat the mapper's poses as constants inside its own residuals, as VINS-Mono does. It should compare relative poses, as the 4 DOF graph, OKVIS2, ICE-BA and Basalt's own factors do, because the mapper's absolute position and yaw are free, and it should reuse Basalt's relative pose factor, its residual and its accumulation rather than introduce a second relative residual. Which pairs to compare has precedent both ways, a star from one reference in ICE-BA and in each NFR packet, and all pairs in the union of NFR's stars, so both are implemented and compared (§3.4, §3.5). It should not fold the term into the prior unweighted, because the mapper reuses the odometry's information. It should act only through the solve, so that the delayed store stays consistent.

## 3. Formulation

### 3.1 The constant

At every mapper cycle the estimator stores, per keyframe timestamp $k$, the mapper's pose $\hat{\mathbf{T}}_k$ and the correction epoch it was expressed in. The map keeps only the newest estimate per timestamp and drops entries older than the window. Before each optimisation every entry is carried to the latest epoch with `GravityScaleCorrectionLog::Between` (`doc/VIO.md` §3.3.6), so the constants live in the metric, gravity aligned world in which the window starts every solve.

### 3.2 Residual

The residual is Basalt's relative pose factor of §2.7.4, extended by the scale of the gravity and scale period. Note, 2026-11-18. This section previously defined a separate residual in the frame of an anchor $a$, $\mathbf{r}_t = s\,\mathbf{R}_a^\top(\mathbf{p}_k - \mathbf{p}_a) - \hat{\mathbf{t}}_{ak}$ and $\mathbf{r}_R = \text{Log}(\mathbf{R}_a^\top \mathbf{R}_k \hat{\mathbf{R}}_{ak}^\top)$, built by a new function `LinearizeLocalMapPair`. It was replaced so that the window and the mapper share one relative residual, its Jacobians and its tests. Both forms vanish on the same set of poses and are invariant to the same gauges.

Let $\mathcal{K}$ be the timestamps present both in the map of §3.1 and in the window's order map, with $N = |\mathcal{K}|$. A pair $(i, j)$ of $\mathcal{K}$ with $i < j$ is stored as a `RelPoseFactor` with `t_i_ns` $= i$, `t_j_ns` $= j$ and the measurement taken from the mapper's carried poses,

$$\hat{\mathbf{T}}_{ij} = \hat{\mathbf{T}}_i^{-1} \hat{\mathbf{T}}_j,$$

that is $\hat{\mathbf{R}}_{ij} = \hat{\mathbf{R}}_i^\top \hat{\mathbf{R}}_j$ and $\hat{\mathbf{t}}_{ij} = \hat{\mathbf{R}}_i^\top(\hat{\mathbf{p}}_j - \hat{\mathbf{p}}_i)$. Which pairs are formed is the subject of §3.4.

In the gravity and scale period the window world $D$ relates to the metric world $M$ by $\mathbf{p}^M = s\,\mathbf{R}_{wg}^\top \mathbf{p}^D$ and $\mathbf{R}^M = \mathbf{R}_{wg}^\top \mathbf{R}^D$ with $s = e^\sigma$ (`doc/VIO.md` §3.3.1). The relative pose $\mathbf{T}_j^{-1} \mathbf{T}_i$ that `relPoseError` compares is then, in the metric world,

$$\mathbf{R}_j^{M\top} \mathbf{R}_i^M = \mathbf{R}_j^\top \mathbf{R}_{wg} \mathbf{R}_{wg}^\top \mathbf{R}_i = \mathbf{R}_j^\top \mathbf{R}_i,$$

$$\mathbf{R}_j^{M\top}(\mathbf{p}_i^M - \mathbf{p}_j^M) = \mathbf{R}_j^\top \mathbf{R}_{wg}\,s\,\mathbf{R}_{wg}^\top(\mathbf{p}_i - \mathbf{p}_j) = s\,\mathbf{R}_j^\top(\mathbf{p}_i - \mathbf{p}_j).$$

The gravity direction cancels and only the scale remains, as in the visual residual (`doc/VIO.md` §3.3.2). Outside the period $s = 1$. Substituting into §2.7.4, with $\mathbf{u} = \mathbf{p}_i - \mathbf{p}_j$, the residual is

$$\mathbf{r}_t = s\,\hat{\mathbf{R}}_{ij} \mathbf{R}_j^\top \mathbf{u} + \hat{\mathbf{t}}_{ij},$$

$$\mathbf{r}_R = \text{Log}\big(\hat{\mathbf{R}}_{ij} \mathbf{R}_j^\top \mathbf{R}_i\big),$$

and at $s = 1$ it equals `relPoseError` exactly. Each pair's six rows are whitened by $\boldsymbol{\Lambda}^{\frac{1}{2}}$ for the information $\boldsymbol{\Lambda}$ stored in the factor's `cov_inv`, which §3.4 sets per topology.

### 3.3 Jacobians

The translation rows of §2.7.4 carry the factor $s$, because $\mathbf{u}$ enters $\mathbf{r}_t$ only through $s\,\mathbf{u}$. With $\mathbf{E} = \text{Exp}(\mathbf{r}_R)$ as in §2.7.4, Steps 1 to 3 of §2.7.4 give

$$\frac{\partial\mathbf{r}_t}{\partial\delta\mathbf{p}_i} = s\,\mathbf{E} \mathbf{R}_i^\top,$$

$$\frac{\partial\mathbf{r}_t}{\partial\delta\mathbf{p}_j} = -s\,\mathbf{E} \mathbf{R}_i^\top,$$

$$\frac{\partial\mathbf{r}_t}{\partial\delta\boldsymbol{\varphi}_j} = -s\,\mathbf{E}\,[\mathbf{R}_i^\top(\mathbf{p}_j - \mathbf{p}_i)]_\times \mathbf{R}_i^\top,$$

and the rotation rows are unchanged,

$$\frac{\partial\mathbf{r}_R}{\partial\delta\boldsymbol{\varphi}_i} = \mathbf{J}_r^{-1}(\mathbf{r}_R) \mathbf{R}_i^\top,$$

$$\frac{\partial\mathbf{r}_R}{\partial\delta\boldsymbol{\varphi}_j} = -\mathbf{J}_r^{-1}(\mathbf{r}_R) \mathbf{R}_i^\top.$$

In code this is the stacked form of §2.7.4 Step 4 with the top row of the second factor, the one acting on translation, multiplied by $s$. For the log scale, $\frac{\partial s}{\partial\sigma} = s$ gives

$$\frac{\partial\mathbf{r}_t}{\partial\delta\sigma} = s\,\hat{\mathbf{R}}_{ij} \mathbf{R}_j^\top \mathbf{u} = \mathbf{r}_t - \hat{\mathbf{t}}_{ij},$$

$$\frac{\partial\mathbf{r}_R}{\partial\delta\sigma} = \mathbf{0},$$

and the residual does not depend on the tilt increment $\boldsymbol{\delta}_\theta$. This is the pattern of `computeRelPoseGS` (`include/basalt/utils/ba_utils.h:82-133`), whose `Ds` scales the translation block and whose `d_rel_d_logscale` is the scaled relative translation.

Residuals are evaluated at current values and Jacobians at first estimates ($\mathbf{T}_i$, $\mathbf{T}_j$ from `getPoseLin`, $s$ from `GetScaleLin`), the convention of every other factor of the window (`context/vio_opt.md` §11.6) and of `linearizeHelperStatic` (§2.7.4). The Jacobians therefore use $\mathbf{r}_R$ and $\mathbf{E}$ of the linearisation point. The velocity, bias and tilt columns are zero.

### 3.4 Two topologies

Two implementations differ only in which pairs of $\mathcal{K}$ they form and in the weight they give each pair. Both use the residual of §3.2, isotropic informations

$$\boldsymbol{\Lambda} = \text{diag}(w_p^2 \mathbf{I}_3, w_R^2 \mathbf{I}_3),$$

and the standard deviations $\sigma_p$ = `vio_local_map_pos_std` and $\sigma_R$ = `vio_local_map_rot_std_deg` in radians.

1. Anchor, `LocalMapRows`. With $a = \min \mathcal{K}$ the oldest keyframe the map and the window share, the $N - 1$ pairs $(a, k)$ for $k \in \mathcal{K} \setminus \{a\}$, with $w_p = \frac{1}{\sigma_p \sqrt{N - 1}}$ and $w_R = \frac{1}{\sigma_R \sqrt{N - 1}}$. This is the formulation of the previous revision of this plan in the residual of §3.2, and the topology of one NFR packet and of ICE-BA's prior.
2. Pairwise, `LocalMapRows2`. All $\frac{N(N - 1)}{2}$ pairs $(i, j)$ with $i < j$, with $w_p = \frac{1}{\sigma_p N}$ and $w_R = \frac{1}{\sigma_R N}$. This is the topology NFR's stars approach over a run (§2.7.5), restricted to the window.

`LocalMapRows` picks its anchor through the order of `mvpLocalMapPoses`, a `std::map` keyed by timestamp, whose first element present in the window is the oldest shared keyframe. Its choice of anchor is therefore fixed by the iteration order, and changes whenever that keyframe leaves the window. `LocalMapRows2` does not depend on the order. Both sets of weights make the cost a mean over keyframes rather than a sum over pairs, so the total pull does not grow with the overlap, as §3.5.1 shows.

### 3.5 Mathematical basis of the comparison

The two topologies are compared in a linearised model that isolates their difference, followed by the effects the model does not capture.

#### 3.5.1 The two costs as mean squared deviations

Assume for this section that the window's rotations equal the mapper's, $\mathbf{R}_k = \hat{\mathbf{R}}_k$, and $s = 1$. Express the map in the window's world by any rigid motion, which changes no residual by §3.6, and define the deviation of keyframe $k$ as

$$\mathbf{d}_k = \mathbf{p}_k - \hat{\mathbf{p}}_k.$$

Step 1. With $\mathbf{R}_k = \hat{\mathbf{R}}_k$ the measured rotation gives $\hat{\mathbf{R}}_{ij} \mathbf{R}_j^\top = \hat{\mathbf{R}}_i^\top \hat{\mathbf{R}}_j \hat{\mathbf{R}}_j^\top = \hat{\mathbf{R}}_i^\top$, so §3.2 becomes

$$\mathbf{r}_t = \hat{\mathbf{R}}_i^\top(\mathbf{p}_i - \mathbf{p}_j) + \hat{\mathbf{R}}_i^\top(\hat{\mathbf{p}}_j - \hat{\mathbf{p}}_i) = \hat{\mathbf{R}}_i^\top(\mathbf{d}_i - \mathbf{d}_j),$$

and $|\mathbf{r}_t| = |\mathbf{d}_i - \mathbf{d}_j|$ because a rotation preserves length.

Step 2. The anchor cost is therefore

$$E_1 = \frac{1}{2}\sum_{k \neq a} w_p^2 |\mathbf{d}_k - \mathbf{d}_a|^2 = \frac{1}{2\sigma_p^2} \cdot \frac{1}{(N - 1)}\sum_{k \neq a} |\mathbf{d}_k - \mathbf{d}_a|^2,$$

the mean squared deviation of the keyframes measured from the anchor's deviation.

Step 3. The pairwise cost is built in four statements.

Step 3a. By Step 1 each pair $(i, j)$ contributes $\frac{1}{2} w_p^2 |\mathbf{d}_j - \mathbf{d}_i|^2$, so summing over the $\frac{N(N - 1)}{2}$ pairs of §3.4 item 2 gives

$$E_2 = \frac{1}{2}\sum_{i < j} w_p^2 |\mathbf{d}_j - \mathbf{d}_i|^2.$$

Step 3b. The weight is the same for every pair, $w_p^2 = \frac{1}{\sigma_p^2 N^2}$ by §3.4, so it leaves the sum,

$$E_2 = \frac{1}{2\sigma_p^2 N^2}\sum_{i < j} |\mathbf{d}_j - \mathbf{d}_i|^2.$$

Step 3c. The remaining sum is over differences, and (I9) rewrites it as a sum over keyframes. Its proof, written out for these vectors, has four lines. First, $|\mathbf{d}_j - \mathbf{d}_i|^2$ is symmetric in $i$ and $j$ and vanishes for $i = j$, so summing over all ordered pairs counts every unordered pair twice,

$$\sum_{i < j} |\mathbf{d}_j - \mathbf{d}_i|^2 = \frac{1}{2}\sum_{i = 1}^{N}\sum_{j = 1}^{N} |\mathbf{d}_j - \mathbf{d}_i|^2.$$

Second, each term expands as $|\mathbf{d}_j|^2 - 2\mathbf{d}_i^\top \mathbf{d}_j + |\mathbf{d}_i|^2$. Summed over both indices, the first and last pieces each give $N \sum_k |\mathbf{d}_k|^2$, and the middle piece gives $2(\sum_i \mathbf{d}_i)^\top(\sum_j \mathbf{d}_j) = 2N^2 |\bar{\mathbf{d}}|^2$, using $\sum_k \mathbf{d}_k = N\bar{\mathbf{d}}$, so

$$\sum_{i < j} |\mathbf{d}_j - \mathbf{d}_i|^2 = \frac{1}{2}\big(2N \sum_k |\mathbf{d}_k|^2 - 2N^2 |\bar{\mathbf{d}}|^2\big) = N \sum_k |\mathbf{d}_k|^2 - N^2 |\bar{\mathbf{d}}|^2.$$

Third, the spread about the mean expands as $|\mathbf{d}_k - \bar{\mathbf{d}}|^2 = |\mathbf{d}_k|^2 - 2\bar{\mathbf{d}}^\top \mathbf{d}_k + |\bar{\mathbf{d}}|^2$, and summing over $k$ with the same substitution gives

$$\sum_k |\mathbf{d}_k - \bar{\mathbf{d}}|^2 = \sum_k |\mathbf{d}_k|^2 - 2N |\bar{\mathbf{d}}|^2 + N |\bar{\mathbf{d}}|^2 = \sum_k |\mathbf{d}_k|^2 - N |\bar{\mathbf{d}}|^2.$$

Fourth, $N$ times the third line is the second, so

$$\sum_{i < j} |\mathbf{d}_j - \mathbf{d}_i|^2 = N \sum_k |\mathbf{d}_k - \bar{\mathbf{d}}|^2.$$

Step 3d. Substituting Step 3c into Step 3b, and cancelling one $N$,

$$E_2 = \frac{1}{2\sigma_p^2 N^2} \cdot N \sum_k |\mathbf{d}_k - \bar{\mathbf{d}}|^2 = \frac{1}{2\sigma_p^2} \cdot \frac{1}{N}\sum_k |\mathbf{d}_k - \bar{\mathbf{d}}|^2.$$

The factor $\frac{1}{N}\sum_k |\mathbf{d}_k - \bar{\mathbf{d}}|^2$ is the mean squared deviation of the keyframes measured from their mean deviation, and the weight $\frac{1}{\sigma_p^2 N^2}$ of §3.4 was chosen so that exactly one factor $\frac{1}{N}$ survives and makes it a mean. The mean is the best common offset. For any constant $\mathbf{c}$, the derivative of $\sum_k |\mathbf{d}_k - \mathbf{c}|^2$ with respect to $\mathbf{c}$ is $-2\sum_k(\mathbf{d}_k - \mathbf{c})$, which vanishes only at $\mathbf{c} = \bar{\mathbf{d}}$. $E_2$ is therefore the mean squared residual between the window and the local map that §1 asks for, after the one common translation that best aligns them, which is the part the relative residuals cannot see. The offset is removed optimally rather than by fixing it at one keyframe. Note, 2026-11-19. This step previously stated the result of Steps 3b to 3d as one chain of equalities citing (I9).

Step 4. A small rotation deviation $\mathbf{R}_k = \text{Exp}(\boldsymbol{\phi}_k) \hat{\mathbf{R}}_k$ gives, by (I1) and (I2), $\mathbf{r}_R \approx \hat{\mathbf{R}}_i^\top(\boldsymbol{\phi}_i - \boldsymbol{\phi}_j)$, the same difference structure, so Steps 2 and 3 hold for the rotation rows with $\boldsymbol{\phi}_k$ in place of $\mathbf{d}_k$ and $\sigma_R$ in place of $\sigma_p$.

Both costs vanish exactly when all $\mathbf{d}_k$ are equal, that is when the window's constellation equals the map's up to the gauge. The measurements of every pair come from one set of mapper poses, so they agree around every cycle, and the two topologies share their minimiser when nothing else acts. They differ only in how stiffly they resist each way of departing from it, which matters once the visual, inertial and prior terms pull elsewhere.

Step 5. The two costs differ by one term. The anchor cost of Step 2 includes the anchor itself at no cost, since $\mathbf{d}_a - \mathbf{d}_a = \mathbf{0}$, so its sum runs equally over all $k$. Expanding about the mean, with $\mathbf{d}_k - \mathbf{d}_a = (\mathbf{d}_k - \bar{\mathbf{d}}) + (\bar{\mathbf{d}} - \mathbf{d}_a)$, and noting that the cross term vanishes because $\sum_k (\mathbf{d}_k - \bar{\mathbf{d}}) = \mathbf{0}$,

$$\sum_k |\mathbf{d}_k - \mathbf{d}_a|^2 = \sum_k |\mathbf{d}_k - \bar{\mathbf{d}}|^2 + N |\mathbf{d}_a - \bar{\mathbf{d}}|^2.$$

Substituting into Step 2 and using Step 3d,

$$E_1 = \frac{N}{(N - 1)} E_2 + \frac{N}{2\sigma_p^2 (N - 1)} |\mathbf{d}_a - \bar{\mathbf{d}}|^2.$$

The anchor topology is the pairwise topology, enlarged by $\frac{N}{N - 1}$, which is $1.17$ at $N = 7$, plus a penalty on how far the anchor's deviation lies from the mean deviation. That penalty is the whole of the difference between the topologies, and §3.5.2 shows it as the anchor mode.

#### 3.5.2 Information matrices

The solver does not see the costs of §3.5.1 but their Hessians, the information the term adds to the window's normal equations. In the model of §3.5.1 the translation rows of a pair depend on the positions only through $\mathbf{d}_j - \mathbf{d}_i$, rotated by $\hat{\mathbf{R}}_i^\top$, which preserves length. The three axes therefore separate, and per axis, with $\mathbf{d}$ now the vector of the $N$ keyframes' deviations along that axis, each cost is a quadratic $E = \frac{1}{2}\mathbf{d}^\top \mathbf{H} \mathbf{d}$. The matrix $\mathbf{H}$ is the restriction to these positions of $\mathbf{J}^\top \mathbf{J}$ for the rows `RelPoseFactorRows` writes, which Change 6 adds to the window's system, and it has the structure of a graph Laplacian [12]. The steps below define the Laplacian, compute it for both topologies, and read off what each topology claims to know.

Step 1. The graph. Its nodes are the $N$ keyframes of $\mathcal{K}$, and it has one edge for every pair that carries a residual. The anchor topology is the star, with $N - 1$ edges from $a$ to every other keyframe. The pairwise topology is the complete graph, with all $\frac{N(N - 1)}{2}$ edges.

Step 2. The incidence matrix. $\mathbf{D}$ has one row per edge and one column per node. The row of the edge $(i, j)$ holds $-1$ in column $i$, $+1$ in column $j$ and zeros elsewhere, so

$$(\mathbf{D} \mathbf{d})_{(i, j)} = d_j - d_i,$$

and $\mathbf{D} \mathbf{d}$ stacks the differences that Step 1 of §3.5.1 shows each pair to measure.

Step 3. The Laplacian is defined as

$$\mathbf{L} = \mathbf{D}^\top \mathbf{D}.$$

Its entries follow from $(\mathbf{D}^\top \mathbf{D})_{kl} = \sum_e D_{ek} D_{el}$, a sum over edges. On the diagonal, every edge at $k$ contributes $(\pm 1)^2 = 1$, so $L_{kk}$ is the degree of $k$, the number of its edges. Off the diagonal, only the edge joining $k$ and $l$, if it exists, has both entries nonzero, and it contributes $(+1)(-1) = -1$. The Laplacian is thus the degree matrix minus the adjacency matrix, a form that does not depend on the orientation chosen for the edges.

Step 4. The quadratic form. For any $\mathbf{d}$,

$$\mathbf{d}^\top \mathbf{L} \mathbf{d} = |\mathbf{D} \mathbf{d}|^2 = \sum_{(i, j)} (d_j - d_i)^2,$$

the sum of the squared differences over the edges. A cost that sums $\frac{1}{2} w^2 (d_j - d_i)^2$ over the edges with one weight $w$ is therefore $\frac{1}{2} w^2 \mathbf{d}^\top \mathbf{L} \mathbf{d}$, and its Hessian is $\mathbf{H} = w^2 \mathbf{L}$.

Step 5. Three properties. $\mathbf{L}$ is symmetric and positive semidefinite, as any $\mathbf{D}^\top \mathbf{D}$ is, because $\mathbf{x}^\top \mathbf{D}^\top \mathbf{D} \mathbf{x} = |\mathbf{D} \mathbf{x}|^2 \geq 0$. Every row of $\mathbf{D}$ holds one $+1$ and one $-1$, so for the all ones vector $\boldsymbol{\iota}_N$ of (I10) each entry of $\mathbf{D} \boldsymbol{\iota}_N$ is $1 - 1 = 0$, which gives $\mathbf{D} \boldsymbol{\iota}_N = \mathbf{0}$ and then $\mathbf{L} \boldsymbol{\iota}_N = \mathbf{D}^\top \mathbf{0} = \mathbf{0}$. This says that moving every keyframe by the same amount costs nothing, the translation gauge of §3.6. If the graph is connected, $\boldsymbol{\iota}_N$ is the only such direction, because $\mathbf{d}^\top \mathbf{L} \mathbf{d} = 0$ forces $d_i = d_j$ along every edge, and a path of edges joins any two keyframes. Every other direction has a positive eigenvalue.

Step 6. The star. An eigenvector of $\mathbf{L}$ is a nonzero vector $\mathbf{x}$ with $\mathbf{L} \mathbf{x} = \lambda\mathbf{x}$ for a scalar $\lambda$, its eigenvalue. The eigenvectors of the star are found by writing $\mathbf{L}_\text{star}$ out, applying it to a candidate vector, and comparing the result with the candidate entry by entry.

Step 6a. The matrix. Order the keyframes with the anchor $a$ first. The anchor is joined to all $N - 1$ other keyframes, and every other keyframe is joined only to the anchor. By Step 3 the diagonal holds the degrees, $N - 1$ for the anchor and $1$ for every other keyframe, and the off diagonal holds $-1$ between the anchor and each other keyframe and $0$ between two non anchor keyframes, which share no edge. In blocks,

$$\mathbf{L}_\text{star} = \begin{bmatrix} N - 1 & -\boldsymbol{\iota}_{N - 1}^\top \\ -\boldsymbol{\iota}_{N - 1} & \mathbf{I} \end{bmatrix},$$

with $\mathbf{I}$ the identity of size $N - 1$. For $N = 3$, with the anchor as keyframe $1$,

$$\mathbf{L}_\text{star} = \begin{bmatrix} 2 & -1 & -1 \\ -1 & 1 & 0 \\ -1 & 0 & 1 \end{bmatrix}.$$

Step 6b. The two row formulas. Multiplying the first block row and any other row of Step 6a by a vector $\mathbf{x}$ gives the entry at the anchor and the entry at any non anchor keyframe $k$,

$$(\mathbf{L}_\text{star} \mathbf{x})_a = (N - 1)x_a - \sum_{m \neq a} x_m,$$

$$(\mathbf{L}_\text{star} \mathbf{x})_k = x_k - x_a.$$

Every candidate below is checked with these two formulas alone.

Step 6c. The eigenvalue $0$. Take $\mathbf{x} = \boldsymbol{\iota}_N$, whose every entry is $1$. The anchor's formula gives $(N - 1) \cdot 1 - (N - 1) \cdot 1 = 0$, and the formula of every other keyframe gives $1 - 1 = 0$. Hence $\mathbf{L}_\text{star} \boldsymbol{\iota}_N = \mathbf{0} = 0 \cdot \boldsymbol{\iota}_N$, so the all ones vector is an eigenvector of eigenvalue $0$. It is the uniform shift of Step 5, which the star, like any connected graph, cannot see. For $N = 3$, $\mathbf{L}_\text{star} (1, 1, 1)^\top = (2 - 1 - 1, -1 + 1, -1 + 1)^\top = (0, 0, 0)^\top$.

Step 6d. The eigenvalue $1$. Let $\mathbf{e}_m$ be the $m$th standard basis vector of $\mathbb{R}^N$, which holds $1$ in entry $m$ and $0$ in every other entry. For two distinct non anchor keyframes $k$ and $l$, the vector $\mathbf{x} = \mathbf{e}_k - \mathbf{e}_l$ displaces keyframe $k$ by $+1$ and keyframe $l$ by $-1$ and leaves the anchor and every other keyframe in place, so $x_k = 1$, $x_l = -1$, and $x_a = 0$. The anchor's formula gives $(N - 1) \cdot 0 - (1 - 1) = 0 = x_a$. The formula of $k$ gives $1 - 0 = 1 = x_k$, that of $l$ gives $-1 - 0 = -1 = x_l$, and that of any other non anchor keyframe $m$ gives $0 - 0 = 0 = x_m$. Hence $\mathbf{L}_\text{star} \mathbf{x} = 1 \cdot \mathbf{x}$, an eigenvector of eigenvalue $1$. For $N = 3$, $\mathbf{e}_2 - \mathbf{e}_3 = (0, 1, -1)^\top$, and $\mathbf{L}_\text{star} (0, 1, -1)^\top = (0 - 1 + 1, 1 - 0, -1 - 0)^\top = (0, 1, -1)^\top$.

To count them, fix one non anchor keyframe $l$ and let $k$ run over the other $N - 2$ non anchor keyframes. Each of the $N - 2$ vectors $\mathbf{e}_k - \mathbf{e}_l$ is the only one of them with a nonzero entry at its own $k$, so none is a combination of the others, and they are linearly independent. The same two formulas show more generally that every $\mathbf{x}$ with $x_a = 0$ and $\sum_m x_m = 0$ satisfies $\mathbf{L}_\text{star} \mathbf{x} = \mathbf{x}$. A mode of $\mathbf{H}$, or equivalently of $\mathbf{L}$ since $\mathbf{H}$ is a scalar multiple of $\mathbf{L}$ by Step 4, is one of its eigenvectors, a single pattern in which the window's keyframes can move together away from the map's constellation, and its eigenvalue is the stiffness of that pattern, the cost per unit squared amplitude of a displacement along it (made precise in Step 9). These $N - 2$ eigenvectors are therefore the modes that move the non anchor keyframes against one another while the anchor and their mean stay fixed, and each such keyframe resists through its single edge of unit weight.

Step 6e. The eigenvalue $N$. Let $\mathbf{v} = N\mathbf{e}_a - \boldsymbol{\iota}_N$, so $v_a = N - 1$ and $v_k = -1$ for every $k \neq a$. The anchor's formula gives

$$(\mathbf{L}_\text{star} \mathbf{v})_a = (N - 1)(N - 1) - (N - 1)(-1) = N(N - 1) = N v_a,$$

and the formula of any other $k$ gives

$$(\mathbf{L}_\text{star} \mathbf{v})_k = v_k - v_a = -1 - (N - 1) = -N = N v_k,$$

so $\mathbf{v}$ is an eigenvector of eigenvalue $N$. For $N = 3$, $\mathbf{L}_\text{star} (2, -1, -1)^\top = (4 + 1 + 1, -1 - 2, -1 - 2)^\top = 3 \cdot (2, -1, -1)^\top$. It is the anchor mode, the anchor moving one way and all other keyframes the other, with $\boldsymbol{\iota}_N^\top \mathbf{v} = (N - 1) - (N - 1) = 0$, so it carries no uniform shift.

Step 6f. Completeness. Steps 6c to 6e give $1 + (N - 2) + 1 = N$ independent eigenvectors in $\mathbb{R}^N$. They form a basis, so the spectrum of the star is $0$ once, $1$ with multiplicity $N - 2$, and $N$ once, as (I11) states.

Step 7. The complete graph. Every keyframe is joined to every other, so every degree is $N - 1$ and every off diagonal entry is $-1$. The adjacency matrix therefore holds $1$ in every off diagonal entry and $0$ on the diagonal, which is the all ones matrix with its diagonal removed, $\mathbf{K}_N - \mathbf{I}$. By Step 3 the Laplacian is the degree matrix minus the adjacency matrix,

$$\mathbf{L}_\text{complete} = (N - 1)\mathbf{I} - (\mathbf{K}_N - \mathbf{I}) = N\mathbf{I} - \mathbf{K}_N = N\Big(\mathbf{I} - \frac{\mathbf{K}_N}{N}\Big).$$

For $N = 3$,

$$\mathbf{L}_\text{complete} = \begin{bmatrix} 2 & -1 & -1 \\ -1 & 2 & -1 \\ -1 & -1 & 2 \end{bmatrix}.$$

Step 7a. The centring projector. Define

$$\mathbf{P} = \mathbf{I} - \frac{\mathbf{K}_N}{N}.$$

Applied to any $\mathbf{x}$, the all ones matrix gives $\mathbf{K}_N \mathbf{x} = \boldsymbol{\iota}_N (\boldsymbol{\iota}_N^\top \mathbf{x}) = (\sum_m x_m)\boldsymbol{\iota}_N$, the vector whose every entry is the sum of the entries of $\mathbf{x}$. Dividing by $N$ turns the sum into the mean $\bar{x} = \frac{1}{N}\sum_m x_m$, so

$$(\mathbf{P} \mathbf{x})_k = x_k - \bar{x}.$$

$\mathbf{P}$ subtracts the mean, and applied to the deviations it gives $(\mathbf{P} \mathbf{d})_k = d_k - \bar{d}$.

Step 7b. Its eigenvectors. The mean of $\boldsymbol{\iota}_N$ is $1$, so $\mathbf{P} \boldsymbol{\iota}_N = \boldsymbol{\iota}_N - \boldsymbol{\iota}_N = \mathbf{0}$, the eigenvalue $0$. Any $\mathbf{x}$ with $\boldsymbol{\iota}_N^\top \mathbf{x} = 0$ has mean $0$, so $\mathbf{P} \mathbf{x} = \mathbf{x}$, the eigenvalue $1$ on the $N - 1$ independent directions orthogonal to $\boldsymbol{\iota}_N$. Using $\mathbf{K}_N^2 = N\mathbf{K}_N$ from (I10),

$$\mathbf{P}^2 = \mathbf{I} - \frac{2\mathbf{K}_N}{N} + \frac{N\mathbf{K}_N}{N^2} = \mathbf{I} - \frac{\mathbf{K}_N}{N} = \mathbf{P},$$

so subtracting the mean twice is the same as subtracting it once, and $\mathbf{P}$ is symmetric, which together make it the orthogonal projector onto the directions orthogonal to $\boldsymbol{\iota}_N$.

Step 7c. The spectrum. $\mathbf{L}_\text{complete} = N\mathbf{P}$ by Step 7, and multiplying a matrix by a scalar multiplies its eigenvalues by that scalar and keeps its eigenvectors. $\mathbf{L}_\text{complete}$ therefore has the eigenvalue $0$ on $\boldsymbol{\iota}_N$ and $N$ on every one of the $N - 1$ directions orthogonal to it. Note, 2026-11-19. Steps 1 to 7 were added to derive (I11) and the relation between $\mathbf{L}_\text{complete}$ and the projector, which this section previously stated without derivation. Note, 2026-11-20. Steps 7a to 7c were split out of one paragraph of Step 7, and $\mathbf{K}_N$ replaced $\mathbf{1} \mathbf{1}^\top$.

Step 8. The information matrices. Step 4 says that a cost with one weight $w_p$ on every edge has the Hessian $w_p^2 \mathbf{L}$. The anchor topology has $w_p^2 = \frac{1}{(N - 1)\sigma_p^2}$ on the star by §3.4, so

$$\mathbf{H}_1 = \frac{1}{(N - 1)\sigma_p^2} \mathbf{L}_\text{star}.$$

The pairwise topology has $w_p^2 = \frac{1}{N^2 \sigma_p^2}$ on the complete graph, and substituting $\mathbf{L}_\text{complete} = N\mathbf{P}$ from Step 7 cancels one $N$,

$$\mathbf{H}_2 = \frac{1}{N^2 \sigma_p^2} \mathbf{L}_\text{complete} = \frac{1}{N^2 \sigma_p^2} N\mathbf{P} = \frac{1}{N \sigma_p^2}\mathbf{P}.$$

As a check, $\mathbf{P}$ is symmetric with $\mathbf{P}^2 = \mathbf{P}$ by Step 7b, so $\mathbf{d}^\top \mathbf{P} \mathbf{d} = \mathbf{d}^\top \mathbf{P}^\top \mathbf{P} \mathbf{d} = |\mathbf{P} \mathbf{d}|^2$, and by Step 7a

$$\frac{1}{2}\mathbf{d}^\top \mathbf{H}_2 \mathbf{d} = \frac{1}{2N\sigma_p^2}|\mathbf{P} \mathbf{d}|^2 = \frac{1}{2N\sigma_p^2}\sum_k (d_k - \bar{d})^2,$$

which is $E_2$ of §3.5.1 Step 3d along one axis. These are the blocks the term adds over the shared keyframes' positions, repeated for each axis, and the rotation rows add the same matrices with $\sigma_R$ in place of $\sigma_p$ (§3.5.1 Step 4). Their common null vector $\boldsymbol{\iota}_N$ means that neither adds information to the gauge, as §3.6 requires.

Step 9. The spectra. The window's Gauss Newton step solves normal equations to which Change 6 adds $\mathbf{H}$ beside the information of the visual, inertial and prior terms (§2.2.2), so everything the term does to a step is contained in $\mathbf{H}$. A symmetric matrix is described completely by its eigenvectors and eigenvalues, together its spectrum. Each eigenvector is a mode in the sense of Step 6d, one pattern in which the window's keyframes can depart together from the map's constellation, and its eigenvalue is the cost of that departure per unit squared amplitude, since for a unit eigenvector $\mathbf{u}$ the cost of $\alpha\mathbf{u}$ is $\frac{1}{2}(\alpha\mathbf{u})^\top \mathbf{H} (\alpha\mathbf{u}) = \frac{1}{2}\lambda\alpha^2$. The spectrum therefore lists, mode by mode, how firmly the term holds the window to the map's shape. The two topologies can be compared mode by mode because their matrices share eigenvectors. $\mathbf{P}$ leaves every vector orthogonal to $\boldsymbol{\iota}_N$ unchanged (Step 7b), so the eigenvectors of $\mathbf{L}_\text{star}$ of Steps 6d and 6e are eigenvectors of $\mathbf{P}$ too, with eigenvalue $1$, and comparing the topologies reduces to comparing two numbers on each of the same $N - 1$ modes.

Each $\mathbf{H}$ of Step 8 is a positive scalar times a Laplacian, so it keeps the Laplacian's eigenvectors and scales its eigenvalues by that scalar. Scaling the spectrum of Step 7c by $\frac{1}{N^2\sigma_p^2}$, $\mathbf{H}_2$ has the eigenvalue $\frac{N}{N^2\sigma_p^2} = \frac{1}{N\sigma_p^2}$ on all $N - 1$ observable modes. Scaling the spectrum of Step 6f by $\frac{1}{(N - 1)\sigma_p^2}$, $\mathbf{H}_1$ has $\frac{1}{(N - 1)\sigma_p^2}$ on the $N - 2$ modes between non anchor keyframes of Step 6d and $\frac{N}{(N - 1)\sigma_p^2}$ on the anchor mode of Step 6e. Step 10 reads each eigenvalue as the stiffness of a spring, Step 11 compares the stiffnesses of the two topologies mode by mode, and the paragraph on the trace that follows Step 11 compares their sums. Note, 2026-11-20. The first paragraph of this step was added to state why the spectra are computed and why they can be compared mode by mode.

Step 10. Stiffness. Let $\mathbf{u}$ be a unit eigenvector of $\mathbf{H}$ orthogonal to $\boldsymbol{\iota}_N$, with eigenvalue $\lambda$. Displacing the window's keyframes from the map by $\alpha\mathbf{u}$ costs $E = \frac{1}{2}\lambda\alpha^2$, and the term pulls back with the gradient $\lambda\alpha\mathbf{u}$, so $\lambda$ acts as the constant of a spring along $\mathbf{u}$, and a larger $\lambda$ is a stiffer spring. In the probabilistic reading of §2.2.1, $\lambda$ is the information the term claims along $\mathbf{u}$, and $\frac{1}{\lambda}$ is the variance it allows the window along that mode. Its effect on the solution follows when another part of the window, with information $\lambda_o$ along $\mathbf{u}$, prefers a displacement $\Delta$ from the map. The window settles at the minimum of $\frac{1}{2}\lambda\alpha^2 + \frac{1}{2}\lambda_o(\alpha - \Delta)^2$,

$$\alpha = \frac{\lambda_o}{\lambda + \lambda_o}\Delta,$$

so it moves toward the map by the fraction $\frac{\lambda}{\lambda + \lambda_o}$ of the disagreement. The stiffer the term along a mode, the more of the map's shape it imposes along that mode against the visual, inertial and prior terms. The balance is exact when $\mathbf{u}$ is also an eigenvector of the other terms' information, and otherwise it is the one dimensional picture of a coupled system, which still orders the topologies correctly because both share the same modes.

The paragraph above takes $\mathbf{u}$ to be a unit eigenvector so that $\lambda = \mathbf{u}^\top \mathbf{H} \mathbf{u}$ is a single number read off the spectrum, but the same quadratic form gives the cost of a displacement along any vector $\mathbf{v}$, eigenvector or not, which §3.5.4 item 1 uses for the scale direction $\mathbf{v}_k = \hat{\mathbf{p}}_k$. By §2.2.2 the cost near the map is $E(\mathbf{d}) = \frac{1}{2}\mathbf{d}^\top \mathbf{H} \mathbf{d}$ for every $\mathbf{d}$, with no restriction to eigenvectors. Restricting to the one parameter family $\mathbf{d}(\alpha) = \alpha\mathbf{v}$ for an arbitrary $\mathbf{v}$ and expanding the product gives

$$E(\alpha\mathbf{v}) = \frac{1}{2}\alpha^2(\mathbf{v}^\top \mathbf{H} \mathbf{v}),$$

so $\mathbf{v}^\top \mathbf{H} \mathbf{v}$ is the curvature of the cost along $\mathbf{v}$ whatever $\mathbf{v}$ is, and it reduces to $\lambda$ above exactly when $\mathbf{v} = \mathbf{u}$ is a unit eigenvector. Writing $\mathbf{v}$ in $\mathbf{H}$'s orthonormal eigenbasis $\{\mathbf{u}_i\}$ with eigenvalues $\lambda_i$ as $\mathbf{v} = \sum_i c_i \mathbf{u}_i$, orthonormality cancels the cross terms and gives

$$\mathbf{v}^\top \mathbf{H} \mathbf{v} = \sum_i c_i^2 \lambda_i,$$

a weighted sum of the stiffnesses of every mode $\mathbf{v}$ excites, weighted by how much of $\mathbf{v}$ lies along each one. The quadratic form therefore still reads as the information the term claims along $\mathbf{v}$ when $\mathbf{v}$ mixes modes, it is just that mixture's own weighted total rather than a single eigenvalue, and no eigendecomposition of $\mathbf{v}$ is needed to evaluate it, since $\mathbf{v}^\top \mathbf{H} \mathbf{v}$ is computed directly from $\mathbf{H}$ by substitution.

Step 11. The ratio of the stiffnesses. On the anchor mode the anchor topology has $\lambda = \frac{N}{(N - 1)\sigma_p^2}$ and the pairwise topology $\lambda = \frac{1}{N\sigma_p^2}$, so the first is stiffer by

$$\frac{N}{(N - 1)\sigma_p^2} \cdot N \sigma_p^2 = \frac{N^2}{N - 1},$$

which is $8.17$ for the seven keyframes of `vio_max_kfs` (`data/euroc_config.json:14`). On each of the other $N - 2$ modes the ratio is $\frac{1}{(N - 1)\sigma_p^2} \cdot N\sigma_p^2 = \frac{N}{N - 1}$, $1.17$, the enlargement found in §3.5.1 Step 5. The ratio $8.17$ means that the anchor topology charges $8.17$ times more for the same displacement along the anchor mode, claims a variance $8.17$ times smaller for it, a standard deviation $\frac{N}{\sqrt{N - 1}} = 2.86$ times smaller, and, by Step 10, pulls the window much further toward the map along that mode. For example, if the other terms have $\lambda_o$ equal to the pairwise stiffness $\frac{1}{N\sigma_p^2}$ on every mode, the pairwise topology moves the window toward the map by $\frac{1}{1 + 1} = 0.5$ of the disagreement on every mode. The anchor topology moves it by $\frac{1.17}{1.17 + 1} = 0.54$ on the $N - 2$ non anchor modes and by $\frac{8.17}{8.17 + 1} = 0.89$ on the anchor mode, so the shape of the map imposed on the window is the same up to $8$ percent except along the one mode that moves the anchor against the rest, where the anchor topology imposes nearly all of it.

The anchor mode has a direct reading. Moving the anchor alone is the displacement $\mathbf{d} = \mathbf{e}_a$. Rearranging $\mathbf{v} = N\mathbf{e}_a - \boldsymbol{\iota}_N$ of Step 6e gives $\mathbf{e}_a = \frac{1}{N}\mathbf{v} + \frac{1}{N}\boldsymbol{\iota}_N$, and the second part is a uniform shift, which costs nothing by Step 5, so the displacement is a pure anchor mode displacement. A quadratic form on a basis vector picks out one diagonal entry, $\mathbf{e}_m^\top \mathbf{H} \mathbf{e}_m = H_{mm}$. The anchor's diagonal entry of $\mathbf{L}_\text{star}$ is its degree $N - 1$ and that of $\mathbf{P}$ is $1 - \frac{1}{N}$, so by Step 8 the costs are $\mathbf{e}_a^\top \mathbf{H}_1 \mathbf{e}_a = \frac{N - 1}{(N - 1)\sigma_p^2} = \frac{1}{\sigma_p^2}$ and $\mathbf{e}_a^\top \mathbf{H}_2 \mathbf{e}_a = \frac{1}{N\sigma_p^2}\big(1 - \frac{1}{N}\big) = \frac{N - 1}{N^2\sigma_p^2}$, in the ratio $\frac{N^2}{N - 1}$. Moving any other keyframe $k$ alone meets its degree $1$ in $\mathbf{L}_\text{star}$ and the same diagonal $1 - \frac{1}{N}$ in $\mathbf{P}$, so it costs $\mathbf{e}_k^\top \mathbf{H}_1 \mathbf{e}_k = \frac{1}{(N - 1)\sigma_p^2}$ against the same $\frac{N - 1}{N^2\sigma_p^2}$, a ratio of only $\frac{N^2}{(N - 1)^2} = 1.36$. The anchor topology thus singles out one keyframe, the oldest shared one by the order of §3.4, and ties it to the map with a spring eight times stiffer than the pairwise topology does, while treating every other keyframe almost alike. This is the penalty on $|\mathbf{d}_a - \bar{\mathbf{d}}|$ of §3.5.1 Step 5 seen from the Hessian.

The total information. Steps 9 to 11 compare the topologies one mode at a time, and the trace compares them over all modes at once. The trace of a matrix is the sum of its diagonal entries and equals the sum of its eigenvalues, so the trace of $\mathbf{H}$ is the information the term claims summed over its modes, one number for how strongly the term as a whole pulls the window toward the map. The diagonal of $\mathbf{L}_\text{star}$ sums to $(N - 1) + (N - 1) \cdot 1 = 2(N - 1)$ and that of $\mathbf{P}$ to $N\big(1 - \frac{1}{N}\big) = N - 1$, so by Step 8 the traces are $\frac{2(N - 1)}{(N - 1)\sigma_p^2} = \frac{2}{\sigma_p^2}$ and $\frac{N - 1}{N\sigma_p^2}$. Summing the eigenvalues of Step 9 gives the same, $(N - 2)\frac{1}{(N - 1)\sigma_p^2} + \frac{N}{(N - 1)\sigma_p^2} = \frac{2}{\sigma_p^2}$ and $(N - 1)\frac{1}{N\sigma_p^2}$.

The role of the mean weighting of §3.4 is seen by removing it. Every edge adds $1$ to the diagonal entries of its two keyframes (Step 3), so the trace of a Laplacian is twice its number of edges, and a weight $w_p^2$ on every edge gives the trace $2 w_p^2$ times the number of edges. With the plain weight $w_p^2 = \frac{1}{\sigma_p^2}$ on every pair the traces would be $\frac{2(N - 1)}{\sigma_p^2}$ for the star and $\frac{N(N - 1)}{\sigma_p^2}$ for the complete graph, $\frac{12}{\sigma_p^2}$ and $\frac{42}{\sigma_p^2}$ at $N = 7$. Both would grow with the number of pairs, the complete graph's quadratically, and their ratio $\frac{N}{2}$ would grow with the window. The weights of §3.4 divide out the number of pairs, which leaves $\frac{2}{\sigma_p^2}$ for the star, independent of $N$, and $\frac{N - 1}{N\sigma_p^2}$ for the complete graph, which stays below $\frac{1}{\sigma_p^2}$. Their ratio is $\frac{2N}{N - 1}$, which is $3$ at $N = 3$, $2.33$ at $N = 7$ and tends to $2$ for large windows, so the mean weighting keeps the total information of the two topologies within a fixed small factor of each other whatever the size of the overlap.

For the comparison this has two consequences. First, the two topologies claim comparable total information, so their difference lies less in how much they claim than in where. The anchor topology places $\frac{N}{(N - 1)\sigma_p^2}$ of its $\frac{2}{\sigma_p^2}$ on the anchor mode, a fraction $\frac{N}{2(N - 1)} = 0.58$ on one of its $N - 1 = 6$ modes, while the pairwise topology places $\frac{1}{N - 1} = 0.17$ of its total on every mode. Second, a rescaling of $\sigma_p$ multiplies every eigenvalue by the same factor, so it can equalise the traces but cannot change the ratio of the largest eigenvalue to the smallest, which is $N$ for the anchor topology and $1$ for the pairwise topology. No choice of $\sigma_p$ makes one topology behave as the other, and the comparison is one of shape rather than of scale. Against the exact information of §3.5.3, whose mean form $\frac{1}{N}\mathbf{H}^\ast$ has the pairwise trace and spectrum, the anchor topology claims $\frac{2}{\sigma_p^2} - \frac{N - 1}{N\sigma_p^2} = \frac{1.14}{\sigma_p^2}$ more at $N = 7$, and $\frac{N}{(N - 1)\sigma_p^2} - \frac{1}{N\sigma_p^2} = \frac{1.02}{\sigma_p^2}$ of that excess, about $90$ percent, lies on the anchor mode.

In conclusion, the anchor topology is the stiffer spring on every observable mode and therefore holds the window's relative poses closer to the map's than the pairwise topology does, but its excess is $1.17$ on the $N - 2$ modes that move non anchor keyframes against one another and $8.17$ on the single mode that moves the anchor against the rest, so what it chiefly holds is the pose of the anchor relative to the other keyframes, with a confidence that §3.5.3 shows independent map errors do not justify. Note, 2026-11-20. The paragraphs from the total information to this conclusion replace one paragraph that stated the traces and the factor $2.33$ without deriving how the mean weighting bounds them or what the bound means for the comparison.

Step 12. Objective comparison. The table collects what Steps 1 to 11 and §3.5.1 establish, with no judgement on which topology the data favour. §3.5.3 asks which spectrum the mapper's errors justify, §3.5.4 follows the anchor mode outside the linear model, and §3.5.5 turns the rows of this table into predictions.

| Property | Anchor, `LocalMapRows` | Pairwise, `LocalMapRows2` |
| --- | --- | --- |
| Pairs | $N - 1$ | $\frac{N(N - 1)}{2}$ |
| Weight $w_p^2$ per pair | $\frac{1}{(N - 1)\sigma_p^2}$ | $\frac{1}{N^2\sigma_p^2}$ |
| Cost | $\frac{N}{N - 1}E_2 + \frac{N}{2\sigma_p^2(N - 1)}\vert\mathbf{d}_a - \bar{\mathbf{d}}\vert^2$ | $\frac{1}{2\sigma_p^2} \cdot \frac{1}{N}\sum_k \vert\mathbf{d}_k - \bar{\mathbf{d}}\vert^2$ |
| Common offset removed | the anchor's deviation $\mathbf{d}_a$ | the mean deviation $\bar{\mathbf{d}}$, optimal |
| Information matrix per axis | $\frac{1}{(N - 1)\sigma_p^2}\mathbf{L}_\text{star}$ | $\frac{1}{N\sigma_p^2}\mathbf{P}$ |
| Null space | $\boldsymbol{\iota}_N$, the gauge | $\boldsymbol{\iota}_N$, the gauge |
| Eigenvalues | $\frac{1}{(N - 1)\sigma_p^2}$ on $N - 2$ modes, $\frac{N}{(N - 1)\sigma_p^2}$ on the anchor mode | $\frac{1}{N\sigma_p^2}$ on all $N - 1$ modes |
| Stiffness relative to pairwise, non anchor modes | $\frac{N}{N - 1} = 1.17$ | $1$ |
| Stiffness relative to pairwise, anchor mode | $\frac{N^2}{N - 1} = 8.17$ | $1$ |
| Trace | $\frac{2}{\sigma_p^2}$ | $\frac{N - 1}{N\sigma_p^2}$ |
| Share of the trace on the anchor mode | $\frac{N}{2(N - 1)} = 0.58$ | $\frac{1}{N - 1} = 0.17$ |
| Largest over smallest nonzero eigenvalue | $N = 7$ | $1$ |
| Fraction of a disagreement imposed when $\lambda_o = \frac{1}{N\sigma_p^2}$ | $0.54$ on $N - 2$ modes, $0.89$ on the anchor mode | $0.5$ on all modes |
| Depends on keyframe order | yes, the anchor is the oldest shared keyframe | no |
| Minimiser when acting alone | all $\mathbf{d}_k$ equal | all $\mathbf{d}_k$ equal |
| Rows at $N = 7$ | 36 | 126 |

#### 3.5.3 Which weighting the measurements justify

Model the mapper's poses as the truth plus independent errors, $\hat{\mathbf{p}}_k = \mathbf{p}^\ast_k + \boldsymbol{\epsilon}_k$ with $\boldsymbol{\epsilon}_k \sim \mathcal{N}(\mathbf{0}, \sigma^2 \mathbf{I})$, since the mapper does not expose a marginal covariance. As in §3.5.2 the axes separate, so along one axis the stacked errors $\boldsymbol{\epsilon}$ of the $N$ keyframes have mean $\mathbf{0}$ and covariance $\sigma^2 \mathbf{I}$. The steps below derive the information the pairs actually carry under this model and compare each topology's weighting with it.

Step 1. What the pairs observe. The pair $(i, j)$ measures $\hat{p}_j - \hat{p}_i$, so by Step 2 of §3.5.2 the stacked measurements of a topology with incidence matrix $\mathbf{D}$ are

$$\mathbf{z} = \mathbf{D} \hat{\mathbf{p}} = \mathbf{D} \mathbf{p}^\ast + \mathbf{D} \boldsymbol{\epsilon}.$$

Their noise $\mathbf{D} \boldsymbol{\epsilon}$ has mean $\mathbf{0}$ and covariance

$$\text{E}\big[\mathbf{D} \boldsymbol{\epsilon} \boldsymbol{\epsilon}^\top \mathbf{D}^\top\big] = \mathbf{D}(\sigma^2 \mathbf{I})\mathbf{D}^\top = \sigma^2 \mathbf{D} \mathbf{D}^\top.$$

The entry of $\mathbf{D} \mathbf{D}^\top$ for the edges $e$ and $f$ is $\sum_m D_{em} D_{fm}$, which is $2$ on the diagonal, since every edge has two nonzero entries of magnitude $1$, and $\pm 1$ off it exactly when $e$ and $f$ share a keyframe. Two pairs that share a keyframe both contain its error, so their measurements are correlated, and only a dense weight can account for that.

Step 2. The exact information. The window's positions $\mathbf{p}$ enter the same rows as $\mathbf{D} \mathbf{p}$, so the negative log likelihood of §2.2.1 for these measurements is

$$E^\ast = \frac{1}{2}(\mathbf{D} \mathbf{p} - \mathbf{z})^\top (\sigma^2 \mathbf{D} \mathbf{D}^\top)^{+} (\mathbf{D} \mathbf{p} - \mathbf{z}),$$

with $^{+}$ the pseudo inverse, needed because $\mathbf{D} \mathbf{D}^\top$ is singular whenever the graph has a cycle. The complete graph has $\frac{N(N - 1)}{2}$ rows in $\mathbf{D}$ but rank $N - 1$, because its only null vector is $\boldsymbol{\iota}_N$ (§3.5.2 Step 5). The cost is quadratic in $\mathbf{p}$, and its Hessian is the exact information the pairs carry,

$$\mathbf{H}^\ast = \frac{1}{\sigma^2} \mathbf{D}^\top (\mathbf{D} \mathbf{D}^\top)^{+} \mathbf{D}.$$

Step 3. The exact information is the projector for every connected topology. Write $\mathbf{M} = \mathbf{D}^\top (\mathbf{D} \mathbf{D}^\top)^{+} \mathbf{D}$. It is symmetric, and the pseudo inverse satisfies $\mathbf{A}^{+} \mathbf{A} \mathbf{A}^{+} = \mathbf{A}^{+}$ [10], which applied to $\mathbf{A} = \mathbf{D} \mathbf{D}^\top$ gives

$$\mathbf{M}^2 = \mathbf{D}^\top (\mathbf{D} \mathbf{D}^\top)^{+} (\mathbf{D} \mathbf{D}^\top) (\mathbf{D} \mathbf{D}^\top)^{+} \mathbf{D} = \mathbf{D}^\top (\mathbf{D} \mathbf{D}^\top)^{+} \mathbf{D} = \mathbf{M}.$$

A symmetric $\mathbf{M}$ with $\mathbf{M}^2 = \mathbf{M}$ is an orthogonal projector. It sends $\boldsymbol{\iota}_N$ to $\mathbf{0}$, because $\mathbf{D} \boldsymbol{\iota}_N = \mathbf{0}$, and its rank is that of $\mathbf{D}$, which is $N - 1$ for a connected graph. It is therefore the projector onto the $N - 1$ directions orthogonal to $\boldsymbol{\iota}_N$, which by §3.5.2 Step 7b is $\mathbf{P}$. Hence for every connected topology

$$\mathbf{H}^\ast = \frac{1}{\sigma^2}\mathbf{P} = \frac{1}{\sigma^2}\Big(\mathbf{I} - \frac{\mathbf{K}_N}{N}\Big),$$

and with the dense weights $(\mathbf{D} \mathbf{D}^\top)^{+}$ the choice of pairs would not matter. This is ICE-BA's situation (§2.6.2).

Step 4. The exact weight of the star. With the anchor first and one row per edge $(a, k)$, which holds $-1$ at $a$ and $+1$ at $k$,

$$\mathbf{D}_\text{star} = \begin{bmatrix} -\boldsymbol{\iota}_{(N - 1)} & \mathbf{I} \end{bmatrix},$$

with $\mathbf{I}$ of size $N - 1$. Multiplying out,

$$\mathbf{D}_\text{star} \mathbf{D}_\text{star}^\top = \boldsymbol{\iota}_{N - 1} \boldsymbol{\iota}_{N - 1}^\top + \mathbf{I} = \mathbf{I} + \mathbf{K}_{(N - 1)},$$

which holds $2$ on the diagonal, since each pair contains two errors, and $1$ off it, since any two pairs share exactly the anchor's error. The star is a tree, so this matrix is invertible, and (I10) with $n = N - 1$ gives

$$(\mathbf{I} + \mathbf{K}_{N - 1})^{-1} = \mathbf{I} - \frac{\mathbf{K}_{N - 1}}{N}.$$

Its off diagonal entries $-\frac{1}{N}$ are the correction for the shared anchor error. As a check of Step 3, write $\mathbf{W} = \mathbf{I} - \frac{\mathbf{K}_{N - 1}}{N}$. Then $\mathbf{W} \boldsymbol{\iota}_{N - 1} = \boldsymbol{\iota}_{N - 1} - \frac{N - 1}{N}\boldsymbol{\iota}_{N - 1} = \frac{1}{N}\boldsymbol{\iota}_{N - 1}$, and multiplying the blocks,

$$\mathbf{D}_\text{star}^\top \mathbf{W} \mathbf{D}_\text{star} = \begin{bmatrix} \boldsymbol{\iota}_{N - 1}^\top \mathbf{W} \boldsymbol{\iota}_{N - 1} & -\boldsymbol{\iota}_{N - 1}^\top \mathbf{W} \\ -\mathbf{W} \boldsymbol{\iota}_{N - 1} & \mathbf{W} \end{bmatrix} = \begin{bmatrix} \frac{N - 1}{N} & -\frac{1}{N}\boldsymbol{\iota}_{N - 1}^\top \\ -\frac{1}{N}\boldsymbol{\iota}_{N - 1} & \mathbf{I} - \frac{\mathbf{K}_{N - 1}}{N} \end{bmatrix},$$

which holds $1 - \frac{1}{N}$ on the diagonal and $-\frac{1}{N}$ off it, and is therefore $\mathbf{P}$.

Step 5. The diagonal weight of `LocalMapRows`. `LocalMapRows` uses the diagonal weight $\mathbf{I}$ in place of $\mathbf{W}$, which gives $\mathbf{D}_\text{star}^\top \mathbf{D}_\text{star} = \mathbf{L}_\text{star}$ by §3.5.2 Step 3 rather than $\mathbf{P}$. By §3.5.2 Steps 6f and 7b both have the same eigenvectors, and $\mathbf{L}_\text{star}$ has the eigenvalue $1$ on the $N - 2$ non anchor modes, equal to that of $\mathbf{P}$, but $N$ on the anchor mode, where $\mathbf{P}$ has $1$. Dropping the off diagonal correction thus overstates the anchor mode by a factor $N$ and leaves the other modes exact. The ratio $\frac{N^2}{N - 1}$ of §3.5.2 Step 11 factors as $N \cdot \frac{N}{N - 1}$. The factor $N$ is this overstatement of the anchor mode relative to the exact shape $\mathbf{P}$, which the pairwise topology has, and the factor $\frac{N}{N - 1}$ is the difference of the two mean normalisations, which also scales the other modes.

Step 6. The complete graph. Its $\mathbf{D} \mathbf{D}^\top$ is singular by Step 2, but no inverse is needed. A uniform diagonal weight $c$ gives $c\,\mathbf{D}^\top \mathbf{D} = c\,\mathbf{L}_\text{complete} = cN\mathbf{P}$ by §3.5.2 Step 7, so $c = \frac{1}{N}$ reproduces the shape $\mathbf{P}$ of $\mathbf{H}^\ast$ exactly. `LocalMapRows2` with $w_p = \frac{1}{\sigma_p N}$ has, by §3.5.2 Step 8, $\mathbf{H}_2 = \frac{1}{N\sigma_p^2}\mathbf{P}$, which is $\frac{1}{N}\mathbf{H}^\ast$ with $\sigma = \sigma_p$, the exact information scaled to a mean.

Under independent map errors the pairwise topology is therefore the exact Gaussian information of the map with its gauge removed, and the anchor topology is a diagonal approximation that treats the anchor's map pose as $N$ times more certain than the others. The anchor topology is right only if the anchor's map pose is in fact the most accurate. Note, 2026-11-20. Steps 1 to 6 replace three steps that stated the covariance $\sigma^2 \mathbf{D} \mathbf{D}^\top$, the projector property and the inverse for the star without their intermediate steps, and $\mathbf{K}_N$ replaced $\mathbf{1} \mathbf{1}^\top$.

#### 3.5.4 Effects outside the linear model

1. Scale. The log scale step moves the window's positions along $s\,\mathbf{u}$, which in the model is the direction $\mathbf{v}_k = \hat{\mathbf{p}}_k$. Its information is $\mathbf{v}^\top \mathbf{H} \mathbf{v}$, namely $\frac{1}{(N - 1)\sigma_p^2}\sum_k |\hat{\mathbf{p}}_k - \hat{\mathbf{p}}_a|^2$ for the anchor and $\frac{1}{N\sigma_p^2}\sum_k |\hat{\mathbf{p}}_k - \bar{\hat{\mathbf{p}}}|^2$ for all pairs. To turn these two sums into closed forms, index the keyframes $k = 0, \dots, N - 1$ with the anchor at $k = 0$ and spaced evenly by $\Delta$ along the line, so $\hat{p}_k = k\Delta$ in the scale direction.

Step a. The anchor topology's information. Substituting $\hat{p}_a = \hat{p}_0 = 0$ gives $\hat{p}_k - \hat{p}_a = k\Delta$ for every $k$, so the anchor's sum runs over $k = 1, \dots, N - 1$, since $k = 0$ is the anchor itself and contributes $0$,

$$\sum_k |\hat{p}_k - \hat{p}_a|^2 = \Delta^2 \sum_{k=1}^{N-1} k^2 = \Delta^2 \cdot \frac{(N - 1)N(2N - 1)}{6},$$

using $\sum_{k=1}^{N-1} k^2 = \frac{(N - 1)N(2N - 1)}{6}$. Multiplying by the anchor weight $\frac{1}{(N - 1)\sigma_p^2}$ of §3.5.4 item 1's first sentence cancels one factor of $(N - 1)$,

$$\mathbf{v}^\top \mathbf{H}_1 \mathbf{v} = \frac{1}{(N - 1)\sigma_p^2} \cdot \Delta^2 \cdot \frac{(N - 1)N(2N - 1)}{6} = \frac{\Delta^2 N(2N - 1)}{6\sigma_p^2},$$

the anchor's scale information.

Step b. The pairwise topology's information. The mean position is $\bar{\hat{p}} = \frac{1}{N}\sum_{k=0}^{N-1} k\Delta = \Delta \cdot \frac{N - 1}{2}$, so $\hat{p}_k - \bar{\hat{p}} = \Delta\big(k - \frac{N - 1}{2}\big)$ and

$$\sum_k |\hat{p}_k - \bar{\hat{p}}|^2 = \Delta^2 \sum_k \Big(k - \frac{N - 1}{2}\Big)^2 = \Delta^2 \cdot \frac{N(N^2 - 1)}{12},$$

using $\sum_k \big(k - \frac{N - 1}{2}\big)^2 = \frac{N(N^2 - 1)}{12}$. Multiplying by the pairwise weight $\frac{1}{N\sigma_p^2}$ cancels the factor of $N$,

$$\mathbf{v}^\top \mathbf{H}_2 \mathbf{v} = \frac{1}{N\sigma_p^2} \cdot \Delta^2 \cdot \frac{N(N^2 - 1)}{12} = \frac{\Delta^2 (N^2 - 1)}{12\sigma_p^2},$$

the pairwise topology's scale information.

Step c. The ratio. Dividing Step a's result by Step b's,

$$\frac{\Delta^2 N(2N - 1) / (6\sigma_p^2)}{\Delta^2 (N^2 - 1) / (12\sigma_p^2)} = \frac{2N(2N - 1)}{N^2 - 1},$$

which is $3.8$ at $N = 7$. The anchor imports more scale, and imports it through the anchor mode that §3.5.3 shows it overweights.
2. A single wrong map pose. The anchor topology measures every deviation from $\mathbf{d}_a$, so an error $\boldsymbol{\epsilon}$ in the anchor's map pose shifts the target of every other keyframe by $\boldsymbol{\epsilon}$, and an error at any other keyframe affects only its own pair. The pairwise topology measures from $\bar{\mathbf{d}}$, so an error anywhere shifts the targets of the others by $\frac{\boldsymbol{\epsilon}}{N}$. Its influence is bounded and does not depend on which keyframe is wrong.
3. Marginalisation. When the anchor leaves the window, `LocalMapRows` re-anchors at the next keyframe and every one of its pairs changes, including the direction of its stiff mode. `LocalMapRows2` loses the $N - 1$ pairs of the departing keyframe and rescales the rest by $\frac{N^2}{(N - 1)^2}$, keeping the same centred form. The anchor topology should therefore show a step in the term's cost at every keyframe marginalisation.
4. Rotation lever arm. In the residual of §3.2 the translation of pair $(i, j)$ depends on $\mathbf{R}_j$ through $[\mathbf{R}_i^\top(\mathbf{p}_j - \mathbf{p}_i)]_\times$. In the anchor topology every keyframe's rotation is coupled to its own baseline from the anchor, the longest available, and in the pairwise topology to all its baselines, the longest of them shared with the extremes of the window.
5. Cost. The anchor topology has $6(N - 1)$ rows and the pairwise topology $3N(N - 1)$, 36 and 126 at $N = 7$, each over `aom.total_size` columns. Both are negligible beside the landmark QR of `optimize`.

#### 3.5.5 Expected outcome of the comparison

The model predicts the following, to be tested by §6.

1. Without the gravity and scale period, with a mapper whose keyframe errors are of similar size, `LocalMapRows2` reaches an equal or lower trajectory error and a smoother correction, because it uses the information the measurements carry and no more.
2. During the delayed period `LocalMapRows` corrects scale faster, by up to the ratio of §3.5.4 item 1, and its correction is only as good as the oldest shared keyframe's map pose. The mapper has refined its oldest keyframes over the most cycles, so this is the regime in which the anchor's privilege may be earned.
3. `LocalMapRows` shows steps in the local map cost and in the pose of the window at keyframe marginalisations (§3.5.4 item 3), which `LocalMapRows2` does not.
4. Under a single wrong map keyframe, `LocalMapRows` is the more robust when the wrong keyframe is not the anchor and the less robust when it is.

If the predictions hold, `LocalMapRows2` is the default and `LocalMapRows` is kept as the faster scale corrector. If the anchor topology wins throughout, the mapper's oldest keyframes are more accurate than the uniform model assumes, and per pair information from the mapper becomes the next step (§7).

### 3.6 Gauge

Under a global rigid motion $\mathbf{G}$ of the window, $\mathbf{T}_j^{-1} \mathbf{T}_i$ is unchanged by the computation of §2.6.3, so both rows of every pair are unchanged. Under the scale gauge $\mathbf{p} \to k\mathbf{p}$, $s \to s/k$ the product $s\,(\mathbf{p}_i - \mathbf{p}_j)$ is unchanged, and under the tilt gauge the residual is a rigid motion invariant. The residual therefore adds no information along any null vector of `ComputeGaugeProjector`, and the projector needs no change. The mapper's free position and yaw cannot leak into the window. This holds for both topologies, since it holds pair by pair.

### 3.7 Consistency with the prior

The residual is frame local, as the zero velocity rows were before the warm start. It enters the solve and is rebuilt at every `optimize`, so it moves states only through the step, and the first estimates and $\boldsymbol{\delta}$ stay consistent. It is not marginalised, because the mapper's relative poses are built from the estimator's own marginalisation packets (`RelPoseFactor`) and folding them back would count that information twice. The mean weighting bounds that double counting while the term is frame local. Folding into the prior later only needs the `leaving` path of `StationaryRows`, with the mapper's marginal information in place of the isotropic weights.

### 3.8 Start

The rows act once the first delayed job has been installed, or at the first marginalisation when the gravity and scale period is not configured, or after the fold. Before then the block is held and coarse jobs rescale the window, and the mapper's scale is the estimator's own unreliable scale.

## 4. Implementation plan

### Change 1. Configuration keys, `include/basalt/utils/vio_config.h`, `src/utils/vio_config.cpp`, `data/*.json`, `data/iccv21/basalt_batch_config.toml`

Add three keys beside `vio_zero_motion_rot_std_deg` (`vio_config.h:109`, `vio_config.cpp:91`, `:248`). The two standard deviations are 0 by default, meaning off, and `vio_local_map_pairwise` selects `LocalMapRows2` when true and `LocalMapRows` when false.

```cpp
    double vio_local_map_pos_std;      // vio_config.h
    double vio_local_map_rot_std_deg;
    bool vio_local_map_pairwise;

  vio_local_map_pos_std = 0.0;         // VioConfig::VioConfig
  vio_local_map_rot_std_deg = 0.0;
  vio_local_map_pairwise = true;

  ar(CEREAL_NVP(config.vio_local_map_pos_std));   // serialize
  ar(CEREAL_NVP(config.vio_local_map_rot_std_deg));
  ar(CEREAL_NVP(config.vio_local_map_pairwise));
```

Every file in `data/` except `data/sitl_config_vo.json` takes the code defaults, which leave the term off.

```json
        "config.vio_local_map_pos_std": 0.0,
        "config.vio_local_map_rot_std_deg": 0.0,
        "config.vio_local_map_pairwise": true,
```

`data/sitl_config_vo.json` takes the values derived below, beside `vio_zero_motion_rot_std_deg`.

```json
        "config.vio_local_map_pos_std": 0.15,
        "config.vio_local_map_rot_std_deg": 0.11,
        "config.vio_local_map_pairwise": true,
```

```toml
"config.vio_local_map_pos_std" = 0.0
"config.vio_local_map_rot_std_deg" = 0.0
"config.vio_local_map_pairwise" = true
```

Remove `vio_gs_dm_hold_pose_updates` from `vio_config.h:165`, `vio_config.cpp:115` and its serialisation at `:268`, and from every file in `data/`, since nothing moves a pose outside the solve any more. Add the three keys to every file in `data/` and the batch TOML, because a missing key is fatal (`context/vio_config_json.md`). The SITL values are set in `data/sitl_config_vo.json` by this change, as derived below. Note, 2026-11-21. This paragraph previously deferred the SITL values to a separate tuning commit. Note, 2026-11-18. This change previously added two keys, and gained `vio_local_map_pairwise` when the pairwise topology was added for comparison.

The SITL values. By §3.5.3 Step 6 the pairwise topology with $\sigma_p$ is the exact information of independent keyframe errors of standard deviation $\sigma_p$, divided by $N$ for the mean weighting of §3.4. $\sigma_p$ and $\sigma_R$ are therefore set to the standard deviation of the mapper's keyframe poses about the truth within one window, after the window's common rigid motion, the gauge of §3.6, is removed. The division by $N$ then remains what §3.7 makes it, the discount for the information the mapper shares with the estimator, and is not spent a second time on the choice of $\sigma_p$.

The mapper's poses are not logged, so the estimate uses the estimator's own poses and ground truth from `gt_eval.csv`. These bound the mapper's error from above, because the mapper refines the same keyframes from every observation the window had and from more keyframes. The keyframe timestamps are those of `map_publish.csv`, one per mapper cycle, that also appear in `gt_eval.csv`. Every run of $N = 7$ consecutive keyframes, the `vio_max_kfs` of the configuration, is one window, and windows whose true positions span less than 1 m on every axis are skipped, since a stationary window measures nothing about the relative poses. For each window the calculation has four steps.

Step 1. Centre the estimated and the true positions, $\mathbf{a}_k = \mathbf{p}_k - \bar{\mathbf{p}}$ and $\mathbf{b}_k = \mathbf{p}^\ast_k - \bar{\mathbf{p}}^\ast$, which removes the common translation.

Step 2. Remove the common rotation $\mathbf{R}_g$ that minimises $\sum_k |\mathbf{b}_k - \mathbf{R}_g \mathbf{a}_k|^2$. With the singular value decomposition $\sum_k \mathbf{b}_k \mathbf{a}_k^\top = \mathbf{U} \mathbf{S} \mathbf{V}^\top$ the minimiser is $\mathbf{R}_g = \mathbf{U} \, \text{diag}(1, 1, \text{det}(\mathbf{U} \mathbf{V}^\top)) \mathbf{V}^\top$, the rigid alignment of Umeyama [13] without its scale, since the scale is no gauge outside the gravity and scale period.

Step 3. The position standard deviation of the window is the root mean square of the remaining residuals per axis,

$$\sigma_p = \sqrt{\frac{1}{3N}\sum_k |\mathbf{b}_k - \mathbf{R}_g \mathbf{a}_k|^2}.$$

Step 4. The rotation errors are $\boldsymbol{\phi}_k = \text{Log}(\mathbf{R}^\ast_k \mathbf{R}_k^\top)$, and removing their mean $\bar{\boldsymbol{\phi}}$ as the common rotation gives, converted to degrees,

$$\sigma_R = \sqrt{\frac{1}{3N}\sum_k |\boldsymbol{\phi}_k - \bar{\boldsymbol{\phi}}|^2}.$$

The runs are every run under `/ws/logs/slam` whose `config.json` equals the present `data/sitl_config_vo.json` and which lasted more than 150 s without diverging. `2026-11-15_20-22-22` has the same configuration and was excluded, because its error reached 372 m. The calculation was run on 2026-11-21.

| Run | Duration | Windows | $\sigma_p$ in m, median, 75th and 90th percentile | $\sigma_R$ in degrees, median, 75th and 90th percentile |
| --- | --- | --- | --- | --- |
| `2026-11-15_20-42-02` | 178 s | 879 | 0.057, 0.119, 0.178 | 0.080, 0.114, 0.140 |
| `2026-11-16_21-37-55` | 178 s | 880 | 0.051, 0.109, 0.173 | 0.081, 0.123, 0.149 |
| `2026-11-16_22-38-18` | 343 s | 1945 | 0.094, 0.238, 0.338 | 0.054, 0.091, 0.129 |
| `2026-11-16_23-17-19` | 157 s | 767 | 0.082, 0.114, 0.190 | 0.072, 0.124, 0.167 |
| All four | | 4471 | 0.072, 0.147, 0.264 | 0.064, 0.111, 0.143 |

The values are the pooled 75th percentiles, $\sigma_p = 0.15$ m and $\sigma_R = 0.11$ degrees. The median would claim more than the window has in a quarter to a half of the windows, and the 90th percentile is dominated by the longest run, whose 90th percentile alone is 0.338 m. Since the estimator's poses bound the mapper's from above, the 75th percentile errs on the side of a term that claims too little. At $N = 7$ the pairwise topology then has the eigenvalue $\frac{1}{N\sigma_p^2} = \frac{1}{7 \cdot 0.15^2} = 6.3$ per m² on every mode (§3.5.2 Step 9), which is the information of independent keyframe errors of $\sigma_p\sqrt{N} = 0.40$ m, and $0.29$ degrees in rotation.

The disagreement between the mapper and the window that `vio_pose_update.csv` logs as `max_trans_err` is no substitute for this calculation. Its median per cycle is 0.35 to 0.50 m in the same runs, but it compares absolute poses, so it contains the drift of the common position and yaw between the mapper's world and the window's, which §3.6 shows the relative residual cannot see.

Backwards compatibility. The mapper no longer overwrites `frame_poses` in any configuration. With both standard deviations at 0 the estimator ignores the mapper, which changes every configuration that ran a mapper with small corrections, and leaves the `vio_gs_dm_hold_pose_updates` true behaviour of every configuration other than SITL unchanged. SITL, through `data/sitl_config_vo.json`, runs the pairwise term with the values above. `vio_local_map_pairwise` is read only when the term is on. Note, 2026-11-21. This paragraph previously stated that the SITL behaviour was unchanged, which held while every file kept both standard deviations at 0.

### Change 2. One relative pose residual for the mapper and the window, `include/basalt/utils/ba_utils.h` and `include/basalt/utils/nfr.h`

Add `relPoseErrorGS` after `computeRelPoseGS` (`ba_utils.h:82-133`), where the window's other relative pose linearisations live, and make it the only implementation of the relative pose residual. It is the residual and Jacobians of §3.2 and §3.3, templated on `Scalar` so that the `float` instantiation of the estimator can use it. `relPoseError` (`include/basalt/utils/nfr.h:43-73`) keeps its signature and becomes its `double` instantiation at `scale` = 1, so the mapper and the window run the same lines. The name follows Basalt's free functions, `computeRelPose`, `linearizePoint` and `relPoseError`, which start lower case. Note, 2026-11-18. This change previously added a new header `include/basalt/vi_estimator/local_map_residual.h` with a struct `LocalMapPairLin`, alias templates in a namespace `lmr` and a free function `LinearizeLocalMapPair` implementing the anchor frame residual. The header was dropped because residual and linearisation code belongs in `ba_utils.h`, the residual was replaced by Basalt's own relative pose residual (§3.2), and the function name broke the lower case rule for free functions. Note, 2026-11-19. This change previously left `relPoseError` untouched beside a second copy of its body in `relPoseErrorGS`, which kept two implementations of one residual. The copy was removed so that the code common to the mapper and the window lives in one place.

```cpp
#include <basalt/utils/sophus_utils.hpp>      // ba_utils.h, beside the existing include

// Decoupled residual of the relative pose T_w_j^-1 * T_w_i, with its
// translation multiplied by scale, against the measurement T_i_j, and its
// Jacobians on the left world increments of both poses and on the log scale.
template <class Scalar>
Sophus::Vector6<Scalar> relPoseErrorGS(
    const Sophus::SE3<Scalar>& T_i_j, const Sophus::SE3<Scalar>& T_w_i,
    const Sophus::SE3<Scalar>& T_w_j, Scalar scale,
    Sophus::Matrix6<Scalar>* d_res_d_T_w_i = nullptr,
    Sophus::Matrix6<Scalar>* d_res_d_T_w_j = nullptr,
    Sophus::Vector6<Scalar>* d_res_d_logscale = nullptr) {
  Sophus::SE3<Scalar> T_j_i = T_w_j.inverse() * T_w_i;
  T_j_i.translation() *= scale;
  Sophus::Vector6<Scalar> res = Sophus::se3_logd(T_i_j * T_j_i);

  if (d_res_d_T_w_i || d_res_d_T_w_j) {
    Sophus::Matrix6<Scalar> J;
    Sophus::rightJacobianInvSE3Decoupled(res, J);

    Sophus::Matrix3<Scalar> R = T_w_i.so3().inverse().matrix();

    Sophus::Matrix6<Scalar> Adj;
    Adj.setZero();
    Adj.template topLeftCorner<3, 3>() = scale * R;
    Adj.template bottomRightCorner<3, 3>() = R;

    if (d_res_d_T_w_i) {
      *d_res_d_T_w_i = J * Adj;
    }

    if (d_res_d_T_w_j) {
      Adj.template topRightCorner<3, 3>() =
          Sophus::SO3<Scalar>::hat(T_j_i.inverse().translation()) * R;
      *d_res_d_T_w_j = -J * Adj;
    }
  }

  if (d_res_d_logscale) {
    d_res_d_logscale->setZero();
    d_res_d_logscale->template head<3>() = T_i_j.so3() * T_j_i.translation();
  }

  return res;
}
```

`relPoseError` in `nfr.h` keeps its declaration and default arguments, and its body is replaced by the call. `nfr.h` includes `ba_utils.h`, whose own include chain is `landmark_database.h`, `imu_types.h` and `eigen_utils.hpp` and does not lead back to `nfr.h`.

```cpp
#include <basalt/utils/ba_utils.h>      // nfr.h, beside the existing includes

inline Sophus::Vector6d relPoseError(
    const Sophus::SE3d& T_i_j, const Sophus::SE3d& T_w_i,
    const Sophus::SE3d& T_w_j, Sophus::Matrix6d* d_res_d_T_w_i = nullptr,
    Sophus::Matrix6d* d_res_d_T_w_j = nullptr) {
  return relPoseErrorGS<double>(T_i_j, T_w_i, T_w_j, 1.0, d_res_d_T_w_i,
                                d_res_d_T_w_j);
}
```

Line by line against §3.3. `J` is $\text{blkdiag}(\mathbf{E}, \mathbf{J}_r^{-1}(\mathbf{r}_R))$. `Adj` is the second factor of §2.7.4 Step 4 with its translation row multiplied by $s$, $s\,\mathbf{R}_i^\top$ on the diagonal and $s\,[\mathbf{R}_i^\top(\mathbf{p}_j - \mathbf{p}_i)]_\times \mathbf{R}_i^\top$ off it. The off diagonal block takes its scale from the scaled `T_j_i`, whose translation is $s\,\mathbf{R}_j^\top(\mathbf{p}_i - \mathbf{p}_j)$, so that its inverse has the translation

$$-\mathbf{R}_i^\top \mathbf{R}_j \cdot s\,\mathbf{R}_j^\top(\mathbf{p}_i - \mathbf{p}_j) = s\,\mathbf{R}_i^\top(\mathbf{p}_j - \mathbf{p}_i),$$

the required factor, by the same expression `relPoseError` uses. `T_j_i.translation()` is $s\,\mathbf{R}_j^\top \mathbf{u}$ after the scaling, so `d_res_d_logscale` is $\hat{\mathbf{R}}_{ij}\,s\,\mathbf{R}_j^\top \mathbf{u} = \mathbf{r}_t - \hat{\mathbf{t}}_{ij}$.

At `scale` = 1 the two multiplications by `scale` are multiplications by exactly one, which IEEE 754 arithmetic leaves exact, and every other operation is that of the former body of `relPoseError` in the same order. The wrapper therefore reproduces the former results bit for bit. This was checked on 2026-11-19 in the header only harness of `/ws/skills/cpp-syntax-check-tooling.md` against a verbatim copy of the former body, with exact equality of the residual and both Jacobians over 20000 random pose triples, compiled at `-O0` and at `-O2 -march=x86-64-v3`, the host's `CXX_MARCH`. The `float` instantiation compiled in the same harness.

Backwards compatibility. `relPoseError` keeps its signature and its results bit for bit, so `NfrMapper::extractNonlinearFactors` (`src/vi_estimator/nfr_mapper.cpp:228`), `NfrMapper::optimize` (`:462`), `MapperLinearizeAbsReduce` (`include/basalt/vi_estimator/nfr_mapper.h:144`), `src/mapper_sim.cpp:454` and `RelPoseTest` behave as before. `ba_utils.h` gains one template and one include of a header its consumers, `sc_ba_base.cpp`, `ba_base.cpp`, the three linearisation translation units, `landmark_block_abs_dynamic.hpp` and `test_vio.cpp`, already reach through `imu_types.h`, so an uninstantiated template changes none of them. The units that include `nfr.h`, namely `nfr_mapper.h` and through it `local_mapper.h`, `controller.h` and `nfr_mapper.cpp`, and `src/mapper.cpp`, `src/mapper_sim.cpp` and `test/src/test_nfr.cpp`, now also read `ba_utils.h` and `landmark_database.h`, which `nfr_mapper.h` already reaches through `sc_ba_base.h`.

### Change 3. The constant map and the row builders, `include/basalt/vi_estimator/sqrt_keypoint_vio.h`

`QueuePoseUpdates` and its mutex stay as the cross thread hand off. Replace `kRelinThresholdTrans` and `kRelinThresholdRot` (`:401-404`) with the estimator thread's map and gate, and declare the row builders beside `StationaryRows` (`:161-166`).

```cpp
    // Relative pose rows of every keyframe the mapper and the window share
    // against the oldest of them.
    void LocalMapRows(const AbsOrderMap& aom, MatX& J, VecX& r,
                      Scalar* maxTrans = nullptr,
                      Scalar* maxRot = nullptr) const;
    // Relative pose rows of every pair of keyframes the mapper and the window
    // share.
    void LocalMapRows2(const AbsOrderMap& aom, MatX& J, VecX& r,
                       Scalar* maxTrans = nullptr,
                       Scalar* maxRot = nullptr) const;
    // Timestamps of the keyframes the mapper and the window share, oldest
    // first, and their mapper poses carried to the latest epoch. Empty while
    // the term is off or the gate is closed.
    void GetLocalMapPoses(const AbsOrderMap& aom, std::vector<int64_t>& vIds,
                          Eigen::aligned_vector<Sophus::SE3d>& vPoses) const;
    // Whitened rows of relative pose factors measured in the metric world,
    // with the scale of the gravity and scale block when it is active.
    void RelPoseFactorRows(const AbsOrderMap& aom,
                           const Eigen::aligned_vector<RelPoseFactor>& vFactors,
                           MatX& J, VecX& r, Scalar* maxTrans,
                           Scalar* maxRot) const;

    // Latest mapper pose per keyframe timestamp and the epoch it is expressed
    // in, owned by the estimator thread.
    std::map<int64_t, std::pair<Sophus::SE3d, int64_t>> mvpLocalMapPoses;
    bool mpLocalMapOpen = false;
```

`GetLocalMapPoses` returns the identifiers and the carried mapper poses of the keyframes that the local map and the window share, which its name states. Note, 2026-11-21. This method was previously named `LocalMapShared`.

`QueuePoseUpdates` keeps accepting every keyframe of the local map without a filter on the window. A filter there is not needed, for three reasons.

1. It cannot be exact. `QueuePoseUpdates` runs on the mapper thread, and `frame_poses`, `frame_states` and `kf_ids` belong to the estimator thread, so a filter there would read them without a lock or add one to every `optimize`. A keyframe that is in the window when the mapper queues it may also be marginalised before the estimator drains it, so the estimator would still have to filter at its own time.
2. The estimator already filters at its own time, in two places. The drain of Change 4 erases every entry older than the oldest pose or state of the window. `GetLocalMapPoses` then keeps only the entries present in `aom.abs_order_map`, which holds exactly the poses and states of the window being solved. This second filter matters, because `marginalize` may remove a keyframe from the middle of the window, through the feature ratio test or the score taken from DSO (`src/vi_estimator/sqrt_keypoint_vio.cpp:1199-1272`), so an entry newer than the oldest keyframe need not be in the window.
3. Nothing grows without bound. `mpPosesToUpdate` and `mpPosesToUpdateEpoch` are keyed by timestamp and drained at every frame, so they hold at most one cycle's keyframes, the mapper's `frame_poses` of at most `local_mapper_max_local_map_size` entries (`src/vi_estimator/local_mapper.cpp:322-323`). `mvpLocalMapPoses` keeps in addition only keyframes no older than the window's oldest, and each is erased once the window's oldest passes it.

Backwards compatibility. `QueuePoseUpdates` keeps its signature, so `src/controller.cpp:220-223` and `LocalMapper` are unchanged. `RelPoseFactor` is visible through `imu_types.h`, which the header already includes through `ba_base.h`.

### Change 4. Draining instead of overwriting, `SqrtKeypointVioEstimator::measure`, `src/vi_estimator/sqrt_keypoint_vio.cpp:637-714`

Replace the whole block, including the hold of `vio_gs_dm_hold_pose_updates`, by a drain into `mvpLocalMapPoses` and a prune of entries older than the window.

```cpp
    Timer tPoseUpdate;
    int poseUpdatePending = 0;
    {
        std::lock_guard<std::mutex> lock(mpPosesToUpdateMutex);
        poseUpdatePending = int(mpPosesToUpdate.size());
        for (const auto& [t_ns, pose] : mpPosesToUpdate) {
            const auto it = mpPosesToUpdateEpoch.find(t_ns);
            mvpLocalMapPoses[t_ns] = {
                pose.getPose(),
                it != mpPosesToUpdateEpoch.end() ? it->second : 0};
        }
        mpPosesToUpdate.clear();
        mpPosesToUpdateEpoch.clear();
    }
    const int64_t oldest = !frame_poses.empty()
                               ? frame_poses.begin()->first
                               : (frame_states.empty()
                                      ? 0
                                      : frame_states.begin()->first);
    mvpLocalMapPoses.erase(mvpLocalMapPoses.begin(),
                           mvpLocalMapPoses.lower_bound(oldest));
    const double poseUpdateSeconds = tPoseUpdate.elapsed();
```

`AddVioPoseUpdate` moves to the end of `optimize` (Change 6), where the residual is known.

Backwards compatibility. `frame_poses` is no longer written here, which is the intended change of Change 1.

### Change 5. The rows, `SqrtKeypointVioEstimator::LocalMapRows`, `LocalMapRows2`, `GetLocalMapPoses` and `RelPoseFactorRows`, `src/vi_estimator/sqrt_keypoint_vio.cpp`

Placed after `StationaryRows`, with the same output shape, $J$ of `aom.total_size` columns. The two topologies differ only in the factors they build. Both prune to the window through `GetLocalMapPoses`, the filter on the window explained in Change 3, both store their pairs as `RelPoseFactor`, and both linearise through `RelPoseFactorRows`, which plays the role of `MapperLinearizeAbsReduce`'s factor loop (`include/basalt/vi_estimator/nfr_mapper.h:131-162`) in the square root form the window uses. Note, 2026-11-18. This change previously held a single `LocalMapRows` that wrote the anchor rows from `LinearizeLocalMapPair` directly, without `RelPoseFactor`. It was split so that the two topologies of §3.4 share every line except their pair loop.

The translation unit includes `basalt/utils/ba_utils.h` beside `basalt/utils/assert.h` for `relPoseErrorGS`, since none of its other includes reaches that header. The isotropic information and the factor constructor are free functions in an anonymous namespace inside `namespace basalt` of the translation unit, lower case by the naming rule. Note, 2026-11-21. This change previously assumed that `ba_utils.h` was reachable from `sqrt_keypoint_vio.cpp`, which the first build disproved with `relPoseErrorGS was not declared in this scope`.

```cpp
namespace {
// Isotropic information of one local map pair with both variances multiplied
// by norm.
Sophus::Matrix6d localMapInformation(const VioConfig& config, double norm) {
    const double rotStd = config.vio_local_map_rot_std_deg * M_PI / 180.0;
    Sophus::Matrix6d info = Sophus::Matrix6d::Zero();
    info.diagonal().head<3>().setConstant(
        1.0 / (config.vio_local_map_pos_std * config.vio_local_map_pos_std *
               norm));
    info.diagonal().tail<3>().setConstant(1.0 / (rotStd * rotStd * norm));
    return info;
}

RelPoseFactor makeLocalMapFactor(int64_t i, int64_t j, const Sophus::SE3d& Ti,
                                 const Sophus::SE3d& Tj,
                                 const Sophus::Matrix6d& info) {
    RelPoseFactor rpf;
    rpf.t_i_ns = i;
    rpf.t_j_ns = j;
    rpf.T_i_j = Ti.inverse() * Tj;
    rpf.cov_inv = info;
    return rpf;
}
}  // namespace
```

The shared keyframes. The iteration over `mvpLocalMapPoses`, a `std::map`, yields them oldest first, which fixes the anchor of `LocalMapRows` as `vIds.front()`.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::GetLocalMapPoses(
    const AbsOrderMap& aom, std::vector<int64_t>& vIds,
    Eigen::aligned_vector<Sophus::SE3d>& vPoses) const {
    vIds.clear();
    vPoses.clear();
    if (!mpLocalMapOpen || config.vio_local_map_pos_std <= 0 ||
        config.vio_local_map_rot_std_deg <= 0)
        return;

    const int64_t latest = this->mGsCorrectionLog
                               ? this->mGsCorrectionLog->LatestEpoch()
                               : 0;
    for (const auto& [id, entry] : mvpLocalMapPoses) {
        if (aom.abs_order_map.count(id) == 0) continue;
        const auto& [T, epoch] = entry;
        vIds.push_back(id);
        vPoses.push_back(this->mGsCorrectionLog
                             ? this->mGsCorrectionLog->Between(epoch, latest)
                                   .Apply(T)
                             : T);
    }
}
```

The anchor topology of §3.4 item 1, $N - 1$ pairs from the oldest shared keyframe, each with variances multiplied by $N - 1$.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::LocalMapRows(const AbsOrderMap& aom,
                                                     MatX& J, VecX& r,
                                                     Scalar* maxTrans,
                                                     Scalar* maxRot) const {
    std::vector<int64_t> vIds;
    Eigen::aligned_vector<Sophus::SE3d> vPoses;
    GetLocalMapPoses(aom, vIds, vPoses);

    Eigen::aligned_vector<RelPoseFactor> vFactors;
    const size_t n = vIds.size();
    if (n >= 2) {
        const Sophus::Matrix6d info = localMapInformation(config, double(n - 1));
        for (size_t k = 1; k < n; ++k) {
            vFactors.push_back(makeLocalMapFactor(vIds[0], vIds[k], vPoses[0],
                                                  vPoses[k], info));
        }
    }
    RelPoseFactorRows(aom, vFactors, J, r, maxTrans, maxRot);
}
```

The pairwise topology of §3.4 item 2, all $\frac{N(N - 1)}{2}$ pairs, each with variances multiplied by $N^2$.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::LocalMapRows2(const AbsOrderMap& aom,
                                                      MatX& J, VecX& r,
                                                      Scalar* maxTrans,
                                                      Scalar* maxRot) const {
    std::vector<int64_t> vIds;
    Eigen::aligned_vector<Sophus::SE3d> vPoses;
    GetLocalMapPoses(aom, vIds, vPoses);

    Eigen::aligned_vector<RelPoseFactor> vFactors;
    const size_t n = vIds.size();
    if (n >= 2) {
        const Sophus::Matrix6d info =
            localMapInformation(config, double(n) * double(n));
        vFactors.reserve(n * (n - 1) / 2);
        for (size_t i = 0; i + 1 < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                vFactors.push_back(makeLocalMapFactor(vIds[i], vIds[j],
                                                      vPoses[i], vPoses[j],
                                                      info));
            }
        }
    }
    RelPoseFactorRows(aom, vFactors, J, r, maxTrans, maxRot);
}
```

The linearisation. Jacobians at the first estimates and the residual at the current values, as `linearizeHelperStatic` does with `computeRelPose` (`src/vi_estimator/sc_ba_base.cpp:278-290`). Each factor's rows are whitened by the upper Cholesky factor $\mathbf{U}$ of `cov_inv`, $\mathbf{U}^\top \mathbf{U} = \boldsymbol{\Lambda}$, so that the injection's $\mathbf{J}^\top \mathbf{J}$ and $\mathbf{J}^\top \mathbf{r}$ equal the mapper's $\mathbf{J}^\top \boldsymbol{\Lambda} \mathbf{J}$ and $\mathbf{J}^\top \boldsymbol{\Lambda} \mathbf{r}$. This keeps a full `cov_inv` usable unchanged should the mapper later supply one (§7).

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::RelPoseFactorRows(
    const AbsOrderMap& aom,
    const Eigen::aligned_vector<RelPoseFactor>& vFactors, MatX& J, VecX& r,
    Scalar* maxTrans, Scalar* maxRot) const {
    J.setZero(POSE_SIZE * vFactors.size(), aom.total_size);
    r.setZero(POSE_SIZE * vFactors.size());
    if (vFactors.empty()) return;

    const bool gs = mpGsActive && this->mGsState.has_value();
    const Scalar s = gs ? this->mGsState->GetScale() : Scalar(1);
    const Scalar sLin = gs ? this->mGsState->GetScaleLin() : Scalar(1);
    const int cs =
        gs ? aom.abs_order_map.at(kGravityScaleStateId).first +
                 GRAVITY_SCALE_SIZE - 1
           : -1;

    for (size_t f = 0; f < vFactors.size(); ++f) {
        const RelPoseFactor& rpf = vFactors[f];
        const PoseStateWithLin<Scalar> pi = getPoseStateWithLin(rpf.t_i_ns);
        const PoseStateWithLin<Scalar> pj = getPoseStateWithLin(rpf.t_j_ns);
        const SE3 Tij = rpf.T_i_j.template cast<Scalar>();

        Sophus::Matrix6<Scalar> Ji, Jj;
        Sophus::Vector6<Scalar> Js;
        relPoseErrorGS<Scalar>(Tij, pi.getPoseLin(), pj.getPoseLin(), sLin,
                               &Ji, &Jj, &Js);
        const Sophus::Vector6<Scalar> res =
            relPoseErrorGS<Scalar>(Tij, pi.getPose(), pj.getPose(), s);

        const Sophus::Matrix6d U = rpf.cov_inv.llt().matrixU();
        const Sophus::Matrix6<Scalar> W = U.template cast<Scalar>();
        const int row = int(POSE_SIZE * f);
        J.template block<POSE_SIZE, POSE_SIZE>(
            row, aom.abs_order_map.at(rpf.t_i_ns).first) = W * Ji;
        J.template block<POSE_SIZE, POSE_SIZE>(
            row, aom.abs_order_map.at(rpf.t_j_ns).first) = W * Jj;
        if (gs) J.template block<POSE_SIZE, 1>(row, cs) = W * Js;
        r.template segment<POSE_SIZE>(row) = W * res;

        if (maxTrans)
            *maxTrans = std::max(*maxTrans, res.template head<3>().norm());
        if (maxRot) *maxRot = std::max(*maxRot, res.template tail<3>().norm());
    }
}
```

Design rationale.

1. Pose columns are the first six of both `frame_poses` and `frame_states` blocks, and `getPoseStateWithLin` serves both (`include/basalt/vi_estimator/ba_base.h:150-161`), so one path serves both. The log scale column is `first + GRAVITY_SCALE_SIZE - 1`, the convention of `LandmarkBlockAbsDynamic` (`include/basalt/linearization/landmark_block_abs_dynamic.hpp:58-61`).
2. The factors are rebuilt at every call from `mvpLocalMapPoses` rather than stored as a member list of `RelPoseFactor`. Their endpoints depend on the window and their measurements on the epoch, so a stored list would need a second prune in `marginalize`, the counterpart of `PruneFactorsWithUnknownKeyframes`, which this avoids. The estimator thread stores the mapper's poses, which is the state the factors are a function of, and at most 21 factors are built per call at $N = 7$.
3. The rows are written whitened, as `StationaryRows` writes its rows, rather than accumulated into $\mathbf{H}$ and $\mathbf{b}$ blocks as the mapper does, so that the four injection sites of Change 6 stay as they are and the term enters `error_total`, `l_diff` and `after_error_total` from the same rows.
4. Isotropic weights are used because the mapper does not expose a marginal covariance, and computing one over its dense window would cost more than the mapper cycle.

Backwards compatibility. New methods, returning zero rows unless both standard deviations are positive and the gate is open.

### Change 6. Injection into `SqrtKeypointVioEstimator::optimize`

Follow the stationary rows at their four sites, `:1955-1960`, `:2038-2041`, `:2122-2127` and `:2205-2214`, before the gauge projector. The two sites that build rows select the topology with `vio_local_map_pairwise`, and the other two consume the rows unchanged. Note, 2026-11-18. This change previously called `LocalMapRows` alone at both sites.

```cpp
            MatX localMapJ;                                     // after :1960
            VecX localMapR;
            Scalar localMapMaxTrans = 0, localMapMaxRot = 0;
            if (config.vio_local_map_pairwise)
                LocalMapRows2(aom, localMapJ, localMapR, &localMapMaxTrans,
                              &localMapMaxRot);
            else
                LocalMapRows(aom, localMapJ, localMapR, &localMapMaxTrans,
                             &localMapMaxRot);
            const bool applyLocalMap = localMapJ.rows() > 0;
            if (applyLocalMap)
                error_total += Scalar(0.5) * localMapR.squaredNorm();

                    if (applyLocalMap) {                        // after :2041
                        H += localMapJ.transpose() * localMapJ;
                        b += localMapJ.transpose() * localMapR;
                    }

                    if (applyLocalMap) {                        // after :2127
                        const VecX dr = localMapJ * inc;
                        l_diff -= dr.dot(localMapR + Scalar(0.5) * dr);
                    }

                if (applyLocalMap) {                            // after :2214
                    MatX localMapJNew;
                    VecX localMapRNew;
                    if (config.vio_local_map_pairwise)
                        LocalMapRows2(aom, localMapJNew, localMapRNew);
                    else
                        LocalMapRows(aom, localMapJNew, localMapRNew);
                    after_error_total +=
                        Scalar(0.5) * localMapRNew.squaredNorm();
                }
```

The first linearisation of each `optimize` records its pair count and largest unweighted residuals in three members, which `measure` resets at the drain and logs right after `optimize_and_marg`, so that `vio_pose_update.csv` keeps one row per frame, including the frames before `optimize` starts, and each row describes its own frame.

```cpp
            if (it == 0) {                                      // in optimize
                mpLocalMapPairs = int(localMapJ.rows() / POSE_SIZE);
                mpLocalMapMaxTrans = localMapMaxTrans;
                mpLocalMapMaxRot = localMapMaxRot;
            }

    // Pair count and largest unweighted residuals of the last optimize.
    int mpLocalMapPairs = 0;                                    // header
    Scalar mpLocalMapMaxTrans = 0, mpLocalMapMaxRot = 0;

    mpLocalMapPairs = 0;                                        // measure, drain
    mpLocalMapMaxTrans = mpLocalMapMaxRot = 0;

    optimize_and_marg(num_points_connected, lost_landmaks);    // measure
    const double optMargSeconds = tOptMarg.elapsed();
    mpLogger->AddVioPoseUpdate(
        opt_flow_meas->t_ns, poseUpdatePending, mpLocalMapPairs, 0,
        double(mpLocalMapMaxTrans), double(mpLocalMapMaxRot),
        config.vio_local_map_pos_std,
        config.vio_local_map_rot_std_deg * M_PI / 180.0);
```

`scripts/analysis/analysis.py` keeps reading the same columns, and the titles of its `pose_update_acceptance` and `pose_update_gate` plots now name the local map pairs and the standard deviations. Note, 2026-11-21. This passage previously left open whether `poseUpdatePending` would become a member or the log call stay in `measure` one frame late. Keeping the call in `measure` after `optimize_and_marg` avoids both.

`LocalMapGaps` and the record `vio_local_map.csv` were added on 2026-11-21 to diagnose the scale behaviour of §6. Over the consecutive shared keyframes they log the scale gap $\ln \frac{\sum_k |\Delta\hat{\mathbf{p}}_k|}{s \sum_k |\Delta\mathbf{p}_k|}$, the horizontal and vertical parts of the relative rotation discrepancy rotated into the world, the speeds both imply and their largest difference, and, at the first iteration of `optimize`, the gradient and curvature on the log scale column of the rest of the system and of the local map rows and the solved scale step.

Backwards compatibility. The three cost sites stay consistent (`context/vio_opt.md` §9.3). With zero rows every branch is skipped.

### Change 7. The gate, `SqrtKeypointVioEstimator::UpdateDelayedMarg` and `marginalize`

```cpp
        } else {                                        // install, :2873
            catchup = InstallDelayedJob(*job);
            maxPosInc = job->mMaxPosInc;
            installed = 1;
            mpGsSwapEvent = 6;
            mpLocalMapOpen = true;
        }

    if (!mpGsActive) mpLocalMapOpen = true;             // end of marginalize
```

The second line covers runs without the period and the frames after the fold. The period always runs with delayed marginalisation, since `initialize` creates `mpDm` whenever it activates the block (`src/vi_estimator/sqrt_keypoint_vio.cpp:165-172`) and only `FoldGravityScale` stops it, so every period either installs a fine job or ends in the fold, and the gate opens in both cases. A period whose coarse phase lasts until the fold receives no residuals at all.

Backwards compatibility. Read only by `LocalMapRows`.

### Change 8. Tests and documentation

Add `RelPoseGSTest` to `test/src/test_nfr.cpp` after `RelPoseTest` (`:24-61`). It checks the three Jacobians of `relPoseErrorGS` against `test_jacobian` at a scale other than 1. The scale one case needs no test of its own, because `RelPoseTest` already checks `relPoseError`, which is now that case. `ba_utils.h` reaches the test through `nfr.h`. `test_nfr` already links `basalt` (`test/CMakeLists.txt:14-15`). Note, 2026-11-18. This change previously added `LocalMapPairJacobianTest` to `test/src/test_vio.cpp` for the dropped `LinearizeLocalMapPair`. Note, 2026-11-19. The test previously also compared `relPoseErrorGS` at scale 1 with `relPoseError`, which became an identity once `relPoseError` called `relPoseErrorGS` (Change 2).

```cpp
TEST(PreIntegrationTestSuite, RelPoseGSTest) {
  Sophus::SE3d T_w_i = Sophus::se3_expd(Sophus::Vector6d::Random());
  Sophus::SE3d T_w_j = Sophus::se3_expd(Sophus::Vector6d::Random());
  Sophus::SE3d T_i_j = Sophus::se3_expd(Sophus::Vector6d::Random() / 100) *
                       T_w_i.inverse() * T_w_j;
  const double scale = 1.3;

  Sophus::Matrix6d d_res_d_T_w_i, d_res_d_T_w_j;
  Sophus::Vector6d d_res_d_logscale;
  basalt::relPoseErrorGS(T_i_j, T_w_i, T_w_j, scale, &d_res_d_T_w_i,
                         &d_res_d_T_w_j, &d_res_d_logscale);

  {
    Sophus::Vector6d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_T_w_i", d_res_d_T_w_i,
        [&](const Sophus::Vector6d& x) {
          auto T_w_i_new = T_w_i;
          basalt::PoseState<double>::incPose(x, T_w_i_new);
          return basalt::relPoseErrorGS(T_i_j, T_w_i_new, T_w_j, scale);
        },
        x0);
  }

  {
    Sophus::Vector6d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_T_w_j", d_res_d_T_w_j,
        [&](const Sophus::Vector6d& x) {
          auto T_w_j_new = T_w_j;
          basalt::PoseState<double>::incPose(x, T_w_j_new);
          return basalt::relPoseErrorGS(T_i_j, T_w_i, T_w_j_new, scale);
        },
        x0);
  }

  {
    Eigen::Matrix<double, 1, 1> x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_logscale", d_res_d_logscale,
        [&](const Eigen::Matrix<double, 1, 1>& x) {
          return basalt::relPoseErrorGS(T_i_j, T_w_i, T_w_j,
                                        scale * std::exp(x[0]));
        },
        x0);
  }
}
```

`RelPoseGSTest` instantiates only header templates, so it also runs in the container's header only harness (`/ws/skills/cpp-syntax-check-tooling.md`, additions of 2026-11-14) when the full build is unavailable, compiled with `test_utils.h` from `thirdparty/basalt-headers/test/include`.

Add §3.2.9 to `doc/VIO.md` with the residual of §3.2 and §3.3 and both topologies of §3.4, correct `doc/VIO.md` §3.3.6 and `context/vio_localmapper_correction_loop.md` where they describe the overwrite and the threshold, with a dated note.

## 5. Blast radius

1. `QueuePoseUpdates` callers, `src/controller.cpp:220-223` only, signature unchanged.
2. `kRelinThresholdTrans` and `kRelinThresholdRot`, read only in `measure`, removed.
3. `vio_gs_dm_hold_pose_updates`, read only at `:651`, removed with its keys. SITL set it true, and the new gate keeps updates out of the coarse phase as the hold did.
4. `vio_pose_update.csv` keeps its columns with new meanings, which the `pose_update_acceptance` plot of `scripts/analysis/analysis.py:1014` reads as acceptance counts and must relabel.
5. `ComputeGaugeProjector`, unchanged by §3.6.
6. `marginalize` and the delayed store, unchanged, since the rows are not marginalised.
7. `include/basalt/utils/ba_utils.h`, included by `src/vi_estimator/sc_ba_base.cpp`, `src/vi_estimator/ba_base.cpp`, `src/linearization/linearization_rel_sc.cpp`, `src/linearization/linearization_abs_qr.cpp`, `src/linearization/linearization_abs_sc.cpp`, `include/basalt/linearization/landmark_block_abs_dynamic.hpp` and `test/src/test_vio.cpp`. It gains one template and one include of a header these units already reach, so none of them changes.
8. `relPoseError`, read by `NfrMapper`, `MapperLinearizeAbsReduce`, `src/mapper_sim.cpp` and `test/src/test_nfr.cpp`, keeps its signature and its results bit for bit while its body moves to `relPoseErrorGS` (Change 2). `nfr.h` and its includers now also read `ba_utils.h`. `RelPoseFactor` is not modified, and the estimator only constructs new values of it. Note, 2026-11-19. This item previously stated that `relPoseError` is not modified.
9. `data/sitl_config_vo.json`, the only configuration that turns the term on, with $\sigma_p = 0.15$ m and $\sigma_R = 0.11$ degrees (Change 1). Every other file in `data/` and the batch TOML keep it off.

## 6. Validation

1. GCC 12.2 syntax check of `src/vi_estimator/sqrt_keypoint_vio.cpp`, `src/utils/vio_config.cpp` and `src/utils/logger.cpp` with both instantiations, of `src/vi_estimator/sc_ba_base.cpp` for the edited `ba_utils.h`, and of `src/vi_estimator/nfr_mapper.cpp` and `src/vi_estimator/local_mapper.cpp` for the edited `nfr.h`. Run on 2026-11-21 as a full build of the `slam` package with `/ws/scripts/build_ros_packages.sh`, `-Werror`, both instantiations, which passed after the include of Change 5 was added. `tello_gazebo` fails in the same script because the container has no Gazebo, independently of this change.
2. `RelPoseGSTest` and the existing `RelPoseTest`, in `test_nfr` or in the header only harness. Run on 2026-11-18 in the header only harness with the function and the test extracted verbatim from Changes 2 and 8 by script, which passed, and a `float` instantiation of `relPoseErrorGS` compiled without warnings. On 2026-11-21 the committed `test/src/test_nfr.cpp` with `RelPoseGSTest` passed all five tests in the header only harness against the implemented headers, since the ROS build sets `BASALT_BUILD_TESTS` off. On 2026-11-19 the revised `relPoseErrorGS` passed the same Jacobian checks, and the wrapper form of `relPoseError` matched a verbatim copy of its former body exactly over 20000 random pose triples at `-O0` and `-O2 -march=x86-64-v3` (Change 2). The decomposition of §3.5.1 Step 5 and the single keyframe stiffness ratios of §3.5.2 were checked numerically at $N = 7$ the same day. The spectra, the ratio $\frac{N^2}{N - 1} = 8.17$, the traces, the projector identity of §3.5.3 and the scale ratio $3.79$ of §3.5.4 were checked numerically at $N = 7$ the same day.
3. A replay with both standard deviations at 0, which must match the previous binary except for the removed overwrite. Run on 2026-11-21 on `/ws/data/telemetry_40`, 170 s of wall time covering the first 51.8 s of the flight. Two replays were identical to each other, with an absolute trajectory error of 1.475 m after `se3` alignment. That window lies wholly inside the gravity and scale period, where the former `vio_gs_dm_hold_pose_updates` dropped every update, so the former binary also ran without corrections there. The comparison against a run of the former binary itself remains outstanding.
4. The comparison of §3.5.5. Replays of the same bags in four configurations, the term off, `LocalMapRows`, `LocalMapRows2`, and `LocalMapRows2` with $\sigma_p$ halved so that its anchor free stiffness approaches the anchor topology's scale information. The bags are `/ws/data/telemetry_40` and a square take off, which cover a run without the gravity and scale period and one with it. For each run, record the absolute trajectory error after `posyaw` and `sim3` alignment and the scale ratio over time (`context/slam_analysis.md`), the local map cost and pair count per `optimize` from `vio_pose_update.csv`, the size of the window's pose change at each keyframe marginalisation, and the `optimize` timing.
First replays of the comparison, 2026-11-21, `/ws/data/telemetry_40`, the first 51.8 s of flight, three replays in parallel per batch, two batches. The term opens at 14.4 s with the first fine installation and is active on 719 of 992 frames, with 8 shared keyframes, 28 pairs pairwise and 7 anchored. With the term off the replays are deterministic. With it on they vary with the timing of the mapper thread.

| Configuration | ATE after `se3`, m, two replays | Final position error, m | Median largest translation residual, m |
| --- | --- | --- | --- |
| Off | 1.475, 1.475 | 3.29, 3.29 | |
| `LocalMapRows2` | 2.388, 1.855 | 4.62, 2.84 | 0.560, 0.546 |
| `LocalMapRows` | 4.478, 1.683 | 12.26, 2.46 | 0.390, 0.406 |

The largest unweighted translation residual per frame has a median of 0.07 to 0.08 m between 10 and 20 s, consistent with $\sigma_p = 0.15$ m, and grows to 0.8 m pairwise and 0.6 m anchored by 35 to 45 s, while the period is still active and the cumulative scale correction reaches 1.15. The mapper's relative poses therefore drift away from the window's in this period, at several times the standard deviation the term assumes, and both topologies made the trajectory worse in this bag. Whether the disagreement lies in the mapper's scale during the period, in the carry of its poses through `GravityScaleCorrectionLog`, or in the mapper's geometry itself is not yet established, because the mapper's poses are not logged. Logging them, and replays that reach the end of the period, are outstanding.

5. A robustness check for §3.5.5 item 4. Offset the mapper pose of one keyframe by a fixed amount in a scratch build, once for the oldest shared keyframe and once for a middle one, and compare the window's response under both topologies.

## 7. Decisions and deliberately not done

1. Relative rather than absolute residuals, because the mapper's absolute position and yaw are gauge freedoms of its BA, and an absolute residual would import their drift.
2. No roll pitch residual, because the mapper's tilt comes from the estimator's own `RollPitchFactor` and adds no independent information. Gravity is corrected through the relative poses acting on the inertial residuals. ICE-BA's prior carries a roll pitch residual on its reference (§2.6.2), but that is the window's own information, not a correction from the global map.
3. No robust loss. The mean weighting gives a single wrong pair a share of $1/N$ of the pull.
4. No marginalisation of the rows, by §3.7.
5. Feature level coupling as in VINS-Mono [3] was rejected, because it needs map point correspondences in the window and a second landmark type in the linearisation.
6. One implementation of the relative pose residual and its Jacobians serves the mapper and the window, `relPoseErrorGS`, with `relPoseError` reduced to its scale one case (Change 2). The accumulation is not shared, because the mapper adds $\mathbf{J}^\top \boldsymbol{\Lambda} \mathbf{J}$ blocks inside a TBB reduction over its own `frame_poses` while the window writes whitened rows into the dense system of `optimize`, and merging them would change the mapper's code for no gain to it. Note, 2026-11-19. This item previously recorded that the mapper's recovered factors are not fed to the window, a question that had not been asked.
7. No overwrite in the manner of ICE-BA (§2.6.4), because Basalt's prior has first estimate Jacobians and the delayed store records first estimates, which an overwrite invalidates (§2.6.5).
8. Dense weights in the anchor topology, $(\mathbf{D} \mathbf{D}^\top)^{-1} = \mathbf{I} - \frac{\mathbf{K}_{N - 1}}{N}$ by §3.5.3, would make it equal to the pairwise topology and so remove the comparison. `LocalMapRows` therefore keeps the diagonal weights it was planned with, as the representative of anchored designs with independent pair weights.
9. Per pair information from the mapper, the NFR recovery of §2.7.2 applied to the mapper's own marginal over the shared keyframes, is not computed. The mapper does not expose that marginal, and computing it over its dense window would cost more than the mapper cycle. `RelPoseFactorRows` already whitens by a full `cov_inv`, so it can be adopted without changing the rows.
10. No relative position and yaw only residual in the manner of the VINS-Mono pose graph (§2.4.3). Roll and pitch of the relative pose are observable to both estimators, so dropping them would discard information without removing a gauge.
11. No filter on the window in `QueuePoseUpdates`. The mapper thread cannot read the window without a lock, a keyframe may leave the window between queueing and draining, and the drain of Change 4 with `GetLocalMapPoses` already restricts the term to the window being solved (Change 3).

## References


[1] R. Mur-Artal, J. D. Tardós, "Visual-Inertial Monocular SLAM with Map Reuse", IEEE Robotics and Automation Letters, 2017, arXiv:1610.05949. Eq. 2 to 8 and the tracking scheme read from the ar5iv text on 2026-11-17.

[2] C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, J. D. Tardós, "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM", IEEE Transactions on Robotics, 2021, arXiv:2007.11898. The statement on fixed states was confirmed from a search summary only.

[3] T. Qin, P. Li, S. Shen, "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator", IEEE Transactions on Robotics, 2018, arXiv:1708.03852. The window cost, the residuals, eq. 26, 28 and 29 read from the first 100000 characters of the ar5iv text on 2026-11-17, and eq. 10, 21, 22 and 23 with the definitions of their symbols on 2026-11-18. Eq. 23 is printed as $\rho(s) = 1$ for $s \geq 1$ and $2\sqrt{s} - 1$ for $s < 1$, read as a typesetting error in §2.4.1.

[4] H. Liu, M. Chen, G. Zhang, H. Bao, Y. Bao, "ICE-BA: Incremental, Consistent and Efficient Bundle Adjustment for Visual-Inertial SLAM", CVPR 2018. Confirmed from the abstract and search text only, since the CVF PDF returned HTTP 403 on 2026-11-17 and 2026-11-18. Its formulation is read from the code [11].

[5] S. Leutenegger, "OKVIS2: Realtime Scalable Visual-Inertial SLAM with Loop Closure", arXiv:2202.09199, 2022. The cost, the relative pose error and its derivation by Schur complement were read from the ar5iv text on 2026-11-17. Loop closure and synchronisation are described there in prose only.

[6] V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers, "Visual-Inertial Mapping with Non-Linear Factor Recovery", IEEE Robotics and Automation Letters, 2020, arXiv:1904.06504. Eq. 22 to 35 read from the ar5iv text on 2026-11-17, and the factor types confirmed from `include/basalt/utils/imu_types.h:504-521`.

[7] L. von Stumberg, D. Cremers, "DM-VIO: Delayed Marginalization Visual-Inertial Odometry", IEEE Robotics and Automation Letters, 2022, arXiv:2201.04114. As read in `context/dynamic_marginalisation.md` §3.3.

[8] J. Solà, J. Deray, D. Atchuthan, "A micro Lie theory for state estimation in robotics", arXiv:1812.01537, 2018. Cited from memory for (I1), (I2), the right Jacobian form of (I2) used in §2.7.4, and (I8).

[9] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", IEEE Transactions on Robotics, 2017, arXiv:1512.02363. As read in `context/dynamic_marginalisation.md` §3.5, with the bias expansion confirmed from [1].

[10] K. B. Petersen, M. S. Pedersen, "The Matrix Cookbook", Technical University of Denmark, 2012. Cited from memory for (I5), (I6) and (I10).

[11] Baidu Robotic Vision, ICE-BA source code, https://github.com/baidu/ICE-BA, Apache 2.0. `Backend/Geometry/CameraPrior.h` (`CameraPrior::Pose`, `GetError`, `SetPose`, `Initialize`), `Backend/BundleAdjustment/LocalBundleAdjustor.cpp` (`MarginalizeLocalFrame`, `SynchronizeData`, both `UpdateCameras` overloads) and `Backend/BundleAdjustment/GlobalBundleAdjustor.cpp` (`PushCameraPriorPose`, `PushCameraPriorMotion`, `GetFactor` from `m_Cs`) read through a summarising fetch of the raw files on 2026-11-18, which quoted lines of at most 125 characters. The bodies of `Pose::GetFactor` and the second half of `LocalBundleAdjustor.cpp` were read only in part.

[12] C. Godsil, G. Royle, "Algebraic Graph Theory", Springer, 2001, ch. 13, for the Laplacian and the spectra of the star and the complete graph. Cited from memory, and both spectra are verified in (I11) by the stated eigenvectors.

[13] S. Umeyama, "Least-Squares Estimation of Transformation Parameters Between Two Point Patterns", IEEE Transactions on Pattern Analysis and Machine Intelligence, 13(4), 1991. Cited from memory for the rotation of Change 1 Step 2, which `context/slam_analysis.md` also uses for its `se3` alignment.
