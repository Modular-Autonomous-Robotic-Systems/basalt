# Delayed Marginalisation as the Initialisation Mechanism of the Gravity and Scale Period

Status, 2026-11-13. Implemented in the working tree of branch `shreyas/kf_driven_mapping_and_logger`, where delayed marginalisation is the only mechanism of the gravity and scale period. Changes 1 to 11 are the plan of 2026-11-10, built, unit tested and replayed on `telemetry_40` as §3.4.2 and §3.4.3 record. Changes 12 to 15 revise it. They start the store at the first marginalisation with the block held in the window and corrected by a coarse inertial optimisation (§2.3.9 and §2.3.11), keep the store for the whole period (§2.3.7), carry the gravity prior through the delayed prior and the coarse optimisation (§2.3.12), and remove the dynamic marginalisation, the tilt hold, the update cadence and the window's accelerometer bias step prior (§2.3.13). The revision was built and replayed on `telemetry_34` and `telemetry_100` (§3.4.5), and its unit tests remain to be run (§3.4.4). Change 16 of 2026-11-14 replaces its coarse optimisation by DM-VIO's inertial only estimate of the velocities, both biases, the gravity direction and the scale, followed by a readvance in the same job (§2.3.11, §2.3.14 and §2.3.15), and was syntax checked, built standalone with every unit test passing, and compared with the form it replaces by Monte Carlo (§3.4.6), but not replayed. Change 17 of the same day replaces it by DM-VIO's coarse initialiser transferred as closely as Basalt allows, with keyframe nodes, the scale on the held positions, one bias, a trigger at every keyframe and every job installed one frame after its snapshot (§2.3.11 and §3.4.7), also not replayed. Every `file:line` anchor of §1 to §3.3 was read on 2026-11-10 against the tree before the revision, so anchors into `sqrt_keypoint_vio.cpp` are offset in the current tree. The document is written to be read on its own, and every result it borrows is restated with its derivation. Note, 2026-11-13. This paragraph previously stated that nothing was applied to the working tree and that the plan extended plans 1 and 2 with an opt in mode beside their dynamic marginalisation, which the revision superseded.

## 1. Introduction

### 1.1 The gravity and scale period

Basalt fixes the direction of gravity from the first accelerometer sample and has no variable for metric scale (`src/vi_estimator/sqrt_keypoint_vio.cpp:248`). On the vertical take off of every SITL flight neither quantity is resolved in the first seconds, and the error is absorbed by the accelerometer bias and frozen into the marginalisation prior (`context/takeoff_bias_divergence.md`). The first plan therefore added an initialisation period in which a three dimensional block $\boldsymbol{\delta}_{gs} = [\delta\theta_1, \delta\theta_2, \delta\sigma]^\top$, two tilt coordinates of the gravity direction and a log scale, is optimised with every pose, velocity, bias and landmark of the window. The block enters the inertial residual explicitly and the visual residual through the extrinsic lever arm. The three gauge freedoms it creates are removed by an orthogonal projection, the estimated correction is applied to every state and prior at the end of each frame after which the block is the identity again, and an epoch stamped correction log keeps the local mapper in the same world. Old frames are marginalised into a square root prior with first estimate Jacobians, and, following VI-DSO [1], three priors $M_\text{curr}$, $M_\text{half}$ and $M_\text{visual}$ are kept and swapped whenever the scale leaves an interval of half width $\ln d_\text{min}$ with $d_\text{min} = \sqrt{1.1}$. Plan 2 added Change A, which keeps the scale free rows of the inertial factors in $M_\text{visual}$, and Change B, a random walk residual on the per frame step of the block. The working SITL configuration enables both and sets `vio_gs_duration_s` to 7000 s (`data/sitl_config_vo.json:50`), so the period lasts the whole flight.

### 1.2 Why dynamic marginalisation is not sufficient

The marginalisation scheme of the period has to solve one problem. An inertial factor contains the product of the scale and a velocity difference, and a prior built from it is a linear model at the scale of the moment it was built. Dynamic marginalisation solves the problem by discarding information, and four observations show that the price is too high for a take off.

1. The linear model of a frozen inertial factor is wrong by a bilinear term. With $\mathbf{u} = \mathbf{v}_1 - \mathbf{v}_0$ and the velocity row $\mathbf{r}_v = \mathbf{R}_0^\top (s\,\mathbf{u} - \mathbf{g}_D \Delta t) - \hat{\Delta\mathbf{v}}$ linearised at $(\bar{s}, \bar{\mathbf{u}})$ and evaluated at $(\bar{s} + \Delta s, \bar{\mathbf{u}} + \Delta\mathbf{u})$, the error is $\Delta s\,\mathbf{R}_0^\top \Delta\mathbf{u}$ (§2.1.7 derives it). With $\Delta s = 0.5$ and $|\Delta\mathbf{u}| = 1$ m/s it is 0.5 m/s, against a standard deviation of the preintegrated velocity of about 0.018 m/s over one frame (plan 2 §3.2.2). Every scheme must therefore keep the scale at which a prior was linearised close to the current scale.
2. Dynamic marginalisation keeps that distance small by swapping priors, and a swap discards every inertial factor the outgoing prior held and the incoming one did not. The scheme assumes that the scale moves slowly compared with $\ln d_\text{min} \approx 0.048$ per marginalisation. At take off single solves returned steps of up to $\delta\sigma = -0.56$ and 1.04, so a swap fired almost every frame, each discarding what had accumulated, and the prior in use held one or two inertial factors at a time (plan 2 §3.2.5). The replay of 2026-11-06 with Changes A and B logged 62 swap events, most of them between 55 s and 70 s (plan 2 §5.5 item 3).
3. Change A made the fallback prior keep everything that is free of the scale, namely the rotation rows, the bias random walk rows and the full rows of factors without acceleration. What it cannot keep, by construction, is the scale information of accelerating factors, loss L5 of `context/dynamic_marginalisation.md` §6.1.2, together with the position pins and the ground phase tilt held only in the scale dependent rows, losses L1 and L4. The scale is decided in the take off transient, because the cruise carries almost no scale information, and the two replays with Changes A and B ended at scale ratios of 1.14 and about 3 (plan 2 §5.5 item 2). Plan 2 attributes the spread to the scale dependent rows discarded at each swap and names delayed marginalisation as the recommended next step (§3.6 item 1 and §5.5 item 3 there).
4. VI-DSO's construction is one dimensional by design. DM-VIO describes it as keeping the prior consistent but losing "too much information in the process", and as "limited in its application to a single one-dimensional variable, namely the scale" [2, §I and §III-D]. The period estimates the tilt as well, and the tilt is held in $M_\text{curr}$ only as long as its swaps allow.

### 1.3 What delayed marginalisation changes

Delayed marginalisation [2] keeps a second record of the last $d$ marginalisations with their original factors, so that a prior can be rebuilt at any time with every inertial factor of those $d$ steps relinearised at the current scale and tilt. The rebuilt prior replaces the one in use when the scale or the tilt has moved past a threshold since the prior's linearisation. Nothing of the last $d$ steps is selected out, only relinearised, so losses L1 to L5 are kept within the delay (`context/dynamic_marginalisation.md` §6.3), and with $d = 200$ marginalisations, 10.8 s at the 54 ms frame period of the SITL camera, the whole ground phase and take off transient of the live runs stays inside the delay. The three priors and the swap rule become one prior and a replacement rule. DM-VIO uses the same machinery to initialise its inertial system [2, §III-E and §III-F], by optimising the delayed graph with its inertial factors (pose graph bundle adjustment) and readvancing it into the prior of the main system. This plan uses delayed marginalisation in the same role for Basalt's gravity and scale period, with a square root smoother over the delayed window in the place of DM-VIO's pose graph bundle adjustment (§2.3.6).

The revision of 2026-11-13 changes how the period begins. Basalt's window is metric from the first frame, the initial gravity direction from the accelerometer was accurate to about 1° in the SITL runs (§2.3.12), and the vehicle needs metric poses from lift off, so the estimator cannot wait for an initialiser as DM-VIO's visual odometry does. The store therefore starts at the first marginalisation with the block held in the window. Until the store holds $d$ steps, the coarse initialiser of DM-VIO [2, §III-F] runs in the worker at every new keyframe, holding every keyframe pose and estimating the gravity direction, the scale, one bias of both sensors and the keyframe velocities, and readvances the prior at its result in the same job, and the VIO thread installs the correction by the re-centring together with the readvanced prior at the next frame. Note, 2026-11-14. Until Change 17 this optimisation ran every $k$ marginalisations with nodes $k$ frames apart and the scale on the inertial side. Note, 2026-11-14. This sentence previously named the initialisation of Mur-Artal and Tardós [7], which estimated only the gravity direction, the log scale and the accelerometer bias with the velocities eliminated and was followed by a separate readvance. Once the store holds $d$ steps, the block is freed in the window and the smoothed readvance and replacement rule of DM-VIO take over. The period thus combines an ORB-SLAM style initialisation, which corrects a running metric estimator, with DM-VIO's delayed marginalisation, which keeps every inertial factor of the take off relinearisable.

What the plan does not claim matters as much. Delayed marginalisation keeps information and makes its linearisation consistent. It does not by itself remove the kinematic inconsistency of the window at lift off, failure F1 of `context/dynamic_marginalisation.md` §6.1.2, nor bound the cumulative correction of a permanent period, failure F2 (§6.3 Q5 there). The smoother of §2.3.6 revises the take off history, which bears on F1, and the optional end of the period of §2.3.7 bears on F2, and both are offered as mechanisms whose effect the replays and flights of §3.4 must establish.

### 1.4 Structure of the document

§2.1 formulates delayed marginalisation in general, the full graph, the main graph, the delayed graph, readvancing, the marginalisation replacement and DM-VIO's inertial initialisation, with every identity and step. §2.2 describes how DM-VIO implements it in real time, read from its source, with a flow chart. §2.3 derives the Basalt formulation from the estimator of the period as it is implemented, including the latency analysis that fixes the configuration. §3 gives the architecture, the changes with their code exactly as applied to the scratch copy, the blast radius, the validation performed and outstanding, and the review of the plan against the code.

## 2. Delayed Marginalisation Formulation and Architecture

### 2.1 Formulation

#### 2.1.1 Notation

The estimator's variables are the frame states, the landmarks and, in the period, the block. A frame state $f$ holds the body pose $(\mathbf{R}_f, \mathbf{p}_f)$, the velocity $\mathbf{v}_f$ and the biases $\mathbf{b}_{g,f}$, $\mathbf{b}_{a,f}$, with the tangent ordering $[\delta\mathbf{p}, \delta\boldsymbol{\varphi}, \delta\mathbf{v}, \delta\mathbf{b}_g, \delta\mathbf{b}_a]$ and the update rules $\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}$, $\mathbf{R} \leftarrow \text{Exp}(\delta\boldsymbol{\varphi})\,\mathbf{R}$, $\mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v}$ and additive biases (`PoseVelBiasState::applyInc`, `thirdparty/basalt-headers/include/basalt/imu/imu_types.h:221-225`). A keyframe pose holds the first six coordinates only. The block holds the gravity direction $\mathbf{d}$ and the log scale $\sigma$, with $s = e^\sigma$ and $\mathbf{g}_D = |\mathbf{g}|\,\mathbf{d}$, stored as a linearisation point $(\bar{\mathbf{d}}, \bar{\mathbf{B}}, \bar{\sigma})$ and an offset, $\mathbf{d} = \text{Exp}(\bar{\mathbf{B}} \boldsymbol{\delta}_\theta)\,\bar{\mathbf{d}}$ and $\sigma = \bar{\sigma} + \delta\sigma$, where $\bar{\mathbf{B}}$ is an orthonormal basis of the plane perpendicular to $\bar{\mathbf{d}}$ (`GravityScaleStateWithLin`, `include/basalt/utils/imu_types.h:329-395`).

$\boldsymbol{\delta}$ denotes the stacked tangent offsets of a set of variables from their linearisation points, and $\mathbf{x}$ an increment applied on top of them. A factor set $\mathcal{F}$ linearised at a point is written as a stack of whitened rows $\mathbf{r} + \mathbf{J} \mathbf{x}$, and its information as $\mathbf{H} = \mathbf{J}^\top \mathbf{J}$, $\mathbf{b} = \mathbf{J}^\top \mathbf{r}$. Marginalisation steps are indexed by $j$, with $\beta_j$ the coordinates removed at step $j$ and $\mathcal{F}_j$ the factors removed with them. $\text{marg}_\beta(\cdot)$ denotes the removal of $\beta$ from a linear system by the Schur complement of (D1). $d$ is the delay in marginalisation steps.

#### 2.1.2 Identities

The derivations of this document cite the following identities by label.

(D1) Minimisation over a block. For a symmetric $\mathbf{H}$ with $\mathbf{H}_{\beta\beta}$ invertible, partitioned into kept coordinates $\alpha$ and removed coordinates $\beta$, the minimum over $\boldsymbol{\delta}_\beta$ of $F = \frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H} \boldsymbol{\delta} + \mathbf{b}^\top \boldsymbol{\delta}$ is $\frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}' \boldsymbol{\delta}_\alpha + \mathbf{b}'^\top \boldsymbol{\delta}_\alpha + \text{const}$ with

$$\mathbf{H}' = \mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{H}_{\beta\alpha},$$

$$\mathbf{b}' = \mathbf{b}_\alpha - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{b}_\beta.$$

Proof. Setting the derivative of $F$ with respect to $\boldsymbol{\delta}_\beta$ to zero gives $\mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha + \mathbf{H}_{\beta\beta} \boldsymbol{\delta}_\beta + \mathbf{b}_\beta = \mathbf{0}$, so $\boldsymbol{\delta}_\beta^* = -\mathbf{H}_{\beta\beta}^{-1}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)$. At $\boldsymbol{\delta}_\beta^*$ the three terms of $F$ that contain $\boldsymbol{\delta}_\beta$ sum to $-\frac{1}{2}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)^\top \mathbf{H}_{\beta\beta}^{-1} (\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)$, and collecting the terms quadratic and linear in $\boldsymbol{\delta}_\alpha$ gives $\mathbf{H}'$ and $\mathbf{b}'$. The full derivation is plan 2 §3.1.6 and `context/dynamic_marginalisation.md` §3.1.1. This is eq. 10 and 11 of [2].

(D2) Successive removal equals joint removal. For $\beta = \beta_1 \cup \beta_2$,

$$\text{marg}_{\beta_1 \cup \beta_2}(\mathcal{S}) = \text{marg}_{\beta_2}\big(\text{marg}_{\beta_1}(\mathcal{S})\big).$$

Proof. By (D1) each side is the quadratic obtained by minimising $F$ over the removed coordinates, and a minimum over two blocks equals the minimum over one block of the minimum over the other (`context/dynamic_marginalisation.md` §3.1.2). The order of removal therefore changes the cost of the computation and not its result, provided every factor is linearised at the same point in both.

(D3) Square root marginalisation. Let $[\mathbf{J}_\beta\ \mathbf{J}_\alpha \,|\, \mathbf{r}]$ be the stacked rows with the removed columns first, and let a Householder QR give an orthogonal $\mathbf{Q}$ with

$$\mathbf{Q}^\top [\mathbf{J}_\beta\ \mathbf{J}_\alpha \,|\, \mathbf{r}] = \begin{bmatrix} \mathbf{R}_{\beta\beta} & \mathbf{R}_{\beta\alpha} & \mathbf{c}_\beta \\ \mathbf{0} & \mathbf{J}' & \mathbf{r}' \end{bmatrix}.$$

Then $\mathbf{J}'^\top \mathbf{J}' = \mathbf{H}'$ and $\mathbf{J}'^\top \mathbf{r}' = \mathbf{b}'$ of (D1). Proof. $\mathbf{Q}$ is orthogonal, so $|\mathbf{r} + \mathbf{J} \mathbf{x}|^2$ equals the squared norm of the transformed rows, $|\mathbf{R}_{\beta\beta} \mathbf{x}_\beta + \mathbf{R}_{\beta\alpha} \mathbf{x}_\alpha + \mathbf{c}_\beta|^2 + |\mathbf{J}' \mathbf{x}_\alpha + \mathbf{r}'|^2$. For an invertible $\mathbf{R}_{\beta\beta}$ the first term is made zero by $\mathbf{x}_\beta = -\mathbf{R}_{\beta\beta}^{-1}(\mathbf{c}_\beta + \mathbf{R}_{\beta\alpha} \mathbf{x}_\alpha)$, so the minimum over $\mathbf{x}_\beta$ is the second term, whose information is $\mathbf{J}'^\top \mathbf{J}'$ and whose gradient is $\mathbf{J}'^\top \mathbf{r}'$, and by (D1) the minimum is also $\frac{1}{2}\mathbf{x}_\alpha^\top \mathbf{H}' \mathbf{x}_\alpha + \mathbf{b}'^\top \mathbf{x}_\alpha + \text{const}$. Basalt implements this as `MargHelper::marginalizeHelperSqrtToSqrt` (`src/vi_estimator/marg_helper.cpp:258-347`), which skips a column whose Householder pivot is below $\sqrt{\epsilon}$, the Moore–Penrose form of a rank deficient removal [3]. The same argument shows that any orthogonal transformation of a stack of rows, in particular the compression of $m$ rows over $n < m$ columns to the $n$ rows of their triangular factor, leaves $\mathbf{H}$ and $\mathbf{b}$ unchanged and changes only the constant.

(D4) The two forms of a linear prior. A prior with Jacobian $\mathbf{J}$ built at linearisation points $\bar{\mathbf{x}}$ is stored delta independently, $P(\mathbf{x}) = \frac{1}{2}|\mathbf{J}(\boldsymbol{\delta} + \mathbf{x}) + \mathbf{r}_0|^2$, where $\boldsymbol{\delta}$ is the offset of the current value from $\bar{\mathbf{x}}$ (`sqrt_keypoint_vio.cpp:1543-1564`). Evaluated at the current value it reads $P(\mathbf{x}) = \frac{1}{2}|\mathbf{J} \mathbf{x} + \mathbf{r}_c|^2$ with

$$\mathbf{r}_c = \mathbf{r}_0 + \mathbf{J} \boldsymbol{\delta}.$$

Proof. Expand $\mathbf{J}(\boldsymbol{\delta} + \mathbf{x}) + \mathbf{r}_0 = \mathbf{J} \mathbf{x} + (\mathbf{r}_0 + \mathbf{J} \boldsymbol{\delta})$. Conversely a set of rows evaluated at the current value becomes delta independent by $\mathbf{r}_0 = \mathbf{r}_c - \mathbf{J} \boldsymbol{\delta}$, which is the subtraction `marg_data.b -= marg_data.H * delta` of `sqrt_keypoint_vio.cpp:1569-1571`. Basalt's offsets are exact, because a frozen variable's current value is defined as its linearisation point with the accumulated offset applied (`PoseVelBiasStateWithLin::applyInc`, `include/basalt/utils/imu_types.h:117-126`).

(D5) Re-centring maps rows exactly. The re-centring of the first plan §2.7 applies the similarity $(\mathbf{Q}, s_c)$ to every pose, velocity and linearisation point, $\mathbf{p} \to s_c \mathbf{Q} \mathbf{p}$, $\mathbf{R} \to \mathbf{Q} \mathbf{R}$, $\mathbf{v} \to s_c \mathbf{Q} \mathbf{v}$, leaves the biases unchanged, and moves the block to the identity. The tangent offsets of a frame map as $\boldsymbol{\delta}' = \mathbf{A} \boldsymbol{\delta}$ with $\mathbf{A} = \text{blkdiag}(s_c \mathbf{Q}, \mathbf{Q}, s_c \mathbf{Q}, \mathbf{I}, \mathbf{I})$, every residual is invariant (first plan §2.7.2 and §2.7.3), and so a row set is carried to the new coordinates exactly by $\mathbf{J}' = \mathbf{J} \mathbf{A}^{-1}$ with $\mathbf{r}_0$ unchanged, since $\mathbf{J}' \boldsymbol{\delta}' = \mathbf{J} \mathbf{A}^{-1} \mathbf{A} \boldsymbol{\delta} = \mathbf{J} \boldsymbol{\delta}$. This is the column map `transform_prior` of `RecentreGravityScale` (`sqrt_keypoint_vio.cpp:2333-2345`). Two successive maps compose to one, $\mathbf{A}_1^{-1} \mathbf{A}_2^{-1} = (\mathbf{A}_2 \mathbf{A}_1)^{-1}$, and $\mathbf{A}_2 \mathbf{A}_1$ is the map of the composed similarity $(\mathbf{Q}_2 \mathbf{Q}_1, s_1 s_2)$, which is what `GravityScaleCorrectionLog::Between` returns (`include/basalt/utils/imu_types.h:427-436`). A row set stored at epoch $e$ is therefore carried to epoch $e'$ in one step by the entry $\text{Between}(e, e')$.

(D6) Monotonicity of the marginal. If $\mathbf{H}_1 \succeq \mathbf{H}_0$, their Schur complements onto the same kept block satisfy $\mathbf{H}_1' \succeq \mathbf{H}_0'$. Proof in plan 2 §3.1.6 (N2). Adding factors to a factor set therefore never removes information from its marginal in any direction.

(D7) Back substitution solves the conditional least squares problem. In the triangular system of (D3), for given $\mathbf{x}_\alpha$ the minimiser over $\mathbf{x}_\beta$ of $|\mathbf{r} + \mathbf{J} \mathbf{x}|^2$ is

$$\mathbf{x}_\beta^* = -\mathbf{R}_{\beta\beta}^{-1}(\mathbf{c}_\beta + \mathbf{R}_{\beta\alpha} \mathbf{x}_\alpha).$$

Proof. The proof of (D3) splits the cost into a first term that $\mathbf{x}_\beta^*$ makes zero and a second term free of $\mathbf{x}_\beta$. A nonnegative term made zero is minimal. When a column of $\mathbf{R}_{\beta\beta}$ has no pivot, its coordinate does not enter the first term and any value minimises, and the solution sets it to zero.

(D8) Bilinear error of a frozen product. For a scalar $\sigma$ and a vector $\mathbf{u}$, linearising $e^\sigma \mathbf{u}$ at $(\bar{\sigma}, \bar{\mathbf{u}})$ and evaluating at $(\bar{\sigma} + \Delta\sigma, \bar{\mathbf{u}} + \Delta\mathbf{u})$ leaves the error

$$e^{\bar{\sigma}}\big((e^{\Delta\sigma} - 1 - \Delta\sigma)\,\bar{\mathbf{u}} + (e^{\Delta\sigma} - 1)\,\Delta\mathbf{u}\big).$$

Proof. The exact value is $e^{\bar{\sigma}} e^{\Delta\sigma} (\bar{\mathbf{u}} + \Delta\mathbf{u})$. The linear model is the value $e^{\bar{\sigma}} \bar{\mathbf{u}}$ plus the derivative with respect to $\mathbf{u}$, $e^{\bar{\sigma}} \Delta\mathbf{u}$, plus the derivative with respect to $\sigma$, $e^{\bar{\sigma}} \bar{\mathbf{u}}\,\Delta\sigma$. Subtracting and grouping the terms in $\bar{\mathbf{u}}$ and in $\Delta\mathbf{u}$ gives the error. To leading order it is $e^{\bar{\sigma}}(\frac{1}{2}\Delta\sigma^2\,\bar{\mathbf{u}} + \Delta\sigma\,\Delta\mathbf{u})$, and it vanishes for every $\Delta\mathbf{u}$ exactly when $\Delta\sigma = 0$. With $s = e^\sigma$ the second term is the $\Delta s\,\Delta\mathbf{u}$ of §1.2 item 1.

(D9) Block coordinate descent does not increase a cost. If a cost $F(\mathbf{y}, \mathbf{z})$ is minimised exactly over $\mathbf{z}$ for fixed $\mathbf{y}$ and then over $\mathbf{y}$ for fixed $\mathbf{z}$, neither step raises $F$, since each new value is a minimum over a set that contains the old one. For a convex quadratic the iteration converges to the joint minimum.

(D10) A factor evaluated through the metric states is gauge invariant. A factor that depends on the estimator's variables only through the metric states $\mathbf{R}^M = \mathbf{R}_{wg}^\top \mathbf{R}$, $\mathbf{p}^M = s\,\mathbf{R}_{wg}^\top \mathbf{p}$ and $\mathbf{v}^M = s\,\mathbf{R}_{wg}^\top \mathbf{v}$, and is invariant under yaw, satisfies $\mathbf{J} \mathbf{N} = \mathbf{0}$ for the three gauge vectors of the block when its Jacobians are evaluated at the points from which $\mathbf{N}$ is built. This is (M11) of `context/dynamic_marginalisation.md` §6.2, proved there. Every inertial factor of the period (`residualGS`) and every visual factor is of this kind.

#### 2.1.3 The full graph

Without marginalisation the estimator would minimise, over every frame state, landmark and the block, the cost of every factor ever created,

$$F(\mathcal{X}) = \frac{1}{2}|\mathbf{r}_0(\mathcal{X})|^2 + \sum_{v} \frac{1}{2}|\mathbf{r}_v(\mathcal{X})|^2 + \sum_{i} \frac{1}{2}|\mathbf{r}_i(\mathcal{X})|^2,$$

where $\mathbf{r}_0$ is the initial prior on the first state, the sum over $v$ runs over the whitened, robustified visual residuals and the sum over $i$ over the whitened inertial residuals with their bias random walk rows. This is the problem that iSAM2 keeps whole (`context/dynamic_marginalisation.md` §6.4), and Basalt's window solves a fixed lag version of it [5]. It is consistent by construction, because a Gauss–Newton step relinearises every factor at the current estimate, and it is unbounded in size. DM-VIO's factor graph is the same with photometric instead of reprojection residuals, its energy being $E(\mathbf{s}) = W(e_\text{photo})\,E_\text{photo} + E_\text{imu} + E_\text{prior}$ [2, eq. 1].

#### 2.1.4 The current graph, marginalisation and first estimates

A fixed lag estimator keeps a window and removes old variables. At step $j$ it removes the coordinates $\beta_j$ together with the factors $\mathcal{F}_j$ connected to them, and keeps the marginal as a linear prior. Starting from the initial prior $M_0$,

$$M_j = \text{marg}_{\beta_j}\big(M_{j-1} + \mathcal{F}_j(\bar{\mathbf{x}}_j)\big),$$

where $\mathcal{F}_j(\bar{\mathbf{x}}_j)$ is the factor set linearised at the linearisation points $\bar{\mathbf{x}}_j$ of its variables at step $j$. By (D1) and (D3) $M_j$ is the exact marginal of the linearised problem. The prior is a function of the offsets of its variables from the points at which its Jacobians were evaluated, so every variable that enters it is frozen at that point and every later factor on it is evaluated there too. Otherwise the null spaces of the prior and the later factors would differ and their sum would appear to observe a gauge direction [1, 4]. Basalt freezes a state when it first becomes the oldest full state of the window (`setLinTrue`, `sqrt_keypoint_vio.cpp:1478`), and DM-VIO describes the consequence, that "the linearization point of all connected variables has to be fixed" [2, §III-D].

In Basalt one step removes, per frame, the states older than the oldest state kept, `last_state_to_marg`. A non keyframe state leaves whole, a keyframe state leaves its velocity and biases and keeps its pose as a keyframe pose, and once the window holds more than `vio_max_kfs` keyframes one keyframe pose leaves with the landmarks it hosts (`sqrt_keypoint_vio.cpp:1085-1223`, `context/vio_opt.md` §12.1). The removed factors are the inertial factor of each departing state, the visual factors of the landmarks hosted by a departing keyframe, and, with `vio_marg_lost_landmarks`, the visual factors of every landmark whose track was lost in the frame (`context/vio_opt.md` §12.2). In the SITL replay of `_23-21-18` one step removes a median of 15 coordinates from a system of 75 and keeps 60 (`vio_marg.csv`).

#### 2.1.5 The delayed graph

DM-VIO's observation is that "marginalization cannot be undone, but it can be delayed" [2, §III-D]. Beside the main graph it keeps "a second, delayed marginalization prior and corresponding factor graph", in which every removal of the main graph is repeated $d$ steps later, in the same order [2, §III-D]. At step $k$ the delayed graph holds

$$\mathcal{G}^d_k = \Big(M^d_{k-d},\ \{(\beta_j, \mathcal{F}_j)\}_{j = k-d+1}^{k}\Big),$$

a delayed prior $M^d_{k-d}$ that summarises everything removed up to step $k - d$, and the original factors and removal sets of the last $d$ steps, which the main graph has already turned into its prior. When a step is added the oldest one is removed from the delayed graph,

$$M^d_{k-d+1} = \text{marg}_{\beta_{k-d+1}}\big(M^d_{k-d} + \mathcal{F}_{k-d+1}(\hat{\mathbf{x}})\big).$$

Because the delayed graph removes the same sets in the same order, its Markov blanket is that of the main graph, and "marginalization in the delayed graph has the same runtime as marginalization in the original graph" [2, §III-D]. DM-VIO uses $d = 100$ keyframes with at most $N_f = 8$ keyframes in the window [2, §III-D and §III-F].

#### 2.1.6 Readvancing

A prior with new linearisation points is obtained by readvancing the delayed graph, which "works by successively marginalizing all the variables which have been marginalized in the main graph" [2, §III-E], with every factor linearised at a chosen evaluation point $\hat{\mathbf{x}}$. Starting from $R_{k-d} = M^d_{k-d}$,

$$R_j = \text{marg}_{\beta_j}\big(R_{j-1} + \mathcal{F}_j(\hat{\mathbf{x}})\big),$$

for $j = k - d + 1$ to $k$. Step 1. By (D2) applied $d - 1$ times, the result equals the joint removal,

$$R_k = \text{marg}_{\beta_{k-d+1} \cup \cdots \cup \beta_k}\Big(M^d_{k-d} + \sum_{j = k-d+1}^{k} \mathcal{F}_j(\hat{\mathbf{x}})\Big).$$

Step 2. Every factor of the last $d$ steps enters $R_k$, so nothing is selected out, only relinearised. Step 3. The successive form is preferred because each step inverts one small block, "marginalizing step by step is significantly faster than marginalizing all variables at once" [2, §III-E]. Step 4. The variables kept at the end, $\alpha_k$, are the variables of the main prior $M_k$, so $R_k$ can replace $M_k$ provided its linearisation points for $\alpha_k$ are those at which the window evaluates them, by §2.1.4. DM-VIO's readvanced prior has "new and consistent linearization points" [2, abstract].

#### 2.1.7 Marginalisation replacement

DM-VIO optimises scale and gravity direction "as explicit variables", with the visual frame of arbitrary scale and rotation and the yaw coordinate of the gravity rotation fixed [2, §III-B]. It compares the current scale with the scale at the prior's first estimate,

$$\delta_s = \frac{\max(s, s_\text{fej})}{\min(s, s_\text{fej})},$$

and when $\delta_s$ exceeds a threshold it triggers a marginalisation replacement, in which "we do not optimize in this graph but instead just readvance it to obtain an updated marginalization prior" [2, §III-F]. The replacement is disabled "if more than $\theta_\text{lost} = 50\,\%$ of the IMU factors in the previous prior would be lost" [2, §III-F]. In the code the threshold is `threshScale = 1.02` and the gravity threshold `threshGravdir = 1000` rad, which disables it (`IMUInitSettings.cpp:43` and `IMUInitSettings.h:82-83` of [6]).

The error a replacement removes and the error it leaves follow from (D8). Step 1. Right after a replacement every inertial factor of the last $d$ steps is linearised at the current scale, so $\Delta\sigma = 0$ and the error of (D8) is zero. Step 2. Until the next replacement $\delta_s \le \theta_s$, so $|\Delta\sigma| \le \ln\theta_s$. Step 3. Substituting into the leading term of (D8), the error of a velocity row is bounded by

$$|\mathbf{r}_v - \mathbf{r}_v^\text{lin}| \le s_\text{fej}\Big(\frac{1}{2}(\ln\theta_s)^2 |\bar{\mathbf{u}}| + \ln\theta_s\,|\Delta\mathbf{u}|\Big).$$

Step 4. At $\theta_s = 1.02$, $\ln\theta_s = 0.0198$, and with $|\Delta\mathbf{u}| = 1$ m/s and $|\bar{\mathbf{u}}| = 0.24$ m/s, the velocity change over 54 ms at the 4.5 m/s² of the live climb, the bound is $0.0198 + 0.00005 \approx 0.020$ m/s, about one standard deviation of the preintegrated velocity. Step 5. Dynamic marginalisation bounds the spread of scales inside $M_\text{curr}$ by $2\ln d_\text{min} = 0.095$ (`context/dynamic_marginalisation.md` §3.2.4), five times larger, and only while its swaps keep up with the scale. The two bounds are of the same kind. The difference is that a replacement keeps the information, so no ratchet can form, and by (D6) the replaced prior holds at least the information of a prior reset to the delayed prior.

#### 2.1.8 Inertial initialisation by delayed marginalisation in DM-VIO

DM-VIO starts as visual odometry and runs a multi stage inertial initialiser beside it [2, §III-F].

1. Coarse initialisation. The coarse initialisation uses the last $d = 100$ keyframes, holds their poses fixed and optimises velocities, biases, gravity direction and scale, with gravity initialised from the accelerometer between the first two keyframes, scale 1 and zero bias and velocity [2, §III-F]. In the code the poses are held through `fixPoses` and the factors are inertial factors transformed into the visual frame (`CoarseIMUInitOptimizer::addPose`, `CoarseIMUInitOptimizer.cpp:67-111` of [6]). "We compute the marginal covariance for the scale cov(s) and continue to the PGBA if it is smaller than a threshold $\theta_\text{init}$" [2, §III-F], the variance of the log scale, since the scale variable's tangent is the log of the Sim(3) scale (`ScaleGTSAM::Logmap`, `Sim3GTSAM.cpp:72-78` of [6]). The default is `coarseScaleUncertaintyThresh = 1.0` (`IMUInitSettings.h:100` of [6]). Basalt cannot hold back its estimate during this stage, and §2.3.11 derives the coarse optimisation it runs instead beside a window that is metric from the first frame.
2. Pose graph bundle adjustment (PGBA). Inertial and bias factors are inserted into a copy of the delayed graph from the first frame of the delayed graph that is still directly connected to the newest frame, and the graph is optimised with all its variables, while "we do not update our estimates for point depths and do not relinearize photometric error terms" [2, §III-E]. The visual factors of the delayed graph are therefore linear factors at their first estimates, and only the inertial factors, the scale and the gravity direction are relinearised.
3. Readvance and initialisation. The optimised graph is readvanced to delay zero and becomes the main graph, so the main system starts with a prior that holds all the visual and inertial information of the delayed window [2, §III-E]. If the marginal scale variance after the PGBA is below $\theta_\text{init}$ but above $\theta_\text{reinit}$, the system initialises and runs another PGBA later, and it stops reinitialising once the variance falls below $\theta_\text{reinit}$ (defaults 1.0 and 0.5, `IMUInitSettings.h:64-65` of [6]).
4. Marginalisation replacement. After initialisation the replacement of §2.1.7 keeps the prior consistent while the scale still moves, and it ends once a replacement would drop more than half of the delay (§2.2.2).

The cost DM-VIO reports on EuRoC in real time mode was 10.34 ms of tracking and 53.67 ms of keyframe processing on average, of which maintaining the delayed graph took 0.44 ms, 0.8 % [2, §IV-A].

#### 2.1.9 Information kept, compared with dynamic marginalisation

The comparison is made with the losses L1 to L5 of `context/dynamic_marginalisation.md` §6.1.2, the position pins (L1), the yaw pin (L2), the accelerometer bias anchor (L3), the roll and pitch of the ground phase (L4) and the scale information of accelerating factors (L5).

| Loss | Dynamic marginalisation with Change A, after a swap | Delayed marginalisation, within $d$ steps | Delayed marginalisation, beyond $d$ steps |
|---|---|---|---|
| L1 position pins | lost unless a landmark links the first state | kept through the relinearised position rows | as Change A, lost |
| L2 yaw pin | kept through the rotation rows | kept | kept through the rotation rows |
| L3 bias anchor | kept through the random walk rows | kept | kept through the random walk rows |
| L4 ground roll and pitch | lost, the tilt held only by L3 | kept with all nine rows of every ground factor | only through factors admitted by Change A |
| L5 scale of accelerating factors | lost at every swap | kept and relinearised | lost |

The middle column follows from Step 2 of §2.1.6. The right column holds because the delayed prior is advanced with the scale free rows of Change A (§2.3.5), and by (D6) a readvanced prior holds at least the information of that delayed prior. The live runs spent 4 s to 5 s on the ground before lift off (plan 2 §5.5 item 5), so at $d = 200$ steps of 54 ms every factor from the first frame to the end of the take off transient is in the delay when the transient is decided.

### 2.2 The real time implementation in DM-VIO

This section describes DM-VIO's implementation as read from its repository [6], cloned at commit `e306328` on 2026-11-10, so that the Basalt architecture of §2.3 and §3.1 can be compared with it piece by piece. Paths are relative to `src/` of that repository.

#### 2.2.1 Graph classes

`DelayedGraph` (`GTSAMIntegration/DelayedMarginalization.h:35-92`) is one factor graph with a delay. It stores the nonlinear factors, the queue `marginalizationOrder` of key sets removed by the main graph, and two value sets, `delayedValues`, the evaluation point (first estimates for visual variables, current values for the rest), and `delayedCurrValues`, the current values. `marginalize` appends a key set and its values and calls `readvanceUntilDelay`, which removes key sets from the front while more than `delayN` are queued (`DelayedMarginalization.cpp:191-253`). Each removal linearises the connected factors at the first estimate values and replaces them by a linear container factor (`marginalizeOut`, `GTSAMIntegration/Marginalization.cpp`). `readvanceGraph(newDelay)` lowers the delay and readvances, and `setMarginalizationPaused` holds the queue while a copy is in use.

`DelayedMarginalizationGraphs` (`DelayedMarginalization.h:115-163`) owns the main graph, which is a `DelayedGraph` with delay zero, the delayed graphs and the disconnected graphs. Every factor of the bundle adjustment is offered to every graph, each keeping only the factor groups up to its `maxGroupInGraph`, and every removal of the main graph is repeated in every graph (`marginalizeFrame`, `DelayedMarginalization.cpp:82-105`). The delayed graph that feeds the initialiser holds visual factors only, `BAIMULogic::NO_IMU_GROUP`, so its delayed prior is a visual prior (`PoseGraphBundleAdjustment.cpp:40-49`). `replaceMainGraph` swaps in a new main graph and notifies the bundle adjustment.

`DisconnectedDelayedGraph` (`DelayedMarginalization.h:97-111`) does no linear algebra at all. It records the factors offered and the key sets removed while a copy of a delayed graph is being optimised in another thread, so that the copy can be brought up to date afterwards. This is the device that lets the expensive work run outside the keyframe thread without missing what that thread does in the meantime.

`PoseGraphBundleAdjustment` (`IMUInitialization/PoseGraphBundleAdjustment.h:36-158`) clones the visual delayed graph (`prepareOptimization`), adds the active window as one linear factor and an inertial factor with a bias random walk factor between every pair of successive keyframes from the first pose still connected to the newest frame (`buildGraph`, `insertIMUFactorsAndValues`, `PoseGraphBundleAdjustment.cpp:210-331`), and optimises the result with GTSAM's Levenberg–Marquardt with first estimate values enabled for the visual factors (`optimize`, `:59-97`). Before handing the graph to the main system it adds the velocity and bias keys of every pose to the queued key sets, so that the readvance also removes the inertial variables (`updateGraphMarginalizationOrder`, `:425-461`), writes the optimised values into the evaluation point of every variable except the visual ones, and readvances to delay zero (`mainPreparation`, `:473-497`). `extendGraph` (`:108-208`) adds the keyframes and factors recorded by the disconnected graph while the optimisation ran and runs a short second optimisation for the new variables.

#### 2.2.2 The initialiser state machine

`IMUInitializer` (`IMUInitialization/IMUInitializer.cpp`) holds the current state behind a shared mutex, and every call from the bundle adjustment is forwarded to the state, which may return a new state. The default transition model is `COMBINED_REPLACEMENT_MODEL`, `transitionModel = 2` (`IMUInitSettings.h:93`, `IMUInitializerTransitions.h:68-72`), whose transitions are the following (`IMUInitializerTransitions.cpp:52-170`).

1. `RealtimeCoarseIMUInitState` runs the coarse optimisation in a detached thread on every keyframe once more than five frames are available, caches the poses that arrive meanwhile and replays them under the state lock when it finishes (`IMUInitializerStates.cpp:106-160`). A scale variance below `coarseScaleUncertaintyThresh` moves to the PGBA state.
2. `RealtimePGBAState::postBAInit` (`:307-337`) adds the keyframe, clones the delayed graph in the keyframe thread and starts the PGBA in a detached thread. While it runs, every new keyframe's preintegrated measurement is cached and the disconnected graph records the factors and removals. On failure the cache is replayed and the state returns to the coarse initialiser. On success the thread already readvances what it can (`preparePreparation`) and hands over to `InitializedFromRealtimePGBAState`.
3. `InitializedFromRealtimePGBAState::postBAInit` (`:450-462`), on the next keyframe in the keyframe thread, extends the graph with the cached keyframes and reoptimises the new variables. `initializeIfReady` (`:464-476`), at the end of that keyframe's operations, finishes the readvance to delay zero, writes the optimised values into the bundle adjustment through the initialisation callback (`BAIMULogic::initFromIMUInit`, `IMU/BAIMULogic.cpp:683-760`) and replaces the main graph.
4. While the scale variance exceeds `reinitScaleUncertaintyThresh` the model returns to the PGBA state for a reinitialisation, and afterwards it moves to `PotentialMarginalizationReplacementState` (`:529-576`).
5. `PotentialMarginalizationReplacementState::postBAInit` runs synchronously in the keyframe thread. If the scale ratio to the first estimate exceeds `threshScale` (or the gravity angle `threshGravdir`), it builds the PGBA graph without optimising it, `optimize(..., noOptimization = true)`, and moves to `MarginalizationReplacementReadyState`, whose `initializeIfReady` readvances to delay zero and replaces the main graph at the end of the keyframe's operations (`:590-600`).
6. The end of replacements. When a replacement is installed, the model stores the identifier of the inertial factor at position $n = \lfloor 0.5 \cdot d \rfloor$ of the readvanced graph (`takeOverLargeBAOptim` and `marginalizationReplacementReady`, `IMUInitializerTransitions.cpp:143-163`). Once the first pose still connected in a new graph is at or after that identifier, a replacement would drop at least half of the delay, and the model switches to `secondThresholdSettings`, whose `threshScale = 10^7` never fires (`IMUInitSettings.cpp:45`). In the code the rule is therefore a count, at least $0.5\,d$ removal steps since the last replacement, rather than the fraction of the previous prior's inertial factors described in the paper. This plan follows the code.

#### 2.2.3 Threads and the data handed between them

The keyframe thread calls, in order, the optimisation, `IMUIntegration::postOptimization` (which forwards to `postBAInit`), `finishKeyframeOptimization`, the removal of the keyframe's variables in every graph, and `finishKeyframeOperations` (which calls `initializeIfReady`) (`dso/FullSystem/FullSystem.cpp:1444-1454`, `IMU/IMUIntegration.cpp:126-139` and `:285-312`). The coarse initialisation and the PGBA run in detached threads that read a cloned graph and write their result into the state object, and the cache of keyframes plus the disconnected graph carry what happened in the keyframe thread meanwhile. The two operations that touch the main graph, the extension and the replacement, run in the keyframe thread, so the main graph needs no lock. The marginalisation replacement is cheap enough to run synchronously, since it skips the optimisation and only readvances.

#### 2.2.4 Flow chart

```
 keyframe thread                                         worker threads (detached)
 ───────────────                                         ─────────────────────────
 optimise window (main graph prior, delay 0)
   │
 postOptimization ── postBAInit(state) ─────────────┐
   │                                                │
   │   CoarseIMUInit:      every KF, if idle  ──────┼──▶ coarse optimisation (poses fixed),
   │                       cache poses while busy   │     cov(s) < θ_init ? ──▶ PGBA state
   │                                                │
   │   RealtimePGBA:       clone delayed graph ─────┼──▶ add IMU + bias factors from P_conn,
   │                       cache KFs, disconnected  │     LM optimise, cov(s) test,
   │                       graph records factors    │     readvance part way ──▶ Initialized
   │                                                │
   │   InitializedFromRT:  extendGraph(cached KFs) ◀┘
   │                       short reoptimisation
   │
   │   PotentialReplacement: δ_s > θ_s ? build graph, no optimisation ──▶ ReplacementReady
   │
 finishKeyframeOptimization (coarse tracker reference)
   │
 marginalizeFrame: main graph, every delayed graph (delay d), every disconnected graph (record only)
   │
 finishKeyframeOperations ── initializeIfReady(state)
       Initialized*:      readvance to delay 0, write values (initFromIMUInit), replaceMainGraph
       ReplacementReady:  readvance to delay 0, replaceMainGraph
       after ≥ 0.5 d steps without replacement: second threshold, no more replacements
```

#### 2.2.5 What the design implies for Basalt

Four features of DM-VIO's implementation carry over. The delayed graph repeats the main graph's removals in the same order, so its cost per keyframe equals one marginalisation. The expensive optimisation runs on a copy in another thread, and a record of what happened meanwhile is replayed in the main thread before the result is installed. The installation happens between two optimisations of the window, so the window never sees a half replaced prior. The replacement phase ends by itself once the scale has been stable for half the delay.

Three features do not carry over and §2.3.9 derives why. DM-VIO's main system has no inertial factors before initialisation, while Basalt's window is visual inertial from the first frame. DM-VIO thresholds the marginal variance of its scale, which is defined because its visual frame has a scale of its own, while the block of the period has identically zero marginal information under the gauge (plan 2 §3.2.7). DM-VIO keeps visual variables at their first estimates in a nonlinear factor graph, while Basalt has no nonlinear graph of the past at all, only a dense square root prior, so the delayed graph must be built from what Basalt's marginalisation already produces.

### 2.3 The Basalt formulation

This section derives delayed marginalisation for the estimator of the period as it is implemented, using the general results of §2.1 and the implementation choices of §2.2 where they transfer.

#### 2.3.1 The estimator of the period

The inertial factor between states 0 and 1 over $\Delta t$ has, in Basalt's row order, the rows (`residualGS`, `thirdparty/basalt-headers/include/basalt/imu/preintegration.h:298-386`)

$$\mathbf{r}_p = \mathbf{R}_0^\top \Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0 \Delta t) - \frac{1}{2}\mathbf{g}_D \Delta t^2\Big) - \hat{\Delta\mathbf{p}},$$

$$\mathbf{r}_R = \text{Log}\big(\text{Exp}(\mathbf{J}_{R,g}\,\Delta\mathbf{b}_g)\,\Delta\mathbf{R}\,\mathbf{R}_1^\top \mathbf{R}_0\big),$$

$$\mathbf{r}_v = \mathbf{R}_0^\top \big(s(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D \Delta t\big) - \hat{\Delta\mathbf{v}},$$

whitened by the square root information of the preintegration, followed by the bias random walk rows $\mathbf{W}_g (\mathbf{b}_{g,0} - \mathbf{b}_{g,1})$ and $\mathbf{W}_a (\mathbf{b}_{a,0} - \mathbf{b}_{a,1})$ (`ImuBlockGS::linearizeImu`, `include/basalt/linearization/imu_block.hpp:276-374`). The visual residual sees the scale only through the extrinsic lever arm and does not contain $\mathbf{g}_D$ (first plan §2.2). The three gauge directions of the block are removed by the projector $\mathbf{P} = \mathbf{I} - \mathbf{N}(\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top$ applied to the system and the step (`ComputeGaugeProjector`, `sqrt_keypoint_vio.cpp:2280-2322`). At the end of every `optimize`, `RecentreGravityScale` applies the correction $(\mathbf{Q}, s_c)$ with $\mathbf{Q}$ the minimal rotation carrying $\mathbf{d}$ onto $\hat{\mathbf{g}}$ and $s_c = e^\sigma$ to every state, linearisation point, prior and anchor, after which the block is the identity (`:2325-2376`). Each prior carries its own linearisation point of the block, an anchor $(\bar{\mathbf{d}}_a, \bar{\mathbf{B}}_a, \bar{\sigma}_a)$, which `AnchorGs` installs as the block's linearisation point before the prior is used (`:2379-2388`).

#### 2.3.2 Where the inconsistency of the prior lives after re-centring

The derivation shows that after re-centring, the only quantity that makes a Basalt prior inconsistent with the current scale is the anchor of its block, which is what a readvance must change.

Step 1. By (D5) the re-centring maps every prior, every frame state and every linearisation point by one similarity under which every residual is invariant. A prior's linear model is therefore the same function before and after the map, and no re-centring creates or removes an inconsistency.

Step 2. The anchor of a prior moves with the re-centring as $\bar{\sigma}_a \leftarrow \bar{\sigma}_a - \sigma_c$ and $\bar{\mathbf{d}}_a \leftarrow \mathbf{Q} \bar{\mathbf{d}}_a$ (`transform_anchor`, `:2356-2361`), while the block itself returns to $\sigma = 0$ and $\mathbf{d} = \hat{\mathbf{g}}$. After $n$ frames the offset of the block from the anchor of a prior built at frame $m$ is

$$\sigma - \bar{\sigma}_a = \sum_{i = m+1}^{m+n} \sigma_{c,i},$$

the cumulative log scale correction since the prior's anchor was set, and likewise for the tilt.

Step 3. The frozen linearisation points of the states move with the same similarity, so a velocity difference $\bar{\mathbf{u}}$ at a linearisation point becomes $s_c \mathbf{Q} \bar{\mathbf{u}}$ and the offsets $\Delta\mathbf{u}$ of the states from their linearisation points are carried in the same way.

Step 4. The inertial rows of the prior were linearised at the anchor's scale $e^{\bar{\sigma}_a}$ and the states' linearisation points. By (D8) with $\Delta\sigma = \sigma - \bar{\sigma}_a$ of Step 2, their error at the current value is $e^{\bar{\sigma}_a}((e^{\Delta\sigma} - 1 - \Delta\sigma)\,\bar{\mathbf{u}} + (e^{\Delta\sigma} - 1)\,\Delta\mathbf{u})$ in the coordinates of Step 3, which vanishes for every offset exactly when $\Delta\sigma = 0$.

Step 5. The tilt enters the inertial rows only through $\mathbf{g}_D = |\mathbf{g}|\,\text{Exp}(\bar{\mathbf{B}} \boldsymbol{\delta}_\theta)\,\bar{\mathbf{d}}$, whose linear model at the anchor errs by $\frac{1}{2}|\mathbf{g}|\,|\boldsymbol{\delta}_\theta|^2$ per unit of $\Delta t$ in $\mathbf{r}_v$, a second order term (plan 2 §3.3.1).

Step 6. It follows that relinearising the inertial factors of a prior at the current block, while keeping every state at its first estimate, removes the error of (D8) completely and the tilt error of Step 5 too. It leaves the first estimates of the states untouched, so the null spaces of the prior and of the window's factors on the same states still agree, as §2.1.4 requires. This is DM-VIO's choice as well, whose readvance keeps the visual variables at their first estimates and relinearises scale and gravity [2, §III-E] (`DelayedGraph::readvanceUntilDelay` inserts the evaluation values of visual variables, `DelayedMarginalization.cpp:212-253`).

#### 2.3.3 What a stored step holds, and why its visual rows may stay linear

A Basalt marginalisation removes landmarks by QR inside the linearisation and never keeps a nonlinear graph of the past, so the delayed store must hold what the marginalisation already computes. For step $j$ it holds the following.

1. The order $\mathcal{O}_j$ of the marginalisation, the block followed by every keyframe pose and every state up to `last_state_to_marg`, the column sets $\alpha_j$ kept and $\beta_j$ removed, and the order $\mathcal{O}'_j$ of the prior after the step.
2. The visual rows of the landmarks removed at the step, after landmark elimination, as one block of rows $[\mathbf{J}^V_j \,|\, \mathbf{r}^V_j]$ over $\mathcal{O}_j$, delta independent by (D4) and compressed by (D3) to at most one row per column.
3. A copy of every preintegrated inertial factor removed at the step.
4. For every coordinate that leaves the window at the step, its first estimate and offset, kept in a record that outlives the step until no stored row refers to it.
5. The epoch $e_j$ of the correction log at the step.

The visual rows are kept as linear rows at their first estimates, with their scale column set to zero, for three reasons.

Step 1. They cannot be relinearised, because the landmarks they eliminated are no longer in the landmark database once the step is done (`lmdb.removeKeyframes`, `sqrt_keypoint_vio.cpp:1501`). DM-VIO makes the same choice deliberately, "we do not update our estimates for point depths and do not relinearize photometric error terms" [2, §III-E].

Step 2. They do not depend on the scale for a monocular camera without lever arm. Basalt's visual residual sees a scale only through the relative translation $s\,\mathbf{R}_t^\top (\mathbf{p}_h - \mathbf{p}_t)$ between the IMU frames of host and target and through the lever arm $\mathbf{c}$ of the extrinsic, and its scale column satisfies $\partial\mathbf{r} / \partial\sigma = \rho\,\partial\mathbf{r} / \partial\rho - \rho\,\mathbf{J}_\pi \mathbf{c}$, so that without a lever arm the per landmark QR annihilates it (`context/vio_opt.md` §15.6). Setting the column to zero therefore drops only the lever arm's contribution, as $M_\text{visual}$ already does (`LinearizationGSAbsQR` with `visualScaleFree`, `linearization_abs_qr.cpp:355`).

Step 3. The function they represent is carried exactly through every later re-centring by (D5), so a step stored at epoch $e_j$ is mapped once, when it is read, by the composed similarity $\text{Between}(e_j, e)$. Under that map the position columns are divided by the cumulative scale. For rows that constrain bearings, which is what scale free visual rows do, this is the correct relinearisation and not an approximation, since the angular residual of a displacement $\Delta\mathbf{p}$ changes with a metric perturbation $\delta\mathbf{p}$ as $|\delta\mathbf{p}| / |\Delta\mathbf{p}|$, and scaling $\Delta\mathbf{p}$ by $s$ scales that derivative by $1/s$.

The visual rows are linearised at the current block, not at the main prior's anchor, so that the visual Jacobians are evaluated at the scale of the epoch in which they are stored. Plan 2 found that an anchor scale that drifts from the current one shrinks the visual translation information of $M_\text{visual}$ (plan 2 §5.5 item 6), and the current block avoids that by construction.

#### 2.3.4 The readvance in Basalt

The readvance of §2.1.6 needs one evaluation point $\hat{\mathbf{x}}$ for every variable, a way to bring rows of different origins to that point, and the recursion itself.

The evaluation point. Every state coordinate is evaluated at its first estimate for the Jacobians and at its current value for the residual, which is Basalt's convention for a frozen variable (`ImuBlock::linearizeImu` evaluates the Jacobians at `getStateLin()` and the residual at `getState()` once a state is frozen, `imu_block.hpp:28-79`). A coordinate still in the window takes its first estimate and offset from the window, and a coordinate that has left takes them from its record. The block is evaluated at its current value, with a fresh anchor equal to that value, so its offset is zero, by §2.3.2 Step 6.

Bringing rows to the evaluation point. Every stored row set is delta independent with respect to the first estimates of its variables and the anchor of its block. By (D4) its residual at the evaluation point is

$$\mathbf{r}_c = \mathbf{r}_0 + \mathbf{J} \boldsymbol{\delta},$$

where $\boldsymbol{\delta}$ stacks the offsets of the state coordinates from their first estimates and, at the block, the offset of the current block from the row set's anchor. That offset is the one `GravityScaleStateWithLin::Reanchor` computes, $\boldsymbol{\delta}_\theta = \bar{\mathbf{B}}_a^\top \mathbf{w}$ with $\mathbf{w}$ the rotation vector carrying $\bar{\mathbf{d}}_a$ onto $\mathbf{d}$, and $\delta\sigma = \sigma - \bar{\sigma}_a$ (`include/basalt/utils/imu_types.h:377-392`).

The recursion. Let $R_{j-1}$ be delta independent over $\mathcal{O}'_{j-1}$, which is a prefix of $\mathcal{O}_j$ with the same offsets, the invariant Basalt asserts for its own prior at every marginalisation (`sqrt_keypoint_vio.cpp:1099-1142`). Step 1. Stack the rows of $R_{j-1}$ at the evaluation point, the visual rows of step $j$ mapped to the current epoch by (D5) and brought to the evaluation point by (D4), and the inertial rows of step $j$ linearised by `ImuBlockGS` at the evaluation point, which are at the current value by construction. Step 2. Remove $\beta_j$ by (D3) with the helper of Basalt's own marginalisation. Step 3. The kept rows are at the evaluation point, and (D4) in reverse makes them delta independent over $\mathcal{O}'_j$, with the new anchor equal to the block at which the inertial rows were linearised. Step 4. By (D2), after the last step

$$R_k = \text{marg}_{\beta_{k-n+1} \cup \cdots \cup \beta_k}\Big(B + \sum_{j = k-n+1}^{k} \big(\mathcal{F}^V_j + \mathcal{F}^I_j(\hat{\mathbf{x}})\big)\Big),$$

with $B$ the delayed prior of §2.3.5, $n \le d$ the number of stored steps, $\mathcal{F}^V_j$ the linear visual rows and $\mathcal{F}^I_j(\hat{\mathbf{x}})$ the inertial factors at the evaluation point. Step 5. The kept variables of $R_k$ are evaluated at their window first estimates, so $R_k$ replaces the main prior without any further re-expression, and its anchor is the current block. The unit test `DelayedMargReadvanceJointTest` of §3.2 Change 10 checks Step 4 numerically against a joint removal of the same rows, to a relative $10^{-8}$.

#### 2.3.5 The delayed prior

DM-VIO's delayed prior holds visual factors only (§2.2.1). In Basalt the delayed prior $B$ is advanced like plan 2's $M_\text{visual}$. When a step leaves the delay, its visual rows and the scale free rows of its inertial factors are marginalised into $B$. The scale free rows are, by plan 2 §3.3, the rotation rows weighted by the marginal covariance of the rotation residual, the bias random walk rows, and the full rows with a zeroed scale column of every factor whose whitened scale column is at most `vio_gs_visual_prior_scale_tol`. Step 1. These rows hold the yaw pin, the accelerometer bias anchor and the ground tilt of admitted factors beyond the delay (plan 2 §3.3.6). Step 2. They depend on the tilt but not on the scale, so $B$ keeps a tilt anchor fixed at its first estimate and lets the anchor's scale follow the current value, exactly as plan 2 §3.3.5 does for $M_\text{visual}$. Step 3. By (D6), every readvanced prior holds at least the information of $B$ in every direction, because it is $B$ plus factors. With `vio_gs_visual_prior_inertial` off, $B$ receives the visual rows only, which is DM-VIO's delayed prior. The initial value of $B$ is the main prior at the first marginalisation of the period, the initial prior.

#### 2.3.6 Smoothing the delayed window, Basalt's pose graph bundle adjustment

DM-VIO initialises by optimising the delayed graph with its inertial factors before readvancing it (§2.1.8). Basalt's forward recursion of §2.3.4 already performs the elimination that such an optimisation needs, so one Gauss–Newton step on every departed variable costs one back substitution.

Step 1. Keep, at every step, the rows that eliminate $\beta_j$, $[\mathbf{R}^j_{\beta\beta}\ \mathbf{R}^j_{\beta\alpha} \,|\, \mathbf{c}^j]$ of (D3), which the helper otherwise discards. Step 2. By (D3) applied at every step, the joint cost of the stored rows at the evaluation point decomposes into a sum of the eliminated rows of every step and the final prior,

$$\frac{1}{2}\sum_{j} \big|\mathbf{R}^j_{\beta\beta} \mathbf{x}_{\beta_j} + \mathbf{R}^j_{\beta\alpha} \mathbf{x}_{\alpha_j} + \mathbf{c}^j\big|^2 + \frac{1}{2}\big|\mathbf{J}_k \mathbf{x}_{\alpha_k} + \mathbf{r}_k\big|^2 + \text{const}.$$

Step 3. Condition on the window. The variables kept at the end are the window's, already estimated by the window's own optimisation, so their increment from the evaluation point is zero, and the block's increment is zero because its evaluation point is its current value. Step 4. Solve backwards. At step $j$, every variable of $\alpha_j$ is either kept at the end or removed at a later step, whose increment the backward order has already produced, so (D7) gives $\mathbf{x}_{\beta_j}^* = -(\mathbf{R}^j_{\beta\beta})^{-1}(\mathbf{c}^j + \mathbf{R}^j_{\beta\alpha} \mathbf{x}_{\alpha_j})$, coordinate by coordinate from the last pivot to the first, with zero for a coordinate without a pivot. Step 5. By (D7) every eliminated term is then zero, which is its minimum, so the increments minimise the joint cost of the delayed window given the window. The test `DelayedMargSmoothingJointTest` checks this against the dense least squares solution to a relative $10^{-6}$.

Step 6. The increments are added to the offsets of the departed records, and the readvance is run again so that the installed prior is linearised at the smoothed history. The window then optimises against the new prior, and the next replacement smooths again. By (D9) neither half of this alternation raises the joint cost of the linearised problem, so it converges towards the joint optimum of window and delayed history that DM-VIO's PGBA reaches in one optimisation. The visual rows stay linear and the first estimates stay fixed, which matches the PGBA's treatment of photometric terms [2, §III-E]. The difference is that Basalt takes one step per replacement instead of iterating to convergence, which keeps the cost at two readvances.

#### 2.3.7 The replacement rule and its end

Trigger. A replacement is started when the block has moved from the main prior's anchor by more than $\ln\theta_s$ in log scale or by more than $\theta_g$ in tilt,

$$|\sigma - \bar{\sigma}_a| > \ln\theta_s,$$

$$\arccos(\mathbf{d}^\top \bar{\mathbf{d}}_a) > \theta_g,$$

evaluated after every marginalisation. The scale threshold is DM-VIO's, $\theta_s = 1.02$, with the error bound of §2.1.7 Step 4, about one standard deviation of a velocity row. DM-VIO disables its gravity threshold. The tilt threshold here bounds the error of §2.3.2 Step 5. At $\theta_g = 5° = 0.0873$ rad and $\Delta t = 0.054$ s it is $\frac{1}{2} \cdot 9.81 \cdot 0.0873^2 \cdot 0.054 = 0.0020$ m/s, 0.11 standard deviations of a velocity row with $\sigma_c = 0.08$ m/s²/√Hz, so a tilt that has moved by less than 5° does not need a replacement and one that has moved by more is caught before its error matters.

End of replacements. Replacements do not end. The store keeps the last $d$ steps for the whole period, so that a prior that drifts after a replacement can be readvanced again from the same history, and a readvance never empties it. A replacement at a time when the main prior still holds full rows older than the store drops the scale dependent rows of those steps and keeps their scale free rows in $B$, which is the price of a store of bounded length. DM-VIO stops replacing once a replacement would drop the scale dependent rows of at least $0.5\,d$ steps (§2.2.2 item 6), because its main system then optimises scale and gravity on its own. In Basalt the window's block alone would let the prior drift again, which the replays of 2026-11-10 showed after a release (§3.4.3 item 4). Note, 2026-11-13. This paragraph previously stopped replacements once $f - o \ge \theta_\text{lost}\,d$ with $\theta_\text{lost} = 0.5$ after the first replacement, where $o$ was the oldest step whose rows the main prior held in full and $f$ the oldest step in the store, then released the store, resumed the mapper's pose updates and optionally folded the period, through `vio_gs_dm_max_lost` and `vio_gs_dm_fold_on_settle`. Both keys are removed.

#### 2.3.8 Gauge, re-centring and the local mapper

Gauge. Every inertial row of a readvance is a `residualGS` row evaluated at one evaluation point, so by (D10) it satisfies $\mathbf{J} \mathbf{N} = \mathbf{0}$ with $\mathbf{N}$ built at the first estimates and the current block. The stored visual rows satisfy it except through their zeroed lever arm column, the same exception $M_\text{visual}$ has, which the projector of `optimize` removes (plan 2 §3.3.4 Step 4). The readvanced prior therefore enters the projected system as the main prior does.

Re-centring. The main prior and the delayed prior are mapped at every re-centring by (D5), the records of departed coordinates by the same similarity, and each stored step at the moment it is read through its epoch. A worker that readvances a snapshot in the epoch of the snapshot returns a prior that the main thread maps to the current epoch by the composed similarity before installing it, which is again (D5).

The local mapper. The mapper sends refined keyframe poses back, which `measure` applies by rebuilding the keyframe pose with the refined pose as its new first estimate and a zero offset (`sqrt_keypoint_vio.cpp:602-664`). The main prior was built at the old first estimate. It keeps evaluating its rows with a zero offset, that is at its own old first estimate, so it silently ignores the update (plan 2 §3.6 item 8). A readvance does not ignore it. It evaluates the stored inertial factors at the window's first estimates, which are now the mapper's poses. The mapper's bundle adjustment is visual only, so its poses carry no inertial information and need not agree with the preintegrated positions, whose standard deviation over one frame is about 0.5 mm (plan 2 §5.5 item 4). The replays of 2026-11-10 measured this conflict directly. In the ground phase the mapper moved keyframe poses by 1 mm to 18 mm per update, and a readvance at the main prior's own anchor, which must reproduce the main prior, reproduced its information to a relative $10^{-3}$ but carried a whitened cost of 1.7 to 7.2 at the current values against 0.0004 to 0.004 for the main prior (§3.4.3, run `2026-11-10_17-33-07`). Every replacement therefore installed a prior that pulled the window towards the mapper's geometry through the only free coordinates, the scale and the velocities, and the scale collapsed in every replay with replacements and pose updates both active (§3.4.3). With the pose updates held, the same binary installed one replacement in all but one replay, five in that one, and reached the scale ratios of §3.4.3. DM-VIO has no such feedback during its initialisation, since its main system is visual odometry until the PGBA succeeds. The plan therefore holds the mapper's pose updates while the delayed store is active, `vio_gs_dm_hold_pose_updates`, which is the whole period since the store is kept (§2.3.7). Note, 2026-11-13. This sentence previously let the updates resume once replacements stopped. The held updates are discarded, not queued, because the mapper sends the full set of its keyframe poses at every cycle (`local_mapper.cpp:322-324`).

#### 2.3.9 The inertial initialisation of the take off in Basalt

The period has two phases, and in both the estimator publishes metric poses from the first frame.

1. Coarse phase. The store starts at the first marginalisation, its delayed prior $B$ is the initial prior with its position, yaw, gravity and bias rows, and every step is kept (§2.3.5). The block is held in the window, with its three rows and columns masked in every solve, no gauge projection and no Change B, so the window is Basalt's own estimator at the initial gravity direction and unit scale. At every new keyframe once more than five keyframes are known, as DM-VIO runs its coarse initialiser [9], a coarse inertial optimisation (§2.3.11) runs in the worker on a snapshot of the store and the window. It estimates the gravity direction, the scale, one bias of both sensors and the keyframe velocities with every pose held, and the same job then readvances the prior at the corrected values, after a smoothing pass when the scale was estimated. The VIO thread installs the result at the next marginalisation by the re-centring of §2.3.1 with the estimated block, adds the velocity increments and sets the bias of every state and record, and installs the readvanced prior in the same frame, so that the prior agrees with the correction (§2.3.14). Before lift off the scale carries almost no information, so the gate of §2.3.11 Step 6 holds it and only the tilt, the bias and the velocities are corrected, which DM-VIO, whose main system is visual only until it initialises, does not do. Note, 2026-11-14. This item previously ran the coarse optimisation every $k$ = `vio_gs_dm_coarse_every` marginalisations once the stationary detector reported motion, with common bias increments and no smoothing, and before that it corrected the gravity direction, the log scale and the accelerometer bias alone, followed by a separate readvance one or two frames later.
2. Delayed phase. Once the store has first held $d$ steps, 10.8 s after the first marginalisation at $d = 200$ and 54 ms per frame, the block is freed in the window with the gauge projection and Change B, and a smoothed readvance (§2.3.6) starts at once in the role of DM-VIO's pose graph bundle adjustment. From then on the replacement rule of §2.3.7 readvances the prior whenever the block has moved by $\theta_s$ in scale or $\theta_g$ in tilt from the prior's anchor.

The coarse phase stands in for DM-VIO's coarse initialisation. DM-VIO's poses have an arbitrary scale until initialisation, so its coarse stage holds the inertial system back until the variance of the log scale falls below $\theta_\text{init}$ [2, §III-F]. Basalt's window is metric from the first frame, so the coarse result corrects the running estimator, which is ORB-SLAM's order of operations [7, §IV] [8]. DM-VIO's gate on the marginal variance of the log scale transfers to the coarse problem, in which every pose is held and the scale is observable, and it is applied there with DM-VIO's default (§2.3.11 Step 6). In the delayed phase the block has identically zero marginal information under the gauge (plan 2 §3.2.7), so no such variance exists and Change B remains the soft gate. DM-VIO's reinitialisation while the variance exceeds $\theta_\text{reinit}$ is subsumed by the repeated coarse corrections and by the replacement rule. In the replays of 2026-11-13 the coarse phase installed 13 or 14 corrections and ended at 10.4 s to 10.7 s of the run, and the delayed phase installed 29 to 49 readvances over three minutes of flight (§3.4.5).

Note, 2026-11-13. This section previously described a single phase in which the block was free in the window from the first frame, the store filled through the ground phase, one replacement at lift off settled the take off and replacements stopped at about 66 s in the replays of `telemetry_40`, and it stated that DM-VIO's coarse initialisation does not transfer because Basalt's window solve already plays its role. The coarse phase replaced it because the block optimised in the window from the first frame diverged in tilt in the live runs while the runs that held the tilt did not (§2.3.12), and because the take off needs a metric estimate from its first second.

#### 2.3.10 Latency, throughput and configuration

Measured baseline. In the replay `_23-21-18` of plan 2 the camera period is 54 ms, `measure` takes 6.77 ms at the median, 8.51 ms at the 90th percentile and 11.2 ms at most, `optimize` 5.76 ms at the median, and `marginalize` 0.40 ms at the median and 0.86 ms at the 90th percentile, of which the three square root QRs take 0.14 ms (`vio_timing.csv`, `vio_solver_summary.csv`, `vio_marg_timing.csv`). One marginalisation removes 15 coordinates from 75 and keeps 60 (`vio_marg.csv`).

Cost model. One readvance step is a QR of the stacked rows of the previous prior (at most 60), the compressed visual rows (at most 75) and 15 inertial rows per factor, over at most 78 columns. A micro benchmark of `marginalizeHelperSqrtToSqrt` with 15 removed columns measured 0.051 ms for 135 rows by 78 columns and 0.065 ms for 171 by 78 (§3.4.2). A readvance of $d$ steps therefore costs about $d \times 0.06$ ms, 12 ms at $d = 200$, and with smoothing about twice that. The replays measured the readvance of a full store at 9 ms to 13.6 ms without smoothing (`DMDIAG`, run `2026-11-10_17-33-07`), and installed jobs at 3.8 ms to 11 ms without smoothing and 13.2 ms to 28.6 ms with smoothing, depending on how full the store was (§3.4.3).

Per frame cost. Storing a step adds one visual only landmark linearisation and a QR compression, and advancing the delayed prior adds one marginalisation of a small system. The delayed mode drops the linearisations and QRs of $M_\text{half}$ and $M_\text{visual}$. Measured in the same batches, `marginalize` took 0.29 ms to 0.38 ms at the median with the delayed mode against 0.53 ms to 0.62 ms with dynamic marginalisation, and the store's own share was 0.012 ms at the median, 0.13 ms at the 90th percentile and at most 0.8 ms (§3.4.3).

Synchronous against asynchronous. A synchronous readvance adds the whole job to the frame in which it fires. At $d = 200$ with smoothing that is up to about 30 ms, which stays below the 54 ms period, so no frame is dropped, but it raises the latency of the published state on that frame from about 7 ms to about 37 ms, and during a transient that fires every frame it would do so on every frame. The asynchronous design of §2.2.3 moves the job to a worker that reads a snapshot. The main thread only installs the result and readvances the steps stored while the job ran, which was one step and 0.04 ms to 0.2 ms in every replay. The latency of the published state is then unchanged when the job ends within a frame. Every job is installed at the marginalisation that follows the one that started it, and the VIO thread waits there for a job that has not finished, so that every correction takes effect one frame after its snapshot and the `wait` column of `vio_delayed_marg` records any delay this adds. The coarse optimisation stops iterating after a quarter of the frame period and then installs nothing (§2.3.11 Step 7), which leaves the rest of the period to the readvance. The plan runs asynchronously by default, with `vio_gs_dm_async` set to false for deterministic debugging, which is DM-VIO's `multithreadedInitDespiteNonRT` in reverse. Note, 2026-11-14. This paragraph previously let the prior lag the trigger by one or two frames, installing a job at the first marginalisation after it had finished.

Delay. $d = 200$ steps of 54 ms, 10.8 s, holds the whole ground phase of the live runs and the take off transient, which §2.1.9 requires. The memory is bounded by $d$ steps of at most $78 \times 79$ values of visual rows and a few inertial factors, about 10 MB at $d = 200$, and the replays held 200 steps and at most 207 records. A larger delay buys nothing for the take off and lengthens every job linearly.

The configuration that follows is the one of `data/sitl_config_vo.json` in Changes 9 and 15, namely $d = 200$, $\theta_s = 1.02$, $\theta_g = 5°$, coarse optimisations at every new keyframe with a scale variance gate of 1.0, smoothing on, asynchronous, pose updates held, a gravity prior of 0.75°, and the period kept permanent through `vio_gs_duration_s` = 7000 s. Note, 2026-11-13. This paragraph previously listed $\theta_\text{lost} = 0.5$ and folding on settle, which the revision removed, and left `vio_gs_duration_s` at the working value, which was 1 s from lift off on 2026-11-13 and would end the period inside the coarse phase. Note, 2026-11-14. This paragraph previously listed coarse optimisations every $k = 10$ marginalisations with nodes 10 frames apart and an accelerometer bias intensity of 0.05 m/s²/√s, whose keys `vio_gs_dm_coarse_every` and `vio_ba_step_prior_std` Change 17 removed.

#### 2.3.11 The coarse inertial optimisation

The coarse optimisation is the coarse IMU initialiser of DM-VIO, `CoarseIMUInitOptimizer` [2, §III-F] [9], transferred to Basalt so that the two can be compared on equal terms. It holds the poses of the last 100 keyframes, joins consecutive keyframes by one preintegrated inertial factor, and estimates the gravity direction, the scale, one gyroscope and accelerometer bias shared by every state and the metric velocity of every keyframe by Levenberg–Marquardt, with the scale applied to the held positions as DM-VIO applies it to the visual poses. Three choices depart from DM-VIO. The held positions carry an explicit noise in the weighting (Step 4), the scale enters through a relative increment in which every row is linear (Step 5), and the gravity prior is 0.75° rather than DM-VIO's 0.4 rad (§2.3.12). The result is applied to the job's copy of the departed records and the window states, and the same job then readvances the prior at the corrected values, with a smoothing pass when the scale was estimated (Step 7, §2.3.14). Note, 2026-11-14. This section previously described Change 16, whose nodes were 10 frames apart, whose factors were chained through every held frame rotation, whose scale multiplied the inertial side, whose biases were common increments with an accelerometer bias prior, and whose gate read the information at the snapshot inflated by an a posteriori variance factor. Change 17 replaced it to match DM-VIO, at a measured cost in accuracy on jittered poses (Step 8).

Notation. $\hat{\Delta\mathbf{R}}_f$, $\hat{\Delta\mathbf{p}}_f$, $\hat{\Delta\mathbf{v}}_f$ and $\Delta t_f$ are the preintegrated rotation, position and velocity and the duration of a per frame inertial factor $f$ at zero bias, expressed in the body frame at its start, $\mathbf{R}_a$ and $\mathbf{p}_a$ the held rotation and position of keyframe $a$ in the estimator's units, $\mathbf{u}_a$ its metric velocity, $s$ the scale with metric positions $s\,\mathbf{p}$, and $\mathbf{b} = (\mathbf{b}_g, \mathbf{b}_a)$ the one bias. Perturbations of a rotation are on the left, $\text{Exp}(\boldsymbol{\phi})\hat{\Delta\mathbf{R}}$, as in Basalt's preintegration.

Step 1, a factor at zero bias. `residualGS` at identity states, zero gravity, zero scale and zero bias returns $-\hat{\Delta\mathbf{p}}_f$ and $-\hat{\Delta\mathbf{v}}_f$ in its position and velocity rows, $\text{Log}(\hat{\Delta\mathbf{R}}_f)$ in its rotation row, and the negated bias Jacobians of the deltas, except that its rotation row's gyroscope Jacobian carries the inverse left Jacobian of $\text{Log}(\hat{\Delta\mathbf{R}}_f)$, which is removed by multiplying with the left Jacobian. The factor's covariance $\boldsymbol{\Sigma}_f$ is ordered [position, rotation, velocity].

Step 2, composition between keyframes. For consecutive keyframes $a$ and $b$ the factors from $a$ to $b$ are composed in time order. With $\hat{\Delta\mathbf{R}}$, $\hat{\Delta\mathbf{p}}$ and $\hat{\Delta\mathbf{v}}$ the composed deltas up to the start of factor $f$,

$$\hat{\Delta\mathbf{p}} \leftarrow \hat{\Delta\mathbf{p}} + \hat{\Delta\mathbf{v}}\,\Delta t_f + \hat{\Delta\mathbf{R}}\,\hat{\Delta\mathbf{p}}_f,$$

$$\hat{\Delta\mathbf{v}} \leftarrow \hat{\Delta\mathbf{v}} + \hat{\Delta\mathbf{R}}\,\hat{\Delta\mathbf{v}}_f,$$

$$\hat{\Delta\mathbf{R}} \leftarrow \hat{\Delta\mathbf{R}}\,\hat{\Delta\mathbf{R}}_f,$$

which is the preintegration over $[a, b]$ by the definition of the per frame deltas. Write the composed errors as $\mathbf{e} = (\delta\mathbf{p}, \boldsymbol{\phi}, \delta\mathbf{v})$ with $\text{Exp}(\boldsymbol{\phi})\hat{\Delta\mathbf{R}}$ the true rotation. Substituting the perturbed deltas into the three updates and keeping first order terms, with $[\cdot]_\times$ the skew matrix and $\text{Exp}(\boldsymbol{\phi})\mathbf{x} \approx \mathbf{x} - [\mathbf{x}]_\times \boldsymbol{\phi}$, gives

$$\mathbf{e} \leftarrow \mathbf{A}_f \mathbf{e} + \mathbf{B}_f \mathbf{e}_f,$$

where $\mathbf{A}_f$ is the identity with $-[\hat{\Delta\mathbf{R}}\,\hat{\Delta\mathbf{p}}_f]_\times$ in block (1, 2), $\Delta t_f \mathbf{I}$ in block (1, 3) and $-[\hat{\Delta\mathbf{R}}\,\hat{\Delta\mathbf{v}}_f]_\times$ in block (3, 2), and $\mathbf{B}_f$ is block diagonal with $\hat{\Delta\mathbf{R}}$ three times. The covariance and both bias Jacobians follow the same map,

$$\boldsymbol{\Sigma}_{ab} \leftarrow \mathbf{A}_f \boldsymbol{\Sigma}_{ab} \mathbf{A}_f^\top + \mathbf{B}_f \boldsymbol{\Sigma}_f \mathbf{B}_f^\top,$$

$$\mathbf{J}_{ab} \leftarrow \mathbf{A}_f \mathbf{J}_{ab} + \mathbf{B}_f \mathbf{J}_f.$$

This is the factor DM-VIO preintegrates between keyframes with `onlyKFs` [9]. As in DM-VIO, which integrates the inertial data of every frame into the factor that ends at the next keyframe and adds a pose only at a keyframe, the composition is done once, as the per frame factors arrive. The store composes every factor it receives into `DelayedMargKfChain`, which keeps the closed factors of the last 99 keyframe intervals and an open one from the latest keyframe, and a job extends a copy of it by the factors still in the window. A job therefore reads only keyframe factors, and the optimisation works over keyframe intervals of several frames, never over a single frame, whatever the length of the store. Only the rotations of the keyframes enter the residual of Step 3. The rotations of the frames between them are used only to propagate the velocities in Step 7, through the per frame factors each keyframe factor keeps.

Step 3, the residual of GTSAM's `ImuFactor` with the scale on the held positions. With the bias correction $\mathbf{c} = \mathbf{J}_{ab}\,\mathbf{b}$ split into its position, rotation and velocity parts,

$$\mathbf{r}_p = \mathbf{R}_a^\top\Big(s(\mathbf{p}_b - \mathbf{p}_a) - \mathbf{u}_a T_{ab} - \frac{1}{2}\mathbf{g}_D T_{ab}^2\Big) - (\hat{\Delta\mathbf{p}} + \mathbf{c}_p),$$

$$\mathbf{r}_R = \text{Log}\big(\text{Exp}(\mathbf{c}_R)\,\hat{\Delta\mathbf{R}}\,\mathbf{R}_b^\top \mathbf{R}_a\big),$$

$$\mathbf{r}_v = \mathbf{R}_a^\top(\mathbf{u}_b - \mathbf{u}_a - \mathbf{g}_D T_{ab}) - (\hat{\Delta\mathbf{v}} + \mathbf{c}_v),$$

which are Basalt's `residualGS` rows, and GTSAM's `ImuFactor` rows up to the sign convention of the rotation, after DM-VIO's `PoseTransformationFactor` has scaled the visual positions [9].

Step 4, the weighting. The residuals are whitened by the lower Cholesky factor of

$$\boldsymbol{\Sigma}_{ab} + \text{diag}(2 s_0^2 \sigma_x^2 \mathbf{I}, \mathbf{0}, \mathbf{0}),$$

with $s_0$ the scale of the snapshot and $\sigma_x$ = `kCoarsePositionStd` = 0.02 m the noise of one held keyframe position, entering twice through $\mathbf{p}_a$ and $\mathbf{p}_b$. DM-VIO weights by $\boldsymbol{\Sigma}_{ab}$ alone, because its keyframe poses come from DSO's bundle adjustment. Basalt's departed keyframes sit about 5 cm from their neighbours (§3.5 item 10), so without this term the position rows would claim a precision of about 2 cm per keyframe interval that the held positions do not have.

Step 5, unknowns, priors and the solver. The unknowns are the tilt $\boldsymbol{\delta}_\theta$, the relative scale increment $\eta$ with

$$s = s_0(1 + \eta),$$

the bias $\mathbf{b}$, starting at zero as in DM-VIO, and one metric velocity per keyframe, starting at $s_0$ times its current value. Every row is linear in $\eta$, in the velocities and, through $\mathbf{c}$, in the bias, so only the tilt and the rotation row are nonlinear. The Jacobians of $\mathbf{r}_p$ are $\frac{1}{2}T_{ab}^2 \mathbf{R}_a^\top [\mathbf{g}_D]_\times \mathbf{B}$ in the tilt, from $\partial\mathbf{g}_D / \partial\boldsymbol{\delta}_\theta = -[\mathbf{g}_D]_\times \mathbf{B}$, $s_0 \mathbf{R}_a^\top(\mathbf{p}_b - \mathbf{p}_a)$ in $\eta$, the negated position rows of $\mathbf{J}_{ab}$ in the bias and $-T_{ab}\mathbf{R}_a^\top$ in $\mathbf{u}_a$. Those of $\mathbf{r}_v$ are $T_{ab} \mathbf{R}_a^\top [\mathbf{g}_D]_\times \mathbf{B}$ in the tilt, the negated velocity rows of $\mathbf{J}_{ab}$ in the bias, $-\mathbf{R}_a^\top$ in $\mathbf{u}_a$ and $\mathbf{R}_a^\top$ in $\mathbf{u}_b$. That of $\mathbf{r}_R$ in the gyroscope bias is the inverse left Jacobian of $\mathbf{r}_R$ times the rotation rows of $\mathbf{J}_{ab}$. The tilt prior is that of the previous form, $\sqrt{w_g}\,\mathbf{B}^\top \mathbf{w}$ with $w_g = 1 / \sigma_g^2$ for $\sigma_g$ = `vio_gravity_prior_std_deg` towards the initial gravity direction mapped into the snapshot's epoch, and the bias has no prior, as in DM-VIO [9]. A keyframe velocity meets only its two neighbours, so the normal equations are accumulated as $15 \times 15$ blocks into a sparse matrix and factorised by `SimplicialLDLT`, which keeps an iteration near linear in the number of keyframes. Levenberg–Marquardt runs at most 100 iterations, GTSAM's default, with $\lambda$ starting at $10^{-4}$ times the diagonal, divided by 10 after an accepted step and multiplied by 10 after a rejected one, and stops when the relative decrease of the cost falls below $10^{-6}$.

Step 6, the scale gate. After the solve with the scale free, the marginal variance of the log scale at the solution is

$$\text{var}(\ln s) = \frac{[\mathbf{H}^{-1}]_{\eta\eta}}{(1 + \eta)^2},$$

with $\mathbf{H}$ the information matrix at the solution, read by one solve of the factorised $\mathbf{H}$ against the unit vector of $\eta$. If it exceeds $\theta_\text{init}$ = `vio_gs_dm_coarse_scale_var`, DM-VIO's `coarseScaleUncertaintyThresh` with its default of 1.0, the solve is repeated from the snapshot with the scale row and column masked, and only the tilt, the bias and the velocities are estimated. DM-VIO then leaves its main system as it is, whereas here the tilt, bias and velocity corrections are still installed (§2.3.9 item 1).

Step 7, application, readvance and the deadline. The velocity of every keyframe is $\mathbf{u}/s$ in the estimator's units, and that of every state between two keyframes, or after the last one, is propagated from the preceding keyframe through the per frame factors at the estimated bias and the held frame rotations,

$$\mathbf{v}_{f+1} = \mathbf{v}_f + \frac{1}{s}\big(\mathbf{g}_D \Delta t_f + \mathbf{R}_f(\hat{\Delta\mathbf{v}}_f + \mathbf{c}_{v,f})\big).$$

The job adds the velocity increments to its copies of the records and window states, sets every state's bias to $\mathbf{b}$, moves its block to the estimated direction and log scale, and readvances the prior from $B$ through every stored step. When the gate let the scale free, the readvance is preceded by the smoothing pass of §2.3.6, which takes the place of DM-VIO's pose graph bundle adjustment after a successful coarse initialisation [2, §III-E]. The optimisation stops iterating once a quarter of the frame period has passed, and a job stopped this way installs nothing, so that the readvance after it still ends before the next frame. The VIO thread installs every job at the next marginalisation, waiting for it if it has not finished (§2.3.10).

Step 8, the cost of matching DM-VIO. A Monte Carlo of 2026-11-14 over 30 random trajectories of 200 frames at 54 ms, with the SITL noise densities, the positions and velocities shrunk by 1.6 and white jitter on the held positions, measured the error of the log scale. With exact positions every form recovered the scale to $10^{-4}$. With jitter the present form underestimates the scale, because the scale multiplies the jittered positions, the errors in variables bias that §3.5 item 11 first met.

| Keyframe spacing | Jitter | Present form, mean | Present form, rms | Change 16 form, rms at 10 frames |
|---|---|---|---|---|
| 10 frames | 1 cm | −0.014 | 0.015 | |
| 10 frames | 2 cm | −0.052 | 0.056 | 0.009 |
| 10 frames | 5 cm | −0.279 | 0.298 | 0.021 |
| 5 frames | 2 cm | −0.149 | 0.158 | |
| 2 frames | 2 cm | −0.475 | 0.495 | |

The bias grows as the keyframes come closer, since the displacement between them shrinks against the jitter, and the scale variance stays near $5 \times 10^{-4}$ to $1.4 \times 10^{-3}$, so the gate passes the biased scale. The per frame noise term of Step 4 changes the weighting but cannot remove this bias. DM-VIO avoids it through poses whose noise is small against its keyframe spacing, not through its formulation. At rest the gate held the scale in every trial. A job took 0.6 ms to 3.6 ms while moving and up to 23 ms at rest, where the flat scale direction makes the solver use most of its iterations, and the deadline of Step 7 bounds that case.

The forms this section replaced were rejected on these grounds, in order.

1. Every per frame factor with the poses and velocities held and the scale on the held positions. At rest a factor's residual was about $s$ times the 5 cm jitter of a departed keyframe against an inertial side near zero, so the cost fell monotonically as $s$ fell, to $e^{-20}$, and the jitter curved the cost in $s$ so that the gate did not catch it (§3.5 item 10).
2. The elimination of the velocities over three nodes after Mur-Artal and Tardós [7, eq. 12], $s\,\boldsymbol{\lambda} = \kappa\,\mathbf{g}_D + \boldsymbol{\gamma}$ with $\boldsymbol{\lambda} = T_{ab}(\mathbf{p}_c - \mathbf{p}_b) - T_{bc}(\mathbf{p}_b - \mathbf{p}_a)$, $\kappa = \frac{1}{2} T_{ab} T_{bc}(T_{ab} + T_{bc})$ and $\boldsymbol{\gamma} = T_{ab} \mathbf{P}_{bc} - T_{bc} \mathbf{P}_{ab} + T_{ab} T_{bc} \mathbf{V}_{ab}$, solved for the tilt, the log scale and the accelerometer bias with the residual $\mathbf{r} = \frac{s_0}{\kappa}(\boldsymbol{\lambda} - e^{-\sigma}(\kappa\,\mathbf{g}_D + \boldsymbol{\gamma}))$ whitened by `kCoarseAccelStd` = 0.1 m/s² and only for triples whose visual displacement exceeded `kCoarseMinDisplacement` = 0.2 m. With $\boldsymbol{\lambda}$ as the regressor the scale attenuated geometrically over corrections (§3.5 item 11), which is why the visual side was made the predicted side. The overlapping triples counted every span two or three times, its velocities stayed frozen, and its variance was 4 to 27 times too small in flight (§5.3 item 1).
3. Change 16, which estimated the node velocities and common bias increments with nodes every 10 frames, chained the factors through every held frame rotation, put the inverse scale on the inertial side so that the held positions were never multiplied by an unknown, added an accelerometer bias prior, and gated on the information at the snapshot inflated by the a posteriori variance factor. It was unbiased on jittered positions, with a root mean square log scale error of 0.0086 at 2 cm and 0.0213 at 5 cm in the trials above, and it was replaced only so that the coarse phase reproduces DM-VIO for comparison. Should the comparison favour it, its code is recorded in Change 16.

#### 2.3.12 The gravity direction

The live runs of 2026-11-11 and 2026-11-12 with the block free in the window reached tilt errors of 8.8° to 23.8° after lift off, while the run `2026-11-13_15-49-25`, whose period folded 1 s after lift off so that the tilt stayed at its initial estimate, stayed within 2.6° (evaluated from `gt_eval.csv` as the angle between the body frame gravity directions of estimate and ground truth). The initial direction from the accelerometer is therefore close enough, and the period must keep the gravity direction near it rather than free it. Three mechanisms do so. First, the gravity prior `vio_gravity_prior_std_deg` of `plans/gravity_and_bias_step_priors.md` adds the rows $\sqrt{w_g}(\delta\boldsymbol{\varphi}_{0,xy} - \boldsymbol{\delta}_\theta)$ to the initial prior, which pin the first state's tilt relative to the block. The initial prior is the initial value of $B$ (§2.3.5), its rows enter $B$ through the scale free rotation rows when the first steps leave the store, and by (D6) every readvanced prior holds them. Second, the coarse optimisation carries the same prior on the block (§2.3.11 Step 5). Third, Change B bounds the tilt step per frame in the delayed phase. The replays of the revision stayed within 2.1° of tilt error on both flights (§3.4.5).

#### 2.3.13 What the revision removed and kept

Delayed marginalisation is the only mechanism of the period, and every earlier mechanism for the scale and gravity was either absorbed into it or removed.

| Mechanism | Origin | Fate | Reason |
|---|---|---|---|
| Three priors $M_\text{curr}$, $M_\text{half}$, $M_\text{visual}$, the swap rule, `vio_gs_dmin` | plan 1 §2.6, plan 2 | removed | replaced by one prior and the delayed store, §1.2 |
| `vio_gs_delayed_marg` | Change 1 | removed | the delayed store is the period's only mode |
| Scale free inertial rows of Change A, `vio_gs_visual_prior_inertial`, `vio_gs_visual_prior_scale_tol` | plan 2 | kept | they advance the delayed prior $B$, §2.3.5 |
| Random walk on the block step, Change B | plan 2 | kept in the delayed phase | the soft gate of the block in the window, §2.3.9 |
| Tilt hold, `vio_gs_optimise_gravity_dir` | `plans/gravity_scale_tilt_optional.md` | removed | the gravity prior constrains the tilt softly, §2.3.12 |
| Update cadence, `vio_gs_update_every_n_frames` | `plans/gravity_scale_update_cadence.md` | removed | the coarse phase holds the block and corrects it every $k$ steps |
| Accelerometer bias step prior in the window | `plans/gravity_and_bias_step_priors.md` | removed from the window, and `vio_ba_step_prior_std` removed on 2026-11-14 | DM-VIO's coarse initialiser has no bias prior, §2.3.11 Step 5 |
| Coarse cadence and node spacing, `vio_gs_dm_coarse_every` | Change 12 | removed on 2026-11-14 | the coarse optimisation runs at every new keyframe with the keyframes as nodes, after DM-VIO |
| Gravity prior, `vio_gravity_prior_std_deg` | `plans/gravity_and_bias_step_priors.md` | kept | it lives in $B$ and in the coarse optimisation |
| Period timer from motion | `plans/gravity_scale_timer_from_motion.md` | kept | it starts the coarse optimisations and times the fold |
| End of replacements, `vio_gs_dm_max_lost`, `vio_gs_dm_fold_on_settle` | Change 1 | removed | the store is kept, §2.3.7 |
| Velocity elimination over three nodes, `kCoarseAccelStd`, `kCoarseMinDisplacement` | Change 13 | removed on 2026-11-14 | the velocities and both biases are estimated after DM-VIO, §2.3.11 and §2.3.14 |

#### 2.3.14 Re-estimating the departed velocities and biases, and the readvance that follows

The coarse optimisation of §2.3.11 moves the velocities and biases of states that have left the window, and of the oldest window states, whose first estimates the main prior was linearised at. The question is whether the main prior is still correct afterwards, and the answer rests on three facts.

1. In the coarse phase the store holds every step since the first marginalisation. The phase ends when the store first holds $d$ steps (§2.3.9), and `AdvanceBoundary` moves a step into $B$ only once more than $d$ are stored, so throughout the coarse phase $B$ is the initial prior. A readvance from $B$ through every stored step therefore recomputes the whole main prior, not a part of it, and no older linearisation survives in it.
2. The stored visual rows are landmark eliminated, delta independent and touch only poses (§2.3.3), and the coarse optimisation holds every pose. They are therefore unaffected by the new velocities and biases and remain exact at their first estimates, the position the plan already takes for the visual information.
3. The inertial rows are relinearised by the readvance at the current value of every state, its first estimate plus its offset (§2.3.4). The coarse increments enter as offsets, because every record is frozen (§3.5 item 2) and every window state in the main prior is linearised, so no first estimate moves. The `last_state_to_marg` of the window and the other states in the main prior keep their first estimates. The readvanced prior is expressed in delta independent form about those same first estimates and evaluated at their new offsets, exactly as for a readvance after a scale change.

The prior installed after a coarse optimisation is therefore the prior that Basalt would hold had every inertial factor since the start been linearised at the corrected velocities, biases, gravity direction and scale, with the visual factors at their first estimates. The initial prior's velocity and bias rows on the first state are evaluated at its new values, so they act as the priors they are. The readvance runs in the same job as the coarse optimisation, at the job's corrected copies of the values, and is installed in the same frame as the correction. In the form this section replaces, a separate readvance followed one or two frames later, and the window ran those frames with corrected states against the old prior.

The literature supports the direction. DM-VIO's coarse initialiser optimises the velocities, one bias of both sensors, the gravity direction and the scale with the poses fixed [2, §III-F] [9], and the result of its pose graph bundle adjustment reaches the main system by readvancing the delayed graph, "successively marginalizing all the variables which have been marginalized in the main graph" [2, §III-E]. Campos et al. showed that the inertial only maximum a posteriori estimate with the keyframe velocities and both biases as unknowns, every residual weighted by its preintegration covariance, beats initialisations that ignore the inertial noise. It reached a mean scale error of 5.29 % against 5.98 % for the joint method it compares with, at 3.75 s against 13.54 s, and 9.09 % against 22.05 % for VINS-Mono on trajectories of about 1.25 s [10, Table I]. The authors attribute the gain to the weighting and to solving all inertial parameters in one step, and say they chose to estimate the velocities because doing so "eases the initialization process" [10, §II-B], not that the velocities themselves improve the accuracy. Zuñiga-Noël et al. eliminate the velocities over three keyframes with covariance weighted residuals, after estimating the gyroscope bias from the rotations, and with ground truth poses reach a lower scale error than Campos et al., 1.11 % against 2.05 % over 18.75 s, but a worse gravity direction over short windows, 7.6° against 1.11° at 1.25 s [11]. Elimination is therefore not worse for the scale when it is weighted, while explicit velocities are better for the gravity direction and the biases over short windows, and only explicit velocities give the readvance values to relinearise at. The triple form here was weighted by one constant, and because every span entered two or three overlapping triples whitened as independent, its inertial terms and node positions were counted several times, which is consistent with its scale variance being 4 to 27 times too small in flight (§5.3 item 1). With explicit velocities each span enters once, and the $6(N - 1)$ equations of $N - 1$ spans against $3N$ velocities leave the same $3N - 6$ constraints as the triples, with their correct correlation. The Monte Carlo comparison of §3.4.6 measured the same effect here. Over a store of 200 frames the present form halved the root mean square scale error of the triple form, from 2.9 % to 1.3 % at a held position jitter of 4 cm, and halved the tilt and accelerometer bias errors.

The gyroscope bias is estimated as DM-VIO estimates it, through the rotation rows of factors that span whole keyframe intervals (§2.3.11 Step 3). Inside such an interval the rotation is integrated with the gyroscope bias, so the bias also moves the composed $\hat{\Delta\mathbf{p}}$ and $\hat{\Delta\mathbf{v}}$ through the skew terms of Step 2, and it couples to the translation more strongly than when every factor was rotated by its own held frame rotation, where an error of 0.01 rad/s changed a frame's $\hat{\Delta\mathbf{v}}$ by only about $\frac{1}{2} \cdot 9.81 \cdot 0.01 \cdot 0.054^2 = 1.4 \times 10^{-4}$ m/s. Its estimate lets the readvance relinearise every inertial factor at a gyroscope bias that agrees with the held rotations. Note, 2026-11-14. This paragraph previously described the per frame rotations of Change 16, under which the gyroscope bias was estimated for consistency rather than for the scale.

What the coarse optimisation still does not do is move a pose. DM-VIO's pose graph bundle adjustment and ORB-SLAM3's visual inertial bundle adjustments also refine the poses. Here that role falls to the smoothing pass that opens the delayed phase (§2.3.6), and §5.8 lists what remains to be tuned.

#### 2.3.15 Why the inertial factors are chained between nodes

DM-VIO and ORB-SLAM3 have no spans because their inertial factors already join keyframes. DM-VIO connects its last 100 keyframes with inertial factors [2, §III-F], each preintegrated over the interval between two keyframes, and ORB-SLAM3 forces keyframes at 4 Hz to 10 Hz during its initialisation so that each preintegration is short [10, §II-A]. Basalt's store holds one inertial factor per frame, between consecutive frame states 54 ms apart, and one pose per frame (§2.3.3). A span is the keyframe preintegration of DM-VIO rebuilt from those per frame factors. Step 2 of §2.3.11 proves that the chained quantities equal the preintegration over the span, so a span changes the grouping of the measurements, not the model. With $k$ = 10 frames the nodes are 0.54 s apart, close to ORB-SLAM3's slowest keyframe rate.

The per frame collapse that motivated the spans belonged to the form, not to the grouping. In the first form the scale multiplied the held positions and the velocities were held, so at rest a factor's residual was about $s$ times the 5 cm jitter of a departed keyframe against an inertial side near zero, and the cost fell monotonically as $s$ fell, to $e^{-20}$ (§3.5 item 10). The jitter also curved the cost in $s$, so the variance looked small and the gate did not catch it. In the present form the scale multiplies only the inertial side and the velocities are free, so a span at rest is flat in the scale and the gate holds it. A Monte Carlo of 2026-11-14 over 30 trajectories of 200 frames confirmed this. Per frame nodes, $k$ = 1, gave a root mean square log scale error of 0.0040 at 2 cm and 0.0101 at 5 cm of white jitter, against 0.0086 and 0.0213 at $k$ = 10, and the scale was held in every static trial at every $k$. The spans are kept for two other reasons. A job at $k$ = 1 took 317 ms against 0.8 ms at $k$ = 10, because the dense system grows from about 70 to 609 unknowns. And nodes a keyframe interval apart are what DM-VIO's factors join. The synthetic jitter was white and independent per frame, while the real jitter is concentrated at departed keyframes, so whether $k$ = 1 would also win on flight data is open. The velocities of the states between nodes are not lost with spans, since §2.3.11 Step 7 propagates them from the node velocities through the per frame factors, so the readvance still relinearises every per frame factor at consistent values. Note, 2026-11-14. This paragraph previously stated that the grouping was needed because the first form, which used every per frame factor, collapsed the scale, and that spans keep the real motion larger than the jitter. The collapse came from the scale multiplying the held positions, which the present form avoids.

No later work motivated the elimination over three nodes that the previous form added on top of the spans. It came from Mur-Artal and Tardós [7], which predates DM-VIO, and it was chosen because the departed velocities were then held at their frozen departure values and a three node elimination needs no velocity. Once the velocities became unknowns, as in DM-VIO [2] and ORB-SLAM3 [8] [10], the elimination had no remaining purpose and was removed, while the spans stayed as the keyframe spacing that DM-VIO's factors have by construction.

With Change 17 the nodes are Basalt's keyframes, as DM-VIO's are its keyframes with `onlyKFs` [9], and the factor between two of them is composed from the per frame factors without the held rotations of the frames between them (§2.3.11 Step 2). The spacing of the nodes is therefore set by Basalt's keyframe policy, and the Monte Carlo of §2.3.11 Step 8 shows that with the scale on the held positions a close spacing biases the scale, since the displacement between keyframes shrinks against the jitter. Note, 2026-11-14. The paragraphs above describe the spans of Change 16, a node every $k$ frames, and are kept for the measurements that justify chaining at all.

## 3. Implementation Plan

### 3.1 Architecture

#### 3.1.1 Current, in the period with Changes A and B

```
 VIO thread (controller driver)                                  local mapper thread
 ──────────────────────────────                                  ───────────────────
 measure
   ├─ apply mapper pose updates (rewrite frame_poses first estimates) ◀── QueuePoseUpdates
   ├─ predict, associate, triangulate
   ├─ optimize
   │    aom = [gs(3) | poses(6)… | states(15)…]
   │    LinearizationGSAbsQR(M_curr), LM loop with gauge projection and random walk (B)
   │    RecentreGravityScale: Φ_c on states, M_curr, M_half, M_visual, three anchors
   ├─ marginalize
   │    UpdateScaleInterval: swap rule (M_curr ← M_half ← M_visual)
   │    LinearizeMargGS: M_curr full, M_half full, M_visual scale free (A)
   │    MarginalizeGS: three square root QRs
   │    out_marg_queue packet from M_curr ─────────────────────────────▶ MargData
   │    RecentrePriorsGS, LogGravityScale, fold at vio_gs_duration_s
   └─ PublishKeyframe ──────────────────────────────────────────────────▶ Keyframe
```

#### 3.1.2 Updated, the period with delayed marginalisation only

```
 VIO thread                                                    worker (std::async, one at a time)
 ──────────                                                    ─────────────────────────────────
 measure
   ├─ mapper pose updates: dropped for the whole period (hold)
   ├─ optimize
   │    coarse phase: block rows and columns masked, no projection, no Change B
   │    delayed phase: gauge projection and Change B, as in §3.1.1
   │    RecentreGravityScale also maps the delayed prior B and the records
   ├─ marginalize
   │    store B ← M_curr at the first marginalisation
   │    LinearizeMargGS: M_curr full (anchor A_curr) + visual rows at the current block
   │    MarginalizeGS: one square root QR
   │    capture step j: order, kept and removed columns, compressed visual rows,
   │                    inertial factors, records of departing coordinates, epoch
   │    RecentrePriorsGS
   │    UpdateDelayedMarg
   │      push step j, advance B while more than d steps (scale free rows of A)
   │      job ready?   ── coarse: set the block, RecentreGravityScale, add δb_a,
   │                              request a readvance
   │                     readvance: map R to now, apply smoothing increments,
   │                                readvance steps stored meanwhile, M_curr ← R
   │      d steps reached once? ── leave the coarse phase, request a smoothed readvance
   │      readvance requested, or (delayed phase and |σ − σ_a| > ln θ_s or tilt > θ_g)?
   │                    ── snapshot (B, steps, records, window, block, epoch) ─────▶ Run(): forward readvance,
   │                                                                                 backward smoothing in the
   │                                                                                 delayed phase, forward
   │      else coarse phase, moving, k steps since the last coarse?
   │                    ── snapshot (+ window factors, initial gravity, priors) ───▶ Run(): Coarse(), three node
   │                                                                                 Gauss–Newton in tilt, log
   │                                                                                 scale and accelerometer bias
   │    LogGravityScale (event 5 coarse installed, 6 readvance installed), fold at vio_gs_duration_s
   └─ PublishKeyframe
```

With `vio_gs_enable` false, every new branch is skipped and the estimator is Basalt's. Note, 2026-11-13. This diagram previously showed the delayed mode as an opt in beside dynamic marginalisation, with the block free from the first frame and a settle branch that released the store, all of which the revision replaced.

#### 3.1.3 Class relations and threads

`DelayedMarginalisation<Scalar>` (new, `include/basalt/vi_estimator/delayed_marginalisation.h`) owns the delayed prior $B$, a deque of immutable `DelayedMargStep` objects held by `std::shared_ptr<const ...>`, and the records of departed coordinates. `DelayedMargJob<Scalar>` holds a snapshot and runs the readvance and the smoothing on it without touching the estimator, so a worker can run it while the VIO thread continues. Its static `MarginaliseStep` is the one recursion step of §2.3.4, shared by the worker, the catch up in the VIO thread and the advance of $B$. `SqrtKeypointVioEstimator` owns the store and the future of the running job, captures each step in `marginalize`, and installs results. `MargHelper::marginalizeHelperSqrtToSqrt` gains optional outputs for the eliminated rows that the smoothing needs. `VioConfig` gains nine keys and `Logger` one record.

Thread safety follows from three rules. A stored step is never modified after it is pushed, and the lazily cached whitening of each preintegrated factor is computed before the step is shared (`IntegratedImuMeasurement::get_sqrt_cov_inv` writes a mutable cache on first use, `preintegration.h:405-411`). The worker reads only its snapshot and the correction log, whose `Between` takes the log's mutex. Only the VIO thread touches the main prior, the store and the records, and the installation happens between two calls of `optimize`, as in DM-VIO (§2.2.3). At most one job runs at a time, and `FoldGravityScale` and the destructor wait for it.

### 3.2 Implementation

Changes 1 to 11 are unified diffs against the working tree as it stood on 2026-11-10, taken from the scratch copy that was built and tested in §3.4.2 and §3.4.3. Changes 12 to 15 revise them on 2026-11-13 and give the final code of every function they touch, copied from the working tree. Where a hunk of Changes 1, 2 or 5 to 9 differs from the tree, the later change holds the code as implemented, and the hunk is kept as the record of the first design. Hunks are grouped by change, and every change states its backwards compatibility.

#### Change 1. Configuration keys, `include/basalt/utils/vio_config.h` and `src/utils/vio_config.cpp`

Nine keys, all read by name through cereal (`context/vio_config_json.md`). Their defaults leave the delayed mode off. `vio_gs_dm_*` are read only when `vio_gs_delayed_marg` is set, and that is read only when `vio_gs_enable` is set.

| Key | Default | Meaning | Section |
|---|---|---|---|
| `vio_gs_delayed_marg` | false | one prior and the delayed store instead of three priors | §2.3 |
| `vio_gs_dm_delay` | 200 | delay $d$ in marginalisation steps | §2.3.10 |
| `vio_gs_dm_thresh_scale` | 1.02 | $\theta_s$ | §2.3.7 |
| `vio_gs_dm_thresh_tilt_deg` | 5.0 | $\theta_g$ in degrees | §2.3.7 |
| `vio_gs_dm_max_lost` | 0.5 | $\theta_\text{lost}$ | §2.3.7 |
| `vio_gs_dm_smooth` | true | backward smoothing before the installed readvance | §2.3.6 |
| `vio_gs_dm_async` | true | readvance in a worker | §2.3.10 |
| `vio_gs_dm_fold_on_settle` | false | end the period when replacements stop | §2.3.7 |
| `vio_gs_dm_hold_pose_updates` | false | drop mapper pose updates while the store is active | §2.3.8 |

Note, 2026-11-13. Change 12 removed `vio_gs_delayed_marg`, `vio_gs_dm_max_lost` and `vio_gs_dm_fold_on_settle` from this table and added `vio_gs_dm_coarse_every` (default 5) and `vio_gs_dm_coarse_scale_var` (default 1.0), so the period now has eight `vio_gs_dm_*` keys and no opt in.

```diff
--- a/include/basalt/utils/vio_config.h
+++ b/include/basalt/utils/vio_config.h
@@ -142,6 +142,23 @@
     // Change B.
     double vio_gs_jump_sigma_rw;
     double vio_gs_jump_tilt_rw_deg;
+    // Keep one prior and a delayed store of the last vio_gs_dm_delay
+    // marginalisations, readvanced at the current block when the block moves
+    // past either threshold, instead of the three priors of dynamic
+    // marginalisation, plans/delayed_marginalisation.md.
+    bool vio_gs_delayed_marg;
+    int vio_gs_dm_delay;
+    double vio_gs_dm_thresh_scale;
+    double vio_gs_dm_thresh_tilt_deg;
+    // Fraction of the delay whose scale dependent rows a replacement may
+    // drop. Replacements stop once it is reached after the first one.
+    double vio_gs_dm_max_lost;
+    bool vio_gs_dm_smooth;
+    bool vio_gs_dm_async;
+    bool vio_gs_dm_fold_on_settle;
+    // Drop the local mapper's pose updates while the delayed store is
+    // active, plans/delayed_marginalisation.md §2.3.8.
+    bool vio_gs_dm_hold_pose_updates;
 
     bool vio_marg_lost_landmarks;
     double vio_kf_marg_feature_ratio;
```

```diff
--- a/src/utils/vio_config.cpp
+++ b/src/utils/vio_config.cpp
@@ -105,6 +105,15 @@
   vio_gs_visual_prior_scale_tol = 1.0;
   vio_gs_jump_sigma_rw = 0.0;
   vio_gs_jump_tilt_rw_deg = 0.0;
+  vio_gs_delayed_marg = false;
+  vio_gs_dm_delay = 200;
+  vio_gs_dm_thresh_scale = 1.02;
+  vio_gs_dm_thresh_tilt_deg = 5.0;
+  vio_gs_dm_max_lost = 0.5;
+  vio_gs_dm_smooth = true;
+  vio_gs_dm_async = true;
+  vio_gs_dm_fold_on_settle = false;
+  vio_gs_dm_hold_pose_updates = false;
 
   vio_marg_lost_landmarks = true;
 
@@ -247,6 +256,15 @@
   ar(CEREAL_NVP(config.vio_gs_visual_prior_scale_tol));
   ar(CEREAL_NVP(config.vio_gs_jump_sigma_rw));
   ar(CEREAL_NVP(config.vio_gs_jump_tilt_rw_deg));
+  ar(CEREAL_NVP(config.vio_gs_delayed_marg));
+  ar(CEREAL_NVP(config.vio_gs_dm_delay));
+  ar(CEREAL_NVP(config.vio_gs_dm_thresh_scale));
+  ar(CEREAL_NVP(config.vio_gs_dm_thresh_tilt_deg));
+  ar(CEREAL_NVP(config.vio_gs_dm_max_lost));
+  ar(CEREAL_NVP(config.vio_gs_dm_smooth));
+  ar(CEREAL_NVP(config.vio_gs_dm_async));
+  ar(CEREAL_NVP(config.vio_gs_dm_fold_on_settle));
+  ar(CEREAL_NVP(config.vio_gs_dm_hold_pose_updates));
 
   ar(CEREAL_NVP(config.vio_marg_lost_landmarks));
   ar(CEREAL_NVP(config.vio_kf_marg_feature_ratio));
```

Backwards compatibility. A missing key is fatal in cereal's JSON input, so every configuration file gains the keys in Change 9. With `vio_gs_delayed_marg` false the keys are read and stored only.

#### Change 2. The `vio_delayed_marg` record, `include/basalt/utils/logger.h` and `src/utils/logger.cpp`

One row per marginalisation in the delayed mode while the store is active. `steps` and `records` are the store's size, `started`, `installed` and `settled` the events of the frame, `anchor_sigma` and `anchor_tilt_deg` the trigger quantities after any installation, `job` the worker's duration of an installed job, `catchup_steps` and `catchup` the readvance in the VIO thread at installation, `max_pos_inc` the largest position increment of the smoothing, `store` the VIO thread's cost of capturing, pushing and advancing, and `lost_steps` the $f - o$ of §2.3.7.

```diff
--- a/include/basalt/utils/logger.h
+++ b/include/basalt/utils/logger.h
@@ -190,6 +190,12 @@
                          double maxTransErr, double maxRotErr,
                          double limitTrans, double limitRot);
     void PrintVioPoseUpdate();
+    void AddVioDelayedMarg(int64_t tNs, int steps, int records, int started,
+                           int installed, int settled, double anchorSigma,
+                           double anchorTiltDeg, double jobSeconds,
+                           int catchupSteps, double catchupSeconds,
+                           double maxPosInc, double storeSeconds,
+                           int lostSteps);
     void AddVioGravityScale(int64_t tNs, int64_t epoch, double sigmaStep,
                             double tiltStepDeg, double scaleCumulative,
                             double tiltCumulativeDeg, int swapEvent,
```

```diff
--- a/src/utils/logger.cpp
+++ b/src/utils/logger.cpp
@@ -378,6 +378,15 @@
               {"jump_cost", "f"}, {"visual_imu_full", "count"},
               {"visual_imu_rot", "count"}, {"visual_imu_scale_col", "f"}});
 
+    Register("vio_delayed_marg", "[VIO][delayed-marg]",
+             {{"t_ns", "ns"}, {"steps", "count"}, {"records", "count"},
+              {"started", "count"}, {"installed", "count"},
+              {"settled", "count"}, {"anchor_sigma", "f"},
+              {"anchor_tilt_deg", "f"}, {"job", "ms"},
+              {"catchup_steps", "count"}, {"catchup", "ms"},
+              {"max_pos_inc", "f"}, {"store", "ms"},
+              {"lost_steps", "count"}});
+
     Register("vio_assoc", "[VIO][assoc]",
              {{"t_ns", "ns"}, {"of_obs", "count"}, {"connected0", "count"},
               {"unconnected0", "count"}, {"connected_ratio", "ratio"},
@@ -852,6 +861,31 @@
     s.CommitRow();
 }
 
+void Logger::AddVioDelayedMarg(int64_t tNs, int steps, int records,
+                               int started, int installed, int settled,
+                               double anchorSigma, double anchorTiltDeg,
+                               double jobSeconds, int catchupSteps,
+                               double catchupSeconds, double maxPosInc,
+                               double storeSeconds, int lostSteps) {
+    if (!mpEnabled) return;
+    AggregateStats& s = mpAggregates.at("vio_delayed_marg");
+    s.add_int("t_ns", tNs);
+    s.add("steps", double(steps));
+    s.add("records", double(records));
+    s.add("started", double(started));
+    s.add("installed", double(installed));
+    s.add("settled", double(settled));
+    s.add("anchor_sigma", anchorSigma);
+    s.add("anchor_tilt_deg", anchorTiltDeg);
+    s.add("job", jobSeconds);
+    s.add("catchup_steps", double(catchupSteps));
+    s.add("catchup", catchupSeconds);
+    s.add("max_pos_inc", maxPosInc);
+    s.add("store", storeSeconds);
+    s.add("lost_steps", double(lostSteps));
+    s.CommitRow();
+}
+
 void Logger::AddVioAssoc(int64_t tNs, int ofObs, int connected0,
                          int unconnected0, double thresh, int framesAfterKf,
                          int lmdbLandmarks, int kfs, int frameStates,
```

Backwards compatibility. A new record, registered like every other and so written to `vio_delayed_marg.csv` with a header in every run, empty outside the delayed mode. No existing record changes, so `scripts/analysis` and every reader of `vio_gravity_scale.csv` are unaffected. The logger flushes every `kFlushPeriod` rows (`logger.cpp:200`), so the last rows of a run stopped by a signal, the settle row included, can be missing from every record, which §3.4.3 had to account for.

#### Change 3. Eliminated rows from the square root helper, `include/basalt/vi_estimator/marg_helper.h` and `src/vi_estimator/marg_helper.cpp`

The smoothing of §2.3.6 needs the rows $[\mathbf{R}_{\beta\beta}\ \mathbf{R}_{\beta\alpha} \,|\, \mathbf{c}]$ that the helper computes and discards, and the row of each removed column's pivot. Three optional pointer arguments return them. The Householder loop is unchanged except for one guarded store of the pivot row.

```diff
--- a/include/basalt/vi_estimator/marg_helper.h
+++ b/include/basalt/vi_estimator/marg_helper.h
@@ -37,6 +37,7 @@
 
 #include <Eigen/Dense>
 #include <set>
+#include <vector>
 
 namespace basalt {
 
@@ -59,10 +60,14 @@
                                         const std::set<int>& idx_to_marg,
                                         MatX& marg_sqrt_H, VecX& marg_sqrt_b);
 
-  // Modifies Q2Jp and Q2r as a side effect.
-  static void marginalizeHelperSqrtToSqrt(MatX& Q2Jp, VecX& Q2r,
-                                          const std::set<int>& idx_to_keep,
-                                          const std::set<int>& idx_to_marg,
-                                          MatX& marg_sqrt_H, VecX& marg_sqrt_b);
+  // Modifies Q2Jp and Q2r as a side effect. When elim_J is given, it also
+  // returns the rows that eliminate the marginalised columns, in the column
+  // order marginalised then kept, with the row of each marginalised column's
+  // pivot in elim_pivot_row, or -1 for a column without a pivot.
+  static void marginalizeHelperSqrtToSqrt(
+      MatX& Q2Jp, VecX& Q2r, const std::set<int>& idx_to_keep,
+      const std::set<int>& idx_to_marg, MatX& marg_sqrt_H, VecX& marg_sqrt_b,
+      MatX* elim_J = nullptr, VecX* elim_r = nullptr,
+      std::vector<Eigen::Index>* elim_pivot_row = nullptr);
 };
 }  // namespace basalt
```

```diff
--- a/src/vi_estimator/marg_helper.cpp
+++ b/src/vi_estimator/marg_helper.cpp
@@ -257,7 +257,8 @@
 template <class Scalar_>
 void MargHelper<Scalar_>::marginalizeHelperSqrtToSqrt(
     MatX& Q2Jp, VecX& Q2r, const std::set<int>& idx_to_keep,
-    const std::set<int>& idx_to_marg, MatX& marg_sqrt_H, VecX& marg_sqrt_b) {
+    const std::set<int>& idx_to_marg, MatX& marg_sqrt_H, VecX& marg_sqrt_b,
+    MatX* elim_J, VecX* elim_r, std::vector<Eigen::Index>* elim_pivot_row) {
   Eigen::Index keep_size = idx_to_keep.size();
   Eigen::Index marg_size = idx_to_marg.size();
 
@@ -293,6 +294,8 @@
   Eigen::Index marg_rank = 0;
   Eigen::Index total_rank = 0;
 
+  if (elim_pivot_row) elim_pivot_row->assign(marg_size, -1);
+
   {
     const Scalar rank_threshold =
         std::sqrt(std::numeric_limits<Scalar>::epsilon());
@@ -314,6 +317,8 @@
 
       if (std::abs(beta) > rank_threshold) {
         Q2Jp.coeffRef(total_rank, k) = beta;
+        if (elim_pivot_row && k < marg_size)
+          (*elim_pivot_row)[k] = total_rank;
 
         Q2Jp.bottomRightCorner(remainingRows, remainingCols)
             .applyHouseholderOnTheLeft(Q2Jp.col(k).tail(remainingRows - 1),
@@ -342,6 +347,9 @@
   marg_sqrt_H = Q2Jp.block(marg_rank, marg_size, keep_valid_rows, keep_size);
   marg_sqrt_b = Q2r.segment(marg_rank, keep_valid_rows);
 
+  if (elim_J) *elim_J = Q2Jp.topRows(marg_rank);
+  if (elim_r) *elim_r = Q2r.head(marg_rank);
+
   Q2Jp.resize(0, 0);
   Q2r.resize(0);
 }
```

Backwards compatibility. The callers are `marginalize` (`sqrt_keypoint_vio.cpp:1429`, `:1456`), `MarginalizeGS` (`:2518`), `NfrMapper` and the tests `test_qr`, `test_linearization` and `test_nfr`, which all pass six arguments and get the defaults, whose branches are all skipped. The double instantiation is unconditional because `NfrMapper` depends on it (`marg_helper.cpp:352-354`), which is unchanged.

#### Change 4. The delayed store, `include/basalt/vi_estimator/delayed_marginalisation.h` (new)

The whole file. `DelayedMargValues::State` and `Delta` implement the evaluation point of §2.3.4, `DelayedMargJob::MarginaliseStep` the recursion of §2.3.4, `Forward` and `Backward` the readvance and the smoothing of §2.3.6, and `DelayedMarginalisation` the store, the records and the advance of the delayed prior of §2.3.5.

```cpp
#pragma once

#include <basalt/linearization/imu_block.hpp>
#include <basalt/utils/imu_types.h>
#include <basalt/vi_estimator/marg_helper.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

// Delayed marginalisation of the gravity and scale period after DM-VIO,
// plans/delayed_marginalisation.md.

namespace basalt {

// Linearisation point of the gravity and scale block of a stored prior.
template <class Scalar_>
struct DelayedMargAnchor {
    using Scalar = Scalar_;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat32 = Eigen::Matrix<Scalar, 3, 2>;

    Vec3 mDir = Vec3(0, 0, -1);
    Mat32 mBasis = (Mat32() << 1, 0, 0, 1, 0, 0).finished();
    Scalar mSigma = 0;

    void Set(const GravityScaleStateWithLin<Scalar>& gs) {
        mDir = gs.GetDir();
        mBasis = gs.GetBasis();
        mSigma = gs.GetSigma();
    }

    void Transform(const Sophus::SO3<Scalar>& Q, Scalar sigmaShift) {
        mDir = Q * mDir;
        mBasis = Q.matrix() * mBasis;
        mSigma -= sigmaShift;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Square root prior 0.5 |H (delta + x) + b|^2 in the offsets delta of its
// variables from their linearisation points, with its block anchor.
template <class Scalar_>
struct DelayedMargPrior {
    using Scalar = Scalar_;
    using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    MatX mH;
    VecX mB;
    AbsOrderMap mOrder;
    DelayedMargAnchor<Scalar> mAnchor;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// One marginalisation of the window. Immutable once stored, so that a
// readvance in another thread can share it.
template <class Scalar_>
struct DelayedMargStep {
    using Scalar = Scalar_;
    using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    int64_t mIndex = 0;
    int64_t mEpoch = 0;
    AbsOrderMap mOrder;
    AbsOrderMap mOrderNew;
    std::set<int> mIdxKeep;
    std::set<int> mIdxMarg;
    // Landmark eliminated visual rows with a zero block column, in the
    // epoch mEpoch, delta independent.
    MatX mVisJ;
    VecX mVisR;
    Eigen::aligned_vector<IntegratedImuMeasurement<Scalar>> mvImu;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Value of a variable after some of its coordinates left the window, with
// its first estimate and offset. mLastStep is the step at which its last
// coordinates left.
template <class Scalar_>
struct DelayedMargRecord {
    PoseVelBiasStateWithLin<Scalar_> mState;
    int64_t mLastStep = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// First estimates and offsets of every variable a readvance can meet,
// expressed in one epoch.
template <class Scalar_>
struct DelayedMargValues {
    using Scalar = Scalar_;
    using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    Eigen::aligned_map<int64_t, DelayedMargRecord<Scalar>> mvRecords;
    Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<Scalar>> mvWindowStates;
    Eigen::aligned_map<int64_t, PoseStateWithLin<Scalar>> mvWindowPoses;

    // Full state of a variable, its pose taken from the window while the
    // pose is still there.
    PoseVelBiasStateWithLin<Scalar> State(int64_t id) const {
        const auto it_s = mvWindowStates.find(id);
        if (it_s != mvWindowStates.end()) return it_s->second;

        const PoseVelBiasStateWithLin<Scalar>& rec = mvRecords.at(id).mState;
        const auto it_p = mvWindowPoses.find(id);
        if (it_p == mvWindowPoses.end()) return rec;

        const PoseVelBiasState<Scalar>& lin = rec.getStateLin();
        PoseVelBiasStateWithLin<Scalar> st(id, it_p->second.getPoseLin(),
                                           lin.vel_w_i, lin.bias_gyro,
                                           lin.bias_accel, true);
        typename PoseVelBiasStateWithLin<Scalar>::VecN delta = rec.getDelta();
        delta.template head<POSE_SIZE>() = it_p->second.getDelta();
        st.applyInc(delta);
        return st;
    }

    // Offsets of the variables of an order from their first estimates, with
    // blockDelta at the block.
    VecX Delta(const AbsOrderMap& order, const Vec3& blockDelta) const {
        VecX delta = VecX::Zero(order.total_size);
        for (const auto& [id, idx] : order.abs_order_map) {
            if (id == kGravityScaleStateId) {
                delta.template segment<GRAVITY_SCALE_SIZE>(idx.first) =
                    blockDelta;
            } else if (idx.second == POSE_SIZE) {
                const auto it_p = mvWindowPoses.find(id);
                delta.template segment<POSE_SIZE>(idx.first) =
                    it_p != mvWindowPoses.end()
                        ? it_p->second.getDelta()
                        : mvRecords.at(id)
                              .mState.getDelta()
                              .template head<POSE_SIZE>();
            } else {
                delta.template segment<POSE_VEL_BIAS_SIZE>(idx.first) =
                    State(id).getDelta();
            }
        }
        return delta;
    }

    void TransformBySim3(const Sophus::SO3<Scalar>& Q, Scalar s) {
        for (auto& [id, rec] : mvRecords) rec.mState.TransformBySim3(Q, s);
        for (auto& [id, st] : mvWindowStates) st.TransformBySim3(Q, s);
        for (auto& [id, ps] : mvWindowPoses) ps.TransformBySim3(Q, s);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Columns of a delta independent prior after the re-centring (Q, s), as
// RecentreGravityScale maps marg_data, plans/gravity_scale_optimisation.md
// §2.7.3.
template <class Scalar, class Derived>
void DelayedMargTransformColumns(Eigen::MatrixBase<Derived>& H,
                                 const AbsOrderMap& order,
                                 const Sophus::SO3<Scalar>& Q, Scalar s) {
    const Eigen::Matrix<Scalar, 3, 3> Q_inv_m = Q.inverse().matrix();
    for (const auto& [id, idx] : order.abs_order_map) {
        if (id == kGravityScaleStateId) continue;
        const int col = idx.first;
        H.middleCols(col, 3) = H.middleCols(col, 3) * Q_inv_m / s;
        H.middleCols(col + 3, 3) = H.middleCols(col + 3, 3) * Q_inv_m;
        if (idx.second == POSE_VEL_BIAS_SIZE) {
            H.middleCols(col + 6, 3) = H.middleCols(col + 6, 3) * Q_inv_m / s;
        }
    }
}

// Everything one readvance needs, copied so that it can run in a worker.
template <class Scalar_>
struct DelayedMargJob {
    using Scalar = Scalar_;
    using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Vec15 = Eigen::Matrix<Scalar, POSE_VEL_BIAS_SIZE, 1>;
    using StepPtr = std::shared_ptr<const DelayedMargStep<Scalar>>;

    struct Elim {
        MatX mJ;
        VecX mR;
        std::vector<Eigen::Index> mPivotRow;
    };

    // Inputs, all in the epoch mEpoch.
    DelayedMargPrior<Scalar> mBoundary;
    std::vector<StepPtr> mvSteps;
    DelayedMargValues<Scalar> mValues;
    GravityScaleStateWithLin<Scalar> mGs;
    Vec3 mG = Vec3::Zero();
    Vec3 mGyroBiasWeightSqrt = Vec3::Zero();
    Vec3 mAccelBiasWeightSqrt = Vec3::Zero();
    GravityScaleCorrectionLog::Ptr mLog;
    int64_t mEpoch = 0;
    bool mSmooth = false;

    // Outputs.
    DelayedMargPrior<Scalar> mResult;
    // Smoothing increment of each departed variable, and which of its 15
    // coordinates it covers.
    Eigen::aligned_map<int64_t, std::pair<Vec15, uint32_t>> mvSmoothInc;
    Scalar mMaxPosInc = 0;
    double mSeconds = 0;

    int64_t FirstStep() const { return mvSteps.front()->mIndex; }
    int64_t LastStep() const { return mvSteps.back()->mIndex; }

    // Readvance, preceded by one smoothing pass when mSmooth is set.
    void Run() {
        const auto t0 = std::chrono::steady_clock::now();
        if (mSmooth) {
            std::vector<Elim> elims;
            Forward(&elims);
            Backward(elims);
        }
        Forward(nullptr);
        mSeconds = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - t0)
                       .count();
    }

    // Marginalises every step into the boundary in order, with the inertial
    // factors linearised at the current block, plans/delayed_marginalisation.md
    // §2.3.4.
    void Forward(std::vector<Elim>* elims) {
        GravityScaleStateWithLin<Scalar> gsNow = mGs;
        gsNow.Reanchor(mGs.GetDir(), mGs.GetBasis(), mGs.GetSigma());

        mResult = mBoundary;
        if (elims) elims->resize(mvSteps.size());
        for (size_t i = 0; i < mvSteps.size(); i++) {
            MarginaliseStep(*mvSteps[i], mValues, gsNow, mG,
                            mGyroBiasWeightSqrt, mAccelBiasWeightSqrt,
                            mLog.get(), mEpoch, false, Scalar(0), mResult,
                            elims ? &(*elims)[i] : nullptr);
        }
    }

    // Solves every departed variable given the window, by back substitution
    // through the eliminated rows, §2.3.6.
    void Backward(const std::vector<Elim>& elims) {
        // Increments from the current values, known at the window and block.
        Eigen::aligned_map<int64_t, std::pair<Vec15, uint32_t>> inc;
        constexpr uint32_t kAll = (1u << POSE_VEL_BIAS_SIZE) - 1;
        for (const auto& [id, idx] : mvSteps.back()->mOrderNew.abs_order_map) {
            if (id == kGravityScaleStateId) continue;
            inc[id] = {Vec15::Zero(), kAll};
        }

        for (size_t i = mvSteps.size(); i-- > 0;) {
            const DelayedMargStep<Scalar>& step = *mvSteps[i];
            const Elim& e = elims[i];

            std::vector<std::pair<int64_t, int>> colVar(step.mOrder.total_size,
                                                        {0, -1});
            for (const auto& [id, idx] : step.mOrder.abs_order_map) {
                for (int c = 0; c < idx.second; c++) {
                    colVar[idx.first + c] = {id, c};
                }
            }

            const int m = int(step.mIdxMarg.size());
            const int n = int(step.mIdxKeep.size());
            VecX xKeep = VecX::Zero(n);
            {
                int j = 0;
                for (int col : step.mIdxKeep) {
                    const auto& [id, c] = colVar[col];
                    if (id != kGravityScaleStateId) {
                        const auto it = inc.find(id);
                        BASALT_ASSERT(it != inc.end() &&
                                      ((it->second.second >> c) & 1u));
                        xKeep[j] = it->second.first[c];
                    }
                    j++;
                }
            }

            VecX xMarg = VecX::Zero(m);
            for (int k = m - 1; k >= 0; k--) {
                const Eigen::Index t = e.mPivotRow[k];
                if (t < 0 || t >= e.mJ.rows()) continue;
                Scalar s = e.mR[t] + e.mJ.row(t).tail(n).dot(xKeep);
                if (k + 1 < m) {
                    s += e.mJ.row(t).segment(k + 1, m - k - 1).dot(
                        xMarg.segment(k + 1, m - k - 1));
                }
                xMarg[k] = -s / e.mJ(t, k);
            }

            int k = 0;
            for (int col : step.mIdxMarg) {
                const auto& [id, c] = colVar[col];
                auto& entry = inc[id];
                if (entry.second == 0) entry.first.setZero();
                entry.first[c] = xMarg[k];
                entry.second |= 1u << c;
                k++;
            }
        }

        mvSmoothInc.clear();
        mMaxPosInc = 0;
        for (auto& [id, rec] : mValues.mvRecords) {
            const auto it = inc.find(id);
            if (it == inc.end()) continue;
            // Coordinates still in the window are conditioned, not solved.
            uint32_t mask = it->second.second;
            if (mValues.mvWindowPoses.count(id) > 0) mask &= ~0x3Fu;
            if (mValues.mvWindowStates.count(id) > 0) mask = 0;
            if (mask == 0) continue;

            Vec15 x = Vec15::Zero();
            for (int c = 0; c < int(POSE_VEL_BIAS_SIZE); c++) {
                if ((mask >> c) & 1u) x[c] = it->second.first[c];
            }
            if (!x.allFinite()) continue;
            rec.mState.applyInc(x);
            mvSmoothInc[id] = {x, mask};
            mMaxPosInc = std::max(mMaxPosInc, x.template head<3>().norm());
        }
    }

    // Marginalises one step into prior. The prior arrives and leaves delta
    // independent, its rows evaluated at the current values in between,
    // §2.3.4. gsImu is the block at which the inertial factors are
    // linearised and becomes the anchor of the result.
    static void MarginaliseStep(const DelayedMargStep<Scalar>& step,
                                const DelayedMargValues<Scalar>& values,
                                const GravityScaleStateWithLin<Scalar>& gsImu,
                                const Vec3& g, const Vec3& gyroW,
                                const Vec3& accelW,
                                const GravityScaleCorrectionLog* log,
                                int64_t epoch, bool imuScaleFree,
                                Scalar imuScaleFreeTol,
                                DelayedMargPrior<Scalar>& prior, Elim* elim) {
        const int n = int(step.mOrder.total_size);
        const int pr = int(prior.mH.rows());
        const int pc = int(prior.mH.cols());
        const int vr = int(step.mVisJ.rows());
        const int ir = int(step.mvImu.size()) * POSE_VEL_BIAS_SIZE;

        BASALT_ASSERT(pc <= n);
        BASALT_ASSERT(step.mOrder.abs_order_map.at(kGravityScaleStateId) ==
                      prior.mOrder.abs_order_map.at(kGravityScaleStateId));

        MatX Q2Jp = MatX::Zero(pr + vr + ir, n);
        VecX Q2r = VecX::Zero(pr + vr + ir);

        // The current block seen from the prior's anchor.
        GravityScaleStateWithLin<Scalar> gsPrior = gsImu;
        gsPrior.Reanchor(prior.mAnchor.mDir, prior.mAnchor.mBasis,
                         prior.mAnchor.mSigma);
        Q2Jp.topLeftCorner(pr, pc) = prior.mH;
        Q2r.head(pr) =
            prior.mB +
            prior.mH * values.Delta(prior.mOrder, gsPrior.GetDelta());

        if (vr > 0) {
            MatX J = step.mVisJ;
            if (step.mEpoch != epoch) {
                const GravityScaleCorrectionLog::Entry c =
                    log->Between(step.mEpoch, epoch);
                DelayedMargTransformColumns(
                    J, step.mOrder, c.mQ.template cast<Scalar>(),
                    Scalar(c.mScale));
            }
            Q2Jp.middleRows(pr, vr) = J;
            Q2r.segment(pr, vr) =
                step.mVisR + J * values.Delta(step.mOrder, Vec3::Zero());
        }

        if (ir > 0) {
            ImuLinData<Scalar> ild = {g, gyroW, accelW, {}};
            Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<Scalar>> states;
            int row = pr + vr;
            for (const IntegratedImuMeasurement<Scalar>& meas : step.mvImu) {
                const int64_t t0 = meas.get_start_t_ns();
                const int64_t t1 = t0 + meas.get_dt_ns();
                states[t0] = values.State(t0);
                states[t1] = values.State(t1);
                ImuBlockGS<Scalar> block(&meas, &ild, step.mOrder, &gsImu,
                                         imuScaleFree, imuScaleFreeTol);
                block.linearizeImu(states);
                block.add_dense_Q2Jp_Q2r(Q2Jp, Q2r, row);
                row += POSE_VEL_BIAS_SIZE;
            }
        }

        MatX H_new;
        VecX b_new;
        MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
            Q2Jp, Q2r, step.mIdxKeep, step.mIdxMarg, H_new, b_new,
            elim ? &elim->mJ : nullptr, elim ? &elim->mR : nullptr,
            elim ? &elim->mPivotRow : nullptr);

        prior.mH = H_new;
        prior.mOrder = step.mOrderNew;
        prior.mAnchor.mDir = gsImu.mDirLin;
        prior.mAnchor.mBasis = gsImu.mBasisLin;
        prior.mAnchor.mSigma = gsImu.mSigmaLin;
        prior.mB = b_new - prior.mH * values.Delta(step.mOrderNew,
                                                    gsImu.GetDelta());
    }
};

// The delayed store of the main thread, plans/delayed_marginalisation.md
// §3.2.
template <class Scalar_>
class DelayedMarginalisation {
   public:
    using Scalar = Scalar_;
    using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Step = DelayedMargStep<Scalar>;
    using StepPtr = std::shared_ptr<const Step>;
    using Job = DelayedMargJob<Scalar>;

    explicit DelayedMarginalisation(int delay) : mpDelay(delay) {}

    bool BoundarySet() const { return mpBoundarySet; }

    void SetBoundary(const DelayedMargPrior<Scalar>& prior) {
        mpBoundary = prior;
        mpBoundarySet = true;
    }

    const DelayedMargPrior<Scalar>& Boundary() const { return mpBoundary; }
    const std::deque<StepPtr>& Steps() const { return mvpSteps; }
    DelayedMargValues<Scalar>& Values() { return mpValues; }
    const DelayedMargValues<Scalar>& Values() const { return mpValues; }
    int64_t NextIndex() const { return mpNextIndex; }

    // Records the coordinates that leave the window at this step.
    void RecordDeparture(int64_t id, const PoseVelBiasStateWithLin<Scalar>& st,
                         int64_t step) {
        DelayedMargRecord<Scalar>& rec = mpValues.mvRecords[id];
        rec.mState = Frozen(st);
        rec.mLastStep = step;
    }

    void RecordPoseDeparture(int64_t id, const PoseStateWithLin<Scalar>& ps,
                             int64_t step) {
        auto it = mpValues.mvRecords.find(id);
        PoseVelBiasState<Scalar> lin;
        typename PoseVelBiasStateWithLin<Scalar>::VecN delta;
        delta.setZero();
        if (it != mpValues.mvRecords.end()) {
            lin = it->second.mState.getStateLin();
            delta = it->second.mState.getDelta();
        }
        lin.t_ns = id;
        lin.T_w_i = ps.getPoseLin();
        delta.template head<POSE_SIZE>() = ps.getDelta();
        PoseVelBiasStateWithLin<Scalar> st(id, lin.T_w_i, lin.vel_w_i,
                                           lin.bias_gyro, lin.bias_accel,
                                           true);
        st.applyInc(delta);
        DelayedMargRecord<Scalar>& rec = mpValues.mvRecords[id];
        rec.mState = st;
        rec.mLastStep = step;
    }

    void Push(Step&& step) {
        for (IntegratedImuMeasurement<Scalar>& meas : step.mvImu) {
            // Fills the lazily computed whitening before the step is shared.
            meas.get_sqrt_cov_inv();
        }
        mpNextIndex = step.mIndex + 1;
        mvpSteps.push_back(std::make_shared<const Step>(std::move(step)));
    }

    // Moves the oldest steps beyond the delay into the boundary prior, with
    // the scale free inertial rows of Change A when boundaryImu is set,
    // §2.3.5.
    int AdvanceBoundary(const GravityScaleStateWithLin<Scalar>& gs,
                        const Vec3& g, const Vec3& gyroW, const Vec3& accelW,
                        const GravityScaleCorrectionLog* log, int64_t epoch,
                        bool boundaryImu, Scalar scaleFreeTol) {
        int advanced = 0;
        while (int(mvpSteps.size()) > mpDelay) {
            const StepPtr step = mvpSteps.front();
            GravityScaleStateWithLin<Scalar> gsB = gs;
            gsB.Reanchor(mpBoundary.mAnchor.mDir, mpBoundary.mAnchor.mBasis,
                         gs.GetSigma());

            Step stepB = *step;
            if (!boundaryImu) stepB.mvImu.clear();
            Job::MarginaliseStep(stepB, mpValues, gsB, g, gyroW, accelW, log,
                                 epoch, true, scaleFreeTol, mpBoundary,
                                 nullptr);

            mvpSteps.pop_front();
            for (auto it = mpValues.mvRecords.begin();
                 it != mpValues.mvRecords.end();) {
                if (it->second.mLastStep <= step->mIndex) {
                    it = mpValues.mvRecords.erase(it);
                } else {
                    ++it;
                }
            }
            advanced++;
        }
        return advanced;
    }

    // Applies a re-centring to the boundary and the records. The steps keep
    // their own epoch and are mapped when read.
    void Transform(const Sophus::SO3<Scalar>& Q, Scalar s, Scalar sigmaShift) {
        DelayedMargTransformColumns(mpBoundary.mH, mpBoundary.mOrder, Q, s);
        mpBoundary.mAnchor.Transform(Q, sigmaShift);
        for (auto& [id, rec] : mpValues.mvRecords) {
            rec.mState.TransformBySim3(Q, s);
        }
    }

    static PoseVelBiasStateWithLin<Scalar> Frozen(
        const PoseVelBiasStateWithLin<Scalar>& st) {
        const PoseVelBiasState<Scalar>& lin = st.getStateLin();
        PoseVelBiasStateWithLin<Scalar> out(lin.t_ns, lin.T_w_i, lin.vel_w_i,
                                            lin.bias_gyro, lin.bias_accel,
                                            true);
        out.applyInc(st.getDelta());
        return out;
    }

   private:
    int mpDelay;
    bool mpBoundarySet = false;
    int64_t mpNextIndex = 0;
    DelayedMargPrior<Scalar> mpBoundary;
    std::deque<StepPtr> mvpSteps;
    DelayedMargValues<Scalar> mpValues;
};

}  // namespace basalt
```

Backwards compatibility. A new header, included only by `sqrt_keypoint_vio.h` and the test.

#### Change 5. Estimator declarations, `include/basalt/vi_estimator/sqrt_keypoint_vio.h`

The include, two optional output arguments of `LinearizeMargGS` for the store's visual rows, four methods and the members of the store. `mpDmEnabled` selects the single prior path for the whole period, while `mpDm` exists only while the store is active.

```diff
--- a/include/basalt/vi_estimator/sqrt_keypoint_vio.h
+++ b/include/basalt/vi_estimator/sqrt_keypoint_vio.h
@@ -35,11 +35,13 @@
 #pragma once
 
 #include <basalt/imu/preintegration.h>
+#include <basalt/vi_estimator/delayed_marginalisation.h>
 #include <basalt/vi_estimator/sqrt_ba_base.h>
 #include <basalt/vi_estimator/vio_estimator.h>
 
 #include <basalt/utils/logger.h>
 #include <deque>
+#include <future>
 #include <mutex>
 #include <thread>
 
@@ -143,7 +145,9 @@
                          const std::unordered_set<KeypointId>& lost_landmaks,
                          int64_t last_state_to_marg, MatX& Q2Jp_curr,
                          VecX& Q2r_curr, MatX& Q2Jp_half, VecX& Q2r_half,
-                         MatX& Q2Jp_visual, VecX& Q2r_visual);
+                         MatX& Q2Jp_visual, VecX& Q2r_visual,
+                         MatX* Q2Jp_store = nullptr,
+                         VecX* Q2r_store = nullptr);
     void MarginalizeGS(MatX& Q2Jp_curr, VecX& Q2r_curr, MatX& Q2Jp_half,
                        VecX& Q2r_half, MatX& Q2Jp_visual, VecX& Q2r_visual,
                        const std::set<int>& idx_to_keep,
@@ -155,6 +159,10 @@
     MatX ComputeGaugeProjector(const AbsOrderMap& aom) const;
     void SetInitialPriorOrder(int64_t t_ns);
     void LogGravityScale(int swapEvent);
+    void UpdateDelayedMarg(DelayedMargStep<Scalar>&& step);
+    std::shared_ptr<DelayedMargJob<Scalar>> MakeDelayedJob() const;
+    int InstallDelayedJob(DelayedMargJob<Scalar>& job);
+    void StopDelayedMarg();
 
     void debug_finalize() override;
 
@@ -289,6 +297,15 @@
     Scalar mpGsStepTiltDeg = 0;
     Scalar mpGsJumpCost = 0;
     int mpGsVisualImuFull = 0;
+    // Delayed marginalisation, plans/delayed_marginalisation.md. mpDm is the
+    // store, released once replacements stop while mpDmEnabled stays set.
+    bool mpDmEnabled = false;
+    std::unique_ptr<DelayedMarginalisation<Scalar>> mpDm;
+    std::future<std::shared_ptr<DelayedMargJob<Scalar>>> mpDmFuture;
+    int64_t mpDmPriorOldestStep = 0;
+    bool mpDmReplaced = false;
+    bool mpDmFoldRequested = false;
+    double mpDmStoreSeconds = 0;
     int mpGsVisualImuRot = 0;
     Scalar mpGsVisualImuScaleCol = 0;
     int frames_after_kf;
```

Backwards compatibility. `LinearizeMargGS` has one caller, `marginalize`, and its new arguments default to null. `std::future` makes the class move only, which it already is through `std::mutex` and `std::shared_ptr<std::thread>`.

#### Change 6. Construction, pose updates and `marginalize`, `src/vi_estimator/sqrt_keypoint_vio.cpp`

Construction creates the store when both keys are set.

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -144,6 +144,14 @@
         this->mGsCorrectionLog = std::make_shared<GravityScaleCorrectionLog>();
         mpGsActive = true;
         mpLogD = std::log(Scalar(config.vio_gs_dmin));
+
+        mpDmEnabled = config.vio_gs_delayed_marg;
+        if (mpDmEnabled) {
+            BASALT_ASSERT_STREAM(config.vio_gs_dm_delay >= 1,
+                                 "vio_gs_dm_delay must be at least 1");
+            mpDm = std::make_unique<DelayedMarginalisation<Scalar>>(
+                config.vio_gs_dm_delay);
+        }
     }
 
     std::cout << "marg_H (sqrt:" << marg_data.is_sqrt << ")\n"
```

`measure` drops the mapper's pose updates while the store is active and the hold is set (§2.3.8). The dropped count is logged as rejected in `vio_pose_update.csv`.

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -610,6 +618,14 @@
     {
         std::lock_guard<std::mutex> lock(mpPosesToUpdateMutex);
         poseUpdatePending = int(mpPosesToUpdate.size());
+        // Held while the delayed store is active, so that no readvance meets
+        // a keyframe pose moved by the visual only mapper,
+        // plans/delayed_marginalisation.md §2.3.8.
+        if (mpDm && config.vio_gs_dm_hold_pose_updates) {
+            poseUpdateRejected = poseUpdatePending;
+            mpPosesToUpdate.clear();
+            mpPosesToUpdateEpoch.clear();
+        }
         if (!mpPosesToUpdate.empty()) {
             for (auto it = mpPosesToUpdate.begin();
                  it != mpPosesToUpdate.end();) {
```

`marginalize` skips the swap rule, sets the delayed prior at the first marginalisation, asks `LinearizeMargGS` for the store's visual rows, captures the step before the departing coordinates are erased, keeps $M_\text{half}$ and $M_\text{visual}$ untouched, pushes the step after the main prior is recentred, and folds on settle. The visual rows are made delta independent with offsets read directly from the window, because `computeDelta` asserts that every state is frozen and `last_state_to_marg` is frozen only after the capture (`ba_base.cpp:327-334`, `sqrt_keypoint_vio.cpp:1478`). An unfrozen state has a zero offset, so the direct read is exact.

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -1088,7 +1104,7 @@
         for (int i = 0; i < states_to_remove; i++) it++;
         int64_t last_state_to_marg = it->first;
 
-        if (mpGsActive) UpdateScaleInterval();
+        if (mpGsActive && !mpDmEnabled) UpdateScaleInterval();
 
         AbsOrderMap aom;
 
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -1244,10 +1260,26 @@
         MatX Q2Jp_half, Q2Jp_visual;
         VecX Q2r_half, Q2r_visual;
 
+        MatX Q2Jp_store;
+        VecX Q2r_store;
+
         if (mpGsActive) {
+            if (mpDm && !mpDm->BoundarySet()) {
+                // The prior before the first marginalisation of the period is
+                // the boundary of the store, plans/delayed_marginalisation.md
+                // §2.3.5.
+                DelayedMargPrior<Scalar> boundary;
+                boundary.mH = marg_data.H;
+                boundary.mB = marg_data.b;
+                boundary.mOrder = marg_data.order;
+                boundary.mAnchor.Set(*this->mGsState);
+                mpDm->SetBoundary(boundary);
+            }
             LinearizeMargGS(aom, kfs_to_marg, lost_landmaks, last_state_to_marg,
                             Q2Jp_or_H, Q2r_or_b, Q2Jp_half, Q2r_half,
-                            Q2Jp_visual, Q2r_visual);
+                            Q2Jp_visual, Q2r_visual,
+                            mpDm ? &Q2Jp_store : nullptr,
+                            mpDm ? &Q2r_store : nullptr);
         } else {
             Timer t_linearize;
 
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -1472,6 +1504,68 @@
                 .format("ms");
         }
 
+        DelayedMargStep<Scalar> dmStep;
+        if (mpDm) {
+            Timer tDm;
+            dmStep.mIndex = mpDm->NextIndex();
+            dmStep.mEpoch = this->mGsCorrectionLog->LatestEpoch();
+            dmStep.mOrder = aom;
+            dmStep.mIdxKeep = idx_to_keep;
+            dmStep.mIdxMarg = idx_to_marg;
+
+            // Visual rows in delta independent form, compressed to at most one
+            // row per column, plans/delayed_marginalisation.md §3.2 Change 6.
+            VecX delta = VecX::Zero(aom.total_size);
+            for (const auto& [id, idx] : aom.abs_order_map) {
+                if (id == kGravityScaleStateId) continue;
+                if (idx.second == POSE_SIZE) {
+                    delta.template segment<POSE_SIZE>(idx.first) =
+                        frame_poses.at(id).getDelta();
+                } else {
+                    delta.template segment<POSE_VEL_BIAS_SIZE>(idx.first) =
+                        frame_states.at(id).getDelta();
+                }
+            }
+            Q2r_store -= Q2Jp_store * delta;
+            const int n = int(aom.total_size);
+            if (Q2Jp_store.rows() > n) {
+                MatX A(Q2Jp_store.rows(), n + 1);
+                A << Q2Jp_store, Q2r_store;
+                const Eigen::HouseholderQR<MatX> qr(A);
+                const MatX R = qr.matrixQR()
+                                   .topRows(n)
+                                   .template triangularView<Eigen::Upper>();
+                dmStep.mVisJ = R.leftCols(n);
+                dmStep.mVisR = R.col(n);
+            } else {
+                dmStep.mVisJ = Q2Jp_store;
+                dmStep.mVisR = Q2r_store;
+            }
+
+            for (const auto& kv : imu_meas) {
+                const int64_t start_t = kv.second.get_start_t_ns();
+                const int64_t end_t = start_t + kv.second.get_dt_ns();
+                if (aom.abs_order_map.count(start_t) == 0 ||
+                    aom.abs_order_map.count(end_t) == 0)
+                    continue;
+                dmStep.mvImu.push_back(kv.second);
+            }
+
+            for (const int64_t id : states_to_marg_all) {
+                mpDm->RecordDeparture(id, frame_states.at(id), dmStep.mIndex);
+            }
+            for (const int64_t id : states_to_marg_vel_bias) {
+                mpDm->RecordDeparture(id, frame_states.at(id), dmStep.mIndex);
+            }
+            for (const int64_t id : poses_to_marg) {
+                if (frame_poses.count(id) > 0) {
+                    mpDm->RecordPoseDeparture(id, frame_poses.at(id),
+                                              dmStep.mIndex);
+                }
+            }
+            mpDmStoreSeconds = tDm.elapsed();
+        }
+
         {
             BASALT_ASSERT(frame_states.at(last_state_to_marg).isLinearized() ==
                           false);
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -1533,7 +1627,7 @@
             marg_data.b = marg_b_new;
         }
         marg_data.order = marg_order_new;
-        if (mpGsActive) {
+        if (mpGsActive && !mpDmEnabled) {
             mpMargHalf.order = marg_order_new;
             mpMargVisual.order = marg_order_new;
         }
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -1565,6 +1659,10 @@
 
         if (mpGsActive) {
             RecentrePriorsGS();
+            if (mpDm) {
+                dmStep.mOrderNew = marg_order_new;
+                UpdateDelayedMarg(std::move(dmStep));
+            }
         } else {
             VecX delta;
             computeDelta(marg_data.order, delta);
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -1601,7 +1699,8 @@
     if (mpGsActive) {
         LogGravityScale(mpGsSwapEvent);
         if (last_state_t_ns - mpGsStartNs >=
-            int64_t(config.vio_gs_duration_s * 1e9)) {
+                int64_t(config.vio_gs_duration_s * 1e9) ||
+            mpDmFoldRequested) {
             FoldGravityScale();
         }
     }
```

Backwards compatibility. Every statement is behind `mpDm`, `mpDmEnabled` or `mpDmFoldRequested`, which are null and false unless both keys are set. The packet to the mapper is still built from the main prior's linearisation, unchanged.

#### Change 7. Re-centring, linearisation, marginalisation and fold in the delayed mode, `src/vi_estimator/sqrt_keypoint_vio.cpp`

`RecentreGravityScale` maps the store and stops mapping $M_\text{half}$ and $M_\text{visual}$, which the delayed mode no longer maintains. `LinearizeMargGS`, `MarginalizeGS` and `RecentrePriorsGS` handle the main prior only, and `LinearizeMargGS` adds the visual only linearisation at the current block of §2.3.3. `FoldGravityScale` stops the store first.

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -2345,8 +2444,11 @@
     };
 
     transform_prior(marg_data);
-    transform_prior(mpMargHalf);
-    transform_prior(mpMargVisual);
+    if (!mpDmEnabled) {
+        transform_prior(mpMargHalf);
+        transform_prior(mpMargVisual);
+    }
+    if (mpDm) mpDm->Transform(Q, s, sigma_c);
 
     for (auto& [frame_id, state] : frame_poses) state.TransformBySim3(Q, s);
     for (auto& [frame_id, state] : frame_states) state.TransformBySim3(Q, s);
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -2442,7 +2544,8 @@
     const AbsOrderMap& aom, const std::set<int64_t>& kfs_to_marg,
     const std::unordered_set<KeypointId>& lost_landmaks,
     int64_t last_state_to_marg, MatX& Q2Jp_curr, VecX& Q2r_curr,
-    MatX& Q2Jp_half, VecX& Q2r_half, MatX& Q2Jp_visual, VecX& Q2r_visual) {
+    MatX& Q2Jp_half, VecX& Q2r_half, MatX& Q2Jp_visual, VecX& Q2r_visual,
+    MatX* Q2Jp_store, VecX* Q2r_store) {
     typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
     lqr_options.lb_options.huber_parameter = huber_thresh;
     lqr_options.lb_options.obs_std_dev = obs_std_dev;
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -2485,6 +2588,25 @@
         }
     };
 
+    if (mpDmEnabled) {
+        linearize(marg_data, mpAnchorCurr, true, false, false, Q2Jp_curr,
+                  Q2r_curr);
+        if (Q2Jp_store) {
+            // Visual rows of the store, scale free and evaluated at the
+            // current block, plans/delayed_marginalisation.md §2.3.3.
+            GravityScaleStateWithLin<Scalar> gsNow = *this->mGsState;
+            gsNow.Reanchor(gsNow.GetDir(), gsNow.GetBasis(), gsNow.GetSigma());
+            LinearizationGSAbsQR<Scalar, POSE_SIZE> lqr(
+                this, aom, lqr_options, &gsNow, true, nullptr, nullptr,
+                &kfs_to_marg, &lost_landmaks, last_state_to_marg);
+            lqr.linearizeProblem();
+            lqr.performQR();
+            lqr.get_dense_Q2Jp_Q2r(*Q2Jp_store, *Q2r_store);
+        }
+        AnchorGs(mpAnchorCurr);
+        return;
+    }
+
     const bool visual_inertial = config.vio_gs_visual_prior_inertial;
     GsAnchor visual_anchor_copy = mpAnchorCurr;
     GsAnchor& visual_anchor =
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -2522,6 +2644,7 @@
     };
 
     marginalize_into(marg_data, Q2Jp_curr, Q2r_curr);
+    if (mpDmEnabled) return;
     marginalize_into(mpMargHalf, Q2Jp_half, Q2r_half);
     marginalize_into(mpMargVisual, Q2Jp_visual, Q2r_visual);
 }
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -2535,6 +2658,12 @@
         mld.b -= mld.H * delta;
     };
 
+    if (mpDmEnabled) {
+        recentre(marg_data, mpAnchorCurr);
+        AnchorGs(mpAnchorCurr);
+        return;
+    }
+
     GsAnchor visual_anchor_copy = mpAnchorCurr;
     GsAnchor& visual_anchor = config.vio_gs_visual_prior_inertial
                                   ? mpAnchorVisual
```

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -2572,6 +2701,7 @@
 
 template <class Scalar_>
 void SqrtKeypointVioEstimator<Scalar_>::FoldGravityScale() {
+    StopDelayedMarg();
     AnchorGs(mpAnchorCurr);
 
     const GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
```

Backwards compatibility. Each early return is taken only with `mpDmEnabled`, and the paths after it are byte for byte today's. `StopDelayedMarg` is a no operation when no store exists.

#### Change 8. Installing, updating and stopping the store, `src/vi_estimator/sqrt_keypoint_vio.cpp`

`MakeDelayedJob` copies the snapshot. `InstallDelayedJob` maps the job's prior to the current epoch by (D5), applies the smoothing increments to the live records through the same map, readvances the steps stored meanwhile at the current block, and installs the result with its anchor. `UpdateDelayedMarg` runs after every marginalisation of the delayed mode, in the order push, advance, install, settle test, trigger, log. `StopDelayedMarg` waits for a running job and releases the store.

```diff
--- a/src/vi_estimator/sqrt_keypoint_vio.cpp
+++ b/src/vi_estimator/sqrt_keypoint_vio.cpp
@@ -2612,6 +2742,167 @@
 }
 
 template <class Scalar_>
+std::shared_ptr<DelayedMargJob<Scalar_>>
+SqrtKeypointVioEstimator<Scalar_>::MakeDelayedJob() const {
+    auto job = std::make_shared<DelayedMargJob<Scalar>>();
+    job->mBoundary = mpDm->Boundary();
+    job->mvSteps.assign(mpDm->Steps().begin(), mpDm->Steps().end());
+    job->mValues.mvRecords = mpDm->Values().mvRecords;
+    job->mValues.mvWindowStates = frame_states;
+    job->mValues.mvWindowPoses = frame_poses;
+    job->mGs = *this->mGsState;
+    job->mG = g;
+    job->mGyroBiasWeightSqrt = gyro_bias_sqrt_weight;
+    job->mAccelBiasWeightSqrt = accel_bias_sqrt_weight;
+    job->mLog = this->mGsCorrectionLog;
+    job->mEpoch = this->mGsCorrectionLog->LatestEpoch();
+    job->mSmooth = config.vio_gs_dm_smooth;
+    return job;
+}
+
+template <class Scalar_>
+int SqrtKeypointVioEstimator<Scalar_>::InstallDelayedJob(
+    DelayedMargJob<Scalar>& job) {
+    const int64_t now = this->mGsCorrectionLog->LatestEpoch();
+    DelayedMargPrior<Scalar> prior = job.mResult;
+
+    // The job ran in the epoch of its snapshot.
+    Sophus::SO3<Scalar> Q;
+    Scalar s = 1;
+    if (job.mEpoch != now) {
+        const GravityScaleCorrectionLog::Entry c =
+            this->mGsCorrectionLog->Between(job.mEpoch, now);
+        Q = c.mQ.template cast<Scalar>();
+        s = Scalar(c.mScale);
+        DelayedMargTransformColumns(prior.mH, prior.mOrder, Q, s);
+        prior.mAnchor.Transform(Q, std::log(s));
+    }
+
+    DelayedMargValues<Scalar>& values = mpDm->Values();
+    const Mat3 Qm = Q.matrix();
+    for (const auto& [id, inc] : job.mvSmoothInc) {
+        auto it = values.mvRecords.find(id);
+        if (it == values.mvRecords.end()) continue;
+        typename PoseVelBiasStateWithLin<Scalar>::VecN x = inc.first;
+        x.template segment<3>(0) = s * (Qm * x.template segment<3>(0));
+        x.template segment<3>(3) = Qm * x.template segment<3>(3);
+        x.template segment<3>(6) = s * (Qm * x.template segment<3>(6));
+        it->second.mState.applyInc(x);
+    }
+
+    // Steps stored while the job ran, readvanced here at the current block.
+    values.mvWindowStates = frame_states;
+    values.mvWindowPoses = frame_poses;
+    GravityScaleStateWithLin<Scalar> gsNow = *this->mGsState;
+    gsNow.Reanchor(gsNow.GetDir(), gsNow.GetBasis(), gsNow.GetSigma());
+    int catchup = 0;
+    for (const auto& step : mpDm->Steps()) {
+        if (step->mIndex <= job.LastStep()) continue;
+        DelayedMargJob<Scalar>::MarginaliseStep(
+            *step, values, gsNow, g, gyro_bias_sqrt_weight,
+            accel_bias_sqrt_weight, this->mGsCorrectionLog.get(), now, false,
+            Scalar(0), prior, nullptr);
+        catchup++;
+    }
+
+    BASALT_ASSERT(prior.mOrder.abs_order_map == marg_data.order.abs_order_map);
+    marg_data.H = prior.mH;
+    marg_data.b = prior.mB;
+    mpAnchorCurr.dir = prior.mAnchor.mDir;
+    mpAnchorCurr.basis = prior.mAnchor.mBasis;
+    mpAnchorCurr.sigma = prior.mAnchor.mSigma;
+    mpAnchorCurr.set = true;
+    AnchorGs(mpAnchorCurr);
+
+    mpDmPriorOldestStep = job.FirstStep();
+    mpDmReplaced = true;
+    return catchup;
+}
+
+template <class Scalar_>
+void SqrtKeypointVioEstimator<Scalar_>::UpdateDelayedMarg(
+    DelayedMargStep<Scalar>&& step) {
+    Timer tStore;
+    const int64_t epoch = this->mGsCorrectionLog->LatestEpoch();
+    mpDm->Push(std::move(step));
+    DelayedMargValues<Scalar>& values = mpDm->Values();
+    values.mvWindowStates = frame_states;
+    values.mvWindowPoses = frame_poses;
+    mpDm->AdvanceBoundary(*this->mGsState, g, gyro_bias_sqrt_weight,
+                          accel_bias_sqrt_weight,
+                          this->mGsCorrectionLog.get(), epoch,
+                          config.vio_gs_visual_prior_inertial,
+                          Scalar(config.vio_gs_visual_prior_scale_tol));
+    const double storeSeconds = mpDmStoreSeconds + tStore.elapsed();
+
+    int started = 0, installed = 0, settled = 0, catchup = 0;
+    double jobSeconds = 0, catchupSeconds = 0;
+    Scalar maxPosInc = 0;
+
+    auto install = [&](DelayedMargJob<Scalar>& job) {
+        Timer t;
+        catchup = InstallDelayedJob(job);
+        catchupSeconds = t.elapsed();
+        jobSeconds = job.mSeconds;
+        maxPosInc = job.mMaxPosInc;
+        installed = 1;
+    };
+
+    if (mpDmFuture.valid() && mpDmFuture.wait_for(std::chrono::seconds(0)) ==
+                                  std::future_status::ready) {
+        std::shared_ptr<DelayedMargJob<Scalar>> job = mpDmFuture.get();
+        install(*job);
+    }
+
+    const GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
+    const Scalar anchorSigma = std::abs(gs.GetSigma() - mpAnchorCurr.sigma);
+    const Scalar anchorTilt = std::acos(std::clamp(
+        gs.GetDir().dot(mpAnchorCurr.dir), Scalar(-1), Scalar(1)));
+    const int steps = int(mpDm->Steps().size());
+    const int records = int(values.mvRecords.size());
+    const int64_t front =
+        steps > 0 ? mpDm->Steps().front()->mIndex : mpDm->NextIndex();
+    const int64_t lost = std::max<int64_t>(0, front - mpDmPriorOldestStep);
+
+    // DM-VIO stops replacing once a replacement would drop the scale
+    // dependent rows of more than this share of the delay,
+    // plans/delayed_marginalisation.md §2.3.7.
+    if (mpDmReplaced &&
+        double(lost) >= config.vio_gs_dm_max_lost * config.vio_gs_dm_delay) {
+        settled = 1;
+        mpDmFoldRequested = config.vio_gs_dm_fold_on_settle;
+    } else if (!mpDmFuture.valid() && steps > 0 &&
+               (anchorSigma > std::log(Scalar(config.vio_gs_dm_thresh_scale)) ||
+                anchorTilt >
+                    Scalar(config.vio_gs_dm_thresh_tilt_deg * M_PI / 180.0))) {
+        std::shared_ptr<DelayedMargJob<Scalar>> job = MakeDelayedJob();
+        started = 1;
+        if (config.vio_gs_dm_async) {
+            mpDmFuture = std::async(std::launch::async, [job]() {
+                job->Run();
+                return job;
+            });
+        } else {
+            job->Run();
+            install(*job);
+        }
+    }
+
+    mpLogger->AddVioDelayedMarg(
+        last_state_t_ns, steps, records, started, installed, settled,
+        double(anchorSigma), double(anchorTilt) * 180.0 / M_PI, jobSeconds,
+        catchup, catchupSeconds, double(maxPosInc), storeSeconds, int(lost));
+
+    if (settled) StopDelayedMarg();
+}
+
+template <class Scalar_>
+void SqrtKeypointVioEstimator<Scalar_>::StopDelayedMarg() {
+    if (mpDmFuture.valid()) mpDmFuture.get();
+    mpDm.reset();
+}
+
+template <class Scalar_>
 void SqrtKeypointVioEstimator<Scalar_>::debug_finalize() {
     // mpLogger->PrintSummary();
     mpLogger->SaveLegacyStats();
```

Backwards compatibility. New methods, called only from the branches of Changes 6 and 7.

#### Change 9. Configuration files, `data/*.json` and `data/iccv21/basalt_batch_config.toml`

All nine keys are added after `vio_gs_jump_tilt_rw_deg` in the seven JSON files that carry the period's keys and in the batch template. Every file gets the defaults, except `data/sitl_config_vo.json`, which enables the delayed mode and the hold.

```diff
--- a/data/sitl_config_vo.json
+++ b/data/sitl_config_vo.json
@@ -53,6 +53,15 @@
         "config.vio_gs_visual_prior_scale_tol": 1.0,
         "config.vio_gs_jump_sigma_rw": 0.1,
         "config.vio_gs_jump_tilt_rw_deg": 1.0,
+        "config.vio_gs_delayed_marg": true,
+        "config.vio_gs_dm_delay": 200,
+        "config.vio_gs_dm_thresh_scale": 1.02,
+        "config.vio_gs_dm_thresh_tilt_deg": 5.0,
+        "config.vio_gs_dm_max_lost": 0.5,
+        "config.vio_gs_dm_smooth": true,
+        "config.vio_gs_dm_async": true,
+        "config.vio_gs_dm_fold_on_settle": false,
+        "config.vio_gs_dm_hold_pose_updates": true,
         "config.vio_marg_lost_landmarks": true,
         "config.vio_kf_marg_feature_ratio": 0.2,
         "config.mapper_obs_std_dev": 0.25,
```

```diff
--- a/data/euroc_config.json
+++ b/data/euroc_config.json
@@ -53,6 +53,15 @@
         "config.vio_gs_visual_prior_scale_tol": 1.0,
         "config.vio_gs_jump_sigma_rw": 0.0,
         "config.vio_gs_jump_tilt_rw_deg": 0.0,
+        "config.vio_gs_delayed_marg": false,
+        "config.vio_gs_dm_delay": 200,
+        "config.vio_gs_dm_thresh_scale": 1.02,
+        "config.vio_gs_dm_thresh_tilt_deg": 5.0,
+        "config.vio_gs_dm_max_lost": 0.5,
+        "config.vio_gs_dm_smooth": true,
+        "config.vio_gs_dm_async": true,
+        "config.vio_gs_dm_fold_on_settle": false,
+        "config.vio_gs_dm_hold_pose_updates": false,
         "config.vio_marg_lost_landmarks": true,
         "config.vio_kf_marg_feature_ratio": 0.1,
         "config.mapper_obs_std_dev": 0.25,
```

The same nine lines with the defaults are added to the five other JSON files and to the template.

```diff
--- a/data/euroc_config_no_factors.json
+++ b/data/euroc_config_no_factors.json
@@ -53,6 +53,15 @@
         "config.vio_gs_visual_prior_scale_tol": 1.0,
         "config.vio_gs_jump_sigma_rw": 0.0,
         "config.vio_gs_jump_tilt_rw_deg": 0.0,
+        "config.vio_gs_delayed_marg": false,
+        "config.vio_gs_dm_delay": 200,
+        "config.vio_gs_dm_thresh_scale": 1.02,
+        "config.vio_gs_dm_thresh_tilt_deg": 5.0,
+        "config.vio_gs_dm_max_lost": 0.5,
+        "config.vio_gs_dm_smooth": true,
+        "config.vio_gs_dm_async": true,
+        "config.vio_gs_dm_fold_on_settle": false,
+        "config.vio_gs_dm_hold_pose_updates": false,
         "config.vio_marg_lost_landmarks": true,
         "config.vio_kf_marg_feature_ratio": 0.1,
         "config.mapper_obs_std_dev": 0.25,
```

```diff
--- a/data/euroc_config_no_weights.json
+++ b/data/euroc_config_no_weights.json
@@ -53,6 +53,15 @@
         "config.vio_gs_visual_prior_scale_tol": 1.0,
         "config.vio_gs_jump_sigma_rw": 0.0,
         "config.vio_gs_jump_tilt_rw_deg": 0.0,
+        "config.vio_gs_delayed_marg": false,
+        "config.vio_gs_dm_delay": 200,
+        "config.vio_gs_dm_thresh_scale": 1.02,
+        "config.vio_gs_dm_thresh_tilt_deg": 5.0,
+        "config.vio_gs_dm_max_lost": 0.5,
+        "config.vio_gs_dm_smooth": true,
+        "config.vio_gs_dm_async": true,
+        "config.vio_gs_dm_fold_on_settle": false,
+        "config.vio_gs_dm_hold_pose_updates": false,
         "config.vio_marg_lost_landmarks": true,
         "config.vio_kf_marg_feature_ratio": 0.1,
         "config.mapper_obs_std_dev": 0.25,
```

```diff
--- a/data/euroc_config_vo.json
+++ b/data/euroc_config_vo.json
@@ -53,6 +53,15 @@
         "config.vio_gs_visual_prior_scale_tol": 1.0,
         "config.vio_gs_jump_sigma_rw": 0.0,
         "config.vio_gs_jump_tilt_rw_deg": 0.0,
+        "config.vio_gs_delayed_marg": false,
+        "config.vio_gs_dm_delay": 200,
+        "config.vio_gs_dm_thresh_scale": 1.02,
+        "config.vio_gs_dm_thresh_tilt_deg": 5.0,
+        "config.vio_gs_dm_max_lost": 0.5,
+        "config.vio_gs_dm_smooth": true,
+        "config.vio_gs_dm_async": true,
+        "config.vio_gs_dm_fold_on_settle": false,
+        "config.vio_gs_dm_hold_pose_updates": false,
         "config.vio_marg_lost_landmarks": true,
         "config.vio_kf_marg_feature_ratio": 0.2,
         "config.mapper_obs_std_dev": 0.25,
```

```diff
--- a/data/kitti_config.json
+++ b/data/kitti_config.json
@@ -53,6 +53,15 @@
         "config.vio_gs_visual_prior_scale_tol": 1.0,
         "config.vio_gs_jump_sigma_rw": 0.0,
         "config.vio_gs_jump_tilt_rw_deg": 0.0,
+        "config.vio_gs_delayed_marg": false,
+        "config.vio_gs_dm_delay": 200,
+        "config.vio_gs_dm_thresh_scale": 1.02,
+        "config.vio_gs_dm_thresh_tilt_deg": 5.0,
+        "config.vio_gs_dm_max_lost": 0.5,
+        "config.vio_gs_dm_smooth": true,
+        "config.vio_gs_dm_async": true,
+        "config.vio_gs_dm_fold_on_settle": false,
+        "config.vio_gs_dm_hold_pose_updates": false,
         "config.vio_marg_lost_landmarks": true,
         "config.vio_kf_marg_feature_ratio": 0.1,
         "config.mapper_obs_std_dev": 0.25,
```

```diff
--- a/data/tumvi_512_config.json
+++ b/data/tumvi_512_config.json
@@ -53,6 +53,15 @@
         "config.vio_gs_visual_prior_scale_tol": 1.0,
         "config.vio_gs_jump_sigma_rw": 0.0,
         "config.vio_gs_jump_tilt_rw_deg": 0.0,
+        "config.vio_gs_delayed_marg": false,
+        "config.vio_gs_dm_delay": 200,
+        "config.vio_gs_dm_thresh_scale": 1.02,
+        "config.vio_gs_dm_thresh_tilt_deg": 5.0,
+        "config.vio_gs_dm_max_lost": 0.5,
+        "config.vio_gs_dm_smooth": true,
+        "config.vio_gs_dm_async": true,
+        "config.vio_gs_dm_fold_on_settle": false,
+        "config.vio_gs_dm_hold_pose_updates": false,
         "config.vio_marg_lost_landmarks": true,
         "config.vio_kf_marg_feature_ratio": 0.1,
         "config.mapper_obs_std_dev": 0.25,
```

```diff
--- a/data/iccv21/basalt_batch_config.toml
+++ b/data/iccv21/basalt_batch_config.toml
@@ -61,6 +61,15 @@
 "config.vio_gs_visual_prior_scale_tol" = 1.0
 "config.vio_gs_jump_sigma_rw" = 0.0
 "config.vio_gs_jump_tilt_rw_deg" = 0.0
+"config.vio_gs_delayed_marg" = false
+"config.vio_gs_dm_delay" = 200
+"config.vio_gs_dm_thresh_scale" = 1.02
+"config.vio_gs_dm_thresh_tilt_deg" = 5.0
+"config.vio_gs_dm_max_lost" = 0.5
+"config.vio_gs_dm_smooth" = true
+"config.vio_gs_dm_async" = true
+"config.vio_gs_dm_fold_on_settle" = false
+"config.vio_gs_dm_hold_pose_updates" = false
 "config.vio_marg_lost_landmarks" = true
 "config.vio_kf_marg_feature_ratio" = 0.1
 
```

Backwards compatibility. Every configuration except the SITL one keeps the period off or the dynamic mode, and all seven JSON files were loaded with `VioConfig::load` after the change (§3.4.2). `data/sitl_config_vo.json` changes the behaviour of the SITL runs deliberately, and setting `vio_gs_delayed_marg` to false restores today's behaviour exactly.

#### Change 10. Unit tests, `test/src/test_vio.cpp`

Three tests on a synthetic chain of eight states with one inertial factor and ten random visual rows per step and a pinned initial prior. `DelayedMargReadvanceJointTest` compares the readvance with a joint removal of the same rows by the original helper, $\mathbf{H}^\top \mathbf{H}$ and $\mathbf{H}^\top \mathbf{b}$ to a relative $10^{-8}$, and checks that relinearising at a scale moved by 0.2 changes the information by more than $10^{-3}$ relative, so that the comparison is sensitive to the block. `DelayedMargSmoothingJointTest` compares the back substitution with the dense least squares solution of the departed states given the window, to $10^{-6}$. `DelayedMargRecentreInvarianceTest` applies a re-centring to the values, the delayed prior, the block and the log while the steps stay in their epoch, and checks that the readvance equals the mapped readvance of the original, to $10^{-7}$.

```diff
--- a/test/src/test_vio.cpp
+++ b/test/src/test_vio.cpp
@@ -5,6 +5,8 @@
 #include <basalt/utils/ba_utils.h>
 #include <basalt/vi_estimator/sc_ba_base.h>
 #include <basalt/linearization/imu_block.hpp>
+#include <basalt/vi_estimator/delayed_marginalisation.h>
+#include <basalt/vi_estimator/marg_helper.h>
 
 #include <iostream>
 
@@ -535,3 +537,265 @@
         x0);
   }
 }
+
+#ifdef BASALT_INSTANTIATIONS_DOUBLE
+namespace {
+
+// A chain of states 0..N-1 marginalised one per step, each step with one
+// inertial factor and random visual rows on the two poses, for the tests of
+// plans/delayed_marginalisation.md §3.2 Change 10.
+struct DelayedMargChain {
+  static constexpr int N = 8;
+  static constexpr int64_t kDt = 50000000;
+
+  basalt::Se3Spline<5> spline{int64_t(10e9)};
+  basalt::DelayedMargValues<double> values;
+  basalt::DelayedMargPrior<double> boundary;
+  std::vector<std::shared_ptr<const basalt::DelayedMargStep<double>>> steps;
+  basalt::GravityScaleStateWithLin<double> gs;
+  Eigen::Vector3d weight = Eigen::Vector3d::Constant(1e2);
+  basalt::GravityScaleCorrectionLog::Ptr log =
+      std::make_shared<basalt::GravityScaleCorrectionLog>();
+
+  static basalt::AbsOrderMap Order(std::initializer_list<int64_t> ids) {
+    basalt::AbsOrderMap o;
+    o.abs_order_map[basalt::kGravityScaleStateId] = {0, 3};
+    o.total_size = 3;
+    o.items = 1;
+    for (int64_t id : ids) {
+      o.abs_order_map[id] = {int(o.total_size), 15};
+      o.total_size += 15;
+      o.items++;
+    }
+    return o;
+  }
+
+  DelayedMargChain() {
+    spline.genRandomTrajectory(15);
+    const Eigen::Vector3d bg = Eigen::Vector3d::Random() / 100;
+    const Eigen::Vector3d ba = Eigen::Vector3d::Random() / 10;
+
+    for (int i = 0; i < N; i++) {
+      const int64_t t = i * kDt;
+      basalt::PoseVelBiasState<double> lin(
+          t, spline.pose(t) * Sophus::se3_expd(Sophus::Vector6d::Random() / 50),
+          spline.transVelWorld(t) + Eigen::Vector3d::Random() / 20, bg, ba);
+      basalt::PoseVelBiasStateWithLin<double> st(t, lin.T_w_i, lin.vel_w_i,
+                                                 bg, ba, true);
+      Eigen::Matrix<double, 15, 1> d = Eigen::Matrix<double, 15, 1>::Random();
+      st.applyInc(d / 100);
+      if (i == N - 1) {
+        values.mvWindowStates[t] = st;
+      } else {
+        values.mvRecords[t].mState = st;
+        values.mvRecords[t].mLastStep = i;
+      }
+    }
+
+    gs.ApplyInc(Eigen::Vector3d(0.03, -0.02, 0.1));
+    gs.Reanchor(gs.GetDir(), gs.GetBasis(), gs.GetSigma());
+    gs.mDelta = Eigen::Vector3d(0.01, 0.02, -0.05);
+
+    boundary.mOrder = Order({0});
+    boundary.mH.setZero(15, 18);
+    boundary.mH.diagonal().setZero();
+    for (int c : {3, 4, 5, 8, 12, 13, 14, 15, 16, 17}) boundary.mH(c - 3, c) = 1e2;
+    boundary.mB = Eigen::VectorXd::Random(15);
+    boundary.mAnchor.Set(gs);
+
+    for (int i = 0; i + 1 < N; i++) {
+      Eigen::Vector3d accel_cov = Eigen::Vector3d::Constant(0.04);
+      Eigen::Vector3d gyro_cov = Eigen::Vector3d::Constant(1e-4);
+      basalt::IntegratedImuMeasurement<double> meas(i * kDt, bg, ba);
+      for (int64_t t = i * kDt + 2500000; t < (i + 1) * kDt; t += 5000000) {
+        basalt::ImuData<double> data;
+        const Sophus::SE3d pose = spline.pose(t);
+        data.accel = pose.so3().inverse() *
+                         (spline.transAccelWorld(t) - basalt::constants::g) +
+                     ba;
+        data.gyro = spline.rotVelBody(t) + bg;
+        data.t_ns = t + 2500000;
+        meas.integrate(data, accel_cov, gyro_cov);
+      }
+
+      basalt::DelayedMargStep<double> step;
+      step.mIndex = i;
+      step.mEpoch = 0;
+      step.mOrder = Order({i * kDt, (i + 1) * kDt});
+      step.mOrderNew = Order({(i + 1) * kDt});
+      for (int c = 0; c < 33; c++) {
+        (c >= 3 && c < 18 ? step.mIdxMarg : step.mIdxKeep).insert(c);
+      }
+      step.mVisJ.setZero(10, 33);
+      step.mVisJ.middleCols(3, 6).setRandom();
+      step.mVisJ.middleCols(18, 6).setRandom();
+      step.mVisJ *= 30;
+      step.mVisR = Eigen::VectorXd::Random(10);
+      step.mvImu.push_back(meas);
+      step.mvImu.back().get_sqrt_cov_inv();
+      steps.push_back(
+          std::make_shared<const basalt::DelayedMargStep<double>>(step));
+    }
+  }
+
+  basalt::DelayedMargJob<double> Job() const {
+    basalt::DelayedMargJob<double> job;
+    job.mBoundary = boundary;
+    job.mvSteps = steps;
+    job.mValues = values;
+    job.mGs = gs;
+    job.mG = basalt::constants::g;
+    job.mGyroBiasWeightSqrt = weight;
+    job.mAccelBiasWeightSqrt = weight;
+    job.mLog = log;
+    job.mEpoch = log->LatestEpoch();
+    return job;
+  }
+
+  // Every row of the chain at the current values over the joint order.
+  void Joint(Eigen::MatrixXd& J, Eigen::VectorXd& r,
+             basalt::AbsOrderMap& order) const {
+    std::vector<int64_t> ids;
+    for (int i = 0; i < N; i++) ids.push_back(i * kDt);
+    order = Order({});
+    for (int64_t id : ids) {
+      order.abs_order_map[id] = {int(order.total_size), 15};
+      order.total_size += 15;
+      order.items++;
+    }
+    basalt::GravityScaleStateWithLin<double> gsNow = gs;
+    gsNow.Reanchor(gs.GetDir(), gs.GetBasis(), gs.GetSigma());
+
+    const int rows = 15 + int(steps.size()) * (10 + 15);
+    J.setZero(rows, order.total_size);
+    r.setZero(rows);
+
+    basalt::GravityScaleStateWithLin<double> gsB = gsNow;
+    gsB.Reanchor(boundary.mAnchor.mDir, boundary.mAnchor.mBasis,
+                 boundary.mAnchor.mSigma);
+    J.topLeftCorner(15, 18) = boundary.mH;
+    r.head(15) = boundary.mB +
+                 boundary.mH * values.Delta(boundary.mOrder, gsB.GetDelta());
+
+    int row = 15;
+    for (const auto& s : steps) {
+      const int c0 = order.abs_order_map.at(s->mvImu[0].get_start_t_ns()).first;
+      J.block(row, 0, 10, 3) = s->mVisJ.leftCols(3);
+      J.block(row, c0, 10, 30) = s->mVisJ.rightCols(30);
+      r.segment(row, 10) =
+          s->mVisR + s->mVisJ * values.Delta(s->mOrder, Eigen::Vector3d::Zero());
+      row += 10;
+
+      basalt::ImuLinData<double> ild = {basalt::constants::g, weight, weight, {}};
+      Eigen::aligned_map<int64_t, basalt::PoseVelBiasStateWithLin<double>> st;
+      const int64_t t0 = s->mvImu[0].get_start_t_ns();
+      const int64_t t1 = t0 + s->mvImu[0].get_dt_ns();
+      st[t0] = values.State(t0);
+      st[t1] = values.State(t1);
+      basalt::ImuBlockGS<double> block(&s->mvImu[0], &ild, order, &gsNow);
+      block.linearizeImu(st);
+      block.add_dense_Q2Jp_Q2r(J, r, row);
+      row += 15;
+    }
+  }
+};
+
+}  // namespace
+
+TEST(VioTestSuite, DelayedMargReadvanceJointTest) {
+  DelayedMargChain chain;
+  basalt::DelayedMargJob<double> job = chain.Job();
+  job.Forward(nullptr);
+
+  Eigen::MatrixXd J;
+  Eigen::VectorXd r;
+  basalt::AbsOrderMap order;
+  chain.Joint(J, r, order);
+
+  const int last = order.abs_order_map.at((chain.N - 1) * chain.kDt).first;
+  std::set<int> keep, marg;
+  for (int c = 0; c < int(order.total_size); c++) {
+    (c < 3 || c >= last ? keep : marg).insert(c);
+  }
+  Eigen::MatrixXd H;
+  Eigen::VectorXd b;
+  basalt::MargHelper<double>::marginalizeHelperSqrtToSqrt(J, r, keep, marg, H,
+                                                          b);
+  b -= H * chain.values.Delta(chain.steps.back()->mOrderNew,
+                              Eigen::Vector3d::Zero());
+
+  const Eigen::MatrixXd& Hs = job.mResult.mH;
+  const Eigen::VectorXd& bs = job.mResult.mB;
+  const Eigen::MatrixXd G_ref = H.transpose() * H;
+  const Eigen::MatrixXd G_seq = Hs.transpose() * Hs;
+  EXPECT_LE((G_seq - G_ref).norm(), 1e-8 * G_ref.norm());
+  EXPECT_LE((Hs.transpose() * bs - H.transpose() * b).norm(),
+            1e-8 * (H.transpose() * b).norm());
+  EXPECT_LE(std::abs(job.mResult.mAnchor.mSigma - chain.gs.GetSigma()), 1e-12);
+
+  // The prior depends on the block at which the inertial factors are
+  // relinearised, which is what a replacement changes.
+  basalt::DelayedMargJob<double> moved = chain.Job();
+  moved.mGs.mDelta[2] += 0.2;
+  moved.Forward(nullptr);
+  const Eigen::MatrixXd G_moved = moved.mResult.mH.transpose() * moved.mResult.mH;
+  EXPECT_GE((G_moved - G_ref).norm(), 1e-3 * G_ref.norm());
+}
+
+TEST(VioTestSuite, DelayedMargSmoothingJointTest) {
+  DelayedMargChain chain;
+  basalt::DelayedMargJob<double> job = chain.Job();
+  std::vector<basalt::DelayedMargJob<double>::Elim> elims;
+  job.Forward(&elims);
+  job.Backward(elims);
+
+  Eigen::MatrixXd J;
+  Eigen::VectorXd r;
+  basalt::AbsOrderMap order;
+  chain.Joint(J, r, order);
+
+  const int last = order.abs_order_map.at((chain.N - 1) * chain.kDt).first;
+  const Eigen::MatrixXd Jd = J.middleCols(3, last - 3);
+  const Eigen::VectorXd x = -(Jd.transpose() * Jd).ldlt().solve(Jd.transpose() * r);
+
+  for (int i = 0; i + 1 < chain.N; i++) {
+    const auto& inc = job.mvSmoothInc.at(i * chain.kDt);
+    EXPECT_EQ(inc.second, (1u << 15) - 1);
+    EXPECT_LE((inc.first - x.segment<15>(15 * i)).norm(),
+              1e-6 * (1.0 + x.segment<15>(15 * i).norm()));
+  }
+}
+
+TEST(VioTestSuite, DelayedMargRecentreInvarianceTest) {
+  DelayedMargChain chain;
+  basalt::DelayedMargJob<double> before = chain.Job();
+  before.Forward(nullptr);
+
+  const Sophus::SO3d Q = chain.gs.AlignmentRotation(basalt::constants::g);
+  const double sigma_c = chain.gs.GetSigma();
+  const double s = std::exp(sigma_c);
+  chain.values.TransformBySim3(Q, s);
+  basalt::DelayedMargTransformColumns(chain.boundary.mH,
+                                      chain.boundary.mOrder, Q, s);
+  chain.boundary.mAnchor.Transform(Q, sigma_c);
+  chain.gs.TransformLin(Q, sigma_c);
+  chain.log->Append(Q, s);
+
+  basalt::DelayedMargJob<double> after = chain.Job();
+  after.Forward(nullptr);
+
+  Eigen::MatrixXd H_mapped = before.mResult.mH;
+  basalt::DelayedMargTransformColumns(H_mapped, before.mResult.mOrder, Q, s);
+  const Eigen::MatrixXd G_ref = H_mapped.transpose() * H_mapped;
+  const Eigen::MatrixXd G_new =
+      after.mResult.mH.transpose() * after.mResult.mH;
+  EXPECT_LE((G_new - G_ref).norm(), 1e-7 * G_ref.norm());
+  EXPECT_LE((after.mResult.mH.transpose() * after.mResult.mB -
+             H_mapped.transpose() * before.mResult.mB)
+                .norm(),
+            1e-7 * (1.0 + (H_mapped.transpose() * before.mResult.mB).norm()));
+  EXPECT_LE((after.mResult.mAnchor.mDir - Q * before.mResult.mAnchor.mDir)
+                .norm(),
+            1e-9);
+}
+#endif
```

Backwards compatibility. Tests only, behind `BASALT_INSTANTIATIONS_DOUBLE` like the existing ones.

#### Change 12. Delayed marginalisation as the only mechanism, configuration, estimator declarations and the removals

Configuration, `include/basalt/utils/vio_config.h` and `src/utils/vio_config.cpp`. The keys `vio_gs_dmin`, `vio_gs_delayed_marg`, `vio_gs_dm_max_lost`, `vio_gs_dm_fold_on_settle`, `vio_gs_optimise_gravity_dir` and `vio_gs_update_every_n_frames` are removed with their defaults and their `CEREAL_NVP` lines, `vio_gs_dm_coarse_every` (default 5) and `vio_gs_dm_coarse_scale_var` (default 1.0) are added after `vio_gs_dm_thresh_tilt_deg`, and the comments of `vio_ba_step_prior_std` and `vio_gravity_prior_std_deg` state their new roles. The block of `vio_config.h` reads

```cpp
    // Linearise the delayed prior with the scale free rows of the inertial
    // factors, plans/gravity_scale_optimisation_2.md Change A.
    bool vio_gs_visual_prior_inertial;
    // Largest whitened scale column with which an inertial factor enters
    // the delayed prior with all of its rows.
    double vio_gs_visual_prior_scale_tol;
    // Random walk intensities of the per frame gravity and scale step, per
    // square root second, zero disables, plans/gravity_scale_optimisation_2.md
    // Change B.
    double vio_gs_jump_sigma_rw;
    double vio_gs_jump_tilt_rw_deg;
    // Delayed marginalisation, plans/delayed_marginalisation.md. The store
    // keeps the last vio_gs_dm_delay marginalisations. Until it first holds
    // that many, the block is held in the window and a coarse inertial
    // optimisation runs every vio_gs_dm_coarse_every marginalisations. Then
    // the block is free and the prior is readvanced at the current block when
    // the block moves past either threshold.
    int vio_gs_dm_delay;
    double vio_gs_dm_thresh_scale;
    double vio_gs_dm_thresh_tilt_deg;
    int vio_gs_dm_coarse_every;
    // Largest marginal variance of the log scale with which a coarse
    // optimisation updates the scale.
    double vio_gs_dm_coarse_scale_var;
    bool vio_gs_dm_smooth;
    bool vio_gs_dm_async;
    // Drop the local mapper's pose updates during the period,
    // plans/delayed_marginalisation.md §2.3.8.
    bool vio_gs_dm_hold_pose_updates;
    // Random walk intensity of the accelerometer bias change estimated by a
    // coarse optimisation, m/s^2 per square root second, 0 leaves it free.
    double vio_ba_step_prior_std;
    // Prior on the tilt of gravity relative to its initial estimate, on the
    // first state in the initial prior and on the block in every coarse
    // optimisation, degrees, 0 disables.
    double vio_gravity_prior_std_deg;
```

Estimator declarations, `include/basalt/vi_estimator/sqrt_keypoint_vio.h`. `UpdateScaleInterval`, `ResetHalfFromVisual`, `mpGsMotionStarted`, `mpGsMotionFrames`, `mpMargHalf`, `mpMargVisual`, `mpAnchorHalf`, `mpAnchorVisual`, `mpSigmaMid`, `mpSigmaPrevMarg`, `mpLogD`, `mpLastUpper`, `mpDmEnabled`, `mpDmPriorOldestStep`, `mpDmReplaced` and `mpDmFoldRequested` are removed. `LinearizeMargGS` takes the store's outputs by reference and no longer returns the rows of $M_\text{half}$ and $M_\text{visual}$, `MarginalizeGS` takes the current prior's rows only, `MakeDelayedJob` takes the kind of job, and `InstallCoarseJob` is new. The members of the store read

```cpp
    // Delayed marginalisation, plans/delayed_marginalisation.md. mpDm is the
    // store, kept for the whole period. The block is held in the window while
    // mpDmCoarse is set, until the store first holds vio_gs_dm_delay steps.
    std::unique_ptr<DelayedMarginalisation<Scalar>> mpDm;
    std::future<std::shared_ptr<DelayedMargJob<Scalar>>> mpDmFuture;
    bool mpDmCoarse = true;
    bool mpDmForceReplace = false;
    int64_t mpDmLastCoarseIndex = 0;
```

Construction, `src/vi_estimator/sqrt_keypoint_vio.cpp`. The store is created whenever the period is enabled.

```cpp
        mpGsActive = true;

        BASALT_ASSERT_STREAM(config.vio_gs_dm_delay >= 1 &&
                                 config.vio_gs_dm_coarse_every >= 1,
                             "vio_gs_dm_delay and vio_gs_dm_coarse_every must "
                             "be at least 1");
        mpDm = std::make_unique<DelayedMarginalisation<Scalar>>(
            config.vio_gs_dm_delay);
```

The marginalisation of the period. `marginalize` no longer calls `UpdateScaleInterval`, captures the step whenever the period is active, and passes the two row sets of its single prior and of the store.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::LinearizeMargGS(
    const AbsOrderMap& aom, const std::set<int64_t>& kfs_to_marg,
    const std::unordered_set<KeypointId>& lost_landmaks,
    int64_t last_state_to_marg, MatX& Q2Jp_curr, VecX& Q2r_curr,
    MatX& Q2Jp_store, VecX& Q2r_store) {
    typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
    lqr_options.lb_options.huber_parameter = huber_thresh;
    lqr_options.lb_options.obs_std_dev = obs_std_dev;
    lqr_options.linearization_type = config.vio_linearization_type;

    ImuLinData<Scalar> ild = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight,
                              {}};

    for (const auto& kv : imu_meas) {
        int64_t start_t = kv.second.get_start_t_ns();
        int64_t end_t = kv.second.get_start_t_ns() + kv.second.get_dt_ns();

        if (aom.abs_order_map.count(start_t) == 0 ||
            aom.abs_order_map.count(end_t) == 0)
            continue;

        ild.imu_meas[kv.first] = &kv.second;
    }

    {
        AnchorGs(mpAnchorCurr);
        LinearizationGSAbsQR<Scalar, POSE_SIZE> lqr(
            this, aom, lqr_options, &*this->mGsState, false, &marg_data, &ild,
            &kfs_to_marg, &lost_landmaks, last_state_to_marg);
        lqr.linearizeProblem();
        lqr.performQR();
        lqr.get_dense_Q2Jp_Q2r(Q2Jp_curr, Q2r_curr);
    }

    {
        // Visual rows of the store, scale free and evaluated at the current
        // block, plans/delayed_marginalisation.md §2.3.3.
        GravityScaleStateWithLin<Scalar> gsNow = *this->mGsState;
        gsNow.Reanchor(gsNow.GetDir(), gsNow.GetBasis(), gsNow.GetSigma());
        LinearizationGSAbsQR<Scalar, POSE_SIZE> lqr(
            this, aom, lqr_options, &gsNow, true, nullptr, nullptr,
            &kfs_to_marg, &lost_landmaks, last_state_to_marg);
        lqr.linearizeProblem();
        lqr.performQR();
        lqr.get_dense_Q2Jp_Q2r(Q2Jp_store, Q2r_store);
    }

    AnchorGs(mpAnchorCurr);
}
```

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::MarginalizeGS(
    MatX& Q2Jp_curr, VecX& Q2r_curr, const std::set<int>& idx_to_keep,
    const std::set<int>& idx_to_marg) {
    MatX H_new;
    VecX b_new;
    MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
        Q2Jp_curr, Q2r_curr, idx_to_keep, idx_to_marg, H_new, b_new);
    marg_data.H = H_new;
    marg_data.b = b_new;
}
```

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::RecentrePriorsGS() {
    AnchorGs(mpAnchorCurr);
    VecX delta;
    computeDelta(marg_data.order, delta);
    marg_data.b -= marg_data.H * delta;
}
```

The window's accelerometer bias step prior of `plans/gravity_and_bias_step_priors.md` (`baStepW2`, `baRef`, `baStepCost` and their four uses in `optimize`) is removed, and so are the scale only projector of `ComputeGaugeProjector`, the copies of the initial prior into $M_\text{half}$ and $M_\text{visual}$ in `SetInitialPriorOrder`, their transforms and anchors in `RecentreGravityScale`, and their reset in `FoldGravityScale`. The hold of the coarse phase replaces the cadence and the tilt hold in `optimize`.

```cpp
        // The block is moved only by the coarse optimisation until the store
        // first holds the delay, plans/delayed_marginalisation.md §2.3.9.
        const bool gsHoldBlock = mpGsActive && mpDmCoarse;

        // Random walk residual on the gravity and scale step of this frame,
        // plans/gravity_scale_optimisation_2.md §3.4. gsJump is the block
        // increment accumulated over the accepted iterations.
        Vec3 gsJump = Vec3::Zero();
        Vec3 gsJumpW2 = Vec3::Zero();
        if (mpGsActive && !imu_meas.empty()) {
            const Scalar dtFrame =
                Scalar(imu_meas.rbegin()->second.get_dt_ns()) * Scalar(1e-9);
            if (config.vio_gs_jump_tilt_rw_deg > 0) {
                const Scalar tiltStd =
                    Scalar(config.vio_gs_jump_tilt_rw_deg * M_PI / 180.0) *
                    std::sqrt(dtFrame);
                gsJumpW2.template head<2>().setConstant(
                    Scalar(1) / (tiltStd * tiltStd));
            }
            if (config.vio_gs_jump_sigma_rw > 0) {
                const Scalar sigmaStd =
                    Scalar(config.vio_gs_jump_sigma_rw) * std::sqrt(dtFrame);
                gsJumpW2[2] = Scalar(1) / (sigmaStd * sigmaStd);
            }
        }
        const bool applyGsJump =
            !gsHoldBlock && (gsJumpW2.array() > Scalar(0)).any();
```

```cpp
                    MatX gauge_P;
                    if (mpGsActive) {
                        if (gsHoldBlock) {
                            gauge_P = MatX::Identity(H.rows(), H.cols());
                        } else {
                            gauge_P = ComputeGaugeProjector(aom);
                            H = gauge_P * H * gauge_P;
                            b = gauge_P * b;
                        }
                        if (gsHoldBlock) {
                            const int nHeld = GRAVITY_SCALE_SIZE;
                            const int gs =
                                aom.abs_order_map.at(kGravityScaleStateId)
                                    .first;
                            H.middleRows(gs, nHeld).setZero();
                            H.middleCols(gs, nHeld).setZero();
                            H.diagonal().segment(gs, nHeld).setOnes();
                            b.segment(gs, nHeld).setZero();
                        }
                    }
```

Backwards compatibility. With `vio_gs_enable` false, the default of every configuration file except the SITL one, none of the removed or changed code executed before or executes now, and Basalt's estimator is unchanged. With the period enabled the behaviour changes by design, which is the purpose of the revision, and every configuration file loses the six removed keys in Change 15, since cereal ignores extra keys but the removed keys would be dead (`context/vio_config_json.md`). No consumer outside `ext/basalt` reads any removed symbol, which a search of `/ws/ros_ws/src/slam` confirmed. The `swap_event` column of `vio_gravity_scale.csv` keeps its schema, no longer carries the values 1 to 3 of the swaps, and gains 5 for an installed coarse correction and 6 for an installed readvance, beside 4 for the fold, so the dashboard's event timeline shows the delayed marginalisation events.

#### Change 13. The coarse optimisation, `include/basalt/vi_estimator/delayed_marginalisation.h`

`DelayedMargJob` gains the inputs and outputs of the coarse optimisation and runs it instead of a readvance when `mCoarse` is set. The forward and backward passes of Change 4 are unchanged. Note, 2026-11-14. Change 16 replaced the coarse optimisation below, which eliminated the velocities over three nodes, by DM-VIO's inertial only estimate, and the code below is kept as the record of the form of 2026-11-13 that §2.3.11 rejects.

```cpp
    // Coarse inertial optimisation instead of a readvance, §2.3.9. The
    // inertial factors of the window, the initial gravity direction in the
    // epoch mEpoch, and the information of the tilt and accelerometer bias
    // priors.
    bool mCoarse = false;
    Eigen::aligned_vector<IntegratedImuMeasurement<Scalar>> mvWindowImu;
    Vec3 mInitialDir = Vec3::Zero();
    Scalar mTiltPriorW = 0;
    Scalar mBaPriorW = 0;
    Scalar mScaleVarMax = 0;
    int mNodeSpacing = 1;
    // Effective acceleration error of three chained nodes and the smallest
    // visual displacement over which they enter, §2.3.9.
    static constexpr Scalar kCoarseAccelStd = Scalar(0.1);
    static constexpr Scalar kCoarseMinDisplacement = Scalar(0.2);

    // Outputs.
    DelayedMargPrior<Scalar> mResult;
    // Smoothing increment of each departed variable, and which of its 15
    // coordinates it covers.
    Eigen::aligned_map<int64_t, std::pair<Vec15, uint32_t>> mvSmoothInc;
    Scalar mMaxPosInc = 0;
    double mSeconds = 0;
    // Coarse outputs, the block in the epoch mEpoch and the common bias
    // increments.
    Vec3 mCoarseDir = Vec3::Zero();
    Scalar mCoarseSigma = 0;
    Vec3 mCoarseBg = Vec3::Zero();
    Vec3 mCoarseBa = Vec3::Zero();
    Scalar mCoarseScaleVar = 0;
    bool mCoarseScaleFree = false;
    Scalar mCoarseCost0 = 0;
    Scalar mCoarseCost1 = 0;
    int mCoarseFactors = 0;
    int mCoarseTriples = 0;
```

```cpp
    void Run() {
        const auto t0 = std::chrono::steady_clock::now();
        if (mCoarse) {
            Coarse();
        } else if (mSmooth) {
            std::vector<Elim> elims;
            Forward(&elims);
            Backward(elims);
        }
```

The optimisation of §2.3.11.

```cpp
    // Gravity direction, log scale and a common accelerometer bias increment
    // from the inertial factors of the store and the window with every pose
    // held, after Mur-Artal and Tardos, with the visual displacements as the
    // predicted side. The factors are chained between nodes
    // mNodeSpacing frames apart in the world frame of the held rotations, and
    // the node velocities are eliminated over every three consecutive nodes
    // whose visual displacement exceeds kCoarseMinDisplacement. The scale is
    // held when its marginal variance exceeds mScaleVarMax, §2.3.9.
    void Coarse() {
        using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
        using Mat6 = Eigen::Matrix<Scalar, 6, 6>;
        using Vec6 = Eigen::Matrix<Scalar, 6, 1>;
        using Mat36 = Eigen::Matrix<Scalar, 3, 6>;
        using Mat32 = Eigen::Matrix<Scalar, 3, 2>;
        using SO3 = Sophus::SO3<Scalar>;

        // Chained inertial quantities from one node to the next, in the world
        // frame, with their derivatives in the accelerometer bias increment.
        struct Span {
            Vec3 pa = Vec3::Zero(), pb = Vec3::Zero();
            Scalar T = 0;
            Vec3 P = Vec3::Zero(), V = Vec3::Zero();
            Mat3 JP = Mat3::Zero(), JV = Mat3::Zero();
        };

        std::map<int64_t, const IntegratedImuMeasurement<Scalar>*> factors;
        auto add = [&](const IntegratedImuMeasurement<Scalar>& meas) {
            const int64_t t0 = meas.get_start_t_ns();
            if (Known(t0) && Known(t0 + meas.get_dt_ns())) factors[t0] = &meas;
        };
        for (const StepPtr& step : mvSteps) {
            for (const auto& meas : step->mvImu) add(meas);
        }
        for (const auto& meas : mvWindowImu) add(meas);
        mCoarseFactors = int(factors.size());

        std::vector<std::vector<Span>> chains(1);
        Span span;
        int inSpan = 0;
        int64_t tNext = -1;
        for (const auto& [t0, meas] : factors) {
            const PoseVelBiasState<Scalar> s0 = mValues.State(t0).getState();
            if (t0 != tNext) {
                if (!chains.back().empty()) chains.emplace_back();
                span = Span();
                span.pa = s0.T_w_i.translation();
                inSpan = 0;
            }
            // Bias corrected deltas and their bias derivatives, read from the
            // residual at identity states, zero gravity and zero scale.
            typename IntegratedImuMeasurement<Scalar>::MatN3 dBa;
            const typename PoseVelState<Scalar>::VecN res = meas->residualGS(
                PoseVelState<Scalar>(), Vec3::Zero(), PoseVelState<Scalar>(),
                s0.bias_gyro, s0.bias_accel, Scalar(0), nullptr, nullptr,
                nullptr, &dBa);
            const Mat3 R = s0.T_w_i.so3().matrix();
            const Scalar dt = Scalar(meas->get_dt_ns()) * Scalar(1e-9);
            span.P += span.V * dt - R * res.template head<3>();
            span.JP += span.JV * dt - R * dBa.template topRows<3>();
            span.V -= R * res.template tail<3>();
            span.JV -= R * dBa.template bottomRows<3>();
            span.T += dt;
            tNext = t0 + meas->get_dt_ns();
            if (++inSpan == mNodeSpacing) {
                span.pb = mValues.State(tNext).getState().T_w_i.translation();
                chains.back().push_back(span);
                span = Span();
                span.pa = chains.back().back().pb;
                inSpan = 0;
            }
        }

        const Vec3 dir0 = mGs.GetDir();
        const Mat32 basis0 = mGs.GetBasis();
        const Scalar sigma0 = mGs.GetSigma();
        const Scalar gNorm = mG.norm();
        const Scalar s0 = std::exp(sigma0);
        const Scalar w = Scalar(1) / (kCoarseAccelStd * kCoarseAccelStd);

        // Consecutive span pairs with enough visual displacement.
        std::vector<std::pair<const Span*, const Span*>> pairs;
        for (const auto& chain : chains) {
            for (size_t i = 0; i + 1 < chain.size(); i++) {
                if (s0 * (chain[i + 1].pb - chain[i].pa).norm() >=
                    kCoarseMinDisplacement) {
                    pairs.emplace_back(&chain[i], &chain[i + 1]);
                }
            }
        }
        mCoarseTriples = int(pairs.size());

        SO3 R;
        Scalar dSigma = 0;
        Vec3 dBa = Vec3::Zero();

        auto tiltResidual = [&](const Vec3& dir, const Mat32& basis) {
            const Vec3 axis = mInitialDir.cross(dir);
            const Scalar sinAngle = axis.norm();
            Vec3 v = Vec3::Zero();
            if (sinAngle > Sophus::Constants<Scalar>::epsilon()) {
                v = axis *
                    (std::atan2(sinAngle, mInitialDir.dot(dir)) / sinAngle);
            }
            return Eigen::Matrix<Scalar, 2, 1>(basis.transpose() * v);
        };

        // Unknowns [tilt, log scale, accelerometer bias increment].
        auto linearise = [&](bool withJ, Mat6* H, Vec6* b) {
            const Vec3 dir = R * dir0;
            const Mat32 basis = R.matrix() * basis0;
            const Vec3 gD = gNorm * dir;
            const Scalar s = std::exp(sigma0 + dSigma);
            Scalar cost = 0;
            if (withJ) {
                H->setZero();
                b->setZero();
            }
            for (const auto& [a, c] : pairs) {
                const Scalar Ta = a->T, Tc = c->T;
                const Scalar k = Scalar(0.5) * Ta * Tc * (Ta + Tc);
                const Vec3 vis = (c->pb - c->pa) * Ta - (a->pb - a->pa) * Tc;
                const Vec3 imu = Ta * (c->P + c->JP * dBa) -
                                 Tc * (a->P + a->JP * dBa) +
                                 Ta * Tc * (a->V + a->JV * dBa);
                // The visual side carries the noise of the held positions, so
                // it is the side predicted, which keeps the scale unbiased.
                const Vec3 pred = (k * gD + imu) / s;
                const Vec3 r = s0 * (vis - pred) / k;
                cost += Scalar(0.5) * w * r.squaredNorm();
                if (!withJ) continue;
                Mat36 J;
                J.template leftCols<2>() =
                    -(s0 / s) * (-SO3::hat(gD) * basis);
                J.col(2) = s0 * pred / k;
                J.template rightCols<3>() =
                    -(s0 / s) * (Ta * c->JP - Tc * a->JP + Ta * Tc * a->JV) /
                    k;
                *H += w * J.transpose() * J;
                *b += w * J.transpose() * r;
            }
            if (mTiltPriorW > 0) {
                const Eigen::Matrix<Scalar, 2, 1> rt =
                    tiltResidual(dir, basis);
                cost += Scalar(0.5) * mTiltPriorW * rt.squaredNorm();
                if (withJ) {
                    H->template topLeftCorner<2, 2>().diagonal().array() +=
                        mTiltPriorW;
                    b->template head<2>() += mTiltPriorW * rt;
                }
            }
            if (mBaPriorW > 0) {
                cost += Scalar(0.5) * mBaPriorW * dBa.squaredNorm();
                if (withJ) {
                    H->template bottomRightCorner<3, 3>()
                        .diagonal()
                        .array() += mBaPriorW;
                    b->template tail<3>() += mBaPriorW * dBa;
                }
            }
            return cost;
        };

        Mat6 H;
        Vec6 b;
        mCoarseCost0 = linearise(false, nullptr, nullptr);
        mCoarseScaleFree = false;
        for (int it = 0; it < 10; it++) {
            linearise(true, &H, &b);
            H.diagonal().array() += Scalar(1e-9);
            if (it == 0) {
                mCoarseScaleVar = H.inverse()(2, 2);
                mCoarseScaleFree = std::isfinite(double(mCoarseScaleVar)) &&
                                   mCoarseScaleVar <= mScaleVarMax;
            }
            if (!mCoarseScaleFree) {
                H.row(2).setZero();
                H.col(2).setZero();
                H(2, 2) = 1;
                b[2] = 0;
            }
            const Vec6 x = -H.ldlt().solve(b);
            if (!x.allFinite()) break;
            const Mat32 basis = R.matrix() * basis0;
            R = SO3::exp(basis * x.template head<2>()) * R;
            dSigma += x[2];
            dBa += x.template tail<3>();
            if (x.norm() < Scalar(1e-9)) break;
        }
        mCoarseCost1 = linearise(false, nullptr, nullptr);

        mCoarseDir = R * dir0;
        mCoarseSigma = sigma0 + dSigma;
        mCoarseBg.setZero();
        mCoarseBa = dBa;
    }

    bool Known(int64_t id) const {
        return mValues.mvWindowStates.count(id) > 0 ||
               mValues.mvRecords.count(id) > 0;
    }
```

Backwards compatibility. Only the estimator in the coarse phase and the unit test construct a coarse job. The readvance of Change 4 runs as before for every other job.

#### Change 14. The two phases, `src/vi_estimator/sqrt_keypoint_vio.cpp`, and the `vio_delayed_marg` record

`MakeDelayedJob` fills the coarse inputs, including copies of the window's inertial factors with their whitening cached before they reach the worker, the initial gravity direction in the snapshot's epoch and the two prior weights. Readvances of the coarse phase run without smoothing. Note, 2026-11-14. Change 16 replaced `InstallCoarseJob`, the coarse branch of the install and the logger call below, so that the coarse job's own readvance is installed with its correction and no separate readvance is forced.

```cpp
template <class Scalar_>
std::shared_ptr<DelayedMargJob<Scalar_>>
SqrtKeypointVioEstimator<Scalar_>::MakeDelayedJob(bool coarse) const {
    auto job = std::make_shared<DelayedMargJob<Scalar>>();
    job->mBoundary = mpDm->Boundary();
    job->mvSteps.assign(mpDm->Steps().begin(), mpDm->Steps().end());
    job->mValues.mvRecords = mpDm->Values().mvRecords;
    job->mValues.mvWindowStates = frame_states;
    job->mValues.mvWindowPoses = frame_poses;
    job->mGs = *this->mGsState;
    job->mG = g;
    job->mGyroBiasWeightSqrt = gyro_bias_sqrt_weight;
    job->mAccelBiasWeightSqrt = accel_bias_sqrt_weight;
    job->mLog = this->mGsCorrectionLog;
    job->mEpoch = this->mGsCorrectionLog->LatestEpoch();
    // Readvances of the coarse phase keep the departed states, so that the
    // coarse optimisation alone moves the history.
    job->mSmooth = config.vio_gs_dm_smooth && !mpDmCoarse;
    job->mCoarse = coarse;
    if (coarse) {
        for (const auto& kv : imu_meas) {
            job->mvWindowImu.push_back(kv.second);
            job->mvWindowImu.back().get_sqrt_cov_inv();
        }
        job->mInitialDir =
            this->mGsCorrectionLog->Between(0, job->mEpoch)
                .mQ.template cast<Scalar>() *
            g.normalized();
        if (config.vio_gravity_prior_std_deg > 0) {
            const Scalar sigma =
                Scalar(config.vio_gravity_prior_std_deg * M_PI / 180.0);
            job->mTiltPriorW = Scalar(1) / (sigma * sigma);
        }
        if (config.vio_ba_step_prior_std > 0 && !imu_meas.empty()) {
            const Scalar dt =
                Scalar(imu_meas.rbegin()->second.get_dt_ns()) * Scalar(1e-9) *
                Scalar(config.vio_gs_dm_coarse_every);
            job->mBaPriorW =
                Scalar(1) / (Scalar(config.vio_ba_step_prior_std *
                                    config.vio_ba_step_prior_std) *
                             dt);
        }
        job->mScaleVarMax = Scalar(config.vio_gs_dm_coarse_scale_var);
        job->mNodeSpacing = config.vio_gs_dm_coarse_every;
    }
    return job;
}
```

```cpp
// Applies a coarse optimisation as a re-centring of the window, the priors and
// the store, and its common bias increments to every state, §2.3.9.
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::InstallCoarseJob(
    const DelayedMargJob<Scalar>& job) {
    const GravityScaleCorrectionLog::Entry c =
        this->mGsCorrectionLog->Between(
            job.mEpoch, this->mGsCorrectionLog->LatestEpoch());
    const Vec3 dir = c.mQ.template cast<Scalar>() * job.mCoarseDir;
    const Scalar sigma = job.mCoarseSigma - Scalar(std::log(c.mScale));

    GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
    gs.Reanchor(gs.GetDir(), gs.GetBasis(), gs.GetSigma());
    const Vec3 axis = gs.GetDir().cross(dir);
    const Scalar sinAngle = axis.norm();
    Vec3 w = Vec3::Zero();
    if (sinAngle > Sophus::Constants<Scalar>::epsilon()) {
        w = axis * (std::atan2(sinAngle, gs.GetDir().dot(dir)) / sinAngle);
    }
    Vec3 inc;
    inc.template head<2>() = gs.mBasisLin.transpose() * w;
    inc[2] = sigma - gs.GetSigma();
    gs.ApplyInc(inc);
    RecentreGravityScale();
    AnchorGs(mpAnchorCurr);

    typename PoseVelBiasStateWithLin<Scalar>::VecN x;
    x.setZero();
    x.template segment<3>(9) = job.mCoarseBg;
    x.template segment<3>(12) = job.mCoarseBa;
    for (auto& [id, state] : frame_states) state.applyInc(x);
    for (auto& [id, rec] : mpDm->Values().mvRecords) {
        if (frame_states.count(id) == 0) rec.mState.applyInc(x);
    }
}
```

`UpdateDelayedMarg` installs a finished job of either kind, leaves the coarse phase once the store has first held the delay, and starts the next job. The settle branch of Change 8 is removed.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::UpdateDelayedMarg(
    DelayedMargStep<Scalar>&& step) {
    Timer tStore;
    const int64_t epoch = this->mGsCorrectionLog->LatestEpoch();
    mpDm->Push(std::move(step));
    DelayedMargValues<Scalar>& values = mpDm->Values();
    values.mvWindowStates = frame_states;
    values.mvWindowPoses = frame_poses;
    mpDm->AdvanceBoundary(*this->mGsState, g, gyro_bias_sqrt_weight,
                          accel_bias_sqrt_weight,
                          this->mGsCorrectionLog.get(), epoch,
                          config.vio_gs_visual_prior_inertial,
                          Scalar(config.vio_gs_visual_prior_scale_tol));
    const double storeSeconds = mpDmStoreSeconds + tStore.elapsed();

    int started = 0, installed = 0, coarse = 0, catchup = 0;
    double jobSeconds = 0, catchupSeconds = 0;
    Scalar maxPosInc = 0;
    const DelayedMargJob<Scalar>* coarseJob = nullptr;
    std::shared_ptr<DelayedMargJob<Scalar>> doneJob;

    auto install = [&](const std::shared_ptr<DelayedMargJob<Scalar>>& job) {
        Timer t;
        if (job->mCoarse) {
            InstallCoarseJob(*job);
            coarseJob = job.get();
            coarse = 1;
            mpGsSwapEvent = 5;
            // A readvance at the corrected block follows at once.
            mpDmForceReplace = true;
        } else {
            catchup = InstallDelayedJob(*job);
            maxPosInc = job->mMaxPosInc;
            installed = 1;
            mpGsSwapEvent = 6;
        }
        catchupSeconds = t.elapsed();
        jobSeconds = job->mSeconds;
        doneJob = job;
    };

    if (mpDmFuture.valid() && mpDmFuture.wait_for(std::chrono::seconds(0)) ==
                                  std::future_status::ready) {
        install(mpDmFuture.get());
    }

    // The coarse phase ends once the store first holds the delay, and a
    // smoothed readvance starts the delayed phase.
    const int steps = int(mpDm->Steps().size());
    if (mpDmCoarse && mpDm->NextIndex() >= config.vio_gs_dm_delay &&
        !mpDmFuture.valid()) {
        mpDmCoarse = false;
        mpDmForceReplace = true;
    }

    const GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
    const Scalar anchorSigma = std::abs(gs.GetSigma() - mpAnchorCurr.sigma);
    const Scalar anchorTilt = std::acos(std::clamp(
        gs.GetDir().dot(mpAnchorCurr.dir), Scalar(-1), Scalar(1)));
    const int records = int(values.mvRecords.size());

    auto start = [&](bool coarseJobKind) {
        std::shared_ptr<DelayedMargJob<Scalar>> job =
            MakeDelayedJob(coarseJobKind);
        started = coarseJobKind ? 2 : 1;
        if (config.vio_gs_dm_async) {
            mpDmFuture = std::async(std::launch::async, [job]() {
                job->Run();
                return job;
            });
        } else {
            job->Run();
            install(job);
        }
    };

    if (!mpDmFuture.valid() && steps > 0) {
        if (mpDmForceReplace ||
            (!mpDmCoarse &&
             (anchorSigma > std::log(Scalar(config.vio_gs_dm_thresh_scale)) ||
              anchorTilt >
                  Scalar(config.vio_gs_dm_thresh_tilt_deg * M_PI / 180.0)))) {
            mpDmForceReplace = false;
            start(false);
        } else if (mpDmCoarse && mpGsStartNs >= 0 &&
                   mpDm->NextIndex() - mpDmLastCoarseIndex >=
                       config.vio_gs_dm_coarse_every) {
            mpDmLastCoarseIndex = mpDm->NextIndex();
            start(true);
        }
    }

    mpLogger->AddVioDelayedMarg(
        last_state_t_ns, steps, records, started, installed, coarse,
        double(anchorSigma), double(anchorTilt) * 180.0 / M_PI, jobSeconds,
        catchup, catchupSeconds, double(maxPosInc), storeSeconds,
        mpDmCoarse ? 1 : 0, coarseJob ? double(coarseJob->mCoarseSigma) : 0.0,
        coarseJob ? double(std::acos(std::clamp(
                        coarseJob->mCoarseDir.dot(coarseJob->mGs.GetDir()),
                        Scalar(-1), Scalar(1)))) *
                        180.0 / M_PI
                  : 0.0,
        coarseJob ? double(coarseJob->mCoarseScaleVar) : 0.0,
        coarseJob ? int(coarseJob->mCoarseScaleFree) : 0,
        coarseJob ? coarseJob->mCoarseBa.norm() : 0.0,
        coarseJob ? double(coarseJob->mCoarseCost0) : 0.0,
        coarseJob ? double(coarseJob->mCoarseCost1) : 0.0,
        coarseJob ? coarseJob->mCoarseTriples : 0);
}
```

The record `vio_delayed_marg` of Change 2 replaces `settled` with `coarse`, the count of installed coarse corrections, replaces `lost_steps` with `coarse_phase`, and gains the coarse result, its scale variance and gate, the bias increment norm, the cost before and after, and the number of triples.

```cpp
    Register("vio_delayed_marg", "[VIO][delayed-marg]",
             {{"t_ns", "ns"}, {"steps", "count"}, {"records", "count"},
              {"started", "count"}, {"installed", "count"},
              {"coarse", "count"}, {"anchor_sigma", "f"},
              {"anchor_tilt_deg", "f"}, {"job", "ms"},
              {"catchup_steps", "count"}, {"catchup", "ms"},
              {"max_pos_inc", "f"}, {"store", "ms"},
              {"coarse_phase", "count"}, {"coarse_sigma", "f"},
              {"coarse_tilt_deg", "f"}, {"coarse_scale_var", "f"},
              {"coarse_scale_free", "count"}, {"coarse_ba", "f"},
              {"coarse_cost0", "f"}, {"coarse_cost1", "f"},
              {"coarse_triples", "count"}});
```

Backwards compatibility. The record is written only while the period is active, and no reader of the earlier columns existed outside this plan.

#### Change 15. Configuration files and unit test

Every configuration file of Change 9 loses the six removed keys and gains `vio_gs_dm_coarse_every` and `vio_gs_dm_coarse_scale_var`, at 5 and 1.0 except in `data/sitl_config_vo.json`, which sets `vio_gs_dm_coarse_every` to 10 and `vio_gs_duration_s` to 7000 s and keeps `vio_gs_dm_hold_pose_updates` true, `vio_ba_step_prior_std` 0.05 and `vio_gravity_prior_std_deg` 0.75. The unit test checks the coarse optimisation on a synthetic trajectory whose positions and velocities are shrunk by a known factor and whose states lack the accelerometer bias of the measurements. Note, 2026-11-14. Change 16 replaced this test by two tests of the new form.

```cpp
// The coarse optimisation recovers the scale of a trajectory whose positions
// and velocities are shrunk by a known factor, and an accelerometer bias the
// states do not carry, plans/delayed_marginalisation.md §2.3.9.
TEST(VioTestSuite, DelayedMargCoarseScaleTest) {
  constexpr int N = 41;
  constexpr int64_t kDt = 50000000;
  const double scale = 1.6;
  basalt::Se3Spline<5> spline(int64_t(10e9));
  spline.genRandomTrajectory(15);
  const Eigen::Vector3d ba(0.05, -0.03, 0.02);

  basalt::DelayedMargJob<double> job;
  for (int i = 0; i < N; i++) {
    const int64_t t = i * kDt;
    Sophus::SE3d pose = spline.pose(t);
    pose.translation() /= scale;
    basalt::PoseVelBiasStateWithLin<double> st(
        t, pose, spline.transVelWorld(t) / scale, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), true);
    if (i == N - 1) {
      job.mValues.mvWindowStates[t] = st;
    } else {
      job.mValues.mvRecords[t].mState = st;
    }
  }
  for (int i = 0; i + 1 < N; i++) {
    basalt::IntegratedImuMeasurement<double> meas(
        i * kDt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    for (int64_t t = i * kDt + 2500000; t < (i + 1) * kDt; t += 5000000) {
      basalt::ImuData<double> data;
      const Sophus::SE3d pose = spline.pose(t);
      data.accel = pose.so3().inverse() *
                       (spline.transAccelWorld(t) - basalt::constants::g) +
                   ba;
      data.gyro = spline.rotVelBody(t);
      data.t_ns = t + 2500000;
      meas.integrate(data, Eigen::Vector3d::Constant(1e-4),
                     Eigen::Vector3d::Constant(1e-6));
    }
    auto step = std::make_shared<basalt::DelayedMargStep<double>>();
    step->mvImu.push_back(meas);
    job.mvSteps.push_back(step);
  }

  job.mG = basalt::constants::g;
  job.mInitialDir = basalt::constants::g.normalized();
  job.mScaleVarMax = 1.0;
  job.mNodeSpacing = 4;
  job.mCoarse = true;
  job.Run();

  EXPECT_TRUE(job.mCoarseScaleFree);
  EXPECT_GT(job.mCoarseTriples, 5);
  EXPECT_NEAR(job.mCoarseSigma, std::log(scale), 1e-3);
  EXPECT_LT(std::acos(std::min(1.0, job.mCoarseDir.dot(
                                        basalt::constants::g.normalized()))),
            1e-3);
  EXPECT_LT((job.mCoarseBa - ba).norm(), 1e-3);
}
```

Backwards compatibility. The test is added beside the three of Change 10 and touches no existing test.

#### Change 16. The coarse optimisation after DM-VIO, with the readvance in the same job, `include/basalt/vi_estimator/delayed_marginalisation.h`, `src/vi_estimator/sqrt_keypoint_vio.cpp`, the `vio_delayed_marg` record and the unit tests

Note, 2026-11-14. Change 17 replaced the coarse optimisation of this change by DM-VIO's form, and the code below is kept as the record of the unbiased form that §2.3.11 Step 8 compares with. The type abbreviations, `DelayedMargValues::ApplyInc`, `DelayedMargSetBlock` and the readvance in the same job remain as written here.

Change 16 replaces the coarse optimisation of Change 13 and its installation in Change 14 by the inertial only estimate of §2.3.11 and the readvance of §2.3.14, and it moves every type abbreviation of the store's header to the top of the file. No configuration key changes. The constants `kCoarseAccelStd` and `kCoarseMinDisplacement` and the output `mCoarseTriples` are removed, and `kCoarsePositionStd`, `mvCoarseVelInc`, `mCoarseMaxVelInc`, `mCoarseVarFactor` and `mCoarseSpans` are added.

Type abbreviations. Basalt has no package wide header of abbreviations for its `Scalar` templated Eigen and Sophus types, since each class declares its own, so the abbreviations of `delayed_marginalisation.h` are declared once at the top of the file as alias templates in the namespace `basalt::dm`, which keeps them out of `basalt` for every file that includes the header, and every struct refers to them as `dm::Vec3<Scalar>` and so on (`/ws/skills/cpp-type-aliases.md`). The structs keep only `using Scalar = Scalar_;`, which exposes their template parameter, and `DelayedMarginalisation` names `DelayedMargStep<Scalar>` and `DelayedMargJob<Scalar>` directly instead of its former `Step` and `Job` aliases. No code outside the header used the removed member aliases, which a search of `src`, `include` and `test` confirmed on 2026-11-14.

```cpp
template <class Scalar_>
struct DelayedMargStep;

// Type abbreviations of this file. Basalt has no package wide abbreviations
// of Scalar templated types, so they are scoped to this file's namespace.
namespace dm {
template <class Scalar>
using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
template <class Scalar>
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
template <class Scalar>
using Vec6 = Eigen::Matrix<Scalar, 6, 1>;
template <class Scalar>
using Vec15 = Eigen::Matrix<Scalar, POSE_VEL_BIAS_SIZE, 1>;
template <class Scalar>
using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
template <class Scalar>
using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
template <class Scalar>
using Mat6 = Eigen::Matrix<Scalar, 6, 6>;
template <class Scalar>
using Mat32 = Eigen::Matrix<Scalar, 3, 2>;
template <class Scalar>
using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
template <class Scalar>
using SO3 = Sophus::SO3<Scalar>;
template <class Scalar>
using StepPtr = std::shared_ptr<const DelayedMargStep<Scalar>>;
}  // namespace dm
```

The values gain one method that adds an increment to whichever state holds a variable's velocity and biases, and the block move of the former `InstallCoarseJob` becomes a free function shared by the job and the estimator.

```cpp
    // Adds x to the state that holds the velocity and biases of id, the
    // window state while it is there and the record otherwise.
    void ApplyInc(int64_t id, const dm::Vec15<Scalar>& x) {
        const auto it_s = mvWindowStates.find(id);
        if (it_s != mvWindowStates.end()) {
            it_s->second.applyInc(x);
            return;
        }
        const auto it_r = mvRecords.find(id);
        if (it_r != mvRecords.end()) it_r->second.mState.applyInc(x);
    }
```

```cpp
// Moves the block to the gravity direction dir and log scale sigma.
template <class Scalar>
void DelayedMargSetBlock(GravityScaleStateWithLin<Scalar>& gs,
                         const dm::Vec3<Scalar>& dir, Scalar sigma) {
    gs.Reanchor(gs.GetDir(), gs.GetBasis(), gs.GetSigma());
    const dm::Vec3<Scalar> axis = gs.GetDir().cross(dir);
    const Scalar sinAngle = axis.norm();
    dm::Vec3<Scalar> w = dm::Vec3<Scalar>::Zero();
    if (sinAngle > Sophus::Constants<Scalar>::epsilon()) {
        w = axis * (std::atan2(sinAngle, gs.GetDir().dot(dir)) / sinAngle);
    }
    dm::Vec3<Scalar> inc;
    inc.template head<2>() = gs.mBasisLin.transpose() * w;
    inc[2] = sigma - gs.GetSigma();
    gs.ApplyInc(inc);
}
```

The job's coarse inputs and outputs.

```cpp
    // Coarse inertial optimisation followed by a readvance at its result,
    // §2.3.11. The inertial factors of the window, the initial gravity
    // direction in the epoch mEpoch, and the information of the tilt and
    // accelerometer bias priors.
    bool mCoarse = false;
    Eigen::aligned_vector<IntegratedImuMeasurement<Scalar>> mvWindowImu;
    dm::Vec3<Scalar> mInitialDir = dm::Vec3<Scalar>::Zero();
    Scalar mTiltPriorW = 0;
    Scalar mBaPriorW = 0;
    Scalar mScaleVarMax = 0;
    int mNodeSpacing = 1;
    // Standard deviation of a held node position, §2.3.11 Step 4.
    static constexpr Scalar kCoarsePositionStd = Scalar(0.02);

    // Outputs.
    DelayedMargPrior<Scalar> mResult;
    // Smoothing increment of each departed variable, and which of its 15
    // coordinates it covers.
    Eigen::aligned_map<int64_t, std::pair<dm::Vec15<Scalar>, uint32_t>>
        mvSmoothInc;
    Scalar mMaxPosInc = 0;
    double mSeconds = 0;
    // Coarse outputs, the block and the velocity increments in the epoch
    // mEpoch, and the common bias increments.
    dm::Vec3<Scalar> mCoarseDir = dm::Vec3<Scalar>::Zero();
    Scalar mCoarseSigma = 0;
    dm::Vec3<Scalar> mCoarseBg = dm::Vec3<Scalar>::Zero();
    dm::Vec3<Scalar> mCoarseBa = dm::Vec3<Scalar>::Zero();
    Eigen::aligned_map<int64_t, dm::Vec3<Scalar>> mvCoarseVelInc;
    Scalar mCoarseMaxVelInc = 0;
    Scalar mCoarseScaleVar = 0;
    Scalar mCoarseVarFactor = 1;
    bool mCoarseScaleFree = false;
    Scalar mCoarseCost0 = 0;
    Scalar mCoarseCost1 = 0;
    int mCoarseFactors = 0;
    int mCoarseSpans = 0;
```

`Run` readvances after a coarse optimisation that found inertial factors, and `Coarse` is the estimate of §2.3.11 Steps 2 to 7.

```cpp
    // Readvance, preceded by one smoothing pass when mSmooth is set, or by
    // the coarse optimisation when mCoarse is set.
    void Run() {
        const auto t0 = std::chrono::steady_clock::now();
        if (mCoarse) {
            Coarse();
            if (mCoarseSpans > 0) Forward(nullptr);
        } else {
            if (mSmooth) {
                std::vector<Elim> elims;
                Forward(&elims);
                Backward(elims);
            }
            Forward(nullptr);
        }
        mSeconds = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - t0)
                       .count();
    }

    // Inertial only maximum a posteriori estimate of the gravity direction,
    // the log scale, common gyroscope and accelerometer bias increments and
    // the velocities of nodes mNodeSpacing factors apart, from the inertial
    // factors of the store and the window with every pose held, after the
    // coarse initialiser of DM-VIO and the inertial only optimisation of
    // ORB-SLAM3. The factors are chained between nodes in the world frame
    // of the held rotations, the visual displacements are the predicted
    // side, and the scale is held when its marginal variance, inflated by
    // the a posteriori variance factor, exceeds mScaleVarMax. The result is
    // applied to mValues and mGs, §2.3.11.
    void Coarse() {
        // One per frame factor of a span, for the velocity propagation.
        struct Factor {
            int64_t mT1 = 0;
            Scalar mDt = 0;
            dm::Mat3<Scalar> mR = dm::Mat3<Scalar>::Identity();
            dm::Vec3<Scalar> mDv = dm::Vec3<Scalar>::Zero();
            dm::Mat3<Scalar> mDvBa = dm::Mat3<Scalar>::Zero();
            dm::Mat3<Scalar> mDvBg = dm::Mat3<Scalar>::Zero();
        };
        // Chained inertial quantities from node mA to node mB in the world
        // frame, their bias derivatives and the covariance of [P; V].
        struct Span {
            int64_t mA = 0, mB = 0;
            dm::Vec3<Scalar> mPa = dm::Vec3<Scalar>::Zero();
            dm::Vec3<Scalar> mPb = dm::Vec3<Scalar>::Zero();
            Scalar mT = 0;
            dm::Vec3<Scalar> mP = dm::Vec3<Scalar>::Zero();
            dm::Vec3<Scalar> mV = dm::Vec3<Scalar>::Zero();
            dm::Mat3<Scalar> mPBa = dm::Mat3<Scalar>::Zero();
            dm::Mat3<Scalar> mPBg = dm::Mat3<Scalar>::Zero();
            dm::Mat3<Scalar> mVBa = dm::Mat3<Scalar>::Zero();
            dm::Mat3<Scalar> mVBg = dm::Mat3<Scalar>::Zero();
            dm::Mat6<Scalar> mCov = dm::Mat6<Scalar>::Zero();
            dm::Mat6<Scalar> mL = dm::Mat6<Scalar>::Identity();
            Eigen::aligned_vector<Factor> mvFactors;
        };
        // Rotation row of one factor, linear in the gyroscope bias increment.
        struct RotRow {
            dm::Vec3<Scalar> mR = dm::Vec3<Scalar>::Zero();
            dm::Mat3<Scalar> mJ = dm::Mat3<Scalar>::Zero();
            dm::Mat3<Scalar> mW = dm::Mat3<Scalar>::Identity();
        };

        std::map<int64_t, const IntegratedImuMeasurement<Scalar>*> factors;
        auto add = [&](const IntegratedImuMeasurement<Scalar>& meas) {
            const int64_t t0 = meas.get_start_t_ns();
            if (Known(t0) && Known(t0 + meas.get_dt_ns())) factors[t0] = &meas;
        };
        for (const dm::StepPtr<Scalar>& step : mvSteps) {
            for (const auto& meas : step->mvImu) add(meas);
        }
        for (const auto& meas : mvWindowImu) add(meas);
        mCoarseFactors = int(factors.size());

        auto position = [&](int64_t id) {
            return dm::Vec3<Scalar>(
                mValues.State(id).getState().T_w_i.translation());
        };

        Eigen::aligned_vector<Span> vSpans;
        Eigen::aligned_vector<RotRow> vRotRows;
        Span span;
        int inSpan = 0;
        int64_t tNext = -1;
        auto close = [&](int64_t tEnd) {
            span.mB = tEnd;
            span.mPb = position(tEnd);
            vSpans.push_back(span);
            inSpan = 0;
        };
        for (const auto& [t0, meas] : factors) {
            if (t0 != tNext && inSpan > 0) close(tNext);
            if (inSpan == 0) {
                span = Span();
                span.mA = t0;
                span.mPa = position(t0);
            }
            const int64_t t1 = t0 + meas->get_dt_ns();
            const PoseVelBiasState<Scalar> s0 = mValues.State(t0).getState();
            const PoseVelBiasState<Scalar> s1 = mValues.State(t1).getState();

            // Bias corrected deltas and their bias derivatives, read from the
            // residual at identity states, zero gravity and zero scale.
            typename IntegratedImuMeasurement<Scalar>::MatN3 dBa, dBg;
            const typename PoseVelState<Scalar>::VecN res = meas->residualGS(
                PoseVelState<Scalar>(), dm::Vec3<Scalar>::Zero(),
                PoseVelState<Scalar>(), s0.bias_gyro, s0.bias_accel, Scalar(0),
                nullptr, nullptr, &dBg, &dBa);
            const dm::Mat3<Scalar> R = s0.T_w_i.so3().matrix();
            const Scalar dt = Scalar(meas->get_dt_ns()) * Scalar(1e-9);

            // Covariance of [P; V] after this factor, from that of the deltas.
            dm::Mat6<Scalar> F = dm::Mat6<Scalar>::Identity();
            F.template topRightCorner<3, 3>().diagonal().setConstant(dt);
            const auto& cov = meas->get_cov();
            dm::Mat6<Scalar> C;
            C.template topLeftCorner<3, 3>() = cov.template block<3, 3>(0, 0);
            C.template topRightCorner<3, 3>() = cov.template block<3, 3>(0, 6);
            C.template bottomLeftCorner<3, 3>() =
                cov.template block<3, 3>(6, 0);
            C.template bottomRightCorner<3, 3>() =
                cov.template block<3, 3>(6, 6);
            dm::Mat6<Scalar> G = dm::Mat6<Scalar>::Zero();
            G.template topLeftCorner<3, 3>() = R;
            G.template bottomRightCorner<3, 3>() = R;
            span.mCov = F * span.mCov * F.transpose() + G * C * G.transpose();

            span.mP += span.mV * dt - R * res.template head<3>();
            span.mPBa += span.mVBa * dt - R * dBa.template topRows<3>();
            span.mPBg += span.mVBg * dt - R * dBg.template topRows<3>();
            span.mV -= R * res.template tail<3>();
            span.mVBa -= R * dBa.template bottomRows<3>();
            span.mVBg -= R * dBg.template bottomRows<3>();
            span.mT += dt;

            Factor f;
            f.mT1 = t1;
            f.mDt = dt;
            f.mR = R;
            f.mDv = -res.template tail<3>();
            f.mDvBa = -dBa.template bottomRows<3>();
            f.mDvBg = -dBg.template bottomRows<3>();
            span.mvFactors.push_back(f);

            // The rotation row at the held rotations.
            typename IntegratedImuMeasurement<Scalar>::MatN3 dRBg;
            const typename PoseVelState<Scalar>::VecN resR = meas->residualGS(
                s0, dm::Vec3<Scalar>::Zero(), s1, s0.bias_gyro, s0.bias_accel,
                Scalar(0), nullptr, nullptr, &dRBg, nullptr);
            RotRow row;
            row.mR = resR.template segment<3>(3);
            row.mJ = dRBg.template block<3, 3>(3, 0);
            const dm::Mat3<Scalar> covR = cov.template block<3, 3>(3, 3);
            row.mW = covR.inverse();
            if (row.mW.allFinite()) vRotRows.push_back(row);

            tNext = t1;
            if (++inSpan == mNodeSpacing) close(tNext);
        }
        if (inSpan > 0) close(tNext);
        mCoarseSpans = int(vSpans.size());

        const dm::Vec3<Scalar> dir0 = mGs.GetDir();
        const dm::Mat32<Scalar> basis0 = mGs.GetBasis();
        const Scalar sigma0 = mGs.GetSigma();
        mCoarseDir = dir0;
        mCoarseSigma = sigma0;
        mCoarseBg.setZero();
        mCoarseBa.setZero();
        mvCoarseVelInc.clear();
        mCoarseMaxVelInc = 0;
        mCoarseScaleFree = false;
        mCoarseScaleVar = 0;
        mCoarseVarFactor = 1;
        if (vSpans.empty()) return;

        // Unknowns [tilt (2), inverse scale increment, accelerometer bias (3),
        // gyroscope bias (3), node velocities (3 each)].
        std::map<int64_t, int> nodeIdx;
        for (const Span& sp : vSpans) {
            nodeIdx.emplace(sp.mA, 0);
            nodeIdx.emplace(sp.mB, 0);
        }
        const int nNodes = int(nodeIdx.size());
        const int n = 9 + 3 * nNodes;
        dm::VecX<Scalar> v0(3 * nNodes);
        {
            int i = 0;
            for (auto& [id, idx] : nodeIdx) {
                idx = i;
                v0.template segment<3>(3 * i) =
                    mValues.State(id).getState().vel_w_i;
                i++;
            }
        }

        // Whitening in visual units, the inertial covariance at the scale of
        // the snapshot and the noise of the two held positions.
        const Scalar s0 = std::exp(sigma0);
        const Scalar e0 = Scalar(1) / s0;
        for (Span& sp : vSpans) {
            dm::Mat6<Scalar> S = sp.mCov / (s0 * s0);
            S.template topLeftCorner<3, 3>().diagonal().array() +=
                Scalar(2) * kCoarsePositionStd * kCoarsePositionStd;
            sp.mL = S.llt().matrixL();
        }
        const Scalar gNorm = mG.norm();

        dm::SO3<Scalar> R;
        // Relative increment of the inverse scale, e = e0 (1 + eta), in which
        // every inertial row is linear.
        Scalar eta = 0;
        dm::Vec3<Scalar> dBa = dm::Vec3<Scalar>::Zero();
        dm::Vec3<Scalar> dBg = dm::Vec3<Scalar>::Zero();
        dm::VecX<Scalar> dv = dm::VecX<Scalar>::Zero(3 * nNodes);
        Scalar costTrans = 0;

        auto tiltResidual = [&](const dm::Vec3<Scalar>& dir,
                                const dm::Mat32<Scalar>& basis) {
            const dm::Vec3<Scalar> axis = mInitialDir.cross(dir);
            const Scalar sinAngle = axis.norm();
            dm::Vec3<Scalar> w = dm::Vec3<Scalar>::Zero();
            if (sinAngle > Sophus::Constants<Scalar>::epsilon()) {
                w = axis *
                    (std::atan2(sinAngle, mInitialDir.dot(dir)) / sinAngle);
            }
            return dm::Vec2<Scalar>(basis.transpose() * w);
        };

        auto linearise = [&](dm::MatX<Scalar>* H, dm::VecX<Scalar>* b) {
            const dm::Vec3<Scalar> dir = R * dir0;
            const dm::Mat32<Scalar> basis = R.matrix() * basis0;
            const dm::Vec3<Scalar> gD = gNorm * dir;
            const Scalar e = e0 * (Scalar(1) + eta);
            const dm::Mat32<Scalar> dGd = -dm::SO3<Scalar>::hat(gD) * basis;
            Scalar cost = 0;
            costTrans = 0;
            if (H) {
                H->setZero(n, n);
                b->setZero(n);
            }
            dm::MatX<Scalar> J(6, n);
            for (const Span& sp : vSpans) {
                const int ia = 9 + 3 * nodeIdx.at(sp.mA);
                const int ib = 9 + 3 * nodeIdx.at(sp.mB);
                const dm::Vec3<Scalar> va = v0.template segment<3>(ia - 9) +
                                dv.template segment<3>(ia - 9);
                const dm::Vec3<Scalar> vb = v0.template segment<3>(ib - 9) +
                                dv.template segment<3>(ib - 9);
                const Scalar T = sp.mT;
                const dm::Vec3<Scalar> imuP = Scalar(0.5) * gD * T * T + sp.mP +
                                  sp.mPBa * dBa + sp.mPBg * dBg;
                const dm::Vec3<Scalar> imuV = gD * T + sp.mV + sp.mVBa * dBa +
                                  sp.mVBg * dBg;
                dm::Vec6<Scalar> r;
                r.template head<3>() = sp.mPb - sp.mPa - va * T - e * imuP;
                r.template tail<3>() = vb - va - e * imuV;
                const dm::Vec6<Scalar> rw =
                    sp.mL.template triangularView<Eigen::Lower>().solve(r);
                costTrans += Scalar(0.5) * rw.squaredNorm();
                if (!H) continue;
                J.setZero();
                J.template block<3, 2>(0, 0) = -e * Scalar(0.5) * T * T * dGd;
                J.template block<3, 2>(3, 0) = -e * T * dGd;
                J.template block<3, 1>(0, 2) = -e0 * imuP;
                J.template block<3, 1>(3, 2) = -e0 * imuV;
                J.template block<3, 3>(0, 3) = -e * sp.mPBa;
                J.template block<3, 3>(3, 3) = -e * sp.mVBa;
                J.template block<3, 3>(0, 6) = -e * sp.mPBg;
                J.template block<3, 3>(3, 6) = -e * sp.mVBg;
                J.template block<3, 3>(0, ia).diagonal().setConstant(-T);
                J.template block<3, 3>(3, ia).diagonal().setConstant(-1);
                J.template block<3, 3>(3, ib).diagonal().array() += Scalar(1);
                const dm::MatX<Scalar> Jw =
                    sp.mL.template triangularView<Eigen::Lower>().solve(J);
                *H += Jw.transpose() * Jw;
                *b += Jw.transpose() * rw;
            }
            cost += costTrans;
            for (const RotRow& row : vRotRows) {
                const dm::Vec3<Scalar> r = row.mR + row.mJ * dBg;
                cost += Scalar(0.5) * r.dot(row.mW * r);
                if (!H) continue;
                H->template block<3, 3>(6, 6) +=
                    row.mJ.transpose() * row.mW * row.mJ;
                b->template segment<3>(6) += row.mJ.transpose() * row.mW * r;
            }
            if (mTiltPriorW > 0) {
                const dm::Vec2<Scalar> rt = tiltResidual(dir, basis);
                cost += Scalar(0.5) * mTiltPriorW * rt.squaredNorm();
                if (H) {
                    H->template block<2, 2>(0, 0).diagonal().array() +=
                        mTiltPriorW;
                    b->template head<2>() += mTiltPriorW * rt;
                }
            }
            if (mBaPriorW > 0) {
                cost += Scalar(0.5) * mBaPriorW * dBa.squaredNorm();
                if (H) {
                    H->template block<3, 3>(3, 3).diagonal().array() +=
                        mBaPriorW;
                    b->template segment<3>(3) += mBaPriorW * dBa;
                }
            }
            return cost;
        };

        // Levenberg-Marquardt from the snapshot, with the scale free or held.
        dm::MatX<Scalar> H;
        dm::VecX<Scalar> b;
        auto solve = [&](bool scaleFree) {
            R = dm::SO3<Scalar>();
            eta = 0;
            dBa.setZero();
            dBg.setZero();
            dv.setZero();
            Scalar cost = linearise(&H, &b);
            Scalar lambda = Scalar(1e-4);
            for (int it = 0; it < 30 && lambda < Scalar(1e8); it++) {
                dm::MatX<Scalar> Hd = H;
                Hd.diagonal() += lambda * H.diagonal();
                Hd.diagonal().array() += Scalar(1e-9);
                dm::VecX<Scalar> bd = b;
                if (!scaleFree) {
                    Hd.row(2).setZero();
                    Hd.col(2).setZero();
                    Hd(2, 2) = 1;
                    bd[2] = 0;
                }
                const dm::VecX<Scalar> x = -Hd.ldlt().solve(bd);
                if (!x.allFinite()) break;
                const dm::SO3<Scalar> R0 = R;
                const Scalar eta0 = eta;
                const dm::Vec3<Scalar> dBa0 = dBa, dBg0 = dBg;
                const dm::VecX<Scalar> dv0 = dv;
                R = dm::SO3<Scalar>::exp(R.matrix() * basis0 *
                                         x.template head<2>()) *
                    R;
                eta += x[2];
                dBa += x.template segment<3>(3);
                dBg += x.template segment<3>(6);
                dv += x.tail(3 * nNodes);
                const Scalar costNew = linearise(nullptr, nullptr);
                if (eta > Scalar(-1) && std::isfinite(double(costNew)) &&
                    costNew < cost) {
                    const bool converged =
                        cost - costNew < Scalar(1e-6) * cost;
                    cost = linearise(&H, &b);
                    lambda = std::max(lambda / Scalar(10), Scalar(1e-7));
                    if (converged) break;
                } else {
                    R = R0;
                    eta = eta0;
                    dBa = dBa0;
                    dBg = dBg0;
                    dv = dv0;
                    lambda *= Scalar(10);
                }
            }
            linearise(&H, &b);
            H.diagonal().array() += Scalar(1e-9);
            return cost;
        };

        // The marginal variance of the log scale at the snapshot, where it
        // reflects the excitation, since at a degenerate free solution a large
        // inverse scale shrinks it. eta is zero there, so the variance of eta
        // is that of the log scale.
        mCoarseCost0 = linearise(&H, &b);
        H.diagonal().array() += Scalar(1e-9);
        const Scalar scaleVar0 = H.inverse()(2, 2);
        mCoarseCost1 = solve(true);
        // A posteriori variance factor of the translational rows, whose
        // unknowns are the velocities, the tilt, the scale and the
        // accelerometer bias, §2.3.11 Step 6.
        const int redundancy = 6 * mCoarseSpans - (3 * nNodes + 6);
        if (redundancy > 0) {
            mCoarseVarFactor = std::max(
                Scalar(1), Scalar(2) * costTrans / Scalar(redundancy));
        }
        mCoarseScaleVar = scaleVar0 * mCoarseVarFactor;
        mCoarseScaleFree = std::isfinite(double(mCoarseScaleVar)) &&
                           mCoarseScaleVar <= mScaleVarMax;
        if (!mCoarseScaleFree) mCoarseCost1 = solve(false);

        mCoarseDir = R * dir0;
        mCoarseSigma = sigma0 - std::log(Scalar(1) + eta);
        mCoarseBg = dBg;
        mCoarseBa = dBa;

        // Velocities of the nodes, and of the states between them propagated
        // from the node at the start of their span.
        const dm::Vec3<Scalar> gD = gNorm * mCoarseDir;
        const Scalar e = std::exp(-mCoarseSigma);
        std::map<int64_t, dm::Vec3<Scalar>> vel;
        for (const Span& sp : vSpans) {
            const int ia = nodeIdx.at(sp.mA);
            dm::Vec3<Scalar> v = v0.template segment<3>(3 * ia) +
                     dv.template segment<3>(3 * ia);
            vel[sp.mA] = v;
            for (size_t i = 0; i + 1 < sp.mvFactors.size(); i++) {
                const Factor& f = sp.mvFactors[i];
                v += e * (gD * f.mDt +
                          f.mR * (f.mDv + f.mDvBa * dBa + f.mDvBg * dBg));
                vel[f.mT1] = v;
            }
            const int ib = nodeIdx.at(sp.mB);
            vel[sp.mB] = v0.template segment<3>(3 * ib) +
                         dv.template segment<3>(3 * ib);
        }
        for (const auto& [id, v] : vel) {
            const dm::Vec3<Scalar> inc =
                v - mValues.State(id).getState().vel_w_i;
            if (!inc.allFinite()) continue;
            mvCoarseVelInc[id] = inc;
            mCoarseMaxVelInc = std::max(mCoarseMaxVelInc, inc.norm());
        }

        // The readvance that follows sees the corrected values.
        auto apply = [&](int64_t id) {
            dm::Vec15<Scalar> x = dm::Vec15<Scalar>::Zero();
            const auto it = mvCoarseVelInc.find(id);
            if (it != mvCoarseVelInc.end()) {
                x.template segment<3>(6) = it->second;
            }
            x.template segment<3>(9) = mCoarseBg;
            x.template segment<3>(12) = mCoarseBa;
            mValues.ApplyInc(id, x);
        };
        for (const auto& [id, st] : mValues.mvWindowStates) apply(id);
        for (const auto& [id, rec] : mValues.mvRecords) {
            if (mValues.mvWindowStates.count(id) == 0) apply(id);
        }
        DelayedMargSetBlock(mGs, mCoarseDir, mCoarseSigma);
    }
```

`InstallCoarseJob` returns whether there was anything to install, applies the velocity increments mapped to the current epoch, and leaves the prior to `InstallDelayedJob`, which the install of `UpdateDelayedMarg` now calls for a coarse job as well. The coarse install no longer sets `mpDmForceReplace`, since the job already readvanced, and the declaration in `sqrt_keypoint_vio.h` changes its return type to `bool`.

```cpp
// Applies a coarse optimisation as a re-centring of the window, the priors and
// the store, and its velocity and common bias increments to every state, so
// that the prior the job readvanced at these values can be installed, §2.3.9.
// Returns false when the job had no inertial factor to fit.
template <class Scalar_>
bool SqrtKeypointVioEstimator<Scalar_>::InstallCoarseJob(
    const DelayedMargJob<Scalar>& job) {
    if (job.mCoarseSpans == 0) return false;
    const GravityScaleCorrectionLog::Entry c =
        this->mGsCorrectionLog->Between(
            job.mEpoch, this->mGsCorrectionLog->LatestEpoch());
    const Vec3 dir = c.mQ.template cast<Scalar>() * job.mCoarseDir;
    const Scalar sigma = job.mCoarseSigma - Scalar(std::log(c.mScale));
    DelayedMargSetBlock(*this->mGsState, dir, sigma);
    RecentreGravityScale();
    AnchorGs(mpAnchorCurr);

    // The velocity increments were solved in the epoch of the snapshot.
    const GravityScaleCorrectionLog::Entry cNow =
        this->mGsCorrectionLog->Between(
            job.mEpoch, this->mGsCorrectionLog->LatestEpoch());
    const Mat3 Qm = cNow.mQ.template cast<Scalar>().matrix();
    const Scalar s = Scalar(cNow.mScale);
    auto inc = [&](int64_t id) {
        typename PoseVelBiasStateWithLin<Scalar>::VecN x;
        x.setZero();
        const auto it = job.mvCoarseVelInc.find(id);
        if (it != job.mvCoarseVelInc.end()) {
            x.template segment<3>(6) = s * (Qm * it->second);
        }
        x.template segment<3>(9) = job.mCoarseBg;
        x.template segment<3>(12) = job.mCoarseBa;
        return x;
    };
    for (auto& [id, state] : frame_states) state.applyInc(inc(id));
    for (auto& [id, rec] : mpDm->Values().mvRecords) {
        if (frame_states.count(id) == 0) rec.mState.applyInc(inc(id));
    }
    return true;
}
```

```cpp
    auto install = [&](const std::shared_ptr<DelayedMargJob<Scalar>>& job) {
        Timer t;
        if (job->mCoarse) {
            coarseJob = job.get();
            coarse = 1;
            if (InstallCoarseJob(*job)) {
                catchup = InstallDelayedJob(*job);
                installed = 1;
                mpGsSwapEvent = 5;
            }
        } else {
            catchup = InstallDelayedJob(*job);
            maxPosInc = job->mMaxPosInc;
            installed = 1;
            mpGsSwapEvent = 6;
        }
        catchupSeconds = t.elapsed();
        jobSeconds = job->mSeconds;
        doneJob = job;
    };
```

The record `vio_delayed_marg` replaces `coarse_triples` with `coarse_spans` and gains the norm of the gyroscope bias increment, the a posteriori variance factor and the largest velocity increment. `Logger::AddVioDelayedMarg` in `include/basalt/utils/logger.h` and `src/utils/logger.cpp` takes the four values in place of `coarseTriples`.

```cpp
              {"coarse_phase", "count"}, {"coarse_sigma", "f"},
              {"coarse_tilt_deg", "f"}, {"coarse_scale_var", "f"},
              {"coarse_scale_free", "count"}, {"coarse_ba", "f"},
              {"coarse_cost0", "f"}, {"coarse_cost1", "f"},
              {"coarse_spans", "count"}, {"coarse_bg", "f"},
              {"coarse_var_factor", "f"}, {"coarse_max_vel_inc", "f"}});
```

```cpp
    mpLogger->AddVioDelayedMarg(
        last_state_t_ns, steps, records, started, installed, coarse,
        double(anchorSigma), double(anchorTilt) * 180.0 / M_PI, jobSeconds,
        catchup, catchupSeconds, double(maxPosInc), storeSeconds,
        mpDmCoarse ? 1 : 0, coarseJob ? double(coarseJob->mCoarseSigma) : 0.0,
        coarseJob ? double(std::acos(std::clamp(
                        coarseJob->mCoarseDir.dot(coarseJob->mGs.GetDir()),
                        Scalar(-1), Scalar(1)))) *
                        180.0 / M_PI
                  : 0.0,
        coarseJob ? double(coarseJob->mCoarseScaleVar) : 0.0,
        coarseJob ? int(coarseJob->mCoarseScaleFree) : 0,
        coarseJob ? coarseJob->mCoarseBa.norm() : 0.0,
        coarseJob ? double(coarseJob->mCoarseCost0) : 0.0,
        coarseJob ? double(coarseJob->mCoarseCost1) : 0.0,
        coarseJob ? coarseJob->mCoarseSpans : 0,
        coarseJob ? double(coarseJob->mCoarseBg.norm()) : 0.0,
        coarseJob ? double(coarseJob->mCoarseVarFactor) : 0.0,
        coarseJob ? double(coarseJob->mCoarseMaxVelInc) : 0.0);
}
```

The unit tests replace the test of Change 15. A helper builds a job over a random spline trajectory with knots 1 s apart, positions and velocities shrunk by a known scale, velocities perturbed by a constant, positions jittered, inertial factors carrying both biases, and the noise densities of `data/sitl_calib.json`. The first test checks the scale, the gravity direction, both biases and every velocity on exact data, and the second checks that a position jitter of twice `kCoarsePositionStd` leaves the scale within 0.05 of its true value and raises the variance factor above 1.5.

```cpp
// A coarse job over a random trajectory whose positions and velocities are
// shrunk by scale, whose velocities are perturbed by velError and whose
// positions carry a jitter of jitterStd, with inertial factors that carry the
// biases ba and bg, plans/delayed_marginalisation.md §2.3.11.
static basalt::DelayedMargJob<double> MakeCoarseTestJob(
    const basalt::Se3Spline<5>& spline, double scale, double velError,
    double jitterStd, const Eigen::Vector3d& ba, const Eigen::Vector3d& bg) {
  constexpr int N = 41;
  constexpr int64_t kDt = 50000000;
  std::mt19937 gen(3);
  std::normal_distribution<double> noise(0.0, 1.0);

  basalt::DelayedMargJob<double> job;
  for (int i = 0; i < N; i++) {
    const int64_t t = i * kDt;
    Sophus::SE3d pose = spline.pose(t);
    pose.translation() /= scale;
    pose.translation() +=
        jitterStd *
        Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
    const Eigen::Vector3d vel =
        spline.transVelWorld(t) / scale + Eigen::Vector3d::Constant(velError);
    basalt::PoseVelBiasStateWithLin<double> st(
        t, pose, vel, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), true);
    if (i == N - 1) {
      job.mValues.mvWindowStates[t] = st;
    } else {
      job.mValues.mvRecords[t].mState = st;
    }
  }
  for (int i = 0; i + 1 < N; i++) {
    basalt::IntegratedImuMeasurement<double> meas(
        i * kDt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    for (int64_t t = i * kDt + 2500000; t < (i + 1) * kDt; t += 5000000) {
      basalt::ImuData<double> data;
      const Sophus::SE3d pose = spline.pose(t);
      data.accel = pose.so3().inverse() *
                       (spline.transAccelWorld(t) - basalt::constants::g) +
                   ba;
      data.gyro = spline.rotVelBody(t) + bg;
      data.t_ns = t + 2500000;
      // The noise densities of data/sitl_calib.json at 200 Hz.
      meas.integrate(data, Eigen::Vector3d::Constant(1.28),
                     Eigen::Vector3d::Constant(1.1e-3));
    }
    auto step = std::make_shared<basalt::DelayedMargStep<double>>();
    step->mvImu.push_back(meas);
    job.mvSteps.push_back(step);
  }

  job.mG = basalt::constants::g;
  job.mInitialDir = basalt::constants::g.normalized();
  job.mScaleVarMax = 1.0;
  job.mNodeSpacing = 4;
  job.mCoarse = true;
  return job;
}

// The coarse optimisation recovers the scale, the gravity direction, both
// biases and the velocities of exact data, plans/delayed_marginalisation.md
// §2.3.11.
TEST(VioTestSuite, DelayedMargCoarseScaleTest) {
  basalt::Se3Spline<5> spline(int64_t(1e9));
  spline.genRandomTrajectory(15);
  const double scale = 1.6;
  const Eigen::Vector3d ba(0.05, -0.03, 0.02);
  const Eigen::Vector3d bg(0.004, -0.003, 0.002);

  basalt::DelayedMargJob<double> job =
      MakeCoarseTestJob(spline, scale, 0.3, 0.0, ba, bg);
  job.Coarse();

  EXPECT_TRUE(job.mCoarseScaleFree);
  EXPECT_EQ(job.mCoarseSpans, 10);
  EXPECT_NEAR(job.mCoarseSigma, std::log(scale), 1e-3);
  EXPECT_LT(std::acos(std::min(1.0, job.mCoarseDir.dot(
                                        basalt::constants::g.normalized()))),
            1e-3);
  EXPECT_LT((job.mCoarseBa - ba).norm(), 1e-3);
  EXPECT_LT((job.mCoarseBg - bg).norm(), 1e-4);
  for (int i = 0; i < 41; i++) {
    const int64_t t = i * int64_t(50000000);
    EXPECT_LT((job.mValues.State(t).getState().vel_w_i -
               spline.transVelWorld(t) / scale)
                  .norm(),
              1e-2);
  }
}

// With a held position jitter of twice kCoarsePositionStd the scale stays
// unbiased, because the visual displacements are the predicted side, and the
// a posteriori variance factor exceeds one.
TEST(VioTestSuite, DelayedMargCoarseJitterTest) {
  basalt::Se3Spline<5> spline(int64_t(1e9));
  spline.genRandomTrajectory(15);
  const double scale = 1.6;

  basalt::DelayedMargJob<double> job =
      MakeCoarseTestJob(spline, scale, 0.0, 0.04, Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero());
  job.Coarse();

  EXPECT_TRUE(job.mCoarseScaleFree);
  EXPECT_NEAR(job.mCoarseSigma, std::log(scale), 0.05);
  EXPECT_GT(job.mCoarseVarFactor, 1.5);
}
```

Backwards compatibility. Every change is confined to the gravity and scale period with the delayed store, which is the period's only mode, so nothing changes when `vio_gs_enable` is false. Within the period the coarse phase now corrects the velocities and the gyroscope bias as well, installs the readvanced prior in the frame of the correction instead of one or two frames later, and holds the scale before lift off by its variance rather than by the displacement threshold. The delayed phase is unchanged. The column `coarse_triples` of `vio_delayed_marg.csv` is renamed `coarse_spans`, and no script under `scripts/analysis` reads any `coarse_` column, as a search confirmed on 2026-11-14. The logger's signature change reaches only `UpdateDelayedMarg`.

#### Change 17. The coarse optimisation as DM-VIO's coarse initialiser, triggered by keyframes and installed within one frame

Change 17 replaces the coarse optimisation of Change 16 by the form of §2.3.11, which matches DM-VIO's `CoarseIMUInitOptimizer` in its nodes, its factor, the placement of the scale, its bias, its solver, its trigger, its gate and the smoothing after success, and keeps three departures, the noise of the held positions, the linear scale increment and the gravity prior of 0.75°. The keyframe factors are composed once in the store as the per frame factors arrive, so a job reads only keyframe factors. It also makes every job of the store take effect one frame after its snapshot. The keys `vio_gs_dm_coarse_every` and `vio_ba_step_prior_std` are removed from `VioConfig`, from its loader and from the eight configuration files of Change 9, since the keyframes now set both the nodes and the cadence and DM-VIO's bias has no prior. The members `mNodeSpacing`, `mBaPriorW` and `mCoarseVarFactor` of the job and `mpDmLastCoarseIndex` of the estimator are removed.

The header gains four abbreviations at the top of the file.

```cpp
template <class Scalar>
using Vec9 = Eigen::Matrix<Scalar, 9, 1>;
template <class Scalar>
using Mat9 = Eigen::Matrix<Scalar, 9, 9>;
template <class Scalar>
using Mat93 = Eigen::Matrix<Scalar, 9, 3>;
template <class Scalar>
using SpMat = Eigen::SparseMatrix<Scalar>;
```

The per frame factor at zero bias, the keyframe factor composed from per frame factors, and the chain that builds keyframe factors as per frame factors arrive are file level types, shared by the store and the job.

```cpp
// One per frame inertial factor at zero bias, §2.3.11 Step 1. The deltas are
// read from the residual at identity states, zero gravity and zero scale,
// where the rotation row is Log(dR) and its gyroscope bias Jacobian carries
// the inverse left Jacobian of Log(dR), which is removed.
template <class Scalar_>
struct DelayedMargFrameFactor {
    using Scalar = Scalar_;

    int64_t mT0 = 0, mT1 = 0;
    Scalar mDt = 0;
    dm::SO3<Scalar> mDR;
    dm::Vec3<Scalar> mDp = dm::Vec3<Scalar>::Zero();
    dm::Vec3<Scalar> mDv = dm::Vec3<Scalar>::Zero();
    dm::Mat93<Scalar> mJg = dm::Mat93<Scalar>::Zero();
    dm::Mat93<Scalar> mJa = dm::Mat93<Scalar>::Zero();
    dm::Mat9<Scalar> mCov = dm::Mat9<Scalar>::Zero();

    explicit DelayedMargFrameFactor(
        const IntegratedImuMeasurement<Scalar>& meas) {
        typename IntegratedImuMeasurement<Scalar>::MatN3 dBa, dBg;
        const typename PoseVelState<Scalar>::VecN res = meas.residualGS(
            PoseVelState<Scalar>(), dm::Vec3<Scalar>::Zero(),
            PoseVelState<Scalar>(), dm::Vec3<Scalar>::Zero(),
            dm::Vec3<Scalar>::Zero(), Scalar(0), nullptr, nullptr, &dBg, &dBa);
        mT0 = meas.get_start_t_ns();
        mT1 = mT0 + meas.get_dt_ns();
        mDt = Scalar(meas.get_dt_ns()) * Scalar(1e-9);
        const dm::Vec3<Scalar> phi = res.template segment<3>(3);
        mDR = dm::SO3<Scalar>::exp(phi);
        mDp = -res.template head<3>();
        mDv = -res.template tail<3>();
        dm::Mat3<Scalar> Jl;
        Sophus::leftJacobianSO3(phi, Jl);
        mJg = -dBg;
        mJg.template middleRows<3>(3) = Jl * dBg.template middleRows<3>(3);
        mJa = -dBa;
        mCov = meas.get_cov();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// The inertial factor from keyframe mA to mB, composed from the per frame
// factors between them in the body frame of mA, with its bias Jacobians and
// covariance ordered [position, rotation, velocity], §2.3.11 Step 2.
template <class Scalar_>
struct DelayedMargKfSpan {
    using Scalar = Scalar_;

    int64_t mA = -1, mB = -1;
    Scalar mT = 0;
    dm::SO3<Scalar> mDR;
    dm::Vec3<Scalar> mDp = dm::Vec3<Scalar>::Zero();
    dm::Vec3<Scalar> mDv = dm::Vec3<Scalar>::Zero();
    dm::Mat93<Scalar> mJg = dm::Mat93<Scalar>::Zero();
    dm::Mat93<Scalar> mJa = dm::Mat93<Scalar>::Zero();
    dm::Mat9<Scalar> mCov = dm::Mat9<Scalar>::Zero();
    Eigen::aligned_vector<DelayedMargFrameFactor<Scalar>> mvFactors;

    void Append(const DelayedMargFrameFactor<Scalar>& f) {
        const dm::Mat3<Scalar> R = mDR.matrix();
        dm::Mat9<Scalar> A = dm::Mat9<Scalar>::Identity();
        A.template block<3, 3>(0, 3) = -dm::SO3<Scalar>::hat(R * f.mDp);
        A.template block<3, 3>(0, 6).diagonal().setConstant(f.mDt);
        A.template block<3, 3>(6, 3) = -dm::SO3<Scalar>::hat(R * f.mDv);
        dm::Mat9<Scalar> B = dm::Mat9<Scalar>::Zero();
        for (int i = 0; i < 3; i++) {
            B.template block<3, 3>(3 * i, 3 * i) = R;
        }
        mCov = A * mCov * A.transpose() + B * f.mCov * B.transpose();
        mJg = A * mJg + B * f.mJg;
        mJa = A * mJa + B * f.mJa;
        mDp += mDv * f.mDt + R * f.mDp;
        mDv += R * f.mDv;
        mDR = mDR * f.mDR;
        mT += f.mDt;
        mB = f.mT1;
        mvFactors.push_back(f);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// The factors between consecutive keyframes, built once as the per frame
// factors arrive in time order, as DM-VIO's coarse initialiser integrates the
// inertial data of the frames between keyframes into the factor that ends at
// the next keyframe. mOpen runs from the latest keyframe.
template <class Scalar_>
struct DelayedMargKfChain {
    using Scalar = Scalar_;

    std::deque<DelayedMargKfSpan<Scalar>> mvSpans;
    DelayedMargKfSpan<Scalar> mOpen;

    void Add(const IntegratedImuMeasurement<Scalar>& meas,
             const std::set<int64_t>& keyframes, int maxSpans) {
        const int64_t t0 = meas.get_start_t_ns();
        if (mOpen.mA >= 0 && t0 < mOpen.mB) return;
        if (mOpen.mA < 0 || t0 != mOpen.mB) {
            mOpen = DelayedMargKfSpan<Scalar>();
            if (keyframes.count(t0) == 0) return;
            mOpen.mA = mOpen.mB = t0;
        }
        mOpen.Append(DelayedMargFrameFactor<Scalar>(meas));
        if (keyframes.count(mOpen.mB) > 0) {
            mvSpans.push_back(mOpen);
            while (int(mvSpans.size()) > maxSpans) mvSpans.pop_front();
            const int64_t kf = mOpen.mB;
            mOpen = DelayedMargKfSpan<Scalar>();
            mOpen.mA = mOpen.mB = kf;
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
```

The store keeps the keyframes of the period and the chain, composes every per frame factor into the chain as its step is pushed, and drops a keyframe factor with its first keyframe's record once that keyframe's last step has advanced into $B$.

```cpp
    // Keyframes of the period, the nodes of the coarse optimisation, and the
    // factors between them composed from the stored per frame factors.
    void AddKeyframe(int64_t id) { mvpKeyframes.insert(id); }
    const std::set<int64_t>& Keyframes() const { return mvpKeyframes; }
    const DelayedMargKfChain<Scalar>& KfChain() const { return mpKfChain; }
```

```cpp
    void Push(DelayedMargStep<Scalar>&& step) {
        for (IntegratedImuMeasurement<Scalar>& meas : step.mvImu) {
            // Fills the lazily computed whitening before the step is shared.
            meas.get_sqrt_cov_inv();
            mpKfChain.Add(meas, mvpKeyframes,
                          DelayedMargJob<Scalar>::kCoarseMaxKeyframes - 1);
        }
        mpNextIndex = step.mIndex + 1;
        mvpSteps.push_back(
            std::make_shared<const DelayedMargStep<Scalar>>(std::move(step)));
    }
```

```cpp
            mvpSteps.pop_front();
            while (!mpKfChain.mvSpans.empty() &&
                   mpValues.mvRecords.count(mpKfChain.mvSpans.front().mA) > 0 &&
                   mpValues.mvRecords.at(mpKfChain.mvSpans.front().mA)
                           .mLastStep <= step->mIndex) {
                mpKfChain.mvSpans.pop_front();
            }
            for (auto it = mpValues.mvRecords.begin();
                 it != mpValues.mvRecords.end();) {
                if (it->second.mLastStep <= step->mIndex) {
                    mvpKeyframes.erase(it->first);
                    it = mpValues.mvRecords.erase(it);
                } else {
                    ++it;
                }
            }
```

The job's coarse inputs and outputs, `Run`, which smooths after a coarse optimisation only when the scale was estimated, and `Coarse`, Steps 2 to 7 of §2.3.11 on the chain extended by the window's factors.

```cpp
    // Coarse inertial optimisation followed by a readvance at its result,
    // §2.3.11. The keyframe factors of the store, the inertial factors of the
    // window, the keyframes known to the store and the window, the initial
    // gravity direction in the epoch
    // mEpoch, the information of the tilt prior, and the time after which the
    // optimisation stops iterating.
    bool mCoarse = false;
    DelayedMargKfChain<Scalar> mKfChain;
    Eigen::aligned_vector<IntegratedImuMeasurement<Scalar>> mvWindowImu;
    std::set<int64_t> mvKeyframes;
    dm::Vec3<Scalar> mInitialDir = dm::Vec3<Scalar>::Zero();
    Scalar mTiltPriorW = 0;
    Scalar mScaleVarMax = 0;
    double mCoarseDeadlineSeconds = 0;
    // Standard deviation of a held keyframe position, §2.3.11 Step 4.
    static constexpr Scalar kCoarsePositionStd = Scalar(0.02);
    // DM-VIO's maxNumPoses and the iteration limit of GTSAM's
    // Levenberg-Marquardt.
    static constexpr int kCoarseMaxKeyframes = 100;
    static constexpr int kCoarseMaxIterations = 100;

    // Outputs.
    DelayedMargPrior<Scalar> mResult;
    // Smoothing increment of each departed variable, and which of its 15
    // coordinates it covers.
    Eigen::aligned_map<int64_t, std::pair<dm::Vec15<Scalar>, uint32_t>>
        mvSmoothInc;
    Scalar mMaxPosInc = 0;
    double mSeconds = 0;
    // Coarse outputs, the block and the velocity increments in the epoch
    // mEpoch, and the one gyroscope and accelerometer bias of every state.
    dm::Vec3<Scalar> mCoarseDir = dm::Vec3<Scalar>::Zero();
    // Angle between the snapshot's and the estimated gravity direction.
    Scalar mCoarseTilt = 0;
    Scalar mCoarseSigma = 0;
    dm::Vec3<Scalar> mCoarseBg = dm::Vec3<Scalar>::Zero();
    dm::Vec3<Scalar> mCoarseBa = dm::Vec3<Scalar>::Zero();
    Eigen::aligned_map<int64_t, dm::Vec3<Scalar>> mvCoarseVelInc;
    Scalar mCoarseMaxVelInc = 0;
    Scalar mCoarseScaleVar = 0;
    bool mCoarseScaleFree = false;
    Scalar mCoarseCost0 = 0;
    Scalar mCoarseCost1 = 0;
    int mCoarseFactors = 0;
    int mCoarseSpans = 0;
    int mCoarseIterations = 0;
    // The deadline stopped the optimisation, whose result is then discarded.
    bool mCoarseTimedOut = false;

    int64_t FirstStep() const { return mvSteps.front()->mIndex; }
    int64_t LastStep() const { return mvSteps.back()->mIndex; }

    // Readvance, preceded by one smoothing pass when mSmooth is set. A coarse
    // job first runs the coarse optimisation, and smooths only when that
    // estimated the scale, in the role of DM-VIO's pose graph bundle
    // adjustment.
    void Run() {
        const auto t0 = std::chrono::steady_clock::now();
        bool readvance = true;
        bool smooth = mSmooth;
        if (mCoarse) {
            Coarse();
            readvance = mCoarseSpans > 0 && !mCoarseTimedOut;
            smooth = mSmooth && mCoarseScaleFree;
        }
        if (readvance && smooth) {
            std::vector<Elim> elims;
            Forward(&elims);
            Backward(elims);
        }
        if (readvance) Forward(nullptr);
        mSeconds = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - t0)
                       .count();
    }

    // The coarse IMU initialiser of DM-VIO, CoarseIMUInitOptimizer. The last
    // kCoarseMaxKeyframes keyframes are held, consecutive keyframes are joined
    // by the factors of mKfChain extended by the window's, and the gravity
    // direction, the scale applied to the held positions, one gyroscope and
    // accelerometer bias starting at zero, and the metric velocity of every
    // keyframe are estimated by Levenberg-Marquardt. The scale is held when
    // its marginal variance at the solution exceeds mScaleVarMax. The result
    // is applied to mValues and mGs, §2.3.11.
    void Coarse() {
        const auto tStart = std::chrono::steady_clock::now();

        // The store's keyframe factors, extended by the window's, and the held
        // poses of their keyframes.
        DelayedMargKfChain<Scalar> chain = mKfChain;
        for (const auto& meas : mvWindowImu) {
            chain.Add(meas, mvKeyframes, kCoarseMaxKeyframes - 1);
        }
        struct Span {
            const DelayedMargKfSpan<Scalar>* mSp = nullptr;
            dm::SO3<Scalar> mRa, mRb;
            dm::Vec3<Scalar> mPa = dm::Vec3<Scalar>::Zero();
            dm::Vec3<Scalar> mPb = dm::Vec3<Scalar>::Zero();
            dm::Mat9<Scalar> mL = dm::Mat9<Scalar>::Identity();
        };
        Eigen::aligned_vector<Span> vSpans;
        mCoarseFactors = 0;
        for (const DelayedMargKfSpan<Scalar>& kfs : chain.mvSpans) {
            if (!Known(kfs.mA) || !Known(kfs.mB)) continue;
            Span sp;
            sp.mSp = &kfs;
            const PoseVelBiasState<Scalar> sa =
                mValues.State(kfs.mA).getState();
            const PoseVelBiasState<Scalar> sb =
                mValues.State(kfs.mB).getState();
            sp.mRa = sa.T_w_i.so3();
            sp.mRb = sb.T_w_i.so3();
            sp.mPa = sa.T_w_i.translation();
            sp.mPb = sb.T_w_i.translation();
            vSpans.push_back(sp);
            mCoarseFactors += int(kfs.mvFactors.size());
        }
        mCoarseSpans = int(vSpans.size());

        const dm::Vec3<Scalar> dir0 = mGs.GetDir();
        const dm::Mat32<Scalar> basis0 = mGs.GetBasis();
        const Scalar sigma0 = mGs.GetSigma();
        mCoarseDir = dir0;
        mCoarseTilt = 0;
        mCoarseSigma = sigma0;
        mCoarseBg.setZero();
        mCoarseBa.setZero();
        mvCoarseVelInc.clear();
        mCoarseMaxVelInc = 0;
        mCoarseScaleFree = false;
        mCoarseScaleVar = 0;
        mCoarseIterations = 0;
        mCoarseTimedOut = false;
        if (vSpans.empty()) return;

        // Unknowns [tilt (2), scale increment, gyroscope bias (3),
        // accelerometer bias (3), keyframe velocities (3 each)].
        std::map<int64_t, int> nodeIdx;
        for (const Span& sp : vSpans) {
            nodeIdx.emplace(sp.mSp->mA, 0);
            nodeIdx.emplace(sp.mSp->mB, 0);
        }
        const int nNodes = int(nodeIdx.size());
        const int n = 9 + 3 * nNodes;
        const Scalar s0 = std::exp(sigma0);
        dm::VecX<Scalar> u0(3 * nNodes);
        {
            int i = 0;
            for (auto& [id, idx] : nodeIdx) {
                idx = i;
                u0.template segment<3>(3 * i) =
                    s0 * mValues.State(id).getState().vel_w_i;
                i++;
            }
        }

        // Whitening in metric units, the composed covariance and the noise of
        // the two held positions at the scale of the snapshot.
        for (Span& sp : vSpans) {
            dm::Mat9<Scalar> S = sp.mSp->mCov;
            S.template block<3, 3>(0, 0).diagonal().array() +=
                Scalar(2) * s0 * s0 * kCoarsePositionStd * kCoarsePositionStd;
            sp.mL = S.llt().matrixL();
        }
        const Scalar gNorm = mG.norm();

        dm::SO3<Scalar> Rg;
        Scalar eta = 0;
        dm::Vec3<Scalar> bg = dm::Vec3<Scalar>::Zero();
        dm::Vec3<Scalar> ba = dm::Vec3<Scalar>::Zero();
        dm::VecX<Scalar> du = dm::VecX<Scalar>::Zero(3 * nNodes);

        auto tiltResidual = [&](const dm::Vec3<Scalar>& dir,
                                const dm::Mat32<Scalar>& basis) {
            const dm::Vec3<Scalar> axis = mInitialDir.cross(dir);
            const Scalar sinAngle = axis.norm();
            dm::Vec3<Scalar> w = dm::Vec3<Scalar>::Zero();
            if (sinAngle > Sophus::Constants<Scalar>::epsilon()) {
                w = axis *
                    (std::atan2(sinAngle, mInitialDir.dot(dir)) / sinAngle);
            }
            return dm::Vec2<Scalar>(basis.transpose() * w);
        };

        // The residual of GTSAM's ImuFactor between the held keyframe poses
        // scaled by s = s0 (1 + eta), §2.3.11 Step 3. The normal equations
        // are accumulated block by block into a sparse matrix, since a
        // keyframe velocity meets only its neighbours.
        std::vector<Eigen::Triplet<Scalar>> vTriplets;
        auto linearise = [&](dm::SpMat<Scalar>* H, dm::VecX<Scalar>* b) {
            const dm::Vec3<Scalar> dir = Rg * dir0;
            const dm::Mat32<Scalar> basis = Rg.matrix() * basis0;
            const dm::Vec3<Scalar> gD = gNorm * dir;
            const Scalar s = s0 * (Scalar(1) + eta);
            const dm::Mat32<Scalar> dGd = -dm::SO3<Scalar>::hat(gD) * basis;
            Scalar cost = 0;
            if (H) {
                vTriplets.clear();
                b->setZero(n);
            }
            Eigen::Matrix<Scalar, 9, 15> J;
            for (const Span& sp : vSpans) {
                const DelayedMargKfSpan<Scalar>& kf = *sp.mSp;
                const int ia = nodeIdx.at(kf.mA);
                const int ib = nodeIdx.at(kf.mB);
                const dm::Vec3<Scalar> ua = u0.template segment<3>(3 * ia) +
                                            du.template segment<3>(3 * ia);
                const dm::Vec3<Scalar> ub = u0.template segment<3>(3 * ib) +
                                            du.template segment<3>(3 * ib);
                const Scalar T = kf.mT;
                const dm::Mat3<Scalar> RaT = sp.mRa.inverse().matrix();
                const dm::Vec9<Scalar> corr = kf.mJg * bg + kf.mJa * ba;
                const dm::Vec3<Scalar> dpVis = sp.mPb - sp.mPa;

                dm::Vec9<Scalar> r;
                r.template head<3>() =
                    RaT * (s * dpVis - ua * T - Scalar(0.5) * gD * T * T) -
                    (kf.mDp + corr.template head<3>());
                const dm::Vec3<Scalar> rR =
                    (dm::SO3<Scalar>::exp(corr.template segment<3>(3)) *
                     kf.mDR * sp.mRb.inverse() * sp.mRa)
                        .log();
                r.template segment<3>(3) = rR;
                r.template tail<3>() = RaT * (ub - ua - gD * T) -
                                       (kf.mDv + corr.template tail<3>());
                const dm::Vec9<Scalar> rw =
                    sp.mL.template triangularView<Eigen::Lower>().solve(r);
                cost += Scalar(0.5) * rw.squaredNorm();
                if (!H) continue;

                // Columns [tilt, scale, bg, ba, ua, ub].
                J.setZero();
                J.template block<3, 2>(0, 0) =
                    -Scalar(0.5) * T * T * RaT * dGd;
                J.template block<3, 2>(6, 0) = -T * RaT * dGd;
                J.template block<3, 1>(0, 2) = s0 * RaT * dpVis;
                dm::Mat3<Scalar> JlInv;
                Sophus::leftJacobianInvSO3(rR, JlInv);
                J.template block<3, 3>(0, 3) = -kf.mJg.template topRows<3>();
                J.template block<3, 3>(3, 3) =
                    JlInv * kf.mJg.template middleRows<3>(3);
                J.template block<3, 3>(6, 3) = -kf.mJg.template bottomRows<3>();
                J.template block<3, 3>(0, 6) = -kf.mJa.template topRows<3>();
                J.template block<3, 3>(6, 6) = -kf.mJa.template bottomRows<3>();
                J.template block<3, 3>(0, 9) = -T * RaT;
                J.template block<3, 3>(6, 9) = -RaT;
                J.template block<3, 3>(6, 12) = RaT;
                const Eigen::Matrix<Scalar, 9, 15> Jw =
                    sp.mL.template triangularView<Eigen::Lower>().solve(J);
                const Eigen::Matrix<Scalar, 15, 15> Hs = Jw.transpose() * Jw;
                const Eigen::Matrix<Scalar, 15, 1> bs = Jw.transpose() * rw;
                const int col[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8,
                                     9 + 3 * ia, 10 + 3 * ia, 11 + 3 * ia,
                                     9 + 3 * ib, 10 + 3 * ib, 11 + 3 * ib};
                for (int i = 0; i < 15; i++) {
                    (*b)[col[i]] += bs[i];
                    for (int j = 0; j < 15; j++) {
                        vTriplets.emplace_back(col[i], col[j], Hs(i, j));
                    }
                }
            }
            if (mTiltPriorW > 0) {
                const dm::Vec2<Scalar> rt = tiltResidual(dir, basis);
                cost += Scalar(0.5) * mTiltPriorW * rt.squaredNorm();
                if (H) {
                    vTriplets.emplace_back(0, 0, mTiltPriorW);
                    vTriplets.emplace_back(1, 1, mTiltPriorW);
                    b->template head<2>() += mTiltPriorW * rt;
                }
            }
            if (H) {
                H->resize(n, n);
                H->setFromTriplets(vTriplets.begin(), vTriplets.end());
            }
            return cost;
        };

        auto elapsed = [&]() {
            return std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - tStart)
                .count();
        };

        // Levenberg-Marquardt from the snapshot with the scale free or held,
        // stopping at the deadline so that the job ends within a frame.
        dm::SpMat<Scalar> H;
        dm::VecX<Scalar> b;
        auto solve = [&](bool scaleFree) {
            Rg = dm::SO3<Scalar>();
            eta = 0;
            bg.setZero();
            ba.setZero();
            du.setZero();
            Scalar cost = linearise(&H, &b);
            Scalar lambda = Scalar(1e-4);
            for (int it = 0; it < kCoarseMaxIterations && lambda < Scalar(1e8);
                 it++) {
                if (mCoarseDeadlineSeconds > 0 &&
                    elapsed() > mCoarseDeadlineSeconds) {
                    mCoarseTimedOut = true;
                    break;
                }
                mCoarseIterations++;
                dm::SpMat<Scalar> Hd = H;
                dm::VecX<Scalar> bd = b;
                for (int i = 0; i < n; i++) {
                    Hd.coeffRef(i, i) +=
                        lambda * H.coeff(i, i) + Scalar(1e-9);
                }
                if (!scaleFree) {
                    Hd.prune([](Eigen::Index r, Eigen::Index c, const Scalar&) {
                        return (r != 2 && c != 2) || r == c;
                    });
                    Hd.coeffRef(2, 2) = 1;
                    bd[2] = 0;
                }
                Eigen::SimplicialLDLT<dm::SpMat<Scalar>> ldlt(Hd);
                if (ldlt.info() != Eigen::Success) break;
                const dm::VecX<Scalar> x = -ldlt.solve(bd);
                if (!x.allFinite()) break;
                const dm::SO3<Scalar> Rg0 = Rg;
                const Scalar eta0 = eta;
                const dm::Vec3<Scalar> bg0 = bg, ba0 = ba;
                const dm::VecX<Scalar> du0 = du;
                Rg = dm::SO3<Scalar>::exp(Rg.matrix() * basis0 *
                                          x.template head<2>()) *
                     Rg;
                eta += x[2];
                bg += x.template segment<3>(3);
                ba += x.template segment<3>(6);
                du += x.tail(3 * nNodes);
                const Scalar costNew = linearise(nullptr, nullptr);
                if (eta > Scalar(-1) && std::isfinite(double(costNew)) &&
                    costNew < cost) {
                    const bool converged =
                        cost - costNew < Scalar(1e-6) * cost;
                    cost = linearise(&H, &b);
                    lambda = std::max(lambda / Scalar(10), Scalar(1e-7));
                    if (converged) break;
                } else {
                    Rg = Rg0;
                    eta = eta0;
                    bg = bg0;
                    ba = ba0;
                    du = du0;
                    lambda *= Scalar(10);
                }
            }
            linearise(&H, &b);
            return cost;
        };

        mCoarseCost0 = linearise(nullptr, nullptr);
        mCoarseCost1 = solve(true);
        // The marginal variance of the log scale at the solution, where
        // log s = sigma0 + log(1 + eta).
        {
            dm::SpMat<Scalar> Hv = H;
            for (int i = 0; i < n; i++) Hv.coeffRef(i, i) += Scalar(1e-9);
            Eigen::SimplicialLDLT<dm::SpMat<Scalar>> ldlt(Hv);
            dm::VecX<Scalar> e2 = dm::VecX<Scalar>::Zero(n);
            e2[2] = 1;
            const Scalar varEta =
                ldlt.info() == Eigen::Success ? ldlt.solve(e2)[2]
                                              : Scalar(1e30);
            mCoarseScaleVar = varEta / ((Scalar(1) + eta) * (Scalar(1) + eta));
        }
        mCoarseScaleFree = std::isfinite(double(mCoarseScaleVar)) &&
                           mCoarseScaleVar <= mScaleVarMax;
        if (!mCoarseScaleFree && !mCoarseTimedOut) {
            mCoarseCost1 = solve(false);
        }
        if (mCoarseTimedOut) return;

        mCoarseDir = Rg * dir0;
        mCoarseTilt = std::acos(
            std::clamp(mCoarseDir.dot(dir0), Scalar(-1), Scalar(1)));
        const Scalar s = s0 * (Scalar(1) + eta);
        mCoarseSigma = std::log(s);
        mCoarseBg = bg;
        mCoarseBa = ba;

        // Velocities in the estimator's units, u / s, at the keyframes, and at
        // the states between and after them propagated through the per frame
        // factors at the estimated biases and the held frame rotations.
        const dm::Vec3<Scalar> gD = gNorm * mCoarseDir;
        std::map<int64_t, dm::Vec3<Scalar>> vel;
        // Propagates v from the first state of span through its factors,
        // keeping the solved velocity of a keyframe at its end.
        auto propagate = [&](dm::Vec3<Scalar> v,
                             const DelayedMargKfSpan<Scalar>& span, bool open) {
            for (const DelayedMargFrameFactor<Scalar>& f : span.mvFactors) {
                if (!Known(f.mT0) || !Known(f.mT1)) break;
                const dm::Vec9<Scalar> c = f.mJg * bg + f.mJa * ba;
                const dm::Mat3<Scalar> R =
                    mValues.State(f.mT0).getState().T_w_i.so3().matrix();
                v += (gD * f.mDt + R * (f.mDv + c.template tail<3>())) / s;
                if (!open && f.mT1 == span.mB) break;
                vel[f.mT1] = v;
            }
        };
        for (const auto& [id, idx] : nodeIdx) {
            vel[id] = (u0.template segment<3>(3 * idx) +
                       du.template segment<3>(3 * idx)) /
                      s;
        }
        for (const Span& sp : vSpans) {
            propagate(vel.at(sp.mSp->mA), *sp.mSp, false);
        }
        if (nodeIdx.count(chain.mOpen.mA) > 0) {
            propagate(vel.at(chain.mOpen.mA), chain.mOpen, true);
        }
        for (const auto& [id, v] : vel) {
            const dm::Vec3<Scalar> inc =
                v - mValues.State(id).getState().vel_w_i;
            if (!inc.allFinite()) continue;
            mvCoarseVelInc[id] = inc;
            mCoarseMaxVelInc = std::max(mCoarseMaxVelInc, inc.norm());
        }

        // The readvance that follows sees the corrected values, every state
        // with the one estimated bias.
        auto apply = [&](int64_t id) {
            const PoseVelBiasState<Scalar> st = mValues.State(id).getState();
            dm::Vec15<Scalar> x = dm::Vec15<Scalar>::Zero();
            const auto it = mvCoarseVelInc.find(id);
            if (it != mvCoarseVelInc.end()) {
                x.template segment<3>(6) = it->second;
            }
            x.template segment<3>(9) = mCoarseBg - st.bias_gyro;
            x.template segment<3>(12) = mCoarseBa - st.bias_accel;
            mValues.ApplyInc(id, x);
        };
        for (const auto& [id, st] : mValues.mvWindowStates) apply(id);
        for (const auto& [id, rec] : mValues.mvRecords) {
            if (mValues.mvWindowStates.count(id) == 0) apply(id);
        }
        DelayedMargSetBlock(mGs, mCoarseDir, mCoarseSigma);
    }
```

The estimator records every new keyframe in the store, `measure` in `src/vi_estimator/sqrt_keypoint_vio.cpp`.

```cpp
        kf_ids.emplace(last_state_t_ns);
        if (mpDm) mpDm->AddKeyframe(last_state_t_ns);
```

`MakeDelayedJob` passes the store's chain, the keyframes of the store and the window and the deadline, and lets a coarse job smooth.

```cpp
    // Readvances of the coarse phase keep the departed states, so that the
    // coarse optimisation alone moves the history, except after a coarse
    // optimisation that estimated the scale.
    job->mSmooth = config.vio_gs_dm_smooth && (coarse || !mpDmCoarse);
    job->mCoarse = coarse;
    if (coarse) {
        for (const auto& kv : imu_meas) {
            job->mvWindowImu.push_back(kv.second);
            job->mvWindowImu.back().get_sqrt_cov_inv();
        }
        job->mInitialDir =
            this->mGsCorrectionLog->Between(0, job->mEpoch)
                .mQ.template cast<Scalar>() *
            g.normalized();
        if (config.vio_gravity_prior_std_deg > 0) {
            const Scalar sigma =
                Scalar(config.vio_gravity_prior_std_deg * M_PI / 180.0);
            job->mTiltPriorW = Scalar(1) / (sigma * sigma);
        }
        job->mKfChain = mpDm->KfChain();
        job->mvKeyframes = mpDm->Keyframes();
        job->mvKeyframes.insert(kf_ids.begin(), kf_ids.end());
        job->mScaleVarMax = Scalar(config.vio_gs_dm_coarse_scale_var);
        // A quarter of the frame period, so that the readvance that follows
        // also ends before the next frame.
        if (!imu_meas.empty()) {
            job->mCoarseDeadlineSeconds =
                0.25 * double(imu_meas.rbegin()->second.get_dt_ns()) * 1e-9;
        }
    }
```

`InstallCoarseJob` installs nothing for a job that timed out, and sets every state's bias to the estimated one.

```cpp
// Applies a coarse optimisation as a re-centring of the window, the priors and
// the store, its velocity increments and its one bias to every state, so that
// the prior the job readvanced at these values can be installed, §2.3.9.
// Returns false when the job had no keyframe pair to fit or ran out of time.
template <class Scalar_>
bool SqrtKeypointVioEstimator<Scalar_>::InstallCoarseJob(
    const DelayedMargJob<Scalar>& job) {
    if (job.mCoarseSpans == 0 || job.mCoarseTimedOut) return false;
    const GravityScaleCorrectionLog::Entry c =
        this->mGsCorrectionLog->Between(
            job.mEpoch, this->mGsCorrectionLog->LatestEpoch());
    const Vec3 dir = c.mQ.template cast<Scalar>() * job.mCoarseDir;
    const Scalar sigma = job.mCoarseSigma - Scalar(std::log(c.mScale));
    DelayedMargSetBlock(*this->mGsState, dir, sigma);
    RecentreGravityScale();
    AnchorGs(mpAnchorCurr);

    // The velocity increments were solved in the epoch of the snapshot.
    const GravityScaleCorrectionLog::Entry cNow =
        this->mGsCorrectionLog->Between(
            job.mEpoch, this->mGsCorrectionLog->LatestEpoch());
    const Mat3 Qm = cNow.mQ.template cast<Scalar>().matrix();
    const Scalar s = Scalar(cNow.mScale);
    auto inc = [&](int64_t id, const PoseVelBiasState<Scalar>& st) {
        typename PoseVelBiasStateWithLin<Scalar>::VecN x;
        x.setZero();
        const auto it = job.mvCoarseVelInc.find(id);
        if (it != job.mvCoarseVelInc.end()) {
            x.template segment<3>(6) = s * (Qm * it->second);
        }
        x.template segment<3>(9) = job.mCoarseBg - st.bias_gyro;
        x.template segment<3>(12) = job.mCoarseBa - st.bias_accel;
        return x;
    };
    for (auto& [id, state] : frame_states) {
        state.applyInc(inc(id, state.getState()));
    }
    for (auto& [id, rec] : mpDm->Values().mvRecords) {
        if (frame_states.count(id) == 0) {
            rec.mState.applyInc(inc(id, rec.mState.getState()));
        }
    }
    return true;
}
```

`UpdateDelayedMarg` installs the job of the previous marginalisation, waiting for it if needed, and starts a coarse job at every new keyframe once more than five are known, without the motion gate of Change 14.

```cpp
    // A job started at the previous marginalisation is installed now, waiting
    // for it if needed, so that every job takes effect within one frame.
    double waitSeconds = 0;
    if (mpDmFuture.valid()) {
        Timer tWait;
        std::shared_ptr<DelayedMargJob<Scalar>> job = mpDmFuture.get();
        waitSeconds = tWait.elapsed();
        install(job);
    }
```

```cpp
        } else if (mpDmCoarse && int(mpDm->Keyframes().size()) > 5 &&
                   *mpDm->Keyframes().rbegin() > mpDmLastCoarseKeyframe) {
            // DM-VIO runs its coarse initialiser at every new keyframe once
            // more than five are known.
            mpDmLastCoarseKeyframe = *mpDm->Keyframes().rbegin();
            start(true);
        }
    }
```

The record `vio_delayed_marg` drops `coarse_var_factor` and gains the iteration count, whether the deadline stopped the job, and the time the VIO thread waited for the job. `coarse_tilt_deg` is now the angle between the snapshot's and the estimated gravity direction, which Change 16 read from the job's block after the job had moved it to the result, so that it logged zero. `coarse_ba` and `coarse_bg` are the norms of the absolute biases, and `coarse_spans` counts keyframe factors.

```cpp
              {"coarse_spans", "count"}, {"coarse_bg", "f"},
              {"coarse_max_vel_inc", "f"}, {"coarse_iterations", "count"},
              {"coarse_timed_out", "count"}, {"wait", "ms"}});
```

```cpp
    mpLogger->AddVioDelayedMarg(
        last_state_t_ns, steps, records, started, installed, coarse,
        double(anchorSigma), double(anchorTilt) * 180.0 / M_PI, jobSeconds,
        catchup, catchupSeconds, double(maxPosInc), storeSeconds,
        mpDmCoarse ? 1 : 0, coarseJob ? double(coarseJob->mCoarseSigma) : 0.0,
        coarseJob ? double(coarseJob->mCoarseTilt) * 180.0 / M_PI : 0.0,
        coarseJob ? double(coarseJob->mCoarseScaleVar) : 0.0,
        coarseJob ? int(coarseJob->mCoarseScaleFree) : 0,
        coarseJob ? coarseJob->mCoarseBa.norm() : 0.0,
        coarseJob ? double(coarseJob->mCoarseCost0) : 0.0,
        coarseJob ? double(coarseJob->mCoarseCost1) : 0.0,
        coarseJob ? coarseJob->mCoarseSpans : 0,
        coarseJob ? double(coarseJob->mCoarseBg.norm()) : 0.0,
        coarseJob ? double(coarseJob->mCoarseMaxVelInc) : 0.0,
        coarseJob ? coarseJob->mCoarseIterations : 0,
        coarseJob ? int(coarseJob->mCoarseTimedOut) : 0, waitSeconds);
}
```

The unit tests mark every fourth frame as a keyframe and build the job's chain from the generated factors. The scale test checks exact recovery, a static test checks that the gate holds the scale at rest while the gyroscope bias is still estimated, and a deadline test checks that a job stopped before its first iteration installs nothing. The jitter test of Change 16 is removed, because the present form is biased on jittered positions by design (§2.3.11 Step 8).

```cpp
// A coarse job over a random trajectory, or a vehicle at rest when moving is
// false, whose positions and velocities are shrunk by scale, whose velocities
// are perturbed by velError and whose positions carry a jitter of jitterStd,
// with inertial factors that carry the biases ba and bg and a keyframe every
// fourth frame, plans/delayed_marginalisation.md §2.3.11.
static basalt::DelayedMargJob<double> MakeCoarseTestJob(
    const basalt::Se3Spline<5>& spline, double scale, double velError,
    double jitterStd, const Eigen::Vector3d& ba, const Eigen::Vector3d& bg,
    bool moving = true) {
  constexpr int N = 41;
  constexpr int64_t kDt = 50000000;
  std::mt19937 gen(3);
  std::normal_distribution<double> noise(0.0, 1.0);

  basalt::DelayedMargJob<double> job;
  for (int i = 0; i < N; i += 4) job.mvKeyframes.insert(i * kDt);
  for (int i = 0; i < N; i++) {
    const int64_t t = i * kDt;
    Sophus::SE3d pose = moving ? spline.pose(t) : Sophus::SE3d();
    pose.translation() /= scale;
    pose.translation() +=
        jitterStd *
        Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
    const Eigen::Vector3d vel =
        (moving ? spline.transVelWorld(t) : Eigen::Vector3d::Zero().eval()) /
            scale +
        Eigen::Vector3d::Constant(velError);
    basalt::PoseVelBiasStateWithLin<double> st(
        t, pose, vel, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), true);
    if (i == N - 1) {
      job.mValues.mvWindowStates[t] = st;
    } else {
      job.mValues.mvRecords[t].mState = st;
    }
  }
  for (int i = 0; i + 1 < N; i++) {
    basalt::IntegratedImuMeasurement<double> meas(
        i * kDt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    for (int64_t t = i * kDt + 2500000; t < (i + 1) * kDt; t += 5000000) {
      basalt::ImuData<double> data;
      const Sophus::SE3d pose = moving ? spline.pose(t) : Sophus::SE3d();
      const Eigen::Vector3d accel =
          moving ? spline.transAccelWorld(t) : Eigen::Vector3d::Zero().eval();
      data.accel =
          pose.so3().inverse() * (accel - basalt::constants::g) + ba;
      data.gyro =
          (moving ? spline.rotVelBody(t) : Eigen::Vector3d::Zero().eval()) +
          bg;
      data.t_ns = t + 2500000;
      // The noise densities of data/sitl_calib.json at 200 Hz.
      meas.integrate(data, Eigen::Vector3d::Constant(1.28),
                     Eigen::Vector3d::Constant(1.1e-3));
    }
    auto step = std::make_shared<basalt::DelayedMargStep<double>>();
    job.mKfChain.Add(meas, job.mvKeyframes, 99);
    step->mvImu.push_back(meas);
    job.mvSteps.push_back(step);
  }

  job.mG = basalt::constants::g;
  job.mInitialDir = basalt::constants::g.normalized();
  job.mScaleVarMax = 1.0;
  job.mCoarse = true;
  return job;
}

// The coarse optimisation recovers the scale, the gravity direction, both
// biases and the velocities of exact data, plans/delayed_marginalisation.md
// §2.3.11.
TEST(VioTestSuite, DelayedMargCoarseScaleTest) {
  basalt::Se3Spline<5> spline(int64_t(1e9));
  spline.genRandomTrajectory(15);
  const double scale = 1.6;
  const Eigen::Vector3d ba(0.05, -0.03, 0.02);
  const Eigen::Vector3d bg(0.004, -0.003, 0.002);

  basalt::DelayedMargJob<double> job =
      MakeCoarseTestJob(spline, scale, 0.3, 0.0, ba, bg);
  job.Coarse();

  EXPECT_TRUE(job.mCoarseScaleFree);
  EXPECT_EQ(job.mCoarseSpans, 10);
  EXPECT_NEAR(job.mCoarseSigma, std::log(scale), 1e-3);
  EXPECT_LT(std::acos(std::min(1.0, job.mCoarseDir.dot(
                                        basalt::constants::g.normalized()))),
            1e-3);
  EXPECT_LT((job.mCoarseBa - ba).norm(), 1e-3);
  EXPECT_LT((job.mCoarseBg - bg).norm(), 1e-4);
  for (int i = 0; i < 41; i++) {
    const int64_t t = i * int64_t(50000000);
    EXPECT_LT((job.mValues.State(t).getState().vel_w_i -
               spline.transVelWorld(t) / scale)
                  .norm(),
              1e-2);
  }
}

// At rest the scale has no information, so the gate holds it, while the
// biases are still estimated.
TEST(VioTestSuite, DelayedMargCoarseStaticTest) {
  basalt::Se3Spline<5> spline(int64_t(1e9));
  spline.genRandomTrajectory(15);
  const Eigen::Vector3d ba(0.05, -0.03, 0.02);
  const Eigen::Vector3d bg(0.004, -0.003, 0.002);

  basalt::DelayedMargJob<double> job =
      MakeCoarseTestJob(spline, 1.6, 0.0, 0.02, ba, bg, false);
  job.Coarse();

  EXPECT_FALSE(job.mCoarseTimedOut);
  EXPECT_FALSE(job.mCoarseScaleFree);
  EXPECT_DOUBLE_EQ(job.mCoarseSigma, 0.0);
  EXPECT_LT((job.mCoarseBg - bg).norm(), 1e-4);
}

// A job that misses its deadline returns no correction.
TEST(VioTestSuite, DelayedMargCoarseDeadlineTest) {
  basalt::Se3Spline<5> spline(int64_t(1e9));
  spline.genRandomTrajectory(15);

  basalt::DelayedMargJob<double> job =
      MakeCoarseTestJob(spline, 1.6, 0.3, 0.0, Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero());
  job.mCoarseDeadlineSeconds = 1e-12;
  job.Coarse();

  EXPECT_TRUE(job.mCoarseTimedOut);
  EXPECT_TRUE(job.mvCoarseVelInc.empty());
  EXPECT_EQ(job.mCoarseIterations, 0);
}
```

Backwards compatibility. Every change is confined to the gravity and scale period, so nothing changes when `vio_gs_enable` is false. The eight configuration files of Change 9 were edited so that none sets a removed key, and no file outside `ext/basalt` sets either, which a search of `/ws/ros_ws/src` confirmed on 2026-11-14. In the delayed phase the only change is that a readvance is installed at the next marginalisation, waiting if needed, instead of at the first after it finished. The coarse phase no longer waits for the motion flag, so coarse optimisations run on the ground as soon as six keyframes exist, where the gate holds the scale.

#### Change 11. Documentation and context

After implementation, `doc/VIO.md` §3.3.4 gains a paragraph on the delayed mode, `context/dynamic_marginalisation.md` §6.3 gains a note that it is implemented with the results of §3.4.3, including the measured conflict with the mapper's pose updates, `context/vio_opt.md` §15 gains an entry on the store's invariants, and the `context/README.md` row of `dynamic_marginalisation.md` is refreshed. These are prose changes and follow the incremental rules of `context/README.md`.

### 3.3 Blast radius

| Symbol or file | Consumers | Effect with the keys at their defaults |
|---|---|---|
| `VioConfig` | every configuration source, the estimator, the batch scripts | nine stored values |
| `Logger` | every component, `scripts/analysis` | one new, empty CSV |
| `MargHelper::marginalizeHelperSqrtToSqrt` | `marginalize`, `MarginalizeGS`, `NfrMapper`, three tests | identical, the new arguments default to null |
| `SqrtKeypointVioEstimator::LinearizeMargGS` | `marginalize` | identical, the new arguments default to null |
| `marginalize`, `measure`, `RecentreGravityScale`, `MarginalizeGS`, `RecentrePriorsGS`, `FoldGravityScale` | `optimize_and_marg`, the controller | identical, every new branch needs `mpDm` or `mpDmEnabled` |
| `delayed_marginalisation.h` | the estimator, `test_vio` | not executed |
| `data/sitl_config_vo.json` | `djinn start sitl2 basalt`, `djinn start basalt-replay` | the delayed mode with the hold, by design |
| Removed keys and members of Change 12 | the eight configuration files, the estimator | none outside `ext/basalt`, confirmed by a search of `/ws/ros_ws/src/slam` on 2026-11-13 |
| `vio_gravity_scale.csv` `swap_event` | `scripts/analysis` swap timeline and counts | values 5 and 6 instead of 1 to 3 in the period |
| `vio_ba_step_prior_std` | the coarse optimisation only | no window prior on the accelerometer bias step any more |
| `vio_delayed_marg.csv` `coarse_triples` | none, no script reads a `coarse_` column | renamed `coarse_spans`, with `coarse_bg`, `coarse_var_factor` and `coarse_max_vel_inc` added by Change 16 |
| `SqrtKeypointVioEstimator::InstallCoarseJob`, `Logger::AddVioDelayedMarg` | `UpdateDelayedMarg` only | return type and arguments of Change 16 |
| `basalt::dm` alias templates | every file that includes `delayed_marginalisation.h`, the estimator and `test_vio` | new names in a nested namespace, no clash found in `include`, `src` or the third party headers they include |
| `vio_gs_dm_coarse_every`, `vio_ba_step_prior_std` | `VioConfig`, its loader, the eight configuration files, the estimator | removed by Change 17, no consumer elsewhere |
| `vio_delayed_marg.csv` `coarse_var_factor` | none | removed by Change 17, with `coarse_iterations`, `coarse_timed_out` and `wait` added |
| `UpdateDelayedMarg` install timing | every job of the store | installed at the next marginalisation, waiting if needed, Change 17 |

Note, 2026-11-13. The first rows describe the opt in mode of 2026-11-10. With the revision the delayed store is the period's only mode, so every row applies whenever `vio_gs_enable` is set and none applies otherwise.

The local mapper, the visualiser, the optical flow, the controller and the ROS node are not touched. With the delayed mode on, the mapper receives the same packets and keyframes, and only the pose updates it sends back are dropped while the store is active.

### 3.4 Validation

#### 3.4.1 Code reading, before any run

1. With `vio_gs_enable` false or `vio_gs_delayed_marg` false, trace every statement added by Changes 5 to 8 to a branch on `mpDm`, `mpDmEnabled` or `mpDmFoldRequested`, and confirm that none executes, and that `marginalizeHelperSqrtToSqrt` is called with six arguments everywhere outside the store.
2. Confirm that the capture of Change 6 runs after `MarginalizeGS` and before `setLinTrue` and the erasure of the departing states, that it reads offsets directly rather than through `computeDelta`, and that `mOrderNew` is set from `marg_order_new` before `UpdateDelayedMarg`.
3. Confirm that every prior or row set in the store is delta independent with respect to the first estimates of its variables and the anchor of its block, and that `MarginaliseStep` brings each to the evaluation point by (D4) before the QR and back afterwards.
4. Confirm that the worker reads only its snapshot and the correction log, that every stored `IntegratedImuMeasurement` has its whitening cached by `Push`, and that `InstallDelayedJob` maps the job's prior and increments by `Between(job.mEpoch, now)`.
5. Confirm that `RecentreGravityScale` maps the delayed prior, its anchor and the records, and that the steps are mapped when read.

#### 3.4.2 Builds and unit tests, run on 2026-11-10

The working tree of `ext/basalt` was copied to the scratchpad and Changes 1 to 10 were applied to the copy. The copy was configured standalone with `-DBASALT_BUILD_TESTS=ON`, both instantiations and `RelWithDebInfo`, whose flags include `-Werror` and `-DEIGEN_INITIALIZE_MATRICES_BY_NAN` (`CMakeLists.txt:110`), and built with GCC 12.2. The unmodified copy was built the same way as the reference.

| Check | Result |
|---|---|
| `basalt` and every test target, modified copy | built, no warning or error |
| `test_vio`, unmodified copy | 5 of 5 passed |
| `test_vio`, modified copy | 8 of 8 passed, the 5 existing tests and the 3 of Change 10 |
| `test_qr`, `test_linearization`, `test_nfr`, `test_patch`, modified copy | 3, 4, 4 and 2 passed, which exercise the helper of Change 3 through its existing callers |
| Negative control in `DelayedMargReadvanceJointTest` | a scale moved by 0.2 changes the readvanced information by more than $10^{-3}$ relative |
| `VioConfig::load` of the seven JSON files of Change 9 | all loaded, the SITL file with the delayed mode and the hold on |
| Micro benchmark of `marginalizeHelperSqrtToSqrt`, 15 removed columns, `-O3 -march=native` | 0.022 ms at 90 by 63, 0.035 ms at 135 by 63, 0.051 ms at 135 by 78, 0.065 ms at 171 by 78 |

The `slam` ROS package was then built from a scratch copy of `/ws/ros_ws/src/slam` with the modified `ext/basalt` into a separate build and install space, with `-DCMAKE_BUILD_TYPE=RelWithDebInfo -DCXX_MARCH=x86-64-v3` as the host build uses and `-DPangolin_DIR=/usr/local/lib/cmake/Pangolin`, because the Pangolin package configuration installed under `/ws/ros_ws/install/slam` lacks its targets file. The workspace's own build and install spaces were not touched.

#### 3.4.3 Replays of `telemetry_40`, run on 2026-11-10

Every replay ran `basalt_slam_test_recorded.launch.py` headless with the scratch install overlaid on `/ws/ros_ws/install`, on its own `ROS_DOMAIN_ID` between 46 and 67, with `compressed_input_topic:=/unused_in_this_replay/compressed` (`/ws/context/djinn-workflows.md`, basalt-replay), a 170 s wall clock limit, and starts staggered by 4 s when several ran at once. The measures are those of plan 2, the scale ratio as the length of the estimated displacement over 2 s divided by that of ground truth, and the tilt error as the angle between the body frame gravity directions of estimate and ground truth, both from `gt_eval.csv`. The flight hops to 1 m, hovers and then climbs, and the period began at 50.85 s in every run. Logs are under `/ws/logs/slam/2026-11-10_*`.

| Configuration | Run | Scale ratio at 70 s, 80 s, 94 s, 100 s | Tilt error at the same times | Replacements |
|---|---|---|---|---|
| Dynamic, Changes A and B, as configured today | `17-17-31` | 5.71, 5.78, 5.79, 5.72 | 3.8°, 0.7°, 1.1°, 0.8° | 35 swaps |
| | `17-50-04` | 3.46, 3.21, 3.30, 3.26 | 1.1°, 1.2°, 1.1°, 0.5° | 38 swaps |
| | `17-50-08` | 3.21, 3.19, 3.58, 4.08 | 2.3°, 10.0°, 20.3°, 25.2° | 30 swaps |
| Delayed, pose updates applied, smoothing on | `17-13-13` | 0.18, 0.18, 0.21, 0.19 | 36.5°, 35.9°, 34.4°, 34.5° | 79 |
| Delayed, pose updates applied, smoothing off | `17-17-34` | 0.01, 0.00, 0.00, 0.01 | 8.5°, 18.8°, 18.7°, 23.7° | 351 |
| | `17-22-01` | 0.31, 0.30, 0.31, 0.32 | 7.2°, 2.7°, 1.0°, 1.2° | 78 |
| Single prior, no replacement ($\theta_s = \theta_g = 10^9$) | `17-21-58` | 12.4, 34.1, 68.0, 93.8 | 85.9°, 92.2°, 93.8°, 94.9° | none |
| No pose updates at all, dynamic | `17-39-28` | 0.98, 0.88, 0.89, 0.89 | 0.4°, 0.6°, 0.8°, 0.5° | 2 swaps |
| No pose updates at all, delayed, smoothing off | `17-39-32` | 0.89, 0.78, 0.82, 0.87 | 0.6°, 0.4°, 0.5°, 0.2° | 1 |
| No pose updates at all, delayed, smoothing on | `17-39-36` | 0.91, 0.85, 0.90, 0.91 | 0.5°, 0.7°, 0.7°, 0.7° | 5 |
| Delayed with hold, as configured by Change 9 | `17-46-22` | 1.53, 1.49, 1.11, 1.24 | 3.5°, 1.4°, 0.9°, 0.9° | 1 |
| | `17-49-56` | 0.84, 1.01, 0.92, 0.91 | 0.6°, 0.4°, 1.4°, 1.3° | 1 |
| | `17-53-44` | 1.53, 1.46, 1.48, 1.50 | 3.4°, 1.1°, 0.9°, 0.6° | 1 |
| Delayed with hold, smoothing off | `17-53-37` | 0.95, 1.04, 0.95, 0.91 | 1.7°, 0.8°, 1.6°, 1.3° | 1 |
| | `17-53-41` | 1.85, 1.69, 1.72, 1.67 | 4.0°, 1.4°, 0.6°, 0.9° | 5 |
| Delayed with hold, fold on settle | `17-46-26` | 1.25, 1.06, 1.11, 1.22 | 0.5°, 3.0°, 2.4°, 1.9° | 1 |
| | `17-50-00` | 1.44, 1.17, 1.62, 1.62 | 0.5°, 2.5°, 1.9°, 1.6° | 1 |
| | `17-53-48` | 1.70, 1.62, 1.75, 1.76 | 0.5°, 5.4°, 6.3°, 6.6° | 1 |
| Delayed with hold, synchronous | `17-46-30` | 1.52, 1.68, 1.62, 1.56 | 2.4°, 2.5°, 1.2°, 1.2° | 1 |

The pose update rows were obtained with a diagnostic variable that made `QueuePoseUpdates` return early, present only in the scratch build for these runs and not part of any change.

Six results follow from the table.

1. The machinery is correct on real data. A diagnostic build readvanced the store at the main prior's own anchor after every marginalisation of a single prior run (`17-33-07`) and compared the result with the main prior. At the first step the two agreed to $3 \times 10^{-15}$, and with up to 200 steps in the store and the vehicle on the ground, their information agreed to a relative $3 \times 10^{-5}$ to $10^{-3}$, the remainder being the lever arm column that the stored visual rows drop by design (§2.3.3). The same diagnostic measured a full readvance of 200 steps at 9 ms to 13.6 ms, which confirms the cost model of §2.3.10.
2. The residuals did not agree, and the reason is the mapper. On the ground the readvanced prior had a whitened cost of 1.7 to 7.2 at the current values against 0.0004 to 0.004 for the main prior, while the mapper moved keyframe poses by 1 mm to 18 mm per update (`vio_pose_update.csv` of the same run). §2.3.8 derives why only the readvance sees these moves.
3. With pose updates applied and replacements active, the scale collapsed in all three runs, to ratios of 0.0 to 0.3, and the replacements fired 78 to 351 times, chasing the scale down. With no pose updates at all, the dynamic mode, the delayed mode and the delayed mode with smoothing all held the scale at 0.78 to 0.98 and the tilt below 1°, and the delayed mode needed one to five replacements. The collapse is therefore the conflict of §2.3.8, made visible by the readvance, and not a property of the delayed graph.
4. With the hold of Change 9, the delayed mode installed one replacement at 58.9 s in every run but one, released the store at about 66 s, and ended with scale ratios of 0.84 to 1.85 and tilt errors at or below 6.6°, against 3.2 to 5.8 for the dynamic mode on the same binary, one of whose runs also diverged in tilt to 25°. Before 66 s every held run was identical, scale 0.80 at 60 s, because the mapper had no influence yet. After the release the scale drifted in some runs from about 1.0 to about 1.5, so the mapper's pose updates also degrade the estimate after the delayed phase, and the runs without any pose updates remained the best.
5. A single prior that is never replaced diverges in scale and tilt, which is why some form of relinearisation is needed at all, and folding on settle did not change the outcome measurably in three runs.
6. Smoothing made no measurable difference in this flight, whose one replacement came early and whose smoothing increments were 0.05 m to 0.19 m at most in the held runs. Its effect on the kinematic inconsistency at lift off of the live climbs, F1, remains to be measured.

Timing, from the same runs. In the asynchronous runs the worker's installed jobs took 3.8 ms to 11 ms without smoothing and 13.2 ms to 28.6 ms with smoothing, depending on how full the store was. The readvance of the steps stored while a job ran was one step every time, costing 0.04 ms to 0.2 ms in the VIO thread. The store's per frame cost in the VIO thread was 0.012 ms at the median, about 0.13 ms at the 90th percentile and at most 0.8 ms. `marginalize` took 0.29 ms to 0.38 ms at the median in the delayed mode and 0.53 ms to 0.62 ms in the dynamic mode, because the delayed mode linearises and factorises one prior instead of three. In the synchronous run the job ran inside `marginalize`, whose maximum rose to 13.5 ms, and `measure` peaked at 19 ms, still below the 54 ms period. Runs started in parallel shared the 32 cores and show `measure` medians of 6 ms to 7.8 ms against 5.3 ms for a run alone, so absolute times of parallel runs are upper bounds.

#### 3.4.4 Outstanding, to be run by the user

1. The working tree change itself, `MAKEFLAGS=-j4 colcon build --packages-select slam` on the host. The revision was built into a separate install space on 2026-11-13 (§3.4.5) and syntax checked in the working tree, which the build does not replace.
2. `test_vio`, `test_qr`, `test_linearization` and `test_nfr` passed in a standalone build on 2026-11-14 with Change 16, which replaced the test of Change 15 (§3.4.6). Note, 2026-11-14. This item previously recorded that a standalone build of 2026-11-13 ended with exit code 1 before its log could be read and that the tests remained to be run.
3. Live `sitl2` flights with Changes 9 and 15, the three profiles of plan 2 §5.5 item 5.
4. Replays of further flights, and three or more replays of `telemetry_100`, whose single replay ended with a scale ratio of 0.30 in the last window before landing (§3.4.5).
5. The readvances fired on every frame for the last 2 s of `telemetry_34` (§3.4.5), at landing. Whether the contact with the ground drives the block, and whether the fold should end the period on landing, is to be determined.
6. The constant `kCoarsePositionStd` of §2.3.11 was set from the measured jitter and not tuned, and the cadence $k$ was not varied. Note, 2026-11-14. This item previously named `kCoarseAccelStd` and `kCoarseMinDisplacement`, which Change 16 removed.
7. A decision on the mapper's pose updates, which are now held for the whole period. The runs of 2026-11-10 without pose updates were the most accurate in both modes.

Note, 2026-11-13. Items 2 and 3 of the list of 2026-11-10 named flights with Change 9 alone and replays with `vio_gs_dm_fold_on_settle`, `vio_gs_dm_delay` 100 and `vio_gs_dm_thresh_scale` 1.05, which the revision superseded or removed.

#### 3.4.5 Replays of `telemetry_34` and `telemetry_100`, run on 2026-11-13

The revision was applied to a scratch copy of the working tree, built into a separate install space as in §3.4.2, and replayed headless through a copy of `basalt_slam_test_recorded.launch.py` that passes `--rate 2` to `ros2 bag play`, since the bags hold about 185 s of simulated time over 530 s to 555 s of wall time. Each replay ran on its own `ROS_DOMAIN_ID` with `compressed_input_topic:=/unused_in_this_replay/compressed` and the configuration of Change 15 with `vio_gs_duration_s` 7000 s, and logged to the scratchpad through a scratch only change of `slam.cpp` that reads the log root from `BASALT_LOG_ROOT`. `telemetry_34` is the bag of the live run `2026-11-13_15-49-25` and `telemetry_100` that of `2026-11-11_20-05-19`. The scale ratio is the length of the estimated displacement over 2 s divided by that of ground truth, centred at the given time after lift off, and the tilt error the angle between the body frame gravity directions of estimate and ground truth, both from `gt_eval.csv`. The scratch logs were lost with the container on 2026-11-13, so the table holds the values as computed that day.

| Flight | Run | Scale ratio and tilt error at 2, 5, 8, 12, 20, 30, 45, 60, 90, 120 and 170 s after lift off | Largest tilt error | Final position error |
|---|---|---|---|---|
| `telemetry_34` | live, period folded 1 s after lift off | 2.22/1.3°, 1.25/0.8°, 0.92/0.9°, 0.67/0.2°, 0.70/1.2°, 0.74/0.5°, 0.81/0.4°, 0.88/0.9°, 0.95/0.4°, 1.00/0.8°, 1.40/0.8° | 2.6° | 43.2 m |
| | replay, `vio_gs_enable` false | 5.63/3.1°, 3.58/2.8°, 3.29/2.4°, 3.32/4.7°, 3.22/2.8°, 2.22/2.7°, 1.85/1.2°, 1.37/1.6°, 0.98/0.2°, 1.83/0.6°, 2.00/0.4° | 6.1° | 89.3 m |
| | replay, three node form regressing the scale on the visual side | 0.12/1.5°, 0.12/1.0°, 0.34/1.5°, 0.32/2.1°, 0.34/0.8°, 6.82/1.1°, 1.15/0.2°, 0.91/0.2°, 0.91/0.3°, 0.95/0.2°, 0.95/0.0° | 3.3° | 0.4 m |
| | replay, Changes 12 to 15 | 0.56/0.6°, 0.99/0.6°, 0.91/0.6°, 0.94/0.7°, 0.92/0.6°, 1.10/0.5°, 1.00/0.2°, 0.92/0.2°, 0.93/0.2°, 0.98/0.5°, 0.88/0.3° | 1.7° | 0.8 m |
| | replay, Changes 12 to 15 | 0.56/0.6°, 0.99/0.6°, 0.91/0.6°, 0.94/0.7°, 0.92/0.6°, 0.99/0.5°, 0.99/0.2°, 0.90/0.5°, 0.93/0.4°, 1.01/0.6°, 0.96/0.4° | 2.1° | 0.4 m |
| | replay, Changes 12 to 15 | 0.56/0.6°, 0.99/0.6°, 0.91/0.6°, 0.94/0.7°, 0.92/0.6°, 0.99/0.5°, 0.99/0.2°, 0.90/0.5°, 0.92/0.4°, 1.01/0.6°, 0.98/0.3° | 2.1° | 1.5 m |
| `telemetry_100` | live, dynamic marginalisation with Changes A and B and the cadence 10 | 0.81/7.4°, 1.24/2.2°, 1.03/0.6°, 1.08/0.0°, 1.03/0.1°, 0.93/0.1°, 0.98/0.6°, 1.04/0.9°, 1.09/1.8°, 0.98/0.5°, 2.21/7.8° | 15.4° | 4.8 m |
| | replay, `vio_gs_enable` false | 3.63/36.9°, 4.89/35.4°, 6.25/30.9°, 8.63/32.2°, 18.0/40.0°, 47.9/48.1°, 20.0/48.0°, 34.0/39.7°, 63.1/68.3°, 370/60.8°, 5306/65.5° | 75.3° | 117 km |
| | replay, Changes 12 to 15 | 1.48/0.6°, 0.63/0.4°, 1.18/0.1°, 1.20/0.6°, 1.21/0.4°, 1.11/0.9°, 1.09/0.2°, 1.05/1.3°, 1.08/1.3°, 1.00/0.5°, 0.30/0.3° | 1.8° | 2.9 m |

Six results follow from the table and the logs.

1. No replay of Changes 12 to 15 crashed, on either flight. The first form of the coarse optimisation aborted 7 s into `telemetry_34` (§3.5 item 10), and the three node form with the scale regressed on the visual side collapsed the scale (§3.5 item 11).
2. On `telemetry_34`, Basalt alone over scales by a factor of 3 to 6 in the first 30 s after lift off. The revision holds the scale ratio between 0.88 and 1.10 from 5 s after lift off to the end and the tilt within 2.1°, in three replays that agree to the second decimal, against 0.67 to 2.22 for the live run with the period folded after 1 s.
3. On `telemetry_100`, Basalt alone diverges in tilt and position. The revision holds the scale ratio between 1.00 and 1.21 from 8 s after lift off to 120 s and the tilt within 1.8°, against a tilt error of up to 15.4° for the live run with dynamic marginalisation, and its last window before landing measured 0.30.
4. The coarse phase installed 13 or 14 corrections and ended 10.4 s to 10.7 s into the run. Its corrections were within ±0.27 in log scale and 0.6° in tilt in the final form, the scale gate held the scale until the first triple entered after lift off, and the coarse jobs took 0.08 ms to 0.24 ms in the worker. The delayed phase then installed 29 to 49 readvances.
5. The readvances fired on every frame for the last 2 s of `telemetry_34`, from 181.7 s to 183.4 s, around the landing.
6. Timing, from four replays run at once on 32 cores, so upper bounds. `measure` took 11.7 ms at the median and 21.2 ms at the 90th percentile against the 54 ms period, `marginalize` 0.85 ms and 2.1 ms, the store's capture, push and advance 0.24 ms and 0.52 ms, an installation's catch up 0.10 ms at the median and at most 0.38 ms, and a smoothed readvance of a full store 27 ms at the median and at most 45 ms in the worker. The store held at most 200 steps and 207 records.

#### 3.4.6 Checks of Change 16, run on 2026-11-14

Five checks ran on 2026-11-14. The container lacked `cmake`, `ninja`, `colcon`, ROS and the system Eigen in the morning and had them again in the afternoon, after the first three checks.

1. Syntax. `sqrt_keypoint_vio.cpp`, `logger.cpp`, `vio_config.cpp`, `marg_helper.cpp`, `controller.cpp`, `nfr_mapper.cpp`, `vio_estimator.cpp` and `test_vio.cpp` pass `-fsyntax-only` with the recorded flags, `-Werror` included, both instantiations and the vendored Eigen 3.4.0, by the recipe of `/ws/skills/cpp-syntax-check-tooling.md`.
2. Unit tests. The coarse job is header only and needs no `MargHelper` when only `Coarse` runs, so the two tests of Change 16 were compiled out of `test_vio.cpp` with the vendored googletest, `-DFMT_HEADER_ONLY` and `-DEIGEN_INITIALIZE_MATRICES_BY_NAN`, and both passed. The readvance that follows the coarse optimisation in `Run` is the forward pass of Change 4, unchanged, and was not exercised.
3. Solver convergence. On a trajectory with spline knots 0.5 s apart and accelerations up to 30 m/s², exact data with the SITL noise densities converged in three or four iterations to a log scale of 0.46999 against a true 0.47000, with a final cost of $1.1 \times 10^{-4}$. With an inertial noise of 0.01 m/s² per sample the log scale parameterisation stopped after 20 iterations at 0.423, and the inverse scale parameterisation at 0.427, which led to the inverse scale as the unknown and to the cap of 30 iterations (§2.3.11 Step 5).
4. Library build and unit tests. The library and `test_vio`, `test_qr`, `test_linearization` and `test_nfr` were built standalone in the session scratchpad with both instantiations by recipe 1 of `/ws/skills/cpp-syntax-check-tooling.md`, without error, and all passed, `test_vio` with 10 tests including the two of Change 16 and the three of Change 10, `test_qr` with 3, `test_linearization` with 4 and `test_nfr` with 4.
5. Monte Carlo against the triple form. Forty random trajectories with spline knots 2 s apart, positions and velocities shrunk by 1.6, Gaussian position jitter, exact inertial data with the SITL noise densities, the tilt prior of 0.75° and the accelerometer bias prior of the SITL configuration, and the scale gate open. The error is the estimated log scale minus $\ln 1.6$.

| Store | Jitter | Present form, mean | Present form, rms | Present form, tilt | Triple form, mean | Triple form, rms | Triple form, tilt |
|---|---|---|---|---|---|---|---|
| 200 frames, $k$ = 10 | 1 cm | +0.0003 | 0.0033 | 0.031° | −0.0023 | 0.0070 | 0.067° |
| 200 frames, $k$ = 10 | 2 cm | +0.0007 | 0.0067 | 0.062° | −0.0045 | 0.0140 | 0.135° |
| 200 frames, $k$ = 10 | 4 cm | +0.0013 | 0.0134 | 0.124° | −0.0098 | 0.0289 | 0.279° |
| 41 frames, $k$ = 4 | 1 cm | +0.0055 | 0.0328 | 0.084° | −0.0829 | 0.5287 | 0.886° |
| 41 frames, $k$ = 4 | 2 cm | +0.0112 | 0.0661 | 0.166° | −0.1222 | 0.7350 | 1.769° |
| 41 frames, $k$ = 4 | 4 cm | +0.0229 | 0.1361 | 0.330° | +47.19 | 296.05 | 3.190° |

The mean accelerometer bias error was 0.003 to 0.012 m/s² for the present form against 0.007 to 0.028 m/s² for the triple form over 200 frames. Over a full store the present form halves the scale, tilt and bias errors, and its mean error stays within one standard error of zero, so it is unbiased within the resolution of forty trials. Over 2 s with short spans of 0.2 s the triple form is unstable, with one divergent trial at 4 cm, while the present form degrades gracefully. The triple form's runs used the header of 2026-11-13 unchanged, with its displacement threshold and constant whitening.

Outstanding, in addition to §3.4.4.

1. The host build, and a unit test of the readvance that follows a coarse optimisation, which no test exercises with stored visual rows.
2. Replays of `telemetry_35` and `telemetry_36` against the coarse phase timelines of §5.3, and of `telemetry_34` and `telemetry_100` against the table of §3.4.5, with the recipe of §3.4.5. The flights of §5 ran the triple form, so they are the baseline.
3. In those replays, the first corrections after the motion flag. Spans recorded on the ground now enter, and while the snapshot gate holds the scale at rest, a wrong tilt or bias multiplied by the inverse scale could pull the scale upwards once the gate opens. If it does, restore a per span displacement threshold of about 0.1 m, half of the former 0.2 m over two spans.
4. In those replays, `coarse_bg` against the window's gyroscope bias. The rotation rows are whitened by the preintegration covariance, about $5.4 \times 10^{-4}$ rad per frame with the SITL `gyro_noise_std`, while factors touching a departed keyframe showed whitened rotation residuals of 30 to 200 (§3.5 item 10). The jitter cancels in a common bias to first order (§2.3.11 Step 4), but if $\delta\mathbf{b}_g$ is erratic the rows need a constant standard deviation of a few milliradians or a Huber weight.
5. In those replays, `coarse_max_vel_inc` and the cost of the installed prior, to confirm that the velocity increments stay within the noise of a span.

#### 3.4.7 Checks of Change 17, run on 2026-11-14

1. Build and unit tests. The library and `test_vio`, `test_qr`, `test_linearization` and `test_nfr` were built standalone in the session scratchpad with both instantiations and the build's `-Werror`, and all passed, `test_vio` with 11 tests including the three of Change 17.
2. Syntax. The eleven translation units of the `slam` package that include `vio_config.h`, `logger.h`, `sqrt_keypoint_vio.h` or the store's header, among them `controller.cpp` and `src/basalt/slam.cpp`, pass `-fsyntax-only` with the recorded flags and the system Eigen.
3. Monte Carlo of §2.3.11 Step 8, with the timing of one coarse job on a 32 core container, 0.6 ms to 3.6 ms while moving and up to 23 ms at rest at a keyframe spacing of 2 frames.

Outstanding, in addition to §3.4.4 and §3.4.6.

1. Replays of `telemetry_34`, `telemetry_35`, `telemetry_36` and `telemetry_100`, which are the comparison this change was made for. Check the scale against ground truth after each coarse correction, `coarse_timed_out` and `wait`, the number of keyframes per job, and whether Basalt's keyframe spacing at lift off is close enough for the bias of §2.3.11 Step 8 to matter.
2. Whether a quarter of the frame period is the right deadline on the host, whose cores differ from the container's.

### 3.5 Review of the plan against the code

The plan was reviewed against the code and the runs before it was finished, to find anything that would stop a one shot implementation. Each finding was folded back into the changes or the derivations, as listed.

1. `computeDelta` asserts that every state of its order is frozen (`ba_base.cpp:327-334`), and the marginalisation order contains `last_state_to_marg` before it is frozen. The capture of Change 6 therefore reads the offsets directly, which is exact because an unfrozen state has a zero offset.
2. A departing state of the first marginalisation of the period is not frozen, and a record built from it with Basalt's constructor would move its first estimate when the smoothing applies an increment. `DelayedMarginalisation::Frozen` builds every record as frozen with the state's first estimate and offset, which makes the increment an offset.
3. `IntegratedImuMeasurement::get_sqrt_cov_inv` writes a mutable cache on first use, which would be a data race between the worker and the VIO thread. `Push` fills the cache before the step is shared.
4. The main prior's order is a prefix of the next marginalisation's order with equal offsets, which `MarginaliseStep` relies on when it places the previous prior's rows. Basalt asserts this for its own prior at every marginalisation, and the install asserts that the readvanced prior's order equals the main prior's.
5. The visual rows of the store were first linearised at the main prior's anchor. Plan 2 showed that a scale anchor drifting from the current scale shrinks the visual translation information, so they are linearised at a fresh anchor at the current block (§2.3.3).
6. With the delay longer than the time to the first replacement, the replacement's first step is the first step of the period, so the end of replacements is reached $0.5\,d$ steps after the store fills, not after the replacement. This is DM-VIO's rule applied to Basalt's step counting, and it was observed at about 66 s in every held run.
7. The mapper conflict of §2.3.8 was not anticipated by `context/dynamic_marginalisation.md` §6.3 Q4, which expected a readvance to remove the mismatch left by accepted pose updates. The runs showed that it imports the mismatch into the inertial factors instead, which led to the hold of Change 1 and Change 6.
8. The logger loses the rows after its last periodic flush when a run is stopped by a signal (`logger.cpp:200`), which hid the settle rows of every replay. The settle times above were inferred from `lost_steps` in the last flushed row, 88 at 65.69 s, and the 54 ms step.
9. The revision of 2026-11-13 found `rsync` and `patch` absent after a container rebuild, and the commands that rebuilt the scratch copy began with `cd` and a semicolon, so the revision was applied to the working tree instead of a copy. The tree it produced is the one the replays of §3.4.5 ran, because the base had not changed, and `find . -name '*.orig' -delete` ran at the root of `ext/basalt`, which deletes no tracked file but would have deleted an untracked one.
10. Departed keyframe states sit about 5 cm from their neighbours, because the window keeps optimising a keyframe's pose after its neighbours left, and at rest the held positions and velocities carry this jitter and no motion. A coarse optimisation over per frame factors with every pose and velocity held then prefers a scale near zero, which removes the jitter terms and leaves factors consistent with rest, and the jitter also makes the scale variance small, so the gate cannot catch it. The first replay collapsed the scale to $e^{-20}$ and aborted after numerical failures in back substitution. A diagnostic printed whitened rotation residuals of 30 to 200 at the current values against 5 to 20 at the first estimates for every factor touching a keyframe, which located the cause. The three node form of §2.3.11 replaced it.
11. The three node form first solved $s\,\boldsymbol{\lambda} = \kappa\,\mathbf{g}_D + \boldsymbol{\gamma}$ with $\boldsymbol{\lambda}$ as the regressor. Each correction shrank the scale by the same attenuation, from 1 to 0.75, 0.50, 0.21 and 0.12 within 6 s, while the correct cumulative scale was about 0.3, and the delayed phase recovered it only at 35 s through a cascade of readvances. Predicting the visual side removed the bias (§2.3.11 Step 4).
12. `IntegratedImuMeasurement` keeps its bias linearisation point private, so the bias corrected deltas are read from `residualGS` at identity states, zero gravity and zero scale (§2.3.11 Step 1).

## 4. Conclusion

Dynamic marginalisation keeps the period's prior consistent by discarding the inertial factors that a scale change would make inconsistent, and at take off it discards almost all of them. Delayed marginalisation keeps the last $d$ marginalisations with their original inertial factors, and when the block has moved by 2 % in scale or 5° in tilt it rebuilds the prior with every one of those factors relinearised at the current block. In Basalt that rebuild is a re-anchoring of the block in the inertial rows, because the per frame re-centring already carries every state, prior and first estimate exactly. The visual rows are kept linear and scale free, the delayed prior keeps the scale free inertial rows of Change A, and a backward pass through the same factorisation smooths the departed history given the window, which is Basalt's counterpart of DM-VIO's pose graph bundle adjustment. A worker thread does the readvance, so the VIO thread pays about 0.1 ms per installation, and the store costs less per frame than the three priors it replaces.

The replays established three things. The store reproduces the main prior's information exactly where it should. The local mapper's visual only pose updates conflict with the inertial history, and a readvance that honours them collapses the scale, so the updates must be held while the store is active. With the hold, the delayed mode settles the take off with one replacement in all but one replay and ends the replays at scale ratios of 0.84 to 1.85, against 3.2 to 5.8 for dynamic marginalisation on the same binary. The live climbs of `sitl2`, whose lift off is kinematically inconsistent, remain the test that decides whether this is enough.

The revision makes delayed marginalisation the period's only mechanism and starts it at the first frame. A coarse inertial only optimisation after DM-VIO's coarse initialiser, with the poses held and the visual side predicted, corrects the scale, tilt, both biases and the velocities of a running metric estimator, and readvances the prior at the corrected values in the same job, until the store holds its delay, after which the window's block, the smoothed readvance and the replacement rule take over, and the store is kept for the whole period. On two flights where Basalt alone over scales by a factor of 3 to 6 or diverges, it held the scale ratio near 1 and the tilt within about 2° from 5 s after lift off. Unit tests, host builds and live flights remain (§3.4.4). Note, 2026-11-14. The two preceding sentences previously described the coarse optimisation after Mur-Artal and Tardós, and the flight results they quote were obtained with that form, which Change 16 replaced after it halved the scale error in simulation (§3.4.6).

The re-anchoring view of a readvance under per frame re-centring, the square root smoother through the readvance's own eliminated rows, and the conflict between a visual only mapper's pose feedback and readvanced inertial factors were not found in the papers read, and their novelty would need a wider search before it is claimed.

## References

[1] L. von Stumberg, V. Usenko, D. Cremers, "Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization", IEEE ICRA 2018, arXiv:1804.05625. Read §III-E and §III-F on 2026-11-06 (`context/dynamic_marginalisation.md`, reference [1]).

[2] L. von Stumberg, D. Cremers, "DM-VIO: Delayed Marginalization Visual-Inertial Odometry", IEEE Robotics and Automation Letters, vol. 7, no. 2, 2022, arXiv:2201.04114. §III-B to §III-F and §IV-A read on 2026-11-06, and §III read again on 2026-11-10 through the ar5iv rendering at https://ar5iv.labs.arxiv.org/html/2201.04114, which returned the numbered equations and a summary of the text. Quotations are taken from the reading of 2026-11-06 recorded in `context/dynamic_marginalisation.md` §3.3. §III-E and §III-F were read again on 2026-11-14 through the same rendering for the coarse initialiser's variables and the transfer of the pose graph bundle adjustment by readvancing, which are quoted in §2.3.11 and §2.3.14.

[3] N. Demmel, D. Schubert, C. Sommer, D. Cremers, V. Usenko, "Square Root Marginalization for Sliding-Window Bundle Adjustment", IEEE/CVF ICCV 2021, cited through plan 2, reference [10].

[4] G. P. Huang, A. I. Mourikis, S. I. Roumeliotis, "Analysis and Improvement of the Consistency of Extended Kalman Filter based SLAM", IEEE ICRA 2008. Abstract read (`context/dynamic_marginalisation.md`, reference [16]).

[5] V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers, "Visual-Inertial Mapping with Non-Linear Factor Recovery", IEEE Robotics and Automation Letters, 2020, arXiv:1904.06504, the paper of Basalt's estimator. Cited through `context/vio_opt.md`, reference [2].

[6] L. von Stumberg, DM-VIO source code, https://github.com/lukasvst/dm-vio, cloned at commit `e306328098e08facbec1bfd45d22e19c77a3ccad` on 2026-11-10 into the session scratchpad. `src/GTSAMIntegration/DelayedMarginalization.{h,cpp}`, `src/IMUInitialization/*`, `src/IMU/IMUIntegration.cpp`, `src/IMU/BAIMULogic.cpp` and the relevant part of `src/dso/FullSystem/FullSystem.cpp` read in full.

[7] R. Mur-Artal, J. D. Tardós, "Visual-Inertial Monocular SLAM with Map Reuse", IEEE Robotics and Automation Letters, vol. 2, no. 2, 2017, arXiv:1610.05949. §IV read on 2026-11-13 through https://ar5iv.labs.arxiv.org/html/1610.05949, the velocity elimination over three consecutive keyframes (eq. 12) and the joint solution for scale, two tilt angles and accelerometer bias at known gravity magnitude (eq. 19).

[8] C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, J. D. Tardós, "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM", IEEE Transactions on Robotics, 2021, arXiv:2007.11898. §V-B read on 2026-11-13 through https://ar5iv.labs.arxiv.org/html/2007.11898. Its inertial only optimisation holds the up to scale visual trajectory constant and estimates the scale, the gravity direction, both biases and the keyframe velocities after 2 s of keyframes at 4 Hz, and visual inertial bundle adjustments follow 5 s and 15 s after initialisation, with a scale refinement every 10 s afterwards.

[9] L. von Stumberg, DM-VIO source code, `src/IMUInitialization/CoarseIMUInitOptimizer.cpp`, read on 2026-11-14 at https://raw.githubusercontent.com/lukasvst/dm-vio/master/src/IMUInitialization/CoarseIMUInitOptimizer.cpp. Between consecutive keyframes it adds a GTSAM `ImuFactor` on the poses, velocities and one `imuBias::ConstantBias` of both sensors, holds the poses as fixed values when `settings.fixPoses` is set, initialises the velocities and the bias at zero, and solves with `LevenbergMarquardtOptimizer`.

[10] C. Campos, J. M. M. Montiel, J. D. Tardós, "Inertial-Only Optimization for Visual-Inertial Initialization", IEEE ICRA 2020, arXiv:2003.05766. §II and §III read on 2026-11-14 through https://ar5iv.labs.arxiv.org/html/2003.05766. Its state is the scale, the gravity direction, both biases and the up to scale keyframe velocities with the trajectory fixed, every inertial residual is weighted by its preintegration covariance, the accelerometer bias alone has a prior, keyframes are forced at 4 Hz to 10 Hz, and the result seeds a full visual inertial bundle adjustment.

[11] D. Zuñiga-Noël, F.-A. Moreno, J. Gonzalez-Jimenez, "An Analytical Solution to the IMU Initialization Problem for Visual-Inertial Systems", IEEE Robotics and Automation Letters, 2021, arXiv:2103.03389. Read on 2026-11-14 through https://ar5iv.labs.arxiv.org/html/2103.03389. It estimates the gyroscope bias first, then eliminates the velocities over three consecutive keyframes with covariance weighted residuals, and compares with [10].

## 5. Flight analysis of 2026-11-13 and the tuning it points to

This section analyses the first two live `sitl2` flights with Changes 12 to 15, `/ws/logs/slam/2026-11-13_22-21-55` (run 1, 178 s) and `/ws/logs/slam/2026-11-13_22-33-36` (run 2, 143 s), both with the configuration of Change 15. Their bags are, by their write times, `/ws/data/telemetry_35` and `/ws/data/telemetry_36`. Times are in seconds from lift off, defined as the first frame at which the ground truth position is more than 0.3 m from its start, which came 4.13 s and 5.27 s after the first frame. The scale ratio is the length of the estimated displacement over a window divided by that of ground truth, and the tilt error the angle between the body frame gravity directions of estimate and ground truth, both from `gt_eval.csv`. The cumulative scale and the cumulative tilt are the columns `scale_cumulative` and `tilt_cumulative_deg` of `vio_gravity_scale.csv`, the composition of every correction since the start. The ground truth kinematics are finite differences of the ground truth positions, so their peaks are approximate.

### 5.1 The flight and the phases of the estimator

Both flights fly the same profile. After lift off the vehicle climbs while orbiting at about 4.3 m/s with a yaw rate of 17° to 23° per second, which gives a horizontal centripetal acceleration of about $4.3 \cdot 0.31 = 1.3$ m/s². From 26 s it climbs vertically at about 4 m/s without turning, from 38 s it flies the square, four legs of 11 m/s to 16 m/s with accelerations of 10 m/s² to 17 m/s² at their starts and ends and stops near 50 s, 62 s, 74 s and 94 s, and from 98 s it cruises at 3 m/s until the descent.

```
 t (s, from lift off)   -5      0   5.5-6.6          26          38                              94   98                 146
                         |------|-----|---------------|-----------|-------------------------------|----|------------------|----▶
 flight                  ground │ climb + orbit       │ vertical  │ square: 4 legs, 11-16 m/s,    │stop│ cruise 3 m/s      │descent
                                │ 4.3 m/s, 18°/s yaw  │ climb     │ |a_h| 10-17 m/s² at the       │    │                   │
                                │ |a_h| ≈ 1.3 m/s²    │ ≈ 4 m/s   │ starts and stops of each leg  │    │                   │
 estimator               ├─ COARSE PHASE ─┤├──────────────────── DELAYED PHASE (block free in the window) ──────────────────────▶
 store                   fills from the first marginalisation, first full (d = 200) at 5.5 s (run 2) and 6.6 s (run 1)
 worker jobs             C C C C C C C C C C C C │P            R R RRRR RRR  R R RR R RR R R   (none after 49 s)
                         every 10 marginalisations │             └ run 2: 9 readvances in 29.9 s to 31.8 s
 scale, run 1 (cum.)     1.00 → 0.92 ───────── 0.91 ──────────── 0.92 ── 0.93 → 0.96 ────────────────────── 0.97
 scale, run 2 (cum.)     1.00 → 1.24 → 1.00 ── 1.00 → 1.02 ───── 1.11 ── 1.13 → 1.28 → 1.20 ─────────────── 1.20
 est/GT ratio, run 2     0.68, 0.84             0.85 to 0.90      0.87 to 1.26      0.91 to 1.10                 1.00 to 1.03
 world tilt (cum.)       0 ─────────────────── rises ≈0.06°/s ── 1.3° to 2.0° ─── flat, 1.4° (run 2), 2.2° (run 1) ──────────
 attitude tilt error     ≤ 1.5° ─────────────── 0.2° to 0.9° ──────────────────── peaks 1.2° to 1.7° at leg ends ── ≤ 1.3°
```

The coarse phase covered only the first 5.5 s to 6.6 s of flight, because the store counts its $d$ steps from the first marginalisation and the vehicle spent 4.1 s to 5.3 s on the ground. The delayed phase began during the orbit, and every readvance after it fell between 28 s and 49 s, in the vertical climb and the first leg of the square. No readvance fired in the last 95 s of either flight.

### 5.2 What the worker does in the coarse phase

Note, 2026-11-14. This section and §5.3 describe the coarse optimisation of Change 13, which the flights ran. Change 16 replaced it by the form of §2.3.11, which estimates the velocities and both biases instead of eliminating the velocities over triples, weights every span by its covariance and inflates the scale variance by the a posteriori variance factor.

The worker runs `DelayedMargJob::Coarse` (Change 13) on a snapshot taken every `vio_gs_dm_coarse_every` = 10 marginalisations once the stationary detector reports motion. The snapshot holds every stored step's inertial factors, the window's inertial factors, the departed records and the window states, the block, the initial gravity direction in the snapshot's epoch and the two prior weights. Nothing in the snapshot changes while the job runs, and the job never touches the estimator.

```
 snapshot (copies)                                   worker, DelayedMargJob::Coarse                          VIO thread, next marginalisation
 ─────────────────                                   ─────────────────────────────                          ────────────────────────────────
 store steps' IMU factors ─┐                         1 order every factor by start time, split where
 window IMU factors ───────┼──▶ factors (t0 → meas)    two factors do not meet (contiguous chains)
 records + window states ──┘   held p, R per frame   2 every 10 factors close a span a→b:
                                                        P_ab += V_ab Δt_f + R_f Δp_f,  V_ab += R_f Δv_f,  T_ab += Δt_f
                                                        (Δp_f, Δv_f bias corrected, read from residualGS at
                                                         identity states, zero gravity, zero scale)
                                                     3 consecutive spans (a,b),(b,c) form a triple if
                                                        s0 |p_c − p_a| ≥ 0.2 m  (none on the ground)
                                                     4 per triple, velocities eliminated:
                                                        λ = T_ab(p_c − p_b) − T_bc(p_b − p_a)
                                                        κ = ½ T_ab T_bc (T_ab + T_bc)
                                                        γ = T_ab P_bc − T_bc P_ab + T_ab T_bc V_ab
                                                        r = s0 (λ − e^{−σ}(κ g_D + γ)) / κ,  whitened by 0.1 m/s²
 block (d, σ0), initial gravity,                     5 Gauss–Newton, 10 iterations, unknowns δθ (2), δσ, δb_a (3)
 prior weights w_g, w_b ──────────────────────────▶     + tilt prior towards the initial gravity (0.75°)
                                                        + accelerometer bias prior (0.05 m/s²/√s over 10 frames)
                                                     6 gate: var(δσ) from the first iteration > 1.0 → hold σ
                                                     7 output d*, σ*, δb_a, var, costs, triples ─────────────────▶ InstallCoarseJob: map to the current
                                                                                                                   epoch, set the block, RecentreGravity-
                                                                                                                   Scale, add δb_a to every state and
                                                                                                                   record, request a readvance (no smoothing)
```

Every coarse optimisation fits all triples since the store began, so the later ones refit the earlier ones together with one new triple, and each correction is relative to the state left by the previous correction. A job took well under a millisecond. The coarse optimisation therefore costs nothing in time, and its quality is set by the triples it sees and by how it weighs them.

### 5.3 The coarse phase, corrections against ground truth

The ground truth implied correction below is $-\ln$ of the scale ratio of the estimate over the 2 s before an installation, taken from frames before it, so it is the correction that would have made that stretch metric. Over the first second after lift off the ground truth displacement is short and this value is noisy.

| Run | Time (s) | Applied $\delta\sigma$ | Implied, last 2 s | Triples | Variance of $\delta\sigma$ | A posteriori residual |
|---|---|---|---|---|---|---|
| 1 | 0.73 | +0.012 | +0.33 | 2 | 0.0032 | 0.13 m/s² |
| 1 | 1.26 | +0.029 | +0.23 | 3 | 0.0026 | 0.24 m/s² |
| 1 | 2.87 | +0.027 | +0.07 | 6 | 0.0011 | 0.39 m/s² |
| 1 | 4.98 | +0.006 | +0.04 | 10 | 0.0009 | 0.37 m/s² |
| 1 | 6.57 | +0.001 | +0.03 | 13 | 0.0007 | 0.42 m/s² |
| 2 | 0.53 | +0.218 | +0.58 | 2 | 0.0198 | 0.52 m/s² |
| 2 | 1.06 | −0.096 | +0.39 | 3 | 0.0128 | 0.39 m/s² |
| 2 | 2.11 | +0.269 | +0.30 | 5 | 0.0091 | 0.22 m/s² |
| 2 | 2.64 | −0.290 | −0.04 | 7 | 0.0023 | 0.46 m/s² |
| 2 | 4.20 | −0.059 | +0.27 | 9 | 0.0015 | 0.46 m/s² |
| 2 | 5.24 | +0.033 | +0.18 | 11 | 0.0012 | 0.52 m/s² |

The a posteriori residual is $0.1\sqrt{2 c / (3n - 6)}$ m/s² with $c$ the final cost and $n$ the number of triples. Four findings follow.

1. The data are two to five times noisier than the 0.1 m/s² that `kCoarseAccelStd` assumes, 0.13 m/s² to 0.52 m/s² a posteriori. The variance of $\delta\sigma$ is therefore 4 to 27 times too small, and the gate of 1.0 never held the scale after the first triple. A variance of 0.02 is a standard deviation of 14 % in scale, which DM-VIO's threshold, defined for its own whitening, does not reject either.
2. Run 2 found the right correction at 2.11 s, +0.27 against an implied +0.30, and undid it at 2.64 s with −0.29 against an implied −0.04, when two new triples raised the cost from 25 to 206. Two triples dominated the solution, which is the signature of outliers in an unweighted least squares fit. The first triples of a flight contain the lift off itself, where ground contact, the hop of the first metre and the 5 cm keyframe jitter of §3.5 item 10 meet.
3. After 3 s every refit was anchored by its earlier triples and moved by less than 0.06, while the implied correction stayed between +0.13 and +0.27 in run 2. Run 2 left the coarse phase with its scale about 15 % low (cumulative 1.00, later found to be 1.20), and run 1 about 4 % low.
4. The first correction of run 1, −0.10 at 0.21 s from a single triple, went against the direction implied a moment later, and the scale left it at 0.92. In run 1 the early corrections were far smaller than implied (+0.012 against +0.33 at 0.73 s) because two or three triples over less than 2 m of displacement carry little information against the priors. The phase ended before the orbit had produced enough triples to finish the correction.

### 5.4 The orbit, 6 s to 26 s

```
 Δσ per 4 s, run 2    +0.001  +0.004  +0.002  +0.005  +0.005      est/GT  0.85 0.85 0.86 0.88 0.90
 Δtilt per 4 s        −0.18   +0.35   +0.34   −0.15   +0.33       tilt error ≤ 0.9°
```

The block barely moved in scale for 20 s, so the coarse phase's error persisted, 10 % to 15 % low in run 2. The excitation of the orbit is a centripetal acceleration of about 1.3 m/s² that rotates with the heading, which over the window of `vio_max_states` 3, two inertial factors over about 0.1 s, changes the velocity by about 0.13 m/s, and the readvanced prior, which holds the store's factors evaluated at the coarse phase's scale, presumably outweighs it, which was not measured. The world tilt drifted by about 0.06° per second, 1.3° to 2.0° in total, while the attitude tilt error stayed below 0.9°. A re-centring is a change of variables under which the body's tilt relative to gravity is invariant (D5), so this drift rotates the whole trajectory relative to the ground truth world and does not mis-tilt the body. At 100 m from the origin a world tilt of 2° displaces the trajectory by about 3.5 m.

### 5.5 The vertical climb, 26 s to 38 s

```
 run 2, block log scale step per frame (selected frames, anchor offset at the step, readvance R)
 28.52  +0.041 ─R─▶ 28.57 +0.007   28.62 −0.009
 30.28  −0.032 ─R─▶ 30.38 +0.040 ─R─▶ 30.49 −0.048 ─R─▶ 30.65 +0.006
 33.40  +0.042 ─R─▶ 33.50 −0.008   33.77 −0.012 ─R─▶ 33.88 +0.025 ─R─▶
 net over 26 s to 38 s  +0.11 (scale 1.02 → 1.13), est/GT 0.87 → 1.12 → 0.99 → 1.26
```

A climb at constant velocity carries no scale information, yet the block took single frame steps of ±4 % that reversed within one to three frames. Each crossed $\ln 1.02 = 0.0198$ from the freshly installed anchor and triggered a readvance, nine of them between 29.9 s and 31.8 s in run 2, and each readvance relinearised the store at the peak of a transient. Frame timing is not the cause, because the frame intervals, the inertial sample counts and the coverage on the jump frames equal those of quiet frames (`vio_imu_frame.csv`). The jump frames differ in their inertial content. Their largest within frame deviation of the accelerometer from its mean is 1.09 m/s² against 0.41 m/s², and their mean vertical specific force 9.59 m/s² against 9.96 m/s², so the steps follow thrust transients of the climb, where a single inertial factor in a two factor window sees a vertical acceleration that the visual side cannot yet confirm. Change B allows a per frame step with a standard deviation of $0.6\sqrt{0.054} = 0.14$, so a step of 0.04 costs $\frac{1}{2}(0.04/0.14)^2 = 0.04$ and is not restrained. The net effect in run 2 was the right direction, from 15 % low towards metric, but with overshoot to a ratio of 1.26.

### 5.6 The square, 38 s to 94 s, and the cruise

```
 leg            1 (38-50)        2 (50-62)        3 (62-74)        4 (78-94)        cruise (98-146)
 |a_h| peak     14 m/s²          17 m/s²          15 m/s²          16 m/s²          < 1 m/s²
 run 2 scale    1.11→1.28→1.20   1.20             1.20             1.20             1.20
 est/GT         0.96-1.26        0.91-1.10        0.92-1.19        0.87-1.23        0.84-1.19 to 106, then 1.00-1.03
 tilt error     0.4°-1.5°        0.8°-1.7°        0.6°-1.5°        0.8°-1.5°        0.3°-1.3°
 block Δtilt    +0.05°           0                0                0                ≤ +0.03°
```

The first leg's accelerations of 10 m/s² to 17 m/s² settled the scale within about 8 s, 1.11 to 1.28 and back to 1.20 in run 2, after which the block did not move and the scale ratio stayed between 0.87 and 1.23 per 4 s window, and between 1.00 and 1.03 in the cruise from 106 s. The extreme ratios fall in the windows of the stops, where the ground truth displacement is short and the ratio is ill conditioned. The tilt errors of 1.2° to 1.7° at the ends of the legs are not the block's, which was still, but the window's attitude during strong decelerations, and they recovered on every leg. In the cruise of run 1 the accelerometer bias drifted from (0.016, 0.013) m/s² to (−0.031, 0.030) m/s² in its horizontal components and the tilt error rose from 0.65° to 1.26°, which is the slow bias tilt trade of a flight without excitation.

### 5.7 The variables behind each change

| Change | When | Driving variable | Evidence |
|---|---|---|---|
| Scale jumps of ±0.1 to ±0.29 | coarse phase, 0 s to 5 s | few triples, unweighted outliers, data noise 2 to 5 times `kCoarseAccelStd` | §5.3 items 1 to 3 |
| Residual scale error of 4 % to 15 % | 6 s to 26 s | coarse phase too short, refits anchored by early triples, weak excitation of the orbit in a two factor window | §5.3 items 3 and 4, §5.4 |
| Single frame scale steps of ±4 % and readvance bursts | vertical climb | thrust transients in one inertial factor, Change B permissive at 0.6/√s, readvance threshold crossed by transients | §5.5 |
| Scale convergence with overshoot | first square leg | strong horizontal acceleration, the first real scale information | §5.6 |
| World tilt drift of 1.4° to 2.2° | orbit and climb | block tilt steps under Change B at 0.75°/√s with a rotating centripetal acceleration | §5.4 |
| Attitude tilt error peaks of 1.2° to 1.7° | ends of the legs | window attitude under strong deceleration, not the block | §5.6 |
| Slow tilt error growth | cruise | accelerometer bias drift without excitation | §5.6 |

### 5.8 Next steps for tuning

The steps are ordered by the size of the error they address, and each is tested by replaying `telemetry_35` and `telemetry_36` with the recipe of §3.4.5 against the timelines above.

1. Make the coarse variance honest. Change 16 scales the variance of the log scale by the a posteriori variance factor of the translational rows before the gate (§2.3.11 Step 6). What remains is to tighten `vio_gs_dm_coarse_scale_var` to about $2.5 \times 10^{-3}$, a standard deviation of 5 % in scale. Expected effect, the corrections of the first two seconds of run 2 held until enough spans agree. Note, 2026-11-14. This item previously proposed the variance factor $2c / (3n - 6)$ of the triples or raising `kCoarseAccelStd` to about 0.4 m/s², which Change 16 removed. Note, 2026-11-14. Change 17 removed the variance factor again, since DM-VIO gates on the variance at the solution, so this item is open once more for the gate's threshold.
2. Make the coarse fit robust. A Huber loss on each span's whitened residual, at about twice the a posteriori residual, keeps a few spans from undoing a correction as two triples did at 2.64 s in run 2. Spans that contain the first metre of the lift off can also be excluded by starting the chains at the motion flag. Note, 2026-11-14. This item previously named triples, which Change 16 replaced by spans with free node velocities, and it was not implemented. With Change 17 the residuals join keyframes, and DM-VIO uses no robust kernel in its coarse initialiser.
3. Count the coarse phase from the motion flag instead of the first marginalisation, so that it spans $d \Delta t = 10.8$ s of flight whatever the time on the ground (§2.3.9 caveat). In both flights it would then have included the whole orbit.
4. Lower `vio_gs_jump_sigma_rw` from 0.6 to about 0.15 per square root second, a per frame standard deviation of 3.5 %, so that a 4 % step on one thrust transient costs about 0.65 instead of 0.04. The genuine correction of run 2 took 8 s to 12 s, over which 0.15 still allows a standard deviation of about 0.45 in log scale.
5. Make the readvance trigger ignore transients, by requiring the anchor offset to exceed $\ln\theta_s$ for several consecutive frames or by testing a low pass filtered block, or by raising `vio_gs_dm_thresh_scale` to about 1.05. Expected effect, one readvance per real change instead of bursts.
6. Lower `vio_gs_jump_tilt_rw_deg` from 0.75 to about 0.3 degrees per square root second, which bounds the world tilt drift of the orbit at a rate the gravity prior can hold, while the attitude error, which this drift does not cause, is unaffected.
7. Leave the delayed phase's thresholds otherwise unchanged until steps 1 to 5 are replayed, since after the first square leg the period was stable in both flights with no readvance in its last 95 s.
