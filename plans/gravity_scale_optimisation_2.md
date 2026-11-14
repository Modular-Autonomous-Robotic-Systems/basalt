# Gravity Direction and Scale Optimisation, Revision 2. Keeping the Inertial Prior and Bounding the Per Frame Correction

Status, 2026-11-06. Approved and implemented, Changes 1 to 9, with both changes enabled in `data/sitl_config_vo.json`. Every snippet of §4.2 was first applied to a scratch copy of `include/`, `src/` and `test/` and syntax checked under GCC 12 against the build's compile flags, then applied unchanged to the working tree. The unit test of §5.3 and the replay matrix of §5.4 are outstanding. Two smoke replays with both changes on are recorded in §5.5. Note, 2026-11-06. This line previously read that the plan awaited review and that nothing had been implemented. It extends `plans/gravity_scale_optimisation.md` (called the first plan below), whose Changes 1 to 17 are implemented in the working tree of branch `shreyas/kf_driven_mapping_and_logger` on top of `7e91ec4`. Every `file:line` anchor below was read against that working tree on 2026-11-06. The document is written to be read on its own. Every result of the first plan and of `context/dynamic_marginalisation.md` that the new changes rely on is restated here with its derivation, so that no other source is needed to follow or to implement it.

## 1. Introduction

### 1.1 The problem

Basalt fixes the direction of gravity once, from the first accelerometer sample (`src/vi_estimator/sqrt_keypoint_vio.cpp:248`), and never represents metric scale as a variable. On the vertical take off of every SITL flight the first seconds cannot resolve either quantity, and the error is absorbed by the accelerometer bias and frozen into the marginalisation prior (`context/takeoff_bias_divergence.md`). The first plan added an initialisation period in which a three dimensional block, two tilt coordinates of the gravity direction and a log scale, is optimised jointly with every pose, velocity, bias and landmark. The block enters the inertial residual explicitly and the visual residual through the extrinsic lever arm, the three gauge freedoms it creates are removed by an orthogonal projection, the estimated correction is applied to every state and prior at the end of each frame and the block reset to the identity, the marginalisation follows the three prior scheme of VI-DSO [1], and an epoch stamped correction log keeps the local mapper consistent.

The replays of 2026-11-06 (first plan §4.3, record of runs) showed that the scale part works and the tilt part fails.

| Run | Tilt error after take off | Scale ratio after the period | Outcome |
|---|---|---|---|
| Mode off | 2.4° to 6° | 2.1 to 2.7 | stable, scale too large |
| Mode on, 10 s period | 26° to 27° | 0.86 to 1.08 | scale corrected, tilt diverged |
| Mode on, 70 s period, strong gyroscope bias prior | 74° to 113° | 10 to 19 | diverged |

Four observations from the per frame record define the problem. At take off single solves return steps of up to $\delta\sigma = -0.56$ and 42° of tilt. These steps cross the swap interval of VI-DSO at almost every frame, so the prior in use is replaced at almost every frame. The horizontal accelerometer bias equals $|\mathbf{g}| \sin(\text{tilt error})$ to within about 0.1 m/s² at every sample, mode off included. In the first isolated run the bias drift began 0.3 s after the last of 24 swaps. The user's working configuration now sets `vio_gs_duration_s` to 7000 s (`data/sitl_config_vo.json:50`), so the period is effectively permanent and every mechanism below must remain sound for the whole flight, not only for ten seconds.

### 1.2 Findings that shaped this revision

Five findings, each derived in §3, decide the design. Three of them correct statements made earlier, and those statements have been corrected in place in the documents that made them.

1. The prior $M_\text{visual}$, the fallback of every swap, holds no information on the tilt and no absolute anchor of the accelerometer bias. At constant attitude the tilt and the horizontal accelerometer bias form an exact null direction of every inertial factor (§3.2.3), so after a swap the tilt is free to trade one for one against the bias. This is the mechanism of the tilt divergence.
2. VI-DSO's swap rule tracks the scale alone. A prior that may survive any swap must therefore be independent of the scale, but it need not be independent of the tilt, because the tilt enters the inertial residual through $\mathbf{g}_D$ with a first estimate error of second order (§3.3.1). `context/dynamic_marginalisation.md` §4.1 had concluded that the ground phase inertial rows could not enter $M_\text{visual}$ because they depend on the tilt, which is now corrected there.
3. The rows of an inertial factor that are free of the scale cannot be read off the whitened residual, because Basalt whitens the nine rows with a lower triangular factor of the full covariance that mixes position, rotation and velocity. A rotation only factor must be weighted with the marginal covariance of the rotation residual (§3.3.3).
4. The information the projected system holds on the block alone is identically zero (§3.2.7), so a gate that releases the block when that information exceeds a threshold, proposed in `context/dynamic_marginalisation.md` §4.5.4, cannot work and has been corrected there. A random walk residual on the per frame step, added before the projection, gates the block softly instead (§3.4).
5. `vio_init_ba_weight` reaches the gyroscope bias, because the constructor writes it to tangent indices 9 to 11 (`sqrt_keypoint_vio.cpp:114-117`, the transposition recorded in `context/linearisation.md`). The runs labelled "strong bias prior" on 2026-11-06 stiffened the gyroscope bias, and the accelerometer bias kept `vio_init_bg_weight` $= 10^2$, a standard deviation of 0.1 m/s². The leak of the bias anchor inferred from those runs is therefore not established. The first plan's record and `context/dynamic_marginalisation.md` were corrected in place on 2026-11-06.

### 1.3 The changes

Two independent changes follow, each behind its own configuration key whose default reproduces the present behaviour exactly.

Change A keeps the inertial information through every swap. $M_\text{visual}$ is linearised with the inertial factors in a scale free form, namely every inertial factor's rotation rows weighted by their marginal covariance, the bias random walk rows, and, for a factor whose whitened scale column is small, all of its rows with the scale column set to zero. The last group holds exactly the factors of the stationary phase and of unaccelerated flight, which carry the tilt and the bias anchor and carry no scale information. $M_\text{visual}$ then keeps the relative rotations, the gyroscope bias, the accelerometer bias anchor and the ground phase tilt through every swap, while remaining independent of the scale as VI-DSO requires.

Change B bounds the per frame correction. A random walk residual on the block's step within each frame, the process model of a filter that carries the scale as a state [7], is added to the normal equations before the gauge projection and to all three terms of the cost bookkeeping. Its weight is fixed, so it shrinks the step in proportion to how little the data say about it, which is the soft information gate of finding 4, and it keeps the per frame scale step below the swap interval in the regime where the swaps did harm.

A configuration change sets both in `data/sitl_config_vo.json`. Delayed marginalisation [2], composition of preintegrated factors, scale aware recovered factors, partial re-centring, a hard information gate and removal of the tilt from the block were considered and are not adopted, for the reasons of §3.6.

## 2. Background and Literature

This section describes each line of work the revision draws on, states its idea and the mathematics this plan uses from it, and says what is taken and what is not. The sources of 2026-11-06 (`context/dynamic_marginalisation.md`, references [1] to [9]) are summarised with the results needed here, and the wider survey of the same day adds [10] to [17]. A claim attributed to a paper is limited to what was read, and the reference list states how each was read.

### 2.1 Marginalisation as a Schur complement, and its square root form

Linearise every residual at the current point, $\mathbf{r}(\boldsymbol{\delta}) = \mathbf{r} + \mathbf{J} \boldsymbol{\delta}$, so that the cost is $F(\boldsymbol{\delta}) = \frac{1}{2}|\mathbf{r} + \mathbf{J} \boldsymbol{\delta}|^2$, with $\mathbf{H} = \mathbf{J}^\top \mathbf{J}$ and $\mathbf{b} = \mathbf{J}^\top \mathbf{r}$. Partition the increment into variables kept, $\boldsymbol{\delta}_\alpha$, and variables removed, $\boldsymbol{\delta}_\beta$. Minimising $F$ over $\boldsymbol{\delta}_\beta$ for fixed $\boldsymbol{\delta}_\alpha$ leaves a quadratic in $\boldsymbol{\delta}_\alpha$ with

$$\mathbf{H}' = \mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{H}_{\beta\alpha},$$

$$\mathbf{b}' = \mathbf{b}_\alpha - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{b}_\beta,$$

which §3.1.1 derives step by step. It is the prior of VI-DSO [1, §III-F2], of DM-VIO [2, eq. 10 and 11] and of Basalt (`context/vio_opt.md` §12). Basalt never forms $\mathbf{H}$. `MargHelper::marginalizeHelperSqrtToSqrt` applies a Householder QR to the stacked Jacobian, whose lower right block $\mathbf{J}_\text{new}$ satisfies $\mathbf{J}_\text{new}^\top \mathbf{J}_\text{new} = \mathbf{H}'$ (first plan §2.6.2). Demmel et al. [10] show that this square root marginalisation is algebraically equivalent to the Schur complement, and that for a rank deficient Jacobian it yields the prior of the Schur complement with the Moore–Penrose inverse. This plan relies on that property, since a departing velocity that no factor constrains has a zero column, which the rank threshold of the helper (`src/vi_estimator/marg_helper.cpp:297-298`) drops instead of inverting.

### 2.2 First estimate Jacobians and spurious information

A prior is a linear model at the point where its Jacobians were evaluated. If later factors on the same variables were linearised at different points, their null spaces would disagree with the prior's and the sum would appear to observe a direction that no sensor measured. Huang, Mourikis and Roumeliotis showed for EKF SLAM that evaluating Jacobians at the latest estimates gives the linearised model an observable subspace of higher dimension than the nonlinear system, so the covariance shrinks along directions with no information, and they proposed evaluating every Jacobian at the first estimate of its variable [11] (abstracts and technical report summary read). Basalt freezes every variable that enters a prior and evaluates all its Jacobians there (`setLinTrue`, `sqrt_keypoint_vio.cpp:1475`). VI-DSO states the same requirement, "it is important that Jacobians are all evaluated at the same value" [1, §III-F2]. This plan uses the principle twice. The tilt columns that Change A puts into $M_\text{visual}$ need their own linearisation point of the block, kept across the life of $M_\text{visual}$ (§3.3.5), and the gauge vectors must be built at the same points as the Jacobians (§3.1.3).

### 2.3 Dynamic marginalisation, VI-DSO [1]

VI-DSO evaluates the photometric error in a visual frame related to the metric frame by a translation free similarity, so the scale enters only the inertial factors. Since a prior linearised at one scale is wrong in proportion to the scale change (§3.2.2), it keeps three priors [1, §III-F3]. $M_\text{curr}$, used by the optimisation, holds all factors marginalised since its scale interval was last reset. $M_\text{half}$ holds the factors marginalised while the scale stayed in the half of the interval in which it now lies. $M_\text{visual}$ "contains only scale independent information from previous states of the vision" and is the fallback. With $\sigma$ the log scale, $\sigma_\text{mid}$ the middle of the interval and $\ln d_i$ its half width, Algorithm 1 of [1] runs at every marginalisation.

```
upper ← σ_curr > σ_mid
if upper ≠ lastUpper:                 M_half ← M_visual
if σ_curr > σ_mid + ln d_i:           M_curr ← M_half;  M_half ← M_visual;  σ_mid ← σ_mid + ln d_i
if σ_curr < σ_mid − ln d_i:           M_curr ← M_half;  M_half ← M_visual;  σ_mid ← σ_mid − ln d_i
lastUpper ← upper
```

The interval keeps every factor in $M_\text{curr}$ within $2\ln d_i$ of the current log scale, and $d_\text{min} = \sqrt{1.1}$, so $\ln d_\text{min} \approx 0.048$ (first plan §2.6.6 derives both properties). The scheme assumes that the scale moves slowly compared with $\ln d_\text{min}$ per marginalisation. What this plan takes is the requirement on $M_\text{visual}$, that it be valid at every scale, and it widens what $M_\text{visual}$ may contain under that requirement (§3.3).

### 2.4 Delayed marginalisation and the initialiser gate, DM-VIO [2]

DM-VIO keeps, beside the main graph, "a second, delayed marginalization prior and corresponding factor graph", in which frames are marginalised with a delay of $d = 100$ keyframes in the same order [2, §III-D]. When an updated prior is needed, the delayed graph is readvanced "by successively marginalizing all the variables which have been marginalized in the main graph" [2, §III-E], relinearised at the current estimate. Marginalising one variable at a time equals marginalising all at once, because a minimum over two blocks is the minimum over one of the minimum over the other (§3.1.1). When the scale moves by more than a threshold relative to the scale of the prior's first estimates, $\delta_s = \max(s, s_\text{fej}) / \min(s, s_\text{fej}) > \theta_s$, DM-VIO replaces the prior by the readvanced one, unless more than $\theta_\text{lost} = 50\,\%$ of its IMU factors would be lost [2, §III-F]. The thresholds $\theta_s$, $\theta_\text{init}$ and $\theta_\text{reinit}$ are not given numerically in the text.

The wider reading of 2026-11-06 adds DM-VIO's initialiser, which gates the start of the main system on information. "We compute the marginal covariance for the scale cov(s) and continue to the PGBA if it is smaller than a threshold $\theta_\text{init}$", and after the pose graph bundle adjustment "we again threshold on the marginal covariance for the scale to find out if the optimization was successful" [2, §III-F]. In the PGBA "we do not update our estimates for point depths and do not relinearize photometric error terms" [2, §III-E]. DM-VIO's scale is a well defined quantity with a marginal covariance because its visual frame keeps an arbitrary but fixed scale, so the metric scale is the ratio between two frames that both exist. In this plan the frames are metric and re-centred every frame, and the block is redundant with a uniform rescaling of the frames, so its marginal covariance is not defined (§3.2.7). This plan therefore takes DM-VIO's idea, to let information decide when the correction moves, in the soft form of §3.4. It does not take the delayed graph, for the reasons of §3.6.

### 2.5 Inertial only initialisation and the accelerometer bias prior, Campos et al. [12] and ORB-SLAM3 [8]

Campos, Montiel and Tardós solve gravity direction, scale and biases in a maximum a posteriori problem over keyframes whose poses are held fixed [12, eq. 2 and 5]. The gravity direction is a rotation $\mathbf{R}_{wg}$ with two degrees of freedom, $\mathbf{g} = \mathbf{R}_{wg} \mathbf{g}_I$, and the scale is updated multiplicatively, $s \leftarrow s\,\exp(\delta s)$, "to guarantee that scale factor remains positive" [12, eq. 10 and 11]. The first plan's block uses the same two choices in Basalt's conventions. The point this plan takes is the prior. The cost contains a residual $|\mathbf{b}^a|^2_{\boldsymbol{\Sigma}_p}$ on the accelerometer bias and none on the gyroscope bias, which is "always well observable", because "gravity and accelerometer bias tends to be coupled, being difficult to distinguish in most cases", and "if the motion performed does not contain enough information to estimate the bias, the prior will keep its estimation close to zero" [12, §III]. Attempts whose mean acceleration is below 0.5 % of gravity are discarded as not observable [12]. §3.2.3 derives the coupling for Basalt's residual and shows that the bias prior is the only anchor of the tilt at constant attitude, which is why Change A must carry that prior through every swap. ORB-SLAM3 refines scale and gravity by repeating the inertial only optimisation at about 5 s and 15 s [8] (search snippets read), which also suggests that the correction need not be applied fully at every frame.

### 2.6 Gravity on the tangent plane, VINS-Mono [13]

VINS-Mono refines gravity with its magnitude fixed by re-parameterising it "with two variables on its tangent space", $g\,\hat{\bar{\mathbf{g}}} + w_1 \mathbf{b}_1 + w_2 \mathbf{b}_2$ with $\mathbf{b}_1$, $\mathbf{b}_2$ an orthonormal basis of the tangent plane, and iterates "until $\hat{\mathbf{g}}$ converges" [13, §V-B3]. The first plan's direction $\mathbf{d} = \text{Exp}(\bar{\mathbf{B}} \boldsymbol{\delta}_\theta)\,\bar{\mathbf{d}}$ with a transported basis $\bar{\mathbf{B}}$ (§3.1.1) is the rotation based form of the same two parameter tangent update. VINS-Mono notes that "marginalization results in the early fix of linearization points", which it accepts as a small drift [13, §VI-D], and it states that "acceleration excitation is needed to make metric scale observable" without giving a test [13, §I].

### 2.7 Unobservable directions under special motions [14] and degenerate motions [15]

For visual inertial navigation, the technical report of Wu and Roumeliotis states that the scale direction becomes unobservable when the platform moves with constant local linear acceleration, and that this condition is necessary and sufficient [14] (search snippet read). The intuition given there is that a constant accelerometer bias cannot be told apart from a constant true acceleration. Yang, Geneva, Eckenhoff and Huang list degenerate motion primitives for aided inertial navigation, among them constant local angular and linear velocity [15] (abstract and table of contents read). This plan uses the special case that matters for a single inertial factor, namely that a factor over an interval of zero acceleration has a zero scale column (§3.3.4), so its information is scale free and it may enter $M_\text{visual}$.

### 2.8 Gauge freedom and the covariance of a free gauge, Zhang, Gallego and Scaramuzza [5]

Zhang, Gallego and Scaramuzza compare three treatments of the gauge freedom of visual inertial estimation, fixing the unobservable states, a prior on them, and a free gauge in which the states evolve freely, and find similar accuracy and cost but "dramatically different" covariance estimates, the free gauge covariance being the pseudo inverse of the Fisher information [5] (abstract and repository description read). This is the reason a covariance read off one coordinate of a free gauge problem is not a property of the physical estimate, and §3.2.7 makes the statement exact for the block.

### 2.9 Direct Sparse Odometry's null space projection [4]

DSO projects its seven dimensional gauge out of the step with the projector $\mathbf{P} = \mathbf{I} - \mathbf{N}(\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top$, the default projecting only the step and an option projecting the system as well (`dm-vio/src/dso/OptimizationBackend/EnergyFunctional.cpp:784-829`, read for the first plan). The first plan projects both, and §3.1.3 restates the projector. This plan adds one term before the projection and relies on the projector's properties to show that the term acts only on the physical part of the step (§3.4.3).

### 2.10 Scale as a filter state with process noise, Nützi et al. and Weiss and Siegwart [7]

Monocular visual inertial filters of the EKF era added "the scale factor of the monocular SLAM as an additional variable to the state" [7] (abstract and citing statements read), with a random walk process model. In a smoother the same model is a residual on consecutive values, $r_k = (\sigma_{k+1} - \sigma_k) / \sqrt{q\,\Delta t_k}$ with intensity $q$. Because the first plan re-centres the block to the identity at every frame, the value at the start of a frame is the identity and the residual becomes a penalty on the frame's step. Change B is this residual. A random walk limits the rate of change but supplies no information about the value, so it cannot replace lost information (§3.4.2).

### 2.11 Damping, trust regions and a prior on the step [16]

Levenberg–Marquardt adds $\lambda\,\mathbf{D}$ to $\mathbf{H}$, which is the exact minimiser of the quadratic model plus $\frac{1}{2}\lambda\,\boldsymbol{\delta}^\top \mathbf{D} \boldsymbol{\delta}$ and the solution of a trust region subproblem for some radius [16] (textbook result, cited from memory). Damping is an algorithmic device that leaves the minimum unchanged, since it vanishes at convergence of the iteration. Change B differs in kind. Its residual is part of the objective of the frame, so it changes the minimum of that frame and enters the cost bookkeeping, as a prior on the step does. §3.4.4 derives the bookkeeping.

### 2.12 Non-linear factor recovery [3, 9] and the composition of preintegrated factors [18]

Non-linear factor recovery replaces a dense prior by nonlinear factors whose information minimises the Kullback–Leibler divergence to the marginal, with the closed form $\mathbf{H}_i = (\{\mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top\}_i)^{-1}$ for each recovered factor [3, eq. 27 to 30], derived step by step in `context/dynamic_marginalisation.md` §3.4.2. The result says that the information of a factor that measures a quantity $\mathbf{z}$ is the inverse of the marginal covariance of $\mathbf{z}$. §3.3.3 uses the same statement for the rotation rows of an inertial factor, where it follows directly from marginalising a Gaussian measurement. The composition of two adjacent preintegrated intervals, $\Delta\mathbf{R}_{ik} = \Delta\mathbf{R}_{ij} \Delta\mathbf{R}_{jk}$, $\Delta\mathbf{v}_{ik} = \Delta\mathbf{v}_{ij} + \Delta\mathbf{R}_{ij} \Delta\mathbf{v}_{jk}$ and $\Delta\mathbf{p}_{ik} = \Delta\mathbf{p}_{ij} + \Delta\mathbf{v}_{ij} \Delta t_{jk} + \Delta\mathbf{R}_{ij} \Delta\mathbf{p}_{jk}$, follows from the model of Forster et al. [18, eq. 33 and 38] and is derived in `context/dynamic_marginalisation.md` §3.5.2. Neither is adopted here (§3.6), and they are summarised so that the reasons for that can be judged.

## 3. Mathematical Formulation

§3.1 restates the estimator of the period with the identities every derivation cites. §3.2 derives why the period diverges. §3.3 and §3.4 derive Changes A and B. §3.5 shows that the changes can only improve the present behaviour, up to two bounded approximations, and §3.6 records what is not changed and why.

### 3.1 The estimator in the initialisation period

#### 3.1.1 Variables and update rules

The metric world is gravity aligned, with $\mathbf{g} = (0, 0, -9.81)^\top$ (`include/basalt/utils/imu_types.h:64`) and $\hat{\mathbf{g}} = \mathbf{g} / |\mathbf{g}|$. Frame $f$ holds the body pose $(\mathbf{R}_f, \mathbf{p}_f)$, the velocity $\mathbf{v}_f$ and the biases $\mathbf{b}_{g,f}$, $\mathbf{b}_{a,f}$, with the tangent ordering $[\delta\mathbf{p}, \delta\boldsymbol{\varphi}, \delta\mathbf{v}, \delta\mathbf{b}_g, \delta\mathbf{b}_a]$ and the update rules

$$\mathbf{p}_f \leftarrow \mathbf{p}_f + \delta\mathbf{p}_f,$$

$$\mathbf{R}_f \leftarrow \text{Exp}(\delta\boldsymbol{\varphi}_f)\,\mathbf{R}_f,$$

$$\mathbf{v}_f \leftarrow \mathbf{v}_f + \delta\mathbf{v}_f,$$

and additive updates of the biases (`PoseVelBiasState::applyInc`, `thirdparty/basalt-headers/include/basalt/imu/imu_types.h:221-225`). A keyframe pose has the first six coordinates only.

The gravity and scale block holds the direction of gravity $\mathbf{d}$ on the unit sphere and the log scale $\sigma$, with $s = e^\sigma$ and $\mathbf{g}_D = |\mathbf{g}|\,\mathbf{d}$, stored as a linearisation point $(\bar{\mathbf{d}}, \bar{\mathbf{B}}, \bar{\sigma})$ and an increment $\boldsymbol{\delta}_{gs} = [\delta\theta_1, \delta\theta_2, \delta\sigma]^\top$,

$$\mathbf{d} = \text{Exp}(\bar{\mathbf{B}} \boldsymbol{\delta}_\theta)\,\bar{\mathbf{d}},$$

$$\sigma = \bar{\sigma} + \delta\sigma,$$

where $\bar{\mathbf{B}} = [\bar{\mathbf{b}}_1\ \bar{\mathbf{b}}_2]$ is an orthonormal basis of the plane perpendicular to $\bar{\mathbf{d}}$ and $\boldsymbol{\delta}_\theta = [\delta\theta_1, \delta\theta_2]^\top$ (`GravityScaleStateWithLin`, `include/basalt/utils/imu_types.h:329-395`). Initially $\bar{\mathbf{d}} = \hat{\mathbf{g}}$ and $\bar{\mathbf{B}} = [\mathbf{e}_x, \mathbf{e}_y]$. The block occupies offsets 0 to 2 of every order map of the period. Its first order Jacobian, derived in the first plan §2.1.3 from (I1) and (I2), is

$$\frac{\partial \mathbf{g}_D}{\partial \boldsymbol{\delta}_\theta} = -[\bar{\mathbf{g}}_D]_\times \bar{\mathbf{B}},$$

with $\bar{\mathbf{g}}_D = |\mathbf{g}|\,\bar{\mathbf{d}}$.

#### 3.1.2 Residuals

The inertial factor between states 0 and 1 over $\Delta t$ has, in Basalt's row order position, rotation, velocity (`residualGS`, `thirdparty/basalt-headers/include/basalt/imu/preintegration.h:298-386`),

$$\mathbf{r}_p = \mathbf{R}_0^\top \Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0 \Delta t) - \frac{1}{2}\mathbf{g}_D \Delta t^2\Big) - \hat{\Delta\mathbf{p}},$$

$$\mathbf{r}_R = \text{Log}\big(\text{Exp}(\mathbf{J}_{R,g}\,\Delta\mathbf{b}_g)\,\Delta\mathbf{R}\,\mathbf{R}_1^\top \mathbf{R}_0\big),$$

$$\mathbf{r}_v = \mathbf{R}_0^\top \big(s(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D \Delta t\big) - \hat{\Delta\mathbf{v}},$$

where $\hat{\Delta\mathbf{p}}$ and $\hat{\Delta\mathbf{v}}$ are the preintegrated position and velocity corrected to first order for the bias change from the preintegration point, $\hat{\Delta\mathbf{p}} = \Delta\mathbf{p} + \mathbf{J}_{p,g}\,\Delta\mathbf{b}_g + \mathbf{J}_{p,a}\,\Delta\mathbf{b}_a$ and likewise for the velocity, $\Delta\mathbf{b} = \mathbf{b}_0 - \mathbf{b}_\text{lin}$, and the matrices $\mathbf{J}_{\cdot,\cdot}$ are the blocks of $d_state_d_bg_$ and $d_state_d_ba_$. The nine rows are whitened by $\mathbf{L}$ with $\mathbf{L}^\top \mathbf{L} = \boldsymbol{\Sigma}^{-1}$, where $\boldsymbol{\Sigma}$ is the $9 \times 9$ preintegration covariance (`get_cov`, `get_sqrt_cov_inv`, `preintegration.h:395-445`). Two random walk residuals on the biases follow,

$$\mathbf{r}_{b_g} = \mathbf{W}_g (\mathbf{b}_{g,0} - \mathbf{b}_{g,1}),$$

$$\mathbf{r}_{b_a} = \mathbf{W}_a (\mathbf{b}_{a,0} - \mathbf{b}_{a,1}),$$

with diagonal weights $\mathbf{W} = \text{diag}(1 / (\sigma_b \sqrt{\Delta t}))$ (`ImuBlock::LinearizeBiasRandomWalk`, `include/basalt/linearization/imu_block.hpp:244-286`). The visual residual sees the scale only through the scaled relative translation $s\,\mathbf{R}_t^\top (\mathbf{p}_h - \mathbf{p}_t)$ between host and target and through the extrinsic lever arm, and does not contain $\mathbf{g}_D$ (first plan §2.2).

The Jacobians with respect to the block, derived in the first plan §2.3.3 by perturbing one variable at a time and applying (I9), are

$$\frac{\partial \mathbf{r}_p}{\partial \boldsymbol{\delta}_\theta} = \frac{1}{2}\Delta t^2\,\mathbf{R}_0^\top [\bar{\mathbf{g}}_D]_\times \bar{\mathbf{B}},$$

$$\frac{\partial \mathbf{r}_v}{\partial \boldsymbol{\delta}_\theta} = \Delta t\,\mathbf{R}_0^\top [\bar{\mathbf{g}}_D]_\times \bar{\mathbf{B}},$$

$$\frac{\partial \mathbf{r}_p}{\partial \sigma} = s\,\mathbf{R}_0^\top (\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0 \Delta t),$$

$$\frac{\partial \mathbf{r}_v}{\partial \sigma} = s\,\mathbf{R}_0^\top (\mathbf{v}_1 - \mathbf{v}_0),$$

and $\mathbf{r}_R$, $\mathbf{r}_{b_g}$ and $\mathbf{r}_{b_a}$ have zero block columns. In code these are `mpJgs` of `ImuBlockGS` (`imu_block.hpp:337-342`), whitened by $\mathbf{L}$.

#### 3.1.3 Gauge directions and the projector

Replacing the constants $1$ and $\mathbf{g}$ by $s$ and $\mathbf{g}_D$ adds three directions along which every residual is constant (first plan §2.5.3). In `aom` coordinates at the linearisation points, with the block at offset $g = 0$ and a frame at offset $c$, the scale direction $\mathbf{n}_\sigma$ has $-1$ at $g + 2$, $\mathbf{p}_f$ in rows $c$ to $c + 2$ and, for a full state, $\mathbf{v}_f$ in rows $c + 6$ to $c + 8$. The tilt direction $\mathbf{n}_\mathbf{a}$ for $\mathbf{a} \in \{\mathbf{e}_x, \mathbf{e}_y\}$ has $\bar{\mathbf{B}}^\top \mathbf{a}$ in rows $g$ and $g + 1$, $\mathbf{a} \times \mathbf{p}_f$ in rows $c$ to $c + 2$, $\mathbf{a}$ in rows $c + 3$ to $c + 5$ and, for a full state, $\mathbf{a} \times \mathbf{v}_f$ in rows $c + 6$ to $c + 8$. Each is the derivative at $\varepsilon = 0$ of the increment that reproduces the gauge family, scaling every position and velocity by $e^\varepsilon$ while lowering $\sigma$ by $\varepsilon$, or rotating the whole world and the gravity direction by $\text{Exp}(\varepsilon\,\mathbf{a})$ (first plan §2.5.6). With $\mathbf{N} = [\mathbf{n}_\sigma, \mathbf{n}_{\mathbf{e}_x}, \mathbf{n}_{\mathbf{e}_y}]$ every gauge invariant factor satisfies $\mathbf{J} \mathbf{N} = \mathbf{0}$, hence $\mathbf{H} \mathbf{N} = \mathbf{0}$ and $\mathbf{N}^\top \mathbf{b} = \mathbf{0}$. The projector

$$\mathbf{P} = \mathbf{I} - \mathbf{N} (\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top$$

is symmetric, idempotent, satisfies $\mathbf{P} \mathbf{N} = \mathbf{0}$ and leaves every vector orthogonal to $\mathbf{N}$ unchanged. `optimize` applies it as

$$\bar{\mathbf{H}} = \mathbf{P} \mathbf{H} \mathbf{P},$$

$$\bar{\mathbf{b}} = \mathbf{P} \mathbf{b},$$

$$\mathbf{x} \leftarrow \mathbf{P} \mathbf{x},$$

the last after the damped solve (`ComputeGaugeProjector`, `sqrt_keypoint_vio.cpp:2223-2266`, and its use at `:1870-1876` and `:1913`). The squared norms

$$|\mathbf{n}_\sigma|^2 = 1 + \sum_f |\mathbf{p}_f|^2 + \sum_f |\mathbf{v}_f|^2,$$

$$|\mathbf{n}_\mathbf{a}|^2 = |\bar{\mathbf{B}}^\top \mathbf{a}|^2 + \sum_f \big(1 + |\mathbf{a} \times \mathbf{p}_f|^2 + |\mathbf{a} \times \mathbf{v}_f|^2\big),$$

follow by summing the squares of the entries listed above, the velocity terms running over full states only. They are used in §3.2.7 and §3.4.3.

#### 3.1.4 Priors, re-centring and the swap rule as implemented

Three square root priors share one order map, $M_\text{curr}$ (`marg_data`), $M_\text{half}$ (`mpMargHalf`) and $M_\text{visual}$ (`mpMargVisual`). Each prior has the form $\frac{1}{2}|\mathbf{J}_m(\boldsymbol{\delta} + \mathbf{x}) + \mathbf{r}_m|^2$ in the offsets $\boldsymbol{\delta}$ of its variables from their linearisation points, and $M_\text{curr}$ and $M_\text{half}$ each carry their own linearisation point of the block, an anchor $(\bar{\mathbf{d}}_a, \bar{\mathbf{B}}_a, \bar{\sigma}_a)$ (`GsAnchor`, `include/basalt/vi_estimator/sqrt_keypoint_vio.h:263-271`). $M_\text{visual}$ is linearised from the visual factors with the scale column zeroed and no inertial factor, against a copy of $M_\text{curr}$'s anchor that is irrelevant because all of its block columns are zero (`LinearizeMargGS`, `sqrt_keypoint_vio.cpp:2372-2420`). At the end of every `optimize` the current correction $(\mathbf{Q}, s_c)$, with $\mathbf{Q}$ the minimal rotation carrying $\mathbf{d}$ onto $\hat{\mathbf{g}}$ and $s_c = e^{\sigma}$, is applied to every pose, state, linearisation point, prior and anchor, after which the block is the identity (`RecentreGravityScale`, `:2268-2319`, first plan §2.7). The swap rule of §2.3 runs at the start of every marginalisation on the block's log scale (`UpdateScaleInterval`, `:2333-2370`), with the interval half width the smallest multiple of $\ln d_\text{min}$ that exceeds the change of $\sigma$ since the previous marginalisation. A reset of $M_\text{half}$ copies $M_\text{visual}$ and clears $M_\text{half}$'s anchor.

#### 3.1.5 Identities

(I1) $\text{Exp}(\boldsymbol{\omega}) = \mathbf{I} + [\boldsymbol{\omega}]_\times + O(|\boldsymbol{\omega}|^2)$, the series `SO3::exp` evaluates.

(I2) $[\mathbf{a}]_\times \mathbf{b} = \mathbf{a} \times \mathbf{b} = -[\mathbf{b}]_\times \mathbf{a}$.

(I3) $[\lambda\mathbf{a}]_\times = \lambda [\mathbf{a}]_\times$ for a scalar $\lambda$.

(I4) $[\mathbf{R} \mathbf{a}]_\times = \mathbf{R} [\mathbf{a}]_\times \mathbf{R}^\top$ and $\text{Exp}(\mathbf{R} \mathbf{a}) = \mathbf{R}\,\text{Exp}(\mathbf{a})\,\mathbf{R}^\top$ for a rotation $\mathbf{R}$.

(I8) $\text{Exp}(\boldsymbol{\omega})^\top = \text{Exp}(-\boldsymbol{\omega})$.

(I9) The Jacobian of a residual with respect to an increment $\boldsymbol{\epsilon}$ applied by a variable's update rule is the matrix $\mathbf{J}$ with $\mathbf{r}(\boldsymbol{\epsilon}) = \mathbf{r} + \mathbf{J} \boldsymbol{\epsilon} + O(|\boldsymbol{\epsilon}|^2)$.

These carry the labels of the first plan, whose (I5) to (I7) and (I10) are not needed here. The identities added by this plan are the following.

(N1) Minimisation over a block. For a symmetric $\mathbf{H}$ with $\mathbf{H}_{\beta\beta}$ invertible, the minimum over $\boldsymbol{\delta}_\beta$ of $F = \frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H} \boldsymbol{\delta} + \mathbf{b}^\top \boldsymbol{\delta}$ is $\frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}' \boldsymbol{\delta}_\alpha + \mathbf{b}'^\top \boldsymbol{\delta}_\alpha + \text{const}$ with $\mathbf{H}'$ and $\mathbf{b}'$ of §2.1. Derived in §3.1.6.

(N2) Monotonicity of the marginal. If $\mathbf{H}_1 \succeq \mathbf{H}_0$, then $\mathbf{H}_1' \succeq \mathbf{H}_0'$ for their Schur complements onto the same kept block. Derived in §3.1.6.

(N3) Marginal of a Gaussian vector. If $\mathbf{z} = [\mathbf{z}_1; \mathbf{z}_2] \sim \mathcal{N}(\boldsymbol{\mu}, \boldsymbol{\Sigma})$, then $\mathbf{z}_1 \sim \mathcal{N}(\boldsymbol{\mu}_1, \boldsymbol{\Sigma}_{11})$, the corresponding block of the covariance. This is the standard property that a linear map $\mathbf{z}_1 = [\mathbf{I}\ \mathbf{0}]\,\mathbf{z}$ of a Gaussian is Gaussian with covariance $[\mathbf{I}\ \mathbf{0}]\,\boldsymbol{\Sigma}\,[\mathbf{I}\ \mathbf{0}]^\top = \boldsymbol{\Sigma}_{11}$.

(N4) Accelerometer bias Jacobians of the preintegration. The preintegrated velocity and position are sums over samples $k$ of $\Delta\mathbf{R}_k (\mathbf{a}_k - \mathbf{b}_a)\,\delta t$ and of the corresponding double integral, so their derivatives with respect to $\mathbf{b}_a$ are $-\sum_k \Delta\mathbf{R}_k\,\delta t$ and its double integral, computed recursively as $d_state_d_ba_ = -A + F * d_state_d_ba_$ (`preintegration.h:182`). When the attitude is constant over the interval, $\Delta\mathbf{R}_k = \mathbf{I}$, so $\partial \Delta\mathbf{v} / \partial \mathbf{b}_a = -\Delta t\,\mathbf{I}$ and $\partial \Delta\mathbf{p} / \partial \mathbf{b}_a = -\frac{1}{2}\Delta t^2\,\mathbf{I}$, and the residual, which subtracts them ($d_res_d_ba = -d_state_d_ba_$, `:360`), has $\partial \mathbf{r}_v / \partial \mathbf{b}_a = \Delta t\,\mathbf{I}$ and $\partial \mathbf{r}_p / \partial \mathbf{b}_a = \frac{1}{2}\Delta t^2\,\mathbf{I}$.

(N5) Preintegration noise over a short interval. For white accelerometer noise of density $\sigma_c$ integrated over $\Delta t$, the velocity increment has variance $\sigma_c^2 \Delta t$ per axis, the position increment $\sigma_c^2 \Delta t^3 / 3$, and their covariance $\sigma_c^2 \Delta t^2 / 2$, the moments of the integrals $\int_0^{\Delta t} \mathbf{w}\,d\tau$ and $\int_0^{\Delta t} (\Delta t - \tau)\,\mathbf{w}\,d\tau$ of white noise $\mathbf{w}$. Basalt propagates the same moments in discrete form (`preintegration.h:177`).

#### 3.1.6 Derivation of (N1) and (N2)

(N1). Step 1. Partitioning $F$ by $\alpha$ and $\beta$,

$$F = \frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}_{\alpha\alpha} \boldsymbol{\delta}_\alpha + \boldsymbol{\delta}_\alpha^\top \mathbf{H}_{\alpha\beta} \boldsymbol{\delta}_\beta + \frac{1}{2}\boldsymbol{\delta}_\beta^\top \mathbf{H}_{\beta\beta} \boldsymbol{\delta}_\beta + \mathbf{b}_\alpha^\top \boldsymbol{\delta}_\alpha + \mathbf{b}_\beta^\top \boldsymbol{\delta}_\beta.$$

Step 2. Setting the derivative with respect to $\boldsymbol{\delta}_\beta$ to zero,

$$\mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha + \mathbf{H}_{\beta\beta} \boldsymbol{\delta}_\beta + \mathbf{b}_\beta = \mathbf{0},$$

$$\boldsymbol{\delta}_\beta^* = -\mathbf{H}_{\beta\beta}^{-1}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha).$$

Step 3. At $\boldsymbol{\delta}_\beta^*$ the three terms containing $\boldsymbol{\delta}_\beta$ sum to $\frac{1}{2}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)^\top \boldsymbol{\delta}_\beta^*$, because Step 2 gives $\mathbf{H}_{\beta\beta} \boldsymbol{\delta}_\beta^* = -(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)$, and substituting $\boldsymbol{\delta}_\beta^*$,

$$\frac{1}{2}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)^\top \boldsymbol{\delta}_\beta^* = -\frac{1}{2}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)^\top \mathbf{H}_{\beta\beta}^{-1} (\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha).$$

Step 4. Expanding, merging the two cross terms by the symmetry of $\mathbf{H}_{\beta\beta}^{-1}$, and adding the terms in $\boldsymbol{\delta}_\alpha$ alone gives the quadratic with $\mathbf{H}'$ and $\mathbf{b}'$ of §2.1.

A consequence used by DM-VIO and in §2.4 is that a minimum over two blocks equals the minimum over one block of the minimum over the other, so successive Schur complements equal the joint one.

(N2). Step 1. By (N1) with $\mathbf{b} = \mathbf{0}$, for every $\boldsymbol{\delta}_\alpha$,

$$\frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}_i' \boldsymbol{\delta}_\alpha = \min_{\boldsymbol{\delta}_\beta} \frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H}_i \boldsymbol{\delta}.$$

Step 2. $\mathbf{H}_1 \succeq \mathbf{H}_0$ means $\boldsymbol{\delta}^\top \mathbf{H}_1 \boldsymbol{\delta} \ge \boldsymbol{\delta}^\top \mathbf{H}_0 \boldsymbol{\delta}$ for every $\boldsymbol{\delta}$. Step 3. The minimum of a pointwise larger function is at least the minimum of the smaller one, so $\boldsymbol{\delta}_\alpha^\top \mathbf{H}_1' \boldsymbol{\delta}_\alpha \ge \boldsymbol{\delta}_\alpha^\top \mathbf{H}_0' \boldsymbol{\delta}_\alpha$ for every $\boldsymbol{\delta}_\alpha$, which is $\mathbf{H}_1' \succeq \mathbf{H}_0'$. Adding factors to a prior's factor set adds $\mathbf{J}^\top \mathbf{J} \succeq \mathbf{0}$ to its $\mathbf{H}$, so by (N2) the marginal holds at least as much information in every direction. When $\mathbf{H}_{\beta\beta}$ is singular, the same argument holds with the minimum taken over the subspace where it is finite, which is the Moore–Penrose form of [10].

### 3.2 Why the period diverges

#### 3.2.1 What each prior holds

$M_\text{curr}$ and $M_\text{half}$ hold every factor marginalised into them, inertial factors included. $M_\text{visual}$ holds the visual factors with the scale column zeroed and the initial prior. The initial prior pins the position and the yaw of the first state with weight `vio_init_pose_weight` and puts diagonal weights on its biases (`sqrt_keypoint_vio.cpp:105-117`), namely $\sqrt{w}$ with `vio_init_ba_weight` on indices 9 to 11, which are the gyroscope bias, and `vio_init_bg_weight` on indices 12 to 14, which are the accelerometer bias (finding 5 of §1.2). It holds nothing on roll and pitch. Visual factors are invariant under a rotation of the whole world, so they hold nothing on roll and pitch either.

#### 3.2.2 The scale error of a frozen inertial factor

Take the velocity row with $\mathbf{u} = \mathbf{v}_1 - \mathbf{v}_0$, $\mathbf{r}_v = \mathbf{R}_0^\top (s\,\mathbf{u} - \mathbf{g}_D \Delta t) - \hat{\Delta\mathbf{v}}$, linearised at $(\bar{s}, \bar{\mathbf{u}})$ and evaluated at $(\bar{s} + \Delta s, \bar{\mathbf{u}} + \Delta\mathbf{u})$ with the rotation held fixed. Step 1. The product expands exactly,

$$(\bar{s} + \Delta s)(\bar{\mathbf{u}} + \Delta\mathbf{u}) = \bar{s}\,\bar{\mathbf{u}} + \bar{s}\,\Delta\mathbf{u} + \Delta s\,\bar{\mathbf{u}} + \Delta s\,\Delta\mathbf{u}.$$

Step 2. The linear model of the frozen factor holds the first three terms, its Jacobians being $\bar{s}\,\mathbf{R}_0^\top$ for $\mathbf{u}$ and $\mathbf{R}_0^\top \bar{\mathbf{u}}$ for $s$. Step 3. Its error is the last term,

$$\mathbf{r}_v - \mathbf{r}_v^\text{lin} = \Delta s\,\mathbf{R}_0^\top \Delta\mathbf{u}.$$

With $\Delta s = 0.5$ and $|\Delta\mathbf{u}| = 1$ m/s the error is 0.5 m/s against a standard deviation of the preintegrated velocity of about 0.018 m/s over 50 ms (§3.3.4), about thirty standard deviations. A prior that holds the scale dependent inertial rows is therefore valid only near the scale it was built at, which is the reason for the three priors of §2.3.

#### 3.2.3 Tilt and horizontal accelerometer bias form a null direction at constant attitude

Consider an inertial factor over an interval in which the attitude is constant, and the joint perturbation that rotates the attitude of both of its states by the same small horizontal rotation vector $\boldsymbol{\phi}$, $\mathbf{R}_f \leftarrow \text{Exp}(\boldsymbol{\phi})\,\mathbf{R}_f$ for $f \in \{0, 1\}$, while changing the accelerometer bias of the start state by $\delta\mathbf{b}_a$. Positions, velocities, the block and the gyroscope bias are held.

Step 1, rotation row. $\mathbf{r}_R$ contains $\mathbf{R}_1^\top \mathbf{R}_0$, which becomes $\mathbf{R}_1^\top \text{Exp}(\boldsymbol{\phi})^\top \text{Exp}(\boldsymbol{\phi})\,\mathbf{R}_0 = \mathbf{R}_1^\top \mathbf{R}_0$ by (I8), so $\mathbf{r}_R$ is unchanged.

Step 2, velocity row, the rotation. By (I8) and (I1), $(\text{Exp}(\boldsymbol{\phi})\,\mathbf{R}_0)^\top = \mathbf{R}_0^\top \text{Exp}(-\boldsymbol{\phi}) = \mathbf{R}_0^\top (\mathbf{I} - [\boldsymbol{\phi}]_\times) + O(|\boldsymbol{\phi}|^2)$. Applied to $s\,\mathbf{u} - \mathbf{g}_D \Delta t$, the change is

$$-\mathbf{R}_0^\top [\boldsymbol{\phi}]_\times (s\,\mathbf{u} - \mathbf{g}_D \Delta t).$$

In the rest and hover case of interest $\mathbf{u} = \mathbf{0}$, so the change is $\Delta t\,\mathbf{R}_0^\top [\boldsymbol{\phi}]_\times \mathbf{g}_D$, and by (I2)

$$\Delta t\,\mathbf{R}_0^\top [\boldsymbol{\phi}]_\times \mathbf{g}_D = -\Delta t\,\mathbf{R}_0^\top [\mathbf{g}_D]_\times \boldsymbol{\phi}.$$

Step 3, velocity row, the bias. By (N4) the change is $\Delta t\,\delta\mathbf{b}_a$.

Step 4. The two changes cancel when

$$\delta\mathbf{b}_a = \mathbf{R}_0^\top [\mathbf{g}_D]_\times \boldsymbol{\phi} = \mathbf{R}_0^\top (\mathbf{g}_D \times \boldsymbol{\phi}).$$

Step 5, position row. The same two steps with $\frac{1}{2}\Delta t^2$ in place of $\Delta t$, and $\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0 \Delta t = \mathbf{0}$ at rest, give the change $\frac{1}{2}\Delta t^2 (\delta\mathbf{b}_a - \mathbf{R}_0^\top [\mathbf{g}_D]_\times \boldsymbol{\phi})$, which vanishes for the same $\delta\mathbf{b}_a$.

Step 6, bias random walk. Shifting the accelerometer bias of every state in the window and in the prior by the same body frame vector leaves every difference $\mathbf{b}_{a,0} - \mathbf{b}_{a,1}$ unchanged, since the attitude is constant and the vector is the same in every body frame.

Step 7, the remaining factors. Vision is invariant under a rotation of the attitudes when the positions do not change between frames, since $\mathbf{R}_t^\top (\mathbf{p}_h - \mathbf{p}_t) = \mathbf{0}$ then and $\mathbf{R}_t^\top \mathbf{R}_h$ is unchanged. The gyroscope bias is untouched.

So at rest and in hover the direction that tilts every attitude by $\boldsymbol{\phi}$ and shifts every accelerometer bias by $\mathbf{R}_0^\top (\mathbf{g}_D \times \boldsymbol{\phi})$ is a null direction of every factor except a prior that holds an absolute value of the bias. Since $\boldsymbol{\phi}$ is horizontal and $\mathbf{g}_D$ vertical, $|\mathbf{g}_D \times \boldsymbol{\phi}| = |\mathbf{g}|\,|\boldsymbol{\phi}|$, and for a finite tilt the same construction with $\text{Exp}(\boldsymbol{\phi})$ gives a horizontal bias of magnitude $|\mathbf{g}| \sin|\boldsymbol{\phi}|$, which is the relation the replays measured at every sample. With the default accelerometer bias prior of standard deviation $\sigma_{b_a} = 0.1$ m/s², the tilt is anchored to $\sigma_{b_a} / |\mathbf{g}| = 0.0102$ rad, 0.58°. This is the coupling Campos et al. report, "difficult to distinguish in most cases" [12].

#### 3.2.4 The bias anchor survives marginalisation only through the random walk

Take one bias coordinate. Let the initial prior give information $w_0$ on $b_0$ and the random walk factor information $w_1$ on $b_0 - b_1$, and marginalise $b_0$.

Step 1. With the random walk factor present, the cost and its information over $(b_0, b_1)$ are

$$F = \frac{1}{2} w_0 b_0^2 + \frac{1}{2} w_1 (b_0 - b_1)^2,$$

$$\mathbf{H} = \begin{bmatrix} w_0 + w_1 & -w_1 \\ -w_1 & w_1 \end{bmatrix}.$$

Step 2. By (N1) with $\beta = b_0$ and $\alpha = b_1$,

$$H' = w_1 - \frac{w_1^2}{w_0 + w_1} = \frac{w_0 w_1}{w_0 + w_1}.$$

Step 3. Its inverse is

$$\frac{1}{H'} = \frac{1}{w_0} + \frac{1}{w_1},$$

the variance of the initial prior plus that of one random walk step. The anchor survives.

Step 4. In $M_\text{visual}$ as implemented the random walk factor is absent, so

$$\mathbf{H} = \begin{bmatrix} w_0 & 0 \\ 0 & 0 \end{bmatrix},$$

and (N1) gives $H' = 0 - 0 \cdot w_0^{-1} \cdot 0 = 0$. The initial prior leaves with the first state and nothing remains on $b_1$. Every prior that has been reset to $M_\text{visual}$ therefore holds no absolute information on the accelerometer bias, and by §3.2.3 no information on the tilt at constant attitude.

#### 3.2.5 The swap ratchet

A swap $M_\text{curr} \leftarrow M_\text{half}$ discards every inertial factor that $M_\text{curr}$ held and $M_\text{half}$ did not, and $M_\text{half} \leftarrow M_\text{visual}$ discards every inertial factor of $M_\text{half}$. Step 1. Let the per frame step at take off exceed $\ln d_\text{min} \approx 0.048$ for several frames, as logged ($-0.078$, $+0.062$, $-0.064$, $-0.084$, and one of $-0.56$). Step 2. Each such step changes side or leaves the interval, so $M_\text{half}$ is reset to $M_\text{visual}$ at every frame and holds at most the one inertial factor marginalised since. Step 3. Each crossing sets $M_\text{curr}$ to that nearly empty $M_\text{half}$. Step 4. By §3.2.3 and §3.2.4 the prior in use then holds neither tilt nor bias anchor, the window's two inertial factors cannot separate them, and the next step is again poorly determined and large, which closes the loop. The bias drift that began 0.3 s after the last of 24 swaps is this mechanism.

#### 3.2.6 Why the steps are large at take off

At rest $|\mathbf{n}_\sigma| \to 1$ by §3.1.3, so $\mathbf{n}_\sigma \to -\mathbf{e}_\sigma$, the scale coordinate is itself a gauge direction, and the projection removes every scale step. As motion starts, the scale becomes observable only through factors that accelerate (§3.3.4), and the first such factors are few. The first solves after lift off therefore determine the scale from very little information, and the per frame re-centring applies whatever they return to every state and prior at once.

#### 3.2.7 The block alone carries no information under the gauge

A gate on the information about the block cannot be formed from the block's marginal. Write $\bar{\mathbf{H}} = \mathbf{P} \mathbf{H} \mathbf{P}$ in block ($g$) and frame ($f$) parts, and split each gauge vector as $\mathbf{n} = [\mathbf{n}_g; \mathbf{n}_f]$. Step 1. $\bar{\mathbf{H}} \mathbf{n} = \mathbf{0}$ by $\mathbf{P} \mathbf{n} = \mathbf{0}$. Its two block rows are

$$\bar{\mathbf{H}}_{gg} \mathbf{n}_g + \bar{\mathbf{H}}_{gf} \mathbf{n}_f = \mathbf{0},$$

$$\bar{\mathbf{H}}_{fg} \mathbf{n}_g + \bar{\mathbf{H}}_{ff} \mathbf{n}_f = \mathbf{0}.$$

Step 2. The marginal information on the block is the Schur complement $\mathbf{S} = \bar{\mathbf{H}}_{gg} - \bar{\mathbf{H}}_{gf} \bar{\mathbf{H}}_{ff}^{-1} \bar{\mathbf{H}}_{fg}$ by (N1). Applying it to $\mathbf{n}_g$ and substituting the second row of Step 1, $\bar{\mathbf{H}}_{fg} \mathbf{n}_g = -\bar{\mathbf{H}}_{ff} \mathbf{n}_f$,

$$\mathbf{S} \mathbf{n}_g = \bar{\mathbf{H}}_{gg} \mathbf{n}_g + \bar{\mathbf{H}}_{gf} \mathbf{n}_f = \mathbf{0},$$

by the first row. Step 3. The block parts of the three gauge vectors are $(0, 0, -1)$, $(\bar{\mathbf{B}}^\top \mathbf{e}_x, 0)$ and $(\bar{\mathbf{B}}^\top \mathbf{e}_y, 0)$, which at $\bar{\mathbf{B}} = [\mathbf{e}_x, \mathbf{e}_y]$ form a signed permutation of $\mathbb{R}^3$, so they span $\mathbb{R}^3$ and $\mathbf{S} = \mathbf{0}$. Any value of the block can be traded against a uniform transformation of the frames, so its marginal variance is infinite whatever the data, while the physical scale and tilt may be well determined. This is the exact form of the observation of [5] that free gauge covariances depend on the gauge. DM-VIO can threshold $\text{cov}(s)$ [2] because its visual frame has a scale of its own (§2.4), which this plan's metric frames do not.

### 3.3 Change A. A scale free prior that keeps the inertial rows

#### 3.3.1 What $M_\text{visual}$ must satisfy

$M_\text{visual}$ replaces $M_\text{half}$ at a reset and, through it, $M_\text{curr}$ at a crossing, at whatever scale the estimate then has. The requirement of VI-DSO is therefore that its linear model stay valid at every scale. The swap rule bounds the scale and nothing else, so no corresponding requirement exists for the tilt. A tilt dependent row is admissible provided its first estimate error stays small for the tilt changes that occur, and the following derivation shows that it is of second order. Step 1. With the direction at $\boldsymbol{\delta}_\theta$ from the anchor, (I1) to second order gives

$$\mathbf{g}_D(\boldsymbol{\delta}_\theta) = \bar{\mathbf{g}}_D + [\bar{\mathbf{B}} \boldsymbol{\delta}_\theta]_\times \bar{\mathbf{g}}_D + \frac{1}{2}[\bar{\mathbf{B}} \boldsymbol{\delta}_\theta]_\times^2 \bar{\mathbf{g}}_D + O(|\boldsymbol{\delta}_\theta|^3).$$

Step 2. The prior's linear model holds the first two terms, so its error in $\mathbf{g}_D$ is the third, of size at most $\frac{1}{2}|\mathbf{g}|\,|\boldsymbol{\delta}_\theta|^2$. Step 3. $\mathbf{g}_D$ enters $\mathbf{r}_v$ as $-\mathbf{R}_0^\top \mathbf{g}_D \Delta t$, linearly, so the error in $\mathbf{r}_v$ is at most $\frac{1}{2}|\mathbf{g}|\,|\boldsymbol{\delta}_\theta|^2 \Delta t$. Step 4. For a tilt change of 2° from the anchor, $|\boldsymbol{\delta}_\theta| = 0.035$ rad, this is $0.006 \cdot \Delta t$ m/s, 0.0003 m/s over 50 ms, 1.7 % of the velocity standard deviation of §3.3.4. Step 5. The product $\mathbf{R}_0^\top \mathbf{g}_D$ couples the frame's attitude offset and the tilt offset bilinearly, with error $O(|\delta\boldsymbol{\varphi}_0|\,|\boldsymbol{\delta}_\theta|)$, a product of two small angles, as in every rotation row of Basalt. The scale, by contrast, enters as the product $s\,\mathbf{u}$ of §3.2.2, whose error is first order in the scale change and is the reason for the swaps. $M_\text{visual}$ may therefore hold tilt dependent rows linearised at its own anchor, and must hold only rows whose dependence on the scale is negligible.

#### 3.3.2 The rows that are exactly scale free

Step 1. $\mathbf{r}_R$ contains $\mathbf{R}_0$, $\mathbf{R}_1$ and the gyroscope bias only. Its Jacobians, `d_res_d_state0` and `d_res_d_state1` rows 3 to 5 equal to $\mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top$ and $-\mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top$ in the rotation columns and zero elsewhere, and `d_res_d_bg` rows 3 to 5 equal to $\mathbf{J}_l^{-1}(\mathbf{r}_R)$ times the rotation block of $d_state_d_bg_$ (`preintegration.h:342-370`), contain neither $s$ nor $\mathbf{g}_D$. `d_res_d_ba` rows 3 to 5 are zero, which `residualGS` asserts (`:313`). Its block columns are zero (§3.1.2). Step 2. $\mathbf{r}_{b_g}$ and $\mathbf{r}_{b_a}$ are linear in the biases with constant Jacobians $\pm\mathbf{W}$, and contain neither $s$ nor $\mathbf{g}_D$. Step 3. The initial prior acts on the first state's position, yaw and biases with constant weights. These three groups are exactly scale free and tilt free, and their Jacobians do not depend on any block value, so no anchor enters them.

#### 3.3.3 The rotation rows must be weighted by their marginal covariance

The whitened residual $\mathbf{L} \mathbf{r}$ cannot be split into a rotation part and the rest by taking rows 3 to 5. `compute_sqrt_cov_inv` (`preintegration.h:430-446`) forms $\mathbf{L}$ from an $\mathbf{L}\mathbf{D}\mathbf{L}^\top$ factorisation of $\boldsymbol{\Sigma}$ with a pivoting permutation, so $\mathbf{L}$ is lower triangular in permuted order, and row $i$ of $\mathbf{L} \mathbf{r}$ is a combination of several of $\mathbf{r}_p$, $\mathbf{r}_R$ and $\mathbf{r}_v$. Keeping three whitened rows would keep a combination that contains the scale dependent rows, and the cross covariances of the preintegration, for instance between $\Delta\mathbf{v}$ and $\Delta\mathbf{R}$ through the attitude error, make those combinations nontrivial.

The correct factor is the likelihood of the rotation measurement alone. Step 1. The inertial factor is the Gaussian likelihood $\mathbf{r} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma})$ of the nine dimensional measurement $[\hat{\Delta\mathbf{p}}; \Delta\mathbf{R}; \hat{\Delta\mathbf{v}}]$, with cost $\frac{1}{2}\mathbf{r}^\top \boldsymbol{\Sigma}^{-1} \mathbf{r}$. Step 2. Discarding the position and velocity measurements means marginalising them out of the measurement distribution. By (N3) with $\mathbf{z}_1 = \mathbf{r}_R$, the rotation residual alone is distributed as $\mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma}_{RR})$, with $\boldsymbol{\Sigma}_{RR}$ the $3 \times 3$ block of $\boldsymbol{\Sigma}$ at rows and columns 3 to 5. Step 3. Its negative log likelihood is, up to a constant,

$$F_R = \frac{1}{2}\mathbf{r}_R^\top \boldsymbol{\Sigma}_{RR}^{-1} \mathbf{r}_R.$$

Step 4. With the Cholesky factorisation $\boldsymbol{\Sigma}_{RR} = \mathbf{C} \mathbf{C}^\top$, $\mathbf{C}$ lower triangular, and $\mathbf{L}_R = \mathbf{C}^{-1}$,

$$\mathbf{L}_R^\top \mathbf{L}_R = \mathbf{C}^{-\top} \mathbf{C}^{-1} = (\mathbf{C} \mathbf{C}^\top)^{-1} = \boldsymbol{\Sigma}_{RR}^{-1},$$

so $F_R = \frac{1}{2}|\mathbf{L}_R \mathbf{r}_R|^2$, and the square root rows of the factor are $\mathbf{L}_R \mathbf{r}_R$ with Jacobians $\mathbf{L}_R$ times rows 3 to 5 of the unwhitened Jacobians. $\boldsymbol{\Sigma}_{RR}$ is positive definite because the gyroscope noise is, so the factorisation exists.

The same result follows from non-linear factor recovery (§2.12). A recovered factor measuring $\mathbf{r}_R$ has the KL optimal information $(\{\mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top\}_i)^{-1}$, which for a selection $\mathbf{J}_r = [\mathbf{0}\ \mathbf{I}\ \mathbf{0}]$ of the measurement with covariance $\boldsymbol{\Sigma}_o = \boldsymbol{\Sigma}$ is $\boldsymbol{\Sigma}_{RR}^{-1}$. The rotation only factor is weaker than the rotation rows of the full factor would appear to be, because $\boldsymbol{\Sigma}_{RR}^{-1} \preceq (\boldsymbol{\Sigma}^{-1})_{RR}$, the information of a block of a Gaussian being at most the corresponding block of the joint information. Using the joint block would claim information that only the discarded position and velocity rows justify, which would be spurious in the sense of §2.2.

#### 3.3.4 Full rows of a factor without acceleration, and the admission test

The scale columns of §3.1.2 contain the displacement and the velocity change. Over an interval in which the body moves with constant acceleration $\mathbf{a}$ in the estimator's world, excluding gravity, kinematics give

$$\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0 \Delta t = \frac{1}{2}\mathbf{a} \Delta t^2,$$

$$\mathbf{v}_1 - \mathbf{v}_0 = \mathbf{a} \Delta t,$$

so the scale column is $s\,\mathbf{R}_0^\top \mathbf{a}\,[\frac{1}{2}\Delta t^2; \Delta t]$ in the position and velocity rows, and it vanishes when $\mathbf{a} = \mathbf{0}$, at rest, in hover and in unaccelerated flight. This is the single factor form of the statement that the scale is not observable without a change of acceleration [14]. A factor with a negligible scale column is valid at every scale with that column set to zero, and it carries the tilt and the bias information of §3.2.3, so it should enter $M_\text{visual}$ with all its rows.

The admission test bounds the error that zeroing makes. Step 1. Let $\mathbf{c} = \mathbf{L}\,\partial\mathbf{r} / \partial\sigma$ be the whitened scale column evaluated at the anchor. Zeroing it removes the term $\mathbf{c}\,\Delta\sigma$ from the linear model, so after a later scale change $\Delta\sigma$ the factor's whitened residual is wrong by

$$|\mathbf{c}\,\Delta\sigma| = |\mathbf{c}|\,|\Delta\sigma|.$$

Step 2. Requiring that error to stay below one standard deviation for the largest scale change expected in the period, $\Delta\sigma_\text{max}$, gives the test

$$|\mathbf{c}| \le \tau,$$

$$\tau = \frac{1}{\Delta\sigma_\text{max}}.$$

The replays needed corrections of up to a factor of 2.7, $\Delta\sigma_\text{max} = \ln 2.7 \approx 1$, so the default is $\tau = 1$, configured as `vio_gs_visual_prior_scale_tol`. $|\mathbf{c}|$ is the Mahalanobis norm $(\mathbf{c}_u^\top \boldsymbol{\Sigma}^{-1} \mathbf{c}_u)^{1/2}$ of the unwhitened column $\mathbf{c}_u$, so it does not depend on the choice of $\mathbf{L}$.

Step 3, the test in physical units. By (N5) the position and velocity increments over $\Delta t$ have the covariance

$$\boldsymbol{\Sigma}_{pv} = \sigma_c^2 \begin{bmatrix} \frac{1}{3}\Delta t^3 & \frac{1}{2}\Delta t^2 \\ \frac{1}{2}\Delta t^2 & \Delta t \end{bmatrix}$$

per axis, ignoring the small coupling to the rotation. Its determinant is $\sigma_c^4 (\frac{1}{3} - \frac{1}{4})\Delta t^4 = \frac{1}{12}\sigma_c^4 \Delta t^4$, so

$$\boldsymbol{\Sigma}_{pv}^{-1} = \frac{12}{\sigma_c^2 \Delta t^4} \begin{bmatrix} \Delta t & -\frac{1}{2}\Delta t^2 \\ -\frac{1}{2}\Delta t^2 & \frac{1}{3}\Delta t^3 \end{bmatrix}.$$

With $s = 1$ and the per axis column $\mathbf{x} = a\,[\frac{1}{2}\Delta t^2; \Delta t]$,

$$\mathbf{x}^\top \boldsymbol{\Sigma}_{pv}^{-1} \mathbf{x} = \frac{12 a^2}{\sigma_c^2 \Delta t^4}\Big(\frac{1}{4}\Delta t^5 - \frac{1}{2}\Delta t^5 + \frac{1}{3}\Delta t^5\Big) = \frac{12 a^2}{\sigma_c^2 \Delta t^4} \cdot \frac{1}{12}\Delta t^5 = \frac{a^2 \Delta t}{\sigma_c^2},$$

so

$$|\mathbf{c}| = \frac{|\mathbf{a}|\sqrt{\Delta t}}{\sigma_c},$$

and the test admits a factor when $|\mathbf{a}| \le \tau\,\sigma_c / \sqrt{\Delta t}$. For the SITL calibration, $\sigma_c = 0.08$ m/s²/√Hz (`data/sitl_calib.json`, `accel_noise_std`), and $\Delta t = 0.05$ s, the bound is $0.358$ m/s² at $\tau = 1$. For kinematically consistent states this admits the stationary phase, hover and steady flight and rejects the take off transient. Estimated states are not kinematically consistent at rest, and on the SITL replays millimetre position jitter gives the ground phase a median $|\mathbf{c}|$ of about 6, so at $\tau = 1$ nearly every ground factor enters with its rotation rows only (§5.5 item 4). Note, 2026-11-06. This sentence previously stated that the stationary phase is admitted at $\tau = 1$, which the smoke replays of §5.5 contradicted. The same calibration gives the velocity standard deviation used in §3.2.2, $\sigma_c \sqrt{\Delta t} = 0.0179$ m/s. Estimation noise in the velocities adds an apparent acceleration of the order of the velocity noise divided by $\Delta t$, so some factors at rest may fail the test. The counts of admitted and rotation only factors per marginalisation, logged by Change 7, show how often this happens, and $\tau$ can be raised if the ground phase is mostly rejected.

Step 4, gauge consistency. A factor whose scale column is zeroed no longer annihilates $\mathbf{n}_\sigma$. With $\mathbf{J}' = \mathbf{J} - \mathbf{c}\,\mathbf{e}_\sigma^\top$ and $\mathbf{J} \mathbf{n}_\sigma = \mathbf{0}$,

$$\mathbf{J}' \mathbf{n}_\sigma = -\mathbf{c}\,(\mathbf{e}_\sigma^\top \mathbf{n}_\sigma) = \mathbf{c},$$

since the $\sigma$ entry of $\mathbf{n}_\sigma$ is $-1$. After the projection of the system the factor enters as $\mathbf{J}' \mathbf{P}$, and since $\mathbf{J} \mathbf{P} = \mathbf{J}$,

$$\mathbf{J}' \mathbf{P} = \mathbf{J} - \mathbf{c}\,(\mathbf{P} \mathbf{e}_\sigma)^\top.$$

The gauge component is removed exactly, and what remains differs from the exact factor by the rank one term $\mathbf{c}\,(\mathbf{P} \mathbf{e}_\sigma)^\top$, which is the bounded physical approximation of Step 1. The visual factors of $M_\text{visual}$ already carry the same approximation for their lever arm column (first plan §2.6.5). Rotation rows and bias rows annihilate all three gauge vectors exactly, since the tilt family leaves $\mathbf{R}_1^\top \mathbf{R}_0$ unchanged and neither family changes a bias.

#### 3.3.5 The anchor of $M_\text{visual}$

Full rows admitted by §3.3.4 have nonzero tilt columns, evaluated at the block's linearisation point at the moment of their marginalisation. By §2.2 every later linearisation into the same prior must use the same point. $M_\text{visual}$ therefore receives its own anchor $(\bar{\mathbf{d}}_v, \bar{\mathbf{B}}_v, \bar{\sigma}_v)$, recorded at its first marginalisation in the period, re-expressed by every re-centring like the other two anchors (first plan §2.7.1 item 3), and used for its linearisation and for the re-centring of its $\mathbf{b}$. The scale coordinate of that anchor is not free, although every scale column of $M_\text{visual}$ is zero. It is the scale at which the pose Jacobians of its visual factors are evaluated, since `computeRelPoseGS` multiplies the translation part of `d_rel_d_h` and `d_rel_d_t` by it (`include/basalt/utils/ba_utils.h:97-103`), and the scale in the state Jacobians $\pm s\,\mathbf{R}_0^\top$ of admitted full inertial rows. Because $M_\text{visual}$ treats the scale as a constant, those Jacobians must be evaluated at the current scale, so the anchor's scale is set to the current value before every linearisation of $M_\text{visual}$, and only its tilt part is held as a first estimate. Note, 2026-11-06. This paragraph previously stated that the anchor's scale multiplies only zero columns and is therefore free, and Change 5 as first implemented held it fixed and shifted it at every re-centring. Its scale then drifted with the cumulative correction, to a linearisation scale of 0.0044 in the live run `/ws/logs/slam/2026-11-06_23-14-47`, which scaled the visual translation Jacobians of $M_\text{visual}$ down by a factor of about 220 (§5.5 item 6).

The freedom matters at a reset. Today $M_\text{half} \leftarrow M_\text{visual}$ clears $M_\text{half}$'s anchor, and the next marginalisation records a fresh one at the current value, which was correct while $M_\text{visual}$ had no block columns. With tilt columns, the copy must carry $M_\text{visual}$'s tilt anchor, or $M_\text{half}$'s first estimates would change point. Step 1. Set $M_\text{half}$'s anchor to $(\bar{\mathbf{d}}_v, \bar{\mathbf{B}}_v, \sigma_\text{now})$, with $\sigma_\text{now}$ the block's current log scale. Step 2. The tilt part matches the point at which every tilt column of the copied prior was evaluated, so the copy is first estimate consistent. Step 3. The scale part may be chosen freely, because the copied scale columns are zero, and choosing $\sigma_\text{now}$ gives the scale factors that $M_\text{half}$ absorbs next the same first estimate as a fresh anchor would, which is today's behaviour for the scale. Step 4. The re-centring of $\mathbf{b}$ (`RecentrePriorsGS`) uses the offset of the block from the anchor, whose scale part again multiplies zero columns, so it is unaffected. A crossing sets $M_\text{curr}$'s anchor to $M_\text{half}$'s, as today.

#### 3.3.6 What survives a swap with Change A

Step 1, the bias anchor. $M_\text{visual}$ now holds the accelerometer bias random walk. By §3.2.4 Steps 1 to 3, the initial prior then propagates to every later state with variance

$$\frac{1}{w_0} + \sum_k \frac{1}{w_k} = \sigma_{b_a,0}^2 + \sigma_{b_a,\text{rw}}^2\,T,$$

after a time $T$ from the start, where $\sigma_{b_a,\text{rw}}$ is the random walk density, `accel_bias_std`. For the SITL calibration $\sigma_{b_a,\text{rw}} = 0.001132$ m/s²/√s, so after 7000 s the random walk adds $0.001132 \sqrt{7000} = 0.095$ m/s² in quadrature to the prior's 0.1 m/s², and the anchor stays below 0.14 m/s² for the whole flight. By §3.2.3 the tilt stays anchored to about $0.14 / 9.81$ rad, 0.8°, through every swap.

Step 2, the tilt of the ground phase and of unaccelerated flight. Their factors are admitted with all rows, so the precise tilt and bias information of the stationary phase, where the accelerometer measures gravity alone, survives every swap.

Step 3, the relative rotations and the gyroscope bias. The rotation rows keep the gyroscope chain, so the attitudes of the departing states stay tied to the current ones.

Step 4, the information never decreases. Every factor $M_\text{visual}$ held before Change A it still holds, and Change A only adds factors. By (N2) the new $M_\text{visual}$ dominates the old one in the Loewner order, and since each prior after a reset is $M_\text{visual}$ plus the factors marginalised since, so does every $M_\text{half}$ and every $M_\text{curr}$ after a crossing. Only the scale dependent rows of accelerating factors are still lost at a swap, which is VI-DSO's own price for consistency.

### 3.4 Change B. A random walk residual on the per frame correction

#### 3.4.1 The process model and its residual

Model the block as a random walk between consecutive frames, the process model of the filters of [7],

$$\boldsymbol{\eta}_{k+1} = \boldsymbol{\eta}_k + \mathbf{w}_k,$$

$$\mathbf{w}_k \sim \mathcal{N}\big(\mathbf{0},\ \text{diag}(q_\theta, q_\theta, q_\sigma)\,\Delta t_k\big),$$

where $\boldsymbol{\eta}$ stands for the block's tangent coordinates, $q_\theta$ and $q_\sigma$ are the intensities of the tilt and of the log scale, and $\Delta t_k$ is the time between the frames. In a smoother the model is the residual

$$\mathbf{r}_\text{rw} = \mathbf{W}_\text{rw} (\boldsymbol{\eta}_{k+1} - \boldsymbol{\eta}_k),$$

$$\mathbf{W}_\text{rw} = \text{diag}\Big(\frac{1}{\sqrt{q_\theta \Delta t_k}}, \frac{1}{\sqrt{q_\theta \Delta t_k}}, \frac{1}{\sqrt{q_\sigma \Delta t_k}}\Big).$$

The re-centring of the first plan §2.7 sets the block to the identity at the end of every frame, so at the start of frame $k + 1$ the block is at the identity and $\boldsymbol{\eta}_{k+1} - \boldsymbol{\eta}_k$ is the correction accumulated within the frame. Write $\mathbf{a} \in \mathbb{R}^3$ for that accumulated block increment, the sum of the block parts of the increments applied by the accepted iterations of the frame. The residual of the frame is

$$\mathbf{r}_\text{rw} = \mathbf{W}_\text{rw}\,\mathbf{a}.$$

The intensities are configured as `vio_gs_jump_tilt_rw_deg`, $\sqrt{q_\theta}$ in degrees per square root second, and `vio_gs_jump_sigma_rw`, $\sqrt{q_\sigma}$ per square root second. A value of zero disables the component, and both default to zero. $\Delta t_k$ is the duration of the newest preintegrated measurement, the time from the previous frame to the current one.

The residual is not marginalised into any prior. It exists only within one frame's optimisation, like the zero velocity term (`sqrt_keypoint_vio.cpp:1773-1780`), because its variable, the within frame increment, is reset to zero by the re-centring at the end of the frame.

#### 3.4.2 The step it produces, and why it gates softly

Consider one scalar coordinate, with the data contributing information $I$ and pulling towards a correction $\Delta$, so that the data cost is $\frac{1}{2} I (a - \Delta)^2$, and the random walk contributing $\frac{1}{2} w^2 a^2$ with $w^2 = 1 / (q\,\Delta t)$. Step 1. The total cost is

$$F(a) = \frac{1}{2} I (a - \Delta)^2 + \frac{1}{2} w^2 a^2.$$

Step 2. Setting $F'(a) = I (a - \Delta) + w^2 a = 0$,

$$a^* = \frac{I}{I + w^2}\,\Delta.$$

Step 3. The applied step is the data's correction shrunk by the factor $I / (I + w^2)$. When the data carry little information, $I \ll w^2$, the step is small. When they carry much, $I \gg w^2$, the full correction is applied. Step 4. If the data keep the same pull in the next frame, because the correction was right, the remainder $(1 - \frac{I}{I + w^2})\,\Delta$ is pulled again, so after $n$ frames the applied total is

$$\Delta\Big(1 - \Big(\frac{w^2}{I + w^2}\Big)^n\Big),$$

a geometric convergence to the full correction. Step 5. A correction driven by noise, by contrast, changes sign from frame to frame and is attenuated by $I / (I + w^2)$ at every frame. The residual is therefore a soft information gate, which needs neither the block's marginal information, zero by §3.2.7, nor a threshold. It also bounds the per frame step statistically, by $\sqrt{q\,\Delta t}$ when the data are uninformative, which is what stops the swap ratchet of §3.2.5.

#### 3.4.3 Interaction with the gauge projection

The residual acts on the block coordinates, which are not gauge invariant. Consider the scale part, $\mathbf{H}_\text{rw} = w_\sigma^2\,\mathbf{e}_\sigma \mathbf{e}_\sigma^\top$, with $\mathbf{e}_\sigma$ the unit vector of the block's $\sigma$ coordinate. Step 1. Along the scale gauge direction its curvature is

$$\mathbf{n}_\sigma^\top \mathbf{H}_\text{rw} \mathbf{n}_\sigma = w_\sigma^2 (\mathbf{e}_\sigma^\top \mathbf{n}_\sigma)^2 = w_\sigma^2,$$

which is not zero, so without the projection the residual would be a gauge prior. It would prefer the representative with $\sigma = 0$, in which every correction is carried by the frames, which is today's estimator (first plan §2.5.4). Step 2. The residual is therefore added to $\mathbf{H}$ and $\mathbf{b}$ before the projection of §3.1.3, and enters the solve as

$$\mathbf{P} \mathbf{H}_\text{rw} \mathbf{P} = w_\sigma^2 (\mathbf{P} \mathbf{e}_\sigma)(\mathbf{P} \mathbf{e}_\sigma)^\top.$$

Step 3. For the scale direction alone $\mathbf{P} = \mathbf{I} - \mathbf{n}_\sigma \mathbf{n}_\sigma^\top / |\mathbf{n}_\sigma|^2$, and with $\mathbf{n}_\sigma^\top \mathbf{e}_\sigma = -1$,

$$\mathbf{P} \mathbf{e}_\sigma = \mathbf{e}_\sigma + \frac{\mathbf{n}_\sigma}{|\mathbf{n}_\sigma|^2},$$

whose $\sigma$ entry is $1 - 1 / |\mathbf{n}_\sigma|^2$ and whose frame entries are $\mathbf{p}_f / |\mathbf{n}_\sigma|^2$ and $\mathbf{v}_f / |\mathbf{n}_\sigma|^2$. The projected residual penalises a combination of the block's scale step and a uniform radial scaling of the frames, which is the physical scale change in the minimum norm representation the projection selects. Step 4. Its squared length, by the symmetry and idempotence of $\mathbf{P}$, is

$$|\mathbf{P} \mathbf{e}_\sigma|^2 = \mathbf{e}_\sigma^\top \mathbf{P} \mathbf{e}_\sigma = 1 - \frac{(\mathbf{e}_\sigma^\top \mathbf{n}_\sigma)^2}{|\mathbf{n}_\sigma|^2} = 1 - \frac{1}{|\mathbf{n}_\sigma|^2},$$

so the effective weight is $w_\sigma^2 (1 - 1 / |\mathbf{n}_\sigma|^2)$. At rest at the origin it vanishes, consistent with the projection already removing every scale step there (§3.2.6). With three states moving at 1 m/s and keyframes near the origin, $|\mathbf{n}_\sigma|^2 \approx 4$ and the effective weight is $0.75\,w_\sigma^2$, so the residual acts from the first second of the climb. The tilt components behave in the same way with $|\mathbf{n}_\mathbf{a}|^2$ of §3.1.3, which is at least $1 + K$ for $K$ frames, so their effective weight is at least $1 - 1/(1 + K)$, about 0.9 for the ten frames of the SITL window.

Step 5, consistency of the bookkeeping. For a step $\mathbf{x}$ orthogonal to $\mathbf{N}$, which the final projection guarantees, $\mathbf{P} \mathbf{x} = \mathbf{x}$, so

$$\mathbf{x}^\top \mathbf{P} \mathbf{H}_\text{rw} \mathbf{P} \mathbf{x} = \mathbf{x}^\top \mathbf{H}_\text{rw} \mathbf{x},$$

$$\mathbf{x}^\top \mathbf{P} \mathbf{b}_\text{rw} = \mathbf{x}^\top \mathbf{b}_\text{rw}.$$

The quadratic model of the projected system and that of the unprojected residual agree on every applied step, so the model decrease of §3.4.4 can be computed from the unprojected residual and the applied step.

#### 3.4.4 Entry into the normal equations and the cost bookkeeping

Basalt solves $\mathbf{H}\,\mathbf{x} = \mathbf{b}$ with $\mathbf{H} = \mathbf{J}^\top \mathbf{J}$ and $\mathbf{b} = \mathbf{J}^\top \mathbf{r}$ and applies $-\mathbf{x}$ (`context/linearisation.md`, `sqrt_keypoint_vio.cpp:1893` and `:1924`). Write $\mathbf{w}^2 = (w_\theta^2, w_\theta^2, w_\sigma^2)$ for the diagonal of $\mathbf{W}_\text{rw}^\top \mathbf{W}_\text{rw}$ and $\odot$ for the elementwise product.

Step 1. The residual $\mathbf{W}_\text{rw}\,\mathbf{a}$ has the Jacobian $\mathbf{W}_\text{rw}$ with respect to the block increment, because an increment $\boldsymbol{\epsilon}$ changes $\mathbf{a}$ to $\mathbf{a} + \boldsymbol{\epsilon}$ exactly. Step 2. Its contributions to the system at the block's offsets 0 to 2 are

$$\mathbf{H}_{gg} \leftarrow \mathbf{H}_{gg} + \text{diag}(\mathbf{w}^2),$$

$$\mathbf{b}_g \leftarrow \mathbf{b}_g + \mathbf{w}^2 \odot \mathbf{a}.$$

Step 3. Its cost at the linearisation of an outer iteration enters `error_total` as

$$E_\text{rw}(\mathbf{a}) = \frac{1}{2}\sum_{i=1}^{3} w_i^2 a_i^2.$$

Step 4. For the applied block step $\mathbf{d}$, the block part of the negated and projected increment, the residual is exactly quadratic, so the model decrease equals the actual decrease,

$$E_\text{rw}(\mathbf{a}) - E_\text{rw}(\mathbf{a} + \mathbf{d}) = \frac{1}{2}\mathbf{a}^\top \text{diag}(\mathbf{w}^2)\,\mathbf{a} - \frac{1}{2}(\mathbf{a} + \mathbf{d})^\top \text{diag}(\mathbf{w}^2)(\mathbf{a} + \mathbf{d}),$$

$$E_\text{rw}(\mathbf{a}) - E_\text{rw}(\mathbf{a} + \mathbf{d}) = -\sum_{i=1}^{3} w_i^2 d_i \Big(a_i + \frac{1}{2} d_i\Big),$$

which is added to `l_diff`, the same form as the zero velocity term's `l_diff -= w2 * dv.dot(v + 0.5 * dv)` (`:1930-1937`). Step 5. `after_error_total` receives $E_\text{rw}(\mathbf{a} + \mathbf{d})$, and $\mathbf{a}$ advances to $\mathbf{a} + \mathbf{d}$ only when the step is accepted. A rejected step restores the block by `restore()` and leaves $\mathbf{a}$ unchanged. The three terms of the cost triple of `context/vio_opt.md` §9.3 therefore all contain the residual, and the acceptance test judges the objective the step was solved for.

Step 6, the block before its first freezing. Until the first marginalisation the block is not frozen, and `GravityScaleStateWithLin::ApplyInc` moves its linearisation point and transports the basis instead of adding to an offset (`imu_types.h:357-366`). The accumulated $\mathbf{a}$ is then the sum of tangent increments taken in successively transported bases, which equals the tangent coordinate of the total correction to first order. This affects only the first optimised frame of a run, at rest, where the scale part is projected out anyway.

#### 3.4.5 Choice of the intensities

The per frame standard deviation is $\sqrt{q\,\Delta t}$. At 20 frames per second, $\Delta t = 0.05$ s, the proposed `vio_gs_jump_sigma_rw` $= 0.1$ gives $0.022$ per frame and $w_\sigma^2 = 2000$, and `vio_gs_jump_tilt_rw_deg` $= 1$ gives $0.22°$, 0.0039 rad, per frame and $w_\theta^2 = 6.6 \cdot 10^4$ rad⁻².

For the scale, the per frame standard deviation of 0.022 is less than half the swap interval $\ln d_\text{min} = 0.048$, so a step driven by uninformative data crosses the interval with a probability of about 3 %, against almost every frame in the replays. The information of one accelerating factor on $\sigma$ is $|\mathbf{c}|^2 = |\mathbf{a}|^2 \Delta t / \sigma_c^2$ by §3.3.4, which is 31 at 2 m/s² and 195 at 5 m/s². A single factor therefore moves the scale by 1.5 % to 9 % of its pull per frame, while $M_\text{curr}$ accumulates the information of every accelerating factor since its last reset, about 600 after one second at 2 m/s², which moves it by 23 % per frame. A scale error of a factor of 2.7, the mode off error of the replays, is then corrected to within 10 % in about ten frames once the climb is under way, by Step 4 of §3.4.2.

For the tilt, the information with the bias anchored is bounded by the bias prior, $|\mathbf{g}|^2 / \sigma_{b_a}^2 \approx 9600$ rad⁻² at $\sigma_{b_a} = 0.1$ m/s² (§3.2.3), so a tilt correction is applied at about 13 % per frame and completed in about fifteen frames, while a noise driven tilt step is attenuated to 13 %. The replays drifted at 7° per second, seven times the one second standard deviation of the random walk, 1°, so a drift of that rate would need the data to pull consistently in one direction, which with the bias anchored by Change A they no longer do.

Both values are starting points for the sweep of §5.4, which varies each by a factor of two in either direction.

#### 3.4.6 Why within the solve, and not by limiting what the re-centring applies

Partial re-centring, which applies only part of the block's value at the end of the frame and leaves the remainder in the block, is exact as a change of variables (first plan §2.7.3), but it breaks an invariant that four consumers rely on. After `optimize` the block is the identity, so `marginalize` builds and strips the mapper's packet at $s = 1$ and $\mathbf{g}_D = \mathbf{g}$ (`StripGsFromMargData`, `sqrt_keypoint_vio.cpp:2460-2481`, first plan §2.9.3), `PublishKeyframe` sends metric poses, the published state is metric, and prediction and triangulation in the next `measure` assume $s = 1$ and $\mathbf{g}_D = \mathbf{g}$ (first plan §2.3.4). A remainder in the block would require a conversion at each of these. The residual of §3.4.1 limits the step inside the solve, where the result is still re-centred in full, so every consumer keeps its present contract.

### 3.5 Validation of the formulation against regressions

This section checks, configuration by configuration, that the changes can only improve the present behaviour, and names the two places where an approximation enters.

1. Mode off, `vio_gs_enable = false`. Every new statement is inside a branch on `mpGsActive` or on a new key, `mGsState` is empty, and `ImuBlockGS` and `LinearizationGSAbsQR` are never constructed. The four new keys are read and stored only. The execution path is identical, which §5.1 item 1 confirms by reading.
2. Mode on with the new keys at their defaults, `vio_gs_visual_prior_inertial = false` and both intensities zero. `LinearizeMargGS` passes no inertial data to the $M_\text{visual}$ linearisation and the anchor copy of today, the resets clear $M_\text{half}$'s anchor as today, and the random walk branch is not entered. The `vio_gravity_scale` record gains four columns that are written as zero. The estimate is identical to today's mode on estimate.
3. Change A on. By §3.3.6 Step 4 and (N2), every prior after every swap holds at least the information it held before, so no direction loses information. The added rows are exact, rotation rows by §3.3.3 and bias rows by §3.3.2, except the zeroed scale column of admitted full rows, whose error after a scale change $\Delta\sigma$ is at most $\tau\,|\Delta\sigma|$ standard deviations per factor by §3.3.4. The tilt columns are first estimate consistent by §3.3.5, with a second order linearisation error by §3.3.1. The gauge projection removes the only gauge effect of the zeroed column by §3.3.4 Step 4. The cost is one inertial block, 15 rows, per marginalisation in the $M_\text{visual}$ linearisation, against several hundred landmark rows.
4. Change B on. The residual changes each frame's minimum towards a smaller correction, never the direction of the correction, and by §3.4.2 Step 4 a correction the data keep supporting is completed geometrically. Its cost on the converged estimate is therefore a lag of a few frames, bounded by §3.4.5. It is not stored in any prior, so it cannot accumulate spurious information. It is added before the projection, so it acts on the physical correction only (§3.4.3). Its bookkeeping is exact (§3.4.4).
5. The local mapper, the published state and the fold. Neither change alters the end of `optimize`, which re-centres in full, nor the packet built from $M_\text{curr}$, nor the fold, so the contracts of the first plan §2.8 to §2.10 hold unchanged.
6. Numerical conditioning. $\boldsymbol{\Sigma}_{RR}$ is positive definite and a $3 \times 3$ Cholesky factorisation cannot fail on it. A departing velocity in $M_\text{visual}$ with rotation rows only has no constraint and a zero column, which the rank threshold of the square root helper drops, as it already does today for every departing velocity of $M_\text{visual}$ (§2.1). The random walk adds a positive diagonal to $\mathbf{H}$, which can only improve the conditioning of the damped solve.

The two approximations are the zeroed scale column of admitted factors, bounded by the admission test, and the first order accumulation of the block step before the first freezing (§3.4.4 Step 6). Neither is new in kind, since $M_\text{visual}$ already zeroes the visual scale column and the first plan already accumulates the block step to first order.

### 3.6 What is not changed, and why

1. Delayed marginalisation of the inertial chain, after DM-VIO [2] (`context/dynamic_marginalisation.md` §4.2). It keeps every inertial factor of the last $d$ keyframes and relinearises it at the new scale, which would also keep the scale information of the accelerating factors that Change A still loses at a swap. It requires a second graph that stores the departed states, their inertial factors and a frozen visual prior, a readvance by successive Schur complements, and a replacement rule, which is several hundred lines across the estimator and the linearisation. Change A addresses the observed mechanism, the loss of the tilt and bias anchor, with one constructor flag and one branch. Delayed marginalisation remains the next step if the replays of §5.4 show that the loss of accelerating factors at swaps still limits the scale.
2. Composition of preintegrated factors (`context/dynamic_marginalisation.md` §4.3). In Basalt a keyframe keeps its pose and loses its velocity and biases at marginalisation, so a state is never removed whole and composition applies only to non keyframe states, or to a variant that keeps nine extra coordinates per keyframe. Its benefit overlaps with item 1.
3. Scale aware recovered factors (`context/dynamic_marginalisation.md` §4.4). They concern the mapper's factors, whose consistency the first plan §2.9.2 already maintains exactly through the correction log.
4. Partial re-centring. It breaks the identity of the block after `optimize` on which four consumers rely (§3.4.6).
5. A hard information gate on the block. The block's marginal information is identically zero (§3.2.7), and a gate on the stationary detector is unnecessary because the projection already removes every scale step at rest (§3.2.6).
6. Removing the tilt from the block, which the first plan's record of runs listed as option 4. Change A removes the cause of the tilt drift, the loss of the bias anchor at swaps, while the block's tilt lets the estimator correct a wrong initial gravity direction exactly through the re-centring. A very small tilt intensity is not a substitute for removing the tilt. The projection keeps a share $1 - 1/|\mathbf{n}_\mathbf{a}|^2$, about 0.9, of every physical tilt correction in the block (§3.4.3), so a stiff tilt residual would suppress the physical tilt correction itself, whereas a block without tilt leaves the frames free to carry it as today. If the replays with Changes A and B still show a tilt worse than the mode off runs, removing the tilt is a code change for a later revision.
7. The accelerometer and gyroscope bias weight transposition. It is inherited from upstream, and correcting it would change every existing configuration's behaviour (`context/linearisation.md`). The SITL configuration should instead set `vio_init_bg_weight`, which reaches the accelerometer bias, from the accelerometer's turn on bias, which §5.4 sweeps.
8. The rebuilding of frozen keyframe linearisation points by accepted mapper pose updates (`sqrt_keypoint_vio.cpp:640-650`), which moves a frozen pose's linearisation point without re-expressing the priors. It is a candidate source of spurious information in both modes (first plan §2.5.6, the four effects), and investigating it is a separate task.

## 4. Implementation Plan

### 4.1 Architecture

#### 4.1.1 Current, in the initialisation period

```
measure ── apply mapper pose updates ── predict ── associate ── triangulate
   │
   ├─ optimize
   │    aom = [gs(3) | poses(6)… | states(15)…]
   │    LinearizationGSAbsQR(M_curr, ImuBlockGS full rows)
   │    LM loop: H,b ← dense; + zero velocity; H ← P H P, b ← P b; solve; x ← P x
   │    RecentreGravityScale: Φ_c applied to states, three priors, two anchors
   │
   ├─ marginalize
   │    UpdateScaleInterval (swap rule, resets clear M_half's anchor)
   │    LinearizeMargGS
   │       M_curr   : landmarks + ImuBlockGS full + M_curr,   anchor A_curr
   │       M_half   : landmarks + ImuBlockGS full + M_half,   anchor A_half
   │       M_visual : landmarks (σ column zeroed) + M_visual,  copy of A_curr
   │    MarginalizeGS, RecentrePriorsGS
   │    LogGravityScale
   └─ PublishKeyframe
```

#### 4.1.2 Updated

```
   ├─ optimize
   │    LM loop: H,b ← dense; + zero velocity; + random walk on gs step (Change B);
   │             H ← P H P, b ← P b; solve; x ← P x
   │             error_total, l_diff, after_error_total include ½ Σ w_i² a_i²
   │    RecentreGravityScale: also re-expresses A_visual
   │
   ├─ marginalize
   │    UpdateScaleInterval (resets copy A_visual's tilt with the current σ)
   │    LinearizeMargGS
   │       M_curr   : unchanged
   │       M_half   : unchanged
   │       M_visual : landmarks (σ zeroed) + ImuBlockGS scale free rows (Change A)
   │                  + M_visual, anchor A_visual
   │    RecentrePriorsGS: M_visual against A_visual
   │    LogGravityScale: + rw cost, factor counts per form, largest |c|
```

Both changes are switched by configuration keys. With the keys at their defaults the updated diagram reduces to the current one.

#### 4.1.3 Class relations

`ImuBlockGS` (`include/basalt/linearization/imu_block.hpp`) gains a scale free mode, chosen per instance by two optional constructor arguments. `LinearizationGSAbsQR` (`include/basalt/linearization/linearization_abs_qr.hpp`, `src/linearization/linearization_abs_qr.cpp`) forwards them through two optional constructor arguments and reports how many factors took each form. `SqrtKeypointVioEstimator` (`include/basalt/vi_estimator/sqrt_keypoint_vio.h`, `src/vi_estimator/sqrt_keypoint_vio.cpp`) owns a third anchor and the random walk residual. `VioConfig` (`include/basalt/utils/vio_config.h`, `src/utils/vio_config.cpp`) gains four keys, and `Logger` (`include/basalt/utils/logger.h`, `src/utils/logger.cpp`) four columns of `vio_gravity_scale`.

### 4.2 Implementation

The changes are listed in the order in which they compile, each with the code to be placed in the working tree, its rationale and its backwards compatibility.

#### Change 1. Configuration keys

Need. Changes A and B must be selectable independently, and both must default to the present behaviour.

`include/basalt/utils/vio_config.h`, after `double vio_gs_dmin;` (`:133`).

```cpp
    // VI-DSO d_min of the dynamic marginalisation scale interval.
    double vio_gs_dmin;
    // Linearise M_visual with the scale free rows of the inertial factors,
    // plans/gravity_scale_optimisation_2.md Change A.
    bool vio_gs_visual_prior_inertial;
    // Largest whitened scale column with which an inertial factor enters
    // M_visual with all of its rows.
    double vio_gs_visual_prior_scale_tol;
    // Random walk intensities of the per frame gravity and scale step, per
    // square root second, zero disables, plans/gravity_scale_optimisation_2.md
    // Change B.
    double vio_gs_jump_sigma_rw;
    double vio_gs_jump_tilt_rw_deg;
```

`src/utils/vio_config.cpp`, defaults after `vio_gs_dmin = 1.0488088481701516;` (`:103`).

```cpp
  vio_gs_dmin = 1.0488088481701516;
  vio_gs_visual_prior_inertial = false;
  vio_gs_visual_prior_scale_tol = 1.0;
  vio_gs_jump_sigma_rw = 0.0;
  vio_gs_jump_tilt_rw_deg = 0.0;
```

`src/utils/vio_config.cpp`, serialisation after `ar(CEREAL_NVP(config.vio_gs_dmin));` (`:241`).

```cpp
  ar(CEREAL_NVP(config.vio_gs_dmin));
  ar(CEREAL_NVP(config.vio_gs_visual_prior_inertial));
  ar(CEREAL_NVP(config.vio_gs_visual_prior_scale_tol));
  ar(CEREAL_NVP(config.vio_gs_jump_sigma_rw));
  ar(CEREAL_NVP(config.vio_gs_jump_tilt_rw_deg));
```

A key missing from a JSON file is fatal with an uncaught `cereal::Exception` (`context/vio_config_json.md`), so every configuration file that carries `config.vio_gs_dmin` receives the four keys after that line. These are `data/euroc_config.json`, `data/euroc_config_no_factors.json`, `data/euroc_config_no_weights.json`, `data/euroc_config_vo.json`, `data/kitti_config.json` and `data/tumvi_512_config.json`, each at `:51`, with the defaults,

```json
        "config.vio_gs_dmin": 1.0488088481701516,
        "config.vio_gs_visual_prior_inertial": false,
        "config.vio_gs_visual_prior_scale_tol": 1.0,
        "config.vio_gs_jump_sigma_rw": 0.0,
        "config.vio_gs_jump_tilt_rw_deg": 0.0,
```

`data/iccv21/basalt_batch_config.toml` after `:59`,

```toml
"config.vio_gs_dmin" = 1.0488088481701516
"config.vio_gs_visual_prior_inertial" = false
"config.vio_gs_visual_prior_scale_tol" = 1.0
"config.vio_gs_jump_sigma_rw" = 0.0
"config.vio_gs_jump_tilt_rw_deg" = 0.0
```

and `data/sitl_config_vo.json` after `:51` with both changes enabled at the values of §3.3.4 and §3.4.5,

```json
        "config.vio_gs_dmin": 1.0488088481701516,
        "config.vio_gs_visual_prior_inertial": true,
        "config.vio_gs_visual_prior_scale_tol": 1.0,
        "config.vio_gs_jump_sigma_rw": 0.1,
        "config.vio_gs_jump_tilt_rw_deg": 1.0,
```

The search of 2026-11-06 over `/ws/ros_ws/src` for `config.vio_max_states` in JSON, TOML, YAML and Python files found exactly these eight files.

Backwards compatibility. The defaults reproduce the present behaviour in every mode (§3.5 items 1 and 2). Only `data/sitl_config_vo.json`, which already enables the mode, changes behaviour, and the user's `vio_gs_duration_s` of 7000 s there is left as it is.

#### Change 2. Scale free rows in `ImuBlockGS`, `include/basalt/linearization/imu_block.hpp`

Need. §3.3.2 to §3.3.4. In scale free mode an inertial factor is linearised either with all rows and the scale column zeroed, when its whitened scale column passes the admission test, or with the rotation rows weighted by $\mathbf{L}_R$ and every position and velocity row removed. The bias random walk rows are kept in both forms.

The constructor gains two optional arguments, and three members and one accessor are added. The class head and members become as follows, with the unchanged methods `add_dense_Q2Jp_Q2r`, `add_dense_H_b`, `scaleJp_cols`, `addJp_diag2` and `backSubstitute` left as they are (`imu_block.hpp:338-412`).

```cpp
template <class Scalar_>
class ImuBlockGS : public ImuBlock<Scalar_> {
 public:
  using Scalar = Scalar_;
  using Base = ImuBlock<Scalar>;
  using VecX = typename Base::VecX;
  using MatX = typename Base::MatX;
  using Vec3 = typename Base::Vec3;
  using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
  using MatGs = Eigen::Matrix<Scalar, POSE_VEL_BIAS_SIZE, GRAVITY_SCALE_SIZE>;

  // With scaleFreeRows, the factor keeps all rows with the scale column zeroed
  // when the whitened scale column norm is at most scaleFreeTol, and the
  // rotation rows weighted by their marginal covariance otherwise.
  ImuBlockGS(const IntegratedImuMeasurement<Scalar>* meas,
             const ImuLinData<Scalar>* imu_lin_data, const AbsOrderMap& aom,
             const GravityScaleStateWithLin<Scalar>* gs,
             bool scaleFreeRows = false, Scalar scaleFreeTol = Scalar(0))
      : Base(meas, imu_lin_data, aom),
        mpGs(gs),
        mpGsIdx(aom.abs_order_map.at(kGravityScaleStateId).first),
        mpScaleFreeRows(scaleFreeRows),
        mpScaleFreeTol(scaleFreeTol) {
    mpJgs.setZero();
  }
```

`linearizeImu` gains the scale free branch between the block Jacobians and the bias random walk. The full method reads as follows, unchanged up to the line `this->r.template segment<9>(0) = ...`.

```cpp
  Scalar linearizeImu(
      const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<Scalar>>&
          frame_states) override {
    this->Jp.setZero();
    this->r.setZero();
    mpJgs.setZero();

    const int64_t start_t = this->imu_meas->get_start_t_ns();
    const int64_t end_t =
        this->imu_meas->get_start_t_ns() + this->imu_meas->get_dt_ns();

    const size_t start_idx = 0;
    const size_t end_idx = POSE_VEL_BIAS_SIZE;

    PoseVelBiasStateWithLin<Scalar> start_state = frame_states.at(start_t);
    PoseVelBiasStateWithLin<Scalar> end_state = frame_states.at(end_t);

    typename IntegratedImuMeasurement<Scalar>::MatNN d_res_d_start, d_res_d_end;
    typename IntegratedImuMeasurement<Scalar>::MatN3 d_res_d_bg, d_res_d_ba,
        d_res_d_gD;
    typename PoseVelState<Scalar>::VecN d_res_d_logscale;

    const Scalar g_norm = this->imu_lin_data->g.norm();
    const Vec3 g_D_lin = g_norm * mpGs->mDirLin;

    typename PoseVelState<Scalar>::VecN res = this->imu_meas->residualGS(
        start_state.getStateLin(), g_D_lin, end_state.getStateLin(),
        start_state.getStateLin().bias_gyro,
        start_state.getStateLin().bias_accel, mpGs->GetScaleLin(),
        &d_res_d_start, &d_res_d_end, &d_res_d_bg, &d_res_d_ba, &d_res_d_gD,
        &d_res_d_logscale);

    if (start_state.isLinearized() || end_state.isLinearized() ||
        mpGs->mLinearized) {
      res = this->imu_meas->residualGS(
          start_state.getState(), Vec3(g_norm * mpGs->GetDir()),
          end_state.getState(), start_state.getState().bias_gyro,
          start_state.getState().bias_accel, mpGs->GetScale());
    }

    Scalar imu_error = Scalar(0.5) *
                       (this->imu_meas->get_sqrt_cov_inv() * res).squaredNorm();

    this->Jp.template block<9, 9>(0, start_idx) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_start;
    this->Jp.template block<9, 9>(0, end_idx) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_end;

    this->Jp.template block<9, 3>(0, start_idx + 9) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_bg;
    this->Jp.template block<9, 3>(0, start_idx + 12) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_ba;

    const Eigen::Matrix<Scalar, 3, 2> d_gD_d_theta =
        -Sophus::SO3<Scalar>::hat(g_D_lin) * mpGs->mBasisLin;

    mpJgs.template block<9, 2>(0, 0) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_gD * d_gD_d_theta;
    mpJgs.template block<9, 1>(0, 2) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_logscale;

    this->r.template segment<9>(0) = this->imu_meas->get_sqrt_cov_inv() * res;

    if (mpScaleFreeRows) {
      mpScaleColNorm = mpJgs.col(2).norm();
      mpUsedFullRows = mpScaleColNorm <= mpScaleFreeTol;
      if (mpUsedFullRows) {
        mpJgs.col(2).setZero();
      } else {
        const Mat3 sqrt_inv_cov_rot =
            Eigen::LLT<Mat3>(
                this->imu_meas->get_cov().template block<3, 3>(3, 3))
                .matrixL()
                .solve(Mat3::Identity());

        this->Jp.template topRows<9>().setZero();
        this->r.template head<9>().setZero();
        mpJgs.setZero();

        this->Jp.template block<3, 9>(3, start_idx) =
            sqrt_inv_cov_rot * d_res_d_start.template middleRows<3>(3);
        this->Jp.template block<3, 9>(3, end_idx) =
            sqrt_inv_cov_rot * d_res_d_end.template middleRows<3>(3);
        this->Jp.template block<3, 3>(3, start_idx + 9) =
            sqrt_inv_cov_rot * d_res_d_bg.template middleRows<3>(3);
        this->Jp.template block<3, 3>(3, start_idx + 12) =
            sqrt_inv_cov_rot * d_res_d_ba.template middleRows<3>(3);
        this->r.template segment<3>(3) =
            sqrt_inv_cov_rot * res.template segment<3>(3);

        imu_error = Scalar(0.5) * this->r.template segment<3>(3).squaredNorm();
      }
    }

    const std::pair<Scalar, Scalar> bias_error =
        this->LinearizeBiasRandomWalk(start_state, end_state);

    return imu_error + bias_error.first + bias_error.second;
  }
```

After `backSubstitute`, before `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` (`imu_block.hpp:414`), the accessor.

```cpp
  // False when the last linearisation kept the rotation rows only.
  bool UsedFullRows() const { return mpUsedFullRows; }
  // Whitened scale column norm of the last scale free linearisation.
  Scalar ScaleColNorm() const { return mpScaleColNorm; }
```

The private members become

```cpp
 private:
  const GravityScaleStateWithLin<Scalar>* mpGs;
  size_t mpGsIdx;
  MatGs mpJgs;
  bool mpScaleFreeRows;
  Scalar mpScaleFreeTol;
  bool mpUsedFullRows = true;
  Scalar mpScaleColNorm = 0;
};
```

Mathematical background. §3.3.3 for $\mathbf{L}_R$ and §3.3.4 for the test. `matrixL().solve(Identity)` returns $\mathbf{C}^{-1} = \mathbf{L}_R$ for $\boldsymbol{\Sigma}_{RR} = \mathbf{C} \mathbf{C}^\top$. The test uses `mpJgs.col(2)` before it is zeroed, which is $\mathbf{L}$ times the unwhitened scale column at the anchor, so its norm is the Mahalanobis norm of §3.3.4. Rows 9 to 14 are written afterwards by `LinearizeBiasRandomWalk`, which assigns the bias Jacobians and adds to `r` rows 9 to 14 only, so it is unaffected by the zeroing of rows 0 to 8. The remaining methods read `Jp`, `r` and `mpJgs` as they are, so the zero rows of the rotation only form contribute nothing to $\mathbf{H}$, $\mathbf{b}$, the dense square root rows or `l_diff`, and the zero scale column contributes nothing to the block's $\sigma$ column. `get_dense_Q2Jp_Q2r` still reserves 15 rows per factor (`linearization_abs_qr.cpp:564`), of which the zero rows are harmless to the Householder QR of the marginalisation.

Backwards compatibility. Both new arguments default to the present behaviour, `mpScaleFreeRows = false` skips the new branch, and the two existing constructions, in `LinearizationGSAbsQR` and in no test, pass four arguments. `ImuBlock` is untouched.

#### Change 3. `LinearizationGSAbsQR` forwards the mode and counts the forms

Need. `LinearizeMargGS` must construct the $M_\text{visual}$ linearisation with scale free inertial blocks and log how many factors took each form.

`include/basalt/linearization/linearization_abs_qr.hpp`, the class at `:143-166` becomes

```cpp
template <typename Scalar_, int POSE_SIZE_>
class LinearizationGSAbsQR : public LinearizationAbsQR<Scalar_, POSE_SIZE_> {
 public:
  using Scalar = Scalar_;
  static constexpr int POSE_SIZE = POSE_SIZE_;
  using Base = LinearizationAbsQR<Scalar, POSE_SIZE>;
  using typename Base::Options;

  LinearizationGSAbsQR(
      BundleAdjustmentBase<Scalar>* estimator, const AbsOrderMap& aom,
      const Options& options, const GravityScaleStateWithLin<Scalar>* gs,
      bool visualScaleFree, const MargLinData<Scalar>* marg_lin_data = nullptr,
      const ImuLinData<Scalar>* imu_lin_data = nullptr,
      const std::set<FrameId>* used_frames = nullptr,
      const std::unordered_set<KeypointId>* lost_landmarks = nullptr,
      int64_t last_state_to_marg = std::numeric_limits<int64_t>::max(),
      bool imuScaleFree = false, Scalar imuScaleFreeTol = Scalar(0));

  // Numbers of inertial factors linearised with all rows and with the
  // rotation rows only.
  std::pair<int, int> CountImuRowForms() const;

  // Largest whitened scale column norm over the inertial factors.
  Scalar MaxImuScaleColNorm() const;

 protected:
  void linearizeRelPoses() override;

 private:
  const GravityScaleStateWithLin<Scalar>* mpGs;
  bool mpVisualScaleFree;
};
```

`src/linearization/linearization_abs_qr.cpp`, the constructor at `:275-299` becomes

```cpp
template <typename Scalar, int POSE_SIZE>
LinearizationGSAbsQR<Scalar, POSE_SIZE>::LinearizationGSAbsQR(
    BundleAdjustmentBase<Scalar>* estimator, const AbsOrderMap& aom,
    const Options& options, const GravityScaleStateWithLin<Scalar>* gs,
    bool visualScaleFree, const MargLinData<Scalar>* marg_lin_data,
    const ImuLinData<Scalar>* imu_lin_data,
    const std::set<FrameId>* used_frames,
    const std::unordered_set<KeypointId>* lost_landmarks,
    int64_t last_state_to_marg, bool imuScaleFree, Scalar imuScaleFreeTol)
    : Base(estimator, aom, options, marg_lin_data, imu_lin_data, used_frames,
           lost_landmarks, last_state_to_marg),
      mpGs(gs),
      mpVisualScaleFree(visualScaleFree) {
  BASALT_ASSERT(gs != nullptr);
  BASALT_ASSERT(aom.abs_order_map.at(kGravityScaleStateId).first == 0);

  this->imu_blocks.clear();
  if (imu_lin_data) {
    for (const auto& kv : imu_lin_data->imu_meas) {
      this->imu_blocks.emplace_back(new ImuBlockGS<Scalar>(
          kv.second, imu_lin_data, aom, gs, imuScaleFree, imuScaleFreeTol));
    }
  }
}

template <typename Scalar, int POSE_SIZE>
std::pair<int, int>
LinearizationGSAbsQR<Scalar, POSE_SIZE>::CountImuRowForms() const {
  int full = 0;
  int rot = 0;
  for (const auto& imu_block : this->imu_blocks) {
    if (static_cast<const ImuBlockGS<Scalar>*>(imu_block.get())
            ->UsedFullRows()) {
      full++;
    } else {
      rot++;
    }
  }
  return {full, rot};
}

template <typename Scalar, int POSE_SIZE>
Scalar LinearizationGSAbsQR<Scalar, POSE_SIZE>::MaxImuScaleColNorm() const {
  Scalar max_norm = 0;
  for (const auto& imu_block : this->imu_blocks) {
    max_norm = std::max(
        max_norm,
        static_cast<const ImuBlockGS<Scalar>*>(imu_block.get())->ScaleColNorm());
  }
  return max_norm;
}
```

The `static_cast` is valid because this constructor fills `imu_blocks` with `ImuBlockGS` only. `<utility>` for `std::pair` is already reached through the Eigen and standard headers of the file.

Backwards compatibility. The two arguments are appended with defaults, so the call in `optimize` (`sqrt_keypoint_vio.cpp:1704-1706`), which passes seven arguments, is unchanged in meaning. `CountImuRowForms` is new and read only by Change 5.

#### Change 4. Estimator declarations, `include/basalt/vi_estimator/sqrt_keypoint_vio.h`

Need. The third anchor of §3.3.5, the factor counts and the random walk cost for the log.

After `GsAnchor mpAnchorHalf;` (`:278`),

```cpp
    GsAnchor mpAnchorHalf;
    // Tilt anchor of M_visual when it holds inertial rows,
    // plans/gravity_scale_optimisation_2.md §3.3.5.
    GsAnchor mpAnchorVisual;
```

After `Scalar mpGsStepTiltDeg = 0;` (`:285`),

```cpp
    Scalar mpGsStepTiltDeg = 0;
    Scalar mpGsJumpCost = 0;
    int mpGsVisualImuFull = 0;
    int mpGsVisualImuRot = 0;
    Scalar mpGsVisualImuScaleCol = 0;
```

and one private helper next to `AnchorGs` (`:271`),

```cpp
    void AnchorGs(GsAnchor& anchor);
    void ResetHalfFromVisual(Scalar sigma);
```

Backwards compatibility. Private members and a private method only.

#### Change 5. Anchors, resets and logging of the three priors, `src/vi_estimator/sqrt_keypoint_vio.cpp`

Need. §3.3.5 and §3.3.6. $M_\text{visual}$ is linearised with scale free inertial rows against its own anchor, the anchor is re-expressed by every re-centring, a reset of $M_\text{half}$ carries it with the current scale, and the forms are logged.

At the top of `marginalize`, after `mpGsSwapEvent = 0;` (`:1077`), the counts are cleared so that a frame without marginalisation logs zero.

```cpp
    mpGsSwapEvent = 0;
    mpGsVisualImuFull = 0;
    mpGsVisualImuRot = 0;
    mpGsVisualImuScaleCol = 0;
```

The new helper, placed before `UpdateScaleInterval` (`:2333`).

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::ResetHalfFromVisual(Scalar sigma) {
    mpMargHalf = mpMargVisual;
    if (config.vio_gs_visual_prior_inertial && mpAnchorVisual.set) {
        // The copied scale columns are zero, so the scale anchor is free and
        // is taken at the current value, plans/gravity_scale_optimisation_2.md
        // §3.3.5.
        mpAnchorHalf = mpAnchorVisual;
        mpAnchorHalf.sigma = sigma;
    } else {
        mpAnchorHalf.set = false;
    }
}
```

`UpdateScaleInterval` (`:2333-2370`) becomes the following, in which the three resets call the helper and nothing else changes.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::UpdateScaleInterval() {
    const Scalar sigma = this->mGsState->GetSigma();
    const Scalar log_dmin = std::log(Scalar(config.vio_gs_dmin));

    const Scalar step = std::abs(sigma - mpSigmaPrevMarg);
    mpLogD = log_dmin;
    while (mpLogD <= step) mpLogD += log_dmin;
    mpSigmaPrevMarg = sigma;

    const bool upper = sigma > mpSigmaMid;

    if (upper != mpLastUpper) {
        ResetHalfFromVisual(sigma);
        mpGsSwapEvent = 1;
    }

    if (sigma > mpSigmaMid + mpLogD) {
        marg_data = mpMargHalf;
        mpAnchorCurr = mpAnchorHalf;
        ResetHalfFromVisual(sigma);
        mpSigmaMid += mpLogD;
        mpGsSwapEvent = 2;
    }

    if (sigma < mpSigmaMid - mpLogD) {
        marg_data = mpMargHalf;
        mpAnchorCurr = mpAnchorHalf;
        ResetHalfFromVisual(sigma);
        mpSigmaMid -= mpLogD;
        mpGsSwapEvent = 3;
    }

    mpLastUpper = upper;
}
```

`LinearizeMargGS` (`:2372-2420`), from the lambda to the end of the function, becomes

```cpp
    auto linearize = [&](const MargLinData<Scalar>& mld, GsAnchor& anchor,
                         bool with_imu, bool visual_scale_free,
                         bool imu_scale_free, MatX& Q2Jp, VecX& Q2r) {
        AnchorGs(anchor);

        LinearizationGSAbsQR<Scalar, POSE_SIZE> lqr(
            this, aom, lqr_options, &*this->mGsState, visual_scale_free, &mld,
            with_imu ? &ild : nullptr, &kfs_to_marg, &lost_landmaks,
            last_state_to_marg, imu_scale_free,
            Scalar(config.vio_gs_visual_prior_scale_tol));

        lqr.linearizeProblem();
        lqr.performQR();
        lqr.get_dense_Q2Jp_Q2r(Q2Jp, Q2r);

        if (imu_scale_free) {
            const std::pair<int, int> forms = lqr.CountImuRowForms();
            mpGsVisualImuFull = forms.first;
            mpGsVisualImuRot = forms.second;
            mpGsVisualImuScaleCol = lqr.MaxImuScaleColNorm();
        }
    };

    const bool visual_inertial = config.vio_gs_visual_prior_inertial;
    GsAnchor visual_anchor_copy = mpAnchorCurr;
    GsAnchor& visual_anchor =
        visual_inertial ? mpAnchorVisual : visual_anchor_copy;
    // The scale columns of M_visual are zero, but the anchor's scale is where
    // its pose and state Jacobians are evaluated, so it follows the current
    // scale, plans/gravity_scale_optimisation_2.md §3.3.5.
    if (visual_inertial && mpAnchorVisual.set) {
        mpAnchorVisual.sigma = this->mGsState->GetSigma();
    }

    linearize(marg_data, mpAnchorCurr, true, false, false, Q2Jp_curr,
              Q2r_curr);
    linearize(mpMargHalf, mpAnchorHalf, true, false, false, Q2Jp_half,
              Q2r_half);
    linearize(mpMargVisual, visual_anchor, visual_inertial, true,
              visual_inertial, Q2Jp_visual, Q2r_visual);

    AnchorGs(mpAnchorCurr);
}
```

`RecentrePriorsGS` (`:2442-2458`) becomes

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::RecentrePriorsGS() {
    auto recentre = [&](MargLinData<Scalar>& mld, GsAnchor& anchor) {
        AnchorGs(anchor);
        VecX delta;
        computeDelta(mld.order, delta);
        mld.b -= mld.H * delta;
    };

    GsAnchor visual_anchor_copy = mpAnchorCurr;
    GsAnchor& visual_anchor = config.vio_gs_visual_prior_inertial
                                  ? mpAnchorVisual
                                  : visual_anchor_copy;

    recentre(marg_data, mpAnchorCurr);
    recentre(mpMargHalf, mpAnchorHalf);
    recentre(mpMargVisual, visual_anchor);

    AnchorGs(mpAnchorCurr);
}
```

In `RecentreGravityScale`, after `transform_anchor(mpAnchorHalf);` (`:2308`),

```cpp
    transform_anchor(mpAnchorCurr);
    transform_anchor(mpAnchorHalf);
    transform_anchor(mpAnchorVisual);
```

In `FoldGravityScale`, after `mpMargVisual = MargLinData<Scalar>();` (`:2517`),

```cpp
    mpMargHalf = MargLinData<Scalar>();
    mpMargVisual = MargLinData<Scalar>();
    mpAnchorVisual.set = false;
```

`LogGravityScale` (`:2212-2221`) passes the three new values.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::LogGravityScale(int swapEvent) {
    const int64_t epoch = this->mGsCorrectionLog->LatestEpoch();
    const GravityScaleCorrectionLog::Entry cumulative =
        this->mGsCorrectionLog->Between(0, epoch);
    mpLogger->AddVioGravityScale(
        last_state_t_ns, epoch, double(mpGsStepSigma), double(mpGsStepTiltDeg),
        cumulative.mScale, cumulative.mQ.log().norm() * 180.0 / M_PI,
        swapEvent, double(mpGsJumpCost), mpGsVisualImuFull, mpGsVisualImuRot,
        double(mpGsVisualImuScaleCol));
}
```

Design rationale. The visual anchor is a reference, so that `AnchorGs` records it at the first marginalisation and every later use sees the same point, which is the first estimate rule of §2.2. The copy of $M_\text{curr}$'s anchor is kept for the default path, where it reproduces today's arithmetic exactly. `transform_anchor` already returns early for an anchor that is not set, so the added call is inert with the flag off. The order of the three `linearize` calls and the copy taken before the first call are those of today.

Backwards compatibility. With `vio_gs_visual_prior_inertial = false`, `ResetHalfFromVisual` clears the anchor as the three removed lines did, `visual_anchor` refers to the copy as today, `with_imu` and `imu_scale_free` are false for $M_\text{visual}$ so its linearisation receives no inertial data as today, `mpAnchorVisual` is never set, and the counts stay zero. The mode off path never calls any of these functions.

#### Change 6. The random walk residual in `optimize`, `src/vi_estimator/sqrt_keypoint_vio.cpp`

Need. §3.4.1 to §3.4.4.

After `bool converged = false;` (`:1719`), the weights and the accumulated step of the frame.

```cpp
        bool terminated = false;
        bool converged = false;

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
        const bool applyGsJump = (gsJumpW2.array() > Scalar(0)).any();
```

After the zero velocity term of `error_total` (`:1777-1781`),

```cpp
            if (applyZeroVelocity)
                error_total += Scalar(0.5) * zeroVelocityW2 *
                               frame_states.at(last_state_t_ns)
                                   .getState()
                                   .vel_w_i.squaredNorm();
            if (applyGsJump)
                error_total +=
                    Scalar(0.5) * gsJumpW2.dot(gsJump.cwiseProduct(gsJump));
```

After the zero velocity block of `H` and `b` and before `MatX gauge_P;` (`:1860-1870`), so that the projection acts on it (§3.4.3).

```cpp
                    if (applyGsJump) {
                        H.diagonal().template head<GRAVITY_SCALE_SIZE>() +=
                            gsJumpW2;
                        b.template head<GRAVITY_SCALE_SIZE>() +=
                            gsJumpW2.cwiseProduct(gsJump);
                    }

                    MatX gauge_P;
```

In the back substitution block, after the zero velocity model decrease (`:1930-1937`), with `inc` already negated, so that its head is the applied block step $\mathbf{d}$ of §3.4.4 Step 4.

```cpp
                    if (applyGsJump) {
                        const Vec3 dg =
                            inc.template head<GRAVITY_SCALE_SIZE>();
                        l_diff -= gsJumpW2.dot(
                            dg.cwiseProduct(gsJump + Scalar(0.5) * dg));
                    }
```

After the zero velocity term of `after_error_total` (`:2010-2016`),

```cpp
                if (applyZeroVelocity)
                    after_error_total += Scalar(0.5) * zeroVelocityW2 *
                                         frame_states.at(last_state_t_ns)
                                             .getState()
                                             .vel_w_i.squaredNorm();
                const Vec3 gsJumpNew =
                    applyGsJump
                        ? Vec3(gsJump + inc.template head<GRAVITY_SCALE_SIZE>())
                        : gsJump;
                if (applyGsJump)
                    after_error_total +=
                        Scalar(0.5) *
                        gsJumpW2.dot(gsJumpNew.cwiseProduct(gsJumpNew));
```

In the accepted branch, after `lambda_vee = initial_vee;` (`:2088`),

```cpp
                    lambda_vee = initial_vee;
                    gsJump = gsJumpNew;
```

and before the re-centring (`:2130`),

```cpp
        if (mpGsActive) {
            mpGsJumpCost =
                Scalar(0.5) * gsJumpW2.dot(gsJump.cwiseProduct(gsJump));
        }
        if (mpGsActive) RecentreGravityScale();
```

Design rationale. The residual follows the zero velocity term at every site, because both are frame local quadratic residuals that enter $\mathbf{H}$, $\mathbf{b}$ and the three cost terms and are not marginalised. The block occupies offsets 0 to 2 of `aom` in the period, which `optimize` asserts against the prior's order (`:1632-1634`), so the head of `H`, `b` and `inc` is the block. The rejected branch calls `restore()` and does not touch `gsJump`, so the accumulated step is that of the accepted iterations, matching the block value. `inc` is projected before negation (`:1913`), so the applied block step is the projected one, as §3.4.3 Step 5 requires.

Backwards compatibility. With both intensities zero, or with the mode off, `gsJumpW2` is zero and `applyGsJump` false, so no statement changes `H`, `b`, a cost or `l_diff`. `mpGsJumpCost` is then zero and is only logged.

#### Change 7. Four columns of `vio_gravity_scale`, `include/basalt/utils/logger.h` and `src/utils/logger.cpp`

Need. The replays of §5.4 must show the random walk cost, how many inertial factors entered $M_\text{visual}$ in each form, and the largest whitened scale column $|\mathbf{c}|$ among them, from which $\tau$ is tuned. The last column was added after the first smoke replay of §5.5 found the ground phase mostly rejected.

`include/basalt/utils/logger.h:193-195` becomes

```cpp
    void AddVioGravityScale(int64_t tNs, int64_t epoch, double sigmaStep,
                            double tiltStepDeg, double scaleCumulative,
                            double tiltCumulativeDeg, int swapEvent,
                            double jumpCost = 0.0, int visualImuFull = 0,
                            int visualImuRot = 0,
                            double visualImuScaleCol = 0.0);
```

`src/utils/logger.cpp:374-377` becomes

```cpp
    Register("vio_gravity_scale", "[VIO][gravity-scale]",
             {{"t_ns", "ns"}, {"epoch", "count"}, {"sigma_step", "f"},
              {"tilt_step_deg", "f"}, {"scale_cumulative", "f"},
              {"tilt_cumulative_deg", "f"}, {"swap_event", "count"},
              {"jump_cost", "f"}, {"visual_imu_full", "count"},
              {"visual_imu_rot", "count"}, {"visual_imu_scale_col", "f"}});
```

and `Logger::AddVioGravityScale` (`:832-845`) becomes

```cpp
void Logger::AddVioGravityScale(int64_t tNs, int64_t epoch, double sigmaStep,
                                double tiltStepDeg, double scaleCumulative,
                                double tiltCumulativeDeg, int swapEvent,
                                double jumpCost, int visualImuFull,
                                int visualImuRot, double visualImuScaleCol) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_gravity_scale");
    s.add_int("t_ns", tNs);
    s.add_int("epoch", epoch);
    s.add("sigma_step", sigmaStep);
    s.add("tilt_step_deg", tiltStepDeg);
    s.add("scale_cumulative", scaleCumulative);
    s.add("tilt_cumulative_deg", tiltCumulativeDeg);
    s.add("swap_event", double(swapEvent));
    s.add("jump_cost", jumpCost);
    s.add("visual_imu_full", double(visualImuFull));
    s.add("visual_imu_rot", double(visualImuRot));
    s.add("visual_imu_scale_col", visualImuScaleCol);
    s.CommitRow();
}
```

Backwards compatibility. The record is written only in the period, and its only producer is `LogGravityScale`, so the four appended columns change no other record. A search of 2026-11-06 found no script under `scripts/` that reads `vio_gravity_scale`. The new arguments have defaults.

#### Change 8. Unit test of the scale free rows, `test/src/test_vio.cpp`

Need. Change 2 is the only new arithmetic that can be checked in isolation. The test builds one inertial factor on a random spline, as `ImuNullspace2Test` does (`test/src/test_vio.cpp:28-160`), and checks the three properties of §3.3.3 and §3.3.4. Placed after `ImuNullspace3Test`, inside the same `BASALT_INSTANTIATIONS_DOUBLE` guard as the other tests.

```cpp
#ifdef BASALT_INSTANTIATIONS_DOUBLE
TEST(VioTestSuite, ImuBlockGSScaleFreeRowsTest) {
  Eigen::Vector3d bg = Eigen::Vector3d::Random() / 100;
  Eigen::Vector3d ba = Eigen::Vector3d::Random() / 10;

  basalt::IntegratedImuMeasurement<double> imu_meas(0, bg, ba);

  basalt::Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(15);

  Eigen::Vector3d accel_cov, gyro_cov;
  accel_cov.setConstant(accel_std_dev * accel_std_dev);
  gyro_cov.setConstant(gyro_std_dev * gyro_std_dev);

  const int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2; t_ns < int64_t(1e8); t_ns += dt_ns) {
    const Sophus::SE3d pose = gt_spline.pose(t_ns);
    basalt::ImuData<double> data;
    data.accel = pose.so3().inverse() * (gt_spline.transAccelWorld(t_ns) -
                                         basalt::constants::g) +
                 ba;
    data.gyro = gt_spline.rotVelBody(t_ns) + bg;
    data.t_ns = t_ns + dt_ns / 2;
    imu_meas.integrate(data, accel_cov, gyro_cov);
  }

  basalt::PoseVelBiasState<double> state0, state1;
  state0.t_ns = 0;
  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));
  state0.bias_gyro = bg;
  state0.bias_accel = ba;

  state1.t_ns = imu_meas.get_dt_ns();
  state1.T_w_i = gt_spline.pose(state1.t_ns) *
                 Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  state1.vel_w_i =
      gt_spline.transVelWorld(state1.t_ns) + Sophus::Vector3d::Random() / 10;
  state1.bias_gyro = bg;
  state1.bias_accel = ba;

  Eigen::aligned_map<int64_t, basalt::PoseVelBiasStateWithLin<double>>
      frame_states;
  frame_states[state0.t_ns] = basalt::PoseVelBiasStateWithLin<double>(state0);
  frame_states[state1.t_ns] = basalt::PoseVelBiasStateWithLin<double>(state1);

  basalt::AbsOrderMap aom;
  aom.abs_order_map[basalt::kGravityScaleStateId] =
      std::make_pair(0, basalt::GRAVITY_SCALE_SIZE);
  aom.abs_order_map[state0.t_ns] = std::make_pair(3, 15);
  aom.abs_order_map[state1.t_ns] = std::make_pair(18, 15);
  aom.total_size = 33;
  aom.items = 3;

  Eigen::Vector3d weight_sqrt;
  weight_sqrt.setConstant(1e3);
  basalt::ImuLinData<double> ild = {basalt::constants::g,
                                    weight_sqrt,
                                    weight_sqrt,
                                    {std::make_pair(state0.t_ns, &imu_meas)}};

  basalt::GravityScaleStateWithLin<double> gs;

  // Rotation rows only, forced by a zero tolerance.
  basalt::ImuBlockGS<double> ib_rot(&imu_meas, &ild, aom, &gs, true, 0.0);
  const double e_rot = ib_rot.linearizeImu(frame_states);
  EXPECT_FALSE(ib_rot.UsedFullRows());

  const Eigen::Matrix<double, 9, 1> res =
      imu_meas.residual(state0, basalt::constants::g, state1, bg, ba);
  const Eigen::Vector3d r_rot = res.segment<3>(3);
  const Eigen::Matrix3d cov_rot = imu_meas.get_cov().block<3, 3>(3, 3);
  const double e_expected = 0.5 * r_rot.dot(cov_rot.ldlt().solve(r_rot));
  EXPECT_NEAR(e_rot, e_expected, 1e-9 * (1.0 + e_expected));

  basalt::DenseAccumulator<double> acc_rot;
  acc_rot.reset(aom.total_size);
  ib_rot.add_dense_H_b(acc_rot);
  EXPECT_LE(acc_rot.getH().topRows(3).norm(), 1e-12);
  EXPECT_LE(acc_rot.getB().head(3).norm(), 1e-12);
  EXPECT_LE(acc_rot.getH().block(9, 0, 3, 33).norm(), 1e-12);
  EXPECT_LE(acc_rot.getH().block(24, 0, 3, 33).norm(), 1e-12);

  // All rows with the scale column removed, forced by a large tolerance.
  basalt::ImuBlockGS<double> ib_full(&imu_meas, &ild, aom, &gs, true, 1e12);
  ib_full.linearizeImu(frame_states);
  EXPECT_TRUE(ib_full.UsedFullRows());

  basalt::ImuBlockGS<double> ib_ref(&imu_meas, &ild, aom, &gs);
  ib_ref.linearizeImu(frame_states);

  basalt::DenseAccumulator<double> acc_full, acc_ref;
  acc_full.reset(aom.total_size);
  acc_ref.reset(aom.total_size);
  ib_full.add_dense_H_b(acc_full);
  ib_ref.add_dense_H_b(acc_ref);

  Eigen::MatrixXd H_ref = acc_ref.getH();
  Eigen::VectorXd b_ref = acc_ref.getB();
  H_ref.row(2).setZero();
  H_ref.col(2).setZero();
  b_ref[2] = 0;
  EXPECT_LE((acc_full.getH() - H_ref).norm(), 1e-9 * (1.0 + H_ref.norm()));
  EXPECT_LE((acc_full.getB() - b_ref).norm(), 1e-9 * (1.0 + b_ref.norm()));
}
#endif
```

The rotation only factor must have the cost $\frac{1}{2}\mathbf{r}_R^\top \boldsymbol{\Sigma}_{RR}^{-1} \mathbf{r}_R$, since both biases equal their preintegration values and the random walk residuals vanish, no block columns and no velocity rows or columns, the velocities sitting at offsets $3 + 6$ and $18 + 6$. The admitted factor must equal the full factor with the $\sigma$ row and column of $\mathbf{H}$ and the $\sigma$ entry of $\mathbf{b}$ removed, because zeroing one column of $\mathbf{J}$ removes exactly that row and column of $\mathbf{J}^\top \mathbf{J}$ and that entry of $\mathbf{J}^\top \mathbf{r}$. At the default block, $\mathbf{g}_D = \mathbf{g}$ and $s = 1$, so `residualGS` equals `residual` (first plan §4.1 item 2), which the expected cost relies on.

Backwards compatibility. A new test only.

#### Change 9. Documentation and context

1. `doc/VIO.md` §3.3.4, appended to the paragraph on the three priors (`doc/VIO.md:1045`), "With `vio_gs_visual_prior_inertial`, $M_\text{visual}$ also holds every inertial factor in a scale free form, the rotation rows weighted by the marginal covariance of the rotation residual and the bias random walk rows, and all rows with the scale column zeroed for a factor whose whitened scale column is at most `vio_gs_visual_prior_scale_tol`, which admits the stationary phase and unaccelerated flight. $M_\text{visual}$ then keeps the accelerometer bias anchor and the ground phase tilt through every swap, against an anchor of its own for its tilt columns. `plans/gravity_scale_optimisation_2.md` §3.3 derives this."
2. `doc/VIO.md` §3.3.3, appended, "With `vio_gs_jump_sigma_rw` or `vio_gs_jump_tilt_rw_deg` positive, a random walk residual on the block's step within the frame, of standard deviation the intensity times the square root of the frame interval, is added to the system before the projection and to all three cost terms. It shrinks the step in proportion to how little the data determine it (`plans/gravity_scale_optimisation_2.md` §3.4)."
3. `context/dynamic_marginalisation.md`, a section recording that §4.1 and §4.5.1 were implemented as Changes A and B, with the corrected admission of tilt dependent rows and the marginal covariance of the rotation rows.
4. `context/vio_config_json.md`, the four keys, their defaults and the files that carry them.
5. `context/vio_opt.md` §15.11, the random walk residual as a fourth frame local term beside the zero velocity term, and the third anchor.
6. `context/README.md`, the rows of the three files above refreshed.
7. `plans/gravity_scale_optimisation.md` §4.3, a sentence after the solution strategy pointing to this plan for items 1 to 3 and recording that items 4 and 5 were not adopted, with the reasons of §3.6.

#### Change 10 (removed). Seven dimensional free gauge, `src/vi_estimator/sqrt_keypoint_vio.cpp` and the configuration

Status, 2026-11-07. Implemented, tested, and removed completely from the code base and from every configuration file at the user's request, because both replays with it diverged at lift off (Result below). The derivation and the code are kept here as the record of a rejected design, so that it is not proposed again without first resolving the cause of the divergence.

Added 2026-11-06 after the live runs of §5.5 item 5 and the survey of `context/dynamic_marginalisation.md` §6. Need. The position and yaw pins of the initial prior reach later states only through rows that link them to the departing state. With no landmark before lift off and only rotation and bias rows in $M_\text{visual}$, a swap leaves the prior in use without any position pin (`context/dynamic_marginalisation.md` §6.1), and the gauge projector, which excludes the pinned directions, then leaves them to the damping. Treating all seven directions of the problem as free gauge, as DSO does [4], needs no pin and so cannot lose one.

Mathematical background. Step 1, the translation directions. Shifting every frame's position by $\varepsilon\,\mathbf{e}_k$ changes no residual, because the inertial factor contains positions only through $\mathbf{p}_1 - \mathbf{p}_0$ and the visual factor only through $\mathbf{p}_h - \mathbf{p}_t$, and a landmark is stored relative to its host. The increment that reproduces the family is $\delta\mathbf{p}_f = \varepsilon\,\mathbf{e}_k$ for every frame and zero elsewhere, so by the construction of §3.1.3 the gauge vector $\mathbf{n}_{t_k}$ has $\mathbf{e}_k$ in rows $c$ to $c + 2$ of every frame. Step 2, the yaw direction. The tilt family of §3.1.3 rotates the whole world, gravity direction included, by $\text{Exp}(\varepsilon\,\mathbf{a})$, and its derivation holds for any axis $\mathbf{a}$, so $\mathbf{a} = \mathbf{e}_z$ gives $\mathbf{n}_{\mathbf{e}_z}$ with $\bar{\mathbf{B}}^\top \mathbf{e}_z$ in the block rows, $\mathbf{e}_z \times \mathbf{p}_f$, $\mathbf{e}_z$ and $\mathbf{e}_z \times \mathbf{v}_f$ in the frame rows. Step 3, independence. At rest at the origin the seven columns have disjoint supports, the scale entry, the tilt and yaw rotation rows, and the position rows, so $\mathbf{N}^\top \mathbf{N}$ is invertible, and away from the origin the cross terms do not reduce its rank. Step 4, the pins must be removed. A pin of weight $w$ on a coordinate $\mathbf{e}_p$ of the first state enters the projected system as $w\,(\mathbf{P} \mathbf{e}_p)(\mathbf{P} \mathbf{e}_p)^\top$. With the translation vector in $\mathbf{N}$ over $K$ frames, $\mathbf{P} \mathbf{e}_p = \mathbf{e}_p - \mathbf{n}_t / K$, which is the difference between the first frame's position and the mean of all frames, a relative quantity, so the projected pin would constrain the relative motion with weight $w (1 - 1/K)$, about $10^8$. DSO avoids this by projecting the system only after its first frame has left [4] (first plan §2.5.6). Here the pins are removed from the initial prior when the key is set, and the bias weights are kept. Step 5, where the gauge must be fixed again. `NfrMapper::extractNonlinearFactors` rejects a packet whose $\mathbf{H}$ is not of full rank (`nfr_mapper.cpp:161-162`), so `StripGsFromMargData` adds $w$ to the diagonal at the position and yaw coordinates of the oldest keyframe pose in the packet, a pin at the current value that makes the packet full rank and, being a minimal fixation of exactly the four free directions, leaves the covariance of every relative quantity, and so every recovered relative pose and roll pitch factor, unchanged. After the end of the period the projector is off, so `FoldGravityScale` appends four square root rows $\sqrt{w}\,(\boldsymbol{\delta}_i + \mathbf{x}_i)$ with $\mathbf{b}_i = -\sqrt{w}\,\boldsymbol{\delta}_i$ at the oldest frame of the prior, which pins its position and yaw at their current values, as the initial prior does on the default path.

`include/basalt/utils/vio_config.h`, after `vio_gs_jump_tilt_rw_deg`,

```cpp
    // Treat translation and yaw as free gauge directions in the period, by the
    // projector instead of the initial prior's pins,
    // plans/gravity_scale_optimisation_2.md Change 10.
    bool vio_gs_free_gauge;
```

with the default `vio_gs_free_gauge = false;` and `ar(CEREAL_NVP(config.vio_gs_free_gauge));` in `src/utils/vio_config.cpp`, `"config.vio_gs_free_gauge": false` in the six non SITL JSON files and the batch TOML, and `true` in `data/sitl_config_vo.json` as tested. None of these keys remains after the removal.

Constructor, in the `vio_gs_enable` block after `H_gs.rightCols(POSE_VEL_BIAS_SIZE) = marg_data.H;`,

```cpp
        if (config.vio_gs_free_gauge) {
            // Translation and yaw are handled by the gauge projector, which
            // would turn these pins into a constraint on relative motion.
            for (const int i : {0, 1, 2, 5})
                H_gs(i, GRAVITY_SCALE_SIZE + i) = Scalar(0);
        }
        marg_data.H = H_gs;
```

`ComputeGaugeProjector`, the construction of $\mathbf{N}$,

```cpp
    // Columns: scale, tilt about x and y, and with vio_gs_free_gauge also yaw
    // and the three translations.
    const int numRot = config.vio_gs_free_gauge ? 3 : 2;
    const int numCols = 1 + numRot + (config.vio_gs_free_gauge ? 3 : 0);
    MatX N = MatX::Zero(n, numCols);
    N(gs + 2, 0) = Scalar(-1);
    for (int k = 0; k < numRot; k++) {
        N.template block<2, 1>(gs, 1 + k) =
            basis.transpose() * Vec3::Unit(k);
    }
```

and in the frame loop

```cpp
        for (int k = 0; k < numRot; k++) {
            const Vec3 a = Vec3::Unit(k);
            N.template block<3, 1>(c, 1 + k) = a.cross(p);
            N.template block<3, 1>(c + 3, 1 + k) = a;
            if (has_vel) N.template block<3, 1>(c + 6, 1 + k) = a.cross(v);
        }

        if (config.vio_gs_free_gauge) {
            for (int k = 0; k < 3; k++) N(c + k, 1 + numRot + k) = Scalar(1);
        }
    }

    const MatX NtN = N.transpose() * N;
    return MatX::Identity(n, n) - N * NtN.ldlt().solve(N.transpose());
```

`StripGsFromMargData`, after the order map is rebuilt,

```cpp
    if (config.vio_gs_free_gauge) {
        // The mapper needs a full rank packet, so the four free gauge
        // directions are pinned at the current value on the oldest keyframe
        // pose, or the oldest frame if the packet holds no keyframe pose.
        int pin = -1;
        for (const auto& [id, idx] : m.aom.abs_order_map) {
            if (idx.second == POSE_SIZE) {
                pin = idx.first;
                break;
            }
        }
        if (pin < 0) pin = m.aom.abs_order_map.begin()->second.first;
        for (const int i : {0, 1, 2, 5})
            m.abs_H(pin + i, pin + i) += config.vio_init_pose_weight;
    }
```

`FoldGravityScale`, after `marg_data.order = order;`,

```cpp
    if (config.vio_gs_free_gauge) {
        // Without the projector after the period, translation and yaw are
        // pinned again at their current values on the oldest frame of the
        // prior, as the initial prior pins them in the default path.
        VecX delta;
        computeDelta(marg_data.order, delta);
        const int pin = marg_data.order.abs_order_map.begin()->second.first;
        const Scalar w = std::sqrt(Scalar(config.vio_init_pose_weight));
        const int rows = int(marg_data.H.rows());
        marg_data.H.conservativeResize(rows + 4, Eigen::NoChange);
        marg_data.b.conservativeResize(rows + 4);
        marg_data.H.bottomRows(4).setZero();
        int r = rows;
        for (const int i : {0, 1, 2, 5}) {
            marg_data.H(r, pin + i) = w;
            marg_data.b(r) = -w * delta(pin + i);
            r++;
        }
    }
```

Backwards compatibility. With the key false, the pins stay, $\mathbf{N}$ has the same three columns, the packet and the fold are untouched, and the only difference is that $\mathbf{N}^\top \mathbf{N}$ is now factorised as a dynamic rather than a fixed $3 \times 3$ matrix. Mode off never reaches any of these sites.

Result, 2026-11-07. Change 10 was removed, because two replays of `telemetry_40` with it on diverged at lift off (`/ws/logs/slam/2026-11-07_00-03-18` and `_00-06-45`, scale ratios of 485 and 146 at 58 s, 263 and 280 swaps, cumulative block scale collapsing to zero), while the replay with the same binary and the key off (`_00-06-42`) logged 25 swaps, a scale ratio of 1.9 and a tilt error of 1.5° to 3.6° from 70 s. The cause is not identified. The leading hypothesis is the conflict between a drifting estimator gauge and the local mapper's pose updates, which overwrite frozen keyframe linearisation points in the mapper's own world (528 and 349 updates applied against 239 to 314 with the key off). A replay with the free gauge on and the mapper's pose updates disabled would test it.

Expected side effects, to be checked in the replays. The free gauge selects the minimum norm step, so the published absolute position and yaw are no longer tied to the first frame and drift by the mean of each step's gauge component, as in DSO. The relative trajectory and the roll and pitch are unaffected. The local mapper's pose updates are compared with the estimator's poses (`sqrt_keypoint_vio.cpp:619-660`), so a drift of the estimator's gauge relative to the mapper's world shows as larger pose update errors and more rejections in `vio_pose_update.csv`.

### 4.3 Blast radius

| Symbol | Consumers | Effect outside the new configuration |
|---|---|---|
| `VioConfig` four keys | eight configuration files, `SqrtKeypointVioEstimator` | defaults reproduce the present behaviour, keys must exist in every file |
| `ImuBlockGS` constructor arguments | `LinearizationGSAbsQR`, the new test | defaults skip the new branch |
| `ImuBlockGS::UsedFullRows` | `LinearizationGSAbsQR::CountImuRowForms` | read only |
| `LinearizationGSAbsQR` constructor arguments | `optimize`, `LinearizeMargGS` | defaults reproduce full rows |
| `LinearizationGSAbsQR::CountImuRowForms` | `LinearizeMargGS` | read only |
| `SqrtKeypointVioEstimator::ResetHalfFromVisual` | `UpdateScaleInterval` | flag off reproduces the cleared anchor |
| `mpAnchorVisual` | `LinearizeMargGS`, `RecentrePriorsGS`, `RecentreGravityScale`, `FoldGravityScale`, `ResetHalfFromVisual` | never set with the flag off |
| random walk residual in `optimize` | `optimize` only | zero weights skip every site |
| `Logger::AddVioGravityScale` arguments and columns | `LogGravityScale` | four columns appended to a period only record |

`ImuBlock`, `LinearizationAbsQR`, `LinearizationAbsSC`, `LinearizationRelSC`, the local mapper, `NfrMapper`, the VO estimator and the controller are not touched.

## 5. Validation

### 5.1 Code reading, before any run

1. With `vio_gs_enable = false`, trace every new statement to a branch on `mpGsActive`, on `applyGsJump` or on `vio_gs_visual_prior_inertial`, and confirm that none executes.
2. With the mode on and the new keys at their defaults, confirm that `LinearizeMargGS` passes `nullptr` inertial data and `imu_scale_free = false` for $M_\text{visual}$, that `visual_anchor` refers to the copy of $M_\text{curr}$'s anchor, that `ResetHalfFromVisual` clears the anchor, and that `applyGsJump` is false.
3. Confirm that `ImuBlockGS::linearizeImu` in scale free mode tests `mpJgs.col(2)` before zeroing it, uses rows 3 to 5 of the unwhitened Jacobians and residual for the rotation only form, and leaves rows 9 to 14 to `LinearizeBiasRandomWalk`.
4. Confirm that every use of $M_\text{visual}$'s anchor in `LinearizeMargGS` and `RecentrePriorsGS` refers to `mpAnchorVisual` when the flag is set, and that `RecentreGravityScale` re-expresses it.
5. Confirm that the random walk enters `error_total`, `H` and `b` before the projection, `l_diff` with the negated increment, and `after_error_total`, and that `gsJump` advances only in the accepted branch.

### 5.2 Compilation in the dev container

Every touched translation unit is syntax checked with the recipe of `/ws/skills/cpp-syntax-check-tooling.md`, namely `src/vi_estimator/sqrt_keypoint_vio.cpp`, `src/linearization/linearization_abs_qr.cpp`, `src/utils/vio_config.cpp` and `src/utils/logger.cpp`, and `test/src/test_vio.cpp` by reading, since it is not in the compile database. Then `MAKEFLAGS=-j4 colcon build --packages-select slam`, with `-j4` because a full parallel build was killed by the out of memory handler on 2026-11-06.

### 5.3 Unit tests, run by the user

1. `VioTestSuite.ImuBlockGSScaleFreeRowsTest` of Change 8.
2. The existing `VioTestSuite` and `ImuPreintegrationTestCase` suites, unchanged, which exercise `ImuBlock` and the preintegration.
3. The outstanding tests of the first plan §4.2, of which `GaugeNullspaceGSTest` should be extended to check that an $M_\text{visual}$ linearised with rotation only rows annihilates all three gauge vectors, and that one with admitted full rows annihilates the two tilt vectors exactly and $\mathbf{n}_\sigma$ up to the zeroed column, $|\mathbf{J}' \mathbf{n}_\sigma| = |\mathbf{c}|$ (§3.3.4 Step 4).

### 5.4 Replays and flights, run by the user

All replays use an isolated `ROS_DOMAIN_ID`, the decoder pointed away from the camera topic, and starts staggered by more than one second so that the logger's directories do not collide (`/ws/context/djinn-workflows.md`, `djinn start basalt-replay`). The bag is `/ws/data/telemetry_40`, and each configuration runs at least three times because the local mapper makes runs non deterministic.

| Configuration | `vio_gs_visual_prior_inertial` | `vio_gs_jump_sigma_rw` | `vio_gs_jump_tilt_rw_deg` |
|---|---|---|---|
| R0, mode off | n/a | n/a | n/a |
| R1, mode on as today | false | 0 | 0 |
| R2, Change A | true | 0 | 0 |
| R3, Change B | false | 0.1 | 1.0 |
| R4, Changes A and B | true | 0.1 | 1.0 |

Measures, from `gt_eval.csv`, `vio_state.csv` and `vio_gravity_scale.csv`, are the tilt error and the scale ratio in the windows of the first plan's record, the horizontal accelerometer bias against $|\mathbf{g}| \sin(\text{tilt error})$, the number of swap events per second during and after take off, the per frame $\sigma$ and tilt steps, the counts `visual_imu_full` and `visual_imu_rot` in the ground phase and in the climb, and `jump_cost`. The expectations derived above are the following.

1. R2 against R1. The bias stays within its prior after swaps, and the tilt error stays near the mode off level (§3.3.6). Swaps may still be frequent at take off.
2. R3 against R1. Swap events at take off fall from almost every frame to a few, and the per frame $\sigma$ step stays mostly below 0.048 (§3.4.5). The tilt may still drift, since the anchor is still lost at the swaps that remain.
3. R4. Both, with a scale ratio within 15 % of 1 after the climb as in the first plan's 10 s run, and a tilt error no worse than R0.
4. `visual_imu_full` dominates in the ground phase and `visual_imu_rot` in the take off transient (§3.3.4 Step 3). If the ground phase is mostly rotation only, raise `vio_gs_visual_prior_scale_tol` to 2 and repeat.

Sweeps after R4, each on three runs, are `vio_gs_jump_sigma_rw` in $\{0.05, 0.2\}$, `vio_gs_jump_tilt_rw_deg` in $\{0.5, 2\}$, and `vio_init_bg_weight`, which reaches the accelerometer bias, in $\{10^2, 10^4\}$, the second a standard deviation of 0.01 m/s². The last is the experiment the first plan's record intended with `vio_init_ba_weight`. Timing is read from `marginalize`, which should rise by one inertial block per marginalisation only, and from `optimize`, which should not change measurably.

### 5.5 Record of smoke runs, 2026-11-06

After implementation, `MAKEFLAGS=-j4 colcon build --packages-select slam` succeeded with no error, and two replays of `/ws/data/telemetry_40` ran with `data/sitl_config_vo.json` as committed by Change 1 (both changes on, $\tau = 1$, intensities 0.1 and 1.0, `vio_gs_duration_s` 7000), on isolated domains 44 and 45 with a wall clock timeout of 150 s, which covers bag time 50.6 s to 96.6 s. The log directories are `/ws/logs/slam/2026-11-06_23-00-46` (run 1, before the `visual_imu_scale_col` column existed) and `/ws/logs/slam/2026-11-06_23-07-49` (run 2). Neither run asserted, and the backtrace at the end of each launch log is gdb reporting the threads after the timeout's SIGINT, with every process finishing cleanly. The measures are those of the first plan's record.

| Window | Run 1 scale ratio | Run 1 tilt error | Run 2 scale ratio | Run 2 tilt error |
|---|---|---|---|---|
| 60 s, take off transient | 5.89 | 17.3° | 6.99 | 18.4° |
| 70 s | 1.10 | 3.9° | 3.47 | 0.6° |
| 80 s | 1.10 | 0.9° | 1.94 | 1.3° |
| 88 s | 1.10 | 1.2° | 3.08 | 0.5° |
| 94 s | 1.14 | 1.0° | 3.04 | 0.9° |

1. The tilt divergence is gone in both replays of `telemetry_40`, a flight that hops to 1 m, hovers for about 6 s and only then climbs. It is not gone on the straight climbs of the live `sitl2` missions (item 5). From 70 s the tilt error of the replays stays between 0.5° and 1.3°, against 26° to 27° for the first plan's 10 s period, 74° to 113° for its 70 s period, and 2.4° to 6° with the mode off. The horizontal accelerometer bias of run 2 is 0.01 to 0.08 m/s² from 70 s, against 1 to 4 m/s² before, so the anchor now survives the swaps as §3.3.6 predicted.
2. The scale is not deterministic. Run 1 ends within 14 % of ground truth, run 2 near 3, which is no better than the mode off runs (2.1 to 2.7). In both runs the block stops moving after about 70 s, with per frame steps below $10^{-3}$, because the cruise carries almost no scale information (§3.3.4) and the random walk residual then holds the block. The scale is therefore decided in the take off transient.
3. Change B did not stop the swaps during the transient. Run 2 logged 62 swap events, mostly between 55 s and 70 s, and steps of up to $|\delta\sigma| = 1.04$ in run 1, at a random walk cost of 1016. On those frames the data pull harder than the residual, which is the case §3.4.2 says it must let through, so the steps are data driven rather than uninformed. Each swap still discards the scale dependent rows of the accelerating factors (§3.3.6 Step 4), and which way the transient settles then varies between runs. This is the loss that delayed marginalisation of the inertial chain (§3.6 item 1) would remove, and it is the recommended next step.
4. The admission test of §3.3.4 rejects almost every factor of the ground phase at $\tau = 1$, with 5 of 84 factors admitted in full between 50 s and 55 s in run 1. The logged $|\mathbf{c}|$ in run 2 has the quantiles 2.1, 5.7 and 13.9 (10 %, 50 %, 90 %) on the ground, 3.3, 12.9 and 193 in the transient, and 11.7, 34.6 and 142 in the cruise. On the ground the column is dominated by the position row, through millimetre jitter of the estimated positions against $\sigma_{\Delta p} \approx 0.5$ mm, not by acceleration, since the logged velocities give $|\mathbf{a}|\sqrt{\Delta t}/\sigma_c$ below $10^{-3}$. The distributions overlap, so no $\tau$ admits the ground phase without also admitting accelerating factors, and $\tau = 1$ is kept. The tilt and bias results of item 1 were obtained with rotation rows and the bias random walk alone.
5. Live `sitl2` flights with both changes on diverge from lift off, in three of three runs (`/ws/logs/slam/2026-11-06_23-14-47`, `_23-18-44`, `_23-21-18`), with scale ratios of 100 to 1300 and tilt errors of 30° to 79° within 20 s. `_23-14-47` and `_23-18-44` hop to 1 m like the replay and then climb with 3 m to 5 m of horizontal motion, reaching 2.5 m at 4 s, and `_23-21-18` climbs vertically to 10 m in 4 s with up to 4.5 m/s² of vertical acceleration, each after only 4 to 5 s on the ground. The replays above exercised neither profile. Note, 2026-11-06. This sentence previously described all three as straight climbs from the ground, which the ground truth trajectories contradict for the first two. The live divergence predates this revision. Of the seven live runs of 2026-11-06 with the period effectively permanent (`vio_gs_duration_s` 7000) and without Changes A and B, four diverged in the same way (`_17-23-34`, `_17-44-45`, `_17-56-52`, `_18-25-25`), while the live runs with the mode off kept scale ratios of 2 to 4 and did not diverge. In `_23-14-47` the scale runs away. A cascade of swaps from lift off is followed by per frame steps of $+0.02$ to $+0.17$ in $\sigma$, all of one sign, so the cumulative scale grows from 1 to 3.7 in 2 s and to 223 in 8 s while each single step stays within what the random walk residual allows. In `_23-18-44` the tilt runs away with no swap at all. Steps of 0.5° to 1.5° per frame, all in one direction, accumulate to 43° in 4 s, after the estimate failed to follow the climb (estimated height near 0 m while the truth reached 1.9 m at 131.7 s). The horizontal bias of 2.3 m/s² is far from $|\mathbf{g}| \sin(70°)$, so this is not the tilt and bias trade of §3.2.3. Both runaways have the same structure. The random walk residual bounds each step but not their sum (§3.4.2, `context/dynamic_marginalisation.md` §4.5.2), and nothing in the estimator penalises the cumulative correction held in the correction log. With a permanent period, any persistent inconsistency between the inertial and visual data at take off is absorbed by the block one bounded step at a time, without limit. Note, 2026-11-06. Item 1 previously stated that the tilt divergence was gone without qualification. That conclusion rested on two replays of a hop and hover profile and does not hold for the live straight climbs.
6. A defect of Change 5 as first implemented amplified two of the live runs. $M_\text{visual}$'s anchor kept its scale as a first estimate and every re-centring shifted it by the frame's correction, so its linearisation scale followed the inverse of the cumulative correction. Relative to the current scale it fell to 0.018 at 4 s and 0.0044 at 8 s after lift off in `_23-14-47`, and rose to 9.5 at 2 s in `_23-21-18`, against 0.44 in the replay and 0.94 in `_23-18-44`. With swaps copying $M_\text{visual}$ into $M_\text{half}$ and $M_\text{curr}$ almost every frame, the visual translation information of the prior in use shrank with the scale, which closes a positive feedback loop on the scale in `_23-14-47`. The anchor's scale now follows the current value (§3.3.5). The defect does not explain the tilt runaway of `_23-18-44`, nor the divergence of the permanent period before this revision. The trigger common to all live runs precedes any block motion. In `_23-21-18`, in the second after lift off and before any block step exceeded 0.01, the whitened scale columns of the departing inertial factors were 100 to 800, against 2 to 15 on the ground, so the estimated states implied accelerations far beyond the 4.5 m/s² of the climb. The window's states are kinematically inconsistent with the preintegration at lift off, before the block reacts, and the block then absorbs the inconsistency. The live runs also show that horizontal excitation does not prevent the divergence, since `_23-14-47` and `_23-18-44` add 3 m to 5 m of horizontal motion to the hop of the replay profile and `_23-21-18` is a vertical climb to 10 m in 4 s, and all three diverge.
7. With the anchor fix, a replay with the three dimensional gauge (`/ws/logs/slam/2026-11-07_00-06-42`) logged 25 swap events against 62 and 74 before it, a scale ratio of 1.85 to 1.97 and a tilt error of 1.5° to 3.6° from 70 s. Two replays with the seven dimensional gauge of Change 10 diverged at lift off, so Change 10 was removed from the code base (Change 10, Result). Each configuration has one or two runs, so these are indications, not distributions.

## 6. Conclusion

The first plan's gravity and scale period corrected the scale but let the tilt diverge. The derivations of §3.2 trace the divergence to the fallback prior of VI-DSO's dynamic marginalisation, which in Basalt held no tilt and no accelerometer bias anchor, so every swap freed the exact tilt and bias null direction of the inertial factors at constant attitude. Large per frame steps at take off made swaps fire almost every frame, and each swap discarded what information had accumulated.

Change A widens the fallback prior to every row that is free of the scale. These are the rotation rows weighted by their marginal covariance, the bias random walk, and the full rows of every factor without acceleration with the scale column zeroed, the last admitted by a test that bounds the error in standard deviations. By the monotonicity of the Schur complement, no prior after any swap holds less information than before, and the bias anchor and the ground phase tilt survive every swap. Change B adds the process model of a scale filter as a random walk residual on the per frame step, before the gauge projection and in all three cost terms. It shrinks the step in proportion to how little the data determine it, which gates the correction softly where the block's own information is identically zero, and keeps uninformed steps below the swap interval. Both changes are opt in, with defaults that reproduce the present behaviour exactly. Delayed marginalisation of the inertial chain remains the next step if the replays show that the accelerating factors still lost at swaps limit the scale.

The admission of tilt dependent but scale free inertial rows into VI-DSO's visual prior, the rotation only factor with the marginal covariance as the scale free remainder of an accelerating factor, and the analysis of a random walk residual under a projected free gauge were not found in the papers read. Whether they are new would need a wider search, of the works citing VI-DSO and DM-VIO in particular, before it is claimed.

## References

[1] L. von Stumberg, V. Usenko, D. Cremers, "Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization", IEEE ICRA 2018, arXiv:1804.05625. §III-E, §III-F, Algorithm 1 and eq. 14 to 17 read on 2026-11-04 and 2026-11-06 at https://ar5iv.labs.arxiv.org/html/1804.05625.

[2] L. von Stumberg, D. Cremers, "DM-VIO: Delayed Marginalization Visual-Inertial Odometry", IEEE Robotics and Automation Letters, 2022, arXiv:2201.04114. §III-B to §III-F and §IV-A read on 2026-11-06 at https://ar5iv.labs.arxiv.org/html/2201.04114, including the initialiser's thresholds on the marginal covariance of the scale.

[3] V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers, "Visual-Inertial Mapping with Non-Linear Factor Recovery", IEEE Robotics and Automation Letters, 2020, arXiv:1904.06504. §V-B and §V-C, eq. 27 to 35, read on 2026-11-06.

[4] J. Engel, V. Koltun, D. Cremers, "Direct Sparse Odometry", IEEE TPAMI, 2018. Cited from memory. The null space projection was read in DM-VIO's copy, `src/dso/OptimizationBackend/EnergyFunctional.cpp:720-829`, for the first plan.

[5] Z. Zhang, G. Gallego, D. Scaramuzza, "On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation", IEEE Robotics and Automation Letters, vol. 3, no. 3, pp. 2710 to 2717, 2018, DOI 10.1109/LRA.2018.2833152. Abstract and the description of the authors' repository `uzh-rpg/rpg_vi_cov_transformation` read on 2026-11-06.

[6] J. Hsiung, M. Hsiao, E. Westman, R. Valencia, M. Kaess, "Information Sparsification in Visual-Inertial Odometry", IEEE/RSJ IROS 2018. Abstract only.

[7] G. Nützi, S. Weiss, D. Scaramuzza, R. Siegwart, "Fusion of IMU and Vision for Absolute Scale Estimation in Monocular SLAM", Journal of Intelligent and Robotic Systems, vol. 61, 2011. Abstract and citing statements only.

[8] C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, J. D. Tardós, "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM", IEEE Transactions on Robotics, 2021, arXiv:2007.11898. Search snippets only, and the implementation read for the first plan.

[9] M. Mazuran, W. Burgard, G. D. Tipaldi, "Nonlinear Factor Recovery for Long-Term SLAM", The International Journal of Robotics Research, vol. 35, 2016. Abstract only.

[10] N. Demmel, D. Schubert, C. Sommer, D. Cremers, V. Usenko, "Square Root Marginalization for Sliding-Window Bundle Adjustment", IEEE/CVF ICCV 2021, pp. 13260 to 13268, arXiv:2109.02182. Abstract read on 2026-11-06.

[11] G. P. Huang, A. I. Mourikis, S. I. Roumeliotis, "Analysis and Improvement of the Consistency of Extended Kalman Filter based SLAM", IEEE ICRA 2008, and "Observability-based Rules for Designing Consistent EKF SLAM Estimators", The International Journal of Robotics Research, 2010. Abstracts and search summaries read on 2026-11-06.

[12] C. Campos, J. M. M. Montiel, J. D. Tardós, "Inertial-Only Optimization for Visual-Inertial Initialization", IEEE ICRA 2020, arXiv:2003.05766. Read on 2026-11-06 at https://ar5iv.labs.arxiv.org/html/2003.05766, including eq. 2, 5, 10 and 11 and the accelerometer bias prior.

[13] T. Qin, P. Li, S. Shen, "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator", IEEE Transactions on Robotics, 2018, arXiv:1708.03852. §I, §V-B and §VI-D read on 2026-11-06 at https://ar5iv.labs.arxiv.org/html/1708.03852.

[14] K. J. Wu, S. I. Roumeliotis, "Unobservable Directions of VINS Under Special Motions", MARS Lab technical report, University of Minnesota, 2016, https://mars.cs.umn.edu/papers/tech_report_unobs_directs.pdf. Search snippet only, stating that the scale is unobservable under constant local acceleration.

[15] Y. Yang, P. Geneva, K. Eckenhoff, G. Huang, "Degenerate Motion Analysis for Aided INS With Online Spatial and Temporal Sensor Calibration", IEEE Robotics and Automation Letters, 2019. Abstract and table of contents only.

[16] J. Nocedal, S. J. Wright, "Numerical Optimization", 2nd ed., Springer, 2006, chapters 4 and 10 on trust regions and Levenberg–Marquardt. Cited from memory.

[17] `plans/gravity_scale_optimisation.md`, the first plan, and `context/dynamic_marginalisation.md`, from which §2 and §3.1 restate results.

[18] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", IEEE Transactions on Robotics, 2017, arXiv:1512.02363. Eq. 33, 34 and 38 read on 2026-11-06.
