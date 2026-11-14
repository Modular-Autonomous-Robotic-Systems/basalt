# Gravity Direction and Scale Optimisation in the VIO Window

Status, 2026-11-06. Approved and implemented, Changes 1 to 17. The unit tests of §4.2 are outstanding, and the first replays of §4.3 were run on 2026-11-06 and found the scale estimate working and the tilt estimate diverging (§4.3, record of runs). Note, 2026-11-05. This line previously read that the plan was awaiting review and that nothing had been implemented. Note, 2026-11-06. It then read that the replays of §4.3 were outstanding. Every `file:line` anchor is against the working tree of branch `shreyas/kf_driven_mapping_and_logger` on top of `7e91ec4`, and every statement about present behaviour is a reading of source. The mathematical background this plan extends is `doc/VIO.md` §2.2 and §3.1 and `context/vio_opt.md`, whose notation is reused.

Revision, 2026-11-04. The plan previously kept the estimator's states in a scale and tilt free world for the whole initialisation period, mapped them to metric only when publishing, folded the estimated similarity into the state once at the end of the period, and suspended the local mapper for the period. It now re-centres the gravity and scale block at the end of every `optimize` call, so the stored states, the landmarks and every prior are metric and gravity aligned at all times, and the block carries only the correction estimated within the current frame (§2.7). The local mapper runs from the first keyframe and is kept consistent through an epoch stamped correction log (§2.9). Three consequences follow. The gravity direction is now parameterised on the unit sphere with a transported tangent basis, because the earlier left rotation increment could not be re-centred exactly (§2.1.3). The fold at the end of the period reduces to conditioning the prior on the block (§2.8). `predictStateGS`, `MapToMetric`, `GetCurrentPointsMetric` and the scale argument of `computeProjections` are no longer needed and were removed from the plan.

Revision, 2026-11-05. Sections 1 to 2.3 were rewritten for review feedback without changing any result. All mathematics is now written in LaTeX, including inline references to variables. The identities every derivation relies on are collected and labelled at the head of §2, and each derivation now cites them step by step. The pose perturbation of §2.2.3 now follows `doc/VIO.md` §2.2.1, a right perturbation $\mathbf{T}_{w,i}\,\text{Exp}(\boldsymbol{\delta})$, and derives its relation to Basalt's state increment instead of introducing it as $\boldsymbol{\zeta}_h = \mathbf{B}_h\boldsymbol{\delta}_h$. The symbols $\boldsymbol{\tau}$, $\mathbf{A}_s$, $\mathbf{B}_h$ and $\mathbf{R}_{c_t i}$ were removed in favour of the expressions they abbreviated, the relative translation is now written $\mathbf{t}_{ti,hi}$, and $\mathbf{D}_s$ is now defined and derived where it first appears (§2.2.2). A second pass on the same day split every display line that held several equations into one equation per line, replaced `\widehat` by `\hat` because the `latex2text` converter used by the editor's Markdown renderer drops `\widehat`, added the identities (I9) and (I10), and rewrote §2.3 so that every Jacobian is derived by perturbing one variable, separating the unperturbed residual, and identifying the matrix by (I9).

## 1. Introduction

Basalt fixes the direction of gravity once, from the first accelerometer sample, by `FromTwoVectors(accel, UnitZ())` (`src/vi_estimator/sqrt_keypoint_vio.cpp:248`, `doc/VIO.md` §3.2.2), and it never represents metric scale as a variable at all. Scale and the two tilt angles are instead carried implicitly by every position, every velocity and every landmark inverse distance in the window, and by the marginalisation prior that summarises the frames already removed. This is sound when the first seconds are well excited. It fails on a vertical take off, which is the regime of every SITL flight recorded in `context/takeoff_bias_divergence.md`. A straight climb at near constant velocity leaves the accelerometer bias along the direction of travel unobservable (§2 there), the take off transient corrupts the first landmark depths (§9), the estimator then settles into a self consistent but scaled world explained by a biased accelerometer (§1), a tilt error of 10 to 17° is absorbed by a horizontal bias (§10), and later excitation repairs the bias but not the scale, because the scale has been frozen into the velocities and depths held by the marginalisation prior (§8, root cause R5 of `plans/vio_drift_analysis.md`).

ORB-SLAM3 [1] avoids this by an explicit inertial only optimisation of the gravity direction $\mathbf{R}_{wg}$ and the scale $s$ over a batch of keyframes whose poses are held fixed (`Optimizer::InertialOptimization`, `/ws/ros_ws/src/slam/orb_slam3/src/Optimizer.cc:3044-3226`, with the residual `EdgeInertialGS`, `G2oTypes.cc:617-718`), after which the whole map is transformed by the estimated similarity (`LocalMapping.cc:1295-1298`). VI-DSO [2] and its successor DM-VIO [3] instead keep scale and gravity direction as permanent variables of the sliding window optimisation, express the visual geometry in a scale free frame and the inertial factors in the metric frame, and introduce dynamic marginalisation so that the prior never freezes information linearised at a scale far from the current estimate.

This plan adds to Basalt a gravity and scale mode for the first seconds of operation, called the initialisation period. During that period a three dimensional global block $[\delta\theta_1, \delta\theta_2, \delta\sigma]^\top$, two tilt angles and a log scale, is optimised jointly with every pose, velocity, bias and landmark, and it is coupled tightly into both the visual and the inertial residuals. At the end of every frame's optimisation the estimated correction is applied to the whole estimator, which keeps the stored states and the map metric and gravity aligned at all times, and the same correction is replayed onto the local mapper's map. The marginalisation follows the three prior scheme of VI-DSO [2] adapted to Basalt's square root marginalisation. At the end of the period the block is removed from the prior and the estimator continues exactly as it does today. With the mode disabled, which is the default, no line of the existing execution path changes behaviour.

Four findings shaped the design and are stated here because each one contradicts an assumption a reader might bring.

1. Adding $s$ and $\mathbf{R}_{wg}$ to a joint visual inertial problem adds three gauge freedoms, directions along which the cost is exactly flat. Rescaling every position and velocity by $k$ while dividing $s$ by $k$ leaves every residual unchanged, and so does rotating the whole estimator world about a horizontal axis while rotating $\mathbf{R}_{wg}$ with it. Without an explicit gauge treatment the new variables carry no information and the converged metric trajectory equals today's. Section 2.5 derives the null vectors and §3.2 removes them by projection, as DSO [4] does for its own gauge.
2. The new variables are useful only together with a marginalisation that does not freeze a scale linearised at the wrong value. Section 2.6 derives the three prior scheme that achieves this.
3. Gravity direction does not enter the visual residual, and scale enters it only through the IMU to IMU relative translation, so the reprojection function `linearizePoint` itself is unchanged and no `linearizePointGS` is needed. Section 2.2 derives this, and Change 2 records the decision.
4. Re-centring the block every frame is an exact change of variables only if the prior keeps the block's columns and is re-expressed, not conditioned. Conditioning at every frame would hand the prior's scale information to the frame positions, where the gauge projection of the next solve deletes it. Section 2.7 derives the exact re-centring.

## 2. Mathematical Formulation

The derivations of this section use the following identities, cited by label. $[\mathbf{a}]_\times$ denotes the skew symmetric matrix with $[\mathbf{a}]_\times \mathbf{b} = \mathbf{a}\times \mathbf{b}$, built by `SO3::hat` (`thirdparty/basalt-headers/thirdparty/Sophus/sophus/so3.hpp:783-791`). A pose is written as a pair $\mathbf{T} = (\mathbf{R}, \mathbf{t})$ of a rotation and a translation, and a tangent vector of $SE(3)$ as $[\boldsymbol{\upsilon}; \boldsymbol{\omega}]$, translation first, which is the ordering of Sophus and of Basalt.

(I1) First order exponential of $SO(3)$. $\text{Exp}(\boldsymbol{\omega}) = \sum_{n\ge 0}\frac{1}{n!}[\boldsymbol{\omega}]_\times^n = \mathbf{I} + [\boldsymbol{\omega}]_\times + O(|\boldsymbol{\omega}|^2)$, the series that `SO3::exp` (`so3.hpp:685`) evaluates in closed form, used in the same role by `doc/VIO.md` §3.1.2, Preliminary 1.

(I2) Antisymmetry of the cross product. $[\mathbf{a}]_\times \mathbf{b} = \mathbf{a}\times \mathbf{b} = -\mathbf{b}\times \mathbf{a} = -[\mathbf{b}]_\times \mathbf{a}$.

(I3) Linearity of the hat map. $[\lambda\mathbf{a}]_\times = \lambda[\mathbf{a}]_\times$ for every scalar $\lambda$, read directly from the entries of `SO3::hat`.

(I4) Conjugation by a rotation. A rotation preserves the cross product, $\mathbf{R}(\mathbf{a}\times \mathbf{b}) = (\mathbf{R} \mathbf{a})\times(\mathbf{R} \mathbf{b})$. Setting $\mathbf{b} = \mathbf{R}^\top \mathbf{x}$ gives $\mathbf{R}[\mathbf{a}]_\times \mathbf{R}^\top \mathbf{x} = [\mathbf{R} \mathbf{a}]_\times \mathbf{x}$ for every $\mathbf{x}$, hence $[\mathbf{R} \mathbf{a}]_\times = \mathbf{R}[\mathbf{a}]_\times \mathbf{R}^\top$, which is `doc/VIO.md` §3.1.2, Preliminary 2. Raising both sides to the $n$-th power gives $[\mathbf{R} \mathbf{a}]_\times^n = \mathbf{R}[\mathbf{a}]_\times^n\mathbf{R}^\top$, because the inner factors $\mathbf{R}^\top \mathbf{R}$ cancel, and summing the series of (I1) term by term gives $\text{Exp}(\mathbf{R} \mathbf{a}) = \mathbf{R}\,\text{Exp}(\mathbf{a})\,\mathbf{R}^\top$.

(I5) Composition and inverse in $SE(3)$. $(\mathbf{R}_1, \mathbf{t}_1)(\mathbf{R}_2, \mathbf{t}_2) = (\mathbf{R}_1 \mathbf{R}_2,\ \mathbf{R}_1 \mathbf{t}_2 + \mathbf{t}_1)$ and $(\mathbf{R}, \mathbf{t})^{-1} = (\mathbf{R}^\top,\ -\mathbf{R}^\top \mathbf{t})$, which follow from the $4\times 4$ homogeneous matrix form of `doc/VIO.md` §2.2.4. The inverse of a product is $(\mathbf{T}_1 \mathbf{T}_2)^{-1} = \mathbf{T}_2^{-1} \mathbf{T}_1^{-1}$.

(I6) Exponential of $SE(3)$. $\text{Exp}([\boldsymbol{\upsilon}; \boldsymbol{\omega}]) = \big(\text{Exp}(\boldsymbol{\omega}),\ \mathbf{J}_l(\boldsymbol{\omega})\,\boldsymbol{\upsilon}\big)$, where $\mathbf{J}_l$ is the left Jacobian of $SO(3)$, as `SE3::exp` computes it (`thirdparty/basalt-headers/thirdparty/Sophus/sophus/se3.hpp:852-861`). Since $\mathbf{J}_l(\boldsymbol{\omega}) = \mathbf{I} + O(|\boldsymbol{\omega}|)$ and $\mathbf{J}_l(\mathbf{0}) = \mathbf{I}$, two consequences follow, $\text{Exp}([\boldsymbol{\upsilon}; \boldsymbol{\omega}]) = \big(\text{Exp}(\boldsymbol{\omega}),\ \boldsymbol{\upsilon} + O(|[\boldsymbol{\upsilon}; \boldsymbol{\omega}]|^2)\big)$ to first order, and $\text{Exp}([\boldsymbol{\upsilon}; \mathbf{0}]) = (\mathbf{I}, \boldsymbol{\upsilon})$ exactly.

(I7) The Adjoint identity. $\mathbf{T}\,\text{Exp}(\boldsymbol{\zeta}) = \text{Exp}\big(\text{Adj}(\mathbf{T})\,\boldsymbol{\zeta}\big)\,\mathbf{T}$, labelled $(\star)$ in `doc/VIO.md` §2.2.1, with
$$\text{Adj}\big((\mathbf{R}, \mathbf{t})\big) = \begin{bmatrix} \mathbf{R} & [\mathbf{t}]_\times \mathbf{R}\\ \mathbf{0} & \mathbf{R}\end{bmatrix}$$
as `SE3::Adj` builds it (`se3.hpp:105-113`).

(I8) Inverse of an exponential. $\text{Exp}(\boldsymbol{\zeta})^{-1} = \text{Exp}(-\boldsymbol{\zeta})$, as used in `doc/VIO.md` §2.2.1. For $SO(3)$ the inverse is the transpose, so $\text{Exp}(\boldsymbol{\omega})^\top = \text{Exp}(-\boldsymbol{\omega})$, which also follows from (I1) because $[\boldsymbol{\omega}]_\times^\top = -[\boldsymbol{\omega}]_\times = [-\boldsymbol{\omega}]_\times$.

(I9) Jacobian of a perturbed residual. For a residual $\mathbf{r}$ and an increment $\boldsymbol{\epsilon}$ applied by a variable's update rule, the Jacobian $\partial\mathbf{r}/\partial\boldsymbol{\epsilon}$ is the matrix $\mathbf{J}$ for which $\mathbf{r}(\boldsymbol{\epsilon}) = \mathbf{r} + \mathbf{J}\boldsymbol{\epsilon} + O(|\boldsymbol{\epsilon}|^2)$, where $\mathbf{r}$ is the unperturbed value. This is the first order expansion that Gauss–Newton uses (`doc/VIO.md` §2.2.4), so a Jacobian is identified by perturbing one variable, separating the unperturbed residual, and reading the matrix that multiplies the increment.

(I10) Right Jacobian of the logarithm. For $\mathbf{C} \in SO(3)$ and a small $\boldsymbol{\epsilon}$, $\text{Log}\big(\mathbf{C}\,\text{Exp}(\boldsymbol{\epsilon})\big) = \text{Log}(\mathbf{C}) + \mathbf{J}_r^{-1}\big(\text{Log}(\mathbf{C})\big)\,\boldsymbol{\epsilon} + O(|\boldsymbol{\epsilon}|^2)$, where $\mathbf{J}_r$ is the right Jacobian of $SO(3)$, `doc/VIO.md` §3.1.2, Preliminary 3, computed by `Sophus::rightJacobianInvSO3`.

### 2.1 Variable Parameterisation

#### 2.1.1 The metric world and the per frame correction

The metric world $M$ is gravity aligned, with gravity $\mathbf{g} = (0, 0, -9.81)^\top$ (`basalt::constants::g`, `include/basalt/utils/imu_types.h:60`, passed in at `src/controller.cpp:194`), and lengths in metres. The estimator stores every state in $M$ at the start of every frame, which is today's Basalt. During one frame's optimisation the estimator may conclude that its world is mis-scaled by $s$ and tilted relative to gravity, and it expresses this through a correction, a similarity without translation that maps the stored world $D$ of the current frame into a better estimate of $M$,

$$\mathbf{x}^M = s\,\mathbf{R}_{wg}^\top\,\mathbf{x}^D,$$

$$s = e^{\sigma} > 0,$$

$$\mathbf{R}_{wg} \in SO(3),$$

for any point or displacement $\mathbf{x}$. The rotation $\mathbf{R}_{wg}$ takes a vector from the metric (gravity) frame into $D$, so that the gravity vector seen in $D$ is

$$\mathbf{g}_D = \mathbf{R}_{wg}\,\mathbf{g}.$$

This is the convention of ORB-SLAM3, whose `EdgeInertialGS::computeError` forms `g = VGDir->estimate().Rwg*gI` (`G2oTypes.cc:629`). The log scale $\sigma$ is used rather than $s$ itself, as in ORB-SLAM3's `VertexScale::oplusImpl` (`G2oTypes.h:314-316`), so that the update is additive, the variable is unconstrained, and $s$ stays positive. At the start of every frame $s = 1$ and $\mathbf{R}_{wg} = \mathbf{I}$, so $D = M$. At the end of the frame the correction is applied to everything the estimator holds and reset to identity (§2.7). Within a frame, the residuals of §2.2 and §2.3 see the current $s$ and $\mathbf{g}_D$.

Only $\mathbf{g}_D$ enters any residual, so the rotation $\mathbf{R}_{wg}$ is not itself a variable. The variable is the gravity direction $\mathbf{d} = \mathbf{g}_D / |\mathbf{g}|$ on the unit sphere (§2.1.3). When the correction is applied, $\mathbf{R}_{wg}$ is taken as the inverse of the minimal rotation $\mathbf{Q}$ that carries $\mathbf{d}$ onto the gravity direction $\hat{\mathbf{g}} = \mathbf{g} / |\mathbf{g}|$,

$$\mathbf{Q} = \text{FromTwoVectors}(\mathbf{d}, \hat{\mathbf{g}}),$$

$$\mathbf{R}_{wg} = \mathbf{Q}^\top,$$

$$\mathbf{Q}\,\mathbf{d} = \hat{\mathbf{g}}.$$

The rotation axis of the minimal rotation is $\mathbf{d} \times \hat{\mathbf{g}}$, which is perpendicular to $\hat{\mathbf{g}}$ and therefore horizontal, so the correction never rotates the world about the vertical. The yaw gauge of the estimator, pinned by the initial prior at index 5 (`sqrt_keypoint_vio.cpp:105-116`), is therefore untouched.

#### 2.1.2 Frame states

The stored pose of frame $i$ is the body (IMU) pose $\mathbf{T}^D_{w,i} = (\mathbf{R}_i, \mathbf{p}_i)$ in $D$, and the stored velocity $\mathbf{v}_i$ is in $D$ as well. $\mathbf{R}_i$ and $\mathbf{p}_i$ are the $\mathbf{R}_{w,i}$ and $\mathbf{p}_{w,i}$ of `doc/VIO.md` §2.2.1, shortened because they appear in nearly every equation below, and the host and target frames of a visual residual are written $h$ and $t$, so that $\mathbf{R}_h = \mathbf{R}_{w,i_h}$ and $\mathbf{p}_t = \mathbf{p}_{w,i_t}$. Their metric counterparts under the current correction are

$$\mathbf{R}^M_i = \mathbf{R}_{wg}^\top \mathbf{R}_i,$$

$$\mathbf{p}^M_i = s\,\mathbf{R}_{wg}^\top \mathbf{p}_i,$$

$$\mathbf{v}^M_i = s\,\mathbf{R}_{wg}^\top \mathbf{v}_i,$$

and the biases $\mathbf{b}_g$, $\mathbf{b}_a$ are body frame quantities that no change of world frame affects. The tangent ordering $[\delta\mathbf{p}, \delta\boldsymbol{\varphi}, \delta\mathbf{v}, \delta\mathbf{b}_g, \delta\mathbf{b}_a]$ and the left multiplicative rotation update of `PoseVelBiasState::applyInc` (`thirdparty/basalt-headers/include/basalt/imu/imu_types.h:221-225`), which acts on the pose through `PoseState::incPose` (`:98-101`) as

$$\mathbf{p}_i \leftarrow \mathbf{p}_i + \delta\mathbf{p}_i,$$

$$\mathbf{R}_i \leftarrow \text{Exp}(\delta\boldsymbol{\varphi}_i)\,\mathbf{R}_i,$$

are unchanged.

#### 2.1.3 The gravity and scale block

The gravity direction is a point $\mathbf{d}$ on the unit sphere. Following Basalt's first estimate convention, it is stored as a linearisation point $\bar{\mathbf{d}}$ with an orthonormal basis $\bar{\mathbf{B}} = [\bar{\mathbf{b}}_1\ \bar{\mathbf{b}}_2] \in \mathbb{R}^{3\times 2}$ of the plane perpendicular to $\bar{\mathbf{d}}$, plus an accumulated increment $\boldsymbol{\delta}_\theta \in \mathbb{R}^2$,

$$\mathbf{d} = \text{Exp}\big(\bar{\mathbf{B}}\,\boldsymbol{\delta}_\theta\big)\,\bar{\mathbf{d}},$$

$$\sigma = \bar{\sigma} + \delta\sigma,$$

$$\mathbf{g}_D = |\mathbf{g}|\,\mathbf{d},$$

with tangent vector $\boldsymbol{\delta}_{gs} = [\delta\theta_1, \delta\theta_2, \delta\sigma]^\top \in \mathbb{R}^3$. The increment rotates $\bar{\mathbf{d}}$ about an axis lying in the plane perpendicular to it, which moves $\bar{\mathbf{d}}$ in exactly two directions and leaves no redundant third component. Initially $\bar{\mathbf{d}} = \hat{\mathbf{g}} = (0, 0, -1)^\top$ and $\bar{\mathbf{B}} = [\mathbf{e}_x, \mathbf{e}_y]$. The gravity vector at the linearisation point is written $\bar{\mathbf{g}}_D = |\mathbf{g}|\,\bar{\mathbf{d}}$.

When the block is unfrozen, an increment moves the linearisation point and transports the basis with it, $\bar{\mathbf{d}} \leftarrow \text{Exp}(\bar{\mathbf{B}}\boldsymbol{\delta}_\theta)\,\bar{\mathbf{d}}$ and $\bar{\mathbf{B}} \leftarrow \text{Exp}(\bar{\mathbf{B}}\boldsymbol{\delta}_\theta)\,\bar{\mathbf{B}}$, which keeps $\bar{\mathbf{B}}$ orthonormal and perpendicular to $\bar{\mathbf{d}}$ because a rotation preserves inner products. When the block is frozen, $\boldsymbol{\delta}_\theta$ accumulates additively and $\mathbf{d}$ is computed from the frozen $\bar{\mathbf{d}}$, $\bar{\mathbf{B}}$, matching `PoseStateWithLin` (`include/basalt/utils/imu_types.h:238-246`).

Two properties decide this choice. The first is the Jacobian of $\mathbf{g}_D$ at the linearisation point $\boldsymbol{\delta}_\theta = \mathbf{0}$, at which every Jacobian of §2.3 is evaluated. Starting from the definition,

$$\mathbf{g}_D(\boldsymbol{\delta}_\theta) = |\mathbf{g}|\,\text{Exp}\big(\bar{\mathbf{B}}\boldsymbol{\delta}_\theta\big)\,\bar{\mathbf{d}}.$$

Expanding the exponential to first order by (I1), with $\boldsymbol{\omega} = \bar{\mathbf{B}}\boldsymbol{\delta}_\theta$,

$$\mathbf{g}_D(\boldsymbol{\delta}_\theta) = |\mathbf{g}|\,\big(\mathbf{I} + [\bar{\mathbf{B}}\boldsymbol{\delta}_\theta]_\times\big)\,\bar{\mathbf{d}} + O(|\boldsymbol{\delta}_\theta|^2) = |\mathbf{g}|\,\bar{\mathbf{d}} + |\mathbf{g}|\,[\bar{\mathbf{B}}\boldsymbol{\delta}_\theta]_\times\,\bar{\mathbf{d}} + O(|\boldsymbol{\delta}_\theta|^2).$$

Swapping the two factors of the cross product by (I2), with $\mathbf{a} = \bar{\mathbf{B}}\boldsymbol{\delta}_\theta$ and $\mathbf{b} = \bar{\mathbf{d}}$,

$$[\bar{\mathbf{B}}\boldsymbol{\delta}_\theta]_\times\,\bar{\mathbf{d}} = -[\bar{\mathbf{d}}]_\times\,\bar{\mathbf{B}}\,\boldsymbol{\delta}_\theta,$$

so that

$$\mathbf{g}_D(\boldsymbol{\delta}_\theta) = |\mathbf{g}|\,\bar{\mathbf{d}} - |\mathbf{g}|\,[\bar{\mathbf{d}}]_\times\,\bar{\mathbf{B}}\,\boldsymbol{\delta}_\theta + O(|\boldsymbol{\delta}_\theta|^2).$$

The first order term is linear in $\boldsymbol{\delta}_\theta$, so its coefficient matrix is the Jacobian. Moving the scalar $|\mathbf{g}|$ inside the hat by (I3) and using $\bar{\mathbf{g}}_D = |\mathbf{g}|\,\bar{\mathbf{d}}$,

$$\frac{\partial \mathbf{g}_D}{\partial\boldsymbol{\delta}_\theta}\Big|_{\boldsymbol{\delta}_\theta = \mathbf{0}} = -|\mathbf{g}|\,[\bar{\mathbf{d}}]_\times\,\bar{\mathbf{B}} = -\big[|\mathbf{g}|\,\bar{\mathbf{d}}\big]_\times\,\bar{\mathbf{B}} = -[\bar{\mathbf{g}}_D]_\times\,\bar{\mathbf{B}}.$$

The second property is that a rotation $\mathbf{Q}$ of the whole world maps the block to $\mathbf{Q}\bar{\mathbf{d}}$, $\mathbf{Q}\bar{\mathbf{B}}$ with the same increment $\boldsymbol{\delta}_\theta$, exactly. The direction the rotated block represents is $\text{Exp}(\mathbf{Q}\bar{\mathbf{B}}\boldsymbol{\delta}_\theta)\,\mathbf{Q}\bar{\mathbf{d}}$. Applying (I4) with $\mathbf{R} = \mathbf{Q}$ and $\mathbf{a} = \bar{\mathbf{B}}\boldsymbol{\delta}_\theta$, and then $\mathbf{Q}^\top \mathbf{Q} = \mathbf{I}$,

$$\text{Exp}\big(\mathbf{Q}\bar{\mathbf{B}}\boldsymbol{\delta}_\theta\big)\,\mathbf{Q}\bar{\mathbf{d}} = \mathbf{Q}\,\text{Exp}\big(\bar{\mathbf{B}}\boldsymbol{\delta}_\theta\big)\,\mathbf{Q}^\top \mathbf{Q}\,\bar{\mathbf{d}} = \mathbf{Q}\,\text{Exp}\big(\bar{\mathbf{B}}\boldsymbol{\delta}_\theta\big)\,\bar{\mathbf{d}} = \mathbf{Q}\,\mathbf{d},$$

which is the rotated direction, for every $\boldsymbol{\delta}_\theta$ and not only to first order. The per frame re-centring of §2.7 relies on this property. An $SO(3)$ parameterisation with a fixed zero third component, as in ORB-SLAM3 (`Rwg = Rwg*ExpSO3(pu[0],pu[1],0.0)`, `G2oTypes.h:266`) or with the same restriction applied on the left, maps an increment to one with a nonzero third component under a world rotation, so it cannot be re-centred exactly.

#### 2.1.4 Landmarks

A landmark keeps its present parameterisation (`doc/VIO.md` §2.2.3), a stereographic bearing $\boldsymbol{\theta} \in \mathbb{R}^2$ in the host camera frame and an inverse distance $\rho$, stacked as $\mathbf{l} = [\boldsymbol{\theta}^\top, \rho]^\top$. $\rho$ is a metric inverse distance, in inverse metres, at all times, exactly as it is today.

The bearing is unaffected by gravity because it is expressed in the host camera frame, which rotates rigidly with the body, so a correction that rotates the whole world leaves the bearing of a landmark relative to its host camera unchanged. The estimator's own landmarks need no update when the correction is applied either. Within the frame, the scale enters the visual residual through $s\,\mathbf{t}_{ti,hi}$ (§2.2.1), and the joint solve moves every landmark's $\rho$ together with $\sigma$, by approximately $-\rho\,\delta\sigma$ for a camera without lever arm (§2.2.4). At the end of the frame the landmarks are therefore already consistent with the corrected geometry, and the re-centring of §2.7 leaves them untouched. The local mapper's landmarks did not take part in the solve, so the mapper must divide their inverse distances by $s$ when it replays the correction (§2.9). This is the request's intuition that a change of scale updates the inverse distance with respect to the host frame.

#### 2.1.5 The state vector in the initialisation period

$$\mathbf{s} = \Big[\ \delta\theta_1,\ \delta\theta_2,\ \delta\sigma,\ \ \mathbf{T}_{k},\ \ (\mathbf{T}, \mathbf{v}, \mathbf{b}_g, \mathbf{b}_a)_f,\ \ \mathbf{l}_j\ \Big].$$

The four groups are, in order, the gravity and scale block of three dimensions, the keyframe poses $\mathbf{T}_k$ held in `frame_poses` with six dimensions each, the full states held in `frame_states` with fifteen dimensions each, and the landmarks with three dimensions each.

The gravity and scale block is placed first in the absolute order map. The reason is the prefix invariant of `context/vio_opt.md` §4.2. The marginalisation prior must occupy a contiguous prefix of `aom` in the same order, and the block is part of every prior built during the period, so it must precede every frame. Placed first, the prior order becomes `[gs, poses…, last state]` and the optimisation order `[gs, poses…, states…]`, and the prefix property is preserved without any permutation.

### 2.2 Visual Residual

#### 2.2.1 Formulation

The reprojection residual must be evaluated with metric quantities, because the camera to IMU extrinsics $\mathbf{T}_{i,c}$ are metric and the landmark inverse distance is metric. Following `doc/VIO.md` §2.2.1, the host to target camera transform is built from the metric IMU poses,

$$\mathbf{T}_{t,h} = \mathbf{T}_{i,c_t}^{-1}\ \big(\mathbf{T}^M_{w,i_t}\big)^{-1}\ \mathbf{T}^M_{w,i_h}\ \mathbf{T}_{i,c_h}.$$

The metric poses of the two frames follow from §2.1.2,

$$\mathbf{T}^M_{w,i_h} = \big(\mathbf{R}_{wg}^\top \mathbf{R}_h,\ s\,\mathbf{R}_{wg}^\top \mathbf{p}_h\big),$$

$$\mathbf{T}^M_{w,i_t} = \big(\mathbf{R}_{wg}^\top \mathbf{R}_t,\ s\,\mathbf{R}_{wg}^\top \mathbf{p}_t\big).$$

The inverse of the target pose follows from the inverse rule of (I5), $(\mathbf{R}, \mathbf{t})^{-1} = (\mathbf{R}^\top, -\mathbf{R}^\top \mathbf{t})$, with $\mathbf{R} = \mathbf{R}_{wg}^\top \mathbf{R}_t$ and $\mathbf{t} = s\,\mathbf{R}_{wg}^\top \mathbf{p}_t$. The transpose of a product reverses its factors, $(\mathbf{R}_{wg}^\top \mathbf{R}_t)^\top = \mathbf{R}_t^\top \mathbf{R}_{wg}$, so

$$\big(\mathbf{T}^M_{w,i_t}\big)^{-1} = \big(\mathbf{R}_t^\top \mathbf{R}_{wg},\ -\mathbf{R}_t^\top \mathbf{R}_{wg}\,s\,\mathbf{R}_{wg}^\top \mathbf{p}_t\big) = \big(\mathbf{R}_t^\top \mathbf{R}_{wg},\ -s\,\mathbf{R}_t^\top \mathbf{p}_t\big),$$

where the second step uses $\mathbf{R}_{wg} \mathbf{R}_{wg}^\top = \mathbf{I}$ and moves the scalar $s$ to the front. The product then follows from the composition rule of (I5), $(\mathbf{R}_1, \mathbf{t}_1)(\mathbf{R}_2, \mathbf{t}_2) = (\mathbf{R}_1 \mathbf{R}_2, \mathbf{R}_1 \mathbf{t}_2 + \mathbf{t}_1)$, with the inverse above as the first factor and $\mathbf{T}^M_{w,i_h}$ as the second. The rotation part is

$$\mathbf{R}_t^\top \mathbf{R}_{wg}\,\mathbf{R}_{wg}^\top \mathbf{R}_h = \mathbf{R}_t^\top \mathbf{R}_h,$$

and the translation part is

$$\mathbf{R}_t^\top \mathbf{R}_{wg}\,s\,\mathbf{R}_{wg}^\top \mathbf{p}_h - s\,\mathbf{R}_t^\top \mathbf{p}_t = s\,\mathbf{R}_t^\top \mathbf{p}_h - s\,\mathbf{R}_t^\top \mathbf{p}_t = s\,\mathbf{R}_t^\top(\mathbf{p}_h - \mathbf{p}_t),$$

again by $\mathbf{R}_{wg} \mathbf{R}_{wg}^\top = \mathbf{I}$. Together,

$$\big(\mathbf{T}^M_{w,i_t}\big)^{-1} \mathbf{T}^M_{w,i_h} = \big(\mathbf{R}_t^\top \mathbf{R}_h,\ \ s\,\mathbf{R}_t^\top(\mathbf{p}_h - \mathbf{p}_t)\big).$$

$\mathbf{R}_{wg}$ cancels, so gravity direction never enters the visual residual, which is the expectation stated in the request. Scale survives as a factor on the relative translation of the two IMU frames. The same product in $D$, which is the result above at $s = 1$, is the IMU to IMU relative pose that `computeRelPose` forms as `T_t_i_h_i` (`include/basalt/utils/ba_utils.h:49-52`),

$$\mathbf{T}_{ti,hi} = \big(\mathbf{T}^D_{w,i_t}\big)^{-1} \mathbf{T}^D_{w,i_h} = \big(\mathbf{R}_{ti,hi},\ \mathbf{t}_{ti,hi}\big),$$

$$\mathbf{R}_{ti,hi} = \mathbf{R}_t^\top \mathbf{R}_h,$$

$$\mathbf{t}_{ti,hi} = \mathbf{R}_t^\top(\mathbf{p}_h - \mathbf{p}_t).$$

Define the scaling map on $SE(3)$, which multiplies the translation of a pose by $s$ and leaves its rotation alone,

$$\mathcal{S}_s\big((\mathbf{R}, \mathbf{t})\big) = (\mathbf{R},\ s\,\mathbf{t}).$$

The metric product above is then $\mathcal{S}_s(\mathbf{T}_{ti,hi})$, and

$$\mathbf{T}_{t,h}(s) = \mathbf{T}_{i,c_t}^{-1}\ \mathcal{S}_s\big(\mathbf{T}_{ti,hi}\big)\ \mathbf{T}_{i,c_h}.$$

The residual keeps the form of `doc/VIO.md` §2.2.4 unchanged,

$$\tilde{\mathbf{p}}_t = \mathbf{T}_{t,h}(s)\,\tilde{\mathbf{p}}_h,$$

$$\tilde{\mathbf{p}}_h = \begin{bmatrix}\pi_\text{st}^{-1}(\boldsymbol{\theta}) \\ \rho\end{bmatrix},$$

$$\mathbf{r} = \pi\big(\tilde{\mathbf{p}}_t[1{:}3]\big) - \mathbf{z}.$$

Since $\mathbf{T}_{t,h}(s)$ is the only place $s$ appears, the projection, the landmark Jacobian $\partial\mathbf{r}/\partial\mathbf{l}$ and the Jacobian $\partial\mathbf{r}/\partial\boldsymbol{\xi}$ with respect to the left perturbation $\text{Exp}(\boldsymbol{\xi})\,\mathbf{T}_{t,h}$ are exactly those `linearizePoint` already computes (`ba_utils.h:83-144`). What changes is only the chain rule from the state increments to $\boldsymbol{\xi}$, which is the job of `computeRelPose`, and therefore of a new `computeRelPoseGS`.

#### 2.2.2 The scaling map and its tangent matrix

Two properties of $\mathcal{S}_s$ carry the whole derivation.

First, $\mathcal{S}_s$ preserves products. Take two poses $\mathbf{T}_1 = (\mathbf{R}_1, \mathbf{t}_1)$ and $\mathbf{T}_2 = (\mathbf{R}_2, \mathbf{t}_2)$. Applying the definition of $\mathcal{S}_s$ to each and then the composition rule of (I5),

$$\mathcal{S}_s(\mathbf{T}_1)\,\mathcal{S}_s(\mathbf{T}_2) = (\mathbf{R}_1,\ s\mathbf{t}_1)(\mathbf{R}_2,\ s\mathbf{t}_2) = (\mathbf{R}_1 \mathbf{R}_2,\ \mathbf{R}_1 s\mathbf{t}_2 + s\mathbf{t}_1) = \big(\mathbf{R}_1 \mathbf{R}_2,\ s(\mathbf{R}_1 \mathbf{t}_2 + \mathbf{t}_1)\big).$$

By (I5) again, $\mathbf{T}_1 \mathbf{T}_2 = (\mathbf{R}_1 \mathbf{R}_2, \mathbf{R}_1 \mathbf{t}_2 + \mathbf{t}_1)$, and the right hand side above is $\mathcal{S}_s$ applied to it. Hence

$$\mathcal{S}_s(\mathbf{T}_1)\,\mathcal{S}_s(\mathbf{T}_2) = \mathcal{S}_s(\mathbf{T}_1 \mathbf{T}_2). \tag{S1}$$

The map is also invertible, since $\mathcal{S}_{1/s}\big(\mathcal{S}_s((\mathbf{R}, \mathbf{t}))\big) = (\mathbf{R}, \mathbf{t})$. A bijection of a group onto itself that preserves products is a group automorphism, so $\mathcal{S}_s$ is an automorphism of $SE(3)$.

Second, $\mathcal{S}_s$ acts on a perturbation $\text{Exp}(\boldsymbol{\zeta})$ through a fixed $6\times 6$ matrix. Writing $\boldsymbol{\zeta} = [\boldsymbol{\upsilon}; \boldsymbol{\omega}]$ and expanding the exponential by (I6),

$$\mathcal{S}_s\big(\text{Exp}(\boldsymbol{\zeta})\big) = \mathcal{S}_s\big((\text{Exp}(\boldsymbol{\omega}),\ \mathbf{J}_l(\boldsymbol{\omega})\boldsymbol{\upsilon})\big) = \big(\text{Exp}(\boldsymbol{\omega}),\ s\,\mathbf{J}_l(\boldsymbol{\omega})\boldsymbol{\upsilon}\big) = \big(\text{Exp}(\boldsymbol{\omega}),\ \mathbf{J}_l(\boldsymbol{\omega})\,(s\boldsymbol{\upsilon})\big),$$

where the last step moves the scalar $s$ past the matrix $\mathbf{J}_l(\boldsymbol{\omega})$. Reading (I6) backwards, the right hand side is $\text{Exp}([s\boldsymbol{\upsilon}; \boldsymbol{\omega}])$. Writing the map from $[\boldsymbol{\upsilon}; \boldsymbol{\omega}]$ to $[s\boldsymbol{\upsilon}; \boldsymbol{\omega}]$ as a matrix,

$$\mathcal{S}_s\big(\text{Exp}(\boldsymbol{\zeta})\big) = \text{Exp}\big(\mathbf{D}_s\boldsymbol{\zeta}\big) \tag{S2}$$

with

$$\mathbf{D}_s = \begin{bmatrix}s\,\mathbf{I}_3 & \mathbf{0}\\ \mathbf{0} & \mathbf{I}_3\end{bmatrix}.$$

$\mathbf{D}_s$ is therefore the tangent space counterpart of $\mathcal{S}_s$, in other words the derivative of $\mathcal{S}_s$ at the identity. Whatever perturbation passes through $\mathcal{S}_s$ has its translational part multiplied by $s$ and its rotational part left unchanged, which is what scaling a translation should do to a small motion. The identity holds exactly, not only to first order, because the scalar $s$ commutes with $\mathbf{J}_l(\boldsymbol{\omega})$ for every $\boldsymbol{\omega}$. At $s = 1$, $\mathbf{D}_1 = \mathbf{I}_6$.

#### 2.2.3 Jacobians with respect to the host and target states

`doc/VIO.md` §2.2.1 derives the relative pose Jacobians for a right perturbation of each body pose, $\mathbf{T}_{w,i} \to \mathbf{T}_{w,i}\,\text{Exp}(\boldsymbol{\delta})$ with $\boldsymbol{\delta} \in \mathbb{R}^6$, and the left perturbation $\mathbf{T}_{t,h} \to \text{Exp}(\boldsymbol{\xi})\,\mathbf{T}_{t,h}$ of the result. This section keeps both conventions and proceeds in three steps. It first relates $\boldsymbol{\delta}$ to the state increment $[\delta\mathbf{p}; \delta\boldsymbol{\varphi}]$ that Basalt's solver actually applies (§2.1.2), then derives $\partial\boldsymbol{\xi}/\partial\boldsymbol{\delta}$ for the host and the target with the scale, and finally chains the two.

Step 1, the right perturbation and Basalt's increment. Expand a right perturbation of $\mathbf{T}^D_{w,i} = (\mathbf{R}_i, \mathbf{p}_i)$ with $\boldsymbol{\delta} = [\boldsymbol{\upsilon}; \boldsymbol{\omega}]$. By the first order form of (I6) and the composition rule of (I5),

$$\mathbf{T}^D_{w,i}\,\text{Exp}(\boldsymbol{\delta}) = (\mathbf{R}_i, \mathbf{p}_i)\,\big(\text{Exp}(\boldsymbol{\omega}),\ \boldsymbol{\upsilon}\big) + O(|\boldsymbol{\delta}|^2) = \big(\mathbf{R}_i\,\text{Exp}(\boldsymbol{\omega}),\ \mathbf{p}_i + \mathbf{R}_i\boldsymbol{\upsilon}\big) + O(|\boldsymbol{\delta}|^2).$$

The rotation is moved to the left form that `incPose` uses by (I4) with $\mathbf{R} = \mathbf{R}_i$ and $\mathbf{a} = \boldsymbol{\omega}$, $\text{Exp}(\mathbf{R}_i\boldsymbol{\omega}) = \mathbf{R}_i\,\text{Exp}(\boldsymbol{\omega})\,\mathbf{R}_i^\top$, multiplied on the right by $\mathbf{R}_i$,

$$\mathbf{R}_i\,\text{Exp}(\boldsymbol{\omega}) = \text{Exp}(\mathbf{R}_i\boldsymbol{\omega})\,\mathbf{R}_i.$$

The perturbed pose is $\big(\text{Exp}(\mathbf{R}_i\boldsymbol{\omega})\,\mathbf{R}_i,\ \mathbf{p}_i + \mathbf{R}_i\boldsymbol{\upsilon}\big)$, and the increment of §2.1.2 gives $\big(\text{Exp}(\delta\boldsymbol{\varphi}_i)\,\mathbf{R}_i,\ \mathbf{p}_i + \delta\mathbf{p}_i\big)$. The two agree when $\mathbf{R}_i\boldsymbol{\omega} = \delta\boldsymbol{\varphi}_i$ and $\mathbf{R}_i\boldsymbol{\upsilon} = \delta\mathbf{p}_i$, that is, when

$$\boldsymbol{\delta} = \begin{bmatrix} \mathbf{R}_i^\top & \mathbf{0}\\ \mathbf{0} & \mathbf{R}_i^\top\end{bmatrix}\begin{bmatrix}\delta\mathbf{p}_i\\ \delta\boldsymbol{\varphi}_i\end{bmatrix},$$

$$\frac{\partial\boldsymbol{\delta}}{\partial[\delta\mathbf{p}_i; \delta\boldsymbol{\varphi}_i]} = \begin{bmatrix} \mathbf{R}_i^\top & \mathbf{0}\\ \mathbf{0} & \mathbf{R}_i^\top\end{bmatrix}.$$

This block diagonal matrix is the `RR` that `computeRelPose` builds from `T_w_i_h.so3().inverse()` and `T_w_i_t.so3().inverse()` and multiplies onto both Jacobians (`ba_utils.h:57-80`). It is the rotation only factor that `doc/VIO.md` §2.2.1 records, in its note of 2026-11-02, as an open discrepancy with its own formulas, and Step 1 is its derivation.

Step 2a, the host. Perturb only the host, $\mathbf{T}^D_{w,i_h} \to \mathbf{T}^D_{w,i_h}\,\text{Exp}(\boldsymbol{\delta}_h)$. The relative pose in $D$ becomes

$$\big(\mathbf{T}^D_{w,i_t}\big)^{-1} \mathbf{T}^D_{w,i_h}\,\text{Exp}(\boldsymbol{\delta}_h) = \mathbf{T}_{ti,hi}\,\text{Exp}(\boldsymbol{\delta}_h).$$

The scaling map is applied to this product. By (S1) and then (S2),

$$\mathcal{S}_s\big(\mathbf{T}_{ti,hi}\,\text{Exp}(\boldsymbol{\delta}_h)\big) = \mathcal{S}_s(\mathbf{T}_{ti,hi})\,\mathcal{S}_s\big(\text{Exp}(\boldsymbol{\delta}_h)\big) = \mathcal{S}_s(\mathbf{T}_{ti,hi})\,\text{Exp}(\mathbf{D}_s\boldsymbol{\delta}_h).$$

Substituting into $\mathbf{T}_{t,h}(s)$ of §2.2.1 and grouping the factors to the left of the exponential,

$$\mathbf{T}_{t,h}' = \mathbf{T}_{i,c_t}^{-1}\,\mathcal{S}_s(\mathbf{T}_{ti,hi})\,\text{Exp}(\mathbf{D}_s\boldsymbol{\delta}_h)\,\mathbf{T}_{i,c_h} = \Big(\mathbf{T}_{i,c_t}^{-1}\,\mathcal{S}_s(\mathbf{T}_{ti,hi})\Big)\,\text{Exp}(\mathbf{D}_s\boldsymbol{\delta}_h)\,\mathbf{T}_{i,c_h}.$$

(I7) with $\mathbf{T} = \mathbf{T}_{i,c_t}^{-1}\,\mathcal{S}_s(\mathbf{T}_{ti,hi})$ and $\boldsymbol{\zeta} = \mathbf{D}_s\boldsymbol{\delta}_h$ carries the exponential to the far left,

$$\mathbf{T}_{t,h}' = \text{Exp}\Big(\text{Adj}\big(\mathbf{T}_{i,c_t}^{-1}\,\mathcal{S}_s(\mathbf{T}_{ti,hi})\big)\,\mathbf{D}_s\,\boldsymbol{\delta}_h\Big)\ \mathbf{T}_{i,c_t}^{-1}\,\mathcal{S}_s(\mathbf{T}_{ti,hi})\,\mathbf{T}_{i,c_h} = \text{Exp}(\boldsymbol{\xi})\,\mathbf{T}_{t,h}(s),$$

with $\boldsymbol{\xi} = \text{Adj}\big(\mathbf{T}_{i,c_t}^{-1}\,\mathcal{S}_s(\mathbf{T}_{ti,hi})\big)\,\mathbf{D}_s\,\boldsymbol{\delta}_h$. Hence

$$\frac{\partial\boldsymbol{\xi}}{\partial\boldsymbol{\delta}_h} = \text{Adj}\big(\mathbf{T}_{i,c_t}^{-1}\,\mathcal{S}_s(\mathbf{T}_{ti,hi})\big)\,\mathbf{D}_s.$$

Step 2b, the target. Perturb only the target, $\mathbf{T}^D_{w,i_t} \to \mathbf{T}^D_{w,i_t}\,\text{Exp}(\boldsymbol{\delta}_t)$. By the product inverse rule of (I5) and then (I8),

$$\big(\mathbf{T}^D_{w,i_t}\,\text{Exp}(\boldsymbol{\delta}_t)\big)^{-1} = \text{Exp}(\boldsymbol{\delta}_t)^{-1}\big(\mathbf{T}^D_{w,i_t}\big)^{-1} = \text{Exp}(-\boldsymbol{\delta}_t)\big(\mathbf{T}^D_{w,i_t}\big)^{-1},$$

so the relative pose in $D$ becomes $\text{Exp}(-\boldsymbol{\delta}_t)\,\mathbf{T}_{ti,hi}$. By (S1) and then (S2) with $\boldsymbol{\zeta} = -\boldsymbol{\delta}_t$,

$$\mathcal{S}_s\big(\text{Exp}(-\boldsymbol{\delta}_t)\,\mathbf{T}_{ti,hi}\big) = \text{Exp}(-\mathbf{D}_s\boldsymbol{\delta}_t)\,\mathcal{S}_s(\mathbf{T}_{ti,hi}).$$

Substituting into $\mathbf{T}_{t,h}(s)$, the exponential stands to the right of $\mathbf{T}_{i,c_t}^{-1}$ alone,

$$\mathbf{T}_{t,h}' = \mathbf{T}_{i,c_t}^{-1}\,\text{Exp}(-\mathbf{D}_s\boldsymbol{\delta}_t)\,\mathcal{S}_s(\mathbf{T}_{ti,hi})\,\mathbf{T}_{i,c_h}.$$

(I7) with $\mathbf{T} = \mathbf{T}_{i,c_t}^{-1}$ and $\boldsymbol{\zeta} = -\mathbf{D}_s\boldsymbol{\delta}_t$ gives

$$\mathbf{T}_{t,h}' = \text{Exp}\big(-\text{Adj}(\mathbf{T}_{i,c_t}^{-1})\,\mathbf{D}_s\,\boldsymbol{\delta}_t\big)\,\mathbf{T}_{t,h}(s),$$

$$\frac{\partial\boldsymbol{\xi}}{\partial\boldsymbol{\delta}_t} = -\text{Adj}\big(\mathbf{T}_{i,c_t}^{-1}\big)\,\mathbf{D}_s.$$

Step 3, the chain rule. Multiplying each result of Step 2 by the matrix of Step 1 for the same frame gives the Jacobians with respect to Basalt's increments,

$$\frac{\partial\boldsymbol{\xi}}{\partial[\delta\mathbf{p}_h; \delta\boldsymbol{\varphi}_h]} = \text{Adj}\big(\mathbf{T}_{i,c_t}^{-1}\,\mathcal{S}_s(\mathbf{T}_{ti,hi})\big)\,\mathbf{D}_s\begin{bmatrix} \mathbf{R}_h^\top & \mathbf{0}\\ \mathbf{0} & \mathbf{R}_h^\top\end{bmatrix},$$

$$\frac{\partial\boldsymbol{\xi}}{\partial[\delta\mathbf{p}_t; \delta\boldsymbol{\varphi}_t]} = -\text{Adj}\big(\mathbf{T}_{i,c_t}^{-1}\big)\,\mathbf{D}_s\begin{bmatrix} \mathbf{R}_t^\top & \mathbf{0}\\ \mathbf{0} & \mathbf{R}_t^\top\end{bmatrix}.$$

At $s = 1$, $\mathcal{S}_1(\mathbf{T}_{ti,hi}) = \mathbf{T}_{ti,hi}$ and $\mathbf{D}_1 = \mathbf{I}_6$, and both reduce to `d_rel_d_h = tmp.Adj() * RR` and `d_rel_d_t = −tmp2.Adj() * RR` (`ba_utils.h:65`, `:76`), where `tmp2` is $\mathbf{T}_{i,c_t}^{-1}$ and `tmp` is $\mathbf{T}_{i,c_t}^{-1} \mathbf{T}_{ti,hi}$ (`:47`, `:54`). This is the first consistency check of §4.

There is no derivative with respect to $\boldsymbol{\delta}_\theta$, because $\mathbf{R}_{wg}$ cancelled in §2.2.1.

#### 2.2.4 Jacobian with respect to the log scale

Increase $\sigma$ by a small $\varepsilon$, so that $s$ becomes $s\,e^{\varepsilon}$. By the definition of $\mathcal{S}_s$ and the expansion $e^{\varepsilon} = 1 + \varepsilon + O(\varepsilon^2)$,

$$\mathcal{S}_{s e^{\varepsilon}}(\mathbf{T}_{ti,hi}) = \big(\mathbf{R}_{ti,hi},\ s\,e^{\varepsilon}\,\mathbf{t}_{ti,hi}\big) = \big(\mathbf{R}_{ti,hi},\ s\,\mathbf{t}_{ti,hi} + \varepsilon\,s\,\mathbf{t}_{ti,hi}\big) + O(\varepsilon^2).$$

The added term is a pure translation applied on the left. By the exact form of (I6), $\text{Exp}([\varepsilon s\mathbf{t}_{ti,hi}; \mathbf{0}]) = (\mathbf{I}, \varepsilon s\mathbf{t}_{ti,hi})$, and by the composition rule of (I5),

$$\text{Exp}\big([\varepsilon s\mathbf{t}_{ti,hi};\ \mathbf{0}]\big)\,\mathcal{S}_s(\mathbf{T}_{ti,hi}) = (\mathbf{I},\ \varepsilon s\mathbf{t}_{ti,hi})\,(\mathbf{R}_{ti,hi},\ s\mathbf{t}_{ti,hi}) = \big(\mathbf{R}_{ti,hi},\ s\mathbf{t}_{ti,hi} + \varepsilon s\mathbf{t}_{ti,hi}\big).$$

The two right hand sides agree, so

$$\mathcal{S}_{s e^{\varepsilon}}(\mathbf{T}_{ti,hi}) = \text{Exp}\big([\varepsilon s\mathbf{t}_{ti,hi};\ \mathbf{0}]\big)\,\mathcal{S}_s(\mathbf{T}_{ti,hi}) + O(\varepsilon^2).$$

Substituting into $\mathbf{T}_{t,h}(s)$, the exponential stands to the right of $\mathbf{T}_{i,c_t}^{-1}$ alone, and (I7) with $\mathbf{T} = \mathbf{T}_{i,c_t}^{-1}$ carries it to the left,

$$\mathbf{T}_{t,h}(s\,e^{\varepsilon}) = \text{Exp}\Big(\text{Adj}\big(\mathbf{T}_{i,c_t}^{-1}\big)\begin{bmatrix}\varepsilon s\mathbf{t}_{ti,hi}\\ \mathbf{0}\end{bmatrix}\Big)\,\mathbf{T}_{t,h}(s) + O(\varepsilon^2).$$

By the inverse rule of (I5), $\mathbf{T}_{i,c_t}^{-1} = (\mathbf{R}_{i,c_t}^\top, -\mathbf{R}_{i,c_t}^\top \mathbf{t}_{i,c_t})$, and the Adjoint of (I7) applied to a vector whose rotational part is zero keeps only its upper left block,

$$\text{Adj}\big(\mathbf{T}_{i,c_t}^{-1}\big)\begin{bmatrix} \mathbf{a}\\ \mathbf{0}\end{bmatrix} = \begin{bmatrix} \mathbf{R}_{i,c_t}^\top & [-\mathbf{R}_{i,c_t}^\top \mathbf{t}_{i,c_t}]_\times \mathbf{R}_{i,c_t}^\top\\ \mathbf{0} & \mathbf{R}_{i,c_t}^\top\end{bmatrix}\begin{bmatrix} \mathbf{a}\\ \mathbf{0}\end{bmatrix} = \begin{bmatrix} \mathbf{R}_{i,c_t}^\top \mathbf{a}\\ \mathbf{0}\end{bmatrix}.$$

With $\mathbf{a} = \varepsilon s\mathbf{t}_{ti,hi}$, the coefficient of $\varepsilon$ is

$$\frac{\partial\boldsymbol{\xi}}{\partial\sigma} = \begin{bmatrix}s\,\mathbf{R}_{i,c_t}^\top \mathbf{t}_{ti,hi}\\ \mathbf{0}\end{bmatrix}.$$

For a host observation, or for a second camera of the same frame, $\mathbf{t}_{ti,hi} = \mathbf{0}$ and the derivative vanishes. The residual Jacobian follows by the chain rule, $\partial\mathbf{r}/\partial\sigma = (\partial\mathbf{r}/\partial\boldsymbol{\xi})(\partial\boldsymbol{\xi}/\partial\sigma)$, using the `d_res_d_xi` that `linearizePoint` returns.

What this derivative means is clearest when compared with the inverse distance column. From `doc/VIO.md` §2.2.5 and `ba_utils.h:112-135`, the top three rows of $\partial\tilde{\mathbf{p}}_t/\partial\boldsymbol{\xi}$ are $[\rho\,\mathbf{I}_3,\ -[\tilde{\mathbf{p}}_t[1{:}3]]_\times]$. Multiplying by $\partial\boldsymbol{\xi}/\partial\sigma$, whose rotational part is zero, keeps only the first block,

$$\frac{\partial\tilde{\mathbf{p}}_t[1{:}3]}{\partial\sigma} = \rho\,s\,\mathbf{R}_{i,c_t}^\top \mathbf{t}_{ti,hi}.$$

The translation $\mathbf{t}_{t,h}$ of $\mathbf{T}_{t,h}(s)$ follows from two applications of the composition rule of (I5). First, $\mathcal{S}_s(\mathbf{T}_{ti,hi})\,\mathbf{T}_{i,c_h} = \big(\mathbf{R}_{ti,hi} \mathbf{R}_{i,c_h},\ \mathbf{R}_{ti,hi} \mathbf{t}_{i,c_h} + s\,\mathbf{t}_{ti,hi}\big)$. Second, multiplying on the left by $\mathbf{T}_{i,c_t}^{-1} = (\mathbf{R}_{i,c_t}^\top, -\mathbf{R}_{i,c_t}^\top \mathbf{t}_{i,c_t})$,

$$\mathbf{t}_{t,h} = \mathbf{R}_{i,c_t}^\top\big(\mathbf{R}_{ti,hi} \mathbf{t}_{i,c_h} + s\,\mathbf{t}_{ti,hi}\big) - \mathbf{R}_{i,c_t}^\top \mathbf{t}_{i,c_t} = s\,\mathbf{R}_{i,c_t}^\top \mathbf{t}_{ti,hi} + \mathbf{c},$$

$$\mathbf{c} = \mathbf{R}_{i,c_t}^\top\big(\mathbf{R}_{ti,hi}\,\mathbf{t}_{i,c_h} - \mathbf{t}_{i,c_t}\big),$$

where $\mathbf{c}$ collects the part contributed by the metric extrinsic lever arms. The inverse distance column is $\partial\tilde{\mathbf{p}}_t[1{:}3]/\partial\rho = \mathbf{t}_{t,h}$ (`doc/VIO.md` §2.2.5). Multiplying it by $\rho$ and substituting the decomposition,

$$\rho\,\frac{\partial\tilde{\mathbf{p}}_t[1{:}3]}{\partial\rho} = \rho\,s\,\mathbf{R}_{i,c_t}^\top \mathbf{t}_{ti,hi} + \rho\,\mathbf{c} = \frac{\partial\tilde{\mathbf{p}}_t[1{:}3]}{\partial\sigma} + \rho\,\mathbf{c}.$$

Solving for the scale column and multiplying on the left by the projection Jacobian $\mathbf{J}_\pi$ of `doc/VIO.md` §2.2.2, since $\mathbf{r} = \pi(\tilde{\mathbf{p}}_t[1{:}3]) - \mathbf{z}$,

$$\frac{\partial\mathbf{r}}{\partial\sigma} = \rho\,\frac{\partial\mathbf{r}}{\partial\rho} - \rho\,\mathbf{J}_\pi\,\mathbf{c}.$$

Two conclusions follow. When the lever arm term $\mathbf{c}$ vanishes, the scale column of every observation of a landmark equals $\rho$ times that landmark's inverse distance column, so it lies in the span of the landmark's columns. The per landmark QR of `context/vio_opt.md` §7.3 projects every residual onto the orthogonal complement of the landmark's columns, so it annihilates the scale column exactly. Vision then gives the scale no information, as monocular geometry should, and couples to it only through back substitution. In back substitution the landmark increment solves the landmark's own least squares problem with the frame and scale increments held fixed, and since the scale column enters as $\rho\,\delta\sigma$ times the inverse distance column, the solution absorbs it as a shift of $-\rho\,\delta\sigma$ in $\delta\rho$. That shift keeps the product $\rho\,s$, the landmark's inverse distance in $D$ units, fixed to first order. When $\mathbf{c} \neq \mathbf{0}$, the residual $-\rho\,\mathbf{J}_\pi\,\mathbf{c}$ is genuine metric information from the extrinsic lever arm, and with two cameras of one rig observing the same landmark the metric baseline makes the scale directly observable, which is why ORB-SLAM3 fixes the scale for stereo (`VS->setFixed(!bMono)`, `Optimizer.cc:3125`). In both cases the exact Jacobian is used and the QR handles it without special treatment.

#### 2.2.5 Evaluation point

The scale used to evaluate $\mathbf{T}_{t,h}(s)$ and its Jacobians follows the first estimate rule of the rest of the window. Jacobians are formed at the linearisation point of the gravity and scale block, $\bar{s} = e^{\bar{\sigma}}$, and the value of $\mathbf{T}_{t,h}$ is recomputed at the current $s = e^{\bar{\sigma} + \delta\sigma}$ whenever the block or either frame is frozen, exactly as `linearization_abs_qr.cpp:206-221` does today for the frames.

### 2.3 IMU Residuals

#### 2.3.1 Formulation

The preintegrated measurement $(\Delta\mathbf{R}, \Delta\mathbf{v}, \Delta\mathbf{p})$ with its bias Jacobians (`doc/VIO.md` §3.1.1) is defined in the body frame and is metric. The residual of `IntegratedImuMeasurement::residual` (`thirdparty/basalt-headers/include/basalt/imu/preintegration.h:221-293`) must therefore be evaluated with the metric states of §2.1.2. Its three rows are

$$\mathbf{r}_p = \big(\mathbf{R}^M_0\big)^\top\Big(\mathbf{p}^M_1 - \mathbf{p}^M_0 - \mathbf{v}^M_0\Delta t - \frac{1}{2} \mathbf{g}\Delta t^2\Big) - \hat{\Delta\mathbf{p}},$$

$$\mathbf{r}_R = \text{Log}\Big(\text{Exp}\big(\boldsymbol{\delta}_g[4{:}6]\big)\,\Delta\mathbf{R}\,\big(\mathbf{R}^M_1\big)^\top \mathbf{R}^M_0\Big),$$

$$\mathbf{r}_v = \big(\mathbf{R}^M_0\big)^\top\big(\mathbf{v}^M_1 - \mathbf{v}^M_0 - \mathbf{g}\Delta t\big) - \hat{\Delta\mathbf{v}},$$

where the hatted symbols are the bias corrected deltas of `doc/VIO.md` §3.1.1,

$$\hat{\Delta\mathbf{p}} = \Delta\mathbf{p} + \boldsymbol{\delta}_g[1{:}3] + \boldsymbol{\delta}_a[1{:}3],$$

$$\hat{\Delta\mathbf{v}} = \Delta\mathbf{v} + \boldsymbol{\delta}_g[7{:}9] + \boldsymbol{\delta}_a[7{:}9].$$

Neither depends on a pose, a velocity, $s$ or $\mathbf{g}_D$. The substitution of the metric states of §2.1.2 proceeds row by row.

Position row. Step 1. The common factor follows from $\mathbf{R}^M_0 = \mathbf{R}_{wg}^\top \mathbf{R}_0$, because the transpose of a product reverses its factors and $(\mathbf{R}_{wg}^\top)^\top = \mathbf{R}_{wg}$,

$$\big(\mathbf{R}^M_0\big)^\top = \mathbf{R}_0^\top \mathbf{R}_{wg}.$$

Step 2. Substituting $\mathbf{p}^M_i = s\,\mathbf{R}_{wg}^\top \mathbf{p}_i$ and $\mathbf{v}^M_0 = s\,\mathbf{R}_{wg}^\top \mathbf{v}_0$ and collecting the common factor $s\,\mathbf{R}_{wg}^\top$ by distributivity,

$$\mathbf{p}^M_1 - \mathbf{p}^M_0 - \mathbf{v}^M_0\Delta t = s\,\mathbf{R}_{wg}^\top(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t).$$

Step 3. Multiplying the bracket of $\mathbf{r}_p$ on the left by the factor of Step 1 and distributing over its two terms,

$$\mathbf{R}_0^\top \mathbf{R}_{wg}\Big(s\,\mathbf{R}_{wg}^\top(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}\Delta t^2\Big) = s\,\mathbf{R}_0^\top \mathbf{R}_{wg} \mathbf{R}_{wg}^\top(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2}\Delta t^2\,\mathbf{R}_0^\top \mathbf{R}_{wg} \mathbf{g}.$$

Step 4. The first term uses $\mathbf{R}_{wg} \mathbf{R}_{wg}^\top = \mathbf{I}$ and the second uses $\mathbf{R}_{wg} \mathbf{g} = \mathbf{g}_D$ of §2.1.1,

$$s\,\mathbf{R}_0^\top(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2}\Delta t^2\,\mathbf{R}_0^\top \mathbf{g}_D = \mathbf{R}_0^\top\Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}_D\Delta t^2\Big).$$

Velocity row. Steps 1 to 4 repeat with $\mathbf{v}^M_1 - \mathbf{v}^M_0 = s\,\mathbf{R}_{wg}^\top(\mathbf{v}_1 - \mathbf{v}_0)$ in Step 2 and $\Delta t$ in place of $\frac{1}{2}\Delta t^2$,

$$\mathbf{R}_0^\top \mathbf{R}_{wg}\big(s\,\mathbf{R}_{wg}^\top(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}\Delta t\big) = s\,\mathbf{R}_0^\top \mathbf{R}_{wg} \mathbf{R}_{wg}^\top(\mathbf{v}_1 - \mathbf{v}_0) - \Delta t\,\mathbf{R}_0^\top \mathbf{R}_{wg} \mathbf{g},$$

$$s\,\mathbf{R}_0^\top \mathbf{R}_{wg} \mathbf{R}_{wg}^\top(\mathbf{v}_1 - \mathbf{v}_0) - \Delta t\,\mathbf{R}_0^\top \mathbf{R}_{wg} \mathbf{g} = \mathbf{R}_0^\top\big(s(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D\Delta t\big).$$

Rotation row. Using Step 1 for both states and $\mathbf{R}_{wg} \mathbf{R}_{wg}^\top = \mathbf{I}$,

$$\big(\mathbf{R}^M_1\big)^\top \mathbf{R}^M_0 = \mathbf{R}_1^\top \mathbf{R}_{wg} \mathbf{R}_{wg}^\top \mathbf{R}_0 = \mathbf{R}_1^\top \mathbf{R}_0.$$

The gravity and scale residual, `residualGS`, is therefore

$$\mathbf{r}_p = \mathbf{R}_0^\top\Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}_D\Delta t^2\Big) - \hat{\Delta\mathbf{p}},$$

$$\mathbf{r}_R = \text{Log}\Big(\text{Exp}\big(\boldsymbol{\delta}_g[4{:}6]\big)\,\Delta\mathbf{R}\,\mathbf{R}_1^{\top} \mathbf{R}_0\Big),$$

$$\mathbf{r}_v = \mathbf{R}_0^\top\big(s(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D\Delta t\big) - \hat{\Delta\mathbf{v}}.$$

The rotation residual is untouched because $\mathbf{R}_{wg}$ cancels. This is term for term the residual of ORB-SLAM3's `EdgeInertialGS::computeError` (`G2oTypes.cc:635-637`), `er = LogSO3(dR.transpose()*Rwb1.transpose()*Rwb2)`, `ev = Rwb1.transpose()*(s*(v2 − v1) − g*dt) − dV` and `ep = Rwb1.transpose()*(s*(p2 − p1 − v1*dt) − g*dt*dt/2) − dP`, with `g = Rwg*gI`, written in Basalt's rotation convention, which is the group inverse of ORB-SLAM3's (`context/vio_residuals_and_priors.md`). At $\sigma = 0$ and $\mathbf{R}_{wg} = \mathbf{I}$ it reduces exactly to `residual`. The bias random walk rows of `ImuBlock::linearizeImu` (`include/basalt/linearization/imu_block.hpp:73-105`) involve body frame biases only and are unchanged. The whitening by `get_sqrt_cov_inv()` is unchanged, because the covariance belongs to the metric preintegration.

#### 2.3.2 Jacobians with respect to the states

Every Jacobian of this section and the next is obtained by the same three moves, applied to one variable at a time with all others held fixed. The variable is perturbed by the update rule of §2.1.2, the perturbed residual is expanded and the unperturbed residual is separated from it, and the Jacobian is identified by (I9) as the matrix that multiplies the increment. The derivation follows `doc/VIO.md` §3.1.2, with the additive dependences now multiplied by $s$. Three abbreviations, the scaled counterparts of `tmp` and `tmp2` at `preintegration.h:237-249`, keep the expressions short,

$$\mathbf{u} = s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}_D\Delta t^2,$$

$$\mathbf{t}_s = \mathbf{R}_0^\top \mathbf{u},$$

$$\mathbf{t}_{2,s} = \mathbf{R}_0^\top\big(s(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D\Delta t\big),$$

so that the two translational rows read

$$\mathbf{r}_p = \mathbf{t}_s - \hat{\Delta\mathbf{p}},$$

$$\mathbf{r}_v = \mathbf{t}_{2,s} - \hat{\Delta\mathbf{v}}.$$

Position of the first state. Step 1. Under $\mathbf{p}_0 \to \mathbf{p}_0 + \delta\mathbf{p}_0$ the position row becomes

$$\mathbf{r}_p(\delta\mathbf{p}_0) = \mathbf{R}_0^\top\Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \delta\mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}_D\Delta t^2\Big) - \hat{\Delta\mathbf{p}}.$$

Step 2. Distributing $s$ and then $\mathbf{R}_0^\top$ separates the term in $\delta\mathbf{p}_0$,

$$\mathbf{r}_p(\delta\mathbf{p}_0) = \mathbf{R}_0^\top\Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}_D\Delta t^2\Big) - \hat{\Delta\mathbf{p}} - s\,\mathbf{R}_0^\top\delta\mathbf{p}_0.$$

Step 3. The first two terms are the unperturbed $\mathbf{r}_p$,

$$\mathbf{r}_p(\delta\mathbf{p}_0) = \mathbf{r}_p - s\,\mathbf{R}_0^\top\,\delta\mathbf{p}_0.$$

Step 4. By (I9), with no remainder because the dependence is linear,

$$\frac{\partial\mathbf{r}_p}{\partial\delta\mathbf{p}_0} = -s\,\mathbf{R}_0^\top.$$

$\mathbf{r}_v$ and $\mathbf{r}_R$ contain no position, so their Jacobians with respect to $\delta\mathbf{p}_0$ and $\delta\mathbf{p}_1$ are zero.

Position of the second state. Under $\mathbf{p}_1 \to \mathbf{p}_1 + \delta\mathbf{p}_1$, Steps 1 to 3 give

$$\mathbf{r}_p(\delta\mathbf{p}_1) = \mathbf{R}_0^\top\Big(s(\mathbf{p}_1 + \delta\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}_D\Delta t^2\Big) - \hat{\Delta\mathbf{p}},$$

$$\mathbf{r}_p(\delta\mathbf{p}_1) = \mathbf{r}_p + s\,\mathbf{R}_0^\top\,\delta\mathbf{p}_1,$$

and by (I9)

$$\frac{\partial\mathbf{r}_p}{\partial\delta\mathbf{p}_1} = s\,\mathbf{R}_0^\top.$$

Velocity of the first state in the position row. Under $\mathbf{v}_0 \to \mathbf{v}_0 + \delta\mathbf{v}_0$,

$$\mathbf{r}_p(\delta\mathbf{v}_0) = \mathbf{R}_0^\top\Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t - \delta\mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}_D\Delta t^2\Big) - \hat{\Delta\mathbf{p}},$$

$$\mathbf{r}_p(\delta\mathbf{v}_0) = \mathbf{r}_p - s\,\Delta t\,\mathbf{R}_0^\top\,\delta\mathbf{v}_0,$$

and by (I9)

$$\frac{\partial\mathbf{r}_p}{\partial\delta\mathbf{v}_0} = -s\,\mathbf{R}_0^\top\Delta t.$$

$\mathbf{r}_p$ does not contain $\mathbf{v}_1$, so its Jacobian with respect to $\delta\mathbf{v}_1$ is zero.

Velocity of the first state in the velocity row. Under $\mathbf{v}_0 \to \mathbf{v}_0 + \delta\mathbf{v}_0$,

$$\mathbf{r}_v(\delta\mathbf{v}_0) = \mathbf{R}_0^\top\big(s(\mathbf{v}_1 - \mathbf{v}_0 - \delta\mathbf{v}_0) - \mathbf{g}_D\Delta t\big) - \hat{\Delta\mathbf{v}},$$

$$\mathbf{r}_v(\delta\mathbf{v}_0) = \mathbf{r}_v - s\,\mathbf{R}_0^\top\,\delta\mathbf{v}_0,$$

and by (I9)

$$\frac{\partial\mathbf{r}_v}{\partial\delta\mathbf{v}_0} = -s\,\mathbf{R}_0^\top.$$

Velocity of the second state in the velocity row. Under $\mathbf{v}_1 \to \mathbf{v}_1 + \delta\mathbf{v}_1$,

$$\mathbf{r}_v(\delta\mathbf{v}_1) = \mathbf{R}_0^\top\big(s(\mathbf{v}_1 + \delta\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D\Delta t\big) - \hat{\Delta\mathbf{v}},$$

$$\mathbf{r}_v(\delta\mathbf{v}_1) = \mathbf{r}_v + s\,\mathbf{R}_0^\top\,\delta\mathbf{v}_1,$$

and by (I9)

$$\frac{\partial\mathbf{r}_v}{\partial\delta\mathbf{v}_1} = s\,\mathbf{R}_0^\top.$$

Rotation of the first state in the position row. Step 1. Under $\mathbf{R}_0 \to \text{Exp}(\delta\boldsymbol{\varphi}_0)\,\mathbf{R}_0$, the transpose of the product reverses its factors and (I8) turns the transposed exponential into one of the negated vector,

$$\mathbf{R}_0^\top \to \mathbf{R}_0^\top\,\text{Exp}(\delta\boldsymbol{\varphi}_0)^\top = \mathbf{R}_0^\top\,\text{Exp}(-\delta\boldsymbol{\varphi}_0).$$

Step 2. Expanding the exponential to first order by (I1) with $\boldsymbol{\omega} = -\delta\boldsymbol{\varphi}_0$ and (I3) with $\lambda = -1$,

$$\mathbf{R}_0^\top\,\text{Exp}(-\delta\boldsymbol{\varphi}_0) = \mathbf{R}_0^\top\big(\mathbf{I} - [\delta\boldsymbol{\varphi}_0]_\times\big) + O(|\delta\boldsymbol{\varphi}_0|^2).$$

Step 3. $\mathbf{u}$ does not contain $\mathbf{R}_0$, so only the factor in front of it changes. Distributing over the bracket,

$$\mathbf{t}_s(\delta\boldsymbol{\varphi}_0) = \mathbf{R}_0^\top\big(\mathbf{I} - [\delta\boldsymbol{\varphi}_0]_\times\big)\,\mathbf{u} = \mathbf{R}_0^\top \mathbf{u} - \mathbf{R}_0^\top[\delta\boldsymbol{\varphi}_0]_\times\,\mathbf{u}.$$

Step 4. Inserting $\mathbf{R}_0 \mathbf{R}_0^\top = \mathbf{I}$ in front of $\mathbf{u}$ and using $\mathbf{R}_0^\top \mathbf{u} = \mathbf{t}_s$,

$$\mathbf{R}_0^\top[\delta\boldsymbol{\varphi}_0]_\times\,\mathbf{u} = \mathbf{R}_0^\top[\delta\boldsymbol{\varphi}_0]_\times \mathbf{R}_0\,\mathbf{R}_0^\top \mathbf{u} = \mathbf{R}_0^\top[\delta\boldsymbol{\varphi}_0]_\times \mathbf{R}_0\,\mathbf{t}_s.$$

Step 5. By (I4) with $\mathbf{R} = \mathbf{R}_0^\top$ and $\mathbf{a} = \delta\boldsymbol{\varphi}_0$, the conjugated hat is the hat of the rotated vector,

$$\mathbf{R}_0^\top[\delta\boldsymbol{\varphi}_0]_\times \mathbf{R}_0 = [\mathbf{R}_0^\top\delta\boldsymbol{\varphi}_0]_\times.$$

Step 6. By (I2) with $\mathbf{a} = \mathbf{R}_0^\top\delta\boldsymbol{\varphi}_0$ and $\mathbf{b} = \mathbf{t}_s$, the two factors are swapped so that the increment stands on the right,

$$[\mathbf{R}_0^\top\delta\boldsymbol{\varphi}_0]_\times\,\mathbf{t}_s = -[\mathbf{t}_s]_\times\,\mathbf{R}_0^\top\,\delta\boldsymbol{\varphi}_0.$$

Step 7. Substituting Steps 4 to 6 into Step 3,

$$\mathbf{t}_s(\delta\boldsymbol{\varphi}_0) = \mathbf{t}_s + [\mathbf{t}_s]_\times\,\mathbf{R}_0^\top\,\delta\boldsymbol{\varphi}_0 + O(|\delta\boldsymbol{\varphi}_0|^2).$$

Step 8. $\hat{\Delta\mathbf{p}}$ does not contain $\mathbf{R}_0$, so subtracting it from both sides gives $\mathbf{r}_p(\delta\boldsymbol{\varphi}_0) = \mathbf{r}_p + [\mathbf{t}_s]_\times \mathbf{R}_0^\top\,\delta\boldsymbol{\varphi}_0 + O(|\delta\boldsymbol{\varphi}_0|^2)$, and by (I9)

$$\frac{\partial\mathbf{r}_p}{\partial\delta\boldsymbol{\varphi}_0} = [\mathbf{t}_s]_\times\,\mathbf{R}_0^\top.$$

The derivation never used the content of $\mathbf{u}$, so the scale enters only through the value of $\mathbf{t}_s$.

Rotation of the first state in the velocity row. Steps 1 and 2 are unchanged. Steps 3 to 7 repeat with $s(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D\Delta t$ in place of $\mathbf{u}$ and $\mathbf{t}_{2,s}$ in place of $\mathbf{t}_s$,

$$\mathbf{t}_{2,s}(\delta\boldsymbol{\varphi}_0) = \mathbf{t}_{2,s} - \mathbf{R}_0^\top[\delta\boldsymbol{\varphi}_0]_\times \mathbf{R}_0\,\mathbf{t}_{2,s},$$

$$\mathbf{t}_{2,s}(\delta\boldsymbol{\varphi}_0) = \mathbf{t}_{2,s} - [\mathbf{R}_0^\top\delta\boldsymbol{\varphi}_0]_\times\,\mathbf{t}_{2,s},$$

$$\mathbf{t}_{2,s}(\delta\boldsymbol{\varphi}_0) = \mathbf{t}_{2,s} + [\mathbf{t}_{2,s}]_\times\,\mathbf{R}_0^\top\,\delta\boldsymbol{\varphi}_0,$$

each to first order, and by (I9)

$$\frac{\partial\mathbf{r}_v}{\partial\delta\boldsymbol{\varphi}_0} = [\mathbf{t}_{2,s}]_\times\,\mathbf{R}_0^\top.$$

$\mathbf{r}_p$ and $\mathbf{r}_v$ do not contain $\mathbf{R}_1$, so their Jacobians with respect to $\delta\boldsymbol{\varphi}_1$ are zero.

Rotation of the first state in the rotation row. $\mathbf{r}_R$ contains neither $s$ nor $\mathbf{g}_D$, so this and the next derivation are those of `doc/VIO.md` §3.1.2, written out here step by step. Write $\mathbf{A} = \text{Exp}(\boldsymbol{\delta}_g[4{:}6])\,\Delta\mathbf{R}\,\mathbf{R}_1^\top$, as `doc/VIO.md` does, so that $\mathbf{r}_R = \text{Log}(\mathbf{A} \mathbf{R}_0)$. Step 1. Under $\mathbf{R}_0 \to \text{Exp}(\delta\boldsymbol{\varphi}_0)\,\mathbf{R}_0$,

$$\mathbf{r}_R(\delta\boldsymbol{\varphi}_0) = \text{Log}\big(\mathbf{A}\,\text{Exp}(\delta\boldsymbol{\varphi}_0)\,\mathbf{R}_0\big).$$

Step 2. Inserting $\mathbf{R}_0 \mathbf{R}_0^\top = \mathbf{I}$ between $\mathbf{A}$ and the exponential,

$$\mathbf{A}\,\text{Exp}(\delta\boldsymbol{\varphi}_0)\,\mathbf{R}_0 = \mathbf{A} \mathbf{R}_0\,\mathbf{R}_0^\top\,\text{Exp}(\delta\boldsymbol{\varphi}_0)\,\mathbf{R}_0.$$

Step 3. By (I4) with $\mathbf{R} = \mathbf{R}_0^\top$ and $\mathbf{a} = \delta\boldsymbol{\varphi}_0$,

$$\mathbf{R}_0^\top\,\text{Exp}(\delta\boldsymbol{\varphi}_0)\,\mathbf{R}_0 = \text{Exp}(\mathbf{R}_0^\top\delta\boldsymbol{\varphi}_0).$$

Step 4. The perturbation now stands on the right of the unperturbed argument $\mathbf{A} \mathbf{R}_0$,

$$\mathbf{r}_R(\delta\boldsymbol{\varphi}_0) = \text{Log}\big(\mathbf{A} \mathbf{R}_0\,\text{Exp}(\mathbf{R}_0^\top\delta\boldsymbol{\varphi}_0)\big).$$

Step 5. By (I10) with $\mathbf{C} = \mathbf{A} \mathbf{R}_0$, whose logarithm is $\mathbf{r}_R$, and $\boldsymbol{\epsilon} = \mathbf{R}_0^\top\delta\boldsymbol{\varphi}_0$,

$$\mathbf{r}_R(\delta\boldsymbol{\varphi}_0) = \mathbf{r}_R + \mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top\,\delta\boldsymbol{\varphi}_0 + O(|\delta\boldsymbol{\varphi}_0|^2),$$

and by (I9)

$$\frac{\partial\mathbf{r}_R}{\partial\delta\boldsymbol{\varphi}_0} = \mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top.$$

Rotation of the second state in the rotation row. Write $\mathbf{B} = \text{Exp}(\boldsymbol{\delta}_g[4{:}6])\,\Delta\mathbf{R}$, as `doc/VIO.md` does, so that $\mathbf{r}_R = \text{Log}(\mathbf{B} \mathbf{R}_1^\top \mathbf{R}_0)$. Step 1. Under $\mathbf{R}_1 \to \text{Exp}(\delta\boldsymbol{\varphi}_1)\,\mathbf{R}_1$, the transpose reverses the factors and (I8) negates the exponential, as in Step 1 of the position row,

$$\mathbf{r}_R(\delta\boldsymbol{\varphi}_1) = \text{Log}\big(\mathbf{B}\,\mathbf{R}_1^\top\,\text{Exp}(-\delta\boldsymbol{\varphi}_1)\,\mathbf{R}_0\big).$$

Step 2. Inserting $\mathbf{R}_0 \mathbf{R}_0^\top = \mathbf{I}$ between $\mathbf{R}_1^\top$ and the exponential,

$$\mathbf{R}_1^\top\,\text{Exp}(-\delta\boldsymbol{\varphi}_1)\,\mathbf{R}_0 = \mathbf{R}_1^\top \mathbf{R}_0\,\mathbf{R}_0^\top\,\text{Exp}(-\delta\boldsymbol{\varphi}_1)\,\mathbf{R}_0.$$

Step 3. By (I4) with $\mathbf{R} = \mathbf{R}_0^\top$ and $\mathbf{a} = -\delta\boldsymbol{\varphi}_1$,

$$\mathbf{R}_0^\top\,\text{Exp}(-\delta\boldsymbol{\varphi}_1)\,\mathbf{R}_0 = \text{Exp}(-\mathbf{R}_0^\top\delta\boldsymbol{\varphi}_1).$$

Step 4. The perturbation now stands on the right of the unperturbed argument $\mathbf{B} \mathbf{R}_1^\top \mathbf{R}_0$,

$$\mathbf{r}_R(\delta\boldsymbol{\varphi}_1) = \text{Log}\big(\mathbf{B} \mathbf{R}_1^\top \mathbf{R}_0\,\text{Exp}(-\mathbf{R}_0^\top\delta\boldsymbol{\varphi}_1)\big).$$

Step 5. By (I10) with $\mathbf{C} = \mathbf{B} \mathbf{R}_1^\top \mathbf{R}_0$ and $\boldsymbol{\epsilon} = -\mathbf{R}_0^\top\delta\boldsymbol{\varphi}_1$,

$$\mathbf{r}_R(\delta\boldsymbol{\varphi}_1) = \mathbf{r}_R - \mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top\,\delta\boldsymbol{\varphi}_1 + O(|\delta\boldsymbol{\varphi}_1|^2),$$

and by (I9)

$$\frac{\partial\mathbf{r}_R}{\partial\delta\boldsymbol{\varphi}_1} = -\mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top.$$

$\mathbf{r}_R$ contains no position or velocity, so its Jacobians with respect to them are zero. The blocks are therefore

| row \ column | $\delta\mathbf{p}_0$ | $\delta\boldsymbol{\varphi}_0$ | $\delta\mathbf{v}_0$ | $\delta\mathbf{p}_1$ | $\delta\boldsymbol{\varphi}_1$ | $\delta\mathbf{v}_1$ |
|---|---|---|---|---|---|---|
| $\mathbf{r}_p$ | $-s\,\mathbf{R}_0^\top$ | $[\mathbf{t}_s]_\times \mathbf{R}_0^\top$ | $-s\,\mathbf{R}_0^\top\Delta t$ | $s\,\mathbf{R}_0^\top$ | $\mathbf{0}$ | $\mathbf{0}$ |
| $\mathbf{r}_R$ | $\mathbf{0}$ | $\mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top$ | $\mathbf{0}$ | $\mathbf{0}$ | $-\mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top$ | $\mathbf{0}$ |
| $\mathbf{r}_v$ | $\mathbf{0}$ | $[\mathbf{t}_{2,s}]_\times \mathbf{R}_0^\top$ | $-s\,\mathbf{R}_0^\top$ | $\mathbf{0}$ | $\mathbf{0}$ | $s\,\mathbf{R}_0^\top$ |

and the bias columns `d_res_d_bg` and `d_res_d_ba` (`preintegration.h:278-290`) are unchanged, since the bias corrections enter outside every bracket that $s$ multiplies. ORB-SLAM3's `EdgeInertialGS::linearizeOplus` shows the same structure, `-s*Rbw1` for the first velocity and `s*Rbw1*Rwb2` for the second translation (`G2oTypes.cc:680-707`), allowing for its right perturbation of the pose.

#### 2.3.3 Jacobians with respect to gravity direction and scale

The same three moves apply. The gravity direction is handled in two stages, first with respect to the vector $\mathbf{g}_D$ and then, by substituting the expansion of §2.1.3, with respect to $\boldsymbol{\delta}_\theta$.

Gravity vector in the position row. Step 1. Under $\mathbf{g}_D \to \mathbf{g}_D + \delta\mathbf{g}$,

$$\mathbf{r}_p(\delta\mathbf{g}) = \mathbf{R}_0^\top\Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2}(\mathbf{g}_D + \delta\mathbf{g})\Delta t^2\Big) - \hat{\Delta\mathbf{p}}.$$

Step 2. Distributing $\frac{1}{2}\Delta t^2$ and then $\mathbf{R}_0^\top$ separates the unperturbed residual,

$$\mathbf{r}_p(\delta\mathbf{g}) = \mathbf{r}_p - \frac{1}{2}\Delta t^2\,\mathbf{R}_0^\top\,\delta\mathbf{g}.$$

Step 3. By (I9), exactly,

$$\frac{\partial\mathbf{r}_p}{\partial\mathbf{g}_D} = -\frac{1}{2}\Delta t^2\,\mathbf{R}_0^\top.$$

Gravity vector in the velocity row. The same steps give

$$\mathbf{r}_v(\delta\mathbf{g}) = \mathbf{R}_0^\top\big(s(\mathbf{v}_1 - \mathbf{v}_0) - (\mathbf{g}_D + \delta\mathbf{g})\Delta t\big) - \hat{\Delta\mathbf{v}},$$

$$\mathbf{r}_v(\delta\mathbf{g}) = \mathbf{r}_v - \Delta t\,\mathbf{R}_0^\top\,\delta\mathbf{g},$$

$$\frac{\partial\mathbf{r}_v}{\partial\mathbf{g}_D} = -\Delta t\,\mathbf{R}_0^\top.$$

$\mathbf{r}_R$ does not contain $\mathbf{g}_D$, so $\partial\mathbf{r}_R/\partial\mathbf{g}_D = \mathbf{0}$.

Gravity direction in the position row. Step 1. At the linearisation point $\boldsymbol{\delta}_\theta = \mathbf{0}$ the gravity vector is $\bar{\mathbf{g}}_D$, and §2.1.3 gives its first order change under an increment $\boldsymbol{\delta}_\theta$,

$$\delta\mathbf{g} = \mathbf{g}_D(\boldsymbol{\delta}_\theta) - \bar{\mathbf{g}}_D = -[\bar{\mathbf{g}}_D]_\times\,\bar{\mathbf{B}}\,\boldsymbol{\delta}_\theta + O(|\boldsymbol{\delta}_\theta|^2).$$

Step 2. Substituting this $\delta\mathbf{g}$ into Step 2 of the gravity vector derivation,

$$\mathbf{r}_p(\boldsymbol{\delta}_\theta) = \mathbf{r}_p - \frac{1}{2}\Delta t^2\,\mathbf{R}_0^\top\big(-[\bar{\mathbf{g}}_D]_\times\,\bar{\mathbf{B}}\,\boldsymbol{\delta}_\theta\big) + O(|\boldsymbol{\delta}_\theta|^2).$$

Step 3. The two minus signs cancel,

$$\mathbf{r}_p(\boldsymbol{\delta}_\theta) = \mathbf{r}_p + \frac{1}{2}\Delta t^2\,\mathbf{R}_0^\top[\bar{\mathbf{g}}_D]_\times\,\bar{\mathbf{B}}\,\boldsymbol{\delta}_\theta + O(|\boldsymbol{\delta}_\theta|^2).$$

Step 4. By (I9),

$$\frac{\partial\mathbf{r}_p}{\partial\boldsymbol{\delta}_\theta} = \frac{1}{2}\Delta t^2\,\mathbf{R}_0^\top[\bar{\mathbf{g}}_D]_\times\,\bar{\mathbf{B}}.$$

Gravity direction in the velocity row. Substituting the same $\delta\mathbf{g}$ into the velocity row's expansion,

$$\mathbf{r}_v(\boldsymbol{\delta}_\theta) = \mathbf{r}_v - \Delta t\,\mathbf{R}_0^\top\big(-[\bar{\mathbf{g}}_D]_\times\,\bar{\mathbf{B}}\,\boldsymbol{\delta}_\theta\big) + O(|\boldsymbol{\delta}_\theta|^2),$$

$$\mathbf{r}_v(\boldsymbol{\delta}_\theta) = \mathbf{r}_v + \Delta t\,\mathbf{R}_0^\top[\bar{\mathbf{g}}_D]_\times\,\bar{\mathbf{B}}\,\boldsymbol{\delta}_\theta + O(|\boldsymbol{\delta}_\theta|^2),$$

$$\frac{\partial\mathbf{r}_v}{\partial\boldsymbol{\delta}_\theta} = \Delta t\,\mathbf{R}_0^\top[\bar{\mathbf{g}}_D]_\times\,\bar{\mathbf{B}}.$$

$\partial\mathbf{r}_R/\partial\boldsymbol{\delta}_\theta = \mathbf{0}$, since $\mathbf{r}_R$ does not contain $\mathbf{g}_D$. In both rows the result is the gravity vector Jacobian multiplied on the right by $\partial\mathbf{g}_D/\partial\boldsymbol{\delta}_\theta = -[\bar{\mathbf{g}}_D]_\times\bar{\mathbf{B}}$, which is the chain rule. `residualGS` returns $\partial\mathbf{r}/\partial\mathbf{g}_D$ rather than $\partial\mathbf{r}/\partial\boldsymbol{\delta}_\theta$, so that the preintegration header stays independent of how the direction is parameterised, and `ImuBlockGS` applies the multiplication by $-[\bar{\mathbf{g}}_D]_\times\bar{\mathbf{B}}$.

Log scale in the position row. Step 1. Under $\sigma \to \sigma + \varepsilon$ the scale becomes $s\,e^{\varepsilon}$, and the first order expansion of the exponential gives

$$s\,e^{\varepsilon} = s + \varepsilon\,s + O(\varepsilon^2).$$

Step 2. Substituting into the position row,

$$\mathbf{r}_p(\varepsilon) = \mathbf{R}_0^\top\Big((s + \varepsilon\,s)(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}_D\Delta t^2\Big) - \hat{\Delta\mathbf{p}} + O(\varepsilon^2).$$

Step 3. Distributing $(s + \varepsilon\,s)$ and then $\mathbf{R}_0^\top$ separates the unperturbed residual,

$$\mathbf{r}_p(\varepsilon) = \mathbf{r}_p + \varepsilon\,s\,\mathbf{R}_0^\top(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) + O(\varepsilon^2).$$

Step 4. By (I9),

$$\frac{\partial\mathbf{r}_p}{\partial\sigma} = s\,\mathbf{R}_0^\top(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t).$$

Log scale in the velocity row. The same steps give

$$\mathbf{r}_v(\varepsilon) = \mathbf{R}_0^\top\big((s + \varepsilon\,s)(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D\Delta t\big) - \hat{\Delta\mathbf{v}} + O(\varepsilon^2),$$

$$\mathbf{r}_v(\varepsilon) = \mathbf{r}_v + \varepsilon\,s\,\mathbf{R}_0^\top(\mathbf{v}_1 - \mathbf{v}_0) + O(\varepsilon^2),$$

$$\frac{\partial\mathbf{r}_v}{\partial\sigma} = s\,\mathbf{R}_0^\top(\mathbf{v}_1 - \mathbf{v}_0).$$

$\mathbf{r}_R$ does not contain $s$, so $\partial\mathbf{r}_R/\partial\sigma = \mathbf{0}$.

These are ORB-SLAM3's `_jacobianOplus[6]` and `_jacobianOplus[7]` (`G2oTypes.cc:710-717`) in Basalt's conventions, where ORB-SLAM3's gravity block `−Rbw1*dGdTheta*dt` with `dGdTheta = Rwg*Gm` is the right perturbation counterpart of $\Delta t\,\mathbf{R}_0^\top[\bar{\mathbf{g}}_D]_\times\bar{\mathbf{B}}$. With $\bar{\mathbf{d}} = \hat{\mathbf{g}}$, $\bar{\mathbf{B}} = [\mathbf{e}_x, \mathbf{e}_y]$ and $\mathbf{R}_{wg} = \mathbf{I}$, the two agree to first order. The nine row block is whitened by `get_sqrt_cov_inv()` like every other inertial column, and the six bias rows have zero gravity and scale columns.

#### 2.3.4 Prediction

The state propagation used for the new frame in `measure` (`sqrt_keypoint_vio.cpp:653-673`) runs before the frame's optimisation, when the previous frame's correction has already been applied and reset (§2.7), so $s = 1$ and $\mathbf{g}_D = \mathbf{g}$. `residualGS` then equals `residual`, and the existing `predictState` (`preintegration.h:191-202`) already satisfies it at zero bias drift. No gravity and scale prediction is needed. The same holds for triangulation in `measure`, which also runs before `optimize`.

### 2.4 Objective, Linearisation and the Dense System

#### 2.4.1 Objective

The objective of `context/vio_opt.md` §5 becomes

$$F(\mathbf{s}) = \sum_\ell\sum_{\text{obs}} \frac{\rho_H(|\mathbf{r}_{\ell,\text{obs}}(s)|)}{\sigma_\text{obs}^2} + \sum_k \frac{1}{2}\big|\mathbf{L}_k \mathbf{r}^{GS}_k(\mathbf{R}_{wg}, s)\big|^2 + E_\text{bias} + \frac{1}{2}\big|\mathbf{J}_m(\boldsymbol{\delta} + \mathbf{x}) + \mathbf{r}_m\big|^2 + E_\text{zv},$$

with the visual term of §2.2, the inertial term of §2.3, the bias random walk $E_\text{bias}$ and zero velocity term $E_\text{zv}$ unchanged, and the prior now including the gravity and scale columns, §2.6. The cost bookkeeping triple of `context/vio_opt.md` §9.3 must include every new dependence in all three of `error_total`, `l_diff` and `after_error_total`, which §3.2 does by making the after step recomputation use the gravity and scale variants of `computeError` and `computeImuError`.

#### 2.4.2 Landmark elimination is unchanged

Landmarks still appear in no factor other than their own observations, so the per landmark Householder QR of `context/vio_opt.md` §7.3 applies verbatim. The gravity and scale block is simply three more pose like columns in each landmark block's storage, populated with $\frac{\sqrt{w}}{\sigma_\text{obs}}\,\frac{\partial\mathbf{r}}{\partial\sigma}$ in the scale column and zeros in the two tilt columns. `LandmarkBlockAbsDynamic` already stores `aom.total_size` pose columns (`landmark_block_abs_dynamic.hpp:85-87`), so the new columns exist in storage the moment the block appears in `aom`. The QR rotates them with every other column, `get_dense_H_b` and `get_dense_Q2Jp_Q2r` export them, and `backSubstitute` includes them in $\mathbf{Q}_1^\top \mathbf{J}_p\,\delta\mathbf{p}$ and in `l_diff` because it multiplies the whole of `storage.topLeftCorner(num_rows − 3, padding_idx)` by the full increment (`:313-320`). No change to the QR, the export or the back substitution is needed.

#### 2.4.3 Dense structure

With $P$ keyframe poses and three states, the reduced system acquires a three column border.

```
             gs (3)        poses (6P)        s0 (15)        s1 (15)        s2 (15)
gs       [ imu + vis(σ)   vis(σ)+prior     imu(0,1)+prior  imu(0,1)+imu(1,2) imu(1,2) ]
poses    [                vis + prior      vis + prior     vis              vis      ]
s0       [                                 vis+imu+prior   imu(0,1)         vis      ]
s1       [                                                 imu(0,1)+imu(1,2) imu(1,2)]
s2       [                                                                  imu(1,2) ]
```

The gravity and scale block couples to every state through the inertial factors and, through the lever arm term of §2.2.4, to every frame that observes a landmark. The additional dense work is three rows and columns on a system of about eighty, which is negligible against the landmark linearisation.

### 2.5 Gauge Freedom and Its Removal

This section explains what a gauge freedom is, which gauge freedoms Basalt has today, which three the gravity and scale block adds, and how the plan removes them from each solve. It first fixes the meaning of the linear system $\mathbf{H}$, $\mathbf{b}$ that every later step acts on.

#### 2.5.1 The linear system of one iteration

Collect every residual of §2.4.1, visual, inertial, bias, prior and zero velocity, into one whitened vector $\mathbf{r}(\mathbf{x})$, so that the cost is $F = \frac{1}{2}|\mathbf{r}|^2$. Each Levenberg–Marquardt iteration expands it to first order by (I9) about the current estimate, for an increment $\boldsymbol{\delta}$ of all variables,

$$\mathbf{r}(\boldsymbol{\delta}) = \mathbf{r} + \mathbf{J}\boldsymbol{\delta} + O(|\boldsymbol{\delta}|^2),$$

where $\mathbf{J}$ stacks the Jacobians of every residual. Substituting into the cost gives the quadratic model

$$F(\boldsymbol{\delta}) \approx \frac{1}{2}|\mathbf{r} + \mathbf{J}\boldsymbol{\delta}|^2 = \frac{1}{2} \mathbf{r}^\top \mathbf{r} + \boldsymbol{\delta}^\top \mathbf{J}^\top \mathbf{r} + \frac{1}{2}\boldsymbol{\delta}^\top \mathbf{J}^\top \mathbf{J}\,\boldsymbol{\delta},$$

whose two coefficient matrices are named

$$\mathbf{H} = \mathbf{J}^\top \mathbf{J},$$

$$\mathbf{b} = \mathbf{J}^\top \mathbf{r}.$$

$\mathbf{H}$ is the Gauss–Newton approximation of the Hessian of $F$ and $\mathbf{b}$ is its gradient. Setting the derivative of the model with respect to $\boldsymbol{\delta}$ to zero gives the normal equations

$$\mathbf{H}\boldsymbol{\delta} = -\mathbf{b}.$$

Basalt never forms $\mathbf{H}$ over the landmarks. The per landmark QR of `context/vio_opt.md` §7.3 eliminates them first, which is algebraically the Schur complement, so the $\mathbf{H}$ and $\mathbf{b}$ that `get_dense_H_b` returns (`sqrt_keypoint_vio.cpp:1749`) are over the frame variables and, during the period, the gravity and scale block only. They are summed from each block's $\mathbf{J}^\top \mathbf{J}$ and $\mathbf{J}^\top \mathbf{r}$, `H += J.transpose() * J` and `b += J.transpose() * r` (`landmark_block_abs_dynamic.hpp:523-524`, `imu_block.hpp:135-136`). `optimize` then adds the Marquardt damping $\mathbf{D} = \max(\lambda\,\text{diag}\,\mathbf{H},\ \lambda_\text{min})$ to the diagonal, solves $(\mathbf{H} + \mathbf{D})\,\mathbf{x} = \mathbf{b}$ by LDLT, and applies the step $\boldsymbol{\delta} = -\mathbf{x}$ (`sqrt_keypoint_vio.cpp:1775-1782`, `:1811`). $\mathbf{x}$ below always means this solution, the negated step.

#### 2.5.2 What a gauge freedom is

A gauge freedom is a continuous family of transformations of the whole state that changes no residual. It is a change in how the state is written down, not a change in the physical situation, so no measurement can distinguish one member of the family from another. The standard examples are the choice of origin and orientation of the world frame. Moving every pose and landmark by the same rigid motion describes the same scene, and the camera images and the IMU samples are exactly the same.

Formally, let $\mathbf{x}(\varepsilon)$ be such a family through the current estimate, with $\mathbf{x}(0)$ the estimate, and let $\mathbf{n}$ be its tangent direction, so that $\mathbf{x}(\varepsilon)$ equals the estimate perturbed by $\varepsilon\mathbf{n} + O(\varepsilon^2)$. Every residual is unchanged along the family,

$$\mathbf{r}\big(\mathbf{x}(\varepsilon)\big) = \mathbf{r}\big(\mathbf{x}(0)\big) \quad \text{for every } \varepsilon.$$

Step 1. Expanding the left hand side by (I9) with the increment $\varepsilon\mathbf{n}$,

$$\mathbf{r}\big(\mathbf{x}(0)\big) + \varepsilon\,\mathbf{J} \mathbf{n} + O(\varepsilon^2) = \mathbf{r}\big(\mathbf{x}(0)\big).$$

Step 2. Subtracting the unperturbed residual and dividing by $\varepsilon$ before letting $\varepsilon \to 0$,

$$\mathbf{J} \mathbf{n} = \mathbf{0}.$$

Step 3. Multiplying on the left by $\mathbf{J}^\top$, and separately by $\mathbf{r}^\top$, and using the definitions of §2.5.1,

$$\mathbf{H} \mathbf{n} = \mathbf{J}^\top \mathbf{J } \mathbf{n} = \mathbf{0},$$

$$\mathbf{b}^\top \mathbf{n} = \mathbf{r}^\top \mathbf{J} \mathbf{n} = 0.$$

A gauge direction is therefore a null vector of $\mathbf{H}$ that is also orthogonal to $\mathbf{b}$. It survives the landmark elimination whenever its landmark components are zero, which is the case for every direction in this section. Writing $\mathbf{H}$ over frames $f$ and landmarks $l$, $\mathbf{n} = [\mathbf{n}_f; \mathbf{0}]$ and $\mathbf{H} \mathbf{n} = \mathbf{0}$ give $\mathbf{H}_{ff} \mathbf{n}_f = \mathbf{0}$ and $\mathbf{H}_{lf} \mathbf{n}_f = \mathbf{0}$, so the reduced matrix also annihilates $\mathbf{n}_f$,

$$\big(\mathbf{H}_{ff} - \mathbf{H}_{fl} \mathbf{H}_{ll}^{-1} \mathbf{H}_{lf}\big)\,\mathbf{n}_f = \mathbf{0} - \mathbf{H}_{fl} \mathbf{H}_{ll}^{-1}\,\mathbf{0} = \mathbf{0}.$$

Three consequences follow, and they are why a gauge freedom must be treated explicitly.

1. The solution is not unique. For any solution $\boldsymbol{\delta}^*$ of the normal equations and any scalar $c$, $\mathbf{H}(\boldsymbol{\delta}^* + c\,\mathbf{n}) = \mathbf{H}\boldsymbol{\delta}^* = -\mathbf{b}$, so $\boldsymbol{\delta}^* + c\,\mathbf{n}$ is a solution too, and by $\mathbf{J} \mathbf{n} = \mathbf{0}$ it predicts exactly the same cost.
2. The matrix is singular, so its factorisation is ill posed. Damping makes $\mathbf{H} + \mathbf{D}$ invertible, but along $\mathbf{n}$ the only stiffness is the damping itself, so the component $c$ of the step is set by $\lambda$ and by round off rather than by any measurement. The estimate can then wander along $\mathbf{n}$ from iteration to iteration and frame to frame at no cost.
3. Any information that $\mathbf{H}$ appears to contain along $\mathbf{n}$ is spurious, because no residual depends on that direction. Such information arises when factors that share a variable are linearised at different points, so that their null vectors differ slightly and no single vector annihilates all of them. The estimator then becomes confident in a quantity no sensor measured. Basalt's first estimate Jacobians exist to prevent this for its own gauge (`doc/VIO.md` §3.1.3), and `checkNullspace` (`src/vi_estimator/sqrt_ba_base.cpp:43-149`) measures it.

A gauge freedom is therefore not something the optimiser estimates badly. It has nothing to estimate. The task is to choose one representative of the family consistently and to keep spurious information out of $\mathbf{H}$.

#### 2.5.3 The gauge of today's Basalt and the three new directions

Today's Basalt VIO has four gauge directions, three global translations and one rotation about gravity. A translation of the world, $\mathbf{p}_f \to \mathbf{p}_f + \mathbf{c}$ for every frame, leaves $\mathbf{T}_{ti,hi}$ of §2.2.1 unchanged because it contains only $\mathbf{p}_h - \mathbf{p}_t$, and leaves the inertial residual of §2.3.1 unchanged because it contains only $\mathbf{p}_1 - \mathbf{p}_0$. A rotation $\mathbf{Q}$ of the world about the vertical maps $\mathbf{R}_f \to \mathbf{Q} \mathbf{R}_f$, $\mathbf{p}_f \to \mathbf{Q} \mathbf{p}_f$ and $\mathbf{v}_f \to \mathbf{Q} \mathbf{v}_f$. Every world vector then enters a residual through $\mathbf{R}_0^\top \mathbf{Q}^\top \mathbf{Q} = \mathbf{R}_0^\top$, except the fixed gravity $\mathbf{g}$, which enters as $\mathbf{R}_0^\top \mathbf{Q}^\top \mathbf{g}$. A rotation about the vertical leaves $\mathbf{g} = (0, 0, -9.81)^\top$ unchanged, $\mathbf{Q}^\top \mathbf{g} = \mathbf{g}$, so yaw is a gauge direction. Yaw has never been observable to Basalt. It is fixed by convention, through the initial prior on index 5 (`sqrt_keypoint_vio.cpp:105-116`), together with the position on indices 0 to 2.

Roll, pitch and scale are observable today, and the fixed constants in the inertial residual are what make them so. A rotation about a horizontal axis changes $\mathbf{Q}^\top \mathbf{g}$, so the velocity row changes by about $\varepsilon\,\Delta t\,\mathbf{R}_0^\top[\mathbf{g}]_\times \mathbf{a}$, which is not zero. A rescaling of positions and velocities by $k$ changes $\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t$ by the factor $k$ while the metric preintegrated $\Delta\mathbf{p}$ stays fixed, so the position row changes too. The accelerometer therefore measures the tilt of the world against gravity and its metric scale.

The gravity and scale block replaces the two constants by variables, $\mathbf{g}$ by $\mathbf{g}_D$ and $1$ by $s$. Physically nothing changes. The accelerometer still measures the tilt and scale of the metric world. What changes is the representation. The same metric world can now be written either through the states with $s = 1$ and $\mathbf{g}_D = \mathbf{g}$, or through states that are scaled and tilted together with a block that undoes the scaling and tilt, and the residuals cannot tell these apart. This adds exactly three gauge directions, which are derived below. Their components are given in `aom` coordinates at the linearisation points, where every Jacobian is evaluated.

A worked example makes this concrete. Take two states one second apart, $\Delta t = 1$ s, of a vehicle that starts at rest at the origin, does not rotate, and accelerates at $2$ m/s² along the metric $x$ axis. In the metric world the states are

$$\mathbf{R}_0 = \mathbf{R}_1 = \mathbf{I},$$

$$\mathbf{p}_0 = (0, 0, 0)^\top,$$

$$\mathbf{v}_0 = (0, 0, 0)^\top,$$

$$\mathbf{p}_1 = (1, 0, 0)^\top,$$

$$\mathbf{v}_1 = (2, 0, 0)^\top.$$

With $\mathbf{g} = (0, 0, -9.81)^\top$ and no bias, the preintegrated measurements of the accelerometer and gyroscope are those the residual of §2.3.1 is built to match at the true states,

$$\Delta\mathbf{R} = \mathbf{I},$$

$$\Delta\mathbf{v} = \mathbf{v}_1 - \mathbf{v}_0 - \mathbf{g}\Delta t = (2, 0, 9.81)^\top,$$

$$\Delta\mathbf{p} = \mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t - \frac{1}{2} \mathbf{g}\Delta t^2 = (1, 0, 4.905)^\top.$$

Representation A is today's. The states are stored as above, with $s = 1$ and $\mathbf{g}_D = \mathbf{g}$. Every residual is zero, for instance

$$\mathbf{r}_v = \mathbf{I}\big(1\cdot(2, 0, 0)^\top - (0, 0, -9.81)^\top\big) - (2, 0, 9.81)^\top = \mathbf{0}.$$

Representation B stores the same motion in a world $D$ that is twice as large and tilted by $90°$ about the horizontal $y$ axis. Choose $k = 2$ and

$$\mathbf{R}_{wg} = \text{Exp}\Big(\frac{\pi}{2}\,\mathbf{e}_y\Big) = \begin{bmatrix}0 & 0 & 1\\ 0 & 1 & 0\\ -1 & 0 & 0\end{bmatrix},$$

which maps $\mathbf{e}_x$ to $-\mathbf{e}_z$ and $\mathbf{e}_z$ to $\mathbf{e}_x$. Every state is transformed by the inverse of the map of §2.1.1, $\mathbf{R} \to \mathbf{R}_{wg} \mathbf{R}$, $\mathbf{p} \to k\,\mathbf{R}_{wg} \mathbf{p}$, $\mathbf{v} \to k\,\mathbf{R}_{wg} \mathbf{v}$, and the block is set to undo it, $s = 1/k$ and $\mathbf{g}_D = \mathbf{R}_{wg} \mathbf{g}$. The stored values become

$$\mathbf{R}_0 = \mathbf{R}_1 = \mathbf{R}_{wg},$$

$$\mathbf{p}_1 = 2\,\mathbf{R}_{wg}(1, 0, 0)^\top = (0, 0, -2)^\top,$$

$$\mathbf{v}_1 = 2\,\mathbf{R}_{wg}(2, 0, 0)^\top = (0, 0, -4)^\top,$$

$$s = \frac{1}{2},$$

$$\sigma = -\ln 2,$$

$$\mathbf{g}_D = \mathbf{R}_{wg}(0, 0, -9.81)^\top = (-9.81, 0, 0)^\top,$$

with $\mathbf{p}_0$ and $\mathbf{v}_0$ still zero. Read literally, representation B describes a vehicle that falls two metres along $-z$ in a world where gravity points along $-x$, measured with a ruler that is twice too long. Its residuals are nonetheless identical to A's. The velocity row is

$$\mathbf{r}_v = \mathbf{R}_{wg}^\top\Big(\frac{1}{2}(0, 0, -4)^\top - (-9.81, 0, 0)^\top\Big) - (2, 0, 9.81)^\top = \mathbf{R}_{wg}^\top(9.81, 0, -2)^\top - (2, 0, 9.81)^\top,$$

and with $\mathbf{R}_{wg}^\top(9.81, 0, -2)^\top = (2, 0, 9.81)^\top$ it is zero. The position row is

$$\mathbf{r}_p = \mathbf{R}_{wg}^\top\Big(\frac{1}{2}(0, 0, -2)^\top - \frac{1}{2}(-9.81, 0, 0)^\top\Big) - (1, 0, 4.905)^\top = \mathbf{R}_{wg}^\top(4.905, 0, -1)^\top - (1, 0, 4.905)^\top = \mathbf{0}.$$

The rotation row contains $\mathbf{R}_1^\top \mathbf{R}_0 = \mathbf{R}_{wg}^\top \mathbf{R}_{wg} = \mathbf{I}$, as in A. For a landmark hosted in frame 0 and seen in frame 1, with the camera at the IMU for simplicity, the visual residual depends on $\mathbf{R}_1^\top \mathbf{R}_0$ and on $s\,\mathbf{t}_{ti,hi}$ (§2.2.1). In A, $s\,\mathbf{t}_{ti,hi} = \mathbf{R}_1^\top(\mathbf{p}_0 - \mathbf{p}_1) = (-1, 0, 0)^\top$. In B,

$$s\,\mathbf{t}_{ti,hi} = \frac{1}{2}\,\mathbf{R}_{wg}^\top\big((0, 0, 0)^\top - (0, 0, -2)^\top\big) = \frac{1}{2}\,\mathbf{R}_{wg}^\top(0, 0, 2)^\top = \frac{1}{2}(-2, 0, 0)^\top = (-1, 0, 0)^\top,$$

the same, so the reprojection is the same with the same landmark. Mapping B back to the metric world by §2.1.1 recovers A exactly, for instance

$$\mathbf{p}^M_1 = s\,\mathbf{R}_{wg}^\top \mathbf{p}_1 = \frac{1}{2}\,\mathbf{R}_{wg}^\top(0, 0, -2)^\top = (1, 0, 0)^\top.$$

Nothing in the measurements prefers A to B, and the same holds for every $k > 0$ and every rotation $\mathbf{R}_{wg}$ about a horizontal axis. That continuous two parameter family of equally good descriptions of one physical motion, scale and two tilt angles, is the gauge freedom the block introduces. Its infinitesimal members, $k = 1 + \varepsilon$ and $\mathbf{R}_{wg} = \text{Exp}(\varepsilon\mathbf{a})$, are the null directions derived next. Without a treatment, an iteration may move the estimate part of the way from A towards B, since the cost is flat in between, and the next iteration may move it back, while the published metric result stays the same. The projection of §2.5.6 removes exactly this motion from every step. The example also shows why yaw was a gauge freedom before the block existed. A rotation about $\mathbf{e}_z$ leaves $\mathbf{g}$ unchanged, so representation B with such a rotation needs no change of $\mathbf{g}_D$ at all.

Scale gauge $\mathbf{n}_\sigma$. For every frame $f$, $\delta\mathbf{p}_f = \mathbf{p}_f$, $\delta\mathbf{v}_f = \mathbf{v}_f$ (states only), and $\delta\sigma = -1$, with every other component zero, landmarks included. Moving by $\varepsilon\,\mathbf{n}_\sigma$ maps $\mathbf{p}_f \to (1 + \varepsilon) \mathbf{p}_f$, $\mathbf{v}_f \to (1 + \varepsilon) \mathbf{v}_f$ and $\sigma \to \sigma - \varepsilon$. Step 1. By the first order expansion of the exponential, the scale becomes

$$s\,e^{-\varepsilon} = s\,(1 - \varepsilon) + O(\varepsilon^2).$$

Step 2. The scaled displacement of the inertial position row becomes

$$s(1 - \varepsilon)(1 + \varepsilon)(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) = s(1 - \varepsilon^2)(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) = s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) + O(\varepsilon^2),$$

and the same computation applies to $s(\mathbf{v}_1 - \mathbf{v}_0)$ in the velocity row. Rotations, biases and $\mathbf{g}_D$ are untouched, so every inertial residual is unchanged to first order. Step 3. The visual residual sees the scale only through $s\,\mathbf{t}_{ti,hi}$ (§2.2.1), and $\mathbf{t}_{ti,hi} = \mathbf{R}_t^\top(\mathbf{p}_h - \mathbf{p}_t)$ scales with the positions, so

$$s(1 - \varepsilon)\,\mathbf{R}_t^\top(1 + \varepsilon)(\mathbf{p}_h - \mathbf{p}_t) = s\,\mathbf{t}_{ti,hi} + O(\varepsilon^2).$$

$\mathbf{T}_{t,h}(s)$ is unchanged to first order and the landmark is untouched, so every visual residual is unchanged. Step 4. By Steps 2 and 3 and (I9), $\mathbf{J} \mathbf{n}_\sigma = \mathbf{0}$.

Tilt gauge $\mathbf{n}_\mathbf{a}$, for $\mathbf{a} \in \{\mathbf{e}_x, \mathbf{e}_y\}$. For every frame, $\delta\boldsymbol{\varphi}_f = \mathbf{a}$, $\delta\mathbf{p}_f = \mathbf{a}\times \mathbf{p}_f$, $\delta\mathbf{v}_f = \mathbf{a}\times \mathbf{v}_f$ (states only), and $\boldsymbol{\delta}_\theta = \bar{\mathbf{B}}^\top \mathbf{a}$, with every other component zero. This is the tangent of rotating the whole of $D$, gravity direction included, by $\mathbf{Q} = \text{Exp}(\varepsilon\mathbf{a})$. Step 1. The frame components follow from (I1), $\mathbf{Q} \mathbf{p}_f = \mathbf{p}_f + \varepsilon\,\mathbf{a}\times \mathbf{p}_f + O(\varepsilon^2)$, likewise for $\mathbf{v}_f$, and $\mathbf{Q} \mathbf{R}_f = \text{Exp}(\varepsilon\mathbf{a})\,\mathbf{R}_f$ is the update of §2.1.2 with $\delta\boldsymbol{\varphi}_f = \varepsilon\mathbf{a}$. Step 2. The block component must move $\bar{\mathbf{d}}$ to $\mathbf{Q}\bar{\mathbf{d}} = \bar{\mathbf{d}} + \varepsilon\,\mathbf{a}\times\bar{\mathbf{d}} + O(\varepsilon^2)$. By §2.1.3 an increment $\boldsymbol{\delta}_\theta$ moves it to $\bar{\mathbf{d}} + (\bar{\mathbf{B}}\boldsymbol{\delta}_\theta)\times\bar{\mathbf{d}} + O(|\boldsymbol{\delta}_\theta|^2)$. Since $\{\bar{\mathbf{b}}_1, \bar{\mathbf{b}}_2, \bar{\mathbf{d}}\}$ is an orthonormal basis, $\mathbf{a}$ decomposes as

$$\mathbf{a} = \bar{\mathbf{B}}\bar{\mathbf{B}}^\top \mathbf{a} + \bar{\mathbf{d}}\,\bar{\mathbf{d}}^\top \mathbf{a},$$

and the second part contributes nothing to the motion because $\bar{\mathbf{d}}\times\bar{\mathbf{d}} = \mathbf{0}$,

$$\mathbf{a}\times\bar{\mathbf{d}} = (\bar{\mathbf{B}}\bar{\mathbf{B}}^\top \mathbf{a})\times\bar{\mathbf{d}}.$$

Matching the two expressions gives $\boldsymbol{\delta}_\theta = \varepsilon\,\bar{\mathbf{B}}^\top \mathbf{a}$. Step 3. Under the rotation every world vector in the inertial residual is multiplied by $\mathbf{Q}$, now including $\mathbf{g}_D \to \mathbf{Q} \mathbf{g}_D$, and every one meets $\mathbf{R}_0^\top \mathbf{Q}^\top$,

$$\mathbf{R}_0^\top \mathbf{Q}^\top\Big(s\,\mathbf{Q}(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{Q} \mathbf{g}_D\Delta t^2\Big) = \mathbf{R}_0^\top\Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2} \mathbf{g}_D\Delta t^2\Big),$$

by $\mathbf{Q}^\top \mathbf{Q} = \mathbf{I}$, likewise for the velocity row, and the rotation row contains $\mathbf{R}_1^\top \mathbf{Q}^\top \mathbf{Q} \mathbf{R}_0 = \mathbf{R}_1^\top \mathbf{R}_0$. Step 4. The visual residual contains $\mathbf{T}_{ti,hi} = (\mathbf{R}_t^\top \mathbf{Q}^\top \mathbf{Q} \mathbf{R}_h,\ \mathbf{R}_t^\top \mathbf{Q}^\top \mathbf{Q}(\mathbf{p}_h - \mathbf{p}_t))$, which is unchanged. Step 5. Every residual is unchanged, exactly and for every $\varepsilon$, so by (I9) $\mathbf{J} \mathbf{n}_\mathbf{a} = \mathbf{0}$.

The direction $\mathbf{a} = \mathbf{e}_z$ gives nothing new. At $\bar{\mathbf{d}} = (0, 0, -1)^\top$ its block component $\bar{\mathbf{B}}^\top \mathbf{e}_z$ is zero and the remaining components are today's yaw direction. The gauge of the problem in the initialisation period is therefore seven dimensional, three translations, three rotations and one scale, the same as that of monocular visual odometry (`doc/VIO.md` §2.4.1). Of these, the existing prior fixes four, and the three new ones, $\mathbf{n}_\sigma$, $\mathbf{n}_{\mathbf{e}_x}$ and $\mathbf{n}_{\mathbf{e}_y}$, are left. The physical quantities, the metric trajectory and the direction of gravity relative to the body, remain observable exactly as today. Only their split between the states and the block is undetermined.

#### 2.5.4 Why the gauge cannot be fixed by holding variables or by a prior

Three treatments of a gauge freedom exist [5]. Gauge fixation holds some coordinates constant. A gauge prior adds a stiff quadratic penalty on some coordinates. Free gauge keeps every coordinate free and removes the gauge directions from each step. This section derives the condition under which the first two work and shows that, for the three new directions, every admissible choice either fails, is ill conditioned at take off, or returns the estimator to today's behaviour.

Fixation. Let $\mathbf{S}$ be the selection matrix whose rows pick the coordinates to be held, so that fixation restricts the increment to $\mathbf{S}\boldsymbol{\delta} = \mathbf{0}$. A gauge direction $\mathbf{N} \mathbf{c}$, with $\mathbf{N}$ the matrix of §2.5.6 and $\mathbf{c} \in \mathbb{R}^3$, survives the restriction exactly when it satisfies it,

$$\mathbf{S} \mathbf{N} \mathbf{c} = \mathbf{0}.$$

Fixation therefore removes all three directions only if this has no solution other than $\mathbf{c} = \mathbf{0}$, that is, if $\mathbf{S} \mathbf{N}$ has rank 3. Holding more than three coordinates, or three with $\mathbf{S} \mathbf{N}$ of lower rank, also freezes physical degrees of freedom and biases the estimate. Rank 3 alone is not enough in floating point. The stiffness that fixation leaves along a direction $\mathbf{N} \mathbf{c}$ is governed by $|\mathbf{S} \mathbf{N} \mathbf{c}|$, so a fixation whose $\mathbf{S} \mathbf{N}$ has a small smallest singular value is nearly singular and behaves like no fixation at all. The candidates are as follows, reading the rows of $\mathbf{N}$ off §2.5.3.

1. The first state's position and yaw, which the initial prior already pins. The rows of $\mathbf{N}$ at that state are $\mathbf{p}_0$ and $\mathbf{v}_0$ for $\mathbf{n}_\sigma$, and $\mathbf{a}\times \mathbf{p}_0$, $\mathbf{a}$ and $\mathbf{a}\times \mathbf{v}_0$ for $\mathbf{n}_\mathbf{a}$. With $\mathbf{p}_0 = \mathbf{0}$ (`T_w_i_init` has zero translation, `sqrt_keypoint_vio.cpp:248`), the position rows are $\mathbf{0}$ and $\mathbf{a}\times \mathbf{0} = \mathbf{0}$, and the yaw row is the third component of $\mathbf{a}$, which is $0$ for $\mathbf{a} \in \{\mathbf{e}_x, \mathbf{e}_y\}$. With $\mathbf{S}_0$ selecting these four coordinates,

$$\mathbf{S}_0 \mathbf{N} = \mathbf{0}.$$

Every correction of §2.7 is a similarity about the origin, so the first state stays at the origin and this holds throughout the period. The existing pins therefore contribute nothing along the new directions.

2. The roll and pitch of one frame. The rotation rows of $\mathbf{N}$ at any frame are $\mathbf{0}$ for $\mathbf{n}_\sigma$ and $\mathbf{a}$ for $\mathbf{n}_\mathbf{a}$, so holding $\delta\varphi_x$ and $\delta\varphi_y$ of one frame gives

$$\mathbf{S}_{xy} \mathbf{N} = \begin{bmatrix}0 & 1 & 0\\ 0 & 0 & 1\end{bmatrix},$$

which fixes both tilt directions with perfect conditioning but has rank 2 and leaves the scale direction untouched.

3. A position or velocity coordinate of some frame, for the scale direction. The rows of $\mathbf{n}_\sigma$ there are $\mathbf{p}_f$ and $\mathbf{v}_f$, the frame's own position and velocity in $D$. On the ground before take off every frame sits near the origin at rest, so

$$|\mathbf{p}_f| \approx 0,$$

$$|\mathbf{v}_f| \approx 0,$$

for every $f$, and any such row of $\mathbf{S} \mathbf{N}$ is nearly zero. The stiffness this fixation gives along $\mathbf{n}_\sigma$ is proportional to the held coordinate's value, which is vanishingly small exactly when the scale is decided, during the take off transient of `context/takeoff_bias_divergence.md`. The fixing frame would also have to change at every marginalisation, since the oldest frames leave the window.

4. The gravity and scale block itself. The block's rows of $\mathbf{N}$ are $(0, 0, -1)$ for $\mathbf{n}_\sigma$ and $(\bar{\mathbf{B}}^\top \mathbf{a}, 0)$ for $\mathbf{n}_\mathbf{a}$. With $\mathbf{S}_{gs}$ selecting the block,

$$\mathbf{S}_{gs} \mathbf{N} = \begin{bmatrix}0 & \bar{\mathbf{b}}_1^\top \mathbf{e}_x & \bar{\mathbf{b}}_1^\top \mathbf{e}_y\\ 0 & \bar{\mathbf{b}}_2^\top \mathbf{e}_x & \bar{\mathbf{b}}_2^\top \mathbf{e}_y\\ -1 & 0 & 0\end{bmatrix},$$

which at $\bar{\mathbf{B}} = [\mathbf{e}_x, \mathbf{e}_y]$ is a signed permutation matrix, rank 3 and perfectly conditioned. This fixation is valid, and it is today's estimator. With the block held, $s = 1$ and $\mathbf{g}_D = \mathbf{g}$ in every solve, so any correction of scale or tilt that the data supports is carried by the frame states alone. For the frames that are frozen in the prior, that correction accumulates in their offsets $\boldsymbol{\delta}$ from first estimates that never move, while the prior's Jacobians stay at those points. This is root cause R5 (`context/takeoff_bias_divergence.md` §8), the scale held by the prior. The block is introduced precisely so that the correction can be carried by a variable whose value the re-centring of §2.7 applies exactly to every linearisation point, and holding it defeats that purpose.

A mixed fixation, item 2 for the tilt together with item 3 for the scale, would inherit item 3's failure at take off. No fixation of three coordinates is both well conditioned at take off and different from item 4.

Gauge prior. A gauge prior adds the cost $\frac{1}{2}w|\mathbf{S}\boldsymbol{\delta}_\text{tot}|^2$, with $\boldsymbol{\delta}_\text{tot}$ the offset of the selected coordinates from chosen values and $w$ a large weight, as the initial prior does with $w = 10^8$ (`config.vio_init_pose_weight`). Its contribution to the system is

$$\mathbf{H} \leftarrow \mathbf{H} + w\,\mathbf{S}^\top \mathbf{S}.$$

Along a gauge direction the data cost is flat, so the only curvature there is the prior's,

$$(\mathbf{N} \mathbf{c})^\top\big(\mathbf{H} + w\,\mathbf{S}^\top \mathbf{S}\big)(\mathbf{N} \mathbf{c}) = w\,|\mathbf{S} \mathbf{N} \mathbf{c}|^2,$$

using $\mathbf{H} \mathbf{N} = \mathbf{0}$ from §2.5.2. A gauge prior is therefore a soft version of the fixation with the same $\mathbf{S}$, and the same condition on $\mathbf{S} \mathbf{N}$ decides whether it works. The same equation with $\mathbf{S} = \mathbf{S}_0$ shows why the existing initial prior cannot fix the new directions, since $\mathbf{S}_0 \mathbf{N} = \mathbf{0}$ makes the curvature zero. A gauge prior has two further drawbacks here. The marginalisation of §2.6 folds every factor on a departing frame into the prior, so a gauge prior on a frame is stored there permanently as information along a direction no sensor measured, which is the spurious information of consequence 3 in §2.5.2. A gauge prior on the block, the soft form of item 4, would be marginalised into the prior as a permanent pull of $\sigma$ and $\boldsymbol{\delta}_\theta$ towards zero, and would hold the scale just as item 4 does.

The other priors present in the period are gauge invariant and need no treatment. The bias weights of the initial prior act on coordinates where every column of $\mathbf{N}$ is zero, so their $\mathbf{S} \mathbf{N}$ is zero as well. The zero velocity term has residual $w_0\,\mathbf{v}$ on the newest state, whose change along $\mathbf{n}_\sigma$ is $w_0\,\mathbf{v}$ and along $\mathbf{n}_\mathbf{a}$ is $w_0\,\mathbf{a}\times \mathbf{v}$. It is active only when $\mathbf{v} \approx \mathbf{0}$, so its curvature along $\mathbf{N}$ is negligible whenever it exists. Priors built during the initialisation period contain only gauge invariant factors linearised at frozen points, so they annihilate the null vectors built at the same points, which is the first estimate argument of `doc/VIO.md` §3.1.3 extended to the new directions.

The four directions of today's gauge are different. Let $\mathbf{n}_t$ be a translation or the yaw direction. The initial prior gives $\mathbf{S}_0 \mathbf{n}_t \neq \mathbf{0}$, so $\mathbf{H} \mathbf{n}_t = w\,\mathbf{S}_0^\top \mathbf{S}_0 \mathbf{n}_t \neq \mathbf{0}$ and they are not null directions of $\mathbf{H}$. They must therefore not be included in $\mathbf{N}$, because the projection below would delete that stiffness and unpin the world.

#### 2.5.5 Choice of treatment

By §2.5.4, neither fixation nor a prior handles the scale direction at take off without reverting to today's behaviour. The plan therefore uses the free gauge treatment by orthogonal projection, which DSO [4] uses for its own seven dimensional gauge. It treats all three directions with one operation, needs no choice of fixing frame as frames leave the window, and stores nothing in the prior.

#### 2.5.6 Projection onto the complement of the gauge directions

Stack the three null vectors as the columns of

$$\mathbf{N} = [\mathbf{n}_\sigma,\ \mathbf{n}_{\mathbf{e}_x},\ \mathbf{n}_{\mathbf{e}_y}] \in \mathbb{R}^{n\times 3},$$

where $n$ is the dimension of $\mathbf{H}$. Their span is the gauge subspace. Any vector $\mathbf{v} \in \mathbb{R}^n$ splits uniquely into a part inside the gauge subspace and a part orthogonal to it. To "project against the null space" means to keep only the orthogonal part, deleting the part that moves along a gauge direction.

How $\mathbf{N}$ is obtained. $\mathbf{N}$ is not estimated from the data. Its columns are the tangents of the transformations of §2.5.3, which are known in closed form from the structure of the residuals, so they are written down directly at the point where each variable's Jacobians are evaluated. For the gravity scale block at `aom` offset $g$ and a frame at offset $c$, with $\mathbf{p}_f$, $\mathbf{v}_f$ and $\bar{\mathbf{B}}$ taken at their linearisation points, the nonzero entries are as follows, every other entry, biases included, being zero.

1. Column $\mathbf{n}_\sigma$. Entry $g + 2$ is $-1$. Rows $c$ to $c + 2$ hold $\mathbf{p}_f$, and for a full state rows $c + 6$ to $c + 8$ hold $\mathbf{v}_f$.
2. Column $\mathbf{n}_{\mathbf{e}_k}$, for $k \in \{x, y\}$. Rows $g$ and $g + 1$ hold $\bar{\mathbf{B}}^\top \mathbf{e}_k$. Rows $c$ to $c + 2$ hold $\mathbf{e}_k\times \mathbf{p}_f$, rows $c + 3$ to $c + 5$ hold $\mathbf{e}_k$, and for a full state rows $c + 6$ to $c + 8$ hold $\mathbf{e}_k\times \mathbf{v}_f$.

Where these entries come from. They are not obtained from $\mathbf{H} \mathbf{n} = \mathbf{0}$ and $\mathbf{b}^\top \mathbf{n} = 0$. Those relations are consequences of the construction (§2.5.2), and they could not determine $\mathbf{N}$ in any case. $\mathbf{b}^\top \mathbf{n} = 0$ is a single scalar equation per vector, satisfied by an $(n-1)$ dimensional set of vectors. $\mathbf{H} \mathbf{n} = \mathbf{0}$ holds only approximately in floating point, and solving it numerically returns the weakly observed physical directions together with the gauge directions, as noted below. The entries come instead from the definition of a tangent direction in §2.5.2.

Here $\varepsilon$ is the scalar amount of the gauge transformation, the log of the scale factor for the scale family and the rotation angle in radians for a tilt family, with $\varepsilon = 0$ giving back the current estimate unchanged. $\mathbf{x}(\varepsilon)$ denotes the complete state, every pose, velocity, bias, landmark and the gravity and scale block, after that transformation of amount $\varepsilon$ has been applied to the current estimate $\mathbf{x}(0)$, so that as $\varepsilon$ varies it traces a curve of states that all produce the same residuals. A gauge family $\mathbf{x}(\varepsilon)$ is written down explicitly, each variable's value along it is expressed through that variable's own update rule, and the increment that the update rule needs in order to reproduce $\mathbf{x}(\varepsilon)$ is differentiated at $\varepsilon = 0$. The result is the component of $\mathbf{n}$ in that variable's rows. The update rules are those of §2.1.2 and §2.1.3,

$$\mathbf{p}_f \leftarrow \mathbf{p}_f + \delta\mathbf{p}_f,$$

$$\mathbf{R}_f \leftarrow \text{Exp}(\delta\boldsymbol{\varphi}_f)\,\mathbf{R}_f,$$

$$\mathbf{v}_f \leftarrow \mathbf{v}_f + \delta\mathbf{v}_f,$$

$$\mathbf{d} \leftarrow \text{Exp}(\bar{\mathbf{B}}\boldsymbol{\delta}_\theta)\,\bar{\mathbf{d}},$$

$$\sigma \leftarrow \bar{\sigma} + \delta\sigma,$$

and the rows follow the tangent ordering, $[\delta\mathbf{p}, \delta\boldsymbol{\varphi}, \delta\mathbf{v}, \delta\mathbf{b}_g, \delta\mathbf{b}_a]$ at offsets $c$, $c + 3$, $c + 6$, $c + 9$ and $c + 12$ of a full state, $[\delta\mathbf{p}, \delta\boldsymbol{\varphi}]$ at $c$ and $c + 3$ of a keyframe pose, and $[\delta\theta_1, \delta\theta_2, \delta\sigma]$ at $g$, $g + 1$ and $g + 2$ of the block.

Why the derivative of the increment is the gauge vector. Collect the increments of all variables that reproduce the family into one vector $\boldsymbol{\delta}(\varepsilon) \in \mathbb{R}^n$, in `aom` order, so that applying $\boldsymbol{\delta}(\varepsilon)$ to the estimate through the update rules gives $\mathbf{x}(\varepsilon)$. At $\varepsilon = 0$ the family is at the estimate, so $\boldsymbol{\delta}(0) = \mathbf{0}$. The argument has four steps.

Step 1. The residual along the family is a function of $\varepsilon$ through $\boldsymbol{\delta}(\varepsilon)$ alone,

$$\mathbf{r}\big(\mathbf{x}(\varepsilon)\big) = \mathbf{r}\big(\boldsymbol{\delta}(\varepsilon)\big),$$

where the right hand side is the residual written as a function of the increment, the form in which (I9) defines $\mathbf{J}$.

Step 2. Differentiating with respect to $\varepsilon$ at $\varepsilon = 0$ by the chain rule, the derivative of the residual with respect to the increment is $\mathbf{J}$ by (I9), and the derivative of the increment with respect to $\varepsilon$ is $\boldsymbol{\delta}'(0)$,

$$\frac{d}{d\varepsilon} \mathbf{r}\big(\boldsymbol{\delta}(\varepsilon)\big)\Big|_{\varepsilon = 0} = \mathbf{J}\,\boldsymbol{\delta}'(0).$$

Step 3. The family is a gauge family, so the left hand side of Step 1 does not depend on $\varepsilon$ and its derivative is zero,

$$\mathbf{J}\,\boldsymbol{\delta}'(0) = \mathbf{0}.$$

Step 4. This is the defining property of a gauge vector in §2.5.2, so

$$\mathbf{n} = \boldsymbol{\delta}'(0).$$

Equivalently, since $\boldsymbol{\delta}(0) = \mathbf{0}$, a first order expansion gives $\boldsymbol{\delta}(\varepsilon) = \varepsilon\,\mathbf{n} + O(\varepsilon^2)$, which is the statement in §2.5.2 that the family equals the estimate perturbed by $\varepsilon\mathbf{n} + O(\varepsilon^2)$. Two features of this construction matter. The increment, and not the variable's raw value, is differentiated because $\mathbf{J}$ and $\mathbf{H}$ act on increments, so $\mathbf{n}$ must be expressed in the same coordinates. For a rotation the distinction is visible, since the derivative of the matrix $\text{Exp}(\varepsilon\,\mathbf{e}_k)\,\mathbf{R}_f$ is the $3\times 3$ matrix $[\mathbf{e}_k]_\times \mathbf{R}_f$, whereas the derivative of the increment is the 3-vector $\mathbf{e}_k$ that occupies the rotation rows. Only the derivative at $\varepsilon = 0$ is kept, and not the increment for a finite $\varepsilon$, because $\mathbf{J}$ is a first order object. The higher order terms of $\boldsymbol{\delta}(\varepsilon)$, such as the $O(\varepsilon^2)$ part of $\mathbf{Q}(\varepsilon)\,\mathbf{p}_f - \mathbf{p}_f$, describe how the family curves away from a straight line in increment space, which the linear system of one iteration cannot represent. The two columns below apply this rule variable by variable, and each entry of $\mathbf{N}$ is one block of $\boldsymbol{\delta}'(0)$ placed in that variable's rows.

Scale column. The family scales every position and velocity by $e^{\varepsilon}$ and lowers the log scale by $\varepsilon$, leaving rotations, biases, the gravity direction and the landmarks alone,

$$\mathbf{p}_f(\varepsilon) = e^{\varepsilon}\,\mathbf{p}_f,$$

$$\mathbf{v}_f(\varepsilon) = e^{\varepsilon}\,\mathbf{v}_f,$$

$$\sigma(\varepsilon) = \sigma - \varepsilon.$$

It is an exact gauge family, since every residual sees positions and velocities only through $s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t)$, $s(\mathbf{v}_1 - \mathbf{v}_0)$ and $s \mathbf{t}_{ti,hi}$ (§2.5.3), and $e^{(\sigma - \varepsilon)} e^{\varepsilon} = e^{\sigma}$.

Step 1. The additive position rule reproduces $\mathbf{p}_f(\varepsilon)$ with $\delta\mathbf{p}_f(\varepsilon) = (e^{\varepsilon} - 1)\,\mathbf{p}_f$, whose derivative at $\varepsilon = 0$ is

$$\frac{d}{d\varepsilon}\Big[(e^{\varepsilon} - 1)\,\mathbf{p}_f\Big]_{\varepsilon = 0} = \mathbf{p}_f,$$

which fills rows $c$ to $c + 2$. Step 2. The same computation for the additive velocity rule gives $\mathbf{v}_f$ in rows $c + 6$ to $c + 8$ of a full state. A keyframe pose has no velocity rows. Step 3. The log scale rule reproduces $\sigma(\varepsilon)$ with $\delta\sigma(\varepsilon) = \delta\sigma - \varepsilon$, a change of $-\varepsilon$ whose derivative is $-1$, which fills entry $g + 2$. Step 4. Rotations, biases and the gravity direction do not change along the family, so their increments are zero for every $\varepsilon$ and so are their rows.

Tilt column for the axis $\mathbf{e}_k$. The family rotates the whole of $D$, gravity direction included, by $\mathbf{Q}(\varepsilon) = \text{Exp}(\varepsilon\,\mathbf{e}_k)$, leaving biases, which are body frame quantities, and the scale alone,

$$\mathbf{p}_f(\varepsilon) = \mathbf{Q}(\varepsilon)\,\mathbf{p}_f,$$

$$\mathbf{R}_f(\varepsilon) = \mathbf{Q}(\varepsilon)\,\mathbf{R}_f,$$

$$\mathbf{v}_f(\varepsilon) = \mathbf{Q}(\varepsilon)\,\mathbf{v}_f,$$

$$\mathbf{d}(\varepsilon) = \mathbf{Q}(\varepsilon)\,\bar{\mathbf{d}}.$$

It is an exact gauge family by Steps 3 and 4 of the tilt derivation in §2.5.3. Step 1. The additive position rule needs $\delta\mathbf{p}_f(\varepsilon) = \mathbf{Q}(\varepsilon)\,\mathbf{p}_f - \mathbf{p}_f$. By (I1), $\mathbf{Q}(\varepsilon) = \mathbf{I} + \varepsilon[\mathbf{e}_k]_\times + O(\varepsilon^2)$, so

$$\delta\mathbf{p}_f(\varepsilon) = \varepsilon\,[\mathbf{e}_k]_\times \mathbf{p}_f + O(\varepsilon^2) = \varepsilon\,\mathbf{e}_k\times \mathbf{p}_f + O(\varepsilon^2),$$

whose derivative at $\varepsilon = 0$, $\mathbf{e}_k\times \mathbf{p}_f$, fills rows $c$ to $c + 2$. Step 2. The rotation rule is a left multiplication by an exponential, the same form as the family itself, so $\text{Exp}(\delta\boldsymbol{\varphi}_f)\,\mathbf{R}_f = \text{Exp}(\varepsilon\,\mathbf{e}_k)\,\mathbf{R}_f$ holds exactly with

$$\delta\boldsymbol{\varphi}_f(\varepsilon) = \varepsilon\,\mathbf{e}_k,$$

whose derivative $\mathbf{e}_k$ fills rows $c + 3$ to $c + 5$. Step 3. The velocity rule is additive like the position rule, so Step 1 with $\mathbf{v}_f$ in place of $\mathbf{p}_f$ gives $\mathbf{e}_k\times \mathbf{v}_f$ in rows $c + 6$ to $c + 8$ of a full state. Step 4. The gravity direction rule needs an increment $\boldsymbol{\delta}_\theta(\varepsilon)$ with $\text{Exp}(\bar{\mathbf{B}}\boldsymbol{\delta}_\theta)\,\bar{\mathbf{d}} = \mathbf{Q}(\varepsilon)\,\bar{\mathbf{d}}$ to first order. Step 2 of the tilt derivation in §2.5.3 solves exactly this, by splitting $\mathbf{e}_k$ into its parts along $\bar{\mathbf{B}}$ and along $\bar{\mathbf{d}}$ and noting that the part along $\bar{\mathbf{d}}$ does not move $\bar{\mathbf{d}}$,

$$\boldsymbol{\delta}_\theta(\varepsilon) = \varepsilon\,\bar{\mathbf{B}}^\top \mathbf{e}_k,$$

whose derivative $\bar{\mathbf{B}}^\top \mathbf{e}_k$ fills entries $g$ and $g + 1$. At the initial $\bar{\mathbf{B}} = [\mathbf{e}_x, \mathbf{e}_y]$ this is $(1, 0)^\top$ for $\mathbf{e}_x$ and $(0, 1)^\top$ for $\mathbf{e}_y$. Step 5. The biases and the scale do not change along the family, so entry $g + 2$ and the bias rows are zero.

Two points complete the picture. First, every entry depends on the point at which it is evaluated, through $\mathbf{p}_f$, $\mathbf{v}_f$ and $\bar{\mathbf{B}}$, because the gauge transformation moves a far away frame more than a near one. $\mathbf{N}$ must therefore be built at the same point at which the Jacobians are evaluated. The identity $\mathbf{J} \mathbf{n} = \mathbf{0}$ of §2.5.2 compares $\mathbf{J}(\mathbf{x})$ with $\mathbf{n}(\mathbf{x})$ at one point $\mathbf{x}$, and a mismatch between the two points is exactly the source of spurious information that first estimate Jacobians prevent. Second, the landmarks have no rows in $\mathbf{H}$, which is formed after their elimination, and their components of every gauge direction are zero in any case, because a landmark is stored relative to its host camera and none of these families changes $\mathbf{T}_{t,h}$ (§2.5.3). §2.5.2 shows that such a vector stays a null vector of the reduced system.

The linearisation point of a variable that is not yet frozen follows its current value, because `PoseVelBiasStateWithLin::applyInc` moves `state_linearized` itself until the state is frozen (`include/basalt/utils/imu_types.h:113-121`), so these are exactly the points at which Basalt evaluates every Jacobian of that variable. `ComputeGaugeProjector` (Change 13) builds this matrix, and `GaugeNullspaceGSTest` (§4.2) checks $|\mathbf{H} \mathbf{N}|$ against it. The alternative of finding the null directions numerically, from the eigenvectors of $\mathbf{H}$ with the smallest eigenvalues, is not used. It cannot distinguish a gauge direction from a physical direction that is merely weakly observed, such as the accelerometer bias along the direction of a straight climb (`context/takeoff_bias_divergence.md` §2), and projecting such a direction would delete real information. The gauge directions are known exactly because they follow from a symmetry of the residuals, not from the data.

The orthogonal part is found by least squares. Here $\mathbf{v}$ is any vector in $\mathbb{R}^n$ over the same variables as $\mathbf{H}$, ordered as in `aom`, and in this plan it stands for each of the three quantities the projector is applied to below, the gradient $\mathbf{b}$, each column and row of $\mathbf{H}$, and the solution $\mathbf{x}$.

Step 1. The part inside the subspace has the form $\mathbf{N} \mathbf{c}$ for some $\mathbf{c} \in \mathbb{R}^3$, chosen as the point of the subspace closest to $\mathbf{v}$, that is, the minimiser of

$$f(\mathbf{c}) = |\mathbf{v} - \mathbf{N} \mathbf{c}|^2.$$

Step 2. Expanding the squared norm as $(\mathbf{v} - \mathbf{N} \mathbf{c})^\top(\mathbf{v} - \mathbf{N} \mathbf{c})$ and using that the scalar $\mathbf{v}^\top \mathbf{N} \mathbf{c}$ equals its transpose $\mathbf{c}^\top \mathbf{N}^\top \mathbf{v}$,

$$f(\mathbf{c}) = \mathbf{v}^\top \mathbf{v} - 2\,\mathbf{c}^\top \mathbf{N}^\top \mathbf{v} + \mathbf{c}^\top \mathbf{N}^\top \mathbf{N}\,\mathbf{c}.$$

Step 3. Differentiating with respect to $\mathbf{c}$, with the rules $\partial(\mathbf{c}^\top \mathbf{a})/\partial\mathbf{c} = \mathbf{a}$ and $\partial(\mathbf{c}^\top \mathbf{M} \mathbf{c})/\partial\mathbf{c} = 2\mathbf{M} \mathbf{c}$ for a symmetric $\mathbf{M}$,

$$\frac{\partial f}{\partial\mathbf{c}} = -2\,\mathbf{N}^\top \mathbf{v} + 2\,\mathbf{N}^\top \mathbf{N}\,\mathbf{c}.$$

Step 4. Setting the derivative to zero and dividing by $-2$,

$$\mathbf{N}^\top(\mathbf{v} - \mathbf{N} \mathbf{c}) = \mathbf{0}.$$

This is where $\mathbf{N}^\top$ comes from. It is the derivative of the squared distance, and it has a direct geometric reading. The $i$-th row of $\mathbf{N}^\top$ is the null vector $\mathbf{n}_i^\top$, so the $i$-th row of the condition is $\mathbf{n}_i^\top(\mathbf{v} - \mathbf{N} \mathbf{c}) = 0$. The remainder $\mathbf{v} - \mathbf{N} \mathbf{c}$ is orthogonal to every gauge direction, which is what makes $\mathbf{N} \mathbf{c}$ the closest point.

Step 5. $\mathbf{N}^\top \mathbf{N}$ is invertible because the three null vectors are linearly independent, so

$$\mathbf{c} = (\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top \mathbf{v}.$$

Step 6. Removing that part defines the projector $\mathbf{P}$,

$$\mathbf{P} \mathbf{v} = \mathbf{v} - \mathbf{N}(\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top \mathbf{v},$$

$$\mathbf{P} = \mathbf{I} - \mathbf{N}(\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top.$$

Step 4 states that the remainder $\mathbf{P} \mathbf{v}$ is orthogonal to every column of $\mathbf{N}$. Three properties follow by direct multiplication and are used below. $\mathbf{P}$ is symmetric, since $(\mathbf{N}^\top \mathbf{N})^{-1}$ is. It annihilates the gauge subspace,

$$\mathbf{P} \mathbf{N} = \mathbf{N} - \mathbf{N}(\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top \mathbf{N} = \mathbf{N} - \mathbf{N} = \mathbf{0}.$$

It leaves a vector orthogonal to the subspace unchanged, because $\mathbf{N}^\top \mathbf{v} = \mathbf{0}$ removes the second term, and applying it twice changes nothing more, $\mathbf{P} \mathbf{P} = \mathbf{P}$. $\mathbf{P}$ depends only on the subspace and not on the scaling of its columns, since replacing $\mathbf{N}$ by $\mathbf{N} \mathbf{M}$ for an invertible $3\times 3$ matrix $\mathbf{M}$ gives $\mathbf{N} \mathbf{M}(\mathbf{M}^\top \mathbf{N}^\top \mathbf{N} \mathbf{M})^{-1} \mathbf{M}^\top \mathbf{N}^\top = \mathbf{N}(\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top$, which is why DSO may normalise its null vectors before use without changing the result.

The plan applies $\mathbf{P}$ to the system and to the solution of every iteration of the period,

$$\bar{\mathbf{H}} = \mathbf{P} \mathbf{H} \mathbf{P},$$

$$\bar{\mathbf{b}} = \mathbf{P} \mathbf{b},$$

$$\big(\bar{\mathbf{H}} + \max(\lambda\,\text{diag}\,\bar{\mathbf{H}},\ \lambda_\text{min})\big)\,\mathbf{x} = \bar{\mathbf{b}},$$

$$\mathbf{x} \leftarrow \mathbf{P} \mathbf{x},$$

after which the step $-\mathbf{x}$ is applied as today. The last projection is needed even though $\bar{\mathbf{H}}$ and $\bar{\mathbf{b}}$ are projected, because the damping $\mathbf{D}$ is not a multiple of the identity, so the solution satisfies only $\mathbf{N}^\top \mathbf{D} \mathbf{x} = \mathbf{0}$ and not $\mathbf{N}^\top \mathbf{x} = \mathbf{0}$, as derived below. $\mathbf{D}$ is the diagonal Marquardt damping, with entries $D_{ii} = \max(\lambda \bar{H}_{ii},\ \lambda_\text{min})$. The damped solve is the minimiser of

$$m(\mathbf{x}) = \frac{1}{2}\mathbf{x}^\top \bar{\mathbf{H}} \mathbf{x} - \bar{\mathbf{b}}^\top \mathbf{x} + \frac{1}{2}\mathbf{x}^\top \mathbf{D} \mathbf{x}.$$

Split $\mathbf{x} = \mathbf{x}_\perp + \mathbf{N} \mathbf{c}$ with $\mathbf{N}^\top \mathbf{x}_\perp = \mathbf{0}$. The first two terms do not depend on $\mathbf{c}$, because $\bar{\mathbf{H}} \mathbf{N} = \mathbf{0}$ and $\mathbf{N}^\top \bar{\mathbf{b}} = \mathbf{0}$, so along the gauge directions $\mathbf{D}$ is the only stiffness and $\mathbf{c}$ minimises the last term alone. Setting its derivative with respect to $\mathbf{c}$ to zero,

$$\mathbf{N}^\top \mathbf{D} (\mathbf{x}_\perp + \mathbf{N} \mathbf{c}) = \mathbf{0},$$

$$\mathbf{c} = -(\mathbf{N}^\top \mathbf{D} \mathbf{N})^{-1} \mathbf{N}^\top \mathbf{D} \mathbf{x}_\perp.$$

If $\mathbf{D} = \mu \mathbf{I}$, then $\mathbf{N}^\top \mathbf{D} \mathbf{x}_\perp = \mu \mathbf{N}^\top \mathbf{x}_\perp = \mathbf{0}$ and $\mathbf{c} = \mathbf{0}$. Basalt's $D_{ii}$ follow $\bar{H}_{ii}$, which differ by orders of magnitude between positions, rotations, velocities, biases and the block, so $\mathbf{c} \neq \mathbf{0}$ and the damping itself pushes the step along the gauge directions. The final $\mathbf{P}$ removes that component. Each operation has its own purpose.

Projecting the step selects one representative. By consequence 1 of §2.5.2 the solutions form a family $\mathbf{x}^* + \mathbf{N} \mathbf{c}$, all with the same predicted cost. Writing $\mathbf{x} = \mathbf{P} \mathbf{x} + \mathbf{N} \mathbf{c}$ with $\mathbf{c} = (\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top \mathbf{x}$, which is Step 5 of the least squares derivation at the start of this section applied with $\mathbf{v} = \mathbf{x}$, the two parts are orthogonal, so

$$|\mathbf{x}|^2 = |\mathbf{P} \mathbf{x}|^2 + |\mathbf{N} \mathbf{c}|^2.$$

$\mathbf{P} \mathbf{x}$ is therefore the shortest member of the family, and it changes the linear model by exactly what $\mathbf{x}$ does,

$$\mathbf{r} - \mathbf{J} \mathbf{P} \mathbf{x} = \mathbf{r} - \mathbf{J} \mathbf{x} + \mathbf{J} \mathbf{N} \mathbf{c} = \mathbf{r} - \mathbf{J} \mathbf{x},$$

by $\mathbf{J} \mathbf{N} = \mathbf{0}$. The projected step loses no predicted decrease, and `l_diff`, which `backSubstitute` computes from the step actually applied, keeps the acceptance test of `context/vio_opt.md` §10.3 consistent. Without this, the damping alone would set the gauge component, and the block and the states could trade scale and tilt between each other from iteration to iteration.

Projecting the system is a safeguard, not the essential operation. Is $\mathbf{b}$ not already orthogonal to the gauge directions by §2.5.2? In exact arithmetic it is. The derivation of §2.5.2 holds at whatever point the Jacobians are evaluated, provided $\mathbf{N}$ is evaluated at the same point. Basalt evaluates every Jacobian of a variable at one point, its first estimate if it is frozen and its current value otherwise (`doc/VIO.md` §3.1.3), and $\mathbf{N}$ is built at exactly those points. Every gauge invariant factor therefore satisfies $\mathbf{J} \mathbf{n} = \mathbf{0}$ exactly, whether or not its residual value was recomputed at the current estimate, so

$$\mathbf{H} \mathbf{N} = \mathbf{0},$$

$$\mathbf{N}^\top \mathbf{b} = \mathbf{0}$$

hold up to round off. In that case the projection changes nothing. Step 1. By $\mathbf{H} \mathbf{N} = \mathbf{0}$,

$$\mathbf{H} \mathbf{P} = \mathbf{H} - \mathbf{H} \mathbf{N}(\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top = \mathbf{H}.$$

Step 2. Transposing, with $\mathbf{H}$ and $\mathbf{P}$ symmetric, $\mathbf{P} \mathbf{H} = \mathbf{H}$, so $\mathbf{P} \mathbf{H} \mathbf{P} = \mathbf{H} \mathbf{P} = \mathbf{H}$. Step 3. By $\mathbf{N}^\top \mathbf{b} = \mathbf{0}$,

$$\mathbf{P} \mathbf{b} = \mathbf{b} - \mathbf{N}(\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top \mathbf{b} = \mathbf{b}.$$

Four effects break the exactness in practice. The zero velocity term is not gauge invariant, its Jacobian along $\mathbf{n}_\sigma$ being $w_0\,\mathbf{v}$ and along $\mathbf{n}_\mathbf{a}$ being $w_0\,\mathbf{a}\times \mathbf{v}$, which is small but not zero (§2.5.4). Round off in the per landmark QR and in the accumulation of $\mathbf{H}$ leaves components along $\mathbf{N}$ that grow with the number of landmarks. The pose updates from the local mapper replace a keyframe's linearisation point without rebuilding the priors that were built at the old point (`sqrt_keypoint_vio.cpp:592-641`, `context/vio_opt.md` §12.5), so the priors' null vectors no longer match $\mathbf{N}$ exactly, by an amount bounded by the update thresholds `kRelinThresholdTrans` and `kRelinThresholdRot`. A defect in the evaluation point bookkeeping, for instance a factor that evaluates a Jacobian at a point other than the one used for $\mathbf{N}$, would leave a larger residue. Each of these puts into $\mathbf{H}$ the spurious information of consequence 3 in §2.5.2. The projected system removes it. It has the gauge subspace as an exact null space,

$$\bar{\mathbf{H}} \mathbf{N} = \mathbf{P} \mathbf{H} \mathbf{P} \mathbf{N} = \mathbf{0},$$

$$\mathbf{N}^\top\bar{\mathbf{b}} = (\mathbf{P} \mathbf{N})^\top \mathbf{b} = \mathbf{0},$$

and for any two vectors $\mathbf{u}$, $\mathbf{w}$ orthogonal to $\mathbf{N}$, $\mathbf{P} \mathbf{u} = \mathbf{u}$ and $\mathbf{P} \mathbf{w} = \mathbf{w}$ give $\mathbf{u}^\top\bar{\mathbf{H}} \mathbf{w} = \mathbf{u}^\top \mathbf{H} \mathbf{w}$, so no information the measurements carry is touched. Its cost is two products of an $n\times n$ matrix with an $n\times 3$ matrix.

Projecting the step, by contrast, is needed even in exact arithmetic. With $\bar{\mathbf{H}} \mathbf{N} = \mathbf{0}$ and $\mathbf{N}^\top\bar{\mathbf{b}} = \mathbf{0}$, multiply the damped equation on the left by $\mathbf{N}^\top$,

$$\mathbf{N}^\top\big(\bar{\mathbf{H}} + \mathbf{D}\big) \mathbf{x} = \mathbf{N}^\top\bar{\mathbf{b}} = \mathbf{0}.$$

Since $\mathbf{N}^\top\bar{\mathbf{H}} = (\bar{\mathbf{H}} \mathbf{N})^\top = \mathbf{0}$, this leaves

$$\mathbf{N}^\top \mathbf{D}\,\mathbf{x} = \mathbf{0}.$$

The solution is orthogonal to the gauge directions after weighting by $\mathbf{D}$, not before. Only if $\mathbf{D}$ were a multiple of the identity would this give $\mathbf{N}^\top \mathbf{x} = \mathbf{0}$. Basalt's $\mathbf{D} = \max(\lambda\,\text{diag}\,\mathbf{H},\ \lambda_\text{min})$ varies by orders of magnitude between positions, rotations, velocities, biases and the block, so $\mathbf{x}$ keeps a gauge component that the final $\mathbf{x} \leftarrow \mathbf{P} \mathbf{x}$ removes. Note, 2026-11-05. This paragraph previously justified the projection of the system by saying that the relinearised frames and block, against the frozen priors, leave $\mathbf{H} \mathbf{N}$ small but nonzero. That reasoning was wrong, since $\mathbf{N}$ is built at each variable's own evaluation point and the identity then holds exactly for every gauge invariant factor. The projection of the system was kept as a safeguard against the three effects listed above.

DSO, the origin of the technique, implements exactly this projector in `EnergyFunctional::orthogonalize` (`dm-vio/src/dso/OptimizationBackend/EnergyFunctional.cpp:784-829`), forming $\mathbf{N}(\mathbf{N}^\top \mathbf{N})^{-1} \mathbf{N}^\top$ through a truncated SVD and applying it as `b -= NNpiTS * b` and `H -= NNpiTS * H * NNpiTS`. Its null vectors are the six dimensional pose gauge and the one dimensional scale gauge of monocular odometry, computed by central differences of a global left perturbation of each frame's linearisation point (`dm-vio/src/dso/FullSystem/HessianBlocks.cpp:80-100`) and assembled in `FullSystem::getNullspaces` (`FullSystemOptimize.cpp:704-757`). That is what "projecting against its pose and scale null spaces" means there. The plan's null vectors play the same role for the three directions of §2.5.3 and are written analytically. Under DSO's default `SOLVER_ORTHOGONALIZE_X_LATER` (`dm-vio/src/dso/util/settings.cpp:81`) only the step is projected, from the third iteration on (`EnergyFunctional.cpp:980-984`), while projection of the system is a separate option, `SOLVER_ORTHOGONALIZE_SYSTEM`, applied only once the first frame and its prior have left the window (`:876-890`). The plan projects both. Its step projection is DSO's default, and its system projection is the safeguard described above. DSO applies its system projection only once the first frame has left, because its $\mathbf{N}$ includes the directions the first frame's prior pins. The plan can apply it with the first frame present, because the four directions that prior pins are excluded from $\mathbf{N}$ (§2.5.4). Note, 2026-11-05. This paragraph previously stated that DSO's `orthogonalize` projects both the system and the step under its default mode. Reading `solveSystemF` showed that the default projects only the step, so the attribution was corrected.

#### 2.5.7 What the projection decides and what it does not

The projection fixes how a correction is written, not what the correction is. A change of metric scale that the inertial data supports can be written as a change of $\sigma$, as a rescaling of the frame positions and velocities, or as any mix of the two, and these differ by a multiple of $\mathbf{n}_\sigma$. The projection selects the mix whose increment is shortest. Writing the change as $\delta\sigma = \varepsilon$ alone, the projected increment keeps a share $1 - 1/|\mathbf{n}_\sigma|^2$ of it in $\sigma$ and moves the rest onto the frames along $\mathbf{p}_f$ and $\mathbf{v}_f$, where $|\mathbf{n}_\sigma|^2 = 1 + \sum_f|\mathbf{p}_f|^2 + \sum_f|\mathbf{v}_f|^2$. Once the window has moved a few metres or reached a speed of a few metres per second, nearly all of the correction is carried by $\sigma$. Near the origin and at rest, $\mathbf{n}_\sigma$ approaches $-\mathbf{e}_\sigma$, so $\sigma$ itself becomes a gauge direction, which is the correct conclusion, since a stationary vehicle at the origin carries no information about scale. The tilt directions behave alike with $\mathbf{p}_f$, $\mathbf{v}_f$ and the unit rotation components.

The metric result does not depend on this split. The re-centring of §2.7 maps every state through $s\,\mathbf{R}_{wg}^\top$, and a motion along a gauge direction changes the states and the correction in opposite ways that cancel in that product. The published trajectory, the map and the residuals are therefore the same whichever representative is chosen. What does read the block alone is the swap rule of §2.6.6, which tracks $\sigma$. Because the projection keeps nearly all of a scale correction in $\sigma$ once the vehicle has moved, the rule sees the scale changes that matter, and §4.3 item 4 checks this on the take off replays. Note, 2026-11-05. The previous text of §2.5.3 stated that within a frame the stored world keeps the scale and tilt it had at the start of the frame and that every change is expressed through $\sigma$ and $\boldsymbol{\delta}_\theta$. The projection selects the shortest increment, not the one with zero frame components, so that statement held only approximately and was replaced by the analysis above.

### 2.6 Marginalisation in the Initialisation Period

This section first derives marginalisation as Basalt performs it, then shows why it freezes the scale, and then reconstructs, from the problem it solves, the dynamic marginalisation of VI-DSO [2, §III-F.3], deriving the guarantees that the paper states without proof.

#### 2.6.1 Marginalisation as a Schur complement

Partition the variables of the linear system of §2.5.1 into those to be kept, subscript $k$, and those to be removed, subscript $m$, following `context/vio_opt.md` §12.3. With the increment $\boldsymbol{\delta} = [\boldsymbol{\delta}_m; \boldsymbol{\delta}_k]$, the quadratic model of the cost of every factor connected to the removed variables is

$$F(\boldsymbol{\delta}_m, \boldsymbol{\delta}_k) = c + \boldsymbol{\delta}_m^\top \mathbf{b}_m + \boldsymbol{\delta}_k^\top \mathbf{b}_k + \frac{1}{2}\boldsymbol{\delta}_m^\top \mathbf{H}_{mm} \boldsymbol{\delta}_m + \boldsymbol{\delta}_m^\top \mathbf{H}_{mk} \boldsymbol{\delta}_k + \frac{1}{2}\boldsymbol{\delta}_k^\top \mathbf{H}_{kk} \boldsymbol{\delta}_k,$$

with $\mathbf{H}_{km} = \mathbf{H}_{mk}^\top$ and $c$ a constant. Marginalising $\boldsymbol{\delta}_m$ means replacing $F$ by the best cost it can reach for each value of $\boldsymbol{\delta}_k$, so that the kept variables retain everything the removed factors said about them.

Step 1. For fixed $\boldsymbol{\delta}_k$, $F$ is minimised over $\boldsymbol{\delta}_m$ where its derivative with respect to $\boldsymbol{\delta}_m$ vanishes,

$$\frac{\partial F}{\partial\boldsymbol{\delta}_m} = \mathbf{b}_m + \mathbf{H}_{mm} \boldsymbol{\delta}_m + \mathbf{H}_{mk} \boldsymbol{\delta}_k = \mathbf{0}.$$

Step 2. Solving for the removed increment, with $\mathbf{H}_{mm}$ invertible,

$$\boldsymbol{\delta}_m^* = -\mathbf{H}_{mm}^{-1}(\mathbf{b}_m + \mathbf{H}_{mk} \boldsymbol{\delta}_k).$$

Step 3. At the minimiser, Step 1 gives $\mathbf{H}_{mm} \boldsymbol{\delta}_m^* = -(\mathbf{b}_m + \mathbf{H}_{mk} \boldsymbol{\delta}_k)$, so the three terms of $F$ that contain $\boldsymbol{\delta}_m$ combine into one,

$$\boldsymbol{\delta}_m^{*\top} \mathbf{b}_m + \frac{1}{2}\boldsymbol{\delta}_m^{*\top} \mathbf{H}_{mm} \boldsymbol{\delta}_m^* + \boldsymbol{\delta}_m^{*\top} \mathbf{H}_{mk} \boldsymbol{\delta}_k = \frac{1}{2}\boldsymbol{\delta}_m^{*\top}(\mathbf{b}_m + \mathbf{H}_{mk} \boldsymbol{\delta}_k).$$

Step 4. Substituting Step 2 into the right hand side,

$$\frac{1}{2}\boldsymbol{\delta}_m^{*\top}(\mathbf{b}_m + \mathbf{H}_{mk} \boldsymbol{\delta}_k) = -\frac{1}{2}(\mathbf{b}_m + \mathbf{H}_{mk} \boldsymbol{\delta}_k)^\top \mathbf{H}_{mm}^{-1}(\mathbf{b}_m + \mathbf{H}_{mk} \boldsymbol{\delta}_k).$$

Step 5. Expanding the product, using the symmetry of $\mathbf{H}_{mm}^{-1}$ to merge the two cross terms,

$$-\frac{1}{2}(\mathbf{b}_m + \mathbf{H}_{mk} \boldsymbol{\delta}_k)^\top \mathbf{H}_{mm}^{-1}(\mathbf{b}_m + \mathbf{H}_{mk} \boldsymbol{\delta}_k) = -\frac{1}{2}\mathbf{b}_m^\top \mathbf{H}_{mm}^{-1} \mathbf{b}_m - \boldsymbol{\delta}_k^\top \mathbf{H}_{km} \mathbf{H}_{mm}^{-1} \mathbf{b}_m - \frac{1}{2}\boldsymbol{\delta}_k^\top \mathbf{H}_{km} \mathbf{H}_{mm}^{-1} \mathbf{H}_{mk} \boldsymbol{\delta}_k.$$

Step 6. Adding the terms of $F$ that contain only $\boldsymbol{\delta}_k$, $\boldsymbol{\delta}_k^\top \mathbf{b}_k$ and $\frac{1}{2}\boldsymbol{\delta}_k^\top \mathbf{H}_{kk} \boldsymbol{\delta}_k$, and collecting the linear and quadratic terms,

$$F^*(\boldsymbol{\delta}_k) = c' + \boldsymbol{\delta}_k^\top \mathbf{b}' + \frac{1}{2}\boldsymbol{\delta}_k^\top \mathbf{H}' \boldsymbol{\delta}_k,$$

$$\mathbf{H}' = \mathbf{H}_{kk} - \mathbf{H}_{km} \mathbf{H}_{mm}^{-1} \mathbf{H}_{mk},$$

$$\mathbf{b}' = \mathbf{b}_k - \mathbf{H}_{km} \mathbf{H}_{mm}^{-1} \mathbf{b}_m,$$

where $c'$ collects the constants. $\mathbf{H}'$ is the Schur complement of $\mathbf{H}_{mm}$. In probabilistic terms, $\mathbf{H}$ is the information matrix of a Gaussian over all variables and $\mathbf{H}'$ the information matrix of its marginal over the kept ones, which is where the name comes from. VI-DSO performs exactly this, citing DSO's equations [2, §III-F.2].

#### 2.6.2 The square root form used by Basalt

Basalt never forms $\mathbf{H}$. With `vio_sqrt_marg` on, `marginalizeHelperSqrtToSqrt` (`marg_helper.cpp`, `context/vio_opt.md` §12.3) works on the stacked Jacobian $\mathbf{J} = [\mathbf{J}_m\ \mathbf{J}_k]$ and residual $\mathbf{r}$, with $\mathbf{H} = \mathbf{J}^\top \mathbf{J}$ and $\mathbf{b} = \mathbf{J}^\top \mathbf{r}$. Step 1. A Householder QR of the marginalised columns gives an orthogonal $\mathbf{Q}_h$ with

$$\mathbf{Q}_h^\top [\mathbf{J}_m\ \mathbf{J}_k\ |\ \mathbf{r}] = \begin{bmatrix}\mathbf{R}_{mm} & \mathbf{R}_{mk} & \mathbf{c}_m\\ \mathbf{0} & \mathbf{J}_\text{new} & \mathbf{r}_\text{new}\end{bmatrix},$$

with $\mathbf{R}_{mm}$ upper triangular. Step 2. An orthogonal transformation preserves the norm, so the cost splits into two parts,

$$\frac{1}{2}|\mathbf{J}_m \boldsymbol{\delta}_m + \mathbf{J}_k \boldsymbol{\delta}_k + \mathbf{r}|^2 = \frac{1}{2}|\mathbf{R}_{mm} \boldsymbol{\delta}_m + \mathbf{R}_{mk} \boldsymbol{\delta}_k + \mathbf{c}_m|^2 + \frac{1}{2}|\mathbf{J}_\text{new} \boldsymbol{\delta}_k + \mathbf{r}_\text{new}|^2.$$

Step 3. For any $\boldsymbol{\delta}_k$ the first part can be made zero by $\boldsymbol{\delta}_m = -\mathbf{R}_{mm}^{-1}(\mathbf{R}_{mk} \boldsymbol{\delta}_k + \mathbf{c}_m)$, so minimising over $\boldsymbol{\delta}_m$ leaves the second part, $\frac{1}{2}|\mathbf{J}_\text{new} \boldsymbol{\delta}_k + \mathbf{r}_\text{new}|^2$, which is the new prior. Step 4. It agrees with §2.6.1. From $\mathbf{J}^\top \mathbf{J} = (\mathbf{Q}_h^\top \mathbf{J})^\top(\mathbf{Q}_h^\top \mathbf{J})$, the blocks are

$$\mathbf{H}_{mm} = \mathbf{R}_{mm}^\top \mathbf{R}_{mm},$$

$$\mathbf{H}_{mk} = \mathbf{R}_{mm}^\top \mathbf{R}_{mk},$$

$$\mathbf{H}_{kk} = \mathbf{R}_{mk}^\top \mathbf{R}_{mk} + \mathbf{J}_\text{new}^\top \mathbf{J}_\text{new},$$

and substituting them into the Schur complement,

$$\mathbf{H}_{kk} - \mathbf{H}_{km} \mathbf{H}_{mm}^{-1} \mathbf{H}_{mk} = \mathbf{R}_{mk}^\top \mathbf{R}_{mk} + \mathbf{J}_\text{new}^\top \mathbf{J}_\text{new} - \mathbf{R}_{mk}^\top \mathbf{R}_{mm} \mathbf{R}_{mm}^{-1} \mathbf{R}_{mm}^{-\top} \mathbf{R}_{mm}^\top \mathbf{R}_{mk} = \mathbf{J}_\text{new}^\top \mathbf{J}_\text{new}.$$

The square root form gives the same marginal without squaring the condition number. A removed variable that no factor constrains, such as the velocity of a departing state in a prior built without inertial factors (§2.6.5), has a zero column and a zero pivot, and the helper's rank threshold drops it instead of inverting it (`context/vio_opt.md` §12.3).

#### 2.6.3 First estimate Jacobians, and why the prior freezes the scale

The prior $\frac{1}{2}|\mathbf{J}_\text{new} \boldsymbol{\delta}_k + \mathbf{r}_\text{new}|^2$ is a linear model built at the linearisation points of the kept variables at the moment of marginalisation. If later factors on the same variables were linearised at other points, the prior and those factors would have different null spaces, and the system would acquire the spurious information of §2.5.2. VI-DSO states the rule and its reason directly, "it is important that Jacobians are all evaluated at the same value for variables that are connected to a marginalization factor as otherwise the nullspaces get eliminated" [2, §III-F.2]. Basalt enforces it by freezing every kept state at its first estimate, `setLinTrue` (`sqrt_keypoint_vio.cpp:1419`, `context/vio_opt.md` §12.4), and in this plan the gravity and scale block is frozen the same way (§2.6.8).

The consequence is that a prior is only as good as the linearisation points it was built at. VI-DSO names the problem in one sentence, "This procedure fixes the linearization points of the states connected to the old states, so they should already have a good estimate. In our scenario this is the case for all variables except of scale" [2, §III-F.3]. The size of the error can be derived from the inertial residual of §2.3.1. Let the prior have been built at log scale $\bar{\sigma}$, $\bar{s} = e^{\bar{\sigma}}$, and let the current estimate be $\sigma = \bar{\sigma} + \Delta\sigma$. Two errors arise.

1. Error in the Jacobians with respect to the states. By §2.3.2, $\partial\mathbf{r}_p/\partial\delta\mathbf{p}_1 = s\,\mathbf{R}_0^\top$. The prior holds it at $\bar{s}\,\mathbf{R}_0^\top$, while at the current estimate it is $s\,\mathbf{R}_0^\top$. The relative error is

$$\frac{s - \bar{s}}{\bar{s}} = e^{\Delta\sigma} - 1,$$

and since $\mathbf{H} = \mathbf{J}^\top \mathbf{J}$, the information the prior assigns to positions and velocities is off by the factor $e^{2\Delta\sigma}$.

2. Error in the dependence on the scale itself. The scaled displacement $s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t)$ equals $\bar{s}\,e^{\Delta\sigma}(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t)$ exactly, while the prior's linear model in $\sigma$ represents it as $\bar{s}(1 + \Delta\sigma)(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t)$. The relative error is

$$e^{\Delta\sigma} - 1 - \Delta\sigma = \frac{1}{2}\Delta\sigma^2 + O(\Delta\sigma^3).$$

Both errors grow with $|\Delta\sigma|$ and neither can be removed by further iterations, because the prior's Jacobians are fixed. On the SITL take offs the scale is wrong by a factor of 2.5 to 5 within the first seconds, the speed ratio of `context/takeoff_bias_divergence.md` §3 and §8, so a prior built then holds that scale with stiff, wrong information. This is the formal statement of R5, in which the scale fixed during the take off is held by the prior while the bias is later repaired (`context/takeoff_bias_divergence.md` §8).

#### 2.6.4 From the problem to dynamic marginalisation

The design follows from comparing the options for the inertial information that a departing frame carries.

1. Keep it in one prior with first estimates, as Basalt does. This is consistent, but by §2.6.3 the prior is wrong in proportion to $|\Delta\sigma|$ and grows stiffer with every marginalisation.
2. Relinearise the prior at the current scale. This would require the removed variables, which are gone, and using the current values for the Jacobians of a frozen prior breaks the first estimate rule of §2.6.3.
3. Discard it. This is consistent and never wrong, but the scale is then estimated from the few inertial factors inside the window, and the long baseline information that makes the scale observable is lost.

VI-DSO's idea sits between 1 and 3. A prior is only harmful if the scales at which its inertial factors were linearised are far from the current scale. So the estimator keeps several priors, each built only from inertial factors linearised within a bounded range of scales, and when the current scale leaves the range of the prior in use, that prior is replaced by one whose factors are closer to the current scale. In the paper's words, "The main idea of 'Dynamic marginalization' is to maintain several marginalization priors at the same time and reset the one we currently use when the scale estimate moves too far from the linearization point in the marginalization prior" [2, §III-F.3]. Three requirements shape the construction.

1. There must be a prior that is valid at every scale, to fall back on. Visual factors that do not depend on the scale provide it.
2. The prior in use must contain only inertial factors linearised within a bounded distance of the current scale, which bounds both errors of §2.6.3.
3. After a replacement, the prior in use must still contain some inertial information, or the scale estimate loses its memory at the moment it was moving.

#### 2.6.5 The three priors

VI-DSO splits the factor graph into $G_\text{metric}$, the inertial factors, which depend on the scale, and $G_\text{visual}$, every other factor except the priors [2, eq. 12]. The optimisation always uses $G_\text{metric} \cup G_\text{visual} \cup M_\text{curr}$ [2, eq. 13]. When frame $i$ is marginalised, each prior is updated by marginalising frame $i$ from a different graph [2, §III-F.3].

1. $M_\text{curr}$, the prior in use, from $G_\text{metric} \cup G_\text{visual} \cup M_\text{curr}$. It contains every factor marginalised since its scale interval was last reset.
2. $M_\text{half}$, from $G_\text{metric} \cup G_\text{visual} \cup M_\text{half}$. It contains only the factors marginalised while the scale stayed in the half of the interval in which it currently lies, "the recent states that have a scale close to the current estimate".
3. $M_\text{visual}$, from $G_\text{visual} \cup M_\text{visual}$. It contains "all marginalized visual factors and no marginalized inertial factors making it independent of the scale", and is the fallback of requirement 1. In this plan it also contains the initial prior.

In VI-DSO the photometric residual is evaluated in a scale free frame, so its visual factors are scale independent by construction. In this plan the visual residual is metric and its scale column is generally not zero (§2.2.4), so $M_\text{visual}$ is built with the scale column of its visual factors set to zero. By §2.2.4 that column carries only the extrinsic lever arm information, which is small for a monocular rig, so the loss is minor and $M_\text{visual}$ becomes exactly scale independent, as requirement 1 needs. With a zero lever arm the per landmark QR already annihilates that column (§2.2.4), so zeroing it changes nothing. With a nonzero lever arm the zeroed factor is no longer invariant along $\mathbf{n}_\sigma$ and adds a small stiffness along it once $M_\text{visual}$ enters $M_\text{curr}$, which the projection of the system in §2.5.6 removes. The initial prior is scale independent because its pins are invariant along $\mathbf{n}_\sigma$ (§2.5.4) and its bias weights act on body frame quantities.

Each prior carries its own linearisation point of the gravity and scale block, $(\bar{\mathbf{d}}, \bar{\mathbf{B}}, \bar{\sigma})$, recorded when that prior first absorbed a factor that depends on the block, which matches the paper's remark that "Mcurr and Mhalf have separate First Estimate Jacobians". Frame states share their first estimates across the three priors, because all three marginalise the same frames at the same instant from the same frozen points.

#### 2.6.6 Interval constraint and the swap rule

Let $\sigma_i$ be the log scale when frame $i$ was marginalised [2, eq. 14], $\sigma_\text{curr}$ the current log scale, $\sigma_\text{mid}$ the middle of the current interval, initialised to $\sigma$ at the start of the period, and $d_i > 1$ the interval size. Write $i \in M$ when $M$ contains an inertial factor marginalised at time $i$. VI-DSO's constraints [2, eq. 15-16] are multiplicative in $s$. Taking logarithms, $s_i \in [s_\text{mid}/d_i,\ s_\text{mid}\,d_i]$ becomes $\sigma_i \in [\sigma_\text{mid} - \ln(d_i),\ \sigma_\text{mid} + \ln(d_i)]$, so the constraints read

$$\forall i \in M_\text{curr}: \ \sigma_i \in [\sigma_\text{mid} - \ln(d_i),\ \sigma_\text{mid} + \ln(d_i)],$$

$$\forall i \in M_\text{half}: \ \sigma_i \in \begin{cases}[\sigma_\text{mid},\ \sigma_\text{mid} + \ln(d_i)] & \text{if } \sigma_\text{curr} > \sigma_\text{mid},\\ [\sigma_\text{mid} - \ln(d_i),\ \sigma_\text{mid}] & \text{otherwise.}\end{cases}$$

In this plan every log scale above is expressed in the current frame's coordinates. The re-centring of §2.7 subtracts $\sigma_c$ from $\sigma_\text{mid}$, from the previous log scale and from every anchor (§2.7.1 item 5), so every difference between them is a difference of cumulative scale and the constraints mean the same as in VI-DSO. They are enforced by VI-DSO's Algorithm 1 [2], run before each marginalisation of the period.

```
upper ← σ_curr > σ_mid
if upper ≠ lastUpper:                          side changed
    M_half ← M_visual
if σ_curr > σ_mid + ln d_i:                    upper boundary exceeded
    M_curr ← M_half;  M_half ← M_visual;  σ_mid ← σ_mid + ln d_i
if σ_curr < σ_mid − ln d_i:                    lower boundary exceeded
    M_curr ← M_half;  M_half ← M_visual;  σ_mid ← σ_mid − ln d_i
lastUpper ← upper
```

The paper asserts three properties of this rule. Each is derived below.

Property 1, the rule preserves the constraints. Consider the three branches. A side change resets $M_\text{half}$ to $M_\text{visual}$, which contains no inertial factor, so the constraint on $M_\text{half}$ holds trivially for the new side, and $M_\text{curr}$ is untouched. An upper boundary crossing means $\sigma_\text{curr} > \sigma_\text{mid} + \ln(d_i)$. Before it, $M_\text{half}$ held factors with $\sigma_i \in [\sigma_\text{mid},\ \sigma_\text{mid} + \ln(d_i)]$. After it, $M_\text{curr}$ is that prior and the middle is $\sigma_\text{mid}' = \sigma_\text{mid} + \ln(d_i)$, so the new full interval is

$$[\sigma_\text{mid}' - \ln(d_i),\ \sigma_\text{mid}' + \ln(d_i)] = [\sigma_\text{mid},\ \sigma_\text{mid} + 2\ln(d_i)],$$

which contains every $\sigma_i$ of the old $M_\text{half}$, so the constraint on $M_\text{curr}$ holds again. $M_\text{half}$ is reset to $M_\text{visual}$ and holds trivially. The lower crossing is the mirror image.

Property 2, the scale error inside the prior in use is bounded by $d_i^2$. After the rule has run, $\sigma_\text{curr}$ lies inside the full interval, since otherwise a crossing would have moved it, and every $\sigma_i$ of $M_\text{curr}$ lies inside it by Property 1. Two points of an interval of width $2\ln(d_i)$ differ by at most that width,

$$|\sigma_\text{curr} - \sigma_i| \le 2\ln(d_i),$$

and exponentiating,

$$\frac{1}{d_i^2} \le \frac{s_\text{curr}}{s_i} \le d_i^2,$$

which is the paper's statement that "the scale difference in the currently used marginalization factor stays smaller than $d_i^2$". Inserting $\Delta\sigma = 2\ln(d_i)$ into §2.6.3 bounds both errors. With $d_i = d_\text{min} = \sqrt{1.1}$, $2\ln(d_i) = \ln(1.1) \approx 0.095$, so the Jacobians with respect to the states are off by at most $e^{0.095} - 1 = 10\%$ and the scale dependence by at most $\frac{1}{2}(0.095)^2 \approx 0.5\%$ of the displacement. The bound uses the current $d_i$. When $d_i$ has grown since some factors were absorbed, the bound for those factors uses the larger value.

Property 3, the prior in use never loses all inertial information. A replacement $M_\text{curr} \leftarrow M_\text{half}$ leaves no inertial information only if $M_\text{half}$ had been reset to $M_\text{visual}$ by a side change in the same marginalisation step. A side change in step $i$ means $\sigma_{i-1}$ and $\sigma_i$ lie on opposite sides of $\sigma_\text{mid}$, so $|\sigma_i - \sigma_\text{mid}| \le |\sigma_i - \sigma_{i-1}|$. A crossing in the same step means $|\sigma_i - \sigma_\text{mid}| > \ln(d_i)$. Together they require

$$|\sigma_i - \sigma_{i-1}| > \ln(d_i).$$

The interval size is chosen to make this impossible. VI-DSO's eq. 17 [2] is printed as $d_i = \min\{d_\text{min}^j \mid j \in \mathbb{N}\setminus\{0\},\ s_i/s_{i-1} < d_i\}$, the smallest power of $d_\text{min}$ that exceeds the ratio of the scale estimates at two consecutive marginalisations. In log space and with an absolute value,

$$d_i = \min\big\{d_\text{min}^j \ \big|\ j \in \mathbb{N}\setminus\{0\},\ e^{|\sigma_i - \sigma_{i-1}|} < d_\text{min}^j\big\},$$

which gives $|\sigma_i - \sigma_{i-1}| < \ln(d_i)$ and contradicts the requirement above. The absolute value is this plan's reading, because the printed ratio $s_i/s_{i-1} < d_i$ would place no constraint on a decreasing scale. The paper and VI-Stereo-DSO use $d_\text{min} = \sqrt{1.1}$ (`d_min = sqrt(1.1)`, `VI-Stereo-DSO/src/main_dso_pangolin.cpp:700`). The derivation also holds when $\sigma_\text{mid}$ moved at step $i - 1$. A crossing at step $i - 1$ places $\sigma_{i-1}$ beyond the new middle, on the side recorded in `lastUpper`, so the side of $\sigma_{i-1}$ is the same with respect to the old and the new middle.

The rule therefore trades information for consistency in a controlled way. The size $d_i$ should be small to keep the errors of §2.6.3 small, but not so small that $M_\text{curr}$ is replaced so often that it holds little inertial information. The paper resolves this by adapting $d_i$ to how fast the scale moves [2, eq. 17].

#### 2.6.7 Updating each prior

At every marginalisation of the period each prior $M \in \{M_\text{curr}, M_\text{half}, M_\text{visual}\}$ is updated by the existing procedure of `context/vio_opt.md` §12.2 to §12.4, applied to the factor set of §2.6.5 and with the gravity and scale block anchored at that prior's own linearisation point.

| Prior | Factors linearised | Visual $\sigma$ column | Block anchored at |
|---|---|---|---|
| $M_\text{curr}$ | selected landmarks, inertial factors, $M_\text{curr}$ | exact | $M_\text{curr}$'s point |
| $M_\text{half}$ | selected landmarks, inertial factors, $M_\text{half}$ | exact | $M_\text{half}$'s point, or the current value if unset |
| $M_\text{visual}$ | selected landmarks, $M_\text{visual}$ | zero | irrelevant, the $\sigma$ columns are zero |

The gravity and scale block is always kept, never marginalised, so `idx_to_keep` gains columns 0 to 2 and the new order is `[gs, surviving poses…, last_state_to_marg]`. The re-centring $\mathbf{b} \leftarrow \mathbf{b} - \mathbf{H} \boldsymbol{\delta}$ of `sqrt_keypoint_vio.cpp:1494-1496` is applied to each prior with $\boldsymbol{\delta}$ computed against that prior's own anchor. For $M_\text{visual}$ the velocities and biases of the departing states are constrained by no factor, so their columns are dropped by the rank threshold as noted in §2.6.2.

#### 2.6.8 Freezing the block and swapping anchors

The block's linearisation semantics mirror `PoseStateWithLin` (`include/basalt/utils/imu_types.h:186-289`). It is unfrozen at the start of the period, when every prior has zero gravity and scale columns, so its Jacobians follow the current value and its offset $\boldsymbol{\delta}_{gs}$ is zero. It is frozen at the first marginalisation of the period, when $M_\text{curr}$'s anchor is first recorded. An anchor is the triple $(\bar{\mathbf{d}}_a, \bar{\mathbf{B}}_a, \bar{\sigma}_a)$. When a prior's anchor is made active, the block is re-anchored there while its current value is kept,

$$\bar{\mathbf{d}} \leftarrow \bar{\mathbf{d}}_a,$$

$$\bar{\mathbf{B}} \leftarrow \bar{\mathbf{B}}_a,$$

$$\bar{\sigma} \leftarrow \bar{\sigma}_a,$$

$$\boldsymbol{\delta}_\theta \leftarrow \bar{\mathbf{B}}_a^\top\,\mathbf{w},$$

$$\delta\sigma \leftarrow \sigma - \bar{\sigma}_a,$$

where $\mathbf{w}$ is the rotation vector carrying $\bar{\mathbf{d}}_a$ onto the current $\mathbf{d}$,

$$\mathbf{w} = \text{atan2}\big(|\bar{\mathbf{d}}_a \times \mathbf{d}|,\ \bar{\mathbf{d}}_a\cdot\mathbf{d}\big)\ \frac{\bar{\mathbf{d}}_a \times \mathbf{d}}{|\bar{\mathbf{d}}_a \times \mathbf{d}|}.$$

$\mathbf{w}$ is perpendicular to $\bar{\mathbf{d}}_a$, so it lies in the span of $\bar{\mathbf{B}}_a$ and $\bar{\mathbf{B}}_a\bar{\mathbf{B}}_a^\top \mathbf{w} = \mathbf{w}$. Hence $\text{Exp}(\bar{\mathbf{B}}_a\boldsymbol{\delta}_\theta)\,\bar{\mathbf{d}}_a = \text{Exp}(\mathbf{w})\,\bar{\mathbf{d}}_a = \mathbf{d}$, and the re-anchoring is exact. It is used in three places, in the swap $M_\text{curr} \leftarrow M_\text{half}$ of Algorithm 1, before each of the three marginalisations so that each prior is linearised at its own anchor, and before each prior's re-centring $\mathbf{b} \leftarrow \mathbf{b} - \mathbf{H}\boldsymbol{\delta}$. When an anchor has never been recorded, it is recorded at the current value with the transported basis, which is the moment of freezing. When $M_\text{half}$ has just been reset to $M_\text{visual}$, its gravity and scale columns are zero and its anchor is cleared, so the next marginalisation into it records a fresh anchor.

#### 2.6.9 What the paper shows and what it does not

VI-DSO presents dynamic marginalisation with one qualitative illustration, the scale estimate, the interval boundaries, the side changes and the boundary crossings on EuRoC V1_03 [2, Fig. 4], where by its caption "Mcurr contains the inertial factors since the last blue or red dotted line that is before the last red dotted line". Its quantitative results [2, §IV] evaluate the whole system against DSO and ROVIO. No experiment in the paper compares dynamic marginalisation with a single prior, so the benefit of the technique in isolation is argued from the consistency analysis above, not measured. The bounds of Properties 2 and 3 are derived here and were asserted there without proof. This plan therefore treats the benefit as a hypothesis, which the take off replays of §4.3 test, item 4 by the swap events of `vio_gravity_scale` and item 3 by the scale and bias outcomes.

### 2.7 Per Frame Re-centring

#### 2.7.1 The change of variables

At the end of every `optimize` call in the period, with the block's current values $\mathbf{d}_c$ and $\sigma_c$, let

$$\mathbf{Q} = \text{FromTwoVectors}(\mathbf{d}_c, \hat{\mathbf{g}}),$$

$$s_c = e^{\sigma_c},$$

and let $\Phi_c$ be the map that acts on every rotation, position and velocity as

$$\Phi_c(\mathbf{R}) = \mathbf{Q} \mathbf{R},$$

$$\Phi_c(\mathbf{p}) = s_c\mathbf{Q} \mathbf{p},$$

$$\Phi_c(\mathbf{v}) = s_c\mathbf{Q} \mathbf{v}.$$

The re-centring replaces every variable of the estimator by new coordinates in which the correction just estimated has been absorbed into the states.

1. Every pose and state, both its current value and its linearisation point, is mapped by $\Phi_c$. Biases are unchanged.
2. Every landmark is unchanged (§2.1.4).
3. The gravity and scale block's linearisation point becomes $\bar{\mathbf{d}}' = \mathbf{Q}\bar{\mathbf{d}}$, $\bar{\mathbf{B}}' = \mathbf{Q}\bar{\mathbf{B}}$, $\bar{\sigma}' = \bar{\sigma} - \sigma_c$, and its increment $\boldsymbol{\delta}_{gs}$ is unchanged. The same map is applied to the anchors of $M_\text{curr}$ and $M_\text{half}$.
4. The prior of each of the three priors becomes $\mathbf{J}' = \mathbf{J} \mathbf{M}^{-1}$, $\mathbf{r}' = \mathbf{r}$, where $\mathbf{M}$ is the block diagonal map of the tangent offsets defined below.
5. The interval variables of Algorithm 1 are shifted, $\sigma_\text{mid} \leftarrow \sigma_\text{mid} - \sigma_c$, $\sigma_\text{prev} \leftarrow \sigma_\text{prev} - \sigma_c$.

After step 3 the block's current value is $\text{Exp}(\mathbf{Q} \bar{\mathbf{B}} \boldsymbol{\delta}_\theta) \mathbf{Q} \bar{\mathbf{d}} = \mathbf{Q} \mathbf{d}_c = \hat{\mathbf{g}}$ by the second property of §2.1.3, and $\bar{\sigma} - \sigma_c + \delta\sigma = 0$, so $s = 1$ and $\mathbf{g}_D = \mathbf{g}$ exactly. The correction has been reset to the identity while the information about it is kept in the prior's columns.

#### 2.7.2 The residuals are invariant

Write $\mathbf{g}_D = |\mathbf{g}|\,\mathbf{d}_c$ before the re-centring and $\mathbf{g}_D' = \mathbf{Q} \mathbf{g}_D = \mathbf{g}$ after. For the inertial residual of §2.3.1, the new states give $\mathbf{R}_0'^\top = \mathbf{R}_0^\top \mathbf{Q}^\top$, $\mathbf{v}' = s_c\mathbf{Q} \mathbf{v}$, $\mathbf{p}' = s_c\mathbf{Q} \mathbf{p}$, and the new scale $s' = s / s_c$, so

$$\mathbf{R}_0'^\top\big(s'(\mathbf{v}_1' - \mathbf{v}_0') - \mathbf{g}_D'\Delta t\big) = \mathbf{R}_0^\top \mathbf{Q}^\top\Big(\frac{s}{s_c}\,s_c\mathbf{Q}(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{Q} \mathbf{g}_D\Delta t\Big) = \mathbf{R}_0^\top\big(s(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D\Delta t\big),$$

and identically for $\mathbf{r}_p$, while $\mathbf{r}_R$ depends only on $\mathbf{R}_1^\top \mathbf{R}_0$, which $\Phi_c$ preserves. For the visual residual of §2.2.1, $\mathbf{t}_{ti,hi}' = \mathbf{R}_t'^\top(\mathbf{p}_h' - \mathbf{p}_t') = s_c\,\mathbf{t}_{ti,hi}$ and $s'\,\mathbf{t}_{ti,hi}' = s\,\mathbf{t}_{ti,hi}$, so $\mathbf{T}_{t,h}$ is unchanged. The bias residuals and the landmark parameters are unchanged. Every residual therefore has exactly the same value before and after, and so does its Jacobian once the tangent coordinates are mapped as below.

#### 2.7.3 The tangent offsets map linearly

For a frozen state with $\mathbf{p} = \bar{\mathbf{p}} + \delta\mathbf{p}$, $\mathbf{R} = \text{Exp}(\delta\boldsymbol{\varphi})\,\bar{\mathbf{R}}$ and $\mathbf{v} = \bar{\mathbf{v}} + \delta\mathbf{v}$, mapping both the current value and the linearisation point gives $\delta\mathbf{p}' = s_c\mathbf{Q}\,\delta\mathbf{p}$, $\delta\mathbf{v}' = s_c\mathbf{Q}\,\delta\mathbf{v}$ and $\mathbf{Q}\,\text{Exp}(\delta\boldsymbol{\varphi})\,\bar{\mathbf{R}} = \text{Exp}(\mathbf{Q}\,\delta\boldsymbol{\varphi})\,\mathbf{Q}\bar{\mathbf{R}}$, so $\delta\boldsymbol{\varphi}' = \mathbf{Q}\,\delta\boldsymbol{\varphi}$. The offset of a state transforms by $\mathbf{A} = \text{blkdiag}(s_c\mathbf{Q}, \mathbf{Q}, s_c\mathbf{Q}, \mathbf{I}_3, \mathbf{I}_3)$ and that of a pose by $\text{blkdiag}(s_c\mathbf{Q}, \mathbf{Q})$. The block's offset is unchanged by step 3, because $\text{Exp}(\mathbf{Q}\bar{\mathbf{B}}\boldsymbol{\delta}_\theta)\,\mathbf{Q}\bar{\mathbf{d}} = \mathbf{Q}\,\text{Exp}(\bar{\mathbf{B}}\boldsymbol{\delta}_\theta)\,\bar{\mathbf{d}}$ (§2.1.3) and the log scale offset is a difference. With $\mathbf{M} = \text{blkdiag}(\mathbf{I}_3, \mathbf{A}_1, \mathbf{A}_2, \ldots)$ in `aom` order, every prior $\frac{1}{2}|\mathbf{J}(\boldsymbol{\delta} + \mathbf{x}) + \mathbf{r}|^2$ becomes $\frac{1}{2}|\mathbf{J} \mathbf{M}^{-1}(\boldsymbol{\delta}' + \mathbf{x}') + \mathbf{r}|^2$ exactly, with

$$\mathbf{A}^{-1} = \text{blkdiag}\Big(\frac{\mathbf{Q}^\top}{s_c},\ \mathbf{Q}^\top,\ \frac{\mathbf{Q}^\top}{s_c},\ \mathbf{I}_3,\ \mathbf{I}_3\Big),$$

applied as a right multiplication of each three column block of the frame columns of $\mathbf{J}$. The gravity and scale columns are untouched. Since every factor value is invariant and every offset maps exactly, the re-centring changes no cost and no Jacobian, only the coordinates. The first estimate consistency of the frozen states is preserved, because their Jacobians at the mapped linearisation points equal the old Jacobians mapped by $\mathbf{A}$.

#### 2.7.4 Why the prior is re-expressed and not conditioned

Conditioning the prior on the block, $\mathbf{r} \leftarrow \mathbf{r} + \mathbf{J}_{gs}\boldsymbol{\delta}_{gs}$ and dropping its columns, would also leave $s = 1$ and $\mathbf{g}_D = \mathbf{g}$, but it would turn the prior's information about the block into information about the frame positions, velocities and orientations along $\mathbf{n}_\sigma$ and $\mathbf{n}_\mathbf{a}$ of §2.5.3. The gauge projection of the next solve removes exactly those directions, so the accumulated scale and gravity information would be erased at every frame, and the block would be re-estimated from the two or three inertial factors inside the window. The re-expression of §2.7.3 keeps the block's columns, so the marginalised inertial information keeps accumulating across frames, as VI-DSO's similarity variable does for the whole run [2].

#### 2.7.5 Where it runs

The re-centring is the last action of `optimize`, after the Levenberg–Marquardt loop has finished and before the solver summary is logged. Within a frame the call sequence of `measure` is therefore prediction, association, keyframe decision, triangulation, `optimize` ending in the re-centring, `marginalize`, and `PublishKeyframe`, followed by the publication of the state and the visualisation payload. Everything after `optimize` sees metric, gravity aligned states, so `marginalize`, the keyframe sent to the local mapper, the marginalisation packet, the published state, the logger and the visualiser need no conversion. Everything before `optimize` in the next frame sees $s = 1$ and $\mathbf{g}_D = \mathbf{g}$, so prediction and triangulation use today's code (§2.3.4). When `optimize` does not run, during the first four frames (`sqrt_keypoint_vio.cpp:1530`), the block has not moved and nothing is re-centred.

Each re-centring appends one entry to the correction log of §2.9, which advances the estimator's epoch by one.

### 2.8 End of the Period

The block is removed once, in the first frame whose timestamp is at least `vio_gs_duration_s` after the first frame of confirmed motion (`plans/gravity_scale_timer_from_motion.md`), as the last action of `marginalize`, after the marginalisation block and the re-centring of the three priors and immediately before the final `SolverScratch` timing entry. It sits outside the `if (frame_poses.size() > max_kfs || frame_states.size() >= max_states)` block, so it also runs in a frame that marginalises nothing. Note, 2026-11-12. This paragraph previously measured the duration from the first frame, the timestamp `SetInitialPriorOrder` stored in `mpGsStartNs`, which the timer change moved to take off.

Because `optimize` has already re-centred the block in this frame, its current value is the identity, so removing it changes no state and no landmark. The prior is conditioned on the block at its current value, $\mathbf{r} \leftarrow \mathbf{r} + \mathbf{J}_{gs}\boldsymbol{\delta}_{gs}$ with $\boldsymbol{\delta}_{gs}$ the block's offset from $M_\text{curr}$'s anchor, and the three gravity and scale columns are dropped, which shifts every order entry by $-3$. $M_\text{half}$ and $M_\text{visual}$ are discarded, the gauge projection is switched off, and the next `optimize` call is today's code path. Conditioning loses the prior's remaining information about the block, which is intended. After the period, scale and gravity are again carried implicitly by the states, as today.

### 2.9 Keeping the Local Mapper Consistent

#### 2.9.1 Epochs and the correction log

The local mapper runs on its own thread and holds keyframe poses, landmarks and nonlinear factors that did not take part in the estimator's solve. Each re-centring changes the estimator's metric world, so the mapper must apply the same similarity to its map, and every quantity that crosses between the two threads must say in which world it is expressed.

Let $\Phi_e = (\mathbf{Q}_e, s_e)$ be the correction applied at the $e$-th re-centring. The correction log stores the cumulative similarity of each epoch,

$$C_0 = (\mathbf{I}, 1),$$

$$C_e = \Phi_e \circ C_{e-1} = (\mathbf{Q}_e \mathbf{Q}_{C_{e-1}},\ s_e\,s_{C_{e-1}}),$$

using the composition of similarities without translation, $(\mathbf{Q}_2, s_2)\circ(\mathbf{Q}_1, s_1) = (\mathbf{Q}_2 \mathbf{Q}_1, s_2 s_1)$, which follows from $\mathbf{x} \mapsto s_2\mathbf{Q}_2(s_1\mathbf{Q}_1 \mathbf{x})$. A quantity expressed at epoch $a$ is carried to epoch $b$ by

$$C_b\,C_a^{-1} = \Big(\mathbf{Q}_{C_b} \mathbf{Q}_{C_a}^\top,\ \frac{s_{C_b}}{s_{C_a}}\Big),$$

which is valid for $b < a$ as well. The log is owned by the estimator, appended only by the re-centring, read by both threads under a mutex, and holds one entry per frame of the period, about two hundred entries for ten seconds at twenty frames per second. After the period it stops growing.

Every keyframe and marginalisation packet the estimator emits is stamped with the latest epoch at emission. Because the re-centring of a frame runs before that frame's `marginalize` and `PublishKeyframe`, the stamp always names the world the contents are expressed in. The mapper keeps its own map epoch, and every pose update it sends back is stamped with it.

#### 2.9.2 Applying a correction to the mapper's map

Let $(\mathbf{Q}, s) = C_\text{now}\,C_\text{map}^{-1}$. The mapper applies it at the start of each cycle, before it ingests that cycle's keyframe and packets.

1. Keyframe poses. $\mathbf{R} \leftarrow \mathbf{Q} \mathbf{R}$, $\mathbf{p} \leftarrow s\,\mathbf{Q} \mathbf{p}$. The mapper's poses are never frozen, since `NfrMapper::optimize` asserts `!isLinearized()` (`src/vi_estimator/nfr_mapper.cpp:337`, `:410`), so the pose is rebuilt directly.
2. Landmarks. The bearing is relative to the host camera and is unchanged. The inverse distance becomes $\rho \leftarrow \rho / s$, so that the landmark's world position transforms like every other point, $\mathbf{X} \leftarrow s\,\mathbf{Q} \mathbf{X}$. To check this, write the landmark's position in the host camera as $\mathbf{m}(\boldsymbol{\theta})/\rho$. Its world position $\mathbf{T}_{w,c_h}\,\mathbf{m}/\rho$ becomes $\mathbf{T}_{w,c_h}'\,\mathbf{m}\,s/\rho$ with $\mathbf{T}_{w,c_h}' = (\mathbf{Q} \mathbf{R}_{w,c_h}, s\mathbf{Q} \mathbf{p}_{w,c_h})$, and $\mathbf{Q} \mathbf{R}_{w,c_h}\,\mathbf{m}\,s/\rho + s\mathbf{Q} \mathbf{p}_{w,c_h} = s\mathbf{Q}(\mathbf{R}_{w,c_h}\,\mathbf{m}/\rho + \mathbf{p}_{w,c_h})$. The camera to IMU extrinsics are metric and are not scaled, so this holds exactly only for the IMU frame's translation. The camera centre $\mathbf{p}_{w,c_h} = \mathbf{p} + \mathbf{R}\,\mathbf{t}_{i,c}$ maps to $s\mathbf{Q} \mathbf{p} + \mathbf{Q} \mathbf{R}\,\mathbf{t}_{i,c}$, not to $s\mathbf{Q}(\mathbf{p} + \mathbf{R}\,\mathbf{t}_{i,c})$. The discrepancy is $(1 - s)\,\mathbf{Q} \mathbf{R}\,\mathbf{t}_{i,c}$ per landmark, a few millimetres for a 5 % scale correction and a lever arm of a few centimetres. It is the same lever arm effect that makes scale weakly observable to vision (§2.2.4), and the mapper's next bundle adjustment absorbs it.
3. Relative pose factors. A factor measures $\mathbf{T}_{i,j} = \mathbf{T}_{w,i}^{-1} \mathbf{T}_{w,j}$ with information $\boldsymbol{\Lambda}$ on the residual $\text{Log}(\mathbf{T}_{i,j}^{-1} \mathbf{T}_{w,i}^{-1} \mathbf{T}_{w,j})$ (`include/basalt/utils/nfr.h:43-46`). A world rotation leaves $\mathbf{T}_{i,j}$ unchanged, and a world scaling multiplies its translation by $s$. By (S1) and (S2) of §2.2.2, $\mathcal{S}_s$ preserves products and acts on tangent vectors through $\mathbf{D}_s$, so the residual of the scaled poses and scaled measurement is $\mathbf{D}_s$ times the old residual. The factor's cost $\mathbf{r}^\top\boldsymbol{\Lambda} \mathbf{r}$ is unchanged when $\mathbf{T}_{i,j} \leftarrow \mathcal{S}_s(\mathbf{T}_{i,j})$ and $\boldsymbol{\Lambda} \leftarrow \mathbf{D}_s^{-1}\boldsymbol{\Lambda} \mathbf{D}_s^{-1}$.
4. Roll and pitch factors. A factor stores the orientation estimate $\mathbf{R}_\text{meas}$ from which the body's gravity direction was taken (`nfr_mapper.cpp:200-212`). The correction rotates the world, so $\mathbf{R}_\text{meas} \leftarrow \mathbf{Q} \mathbf{R}_\text{meas}$, which keeps the factor saying that the body's attitude relative to gravity is what the estimator now believes. `rollPitchError` (`nfr.h:102-119`) compares the two attitudes through the fixed world z axis. Writing the attitude difference as $\text{Exp}(\boldsymbol{\epsilon})$, the residual is to first order the horizontal part of $\boldsymbol{\epsilon}$, and after the correction the difference becomes $\mathbf{Q}\,\text{Exp}(\boldsymbol{\epsilon})\,\mathbf{Q}^\top = \text{Exp}(\mathbf{Q} \boldsymbol{\epsilon})$ by (I4). For a correction angle $\alpha$ the residual therefore changes by terms of order $\alpha|\boldsymbol{\epsilon}|$, a product of two small quantities, so the factor is preserved only to first order. No choice of $\mathbf{Q}$ makes it exact in general, since even a rotation about the vertical rotates the residual within its plane, which leaves the cost unchanged only when the factor's $2\times 2$ information matrix is isotropic. The mapper's next bundle adjustment absorbs the change. Note, 2026-11-05. This item previously stated that the residual is exactly unchanged when $\mathbf{Q}$ is a rotation about the vertical, which holds only for an isotropic information matrix.

The stereo `MatchData::T_i_j` written by `match_stereo` (`nfr_mapper.cpp:562`) is the fixed metric extrinsic between the two cameras of one rig and is not affected. The track store and the bag of words index hold no geometry.

#### 2.9.3 Bringing incoming items to the mapper's epoch

A keyframe stamped with epoch $e$ is carried to the mapper's epoch by the pose transform of item 1, with $(\mathbf{Q}, s) = C_\text{map}\,C_e^{-1}$. A marginalisation packet stamped with epoch $e$ holds frame poses and states, a dense `abs_H` and `abs_b` over `aom`, and the keyframe sets. Its poses and states are mapped like the estimator's in §2.7.3, current value and linearisation point together. Its quadratic form $\frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H}\boldsymbol{\delta} + \mathbf{b}^\top\boldsymbol{\delta}$ over offsets that transform as $\boldsymbol{\delta}' = \mathbf{A}\boldsymbol{\delta}$ becomes

$$\mathbf{H}' = \mathbf{A}^{-\top} \mathbf{H}\,\mathbf{A}^{-1},$$

$$\mathbf{b}' = \mathbf{A}^{-\top} \mathbf{b},$$

with $\mathbf{A}$ block diagonal over the packet's `aom`, applied block by block to the columns and rows of $\mathbf{H}$ and to the segments of $\mathbf{b}$. This is exact.

The estimator removes the gravity and scale block from a packet before it is pushed. The packet's $\mathbf{H}$ is the dense Hessian of the estimator's $M_\text{curr}$ marginalisation, which includes the block's three columns, and `NfrMapper::processMargData` aborts on an order entry of size 3 (`nfr_mapper.cpp:122-125`). The estimator therefore conditions the packet on the block at its current value and shifts every remaining order entry by $-3$. The packet's linear model is written in increments from the current estimate, since every residual is evaluated at the current values and the prior rows are $\mathbf{J}_m \boldsymbol{\delta} + \mathbf{r}_m$ (`linearization_abs_qr.cpp:624-625`). Holding the block at its current value is therefore a zero block increment, and conditioning reduces to deleting the block's three rows and columns of $\mathbf{H}$ and its three entries of $\mathbf{b}$. After the re-centring of §2.7 that current value is the identity, so the packet says what the frames are with $s = 1$ and $\mathbf{g}_D = \mathbf{g}$, which is the metric world the mapper works in. Marginalising the block out by the Schur complement instead would destroy the packet. The full $\mathbf{H}$ satisfies $\mathbf{H}\mathbf{n} = \mathbf{0}$ for $\mathbf{n} = [\mathbf{n}_g; \mathbf{n}_f]$, each of $\mathbf{n}_\sigma$, $\mathbf{n}_{\mathbf{e}_x}$ and $\mathbf{n}_{\mathbf{e}_y}$ split into its block part $\mathbf{n}_g$ and frame part $\mathbf{n}_f$ (§2.5.3). The two block rows of that equation give $\mathbf{H}_{gg} \mathbf{n}_g + \mathbf{H}_{gf} \mathbf{n}_f = \mathbf{0}$ and $\mathbf{H}_{fg} \mathbf{n}_g + \mathbf{H}_{ff} \mathbf{n}_f = \mathbf{0}$, so the marginal over the frames annihilates the frame parts,

$$\big(\mathbf{H}_{ff} - \mathbf{H}_{fg} \mathbf{H}_{gg}^{-1} \mathbf{H}_{gf}\big) \mathbf{n}_f = \mathbf{H}_{ff} \mathbf{n}_f + \mathbf{H}_{fg} \mathbf{n}_g = \mathbf{0}.$$

The marginal is therefore singular along the frames only scale and tilt directions, three dimensions in all, and `NfrMapper::extractNonlinearFactors` rejects any packet whose $\mathbf{H}$ is not of full rank (`if (qr.rank() != m.abs_H.cols()) return false;`, `nfr_mapper.cpp:162`), which the local mapper calls for every packet (`local_mapper.cpp:312`). Every packet of the period would be dropped, or, if round off lifted the rank, would yield roll and pitch factors with a nearly infinite covariance. Conditioning keeps the rank, at the price of treating the block as known, so the mapper's factors are slightly overconfident in the metric scale and tilt of the period. Note, 2026-11-05. This paragraph previously specified marginalising the block out with `MargHelper::marginalizeHelperSqToSq` as the exact marginal the mapper expects. The review of the plan against `nfr_mapper.cpp` found the rank deficiency derived above, so conditioning replaced it.

#### 2.9.4 Bringing pose updates back to the estimator

The mapper sends its refined keyframe poses with its map epoch $e$. The estimator stores the epoch next to each staged pose and, when it applies the update at the start of the next `measure`, carries the pose to its own latest epoch by $C_\text{now}\,C_e^{-1}$ before the existing size check and replacement (`sqrt_keypoint_vio.cpp:592-641`). An update computed in an older world is therefore applied in the right world, and the existing thresholds `kRelinThresholdTrans` and `kRelinThresholdRot` judge only the mapper's genuine refinement.

#### 2.9.5 Why this is sufficient

Within a mapper cycle every operation runs in one epoch, because the correction is applied only at the start of the cycle. Every item entering the mapper and every update leaving it carries its epoch, and the transfer between any two epochs is available from the log. Nothing in the mapper is therefore ever combined across worlds, and the local mapper can run from the first keyframe of the period.

### 2.10 What a Subscriber Sees

The published state is metric and gravity aligned at all times. Consecutive published poses differ by the true motion plus the correction estimated in that frame, which is a rescaling and re-tilt about the world origin,

$$\Delta\mathbf{p}^M \approx \delta\sigma\,\mathbf{p}^M + \boldsymbol{\omega}\times \mathbf{p}^M,$$

with $\boldsymbol{\omega}$ the rotation vector of $\mathbf{Q}$. A 5 % scale correction at 10 m from the origin moves the published position by 0.5 m, and a 1° tilt correction there by about 17 cm. Previously published poses are not revised, whereas the local mapper's map and every state inside the estimator are. Nothing changes at the end of the period, because the block is already at the identity when it is removed. If a consumer, for instance the flight controller, needs a smooth signal during the period, either rate limiting the applied correction or feeding it the change explicitly is a separate decision, recorded here as open.

## 3. Implementation Plan

### 3.1 Architecture

#### 3.1.1 Current

```
measure ─► predictState ─► associate ─► triangulate ─► optimize_and_marg
                                                         │
          optimize                                       ▼
          ┌──────────────────────────────────────────────────────────┐
          │ aom = [poses | states]                                     │
          │ LinearizationBase::create ─► LinearizationAbsQR            │
          │     relative poses: computeRelPose ──► RelPoseLin(h,t)     │
          │     LandmarkBlockAbsDynamic: linearizePoint, QR            │
          │     ImuBlock: IntegratedImuMeasurement::residual           │
          │     prior: marg_data (one)                                 │
          │ get_dense_H_b ─► LDLT ─► backSubstitute ─► applyInc        │
          │ after cost: computeError, computeImuError, prior           │
          └──────────────────────────────────────────────────────────┘
          marginalize: one LinearizationAbsQR, MargHelper, one marg_data
                       MargData ─► out_marg_queue ─► LocalMapper
          PublishKeyframe ─► mpKFOutputQueue ─► LocalMapper
          LocalMapper ─► PoseUpdateCallback ─► QueuePoseUpdates ─► measure
```

#### 3.1.2 Updated, in the initialisation period

```
measure ─► predictState ─► associate ─► triangulate ─► optimize_and_marg        (s = 1, g_D = g)
                                                         │
          optimize                                       ▼
          ┌──────────────────────────────────────────────────────────────────┐
          │ aom = [gs(3) | poses | states]       key kGravityScaleStateId     │
          │ LinearizationGSAbsQR : LinearizationAbsQR                         │
          │     linearizeRelPoses (override): computeRelPoseGS                │
          │           ──► RelPoseLin(h,t) + d_rel_d_logscale                  │
          │     LandmarkBlockAbsDynamic: linearizePoint (unchanged),          │
          │           writes σ column when aom holds the gs block, QR        │
          │     ImuBlockGS : ImuBlock: residualGS, gs Jacobian 15 × 3         │
          │     prior: M_curr                                                 │
          │ get_dense_H_b ─► gauge projection P ─► LDLT ─► P x                │
          │ backSubstitute ─► applyInc (+ gs block)                           │
          │ after cost: computeError(s), computeImuError(g_D, s), prior       │
          │ RecentreGravityScale: Φ_c on states, J M⁻¹ on 3 priors,           │
          │           block ─► identity, log.Append(Φ_c)    (every frame)     │
          └──────────────────────────────────────────────────────────────────┘
          marginalize:
              Algorithm 1 swap
              LinearizeMargGS: 3 × LinearizationGSAbsQR (M_curr, M_half, M_visual)
              MargData(M_curr) ─► StripGsFromMargData ─► stamp epoch ─► out_marg_queue
              MarginalizeGS: 3 × MargHelper ─► M_curr, M_half, M_visual
              RecentrePriorsGS
              end of marginalize, once per run, when the period is over:
                  FoldGravityScale: condition M_curr on gs, drop its columns
          PublishKeyframe ─► stamp epoch ─► mpKFOutputQueue
```

#### 3.1.3 Estimator and mapper synchronisation

```
            VIO thread                                    mapper thread
  ┌───────────────────────────────┐              ┌───────────────────────────────────┐
  │ RecentreGravityScale          │  Append      │ MapLocally, each cycle            │
  │   Φ_c ──────────────────────► │ ───────────► │   pop keyframe                    │
  │        GravityScaleCorrection │  (mutex)     │   CatchUpGravityScale             │
  │        Log  C_0, C_1, … C_e   │ ◄─────────── │     Between(map, latest)          │
  │                               │  Between     │     ApplyGravityScaleCorrection   │
  │ PublishKeyframe (epoch e) ────┼─────────────►│   IngestKeyframe: Between(e, map) │
  │ MargData        (epoch e) ────┼─────────────►│   IngestMargData: Between(e, map) │
  │                               │              │   … BA in one epoch …             │
  │ measure: staged update  ◄─────┼──────────────│   PoseUpdateCallback(poses, map)  │
  │   Between(e_map, latest)      │              │                                   │
  └───────────────────────────────┘              └───────────────────────────────────┘
```

#### 3.1.4 Class relations

```
LinearizationBase ◄── LinearizationAbsQR ◄── LinearizationGSAbsQR          (new, same files)
                          │ owns                        │ replaces imu_blocks with
                          ▼                             ▼
                       ImuBlock  (methods made virtual) ◄── ImuBlockGS     (new, imu_block.hpp)
                       LandmarkBlockAbsDynamic          (σ column when gs in aom)
BundleAdjustmentBase  ── mGsState, mGsStateBackup        (new members)
                          computeDelta, computeError     (gs aware, default unchanged)
ScBundleAdjustmentBase::computeImuError                  (optional scale)
IntegratedImuMeasurement  ── residualGS                  (new method)
ba_utils.h               ── computeRelPoseGS             (new function)
imu_types.h              ── GravityScaleStateWithLin, GravityScaleCorrectionLog,
                            WithLin::TransformBySim3, Keyframe/MargData::mGsEpoch
VioEstimatorBase         ── mGsCorrectionLog, QueuePoseUpdates(…, gsEpoch = 0)
LocalMapper              ── CatchUpGravityScale, ApplyGravityScaleCorrection,
                            TransformMargDataBySim3, PoseUpdateCallback(…, epoch)
```

### 3.2 Implementation

The changes are ordered so that each compiles on its own. Every snippet is the exact text to apply, and the prose around it states where it goes and what it replaces. Line anchors are against the current tree and shift as earlier changes land, so each insertion point is also identified by the code it follows. Every change states its backward compatibility impact. New identifiers follow the workspace convention (title case methods, `m` and `mp` member prefixes), except the names fixed by the request (`residualGS`, `computeRelPoseGS`, `LinearizationGSAbsQR`), the lower case methods that override or extend upstream Basalt interfaces (`linearizeImu`, `linearizeRelPoses` and their siblings), and the snake case `VioConfig` keys, which follow `context/vio_config_json.md`.

#### Change 1. Gravity and scale types, `include/basalt/utils/imu_types.h`

Add three includes after `#include <set>` (`:48`).

```cpp
#include <basalt/utils/eigen_utils.hpp>
#include <cmath>
#include <mutex>
#include <vector>
```

Insert into `struct PoseVelBiasStateWithLin`, after `inline int64_t getT_ns() const { return state_linearized.t_ns; }` (`:137`).

```cpp
    // Applies R <- Q R, p <- s Q p, v <- s Q v to the linearisation point and
    // maps delta by blkdiag(sQ, Q, sQ, I, I), plans/gravity_scale_optimisation.md §2.7.
    void TransformBySim3(const Sophus::SO3<Scalar>& Q, Scalar s) {
        const Eigen::Matrix<Scalar, 3, 3> Qm = Q.matrix();
        state_linearized.T_w_i.so3() = Q * state_linearized.T_w_i.so3();
        state_linearized.T_w_i.translation() =
            s * (Qm * state_linearized.T_w_i.translation());
        state_linearized.vel_w_i = s * (Qm * state_linearized.vel_w_i);
        delta.template segment<3>(0) = s * (Qm * delta.template segment<3>(0));
        delta.template segment<3>(3) = Qm * delta.template segment<3>(3);
        delta.template segment<3>(6) = s * (Qm * delta.template segment<3>(6));
        state_current = state_linearized;
        if (linearized) state_current.applyInc(delta);
    }
```

Insert into `struct PoseStateWithLin`, after `inline int64_t getT_ns() const { return pose_linearized.t_ns; }` (`:250`).

```cpp
    // Applies R <- Q R, p <- s Q p to the linearisation point and maps delta by
    // blkdiag(sQ, Q), plans/gravity_scale_optimisation.md §2.7.
    void TransformBySim3(const Sophus::SO3<Scalar>& Q, Scalar s) {
        const Eigen::Matrix<Scalar, 3, 3> Qm = Q.matrix();
        pose_linearized.T_w_i.so3() = Q * pose_linearized.T_w_i.so3();
        pose_linearized.T_w_i.translation() =
            s * (Qm * pose_linearized.T_w_i.translation());
        delta.template head<3>() = s * (Qm * delta.template head<3>());
        delta.template tail<3>() = Qm * delta.template tail<3>();
        T_w_i_current = pose_linearized.T_w_i;
        if (linearized) PoseState<Scalar>::incPose(delta, T_w_i_current);
    }
```

Insert after the closing brace of `struct PoseStateWithLin` (`:289`) and before `struct AbsOrderMap` (`:291`).

```cpp
constexpr int64_t kGravityScaleStateId = -1;
constexpr int GRAVITY_SCALE_SIZE = 3;

// Gravity direction on the unit sphere and log scale with first estimate
// semantics, plans/gravity_scale_optimisation.md §2.1.3.
template <class Scalar_>
struct GravityScaleStateWithLin {
    using Scalar = Scalar_;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat32 = Eigen::Matrix<Scalar, 3, 2>;
    using SO3 = Sophus::SO3<Scalar>;

    Vec3 mDirLin = Vec3(0, 0, -1);
    Mat32 mBasisLin = (Mat32() << 1, 0, 0, 1, 0, 0).finished();
    Scalar mSigmaLin = 0;
    Vec3 mDelta = Vec3::Zero();
    bool mLinearized = false;

    SO3 DeltaRotation() const {
        return SO3::exp(mBasisLin * mDelta.template head<2>());
    }
    Vec3 GetDir() const { return DeltaRotation() * mDirLin; }
    Mat32 GetBasis() const { return DeltaRotation().matrix() * mBasisLin; }
    Scalar GetSigma() const { return mSigmaLin + mDelta[2]; }
    Scalar GetScale() const { return std::exp(GetSigma()); }
    Scalar GetScaleLin() const { return std::exp(mSigmaLin); }
    const Vec3& GetDelta() const { return mDelta; }

    // Minimal rotation carrying the current gravity direction onto that of g.
    SO3 AlignmentRotation(const Vec3& g) const {
        return SO3(Eigen::Quaternion<Scalar>::FromTwoVectors(GetDir(),
                                                             g.normalized()));
    }

    void ApplyInc(const Vec3& inc) {
        if (mLinearized) {
            mDelta += inc;
            return;
        }
        const SO3 R = SO3::exp(mBasisLin * inc.template head<2>());
        mDirLin = R * mDirLin;
        mBasisLin = R.matrix() * mBasisLin;
        mSigmaLin += inc[2];
    }

    // Maps the linearisation point by the re-centring of §2.7, keeping mDelta.
    void TransformLin(const SO3& Q, Scalar sigmaShift) {
        mDirLin = Q * mDirLin;
        mBasisLin = Q.matrix() * mBasisLin;
        mSigmaLin -= sigmaShift;
    }

    // Moves the linearisation point to the given anchor, keeping the current
    // value, §2.6.8.
    void Reanchor(const Vec3& dirLin, const Mat32& basisLin, Scalar sigmaLin) {
        const Vec3 dir = GetDir();
        const Scalar sigma = GetSigma();
        const Vec3 axis = dirLin.cross(dir);
        const Scalar sinAngle = axis.norm();
        Vec3 w = Vec3::Zero();
        if (sinAngle > Sophus::Constants<Scalar>::epsilon()) {
            w = axis * (std::atan2(sinAngle, dirLin.dot(dir)) / sinAngle);
        }
        mDirLin = dirLin;
        mBasisLin = basisLin;
        mSigmaLin = sigmaLin;
        mDelta.template head<2>() = basisLin.transpose() * w;
        mDelta[2] = sigma - sigmaLin;
        mLinearized = true;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Cumulative gravity and scale corrections applied by the VIO estimator, one
// entry per epoch, shared with the local mapper, §2.9.
struct GravityScaleCorrectionLog {
    using Ptr = std::shared_ptr<GravityScaleCorrectionLog>;

    struct Entry {
        Sophus::SO3d mQ;
        double mScale = 1.0;

        Sophus::SE3d Apply(const Sophus::SE3d& T) const {
            return Sophus::SE3d(mQ * T.so3(), mScale * (mQ * T.translation()));
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    void Append(const Sophus::SO3d& Q, double scale) {
        std::lock_guard<std::mutex> lock(mMtxLog);
        Entry entry;
        entry.mQ = Q * mvEntries.back().mQ;
        entry.mScale = scale * mvEntries.back().mScale;
        mvEntries.push_back(entry);
    }

    int64_t LatestEpoch() const {
        std::lock_guard<std::mutex> lock(mMtxLog);
        return int64_t(mvEntries.size()) - 1;
    }

    // Similarity carrying a quantity expressed at epoch `from` to epoch `to`.
    Entry Between(int64_t from, int64_t to) const {
        std::lock_guard<std::mutex> lock(mMtxLog);
        const Entry& a = mvEntries.at(size_t(from));
        const Entry& b = mvEntries.at(size_t(to));
        Entry entry;
        entry.mQ = b.mQ * a.mQ.inverse();
        entry.mScale = b.mScale / a.mScale;
        return entry;
    }

    mutable std::mutex mMtxLog;
    Eigen::aligned_vector<Entry> mvEntries = Eigen::aligned_vector<Entry>(1);
};
```

In `struct MargData` (`:327-340`), replace

```cpp
    bool use_imu;
```

with

```cpp
    bool use_imu;
    int64_t mGsEpoch = 0;
```

and in `struct Keyframe` (`:342-350`), replace

```cpp
    OpticalFlowResult::Ptr opt_flow_res;
```

with

```cpp
    OpticalFlowResult::Ptr opt_flow_res;
    int64_t mGsEpoch = 0;
```

While unfrozen, `mDelta` stays zero and `ApplyInc` moves the linearisation point itself, which is the unfrozen behaviour of `PoseStateWithLin`. `Eigen::aligned_vector` is the alias of `thirdparty/basalt-headers/include/basalt/utils/eigen_utils.hpp:51`. Backward compatibility. Additive declarations and two defaulted fields. `MargData` and `Keyframe` are filled field by field at their only construction sites (`sqrt_keypoint_vio.cpp:1252-1277`, `:2045-2053`, and the VO estimator's equivalents), so the defaulted `mGsEpoch = 0` leaves every existing producer correct, and epoch 0 is the identity of the log.

#### Change 2. `computeRelPoseGS`, `include/basalt/utils/ba_utils.h`

Insert after the closing brace of `computeRelPose` (`:80`).

```cpp
template <class Scalar>
Sophus::SE3<Scalar> computeRelPoseGS(
    const Sophus::SE3<Scalar>& T_w_i_h, const Sophus::SE3<Scalar>& T_i_c_h,
    const Sophus::SE3<Scalar>& T_w_i_t, const Sophus::SE3<Scalar>& T_i_c_t,
    Scalar scale, Sophus::Matrix6<Scalar>* d_rel_d_h = nullptr,
    Sophus::Matrix6<Scalar>* d_rel_d_t = nullptr,
    Sophus::Vector6<Scalar>* d_rel_d_logscale = nullptr) {
  Sophus::SE3<Scalar> tmp2 = (T_i_c_t).inverse();

  const Eigen::Matrix<Scalar, 3, 1> tau =
      T_w_i_t.so3().inverse() *
      (T_w_i_h.translation() - T_w_i_t.translation());

  Sophus::SE3<Scalar> T_t_i_h_i;
  T_t_i_h_i.so3() = T_w_i_t.so3().inverse() * T_w_i_h.so3();
  T_t_i_h_i.translation() = scale * tau;

  Sophus::SE3<Scalar> tmp = tmp2 * T_t_i_h_i;
  Sophus::SE3<Scalar> res = tmp * T_i_c_h;

  Sophus::Matrix6<Scalar> Ds = Sophus::Matrix6<Scalar>::Identity();
  Ds.template topLeftCorner<3, 3>() *= scale;

  if (d_rel_d_h) {
    Sophus::Matrix3<Scalar> R = T_w_i_h.so3().inverse().matrix();

    Sophus::Matrix6<Scalar> RR;
    RR.setZero();
    RR.template topLeftCorner<3, 3>() = R;
    RR.template bottomRightCorner<3, 3>() = R;

    *d_rel_d_h = tmp.Adj() * Ds * RR;
  }

  if (d_rel_d_t) {
    Sophus::Matrix3<Scalar> R = T_w_i_t.so3().inverse().matrix();

    Sophus::Matrix6<Scalar> RR;
    RR.setZero();
    RR.template topLeftCorner<3, 3>() = R;
    RR.template bottomRightCorner<3, 3>() = R;

    *d_rel_d_t = -tmp2.Adj() * Ds * RR;
  }

  if (d_rel_d_logscale) {
    d_rel_d_logscale->setZero();
    d_rel_d_logscale->template head<3>() = tmp2.so3() * (scale * tau);
  }

  return res;
}
```

`linearizePoint` is not duplicated. The scale enters the visual residual only through `T_{t,h}(s)` (§2.2.1), so `linearizePoint` already returns everything needed, namely `d_res_d_xi`, and the landmark block composes `d_res_d_xi · d_rel_d_logscale` exactly as it composes `d_res_d_xi · d_rel_d_h` (`landmark_block_abs_dynamic.hpp:182-185`). A `linearizePointGS` would be a copy of `linearizePoint`, so it is omitted. This is a deliberate departure from the request, recorded here so that it is not re-proposed. Backward compatibility. A new function, and `computeRelPose` is untouched.

#### Change 3. `residualGS`, `thirdparty/basalt-headers/include/basalt/imu/preintegration.h`

Insert after the closing brace of `residual` (`:293`), before `get_dt_ns` (`:296`). `SO3`, `Mat3`, `Vec3`, `VecN`, `MatNN` and `MatN3` are the class's existing aliases (`:62` and neighbours).

```cpp
  /// @brief Residual of plans/gravity_scale_optimisation.md §2.3.1 with
  /// gravity vector g_D in the estimator world and a scale, and optionally
  /// its Jacobians, including those with respect to g_D and log(scale).
  VecN residualGS(const PoseVelState<Scalar>& state0, const Vec3& g_D,
                  const PoseVelState<Scalar>& state1, const Vec3& curr_bg,
                  const Vec3& curr_ba, Scalar scale,
                  MatNN* d_res_d_state0 = nullptr,
                  MatNN* d_res_d_state1 = nullptr, MatN3* d_res_d_bg = nullptr,
                  MatN3* d_res_d_ba = nullptr, MatN3* d_res_d_gD = nullptr,
                  VecN* d_res_d_logscale = nullptr) const {
    Scalar dt = delta_state_.t_ns * Scalar(1e-9);
    VecN res;

    VecN bg_diff;
    VecN ba_diff;
    bg_diff = d_state_d_bg_ * (curr_bg - bias_gyro_lin_);
    ba_diff = d_state_d_ba_ * (curr_ba - bias_accel_lin_);

    BASALT_ASSERT(ba_diff.template segment<3>(3).isApproxToConstant(0));

    const Vec3 dpos = state1.T_w_i.translation() -
                      state0.T_w_i.translation() - state0.vel_w_i * dt;
    const Vec3 dvel = state1.vel_w_i - state0.vel_w_i;

    Mat3 R0_inv = state0.T_w_i.so3().inverse().matrix();
    Vec3 tmp = R0_inv * (scale * dpos - Scalar(0.5) * g_D * dt * dt);

    res.template segment<3>(0) =
        tmp - (delta_state_.T_w_i.translation() +
               bg_diff.template segment<3>(0) + ba_diff.template segment<3>(0));
    res.template segment<3>(3) =
        (SO3::exp(bg_diff.template segment<3>(3)) * delta_state_.T_w_i.so3() *
         state1.T_w_i.so3().inverse() * state0.T_w_i.so3())
            .log();

    Vec3 tmp2 = R0_inv * (scale * dvel - g_D * dt);
    res.template segment<3>(6) =
        tmp2 - (delta_state_.vel_w_i + bg_diff.template segment<3>(6) +
                ba_diff.template segment<3>(6));

    if (d_res_d_state0 || d_res_d_state1) {
      Mat3 J;
      Sophus::rightJacobianInvSO3(res.template segment<3>(3), J);

      if (d_res_d_state0) {
        d_res_d_state0->setZero();
        d_res_d_state0->template block<3, 3>(0, 0) = -scale * R0_inv;
        d_res_d_state0->template block<3, 3>(0, 3) = SO3::hat(tmp) * R0_inv;
        d_res_d_state0->template block<3, 3>(3, 3) = J * R0_inv;
        d_res_d_state0->template block<3, 3>(6, 3) = SO3::hat(tmp2) * R0_inv;

        d_res_d_state0->template block<3, 3>(0, 6) = -scale * R0_inv * dt;
        d_res_d_state0->template block<3, 3>(6, 6) = -scale * R0_inv;
      }

      if (d_res_d_state1) {
        d_res_d_state1->setZero();
        d_res_d_state1->template block<3, 3>(0, 0) = scale * R0_inv;
        d_res_d_state1->template block<3, 3>(3, 3) = -J * R0_inv;

        d_res_d_state1->template block<3, 3>(6, 6) = scale * R0_inv;
      }
    }

    if (d_res_d_ba) {
      *d_res_d_ba = -d_state_d_ba_;
    }

    if (d_res_d_bg) {
      d_res_d_bg->setZero();
      *d_res_d_bg = -d_state_d_bg_;

      Mat3 J;
      Sophus::leftJacobianInvSO3(res.template segment<3>(3), J);
      d_res_d_bg->template block<3, 3>(3, 0) =
          J * d_state_d_bg_.template block<3, 3>(3, 0);
    }

    if (d_res_d_gD) {
      d_res_d_gD->setZero();
      d_res_d_gD->template block<3, 3>(0, 0) = Scalar(-0.5) * dt * dt * R0_inv;
      d_res_d_gD->template block<3, 3>(6, 0) = -dt * R0_inv;
    }

    if (d_res_d_logscale) {
      d_res_d_logscale->setZero();
      d_res_d_logscale->template segment<3>(0) = scale * R0_inv * dpos;
      d_res_d_logscale->template segment<3>(6) = scale * R0_inv * dvel;
    }

    return res;
  }
```

At $\mathbf{g}_D = \mathbf{g}$ and `scale = 1`, every expression above reduces term by term to `residual` (`:221-293`). Backward compatibility. One new method, `residual` and `predictState` untouched. The vendored header is modified, which must be recorded in the `Note` section of the commit.

#### Change 4. `RelPoseLin` and the landmark block scale column

In `include/basalt/linearization/landmark_block.hpp`, replace

```cpp
  Mat4 T_t_h;
  Mat6 d_rel_d_h;
  Mat6 d_rel_d_t;
```

with

```cpp
  Mat4 T_t_h;
  Mat6 d_rel_d_h;
  Mat6 d_rel_d_t;
  Eigen::Matrix<Scalar, 6, 1> d_rel_d_logscale =
      Eigen::Matrix<Scalar, 6, 1>::Zero();
```

In `include/basalt/linearization/landmark_block_abs_dynamic.hpp`, `allocateLandmark`, replace

```cpp
    aom_ = &aom;
```

with

```cpp
    aom_ = &aom;

    const auto it_gs = aom.abs_order_map.find(kGravityScaleStateId);
    mpGsScaleCol = it_gs == aom.abs_order_map.end()
                       ? -1
                       : it_gs->second.first + GRAVITY_SCALE_SIZE - 1;
```

In `linearizeLandmark`, replace

```cpp
                storage.template block<2, 6>(obs_idx, abs_t_idx) +=
                    d_res_d_xi * pose_lin_vec[i]->d_rel_d_t;
```

with

```cpp
                storage.template block<2, 6>(obs_idx, abs_t_idx) +=
                    d_res_d_xi * pose_lin_vec[i]->d_rel_d_t;
                if (mpGsScaleCol >= 0) {
                  storage.template block<2, 1>(obs_idx, mpGsScaleCol) +=
                      d_res_d_xi * pose_lin_vec[i]->d_rel_d_logscale;
                }
```

In the private members, replace

```cpp
  const AbsOrderMap* aom_ = nullptr;
```

with

```cpp
  const AbsOrderMap* aom_ = nullptr;
  int mpGsScaleCol = -1;
```

`d_res_d_xi` has already been multiplied by `sqrt_weight` at `:181`, so the new column is whitened like the pose columns. Backward compatibility. Outside the period `aom` holds no gravity and scale key, `mpGsScaleCol` is −1 and the branch is never taken. The extra 48 bytes per `RelPoseLin` are the only cost. The other landmark block types are untouched, and the mode asserts the `ABS_QR` path.

#### Change 5. `ImuBlock` virtual interface and `ImuBlockGS`, `include/basalt/linearization/imu_block.hpp`

In `class ImuBlock`, insert after the constructor (`:24`)

```cpp
  virtual ~ImuBlock() = default;
```

and prefix the six member functions `linearizeImu` (`:26`), `add_dense_Q2Jp_Q2r` (`:110`), `add_dense_H_b` (`:128`), `scaleJp_cols` (`:163`), `addJp_diag2` (`:179`) and `backSubstitute` (`:197`) with `virtual`, leaving their bodies unchanged.

In `linearizeImu`, replace the bias random walk section, from

```cpp
    // difference between biases
    Scalar dt = imu_meas->get_dt_ns() * Scalar(1e-9);
```

down to and including

```cpp
    return imu_error + bg_error + ba_error;
  }
```

with

```cpp
    const std::pair<Scalar, Scalar> bias_error =
        LinearizeBiasRandomWalk(start_state, end_state);

    return imu_error + bias_error.first + bias_error.second;
  }
```

and add to the class, immediately after `protected:` (`:225`), the extracted helper, whose body is the replaced text verbatim apart from its return statement.

```cpp
  std::pair<Scalar, Scalar> LinearizeBiasRandomWalk(
      const PoseVelBiasStateWithLin<Scalar>& start_state,
      const PoseVelBiasStateWithLin<Scalar>& end_state) {
    const size_t start_idx = 0;
    const size_t end_idx = POSE_VEL_BIAS_SIZE;

    // difference between biases
    Scalar dt = imu_meas->get_dt_ns() * Scalar(1e-9);

    Vec3 gyro_bias_weight_dt =
        imu_lin_data->gyro_bias_weight_sqrt / std::sqrt(dt);
    Vec3 res_bg =
        start_state.getState().bias_gyro - end_state.getState().bias_gyro;

    Jp.template block<3, 3>(9, start_idx + 9) =
        gyro_bias_weight_dt.asDiagonal();
    Jp.template block<3, 3>(9, end_idx + 9) =
        (-gyro_bias_weight_dt).asDiagonal();

    r.template segment<3>(9) += gyro_bias_weight_dt.asDiagonal() * res_bg;

    Scalar bg_error =
        Scalar(0.5) * (gyro_bias_weight_dt.asDiagonal() * res_bg).squaredNorm();

    Vec3 accel_bias_weight_dt =
        imu_lin_data->accel_bias_weight_sqrt / std::sqrt(dt);
    Vec3 res_ba =
        start_state.getState().bias_accel - end_state.getState().bias_accel;

    Jp.template block<3, 3>(12, start_idx + 12) =
        accel_bias_weight_dt.asDiagonal();
    Jp.template block<3, 3>(12, end_idx + 12) =
        (-accel_bias_weight_dt).asDiagonal();

    r.template segment<3>(12) += accel_bias_weight_dt.asDiagonal() * res_ba;

    Scalar ba_error =
        Scalar(0.5) *
        (accel_bias_weight_dt.asDiagonal() * res_ba).squaredNorm();

    return {bg_error, ba_error};
  }
```

Returning the two errors separately keeps the sum `(imu_error + bg_error) + ba_error` in today's order, so the cost is bitwise unchanged. Then append after the closing `};` of `class ImuBlock` (`:233`), still inside `namespace basalt`.

```cpp
template <class Scalar_>
class ImuBlockGS : public ImuBlock<Scalar_> {
 public:
  using Scalar = Scalar_;
  using Base = ImuBlock<Scalar>;
  using VecX = typename Base::VecX;
  using MatX = typename Base::MatX;
  using Vec3 = typename Base::Vec3;
  using MatGs = Eigen::Matrix<Scalar, POSE_VEL_BIAS_SIZE, GRAVITY_SCALE_SIZE>;

  ImuBlockGS(const IntegratedImuMeasurement<Scalar>* meas,
             const ImuLinData<Scalar>* imu_lin_data, const AbsOrderMap& aom,
             const GravityScaleStateWithLin<Scalar>* gs)
      : Base(meas, imu_lin_data, aom),
        mpGs(gs),
        mpGsIdx(aom.abs_order_map.at(kGravityScaleStateId).first) {
    mpJgs.setZero();
  }

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

    const std::pair<Scalar, Scalar> bias_error =
        this->LinearizeBiasRandomWalk(start_state, end_state);

    return imu_error + bias_error.first + bias_error.second;
  }

  void add_dense_Q2Jp_Q2r(MatX& Q2Jp, VecX& Q2r,
                          size_t row_start_idx) const override {
    Base::add_dense_Q2Jp_Q2r(Q2Jp, Q2r, row_start_idx);

    Q2Jp.template block<POSE_VEL_BIAS_SIZE, GRAVITY_SCALE_SIZE>(row_start_idx,
                                                                mpGsIdx) +=
        mpJgs;
  }

  void add_dense_H_b(DenseAccumulator<Scalar>& accum) const override {
    Base::add_dense_H_b(accum);

    const int64_t start_t = this->imu_meas->get_start_t_ns();
    const int64_t end_t =
        this->imu_meas->get_start_t_ns() + this->imu_meas->get_dt_ns();

    const size_t start_idx = this->aom.abs_order_map.at(start_t).first;
    const size_t end_idx = this->aom.abs_order_map.at(end_t).first;

    const Eigen::Matrix<Scalar, GRAVITY_SCALE_SIZE, GRAVITY_SCALE_SIZE> H_gg =
        mpJgs.transpose() * mpJgs;
    const Eigen::Matrix<Scalar, GRAVITY_SCALE_SIZE, POSE_VEL_BIAS_SIZE> H_gs =
        mpJgs.transpose() *
        this->Jp.template leftCols<POSE_VEL_BIAS_SIZE>();
    const Eigen::Matrix<Scalar, GRAVITY_SCALE_SIZE, POSE_VEL_BIAS_SIZE> H_ge =
        mpJgs.transpose() *
        this->Jp.template rightCols<POSE_VEL_BIAS_SIZE>();
    const Eigen::Matrix<Scalar, GRAVITY_SCALE_SIZE, 1> b_g =
        mpJgs.transpose() * this->r;

    accum.template addH<GRAVITY_SCALE_SIZE, GRAVITY_SCALE_SIZE>(
        mpGsIdx, mpGsIdx, H_gg);
    accum.template addH<GRAVITY_SCALE_SIZE, POSE_VEL_BIAS_SIZE>(
        mpGsIdx, start_idx, H_gs);
    accum.template addH<POSE_VEL_BIAS_SIZE, GRAVITY_SCALE_SIZE>(
        start_idx, mpGsIdx, H_gs.transpose());
    accum.template addH<GRAVITY_SCALE_SIZE, POSE_VEL_BIAS_SIZE>(
        mpGsIdx, end_idx, H_ge);
    accum.template addH<POSE_VEL_BIAS_SIZE, GRAVITY_SCALE_SIZE>(
        end_idx, mpGsIdx, H_ge.transpose());
    accum.template addB<GRAVITY_SCALE_SIZE>(mpGsIdx, b_g);
  }

  void scaleJp_cols(const VecX& jacobian_scaling) override {
    Base::scaleJp_cols(jacobian_scaling);

    mpJgs *= jacobian_scaling.template segment<GRAVITY_SCALE_SIZE>(mpGsIdx)
                 .asDiagonal();
  }

  void addJp_diag2(VecX& res) const override {
    Base::addJp_diag2(res);

    res.template segment<GRAVITY_SCALE_SIZE>(mpGsIdx) +=
        mpJgs.colwise().squaredNorm().transpose();
  }

  void backSubstitute(const VecX& pose_inc, Scalar& l_diff) override {
    const int64_t start_t = this->imu_meas->get_start_t_ns();
    const int64_t end_t =
        this->imu_meas->get_start_t_ns() + this->imu_meas->get_dt_ns();

    const size_t start_idx = this->aom.abs_order_map.at(start_t).first;
    const size_t end_idx = this->aom.abs_order_map.at(end_t).first;

    VecX pose_inc_reduced(2 * POSE_VEL_BIAS_SIZE);
    pose_inc_reduced.template head<POSE_VEL_BIAS_SIZE>() =
        pose_inc.template segment<POSE_VEL_BIAS_SIZE>(start_idx);
    pose_inc_reduced.template tail<POSE_VEL_BIAS_SIZE>() =
        pose_inc.template segment<POSE_VEL_BIAS_SIZE>(end_idx);

    VecX Jinc = this->Jp * pose_inc_reduced +
                mpJgs * pose_inc.template segment<GRAVITY_SCALE_SIZE>(mpGsIdx);
    l_diff -= Jinc.transpose() * (Scalar(0.5) * Jinc + this->r);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const GravityScaleStateWithLin<Scalar>* mpGs;
  size_t mpGsIdx;
  MatGs mpJgs;
};
```

`Jp` keeps its 15×30 shape and every fixed offset in the base remains valid, so the base methods are reused unchanged and the derived class only adds the three gravity and scale columns. Backward compatibility. Virtual dispatch adds one indirection per call on a handful of IMU blocks per iteration. `LinearizationAbsSC` and `LinearizationRelSC` hold `ImuBlock` by `unique_ptr` as well (`linearization_abs_sc.cpp:79`, `linearization_rel_sc.cpp:79`) and see the same arithmetic. The bias rows are moved into a helper without changing a single operation or the order of the final sum.

#### Change 6. `LinearizationAbsQR::linearizeRelPoses`, `include/basalt/linearization/linearization_abs_qr.hpp` and `src/linearization/linearization_abs_qr.cpp`

In the header, insert in the `protected:  //  helper` section, before `get_dense_Q2Jp_Q2r_pose_damping`,

```cpp
  virtual void linearizeRelPoses();
```

In the source, in `linearizeProblem`, replace the whole block from

```cpp
  // Linearize relative poses
  for (const auto& [tcid_h, target_map] : lmdb_.getObservations()) {
```

down to and including the closing brace of that outer `for` (`:193-228`) with

```cpp
  linearizeRelPoses();
```

and insert the new definition after `linearizeProblem`, whose body is the removed loop verbatim.

```cpp
template <typename Scalar, int POSE_SIZE>
void LinearizationAbsQR<Scalar, POSE_SIZE>::linearizeRelPoses() {
  // Linearize relative poses
  for (const auto& [tcid_h, target_map] : lmdb_.getObservations()) {
    // if (used_frames && used_frames->count(tcid_h.frame_id) == 0) continue;

    for (const auto& [tcid_t, _] : target_map) {
      std::pair<TimeCamId, TimeCamId> key(tcid_h, tcid_t);
      RelPoseLin<Scalar>& rpl = relative_pose_lin.at(key);

      if (tcid_h != tcid_t) {
        const PoseStateWithLin<Scalar>& state_h =
            estimator->getPoseStateWithLin(tcid_h.frame_id);
        const PoseStateWithLin<Scalar>& state_t =
            estimator->getPoseStateWithLin(tcid_t.frame_id);

        // compute relative pose & Jacobians at linearization point
        Sophus::SE3<Scalar> T_t_h_sophus =
            computeRelPose(state_h.getPoseLin(), calib.T_i_c[tcid_h.cam_id],
                           state_t.getPoseLin(), calib.T_i_c[tcid_t.cam_id],
                           &rpl.d_rel_d_h, &rpl.d_rel_d_t);

        // if either state is already linearized, then the current state
        // estimate is different from the linearization point, so recompute
        // the value (not Jacobian) again based on the current state.
        if (state_h.isLinearized() || state_t.isLinearized()) {
          T_t_h_sophus =
              computeRelPose(state_h.getPose(), calib.T_i_c[tcid_h.cam_id],
                             state_t.getPose(), calib.T_i_c[tcid_t.cam_id]);
        }

        rpl.T_t_h = T_t_h_sophus.matrix();
      } else {
        rpl.T_t_h.setIdentity();
        rpl.d_rel_d_h.setZero();
        rpl.d_rel_d_t.setZero();
      }
    }
  }
}
```

Backward compatibility. A pure extraction with the same statements in the same order.

#### Change 7. `LinearizationGSAbsQR`, `include/basalt/linearization/linearization_abs_qr.hpp` and `src/linearization/linearization_abs_qr.cpp`

In the header, insert after the closing `};` of `class LinearizationAbsQR`, still inside `namespace basalt`.

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
      int64_t last_state_to_marg = std::numeric_limits<int64_t>::max());

 protected:
  void linearizeRelPoses() override;

 private:
  const GravityScaleStateWithLin<Scalar>* mpGs;
  bool mpVisualScaleFree;
};
```

In the source, insert after the definition of `linearizeRelPoses` of Change 6.

```cpp
template <typename Scalar, int POSE_SIZE>
LinearizationGSAbsQR<Scalar, POSE_SIZE>::LinearizationGSAbsQR(
    BundleAdjustmentBase<Scalar>* estimator, const AbsOrderMap& aom,
    const Options& options, const GravityScaleStateWithLin<Scalar>* gs,
    bool visualScaleFree, const MargLinData<Scalar>* marg_lin_data,
    const ImuLinData<Scalar>* imu_lin_data,
    const std::set<FrameId>* used_frames,
    const std::unordered_set<KeypointId>* lost_landmarks,
    int64_t last_state_to_marg)
    : Base(estimator, aom, options, marg_lin_data, imu_lin_data, used_frames,
           lost_landmarks, last_state_to_marg),
      mpGs(gs),
      mpVisualScaleFree(visualScaleFree) {
  BASALT_ASSERT(gs != nullptr);
  BASALT_ASSERT(aom.abs_order_map.at(kGravityScaleStateId).first == 0);

  this->imu_blocks.clear();
  if (imu_lin_data) {
    for (const auto& kv : imu_lin_data->imu_meas) {
      this->imu_blocks.emplace_back(
          new ImuBlockGS<Scalar>(kv.second, imu_lin_data, aom, gs));
    }
  }
}

template <typename Scalar, int POSE_SIZE>
void LinearizationGSAbsQR<Scalar, POSE_SIZE>::linearizeRelPoses() {
  const Scalar scale_lin = mpGs->GetScaleLin();
  const Scalar scale = mpGs->GetScale();

  for (const auto& [tcid_h, target_map] : this->lmdb_.getObservations()) {
    for (const auto& [tcid_t, _] : target_map) {
      std::pair<TimeCamId, TimeCamId> key(tcid_h, tcid_t);
      RelPoseLin<Scalar>& rpl = this->relative_pose_lin.at(key);

      if (tcid_h != tcid_t) {
        const PoseStateWithLin<Scalar>& state_h =
            this->estimator->getPoseStateWithLin(tcid_h.frame_id);
        const PoseStateWithLin<Scalar>& state_t =
            this->estimator->getPoseStateWithLin(tcid_t.frame_id);

        Sophus::SE3<Scalar> T_t_h_sophus = computeRelPoseGS(
            state_h.getPoseLin(), this->calib.T_i_c[tcid_h.cam_id],
            state_t.getPoseLin(), this->calib.T_i_c[tcid_t.cam_id], scale_lin,
            &rpl.d_rel_d_h, &rpl.d_rel_d_t, &rpl.d_rel_d_logscale);

        if (state_h.isLinearized() || state_t.isLinearized() ||
            mpGs->mLinearized) {
          T_t_h_sophus = computeRelPoseGS(
              state_h.getPose(), this->calib.T_i_c[tcid_h.cam_id],
              state_t.getPose(), this->calib.T_i_c[tcid_t.cam_id], scale);
        }

        if (mpVisualScaleFree) rpl.d_rel_d_logscale.setZero();

        rpl.T_t_h = T_t_h_sophus.matrix();
      } else {
        rpl.T_t_h.setIdentity();
        rpl.d_rel_d_h.setZero();
        rpl.d_rel_d_t.setZero();
        rpl.d_rel_d_logscale.setZero();
      }
    }
  }
}
```

In the instantiation block at the end of the source, replace

```cpp
template class LinearizationAbsQR<double, 6>;
```

with

```cpp
template class LinearizationAbsQR<double, 6>;
template class LinearizationGSAbsQR<double, 6>;
```

and

```cpp
template class LinearizationAbsQR<float, 6>;
```

with

```cpp
template class LinearizationAbsQR<float, 6>;
template class LinearizationGSAbsQR<float, 6>;
```

The class is constructed directly in `optimize` and `marginalize`, so `LinearizationBase::create` and its enum stay untouched. Backward compatibility. New class, used only during the period.

#### Change 8. `BundleAdjustmentBase`, `include/basalt/vi_estimator/ba_base.h` and `src/vi_estimator/ba_base.cpp`

In the header, add `#include <optional>` after `#include <basalt/vi_estimator/landmark_database.h>` (`:37`). Replace the declaration of `computeError` (`:57-60`)

```cpp
    void computeError(Scalar& error,
                      std::map<int, std::vector<std::pair<TimeCamId, Scalar>>>*
                          outliers = nullptr,
                      Scalar outlier_threshold = 0) const;
```

with

```cpp
    void computeError(Scalar& error,
                      std::map<int, std::vector<std::pair<TimeCamId, Scalar>>>*
                          outliers = nullptr,
                      Scalar outlier_threshold = 0,
                      Scalar relScale = Scalar(1)) const;
```

Replace `backup` and `restore` (`:130-140`) with

```cpp
    inline void backup() {
        for (auto& kv : this->frame_states) kv.second.backup();
        for (auto& kv : this->frame_poses) kv.second.backup();
        this->lmdb.backup();
        mGsStateBackup = mGsState;
    }

    inline void restore() {
        for (auto& kv : this->frame_states) kv.second.restore();
        for (auto& kv : this->frame_poses) kv.second.restore();
        this->lmdb.restore();
        mGsState = mGsStateBackup;
    }

    std::optional<GravityScaleStateWithLin<Scalar>> mGsState;
    std::optional<GravityScaleStateWithLin<Scalar>> mGsStateBackup;
```

In the source, replace the definition header of `computeError` (`:141-144`)

```cpp
void BundleAdjustmentBase<Scalar_>::computeError(
    Scalar& error,
    std::map<int, std::vector<std::pair<TimeCamId, Scalar>>>* outliers,
    Scalar outlier_threshold) const {
```

with

```cpp
void BundleAdjustmentBase<Scalar_>::computeError(
    Scalar& error,
    std::map<int, std::vector<std::pair<TimeCamId, Scalar>>>* outliers,
    Scalar outlier_threshold, Scalar relScale) const {
```

and inside it replace

```cpp
          Sophus::SE3<Scalar> T_t_h_sophus =
              computeRelPose(state_h.getPose(), calib.T_i_c[tcid_h.cam_id],
                             state_t.getPose(), calib.T_i_c[tcid_t.cam_id]);

          T_t_h = T_t_h_sophus.matrix();
        } else {
          T_t_h.setIdentity();
        }

        std::visit(
            [&](const auto& cam) {
              for (KeypointId kpt_id : obs_kv.second) {
                const Keypoint<Scalar>& kpt_pos = lmdb.getLandmark(kpt_id);
                const Vec2& kpt_obs = kpt_pos.obs.at(tcid_t);

                Vec2 res;

                bool valid = linearizePoint(kpt_obs, kpt_pos, T_t_h, cam, res);
```

with

```cpp
          Sophus::SE3<Scalar> T_t_h_sophus =
              relScale == Scalar(1)
                  ? computeRelPose(state_h.getPose(),
                                   calib.T_i_c[tcid_h.cam_id],
                                   state_t.getPose(),
                                   calib.T_i_c[tcid_t.cam_id])
                  : computeRelPoseGS(state_h.getPose(),
                                     calib.T_i_c[tcid_h.cam_id],
                                     state_t.getPose(),
                                     calib.T_i_c[tcid_t.cam_id], relScale);

          T_t_h = T_t_h_sophus.matrix();
        } else {
          T_t_h.setIdentity();
        }

        std::visit(
            [&](const auto& cam) {
              for (KeypointId kpt_id : obs_kv.second) {
                const Keypoint<Scalar>& kpt_pos = lmdb.getLandmark(kpt_id);
                const Vec2& kpt_obs = kpt_pos.obs.at(tcid_t);

                Vec2 res;

                bool valid = linearizePoint(kpt_obs, kpt_pos, T_t_h, cam, res);
```

In `computeDelta` (`:315-332`), replace

```cpp
    } else {
      BASALT_ASSERT(false);
    }
```

with

```cpp
    } else if (kv.first == kGravityScaleStateId) {
      BASALT_ASSERT(kv.second.second == GRAVITY_SCALE_SIZE);
      BASALT_ASSERT(mGsState.has_value());
      delta.template segment<GRAVITY_SCALE_SIZE>(kv.second.first) =
          mGsState->GetDelta();
    } else {
      BASALT_ASSERT(false);
    }
```

The size tests `== POSE_SIZE` and `== POSE_VEL_BIAS_SIZE` both fail for the block's size 3, so the new branch is reached before the final assertion. `computeProjections` is not touched, since the published payload is assembled after the re-centring, when the scale is 1. Backward compatibility. The callers of `computeError` are `SqrtKeypointVioEstimator`, `SqrtKeypointVoEstimator`, `NfrMapper` and the local mapper. None passes `relScale`, so each keeps the `computeRelPose` path, and none places `kGravityScaleStateId` in an order, so the new `computeDelta` branch is unreachable for them. `mGsState` and `mGsStateBackup` stay empty for every one of them, so the two added assignments in `backup` and `restore` copy an empty optional.

#### Change 9. `ScBundleAdjustmentBase::computeImuError`, `include/basalt/vi_estimator/sc_ba_base.h` and `src/vi_estimator/sc_ba_base.cpp`

In the header, replace the last line of the declaration (`:252`)

```cpp
      const Vec3& g);
```

with

```cpp
      const Vec3& g, Scalar scale = Scalar(1));
```

In the source, replace the last line of the definition header (`:744`)

```cpp
    const Vec3& g) {
```

with

```cpp
    const Vec3& g, Scalar scale) {
```

and replace

```cpp
      const typename PoseVelState<Scalar>::VecN res = kv.second.residual(
          start_state.getState(), g, end_state.getState(),
          start_state.getState().bias_gyro, start_state.getState().bias_accel);
```

with

```cpp
      const typename PoseVelState<Scalar>::VecN res =
          scale == Scalar(1)
              ? kv.second.residual(start_state.getState(), g,
                                   end_state.getState(),
                                   start_state.getState().bias_gyro,
                                   start_state.getState().bias_accel)
              : kv.second.residualGS(start_state.getState(), g,
                                     end_state.getState(),
                                     start_state.getState().bias_gyro,
                                     start_state.getState().bias_accel, scale);
```

The caller in the period passes the current $\mathbf{g}_D$ as `g`, and `residual` evaluated with $\mathbf{g}_D$ equals `residualGS` at scale 1, so the branch is exact in both arms. Backward compatibility. Existing callers omit `scale` and take the `residual` branch with the gravity they already pass.

#### Change 10. Configuration

In `include/basalt/utils/vio_config.h`, insert after `int vio_stationary_latch_frames;` (`:126`)

```cpp
    // Optimise gravity direction and metric scale during the first
    // vio_gs_duration_s seconds, then fold them into the state. Requires
    // vio_linearization_type ABS_QR and vio_sqrt_marg.
    bool vio_gs_enable;
    double vio_gs_duration_s;
    // VI-DSO d_min of the dynamic marginalisation scale interval.
    double vio_gs_dmin;
```

In `src/utils/vio_config.cpp`, insert after `vio_stationary_latch_frames = 3;` (`:100`)

```cpp
  vio_gs_enable = false;
  vio_gs_duration_s = 10.0;
  vio_gs_dmin = 1.0488088481701516;
```

and after `ar(CEREAL_NVP(config.vio_stationary_latch_frames));` (`:235`)

```cpp
  ar(CEREAL_NVP(config.vio_gs_enable));
  ar(CEREAL_NVP(config.vio_gs_duration_s));
  ar(CEREAL_NVP(config.vio_gs_dmin));
```

In each of `data/euroc_config.json`, `data/euroc_config_no_factors.json`, `data/euroc_config_no_weights.json`, `data/euroc_config_vo.json`, `data/kitti_config.json`, `data/sitl_config_vo.json` and `data/tumvi_512_config.json`, insert after the line `"config.vio_stationary_latch_frames": 3,`

```json
        "config.vio_gs_enable": false,
        "config.vio_gs_duration_s": 10.0,
        "config.vio_gs_dmin": 1.0488088481701516,
```

and in `data/iccv21/basalt_batch_config.toml`, insert after the line `"config.vio_stationary_latch_frames" = 3` (`:56`)

```toml
"config.vio_gs_enable" = false
"config.vio_gs_duration_s" = 10.0
"config.vio_gs_dmin" = 1.0488088481701516
```

A key read by `serialize` and absent from a file aborts the process (`context/vio_config_json.md`), so all eight sources change in the same commit. Backward compatibility. With `vio_gs_enable = false` nothing else in this plan executes.

#### Change 11. Estimator declarations, construction and the initial prior

In `include/basalt/vi_estimator/sqrt_keypoint_vio.h`, insert after `void optimize();` (`:138`)

```cpp
    void UpdateScaleInterval();
    void LinearizeMargGS(const AbsOrderMap& aom,
                         const std::set<int64_t>& kfs_to_marg,
                         const std::unordered_set<KeypointId>& lost_landmaks,
                         int64_t last_state_to_marg, MatX& Q2Jp_curr,
                         VecX& Q2r_curr, MatX& Q2Jp_half, VecX& Q2r_half,
                         MatX& Q2Jp_visual, VecX& Q2r_visual);
    void MarginalizeGS(MatX& Q2Jp_curr, VecX& Q2r_curr, MatX& Q2Jp_half,
                       VecX& Q2r_half, MatX& Q2Jp_visual, VecX& Q2r_visual,
                       const std::set<int>& idx_to_keep,
                       const std::set<int>& idx_to_marg);
    void RecentrePriorsGS();
    void RecentreGravityScale();
    void FoldGravityScale();
    void StripGsFromMargData(MargData& m) const;
    MatX ComputeGaugeProjector(const AbsOrderMap& aom) const;
    void SetInitialPriorOrder(int64_t t_ns);
    void LogGravityScale(int swapEvent);
```

Replace `QueuePoseUpdates` (`:218-225`)

```cpp
    void QueuePoseUpdates(
        const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>& updates)
        override {
        std::lock_guard<std::mutex> lock(mpPosesToUpdateMutex);
        for (const auto& kv : updates) mpPosesToUpdate[kv.first] = kv.second;
    }
```

with

```cpp
    void QueuePoseUpdates(
        const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>& updates,
        int64_t gsEpoch = 0) override {
        std::lock_guard<std::mutex> lock(mpPosesToUpdateMutex);
        for (const auto& kv : updates) {
            mpPosesToUpdate[kv.first] = kv.second;
            mpPosesToUpdateEpoch[kv.first] = gsEpoch;
        }
    }
```

Insert after `bool mpIsCurrentFrameKF = false;` (`:240`)

```cpp
    struct GsAnchor {
        Vec3 dir;
        Eigen::Matrix<Scalar, 3, 2> basis;
        Scalar sigma = 0;
        bool set = false;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    void AnchorGs(GsAnchor& anchor);

    bool mpGsActive = false;
    int64_t mpGsStartNs = -1;
    MargLinData<Scalar> mpMargHalf;
    MargLinData<Scalar> mpMargVisual;
    GsAnchor mpAnchorCurr;
    GsAnchor mpAnchorHalf;
    Scalar mpSigmaMid = 0;
    Scalar mpSigmaPrevMarg = 0;
    Scalar mpLogD = 0;
    bool mpLastUpper = false;
    int mpGsSwapEvent = 0;
    Scalar mpGsStepSigma = 0;
    Scalar mpGsStepTiltDeg = 0;
```

and replace

```cpp
    Eigen::aligned_unordered_map<int64_t, PoseStateWithLin<double>>
        mpPosesToUpdate;
    mutable std::mutex mpPosesToUpdateMutex;
```

with

```cpp
    Eigen::aligned_unordered_map<int64_t, PoseStateWithLin<double>>
        mpPosesToUpdate;
    std::unordered_map<int64_t, int64_t> mpPosesToUpdateEpoch;
    mutable std::mutex mpPosesToUpdateMutex;
```

In `src/vi_estimator/sqrt_keypoint_vio.cpp`, add `#include <basalt/linearization/linearization_abs_qr.hpp>` after `#include <basalt/linearization/linearization_base.hpp>` (`:48`). In the constructor, insert immediately before

```cpp
    std::cout << "marg_H (sqrt:" << marg_data.is_sqrt << ")\n"
```

the block

```cpp
    if (config.vio_gs_enable) {
        BASALT_ASSERT_STREAM(
            config.vio_linearization_type == LinearizationType::ABS_QR &&
                config.vio_sqrt_marg,
            "vio_gs_enable requires ABS_QR linearization and vio_sqrt_marg");

        MatX H_gs = MatX::Zero(marg_data.H.rows(),
                               GRAVITY_SCALE_SIZE + POSE_VEL_BIAS_SIZE);
        H_gs.rightCols(POSE_VEL_BIAS_SIZE) = marg_data.H;
        marg_data.H = H_gs;

        this->mGsState.emplace();
        this->mGsCorrectionLog = std::make_shared<GravityScaleCorrectionLog>();
        mpGsActive = true;
        mpLogD = std::log(Scalar(config.vio_gs_dmin));
    }
```

In the five argument `initialize`, replace

```cpp
    marg_data.order.abs_order_map[t_ns] = std::make_pair(0, POSE_VEL_BIAS_SIZE);
    marg_data.order.total_size = POSE_VEL_BIAS_SIZE;
    marg_data.order.items = 1;
```

with

```cpp
    SetInitialPriorOrder(t_ns);
```

and in the lazy bootstrap of `ProcessFrame`, replace

```cpp
        marg_data.order.abs_order_map[last_state_t_ns] =
            std::make_pair(0, POSE_VEL_BIAS_SIZE);
        marg_data.order.total_size = POSE_VEL_BIAS_SIZE;
        marg_data.order.items = 1;
```

with

```cpp
        SetInitialPriorOrder(last_state_t_ns);
```

Add the definition after `PublishKeyframe`.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::SetInitialPriorOrder(int64_t t_ns) {
    marg_data.order = AbsOrderMap();

    if (mpGsActive) {
        marg_data.order.abs_order_map[kGravityScaleStateId] =
            std::make_pair(0, GRAVITY_SCALE_SIZE);
        marg_data.order.total_size = GRAVITY_SCALE_SIZE;
        marg_data.order.items = 1;
        mpGsStartNs = t_ns;
    }

    marg_data.order.abs_order_map[t_ns] =
        std::make_pair(int(marg_data.order.total_size), POSE_VEL_BIAS_SIZE);
    marg_data.order.total_size += POSE_VEL_BIAS_SIZE;
    marg_data.order.items++;

    if (mpGsActive) {
        mpMargHalf = marg_data;
        mpMargVisual = marg_data;
    }
}
```

With the mode off the resulting order is `{t_ns: (0, 15)}`, `total_size` 15 and `items` 1, the same as the replaced lines, since the order is empty before either initialisation path runs. `QueuePoseUpdates` keeps its single argument form for every caller that passes one argument. Backward compatibility. Every new statement is guarded by `vio_gs_enable` or `mpGsActive`, except the epoch map, which is written and erased in step with `mpPosesToUpdate` under the same mutex and is read only when the correction log exists.

#### Change 12. `measure`, mapper updates, keyframe stamp and the log record

All in `src/vi_estimator/sqrt_keypoint_vio.cpp`. Prediction, triangulation, the published state and the visualisation payload are not touched, because the re-centring at the end of `optimize` keeps every state metric (§2.7.5).

Mapper updates are carried to the estimator's epoch. Replace

```cpp
                const int64_t t_ns = it->first;
                const PoseStateWithLin<double>& lm_pose = it->second;
                const SE3 T_new = lm_pose.getPose().template cast<Scalar>();
```

with

```cpp
                const int64_t t_ns = it->first;
                const PoseStateWithLin<double>& lm_pose = it->second;
                Sophus::SE3d T_new_d = lm_pose.getPose();
                const auto it_epoch = mpPosesToUpdateEpoch.find(t_ns);
                if (this->mGsCorrectionLog &&
                    it_epoch != mpPosesToUpdateEpoch.end()) {
                    T_new_d =
                        this->mGsCorrectionLog
                            ->Between(it_epoch->second,
                                      this->mGsCorrectionLog->LatestEpoch())
                            .Apply(T_new_d);
                }
                const SE3 T_new = T_new_d.template cast<Scalar>();
```

and replace

```cpp
                it = mpPosesToUpdate.erase(it);
                // ++it;
```

with

```cpp
                mpPosesToUpdateEpoch.erase(t_ns);
                it = mpPosesToUpdate.erase(it);
                // ++it;
```

Keyframe stamp. In `PublishKeyframe`, replace

```cpp
    kf->opt_flow_res = it_flow->second;

    this->mpKFOutputQueue->push(kf);
```

with

```cpp
    kf->opt_flow_res = it_flow->second;
    if (this->mGsCorrectionLog) {
        kf->mGsEpoch = this->mGsCorrectionLog->LatestEpoch();
    }

    this->mpKFOutputQueue->push(kf);
```

Add the record helper after `SetInitialPriorOrder`.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::LogGravityScale(int swapEvent) {
    const int64_t epoch = this->mGsCorrectionLog->LatestEpoch();
    const GravityScaleCorrectionLog::Entry cumulative =
        this->mGsCorrectionLog->Between(0, epoch);
    mpLogger->AddVioGravityScale(
        last_state_t_ns, epoch, double(mpGsStepSigma), double(mpGsStepTiltDeg),
        cumulative.mScale, cumulative.mQ.log().norm() * 180.0 / M_PI,
        swapEvent);
}
```

In `include/basalt/utils/logger.h`, insert after the declaration of `AddVioPoseUpdate` (`:189-191`)

```cpp
    void AddVioGravityScale(int64_t tNs, int64_t epoch, double sigmaStep,
                            double tiltStepDeg, double scaleCumulative,
                            double tiltCumulativeDeg, int swapEvent);
```

In `src/utils/logger.cpp`, insert after the `Register("vio_pose_update", ...)` call (`:368-372`)

```cpp
    Register("vio_gravity_scale", "[VIO][gravity-scale]",
             {{"t_ns", "ns"}, {"epoch", "count"}, {"sigma_step", "f"},
              {"tilt_step_deg", "f"}, {"scale_cumulative", "f"},
              {"tilt_cumulative_deg", "f"}, {"swap_event", "count"}});
```

and after the definition of `PrintVioPoseUpdate` (`:825`)

```cpp
void Logger::AddVioGravityScale(int64_t tNs, int64_t epoch, double sigmaStep,
                                double tiltStepDeg, double scaleCumulative,
                                double tiltCumulativeDeg, int swapEvent) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_gravity_scale");
    s.add_int("t_ns", tNs);
    s.add_int("epoch", epoch);
    s.add("sigma_step", sigmaStep);
    s.add("tilt_step_deg", tiltStepDeg);
    s.add("scale_cumulative", scaleCumulative);
    s.add("tilt_cumulative_deg", tiltCumulativeDeg);
    s.add("swap_event", double(swapEvent));
    s.CommitRow();
}
```

`swap_event` is 0 for none, 1 for a side change, 2 for an upper boundary crossing, 3 for a lower boundary crossing and 4 for the end of the period. Backward compatibility. The update conversion runs only when the correction log exists, which requires `vio_gs_enable`, and the logger record is new.

#### Change 13. `optimize`, order map, gauge projection, increment, cost and re-centring

All in `src/vi_estimator/sqrt_keypoint_vio.cpp`, inside `optimize`.

Order map. Replace

```cpp
        AbsOrderMap aom;

        for (const auto& kv : frame_poses) {
            aom.abs_order_map[kv.first] =
                std::make_pair(aom.total_size, POSE_SIZE);
```

with

```cpp
        AbsOrderMap aom;

        if (mpGsActive) {
            aom.abs_order_map[kGravityScaleStateId] =
                std::make_pair(0, GRAVITY_SCALE_SIZE);

            BASALT_ASSERT(
                marg_data.order.abs_order_map.at(kGravityScaleStateId) ==
                aom.abs_order_map.at(kGravityScaleStateId));

            aom.total_size += GRAVITY_SCALE_SIZE;
            aom.items++;
        }

        for (const auto& kv : frame_poses) {
            aom.abs_order_map[kv.first] =
                std::make_pair(aom.total_size, POSE_SIZE);
```

Linearisation. Replace

```cpp
            lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
                this, aom, lqr_options, &marg_data, &ild);
            allocateLmbSeconds = t.reset();
```

with

```cpp
            if (mpGsActive) {
                lqr = std::make_unique<LinearizationGSAbsQR<Scalar, POSE_SIZE>>(
                    this, aom, lqr_options, &*this->mGsState, false, &marg_data,
                    &ild);
            } else {
                lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
                    this, aom, lqr_options, &marg_data, &ild);
            }
            allocateLmbSeconds = t.reset();
```

Gauge projection of the system. Replace

```cpp
                    getDenseHBSeconds = t.reset();
```

with

```cpp
                    MatX gauge_P;
                    if (mpGsActive) {
                        gauge_P = ComputeGaugeProjector(aom);
                        H = gauge_P * H * gauge_P;
                        b = gauge_P * b;
                    }

                    getDenseHBSeconds = t.reset();
```

Gauge projection of the step. Replace

```cpp
                    if (!inc_valid) {
                        std::cerr << "Still invalid inc after " << max_num_iter
                                  << " iterations." << std::endl;
                    }
```

with

```cpp
                    if (!inc_valid) {
                        std::cerr << "Still invalid inc after " << max_num_iter
                                  << " iterations." << std::endl;
                    }

                    if (mpGsActive) inc = gauge_P * inc;
```

Both replacements lie inside the scope that declares `MatX H; VecX b;` (`:1742-1801`), so `gauge_P` is in scope for the second. Increment. Replace

```cpp
                for (auto& [frame_id, state] : frame_states) {
                    int idx = aom.abs_order_map.at(frame_id).first;
                    state.applyInc(
                        inc.template segment<POSE_VEL_BIAS_SIZE>(idx));
                }
```

with

```cpp
                for (auto& [frame_id, state] : frame_states) {
                    int idx = aom.abs_order_map.at(frame_id).first;
                    state.applyInc(
                        inc.template segment<POSE_VEL_BIAS_SIZE>(idx));
                }

                if (mpGsActive) {
                    this->mGsState->ApplyInc(
                        inc.template head<GRAVITY_SCALE_SIZE>());
                }
```

Cost after the step. Replace

```cpp
                    computeError(after_update_vision_and_inertial_error);
                    after_update_vision_error =
                        after_update_vision_and_inertial_error;
                    computeMargPriorError(marg_data,
                                          after_update_marg_prior_error);

                    ScBundleAdjustmentBase<Scalar>::computeImuError(
                        aom, after_update_imu_error, after_bg_error,
                        after_ba_error, frame_states, imu_meas,
                        gyro_bias_sqrt_weight.array().square(),
                        accel_bias_sqrt_weight.array().square(), g);
```

with

```cpp
                    const Scalar gs_scale =
                        mpGsActive ? this->mGsState->GetScale() : Scalar(1);
                    const Vec3 gs_g =
                        mpGsActive ? Vec3(g.norm() * this->mGsState->GetDir())
                                   : g;

                    computeError(after_update_vision_and_inertial_error,
                                 nullptr, Scalar(0), gs_scale);
                    after_update_vision_error =
                        after_update_vision_and_inertial_error;
                    computeMargPriorError(marg_data,
                                          after_update_marg_prior_error);

                    ScBundleAdjustmentBase<Scalar>::computeImuError(
                        aom, after_update_imu_error, after_bg_error,
                        after_ba_error, frame_states, imu_meas,
                        gyro_bias_sqrt_weight.array().square(),
                        accel_bias_sqrt_weight.array().square(), gs_g,
                        gs_scale);
```

Re-centring, the last action of the optimisation (§2.7.5). Replace

```cpp
        const double optimizeSeconds = timer_total.elapsed();
```

with

```cpp
        if (mpGsActive) RecentreGravityScale();

        const double optimizeSeconds = timer_total.elapsed();
```

Add the two definitions after `LogGravityScale`.

```cpp
template <class Scalar_>
typename SqrtKeypointVioEstimator<Scalar_>::MatX
SqrtKeypointVioEstimator<Scalar_>::ComputeGaugeProjector(
    const AbsOrderMap& aom) const {
    const int n = int(aom.total_size);
    const int gs = aom.abs_order_map.at(kGravityScaleStateId).first;
    const Eigen::Matrix<Scalar, 3, 2>& basis = this->mGsState->mBasisLin;

    MatX N = MatX::Zero(n, 3);
    N(gs + 2, 0) = Scalar(-1);
    N.template block<2, 1>(gs, 1) = basis.transpose() * Vec3::UnitX();
    N.template block<2, 1>(gs, 2) = basis.transpose() * Vec3::UnitY();

    for (const auto& [t_ns, idx] : aom.abs_order_map) {
        if (t_ns == kGravityScaleStateId) continue;

        const int c = idx.first;
        const bool has_vel = idx.second == POSE_VEL_BIAS_SIZE;

        Vec3 p;
        Vec3 v = Vec3::Zero();
        if (has_vel) {
            const PoseVelBiasState<Scalar>& st =
                frame_states.at(t_ns).getStateLin();
            p = st.T_w_i.translation();
            v = st.vel_w_i;
        } else {
            p = frame_poses.at(t_ns).getPoseLin().translation();
        }

        N.template block<3, 1>(c, 0) = p;
        if (has_vel) N.template block<3, 1>(c + 6, 0) = v;

        for (int k = 0; k < 2; k++) {
            const Vec3 a = Vec3::Unit(k);
            N.template block<3, 1>(c, 1 + k) = a.cross(p);
            N.template block<3, 1>(c + 3, 1 + k) = a;
            if (has_vel) N.template block<3, 1>(c + 6, 1 + k) = a.cross(v);
        }
    }

    const Eigen::Matrix<Scalar, 3, 3> NtN = N.transpose() * N;
    return MatX::Identity(n, n) - N * NtN.ldlt().solve(N.transpose());
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::RecentreGravityScale() {
    GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
    const Vec3 dir = gs.GetDir();
    const Sophus::SO3<Scalar> Q = gs.AlignmentRotation(g);
    const Scalar sigma_c = gs.GetSigma();
    const Scalar s = std::exp(sigma_c);
    const Mat3 Q_inv_m = Q.inverse().matrix();

    auto transform_prior = [&](MargLinData<Scalar>& mld) {
        for (const auto& [id, idx] : mld.order.abs_order_map) {
            if (id == kGravityScaleStateId) continue;
            const int col = idx.first;
            mld.H.middleCols(col, 3) = mld.H.middleCols(col, 3) * Q_inv_m / s;
            mld.H.middleCols(col + 3, 3) =
                mld.H.middleCols(col + 3, 3) * Q_inv_m;
            if (idx.second == POSE_VEL_BIAS_SIZE) {
                mld.H.middleCols(col + 6, 3) =
                    mld.H.middleCols(col + 6, 3) * Q_inv_m / s;
            }
        }
    };

    transform_prior(marg_data);
    transform_prior(mpMargHalf);
    transform_prior(mpMargVisual);

    for (auto& [frame_id, state] : frame_poses) state.TransformBySim3(Q, s);
    for (auto& [frame_id, state] : frame_states) state.TransformBySim3(Q, s);

    gs.TransformLin(Q, sigma_c);

    auto transform_anchor = [&](GsAnchor& anchor) {
        if (!anchor.set) return;
        anchor.dir = Q * anchor.dir;
        anchor.basis = Q.matrix() * anchor.basis;
        anchor.sigma -= sigma_c;
    };

    transform_anchor(mpAnchorCurr);
    transform_anchor(mpAnchorHalf);

    mpSigmaMid -= sigma_c;
    mpSigmaPrevMarg -= sigma_c;

    mpGsStepSigma = sigma_c;
    mpGsStepTiltDeg =
        std::acos(std::clamp(dir.dot(g.normalized()), Scalar(-1), Scalar(1))) *
        Scalar(180) / Scalar(M_PI);

    this->mGsCorrectionLog->Append(Q.template cast<double>(), double(s));
}
```

Column 0 of $\mathbf{N}$ is $\mathbf{n}_\sigma$, columns 1 and 2 are $\mathbf{n}_{\mathbf{e}_x}$ and $\mathbf{n}_{\mathbf{e}_y}$ of §2.5.3. $\mathbf{N}$ is expressed in the unscaled increment coordinates, which is correct because the Jacobi column scaling of `optimize` is commented out (`sqrt_keypoint_vio.cpp:1686-1723`, `:1833`). If it is ever re-enabled, $\mathbf{H}$, $\mathbf{b}$ and the solution are in scaled coordinates $\mathbf{y}$ with $\boldsymbol{\delta} = \text{diag}(\mathbf{j}) \mathbf{y}$ for the scaling vector $\mathbf{j}$, and the projector must then be built from $\text{diag}(\mathbf{j})^{-1} \mathbf{N}$. Eigen evaluates a matrix product into a temporary before assignment, so the in place column updates are free of aliasing. $\mathbf{H}$ is about 80 by 80, so the projector costs well under a millisecond, and the re-centring touches about 60 columns of three priors and about eleven frames, roughly 33 thousand multiply adds per frame. Backward compatibility. Every new statement is guarded by `mpGsActive`, and the unguarded `create` path and the default arguments are today's.

#### Change 14. `marginalize`, three priors and the end of the period

All in `src/vi_estimator/sqrt_keypoint_vio.cpp`, inside `marginalize`.

Swap event reset. Replace

```cpp
    Timer t_total;

    if (frame_poses.size() > max_kfs || frame_states.size() >= max_states) {
```

with

```cpp
    Timer t_total;
    mpGsSwapEvent = 0;

    if (frame_poses.size() > max_kfs || frame_states.size() >= max_states) {
```

Algorithm 1 and the order map. Replace

```cpp
        int64_t last_state_to_marg = it->first;

        AbsOrderMap aom;
```

with

```cpp
        int64_t last_state_to_marg = it->first;

        if (mpGsActive) UpdateScaleInterval();

        AbsOrderMap aom;

        if (mpGsActive) {
            aom.abs_order_map[kGravityScaleStateId] =
                std::make_pair(0, GRAVITY_SCALE_SIZE);

            BASALT_ASSERT(
                marg_data.order.abs_order_map.at(kGravityScaleStateId) ==
                aom.abs_order_map.at(kGravityScaleStateId));

            aom.total_size += GRAVITY_SCALE_SIZE;
            aom.items++;
        }
```

Three linearisations in the period. Replace

```cpp
        {
            Timer t_linearize;
```

with

```cpp
        MatX Q2Jp_half, Q2Jp_visual;
        VecX Q2r_half, Q2r_visual;

        if (mpGsActive) {
            LinearizeMargGS(aom, kfs_to_marg, lost_landmaks, last_state_to_marg,
                            Q2Jp_or_H, Q2r_or_b, Q2Jp_half, Q2r_half,
                            Q2Jp_visual, Q2r_visual);
        } else {
            Timer t_linearize;
```

The existing closing brace of that block now closes the `else`. Packet for the mapper. Replace

```cpp
                } else {
                    m->abs_H = Q2Jp_or_H.template cast<double>();

                    m->abs_b = Q2r_or_b.template cast<double>();
                }

                assign_cast_map_values(m->frame_poses, frame_poses);
```

with

```cpp
                } else {
                    m->abs_H = Q2Jp_or_H.template cast<double>();

                    m->abs_b = Q2r_or_b.template cast<double>();
                }

                if (mpGsActive) StripGsFromMargData(*m);
                if (this->mGsCorrectionLog) {
                    m->mGsEpoch = this->mGsCorrectionLog->LatestEpoch();
                }

                assign_cast_map_values(m->frame_poses, frame_poses);
```

The block is always kept. Replace

```cpp
        for (const auto& kv : aom.abs_order_map) {
            if (kv.second.second == POSE_SIZE) {
```

with

```cpp
        for (const auto& kv : aom.abs_order_map) {
            if (kv.first == kGravityScaleStateId) {
                for (int i = 0; i < GRAVITY_SCALE_SIZE; i++)
                    idx_to_keep.emplace(kv.second.first + i);
                continue;
            }

            if (kv.second.second == POSE_SIZE) {
```

The debug null space prior `nullspace_marg_data` is built without the block, so it is suspended in the period. Replace each of the four occurrences of

```cpp
        if (config.vio_debug || config.vio_extended_logging) {
```

the three inside `marginalize` (`:1325`, `:1493`, `:1502`) and the fourth at `:350` in `ProcessFrame`, which calls `logMargNullspace` at initialisation, with

```cpp
        if ((config.vio_debug || config.vio_extended_logging) && !mpGsActive) {
```

At `:350` the unguarded call would hand `checkNullspace` the order of `marg_data`, which holds the block, against the 15 column `nullspace_marg_data.H`, failing `BASALT_ASSERT(size_t(H.cols()) == order.total_size)` and, past it, the `std::abort` on an entry of size 3 (`sc_ba_base.cpp:600-602`). Asserts are active, since `BASALT_DISABLE_ASSERTS` is commented out (`CMakeLists.txt:409`), and `data/sitl_config_vo.json` sets `vio_debug` true. `FoldGravityScale` resynchronises the debug prior with the folded `marg_data`, so the guards resume on a consistent prior. Note, 2026-11-05. This paragraph previously suspended only the three guards in `marginalize` and left the debug prior stale after the fold. Both defects were found while implementing the plan.

Three priors. Replace

```cpp
        {
            Timer t;
            if (is_lin_sqrt && marg_data.is_sqrt) {
                MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
                    Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_H_new,
                    marg_b_new);
```

with

```cpp
        {
            Timer t;
            if (mpGsActive) {
                MarginalizeGS(Q2Jp_or_H, Q2r_or_b, Q2Jp_half, Q2r_half,
                              Q2Jp_visual, Q2r_visual, idx_to_keep,
                              idx_to_marg);
            } else if (is_lin_sqrt && marg_data.is_sqrt) {
                MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
                    Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_H_new,
                    marg_b_new);
```

New order with the block first. Replace

```cpp
        AbsOrderMap marg_order_new;

        for (const auto& kv : frame_poses) {
```

with

```cpp
        AbsOrderMap marg_order_new;

        if (mpGsActive) {
            marg_order_new.abs_order_map[kGravityScaleStateId] =
                std::make_pair(0, GRAVITY_SCALE_SIZE);
            marg_order_new.total_size += GRAVITY_SCALE_SIZE;
            marg_order_new.items++;
        }

        for (const auto& kv : frame_poses) {
```

Assignment of the new priors. Replace

```cpp
        marg_data.H = marg_H_new;
        marg_data.b = marg_b_new;
        marg_data.order = marg_order_new;
```

with

```cpp
        if (!mpGsActive) {
            marg_data.H = marg_H_new;
            marg_data.b = marg_b_new;
        }
        marg_data.order = marg_order_new;
        if (mpGsActive) {
            mpMargHalf.order = marg_order_new;
            mpMargVisual.order = marg_order_new;
        }
```

Re-centring of the priors. Replace

```cpp
        VecX delta;
        computeDelta(marg_data.order, delta);
        marg_data.b -= marg_data.H * delta;
```

with

```cpp
        if (mpGsActive) {
            RecentrePriorsGS();
        } else {
            VecX delta;
            computeDelta(marg_data.order, delta);
            marg_data.b -= marg_data.H * delta;
        }
```

Log record and end of the period. Replace the last statements of `marginalize`

```cpp
    mpLogger->SolverScratch()
        .add("marginalize", t_total.elapsed())
        .format("ms");
}
```

with

```cpp
    if (mpGsActive) {
        LogGravityScale(mpGsSwapEvent);
        if (last_state_t_ns - mpGsStartNs >=
            int64_t(config.vio_gs_duration_s * 1e9)) {
            FoldGravityScale();
        }
    }

    mpLogger->SolverScratch()
        .add("marginalize", t_total.elapsed())
        .format("ms");
}
```

This is outside the `if (frame_poses.size() > max_kfs || frame_states.size() >= max_states)` block, after the three priors have been rebuilt, the states and landmarks removed and every prior re-centred. Add the seven definitions after `RecentreGravityScale`.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::AnchorGs(GsAnchor& anchor) {
    GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
    if (!anchor.set) {
        anchor.dir = gs.GetDir();
        anchor.basis = gs.GetBasis();
        anchor.sigma = gs.GetSigma();
        anchor.set = true;
    }
    gs.Reanchor(anchor.dir, anchor.basis, anchor.sigma);
}

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
        mpMargHalf = mpMargVisual;
        mpAnchorHalf.set = false;
        mpGsSwapEvent = 1;
    }

    if (sigma > mpSigmaMid + mpLogD) {
        marg_data = mpMargHalf;
        mpAnchorCurr = mpAnchorHalf;
        mpMargHalf = mpMargVisual;
        mpAnchorHalf.set = false;
        mpSigmaMid += mpLogD;
        mpGsSwapEvent = 2;
    }

    if (sigma < mpSigmaMid - mpLogD) {
        marg_data = mpMargHalf;
        mpAnchorCurr = mpAnchorHalf;
        mpMargHalf = mpMargVisual;
        mpAnchorHalf.set = false;
        mpSigmaMid -= mpLogD;
        mpGsSwapEvent = 3;
    }

    mpLastUpper = upper;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::LinearizeMargGS(
    const AbsOrderMap& aom, const std::set<int64_t>& kfs_to_marg,
    const std::unordered_set<KeypointId>& lost_landmaks,
    int64_t last_state_to_marg, MatX& Q2Jp_curr, VecX& Q2r_curr,
    MatX& Q2Jp_half, VecX& Q2r_half, MatX& Q2Jp_visual, VecX& Q2r_visual) {
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

    auto linearize = [&](const MargLinData<Scalar>& mld, GsAnchor& anchor,
                         bool with_imu, bool visual_scale_free, MatX& Q2Jp,
                         VecX& Q2r) {
        AnchorGs(anchor);

        LinearizationGSAbsQR<Scalar, POSE_SIZE> lqr(
            this, aom, lqr_options, &*this->mGsState, visual_scale_free, &mld,
            with_imu ? &ild : nullptr, &kfs_to_marg, &lost_landmaks,
            last_state_to_marg);

        lqr.linearizeProblem();
        lqr.performQR();
        lqr.get_dense_Q2Jp_Q2r(Q2Jp, Q2r);
    };

    GsAnchor visual_anchor = mpAnchorCurr;

    linearize(marg_data, mpAnchorCurr, true, false, Q2Jp_curr, Q2r_curr);
    linearize(mpMargHalf, mpAnchorHalf, true, false, Q2Jp_half, Q2r_half);
    linearize(mpMargVisual, visual_anchor, false, true, Q2Jp_visual,
              Q2r_visual);

    AnchorGs(mpAnchorCurr);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::MarginalizeGS(
    MatX& Q2Jp_curr, VecX& Q2r_curr, MatX& Q2Jp_half, VecX& Q2r_half,
    MatX& Q2Jp_visual, VecX& Q2r_visual, const std::set<int>& idx_to_keep,
    const std::set<int>& idx_to_marg) {
    auto marginalize_into = [&](MargLinData<Scalar>& mld, MatX& Q2Jp,
                                VecX& Q2r) {
        MatX H_new;
        VecX b_new;
        MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
            Q2Jp, Q2r, idx_to_keep, idx_to_marg, H_new, b_new);
        mld.H = H_new;
        mld.b = b_new;
    };

    marginalize_into(marg_data, Q2Jp_curr, Q2r_curr);
    marginalize_into(mpMargHalf, Q2Jp_half, Q2r_half);
    marginalize_into(mpMargVisual, Q2Jp_visual, Q2r_visual);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::RecentrePriorsGS() {
    auto recentre = [&](MargLinData<Scalar>& mld, GsAnchor& anchor) {
        AnchorGs(anchor);
        VecX delta;
        computeDelta(mld.order, delta);
        mld.b -= mld.H * delta;
    };

    GsAnchor visual_anchor = mpAnchorCurr;

    recentre(marg_data, mpAnchorCurr);
    recentre(mpMargHalf, mpAnchorHalf);
    recentre(mpMargVisual, visual_anchor);

    AnchorGs(mpAnchorCurr);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::StripGsFromMargData(
    MargData& m) const {
    constexpr int c = GRAVITY_SCALE_SIZE;
    const int n = int(m.aom.total_size) - c;

    BASALT_ASSERT(m.aom.abs_order_map.at(kGravityScaleStateId).first == 0);

    const Eigen::MatrixXd H_new = m.abs_H.bottomRightCorner(n, n);
    const Eigen::VectorXd b_new = m.abs_b.tail(n);
    m.abs_H = H_new;
    m.abs_b = b_new;

    AbsOrderMap aom_new;
    for (const auto& [id, idx] : m.aom.abs_order_map) {
        if (id == kGravityScaleStateId) continue;
        aom_new.abs_order_map[id] = std::make_pair(idx.first - c, idx.second);
        aom_new.total_size += idx.second;
        aom_new.items++;
    }
    m.aom = aom_new;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::FoldGravityScale() {
    AnchorGs(mpAnchorCurr);

    const GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
    constexpr int c = GRAVITY_SCALE_SIZE;

    BASALT_ASSERT(marg_data.order.abs_order_map.at(kGravityScaleStateId).first ==
                  0);

    marg_data.b += marg_data.H.leftCols(c) * gs.GetDelta();
    MatX J = marg_data.H.rightCols(marg_data.H.cols() - c);

    AbsOrderMap order;
    for (const auto& [id, idx] : marg_data.order.abs_order_map) {
        if (id == kGravityScaleStateId) continue;
        order.abs_order_map[id] = std::make_pair(idx.first - c, idx.second);
        order.total_size += idx.second;
        order.items++;
    }

    marg_data.H = J;
    marg_data.order = order;

    // The debug prior is frozen during the period, resynchronised here.
    nullspace_marg_data.H = marg_data.H;
    nullspace_marg_data.b = marg_data.b;
    nullspace_marg_data.order = marg_data.order;

    mpGsStepSigma = 0;
    mpGsStepTiltDeg = 0;
    LogGravityScale(4);

    mpMargHalf = MargLinData<Scalar>();
    mpMargVisual = MargLinData<Scalar>();
    this->mGsState.reset();
    this->mGsStateBackup.reset();
    mpGsActive = false;
}
```

`AnchorGs` is also the freeze. The first marginalisation of the period finds `mpAnchorCurr` unset, records the block's current value as its anchor and re-anchors the block there, which sets `mLinearized` with a zero offset. The packet's `abs_H` is $(\mathbf{Q}_2^\top \mathbf{J}_p)^\top(\mathbf{Q}_2^\top \mathbf{J}_p)$ of $M_\text{curr}$ and still carries the block, which `StripGsFromMargData` removes by conditioning on its current value, deleting its rows and columns (§2.9.3). Note, 2026-11-05. This function previously marginalised the block with `MargHelper<double>::marginalizeHelperSqToSq`, which leaves the packet rank deficient by three and makes `NfrMapper::extractNonlinearFactors` reject it (§2.9.3). The fold leaves `marg_data` with the frame only order that `optimize` builds once `mpGsActive` is false, so the prefix assertions hold in the next frame. It also copies the folded prior into `nullspace_marg_data`, whose updates were suspended in the period, so that the debug null space log resumes on a prior of matching size. That log then measures the prior including the initial pose and yaw information rather than the version without it, a loss confined to a debug diagnostic. The correction log stays alive after the period, so updates the mapper computed in an older epoch are still carried forward. Backward compatibility. Every new statement is guarded by `mpGsActive` or by the existence of the correction log.

#### Change 15. `VioEstimatorBase` and the controller wiring

In `include/basalt/vi_estimator/vio_estimator.h`, replace (`:123-126`)

```cpp
    virtual void QueuePoseUpdates(
        const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>& updates) {
        (void)updates;
    }
```

with

```cpp
    virtual void QueuePoseUpdates(
        const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>& updates,
        int64_t gsEpoch = 0) {
        (void)updates;
        (void)gsEpoch;
    }

    // Cumulative gravity and scale corrections, created by an estimator that
    // optimises them, null otherwise.
    GravityScaleCorrectionLog::Ptr mGsCorrectionLog;
```

In `src/controller.cpp`, replace (`:217-219`)

```cpp
    local_mapper_->SetKFInputQueue(&local_map_kf_queue_);
    local_mapper_->SetVIOPoseUpdateCallback(
        [this](const auto& poses) { vio_estimator_->QueuePoseUpdates(poses); });
```

with

```cpp
    local_mapper_->SetKFInputQueue(&local_map_kf_queue_);
    local_mapper_->SetGravityScaleCorrectionLog(
        vio_estimator_->mGsCorrectionLog);
    local_mapper_->SetVIOPoseUpdateCallback(
        [this](const auto& poses, int64_t gsEpoch) {
            vio_estimator_->QueuePoseUpdates(poses, gsEpoch);
        });
```

The log is created in the estimator's constructor (Change 11), which runs at `src/controller.cpp:193`, before this wiring. Backward compatibility. The default argument keeps every one argument call valid, the base implementation stays a no-op, and with the mode off the log pointer handed to the mapper is null.

#### Change 16. `LocalMapper`, `include/basalt/vi_estimator/local_mapper.h` and `src/vi_estimator/local_mapper.cpp`

In the header, replace

```cpp
    using PoseUpdateCallback = std::function<void(
        const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>&)>;
```

with

```cpp
    using PoseUpdateCallback = std::function<void(
        const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>&,
        int64_t)>;
```

insert after `void SetVIOPoseUpdateCallback(PoseUpdateCallback cb);`

```cpp
    void SetGravityScaleCorrectionLog(GravityScaleCorrectionLog::Ptr log);
```

and insert after `PoseUpdateCallback mpVioPoseUpdateCallback;`

```cpp
    GravityScaleCorrectionLog::Ptr mpGsLog;
    int64_t mpMapGsEpoch = 0;

    void CatchUpGravityScale();
    void ApplyGravityScaleCorrection(
        const GravityScaleCorrectionLog::Entry& correction);
    static void TransformMargDataBySim3(
        MargData& data, const GravityScaleCorrectionLog::Entry& correction);
```

In the source, insert after the definition of `SetVIOPoseUpdateCallback` (`:75-77`)

```cpp
void LocalMapper::SetGravityScaleCorrectionLog(
    GravityScaleCorrectionLog::Ptr log) {
    mpGsLog = std::move(log);
}

void LocalMapper::CatchUpGravityScale() {
    if (!mpGsLog) return;

    const int64_t latest = mpGsLog->LatestEpoch();
    if (latest == mpMapGsEpoch) return;

    ApplyGravityScaleCorrection(mpGsLog->Between(mpMapGsEpoch, latest));
    mpMapGsEpoch = latest;
}

void LocalMapper::ApplyGravityScaleCorrection(
    const GravityScaleCorrectionLog::Entry& correction) {
    for (auto& [t_ns, state] : frame_poses) {
        state = PoseStateWithLin<double>(t_ns, correction.Apply(state.getPose()));
    }

    for (const auto& kv : lmdb.getLandmarks()) {
        lmdb.getLandmark(kv.first).inv_dist /= correction.mScale;
    }

    Sophus::Matrix6d D_inv = Sophus::Matrix6d::Identity();
    D_inv.topLeftCorner<3, 3>() /= correction.mScale;
    for (RelPoseFactor& rpf : rel_pose_factors) {
        rpf.T_i_j.translation() *= correction.mScale;
        rpf.cov_inv = D_inv * rpf.cov_inv * D_inv;
    }

    for (RollPitchFactor& rpf : roll_pitch_factors) {
        rpf.R_w_i_meas = correction.mQ * rpf.R_w_i_meas;
    }
}

void LocalMapper::TransformMargDataBySim3(
    MargData& data, const GravityScaleCorrectionLog::Entry& correction) {
    const double s = correction.mScale;
    const Eigen::Matrix3d Q_inv_m = correction.mQ.inverse().matrix();

    for (auto& kv : data.frame_poses) kv.second.TransformBySim3(correction.mQ, s);
    for (auto& kv : data.frame_states) kv.second.TransformBySim3(correction.mQ, s);

    for (const auto& [id, idx] : data.aom.abs_order_map) {
        const int col = idx.first;
        std::vector<std::pair<int, Eigen::Matrix3d>> blocks = {
            {col, Q_inv_m / s}, {col + 3, Q_inv_m}};
        if (idx.second == POSE_VEL_BIAS_SIZE) {
            blocks.emplace_back(col + 6, Q_inv_m / s);
        }

        for (const auto& [start, M] : blocks) {
            data.abs_H.middleCols(start, 3) = data.abs_H.middleCols(start, 3) * M;
            data.abs_H.middleRows(start, 3) =
                M.transpose() * data.abs_H.middleRows(start, 3);
            data.abs_b.segment<3>(start) =
                M.transpose() * data.abs_b.segment<3>(start);
        }
    }
}
```

In `MapLocally`, replace

```cpp
        mpNewKeyframesForTracking.clear();
        mpLatestKeyframesMatches.clear();

        for (Keyframe::Ptr& k : vKeyframes) {
```

with

```cpp
        mpNewKeyframesForTracking.clear();
        mpLatestKeyframesMatches.clear();

        CatchUpGravityScale();

        for (Keyframe::Ptr& k : vKeyframes) {
```

and replace (`:257`)

```cpp
        if (mpVioPoseUpdateCallback) mpVioPoseUpdateCallback(frame_poses);
```

with

```cpp
        if (mpVioPoseUpdateCallback) {
            mpVioPoseUpdateCallback(frame_poses, mpMapGsEpoch);
        }
```

In `IngestKeyframe`, replace

```cpp
    frame_poses[t_ns] = PoseStateWithLin<double>(t_ns, kf->pose.getPose());
```

with

```cpp
    Sophus::SE3d T_w_i = kf->pose.getPose();
    if (mpGsLog && kf->mGsEpoch != mpMapGsEpoch) {
        T_w_i = mpGsLog->Between(kf->mGsEpoch, mpMapGsEpoch).Apply(T_w_i);
    }
    frame_poses[t_ns] = PoseStateWithLin<double>(t_ns, T_w_i);
```

In `IngestMargData`, replace

```cpp
    data->opt_flow_res.clear();

    const size_t relBegin = rel_pose_factors.size();
```

with

```cpp
    data->opt_flow_res.clear();

    if (mpGsLog && data->mGsEpoch != mpMapGsEpoch) {
        TransformMargDataBySim3(
            *data, mpGsLog->Between(data->mGsEpoch, mpMapGsEpoch));
        data->mGsEpoch = mpMapGsEpoch;
    }

    const size_t relBegin = rel_pose_factors.size();
```

The catch up runs after the blocking pop and before any keyframe or packet of the cycle is ingested, so the whole cycle, including matching, track building, bundle adjustment, culling and the pose update callback, runs in one epoch. Backward compatibility. With the mode off `mpGsLog` is null, so `CatchUpGravityScale` returns at once and both ingestion branches are skipped. The only change on that path is the extra callback argument, which the controller's lambda receives and forwards as 0.

#### Change 17. Documentation and context

`doc/VIO.md` gains a section on the initialisation period after the implementation has landed, summarising §2 of this plan with its derivations. `context/vio_opt.md` already carries the extension notes uncovered while planning this change (§15 of that file), and gains a note on the per frame re-centring and the mapper synchronisation when this plan is implemented. `context/keyframe_driven_local_mapping.md` gains the epoch contract of §2.9.

### 3.3 Blast Radius

| Symbol | Consumers | Effect outside the period |
|---|---|---|
| `ImuBlock` virtual methods | `LinearizationAbsQR`, `LinearizationAbsSC`, `LinearizationRelSC` | none, same arithmetic, one indirection |
| `ImuBlock::LinearizeBiasRandomWalk` | `ImuBlock::linearizeImu`, `ImuBlockGS::linearizeImu` | pure extraction, sum order kept |
| `RelPoseLin::d_rel_d_logscale` | all landmark blocks | memory only, never read |
| `LandmarkBlockAbsDynamic::mpGsScaleCol` | `LinearizationAbsQR` | −1, branch never taken |
| `LinearizationAbsQR::linearizeRelPoses` | `linearizeProblem` | pure extraction |
| `BundleAdjustmentBase::computeDelta` | VIO, VO, `NfrMapper`, local mapper | new branch unreachable |
| `computeError` `relScale` | VIO, VO, local mapper | default 1, `computeRelPose` path |
| `computeImuError` `scale` | VIO, `NfrMapper` | default 1, `residual` path |
| `backup`, `restore` | VIO, VO | `mGsState` empty, no-op |
| `PoseStateWithLin`, `PoseVelBiasStateWithLin` `TransformBySim3` | estimator, local mapper | not called |
| `Keyframe::mGsEpoch`, `MargData::mGsEpoch` | VIO, VO, local mapper, `NfrMapper` | default 0, identity of the log |
| `VioEstimatorBase::QueuePoseUpdates` | controller lambda, `SqrtKeypointVioEstimator` | default `gsEpoch = 0` |
| `VioEstimatorBase::mGsCorrectionLog` | controller, local mapper | null |
| `LocalMapper::PoseUpdateCallback` | controller | one extra argument, forwarded |
| `VioConfig` keys | seven JSON files, batch TOML, `scripts/analysis/metrics.py` | default false, keys must be added everywhere |
| `IntegratedImuMeasurement` | everywhere | one added method only |

## 4. Validation Steps

### 4.1 Code reading by the AI, before any run

1. Confirm that with `vio_gs_enable = false` no new statement executes, by tracing every `mpGsActive`, `mGsState`, `mGsCorrectionLog` and `mpGsLog` guard and every default argument.
2. Confirm that `computeRelPoseGS` at `scale = 1` reproduces `computeRelPose` term by term, and `residualGS` at $\mathbf{g}_D = \mathbf{g}$, `scale = 1` reproduces `residual`.
3. Confirm the prefix invariant in `optimize` and in every one of the three `linearize` calls of `LinearizeMargGS`, and that all three priors share `marg_order_new`.
4. Confirm the cost triple, namely that `error_total`, `l_diff` and `after_error_total` each include the scale in the visual term, $\mathbf{g}_D$ and $s$ in the inertial term, and the block's offset $\boldsymbol{\delta}_{gs}$ in the prior.
5. Confirm that `ImuBlockGS::add_dense_H_b` and `add_dense_Q2Jp_Q2r` write the block's rows and columns at `aom` offset 0, and that `get_dense_Q2Jp_Q2r` still reserves 15 rows per factor.
6. Confirm that `backup` and `restore` capture the block, so that a rejected step leaves it unchanged.
7. Re-read §2.5.3 against `ComputeGaugeProjector`, in particular that the null vectors use the linearisation point values and that the tilt vector's block component is $\bar{\mathbf{B}}^\top \mathbf{a}$.
8. Re-read §2.7.3 against `RecentreGravityScale`, in particular that only frame columns are multiplied, that both anchors and both interval variables are shifted, and that the log is appended once per call.
9. Re-read §2.9 against Change 16, in particular that every item entering the mapper is brought to `mpMapGsEpoch` before use, and that the callback carries `mpMapGsEpoch`.
10. Syntax check every touched translation unit with the recipe of `/ws/skills/cpp-syntax-check-tooling.md`, through `src/vi_estimator/sqrt_keypoint_vio.cpp`, `src/linearization/linearization_abs_qr.cpp`, `src/vi_estimator/ba_base.cpp`, `src/vi_estimator/sc_ba_base.cpp`, `src/vi_estimator/local_mapper.cpp`, `src/controller.cpp` and `src/utils/vio_config.cpp`.

Run on 2026-11-05. Item 10 passed for the seven units above and for `src/utils/logger.cpp`, `src/vi_estimator/sqrt_keypoint_vo.cpp`, `src/vi_estimator/nfr_mapper.cpp`, `src/vi_estimator/vio_estimator.cpp`, `src/linearization/linearization_base.cpp`, `src/linearization/linearization_abs_sc.cpp` and `src/linearization/linearization_rel_sc.cpp`, with no error or warning, under GCC 12.2 and the vendored Eigen 3.4.0, with both the double and the float instantiations enabled. Items 1, 3, 4, 5, 6 and 9 were checked by reading. Reading for item 1 found that the fourth debug guard, at `ProcessFrame`, also had to be suspended, which Change 14 now does. Items 2, 7 and 8 rest on the derivations reviewed in §2 and are exercised numerically by tests 1 to 5 of §4.2. `test/src/test_vio.cpp` is not in the container's compile database, so its continued compilation against the now virtual `ImuBlock` was confirmed by reading only. It constructs `ImuBlock<double>` directly, which remains a concrete class.

### 4.2 Unit tests, run by the user

New cases next to the existing ones, using the `test_jacobian` numeric differentiation helper already used at `thirdparty/basalt-headers/test/src/test_preintegration.cpp:149-307` and the patterns of `test/src/test_vio.cpp`.

1. `ImuPreintegrationTestCase.ResidualGSTest`. Numeric against analytic Jacobians of `residualGS` for both states, both biases, $\mathbf{g}_D$ and the log scale, at random $\mathbf{g}_D$ within 20° of $\mathbf{g}$ and $s \in [0.3, 3]$. Equality with `residual` at $\mathbf{g}_D = \mathbf{g}$, $s = 1$.
2. `VioTestSuite.RelPoseGSTest`. Numeric against analytic `d_rel_d_h`, `d_rel_d_t` and `d_rel_d_logscale` of `computeRelPoseGS`, extending `RelPoseTest` (`test/src/test_vio.cpp:315`).
3. `VioTestSuite.GravityDirectionJacobianTest`. Numeric against analytic $\partial\mathbf{g}_D/\partial\boldsymbol{\delta}_\theta = -[\bar{\mathbf{g}}_D]_\times\bar{\mathbf{B}}$, and exactness of `Reanchor` and `TransformLin` (the current direction and scale are unchanged to 1e-12).
4. `VioTestSuite.GaugeNullspaceGSTest`. On a synthetic window of the kind built by `ImuNullspace2Test` (`:28`), with landmarks, check $|\mathbf{H} \mathbf{n}| / |\mathbf{H}| < 10^{-8}$ for the three vectors of §2.5.3, and that after projection the step has no component along them.
5. `VioTestSuite.RecentreInvarianceTest`. With the block at a random $\mathbf{d}$ and $s$, the total cost, visual plus inertial plus each of the three priors, before and after `RecentreGravityScale` agrees to relative 1e-9, and the block is the identity afterwards.
6. `VioTestSuite.DynamicMargTest`. Driving $\sigma$ across a boundary swaps $M_\text{curr}$ with $M_\text{half}$ and resets $M_\text{half}$, with $M_\text{visual}$ unchanged by a change of $\sigma$ alone.
7. `LocalMapperTestSuite.GravityScaleCatchUpTest`. A map, factors and a packet transformed by two successive corrections equal the same transformed once by their composition, the factor costs are unchanged, and for a camera without lever arm every landmark's world position maps to $s\,\mathbf{Q} \mathbf{X}$.
8. `LocalMapperTestSuite.MargDataTransformTest`. The quadratic form of a transformed packet, evaluated at mapped offsets, equals the original to relative 1e-12.
9. `VioTestSuite.StripGsFromMargDataTest`. On a packet built during the period, the stripped `abs_H` has full rank, `NfrMapper::extractNonlinearFactors` returns true, and the stripped quadratic form equals the original evaluated at a zero block increment.

### 4.3 Replays and flights, run by the user

The estimator is not deterministic on recorded data because of the local mapper correction loop (`context/takeoff_bias_divergence.md` §5a), so every comparison uses several runs per configuration.

1. Regression with the mode off. Replay EuRoC MH_01 and the SITL bag `/ws/data/telemetry_18` with `vio_gs_enable = false` against the current binary. With the local mapper disabled, the `vio_state.csv` files must be identical. With it enabled, the distribution of ATE over five runs must be indistinguishable.
2. EuRoC with the mode on, five runs each of MH_01, V1_02 and V1_03. The scale factor of a Sim(3) Umeyama alignment must not be worse than with the mode off, and the gravity direction error against ground truth after the period must not be worse.
3. SITL take off, five replays each of `telemetry_18` and of the 2026-10-26 flights, mode off and on, with the configuration of `takeoff_bias_divergence.md` §10. Report the vertical world frame accelerometer bias at take off plus 1 s, the speed ratio in the climb, the tilt error against ground truth at plus 1 s and plus 10 s, the endpoint drift, and the bounded or diverged outcome, which are the measures that file established.
4. From the `vio_gravity_scale` record, check that the scale and tilt steps move during the take off transient rather than the bias, that swaps occur when the cumulative scale moves by more than a few percent, and that the steps are small when the period ends. Also compare each frame's $\sigma$ step with the scale change carried by the frame positions in the same step, to confirm the share predicted in §2.5.7.
5. Mapper consistency. Compare the local mapper's keyframe poses with the estimator's `frame_poses` for the same timestamps, and the mapper's landmark positions with the estimator's for shared tracks, over the period. Both must agree to the mapper's usual refinement level, with no step at any epoch change.
6. Timing. `optimize` time in the period, expected to rise by the projector and the re-centring only, `marginalize` time, expected about three times today's, and the mapper's catch up time, expected well under a millisecond.
7. Duration sweep. `vio_gs_duration_s` at 5, 10 and 20 s on the SITL replays, to choose the default.

Record of runs, 2026-11-06. The first runs were made in the dev container on the SITL bag `/ws/data/telemetry_40`, which takes off about 5 s after initialisation. Each configuration ran once, so the non determinism noted above is not yet averaged out.

1. The crash reported from `djinn start sitl2 basalt`, `BASALT_ASSERT(size_t(mld.H.cols()) == mld.order.total_size)` in `checkNullspace` called from `logMargNullspace` at initialisation, with `15 18` as its message, was a stale binary and not a defect of the formulation. The assertion is the one this plan predicts for the unguarded initialisation call (Change 14), and the working tree carries the guard `&& !mpGsActive` at `sqrt_keypoint_vio.cpp:363`. The installed object `sqrt_keypoint_vio.cpp.o` had been written 19 s after the last edit of the source, while compiling that file takes about 60 s, so it was built from an earlier version, and `make` then treated it as up to date. After `touch src/vi_estimator/sqrt_keypoint_vio.cpp` and `colcon build --packages-select slam` the object changed size and both replays passed initialisation without an assertion.
2. Replays must run on an isolated `ROS_DOMAIN_ID`, with the decoder pointed away from the camera topic, because a live `sitl2` stack on domain 0 and the double camera stream of the replay launch both deliver duplicate frames (`/ws/context/djinn-workflows.md`, `djinn start basalt-replay`). The first mode on run was contaminated in this way and is discarded.
3. Results, from `gt_eval.csv` of `/ws/logs/slam/2026-11-06_16-56-02` (mode off, domain 42) and `/ws/logs/slam/2026-11-06_17-06-11` (mode on, domain 43). The scale ratio is the length of the estimated displacement over 2 s divided by that of ground truth. The tilt error is the angle between the body frame directions of the world $z$ axis in the estimate and in ground truth, which does not depend on yaw.

| Window | Mode off, scale ratio | Mode off, tilt error | Mode on, scale ratio | Mode on, tilt error |
|---|---|---|---|---|
| 56 to 58 s, take off | 3.55 | 0.2° | 0.18 | 0.7° |
| 60 to 62 s, end of the period | 5.31 | 6.0° | 0.78 | 18.4° |
| 64 to 66 s | 2.09 | 2.5° | 0.86 | 26.7° |
| 66 to 68 s | 2.54 | 3.2° | 1.00 | 27.0° |
| 70 to 72 s | 2.73 | 3.9° | 1.08 | 27.1° |
| 80 to 82 s | 2.71 | 2.4° | 1.02 | 26.1° |

4. The scale part works on this bag. The mode off run reproduces the take off failure of `context/takeoff_bias_divergence.md` with a scale about 2.7 times too large, while after the period the mode on run is within 15 % of ground truth.
5. The tilt part fails. The block's cumulative tilt in `vio_gravity_scale.csv` reaches 17.7° at the fold, which accounts for most of the 27° attitude error that follows, whereas the mode off run keeps the tilt error between 1° and 6°.
6. The steps within the period are ill conditioned. During the take off transient, between 55.9 s and 57.5 s, single frame steps reach $\delta\sigma = -0.56$ (56.58 s) and 11.9° of tilt (57.15 s), and the lower boundary swap of §2.6.3 fires about fifteen times, each time replacing $M_\text{curr}$ by $M_\text{half}$ and discarding scale information, so that $\sigma$ falls to $\ln 0.30$ within 1.6 s and stays there. From 58.8 s to the fold the scale is steady, but the tilt drifts monotonically at about 7° per second with steps below 1.1° per frame. A drift of one sign under small steps points to a systematic cause rather than noise, and none has been identified yet.
7. After the fold, the debug null space check reports yaw energy near 600 instead of 0.2 in the mode off run. `FoldGravityScale` resynchronises `nullspace_marg_data` with the full `marg_data`, which contains the initial prior's pins, so the debug metric is no longer prior free after the period. The estimate is unaffected.

Items 5 and 6 are open defects of the method, not of the build, and are carried into the next revision of this plan. Before any change, the cause of the monotonic tilt drift should be isolated. Two candidates are a sign or frame inconsistency between the tilt applied by `RecentreGravityScale` and the tilt the next solve expects, which a run with the block's tilt columns zeroed and the scale kept would expose, and a weakly observed horizontal accelerometer bias trading with tilt during the climb, which the bias columns of `vio_state.csv` would show.

Second set of runs, 2026-11-06. Three further replays of `telemetry_40` on isolated domains, and the user's live `sitl2` runs, extend the record above. The log directories are `/ws/logs/slam/2026-11-06_17-20-13` (A, mode off, `vio_init_ba_weight` $10^4$), `/ws/logs/slam/2026-11-06_17-20-05` (B, mode on, `vio_init_ba_weight` $10^4$, `vio_gs_duration_s` 70) and the live run `/ws/logs/slam/2026-11-06_17-15-14` (mode on, `vio_init_ba_weight` 10, `vio_gs_duration_s` 70). The directory `/ws/logs/slam/2026-11-06_17-16-53` is invalid, because two replays started in the same second and the logger, which names its directory by the wall clock second, interleaved both into it.

| Run | Tilt error after take off | Scale ratio, 64 to 100 s | Outcome |
|---|---|---|---|
| A, mode off, strong gyroscope bias prior | 1.5° to 3.5° | 1.9 to 2.8 | stable, scale still too large |
| B, mode on, strong gyroscope bias prior, 70 s | 6° rising to 25°, then 74° to 113° | 0.55, then 10 to 19 | diverges |

Note, 2026-11-06. The first column previously read "strong bias prior" for A and B, meaning the accelerometer bias. The constructor writes `vio_init_ba_weight` to tangent indices 9 to 11, which are the gyroscope bias (`sqrt_keypoint_vio.cpp:114-117`, `context/linearisation.md`, confirmed defect), so $10^4$ stiffened the gyroscope bias and the accelerometer bias kept `vio_init_bg_weight` $= 10^2$, a standard deviation of 0.1 m/s².
| live, mode on, default prior, 70 s | 86° to 114° from 280 s | not computed | diverges |

8. Tilt is traded for horizontal accelerometer bias in every run, mode off included. At every sample the horizontal bias magnitude equals $|\mathbf{g}| \sin(\text{tilt error})$ to within about 0.1 m/s², and the bias vector opposes the body frame error of the gravity direction, for example $\mathbf{b}_a = (+1.01, -4.28)$ against $|\mathbf{g}| \Delta\hat{\mathbf{z}} = (-1.04, +4.19)$ at 80 s of the first isolated run. At constant attitude the accelerometer cannot separate the two, so only the bias prior anchors the tilt. With the SITL IMU ideal, any estimated accelerometer bias is tilt error in disguise, which is the growth of the $x$ bias the user observed in live runs.
9. The bias random walk does not supply that anchor. It penalises differences between consecutive biases only, so a uniform shift of every bias in the window and the prior costs nothing under it. The per frame bias changes logged reach 0.37 m/s², more than 1000 standard deviations of one random walk step. The absolute anchor is the initial prior on the first state, propagated through the marginalised chain. In run A the accelerometer bias prior had a standard deviation of 0.1 m/s², set by `vio_init_bg_weight` $= 10^2$, and the bias reached 0.7 m/s², seven standard deviations. Whether the anchor also leaks is therefore not established. The rebuilding of frozen keyframe linearisation points by accepted local mapper pose updates (`sqrt_keypoint_vio.cpp:640-650`), which moves a frozen pose's linearisation point without rebuilding the prior, remains a candidate. Note, 2026-11-06. This sentence previously stated that the prior in run A had a standard deviation of 0.01 m/s² and that a leak was therefore established. `vio_init_ba_weight` $= 10^4$ in run A reached the gyroscope bias, not the accelerometer bias (see the note under the table above).
10. In the mode, every swap that resets $M_\text{half}$ to $M_\text{visual}$ removes the anchor entirely, because $M_\text{visual}$ holds no inertial rows, so the initial bias prior on the first state is marginalised without any link to later states. In the first isolated run the bias drift began at 58.8 s, right after the last of 24 swaps at 58.5 s.
11. At take off the block is solved from almost no information. At rest $\mathbf{n}_\sigma$ approaches $-\mathbf{e}_\sigma$ (§2.5.7), so $\sigma$ is itself a gauge direction and the projection leaves the block with no information on scale. When motion starts the information grows from zero, and single solves return steps of $\delta\sigma = -0.46$ and 10° to 42° of tilt in one frame (live run, 248.3 s to 250.3 s). The interval of §2.6.3, $\pm\ln\sqrt{1.1} \approx \pm 4.8$ %, is crossed by nearly every such step, so the swaps fire almost every frame and discard what information had accumulated, which lets the next step be large again.
12. The per frame re-centring of §2.7 applies each such step at once to every state and prior, and no later frame can reject it. The errors therefore compound, to a cumulative scale of 0.04 in the live run, and a longer period gives longer exposure, which is why the 70 s runs diverge while the 10 s run did not.

The solution strategy derived from these findings, in order of increasing change, is recorded here for the next revision and has not been implemented.

1. Gate the block on information rather than on time. Hold the block at the identity, which is exactly today's estimator, while the vehicle is stationary or while the projected marginal information on $\sigma$ and the tilt, the Schur complement of $\mathbf{P} \mathbf{H} \mathbf{P}$ onto the block's three coordinates, is below a threshold, and apply a correction only when its predicted standard deviation is small.
2. Bound the applied step per frame, for example $|\delta\sigma| \le 0.02$ and 0.3° of tilt, as a trust region on what the re-centring applies.
3. Keep the anchor through swaps. Build $M_\text{visual}$ from every row that does not depend on $s$ or $\mathbf{g}_D$, which besides the visual rows with the scale column zeroed includes the inertial rotation rows $\mathbf{r}_R$, the bias random walk rows and the initial prior. These rows are independent of the block, so $M_\text{visual}$ stays scale independent as VI-DSO requires (§2.6.2).
4. Remove the tilt from the block and keep the scale only. Run A shows that the estimator holds the tilt to 1.5° to 3.5° with the default accelerometer bias prior and no tilt variable, while the scale error of about 2.7 persists, which is the error the block corrects. Dropping the tilt removes two gauge directions and the path by which the block trades tilt for bias.
5. Set the accelerometer bias prior from the accelerometer's turn on bias specification. Because of the index transposition, that prior is `vio_init_bg_weight`, not `vio_init_ba_weight`. Then test whether the anchor leaks, as item 9 asks. Note, 2026-11-06. This item previously named `vio_init_ba_weight`, which reaches the gyroscope bias. The item previously also took the leak as established.

Items 1 to 3 were taken up by `plans/gravity_scale_optimisation_2.md` (2026-11-06), item 1 in the soft form of a random walk residual on the per frame step, because the block's marginal information is identically zero under the gauge, item 2 by the same residual inside the solve rather than by limiting what the re-centring applies, and item 3 with the refinement that the rotation rows are weighted by their marginal covariance and that whole factors without acceleration are admitted with the scale column zeroed. Items 4 and 5 were not adopted there, for the reasons of its §3.6.

## 5. Conclusion

The plan adds gravity direction and metric scale as an explicit global block of the Basalt sliding window for the first seconds of operation, tightly coupled into both residual families. The inertial residual takes ORB-SLAM3's gravity and scale form [1] in Basalt's conventions. The visual residual takes the scale through the IMU to IMU relative translation, with the exact derivatives of §2.2, while gravity direction provably does not enter it and the reprojection code itself is unchanged. Three pieces of mathematics make the extra variables meaningful and the result usable. The three gauge freedoms they introduce are removed by projection, so within a frame every metric correction flows through $\sigma$ and $\boldsymbol{\delta}_\theta$. The VI-DSO three prior marginalisation [2] keeps the prior from freezing information linearised at a scale far from the current estimate, which is the mechanism by which the take off scale is held today (R5). The per frame re-centring is an exact change of variables that applies each frame's correction to every state and prior while keeping the block's information, so the estimator's states and map are metric and gravity aligned at all times, and an epoch stamped correction log lets the local mapper apply the same corrections to its own map and run from the first keyframe.

The expected impact is on the failure documented in `context/takeoff_bias_divergence.md`. During a take off transient that the images cannot resolve, a scale and tilt error can now be expressed as a change of $\sigma$ and $\boldsymbol{\delta}_\theta$ rather than being absorbed by the accelerometer bias, the prior can release a scale it has been holding once the estimate moves beyond the interval, and the local map is corrected together with the odometry instead of preserving the scale it was built with. Whether this prevents divergence on a vertical climb, whose bias along the direction of travel remains unobservable, is an empirical question that §4.3 is designed to answer. The plan does not claim it.

## References

[1] C. Campos, J. M. M. Montiel, J. D. Tardós, "Inertial-Only Optimization for Visual-Inertial Initialization", IEEE ICRA 2020, arXiv:2003.05766. Venue and authors confirmed by web search on 2026-11-04. The implementation read is `/ws/ros_ws/src/slam/orb_slam3/src/Optimizer.cc:3044-3226`, `G2oTypes.cc:596-718` and `LocalMapping.cc:192-254`, `:1185-1504`.

[2] L. von Stumberg, V. Usenko, D. Cremers, "Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization", IEEE ICRA 2018, arXiv:1804.05625. Sections III-E, III-F and Algorithm 1, and eq. 14 to 17, were read from the arXiv PDF on 2026-11-04. Its unofficial stereo implementation, VI-Stereo-DSO, was cloned into the session scratchpad and read at `src/FullSystem/FullSystemOptimize.cpp:265-286`.

[3] L. von Stumberg, D. Cremers, "DM-VIO: Delayed Marginalization Visual-Inertial Odometry", IEEE Robotics and Automation Letters, 2022. The code at `github.com/lukasvst/dm-vio` was cloned on 2026-11-04 and read at `src/GTSAMIntegration/PoseTransformationIMU.cpp:57-200` and `src/IMU/BAIMULogic.cpp:115-128`, `:280-345`, `:455-500`. The venue is cited from the repository's own header and README, not from the paper.

[4] J. Engel, V. Koltun, D. Cremers, "Direct Sparse Odometry", IEEE TPAMI, 2018. Cited from memory. The null space orthogonalisation it introduced was read in DM-VIO's copy, `src/dso/OptimizationBackend/EnergyFunctional.cpp:720-829`, and `src/dso/util/settings.cpp:81`.

[5] Z. Zhang, G. Gallego, D. Scaramuzza, "On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation", IEEE Robotics and Automation Letters, vol. 3, no. 3, pp. 2710-2717, 2018. Venue confirmed by web search on 2026-11-04. Only the abstract was read, which names the three treatments of §2.5.3.
