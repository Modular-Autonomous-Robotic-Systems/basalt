# VIO Residuals, Jacobians and the Initial Prior

Reference notes derived while restructuring `doc/VIO.md` and `doc/Marginalisation.md` on 2026-08-31. Everything below was read from source rather than recalled, and the `file:line` anchors are against the tree as of that date.

## Documentation split between the two doc files

`doc/VIO.md` is the repository of estimation mathematics. It holds the reprojection residual and its Jacobians (§2.2.4, §2.2.5), the sliding-window state definition, the IMU residual and its Jacobians, the marginalisation residual statement and the complete VIO objective (§3.1), and the implementation walkthroughs (§2.3 for VO, §3.2 for VIO). `doc/Marginalisation.md` holds only the Gauss-Newton linearisation (§2), the Schur complement and its code orchestration (§3), and the full derivation of the marginalisation residual (Appendix A).

Before 2026-08-31 `doc/Marginalisation.md` carried a section 3 titled "VIO Frame to Frame Tracking" that duplicated the problem statement and the residuals. It was moved into `doc/VIO.md` §3, the remaining `Marginalisation.md` sections were renumbered down by one, and the subsection previously misnumbered 5.3, which actually sat inside section 4, became §3.3. `doc/VIO.md` sections 3 through 9 were correspondingly renumbered to 4 through 10.

## IMU residual, as Basalt actually computes it

`IntegratedImuMeasurement::residual` (`thirdparty/basalt-headers/include/basalt/imu/preintegration.h:221`). The nine-vector is ordered position, rotation, velocity, matching the state tangent ordering.

```
r_p = R0^T (p1 - p0 - v0 dt - 0.5 g dt^2) - (Dp + dg[0:3] + da[0:3])
r_R = Log( Exp(dg[3:6]) * DR * R1^{-1} * R0 )
r_v = R0^T (v1 - v0 - g dt)             - (Dv + dg[6:9] + da[6:9])
```

with `dg = J_{D|bg} (bg - bg_lin)` and `da = J_{D|ba} (ba - ba_lin)`.

The rotation row is **not** the Forster et al. (2017) form `Log((DR)^T R0^T R1)`. Basalt forms `DR R1^{-1} R0`, which is the group inverse of the Forster argument up to conjugation. The two agree in norm and therefore in cost, but not in sign or in Jacobian structure. `doc/Marginalisation.md` previously stated the Forster form and it was corrected on 2026-08-31.

`preintegration.h:234` asserts that `da[3:6]` is identically zero, because the orientation delta is integrated from the gyroscope alone and carries no accelerometer-bias dependence.

## IMU residual Jacobians

Perturbation convention from `PoseVelBiasState::applyInc` (`thirdparty/basalt-headers/include/basalt/imu/imu_types.h:221`), namely `R <- Exp(dphi) R` on the left, everything else additive, with tangent ordering `[dp, dphi, dv, dbg, dba]`.

Writing `t = R0^T (p1 - p0 - v0 dt - 0.5 g dt^2)` and `t2 = R0^T (v1 - v0 - g dt)`, which are `tmp` and `tmp2` in the code, the blocks at `preintegration.h:254-290` are

| row \ col | dp0 | dphi0 | dv0 | dbg | dba |
|---|---|---|---|---|---|
| r_p | -R0^T | [t]x R0^T | -R0^T dt | -J^p_{D\|bg} | -J^p_{D\|ba} |
| r_R | 0 | Jr^{-1}(r_R) R0^T | 0 | **+**Jl^{-1}(r_R) J^R_{D\|bg} | 0 |
| r_v | 0 | [t2]x R0^T | -R0^T | -J^v_{D\|bg} | -J^v_{D\|ba} |

and with respect to state 1, `r_p / dp1 = R0^T`, `r_R / dphi1 = -Jr^{-1}(r_R) R0^T`, `r_v / dv1 = R0^T`, with no bias columns at all. The bias of the second state is constrained only by the random-walk residual.

Two traps here. First, the rotation row uses the **right** Jacobian inverse for the pose columns and the **left** Jacobian inverse for the gyro-bias column, because the pose perturbation ends up on the right of the product after the adjoint move `R^{-1} Exp(e) R = Exp(R^{-1} e)`, whereas the bias correction sits inside an exponential on the extreme left. The code uses `Sophus::rightJacobianInvSO3` at `:255` and `Sophus::leftJacobianInvSO3` at `:287`. Second, the gyro-bias block of the rotation row is **positive**, unlike every other bias block, because the correction is composed with the delta rather than subtracted from it.

`ImuBlock::linearizeImu` (`include/basalt/linearization/imu_block.hpp:26`) whitens the nine inertial rows by `get_sqrt_cov_inv()` and appends six bias random-walk rows with weights `sigma_b^{-1}/sqrt(dt)` (`:76`, `:91`), giving a 15-row residual and a 15x30 Jacobian. It evaluates residual and Jacobians at `getStateLin()` and re-evaluates the residual value alone at `getState()` when either endpoint is FEJ-frozen (`:50-55`).

## The initial marginalisation prior anchors only the nullspace

`SqrtKeypointVioEstimator` constructor, `src/vi_estimator/sqrt_keypoint_vio.cpp:87-111`. `marg_data.H` is a 15x15 diagonal but only ten entries are set.

- Indices 0-2, global position, weight `vio_init_pose_weight` (default 1e8).
- Index 5 only, yaw about world Z, same weight.
- Indices 3-4, roll and pitch, and 6-8, velocity, are deliberately left at zero, because gravity and the accelerometer make them observable and an artificial prior would bias them.
- Indices 9-11 and 12-14, the two biases, get small weights to stop them jumping before enough excitation accumulates.

Square roots of the weights are stored when `vio_sqrt_marg` is set.

### Known defect, weights are swapped relative to their names

Per the `applyInc` ordering, indices 9-11 are the **gyroscope** bias and 12-14 the **accelerometer** bias. The constructor applies `vio_init_ba_weight` to 9-11 and `vio_init_bg_weight` to 12-14, so the two are exchanged. With the defaults of `vio_init_ba_weight = 1e1` and `vio_init_bg_weight = 1e2` (`src/utils/vio_config.cpp:85-86`), the gyroscope bias receives 1e1 and the accelerometer bias 1e2, the opposite of what the names imply. Inherited from upstream Basalt, left standing deliberately so that comparison against historical runs stays valid. Anyone tuning these must set them against the block they actually reach.

## Other corrections made on 2026-08-31

- `doc/VIO.md` §2.3.7 claimed the VO estimator resets the LM damping each frame "unlike the VIO estimator, which carries it across frames". Both reset it. VIO does so at `src/vi_estimator/sqrt_keypoint_vio.cpp:1200`.
- The problem-variable list described `frame_poses` and the pose part of `frame_states` as camera poses. They are body, that is IMU, poses `T_w_i`; the extrinsic `T_i_c` is composed only where a residual needs a camera frame.

## Stale line numbers in the older doc sections

The `sqrt_keypoint_vio.cpp` anchors in `doc/VIO.md` §4 through §9 predate the log-verbosity and deadlock-fix commits and run roughly 70 to 80 lines low. Current anchors are `ProcessFrame:201`, `measure:372`, `marginalize:674`, `optimize:1149`, `optimize_and_marg:1592`. The anchors written into §3 on 2026-08-31 are current; the earlier sections have not been swept.

## Optional roll and pitch rows of the initial prior (added 2026-11-11)

`vio_gravity_prior_std_deg` above 0 sets the diagonal entries 3 and 4 of the initial prior, $1/\sigma_g$ in square root form and $1/\sigma_g^2$ in information form, beside the yaw entry on index 5 (`sqrt_keypoint_vio.cpp`, constructor). `PoseState::incPose` perturbs the attitude on the left in the world frame (`thirdparty/basalt-headers/include/basalt/imu/imu_types.h:98-101`), so indices 3, 4 and 5 are world roll, pitch and yaw. Unlike the position and yaw rows these are physical information, not a gauge choice, since roll and pitch are observable. In the gravity and scale period the padding that inserts the block's three columns also writes $-1/\sigma_g$ at (3, 0) and (4, 1), so the rows read $\sqrt{w_g}(\delta\boldsymbol{\varphi}_{0,xy} - \boldsymbol{\delta}_\theta)$ with $\bar{\mathbf{B}} = [\mathbf{e}_x, \mathbf{e}_y]$ at construction, gauge invariant along both tilt directions of the block (`../plans/gravity_and_bias_step_priors.md` §2.2). `SetInitialPriorOrder` copies them into $M_\text{half}$ and $M_\text{visual}$.


## What a prior and a marginalisation prior are, and how the window and the mapper consume them (added 2026-11-19)

Written for `../plans/local_mapper_correction_ingestion.md` §2.6, which compares Basalt's prior with ICE-BA's. Anchors are as of 2026-11-19.

### A prior in general

Every term of the window's cost is a squared whitened residual $\frac{1}{2}|\mathbf{r}(\mathbf{x})|^2$. A prior is such a term whose information comes from outside the current measurements, from a belief held before them or from measurements that are no longer in the problem. For a Gaussian belief $\mathcal{N}(\boldsymbol{\mu}, \boldsymbol{\Sigma})$ on a state, the residual is $\mathbf{r} = \boldsymbol{\Sigma}^{-\frac{1}{2}}(\mathbf{x} \boxminus \boldsymbol{\mu})$, and its cost is $\frac{1}{2}(\mathbf{x} \boxminus \boldsymbol{\mu})^\top \boldsymbol{\Sigma}^{-1} (\mathbf{x} \boxminus \boldsymbol{\mu})$. A prior is thus fixed by two objects, a point it is stated about and an information matrix, together with the coordinates in which the difference from that point is taken.

Basalt's initial prior is of this kind. It is a diagonal square root information on the first state's offset from its initial value, with weights on position, yaw and the biases (section "The initial marginalisation prior anchors only the nullspace" above). It is stored in the same container as the marginalisation prior, `marg_data`, and consumed by the same code, so after the first marginalisation the two are one object.

### The marginalisation prior, and how it is built

At a marginalisation the window's cost is linearised at the current estimate. With $\boldsymbol{\delta}$ the stacked tangent offsets of all window states from their linearisation points, every residual gives $\mathbf{r}(\boldsymbol{\delta}) \approx \mathbf{r} + \mathbf{J} \boldsymbol{\delta}$, and their sum is

$$E(\boldsymbol{\delta}) = \frac{1}{2}|\mathbf{J} \boldsymbol{\delta} + \mathbf{r}|^2 = \frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H} \boldsymbol{\delta} + \mathbf{b}^\top \boldsymbol{\delta} + \text{const},$$

with $\mathbf{H} = \mathbf{J}^\top \mathbf{J}$ and $\mathbf{b} = \mathbf{J}^\top \mathbf{r}$. Partition the states into those kept, $\alpha$, and those leaving, $\beta$. Minimising over $\boldsymbol{\delta}_\beta$, which by the integral of a Gaussian over a block is the same as marginalising it, gives the Schur complement

$$\mathbf{H}^\ast = \mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{H}_{\beta\alpha},$$

$$\mathbf{b}^\ast = \mathbf{b}_\alpha - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{b}_\beta,$$

and the marginalisation prior is the quadratic

$$E_m(\boldsymbol{\delta}_\alpha) = \frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}^\ast \boldsymbol{\delta}_\alpha + \mathbf{b}^{\ast\top} \boldsymbol{\delta}_\alpha.$$

Basalt does not form $\mathbf{H}^\ast$ by default. `marginalize` takes the square root exit `get_dense_Q2Jp_Q2r` (`src/vi_estimator/sqrt_keypoint_vio.cpp:1341-1342`), and `MargHelper::marginalizeHelperSqrtToSqrt` (`src/vi_estimator/marg_helper.cpp:258`) permutes the leaving columns to the front, triangularises by Householder QR and keeps the rows below the leaving block. Those rows $\mathbf{J}_m$ and their residual $\mathbf{r}_m$ satisfy $\mathbf{J}_m^\top \mathbf{J}_m = \mathbf{H}^\ast$ and $\mathbf{J}_m^\top \mathbf{r}_m = \mathbf{b}^\ast$, and are stored in `marg_data.H` and `marg_data.b` with `is_sqrt` set (section "MargLinData::H is not a Hessian" of `linearisation.md`).

Concretely, `marginalizeHelperSqrtToSqrt` (`src/vi_estimator/marg_helper.cpp:258-354`) stacks the previous prior's own rows together with every currently linearised factor touching the leaving variables, permutes the columns so `idx_to_marg` comes first and `idx_to_keep` second, and runs a column by column, rank revealing Householder QR over the whole block, skipping a column's reflection when its pivot magnitude falls below $\sqrt{\epsilon}$ rather than failing, which is how a genuinely unobservable, gauge direction is carried through as a zero row instead of a division by zero (`:312-318`). `marg_rank` is the rank accumulated after the first `idx_to_marg.size()` columns and `total_rank` the rank after all columns, so `keep_valid_rows` $=$ `total_rank` $-$ `marg_rank`, floored at one row, is the rank contributed by the keep columns alone (`:343-344`). $\mathbf{J}_m$ is then the block of the transformed Jacobian at rows `marg_rank` to `marg_rank + keep_valid_rows` and columns `idx_to_marg.size()` onward (`:347`), and $\mathbf{r}_m$ the matching segment of the transformed residual (`:348`), both dense across every column of `idx_to_keep` regardless of which factor originally supplied which entry, since the QR mixes them. These two objects become `marg_data.H` and `marg_data.b` verbatim, with `marg_data.order` holding only `idx_to_keep`. In the worked example below, $\mathbf{J}_m$ and $\mathbf{r}_m$ are nothing other than whatever `marg_data.H` and `marg_data.b` held at the moment $K_1$ and $K_2$ last absorbed a marginalisation, 12 rows over their 12 columns if that keep block was full rank, re-centred since then by the `computeDelta` and `marg_data.b -= marg_data.H * delta` of the "State Cleanup & Prior Re-centering" step of `doc/Marginalisation.md` §3.3.2 but not re-linearised.

### The Householder sweep itself

The rank revealing QR of `marginalizeHelperSqrtToSqrt` is a sequence of elementary reflections, worth spelling out once since the file above only cites it by code line. A Householder reflection on a vector $\mathbf{v} \in \mathbb{R}^n$ is an orthogonal matrix $\mathbf{H} = \mathbf{I} - \tau\, \mathbf{w}\mathbf{w}^\top$, with $\mathbf{w}$ a unit vector and $\tau$ a scalar, chosen so that $\mathbf{H} \mathbf{v} = \beta\, \mathbf{e}_1$ with $\beta = -\,\text{sign}(v_1)\, |\mathbf{v}|$, the sign choice avoiding cancellation. `Q2Jp.col(k).tail(remainingRows).makeHouseholderInPlace(hCoeff, beta)` (`marg_helper.cpp:309`) computes this $\tau$ as `hCoeff` and this $\beta$, and overwrites the vector in place with the essential part of $\mathbf{w}$, since $\mathbf{H}$ itself is never formed, only applied through `applyHouseholderOnTheLeft`.

The sweep runs $k = 0, 1, \dots$ left to right across the permuted columns, with `total_rank` counting how many rows a reflection has already consumed. At each $k$ it takes the sub-column of column $k$ from row `total_rank` downward, not from row $k$, and reflects that sub-column onto its first entry. If the resulting $|\beta|$ clears `rank_threshold`, the reflection is accepted, written onto the diagonal at `(total_rank, k)`, applied to every column right of $k$ and to the residual, and `total_rank` advances by one. If $|\beta|$ does not clear the threshold, the diagonal entry is written zero and `total_rank` does not advance, so a skipped column costs no row, the next column simply gets to use the row a dependent column would otherwise have consumed. Because every reflection only ever touches columns to its right, once all `idx_to_marg` columns are done nothing later can write back into them, which is why the triangular result is upper triangular at all and why $\mathbf{J}_m$, read off the kept columns afterward, depends only on `idx_to_keep`.

A minimal instance makes the row bookkeeping concrete. One leaving column, one kept column, three residual rows,

$$\mathbf{J} = \begin{bmatrix} a_1 & b_1 \\ a_2 & b_2 \\ a_3 & b_3 \end{bmatrix}, \qquad \mathbf{r} = \begin{bmatrix} r_1 \\ r_2 \\ r_3 \end{bmatrix}.$$

At $k=0$, `total_rank` $=0$, the whole first column reflects onto $[\beta,0,0]^\top$ and the same reflection carries the second column and $\mathbf{r}$ along,

$$\mathbf{J}' = \begin{bmatrix} \beta & b_1' \\ 0 & b_2' \\ 0 & b_3' \end{bmatrix}, \qquad \mathbf{r}' = \begin{bmatrix} r_1' \\ r_2' \\ r_3' \end{bmatrix},$$

`total_rank` becomes $1$ and, this being the last leaving column, `marg_rank` $=1$. At $k=1$, only the tail $[b_2', b_3']^\top$ is reflected, onto $[\gamma,0]^\top$, with $\mathbf{r}$'s tail carried along,

$$\mathbf{J}'' = \begin{bmatrix} \beta & b_1' \\ 0 & \gamma \\ 0 & 0 \end{bmatrix}, \qquad \mathbf{r}'' = \begin{bmatrix} r_1' \\ s \\ t \end{bmatrix},$$

`total_rank` becomes $2$. So `keep_valid_rows` $=1$, and `Q2Jp.block(marg_rank, marg_size, keep_valid_rows, keep_size)` pulls row $1$, column $1$, giving $\mathbf{J}_m = [\gamma]$, while `Q2r.segment(marg_rank, keep_valid_rows)` gives $\mathbf{r}_m = [s]$. Row $0$, $[\beta\ b_1' \mid r_1']$, is exactly the row spent pinning down the leaving variable and is discarded with it as `elim_J`, `elim_r`. Row $2$, $[0\ 0 \mid t]$, is zero on every variable and contributes only the fixed constant $\frac{1}{2}t^2$, the discarded remainder beyond `total_rank`. The real sub-problem has one pair of rows per visual observation, nine per IMU factor, six per bias random walk and so on, and six or fifteen columns per variable, but the sweep is this identical loop over every column of the whole stack.

### The exact quadratic form the QR step is computing

`marg_rank` and `keep_valid_rows` have a direct reading as a Gauss-Newton elimination, not only as QR bookkeeping. Write $\mathbf{J} = [\mathbf{J}_\beta \ \mathbf{J}_\alpha]$ for the stacked Jacobian before the QR step, $\beta$ the leaving columns and $\alpha$ the kept ones, and $\mathbf{r}$ for the matching stacked residual at the current linearisation point, so the sub-problem's cost is $E(\boldsymbol{\delta}_\beta, \boldsymbol{\delta}_\alpha) = \frac{1}{2}|\mathbf{J}_\beta \boldsymbol{\delta}_\beta + \mathbf{J}_\alpha \boldsymbol{\delta}_\alpha + \mathbf{r}|^2$. The Householder sequence is an orthogonal $\mathbf{Q}$ with

$$\mathbf{Q}^\top \mathbf{J} = \begin{bmatrix} \mathbf{R}_{\beta\beta} & \mathbf{R}_{\beta\alpha} \\ \mathbf{0} & \mathbf{J}_m \\ \mathbf{0} & \mathbf{0} \end{bmatrix},$$

$$\mathbf{Q}^\top \mathbf{r} = \begin{bmatrix} \mathbf{r}_\beta' \\ \mathbf{r}_m \\ \mathbf{r}_0 \end{bmatrix},$$

with the row partition exactly `marg_rank`, `keep_valid_rows`, and the discarded remainder, and $\mathbf{R}_{\beta\beta}$ square upper triangular and invertible on its rank. Because $\mathbf{Q}$ is orthogonal it preserves the norm, so

$$E(\boldsymbol{\delta}_\beta, \boldsymbol{\delta}_\alpha) = \frac{1}{2}|\mathbf{R}_{\beta\beta} \boldsymbol{\delta}_\beta + \mathbf{R}_{\beta\alpha} \boldsymbol{\delta}_\alpha + \mathbf{r}_\beta'|^2 + \frac{1}{2}|\mathbf{J}_m \boldsymbol{\delta}_\alpha + \mathbf{r}_m|^2 + \frac{1}{2}|\mathbf{r}_0|^2.$$

For any $\boldsymbol{\delta}_\alpha$ the first term is driven exactly to zero by $\boldsymbol{\delta}_\beta = -\mathbf{R}_{\beta\beta}^{-1}(\mathbf{R}_{\beta\alpha} \boldsymbol{\delta}_\alpha + \mathbf{r}_\beta')$, which is the elimination of $\boldsymbol{\delta}_\beta$, so

$$\min_{\boldsymbol{\delta}_\beta} E(\boldsymbol{\delta}_\beta, \boldsymbol{\delta}_\alpha) = \frac{1}{2}|\mathbf{J}_m \boldsymbol{\delta}_\alpha + \mathbf{r}_m|^2 + \frac{1}{2}|\mathbf{r}_0|^2.$$

This identifies $\mathbf{r}_m$ as the unique vector making $\frac{1}{2}|\mathbf{J}_m \boldsymbol{\delta}_\alpha + \mathbf{r}_m|^2$ equal the exact minimum of the sub-problem's cost over the eliminated variables, for every $\boldsymbol{\delta}_\alpha$, and at $\boldsymbol{\delta}_\alpha = \mathbf{0}$ gives $\frac{1}{2}|\mathbf{r}_m|^2 = \min_{\boldsymbol{\delta}_\beta} E(\boldsymbol{\delta}_\beta, \mathbf{0})$, the lowest cost reachable by moving only the states about to leave with the kept states frozen at their current estimate. $\mathbf{r}_m$ still satisfies $\mathbf{J}_m^\top \mathbf{J}_m = \mathbf{H}^\ast$ and $\mathbf{J}_m^\top \mathbf{r}_m = \mathbf{b}^\ast$, matching Appendix A of `doc/Marginalisation.md`, but Appendix A reaches a valid $\mathbf{r}_{\text{marg},0}$ by eigendecomposing $\mathbf{H}^\ast$ and applying the Moore-Penrose pseudoinverse, one representative of the square root factorisation's equivalence class, whereas the QR step produces this particular representative directly as a sub-block of the rotated residual, without ever forming $\mathbf{H}^\ast$ or $\mathbf{b}^\ast$ explicitly. The two are not generally numerically identical, since a square root factorisation is only fixed up to left multiplication of $\mathbf{J}_m$ and $\mathbf{r}_m$ by a common orthogonal matrix, which leaves $\mathbf{H}^\ast$, $\mathbf{b}^\ast$ and $|\mathbf{r}_m|$ unchanged.

The prior is then

$$E_m = \frac{1}{2}|\mathbf{J}_m \boldsymbol{\delta}_\alpha + \mathbf{r}_m|^2,$$

equal to the quadratic above up to the constant $\frac{1}{2}|\mathbf{r}_m|^2$. $\mathbf{r}_m$ is the marginalisation residual, and the cost is half its squared norm. Its variables are the poses of the remaining keyframes and the full state of the newest frame, laid out at the top left of the window's order map (`marg_order_new`, `:1671-1684`, and section "The prior needs no index remapping" of `linearisation.md`).

### The coordinates and the first estimates

$\boldsymbol{\delta}_\alpha$ is absolute. For each state in the prior, `PoseStateWithLin` and `PoseVelBiasStateWithLin` keep a frozen linearisation point and the accumulated increment since then, `getPoseLin` and `getDelta` (`include/basalt/utils/imu_types.h:205-268`), and `BundleAdjustmentBase::computeDelta` stacks those increments in the prior's order (`src/vi_estimator/ba_base.cpp:322-344`). An increment $\boldsymbol{\xi}$ of the solver changes the coordinate exactly to $\boldsymbol{\delta} + \boldsymbol{\xi}$. Every other factor on a state of the prior evaluates its Jacobians at the frozen point and its residual at the current value, so that all of them share the prior's null space, the gauge at the frozen point. That is the first estimate Jacobian convention (`vio_opt.md` §11.6).

### Consumption in the VIO window

`optimize` adds the prior in one of two equivalent forms. In the square root assembly `LinearizationAbsQR::get_dense_Q2Jp_Q2r_marg_prior` (`src/linearization/linearization_abs_qr.cpp:702-723`) writes the rows $\mathbf{J}_m$ and the residual $\mathbf{J}_m \boldsymbol{\delta} + \mathbf{r}_m$ below the other rows. In the dense assembly `add_dense_H_b_marg_prior` (`:735-744`) calls `BundleAdjustmentBase::linearizeMargPrior` (`src/vi_estimator/ba_base.cpp:419-490`), whose comment derives it. With the solver's increment $\boldsymbol{\xi}$,

$$P(\boldsymbol{\xi}) = \frac{1}{2}|\mathbf{J}_m(\boldsymbol{\delta} + \boldsymbol{\xi}) + \mathbf{r}_m|^2 = \frac{1}{2}|\mathbf{J}_m \boldsymbol{\xi} + (\mathbf{J}_m \boldsymbol{\delta} + \mathbf{r}_m)|^2,$$

a residual $\mathbf{J}_m \boldsymbol{\delta} + \mathbf{r}_m$ with Jacobian $\mathbf{J}_m$. Its contributions to the normal equations $\mathbf{H} \boldsymbol{\xi} + \mathbf{b} = \mathbf{0}$ are

$$\mathbf{H} \mathrel{+}= \mathbf{J}_m^\top \mathbf{J}_m,$$

$$\mathbf{b} \mathrel{+}= \mathbf{J}_m^\top (\mathbf{J}_m \boldsymbol{\delta} + \mathbf{r}_m),$$

added to the top left block, and the cost reported is $\frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H}_m \boldsymbol{\delta} + \boldsymbol{\delta}^\top \mathbf{b}_m$, without the constant, which is why it can be negative. The Jacobian is fixed and the prior is never relinearised, because it is linear in its coordinate. Its nonlinearity was spent when the leaving factors were linearised at the marginalisation.

The prior is a function of the absolute offsets of the window's own states, so writing a state outside the solve changes $\boldsymbol{\delta}$ and with it the prior's residual. This is why the mapper's overwrite of `frame_poses` conflicts with it (`vio_localmapper_correction_loop.md`).

### Consumption in the local mapper

The mapper holds no marginalisation prior of its own. It receives the window's prior in the packet `MargData` (`include/basalt/utils/imu_types.h:477-491`), built before the leaving states are removed, with `abs_H` $= \mathbf{J}^\top \mathbf{J}$ and `abs_b` $= \mathbf{J}^\top \mathbf{r}$ of the stacked rows of that marginalisation and the window's states at that moment (`src/vi_estimator/sqrt_keypoint_vio.cpp:1389-1420`). `NfrMapper::extractNonlinearFactors` (`src/vi_estimator/nfr_mapper.cpp:157-252`) converts it into nonlinear factors by non linear factor recovery.

1. The covariance is $\boldsymbol{\Sigma} = $ `abs_H`$^{-1}$, by a full pivot QR that rejects a rank deficient packet (`:161-164`).
2. For the leaving keyframe $m$, a roll and pitch factor with measurement its current rotation and information the inverse of the roll and pitch block of $\mathbf{J}_g \boldsymbol{\Sigma} \mathbf{J}_g^\top$, $\mathbf{J}_g$ the Jacobian of position, yaw and roll pitch (`:177-212`).
3. For every other keyframe $o$ of `kfs_all`, a `RelPoseFactor` with measurement $\mathbf{T}_m^{-1} \mathbf{T}_o$ at the packet's estimates and information $(\mathbf{J}_{mo} \boldsymbol{\Sigma} \mathbf{J}_{mo}^\top)^{-1}$, $\mathbf{J}_{mo}$ the Jacobian of `relPoseError` on both poses (`:215-249`).

`abs_b` is not used. Each factor's residual is zero at the packet's estimate, so the recovery assumes that estimate is the prior's minimum. The factors are then ordinary nonlinear residuals of the mapper, evaluated at its current poses with Jacobians recomputed at every iteration (`MapperLinearizeAbsReduce`, `include/basalt/vi_estimator/nfr_mapper.h:111-162`, and `NfrMapper::computeRelPose`, `src/vi_estimator/nfr_mapper.cpp:455-466`),

$$E_\text{rel} = \frac{1}{2}\sum \mathbf{r}^\top \boldsymbol{\Lambda} \mathbf{r},$$

$$\mathbf{H} \mathrel{+}= \mathbf{J}^\top \boldsymbol{\Lambda} \mathbf{J},$$

$$\mathbf{b} \mathrel{+}= \mathbf{J}^\top \boldsymbol{\Lambda} \mathbf{r}.$$

Because the relative factors are invariant to translation and yaw, they can be relinearised anywhere without adding gauge information, and the roll pitch factor is invariant to yaw and translation by construction. When the mapper culls a keyframe it erases the factors that name it (`src/vi_estimator/local_mapper.cpp:1284-1296`, and `LocalMapper::PruneFactorsWithUnknownKeyframes`, `:406-430`), which discards their information rather than marginalising it.

### A worked example, the complete window objective assembled

The sections above describe each residual family in isolation. This section assembles a single concrete window to show every family stacked into one $\mathbf{H}$ and $\mathbf{b}$ at one call of `optimize` (`src/vi_estimator/sqrt_keypoint_vio.cpp:1149`), including the stationary pseudo-measurement, a prior family not covered above because it was added after this file's other sections, in the commit "Stationary Warm Start of the VIO Estimator". `doc/VIO.md` §3.1.4 states the window objective in the abstract without this term, since that section predates the stationary residual.

Take a window with `vio_max_kfs` $= 7$ and `vio_max_states` $= 3$ (defaults, `src/utils/vio_config.cpp:61-62`) part way through tracking, with two pose only keyframes $K_1, K_2$ in `frame_poses` and three full states $S_1, S_2, S_3$ in `frame_states`, $S_3$ the newest frame. The `AbsOrderMap` lays these out at

| variable | start column | block size |
| --- | --- | --- |
| $K_1$ | 0 | 6 |
| $K_2$ | 6 | 6 |
| $S_1$ | 12 | 15 |
| $S_2$ | 27 | 15 |
| $S_3$ | 42 | 15 |

giving `aom.total_size` $= 57$. A landmark $l$, hosted in $K_1$ and also observed in $K_2, S_1, S_2, S_3$, contributes three more columns during linearisation that the Schur complement or Householder QR of §2.2.8 and §3.3.1 remove before the reduced system below is formed, so they are not among the 57.

Five residual families contribute rows at this call.

1. The marginalisation prior carried over from earlier steps, $\mathbf{J}_m \boldsymbol{\delta}_{[0,12)} + \mathbf{r}_m$, touching only $K_1$ and $K_2$ because every earlier state has already been eliminated down to this pair. Any gravity prior row from the construction of the very first prior (section "Optional roll and pitch rows" above) is already folded into $\mathbf{J}_m$ at this point, since both are stored in the same `marg_data` container and the gravity rows exist only at construction, not as a separate term at every later step. Say $\mathbf{J}_m$ has 12 rows, full rank on its 12 columns.
2. Five reprojection residuals of $l$, one in each of $K_1, K_2, S_1, S_2, S_3$, $2$ rows each, $10$ rows total, each touching the landmark's 3 columns, later eliminated, and the pose columns of its own frame, the leading 6 of 15 for a full state.
3. Two IMU preintegration factors, $S_1 \to S_2$ and $S_2 \to S_3$, $9$ rows each, $18$ rows total, each touching all 30 columns of its two states (section "IMU residual" above).
4. The matching bias random walk rows of those same two factors, $6$ rows each, $12$ rows total, each touching only the two 6-wide bias columns, $[21,27)$ and $[36,42)$ for the first factor, $[36,42)$ and $[51,57)$ for the second.
5. The stationary rows of `StationaryRows` (`src/vi_estimator/sqrt_keypoint_vio.cpp:2990`), active when $S_1$ and $S_2$ are both flagged in `mvpStationaryStates`, that is both believed to be at rest. A zero velocity row per stationary state, $3$ rows at columns $[18,21)$ for $S_1$ and $[33,36)$ for $S_2$, weight `vio_zero_velocity_weight`. A zero relative motion row pair between the consecutive stationary states, $3$ position rows on $[12,18)$ and $[27,33)$, weight $1/$`vio_zero_motion_pos_std`, and $3$ rotation rows on the same columns, weight $180/(\pi \cdot$ `vio_zero_motion_rot_std_deg`$)$. $12$ rows total. Both standard deviations default to zero (`src/utils/vio_config.cpp:90-91`), which is why this family is usually silent, only `vio_zero_velocity_weight` is read unconditionally.

Stacking all five families gives $12 + 10 + 18 + 12 + 12 = 64$ rows over the same 57 columns, and the total cost is their sum of squared norms,

$$E(\boldsymbol{\xi}) = \frac{1}{2}|\mathbf{J}_m \boldsymbol{\xi}_{[0,12)} + \mathbf{r}_m|^2 + \frac{1}{2}\sum_{f \in \{K_1,K_2,S_1,S_2,S_3\}} \mathbf{r}_{lf}^\top \boldsymbol{\Sigma}_\text{vis}^{-1} \mathbf{r}_{lf}$$

$$+ \frac{1}{2}\sum_{(a,b) \in \{(S_1,S_2),(S_2,S_3)\}} \mathbf{r}_{ab}^\top \boldsymbol{\Sigma}_{ab}^{-1} \mathbf{r}_{ab} + \frac{1}{2}\sum_{(a,b)} \left(\mathbf{r}_{ab}^{b_g\top} \mathbf{W}_{b_g} \mathbf{r}_{ab}^{b_g} + \mathbf{r}_{ab}^{b_a\top} \mathbf{W}_{b_a} \mathbf{r}_{ab}^{b_a}\right) + \frac{1}{2}|\mathbf{r}_\text{stat}|^2,$$

one term per family of item 1 to 5 above, matching the general $E(\mathbf{s})$ of `doc/VIO.md` §3.1.4 with the stationary term appended and the landmark sum of §3.1.4 specialised to the single landmark $l$. Each factor's own Jacobian, read off the per family sections above and `StationaryRows` for the last, occupies exactly the columns named in its item, zero elsewhere, which is the block occupancy that gives $\mathbf{H} = \sum_f \mathbf{J}_f^\top \mathbf{W}_f \mathbf{J}_f$ its sparsity. $K_1$ and $K_2$ are coupled only through $\mathbf{J}_m$ and through $l$'s reprojection rows once the landmark column is eliminated, since no other family reaches a pose only keyframe. $S_1$ and $S_2$ are coupled through the IMU factor, the bias random walk and, in this window, the stationary rows as well, so their mutual block of $\mathbf{H}$ receives four separate contributions, while $S_2$ and $S_3$ receive only the two inertial ones because the stationary flag does not extend to $S_3$. Forming this $\mathbf{H}$ and $\mathbf{b}$, damping the diagonal by $\lambda$ and solving $(\mathbf{H} + \lambda\,\text{diag}(\mathbf{H}))\boldsymbol{\xi} = \mathbf{b}$ by LDLT, with the landmark already eliminated and recovered afterwards by back substitution, is exactly the Gauss-Newton step of `doc/VIO.md` §3.1.5 and §2.2.7 to §2.2.9, carried out for this specific set of factors.

### The four forms side by side

| | Basalt window | Basalt mapper | ICE-BA local BA | ICE-BA global BA |
| --- | --- | --- | --- | --- |
| Object | `marg_data`, square root rows | `RelPoseFactor`, `RollPitchFactor` | `m_Zp` joint, `m_ZpLF` motion | `m_Zps` pose priors, `m_ZpLM` motion |
| Coordinates | absolute $\boldsymbol{\delta}$ from frozen points | relative pose, roll pitch | relative pose, tilt, body frame motion | relative pose, tilt, body frame motion |
| Linear term kept | yes, $\mathbf{r}_m$ | no, `abs_b` dropped | yes, $\mathbf{b}$ | yes, $\mathbf{b}$ |
| Jacobians | identity on $\boldsymbol{\delta}$, other factors at first estimates | at current poses | at current cameras | at current poses |
| Carried across marginalisations | yes, accumulates for the run | factors accumulate, one set per packet, until their keyframes are culled | motion marginal only, rebuilt each time | appended per push |
| Removal of a state | Schur complement into the prior | factors erased | erased rows, conditioning | erased rows, conditioning |
| Effect of writing a state | moves $\boldsymbol{\delta}$, the prior pulls back | none beyond the change of shape | none, keyframes are constants | none beyond the change of shape |

The ICE-BA columns are read from its code in `../plans/local_mapper_correction_ingestion.md` §2.6.
