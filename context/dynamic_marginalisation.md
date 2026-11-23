# Marginalisation Priors under a Changing Scale and Gravity, Literature and Mathematics

Created 2026-11-06. This file collects the literature on keeping marginalisation priors consistent when a global variable such as metric scale or the gravity direction changes, with every derivation written out step by step. It was written to explain why the gravity and scale mode of `plans/gravity_scale_optimisation.md` loses its inertial information at take off (§3.2.3 there, record of runs of 2026-11-06), and to ground the choice of a remedy. The notation and the identities (I1) to (I10) of that plan are reused, and the identities this file adds are labelled (M1) onwards. All mathematics follows the conventions of `/ws/skills/latex-formatting.md`.

Sources were read as follows, and a claim attributed to a paper is limited to what was read. VI-DSO [1] §III-E and §III-F, DM-VIO [2] §III-B to §III-F, the non-linear factor recovery section of the Basalt mapping paper [3] (§V-B and §V-C, eq. 27 to 35) and the definitions and measurement model of on manifold preintegration [4] (eq. 33 and 38) were read in full through their ar5iv renderings on 2026-11-06. For generic linear constraints [5], information sparsification in VIO [6], the scale state EKFs of Nützi et al. [7] and ORB-SLAM3 [8], only abstracts and search snippets were read, and the text says so where they are used.

## 1. The current problem

This section states what the gravity and scale mode does, what the replays of 2026-11-06 showed, and the three mechanisms by which its marginalisation loses the inertial information. The mechanisms are derived here because the literature survey of §3 and the proposals of §4 are judged against them.

### 1.1 What the estimator does

During the initialisation period the estimator of `plans/gravity_scale_optimisation.md` holds a three dimensional block $\boldsymbol{\delta}_{gs} = [\delta\theta_1, \delta\theta_2, \delta\sigma]^\top$, the tilt of the gravity direction and the log scale, jointly with every pose, velocity, bias and landmark. Its inertial residual carries the scale and the gravity vector explicitly (plan §2.3.1),

$$\mathbf{r}_p = \mathbf{R}_0^\top \Big(s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t) - \frac{1}{2}\mathbf{g}_D \Delta t^2\Big) - \hat{\Delta\mathbf{p}},$$

$$\mathbf{r}_v = \mathbf{R}_0^\top \big(s(\mathbf{v}_1 - \mathbf{v}_0) - \mathbf{g}_D \Delta t\big) - \hat{\Delta\mathbf{v}},$$

and the visual residual carries the scale only through the extrinsic lever arm (plan §2.2.4). At the end of every solve the estimated correction is applied to all states and priors and the block is reset to the identity (plan §2.7). Old frames are marginalised into a square root prior with first estimate Jacobians, and, following VI-DSO [1], three priors $M_\text{curr}$, $M_\text{half}$ and $M_\text{visual}$ are kept and swapped whenever the scale leaves an interval of half width $\ln d_i$ with $d_\text{min} = \sqrt{1.1}$ (plan §2.6). The survey of §3.2 describes that scheme in full.

### 1.2 What the replays showed

Replays of the SITL bag `/ws/data/telemetry_40` and live `sitl2` runs on 2026-11-06 (plan §4.3, record of runs) showed the following, with the scale ratio the length of the estimated displacement over 2 s divided by that of ground truth and the tilt error the angle between the body frame gravity directions of estimate and ground truth.

| Run | Tilt error after take off | Scale ratio after the period | Outcome |
|---|---|---|---|
| Mode off | 2.4° to 6° | 2.1 to 2.7 | stable, scale too large |
| Mode on, 10 s period | 26° to 27° | 0.86 to 1.08 | scale corrected, tilt diverged |
| Mode on, 70 s period, strong gyroscope bias prior | 74° to 113° | 10 to 19 | diverged |

Note, 2026-11-06. The last row previously read "strong bias prior", meaning the accelerometer bias. That run set `vio_init_ba_weight` $= 10^4$, which the constructor writes to the gyroscope bias indices 9 to 11 (`context/linearisation.md`), so the accelerometer bias kept the default prior of standard deviation 0.1 m/s².

Two coupled failures appear in the per frame record. First, at take off single solves return steps of up to $\delta\sigma = -0.56$ and 42° of tilt, which cross the interval of the swap rule at almost every frame, so swaps fire almost every frame. Second, the horizontal accelerometer bias tracks the tilt error exactly, $|\mathbf{b}_{a,xy}| = |\mathbf{g}| \sin(\text{tilt error})$ to within about 0.1 m/s², and the bias drift began 0.3 s after the last of 24 swaps (`context/takeoff_bias_divergence.md`, the section added 2026-11-06). The question this file addresses is how to keep the inertial information through a change of scale and gravity, and how to keep the per frame steps small. §1.3 to §1.5 derive why the current scheme fails on both counts.

### 1.3 The linearisation error of an inertial factor when the scale moves

The frozen Jacobian becomes wrong when the variable it was evaluated at moves far, and for the inertial factor with a scale the error can be written exactly. Take the velocity row of the plan §2.3.1 and abbreviate $\mathbf{u} = \mathbf{v}_1 - \mathbf{v}_0$, so that

$$\mathbf{r}_v = \mathbf{R}_0^\top \big(s\,\mathbf{u} - \mathbf{g}_D \Delta t\big) - \hat{\Delta\mathbf{v}}.$$

Step 1. Let the linearisation point be $(\bar{s}, \bar{\mathbf{u}})$ and the current point $(\bar{s} + \Delta s, \bar{\mathbf{u}} + \Delta\mathbf{u})$, with the rotation held fixed to isolate the effect. Step 2. The product expands exactly as

$$(\bar{s} + \Delta s)(\bar{\mathbf{u}} + \Delta\mathbf{u}) = \bar{s}\,\bar{\mathbf{u}} + \bar{s}\,\Delta\mathbf{u} + \Delta s\,\bar{\mathbf{u}} + \Delta s\,\Delta\mathbf{u}.$$

Step 3. The linear model of the frozen factor contains the first three terms, its Jacobians being $\bar{s}\,\mathbf{R}_0^\top$ for $\mathbf{u}$ and $\mathbf{R}_0^\top \bar{\mathbf{u}}$ for $s$. Step 4. The error of the linear model is therefore the last term, multiplied by $\mathbf{R}_0^\top$,

$$\mathbf{r}_v - \mathbf{r}_v^\text{lin} = \Delta s\,\mathbf{R}_0^\top \Delta\mathbf{u}.$$

The error is bilinear, so it vanishes when either the scale or the velocity has stayed at its linearisation point and grows with their product. With the numbers of the take off replays, $\Delta s = 0.5$ and $|\Delta\mathbf{u}| = 1$ m/s give an error of 0.5 m/s on a preintegrated velocity whose standard deviation over 50 ms is about 0.018 m/s, so the frozen factor is wrong by some thirty standard deviations. This is the formal reason a prior linearised at one scale must not be kept unchanged once the scale has moved far, and the motivation of both dynamic [1] and delayed [2] marginalisation.

### 1.4 What a swap discards, and the ratchet observed in the replays

A swap $M_\text{curr} \leftarrow M_\text{half}$ discards every inertial factor that $M_\text{curr}$ held but $M_\text{half}$ did not, and $M_\text{half} \leftarrow M_\text{visual}$ discards every inertial factor $M_\text{half}$ held. The information that survives a swap is therefore the inertial factors marginalised since the last reset of $M_\text{half}$, plus the visual information.

The failure in the Basalt replays follows from this. Step 1. Let the per frame scale step at take off exceed the interval, $|\delta\sigma_k| > \ln d_\text{min} \approx 0.048$, for several consecutive frames, as the logs show ($-0.078$, $+0.062$, $-0.064$, $-0.084$ and so on, with one step of $-0.56$). Step 2. Each such step either changes side or leaves the interval, so $M_\text{half}$ is reset to $M_\text{visual}$ at every frame and holds at most the one inertial factor marginalised since. Step 3. Each boundary crossing then sets $M_\text{curr}$ to that nearly empty $M_\text{half}$, so $M_\text{curr}$ holds at most one or two inertial factors. Step 4. With almost no inertial information in the prior, the next solve is again poorly determined and returns another large step, which closes the loop. Dynamic marginalisation assumes that the scale moves slowly compared with $\ln d_\text{min}$ per marginalisation, and the take off transient violates that assumption.

### 1.5 Why the swap also removes the accelerometer bias anchor

The absolute accelerometer bias is anchored only by the initial prior on the first state, propagated through the chain of bias random walk factors (`context/takeoff_bias_divergence.md`, 2026-11-06). The chain can be shown to carry the anchor in the prior that holds it and to lose it in $M_\text{visual}$, with one dimension per bias. Write $w_0$ for the information of the initial prior on $b_0$, $w_1$ for that of the random walk factor on $b_0 - b_1$, and marginalise $b_0$.

Step 1. The cost and its information matrix over $(b_0, b_1)$ in a prior that holds the random walk factor are

$$F = \frac{1}{2}w_0 b_0^2 + \frac{1}{2}w_1 (b_0 - b_1)^2,$$

$$\mathbf{H} = \begin{bmatrix}w_0 + w_1 & -w_1\\ -w_1 & w_1\end{bmatrix}.$$

Step 2. By §3.1.1 with $\beta = b_0$ and $\alpha = b_1$,

$$H' = w_1 - \frac{w_1^2}{w_0 + w_1} = \frac{(w_0 w_1)}{(w_0 + w_1)}.$$

Step 3. The variance of the anchor on $b_1$ is the inverse,

$$\frac{1}{H'} = \frac{1}{w_0} + \frac{1}{w_1},$$

which is the variance of the initial prior plus that of one random walk step. The anchor survives and widens slowly, as it should.

Step 4. In $M_\text{visual}$ the random walk factor is absent, because it is an inertial factor, so the information matrix is

$$\mathbf{H} = \begin{bmatrix}w_0 & 0\\ 0 & 0\end{bmatrix},$$

and §3.1.1 gives $H' = 0 - 0 \cdot w_0^{-1} \cdot 0 = 0$. The initial prior is removed with the first state and leaves nothing on $b_1$. Every prior reset to $M_\text{visual}$ therefore holds no absolute information on the accelerometer bias, and the tilt that the bias can trade against (`context/takeoff_bias_divergence.md`) becomes free. This matches the replay, in which the bias drift began 0.3 s after the last of 24 swaps.

## 2. Identities used in this file

(M1) Minimisation of a quadratic in a block of its arguments. For a symmetric $\mathbf{H}$ with an invertible block $\mathbf{H}_{\beta\beta}$, partitioned over variables $\boldsymbol{\delta}_\alpha$ and $\boldsymbol{\delta}_\beta$, the minimum over $\boldsymbol{\delta}_\beta$ of $\frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H} \boldsymbol{\delta} + \mathbf{b}^\top \boldsymbol{\delta}$ is a quadratic in $\boldsymbol{\delta}_\alpha$ whose matrix is the Schur complement $\mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{H}_{\beta\alpha}$. It is derived in §3.1.1.

(M2) Gaussian marginal in information form. For a Gaussian density proportional to $\exp(-\frac{1}{2}\boldsymbol{\delta}^\top \mathbf{H} \boldsymbol{\delta} - \mathbf{b}^\top \boldsymbol{\delta})$, integrating out $\boldsymbol{\delta}_\beta$ leaves a Gaussian in $\boldsymbol{\delta}_\alpha$ with the same Schur complement as (M1). The marginal and the minimum of (M1) agree because the integral of $\exp$ of a quadratic over a block equals a constant times $\exp$ of its minimum over that block, the constant being $(2\pi)^{n_\beta/2}\text{det}(\mathbf{H}_{\beta\beta})^{-1/2}$, which does not depend on $\boldsymbol{\delta}_\alpha$.

(M3) Kullback–Leibler divergence between Gaussians. For densities $p = \mathcal{N}(\boldsymbol{\mu}_o, \boldsymbol{\Sigma}_o)$ and $p_a = \mathcal{N}(\boldsymbol{\mu}_a, \mathbf{H}_a^{-1})$ of dimension $d$,
$$D_\text{KL}(p \,|\, p_a) = \frac{1}{2}\Big(\text{tr}(\mathbf{H}_a \boldsymbol{\Sigma}_o) - \ln\,\text{det}(\mathbf{H}_a \boldsymbol{\Sigma}_o) + (\boldsymbol{\mu}_a - \boldsymbol{\mu}_o)^\top \mathbf{H}_a (\boldsymbol{\mu}_a - \boldsymbol{\mu}_o) - d\Big),$$
which is eq. 27 of [3], written there with $\langle \mathbf{H}_a, \boldsymbol{\Sigma}_o \rangle$ for the trace.

(M4) Derivatives of the trace and the log determinant. For a symmetric positive definite $\mathbf{X}$ and a symmetric $\mathbf{S}$, the derivative of $\text{tr}(\mathbf{X} \mathbf{S})$ with respect to $\mathbf{X}$ is $\mathbf{S}$, and that of $\ln\,\text{det}\mathbf{X}$ is $\mathbf{X}^{-1}$. These are standard results of matrix calculus, collected for instance in Petersen and Pedersen's Matrix Cookbook, and cited here from memory.

(M5) Cyclic trace and determinant of a product. $\text{tr}(\mathbf{A} \mathbf{B}) = \text{tr}(\mathbf{B} \mathbf{A})$ for conforming $\mathbf{A}$, $\mathbf{B}$, and $\text{det}(\mathbf{A} \mathbf{B}) = \text{det}\mathbf{A} \text{det}\mathbf{B}$ for square matrices.

(M6) First order product of exponentials. $\text{Exp}(\mathbf{a})\,\text{Exp}(\mathbf{b}) = \text{Exp}(\mathbf{a} + \mathbf{b}) + O(|\mathbf{a}| \, |\mathbf{b}|)$, which follows from (I1) applied to each factor, the product $(\mathbf{I} + [\mathbf{a}]_\times)(\mathbf{I} + [\mathbf{b}]_\times) = \mathbf{I} + [\mathbf{a}]_\times + [\mathbf{b}]_\times + [\mathbf{a}]_\times [\mathbf{b}]_\times$, and (I3).

(M7) Moving an exponential through a rotation. $\text{Exp}(\boldsymbol{\phi})\,\mathbf{R} = \mathbf{R}\,\text{Exp}(\mathbf{R}^\top \boldsymbol{\phi})$, which is (I4) of the plan with $\mathbf{R}^\top$ in place of $\mathbf{R}$, multiplied on the left by $\mathbf{R}$.

## 3. Literature survey

Each subsection below describes one line of work, states its idea, and derives its mathematics step by step, quoting the paper where it was read. §3.1 gives the common background of marginalisation and first estimates on which every method builds.

### 3.1 Background, marginalisation and first estimate Jacobians

#### 3.1.1 The Schur complement as a minimisation

Linearise all residuals at the current point, $\mathbf{r}(\boldsymbol{\delta}) = \mathbf{r} + \mathbf{J} \boldsymbol{\delta}$, so that the cost is $F(\boldsymbol{\delta}) = \frac{1}{2}|\mathbf{r} + \mathbf{J} \boldsymbol{\delta}|^2$ and, as in the plan §2.5.1, $\mathbf{H} = \mathbf{J}^\top \mathbf{J}$ and $\mathbf{b} = \mathbf{J}^\top \mathbf{r}$. Split the increment into the variables that stay, $\boldsymbol{\delta}_\alpha$, and those to be removed, $\boldsymbol{\delta}_\beta$. Up to the constant $\frac{1}{2}\mathbf{r}^\top \mathbf{r}$,

$$F(\boldsymbol{\delta}_\alpha, \boldsymbol{\delta}_\beta) = \frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}_{\alpha\alpha} \boldsymbol{\delta}_\alpha + \boldsymbol{\delta}_\alpha^\top \mathbf{H}_{\alpha\beta} \boldsymbol{\delta}_\beta + \frac{1}{2}\boldsymbol{\delta}_\beta^\top \mathbf{H}_{\beta\beta} \boldsymbol{\delta}_\beta + \mathbf{b}_\alpha^\top \boldsymbol{\delta}_\alpha + \mathbf{b}_\beta^\top \boldsymbol{\delta}_\beta.$$

Step 1. The minimiser over $\boldsymbol{\delta}_\beta$ for fixed $\boldsymbol{\delta}_\alpha$ sets the derivative with respect to $\boldsymbol{\delta}_\beta$ to zero,

$$\mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha + \mathbf{H}_{\beta\beta} \boldsymbol{\delta}_\beta + \mathbf{b}_\beta = \mathbf{0},$$

$$\boldsymbol{\delta}_\beta^* = -\mathbf{H}_{\beta\beta}^{-1}\big(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha\big).$$

Step 2. Substituting $\boldsymbol{\delta}_\beta^*$ and using $\mathbf{H}_{\alpha\beta} = \mathbf{H}_{\beta\alpha}^\top$, the three terms that contain $\boldsymbol{\delta}_\beta$ become

$$\boldsymbol{\delta}_\alpha^\top \mathbf{H}_{\alpha\beta} \boldsymbol{\delta}_\beta^* + \frac{1}{2}\boldsymbol{\delta}_\beta^{*\top} \mathbf{H}_{\beta\beta} \boldsymbol{\delta}_\beta^* + \mathbf{b}_\beta^\top \boldsymbol{\delta}_\beta^* = -\frac{1}{2}\big(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha\big)^\top \mathbf{H}_{\beta\beta}^{-1}\big(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha\big),$$

because the stationarity of Step 1 makes the sum equal to $\frac{1}{2}(\mathbf{b}_\beta + \mathbf{H}_{\beta\alpha} \boldsymbol{\delta}_\alpha)^\top \boldsymbol{\delta}_\beta^*$.

Step 3. Expanding the product and collecting the terms quadratic, linear and constant in $\boldsymbol{\delta}_\alpha$,

$$F^*(\boldsymbol{\delta}_\alpha) = \frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}' \boldsymbol{\delta}_\alpha + \mathbf{b}'^\top \boldsymbol{\delta}_\alpha + \text{const},$$

$$\mathbf{H}' = \mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{H}_{\beta\alpha},$$

$$\mathbf{b}' = \mathbf{b}_\alpha - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{b}_\beta.$$

This is the marginalisation prior of DM-VIO [2, eq. 10 and 11], of VI-DSO [1, §III-F2] and of Basalt (`context/vio_opt.md` §12). By (M2) it is also the exact marginal of the linearised Gaussian. Basalt computes the same result in square root form by a QR decomposition of $\mathbf{J}$, which leaves $\mathbf{H}'$ and $\mathbf{b}'$ unchanged.

#### 3.1.2 Removing variables one at a time equals removing them together

Let $\boldsymbol{\delta}_\beta = [\boldsymbol{\delta}_{\beta_1}; \boldsymbol{\delta}_{\beta_2}]$. Step 1. By §3.1.1 the marginal prior is the minimum of $F$ over $\boldsymbol{\delta}_\beta$. Step 2. A minimum over two blocks equals the minimum over one block of the minimum over the other,

$$\min_{\boldsymbol{\delta}_{\beta_1}, \boldsymbol{\delta}_{\beta_2}} F = \min_{\boldsymbol{\delta}_{\beta_2}} \Big(\min_{\boldsymbol{\delta}_{\beta_1}} F\Big).$$

Step 3. The inner minimum is the prior after removing $\boldsymbol{\delta}_{\beta_1}$ alone, by §3.1.1, and the outer minimum removes $\boldsymbol{\delta}_{\beta_2}$ from that prior, again by §3.1.1. Hence successive Schur complements give exactly the joint one. This is what lets DM-VIO readvance a delayed graph "by successively marginalizing all the variables which have been marginalized in the main graph" [2, §III-E], at the cost of one small block inversion per step instead of one large one.

#### 3.1.3 Why the linearisation points must be frozen

The prior $\frac{1}{2}\boldsymbol{\delta}_\alpha^\top \mathbf{H}' \boldsymbol{\delta}_\alpha + \mathbf{b}'^\top \boldsymbol{\delta}_\alpha$ is a function of the offset $\boldsymbol{\delta}_\alpha$ from the point $\bar{\mathbf{x}}_\alpha$ at which its Jacobians were evaluated. If later factors on the same variables were linearised at a different point, their null directions and the prior's would differ, and the sum would appear to observe the gauge (plan §2.5.2, consequence 3). Basalt therefore freezes every variable that enters a prior, evaluates all its Jacobians at $\bar{\mathbf{x}}_\alpha$, and moves only the offset. VI-DSO states the same requirement, "it is important that Jacobians are all evaluated at the same value" [1, §III-F2], and DM-VIO describes the consequence, that "the linearization point of all connected variables has to be fixed" [2, §III-D].

### 3.2 Dynamic marginalisation, VI-DSO [1]

#### 3.2.1 Setting

VI-DSO evaluates the photometric error in a visual frame that is a scaled and rotated copy of the metric frame, related to it by a translation free similarity $\mathbf{T}_{m\_d}$, so that "the photometric error is always evaluated in the DSO frame, making it independent of the scale and gravity direction", while the inertial error is evaluated in the metric frame [1, §III-E]. The scale therefore enters only the inertial factors, which is the property that makes a scale independent prior possible. In the Basalt plan the scale enters the visual factor too, but only through the extrinsic lever arm (plan §2.2.4), so the same construction applies once that column is zeroed.

#### 3.2.2 The three priors

VI-DSO keeps three priors at once [1, §III-F3]. Write $G_\text{visual}$ for the visual factors connected to the frame being marginalised, $G_\text{metric}$ for its inertial factors and $G_\text{ba} = G_\text{visual} \cup G_\text{metric}$.

1. $M_\text{visual}$ "contains only scale independent information from previous states of the vision". It is updated by marginalising frame $i$ in $G_\text{visual} \cup M_\text{visual}$, so it never holds an inertial factor.
2. $M_\text{curr}$ "contains all information since the time we set the linearization point for the scale". It is updated by marginalising frame $i$ in $G_\text{ba} \cup M_\text{curr}$, and it is the prior the optimisation uses.
3. $M_\text{half}$ "contains only the recent states that have a scale close to the current estimate". It is updated by marginalising frame $i$ in $G_\text{metric} \cup G_\text{visual} \cup M_\text{half}$.

#### 3.2.3 The scale intervals

Let $s_i$ be the scale estimate at the moment frame $i$ was marginalised, and say that $i \in M$ when $M$ contains an inertial factor marginalised at that moment [1, eq. 14]. The constraints are [1, eq. 15 and 16]

$$\forall i \in M_\text{curr} :\ s_i \in \Big[\frac{s_\text{middle}}{d_i},\ s_\text{middle}\,d_i\Big],$$

$$\forall i \in M_\text{half} :\ s_i \in \begin{cases}[s_\text{middle},\ s_\text{middle}\,d_i] & \text{if } s_\text{curr} > s_\text{middle},\\ [s_\text{middle}/d_i,\ s_\text{middle}] & \text{otherwise.}\end{cases}$$

They are enforced by Algorithm 1 [1], run at every marginalisation. When the current scale changes side of $s_\text{middle}$, $M_\text{half}$ is reset to $M_\text{visual}$. When it leaves the interval upwards, $M_\text{curr} \leftarrow M_\text{half}$, $M_\text{half} \leftarrow M_\text{visual}$ and $s_\text{middle} \leftarrow s_\text{middle}\,d_i$, and symmetrically downwards with a division. The interval size follows eq. 17 of [1], printed as $d_i = \min\{d_\text{min}^j \mid j \in \mathbb{N}\setminus\{0\},\ s_i / s_{i-1} < d_i\}$, in which $d_i$ appears on both sides, so its exact form is ambiguous. The paper uses $d_\text{min} = \sqrt{1.1}$, and gives two reasons, that $d_i$ "should be small, in order to keep the system consistent" but "not too small so that $M_\text{curr}$ always contains enough inertial factors" [1].

#### 3.2.4 The bound on the scale spread inside the prior

The paper states that "the scale difference in the currently used marginalization factor stays smaller than $d_i^2$" [1]. The derivation is short. Step 1. By the first constraint, every $s_i$ in $M_\text{curr}$ lies in $[s_\text{middle}/d_i,\ s_\text{middle}\,d_i]$. Step 2. The largest ratio between two values in this interval is that of its end points,

$$\frac{s_\text{middle}\,d_i}{s_\text{middle}/d_i} = d_i^2.$$

Step 3. In log scale the same statement reads $|\sigma_i - \sigma_j| \le 2\ln d_i$ for any two factors in $M_\text{curr}$, which with $d_\text{min} = \sqrt{1.1}$ is about 0.095, a spread of 9.5 % in scale. Inserting $\Delta s / \bar{s} \le 0.095$ into §1.3 bounds the linearisation error of every inertial factor in the prior, which is the consistency the method buys.

### 3.3 Delayed marginalisation, DM-VIO [2]

#### 3.3.1 Idea

DM-VIO starts from the observation that "marginalization cannot be undone, but it can be delayed" [2, §III-D]. Besides the main graph, it maintains "a second, delayed marginalization prior and corresponding factor graph", in which frames are marginalised with a delay of $d$ keyframes and "the same marginalization order as in the original graph is preserved" [2, §III-D]. The delayed graph therefore still contains the original nonlinear factors of the last $d$ keyframes, which the main graph has already turned into a linear prior. The paper reports $d = 100$ with at most $N_f = 8$ keyframes in the main window [2, §III-D and §III-F].

#### 3.3.2 Readvancing

When an updated prior is needed, the delayed graph is readvanced, which "works by successively marginalizing all the variables which have been marginalized in the main graph" [2, §III-E], now linearised at new points. By §3.1.2 the successive Schur complements equal the joint one, and the paper notes that "marginalizing step by step is significantly faster than marginalizing all variables at once" [2, §III-E]. The new prior has "new and consistent linearization points" [2, abstract], because every factor of the delayed window is relinearised at the current estimate before it is marginalised. The measured overhead of maintaining the delayed graph is 0.44 ms, 0.8 % of the keyframe thread [2, §IV-A], because "marginalization in the delayed graph has the same runtime as marginalization in the original graph" [2, §III-D].

#### 3.3.3 Marginalisation replacement on a scale change

DM-VIO optimises scale and gravity direction "as explicit variables", with the visual frame of arbitrary scale and rotation and the yaw coordinate of the gravity rotation fixed [2, §III-B]. Instead of VI-DSO's swaps, it compares the current scale with the one at which the prior was linearised,

$$\delta_s = \frac{\max(s, s_\text{fej})}{\min(s, s_\text{fej})},$$

and when $\delta_s$ exceeds a threshold $\theta_s$ it triggers a marginalisation replacement, in which "we do not optimize in this graph but instead just readvance it to obtain an updated marginalization prior" [2, §III-F]. The replacement is disabled "if more than" half of the IMU factors in the previous prior would be lost, and the new prior keeps all visual factors and at least $d - N_f + 1 = 93$ IMU factors [2, §III-F]. The numeric value of $\theta_s$ is not given in the text read. DM-VIO describes VI-DSO's dynamic marginalisation as keeping the factor consistent but losing "too much information in the process", and as "limited in its application to a single one-dimensional variable, namely the scale" [2, §I and §III-D].

#### 3.3.4 Comparison in information kept

The difference between the two methods can be stated in terms of §1.4. A VI-DSO swap keeps the inertial factors since the last reset of $M_\text{half}$, which at take off is almost none. A DM-VIO replacement keeps the inertial factors of the last $d$ keyframes, relinearised at the new scale, so the information is kept and only its linearisation is renewed. What is lost in DM-VIO is only what lies more than $d$ keyframes back, which for $d = 100$ is far beyond a ten second initialisation period.

### 3.4 Non-linear factor recovery [9, 3, 6] and generic linear constraints [5]

#### 3.4.1 Idea

A dense linear prior cannot be relinearised. Non-linear factor recovery replaces it by a set of nonlinear factors whose linearisation reproduces it as closely as possible, measured by the Kullback–Leibler divergence. Mazuran et al. introduced it for long term graph SLAM [9], Hsiung et al. applied it to sparsify the dense prior of a fixed lag VIO [6] (abstract read), and Basalt uses it to pass VIO information to its global mapper [3, §V]. Carlevaris-Bianco et al.'s generic linear constraints [5] (abstract read) are a related node removal method that produces linearised factors over the elimination clique, exact as dense constraints or approximate on a Chow–Liu tree.

#### 3.4.2 Derivation of the recovered information

Let the marginalised distribution be $p = \mathcal{N}(\boldsymbol{\mu}_o, \boldsymbol{\Sigma}_o)$ with $\boldsymbol{\Sigma}_o = \mathbf{H}_o^{-1}$ [3, §V-B]. Choose nonlinear residuals $\mathbf{r}_i(\mathbf{s}, \mathbf{z}_i)$ with unknown information $\mathbf{H}_i$, stack their Jacobians at $\boldsymbol{\mu}_o$ into $\mathbf{J}_r$ and their informations into the block diagonal $\mathbf{H}_r$, so that the recovered distribution has information [3, eq. 28]

$$\mathbf{H}_a = \mathbf{J}_r^\top \mathbf{H}_r \mathbf{J}_r.$$

Step 1. The pseudo measurements are chosen so that every residual vanishes at the marginal mean, $\mathbf{r}_i(\boldsymbol{\mu}_o, \mathbf{z}_i) = \mathbf{0}$, which makes the recovered mean equal the original, $\boldsymbol{\mu}_a = \boldsymbol{\mu}_o$ [3], so the mean term of (M3) vanishes,

$$D_\text{KL} = \frac{1}{2}\Big(\text{tr}(\mathbf{H}_a \boldsymbol{\Sigma}_o) - \ln\,\text{det}(\mathbf{H}_a \boldsymbol{\Sigma}_o) - d\Big).$$

Step 2. Substituting $\mathbf{H}_a$ and applying the cyclic property of (M5) to the trace,

$$\text{tr}(\mathbf{J}_r^\top \mathbf{H}_r \mathbf{J}_r \boldsymbol{\Sigma}_o) = \text{tr}(\mathbf{H}_r \mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top) = \text{tr}(\mathbf{H}_r \mathbf{S}),$$

with $\mathbf{S} = \mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top$. Since $\mathbf{H}_r$ is block diagonal, only the diagonal blocks $\mathbf{S}_{ii}$ of $\mathbf{S}$ enter,

$$\text{tr}(\mathbf{H}_r \mathbf{S}) = \sum_i \text{tr}(\mathbf{H}_i \mathbf{S}_{ii}).$$

Step 3. For a square and invertible $\mathbf{J}_r$, which [3] assumes for the closed form, the determinant of (M5) splits the log determinant,

$$\ln\,\text{det}(\mathbf{J}_r^\top \mathbf{H}_r \mathbf{J}_r \boldsymbol{\Sigma}_o) = \ln\,\text{det}\mathbf{H}_r + \ln\,\text{det}(\mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top) = \sum_i \ln\,\text{det}\mathbf{H}_i + \ln\,\text{det}\mathbf{S},$$

where the first equality moves $\mathbf{J}_r^\top$ to the right inside the determinant by (M5), and the second uses that a block diagonal determinant is the product of its blocks.

Step 4. The divergence is therefore a sum of independent terms, one per factor, plus terms free of the $\mathbf{H}_i$,

$$D_\text{KL} = \frac{1}{2}\sum_i \Big(\text{tr}(\mathbf{H}_i \mathbf{S}_{ii}) - \ln\,\text{det}\mathbf{H}_i\Big) + \text{const}.$$

Step 5. Setting the derivative with respect to each $\mathbf{H}_i$ to zero with (M4),

$$\mathbf{S}_{ii} - \mathbf{H}_i^{-1} = \mathbf{0},$$

$$\mathbf{H}_i = \big(\{\mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top\}_i\big)^{-1},$$

which is the closed form of [3]. The recovered factor's information is the inverse of the marginal covariance of its own residual, so each factor carries exactly the uncertainty the dense prior assigns to the quantity it measures.

#### 3.4.3 The factors Basalt recovers

Basalt recovers relative pose factors between the keyframe to be removed and the other keyframes, and a roll pitch factor on that keyframe, and drops the absolute position and yaw factors because they are unobservable in VIO [3, §V-C]. The residuals are [3, eq. 31 and 32]

$$\mathbf{r}_\text{rel} = \text{Log}\big(\mathbf{z}_\text{rel}\,\mathbf{T}_j^{-1} \mathbf{T}_i\big),$$

$$\mathbf{z}_\text{rel} = \mathbf{T}_i^{-1} \mathbf{T}_j,$$

$$\mathbf{r}_\text{rp} = \big\lfloor \mathbf{z}_\text{rp}\,\mathbf{R}_i^{-1}(0, 0, -1)^\top \big\rfloor_{xy},$$

$$\mathbf{z}_\text{rp} = \mathbf{R}_i,$$

with the measurements taken "from the estimated state at the time of linearization" [3]. Because these factors are nonlinear, the mapper can relinearise them at any later estimate, which is what makes NFR an alternative to freezing.

### 3.5 On manifold preintegration [4]

#### 3.5.1 Definitions

For an interval from keyframe $i$ to keyframe $j$, Forster et al. define relative motion increments "independent of the pose and velocity at" $t_i$ [4, eq. 33] and model them as functions of the states [4, eq. 38]. Without noise, in their notation,

$$\Delta\mathbf{R}_{ij} = \mathbf{R}_i^\top \mathbf{R}_j,$$

$$\Delta\mathbf{v}_{ij} = \mathbf{R}_i^\top\big(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\Delta t_{ij}\big),$$

$$\Delta\mathbf{p}_{ij} = \mathbf{R}_i^\top\Big(\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i \Delta t_{ij} - \frac{1}{2}\mathbf{g}\Delta t_{ij}^2\Big).$$

Their noisy measurements are $\Delta\tilde{\mathbf{R}}_{ij} = \Delta\mathbf{R}_{ij}\,\text{Exp}(\delta\boldsymbol{\phi}_{ij})$, $\Delta\tilde{\mathbf{v}}_{ij} = \Delta\mathbf{v}_{ij} + \delta\mathbf{v}_{ij}$ and $\Delta\tilde{\mathbf{p}}_{ij} = \Delta\mathbf{p}_{ij} + \delta\mathbf{p}_{ij}$ [4, eq. 38]. Forster et al. estimate only keyframe states and summarise "several IMU measurements" in "a single preintegrated IMU factor" [4], assuming the bias constant between consecutive keyframes [4, eq. 34]. Basalt writes the same quantities in its own rotation convention (`doc/VIO.md` §3.1.1, plan §2.3.1). A rule for composing two adjacent intervals is not stated in [4], so it is derived here.

#### 3.5.2 Composition of two adjacent intervals, derived here from the model of [4]

Let $i < j < k$. Step 1, rotation. Inserting $\mathbf{R}_j \mathbf{R}_j^\top = \mathbf{I}$,

$$\Delta\mathbf{R}_{ik} = \mathbf{R}_i^\top \mathbf{R}_k = \mathbf{R}_i^\top \mathbf{R}_j \mathbf{R}_j^\top \mathbf{R}_k = \Delta\mathbf{R}_{ij}\,\Delta\mathbf{R}_{jk}.$$

Step 2, velocity. Splitting $\mathbf{v}_k - \mathbf{v}_i = (\mathbf{v}_j - \mathbf{v}_i) + (\mathbf{v}_k - \mathbf{v}_j)$ and $\Delta t_{ik} = \Delta t_{ij} + \Delta t_{jk}$,

$$\Delta\mathbf{v}_{ik} = \mathbf{R}_i^\top\big(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\Delta t_{ij}\big) + \mathbf{R}_i^\top\big(\mathbf{v}_k - \mathbf{v}_j - \mathbf{g}\Delta t_{jk}\big).$$

The first term is $\Delta\mathbf{v}_{ij}$. Inserting $\mathbf{R}_j \mathbf{R}_j^\top = \mathbf{I}$ in the second gives $\Delta\mathbf{R}_{ij}\,\Delta\mathbf{v}_{jk}$, so

$$\Delta\mathbf{v}_{ik} = \Delta\mathbf{v}_{ij} + \Delta\mathbf{R}_{ij}\,\Delta\mathbf{v}_{jk}.$$

Step 3, position. Splitting $\mathbf{p}_k - \mathbf{p}_i = (\mathbf{p}_j - \mathbf{p}_i) + (\mathbf{p}_k - \mathbf{p}_j)$, and using $\Delta t_{ik}^2 = \Delta t_{ij}^2 + 2\Delta t_{ij}\Delta t_{jk} + \Delta t_{jk}^2$ and $\mathbf{v}_i \Delta t_{jk} = \mathbf{v}_j \Delta t_{jk} - (\mathbf{v}_j - \mathbf{v}_i)\Delta t_{jk}$,

$$\mathbf{p}_k - \mathbf{p}_i - \mathbf{v}_i \Delta t_{ik} - \frac{1}{2}\mathbf{g}\Delta t_{ik}^2 = \Big(\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i \Delta t_{ij} - \frac{1}{2}\mathbf{g}\Delta t_{ij}^2\Big) + \big(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\Delta t_{ij}\big)\Delta t_{jk} + \Big(\mathbf{p}_k - \mathbf{p}_j - \mathbf{v}_j \Delta t_{jk} - \frac{1}{2}\mathbf{g}\Delta t_{jk}^2\Big).$$

The middle term collects $-\mathbf{v}_i \Delta t_{jk} + \mathbf{v}_j \Delta t_{jk} - \mathbf{v}_j \Delta t_{jk}$ from the regrouping and $-\mathbf{g}\Delta t_{ij}\Delta t_{jk}$ from the cross term of $\Delta t_{ik}^2$. Multiplying on the left by $\mathbf{R}_i^\top$ and inserting $\mathbf{R}_j \mathbf{R}_j^\top = \mathbf{I}$ in the last term,

$$\Delta\mathbf{p}_{ik} = \Delta\mathbf{p}_{ij} + \Delta\mathbf{v}_{ij}\,\Delta t_{jk} + \Delta\mathbf{R}_{ij}\,\Delta\mathbf{p}_{jk}.$$

These three relations use only the measurement model, so they hold for any integrator that is exact on its own interval, including Basalt's.

#### 3.5.3 Composition of the noise, derived here

Step 1, rotation. Substituting the noisy factors and moving the second rotation through the first exponential with (M7),

$$\Delta\tilde{\mathbf{R}}_{ik} = \Delta\mathbf{R}_{ij}\,\text{Exp}(\delta\boldsymbol{\phi}_{ij})\,\Delta\mathbf{R}_{jk}\,\text{Exp}(\delta\boldsymbol{\phi}_{jk}) = \Delta\mathbf{R}_{ij}\,\Delta\mathbf{R}_{jk}\,\text{Exp}(\Delta\mathbf{R}_{jk}^\top \delta\boldsymbol{\phi}_{ij})\,\text{Exp}(\delta\boldsymbol{\phi}_{jk}).$$

By (M6),

$$\delta\boldsymbol{\phi}_{ik} = \Delta\mathbf{R}_{jk}^\top \delta\boldsymbol{\phi}_{ij} + \delta\boldsymbol{\phi}_{jk}.$$

Step 2, velocity. Substituting into Step 2 of §3.5.2 and expanding $\text{Exp}(\delta\boldsymbol{\phi}_{ij})$ by (I1),

$$\Delta\tilde{\mathbf{v}}_{ik} = \Delta\mathbf{v}_{ij} + \delta\mathbf{v}_{ij} + \Delta\mathbf{R}_{ij}\big(\mathbf{I} + [\delta\boldsymbol{\phi}_{ij}]_\times\big)\big(\Delta\mathbf{v}_{jk} + \delta\mathbf{v}_{jk}\big).$$

Keeping first order terms and swapping the cross product by (I2),

$$\delta\mathbf{v}_{ik} = \delta\mathbf{v}_{ij} - \Delta\mathbf{R}_{ij}\,[\Delta\mathbf{v}_{jk}]_\times\,\delta\boldsymbol{\phi}_{ij} + \Delta\mathbf{R}_{ij}\,\delta\mathbf{v}_{jk}.$$

Step 3, position. The same expansion of Step 3 of §3.5.2 gives

$$\delta\mathbf{p}_{ik} = \delta\mathbf{p}_{ij} + \Delta t_{jk}\,\delta\mathbf{v}_{ij} - \Delta\mathbf{R}_{ij}\,[\Delta\mathbf{p}_{jk}]_\times\,\delta\boldsymbol{\phi}_{ij} + \Delta\mathbf{R}_{ij}\,\delta\mathbf{p}_{jk}.$$

Step 4. Stacking the noise as $\boldsymbol{\eta} = [\delta\boldsymbol{\phi}; \delta\mathbf{v}; \delta\mathbf{p}]$ in Forster's order, Steps 1 to 3 read $\boldsymbol{\eta}_{ik} = \mathbf{A}\,\boldsymbol{\eta}_{ij} + \mathbf{B}\,\boldsymbol{\eta}_{jk}$ with

$$\mathbf{A} = \begin{bmatrix}\Delta\mathbf{R}_{jk}^\top & \mathbf{0} & \mathbf{0}\\ -\Delta\mathbf{R}_{ij}\,[\Delta\mathbf{v}_{jk}]_\times & \mathbf{I} & \mathbf{0}\\ -\Delta\mathbf{R}_{ij}\,[\Delta\mathbf{p}_{jk}]_\times & \Delta t_{jk}\,\mathbf{I} & \mathbf{I}\end{bmatrix},$$

$$\mathbf{B} = \begin{bmatrix}\mathbf{I} & \mathbf{0} & \mathbf{0}\\ \mathbf{0} & \Delta\mathbf{R}_{ij} & \mathbf{0}\\ \mathbf{0} & \mathbf{0} & \Delta\mathbf{R}_{ij}\end{bmatrix}.$$

Step 5. The noises of disjoint intervals are independent, so the covariance of the composed measurement is

$$\boldsymbol{\Sigma}_{ik} = \mathbf{A}\,\boldsymbol{\Sigma}_{ij}\,\mathbf{A}^\top + \mathbf{B}\,\boldsymbol{\Sigma}_{jk}\,\mathbf{B}^\top.$$

The bias Jacobians compose by the same chain rule, under the constant bias assumption of [4, eq. 34].

### 3.6 Scale as a filter state with process noise, and inertial only refinement

#### 3.6.1 Idea

Before optimisation based VIO, monocular visual inertial fusion estimated the unknown scale of a visual map as an extra state of an extended Kalman filter. Nützi et al. compared two ways of estimating the scale of monocular SLAM from IMU data and fused the two sensors through a multi rate EKF, and a later work by Kneip, Weiss and Siegwart describes the approach as adding "the scale factor of the monocular SLAM as an additional variable to the state" [7] (abstract and search snippets only). ORB-SLAM3 instead solves scale and gravity in an inertial only maximum a posteriori problem over keyframes whose poses are held fixed, and refines them in the same way at about 5 s and 15 s, reporting a scale error of about 1 % after the refinements on EuRoC [8] (search snippets only, the implementation was read for `plans/gravity_scale_optimisation.md`).

#### 3.6.2 The random walk model of a scale state

The filter form makes explicit what a bound on the per frame change means. Let the log scale follow a random walk between frames $k$ and $k+1$,

$$\sigma_{k+1} = \sigma_k + w_k,$$

$$w_k \sim \mathcal{N}(0,\ q\,\Delta t_k),$$

with intensity $q$. Step 1. In a smoother the same model is a factor on consecutive values,

$$r_k = \frac{\sigma_{k+1} - \sigma_k}{\sqrt{q\,\Delta t_k}}.$$

Step 2. Because the plan re-centres the block to the identity at every frame, the value at the start of a frame is $\sigma_k = 0$ and the block's offset at the end is the step itself, so the factor becomes a penalty on the step, $r_k = \delta\sigma / \sqrt{q\,\Delta t_k}$. Step 3. Without measurements the variance after $n$ frames is $n\,q\,\Delta t$, which grows without bound, so a random walk limits the rate of change but supplies no information on the value. This distinction returns in §4.5.

#### 3.6.3 Relevance

The filter literature shows that a scale state with process noise is a sound model, and ORB-SLAM3 shows that the scale can be refined in a few discrete steps rather than continuously. Neither addresses a marginalisation prior that must stay consistent while the scale moves, which is the subject of §3.2 to §3.5.

## 4. Proposals

The proposals follow from §1 and §3. §4.1 to §4.4 keep the inertial information through a change of scale and tilt, in increasing order of change to the estimator. §4.5 bounds the per frame jump, and §4.6 compares all of them and orders them. Each proposal is derived in full. None has been implemented or run, and the novelty of those not found in the papers read is stated as unverified.

### 4.1 Keep the block independent inertial rows in the scale independent prior

VI-DSO's $M_\text{visual}$ omits all inertial rows. The Jacobians of the plan §2.3.2 and §2.3.3 show that only two of the five row groups of a Basalt inertial factor depend on the gravity and scale block, so the omission is stronger than the requirement of scale independence. The five groups are $\mathbf{r}_p$, $\mathbf{r}_R$, $\mathbf{r}_v$ and the two bias random walk residuals $\mathbf{r}_{b_g}$, $\mathbf{r}_{b_a}$.

Step 1. The rotation row is $\mathbf{r}_R = \text{Log}\big(\text{Exp}(\boldsymbol{\delta}_g[4{:}6])\,\Delta\mathbf{R}\,\mathbf{R}_1^\top \mathbf{R}_0\big)$, which contains neither $s$ nor $\mathbf{g}_D$ (plan §2.3.1), so

$$\frac{\partial\mathbf{r}_R}{\partial\sigma} = \mathbf{0},$$

$$\frac{\partial\mathbf{r}_R}{\partial\boldsymbol{\delta}_\theta} = \mathbf{0}.$$

Step 2. Its Jacobians with respect to the states, $\pm\mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top$ and the gyroscope bias block (plan §2.3.2), contain no $s$ and no $\mathbf{g}_D$, so a linearisation at a wrong scale or tilt does not corrupt them, and §1.3 has no analogue for this row.

Step 3. The bias random walk rows, $\mathbf{W}_{b_g}(\mathbf{b}_{g,0} - \mathbf{b}_{g,1})$ and $\mathbf{W}_{b_a}(\mathbf{b}_{a,0} - \mathbf{b}_{a,1})$ (`doc/VIO.md` §3.1.1), are linear in the biases with constant Jacobians, and contain neither $s$ nor $\mathbf{g}_D$.

Step 4. The position and velocity rows contain $s(\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0\Delta t)$, $s(\mathbf{v}_1 - \mathbf{v}_0)$ and $\mathbf{g}_D$, and their state Jacobians carry $s$ (plan §2.3.2), so they are the rows that §1.3 makes inconsistent.

It follows that a prior built from the visual rows with the scale column zeroed, the inertial rows $\mathbf{r}_R$, $\mathbf{r}_{b_g}$, $\mathbf{r}_{b_a}$ and the initial prior is exactly independent of $\sigma$ and of $\boldsymbol{\delta}_\theta$, as VI-DSO requires of $M_\text{visual}$, and it keeps the relative rotations, the gyroscope bias and, by §1.5, the accelerometer bias anchor through every swap. One subtlety concerns the stationary phase. When the vehicle is at rest, $\mathbf{v}_1 - \mathbf{v}_0 \approx \mathbf{0}$, so $\mathbf{r}_v \approx -\mathbf{R}_0^\top \mathbf{g}_D \Delta t - \hat{\Delta\mathbf{v}}$ depends on the tilt through $\mathbf{g}_D$ but hardly on $s$. These rows carry the tilt and bias information of the ground phase. They can enter $M_\text{visual}$ with their scale column zeroed, because the swap rule of VI-DSO acts on the scale alone, and the tilt enters the inertial rows linearly through $\mathbf{g}_D$ with a first estimate error of second order in the tilt change. `plans/gravity_scale_optimisation_2.md` §2.3 derives the admission test, a bound on the whitened scale column. Note, 2026-11-06. This sentence previously concluded that these rows cannot enter a prior that must be independent of the tilt. $M_\text{visual}$ needs to be independent of the scale only, which is what the swap rule tracks.

This partition was not found in the papers read. VI-DSO defines $M_\text{visual}$ as visual only [1, §III-F3] and DM-VIO replaces dynamic marginalisation altogether [2], so whether it has been published elsewhere is not established.

### 4.2 Delayed marginalisation of the inertial chain

The delayed graph must store, for each of the last $d$ keyframes, the factors that the main graph marginalises. In Basalt these are the preintegrated inertial factors, which are small objects (`IntegratedImuMeasurement`), and the visual factors, which Basalt holds per landmark as linearised blocks and reduces by QR (`context/vio_opt.md` §7.3). Readvancing would need the landmark observations of the delayed keyframes, so that their visual factors can be relinearised, or, following DM-VIO's pose graph bundle adjustment, a choice to keep the visual part at its old linearisation and to relinearise only the inertial part. DM-VIO makes exactly that choice in its initialiser, "we do not update our estimates for point depths and do not relinearize photometric error terms" [2, §III-E]. Since the visual part of the prior is independent of the block (§4.1), keeping it frozen loses no consistency, and the delayed graph then reduces to the chain of inertial factors over the delayed window.

### 4.3 Compose preintegrated factors instead of marginalising them

When a state $j$ is connected only to its two inertial factors and the bias random walk, removing it by composition leaves a single nonlinear inertial factor between $i$ and $k$, which still contains $s$ and $\mathbf{g}_D$ explicitly and can be relinearised at any later scale and tilt. Marginalisation would instead freeze a linear prior over $i$ and $k$ at the scale of the moment. To first order the two agree, because by (M2) the marginal of the linearised pair is the Gaussian whose noise propagates as in §3.5.3. The composed factor differs in remaining valid when the scale moves, which is the property §1.3 shows the frozen prior lacks. In Basalt a state is not removed whole. Its velocity and biases are marginalised while its pose is kept as a keyframe for vision, so composition applies only to states that leave the window entirely, or to a variant in which keyframes keep their velocity and biases during the initialisation period, at a cost of nine extra dimensions per keyframe. Composition of preintegrated factors as a remedy for scale inconsistent priors was not found in the papers read, which preintegrate between keyframes from the raw samples [4].

### 4.4 Scale aware recovered factors

A recovered relative pose factor carries the metric translation of $\mathbf{z}_\text{rel}$, so a change of scale changes its residual, and the scaling map of the plan §2.2.2 says how. Step 1. Under a uniform rescaling of the world by $s$, every pose maps as $\mathbf{T} \to \mathcal{S}_s(\mathbf{T})$ and, if the measurement is scaled with it, $\mathbf{z}_\text{rel} \to \mathcal{S}_s(\mathbf{z}_\text{rel})$. Step 2. By (S1) of the plan, $\mathcal{S}_s$ preserves products and inverses, so the argument of the logarithm maps as

$$\mathcal{S}_s(\mathbf{z}_\text{rel})\,\mathcal{S}_s(\mathbf{T}_j)^{-1} \mathcal{S}_s(\mathbf{T}_i) = \mathcal{S}_s\big(\mathbf{z}_\text{rel}\,\mathbf{T}_j^{-1} \mathbf{T}_i\big).$$

Step 3. By (S2), writing the argument as $\text{Exp}(\mathbf{r}_\text{rel})$, its image is $\text{Exp}(\mathbf{D}_s \mathbf{r}_\text{rel})$, so

$$\mathbf{r}_\text{rel} \to \mathbf{D}_s\,\mathbf{r}_\text{rel}.$$

The factor is therefore covariant under scaling, and keeping it consistent requires only that its information transform as $\mathbf{H} \to \mathbf{D}_s^{-1} \mathbf{H} \mathbf{D}_s^{-1}$, which is the local mapper update of the plan §2.9.2 item 3. A recovered factor could instead be written with the scale as an argument, $\mathbf{r}_\text{rel}(\mathbf{T}_i, \mathbf{T}_j, s) = \text{Log}(\mathcal{S}_{s}(\mathbf{z}_\text{rel})\,\mathbf{T}_j^{-1} \mathbf{T}_i)$ with $\mathbf{z}_\text{rel}$ stored at the scale of its linearisation, so that it follows the scale nonlinearly instead of being frozen at one. Such scale aware recovered factors were not found in the papers read, which recover factors over poses only [3, 6, 9].

### 4.5 Bounding the per frame jump

#### 4.5.1 A residual on the jump, and its interaction with the gauge

The proposal is a residual on the size of the per frame correction. Because the plan re-centres the block to the identity at every frame (plan §2.7), the block's offset at the end of a solve is the jump itself, and a residual

$$\mathbf{r}_\text{jump} = \mathbf{W}\,\boldsymbol{\delta}_{gs}$$

with a diagonal weight $\mathbf{W} = \text{diag}(w_\theta, w_\theta, w_\sigma)$ penalises it directly. Its contribution to the system is $\mathbf{H}_\text{jump} = \mathbf{W}^\top \mathbf{W}$ on the block's three coordinates. This is the random walk model of an EKF that holds the scale as a state with process noise, as Nützi et al. and Weiss and Siegwart do [7] (abstract read). A per frame step of standard deviation $1/w_\sigma$ corresponds to a random walk of intensity $1/(w_\sigma^2 \Delta t)$.

The block coordinates are not gauge invariant, so the residual must be checked against the projection of the plan §2.5.6. Consider the scale part alone, with $\mathbf{e}_\sigma$ the unit vector of the block's $\sigma$ coordinate.

Step 1. Along the scale gauge direction $\mathbf{n}_\sigma$, whose $\sigma$ entry is $-1$ (plan §2.5.3), the residual's curvature is

$$\mathbf{n}_\sigma^\top \mathbf{H}_\text{jump} \mathbf{n}_\sigma = w_\sigma^2 \big(\mathbf{e}_\sigma^\top \mathbf{n}_\sigma\big)^2 = w_\sigma^2,$$

which is not zero, so on its own the residual is a gauge prior in the sense of the plan §2.5.4. Without the projection it would prefer the representation with $\sigma = 0$, the one that carries every correction in the frame states, and the estimator would return to today's behaviour.

Step 2. With the projection, the residual enters as $\mathbf{P}\,\mathbf{H}_\text{jump}\,\mathbf{P} = w_\sigma^2\,(\mathbf{P} \mathbf{e}_\sigma)(\mathbf{P} \mathbf{e}_\sigma)^\top$, which acts only through the gauge free part of $\mathbf{e}_\sigma$. Its squared length is

$$|\mathbf{P} \mathbf{e}_\sigma|^2 = \mathbf{e}_\sigma^\top \mathbf{P} \mathbf{e}_\sigma = 1 - \frac{(\mathbf{e}_\sigma^\top \mathbf{n}_\sigma)^2}{|\mathbf{n}_\sigma|^2} = 1 - \frac{1}{|\mathbf{n}_\sigma|^2},$$

using $\mathbf{P} = \mathbf{I} - \mathbf{n}_\sigma \mathbf{n}_\sigma^\top / |\mathbf{n}_\sigma|^2$ for the scale direction alone, $\mathbf{P}^\top \mathbf{P} = \mathbf{P}$, and $\mathbf{e}_\sigma^\top \mathbf{n}_\sigma = -1$.

Step 3. With $|\mathbf{n}_\sigma|^2 = 1 + \sum_f |\mathbf{p}_f|^2 + \sum_f |\mathbf{v}_f|^2$ (plan §2.5.7), the effective weight is $w_\sigma^2 (1 - 1/|\mathbf{n}_\sigma|^2)$. At rest at the origin it vanishes, so the residual neither constrains nor harms the stationary phase. Once the window has moved, it approaches $w_\sigma^2$ and limits the physical scale change per frame. The projected residual is therefore a jump bound on the physical correction, as intended, provided it is added before the projection is applied.

#### 4.5.2 A jump residual does not replace the lost information

A jump residual bounds each step but not their sum. If the steps are biased, because the prior has lost the inertial information that would determine them (§1.4 and §1.5), a sequence of small steps still drifts, as the steady tilt drift of about 7° per second in the replays shows. The residual is a damping of the transient, and the remedy for the drift is to keep the information. In a Kalman reading, process noise without measurements lets the variance of the scale grow without bound.

#### 4.5.3 Partial re-centring as an exact alternative

Instead of penalising the step inside the solve, the re-centring of the plan §2.7 can apply only part of the estimated correction, for example the log scale clamped to $|\sigma_c| \le \sigma_\text{max}$ and the tilt to an angle $\theta_\text{max}$, and leave the remainder in the block. The change of variables of the plan §2.7.3 is exact for any $\mathbf{Q}$ and $s_c$, so applying a smaller correction is also exact. Nothing is lost, and the remainder is applied in later frames once it persists. This bounds what the published trajectory and the local mapper see per frame without changing what the solve estimates.

#### 4.5.4 Holding the block while it carries no information

§4.5.1 Step 3 shows that at rest the scale coordinate is itself a gauge direction, since $|\mathbf{n}_\sigma| \to 1$, and the replays show that the largest steps occur in the first second of motion, when $|\mathbf{n}_\sigma|$ has only just left 1. Both suggest holding the block at the identity, which is exactly today's estimator (plan §2.5.4, item 4), until the block carries information. The information the projected system holds on the block alone is not such a measure, because it is identically zero. Write $\bar{\mathbf{H}} = \mathbf{P} \mathbf{H} \mathbf{P}$ in block ($g$) and frame ($f$) parts and split each gauge vector as $\mathbf{n} = [\mathbf{n}_g; \mathbf{n}_f]$. The two block rows of $\bar{\mathbf{H}} \mathbf{n} = \mathbf{0}$ give $\bar{\mathbf{H}}_{gg} \mathbf{n}_g + \bar{\mathbf{H}}_{gf} \mathbf{n}_f = \mathbf{0}$ and $\bar{\mathbf{H}}_{fg} \mathbf{n}_g + \bar{\mathbf{H}}_{ff} \mathbf{n}_f = \mathbf{0}$, so the Schur complement $\mathbf{S} = \bar{\mathbf{H}}_{gg} - \bar{\mathbf{H}}_{gf} \bar{\mathbf{H}}_{ff}^{-1} \bar{\mathbf{H}}_{fg}$ satisfies $\mathbf{S} \mathbf{n}_g = \bar{\mathbf{H}}_{gg} \mathbf{n}_g + \bar{\mathbf{H}}_{gf} \mathbf{n}_f = \mathbf{0}$ for all three gauge vectors. Their block parts $(0, 0, -1)$, $(\bar{\mathbf{B}}^\top \mathbf{e}_x, 0)$ and $(\bar{\mathbf{B}}^\top \mathbf{e}_y, 0)$ span $\mathbb{R}^3$, so $\mathbf{S} = \mathbf{0}$. The block's value can always be traded against the frames, and only gauge invariant quantities carry information. `plans/gravity_scale_optimisation_2.md` §2.4 therefore gates softly, through a random walk residual on the block step added before the projection, which shrinks the step whenever the data carry little information. Note, 2026-11-06. This paragraph previously proposed the smallest eigenvalue of that Schur complement as the gate, which is zero by the derivation above. It also proposed holding the block during the ground phase with the stationary detector, which is unnecessary because at rest the projection already removes every scale step. This gate does not change what is estimated once the block is released, so it combines with every proposal of §4.1 to §4.4.

### 4.6 Synthesis and order

| Method | Inertial information kept through a scale change | Cost in Basalt | Evidence |
|---|---|---|---|
| Dynamic marginalisation [1], as implemented | only since the last reset of $M_\text{half}$, almost none at take off | three priors | §1.4, replays of 2026-11-06 |
| $M_\text{visual}$ with the block independent inertial rows (§4.1) | rotation rows, bias random walk and the bias anchor through every swap, position and velocity rows still lost | the same three priors, with a different row selection | derivation §1.5 and §4.1, untested |
| Delayed marginalisation [2] | all inertial factors of the last $d$ keyframes, relinearised | one delayed graph of inertial factors, about 0.8 % of the keyframe thread in [2] | [2], §3.3 |
| Recovered nonlinear factors [3, 6, 9] with scale as an argument (§4.4) | the marginal as relinearisable factors | one recovery per marginalisation | NFR established, scale aware variant not found |
| Preintegration composition (§3.5) | exact nonlinear inertial chain for states removed whole | keeping velocity and bias of keyframes in the period | derivation §3.5, untested |
| Jump residual (§4.5.1) | none, it damps the step only | three rows | derivation §4.5.1 |
| Partial re-centring (§4.5.3) | none, it limits what is applied | none | exact by the plan §2.7.3 |

The derivations point to two independent changes. The information must be kept, and the smallest change that does so is the row partition of §4.1, which preserves the bias anchor whose loss drives the tilt drift. The more complete change is delayed marginalisation restricted to the inertial chain (§4.2), which keeps all inertial information of the period and relinearises it at the current scale and tilt, as DM-VIO does. Independently, the jump at take off should be bounded, either by the projected jump residual of §4.5.1 or by the partial re-centring of §4.5.3, and the block should not be estimated while the vehicle is at rest, where §4.5.1 Step 3 shows the scale carries no information. The row partition of §4.1, the composition of §4.3 as a remedy, and the scale aware recovered factors of §4.4 were not found in the papers read, and their novelty would need a wider search before it is claimed.

## References

[1] L. von Stumberg, V. Usenko, D. Cremers, "Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization", IEEE ICRA 2018, arXiv:1804.05625. Read §III-E and §III-F on 2026-11-06 at https://ar5iv.labs.arxiv.org/html/1804.05625.

[2] L. von Stumberg, D. Cremers, "DM-VIO: Delayed Marginalization Visual-Inertial Odometry", IEEE Robotics and Automation Letters, 2022, arXiv:2201.04114. Read §III-B to §III-F and §IV-A on 2026-11-06 at https://ar5iv.labs.arxiv.org/html/2201.04114. Code read earlier at `github.com/lukasvst/dm-vio` (`plans/gravity_scale_optimisation.md`, reference [3]).

[3] V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers, "Visual-Inertial Mapping with Non-Linear Factor Recovery", IEEE Robotics and Automation Letters, 2020, arXiv:1904.06504. Read §V-B and §V-C, eq. 27 to 35, on 2026-11-06 at https://ar5iv.labs.arxiv.org/html/1904.06504.

[4] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", IEEE Transactions on Robotics, 2017, arXiv:1512.02363. Read the first half, including eq. 33, 34 and 38, on 2026-11-06 at https://ar5iv.labs.arxiv.org/html/1512.02363.

[5] N. Carlevaris-Bianco, M. Kaess, R. M. Eustice, "Generic Node Removal for Factor-Graph SLAM", IEEE Transactions on Robotics, vol. 30, no. 6, pp. 1371 to 1385, 2014. Abstract only, https://www.ri.cmu.edu/publications/generic-node-removal-for-factor-graph-slam.

[6] J. Hsiung, M. Hsiao, E. Westman, R. Valencia, M. Kaess, "Information Sparsification in Visual-Inertial Odometry", IEEE/RSJ IROS 2018, pp. 1146 to 1153. Abstract and search snippets only, https://www.ri.cmu.edu/publications/information-sparsification-in-visual-inertial-odometry.

[7] G. Nützi, S. Weiss, D. Scaramuzza, R. Siegwart, "Fusion of IMU and Vision for Absolute Scale Estimation in Monocular SLAM", Journal of Intelligent and Robotic Systems, vol. 61, 2011. Abstract and search snippets only, including a citing statement that the scale was added "as an additional variable to the state" of an EKF, https://doc.rero.ch/record/322372.

[8] C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, J. D. Tardós, "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM", IEEE Transactions on Robotics, 2021, arXiv:2007.11898. Search snippets only, stating that scale and gravity are refined by inertial only optimisation at about 5 s and 15 s.

[9] M. Mazuran, W. Burgard, G. D. Tipaldi, "Nonlinear Factor Recovery for Long-Term SLAM", The International Journal of Robotics Research, vol. 35, no. 1 to 3, pp. 50 to 72, 2016. Abstract and search snippets only.

## 5. Implementation of §4.1 and §4.5.1 (added 2026-11-06)

`plans/gravity_scale_optimisation_2.md` implements §4.1 as Change A and §4.5.1 as Change B, behind `vio_gs_visual_prior_inertial` and `vio_gs_jump_sigma_rw` / `vio_gs_jump_tilt_rw_deg`, all off by default and on in `data/sitl_config_vo.json`. Three refinements of the proposals were derived there and are recorded here because they change what this file proposed.

1. $M_\text{visual}$ must be independent of the scale only, not of the tilt, because the swap rule tracks the scale alone and the tilt enters the inertial rows linearly through $\mathbf{g}_D$, with a first estimate error of $\frac{1}{2}|\mathbf{g}|\,|\boldsymbol{\delta}_\theta|^2 \Delta t$ in $\mathbf{r}_v$. So a whole inertial factor may enter $M_\text{visual}$ with its scale column zeroed when that column is small. The whitened scale column of a factor over constant acceleration $\mathbf{a}$ has norm $|\mathbf{a}|\sqrt{\Delta t} / \sigma_c$, with $\sigma_c$ the accelerometer noise density, so an admission test $|\mathbf{c}| \le \tau$ admits rest, hover and unaccelerated flight (0.358 m/s² at $\tau = 1$, $\sigma_c = 0.08$, $\Delta t = 0.05$ s) and rejects the take off transient. $M_\text{visual}$ then needs its own tilt anchor, `mpAnchorVisual`, and a reset of $M_\text{half}$ copies it with the current scale, which is free because the copied scale columns are zero.
2. The scale free remainder of a rejected factor is its rotation rows, but not rows 3 to 5 of Basalt's whitened residual. `get_sqrt_cov_inv` is a pivoted LDLT factor of the full $9 \times 9$ covariance, so its rows mix position, rotation and velocity. The correct factor is the marginal likelihood of the rotation measurement, weighted by $\boldsymbol{\Sigma}_{RR}^{-1}$ with $\boldsymbol{\Sigma}_{RR}$ the rotation block of `get_cov()`. This is weaker than the joint block $(\boldsymbol{\Sigma}^{-1})_{RR}$, which would claim information that only the discarded rows justify.
3. At constant attitude, tilt $\boldsymbol{\phi}$ and accelerometer bias $\delta\mathbf{b}_a = \mathbf{R}_0^\top (\mathbf{g}_D \times \boldsymbol{\phi})$ form an exact null direction of every inertial factor, using $\partial\mathbf{r}_v / \partial\mathbf{b}_a = \Delta t\,\mathbf{I}$ from `d_res_d_ba = -d_state_d_ba_`. This is the derivation behind the observed $|\mathbf{b}_{a,xy}| = |\mathbf{g}| \sin(\text{tilt})$, and it is why the bias prior is the only tilt anchor at rest.

The random walk residual is added to $\mathbf{H}$ and $\mathbf{b}$ before the gauge projection, after the zero velocity term, and to all three cost terms. It is not marginalised. Its accumulated increment `gsJump` advances only on accepted steps.

Smoke replays of 2026-11-06 with both changes on (`plans/gravity_scale_optimisation_2.md` §5.5, `/ws/logs/slam/2026-11-06_23-00-46` and `_23-07-49`), of the hop and hover bag `telemetry_40`, removed the tilt divergence on that profile: 0.5° to 1.3° of tilt error and 0.01 to 0.08 m/s² of horizontal bias from 70 s, against 26° and more before. The scale stayed non deterministic, 1.14 in one run and about 3 in the other. Swaps still fire in the take off transient on data driven steps of up to 1.04 in $\sigma$, and the scale is decided there because the cruise carries no scale information. The admission test rejects the ground phase, because the position row's whitened scale column at rest is about 6, driven by millimetre position jitter rather than by acceleration. The kinematic estimate $|\mathbf{a}|\sqrt{\Delta t}/\sigma_c$ of §5 item 1 is therefore a lower bound on real estimates, not a predictor of them. Delayed marginalisation of the inertial chain (§4.2) is the remaining remedy for the scale.

Live `sitl2` straight climbs with both changes on (`_23-14-47`, `_23-18-44`, `_23-21-18`) diverge from lift off, and so did four of seven live runs of the permanent period before the changes, while live mode off runs do not diverge. The pattern is a runaway of one block coordinate through many bounded steps of one sign, the scale in one run and the tilt with no swap in another. The random walk limits the rate and not the total, and no residual acts on the cumulative correction of the correction log, so a permanent block absorbs any persistent take off inconsistency without limit. Evidence and run list in `plans/gravity_scale_optimisation_2.md` §5.5 item 5.

Correction and diagnosis, 2026-11-06. `_23-14-47` and `_23-18-44` are not straight climbs. They hop to 1 m and then climb with 3 m to 5 m of horizontal motion, while `_23-21-18` is a vertical climb to 10 m in 4 s, and all three diverge. Change A as first implemented let the scale of $M_\text{visual}$'s anchor drift with the cumulative correction, which mattered because that scale sets the evaluation point of the visual translation Jacobians (`computeRelPoseGS` scales them). The linearisation scale fell to 0.0044 in `_23-14-47`, a positive feedback on the scale runaway. It now follows the current scale. The trigger common to every live run is earlier. At lift off the window's states already imply accelerations far beyond the true ones (whitened scale columns of 100 to 800 against 2 to 15 on the ground), before any block step, and the block then absorbs the inconsistency (`plans/gravity_scale_optimisation_2.md` §5.5 item 6).

## 6. Wider survey, retaining the initial priors and limiting information loss (added 2026-11-06, derived in full 2026-11-07)

This section surveys methods beyond §3 that keep the priors of the first frame (the position and yaw pins and the bias weights of the initial prior) and the inertial information through marginalisation. It groups them by mechanism, states what each retains, and judges each against Basalt with the gravity and scale block of `plans/gravity_scale_optimisation_2.md`. §6.1 derives which priors the estimator loses today and names each loss, §6.2 collects the identities the later derivations cite and the questions every method is judged by, §6.3 to §6.19 treat one method each, and §6.20 compares them. The methods that keep factors nonlinear for longer are §6.3 to §6.7, those that make the marginal prior relinearisable are §6.8 to §6.12, those that make the prior's linearisation less dependent on the state are §6.13 to §6.16, and those that retain the specific initial priors explicitly are §6.17 to §6.19. Claims are limited to what was read, and the reading of each source is stated in its reference. A derivation that applies a method to Basalt is marked as derived here, and is not taken from the source. References continue the numbering of §References as [10] onwards.

Note, 2026-11-07. This section was rewritten at the user's request, so that every method has its own subsection with a step by step derivation. The summaries of the first version survive inside the subsections. Two were corrected after their papers were read in full, FEJ2 in §6.14 and the robocentric formulation in §6.16, and the synthesis of §6.20 was updated for the removal of the seven dimensional free gauge.

### 6.1 What is lost today

The live runs of 2026-11-06 locate the losses. $M_\text{visual}$ carries a prior on later states only through rows that link them to the departing state. In `/ws/logs/slam/2026-11-06_23-21-18` the landmark database held no landmark before lift off, so on the ground $M_\text{visual}$ had no visual row at all. With Change A on, the yaw pin propagates through the rotation rows and the accelerometer bias anchor through the random walk rows. The position pins do not propagate, because no scale free row involves a position, and the roll and pitch information of the ground phase does not either, because it lives in the scale dependent velocity rows. After the first swap the prior in use therefore holds no position pin. The gauge projector excludes the four directions the initial prior pins (`plans/gravity_scale_optimisation.md` §2.5.4), so those directions are then held by the Levenberg–Marquardt damping alone. §6.1.1 derives the rule behind these statements, §6.1.2 applies it to each prior, and §6.1.3 derives what holds a direction once its prior is gone.

#### 6.1.1 How a prior on a departing variable reaches the variables that stay

Let $\boldsymbol{\delta}_\beta$ be the tangent offset of a departing variable, carrying a prior of information $\mathbf{W}_\beta$, and let the factors that couple it to the variables that stay, $\boldsymbol{\delta}_\alpha$, have the whitened linearised residual $\mathbf{r} + \mathbf{J}_\beta \boldsymbol{\delta}_\beta + \mathbf{J}_\alpha \boldsymbol{\delta}_\alpha$.

Step 1. The cost of the prior and the coupling factors is

$$F = \frac{1}{2}\boldsymbol{\delta}_\beta^\top \mathbf{W}_\beta \boldsymbol{\delta}_\beta + \frac{1}{2}|\mathbf{r} + \mathbf{J}_\beta \boldsymbol{\delta}_\beta + \mathbf{J}_\alpha \boldsymbol{\delta}_\alpha|^2,$$

whose information blocks are

$$\mathbf{H}_{\beta\beta} = \mathbf{W}_\beta + \mathbf{J}_\beta^\top \mathbf{J}_\beta,$$

$$\mathbf{H}_{\beta\alpha} = \mathbf{J}_\beta^\top \mathbf{J}_\alpha,$$

$$\mathbf{H}_{\alpha\alpha} = \mathbf{J}_\alpha^\top \mathbf{J}_\alpha.$$

Step 2. By (M1), removing $\boldsymbol{\delta}_\beta$ leaves the information

$$\mathbf{H}' = \mathbf{J}_\alpha^\top \mathbf{J}_\alpha - \mathbf{J}_\alpha^\top \mathbf{J}_\beta (\mathbf{W}_\beta + \mathbf{J}_\beta^\top \mathbf{J}_\beta)^{-1} \mathbf{J}_\beta^\top \mathbf{J}_\alpha.$$

Step 3. Taking $\mathbf{J}_\alpha^\top$ and $\mathbf{J}_\alpha$ out on both sides,

$$\mathbf{H}' = \mathbf{J}_\alpha^\top \big(\mathbf{I} - \mathbf{J}_\beta (\mathbf{W}_\beta + \mathbf{J}_\beta^\top \mathbf{J}_\beta)^{-1} \mathbf{J}_\beta^\top\big) \mathbf{J}_\alpha.$$

Step 4. By the push through form of (M8) with $\mathbf{J} = \mathbf{J}_\beta$ and $\mathbf{W} = \mathbf{W}_\beta$,

$$\mathbf{H}' = \mathbf{J}_\alpha^\top \big(\mathbf{I} + \mathbf{J}_\beta \mathbf{W}_\beta^{-1} \mathbf{J}_\beta^\top\big)^{-1} \mathbf{J}_\alpha.$$

The marginal is the coupling factor itself with its unit noise covariance raised by $\mathbf{J}_\beta \mathbf{W}_\beta^{-1} \mathbf{J}_\beta^\top$, the covariance of the departing variable pushed through the factor. For one bias coordinate, $\mathbf{J}_\beta = \sqrt{w_1}$, $\mathbf{J}_\alpha = -\sqrt{w_1}$ and $\mathbf{W}_\beta = w_0$ give $H' = \frac{w_1}{(1 + w_1 / w_0)} = \frac{w_0 w_1}{(w_0 + w_1)}$, which is §1.5 Step 2.

Step 5. Let $\mathbf{W}_\beta$ be diagonal, as the initial prior is (`sqrt_keypoint_vio.cpp:105-116`), with entries $w_k$ and with $\mathbf{j}_k$ the $k$-th column of $\mathbf{J}_\beta$. Then

$$\mathbf{J}_\beta \mathbf{W}_\beta^{-1} \mathbf{J}_\beta^\top = \sum_k \frac{1}{w_k}\,\mathbf{j}_k \mathbf{j}_k^\top,$$

and three cases follow. A coordinate whose column $\mathbf{j}_k$ is zero contributes nothing, whatever its prior, so its prior cannot reach $\boldsymbol{\delta}_\alpha$. A pinned coordinate, $w_k \to \infty$, contributes nothing to the inflation, so it acts as a known value. A coordinate without a prior, $w_k \to 0$, inflates the covariance without bound along $\mathbf{j}_k$, and by (M9) the factor then loses every combination of its rows that sees $\mathbf{j}_k$. A pin on a departing coordinate therefore reaches the variables that stay exactly when some coupling factor has a nonzero column at that coordinate and a nonzero column at a remaining variable.

#### 6.1.2 The priors of the first frame, one by one

The rows that $M_\text{visual}$ holds with Change A are the rotation only factor $\mathbf{L}_R \mathbf{r}_R$ (`plans/gravity_scale_optimisation_2.md` §3.3.3), the bias random walk rows, the full rows of admitted factors, of which the ground phase admits almost none at $\tau = 1$ (5 of 84 in run 1 of that plan's §5.5 item 4), and the visual rows, of which `_23-21-18` had none on the ground. The losses are named L1 to L5 for use in every later subsection.

L1, the position pins. Every row above has a zero column at $\delta\mathbf{p}_0$, because $\mathbf{r}_R$ contains rotations and the gyroscope bias only (plan 2 §3.3.2) and the bias rows contain biases only. By §6.1.1 Step 5 the pins reach nothing. In $M_\text{curr}$ the position row has the column $\partial\mathbf{r}_p / \partial\delta\mathbf{p}_0 = -s\,\mathbf{R}_0^\top$ (first plan §2.3.2), so there the pins do propagate, and they are lost at the first reset to $M_\text{visual}$. A visual row would carry them as a relative position constraint, but only once a landmark links the first state to later frames.

L2, the yaw pin. The rotation row has the columns $\mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top$ at $\delta\boldsymbol{\varphi}_0$ and $-\mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top$ at $\delta\boldsymbol{\varphi}_1$ (plan 2 §3.3.2). The yaw of the first state, its world $z$ component of $\delta\boldsymbol{\varphi}_0$, is pinned, while its roll and pitch have no prior (plan 2 §3.2.1). Step 1. Choose the row combination $\mathbf{u}^\top = \mathbf{e}_z^\top \big(\mathbf{J}_r^{-1}(\mathbf{r}_R)\,\mathbf{R}_0^\top\big)^{-1}$, which exists because both factors are invertible. Step 2. Its column at $\delta\boldsymbol{\varphi}_0$ is $\mathbf{e}_z^\top$, which has zero roll and pitch entries, so by §6.1.1 Step 5 it survives the removal of the unconstrained roll and pitch. Step 3. Its column at $\delta\boldsymbol{\varphi}_1$ is $-\mathbf{e}_z^\top$. The surviving combination therefore states that the yaw of state 1 equals the known yaw of state 0 up to the gyroscope noise and bias, and the yaw pin propagates.

L3, the bias anchor. The bias rows are diagonal and per axis, so §1.5 Steps 1 to 3 apply to every axis of both biases. The anchor propagates with variance $\sigma_{b_a,0}^2 + \sigma_{b_a,\text{rw}}^2 T$ (plan 2 §3.3.6 Step 1). Without Change A, §1.5 Step 4 applies and it is lost.

L4, the roll and pitch of the ground phase. At rest the tilt is measured through $\mathbf{g}_D$ in $\mathbf{r}_v$ and $\mathbf{r}_p$, and it trades exactly against the horizontal accelerometer bias (plan 2 §3.2.3). The ground factors are rejected by the admission test, their whitened scale column being about 6 through position jitter (plan 2 §5.5 item 4), so their position and velocity rows never reach $M_\text{visual}$. The initial prior holds nothing on roll and pitch, so after a reset the tilt is held only through the bias anchor L3, to $\sigma_{b_a} / |\mathbf{g}|$, 0.58° at the default bias prior (plan 2 §3.2.3).

L5, the scale information of accelerating factors. It lives in the scale column and in the position and velocity rows of factors with $\mathbf{a} \neq \mathbf{0}$ (plan 2 §3.3.4), which are rejected, so it is lost at every swap (plan 2 §3.3.6 Step 4).

| Loss | $M_\text{curr}$ between swaps | After a swap, Change A off | After a swap, Change A on |
|---|---|---|---|
| L1 position pins | held through $\mathbf{r}_p$ | lost unless a landmark links the first state | lost unless a landmark links the first state |
| L2 yaw pin | held | lost unless a landmark links the first state | held through the rotation rows |
| L3 bias anchor | held | lost | held through the random walk rows |
| L4 ground roll and pitch | held | lost | lost, the tilt is held to 0.58° by L3 only |
| L5 scale of accelerating factors | held, with the error of §1.3 | lost | lost |

Two failure modes are not losses of prior information, but every method is judged against them too. F1 is the kinematic inconsistency of the window at lift off, with whitened scale columns of 100 to 800 before any block step (§5, last paragraph). F2 is the unbounded cumulative correction, in which bounded steps of one sign accumulate without limit (plan 2 §5.5 item 5).

#### 6.1.3 What holds a direction once its prior is gone

Let $\mathbf{n}_t$ be the translation gauge direction along the axis $\mathbf{e}_k$, with $\mathbf{e}_k$ in the position rows of every frame and zero elsewhere (plan 2 Change 10, Step 1). Once the pins are gone, every factor is invariant along it, so $\mathbf{H} \mathbf{n}_t = \mathbf{0}$ and $\mathbf{n}_t^\top \mathbf{b} = 0$. Derived here. Step 1. By (M13) the damped step satisfies $\mathbf{n}_t^\top \mathbf{D}\,\mathbf{x} = 0$. Step 2. Writing out the product over the position rows,

$$\sum_f D_{f,k} x_{f,k} = 0,$$

where $x_{f,k}$ is the $k$-th position entry of frame $f$ in the step and $D_{f,k} = \max(\lambda H_{ii}, \lambda_\text{min})$ its damping. Step 3. The step therefore keeps the damping weighted centroid of the window's positions fixed. Frames tied strongly to the prior have large $H_{ii}$ and move least, so they act as a soft pin, and the window's absolute position drifts by the share of each correction that the weighting assigns to the other frames. The same holds for yaw once its pin is gone. The drift costs nothing in any residual, but it moves the estimator's world relative to the local mapper's world, in which the mapper's accepted pose updates are expressed (first plan §2.9.4). §6.17 returns to this.

### 6.2 Identities and the questions each method answers

The identities (M1) to (M7) of §2 and (N1) to (N5) of `plans/gravity_scale_optimisation_2.md` §3.1.5 are used as before. This section adds the following.

(M8) Woodbury identity. For invertible $\mathbf{A}$ and $\mathbf{C}$ and conforming $\mathbf{U}$, $\mathbf{V}$,

$$(\mathbf{A} + \mathbf{U} \mathbf{C} \mathbf{V})^{-1} = \mathbf{A}^{-1} - \mathbf{A}^{-1} \mathbf{U} (\mathbf{C}^{-1} + \mathbf{V} \mathbf{A}^{-1} \mathbf{U})^{-1} \mathbf{V} \mathbf{A}^{-1}.$$

Proof. Multiply $\mathbf{A} + \mathbf{U} \mathbf{C} \mathbf{V}$ by the right hand side, write $\mathbf{K} = (\mathbf{C}^{-1} + \mathbf{V} \mathbf{A}^{-1} \mathbf{U})^{-1}$, and collect the two terms that end in $\mathbf{K} \mathbf{V} \mathbf{A}^{-1}$,

$$\mathbf{I} + \mathbf{U} \mathbf{C} \mathbf{V} \mathbf{A}^{-1} - \mathbf{U} \mathbf{K} \mathbf{V} \mathbf{A}^{-1} - \mathbf{U} \mathbf{C} \mathbf{V} \mathbf{A}^{-1} \mathbf{U} \mathbf{K} \mathbf{V} \mathbf{A}^{-1} = \mathbf{I} + \mathbf{U} \mathbf{C} \mathbf{V} \mathbf{A}^{-1} - \mathbf{U} \mathbf{C} (\mathbf{C}^{-1} + \mathbf{V} \mathbf{A}^{-1} \mathbf{U}) \mathbf{K} \mathbf{V} \mathbf{A}^{-1},$$

and the last term is $\mathbf{U} \mathbf{C} \mathbf{V} \mathbf{A}^{-1}$ because $(\mathbf{C}^{-1} + \mathbf{V} \mathbf{A}^{-1} \mathbf{U}) \mathbf{K} = \mathbf{I}$, so the product is $\mathbf{I}$. The push through form used in §6.1.1 is the case $\mathbf{A} = \mathbf{I}$, $\mathbf{U} = \mathbf{J}$, $\mathbf{C} = \mathbf{W}^{-1}$, $\mathbf{V} = \mathbf{J}^\top$,

$$(\mathbf{I} + \mathbf{J} \mathbf{W}^{-1} \mathbf{J}^\top)^{-1} = \mathbf{I} - \mathbf{J} (\mathbf{W} + \mathbf{J}^\top \mathbf{J})^{-1} \mathbf{J}^\top.$$

(M9) Removing a nuisance without a prior is a left null space projection. Let $\mathbf{G}$ have full column rank and let $\mathbf{U}$ be an orthonormal basis of its left null space, $\mathbf{U}^\top \mathbf{G} = \mathbf{0}$. Then

$$\lim_{t \to 0}\,(\mathbf{I} + t^{-1} \mathbf{G} \mathbf{G}^\top)^{-1} = \mathbf{U} \mathbf{U}^\top.$$

Proof. By the push through form of (M8) with $\mathbf{J} = \mathbf{G}$ and $\mathbf{W} = t\,\mathbf{I}$, the left hand side before the limit is $\mathbf{I} - \mathbf{G} (t\,\mathbf{I} + \mathbf{G}^\top \mathbf{G})^{-1} \mathbf{G}^\top$, whose limit is $\mathbf{I} - \mathbf{G} (\mathbf{G}^\top \mathbf{G})^{-1} \mathbf{G}^\top$. This is the projector of the first plan §2.5.6 Steps 1 to 6 with $\mathbf{G}$ in place of $\mathbf{N}$, which keeps exactly the part orthogonal to the columns of $\mathbf{G}$, so it equals $\mathbf{U} \mathbf{U}^\top$. Two readings follow. A nuisance $\mathbf{G} \mathbf{w}$ with no prior on $\mathbf{w}$ removes the information of every row combination that sees it, and the same result is obtained by treating $\mathbf{G} \mathbf{w}$ as noise of unbounded variance.

(M10) Loewner order of inverses. For symmetric $\mathbf{A} \succeq \mathbf{B} \succ \mathbf{0}$, $\mathbf{B}^{-1} \succeq \mathbf{A}^{-1}$. Proof. Congruence by $\mathbf{B}^{-1/2}$ preserves the order, so $\mathbf{B}^{-1/2} \mathbf{A} \mathbf{B}^{-1/2} \succeq \mathbf{I}$, whose eigenvalues are therefore at least 1. The inverse $\mathbf{B}^{1/2} \mathbf{A}^{-1} \mathbf{B}^{1/2}$ has the reciprocal eigenvalues, at most 1, so it is $\preceq \mathbf{I}$, and congruence by $\mathbf{B}^{-1/2}$ gives $\mathbf{A}^{-1} \preceq \mathbf{B}^{-1}$.

(M11) A factor evaluated through the metric states is gauge invariant. Let $\mathbf{x}^M$ collect the metric states of the first plan §2.1.2, $\mathbf{R}^M = \mathbf{R}_{wg}^\top \mathbf{R}$, $\mathbf{p}^M = s\,\mathbf{R}_{wg}^\top \mathbf{p}$ and $\mathbf{v}^M = s\,\mathbf{R}_{wg}^\top \mathbf{v}$, and let a factor depend on the estimator's variables only through $\mathbf{x}^M$. Step 1. Along the scale family of the first plan §2.5.6, $s\,\mathbf{p}$ becomes $e^{\sigma - \varepsilon} e^{\varepsilon} \mathbf{p} = s\,\mathbf{p}$ exactly, and likewise for $\mathbf{v}$, so $\mathbf{x}^M$ is unchanged. Step 2. Along the tilt family, the frames become $\mathbf{Q}(\varepsilon) \mathbf{R}$, $\mathbf{Q}(\varepsilon) \mathbf{p}$, $\mathbf{Q}(\varepsilon) \mathbf{v}$ and the direction becomes $\mathbf{Q}(\varepsilon) \mathbf{d}$. Any two rotations that carry $\mathbf{d}$ onto $\hat{\mathbf{g}}$ differ by a rotation about $\hat{\mathbf{g}}$, so the rotation carrying $\mathbf{Q}(\varepsilon) \mathbf{d}$ onto $\hat{\mathbf{g}}$, composed with $\mathbf{Q}(\varepsilon)$, equals the old one up to a rotation about the vertical. $\mathbf{x}^M$ is therefore unchanged up to a yaw rotation of the metric world. Step 3. For a factor that is invariant under yaw, which is every factor except an absolute yaw pin, the factor is constant along both families, and the argument of the first plan §2.5.6 Steps 1 to 4 gives $\mathbf{J} \mathbf{n} = \mathbf{0}$ for all three gauge vectors. Any summary, recovered factor or edge that is written in metric quantities and evaluated through $\mathbf{x}^M$ therefore enters the projected system without spurious information.

(M12) Successive Schur complements equal the joint one (§3.1.2), and adding factors never lowers a marginal in the Loewner order, (N2) of plan 2.

(M13) Damped step along an exact null direction. If $\mathbf{H} \mathbf{n} = \mathbf{0}$ and $\mathbf{n}^\top \mathbf{b} = 0$, the solution of $(\mathbf{H} + \mathbf{D}) \mathbf{x} = \mathbf{b}$ satisfies $\mathbf{n}^\top \mathbf{D} \mathbf{x} = 0$, by multiplying the equation on the left by $\mathbf{n}^\top$ (first plan §2.5.6).

Each method below is judged by the same six questions, so that the subsections can be compared line by line.

1. Q1, what it retains of L1 to L5, and whether exactly, approximately or with overconfidence.
2. Q2, the linearisation error it leaves after a later scale change, against the bilinear error $\Delta s\,\mathbf{R}_0^\top \Delta\mathbf{u}$ of §1.3.
3. Q3, whether everything it adds satisfies $\mathbf{J} \mathbf{N} = \mathbf{0}$ for the projector, whether the re-centring $\Phi_c$ of the first plan §2.7 maps it exactly, and how it interacts with the swap rule.
4. Q4, its interaction with the local mapper and the mapper's pose updates.
5. Q5, whether it addresses F1 or F2.
6. Q6, its cost in Basalt, against the baseline measured in `_23-21-18` (whole frame 6.4 ms median and 8.1 ms at the 90th percentile, `optimize` 5.8 ms, marginalisation of all three priors 0.39 ms and 0.85 ms, one square root marginalisation QR 0.14 ms and 0.26 ms, 75 coordinates in the window with 15 leaving per frame), and the size of the code change.

### 6.3 Delayed marginalisation, DM-VIO [2]

Source and reading. [2] §III-B to §III-F and §IV-A, read 2026-11-06 and summarised in §3.3. A second graph marginalises with a delay of $d$ frames and is readvanced at the current estimate when the scale moves past a threshold. Every factor of the last $d$ frames, the first frame's prior included while it is younger than $d$, is retained and relinearised. DM-VIO also keeps "added priors on the first pose and the gravity direction" in its main graph [2, §III-B] as separate factors.

Formulation, derived here for Basalt. Index the marginalisations by $k$. At step $k$ the estimator removes the variable set $\beta_k$, the departing states, velocities, biases, poses and selected landmarks, together with the factor set $\mathcal{F}_k$ connected to them, the inertial factor of the departing state and the visual factors of the selected landmarks (`context/vio_opt.md` §12.2). Write $\text{marg}_{\beta}(\cdot)$ for the Schur complement of §3.1.1 that removes $\beta$. The main prior follows

$$M_k = \text{marg}_{\beta_k}\big(M_{k-1} + \mathcal{F}_k(\bar{\mathbf{x}}_k)\big),$$

where $\mathcal{F}_k(\bar{\mathbf{x}}_k)$ is the factor set linearised at its first estimates at step $k$. The delayed store keeps, for each $j$ in $k - d < j \le k$, the preintegrated measurement of $\mathcal{F}_j$, its visual rows frozen as linear rows, the set $\beta_j$, the order map and the epoch $e_j$ of the correction log, together with a boundary prior $B_{k-d}$ that holds everything older than $d$. A readvance at step $k$ starts from $R_{k-d} = B_{k-d}$ and repeats, for $j = k - d + 1$ to $k$,

$$R_j = \text{marg}_{\beta_j}\big(R_{j-1} + \mathcal{F}_j(\hat{\mathbf{x}})\big),$$

with every factor linearised at the current estimate $\hat{\mathbf{x}}$ carried to the present epoch by $C_\text{now} C_{e_j}^{-1}$ (first plan §2.9.1). The replacement installs $R_k$ as the new main prior.

Derivation of what it retains. Step 1. By (M12) the result of the loop is the joint marginal,

$$R_k = \text{marg}_{\beta_{k-d+1} \cup \cdots \cup \beta_k}\Big(B_{k-d} + \sum_{j = k-d+1}^{k} \mathcal{F}_j(\hat{\mathbf{x}})\Big),$$

so nothing of the last $d$ steps is selected out, only relinearised. Step 2. The first frame's prior is a factor of $\mathcal{F}_1$. While $k \le d$, the readvance carries its pins through the relinearised position rows, whose column at $\delta\mathbf{p}_0$ is $-s\,\mathbf{R}_0^\top$, so L1, L2 and L3 are kept exactly by §6.1.1. Step 3. The ground phase factors and the accelerating factors are kept with all nine rows, so L4 and L5 are kept within the delay. Step 4. Beyond the delay the boundary prior decides. For Basalt it must itself be valid at every scale, or a replacement would reintroduce the error it removes, so derived here it is advanced as Change A advances $M_\text{visual}$, holding the scale free rows only. What lies more than $d$ frames back is then retained as §6.1.2 lists for Change A on. Step 5. Basalt marginalises a state at every frame, so the delay counts frames, $d = 100$ being 5 s and $d = 200$ being 10 s at 20 Hz. The live runs spent only 4 to 5 s on the ground before lift off (plan 2 §5.5 item 5), so with $d = 200$ the first frame's prior and every ground factor are still in the delayed store throughout the take off transient. DM-VIO's own replacement keeps all visual factors and at least $d - N_f + 1 = 93$ inertial factors and is disabled if more than half of the inertial factors of the previous prior would be lost [2, §III-F], which shows that its boundary prior also does not carry the inertial factors older than the delay.

Q1. L1 to L5 exactly within the delay, by Steps 2 and 3, the first frame's priors through the whole take off transient at $d = 200$, by Step 5, and beyond the delay what Change A keeps, by Step 4.

Q2, consistency. At a replacement every inertial factor is relinearised at the current scale, so the bilinear error of §1.3 is zero. It then grows until the next replacement, which fires when $\delta_s = \frac{\max(s, s_\text{fej})}{\min(s, s_\text{fej})}$ exceeds $\theta_s$ (§3.3.3). Step 1. Between replacements $\delta_s \le \theta_s$, so $|\Delta s| \le (\theta_s - 1)\,s_\text{fej}$. Step 2. Substituting into §1.3,

$$|\mathbf{r}_v - \mathbf{r}_v^\text{lin}| \le (\theta_s - 1)\,s_\text{fej}\,|\Delta\mathbf{u}|.$$

For comparison, dynamic marginalisation bounds the same error through the spread $d_i^2$ of §3.2.4, that is $|\Delta s| / s \le d_i^2 - 1 = 0.1$ at $d_\text{min} = \sqrt{1.1}$. The two bounds are of the same kind. The difference is that a replacement keeps the information, whereas a swap discards it, so no ratchet of §1.4 can form, and by (M12) the replaced prior holds at least the information of a prior reset to the boundary.

Q3, gauge, re-centring and swaps. Every relinearised factor is a `residualGS` inertial factor or a visual factor evaluated through the metric states, so by (M11) it satisfies $\mathbf{J} \mathbf{N} = \mathbf{0}$ at the point of the readvance. The kept variables of $R_k$, the block, the keyframe poses and the oldest state, must be linearised at the points at which the window's other factors evaluate them, so after a replacement those variables are re-frozen at their current values and the block's anchor is set to its current value. Stored nonlinear factors are invariant under $\Phi_c$ (first plan §2.7.2) and frozen visual rows map exactly by $\mathbf{A}^{-1}$ (first plan §2.7.3), so each store entry is mapped once at readvance through its epoch instead of at every frame. The three priors and the swap rule are replaced by one prior and the replacement rule.

Q4, mapper. Packets are still built from the main prior, so the mapper's contract is unchanged. A readvance evaluates the stored inertial factors at the window's first estimates, which accepted mapper pose updates overwrite with the poses of the visual only mapper (plan 2 §3.6 item 8), so it imports the mismatch between the mapper's geometry and the preintegration into the inertial rows of the new prior instead of removing it. The replays of 2026-11-10 measured whitened prior costs of 1.7 to 7.2 for a readvance against 0.0004 to 0.004 for the main prior on the ground, with keyframe pose updates of 1 mm to 18 mm against a preintegrated position standard deviation of about 0.5 mm, and the scale collapsed in every replay with replacements and pose updates both active. The mapper's pose updates must therefore be held while the delayed store is active (`plans/delayed_marginalisation.md` §2.3.8 and §3.4.3, §7 below). Note, 2026-11-10. This paragraph previously said that a readvance removes the mismatch left by accepted mapper pose updates, which the replays contradicted.

Q5. Neither F1 nor F2. Delayed marginalisation keeps information but does not make new information consistent, and nothing in it bounds the cumulative correction.

Q6, cost. Per frame, one more QR advances the boundary and a step is stored, while the linearisations of $M_\text{half}$ and $M_\text{visual}$ are dropped, about 0 to 0.15 ms net, against 0.8 % of the keyframe thread, 0.44 ms, that [2, §IV-A] reports. Per replacement, $d$ successive QRs of 0.14 ms to 0.26 ms, which is 14 ms to 26 ms at $d = 100$ and 28 ms to 52 ms at $d = 200$, a spike of two to eight frames. Memory is at most $75 \times 75$ doubles, 45 KB, per step after QR compression, 4.5 MB at $d = 100$ and 9 MB at $d = 200$. The spike can be moved to a worker thread at the price of one frame of latency, or rate limited. The change is a new store class, a rewritten `marginalize` path, keys and logging, several hundred lines (plan 2 §3.6 item 1). These figures are estimates scaled from measured QR times, not profiled.

### 6.4 Incremental smoothing without marginalisation, iSAM2 [10]

Source and reading. [10] is cited from memory, except for the title of its conference version, "iSAM2: Incremental smoothing and mapping with fluid relinearization and incremental variable reordering", confirmed in the reference list of [11]. The whole history stays in a Bayes tree, and variables are relinearised selectively when their estimate moves past a threshold ("fluid relinearisation"). Nothing is lost, and the initial priors stay exact factors, at a cost that grows with the history. GTSAM's fixed lag smoother built on it marginalises into linear container factors and has Basalt's problem again.

Formulation. Every factor of the period is kept nonlinear. A variable is relinearised when its offset from its linearisation point exceeds a threshold, and only the parts of the factorisation that depend on it are recomputed.

Derivation of what it retains. Step 1. No factor is marginalised, so by construction L1 to L5 are kept as the original factors. Step 2. Derived here, the block becomes unnecessary. The first plan introduced it because a frozen prior cannot follow a correction of the scale, which then accumulates in offsets from first estimates that never move (first plan §2.5.4 item 4, root cause R5). Without a frozen prior every first estimate moves, so the fixation of the block at $s = 1$ and $\mathbf{g}_D = \mathbf{g}$ is admissible, being the signed permutation with $\mathbf{S}_{gs} \mathbf{N}$ of rank 3 and perfect conditioning of that plan's §2.5.4 item 4. A full smoother in Basalt would carry the scale and the tilt in the states, as today's estimator does, and would need no projector.

Q1. Everything, as the original factors, by Step 1, the first frame's pins included.

Q2, consistency. Step 1. With a relinearisation threshold on the states, every factor's linearisation point stays within the threshold of the current estimate, so every linearisation error is bounded by the threshold rather than by the history of the scale. Step 2. Derived here, a correction of the scale defeats the selectivity. A log scale correction $\Delta\sigma$ moves every position by about $\Delta\sigma\,\mathbf{p}_f$ (first plan §2.10), so every frame farther from the origin than the threshold divided by $|\Delta\sigma|$ is relinearised, which in the take off transient is nearly every frame, and the update degenerates to a batch solve over the history.

Q3. With the block fixed there are no new gauge directions, and the four of today's gauge stay pinned by the initial prior as exact factors. No re-centring and no swap are needed.

Q4. The mapper's pose updates would become redundant inside the period, since the smoother is itself a full bundle adjustment, but the mapper's map and the smoother's estimate would have to be reconciled.

Q5. A batch solve over the whole history after lift off can revise the ground phase states with all later data, which is the only mechanism in this survey that could repair a kinematic inconsistency of the window after the fact, F1. This is an expectation, not a result. F2 does not arise in the same form, because no block accumulates a correction.

Q6. The cost of one relinearisation of the history is that of a batch solve over the period, about 200 states of 15 coordinates and their landmarks for 10 s at 20 Hz, and grows without bound for the 7000 s period of the SITL configuration. It has not been measured for Basalt. Adopting it would replace the estimator.

### 6.5 Concurrent filtering and smoothing [11]

Source and reading. [11] abstract, and §5.1 and §5.3 read on 2026-11-07. A low latency filter on recent states runs beside a high latency smoother on all past states, both "sub-operations of a single optimization" on one factor graph, and the concurrent system "is shown to recover the full batch solution" while producing real time estimates. For Basalt the smoother would hold the first frame's prior and every inertial factor nonlinearly and periodically return a summary to the window, which is the structure of the local mapper with inertial factors added.

Formulation, from [11]. The joint density is factorised over the filter states $\Theta_F$, a small set of separator states $\Theta_\text{sep}$ and the smoother states $\Theta_S$ [11, eq. 7],

$$p(\Theta) = p(\Theta_S \mid \Theta_\text{sep})\,p(\Theta_\text{sep})\,p(\Theta_F \mid \Theta_\text{sep}).$$

Each branch holds its own measurements and a cached marginal factor on the separator from the other branch, $\tilde{p}(Z_S \mid \Theta_\text{sep})$ in the filter and $\tilde{p}(Z_F \mid \Theta_\text{sep})$ in the smoother [11, eq. 15]. Synchronisation exchanges the updated marginals, after which the product of the two branches equals the joint density of all measurements, so "the optimal MAP estimate is recovered" [11, eq. 17], and since the marginals are produced by elimination anyway, synchronisation is $O(1)$ [11, §5.3].

Derivation of what it retains, derived here for Basalt. Step 1. Let the filter be the VIO window, the separator the variables of the window's prior, the keyframe poses and the oldest state, and the smoother the departed states with their inertial factors, the first frame's prior and the mapper's keyframes and landmarks. Step 2. Every factor is kept in one branch, nonlinearly, so L1 to L5 are retained, and the smoother relinearises its own factors at its own estimate. Step 3. The filter's prior is replaced at each synchronisation by the smoother's marginal on the separator, which by (M1) is a linear factor built at the smoother's current estimate. Step 4. That marginal must be gauge invariant in the filter. Derived here, if the smoother works in the metric world with its block fixed, as §6.4 Step 2 allows, its marginal is a factor of metric states. Evaluated through $\mathbf{x}^M$, its Jacobian with respect to the filter's variables follows by the chain rule, for instance the column of the log scale is the metric Jacobian times $\partial\mathbf{p}^M / \partial\sigma = \mathbf{p}^M$ for each position, so it acquires block columns automatically and satisfies $\mathbf{J} \mathbf{N} = \mathbf{0}$ by (M11). Applied directly to the frames in $D$ without that map, it would be a gauge prior holding $D = M$, which is today's estimator.

Q1. Everything, nonlinearly in the smoother, by Step 2, and in the filter as a summary that is rebuilt at each synchronisation.

Q2. The summary is linearised at the smoother's estimate of the last synchronisation, so its error after a scale change is that of §1.3 with $\Delta s$ the scale change since that synchronisation, bounded by the mapper cycle rather than by the history.

Q3. The summary enters the projector cleanly by Step 4. It is replaced, not re-centred, at each synchronisation, and between synchronisations it maps by $\mathbf{J} \mathbf{M}^{-1}$ like any prior (first plan §2.7.3). The swap rule is no longer needed, since the summary is renewed from nonlinear factors.

Q4. This is the principled form of the mapper's pose feedback. Derived here, today the accepted pose updates overwrite a frozen keyframe's linearisation point without re-expressing the priors built at the old point (plan 2 §3.6 item 8). Step 1. A prior built at $\bar{\mathbf{x}}_\text{old}$ satisfies $\mathbf{J}_m \mathbf{n}(\bar{\mathbf{x}}_\text{old}) = \mathbf{0}$. Step 2. After the overwrite, $\mathbf{N}$ is built at $\bar{\mathbf{x}}_\text{new}$, and the scale vector's position rows change by the overwrite $\Delta\mathbf{p}_f$ (first plan §2.5.6, rows $c$ to $c + 2$ hold $\mathbf{p}_f$). Step 3. Hence $\mathbf{J}_m \mathbf{n}(\bar{\mathbf{x}}_\text{new}) = \mathbf{J}_m \big(\mathbf{n}(\bar{\mathbf{x}}_\text{new}) - \mathbf{n}(\bar{\mathbf{x}}_\text{old})\big)$, which is nonzero and of the size of the overwrite, bounded by `kRelinThresholdTrans` and `kRelinThresholdRot`, so the prior appears to observe the gauge (first plan §2.5.2 consequence 3). Concurrent smoothing replaces the overwrite by a new marginal built consistently at one point, which removes this source.

Q5. The smoother relinearises the take off history with later data, so it could partly repair F1, under the same caveat as §6.4 Q5. F2 is not addressed in the filter.

Q6. A background thread, which Basalt already has in the local mapper. The change is large. The mapper's `NfrMapper` holds keyframe poses, landmarks, relative pose and roll pitch factors (first plan §2.9.2), and would need the departed states' velocities and biases, inertial factors composed between keyframes by §3.5.2, the separator bookkeeping and the marginal exchange. The synchronisation itself is cheap by [11, §5.3].

### 6.6 Fixation instead of marginalisation, ORB-SLAM3 [8] and OKVIS2 [12]

Source and reading. Old states are held fixed rather than marginalised, so the factors that connect them to the window stay nonlinear and are relinearised at every iteration. OKVIS2 states that "old poses and speed/bias variables are fixed ... to keep the problem realtime capable" [12, read via ar5iv], keeping $A = \max(A_\text{min}, A_{\Delta T})$ states variable, with $A_{\Delta T}$ the number that covers the last $\Delta T$ seconds [12, §V-D2, read 2026-11-07]. With the per frame re-centring of the first plan, fixed states would be mapped by the same similarity, so the boundary factor sees the corrected scale and tilt. A fixed state is treated as known, which overstates confidence, and factors between two fixed states inform nothing. Fixation therefore keeps anchors, not information.

Derivation of the overconfidence. Step 1. Fixation of $\boldsymbol{\delta}_\beta$ keeps only the rows of the boundary factors with $\boldsymbol{\delta}_\beta = \mathbf{0}$, so the information on the window is $\mathbf{J}_\alpha^\top \mathbf{J}_\alpha$, which is §6.1.1 Step 4 in the limit $\mathbf{W}_\beta^{-1} \to \mathbf{0}$. Step 2. Marginalisation gives $\mathbf{J}_\alpha^\top (\mathbf{I} + \mathbf{J}_\beta \mathbf{W}_\beta^{-1} \mathbf{J}_\beta^\top)^{-1} \mathbf{J}_\alpha$. Step 3. Since $\mathbf{I} + \mathbf{J}_\beta \mathbf{W}_\beta^{-1} \mathbf{J}_\beta^\top \succeq \mathbf{I}$, (M10) gives $(\mathbf{I} + \mathbf{J}_\beta \mathbf{W}_\beta^{-1} \mathbf{J}_\beta^\top)^{-1} \preceq \mathbf{I}$, so the fixed information exceeds the marginal by

$$\mathbf{J}_\alpha^\top \big(\mathbf{I} - (\mathbf{I} + \mathbf{J}_\beta \mathbf{W}_\beta^{-1} \mathbf{J}_\beta^\top)^{-1}\big) \mathbf{J}_\alpha \succeq \mathbf{0}.$$

Step 4, the size at take off. For the accelerometer bias, $w_0 = 1 / 0.1^2 = 100$ from the default prior and $w_1 = 1 / (\sigma_{b_a,\text{rw}}^2 \Delta t) = 1 / (0.001132^2 \cdot 0.05) \approx 1.6 \cdot 10^7$ from one random walk step. Marginalisation gives $\frac{w_0 w_1}{w_0 + w_1} \approx 100$ on the next bias, and fixation gives $w_1$, about $1.6 \cdot 10^5$ times more, so a fixed state with a weakly known bias pins the window's bias to the random walk step of 0.25 mm/s². Fixation is sound only for states that are already well determined, as OKVIS2's old states are.

Derivation of its interaction with the block, derived here. Step 1. A fixed state has no rows in $\mathbf{N}$, so a boundary factor between a fixed state and the window is no longer invariant along the gauge vectors restricted to the window. Fixation is therefore a gauge fixation in the sense of the first plan §2.5.4, with $\mathbf{S}$ selecting the fixed coordinates. Step 2. The rows of $\mathbf{S} \mathbf{N}$ at a fixed frame are $\mathbf{p}_f$ and $\mathbf{v}_f$ for the scale and $\mathbf{a} \times \mathbf{p}_f$, $\mathbf{a}$ and $\mathbf{a} \times \mathbf{v}_f$ for the tilt. The rotation rows $\mathbf{a}$ fix both tilt directions with perfect conditioning, as that plan's item 2 does, while the scale rows are of the size of the fixed frames' positions and velocities, which at take off are near zero, so the scale direction is held only weakly, as in item 3. Step 3. The fixed directions stop being null directions of $\mathbf{H}$, so they must be removed from $\mathbf{N}$, or the projector deletes real constraints (first plan §2.5.4, last paragraph). Fixation and the block's free gauge therefore conflict, and fixation decides the scale gauge exactly where it is least well conditioned.

Q1. L1 to L3 are kept exactly while the first state is fixed, but with the overconfidence of Step 4 on the bias. L4 is kept through the fixed attitudes, again as if known. L5 is kept only through boundary factors, and factors between two fixed states inform nothing.

Q2. Boundary factors are relinearised at every iteration, but the fixed states are not re-estimated, so their relative geometry is frozen at what the window estimated when they left, re-centred thereafter.

Q3 to Q5. Conflicts with the projector by the derivation above. The mapper is unaffected. Neither F1 nor F2, and the fixed take off states would hold the inconsistency of F1 as known values.

Q6. Small per frame, the fixed states being constants, but each re-centring must map every fixed state, which grows with the history unless fixed states are dropped after $\Delta T$.

### 6.7 Dropping visual rows but keeping the inertial chain, VINS-Mono [13]

Source and reading. [13, §VI-D], read 2026-11-06. When the second newest frame is not a keyframe, its visual measurements are discarded and its inertial measurements are kept by integrating across it. OKVIS2 does the same for a non keyframe, whose inertial measurements are appended to the previous inertial error by preintegration with no Schur complement [12, §V-D, read 2026-11-07]. Basalt could do the same for non keyframe states in the period, so that their inertial information is never frozen into a prior. This is a special case of the composition of §3.5.2.

What Basalt does today. A departing state that is not a keyframe is removed whole (`states_to_marg_all`, `sqrt_keypoint_vio.cpp:1123-1133`). Its observations of landmarks that are neither hosted by a departing keyframe nor lost are removed without being marginalised (`lmdb.removeKeyframes`, `:1501`, and `context/vio_opt.md` §12.2), so Basalt already discards most of a non keyframe's visual information, as VINS-Mono does. Its inertial factor to `last_state_to_marg`, however, is linearised into the prior at the scale of the moment. The SITL window holds three states and seven keyframes (`data/sitl_config_vo.json`, `vio_max_states` and `vio_max_kfs`), so this happens at every frame that is not a keyframe.

Formulation, derived here. Let state $i$ be the newest keyframe, which keeps its velocity and biases, nine more coordinates, as long as it anchors the chain, and let $j$ be a non keyframe state between $i$ and $k$. Removing $j$ replaces the two inertial factors $(i, j)$ and $(j, k)$ by one factor $(i, k)$ composed by §3.5.2 and §3.5.3, and the two random walk factors by one. At the next keyframe the composed factor is marginalised once, as today.

Derivation of exactness. Step 1, the measurement. By §3.5.2 the composed increments are exact functions of the two measurements, and by §3.5.3 the noise composes to first order through $\boldsymbol{\Sigma}_{ik} = \mathbf{A}\,\boldsymbol{\Sigma}_{ij}\,\mathbf{A}^\top + \mathbf{B}\,\boldsymbol{\Sigma}_{jk}\,\mathbf{B}^\top$. Step 2, the bias random walk. Minimising $\frac{1}{2} w_{ij} (b_i - b_j)^2 + \frac{1}{2} w_{jk} (b_j - b_k)^2$ over $b_j$ sets the derivative $w_{ij}(b_j - b_i) + w_{jk}(b_j - b_k)$ to zero, so

$$b_j^* = \frac{w_{ij} b_i + w_{jk} b_k}{w_{ij} + w_{jk}},$$

and substituting $b_i - b_j^* = \frac{w_{jk}(b_i - b_k)}{w_{ij} + w_{jk}}$ and $b_j^* - b_k = \frac{w_{ij}(b_i - b_k)}{w_{ij} + w_{jk}}$,

$$\min_{b_j} = \frac{1}{2}\,\frac{w_{ij} w_{jk}}{w_{ij} + w_{jk}}\,(b_i - b_k)^2.$$

With $w = 1 / (\sigma_\text{rw}^2 \Delta t)$ the composed weight is $1 / (\sigma_\text{rw}^2 (\Delta t_{ij} + \Delta t_{jk}))$, the random walk weight of the composed interval, exactly. Step 3, the bias inside the second segment. The composed measurement evaluates the second segment's bias correction at $b_i$ instead of $b_j$. The difference $b_j - b_i$ has standard deviation $\sigma_{b_a,\text{rw}} \sqrt{\Delta t_{ij}}$, and by (N4) it changes the velocity increment by about $\Delta t_{jk}$ times that, $0.05 \cdot 0.001132 \cdot \sqrt{0.05} = 1.3 \cdot 10^{-5}$ m/s for 50 ms segments, against a velocity standard deviation of 0.018 m/s, so the approximation is negligible.

Q1. Within the span since the newest keyframe, L3, L4 and L5 are kept nonlinearly, and so are L1 and L2 while the first keyframe anchors the chain. Beyond that span the composed factor is marginalised as today.

Q2. The composed factor is a `residualGS` factor, relinearised at the current scale at every iteration, so it has no first estimate error until it is marginalised.

Q3. Gauge invariant like every inertial factor. It is a nonlinear factor of the window, so the re-centring needs no special treatment, and it is outside every prior until its keyframe, so no swap can discard it.

Q4 and Q5. The mapper is unaffected. Neither F1 nor F2.

Q6. Nine coordinates more in the window per anchoring keyframe, one composition per removed state, and a change to the removal of `states_to_marg_all`. Its benefit is bounded by the keyframe rate, which is not measured here, and it overlaps with §6.3, of which it is the special case of a delay of one keyframe interval restricted to the inertial chain.

### 6.8 Non-linear factor recovery, with the scale aware variant [3, 9]

Source and reading. [3, §V-B and §V-C] read 2026-11-06 and derived in §3.4, and the scale aware variant proposed in §4.4. This subsection applies recovery inside the VIO window, in place of the dense prior, rather than only in the mapper.

Formulation, derived here. Recover from the dense prior over the block, the keyframe poses and the oldest state a set of nonlinear factors written in metric quantities, so that by (M11) each is gauge invariant.

1. Relative pose factors, $\mathbf{r}_\text{rel} = \text{Log}\big(\mathbf{z}_\text{rel}\,\mathcal{S}_s(\mathbf{T}_j^{-1} \mathbf{T}_i)\big)$, with $\mathbf{z}_\text{rel}$ the metric relative pose at recovery. By (S1) the argument equals $\mathbf{z}_\text{rel}\,\mathcal{S}_s(\mathbf{T}_j)^{-1} \mathcal{S}_s(\mathbf{T}_i)$, and along the scale family $\mathcal{S}_s(\mathbf{T})$ is unchanged, along the tilt family $\mathbf{T}_j^{-1} \mathbf{T}_i$ is unchanged.
2. Gravity aware roll pitch factors. The metric attitude is $\mathbf{R}^M_i = \mathbf{R}_{wg}^\top \mathbf{R}_i$ and $\mathbf{R}_{wg} \hat{\mathbf{g}} = \mathbf{d}$ (first plan §2.1.1), so the body frame gravity direction is $(\mathbf{R}^M_i)^\top \hat{\mathbf{g}} = \mathbf{R}_i^\top \mathbf{R}_{wg} \hat{\mathbf{g}} = \mathbf{R}_i^\top \mathbf{d}$. The factor of [3, eq. 32] becomes $\mathbf{r}_\text{rp} = \lfloor \mathbf{z}_\text{rp}\,\mathbf{R}_i^\top \mathbf{d} \rfloor_{xy}$ with the sign convention of [3]. Along the tilt family $\mathbf{R}_i^\top \mathbf{Q}^\top \mathbf{Q} \mathbf{d} = \mathbf{R}_i^\top \mathbf{d}$, and it contains no $s$.
3. For the oldest state, a body frame velocity factor on $s\,\mathbf{R}_i^\top \mathbf{v}_i$, invariant along both families by the same arguments, and absolute bias factors.

The informations follow the closed form of §3.4.2 Step 5, $\mathbf{H}_i = (\{\mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top\}_i)^{-1}$.

Derivation of the covariance under a free gauge, derived here. Step 1. The dense prior with the block satisfies $\mathbf{H} \mathbf{N} = \mathbf{0}$, so $\boldsymbol{\Sigma}_o = \mathbf{H}^{-1}$ does not exist, and by plan 2 §3.2.7 even the block's own marginal information is zero. Step 2. Every recovered residual is gauge invariant, $\mathbf{J}_r \mathbf{N} = \mathbf{0}$, so $\mathbf{J}_r \mathbf{P} = \mathbf{J}_r$ with the projector $\mathbf{P}$ of plan 2 §3.1.3. Step 3. Let $\boldsymbol{\Sigma}_1$ and $\boldsymbol{\Sigma}_2$ be the covariances of two gauge choices, that is two generalised inverses of $\mathbf{H}$ that agree on the orthogonal complement of $\mathbf{N}$, so that $\mathbf{P} \boldsymbol{\Sigma}_1 \mathbf{P} = \mathbf{P} \boldsymbol{\Sigma}_2 \mathbf{P}$. Then

$$\mathbf{J}_r \boldsymbol{\Sigma}_1 \mathbf{J}_r^\top = \mathbf{J}_r \mathbf{P} \boldsymbol{\Sigma}_1 \mathbf{P} \mathbf{J}_r^\top = \mathbf{J}_r \mathbf{P} \boldsymbol{\Sigma}_2 \mathbf{P} \mathbf{J}_r^\top = \mathbf{J}_r \boldsymbol{\Sigma}_2 \mathbf{J}_r^\top,$$

so the covariance $\mathbf{S} = \mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top$ of the recovered measurements is well defined with $\boldsymbol{\Sigma}_o = \mathbf{P} \mathbf{H}^{+} \mathbf{P}$, whatever the gauge. This is the exact form of the observation of Zhang et al. that gauge invariant quantities have gauge independent covariances (plan 2 §2.8).

Derivation of the information lost by recovery, derived here. Step 1. At the optimum of §3.4.2 with a square $\mathbf{J}_r$, $\mathbf{H}_i = \mathbf{S}_{ii}^{-1}$, so the trace term is $\sum_i \text{tr}(\mathbf{S}_{ii}^{-1} \mathbf{S}_{ii}) = \sum_i d_i = d$. Step 2. The log determinant of §3.4.2 Step 3 becomes $-\sum_i \ln\,\text{det}\mathbf{S}_{ii} + \ln\,\text{det}\mathbf{S}$. Step 3. Substituting both into the divergence of §3.4.2 Step 1,

$$D_\text{KL}^* = \frac{1}{2}\Big(d - \big({-\sum_i \ln\,\text{det}\mathbf{S}_{ii}} + \ln\,\text{det}\mathbf{S}\big) - d\Big) = \frac{1}{2}\Big(\sum_i \ln\,\text{det}\mathbf{S}_{ii} - \ln\,\text{det}\mathbf{S}\Big).$$

It vanishes exactly when $\mathbf{S}$ is block diagonal, that is when the chosen measurements are uncorrelated under the marginal, and it is positive otherwise because it is a divergence. It is computable at every marginalisation from quantities the recovery forms anyway, so it measures what a chosen topology discards.

What recovery repairs and what it does not, derived here. Step 1. A recovered factor is nonlinear, so after a scale change its Jacobian follows the scale through $\mathcal{S}_s$, and by (S2) its information on translations transforms as $\mathbf{D}_s^{-1} \mathbf{H} \mathbf{D}_s^{-1}$ automatically. The later linearisation error of §1.3 is removed. Step 2. Its pseudo measurement is the marginal mean at recovery, $\mathbf{r}_i(\boldsymbol{\mu}_o, \mathbf{z}_i) = \mathbf{0}$ (§3.4.2 Step 1), not the original inertial measurement. If the original factors were linearised at a wrong scale when they entered the prior, both $\boldsymbol{\mu}_o$ and $\boldsymbol{\Sigma}_o$ carry that error, and recovery preserves it. Recovery therefore repairs the error of later relinearisation but not the error of the first, whereas delayed marginalisation repairs both within its delay, because it keeps the original measurements.

Q1. L1 to L3 are kept as far as the dense prior holds them, and as relinearisable factors. L4 is kept through the gravity aware roll pitch factor. L5 is kept as metric relative translations, with the caveat of the last derivation.

Q2. No later linearisation error from the scale, by Step 1 above.

Q3. Gauge invariant by construction and (M11). Invariant under $\Phi_c$ because each residual depends on the metric states only. A recovered prior can be relinearised at any scale, so the three priors and the swap rule could be replaced by recovery at every marginalisation.

Q4. The mapper already recovers factors of this kind from packets (§3.4.3), so the window and the mapper would share one factor type, and the mapper's update of the first plan §2.9.2 item 3 becomes unnecessary for factors written with $\mathcal{S}_s$.

Q5. Neither F1 nor F2.

Q6. One recovery per marginalisation, which needs $\mathbf{S} = \mathbf{J}_r \boldsymbol{\Sigma}_o \mathbf{J}_r^\top$ and therefore a factorisation of the prior, of the order of one dense $75 \times 75$ inversion per frame, plus a new factor type in the window's linearisation. Not measured. The closed form is KL optimal only for a square $\mathbf{J}_r$, and with fewer factors than prior dimensions the informations must be found iteratively, which is the subject of §6.10.

### 6.9 Relative pose errors from marginalised observations, OKVIS2 [12]

Source and reading. [12, §V-D1, eq. 10 to 20] read via ar5iv on 2026-11-06 and 2026-11-07. Frames with the least covisibility become pose graph nodes "by marginalisation of common observations under insertion of relative pose error terms", with weight $\mathbf{W}_p = \mathbf{H}^*$, the reduced Hessian over the landmarks, and a constant term $\mathbf{e}_{p,0} = -\mathbf{H}^{*+} \mathbf{b}^*$. On loop closure the edges are "revived" into landmarks and observations. Pre-integrated inertial errors are kept between all successive frames rather than turned into edges. The paper names no prior on the first frame or on gravity.

Formulation, from [12]. For a reference frame $r$ and a frame $c$, the reprojection errors of the landmarks seen in both, expressed in frame $r$, are collected [12, eq. 10], linearised in the relative pose $\mathbf{p} = [{}_{S_r}\mathbf{r}_{S_c}, \mathbf{q}_{S_r S_c}]$ and the landmarks, and the landmarks are eliminated [12, eq. 14 and 15],

$$\mathbf{H}^* = \mathbf{H}_{pp} - \sum_j \mathbf{H}_{pj} \mathbf{H}_{jj}^{+} \mathbf{H}_{pj}^\top,$$

$$\mathbf{b}^* = \mathbf{b}_p - \sum_j \mathbf{H}_{pj} \mathbf{H}_{jj}^{+} \mathbf{b}_j.$$

The edge is the error term [12, eq. 12]

$$\mathbf{e}_p = \mathbf{e}_{p,0} + \begin{bmatrix}{}_{S_r}\mathbf{r}_{S_c} - {}_{S_r}\tilde{\mathbf{r}}_{S_c} \\ \mathbf{q}_{S_r S_c} \boxminus \tilde{\mathbf{q}}_{S_r S_c}\end{bmatrix},$$

with the cost $\frac{1}{2}\mathbf{e}_p^\top \mathbf{W}_p \mathbf{e}_p$ and the tilde quantities the relative pose at creation.

Derivation of the identification. Step 1. At the creation point the bracket of $\mathbf{e}_p$ is the tangent offset $\delta\mathbf{p}$ and its Jacobian is the identity [12], so the cost is

$$\frac{1}{2}(\mathbf{e}_{p,0} + \delta\mathbf{p})^\top \mathbf{W}_p (\mathbf{e}_{p,0} + \delta\mathbf{p}).$$

Step 2. Its minimiser satisfies $\mathbf{W}_p(\mathbf{e}_{p,0} + \delta\mathbf{p}) = \mathbf{0}$, so on the range of $\mathbf{W}_p$ it is $\delta\mathbf{p} = -\mathbf{e}_{p,0}$. Step 3. The reduced system it must reproduce is $\mathbf{H}^* \delta\mathbf{p} = \mathbf{b}^*$ in the sign convention of [12, eq. 16], with the solution $\delta\mathbf{p} = \mathbf{H}^{*+} \mathbf{b}^*$ and curvature $\mathbf{H}^*$. Step 4. Matching curvature and minimiser gives $\mathbf{W}_p = \mathbf{H}^*$ and $\mathbf{e}_{p,0} = -\mathbf{H}^{*+} \mathbf{b}^*$, which are [12, eq. 19 and 20]. The edge is the visual marginal of the pair, made nonlinear in the relative pose. This is NFR restricted to the visual part, which is exactly the part that is independent of the scale.

Derivation of its scale independence, derived here. Step 1. For a monocular camera at the IMU, scaling the relative translation and every landmark's distance together leaves every projection unchanged, so $\mathbf{H}^*$ has the null vector $\mathbf{m} = [\tilde{\mathbf{r}}; \mathbf{0}]$, the relative translation direction with zero rotation, and $\mathbf{W}_p \mathbf{m} = \mathbf{0}$. Step 2. A later scale change $e^{\Delta\sigma}$ of the relative translation, with the rotation unchanged, changes the bracket by $(e^{\Delta\sigma} - 1)\,\tilde{\mathbf{r}} = c\,\mathbf{m}$ with $c = e^{\Delta\sigma} - 1$. Step 3. Expanding the cost,

$$\frac{1}{2}(\mathbf{e}_p + c\,\mathbf{m})^\top \mathbf{W}_p (\mathbf{e}_p + c\,\mathbf{m}) = \frac{1}{2}\mathbf{e}_p^\top \mathbf{W}_p \mathbf{e}_p + c\,\mathbf{m}^\top \mathbf{W}_p \mathbf{e}_p + \frac{1}{2}c^2\,\mathbf{m}^\top \mathbf{W}_p \mathbf{m},$$

and both added terms vanish by $\mathbf{W}_p \mathbf{m} = \mathbf{0}$, so the edge is unchanged for any $\Delta\sigma$, exactly and not only to first order. With a lever arm, as for the SITL camera 0.5 m ahead of the IMU (`data/sitl_calib.json`, `T_imu_cam`), $\mathbf{m}$ is only an approximate null vector, the edge holds the weak metric information of the first plan §2.2.4, and that part has the small first estimate error that $M_\text{visual}$ already accepts for its zeroed lever arm column.

Q1. In OKVIS2 the nodes of the pose graph are kept, so a first frame that becomes a node keeps its pins, L1 and L2, as long as it is pinned. L3 to L5 are inertial and are kept by keeping the inertial errors between successive frames and fixing old states (§6.6), not by the edges.

Q2. The visual part has no scale error, by the derivation above. The inertial part has whatever its treatment has.

Q3. Written in metric relative pose through $\mathcal{S}_s$, the edge is gauge invariant by (M11) and invariant under $\Phi_c$. It could replace the visual rows of $M_\text{visual}$ by nonlinear edges that need no zeroing of the scale column.

Q4. The edges are the factors the mapper would keep in any case, and reviving them into observations at loop closure has no counterpart in Basalt today.

Q5. Neither F1 nor F2.

Q6. One landmark elimination per edge, which Basalt already performs per landmark in the marginalisation QR (`context/vio_opt.md` §7.3), and a growing set of pose graph nodes, which Basalt does not keep in the window. The inertial part would need a separate choice, §6.3, §6.6 or §6.7.

### 6.10 Generic linear constraints [5] and factor descent [14]

Source and reading. Abstracts and search snippets of [5] and [14] read on 2026-11-06. Generic linear constraints remove a node and produce linearised factors over its elimination clique, exact as dense constraints or approximate on a Chow–Liu tree. Factor descent sparsifies the dense marginal into a chosen topology by minimising the Kullback–Leibler divergence one factor at a time, without tuning parameters, and has a closed form for Chow–Liu trees. It decides how many factors replace the prior, not whether they can be relinearised.

Formulation of the Chow–Liu tree, derived here from the standard construction. Step 1. The best tree shaped approximation of a density in the Kullback–Leibler sense keeps the edges of the spanning tree of maximum total mutual information. Step 2. A Gaussian of dimension $n$ and covariance $\boldsymbol{\Sigma}$ has the entropy $\frac{1}{2}\ln\big((2\pi e)^n\,\text{det}\boldsymbol{\Sigma}\big)$. Step 3. The mutual information of two blocks is the sum of their entropies minus the joint one, and the $(2\pi e)$ factors cancel because the dimensions add,

$$I(\mathbf{x}_i; \mathbf{x}_j) = \frac{1}{2}\ln\frac{\text{det}\boldsymbol{\Sigma}_{ii}\,\text{det}\boldsymbol{\Sigma}_{jj}}{\text{det}\boldsymbol{\Sigma}_{\{ij\}}},$$

with $\boldsymbol{\Sigma}_{\{ij\}}$ the joint covariance of the two blocks. Each tree edge is then a factor on the pair.

Formulation of one factor descent step, derived here. Step 1. With every factor but the $k$-th held fixed, write $\mathbf{H}_a = \mathbf{A} + \mathbf{J}_k^\top \boldsymbol{\Omega}_k \mathbf{J}_k$, where $\mathbf{A}$ collects the others and $\boldsymbol{\Omega}_k$ is the unknown information. The divergence of §3.4.2 Step 1 depends on $\boldsymbol{\Omega}_k$ through $\frac{1}{2}\text{tr}(\mathbf{H}_a \boldsymbol{\Sigma}_o) - \frac{1}{2}\ln\,\text{det}\mathbf{H}_a$. Step 2. By (M4) and (M5), the derivative of the trace term with respect to $\boldsymbol{\Omega}_k$ is $\frac{1}{2}\mathbf{J}_k \boldsymbol{\Sigma}_o \mathbf{J}_k^\top$, and that of the log determinant term is $\frac{1}{2}\mathbf{J}_k \mathbf{H}_a^{-1} \mathbf{J}_k^\top$, from $d\ln\,\text{det}\mathbf{X} = \text{tr}(\mathbf{X}^{-1} d\mathbf{X})$ with $d\mathbf{X} = \mathbf{J}_k^\top d\boldsymbol{\Omega}_k \mathbf{J}_k$. Step 3. The stationarity condition is

$$\mathbf{J}_k \mathbf{H}_a^{-1} \mathbf{J}_k^\top = \mathbf{J}_k \boldsymbol{\Sigma}_o \mathbf{J}_k^\top,$$

so the approximation must reproduce the marginal covariance of the factor's own measurement. Step 4. Write $\mathbf{C} = \mathbf{J}_k \mathbf{A}^{-1} \mathbf{J}_k^\top$ and $\mathbf{S}_k = \mathbf{J}_k \boldsymbol{\Sigma}_o \mathbf{J}_k^\top$. By (M8) with $\mathbf{U} = \mathbf{J}_k^\top$, $\mathbf{V} = \mathbf{J}_k$ and $\boldsymbol{\Omega}_k$ in the role of $\mathbf{C}$, $\mathbf{J}_k \mathbf{H}_a^{-1} \mathbf{J}_k^\top = \mathbf{C} - \mathbf{C}(\boldsymbol{\Omega}_k^{-1} + \mathbf{C})^{-1} \mathbf{C}$, and by (M8) again, read with $\mathbf{A} = \mathbf{C}^{-1}$ and $\mathbf{U} = \mathbf{V} = \mathbf{I}$, this equals $(\boldsymbol{\Omega}_k + \mathbf{C}^{-1})^{-1}$. Step 5. Setting it equal to $\mathbf{S}_k$,

$$\boldsymbol{\Omega}_k = \mathbf{S}_k^{-1} - (\mathbf{J}_k \mathbf{A}^{-1} \mathbf{J}_k^\top)^{-1},$$

which is the information of the factor's measurement under the marginal minus what the other factors already supply. It reduces to the closed form of §3.4.2 when the other factors supply nothing about this measurement. A step that leaves $\boldsymbol{\Omega}_k$ indefinite must be clipped to the positive semidefinite cone. This is a derivation of the coordinate descent that [14] names, not a transcription of its algorithm.

Q1 to Q6. These methods choose the topology and the informations of recovered factors. With the gauge invariant residuals of §6.8 they inherit its answers to Q1 to Q5, and the loss measure $D_\text{KL}^*$ of §6.8 applies to any topology they choose. Their cost is iterative, a few factorisations of $\mathbf{H}_a$ per descent sweep, which is not measured here.

### 6.11 Information sparsification in VIO [6]

Source and reading. Abstract and search snippets of [6] read on 2026-11-06, as in §3.4. The dense marginalisation prior of a fixed lag VIO is replaced by a sparse set of nonlinear factors obtained by non-linear factor recovery.

Formulation and derivation. In Basalt this is §6.8 with a sparse topology, the relative pose factors to the departing keyframe and a roll pitch factor, as [3, §V-C] chooses for the mapper. Its information loss is $D_\text{KL}^*$ of §6.8, and with fewer factors than prior dimensions the informations follow §6.10. What [6] adds beyond §6.8 was not read, so no further claim is made.

Q1 to Q6. As §6.8, with a smaller prior and a larger $D_\text{KL}^*$. The sparsity matters for a large window and little for Basalt's 75 coordinates.

### 6.12 Relative marginalisation, ICE-BA [15]

Source and reading. Abstract and repository README read on 2026-11-06, and §5 with eq. 15 to 21 and Fig. 5 read on 2026-11-07. A "relative marginalization algorithm" resolves the conflict between the sliding window marginalisation prior and global loop closure constraints, with code at `github.com/baidu/ice-ba`. Its relevance is the same conflict in Basalt between the estimator's frozen prior and the local mapper's pose updates (`plans/gravity_scale_optimisation_2.md` §3.6 item 8).

Formulation, from [15]. "The key idea is to formulate the prior relative to the reference keyframe coordinate system instead of the global coordinate system" [15, §5]. With the closest keyframe $k_0$ as reference, a frame's pose is written relative to it, the gravity direction is written in the reference keyframe's frame, $\mathbf{g}_{k_0} = \mathbf{R}_{k_0} \mathbf{g}$ in the paper's world to camera convention, and the velocity in the frame's own body frame. The inertial error is rewritten in these quantities [15, eq. 17], the prior is a Gaussian over $(\mathbf{g}_{k_0}$, the relative state of the next frame, the relative keyframe poses$)$ [15, eq. 18], and it is converted to the global frame for optimisation [15, eq. 19]. When the reference keyframe changes, a weak prior on the new gravity variable is added [15, Fig. 5f]. The paper states that "the relative representation is more complicated for VI-SLAM since the gravity direction becomes observable" [15, §5].

Derivation of its invariances, derived here in Basalt's convention. In Basalt's body to world convention the relative quantities are $\mathbf{T}_{k_0}^{-1} \mathbf{T}_i$, the gravity in the reference body frame $\mathbf{R}_{k_0}^\top \mathbf{g}_D$, and the body frame velocity $\mathbf{R}_i^\top \mathbf{v}_i$.

Step 1, a rigid change of the whole map. Under $\mathbf{T} \to \mathbf{G} \mathbf{T}$ for every frame, $(\mathbf{G} \mathbf{T}_{k_0})^{-1} \mathbf{G} \mathbf{T}_i = \mathbf{T}_{k_0}^{-1} \mathbf{T}_i$ by (I5). If $\mathbf{G}$ is a yaw rotation and a translation, gravity is unchanged by it and the gravity in the reference frame is unchanged too. The relative prior is therefore exactly invariant under the four directions of today's gauge.

Step 2, the re-centring $\Phi_c = (\mathbf{Q}, s_c)$. The relative rotation $\mathbf{R}_{k_0}^\top \mathbf{Q}^\top \mathbf{Q} \mathbf{R}_i$ and the reference gravity $\mathbf{R}_{k_0}^\top \mathbf{Q}^\top \mathbf{Q} \mathbf{g}_D$ are unchanged, while the relative translation $\mathbf{R}_{k_0}^\top (\mathbf{p}_i - \mathbf{p}_{k_0})$ and the body frame velocity $\mathbf{R}_i^\top \mathbf{v}_i$ are multiplied by $s_c$. The re-expression of a relative prior is therefore a diagonal scaling of its translation and velocity columns by $\frac{1}{s_c}$, with no rotation, in place of the block rotations $\mathbf{Q}^\top$ of $\mathbf{A}^{-1}$ (first plan §2.7.3).

Step 3, the mapper. An accepted pose update that moves the window's keyframes rigidly leaves a relative prior unchanged by Step 1, so only the non rigid part of the update can conflict with it, whereas today the whole overwrite does (§6.5 Q4).

Q1. The prior's content is the same as Basalt's, so L1 to L5 fare as today. The pins are absolute, and a relative prior holds them only through the reference keyframe, the weak gravity prior at each new reference being a fresh, unanchored regulariser.

Q2. The scale dependent rows are linearised as today, so §1.3 applies unchanged. Relative marginalisation addresses consistency with global corrections, not with a moving scale.

Q3. The gravity variable $\mathbf{R}_{k_0}^\top \mathbf{g}_D$ is the counterpart of the block's direction, held in a frame that moves with the reference keyframe. A relative prior satisfies $\mathbf{J} \mathbf{N} = \mathbf{0}$ along the yaw and translation directions exactly by Step 1.

Q4. Its main benefit, by Step 3. It addresses the conflict that is the leading hypothesis for the divergence of the seven dimensional free gauge (§6.17).

Q5. Neither F1 nor F2.

Q6. A change of the prior's parameterisation and a conversion to the global frame at every optimisation [15, eq. 19], large for Basalt, whose marginalisation, re-centring and mapper packets all assume world frame priors.

### 6.13 First estimate Jacobians and observability constraints [16, 17]

Source and reading. Abstracts and search summaries of [16, 17] read on 2026-11-06, and the account of both in [18, §III-A] read on 2026-11-07. FEJ evaluates every Jacobian at the first estimate so that the prior and the window share null spaces, and the observability constrained filters modify the Jacobians so that the unobservable directions are preserved. Basalt and the first plan use FEJ.

Formulation of FEJ. Every factor that touches a variable evaluates its Jacobian at that variable's first estimate $\bar{\mathbf{x}}$, so that for every factor $\mathbf{J}(\bar{\mathbf{x}})\,\mathbf{n}(\bar{\mathbf{x}}) = \mathbf{0}$ holds at one common point, and the sum, prior included, keeps the gauge as a null space (first plan §2.5.2 and §2.5.6). [18, eq. 17] states the filtering form, $\bar{\mathbf{H}}_{k+1} \bar{\boldsymbol{\Phi}}(k+1, 0) \mathbf{N}_0 = \mathbf{0}$.

Formulation of the observability constraint. [18, eq. 14 and 15] describes OC as finding the Jacobians closest to the current ones that annihilate the propagated null space, $\mathbf{H}^*_k \mathbf{N}_k = \mathbf{0}$, and notes that "the Jacobians used do not strictly follow the first-order Taylor series expansion". Derived here, the closest such Jacobian in the Frobenius norm is the current one times the projector. Step 1. $\mathbf{J}^* \mathbf{N} = \mathbf{0}$ holds exactly when every row of $\mathbf{J}^*$ is orthogonal to the columns of $\mathbf{N}$. Step 2. The Frobenius distance is the sum of the squared distances of the rows, so each row is chosen independently as the closest vector of the orthogonal complement of $\mathbf{N}$ to the corresponding row of $\hat{\mathbf{J}}$. Step 3. By the least squares of the first plan §2.5.6, that vector is $\mathbf{P}$ times the row, so with $\mathbf{P}$ symmetric

$$\mathbf{J}^* = \hat{\mathbf{J}} \mathbf{P}.$$

Step 4. Stacking all factors, $\mathbf{J}^{*\top} \mathbf{J}^* = \mathbf{P} \hat{\mathbf{J}}^\top \hat{\mathbf{J}} \mathbf{P} = \mathbf{P} \mathbf{H} \mathbf{P}$. The system projection the first plan applies as a safeguard is therefore the observability constrained correction of every Jacobian at the points where $\mathbf{N}$ is built.

Q1 to Q6. FEJ and OC keep the gauge free of spurious information. They retain no prior that marginalisation has discarded, they leave the error of §1.3 untouched because they act on null spaces and not on the scale's first estimate, and both are already in Basalt, FEJ throughout and OC as the projection. Their cost is nil.

### 6.14 FEJ2 [18]

Source and reading. [18] read in full, §II and §III with eq. 1 to 30, on 2026-11-07. FEJ keeps the right null space but its first estimates may be poor. FEJ2 linearises at the current estimate for the smallest linearisation error and keeps the first estimate Jacobian for the observability properties, treating the difference between the two Jacobians as an unknown that it removes from the residual by a left null space projection. This deflates the information of each update, so the covariance is inflated and stays consistent. For a prior linearised at a wrong scale, this is the filtering counterpart of widening the prior instead of discarding it, and it would bound the overconfidence of §1.3 without a swap, at the price, derived below, of keeping only the rotation rows of a scale mismatched factor. Note, 2026-11-07. This paragraph previously stated, from the abstract, that FEJ2 models the gap between the two Jacobians as additional noise that inflates the covariance. The paper projects the residual onto the left null space of the gap, which by (M9) is the limit of noise of unbounded variance along the gap's columns, so the earlier reading was the limiting case rather than the mechanism.

Formulation, from [18]. Step 1. With $\hat{\mathbf{H}}$ the Jacobian at the current estimate and $\bar{\mathbf{H}}$ the first estimate Jacobian, the residual is [18, eq. 22 to 24]

$$\hat{\mathbf{r}} \approx \bar{\mathbf{H}}\,\tilde{\mathbf{x}} + \Delta\mathbf{H}\,\tilde{\mathbf{x}} + \mathbf{n},$$

$$\Delta\mathbf{H} = \hat{\mathbf{H}} - \bar{\mathbf{H}},$$

with $\tilde{\mathbf{x}}$ the error state. Step 2. A QR factorisation $\Delta\mathbf{H} = [\Delta\mathbf{Q}\ \Delta\mathbf{U}]\,[\Delta\mathbf{T}; \mathbf{0}]$ gives the left null space $\Delta\mathbf{U}^\top \Delta\mathbf{H} = \mathbf{0}$ [18, eq. 25], and the update uses [18, eq. 26]

$$\Delta\mathbf{U}^\top \hat{\mathbf{r}} = \Delta\mathbf{U}^\top \bar{\mathbf{H}}\,\tilde{\mathbf{x}} + \Delta\mathbf{U}^\top \mathbf{n}.$$

Step 3. The null space of the observability matrix is preserved, $\Delta\mathbf{U}^\top \bar{\mathbf{H}} \bar{\boldsymbol{\Phi}} \mathbf{N}_0 = \mathbf{0}$ [18, Lemma 3.1]. Step 4. With whitened noise, $\Delta\mathbf{Q} \Delta\mathbf{Q}^\top + \Delta\mathbf{U} \Delta\mathbf{U}^\top = \mathbf{I}$, so the information is [18, eq. 28 to 30]

$$\boldsymbol{\Lambda}_\text{FEJ2} = \bar{\mathbf{H}}^\top \bar{\mathbf{H}} - \bar{\mathbf{H}}^\top \Delta\mathbf{Q} \Delta\mathbf{Q}^\top \bar{\mathbf{H}} \preceq \bar{\mathbf{H}}^\top \bar{\mathbf{H}},$$

which is [18, Lemma 3.2]. The projection needs $\Delta\mathbf{H}$ of full column rank on its nonzero block, and [18, §III-D] applies it to the nonzero block only.

Derivation for one inertial factor at a moved scale, derived here. Let the factor's Jacobians be held at the scale $\bar{s}$ while the current scale is $s$. Step 1. By the first plan §2.3.2 and §2.3.3, the position and velocity rows have the state columns $\pm s\,\mathbf{R}_0^\top$ and the scale columns $s\,\mathbf{R}_0^\top (\mathbf{p}_1 - \mathbf{p}_0 - \mathbf{v}_0 \Delta t)$ and $s\,\mathbf{R}_0^\top (\mathbf{v}_1 - \mathbf{v}_0)$. Step 2. The gap therefore has the blocks $(s - \bar{s})\,\mathbf{R}_0^\top$ in the position rows at $\delta\mathbf{p}_1$ and in the velocity rows at $\delta\mathbf{v}_1$, both invertible, so $\Delta\mathbf{H}$ has rank 6 over those six rows. Step 3. Its left null space within the nine rows is then at most the three rotation rows, and by (M9) FEJ2 keeps exactly the rotation rows, which is the rotation only factor of Change A (plan 2 §3.3.3). Applied exactly, FEJ2 confirms Change A's treatment of a scale mismatched factor as the consistent one, and offers no way to keep its position and velocity information.

A thresholded FEJ2 and the admission test, derived here. Step 1. For a factor marginalised into $M_\text{visual}$ at the current scale, the state columns are exact at that moment, and its only scale dependent error after a later change $\Delta\sigma$ is the term $\mathbf{c}\,\Delta\sigma$ of the whitened scale column (plan 2 §3.3.4 Step 1), up to the second order bilinear term. Step 2. Treating the future $\Delta\sigma$ as a nuisance without a prior, (M9) with $\mathbf{G} = \mathbf{c}$ removes exactly one row combination, the one along $\mathbf{c} / |\mathbf{c}|$, and keeps the eight whitened combinations orthogonal to it,

$$\mathbf{U}_c^\top \mathbf{L}\,\mathbf{r},$$

$$\mathbf{U}_c^\top \mathbf{c} = \mathbf{0}.$$

Step 3. The projected factor is exactly independent of the scale to first order, and it stays gauge invariant, $\mathbf{U}_c^\top \mathbf{J} \mathbf{n}_\sigma = \mathbf{0}$, because the exact factor satisfies $\mathbf{J} \mathbf{n}_\sigma = \mathbf{0}$. Zeroing the column instead leaves $\mathbf{J}' \mathbf{n}_\sigma = \mathbf{c}$ (plan 2 §3.3.4 Step 4). Step 4. Change A's admission test either zeroes $\mathbf{c}$, with an error of at most $\tau\,|\Delta\sigma|$ standard deviations, or keeps the three rotation rows, losing six. The projection keeps eight rows whatever $|\mathbf{c}|$ is. On the ground, where position jitter gives $|\mathbf{c}| \approx 6$ and rejects almost every factor (plan 2 §5.5 item 4), the column is dominated by the position row, while the tilt and bias information of §6.1.2 L4 lies mainly in the velocity row through $\Delta t\,\mathbf{R}_0^\top [\bar{\mathbf{g}}_D]_\times \bar{\mathbf{B}}$, so most of it would survive the projection. This variant has not been implemented or run, and how much tilt information survives depends on the angle between $\mathbf{c}$ and the whitened tilt columns, which can be logged.

Q1. Exact FEJ2 retains no more than Change A. The thresholded variant would retain L4 for the ground factors that Change A rejects.

Q2. The projected factor has no first order scale error, and the bilinear term of §1.3 is of second order for factors whose velocity change is small, which is the ground case.

Q3. Gauge invariant by Step 3. The rows are fixed linear combinations of whitened residual rows, so the re-centring maps them like any frozen row.

Q4 to Q6. The mapper is unaffected, neither F1 nor F2 is addressed, and the cost is one Householder reflection of nine rows per marginalised factor, inside the existing `ImuBlockGS` branch of Change A.

### 6.15 Invariant and equivariant formulations [19, 20]

Source and reading. Abstracts of [19, 20] read on 2026-11-06, and the account of invariant VINS in [18, §I] read on 2026-11-07. A right invariant error makes the linearised dynamics independent of the global pose, so a fixed lag smoother is consistent "without artificially correcting Jacobians" [19]. EqVIO builds a filter on a symmetry of VIO "compatible with the invariance of the VIO reference frame" [20]. [18, §I] describes invariant VINS as associating the uncertainty with "an invariant error state on the manifold, which does not change under any stochastic observable transformation". Neither [19] nor [20] treats the scale as a state, so whether a symmetry that includes the scale removes the scale dependence of the inertial prior is open.

Derivation of how the scale acts on an invariant error, derived here. Step 1. Write a navigation state as the triple $\mathbf{X} = (\mathbf{R}, \mathbf{v}, \mathbf{p})$ with the product $(\mathbf{R}_1, \mathbf{v}_1, \mathbf{p}_1)(\mathbf{R}_2, \mathbf{v}_2, \mathbf{p}_2) = (\mathbf{R}_1 \mathbf{R}_2,\ \mathbf{R}_1 \mathbf{v}_2 + \mathbf{v}_1,\ \mathbf{R}_1 \mathbf{p}_2 + \mathbf{p}_1)$, the extension of (I5) that the invariant filtering literature uses. Step 2. The map $(\mathbf{R}, \mathbf{v}, \mathbf{p}) \to (\mathbf{R}, k\,\mathbf{v}, k\,\mathbf{p})$ preserves this product, by the computation of (S1) applied to each translational part. It is an automorphism, not a multiplication by a group element. Step 3. A world change by a group element $\mathbf{G}$ acting on the left leaves the error $\mathbf{X}^{-1} \hat{\mathbf{X}}$ unchanged, since $(\mathbf{G} \mathbf{X})^{-1} \mathbf{G} \hat{\mathbf{X}} = \mathbf{X}^{-1} \hat{\mathbf{X}}$, which is the invariance under yaw and translation. Step 4. The scale family maps the error by the automorphism of Step 2, so its translational components are multiplied by $k$, in the tangent space by a diagonal matrix of the form of $\mathbf{D}_s$. The scale is therefore a covariance of the error, not an invariance, and a prior in invariant coordinates must be rescaled under a scale change, exactly as the first plan's $\mathbf{A}^{-1}$ already rescales Basalt's prior (first plan §2.7.3). Step 5. The bilinear error of §1.3 comes from the product $s\,\mathbf{u}$ in the measurement function $\mathbf{r}_v$, not from the coordinates of the error, so no choice of error coordinates removes it. It can only be moved into a factor in which the scale is unobservable, which is what VI-DSO does by evaluating the photometric error in a scaled visual frame (§3.2.1).

Q1 to Q6. An invariant or equivariant formulation would remove the need for FEJ along yaw and translation, which Basalt already handles with FEJ. It retains no discarded prior, leaves §1.3 unchanged by Step 5, and would be a rewrite of the estimator.

### 6.16 Robocentric formulation, R-VIO [21]

Source and reading. Abstract of the journal version [21] read on 2026-11-06, and the conference version [21b] read on 2026-11-07 via ar5iv, §III-A, §III-D and §IV, through a summary of the rendered page. Estimating relative to a moving local frame makes the local frame the reference of the state. The state holds the pose of the global frame relative to the current local frame and the gravity vector expressed in the local frame, $\mathbf{x}_G = [\bar{\mathbf{q}}_G^k, \mathbf{p}_G^{R_k}, \mathbf{g}^{R_k}]$ [21b, eq. 1], so no first frame pins are needed, because the reference is the current local frame itself, which a composition step resets with "no uncertainty for the robocentric frame of reference itself" [21b, §III-D]. The paper states that its unobservable subspace is constant and independent of the linearisation points [21b, §IV], so the observability mismatch of world centric filters does not arise. Note, 2026-11-07. This paragraph previously stated, from the abstract of [21], that the robocentric formulation removes the global frame from the state. The conference paper keeps the global frame's pose relative to the local frame as a state, which is unobservable, and gives a null space of nine directions [21b, eq. 66].

Formulation, from [21b]. After each update the current IMU frame becomes the new local frame. The global pose is re-expressed in it, gravity is rotated, $\mathbf{g}^{R_{k+1}} = \mathbf{C}(\bar{\mathbf{q}}_k^{k+1})\,\mathbf{g}^{R_k}$, the relative pose is reset to the identity, and the covariance is transformed as $\mathbf{P}_{k+1} = \mathbf{U} \mathbf{P}_{k+1|k+1} \mathbf{U}^\top$ [21b, eq. 37 and 38]. The scale is not a state, and is shown unobservable along a fixed direction under two degenerate motions, no rotation with a particular relation of velocity and acceleration, and rest [21b, §IV-C, Lemma 1 and 2].

Derivation of the analogy with the block, derived here. Step 1. $\mathbf{g}^{R_k}$ is the gravity vector in a frame chosen by the estimator, as $\mathbf{g}_D$ is the gravity vector in $D$. Step 2. The composition step rotates the gravity state and resets the frame, as the re-centring of the first plan §2.7.1 rotates the block's linearisation point, $\bar{\mathbf{d}}' = \mathbf{Q} \bar{\mathbf{d}}$, and resets the block to the identity. Step 3. The covariance transform $\mathbf{U} \mathbf{P} \mathbf{U}^\top$ corresponds to the prior re-expression $\mathbf{J}' = \mathbf{J} \mathbf{M}^{-1}$, which in information form is $\mathbf{H}' = \mathbf{M}^{-\top} \mathbf{H} \mathbf{M}^{-1}$. The block with its re-centring is therefore a world centric relative of the robocentric gravity state, with the difference that the re-centring rotates the world onto gravity instead of moving the reference onto the body.

Q1. A robocentric estimator needs no position and yaw pins, so L1 and L2 cannot be lost. The bias and the inertial information are carried in the covariance as in any filter, linearised at the estimate of each step.

Q2. Without a scale state it has no product $s\,\mathbf{u}$, and the scale is carried by the states, as in today's Basalt.

Q3 to Q6. Its gauge structure is fixed by construction. Adopting it would replace Basalt's world frame estimator, its priors and its mapper interface. [18, §I] adds that it "requires extra states to be estimated and causes more unobservable directions".

### 6.17 A persistent gauge prior, and the seven dimensional free gauge

Source and reading. DM-VIO keeps priors on the first pose and gravity direction as separate factors [2, §III-B]. For Basalt the equivalent is to stop folding the position and yaw pins into the marginal prior and to re-impose them at every frame on the oldest state of the window. Alternatively, all seven directions of the gauge can be handled by the projector, as DSO does for its own seven [4], which needs no pin at all. The second option extends `ComputeGaugeProjector` with the three translation and the yaw vectors, whose entries are known in closed form (`plans/gravity_scale_optimisation.md` §2.5.3). It was implemented and removed, as recorded at the end of this subsection.

What the pins carry, derived here. The absolute position and yaw are unobservable, so L1 and L2 carry no physical information. They choose the representative of the four directions of today's gauge. What is physical is that the representative stays where the rest of the system expects it, which is the local mapper's world, because the mapper's accepted pose updates are expressed there (first plan §2.9.4).

Option 1, a persistent pin on the oldest frame, derived here. Step 1. The residual $\sqrt{w}\,(\mathbf{p}_o - \bar{\mathbf{p}}_o)$ together with $\sqrt{w}$ times the yaw offset of the oldest frame $o$, re-imposed at every frame at its current value and never marginalised, is a gauge prior with the selection $\mathbf{S}_o$ of that frame's position and yaw. Step 2. Along the translation vectors $\mathbf{S}_o \mathbf{n}_t = \mathbf{e}_k$, and along the yaw vector the yaw entry is 1, so the four directions of today's gauge are fixed with rank 4 and perfect conditioning (first plan §2.5.4, gauge prior). Step 3. Along the scale and tilt vectors of the block, $\mathbf{S}_o \mathbf{n}_\sigma = \mathbf{p}_o$ and $\mathbf{S}_o \mathbf{n}_\mathbf{a} = [\mathbf{a} \times \mathbf{p}_o; 0]$, the yaw entry being zero for a horizontal $\mathbf{a}$. These are nonzero once the oldest frame has left the origin, so the pin would also resist the block's gauge, with the curvature $w\,|\mathbf{p}_o|^2$ along $\mathbf{n}_\sigma$, about $10^8$ times the squared distance. Step 4. The initial pin avoids this only because the first state sits at the origin, about which every correction of the first plan §2.7 is a similarity (that plan's §2.5.4 item 1). A persistent pin on a frame away from the origin therefore requires the scale and tilt families, and so $\mathbf{N}$ and the re-centring $\Phi_c$, to be taken about the pinned frame, $\mathbf{p} \to \mathbf{p}_o + e^{\varepsilon}(\mathbf{p} - \mathbf{p}_o)$ for the scale and $\mathbf{p} \to \mathbf{p}_o + \mathbf{Q}(\mathbf{p} - \mathbf{p}_o)$ for the tilt. Both are exact gauge families, because every residual sees positions only through differences, and both give $\mathbf{S}_o \mathbf{N} = \mathbf{0}$ by construction, so the position rows of $\mathbf{n}_\sigma$ become $\mathbf{p}_f - \mathbf{p}_o$ and those of $\mathbf{n}_\mathbf{a}$ become $\mathbf{a} \times (\mathbf{p}_f - \mathbf{p}_o)$. Step 5. When the oldest frame leaves, the pin moves to the next frame at its current value, so the representative changes without a jump. Step 6. Pinning the oldest keyframe at the mapper's refined pose instead of its own current value would place the estimator's representative in the mapper's world, which is the cheap counterpart of the separator exchange of §6.5.

Option 2, the seven dimensional free gauge, from plan 2 Change 10. The translation vectors have $\mathbf{e}_k$ in the position rows of every frame, the yaw vector is the tilt vector with $\mathbf{a} = \mathbf{e}_z$, and the initial pins must be removed, because a pin of weight $w$ projected with $K$ frames becomes $w\,(\mathbf{P} \mathbf{e}_p)(\mathbf{P} \mathbf{e}_p)^\top$ with $\mathbf{P} \mathbf{e}_p = \mathbf{e}_p - \mathbf{n}_t / K$, a constraint of weight $w(1 - 1/K)$ on relative motion. The packet for the mapper is re-pinned, because `extractNonlinearFactors` rejects a rank deficient $\mathbf{H}$, and so is the prior at the end of the period.

How the free gauge places the window, derived here. Step 1. The projected step satisfies $\mathbf{n}_t^\top \mathbf{x} = 0$, that is $\sum_f x_{f,k} = 0$ over the $K$ frames, so a solve never moves the window's centroid. Step 2. A correction that physically belongs to the newest frame, of size $\mathbf{c}$, is therefore applied as $\mathbf{c} (1 - 1/K)$ to the newest frame and $-\mathbf{c} / K$ to every other frame. Step 3. Every frame the whole window slides relative to the mapper's world by the mean of the corrections, which is largest at lift off. Step 4. With lost pins and the three dimensional projector, §6.1.3 shows the same mechanism with the centroid weighted by the damping, in which the prior connected frames weigh most and act as a soft pin, so the slide is smaller. This is a candidate mechanism for the hypothesis recorded below, consistent with the larger number of pose updates the free gauge runs applied, and it is untested.

Q1 to Q6. Option 1 retains the gauge choice of L1 and L2 exactly at negligible cost per frame, but it requires the scale and tilt gauge and the re-centring to be redefined about the pinned frame, which touches `ComputeGaugeProjector`, `RecentreGravityScale` and the correction log. Option 2 retains them by needing no pins, and diverged. Neither retains L3 to L5 or addresses F1 or F2.

Result of the seven dimensional free gauge, 2026-11-07. Implemented as `vio_gs_free_gauge` (`plans/gravity_scale_optimisation_2.md` Change 10). It removes the position and yaw pins from the initial prior, adds translation and yaw to the projector, and re-pins at the mapper packet and at the fold. On two replays of `telemetry_40` it diverged at lift off, with scale ratios of 146 to 485 and 263 to 280 swaps, while the same binary with the key off gave 25 swaps and a tilt error below 4°. It was therefore removed from the code base and from every configuration file on 2026-11-07, and the plan keeps its derivation and code as a rejected design. §6.17 Option 2 should not be taken as a cheap fix until the cause is found. The leading hypothesis is that a drifting estimator gauge conflicts with the local mapper's pose updates, which overwrite frozen keyframe linearisation points in the mapper's world. A replay with the free gauge on and the mapper's pose updates disabled would test it (plan 2 Change 10, Result). Note, 2026-11-07. This paragraph previously referred to §6.5 item 15, the numbering of the first version of this section.

### 6.18 A persistent bias prior

Source and reading. Campos, Montiel and Tardós keep an explicit accelerometer bias prior in the inertial only problem (reference [12] of `plans/gravity_scale_optimisation_2.md`, read via ar5iv). For Basalt the prior can be propagated as a separate factor on the newest state, with variance $\sigma_{b_a,0}^2 + \sigma_{b_a,\text{rw}}^2 T$ as derived in `plans/gravity_scale_optimisation_2.md` §3.3.6. Change A already carries it through the random walk rows, so this matters only if Change A is off.

Derivation of the double counting, derived here. Step 1. With Change A on, the chain carries the anchor of information $\frac{1}{\sigma_{b_a,0}^2 + \sigma_{b_a,\text{rw}}^2 T}$ to the newest state (§6.1.2 L3). Step 2. A separate factor with the same variance adds the same information, so the total is twice the information, and the variance is halved, although no new measurement was made. Step 3. If the factor were marginalised at every frame and re-imposed on the next state, each marginalisation would store another copy in the prior, and after $n$ frames the prior would hold about $n$ times the information of the initial prior. Step 4. A persistent bias prior is therefore correct only as a factor outside every prior, like the zero velocity term, and only when nothing else carries the anchor, that is with Change A off or in a prior without the bias chain.

Q1 to Q6. It retains L3 exactly at negligible cost, it is redundant with Change A, and it addresses neither F1 nor F2.

### 6.19 A prior on the tilt of the first frame

Source and reading. Proposed in the first version of this section. The first gravity alignment is accurate to about $\sigma_{b_a} / |\mathbf{g}|$, 0.58° (`plans/gravity_scale_optimisation_2.md` §3.2.3), so a residual on the total tilt held by the correction log, of a few degrees, is a physical prior on the first frame's attitude that no swap can discard. It also bounds the tilt runaway of `_23-18-44`. Two formulations are derived here, and the second is the gauge invariant one.

Formulation 1, a residual on the correction log. The correction log holds the cumulative rotation $\mathbf{Q}_C$ of every re-centring (first plan §2.9.1), and within a frame the block holds the direction $\mathbf{d}$, so the gravity direction expressed in the initial world is $\mathbf{Q}_C^\top \mathbf{d}$, and the residual is the horizontal part of its deviation from $\hat{\mathbf{g}}$. Step 1. It depends on the block's tilt coordinates only, so along $\mathbf{n}_\mathbf{a}$ its curvature is nonzero, and like Change B it must be added before the projection. Step 2. After the projection it acts on the share $1 - 1/|\mathbf{n}_\mathbf{a}|^2$, about 0.9, of each physical tilt correction that the block carries (plan 2 §3.4.3). Step 3. The remaining share is carried by the frames' attitudes and is never written to the correction log, so $\mathbf{Q}_C$ underestimates the physical cumulative tilt by about a tenth of every correction. Formulation 1 is therefore a bound on the block's history rather than on the physical tilt.

Formulation 2, a body frame gravity prior on the first state. Step 1. The physical tilt of a frame is its body frame gravity direction, $(\mathbf{R}^M)^\top \hat{\mathbf{g}} = \mathbf{R}^\top \mathbf{d}$ (§6.8, item 2), and the residual on the first state is

$$\mathbf{r}_g = \mathbf{W}_g\,\big\lfloor \mathbf{R}_0^\top \mathbf{d} - \mathbf{z}_0 \big\rfloor_{xy},$$

with $\mathbf{z}_0$ the body frame gravity direction of the initial alignment and $\mathbf{W}_g$ a diagonal weight. Step 2, its Jacobian with respect to the attitude. Under $\mathbf{R}_0 \to \text{Exp}(\delta\boldsymbol{\varphi}_0)\,\mathbf{R}_0$, (I8) and (I1) give $\mathbf{R}_0^\top \text{Exp}(-\delta\boldsymbol{\varphi}_0)\,\mathbf{d} = \mathbf{R}_0^\top \mathbf{d} - \mathbf{R}_0^\top [\delta\boldsymbol{\varphi}_0]_\times \mathbf{d} + O(|\delta\boldsymbol{\varphi}_0|^2)$, and by (I2)

$$\frac{\partial(\mathbf{R}_0^\top \mathbf{d})}{\partial\delta\boldsymbol{\varphi}_0} = \mathbf{R}_0^\top [\mathbf{d}]_\times.$$

Step 3, its Jacobian with respect to the block. By the first plan §2.1.3, $\partial\mathbf{d} / \partial\boldsymbol{\delta}_\theta = -[\bar{\mathbf{d}}]_\times \bar{\mathbf{B}}$, so

$$\frac{\partial(\mathbf{R}_0^\top \mathbf{d})}{\partial\boldsymbol{\delta}_\theta} = -\mathbf{R}_0^\top [\bar{\mathbf{d}}]_\times \bar{\mathbf{B}}.$$

Step 4, gauge invariance. Along $\mathbf{n}_\mathbf{a}$ the attitude component is $\mathbf{a}$ and the block component is $\bar{\mathbf{B}}^\top \mathbf{a}$, so at $\mathbf{d} = \bar{\mathbf{d}}$ the change is

$$\mathbf{R}_0^\top [\bar{\mathbf{d}}]_\times \mathbf{a} - \mathbf{R}_0^\top [\bar{\mathbf{d}}]_\times \bar{\mathbf{B}} \bar{\mathbf{B}}^\top \mathbf{a} = \mathbf{R}_0^\top [\bar{\mathbf{d}}]_\times (\mathbf{I} - \bar{\mathbf{B}} \bar{\mathbf{B}}^\top)\,\mathbf{a} = \mathbf{R}_0^\top [\bar{\mathbf{d}}]_\times \bar{\mathbf{d}}\,\bar{\mathbf{d}}^\top \mathbf{a} = \mathbf{0},$$

using the decomposition $\mathbf{a} = \bar{\mathbf{B}} \bar{\mathbf{B}}^\top \mathbf{a} + \bar{\mathbf{d}} \bar{\mathbf{d}}^\top \mathbf{a}$ of the first plan §2.5.3 and $[\bar{\mathbf{d}}]_\times \bar{\mathbf{d}} = \mathbf{0}$. The residual contains no scale, so it is invariant along $\mathbf{n}_\sigma$ too, and it is exactly the gravity aware roll pitch factor of §6.8 item 2 placed on the first state. Step 5, propagation. When the first state leaves, §6.1.1 applies with this residual as part of the departing variable's prior on roll and pitch. The rotation row links $\delta\boldsymbol{\varphi}_0$ to $\delta\boldsymbol{\varphi}_1$, so by the argument of §6.1.2 L2, now with roll and pitch carrying a prior, the body frame gravity of state 1 receives the information. The rows have tilt columns and no scale column, so they are admissible in $M_\text{visual}$ with its tilt anchor (plan 2 §3.3.1 and §3.3.5), and with Change A on they survive every swap.

Derivation of its weight, derived here. Step 1. At rest, the tilt and the horizontal accelerometer bias form one null direction of the inertial factors (plan 2 §3.2.3), and the bias anchor already gives the tilt the information $|\mathbf{g}|^2 / \sigma_{b_a}^2 \approx 9600$ rad⁻² at $\sigma_{b_a} = 0.1$ m/s² (plan 2 §3.4.5). Step 2. The initial alignment's error is that same bias, so a tilt prior of standard deviation $\sigma_{b_a} / |\mathbf{g}|$ would count the bias prior twice. Step 3. A prior of 3°, 0.0524 rad, adds $1 / 0.0524^2 \approx 365$ rad⁻², 3.8 % of the information already held, so it acts as a plausibility bound rather than as new information. Step 4. Against the tilt runaway of `_23-18-44`, which reached 43° in 4 s, such a prior would cost $\frac{1}{2}(43/3)^2 \approx 103$ in whitened units when the runaway is complete, so it would resist the runaway long before. Where the runaway is driven by the kinematic inconsistency F1, the prior can only move the inconsistency into another coordinate, such as the scale.

Q1. It retains a bound on the physical tilt of the first frame, and through the rotation rows on every later state, which by §6.1.2 L4 is otherwise held only through the bias anchor.

Q2. It contains no scale, so it has no scale error.

Q3. Gauge invariant by Step 4, invariant under $\Phi_c$ because it depends on $\mathbf{R}^\top \mathbf{d}$ only, and admissible in $M_\text{visual}$.

Q4. The mapper's roll pitch factors measure the same quantity, so the two are consistent.

Q5. It bounds the tilt part of F2. It does not bound the scale part, for which no physical prior on the cumulative scale exists, and it does not address F1.

Q6. One three row residual in the initial prior and the tilt rows in Change A's linearisation, negligible.

### 6.20 Synthesis

| Method | §6.1 losses retained | Error after a scale change | Gauge and mapper | Change to Basalt | Run time cost | Evidence |
|---|---|---|---|---|---|---|
| Delayed marginalisation [2] | L1 to L5 within $d$ frames, relinearised, first frame priors while younger than $d$, beyond $d$ as Change A | zero at a replacement, at most $(\theta_s - 1)\,s_\text{fej}\,\vert\Delta\mathbf{u}\vert$ between | invariant by (M11), imports mapper pose overwrites into the inertial rows, so needs the mapper's pose updates held while the store is active (§7) | new delayed store, readvance, replacement rule | about 1 % per frame, 15 to 50 ms per replacement | [2], §3.3, §6.3 |
| iSAM2 [10] | everything, nonlinearly | bounded by the relinearisation threshold | block unnecessary, needs no projector | estimator replaced | a batch solve per scale correction, unbounded history | cited from memory, §6.4 |
| Concurrent smoother [11] | everything, nonlinearly | bounded by the synchronisation period | summary invariant if evaluated through $\mathbf{x}^M$, principled mapper feedback | inertial factors in a background smoother, summary back to the window | background thread, $O(1)$ synchronisation | [11, §5], §6.5 |
| Fixation [8, 12] | anchors only, overconfident by up to $10^5$ on the bias at take off | boundary factors relinearised | conflicts with the projector, scale gauge ill conditioned at take off | departed states kept fixed, re-centred | small | [12], §6.6 |
| Keep the inertial chain across non keyframes [13] | L3 to L5 since the newest keyframe | none until marginalised | invariant | composition in the period, nine coordinates per anchor | small | [13, 12], §6.7 |
| NFR with scale aware factors [3, 9] | the marginal as relinearisable factors, with loss $D_\text{KL}^*$ | none later, the first linearisation's error kept | invariant by (M11), shares the mapper's factor type | recovery per marginalisation | moderate, one dense inversion per frame | [3], §6.8 |
| Relative pose errors [12] | the visual marginal, relinearisable | exactly scale free for a monocular camera without lever arm | invariant | edges per departing keyframe | moderate | [12], §6.9 |
| Factor descent and GLC [5, 14] | as NFR, with a chosen topology | as NFR | as NFR | iterative recovery | iterative | abstracts, §6.10 |
| Information sparsification [6] | as NFR, sparse | as NFR | as NFR | as NFR | as NFR | abstract, §6.11 |
| Relative marginalisation [15] | as today | as today | invariant to rigid map updates, removes most of the mapper conflict | prior reparameterised, converted every optimisation | moderate | [15, §5], §6.12 |
| FEJ and OC [16, 17] | none added | unchanged | already in Basalt, OC equals the system projection | none | none | §6.13 |
| FEJ2 [18] | exact form as Change A, thresholded form keeps L4 of ground factors | none to first order for the projected factor | invariant | one reflection per factor in Change A's branch | negligible | [18], §6.14 |
| Invariant and equivariant [19, 20] | none added | unchanged, the product $s\,\mathbf{u}$ remains | yaw and translation without FEJ | estimator rewritten | not assessed | abstracts, §6.15 |
| Robocentric [21] | L1 and L2 by construction | no scale state | fixed by construction | estimator rewritten | not assessed | [21b], §6.16 |
| Persistent pins | L1 and L2 as a gauge choice, possibly in the mapper's world | none | needs $\mathbf{N}$ and $\Phi_c$ about the pinned frame | projector, re-centring and log redefined | negligible | §6.17 |
| Seven dimensional free gauge [4] | removes the need for position and yaw pins | none | diverged in two replays, removed | removed | negligible | plan 2 Change 10, §6.17 |
| Persistent bias prior | L3, redundant with Change A, double counts if added to it | none | invariant | one factor outside the priors | negligible | §6.18 |
| Body frame gravity prior on the first state | a 3° bound on the physical tilt, through every swap with Change A | none | invariant, admissible in $M_\text{visual}$ | three rows in the initial prior | negligible | §6.19 |

Note, 2026-11-07. The table previously listed the seven dimensional free gauge as a negligible cost way to remove the position and yaw pins, and the cumulative tilt prior as a residual on the correction log. The free gauge diverged and was removed (§6.17), and the correction log residual was shown to miss the frames' share of each tilt correction, so the gauge invariant body frame gravity prior of §6.19 replaced it in the table.

The cheapest changes that retain the initial priors exactly are the explicit ones of §6.17 to §6.19, persistent pins for position and yaw, and a body frame gravity prior for the first frame's attitude, together with Change A for the bias. They do not keep the scale information of accelerating factors, which only the methods of §6.3 to §6.7 keep, delayed marginalisation at bounded cost and concurrent smoothing at the cost of a second thread. Note, 2026-11-07. This paragraph previously named the seven dimensional free gauge alongside the persistent pins as the cheapest change, and cited the subsections of the first version of this section, §6.5 and §6.2. The free gauge was removed after it diverged, and the persistent pins were shown in §6.17 to require the scale and tilt gauge to be taken about the pinned frame.

The derivations of §6.3 to §6.19 sort the candidates by the loss they address, which is the basis on which to choose among them.

1. L4, the tilt of the ground phase. The thresholded FEJ2 projection of §6.14 keeps eight of nine rows of every ground factor instead of three, with exact gauge invariance, inside Change A's existing branch. It is the smallest change that addresses an observed loss, and the angle between $\mathbf{c}$ and the tilt columns, which decides its benefit, can be logged before it is implemented.
2. The tilt part of F2. The body frame gravity prior of §6.19, at about 3°, is gauge invariant, survives every swap with Change A, double counts less than 4 % of the information already held, and would have resisted the runaway of `_23-18-44`.
3. L5, the scale information of accelerating factors, which decides whether the scale settles at 1.1 or at 3 (plan 2 §5.5 item 2). Delayed marginalisation (§6.3) is the only bounded cost method that keeps the original measurements and relinearises them, so it repairs both the first and the later linearisation. With $d = 200$ it also keeps the first frame's priors through the whole take off transient of the live runs. The composition of §6.7 is its special case at the delay of one keyframe interval and could be built first, as a step towards it.
4. The conflict with the mapper. Whether the mapper's pose updates against a sliding estimator gauge cause divergence is the open question behind the removal of the free gauge. The replay with the mapper's pose updates disabled tests it. If it is confirmed, the candidates are pinning the oldest keyframe at the mapper's pose (§6.17 Option 1, Step 6), relative marginalisation (§6.12), and, most completely, concurrent smoothing (§6.5).
5. F1, the kinematic inconsistency at lift off. No method of this survey addresses it directly, except a smoother that revises the take off history (§6.4 and §6.5), which is an expectation rather than a result. It needs its own investigation (plan 2 §5.5 item 6).

A diagnostic serves every candidate. Logging the information that the prior in use holds along the three translations, the yaw, the accelerometer bias and the roll and pitch, before and after each swap, would show which of L1 to L4 is lost, and when, on the live runs, and would give each change above a test it must pass.

The row partition of §4.1, the composition of §4.3 as a remedy, the scale aware recovered factors of §4.4, the thresholded FEJ2 projection of §6.14 and the body frame gravity prior with its propagation through $M_\text{visual}$ of §6.19 were not found in the papers read, and their novelty would need a wider search before it is claimed.

References for this section.

[10] M. Kaess, H. Johannsson, R. Roberts, V. Ila, J. Leonard, F. Dellaert, "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree", The International Journal of Robotics Research, vol. 31, pp. 217 to 236, 2012. Cited from memory. The conference version, "iSAM2: Incremental smoothing and mapping with fluid relinearization and incremental variable reordering", IEEE ICRA 2011, is confirmed in the reference list of [11].

[11] S. Williams, V. Indelman, M. Kaess, R. Roberts, J. J. Leonard, F. Dellaert, "Concurrent Filtering and Smoothing: A Parallel Architecture for Real-Time Navigation and Full Smoothing", The International Journal of Robotics Research, vol. 33, no. 12, pp. 1544 to 1568, 2014. Abstract and repository records read on 2026-11-06, and §5.1 and §5.3 with eq. 7, 15 and 17 read on 2026-11-07 from https://people.csail.mit.edu/kaess/pub/Williams14ijrr.pdf.

[12] S. Leutenegger, "OKVIS2: Realtime Scalable Visual-Inertial SLAM with Loop Closure", arXiv:2202.09199, 2022. The pose graph and marginalisation passages read on 2026-11-06, and §V-B, §V-D with eq. 4 and 7 to 20 read on 2026-11-07, at https://ar5iv.labs.arxiv.org/html/2202.09199.

[13] T. Qin, P. Li, S. Shen, "VINS-Mono", IEEE Transactions on Robotics, 2018, arXiv:1708.03852, §VI-D read on 2026-11-06.

[14] J. Vallvé, J. Solà, J. Andrade-Cetto, "Graph SLAM Sparsification with Populated Topologies Using Factor Descent Optimization", IEEE Robotics and Automation Letters, vol. 3, no. 2, pp. 1322 to 1329, 2018. Abstract and search snippets read on 2026-11-06.

[15] H. Liu, M. Chen, G. Zhang, H. Bao, Y. Bao, "ICE-BA: Incremental, Consistent and Efficient Bundle Adjustment for Visual-Inertial SLAM", IEEE/CVF CVPR 2018, pp. 1974 to 1982. Abstract and repository README read on 2026-11-06, and §5 with eq. 15 to 21 and Fig. 5 read on 2026-11-07 from https://openaccess.thecvf.com/content_cvpr_2018/papers/Liu_ICE-BA_Incremental_Consistent_CVPR_2018_paper.pdf.

[16] G. P. Huang, A. I. Mourikis, S. I. Roumeliotis, "Analysis and Improvement of the Consistency of Extended Kalman Filter based SLAM", IEEE ICRA 2008. Abstract read.

[17] G. P. Huang, A. I. Mourikis, S. I. Roumeliotis, "An Observability-Constrained Sliding Window Filter for SLAM", IEEE/RSJ IROS 2011, and T.-C. Dong-Si, A. I. Mourikis, "Motion Tracking with Fixed-lag Smoothing: Algorithm and Consistency Analysis", IEEE ICRA 2011. Abstracts and search summaries read on 2026-11-06.

[18] C. Chen, Y. Yang, P. Geneva, G. Huang, "FEJ2: A Consistent Visual-Inertial State Estimator Design", IEEE ICRA 2022, pp. 9506 to 9512. Abstract read on 2026-11-06, and §I to §III with eq. 1 to 32 read on 2026-11-07 from https://par.nsf.gov/servlets/purl/10376064. The first author's given name is Chuchu.

[19] J. Huai, Y. Lin, Y. Zhuang, M. Shi, "Consistent Right-Invariant Fixed-Lag Smoother with Application to Visual Inertial SLAM", AAAI 2021, arXiv:2102.08596. Abstract read on 2026-11-06.

[20] P. van Goor, R. Mahony, "EqVIO: An Equivariant Filter for Visual Inertial Odometry", IEEE Transactions on Robotics, 2023, arXiv:2205.01980. Abstract read on 2026-11-06.

[21] Z. Huai, G. Huang, "Robocentric Visual-Inertial Odometry", The International Journal of Robotics Research, vol. 41, no. 7, pp. 667 to 689, 2022. Abstract read on 2026-11-06.

[21b] Z. Huai, G. Huang, "Robocentric Visual-Inertial Odometry", IEEE/RSJ IROS 2018, arXiv:1805.04031. §III-A, §III-D and §IV read on 2026-11-07 at https://ar5iv.labs.arxiv.org/html/1805.04031 through a summary of the rendered page, not verbatim.

## 7. Delayed marginalisation implemented in a scratch copy and replayed (added 2026-11-10)

`plans/delayed_marginalisation.md` implements §6.3 for the period, applied to a scratch copy of the tree, built with tests and replayed on `/ws/data/telemetry_40`. The plan holds the code, the derivations and the run table, and the facts a later session needs are these.

1. In Basalt the per frame re-centring carries every state, prior and first estimate exactly, so the only inconsistency of a prior after a scale change is the anchor of its block. A readvance is therefore a re-anchoring of the block in the stored inertial rows at the current block, with every state kept at its first estimate (plan §2.3.2). The visual rows can stay linear and scale free, mapped through their epoch when read, because landmark eliminated monocular rows carry scale only through the lever arm (§15.6 of `context/vio_opt.md`).
2. A readvance at the main prior's own anchor reproduced the main prior's information to a relative $3 \times 10^{-15}$ at the first step and $3 \times 10^{-5}$ to $10^{-3}$ with up to 200 steps on the ground (run `/ws/logs/slam/2026-11-10_17-33-07`), the remainder being the dropped lever arm column. The store's bookkeeping is therefore correct on real data.
3. The residuals differed by the mapper's pose updates, as the corrected §6.3 Q4 above records. With pose updates applied, the delayed mode collapsed the scale to ratios of 0.0 to 0.3 in three replays. With no pose updates at all, the dynamic mode, the delayed mode and the delayed mode with smoothing all held scale ratios of 0.78 to 0.98 with tilt errors below 1°. With pose updates held while the store is active, the delayed mode needed one replacement and ended at 0.84 to 1.85, against 3.2 to 5.8 for dynamic marginalisation on the same binary, one of whose runs also diverged in tilt to 25°.
4. The mapper's pose updates also degraded the dynamic mode, 3.2 to 5.8 with updates against 0.89 without, and the held runs drifted after the store was released at about 66 s. This supports the hypothesis of §6.17 that the visual only mapper's pose feedback conflicts with the period's inertial estimate, and holding the updates for the whole period is the next experiment (plan §3.4.4 item 4).
5. A single prior that is never replaced, the delayed mode with its thresholds at $10^9$, diverged in scale and tilt (scale ratio 12 to 94, tilt near 90°), which confirms that the frozen bilinear error of §1.3 needs some form of relinearisation.
6. Timing. A readvance step costs one square root QR of about 135 to 171 rows by 78 columns, 0.05 ms to 0.065 ms, so a full store of 200 steps readvances in 9 ms to 13.6 ms, and with the smoothing pass of plan §2.3.6 the installed jobs took up to 28.6 ms. In a worker thread the VIO thread pays 0.04 ms to 0.2 ms per installation and 0.012 ms per frame at the median for the store, and `marginalize` is cheaper than with three priors, 0.29 ms to 0.38 ms against 0.53 ms to 0.62 ms at the median.
7. DM-VIO's code stops marginalisation replacements once at least $0.5\,d$ steps have passed since the last one, a count rather than the fraction of the previous prior's inertial factors described in the paper (`IMUInitializerTransitions.cpp:143-163` of the repository cloned at `e306328`), and its default thresholds are `threshScale = 1.02`, `threshGravdir = 1000` (off), `coarseScaleUncertaintyThresh = 1.0`, `scaleUncertaintyThresh = 1.0` and `reinitScaleUncertaintyThresh = 0.5` on the variance of the log scale, and `delay = 100`.


## 8. Holding the gravity direction, a scale only block (added 2026-11-11)

`plans/gravity_scale_tilt_optional.md` adds `vio_gs_optimise_gravity_dir` (default true), which with false holds the two tilt coordinates of the block and leaves $\sigma$ free. Five findings were established while planning it and are recorded here.

1. VI-DSO and DM-VIO both optimise the gravity direction. VI-DSO relates the visual and metric frames by a translation free similarity $\mathbf{T}_{m\_d}$, scale and rotation [1, §III-E], and DM-VIO optimises scale and gravity direction "as explicit variables" [2, §III-B]. Only VI-DSO's dynamic marginalisation is scale only, and `UpdateScaleInterval` already tracks $\sigma$ alone. A scale only block therefore reproduces neither paper. It keeps the world gravity aligned, as Basalt is outside the period.
2. Holding the tilt is the tilt half of the fixation of the first plan §2.5.4 item 4. The selection of the two tilt rows gives $\mathbf{S}_\theta \mathbf{N}$ of rank 2 with perfect conditioning on $\mathbf{n}_{\mathbf{e}_x}$, $\mathbf{n}_{\mathbf{e}_y}$ and a zero column for $\mathbf{n}_\sigma$, so the projector reduces to $\mathbf{n}_\sigma$ alone. Since the tilt rows of $\mathbf{n}_\sigma$ are zero, the projection and the restriction to $\boldsymbol{\delta}_\theta = \mathbf{0}$ commute. It is also the tilt half of §4.5.4's proposal, applied for the whole period.
3. With the tilt held the block's direction is exactly $\hat{\mathbf{g}}$ at every frame. `g.normalized()` of $(0, 0, -9.81)^\top$ is exactly $(0, 0, -1)^\top$ because IEEE square root returns $|x|$ for $\sqrt{x^2}$, `FromTwoVectors` of equal vectors is the exact identity, `SO3::exp` of a zero vector is the exact identity, and `Reanchor` with equal directions computes a zero axis. The only writers of the block's direction are `ApplyInc`, `TransformLin` and `Reanchor` (`imu_types.h:357-391`), all three of which then leave it unchanged.
4. The tilt and horizontal accelerometer bias null direction of plan 2 §3.2.3 involves only the frame attitudes and the biases, so it survives the flag, and Change A's bias anchor stays relevant.
5. Tilt corrections are then carried by the frame attitudes, and for frames frozen in a prior they accumulate in offsets from first estimates, the tilt analogue of root cause R5. The scale correction and the swap rule are unchanged.

No replay had been run as of 2026-11-11, because the container lacked CMake, colcon, ROS and OpenCV that day. The comparison against the dynamic mode baseline of §7 (`17-17-31`, `17-50-04`, `17-50-08`) is the plan's §5.3.

## 9. Live runs with the tilt held, compared with the tilt free live runs (added 2026-11-11)

Two live `sitl2` flights ran on 2026-11-11 with `vio_gs_optimise_gravity_dir` false and otherwise the configuration of `data/sitl_config_vo.json` (Changes A and B on, permanent period), logged under `/ws/logs/slam/2026-11-11_17-37-47` (recorded to `/ws/data/telemetry_85`) and `/ws/logs/slam/2026-11-11_17-46-58` (`telemetry_86`). They are compared with the thirteen live runs of 2026-11-06 23:14 to 2026-11-07 00:37 with the same configuration and the tilt free, none of which had been analysed before except the first three (§5). The runs from 2026-11-07 00:15 onwards postdate the anchor fix of plan 2 §5.5 item 6, whose replay is `00-06-42`. The measures are those of §7, the scale ratio over 2 s and the tilt error between estimated and true body frame gravity directions, both from `gt_eval.csv`, whose quaternions are in Eigen's $(x, y, z, w)$ order and whose ground truth starts at the estimate's initial pose, so no frame flip is needed. Lift off is taken where the true height first exceeds 0.15 m. The analysis scripts were kept in the session scratchpad and are not part of the repository.

### 9.1 Flight profiles

The live missions fall into three profiles, identified from the true height at 1, 2, 4 and 10 s after lift off and the peak acceleration of the first 3 s.

| Profile | Height at 2 s | Peak height | Peak acceleration | Tilt free runs | Tilt free outcome |
|---|---|---|---|---|---|
| P1, hop to 1 m then a slow climb with 5 m to 6 m of horizontal motion | 1.1 m | 10 m to 150 m | 1.1 to 1.5 m/s² | `23-14-47`, `23-18-44`, `23-42-49`, `23-52-04`, `23-54-43`, `00-15-29`, `00-20-26`, `00-37-18` | 5 diverged, `23-52-04` marginal (tilt to 20°), `00-15-29` and `00-20-26` bounded at scale ratios of 0.11 to 0.22 |
| P2, vertical climb to 10 m | 5 m | 10 m | 3.3 to 3.6 m/s² | `23-21-18`, `23-29-34` | 2 diverged |
| P3, fast climb to 150 m | 5.5 m to 6.2 m | 150 m | 3.5 to 4.3 m/s² | `23-33-43`, `00-28-15` | 2 bounded, scale ratio 0.9 to 1.0, tilt peak 8.5° to 13.8° |

`17-37-47` is P3 with a 0.14 m pre hop 1.5 s before the climb, which released the zero velocity residual for good through the start latch. `17-46-58` is intermediate, 2.2 m at 2 s and 2.8 m/s² peak, and climbs to 142 m.

### 9.2 Results with the tilt held

| Run | Scale ratio at 3, 10, 20, 30 s | Tilt error at 2, 5, 10 s | Peak tilt error | Block scale | Swaps |
|---|---|---|---|---|---|
| `17-37-47` | 0.59, 0.70, 0.70, 0.81, then 0.9 to 1.07 in the cruise | 8.4°, 3.4°, 1.9° | 9.8° in the climb, 13.3° at landing | 1.0 to 0.58 in the pre hop and first 0.3 s, then 0.63, then 0.345 at landing | 17 by 2 s, 24 in all |
| `17-46-58` | 0.94, 0.97, 1.07, 1.14 | 8.6°, 14.2°, 5.3° | 18.0° | 1.0 to 0.62 in the first second, then back to 0.86 between 2 s and 3 s | 28 |

Neither run diverged, and the tilt error returned below 2° within 10 s to 20 s in both. `17-37-47` ended its climb 40 m low (110 m against 150 m), entirely from the first 20 s, after which its relative scale was correct for the cruise and descent. At touchdown a run of seven negative $\sigma$ steps summing to $-0.58$ with swaps rescaled every position about the world origin, which moved the estimated height from $-36.6$ m to $-20.9$ m in 0.3 s, and the horizontal accelerometer bias rose to 1.7 m/s². Feature tracking was normal throughout (150 to 190 tracks), while the departing inertial factors' whitened scale column rose to 210 to 445 and the solver stopped converging, so the landing is a second instance of F1.

### 9.3 What the comparison shows and what it does not

1. Both held runs fly profiles on which the tilt free estimator never failed. Holding the tilt has not yet been tested on P1 or P2, so the absence of divergence on 2026-11-11 is not yet evidence that holding the tilt prevents it.
2. The block's cumulative scale and tilt are gauge quantities and do not measure the physical error. In `00-15-29` the block's tilt reached 44° with a physical tilt error of 1.4°, and in `00-28-15` the block's scale reached 0.34 with a physical scale ratio of 0.9 to 1.0, because the frame states carry their own share of each correction.
3. Holding the tilt removes the cumulative world rotation, which reached 39° to 96° in five tilt free runs (`23-18-44`, `23-21-18`, `23-29-34`, `23-42-49`, `23-52-04`) and accompanied physical tilt errors of 20° to 85°. It does not remove the physical tilt freedom of the window's attitudes against a fixed gravity, which reached 13.6° within 0.5 s of lift off in `17-46-58` while the horizontal bias was still 0.05 m/s². That rotation is not the tilt and bias null direction of plan 2 §3.2.3, which the bias anchor L3 resists. It is the window absorbing a kinematically inconsistent acceleration, and after the swaps of the first second only the window's own inertial factors oppose it, because L4 is lost at every swap (§6.1.2, `visual_imu_full` is 0 in flight in both runs).
4. Both post fix tilt free failures were scale runaways, which holding the tilt does not address. In `00-37-18` the block's scale rose to 32 through 17 to 19 positive steps per second with 10 to 14 swaps per second while the block's tilt stayed below 1°, and in `00-20-26` it fell to 0.02 the same way with jump costs of up to 144.
5. Every scale event in the held runs, the pre hop and lift off of `17-37-47`, the first second of `17-46-58` and the landing of `17-37-47`, has the structure of F2 at a smaller amplitude, a run of one sign steps each within Change B's tolerance or overriding it at jump costs of 5 to 25, with swaps every second or third frame, coinciding with whitened scale columns of 80 to 450 (F1). The scale recovers only when a consistent acceleration follows, as between 2 s and 3 s of `17-46-58`.
6. The mapper's pose updates are rejected by the estimator's gate (`kRelinThresholdTrans` 0.1 m, `kRelinThresholdRot` 0.05 rad, `sqrt_keypoint_vio.cpp:633-641`) through most of the take off transient of both held runs, 31 to 49 rejections per second, so they did not drive these events.

7. Change B is the only limit on the size of a correction, and nothing clamps the correction that the re-centring applies. Measured against its nominal per frame standard deviation $0.1\sqrt{\Delta t} \approx 0.023$ in $\sigma$, the pre hop and lift off of `17-37-47` moved $\sigma$ by $-0.52$ in 38 frames with only one step above $3$ standard deviations and a largest jump cost of 7.1, the first second of `17-46-58` by $-0.49$ in 20 frames with four steps above 3 and a largest cost of 24, the landing of `17-37-47` by $-0.56$ in 11 frames with five steps above 3, and the tilt free runaway of `00-37-18` by $+2.86$ between 2 s and 6 s with 23 steps above 3 and a largest cost of 38. The residual has no memory across frames, so a sum of $-0.52$ over 2 s, about 3.7 times the $0.1\sqrt{2} \approx 0.14$ that the random walk itself would allow, costs no more than its individual steps. Near the origin the projection also shrinks its weight by $1 - 1/|\mathbf{n}_\sigma|^2$ (plan 2 §3.4.3), which a rough bound from the newest state's position and velocity puts at 0.01 to 0.69 in the pre hop of `17-37-47` and 0.12 to 0.64 in the first second of `17-46-58`. With the tilt held its tilt term is inert, so no residual limits the attitude rotation of §9.3 item 3.

### 9.4 Constraints implied for the gravity direction

With the tilt held, a further constraint on gravity can only act on the attitudes of the window against the fixed gravity. The evidence of §9.3 item 3 points to §6.19 Formulation 2, the body frame gravity prior on the first state, which with $\mathbf{d} = \hat{\mathbf{g}}$ fixed reduces to $\mathbf{r}_g = \mathbf{W}_g \lfloor \mathbf{R}_0^\top \hat{\mathbf{g}} - \mathbf{z}_0 \rfloor_{xy}$ with no block Jacobian. It has no scale column, so it is admissible in $M_\text{visual}$ and survives every swap, restoring L4. Against the 13.6° rotation of `17-46-58` a 3° prior costs $\frac{1}{2}(13.6/3)^2 \approx 10$, comparable to the jump costs of the same frames, and a 1° prior about 92. Neither constrains the scale events of §9.3 items 4 and 5, which need a bound on the swap ratchet or on the cumulative scale, or delayed marginalisation, and the residual can only move F1 into another coordinate (§6.19, Step 4 of the weight derivation).

## 10. Update cadence of the block (added 2026-11-11)

`plans/gravity_scale_update_cadence.md` adds `vio_gs_update_every_n_frames` (default 1). With $N > 1$ the block is held before the first frame of motion, which is the first frame with `mpIsStationary` false, and afterwards free only on frames $k = N, 2N, \dots$ of a count that starts at 1 on that frame and never resets. It was motivated by §9.3 item 5, where every scale event was a run of one sign steps with a swap every second or third frame.

1. A fully held block is the fixation of the first plan §2.5.4 item 4, $\mathbf{S}_{gs} \mathbf{N}$ of rank 3, so a held frame skips the projector. Projecting would remove the uniform scaling and joint tilt of the frames from the step, which are physical when the block cannot carry them. The held frame is today's estimator outside the period.
2. On a held frame the re-centring applies the block's leftover from the previous re-centring, zero to about $10^{-17}$, so it is an identity to rounding, exact for the rotation when the tilt is held.
3. `UpdateScaleInterval` sees no change of $\sigma$ on a held frame, but `mpLogD` falls back to $\ln d_\text{min}$ there, so after a large update the following held frames fire catch up swaps until the interval's centre is within $\ln d_\text{min}$ of $\sigma$. Swaps are triggered only by updates and their catch up, which limits the ratchet to the update rate but does not confine swaps to free frames. Note, 2026-11-11. This item previously stated that no swap can fire on a held frame, which the live run `/ws/logs/slam/2026-11-11_19-43-15` (N = 10) contradicted with 3 of its 10 swaps on held frames, because `mpLogD` is recomputed from the current frame's step and falls back to $\ln d_\text{min}$.
4. Change B uses the interval $N \Delta t$ on a free frame, because under its random walk model the step summarises $N$ frames. Its allowance per unit time is therefore unchanged, and the cadence alone does not tighten the rate. To tighten it, lower `vio_gs_jump_sigma_rw` as well.
5. During held frames any correction is carried by the frame states, and for frozen frames it accumulates in offsets from first estimates (root cause R5) until the next free frame lets the block take it over.

No replay had been run as of 2026-11-11. The comparison is item 4 of the plan's §5, replays of `telemetry_85` and `telemetry_86` with $N \in \{1, 5, 10, 20\}$.

## 11. Live run with the cadence and gravity optimised, `2026-11-11_19-43-15` (added 2026-11-11)

The run used $N = 10$, `vio_gs_optimise_gravity_dir` true, `vio_gs_jump_sigma_rw` 0.1 and `vio_gs_jump_tilt_rw_deg` 1.0, on the P3 profile of §9.1 (5 m at 2 s, 37 m at 10 s, 68 m peak, then hover, descent and landing). The schedule worked as designed, one update every 10 frames, 1.9 per second, with held frames showing the rounding residue of §10 item 2 (`tilt_step_deg` about $10^{-6}$, because `acos` of a dot product near 1 amplifies rounding, so a threshold of $10^{-3}$ degrees separates the frames).

### 11.1 Scale

Six updates between 1.7 s and 4.3 s moved $\sigma$ by $-0.07$, $-0.09$, $+0.11$, $-0.16$, $-0.14$ and $+0.04$, after which the block's scale stayed at 0.70 for the rest of the run. The estimate was over scaled by about 1.3 to 1.36 from then on (91 m estimated against 67 m true at the top of the climb), with a median 2 s ratio of 1.8 between 5 s and 30 s. On the same profile with gravity optimised, `19-18-06` ($N = 1$, scale intensity 0.05) had 7.3 and `19-37-59` ($N = 5$, 0.75) had 4.0, so this run's scale is the best of the three, but the intensities differ, so the effect of $N$ is not isolated. The tilt held runs on this profile had 0.6 to 0.8 (`17-37-47`) and about 1.0 (`17-46-58`).

### 11.2 Orientation

1. At 3.86 s, a held frame following two unconverged solves (5 and 6 rejected iterations), the estimated body frame gravity rotated by 31.6° in one solve while the position moved 0.16 m and the true tilt stayed at 4.5°. The same frame rejected mapper corrections of up to 0.66 m and 11.8°. The physical tilt error went from 10.5° to 42.7°. This is the window's attitudes rotating against a fixed gravity, as in §9.3 item 3, and not a block update.
2. The following updates rotated the block by 12.5°, 7.0°, 7.7°, 6.7°, 5.5°, 5.3°, 5.1°, 4.4°, 4.8° and 4.5°, and the tilt error fell by only 1.5° to 2.5° per update, from 42.7° to 2.5° at 9 s. The block's cumulative rotation reached 101.5° by 20 s and 105° by the end, while the estimated yaw stayed within 4.4° of the truth until landing. Most of each update's rotation therefore does not correct the attitude.
3. Every update rotates every position about the world origin through the re-centring. Across 121 updates between 2 s and 65 s the position jump equals $|\mathbf{p}| \cdot \theta$, with $\theta$ the block's tilt step, at a correlation of 0.96 and a slope of 0.93, 2.3 m to 3.8 m per update between 2 s and 12 s, against 0.002 m to 0.02 m on held frames. The direction of the estimated position from the origin jumps by about 4.5° at each update and relaxes by about 2.4° over the nine held frames, so it accumulates about 2.5° per update during the climb (1.8°, 5.8°, 9.2°, 11.6°, 14.0° at successive updates between 4.9 s and 7.0 s). This staircase is the rugged trajectory.
4. The total rotation does not depend on $N$. Gravity optimised runs with $N = 1$ accumulated 38° to 91° of block rotation in the first 18 s (`19-21-27`, `19-25-50`, `17-56-00`) in steps of 0.1° to 0.3° per frame and position jumps of 0.05 m to 0.2 m per frame. $N = 10$ packs the same total into steps of 2.9° on average per update.
5. After the climb the physical tilt error settled at 9° to 10°, with the horizontal accelerometer bias rising from 0.5 to 1.67 m/s² (6.5° to 8.1° equivalent), correlation 0.56 between the tilt error and the bias equivalent tilt, the tilt and bias trade of plan 2 §3.2.3. The velocity direction error matched the tilt error (median difference $-0.8°$). At landing the tilt error reached 15.6° and the yaw error 15°.
6. The run had 10 swaps, 7 on update frames and 3 on the held frames after them, which corrected §10 item 3.

### 11.3 What a larger $N$ would change

An update's solve sees the same window and priors whatever $N$ is, since held frames do not accumulate evidence for the block, they put any correction into the frame states and the marginalised priors, which accumulate at every frame regardless of $N$. A larger $N$ therefore does not average more data into each update. It delays the first update ($N = 20$ is about 1.1 s after motion), packs the total into fewer and larger steps, and through Change B's interval of $N \Delta t$ allows each update $\sqrt{N}$ times the standard deviation. By item 3 the position jump grows in proportion to the step and to the distance from the origin.

### 11.4 Three further cadence runs, $N = 10$, gravity optimised (added 2026-11-11)

| Run | Profile (§9.1) | Scale intensity | Peak tilt error | Block rotation by 20 s | Scale ratio | Where the scale error arose |
|---|---|---|---|---|---|---|
| `19-49-07` | P2, vertical to 10 m | 0.1 | 8.9° | 15° | 1.3 at 2 s, then the block's scale grew from 1.23 to 2.25 in the hover from 4 s to 12 s | Updates of $-0.08$, $-0.07$, $-0.04$ in the climb were right in sign but too small, each near Change B's $0.1\sqrt{0.54} \approx 0.073$, while the states' velocity grew to 1.5 times the truth. In the hover the block drifted up by $+0.13$ to $+0.17$ per second, and the re-centring about the origin turned it into a phantom climb from 24 m to 46 m with the true height at 10 m and the estimated velocity only 0.3 m/s |
| `19-51-16` | P1, hop then climb | 0.75 | 20.4°, at landing | 4° | 2.3 to 3.0 from 1 s | On held frames, 0.26 s to 0.36 s after lift off, the estimated vertical velocity rose from 0.49 to 1.68 m/s against a true 0.58 m/s, the frame states rescaling the window as in Basalt without the period. The block's updates stayed small ($-0.014$ to $-0.025$) and never removed it |
| `19-54-39` | P3, fast climb | 0.75 | 10.4° | 35° | 1.44 to 1.66 throughout | The first update fell at 0.05 s after lift off, because motion was detected at spool up 0.4 s earlier, and applied $+0.235$ at a jump cost of 0.10. The next three updates came with departing scale columns of 190 to 670 |

None of the three diverged, including the P1 and P2 profiles on which tilt free runs with $N = 1$ diverged in 7 of 10 cases (§9.1), but all three were over scaled by 1.4 to 3. The tilt stayed within 10° in flight in all three, and the large block rotations of `19-43-15` (§11.2) did not recur, which ties that run's 100° to the 31° held frame attitude jump it had to repair. With $N = 10$ and the count started by the stationarity test, the first update lands about 0.5 s after the spool up, inside the lift off transient, so the cadence did not keep the first alignment out of the inconsistent window. Between updates the frame states carry the scale with Basalt's own behaviour, which over scales by 2 to 4 without the period (§1.2), and the block's later updates, limited per update by Change B and seeing the same window as at $N = 1$, did not correct it.

## 12. Gravity prior and accelerometer bias step prior (added 2026-11-11)

The live runs from 19:54 to 20:33 on 2026-11-11 moved the accelerometer bias by 0.8 to 4.2 m/s² within one second, 700 to 3800 times the one second spread of the random walk (`accel_bias_std` 0.001132 m/s²/√s), with per frame changes of 190σ to 1500σ at the 99th percentile. A difference residual of that weight would never be violated so far, so the movement is a common shift of every bias state of the window, invisible to the random walk and resisted only by the absolute bias anchor. Lowering `accel_bias_std` therefore cannot bound it, and `accel_noise_std` weights the preintegrated measurements, not the bias.

`../plans/gravity_and_bias_step_priors.md` added two optional priors, both off by default. `vio_ba_step_prior_std` is a frame local residual on each window state's accelerometer bias against its value at the start of the frame's solve, which bounds the common shift per frame at $q_b\sqrt{\Delta t}$, and through the trade of plan 2 §3.2.3 the tilt it carries. Its bias coordinates are absent from every gauge vector, so the projection, the cadence's held frames and the tilt hold leave it unaffected. `vio_gravity_prior_std_deg` is §6.19 Formulation 2 in its linearised world frame form, two rows of the initial prior on the first state's roll and pitch, coupled to the block's tilt in the period so that they are gauge invariant, restoring L4 through every swap when Change A is on. The weight derivation of §6.19 applies, 3° being a plausibility bound and 1° substantial information against the bias anchor's 9600 rad⁻².

By 20:33 the user had also set `vio_init_bg_weight` to $10^4$ in `data/sitl_config_vo.json`, which by the index swap of `vio_residuals_and_priors.md` tightens the accelerometer bias anchor to 0.01 m/s². No run with either new prior had been made as of 2026-11-11.


## 13. The period's duration counted from take-off (added 2026-11-12)

`../plans/gravity_scale_timer_from_motion.md` changed the meaning of `vio_gs_duration_s`. Before 2026-11-12 the duration was measured from the first state, the timestamp `SetInitialPriorOrder` stored in `mpGsStartNs`, so the time on the ground before take-off consumed part of the period. Since then `mpGsStartNs` stays at -1 until the first frame of confirmed motion, which `processFrame` records right after the start latch update, and the fold check in `marginalize` requires it to be non-negative. Confirmed motion means `mpIsStationary` false and, when `vio_stationary_enable_start_latch` is set, the latch closed, so a single noisy ground frame cannot start the timer and the start lags the true take-off by `vio_stationary_latch_frames` frames. With every stationary stage disabled, the flag is always false and the old meaning returns. A vehicle that never moves never folds. The user set `vio_gs_duration_s` to 2 s in `data/sitl_config_vo.json` to make the period a very short phase after lift off.

The timer's take-off and the cadence's motion start of §10 differ. `mpGsMotionStarted` takes the first non-stationary frame without the latch and counts only when N > 1. With the latch on, a ground miss can therefore start the cadence's count before the period's timer. Any comparison of fold times with runs made before 2026-11-12 has to account for the different reference.

## 14. Scale loss across take-offs, the runs of 2026-11-11 and 2026-11-12 (added 2026-11-12)

The user reported that the scale was lost within seconds of take-off in the runs of 2026-11-12, including the two with the gravity direction held, and named `2026-11-11_20-38-15` as the reference with good scale. Twenty six runs were compared. Most of their directories were deleted during the analysis, so the numbers below are the only record. The scale ratio is the length of the estimated displacement over a 1 s window divided by that of ground truth, and the median over windows ending 3 to 10 s after lift off summarises a run. The analysis scripts are `tab.py`, `absorb.py`, `tko.py` and `excess.py` in the session scratchpad.

### 14.1 The flight profile separates the outcomes, the configuration does not

| Runs | Profile | Configuration | Ratio, 3 to 10 s |
|---|---|---|---|
| `20-38-15` (reference), `20-05-19`, `20-22-09`, `19-54-39` | P3, 150 m | $N = 10$, `vio_init_bg_weight` 100, no new priors | 1.06 to 1.51 |
| `20-17-25`, `20-33-52` | 41 m and 22 m | that of `20-38-15`, differing only in `vio_gs_jump_tilt_rw_deg` and `vio_gs_jump_sigma_rw` | 4.21 and 3.85 |
| `20-48-31`, `20-51-36` | 26 m and 61 m | `vio_init_bg_weight` $10^3$ and $10^4$ | 5.22 and 4.99 |
| `21-12-19`, `21-29-44`, `21-38-48`, `17-55-22`, `21-37-22` | P3, 123 to 150 m | gravity prior 0.75° to 1°, bias anchor $10^3$ to $10^4$ | 0.76 to 1.87 |
| `18-00-58`, `18-04-59`, `21-22-00`, `21-53-09` | 51 to 95 m | mixed | 0.67 to 0.85 |
| `17-43-05`, `17-45-50`, `17-52-42`, `21-26-58`, `21-31-50`, `21-34-29`, `21-56-53`, `21-59-33` | 10 to 39 m | gravity prior, $N$ 1 to 15, tilt held (`21-31-50`, `21-34-29`) and free | 1.91 to 6.49 |

Every run with a ceiling of 41 m or less over scaled by at least 1.9 under every configuration, and the configuration of the reference produced 3.85 to 4.21 on that profile. The reference's good scale is therefore not attributable to its configuration, and the two tilt held runs, both short climbs, are no evidence against holding the tilt. The ratio already departs in the first 1 to 1.5 s, while the ground truth kinematics of `20-17-25` and `20-38-15` are alike (vertical speed 1.7, 2.1, 4.0 and 3.6 m/s at 0.5, 1, 2 and 3 s against 1.8, 1.4, 4.7 and 3.9, horizontal displacement at 3 s of 1.87 m against 1.66 m, initial yaw 0.69 against 0.83 rad), so what makes the short profile fail early is not identified. Five of the short runs began with a slow climb of 0.4 to 0.6 m/s at 1 s, the slow take-off case of `takeoff_bias_divergence.md` §9. A correlation of the scale ratio with the maximum tilt or horizontal bias in the first 2 s, across all runs, was weak ($-0.13$ and $0.08$).

### 14.2 The estimator departs from the inertial data, not the data from the truth

For each frame, the vertical velocity change predicted from the frame's mean specific force, the estimated attitude and bias, $\mathbf{R} (\bar{\mathbf{a}} - \mathbf{b}_a) + \mathbf{g}$ times `meas_dt` (milliseconds in `vio_imu_frame.csv`), was compared with the change between consecutive logged newest states and with ground truth, over 0.5 s windows from 0.5 s before to 5 s after lift off. In all eight runs still on disk the inertial prediction matches ground truth to 0.1 to 0.3 m/s per window, so the inertial data, the attitude and the bias are not the source. The estimator's own change departs from both, by $+2.7$ m/s over the first 3 s in `21-37-22` (6.5 m/s estimated against 4.0 true), and by single frame excesses of up to 2.8 m/s in `21-38-48` and 6.3 m/s in `20-22-09`. The excess is the revision of the previous state's velocity by the next solve, which the inertial factors do not resist strongly enough.

The excess concentrates on swap frames. Its mean magnitude on frames with a swap event was 0.21 against 0.085 m/s in `21-37-22`, 0.70 against 0.15 m/s in `21-38-48` and 0.37 against 0.25 m/s in `20-22-09`, and 0.078 against 0.062 m/s in the reference. In the reference the excess correlates with the block's scale step (0.80), so its velocity changes are the block's intended rescalings. In the gravity prior runs the large excesses fall on frames with a zero or negligible scale step, so the states rescale outside the block. The configuration is monocular (one camera in `calibration.json`), so vision carries no scale, and `vio_max_states` 3 leaves two inertial factors spanning about 0.1 s in the window. The scale is therefore held almost entirely by the marginalisation prior, which every swap replaces with one built from scale free rows.

### 14.3 The lift off inconsistency takes the weakest outlet

The reference absorbed the lift off inconsistency F1 into the tilt, 23.8° at 0.8 s with a horizontal world bias of 1.56 m/s², the trade of `takeoff_bias_divergence.md`, which costs nothing in scale on a near vertical climb. `20-17-25`, with the same configuration, absorbed it into the vertical world bias, from $-0.24$ to $-3.4$ m/s² between 0.7 and 2.5 s, with speed ratios of 3 to 4, at a pre optimisation median reprojection error of 4.2 px and a triangulated landmark at 106 m at 0.6 s. Since `21-12-19` the gravity prior, the bias anchor of $10^3$ to $10^4$ and the bias step prior hold the tilt below 4° and the bias below 0.1 m/s² in the first 2 s of most runs, which closes the tilt outlet and leaves the scale. This matches the expectation recorded with the priors in §12, but the P3 runs alone (0.76 to 1.87 pinned, 1.06 to 1.51 unpinned) are too few and too varied to show it.

### 14.4 First map depth

Across 25 runs, the logarithm of the mean depth of the landmarks triangulated between 0.3 s after the first triangulation and 1 s after lift off correlates at 0.75 with the logarithm of the 3 to 10 s scale ratio, and at 0.54 for the first 0.3 s of triangulations. The slow take-offs built their first map at 4.6 to 9.2 m mean depth at 0.7 to 1.0 m of altitude. This is the finding of `takeoff_bias_divergence.md` §9, that the depths of the first map set the scale, now across the gravity and scale configurations.

### 14.5 Folding inside the acceleration phase

In `21-56-53` (`vio_gs_duration_s` 1.5 counted from take-off, §13) the block folded 1.22 s after lift off, in the acceleration phase at 3.2 m/s. During the period the ratio was 0.8 to 1.15 with the tilt below 1.2°. On the first frame after the fold the estimated vertical speed rose from 3.09 to 6.02 m/s against 3.33 true, and the ratio stayed at 1.8 to 2.3 afterwards. The period had seen 14 swaps, so the prior at the fold was largely rebuilt from scale free rows, and after the fold neither the projector nor Change B constrains the states' scale. `21-53-09` folded 1.54 s after lift off at 0.4 m/s without a jump and kept a ratio of 0.8. One instance does not establish the mechanism, but a period that ends during the take-off transient is a hazard of the short durations.

### 14.6 Not causes

The inertial coverage during take-off is 0.96 on average, with the stretch fallback in about half of the frames, in good and bad runs alike, and the inertial prediction matches ground truth, so the inertial ingestion is not a cause. The `vio_gravity_scale.csv` rows after the last flush are lost when a run is stopped, so a fold can be missing from the log, as in `21-59-33`.

### 14.7 Good scale against poor scale, the differences

The good group is the reference `20-38-15` with `20-05-19`, `20-22-09`, `21-29-44`, `18-00-58`, `18-04-59`, `21-22-00` and `21-53-09`, ratios 0.67 to 1.17. The poor group is every run with a ratio above 1.8, most of them the short climbs of §14.1.

| Aspect | Good scale | Poor scale | Discriminates |
|---|---|---|---|
| Ceiling of the flight | 51 to 151 m, a climb that continues with 1.1 to 1.8 m/s of horizontal speed at 15 s | 10 to 41 m in all but `20-51-36` (61 m), at most 0.2 m/s horizontal at 15 s in most | yes, the strongest separator found |
| Configuration | the reference's ($N = 10$, anchor 100, no new priors) and the pinned configurations | the same configurations, including the reference's in `20-17-25` and `20-33-52` | no |
| Gravity direction optimised or held | optimised in all good runs | held in `21-31-50` and `21-34-29`, both short climbs | not tested on a matched profile |
| Kinematics of the first 3 s | 1.4 to 2.7 m/s at 1 s, 3.6 to 4.2 m/s at 3 s on P3 | the same in `20-17-25`, `20-48-31`, `21-56-53`, and 0.4 to 0.6 m/s at 1 s in the five slow take-offs | only for the slow take-offs |
| When the ratio departs | stays within 0.5 to 1.3 from lift off | already 1.5 to 4 at 1 s | yes, the outcome is decided in the first 1 to 1.5 s |
| Where the lift off inconsistency goes | tilt and horizontal bias (reference, 23.8° and 1.56 m/s²) or the block's scale step | vertical world bias (`20-17-25`, down to $-3.4$ m/s²) or the states' scale outside the block (pinned runs) | yes, by mechanism |
| Tilt and horizontal bias in the first 2 s | 2° to 24°, up to 1.6 m/s² | 1° to 22°, up to 1.8 m/s² | no, as a scalar across runs |
| Inertial prediction against ground truth | within 0.1 to 0.3 m/s per 0.5 s | the same | no, the inertial data are sound in both |
| Estimator velocity in excess of the inertial prediction, first 3 s | $+1.1$ m/s in the reference, carried by the block's scale step (correlation 0.80) | $+2.7$ m/s in `21-37-22`, single frames of 0.6 to 2.8 m/s, on frames with no scale step | yes |
| Excess on swap frames against other frames | 0.078 against 0.062 m/s in the reference | 0.21 against 0.085 and 0.70 against 0.15 m/s | yes |
| Mean depth of the first map, 0.3 s after the first triangulation to 1 s after lift off | 0.5 to 2.5 m | 2.2 to 9.2 m, highest in the slow take-offs | yes, log correlation 0.75 with the ratio |
| Vision during lift off | flow 15 to 23 px, every frame at the iteration cap, median reprojection error spikes of 2 to 14 px | the same, plus a 106 m outlier landmark in `20-17-25` | no, present in both |
| End of the gravity and scale period | never (7000 s), or 10 to 26 s after lift off | 1.22 s after lift off in `21-56-53`, followed by a speed jump from 3.1 to 6.0 m/s in one frame | in that one run |
| Inertial coverage and stretch fallback | 0.96, stretch in about half of the frames | the same | no |

### 14.8 Open questions and suggested tests

1. What fails on the short profile during the first second, while the kinematics match P3, is not identified. The scene seen at lift off, which depends on the start location and is hidden by the normalised ground truth, was not compared. Logging the world position of the start, or comparing the first images, would test it.
2. `vio_max_states` 3 leaves the monocular estimator two inertial factors in the window. Raising it puts the take-off acceleration directly into the window instead of only into the prior, which is the most direct test of §14.2.
3. Compare configurations only on matched profiles. A short climb with the gravity prior and the bias anchor relaxed (the reference's configuration has already failed there), and a P3 flight with the tilt held, are the missing pairs.
4. End the period only after the acceleration phase, for example `vio_gs_duration_s` of 5 s or more from take-off, to test §14.5.
5. Suppress swaps while the take-off flow is above about 10 px, to test whether the swap frame excess of §14.2 is causal.

## 15. Delayed marginalisation as the period's only mechanism (added 2026-11-13)

`plans/delayed_marginalisation.md` Changes 12 to 15 put §7's scratch implementation into the working tree as the only mechanism of the period and removed the dynamic marginalisation of §1 to §5, the tilt hold of §8, the cadence of §10 and the window's accelerometer bias step prior of §12. The gravity prior of §12 is kept, and the timer of §13 now also gates the first coarse optimisation. The facts a later session needs are these.

1. Two phases. Until the store first holds $d$ steps, the block is held in the window and a coarse inertial optimisation in the worker corrects scale, tilt and accelerometer bias every $k$ marginalisations, each correction applied by `RecentreGravityScale` and followed by a readvance without smoothing. Afterwards the block is free with the gauge projection and Change B, a smoothed readvance starts at once, and the replacement rule of §6.3 applies. The store is never released, because the replays of §7 item 4 drifted after a release.
2. Departed keyframe states disagree with their neighbours by about 5 cm at rest in SITL, because the window keeps optimising a keyframe's pose after its neighbours have left. Per frame inertial factors evaluated at held states therefore carry whitened rotation residuals of 30 to 200 at the current values against 5 to 20 at the first estimates. Any estimator that holds the window's departed states and fits per frame inertial factors to them inherits this jitter, and with the scale free it prefers a scale near zero at rest, which collapsed the scale to $e^{-20}$ in the first replay.
3. The coarse optimisation chains the factors over spans of $k$ frames in the world frame of the held rotations, which rebuilds the keyframe preintegration of DM-VIO and ORB-SLAM3 from Basalt's per frame factors, and estimates the node velocities, common gyroscope and accelerometer bias increments, the tilt and the inverse scale with every pose held, after DM-VIO's coarse initialiser (arXiv:2201.04114 §III-F, `CoarseIMUInitOptimizer.cpp`) and the inertial only optimisation of Campos et al. (arXiv:2003.05766). The residuals of a span are $\mathbf{p}_b - \mathbf{p}_a - \mathbf{v}_a T - \varepsilon(\frac{1}{2}\mathbf{g}_D T^2 + \mathbf{P})$ and $\mathbf{v}_b - \mathbf{v}_a - \varepsilon(\mathbf{g}_D T + \mathbf{V})$, so the held positions stay the predicted side, whitened by the chained preintegration covariance plus $2\sigma_x^2$ with $\sigma_x$ = 2 cm on the position rows. Before 2026-11-14 it eliminated the velocities over three consecutive nodes instead, after Mur-Artal and Tardós (eq. 12 and 19 of arXiv:1610.05949), with triples below 0.2 m of visual displacement excluded. The visual side must be the predicted side, $\boldsymbol{\lambda} \approx e^{-\sigma}(\kappa\,\mathbf{g}_D + \boldsymbol{\gamma})$. With $\boldsymbol{\lambda}$ as the regressor the jitter attenuates the scale towards zero, and because each correction rescales the jitter too, repeated corrections compound the attenuation geometrically, 1 to 0.12 in 6 s on `telemetry_34` while the correct value was about 0.3. Note, 2026-11-14. This item previously described the three node elimination as the form that works. In a Monte Carlo of forty random trajectories over 200 frames the free velocity form halved its scale, tilt and bias errors, 1.3 % against 2.9 % root mean square scale error at 4 cm jitter, and over 41 frames with short spans the three node form was unstable (`plans/delayed_marginalisation.md` §3.4.6).
4. `IntegratedImuMeasurement` keeps `bias_accel_lin_` private. The bias corrected preintegrated deltas and their accelerometer bias Jacobians are read from `residualGS` at identity states, zero gravity and zero scale, whose position and velocity rows are then $-\hat{\Delta\mathbf{p}}$ and $-\hat{\Delta\mathbf{v}}$.
5. Replays of 2026-11-13. On `telemetry_34`, Basalt alone over scales by 3 to 6 in the first 30 s, while the implementation held the scale ratio at 0.88 to 1.10 from 5 s after lift off with tilt within 2.1° in three replays. On `telemetry_100`, Basalt alone diverged to 75° of tilt, and the implementation held 1.00 to 1.21 to 120 s with tilt within 1.8°. The live runs with the block free in the window had reached 8.8° to 23.8° of tilt, and the live run whose period folded 1 s after lift off stayed within 2.6°, which is the evidence that the initial gravity direction is accurate and should be constrained rather than freed.
6. Readvances fired on every frame for the last 2 s of `telemetry_34`, around landing, which is open.
7. The inverse scale $\varepsilon = \varepsilon_0(1 + \eta)$ is the coarse unknown because every translational row is then linear in it. With very precise inertial rows, a synthetic 0.01 m/s² per sample, the curvature of $e^{-\sigma}$ and of the tilt made undamped steps increase the cost and Levenberg–Marquardt crawled. At the SITL noise density of 0.08 m/s²/√Hz the same problem converges in three or four iterations. A unit test that uses Basalt's `integrate` with a tiny covariance therefore tests the solver on a far stiffer problem than the estimator meets, and the tests of the coarse optimisation use the noise densities of `data/sitl_calib.json` (added 2026-11-14).
8. During the coarse phase no step has reached the delayed prior $B$, which is still the initial prior, so a readvance from $B$ recomputes the entire main prior. Updating departed velocities and biases as offsets of frozen records and then readvancing therefore leaves no prior linearised at the old values, and the coarse job runs that readvance itself so that the correction and the prior are installed in one frame (`plans/delayed_marginalisation.md` §2.3.14, added 2026-11-14).
9. With one rotation held per frame, the gyroscope bias reaches the translational rows only through the rotation inside one 54 ms frame, about $1.4 \times 10^{-4}$ m/s per frame for 0.01 rad/s, so estimating it does not help the scale. It is estimated from the per frame rotation rows so that the readvance relinearises at a gyroscope bias that agrees with the held rotations (added 2026-11-14).
10. The triple form that items 3 and 7 replaced was overconfident by construction. Every span entered two or three overlapping triples whose residuals were whitened as independent, so the inertial terms and the node positions were counted several times, which is consistent with the scale variance being 4 to 27 times too small in the flights of `plans/delayed_marginalisation.md` §5.3. With explicit node velocities each span enters once. Eliminating the velocities exactly from the chained span equations gives the same $3N - 6$ constraints for $N$ nodes, $6(N - 1)$ equations against $3N$ velocities, but with their correct correlation (added 2026-11-14).
11. Monte Carlo of 2026-11-14 over forty random spline trajectories with the SITL noise densities, positions and velocities shrunk by 1.6 and Gaussian position jitter. Over 200 frames with $k$ = 10 the explicit form's root mean square log scale error was 0.0033, 0.0067 and 0.0134 at 1, 2 and 4 cm of jitter against 0.0070, 0.0140 and 0.0289 for the triple form, with mean errors of +0.0003 to +0.0013 against −0.0023 to −0.0098, and its tilt and accelerometer bias errors were also halved. Over 41 frames with $k$ = 4 the triple form was unstable, rms 0.53 to 296, while the explicit form degraded to 0.03 to 0.14 (`plans/delayed_marginalisation.md` §3.4.6).
12. The coarse scale gate reads the information at the snapshot, not at the free solution. On a nearly static trajectory the free solve drifts to a degenerate inverse scale, a log scale of about $-4.2$ on a spline with knots 10 s apart, where the magnified bias and tilt Jacobians shrink the apparent scale variance to about 1.6, against $1.3 \times 10^4$ at the snapshot (added 2026-11-14).
13. Literature on velocity elimination against explicit velocities, read through ar5iv on 2026-11-14. Campos, Montiel and Tardós (arXiv:2003.05766) estimate the scale, gravity direction, both biases and up to scale keyframe velocities with the trajectory fixed, weight every inertial residual by its preintegration covariance, put a prior on the accelerometer bias only, and call the eliminated residuals of Mur-Artal and Tardós "meaningless and unrelated to sensor noise properties". They attribute their gain to noise weighting and joint estimation, and estimate the velocities because doing so "eases the initialization process". Zuñiga-Noël, Moreno and Gonzalez-Jimenez (arXiv:2103.03389) eliminate the velocities over three keyframes with covariance weighted residuals after estimating the gyroscope bias, and reach a lower scale error than Campos et al. with ground truth poses (1.11 % against 2.05 % over 18.75 s) but worse gravity and bias over short windows (7.6° against 1.11° gravity error at 1.25 s). Elimination is therefore not worse for the scale when it is weighted, while explicit velocities are better for gravity and biases over short windows and are what a readvance needs. DM-VIO's `CoarseIMUInitOptimizer.cpp` (GitHub master, read 2026-11-14) adds one GTSAM `ImuFactor` between consecutive keyframes, holds the poses as fixed values, uses one `imuBias::ConstantBias` of both sensors with no prior, and solves with `LevenbergMarquardtOptimizer`. He et al., "A Rotation-Translation-Decoupled Solution for Robust and Efficient Visual-Inertial Initialization" (CVPR 2023), and its follow up by Mu et al. (arXiv:2410.19473) estimate the gyroscope bias from the rotations first and then the velocities and gravity from linear translation constraints with explicit velocities, which was found by search and read from the abstracts only.
14. A header only part such as `DelayedMargJob::Coarse` can be unit tested without the library by the harness recipe of `/ws/skills/cpp-syntax-check-tooling.md`, Additions of 2026-11-14 (added 2026-11-14).
15. Change 17 of `plans/delayed_marginalisation.md` (2026-11-14) made the coarse optimisation DM-VIO's coarse IMU initialiser as closely as Basalt allows, at the user's request for a like for like comparison. DM-VIO's defaults, read from `IMUInitSettings.h` and `IMUUtils.h` on GitHub master on 2026-11-14, are `onlyKFs` true, `maxNumPoses` 100 with older keyframes dropped rather than marginalised, `fixPoses` true, one `imuBias::ConstantBias`, no bias prior, a gravity direction prior of 0.4 rad in roll and pitch and $10^{-4}$ in yaw, no scale prior, `coarseScaleUncertaintyThresh` 1.0 on the marginal variance of key `s0` at the solution, GTSAM's `LevenbergMarquardtOptimizer` with `lambdaLowerBound` $10^{-16}$, a run at every keyframe once the optimiser holds more than five frames, and on success a PGBA with `delay` 100, `scaleUncertaintyThresh` 1.0 and `reinitScaleUncertaintyThresh` 0.5, followed by marginalisation replacement at a scale threshold of 1.02. The scale is applied to the DSO poses through a Sim(3) in `PoseTransformationIMU.cpp`, and the gravity is a `Rot3` key with its yaw Jacobian column zeroed. The Basalt version keeps three departures, a 2 cm noise on each held keyframe position, a linear scale increment, and the gravity prior of 0.75°. Like DM-VIO, which integrates the inertial data of non keyframes into the factor ending at the next keyframe, the store composes each per frame factor into `DelayedMargKfChain` as it arrives, keeping the last 99 keyframe factors and an open one from the latest keyframe, and a job extends a copy of it by the window's factors, so the coarse optimisation only ever sees factors that span whole keyframe intervals.
16. With the scale on the held positions the coarse estimate is biased low on jittered poses, the errors in variables effect of item 2. Over 30 synthetic trajectories of 200 frames the mean log scale error was −0.014, −0.052 and −0.279 at 1, 2 and 5 cm of white jitter with keyframes 10 frames apart, and −0.149 and −0.475 at 2 cm with keyframes 5 and 2 frames apart, while the scale variance stayed below $1.5 \times 10^{-3}$ so the gate passed. The form of Change 16 had a root mean square error of 0.009 and 0.021 at 2 and 5 cm. Basalt's keyframe spacing at lift off therefore decides how much of this bias the replays will show (added 2026-11-14).
17. Every job of the store is now installed at the marginalisation after the one that started it, with the VIO thread waiting if needed and the wait logged in `vio_delayed_marg.csv` column `wait`, and the coarse optimisation stops after a quarter of the frame period and then installs nothing. A coarse job took 0.6 ms to 3.6 ms while moving and up to 23 ms at rest, where the flat scale direction uses most of the 100 iterations (added 2026-11-14).

## 16. A visual only coarse phase in Basalt, findings of 2026-11-15

These findings come from the investigation behind `../plans/dm_vo_vio_toggle.md`, which plans DM-VIO's take off order, visual odometry until the coarse scale settles, for Basalt's period. Nothing of it was implemented on 2026-11-15. Note, 2026-11-15. That plan and the visual only phase it implemented were removed from the working tree later the same day, its backup kept outside the repository, and `../plans/stationary_warm_start.md` §1.3 records why, so references to it below are historical.

1. Zeroing the velocity and bias rows and columns of the window's dense $\mathbf{H}$ does not make the window visual only. The preintegrated position and rotation rows depend on both poses, so their contribution to the pose blocks survives, and with the velocity held the position row pulls $\mathbf{p}_1 - \mathbf{p}_0$ towards $\mathbf{v}_0 \Delta t + \frac{1}{2}\mathbf{g}_D \Delta t^2 + \mathbf{R}_0 \hat{\Delta\mathbf{p}}$ with a standard deviation of about 0.5 mm per frame. A velocity wrong by 0.5 m/s then moves the target by 54 standard deviations. The correct construction removes the inertial residual rows from $\mathbf{J}$, which an `ImuLinData` with an empty `imu_meas` map does, since the inertial and bias random walk rows exist only in the `ImuBlock` and `ImuBlockGS` created per map entry (`src/linearization/linearization_abs_qr.cpp:163-166`, `:291-296`), and then holds the velocity and bias columns as the block is held.
2. The main prior must be visual only too, because a prior built with inertial factors at held velocities and zero biases acts on the window through the pose columns in the same way. The store needs no change, since its visual rows are already computed with a null `ImuLinData` (`src/vi_estimator/sqrt_keypoint_vio.cpp:2539-2551`) and every step keeps a copy of its inertial factors (`:1557-1564`). In the coarse phase a readvance from $B$ recomputes the whole main prior (§15 and `../plans/delayed_marginalisation.md` §2.3.14), so one coarse job's readvance turns a visual only main prior into the full visual inertial prior.
3. `MargHelper::marginalizeHelperSqrtToSqrt` is rank revealing and skips a zero column (`src/vi_estimator/marg_helper.cpp:300-318`), so marginalising velocity and bias columns without residuals is safe in the estimator. `NfrMapper::processMargData` uses `marginalizeHelperSqToSq`, whose `fullPivLu` inverse of a zero block is not finite, so a visual only marginalisation packet must not reach the local mapper.
4. A coarse installation sets the block to the estimated scale before re-centring (`InstallCoarseJob`, `sqrt_keypoint_vio.cpp:2690-2729`). The re-centring alone leaves every visual residual invariant, but setting the block changes the residual, so every window landmark's inverse distance is then wrong by the installed factor $s$. The local mapper divides its own landmarks' inverse distances by the correction's scale (`LocalMapper::ApplyGravityScaleCorrection`, `src/vi_estimator/local_mapper.cpp:94-103`), and the estimator does not. The defect stood in the visual inertial coarse phase on 2026-11-15, where corrections up to ±0.29 in log scale were installed.
5. The SITL camera is monocular with a lever arm of 0.5 m against a slant range of about 171 m, so in a visual only window the scale is nearly a gauge direction, held only by the Marquardt damping, and roll and pitch are held only by the gravity prior rows of `vio_gravity_prior_std_deg` with the block held.
6. The optical flow does not read the estimator's prediction, but the triangulation and the first linearisation of a new frame do. With the velocities held, the newest velocity would integrate the accelerometer without correction, 1.7 m/s of drift over 10 s for a tilt error of 1°, so a visual only window needs a constant velocity seed from the finite difference of the two newest positions.
7. A settle test on the change of the block between consecutive readvances cannot pass with thresholds below the replacement thresholds, because a readvance fires only once the block has moved past them from the prior's anchor. Stillness is measured instead by `GravityScaleCorrectionLog::Between` over the epochs of the last $K$ marginalisations. In the orbit of 2026-11-13 run 2 the scale moved by only 0.001 to 0.005 per 4 s while 10 % to 15 % low, but the tilt moved by 0.15° to 0.35° per 4 s, so a tilt threshold of 0.1° over 10 s separates the orbit from the settled square.
8. Applying a coarse correction to a visual only window, found 2026-11-15 for the revised `../plans/dm_vo_vio_toggle.md` §3. The visual factors depend on the relative camera pose only, which a global rotation leaves unchanged, so the gravity direction never reaches them, and the scale reaches them as $s\rho$ in $\pi(\mathbf{R}_{th}\mathbf{u} + \rho\,\mathbf{t}_{th})$ plus the lever arm term $(\mathbf{R}_t^\top \mathbf{R}_h - \mathbf{I})\,\mathbf{t}_{ic}$. Dividing every inverse distance by $s$ leaves only $(s - 1)\,\mathbf{R}_{ic}^\top(\mathbf{R}_t^\top \mathbf{R}_h - \mathbf{I})\,\mathbf{t}_{ic}$, at most 4.6 cm for $s - 1 = 0.3$ over a 0.3 rad window rotation with the 0.51 m SITL lever arm, 0.09 px at 171 m and 0.7 px at 20 m. Without the rescale a landmark at 20 m over a 1 m baseline moves by 4.8 px for $s - 1 = 0.3$.
9. In a visual only window the gravity prior rows $\frac{1}{\sigma_g}(\delta\boldsymbol{\varphi}_{0,xy} - \boldsymbol{\delta}_\theta)$ are the only roll and pitch information, and after a tilt correction $\boldsymbol{\delta}_c$ their residual becomes $-\frac{1}{\sigma_g}\boldsymbol{\delta}_c$. The window then rotates the world back at no visual cost, so a gravity correction applied without changing the prior is reverted. Resetting the main prior's block anchor to the corrected block (`mpAnchorCurr.set = false` then `AnchorGs`) moves the rows' mean to the correction, and it does not double count, because the prior installed at the switch is readvanced from $B$, whose gravity rows keep the initial direction.
10. Repeated application of a coarse estimator that regresses on the visual side can ratchet, since each result is regressed on a world the previous one already shrank while the keyframe jitter is renewed at every solve. The three node form of 2026-11-13 shrank the scale 1, 0.75, 0.50, 0.21, 0.12 in 6 s (`../plans/delayed_marginalisation.md` §3.5 item 11), and Change 17 of that plan also applies the scale to the held positions. A cumulative settle test over mapped estimates cannot pass during a ratchet, because the sums of similar sized corrections leave the band.
11. The stationary marginalisation counter added to the working tree on 2026-11-15 (`mpStationaryMargCount`, `src/vi_estimator/sqrt_keypoint_vio.cpp:1111-1116`) counts calls of `marginalize`, including those before `opt_started` and those that marginalise nothing, against `DelayedMarginalisation::NextIndex`, which counts pushed steps. It also counts the first moving frame and stops at the first non stationary frame even while the start latch is open, whereas `mpGsStartNs` waits for the latch to close. Reading `NextIndex()` where `mpGsStartNs` is set gives the steps before the debounced motion start.
12. ORB-SLAM3 (arXiv:2007.11898 §V-B, read 2026-11-15) scales and rotates its frame poses, velocities and map points at its first inertial initialisation, updates the biases and repeats the preintegration, and is visual inertial from then on, with visual inertial bundle adjustments 5 s and 15 s later and a scale and gravity only refinement every 10 s until 100 keyframes or 75 s. Its first initialisation waits for 2 s of keyframes at 4 Hz, whereas Basalt's first coarse result comes after six keyframes at 156 ms.
13. What the prior installed at a visual only phase's switch holds, derived 2026-11-15 (`../plans/dm_vo_vio_toggle.md` §2.11). Steps leave the store into $B$ only once more than `vio_gs_dm_delay` are stored, counted from the first marginalisation, and a motion based lengthening of the coarse phase does not lengthen the store. If the switch comes first, $B$ is still exactly the initial prior (position, yaw, two gravity rows, bias rows, no velocity row) and the switch's readvance rebuilds the whole prior with every inertial factor since the first marginalisation. If the visual only phase outlasts the store, departing steps enter $B$ with visual rows only, so the position, yaw and gravity pins survive through the visual relative poses while the initial bias rows, which link to later states only through the bias random walk rows, are eliminated as a constant, and the ground phase's inertial factors are lost. $B$ then holds no scale information, since its visual rows have a zero scale column, so the switch's scale comes only from the coarse optimisation over the keyframes still stored. Letting the inertial factors of steps marginalised before the debounced motion start into $B$ with Change A's scale free admission keeps both, because at rest the velocities are zero, the whitened scale column is near zero and the bias enters linearly.
14. First flights of the visual only coarse phase, analysed 2026-11-15 (`/ws/logs/slam/2026-11-15_17-32-54`, 49 s, and `/ws/logs/slam/2026-11-15_17-35-28`, partial at 42 s, both with `vio_gs_dm_coarse_vo`, `vio_gs_dm_vo_apply`, `vio_gs_dm_vo_stationary_inertial` and `vio_gs_dm_delay_from_motion` true and $K_\text{vo} = 10$). The visual only window failed in rotation at lift off, before any scale or gravity correction of note. Until the first triangulation the window had no landmark and followed the inertial prediction with a tilt error of about 1.3°. At the first triangulation, with 1 to 6 landmarks, the estimate rotated by 16° in one frame in the first run (ground truth 0.4°) and by 1.4° to 4.7° per frame for half a second in the second (ground truth 0.1° to 0.3°), reaching tilt errors of 15.6° and 18° while the installed gyroscope bias was still below 0.05 rad/s in the second run. A window without inertial rows, with held velocities and a handful of landmarks over short baselines, leaves the relative rotation underdetermined, the rotation against translation ambiguity of distant points, and the solves ran to the iteration limit without converging. The coarse optimisation then held these rotations and explained the inconsistency with the inertial data by biases, a gyroscope bias of 0.25 rad/s after the 16° jump, and an accelerometer bias equal to the gravity misprojection of the attitude error, $|\mathbf{b}_a| = |\mathbf{g}|\,|\mathbf{u} - \mathbf{v}|$ with $\mathbf{u}$ and $\mathbf{v}$ the body frame up directions of estimate and ground truth, with a correlation of 0.89 and 0.85 and a median ratio of 1.00 in the two runs. With `vio_gs_dm_vo_apply` these biases were installed, and in the first run the gyroscope bias entered the prediction and drove a further drift of about 1° per frame, to a tilt error of 85° and an accelerometer bias of 13 m/s². In the second run the scale never passed the variance gate (marginal variance 0.2 to $6.7 \times 10^4$, the visual scale 0.13 to 0.45 of metric), so every result was a scale held fit, whose tilt and bias absorb the scale error, and the applied gravity corrections of up to 17.7° cumulative brought the tilt error back to 1° to 3° at 18 s to 26 s, with the accelerometer bias falling from 2.9 to 0.4 m/s² and rising again to 2.9 when the rotation drifted at 38 s. In the visual only phase the window holds the biases, so every bias value is written by a coarse installation. The main prior's cost was negative in these and in the visual inertial baselines alike, as expected of Basalt's prior error, and does not single them out. The visual inertial baselines `2026-11-15_12-51-19` and `2026-11-15_11-30-32` stayed within 1.7° and 8.1° of tilt, 0.26 and 0.19 m/s² of accelerometer bias and 0.014 rad/s of gyroscope bias. The first run switched at 26.5 s on consistent but wrong coarse results with an 85° tilt error, which shows that the settle test cannot detect a consistent error.
15. Scale jumps in the flights of 2026-11-15 afternoon, analysed the same day. `2026-11-15_17-54-10` (143 s) ran the gyro aided VO window of `../plans/dm_vo_vio_toggle.md` (inertial cost on 15,613 of 15,688 VO iterations, `libbasalt.so` built one minute before), `2026-11-15_17-39-58` (226 s) the purely visual window. The gyro aid held the tilt error to 0.1° to 4.4° for the whole flight. Three findings. First, the VO world's arbitrary scale was set at the first triangulation. At 3.50 s, with the vehicle still on the ground, 5 landmarks over a 5.8 cm baseline produced a non converged solve that moved the newest position by 0.32 m against 1 cm of ground truth, the finite difference velocity seed turned that into 3.76 m/s, the inflated predictions gave the next triangulations baselines of about 1 m against 0.1 m of true motion, and the world was 4.2× to 5.3× metric from 4.0 s to 8.5 s. The first scale free coarse result at 8.78 s applied −1.76 in log scale against −1.65 implied by ground truth, a correct correction, after which the 2 s scale ratio stayed at 1.07 to 1.20 until 45 s. Second, the mid flight jumps of the gyro aided run all came from coarse results of the VO phase passing the gate at a variance of 2.2 to $2.5 \times 10^{-3}$ against the gate's $2.5 \times 10^{-3}$, during the high acceleration legs (ground truth acceleration fluctuation 8 to 23 m/s² rms over the store), with costs two to ten times that of the good result, and against ground truth in direction, −1.31 at 45.65 s where ground truth implied −0.05, −1.40 at 54.05 s where it implied +0.95, and +1.74 at 56.07 s where it implied −1.25. The VO prior holds no scale information, so each installed result set the world's scale directly, and the coarse optimisation reads neither $B$ nor its own earlier results. No readvance ran before the switch at 120.26 s, and after it the 2 s scale ratio held at 0.93 to 0.99 with 4 readvances. With $K_\text{vo} = 1$ the switch would have come at 8.78 s on the correct result, and with `vio_gs_dm_delay_from_motion` the delayed phase would have started about 14 s after the first marginalisation, before any of the wrong results. Third, the purely visual run switched at 68 s with a 9.5° tilt error and an accelerometer bias of 1.4 m/s² inherited from its failed VO phase and a prior whose inertial rows came only from the last 200 steps, and its delayed phase readvanced on almost every frame from 80 s to 88 s, with smoothing moving departed states by up to 4.8 m, while the window's speed fell to 30 % of ground truth.
16. The stationary warm start that replaced the visual only coarse phase, 2026-11-15 (`../plans/stationary_warm_start.md`). The visual only phase and its plan `plans/dm_vo_vio_toggle.md` were removed from the working tree after items 14 and 15, and the estimator stays visual inertial from the first frame. Before this change the zero velocity residual acted on the newest state only and was never marginalised, so with a window of three states the prior kept about 0.1 s of the stationary period, and nothing held the poses on the ground, where the window has no landmark. At rest the gyroscope bias is observable in all three axes and the accelerometer bias along gravity, while the horizontal accelerometer bias is confounded with the tilt and is split only by the gravity prior until rotation separates a body fixed bias from world fixed gravity. The change flags every state created at rest, adds zero velocity rows on each and zero motion rows, identity relative rotation and zero translation, between consecutive flagged states, and marginalises each factor into the prior and the delayed store's step when its first state leaves the window, so the whole ground phase reaches the prior. The coarse optimisation's bias starts at the window's estimate under a prior, the delay counts from the end of the stationary period with the store keeping the last stationary step, and the settled fold, the landmark rescale of a coarse installation and the SITL retuning were kept from the removed work.
17. The first flight of the stationary warm start froze the estimate on the ground, analysed 2026-11-15 (`/ws/logs/slam/2026-11-15_18-34-59`, 59 s, details in `../plans/stationary_warm_start.md` §1.5). The flag cleared at 3.63 s, was set again at 3.74 s by a quiet frame (flow 0.37 px, the SHOE statistic still below its threshold because it averages 0.5 s) while the vehicle accelerated at 0.5 m/s², and cleared for good at 3.79 s. Because the stationary rows are marginalised, that one zero velocity row held a velocity error of about 0.07 m/s in the prior permanently, the window then lagged the climb by about 40 %, the first triangulation at 4.26 s gave 37 landmarks with depths about 40 % short, and the coarse installations, which fit keyframe velocities to held keyframe positions, then drove the window's velocity to zero every three frames. With the estimate held, no track reached the 5 cm triangulation baseline, no landmark was added after 4.26 s, and the run never recovered, while the cumulative scale decayed to zero from 15 s. The fix clears every flag in `measure` on a non stationary frame, so a flag reaches the prior only if the detector reported rest until its state left the window. Two general lessons. A residual that is marginalised makes a detector's transient error permanent, so its gating needs a confirmation delay at least as long as the window's residence. And during the coarse phase the coarse installations amplify a window that lags the true motion, since they trust the held visual poses, so a slow lift off with a bad first triangulation can lock the estimate in place.
   Correction, 2026-11-15. The wrong flag of item 17 was a real defect but not the cause of the freeze, since the next flight froze without it, item 18.
18. The second flight of the warm start froze the same way without any wrong flag, analysed 2026-11-15 (`/ws/logs/slam/2026-11-15_20-08-50`, 22 s, details in `../plans/stationary_warm_start.md` §1.6). The cause was the marginalised zero velocity rows. Ground truth shows the vehicle settling or sliding by 1 to 5 cm while the detector reports rest (5 cm down in `20-08-50`, 3.6 cm in x in `18-39-15`), the slow motion of a vehicle spinning up on its landing gear, which no inertial sample resolves. About 70 rows of $\sigma_v = 0.01$ m/s, marginalised as independent, hold the period's mean velocity to about 1.2 mm/s and so every ground position to a few millimetres, and all three new runs show the ground positions at exactly 0.000 m, whereas `2026-11-15_18-12-31` let them drift. With $f = 320$ px and ground features at 0.34 to 0.58 m depth, a centimetre is 6 to 9 px, and at the first triangulation (4.07 s, 74 landmarks) the solve started at a cost of about 3800 and ended with a visual cost of 524, against 29 in `18-12-31`, while the estimate stopped rising. Marginalised rows must model correlated errors as such, and the zero motion rows, a random walk of 5 mm per step, allow about 4 cm over the period. The fix marginalises only the zero motion rows and keeps the zero velocity rows in the window. Signatures to check in the next flight: ground positions drifting by about the true settling, a first triangulation visual cost of the order of 30, and the estimated height following ground truth through lift off.
19. The third flight still froze after the zero velocity rows were taken out of the prior, analysed 2026-11-15 (`/ws/logs/slam/2026-11-15_20-22-22`, 40 s, `libbasalt.so` built minutes before, details in `../plans/stationary_warm_start.md` §1.7). The ground positions now drifted by a few millimetres, but the estimate tracked the lift off only until the first triangulation at 3.91 s, 8 landmarks at a visual cost of 118 after the solve, and from then on the coarse installations every three frames, whose costs before the solve rose from 3.8 at rest to 30 to 48, lowered the vertical velocity from about 0.3 to 0.08 m/s, while the main prior's cost fell to −1000 and only 7 landmarks remained. The estimate reached 1.3 m when ground truth was at 16 m. At the user's direction the prior is now reduced, at every moving frame that follows a stationary one, to its exact marginal on the gravity and scale block and the last stationary state, the ground keyframes and their landmarks are dropped, and the delayed store restarts from that prior (`ResetPriorToLastState`, `DelayedMarginalisation::Reset`), so that only the last two frames carry the stationary period into flight, with the warm started velocity and biases.
