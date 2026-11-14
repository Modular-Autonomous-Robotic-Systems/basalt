# Stationary Body Detection for Basalt VIO

## 1. Introduction

Basalt's estimator gates a zero velocity soft constraint on a per frame stationarity flag, `mpIsStationary` at `include/basalt/vi_estimator/sqrt_keypoint_vio.h#L285`, computed every frame from the raw IMU samples integrated since the previous frame at `src/vi_estimator/sqrt_keypoint_vio.cpp#L373-L438`. This document measures that flag directly against the eight logged experiments under `/ws/logs/slam`, by joining `vio_imu_frame.csv` (the flag and its two inputs) against `vio_state.csv` (the optimiser's own fused velocity, `v_norm`) on `t_ns`, and finds the flag unreliable on every single one of them, not only on the run first inspected.

| Experiment | Frames | Flagged stationary | Flagged, but `v_norm` > 0.1 m/s | Fraction of flagged frames that are false |
|---|---|---|---|---|
| `phase_1_1` | 3162 | 1787 (56.5%) | 1766 | 98.8% |
| `phase_1_2` | 3174 | 1920 (60.5%) | 1865 | 97.1% |
| `phase_2_1` | 3188 | 1827 (57.3%) | 1800 | 98.5% |
| `phase_2_2` | 3033 | 1561 (51.5%) | 1527 | 97.8% |
| `phase_3a_1` | 3197 | 1446 (45.2%) | 1398 | 96.7% |
| `phase_3a_2` | 3183 | 1605 (50.4%) | 1550 | 96.6% |
| `phase_3b_1` | 3183 | 1762 (55.4%) | 1673 | 94.9% |
| `phase_3b_2` | 3005 | 1662 (55.3%) | 1553 | 93.4% |

Every experiment flags roughly half of all frames stationary, and of the frames each one flags, 93 to 99% of them carry a VIO velocity estimate above 0.1 m/s, which on this platform's mission profiles is unambiguously moving. This is not a tuning accident isolated to one recording, it reproduces across every phase of the campaign, which is the first piece of evidence that the defect is structural rather than a matter of the two configured tolerances being slightly off for one particular flight. Section 3.4 below sweeps those two tolerances exhaustively against `phase_3b_1` and shows precisely how much of the failure a better choice of tolerance can and cannot remove.

The reason is visible directly in what the test measures. Specific force and angular rate are the physical quantities an accelerometer and a gyroscope read, and both are proportional to acceleration and rotation, not to velocity. A vehicle translating at a constant, unaccelerating velocity along a straight, level path presents specific force equal to gravity and angular rate equal to zero, to within sensor noise, in exactly the same way a vehicle truly at rest does. No function of specific force and angular rate over any window length can distinguish the two cases, because the two cases are, by construction, indistinguishable in exactly the inputs such a function is allowed to see. Resolving the ambiguity requires either a second physical quantity that is zero at rest and non zero in constant velocity cruise, most directly the vehicle's own velocity estimate or a vision based parallax cue, or, short of that, enough statistical care in the IMU only test to shrink the problem to the residual cases where the vehicle is both unaccelerated and non rotating for an extended interval, which section 3.4's run length analysis shows is a real but much smaller fraction of the measured failure than the raw numbers above suggest. This document surveys the published literature on both kinds of fix, restates Basalt's own implementation in the same mathematical language used throughout that literature, and proposes a layered, quantitatively justified replacement grounded in data already logged by this codebase, so that the eventual implementation choice is made from evidence rather than from a fresh literature pass or from intuition about what "should" work.

## 2. Literature Survey

### 2.1 Statistical foundations, the generalized likelihood ratio test

Every classical inertial zero velocity detector in this survey, including Basalt's own, is an instance of a single statistical construction, the generalized likelihood ratio test (GLRT) for a binary hypothesis problem with unknown nuisance parameters, developed generically in detection theory texts such as Kay [10] and applied specifically to zero velocity detection by Skog, Handel, Nilsson and Rantakokko [1] and restated with a unified derivation by Wahlström and Skog [2]. The problem is posed over a window of `W` consecutive samples, indexed `n = k, ..., k+W-1`, of accelerometer readings `a_n` and gyroscope readings `ω_n`. `z_k` denotes everything the test is allowed to look at when deciding at time `k`, the entire stacked window rather than any single instant, `z_k = (a_k, ω_k, a_{k+1}, ω_{k+1}, ..., a_{k+W-1}, ω_{k+W-1})`, and every formula below that writes `p(z_k | ...)` means the joint density of this whole collection of `2W` vector samples, not of one sample alone, which is itself the reason a window of several samples carries more discriminating power than any one of them read in isolation. The choice is between two hypotheses, `H0`, the body is moving, against which no model is assumed beyond the sensor noise itself, and `H1`, the body is stationary, under which the gyroscope should read only noise and the accelerometer should read the local gravity vector plus noise,

```
H1:  a_n = g · ĝ + e_a[n],   e_a[n] ~ N(0, σ_a² I)
     ω_n = e_ω[n],           e_ω[n] ~ N(0, σ_ω² I)
H0:  a_n, ω_n unconstrained (free mean under the noise model)
```

A hypothesis is simple when it fully determines the probability distribution of the data by itself, leaving nothing for the data to still inform. A hypothesis is composite when it only narrows the data down to a family of distributions, one member of that family for each possible value of some parameter the hypothesis leaves open. `H1` above is composite in this sense. Knowing the body is stationary tells us the accelerometer should read a vector of fixed length `g` plus noise, but it does not tell us which direction that vector points, since the same IMU can be mounted at any orientation relative to true vertical, and only the window's own data can reveal that direction.

A nuisance parameter is a parameter the model needs some value for before a likelihood can even be computed, but whose own particular value nobody actually cares about once the test is decided. Here, `ĝ`, the unit gravity direction, is exactly this kind of parameter. The test has to assume some direction for it to evaluate `H1`'s likelihood at all, but the test is not trying to find out which way gravity points, it is trying to decide between `H0` and `H1`, which is the one choice this whole construction exists to report. Whatever value `ĝ` is estimated to take along the way is simply discarded once that decision is made, in contrast with the decision itself, which is the actual parameter of interest.

`H0` is composite as well, and more severely so than `H1`. It places no constraint on the data's mean at all, so every sample is free to equal whatever it equals. The next paragraph uses this directly.

The ordinary likelihood ratio test of Neyman-Pearson theory, the classical starting point every text in this area including Kay [10] builds from, only compares two simple hypotheses directly, because when both `p(z_k|H0)` and `p(z_k|H1)` are single, fully specified numbers with no free parameter inside either one, their ratio is itself a single number and the comparison against a threshold `γ` is immediate. The moment either hypothesis is composite, as `H1` is here on account of `ĝ` and as `H0` is here even more severely, the likelihood is no longer a single number for that hypothesis, it is a whole surface of possible values, one for every value the hypothesis's free parameter could take, and a direct ratio is no longer well defined. The generalized likelihood ratio test is the standard resolution, due to the same detection theory [10], and it works by first finding, separately for each hypothesis, the particular parameter value that makes that hypothesis's own model fit the observed window `z_k` as well as that hypothesis's structure possibly allows, the maximum likelihood estimate under that hypothesis, and only then forming the ratio of these two best possible fits rather than attempting to compare the hypotheses at some single, shared but unknown parameter value. Concretely for `H1`, best possible fit means substituting `ĝ` with whichever unit direction the window's own accelerometer samples best support, and for `H0`, which constrains nothing about the mean at all, best possible fit is achieved trivially by letting each sample's assumed mean equal that very sample, a fit with zero residual beyond the noise model itself, the largest value the Gaussian density can possibly take for that data, a perfect fit in the specific sense that no further choice of any parameter could make the unconstrained `H0` model explain the observed window any better than exactly reproducing it. The likelihood ratio in the generalized test is the ratio of these two best cases,

```
L(z_k) = [ max_ĝ p(z_k | H1, ĝ) ] / [ max p(z_k | H0) ],   decide H1 iff L(z_k) > γ
```

which is the precise sense in which the test compares `H1`'s best constrained fit against `H0`'s best, unconstrained, parameter free fit, and is the generalized form of the ordinary likelihood ratio test built specifically to handle a composite, nuisance parameter laden hypothesis on either or both sides of the comparison. Substituting the Gaussian densities above and carrying out the maximization over the one free nuisance parameter `ĝ` under `H1`, which turns out to have the closed form maximum likelihood estimate `ĝ_ML = ā / ||ā||` where `ā = (1/W) Σ_n a_n` is the window sample mean of the accelerometer, reduces the log likelihood ratio, after discarding additive constants that do not depend on the data, to exactly the SHOE statistic given in section 2.2. This derivation is the origin of two facts that matter directly for section 3 and section 4 below.

First, the correct gravity reference for the accelerometer term is the window's own estimated mean direction, `ā / ||ā||`, not an externally fixed constant such as the literal value 9.81. The maximum likelihood derivation above is what produces that particular reference, and a fixed constant discards information the window already contains about small residual tilt, bias, or scale.

Second, the test statistic connects to a chi squared distribution, and seeing exactly how requires three preliminaries - what a chi squared distribution actually is, how dividing by `σ_a` or `σ_ω` turns the SHOE formula's own Gaussian terms into the specific kind of term that distribution is built from, and how many degrees of freedom survive once `ĝ` is fitted from the same data the test statistic is computed on rather than assumed known.

By definition, if `x_1, ..., x_k` are independent standard normal random variables, each `x_i ~ N(0,1)`, then the sum of their squares, `Σ_i x_i²`, follows a chi squared distribution with `k` degrees of freedom, written `χ²_k`. This distribution shows up naturally whenever Gaussian noise terms are squared and added, which is exactly what every detector in section 2.2 does, and it closes under addition in the obvious way, if two such sums are independent of each other and have `k1` and `k2` terms respectively, their total is again a sum of squared standard normals, now with `k1+k2` terms, so it is itself `χ²_{k1+k2}`.

The SHOE formula's own residuals, `e_a[n] ~ N(0, σ_a² I)` and `e_ω[n] ~ N(0, σ_ω² I)`, are Gaussian but not yet unit variance, their variance is `σ_a²` and `σ_ω²`, not 1. Dividing a zero mean Gaussian by its own standard deviation is exactly the operation that produces a standard normal, since if `e ~ N(0, σ²)` then `e/σ ~ N(0,1)`, which is why the SHOE statistic divides the accelerometer residual by `σ_a²` and the gyroscope residual by `σ_ω²` before squaring, each division is what turns that term's three components into three independent standard normal variables first. Momentarily treating `ĝ` as if it were its true, known value rather than an estimate, one sample's accelerometer term becomes `(1/σ_a²)||a_n − g ĝ||² = Σ_{j=1}^3 (e_a[n]_j / σ_a)²`, a sum of three squared standard normals, hence `χ²_3`, and that same sample's gyroscope term, `(1/σ_ω²)||ω_n||² = Σ_{j=1}^3 (ω_n_j / σ_ω)²`, is `χ²_3` by the same argument, so one full bracketed term of the SHOE sum is `χ²_6`, and summing `W` independent samples gives `χ²_{6W}` for the whole window by the additivity property above.

That `6W` count has to be corrected downward, because `ĝ` is not actually known, it is replaced by its own maximum likelihood estimate fitted from the very data it is then tested against, and fitting a parameter from the data it is tested against always removes as many degrees of freedom from the resulting sum of squares as the number of free parameters fitted, the same bookkeeping behind why a residual sum of squares in ordinary linear regression has `n − p` degrees of freedom rather than `n`, for `p` fitted coefficients. `ĝ` is a unit vector, a point on a two dimensional sphere rather than a free point in three dimensional space, so fitting it from the data consumes 2 degrees of freedom, not 3, leaving the accelerometer term's own contribution at `χ²_{3W−2}` rather than `χ²_{3W}`, exactly so in the flat limit and essentially exactly so here because the gravity magnitude `g` is large compared to the accelerometer noise `σ_a`, which keeps the sphere's curvature negligible over the small region the noise actually explores, the same flat approximation ordinary least squares theory itself relies on. The gyroscope term fits no parameter at all and keeps its full `χ²_{3W}`. Because the accelerometer and gyroscope are independent sensors, the two terms are independent of each other, and the additivity property above gives their sum, the full window total `W · T_SHOE` as defined in section 2.2, a `χ²_{6W−2}` distribution under `H1`.

This result controls one specific thing precisely, the probability that a window which genuinely is stationary gets correctly accepted by the test at a given threshold `γ`, read directly off the cumulative distribution function of `χ²_{6W−2}` as `P(W · T_SHOE ≤ γ)`. It does not, by itself, control the probability of the opposite error, declaring a genuinely moving window stationary, because that probability depends on the distribution of `W · T_SHOE` under `H0`, and `H0` was deliberately left with no parametric noise model for the chi squared argument above to apply to, motion can look like anything. This is exactly why section 3.4 below cannot simply compute a threshold from this formula and stop, it still sweeps `γ` empirically against genuinely moving data from the logged experiments, and the chi squared result here should be read as a principled way to pick a starting point on the stationary side of that sweep, not as a substitute for running it.

### 2.2 The four classical inertial detectors as instances of the GLRT

Skog et al.'s evaluation [1], restated in [2], derives four detectors as special cases of the construction in section 2.1 by varying which of the two sensor models is kept and which is dropped or simplified.

The acceleration magnitude detector, MAG, keeps only the assumption that specific force magnitude should equal the known gravity magnitude, discarding the gyroscope entirely and discarding the direction information in the accelerometer, with test statistic

```
T_MAG(z) = (1/W) Σ_n | ||a_n|| − g |,   decide H1 iff T_MAG ≤ γ_MAG
```

The moving variance detector, MV, keeps only the assumption that the accelerometer reading should be constant across the window, without assuming it equals any particular known magnitude or direction, which makes it robust to an uncalibrated accelerometer scale factor or bias at the cost of also discarding the gyroscope,

```
T_MV(z) = (1/W) Σ_n ||a_n − ā||²,   ā = (1/W) Σ_n a_n,   decide H1 iff T_MV ≤ γ_MV
```

The angular rate energy detector, ARE, keeps only the gyroscope assumption, that angular rate should be zero mean, discarding the accelerometer entirely,

```
T_ARE(z) = (1/W) Σ_n ||ω_n||²,   decide H1 iff T_ARE ≤ γ_ARE
```

The stance hypothesis optimal estimation detector, SHOE, is the full GLRT of section 2.1, keeping both sensor models and estimating the gravity direction from the window itself,

```
T_SHOE(z) = (1/W) Σ_{n=k}^{k+W-1} [ (1/σ_a²) ||a_n − g·(ā/||ā||)||² + (1/σ_ω²) ||ω_n||² ],   decide H1 iff T_SHOE ≤ γ_SHOE
```

Skog and Wahlström's empirical comparison across these four finds that "the most valuable information for zero-velocity detection is contained in the gyroscope measurements" [2], so ARE and SHOE, which both use the gyroscope, consistently outperform MAG and MV, which do not, a finding independently reproduced in section 3.4 of this document, where tightening the gyroscope tolerance alone, holding the accelerometer tolerance fixed, is shown to account for nearly all of the achievable improvement in Basalt's own data. The review's single largest reported effect concerns the threshold `γ` itself rather than the choice of detector, "an adaptive navigation system, which alternates between the optimal thresholds for walking and running, reduces the position RMSE by more than 50% in comparison with the best fixed threshold" [2], establishing that even the strongest fixed threshold detector is badly outperformed by the same detector with a context dependent threshold, a result this document's section 4.1 takes as direct motivation for proposing a layered rather than a single fixed threshold replacement. Jiménez, Seco, Prieto and Guevara's independent 2009 comparison [3] evaluates a similar accelerometer magnitude and variance test inside a complete strapdown INS-EKF-ZUPT pipeline, described further in section 2.3, and reports stride length errors near 1% and position errors generally under 5% of distance travelled on a walking dataset, corroborating that these simple tests are adequate specifically because human gait structurally alternates between an unambiguous full stance, where both feet are briefly motionless, and an unambiguous swing phase, a two regime structure with no analogue in this platform's sustained, continuously varying flight profile.

For our setting, every member of this family is attractive on cost grounds and unattractive on power grounds for exactly the reason given in section 1, each is `O(W)` per window with no state beyond a few running sums, each can be computed from exactly the accelerometer and gyroscope samples Basalt already integrates every frame, and none of the four, individually or in any linear combination, can see velocity, so none can resolve the constant velocity cruise case by itself. SHOE and ARE remain the correct reference points for any pure IMU replacement of Basalt's own test, both because the literature consistently ranks them above MAG and MV and because section 3.4's own sweep of Basalt's two existing tolerances shows the same ordering, the gyroscope term carries almost all of the discriminative power actually available in this platform's own logged data.

### 2.3 Kalman filter integration of a zero velocity measurement

Foxlin's 2005 NavShoe system [11] is the paper credited by [2] with popularizing the ZUPT technique in its now standard form, injecting the stationarity decision not as a hard reset but as a pseudo measurement inside an extended Kalman filter running over the full strapdown inertial navigation state, position, velocity, attitude, and sensor biases. Seeing precisely why a measurement that only ever touches velocity also corrects position and attitude, which is the detail the rest of this document leans on, requires building the Kalman filter up from its own preliminaries first, then specialising it to the particular state Foxlin's filter carries, rather than quoting the familiar gain formula and asserting the cross correction follows.

A discrete time Kalman filter assumes two linear, Gaussian noise corrupted models, a process model describing how the true state evolves between measurements, `x_k = F x_{k-1} + w_{k-1}`, `w_{k-1} ~ N(0, Q)`, and a measurement model describing what a sensor reads given the true state, `y_k = H x_k + v_k`, `v_k ~ N(0, R)`, with `w` and `v` independent of each other and of the state's own past. The filter's job is to maintain a Gaussian belief over the state at every step, a mean `x̂_k` and a covariance `P_k` summarising everything the filter currently believes about `x_k` and how uncertain that belief is, updating both quantities every time a new process step or a new measurement arrives.

The prediction step propagates this belief across one process step with no new measurement. Because `x_k = F x_{k-1} + w_{k-1}` is a linear map of a Gaussian state plus an independent Gaussian noise term, the mean and covariance of a linear combination of independent Gaussian vectors are themselves given by the same linear combination applied to the means, and by the matrix congruence transform applied to the covariances plus the added noise covariance, which gives the prediction equations directly, `x̂_k^- = F x̂_{k-1}^+` and `P_k^- = F P_{k-1}^+ F^T + Q`, where the superscript `-` marks a quantity computed before the current step's measurement is used and `+` marks one computed after. This single equation for `P_k^-` is the mechanism this section eventually rests everything on, since it shows the predicted covariance depends on the previous covariance only through the sandwich product `F P F^T`, so any off diagonal structure already present in `F` reshapes `P` on every single prediction step, whether or not `Q` itself has any off diagonal structure of its own.

The update step folds in a new measurement `y_k`, and deriving it needs one more preliminary, the standard fact that if two random vectors `x` and `y` are jointly Gaussian, with means `μ_x`, `μ_y` and a joint covariance whose blocks are `Σ_xx`, `Σ_xy`, `Σ_yx = Σ_xy^T`, `Σ_yy`, then the conditional distribution of `x` given an observed value of `y` is itself Gaussian, with mean `μ_x + Σ_xy Σ_yy^{-1} (y − μ_y)` and covariance `Σ_xx − Σ_xy Σ_yy^{-1} Σ_yx`. Applying this fact to the prior belief `x_k ~ N(x̂_k^-, P_k^-)` together with the measurement model `y_k = H x_k + v_k` gives the joint covariance blocks `Σ_xx = P_k^-`, `Σ_xy = P_k^- H^T`, `Σ_yy = H P_k^- H^T + R`, since `Cov(x_k, y_k) = Cov(x_k, Hx_k + v_k) = P_k^- H^T` and `Cov(y_k) = H P_k^- H^T + R` follow from the same linear combination rule used for the prediction step. Substituting these blocks into the conditioning formula above gives exactly the Kalman update, a posterior mean `x̂_k^+ = x̂_k^- + K (y_k − H x̂_k^-)` and a posterior covariance `P_k^+ = (I − K H) P_k^-`, where `K = P_k^- H^T (H P_k^- H^T + R)^{-1}` is the Kalman gain, derived here, not assumed, as nothing more than the cross covariance block `Σ_xy` from the conditioning formula divided through by the variance block `Σ_yy`. Everything the rest of this section does is this one substitution applied to a particular `H` and a particular, structured `P_k^-`.

Foxlin's filter carries an error state built from the strapdown inertial navigation equations, position error `δp`, velocity error `δv`, attitude error `δθ`, and the gyroscope and accelerometer bias errors `δb_g`, `δb_a`, fifteen scalar components in total, the same five block, fifteen dimensional layout Basalt's own `PoseVelBiasState` carries as `POSE_VEL_BIAS_SIZE = 15` in `thirdparty/basalt-headers/include/basalt/imu/imu_types.h#L49`. The linearised continuous time dynamics of the position, velocity and attitude blocks, standard in strapdown inertial navigation and given in full in Groves [12], are

```
d/dt δp =        δv
d/dt δv = [f×]·δθ + δb_a
d/dt δθ = -[ω×]·δθ − δb_g
```

where `f` is the specific force the accelerometer measures, `ω` is the angular rate the gyroscope measures, and `[f×]` denotes the skew symmetric matrix built from `f` so that `[f×] x = f × x` for any vector `x`, the matrix form of the cross product. Discretised to first order over one sampling interval `Δt`, small enough here that higher order terms in `Δt` are negligible, this continuous time generator becomes a transition matrix `F` whose off diagonal blocks are `Δt · I` coupling `δv` into `δp`'s row and `Δt · [f×]` coupling `δθ` into `δv`'s row, with every block below the diagonal equal to zero. `F` is upper block triangular, not block diagonal, and this one structural fact, not anything about the noise `Q`, is what builds the cross correlation terms this section is about.

To see the mechanism explicitly rather than take it on faith, consider just the position and velocity blocks in isolation, `F = [[I, Δt I], [0, I]]` for the position/velocity pair, and suppose the filter starts from a covariance with no cross correlation at all between position and velocity error, `P_{k-1}^+ = [[P_p, 0], [0, P_v]]`. Carrying out the prediction step's sandwich product block by block,

```
F P F^T = [[I, ΔtI], [0, I]] · [[P_p, 0], [0, P_v]] · [[I, 0], [ΔtI, I]]
        = [[P_p, Δt·P_v], [0, P_v]] · [[I, 0], [ΔtI, I]]
        = [[P_p + Δt²·P_v, Δt·P_v], [Δt·P_v, P_v]]
```

shows the position/velocity cross block, exactly zero one step earlier, becomes `Δt · P_v` after a single prediction step, purely because `F`'s own off diagonal `Δt I` block mixed velocity uncertainty into the position row. The identical mechanism, with `[f×]` in place of `Δt I`, builds a non zero velocity/attitude cross block `P_vθ` out of an initially uncorrelated `P_v` and `P_θ`. Repeated over the many prediction steps between one zero velocity window and the next, this is why `P_k^-` arrives at the next ZUPT update carrying real, non zero `P_pv` and `P_vθ` blocks even if it started from a perfectly block diagonal covariance, with no need for the process noise `Q` to couple the blocks on its own.

The ZUPT update itself sets `H = [0_{3x3}  I_{3x3}  0_{3x3}  0_{3x3}  0_{3x3}]` in the `[δp, δv, δθ, δb_g, δb_a]` ordering, selecting the velocity block alone, with `R` a small or zero measurement noise covariance expressing high confidence that true velocity is zero. Because `H` picks out only the `δv` row, `H P_k^-` is simply the velocity block row of `P_k^-`, and the Kalman gain from the update derivation above, `K = P_k^- H^T (H P_k^- H^T + R)^{-1}`, splits into one block per state component,

```
K_p = P_pv (P_v + R)^{-1}      (position block)
K_v = P_v  (P_v + R)^{-1}      (velocity block, → I as R → 0)
K_θ = P_vθ^T (P_v + R)^{-1}    (attitude block)
K_bg, K_ba similarly, through whatever cross correlation the bias blocks have built up with δv
```

and applying the posterior mean update `x̂_k^+ = x̂_k^- + K (y_k − H x̂_k^-)` with the pseudo measurement `y_k = 0`, so that the innovation `y_k − H x̂_k^- = −δv̂`, gives `δp^+ = δp^- − K_p · δv̂`, `δv^+ = δv^- − K_v · δv̂`, and `δθ^+ = δθ^- − K_θ · δv̂`, position and attitude corrected by an amount set by exactly the cross covariance blocks `P_pv` and `P_vθ` the previous paragraph showed accumulate automatically from `F`'s own off diagonal structure, with velocity itself driven essentially to zero as `R → 0` since `K_v → I` in that limit. This is the complete mechanism, not an assertion, by which a measurement that only ever constrains velocity nonetheless bounds the growth of position and attitude error indefinitely, as long as zero velocity intervals recur often enough to re-zero the velocity block before its own cross correlations with position and attitude grow too large to correct accurately.

This is the architectural slot Basalt's own `mpZeroVelocityWeight` soft constraint occupies, described fully in section 3.2, except implemented as an extra weighted residual inside a batch nonlinear least squares optimisation over a sliding window of keyframes rather than as a single step Kalman update of the kind just derived. This substitution is the expected adaptation for a smoother style, factor graph estimator such as Basalt's `SqrtKeypointVioEstimator` rather than a cause for concern, since a linear Gaussian batch least squares problem and a sequential Kalman filter compute the identical posterior mean when run over the same linear Gaussian model, the batch normal equations are simply the same conditioning argument derived above applied to every state in the window jointly rather than one state at a time, which Forster et al.'s preintegration framework [7] that Basalt's own `IntegratedImuMeasurement` is built from makes the same equivalence between a sequential filter and a windowed smoother explicit for exactly this IMU factor. A zero velocity residual inside that batch optimisation is a Gaussian pseudo measurement pinning velocity toward zero with a given confidence, and the same marginal covariance structure derived above, real, non zero correlation between velocity and the other pose states built up by the IMU factors linking consecutive keyframes, is what lets that one residual's correction propagate into the window's position and attitude estimates as well, through the normal equations rather than through an explicit Kalman gain, but by the same underlying mechanism.

### 2.4 Adaptive and data driven detectors

A second line of work treats the threshold `γ` itself as the weak point exposed in section 2.2's adaptive threshold result, and either learns a context dependent threshold or replaces the entire test with a learned classifier. Wagstaff, Peretroukhin and Kelly's 2020 paper [4] proposes two designs evaluated on a large VICON referenced foot mounted dataset covering more than 7.5 km of walking, running and stair climbing. The first, a motion adaptive detector, trains a support vector machine with an RBF kernel on six axis windows of 200 samples, drawn from roughly 2000 samples per trial across 4 walking, 5 running and 6 stair climbing VICON trials, to classify the current gait into one of the three categories at better than 70% per class accuracy, then uses that label to switch the SHOE threshold `γ_SHOE` to the value independently found optimal for that gait, directly operationalising the adaptive threshold result from [2] rather than inventing a new detector. The second design, from Wagstaff and Kelly's earlier 2018 paper [5], removes the threshold and the hand built statistic entirely, training a six layer, 80 unit per layer LSTM network on 100 sample windows of raw six axis IMU data, drawn from 51 training trials at roughly 7000 samples per trial, against cross entropy loss over 300 epochs with the Adam optimizer and data augmentation by random rotation and scaling, to output a direct binary zero velocity classification. The LSTM detector reports a 32.6% lower RMSE than the best fixed threshold detector on hallway walking, and more tellingly keeps vertical error under one metre over 100 steps of stair climbing where SHOE and an adaptive variant exceed that bound within 25 steps, because the recurrent network has implicitly learned a stair specific motion signature that no instantaneous or short window statistic encodes, and the same paper shows the trained model can be adapted to a different, lower cost IMU with only an 18.4% residual error increase after retraining, and runs at 3.35 kHz on an ordinary CPU, well inside real time budget. Both papers are explicit about the cost of this gain, [4] reports its own SVM motion classifier still misclassifies stair climbing as walking roughly 30% of the time, [5] shows the LSTM detector fails outright on gait speeds outside its training distribution, and both require a labelled reference trajectory, from a VICON rig in both cases, to train against in the first place.

For our setting this family is the least attractive of the three surveyed here, for reasons sharper than a generic "needs training data" objection. The accuracy gains in both papers are earned specifically on human gait, where stance and swing are repeating, learnable phases of a periodic motion, a structure this platform's mission profiles, continuous flight with no analogous periodic stance phase, do not share, so a network trained on foot mounted pedestrian data would not transfer, and there is no labelled stationary or moving reference trajectory for this airframe and this IMU to train a bespoke model against, the nearest substitute being the VIO's own `v_norm`, which is precisely the signal section 4.3 proposes using directly rather than as a training label for a heavier learned model. The implementation cost, a trained model plus an inference call every frame inside an already real time estimator, is an order of magnitude above every other candidate surveyed in this document. The one transferable idea, letting a cheap auxiliary signal adapt the decision rather than fixing a single global threshold, reappears in section 2.5 and section 2.6 in a form that costs nothing beyond reading a value this codebase already logs.

### 2.5 Platform specific stationarity beyond foot mounted navigation

Engelsman and Klein's 2025 paper on controlled zero velocity updates for aerial hovering, C-ZUPT [6], is the closest published analogue to this platform in the surveyed literature, since it targets a free flying vehicle that never has the ground contact a foot mounted sensor relies on to define its stationary phase. Their detector monitors two sliding window norms over the past `K` samples, the specific force norm `||f~[k−K:k]||₂` and, distinctively, the vehicle's own estimated body velocity norm drawn from the navigation filter's own state, `||v̂^B[k−K:k]||₂`, declaring a quasi static equilibrium and triggering a zero velocity pseudo measurement `y_k = C_v x_k = v^B = 0` only when both norms stay below their respective thresholds `δ_f` and `δ_v̂` for the entire window, which is injected as a measurement update into a linear quadratic Gaussian estimator and controller running on a real hovering quadrotor. The reported gains, quoted directly from the paper's results section rather than inferred, are a 28.8% tracking error reduction at the paper's disturbance level `Γ = 0.005`, a 21.6% attitude error reduction, a 76.9% reduction in control saturation events, a 31.9% reduction in control effort, a 28% reduction in estimation uncertainty, and roughly a minute of extended hover time per flight together with a 3 to 7% reduction in power consumption, all attributable to the resulting reduction in control effort once the estimator's velocity error is periodically zeroed rather than allowed to integrate freely between hover corrections. The paper is explicit about why this detector can exist on an aerial platform at all despite the platform never touching down, "recent advancements in control strategies have enabled aerial platforms to maintain nearly fixed positions while hovering, even in the presence of significant disturbances" [6], that is, the quasi static equilibrium the detector looks for is a property of the control loop holding the vehicle still, not a passive mechanical contact.

The detail that matters most for our setting is architectural rather than numerical, C-ZUPT's own gate is not IMU only, it is a conjunction of a specific force condition and an estimated velocity condition, exactly the missing ingredient identified in section 1 and reproduced quantitatively in section 3.4, and this is evidence from the closest analogous platform in the literature that the established fix, once a vehicle leaves the ground contact regime every detector in section 2.2 was built for, is to fold an already available velocity estimate directly into the gate rather than to keep refining a test built from raw inertial signal shape alone. Section 4.3 below proposes exactly this, adapted to the one frame lag Basalt's own optimisation loop requires that an EKF based filter such as C-ZUPT's does not.

### 2.6 Vision and VIO specific stationarity and excitation cues

A visual inertial estimator carries a second, physically independent motion sensor the pure inertial literature in sections 2.2 through 2.5 has no access to, the camera, and Basalt already logs a per frame summary of it, `flow_px_mean` and `flow_px_max` in `of_frame.csv`, produced by the front end at `include/basalt/optical_flow/frame_to_frame_optical_flow.h#L254`, with no modification needed to obtain it. Optical flow magnitude is established practice as a motion cue in visual odometry generally, since true rest produces near zero flow together with near gravity specific force, while constant velocity cruise against any scene at finite depth necessarily produces non zero parallax flow proportional to `v / d` for depth `d`, even though specific force still reads near gravity throughout, which is the vision side of exactly the same architectural idea as C-ZUPT's velocity gate in section 2.5, a second quantity that is zero in the stationary case and generically non zero in the constant velocity case the IMU alone cannot resolve. Section 3.4 measures this correlation directly in the logged data rather than asserting it, and finds it real but considerably weaker and noisier at the single frame level than the clean theoretical story above suggests, which section 4.5 below treats honestly as a partial, complementary signal rather than as a standalone fix.

A related but distinct body of work addresses motion excitation sufficiency for calibration and initialisation rather than stationarity directly, and is worth separating carefully from the stationarity question because the two are often conflated in VIO literature. Qin, Li and Shen's VINS-Mono [8] solves its own visual inertial alignment as a linear least squares problem over a sliding initialisation window to recover metric scale, gravity, velocity, and gyroscope bias jointly, and the paper notes directly that "acceleration excitation is needed to make metric scale observable" in this formulation, since a window with no linear acceleration leaves the linear system that solves for scale rank deficient, and separately that its keyframe selection logic applies rotation compensation specifically to avoid accepting pure rotation segments, where features at infinite effective depth cannot be triangulated at all. Neither of these is the stationarity test this document is about, VINS-Mono's concern is whether an initialisation window was informative enough to solve for scale and bias, not whether the vehicle is currently at rest, but the underlying mathematics is the same observability argument generalised, a motion segment with insufficient linear acceleration, or insufficient rotation, leaves some parameter of the estimation problem unobservable regardless of how long the segment is extended, and true stationarity is simply the most extreme point in that space, where both linear acceleration and rotation vanish simultaneously rather than merely one of them. Yang, Geneva and Huang's 2024 analysis of multi IMU visual inertial systems [9] formalises this generalised observability question directly, proving that four unobservable directions, the classical VIO gauge freedoms, persist "no matter how many inertial sensors are used," and separately characterising specific degenerate motion primitives, including sustained constant velocity travel, that leave particular spatiotemporal extrinsic or intrinsic calibration parameters unobservable for as long as that motion persists, independent of segment length. This is a superset of the stationarity problem rather than a competing algorithm for it, but it supplies the correct general purpose tool for recognising any degenerate motion, a windowed covariance or information measure of the excitation signal, the same family of statistic as section 4.1's own windowed SHOE test, which Basalt's analysis dashboard already computes offline for this exact observability purpose, over `excitation_covariance`'s windowed world frame specific force statistic in `scripts/analysis/metrics.py#L233-L243`, without yet porting that computation into the live estimator.

### 2.7 Why no statistical detector reaches zero false stationary, and the detectors that do not rely on statistics (added 2026-10-25)

Every detector in sections 2.1 to 2.6 is a threshold test on a statistic, and the Neyman-Pearson construction of section 2.1 [10] fixes what such a test can promise. It bounds the probability of rejecting a truly stationary window. It cannot bound the probability of accepting a moving one, because `H0` carries no model, and for any motion whose statistic is distributed exactly as under `H1` the two error rates are tied. Constant velocity flight is such a motion for every inertial statistic (section 1), and a distant or textureless scene makes it such a motion for the flow statistic of section 4.5. For those motions the probability of a false stationary equals the probability of a true detection, so driving the former to zero drives the latter to zero as well. Wahlström and Skog's review states that the design of a robust zero velocity detector is still an open problem [2], and Wahlström et al.'s Bayesian adaptive threshold [20] trades the costs of missed detections and false alarms rather than removing either. A guarantee of zero false stationary therefore cannot come from a better statistic. It has to come from a signal that is zero by construction whenever the body could be moving, and the literature offers three families of such signals.

The first is a one-way latch on the start of the run. OpenVINS [14] applies its zero velocity update only while the platform has not yet moved when `zupt_only_at_beginning` is set. Its `VioManager` sets `has_moved_since_zupt` on the first propagation that is not a zero velocity update and never clears it, and every later call to `UpdaterZeroVelocity::try_update` is skipped while the flag is set. The detector inside the latch combines an IMU chi squared test, a bound `zupt_max_velocity` on the estimated speed and a feature disparity test, `disp_avg < zupt_max_disparity` with more than 20 features, which alone accepts the update. The latch removes every false stationary after the first rejection, but the first moving frames are still judged by the statistical test, so the guarantee holds only if the test rejects the very first moving frame.

The second is contact sensing, the construction legged robots use. Bloesch et al. [17] apply the zero velocity of a stance foot as a kinematic constraint gated by contact, and Camurri et al. [18] estimate the contact probability from joint torques instead of a contact switch. The zero velocity is a property of the contact, not of the motion statistics, so its validity follows from the mechanics. Ramanandan, Chen and Farrell [19] make the same move for road vehicles, where stationary updates are triggered from vehicle sensors rather than from inertial statistics alone. For aircraft the contact sensor is a weight on wheels switch, which ArduPilot exposes as `AP_LandingGear::get_wow_state` (`libraries/AP_LandingGear/AP_LandingGear.h:64-70`) and which loosens its own land detector when present.

The third is the autopilot's own landed state, a contact proxy derived from actuation. PX4's land detector passes through ground contact, maybe landed and landed states, each adding constraints on vertical and horizontal speed and thrust to the last [15]. ArduCopter's `Copter::update_land_detector` (`ArduCopter/land_detector.cpp:37-158`) sets `land_complete` whenever the motors are disarmed, and while armed only after the throttle has sat at its lower limit for `LAND_DETECTOR_TRIGGER_SEC`, 1 s (`ArduCopter/config.h:354-355`). It clears it during take-off only once the motors have spooled up to `THROTTLE_UNLIMITED`, before which the take-off controller forces the throttle to decay to zero (`ArduCopter/takeoff.cpp:143-149`). While `land_complete` is true the rotors are therefore unable to produce lift, and a multirotor cannot move under its own power. `Copter::set_land_complete` publishes the state as `likely_flying = !land_complete` (`land_detector.cpp:207-229`), and AP_DDS sends it as `ardupilot_msgs/Status.flying` on `/ap/status` (`libraries/AP_DDS/AP_DDS_Client.cpp:731-789`).

For our setting the third family is the only one that is both available and exact. The latch of [14] is cheap and Basalt-internal but inherits the statistical test's behaviour at the first moving frame (section 3.5 measures it), contact sensors do not exist on this airframe, and the autopilot's landed state is already published on the bus the SLAM driver reads. Section 4.8 builds the detector on it.

## 3. Current Implementation

### 3.1 The detector

The live test, `src/vi_estimator/sqrt_keypoint_vio.cpp#L373-L438`, runs once per `ProcessFrame` call, after preintegrating every IMU sample between the previous and current frame. Over that single frame interval, typically 50 to 105 ms and 8 to 20 samples at this dataset's 167 Hz IMU rate, it accumulates the per sample deviation of accelerometer magnitude from a fixed 9.81 and the per sample gyroscope norm, keeping only the maximum of each over the window rather than the mean or the energy statistic every detector in section 2.2 uses,

```cpp
const Scalar accelDeviation = std::abs(imuData->accel.norm() - Scalar(9.81));
if (accelDeviation > accelDeviationMax) accelDeviationMax = accelDeviation;
const Scalar gyroNorm = imuData->gyro.norm();
if (gyroNorm > gyroNormMax) gyroNormMax = gyroNorm;
```

and declares the frame stationary if both maxima clear their configured tolerances and at least one sample was integrated,

```cpp
mpIsStationary = numImuIntegrated > 0 &&
                accelDeviationMax < mpStationaryAccelTol &&
                gyroNormMax < mpStationaryGyroTol;
```

with defaults `mpStationaryAccelTol = 0.5` m/s² and `mpStationaryGyroTol = 0.05` rad/s at `include/basalt/vi_estimator/sqrt_keypoint_vio.h#L283-L286`. In the language of section 2.2 this is a MAG detector on accelerometer magnitude fused by logical AND with an ARE style gyroscope test, but using the sample maximum rather than the window mean or energy, comparing accelerometer magnitude against a hardcoded 9.81 rather than the window's own estimated gravity direction as the GLRT derivation in section 2.1 shows is the maximum likelihood choice, and with no persistence or hysteresis requirement, the flag is recomputed from scratch every single frame with no memory of the previous one, unlike every detector surveyed in sections 2.2 through 2.5, each of which decides over a window spanning multiple samples and several of which, C-ZUPT explicitly, require the sub threshold condition to hold for the window's full duration before triggering. `mpIsStationary`, `accelMean`, `gyroMean`, `accelDeviationMax` and `gyroNormMax` are all written to `vio_imu_frame.csv` by `AddVioImuFrame` for offline inspection, which is what makes the measurement in section 1 and section 3.4 possible without instrumenting anything new.

### 3.2 What the flag gates

`mpIsStationary` has exactly one consumer, the zero velocity soft constraint inside the optimisation loop at `src/vi_estimator/sqrt_keypoint_vio.cpp#L1583-L1807`. The constraint is active only when both a configured weight is positive and the current frame is flagged stationary, `const bool applyZeroVelocity = mpZeroVelocityWeight > 0 && mpIsStationary`, and when active it adds a weighted zero mean Gaussian prior on the last state's own velocity, contributing `zeroVelocityW2 * frame_states.at(last_state_t_ns).vel_w_i.squaredNorm()` to the cost and the corresponding residual and identity shaped Jacobian block into the normal equations, where `zeroVelocityW2 = mpZeroVelocityWeight²`. This is, in the language of section 2.3, exactly a Foxlin style ZUPT pseudo measurement pinning the velocity block of the state toward zero, implemented as a soft residual inside Basalt's batch nonlinear least squares optimisation rather than as a single step Kalman gain, the two being informationally equivalent ways of injecting the same constraint into their respective estimator architectures. `vio_zero_velocity_weight` defaults to 0.0 in `src/utils/vio_config.cpp#L89` and is non zero in the committed template `data/sitl_config_vo.json` (100.0), a VO only template the VO estimator never reads. Correction, 2026-10-19. This section previously stated the constraint was inert in every configuration this document's logged experiments were run under, inferred from the committed `data/*.json` templates alone rather than checked against the actual per-run `config.json` each experiment logged. Checking those logged configs directly shows `phase_3b_1` and `phase_3b_2` were in fact run with `vio_zero_velocity_weight = 100.0`, so the constraint was live, gated by the single-frame detector this section analyses, for exactly those two experiments. Since that detector's own measured false positive rate against its shipped defaults is 0.549 (section 3.4), the constraint would have been pinning `vel_w_i` toward zero on roughly half of every genuinely moving frame throughout both flights, which means `v_norm`, the fused velocity estimate section 3.3 and 3.4 use as ground truth to classify a frame truly stationary or truly moving, was not a clean, unconstrained reference for those two experiments, it was itself partly shaped by the constraint under evaluation. The other six experiments, where the field is either absent (an older binary predating it) or present at 0.0, do not carry this confound. This does not invalidate section 3.4's qualitative finding that tightening `gyroTol` helps, since the same confound affects every column of that sweep equally, but it does mean the absolute TPR/FPR numbers computed against `phase_3b_1` specifically should be read as a description of a `v_norm` partially shaped by the very constraint being tuned, not an independent reference.

### 3.3 Cross run measurement of the failure

Section 1's table already shows the headline numbers across all eight experiments. Restricting to `phase_3b_1` for a closer look, joining `vio_imu_frame.csv`, `vio_state.csv` and `of_frame.csv` on `t_ns` gives 3183 common frames, of which 61 are genuinely at rest by a strict criterion, `v_norm < 0.02` m/s, and 3077 are unambiguously moving, `v_norm > 0.1` m/s. Comparing the two existing inputs to the detector across these two populations shows the theoretical argument from section 1 directly in the data,

| Quantity | Truly stationary (n=61) | Truly moving (n=3077) |
|---|---|---|
| `accel_deviation_max`, mean / median / p90 | 0.121 / 0.109 / 0.187 | 0.272 / 0.147 / 0.594 |
| `gyro_norm_max`, mean / median / p90 | 0.0197 / 0.0156 / 0.0209 | 0.0663 / 0.0423 / 0.1467 |
| `flow_px_mean`, mean / median / p90 | 0.441 / 0.393 / 0.579 | 1.741 / 0.938 / 3.704 |

The two populations overlap substantially on both existing inputs, the 90th percentile of `accel_deviation_max` among truly stationary frames, 0.187, sits comfortably inside the moving population's own interquartile range, and the same is true of `gyro_norm_max`, which is the direct, measured cause of the headline false positive rate, under Basalt's defaults of `accelTol = 0.5` and `gyroTol = 0.05`, essentially every truly moving frame whose instantaneous motion happens to be smooth and slow, a turn entered or exited gently, a straight segment at constant altitude, clears both tolerances at some point within its one frame window.

### 3.4 Exhaustive sweep of the two existing tolerances

To separate "the test is fundamentally blind to constant velocity" from "the test is merely mistuned," every combination of `accelTol ∈ {0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 0.7}` and `gyroTol ∈ {0.01, 0.015, 0.02, 0.03, 0.05, 0.07}` was evaluated against the same truly stationary (n=61) and truly moving (n=3077) populations from section 3.3, recording the true positive rate, the fraction of truly stationary frames correctly flagged, and the false positive rate, the fraction of truly moving frames incorrectly flagged.

| `accelTol` | `gyroTol` | TPR | FPR |
|---|---|---|---|
| 0.5 (current default) | 0.05 (current default) | 0.967 | 0.549 |
| 0.2 | 0.02 | 0.803 | 0.043 |
| 0.3 | 0.02 | 0.852 | 0.045 |
| 0.5 | 0.02 | 0.852 | 0.046 |
| 0.1 | 0.02 | 0.410 | 0.020 |
| 0.3 | 0.015 | 0.426 | 0.011 |

Two findings follow directly from the full sweep, of which the above is a representative excerpt. First, Basalt's shipped defaults sit at essentially the worst usable point on this tradeoff curve, a 0.549 false positive rate at only a marginally better true positive rate than the much better operating points immediately below it, which confirms that a meaningful share of the headline failure in section 1 and section 3.3 is a tuning defect and not only a fundamental limitation, the gyroscope tolerance of 0.05 rad/s is simply too loose for this platform's typical cruise angular rates. Second, and consistent with the literature finding in section 2.2 that the gyroscope carries most of the discriminative power, the accelerometer tolerance barely moves the frontier once the gyroscope tolerance is tightened, `accelTol = 0.2, 0.3, 0.5` and even `0.7` all give nearly identical TPR and FPR at a fixed `gyroTol = 0.02`, so retuning should concentrate on the gyroscope term.

Even at the best operating points found in the full sweep, a residual false positive rate of roughly 4 to 5% remains, which is the number this document's introduction attributes to the genuinely unresolvable constant velocity case rather than to mistuning. Isolating the 131 residual false positives at `accelTol = 0.2, gyroTol = 0.02` and grouping them into contiguous runs of consecutive flagged frames shows this residual is not uniform, it is a mixture of two distinct populations. Sixty eight separate runs were found, with a median run length of one frame and a mean of 1.9, the great majority isolated single frame false alarms consistent with ordinary sensor noise momentarily dipping below both tolerances, exactly the kind of transient a multi sample window and a persistence requirement, as every detector in sections 2.2 through 2.5 already imposes and Basalt's own single frame test does not, would filter out. But two of those sixty eight runs are sustained, one reaching 49 consecutive frames, together accounting for 59 of the 131 residual false positives, with a median velocity of 0.402 m/s and a median `flow_px_mean` of only 0.337, close to the truly stationary population's own median of 0.393, this is the genuine, sustained, straight line constant velocity cruise case the introduction's theoretical argument predicts, and no amount of further threshold tightening or persistence filtering on the IMU signal alone can remove it, because across that entire 49 frame run the IMU genuinely does read as though the vehicle were at rest.

This decomposition is the quantitative basis for section 4's layered recommendation, a tightened, multi frame, persistence requiring IMU test closes most of the residual gap cheaply, by removing the isolated single frame false alarms, but a second, velocity aware signal is still required to close the remainder, the sustained cruise segments where the IMU test is not wrong because it is mistuned, it is wrong because the information it needs is not present in its inputs at all.

### 3.5 False stationary rates of the three-stage detector on the multirotor, and of an OpenVINS-style latch (added 2026-10-25)

The detector of section 4.6 logs every stage to `vio_stationary.csv` whether or not the stage is enabled, so its behaviour can be measured on flights where it gated nothing. Thirteen SITL runs of 2026-10-24 and 2026-10-25 carry both that file and `gt_eval.csv`. Ground truth there is `/ap/pose/filtered`, ArduPilot's EKF estimate, which creeps by up to 13 cm on the ground while the image flow stays at 0.25 px, so a displacement threshold mislabels the ground phase as motion. Motion onset is therefore taken as the first of three consecutive frames whose ground truth speed over a 0.25 s baseline exceeds 0.15 m/s, four times that creep, and every later frame counts as moving.

| Detector | False stationary, of 30003 moving frames | Ground frames accepted, of 1554 |
|---|---|---|
| SHOE, stage 1 | 17384 (57.9%) | 98.1% |
| Flow, stage 3 | 11185 (37.3%) | 93.8% |
| SHOE and flow | 8416 (28.1%) | 92.7% |
| Flow inside a one-way latch that closes on the first non-stationary frame | 5 (0.02%) | 48.5% |
| Flow inside a one-way latch that closes after 3 consecutive non-stationary frames | 5 (0.02%) | 93.8% |

The SHOE stage is true on 54 to 64 percent of moving frames in the three runs of 2026-10-25, including the whole lift-off, because steady flight presents a constant specific force and no rotation (section 1). The flow stage fires at up to 5.3 m/s when the scene is distant. A latch in the manner of OpenVINS's `zupt_only_at_beginning` [14] removes almost all of it. Closing on a single non-stationary frame loses half of the ground phase, because the flow stage drops out on 23 to 38 percent of frames during motor spool-up, and requiring three consecutive frames keeps 93.8 percent. The five remaining false stationary frames all come from `2026-10-24_11-29-39`, where the lift-off flow stayed below the 0.8 px threshold for 0.25 s (0.2 to 0.77 px at 0.2 to 0.44 m/s, five pyramid levels), so the latch closed five frames late. A Basalt-internal latch therefore reaches 0.02 percent on this campaign but cannot guarantee zero, because its first moving frames are still judged by the flow statistic.

The dataflash logs bound the alternative of section 2.7. Across the 16 most recent logs, about 50 take-offs, physical motion of 2 cm in the simulator's true position (`SIM`) began 0.054 to 0.69 s after the `NOT_LANDED` event (event 28), typically 0.09 to 0.19 s. Across 87 landed intervals of 30 logs, 559 s in total, from `ARMED` or `LAND_COMPLETE` to `NOT_LANDED` or `DISARMED` with the first 0.5 s skipped, the true position never moved by more than 1.0 cm, and the one apparent speed of 0.28 m/s is a single sample of the 1e-7 degree position quantisation. AP_DDS evaluates the status every `AP_DDS_DELAY_STATUS_TOPIC_MS`, 100 ms by default (`libraries/AP_DDS/AP_DDS_config.h:109-111`, `AP_DDS_Client.cpp:1967-1972`), publishes it at once on a change and otherwise at 2 Hz (`AP_DDS_Client.cpp:760-787`). The 100 ms evaluation period exceeds the shortest 54 ms gap between `NOT_LANDED` and motion, so a gate that simply follows the latest `flying` value could still admit a moving frame. Section 4.8 removes that dependence on latency.

## 4. Candidate Solutions

### 4.1 Multi frame SHOE style GLRT with persistence, in place of the single frame maximum

Replace the per frame maximum test with the full SHOE statistic from section 2.1 and section 2.2, accumulated over a sliding window spanning several frames, `W_s` on the order of 0.5 to 1 s, 80 to 170 samples at 167 Hz rather than one optical flow frame interval, and require the statistic to stay below threshold for the whole window before latching the flag, rather than recomputing it independently every frame as the current code does,

```cpp
// Member-level ring buffer spanning the last W_s seconds of IMU samples,
// replacing the per-frame accelSum/gyroSum which reset every ProcessFrame
// call. mvRingAccel/mvRingGyro hold raw samples, mpStationaryRunLength
// counts consecutive windows that have already cleared the threshold.
struct ImuSample { Vec3 accel, gyro; };
std::deque<ImuSample> mvStationaryWindow;  // bounded to W_s seconds

// inside ProcessFrame, after the existing accel/gyro integration loop:
for each imuData in this frame's integration range:
  mvStationaryWindow.push_back({imuData->accel, imuData->gyro});
while (window spans more than mpStationaryWindowS seconds) mvStationaryWindow.pop_front();

const Vec3 aBar = meanOf(mvStationaryWindow, &ImuSample::accel);
Scalar T_shoe = Scalar(0);
for (const auto& s : mvStationaryWindow) {
  const Vec3 gDir = aBar / aBar.norm();
  const Vec3 accelResidual = s.accel - Scalar(9.81) * gDir;
  // mpAccelCov and mpGyroCov are Vec3, the per-axis noise variance itself
  // (sqrt_keypoint_vio.h:293), not a matrix, so cwiseQuotient normalises
  // each axis by its own variance directly rather than calling the
  // nonexistent .diagonal() this snippet called for before 2026-10-19.
  T_shoe += accelResidual.cwiseAbs2().cwiseQuotient(mpAccelCov).sum()
          + s.gyro.cwiseAbs2().cwiseQuotient(mpGyroCov).sum();
}
T_shoe /= Scalar(mvStationaryWindow.size());

const bool windowClears = T_shoe < mpStationaryGlrtTol &&
                           mvStationaryWindow.size() >= mpStationaryMinSamples;
mpStationaryRunLength = windowClears ? mpStationaryRunLength + 1 : 0;
mpIsStationary = mpStationaryRunLength >= mpStationaryMinConsecutiveFrames;
```

reusing `mpAccelCov` and `mpGyroCov`, the noise covariances already configured and already passed into `meas->integrate()`, as the `σ_a²` and `σ_ω²` normalisers the GLRT derivation in section 2.1 calls for, and replacing the fixed `9.81` comparison with the window's own estimated gravity direction `aBar / aBar.norm()`, the maximum likelihood substitution that same derivation shows is the statistically correct one. The threshold `mpStationaryGlrtTol` should be set from the chi squared argument in section 2.1 for an initial value, then validated against the eight logged experiments the same way section 3.4 validated the existing tolerances, by sweeping it against the measured `v_norm < 0.02` and `v_norm > 0.1` populations and reading the TPR/FPR tradeoff directly. The additional `mpStationaryMinConsecutiveFrames` run length requirement is what section 3.4's run length finding motivates directly, sixty eight of the measured false positive runs are length one or two and would be rejected outright by requiring, for instance, three or four consecutive clearing frames, roughly 0.15 to 0.2 s, while the two genuinely sustained cruise runs, 49 and the remainder of the 59 frame total, would still pass, which is the expected and correct behaviour, this candidate is designed to remove transient noise, not to solve the fundamental ambiguity.

This is the minimum effort generalisation of the existing code that matches literature best practice, and the change section 2.2's own empirical ranking most strongly supports, SHOE and ARE both outperform a MAG style magnitude test like the current one, and the run length evidence in section 3.4 shows a persistence requirement would remove roughly half of the residual false positives essentially for free. It remains, by construction, structurally blind to sustained constant velocity, so on its own it would close the isolated noise component of the gap but leave the 59 frame sustained cruise component measured in section 3.4 untouched, which is why section 4.3 and section 4.4 below are proposed as a required second layer rather than an optional refinement.

### 4.2 Preintegration covariance Mahalanobis test, native to Basalt's own preintegration

`IntegratedImuMeasurement` already accumulates both a delta state, `getDeltaState()`, and its propagated covariance, `get_cov()`, a 9 by 9 matrix ordered `[trans, rot, vel]` per `thirdparty/basalt-headers/include/basalt/imu/preintegration.h#L138-L359`, computed every frame as a side effect of the `meas->integrate()` call the estimator already makes, whether or not any stationarity test reads it. A Mahalanobis test on the preintegrated velocity change against the hypothesis of zero velocity change costs nothing beyond a 3 by 3 block extraction and a linear solve,

```cpp
const auto& cov = meas->get_cov();                 // 9x9, [trans, rot, vel]
const Mat3 Sigma_v = cov.template block<3, 3>(6, 6);
const Vec3 dv = meas->getDeltaState().vel_w_i;      // preintegrated delta-v
const Scalar d2 = dv.transpose() * Sigma_v.ldlt().solve(dv);
mpIsStationary = d2 < mpStationaryChiSqTol;         // chi-squared critical value, 3 DOF
```

`d2` is the squared Mahalanobis distance of `dv` from zero under the metric `Σ_v` itself defines, not a raw Euclidean distance. A raw Euclidean test, `||dv|| < tol`, treats every direction in velocity space as equally noisy, but `Σ_v`, the actual propagated covariance of the three components of `dv`, generally is not isotropic, an axis the accelerometer is more precise along, or one the current bias linearisation makes more certain, has a smaller variance in `Σ_v` along that axis, and the same raw deviation along it should count as far more surprising than an equally large deviation along a noisier axis. Dividing out by `Σ_v` is exactly what makes the statistic directionally fair, a `dv` of a given raw size is judged relative to how much the sensor model itself says deviations of that size, in that particular direction, actually happen under `H1`.

The reason this division is written as a linear solve, `Sigma_v.ldlt().solve(dv)`, rather than as an explicit matrix inverse, `Sigma_v.inverse() * dv`, is both a numerical and a computational choice. `Σ_v` is symmetric positive semi definite by construction, being a propagated noise covariance, and `.ldlt()` computes its `L D L^T` decomposition, a Cholesky style factorisation specialised for exactly this kind of matrix, after which `.solve(dv)` back substitutes through `L`, then `D`, then `L^T` to obtain `Σ_v^{-1} dv` without ever forming `Σ_v^{-1}` as an explicit matrix. Forming the inverse explicitly is both more arithmetic than necessary, since only the single product `Σ_v^{-1} dv` is ever needed, and less numerically stable, since explicit inversion amplifies whatever conditioning problems `Σ_v` has before the result is even used, where solving directly against the factorisation does not. This is not a new idiom introduced for this candidate, it is the exact same operation, on the exact same `cov_` matrix, that Basalt's own `compute_sqrt_cov_inv()` already performs at `thirdparty/basalt-headers/include/basalt/imu/preintegration.h#L337-353` to whiten this same preintegrated measurement's residual for the estimator's own IMU factor, so `d2`'s `.ldlt()` call is reusing, at the level of numerical method, machinery the optimisation backend already relies on for this exact covariance matrix, not introducing anything novel to the codebase.

Relating `d2` back to section 2.1's chi squared construction makes precise why the comparison against a chi squared critical value is not an approximation here, unlike the SHOE statistic's own `χ²_{6W−2}` result, which needed a degrees of freedom correction for the gravity direction fitted from the same window. Write the `L D L^T` factorisation underlying the solve above as an equivalent Cholesky factor `Σ_v = L L^T`, and define `w = L^{-1} dv`. Then `Cov(w) = L^{-1} Σ_v L^{-T} = L^{-1} (L L^T) L^{-T} = I_3`, so, under `H1`, with `dv` genuinely drawn from `N(0, Σ_v)`, `w` is exactly three independent standard normal variables, and

```
d2 = dv^T Σ_v^{-1} dv = dv^T (L L^T)^{-1} dv = (L^{-1} dv)^T (L^{-1} dv) = w^T w = ||w||²
```

is, by the very definition given in section 2.1, a sum of three squared standard normals, hence exactly `χ²_3`, with no flat limit or large signal approximation required, because nothing here was fitted from `dv` itself, `Σ_v` is computed independently by error propagation through the sensor noise model `mpAccelCov`/`mpGyroCov` rather than estimated as a nuisance parameter from the same data the test statistic reads, so there is no parameter count to subtract. `d2 < mpStationaryChiSqTol` for `mpStationaryChiSqTol = 7.81`, the 95% critical value of `χ²_3`, is therefore an exact statement, under the Gaussian preintegration noise model Basalt already assumes elsewhere, about the probability that a genuinely stationary window produces a `dv` this large purely by sensor noise, in precisely the same sense section 2.1 derived for the SHOE statistic, just without that derivation's one complicating wrinkle.

This differs from the raw accelerometer test in section 3.1 and from the windowed statistic in section 4.1 by testing an accumulated velocity change rather than an instantaneous or windowed specific force deviation, so a small, correlated drift in specific force across the window, which a MAG or SHOE style test only sees as a sequence of individually small deviations, shows up directly as a non zero `dv` that the Mahalanobis distance is sensitive to in a way an averaged energy statistic is not. It is also the cheapest candidate surveyed here in CPU terms, since `cov_` is computed by the existing preintegration call regardless of whether this test exists, the only new cost is one 3 by 3 Cholesky style solve per frame. It inherits the same fundamental limitation measured in section 3.4 as every other purely inertial detector, a single frame's `dv` over 50 to 105 ms carries too little signal to separate true rest from slow constant velocity cruise, the preintegrated delta velocity over that short an interval is near zero in both cases, so this test needs the same multi frame accumulation `4.1` adds, summing `d²` over a sliding window of several preintegration segments rather than testing one frame's `dv` in isolation, to be useful as anything more than a cheaper drop in replacement for section 3.1's existing accelerometer term. Section 4.4 below takes this same Mahalanobis construction and applies it to a different, and more directly useful, quantity than `dv` alone.

### 4.3 Cross check against the VIO state's own velocity estimate

`frame_states.at(last_state_t_ns)` already holds `vel_w_i`, the optimiser's own fused estimate of the vehicle's velocity, available after every optimisation call, and this is precisely the quantity section 1's theoretical argument and section 3.4's measurement both identify as the missing ingredient, a signal that correctly reads non zero during constant velocity cruise because it fuses vision, not only specific force and angular rate. Gating stationarity additionally on `frame_states.at(prev_state_t_ns).getState().vel_w_i.norm()` below a small threshold,

```cpp
const bool vioStateAgrees =
    frame_states.count(prev_state_t_ns) &&
    frame_states.at(prev_state_t_ns).getState().vel_w_i.norm() < mpStationaryVelTol;
mpIsStationary = imuTest(...) && vioStateAgrees;
```

directly targets the 59 frame sustained cruise residual measured in section 3.4, since that residual is defined exactly by the IMU test accepting while the fused velocity estimate is unambiguously non zero, 0.402 m/s median across the sustained run. The one structural caveat, worth stating precisely rather than glossing over, is that `mpIsStationary` currently gates the optimisation call that produces `vel_w_i` for the current frame, so this check must read the previous frame's post optimisation velocity, not the one about to be computed, introducing a one frame lag of 50 to 105 ms. Measuring that lag's cost directly against `phase_3b_1`, by comparing, for every consecutive pair of frames, whether the previous frame's `v_norm < 0.1` m/s decision agrees with the current frame's, shows disagreement at only 23 of 3182 transitions, 0.72%, almost entirely concentrated at the true onset and offset of motion where a one frame lag is expected and tolerable, not spread across the sustained cruise interior where this candidate is meant to help. This mirrors C-ZUPT's own design in section 2.5, which gates its aerial hover detector on the filter's own estimated body velocity norm for exactly the same reason, specific force alone cannot distinguish a held hover from level cruise, and the measured 0.72% disagreement rate here is the direct, dataset specific confirmation that the one frame lag this architecture forces is a minor cost relative to the 94 to 99% false positive rate it would remove.

### 4.4 Combined preintegration prediction and velocity Mahalanobis test

Sections 4.2 and 4.3 each have exactly one weakness, and each weakness is exactly what the other candidate's own quantity is positioned to fix. Section 4.2's `dv` is statistically principled, a proper Mahalanobis distance against a propagated covariance, but it is blind to velocity itself by construction, it tests only whether velocity changed, so it cannot see the constant velocity cruise case section 3.4 isolates no matter how the window is extended. Section 4.3's `vel_w_i` is exactly the quantity that does see velocity, but the candidate as written compares it against nothing more than a raw threshold, with no covariance behind it, and needs a one frame lag because it reads the very quantity `mpIsStationary` is used to help compute. `IntegratedImuMeasurement::predictState`, already called by the estimator itself at `src/vi_estimator/sqrt_keypoint_vio.cpp#L579-L583` to seed its own next state guess, combines both quantities into one already, `state1.vel_w_i = state0.vel_w_i + g·dt + R_0 · dv`, where `state0` is read from `frame_states.at(last_state_t_ns)`, the previous keyframe's already optimised state, `R_0 = state0.T_w_i.so3()` is its orientation, and `dv = meas->getDeltaState().vel_w_i` is the same preintegrated delta velocity section 4.2 already uses. Because `state0` is the previous, already converged state rather than the current frame's own not yet computed one, reading it here carries none of section 4.3's circularity, `predictState` is a pure kinematic extrapolation that needs nothing the current frame's optimisation has not already produced, so the combined test below needs no one frame lag at all.

The quantity to test is whether the predicted next velocity `v1 = state1.vel_w_i` is itself close to zero, not merely whether it is close to `state0.vel_w_i`, and this single choice is what gives the combined test the property neither 4.2 nor 4.3 has alone. If the vehicle is truly stationary, `state0.vel_w_i` is already near zero and `dv` reflects only sensor noise around the specific force needed to hold still against gravity, so `v1` comes out near zero. If the vehicle is cruising at a roughly constant velocity, exactly the case section 1 shows no IMU only test can resolve, `dv` is still near its stationary expected value, since there is little or no acceleration, but `state0.vel_w_i` is not near zero, and that non zero value survives unchanged into `v1`, because nothing in the prediction formula cancels a non zero `state0.vel_w_i` against a near zero `dv`. Testing `v1` rather than `dv` is therefore what inherits section 4.3's sensitivity to the fundamental ambiguity, while keeping section 4.2's covariance weighted, chi squared grounded test rather than section 4.3's bare threshold.

Deriving the covariance of `v1` needs one approximation, stated explicitly rather than left implicit, that `state0.vel_w_i` and `R_0` are treated as known constants rather than as themselves uncertain, so that the only randomness in `v1` is the randomness already propagated into `Σ_v`, the same `cov_` block section 4.2 reads. Under this approximation `Cov(v1) = R_0 Σ_v R_0^T`, a rotation of the same covariance by the same orthogonal matrix that rotates `dv` into world frame in the prediction formula itself, and because `R_0` is orthogonal, `(R_0 Σ_v R_0^T)^{-1} = R_0 Σ_v^{-1} R_0^T`, which means the Mahalanobis distance of `v1` from zero can be computed by rotating `v1` back into the integration start frame first and then reusing `Σ_v` directly, with no need to ever form the rotated covariance `R_0 Σ_v R_0^T` as its own 3 by 3 matrix,

```cpp
// Guarded on numImuIntegrated > 0, added 2026-10-19 in
// plans/stationary_detection.md change 4. Sigma_v is exactly zero before
// any integrate() call this frame, and ldlt().solve() on a zero matrix is
// singular, a case this snippet did not handle before that date.
if (numImuIntegrated > 0) {
  const auto& cov = meas->get_cov();
  const Mat3 Sigma_v = cov.template block<3, 3>(6, 6);
  const Vec3 dv = meas->getDeltaState().vel_w_i;
  const auto& state0 = frame_states.at(last_state_t_ns).getState();
  const Scalar dt = Scalar(meas->get_dt_ns()) * Scalar(1e-9);

  // residual = R_0^T * v1_pred, algebraically simplified so Sigma_v itself
  // never needs to be rotated, only the much cheaper vector does:
  //   v1_pred = state0.vel_w_i + g*dt + R_0*dv
  //   R_0^T v1_pred = R_0^T*state0.vel_w_i + R_0^T*g*dt + dv
  const Vec3 residual =
      dv + state0.T_w_i.so3().inverse() * (state0.vel_w_i + g * dt);

  const Scalar d2 = residual.transpose() * Sigma_v.ldlt().solve(residual);
  mpIsStationary = d2 < mpStationaryChiSqTol;  // chi-squared critical value, 3 DOF
}
```

and because this is, after the rotation, exactly the same `L^{-1}(·)` standardisation argument section 4.2 already gave for `dv` applied now to `residual`, `d2` here is, under the same stated approximation, exactly `χ²_3` as well, with the same `mpStationaryChiSqTol ≈ 7.81` at 95% confidence carrying over unchanged. The approximation of treating `state0.vel_w_i` as exactly known rather than itself uncertain is the one place this candidate is not fully rigorous, Basalt's square root sliding window smoother does not expose a single state's own marginal covariance without an explicit back substitution the estimator does not currently perform, so a more complete version would add `R_0^T Σ_{v0} R_0` to `Σ_v` inside the solve above, where `Σ_{v0}` is `state0.vel_w_i`'s own marginal covariance, were it available, which would only widen the gate and make the test slightly more conservative, never change its qualitative behaviour, since `Σ_v`, the fresh single frame IMU noise, already dominates a converged keyframe's comparatively small residual velocity uncertainty in practice.

This construction is a direct instance of the normalised innovation squared, or NIS, consistency test from the Kalman filtering literature, where a filter's own predicted measurement and its propagated covariance are compared against an actual measurement by exactly the Mahalanobis distance built above, `ε = ν^T S^{-1} ν`, with `ν` the innovation and `S` its covariance, and `ε` following `χ²_{n_y}` for an `n_y` dimensional measurement under a correctly specified, consistent filter, the standard reference being Bar-Shalom, Li and Kirubarajan [13]. The role reversal here, treating the predicted state itself as the thing under test against a fixed hypothesis of zero rather than against an incoming measurement, is the same construction NIS based manoeuvre detectors in the tracking literature already use, a tracked target's predicted state is tested for consistency with a constant velocity model, and a NIS value far outside its expected `χ²` range signals the model no longer fits, flagging a manoeuvre, exactly mirrored here with the sign of the question reversed, testing whether the zero velocity model fits rather than whether a constant velocity model has broken down.

Section 4.6 below adopts this candidate as the second stage of the detector actually selected for implementation, paired with the growing window SHOE test from section 4.1 as the first stage.

### 4.5 Vision flow magnitude cross check

`of_frame.csv` already logs `flow_px_mean` and `flow_px_max` per frame, computed in `include/basalt/optical_flow/frame_to_frame_optical_flow.h#L254`, the mean and maximum pixel displacement of tracked points between consecutive frames, with no new instrumentation required to read it. Section 2.6 argues on physical grounds that this should separate true rest from constant velocity cruise, since flow is proportional to `v / d` for finite depth `d` while specific force stays near gravity in both cases, and this section measures that argument directly against `phase_3b_1` rather than asserting it. The correlation between `flow_px_mean` and the VIO's own `v_norm` across all 3183 common frames is real but moderate, a Pearson correlation of 0.344, and binning frames by `v_norm` shows the relationship is not even monotonic in this dataset, the `v_norm ∈ [0.3, 0.6)` bin has a lower median flow, 0.484 px, than the slower `v_norm ∈ [0.1, 0.3)` bin, 0.756 px, almost certainly because flow depends on scene depth as well as velocity and the two bins sample different parts of the mission's geometry, a texture rich near field segment against a distant, low parallax segment. This matters directly for how strong a claim this candidate can support, flow magnitude is a real but noisy proxy for velocity, not a clean substitute for it.

Quantifying this candidate's actual effect on the specific failure measured in section 3.3, among the 1673 frames `phase_3b_1`'s current defaults flag as stationary while `v_norm > 0.1` m/s, a veto rule `mpIsStationary = imuTest(...) && flow_px_mean < flowTol` performs as follows across a sweep of `flowTol`,

| `flowTol` (px) | False positives reclassified | Truly stationary frames wrongly vetoed |
|---|---|---|
| 0.5 | 957 / 1673 (57.2%) | 10 / 59 (16.9%) |
| 0.6 | 771 / 1673 (46.1%) | 5 / 59 (8.5%) |
| 0.7 | 640 / 1673 (38.3%) | 4 / 59 (6.8%) |
| 0.8 | 550 / 1673 (32.9%) | 3 / 59 (5.1%) |
| 1.0 | 387 / 1673 (23.1%) | 1 / 59 (1.7%) |
| 1.2 | 294 / 1673 (17.6%) | 0 / 59 (0.0%) |

so at the single frame level this candidate alone trades recall for precision much like every other candidate in this document, no single threshold both catches a majority of the false positives and leaves the genuinely stationary population untouched, the best compromise in this sweep, `flowTol ≈ 0.7` to `0.8`, reclassifies roughly a third to two fifths of the false positives at a 5 to 7% cost against the true positives. This is weaker, on this specific dataset, than the clean theoretical separation section 2.6 suggests and weaker than candidate 4.3's near perfect, 0.72% error cross check, because single frame flow is itself a short window, noisy statistic in exactly the sense section 3.1 criticises Basalt's own IMU test for being, and would likely improve, by the same logic as section 4.1, if averaged over a short sliding window of several frames rather than read from one frame at a time, which this document has not yet measured and flags as the natural next refinement if this candidate is pursued further. Used as intended, as a complement layered on top of 4.1, 4.3 or 4.4 rather than as a standalone replacement, it remains valuable precisely because it is a physically independent third signal, alongside specific force, angular rate, and the fused velocity estimate, whose failure modes do not coincide with any of the others, flow vanishes also for pure rotation about the camera's optical axis and for motion directly along that axis toward a point at infinity, and it degrades toward zero information in a dark or textureless frame, neither of which is a failure mode shared with the IMU based tests.

Reading `flow_px_mean` from inside the VIO backend, rather than only from the offline log, needs one small plumbing change, since today `AddOpticalFlowFrame` is handed the mean and max flow as local arguments computed from `trackStats`, and nothing stores them back on the `OpticalFlowResult` the front end actually hands downstream. `OpticalFlowResult` at `include/basalt/optical_flow/optical_flow.h#L59-L69` gains two scalar fields alongside its existing `t_ns`, `observations`, `pyramid_levels` and `input_images`,

```cpp
struct OpticalFlowResult {
    using Ptr = std::shared_ptr<OpticalFlowResult>;

    int64_t t_ns;
    std::vector<Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>>
        observations;

    std::vector<std::map<KeypointId, size_t>> pyramid_levels;

    OpticalFlowInput::Ptr input_images;

    double mFlowPxMean = 0.0;
    double mFlowPxMax = 0.0;
};
```

and `FrameToFrameOpticalFlow::processFrame` in `include/basalt/optical_flow/frame_to_frame_optical_flow.h#L253-L263` populates them immediately after the existing `AddOpticalFlowFrame`/`PrintOpticalFlowFrame` calls, reusing the exact same already computed values those calls are handed rather than recomputing anything,

```cpp
const int tracked = trackStats.mTracked.load();
mpLogger->AddOpticalFlowFrame(
    curr_t_ns, int64_t(frame_counter), trackStats.mDtSeconds,
    trackStats.mPointsBefore, trackStats.mAttempted.load(), tracked,
    trackStats.mForwardFailed.load(),
    trackStats.mBackwardFailed.load(),
    trackStats.mRecoveryRejected.load(), trackStats.mDetected,
    trackStats.mEpipolarRejected, trackStats.mOut,
    tracked ? trackStats.mFlowSum / double(tracked) : 0.0,
    trackStats.mFlowMax);
mpLogger->PrintOpticalFlowFrame();

transforms->mFlowPxMean = tracked ? trackStats.mFlowSum / double(tracked) : 0.0;
transforms->mFlowPxMax = trackStats.mFlowMax;
```

By this point in `processFrame`, `transforms` already points at the same `OpticalFlowResult` the function is about to `return`, the reassignment `transforms = new_transforms` happens earlier in the else branch, so this is not a second, parallel copy, it is the one object the VIO thread receives. `SqrtKeypointVioEstimator::ProcessFrame` already holds exactly this object as `curr_frame`, an `OpticalFlowResult::Ptr` per `include/basalt/vi_estimator/sqrt_keypoint_vio.h#L124`, so `curr_frame->mFlowPxMean` and `curr_frame->mFlowPxMax` are available for free at the same point in `src/vi_estimator/sqrt_keypoint_vio.cpp#L373-L438` where `mpIsStationary` is already computed, with no new queue, no new thread hand off, and no change to `AddOpticalFlowFrame`'s own call signature, since the logger call and the new struct fields are populated from the same `trackStats` values independently.

### 4.6 The two stage detector selected for implementation

The design chosen for implementation runs two tests every frame rather than choosing one candidate over the others, a growing window SHOE test as the first stage and the combined preintegration prediction and velocity Mahalanobis test from section 4.4 as the second, latching `mpIsStationary` only when both agree. Both stages are available from the very first integrated frame, not only after some warm up period, which matters because the two problems this document has identified, a window that is too short to be statistically powerful early in a mission and a fundamental blindness to constant velocity cruise at any window length, are independent problems with independent fixes, and neither stage needs to wait on the other.

The first stage is section 4.1's windowed SHOE statistic, but with its ring buffer `mvStationaryWindow` read as already designed to grow from empty up to its bound `W_s` rather than to be gated behind a minimum sample count before it is ever consulted, `mpStationaryMinSamples` set to 1 rather than to some larger warm up value. A `std::deque` already behaves exactly this way with no code change, the first frame after start up sees a one sample window, statistically no stronger than section 3.1's original per frame test, the second sees a two sample window, and so on up to `W_s`, at which point older samples begin falling off the front exactly as section 4.1 describes. This is deliberately favourable rather than merely tolerated, since this platform's missions begin from genuine rest, the short window test's lower statistical power during exactly the first few hundred milliseconds of a mission costs little, there is essentially never a reason for the vehicle to be moving in the first samples after start up, so a weak but correctly biased test during that interval is a reasonable trade against the alternative of having no stationarity signal at all until a full window has accumulated, which is precisely the gap a takeoff sensitive detector cannot afford.

The second stage is section 4.4's combined test exactly as derived there, `predictState` applied to the previous, already optimised keyframe together with the current frame's own preintegrated `dv`, tested as a single Mahalanobis distance against a `χ²_3` threshold. Because this stage reads only the previous keyframe's state and the current frame's own preintegration, both already available by construction, it carries no warm up requirement of its own and is exactly as strong on the first frame as on the thousandth, which is what gives the combined detector its resolution of the constant velocity case throughout the mission, not only once section 4.1's window has filled.

```cpp
// Stage 1, section 4.1's windowed SHOE statistic, min-sample gate relaxed
// to 1 so the window grows from empty rather than waiting for W_s samples
// before ever deciding:
const bool stage1Clears = T_shoe < mpStationaryGlrtTol &&
                           mvStationaryWindow.size() >= 1;
mpStationaryRunLength = stage1Clears ? mpStationaryRunLength + 1 : 0;
const bool stage1 = mpStationaryRunLength >= mpStationaryMinConsecutiveFrames;

// Stage 2, section 4.4's combined preintegration prediction and
// velocity Mahalanobis test, no warm-up requirement of its own:
const bool stage2 = d2 < mpStationaryChiSqTol;

mpIsStationary = stage1 && stage2;
```

The persistence requirement `mpStationaryMinConsecutiveFrames` from section 4.1 is kept in this combination for the same reason section 3.4's run length analysis gave it in the first place, removing the great majority of isolated, single frame false alarms essentially for free, while the chi squared thresholds for both stages, `mpStationaryGlrtTol` and `mpStationaryChiSqTol`, should be validated the same way section 3.4 validated Basalt's existing two tolerances, swept against the `v_norm < 0.02` and `v_norm > 0.1` populations across all eight logged experiments rather than tuned on a single run, with the growing window portion of stage 1 specifically checked against each experiment's own first few hundred milliseconds, since that is the one interval the rest of this document's measurements, all drawn from well into each mission, do not directly cover.

### 4.7 Comparison

| Method | Signal used | Window | Measured effect on `phase_3b_1`'s residual false positives | Implementation cost | Statistical grounding |
|---|---|---|---|---|---|
| Current test (3.1) | raw accel + gyro, per sample max | one frame, 50 to 105 ms | Baseline, 94.9% of flagged frames are false | None, already shipped | Heuristic, MAG+ARE hybrid, fixed gravity constant |
| 4.1 Multi frame SHOE/GLRT with persistence | accel + gyro, window mean energy, run length gate | 0.5 to 1 s, 3 to 4 consecutive windows | Removes roughly half the residual false positives, the isolated single frame runs, leaves the 59 frame sustained cruise run untouched | Low, reuses existing accumulation plus a bounded ring buffer | GLRT with configured noise covariances, chi-squared threshold interpretation |
| 4.2 Preintegration Mahalanobis | preintegrated delta-v + its covariance | one frame alone insufficient, needs 4.1 style accumulation | Same ceiling as 4.1, untested standalone, theoretically equivalent | Very low, `cov_` already computed | Chi squared, most native to Basalt's own noise model |
| 4.3 VIO state velocity cross check | `frame_states.vel_w_i`, one frame lag | previous frame | Directly targets the sustained 59 frame residual, measured 0.72% disagreement cost from the lag | Very low, field already exists | Consistent with the optimiser's own fused model, same architecture as C-ZUPT |
| 4.4 Combined prediction and velocity Mahalanobis test | `predictState`'s full predicted `v1` + rotated `Σ_v` | current frame, no lag, no warm-up | Designed, not yet measured, inherits 4.3's sensitivity to the fundamental ambiguity with 4.2's chi-squared gate and no lag | Very low, `predictState` and `cov_` already computed | Chi squared, exact under the stated approximation, same NIS construction as Kalman filter consistency tests |
| 4.5 Optical flow magnitude cross check | `flow_px_mean` / `flow_px_max` | current frame | Measured 17.6 to 57.2% of the 1673 false positives reclassified depending on threshold, with a 0 to 16.9% cost against true positives, weaker alone than 4.3 or 4.4 on this data | Very low, needs the `OpticalFlowResult` plumbing in section 4.5 | Heuristic, physically grounded in parallax, moderate measured correlation (r=0.34) with `v_norm` |
| 2.4 SVM/LSTM adaptive detectors | 6 axis IMU window, 100 to 200 samples | 100 to 200 samples | Not applicable, no labelled training trajectory exists for this platform | High, needs a labelled reference trajectory and a trained model | Learned / data driven |
| 2.5 C-ZUPT | estimated body velocity norm + specific force norm | sliding window K | Architectural precedent for 4.3 and 4.4, not separately measured against this dataset | Medium, needs a persistent sliding window plus an estimated velocity state Basalt already has | Dual threshold with required temporal persistence |

### 4.8 The certified ground interval gate (added 2026-10-25)

The detector proposed here replaces evidence about stillness with a certificate of it. Let `(s_k, L_k)` be the sequence of `/ap/status` messages received by the estimator, with `s_k` the message stamp in the IMU clock and `L_k = !flying_k` the landed state at that stamp. Two consecutive messages that both report landed certify the interval between them,

```
C = ∪_k { [s_k + ε, s_{k+1} − ε]  :  L_k = L_{k+1} = true,  s_{k+1} − s_k ≤ 2 T_hb }
certified(t) = t ∈ C
mpIsStationary(t) = certified(t) ∧ evidence(t)
```

where `ε` bounds the misalignment between frame and status timestamps, `T_hb` is the status heartbeat period, and `evidence(t)` is any subset of the stages of section 4.6, the flow stage by default. A state at time `t` receives the zero velocity residual only once it is certified.

The guarantee follows in three steps. First, ArduCopter keeps `land_complete` true from arming until the motors reach `THROTTLE_UNLIMITED`, and sets it again only after 1 s at minimum throttle (section 2.7), so between two consecutive landed messages the state can have changed only if it changed twice within one evaluation period of AP_DDS, which the 1 s landing trigger rules out. Second, while `land_complete` is true the rotors cannot produce lift, so the vehicle does not move under its own power, which the 87 landed intervals of section 3.5 confirm to within 1.0 cm. Third, the rule looks forward, not backward. A frame is certified only by a landed message stamped after it, and the first message after the take-off release reports `flying`, so no frame after the last landed message before `NOT_LANDED` is ever certified, however late that message arrives. The false stationary rate is therefore zero for every motion the vehicle makes under its own power, with no dependence on transport latency, scene texture or noise. Latency affects only how soon a frame is certified, never whether a moving frame can be.

Four conditions bound the guarantee, and change 8 of `plans/stationary_detection.md` addresses each.

1. External motion. A landed vehicle that is carried, pushed by wind or standing on a moving platform moves without lift. The optional `evidence(t)` term catches such motion when it produces image flow, and the certificate is intended for a vehicle that is still while landed, as in SITL and on a field.
2. Timestamp alignment. Status stamps come from ArduPilot's clock, as do the IMU stamps of `/ap/imu/experimental/data`, while frame times come from the camera. `ε` must exceed their misalignment, which is zero in SITL under `/clock` and must be measured on hardware.
3. Certification delay. A state is certified only when the next landed message arrives, at most one heartbeat later. At the default heartbeat of 2 Hz a state is marginalised before it can be certified, because the window holds `vio_max_states = 3` states, about 0.15 s. Building ArduPilot with `--define AP_DDS_DELAY_STATUS_TOPIC_MS=10`, which the waf option of `Tools/ardupilotwaf/ardupilotwaf.py:712` applies without editing the submodule (`boards.py:259-263`), raises the heartbeat to 20 Hz, so every state is certified about 50 ms after it is created and while it is still in the window. The zero velocity residual is then applied to every certified state in the window, not only to the newest, as the plan's change 8 specifies.
4. Message loss. A lost `flying` message would make two landed messages look consecutive. AP_DDS writes the status on its reliable output stream (`AP_DDS_Client.cpp:1854-1868`), so the subscriber must use a reliable QoS as well, and an interval longer than twice the heartbeat period, 100 ms at 20 Hz, is not certified even when both ends report landed.

Without an autopilot, for example on EuRoC or TUM-VI, the certificate is unavailable, and the OpenVINS-style latch of section 3.5 is the fallback. It is explicit that it gives 0.02 percent on this campaign rather than a guarantee. The latch is implemented since 2026-10-26 as `vio_stationary_enable_start_latch` and `vio_stationary_latch_frames`, applied in `SqrtKeypointVioEstimator::ProcessFrame` directly after the three-stage expression, so `is_stationary` in `vio_stationary.csv` records the latched flag while the stage columns record the evidence (`plans/stationary_detection.md` section 12.10).

| Method | False stationary on the 13 runs | Guarantee | Needs |
|---|---|---|---|
| Section 4.6 three-stage detector, any stage combination | 28 to 58% | None | Nothing new |
| OpenVINS-style latch around the flow stage [14] | 0.02% | None, the first moving frames are judged statistically | Basalt only |
| Latest `/ap/status` value as a gate | Not measured, can admit frames for up to about 100 ms | None, the evaluation period exceeds the 54 ms minimum margin | ROS subscription |
| Certified ground interval gate | 0 by construction | Exact for self-propelled motion, given the three conditions above | ROS subscription, a 20 Hz status heartbeat, multi-state residual |

## 5. Conclusion

The current detector is a legitimate, cheap instance of the classical MAG and ARE family surveyed in section 2.2, and the cross run measurement in section 1, reproduced in detail against `phase_3b_1` in section 3.3 and section 3.4, shows precisely why that family, as currently tuned, performs this poorly, and precisely how much of the poor performance is a tuning defect versus a fundamental limitation. The exhaustive tolerance sweep in section 3.4 is the central finding of this document, roughly half of the measured failure is simply a badly chosen gyroscope tolerance, 0.05 rad/s against a value nearer 0.02 that the same sweep shows performs far better on this exact mission profile, and is fixable today with a one line configuration change and no new code. The remainder, isolated to a small number of sustained, multi second constant velocity segments such as the 49 frame run identified in section 3.4, is not fixable by any amount of further tuning or any more sophisticated purely inertial statistic, because specific force and angular rate are, by the physical argument in section 1 and the GLRT derivation in section 2.1, provably incapable of encoding velocity information that was never present in their inputs. Sections 2.5 and 2.6 both independently point to the same class of fix for that remainder, C-ZUPT's aerial detector and general visual inertial practice both resolve this exact ambiguity by adding a second, independent signal that is zero at rest and generically non zero in cruise, and section 4.3's own measurement shows the cheapest available version of that signal, the optimiser's own `vel_w_i`, removes the sustained residual at a measured cost of only 0.72% disagreement from the one frame lag reading it directly requires. Section 4.4 removes even that lag, folding the same `vel_w_i` into `predictState`'s own kinematic extrapolation alongside section 4.2's preintegrated `dv`, so the resulting Mahalanobis test inherits section 4.3's sensitivity to the fundamental ambiguity, section 4.2's proper chi squared gate, and a current, not lagged, decision every frame, at the cost of one documented approximation, treating the previous keyframe's own velocity as exactly known rather than itself uncertain.

The design actually selected for implementation, detailed in section 4.6, is a two stage detector rather than a single candidate, a growing window SHOE test, section 4.1's own machinery with its minimum sample gate relaxed so the window fills from one sample up to its bound rather than waiting behind a warm up period, combined by logical AND with section 4.4's combined prediction and Mahalanobis test. The growing window answers the takeoff sensitivity the start of this project raised, a short window is statistically weaker but this platform's missions genuinely begin at rest, so a weak but correctly biased signal from the very first frame costs little, while section 4.4 supplies, from that same first frame onward, the one signal every purely inertial candidate in this document lacks by construction. The optical flow check in section 4.5, now wired through to the VIO backend via the two field addition to `OpticalFlowResult` described there, and the excitation covariance reuse this document originally also proposed, since withdrawn as redundant with sections 4.1 and 4.2 once this two stage design was settled on, remain available as further, independent signals should either stage of the chosen design prove insufficient once validated against all eight logged experiments rather than against `phase_3b_1` alone.

Addendum, 2026-10-25. Measured on the multirotor (section 3.5), every stage of the section 4.6 detector declares stationary during flight, on 28 to 58 percent of moving frames, because steady flight is the constant velocity case of section 1 and a distant scene suppresses the flow cue. No threshold test can remove that, for the reason section 2.7 gives. The detector selected for the multirotor is therefore the certified ground interval gate of section 4.8, which takes stillness from ArduCopter's landed state, certifies a frame only between two landed status messages, and so has a false stationary rate of zero by construction for self-propelled motion. The OpenVINS-style latch remains the fallback for datasets without an autopilot.

## References

[1] I. Skog, P. Handel, J.-O. Nilsson, J. Rantakokko, "Zero-Velocity Detection—An Algorithm Evaluation", IEEE Transactions on Biomedical Engineering, vol. 57, no. 11, 2010.

[2] J. Wahlström, I. Skog, "Fifteen Years of Progress at Zero Velocity: A Review", arXiv:2008.09208, IEEE Sensors Journal, 2021. https://arxiv.org/abs/2008.09208

[3] A. R. Jiménez, F. Seco, J. C. Prieto, J. Guevara, "A comparison of Pedestrian Dead-Reckoning algorithms using a low-cost MEMS IMU", WISP 2009.

[4] B. Wagstaff, V. Peretroukhin, J. Kelly, "Robust Data-Driven Zero-Velocity Detection for Foot-Mounted Inertial Navigation", arXiv:1910.00529, IEEE Sensors Journal, vol. 20, no. 2, 2020. https://arxiv.org/abs/1910.00529

[5] B. Wagstaff, J. Kelly, "LSTM-Based Zero-Velocity Detection for Robust Inertial Navigation", arXiv:1807.05275, IPIN 2018. https://arxiv.org/abs/1807.05275

[6] D. Engelsman, I. Klein, "C-ZUPT: Stationarity-Aided Aerial Hovering", arXiv:2507.09344, IEEE Transactions on Aerospace and Electronic Systems, vol. 62, 2026. https://arxiv.org/abs/2507.09344

[7] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", arXiv:1512.02363, IEEE Transactions on Robotics, 2017. https://arxiv.org/abs/1512.02363

[8] T. Qin, P. Li, S. Shen, "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator", arXiv:1708.03852, IEEE Transactions on Robotics, 2018. https://arxiv.org/abs/1708.03852

[9] Y. Yang, P. Geneva, G. Huang, "Multi-Visual-Inertial System: Analysis, Calibration and Estimation", arXiv:2308.05303, International Journal of Robotics Research, 2024. https://arxiv.org/abs/2308.05303

[10] S. M. Kay, "Fundamentals of Statistical Signal Processing, Volume II: Detection Theory", Prentice Hall, 1998.

[11] E. Foxlin, "Pedestrian Tracking with Shoe-Mounted Inertial Sensors", IEEE Computer Graphics and Applications, vol. 25, no. 6, 2005.

[12] P. D. Groves, "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd ed., Artech House, 2013.

[13] Y. Bar-Shalom, X. R. Li, T. Kirubarajan, "Estimation with Applications to Tracking and Navigation", Wiley, 2001.

[14] P. Geneva, K. Eckenhoff, W. Lee, Y. Yang, G. Huang, "OpenVINS: A Research Platform for Visual-Inertial Estimation", IEEE International Conference on Robotics and Automation, 2020, with the zero velocity update in `ov_msckf/src/update/UpdaterZeroVelocity.cpp` and the `zupt_only_at_beginning` latch in `ov_msckf/src/core/VioManager.cpp`. https://github.com/rpng/open_vins and https://docs.ros.org/en/api/ov_core/html/update-zerovelocity.html

[15] PX4 Development Team, "Land Detector Configuration", PX4 User Guide. https://docs.px4.io/main/en/advanced_config/land_detector

[16] ArduPilot Development Team, ArduCopter land detector and take-off logic, `ArduCopter/land_detector.cpp`, `ArduCopter/takeoff.cpp`, and the AP_DDS status publisher `libraries/AP_DDS/AP_DDS_Client.cpp`, ArduPilot source at commit `e257f85be9`.

[17] M. Bloesch, C. Gehring, P. Fankhauser, M. Hutter, M. A. Hoepflinger, R. Siegwart, "State Estimation for Legged Robots on Unstable and Slippery Terrain", IEEE/RSJ International Conference on Intelligent Robots and Systems, 2013. https://research-collection.ethz.ch/bitstream/handle/20.500.11850/75852/eth-7743-01.pdf

[18] M. Camurri, M. Fallon, S. Bazeille, A. Radulescu, V. Barasuol, D. G. Caldwell, C. Semini, "Probabilistic Contact Estimation and Impact Detection for State Estimation of Quadruped Robots", IEEE Robotics and Automation Letters, vol. 2, no. 2, 2017. https://ori.ox.ac.uk/media/5626/2017ral_camurri.pdf

[19] A. Ramanandan, A. Chen, J. A. Farrell, "Inertial Navigation Aiding by Stationary Updates", IEEE Transactions on Intelligent Transportation Systems, vol. 13, no. 1, pp. 235-248, 2012.

[20] J. Wahlström, I. Skog, F. Gustafsson, A. Markham, N. Trigoni, "Zero-Velocity Detection, A Bayesian Approach to Adaptive Thresholding", IEEE Sensors Letters, 2019, arXiv:1903.07929. https://arxiv.org/abs/1903.07929
