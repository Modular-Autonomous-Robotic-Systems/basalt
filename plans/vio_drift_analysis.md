# VIO Drift Analysis and Experimentation Plan

## 1. Introduction

### 1.1 The state of the system

The visual inertial odometry back end in this workspace has stopped diverging and started drifting, and the distinction is the subject of this document. The failure written up in [`vio_divergence_fix.md`](vio_divergence_fix.md), in which the estimated accelerometer bias ran to 3.37 m/s^2 and carried the trajectory to 45 km and 720 m/s, was traced to a false zero velocity seed applied at a moment of powered climb. It was removed by bringing the estimator up on the rising edge of `armed` rather than on the rising edge of `flying`, so that the seed is true when it is read. That change stands, and the two flights analysed here both initialise correctly, with a first accelerometer sample of 9.939 and 9.948 m/s^2 against a gravity magnitude of 9.81.

What remains is a trajectory that closes 90 m away from where it started after 2,480 m of flight, and whose altitude at the cruise plateau reads 209 m where 121 m was commanded. That is a bounded, structured error rather than a runaway, and its cause is now identified.

### 1.2 The test mission

Every number in this document refers to one mission, flown three times. The vehicle arms on the ground, takes off, climbs to a commanded 121 m, flies a spline of roughly 2.5 km with one rotation in place, and lands at the takeoff point. The camera is `front_center`, mounted 0.5 m forward and 0.1 m below the inertial measurement unit and pitched 45 degrees below the horizon, with `fx = fy = 320` on a 640 by 480 image, derived in [`../context/airsim_camera_extrinsics.md`](../context/airsim_camera_extrinsics.md).

Two geometric consequences follow and are used throughout. At the commanded altitude the ground sits at a slant range of `121 / sin(45 deg) = 171.1 m`, and one pixel therefore subtends `171.1 / 320 = 0.535 m` on the ground. Both are an order of magnitude beyond anything in EuRoC or TUM VI, which is where every default in `data/sitl_config_vo.json` was calibrated.

The mission also supplies one accuracy measurement that requires no ground truth subscription and no external reference. The vehicle lands where it took off, so the distance between the first and last estimated position is drift, in metres, directly.

### 1.3 The two flights

`/ws/log2.log` contains three activations. The third raised `local_mapper_max_local_map_size` from 30 to 100 and is excluded by request. The first two differ in exactly one parameter and are the comparison this document is built on.

| | Flight A, run 1 | Flight B, run 2 |
|---|---|---|
| `config.vio_init_bg_weight` | 1e8 | 1e2 |
| Effective anchor on the accelerometer bias | `sqrt(1e8) = 1e4`, implied sigma 1e-4 m/s^2 | `sqrt(1e2) = 10`, implied sigma 0.1 m/s^2 |
| Frames, duration | 1,835 over 106.9 s | 5,607 over 304.5 s |
| Outcome | diverged, 41.5 km and 757 m/s | bounded, 90.2 m closure error over a 2,480 m path |

The parameter reaches the accelerometer bias block rather than the gyroscope block it names, an upstream transposition confirmed against `PoseVelBiasState::applyInc` and recorded in [`../context/linearisation.md`](../context/linearisation.md). Flight A is therefore the run in which the accelerometer bias was held rigidly at zero, and Flight B the run in which it was left free.

### 1.4 What the evidence shows

Four findings carry the analysis, and each is established from the logs in section 3 before being turned into a remedy in section 5.

The map is created from noise before the vehicle ever moves. Through the whole ground phase the estimator triangulates nothing, because the baseline between frames is zero and every candidate is rejected. Then, at 1.25 s in Flight A and 1.42 s in Flight B, the estimator's own accumulated position noise first exceeds the 0.05 m triangulation threshold, and it immediately creates 89 and 25 landmarks at mean depths of 117.1 m and 63.3 m. The true depth at that moment is under one metre. The first map in the window is wrong by two orders of magnitude, and it is the map against which takeoff is estimated.

Takeoff exceeds the capture range of the tracker. Interframe flow rises from 0.25 px on the ground to 21.4 px within four frames, against a pyramidal Lucas Kanade capture range of `3.5 * 2^3 = 28` px. Track survival collapses to 0.34 in Flight B and to 0.10 in Flight A, and the landmark database empties completely. The map is rebuilt from scratch, at depths of one to five metres, at the precise moment the vehicle is undergoing the only strong acceleration of the mission.

The estimator settles on a confounded solution and stays there. Section 2.2.3 shows that while the vehicle is not accelerating, a body tilt and a horizontal accelerometer bias produce an identical measurement, so the two are indistinguishable. Flight B resolves that ambiguity onto an accelerometer bias of 2.3 m/s^2 on the body x axis, held from 6 s to 78 s, which is a tilt error of 13.8 degrees. It is corrected only at 78 s, the moment the vehicle first accelerates horizontally into the spline. By then the climb has been over integrated by 78 percent, and marginalisation has frozen that error into the prior where no later evidence can reach it.

Denying the bias makes matters worse, not better. Flight A anchored the accelerometer bias at an implied 1e-4 m/s^2 and succeeded, holding it below 0.233 m/s^2 for the whole run. The confounded error did not disappear, it moved. The gyroscope bias, anchored only by `vio_init_ba_weight` at an implied 0.316 rad/s and therefore effectively free, rose to 0.048 rad/s and rotated the world frame continuously, which leaks gravity into the horizontal channel and double integrates into a 41.5 km divergence. The vision cost rose by a factor of 45, from a median of 292 to a median of 13,003, which is the optimiser recording that it is being forced against the images.

### 1.5 Investigation directions

The remedies that follow from those findings, in the order the evidence supports them, are these.

Suppress map creation until parallax is real, and express the threshold as a parallax rather than a distance in metres. Sections 2.4 and 5, Phase 2.

Treat the stationary interval explicitly, by averaging the gravity reference and by asserting the velocity that is known to be zero. Sections 2.2.3 and 5, Phase 3.

Extend the tracker's capture range so that takeoff does not destroy the map. Sections 2.5 and 5, Phase 4.

Correct the inertial characterisation, three of whose four stochastic parameters are wrong and all in the direction that weakens the only scale bearing term in the objective. Sections 2.3 and 5, Phase 1.

Lengthen the observability window, so that a bias error acquired early has more chance of being revised before it is marginalised. Sections 2.2.5, 2.6 and 5, Phase 5.

Measure accuracy rather than self consistency, which the stack currently cannot do at all. Section 5, Phase 0.

### 1.6 How to read this document

The mathematics in this repository is already written down, and this document does not repeat it. The division is as follows and is worth learning, because it is the difference between a five minute answer and a five hour one.

`doc/` holds the general theory of the estimator, written as a textbook. `doc/VIO.md` carries the residuals, the preintegration, the keyframe policy, triangulation and the complete objective. `doc/Linearisation.md` carries the numerical mathematics, from the manifold least squares problem through QR, the Schur complement, gauge freedom and first estimate Jacobians. `doc/Marginalisation.md` carries the marginalisation residual and its derivation.

`context/` holds what has been measured or discovered about this particular stack, including its defects. It is the place where a claim is anchored to a `file:line` and a date.

Section 2 of this document holds only what neither of those covers, namely the mathematics that becomes relevant when the scene is 171 m away and the vehicle spends four seconds standing still, together with the numerical values for this mission. Each subsection opens by naming where the general treatment lives and then adds the specific result.

## 2. Mathematical Background

### 2.1 The estimator as a factor graph, and what each term can say

Basalt solves a sliding window maximum a posteriori problem. The variables are a set of body poses, and for the most recent few frames also velocities and inertial biases,

```
x_k = ( T_w_i,k in SE(3),   v_w_i,k in R^3,   b_g,k in R^3,   b_a,k in R^3 )
```

and the objective is a sum of four terms,

```
E(x) = E_vision(x) + E_imu(x) + E_bias(x) + E_prior(x)
```

The complete statement is `doc/VIO.md` §3.1, which gives each residual, its weight and its Jacobians, with the preintegration in §4 and the marginalisation term in §8. The Jacobian blocks as Basalt actually computes them, including the two sign and handedness traps in the rotation row, are recorded in [`../context/vio_residuals_and_priors.md`](../context/vio_residuals_and_priors.md). None of that is reproduced here.

What matters for everything below is a single property of each term, because almost every question in this document reduces to asking which term is carrying a given quantity.

The vision term is a function of bearings alone. Scaling every pose translation and every landmark distance by a common factor leaves every reprojection residual unchanged, which `doc/VIO.md` §2.4.1 derives as the seven parameter similarity gauge freedom of monocular structure from motion. Vision cannot observe metric scale, and it cannot distinguish a small rotation of the body from the corresponding change in where the landmarks are.

The inertial term is the only place in the objective where a metre appears as a metre. Its position and velocity rows compare a double integral of measured specific force against a difference of estimated positions, so it is the sole source of scale, and it is also the sole source of the gravity direction that makes roll and pitch observable. `doc/VIO.md` §3.1 notes the consequence, that adding the inertial term shrinks the nullspace from seven dimensions to four.

The bias term is a random walk between consecutive states, assembled in `ImuBlock::linearizeImu` at `include/basalt/linearization/imu_block.hpp:73-91` with weight `sigma_b^{-1} / sqrt(dt)`. It constrains how fast a bias may change and says nothing about its level. A common shift applied to every bias state in the window costs exactly zero, which section 2.2.5 shows is decisive.

The prior term carries everything the window has already seen and discarded. Its derivation is `doc/Marginalisation.md` Appendix A and its consistency discipline is section 2.6 below.

### 2.2 Observability

This is the central section of the document, and the one whose content is genuinely absent from `doc/`. The reason is that `doc/Linearisation.md` §2.15 answers a different question, and the difference is easy to miss.

#### 2.2.1 Permanent gauge freedom, which is already documented

`doc/Linearisation.md` §2.15 establishes that visual inertial odometry has exactly four unobservable directions, namely the three components of global position and the rotation about the gravity vector. These are properties of the measurement model that hold for every trajectory and every scene. They are handled by construction, by anchoring precisely those four directions in the initial prior at `src/vi_estimator/sqrt_keypoint_vio.cpp:89-101`, indices 0 to 2 and index 5, and by the first estimate Jacobian discipline that keeps the nullspace aligned thereafter.

#### 2.2.2 Conditional degeneracy, which is not

A direction can also be unobservable because of what the vehicle happens to be doing. Such directions are not in the nullspace of the model, they are in the nullspace of the Jacobian evaluated along this particular trajectory, and they appear and disappear as the motion changes. They are invisible to the four direction analysis, they are not anchored by any prior, and the estimator has no mechanism that notices them.

Three matter here, and all three occur in this mission.

Under zero linear acceleration the body tilt and the horizontal accelerometer bias are confounded, treated in 2.2.3.

Under zero translation the depth of every landmark is unobservable, treated in 2.4.

Under constant velocity the metric scale receives no new information, treated in 2.2.4.

`doc/VIO.md` §2.4.2 treats the pure rotation case for the visual front end and correctly observes that the gyroscope keeps rotation observable when parallax vanishes. What it does not say, and what this mission demonstrates, is that the inertial measurement has degeneracies of its own, that they are the mirror image of the visual ones, and that the two do not overlap in the way the design intends.

#### 2.2.3 The tilt and accelerometer bias confound

While the vehicle is not accelerating, the accelerometer measures only the reaction to gravity, rotated into the body frame, plus its bias,

```
a_meas = R_wb^T ( - g ) + b_a + n_a
```

with `g = (0, 0, -9.81)` in the Z up world frame of `include/basalt/utils/imu_types.h:62`. Perturb the body attitude by a small rotation `delta_theta` about a horizontal axis and perturb the bias at the same time. To first order

```
a_meas  ->  ( I - [delta_theta]_x ) R_wb^T ( - g ) + b_a + delta_b
```

and the measurement is unchanged for every pair satisfying

```
delta_b = [delta_theta]_x R_wb^T ( - g )
```

which for a level vehicle is

```
delta_b_horizontal = g * delta_theta
```

a two dimensional family along which the inertial measurement is exactly constant. One metre per second squared of horizontal bias is worth 5.85 degrees of tilt, and the two cannot be separated by the accelerometer at any averaging time whatever.

In a healthy system the camera breaks the tie, because a tilt rotates every bearing while a bias does not. Two conditions are needed for it to do so, and the mission violates both in succession. There must be landmarks, which requires parallax, which requires translation. And there must be linear acceleration, because only then does the bias enter the position and velocity rows in a way the tilt does not.

The measurement in section 3.4 is exactly this. Flight B sits at `b_a = (2.30, -0.60, -0.13)` from 6 s to 78 s, a horizontal magnitude of 2.377 against a specific force magnitude of 9.96, so the attitude is in error by

```
asin( 2.377 / 9.96 ) = 13.8 degrees
```

and the pair is self consistent, which is why the trajectory does not immediately explode. The cost of being on that manifold is that every real acceleration is rotated by 13.8 degrees before it is integrated, so an acceleration of magnitude `a` acquires a transverse error of `a sin(13.8 deg) = 0.239 a`.

#### 2.2.4 Scale, and the excitation that reveals it

Scale enters through the position row of the inertial residual,

```
r_p = R_0^T ( p_1 - p_0 - v_0 dt - 0.5 g dt^2 ) - Delta_p
```

where `Delta_p` is the double integral of the bias corrected specific force, given in full at `doc/VIO.md` §4.1. The measured quantity is an acceleration and the position is its second integral, so the information the inertial term carries about scale grows with the magnitude of the acceleration excursions and with the square of the interval over which they are observed. Martinelli (2014) gives the closed form solution and shows that scale, gravity, initial velocity and the biases are all determined by a finite set of measurements provided the motion is sufficiently exciting. The word sufficiently carries the content.

Three motions carry no scale information at all. Constant velocity in a straight line, where the specific force is exactly the gravity reaction and is indistinguishable from standing still. Constant acceleration, where the accelerometer reading is constant and therefore linearly dependent with a bias. And pure rotation, which carries no translation to scale.

The mission spends its first 78 seconds in climb and hover, which is close to the first case, and it is exactly at 78 s, when the vehicle accelerates horizontally into the spline and the speed rises from 8.5 to 16.9 m/s, that the accelerometer bias in Flight B collapses from 2.3 to 0.5 m/s^2. Figure `figures/runall_state.png` shows the collapse and figure `figures/run2_trajectory.png` shows the manoeuvre that caused it. This is the Martinelli result made visible in a log file, and it is the single most useful observation in the whole investigation, because it says the estimator was never wrong about what it could see, it was starved of the excitation it needed.

#### 2.2.5 Accelerometer bias observability against scene depth

Vision can contradict an accelerometer bias only through the position error the bias produces. Over a window of duration `T` a constant bias `b` displaces the predicted position by `0.5 b T^2`, and a point at depth `d` observed by a camera of focal length `f` moves in the image by

```
delta_px = f b T^2 / ( 2 d )
```

Setting that equal to the measurement standard deviation and inverting gives the window length at which a bias first becomes visible,

```
T_detect = sqrt( 2 sigma_px d / ( f b ) )
```

With `f = 320`, `sigma_px = vio_obs_std_dev = 0.5` and the mission slant range `d = 171.1 m`, the numerator `2 sigma_px d` is 171.1 and `T_detect = sqrt( 0.5347 / b )` seconds. The active window is set by `vio_max_states = 3`, which keeps three states carrying velocity and bias and therefore two inertial factors, which at the measured frame interval of 51 to 54 ms is 0.102 to 0.108 s.

| Bias `b` [m/s^2] | `T_detect` [s] | `vio_max_states` needed at 19.6 Hz | Displacement in the 0.102 s window [px] |
|---|---|---|---|
| 3.0 | 0.42 | 10 | 0.029 |
| 2.3, the Flight B value | 0.48 | 11 | 0.022 |
| 1.0 | 0.73 | 16 | 0.0097 |
| 0.1 | 2.31 | 47 | 0.00097 |
| 0.01 | 7.31 | 145 | 0.000097 |

The state count in the third column is `T_detect * 19.6 + 1`, because `N` states carry `N - 1` inertial factors.

The last column is the point. The 2.3 m/s^2 bias that Flight B carried for seventy seconds is worth two hundredths of a pixel inside the window the estimator actually optimises. No amount of care in the visual front end can detect it, and no plausible value of `vio_max_states` brings it within reach. At this altitude the accelerometer bias is observable through acceleration, by the argument of 2.2.4, and effectively not at all through the images.

The bias random walk does not fill the gap, because it constrains the rate and not the level. The residual is proportional to `b_k - b_{k-1}`, so a shift common to every bias state in the window costs nothing. Only the marginalisation prior resists a common shift, and its weight on the accelerometer bias block is `sqrt(vio_init_bg_weight)`, which is the parameter that distinguishes the two flights.

#### 2.2.6 Why the two biases are not symmetric, and why Flight A failed

The same calculation applied to the gyroscope gives a very different answer. A gyroscope bias rotates the predicted bearing directly, producing

```
delta_px = f b_g T
```

pixels after a time `T`, with no dependence on scene depth whatever. With `f = 320` and `T = 0.102` s a bias of 0.015 rad/s already produces half a pixel, so the gyroscope bias is well observed from the images at any altitude, and indeed Flight B holds it between 0.005 and 0.008 rad/s throughout.

The asymmetry is the whole explanation of Flight A. Clamping the accelerometer bias does not remove the confounded error of 2.2.3, because that error is a property of the motion and not of the parameterisation. It removes one of the two variables along which the error can be expressed, so the error is forced entirely into the attitude. Attitude, unlike an accelerometer bias, is not free, because an attitude that is wrong and static still leaves the gravity reaction uncancelled. The optimiser's only way to hold a persistently rotated world frame is to keep rotating it, and it pays for that with a gyroscope bias, which at an implied anchor of `sqrt(vio_init_ba_weight) = 3.16`, that is 0.316 rad/s, costs almost nothing. Flight A drives it to 0.048 rad/s, which is 2.75 degrees per second.

The consequence is quantitatively the difference between the two flights. A world frame rotating at rate `omega` leaves a residual horizontal acceleration that grows as `g sin(omega t)`, so the position error is not quadratic but quartic in the early phase and settles into a constant acceleration once the tilt saturates. Flight A's speed grows from 1.7 m/s at 3.5 s to 92.9 m/s at 24.5 s, a mean of 4.34 m/s^2, and reaches 757 m/s at the end, a mean over the whole run of 7.08 m/s^2 which is `9.81 sin(46.2 deg)`. The rate rises as the tilt accumulates, which is what a rotating world frame produces. The estimated position is 41.5 km and the estimated map follows it to a median triangulated depth of 83 km.

The lesson generalises beyond this parameter. A confounded pair must be broken by adding information, never by removing a degree of freedom, because removing one member of the pair does not remove the error, it relocates it to whichever remaining direction is cheapest. Choosing which direction that is, without knowing it, is how a stiff prior turns a bounded error into a divergent one.

### 2.3 Inertial measurement modelling and the calibration it drives

The conversions from the Project AirSim robot configuration to the Basalt calibration fields, the physical meaning of angle random walk and velocity random walk, the Allan deviation reading of each, the pure Wiener bias process, and the three transformations that ArduPilot interposes between the simulated sensor and the topic Basalt subscribes to, are all derived in [`../context/imu_noise_parameters.md`](../context/imu_noise_parameters.md). The preintegration that consumes them is `doc/VIO.md` §4. Neither is repeated. What follows is how each number enters the optimisation and what an error in it costs.

The measurement model is the standard one,

```
a_meas    = a     + b_a + n_a,     n_a ~ N(0, sigma_a^2 I) per sample
om_meas   = omega + b_g + n_g,     n_g ~ N(0, sigma_g^2 I) per sample
b_a(t+dt) = b_a(t) + w_a,          w_a ~ N(0, sigma_ba^2 dt I)
b_g(t+dt) = b_g(t) + w_g,          w_g ~ N(0, sigma_bg^2 dt I)
```

The calibration declares continuous time densities and Basalt converts the first two to per sample standard deviations by multiplying by `sqrt(imu_update_rate)` at `thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp:147-161`. That conversion is worth understanding, because it means a change in sample rate silently rescales the inertial weight. The variance of the integral of white noise of density `sigma_c` over an interval `T` is `sigma_c^2 T` regardless of how finely the interval is sampled, and representing the same process as `N` samples of standard deviation `sigma_d` reproduces that only when `sigma_d = sigma_c sqrt(f)`.

The preintegration covariance propagates those per sample variances and the resulting `sqrt_cov_inv` whitens the nine inertial rows. The weight the inertial term carries is therefore inversely proportional to `sigma_a`, and since 2.1 established that the inertial term is the only scale bearing term in the objective, an overstated `accel_noise_std` divides the influence of the scale constraint while leaving the scale invariant vision term untouched.

The bias random walk weight recurs in 2.2.5 and deserves a number. Per `imu_block.hpp:73-91` the residual between consecutive bias states carries weight `1 / ( sigma_b sqrt(dt) )`, which with the declared `accel_bias_std` of 2.83e-4 and a frame interval of 0.051 s is 15,647 per unit of bias change, and with the derived value of 2.0018e-4 would be 22,121. A step of one hundredth of a metre per second squared between consecutive states costs of order 1.2e4, which is why the bias never steps inside the window and always moves as a block.

The discrepancies, reproduced so this document stands alone, are these.

| Field | In `data/sitl_calib.json` | Derived from the robot configuration | Ratio | Effect of the error |
|---|---|---|---|---|
| `gyro_noise_std` | 5.818e-4 | 5.8178e-4 | 1.0000 | none |
| `accel_noise_std` | 2.000e-2 | 1.1768e-2 | 1.6995 | the only scale bearing term is under weighted by 1.70 |
| `gyro_bias_std` | 7.92e-6 | 5.5981e-6 with `SIM_DRIFT_SPEED` zeroed, else 1.5586e-5 | 1.41 or 0.51 | the gyroscope bias may wander faster than the sensor does, which tilts the estimated attitude |
| `accel_bias_std` | 2.83e-4 | 2.0018e-4 | 1.4137 | the accelerometer bias may wander 1.41 times faster, absorbing more real acceleration |
| `imu_update_rate` | 167 | 167 | 1.0000 | none, and confirmed at 167.08 to 167.93 Hz in both flights |

One term belongs here that no configuration file mentions. `EarthUtils::GetGravity` returns 9.8048 m/s^2 at the 583 m home altitude of the scene while Basalt hard codes 9.81, so there is a constant 5.2e-3 m/s^2 vertical specific force error. The accelerometer bias state absorbs it without difficulty when free, which is correct, and it is one more reason that clamping that state is not automatically an improvement.

### 2.4 Triangulation and the geometry of depth

The direct linear transform Basalt uses, the stereographic landmark parameterisation, the baseline gate and the inverse depth acceptance gate are all set out in `doc/VIO.md` §7.1, with the implementation walk through in §7.2. The measured behaviour of that code on this stack, including the retention policy of `prev_opt_flow_res` and the fact that the candidate loop tries the oldest and therefore widest baseline first, is in [`../context/vio_optical_flow_instrumentation.md`](../context/vio_optical_flow_instrumentation.md). This section adds the uncertainty propagation, which neither carries, and the numerical consequence for a scene at 171 m.

#### 2.4.1 Parallax and the propagation of depth uncertainty

The geometric quantity that governs everything is the parallax angle, the angle subtended at the landmark by the two camera centres, which for a baseline `b` at depth `d` near the optical axis is `alpha = b / d` radians and `alpha_px = f b / d` pixels. Differentiating the triangulation relation `d = f b / disparity` with respect to the disparity and substituting the measurement noise gives

```
sigma_d / d = sigma_px * d / ( f * b )
```

This is the single most useful expression in the document. Relative depth error grows linearly with depth and falls inversely with baseline, so a scene ten times further away needs ten times the baseline for the same quality of map. For this mission, `sigma_px d / f = 0.2674 m`, so `sigma_d / d = 0.2674 / b`.

| Baseline `b` [m] | Parallax [px] | `sigma_d / d` | Reading |
|---|---|---|---|
| 0.05, the configured floor | 0.094 | 5.35 | depth undetermined, error 535 percent |
| 0.535 | 1.0 | 0.50 | one pixel of parallax, still useless |
| 2.01, one keyframe interval at 12.9 m/s | 3.8 | 0.133 | marginal |
| 2.67 | 5.0 | 0.100 | ten percent depth error |
| 5.35 | 10.0 | 0.050 | five percent depth error |
| 30.2, the Flight B median | 56.5 | 0.0089 | good |

Two conclusions follow. The configured `vio_min_triangulation_dist` of 0.05 m admits landmarks whose depth is not determined by the data at all. It is an EuRoC value, correct for a scene two to ten metres away where five centimetres of baseline yields half a degree to two and a half degrees of parallax, and at 171 m it yields one arc minute. And a fixed threshold in metres is the wrong shape for the problem, because the quantity that must be bounded depends on `b`, `d` and `f` jointly, whereas a threshold expressed as a parallax bounds exactly that ratio and transfers between scenes without retuning.

#### 2.4.2 The degenerate limit, and why the acceptance window does not catch it

As the baseline shrinks, the two pairs of rows of the direct linear transform become linearly dependent, the two smallest singular values collapse together, and the returned singular vector is determined by noise rather than by geometry. What it returns in that limit is a vector with a very small fourth component, because a point at infinity is the exact solution of the degenerate problem. The algebraic residual the decomposition minimises is also biased in exactly this regime, as Hartley and Sturm (1997) show.

The acceptance test at `sqrt_keypoint_vio.cpp:681-689` requires the inverse distance to be finite, strictly positive and strictly less than 3.0, which is a depth window of `(0.333 m, infinity)`. It is closed below and open above. A landmark returned at ten thousand metres from a degenerate configuration passes every test.

#### 2.4.3 The phantom map, derived and then observed

Combining the two gives a prediction that section 3.5 confirms exactly. While the vehicle is stationary the true baseline is zero, so the observed interframe motion is tracker noise, measured at 0.22 to 0.30 px in both flights. The estimator's own position estimate nonetheless drifts, because nothing constrains it, and once that drift exceeds `vio_min_triangulation_dist` the estimator believes it has a baseline. It then scales a noise disparity by a noise baseline,

```
d_phantom = f * b_believed / disparity_noise
```

which for `b = 0.055 m` and a disparity of 0.28 px predicts `320 * 0.055 / 0.28 = 62.9 m`. The measurement is 63.3 m in Flight B and 117.1 m in Flight A, against a true depth below one metre.

The expression is worth keeping because it says what to do. The phantom depth is proportional to the believed baseline and inversely proportional to the tracking noise, so no tightening of the tracker removes it, and only refusing to triangulate until the parallax is genuinely above the noise does.

### 2.5 The visual front end and its capture range

The front end is a pyramidal Lucas Kanade tracker in the inverse compositional formulation, described at `doc/VIO.md` §2.1, with the detection occupancy grid and its conditioning rationale in the same section. `doc/VIO.md` §2.4.3 states the capture range as approximately `2^(L-1) r` pixels for `L` levels and patch radius `r` and correctly identifies fast motion as a cascading failure mode. This section supplies the numbers for this configuration, which turn out to decide the takeoff.

`trackPoint` at `include/basalt/optical_flow/frame_to_frame_optical_flow.h:369` loops from `level = config.optical_flow_levels` down to zero, so `optical_flow_levels = 3` builds four levels and the coarsest has scale `2^3 = 8`. The patch is `Pattern51`, defined at `include/basalt/optical_flow/patterns.h:147-155` as one half of `Pattern52`, whose raw extent is plus or minus 7, so the half extent is 3.5 px at whatever level it is applied. The capture range at level zero is therefore

```
r * 2^levels = 3.5 * 8 = 28 px per frame
```

Section 3.6 measures the degradation and finds the knee exactly there. Survival holds near 0.84 up to 8 px of mean flow, falls to 0.68 between 16 and 20 px, and collapses to 0.38 above 20 px, at which point the maximum flow in those frames is 38 to 41 px and therefore beyond the range outright.

The cruise value is far inside it. For a translating camera viewing a plane at depth `d`,

```
flow [px per frame] = f v dt / d
```

which at `f = 320`, `v = 12.9 m/s`, `dt = 0.051 s` and `d = 171.1 m` gives 1.23 px per frame, and the measured median through Flight B is 1.91 px. Takeoff is the only part of the mission that approaches the limit, because the depth in the denominator falls to one metre while the speed is already several metres per second.

That same expression is the cheapest diagnostic in the system, because it ties the images to physical units independently of anything the estimator believes. It cannot, however, detect a scale error, because a global rescaling changes `v` and `d` together and leaves the ratio unchanged. That is the gauge freedom of 2.1 reappearing, and it is why section 5 Phase 0 exists.

The detection grid deserves one correction that the measurements forced. `detectKeypoints` at `src/utils/keypoints.cpp:161-233` adds at most one corner per empty cell, and with `optical_flow_detection_grid_size = 40` on a 640 by 480 image the loop covers sixteen columns and twelve rows, so at most 192 corners are added per frame. That is a bound on new detections, not on the population, because several surviving tracks may occupy one cell. The flights confirm both, with `of_detected` never exceeding 191 and `of_out` reaching 368.

### 2.6 Keyframes, window length, marginalisation and first estimate Jacobians

The keyframe policy is `doc/VIO.md` §6, the marginalisation mathematics is `doc/Marginalisation.md` §3 with the residual derived in Appendix A, and the first estimate Jacobian discipline is `doc/Linearisation.md` §2.16 and `doc/VIO.md` §3.1.3. This section draws the one consequence that governs the drift.

Marginalisation is exact for the linearised problem and is computed once, at the state the variables held when they were discarded, and is never re linearised. First estimate Jacobians make that consistent, at the price that an error present at the moment of marginalisation is preserved in the prior and cannot be revised by later evidence.

Applied to this mission the consequence is severe and is visible directly in the altitude profile of figure `figures/run2_trajectory.png`. The accelerometer bias error of 2.3 m/s^2 is acquired at takeoff, held through the entire climb, and corrected only at 78 s. Every state marginalised during those seventy two seconds carries the resulting position error into the prior. The climb is over integrated, reaching a plateau of 209 m where 121 m was commanded, a factor of 1.73. The descent, flown after the bias had been identified, covers 136.6 m of estimated altitude for a true 121 m, a factor of 1.13. The difference, 78.6 m, is exactly the altitude the estimator still believes it has when the vehicle is on the ground.

That decomposition is the clearest statement of the drift mechanism available. The scale error is not a constant property of the system, it is a transient acquired during one manoeuvre and then frozen by marginalisation.

The window sizes are `vio_max_states = 3` and `vio_max_kfs = 7`. The first bounds the active inertial chain at 0.102 s and is the window of 2.2.5. The second bounds the geometric extent available to triangulation. The keyframe rate is not set by the ratio test at all. The test at `sqrt_keypoint_vio.cpp:566-570` is a strict inequality on a counter reset at each keyframe, so `vio_min_frames_after_kf = 1` permits a keyframe every third frame, and both flights sit exactly at that floor, with 1,835 frames over 612 keyframes and 5,607 over 1,819, that is 3.00 and 3.08. The connected ratio medians of 0.563 and 0.741 are both below the 0.8 threshold, so the ratio test asks for a keyframe on almost every frame and the floor is what answers.

A high keyframe rate is not free. It shortens the temporal span of the seven keyframe window and therefore the baseline available for triangulation, which is the opposite of what a distant scene needs.

### 2.7 Robust cost and outlier handling

The general treatment of robust cost functions and iteratively reweighted least squares is `doc/Linearisation.md` §2.5. The Huber weight is computed in `compute_error_weight` at `include/basalt/linearization/landmark_block_abs_dynamic.hpp:474-490`, and the implementation detail that matters is the order of operations at `:167-174`. The residual returned by `linearizePoint` is in pixels, because `cam.project` applies the intrinsics and the measured keypoint is subtracted in pixel coordinates. The Huber weight is computed from that pixel residual and only afterwards is the row whitened by `sqrt(w) / obs_std_dev`. Therefore `vio_obs_huber_thresh` is expressed in pixels and is independent of `vio_obs_std_dev`, and at the configured 1.0 against 0.5 the transition occurs at two standard deviations, which is conventional.

What a robust loss cannot do is remove an observation. Deletion is the job of `filterOutliers`, which exists on `BundleAdjustmentBase` at `src/vi_estimator/ba_base.cpp:270-309` and is called by the mapping layer but never by the odometry back end, where the call site is a standing TODO at `sqrt_keypoint_vio.cpp:1758`. The two configuration keys that would govern it are commented out of `struct VioConfig` and are inert in every file, as recorded in [`../context/vio_config_json.md`](../context/vio_config_json.md).

The phantom landmarks of 2.4.3 are the reason this matters less than it appears. They are not outliers in reprojection. They fit their observations well and simply carry no depth information, so a residual based filter would not remove them. Outlier rejection is worth restoring for robustness against genuine mismatches, and it is not the remedy for the drift.

### 2.8 Beyond the aerial case

The mission analysed here is a high altitude flight, but the same mathematics determines what will happen on a ground vehicle or a legged platform, and the system should be designed against the general statement. Four results carry over.

Depth sets everything. Every expression in 2.2.5 and 2.4.1 carries `d` in the numerator. A ground vehicle looking at a scene five metres away detects a 0.1 m/s^2 bias in `sqrt(2 * 0.5 * 5 / (320 * 0.1)) = 0.56` s against 2.31 s at altitude, and needs `0.2674 * 5 / 171.1 = 0.0078` m of baseline for a ten percent depth error. The same code that is marginal at altitude is comfortable indoors, which is why the defaults work on EuRoC. A configuration expressed in dimensionless ratios transfers. A configuration expressed in metres does not.

Conditional degeneracy is the general problem, and it takes a different form on every platform. A wheeled vehicle on flat ground has no vertical excitation, and Wu et al. (2017) show it loses observability of the accelerometer bias along the direction of travel and of part of the camera to body extrinsic, which is the same mechanism as 2.2.3 with a different null direction. A legged robot in a stance phase is stationary in exactly the sense of 2.2.3. The remedy in every case is to add an independent measurement, which for a wheeled vehicle is the wheel odometer, for a legged platform the contact constraint of Bloesch et al. (2012) and Hartley et al. (2020), and for the stationary case the zero velocity constraint of Phase 3. All three enter the estimator identically, as a velocity residual on a state, which is why Phase 3 is worth building carefully even though its immediate benefit is confined to the ground interval.

Constant velocity carries no scale, on a motorway as much as in cruise. Scale must be established during genuine acceleration and then protected, and protecting it is the marginalisation question of 2.6.

Degeneracy should be detected rather than assumed absent. Zhang et al. (2016) give a practical test, namely to examine the eigenvalues of the information matrix of the linearised system and to suppress the update along directions whose eigenvalue falls below a threshold. The square root formulation makes the factor available without extra work, and the result is a system that can report that scale is currently unobservable instead of asserting a wrong value with confidence. Flight A is precisely a case where that report would have been more useful than the answer.

## 3. Experiment Analysis

### 3.1 Tooling

Two scripts reduce an instrumented capture to tables and figures, and both are kept in `scripts/` so the reduction is reproducible rather than reconstructed by hand.

`scripts/vio_log_extract.py` splits a capture into runs at each `Setting up filter` line, joins every tagged record sharing a frame timestamp into one row, and writes one wide CSV per run together with the inertial ingestion table and a text summary of the distribution of every quantity of interest. `scripts/vio_log_plot.py` reads those tables and writes eight figures per run, and with `--compare` overlays every run on one set of axes, which is how a configuration change is judged.

```bash
pip3 install --break-system-packages matplotlib      # absent from the dev container by default
python3 scripts/vio_log_extract.py /ws/log2.log -o /tmp/vio2 --runs 1,2
python3 scripts/vio_log_plot.py /tmp/vio2 -o plans/figures
python3 scripts/vio_log_plot.py /tmp/vio2 -o plans/figures --compare
```

The full distribution report for both flights is kept at `figures/summary.txt`, so every number quoted below is traceable to the reduction that produced it.

Four parsing behaviours were forced by the data rather than chosen, and should not be removed.

Fields are delimited by the position of the next key rather than by whitespace, because Eigen prints a vector as space separated components. A vector arriving with fewer than three components is dropped rather than written into the scalar column of the same name, and the `|p|`, `|v|` and `|ba|` norms are renamed `p_norm`, `v_norm` and `ba_norm` so they cannot collide with the vectors they summarise.

A bracketed group such as `baseline_m[min=.. max=..]` names its members by the group, so that the `min`, `mean` and `max` of the two groups in the `[VIO] triang` record do not overwrite one another.

`std::cout` is not synchronised across the estimator, mapper and front end threads, so a foreign record can land inside an estimator record. An intrusion is detected as a bracketed tag or any of `:`, `(` and `,` appearing after the record has begun writing fields, the record is truncated there and the field the truncation landed in is discarded. The script additionally knows the final field each record type writes, so a record cut short without leaving any marker is also caught. Across the two flights this rejects 656 and 1,975 records of 780,000 lines, and without it the reduction reports impossible values such as a 52 second optimisation latency.

A corrupted record can carry a timestamp with foreign digits spliced into it. The frame marker record is short and rarely corrupted, so it arbitrates, and a record whose timestamp differs from the current marker by more than `--max-frame-gap-s` is rejected.

The eight figure groups are `trajectory`, `capture`, `state`, `frontend`, `triangulation`, `cost`, `inertial` and `window`, and each answers a different question. The trajectory figure shows the mission, the altitude against the commanded altitude, the path length and the distance from the takeoff point, which is the closure error. The capture figure shows track survival against interframe flow with the tracker capture range marked. The remaining six show the state and biases, the front end, the triangulation yield and depth, the four cost terms separately, the inertial ingestion and the window occupancy and latency.

### 3.2 The two flights at a glance

| Quantity | Flight A, `vio_init_bg_weight = 1e8` | Flight B, `vio_init_bg_weight = 1e2` |
|---|---|---|
| Frames, duration | 1,835 over 106.9 s | 5,607 over 304.5 s |
| Frame interval, p50 / p90 / max | 54 / 54 / 3,159 ms, 18.52 Hz | 51 / 54 / 267 ms, 19.61 Hz |
| Keyframes, frames per keyframe | 612, 3.00 | 1,819, 3.08 |
| Keyframe interval, p50 / p90 | 159 / 210 ms | 156 / 207 ms |
| Initial accelerometer sample | 9.93888 m/s^2 | 9.94766 m/s^2 |
| Track survival, p50 | 0.902 | 0.888 |
| Tracks handed to the estimator, p50 | 269 | 277 |
| Landmarks in the database, p50 | 158 | 216 |
| Connected track ratio, p50 | 0.563 | 0.741 |
| Landmarks triangulated per keyframe, p50 | 47 | 56 |
| Cheirality rejections per keyframe, p50 / max | 214 / 786 | 1 / 489 |
| Largest baseline used, p50 | 944 m | 30.2 m |
| Mean triangulated depth, p50 | 83,341 m | 197.7 m |
| Accelerometer bias norm, p50 / max | 0.053 / 0.233 m/s^2 | 0.155 / 2.502 m/s^2 |
| Gyroscope bias, peak component | -0.048 rad/s | 0.007 rad/s |
| Position norm, p50 / max | 7,970 / 41,484 m | 222 / 471 m |
| Speed, p50 / p90 / max | 451 / 744 / 757 m/s | 6.09 / 17.5 / 21.8 m/s |
| Vision cost, p50 | 13,003 | 292 |
| Inertial cost, p50 | 55.0 | 11.7 |
| Marginalisation prior cost, p50 | -7,958 | -16,802 |
| Optimise and marginalise, p50 / max | 9.02 / 21.1 ms | 11.79 / 111.2 ms |
| Front end total, p50 / max | 1.29 / 3.77 ms | 1.22 / 3.90 ms |
| Local mapper corrections rejected | 2,232, median error 12.39 m | 9,498, median error 0.387 m |

The inertial path is sound in both. Samples integrated per frame have median 9 against an expected 9 at 167 Hz over 51 ms, coverage of the frame interval has median 0.97 to 1.00 with a minimum of 0.882, the zero order hold fallback spans at most 6 ms which is one inertial period of quantisation residual, and the stamp rate is 167.08 to 167.93 Hz with no non monotonic sample. The epoch offset at initialisation is minus 30 ms and minus 87 ms, so the two streams share the simulation time epoch and the hazard recorded in [`../context/ap_dds_imu_stream.md`](../context/ap_dds_imu_stream.md) is closed. See `figures/runall_inertial.png`.

The front end is sound in both, with survival medians of 0.902 and 0.888 and the dominant rejection being the forward backward round trip gate, which is the intended behaviour of a conservative test. See `figures/runall_frontend.png`. The one interval where it is not sound is takeoff, treated in 3.6.

The two rows that separate the flights are the bias rows, and they separate them in opposite directions. Flight A holds the accelerometer bias at 0.053 and lets the gyroscope bias reach 0.048 rad/s. Flight B holds the gyroscope bias at 0.007 and lets the accelerometer bias reach 2.502 m/s^2. Figure `figures/runall_state.png` shows the two as mirror images, and section 2.2.6 gives the mechanism.

The local mapper correction loop is worth one remark. Flight A rejects every correction with a median translation error of 12.39 m against a 0.10 m acceptance threshold, which is the mapper correctly refusing to follow a diverged estimator. Flight B's median rejected error is 0.387 m, within a factor of four of the threshold, so the correction loop is close to engaging and would be worth revisiting once the drift is reduced.

### 3.3 Flight A, the anchored accelerometer bias

Flight A succeeded at what it was configured to do and failed at what it was intended to do.

The anchor held. The accelerometer bias norm has median 0.053 m/s^2 and maximum 0.233, against 3.37 in the divergence that motivated the parameter and 2.502 in Flight B. The mechanism the parameter targets works exactly as designed.

The trajectory nonetheless diverged, and in the same shape as before. Speed rises from 1.71 m/s at 3.5 s to 92.9 m/s at 24.5 s, a mean of 4.34 m/s^2, and continues to 757 m/s at the end, a mean over the whole run of 7.08 m/s^2 which is `9.81 sin(46.2 deg)`. Position reaches 41,484 m. The path length is 41,708 m and the closure error is 41,484 m, that is 99.5 percent of the distance travelled.

The error is in the attitude, not the bias. The gyroscope bias reaches minus 0.048 rad/s on the body y axis by 5.6 s and holds near that value for fifty seconds before relaxing back through zero, which is visible in the lower right panel of `figures/runall_state.png`. A sustained 0.048 rad/s is 2.75 degrees per second, and it is the rate at which the world frame is being rotated away from gravity. The mean acceleration deduced above corresponds to 46 degrees of accumulated tilt, which the observed rate reaches in seventeen seconds.

The vision term records the conflict. Its median is 13,003 against 292 in Flight B, a factor of 45. The images are contradicting the solution continuously, and the optimiser is overruling them because the anchor placed on the accelerometer bias is stiffer than anything the reprojection residuals can muster. The marginalisation prior reaches minus 3.67e7 at its extreme, an order of magnitude beyond Flight B, which is the first estimates Jacobian deviation growing as the state runs away from its linearisation points.

The map follows the trajectory, which is the appearance section 2.4 predicts. The mean triangulated depth has median 83,341 m and maximum 9.94e7, and the baseline the estimator believes it used has median 944 m. Because poses and map inflate together every reprojection stays satisfiable in relative terms, which is why an intact looking map beside a diverged trajectory is expected rather than contradictory. See `figures/run1_triangulation.png`.

One reading that might be mistaken for the cause is not. The stiff anchor enters the square root factor as `sqrt(1e8) = 1e4`, which `optimize` squares when it forms the dense normal equations before the LDLT at `sqrt_keypoint_vio.cpp:1543`. That is a condition number of order 1e8 against vision rows of order unity, comfortably inside double precision, and the position anchor at `vio_init_pose_weight = 1e8` contributes an identical magnitude in both flights. Conditioning is therefore eliminated as an explanation, and the confound of section 2.2.6 is what remains.

### 3.4 Flight B, the bounded run

Flight B is the reference for every experiment in section 5, and its accuracy can be stated without any ground truth.

The mission is recovered. The top down track in `figures/run2_trajectory.png` shows the commanded spline with its two lobes, flown between 80 s and 240 s, followed by a descent and a landing. The path length is 2,480.4 m.

The closure error is 90.2 m, which is 3.64 percent of the distance travelled. It decomposes into 78.6 m of altitude and 44.6 m of horizontal error.

The altitude tells the whole story of the drift, and is the most valuable single panel in the figure set. The estimator climbs to a plateau whose mean is 209.0 m and whose peak is 215.2 m, where 121 m was commanded, a scale factor of 1.73 to 1.78. It then descends, covering 136.6 m of estimated altitude for a true descent of 121 m, a scale factor of 1.13. The difference of 78.6 m is exactly the altitude it still reports when the vehicle is on the ground. The scale error was therefore acquired during the climb and largely corrected before the descent, and what remains at landing is the frozen residue of the climb rather than a continuing error.

The accelerometer bias explains the timing precisely. It rises to `b_a = (2.30, -0.60, -0.13)` by 6.35 s, holds within a few percent of that value until 78 s, and then collapses to 0.5 by 80 s and to 0.03 by 270 s. The plateau is a horizontal magnitude of 2.377 against a specific force magnitude of 9.96, which by section 2.2.3 is an attitude error of 13.8 degrees. The collapse coincides with the start of the spline, where the speed rises from 8.52 m/s at 77.1 s to 16.90 m/s at 84.8 s. The estimator identified the bias at the first moment the mission gave it the horizontal acceleration required to do so, and not before.

The map is metrically plausible. Mean triangulated depth has median 197.7 m against a true slant range of 171.1 m at the commanded altitude, and a tenth percentile of 20.4 m which is the climb. The largest baseline used has median 30.2 m, which by the table of 2.4.1 gives a relative depth uncertainty below one percent. Whatever else is wrong, triangulation during cruise is being given ample parallax.

The cost split is healthy. Vision has median 292 and the inertial term 11.7, so neither dominates and the inertial constraint is being satisfied rather than overruled. See `figures/run2_cost.png`.

Latency is within budget but not by a wide margin. `optimize_and_marg` has median 11.79 ms and ninetieth percentile 14.21 ms against a 51 ms frame interval, with a maximum of 111 ms. The front end costs 1.22 ms median. There is room to lengthen the window, which Phase 5 proposes, but the maximum should be watched.

### 3.5 The common origin, the first four seconds

Both flights acquire their error in the same interval and in the same way, and this is the section that identifies the root cause. The measurements are in `figures/run1_triangulation.png` and `figures/run2_triangulation.png` and are tabulated below from the `[VIO] triang` and `[Optical Flow] frame` records.

Through the ground phase the estimator creates nothing. Every keyframe reports `added = 0` with every candidate rejected on baseline, the landmark database stays at zero, and the largest baseline the estimator believes it has grows steadily from 5.55e-17 m to 0.059 m over 1.25 s. That growth is not motion. The vehicle is stationary, the measured interframe flow is 0.22 to 0.30 px which is tracker noise, and the baseline is the estimator's own position estimate drifting because nothing constrains it.

Then the drift crosses the threshold.

| | Flight A | Flight B |
|---|---|---|
| Time of the first successful triangulation | 1.25 s | 1.42 s |
| Baseline believed at that keyframe | 0.0590 m | 0.0541 m |
| Landmarks created | 89 | 25 |
| Mean depth assigned | 117.1 m | 63.3 m |
| True depth, camera 0.1 m above ground pitched 45 degrees | under 1 m | under 1 m |
| Measured disparity at that keyframe | 0.296 px | 0.223 px |
| Phantom depth predicted by `f b / disparity` | 63.8 m | 77.6 m |

The prediction of section 2.4.3 is a mean depth of 64 to 78 m from a mean disparity, and the measured means are 117.1 m and 63.3 m. Flight B agrees to eighteen percent and Flight A is high by a factor of 1.8, which is the expected behaviour of a mean taken over a ratio whose denominator is noise, since the small disparities that dominate the mean depth are not the mean disparity. The order of magnitude is the point. The first map the estimator ever builds is wrong by two orders of magnitude, and it is created from nothing but tracking noise scaled by the estimator's own drift.

Takeoff then destroys that map and rebuilds it. Interframe flow rises from 0.25 px to 21.4 px within four frames in Flight B, survival falls to 0.34, and the landmark database reaches zero at 4.39 s. The same interval in Flight A shows flow of 19.6 px and survival of 0.097. Between 4.4 s and 6.5 s both flights rebuild at depths of 1 to 14 m, which are approximately correct for a vehicle a few metres above the ground.

The bias locks during that rebuild. Flight B's `ba_x` goes 0.43 at 4.29 s, 1.46 at 4.97 s, 2.03 at 6.35 s and then plateaus. Flight A's gyroscope bias goes 0.0285 rad/s at 4.29 s and 0.0528 at 5.64 s and then plateaus. The confounded error is therefore acquired in the two seconds during which the map is being rebuilt from a standing start while the vehicle undergoes the only strong vertical acceleration of the mission. Everything after that is consequence.

Flight A carries one further symptom of the inconsistency, and it is the sharpest single discriminator between the two flights. Its `behind` counter, which records triangulations rejected for placing the point behind the camera, has a median of 214 per keyframe and a maximum of 786. Flight B's median is 1. A cheirality failure at that rate means the pose set and the bearings no longer describe a consistent geometry at all, and it is worth adding to the reading table of 3.8 as the cheapest early warning that a run has gone wrong.

### 3.6 The front end capture range, measured

Track survival was binned against mean interframe flow across both flights, and the result is `figures/runall_capture.png`.

| Mean flow [px per frame] | Flight A, mean survival | Flight B, mean survival | Largest flow in those frames [px] |
|---|---|---|---|
| 0 to 1 | 0.872 | 0.910 | 7.6 and 29.1 |
| 1 to 2 | 0.834 | 0.870 | 7.2 and 5.6 |
| 2 to 4 | 0.796 | 0.864 | 8.6 and 9.8 |
| 4 to 8 | 0.680 | 0.840 | 15.7 and 18.2 |
| 8 to 12 | 0.637 | 0.823 | 24.3 and 23.4 |
| 12 to 16 | 0.595 | 0.760 | 33.5 and 33.6 |
| 16 to 20 | 0.397 | 0.683 | 30.0 and 38.4 |
| above 20 | 0.161 | 0.384 | 32.7 and 41.0 |

The degradation is monotone and the knee sits between 16 and 20 px of mean flow, at which point the largest flow in those frames is 30 to 41 px. Section 2.5 computes the capture range of this configuration as `3.5 * 2^3 = 28` px, and the measurement brackets it. The tracker is behaving exactly as its geometry predicts, and the configuration simply does not have enough pyramid levels for the takeoff transient.

The number of frames involved is small, 18 frames above 12 px in Flight A and 256 in Flight B out of 1,831 and 5,600, so the cost in aggregate statistics is invisible. The cost in accuracy is not, because those frames are concentrated at takeoff, which section 3.5 identifies as the moment the error is acquired.

### 3.7 Measuring accuracy without a ground truth subscription

`class BasaltSLAMNode` creates exactly two subscriptions, the image at `ros_ws/src/slam/src/basalt/node.cpp:125` and the inertial stream at `:140`. Nothing subscribes to a reference trajectory, so no statistic the stack produces is an accuracy measurement, and every number in sections 3.2 to 3.4 other than the three below is a statement about internal consistency.

Three measurements escape that limitation, and they are what made this analysis possible. They should be computed for every run from here on.

The closure error, being the distance between the first and last estimated position, is drift in metres, because the mission lands where it took off. Flight A, 41,484 m. Flight B, 90.2 m over a 2,480 m path, that is 3.64 percent.

The altitude scale factor, being the estimated cruise plateau divided by the commanded altitude, is the scale error directly. Flight B, 209.0 over 121, that is 1.73.

The residual altitude at landing, being the estimated height of a vehicle that is on the ground, isolates the frozen component of the error from the part that was later corrected. Flight B, 78.6 m.

These are cheap and require no code change whatever. They are not a substitute for a proper reference trajectory, which Phase 0 adds, because they measure only the endpoints and say nothing about the error in between.

### 3.8 What each statistic decides

This table is the reading order for the next capture. A reader who has not seen this system can work down it and reach a conclusion without re deriving anything.

| Question | Statistic | Healthy value | Reading if it fails |
|---|---|---|---|
| Did the estimator start at rest | `[VIO] init:` accelerometer magnitude | 9.80 to 9.95 | the activation policy has regressed, nothing downstream is interpretable |
| Are the two streams in one epoch | `[VIO] init:` `imu_minus_frame_ns` | order 1e8 or smaller | an epoch mismatch, see `ap_dds_imu_stream.md` |
| Is the inertial stream complete | `imu_integrated` against `imu_expected`, `imu_coverage` | within one sample, coverage above 0.88 | samples are being dropped by the middleware or the queue |
| Is the zero order hold benign | `imu_stretch_ms` | at most one inertial period, 6 ms | a real gap in the stream |
| Was a phantom map created | `tri_added` and `tri_depth_m_mean` on the first keyframes | no landmarks until the vehicle moves | the initialisation of 3.5 has recurred |
| Did takeoff survive | `of_survival` at maximum `of_flow_px_mean` | above 0.6 | the capture range of 2.5 was exceeded |
| Is the accelerometer bias sane | `st_ba_norm` | below 0.05 m/s^2 | the confound of 2.2.3 has been resolved wrongly |
| Is the gyroscope bias sane | `st_bg` per axis | below 1e-3 rad/s | the attitude is rotating, which is the Flight A failure |
| Is the map metrically right | `tri_depth_m_mean` against 171 m | within a factor of two | the scale is wrong by that factor |
| Is triangulation given enough parallax | `tri_baseline_m_max`, `tri_short_baseline` | above 2.7 m on most keyframes | the threshold of 2.4.1 is admitting undetermined landmarks |
| Which term is binding | `split_vision` against `split_imu` | comparable orders, vision below about 1,000 | a vision cost in the tens of thousands means the images are being overruled |
| Is the geometry consistent | `tri_behind` per keyframe | median in single figures | a median in the hundreds means the poses and the bearings no longer agree, which is the Flight A signature |
| Is the window saturated | `assoc_connected_ratio` against `assoc_thresh` | ratio above the threshold on most frames | the keyframe rate is set by `vio_min_frames_after_kf` alone |
| Is there time budget | `tim_opt_and_marg` against the frame interval | p90 below one third | lengthening the window is not affordable |
| How large is the drift | closure error and altitude scale factor of 3.7 | closure below one percent of path | the headline number, and the one an experiment is scored on |

## 4. Root Cause Analysis

Eight causes follow. Each states the mechanism in terms of section 2, the evidence from section 3, and a prediction that the next capture will confirm or refute. They are ordered by the strength of the evidence, not by ease of remedy, and the first four form one causal chain rather than four independent faults.

### R1 The map is created from tracking noise before the vehicle moves

Mechanism. Section 2.4.3. While the vehicle is stationary the true parallax is zero, so the direct linear transform is degenerate and its output is determined by noise. The estimator nevertheless believes it has a baseline, because its own position estimate drifts under no constraint whatever, and once that drift exceeds `vio_min_triangulation_dist = 0.05 m` it scales a noise disparity by a noise baseline and obtains `d = f b / disparity`, of order 60 to 120 m. The acceptance window at `sqrt_keypoint_vio.cpp:681-689` bounds depth only from below at 0.333 m and cannot catch it.

Evidence. Section 3.5. Both flights create their first landmarks at a believed baseline of 0.054 and 0.059 m, assigning mean depths of 63.3 and 117.1 m where the truth is under one metre. The landmark database is empty for the preceding 1.25 to 1.42 s, so this phantom map is the only structure the estimator has when takeoff begins.

Prediction. Suppressing triangulation until the parallax exceeds the measurement noise will leave the database empty through the whole ground phase, which is correct, and will delay the first map until the vehicle has genuinely moved. `tri_added` should be zero until takeoff and the first assigned depths should then be of order one to five metres.

### R2 Takeoff exceeds the capture range of the tracker and annihilates the map

Mechanism. Section 2.5. The pyramidal Lucas Kanade tracker converges while the displacement at the coarsest level lies inside the patch, giving a capture range of `3.5 * 2^3 = 28` px at `optical_flow_levels = 3` with `Pattern51`. At takeoff the scene depth falls to about one metre while the speed is already several metres per second, and `flow = f v dt / d` rises accordingly.

Evidence. Section 3.6. Flow reaches 21.4 px mean and 41 px maximum, survival falls to 0.34 in Flight B and 0.097 in Flight A, and the landmark database reaches zero at 4.39 s in Flight B. The binned survival curve has its knee between 16 and 20 px, bracketing the computed 28 px range.

Prediction. Raising `optical_flow_levels` to 4 doubles the capture range to 56 px and should keep survival above 0.6 through the transient. If survival still collapses, the limitation is motion blur rather than capture range, which is a different problem with a different remedy.

### R3 The stationary phase leaves tilt and accelerometer bias confounded, and takeoff resolves the ambiguity wrongly

Mechanism. Section 2.2.3. With no linear acceleration, a body tilt and a horizontal accelerometer bias related by `delta_b = g delta_theta` produce an identical accelerometer reading. Vision breaks the tie only when there are landmarks with real parallax, which R1 shows there are not. The estimator therefore enters takeoff with two degrees of freedom that no measurement distinguishes, and resolves them during the two seconds in which it is also rebuilding its entire map from a bad one.

Evidence. Section 3.4. Flight B settles at `b_a = (2.30, -0.60, -0.13)` by 6.35 s, a horizontal magnitude of 2.377 against a specific force of 9.96, which is 13.8 degrees of attitude error. The value is established during the map rebuild at 4.29 to 6.35 s and then held for seventy seconds.

Prediction. Averaging the gravity reference over the stationary interval and asserting the known zero velocity will reduce the magnitude the pair settles on. It will not eliminate the confound, because the confound is a property of the motion, and only the excitation of R4 removes that.

### R4 The confounded pair is broken only by horizontal acceleration, which the mission withholds for 78 seconds

Mechanism. Section 2.2.4. The accelerometer bias becomes observable when the vehicle accelerates, because only then does the bias enter the position and velocity rows in a way that a tilt does not. The mission climbs and hovers for its first 78 seconds, which carries no such excitation, and section 2.2.5 shows that the alternative route, the vision term detecting the position error the bias produces, is worth 0.022 px at this altitude and is therefore unavailable.

Evidence. Section 3.4. Flight B's accelerometer bias collapses from 2.3 to 0.5 m/s^2 between 78 s and 80 s, exactly as the speed rises from 8.52 to 16.90 m/s at the start of the spline. Before that moment it is flat to within a few percent for seventy seconds.

Prediction. This one is not removed by a configuration change and should not be treated as a defect. It is the Martinelli observability result. What can be changed is how long the unresolved period lasts, by ensuring the takeoff itself is estimated against a correct map, and how much damage it does, which is R5.

### R5 Marginalisation freezes the error acquired during the unobservable period

Mechanism. Section 2.6. The marginalisation prior is computed once at the state the variables held when they were discarded and is never re linearised, which first estimate Jacobians make consistent at the price that an early error is permanent. Every state marginalised during the seventy two seconds in which the bias was wrong carries the resulting position error into the prior.

Evidence. Section 3.4. The climb is over integrated by a factor of 1.73 to 1.78, reaching a plateau of 209 m where 121 m was commanded. The descent, flown after the bias was identified, is tracked at a factor of 1.13. The difference of 78.6 m is the altitude the estimator still reports with the vehicle on the ground, and it accounts for 87 percent of the 90.2 m closure error.

Prediction. This is the amplifier rather than the source. Fixing R1, R2 and R3 reduces the error that gets frozen, and nothing short of a relinearisation scheme removes the freezing itself, which is out of scope. A useful intermediate check is whether the residual altitude at landing falls in proportion to the reduction in the climb scale factor.

### R6 The triangulation threshold is a distance where it should be a parallax

Mechanism. Section 2.4.1. The quantity that must be bounded is `sigma_d / d = sigma_px d / (f b)`, which depends on depth as well as baseline, and a threshold in metres cannot express it. At 171 m the configured 0.05 m gives 0.094 px of parallax and 535 percent depth uncertainty.

Evidence. Section 3.2. Flight B's median largest baseline is 30.2 m during cruise, so parallax is abundant when the vehicle is moving, and the threshold is not the binding constraint there. It is decisive only at the two moments that matter, the ground phase of R1 and the low altitude rebuild after R2.

Prediction. A parallax threshold will reduce `tri_added` per keyframe, will raise `tri_baseline_m_max`, and will move `tri_depth_m_mean` towards the true slant range. Its main value is that it makes R1 impossible by construction rather than by tuning, and that it transfers to a ground or legged platform without being recomputed.

### R7 The inertial characterisation under weights the only scale bearing term

Mechanism. Section 2.3. The weight of the inertial residual is inversely proportional to `accel_noise_std`, and section 2.1 established that the inertial term is the only term in the objective that observes metric scale. The declared 2.000e-2 against a derived 1.1768e-2 weights the scale constraint at 0.588 of its correct value while the scale invariant vision term is weighted correctly. The two bias densities compound it by permitting both biases to wander 1.41 times faster than the sensors do.

Evidence. The discrepancy table of section 2.3, derived in [`../context/imu_noise_parameters.md`](../context/imu_noise_parameters.md) from the robot configuration and verified against the Project AirSim source.

Prediction. Correcting the three values will reduce the scale error measurably and will not remove it, because it does not address R1 through R4. If the altitude scale factor moves by roughly the 1.70 ratio, the inertial weight was the dominant term, and if it barely moves, the scale is being set by the initialisation rather than by the inertial measurement, which points back at R1.

### R8 The odometry back end performs no outlier rejection

Mechanism. Section 2.7. The Huber loss down weights a gross outlier but never removes it, `filterOutliers` is never called from the odometry estimator, and the two configuration keys that would govern it are inert.

Evidence. The standing TODO at `sqrt_keypoint_vio.cpp:1758` and the commented out fields at `include/basalt/utils/vio_config.h:68-69` and `src/utils/vio_config.cpp:71-72, 189-190`. Flight A's median of 214 cheirality rejections per keyframe against Flight B's 1 shows what an inconsistent geometry looks like when nothing removes it.

Prediction. Little effect on the drift, because the phantom landmarks of R1 fit their observations well and are not outliers in reprojection. Worth restoring for robustness against genuine mismatches, and worth evaluating only after R6 has removed the population it would otherwise be asked to clean up.

## 5. Experimentation Plan

The phases are ordered so that each is measurable on its own and the cheapest changes come first. Every phase changes one thing, because a phase that changes two cannot be attributed. Each carries the change, its justification, the implementation instructions and the rules for reading the result.

### Phase 0, establish a truth reference

#### Need

Section 3.7 showed that the stack has no idea where the vehicle is, and that only three endpoint measurements escape that. Those three were enough to diagnose the problem and are not enough to score a remedy, because they say nothing about where along the trajectory the error accrues.

#### The change

ArduPilot's AP_DDS bridge already publishes `/ap/geopose/filtered` and `/ap/twist/filtered`, and `/ws/mav.tlog` carries the same information offline through the MAVLink `LOCAL_POSITION_NED` message. In simulation the autopilot's filter is driven by the simulator's true state plus the modelled sensor noise and is accurate to well under a metre over this mission, which is three orders of magnitude better than the drift under investigation. It is a reference rather than truth and should be described as such whenever a number derived from it is quoted.

The metrics are the standard pair from Zhang and Scaramuzza (2018). Absolute trajectory error after a similarity alignment reports the accumulated drift, and the scale factor of that alignment reports the scale error directly, which is the single number this investigation most needs. Relative pose error over a fixed distance, every 10 m of travelled path, reports the local drift rate independently of where the error was committed.

One instrumentation change is required, because only the translation is currently printed.

```cpp
        std::cout << "[VIO] state t_ns=" << p.getT_ns()
                  << " p_w_i=" << st.T_w_i.translation().transpose()
                  << " |p|=" << st.T_w_i.translation().norm()
                  << " q_w_i=" << st.T_w_i.unit_quaternion().coeffs().transpose()
                  << " v_w_i=" << st.vel_w_i.transpose()
```

#### Implementation

Add the quaternion to the state record as above, noting that `Eigen`'s `coeffs()` ordering is x, y, z, w. Extend `scripts/vio_log_extract.py` to emit the estimated trajectory in TUM format from the `st_p_w_i_*` and the new quaternion columns, and add a small reader for the telemetry log. Do not write the alignment by hand. Emit both trajectories in TUM format and use `evo`, whose `evo_ape` and `evo_rpe` implement exactly these metrics with the Umeyama alignment.

`EXPECTED_LAST` in `scripts/vio_log_extract.py` names the final field of each record and must be revisited in the same commit if a record's last field changes, which this one does not.

#### Interpretation

Report the absolute trajectory error, the alignment scale factor and the relative pose error per 10 m for every run from here on, alongside the three endpoint measurements of section 3.7 which remain useful as a cross check. A scale factor at 1.00 with a large absolute error is a drift problem. A scale factor away from 1.00 is a scale problem. They are different faults with different remedies and no experiment below can be scored without separating them.

### Phase 1, correct the inertial characterisation

#### The change

Configuration only, no code. In `data/sitl_calib.json`,

```json
"imu_update_rate": 167,
"gyro_noise_std":  [5.8178e-4, 5.8178e-4, 5.8178e-4],
"accel_noise_std": [1.1768e-2, 1.1768e-2, 1.1768e-2],
"gyro_bias_std":   [5.5981e-6, 5.5981e-6, 5.5981e-6],
"accel_bias_std":  [2.0018e-4, 2.0018e-4, 2.0018e-4]
```

and in the ArduPilot parameter file used by this stack,

```
SIM_DRIFT_SPEED 0
```

#### Justification

Section 2.3 and root cause R7. The four values are derived in [`../context/imu_noise_parameters.md`](../context/imu_noise_parameters.md) from the robot configuration at `/ws/sitl_ws/config/airsim2/robot_ardu_copter.jsonc:345-366`. `accel_noise_std` is the important one, because the inertial residual is the only scale bearing term in the objective and its weight is inversely proportional to this value.

`gyro_bias_std` takes the simulator derived figure rather than the inflated 1.5586e-5 because `SIM_DRIFT_SPEED` is being zeroed in the same change. Part 7.3 of the context file explains the alternative. Zeroing the drift is preferred because it removes a second bias process the calibration does not describe rather than accommodating it by modelling a deterministic ramp as a random walk, which it is not.

#### Implementation

Edit the two files. No rebuild is required, because the calibration is read at runtime. Confirm from the launch that `data/sitl_calib.json` is the file actually loaded, since `djinn:366` and `djinn:409` both pass it and more than one branch reads it.

#### Interpretation

Expect the altitude scale factor to move towards 1.00 and expect it not to reach it. Record `split_imu` against `split_vision` before and after, since a correctly weighted inertial term should become a larger fraction of the total. A movement of roughly the 1.70 ratio confirms that the inertial weight was dominant.

### Phase 2, gate triangulation on parallax rather than on a metric baseline

#### The change

A new configuration field whose default reproduces the current behaviour exactly, so no existing run changes and the new behaviour is opted into by the SITL configuration alone.

In `include/basalt/utils/vio_config.h`, beside `vio_min_triangulation_dist`,

```cpp
    double vio_min_triangulation_dist;
    /// Minimum parallax angle in degrees required between the two bearings of a
    /// triangulation. Zero leaves the test inactive, which reproduces the
    /// behaviour of every configuration written before this field existed.
    double vio_min_triangulation_parallax_deg;
```

In `src/utils/vio_config.cpp`, in `VioConfig::VioConfig` beside the existing default and in `serialize`,

```cpp
    vio_min_triangulation_dist = 0.05;
    vio_min_triangulation_parallax_deg = 0.0;
```

```cpp
    ar(CEREAL_NVP(config.vio_min_triangulation_dist));
    ar(CEREAL_NVP(config.vio_min_triangulation_parallax_deg));
```

In `src/vi_estimator/sqrt_keypoint_vio.cpp`, beside the existing `min_triang_distance2` at line 637,

```cpp
        const Scalar min_triang_distance2 =
            Scalar(config.vio_min_triangulation_dist *
                   config.vio_min_triangulation_dist);
        // Cosine of the minimum permitted parallax angle. A configured value of
        // zero leaves the test inactive. Parallax is measured between the two
        // bearings and therefore needs no depth estimate, so it is available
        // before triangulation rather than after it.
        const Scalar min_parallax_cos =
            config.vio_min_triangulation_parallax_deg > 0
                ? std::cos(Scalar(config.vio_min_triangulation_parallax_deg) *
                           Scalar(M_PI / 180.0))
                : Scalar(1);
```

and inside the candidate loop, immediately after the existing baseline test,

```cpp
                if (T_0_1.translation().squaredNorm() < min_triang_distance2) {
                    numShortBaseline++;
                    continue;
                }

                if (min_parallax_cos < Scalar(1)) {
                    const Vec3 f0 = p0_3d.template head<3>().normalized();
                    const Vec3 f1 =
                        (T_0_1.so3() * p1_3d.template head<3>()).normalized();
                    if (f0.dot(f1) > min_parallax_cos) {
                        numLowParallax++;
                        continue;
                    }
                }
```

with `int numLowParallax = 0;` added to the tally declarations at line 604 and the record extended,

```cpp
                      << " short_baseline=" << numShortBaseline
                      << " low_parallax=" << numLowParallax
```

Finally, in `data/sitl_config_vo.json` alone,

```json
        "config.vio_min_triangulation_parallax_deg": 0.9,
```

#### Justification

Section 2.4 and root causes R1 and R6. The quantity to bound is `sigma_d / d = sigma_px / (f alpha)` where `alpha = b / d` is the parallax angle, so a threshold in angle bounds the ratio that matters and transfers between scenes without retuning. At `f = 320` one pixel of parallax is 0.179 degrees, and a ten percent depth uncertainty at `sigma_px = 0.5` needs five pixels, that is 0.9 degrees.

Measuring the angle between the two bearings rather than between the two rays through the triangulated point is deliberate. It requires no depth, so the test applies before the singular value decomposition rather than after it, and it therefore also avoids spending the decomposition on candidates that will be rejected. The same construction is already used by the mapping layer, which carries `mpMaxCosParallax` in `include/basalt/vi_estimator/local_mapper.h`.

Most importantly, this makes R1 impossible by construction. A stationary vehicle produces no parallax at all, so no threshold expressed in parallax can be crossed by the estimator's own drift, whereas any threshold expressed in metres can.

#### Implementation

Adding a field to `serialize` is a breaking change for every configuration file that predates it, because a key read by `serialize` and absent from the file throws an uncaught `cereal::Exception` and aborts the process, as established in [`../context/vio_config_json.md`](../context/vio_config_json.md). All eight configuration sources must be updated in the same commit, namely the seven files matching `data/*config*.json` and the `[value0]` table in `data/iccv21/basalt_batch_config.toml`, which a search for JSON files misses. Seven take the default of 0.0 and only `data/sitl_config_vo.json` takes 0.9, which is what keeps every historical comparison valid.

Rebuild with `colcon build --packages-select slam` on the build host. The dev container has no Eigen and cannot compile this library. The build uses `-Wall -Wextra -Werror`, so check the first build for new diagnostics.

#### Interpretation

The decisive check is the first ten seconds. `tri_added` should be zero for the whole ground phase and `assoc_lmdb_landmarks` should stay at zero until the vehicle genuinely moves, where today both flights create 25 to 89 landmarks at 60 to 120 m. The first depths assigned after takeoff should be of order one to five metres.

Through cruise expect `low_parallax` to account for a modest number of rejections, `tri_baseline_m_max` to rise and `tri_depth_m_mean` to move towards 171 m. A fall in `tri_added` below roughly twenty per keyframe or in `assoc_lmdb_landmarks` below fifty means the threshold is starving the estimator and should be reduced towards 0.45 degrees, which is 2.5 px and twenty percent depth uncertainty. Starvation during the rotation in place is expected rather than a fault, because a pure rotation produces no parallax by definition.

One caveat on reading the tallies. `candidates` counts landmark candidates, but every rejection counter is incremented once per candidate and prior observation pair, because the inner loop tries a candidate against each earlier frame that saw it until one is accepted. The counters therefore routinely exceed `candidates`, and only `added` and `candidates` are directly comparable.

### Phase 3, treat the stationary interval explicitly

Two separable parts, to be measured separately.

#### Phase 3a, average the gravity reference over the samples already discarded

##### The change

The initialisation branch already discards every inertial sample older than the first frame, at `sqrt_keypoint_vio.cpp:259-269`. Those samples are free and are currently thrown away.

```cpp
        Vec3 accelSum = imuData->accel;
        int numAccelAveraged = 1;
        while (imuData->t_ns < curr_frame->t_ns) {
            imuData = popFromImuDataQueue();
            if (!imuData) break;
            numImuSkippedBehind++;
            imuData->accel =
                this->calib.calib_accel_bias.getCalibrated(imuData->accel);
            imuData->gyro =
                this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
            accelSum += imuData->accel;
            numAccelAveraged++;
        }

        if (!imuData) return nullptr;

        Vec3 vel_w_i_init;
        vel_w_i_init.setZero();

        // The gravity reference is the mean of the discarded samples rather
        // than the last of them. One sample carries 0.1521 m/s^2 of noise per
        // axis, which is 0.89 degrees of tilt, and the mean of N reduces that
        // by sqrt(N). See plans/vio_drift_analysis.md section 2.2.3.
        T_w_i_init.setQuaternion(Eigen::Quaternion<Scalar>::FromTwoVectors(
            accelSum / Scalar(numAccelAveraged), Vec3::UnitZ()));
```

and the initialisation record should report what was averaged,

```cpp
            std::cout << "[VIO] init: skipped_behind=" << numImuSkippedBehind
                      << " averaged=" << numAccelAveraged
                      << " accel_mean=" << (accelSum / Scalar(numAccelAveraged)).transpose()
```

##### Justification

Section 2.2.3. The seed attitude is currently taken from a single accelerometer sample whose per axis standard deviation is `1.1768e-2 * sqrt(167) = 0.1521` m/s^2, worth `0.1521 / 9.81 = 0.0155` rad, that is 0.89 degrees of tilt. That is comparable to the 13.8 degree error the confound eventually settles on and is a genuine contributor to where it starts. Averaging `N` samples reduces it by `sqrt(N)`.

The two flights discarded 13 and 5 samples, so the free improvement is a factor of 3.7 and 2.4, giving 0.24 and 0.36 degrees. That is worth having and is not sufficient on its own, which is why Phase 3b follows.

##### Implementation

One function, no interface change, no configuration change, no effect on any other caller. `numAccelAveraged` starts at one and `accelSum` at the sample already held, because that sample is not re popped by the loop.

##### Interpretation

Compare `accel_mean` against 9.8048, the gravity magnitude at the 583 m home altitude, not against the 9.81 Basalt assumes. The two flights read 9.93888 and 9.94766 from a single sample, which are 0.134 and 0.143 above that figure and therefore consistent with one sample of noise. A mean that is still 0.1 away after averaging tens of samples indicates a real accelerometer bias, which is information worth having, because it is the quantity Phase 3b cannot observe either.

A magnitude above 9.9 after averaging would mean the estimator was activated under power and the lifecycle policy has regressed, which invalidates everything downstream.

#### Phase 3b, constrain the velocity while the vehicle is known to be stationary

##### The change

A zero velocity residual on the newest state, active only while a stationarity test passes. The residual is `r = v_w_i` with Jacobian the identity on the velocity block, weighted by `w`. The normal equation convention is stated at `src/vi_estimator/ba_base.cpp:424-441`, namely `H x + b = 0` with `H = J^T J` and `b = J^T r`, and `optimize` computes `inc = ldlt.solve(b)` at `:1544` and negates it at `:1569`, so the applied increment is `-H^{-1} b`. The addition goes immediately after `lqr->get_dense_H_b(H, b)` at `sqrt_keypoint_vio.cpp:1529`,

```cpp
                    lqr->get_dense_H_b(H, b);

                    // Zero velocity constraint. Only the most recent state is
                    // constrained, because the stationarity test is evaluated
                    // on the inertial samples of the current frame alone.
                    if (mpZeroVelocityWeight > 0 && mpIsStationary) {
                        const auto& idx =
                            aom.abs_order_map.at(last_state_t_ns);
                        const int vi = idx.first + 6;
                        const Scalar w2 = mpZeroVelocityWeight *
                                          mpZeroVelocityWeight;
                        H.template block<3, 3>(vi, vi).diagonal().array() += w2;
                        b.template segment<3>(vi) +=
                            w2 * frame_states.at(last_state_t_ns)
                                     .getState()
                                     .vel_w_i;
                    }
```

The velocity block sits at offset six within a `POSE_VEL_BIAS_SIZE` entry, by the tangent ordering `[trans, rot, vel, bias_gyro, bias_accel]` of `PoseVelBiasState::applyInc`, which is the authoritative statement and the same ordering the transposition of section 1.3 turns on.

The stationarity test is evaluated once per frame in `ProcessFrame`, over the samples integrated for that frame in the loop at `sqrt_keypoint_vio.cpp:336-349`,

```cpp
    // A frame is stationary when the specific force magnitude sits close to
    // gravity and the angular rate is small, over every sample of the frame.
    // Thresholds are several times the per sample noise of 0.1521 m/s^2 and
    // 7.518e-3 rad/s implied by the calibration.
    mpIsStationary = numImuIntegrated > 0 &&
                     accelDeviationMax < mpStationaryAccelTol &&
                     gyroNormMax < mpStationaryGyroTol;
```

with `mpStationaryAccelTol = 0.5` m/s^2, `mpStationaryGyroTol = 0.05` rad/s and `mpZeroVelocityWeight` defaulting to zero so the mechanism is inactive until a configuration enables it. Exposing the weight as `vio_zero_velocity_weight` follows the recipe of Phase 2, including the obligation to add the key to all eight configuration sources.

##### Justification

Section 2.2.3 and root cause R3. This does not remove the confound, and the plan should not claim otherwise. What it removes is the second way the stationary interval is harmful, namely that a vertical bias error generates a free parabola in position and velocity which nothing contradicts, because the inertial factors relate consecutive states and are perfectly satisfied by a constant acceleration. Pinning the velocity makes that parabola expensive, and in combination with Phase 2 it also stops the position drift that manufactures the phantom baseline of R1.

The weight is chosen rather than guessed. Treating the constraint as a measurement of standard deviation `sigma_v`, the weight is `1 / sigma_v`. A vehicle on the ground is stationary to well under a centimetre per second, so `sigma_v = 0.01` and `w = 100` is defensible and sits three orders of magnitude below the `1e4` of the position anchor, so it does not dominate the conditioning of the dense system.

The thresholds deserve a check against the data. The two flights show a stationary specific force magnitude of 9.94 and a gyroscope norm of 0.0084 to 0.0092 rad/s, against thresholds of 0.5 and 0.05, so the test will latch true on the ground with ample margin. During the climb the specific force reaches 11.7 and during the spline the gyroscope reaches 0.32 rad/s, both far outside, so a false positive in flight is implausible.

##### Implementation

Four members on `class SqrtKeypointVioEstimator`, one configuration field, one test in `ProcessFrame` and one block in `optimize`. The test needs the maximum deviation of the specific force magnitude from gravity and the maximum angular rate over the samples integrated for the frame, both accumulated in the existing integration loop at negligible cost. The guard `mpZeroVelocityWeight > 0` preserves backwards compatibility, so no existing configuration changes behaviour and no historical comparison is invalidated.

##### Interpretation

Expect `st_v_norm` at or below 0.01 m/s for the whole ground interval, where the flights show it unconstrained, and expect the position at the moment of takeoff to be centimetres rather than the 0.055 m of drift that manufactured the phantom baseline.

Log the fraction of frames on which `mpIsStationary` fires and confirm it is zero after takeoff before trusting any other number from the run. A constraint that latches in flight will actively corrupt the estimate.

### Phase 4, extend the tracker capture range through takeoff

#### The change

Configuration only. In `data/sitl_config_vo.json`,

```json
        "config.optical_flow_levels": 4,
```

#### Justification

Section 2.5 and root cause R2. The capture range is the patch half extent times two to the power of the level count, which with `Pattern51` at 3.5 px is 28 px at three levels and 56 px at four. The measured maximum flow at takeoff is 41 px, which the current setting cannot follow and the proposed one can.

The cost is one more pyramid level per image, which is a quarter of the area of the level below it and therefore about a third of one level's work added to a front end whose median total is 1.22 ms against a 51 ms budget. The risk is that a coarser level carries less texture and can converge to a wrong minimum, which is why the change is made alone and scored on survival rather than assumed beneficial.

#### Implementation

Edit one file. No rebuild. Note that `optical_flow_levels` is also read by the first frame path and by `addPoints`, so the change applies uniformly, and that `ManagedImagePyr::setFromImage` builds `num_levels + 1` levels so the configured value is the index of the coarsest.

#### Interpretation

The single check is survival through the takeoff transient. Plot `of_survival` against `of_flow_px_mean` with `figures/runall_capture.png` and compare the 16 to 20 px and above 20 px bins, which currently read 0.40 and 0.16 in Flight A and 0.68 and 0.38 in Flight B. Above 0.6 in both bins is success.

Watch `assoc_lmdb_landmarks` across takeoff. It currently reaches zero in Flight B at 4.39 s. If it stays above fifty the map survives the transient, which is the outcome that matters, because R3 and R5 both trace to the rebuild.

If survival does not improve, the limitation is motion blur rather than capture range. That is diagnosable from the images rather than the logs and would point at exposure time rather than at the tracker.

### Phase 5, lengthen the observability window

#### The change

Configuration only in the first instance. In `data/sitl_config_vo.json`,

```json
        "config.vio_max_states": 10,
        "config.vio_max_kfs": 10,
        "config.vio_min_frames_after_kf": 3,
```

#### Justification

Section 2.2.5 gives the bias detection time `T_detect = sqrt(0.5347 / b)` seconds at the mission slant range. The active inertial window is `(vio_max_states - 1)` frame intervals, so 0.102 s at the configured 3, which detects a bias of 51 m/s^2, and 0.459 s at 10, which detects 2.5 m/s^2. That is a partial remedy whose purpose is to establish the gradient of the improvement rather than to solve the problem, since reaching the 0.1 m/s^2 that matters would require of order 47 states.

Section 2.6 gives the cost. The dense system grows by fifteen columns per state and is factored by `Eigen::LDLT`. Flight B measures `optimize_and_marg` at 11.79 ms median and 14.21 ms at the ninetieth percentile against a 51 ms interval, with a maximum of 111 ms, so a threefold increase in state count is affordable but the maximum must be watched.

Raising `vio_min_frames_after_kf` from 1 to 3 is the other half. Section 2.6 showed the keyframe rate is pinned at its floor in both flights, at 3.00 and 3.08 frames per keyframe. The test is a strict inequality on a counter reset at each keyframe, so a setting of `n` permits a keyframe every `n + 2` frames, and 3 gives five frames, that is 255 ms. With `vio_max_kfs` at 10 the window then spans about 2.3 s and 30 m at cruise against 0.936 s and 12 m today, and section 2.4.1 shows that doubling the baseline halves the relative depth uncertainty, so this and Phase 2 reinforce each other.

#### Implementation

Edit one file. No rebuild. Run the three variations separately, states alone, keyframes alone and both, so the two effects are separable.

#### Interpretation

Plot `tim_opt_and_marg` first. If its ninetieth percentile approaches a third of the frame interval the window is as long as the budget allows and further improvement must come from elsewhere.

Then read `st_ba_norm` through the climb and the altitude scale factor of section 3.7. A bias estimate that falls as the window lengthens confirms that the limit is observability rather than a fault. One that does not move at any window length is R4, and says the monocular configuration cannot observe what is being asked of it during a pure climb.

Watch `assoc_connected_ratio` for the keyframe change. If it rises above the 0.8 threshold the ratio test becomes binding again, which is the healthy regime and means `vio_min_frames_after_kf` should not be raised further.

### Phase 6, directions that are not tuning changes

Recorded so that a later reader does not mistake their absence for an oversight. None should be attempted before Phases 0 to 5 have been measured.

#### Outlier rejection in the odometry back end

`filterOutliers` exists at `src/vi_estimator/ba_base.cpp:270-309` and is called by the mapping layer but never by the odometry estimator, where the call site is a standing TODO at `sqrt_keypoint_vio.cpp:1758`. The two governing configuration keys are commented out of `struct VioConfig` and are inert in every file. Restoring both is mechanical. Root cause R8 argues it should follow Phase 2 rather than precede it, because the landmarks that currently pollute the map fit their observations well and a reprojection based filter does not remove them.

#### Degeneracy detection

Zhang et al. (2016) propose examining the eigenvalues of the information matrix and suppressing the update along directions whose eigenvalue falls below a threshold. The square root formulation makes the factor available without extra work. This is the principled answer to R4 and to the rotation in place, and it generalises directly to the planar degeneracy a wheeled platform will meet. Flight A is precisely a case where a report that the solution was degenerate would have been more useful than the solution.

#### Relinearisation of the marginalisation prior

R5 is the amplifier that turns a transient error into a permanent one, and first estimate Jacobians are what make it permanent. Schemes exist that periodically rebuild the prior from retained measurements, at considerable cost in complexity and memory. This is recorded as the structural answer to R5 and is well beyond the scope of the present work.

#### An additional metric measurement

Section 2.2.4 shows that monocular scale requires acceleration and section 2.2.5 that at 171 m the residual bias is not observable through the images at any practical window length. Neither is a defect to be fixed by tuning. A stereo pair makes depth observable from a fixed baseline independently of motion, and a barometric or laser range measurement observes the vertical scale directly, which is the channel carrying 87 percent of the closure error in Flight B. For the ground and legged platforms of section 2.8 the equivalents are wheel odometry and the contact constraint, and all of them enter the estimator by the mechanism Phase 3b builds.

## 6. Conclusion

The estimator no longer diverges when it is configured as Flight B was, and it drifts by 90.2 m over a 2,480 m flight, which is 3.64 percent of the distance travelled. The error is not diffuse. Eighty seven percent of it is altitude, and the altitude error is acquired entirely during the first eighty seconds.

The chain is now established end to end and each link is measured rather than inferred. The vehicle stands still for four seconds, during which no landmark can be triangulated because there is no parallax, and the estimator's own unconstrained position drift crosses a threshold expressed in metres and manufactures a map at sixty to one hundred and twenty metres where the truth is under one metre. Takeoff then drives the interframe flow to forty one pixels against a tracker capture range of twenty eight, the map is annihilated and rebuilt, and during that rebuild the estimator must resolve a tilt and an accelerometer bias which, in the absence of horizontal acceleration, produce identical measurements. It resolves them onto 2.3 m/s^2 and 13.8 degrees, holds that for seventy two seconds, and marginalisation freezes the resulting position error into a prior that no later evidence can revise. The bias is corrected the instant the mission supplies horizontal acceleration, at seventy eight seconds, but by then the climb has been over integrated by seventy three percent and the vehicle lands believing it is seventy nine metres in the air.

Two conclusions deserve to be stated separately because they generalise past this mission.

A confounded pair must be broken by adding information, never by removing a degree of freedom. Flight A did the latter, anchoring the accelerometer bias at an implied 1e-4 m/s^2, and succeeded completely at holding that variable near zero. The error did not disappear, it moved into the gyroscope bias, which was effectively unconstrained, and a rotating world frame leaks gravity into the horizontal channel and diverges. The vision cost rose by a factor of forty five, which was the optimiser saying so in the log. Any stiff prior placed on one member of a confounded pair carries the same hazard.

Conditional degeneracy is invisible to the standard observability analysis and is what actually limits this system. `doc/Linearisation.md` §2.15 correctly identifies the four permanently unobservable directions and the estimator handles all four by construction. The directions that cost 90 m are not in that list. They are unobservable only because of what the vehicle is doing at the time, they appear and vanish with the motion, and nothing in the estimator notices them or reports them.

Phases 1 through 5 address the chain at four of its links and will not reduce the drift to zero, because section 2.2.4 shows that monocular visual inertial scale during a pure climb is weakly observable in principle and no amount of tuning changes a property of the trajectory. What they can do is ensure that the map the takeoff is estimated against is real, that the takeoff does not destroy it, and that the interval during which the estimator is guessing is as short and as well constrained as the mission allows. Phase 0 must come first regardless, because the three endpoint measurements that carried this analysis were enough to find the fault and are not enough to score a fix.

## 7. References

Baker, S. and Matthews, I. (2004). Lucas-Kanade 20 Years On, A Unifying Framework. International Journal of Computer Vision, 56(3). The inverse compositional tracker of section 2.5.

Bloesch, M., Hutter, M., Hoepflinger, M., Leutenegger, S., Gehring, C., Remy, C. D. and Siegwart, R. (2012). State Estimation for Legged Robots, Consistent Fusion of Leg Kinematics and IMU. Robotics, Science and Systems.

Civera, J., Davison, A. J. and Montiel, J. M. M. (2008). Inverse Depth Parametrization for Monocular SLAM. IEEE Transactions on Robotics, 24(5). The landmark parameterisation of `doc/VIO.md` §7.1.

Demmel, N., Schubert, D., Sommer, C., Cremers, D. and Usenko, V. (2021). Square Root Marginalization for Sliding-Window Bundle Adjustment. International Conference on Computer Vision. The formulation `doc/Linearisation.md` implements.

Engel, J., Koltun, V. and Cremers, D. (2018). Direct Sparse Odometry. IEEE Transactions on Pattern Analysis and Machine Intelligence, 40(3). Source of the keyframe marginalisation score used at `sqrt_keypoint_vio.cpp:932-973`.

Forster, C., Carlone, L., Dellaert, F. and Scaramuzza, D. (2017). On-Manifold Preintegration for Real-Time Visual-Inertial Odometry. IEEE Transactions on Robotics, 33(1). The preintegration of `doc/VIO.md` §4.

Hartley, R. and Sturm, P. (1997). Triangulation. Computer Vision and Image Understanding, 68(2). The bias of the algebraic minimiser in section 2.4.2.

Hartley, R. and Zisserman, A. (2004). Multiple View Geometry in Computer Vision, second edition. Cambridge University Press. Chapter 12 for the direct linear transform of `doc/VIO.md` §7.1.

Hartley, R., Ghaffari, M., Eustice, R. M. and Grizzle, J. W. (2020). Contact-Aided Invariant Extended Kalman Filtering for Robot State Estimation. International Journal of Robotics Research, 39(4).

Hesch, J. A., Kottas, D. G., Bowman, S. L. and Roumeliotis, S. I. (2014). Camera-IMU-Based Localization, Observability Analysis and Consistency Improvement. International Journal of Robotics Research, 33(1). The filtering counterpart of section 2.2.

Huang, G. P., Mourikis, A. I. and Roumeliotis, S. I. (2010). Observability-Based Rules for Designing Consistent EKF SLAM Estimators. International Journal of Robotics Research, 29(5). Origin of the first estimate Jacobian scheme of `doc/Linearisation.md` §2.16.

Leutenegger, S., Lynen, S., Bosse, M., Siegwart, R. and Furgale, P. (2015). Keyframe-Based Visual-Inertial Odometry Using Nonlinear Optimization. International Journal of Robotics Research, 34(3).

Martinelli, A. (2014). Closed-Form Solution of Visual-Inertial Structure from Motion. International Journal of Computer Vision, 106(2). The observability result underlying section 2.2.4, and the explanation of the 78 second bias collapse.

Mourikis, A. I. and Roumeliotis, S. I. (2007). A Multi-State Constraint Kalman Filter for Vision-Aided Inertial Navigation. International Conference on Robotics and Automation.

Qin, T., Li, P. and Shen, S. (2018). VINS-Mono, A Robust and Versatile Monocular Visual-Inertial State Estimator. IEEE Transactions on Robotics, 34(4). Its initialisation procedure is the standard treatment of the problem section 3.5 identifies.

Usenko, V., Demmel, N., Schubert, D., Stückler, J. and Cremers, D. (2020). Visual-Inertial Mapping with Non-Linear Factor Recovery. IEEE Robotics and Automation Letters, 5(2). The Basalt paper, a copy of which is at `/ws/1904.06504v3.pdf`.

Woodman, O. J. (2007). An Introduction to Inertial Navigation. Technical Report 696, University of Cambridge Computer Laboratory.

Wu, K. J., Guo, C. X., Georgiou, G. and Roumeliotis, S. I. (2017). VINS on Wheels. International Conference on Robotics and Automation. The planar motion degeneracy of section 2.8.

Zhang, J., Kaess, M. and Singh, S. (2016). On Degeneracy of Optimization-Based State Estimation Problems. International Conference on Robotics and Automation. The degeneracy detector of Phase 6.

Zhang, Z. and Scaramuzza, D. (2018). A Tutorial on Quantitative Trajectory Evaluation for Visual-Inertial Odometry. International Conference on Intelligent Robots and Systems. The metrics of Phase 0.

IEEE Std 952-1997. Standard Specification Format Guide and Test Procedure for Single-Axis Interferometric Fiber Optic Gyros. The definitions of angle random walk and bias instability used in section 2.3.
