# A Stationary Warm Start of the Visual Inertial Estimator

Status, 2026-11-15. Implemented in the working tree of branch `shreyas/kf_driven_mapping_and_logger` on top of the committed tree at `41cac8f`. Syntax checks and ten unit tests passed in the dev container, and the host build, the full unit tests and the replays of §5 remain outstanding.

## 1. Motivation

### 1.1 The take off sequences that were flown

The gravity and scale period of `plans/delayed_marginalisation.md` starts visual inertial odometry from the first frame, holds the gravity and scale block while DM-VIO's coarse initialiser corrects the window at every keyframe, and then frees the block under delayed marginalisation. A visual only coarse phase after DM-VIO, and a gyro aided variant of it, were implemented and flown on 2026-11-15 as an alternative, and both are rejected (§1.3). The visual inertial start measured the best scale and attitude of every take off flown (§1.2).

### 1.2 What the visual inertial start achieved

| Run | Start | Scale ratio, 4 s windows, first 30 s after lift off, p10 / median / p90 | Tilt error | Accelerometer bias |
|---|---|---|---|---|
| `/ws/logs/slam/2026-11-15_12-51-19` | VIO from frame 1 | 0.94 / 0.98 / 1.00 | at most 1.7° | at most 0.26 m/s² |
| `/ws/logs/slam/2026-11-15_11-30-32` | VIO from frame 1 | 0.86 / 0.97 / 0.99 | at most 8.1° | at most 0.19 m/s² |
| `/ws/logs/slam/2026-11-15_17-54-10` | gyro aided VO | 1.12 / 1.13 / 4.8 | at most 4.4° | jumps with every coarse installation |

The scale ratio is the length of the estimated displacement over a 4 s window divided by that of ground truth, from `gt_eval.csv`.

### 1.3 Why the visual only coarse phase was rejected

1. A window without inertial rows lost its attitude at the first triangulation, when 1 to 6 landmarks over centimetre baselines left the relative rotation underdetermined. The estimate rotated by 16° in one frame in `2026-11-15_17-32-54` and by 1.4° to 4.7° per frame in `2026-11-15_17-35-28`, and the coarse optimisation then explained the attitude error as an accelerometer bias equal to its gravity misprojection, up to 13 m/s².
2. The gyro aided window held the attitude, but its world took an arbitrary scale at the first triangulation. In `2026-11-15_17-54-10` a non converged solve with 5 landmarks moved the newest position by 0.32 m against 1 cm of ground truth on the ground, the finite difference velocity seed amplified it to 3.76 m/s, and the world was 4.2 to 5.3 times metric.
3. The prior of a visual window holds no scale information, so every installed coarse result set the world's scale directly, and coarse results passing the variance gate at the edge of its threshold applied −1.31, −1.40 and +1.74 in log scale where ground truth implied −0.05, +0.95 and −1.25.

Each of these needs its own repair, while the visual inertial start has none of them.

### 1.4 What the stationary period offers and what the estimator wasted

At rest the accelerometer measures $\mathbf{f} = -\mathbf{R}^\top \mathbf{g} + \mathbf{b}_a$ and the gyroscope $\boldsymbol{\omega} = \mathbf{b}_g$, with the velocity zero and the pose constant. The gyroscope bias is observable in all three axes, about 0.001 rad/s after 4 s at the SITL gyroscope noise density of 0.0023 rad/s/√Hz. The accelerometer bias is observable along gravity, about 0.04 m/s² after 4 s at 0.08 m/s²/√Hz. The horizontal accelerometer bias is not observable, because a tilt error $\delta\boldsymbol{\theta}$ produces the same horizontal specific force $|\mathbf{g}|\,\delta\boldsymbol{\theta}$ (`context/vio_optical_flow_instrumentation.md`, the tilt against accelerometer bias confound). A tilt known to within 1° bounds that bias by $9.81 \sin 1° = 0.17$ m/s², and the gravity prior `vio_gravity_prior_std_deg` is that bound.

The committed estimator applied the zero velocity residual to the newest state only and never marginalised it (`optimize` alone added it), so with a window of three states the prior kept about 0.1 s of the stationary period's information. Nothing held the poses on the ground, where the window has no landmark.

### 1.5 The first flight of the warm start

The flight `/ws/logs/slam/2026-11-15_18-34-59` (59 s, the SITL configuration of Change 1) froze the estimate on the ground. Ground truth climbed to 17 m while the estimated position stayed within 0.7 m of the origin, and from 15 s the cumulative scale fell to zero. The stationary flag itself behaved as designed. It was set from 0.05 s to 3.58 s, cleared at 3.63 s and 3.69 s by the optical flow stage, set again at 3.74 s, and cleared for good from 3.79 s, when the start latch closed after three moving frames.

The flag at 3.74 s was wrong. The specific force had risen from 9.84 to 10.03, 10.20 and 10.30 m/s² over the three frames from 3.63 s, a climb acceleration of up to 0.5 m/s², but the SHOE statistic averages a 0.5 s window and stayed below its threshold until 3.79 s, and the flow fell to 0.37 px, below its 0.8 px veto. Integrating the specific force less its rest value gives a true velocity of about 0.07 m/s at 3.74 s. The zero velocity row of weight 100, a standard deviation of 0.01 m/s, held that state at rest, and since the row entered the prior when the state left the window, the error became permanent. The estimated vertical velocity was 0.03 m/s at 3.79 s and 0.19 m/s at 4.21 s, against about 0.10 and 0.28 m/s from the integrated specific force, and the window's states followed the inertial prediction exactly in this interval, with a zero inertial cost and no landmark.

The deficit decided the first triangulation. At 4.26 s, 37 landmarks were triangulated over an estimated baseline of 7.2 cm against about 12 cm of ground truth, so their depths were short by about 40 %. The next solve moved the states against the inertial prediction, with an inertial cost of 125 and a visual cost of 710, and the coarse installations every three frames, which fit the keyframe velocities to the held keyframe positions, set the window's vertical velocity to within 0.01 m/s of zero at 4.47, 4.63 and 4.78 s. The reprojection error before each solve grew from 5 px to 65 px by 5.16 s, coherent across the landmarks (median equal to mean), so the predictions lagged the images by a common motion. With the estimate held, every candidate track had an estimated baseline below 1.7 cm, the 5 cm minimum of `vio_min_triangulation_dist` rejected all of them, no landmark was added after 4.26 s, and of the 37 landmarks only 20 remained from 5 s on. The visual inertial run `/ws/logs/slam/2026-11-15_18-12-31` of the removed work, which marginalised no stationary row, had a faster lift off and triangulated 60 to 170 landmarks per frame from 4.2 s.

Before this change the zero velocity row acted on the newest state for one solve only, so a single wrong flag was overridden by the next frames. Marginalising the rows makes every flag permanent once its state leaves the window, which is why a flag must be confirmed until then. (Note, 2026-11-15. The wrong flag was a real defect but not the cause of the frozen estimate, since the next flight froze without it, §1.6.)

### 1.6 The second flight, the rigidity of the marginalised zero velocity rows

The flight `/ws/logs/slam/2026-11-15_20-08-50` (22 s), with the clearing of Change 2 in a `libbasalt.so` built minutes before it, froze in the same way without any wrong flag. The flag was set from 0.11 s and cleared once at 3.59 s. Until the first triangulation the estimate followed the inertial prediction closely, 0.10 m and 0.37 m/s at 4.07 s against about 0.12 m from the integrated specific force. At 4.07 s, 74 landmarks were triangulated, and the solve's cost before its first step was about 3800, almost all visual, so the new landmarks were grossly inconsistent with the window's poses although each was triangulated from two of them. After the solve the visual cost was still 524, against 29 at the first triangulation of `/ws/logs/slam/2026-11-15_18-12-31`, and the estimated height stayed at 0.10 to 0.12 m from 4.07 s to 4.18 s while ground truth rose by 6 cm and the inertial cost rose to 114. The coarse installations then lowered the window's velocity at every installation as in §1.5, and the estimate never recovered.

Ground truth shows that the vehicle was not still at the centimetre level while the flag was set. In `2026-11-15_20-08-50` the body sank by 5 cm between 0.4 s and 3.5 s, in `2026-11-15_18-39-15` it slid by 3.6 cm in x, and in `2026-11-15_18-34-59` it moved by 1 to 2 cm, the settling of a vehicle spinning up on its landing gear. The estimate held every ground position at 0.000 m in all three runs, whereas the run `2026-11-15_18-12-31`, which marginalised no stationary row, let them drift by up to 8 cm. Neither tracked the slide, but only the new runs made the error rigid. With a focal length of 320 px and ground features at 0.34 to 0.58 m, a centimetre of pose error between two ground keyframes is 6 to 9 px of reprojection error against an observation standard deviation of 0.5 px.

The rigidity comes from the zero velocity rows. Each claims $\mathbf{v}_k = \mathbf{0}$ with standard deviation $\sigma_v = 0.01$ m/s, and marginalised over the $N pprox 70$ states of a 3.5 s period they hold the mean velocity with standard deviation

$$rac{\sigma_v}{\sqrt{N}} pprox 1.2 	ext{ mm/s},$$

which bounds the drift over the period to a few millimetres, as if the errors of the rows were independent. The true error is a slow correlated slide of about 1 cm/s, nine or more of these standard deviations. The zero motion rows instead form a random walk. With $\sigma_p = 5$ mm per step the relative position of two states $n$ steps apart has standard deviation $\sigma_p \sqrt{n}$, about 4 cm over the period, which accommodates the slide. Change 3 therefore marginalises the zero motion rows only, and the zero velocity rows act on the window's flagged states as the committed code's row did on the newest one.

### 1.7 The third flight, the reset at the first moving frame

The flight `/ws/logs/slam/2026-11-15_20-22-22` (40 s), with the zero velocity rows of §1.6 no longer marginalised and a `libbasalt.so` built minutes before it, froze again. The flag was set from 0.11 s and cleared once at 3.54 s, and the ground positions now drifted by a few millimetres rather than staying at 0.000 m. The estimate followed the lift off until 3.91 s, 0.07 m and 0.32 m/s against 0.06 m of ground truth. The first triangulation then added only 8 landmarks, whose solve ended with a visual cost of 118 and an inertial cost of 55. From 3.96 s the coarse installations every three frames, whose cost before the solve rose from 3.8 at rest to 30 to 48 after lift off, lowered the vertical velocity at each installation, from 0.27 to 0.09 m/s at 4.11 s and to about 0.08 m/s from 4.4 s on. The main prior's cost fell to about −1000, 7 landmarks remained, no candidate reached the 5 cm triangulation baseline, and the estimate was at 0.10 m when ground truth was at 1.5 m, and at 1.3 m when it was at 16 m.

Every run since the stationary rows reached the prior has frozen in this way, and the common element is the information the stationary period leaves in the prior, the ground keyframes held together and the inertial chain from them, against which the first landmarks and the coarse installations compound a lag of the window. The reset of Change 5 therefore keeps, at the end of a stationary run, only the marginal of the prior on the last stationary state, its pose, velocity and biases with their correlation with gravity, and removes the ground keyframes from the window and the store.

## 2. Design

1. The estimator stays visual inertial from the first frame.
2. Every frame state created while the stationary detector reports rest is flagged. A zero velocity residual acts on every flagged state, and a zero motion residual, identity relative rotation and zero translation, acts between consecutive flagged states.
3. Each stationary factor is marginalised into the prior and into the delayed store's step when its first state leaves the window, so the whole stationary period reaches the prior. At lift off the prior holds the gyroscope bias, the vertical accelerometer bias and the combined tilt and horizontal bias under the gravity prior, a warm start from which the window continues without a switch. The start latch of the stationary detector ends the residuals at lift off.
4. The coarse optimisation's one bias starts at the window's estimate and carries a prior centred on it, so that the warm start is refined rather than replaced by DM-VIO's fit from zero.
5. The coarse phase counts its delay from the end of the stationary period, and the store keeps the last stationary step together with the delay's moving steps, so that delayed marginalisation always holds the last stationary state and the take off.
6. Three improvements of the period made during the rejected work are kept. The settled fold ends the period once the scale and gravity have been still over $K$ marginalisations of the delayed phase. A coarse installation rescales the window's landmarks. The SITL variance gate and random walks are retuned.

## 3. Changes

### Change 1. Configuration keys, `include/basalt/utils/vio_config.h`, `src/utils/vio_config.cpp`, `data/*.json`, `data/iccv21/basalt_batch_config.toml`

Eight keys, inert at their defaults in every file except `data/sitl_config_vo.json`.

| Key | Default | SITL | Meaning |
|---|---|---|---|
| `vio_zero_motion_pos_std` | 0 | 0.005 m | standard deviation of the translation between consecutive stationary states, 0 disables |
| `vio_zero_motion_rot_std_deg` | 0 | 0.05° | the same for the relative rotation |
| `vio_gs_dm_delay_from_motion` | false | true | count `vio_gs_dm_delay` from the end of the stationary period and keep its last step in the store |
| `vio_gs_dm_coarse_bg_prior_std` | 0 | 0.002 rad/s | coarse optimisation's gyroscope bias prior, 0 disables |
| `vio_gs_dm_coarse_ba_prior_std` | 0 | 0.1 m/s² | coarse optimisation's accelerometer bias prior, 0 disables |
| `vio_gs_dm_fold_settle_scale` | 1.005 | 1.005 | fold band in scale, a ratio |
| `vio_gs_dm_fold_settle_tilt_deg` | 0.1 | 0.1 | fold band in gravity direction |
| `vio_gs_dm_fold_settle_count` | 0 | 185 | marginalisations of stillness before the fold, 0 disables |

The SITL values follow from the noise densities. A frame's preintegrated rotation has a standard deviation of $0.0023 \sqrt{0.054} = 0.03°$, so 0.05° keeps the zero motion rotation row of the same order as the gyroscope. 0.002 rad/s is twice the gyroscope bias precision of a 4 s stationary period, and 0.1 m/s² lies between the vertical precision and the 0.17 m/s² bound of the horizontal bias. The fold values are those derived from the flights of 2026-11-13 in `plans/delayed_marginalisation.md` §5. The SITL file also sets `vio_gs_dm_coarse_scale_var` to $2.5 \times 10^{-3}$, `vio_gs_jump_sigma_rw` to 0.15 and `vio_gs_jump_tilt_rw_deg` to 0.3, after §5.8 items 1, 4 and 6 there.

Backwards compatibility. Every other configuration keeps its behaviour except for the zero velocity change of Change 2.

### Change 2. Stationary residuals, `SqrtKeypointVioEstimator::measure`, `optimize` and `StationaryRows`, `src/vi_estimator/sqrt_keypoint_vio.cpp` and `include/basalt/vi_estimator/sqrt_keypoint_vio.h`

`measure` inserts the new state's timestamp into `mvpStationaryStates` when `mpIsStationary` is set, which `ProcessFrame` computes before it calls `measure`, and clears the whole set when it is not. A flagged state's rows become permanent only when the state leaves the window, so a non stationary frame withdraws the flags of every state still in the window, and a state's rows reach the prior only if the detector reported rest from its creation until its departure, about three frames later. This absorbs the lag of the detector's averaging window and a single stationary frame between moving ones, §1.5. (Note, 2026-11-15. This paragraph previously stated only the insertion. Without the clearing, the first flight of the change pinned a state of the lift off to zero velocity in the prior, §1.5.) `StationaryRows(aom, leaving, J, r)` builds, over the columns of `aom`, the rows

$$\mathbf{r}_{v,k} = w_v\,\mathbf{v}_k,$$

$$\mathbf{r}_{p,ab} = w_p\,(\mathbf{p}_b - \mathbf{p}_a),$$

$$\mathbf{r}_{R,ab} = w_r\,\text{Log}(\mathbf{R}_a^\top \mathbf{R}_b),$$

for every flagged state $k$ and every pair of consecutive flagged states $a$ and $b$ in `frame_states`, with $w_v$ = `vio_zero_velocity_weight`, $w_p$ the inverse of `vio_zero_motion_pos_std` and $w_r$ the inverse of `vio_zero_motion_rot_std_deg` in radians. Under the left perturbation $\mathbf{R} \leftarrow \text{Exp}(\delta\boldsymbol{\varphi})\,\mathbf{R}$ of `PoseState::applyInc`,

$$\mathbf{R}_a^\top \text{Exp}(\delta\boldsymbol{\varphi}_b)\,\mathbf{R}_b = \text{Exp}(\mathbf{R}_a^\top \delta\boldsymbol{\varphi}_b)\,\mathbf{R}_a^\top \mathbf{R}_b,$$

so with $\mathbf{R}_a^\top \mathbf{R}_b$ near the identity the rotation row has the Jacobians $w_r \mathbf{R}_a^\top$ in $\delta\boldsymbol{\varphi}_b$ and $-w_r \mathbf{R}_a^\top$ in $\delta\boldsymbol{\varphi}_a$, taken at the linearisation point. With `leaving` given, only the zero motion rows whose first state leaves are built. (Note, 2026-11-15. This sentence previously included the zero velocity rows of leaving states, §1.6.)

`optimize` replaces the zero velocity code on the newest state by these rows on every flagged state. With $\mathbf{J}_s$ and $\mathbf{r}_s$ the stacked rows and $\Delta$ the applied increment,

$$E_s = \frac{1}{2}|\mathbf{r}_s|^2$$

enters `error_total`, $\mathbf{J}_s^\top \mathbf{J}_s$ and $\mathbf{J}_s^\top \mathbf{r}_s$ enter $\mathbf{H}$ and $\mathbf{b}$, the model decrease $(\mathbf{J}_s \Delta)^\top(\mathbf{r}_s + \frac{1}{2}\mathbf{J}_s \Delta)$ is subtracted from `l_diff`, and the rows rebuilt at the updated values enter `after_error_total`.

Backwards compatibility. With `vio_zero_velocity_weight` positive and the zero motion keys at zero, the zero velocity residual now acts on every flagged state of the window rather than on the newest one, and is not marginalised. (Note, 2026-11-15. This sentence previously stated that Change 3 marginalises it, §1.6.) This changes the behaviour of every configuration with a positive zero velocity weight, which is the intended semantics.

### Change 3. Marginalisation of the stationary rows, `SqrtKeypointVioEstimator::marginalize`

After the marginalisation's linearisation, the zero motion rows whose first state leaves, the states of `states_to_marg_all` and `states_to_marg_vel_bias`, are appended to the square root system, or added to the dense system when the prior is not in square root form, and with the period active to the store's visual rows of the step, which are then converted to delta independent form and compressed as before. Each factor is thereby counted once, in the window while its first state is there and in the prior and the store afterwards, since the window's later solves no longer see a state that has left. Flags of states that left `frame_states` are erased. The marginalisation packet to the local mapper includes the rows. The zero velocity rows are not marginalised, because over a whole stationary period they would hold the ground poses to a few millimetres while the vehicle slides by centimetres, §1.6. (Note, 2026-11-15. This change previously marginalised the zero velocity rows as well, which froze the estimate after lift off in the flights of §1.5 and §1.6.)

The zero targets are invariant under a re-centring, so only the rows' weights change by the scale of a correction, which at rest is negligible.

### Change 4. The coarse optimisation's bias prior, `include/basalt/vi_estimator/delayed_marginalisation.h` and `MakeDelayedJob`

`DelayedMargJob` gains `mBgPrior`, `mBaPrior`, `mBgPriorW` and `mBaPriorW`. With a positive weight the bias starts at the prior's mean instead of zero, and the residual $\mathbf{b} - \bar{\mathbf{b}}$ with information $w$ enters the cost, the diagonal of columns 3 to 5 or 6 to 8, and the gradient. `MakeDelayedJob` sets the means to the newest window state's biases and the weights to the inverse squares of the two keys.

Backwards compatibility. With both keys at zero the solve is DM-VIO's, from zero without a prior.

### Change 5. The delay and the reset at the end of the stationary period, `ProcessFrame`, `UpdateDelayedMarg`, `SqrtKeypointVioEstimator::ResetPriorToLastState` and `DelayedMarginalisation::Reset`

`ProcessFrame` reads `mpDmMotionIndex = mpDm->NextIndex()` where it sets the period's debounced motion start `mpGsStartNs`, and the coarse phase ends at $\text{NextIndex} \ge d + n_\text{motion}$ with `vio_gs_dm_delay_from_motion`, and at $d$ otherwise.

With the key set, `ProcessFrame` also sets `mpDmResetPending` on every non stationary frame whose predecessor was stationary (`mpWasStationary`). `UpdateDelayedMarg` then discards the step instead of pushing it and calls `ResetPriorToLastState(step.mIndex + 1)`, which

1. waits for and drops the job in flight,
2. marginalises every keyframe pose column out of `marg_data` with `MargHelper::marginalizeHelperSqrtToSqrt`, keeping the block's three columns and the fifteen of the one state the prior holds, `last_state_to_marg`, which at the first moving frame is the last stationary state,
3. sets `marg_data` to the result over the order [block, state],
4. erases every pose of `frame_poses` from `frame_poses`, `prev_opt_flow_res` and `kf_ids` and removes the landmarks they host and their observations with `LandmarkDatabase::removeKeyframes`,
5. restarts the store with `DelayedMarginalisation::Reset(boundary, nextIndex)`, whose boundary is the new prior anchored at the current block as the period's first boundary is, discarding the steps, records, keyframes and keyframe chain while the indices continue.

```cpp
std::set<int> idxKeep, idxMarg;
int64_t stateId = -1;
for (const auto& [id, idx] : marg_data.order.abs_order_map) {
    std::set<int>& s = idx.second == POSE_SIZE ? idxMarg : idxKeep;
    for (int i = 0; i < idx.second; i++) s.insert(idx.first + i);
    if (idx.second == POSE_VEL_BIAS_SIZE) stateId = id;
}
MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(H, b, idxKeep, idxMarg,
                                                HNew, bNew);
```

The prior arrives delta independent, $\frac{1}{2}|\mathbf{H}\,\mathbf{z} + \mathbf{b}|^2$ in the offsets $\mathbf{z}$ of its variables from their linearisation points, so eliminating the pose offsets gives the exact marginal in the same form, and the next step's order begins with [block, state] as every window order begins with the prior's. The window continues with the last stationary state and the first moving one, with no keyframe, and takes its next keyframes as it does after initialisation, and the coarse optimisation resumes once the store knows more than five keyframes again. A stationary flicker resets the prior once more at its next moving frame, which drops only the keyframes taken since. The ground keyframes are never sent to the local mapper.

Backwards compatibility. With the key false, nothing is reset and the end condition is as committed. (Note, 2026-11-15. This change previously kept the last stationary step in the store beyond the delay through `AdvanceBoundary`'s `keepFromIndex` and let the older stationary steps reach $B$ with all of their rows, §1.5 to §1.7. `keepFromIndex` remains with its default for its unit test.)

### Change 6. Kept improvements of the period

1. The settled fold. `DelayedMargBlockSettled` in `delayed_marginalisation.h` tests whether the scale and gravity direction changed by at most $\ln$ `vio_gs_dm_fold_settle_scale` and `vio_gs_dm_fold_settle_tilt_deg` between each of the last $K + 1$ epochs of the delayed phase and the latest, and `marginalize` folds when it holds or when `vio_gs_duration_s` has elapsed. With $K = 0$ the fold is as committed.
2. The landmark rescale. `InstallCoarseJob` sets the block to the estimated scale before the re-centring, which changes every relative translation of the visual residual, so it now divides every window landmark's inverse distance by the installed scale, as `LocalMapper::ApplyGravityScaleCorrection` already does for the mapper's landmarks. This changes the committed coarse installation, which left the landmarks inconsistent with the rescaled poses.
3. The record. `vio_delayed_marg` gains `motion_index` and `fold_settled`, and the dashboard plots the fold test.

### Change 7. Unit tests, `test/src/test_vio.cpp`

`DelayedMargStoreKeepFromTest` checks that a store of seven steps with a delay of two keeps two steps without `keepFromIndex` and three from index 4 with it. `DelayedMargCoarseBiasPriorTest` checks that a tight prior holds the coarse bias at its mean and that without it the bias is estimated. `DelayedMargBlockSettledTest` checks the fold test. `DelayedMargStoreResetTest` checks that a reset leaves the store with the given boundary, no step, record, keyframe or keyframe span, and the next index unchanged.

### Change 8. Documentation

`doc/VIO.md` gains §3.2.8 on the stationary residuals and the warm start, and §3.3.4 and §3.3.5 the delay from the end of the stationary period, the coarse bias prior, the landmark rescale and the settled fold.

## 4. Blast radius

| Symbol | Consumers | Effect |
|---|---|---|
| `VioConfig` | every `data/*.json`, the batch `.toml` | eight new fields |
| `SqrtKeypointVioEstimator::optimize` | the VIO thread | zero velocity on every flagged state, zero motion rows when enabled |
| `SqrtKeypointVioEstimator::marginalize` | the VIO thread, the local mapper through `out_marg_queue` | stationary rows in the prior, the store and the packet |
| `DelayedMarginalisation::AdvanceBoundary` | `UpdateDelayedMarg` | defaulted argument |
| `DelayedMarginalisation::Reset` | `ResetPriorToLastState` | new |
| `SqrtKeypointVioEstimator::ResetPriorToLastState` | `UpdateDelayedMarg` with `vio_gs_dm_delay_from_motion` | prior marginalised to the last stationary state, ground keyframes and their landmarks dropped, store restarted |
| `DelayedMargJob::Coarse` | the coarse job | prior when enabled |
| `InstallCoarseJob` | the coarse installation | landmarks rescaled |
| `Logger::AddVioDelayedMarg` | one caller, the dashboard | two appended columns |

## 5. Validation

1. Run on 2026-11-15 in the dev container. `-fsyntax-only` of `sqrt_keypoint_vio.cpp`, `logger.cpp` and `vio_config.cpp` under GCC 12.2 with the vendored Eigen 3.4.0, with only the existing unused parameter warnings of `optical_flow.h`. Ten tests extracted from `test/src/test_vio.cpp` and linked against `marg_helper.cpp` and googletest passed, the three of Change 7 and the existing `ImuBlockGSScaleFreeRowsTest`, `DelayedMargReadvanceJointTest`, `DelayedMargSmoothingJointTest`, `DelayedMargRecentreInvarianceTest`, `DelayedMargCoarseScaleTest`, `DelayedMargCoarseStaticTest` and `DelayedMargCoarseDeadlineTest`.
2. Outstanding on the host. `colcon build --packages-select slam` and the full Basalt unit tests.
3. The flight of §1.5 led to the clearing of Change 2, checked on 2026-11-15 with `-fsyntax-only` of `sqrt_keypoint_vio.cpp`. A replay of `/ws/logs/slam/2026-11-15_18-34-59`'s bag should show no flagged state after the first non stationary frame and an estimated vertical velocity within a few centimetres per second of the integrated specific force at 4.2 s.
4. The flight of §1.6 led to the removal of the zero velocity rows from Change 3, checked on 2026-11-15 with `-fsyntax-only` of `sqrt_keypoint_vio.cpp`. The next flight should show ground positions that drift by about the true settling, a visual cost at the first triangulation of the order of the 29 of `2026-11-15_18-12-31`, and an estimated height that follows ground truth through lift off. If it still freezes, the next suspect is the coarse installation, which sets the window's velocities from the held keyframe positions, §1.5.
5. The flight of §1.7 led to the reset of Change 5, checked on 2026-11-15 with `-fsyntax-only` of `sqrt_keypoint_vio.cpp` and eleven tests of the harness, the new `DelayedMargStoreResetTest` among them. The next flight should show `frame_poses` at zero in `vio_marg` at the first moving frame, a first triangulation without stationary keyframes, and an estimated height that follows ground truth through lift off.
6. Outstanding replays of `/ws/data/telemetry_35` and `/ws/data/telemetry_36`. In SITL the inertial measurements carry no generated bias, so the biases at lift off should be near zero, the gyroscope bias within about 0.002 rad/s and the vertical accelerometer bias within about 0.05 m/s². Compare the scale ratio and tilt error of the first 30 s after lift off with the two runs of §1.2, with and without the zero motion rows, and with the period disabled through `vio_gs_enable`, since a warm started window may not need it.

## 6. Decisions

1. A start that is not stationary falls back on the coarse initialiser, which remains in place.
2. The horizontal accelerometer bias is separated from the tilt only by rotation in flight, so a platform that never rotates relies on the bias prior throughout.
