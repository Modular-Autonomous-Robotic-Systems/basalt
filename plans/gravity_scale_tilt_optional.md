# Optional Gravity Direction in the Gravity and Scale Period

Removed 2026-11-13. `plans/delayed_marginalisation.md` Change 12 removed the tilt hold and `vio_gs_optimise_gravity_dir`, because delayed marginalisation became the only mechanism of the period and the gravity prior constrains the tilt softly (§2.3.12 and §2.3.13 there). The record below describes the hold as it was implemented on 2026-11-11. Note, 2026-11-13. This line previously read only "Implemented 2026-11-11". Every `file:line` anchor was read against the working tree of branch `shreyas/kf_driven_mapping_and_logger` on top of `7e91ec4`, which holds the changes of `plans/gravity_scale_optimisation.md` (plan 1) and `plans/gravity_scale_optimisation_2.md` (plan 2). The changes of `plans/delayed_marginalisation.md` are not in the working tree and are not assumed.

## 1. Context

### 1.1 The question

During the gravity and scale period the estimator carries a three dimensional block $\boldsymbol{\delta}_{gs} = [\delta\theta_1, \delta\theta_2, \delta\sigma]^\top$, two tilt coordinates of the gravity direction $\mathbf{d}$ and the log scale $\sigma$ (`include/basalt/utils/imu_types.h:323-391`). The question is how the estimator behaves when only $\sigma$ is free and $\mathbf{d}$ is held at its initial alignment, and the request is a configuration flag that makes the gravity direction optional with the existing machinery left in place.

### 1.2 A correction to the premise

The request describes the original dynamic marginalisation as optimising scale alone. VI-DSO in fact optimises both, through a translation free similarity $\mathbf{T}_{m\_d}$ between the visual and metric frames, "making it independent of the scale and gravity direction" [1, §III-E], and DM-VIO likewise optimises scale and gravity direction "as explicit variables" [2, §III-B] (`context/dynamic_marginalisation.md` §3.2.1 and §3.3). What is scale only in VI-DSO is the dynamic marginalisation itself, whose interval and swaps track the scale and not the gravity direction. The working tree already follows that rule, since `UpdateScaleInterval` reads $\sigma$ alone (`src/vi_estimator/sqrt_keypoint_vio.cpp:2404-2438`). The flag proposed here therefore does not reproduce VI-DSO. It produces a variant in which the world stays gravity aligned, as in Basalt outside the period, while the scale remains a variable.

### 1.3 Why holding the tilt is well posed in Basalt

Plan 1 §2.5.3 shows that the block adds three gauge directions to the problem, the scale direction $\mathbf{n}_\sigma$ and two tilt directions $\mathbf{n}_{\mathbf{e}_x}$, $\mathbf{n}_{\mathbf{e}_y}$, which rotate every frame and the gravity direction together. Physically the roll and pitch of the body against gravity remain observable through the frame attitudes and the inertial factors, and only their split between the frames and the block is undetermined. Plan 1 §2.5.4 item 4 shows that holding the block's coordinates is a valid gauge fixation, with $\mathbf{S}_{gs} \mathbf{N}$ a signed permutation matrix. Restricted to the two tilt rows, the same selection gives

$$\mathbf{S}_\theta \mathbf{N} = \begin{bmatrix}0 & \bar{\mathbf{b}}_1^\top \mathbf{e}_x & \bar{\mathbf{b}}_1^\top \mathbf{e}_y\\ 0 & \bar{\mathbf{b}}_2^\top \mathbf{e}_x & \bar{\mathbf{b}}_2^\top \mathbf{e}_y\end{bmatrix},$$

which at $\bar{\mathbf{B}} = [\mathbf{e}_x, \mathbf{e}_y]$ fixes both tilt directions with perfect conditioning and leaves $\mathbf{n}_\sigma$ untouched, since the block row of $\mathbf{n}_\sigma$ has zeros in the tilt coordinates. Holding the tilt and projecting out $\mathbf{n}_\sigma$ alone is therefore a consistent treatment of the gauge. Plan 1 rejected item 4 for the whole block because holding $\sigma$ returns the scale to root cause R5. Holding only the tilt keeps the scale fix and returns only the tilt to today's treatment, which is precisely the experiment requested. `context/dynamic_marginalisation.md` §4.5.4 proposed holding the whole block until it carries information, and this plan is the tilt half of that idea applied for the whole period.

## 2. Design

### 2.1 Key insight

The tilt can be held without touching the linearisation, the priors, the swap rule or the re-centring. The block keeps its three columns everywhere, and only the dense solve of `optimize` is told that the two tilt coordinates are not variables. Restricting a quadratic model to the subspace $\boldsymbol{\delta}_\theta = \mathbf{0}$ is the same as deleting the corresponding rows and columns of $\mathbf{H}$ and entries of $\mathbf{b}$, which is done in place by zeroing them and putting a unit on the diagonal. The solve then returns an exact zero tilt step, so every other mechanism sees a block whose direction never moves.

### 2.2 What follows from a block whose direction never moves

1. The gravity direction is initialised to $\bar{\mathbf{d}} = (0, 0, -1)^\top$ (`imu_types.h:335`) and `g` is `basalt::constants::g` $= (0, 0, -9.81)^\top$ (`src/controller.cpp:194`), so $\mathbf{d} = \hat{\mathbf{g}}$ exactly at every frame.
2. `AlignmentRotation` is `FromTwoVectors` of two equal vectors, which Eigen returns as the exact identity, so the re-centring rotation $\mathbf{Q}$ of `RecentreGravityScale` (`sqrt_keypoint_vio.cpp:2324-2376`) is the identity and the re-centring reduces to the scale map. Every correction log entry is then a pure scale, and the local mapper receives scale only corrections through the unchanged path.
3. Every anchor stores $\mathbf{d} = \hat{\mathbf{g}}$, so `Reanchor` computes a zero rotation axis and a zero tilt delta (`imu_types.h:372-388`). The tilt columns of $M_\text{curr}$, $M_\text{half}$ and $M_\text{visual}$ stay in the priors and multiply a zero offset forever, which contributes nothing to their residuals and nothing to the masked solve.
4. Change B's tilt random walk adds to the tilt diagonal before the mask removes it, and its cost $\frac{1}{2} w\,|\boldsymbol{\delta}_\theta|^2$ is identically zero, so `vio_gs_jump_tilt_rw_deg` becomes inert without a change.
5. The tilt step of `vio_gravity_scale.csv` is $\text{acos}(\mathbf{d} \cdot \hat{\mathbf{g}}) = 0$ at every frame, which is the runtime check that the flag took effect.

### 2.3 The gauge projector

With the tilt held, $\mathbf{n}_{\mathbf{e}_x}$ and $\mathbf{n}_{\mathbf{e}_y}$ are no longer null directions of the masked system, because rotating the frames without the gravity direction changes every inertial residual (plan 1 §2.5.3, second paragraph). They must leave the projector, or the projection would remove physical tilt corrections of the frames. The projector becomes

$$\mathbf{P} = \mathbf{I} - \frac{\mathbf{n}_\sigma \mathbf{n}_\sigma^\top}{\mathbf{n}_\sigma^\top \mathbf{n}_\sigma}.$$

The tilt rows of $\mathbf{n}_\sigma$ are zero, so the tilt rows and columns of $\mathbf{P}$ are those of the identity, and the projection and the mask commute. The order in which they are applied therefore does not matter, and the projected step keeps an exact zero in the tilt coordinates.

### 2.4 Expected change in behaviour, stated as hypotheses to be measured

1. The tilt is carried by the frame attitudes against a fixed gravity, as outside the period. Corrections to the tilt of frames already frozen in a prior accumulate in their offsets from first estimates, which is R5 for the tilt (plan 1 §2.5.4 item 4). This is second order in the correction and is expected to be small when the steps are a degree or less, which Change B's sigma random walk and the initial alignment accurate to about 0.58° (plan 2 §3.2.3) suggest.
2. The tilt runaway through the block is removed. The recorded tilt divergences, 26° to 27° in the first replays and the run `_23-18-44` whose tilt ran away without a swap (`context/dynamic_marginalisation.md` §1.2 and §5), moved the block's tilt through many bounded steps. With the block held, a tilt drift can only appear in the frame attitudes, where the inertial factors of the window observe it directly.
3. The tilt and horizontal accelerometer bias null direction at constant attitude (plan 2 §3.2.3) is a property of the frames and the bias, not of the block, and survives the flag. Change A, which keeps the bias anchor through a swap, remains relevant.
4. The scale is still a variable, the swap rule still tracks it, and the re-centring still applies it exactly, so the scale correction capability is kept. Whether removing two coupled coordinates reduces the take off steps of $\sigma$, which reached $-0.56$ in one solve before plan 2, is the main question of the experiment.
5. The cost per frame does not change measurably, a $1 \times 1$ projector replacing a $3 \times 3$ one.

## 3. Implementation

Two edits of the estimator, one key, and eight edits of configuration. With the flag at its default `true`, every added statement sits behind a branch that is not taken, and the default path is byte identical to today's.

### Change 1. Configuration key, `include/basalt/utils/vio_config.h` and `src/utils/vio_config.cpp`

Need. A key that holds the tilt, defaulting to today's behaviour.

`include/basalt/utils/vio_config.h`, after `vio_gs_jump_tilt_rw_deg` (line 144).

```cpp
    double vio_gs_jump_sigma_rw;
    double vio_gs_jump_tilt_rw_deg;
    // False holds the gravity direction of the block at its initial
    // alignment, leaving the log scale as its only free coordinate.
    bool vio_gs_optimise_gravity_dir;
```

`src/utils/vio_config.cpp`, the default after line 107 and the serialisation after line 249.

```cpp
  vio_gs_jump_tilt_rw_deg = 0.0;
  vio_gs_optimise_gravity_dir = true;
```

```cpp
  ar(CEREAL_NVP(config.vio_gs_jump_tilt_rw_deg));
  ar(CEREAL_NVP(config.vio_gs_optimise_gravity_dir));
```

Backwards compatibility. The default reproduces today's behaviour. A key read by `serialize` and absent from a file is fatal (`context/vio_config_json.md`), so Change 4 adds it to every configuration file in the same change.

### Change 2. Scale only projector, `ComputeGaugeProjector` in `src/vi_estimator/sqrt_keypoint_vio.cpp:2279-2322`

Need. §2.3. The tilt directions must leave the projector when the tilt is held.

The matrix $\mathbf{N}$ is built exactly as today, and an early return before the $3 \times 3$ solve uses its first column, which is $\mathbf{n}_\sigma$. Leaving the existing fixed size `NtN` path untouched keeps the default path's floating point results identical.

```cpp
        for (int k = 0; k < 2; k++) {
            const Vec3 a = Vec3::Unit(k);
            N.template block<3, 1>(c, 1 + k) = a.cross(p);
            N.template block<3, 1>(c + 3, 1 + k) = a;
            if (has_vel) N.template block<3, 1>(c + 6, 1 + k) = a.cross(v);
        }
    }

    if (!config.vio_gs_optimise_gravity_dir) {
        const VecX nSigma = N.col(0);
        return MatX::Identity(n, n) -
               nSigma * nSigma.transpose() / nSigma.squaredNorm();
    }

    const Eigen::Matrix<Scalar, 3, 3> NtN = N.transpose() * N;
    return MatX::Identity(n, n) - N * NtN.ldlt().solve(N.transpose());
```

$\mathbf{n}_\sigma^\top \mathbf{n}_\sigma \geq 1$ because its block entry is $-1$, so the division is always defined.

Backwards compatibility. `ComputeGaugeProjector` has one caller, `optimize` (`sqrt_keypoint_vio.cpp:1908`). With the flag true the early return is not taken.

### Change 3. Holding the tilt in the dense solve, `optimize` in `src/vi_estimator/sqrt_keypoint_vio.cpp:1906-1911`

Need. §2.1. The two tilt coordinates are removed from the solve so that the step on them is exactly zero.

```cpp
                    MatX gauge_P;
                    if (mpGsActive) {
                        gauge_P = ComputeGaugeProjector(aom);
                        H = gauge_P * H * gauge_P;
                        b = gauge_P * b;
                        // Hold the tilt, plans/gravity_scale_tilt_optional.md.
                        if (!config.vio_gs_optimise_gravity_dir) {
                            const int gs =
                                aom.abs_order_map.at(kGravityScaleStateId)
                                    .first;
                            H.middleRows(gs, 2).setZero();
                            H.middleCols(gs, 2).setZero();
                            H.diagonal().template segment<2>(gs).setOnes();
                            b.template segment<2>(gs).setZero();
                        }
                    }
```

Mathematical background. Write the step as $\boldsymbol{\delta} = [\boldsymbol{\delta}_\theta^\top, \boldsymbol{\delta}_r^\top]^\top$ and the damped model as $\frac{1}{2} \boldsymbol{\delta}^\top \mathbf{H} \boldsymbol{\delta} - \mathbf{b}^\top \boldsymbol{\delta}$. On the subspace $\boldsymbol{\delta}_\theta = \mathbf{0}$ the model is $\frac{1}{2} \boldsymbol{\delta}_r^\top \mathbf{H}_{rr} \boldsymbol{\delta}_r - \mathbf{b}_r^\top \boldsymbol{\delta}_r$, whose minimiser solves $\mathbf{H}_{rr} \boldsymbol{\delta}_r = \mathbf{b}_r$. The masked matrix is block diagonal with blocks $\mathbf{I}_2$ and $\mathbf{H}_{rr}$ and the masked vector is $[\mathbf{0}^\top, \mathbf{b}_r^\top]^\top$, so its solution is $[\mathbf{0}^\top, (\mathbf{H}_{rr}^{-1} \mathbf{b}_r)^\top]^\top$. The Levenberg–Marquardt damping of `optimize` adds $\max(\lambda h_{ii}, \lambda_\text{min})$ to each diagonal entry and keeps the block diagonal structure. The decoupled rows are eliminated by the LDLT without fill, so the tilt entries of the step are an exact floating point zero, and `ApplyInc` with a zero tilt leaves $\bar{\mathbf{d}}$ and $\bar{\mathbf{B}}$ bit for bit unchanged (`imu_types.h:354-363`). This is conditioning on the held value and not marginalising it, which is the correct reading of a fixed variable.

Cost bookkeeping. `backSubstitute` evaluates the model decrease with the unmasked Jacobians at the returned step, whose tilt entries are zero, which equals the decrease of the restricted model. Change B's terms in `error_total`, `l_diff` and `after_error_total` multiply the tilt part of `gsJump`, which stays zero. The step acceptance therefore needs no change.

Backwards compatibility. Inside `if (mpGsActive)` and behind the flag, so outside the period and with the flag true nothing executes. Note, 2026-11-11. The mask shown above was generalised by `plans/gravity_scale_update_cadence.md` Change 3 into a count of held rows, `nHeld`, which is 2 for this flag on a free frame and 3 on a held frame of the cadence, and the projector is skipped on held frames. With `vio_gs_update_every_n_frames` at 1 the generalised code executes exactly the two row mask above.

### Change 4. Configuration files, `data/*.json` and `data/iccv21/basalt_batch_config.toml`

Need. Missing keys are fatal. A search of `/ws/ros_ws/src` and `/ws/data` found exactly eight files carrying `vio_gs_enable`, all under `ext/basalt/data`.

In each of `euroc_config.json`, `euroc_config_no_factors.json`, `euroc_config_no_weights.json`, `euroc_config_vo.json`, `kitti_config.json`, `sitl_config_vo.json` and `tumvi_512_config.json`, after line 55.

```json
        "config.vio_gs_jump_tilt_rw_deg": 0.0,
        "config.vio_gs_optimise_gravity_dir": true,
        "config.vio_marg_lost_landmarks": true,
```

In `data/iccv21/basalt_batch_config.toml`, after line 63.

```toml
"config.vio_gs_jump_tilt_rw_deg" = 0.0
"config.vio_gs_optimise_gravity_dir" = true
```

`sitl_config_vo.json` keeps `true`, so the configured behaviour of the replay and of live flights is unchanged. The experiment of §5.3 sets `false` in a scratch copy.

### Change 5. Documentation and context

1. `doc/VIO.md`, the section on the period around line 1041, one paragraph stating that the flag holds $\boldsymbol{\delta}_\theta$, that the projector then removes $\mathbf{n}_\sigma$ alone, and why the two commute.
2. `context/vio_config_json.md`, the key, its default and the eight files, appended to the paragraph on plan 2's keys.
3. `context/dynamic_marginalisation.md`, a new section recording the correction of §1.2, the fixation argument of §1.3, the consequences of §2.2 and the results of §5.3 once run.
4. `context/README.md`, the date and description of the rows of the two context files.

## 4. Blast radius

| Symbol | Consumers | Effect with the flag true | Effect with the flag false |
|---|---|---|---|
| `VioConfig::vio_gs_optimise_gravity_dir` | `ComputeGaugeProjector`, `optimize`, the eight configuration files | none | §2.2 |
| `ComputeGaugeProjector` | `optimize` only | identical | scale only projector |
| The dense solve of `optimize` | every frame of the period | identical | tilt rows decoupled |
| `RecentreGravityScale`, `UpdateScaleInterval`, `LinearizeMargGS`, `MarginalizeGS`, `FoldGravityScale`, `LocalMapper` | unchanged code | identical | receive a block with $\mathbf{d} = \hat{\mathbf{g}}$, a pure scale correction log |
| `plans/delayed_marginalisation.md` (not applied) | its readvance re-anchors the block | not applicable | compatible, the tilt anchor never moves |

## 5. Validation

### 5.1 Code reading, done 2026-11-11

1. With the flag true, both added branches are skipped and no other line of the estimator changed.
2. The only writers of the block's direction are `ApplyInc` (from `optimize`), `TransformLin` (from `RecentreGravityScale`) and `Reanchor` (from `AnchorGs`), at `imu_types.h:357-391`. With the flag false `ApplyInc` receives a zero tilt, `TransformLin` receives `FromTwoVectors` of $(0, 0, -1)^\top$ and `g.normalized()`, which is exactly $(0, 0, -1)^\top$ because IEEE square root returns $|x|$ for $\sqrt{x^2}$, so $\mathbf{Q}$ is the exact identity, and `Reanchor` with equal directions computes a zero axis. The direction therefore never moves.

### 5.2 Checks run in the container on 2026-11-11

1. The syntax check of `/ws/skills/cpp-syntax-check-tooling.md` passed for `src/vi_estimator/sqrt_keypoint_vio.cpp` and `src/utils/vio_config.cpp` with the vendored Eigen 3.4.0, with no error or warning.
2. The seven JSON files parse and carry the key as `true`.
3. The standalone build, the unit tests and the replays could not run, because the container rebuilt since 2026-11-10 had no `/usr/include/eigen3`, `/usr/include/opencv4`, CMake, Ninja, colcon or ROS. The skill file was corrected to record this.

### 5.3 Replays of `/ws/data/telemetry_40`, not yet run

The `slam` package is built into a separate install space per the same skill file, and three replays of each configuration run headless on separate domains with starts 4 s apart, using `sitl_config_vo.json` and a scratch copy with the flag false. The measures are those of the delayed marginalisation plan's table, the scale ratio over 2 s and the tilt error from `gt_eval.csv` at 70 s, 80 s, 94 s and 100 s, together with the number of swaps, the horizontal accelerometer bias magnitude and the tilt step column of `vio_gravity_scale.csv`, which must be zero throughout the period with the flag false. The 2026-11-10 runs of the same binary with the flag's behaviour at true (`17-17-31`, `17-50-04`, `17-50-08`, scale ratios 3.2 to 5.8) are the baseline.

### 5.4 Outstanding, for the user

1. `colcon build --packages-select slam` on the host, and `test_vio`, `test_qr`, `test_linearization` and `test_nfr`.
2. The replays of §5.3.
3. Live `sitl2` climbs with the flag false, the three profiles of plan 2 §5.5 item 5, since they are where the tilt ran away.

## References

[1] L. von Stumberg, V. Usenko, D. Cremers, "Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization", IEEE ICRA 2018, arXiv:1804.05625, §III-E and §III-F, as summarised in `context/dynamic_marginalisation.md` §3.2.

[2] L. von Stumberg, D. Cremers, "DM-VIO: Delayed Marginalization Visual-Inertial Odometry", IEEE RA-L 2022, arXiv:2201.04114, §III-B, as summarised in `context/dynamic_marginalisation.md` §3.3.
