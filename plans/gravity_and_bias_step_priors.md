# Gravity Prior and Accelerometer Bias Step Prior

Implemented 2026-11-11 and revised 2026-11-13 by `plans/delayed_marginalisation.md` Change 12. The gravity prior is kept and also enters the coarse optimisation as a prior on the block (§2.3.12 there). The window's accelerometer bias step prior was removed from `optimize`, and `vio_ba_step_prior_std` now sets the bias prior of the coarse optimisation, $w_b = 1 / (q^2 k \Delta t)$ (§2.3.11 Step 5 there). The record below describes both priors as implemented on 2026-11-11. Note, 2026-11-13. This line previously read only "Implemented 2026-11-11". Anchors were read against the working tree of `shreyas/kf_driven_mapping_and_logger`, which includes `plans/gravity_scale_tilt_optional.md` and `plans/gravity_scale_update_cadence.md`.

## 1. Context

The live runs of 2026-11-11 show the accelerometer bias moving by 0.8 to 4.2 m/s² within one second, 700 to 3800 times the one second spread of the bias random walk (`accel_bias_std` 0.001132 m/s²/√s in `data/sitl_calib.json`), with per frame changes of 190σ to 1500σ at the 99th percentile. A random walk residual of that weight would never be violated so heavily, so the movement is a common shift of every bias state in the window, which the random walk cannot see because it only constrains differences between consecutive states. At constant attitude that shift trades one for one against tilt (plan 2 §3.2.3), which is why the tilt error and the horizontal bias moved together (correlation 0.56, `context/dynamic_marginalisation.md` §11.2). The only absolute anchors are the accelerometer bias prior of the first state ($\sigma = 0.1$ m/s² through `vio_init_bg_weight`, by the index swap of `context/vio_residuals_and_priors.md`) and no prior at all on roll and pitch (L4, lost at every swap, §6.1.2).

The request is two optional constraints behind separate configuration keys, both off by default.

1. A bias step prior that penalises, at every frame, how far the accelerometer bias of each state in the window moves from its value at the start of that frame's solve.
2. A gravity prior, the body frame gravity (roll and pitch) prior on the first state of §6.19 Formulation 2, which bounds the total tilt and survives every swap with Change A.

## 2. Design

### 2.1 Bias step prior

A frame local residual, like the zero velocity term and Change B, never marginalised. For every state $k$ of the window with accelerometer bias $\mathbf{b}_{a,k}$ and its value $\bar{\mathbf{b}}_{a,k}$ when `optimize` starts,

$$\mathbf{r}_k = w\,(\mathbf{b}_{a,k} - \bar{\mathbf{b}}_{a,k}),$$

$$w^2 = \frac{1}{q_{b}^2\,\Delta t},$$

with $q_b$ the configured intensity in m/s²/√s and $\Delta t$ the newest frame interval, the same interval Change B uses. For the newest state $\bar{\mathbf{b}}_{a,k}$ is the value it was created with, copied from the previous state, so the residual penalises any departure from the previous frame's bias. A common shift $\boldsymbol{\delta}$ of all $K$ states costs $\frac{1}{2} K w^2 |\boldsymbol{\delta}|^2$ in that frame, which the random walk does not provide. Its per frame tolerance is $q_b\sqrt{\Delta t}$, a rate limit on the common shift, and through the tilt and bias trade a rate limit on that tilt.

The bias coordinates have zero entries in every gauge vector of the block (`ComputeGaugeProjector`, plan 1 §2.5.3), so the projector's bias rows are those of the identity and the residual is unaffected by the projection, by the cadence's held frames and by the tilt hold.

### 2.2 Gravity prior

The first state's attitude is perturbed on the left in the world frame, $\mathbf{R} \leftarrow \text{Exp}(\delta\boldsymbol{\varphi})\,\mathbf{R}$ (`PoseState::incPose`, `thirdparty/basalt-headers/include/basalt/imu/imu_types.h:98-101`), so indices 3, 4 and 5 of a state are world roll, pitch and yaw, and the existing yaw prior is the diagonal entry at index 5 (`sqrt_keypoint_vio.cpp:105-130`). The roll and pitch prior is the same construction on indices 3 and 4,

$$\mathbf{r}_g = \sqrt{w_g}\,\delta\boldsymbol{\varphi}_{0,xy},$$

$$w_g = \frac{1}{\sigma_g^2},$$

with $\sigma_g$ the configured standard deviation in radians. The prior is linear at the first state's linearisation point with a zero residual there, so its reference is the initial alignment, the $\mathbf{z}_0$ of §6.19.

In the gravity and scale period the tilt directions of the block are a gauge (plan 1 §2.5.3), and a prior on the frames alone would break gauge invariance. The prior must penalise the tilt of the first frame relative to the gravity direction, so it couples the block's tilt coordinates. The block rotates gravity by the world rotation vector $\bar{\mathbf{B}} \boldsymbol{\delta}_\theta$, and at construction $\bar{\mathbf{B}} = [\mathbf{e}_x, \mathbf{e}_y]$ (`imu_types.h:336`), so

$$\mathbf{r}_g = \sqrt{w_g}\,(\delta\boldsymbol{\varphi}_{0,xy} - \boldsymbol{\delta}_\theta).$$

Gauge check. Along $\mathbf{n}_\mathbf{a}$ the first frame rotates by $\mathbf{a}$ and the block by $\bar{\mathbf{B}}^\top \mathbf{a} = \mathbf{a}_{xy}$, so the residual changes by $\sqrt{w_g}(\mathbf{a}_{xy} - \mathbf{a}_{xy}) = \mathbf{0}$. Along $\mathbf{n}_\sigma$ it has no scale column. This is §6.19 Step 4 in the linearised world frame form. With the tilt held, the block's tilt columns are masked in every solve, so the coupling is inert and the prior reduces to the plain roll and pitch prior.

`SetInitialPriorOrder` copies `marg_data` into $M_\text{half}$ and $M_\text{visual}$ (`sqrt_keypoint_vio.cpp:2293-2294`), so the rows reach all three priors. `RecentrePriorsGS` and `RecentreGravityScale` map the prior's columns generically, as they do for the yaw row. With Change A on, the rotation rows carry the prior through every marginalisation into $M_\text{visual}$, restoring L4 (§6.19 Step 5). With Change A off, $M_\text{visual}$ loses it when the first state leaves, as it loses the yaw pin today. Outside the period it propagates through the inertial factors like the yaw pin.

The weight derivation of §6.19 applies: the bias anchor already gives the tilt about 9600 rad⁻² at $\sigma_{b_a} = 0.1$ m/s², so 3° (365 rad⁻²) acts as a plausibility bound and 1° (3283 rad⁻²) as substantial information.

## 3. Implementation

### Change 1. Configuration keys, `include/basalt/utils/vio_config.h` and `src/utils/vio_config.cpp`

```cpp
    int vio_gs_update_every_n_frames;
    // Per frame prior on the accelerometer bias step of every window state,
    // m/s^2 per square root second, 0 disables.
    double vio_ba_step_prior_std;
    // Prior on the first state's roll and pitch relative to gravity, degrees,
    // 0 disables.
    double vio_gravity_prior_std_deg;
```

Defaults `0.0` and `0.0` after `vio_gs_update_every_n_frames = 1;`, and `ar(CEREAL_NVP(config.vio_ba_step_prior_std));` and `ar(CEREAL_NVP(config.vio_gravity_prior_std_deg));` after its serialisation.

### Change 2. Gravity prior in the constructor, `src/vi_estimator/sqrt_keypoint_vio.cpp:105-147`

In both branches of the initial prior, after the yaw entry.

```cpp
        // prior on yaw
        marg_data.H(5, 5) = std::sqrt(Scalar(config.vio_init_pose_weight));
        // prior on roll and pitch relative to gravity,
        // plans/gravity_and_bias_step_priors.md
        if (config.vio_gravity_prior_std_deg > 0) {
            const Scalar sigma = Scalar(config.vio_gravity_prior_std_deg * M_PI / 180.0);
            marg_data.H(3, 3) = marg_data.H(4, 4) = Scalar(1) / sigma;
        }
```

The information form branch sets `Scalar(1) / (sigma * sigma)` instead. In the period, after the existing padding `H_gs.rightCols(POSE_VEL_BIAS_SIZE) = marg_data.H;`, the rows gain the block coupling.

```cpp
        if (config.vio_gravity_prior_std_deg > 0) {
            const Scalar sigma = Scalar(config.vio_gravity_prior_std_deg * M_PI / 180.0);
            H_gs(3, 0) = H_gs(4, 1) = -Scalar(1) / sigma;
        }
```

The period requires the square root form (`sqrt_keypoint_vio.cpp:132-136`), so the coupling exists only there. Rows 3 and 4 keep their index after the padding, the first state's columns move by `GRAVITY_SCALE_SIZE`, and the block's tilt columns are 0 and 1.

### Change 3. Bias step prior in `optimize`, `src/vi_estimator/sqrt_keypoint_vio.cpp`

After Change B's weights, before the iteration loop.

```cpp
        // Accelerometer bias step prior, plans/gravity_and_bias_step_priors.md.
        Scalar baStepW2 = 0;
        Eigen::aligned_map<int64_t, Vec3> baRef;
        if (config.vio_ba_step_prior_std > 0 && !imu_meas.empty()) {
            const Scalar dtFrame =
                Scalar(imu_meas.rbegin()->second.get_dt_ns()) * Scalar(1e-9);
            const Scalar baStd =
                Scalar(config.vio_ba_step_prior_std) * std::sqrt(dtFrame);
            baStepW2 = Scalar(1) / (baStd * baStd);
            for (const auto& [t_ns, state] : frame_states)
                baRef[t_ns] = state.getState().bias_accel;
        }
        const bool applyBaStep = baStepW2 > 0;
        auto baStepCost = [&]() {
            Scalar c = 0;
            for (const auto& [t_ns, state] : frame_states)
                c += (state.getState().bias_accel - baRef.at(t_ns)).squaredNorm();
            return Scalar(0.5) * baStepW2 * c;
        };
```

Its four sites mirror the zero velocity term exactly, with the accelerometer bias at offset 12 of the tangent ordering [trans, rot, vel, bias_gyro, bias_accel].

```cpp
            if (applyBaStep) error_total += baStepCost();
```

```cpp
                    if (applyBaStep) {
                        for (const auto& [t_ns, state] : frame_states) {
                            const int bi = aom.abs_order_map.at(t_ns).first + 12;
                            H.template block<3, 3>(bi, bi).diagonal().array() += baStepW2;
                            b.template segment<3>(bi) +=
                                baStepW2 * (state.getState().bias_accel - baRef.at(t_ns));
                        }
                    }
```

```cpp
                    if (applyBaStep) {
                        for (const auto& [t_ns, state] : frame_states) {
                            const int bi = aom.abs_order_map.at(t_ns).first + 12;
                            const Vec3 dba = inc.template segment<3>(bi);
                            const Vec3 e = state.getState().bias_accel - baRef.at(t_ns);
                            l_diff -= baStepW2 * dba.dot(e + Scalar(0.5) * dba);
                        }
                    }
```

```cpp
                if (applyBaStep) after_error_total += baStepCost();
```

The H and b site sits beside the zero velocity term, before the gauge projection, the l_diff site beside the zero velocity term's model decrease after `inc = -inc`, and the cost after update after the zero velocity term's.

### Change 4. Configuration files

The two keys at `0.0` after `vio_gs_update_every_n_frames` in the seven JSON files under `data/` and in `data/iccv21/basalt_batch_config.toml`. `data/sitl_config_vo.json` also ships `0.0`, so no configured behaviour changes.

### Change 5. Documentation and context

`doc/VIO.md` (the period's section and the initial prior's), `context/vio_config_json.md` (the two keys), `context/dynamic_marginalisation.md` §12 (the evidence of §1, both designs and their results), `context/vio_residuals_and_priors.md` (the roll and pitch rows of the initial prior and their coupling), and the rows of `context/README.md`.

## 4. Blast radius

| Symbol | Consumers | Keys at 0 | Gravity prior on | Bias step prior on |
|---|---|---|---|---|
| Initial prior in the constructor | every run | identical | two rows on the first state's roll and pitch, coupled to the block's tilt in the period | identical |
| `optimize` | every frame | identical | identical | one residual per window state, frame local |
| `nullspace_marg_data`, marginalisation, re-centring, `LocalMapper` | unchanged code | identical | carry the rows generically | identical |

## 5. Verification

1. Code reading. With both keys at 0 every added statement is skipped. The gauge check of §2.2 holds for the coupled rows, and the bias rows of the projector are those of the identity.
2. Syntax check of `sqrt_keypoint_vio.cpp` and `vio_config.cpp` per `/ws/skills/cpp-syntax-check-tooling.md` with the vendored Eigen. All eight configuration files carry every key `serialize` reads. Both passed on 2026-11-11 with no diagnostic, and all 82 keys are present in every file. `data/sitl_config_vo.json` then held `vio_gs_update_every_n_frames` 10 and `vio_init_bg_weight` 1e4 set by the user, which were kept. Items 3 and 4 are outstanding.
3. On the host, `colcon build --packages-select slam` and `test_vio`.
4. Live or replayed flights of `telemetry_96` to `telemetry_99` and of the newer bags, in the dashboard's `gravity_scale` tab with `time_reference` `"liftoff"`. Suggested settings are `vio_gravity_prior_std_deg` 3 and then 1, `vio_ba_step_prior_std` 0.1 and then 0.03, each alone and then together. Measures are the horizontal accelerometer bias and its tilt equivalent, the tilt error, the scale ratio, swaps and the bias trace of `bias_components`.
