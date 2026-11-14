# Update Cadence of the Gravity and Scale Block

Removed 2026-11-13. `plans/delayed_marginalisation.md` Change 12 removed the cadence and `vio_gs_update_every_n_frames`, whose role the coarse phase took over, holding the block in the window and correcting it every `vio_gs_dm_coarse_every` marginalisations (§2.3.9 there). The record below describes the cadence as it was implemented on 2026-11-11. Note, 2026-11-13. This line previously read only "Implemented 2026-11-11". Anchors were read against the working tree of `shreyas/kf_driven_mapping_and_logger`, which includes `plans/gravity_scale_tilt_optional.md` (the tilt hold).

## 1. Context

The live flights of 2026-11-11 (`/ws/logs/slam/2026-11-11_17-37-47` and `_17-46-58`, `context/dynamic_marginalisation.md` §9) show that every scale event of the period is a run of one sign block steps, one per frame, each within or only modestly beyond Change B's tolerance, with a swap every second or third frame. The pre hop and lift off of `17-37-47` moved $\sigma$ by $-0.52$ in 38 frames with 13 swaps, and the first second of `17-46-58` by $-0.49$ in 20 frames with 13 swaps, both while the departing inertial factors were kinematically inconsistent (whitened scale columns of 80 to 450). The block may correct at every frame, so it follows the inconsistency frame by frame.

The request is a cadence. The block is held during the initialisation transient and updated only on every $N$-th frame, counted from the first frame of motion, so that the first scale and gravity alignment happens only once $N$ frames of motion are available, and later ones at most once per $N$ frames.

## 2. Design

### 2.1 Behaviour

A new key `vio_gs_update_every_n_frames` (integer, default 1). With $N = 1$ nothing changes. With $N > 1$, in the period:

1. Before the first frame of motion the block is held.
2. The first frame on which the stationarity test of `ProcessFrame` reports motion (`mpIsStationary` false) starts a count, $k = 1$ on that frame, incremented by every later `optimize` of the period.
3. The block is free on frames with $k \bmod N = 0$, that is $k = N, 2N, \dots$, and held on every other frame.

`mpIsStationary` is computed in `ProcessFrame` (`sqrt_keypoint_vio.cpp:498-509`) before `measure` calls `optimize_and_marg` (`:973`), so the decision uses the current frame's test. With every stationarity stage disabled `mpIsStationary` is always false and the count starts at the first optimised frame of the period. Once started the count never stops, whether or not the start latch is enabled, so a later stationary frame does not reset it.

### 2.2 What a held frame is

A held frame solves with the whole block fixed at the identity, which plan 1 §2.5.4 item 4 shows is today's Basalt estimator. Holding all three coordinates is a fixation with $\mathbf{S}_{gs} \mathbf{N}$ a signed permutation of rank 3, so no gauge direction of the block remains, and the projector must not be applied. Applying it would remove the uniform scaling and the joint tilt of the frames from the step, which are physical once the block cannot carry them. A held frame therefore uses $\mathbf{P} = \mathbf{I}$ and masks the three block rows and columns of $\mathbf{H}$ and entries of $\mathbf{b}$ exactly as the tilt hold masks two (`plans/gravity_scale_tilt_optional.md` Change 3), which returns an exact zero block step.

Consequences, all through code that is unchanged.

1. The held frame's block step is an exact zero, so `RecentreGravityScale` applies only what the previous re-centring left in the block, which is zero to rounding, because `GetSigma()` is `mSigmaLin + mDelta[2]` after its own sum was subtracted and can leave about $10^{-17}$, and likewise for the direction when the tilt is free. Every state, prior and anchor is therefore multiplied by an identity to about $10^{-16}$ (exactly, for the rotation, when the tilt is held), and the correction log appends an identity epoch. The mapper's epoch bookkeeping is unaffected.
2. `UpdateScaleInterval` sees no change of $\sigma$ on a held frame, but its half width `mpLogD` is recomputed from the current frame's step and so falls back to $\ln d_\text{min}$. When a large update leaves the interval's centre more than $\ln d_\text{min}$ from $\sigma$, the held frames that follow fire catch up swaps (events 2 or 3), one per frame, each moving the centre by $\ln d_\text{min}$, until it is within. Swaps are therefore triggered by free frames, as one update and its catch up, but not confined to them. Note, 2026-11-11. This item previously stated that no swap can fire on a held frame, which the live run `/ws/logs/slam/2026-11-11_19-43-15` (N = 10) contradicted with 3 of its 10 swaps on held frames, because `mpLogD` is recomputed from the current frame's step and falls back to $\ln d_\text{min}$.
3. Any correction the data support during held frames is carried by the frame states, as outside the period, and the next free frame lets the block take it over and the re-centring apply it exactly.
4. Change B's cost is zero on a held frame because $\mathbf{a}$ stays zero.

### 2.3 Change B on a free frame

On a free frame the block step summarises $N$ frames, so under the random walk model of plan 2 §3.4.1 its variance is $q \cdot N \Delta t$, not $q \Delta t$. The residual's interval is therefore $N \Delta t$, with $\Delta t$ the newest frame interval as today. Keeping one frame's $\Delta t$ would penalise the single permitted correction $N$ times more heavily than today's per frame steps, which together cover the same time. With $N = 1$ the interval is exactly today's.

### 2.4 Interaction with the tilt hold

With `vio_gs_optimise_gravity_dir` false, free frames mask the tilt as today and leave $\sigma$ free with the scale only projector. Held frames mask all three coordinates and skip the projector in either case.

## 3. Implementation

### Change 1. Configuration key, `include/basalt/utils/vio_config.h` and `src/utils/vio_config.cpp`

```cpp
    bool vio_gs_optimise_gravity_dir;
    // The block is free only on every N-th frame counted from the first
    // frame of motion and held otherwise, 1 frees it on every frame.
    int vio_gs_update_every_n_frames;
```

```cpp
  vio_gs_optimise_gravity_dir = true;
  vio_gs_update_every_n_frames = 1;
```

```cpp
  ar(CEREAL_NVP(config.vio_gs_optimise_gravity_dir));
  ar(CEREAL_NVP(config.vio_gs_update_every_n_frames));
```

Backwards compatibility. Default 1 reproduces today. Missing keys are fatal (`context/vio_config_json.md`), so Change 4 adds it everywhere.

### Change 2. Estimator members, `include/basalt/vi_estimator/sqrt_keypoint_vio.h`

Next to `mpGsActive` (line 274).

```cpp
    bool mpGsActive = false;
    // Update cadence of the block, plans/gravity_scale_update_cadence.md.
    bool mpGsMotionStarted = false;
    int64_t mpGsMotionFrames = 0;
```

### Change 3. The hold and Change B's interval, `optimize` in `src/vi_estimator/sqrt_keypoint_vio.cpp`

Before the Change B weights (line 1724), decide the frame once.

```cpp
        // Update cadence, plans/gravity_scale_update_cadence.md.
        const int gsUpdateEvery =
            std::max(1, config.vio_gs_update_every_n_frames);
        bool gsHoldBlock = false;
        if (mpGsActive && gsUpdateEvery > 1) {
            if (!mpIsStationary) mpGsMotionStarted = true;
            if (mpGsMotionStarted) mpGsMotionFrames++;
            gsHoldBlock =
                !mpGsMotionStarted || mpGsMotionFrames % gsUpdateEvery != 0;
        }
```

Change B's interval (lines 1729-1731).

```cpp
            const Scalar dtFrame =
                Scalar(imu_meas.rbegin()->second.get_dt_ns()) * Scalar(1e-9) *
                Scalar(gsUpdateEvery);
```

The projection and the mask (lines 1906-1921) become the following. The free frame path is today's code, and the tilt mask is generalised to a count of held rows.

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
                        // Hold the block, plans/gravity_scale_update_cadence.md,
                        // or its tilt, plans/gravity_scale_tilt_optional.md.
                        const int nHeld = gsHoldBlock ? GRAVITY_SCALE_SIZE
                                          : !config.vio_gs_optimise_gravity_dir
                                              ? 2
                                              : 0;
                        if (nHeld > 0) {
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

`inc = gauge_P * inc` (line ~1955) is unchanged and multiplies by the identity on held frames. The block's offset `gs` is 0 and its coordinate order is tilt, tilt, $\sigma$ (`ComputeGaugeProjector` puts $\mathbf{n}_\sigma$ at `gs + 2`), so the first `nHeld` rows are the right ones in both cases.

Backwards compatibility. With $N = 1$, `gsHoldBlock` is false, `dtFrame` is multiplied by exactly 1, the projector path is today's, and `nHeld` is 2 or 0 exactly as the tilt hold's branch today, so the default and the tilt hold reproduce the current results. Outside the period nothing runs.

### Change 4. Configuration files

`"config.vio_gs_update_every_n_frames": 1,` after `vio_gs_optimise_gravity_dir` in the seven JSON files under `data/`, and `"config.vio_gs_update_every_n_frames" = 1` in `data/iccv21/basalt_batch_config.toml`. `sitl_config_vo.json` stays at 1, and experiments set $N$ in a scratch copy.

### Change 5. Documentation and context

1. `doc/VIO.md`, after the tilt hold paragraph (around line 1043), one paragraph on the cadence, the held frame as today's estimator without projection, the swap bound and Change B's interval.
2. `context/vio_config_json.md`, the key and its default.
3. `context/dynamic_marginalisation.md`, a §10 recording the design and the results of §5.
4. `context/README.md`, the two rows.

## 4. Blast radius

| Symbol | Consumers | With $N = 1$ | With $N > 1$ |
|---|---|---|---|
| `vio_gs_update_every_n_frames` | `optimize`, eight configuration files | none | §2 |
| Projection and mask in `optimize` | every frame of the period | identical | held frames unprojected and fully masked |
| Change B weights | every frame of the period | identical | interval $N \Delta t$ |
| `RecentreGravityScale`, `UpdateScaleInterval`, `LinearizeMargGS`, `MarginalizeGS`, `LocalMapper` | unchanged | identical | identity corrections on held frames, swaps only from an update and its catch up |
| `vio_gravity_scale.csv` | unchanged | identical | `sigma_step` and `jump_cost` exactly 0 on held frames, the run time check of the schedule |

## 5. Verification

1. Code reading. With $N = 1$ trace that every added statement is skipped or multiplies by one. With $N > 1$ confirm that a held frame's block step is exactly zero and that `RecentreGravityScale` then multiplies by identities to rounding (§2.2 item 1). Done 2026-11-11.
2. Syntax check of `sqrt_keypoint_vio.cpp` and `vio_config.cpp` per `/ws/skills/cpp-syntax-check-tooling.md` (vendored Eigen, the container lacking the build tools on 2026-11-11). JSON files parsed. Both passed on 2026-11-11 with no diagnostic, and all seven JSON files load with the key at 1. Items 3 to 5 are outstanding.
3. On the host, `colcon build --packages-select slam` and `test_vio`.
4. Replays of `/ws/data/telemetry_85` and `/ws/data/telemetry_86` (today's two flights, the tilt held) with $N \in \{1, 5, 10, 20\}$, about 0.27 s, 0.54 s and 1.1 s at 18.5 frames per second, three each, `djinn start basalt-replay` arguments per `/ws/context/djinn-workflows.md`. $N = 1$ must reproduce the live runs' behaviour. Measures, with the scripts of `context/dynamic_marginalisation.md` §9: the lift off and landing scale collapse ($\sigma$ change over the transient), swaps, scale ratio at 3, 10, 20, 30 s, tilt error, and that `sigma_step` is nonzero only on scheduled frames.
5. Live `sitl2` flights of profiles P1 and P2 (§9.1) with the best $N$, since those are the profiles that diverged.
