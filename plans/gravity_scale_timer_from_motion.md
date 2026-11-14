# Gravity and Scale Period Timed From Take-off

Kept by `plans/delayed_marginalisation.md` (§2.3.13 there), where the motion start also gates the first coarse optimisation of the period, added 2026-11-13.

## 1. Motivation

The gravity and scale period ended when `last_state_t_ns - mpGsStartNs >= vio_gs_duration_s` (`src/vi_estimator/sqrt_keypoint_vio.cpp`, end of `marginalize`), with `mpGsStartNs` set in `SetInitialPriorOrder` at the first state. Time spent on the ground therefore counted towards the period. A short period, meant to cover only the first seconds of flight, was consumed in part or entirely before take-off. The meaning of `vio_gs_duration_s` was changed to time from take-off, without a new key, so that the period can be cut to a very short flight time. `data/sitl_config_vo.json` sets it to 2 s.

## 2. Design

`mpGsStartNs` keeps its name and its sole consumer, the fold check. It is no longer set at the first state. It stays at -1 until the first frame of confirmed motion, and the fold check requires it to be non-negative, so the period cannot end before take-off.

Motion is confirmed on a frame where `mpIsStationary` is false and, if `mpStationaryEnableStartLatch` is set, `mpStationaryLatchOpen` is false.

- With the latch enabled, the timer starts when the latch closes, after `vio_stationary_latch_frames` consecutive non-stationary frames (3 in `data/sitl_config_vo.json`). A single noisy frame on the ground cannot start it, at the cost of starting a few frames after the true take-off.
- With the latch disabled, the timer starts on the first non-stationary frame.
- With every stationary stage disabled, `mpIsStationary` is always false, so the timer starts on the first processed frame of the period. This reproduces the old meaning.

The check sits in `processFrame`, directly after the latch update, where the frame's flag is final. It is not placed in `marginalize`, which returns early until `opt_started`. The cadence counter `mpGsMotionStarted` of `plans/gravity_scale_update_cadence.md` is left unchanged. It counts from the first non-stationary frame, without the latch, and only when N > 1, and changing it would shift the cadence's frame phase in runs that have already been logged.

A vehicle on which motion is never confirmed never folds the block. This follows from measuring flight time only.

## 3. Changes

### Change 1. Timer start, `src/vi_estimator/sqrt_keypoint_vio.cpp`

`SetInitialPriorOrder` no longer assigns `mpGsStartNs = t_ns;`. The line was removed.

In `processFrame`, after the start latch block and before `mpLogger->AddVioImuFrame`:

```cpp
        if (mpGsActive && mpGsStartNs < 0 && !mpIsStationary &&
            (!mpStationaryEnableStartLatch || !mpStationaryLatchOpen))
            mpGsStartNs = curr_frame->t_ns;
```

### Change 2. Fold check, end of `marginalize`

```cpp
    if (mpGsActive) {
        LogGravityScale(mpGsSwapEvent);
        if (mpGsStartNs >= 0 &&
            last_state_t_ns - mpGsStartNs >=
                int64_t(config.vio_gs_duration_s * 1e9)) {
            FoldGravityScale();
        }
    }
```

### Change 3. Configuration

- `include/basalt/utils/vio_config.h`: the comment on `vio_gs_enable` and `vio_gs_duration_s` now states that the duration counts from the first frame of confirmed motion.
- `data/sitl_config_vo.json`: `vio_gs_duration_s` changed from 15 to 2.
- No key was added or removed, so the other configuration files are untouched.

### Change 4. Documentation and context

- `doc/VIO.md`, the initialisation period's opening paragraph.
- `context/vio_config_json.md`, the gravity and scale keys.
- `plans/gravity_scale_optimisation.md`, the fold paragraph.
- `context/dynamic_marginalisation.md` §13.
- `context/README.md`, the rows of the last two files.

## 4. Backwards compatibility and blast radius

| Symbol | Consumers | Effect |
|---|---|---|
| `mpGsStartNs` | the fold check only | set at confirmed motion rather than at the first state |
| `vio_gs_duration_s` | the fold check only | counts from take-off |
| configurations | `vio_gs_enable` is true only in `data/sitl_config_vo.json`, and false in the seven others including `data/iccv21/basalt_batch_config.toml` | only the SITL configuration changes behaviour |
| `mpGsMotionStarted`, the cadence | `optimize` | unchanged |

Logged SITL runs made before 2026-11-12 measured the duration from the first state. Their fold times are not comparable with later runs at equal `vio_gs_duration_s`.

## 5. Verification

1. Done, 2026-11-12. A syntax check of `src/vi_estimator/sqrt_keypoint_vio.cpp` and `src/utils/vio_config.cpp` per `/ws/skills/cpp-syntax-check-tooling.md`, using the vendored Eigen, reported no errors.
2. Outstanding. On the host, `colcon build --packages-select slam` and `test_vio`.
3. Outstanding. A flight or replay with `data/sitl_config_vo.json`.
   - In `vio_gravity_scale.csv`, the fold event (event 4) should fall about 2 s after the latch closes.
   - In `vio_stationary.csv`, the latch closing is the start of the final run of false `is_stationary` values.
   - In the dashboard's `gravity_scale` tab with `time_reference` set to `"liftoff"`, the fold should sit near 2 s plus the latch delay.
