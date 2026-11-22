# VIO ↔ Local Mapper Correction Loop Analysis

**Date**: 2026-07-11
**Snapshots analysed**: `/ws/gt-drift.png`, `/ws/gt-drift-2.png`, `/ws/gt-drift-3.png`

---

## Observed Symptoms

1. **Constant offset between GT and SLAM trajectories** — both trajectories follow the same shape (correct alignment fix applied) but diverge in position over time.
2. **Offset between SLAM trajectory and local mapper KF frusta** — the pink/amber local mapper keyframe frustums appear displaced relative to the red VIO trajectory line.

---

## Visualiser Color Map

| Element | Color constant | RGB | Description |
|---------|---------------|-----|-------------|
| VIO trajectory line | `cam_color` | (250, 0, 26) red | `mvpVioTrajectory` from `out_state_queue` |
| VIO sliding-window states | `state_color` | (250, 0, 26) red | `vio->states` frusta |
| VIO non-KF frames | `pose_color` | (0, 50, 255) blue | `vio->frames` frusta |
| GT trajectory | `gt_color` | (0, 171, 47) green | `mvpGroundTruthTrajectory` |
| Local map points | `local_map_point_color` | (255, 128, 0) orange | 3D landmarks from BA |
| Local map KF frusta | `local_map_kf_color` | (255, 128, 128) pink/amber | `lm->keyframes` BA-refined poses |

---

## Correction Data Flow (as-built)

```
VIO (SqrtKeypointVioEstimator)
│
├── frame_states   ← active sliding window poses (IMU-integrated)
│       │
│       └── out_state_queue ──► GUI mvpVioTrajectory (red line)
│                                    ▲ NEVER CORRECTED by local mapper
│
├── frame_poses    ← marginalized KFs (pose-only, frozen from VIO window)
│       │
│       ├── out_marg_queue ──────────────────────────────────────────►
│       │                                                              │
│       └── mpPosesToUpdate ◄── QueuePoseUpdates ◄── PoseUpdateCallback
│                 │
│                 └── applied in measure() (next VIO frame) to frame_poses only
│
LocalMapper (on separate thread)
│
├── frame_poses ← seeded from MargData (VIO's frame_poses at marginalization time)
│       │
│       └── optimize() ► BA-refined poses ──► out_vis_queue ──► GUI pink frusta
│                 │
│                 └── PoseUpdateCallback(frame_poses) ──► VIO QueuePoseUpdates
│
└── lmdb        ← landmarks (world-frame, consistent with LocalMapper frame_poses)
```

**Critical observation**: The live VIO trajectory (`frame_states` → `out_state_queue`) is in a separate data path from the correction loop. BA corrections from the local mapper can only reach VIO's `frame_poses` (the frozen, already-marginalized set), never `frame_states`. The GUI trajectory is driven by `frame_states` exclusively.

---

## Issue 1 — GT vs SLAM Offset

**Root cause**: VIO drift, not alignment error.

The two-part GT alignment fix (IO normalization + `mpSlamFirstPose` visualiser correction) correctly aligns the origins. The divergence observed in the screenshots is accumulated VIO position error from the sliding-window estimator running without any global correction.

This is unavoidable without loop closure. The local mapper runs local BA but:
- Its corrections only reach marginalized `frame_poses`, not the live trajectory
- It has no loop closure (no place recognition for revisited scenes)
- Without loop closure, translation drift grows unboundedly over time

---

## Issue 2 — SLAM Trajectory vs Local Mapper KF Offset

**Three mechanisms** cause the pink KF frusta to appear displaced from the red VIO trajectory:

### Mechanism A — No live-trajectory correction (architectural)

The VIO trajectory (red line in GUI) is `frame_states` → `out_state_queue`. The local mapper BA refines its own copy of `frame_poses`. The feedback callback path is:

```
LocalMapper::MapLocally()
  → mpVioPoseUpdateCallback(frame_poses)   [local_mapper.cpp:246]
  → Controller lambda
  → vio_estimator_->QueuePoseUpdates(poses)
  → mpPosesToUpdate staging map           [sqrt_keypoint_vio.h:219]
  → measure() applies to VIO frame_poses  [sqrt_keypoint_vio.cpp:364-398]
```

`frame_states` is never touched by this path. The live trajectory that the GUI renders does not receive any correction from the local mapper. Note, 2026-11-21. Mechanisms A and B describe the code before `plans/local_mapper_correction_ingestion.md` was implemented. `measure` no longer writes `frame_poses`. It drains `mpPosesToUpdate` into `mvpLocalMapPoses`, and `optimize` adds relative pose residuals between the shared keyframes, which reach `frame_states` through the joint solve (item 10 below).

### Mechanism B — Hard correction threshold blocks large updates

From `sqrt_keypoint_vio.h:290`:
```cpp
static constexpr Scalar kRelinThresholdTrans = Scalar(0.10);  // 10 cm
static constexpr Scalar kRelinThresholdRot   = Scalar(0.05);  // ~3 degrees
```

In `measure()` (lines 378-384), corrections where the BA-refined pose differs from the VIO pose by more than 10 cm or 3° are silently rejected with `"too large update in pose, not updating"`.

As VIO drift accumulates, the gap between VIO's `frame_poses` and the BA-refined `frame_poses` grows. The threshold means corrections become progressively less effective as drift increases — exactly when correction is most needed. Note, 2026-11-21. `kRelinThresholdTrans` and `kRelinThresholdRot` were removed with the overwrite. The residuals carry no threshold, and their weight is set by `vio_local_map_pos_std` and `vio_local_map_rot_std_deg`.

### Mechanism C — Translational gauge freedom in NfrMapper BA

`NfrMapper::optimize()` (`nfr_mapper.cpp:249`) only has:
- `rel_pose_factors` — constrain **relative** poses between KFs (shape of trajectory, not position)
- `roll_pitch_factors` — constrain gravity alignment (2 DOF: roll + pitch)

There is **no absolute position constraint** in the local BA problem. The remaining 4 DOF (absolute translation XYZ + yaw) are unconstrained. If reprojection error is dominant and pulls poses off-axis, the BA solution can be self-consistent but translated/rotated relative to the VIO world frame. This causes the local mapper KFs to appear shifted from where VIO placed them.

This is the "gauge freedom" problem: `rel_pose_factors` fix the SHAPE of the keyframe constellation; `roll_pitch_factors` fix the gravity direction; but the LOCATION of the constellation in world space is unconstrained by the factor graph alone. The only implicit anchor is the initial values of `frame_poses` (seeded from VIO), which acts as a prior, but it is not explicitly enforced.

---

## What the Local Mapper BA DOES Achieve

Despite not correcting the live trajectory, local BA provides:
1. **More accurate KF poses** — refined by visual reprojection over cross-frame tracks
2. **Better landmark positions** — triangulated with refined poses, so the map is geometrically consistent
3. **Improved future frame tracking** — if VIO re-encounters these KFs, the better map geometry reduces reprojection error

What it CANNOT do in the current architecture:
- Correct the running VIO trajectory (`frame_states`)
- Apply corrections larger than 10 cm / 3° back to VIO's `frame_poses`
- Perform loop closure (no place recognition for revisited scenes)

---

## Path to Fixing the Gap

To make local mapper corrections propagate to the live VIO trajectory:

**Option 1 — Sliding window reanchoring**: When the local mapper refines a KF that is still in the VIO window (unlikely given the one-way marg flow, but possible if the window is large), apply the correction as a pose prior in the VIO optimization.

**Option 2 — Velocity/position injection**: Compute the delta between VIO's current `frame_states` origin and the BA-corrected origin. Apply this delta as a velocity or position jump at the start of the next IMU preintegration. Risks consistency violation (FEJ breakage).

**Option 3 — Loop closure with pose graph correction**: Detect revisited places. When a loop is closed, apply a global SE(3) correction to ALL states (past and present). This is the standard ORB-SLAM3 / VINS-Fusion approach.

**Option 4 — Fix gauge freedom in NfrMapper**: Add an absolute position prior on the oldest KF (fix its pose to the VIO-provided value with tight covariance). This prevents the BA from floating in translation, so corrections fed back to VIO are in the same world frame.

---

## Relevant Code Locations

| Location | Role |
|----------|------|
| `src/vi_estimator/sqrt_keypoint_vio.cpp`, `measure` | Drains `mpPosesToUpdate` into `mvpLocalMapPoses` and prunes it to the window, since 2026-11-21. Previously the overwrite and threshold rejection |
| `src/vi_estimator/sqrt_keypoint_vio.cpp`, `GetLocalMapPoses`, `LocalMapRows`, `LocalMapRows2`, `RelPoseFactorRows` | Build the relative pose rows that `optimize` injects, since 2026-11-21 |
| `src/vi_estimator/sqrt_keypoint_vio.h:215-220` | `QueuePoseUpdates` — writes staging map |
| `include/basalt/vi_estimator/sqrt_keypoint_vio.h` | `mpPosesToUpdate`, `mvpLocalMapPoses`, `mpLocalMapOpen`. `kRelinThresholdTrans/Rot` removed 2026-11-21 |
| `src/vi_estimator/local_mapper.cpp:246` | Where local mapper fires callback after BA |
| `src/controller.cpp:167-168` | Wires the callback: `LocalMapper → QueuePoseUpdates` |
| `src/vi_estimator/nfr_mapper.cpp:249-390` | BA implementation (no absolute position prior) |
| `src/visualisation/visualiser.cpp:260-300` | DrawScene: trajectory + KF frusta rendering |

## The `out_state_queue` tap became non-blocking, 2026-08-23

The push at `src/vi_estimator/sqrt_keypoint_vio.cpp:594` that feeds `mvpVioTrajectory` is now `try_push` rather than `push`, honouring the contract stated at `src/visualisation/visualiser.cpp:36-37`. The queue holds one hundred states and the consumer appends rather than replaces, so a drop is a permanent gap in the red trajectory line and a missing row in the plotter log rather than a stale value. The cost is cosmetic, since neither structure feeds the estimate, and it buys the guarantee that a stalled Pangolin thread can no longer block the estimator. The first state cannot be dropped, because the queue is necessarily empty when it is pushed, which matters since `ConsumeVioStateQueue` captures `mpSlamFirstPose` from it for ground truth alignment.

The companion tap at `:621` feeding `mvpVioVisQueue` is also `try_push` now. That queue is already latest wins, since `ConsumeVioVisQueue` stores into `mpLatestVio` and `Run` reads only that pointer, so a drop there is invisible beyond a frame of staleness.

Full analysis in `/ws/ros_ws/src/slam/plans/basalt-local-mapper-deadlock.md` section 4.6.

## Feeding mapper corrections back as window residuals, findings of 2026-11-18

These findings come from the planning of `plans/local_mapper_correction_ingestion.md`, which replaced the overwrite of Mechanisms A and B with relative pose residuals in `SqrtKeypointVioEstimator::optimize`, implemented on 2026-11-21. The plan holds the derivations, and the facts below are the reusable ones.

1. The mapper's own `RelPoseFactor`s cannot be fed back to the window. `NfrMapper::extractNonlinearFactors` (`src/vi_estimator/nfr_mapper.cpp:215-249`) joins the packet's single marginalised keyframe to every other keyframe of `kfs_all`, and the estimator erases that keyframe from `frame_poses` right after sending the packet (`src/vi_estimator/sqrt_keypoint_vio.cpp:1390-1423`, `:1645`). A prune to the window therefore removes all of them. Their measurement and `cov_inv` are also the window's own estimate and marginal, so feeding them back would double count the prior. What can be reused is the type, `relPoseError` and its Jacobians, and the `J^T cov_inv J` accumulation of `MapperLinearizeAbsReduce` (`include/basalt/vi_estimator/nfr_mapper.h:131-162`).
2. `relPoseError` (`include/basalt/utils/nfr.h:43-73`) is double only and has no scale. The planned `relPoseErrorGS` in `include/basalt/utils/ba_utils.h` is a template whose translation is multiplied by the log scale of the gravity and scale block, with `d_res_d_logscale` equal to the scaled relative translation rotated by the measurement, and `relPoseError` becomes a wrapper that calls it at scale 1, so one implementation serves the mapper and the window. Taking the off diagonal Jacobian block from `hat(T_j_i.inverse().translation())` of the already scaled `T_j_i`, as `relPoseError` does, rather than from `scale * hat(R_i^T (p_j - p_i))`, keeps the wrapper bit identical to the former body, since every other difference is a multiplication by exactly one. This was checked on 2026-11-19 over 20000 random triples at `-O0` and `-O2 -march=x86-64-v3`. A verbatim extract passed a numeric Jacobian test in the header only harness on 2026-11-18 and again on 2026-11-19. Note, 2026-11-19. This item previously described `relPoseErrorGS` as a templated copy beside an untouched `relPoseError`.
3. ICE-BA's released code (`github.com/baidu/ICE-BA`, read 2026-11-18, the CVF PDF returns HTTP 403) stores its local BA prior `CameraPrior::Pose` as the relative poses of several keyframes to one reference keyframe `m_iKFr`, plus a roll and pitch residual on the reference, with dense information, evaluated at the current poses. `LocalBundleAdjustor::UpdateCameras` overwrites keyframe poses with the global BA's and moves each local frame rigidly with its nearest keyframe, leaving the prior untouched. The overwrite is safe there only because the prior is relative and relinearised at the current estimate, which Basalt's first estimate prior and delayed store do not allow.
4. For relative pose residuals whose measurements all come from one set of mapper poses, the choice of pairs changes only the weighting, not the minimiser. Under independent map errors, all pairs with uniform weight $\frac{1}{N^2 \sigma^2}$ give exactly the mean centred deviation, while a star from one anchor with diagonal weights overstates the anchor mode by a factor $N$. At $N = 7$ the anchor's stiffness is 8.2 times that of all pairs, and its scale information is 3.8 times for a straight path. The plan implements both as `LocalMapRows` and `LocalMapRows2` for comparison.
5. ICE-BA's marginalisation, read from its code on 2026-11-19 (`LocalBundleAdjustor.cpp`, `LocalBundleAdjustorGN.cpp`, `GlobalBundleAdjustorGN.cpp`, through a summarising fetch that covered the first 100000 characters of each). Its incremental solver concerns the Schur elimination of feature inverse depths at every iteration, with factor contributions updated only when their variables change, and that elimination leaves no prior. The only marginalisation is that of the oldest local frame in `MarginalizeLocalFrame`, which marginalises the frame together with the depths of the features it hosts and its inertial factor into the member prior `m_Zp`. At every marginalisation `m_Zp` is split, its pose part, extracted by `GetPriorPose` and `MarginalizeUninformative`, going to the global BA as `m_ZpKF`, where `m_Zps.push_back` appends it without merging, and its motion part `m_ZpLF` being kept, applied at every local iteration on the oldest local frame as a body frame velocity and bias prior and also sent to the global BA, where it replaces the previous `m_ZpLM`. `m_Zp` is then rebuilt by `m_Zp.Initialize(..., m_ZpLF, ...)` in both branches of `MarginalizeLocalFrame`, from the motion marginal and a weak tilt prior at the reference. The pose part is not applied locally because the local BA holds keyframe poses constant, its update flags being set only for local frames. The global BA marginalises no keyframe in the code read and holds only the priors the local BA hands it, re-evaluated at the current poses every iteration. A reading in which no prior survives across frames therefore holds for the global BA and for all pose information, but not for the local BA's motion marginal, and the prior that results is on cameras and motion, not on landmarks. Note, 2026-11-19. This item previously stated, earlier the same day, that `m_Zp` persists and accumulates for the whole run, which overlooked the rebuild after each split.
6. For a cost that sums uniformly weighted squared differences over the edges of a graph of keyframes, the Hessian per axis is the weight times the graph Laplacian $\mathbf{D}^\top \mathbf{D}$, the degree matrix minus the adjacency matrix. The complete graph's Laplacian is $N$ times the centring projector. The anchor topology's cost equals $\frac{N}{N - 1}$ times the pairwise cost plus $\frac{N}{2\sigma^2(N - 1)}|\mathbf{d}_a - \bar{\mathbf{d}}|^2$, so its whole difference is a spring on the anchor's deviation from the mean. Moving the anchor alone costs $\frac{N^2}{N - 1}$ times more than under all pairs, 8.17 at $N = 7$, and moving any other keyframe alone $\frac{N^2}{(N - 1)^2}$ times more, 1.36. A term of stiffness $\lambda$ along a mode against other information $\lambda_o$ moves the window toward the map by the fraction $\frac{\lambda}{\lambda + \lambda_o}$ of the disagreement. Checked numerically on 2026-11-19.
7. Why ICE-BA can overwrite keyframe poses without updating any prior, read from its code on 2026-11-19. The local BA carries only the motion marginal, whose velocity error is in the body frame, and the rigid carry of each local frame with its nearest keyframe rotates both its attitude and its velocity, so the error is unchanged by any correction. Keyframes are constants of the local BA, so no local information refers to their old values. The pose priors live in the global BA, the writer of the corrections, and are evaluated at its current poses with $\mathbf{H} = \mathbf{J}^\top \mathbf{A} \mathbf{J}$ and $\mathbf{g} = \mathbf{J}^\top(\mathbf{A} \mathbf{e} + \mathbf{b})$, $\mathbf{A}$ and $\mathbf{b}$ frozen at the marginalisation and $\mathbf{e}$ and $\mathbf{J}$ recomputed. Their relative errors satisfy $\mathbf{e}(\mathbf{G} \mathbf{x}) = \mathbf{e}(\mathbf{x})$ for translation and yaw, hence $\mathbf{J}(\mathbf{x})\,\mathbf{N}(\mathbf{x}) = \mathbf{0}$ at every $\mathbf{x}$, so they need no first estimates. A keyframe deleted from a prior has its rows erased, which conditions the prior on it rather than marginalising it. Basalt's window prior is linear in absolute offsets from frozen first estimates of its own states and relies on the first estimate Jacobians of the other factors for its null space, so none of this transfers to it. The general account of priors in Basalt is in `vio_residuals_and_priors.md`.
8. Window relative pose accuracy of SITL from logs, measured 2026-11-21. The mapper's keyframe poses are not logged, so the gauge free accuracy within one VIO window is estimated from `gt_eval.csv` at the keyframe timestamps of `map_publish.csv`, one per mapper cycle. For every run of 7 consecutive keyframes whose true positions span at least 1 m on some axis, centre both sets, remove the rigid rotation by the SVD of the cross covariance (Umeyama without scale), and take the root mean square residual per axis, and for rotations the root mean square of $\text{Log}(\mathbf{R}^\ast_k \mathbf{R}_k^\top)$ about its mean. The `gt_eval.csv` quaternions are in the order x, y, z, w. Over the four runs `2026-11-15_20-42-02`, `2026-11-16_21-37-55`, `2026-11-16_22-38-18` and `2026-11-16_23-17-19` of the present `data/sitl_config_vo.json`, 4471 windows, the percentiles 50, 75 and 90 are 0.072, 0.147 and 0.264 m in position and 0.064, 0.111 and 0.143 degrees in rotation. These bound the mapper's error from above and set `vio_local_map_pos_std = 0.15` and `vio_local_map_rot_std_deg = 0.11` in `plans/local_mapper_correction_ingestion.md` Change 1. `max_trans_err` of `vio_pose_update.csv`, 0.35 to 0.50 m median in the same runs, is not a substitute, because it compares absolute poses and includes the drift of the gauge.
9. Basalt's keyframe marginalisation does not always remove the oldest keyframe. `SqrtKeypointVioEstimator::marginalize` (`src/vi_estimator/sqrt_keypoint_vio.cpp:1199-1272`) first takes the oldest keyframe, skipping the newest two, whose share of points still tracked is below `vio_kf_marg_feature_ratio`, and otherwise the minimum of the score taken from DSO. A mapper pose whose timestamp is newer than the window's oldest keyframe may therefore belong to a keyframe that has already left the window, which is why a correction channel must filter on `aom.abs_order_map` and not only on the oldest timestamp. Found 2026-11-21.
10. The implementation, 2026-11-21. `relPoseErrorGS` lives in `include/basalt/utils/ba_utils.h`, and `relPoseError` in `include/basalt/utils/nfr.h` calls it at scale 1, so `nfr.h` now includes `ba_utils.h`. `measure` drains the mapper's poses with their epochs into `mvpLocalMapPoses`, erases entries older than the oldest of `frame_poses` or `frame_states`, and no longer writes any window pose. `GetLocalMapPoses` returns nothing unless `mpLocalMapOpen` is set and both standard deviations are positive, carries each pose through `GravityScaleCorrectionLog::Between(epoch, latest)` and keeps only timestamps in `aom.abs_order_map`. `mpLocalMapOpen` is set when `UpdateDelayedMarg` installs a fine job and at the end of every `marginalize` without an active gravity and scale block, and since `initialize` creates `mpDm` whenever it activates the block and only `FoldGravityScale` stops it, a period whose coarse phase lasts until the fold receives no residuals at all. The rows are added at the four sites of the stationary rows in `optimize`. `vio_pose_update.csv` keeps its columns, now `applied` the number of pairs of the first linearisation, `rejected` 0, `max_trans_err` and `max_rot_err` the largest unweighted residuals, and `limit_trans` and `limit_rot` the two standard deviations, the rotation in radians. `vio_gs_dm_hold_pose_updates` was removed from the code and from every file in `data/`. Only `data/sitl_config_vo.json` enables the term, with 0.15 m, 0.11 degrees and the pairwise topology.
11. First replays with the term on, 2026-11-21. On `/ws/data/telemetry_40`, whose first 51.8 s of flight a 170 s replay covers, all inside the gravity and scale period, the largest unweighted translation residual between the mapper's and the window's relative poses has a median of 0.07 to 0.08 m in the first seconds after the gate opens and grows to 0.6 to 0.8 m by 35 to 45 s, while the cumulative scale correction reaches 1.15. Both topologies raised the `se3` aligned trajectory error above the 1.475 m of the term off in at least one of two replays each, pairwise 2.39 and 1.86 m, anchored 4.48 and 1.68 m. Replays with the term off are deterministic, and replays with it on vary with the mapper thread's timing, so a comparison needs several replays per configuration. The source of the growing disagreement is not established, because the mapper's poses are not logged.
12. Divergence analysis of 2026-11-21, live runs `2026-11-21_13-57-22` (bag `/ws/data/telemetry_73`) and `2026-11-21_14-07-01` (`telemetry_75`), replays `14-06-*` and `14-33-*` of both bags, and the good live run `14-26-44`. The chain has three links. First, on these take offs the coarse initialiser never frees the scale, because the marginal variance of the log scale stays at 0.005 to 0.015 against `vio_gs_dm_coarse_scale_var` = 0.0025, and it fits an accelerometer bias of 0.3 to 0.5 m/s² instead, so the fine phase starts at 11 to 15 s with the window's scale 0.3 to 0.6 of the truth. In the good run the coarse job freed the scale at 10.2 s, installed $\sigma$ = 0.18, and the period ended at 23.6 s after one readvance. Second, the long period that follows leaves the scale weakly observable in cruise and hover, so it wanders even with the local map term off, up to 3.48 with a 63 m error in `14-06-37`. Third, the term's measurement is not independent of the window, since the mapper's keyframe poses and its NFR relative pose factors are the window's own earlier estimates carried by `GravityScaleCorrectionLog`, so the mapper's speed equals the window's wrong speed, 2.56 against 4.35 m/s true in `14-33-24`. Within a frame the carry is consistent, the scale gap not following scale steps with the keyframe set unchanged (slope 0.09, correlation 0.07), but each mapper delivery or keyframe set change resets the gap by 0.02 to 0.06, mostly toward the side of an ongoing scale ramp. The term then pushes the scale along that ramp, +95 to +171 against −70 to −275 from the rest of the system at 36 to 45 s of `14-33-24`, and dominates in hover, where the rest's curvature on the log scale falls from about 4500 to 700 while the term's is 400 to 900. It also holds the scale between deliveries, so `vio_gs_dm_fold_settle_scale` can fold a wrong scale, as in `14-06-41` and `14-33-20`, folded at 1.05 and 1.09 with errors of 78 and 83 m. A term with a scale column during the period therefore feeds the estimator's own scale history back as a measurement.
13. The first link of item 12 is the coarse scale gate, tested on 2026-11-21. Replays of `/ws/data/telemetry_73` with `vio_gs_dm_coarse_scale_var` raised from 0.0025 to 1.0 (DM-VIO's default) or 0.02 freed the scale in the coarse phase from 3.8 s or 7.4 s, held the displacement ratio of estimate to truth at 0.95 to 1.02 from take off, needed one readvance install, and reached worst errors of 3.1 to 6.7 m and final errors of 1.3 to 1.8 m (`2026-11-21_14-52-49`, `14-52-54`, `14-52-57`), against 25 to 83 m with the gate at 0.0025. With the scale correct from the coarse phase the local map term was harmless, the term on run being the best of the three. At 1.0 the first coarse estimate at 3.8 s, near lift off, had variance 0.38 to 0.62 and log scale −0.16 to −1.64, so a gate between 0.02 and 1.0 trades an early wild installation against the take off failure. A joint optimisation of the delayed window with the poses free, DM-VIO's pose graph bundle adjustment, cannot have a smaller scale variance than the coarse job, which holds the poses, because eliminating more variables can only lower the Schur complement information on the log scale, so it does not address the gate. The local mapper receives keyframes and packets from the first frame, and held 30 keyframes when the coarse phase ended in `13-57-22`.
14. Sweep of `vio_gs_dm_coarse_scale_var` on `/ws/data/telemetry_72`, the 150 m by 120 m square at 150 m altitude, 2026-11-21, five replays per value, seven in parallel, local map term on. Results, script and per run metrics are in `/ws/logs/slam/sweeps/coarse_scale_var_telemetry72/` (`runs.csv`, `summary.csv`, `sweep.py`). Divergences (worst error above 50 m) per five runs and the se3 ATE mean and standard deviation: 0.0025 five, 0.005 five, 0.01 none at 3.01 ± 0.27 m, 0.02 none at 2.70 ± 0.44 m, 0.05 none at 2.17 ± 0.22 m, 0.2 none at 2.34 ± 0.17 m, 1.0 one at 9.61 ± 16.34 m. The tight gates pass a scale only rarely and late, 8.9 to 9.6 s, so a single coarse estimate of log scale −0.30 to −0.42 stays installed, and the scale collapses to 0.13 to 0.74 or later swings to 4.2. From 0.01 upwards the scale is freed at 4.2 to 5.2 s and re-estimated at every new keyframe, so an early error of −0.6 to −1.4 is corrected before the delayed phase. At 1.0 the first estimate arrives at 3.9 to 4.1 s with −1.9 to −2.1, and one replay in five did not recover (scale 0.07, 252 m). The plateau is 0.05 to 0.2, and 0.05 had the lowest mean ATE and worst error. Replays of the same value are close to deterministic apart from those outliers.
15. Fine sweep of `vio_gs_dm_coarse_scale_var` from 0.05 to 0.2 on `/ws/data/telemetry_72`, 2026-11-21, values 0.05, 0.075, 0.1, 0.125, 0.15, 0.175 and 0.2, five replays each, seven in parallel, results in `/ws/logs/slam/sweeps/coarse_scale_var_fine_telemetry72/`. No replay diverged. The se3 ATE means were 2.52, 2.30, 2.33, 2.25, 2.17, 2.26 and 2.34 m with standard deviations of 0.10 to 0.47 m, worst errors 8.2 to 8.6 m, and sim3 scales 1.014 to 1.019. Pooled over both sweeps, 0.05 and 0.2 both averaged 2.34 m over ten replays, so the differences inside the band are run to run noise of the asynchronous coarse job. The gate acts only through which coarse job is the first admitted. On this bag 0.05 admitted the job at 4.40 s with log scale −1.04 to −1.11, and every value from 0.1 to 0.2 the job at 4.25 s with −1.30 to −1.38, so 0.1 to 0.2 behaved as one configuration. `data/sitl_config_vo.json` was set to 0.1, the lower end of that plateau and near the geometric centre of the failing values 0.005 and 1.0, as the least extreme value that admits the earliest well conditioned estimate here. A second bag is needed to separate values inside the band.
16. Comparison of the live runs `2026-11-21_17-06-15` (good, bag `telemetry_86`) and `2026-11-21_17-13-01` (failed, bag `telemetry_87`), identical configuration and calibration, the gate at 0.1. The flights differed although the same trajectory was commanded. The good one climbed steadily at about 4 m/s to 106 m. The failed one crept upward at 0.5 m/s from 4 s to 7 s, reached 23 m at 14 s, hovered and landed, 52 m of path against 221 m. SLAM does not feed the autopilot, so the flight difference arose outside SLAM. In the failed live run four frames at 0.16 s to 0.33 s had a mean optical flow of 0.81 to 1.21 px against `vio_stationary_flow_tol` = 0.8, from a few outlier tracks (track survival 0.35 to 0.45, largest flow 12 to 17.7 px, while the ground truth was static), which exceeded `vio_stationary_latch_frames` = 3 and closed the start latch for good at 0.27 s. The estimator then never declared rest, kept both biases at zero, drifted 0.09 m/s on the ground, started the period's motion index at the first step and skipped the lift off reset. Four replays of `telemetry_87` did not reproduce the latch failure, rest being detected for 34 to 36 frames, but the scale was still wrong in all four, displacement ratios 0.23 to 3.5, with the latch at 3 or 10, so the gentle take off alone leaves the coarse scale poorly determined (first admitted variance 0.089 against the gate of 0.1, five of 61 coarse jobs admitted in the live run). The flow stage's mean is sensitive to a few outlier tracks, and the latch closes permanently on any three consecutive misses.
