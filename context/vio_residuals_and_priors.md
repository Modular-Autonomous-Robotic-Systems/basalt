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
