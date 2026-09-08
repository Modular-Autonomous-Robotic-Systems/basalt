# Linearisation framework, facts read from source

Date of investigation, 2026-09-02. Companion to `doc/Linearisation.md`, which is the narrative treatment. This file records only the non-obvious, reusable facts and the defects, so that a future session need not re-derive them. Anchors are as of the commit following `5c768c6`.

## The default configuration

`vio_linearization_type` defaults to `LinearizationType::ABS_QR` at `src/utils/vio_config.cpp:58`. `isLinearizationSqrt` at `src/linearization/linearization_base.cpp:45-56` returns true for `ABS_QR` alone, and that single boolean, paired with `vio_sqrt_marg`, selects both which exit the estimator takes from the linearisation and which of the three `MargHelper` routines runs.

Every shipped `data/*.json` pins `ABS_QR` explicitly, which matches the default rather than overriding it. The only place the default is genuinely varied is `data/iccv21/basalt_batch_config.toml:370-378`, the paper's own ablation sweep, which compares non square root Schur complement, square root Schur complement and square root QR. `REL_SC` appears in no shipped config and only in `test/src/test_linearization.cpp`.

`vio_use_lm` defaults to false at `src/utils/vio_config.cpp:77`, so the outer loop is Gauss-Newton with a diagonal regularisation, not an adaptive trust region.

## Precision actually used

`BASALT_INSTANTIATIONS_DOUBLE` and `BASALT_INSTANTIATIONS_FLOAT` are both ON by default at `CMakeLists.txt:252-253`, so both scalar types are compiled. The float instantiation of the estimator factory is nonetheless commented out at `src/vi_estimator/vio_estimator.cpp:184-189`, and every call site in the repository requests `getVioEstimator<double>`. The shipped pipeline therefore always runs in double precision, and the single precision conditioning argument that motivates the square root formulation is not exercised by any current code path. Do not claim otherwise in documentation or in a design discussion.

## CONFIRMED DEFECT, initial bias prior weights are transposed

`src/vi_estimator/sqrt_keypoint_vio.cpp:94-98` in the square root branch and `:106-110` in the squared branch apply `vio_init_ba_weight` to tangent indices 9 to 11 and `vio_init_bg_weight` to indices 12 to 14.

The authoritative tangent ordering is the doxygen comment at `thirdparty/basalt-headers/include/basalt/imu/imu_types.h:219-220`, which reads `15x1 increment vector [trans, rot, vel, bias_gyro, bias_accel]`, confirmed by the body of `PoseVelBiasState::applyInc` adding `inc.segment<3>(9)` to `bias_gyro` and `inc.segment<3>(12)` to `bias_accel`. Indices 9 to 11 are therefore the gyroscope bias and 12 to 14 the accelerometer bias, so the two weights are exchanged relative to the blocks they name.

With the defaults of `vio_init_ba_weight = 1e1` and `vio_init_bg_weight = 1e2` the gyroscope bias is held ten times more loosely than intended and the accelerometer bias ten times more tightly. Severity is low, because both are deliberately weak regularisers whose stated purpose of preventing bias jumps survives the transposition, and the defect is inherited from upstream. Anyone tuning these must target the block actually reached rather than the block named. This corroborates the record already in `context/vio_residuals_and_priors.md`.

Correction note. A sub-agent investigation on 2026-09-02 initially reported the opposite assignment, namely accelerometer bias at indices 9 to 11. That reading was wrong and was superseded by direct reading of the doxygen comment cited above.

## The prior needs no index remapping, and why

Both `AbsOrderMap` construction sites build the layout as all `frame_poses` in ascending timestamp followed by all `frame_states` in ascending timestamp. Marginalisation never reorders survivors and `optimize` only appends newer states, so the prior's variables are always the oldest and always occupy a contiguous prefix of the columns. The square root assembly at `src/linearization/linearization_abs_qr.cpp:604-626` therefore writes the prior into the leading columns with a plain block assignment and no permutation.

The invariant is verified rather than computed, by assertions at `sqrt_keypoint_vio.cpp:701`, `:726`, `:1175` and `:1187`, and again inside `BundleAdjustmentBase::linearizeMargPrior` at `src/vi_estimator/ba_base.cpp:415-421`. A source comment at `linearization_abs_qr.cpp:375-376` states the assumption and notes that the check lives outside the class. An earlier investigation described this as an unenforced convention, which is too strong.

## MargLinData::H is not a Hessian in the default configuration

When `is_sqrt` is set, which it is by default, the field named `H` holds a square root factor $J$ with $J^T J = H^\ast$, and the field named `b` holds a residual rather than an information vector. The estimator constructor makes this unambiguous by storing `std::sqrt(weight)` in the square root branch and `weight` in the other, at `sqrt_keypoint_vio.cpp:88-110`. Reading `H` as a Hessian makes the prior assembly code incomprehensible. The flag is fixed once from `config.vio_sqrt_marg` and is never reassigned inside `marginalize`.

## Where the square root property is kept and where it is spent

The odometry solve does form the normal equations. `optimize` calls `get_dense_H_b` and factors with `Eigen::LDLT` at `sqrt_keypoint_vio.cpp:1363`, so it squares the condition number. Only `marginalize` takes the square root exit `get_dense_Q2Jp_Q2r`. The asymmetry is deliberate, since the odometry system is rebuilt at the next frame and its conditioning damage is discarded, whereas the prior persists and compounds. Do not "fix" the odometry path to use the square root exit without measuring, since the dense system is small and the LDLT is fast.

## Dead and latent code in the linearisation path

`setPoseDamping` is dead. Its call site is commented out at `sqrt_keypoint_vio.cpp:1314-1319`, so `get_dense_Q2Jp_Q2r_pose_damping` never runs and the damping row offset reserved in the stacked assembly is always empty. The damping that does run is `H_copy.diagonal() += (H.diagonal()*lambda).cwiseMax(min_lambda)` at `:1358-1361`.

This masks a real inconsistency. The square root pose damping at `linearization_abs_qr.cpp:593-602` writes only `num_cameras * POSE_SIZE` diagonal entries, whereas the dense variant at `:628-634` touches the full `aom.total_size` diagonal including velocity and bias. If the square root path were ever revived the two would disagree.

`add_dense_H_b_marg_prior` asserts `marg_scaling.rows() == 0` at `linearization_abs_qr.cpp:640`, whereas the square root path supports scaling. Since `vio_scale_jacobian` defaults to true, a caller that both scaled the Jacobian and called `get_dense_H_b` with a prior present would abort. It does not happen today because the odometry loop never calls `scaleJp_cols`. This is a latent constraint on any change that enables scaling on that path.

`last_state_to_marg` is accepted by the `LinearizationAbsQR` constructor and discarded by `UNUSED` at `linearization_abs_qr.cpp:67`. It is consumed only by the estimator, to truncate the order map and to build the keep and marginalise partitions.

`log_problem_stats` is an empty body in all three concrete strategies, at `linearization_abs_qr.cpp:178-181` and the corresponding sites in the two Schur complement classes. It is a documented extension point that has not been taken up, and all real telemetry is recorded at the estimator level instead.

`filterOutliers` exists on `BundleAdjustmentBase` at `src/vi_estimator/ba_base.cpp:270-309` and is called by the mapping layer, but at `sqrt_keypoint_vio.cpp:1566` and `sqrt_keypoint_vo.cpp:1364` there are only TODO comments. Robustness inside the odometry loop therefore rests entirely on Huber down weighting, and no observation is ever removed.

`numScalarsLandmark` and `getLandmarkBlocks` do not exist anywhere in the tree. `get_rel_permutation`, `compute_rel_permutation`, `addQ2JpTQ2Jp_blockdiag` and `add_dense_H_b_rel` are declared on `LandmarkBlock` and abort as unimplemented in the only concrete class. `BlockDiagonalAccumulator` is consequently dead in the shipped path.

## The landmark block storage layout

`LandmarkBlockAbsDynamic` holds one row major dense matrix, sized at `landmark_block_abs_dynamic.hpp:84-102`. Rows are `2 * n_obs + 3`, the trailing three reserved for landmark damping. Columns are `aom.total_size`, rounded up to a multiple of four by padding, plus three landmark columns plus one residual column, and the total is asserted to be a multiple of four for alignment.

The pose Jacobian spans the entire active state, not merely the frames this landmark observes, stated by the comment at `:85`. The block is therefore structurally sparse but densely stored. A TODO at `:52-55` acknowledges the waste and proposes a reduced order map. The benefit of the full width is that one orthogonal transformation can be applied across a whole row with no index bookkeeping, which is what makes `performQR` a short loop. Sparsity is retained separately in `res_idx_by_abs_pose_`.

`performQR` at `:456-472` runs three Householder reflections, each computed from one landmark column but applied to `storage.block(k, 0, remainingRows, num_cols)`, that is to every column including the residual. After it, rows 0 to 2 hold the triangular factor with `Q1^T Jp` and `Q1^T r`, and the rows below hold `Q2^T Jp` and `Q2^T r`.

Accounting subtlety. `numQ2rows` returns `num_rows - 3`, equal to `2 * n_obs`, and the export reads rows `[3, num_rows)`, which runs past the last observation row into the three damping rows. Those are zero when damping is inactive, so the effective count of informative reduced rows is `2 * n_obs - 3`. The surplus keeps the exported row count independent of the damping state, which keeps the global row offsets stable across the inner loop.

Landmark damping at `:219-259` writes `sqrt(lambda)` into the reserved rows and folds them in with exactly six Givens rotations, which are stored so that the undo path can replay them in reverse using each adjoint. This is why Givens rather than Householder is used here, since the damping must be varied across the inner loop without repeating the decomposition.

## Whitening, and why it must precede the QR

Each visual row is multiplied by `std::sqrt(weight) / options_->obs_std_dev` at `landmark_block_abs_dynamic.hpp:171`, which is the square root of the weight rather than the weight itself. This is not cosmetic. An orthogonal transformation preserves the Euclidean norm but not a weighted norm, so QR based elimination is only legitimate once the weights have been folded into the rows. The legacy path at `src/vi_estimator/ba_base.cpp:112` applies the squared weight instead, because it accumulates `H` directly.

A consequence for tuning. The Huber threshold is applied after whitening, so `vio_obs_huber_thresh` is in standard deviations and not in pixels, and changing `vio_obs_std_dev` silently changes the effective threshold in pixels.

## Local mapping does not use this framework

`class LocalMapper` extends `NfrMapper` which extends `ScBundleAdjustmentBase<double>`, and it uses the classical Schur complement path through `linearizeHelper`. A grep of `src/vi_estimator/local_mapper.cpp` for `LinearizationBase`, `performQR` or `LandmarkBlock` returns nothing, and `doc/LocalMapper.md` §7.7 states that the optimisation routines are inherited unchanged. The square root formulation is an odometry back end feature only, and porting it to the mapper would be real work rather than a configuration change.

## The two Schur complement strategies, for contrast

`LinearizationAbsSC` and `LinearizationRelSC` eliminate landmarks during `linearizeProblem` by an analytic three by three Schur complement, so their `performQR` overrides have empty bodies. Both always form a dense `H`. Their `get_dense_Q2Jp_Q2r` does not throw, but neither is it an orthogonal projection. It forms the dense normal equations and factors them with LDLT to synthesise a square root, at `linearization_abs_sc.cpp:253-291`, so the name is preserved for interface compatibility and the mathematics is quite different. Both stub out every per landmark operation with a not implemented assertion.

`RelSC` differs from `AbsSC` only in when the chain rule from relative to absolute poses is applied, eliminating in the relative frame first via `linearizeRel` at `src/vi_estimator/sc_ba_base.cpp:501-539` and projecting afterwards.

## Threading

All parallelism lives in `src/linearization/linearization_abs_qr.cpp`, and `sqrt_keypoint_vio.cpp` contains no TBB construct at all. Parallel are block allocation, landmark linearisation, elimination, back substitution over landmarks, column norm accumulation, landmark scaling and damping, and the landmark part of both dense assemblies. Sequential are the relative pose cache, which writes a shared hash map, and everything inertial, damping and prior related, because those number a handful of items against thousands of landmarks.

The row offsets each block writes to are precomputed in the only sequential loop of the constructor at `linearization_abs_qr.cpp:152-161`, so the parallel export writes disjoint ranges and needs no synchronisation.

## The VO twin

`SqrtKeypointVoEstimator` uses the same framework with four differences. It never supplies inertial data. Its order map contains only six wide pose blocks and `frame_states` is asserted empty at `sqrt_keypoint_vo.cpp:1026`. Its order map is seeded from the prior's ordering and extended rather than rebuilt and cross checked, which is the more defensive construction. And its initial prior weights all six pose degrees of freedom uniformly at `:86-89`, correctly, since without a gravity reference a pure visual system has six unobservable directions rather than four.
