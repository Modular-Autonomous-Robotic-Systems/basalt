# Stationary Body Detection, Implementation Plan

Status as of 2026-10-25. Changes 1 to 6 are implemented, and change 8 (section 12), the certified ground interval gate, is a plan awaiting review except for its start latch, which was implemented on 2026-10-26 (section 12.10). Correction, 2026-10-25. This line previously read PLAN awaiting review, which predated the implementation of changes 1 to 6. See `context/stationary_detection.md` for the full literature survey and mathematical derivations this plan implements. Every code change below cites the context document section its design comes from rather than re-deriving it.

## 1. Motivation

`context/stationary_detection.md` section 1 measures Basalt's live stationarity flag, `mpIsStationary` at `include/basalt/vi_estimator/sqrt_keypoint_vio.h#L285`, against all eight logged experiments under `/ws/logs/slam` and finds 93 to 99% of every run's flagged-stationary frames carry a VIO velocity above 0.1 m/s. Section 1's physical argument is that specific force and angular rate alone cannot distinguish true rest from constant velocity cruise, so no purely inertial statistic, however windowed, closes the gap on its own. Sections 4.1 through 4.5 of that document propose three complementary signals, each strong where the others are blind, and section 4.6 selects a combination of all three as the detector to implement. This plan turns that design into concrete, line-referenced code changes across the VIO backend, the VO backend, the shared optical flow front end, the configuration layer, and the logger.

## 2. High-level design

Three independently configurable stages, combined by logical AND into `mpIsStationary` inside `SqrtKeypointVioEstimator`,

1. Stage 1, a growing window multi-frame SHOE statistic with a persistence requirement (`context/stationary_detection.md` section 4.1 and 4.6). Strong once several hundred milliseconds of samples have accumulated, available from the very first integrated frame since the ring buffer grows from empty rather than waiting behind a warm up period.
2. Stage 2, `IntegratedImuMeasurement::predictState`'s full predicted next velocity, tested as a chi-squared(3) Mahalanobis distance from zero against the preintegration's own velocity covariance block (section 4.2, 4.4 and 4.6). This is the one stage that resolves constant velocity cruise, the case stage 1 is structurally blind to, and it is lag free since it reads only the previous, already optimised state.
3. Stage 3, a veto on `OpticalFlowResult::mFlowPxMean`, a vision based signal physically independent of the inertial sensors (section 4.5).

Each stage is gated by its own boolean in `VioConfig`, read once at construction, so an operator can disable any subset without a rebuild. The three stages are ANDed, matching the design in section 4.6 of the context document, with flow defaulting off since it measures weaker and noisier on the logged flights than the other two (section 11 below).

`SqrtKeypointVoEstimator` has no IMU integration and no velocity state at all, confirmed by reading `src/vi_estimator/sqrt_keypoint_vo.cpp:190-194`, which drains and discards every IMU sample every frame, and `PoseStateWithLin`, `POSE_SIZE = 6`, which carries no velocity field. Stages 1 and 2 have nothing to attach to there. Stage 3 reads only `OpticalFlowResult`, which VO already receives unchanged, so VO gets a flow-only diagnostic flag, computed and logged for offline visibility, that gates no residual and changes no existing VO behaviour.

The shared optical flow front end gains the two new fields needed to read flow magnitude from inside either backend, which is the one change common to both pipelines.

## 3. Change 1, `VioConfig` thresholds and enable flags

### The change

`include/basalt/utils/vio_config.h`, insert after `double vio_zero_velocity_weight;` at line 104, before `bool vio_marg_lost_landmarks;` at line 106,

```cpp
    // Three independently gated stationary body detector stages, combined
    // by logical AND into mpIsStationary. See plans/stationary_detection.md
    // and context/stationary_detection.md section 4.6. Shoe and Mahalanobis
    // default on, flow defaults off since it is the weakest of the three on
    // logged flight data, see the plan's threshold determination section.
    bool vio_stationary_enable_shoe;
    bool vio_stationary_enable_mahalanobis;
    bool vio_stationary_enable_flow;
    // Stage 1, section 4.1. Ring buffer span in seconds and the chi-squared
    // critical value on the window-mean SHOE statistic. See the plan for
    // the chi-squared(6W-2) derivation of the default, W = window_s times
    // the IMU rate.
    double vio_stationary_window_s;
    double vio_stationary_glrt_tol;
    // Consecutive clearing windows required before stage 1 latches true.
    // Kept low so a short true rest early in a mission is not missed.
    int vio_stationary_min_consecutive_frames;
    // Stage 2, section 4.4. Chi-squared(3) 95% critical value, exact under
    // the stated approximation that the previous state's own velocity
    // uncertainty is neglected relative to the fresh preintegration noise.
    double vio_stationary_chisq_tol;
    // Stage 3, section 4.5. Optical flow magnitude veto threshold in
    // pixels, read from OpticalFlowResult::mFlowPxMean. Also read directly
    // by SqrtKeypointVoEstimator for its flow-only diagnostic.
    double vio_stationary_flow_tol;
```

`src/utils/vio_config.cpp`, constructor, insert after `vio_zero_velocity_weight = 0.0;` at line 89,

```cpp
  vio_stationary_enable_shoe = true;
  vio_stationary_enable_mahalanobis = true;
  vio_stationary_enable_flow = false;
  vio_stationary_window_s = 0.5;
  vio_stationary_glrt_tol = 6.6;
  vio_stationary_min_consecutive_frames = 2;
  vio_stationary_chisq_tol = 7.81;
  vio_stationary_flow_tol = 0.8;
```

`src/utils/vio_config.cpp`, `serialize`, insert after `ar(CEREAL_NVP(config.vio_zero_velocity_weight));` at line 213,

```cpp
  ar(CEREAL_NVP(config.vio_stationary_enable_shoe));
  ar(CEREAL_NVP(config.vio_stationary_enable_mahalanobis));
  ar(CEREAL_NVP(config.vio_stationary_enable_flow));
  ar(CEREAL_NVP(config.vio_stationary_window_s));
  ar(CEREAL_NVP(config.vio_stationary_glrt_tol));
  ar(CEREAL_NVP(config.vio_stationary_min_consecutive_frames));
  ar(CEREAL_NVP(config.vio_stationary_chisq_tol));
  ar(CEREAL_NVP(config.vio_stationary_flow_tol));
```

### All eight configuration sources

Cereal's JSON archive is fatal on a missing key, confirmed in `context/vio_config_json.md`, so every one of the eight sources that already carries `vio_zero_velocity_weight` needs the eight new keys too, including the two VO-only sources, since `VioConfig` is shared and parsed in full regardless of which estimator consumes it. All eight currently carry `"config.vio_zero_velocity_weight"` at line 38 (line 46 in the `.toml`), confirmed by direct grep. Insert immediately after that line in each of

`data/euroc_config.json`, `data/euroc_config_no_factors.json`, `data/euroc_config_no_weights.json`, `data/euroc_config_vo.json`, `data/kitti_config.json`, `data/sitl_config_vo.json`, `data/tumvi_512_config.json`,

```json
        "config.vio_stationary_enable_shoe": true,
        "config.vio_stationary_enable_mahalanobis": true,
        "config.vio_stationary_enable_flow": false,
        "config.vio_stationary_window_s": 0.5,
        "config.vio_stationary_glrt_tol": 6.6,
        "config.vio_stationary_min_consecutive_frames": 2,
        "config.vio_stationary_chisq_tol": 7.81,
        "config.vio_stationary_flow_tol": 0.8,
```

and in `data/iccv21/basalt_batch_config.toml`, after line 46, using TOML's own assignment syntax,

```toml
"config.vio_stationary_enable_shoe" = true
"config.vio_stationary_enable_mahalanobis" = true
"config.vio_stationary_enable_flow" = false
"config.vio_stationary_window_s" = 0.5
"config.vio_stationary_glrt_tol" = 6.6
"config.vio_stationary_min_consecutive_frames" = 2
"config.vio_stationary_chisq_tol" = 7.81
"config.vio_stationary_flow_tol" = 0.8
```

### Backwards compatibility

Every field has a default equal to the value proposed above, so a config file missing these keys cannot occur after this change lands everywhere, and no existing caller's behaviour changes through the config layer itself. The actual behaviour change is in `SqrtKeypointVioEstimator`'s detector logic, change 4 below, and is deliberate per the user's request. `context/stationary_detection.md` section 3.2 already establishes that `mpZeroVelocityWeight` is zero in every configuration actually read by `SqrtKeypointVioEstimator` today (`sitl_config_vo.json`'s positive value is read by `SqrtKeypointVoEstimator`, which never consumes it), so today the zero velocity constraint this detector gates is inert everywhere in practice and this change has no live optimiser effect until an operator separately raises `vio_zero_velocity_weight` above zero.

## 4. Change 2, `SqrtKeypointVioEstimator` member additions

### The change

`include/basalt/vi_estimator/sqrt_keypoint_vio.h`, add `#include <deque>` beside the existing `<mutex>` and `<thread>` includes at line 42.

Add `using typename SqrtBundleAdjustmentBase<Scalar>::Mat3;` beside the existing `using typename ...` declarations at line 66, reusing the `Mat3` alias `SqrtBundleAdjustmentBase` already defines at `include/basalt/vi_estimator/sqrt_ba_base.h:48` rather than redefining it.

In the private member section, the four existing stationarity members at lines 280-286 are superseded by the three-stage detector below and are commented out in place rather than deleted, per this repository's correction convention,

```cpp
    // Superseded 2026-10-19 by the three-stage detector of
    // plans/stationary_detection.md change 4. Retained for reference since
    // these implemented Basalt's original single-frame MAG+ARE test.
    // Scalar mpStationaryAccelTol = Scalar(0.5);
    // Scalar mpStationaryGyroTol = Scalar(0.05);
    bool mpIsStationary = false;
    Scalar mpZeroVelocityWeight = Scalar(0);
```

and the new members are added immediately after,

```cpp
    // ── Stationary body detector, three independently gated stages ──
    // combined by logical AND. See plans/stationary_detection.md change 4
    // and context/stationary_detection.md section 4.6.
    struct ImuSample {
        int64_t t_ns;
        Vec3 accel, gyro;
    };
    std::deque<ImuSample> mvpStationaryWindow;
    int mpStationaryRunLength = 0;

    bool mpStationaryEnableShoe = true;
    bool mpStationaryEnableMahalanobis = true;
    bool mpStationaryEnableFlow = false;
    Scalar mpStationaryWindowS = Scalar(0.5);
    Scalar mpStationaryGlrtTol = Scalar(6.6);
    int mpStationaryMinConsecutiveFrames = 2;
    Scalar mpStationaryChiSqTol = Scalar(7.81);
    double mpStationaryFlowTol = 0.8;
```

`src/vi_estimator/sqrt_keypoint_vio.cpp`, constructor, read the eight new config fields alongside the existing `mpZeroVelocityWeight = Scalar(config.vio_zero_velocity_weight);` at line 80,

```cpp
    mpStationaryEnableShoe = config.vio_stationary_enable_shoe;
    mpStationaryEnableMahalanobis = config.vio_stationary_enable_mahalanobis;
    mpStationaryEnableFlow = config.vio_stationary_enable_flow;
    mpStationaryWindowS = Scalar(config.vio_stationary_window_s);
    mpStationaryGlrtTol = Scalar(config.vio_stationary_glrt_tol);
    mpStationaryMinConsecutiveFrames =
        config.vio_stationary_min_consecutive_frames;
    mpStationaryChiSqTol = Scalar(config.vio_stationary_chisq_tol);
    mpStationaryFlowTol = config.vio_stationary_flow_tol;
```

### Backwards compatibility

The in-class default member initialisers reproduce the defaults set in change 1, so any code path that constructs `SqrtKeypointVioEstimator` without going through `VioConfig` at all keeps the same behaviour this plan proposes as the new default, there is no third, older behaviour left reachable. `mvpStationaryWindow`, `mpStationaryRunLength` and the new boolean/threshold members have no other reader anywhere in the codebase, confirmed by grep, and the two members they supersede, `mpStationaryAccelTol`/`mpStationaryGyroTol`, had none either, so nothing outside this one class is affected.

## 5. Change 3, shared optical flow flow magnitude plumbing

### The change

`include/basalt/optical_flow/optical_flow.h`, `OpticalFlowResult` at lines 59-69 gains two fields,

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

`include/basalt/optical_flow/frame_to_frame_optical_flow.h`, `processFrame`, immediately after the existing `AddOpticalFlowFrame`/`PrintOpticalFlowFrame` call at lines 254-263, which already computes the identical mean and max from `trackStats`,

```cpp
        mpLogger->PrintOpticalFlowFrame();

        transforms->mFlowPxMean = tracked ? trackStats.mFlowSum / double(tracked) : 0.0;
        transforms->mFlowPxMax = trackStats.mFlowMax;
```

`transforms` is the same object the function is about to `return` on both code paths, confirmed at line 150 for the first ever frame and line 237 (`transforms = new_transforms;`) for every subsequent one, so no second copy is created and no new queue or thread handoff is needed. `SqrtKeypointVioEstimator::ProcessFrame` and `SqrtKeypointVoEstimator::ProcessFrame` both already hold this exact object as their `curr_frame` parameter, confirmed at `include/basalt/vi_estimator/sqrt_keypoint_vio.h:124` and `include/basalt/vi_estimator/sqrt_keypoint_vo.h:116`, so `curr_frame->mFlowPxMean` is available for free in both backends.

### Backwards compatibility

Both new fields default to `0.0` and are read nowhere else in the codebase, confirmed by grep, so every existing consumer of `OpticalFlowResult` is unaffected.

## 6. Change 4, the three-stage detector in `SqrtKeypointVioEstimator::ProcessFrame`

### The change

All of stage 1, stage 2 and stage 3 read quantities already computed inside the per-sample integration loop at `src/vi_estimator/sqrt_keypoint_vio.cpp:377-399`, or immediately available from `frame_states` and `curr_frame`, so no new integration cost is added beyond the stage 1 ring buffer push and the stage 2 linear solve.

Inside the per-sample loop, immediately after the existing `meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);` at line 379, push the sample into stage 1's growing window and prune it to `mpStationaryWindowS` seconds,

```cpp
                meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
                // Stage 1's growing window, section 4.1/4.6. Pushed from
                // every raw sample this frame integrates, pruned to the
                // last mpStationaryWindowS seconds rather than reset every
                // ProcessFrame call.
                mvpStationaryWindow.push_back(
                    {imuData->t_ns, imuData->accel, imuData->gyro});
                while (!mvpStationaryWindow.empty() &&
                       double(mvpStationaryWindow.back().t_ns -
                              mvpStationaryWindow.front().t_ns) *
                               1e-9 >
                           double(mpStationaryWindowS))
                    mvpStationaryWindow.pop_front();
```

The existing `accelDeviationMax`/`gyroNormMax` accumulation at lines 382-387 is left untouched, it remains valuable raw per-frame diagnostic data independent of which detector consumes it, and is still logged to `vio_imu_frame.csv` exactly as before.

The existing final assignment at lines 424-426 is superseded and commented out in place,

```cpp
        // Superseded 2026-10-19 by the three-stage detector below. See
        // plans/stationary_detection.md change 4.
        // mpIsStationary = numImuIntegrated > 0 &&
        //                 accelDeviationMax < mpStationaryAccelTol &&
        //                 gyroNormMax < mpStationaryGyroTol;
```

and replaced with,

```cpp
        // Stage 1, section 4.1/4.6, the growing window SHOE statistic with
        // a persistence requirement.
        Scalar tShoe = Scalar(0);
        if (!mvpStationaryWindow.empty()) {
            Vec3 aBar = Vec3::Zero();
            for (const auto& s : mvpStationaryWindow) aBar += s.accel;
            aBar /= Scalar(mvpStationaryWindow.size());
            const Vec3 gDir = aBar.normalized();
            for (const auto& s : mvpStationaryWindow) {
                const Vec3 accelResidual = s.accel - Scalar(9.81) * gDir;
                tShoe += accelResidual.cwiseAbs2().cwiseQuotient(mpAccelCov).sum() +
                         s.gyro.cwiseAbs2().cwiseQuotient(mpGyroCov).sum();
            }
            tShoe /= Scalar(mvpStationaryWindow.size());
        }
        const bool stage1Clears =
            !mvpStationaryWindow.empty() && tShoe < mpStationaryGlrtTol;
        mpStationaryRunLength = stage1Clears ? mpStationaryRunLength + 1 : 0;
        const bool stage1 =
            mpStationaryRunLength >= mpStationaryMinConsecutiveFrames;

        // Stage 2, section 4.4/4.6, predictState's full predicted v1
        // tested as a Mahalanobis distance against the preintegration's
        // own velocity covariance block. Guarded on numImuIntegrated, a
        // frame with zero samples leaves Sigma_v exactly zero and
        // ldlt().solve() singular.
        Scalar d2 = Scalar(0);
        bool stage2 = false;
        if (numImuIntegrated > 0) {
            const auto& state0 = frame_states.at(last_state_t_ns).getState();
            const Mat3 Sigma_v = meas->get_cov().template block<3, 3>(6, 6);
            const Vec3 dv = meas->getDeltaState().vel_w_i;
            const Scalar predDt = Scalar(meas->get_dt_ns()) * Scalar(1e-9);
            const Vec3 residual =
                dv + state0.T_w_i.so3().inverse() *
                         (state0.vel_w_i + g * predDt);
            d2 = residual.transpose() * Sigma_v.ldlt().solve(residual);
            stage2 = d2 < mpStationaryChiSqTol;
        }

        // Stage 3, section 4.5/4.6, the optical flow magnitude veto.
        const bool stage3 = curr_frame->mFlowPxMean < mpStationaryFlowTol;

        mpIsStationary =
            (!mpStationaryEnableShoe || stage1) &&
            (!mpStationaryEnableMahalanobis || stage2) &&
            (!mpStationaryEnableFlow || stage3) &&
            (mpStationaryEnableShoe || mpStationaryEnableMahalanobis ||
             mpStationaryEnableFlow);
```

### Why `numImuIntegrated > 0` guards stage 2 but not stage 1

Stage 1's `mvpStationaryWindow` persists across frames, so a single frame that integrates zero samples, which `context/stationary_detection.md` section 3.1 notes is rare, leaves the window exactly as it was and `tShoe` is still computed correctly from older, still-in-window samples. Stage 2's `Sigma_v` is this frame's own preintegration covariance, which is exactly zero before any `integrate()` call, so a zero-sample frame must short-circuit to `stage2 = false` rather than call `.ldlt().solve()` on a singular matrix. This guard is not present in `context/stationary_detection.md`'s illustrative section 4.4 code and is a correctness fix folded in here, propagated back into the context document in change 7 below.

### Why `mpAccelCov.cwiseQuotient`, not `.diagonal().mean()`

`context/stationary_detection.md` section 4.1's own sketch reads `mpAccelCov.diagonal().mean()`, but `mpAccelCov` is declared as `Vec3 mpAccelCov` at `include/basalt/vi_estimator/sqrt_keypoint_vio.h:293`, the per-axis noise variance itself, not a matrix, so `.diagonal()` does not compile. The corrected form above normalises each axis by its own variance and sums, `Σ_j residual_j² / σ_j²`, which is the exact GLRT construction of section 2.1 and reduces to the isotropic case section 2.1 derives when all three axis variances are equal, which they are in every calibration file read for this project to date. This correction is also folded back into the context document in change 7.

## 7. Change 5, expanded diagnostic logging

### The change

`include/basalt/utils/logger.h`, after `void PrintVioImuFrame();` at line 176,

```cpp
    // ── Stationary body detector (three stage) ───────────────────────
    void AddVioStationary(int64_t tNs, bool enableShoe, double tShoe,
                          int windowSamples, int runLength, bool stage1,
                          bool enableMahalanobis, double d2, bool stage2,
                          bool enableFlow, double flowMean, bool stage3,
                          bool isStationary);
    void PrintVioStationary();

    void AddVoStationary(int64_t tNs, double flowMean, bool isStationary);
    void PrintVoStationary();
```

`src/utils/logger.cpp`, constructor, after the `Register("vio_imu_frame", ...)` call at lines 346-355,

```cpp
    Register("vio_stationary", "[VIO][stationary]",
             {{"t_ns", "ns"}, {"enable_shoe", "flag"}, {"t_shoe", "f"},
              {"window_samples", "count"}, {"run_length", "count"},
              {"stage1", "flag"}, {"enable_mahalanobis", "flag"},
              {"d2", "f"}, {"stage2", "flag"}, {"enable_flow", "flag"},
              {"flow_mean", "f"}, {"stage3", "flag"},
              {"is_stationary", "flag"}});

    Register("vo_stationary", "[VO][stationary]",
             {{"t_ns", "ns"}, {"flow_mean", "f"}, {"is_stationary", "flag"}});
```

`src/utils/logger.cpp`, implementation, after `void Logger::PrintVioImuFrame() { Emit("vio_imu_frame"); }` at line 759,

```cpp
void Logger::AddVioStationary(int64_t tNs, bool enableShoe, double tShoe,
                              int windowSamples, int runLength, bool stage1,
                              bool enableMahalanobis, double d2, bool stage2,
                              bool enableFlow, double flowMean, bool stage3,
                              bool isStationary) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_stationary");
    s.add_int("t_ns", tNs);
    s.add("enable_shoe", enableShoe ? 1.0 : 0.0);
    s.add("t_shoe", tShoe);
    s.add("window_samples", double(windowSamples));
    s.add("run_length", double(runLength));
    s.add("stage1", stage1 ? 1.0 : 0.0);
    s.add("enable_mahalanobis", enableMahalanobis ? 1.0 : 0.0);
    s.add("d2", d2);
    s.add("stage2", stage2 ? 1.0 : 0.0);
    s.add("enable_flow", enableFlow ? 1.0 : 0.0);
    s.add("flow_mean", flowMean);
    s.add("stage3", stage3 ? 1.0 : 0.0);
    s.add("is_stationary", isStationary ? 1.0 : 0.0);
    s.CommitRow();
}

void Logger::PrintVioStationary() { Emit("vio_stationary"); }

void Logger::AddVoStationary(int64_t tNs, double flowMean, bool isStationary) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vo_stationary");
    s.add_int("t_ns", tNs);
    s.add("flow_mean", flowMean);
    s.add("is_stationary", isStationary ? 1.0 : 0.0);
    s.CommitRow();
}

void Logger::PrintVoStationary() { Emit("vo_stationary"); }
```

`src/vi_estimator/sqrt_keypoint_vio.cpp`, `ProcessFrame`, after the existing `mpLogger->PrintVioImuFrame();` at line 438,

```cpp
        mpLogger->AddVioStationary(
            curr_frame->t_ns, mpStationaryEnableShoe, double(tShoe),
            int(mvpStationaryWindow.size()), mpStationaryRunLength, stage1,
            mpStationaryEnableMahalanobis, double(d2), stage2,
            mpStationaryEnableFlow, curr_frame->mFlowPxMean, stage3,
            mpIsStationary);
        mpLogger->PrintVioStationary();
```

### Backwards compatibility

`vio_imu_frame.csv`'s existing `is_stationary` column keeps its name and its meaning, whether `mpIsStationary` is true for this frame, only its computation changes, which is this plan's entire purpose. `scripts/analysis` reads CSV columns by name through `pandas.read_csv`, confirmed in `scripts/analysis/data_reader.py:40,61`, not by position, so the two brand new files, `vio_stationary.csv` and `vo_stationary.csv`, are additive and break no existing offline analysis.

## 8. Change 6, VO flow-only diagnostic

### The change

`src/vi_estimator/sqrt_keypoint_vo.cpp`, `ProcessFrame`, after the existing IMU buffer drain at lines 190-194,

```cpp
    while (!this->imu_data_queue.empty()) {
        ImuData<double>::Ptr d;
        this->imu_data_queue.pop(d);
    }

    // Flow-only stationary diagnostic, plans/stationary_detection.md
    // change 6. VO has no IMU integration and no velocity state, so this
    // gates no residual and exists purely for offline visibility.
    if (config.vio_stationary_enable_flow) {
        const bool voStationary =
            curr_frame->mFlowPxMean < config.vio_stationary_flow_tol;
        mpLogger->AddVoStationary(curr_frame->t_ns, curr_frame->mFlowPxMean,
                                  voStationary);
        mpLogger->PrintVoStationary();
    }
```

### Backwards compatibility

`vio_stationary_enable_flow` defaults to `false`, change 1, so this block does not run and nothing is logged until an operator explicitly opts in, typically in `data/euroc_config_vo.json` or `data/sitl_config_vo.json`. No existing VO member, residual or optimisation step is touched.

## 9. Threshold determination from logged flight data

This section performs the cross-run analysis the user asked for, joining `vio_imu_frame.csv`, `vio_state.csv` and `of_frame.csv` on `t_ns` across all eight experiments under `/ws/logs/slam`, the same methodology `context/stationary_detection.md` section 3.3 and 3.4 used for `phase_3b_1` alone, extended here to the full campaign. 25125 common frames join across all eight runs, of which 165 are truly stationary (`v_norm < 0.02` m/s) and 24653 are unambiguously moving (`v_norm > 0.1` m/s). The truly stationary population is heavily concentrated in three runs, `phase_3a_1` (27), `phase_3b_1` (61) and `phase_3b_2` (51), while the other five contribute 14 or fewer genuinely stationary frames each (`phase_2_2` 14, `phase_1_1` 4, `phase_3a_2` 4, `phase_2_1` 3, `phase_1_2` 1), which should be read as a caveat on how precisely any threshold's true positive rate is actually pinned down by this campaign, not as a defect in the method.

### Stage 1, persistence against the existing per-frame test, all eight runs combined

The existing `vio_imu_frame.csv` fields, `accel_deviation_max` and `gyro_norm_max`, are per-frame maxima, not the raw per-sample window the real windowed SHOE statistic needs, so this sweep uses them as a conservative proxy, requiring `N` consecutive frames to each individually clear an `(accelTol, gyroTol)` pair, which is at least as strict as a true windowed mean energy statistic since a max dominates a mean.

| `gyroTol` | `minConsecutiveFrames` | TPR | FPR |
|---|---|---|---|
| 0.02 | 1 | 0.800 | 0.046 |
| 0.02 | 2 | 0.642 | 0.017 |
| 0.02 | 3 | 0.521 | 0.011 |
| 0.02 | 4 | 0.430 | 0.008 |
| 0.03 | 2 | 0.879 | 0.135 |
| 0.05 | 2 | 0.879 | 0.423 |

`accelTol = 0.2` is held fixed throughout, consistent with section 3.4's finding that it barely moves the frontier once `gyroTol` is tightened, reconfirmed here across the full campaign. `gyroTol = 0.02` at `minConsecutiveFrames = 1` reproduces section 3.4's `phase_3b_1`-only result almost exactly, 0.800/0.046 here against 0.803/0.043 there, which cross-validates the two analyses. Raising persistence trades true positive rate for false positive rate quickly, since several of the eight runs contain only a handful of genuinely stationary frames at mission start or end, so a high `minConsecutiveFrames` can miss a short true rest entirely, directly working against the takeoff sensitivity section 4.6 of the context document asks for. `mpStationaryMinConsecutiveFrames = 2` is chosen as the default, TPR 0.642 and FPR 0.017, a 2.7x FPR reduction against `minConsecutiveFrames = 1` while keeping the persistence window to roughly 0.1 to 0.2 seconds of flight.

The real implementation's `tShoe` is a window-mean chi-squared-like statistic, not this proxy, so `mpStationaryGlrtTol` is set from the chi-squared derivation in `context/stationary_detection.md` section 2.1 instead of from this sweep. Per-sample, the bracketed SHOE term is `χ²_6`, so `W` samples give `χ²_{6W}` before the gravity direction's own 2 degrees of freedom are subtracted, `χ²_{6W-2}`, and the 95% critical value of that distribution divided by `W` gives the threshold on the window mean. At `mpStationaryWindowS = 0.5` s and this platform's 167 Hz IMU rate, `W ≈ 83`, `χ²_{496,0.95} ≈ 549` by the Wilson-Hilferty approximation, giving `mpStationaryGlrtTol ≈ 549 / 83 ≈ 6.6`. The approximation is insensitive to the exact window length chosen, ranging from 6.9 at `W = 42` to 6.4 at `W = 167`, so 6.6 is a reasonable default across the whole 0.25 to 1 second range section 4.1 proposes.

### Stage 2, pending empirical validation

Neither `dv` (`IntegratedImuMeasurement::getDeltaState().vel_w_i`) nor `Sigma_v` (`IntegratedImuMeasurement::get_cov()`'s velocity block) is logged by any existing CSV, so this quantity cannot be reconstructed retroactively from the eight already-logged experiments with the precision a Mahalanobis test needs, only approximated crudely from per-frame aggregates, which this plan declines to do since change 5 logs `d2` directly going forward. `mpStationaryChiSqTol = 7.81` is therefore set from the exact chi-squared(3) 95% critical value `context/stationary_detection.md` section 4.2 derives, not from an empirical sweep. The validation strategy below names the one step needed to close this gap, a single fresh logged run after this change lands gives `d2`'s own empirical distribution directly, which section 3.4's methodology can then sweep exactly as it already did for the old test.

### Stage 3, flow magnitude veto, all eight runs combined

Applying a flow veto on top of the existing default IMU test's flagged set, `accelTol = 0.5, gyroTol = 0.05`, across all eight experiments, 13132 frames are flagged stationary while truly moving and 162 are flagged stationary and truly are,

| `flowTol` (px) | Reclassified moving | Vetoed stationary |
|---|---|---|
| 0.5 | 7788 (59.3%) | 17 (10.5%) |
| 0.7 | 5374 (40.9%) | 7 (4.3%) |
| 0.8 | 4621 (35.2%) | 3 (1.9%) |
| 1.0 | 3429 (26.1%) | 1 (0.6%) |
| 1.2 | 2631 (20.0%) | 0 (0.0%) |

consistent with section 4.5's single-run finding of a reasonable compromise near 0.7 to 0.8 px, now confirmed at full campaign scale. `mpStationaryFlowTol = 0.8` is chosen, reclassifying over a third of the false positives at under 2% cost against true positives.

### Combined operating point

Stage 1 at `(accelTol=0.2, gyroTol=0.02, minConsecutiveFrames=2)` ANDed with stage 3 at `flowTol=0.8` gives an aggregate TPR of 0.630 and FPR of 0.015 across all eight runs by the same proxy methodology, computed for sanity rather than as the shipped default, since stage 2, the component this document's own argument says is doing the real work against sustained cruise, cannot be included in this particular sweep for the reason given above.

## 10. Backwards compatibility summary

No change in this plan alters the behaviour of any caller that did not ask for it, except `mpIsStationary`'s own computation, which is the change requested. That flag's only consumer is the zero velocity soft constraint at `src/vi_estimator/sqrt_keypoint_vio.cpp:1583-1807`, gated on `mpZeroVelocityWeight > 0`, zero in every configuration actually read by `SqrtKeypointVioEstimator` today, so this plan has no live optimiser effect until an operator separately enables that weight. `SqrtKeypointVoEstimator` gains only an opt-in, default-off diagnostic log with no behavioural side effect. All config and CSV schema changes are additive.

## 11. Validation strategy

1. Build and run against each of the eight existing bags with the new code and `vio_stationary_enable_flow` temporarily forced true in every config, to confirm `vio_stationary.csv` and, where applicable, `vo_stationary.csv` populate correctly and that `t_shoe`, `d2` and `flow_mean` take plausible values, `t_shoe` near 6 at rest, `d2` near 0 to 3 at rest given 3 degrees of freedom.
2. Re-run section 9's TPR/FPR sweep against the freshly logged `d2` column directly, closing the one gap this plan could not close against the existing eight logs, and retune `mpStationaryChiSqTol` if the empirical sweep disagrees materially with the theoretical 7.81.
3. Confirm `mpStationaryRunLength` grows from zero at the start of each of the eight missions rather than starting pre-warmed, verifying the growing window behaves as section 4.6 intends.
4. With `vio_zero_velocity_weight` raised above zero in one SITL configuration, confirm the optimised trajectory's stationary-phase drift improves without introducing spurious corrections during the cruise segments the new detector should no longer flag, repeating the kind of before/after comparison `plans/vio_drift_analysis.md` Phase 3b already established for the original constraint.

## 12. Change 8, the certified ground interval gate (PLAN 2026-10-25, awaiting review)

### 12.1 Motivation

Measured on the multirotor, every stage and every combination of changes 1 to 6 declares the vehicle stationary during flight, SHOE on 57.9 percent, flow on 37.3 percent and both together on 28.1 percent of 30003 moving frames across 13 SITL runs (`context/stationary_detection.md` section 3.5). Enabling `vio_zero_velocity_weight` at the value of 100 that `plans/vio_drift_analysis.md` derives would then pull a vehicle flying at up to 5 m/s towards zero with a confidence of 1 cm/s. Section 2.7 of the context document shows that no threshold test can reach zero false stationary, since constant velocity flight and a distant scene make moving windows statistically identical to still ones. The requirement is zero false stationary, so the gate must take stillness from a signal that is zero by construction while the vehicle can move.

### 12.2 High-level design

The gate certifies stillness from ArduCopter's landed state, published as `ardupilot_msgs/Status.flying` on `/ap/status`. A state at time `t` is certified when two consecutive status messages stamped `s_k ≤ t ≤ s_{k+1}` both report landed and are no further apart than twice the heartbeat period, with a margin `ε` at both ends (`context/stationary_detection.md` section 4.8). ArduCopter keeps `land_complete` true from arming until the motors reach `THROTTLE_UNLIMITED` and forces the throttle to decay until then (`ArduCopter/takeoff.cpp:143-149`), so no state after the last landed message before take-off can ever be certified, however late messages arrive. The false stationary rate is zero for self-propelled motion, independently of latency, noise and scene. The existing three-stage evidence is kept as an optional conjunct, which also catches a landed vehicle that is being carried.

Certification needs the next landed message, so it arrives up to one heartbeat after the state. The zero velocity residual of `plans/vio_drift_analysis.md` Phase 3b is therefore extended from the newest state to every certified state in the window, and the ArduPilot status heartbeat is raised from 2 Hz to 20 Hz at build time so that every state is certified before marginalisation.

A Basalt-internal fallback for datasets without an autopilot, an OpenVINS-style start latch, is added behind its own flag. It reached 0.02 percent false stationary on the 13 runs, not zero, and is documented as such. The latch was implemented first, on 2026-10-26, so that it can be flown before the ArduPilot side of the gate exists (section 12.10).

Every new behaviour is behind a flag that defaults off, so every existing configuration, caller and log stays bit identical.

### 12.3 Change 8a, configuration

`include/basalt/utils/vio_config.h`, after `int vio_stationary_latch_frames;` of section 12.10,

```cpp
    // Certified ground interval gate, plans/stationary_detection.md change 8.
    // A state is stationary only between two consecutive /ap/status messages
    // that both report landed. Suggested true with vio_zero_velocity_weight
    // 100 on ArduPilot platforms built with AP_DDS_DELAY_STATUS_TOPIC_MS=10.
    bool vio_stationary_enable_ground_certificate;
    // Margin applied at both ends of a certified interval, bounding the
    // misalignment of frame and status timestamps. Zero in SITL.
    double vio_stationary_certificate_margin_s;
    // Longest gap between two landed messages that still certifies the
    // interval between them, twice the 50 ms heartbeat at 20 Hz.
    double vio_stationary_certificate_max_gap_s;
```

`src/utils/vio_config.cpp`, constructor defaults `false`, `0.01` and `0.1`, and three `ar(CEREAL_NVP(...))` lines after those of the start latch. Cereal is fatal on a missing key (`context/vio_config_json.md`), so all eight configuration sources gain the three keys at these defaults. The two start latch keys, `vio_stationary_enable_start_latch` and `vio_stationary_latch_frames`, already exist (section 12.10). `data/sitl_config_vo.json` is set by the operator, not by this change.

Backwards compatibility. Both gates default off, so the configuration layer changes no behaviour.

### 12.4 Change 8b, the ground state channel

`include/basalt/vi_estimator/vio_estimator.h`, beside `imu_data_queue` in `VioEstimatorBase`,

```cpp
    struct GroundState {
        int64_t t_ns;   // status stamp in the IMU clock
        bool landed;    // !ardupilot_msgs/Status.flying
    };
    tbb::concurrent_bounded_queue<GroundState> ground_state_queue;
```

with `this->ground_state_queue.set_capacity(100);` in the constructor. `include/basalt/controller.h` and `src/controller.cpp` gain

```cpp
void Controller::GrabGroundState(int64_t tNs, bool landed) {
    if (vio_estimator_) vio_estimator_->ground_state_queue.try_push({tNs, landed});
}
```

`try_push` never blocks the ROS callback, and a full queue drops the newest message, which the gap check of change 8c then treats as a break in certification rather than as a landed interval.

`ros_ws/src/slam/src/basalt/node.cpp` gains an `ardupilot_status_topic` parameter, empty by default, and when it is set a subscription with the publisher's QoS, reliable and transient local (`libraries/AP_DDS/AP_DDS_Topic_Table.h:347-353`), at a depth of 10,

```cpp
    if (!mpArdupilotStatusTopicName.empty()) {
        mpStatusSubscriber = this->create_subscription<StatusMsg>(
            mpArdupilotStatusTopicName,
            rclcpp::QoS(10).reliable().transient_local(),
            [this](const StatusMsg::SharedPtr msg) {
                std::shared_lock<std::shared_mutex> slamLock(mpMtxSlam);
                if (!mpSlam) return;
                mpSlam->GrabGroundState(rclcpp::Time(msg->header.stamp).nanoseconds(),
                                        !msg->flying);
            });
    }
```

`ros_ws/src/slam/package.xml` and `CMakeLists.txt` gain `ardupilot_msgs`, which `controllers` already depends on (`controllers/package.xml:27,39`). `controllers/launch/basalt_slam_test.launch.py` already declares `ap_status_topic_name` with default `/ap/status` (`:118-125`) and passes it to the driver, and it is forwarded to `basalt_slam.launch.py` as `ardupilot_status_topic` as well. Both status stamps and IMU stamps come from ArduPilot's clock through AP_DDS, so the stamp needs no conversion beyond the existing IMU path.

Backwards compatibility. With the parameter empty the node creates no subscription, and with the gate disabled the estimator never reads the queue, so no existing run changes.

### 12.5 Change 8c, certification inside `SqrtKeypointVioEstimator`

New members in `include/basalt/vi_estimator/sqrt_keypoint_vio.h`,

```cpp
    std::deque<GroundState> mvpGroundStates;      // received, time ordered
    std::map<int64_t, bool> mpStationaryEvidence; // three-stage result per state
    std::vector<int64_t> mvpZeroVelocityStates;   // states given r = w v this cycle

    // True when t lies inside an interval between two consecutive landed
    // status messages no further apart than the maximum gap.
    bool IsGroundCertified(int64_t tNs) const;
```

`ProcessFrame`, after the existing `mpIsStationary` assignment at `src/vi_estimator/sqrt_keypoint_vio.cpp:485-490`, keeps that expression as the evidence and composes the gate,

```cpp
        const bool evidence = mpIsStationary;
        if (mpStationaryEnableGroundCertificate) {
            GroundState gs;
            while (ground_state_queue.try_pop(gs)) mvpGroundStates.push_back(gs);
            mpStationaryEvidence[curr_frame->t_ns] = evidence;
            mvpZeroVelocityStates.clear();
            for (const auto& [tNs, state] : frame_states)
                if (IsGroundCertified(tNs) && mpStationaryEvidence[tNs])
                    mvpZeroVelocityStates.push_back(tNs);
            mpIsStationary = !mvpZeroVelocityStates.empty();
        } else {
            // The start latch of section 12.10 has already been applied to
            // mpIsStationary at this point.
            mvpZeroVelocityStates.assign(
                mpIsStationary ? 1 : 0, last_state_t_ns);
        }
```

with `IsGroundCertified` scanning `mvpGroundStates` for the consecutive pair that brackets `t` and returning true only when both are landed, `s_{k+1} − s_k ≤ vio_stationary_certificate_max_gap_s` and `t` lies at least the margin inside both ends. `mvpGroundStates` and `mpStationaryEvidence` are pruned to entries no older than the oldest key of `frame_states` less the maximum gap, which bounds both at a few dozen entries. With every stage of the evidence disabled, the three-stage expression is false, so the gate requires at least one evidence stage enabled, the flow stage in the suggested configuration, and the plan does not change that rule.

### 12.6 Change 8d, the zero velocity residual on every certified state

`plans/vio_drift_analysis.md` Phase 3b applies `r = w v` to `last_state_t_ns` at four sites of `optimize`, the cost in `error_total` (`:1655-1664`), the normal equations (`:1743-1751`), the model decrease `l_diff` (`:1805-1812`) and `after_error_total` (`:1875-1879`). Each site becomes a loop over `mvpZeroVelocityStates`, for example in the normal equations,

```cpp
                    for (const int64_t tZv : mvpZeroVelocityStates) {
                        const int vi = aom.abs_order_map.at(tZv).first + 6;
                        H.template block<3, 3>(vi, vi).diagonal().array() +=
                            zeroVelocityW2;
                        b.template segment<3>(vi) +=
                            zeroVelocityW2 *
                            frame_states.at(tZv).getState().vel_w_i;
                    }
```

and `applyZeroVelocity` becomes `mpZeroVelocityWeight > 0 && !mvpZeroVelocityStates.empty()`. Each residual is independent and touches only its own state's velocity block, so the step acceptance algebra of Phase 3b holds term by term.

Backwards compatibility. When the certificate is disabled, change 8c sets `mvpZeroVelocityStates` to `{last_state_t_ns}` exactly when `mpIsStationary` is true, so each loop runs once on the same state as today or not at all, and the optimiser is bit identical for every existing configuration.

### 12.7 Change 8e, the 20 Hz status heartbeat

AP_DDS evaluates the status every `AP_DDS_DELAY_STATUS_TOPIC_MS`, 100 ms by default, and repeats an unchanged status every five evaluations, 2 Hz (`libraries/AP_DDS/AP_DDS_Client.cpp:780-787,1967-1972`). Basalt's window holds `vio_max_states = 3` states, about 0.15 s, so at 2 Hz a state is marginalised before the landed message that certifies it arrives. The define is guarded by `#ifndef` (`AP_DDS_config.h:109-111`) and waf's `--define` option (`Tools/ardupilotwaf/ardupilotwaf.py:712`, `boards.py:259-263`) sets it without editing the submodule. At 10 ms the status is evaluated at 100 Hz and repeated at 20 Hz, so a state is certified about 50 ms after it is created.

1. SITL. `ardu_ws/scripts/build.sh` passes the define through the cache variable `ARDUPILOT_WAF_CONFIGURE_ARGS`, which `Tools/ros2/ardupilot_sitl/CMakeLists.txt:21,51` appends to `waf configure`,

```bash
colcon build --cmake-args -DBUILD_TESTING=ON "-DARDUPILOT_WAF_CONFIGURE_ARGS=--define=AP_DDS_DELAY_STATUS_TOPIC_MS=10"
```

2. Drones. `djinn flash ardupilot` (`/ws/plans/arducopter_takeoff_behaviour.md` section 4.3) adds `--define=AP_DDS_DELAY_STATUS_TOPIC_MS=10` to its `waf configure`.

Backwards compatibility. Only the status rate changes. The SLAM driver (`ros_ws/src/controllers/controllers/slam/driver_node.py`) and the video logger react to state changes, which still arrive at once, and a status message is a few bytes, so 20 Hz is negligible on DDS. `djinn build ardupilot` is unchanged. Both entries are recorded in `/ws/context/djinn-workflows.md` when implemented.

### 12.8 Change 8f, logging

`vio_stationary.csv` gains `certified`, `latch_open` and `zero_velocity_states`, and a new `vio_ground_state.csv` records every drained status message (`t_ns`, `landed`, `gap_s`). Both are written only when `vio_debug` is true, like every other `mpLogger` record. The existing columns keep their names and meanings, and `scripts/analysis` reads columns by name (`scripts/analysis/data_reader.py:40,61`).

### 12.9 Validation

1. Offline, already run on 2026-10-25. The false stationary rates of `context/stationary_detection.md` section 3.5 for every existing stage, the start latch at 0.02 percent, the 54 ms minimum gap between `NOT_LANDED` and motion over about 50 take-offs, and the 1.0 cm maximum motion over 87 landed intervals of 559 s.
2. Build. `colcon build --packages-select slam` on the host, and the in-container syntax check of `src/vi_estimator/sqrt_keypoint_vio.cpp`, `src/controller.cpp` and `src/utils/vio_config.cpp` (`/ws/skills/cpp-syntax-check-tooling.md`).
3. Bit identity. A replay of `/ws/data/telemetry_18` with the gate disabled gives the same `vio_solver_iter.csv` as before the change, within the run-to-run non-determinism of `context/takeoff_bias_divergence.md` §5a, which a VO-only or single-threaded comparison removes.
4. Certificate. A SITL run with the gate enabled and the 20 Hz build shows `vio_ground_state.csv` at 20 Hz with every gap under 0.1 s on the ground, `certified` true on the ground and false on every frame from the first `flying` message, and, joined with the dataflash `NOT_LANDED` event and `SIM` position, no certified frame after motion onset.
5. Effect. With `vio_zero_velocity_weight` 100 and the flow stage as evidence, the estimated speed on the ground stays under 0.02 m/s, no landmark is triangulated before take-off, and the vertical bias at take-off plus 1 s is compared with `context/takeoff_bias_divergence.md` §8 and §9 over several runs.

### 12.10 Change 8g, the start latch, implemented 2026-10-26

The latch closes the stationarity flag for the rest of the run once `vio_stationary_latch_frames` consecutive frames have been non-stationary, in the manner of OpenVINS's `zupt_only_at_beginning` (`context/stationary_detection.md` section 2.7, [14]). It is the part of change 8 that needs no autopilot, so it was implemented ahead of the rest.

`include/basalt/utils/vio_config.h`, after `double vio_stationary_flow_tol;`,

```cpp
    // One-way start latch, plans/stationary_detection.md change 8. Once
    // vio_stationary_latch_frames consecutive frames are non-stationary, no
    // later frame is stationary for the rest of the run. Suggested true with
    // 3 frames on platforms that only rest before take-off.
    bool vio_stationary_enable_start_latch;
    int vio_stationary_latch_frames;
```

`src/utils/vio_config.cpp` defaults them to `false` and `3` and serialises them after `vio_stationary_flow_tol`, and all eight configuration sources carry `"config.vio_stationary_enable_start_latch": false` and `"config.vio_stationary_latch_frames": 3` after `vio_stationary_flow_tol` (the `.toml` uses its own assignment syntax). `include/basalt/vi_estimator/sqrt_keypoint_vio.h` gains `mpStationaryEnableStartLatch`, `mpStationaryLatchFrames`, `mpStationaryLatchMisses` and `mpStationaryLatchOpen`, read in the constructor beside `mpStationaryFlowTol`. `SqrtKeypointVioEstimator::ProcessFrame` applies the latch immediately after the three-stage expression and before both log calls,

```cpp
        if (mpStationaryEnableStartLatch) {
            mpStationaryLatchMisses =
                mpIsStationary ? 0 : mpStationaryLatchMisses + 1;
            if (mpStationaryLatchMisses >= mpStationaryLatchFrames)
                mpStationaryLatchOpen = false;
            mpIsStationary = mpIsStationary && mpStationaryLatchOpen;
        }
```

so `is_stationary` in `vio_imu_frame.csv` and `vio_stationary.csv` records the latched flag, and the stage columns still record the evidence. A closed latch is visible as `is_stationary` 0 on a frame whose enabled stages are all 1. The latch counts frames whose composed evidence is false, so the stages it acts on are those enabled in the configuration, the flow stage alone in the measurement of `context/stationary_detection.md` section 3.5. With every stage disabled the evidence is always false and the latch closes after three frames, which changes nothing since the flag was already false. `BasaltSLAMNode` builds a new `BasaltSLAM`, and with it a new estimator, on every activation (`ros_ws/src/slam/src/basalt/node.cpp:164`), so each flight starts with the latch open.

Rationale for three frames. A latch that closes on the first non-stationary frame keeps only 48.5 percent of the ground phase, because the flow stage drops out on 23 to 38 percent of frames during motor spool-up, while three frames keep 93.8 percent with the same 5 false stationary frames of 30003 (`context/stationary_detection.md` section 3.5). Those five frames come from one run whose lift-off flow stayed under the 0.8 px threshold for 0.25 s, so the latch narrows the false stationary rate to 0.02 percent on this campaign but cannot guarantee zero. The certified ground interval gate of sections 12.2 to 12.6 removes that remainder.

Backwards compatibility. `vio_stationary_enable_start_latch` defaults to false in the code and in every configuration source, so the block never runs and every existing configuration is bit identical. A `config.json` saved by an earlier run under `/ws/logs/slam` lacks both keys and must have them added before it is replayed, since cereal is fatal on a missing key.

Validation. Run on 2026-10-26 in the dev container, `src/vi_estimator/sqrt_keypoint_vio.cpp` and `src/utils/vio_config.cpp` pass the syntax check of `/ws/skills/cpp-syntax-check-tooling.md`, and all eight configuration sources parse. Outstanding on the host, `colcon build --packages-select slam`, then a SITL run with `vio_stationary_enable_flow` true, the other two stages false, `vio_stationary_enable_start_latch` true and `vio_zero_velocity_weight` 100, which must show `is_stationary` 1 on the ground, 0 from lift-off onwards on every frame, and an estimated speed on the ground under 0.02 m/s.

## 13. References

`context/stationary_detection.md`, especially section 2.1 (GLRT and chi-squared derivation), section 2.3 (Kalman filter and the ZUPT mechanism), section 4.1 (growing window SHOE), section 4.2 and 4.4 (the Mahalanobis tests and their combination), section 4.5 (flow magnitude), and section 4.6 (the two-stage design this plan extends to three stages). `context/vio_config_json.md` for the cereal configuration contract. `plans/vio_drift_analysis.md` Phase 3b for the zero velocity constraint this detector gates and the established pattern for presenting a weighted pseudo-measurement change.

`context/stationary_detection.md` sections 2.7, 3.5 and 4.8 for the literature, measurements and derivation behind change 8, including OpenVINS [14], the PX4 and ArduPilot land detectors [15, 16] and contact-gated zero velocity in legged robots and road vehicles [17, 18, 19].
