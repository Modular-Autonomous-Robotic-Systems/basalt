/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <basalt/optimization/accumulator.h>
#include <basalt/utils/assert.h>
#include <basalt/utils/system_utils.h>
#include <basalt/vi_estimator/marg_helper.h>
#include <basalt/vi_estimator/sc_ba_base.h>
#include <basalt/vi_estimator/sqrt_keypoint_vio.h>
#include <fmt/format.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <basalt/linearization/linearization_base.hpp>
#include <basalt/linearization/linearization_abs_qr.hpp>
#include <basalt/utils/cast_utils.hpp>
#include <basalt/utils/format.hpp>
#include <basalt/utils/time_utils.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include "basalt/imu/imu_types.h"

namespace basalt {

template <class Scalar_>
SqrtKeypointVioEstimator<Scalar_>::SqrtKeypointVioEstimator(
    const Eigen::Vector3d& g_, const basalt::Calibration<double>& calib_,
    const VioConfig& config_, bool useProducerConsumerArchitecture,
    const Logger::Ptr& logger)
    : VioEstimatorBase<Scalar_>(),
      take_kf(true),
      frames_after_kf(0),
      g(g_.cast<Scalar>()),
      initialized(false),
      config(config_),
      lambda(config_.vio_lm_lambda_initial),
      min_lambda(config_.vio_lm_lambda_min),
      max_lambda(config_.vio_lm_lambda_max),
      lambda_vee(2),
      mpLogger(logger ? logger : Logger::Disabled()),
      mpUseProducerConsumerArchitecture(useProducerConsumerArchitecture) {
    obs_std_dev = Scalar(config.vio_obs_std_dev);
    huber_thresh = Scalar(config.vio_obs_huber_thresh);
    mpZeroVelocityWeight = Scalar(config.vio_zero_velocity_weight);
    mpStationaryEnableShoe = config.vio_stationary_enable_shoe;
    mpStationaryEnableMahalanobis = config.vio_stationary_enable_mahalanobis;
    mpStationaryEnableFlow = config.vio_stationary_enable_flow;
    mpStationaryWindowS = Scalar(config.vio_stationary_window_s);
    mpStationaryGlrtTol = Scalar(config.vio_stationary_glrt_tol);
    mpStationaryMinConsecutiveFrames =
        config.vio_stationary_min_consecutive_frames;
    mpStationaryChiSqTol = Scalar(config.vio_stationary_chisq_tol);
    mpStationaryFlowTol = config.vio_stationary_flow_tol;
    mpStationaryEnableStartLatch = config.vio_stationary_enable_start_latch;
    mpStationaryLatchFrames = config.vio_stationary_latch_frames;
    this->calib = calib_.cast<Scalar>();
    std::cout << "vio debug mode: " << config.vio_debug << std::endl;

    // Setup marginalization
    marg_data.is_sqrt = config.vio_sqrt_marg;
    marg_data.H.setZero(POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE);
    marg_data.b.setZero(POSE_VEL_BIAS_SIZE);

    // Version without prior
    nullspace_marg_data.is_sqrt = marg_data.is_sqrt;
    nullspace_marg_data.H.setZero(POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE);
    nullspace_marg_data.b.setZero(POSE_VEL_BIAS_SIZE);

    if (marg_data.is_sqrt) {
        // prior on position
        marg_data.H.diagonal().template head<3>().setConstant(
            std::sqrt(Scalar(config.vio_init_pose_weight)));
        // prior on yaw
        marg_data.H(5, 5) = std::sqrt(Scalar(config.vio_init_pose_weight));
        // prior on roll and pitch relative to gravity,
        // plans/gravity_and_bias_step_priors.md
        if (config.vio_gravity_prior_std_deg > 0) {
            const Scalar sigma =
                Scalar(config.vio_gravity_prior_std_deg * M_PI / 180.0);
            marg_data.H(3, 3) = marg_data.H(4, 4) = Scalar(1) / sigma;
        }

        // small prior to avoid jumps in bias
        marg_data.H.diagonal().template segment<3>(9).array() =
            std::sqrt(Scalar(config.vio_init_ba_weight));
        marg_data.H.diagonal().template segment<3>(12).array() =
            std::sqrt(Scalar(config.vio_init_bg_weight));
    } else {
        // prior on position
        marg_data.H.diagonal().template head<3>().setConstant(
            Scalar(config.vio_init_pose_weight));
        // prior on yaw
        marg_data.H(5, 5) = Scalar(config.vio_init_pose_weight);
        // prior on roll and pitch relative to gravity,
        // plans/gravity_and_bias_step_priors.md
        if (config.vio_gravity_prior_std_deg > 0) {
            const Scalar sigma =
                Scalar(config.vio_gravity_prior_std_deg * M_PI / 180.0);
            marg_data.H(3, 3) = marg_data.H(4, 4) =
                Scalar(1) / (sigma * sigma);
        }

        // small prior to avoid jumps in bias
        marg_data.H.diagonal().template segment<3>(9).array() =
            Scalar(config.vio_init_ba_weight);
        marg_data.H.diagonal().template segment<3>(12).array() =
            Scalar(config.vio_init_bg_weight);
    }

    if (config.vio_gs_enable) {
        BASALT_ASSERT_STREAM(
            config.vio_linearization_type == LinearizationType::ABS_QR &&
                config.vio_sqrt_marg,
            "vio_gs_enable requires ABS_QR linearization and vio_sqrt_marg");

        MatX H_gs = MatX::Zero(marg_data.H.rows(),
                               GRAVITY_SCALE_SIZE + POSE_VEL_BIAS_SIZE);
        H_gs.rightCols(POSE_VEL_BIAS_SIZE) = marg_data.H;
        // The roll and pitch prior penalises the first state's tilt relative
        // to the block's gravity direction, which keeps it gauge invariant.
        if (config.vio_gravity_prior_std_deg > 0) {
            const Scalar sigma =
                Scalar(config.vio_gravity_prior_std_deg * M_PI / 180.0);
            H_gs(3, 0) = H_gs(4, 1) = -Scalar(1) / sigma;
        }
        marg_data.H = H_gs;

        this->mGsState.emplace();
        this->mGsCorrectionLog = std::make_shared<GravityScaleCorrectionLog>();
        mpGsActive = true;

        BASALT_ASSERT_STREAM(config.vio_gs_dm_delay >= 1,
                             "vio_gs_dm_delay must be at least 1");
        mpDm = std::make_unique<DelayedMarginalisation<Scalar>>(
            config.vio_gs_dm_delay);
    }

    std::cout << "marg_H (sqrt:" << marg_data.is_sqrt << ")\n"
              << marg_data.H << std::endl;

    gyro_bias_sqrt_weight = this->calib.gyro_bias_std.array().inverse();
    accel_bias_sqrt_weight = this->calib.accel_bias_std.array().inverse();

    max_states = config.vio_max_states;
    max_kfs = config.vio_max_kfs;

    opt_started = false;

    this->vision_data_queue.set_capacity(10);
    this->imu_data_queue.set_capacity(300);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::initialize(
    int64_t t_ns, const Sophus::SE3d& T_w_i, const Eigen::Vector3d& vel_w_i,
    const Eigen::Vector3d& bg, const Eigen::Vector3d& ba) {
    initialized = true;
    T_w_i_init = T_w_i.cast<Scalar>();

    last_state_t_ns = t_ns;
    imu_meas[t_ns] = IntegratedImuMeasurement<Scalar>(t_ns, bg.cast<Scalar>(),
                                                      ba.cast<Scalar>());
    frame_states[t_ns] = PoseVelBiasStateWithLin<Scalar>(
        t_ns, T_w_i_init, vel_w_i.cast<Scalar>(), bg.cast<Scalar>(),
        ba.cast<Scalar>(), true);

    SetInitialPriorOrder(t_ns);

    nullspace_marg_data.order = marg_data.order;

    initialize(bg, ba);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::initialize(const Eigen::Vector3d& bg_,
                                                   const Eigen::Vector3d& ba_) {
    Vec3 bg_init = bg_.cast<Scalar>();
    Vec3 ba_init = ba_.cast<Scalar>();
    this->mpBg = bg_init;
    this->mpBa = ba_init;
    this->mpAccelCov =
        this->calib.dicrete_time_accel_noise_std().array().square();
    this->mpGyroCov =
        this->calib.dicrete_time_gyro_noise_std().array().square();

    if (mpUseProducerConsumerArchitecture) {
        auto proc_func = [&] {
            OpticalFlowResult::Ptr curr_frame;

            while (true) {
                this->vision_data_queue.pop(curr_frame);

                if (config.vio_enforce_realtime) {
                    // drop current frame if another frame is already in the
                    // queue.
                    while (!this->vision_data_queue.empty())
                        this->vision_data_queue.pop(curr_frame);
                }
                typename PoseVelBiasState<Scalar>::Ptr output_state =
                    this->ProcessFrame(curr_frame);
                if (output_state == nullptr) {
                    std::cout << "vio output state null exiting" << std::endl;
                    break;
                }
            }
            std::cout << "providing nullptr to downstream queues" << std::endl;
            if (this->out_vis_queue) this->out_vis_queue->push(nullptr);
            if (this->out_marg_queue) this->out_marg_queue->push(nullptr);
            if (this->mpKFOutputQueue) this->mpKFOutputQueue->push(nullptr);
            if (this->out_state_queue) this->out_state_queue->push(nullptr);

            this->finished = true;

            std::cout << "Finished VIOFilter " << std::endl;
        };

        processing_thread.reset(new std::thread(proc_func));
    } else {
        this->finished = false;
        this->prev_frame = nullptr;
        this->imuData = nullptr;
    }
}

template <class Scalar>
typename PoseVelBiasState<Scalar>::Ptr
SqrtKeypointVioEstimator<Scalar>::ProcessFrame(
    OpticalFlowResult::Ptr& curr_frame, std::optional<Sophus::SE3d> gtcw) {
    Timer tProcessFrame;
    mpCurrentGTPose = gtcw;
    if (!curr_frame.get() || this->finished) {
        std::cout << "received nullptr data from optical flow" << std::endl;
        if (this->out_vis_queue) this->out_vis_queue->push(nullptr);
        if (this->out_marg_queue) this->out_marg_queue->push(nullptr);
        if (this->mpKFOutputQueue) this->mpKFOutputQueue->push(nullptr);
        if (this->out_state_queue) this->out_state_queue->push(nullptr);
        this->finished = true;
        return nullptr;
    }

    // Counters describing how the inertial stream was consumed for this
    // frame. The preintegration interval is asserted to span the frame gap
    // exactly, so only these numbers reveal whether real samples filled it.
    const int64_t imuQueueSizeOnEntry = int64_t(this->imu_data_queue.size());
    int numImuIntegrated = 0;
    int numImuSkippedBehind = 0;
    int64_t firstIntegratedImuTNs = -1;
    int64_t lastIntegratedImuTNs = -1;
    bool stretchFallbackFired = false;
    int64_t stretchSpanNs = 0;

    if (this->imuData == nullptr) {
        this->imuData = popFromImuDataQueue();
        if (!this->imuData) {
            std::cout << "[VIO] imu starved on first pop, frame t_ns="
                      << curr_frame->t_ns << " dropped" << std::endl;
            return nullptr;
        }
        this->imuData->accel =
            this->calib.calib_accel_bias.getCalibrated(imuData->accel);
        this->imuData->gyro =
            this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
    }

    // Correct camera time offset
    // curr_frame->t_ns += calib.cam_time_offset_ns;
    typename IntegratedImuMeasurement<Scalar>::Ptr meas;

    if (!initialized) {
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
            // std::cout << "Skipping IMU data.." << std::endl;
        }

        if (!imuData) return nullptr;

        Vec3 vel_w_i_init;
        vel_w_i_init.setZero();

        // One sample carries 0.1521 m/s^2 of noise per axis, 0.89 degrees of
        // tilt, which the mean of N reduces by sqrt(N). See
        // plans/vio_drift_analysis.md section 2.2.3.
        const Vec3 accelMean = accelSum / Scalar(numAccelAveraged);
        const Vec3 gravityReference =
            config.vio_init_accel_averaging ? accelMean : imuData->accel;
        T_w_i_init.setQuaternion(Eigen::Quaternion<Scalar>::FromTwoVectors(
            gravityReference, Vec3::UnitZ()));

        // Turn the gravity aligned world about Z so body X takes the ground
        // truth heading. Tilt stays with the accelerometer, which the initial
        // bias state is consistent with.
        const bool yawAligned = mpCurrentGTPose.has_value();
        double gtYawRad = 0.0;
        double slamYawRad = 0.0;
        double yawOffsetRad = 0.0;
        if (mpCurrentGTPose) {
            const Eigen::Matrix<Scalar, 3, 3> rAcc = T_w_i_init.so3().matrix();
            const Eigen::Matrix3d rGt = mpCurrentGTPose->so3().matrix();
            slamYawRad = std::atan2(double(rAcc(1, 0)), double(rAcc(0, 0)));
            gtYawRad = std::atan2(rGt(1, 0), rGt(0, 0));
            // atan2(sin(x), cos(x)) wraps x to (-pi, pi]. rotZ is periodic in
            // its argument modulo 2 pi, so the correction applied is
            // unaffected, but the logged value is only directly comparable
            // against gt_yaw_rad - slam_yaw_rad, both already atan2 results,
            // if wrapped the same way.
            yawOffsetRad = std::atan2(std::sin(gtYawRad - slamYawRad),
                                      std::cos(gtYawRad - slamYawRad));
            T_w_i_init.so3() = Sophus::SO3<Scalar>::rotZ(Scalar(yawOffsetRad)) *
                               T_w_i_init.so3();
            T_w_i_init.translation() =
                mpCurrentGTPose->translation().template cast<Scalar>();
        }

        const Vec3 bodyZInWorld = T_w_i_init.so3() * Vec3::UnitZ();
        const double tiltDeg =
            std::acos(std::clamp(double(bodyZInWorld.z()), -1.0, 1.0)) * 180.0 /
            M_PI;

        last_state_t_ns = curr_frame->t_ns;
        imu_meas[last_state_t_ns] =
            IntegratedImuMeasurement<Scalar>(last_state_t_ns, mpBg, mpBa);
        frame_states[last_state_t_ns] = PoseVelBiasStateWithLin<Scalar>(
            last_state_t_ns, T_w_i_init, vel_w_i_init, mpBg, mpBa, true);

        SetInitialPriorOrder(last_state_t_ns);

        std::cout << "Setting up filter: t_ns " << last_state_t_ns << std::endl;
        std::cout << "T_w_i\n" << T_w_i_init.matrix() << std::endl;
        std::cout << "vel_w_i " << vel_w_i_init.transpose() << std::endl;

        mpLogger->AddVioInit(
            curr_frame->t_ns, imuData->t_ns, imuQueueSizeOnEntry,
            numImuSkippedBehind, imuData->accel.template cast<double>(),
            imuData->gyro.template cast<double>(), mpBg.template cast<double>(),
            mpBa.template cast<double>(), g.template cast<double>(),
            T_w_i_init.unit_quaternion().coeffs().template cast<double>(),
            tiltDeg, yawAligned, yawOffsetRad, gtYawRad, slamYawRad,
            config.vio_init_accel_averaging, numAccelAveraged,
            accelMean.template cast<double>());
        // mpLogger->PrintVioInit();

        if ((config.vio_debug || config.vio_extended_logging) && !mpGsActive) {
            logMargNullspace();
        }

        initialized = true;
    }

    double imuDrainSeconds = 0.0;
    if (this->prev_frame) {
        Timer tImuDrain;
        // preintegrate measurements

        auto last_state = frame_states.at(last_state_t_ns);

        meas.reset(new IntegratedImuMeasurement<Scalar>(
            this->prev_frame->t_ns, last_state.getState().bias_gyro,
            last_state.getState().bias_accel));

        BASALT_ASSERT_MSG(this->prev_frame->t_ns < curr_frame->t_ns,
                          "duplicate frame timestamps?! zero time delta leads "
                          "to invalid IMU integration.");

        bool imuAhead = imuData->t_ns > this->prev_frame->t_ns;
        while (!imuAhead) {
            if (!popFromImuDataQueueNonBlocking(imuData)) break;
            if (!imuData) return nullptr;
            numImuSkippedBehind++;
            imuData->accel =
                this->calib.calib_accel_bias.getCalibrated(imuData->accel);
            imuData->gyro =
                this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
            imuAhead = imuData->t_ns > this->prev_frame->t_ns;
        }

        Vec3 accelSum = Vec3::Zero();
        Vec3 gyroSum = Vec3::Zero();
        Scalar accelDeviationMax = Scalar(0);
        Scalar gyroNormMax = Scalar(0);
        if (imuAhead) {
            while (imuData->t_ns <= curr_frame->t_ns) {
                meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
                mvpStationaryWindow.push_back(
                    {imuData->t_ns, imuData->accel, imuData->gyro});
                while (!mvpStationaryWindow.empty() &&
                       double(mvpStationaryWindow.back().t_ns -
                              mvpStationaryWindow.front().t_ns) *
                               1e-9 >
                           double(mpStationaryWindowS))
                    mvpStationaryWindow.pop_front();
                accelSum += imuData->accel;
                gyroSum += imuData->gyro;
                const Scalar accelDeviation =
                    std::abs(imuData->accel.norm() - Scalar(9.81));
                if (accelDeviation > accelDeviationMax)
                    accelDeviationMax = accelDeviation;
                const Scalar gyroNorm = imuData->gyro.norm();
                if (gyroNorm > gyroNormMax) gyroNormMax = gyroNorm;
                if (firstIntegratedImuTNs < 0)
                    firstIntegratedImuTNs = imuData->t_ns;
                lastIntegratedImuTNs = imuData->t_ns;
                numImuIntegrated++;
                if (!popFromImuDataQueueNonBlocking(imuData)) break;
                if (!imuData) return nullptr;
                imuData->accel =
                    this->calib.calib_accel_bias.getCalibrated(imuData->accel);
                imuData->gyro =
                    this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
            }
        }

        if (meas->get_start_t_ns() + meas->get_dt_ns() < curr_frame->t_ns) {
            if (!imuData.get()) return nullptr;
            stretchFallbackFired = true;
            stretchSpanNs =
                curr_frame->t_ns - (meas->get_start_t_ns() + meas->get_dt_ns());
            int64_t tmp = imuData->t_ns;
            imuData->t_ns = curr_frame->t_ns;
            meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
            imuData->t_ns = tmp;
        }

        const int64_t frameDtNs = curr_frame->t_ns - this->prev_frame->t_ns;
        const double coverage =
            frameDtNs > 0 && lastIntegratedImuTNs > 0
                ? double(lastIntegratedImuTNs - this->prev_frame->t_ns) /
                      double(frameDtNs)
                : 0.0;
        const Vec3 accelMean = numImuIntegrated > 0
                                   ? Vec3(accelSum / Scalar(numImuIntegrated))
                                   : Vec3::Zero();
        const Vec3 gyroMean = numImuIntegrated > 0
                                  ? Vec3(gyroSum / Scalar(numImuIntegrated))
                                  : Vec3::Zero();
        // Superseded 2026-10-19.
        // mpIsStationary = numImuIntegrated > 0 &&
        //                 accelDeviationMax < mpStationaryAccelTol &&
        //                 gyroNormMax < mpStationaryGyroTol;

        Scalar tShoe = Scalar(0);
        if (!mvpStationaryWindow.empty()) {
            Vec3 aBar = Vec3::Zero();
            for (const auto& s : mvpStationaryWindow) aBar += s.accel;
            aBar /= Scalar(mvpStationaryWindow.size());
            const Vec3 gDir = aBar.normalized();
            for (const auto& s : mvpStationaryWindow) {
                const Vec3 accelResidual = s.accel - Scalar(9.81) * gDir;
                tShoe +=
                    accelResidual.cwiseAbs2().cwiseQuotient(mpAccelCov).sum() +
                    s.gyro.cwiseAbs2().cwiseQuotient(mpGyroCov).sum();
            }
            tShoe /= Scalar(mvpStationaryWindow.size());
        }
        const bool stage1Clears =
            !mvpStationaryWindow.empty() && tShoe < mpStationaryGlrtTol;
        mpStationaryRunLength = stage1Clears ? mpStationaryRunLength + 1 : 0;
        const bool stage1 =
            mpStationaryRunLength >= mpStationaryMinConsecutiveFrames;

        // Sigma_v is exactly zero before the first integrate() call this
        // frame, which would make the solve below singular.
        Scalar d2 = Scalar(0);
        bool stage2 = false;
        if (numImuIntegrated > 0) {
            const auto& state0 = frame_states.at(last_state_t_ns).getState();
            const Mat3 Sigma_v = meas->get_cov().template block<3, 3>(6, 6);
            const Vec3 dv = meas->getDeltaState().vel_w_i;
            const Scalar predDt = Scalar(meas->get_dt_ns()) * Scalar(1e-9);
            const Vec3 residual = dv + state0.T_w_i.so3().inverse() *
                                           (state0.vel_w_i + g * predDt);
            d2 = residual.transpose() * Sigma_v.ldlt().solve(residual);
            stage2 = d2 < mpStationaryChiSqTol;
        }

        const bool stage3 = curr_frame->mFlowPxMean < mpStationaryFlowTol;

        mpIsStationary =
            (!mpStationaryEnableShoe || stage1) &&
            (!mpStationaryEnableMahalanobis || stage2) &&
            (!mpStationaryEnableFlow || stage3) &&
            (mpStationaryEnableShoe || mpStationaryEnableMahalanobis ||
             mpStationaryEnableFlow);
        if (mpStationaryEnableStartLatch) {
            mpStationaryLatchMisses =
                mpIsStationary ? 0 : mpStationaryLatchMisses + 1;
            if (mpStationaryLatchMisses >= mpStationaryLatchFrames)
                mpStationaryLatchOpen = false;
            mpIsStationary = mpIsStationary && mpStationaryLatchOpen;
        }
        if (mpGsActive && mpGsStartNs < 0 && !mpIsStationary &&
            (!mpStationaryEnableStartLatch || !mpStationaryLatchOpen))
            mpGsStartNs = curr_frame->t_ns;
        mpLogger->AddVioImuFrame(
            curr_frame->t_ns, double(frameDtNs) * 1e-9, numImuIntegrated,
            int(std::round(double(frameDtNs) * 1e-9 *
                           this->calib.imu_update_rate)),
            numImuSkippedBehind, firstIntegratedImuTNs, lastIntegratedImuTNs,
            coverage, double(meas->get_dt_ns()) * 1e-9, stretchFallbackFired,
            double(stretchSpanNs) * 1e-9, int(imuQueueSizeOnEntry),
            int(this->imu_data_queue.size()),
            double(imuData ? imuData->t_ns - curr_frame->t_ns : 0) * 1e-9,
            accelMean.template cast<double>(), gyroMean.template cast<double>(),
            double(accelDeviationMax), double(gyroNormMax), mpIsStationary);
        // mpLogger->PrintVioImuFrame();

        mpLogger->AddVioStationary(
            curr_frame->t_ns, mpStationaryEnableShoe, double(tShoe),
            int(mvpStationaryWindow.size()), mpStationaryRunLength, stage1,
            mpStationaryEnableMahalanobis, double(d2), stage2,
            mpStationaryEnableFlow, curr_frame->mFlowPxMean, stage3,
            mpIsStationary);
        // mpLogger->PrintVioStationary();

        imuDrainSeconds = tImuDrain.elapsed();
    }

    const double processFrameSoFar = tProcessFrame.elapsed();
    typename PoseVelBiasState<Scalar>::Ptr output_state =
        measure(curr_frame, meas, imuDrainSeconds, processFrameSoFar);
    this->prev_frame = curr_frame;
    return output_state;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::addIMUToQueue(
    const ImuData<double>::Ptr& data) {
    this->imu_data_queue.emplace(data);
    std::cout << "[VIO] IMU Data Queue Size: " << this->imu_data_queue.size()
              << std::endl;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::addVisionToQueue(
    const OpticalFlowResult::Ptr& data) {
    this->vision_data_queue.push(data);
    std::cout << "[VIO] Frame Data Queue Size "
              << this->vision_data_queue.size() << std::endl;
}

template <class Scalar_>
bool SqrtKeypointVioEstimator<Scalar_>::popFromImuDataQueueNonBlocking(
    typename ImuData<Scalar>::Ptr& data) {
    ImuData<double>::Ptr raw;
    if (!this->imu_data_queue.try_pop(raw)) return false;

    if constexpr (std::is_same_v<Scalar, double>) {
        data = raw;
    } else {
        typename ImuData<Scalar>::Ptr converted;
        if (raw) {
            converted.reset(new ImuData<Scalar>);
            *converted = raw->cast<Scalar>();
        }
        data = converted;
    }
    return true;
}

template <class Scalar_>
typename ImuData<Scalar_>::Ptr
SqrtKeypointVioEstimator<Scalar_>::popFromImuDataQueue() {
    typename ImuData<Scalar>::Ptr data;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(mpImuPopTimeoutMs);

    while (!popFromImuDataQueueNonBlocking(data)) {
        if (std::chrono::steady_clock::now() >= deadline) return nullptr;
        std::this_thread::sleep_for(std::chrono::milliseconds(mpImuPopRetryMs));
    }
    return data;
}

template <class Scalar_>
typename PoseVelBiasState<Scalar_>::Ptr
SqrtKeypointVioEstimator<Scalar_>::measure(
    const OpticalFlowResult::Ptr& opt_flow_meas,
    const typename IntegratedImuMeasurement<Scalar>::Ptr& meas,
    double imuDrainSeconds, double processFrameSoFar) {
    mpLogger->SolverScratch()
        .add("frame_id", opt_flow_meas->t_ns)
        .format("none");
    Timer t_total;

    // ── Apply local-mapper pose corrections to frame_poses ──────────
    // Only frame_poses entries are updated (already-marginalised KFs with
    // pose-only state). frame_states entries are left untouched because
    // they are coupled to ongoing IMU preintegration and overwriting them
    // would violate the BASALT_ASSERT at line 352.
    Timer tPoseUpdate;
    int poseUpdatePending = 0, poseUpdateApplied = 0, poseUpdateRejected = 0;
    Scalar poseUpdateMaxTransErr = 0, poseUpdateMaxRotErr = 0;
    {
        std::lock_guard<std::mutex> lock(mpPosesToUpdateMutex);
        poseUpdatePending = int(mpPosesToUpdate.size());
        // Held while the delayed store is active, so that no readvance meets
        // a keyframe pose moved by the visual only mapper,
        // plans/delayed_marginalisation.md §2.3.8.
        if (mpDm && config.vio_gs_dm_hold_pose_updates) {
            poseUpdateRejected = poseUpdatePending;
            mpPosesToUpdate.clear();
            mpPosesToUpdateEpoch.clear();
        }
        if (!mpPosesToUpdate.empty()) {
            for (auto it = mpPosesToUpdate.begin();
                 it != mpPosesToUpdate.end();) {
                const int64_t t_ns = it->first;
                const PoseStateWithLin<double>& lm_pose = it->second;
                Sophus::SE3d T_new_d = lm_pose.getPose();
                const auto it_epoch = mpPosesToUpdateEpoch.find(t_ns);
                if (this->mGsCorrectionLog &&
                    it_epoch != mpPosesToUpdateEpoch.end()) {
                    T_new_d =
                        this->mGsCorrectionLog
                            ->Between(it_epoch->second,
                                      this->mGsCorrectionLog->LatestEpoch())
                            .Apply(T_new_d);
                }
                const SE3 T_new = T_new_d.template cast<Scalar>();

                auto it_fp = frame_poses.find(t_ns);
                if (it_fp != frame_poses.end()) {
                    const SE3 T_old = it_fp->second.getPose();
                    const SE3 diff = T_new.inverse() * T_old;
                    const Scalar trans_err = diff.translation().norm();
                    const Scalar rot_err = diff.so3().log().norm();
                    poseUpdateMaxTransErr =
                        std::max(poseUpdateMaxTransErr, trans_err);
                    poseUpdateMaxRotErr =
                        std::max(poseUpdateMaxRotErr, rot_err);
                    if (trans_err > kRelinThresholdTrans ||
                        rot_err > kRelinThresholdRot) {
                        // Large correction — skip to preserve FEJ
                        // consistency. The entry stays so the mapper
                        // can send a refined (smaller) update next time.
                        poseUpdateRejected++;
                        // it = mpPosesToUpdate.erase(it);
                        // continue;
                    } else {
                        // Small correction — apply and preserve the
                        // linearised flag to avoid tripping the
                        // isLinearized() assertion in computeDelta.
                        const bool was_lin = it_fp->second.isLinearized();
                        it_fp->second =
                            PoseStateWithLin<Scalar>(t_ns, T_new, was_lin);
                        poseUpdateApplied++;
                        // it = mpPosesToUpdate.erase(it);
                        // continue;
                    }
                }
                mpPosesToUpdateEpoch.erase(t_ns);
                it = mpPosesToUpdate.erase(it);
                // ++it;
            }
        }
    }
    const double poseUpdateSeconds = tPoseUpdate.elapsed();
    mpLogger->AddVioPoseUpdate(
        opt_flow_meas->t_ns, poseUpdatePending, poseUpdateApplied,
        poseUpdateRejected, double(poseUpdateMaxTransErr),
        double(poseUpdateMaxRotErr), double(kRelinThresholdTrans),
        double(kRelinThresholdRot));
    // mpLogger->PrintVioPoseUpdate();

    if (meas.get()) {
        BASALT_ASSERT(frame_states[last_state_t_ns].getState().t_ns ==
                      meas->get_start_t_ns());
        BASALT_ASSERT(opt_flow_meas->t_ns ==
                      meas->get_dt_ns() + meas->get_start_t_ns());
        BASALT_ASSERT(meas->get_dt_ns() > 0);

        PoseVelBiasState<Scalar> next_state =
            frame_states.at(last_state_t_ns).getState();

        meas->predictState(frame_states.at(last_state_t_ns).getState(), g,
                           next_state);

        last_state_t_ns = opt_flow_meas->t_ns;
        next_state.t_ns = opt_flow_meas->t_ns;

        frame_states[last_state_t_ns] =
            PoseVelBiasStateWithLin<Scalar>(next_state);

        imu_meas[meas->get_start_t_ns()] = *meas;
    }

    // save results
    prev_opt_flow_res[opt_flow_meas->t_ns] = opt_flow_meas;

    // Make new residual for existing keypoints
    Timer tAssoc;
    int connected0 = 0;
    std::map<int64_t, int> num_points_connected;
    std::unordered_set<int> unconnected_obs0;
    for (size_t i = 0; i < opt_flow_meas->observations.size(); i++) {
        TimeCamId tcid_target(opt_flow_meas->t_ns, i);

        for (const auto& kv_obs : opt_flow_meas->observations[i]) {
            int kpt_id = kv_obs.first;

            if (lmdb.landmarkExists(kpt_id)) {
                const TimeCamId& tcid_host =
                    lmdb.getLandmark(kpt_id).host_kf_id;

                KeypointObservation<Scalar> kobs;
                kobs.kpt_id = kpt_id;
                kobs.pos = kv_obs.second.translation().cast<Scalar>();

                lmdb.addObservation(tcid_target, kobs);
                // obs[tcid_host][tcid_target].push_back(kobs);

                if (num_points_connected.count(tcid_host.frame_id) == 0) {
                    num_points_connected[tcid_host.frame_id] = 0;
                }
                num_points_connected[tcid_host.frame_id]++;

                if (i == 0) connected0++;
            } else {
                if (i == 0) {
                    unconnected_obs0.emplace(kpt_id);
                }
            }
        }
    }

    if (Scalar(connected0) / (connected0 + unconnected_obs0.size()) <
            Scalar(config.vio_new_kf_keypoints_thresh) &&
        frames_after_kf > config.vio_min_frames_after_kf)
        take_kf = true;

    const double associationSeconds = tAssoc.elapsed();
    const size_t obs0 = opt_flow_meas->observations.empty()
                            ? 0
                            : opt_flow_meas->observations[0].size();
    mpLogger->AddVioAssoc(
        opt_flow_meas->t_ns, int(obs0), connected0,
        int(unconnected_obs0.size()), config.vio_new_kf_keypoints_thresh,
        frames_after_kf, int(lmdb.numLandmarks()), int(kf_ids.size()),
        int(frame_states.size()), int(frame_poses.size()), take_kf);
    // mpLogger->PrintVioAssoc();

    {
        std::vector<Eigen::aligned_vector<Eigen::Vector4d>> proj(
            opt_flow_meas->observations.size());
        computeProjections(proj, last_state_t_ns);

        std::vector<double> residualsPx;
        for (size_t cam = 0; cam < proj.size(); cam++) {
            for (const Eigen::Vector4d& p : proj[cam]) {
                const KeypointId kptId = KeypointId(p[3]);
                const auto it = opt_flow_meas->observations[cam].find(kptId);
                if (it == opt_flow_meas->observations[cam].end()) continue;
                residualsPx.push_back(
                    (p.template head<2>() -
                     it->second.translation().template cast<double>())
                        .norm());
            }
        }

        if (!residualsPx.empty()) {
            std::sort(residualsPx.begin(), residualsPx.end());
            const double sum =
                std::accumulate(residualsPx.begin(), residualsPx.end(), 0.0);
            const double mean = sum / double(residualsPx.size());
            const double median = residualsPx[residualsPx.size() / 2];
            const size_t p95Idx = size_t(0.95 * double(residualsPx.size() - 1));
            mpLogger->AddVioReprojectionPreOpt(
                opt_flow_meas->t_ns, int(residualsPx.size()), mean, median,
                residualsPx[p95Idx], residualsPx.back());
        } else {
            mpLogger->AddVioReprojectionPreOpt(opt_flow_meas->t_ns, 0, 0.0, 0.0,
                                               0.0, 0.0);
        }
        // mpLogger->PrintVioReprojectionPreOpt();
    }

    Timer tTriang;
    const bool tookKf = take_kf;
    if (take_kf) {
        // Triangulate new points from one of the observations (with sufficient
        // baseline) and make keyframe for camera 0
        take_kf = false;
        mpIsCurrentFrameKF = true;
        frames_after_kf = 0;
        kf_ids.emplace(last_state_t_ns);
        if (mpDm) mpDm->AddKeyframe(last_state_t_ns);

        TimeCamId tcidl(opt_flow_meas->t_ns, 0);

        int num_points_added = 0;
        // Triangulation rejection tally, mirroring the local mapper's
        // [setup_opt] breakdown so the two front ends can be compared.
        int numNoPriorObs = 0, numUnprojectFail = 0, numShortBaseline = 0;
        int numNotFinite = 0, numBehind = 0, numTooClose = 0,
            numLowParallax = 0;
        Scalar minBaseline = std::numeric_limits<Scalar>::max();
        Scalar maxBaseline = 0;
        Scalar minDepth = std::numeric_limits<Scalar>::max();
        Scalar maxDepth = 0;
        Scalar sumDepth = 0;
        Scalar minCosParallax = std::numeric_limits<Scalar>::max();
        Scalar maxCosParallax = -std::numeric_limits<Scalar>::max();
        Scalar sumCosParallax = 0;
        int numParallaxSamples = 0;
        const Scalar minParallaxCos =
            config.vio_min_triangulation_parallax_deg > 0
                ? std::cos(Scalar(config.vio_min_triangulation_parallax_deg) *
                           Scalar(M_PI / 180.0))
                : Scalar(1);
        for (int lm_id : unconnected_obs0) {
            // Find all observations
            std::map<TimeCamId, KeypointObservation<Scalar>> kp_obs;

            for (const auto& kv : prev_opt_flow_res) {
                for (size_t k = 0; k < kv.second->observations.size(); k++) {
                    auto it = kv.second->observations[k].find(lm_id);
                    if (it != kv.second->observations[k].end()) {
                        TimeCamId tcido(kv.first, k);

                        KeypointObservation<Scalar> kobs;
                        kobs.kpt_id = lm_id;
                        kobs.pos =
                            it->second.translation().template cast<Scalar>();

                        // obs[tcidl][tcido].push_back(kobs);
                        kp_obs[tcido] = kobs;
                    }
                }
            }

            // triangulate
            bool valid_kp = false;
            if (kp_obs.empty()) numNoPriorObs++;
            const Scalar min_triang_distance2 =
                Scalar(config.vio_min_triangulation_dist *
                       config.vio_min_triangulation_dist);
            for (const auto& kv_obs : kp_obs) {
                if (valid_kp) break;
                TimeCamId tcido = kv_obs.first;

                const Vec2 p0 = opt_flow_meas->observations.at(0)
                                    .at(lm_id)
                                    .translation()
                                    .cast<Scalar>();
                const Vec2 p1 = prev_opt_flow_res[tcido.frame_id]
                                    ->observations[tcido.cam_id]
                                    .at(lm_id)
                                    .translation()
                                    .template cast<Scalar>();

                Vec4 p0_3d, p1_3d;
                bool valid1 = this->calib.intrinsics[0].unproject(p0, p0_3d);
                bool valid2 =
                    this->calib.intrinsics[tcido.cam_id].unproject(p1, p1_3d);
                if (!valid1 || !valid2) {
                    numUnprojectFail++;
                    continue;
                }

                SE3 T_i0_i1 =
                    getPoseStateWithLin(tcidl.frame_id).getPose().inverse() *
                    getPoseStateWithLin(tcido.frame_id).getPose();
                SE3 T_0_1 = this->calib.T_i_c[0].inverse() * T_i0_i1 *
                            this->calib.T_i_c[tcido.cam_id];

                const Scalar baseline = T_0_1.translation().norm();
                if (baseline < minBaseline) minBaseline = baseline;
                if (baseline > maxBaseline) maxBaseline = baseline;

                const Vec3 ray0 = p0_3d.template head<3>().normalized();
                const Vec3 ray1 =
                    (T_0_1.so3() * p1_3d.template head<3>()).normalized();
                const Scalar cosParallax = ray0.dot(ray1);
                numParallaxSamples++;
                if (cosParallax < minCosParallax) minCosParallax = cosParallax;
                if (cosParallax > maxCosParallax) maxCosParallax = cosParallax;
                sumCosParallax += cosParallax;

                if (T_0_1.translation().squaredNorm() < min_triang_distance2) {
                    numShortBaseline++;
                    continue;
                }

                if (minParallaxCos < Scalar(1) &&
                    cosParallax > minParallaxCos) {
                    numLowParallax++;
                    continue;
                }

                Vec4 p0_triangulated = triangulate(
                    p0_3d.template head<3>(), p1_3d.template head<3>(), T_0_1);

                if (!p0_triangulated.array().isFinite().all()) {
                    numNotFinite++;
                } else if (p0_triangulated[3] <= 0) {
                    numBehind++;
                } else if (p0_triangulated[3] >= 3.0) {
                    numTooClose++;
                }

                if (p0_triangulated.array().isFinite().all() &&
                    p0_triangulated[3] > 0 && p0_triangulated[3] < 3.0) {
                    const Scalar depth = Scalar(1) / p0_triangulated[3];
                    if (depth < minDepth) minDepth = depth;
                    if (depth > maxDepth) maxDepth = depth;
                    sumDepth += depth;
                    Keypoint<Scalar> kpt_pos;
                    kpt_pos.host_kf_id = tcidl;
                    kpt_pos.direction =
                        StereographicParam<Scalar>::project(p0_triangulated);
                    kpt_pos.inv_dist = p0_triangulated[3];
                    lmdb.addLandmark(lm_id, kpt_pos);

                    num_points_added++;
                    valid_kp = true;
                }
            }

            if (valid_kp) {
                for (const auto& kv_obs : kp_obs) {
                    lmdb.addObservation(kv_obs.first, kv_obs.second);
                }
            }
        }

        num_points_kf[opt_flow_meas->t_ns] = num_points_added;

        mpLogger->AddVioTriang(
            opt_flow_meas->t_ns, int(unconnected_obs0.size()), num_points_added,
            numNoPriorObs, numUnprojectFail, numShortBaseline, numNotFinite,
            numBehind, numTooClose, config.vio_min_triangulation_dist,
            double(maxBaseline > 0 ? minBaseline : Scalar(0)),
            double(maxBaseline),
            double(num_points_added ? minDepth : Scalar(0)), double(sumDepth),
            double(maxDepth), double(minCosParallax), double(sumCosParallax),
            double(maxCosParallax), numParallaxSamples, numLowParallax);
        // mpLogger->PrintVioTriang();
    } else {
        frames_after_kf++;
    }
    const double triangulationSeconds = tookKf ? tTriang.elapsed() : 0.0;

    Timer tLostScan;
    std::unordered_set<KeypointId> lost_landmaks;
    if (config.vio_marg_lost_landmarks) {
        for (const auto& kv : lmdb.getLandmarks()) {
            bool connected = false;
            for (size_t i = 0; i < opt_flow_meas->observations.size(); i++) {
                if (opt_flow_meas->observations[i].count(kv.first) > 0)
                    connected = true;
            }
            if (!connected) {
                lost_landmaks.emplace(kv.first);
            }
        }
    }
    const double lostScanSeconds = tLostScan.elapsed();

    mpLogger->AddVioLandmarks(opt_flow_meas->t_ns, int(lost_landmaks.size()),
                              int(lmdb.numLandmarks()),
                              config.vio_marg_lost_landmarks);
    // mpLogger->PrintVioLandmarks();

    Timer tOptMarg;
    optimize_and_marg(num_points_connected, lost_landmaks);
    const double optMargSeconds = tOptMarg.elapsed();
    PoseVelBiasStateWithLin p = frame_states.at(last_state_t_ns);

    {
        const auto& st = p.getState();
        mpLogger->AddVioState(
            p.getT_ns(), st.T_w_i.translation().template cast<double>(),
            st.T_w_i.unit_quaternion().coeffs().template cast<double>(),
            st.vel_w_i.template cast<double>(),
            st.bias_gyro.template cast<double>(),
            st.bias_accel.template cast<double>());
        // mpLogger->PrintVioState();
    }

    Timer tPublish;
    if (this->out_state_queue) {
        typename PoseVelBiasState<double>::Ptr data(
            new PoseVelBiasState<double>(p.getState().template cast<double>()));

        this->out_state_queue->try_push(data);
    }

    if (this->out_vis_queue && !frame_states.empty()) {
        VioVisualizationData::Ptr data(new VioVisualizationData);

        data->t_ns = last_state_t_ns;

        // BASALT_ASSERT(frame_states.empty());

        for (const auto& kv : frame_states) {
            data->states.emplace_back(
                kv.second.getState().T_w_i.template cast<double>());
        }

        for (const auto& kv : frame_poses) {
            data->frames.emplace_back(
                kv.second.getPose().template cast<double>());
        }

        get_current_points(data->points, data->point_ids);

        data->projections.resize(opt_flow_meas->observations.size());
        computeProjections(data->projections, last_state_t_ns);

        data->opt_flow_res = prev_opt_flow_res[last_state_t_ns];

        this->out_vis_queue->try_push(data);
    }
    const double publishSeconds = tPublish.elapsed();

    this->last_processed_t_ns = last_state_t_ns;

    const double measureSeconds = t_total.elapsed();
    mpLogger->SolverScratch().add("measure", measureSeconds).format("ms");
    mpLogger->AddVioTiming(
        opt_flow_meas->t_ns, poseUpdateSeconds, associationSeconds,
        triangulationSeconds, lostScanSeconds, optMargSeconds, publishSeconds,
        measureSeconds, imuDrainSeconds, processFrameSoFar,
        int(this->vision_data_queue.size()), int(this->imu_data_queue.size()));
    // mpLogger->PrintVioTiming();

    typename PoseVelBiasState<Scalar>::Ptr d(new PoseVelBiasState<Scalar>(
        p.getT_ns(), p.getState().T_w_i, p.getState().vel_w_i, mpBg, mpBa));

    return d;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::logMargNullspace() {
    nullspace_marg_data.order = marg_data.order;
    const Eigen::VectorXd margNs = checkMargNullspace();
    const Eigen::VectorXd margEv = checkMargEigenvalues();
    mpLogger->SolverScratch().add("marg_ns", margNs);
    mpLogger->SolverScratch().add("marg_ev", margEv);

    const double evMin = margEv.size() ? margEv.minCoeff() : 0.0;
    const double evMax = margEv.size() ? margEv.maxCoeff() : 0.0;
    const int evNegative = int((margEv.array() < 0).count());
    const double evCondition = evMin != 0.0 ? evMax / evMin : 0.0;
    mpLogger->AddVioMargNullspace(last_state_t_ns, margNs, evMin, evMax,
                                  evNegative, evCondition);
    // mpLogger->PrintVioMargNullspace();
}

template <class Scalar_>
Eigen::VectorXd SqrtKeypointVioEstimator<Scalar_>::checkMargNullspace() const {
    return checkNullspace(nullspace_marg_data, frame_states, frame_poses,
                          config.vio_debug);
}

template <class Scalar_>
Eigen::VectorXd SqrtKeypointVioEstimator<Scalar_>::checkMargEigenvalues()
    const {
    return checkEigenvalues(nullspace_marg_data, false);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::marginalize(
    const std::map<int64_t, int>& num_points_connected,
    const std::unordered_set<KeypointId>& lost_landmaks) {
    if (!opt_started) return;

    Timer t_total;
    mpGsSwapEvent = 0;
    mpGsVisualImuFull = 0;
    mpGsVisualImuRot = 0;
    mpGsVisualImuScaleCol = 0;

    if (frame_poses.size() > max_kfs || frame_states.size() >= max_states) {
        // Marginalize

        const int states_to_remove = frame_states.size() - max_states + 1;

        auto it = frame_states.cbegin();
        for (int i = 0; i < states_to_remove; i++) it++;
        int64_t last_state_to_marg = it->first;

        AbsOrderMap aom;

        if (mpGsActive) {
            aom.abs_order_map[kGravityScaleStateId] =
                std::make_pair(0, GRAVITY_SCALE_SIZE);

            BASALT_ASSERT(
                marg_data.order.abs_order_map.at(kGravityScaleStateId) ==
                aom.abs_order_map.at(kGravityScaleStateId));

            aom.total_size += GRAVITY_SCALE_SIZE;
            aom.items++;
        }

        // remove all frame_poses that are not kfs
        std::set<int64_t> poses_to_marg;
        for (const auto& kv : frame_poses) {
            aom.abs_order_map[kv.first] =
                std::make_pair(aom.total_size, POSE_SIZE);

            if (kf_ids.count(kv.first) == 0) poses_to_marg.emplace(kv.first);

            // Check that we have the same order as marginalization
            BASALT_ASSERT(marg_data.order.abs_order_map.at(kv.first) ==
                          aom.abs_order_map.at(kv.first));

            aom.total_size += POSE_SIZE;
            aom.items++;
        }

        std::set<int64_t> states_to_marg_vel_bias;
        std::set<int64_t> states_to_marg_all;
        for (const auto& kv : frame_states) {
            if (kv.first > last_state_to_marg) break;

            if (kv.first != last_state_to_marg) {
                if (kf_ids.count(kv.first) > 0) {
                    states_to_marg_vel_bias.emplace(kv.first);
                } else {
                    states_to_marg_all.emplace(kv.first);
                }
            }

            aom.abs_order_map[kv.first] =
                std::make_pair(aom.total_size, POSE_VEL_BIAS_SIZE);

            // Check that we have the same order as marginalization
            if (aom.items < marg_data.order.abs_order_map.size())
                BASALT_ASSERT(marg_data.order.abs_order_map.at(kv.first) ==
                              aom.abs_order_map.at(kv.first));

            aom.total_size += POSE_VEL_BIAS_SIZE;
            aom.items++;
        }

        auto kf_ids_all = kf_ids;
        std::set<int64_t> kfs_to_marg;
        while (kf_ids.size() > max_kfs && !states_to_marg_vel_bias.empty()) {
            int64_t id_to_marg = -1;

            // starting from the oldest kf (and skipping the newest 2 kfs), try
            // to find a kf that has less than a small percentage of it's
            // landmarks tracked by the current frame
            if (kf_ids.size() > 2) {
                // Note: size > 2 check is to ensure prev(kf_ids.end(), 2) is
                // valid
                auto end_minus_2 = std::prev(kf_ids.end(), 2);

                for (auto it = kf_ids.begin(); it != end_minus_2; ++it) {
                    if (num_points_connected.count(*it) == 0 ||
                        (num_points_connected.at(*it) /
                             static_cast<float>(num_points_kf.at(*it)) <
                         config.vio_kf_marg_feature_ratio)) {
                        id_to_marg = *it;
                        break;
                    }
                }
            }

            // Note: This score function is taken from DSO, but it seems to
            // mostly marginalize the oldest keyframe. This may be due to the
            // fact that we don't have as long-lived landmarks, which may change
            // if we ever implement "rediscovering" of lost feature tracks by
            // projecting untracked landmarks into the localized frame.
            if (kf_ids.size() > 2 && id_to_marg < 0) {
                // Note: size > 2 check is to ensure prev(kf_ids.end(), 2) is
                // valid
                auto end_minus_2 = std::prev(kf_ids.end(), 2);

                int64_t last_kf = *kf_ids.crbegin();
                Scalar min_score = std::numeric_limits<Scalar>::max();
                int64_t min_score_id = -1;

                for (auto it1 = kf_ids.begin(); it1 != end_minus_2; ++it1) {
                    // small distance to other keyframes --> higher score
                    Scalar denom = 0;
                    for (auto it2 = kf_ids.begin(); it2 != end_minus_2; ++it2) {
                        denom +=
                            1 / ((frame_poses.at(*it1).getPose().translation() -
                                  frame_poses.at(*it2).getPose().translation())
                                     .norm() +
                                 Scalar(1e-5));
                    }

                    // small distance to latest kf --> lower score
                    Scalar score =
                        std::sqrt(
                            (frame_poses.at(*it1).getPose().translation() -
                             frame_states.at(last_kf)
                                 .getState()
                                 .T_w_i.translation())
                                .norm()) *
                        denom;

                    if (score < min_score) {
                        min_score_id = *it1;
                        min_score = score;
                    }
                }

                id_to_marg = min_score_id;
            }

            // if no frame was selected, the logic above is faulty
            BASALT_ASSERT(id_to_marg >= 0);

            kfs_to_marg.emplace(id_to_marg);
            poses_to_marg.emplace(id_to_marg);

            kf_ids.erase(id_to_marg);
        }
        // keyframe selection for marginalisation ends here.

        //    std::cout << "marg order" << std::endl;
        //    aom.print_order();

        //    std::cout << "marg prior order" << std::endl;
        //    marg_order.print_order();

        Timer t_actual_marg;

        size_t asize = aom.total_size;

        bool is_lin_sqrt = isLinearizationSqrt(config.vio_linearization_type);

        MatX Q2Jp_or_H;
        VecX Q2r_or_b;

        double margLinearizeSeconds = 0.0, margHelperSeconds = 0.0,
               margLogSeconds = 0.0;

        MatX Q2Jp_store;
        VecX Q2r_store;

        if (mpGsActive) {
            if (!mpDm->BoundarySet()) {
                // The prior before the first marginalisation of the period is
                // the boundary of the store, plans/delayed_marginalisation.md
                // §2.3.5.
                DelayedMargPrior<Scalar> boundary;
                boundary.mH = marg_data.H;
                boundary.mB = marg_data.b;
                boundary.mOrder = marg_data.order;
                boundary.mAnchor.Set(*this->mGsState);
                mpDm->SetBoundary(boundary);
            }
            LinearizeMargGS(aom, kfs_to_marg, lost_landmaks, last_state_to_marg,
                            Q2Jp_or_H, Q2r_or_b, Q2Jp_store, Q2r_store);
        } else {
            Timer t_linearize;

            typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
            lqr_options.lb_options.huber_parameter = huber_thresh;
            lqr_options.lb_options.obs_std_dev = obs_std_dev;
            lqr_options.linearization_type = config.vio_linearization_type;

            ImuLinData<Scalar> ild = {
                g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};

            for (const auto& kv : imu_meas) {
                int64_t start_t = kv.second.get_start_t_ns();
                int64_t end_t =
                    kv.second.get_start_t_ns() + kv.second.get_dt_ns();

                if (aom.abs_order_map.count(start_t) == 0 ||
                    aom.abs_order_map.count(end_t) == 0)
                    continue;

                ild.imu_meas[kv.first] = &kv.second;
            }

            auto lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
                this, aom, lqr_options, &marg_data, &ild, &kfs_to_marg,
                &lost_landmaks, last_state_to_marg);

            lqr->linearizeProblem();
            lqr->performQR();

            if (is_lin_sqrt && marg_data.is_sqrt) {
                lqr->get_dense_Q2Jp_Q2r(Q2Jp_or_H, Q2r_or_b);
            } else {
                lqr->get_dense_H_b(Q2Jp_or_H, Q2r_or_b);
            }

            margLinearizeSeconds = t_linearize.elapsed();
            mpLogger->SolverScratch()
                .add("marg_linearize", margLinearizeSeconds)
                .format("ms");
        }

        //    KeypointVioEstimator::linearizeAbsIMU(
        //        aom, accum.getH(), accum.getB(), imu_error, bg_error,
        //        ba_error, frame_states, imu_meas, gyro_bias_weight,
        //        accel_bias_weight, g);
        //    linearizeMargPrior(marg_order, marg_sqrt_H, marg_sqrt_b, aom,
        //    accum.getH(),
        //                       accum.getB(), marg_prior_error);

        // Save marginalization prior
        if (this->out_marg_queue && !kfs_to_marg.empty()) {
            // int64_t kf_id = *kfs_to_marg.begin();

            {
                MargData::Ptr m(new MargData);
                m->aom = aom;

                if (is_lin_sqrt && marg_data.is_sqrt) {
                    m->abs_H = (Q2Jp_or_H.transpose() * Q2Jp_or_H)
                                   .template cast<double>();
                    m->abs_b = (Q2Jp_or_H.transpose() * Q2r_or_b)
                                   .template cast<double>();
                } else {
                    m->abs_H = Q2Jp_or_H.template cast<double>();

                    m->abs_b = Q2r_or_b.template cast<double>();
                }

                if (mpGsActive) StripGsFromMargData(*m);
                if (this->mGsCorrectionLog) {
                    m->mGsEpoch = this->mGsCorrectionLog->LatestEpoch();
                }

                assign_cast_map_values(m->frame_poses, frame_poses);
                assign_cast_map_values(m->frame_states, frame_states);
                m->kfs_all = kf_ids_all;
                m->kfs_to_marg = kfs_to_marg;
                m->use_imu = true;

                for (int64_t t : m->kfs_all) {
                    m->opt_flow_res.emplace_back(prev_opt_flow_res.at(t));
                }

                this->out_marg_queue->push(m);
            }
        }

        std::set<int> idx_to_keep, idx_to_marg;
        for (const auto& kv : aom.abs_order_map) {
            if (kv.first == kGravityScaleStateId) {
                for (int i = 0; i < GRAVITY_SCALE_SIZE; i++)
                    idx_to_keep.emplace(kv.second.first + i);
                continue;
            }

            if (kv.second.second == POSE_SIZE) {
                int start_idx = kv.second.first;
                if (poses_to_marg.count(kv.first) == 0) {
                    for (size_t i = 0; i < POSE_SIZE; i++)
                        idx_to_keep.emplace(start_idx + i);
                } else {
                    for (size_t i = 0; i < POSE_SIZE; i++)
                        idx_to_marg.emplace(start_idx + i);
                }
            } else {
                BASALT_ASSERT(kv.second.second == POSE_VEL_BIAS_SIZE);
                // state
                int start_idx = kv.second.first;
                if (states_to_marg_all.count(kv.first) > 0) {
                    for (size_t i = 0; i < POSE_VEL_BIAS_SIZE; i++)
                        idx_to_marg.emplace(start_idx + i);
                } else if (states_to_marg_vel_bias.count(kv.first) > 0) {
                    for (size_t i = 0; i < POSE_SIZE; i++)
                        idx_to_keep.emplace(start_idx + i);
                    for (size_t i = POSE_SIZE; i < POSE_VEL_BIAS_SIZE; i++)
                        idx_to_marg.emplace(start_idx + i);
                } else {
                    BASALT_ASSERT(kv.first == last_state_to_marg);
                    for (size_t i = 0; i < POSE_VEL_BIAS_SIZE; i++)
                        idx_to_keep.emplace(start_idx + i);
                }
            }
        }

        mpLogger->AddVioMarg(
            last_state_t_ns, states_to_remove, int(poses_to_marg.size()),
            int(states_to_marg_all.size()), int(states_to_marg_vel_bias.size()),
            int(kfs_to_marg.size()), int(kf_ids.size()),
            int(idx_to_keep.size()), int(idx_to_marg.size()), int(asize),
            int(frame_poses.size()), int(frame_states.size()),
            last_state_to_marg);
        // mpLogger->PrintVioMarg();

        if ((config.vio_debug || config.vio_extended_logging) && !mpGsActive) {
            MatX Q2Jp_or_H_nullspace;
            VecX Q2r_or_b_nullspace;

            typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
            lqr_options.lb_options.huber_parameter = huber_thresh;
            lqr_options.lb_options.obs_std_dev = obs_std_dev;
            lqr_options.linearization_type = config.vio_linearization_type;

            nullspace_marg_data.order = marg_data.order;

            ImuLinData<Scalar> ild = {
                g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};

            for (const auto& kv : imu_meas) {
                int64_t start_t = kv.second.get_start_t_ns();
                int64_t end_t =
                    kv.second.get_start_t_ns() + kv.second.get_dt_ns();

                if (aom.abs_order_map.count(start_t) == 0 ||
                    aom.abs_order_map.count(end_t) == 0)
                    continue;

                ild.imu_meas[kv.first] = &kv.second;
            }

            auto lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
                this, aom, lqr_options, &nullspace_marg_data, &ild,
                &kfs_to_marg, &lost_landmaks, last_state_to_marg);

            lqr->linearizeProblem();
            lqr->performQR();

            if (is_lin_sqrt && marg_data.is_sqrt) {
                lqr->get_dense_Q2Jp_Q2r(Q2Jp_or_H_nullspace,
                                        Q2r_or_b_nullspace);
            } else {
                lqr->get_dense_H_b(Q2Jp_or_H_nullspace, Q2r_or_b_nullspace);
            }

            MatX nullspace_sqrt_H_new;
            VecX nullspace_sqrt_b_new;

            if (is_lin_sqrt && marg_data.is_sqrt) {
                MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
                    Q2Jp_or_H_nullspace, Q2r_or_b_nullspace, idx_to_keep,
                    idx_to_marg, nullspace_sqrt_H_new, nullspace_sqrt_b_new);
            } else if (marg_data.is_sqrt) {
                MargHelper<Scalar>::marginalizeHelperSqToSqrt(
                    Q2Jp_or_H_nullspace, Q2r_or_b_nullspace, idx_to_keep,
                    idx_to_marg, nullspace_sqrt_H_new, nullspace_sqrt_b_new);
            } else {
                MargHelper<Scalar>::marginalizeHelperSqToSq(
                    Q2Jp_or_H_nullspace, Q2r_or_b_nullspace, idx_to_keep,
                    idx_to_marg, nullspace_sqrt_H_new, nullspace_sqrt_b_new);
            }

            nullspace_marg_data.H = nullspace_sqrt_H_new;
            nullspace_marg_data.b = nullspace_sqrt_b_new;
        }

        MatX marg_H_new;
        VecX marg_b_new;

        {
            Timer t;
            if (mpGsActive) {
                MarginalizeGS(Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg);
            } else if (is_lin_sqrt && marg_data.is_sqrt) {
                MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
                    Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_H_new,
                    marg_b_new);
            } else if (marg_data.is_sqrt) {
                MargHelper<Scalar>::marginalizeHelperSqToSqrt(
                    Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_H_new,
                    marg_b_new);
            } else {
                MargHelper<Scalar>::marginalizeHelperSqToSq(
                    Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_H_new,
                    marg_b_new);
            }

            margHelperSeconds = t.elapsed();
            mpLogger->SolverScratch()
                .add("marg_helper", margHelperSeconds)
                .format("ms");
        }

        DelayedMargStep<Scalar> dmStep;
        if (mpGsActive) {
            Timer tDm;
            dmStep.mIndex = mpDm->NextIndex();
            dmStep.mEpoch = this->mGsCorrectionLog->LatestEpoch();
            dmStep.mOrder = aom;
            dmStep.mIdxKeep = idx_to_keep;
            dmStep.mIdxMarg = idx_to_marg;

            // Visual rows in delta independent form, compressed to at most one
            // row per column, plans/delayed_marginalisation.md §3.2 Change 6.
            VecX delta = VecX::Zero(aom.total_size);
            for (const auto& [id, idx] : aom.abs_order_map) {
                if (id == kGravityScaleStateId) continue;
                if (idx.second == POSE_SIZE) {
                    delta.template segment<POSE_SIZE>(idx.first) =
                        frame_poses.at(id).getDelta();
                } else {
                    delta.template segment<POSE_VEL_BIAS_SIZE>(idx.first) =
                        frame_states.at(id).getDelta();
                }
            }
            Q2r_store -= Q2Jp_store * delta;
            const int n = int(aom.total_size);
            if (Q2Jp_store.rows() > n) {
                MatX A(Q2Jp_store.rows(), n + 1);
                A << Q2Jp_store, Q2r_store;
                const Eigen::HouseholderQR<MatX> qr(A);
                const MatX R = qr.matrixQR()
                                   .topRows(n)
                                   .template triangularView<Eigen::Upper>();
                dmStep.mVisJ = R.leftCols(n);
                dmStep.mVisR = R.col(n);
            } else {
                dmStep.mVisJ = Q2Jp_store;
                dmStep.mVisR = Q2r_store;
            }

            for (const auto& kv : imu_meas) {
                const int64_t start_t = kv.second.get_start_t_ns();
                const int64_t end_t = start_t + kv.second.get_dt_ns();
                if (aom.abs_order_map.count(start_t) == 0 ||
                    aom.abs_order_map.count(end_t) == 0)
                    continue;
                dmStep.mvImu.push_back(kv.second);
            }

            for (const int64_t id : states_to_marg_all) {
                mpDm->RecordDeparture(id, frame_states.at(id), dmStep.mIndex);
            }
            for (const int64_t id : states_to_marg_vel_bias) {
                mpDm->RecordDeparture(id, frame_states.at(id), dmStep.mIndex);
            }
            for (const int64_t id : poses_to_marg) {
                if (frame_poses.count(id) > 0) {
                    mpDm->RecordPoseDeparture(id, frame_poses.at(id),
                                              dmStep.mIndex);
                }
            }
            mpDmStoreSeconds = tDm.elapsed();
        }

        {
            BASALT_ASSERT(frame_states.at(last_state_to_marg).isLinearized() ==
                          false);
            frame_states.at(last_state_to_marg).setLinTrue();
        }

        for (const int64_t id : states_to_marg_all) {
            frame_states.erase(id);
            imu_meas.erase(id);
            prev_opt_flow_res.erase(id);
        }

        for (const int64_t id : states_to_marg_vel_bias) {
            const PoseVelBiasStateWithLin<Scalar>& state = frame_states.at(id);
            PoseStateWithLin<Scalar> pose(state);

            frame_poses[id] = pose;
            frame_states.erase(id);
            imu_meas.erase(id);
        }

        for (const int64_t id : poses_to_marg) {
            frame_poses.erase(id);
            prev_opt_flow_res.erase(id);
        }

        lmdb.removeKeyframes(kfs_to_marg, poses_to_marg, states_to_marg_all);

        if (config.vio_marg_lost_landmarks) {
            for (const auto& lm_id : lost_landmaks) lmdb.removeLandmark(lm_id);
        }

        AbsOrderMap marg_order_new;

        if (mpGsActive) {
            marg_order_new.abs_order_map[kGravityScaleStateId] =
                std::make_pair(0, GRAVITY_SCALE_SIZE);
            marg_order_new.total_size += GRAVITY_SCALE_SIZE;
            marg_order_new.items++;
        }

        for (const auto& kv : frame_poses) {
            marg_order_new.abs_order_map[kv.first] =
                std::make_pair(marg_order_new.total_size, POSE_SIZE);

            marg_order_new.total_size += POSE_SIZE;
            marg_order_new.items++;
        }

        {
            marg_order_new.abs_order_map[last_state_to_marg] =
                std::make_pair(marg_order_new.total_size, POSE_VEL_BIAS_SIZE);
            marg_order_new.total_size += POSE_VEL_BIAS_SIZE;
            marg_order_new.items++;
        }

        if (!mpGsActive) {
            marg_data.H = marg_H_new;
            marg_data.b = marg_b_new;
        }
        marg_data.order = marg_order_new;

        BASALT_ASSERT(size_t(marg_data.H.cols()) == marg_data.order.total_size);

        // Quadratic prior and "delta" of the current state to the original
        // linearization point give cost function
        //
        //    P(x) = 0.5 || J*(delta+x) + r ||^2.
        //
        // For marginalization this has been linearized at x=0 to give
        // linearization
        //
        //    P(x) = 0.5 || J*x + (J*delta + r) ||^2,
        //
        // with Jacobian J and residual J*delta + r.
        //
        // After marginalization, we recover the original form of the
        // prior. We are left with linearization (in sqrt form)
        //
        //    Pnew(x) = 0.5 || Jnew*x + res ||^2.
        //
        // To recover the original form with delta-independent r, we set
        //
        //    Pnew(x) = 0.5 || Jnew*(delta+x) + (res - Jnew*delta) ||^2,
        //
        // and thus rnew = (res - Jnew*delta).

        if (mpGsActive) {
            RecentrePriorsGS();
            dmStep.mOrderNew = marg_order_new;
            UpdateDelayedMarg(std::move(dmStep));
        } else {
            VecX delta;
            computeDelta(marg_data.order, delta);
            marg_data.b -= marg_data.H * delta;
        }

        if ((config.vio_debug || config.vio_extended_logging) && !mpGsActive) {
            VecX delta;
            computeDelta(marg_data.order, delta);
            nullspace_marg_data.b -= nullspace_marg_data.H * delta;
        }

        const double margSeconds = t_actual_marg.elapsed();
        mpLogger->SolverScratch().add("marg", margSeconds).format("ms");

        if ((config.vio_debug || config.vio_extended_logging) && !mpGsActive) {
            Timer t;
            logMargNullspace();
            margLogSeconds = t.elapsed();
            mpLogger->SolverScratch()
                .add("marg_log", margLogSeconds)
                .format("ms");
        }

        //    std::cout << "new marg prior order" << std::endl;
        //    marg_order.print_order();

        mpLogger->AddVioMargTiming(last_state_t_ns, 0.0, margLinearizeSeconds,
                                   margHelperSeconds, margSeconds,
                                   margLogSeconds, t_total.elapsed());
        // mpLogger->PrintVioMargTiming();
    }

    if (mpGsActive) {
        LogGravityScale(mpGsSwapEvent);
        if (mpGsStartNs >= 0 &&
            last_state_t_ns - mpGsStartNs >=
                int64_t(config.vio_gs_duration_s * 1e9)) {
            FoldGravityScale();
        }
    }

    mpLogger->SolverScratch()
        .add("marginalize", t_total.elapsed())
        .format("ms");
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::optimize() {
    if (opt_started || frame_states.size() > 4) {
        opt_started = true;

        // harcoded configs
        // bool scale_Jp = config.vio_scale_jacobian && is_qr_solver();
        // bool scale_Jl = config.vio_scale_jacobian && is_qr_solver();

        // timing
        Timer timer_total;
        Timer timer_iteration;

        // construct order of states in linear system --> sort by ascending
        // timestamp
        AbsOrderMap aom;

        if (mpGsActive) {
            aom.abs_order_map[kGravityScaleStateId] =
                std::make_pair(0, GRAVITY_SCALE_SIZE);

            BASALT_ASSERT(
                marg_data.order.abs_order_map.at(kGravityScaleStateId) ==
                aom.abs_order_map.at(kGravityScaleStateId));

            aom.total_size += GRAVITY_SCALE_SIZE;
            aom.items++;
        }

        for (const auto& kv : frame_poses) {
            aom.abs_order_map[kv.first] =
                std::make_pair(aom.total_size, POSE_SIZE);

            // Check that we have the same order as marginalization
            BASALT_ASSERT(marg_data.order.abs_order_map.at(kv.first) ==
                          aom.abs_order_map.at(kv.first));

            aom.total_size += POSE_SIZE;
            aom.items++;
        }

        for (const auto& kv : frame_states) {
            aom.abs_order_map[kv.first] =
                std::make_pair(aom.total_size, POSE_VEL_BIAS_SIZE);

            // Check that we have the same order as marginalization
            if (aom.items < marg_data.order.abs_order_map.size())
                BASALT_ASSERT(marg_data.order.abs_order_map.at(kv.first) ==
                              aom.abs_order_map.at(kv.first));

            aom.total_size += POSE_VEL_BIAS_SIZE;
            aom.items++;
        }

        // TODO: Check why we get better accuracy with old SC loop. Possible
        // culprits:
        // - different initial lambda (based on previous iteration)
        // - no landmark damping
        // - outlier removal after 4 iterations?
        lambda = Scalar(config.vio_lm_lambda_initial);

        // record stats
        const int numCams = int(this->frame_poses.size());
        const int numLms = int(this->lmdb.numLandmarks());
        const int numObs = int(this->lmdb.numObservations());
        mpLogger->SolverScratch()
            .add("num_cams", double(numCams))
            .format("count");
        mpLogger->SolverScratch()
            .add("num_lms", double(numLms))
            .format("count");
        mpLogger->SolverScratch()
            .add("num_obs", double(numObs))
            .format("count");

        // setup landmark blocks
        typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
        lqr_options.lb_options.huber_parameter = huber_thresh;
        lqr_options.lb_options.obs_std_dev = obs_std_dev;
        lqr_options.linearization_type = config.vio_linearization_type;

        std::unique_ptr<LinearizationBase<Scalar, POSE_SIZE>> lqr;

        ImuLinData<Scalar> ild = {
            g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};
        for (const auto& kv : imu_meas) {
            ild.imu_meas[kv.first] = &kv.second;
        }

        double allocateLmbSeconds = 0.0;
        {
            Timer t;
            if (mpGsActive) {
                lqr = std::make_unique<LinearizationGSAbsQR<Scalar, POSE_SIZE>>(
                    this, aom, lqr_options, &*this->mGsState, false, &marg_data,
                    &ild);
            } else {
                lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
                    this, aom, lqr_options, &marg_data, &ild);
            }
            allocateLmbSeconds = t.reset();
            mpLogger->SolverScratch()
                .add("allocateLMB", allocateLmbSeconds)
                .format("ms");
            lqr->log_problem_stats(mpLogger->SolverScratch());
        }

        bool terminated = false;
        bool converged = false;

        // The block is moved only by the coarse optimisation until the store
        // first holds the delay, plans/delayed_marginalisation.md §2.3.9.
        const bool gsHoldBlock = mpGsActive && mpDmCoarse;

        // Random walk residual on the gravity and scale step of this frame,
        // plans/gravity_scale_optimisation_2.md §3.4. gsJump is the block
        // increment accumulated over the accepted iterations.
        Vec3 gsJump = Vec3::Zero();
        Vec3 gsJumpW2 = Vec3::Zero();
        if (mpGsActive && !imu_meas.empty()) {
            const Scalar dtFrame =
                Scalar(imu_meas.rbegin()->second.get_dt_ns()) * Scalar(1e-9);
            if (config.vio_gs_jump_tilt_rw_deg > 0) {
                const Scalar tiltStd =
                    Scalar(config.vio_gs_jump_tilt_rw_deg * M_PI / 180.0) *
                    std::sqrt(dtFrame);
                gsJumpW2.template head<2>().setConstant(
                    Scalar(1) / (tiltStd * tiltStd));
            }
            if (config.vio_gs_jump_sigma_rw > 0) {
                const Scalar sigmaStd =
                    Scalar(config.vio_gs_jump_sigma_rw) * std::sqrt(dtFrame);
                gsJumpW2[2] = Scalar(1) / (sigmaStd * sigmaStd);
            }
        }
        const bool applyGsJump =
            !gsHoldBlock && (gsJumpW2.array() > Scalar(0)).any();


        int it = 0;
        int it_rejected = 0;
        for (; it <= config.vio_max_iterations && !terminated;) {
            if (it > 0) {
                timer_iteration.reset();
            }

            Scalar error_total = 0;
            VecX Jp_column_norm2;
            // Hoisted out of the block below so AddVioLinearise can report it.
            bool numerically_valid = false;

            double linearizeProblemSeconds = 0.0, performQrSeconds = 0.0;
            {
                // TODO: execution could be done staged

                Timer t;

                // linearize residuals
                error_total = lqr->linearizeProblem(&numerically_valid);
                BASALT_ASSERT_STREAM(
                    numerically_valid,
                    "did not expect numerical failure during linearization");
                linearizeProblemSeconds = t.reset();
                mpLogger->SolverScratch()
                    .add("linearizeProblem", linearizeProblemSeconds)
                    .format("ms");

                //        // compute pose jacobian norm squared for Jacobian
                //        scaling if (scale_Jp) {
                //          Jp_column_norm2 = lqr->getJp_diag2();
                //          stats.add("getJp_diag2", t.reset()).format("ms");
                //        }

                //        // scale landmark jacobians
                //        if (scale_Jl) {
                //          lqr->scaleJl_cols();
                //          stats.add("scaleJl_cols", t.reset()).format("ms");
                //        }

                // marginalize points in place
                lqr->performQR();
                performQrSeconds = t.reset();
                mpLogger->SolverScratch()
                    .add("performQR", performQrSeconds)
                    .format("ms");
            }

            // The zero velocity cost enters error_total, l_diff and
            // after_error_total as well as H and b, so that step acceptance
            // judges the objective the step was solved for. See
            // plans/vio_drift_analysis.md Phase 3b, step acceptance.
            const bool applyZeroVelocity =
                mpZeroVelocityWeight > 0 && mpIsStationary;
            const Scalar zeroVelocityW2 =
                mpZeroVelocityWeight * mpZeroVelocityWeight;
            if (applyZeroVelocity)
                error_total += Scalar(0.5) * zeroVelocityW2 *
                               frame_states.at(last_state_t_ns)
                                   .getState()
                                   .vel_w_i.squaredNorm();
            if (applyGsJump)
                error_total +=
                    Scalar(0.5) * gsJumpW2.dot(gsJump.cwiseProduct(gsJump));

            mpLogger->AddVioLinearise(
                last_state_t_ns, it, double(error_total), double(lambda),
                int(lmdb.numLandmarks()), int(frame_states.size()),
                int(frame_poses.size()), numerically_valid);
            // mpLogger->PrintVioLinearise();

            // compute pose jacobian scaling
            //      VecX jacobian_scaling;
            //      if (scale_Jp) {
            //        // TODO: what about jacobian scaling for SC solver?

            //        // ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
            //        // we use 1.0 / (eps + sqrt(SquaredColumnNorm))
            //        jacobian_scaling =
            //        (lqr_options.lb_options.jacobi_scaling_eps +
            //                            Jp_column_norm2.array().sqrt())
            //                               .inverse();
            //      }
            // if (config.vio_debug) {
            //   std::cout << "\t[INFO] Stage 1" << std::endl;
            //}

            // inner loop for backtracking in LM (still count as main iteration
            // though)
            for (int j = 0; it <= config.vio_max_iterations && !terminated;
                 j++) {
                if (j > 0) {
                    timer_iteration.reset();
                }

                {
                    // Timer t;

                    // TODO: execution could be done staged

                    //          // set (updated) damping for poses
                    //          if (config.vio_lm_pose_damping_variant == 0) {
                    //            lqr->setPoseDamping(lambda);
                    //            stats.add("setPoseDamping",
                    //            t.reset()).format("ms");
                    //          }

                    //          // scale landmark Jacobians only on the first
                    //          inner iteration. if (scale_Jp && j == 0) {
                    //            lqr->scaleJp_cols(jacobian_scaling);
                    //            stats.add("scaleJp_cols",
                    //            t.reset()).format("ms");
                    //          }

                    //          // set (updated) damping for landmarks
                    //          if (config.vio_lm_landmark_damping_variant == 0)
                    //          {
                    //            lqr->setLandmarkDamping(lambda);
                    //            stats.add("setLandmarkDamping",
                    //            t.reset()).format("ms");
                    //          }
                }

                // if (config.vio_debug) {
                //   std::cout << "\t[INFO] Stage 2 " << std::endl;
                // }

                VecX inc;
                double getDenseHBSeconds = 0.0, solveSeconds = 0.0;
                {
                    Timer t;

                    // get dense reduced camera system
                    MatX H;
                    VecX b;

                    lqr->get_dense_H_b(H, b);

                    // Zero velocity residual r = w v_w_i on the newest state,
                    // the only one the per frame stationarity test describes.
                    // The velocity block sits at offset 6 of the tangent
                    // ordering [trans, rot, vel, bias_gyro, bias_accel].
                    if (applyZeroVelocity) {
                        const int vi =
                            aom.abs_order_map.at(last_state_t_ns).first + 6;
                        H.template block<3, 3>(vi, vi).diagonal().array() +=
                            zeroVelocityW2;
                        b.template segment<3>(vi) +=
                            zeroVelocityW2 *
                            frame_states.at(last_state_t_ns).getState().vel_w_i;
                    }

                    if (applyGsJump) {
                        H.diagonal().template head<GRAVITY_SCALE_SIZE>() +=
                            gsJumpW2;
                        b.template head<GRAVITY_SCALE_SIZE>() +=
                            gsJumpW2.cwiseProduct(gsJump);
                    }


                    MatX gauge_P;
                    if (mpGsActive) {
                        if (gsHoldBlock) {
                            gauge_P = MatX::Identity(H.rows(), H.cols());
                        } else {
                            gauge_P = ComputeGaugeProjector(aom);
                            H = gauge_P * H * gauge_P;
                            b = gauge_P * b;
                        }
                        if (gsHoldBlock) {
                            const int nHeld = GRAVITY_SCALE_SIZE;
                            const int gs =
                                aom.abs_order_map.at(kGravityScaleStateId)
                                    .first;
                            H.middleRows(gs, nHeld).setZero();
                            H.middleCols(gs, nHeld).setZero();
                            H.diagonal().segment(gs, nHeld).setOnes();
                            b.segment(gs, nHeld).setZero();
                        }
                    }

                    getDenseHBSeconds = t.reset();
                    mpLogger->SolverScratch()
                        .add("get_dense_H_b", getDenseHBSeconds)
                        .format("ms");

                    int iter = 0;
                    bool inc_valid = false;
                    constexpr int max_num_iter = 3;

                    while (iter < max_num_iter && !inc_valid) {
                        VecX Hdiag_lambda =
                            (H.diagonal() * lambda).cwiseMax(min_lambda);
                        MatX H_copy = H;
                        H_copy.diagonal() += Hdiag_lambda;

                        Eigen::LDLT<Eigen::Ref<MatX>> ldlt(H_copy);
                        inc = ldlt.solve(b);
                        solveSeconds = t.reset();
                        mpLogger->SolverScratch()
                            .add("solve", solveSeconds)
                            .format("ms");

                        if (!inc.array().isFinite().all()) {
                            lambda = lambda_vee * lambda;
                            lambda_vee *= vee_factor;
                        } else {
                            inc_valid = true;
                        }
                        iter++;
                    }

                    if (!inc_valid) {
                        std::cerr << "Still invalid inc after " << max_num_iter
                                  << " iterations." << std::endl;
                    }

                    if (mpGsActive) inc = gauge_P * inc;
                }

                // backup state (then apply increment and check cost decrease)
                backup();

                // backsubstitute (with scaled pose increment)
                Scalar l_diff = 0;
                double backSubstituteSeconds = 0.0;
                {
                    // negate pose increment before point update
                    inc = -inc;

                    Timer t;
                    l_diff = lqr->backSubstitute(inc);
                    // Exact model decrease of the zero velocity cost, read
                    // before the increment is applied.
                    if (applyZeroVelocity) {
                        const int vi =
                            aom.abs_order_map.at(last_state_t_ns).first + 6;
                        const Vec3 dv = inc.template segment<3>(vi);
                        const Vec3& v =
                            frame_states.at(last_state_t_ns).getState().vel_w_i;
                        l_diff -= zeroVelocityW2 * dv.dot(v + Scalar(0.5) * dv);
                    }
                    if (applyGsJump) {
                        const Vec3 dg =
                            inc.template head<GRAVITY_SCALE_SIZE>();
                        l_diff -= gsJumpW2.dot(
                            dg.cwiseProduct(gsJump + Scalar(0.5) * dg));
                    }
                    backSubstituteSeconds = t.reset();
                    mpLogger->SolverScratch()
                        .add("backSubstitute", backSubstituteSeconds)
                        .format("ms");
                }

                // undo jacobian scaling before applying increment to poses
                //        if (scale_Jp) {
                //          inc.array() *= jacobian_scaling.array();
                //        }

                // apply increment to poses
                for (auto& [frame_id, state] : frame_poses) {
                    int idx = aom.abs_order_map.at(frame_id).first;
                    state.applyInc(inc.template segment<POSE_SIZE>(idx));
                }

                for (auto& [frame_id, state] : frame_states) {
                    int idx = aom.abs_order_map.at(frame_id).first;
                    state.applyInc(
                        inc.template segment<POSE_VEL_BIAS_SIZE>(idx));
                }

                if (mpGsActive) {
                    this->mGsState->ApplyInc(
                        inc.template head<GRAVITY_SCALE_SIZE>());
                }

                // compute stepsize
                Scalar step_norminf = inc.array().abs().maxCoeff();

                // compute error update applying increment
                Scalar after_update_marg_prior_error = 0;
                Scalar after_update_vision_and_inertial_error = 0;
                Scalar after_update_vision_error = 0;
                Scalar after_update_imu_error = 0, after_bg_error = 0,
                       after_ba_error = 0;

                double computeError2Seconds = 0.0;
                {
                    Timer t;
                    const Scalar gs_scale =
                        mpGsActive ? this->mGsState->GetScale() : Scalar(1);
                    const Vec3 gs_g =
                        mpGsActive ? Vec3(g.norm() * this->mGsState->GetDir())
                                   : g;

                    computeError(after_update_vision_and_inertial_error,
                                 nullptr, Scalar(0), gs_scale);
                    after_update_vision_error =
                        after_update_vision_and_inertial_error;
                    computeMargPriorError(marg_data,
                                          after_update_marg_prior_error);

                    ScBundleAdjustmentBase<Scalar>::computeImuError(
                        aom, after_update_imu_error, after_bg_error,
                        after_ba_error, frame_states, imu_meas,
                        gyro_bias_sqrt_weight.array().square(),
                        accel_bias_sqrt_weight.array().square(), gs_g,
                        gs_scale);

                    after_update_vision_and_inertial_error +=
                        after_update_imu_error + after_bg_error +
                        after_ba_error;

                    computeError2Seconds = t.reset();
                    mpLogger->SolverScratch()
                        .add("computerError2", computeError2Seconds)
                        .format("ms");
                }

                Scalar after_error_total =
                    after_update_vision_and_inertial_error +
                    after_update_marg_prior_error;
                if (applyZeroVelocity)
                    after_error_total += Scalar(0.5) * zeroVelocityW2 *
                                         frame_states.at(last_state_t_ns)
                                             .getState()
                                             .vel_w_i.squaredNorm();
                const Vec3 gsJumpNew =
                    applyGsJump
                        ? Vec3(gsJump + inc.template head<GRAVITY_SCALE_SIZE>())
                        : gsJump;
                if (applyGsJump)
                    after_error_total +=
                        Scalar(0.5) *
                        gsJumpW2.dot(gsJumpNew.cwiseProduct(gsJumpNew));

                // check cost decrease compared to quadratic model cost
                Scalar f_diff;
                bool step_is_valid = false;
                bool step_is_successful = false;
                Scalar relative_decrease = 0;
                {
                    // compute actual cost decrease
                    f_diff = error_total - after_error_total;

                    relative_decrease = f_diff / l_diff;

                    // TODO: consider to remove assert. For now we want to test
                    // if we even run into the l_diff <= 0 case ever in practice
                    // BASALT_ASSERT_STREAM(l_diff > 0, "l_diff " << l_diff);

                    // l_diff <= 0 is a theoretical possibility if the model
                    // cost change is tiny and becomes numerically negative
                    // (non-positive). It might not occur since our linear
                    // systems are not that big (compared to large scale BA for
                    // example) and we also abort optimization quite early and
                    // usually don't have large damping (== tiny step size).
                    step_is_valid = l_diff > 0;
                    step_is_successful = step_is_valid && relative_decrease > 0;
                }

                double iteration_time = timer_iteration.elapsed();
                double cumulative_time = timer_total.elapsed();

                mpLogger->SolverScratch()
                    .add("iteration", iteration_time)
                    .format("ms");
                double residentMemory = 0.0, residentMemoryPeak = 0.0;
                {
                    basalt::MemoryInfo mi;
                    if (get_memory_info(mi)) {
                        residentMemory = mi.resident_memory;
                        residentMemoryPeak = mi.resident_memory_peak;
                        mpLogger->SolverScratch().add("resident_memory",
                                                      residentMemory);
                        mpLogger->SolverScratch().add("resident_memory_peak",
                                                      residentMemoryPeak);
                    }
                }
                mpLogger->AddVioSolverTiming(
                    last_state_t_ns, numCams, numLms, numObs,
                    allocateLmbSeconds, linearizeProblemSeconds,
                    performQrSeconds, getDenseHBSeconds, solveSeconds,
                    backSubstituteSeconds, computeError2Seconds, iteration_time,
                    residentMemory, residentMemoryPeak);
                // mpLogger->PrintVioSolverTiming();

                if (step_is_successful) {
                    BASALT_ASSERT(step_is_valid);

                    mpLogger->AddVioSolverIter(
                        last_state_t_ns, it, j, 0, double(after_error_total),
                        double(f_diff), double(l_diff),
                        double(relative_decrease), double(step_norminf),
                        double(after_update_vision_error),
                        double(after_update_imu_error), double(after_bg_error),
                        double(after_ba_error),
                        double(after_update_marg_prior_error), double(lambda),
                        iteration_time, cumulative_time);
                    // mpLogger->PrintVioSolverIter();

                    lambda *= std::max<Scalar>(
                        Scalar(1.0) / 3,
                        1 - std::pow<Scalar>(2 * relative_decrease - 1, 3));
                    lambda = std::max(min_lambda, lambda);

                    lambda_vee = initial_vee;
                    gsJump = gsJumpNew;

                    it++;

                    // check function and parameter tolerance
                    if ((f_diff > 0 && f_diff < Scalar(1e-6)) ||
                        step_norminf < Scalar(1e-4)) {
                        converged = true;
                        terminated = true;
                    }

                    // stop inner lm loop
                    break;
                } else {
                    mpLogger->AddVioSolverIter(
                        last_state_t_ns, it, j, 1, double(after_error_total),
                        double(f_diff), double(l_diff),
                        double(relative_decrease), double(step_norminf),
                        double(after_update_vision_error),
                        double(after_update_imu_error), double(after_bg_error),
                        double(after_ba_error),
                        double(after_update_marg_prior_error), double(lambda),
                        iteration_time, cumulative_time);
                    // mpLogger->PrintVioSolverIter();

                    lambda = lambda_vee * lambda;
                    lambda_vee *= vee_factor;

                    //        lambda = std::max(min_lambda, lambda);
                    //        lambda = std::min(max_lambda, lambda);

                    restore();
                    it++;
                    it_rejected++;

                    if (lambda > max_lambda) {
                        terminated = true;
                    }
                }
            }
        }

        if (mpGsActive) {
            mpGsJumpCost =
                Scalar(0.5) * gsJumpW2.dot(gsJump.cwiseProduct(gsJump));
        }
        if (mpGsActive) RecentreGravityScale();

        const double optimizeSeconds = timer_total.elapsed();
        mpLogger->SolverScratch().add("optimize", optimizeSeconds).format("ms");
        mpLogger->SolverScratch().add("num_it", double(it)).format("count");
        mpLogger->SolverScratch()
            .add("num_it_rejected", double(it_rejected))
            .format("count");

        // TODO: call filterOutliers at least once (also for CG version)

        mpLogger->FinishVioOptimize();

        const int reasonCode = converged ? 0 : (terminated ? 3 : 2);
        mpLogger->AddVioSolverSummary(last_state_t_ns, it, it_rejected,
                                      converged, terminated, reasonCode,
                                      optimizeSeconds);
        // mpLogger->PrintVioSolverSummary();
    }
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::optimize_and_marg(
    const std::map<int64_t, int>& num_points_connected,
    const std::unordered_set<KeypointId>& lost_landmaks) {
    optimize();
    marginalize(num_points_connected, lost_landmaks);
    PublishKeyframe();
}

// Hands the just-selected keyframe to the local mapper. Runs after
// marginalisation so the pose shipped is the jointly optimised one.
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::PublishKeyframe() {
    if (!mpIsCurrentFrameKF) return;
    mpIsCurrentFrameKF = false;

    if (!this->mpKFOutputQueue) return;

    const auto it_state = frame_states.find(last_state_t_ns);
    const auto it_flow = prev_opt_flow_res.find(last_state_t_ns);
    if (it_state == frame_states.end() || it_flow == prev_opt_flow_res.end())
        return;

    Keyframe::Ptr kf(new Keyframe);
    kf->timestamp = last_state_t_ns;
    // Rebuilt rather than copied so linearized stays false, which
    // NfrMapper::optimize asserts before applying an increment.
    kf->pose = PoseStateWithLin<double>(
        last_state_t_ns,
        it_state->second.getState().T_w_i.template cast<double>());
    kf->opt_flow_res = it_flow->second;
    if (this->mGsCorrectionLog) {
        kf->mGsEpoch = this->mGsCorrectionLog->LatestEpoch();
    }

    this->mpKFOutputQueue->push(kf);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::SetInitialPriorOrder(int64_t t_ns) {
    marg_data.order = AbsOrderMap();

    if (mpGsActive) {
        marg_data.order.abs_order_map[kGravityScaleStateId] =
            std::make_pair(0, GRAVITY_SCALE_SIZE);
        marg_data.order.total_size = GRAVITY_SCALE_SIZE;
        marg_data.order.items = 1;
    }

    marg_data.order.abs_order_map[t_ns] =
        std::make_pair(int(marg_data.order.total_size), POSE_VEL_BIAS_SIZE);
    marg_data.order.total_size += POSE_VEL_BIAS_SIZE;
    marg_data.order.items++;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::LogGravityScale(int swapEvent) {
    const int64_t epoch = this->mGsCorrectionLog->LatestEpoch();
    const GravityScaleCorrectionLog::Entry cumulative =
        this->mGsCorrectionLog->Between(0, epoch);
    mpLogger->AddVioGravityScale(
        last_state_t_ns, epoch, double(mpGsStepSigma), double(mpGsStepTiltDeg),
        cumulative.mScale, cumulative.mQ.log().norm() * 180.0 / M_PI,
        swapEvent, double(mpGsJumpCost), mpGsVisualImuFull, mpGsVisualImuRot,
        double(mpGsVisualImuScaleCol));
}

template <class Scalar_>
typename SqrtKeypointVioEstimator<Scalar_>::MatX
SqrtKeypointVioEstimator<Scalar_>::ComputeGaugeProjector(
    const AbsOrderMap& aom) const {
    const int n = int(aom.total_size);
    const int gs = aom.abs_order_map.at(kGravityScaleStateId).first;
    const Eigen::Matrix<Scalar, 3, 2>& basis = this->mGsState->mBasisLin;

    MatX N = MatX::Zero(n, 3);
    N(gs + 2, 0) = Scalar(-1);
    N.template block<2, 1>(gs, 1) = basis.transpose() * Vec3::UnitX();
    N.template block<2, 1>(gs, 2) = basis.transpose() * Vec3::UnitY();

    for (const auto& [t_ns, idx] : aom.abs_order_map) {
        if (t_ns == kGravityScaleStateId) continue;

        const int c = idx.first;
        const bool has_vel = idx.second == POSE_VEL_BIAS_SIZE;

        Vec3 p;
        Vec3 v = Vec3::Zero();
        if (has_vel) {
            const PoseVelBiasState<Scalar>& st =
                frame_states.at(t_ns).getStateLin();
            p = st.T_w_i.translation();
            v = st.vel_w_i;
        } else {
            p = frame_poses.at(t_ns).getPoseLin().translation();
        }

        N.template block<3, 1>(c, 0) = p;
        if (has_vel) N.template block<3, 1>(c + 6, 0) = v;

        for (int k = 0; k < 2; k++) {
            const Vec3 a = Vec3::Unit(k);
            N.template block<3, 1>(c, 1 + k) = a.cross(p);
            N.template block<3, 1>(c + 3, 1 + k) = a;
            if (has_vel) N.template block<3, 1>(c + 6, 1 + k) = a.cross(v);
        }
    }

    const Eigen::Matrix<Scalar, 3, 3> NtN = N.transpose() * N;
    return MatX::Identity(n, n) - N * NtN.ldlt().solve(N.transpose());
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::RecentreGravityScale() {
    GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
    const Vec3 dir = gs.GetDir();
    const Sophus::SO3<Scalar> Q = gs.AlignmentRotation(g);
    const Scalar sigma_c = gs.GetSigma();
    const Scalar s = std::exp(sigma_c);
    const Mat3 Q_inv_m = Q.inverse().matrix();

    auto transform_prior = [&](MargLinData<Scalar>& mld) {
        for (const auto& [id, idx] : mld.order.abs_order_map) {
            if (id == kGravityScaleStateId) continue;
            const int col = idx.first;
            mld.H.middleCols(col, 3) = mld.H.middleCols(col, 3) * Q_inv_m / s;
            mld.H.middleCols(col + 3, 3) =
                mld.H.middleCols(col + 3, 3) * Q_inv_m;
            if (idx.second == POSE_VEL_BIAS_SIZE) {
                mld.H.middleCols(col + 6, 3) =
                    mld.H.middleCols(col + 6, 3) * Q_inv_m / s;
            }
        }
    };

    transform_prior(marg_data);
    if (mpDm) mpDm->Transform(Q, s, sigma_c);

    for (auto& [frame_id, state] : frame_poses) state.TransformBySim3(Q, s);
    for (auto& [frame_id, state] : frame_states) state.TransformBySim3(Q, s);

    gs.TransformLin(Q, sigma_c);

    auto transform_anchor = [&](GsAnchor& anchor) {
        if (!anchor.set) return;
        anchor.dir = Q * anchor.dir;
        anchor.basis = Q.matrix() * anchor.basis;
        anchor.sigma -= sigma_c;
    };

    transform_anchor(mpAnchorCurr);

    mpGsStepSigma = sigma_c;
    mpGsStepTiltDeg =
        std::acos(std::clamp(dir.dot(g.normalized()), Scalar(-1), Scalar(1))) *
        Scalar(180) / Scalar(M_PI);

    this->mGsCorrectionLog->Append(Q.template cast<double>(), double(s));
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::AnchorGs(GsAnchor& anchor) {
    GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
    if (!anchor.set) {
        anchor.dir = gs.GetDir();
        anchor.basis = gs.GetBasis();
        anchor.sigma = gs.GetSigma();
        anchor.set = true;
    }
    gs.Reanchor(anchor.dir, anchor.basis, anchor.sigma);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::LinearizeMargGS(
    const AbsOrderMap& aom, const std::set<int64_t>& kfs_to_marg,
    const std::unordered_set<KeypointId>& lost_landmaks,
    int64_t last_state_to_marg, MatX& Q2Jp_curr, VecX& Q2r_curr,
    MatX& Q2Jp_store, VecX& Q2r_store) {
    typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
    lqr_options.lb_options.huber_parameter = huber_thresh;
    lqr_options.lb_options.obs_std_dev = obs_std_dev;
    lqr_options.linearization_type = config.vio_linearization_type;

    ImuLinData<Scalar> ild = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight,
                              {}};

    for (const auto& kv : imu_meas) {
        int64_t start_t = kv.second.get_start_t_ns();
        int64_t end_t = kv.second.get_start_t_ns() + kv.second.get_dt_ns();

        if (aom.abs_order_map.count(start_t) == 0 ||
            aom.abs_order_map.count(end_t) == 0)
            continue;

        ild.imu_meas[kv.first] = &kv.second;
    }

    {
        AnchorGs(mpAnchorCurr);
        LinearizationGSAbsQR<Scalar, POSE_SIZE> lqr(
            this, aom, lqr_options, &*this->mGsState, false, &marg_data, &ild,
            &kfs_to_marg, &lost_landmaks, last_state_to_marg);
        lqr.linearizeProblem();
        lqr.performQR();
        lqr.get_dense_Q2Jp_Q2r(Q2Jp_curr, Q2r_curr);
    }

    {
        // Visual rows of the store, scale free and evaluated at the current
        // block, plans/delayed_marginalisation.md §2.3.3.
        GravityScaleStateWithLin<Scalar> gsNow = *this->mGsState;
        gsNow.Reanchor(gsNow.GetDir(), gsNow.GetBasis(), gsNow.GetSigma());
        LinearizationGSAbsQR<Scalar, POSE_SIZE> lqr(
            this, aom, lqr_options, &gsNow, true, nullptr, nullptr,
            &kfs_to_marg, &lost_landmaks, last_state_to_marg);
        lqr.linearizeProblem();
        lqr.performQR();
        lqr.get_dense_Q2Jp_Q2r(Q2Jp_store, Q2r_store);
    }

    AnchorGs(mpAnchorCurr);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::MarginalizeGS(
    MatX& Q2Jp_curr, VecX& Q2r_curr, const std::set<int>& idx_to_keep,
    const std::set<int>& idx_to_marg) {
    MatX H_new;
    VecX b_new;
    MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
        Q2Jp_curr, Q2r_curr, idx_to_keep, idx_to_marg, H_new, b_new);
    marg_data.H = H_new;
    marg_data.b = b_new;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::RecentrePriorsGS() {
    AnchorGs(mpAnchorCurr);
    VecX delta;
    computeDelta(marg_data.order, delta);
    marg_data.b -= marg_data.H * delta;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::StripGsFromMargData(
    MargData& m) const {
    constexpr int c = GRAVITY_SCALE_SIZE;
    const int n = int(m.aom.total_size) - c;

    BASALT_ASSERT(m.aom.abs_order_map.at(kGravityScaleStateId).first == 0);

    const Eigen::MatrixXd H_new = m.abs_H.bottomRightCorner(n, n);
    const Eigen::VectorXd b_new = m.abs_b.tail(n);
    m.abs_H = H_new;
    m.abs_b = b_new;

    AbsOrderMap aom_new;
    for (const auto& [id, idx] : m.aom.abs_order_map) {
        if (id == kGravityScaleStateId) continue;
        aom_new.abs_order_map[id] = std::make_pair(idx.first - c, idx.second);
        aom_new.total_size += idx.second;
        aom_new.items++;
    }
    m.aom = aom_new;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::FoldGravityScale() {
    StopDelayedMarg();
    AnchorGs(mpAnchorCurr);

    const GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
    constexpr int c = GRAVITY_SCALE_SIZE;

    BASALT_ASSERT(marg_data.order.abs_order_map.at(kGravityScaleStateId).first ==
                  0);

    marg_data.b += marg_data.H.leftCols(c) * gs.GetDelta();
    MatX J = marg_data.H.rightCols(marg_data.H.cols() - c);

    AbsOrderMap order;
    for (const auto& [id, idx] : marg_data.order.abs_order_map) {
        if (id == kGravityScaleStateId) continue;
        order.abs_order_map[id] = std::make_pair(idx.first - c, idx.second);
        order.total_size += idx.second;
        order.items++;
    }

    marg_data.H = J;
    marg_data.order = order;

    // The debug prior is frozen during the period, resynchronised here.
    nullspace_marg_data.H = marg_data.H;
    nullspace_marg_data.b = marg_data.b;
    nullspace_marg_data.order = marg_data.order;

    mpGsStepSigma = 0;
    mpGsStepTiltDeg = 0;
    LogGravityScale(4);

    this->mGsState.reset();
    this->mGsStateBackup.reset();
    mpGsActive = false;
}

template <class Scalar_>
std::shared_ptr<DelayedMargJob<Scalar_>>
SqrtKeypointVioEstimator<Scalar_>::MakeDelayedJob(bool coarse) const {
    auto job = std::make_shared<DelayedMargJob<Scalar>>();
    job->mBoundary = mpDm->Boundary();
    job->mvSteps.assign(mpDm->Steps().begin(), mpDm->Steps().end());
    job->mValues.mvRecords = mpDm->Values().mvRecords;
    job->mValues.mvWindowStates = frame_states;
    job->mValues.mvWindowPoses = frame_poses;
    job->mGs = *this->mGsState;
    job->mG = g;
    job->mGyroBiasWeightSqrt = gyro_bias_sqrt_weight;
    job->mAccelBiasWeightSqrt = accel_bias_sqrt_weight;
    job->mLog = this->mGsCorrectionLog;
    job->mEpoch = this->mGsCorrectionLog->LatestEpoch();
    // Readvances of the coarse phase keep the departed states, so that the
    // coarse optimisation alone moves the history, except after a coarse
    // optimisation that estimated the scale.
    job->mSmooth = config.vio_gs_dm_smooth && (coarse || !mpDmCoarse);
    job->mCoarse = coarse;
    if (coarse) {
        for (const auto& kv : imu_meas) {
            job->mvWindowImu.push_back(kv.second);
            job->mvWindowImu.back().get_sqrt_cov_inv();
        }
        job->mInitialDir =
            this->mGsCorrectionLog->Between(0, job->mEpoch)
                .mQ.template cast<Scalar>() *
            g.normalized();
        if (config.vio_gravity_prior_std_deg > 0) {
            const Scalar sigma =
                Scalar(config.vio_gravity_prior_std_deg * M_PI / 180.0);
            job->mTiltPriorW = Scalar(1) / (sigma * sigma);
        }
        job->mKfChain = mpDm->KfChain();
        job->mvKeyframes = mpDm->Keyframes();
        job->mvKeyframes.insert(kf_ids.begin(), kf_ids.end());
        job->mScaleVarMax = Scalar(config.vio_gs_dm_coarse_scale_var);
        // A quarter of the frame period, so that the readvance that follows
        // also ends before the next frame.
        if (!imu_meas.empty()) {
            job->mCoarseDeadlineSeconds =
                0.25 * double(imu_meas.rbegin()->second.get_dt_ns()) * 1e-9;
        }
    }
    return job;
}

// Applies a coarse optimisation as a re-centring of the window, the priors and
// the store, its velocity increments and its one bias to every state, so that
// the prior the job readvanced at these values can be installed, §2.3.9.
// Returns false when the job had no keyframe pair to fit or ran out of time.
template <class Scalar_>
bool SqrtKeypointVioEstimator<Scalar_>::InstallCoarseJob(
    const DelayedMargJob<Scalar>& job) {
    if (job.mCoarseSpans == 0 || job.mCoarseTimedOut) return false;
    const GravityScaleCorrectionLog::Entry c =
        this->mGsCorrectionLog->Between(
            job.mEpoch, this->mGsCorrectionLog->LatestEpoch());
    const Vec3 dir = c.mQ.template cast<Scalar>() * job.mCoarseDir;
    const Scalar sigma = job.mCoarseSigma - Scalar(std::log(c.mScale));
    DelayedMargSetBlock(*this->mGsState, dir, sigma);
    RecentreGravityScale();
    AnchorGs(mpAnchorCurr);

    // The velocity increments were solved in the epoch of the snapshot.
    const GravityScaleCorrectionLog::Entry cNow =
        this->mGsCorrectionLog->Between(
            job.mEpoch, this->mGsCorrectionLog->LatestEpoch());
    const Mat3 Qm = cNow.mQ.template cast<Scalar>().matrix();
    const Scalar s = Scalar(cNow.mScale);
    auto inc = [&](int64_t id, const PoseVelBiasState<Scalar>& st) {
        typename PoseVelBiasStateWithLin<Scalar>::VecN x;
        x.setZero();
        const auto it = job.mvCoarseVelInc.find(id);
        if (it != job.mvCoarseVelInc.end()) {
            x.template segment<3>(6) = s * (Qm * it->second);
        }
        x.template segment<3>(9) = job.mCoarseBg - st.bias_gyro;
        x.template segment<3>(12) = job.mCoarseBa - st.bias_accel;
        return x;
    };
    for (auto& [id, state] : frame_states) {
        state.applyInc(inc(id, state.getState()));
    }
    for (auto& [id, rec] : mpDm->Values().mvRecords) {
        if (frame_states.count(id) == 0) {
            rec.mState.applyInc(inc(id, rec.mState.getState()));
        }
    }
    return true;
}

template <class Scalar_>
int SqrtKeypointVioEstimator<Scalar_>::InstallDelayedJob(
    DelayedMargJob<Scalar>& job) {
    const int64_t now = this->mGsCorrectionLog->LatestEpoch();
    DelayedMargPrior<Scalar> prior = job.mResult;

    // The job ran in the epoch of its snapshot.
    Sophus::SO3<Scalar> Q;
    Scalar s = 1;
    if (job.mEpoch != now) {
        const GravityScaleCorrectionLog::Entry c =
            this->mGsCorrectionLog->Between(job.mEpoch, now);
        Q = c.mQ.template cast<Scalar>();
        s = Scalar(c.mScale);
        DelayedMargTransformColumns(prior.mH, prior.mOrder, Q, s);
        prior.mAnchor.Transform(Q, std::log(s));
    }

    DelayedMargValues<Scalar>& values = mpDm->Values();
    const Mat3 Qm = Q.matrix();
    for (const auto& [id, inc] : job.mvSmoothInc) {
        auto it = values.mvRecords.find(id);
        if (it == values.mvRecords.end()) continue;
        typename PoseVelBiasStateWithLin<Scalar>::VecN x = inc.first;
        x.template segment<3>(0) = s * (Qm * x.template segment<3>(0));
        x.template segment<3>(3) = Qm * x.template segment<3>(3);
        x.template segment<3>(6) = s * (Qm * x.template segment<3>(6));
        it->second.mState.applyInc(x);
    }

    // Steps stored while the job ran, readvanced here at the current block.
    values.mvWindowStates = frame_states;
    values.mvWindowPoses = frame_poses;
    GravityScaleStateWithLin<Scalar> gsNow = *this->mGsState;
    gsNow.Reanchor(gsNow.GetDir(), gsNow.GetBasis(), gsNow.GetSigma());
    int catchup = 0;
    for (const auto& step : mpDm->Steps()) {
        if (step->mIndex <= job.LastStep()) continue;
        DelayedMargJob<Scalar>::MarginaliseStep(
            *step, values, gsNow, g, gyro_bias_sqrt_weight,
            accel_bias_sqrt_weight, this->mGsCorrectionLog.get(), now, false,
            Scalar(0), prior, nullptr);
        catchup++;
    }

    BASALT_ASSERT(prior.mOrder.abs_order_map == marg_data.order.abs_order_map);
    marg_data.H = prior.mH;
    marg_data.b = prior.mB;
    mpAnchorCurr.dir = prior.mAnchor.mDir;
    mpAnchorCurr.basis = prior.mAnchor.mBasis;
    mpAnchorCurr.sigma = prior.mAnchor.mSigma;
    mpAnchorCurr.set = true;
    AnchorGs(mpAnchorCurr);
    return catchup;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::UpdateDelayedMarg(
    DelayedMargStep<Scalar>&& step) {
    Timer tStore;
    const int64_t epoch = this->mGsCorrectionLog->LatestEpoch();
    mpDm->Push(std::move(step));
    DelayedMargValues<Scalar>& values = mpDm->Values();
    values.mvWindowStates = frame_states;
    values.mvWindowPoses = frame_poses;
    mpDm->AdvanceBoundary(*this->mGsState, g, gyro_bias_sqrt_weight,
                          accel_bias_sqrt_weight,
                          this->mGsCorrectionLog.get(), epoch,
                          config.vio_gs_visual_prior_inertial,
                          Scalar(config.vio_gs_visual_prior_scale_tol));
    const double storeSeconds = mpDmStoreSeconds + tStore.elapsed();

    int started = 0, installed = 0, coarse = 0, catchup = 0;
    double jobSeconds = 0, catchupSeconds = 0;
    Scalar maxPosInc = 0;
    const DelayedMargJob<Scalar>* coarseJob = nullptr;
    std::shared_ptr<DelayedMargJob<Scalar>> doneJob;

    auto install = [&](const std::shared_ptr<DelayedMargJob<Scalar>>& job) {
        Timer t;
        if (job->mCoarse) {
            coarseJob = job.get();
            coarse = 1;
            if (InstallCoarseJob(*job)) {
                catchup = InstallDelayedJob(*job);
                installed = 1;
                mpGsSwapEvent = 5;
            }
        } else {
            catchup = InstallDelayedJob(*job);
            maxPosInc = job->mMaxPosInc;
            installed = 1;
            mpGsSwapEvent = 6;
        }
        catchupSeconds = t.elapsed();
        jobSeconds = job->mSeconds;
        doneJob = job;
    };

    // A job started at the previous marginalisation is installed now, waiting
    // for it if needed, so that every job takes effect within one frame.
    double waitSeconds = 0;
    if (mpDmFuture.valid()) {
        Timer tWait;
        std::shared_ptr<DelayedMargJob<Scalar>> job = mpDmFuture.get();
        waitSeconds = tWait.elapsed();
        install(job);
    }

    // The coarse phase ends once the store first holds the delay, and a
    // smoothed readvance starts the delayed phase.
    const int steps = int(mpDm->Steps().size());
    if (mpDmCoarse && mpDm->NextIndex() >= config.vio_gs_dm_delay &&
        !mpDmFuture.valid()) {
        mpDmCoarse = false;
        mpDmForceReplace = true;
    }

    const GravityScaleStateWithLin<Scalar>& gs = *this->mGsState;
    const Scalar anchorSigma = std::abs(gs.GetSigma() - mpAnchorCurr.sigma);
    const Scalar anchorTilt = std::acos(std::clamp(
        gs.GetDir().dot(mpAnchorCurr.dir), Scalar(-1), Scalar(1)));
    const int records = int(values.mvRecords.size());

    auto start = [&](bool coarseJobKind) {
        std::shared_ptr<DelayedMargJob<Scalar>> job =
            MakeDelayedJob(coarseJobKind);
        started = coarseJobKind ? 2 : 1;
        if (config.vio_gs_dm_async) {
            mpDmFuture = std::async(std::launch::async, [job]() {
                job->Run();
                return job;
            });
        } else {
            job->Run();
            install(job);
        }
    };

    if (!mpDmFuture.valid() && steps > 0) {
        if (mpDmForceReplace ||
            (!mpDmCoarse &&
             (anchorSigma > std::log(Scalar(config.vio_gs_dm_thresh_scale)) ||
              anchorTilt >
                  Scalar(config.vio_gs_dm_thresh_tilt_deg * M_PI / 180.0)))) {
            mpDmForceReplace = false;
            start(false);
        } else if (mpDmCoarse && int(mpDm->Keyframes().size()) > 5 &&
                   *mpDm->Keyframes().rbegin() > mpDmLastCoarseKeyframe) {
            // DM-VIO runs its coarse initialiser at every new keyframe once
            // more than five are known.
            mpDmLastCoarseKeyframe = *mpDm->Keyframes().rbegin();
            start(true);
        }
    }

    mpLogger->AddVioDelayedMarg(
        last_state_t_ns, steps, records, started, installed, coarse,
        double(anchorSigma), double(anchorTilt) * 180.0 / M_PI, jobSeconds,
        catchup, catchupSeconds, double(maxPosInc), storeSeconds,
        mpDmCoarse ? 1 : 0, coarseJob ? double(coarseJob->mCoarseSigma) : 0.0,
        coarseJob ? double(coarseJob->mCoarseTilt) * 180.0 / M_PI : 0.0,
        coarseJob ? double(coarseJob->mCoarseScaleVar) : 0.0,
        coarseJob ? int(coarseJob->mCoarseScaleFree) : 0,
        coarseJob ? coarseJob->mCoarseBa.norm() : 0.0,
        coarseJob ? double(coarseJob->mCoarseCost0) : 0.0,
        coarseJob ? double(coarseJob->mCoarseCost1) : 0.0,
        coarseJob ? coarseJob->mCoarseSpans : 0,
        coarseJob ? double(coarseJob->mCoarseBg.norm()) : 0.0,
        coarseJob ? double(coarseJob->mCoarseMaxVelInc) : 0.0,
        coarseJob ? coarseJob->mCoarseIterations : 0,
        coarseJob ? int(coarseJob->mCoarseTimedOut) : 0, waitSeconds);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::StopDelayedMarg() {
    if (mpDmFuture.valid()) mpDmFuture.get();
    mpDm.reset();
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::debug_finalize() {
    // mpLogger->PrintSummary();
    mpLogger->SaveLegacyStats();
    mpLogger->SaveAll();
}

// //////////////////////////////////////////////////////////////////
// instatiate templates

#ifdef BASALT_INSTANTIATIONS_DOUBLE
template class SqrtKeypointVioEstimator<double>;
#endif

#ifdef BASALT_INSTANTIATIONS_FLOAT
template class SqrtKeypointVioEstimator<float>;
#endif

}  // namespace basalt
