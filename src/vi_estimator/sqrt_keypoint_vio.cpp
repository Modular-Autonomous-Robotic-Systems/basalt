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

#include <basalt/linearization/linearization_base.hpp>
#include <basalt/utils/cast_utils.hpp>
#include <basalt/utils/format.hpp>
#include <basalt/utils/time_utils.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

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

        // small prior to avoid jumps in bias
        marg_data.H.diagonal().template segment<3>(9).array() =
            Scalar(config.vio_init_ba_weight);
        marg_data.H.diagonal().template segment<3>(12).array() =
            Scalar(config.vio_init_bg_weight);
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

    marg_data.order.abs_order_map[t_ns] = std::make_pair(0, POSE_VEL_BIAS_SIZE);
    marg_data.order.total_size = POSE_VEL_BIAS_SIZE;
    marg_data.order.items = 1;

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
        while (imuData->t_ns < curr_frame->t_ns) {
            imuData = popFromImuDataQueue();
            if (!imuData) break;
            numImuSkippedBehind++;
            imuData->accel =
                this->calib.calib_accel_bias.getCalibrated(imuData->accel);
            imuData->gyro =
                this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
            // std::cout << "Skipping IMU data.." << std::endl;
        }

        if (!imuData) return nullptr;

        Vec3 vel_w_i_init;
        vel_w_i_init.setZero();

        T_w_i_init.setQuaternion(Eigen::Quaternion<Scalar>::FromTwoVectors(
            imuData->accel, Vec3::UnitZ()));

        // Turn the gravity aligned world about Z so body X takes the ground
        // truth heading. Tilt stays with the accelerometer, which the initial
        // bias state is consistent with.
        if (mpCurrentGTPose) {
            const Eigen::Matrix<Scalar, 3, 3> rAcc = T_w_i_init.so3().matrix();
            const Eigen::Matrix3d rGt = mpCurrentGTPose->so3().matrix();
            const Scalar yawOffset =
                Scalar(std::atan2(rGt(1, 0), rGt(0, 0))) -
                std::atan2(rAcc(1, 0), rAcc(0, 0));
            T_w_i_init.so3() =
                Sophus::SO3<Scalar>::rotZ(yawOffset) * T_w_i_init.so3();
            T_w_i_init.translation() =
                mpCurrentGTPose->translation().template cast<Scalar>();
        }

        const Vec3 bodyZInWorld = T_w_i_init.so3() * Vec3::UnitZ();
        const double tiltDeg =
            std::acos(std::clamp(double(bodyZInWorld.z()), -1.0, 1.0)) *
            180.0 / M_PI;

        last_state_t_ns = curr_frame->t_ns;
        imu_meas[last_state_t_ns] =
            IntegratedImuMeasurement<Scalar>(last_state_t_ns, mpBg, mpBa);
        frame_states[last_state_t_ns] = PoseVelBiasStateWithLin<Scalar>(
            last_state_t_ns, T_w_i_init, vel_w_i_init, mpBg, mpBa, true);

        marg_data.order.abs_order_map[last_state_t_ns] =
            std::make_pair(0, POSE_VEL_BIAS_SIZE);
        marg_data.order.total_size = POSE_VEL_BIAS_SIZE;
        marg_data.order.items = 1;

        std::cout << "Setting up filter: t_ns " << last_state_t_ns << std::endl;
        std::cout << "T_w_i\n" << T_w_i_init.matrix() << std::endl;
        std::cout << "vel_w_i " << vel_w_i_init.transpose() << std::endl;

        mpLogger->AddVioInit(
            curr_frame->t_ns, imuData->t_ns, imuQueueSizeOnEntry,
            numImuSkippedBehind, imuData->accel.template cast<double>(),
            imuData->gyro.template cast<double>(), mpBg.template cast<double>(),
            mpBa.template cast<double>(), g.template cast<double>(),
            T_w_i_init.unit_quaternion().coeffs().template cast<double>(),
            tiltDeg);
        mpLogger->PrintVioInit();

        if (config.vio_debug || config.vio_extended_logging) {
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

        if (imuAhead) {
            while (imuData->t_ns <= curr_frame->t_ns) {
                meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
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
        mpLogger->AddVioImuFrame(
            curr_frame->t_ns, double(frameDtNs) * 1e-9, numImuIntegrated,
            int(std::round(double(frameDtNs) * 1e-9 *
                          this->calib.imu_update_rate)),
            numImuSkippedBehind, firstIntegratedImuTNs, lastIntegratedImuTNs,
            coverage, double(meas->get_dt_ns()) * 1e-9, stretchFallbackFired,
            double(stretchSpanNs) * 1e-9, int(imuQueueSizeOnEntry),
            int(this->imu_data_queue.size()),
            double(imuData ? imuData->t_ns - curr_frame->t_ns : 0) * 1e-9);
        mpLogger->PrintVioImuFrame();

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
        if (!mpPosesToUpdate.empty()) {
            for (auto it = mpPosesToUpdate.begin();
                 it != mpPosesToUpdate.end();) {
                const int64_t t_ns = it->first;
                const PoseStateWithLin<double>& lm_pose = it->second;
                const SE3 T_new = lm_pose.getPose().template cast<Scalar>();

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
                it = mpPosesToUpdate.erase(it);
                // ++it;
            }
        }
    }
    const double poseUpdateSeconds = tPoseUpdate.elapsed();
    mpLogger->AddVioPoseUpdate(opt_flow_meas->t_ns, poseUpdatePending,
                               poseUpdateApplied, poseUpdateRejected,
                               double(poseUpdateMaxTransErr),
                               double(poseUpdateMaxRotErr),
                               double(kRelinThresholdTrans),
                               double(kRelinThresholdRot));
    mpLogger->PrintVioPoseUpdate();

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
    mpLogger->AddVioAssoc(opt_flow_meas->t_ns, int(obs0), connected0,
                         int(unconnected_obs0.size()),
                         config.vio_new_kf_keypoints_thresh, frames_after_kf,
                         int(lmdb.numLandmarks()), int(kf_ids.size()),
                         int(frame_states.size()), int(frame_poses.size()),
                         take_kf);
    mpLogger->PrintVioAssoc();

    Timer tTriang;
    const bool tookKf = take_kf;
    if (take_kf) {
        // Triangulate new points from one of the observations (with sufficient
        // baseline) and make keyframe for camera 0
        take_kf = false;
        mpIsCurrentFrameKF = true;
        frames_after_kf = 0;
        kf_ids.emplace(last_state_t_ns);

        TimeCamId tcidl(opt_flow_meas->t_ns, 0);

        int num_points_added = 0;
        // Triangulation rejection tally, mirroring the local mapper's
        // [setup_opt] breakdown so the two front ends can be compared.
        int numNoPriorObs = 0, numUnprojectFail = 0, numShortBaseline = 0;
        int numNotFinite = 0, numBehind = 0, numTooClose = 0;
        Scalar minBaseline = std::numeric_limits<Scalar>::max();
        Scalar maxBaseline = 0;
        Scalar minDepth = std::numeric_limits<Scalar>::max();
        Scalar maxDepth = 0;
        Scalar sumDepth = 0;
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
                if (T_0_1.translation().squaredNorm() < min_triang_distance2) {
                    numShortBaseline++;
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
            opt_flow_meas->t_ns, int(unconnected_obs0.size()),
            num_points_added, numNoPriorObs, numUnprojectFail,
            numShortBaseline, numNotFinite, numBehind, numTooClose,
            config.vio_min_triangulation_dist,
            double(maxBaseline > 0 ? minBaseline : Scalar(0)),
            double(maxBaseline), double(num_points_added ? minDepth : Scalar(0)),
            double(sumDepth), double(maxDepth));
        mpLogger->PrintVioTriang();
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
    mpLogger->PrintVioLandmarks();

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
        mpLogger->PrintVioState();
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
    mpLogger->AddVioTiming(opt_flow_meas->t_ns, poseUpdateSeconds,
                           associationSeconds, triangulationSeconds,
                           lostScanSeconds, optMargSeconds, publishSeconds,
                           measureSeconds, imuDrainSeconds, processFrameSoFar,
                           int(this->vision_data_queue.size()),
                           int(this->imu_data_queue.size()));
    mpLogger->PrintVioTiming();

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
    mpLogger->PrintVioMargNullspace();
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

    if (frame_poses.size() > max_kfs || frame_states.size() >= max_states) {
        // Marginalize

        const int states_to_remove = frame_states.size() - max_states + 1;

        auto it = frame_states.cbegin();
        for (int i = 0; i < states_to_remove; i++) it++;
        int64_t last_state_to_marg = it->first;

        AbsOrderMap aom;

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

        {
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
        mpLogger->PrintVioMarg();

        if (config.vio_debug || config.vio_extended_logging) {
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
            if (is_lin_sqrt && marg_data.is_sqrt) {
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

        marg_data.H = marg_H_new;
        marg_data.b = marg_b_new;
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

        VecX delta;
        computeDelta(marg_data.order, delta);
        marg_data.b -= marg_data.H * delta;

        if (config.vio_debug || config.vio_extended_logging) {
            VecX delta;
            computeDelta(marg_data.order, delta);
            nullspace_marg_data.b -= nullspace_marg_data.H * delta;
        }

        const double margSeconds = t_actual_marg.elapsed();
        mpLogger->SolverScratch().add("marg", margSeconds).format("ms");

        if (config.vio_debug || config.vio_extended_logging) {
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
        mpLogger->PrintVioMargTiming();
    }

    mpLogger->SolverScratch().add("marginalize", t_total.elapsed()).format("ms");
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
        mpLogger->SolverScratch().add("num_cams", double(numCams)).format("count");
        mpLogger->SolverScratch().add("num_lms", double(numLms)).format("count");
        mpLogger->SolverScratch().add("num_obs", double(numObs)).format("count");

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
            lqr = LinearizationBase<Scalar, POSE_SIZE>::create(
                this, aom, lqr_options, &marg_data, &ild);
            allocateLmbSeconds = t.reset();
            mpLogger->SolverScratch().add("allocateLMB", allocateLmbSeconds).format("ms");
            lqr->log_problem_stats(mpLogger->SolverScratch());
        }

        bool terminated = false;
        bool converged = false;

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

            mpLogger->AddVioLinearise(last_state_t_ns, it, double(error_total),
                                     double(lambda), int(lmdb.numLandmarks()),
                                     int(frame_states.size()),
                                     int(frame_poses.size()),
                                     numerically_valid);
            mpLogger->PrintVioLinearise();

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
                    computeError(after_update_vision_and_inertial_error);
                    after_update_vision_error =
                        after_update_vision_and_inertial_error;
                    computeMargPriorError(marg_data,
                                          after_update_marg_prior_error);

                    ScBundleAdjustmentBase<Scalar>::computeImuError(
                        aom, after_update_imu_error, after_bg_error,
                        after_ba_error, frame_states, imu_meas,
                        gyro_bias_sqrt_weight.array().square(),
                        accel_bias_sqrt_weight.array().square(), g);

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
                    numCams, numLms, numObs, allocateLmbSeconds,
                    linearizeProblemSeconds, performQrSeconds,
                    getDenseHBSeconds, solveSeconds, backSubstituteSeconds,
                    computeError2Seconds, iteration_time, residentMemory,
                    residentMemoryPeak);
                mpLogger->PrintVioSolverTiming();

                if (step_is_successful) {
                    BASALT_ASSERT(step_is_valid);

                    mpLogger->AddVioSolverIter(
                        last_state_t_ns, it, j, 0, double(after_error_total),
                        double(f_diff), double(l_diff),
                        double(relative_decrease), double(step_norminf),
                        double(after_update_vision_error),
                        double(after_update_imu_error),
                        double(after_bg_error), double(after_ba_error),
                        double(after_update_marg_prior_error), double(lambda),
                        iteration_time, cumulative_time);
                    mpLogger->PrintVioSolverIter();

                    lambda *= std::max<Scalar>(
                        Scalar(1.0) / 3,
                        1 - std::pow<Scalar>(2 * relative_decrease - 1, 3));
                    lambda = std::max(min_lambda, lambda);

                    lambda_vee = initial_vee;

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
                        double(after_update_imu_error),
                        double(after_bg_error), double(after_ba_error),
                        double(after_update_marg_prior_error), double(lambda),
                        iteration_time, cumulative_time);
                    mpLogger->PrintVioSolverIter();

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
        mpLogger->PrintVioSolverSummary();
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

    this->mpKFOutputQueue->push(kf);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::debug_finalize() {
    mpLogger->PrintSummary();
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
