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

#include <basalt/controller.h>
#include <basalt/serialization/headers_serialization.h>
#include <fstream>
#include <iostream>

namespace basalt {

Controller::Controller(const std::string &config_path,
                       const std::string &calib_path, SlamMode mode)
    : config_path_(config_path), calib_path_(calib_path), mode_(mode) {
    // Ideally we'd set queue capacities here if not default
    out_state_queue_.set_capacity(100);
}

Controller::~Controller() {
    terminate_processing_thread_ = true;
    // Push a null pointer to unblock the queue if the processing thread is
    // waiting
    out_state_queue_.push(nullptr);
    if (pose_processing_thread_.joinable()) {
        pose_processing_thread_.join();
    }
}

void Controller::load_config() {
    // Load Config
    if (!config_path_.empty()) {
        vio_config_.load(config_path_);
    } else {
        std::cerr << "Controller: No config path provided. Using defaults."
                  << std::endl;
    }

    // Load Calibration
    std::ifstream os(calib_path_, std::ios::binary);
    if (os.is_open()) {
        cereal::JSONInputArchive archive(os);
        archive(calib_);
        std::cout << "Loaded camera with " << calib_.intrinsics.size()
                  << " cameras" << std::endl;
    } else {
        std::cerr << "Could not load camera calibration " << calib_path_
                  << std::endl;
        std::abort();
    }
}

void Controller::initialize() {
    // Default initialization with zero biases
    initialize(0, Sophus::SE3d(), Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void Controller::initialize(int64_t t_ns, const Sophus::SE3d &T_w_i,
                            const Eigen::Vector3d &vel_w_i,
                            const Eigen::Vector3d &bg,
                            const Eigen::Vector3d &ba,
                            bool useProducerConsumerArchitecture) {
    // 1. Create Optical Flow Frontend
    opt_flow_ptr_ = basalt::OpticalFlowFactory::getOpticalFlow(
        vio_config_, calib_, useProducerConsumerArchitecture);

    // 2. Create VIO/VO Backend
    bool use_imu = (mode_ == SlamMode::VIO);
    // Using default gravity and double precision for now, could be
    // parameterized if needed
    vio_estimator_ = basalt::VioEstimatorFactory::getVioEstimator<double>(
        vio_config_, calib_, basalt::constants::g, use_imu,
        useProducerConsumerArchitecture); // true for use_double

    // 3. Initialize the backend
    if (t_ns == 0 && T_w_i.log().norm() < 1e-9 && vel_w_i.norm() < 1e-9) {
        vio_estimator_->initialize(bg, ba);
    } else {
        vio_estimator_->initialize(t_ns, T_w_i, vel_w_i, bg, ba);
    }

    if (useProducerConsumerArchitecture) {
        // 4. Connect Queues
        // Connect frontend output to backend input
        opt_flow_ptr_->output_queue = &vio_estimator_->vision_data_queue;

        // Connect backend output to our output queue
        vio_estimator_->out_state_queue = &out_state_queue_;

        // Start the pose processing thread
        terminate_processing_thread_ = false;
        pose_processing_thread_ =
            std::thread(&Controller::process_pose_queue_loop, this);
    }
}

void Controller::TrackMonocular(OpticalFlowInput::Ptr &frame,
                                Sophus::SE3f &tcw) {
    OpticalFlowResult::Ptr res =
        opt_flow_ptr_->processFrame(frame->t_ns, frame);
    current_latest_pose_ = vio_estimator_->ProcessFrame(res);
    tcw = current_latest_pose_->T_w_i.cast<float>();
}

void Controller::GrabImage(basalt::OpticalFlowInput::Ptr data) {
    if (opt_flow_ptr_) {
        opt_flow_ptr_->input_queue.push(data);
    }
}

void Controller::GrabIMU(basalt::ImuData<double>::Ptr data) {
    if (vio_estimator_) {
        vio_estimator_->imu_data_queue.push(data);
    }
}

basalt::PoseVelBiasState<double>::Ptr Controller::GetLatestPose() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return current_latest_pose_;
}

bool Controller::TryPopPose(basalt::PoseVelBiasState<double>::Ptr &pose) {
    return out_state_queue_.try_pop(pose);
}

void Controller::process_pose_queue_loop() {
    // TODO Will Need to check implications on latency
    basalt::PoseVelBiasState<double>::Ptr pose;
    while (!terminate_processing_thread_ || !out_state_queue_.empty()) {
        if (out_state_queue_.try_pop(pose)) {
            if (pose) { // Check for nullptr indicating end of stream or
                        // shutdown
                std::lock_guard<std::mutex> lock(pose_mutex_);
                current_latest_pose_ = pose;
            } else {
                // nullptr received, means end of stream or shutdown signal
                break;
            }
        } else {
            // Queue empty, wait a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

} // namespace basalt
