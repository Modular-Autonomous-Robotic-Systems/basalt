#pragma once

#include <basalt/calibration/calibration.hpp>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <mutex>
#include <string>

namespace basalt {

enum class SlamMode { VO, VIO };

class Controller {
  public:
    Controller(const std::string &config_path, const std::string &calib_path,
               SlamMode mode);

    ~Controller();

    void load_config();

    // Initialize at the start (default biases)
    void initialize();

    // Initialize mid-flight with specific state
    void initialize(int64_t t_ns, const Sophus::SE3d &T_w_i,
                    const Eigen::Vector3d &vel_w_i, const Eigen::Vector3d &bg,
                    const Eigen::Vector3d &ba,
                    bool useProducerConsumerArchitecture = false);

    void GrabImage(basalt::OpticalFlowInput::Ptr data);
    void GrabIMU(basalt::ImuData<double>::Ptr data);

    // Helper to retrieve the latest pose estimate (does not pop from queue)
    basalt::PoseVelBiasState<double>::Ptr GetLatestPose() const;

    // Explicitly pop a pose reading from the queue
    bool TryPopPose(basalt::PoseVelBiasState<double>::Ptr &pose);

    void TrackMonocular(OpticalFlowInput::Ptr &frame, Sophus::SE3f &tcw);

  private:
    // configuration
    std::string config_path_;
    std::string calib_path_;
    SlamMode mode_;

    basalt::VioConfig vio_config_;
    basalt::Calibration<double> calib_;

    basalt::OpticalFlowBase::Ptr opt_flow_ptr_;
    basalt::VioEstimatorBase<double>::Ptr vio_estimator_;

    tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr>
        out_state_queue_;

    // Member for latest pose, updated by an internal thread
    basalt::PoseVelBiasState<double>::Ptr current_latest_pose_;
    mutable std::mutex pose_mutex_;
    std::thread pose_processing_thread_;
    std::atomic<bool> terminate_processing_thread_ = false;

    void process_pose_queue_loop();
};

} // namespace basalt
