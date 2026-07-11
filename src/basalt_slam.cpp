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
#include <basalt/io/dataset_io.h>
#include <basalt/visualisation/visualiser.h>
#include <tbb/global_control.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <optional>
#include <thread>

// Globals shared with feed thread
size_t max_frames = 0;
std::atomic<bool> terminate = false;

int64_t get_sequence_frame_duration(std::vector<int64_t> timestamps) {
    if (timestamps.size() < 2) return 0;
    int64_t total_duration = 0;
    for (size_t i = 1; i < timestamps.size(); i++) {
        total_duration += timestamps[i] - timestamps[i - 1];
    }
    return total_duration / static_cast<int64_t>(timestamps.size() - 1);
}

void feed_data(std::shared_ptr<basalt::Controller> controller,
               basalt::VioDatasetPtr vio_dataset) {
    std::cout << "Started feed_data thread" << std::endl;

    basalt::VioEstimatorBase<double>::Ptr vio = controller->GetVIO();

    const Eigen::aligned_vector<basalt::GyroData>& gyro_data =
        vio_dataset->get_gyro_data();
    const Eigen::aligned_vector<basalt::AccelData>& accel_data =
        vio_dataset->get_accel_data();
    std::vector<int64_t>& image_timestamps =
        vio_dataset->get_image_timestamps();
    int64_t duration = get_sequence_frame_duration(image_timestamps);

    // Ground-truth poses are sampled at the mocap rate, which rarely coincides
    // with an image timestamp, so resolve the nearest GT sample per frame.
    const std::vector<int64_t>& gt_timestamps =
        vio_dataset->get_gt_timestamps();
    const Eigen::aligned_vector<Sophus::SE3d>& gt_poses =
        vio_dataset->get_gt_pose_data();
    std::function<std::optional<Sophus::SE3d>(int64_t)> gt_pose_for =
        [&](int64_t t_ns) -> std::optional<Sophus::SE3d> {
        if (gt_timestamps.empty()) return std::nullopt;
        std::vector<int64_t>::const_iterator it =
            std::lower_bound(gt_timestamps.begin(), gt_timestamps.end(), t_ns);
        size_t idx = static_cast<size_t>(it - gt_timestamps.begin());
        if (idx == gt_timestamps.size()) idx = gt_timestamps.size() - 1;
        // Prefer the closer of the two neighbouring samples.
        else if (idx > 0 &&
                 (t_ns - gt_timestamps[idx - 1]) < (gt_timestamps[idx] - t_ns))
            idx = idx - 1;
        return gt_poses[idx];
    };

    size_t k = 0;
    std::cout << "number of images found: " << image_timestamps.size()
              << std::endl;

    for (size_t i = 0; i < image_timestamps.size(); i++) {
        if (vio->finished || terminate || (max_frames > 0 && i >= max_frames)) {
            break;
        }

        std::chrono::high_resolution_clock::time_point t1 =
            std::chrono::high_resolution_clock::now();
        basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);
        data->t_ns = image_timestamps[i];
        data->img_data = vio_dataset->get_image_data(data->t_ns);

        // Push all IMU data up to and including the current frame timestamp
        while (k < gyro_data.size() &&
               gyro_data[k].timestamp_ns <= data->t_ns) {
            basalt::ImuData<double>::Ptr imu_data(new basalt::ImuData<double>);
            imu_data->t_ns = gyro_data[k].timestamp_ns;
            imu_data->accel = accel_data[k].data;
            imu_data->gyro = gyro_data[k].data;
            k++;

            controller->GrabIMU(imu_data);
        }

        // Push one extra IMU sample past the frame timestamp (lookahead for
        // VIO preintegration interpolation)
        if (k < gyro_data.size()) {
            basalt::ImuData<double>::Ptr imu_data(new basalt::ImuData<double>);
            imu_data->t_ns = gyro_data[k].timestamp_ns;
            imu_data->accel = accel_data[k].data;
            imu_data->gyro = gyro_data[k].data;
            k++;
            controller->GrabIMU(imu_data);
        }

        // Process frame synchronously (optical flow + VIO on this thread). The
        // ground-truth pose is forwarded only when the GUI is enabled, which
        // the Controller decides internally.
        Sophus::SE3f tcw;
        std::optional<Sophus::SE3d> gtcw = gt_pose_for(image_timestamps[i]);
        controller->TrackMonocular(data, tcw, gtcw);
        std::chrono::high_resolution_clock::time_point t2 =
            std::chrono::high_resolution_clock::now();
        std::chrono::microseconds elapsed =
            std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        // sleep here
        int64_t remaining_us = duration / 1000 - elapsed.count();
        if (remaining_us > 0) {
            // std::cout << "remaining micro seconds: " << remaining_us
            //           << std::endl;
            std::this_thread::sleep_for(
                std::chrono::microseconds(remaining_us));
        }
    }

    std::cout << "Finished feed_data thread" << std::endl;
}

int main(int argc, char** argv) {
    std::string cam_calib_path;
    std::string dataset_path;
    std::string dataset_type;
    std::string config_path;
    int num_threads = 0;
    bool show_gui = false;

    CLI::App app{"Basalt SLAM integration test"};

    app.add_option("--cam-calib", cam_calib_path, "Camera calibration file.")
        ->required();

    app.add_option("--dataset-path", dataset_path, "Path to dataset.")
        ->required();

    app.add_option("--dataset-type", dataset_type, "Dataset type <euroc, bag>.")
        ->required();

    app.add_option("--config-path", config_path, "Path to config file.");
    app.add_option("--num-threads", num_threads, "Number of threads.");
    app.add_option(
        "--max-frames", max_frames,
        "Limit number of frames to process from dataset (0 means unlimited)");
    app.add_option("--show-gui", show_gui, "Show the SLAM GUI");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    // TBB thread control
    std::unique_ptr<tbb::global_control> tbb_global_control;
    if (num_threads > 0) {
        tbb_global_control = std::make_unique<tbb::global_control>(
            tbb::global_control::max_allowed_parallelism, num_threads);
    }

    // ── Controller creation ─────────────────────────────────────────
    auto controller = std::make_shared<basalt::Controller>(
        config_path, cam_calib_path, basalt::SlamMode::VIO);
    controller->load_config();

    // Extract calibration biases for VIO initialization
    basalt::Calibration<double>& calib = controller->GetCalibration();
    Eigen::Matrix<double, 3, 1> bg;
    Eigen::Matrix<double, 3, 3> sg;
    Eigen::Matrix<double, 3, 1> ba;
    Eigen::Matrix<double, 3, 3> sa;
    calib.calib_gyro_bias.getBiasAndScale(bg, sg);
    calib.calib_accel_bias.getBiasAndScale(ba, sa);

    controller->initialize(0,                        // t_ns: start at origin
                           Sophus::SE3d(),           // T_w_i: identity pose
                           Eigen::Vector3d::Zero(),  // vel_w_i: zero velocity
                           bg.cast<double>(),  // gyro bias from calibration
                           ba.cast<double>(),  // accel bias from calibration
                           false,    // useProducerConsumerArchitecture = false
                           show_gui  // enableVisualisation
    );

    // ── Dataset loading ─────────────────────────────────────────────
    basalt::DatasetIoInterfacePtr dataset_io =
        basalt::DatasetIoFactory::getDatasetIo(dataset_type);
    dataset_io->read(dataset_path);
    basalt::VioDatasetPtr vio_dataset = dataset_io->get_data();

    std::cout << "Dataset loaded: "
              << vio_dataset->get_image_timestamps().size() << " images, "
              << vio_dataset->get_gyro_data().size() << " IMU samples"
              << std::endl;

    // ── Data feeding + optional GUI ─────────────────────────────────
    if (show_gui) {
        // Start the visualiser first so its queues are wired before the feeder
        // pushes any frames (no frame is missed). The render loop owns the GL
        // context and must run on this main thread.
        basalt::SlamVisualiser visualiser(*controller);
        visualiser.Start();

        std::thread t1(feed_data, controller, vio_dataset);
        visualiser.Run();  // blocks until the window is closed

        terminate = true;  // stop the feeder early if the user quit first
        t1.join();
        controller->Stop();  // drains VIO + local mapper via their sentinels
        visualiser.Stop();   // releases the visualiser's consumer threads
    } else {
        std::thread t1(feed_data, controller, vio_dataset);
        t1.join();
        controller->Stop();
    }

    // ── Test assertions ─────────────────────────────────────────────
    auto local_mapper = controller->GetLocalMapper();

    if (!local_mapper) {
        std::cerr << "FAIL: LocalMapper is null" << std::endl;
        return 1;
    }

    size_t fp_size = local_mapper->frame_poses.size();
    size_t max_size = local_mapper->mpMaxLocalMapSize;

    std::cout << "LocalMapper frame_poses.size() = " << fp_size << std::endl;
    std::cout << "mpMaxLocalMapSize = " << max_size << std::endl;

    // Assertion 1: The local mapper processed at least some data
    if (fp_size == 0) {
        std::cerr << "FAIL: LocalMapper frame_poses is empty after processing"
                  << std::endl;
        return 1;
    }

    // Assertion 2: The local map respects the size bound
    if (fp_size > max_size) {
        std::cerr << "FAIL: Local map size (" << fp_size
                  << ") exceeds mpMaxLocalMapSize (" << max_size << ")"
                  << std::endl;
        return 1;
    }

    std::cout << "PASS: All assertions passed" << std::endl;
    return 0;
}
