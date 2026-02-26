# System Architecture

## VIO (Visual Inertial Odometry)

The VIO system in Basalt is designed as a pipeline with distinct frontend (Optical Flow) and backend (Estimator) components, decoupled by concurrent queues.

### Overview of Consumer Producer architecture

The basalt package implements both consumer producer architecture and a receive and return based architecture as well. Desired architecture may be set by initialzing the controller with the argument `useProducerConsumerArchitecture` set as true when invoking the function `Controller::initialize()`. Refer to [this](#orchestration) section for more information regarding `class Orchestration` and the high level management of SLAM within basalt. Sample im 

The data flow is as follows:
1.  **Input**: Images and IMU data are read from a dataset (or camera driver).
2.  **Frontend**: Images are fed into the `OpticalFlowBase` implementation.
3.  **Communication**: Tracked feature points (`OpticalFlowResult`) are passed to the backend via a thread-safe queue.
4.  **Backend**: The `VioEstimatorBase` implementation fuses visual data with IMU measurements to estimate state (Pose, Velocity, Biases).
5.  **Output**: State estimates and visualization data are pushed to output queues.

### Sensor Configuration (Monocular vs. Stereo)

The sensor suite configuration (e.g., monocular vs. stereo) is **defined implicitly by the calibration file**. There is no explicit "stereo" flag in `VioConfig`. Please note that only single channel images are currently supported.

*   **Mechanism**: The `basalt::Calibration` struct contains vectors for camera intrinsics and extrinsics. The size of these vectors determines the system mode.
    *   **Monocular**: `intrinsics.size() == 1`
    *   **Stereo**: `intrinsics.size() == 2`
    *   **Multi-camera**: `intrinsics.size() > 2`

*   **Calibration File**: A JSON file (e.g., `euroc_ds_calib.json`) loaded at runtime.
    ```json
    {
      "value0": {
        "T_imu_cam": [ ... ],   // Array of SE3 transforms (size N)
        "intrinsics": [ ... ],  // Array of camera models (size N)
        ...
      }
    }
    ```

*   **Adaptability**:
    *   **Frontend**: `OpticalFlowBase` implementations iterate over `calib.intrinsics.size()` to resize observation vectors and process image pyramids for each camera.
    *   **Backend**: The VIO estimator uses the same size to manage projection models and error terms.

### Key Classes and Interfaces

#### 1. Configuration: `basalt::VioConfig`
*   **Header**: [`include/basalt/utils/vio_config.h`](include/basalt/utils/vio_config.h)
*   **Implementation**: [`src/utils/vio_config.cpp`](src/utils/vio_config.cpp)
*   **Purpose**: Centralizes all configuration parameters for the system.

**Methods:**
*   `void load(const std::string& filename)`: Loads configuration from a JSON file.
*   `void save(const std::string& filename)`: Saves current configuration to a JSON file.

**Attributes:**
| Attribute | Type | Description |
| :--- | :--- | :--- |
| `optical_flow_type` | `std::string` | Type of optical flow ("patch", "frame_to_frame"). |
| `optical_flow_detection_grid_size` | `int` | Grid size for keypoint detection. |
| `optical_flow_max_recovered_dist2` | `float` | Max squared distance for recovered points. |
| `optical_flow_pattern` | `int` | Pattern ID for patch tracking (e.g., 51). |
| `optical_flow_max_iterations` | `int` | Max iterations for tracking optimization. |
| `optical_flow_levels` | `int` | Number of pyramid levels. |
| `optical_flow_epipolar_error` | `float` | Threshold for epipolar constraint check. |
| `optical_flow_skip_frames` | `int` | Number of frames to skip in output (1 = output all). |
| `vio_linearization_type` | `LinearizationType` | `ABS_QR`, `ABS_SC`, or `REL_SC`. |
| `vio_sqrt_marg` | `bool` | Enable square-root marginalization. |
| `vio_max_states` | `int` | Size of the sliding window for temporal states. |
| `vio_max_kfs` | `int` | Max number of keyframes in the optimization window. |
| `vio_min_frames_after_kf` | `int` | Min frames to pass before selecting a new keyframe. |
| `vio_new_kf_keypoints_thresh` | `float` | Threshold (ratio of tracked points) to trigger new KF. |
| `vio_debug` | `bool` | Enable debug printing. |
| `vio_extended_logging` | `bool` | Enable extended logging (marg data). |
| `vio_max_iterations` | `int` | Max iterations for the VIO solver. |
| `vio_obs_std_dev` | `double` | Observation standard deviation. |
| `vio_obs_huber_thresh` | `double` | Huber loss threshold. |
| `vio_min_triangulation_dist` | `double` | Min distance for triangulation. |
| `vio_enforce_realtime` | `bool` | If true, drops frames to maintain real-time speed. |
| *Various Mapper Settings* | ... | Settings prefixed with `mapper_` for the mapping module. |

*To add a new parameter, modify `src/utils/vio_config.cpp` in `VioConfig()` (init) and `serialize()` (binding).*

### Optical Flow Deep Dive

The frontend of the VIO pipeline is handled by the `OpticalFlowBase` class hierarchy. This component is responsible for processing raw images to track sparse feature points over time.

#### 1. Base Class: `basalt::OpticalFlowBase`
*   **Header**: [`include/basalt/optical_flow/optical_flow.h`](include/basalt/optical_flow/optical_flow.h)
*   **Purpose**: Defines the abstract interface for all tracking algorithms and manages data queues.

**Attributes & Usage in `src/vio.cpp`:**
*   `input_queue`: `tbb::concurrent_bounded_queue<OpticalFlowInput::Ptr>`
    *   **Description**: Thread-safe queue receiving raw image data.
    *   **Usage**: In `feed_images()` function, raw images are pushed here.
        ```cpp
        // src/vio.cpp
        basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);
        data->t_ns = ...;
        data->img_data = ...;
        opt_flow_ptr->input_queue.push(data);
        ```
*   `output_queue`: `tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr>*`
    *   **Description**: Pointer to the output queue where results should be pushed. Note that it is a *pointer*, allowing it to be linked directly to the input queue of the next stage (VIO estimator).
    *   **Usage**: Connected in `main()` to the estimator's input queue.
        ```cpp
        // src/vio.cpp
        opt_flow_ptr->output_queue = &vio->vision_data_queue;
        ```
*   `patch_coord`: `Eigen::MatrixXf`
    *   **Description**: Stores the relative coordinates of the patch pattern used for tracking.

#### 2. Data Structures
These structs pass data through the queues.

*   **`basalt::OpticalFlowInput`**
    *   **Header**: [`include/basalt/optical_flow/optical_flow.h`](include/basalt/optical_flow/optical_flow.h)
    *   **Attributes**:
        *   `t_ns` (`int64_t`): Timestamp of the frame(s) in nanoseconds.
        *   `img_data` (`std::vector<ImageData>`): Vector containing image buffers for each camera (stereo/multi-cam).
    *   **Usage**: Instantiated in `feed_images()` in `src/vio.cpp` to wrap dataset images before pushing to the frontend.

*   **`basalt::OpticalFlowResult`**
    *   **Header**: [`include/basalt/optical_flow/optical_flow.h`](include/basalt/optical_flow/optical_flow.h)
    *   **Attributes**:
        *   `t_ns` (`int64_t`): Timestamp.
        *   `observations` (`std::vector<Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>>`): A vector (one per camera) of maps. Each map links a unique `KeypointId` to its 2D location (`AffineCompact2f` includes translation and optional linear deformation).
        *   `pyramid_levels` (`std::vector<std::map<KeypointId, size_t>>`): Stores the pyramid level at which each keypoint was detected (used in multiscale).
        *   `input_images` (`OpticalFlowInput::Ptr`): Pointer to the source images, useful for visualization or debugging.

#### 3. Execution Flow & Pipeline

The feature extraction and tracking pipeline is common across all implementations, driven by the `processingLoop` method running in a dedicated thread.

**Key Pipeline Stages:**

1.  **`processingLoop`**:
    *   Continuously pops `OpticalFlowInput` from the `input_queue`.
    *   Calls `processFrame` for each input.
    *   Pushes a `nullptr` to the `output_queue` when the input stream ends.

2.  **`processFrame(int64_t t_ns, OpticalFlowInput::Ptr& new_img_vec)`**:
    *   **Pyramid Construction**: Builds an image pyramid for the current frame(s). Uses [`basalt::ManagedImagePyr`](include/basalt/image/image_pyr.h).
    *   **Tracking**: Calls `trackPoints` to track existing landmarks from the previous/reference frame to the current frame.
    *   **Output Preparation**: Bundles the tracking results into `OpticalFlowResult`.
    *   **New Keypoints**: Calls `addPoints` to detect features in areas of the image not covered by existing tracks.
    *   **Stereo Filtering**: Calls `filterPoints` to remove outliers using epipolar constraints (if stereo is available).
    *   **Publication**: Pushes the result to `output_queue`.

3.  **`addPoints`**:
    *   **Detection**: Uses `detectKeypoints` to find new corners.
    *   **Grid Search**: Divides the image into a grid (size defined by `optical_flow_detection_grid_size`) to ensure uniform distribution. Only detects points in empty cells.
    *   **Stereo Matching**: If stereo is available, immediately tracks the new points to the second camera to establish 3D capability.

4.  **`filterPoints`**:
    *   **Epipolar Check**: Computes the Fundamental/Essential matrix error between the left and right camera observations.
    *   **Pruning**: Discards points with error exceeding `optical_flow_epipolar_error`.

**External Helper Functions:**

*   **`detectKeypoints`**:
    *   **Header**: [`include/basalt/utils/keypoints.h`](include/basalt/utils/keypoints.h)
    *   **Implementation**: [`src/utils/keypoints.cpp`](src/utils/keypoints.cpp)
    *   **Logic**: Performs corner detection (likely FAST-based or similar) on the image pyramid level. It respects a grid-based occupancy map to avoid clustering.

*   **`computeEssential`**:
    *   **Header**: [`include/basalt/utils/keypoints.h`](include/basalt/utils/keypoints.h)
    *   **Logic**: Computes the Essential matrix from the relative pose between cameras ($T_{i,j}$) to verify stereo matches.

#### 4. Implementations

Basalt provides three concrete tracking strategies, selectable via `VioConfig::optical_flow_type`.

**A. `PatchOpticalFlow`**
*   **Header**: [`include/basalt/optical_flow/patch_optical_flow.h`](include/basalt/optical_flow/patch_optical_flow.h)
*   **Config Name**: `"patch"`
*   **Strategy**: Tracks features by matching patches against the **reference keyframe** where the point was first detected.
*   **Mechanism**:
    *   Stores a permanent copy of the patch from the initial frame.
    *   Tracking performs a forward search (Reference -> Current).
    *   Backward check (Current -> Reference) validates the match.
*   **Pros/Cons**:
    *   (+) Drift-free: patch appearance doesn't accumulate error.
    *   (-) Shorter tracks: fails once the viewpoint changes significantly (perspective distortion/illumination).

**B. `FrameToFrameOpticalFlow`**
*   **Header**: [`include/basalt/optical_flow/frame_to_frame_optical_flow.h`](include/basalt/optical_flow/frame_to_frame_optical_flow.h)
*   **Config Name**: `"frame_to_frame"`
*   **Strategy**: Tracks features consecutively from **Frame T-1 to Frame T**.
*   **Mechanism**:
    *   Uses the patch from the previous frame ($T-1$) to track in the current frame ($T$).
    *   Updates the patch model at every step.
*   **Pros/Cons**:
    *   (+) Longer tracks: adapts to gradual appearance changes.
    *   (-) Drift: small errors accumulate over time.

**C. `MultiscaleFrameToFrameOpticalFlow`**
*   **Header**: [`include/basalt/optical_flow/multiscale_frame_to_frame_optical_flow.h`](include/basalt/optical_flow/multiscale_frame_to_frame_optical_flow.h)
*   **Config Name**: `"multiscale_frame_to_frame"`
*   **Strategy**: Similar to `FrameToFrame` but operates across **multiple pyramid levels**.
*   **Mechanism**:
    *   New keypoints can be detected at higher pyramid levels (coarser scales), not just level 0.
    *   `OpticalFlowResult` stores `pyramid_levels` to track the scale of each point.
    *   Tracking logic respects the specific level of each point during the pyramid search.
*   **Pros/Cons**:
    *   (+) Robustness: Can track larger motions or blurry features at coarser scales.
    *   (+) Efficiency: Points at coarse levels require less computation.

#### 5. Helpers: `OpticalFlowPatch`
*   **Header**: [`include/basalt/optical_flow/patch.h`](include/basalt/optical_flow/patch.h)
*   **Purpose**: Manages the pixel data and math for a single image patch.
*   **Key Attributes**:
    *   `data`: Intensity values of the patch.
    *   `mean`: Mean intensity (for brightness normalization).
    *   `valid`: Boolean flag indicating if the patch is valid (e.g., inside image bounds).
*   **Key Methods**:
    *   `setFromImage()`: Extracts patch data from an image at a given position.
    *   `residual()`: Computes the photometric error between this patch and a target image area.

### Backend: `basalt::VioEstimatorBase`
*   **Header**: [`include/basalt/vi_estimator/vio_estimator.h`](include/basalt/vi_estimator/vio_estimator.h)
*   **Purpose**: Abstract base class for the state estimator.

**Public Interface (Methods):**
*   `initialize(int64_t t_ns, const Sophus::SE3d& T_w_i, ...)`: Initializes the system state (time, pose, velocity, biases).
*   `initialize(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba)`: Initializes only biases.
*   `maybe_join()`: Waits for the processing thread to finish.
*   `drain_input_queues()`: Empties input queues (useful during shutdown).
*   `debug_finalize()`: Helper for final debug operations.
*   `getT_w_i_init()`: Returns the initial pose.
*   `setMaxStates(size_t val)`: Sets the sliding window size for states.
*   `setMaxKfs(size_t val)`: Sets the sliding window size for keyframes.
*   `addIMUToQueue(const ImuData<double>::Ptr& data)`: Adds IMU data to the processing queue.
*   `addVisionToQueue(const OpticalFlowResult::Ptr& data)`: Adds vision data to the processing queue.

**Public Interface (Attributes/Queues):**
*   `vision_data_queue`: `tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr>`
    *   Input queue for tracked features.
*   `imu_data_queue`: `tbb::concurrent_bounded_queue<ImuData<double>::Ptr>`
    *   Input queue for IMU measurements.
*   `out_state_queue`: `tbb::concurrent_bounded_queue<PoseVelBiasState<double>::Ptr>*`
    *   Output queue for estimated states. Set this pointer to receive results.
*   `out_vis_queue`: `tbb::concurrent_bounded_queue<VioVisualizationData::Ptr>*`
    *   Output queue for visualization data. Set this pointer to drive the GUI.
*   `last_processed_t_ns`: `std::atomic<int64_t>`
    *   Timestamp of the last processed frame.
*   `finished`: `std::atomic<bool>`
    *   Flag indicating if processing is complete.

**Concrete Implementations:**
*   **`SqrtKeypointVioEstimator`**: [`include/basalt/vi_estimator/sqrt_keypoint_vio.h`](include/basalt/vi_estimator/sqrt_keypoint_vio.h)
    *   Full VIO implementation.
*   **`SqrtKeypointVoEstimator`**: [`include/basalt/vi_estimator/sqrt_keypoint_vo.h`](include/basalt/vi_estimator/sqrt_keypoint_vo.h)
    *   VO only (no IMU).

#### 1. Visual Odometry (`SqrtKeypointVoEstimator`)
*   **Header**: [`include/basalt/vi_estimator/sqrt_keypoint_vo.h`](include/basalt/vi_estimator/sqrt_keypoint_vo.h)
*   **Implementation**: [`src/vi_estimator/sqrt_keypoint_vo.cpp`](src/vi_estimator/sqrt_keypoint_vo.cpp)
*   **Inheritance**: `VioEstimatorBase` and `SqrtBundleAdjustmentBase`.

**Execution Pipeline (`measure` method):**

1.  **State Initialization**:
    *   **Methods**: `initialize()`, `PoseStateWithLin` (constructor).
    *   **Description**: If not initialized, sets up the first pose and marginalization priors.
    *   **Helper Class**: `PoseStateWithLin` ([`include/basalt/utils/imu_types.h`](include/basalt/utils/imu_types.h)) creates a linearized pose state.

2.  **Pose Prediction**:
    *   **Methods**: `PoseStateWithLin` (constructor).
    *   **Description**: Initializes the current frame's pose with the previous frame's pose (constant position model).

3.  **Landmark Association**:
    *   **Methods**: `lmdb.landmarkExists()`, `lmdb.getLandmark()`, `lmdb.addObservation()`.
    *   **Description**: Iterates through tracked keypoints, associates observations with existing landmarks in `LandmarkDatabase`, and counts connected landmarks.
    *   **External Class**: `LandmarkDatabase` ([`include/basalt/vi_estimator/landmark_database.h`](include/basalt/vi_estimator/landmark_database.h)).

4.  **Single Frame Optimization**:
    *   **Methods**: `BundleAdjustmentBase::optimize_single_frame_pose`.
    *   **Description**: Optimizes *only* the current frame's pose against the existing map of landmarks. This provides a robust initial estimate for the full Bundle Adjustment.
    *   **Base Class**: `BundleAdjustmentBase` ([`include/basalt/vi_estimator/ba_base.h`](include/basalt/vi_estimator/ba_base.h)).

5.  **Keyframe Selection**:
    *   **Logic**: Checks if `connected0 / (connected0 + unconnected_obs0.size()) < vio_new_kf_keypoints_thresh` or if `frames_after_kf > vio_min_frames_after_kf`.

6.  **Triangulation (New Landmarks)**:
    *   **Methods**:
        *   `triangulate()`: Static helper in `BundleAdjustmentBase`.
        *   `calib.intrinsics[].unproject()`: Projects 2D pixels to 3D bearings.
        *   `lmdb.addLandmark()`: Registers the new 3D point.
        *   `StereographicParam::project()`: Handles camera projection logic.
    *   **Description**: If a KF is selected, triangulates new points from `unconnected_obs` using the baseline between the host KF and current frame. Adds valid landmarks to `lmdb`.
    *   **External Files**:
        *   [`include/basalt/vi_estimator/ba_base.h`](include/basalt/vi_estimator/ba_base.h) (triangulate)
        *   [`include/basalt/camera/stereographic_param.hpp`](include/basalt/camera/stereographic_param.hpp) (project)

7.  **Joint Optimization (`optimize`)**:
    *   **Methods**:
        *   `LinearizationBase::create()`: Factory to create the linear system solver.
        *   `lqr->linearizeProblem()`: Computes Jacobians and residuals.
        *   `lqr->performQR()`: Performs marginalization of landmarks (Schur complement) in place.
        *   `lqr->get_dense_H_b()`: Retrieves the reduced camera system.
        *   `lqr->backSubstitute()`: Computes landmark updates after pose updates are found.
    *   **Description**: Performs Square Root Bundle Adjustment (Levenberg-Marquardt) on the sliding window. Optimizes active Keyframes and the current frame.
    *   **External Class**: `LinearizationBase` ([`include/basalt/linearization/linearization_base.hpp`](include/basalt/linearization/linearization_base.hpp)).

8.  **Marginalization (`marginalize`)**:
    *   **Methods**:
        *   `lmdb.removeFrame()`, `lmdb.removeKeyframes()`: Cleans up the landmark database.
        *   `MargHelper::marginalizeHelperSqrtToSqrt()`: Updates the marginalization prior using the current linearization.
    *   **Description**: Removes non-keyframes or old keyframes from the sliding window. Uses Schur complement to update the prior.
    *   **External Files**:
        *   [`include/basalt/vi_estimator/marg_helper.h`](include/basalt/vi_estimator/marg_helper.h) (MargHelper)

#### 2. Visual-Inertial Odometry (`SqrtKeypointVioEstimator`)
*   **Header**: [`include/basalt/vi_estimator/sqrt_keypoint_vio.h`](include/basalt/vi_estimator/sqrt_keypoint_vio.h)
*   **Implementation**: [`src/vi_estimator/sqrt_keypoint_vio.cpp`](src/vi_estimator/sqrt_keypoint_vio.cpp)
*   **Inheritance**: `VioEstimatorBase` and `SqrtBundleAdjustmentBase`.

**Execution Pipeline (`measure` method):**

1.  **State Initialization**:
    *   **Methods**: `initialize()`, `PoseVelBiasStateWithLin` (constructor).
    *   **Description**: If not initialized, sets up the first state (pose + velocity + biases) and marginalization priors.
    *   **Helper Class**: `PoseVelBiasStateWithLin` ([`include/basalt/utils/imu_types.h`](include/basalt/utils/imu_types.h)) creates a linearized state vector containing Pose (SE3), Velocity (R3), and Biases (R6).

2.  **IMU Preintegration & Prediction**:
    *   **Methods**:
        *   `IntegratedImuMeasurement::integrate()`: Accumulates raw IMU readings (accel + gyro) between the previous frame and the current frame.
        *   `IntegratedImuMeasurement::predictState()`: Propagates the previous state forward in time using the preintegrated IMU measurements. This provides a highly accurate initial estimate for the new frame's pose, velocity, and bias.
    *   **Description**: Processes high-rate IMU data to estimate motion. Unlike VO's constant position model, VIO uses inertial physics for prediction.
    *   **External Class**: `IntegratedImuMeasurement` ([`thirdparty/basalt-headers/include/basalt/imu/preintegration.h`](thirdparty/basalt-headers/include/basalt/imu/preintegration.h)).

3.  **Landmark Association**:
    *   **Methods**: `lmdb.landmarkExists()`, `lmdb.addObservation()`.
    *   **Description**: Iterates through tracked keypoints from the Optical Flow result. Associations observations with existing 3D landmarks in the `LandmarkDatabase`. Counts connected landmarks to inform Keyframe selection logic.
    *   **External Class**: `LandmarkDatabase` ([`include/basalt/vi_estimator/landmark_database.h`](include/basalt/vi_estimator/landmark_database.h)).

4.  **Keyframe Selection**:
    *   **Logic**: Decisions are based on the ratio of tracked landmarks versus new detections (`vio_new_kf_keypoints_thresh`) or if a maximum number of frames has passed (`vio_min_frames_after_kf`).

5.  **Triangulation (New Landmarks)**:
    *   **Methods**: `triangulate()`, `calib.intrinsics[].unproject()`, `lmdb.addLandmark()`.
    *   **Description**: If the current frame is selected as a Keyframe, new landmarks are triangulated from observations that were not previously associated. It uses the baseline between the host Keyframe and the current frame to estimate depth. Valid landmarks are added to the database.
    *   **External Files**: [`include/basalt/vi_estimator/ba_base.h`](include/basalt/vi_estimator/ba_base.h).

6.  **Joint Optimization (`optimize`)**:
    *   **Methods**:
        *   `ImuLinData`: Struct populated with the gravity vector, bias weights, and map of `IntegratedImuMeasurement`s.
        *   `LinearizationBase::create(..., &ild)`: Initializes the linearizer with IMU data.
        *   `lqr->linearizeProblem()`: Computes residuals and Jacobians for both visual terms (reprojection error) and inertial terms (preintegration error).
        *   `lqr->performQR()`: Marginalizes landmarks.
        *   `lqr->get_dense_H_b()`: Gets the system for poses/states.
    *   **Description**: Performs a joint optimization of the sliding window states. The cost function includes visual reprojection errors, IMU preintegration errors, and marginalization priors.
    *   **External Class**: `LinearizationBase` ([`include/basalt/linearization/linearization_base.hpp`](include/basalt/linearization/linearization_base.hpp)).

7.  **Marginalization (`marginalize`)**:
    *   **Methods**:
        *   `MargHelper::marginalizeHelperSqrtToSqrt()`: Updates the marginalization prior.
        *   `lmdb.removeKeyframes()`: Removes old Keyframes from the map.
    *   **Description**: Marginalizes out old states to keep the sliding window size bounded. Unlike VO, VIO marginalization involves the full state (Pose, Velocity, Bias) and handles the dense connectivity introduced by IMU factors.
    *   **External Files**: [`include/basalt/vi_estimator/marg_helper.h`](include/basalt/vi_estimator/marg_helper.h).

### Usage Examples

#### 1. Dataset Processing
*   **File**: [`src/vio.cpp`](src/vio.cpp)

The standard usage pattern for offline processing:

1.  **Initialization**:
    ```cpp
    basalt::VioConfig vio_config;
    vio_config.load(config_path);
    
    // Create Frontend
    auto opt_flow_ptr = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    
    // Create Backend
    auto vio = basalt::VioEstimatorFactory::getVioEstimator(
        vio_config, calib, basalt::constants::g, use_imu, use_double);
    ```

2.  **Connecting Queues**:
    ```cpp
    // Link frontend output to backend input
    opt_flow_ptr->output_queue = &vio->vision_data_queue;
    ```

3.  **Data Feeding**:
    *   Feed images to `opt_flow_ptr->input_queue`.
    *   Feed IMU data to `vio->imu_data_queue`.

4.  **Processing**:
    *   Both `opt_flow_ptr` and `vio` run internal threads to process data from their input queues.

5.  **Retrieving Results**:
    *   Consume `vio->out_state_queue` for real-time pose estimates.

#### 2. RealSense Live VIO
*   **File**: [`src/rs_t265_vio.cpp`](src/rs_t265_vio.cpp)

Usage with a live RealSense T265 device is similar but utilizes the `RsT265Device` abstraction:

*   **Device Header**: [`include/basalt/device/rs_t265.h`](include/basalt/device/rs_t265.h)

1.  **Device Initialization**:
    ```cpp
    basalt::RsT265Device::Ptr t265_device;
    t265_device.reset(new basalt::RsT265Device(false, 1, 90, 10.0));
    t265_device->start();
    ```

2.  **Calibration**:
    *   Can load from file or export directly from the device:
        ```cpp
        if (cam_calib_path.empty()) {
          calib = *t265_device->exportCalibration();
        }
        ```

3.  **Direct Queue Linking**:
    *   Instead of manually feeding data loops, pointers to the processing queues are passed directly to the device driver, which pushes data as it arrives.
    ```cpp
    // Link Device Image Output -> Optical Flow Input
    t265_device->image_data_queue = &opt_flow_ptr->input_queue;

    // Link Device IMU Output -> Estimator IMU Input
    t265_device->imu_data_queue = &vio->imu_data_queue;
    ```

## Build System (CMake)

The Basalt build system is modular, using CMake to manage dependencies, compiler flags, and target definitions.

### Key Components

*   **Root `CMakeLists.txt`**:
    *   **Project Setup**: Defines project name `basalt` and version.
    *   **Compiler Flags**: Sets default C++ standard (C++17) and warning levels (`-Wall -Wextra -Werror`. It must be noted that the way the build flags have been provided may cause errors in some downstream CMakeLists.txt files. Refer to the CMakeLists.txt at `ext/basalt/thirdparty/CMakeLists.txt`. An error was observed due to the aforementioned flags being provided as a single string while attempting to build OpenGV).
    *   **Platform Specifics**: Handles macOS vs. Linux differences (e.g., `stdlib` on macOS).
    *   **SIMD**: detection and flags (`-march=native` by default, disabled on Apple Silicon).
    *   **Dependencies**: Finds external packages:
        *   `Eigen3` (Linear Algebra)
        *   `TBB` (Threading)
        *   `OpenCV` (Image processing)
        *   `fmt` (String formatting)
    *   **Options**:
        *   `BASALT_INSTANTIATIONS_DOUBLE`/`FLOAT`: Controls template instantiation for `double` and `float` precision. Disabling one can reduce compile times.
        *   `BASALT_BUILTIN_EIGEN`/`SOPHUS`/`CEREAL`: Controls whether to use vendored submodules or system-installed versions.

*   **Target Definitions**:
    *   **`basalt` (Library)**: The core shared library containing all SLAM logic.
        *   **Sources**: `src/` and `include/basalt/`.
        *   **Includes**: Public headers are exported.
    *   **Executables**:
        *   `basalt_vio`: Offline VIO processing (Reference implementation).
        *   `basalt_mapper`: Mapping tool.
        *   `basalt_calibrate`: Camera calibration tool.
        *   `basalt_kitti_eval`: KITTI evaluation tool.

*   **Subdirectories**:
    *   `thirdparty/`: Contains vendored dependencies (Pangolin, opengv, etc.) and their build logic.
    *   `thirdparty/basalt-headers/`: A header-only library target `basalt::basalt-headers` that exposes core types and serialization.

## Orchestration

This section details how the core components (Frontend and Backend) are instantiated, connected, and executed. The reference implementation for offline dataset processing is found in `src/vio.cpp`.

### Real-time SLAM (`include/basalt/controller.h`)

The `basalt::Controller` class provides a high-level interface to orchestrate the SLAM system, abstracting away the complexities of manual thread management and queue connections. It is designed for integration with external systems like ROS 2.

*   **Header**: [`include/basalt/controller.h`](include/basalt/controller.h)
*   **Implementation**: [`src/controller.cpp`](src/controller.cpp)

#### 1. Purpose
The `Controller` encapsulates the entire VIO/VO pipeline. It manages:
*   Loading of configuration and calibration.
*   Instantiation of the Frontend (`OpticalFlowBase`) and Backend (`VioEstimatorBase`) via factories.
*   Thread-safe data ingestion (Images, IMU).
*   Retrieval of the latest estimated pose.
*   Graceful shutdown and thread lifecycle management.

#### 2. Usage
1.  **Instantiation**:
    ```cpp
    basalt::Controller controller(config_path, calib_path, basalt::SlamMode::VIO);
    controller.load_data();
    controller.initialize(); 
    // Or initialize with a specific prior state:
    // controller.initialize(t_ns, T_w_i, vel_w_i, bg, ba);
    ```

2.  **Data Feeding**:
    External drivers (e.g., ROS 2 nodes) push data into the controller:
    ```cpp
    controller.GrabImage(image_data); // basalt::OpticalFlowInput::Ptr
    controller.GrabImu(imu_data);     // basalt::ImuData<double>::Ptr
    ```

3.  **Result Retrieval**:
    The controller provides thread-safe access to the latest estimated state:
    *   **`GetLatestPose()`**: Returns the most recent `PoseVelBiasState` observed by the internal consumer thread. This method is non-destructive (does not pop from the queue) and is suitable for high-frequency polling.
    *   **`TryPopPose(pose)`**: Explicitly attempts to pop the next pose from the queue. Returns `true` if successful.

#### 3. Limitations (vs. `src/vio.cpp`)
The `Controller` focuses on the core estimation loop and lacks some auxiliary features present in the offline pipeline:
*   **Visualization**: Does not connect to the Pangolin GUI (`out_vis_queue` is unused).
*   **Marginalization Saving**: Does not support saving marginalization data for debugging (`out_marg_queue` is unused).
*   **Thread Pool Configuration**: Does not manage the global TBB thread pool (`tbb::global_control`). The embedding application is responsible for this.

<!-- TODO -->
These features need to be added in basalt SLAM system.

### Recorded Data Pipeline (`src/vio.cpp`)

The system orchestrates the VIO pipeline by spinning up separate threads for data feeding, processing, and result consumption, connected via thread-safe queues.

#### 1. Initialization Phase

1.  **Configuration**:
    *   `VioConfig` ([`src/utils/vio_config.cpp`](src/utils/vio_config.cpp)) and `Calibration` ([`include/basalt/calibration/calibration.hpp`](include/basalt/calibration/calibration.hpp)) are loaded from files.
    *   **Examples**:
        *   EuroC: `data/euroc_config.json`
        *   Kitti: `data/kitti_config.json`
        *   TUM VI: `data/tumvi_512_config.json`

2.  **Dataset**:
    *   `DatasetIoFactory` ([`include/basalt/io/dataset_io.h`](include/basalt/io/dataset_io.h)) creates the appropriate reader (e.g., EuroC, Kitti, Bag) based on the type string.
    *   The returned `DatasetIoInterface` loads the dataset into memory or prepares it for streaming.

3.  **Component Creation**:
    *   **Frontend**: `OpticalFlowFactory::getOpticalFlow` ([`include/basalt/optical_flow/optical_flow.h`](include/basalt/optical_flow/optical_flow.h)) creates the tracker (e.g., `PatchOpticalFlow`).
    *   **Backend**: `VioEstimatorFactory::getVioEstimator` ([`include/basalt/vi_estimator/vio_estimator.h`](include/basalt/vi_estimator/vio_estimator.h)) creates the estimator.

4.  **Wiring**:
    *   The critical step is connecting the frontend's output to the backend's input.
    *   `opt_flow_ptr->output_queue = &vio->vision_data_queue;` (Direct pointer assignment).

5.  **Output Setup**:
    *   Global queues in `src/vio.cpp` (`out_state_queue`, `out_vis_queue`) are linked to the estimator's output pointers to intercept results.

#### 2. Threading Model

The application spawns multiple threads to maximize parallelism while maintaining causal order via queues.

*   **T1: Image Feed (`feed_images`)**:
    *   **Source**: `src/vio.cpp`
    *   Iterates through dataset images, wraps them in `OpticalFlowInput` ([`include/basalt/optical_flow/optical_flow.h`](include/basalt/optical_flow/optical_flow.h)), and pushes to `opt_flow_ptr->input_queue`.
    *   *Control*: Respects `step_by_step` flags using `std::condition_variable` to pause feeding.

*   **T2: IMU Feed (`feed_imu`)**:
    *   **Source**: `src/vio.cpp`
    *   Iterates through IMU messages and pushes `ImuData` to `vio->imu_data_queue`.

*   **Internal Processing Threads**:
    *   **Frontend**: The `OpticalFlowBase` instance launches its own thread consuming `input_queue`.
    *   **Backend**: The `VioEstimatorBase` instance launches its own thread consuming `vision_data_queue` and `imu_data_queue`.
    *   **Note**: The backend must be explicitly initialized. In `src/vio.cpp`, `vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero())` is called to set initial Gyro and Accel biases to zero.

*   **T3: Visualization Consumer**:
    *   **Source**: `src/vio.cpp` (Lambda in `main`)
    *   Pops `VioVisualizationData` from `out_vis_queue` and updates the global `vis_map` for the GUI.

*   **T4: State Consumer**:
    *   **Source**: `src/vio.cpp` (Lambda in `main`)
    *   Pops `PoseVelBiasState` from `out_state_queue`.
    *   Logs data to `vio_data_log` (`pangolin::DataLog`) for real-time plotting.
    *   Records the trajectory (`vio_t_w_i`) for final evaluation (ATE/RMSE).

#### 3. GUI Pipeline (Pangolin)

The visualization is built on the Pangolin library, running in the main thread.

**Logical Blocks:**

1.  **Setup (`main`)**:
    *   **Window**: `pangolin::CreateWindowAndBind` creates the OpenGL context.
    *   **Viewports**:
        *   `main_display`: Container for other views.
        *   `img_view_display`: Displays camera feeds (with overlays).
        *   `plot_display`: Displays real-time data plots.
        *   `display3D`: Renders the 3D scene (camera poses, landmarks).
    *   **Panel**: `pangolin::CreatePanel` creates the side menu with buttons/checkboxes (`pangolin::Var`).

2.  **State Management**:
    *   **`vis_map`**: A `std::unordered_map` mapping timestamps to `VioVisualizationData`. Filled by T3, read by the rendering loop.
    *   **`show_frame`**: A `pangolin::Var<int>` controlling which frame's data is currently visualized.

3.  **Rendering Loop**:
    *   **Frequency**: Runs as fast as possible (`while (!pangolin::ShouldQuit())`).
    *   **Sync**: Checks `GuiChanged()` flags on variables to trigger updates.
    *   **3D Scene**: Calls `draw_scene` to render the trajectory and landmarks using OpenGL.
    *   **2D Overlays**: Calls `draw_image_overlay` to draw tracked features on camera images.
    *   **Plots**: `pangolin::Plotter` automatically visualizes data added to `vio_data_log`.

**Future Visualization Support**:
*   To add new 3D elements, modify `draw_scene`.
*   To add new 2D overlays, modify `draw_image_overlay`.
*   Ensure thread safety: The GUI thread reads `vis_map`, while T3 writes to it. `vis_map` is not protected by a mutex in the example, relying on the fact that `std::unordered_map` insertion is generally safe if keys are unique and iterators aren't invalidated (though strictly speaking, a read/write lock would be safer).

#### 4. Auxiliary Components

Table of key methods and objects used in `src/vio.cpp` for orchestration.

| Name | Type | Source | Description |
| :--- | :--- | :--- | :--- |
| `feed_images` | Function | `src/vio.cpp` | Feeds images from dataset to frontend queue. |
| `feed_imu` | Function | `src/vio.cpp` | Feeds IMU data from dataset to backend queue. |
| `vio_data_log` | Object (`pangolin::DataLog`) | External (Pangolin) | Buffer for real-time plotting of states (Vel, Pos, Biases). |
| `draw_image_overlay` | Function | `src/vio.cpp` | Draws keypoints and tracks on 2D images. |
| `draw_scene` | Function | `src/vio.cpp` | Renders 3D trajectory, cameras, and landmarks. |
| `draw_plots` | Function | `src/vio.cpp` | Configures Pangolin plotter series based on checkboxes. |
| `load_data` | Function | `src/vio.cpp` | Loads camera calibration from JSON. |
| `alignSVD` | Function | `src/utils/system_utils.cpp` | Computes Sim3 alignment between estimated and GT trajectories. |
| `saveTrajectoryButton` | Function | `src/vio.cpp` | GUI callback to save the estimated trajectory to file. |
| `DatasetIoFactory` | Class | `include/basalt/io/dataset_io.h` | Factory for creating dataset readers. |
| `OpticalFlowFactory` | Class | `include/basalt/optical_flow/optical_flow.h` | Factory for creating the frontend. |
| `VioEstimatorFactory` | Class | `include/basalt/vi_estimator/vio_estimator.h` | Factory for creating the backend. |

#### 5. Cleanup & Termination

Proper shutdown is critical to avoid hanging threads, especially given the use of blocking bounded queues. The sequence implemented in `src/vio.cpp` handles both natural completion and user interruption.

**Standard Sequence:**
1.  **Wait for VIO**: `vio->maybe_join()` is called. This blocks the main thread until the VIO estimator's internal processing thread finishes (triggered by receiving `nullptr` in its input queues).
2.  **Unblock Input Threads**: `vio->drain_input_queues()` is called immediately after.
    *   *Reason*: Input threads (`T1`, `T2`) might be blocked trying to push to a full queue. Draining these queues ensures those threads can wake up, exit their loops (checking `vio->finished`), and terminate naturally.
3.  **Join Input Threads**: `t1.join()` and `t2.join()` are called to ensure data feeding has ceased.
4.  **Signal Global Termination**: The global atomic flag `terminate` is set to `true`. This signals auxiliary threads (like the queue printer `T5` or visualization consumers) to stop their loops.
5.  **Join Consumer Threads**: `t3`, `t4`, and `t5` are joined.
6.  **Finalize**: `vio->debug_finalize()` is called to perform any remaining cleanup or stat logging within the estimator.

**GUI Interruption (User Abort):**
If the user closes the Pangolin window (`pangolin::ShouldQuit()` returns true) *before* the dataset is fully processed:
1.  **Detection**: The main rendering loop breaks.
2.  **Abort Signal**: The code checks `if (!vio->finished)`.
3.  **Flag Setting**:
    *   `terminate = true`: Tells input threads (`feed_images`, `feed_imu`) to stop pushing data immediately.
    *   `aborted = true`: Prevents the saving of partial results (trajectory files) after cleanup.
4.  **Queue Draining**: The standard `drain_input_queues()` (in step 2 above) becomes critical here to unblock the input threads so they can check the `terminate` flag and exit.

## Serialization (Cereal)

The project uses the **Cereal** library for both configuration management (JSON) and data storage (Binary/JSON).

### 1. Configuration (JSON)
*   **Usage**: Human-readable configuration files (e.g., `basalt_config.json`).
*   **Mechanism**:
    *   Uses `cereal::JSONInputArchive` and `cereal::JSONOutputArchive`.
    *   The `CEREAL_NVP(member)` macro is essential. It binds the member variable to a named JSON field (Name-Value Pair).
    *   Example: `ar(CEREAL_NVP(config.vio_max_states));` creates `{"vio_max_states": 3}` in the JSON file.

### 2. Data Storage (Binary & JSON)
*   **Usage**: Saving large datasets, calibration caches, or intermediate results (e.g., `.cereal` files).
*   **Mechanism**:
    *   **Binary**: `cereal::BinaryOutputArchive` is used for efficiency when saving large vectors of poses, features, or calibration states (e.g., [`src/io/marg_data_io.cpp`](src/io/marg_data_io.cpp)).
    *   **JSON**: Used for results files (e.g., `results.json`) to store metrics like RMSE or execution time.
*   **Custom Types**:
    *   Basalt extends Cereal to support Eigen types (vectors, matrices) and Sophus Lie groups (`SE3`, `SO3`).
    *   **Headers**:
        *   [`thirdparty/basalt-headers/include/basalt/serialization/eigen_io.h`](thirdparty/basalt-headers/include/basalt/serialization/eigen_io.h)
        *   [`thirdparty/basalt-headers/include/basalt/serialization/headers_serialization.h`](thirdparty/basalt-headers/include/basalt/serialization/headers_serialization.h)

## Core Data Structures

This section serves as a reference for the primary data structures used throughout the Basalt architecture.

### 1. Image Management

*   **`basalt::Image<T>`**
    *   **Header**: [`thirdparty/basalt-headers/include/basalt/image/image.h`](thirdparty/basalt-headers/include/basalt/image/image.h)
    *   **Description**: A lightweight wrapper around a raw data pointer, supporting 2D access, sub-images, and interpolation. It does *not* manage memory.
    *   **Key Methods**: `interp()`, `interpGrad()`, `SubImage()`.

*   **`basalt::ManagedImage<T>`**
    *   **Header**: [`thirdparty/basalt-headers/include/basalt/image/image.h`](thirdparty/basalt-headers/include/basalt/image/image.h)
    *   **Description**: Inherits from `Image<T>` but owns the memory. Uses RAII for allocation and deallocation.
    *   **Usage**: Used for storing image data in the system (e.g., in `ImageData`).

*   **`basalt::OpticalFlowInput`**
    *   **Header**: [`include/basalt/optical_flow/optical_flow.h`](include/basalt/optical_flow/optical_flow.h)
    *   **Description**: Container for raw image data passed to the optical flow module.
    *   **Attributes**:
        *   `t_ns`: Timestamp in nanoseconds.
        *   `img_data`: `std::vector<ImageData>` (wraps `ManagedImage`).

*   **`basalt::OpticalFlowResult`**
    *   **Header**: [`include/basalt/optical_flow/optical_flow.h`](include/basalt/optical_flow/optical_flow.h)
    *   **Description**: Output of the optical flow module, containing tracked features.
    *   **Attributes**:
        *   `observations`: `vector` of maps (KeypointId -> Location).
        *   `input_images`: Pointer to source images.

### 2. IMU & Calibration

*   **`basalt::ImuData<Scalar>`**
    *   **Header**: [`thirdparty/basalt-headers/include/basalt/imu/imu_types.h`](thirdparty/basalt-headers/include/basalt/imu/imu_types.h)
    *   **Description**: Holds a single IMU measurement.
    *   **Attributes**:
        *   `t_ns`: Timestamp.
        *   `accel`: `Eigen::Vector3` accelerometer reading.
        *   `gyro`: `Eigen::Vector3` gyroscope reading.

*   **`basalt::Calibration<Scalar>`**
    *   **Header**: [`thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp`](thirdparty/basalt-headers/include/basalt/calibration/calibration.hpp)
    *   **Description**: Stores system calibration parameters.
    *   **Attributes**:
        *   `T_i_c`: Vector of `SE3` transforms (Camera to IMU).
        *   `intrinsics`: Vector of camera models.
        *   `calib_accel_bias`, `calib_gyro_bias`: Static biases.
        *   `imu_update_rate`: Rate in Hz.
        *   `accel_noise_std`, `gyro_noise_std`: Noise characteristics.

### 3. State Types

*   **`basalt::PoseState<Scalar>`**
    *   **Header**: [`thirdparty/basalt-headers/include/basalt/imu/imu_types.h`](thirdparty/basalt-headers/include/basalt/imu/imu_types.h)
    *   **Description**: Basic state with Timestamp and Pose (`SE3`).

*   **`basalt::PoseVelState<Scalar>`**
    *   **Header**: [`thirdparty/basalt-headers/include/basalt/imu/imu_types.h`](thirdparty/basalt-headers/include/basalt/imu/imu_types.h)
    *   **Description**: Adds Velocity (`Eigen::Vector3`) to `PoseState`.

*   **`basalt::PoseVelBiasState<Scalar>`**
    *   **Header**: [`thirdparty/basalt-headers/include/basalt/imu/imu_types.h`](thirdparty/basalt-headers/include/basalt/imu/imu_types.h)
    *   **Description**: Full state for VIO. Adds Gyro Bias and Accel Bias to `PoseVelState`.
    *   **Key Methods**: `applyInc()` (applies state update), `diff()` (computes difference).

### 4. Optimization Data

*   **`basalt::PoseVelBiasStateWithLin<Scalar>`**
    *   **Header**: [`include/basalt/utils/imu_types.h`](include/basalt/utils/imu_types.h)
    *   **Description**: Augmented state used during optimization. Stores both the linearization point (`state_linearized`) and the current estimate (`state_current`), plus the `delta` update.

*   **`basalt::MargData`**
    *   **Header**: [`include/basalt/utils/imu_types.h`](include/basalt/utils/imu_types.h)
    *   **Description**: Stores marginalization priors.
    *   **Attributes**:
        *   `abs_H`: Marginalized Hessian matrix.
        *   `abs_b`: Marginalized residual vector.
        *   `frame_states`: Linearization states for marginalized frames.

## Helper Macros & Utilities

This section details common macros and utility helpers found in the codebase, primarily defined in `thirdparty/basalt-headers/include/basalt/utils/assert.h` and `include/basalt/utils/format.hpp`.

### Assertions & Error Handling

*   **`BASALT_ASSERT(expr)`**:
    *   **Usage**: `BASALT_ASSERT(x > 0);`
    *   **Behavior**: Checks if `expr` is true. If not, aborts execution. Disabled if `BASALT_DISABLE_ASSERTS` is defined.
*   **`BASALT_ASSERT_MSG(expr, msg)`**:
    *   **Usage**: `BASALT_ASSERT_MSG(ptr != nullptr, "Pointer cannot be null");`
    *   **Behavior**: Same as `BASALT_ASSERT` but prints a custom message on failure.
*   **`BASALT_ASSERT_STREAM(expr, msg)`**:
    *   **Usage**: `BASALT_ASSERT_STREAM(x == y, "Mismatch: " << x << " != " << y);`
    *   **Behavior**: Allows streaming complex messages using `<<`.
*   **`BASALT_LOG_FATAL(msg)`**:
    *   **Usage**: `BASALT_LOG_FATAL("Critical failure");`
    *   **Behavior**: Logs the message and immediately aborts.
*   **`BASALT_LOG_FATAL_STREAM(msg)`**:
    *   **Usage**: `BASALT_LOG_FATAL_STREAM("Failed to load: " << filename);`
    *   **Behavior**: Streaming version of fatal log.

### Miscellaneous

*   **`UNUSED(x)`**:
    *   **Usage**: `UNUSED(var);`
    *   **Purpose**: Suppresses "unused variable" compiler warnings.
*   **`BASALT_LIKELY(x)`**:
    *   **Usage**: `if (BASALT_LIKELY(ptr)) { ... }`
    *   **Purpose**: Optimization hint to the compiler that `x` is expected to be true.
*   **`BASALT_BOUNDS_ASSERT(...)`**:
    *   **Usage**: Inside image accessors.
    *   **Purpose**: Specialized assertion for image bounds checking. Can be toggled independently.
*   **`_format`**:
    *   **Usage**: `"Value: {:.3f}"_format(3.14159)`
    *   **Purpose**: String literal operator for safe, python-style string formatting (wraps `fmt::format`).
