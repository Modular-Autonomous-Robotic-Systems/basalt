# System Architecture

## VIO (Visual Inertial Odometry)

The VIO system in Basalt is designed as a pipeline with distinct frontend (Optical Flow) and backend (Estimator) components, decoupled by concurrent queues.

### Overview

The data flow is as follows:
1.  **Input**: Images and IMU data are read from a dataset (or camera driver).
2.  **Frontend**: Images are fed into the `OpticalFlowBase` implementation.
3.  **Communication**: Tracked feature points (`OpticalFlowResult`) are passed to the backend via a thread-safe queue.
4.  **Backend**: The `VioEstimatorBase` implementation fuses visual data with IMU measurements to estimate state (Pose, Velocity, Biases).
5.  **Output**: State estimates and visualization data are pushed to output queues.

### Sensor Configuration (Monocular vs. Stereo)

The sensor suite configuration (e.g., monocular vs. stereo) is **defined implicitly by the calibration file**. There is no explicit "stereo" flag in `VioConfig`.

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
