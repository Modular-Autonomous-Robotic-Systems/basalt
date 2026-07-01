# Visual-Inertial Odometry: Linearisation and Marginalisation

## 1. Introduction

For long-term operations in graph-based Simultaneous Localization and Mapping (SLAM) and Visual-Inertial Odometry (VIO), the computational complexity of the estimation problem grows unbounded as new states and measurements are added over time. To maintain a bounded computational cost while operating in real-time, systems typically rely on a fixed-lag smoothing (or sliding window) approach. Instead of optimizing the entire trajectory, only a recent window of states is actively optimized.

However, simply discarding old states and measurements would result in a loss of valuable information, leading to drift. To preserve the information encapsulated in older observations without keeping the variables themselves in the active state vector, the older states are mathematically eliminated through a process called **marginalisation**. The resulting marginalized distribution acts as a prior for the active optimization window. 

This document explores the mathematical formulation and implementation of problem linearisation and marginalisation within the `basalt` VIO framework. The concepts are grounded in the theoretical background of nonlinear factor recovery for long-term SLAM and visual-inertial mapping.

---

## 2. Linearisation

The state estimation is formulated as a nonlinear least squares problem defined on a factor graph. The goal is to find the state increment $\xi$ that minimizes the objective function $E(\mathbf{s})$, which is the sum of squared Mahalanobis distances of all residuals (visual, inertial, and marginalisation prior):

$$ E(\mathbf{s}) = \frac{1}{2} \mathbf{r}(\mathbf{s})^T \mathbf{W} \mathbf{r}(\mathbf{s}) $$

where $\mathbf{W} = \Sigma^{-1}$ is the block-diagonal weight (information) matrix.

### Gauss-Newton Approximation
Because the residual function $\mathbf{r}(\mathbf{s})$ is nonlinear, we linearize it around the current state estimate $\mathbf{s}$ using a first-order Taylor expansion:
$$ \mathbf{r}(\mathbf{s} \oplus \xi) \approx \mathbf{r}(\mathbf{s}) + \mathbf{J} \xi $$
where $\xi$ is the state increment, $\mathbf{J} = \frac{\partial \mathbf{r}}{\partial \xi} |_{\mathbf{s}}$ is the Jacobian matrix evaluated at $\mathbf{s}$, and $\oplus$ is the update operator on the manifold.

Substituting this linearized residual into the energy function yields a quadratic approximation of the objective:
$$ E(\mathbf{s} \oplus \xi) \approx \frac{1}{2} ( \mathbf{r}(\mathbf{s}) + \mathbf{J} \xi )^T \mathbf{W} ( \mathbf{r}(\mathbf{s}) + \mathbf{J} \xi ) $$

Expanding this expression (and using the fact that $\mathbf{r}^T \mathbf{W} \mathbf{J} \xi = \xi^T \mathbf{J}^T \mathbf{W} \mathbf{r}$ since it is a scalar), we obtain:
$$ E(\mathbf{s} \oplus \xi) \approx \frac{1}{2} \mathbf{r}(\mathbf{s})^T \mathbf{W} \mathbf{r}(\mathbf{s}) + \xi^T \mathbf{J}^T \mathbf{W} \mathbf{r}(\mathbf{s}) + \frac{1}{2} \xi^T \mathbf{J}^T \mathbf{W} \mathbf{J} \xi $$
$$ E(\mathbf{s} \oplus \xi) \approx E(\mathbf{s}) + \xi^T \mathbf{J}^T \mathbf{W} \mathbf{r}(\mathbf{s}) + \frac{1}{2} \xi^T \mathbf{J}^T \mathbf{W} \mathbf{J} \xi $$

To find the minimum increment $\xi^*$ that minimizes the error, we take the partial derivative of this quadratic objective with respect to $\xi$ and set it to zero:
$$ \frac{\partial E(\mathbf{s} \oplus \xi)}{\partial \xi} = \mathbf{J}^T \mathbf{W} \mathbf{r}(\mathbf{s}) + \mathbf{J}^T \mathbf{W} \mathbf{J} \xi = \mathbf{0} $$

Rearranging this equation yields the standard normal equations:
$$ (\mathbf{J}^T \mathbf{W} \mathbf{J}) \xi = - \mathbf{J}^T \mathbf{W} \mathbf{r}(\mathbf{s}) $$

We concisely define these terms as the Information Matrix $\mathbf{H}$ and the Information Vector $\mathbf{b}$:
$$ \mathbf{H} = \mathbf{J}^T \mathbf{W} \mathbf{J} $$
$$ \mathbf{b} = - \mathbf{J}^T \mathbf{W} \mathbf{r}(\mathbf{s}) $$

Substituting these into the normal equations gives the final linear system:
$$ \mathbf{H} \xi = \mathbf{b} $$

### The Information Matrix ($\mathbf{H}$) and Information Vector ($\mathbf{b}$)
*   **Information Matrix ($\mathbf{H}$)**: The matrix $\mathbf{H} = \mathbf{J}^T \mathbf{W} \mathbf{J}$ is the Gauss-Newton approximation of the Hessian matrix (also related to the Fisher Information Matrix). It is a sparse, symmetric, positive semi-definite matrix that encapsulates the certainty and structural correlation of the state variables. Each block element $\mathbf{H}_{ij}$ describes how strongly variable $i$ is coupled with variable $j$ given the measurements.
*   **Information Vector ($\mathbf{b}$)**: The right-hand side vector $\mathbf{b} = - \mathbf{J}^T \mathbf{W} \mathbf{r}(\mathbf{s})$ is the gradient-related vector that drives the optimization. It represents the negative gradient of the objective function at the current linearization point. It aggregates the weighted residual errors projected into the state space via the Jacobian, effectively "pulling" the state update $\xi$ in the direction that reduces the error. When the system converges and residuals are minimized, $\mathbf{b}$ approaches $\mathbf{0}$. 

<!-- **First-Estimate Jacobians (FEJ)**: -->
<!-- In the `basalt` implementation, as variables are moved into the marginalization prior, their linearization point is permanently fixed. This approach, known as First-Estimate Jacobians, ensures that the nullspace properties of the unobservable directions (like absolute position and yaw) are preserved, avoiding spurious information gain. -->

---

## 3. VIO Frame to Frame Tracking

The visual-inertial odometry pipeline continuously processes incoming camera images and high-frequency Inertial Measurement Unit (IMU) data to estimate the sensor's ego-motion.

### Problem Formulation
The state estimation is cast as a fixed-lag smoothing (or sliding window) non-linear least squares optimization problem. A factor graph is formed where nodes represent state variables and edges represent measurement constraints (factors). 

**Variables:**
*   **Optimized Variables (Active State, $\mathbf{s}$):**
    *   $\mathbf{s}_k$: Camera poses $\mathbf{T}_k \in SE(3)$ for a set of older, selected keyframes.
    *   $\mathbf{s}_f$: Full navigation states for the most recent frames in the sliding window. Each state includes the camera pose $\mathbf{T}_i \in SE(3)$, linear velocity $\mathbf{v}_i \in \mathbb{R}^3$, and IMU biases $\mathbf{b}_i = [\mathbf{b}_i^a, \mathbf{b}_i^g]^T \in \mathbb{R}^6$.
    *   $\mathbf{s}_l$: Landmark parameters $\mathbf{l}_j$ (parameterized by a 2D stereographic projection direction and an inverse distance relative to their host frame).
*   **Non-Optimized Variables (Fixed/Prior Context):**
    *   Marginalized states that are no longer actively optimized but influence the current estimate through the marginalization prior $E_{marg}(\mathbf{s})$.
    *   Fixed calibration parameters (e.g., camera intrinsics, camera-IMU extrinsics).
*   **Configuration Parameters:**
    *   Gravity vector $\mathbf{g}$, noise covariances for IMU ($\Sigma_{a}, \Sigma_{g}, \Sigma_{ba}, \Sigma_{bg}$) and vision ($\Sigma_{v}$), and threshold heuristics for keyframe selection and marginalization.

### Residual Errors
To find the optimal state $\mathbf{s}^*$, we minimize the total objective function $E(\mathbf{s})$, which aggregates the residual errors from visual observations, IMU preintegration, and the marginalization prior. 

**Total Objective Function:**
$$ E(\mathbf{s}) = \sum_{(i,j) \in \mathcal{V}} \mathbf{r}_{ij}^T \Sigma_{ij}^{-1} \mathbf{r}_{ij} + \sum_{k \in \mathcal{I}} \mathbf{r}_k^T \Sigma_k^{-1} \mathbf{r}_k + E_{\mathbf{marg}}(\mathbf{s}) $$
Where:
*   $\mathcal{V}$ is the set of all valid visual observations (feature $j$ observed in frame $i$).
*   $\mathcal{I}$ is the set of IMU preintegrated measurements connecting consecutive frames.
*   $E_{marg}(\mathbf{s})$ is the quadratic prior derived from previously marginalized states.

#### Reprojection Error
Visual tracking operates by matching sparse features between frames. In `basalt`, patch-based KLT Optical Flow tracking is used. For a point $j$ hosted in frame $h(j)$ and observed in target frame $i$ at coordinates $\mathbf{z}_{ij}$, the reprojection residual $\mathbf{r}_{ij}$ is:
$$ \mathbf{r}_{ij} = \mathbf{z}_{ij} - \pi ( \mathbf{T}_{i}^{-1} \mathbf{T}_{h(j)} \mathbf{q}_j(u, v, d) ) $$
where $\mathbf{T}_{i}$ and $\mathbf{T}_{h(j)}$ are the poses of the target and host frames, $\mathbf{q}_j(u, v, d)$ is the 3D landmark reconstructed from its minimal parameters, and $\pi(\cdot)$ is the camera projection model. $\Sigma_{ij}$ is the covariance of the visual measurement.

#### IMU Preintegration Error
Due to the high frequency of IMU measurements, multiple readings between consecutive frames $k$ and $k+1$ are preintegrated into a single pseudo-measurement $\Delta \mathbf{s} = (\Delta \mathbf{R}, \Delta \mathbf{v}, \Delta \mathbf{p})$. The IMU residuals $\mathbf{r}_k$ are defined as:
$$ \mathbf{r}_{\Delta R} = \text{Log}( (\Delta \mathbf{R} \hat{\mathbf{R}})^T \mathbf{R}_k^T \mathbf{R}_{k+1} ) $$
$$ \mathbf{r}_{\Delta v} = \mathbf{R}_k^T ( \mathbf{v}_{k+1} - \mathbf{v}_k - \mathbf{g}\Delta t ) - \Delta \hat{\mathbf{v}} $$
$$ \mathbf{r}_{\Delta p} = \mathbf{R}_k^T ( \mathbf{p}_{k+1} - \mathbf{p}_k - \frac{1}{2}\mathbf{g}\Delta t^2 ) - \Delta \hat{\mathbf{p}} $$
where $\mathbf{R}$ and $\mathbf{p}$ represent rotation and translation of the pose $\mathbf{T}$, $\mathbf{v}$ is velocity, and $\mathbf{g}$ is gravity. Changes in biases $\mathbf{b}^{a}$ and $\mathbf{b}^{g}$ during the preintegration interval are accounted for using a first-order linear approximation. $\Sigma_k$ is the preintegrated noise covariance.

#### Marginalisation Residual
When variables (e.g., old keyframes, velocities, biases) are removed from the active state to cap computational complexity, the constraints connected to them must not be simply discarded. As discussed in [1] and [2], discarding these variables would lead to a significant loss of information and rapid accumulation of drift. Instead, the information from the marginalized variables is compressed into a dense quadratic prior on the remaining coupled variables (the Markov blanket). This prior is parameterized by an information matrix $\mathbf{H}^*$ and an information vector $\mathbf{b}^*$, derived analytically by performing the Schur complement on the partitioned linearized system (as detailed in Section 4.2).

For the non-linear least squares optimization, this newly constructed prior must be incorporated into the total objective function. This yields a marginalization penalty acting on the active state increment $\xi$:
$$ E_{\mathbf{marg}}(\xi) = \mathbf{b}^{*T} \xi + \frac{1}{2} \xi^T \mathbf{H}^* \xi $$

To optimize this objective alongside standard visual and inertial residuals using Gauss-Newton or Levenberg-Marquardt solvers, the marginalization energy is conventionally formulated as a sum of squared residuals. The relationship between the energy $E_{\mathbf{marg}}(\xi)$ and its equivalent squared residual $\mathbf{r}_{\mathbf{marg}}(\xi)$ is given by $E_{\mathbf{marg}}(\xi) = \frac{1}{2} \|\mathbf{r}_{\mathbf{marg}}(\xi)\|^2 + C$. To find this residual, the information matrix $\mathbf{H}^*$ is decomposed using a square-root decomposition (such as Cholesky or Eigenvalue decomposition) such that $\mathbf{J}_{\mathbf{marg}}^T \mathbf{J}_{\mathbf{marg}} = \mathbf{H}^*$. The corresponding residual vector is then defined as:
$$ \mathbf{r}_{\mathbf{marg}}(\xi) = \mathbf{J}_{\mathbf{marg}} \xi + \mathbf{r}_{\mathbf{marg}, 0} $$
By expanding the squared norm:
$ \frac{1}{2} (\|\mathbf{J}_{\mathbf{marg}} \xi + \mathbf{r}_{\mathbf{marg}, 0}\|)^2 = \frac{1}{2} \xi^T \mathbf{J}_{\mathbf{marg}}^T \mathbf{J}_{\mathbf{marg}} \xi + \mathbf{r}_{\mathbf{marg}, 0}^T \mathbf{J}_{\mathbf{marg}} \xi + \frac{1}{2} \mathbf{r}_{\mathbf{marg}, 0}^T \mathbf{r}_{\mathbf{marg}, 0} $
, we can match the linear terms with the marginalization energy $E_{\mathbf{marg}}(\xi)$ to find the constant offset vector $\mathbf{r}_{\mathbf{marg}, 0}$. It must satisfy $\mathbf{J}_{\mathbf{marg}}^T \mathbf{r}_{\mathbf{marg}, 0} = \mathbf{b}^*$, which yields the formulation derivation $\mathbf{r}_{\mathbf{marg}, 0} = (\mathbf{J}_{\mathbf{marg}}^T)^{+} \mathbf{b}^*$. Minimizing the squared norm of $\mathbf{r}_{\mathbf{marg}}(\xi)$ perfectly reproduces the behavior of minimizing $E_{\mathbf{marg}}(\xi)$.

The inclusion of the marginalization residual is vital because it anchors the actively optimized states to the historically accumulated visual-inertial constraints, significantly improving both tracking robustness and accuracy. A critical mathematical consideration when adding this residual is the proper handling of unobservable state directions, namely global position and yaw around the gravity vector. The true VIO system has a nullspace corresponding to these dimensions, meaning the system possesses no absolute information about them. When constructing $\mathbf{H}^*$, its nullspace should perfectly align with these unobservable directions. However, if the active variables were to be relinearized around new estimates in subsequent optimization steps, the Jacobians would change. This inconsistency would alter the nullspace of the marginalization prior relative to the current state, causing the non-linear solver to artificially introduce non-zero information (spurious information gain) along the global position and yaw dimensions. To prevent this, the system enforces the First-Estimate Jacobians (FEJ) approach. As soon as a variable becomes part of the marginalization prior, its linearization point $\mathbf{s}_0$ is permanently fixed for all future evaluations of its associated Jacobians. By freezing the linearization point, the mathematical structure of $\mathbf{H}^*$ is preserved, ensuring that the marginalization residual only penalizes deviations in the observable subspace, maintaining global consistency and preventing long-term drift.

---

## 4. Marginalisation

Marginalisation allows the system to bound the size of the optimization problem (the number of variables in $\mathbf{H}$) by analytically eliminating old variables.

### 4.1 Keyframe Selection Logic
The decision to spawn a new keyframe or marginalise an old one defines the structure of the factor graph.

**Spawning a Keyframe:** 
In `SqrtKeypointVioEstimator::measure`, a new keyframe is triggered if the ratio of visually tracked landmarks falls below a threshold (`config.vio_new_kf_keypoints_thresh`):
```cpp
if (Scalar(connected0) / (connected0 + unconnected_obs0.size()) < Scalar(config.vio_new_kf_keypoints_thresh) && frames_after_kf > config.vio_min_frames_after_kf)
    take_kf = true;
```

**Dropping a Keyframe:** 
In `SqrtKeypointVioEstimator::marginalize`, if the active window exceeds its capacity (`frame_poses.size() > max_kfs` or `frame_states.size() >= max_states`), older frames must be removed. 

The strategy for selecting a keyframe to drop (excluding the two most recent ones) operates in two modes:

1. **Feature Tracking Ratio:**
   Starting from the oldest keyframe, if any keyframe has less than a small percentage of its landmarks tracked by the current frame (defined by `config.vio_kf_marg_feature_ratio`), it is immediately selected for marginalisation. This removes keyframes that no longer contribute useful visual constraints.
   ```cpp
   if (num_points_connected.count(*it) == 0 ||
       (num_points_connected.at(*it) / static_cast<float>(num_points_kf.at(*it)) < config.vio_kf_marg_feature_ratio)) {
       id_to_marg = *it;
       break;
   }
   ```

2. **Distance-Based Score:**
   If no keyframe satisfies the first condition, a score is computed based on spatial distribution to find the most redundant one:
*   Higher score for keyframes that are close to many other keyframes (high redundancy).
*   Lower score for keyframes that are close to the latest keyframe (to maintain tracking stability).

    ```cpp
    // small distance to other keyframes --> higher score
    Scalar denom = 0;
    for (auto it2 = kf_ids.begin(); it2 != end_minus_2; ++it2) {
        denom += 1 / ((frame_poses.at(*it1).getPose().translation() - frame_poses.at(*it2).getPose().translation()).norm() + Scalar(1e-5));
    }
    // small distance to latest kf --> lower score
    Scalar score = std::sqrt((frame_poses.at(*it1).getPose().translation() - frame_states.at(last_kf).getState().T_w_i.translation()).norm()) * denom;
    ```
The keyframe with the lowest score (most redundant) is selected as `id_to_marg` and scheduled for marginalisation.

### 4.2 Schur's Complement (Mathematical Formulation)

Given the linearized system $\mathbf{H} \xi = \mathbf{b}$, we want to partition the state $\xi$ into a set of variables to keep, $\xi_{\alpha}$, and a set of variables to marginalize out, $\xi_\beta$. 

We can rearrange the normal equations into a block structure representing the Markov blanket of the variables to be removed:
$$ \begin{bmatrix} \mathbf{H}_{\alpha\alpha} & \mathbf{H}_{\alpha\beta} \\ \mathbf{H}_{\beta\alpha} & \mathbf{H}_{\beta\beta} \end{bmatrix} \begin{bmatrix} \xi_\alpha \\ \xi_\beta \end{bmatrix} = \begin{bmatrix} \mathbf{b}_\alpha \\ \mathbf{b}_\beta \end{bmatrix} $$

From the bottom row, we can solve for $\xi_\beta$:
$$ \mathbf{H}_{\beta\alpha} \xi_\alpha + \mathbf{H}_{\beta\beta} \xi_\beta = \mathbf{b}_\beta $$
$$ \xi_\beta = \mathbf{H}_{\beta\beta}^{-1}(\mathbf{b}_\beta - \mathbf{H}_{\beta\alpha} \xi_\alpha) $$

Substituting $\xi_\beta$ into the top row yields the marginalized system:
$$ ( \mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{H}_{\beta\alpha} ) \xi_\alpha = \mathbf{b}_\alpha - \mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{b}_\beta $$

This defines the new dense prior information matrix $\mathbf{H}^*$ and information vector $\mathbf{b}^*$:
$$ \mathbf{H}^{*} = \mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{H}_{\beta\alpha} $$
$$ \mathbf{b}^* = \mathbf{b}_\alpha - \mathbf{H}_{\alpha\beta} \mathbf{H}_{\beta\beta}^{-1} \mathbf{b}_\beta $$

The term $\mathbf{H}_{\alpha\alpha} - \mathbf{H}_{\alpha\beta}\mathbf{H}_{\beta\beta}^{-1}\mathbf{H}_{\beta\alpha}$ is known as the **Schur complement** of block $\mathbf{H}_{\beta\beta}$.

### 5.3 Implementation in Code

In `basalt`, marginalisation is systematically orchestrated to cleanly transition the mathematical theory into efficient matrix operations.

#### 5.3.1 Key Classes and Interfaces

**1. `AbsOrderMap`**
*   **Header Path:** `include/basalt/utils/imu_types.h`
*   **Source Path:** N/A (Struct defined in header)
*   **Purpose:** A mapping structure that dictates the block layout of the $\mathbf{H}$ matrix. It acts as a layout dictionary, allowing the dynamic sliding window to assemble a single monolithic dense matrix for mathematical elimination. It ensures that every variable factor (pose, velocity, bias) knows its exact row/column location.
*   **Inheritance:** None.
*   **Variables:**
    *   `std::map<int64_t, std::pair<int, int>> abs_order_map`: Maps state timestamps to a pair of `(start_index, block_size)`.
    *   `size_t items`: Number of map elements.
    *   `size_t total_size`: Total size of the active states vector mapped.
*   **Methods:**
    *   `void print_order()`: Utility to print the layout.

**2. `MargLinData<Scalar>`**
*   **Header Path:** `include/basalt/utils/imu_types.h`
*   **Source Path:** N/A (Template struct defined in header)
*   **Purpose:** A container for the linearized prior obtained from marginalization. It stores the prior information matrix `H` (or Jacobian) and the right-hand side vector `b`.
*   **Inheritance:** None.
*   **Variables:**
    *   `bool is_sqrt`: Flag indicating if the square-root mathematical formulation ($\mathbf{J}^T \mathbf{J}$) is used.
    *   `AbsOrderMap order`: The layout of the variables in the prior.
    *   `Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> H`: The prior Information Matrix or Jacobian.
    *   `Eigen::Matrix<Scalar, Eigen::Dynamic, 1> b`: The prior Information Vector or Residual.

**3. `MargData`**
*   **Header Path:** `include/basalt/utils/imu_types.h`
*   **Source Path:** N/A (Struct defined in header)
*   **Purpose:** The global Non-Linear Factor Recovery (NFR) payload. It packages up the full, un-eliminated dense sub-problem and pushes it to an output queue for global mapping.
*   **Inheritance:** None.
*   **Variables:**
    *   `AbsOrderMap aom`: The full pre-marginalisation variable layout.
    *   `Eigen::MatrixXd abs_H`: The full dense matrix (Jacobian or Hessian) before Schur complement.
    *   `Eigen::VectorXd abs_b`: The full residual vector before Schur complement.
    *   `Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<double>> frame_states`: The states actively included in the dense matrix.
    *   `Eigen::aligned_map<int64_t, PoseStateWithLin<double>> frame_poses`: Fixed linearisation points (pure poses) included in the dense matrix.
    *   `std::set<int64_t> kfs_all`: Set of all keyframes in the active window at the time of marginalisation.
    *   `std::set<int64_t> kfs_to_marg`: The lists of which keyframes are being marginalised.
    *   `bool use_imu`: Flag to indicate whether IMU residuals were included in the system.
    *   `std::vector<OpticalFlowResult::Ptr> opt_flow_res`: Optical flow tracking results associated with the keyframes.

**4. `LinearizationBase<Scalar, POSE_SIZE>`**
*   **Header Path:** `include/basalt/linearization/linearization_base.hpp`
*   **Source Path:** `src/linearization/linearization_base.cpp` (and derived headers/cpps)
*   **Purpose:** An abstract base class defining the interface for linearizing the VIO problem. It evaluates residuals, computes Jacobians $\mathbf{J}$, and aggregates the dense system matrices ($\mathbf{H}$, $\mathbf{b}$) for a given set of frames and observations.
*   **Inheritance Tree:** 
    *   `LinearizationBase` (Abstract Base)
        *   $\hookrightarrow$ `LinearizationAbsSC` (Absolute Pose parameterization, Schur Complement point marginalization)
        *   $\hookrightarrow$ `LinearizationRelSC` (Relative Pose parameterization, Schur Complement point marginalization)
        *   $\hookrightarrow$ `LinearizationAbsQR` (Absolute Pose parameterization, QR point marginalization)
*   **Key Methods:**
    *   `static std::unique_ptr<LinearizationBase> create(...)`: Factory method to instantiate a concrete derived class based on `config.vio_linearization_type`.
    *   `virtual Scalar linearizeProblem(bool* valid = nullptr) = 0;`: Linearizes the objective function.
    *   `virtual void performQR() = 0;`: Performs the in-place marginalization of visual landmarks.
    *   `virtual void get_dense_H_b(MatX& H, VecX& b) = 0;`: Returns the dense $\mathbf{H}$ and $\mathbf{b}$ matrices.
    *   `virtual void get_dense_Q2Jp_Q2r(MatX& Q2Jp, VecX& Q2r) = 0;`: Returns the square-root form $\mathbf{J}$ and $\mathbf{r}$.

**5. `MargHelper<Scalar>`**
*   **Header Path:** `include/basalt/vi_estimator/marg_helper.h`
*   **Source Path:** `src/vi_estimator/marg_helper.cpp`
*   **Purpose:** A mathematical utility class containing the static implementations of the Schur complement block operations used to mathematically eliminate rows and columns.
*   **Inheritance:** None.
*   **Key Methods:**
    *   `static void marginalizeHelperSqToSq(...)`: Standard Gaussian elimination ($\mathbf{H}$ to $\mathbf{H}^*$).
    *   `static void marginalizeHelperSqToSqrt(...)`: Gaussian elimination returning the square-root format.
    *   `static void marginalizeHelperSqrtToSqrt(...)`: Stable Householder/QR decomposition on Jacobians.

#### 5.3.2 Execution Flow & Pipeline

The pipeline is executed primarily within `SqrtKeypointVioEstimator::marginalize` (located in `src/vi_estimator/sqrt_keypoint_vio.cpp` and `src/vi_estimator/sqrt_keypoint_vo.cpp`). The sequence of operations is as follows:

##### Keyframe Selection & Sub-Graph Identification
The system evaluates `kf_ids` (the active window) to identify redundant keyframes based on tracking heuristics. Non-keyframe poses are also gathered for elimination. Keyframes scheduled for removal are inserted into the `kfs_to_marg` set.

##### Variable Partitioning and Ordering
An `AbsOrderMap` (`aom`) is constructed to define the exact layout of the dense matrix for the local sub-problem. This sequentially lays out all relevant variables to define the exact row/column dimensions (e.g., $N \times N$) of the dense problem matrix. The following variables are included in this layout:
*   **Previous Prior Variables:** Variables already present in the existing marginalisation prior (`marg_data.order`) are explicitly carried over to preserve historical constraints.
*   **Pure Poses (`frame_poses`):** Older keyframes consisting of a 6-DoF pose ($SE(3)$). These add a `POSE_SIZE` block (6 dimensions).
*   **Full States (`frame_states`):** Recent frames in the active window containing the full kinematic state (Pose, Velocity, and IMU Biases). These add a `POSE_VEL_BIAS_SIZE` block (15 dimensions).

##### Linearization of the Sub-problem
`LinearizationBase` evaluates the current residuals and Jacobians for the isolated sub-graph. It calculates the dense Information Matrix or its square-root equivalent for the combined system (including the previous prior).

##### Index Assignment (Keep vs. Marginalize)
The algorithm iterates over the structured `aom` to partition the matrix indices into two distinct sets: the $\alpha$ variables (`idx_to_keep`), which act as the Markov blanket absorbing the prior to remain active, and the $\beta$ variables (`idx_to_marg`) scheduled for mathematical elimination.
The logic applies nuanced selection to ensure kinematic constraints are dropped for older frames while preserving valuable visual anchor points:
*   **Pure Poses (`poses_to_marg`):** If an old keyframe is redundant, all of its 6 indices are pushed to `idx_to_marg`. If kept, they go to `idx_to_keep`.
*   **Full Non-Keyframe States (`states_to_marg_all`):** Active frames dropping out of the window that were never selected as keyframes are entirely useless for future visual tracking. All 15 dimensions are pushed to `idx_to_marg`.
*   **Full Keyframe States transitioning to Poses (`states_to_marg_vel_bias`):** If an active keyframe is aging out of the "recent" window, its visual constraints remain valuable, but kinematic constraints do not. The first 6 indices (Pose) are pushed to `idx_to_keep`, while the remaining 9 indices (Velocity + Biases) are pushed to `idx_to_marg`.
*   **The Latest State (`last_state_to_marg`):** The boundary state evaluated in the sub-problem has all 15 dimensions pushed to `idx_to_keep`.

##### Schur Complement Computation
The system applies the Schur complement equation to perfectly eliminate the $\beta$ variables, resulting in a dense prior entirely dependent on the $\alpha$ variables. This is handled by `MargHelper::marginalizeHelperSqrtToSqrt` (or equivalent), which physically collapses the dense matrix down, eliminating `idx_to_marg` entirely and returning a smaller dense prior (`marg_H_new`, `marg_b_new`).

##### Logging for Global Mapping
Before the Schur complement is executed, the full, un-eliminated dense matrix, residual vector, and corresponding layout map (`aom`) are assembled into a `MargData::Ptr` object. Along with information about which keyframes are being eliminated (`kfs_to_marg`), this payload is pushed to an output queue (`out_marg_queue`). This comprehensive logging enables the background Non-Linear Factor Recovery (NFR) thread to extract sparse, independent relative pose factors, bridging high-frequency VIO tracking with slower, sparse global mapping without losing information.

##### State Cleanup & Prior Re-centering
The physical state variables and landmark observations belonging exclusively to the marginalized keyframes are deleted from the local system (`lmdb.removeKeyframes(...)`, `frame_poses.erase(...)`). The local `marg_data` is updated with `marg_sqrt_H_new` and `marg_sqrt_b_new`, providing the anchor for the next frame optimization.
*   **Re-centering:** Because the prior is calculated based on the difference (or `delta`) from the *original linearization point*, `marg_data.b` is compensated to ensure the subsequent optimization steps evaluate the cost around a zero-increment baseline.
    ```cpp
    VecX delta;
    computeDelta(marg_data.order, delta); // Compute diff from linearization point
    marg_data.b -= marg_data.H * delta;   // Re-center b
    ```

##### Storage of New Prior
After computing the Schur complement and finalizing state cleanup, the new dense prior (represented by `marg_H_new` and `marg_b_new`) is stored back into the local `marg_data` structure. The layout `marg_data.order` is updated to contain only the configuration of the remaining $\alpha$ variables (`idx_to_keep`). This updated prior carries forward all the historical constraints into the non-linear objective function for the very next tracking optimization step.

---

## 6. Conclusion: From Marginalisation to Global Mapping

The process of marginalisation effectively reduces the dimensionality of the state vector while perfectly summarizing the past linearised visual-inertial constraints into a dense prior ($\mathbf{H}^*$, $\mathbf{b}^*$). Within the fixed-lag smoother, this prior provides the necessary anchoring to prevent drift in the local window.

However, as explored in the theoretical foundations (Usenko et al., Mazuran et al.), a purely dense marginalisation prior becomes computationally intractable to use in a large-scale global factor graph, as every marginalized variable becomes densely correlated with its Markov blanket.

To achieve globally consistent mapping and loop closures, this dense prior is subjected to **Non-Linear Factor Recovery (NFR)**. NFR approximates the dense distribution $p(\mathbf{x}) \sim \mathcal{N}(\mu, \mathbf{H}^{-1})$ stored in the marginalisation prior with a sparse set of new, synthetic non-linear factors $q(\mathbf{x})$. By minimizing the Kullback-Leibler divergence $D_{KL}(p || q)$, the system extracts statistically independent relative pose and roll-pitch constraints. These recovered factors act as highly accurate pseudo-measurements that summarize the high-frame-rate VIO tracking, allowing a much slower, sparser, and computationally efficient global Bundle Adjustment layer to optimize the overall map and correct long-term drift.

---

## 6. References

1. Usenko, V., Demmel, N., Schubert, D., Stückler, J., & Cremers, D. (2020). *Visual-Inertial Mapping with Non-Linear Factor Recovery*. arXiv preprint arXiv:1904.06504v3.
2. Mazuran, M., Burgard, W., & Tipaldi, G. D. (2015). *Nonlinear Factor Recovery for Long-Term SLAM*. The International Journal of Robotics Research (IJRR).
3. `basalt` Open Source Implementation: `include/basalt/vi_estimator/sqrt_keypoint_vio.h` and `src/vi_estimator/sqrt_keypoint_vio.cpp`.
