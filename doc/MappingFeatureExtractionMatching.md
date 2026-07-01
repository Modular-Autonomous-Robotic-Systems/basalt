# Feature Extraction and Matching in Basalt Mapping

## 1. Introduction

A core challenge in long-term Simultaneous Localisation and Mapping (SLAM) is the accumulation of drift: even a well-tuned Visual-Inertial Odometry (VIO) system, operating on a bounded sliding window of recent frames, cannot correct errors that accrued before a keyframe left the active estimation window. Correcting such drift requires a global mapping layer capable of recognising previously visited places and closing loops across the full trajectory. This capability rests entirely on the ability to reliably extract repeatable visual features from images, describe them with discriminative descriptors, and efficiently match those descriptors across frames separated arbitrarily in time and viewpoint.

In the `basalt` framework, the global mapping pipeline — implemented in the `NfrMapper` class — receives keyframe data through `MargData` packets emitted by the VIO marginalisation step (described in `doc/Mapping.md` and `doc/Marginalisation.md`). For each incoming keyframe, the mapper independently re-detects features, computes binary descriptors, and organises them in a Bag-of-Words (BoW) database to enable scalable appearance-based retrieval. Stereo image pairs are matched using known epipolar geometry; non-adjacent keyframes are matched via BoW retrieval followed by geometric verification with a 5-point RANSAC solver. The resulting set of geometrically verified correspondences is fused into multi-view feature tracks, triangulated into 3D landmarks, and ultimately consumed by the global Bundle Adjustment (BA) optimisation that refines the complete trajectory.

The end-to-end pipeline, from raw image to 3D landmark, proceeds as follows:

```
MargData packets (keyframe images + VIO poses)
        │
        ▼
NfrMapper::detect_keypoints()
        │   ├─ detectKeypointsMapping()   — Shi-Tomasi corner detection
        │   ├─ computeAngles()            — intensity-centroid orientation
        │   ├─ computeDescriptors()       — rotated BRIEF (256-bit binary)
        │   ├─ unproject()                — 2D corners → bearing vectors
        │   └─ HashBow: compute_bow()     — L1-normalised BoW vector
        │              add_to_database()  — inverted index insert
        ▼
NfrMapper::match_stereo()               — epipolar filter (known baseline)
NfrMapper::match_all()                  — BoW retrieval + 5-pt RANSAC
        │
        ▼
NfrMapper::build_tracks()               — union-find track fusion + filter
        │
        ▼
NfrMapper::setup_opt()                  — triangulation → LandmarkDatabase
```

The following sections examine each stage in depth, beginning with feature extraction (Section 2) and moving through the two-stage matching pipeline (Section 3). Section 4 provides a consolidated reference for all relevant classes, data structures, and configuration parameters. Section 5 ties the discussion back to the broader mapping system.

---

## 2. Feature Extraction

The feature extraction stage transforms a raw keyframe image into a structured per-camera data record containing detected corner positions, their orientations, binary descriptors, back-projected bearing vectors, and a BoW representation suitable for database lookup. The entire computation is executed in the method `NfrMapper::detect_timestamp_keypoints()` (`src/vi_estimator/nfr_mapper.cpp:465`), called in parallel across all keyframe timestamps by `NfrMapper::detect_keypoints()` (`nfr_mapper.cpp:496`).

### 2.1 Mathematical Background

#### 2.1.1 Corner Detection — Shi-Tomasi Criterion

Interest point detection seeks image locations that are locally distinctive in multiple directions. The Shi-Tomasi detector [Shi and Tomasi, 1994] analyses the second-moment (structure tensor) matrix:

$$\mathbf{M}(x,y) = \sum_{(u,v) \in \mathcal{W}} w(u,v) \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix}$$

where $I_x$ and $I_y$ are image intensity gradients and $\mathcal{W}$ is a local window. The minimum eigenvalue of $\mathbf{M}$ serves as the corner response:

$$R = \min(\lambda_1, \lambda_2)$$

A point is accepted as a corner if $R > q_{\text{level}} \cdot \max_{(x,y)} R$, where $q_{\text{level}}$ is the quality level threshold, and adjacent detections are suppressed to enforce a minimum inter-point distance. This criterion ensures that the selected points have well-conditioned local structure and are thus trackable and matchable.

#### 2.1.2 Keypoint Orientation — Intensity Centroid

To achieve rotation-invariant descriptors, each detected corner is assigned an orientation angle based on the intensity centroid of a circular image patch of radius $r = $ `HALF_PATCH_SIZE` $= 15$ pixels. The image moments within the patch are:

$$m_{10} = \sum_{x,y \in \mathcal{D}} x \cdot I(x, y), \qquad m_{01} = \sum_{x,y \in \mathcal{D}} y \cdot I(x, y)$$

where $\mathcal{D} = \{(x,y) : x^2 + y^2 \leq r^2\}$. The keypoint orientation is then:

$$\theta = \text{atan2}(m_{01},\, m_{10})$$

This is the same orientation formulation used by the ORB descriptor [Rublee et al., 2011], and it provides a stable estimate of the dominant local gradient direction.

#### 2.1.3 Binary Descriptor — Rotated BRIEF

The descriptor for each keypoint is a 256-bit binary string computed by the BRIEF (Binary Robust Independent Elementary Features) paradigm. For each of the 256 bits, a pair of pixel sample locations $(\mathbf{a}_i, \mathbf{b}_i)$ is drawn from a pre-defined pattern. Prior to sampling, both locations are rotated by the keypoint orientation $\theta$:

$$\tilde{\mathbf{a}}_i = \mathbf{R}(\theta)\, \mathbf{a}_i, \qquad \tilde{\mathbf{b}}_i = \mathbf{R}(\theta)\, \mathbf{b}_i$$

where $\mathbf{R}(\theta) \in SO(2)$ is the 2D rotation matrix. The $i$-th descriptor bit is then set according to the binary intensity test:

$$d_i = \begin{cases} 1 & \text{if } I\!\left(\lfloor \tilde{\mathbf{a}}_i \rceil\right) < I\!\left(\lfloor \tilde{\mathbf{b}}_i \rceil\right) \\ 0 & \text{otherwise} \end{cases}$$

where $\lfloor \cdot \rceil$ denotes rounding to the nearest integer pixel. Similarity between two descriptors $\mathbf{d}^{(1)}, \mathbf{d}^{(2)} \in \{0,1\}^{256}$ is measured by the Hamming distance:

$$d_H(\mathbf{d}^{(1)}, \mathbf{d}^{(2)}) = \text{popcount}\!\left(\mathbf{d}^{(1)} \oplus \mathbf{d}^{(2)}\right)$$

which counts the number of differing bits and can be computed in $O(1)$ time on modern hardware using the `popcount` instruction.

#### 2.1.4 Bag-of-Words Hashing

Computing Hamming distances between all descriptor pairs across a growing database of keyframes becomes intractable at scale. The Bag-of-Words paradigm sidesteps this by projecting each descriptor into a discrete vocabulary space. In `basalt`, this projection is implemented as a hash: a subset of `num_bits` (≤ 32) descriptor bits, selected at fixed positions drawn from a pre-computed random permutation, are extracted to form a compact 32-bit word $h \in \{0,1\}^{32}$:

$$h = \text{HashBow::compute\_hash}(\mathbf{d})$$

The BoW vector for an image is the L1-normalised histogram of word frequencies:

$$\mathbf{v}_i = \frac{c_i}{\sum_j c_j}, \qquad c_w = |\{k : h_k = w\}|$$

Two images are compared using the L1 similarity score, computed efficiently via an inverted index without exhaustive pairwise comparison.

### 2.2 Pipeline Description

The extraction pipeline is orchestrated at two levels. The outer function `NfrMapper::detect_keypoints()` (`nfr_mapper.cpp:496`) iterates over the `img_data` store, selects only those timestamps for which a pose exists in `frame_poses`, and dispatches the per-frame work in parallel:

```cpp
// nfr_mapper.cpp:506
tbb::parallel_for(tbb::blocked_range<size_t>(0, keys.size()),
    [&](const tbb::blocked_range<size_t>& r) {
        for (size_t j = r.begin(); j != r.end(); ++j) {
            auto kv = img_data.find(keys[j]);
            if (kv->second.get()) {
                detect_timestamp_keypoints(kv->first, kv->second);
            }
        }
    });
```

For each frame, `NfrMapper::detect_timestamp_keypoints()` (`nfr_mapper.cpp:465`) iterates over every camera in the rig (indexed by `i`) and performs the following steps:

**Step 1 — Image preparation.** The 16-bit raw image stored in `data->img_data[i].img` is reinterpreted as `Image<const uint16_t>`. Images that carry a null pointer are silently skipped (`nfr_mapper.cpp:469`).

**Step 2 — Corner detection.** `detectKeypointsMapping()` (`src/utils/keypoints.cpp:136`) converts the 16-bit image to an 8-bit OpenCV `cv::Mat` by right-shifting each pixel value by 8 bits, then invokes OpenCV's `goodFeaturesToTrack` to detect up to `config.mapper_detection_num_points` Shi-Tomasi corners. Only corners that fall within the image boundary after applying a margin of `EDGE_THRESHOLD = 19` pixels are retained, ensuring that subsequent descriptor sampling at offset $\pm 15$ pixels remains within the image domain.

**Step 3 — Orientation assignment.** `computeAngles()` (`keypoints.cpp:252`) computes the intensity centroid orientation for every detected corner. The summation is performed over all pixels within the circular disc of radius `HALF_PATCH_SIZE = 15`, and the angle is stored per-keypoint in `kd.corner_angles`. The `rotate_features` flag is set to `true` for mapping, enabling the rotation-invariant descriptor computation that follows.

**Step 4 — Descriptor computation.** `computeDescriptors()` (`keypoints.cpp:283`) constructs the 256-bit binary descriptor for each corner. For each bit $i$, the sample coordinates from look-up tables `pattern_31_x_a[i]`, `pattern_31_y_a[i]`, `pattern_31_x_b[i]`, `pattern_31_y_b[i]` are rotated by the stored keypoint angle via `Eigen::Rotation2Dd`, rounded to integer pixel offsets, and used to index the raw image for the intensity comparison test.

**Step 5 — Bearing vector unprojection.** The 2D corner positions are lifted to homogeneous bearing vectors on the unit sphere using the camera intrinsic model:

```cpp
// nfr_mapper.cpp:481
calib.intrinsics[tcid.cam_id].unproject(kd.corners, kd.corners_3d, success);
```

The resulting `kd.corners_3d` (a vector of `Eigen::Vector4d`) are used later by the epipolar and RANSAC geometric verifiers.

**Step 6 — BoW computation.** `HashBow::compute_bow()` (`include/basalt/hash_bow/hash_bow.h:33`) iterates over all descriptors, hashes each one to a 32-bit word via `compute_hash()`, accumulates word frequencies in a local map, and produces the L1-normalised `kd.bow_vector`.

**Step 7 — Database insertion.** `HashBow::add_to_database()` (`hash_bow.h:61`) inserts each `(FeatureHash, weight)` entry from the BoW vector into the thread-safe `inverted_index`, a `tbb::concurrent_unordered_map` mapping each word to a list of `(TimeCamId, weight)` pairs.

**Extraction Pipeline Flowchart:**

```
Raw Image (uint16, per camera)
         │
         ▼
detectKeypointsMapping()          [keypoints.cpp:136]
  • 16-bit → 8-bit (>> 8)
  • goodFeaturesToTrack (Shi-Tomasi)
  • clip to EDGE_THRESHOLD = 19 px border
         │  kd.corners: vector<Eigen::Vector2d>
         ▼
computeAngles()                   [keypoints.cpp:252]
  • intensity centroid over disc radius = 15
  • θ = atan2(m01, m10)
         │  kd.corner_angles: vector<double>
         ▼
computeDescriptors()              [keypoints.cpp:283]
  • rotate pattern pairs by θ via Eigen::Rotation2Dd
  • 256-bit binary intensity tests
         │  kd.corner_descriptors: vector<bitset<256>>
         ▼
calib.intrinsics[cam].unproject() [per camera model]
  • 2D corner → 4D homogeneous bearing vector
         │  kd.corners_3d: vector<Eigen::Vector4d>
         ▼
HashBow::compute_bow()            [hash_bow.h:33]
  • 256-bit descriptor → 32-bit hash word
  • L1-normalised word frequency histogram
         │  kd.hashes, kd.bow_vector
         ▼
HashBow::add_to_database()        [hash_bow.h:61]
  • insert (TimeCamId, weight) into inverted_index
         │
         ▼
feature_corners[tcid] = kd        [NfrMapper member]
```

The complete `KeypointsData` record accumulated at each `TimeCamId` encapsulates every quantity needed by the subsequent matching stages: 2D image locations for track management, bearing vectors for geometric verification, binary descriptors for Hamming-distance matching, and BoW vectors for scalable retrieval.

---

## 3. Feature Matching

With descriptors extracted and indexed for every keyframe and camera, the matching stage establishes geometrically verified correspondences across image pairs. Two distinct matching strategies are applied in sequence: stereo matching, which exploits the known calibrated baseline between cameras at the same timestep, and appearance-based matching across all frame pairs, which uses BoW retrieval to limit the search space and RANSAC to enforce geometric consistency. The verified correspondences are subsequently fused into multi-view feature tracks.

### 3.1 Mathematical Background and Prerequisites

#### 3.1.1 Mutual Nearest-Neighbour with Ratio Test

A naive nearest-neighbour match between descriptor sets $\mathcal{D}_1$ and $\mathcal{D}_2$ is prone to false positives when a query descriptor has no genuine match in the target set and simply attaches to the globally closest (but still distant) descriptor. Two mechanisms are applied jointly to reduce this ambiguity:

**Mutual nearest-neighbour (MNN):** A match $(i, j)$ is accepted only if $j$ is the nearest neighbour of $i$ in $\mathcal{D}_2$ *and* $i$ is the nearest neighbour of $j$ in $\mathcal{D}_1$. This bidirectional check rejects one-to-many assignments.

**Lowe's ratio test:** Even among mutual nearest neighbours, a match is accepted only when the best Hamming distance is significantly smaller than the second-best distance in the same direction:

$$d_H(\mathbf{d}_i^{(1)}, \mathbf{d}_j^{(2)}) < \tau \quad \text{and} \quad d_H(\mathbf{d}_i^{(1)}, \mathbf{d}_j^{(2)}) \cdot \rho \leq d_H(\mathbf{d}_i^{(1)}, \mathbf{d}_{j'}^{(2)})$$

where $\tau$ is the absolute Hamming threshold, $\rho$ is the ratio test value (`mapper_second_best_test_ratio`), and $j'$ is the second-best match. Together, MNN and the ratio test ensure that only unambiguous, distinctive matches proceed to geometric verification.

#### 3.1.2 Epipolar Geometry — Essential Matrix for Stereo

For a stereo camera pair with known relative pose $T_{0,1} \in SE(3)$ (translation $\mathbf{t}_{0,1}$ and rotation $\mathbf{R}_{0,1}$), the epipolar constraint on bearing vectors $\mathbf{p}_0, \mathbf{p}_1$ (homogeneous, from `corners_3d`) is:

$$\mathbf{p}_0^T \mathbf{E} \, \mathbf{p}_1 = 0$$

where the Essential matrix is:

$$\mathbf{E} = [\hat{\mathbf{t}}_{0,1}]_\times \mathbf{R}_{0,1}, \qquad \hat{\mathbf{t}}_{0,1} = \frac{\mathbf{t}_{0,1}}{\|\mathbf{t}_{0,1}\|}$$

with $[\cdot]_\times$ denoting the skew-symmetric cross-product matrix. Any matched pair $(i,j)$ with $|\mathbf{p}_{0,i}^T \mathbf{E} \, \mathbf{p}_{1,j}| > \epsilon_E$ violates the epipolar constraint and is classified as an outlier.

#### 3.1.3 Five-Point RANSAC for Unknown Relative Pose

When the relative pose between two keyframes is unknown, geometric verification must simultaneously estimate the pose and classify inliers. The minimal case for a calibrated camera pair requires five point correspondences to recover the Essential matrix (Nistér, 2004; Stewenius et al., 2006). RANSAC iterates:

1. Draw a minimal sample of 5 bearing-vector pairs.
2. Compute the set of Essential matrix hypotheses from the 5-point solver.
3. Count inliers: correspondences $(i,j)$ satisfying $|\mathbf{p}_i^T \mathbf{E} \, \mathbf{p}_j| \leq \epsilon_R$.
4. Retain the hypothesis with the largest inlier count.

After RANSAC, the best hypothesis is refined by non-linear optimisation over all inliers, and a final inlier re-classification is performed. The recovered translation $\mathbf{t}$ is scale-ambiguous in the monocular case and is therefore normalised to unit length.

#### 3.1.4 Bag-of-Words Retrieval via Inverted Index

Place recognition across a database of $N$ keyframes would require $O(N)$ descriptor set comparisons per query if performed naively. The BoW inverted index reduces this to a small set of candidate frames. Each database frame $d$ has an associated normalised BoW vector $\mathbf{v}_d$, and a query frame $q$ presents vector $\mathbf{v}_q$. The L1-based similarity score is:

$$s(q, d) = \frac{1}{2} \sum_w \left( |v_{q,w}| + |v_{d,w}| - |v_{q,w} - v_{d,w}| \right)$$

This is equivalent to the inner product on BoW vectors and ranges in $[0, 1]$. The score is computed in $O(|\mathbf{v}_q|)$ time using the inverted index: only words present in the query vector are examined, and only frames sharing at least one word contribute to the score. The top-$k$ ranked frames above a threshold $\tau_{\text{bow}}$ (`mapper_frames_to_match_threshold`) are returned as candidates for descriptor-level matching.

### 3.2 Pipeline Description

#### 3.2.1 Stereo Matching

`NfrMapper::match_stereo()` (`src/vi_estimator/nfr_mapper.cpp:529`) matches left and right camera images at each timestamp using the calibrated stereo geometry.

The method first computes the Essential matrix from the known inter-camera transform:

```cpp
// nfr_mapper.cpp:531
const Sophus::SE3d T_0_1 = calib.T_i_c[0].inverse() * calib.T_i_c[1];
Eigen::Matrix4d E;
computeEssential(T_0_1, E);  // keypoints.h:73
```

`computeEssential()` (`include/basalt/utils/keypoints.h:73`) constructs `E` by computing the skew-symmetric matrix of the normalised translation and multiplying by the rotation matrix.

For every timestamp present in `img_data`, the method retrieves `KeypointsData` for `TimeCamId(t, 0)` and `TimeCamId(t, 1)`, then calls `matchDescriptors()` (`keypoints.cpp:343`) with the stereo-specific thresholds `config.mapper_max_hamming_distance` and `config.mapper_second_best_test_ratio`. The resulting candidate matches are passed to `findInliersEssential()` (`keypoints.h:81`), which retains only those pairs satisfying:

$$\left| \mathbf{p}_{0,i}^T \, \mathbf{E} \, \mathbf{p}_{1,j} \right| < 10^{-3}$$

using the pre-unprojected bearing vectors from `kd.corners_3d`. Stereo pairs with at least 16 inliers are stored in `feature_matches` (`nfr_mapper.cpp:562`).

#### 3.2.2 Place-Recognition Matching

`NfrMapper::match_all()` (`nfr_mapper.cpp:571`) handles matching between arbitrary non-adjacent keyframes. The procedure comprises two parallel phases.

**Phase 1 — BoW retrieval** (`nfr_mapper.cpp:590`). A TBB parallel loop iterates over every `TimeCamId` in `feature_corners`. For each, `HashBow::querry_database()` (`hash_bow.h:69`) is called with the frame's BoW vector, requesting the top `config.mapper_num_frames_to_match` results. The scoring function accumulates partial L1 contributions through the inverted index; only frames with a BoW score exceeding `config.mapper_frames_to_match_threshold` and with a different `frame_id` are admitted as matching candidates. Valid pairs are collected into the thread-safe `ids_to_match` vector.

**Phase 2 — Geometric verification** (`nfr_mapper.cpp:632`). A second parallel loop processes each candidate pair. Descriptor-level matching is performed first:

```cpp
// nfr_mapper.cpp:645
matchDescriptors(f1.corner_descriptors, f2.corner_descriptors,
                 md.matches, 70, 1.2);
```

The Hamming threshold is fixed at 70 bits and the ratio test at 1.2. If the number of putative matches exceeds `config.mapper_min_matches`, RANSAC geometric verification is triggered:

```cpp
// nfr_mapper.cpp:651
findInliersRansac(f1, f2, config.mapper_ransac_threshold,
                  config.mapper_min_matches, md);
```

`findInliersRansac()` (`keypoints.cpp:362`) constructs bearing vector arrays from `kd.corners_3d`, creates an OpenGV `CentralRelativeAdapter`, and runs a RANSAC loop with the `CentralRelativePoseSacProblem` solver set to the STEWENIUS algorithm (`keypoints.cpp:386`). After 100 RANSAC iterations, the best model is refined by `opengv::relative_pose::optimize_nonlinear()` over the full inlier set, and a final inlier re-classification is performed. The recovered unit-normalised translation and rotation are stored in `md.T_i_j`. If the inlier count meets `config.mapper_min_matches`, the `MatchData` is inserted into `feature_matches`.

#### 3.2.3 Mutual Nearest-Neighbour Descriptor Matching

Both stereo and place-recognition matching delegate the raw descriptor comparison to `matchDescriptors()` (`keypoints.cpp:343`). This function performs mutual nearest-neighbour matching by calling the inner helper `matchFastHelper()` (`keypoints.cpp:314`) twice — once in each direction — and then intersecting the two result sets:

```cpp
// keypoints.cpp:350
matchFastHelper(corner_descriptors_1, corner_descriptors_2, matches_1_2, ...);
matchFastHelper(corner_descriptors_2, corner_descriptors_1, matches_2_1, ...);

for (const auto& kv : matches_1_2) {
    if (matches_2_1[kv.second] == kv.first)
        matches.emplace_back(kv.first, kv.second);
}
```

`matchFastHelper()` performs a brute-force linear scan, tracking the best and second-best Hamming distances for each query descriptor. A match is accepted only when both the absolute threshold and the ratio test are satisfied (`keypoints.cpp:337`).

#### 3.2.4 Track Building

`NfrMapper::build_tracks()` (`nfr_mapper.cpp:690`) fuses all pairwise inlier correspondences stored in `feature_matches` into consistent multi-view feature tracks using a union-find data structure encapsulated in `TrackBuilder` (`include/basalt/utils/tracks.h`):

```cpp
// nfr_mapper.cpp:692
trackBuilder.Build(feature_matches);          // union-find fusion
trackBuilder.Filter(config.mapper_min_track_length);  // remove short tracks
trackBuilder.Export(feature_tracks);          // emit {TrackId → FeatureTrack}
```

`TrackBuilder::Build()` assigns a unique node to every `(TimeCamId, FeatureId)` observation and calls `Union()` for every inlier pair, merging the equivalence classes. `Filter()` discards equivalence classes whose root corresponds to a track shorter than `mapper_min_track_length`. `Export()` traverses the union-find forest to produce the final `FeatureTracks` map indexed by `TrackId`.

**Matching Pipeline Flowchart:**

```
feature_corners {TimeCamId → KeypointsData}
         │
         ├─────────────────────────────────────────────┐
         │  STEREO PATH (known geometry)                │  PLACE RECOGNITION PATH
         │                                              │
         ▼                                              ▼
computeEssential(T_0_1)          HashBow::querry_database()   [hash_bow.h:69]
  [keypoints.h:73]                 • L1 BoW score via inverted index
         │                         • top-k frames above threshold
         ▼                                              │
matchDescriptors()               ids_to_match: candidate pairs
  [keypoints.cpp:343]                                   │
  • matchFastHelper() ×2                                ▼
  • MNN + ratio test              matchDescriptors()    [keypoints.cpp:343]
         │                         • MNN + Hamming threshold 70, ratio 1.2
         ▼                                              │
findInliersEssential()                                  ▼
  [keypoints.h:81]               findInliersRansac()    [keypoints.cpp:362]
  • |p0ᵀ E p1| < 1e-3             • OpenGV STEWENIUS 5-point RANSAC
  • min 16 inliers                 • non-linear refinement
         │                         • normalised translation
         └──────────────┬──────────────────────────────┘
                        ▼
            feature_matches {(TimeCamId, TimeCamId) → MatchData}
                        │
                        ▼
            TrackBuilder::Build()     [tracks.h]
              • union-find fusion of all inlier pairs
            TrackBuilder::Filter()
              • discard tracks < mapper_min_track_length
            TrackBuilder::Export()
                        │
                        ▼
            feature_tracks {TrackId → FeatureTrack}
                        │
                        ▼
            NfrMapper::setup_opt()
              • triangulation → LandmarkDatabase (lmdb)
```

---

## 4. Key Classes and Interfaces

### 4.1 Data Types

| Type | Header | Line | Description |
| :--- | :--- | :--- | :--- |
| `KeypointsData` | `include/basalt/utils/common_types.h` | 95 | Per-image feature record: `corners` (2D), `corner_angles`, `corner_descriptors` (256-bit), `corners_3d` (bearing vectors), `hashes`, `bow_vector`. |
| `MatchData` | `include/basalt/utils/common_types.h` | 117 | Per-pair match record: `T_i_j` (relative pose), `matches` (putative pairs), `inliers` (verified pairs). |
| `Descriptor` | `include/basalt/utils/keypoints.h` | 51 | `std::bitset<256>` — full 256-bit binary descriptor. |
| `FeatureHash` | `include/basalt/utils/common_types.h` | 91 | `std::bitset<32>` — 32-bit BoW word hash. |
| `HashBowVector` | `include/basalt/utils/common_types.h` | 92 | `std::vector<std::pair<FeatureHash, double>>` — L1-normalised BoW vector. |
| `Corners` | `include/basalt/utils/common_types.h` | 113 | `tbb::concurrent_unordered_map<TimeCamId, KeypointsData>` — database of per-frame feature records. |
| `Matches` | `include/basalt/utils/common_types.h` | 130 | `tbb::concurrent_unordered_map<std::pair<TimeCamId,TimeCamId>, MatchData>` — database of per-pair match results. |
| `FeatureTracks` | `include/basalt/utils/common_types.h` | 150 | `std::unordered_map<TrackId, FeatureTrack>` — multi-view feature track database. |
| `ImageFeaturePair` | `include/basalt/utils/common_types.h` | 138 | `std::pair<TimeCamId, FeatureId>` — node type used by union-find track builder. |

### 4.2 Classes

| Class | Header | Source | Purpose |
| :--- | :--- | :--- | :--- |
| `NfrMapper` | `include/basalt/vi_estimator/nfr_mapper.h:56` | `src/vi_estimator/nfr_mapper.cpp` | Top-level mapper. Owns `img_data`, `feature_corners`, `feature_matches`, `feature_tracks`, `hash_bow_database`. Orchestrates all extraction and matching stages. |
| `HashBow<N>` | `include/basalt/hash_bow/hash_bow.h:17` | header-only | Template BoW database parameterised by descriptor length `N`. Computes hashes, BoW vectors, and maintains a concurrent inverted index for image retrieval. |
| `TrackBuilder` | `include/basalt/utils/tracks.h:50` | header-only | Union-find based multi-view track construction. Provides `Build()`, `Filter()`, and `Export()`. |

### 4.3 Free Functions

| Function | File | Line | Role |
| :--- | :--- | :--- | :--- |
| `detectKeypointsMapping()` | `src/utils/keypoints.cpp` | 136 | Shi-Tomasi corner detection via OpenCV `goodFeaturesToTrack`; edge-margin filtering. |
| `detectKeypoints()` | `src/utils/keypoints.cpp` | 161 | Grid-based FAST corner detection (used in VIO frontend, not mapping). |
| `computeAngles()` | `src/utils/keypoints.cpp` | 252 | Intensity-centroid orientation estimation over circular patch. |
| `computeDescriptors()` | `src/utils/keypoints.cpp` | 283 | Rotated BRIEF 256-bit binary descriptor computation. |
| `matchFastHelper()` | `src/utils/keypoints.cpp` | 314 | Brute-force Hamming nearest-neighbour with ratio test (single direction). |
| `matchDescriptors()` | `src/utils/keypoints.cpp` | 343 | Mutual nearest-neighbour descriptor matching using `matchFastHelper()` in both directions. |
| `computeEssential()` | `include/basalt/utils/keypoints.h` | 73 | Computes Essential matrix $\mathbf{E} = [\hat{\mathbf{t}}]_\times \mathbf{R}$ from an `SE3` transform. |
| `findInliersEssential()` | `include/basalt/utils/keypoints.h` | 81 | Epipolar inlier filtering using pre-computed Essential matrix and bearing vectors. |
| `findInliersRansac()` | `src/utils/keypoints.cpp` | 362 | 5-point RANSAC via OpenGV STEWENIUS solver with non-linear refinement. |
| `HashBow::compute_hash()` | `include/basalt/hash_bow/hash_bow.h` | 25 | Projects a 256-bit descriptor to a 32-bit BoW word using a fixed bit permutation. |
| `HashBow::compute_bow()` | `include/basalt/hash_bow/hash_bow.h` | 33 | Computes the L1-normalised BoW vector for a set of descriptors. |
| `HashBow::add_to_database()` | `include/basalt/hash_bow/hash_bow.h` | 61 | Inserts a frame's BoW vector into the concurrent inverted index. |
| `HashBow::querry_database()` | `include/basalt/hash_bow/hash_bow.h` | 69 | Retrieves the top-$k$ most similar frames from the inverted index by L1 BoW score. |

### 4.4 Configuration Parameters

All parameters below are members of `VioConfig` (`include/basalt/utils/vio_config.h`), prefixed with `mapper_`.

| Parameter | Role |
| :--- | :--- |
| `mapper_detection_num_points` | Maximum number of corners requested from `goodFeaturesToTrack` per image. |
| `mapper_bow_num_bits` | Number of bits extracted per descriptor to form a BoW hash word (clamped to ≤ 32). |
| `mapper_num_frames_to_match` | Top-$k$ frames retrieved from the BoW database per query. |
| `mapper_frames_to_match_threshold` | Minimum BoW similarity score for a frame to enter the candidate matching list. |
| `mapper_max_hamming_distance` | Maximum Hamming distance for stereo descriptor matching. |
| `mapper_second_best_test_ratio` | Lowe's ratio test multiplier $\rho$ applied in both stereo and place-recognition matching. |
| `mapper_min_matches` | Minimum number of putative matches required before RANSAC is attempted. |
| `mapper_ransac_threshold` | Inlier threshold $\epsilon_R$ for the 5-point RANSAC epipolar constraint check. |
| `mapper_min_track_length` | Minimum number of observations required for a track to survive `TrackBuilder::Filter()`. |

---

## 5. Conclusion

The feature extraction and matching pipeline described in this document forms the perceptual backbone of the `basalt` global mapping layer. By operating independently of the VIO sliding-window estimator — re-detecting features from scratch on each marginalised keyframe — the mapper avoids the temporal locality constraints imposed on the VIO optical flow frontend and can establish correspondences across the full trajectory.

The extraction pipeline combines well-understood, computationally efficient components: Shi-Tomasi corners for detection, intensity-centroid orientation for rotation invariance, and rotated BRIEF binary descriptors for compact, Hamming-computable representation. The BoW hashing layer built on top of these descriptors enables logarithmic-time image retrieval regardless of map size, confining the expensive RANSAC verification to a small set of genuinely similar candidate pairs per query frame.

Within the global mapping sequence, the verified feature tracks produced by `build_tracks()` are consumed immediately by `setup_opt()`, which triangulates each track into a 3D landmark hosted in the `LandmarkDatabase` (`lmdb`). These landmarks, together with the Non-Linear Factor Recovery (NFR) relative-pose and roll-pitch factors derived from the VIO marginal covariance (described in `doc/Mapping.md`), constitute the full factor graph over which `NfrMapper::optimize()` performs global Bundle Adjustment. The quality of that optimisation — and therefore the global consistency of the reconstructed map — is directly conditioned on the precision and recall of the matching stage documented here.

---

## References

1. Usenko, V., Demmel, N., Schubert, D., Stückler, J., & Cremers, D. (2020). *Visual-Inertial Mapping with Non-Linear Factor Recovery*. arXiv preprint arXiv:1904.06504v3.
2. Mazuran, M., Burgard, W., & Tipaldi, G. D. (2015). *Nonlinear Factor Recovery for Long-Term SLAM*. The International Journal of Robotics Research (IJRR).
3. Rublee, E., Rabaud, V., Konolige, K., & Bradski, G. (2011). *ORB: An efficient alternative to SIFT or SURF*. ICCV 2011.
4. Nistér, D. (2004). *An efficient solution to the five-point relative pose problem*. IEEE TPAMI.
5. Stewenius, H., Engels, C., & Nistér, D. (2006). *Recent developments on direct relative orientation*. ISPRS Journal.
6. Shi, J., & Tomasi, C. (1994). *Good features to track*. CVPR 1994.
