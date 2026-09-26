# Feature Detection and Descriptors in the Mapping Path

Created 2026-09-12 while expanding `doc/Mapping.md` Section 3.2. This file records what was derived or measured about `detectKeypointsMapping`, `computeAngles` and `computeDescriptors` in `src/utils/keypoints.cpp`, together with the constants and thresholds that depend on them. The prose exposition lives in `doc/Mapping.md` Section 3.2 and `doc/MappingFeatureExtractionMatching.md`. What is here is the set of facts that are not visible from reading either the code or those documents alone.

## Which detector is actually used

`detectKeypointsMapping` (`keypoints.cpp:136-159`) calls `goodFeaturesToTrack(image, points, num_features, 0.01, 8)`. The five-argument form leaves the OpenCV defaults `blockSize = 3`, `useHarrisDetector = false`, `k = 0.04`, so the response is the Shi-Tomasi minimum eigenvalue of the 3 by 3 structure tensor, not Harris and not FAST. This matters because `doc/Mapping.md` said "FAST/Harris" until 2026-09-12 and because the same file contains a genuinely FAST-based detector.

`detectKeypoints` (`keypoints.cpp:161-250`) is a different function entirely. It tiles the image into `PATCH_SIZE` cells, skips cells that already contain a tracked point, and runs `cv::FAST` per cell with a threshold that halves from 40 down to 5 until `num_points_cell` corners are found. It serves the visual odometry front end and is never called from the mapping path. Do not confuse the two when grepping.

Two functions, two detectors, one file. The mapping one is the one without a cell grid.

## EDGE_THRESHOLD = 19 is derived, not chosen

The four `pattern_31_*` tables hold values in `[-13, 12]`, so the largest distance from a patch centre to a BRIEF sample is `sqrt(13^2 + 13^2) = 18.38` pixels. Rotation preserves that norm and the per-axis rounding in `computeDescriptors` adds at most half a pixel, so a steered sample never lands further than 19 pixels from the corner. `Image::InBounds(x, y, border)` requires `border <= x < w - border - 1` (`thirdparty/basalt-headers/include/basalt/image/image.h:723-727`). A border of 19 is therefore exactly sufficient and not one pixel more. `HALF_PATCH_SIZE = 15`, needed by `computeAngles`, is covered a fortiori.

Consequence. Changing the pattern tables, or adding a scale pyramid that scales the pattern by the octave, invalidates `EDGE_THRESHOLD` and must be accompanied by recomputing it. Nothing in the code asserts the relationship.

## The pattern is OpenCV's learned ORB table verbatim

`pattern_31_x_a`, `pattern_31_y_a`, `pattern_31_x_b`, `pattern_31_y_b` at `keypoints.cpp:58-134` are OpenCV's `bit_pattern_31_` de-interleaved into four arrays. Verified element for element on the leading quadruples, which are `(8,-3,9,5)`, `(4,2,7,-12)`, `(-11,9,-8,2)`, `(7,-12,12,-13)`. `basalt` therefore inherits ORB's greedy decorrelation learning, the search over roughly 205k candidate tests on about 300k patches selecting for mean near 0.5 and low pairwise correlation, without carrying any training code. Treat the tables as a frozen external artefact.

## Two deliberate deviations from reference ORB

1. Steering is computed at full precision per keypoint through `Eigen::Rotation2Dd` (`keypoints.cpp:296-304`). OpenCV quantises the angle to `2*pi/30` and precomputes thirty steered tables. `basalt` pays 512 matrix-vector products per descriptor to remove a steering quantisation of up to six degrees.

2. No smoothing is applied before the intensity tests. BRIEF as published requires it, a Gaussian of standard deviation 2 over a 9 by 9 window, and OpenCV's ORB uses a 5 by 5 box filter through an integral image. `basalt` samples raw 16 bit pixels. This is the cheapest available accuracy fix in the whole extraction stage.

Also note the bit-depth asymmetry. Detection runs on the 8 bit image produced by `>> 8`, while `computeAngles` and `computeDescriptors` both read the original 16 bit `img_raw`. The shifted copy is built and discarded once per camera per keyframe.

## Threshold calibrations, computed

Two independent random 256 bit codes have Hamming distance distributed as `Binomial(256, 0.5)`, mean 128, standard deviation 8.

- `mapper_max_hamming_distance = 70` sits 7.25 standard deviations into the lower tail. Exact tail probability `P(d_H <= 70) = 1.28e-13`. The threshold is extremely conservative against chance agreement, so a surviving false match is a real appearance ambiguity in the scene, never a statistical accident. Raising it is safe from a chance-collision standpoint and the binding constraint is elsewhere.

- `mapper_bow_num_bits = 16` selects 16 of the 256 bits through `word_bit_permutation`, giving a 65536 word vocabulary. Two unrelated descriptors collide with probability `2^-16 = 1.5e-5`. A true match at Hamming distance `h` shares a word only with probability `C(256-h,16)/C(256,16)`, which evaluates to 0.52 at h=10, 0.26 at h=20, 0.13 at h=30, 0.027 at h=50 and 0.005 at h=70. A good match is therefore more likely than not to be split across two words. This is the quantitative reason `mapper_frames_to_match_threshold` has to be as low as 0.04, and it is a prime suspect for the 2026-09-10 zero-retrieval failure recorded in `keyframe_driven_local_mapping.md`. The remedy that needs no training is to hash each descriptor into L disjoint bit subsets, which takes the miss rate from `1-p` to `(1-p)^L`.

- `mapper_obs_std_dev = 0.25` against a pixel-grid quantisation standard deviation of `1/sqrt(12) = 0.289`. Corner positions are never refined below the pixel, because the `cv::cornerSubPix` call is commented out at `keypoints.cpp:239-243`. The assumed observation noise in the bundle adjustment is therefore already exceeded by quantisation alone, leaving no budget for detector jitter or calibration residual. Restoring sub-pixel refinement is what would make the assumed covariance defensible, not merely smaller residuals.

## The HashBow score is exactly the DBoW2 L1 score

`HashBowStl::querry_database` (`hash_bow.h:229-260`) accumulates `|q_w - d_w| - |q_w| - |d_w|` per shared word and reports `-score/2`. For non-negative histogram entries the summand is `-2*min(q_w, d_w)`, so the reported score is the histogram intersection `sum_w min(q_w, d_w)`. Both vectors are L1 normalised, so this equals `1 - 0.5 * ||q - d||_1`, which is the L1 similarity of Galvez-Lopez and Tardos. `basalt` reproduces the DBoW2 score over an untrained locality-sensitive hash vocabulary rather than a k-means tree, which is why no vocabulary file ships with the project. Score range is [0,1].

The query at `nfr_mapper.cpp:612-614` passes `&tcid.frame_id` as `max_t_ns`, so retrieval is strictly backward in time. A frame never retrieves a later frame.

## Detection is serial, retrieval is not

The `tbb::parallel_for` in `NfrMapper::detect_keypoints` was removed in commit `5de98a1` on 2026-06-25 because two threads inserting into `HashBowStl::inverted_index` corrupted it. The loop at `nfr_mapper.cpp:518-523` carries a comment saying so. `match_all` kept its `tbb::parallel_for` at `nfr_mapper.cpp:638` because `querry_database` only reads the index.

The detection work itself was never the race. Splitting the loop into a parallel phase that computes corners, angles, descriptors, bearing vectors and bag-of-words vectors into per-frame local storage, followed by a short serial phase performing only `add_to_database`, recovers nearly all the concurrency and removes the race by construction. `detect_keypoints` is on the live local-mapper path through `local_mapper.cpp:170`, so this is a real-time cost, not an offline one.

Both `doc/Mapping.md` Section 3.2 and `doc/MappingFeatureExtractionMatching.md` Section 2.2 asserted the parallel version until 2026-09-12, the latter with a `tbb::parallel_for` code snippet that no longer existed.

## Failure modes worth instrumenting

1. `goodFeaturesToTrack` accepts a corner only if its response exceeds `qualityLevel` times the maximum response in the whole image. One specular highlight or one very high contrast structure therefore raises the bar across the entire frame and the detected count can collapse on an otherwise well-textured image. A per-cell quality threshold removes this.

2. `computeAngles` applies no gate on the centroid magnitude. When the disc is close to radially symmetric the moment vector shrinks towards zero and the angle is noise, and in the exact degenerate case IEEE 754 gives `atan2(0, 0) = 0`, so a wholly undetermined orientation is silently reported as zero radians. Such keypoints produce descriptors that cannot match their own counterparts in an adjacent view.

3. No scale pyramid anywhere in the mapping path. Single-resolution detection means no scale invariance, which bounds the range of viewpoint change over which a loop can close.

These three are the concrete candidates behind the `[bow]` instrumentation item in `keyframe_driven_local_mapping.md` Section 6.8, which needs to separate "no keypoints at all" from "keypoints fine, retrieval too strict".

## Related

- [`keyframe_driven_local_mapping.md`](keyframe_driven_local_mapping.md) for the local mapper pipeline that calls `detect_keypoints`, and for the 2026-09-10 retrieval failure measurement.
- [`vio_config_json.md`](vio_config_json.md) for the recipe to expose any of these constants as a configuration parameter.
- `doc/Mapping.md` Section 3.2 for the full derivations and the improvement survey with references.
