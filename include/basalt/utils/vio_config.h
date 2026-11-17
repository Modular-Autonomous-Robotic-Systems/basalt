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
#pragma once

#include <cstddef>
#include <string>

namespace basalt {

enum class LinearizationType { ABS_QR, ABS_SC, REL_SC };

struct VioConfig {
    VioConfig();
    void load(const std::string& filename);
    void save(const std::string& filename);

    std::string optical_flow_type;
    int optical_flow_detection_grid_size;
    float optical_flow_max_recovered_dist2;
    int optical_flow_pattern;
    int optical_flow_max_iterations;
    int optical_flow_levels;
    float optical_flow_epipolar_error;
    int optical_flow_skip_frames;

    LinearizationType vio_linearization_type;
    bool vio_sqrt_marg;

    int vio_max_states;
    int vio_max_kfs;
    int vio_min_frames_after_kf;
    float vio_new_kf_keypoints_thresh;
    bool vio_debug;
    bool vio_extended_logging;

    //  double vio_outlier_threshold;
    //  int vio_filter_iteration;
    int vio_max_iterations;

    double vio_obs_std_dev;
    double vio_obs_huber_thresh;
    double vio_min_triangulation_dist;
    // Minimum parallax angle in degrees required between the two bearings of
    // a triangulation candidate. Zero leaves the test inactive, reproducing
    // the behaviour of every configuration written before this field existed.
    double vio_min_triangulation_parallax_deg;

    bool vio_enforce_realtime;

    bool vio_use_lm;
    double vio_lm_lambda_initial;
    double vio_lm_lambda_min;
    double vio_lm_lambda_max;

    bool vio_scale_jacobian;

    double vio_init_pose_weight;
    double vio_init_ba_weight;
    double vio_init_bg_weight;
    // Seed the initial attitude from the mean of the inertial samples skipped
    // before the first frame rather than from the last of them. False
    // reproduces the single sample seed of every earlier configuration.
    // Suggested true, which divides the 0.89 degree tilt noise of one sample
    // by sqrt(N). See plans/vio_drift_analysis.md Phase 3a.
    bool vio_init_accel_averaging;
    // Weight 1 / sigma_v of the zero velocity constraint applied to the newest
    // state while the stationarity test passes. Zero leaves it inactive.
    // Suggested 100, that is sigma_v = 0.01 m/s for a vehicle on the ground,
    // three orders below the 1e4 position anchor so it cannot dominate the
    // conditioning of the dense system. See plans/vio_drift_analysis.md
    // Phase 3b.
    double vio_zero_velocity_weight;
    // Standard deviations of the zero relative motion between consecutive
    // stationary states, translation in metres and rotation in degrees,
    // 0 disables.
    double vio_zero_motion_pos_std;
    double vio_zero_motion_rot_std_deg;

    // Independently gated stationary body detector stages, combined by
    // logical AND into mpIsStationary.
    bool vio_stationary_enable_shoe;
    bool vio_stationary_enable_mahalanobis;
    bool vio_stationary_enable_flow;
    // Ring buffer span in seconds for the windowed SHOE statistic, and its
    // chi-squared critical value.
    double vio_stationary_window_s;
    double vio_stationary_glrt_tol;
    // Consecutive clearing windows required before the SHOE stage latches.
    int vio_stationary_min_consecutive_frames;
    // Chi-squared(3) critical value for the Mahalanobis stage.
    double vio_stationary_chisq_tol;
    // Optical flow magnitude veto threshold in pixels.
    double vio_stationary_flow_tol;
    // One-way start latch, plans/stationary_detection.md change 8. Once
    // vio_stationary_latch_frames consecutive frames are non-stationary, no
    // later frame is stationary for the rest of the run. Suggested true with
    // 3 frames on platforms that only rest before take-off.
    bool vio_stationary_enable_start_latch;
    int vio_stationary_latch_frames;
    // Optimise gravity direction and metric scale until vio_gs_duration_s
    // seconds after the first frame of confirmed motion, then fold them into
    // the state. Requires
    // vio_linearization_type ABS_QR and vio_sqrt_marg.
    bool vio_gs_enable;
    double vio_gs_duration_s;
    // Linearise the delayed prior with the scale free rows of the inertial
    // factors, plans/gravity_scale_optimisation_2.md Change A.
    bool vio_gs_visual_prior_inertial;
    // Largest whitened scale column with which an inertial factor enters
    // the delayed prior with all of its rows.
    double vio_gs_visual_prior_scale_tol;
    // Random walk intensities of the per frame gravity and scale step, per
    // square root second, zero disables, plans/gravity_scale_optimisation_2.md
    // Change B.
    double vio_gs_jump_sigma_rw;
    double vio_gs_jump_tilt_rw_deg;
    // Delayed marginalisation, plans/delayed_marginalisation.md. The store
    // keeps the last vio_gs_dm_delay marginalisations. Until it first holds
    // that many, the block is held in the window and a coarse inertial
    // optimisation runs at every new keyframe. Then
    // the block is free and the prior is readvanced at the current block when
    // the block moves past either threshold.
    int vio_gs_dm_delay;
    double vio_gs_dm_thresh_scale;
    double vio_gs_dm_thresh_tilt_deg;
    // Largest marginal variance of the log scale with which a coarse
    // optimisation updates the scale.
    double vio_gs_dm_coarse_scale_var;
    bool vio_gs_dm_smooth;
    bool vio_gs_dm_async;
    // Drop the local mapper's pose updates during the period,
    // plans/delayed_marginalisation.md §2.3.8.
    bool vio_gs_dm_hold_pose_updates;
    // Count vio_gs_dm_delay from the end of the stationary period, and at
    // every moving frame that follows a stationary one reduce the prior to
    // its marginal on the last stationary state, dropping the ground
    // keyframes and restarting the store from that prior.
    bool vio_gs_dm_delay_from_motion;
    // Standard deviations of the coarse optimisation's prior on its one
    // gyroscope and accelerometer bias, centred on the window's estimate,
    // 0 disables.
    double vio_gs_dm_coarse_bg_prior_std;
    double vio_gs_dm_coarse_ba_prior_std;
    // Fold the period once the cumulative scale and gravity direction have
    // stayed within the two thresholds over the last
    // vio_gs_dm_fold_settle_count marginalisations of the delayed phase, 0
    // disables.
    double vio_gs_dm_fold_settle_scale;
    double vio_gs_dm_fold_settle_tilt_deg;
    int vio_gs_dm_fold_settle_count;
    // Prior on the tilt of gravity relative to its initial estimate, on the
    // first state in the initial prior and on the block in every coarse
    // optimisation, degrees, 0 disables.
    double vio_gravity_prior_std_deg;

    bool vio_marg_lost_landmarks;
    double vio_kf_marg_feature_ratio;

    double mapper_obs_std_dev;
    double mapper_obs_huber_thresh;
    int mapper_detection_num_points;
    double mapper_num_frames_to_match;
    double mapper_frames_to_match_threshold;
    double mapper_min_matches;
    double mapper_ransac_threshold;
    double mapper_min_track_length;
    double mapper_max_hamming_distance;
    double mapper_second_best_test_ratio;
    int mapper_bow_num_bits;
    double mapper_min_triangulation_dist;
    bool mapper_no_factor_weights;
    bool mapper_use_factors;

    bool mapper_use_lm;
    double mapper_lm_lambda_min;
    double mapper_lm_lambda_max;

    size_t local_mapper_max_local_map_size;
    size_t local_mapper_min_local_map_size;
    size_t local_mapper_min_redundant_observers;
    double local_mapper_cull_redundancy_thresh;
    size_t local_mapper_min_observed_for_cull;
    size_t local_mapper_max_cull_per_pass;
    int local_mapper_opt_iterations;
    double local_mapper_filter_outlier_threshold;
};

}  // namespace basalt
