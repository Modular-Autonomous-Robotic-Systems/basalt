#pragma once

#include <Eigen/Dense>
#include <basalt/utils/sophus_utils.hpp>  // Eigen::aligned_vector
#include <memory>
#include <sophus/se3.hpp>
#include <vector>

namespace basalt {

// Immutable per-cycle snapshot of the local map, published by LocalMapper for
// the GUI. Mirrors VioVisualizationData; carries only value copies so the
// consumer never touches the mapper's live lmdb/frame_poses (cf. §4.1 G3).
struct LocalMapperVisualizationData {
    typedef std::shared_ptr<LocalMapperVisualizationData> Ptr;

    int64_t t_ns;

    // World-frame keyframe poses, copied from LocalMapper::frame_poses.
    Eigen::aligned_vector<Sophus::SE3d> keyframes;

    // World-frame triangulated landmarks, materialised by get_current_points().
    Eigen::aligned_vector<Eigen::Vector3d> points;
    std::vector<int> point_ids;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Ground-truth pose sample forwarded by Controller::TrackMonocular when the
// caller supplies it. Pangolin-free so Controller (libbasalt.so) can use it.
struct GtPose {
    int64_t t_ns;
    Sophus::SE3d T_w_i;
};

// Distinct rendering colours for the local map, kept separate from the VIO
// palette in vis_utils.h so map points read as a different layer (G1). The
// constants are const at namespace scope, giving them internal linkage, so
// including this header in several translation units raises no one-definition
// rule violation.
const uint8_t local_map_point_color[3]{255, 128, 0};  // orange
const uint8_t local_map_kf_color[3]{255, 128, 128};   // amber

}  // namespace basalt
