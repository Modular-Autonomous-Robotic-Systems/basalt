#pragma once

#include <tbb/concurrent_queue.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

#include "basalt/hash_bow/hash_bow.h"
#include "basalt/utils/tracks.h"
#include "basalt/vi_estimator/nfr_mapper.h"
#include "basalt/visualisation/utils.h"  // LocalMapperVisualizationData (Pangolin-free)

namespace basalt {

class LocalMapper : public basalt::NfrMapper {
public:
    using Scalar = double;
    using Ptr = std::shared_ptr<LocalMapper>;
    using PoseUpdateCallback = std::function<void(
        const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>&)>;

    LocalMapper(const Calibration<double>& calib, const VioConfig& config);
    ~LocalMapper();

    // ── Lifecycle ───────────────────────────────────────────────────
    void Initialise();
    void Stop();
    void SetMarginalisationDataInputQueue(
        tbb::concurrent_bounded_queue<MargData::Ptr>* queue);
    void SetVIOPoseUpdateCallback(PoseUpdateCallback cb);

    // ── Pipeline (public for testability) ───────────────────────────
    void MapLocally();  // thread entry point
    void IngestMargData(MargData::Ptr& data);
    void MatchLocal();
    void build_tracks();  // shadows NfrMapper
    void setup_opt();     // shadows NfrMapper

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void CullRedundantKeyframes();
    size_t ComputeCovisibility(int64_t tid_a, int64_t tid_b, int num_cameras);
    void CollectNewKeyframesAfterMatching();

    // ── Visualisation publish hook ──────────────────────────────────
    // A null pointer means the mapper assembles no snapshot, so the GUI tap
    // costs nothing. This mirrors the contract of
    // VioEstimatorBase::out_vis_queue exactly. The name is kept un-prefixed for
    // API symmetry with the VIO hook.
    tbb::concurrent_bounded_queue<LocalMapperVisualizationData::Ptr>*
        out_vis_queue = nullptr;

    // ── Config ──────────────────────────────────────────────────────
    size_t mpMaxLocalMapSize = 50;
    double mpCullCovisibilityThresh = 0.5;
    int mpOptIterations = 5;
    double mpFilterOutlierThreshold = 3.0;

    // ── State exposed for tests and introspection ───────────────────
    std::set<int64_t> mpNewKeyframesForTracking;

    // Stores std::shared_ptr<MatchData> (mirroring the Matches typedef) so
    // copies between feature_matches and this map are pointer-copies, and so
    // the over-aligned Sophus::SE3d inside MatchData is never embedded inside
    // an STL node either.
    std::unordered_map<std::pair<TimeCamId, TimeCamId>,
                       std::shared_ptr<MatchData>,
                       std::hash<std::pair<TimeCamId, TimeCamId>>>
        mpLatestKeyframesMatches;

    TrackBuilder mpTrackBuilder;
    bool mpIsTrackBuilderInitialised = false;
    std::set<TrackId> mpRetiredTrackIds;

    std::atomic<bool> mpStopLocalMapping{false};
    std::atomic<bool> mpIsMargDataInputQueueSet{false};
    std::thread mpLocalMappingThread;

private:
    tbb::concurrent_bounded_queue<MargData::Ptr>* mpMargInputQueue = nullptr;
    PoseUpdateCallback mpVioPoseUpdateCallback;

    // Helpers
    bool SelectKeyframesToCull(std::vector<int64_t>& keyframesToCull);
    int64_t FindBestRehostKf(int64_t culled_kf, TrackId lm_id,
                             const std::set<int64_t>& candidates);
    void RehostLandmark(TrackId lm_id, int64_t culled_kf, int64_t new_host_kf);
};

}  // namespace basalt
