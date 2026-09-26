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

    LocalMapper(const Calibration<double>& calib, const VioConfig& config,
                const Logger::Ptr& logger = nullptr);
    ~LocalMapper();

    // ── Lifecycle ───────────────────────────────────────────────────
    void Initialise();
    void Stop();
    void SetMarginalisationDataInputQueue(
        tbb::concurrent_bounded_queue<MargData::Ptr>* queue);
    void SetKFInputQueue(tbb::concurrent_bounded_queue<Keyframe::Ptr>* queue);
    void SetVIOPoseUpdateCallback(PoseUpdateCallback cb);

    // ── Pipeline (public for testability) ───────────────────────────
    void MapLocally();  // thread entry point
    void IngestKeyframe(Keyframe::Ptr& kf);
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

    // The eight members up to mpFilterOutlierThreshold carry no initialiser
    // because LocalMapper::LocalMapper assigns each from the local_mapper_*
    // field of the VioConfig it is handed. VioConfig::VioConfig holds the
    // defaults.
    size_t mpMaxLocalMapSize;
    size_t mpMinLocalMapSize;
    size_t mpMinRedundantObservers;  // ORB-SLAM3 thObs
    double mpCullRedundancyThresh;   // ORB-SLAM3 redundant_th, monocular
    size_t mpMinObservedForCull;     // anchors below this are exempt
    size_t mpMaxCullPerPass;         // bound the blast radius
    int mpOptIterations;
    double mpFilterOutlierThreshold;
    int mpFilterMinObs = 2;
    double mpMaxInvDist = 2.0;
    double mpMaxCosParallax = 0.9998;
    size_t mpLocalMatchNeighbours = 5;

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
    std::atomic<bool> mpIsKFInputQueueSet{false};
    std::thread mpLocalMappingThread;

private:
    tbb::concurrent_bounded_queue<MargData::Ptr>* mpMargInputQueue = nullptr;
    tbb::concurrent_bounded_queue<Keyframe::Ptr>* mpKFInputQueue = nullptr;
    PoseUpdateCallback mpVioPoseUpdateCallback;

    // Helpers
    void PruneFactorsWithUnknownKeyframes(size_t relBegin, size_t rpBegin);
    bool SelectKeyframesToCull(std::vector<int64_t>& keyframesToCull);

    // Deprected
    ObservedByFrameMap BuildObservedSets() const;

    // Deprecated
    CovisMatrix BuildCovisibilityMatrix(const ObservedByFrameMap& seen) const;

    double RedundancyScore(int64_t kf, const ObservedByFrameMap& seen,
                           const std::set<int64_t>& alive,
                           size_t& nObservedOut) const;
    int64_t FindBestRehostKf(int64_t culled_kf, TrackId lm_id,
                             const std::set<int64_t>& candidates);
    void RehostLandmark(TrackId lm_id, int64_t culled_kf, int64_t new_host_kf);
};

}  // namespace basalt
