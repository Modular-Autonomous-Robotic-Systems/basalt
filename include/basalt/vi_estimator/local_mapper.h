#pragma once

#include "basalt/vi_estimator/nfr_mapper.h"
#include "basalt/utils/tracks.h"
#include "basalt/hash_bow/hash_bow.h"

#include <atomic>
#include <functional>
#include <mutex>
#include <set>
#include <thread>

#include <tbb/concurrent_queue.h>

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
  void build_tracks();   // shadows NfrMapper
  void setup_opt();      // shadows NfrMapper
  void CullRedundantKeyframes();
  size_t ComputeCovisibility(int64_t tid_a, int64_t tid_b, int num_cameras);
  void CollectNewKeyframesAfterMatching();

  // ── Config ──────────────────────────────────────────────────────
  size_t mpMaxLocalMapSize = 50;
  double mpCullCovisibilityThresh = 0.90;
  int mpOptIterations = 5;
  double mpFilterOutlierThreshold = 3.0;

  // ── State exposed for tests and introspection ───────────────────
  std::set<int64_t> mpNewKeyframesForTracking;

  std::unordered_map<
      std::pair<TimeCamId, TimeCamId>, MatchData,
      std::hash<std::pair<TimeCamId, TimeCamId>>,
      std::equal_to<std::pair<TimeCamId, TimeCamId>>,
      Eigen::aligned_allocator<
          std::pair<const std::pair<TimeCamId, TimeCamId>, MatchData>>>
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
  int64_t SelectKeyframeToCull();
  int64_t FindBestRehostKf(int64_t culled_kf, TrackId lm_id,
                           const std::set<int64_t>& candidates);
  void RehostLandmark(TrackId lm_id, int64_t culled_kf, int64_t new_host_kf);
};

}  // namespace basalt
