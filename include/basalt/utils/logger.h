#pragma once

#include <basalt/utils/time_utils.hpp>

#include <atomic>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace basalt {

/// One table of diagnostic measurements. Columns are declared once through
/// RegisterStats, rows are appended through the inherited add methods and
/// closed with CommitRow, and the two sinks are a rendered line and a comma
/// separated file.
class AggregateStats : public ExecutionStats {
public:
    using Registry = std::map<std::string, std::string>;

    AggregateStats() = default;
    explicit AggregateStats(std::string name);
    ~AggregateStats() override;

    AggregateStats(const AggregateStats&) = delete;
    AggregateStats& operator=(const AggregateStats&) = delete;

    void RegisterStats(const Registry& registry);
    void SetPrefixForLogging(const std::string& prefix);

    Meta& add(const std::string& name, double value) override;
    Meta& add(const std::string& name, const Eigen::VectorXd& value) override;
    Meta& add(const std::string& name, const Eigen::VectorXf& value) override;
    Meta& add_int(const std::string& name, int64_t value) override;

    std::string PrintLatest() const;
    std::string PrintLatest(const std::string& prefix) const;

    bool SaveCsv(const std::string& path) const;

    bool OpenStream(const std::string& path);
    void CommitRow();
    void CloseStream();

    void SetRetainedRows(size_t rows);
    /// Writes an empty vector as blank cells instead of zeros.
    void SetBlankEmptyVectors(bool blank);
    void SetRowCallback(std::function<void(const AggregateStats&)> cb);

    const std::string& Name() const;
    size_t RowCount() const;

private:
    std::string FormatFor(const std::string& name) const;
    std::string RenderValue(const Meta& meta) const;
    std::vector<std::string> ColumnNames() const;
    void WriteHeaderLocked();
    void WriteRowLocked();
    void TrimLocked();

    Registry mpRegistry;
    std::string mpPrefix;
    std::string mpName;

    std::ofstream mpStream;
    bool mpHeaderWritten = false;
    size_t mpRowsWrittenToFile = 0;
    size_t mpRowCount = 0;
    size_t mpRetainedRows = 0;  // zero keeps every row in memory
    bool mpBlankEmptyVectors = false;
    static constexpr size_t kFlushPeriod = 32;

    std::function<void(const AggregateStats&)> mpRowCallback;
    mutable std::mutex mpMtxRows;
};

/// Per frame counters for the frame to frame tracker. The five atomics are
/// accumulated from inside a tbb::parallel_for in trackPoints, the rest are
/// written once on the calling thread.
struct OpticalFlowTrackStats {
    std::atomic<int> mAttempted{0};
    std::atomic<int> mForwardFailed{0};
    std::atomic<int> mBackwardFailed{0};
    std::atomic<int> mRecoveryRejected{0};
    std::atomic<int> mTracked{0};

    int mPointsBefore = 0;
    int mDetected = 0;
    int mEpipolarRejected = 0;
    int mOut = 0;
    double mFlowSum = 0.0;
    double mFlowMax = 0.0;
    double mPyramidSeconds = 0.0;
    double mTrackSeconds = 0.0;
    double mDetectSeconds = 0.0;
    double mTotalSeconds = 0.0;
    double mDtSeconds = 0.0;
};

/// Owns one AggregateStats per class of diagnostic record and exposes one
/// named recording method and one named rendering method per class. The
/// constructor registers every column and format, so a call site never
/// repeats one.
class Logger {
public:
    using Ptr = std::shared_ptr<Logger>;

    /// outputDirectory empty disables the comma separated sink and leaves the
    /// console sink alone, which reproduces today's behaviour exactly.
    explicit Logger(const std::string& outputDirectory = std::string(),
                    bool consoleEnabled = true);
    ~Logger();

    /// Shared null object. Every Add and Print method returns immediately, so
    /// a component constructed without a logger needs no null guard.
    static Ptr Disabled();

    bool IsEnabled() const { return mpEnabled; }
    bool IsConsoleEnabled() const { return mpEnabled && mpConsoleEnabled; }

    // ── Optical flow ────────────────────────────────────────────────
    void AddOpticalFlowFrame(int64_t tNs, int64_t frame, double dt, int in,
                             int attempted, int tracked, int fwdFail,
                             int bwdFail, int recovRej, int detected,
                             int epiRej, int out, double flowMean,
                             double flowMax);
    void PrintOpticalFlowFrame();
    void AddOpticalFlowTiming(int64_t tNs, double pyramid, double track,
                              double detectAdd, double total, int queue);
    void PrintOpticalFlowTiming();

    // ── Inertial ingestion ──────────────────────────────────────────
    void AddVioImuIngest(int64_t tNs, int64_t n, double stampRateHz,
                         double wallRateHz, double maxGap,
                         int64_t nonMonotonic, int queue,
                         const Eigen::Vector3d& accel,
                         const Eigen::Vector3d& gyro);
    void PrintVioImuIngest();

    // ── Estimator ───────────────────────────────────────────────────
    void AddVioInit(int64_t tNs, int64_t imuTNs, int64_t imuQueue,
                    int64_t skippedBehind, const Eigen::Vector3d& accel,
                    const Eigen::Vector3d& gyro, const Eigen::Vector3d& bg,
                    const Eigen::Vector3d& ba, const Eigen::Vector3d& g,
                    const Eigen::Vector4d& qWI, double tiltDeg);
    void PrintVioInit();

    void AddVioImuFrame(int64_t tNs, double frameDt, int integrated,
                        int expected, int skippedBehind, int64_t firstImuNs,
                        int64_t lastImuNs, double coverage, double measDt,
                        bool stretch, double stretchSpan, int imuQueueIn,
                        int imuQueueOut, double headMinusFrame);
    void PrintVioImuFrame();

    void AddVioPoseUpdate(int64_t tNs, int pending, int applied, int rejected,
                         double maxTransErr, double maxRotErr,
                         double limitTrans, double limitRot);
    void PrintVioPoseUpdate();

    void AddVioAssoc(int64_t tNs, int ofObs, int connected0, int unconnected0,
                     double thresh, int framesAfterKf, int lmdbLandmarks,
                     int kfs, int frameStates, int framePoses, bool takeKf);
    void PrintVioAssoc();

    void AddVioTriang(int64_t tNs, int candidates, int added, int noPriorObs,
                      int unprojectFail, int shortBaseline, int notFinite,
                      int behind, int tooClose, double minTriangDist,
                      double baselineMin, double baselineMax,
                      double depthMin, double depthSum, double depthMax);
    void PrintVioTriang();

    void AddVioLandmarks(int64_t tNs, int lost, int lmdb, bool margLost);
    void PrintVioLandmarks();

    // An empty vector marks an absent pose.
    void AddGtEval(int64_t tNs, const Eigen::VectorXd& pWI,
                   const Eigen::VectorXd& qWI, const Eigen::VectorXd& pGt,
                   const Eigen::VectorXd& qGt);

    void AddVioState(int64_t tNs, const Eigen::Vector3d& pWI,
                     const Eigen::Vector4d& qWI, const Eigen::Vector3d& vWI,
                     const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);
    void PrintVioState();

    void AddVioTiming(int64_t tNs, double poseUpdate, double association,
                      double triangulation, double lostScan,
                      double optAndMarg, double publish, double measure,
                      double imuDrain, double processFrame, int visionQueue,
                      int imuQueue);
    void PrintVioTiming();

    void AddVioLinearise(int64_t tNs, int iter, double errorTotal,
                        double lambda, int landmarks, int states, int poses,
                        bool numericallyValid);
    void PrintVioLinearise();

    void AddVioSolverIter(int64_t tNs, int iter, int inner, int status,
                         double errorTotal, double fDiff, double lDiff,
                         double stepQuality, double stepSize, double vision,
                         double imu, double biasG, double biasA,
                         double margPrior, double lambda, double itTime,
                         double totalTime);
    void PrintVioSolverIter();

    void AddVioSolverSummary(int64_t tNs, int numIt, int numItRejected,
                            bool converged, bool terminated, int reason,
                            double optimize);
    void PrintVioSolverSummary();

    // Mirrors the keys the local ExecutionStats scratch object in
    // SqrtKeypointVioEstimator::optimize holds, so the comma separated file
    // and stats_all.json / stats_sums.json carry the same numbers.
    void AddVioSolverTiming(int numCams, int numLms, int numObs,
                           double allocateLmb, double linearizeProblem,
                           double performQr, double getDenseHB, double solve,
                           double backSubstitute, double computeError2,
                           double iteration, double residentMemory,
                           double residentMemoryPeak);
    void PrintVioSolverTiming();

    void AddVioMarg(int64_t tNs, int statesToRemove, int posesToMarg,
                    int statesToMarg, int statesToMargVelBias,
                    int kfsToMarg, int kfIds, int keeping, int marg,
                    int total, int framePoses, int frameStates,
                    int64_t lastStateToMarg);
    void PrintVioMarg();

    void AddVioMargNullspace(int64_t tNs, const Eigen::VectorXd& margNs,
                            double evMin, double evMax, int evNegative,
                            double evCondition);
    void PrintVioMargNullspace();

    void AddVioMargTiming(int64_t frameId, double measure,
                         double margLinearize, double margHelper,
                         double marg, double margLog, double marginalize);
    void PrintVioMargTiming();

    // ── Local mapper ────────────────────────────────────────────────
    void AddMapCycleTiming(int64_t tNs, int keyframes, int margPackets,
                          double queueWait, double ingest,
                          double detectKeypoints, double matchStereo,
                          double matchLocal, double collectNewKfs,
                          double buildTracks, double setupOpt, double cull,
                          double optimize1, double filterOutliers,
                          double optimize2, double publishVis,
                          double poseUpdateCb, double total);
    void PrintMapCycleTiming();

    void AddMapBowQuery(int64_t tNs, int64_t kf, int keypoints,
                       int descriptors, int bowBuckets, int dbHits,
                       int aboveThresh, double bestScore, double thresh);
    void PrintMapBowQuery();

    void AddMapPairMatch(int64_t kf1, int64_t kf2, int descriptorMatches,
                        int minMatches, int ransacInliers, bool stored);
    void PrintMapPairMatch();

    void AddMapMatchSummary(int64_t tNs, int pairs, int neighbourPairs,
                           int neighbourLimit, int verificationAttempts,
                           int inlierMatches, int totalMatches,
                           double dbQuery, double matching);
    void PrintMapMatchSummary();

    void AddMapTracks(int64_t tNs, int exported, int liveInBuilder,
                     int matches, int newKfs, int corners,
                     const Eigen::VectorXd& lenHist);
    void PrintMapTracks();

    void AddMapSetupOpt(int64_t tNs, int tracks, int shortTrack, int known,
                       int retired, int newOk, int noTriang, int noCorners,
                       int noHostPose, int noObsPose, int shortBaseline,
                       int lowParallax, int badDepth, int behind,
                       int tooClose, int nan, int obsAdded, int lmdbBefore,
                       int lmdbAfter, double minTriangDist,
                       double maxCosParallax, double maxInvDist);
    void PrintMapSetupOpt();

    void AddMapCovis(int64_t tNs, bool parity, int frames, int refFrames,
                    int cells, int refCells);
    void PrintMapCovis();

    void AddMapCullSelect(int64_t tNs, int map, int eligible, int keepRecent,
                         int maxCull, int minObservers, int selected,
                         bool capacityRule, double thresh,
                         const Eigen::VectorXd& observersHist);
    void PrintMapCullSelect();

    void AddMapRehost(int64_t tNs, int64_t lm, int64_t culledHost,
                     int64_t newHost, int obsBefore, int obsReadded,
                     int outcome, int reason);
    void PrintMapRehost();

    void AddMapCull(int64_t tNs, int64_t kf, int victims, int hostedLms,
                   int rehostAttempted, int noHost, int lmBefore,
                   int lmAfter, int mapBefore, int mapAfter);
    void PrintMapCull();

    void AddMapFilter(int64_t tNs, int lmBefore, int lmAfter, int minObs,
                     double thresh);
    void PrintMapFilter();

    void AddMapPublish(int64_t tNs, int points, int keyframes,
                      int lmdbLandmarks, int hostKfs);
    void PrintMapPublish();

    // ── Shared mapping base ─────────────────────────────────────────
    void AddMapperLinearise(int iter, double vision, double relError,
                           double rollPitchError, double total,
                           int landmarks, int observations,
                           int relPoseFactors, int rollPitchFactors);
    void PrintMapperLinearise();

    void AddMapperSolverIter(int iter, int inner, int status, double lambda,
                            double fDiff, double maxInc, double visionError,
                            double relError, double rollPitchError,
                            double total, bool converged,
                            bool errorIncreased);
    void PrintMapperSolverIter();

    void AddMapperIterSummary(int iter, double iteration, int numStates,
                             int numPoses, int innerSteps);
    void PrintMapperIterSummary();

    void AddMapperDetect(int frames, double detection, int cornersTotal);
    void PrintMapperDetect();

    void AddMapperStereo(int pairs, int matches, int inliers);
    void PrintMapperStereo();

    // ── Legacy ExecutionStats artefacts, preserved for the batch tools ──
    ExecutionStats& SolverScratch();
    void FinishVioOptimize();
    void SaveLegacyStats();

    void PrintSummary();
    void SaveAll();

    AggregateStats& Aggregate(const std::string& name);

private:
    void Register(const std::string& name, const std::string& prefix,
                  const AggregateStats::Registry& registry);
    void Emit(const std::string& name);

    bool mpEnabled = false;
    bool mpConsoleEnabled = true;
    std::string mpOutputDirectory;
    std::map<std::string, AggregateStats> mpAggregates;

    ExecutionStats mpStatsAll;
    ExecutionStats mpStatsSums;
    ExecutionStats mpSolverScratch;

    std::mutex mpMtxConsole;
};

}  // namespace basalt
