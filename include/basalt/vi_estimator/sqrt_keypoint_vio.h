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

#include <basalt/imu/preintegration.h>
#include <basalt/utils/logger.h>
#include <basalt/vi_estimator/delayed_marginalisation.h>
#include <basalt/vi_estimator/sqrt_ba_base.h>
#include <basalt/vi_estimator/vio_estimator.h>

#include <deque>
#include <future>
#include <mutex>
#include <thread>

namespace basalt {

template <class Scalar_>
class SqrtKeypointVioEstimator : public VioEstimatorBase<Scalar_>,
                                 public SqrtBundleAdjustmentBase<Scalar_> {
public:
    using Scalar = Scalar_;

    typedef std::shared_ptr<SqrtKeypointVioEstimator> Ptr;

    static const int N = 9;
    using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Vec4 = Eigen::Matrix<Scalar, 4, 1>;
    using VecN = Eigen::Matrix<Scalar, N, 1>;
    using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using MatN3 = Eigen::Matrix<Scalar, N, 3>;
    using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using SE3 = Sophus::SE3<Scalar>;

    using typename SqrtBundleAdjustmentBase<Scalar>::RelLinData;
    using typename SqrtBundleAdjustmentBase<Scalar>::AbsLinData;
    using typename SqrtBundleAdjustmentBase<Scalar>::Mat3;

    using BundleAdjustmentBase<Scalar>::computeError;
    using BundleAdjustmentBase<Scalar>::get_current_points;
    using BundleAdjustmentBase<Scalar>::computeDelta;
    using BundleAdjustmentBase<Scalar>::computeProjections;
    using BundleAdjustmentBase<Scalar>::triangulate;
    using BundleAdjustmentBase<Scalar>::backup;
    using BundleAdjustmentBase<Scalar>::restore;
    using BundleAdjustmentBase<Scalar>::getPoseStateWithLin;
    using BundleAdjustmentBase<Scalar>::computeModelCostChange;

    using SqrtBundleAdjustmentBase<Scalar>::linearizeHelper;
    using SqrtBundleAdjustmentBase<Scalar>::linearizeAbsHelper;
    using SqrtBundleAdjustmentBase<Scalar>::linearizeRel;
    using SqrtBundleAdjustmentBase<Scalar>::linearizeAbs;
    using SqrtBundleAdjustmentBase<Scalar>::updatePoints;
    using SqrtBundleAdjustmentBase<Scalar>::updatePointsAbs;
    using SqrtBundleAdjustmentBase<Scalar>::linearizeMargPrior;
    using SqrtBundleAdjustmentBase<Scalar>::computeMargPriorError;
    using SqrtBundleAdjustmentBase<Scalar>::computeMargPriorModelCostChange;
    using SqrtBundleAdjustmentBase<Scalar>::checkNullspace;
    using SqrtBundleAdjustmentBase<Scalar>::checkEigenvalues;

    SqrtKeypointVioEstimator(const Eigen::Vector3d& g,
                             const basalt::Calibration<double>& calib_,
                             const VioConfig& config,
                             bool useProducerConsumerArchitecure,
                             const Logger::Ptr& logger = nullptr);

    void initialize(int64_t t_ns, const Sophus::SE3d& T_w_i,
                    const Eigen::Vector3d& vel_w_i, const Eigen::Vector3d& bg,
                    const Eigen::Vector3d& ba) override;

    void initialize(const Eigen::Vector3d& bg,
                    const Eigen::Vector3d& ba) override;

    virtual ~SqrtKeypointVioEstimator() { maybe_join(); }

    inline void maybe_join() override {
        if (processing_thread) {
            processing_thread->join();
            processing_thread.reset();
        }
    }

    void addIMUToQueue(const ImuData<double>::Ptr& data) override;
    void addVisionToQueue(const OpticalFlowResult::Ptr& data) override;

    typename ImuData<Scalar>::Ptr popFromImuDataQueue();
    bool popFromImuDataQueueNonBlocking(typename ImuData<Scalar>::Ptr& data);

    typename PoseVelBiasState<Scalar>::Ptr measure(
        const OpticalFlowResult::Ptr& opt_flow_meas,
        const typename IntegratedImuMeasurement<Scalar>::Ptr& meas,
        double imuDrainSeconds, double processFrameSoFar);

    typename PoseVelBiasState<Scalar>::Ptr ProcessFrame(
        OpticalFlowResult::Ptr& curr_frame,
        std::optional<Sophus::SE3d> gtcw = std::nullopt);

    // int64_t propagate();
    // void addNewState(int64_t data_t_ns);

    void optimize_and_marg(const std::map<int64_t, int>& num_points_connected,
                           const std::unordered_set<KeypointId>& lost_landmaks);

    void PublishKeyframe();

    void marginalize(const std::map<int64_t, int>& num_points_connected,
                     const std::unordered_set<KeypointId>& lost_landmaks);
    void optimize();
    void LinearizeMargGS(const AbsOrderMap& aom,
                         const std::set<int64_t>& kfs_to_marg,
                         const std::unordered_set<KeypointId>& lost_landmaks,
                         int64_t last_state_to_marg, MatX& Q2Jp_curr,
                         VecX& Q2r_curr, MatX& Q2Jp_store, VecX& Q2r_store);
    void MarginalizeGS(MatX& Q2Jp_curr, VecX& Q2r_curr,
                       const std::set<int>& idx_to_keep,
                       const std::set<int>& idx_to_marg);
    void RecentrePriorsGS();
    void RecentreGravityScale();
    void FoldGravityScale();
    void StripGsFromMargData(MargData& m) const;
    MatX ComputeGaugeProjector(const AbsOrderMap& aom) const;
    void SetInitialPriorOrder(int64_t t_ns);
    void LogGravityScale(int swapEvent);
    void UpdateDelayedMarg(DelayedMargStep<Scalar>&& step);
    std::shared_ptr<DelayedMargJob<Scalar>> MakeDelayedJob(bool coarse) const;
    int InstallDelayedJob(DelayedMargJob<Scalar>& job);
    bool InstallCoarseJob(const DelayedMargJob<Scalar>& job);
    // Rows of the zero velocity and zero motion residuals of the stationary
    // states in aom. With leaving, only the zero motion factors whose first
    // state leaves the window.
    void StationaryRows(const AbsOrderMap& aom,
                        const std::set<int64_t>* leaving, MatX& J,
                        VecX& r) const;
    // Relative pose rows of every keyframe the mapper and the window share
    // against the oldest of them.
    void LocalMapRows(const AbsOrderMap& aom, MatX& J, VecX& r,
                      Scalar* maxTrans = nullptr,
                      Scalar* maxRot = nullptr) const;
    // Relative pose rows of every pair of keyframes the mapper and the window
    // share.
    void LocalMapRows2(const AbsOrderMap& aom, MatX& J, VecX& r,
                       Scalar* maxTrans = nullptr,
                       Scalar* maxRot = nullptr) const;
    // Timestamps of the keyframes the mapper and the window share, oldest
    // first, and their mapper poses carried to the latest epoch. Empty while
    // the term is off or the gate is closed.
    void GetLocalMapPoses(const AbsOrderMap& aom, std::vector<int64_t>& vIds,
                          Eigen::aligned_vector<Sophus::SE3d>& vPoses) const;
    // Agreement between the mapper's and the window's consecutive shared
    // keyframes, and the pull of the solve and of the local map rows on the
    // log scale in the first iteration of optimize.
    struct LocalMapDiagnostics {
        int mKeyframes = 0;
        // Log of the mapper's over the window's summed relative translations.
        double mScaleGap = 0;
        // Mean horizontal and vertical parts, in degrees, of the relative
        // rotation discrepancy expressed in the world.
        double mTiltGapDeg = 0, mYawGapDeg = 0;
        // Mean speeds implied by consecutive keyframes and the largest
        // difference between them on one segment.
        double mMapSpeed = 0, mWindowSpeed = 0, mMaxSpeedJump = 0;
        // Gradient and curvature on the log scale of the visual, inertial and
        // prior system, and of the local map rows, and the scale step solved.
        double mGradRest = 0, mHessRest = 0;
        double mGradLocalMap = 0, mHessLocalMap = 0;
        double mSigmaStep = 0;
    };
    void LocalMapGaps(const AbsOrderMap& aom,
                      LocalMapDiagnostics& diag) const;
    // Whitened rows of relative pose factors measured in the metric world,
    // with the scale of the gravity and scale block when it is active.
    void RelPoseFactorRows(const AbsOrderMap& aom,
                           const Eigen::aligned_vector<RelPoseFactor>& vFactors,
                           MatX& J, VecX& r, Scalar* maxTrans,
                           Scalar* maxRot) const;
    void StopDelayedMarg();
    // Replaces the prior by its marginal on the gravity and scale block and
    // the window state it holds, drops every keyframe pose with its landmarks
    // and restarts the delayed store from the new prior at nextIndex.
    void ResetPriorToLastState(int64_t nextIndex);

    void debug_finalize() override;

    void logMargNullspace();
    Eigen::VectorXd checkMargNullspace() const;
    Eigen::VectorXd checkMargEigenvalues() const;

    int64_t get_t_ns() const {
        return frame_states.at(last_state_t_ns).getState().t_ns;
    }
    const SE3& get_T_w_i() const {
        return frame_states.at(last_state_t_ns).getState().T_w_i;
    }
    const Vec3& get_vel_w_i() const {
        return frame_states.at(last_state_t_ns).getState().vel_w_i;
    }

    const PoseVelBiasState<Scalar>& get_state() const {
        return frame_states.at(last_state_t_ns).getState();
    }
    PoseVelBiasState<Scalar> get_state(int64_t t_ns) const {
        PoseVelBiasState<Scalar> state;

        auto it = frame_states.find(t_ns);

        if (it != frame_states.end()) {
            return it->second.getState();
        }

        auto it2 = frame_poses.find(t_ns);
        if (it2 != frame_poses.end()) {
            state.T_w_i = it2->second.getPose();
        }

        return state;
    }

    void setMaxStates(size_t val) override { max_states = val; }
    void setMaxKfs(size_t val) override { max_kfs = val; }

    Eigen::aligned_vector<SE3> getFrameStates() const {
        Eigen::aligned_vector<SE3> res;

        for (const auto& kv : frame_states) {
            res.push_back(kv.second.getState().T_w_i);
        }

        return res;
    }

    Eigen::aligned_vector<SE3> getFramePoses() const {
        Eigen::aligned_vector<SE3> res;

        for (const auto& kv : frame_poses) {
            res.push_back(kv.second.getPose());
        }

        return res;
    }

    Eigen::aligned_map<int64_t, SE3> getAllPosesMap() const {
        Eigen::aligned_map<int64_t, SE3> res;

        for (const auto& kv : frame_poses) {
            res[kv.first] = kv.second.getPose();
        }

        for (const auto& kv : frame_states) {
            res[kv.first] = kv.second.getState().T_w_i;
        }

        return res;
    }

    Sophus::SE3d getT_w_i_init() override {
        return T_w_i_init.template cast<double>();
    }

    // Accept refined keyframe poses from the local mapper. Called from the
    // mapper thread; the mutex ensures thread-safe handoff to the VIO thread.
    void QueuePoseUpdates(
        const Eigen::aligned_map<int64_t, PoseStateWithLin<double>>& updates,
        int64_t gsEpoch = 0) override {
        std::lock_guard<std::mutex> lock(mpPosesToUpdateMutex);
        for (const auto& kv : updates) {
            mpPosesToUpdate[kv.first] = kv.second;
            mpPosesToUpdateEpoch[kv.first] = gsEpoch;
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    using BundleAdjustmentBase<Scalar>::frame_poses;
    using BundleAdjustmentBase<Scalar>::frame_states;
    using BundleAdjustmentBase<Scalar>::lmdb;
    using BundleAdjustmentBase<Scalar>::obs_std_dev;
    using BundleAdjustmentBase<Scalar>::huber_thresh;
    using BundleAdjustmentBase<Scalar>::calib;

private:
    bool take_kf;
    // Raised where the keyframe is committed, cleared by PublishKeyframe.
    bool mpIsCurrentFrameKF = false;

    struct GsAnchor {
        Vec3 dir;
        Eigen::Matrix<Scalar, 3, 2> basis;
        Scalar sigma = 0;
        bool set = false;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    void AnchorGs(GsAnchor& anchor);

    bool mpGsActive = false;
    int64_t mpGsStartNs = -1;
    GsAnchor mpAnchorCurr;
    int mpGsSwapEvent = 0;
    Scalar mpGsStepSigma = 0;
    Scalar mpGsStepTiltDeg = 0;
    Scalar mpGsJumpCost = 0;
    int mpGsVisualImuFull = 0;
    // Delayed marginalisation, plans/delayed_marginalisation.md. mpDm is the
    // store, kept for the whole period. The block is held in the window while
    // mpDmCoarse is set, until the store first holds vio_gs_dm_delay steps.
    std::unique_ptr<DelayedMarginalisation<Scalar>> mpDm;
    std::future<std::shared_ptr<DelayedMargJob<Scalar>>> mpDmFuture;
    bool mpDmCoarse = true;
    bool mpDmForceReplace = false;
    int64_t mpDmLastCoarseKeyframe = -1;
    double mpDmStoreSeconds = 0;
    // Steps pushed into the store before the period's first moving frame.
    int64_t mpDmMotionIndex = 0;
    // Set by a moving frame that follows a stationary one, so that the next
    // marginalisation resets the prior to the last stationary state.
    bool mpDmResetPending = false;
    bool mpWasStationary = false;
    // Epochs of the last vio_gs_dm_fold_settle_count + 1 marginalisations of
    // the delayed phase, and whether they satisfy the fold test.
    std::deque<int64_t> mvpGsFoldEpochs;
    bool mpGsFoldSettled = false;
    int mpGsVisualImuRot = 0;
    Scalar mpGsVisualImuScaleCol = 0;
    int frames_after_kf;
    std::set<int64_t> kf_ids;

    int64_t last_state_t_ns;
    Eigen::aligned_map<int64_t, IntegratedImuMeasurement<Scalar>> imu_meas;

    const Vec3 g;

    // Input

    Eigen::aligned_map<int64_t, OpticalFlowResult::Ptr> prev_opt_flow_res;

    std::map<int64_t, int> num_points_kf;

    // Marginalization
    MargLinData<Scalar> marg_data;

    // Used only for debug and log purporses.
    MargLinData<Scalar> nullspace_marg_data;

    Vec3 gyro_bias_sqrt_weight, accel_bias_sqrt_weight;

    size_t max_states;
    size_t max_kfs;

    SE3 T_w_i_init;
    std::optional<Sophus::SE3d> mpCurrentGTPose;

    bool initialized;
    bool opt_started;

    VioConfig config;

    constexpr static Scalar vee_factor = Scalar(2.0);
    constexpr static Scalar initial_vee = Scalar(2.0);
    Scalar lambda, min_lambda, max_lambda, lambda_vee;

    std::shared_ptr<std::thread> processing_thread;

    Logger::Ptr mpLogger;
    bool mpUseProducerConsumerArchitecture = false;
    // Superseded 2026-10-19, replaced by the three-stage detector below.
    // Scalar mpStationaryAccelTol = Scalar(0.5);
    // Scalar mpStationaryGyroTol = Scalar(0.05);
    bool mpIsStationary = false;
    // Frame states created while the vehicle was stationary.
    std::set<int64_t> mvpStationaryStates;
    Scalar mpZeroVelocityWeight = Scalar(0);

    struct ImuSample {
        int64_t t_ns;
        Vec3 accel, gyro;
    };
    std::deque<ImuSample> mvpStationaryWindow;
    int mpStationaryRunLength = 0;

    bool mpStationaryEnableShoe = true;
    bool mpStationaryEnableMahalanobis = true;
    bool mpStationaryEnableFlow = false;
    Scalar mpStationaryWindowS = Scalar(0.5);
    Scalar mpStationaryGlrtTol = Scalar(6.6);
    int mpStationaryMinConsecutiveFrames = 2;
    Scalar mpStationaryChiSqTol = Scalar(7.81);
    double mpStationaryFlowTol = 0.8;
    bool mpStationaryEnableStartLatch = false;
    int mpStationaryLatchFrames = 3;
    int mpStationaryLatchMisses = 0;
    bool mpStationaryLatchOpen = true;
    typename ImuData<Scalar>::Ptr imuData;
    // Deadline for the blocking pop, so a stack brought up without an inertial
    // publisher fails visibly instead of hanging.
    int64_t mpImuPopTimeoutMs = 5000;
    static constexpr int64_t mpImuPopRetryMs = 1;
    Vec3 mpBg, mpBa;
    Vec3 mpAccelCov, mpGyroCov;

    // ── Local-mapper pose feedback ──────────────────────────────────
    // Staging map written by LocalMapper (via QueuePoseUpdates callback)
    // and consumed at the top of measure(). Hardcoded to double because
    // the LocalMapper always operates in double precision.
    Eigen::aligned_unordered_map<int64_t, PoseStateWithLin<double>>
        mpPosesToUpdate;
    std::unordered_map<int64_t, int64_t> mpPosesToUpdateEpoch;
    mutable std::mutex mpPosesToUpdateMutex;

    // Latest mapper pose per keyframe timestamp and the epoch it is expressed
    // in, owned by the estimator thread.
    std::map<int64_t, std::pair<Sophus::SE3d, int64_t>> mvpLocalMapPoses;
    bool mpLocalMapOpen = false;
    // Pair count and largest unweighted residuals of the last optimize.
    int mpLocalMapPairs = 0;
    Scalar mpLocalMapMaxTrans = 0, mpLocalMapMaxRot = 0;
    LocalMapDiagnostics mpLocalMapDiag;
};
}  // namespace basalt
