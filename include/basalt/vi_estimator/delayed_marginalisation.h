#pragma once

#include <basalt/linearization/imu_block.hpp>
#include <basalt/utils/imu_types.h>
#include <basalt/vi_estimator/marg_helper.h>

#include <Eigen/Sparse>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

// Delayed marginalisation of the gravity and scale period after DM-VIO,
// plans/delayed_marginalisation.md.

namespace basalt {

template <class Scalar_>
struct DelayedMargStep;

// Type abbreviations of this file. Basalt has no package wide abbreviations
// of Scalar templated types, so they are scoped to this file's namespace.
namespace dm {
template <class Scalar>
using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
template <class Scalar>
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
template <class Scalar>
using Vec6 = Eigen::Matrix<Scalar, 6, 1>;
template <class Scalar>
using Vec15 = Eigen::Matrix<Scalar, POSE_VEL_BIAS_SIZE, 1>;
template <class Scalar>
using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
template <class Scalar>
using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
template <class Scalar>
using Mat6 = Eigen::Matrix<Scalar, 6, 6>;
template <class Scalar>
using Mat32 = Eigen::Matrix<Scalar, 3, 2>;
template <class Scalar>
using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
template <class Scalar>
using Vec9 = Eigen::Matrix<Scalar, 9, 1>;
template <class Scalar>
using Mat9 = Eigen::Matrix<Scalar, 9, 9>;
template <class Scalar>
using Mat93 = Eigen::Matrix<Scalar, 9, 3>;
template <class Scalar>
using SpMat = Eigen::SparseMatrix<Scalar>;
template <class Scalar>
using SO3 = Sophus::SO3<Scalar>;
template <class Scalar>
using StepPtr = std::shared_ptr<const DelayedMargStep<Scalar>>;
}  // namespace dm

// Linearisation point of the gravity and scale block of a stored prior.
template <class Scalar_>
struct DelayedMargAnchor {
    using Scalar = Scalar_;

    dm::Vec3<Scalar> mDir = dm::Vec3<Scalar>(0, 0, -1);
    dm::Mat32<Scalar> mBasis =
        (dm::Mat32<Scalar>() << 1, 0, 0, 1, 0, 0).finished();
    Scalar mSigma = 0;

    void Set(const GravityScaleStateWithLin<Scalar>& gs) {
        mDir = gs.GetDir();
        mBasis = gs.GetBasis();
        mSigma = gs.GetSigma();
    }

    void Transform(const dm::SO3<Scalar>& Q, Scalar sigmaShift) {
        mDir = Q * mDir;
        mBasis = Q.matrix() * mBasis;
        mSigma -= sigmaShift;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Square root prior 0.5 |H (delta + x) + b|^2 in the offsets delta of its
// variables from their linearisation points, with its block anchor.
template <class Scalar_>
struct DelayedMargPrior {
    using Scalar = Scalar_;

    dm::MatX<Scalar> mH;
    dm::VecX<Scalar> mB;
    AbsOrderMap mOrder;
    DelayedMargAnchor<Scalar> mAnchor;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// One marginalisation of the window. Immutable once stored, so that a
// readvance in another thread can share it.
template <class Scalar_>
struct DelayedMargStep {
    using Scalar = Scalar_;

    int64_t mIndex = 0;
    int64_t mEpoch = 0;
    AbsOrderMap mOrder;
    AbsOrderMap mOrderNew;
    std::set<int> mIdxKeep;
    std::set<int> mIdxMarg;
    // Landmark eliminated visual rows with a zero block column, in the
    // epoch mEpoch, delta independent.
    dm::MatX<Scalar> mVisJ;
    dm::VecX<Scalar> mVisR;
    Eigen::aligned_vector<IntegratedImuMeasurement<Scalar>> mvImu;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Value of a variable after some of its coordinates left the window, with
// its first estimate and offset. mLastStep is the step at which its last
// coordinates left.
template <class Scalar_>
struct DelayedMargRecord {
    PoseVelBiasStateWithLin<Scalar_> mState;
    int64_t mLastStep = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// First estimates and offsets of every variable a readvance can meet,
// expressed in one epoch.
template <class Scalar_>
struct DelayedMargValues {
    using Scalar = Scalar_;

    Eigen::aligned_map<int64_t, DelayedMargRecord<Scalar>> mvRecords;
    Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<Scalar>> mvWindowStates;
    Eigen::aligned_map<int64_t, PoseStateWithLin<Scalar>> mvWindowPoses;

    // Full state of a variable, its pose taken from the window while the
    // pose is still there.
    PoseVelBiasStateWithLin<Scalar> State(int64_t id) const {
        const auto it_s = mvWindowStates.find(id);
        if (it_s != mvWindowStates.end()) return it_s->second;

        const PoseVelBiasStateWithLin<Scalar>& rec = mvRecords.at(id).mState;
        const auto it_p = mvWindowPoses.find(id);
        if (it_p == mvWindowPoses.end()) return rec;

        const PoseVelBiasState<Scalar>& lin = rec.getStateLin();
        PoseVelBiasStateWithLin<Scalar> st(id, it_p->second.getPoseLin(),
                                           lin.vel_w_i, lin.bias_gyro,
                                           lin.bias_accel, true);
        typename PoseVelBiasStateWithLin<Scalar>::VecN delta = rec.getDelta();
        delta.template head<POSE_SIZE>() = it_p->second.getDelta();
        st.applyInc(delta);
        return st;
    }

    // Offsets of the variables of an order from their first estimates, with
    // blockDelta at the block.
    dm::VecX<Scalar> Delta(const AbsOrderMap& order,
                           const dm::Vec3<Scalar>& blockDelta) const {
        dm::VecX<Scalar> delta = dm::VecX<Scalar>::Zero(order.total_size);
        for (const auto& [id, idx] : order.abs_order_map) {
            if (id == kGravityScaleStateId) {
                delta.template segment<GRAVITY_SCALE_SIZE>(idx.first) =
                    blockDelta;
            } else if (idx.second == POSE_SIZE) {
                const auto it_p = mvWindowPoses.find(id);
                delta.template segment<POSE_SIZE>(idx.first) =
                    it_p != mvWindowPoses.end()
                        ? it_p->second.getDelta()
                        : mvRecords.at(id)
                              .mState.getDelta()
                              .template head<POSE_SIZE>();
            } else {
                delta.template segment<POSE_VEL_BIAS_SIZE>(idx.first) =
                    State(id).getDelta();
            }
        }
        return delta;
    }

    // Adds x to the state that holds the velocity and biases of id, the
    // window state while it is there and the record otherwise.
    void ApplyInc(int64_t id, const dm::Vec15<Scalar>& x) {
        const auto it_s = mvWindowStates.find(id);
        if (it_s != mvWindowStates.end()) {
            it_s->second.applyInc(x);
            return;
        }
        const auto it_r = mvRecords.find(id);
        if (it_r != mvRecords.end()) it_r->second.mState.applyInc(x);
    }

    void TransformBySim3(const dm::SO3<Scalar>& Q, Scalar s) {
        for (auto& [id, rec] : mvRecords) rec.mState.TransformBySim3(Q, s);
        for (auto& [id, st] : mvWindowStates) st.TransformBySim3(Q, s);
        for (auto& [id, ps] : mvWindowPoses) ps.TransformBySim3(Q, s);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Columns of a delta independent prior after the re-centring (Q, s), as
// RecentreGravityScale maps marg_data, plans/gravity_scale_optimisation.md
// §2.7.3.
template <class Scalar, class Derived>
void DelayedMargTransformColumns(Eigen::MatrixBase<Derived>& H,
                                 const AbsOrderMap& order,
                                 const dm::SO3<Scalar>& Q, Scalar s) {
    const dm::Mat3<Scalar> Q_inv_m = Q.inverse().matrix();
    for (const auto& [id, idx] : order.abs_order_map) {
        if (id == kGravityScaleStateId) continue;
        const int col = idx.first;
        H.middleCols(col, 3) = H.middleCols(col, 3) * Q_inv_m / s;
        H.middleCols(col + 3, 3) = H.middleCols(col + 3, 3) * Q_inv_m;
        if (idx.second == POSE_VEL_BIAS_SIZE) {
            H.middleCols(col + 6, 3) = H.middleCols(col + 6, 3) * Q_inv_m / s;
        }
    }
}

// Moves the block to the gravity direction dir and log scale sigma.
template <class Scalar>
void DelayedMargSetBlock(GravityScaleStateWithLin<Scalar>& gs,
                         const dm::Vec3<Scalar>& dir, Scalar sigma) {
    gs.Reanchor(gs.GetDir(), gs.GetBasis(), gs.GetSigma());
    const dm::Vec3<Scalar> axis = gs.GetDir().cross(dir);
    const Scalar sinAngle = axis.norm();
    dm::Vec3<Scalar> w = dm::Vec3<Scalar>::Zero();
    if (sinAngle > Sophus::Constants<Scalar>::epsilon()) {
        w = axis * (std::atan2(sinAngle, gs.GetDir().dot(dir)) / sinAngle);
    }
    dm::Vec3<Scalar> inc;
    inc.template head<2>() = gs.mBasisLin.transpose() * w;
    inc[2] = sigma - gs.GetSigma();
    gs.ApplyInc(inc);
}

// One per frame inertial factor at zero bias, §2.3.11 Step 1. The deltas are
// read from the residual at identity states, zero gravity and zero scale,
// where the rotation row is Log(dR) and its gyroscope bias Jacobian carries
// the inverse left Jacobian of Log(dR), which is removed.
template <class Scalar_>
struct DelayedMargFrameFactor {
    using Scalar = Scalar_;

    int64_t mT0 = 0, mT1 = 0;
    Scalar mDt = 0;
    dm::SO3<Scalar> mDR;
    dm::Vec3<Scalar> mDp = dm::Vec3<Scalar>::Zero();
    dm::Vec3<Scalar> mDv = dm::Vec3<Scalar>::Zero();
    dm::Mat93<Scalar> mJg = dm::Mat93<Scalar>::Zero();
    dm::Mat93<Scalar> mJa = dm::Mat93<Scalar>::Zero();
    dm::Mat9<Scalar> mCov = dm::Mat9<Scalar>::Zero();

    explicit DelayedMargFrameFactor(
        const IntegratedImuMeasurement<Scalar>& meas) {
        typename IntegratedImuMeasurement<Scalar>::MatN3 dBa, dBg;
        const typename PoseVelState<Scalar>::VecN res = meas.residualGS(
            PoseVelState<Scalar>(), dm::Vec3<Scalar>::Zero(),
            PoseVelState<Scalar>(), dm::Vec3<Scalar>::Zero(),
            dm::Vec3<Scalar>::Zero(), Scalar(0), nullptr, nullptr, &dBg, &dBa);
        mT0 = meas.get_start_t_ns();
        mT1 = mT0 + meas.get_dt_ns();
        mDt = Scalar(meas.get_dt_ns()) * Scalar(1e-9);
        const dm::Vec3<Scalar> phi = res.template segment<3>(3);
        mDR = dm::SO3<Scalar>::exp(phi);
        mDp = -res.template head<3>();
        mDv = -res.template tail<3>();
        dm::Mat3<Scalar> Jl;
        Sophus::leftJacobianSO3(phi, Jl);
        mJg = -dBg;
        mJg.template middleRows<3>(3) = Jl * dBg.template middleRows<3>(3);
        mJa = -dBa;
        mCov = meas.get_cov();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// The inertial factor from keyframe mA to mB, composed from the per frame
// factors between them in the body frame of mA, with its bias Jacobians and
// covariance ordered [position, rotation, velocity], §2.3.11 Step 2.
template <class Scalar_>
struct DelayedMargKfSpan {
    using Scalar = Scalar_;

    int64_t mA = -1, mB = -1;
    Scalar mT = 0;
    dm::SO3<Scalar> mDR;
    dm::Vec3<Scalar> mDp = dm::Vec3<Scalar>::Zero();
    dm::Vec3<Scalar> mDv = dm::Vec3<Scalar>::Zero();
    dm::Mat93<Scalar> mJg = dm::Mat93<Scalar>::Zero();
    dm::Mat93<Scalar> mJa = dm::Mat93<Scalar>::Zero();
    dm::Mat9<Scalar> mCov = dm::Mat9<Scalar>::Zero();
    Eigen::aligned_vector<DelayedMargFrameFactor<Scalar>> mvFactors;

    void Append(const DelayedMargFrameFactor<Scalar>& f) {
        const dm::Mat3<Scalar> R = mDR.matrix();
        dm::Mat9<Scalar> A = dm::Mat9<Scalar>::Identity();
        A.template block<3, 3>(0, 3) = -dm::SO3<Scalar>::hat(R * f.mDp);
        A.template block<3, 3>(0, 6).diagonal().setConstant(f.mDt);
        A.template block<3, 3>(6, 3) = -dm::SO3<Scalar>::hat(R * f.mDv);
        dm::Mat9<Scalar> B = dm::Mat9<Scalar>::Zero();
        for (int i = 0; i < 3; i++) {
            B.template block<3, 3>(3 * i, 3 * i) = R;
        }
        mCov = A * mCov * A.transpose() + B * f.mCov * B.transpose();
        mJg = A * mJg + B * f.mJg;
        mJa = A * mJa + B * f.mJa;
        mDp += mDv * f.mDt + R * f.mDp;
        mDv += R * f.mDv;
        mDR = mDR * f.mDR;
        mT += f.mDt;
        mB = f.mT1;
        mvFactors.push_back(f);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// The factors between consecutive keyframes, built once as the per frame
// factors arrive in time order, as DM-VIO's coarse initialiser integrates the
// inertial data of the frames between keyframes into the factor that ends at
// the next keyframe. mOpen runs from the latest keyframe.
template <class Scalar_>
struct DelayedMargKfChain {
    using Scalar = Scalar_;

    std::deque<DelayedMargKfSpan<Scalar>> mvSpans;
    DelayedMargKfSpan<Scalar> mOpen;

    void Add(const IntegratedImuMeasurement<Scalar>& meas,
             const std::set<int64_t>& keyframes, int maxSpans) {
        const int64_t t0 = meas.get_start_t_ns();
        if (mOpen.mA >= 0 && t0 < mOpen.mB) return;
        if (mOpen.mA < 0 || t0 != mOpen.mB) {
            mOpen = DelayedMargKfSpan<Scalar>();
            if (keyframes.count(t0) == 0) return;
            mOpen.mA = mOpen.mB = t0;
        }
        mOpen.Append(DelayedMargFrameFactor<Scalar>(meas));
        if (keyframes.count(mOpen.mB) > 0) {
            mvSpans.push_back(mOpen);
            while (int(mvSpans.size()) > maxSpans) mvSpans.pop_front();
            const int64_t kf = mOpen.mB;
            mOpen = DelayedMargKfSpan<Scalar>();
            mOpen.mA = mOpen.mB = kf;
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Everything one readvance needs, copied so that it can run in a worker.
template <class Scalar_>
struct DelayedMargJob {
    using Scalar = Scalar_;

    struct Elim {
        dm::MatX<Scalar> mJ;
        dm::VecX<Scalar> mR;
        std::vector<Eigen::Index> mPivotRow;
    };

    // Inputs, all in the epoch mEpoch.
    DelayedMargPrior<Scalar> mBoundary;
    std::vector<dm::StepPtr<Scalar>> mvSteps;
    DelayedMargValues<Scalar> mValues;
    GravityScaleStateWithLin<Scalar> mGs;
    dm::Vec3<Scalar> mG = dm::Vec3<Scalar>::Zero();
    dm::Vec3<Scalar> mGyroBiasWeightSqrt = dm::Vec3<Scalar>::Zero();
    dm::Vec3<Scalar> mAccelBiasWeightSqrt = dm::Vec3<Scalar>::Zero();
    GravityScaleCorrectionLog::Ptr mLog;
    int64_t mEpoch = 0;
    bool mSmooth = false;

    // Coarse inertial optimisation followed by a readvance at its result,
    // §2.3.11. The keyframe factors of the store, the inertial factors of the
    // window, the keyframes known to the store and the window, the initial
    // gravity direction in the epoch
    // mEpoch, the information of the tilt prior, and the time after which the
    // optimisation stops iterating.
    bool mCoarse = false;
    DelayedMargKfChain<Scalar> mKfChain;
    Eigen::aligned_vector<IntegratedImuMeasurement<Scalar>> mvWindowImu;
    std::set<int64_t> mvKeyframes;
    dm::Vec3<Scalar> mInitialDir = dm::Vec3<Scalar>::Zero();
    Scalar mTiltPriorW = 0;
    Scalar mScaleVarMax = 0;
    double mCoarseDeadlineSeconds = 0;
    // Standard deviation of a held keyframe position, §2.3.11 Step 4.
    static constexpr Scalar kCoarsePositionStd = Scalar(0.02);
    // DM-VIO's maxNumPoses and the iteration limit of GTSAM's
    // Levenberg-Marquardt.
    static constexpr int kCoarseMaxKeyframes = 100;
    static constexpr int kCoarseMaxIterations = 100;

    // Outputs.
    DelayedMargPrior<Scalar> mResult;
    // Smoothing increment of each departed variable, and which of its 15
    // coordinates it covers.
    Eigen::aligned_map<int64_t, std::pair<dm::Vec15<Scalar>, uint32_t>>
        mvSmoothInc;
    Scalar mMaxPosInc = 0;
    double mSeconds = 0;
    // Coarse outputs, the block and the velocity increments in the epoch
    // mEpoch, and the one gyroscope and accelerometer bias of every state.
    dm::Vec3<Scalar> mCoarseDir = dm::Vec3<Scalar>::Zero();
    // Angle between the snapshot's and the estimated gravity direction.
    Scalar mCoarseTilt = 0;
    Scalar mCoarseSigma = 0;
    dm::Vec3<Scalar> mCoarseBg = dm::Vec3<Scalar>::Zero();
    dm::Vec3<Scalar> mCoarseBa = dm::Vec3<Scalar>::Zero();
    Eigen::aligned_map<int64_t, dm::Vec3<Scalar>> mvCoarseVelInc;
    Scalar mCoarseMaxVelInc = 0;
    Scalar mCoarseScaleVar = 0;
    bool mCoarseScaleFree = false;
    Scalar mCoarseCost0 = 0;
    Scalar mCoarseCost1 = 0;
    int mCoarseFactors = 0;
    int mCoarseSpans = 0;
    int mCoarseIterations = 0;
    // The deadline stopped the optimisation, whose result is then discarded.
    bool mCoarseTimedOut = false;

    int64_t FirstStep() const { return mvSteps.front()->mIndex; }
    int64_t LastStep() const { return mvSteps.back()->mIndex; }

    // Readvance, preceded by one smoothing pass when mSmooth is set. A coarse
    // job first runs the coarse optimisation, and smooths only when that
    // estimated the scale, in the role of DM-VIO's pose graph bundle
    // adjustment.
    void Run() {
        const auto t0 = std::chrono::steady_clock::now();
        bool readvance = true;
        bool smooth = mSmooth;
        if (mCoarse) {
            Coarse();
            readvance = mCoarseSpans > 0 && !mCoarseTimedOut;
            smooth = mSmooth && mCoarseScaleFree;
        }
        if (readvance && smooth) {
            std::vector<Elim> elims;
            Forward(&elims);
            Backward(elims);
        }
        if (readvance) Forward(nullptr);
        mSeconds = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - t0)
                       .count();
    }

    // The coarse IMU initialiser of DM-VIO, CoarseIMUInitOptimizer. The last
    // kCoarseMaxKeyframes keyframes are held, consecutive keyframes are joined
    // by the factors of mKfChain extended by the window's, and the gravity
    // direction, the scale applied to the held positions, one gyroscope and
    // accelerometer bias starting at zero, and the metric velocity of every
    // keyframe are estimated by Levenberg-Marquardt. The scale is held when
    // its marginal variance at the solution exceeds mScaleVarMax. The result
    // is applied to mValues and mGs, §2.3.11.
    void Coarse() {
        const auto tStart = std::chrono::steady_clock::now();

        // The store's keyframe factors, extended by the window's, and the held
        // poses of their keyframes.
        DelayedMargKfChain<Scalar> chain = mKfChain;
        for (const auto& meas : mvWindowImu) {
            chain.Add(meas, mvKeyframes, kCoarseMaxKeyframes - 1);
        }
        struct Span {
            const DelayedMargKfSpan<Scalar>* mSp = nullptr;
            dm::SO3<Scalar> mRa, mRb;
            dm::Vec3<Scalar> mPa = dm::Vec3<Scalar>::Zero();
            dm::Vec3<Scalar> mPb = dm::Vec3<Scalar>::Zero();
            dm::Mat9<Scalar> mL = dm::Mat9<Scalar>::Identity();
        };
        Eigen::aligned_vector<Span> vSpans;
        mCoarseFactors = 0;
        for (const DelayedMargKfSpan<Scalar>& kfs : chain.mvSpans) {
            if (!Known(kfs.mA) || !Known(kfs.mB)) continue;
            Span sp;
            sp.mSp = &kfs;
            const PoseVelBiasState<Scalar> sa =
                mValues.State(kfs.mA).getState();
            const PoseVelBiasState<Scalar> sb =
                mValues.State(kfs.mB).getState();
            sp.mRa = sa.T_w_i.so3();
            sp.mRb = sb.T_w_i.so3();
            sp.mPa = sa.T_w_i.translation();
            sp.mPb = sb.T_w_i.translation();
            vSpans.push_back(sp);
            mCoarseFactors += int(kfs.mvFactors.size());
        }
        mCoarseSpans = int(vSpans.size());

        const dm::Vec3<Scalar> dir0 = mGs.GetDir();
        const dm::Mat32<Scalar> basis0 = mGs.GetBasis();
        const Scalar sigma0 = mGs.GetSigma();
        mCoarseDir = dir0;
        mCoarseTilt = 0;
        mCoarseSigma = sigma0;
        mCoarseBg.setZero();
        mCoarseBa.setZero();
        mvCoarseVelInc.clear();
        mCoarseMaxVelInc = 0;
        mCoarseScaleFree = false;
        mCoarseScaleVar = 0;
        mCoarseIterations = 0;
        mCoarseTimedOut = false;
        if (vSpans.empty()) return;

        // Unknowns [tilt (2), scale increment, gyroscope bias (3),
        // accelerometer bias (3), keyframe velocities (3 each)].
        std::map<int64_t, int> nodeIdx;
        for (const Span& sp : vSpans) {
            nodeIdx.emplace(sp.mSp->mA, 0);
            nodeIdx.emplace(sp.mSp->mB, 0);
        }
        const int nNodes = int(nodeIdx.size());
        const int n = 9 + 3 * nNodes;
        const Scalar s0 = std::exp(sigma0);
        dm::VecX<Scalar> u0(3 * nNodes);
        {
            int i = 0;
            for (auto& [id, idx] : nodeIdx) {
                idx = i;
                u0.template segment<3>(3 * i) =
                    s0 * mValues.State(id).getState().vel_w_i;
                i++;
            }
        }

        // Whitening in metric units, the composed covariance and the noise of
        // the two held positions at the scale of the snapshot.
        for (Span& sp : vSpans) {
            dm::Mat9<Scalar> S = sp.mSp->mCov;
            S.template block<3, 3>(0, 0).diagonal().array() +=
                Scalar(2) * s0 * s0 * kCoarsePositionStd * kCoarsePositionStd;
            sp.mL = S.llt().matrixL();
        }
        const Scalar gNorm = mG.norm();

        dm::SO3<Scalar> Rg;
        Scalar eta = 0;
        dm::Vec3<Scalar> bg = dm::Vec3<Scalar>::Zero();
        dm::Vec3<Scalar> ba = dm::Vec3<Scalar>::Zero();
        dm::VecX<Scalar> du = dm::VecX<Scalar>::Zero(3 * nNodes);

        auto tiltResidual = [&](const dm::Vec3<Scalar>& dir,
                                const dm::Mat32<Scalar>& basis) {
            const dm::Vec3<Scalar> axis = mInitialDir.cross(dir);
            const Scalar sinAngle = axis.norm();
            dm::Vec3<Scalar> w = dm::Vec3<Scalar>::Zero();
            if (sinAngle > Sophus::Constants<Scalar>::epsilon()) {
                w = axis *
                    (std::atan2(sinAngle, mInitialDir.dot(dir)) / sinAngle);
            }
            return dm::Vec2<Scalar>(basis.transpose() * w);
        };

        // The residual of GTSAM's ImuFactor between the held keyframe poses
        // scaled by s = s0 (1 + eta), §2.3.11 Step 3. The normal equations
        // are accumulated block by block into a sparse matrix, since a
        // keyframe velocity meets only its neighbours.
        std::vector<Eigen::Triplet<Scalar>> vTriplets;
        auto linearise = [&](dm::SpMat<Scalar>* H, dm::VecX<Scalar>* b) {
            const dm::Vec3<Scalar> dir = Rg * dir0;
            const dm::Mat32<Scalar> basis = Rg.matrix() * basis0;
            const dm::Vec3<Scalar> gD = gNorm * dir;
            const Scalar s = s0 * (Scalar(1) + eta);
            const dm::Mat32<Scalar> dGd = -dm::SO3<Scalar>::hat(gD) * basis;
            Scalar cost = 0;
            if (H) {
                vTriplets.clear();
                b->setZero(n);
            }
            Eigen::Matrix<Scalar, 9, 15> J;
            for (const Span& sp : vSpans) {
                const DelayedMargKfSpan<Scalar>& kf = *sp.mSp;
                const int ia = nodeIdx.at(kf.mA);
                const int ib = nodeIdx.at(kf.mB);
                const dm::Vec3<Scalar> ua = u0.template segment<3>(3 * ia) +
                                            du.template segment<3>(3 * ia);
                const dm::Vec3<Scalar> ub = u0.template segment<3>(3 * ib) +
                                            du.template segment<3>(3 * ib);
                const Scalar T = kf.mT;
                const dm::Mat3<Scalar> RaT = sp.mRa.inverse().matrix();
                const dm::Vec9<Scalar> corr = kf.mJg * bg + kf.mJa * ba;
                const dm::Vec3<Scalar> dpVis = sp.mPb - sp.mPa;

                dm::Vec9<Scalar> r;
                r.template head<3>() =
                    RaT * (s * dpVis - ua * T - Scalar(0.5) * gD * T * T) -
                    (kf.mDp + corr.template head<3>());
                const dm::Vec3<Scalar> rR =
                    (dm::SO3<Scalar>::exp(corr.template segment<3>(3)) *
                     kf.mDR * sp.mRb.inverse() * sp.mRa)
                        .log();
                r.template segment<3>(3) = rR;
                r.template tail<3>() = RaT * (ub - ua - gD * T) -
                                       (kf.mDv + corr.template tail<3>());
                const dm::Vec9<Scalar> rw =
                    sp.mL.template triangularView<Eigen::Lower>().solve(r);
                cost += Scalar(0.5) * rw.squaredNorm();
                if (!H) continue;

                // Columns [tilt, scale, bg, ba, ua, ub].
                J.setZero();
                J.template block<3, 2>(0, 0) =
                    -Scalar(0.5) * T * T * RaT * dGd;
                J.template block<3, 2>(6, 0) = -T * RaT * dGd;
                J.template block<3, 1>(0, 2) = s0 * RaT * dpVis;
                dm::Mat3<Scalar> JlInv;
                Sophus::leftJacobianInvSO3(rR, JlInv);
                J.template block<3, 3>(0, 3) = -kf.mJg.template topRows<3>();
                J.template block<3, 3>(3, 3) =
                    JlInv * kf.mJg.template middleRows<3>(3);
                J.template block<3, 3>(6, 3) = -kf.mJg.template bottomRows<3>();
                J.template block<3, 3>(0, 6) = -kf.mJa.template topRows<3>();
                J.template block<3, 3>(6, 6) = -kf.mJa.template bottomRows<3>();
                J.template block<3, 3>(0, 9) = -T * RaT;
                J.template block<3, 3>(6, 9) = -RaT;
                J.template block<3, 3>(6, 12) = RaT;
                const Eigen::Matrix<Scalar, 9, 15> Jw =
                    sp.mL.template triangularView<Eigen::Lower>().solve(J);
                const Eigen::Matrix<Scalar, 15, 15> Hs = Jw.transpose() * Jw;
                const Eigen::Matrix<Scalar, 15, 1> bs = Jw.transpose() * rw;
                const int col[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8,
                                     9 + 3 * ia, 10 + 3 * ia, 11 + 3 * ia,
                                     9 + 3 * ib, 10 + 3 * ib, 11 + 3 * ib};
                for (int i = 0; i < 15; i++) {
                    (*b)[col[i]] += bs[i];
                    for (int j = 0; j < 15; j++) {
                        vTriplets.emplace_back(col[i], col[j], Hs(i, j));
                    }
                }
            }
            if (mTiltPriorW > 0) {
                const dm::Vec2<Scalar> rt = tiltResidual(dir, basis);
                cost += Scalar(0.5) * mTiltPriorW * rt.squaredNorm();
                if (H) {
                    vTriplets.emplace_back(0, 0, mTiltPriorW);
                    vTriplets.emplace_back(1, 1, mTiltPriorW);
                    b->template head<2>() += mTiltPriorW * rt;
                }
            }
            if (H) {
                H->resize(n, n);
                H->setFromTriplets(vTriplets.begin(), vTriplets.end());
            }
            return cost;
        };

        auto elapsed = [&]() {
            return std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - tStart)
                .count();
        };

        // Levenberg-Marquardt from the snapshot with the scale free or held,
        // stopping at the deadline so that the job ends within a frame.
        dm::SpMat<Scalar> H;
        dm::VecX<Scalar> b;
        auto solve = [&](bool scaleFree) {
            Rg = dm::SO3<Scalar>();
            eta = 0;
            bg.setZero();
            ba.setZero();
            du.setZero();
            Scalar cost = linearise(&H, &b);
            Scalar lambda = Scalar(1e-4);
            for (int it = 0; it < kCoarseMaxIterations && lambda < Scalar(1e8);
                 it++) {
                if (mCoarseDeadlineSeconds > 0 &&
                    elapsed() > mCoarseDeadlineSeconds) {
                    mCoarseTimedOut = true;
                    break;
                }
                mCoarseIterations++;
                dm::SpMat<Scalar> Hd = H;
                dm::VecX<Scalar> bd = b;
                for (int i = 0; i < n; i++) {
                    Hd.coeffRef(i, i) +=
                        lambda * H.coeff(i, i) + Scalar(1e-9);
                }
                if (!scaleFree) {
                    Hd.prune([](Eigen::Index r, Eigen::Index c, const Scalar&) {
                        return (r != 2 && c != 2) || r == c;
                    });
                    Hd.coeffRef(2, 2) = 1;
                    bd[2] = 0;
                }
                Eigen::SimplicialLDLT<dm::SpMat<Scalar>> ldlt(Hd);
                if (ldlt.info() != Eigen::Success) break;
                const dm::VecX<Scalar> x = -ldlt.solve(bd);
                if (!x.allFinite()) break;
                const dm::SO3<Scalar> Rg0 = Rg;
                const Scalar eta0 = eta;
                const dm::Vec3<Scalar> bg0 = bg, ba0 = ba;
                const dm::VecX<Scalar> du0 = du;
                Rg = dm::SO3<Scalar>::exp(Rg.matrix() * basis0 *
                                          x.template head<2>()) *
                     Rg;
                eta += x[2];
                bg += x.template segment<3>(3);
                ba += x.template segment<3>(6);
                du += x.tail(3 * nNodes);
                const Scalar costNew = linearise(nullptr, nullptr);
                if (eta > Scalar(-1) && std::isfinite(double(costNew)) &&
                    costNew < cost) {
                    const bool converged =
                        cost - costNew < Scalar(1e-6) * cost;
                    cost = linearise(&H, &b);
                    lambda = std::max(lambda / Scalar(10), Scalar(1e-7));
                    if (converged) break;
                } else {
                    Rg = Rg0;
                    eta = eta0;
                    bg = bg0;
                    ba = ba0;
                    du = du0;
                    lambda *= Scalar(10);
                }
            }
            linearise(&H, &b);
            return cost;
        };

        mCoarseCost0 = linearise(nullptr, nullptr);
        mCoarseCost1 = solve(true);
        // The marginal variance of the log scale at the solution, where
        // log s = sigma0 + log(1 + eta).
        {
            dm::SpMat<Scalar> Hv = H;
            for (int i = 0; i < n; i++) Hv.coeffRef(i, i) += Scalar(1e-9);
            Eigen::SimplicialLDLT<dm::SpMat<Scalar>> ldlt(Hv);
            dm::VecX<Scalar> e2 = dm::VecX<Scalar>::Zero(n);
            e2[2] = 1;
            const Scalar varEta =
                ldlt.info() == Eigen::Success ? ldlt.solve(e2)[2]
                                              : Scalar(1e30);
            mCoarseScaleVar = varEta / ((Scalar(1) + eta) * (Scalar(1) + eta));
        }
        mCoarseScaleFree = std::isfinite(double(mCoarseScaleVar)) &&
                           mCoarseScaleVar <= mScaleVarMax;
        if (!mCoarseScaleFree && !mCoarseTimedOut) {
            mCoarseCost1 = solve(false);
        }
        if (mCoarseTimedOut) return;

        mCoarseDir = Rg * dir0;
        mCoarseTilt = std::acos(
            std::clamp(mCoarseDir.dot(dir0), Scalar(-1), Scalar(1)));
        const Scalar s = s0 * (Scalar(1) + eta);
        mCoarseSigma = std::log(s);
        mCoarseBg = bg;
        mCoarseBa = ba;

        // Velocities in the estimator's units, u / s, at the keyframes, and at
        // the states between and after them propagated through the per frame
        // factors at the estimated biases and the held frame rotations.
        const dm::Vec3<Scalar> gD = gNorm * mCoarseDir;
        std::map<int64_t, dm::Vec3<Scalar>> vel;
        // Propagates v from the first state of span through its factors,
        // keeping the solved velocity of a keyframe at its end.
        auto propagate = [&](dm::Vec3<Scalar> v,
                             const DelayedMargKfSpan<Scalar>& span, bool open) {
            for (const DelayedMargFrameFactor<Scalar>& f : span.mvFactors) {
                if (!Known(f.mT0) || !Known(f.mT1)) break;
                const dm::Vec9<Scalar> c = f.mJg * bg + f.mJa * ba;
                const dm::Mat3<Scalar> R =
                    mValues.State(f.mT0).getState().T_w_i.so3().matrix();
                v += (gD * f.mDt + R * (f.mDv + c.template tail<3>())) / s;
                if (!open && f.mT1 == span.mB) break;
                vel[f.mT1] = v;
            }
        };
        for (const auto& [id, idx] : nodeIdx) {
            vel[id] = (u0.template segment<3>(3 * idx) +
                       du.template segment<3>(3 * idx)) /
                      s;
        }
        for (const Span& sp : vSpans) {
            propagate(vel.at(sp.mSp->mA), *sp.mSp, false);
        }
        if (nodeIdx.count(chain.mOpen.mA) > 0) {
            propagate(vel.at(chain.mOpen.mA), chain.mOpen, true);
        }
        for (const auto& [id, v] : vel) {
            const dm::Vec3<Scalar> inc =
                v - mValues.State(id).getState().vel_w_i;
            if (!inc.allFinite()) continue;
            mvCoarseVelInc[id] = inc;
            mCoarseMaxVelInc = std::max(mCoarseMaxVelInc, inc.norm());
        }

        // The readvance that follows sees the corrected values, every state
        // with the one estimated bias.
        auto apply = [&](int64_t id) {
            const PoseVelBiasState<Scalar> st = mValues.State(id).getState();
            dm::Vec15<Scalar> x = dm::Vec15<Scalar>::Zero();
            const auto it = mvCoarseVelInc.find(id);
            if (it != mvCoarseVelInc.end()) {
                x.template segment<3>(6) = it->second;
            }
            x.template segment<3>(9) = mCoarseBg - st.bias_gyro;
            x.template segment<3>(12) = mCoarseBa - st.bias_accel;
            mValues.ApplyInc(id, x);
        };
        for (const auto& [id, st] : mValues.mvWindowStates) apply(id);
        for (const auto& [id, rec] : mValues.mvRecords) {
            if (mValues.mvWindowStates.count(id) == 0) apply(id);
        }
        DelayedMargSetBlock(mGs, mCoarseDir, mCoarseSigma);
    }

    bool Known(int64_t id) const {
        return mValues.mvWindowStates.count(id) > 0 ||
               mValues.mvRecords.count(id) > 0;
    }

    // Marginalises every step into the boundary in order, with the inertial
    // factors linearised at the current block, plans/delayed_marginalisation.md
    // §2.3.4.
    void Forward(std::vector<Elim>* elims) {
        GravityScaleStateWithLin<Scalar> gsNow = mGs;
        gsNow.Reanchor(mGs.GetDir(), mGs.GetBasis(), mGs.GetSigma());

        mResult = mBoundary;
        if (elims) elims->resize(mvSteps.size());
        for (size_t i = 0; i < mvSteps.size(); i++) {
            MarginaliseStep(*mvSteps[i], mValues, gsNow, mG,
                            mGyroBiasWeightSqrt, mAccelBiasWeightSqrt,
                            mLog.get(), mEpoch, false, Scalar(0), mResult,
                            elims ? &(*elims)[i] : nullptr);
        }
    }

    // Solves every departed variable given the window, by back substitution
    // through the eliminated rows, §2.3.6.
    void Backward(const std::vector<Elim>& elims) {
        // Increments from the current values, known at the window and block.
        Eigen::aligned_map<int64_t, std::pair<dm::Vec15<Scalar>, uint32_t>>
            inc;
        constexpr uint32_t kAll = (1u << POSE_VEL_BIAS_SIZE) - 1;
        for (const auto& [id, idx] : mvSteps.back()->mOrderNew.abs_order_map) {
            if (id == kGravityScaleStateId) continue;
            inc[id] = {dm::Vec15<Scalar>::Zero(), kAll};
        }

        for (size_t i = mvSteps.size(); i-- > 0;) {
            const DelayedMargStep<Scalar>& step = *mvSteps[i];
            const Elim& e = elims[i];

            std::vector<std::pair<int64_t, int>> colVar(step.mOrder.total_size,
                                                        {0, -1});
            for (const auto& [id, idx] : step.mOrder.abs_order_map) {
                for (int c = 0; c < idx.second; c++) {
                    colVar[idx.first + c] = {id, c};
                }
            }

            const int m = int(step.mIdxMarg.size());
            const int n = int(step.mIdxKeep.size());
            dm::VecX<Scalar> xKeep = dm::VecX<Scalar>::Zero(n);
            {
                int j = 0;
                for (int col : step.mIdxKeep) {
                    const auto& [id, c] = colVar[col];
                    if (id != kGravityScaleStateId) {
                        const auto it = inc.find(id);
                        BASALT_ASSERT(it != inc.end() &&
                                      ((it->second.second >> c) & 1u));
                        xKeep[j] = it->second.first[c];
                    }
                    j++;
                }
            }

            dm::VecX<Scalar> xMarg = dm::VecX<Scalar>::Zero(m);
            for (int k = m - 1; k >= 0; k--) {
                const Eigen::Index t = e.mPivotRow[k];
                if (t < 0 || t >= e.mJ.rows()) continue;
                Scalar s = e.mR[t] + e.mJ.row(t).tail(n).dot(xKeep);
                if (k + 1 < m) {
                    s += e.mJ.row(t).segment(k + 1, m - k - 1).dot(
                        xMarg.segment(k + 1, m - k - 1));
                }
                xMarg[k] = -s / e.mJ(t, k);
            }

            int k = 0;
            for (int col : step.mIdxMarg) {
                const auto& [id, c] = colVar[col];
                auto& entry = inc[id];
                if (entry.second == 0) entry.first.setZero();
                entry.first[c] = xMarg[k];
                entry.second |= 1u << c;
                k++;
            }
        }

        mvSmoothInc.clear();
        mMaxPosInc = 0;
        for (auto& [id, rec] : mValues.mvRecords) {
            const auto it = inc.find(id);
            if (it == inc.end()) continue;
            // Coordinates still in the window are conditioned, not solved.
            uint32_t mask = it->second.second;
            if (mValues.mvWindowPoses.count(id) > 0) mask &= ~0x3Fu;
            if (mValues.mvWindowStates.count(id) > 0) mask = 0;
            if (mask == 0) continue;

            dm::Vec15<Scalar> x = dm::Vec15<Scalar>::Zero();
            for (int c = 0; c < int(POSE_VEL_BIAS_SIZE); c++) {
                if ((mask >> c) & 1u) x[c] = it->second.first[c];
            }
            if (!x.allFinite()) continue;
            rec.mState.applyInc(x);
            mvSmoothInc[id] = {x, mask};
            mMaxPosInc = std::max(mMaxPosInc, x.template head<3>().norm());
        }
    }

    // Marginalises one step into prior. The prior arrives and leaves delta
    // independent, its rows evaluated at the current values in between,
    // §2.3.4. gsImu is the block at which the inertial factors are
    // linearised and becomes the anchor of the result.
    static void MarginaliseStep(const DelayedMargStep<Scalar>& step,
                                const DelayedMargValues<Scalar>& values,
                                const GravityScaleStateWithLin<Scalar>& gsImu,
                                const dm::Vec3<Scalar>& g,
                                const dm::Vec3<Scalar>& gyroW,
                                const dm::Vec3<Scalar>& accelW,
                                const GravityScaleCorrectionLog* log,
                                int64_t epoch, bool imuScaleFree,
                                Scalar imuScaleFreeTol,
                                DelayedMargPrior<Scalar>& prior, Elim* elim) {
        const int n = int(step.mOrder.total_size);
        const int pr = int(prior.mH.rows());
        const int pc = int(prior.mH.cols());
        const int vr = int(step.mVisJ.rows());
        const int ir = int(step.mvImu.size()) * POSE_VEL_BIAS_SIZE;

        BASALT_ASSERT(pc <= n);
        BASALT_ASSERT(step.mOrder.abs_order_map.at(kGravityScaleStateId) ==
                      prior.mOrder.abs_order_map.at(kGravityScaleStateId));

        dm::MatX<Scalar> Q2Jp = dm::MatX<Scalar>::Zero(pr + vr + ir, n);
        dm::VecX<Scalar> Q2r = dm::VecX<Scalar>::Zero(pr + vr + ir);

        // The current block seen from the prior's anchor.
        GravityScaleStateWithLin<Scalar> gsPrior = gsImu;
        gsPrior.Reanchor(prior.mAnchor.mDir, prior.mAnchor.mBasis,
                         prior.mAnchor.mSigma);
        Q2Jp.topLeftCorner(pr, pc) = prior.mH;
        Q2r.head(pr) =
            prior.mB +
            prior.mH * values.Delta(prior.mOrder, gsPrior.GetDelta());

        if (vr > 0) {
            dm::MatX<Scalar> J = step.mVisJ;
            if (step.mEpoch != epoch) {
                const GravityScaleCorrectionLog::Entry c =
                    log->Between(step.mEpoch, epoch);
                DelayedMargTransformColumns(
                    J, step.mOrder, c.mQ.template cast<Scalar>(),
                    Scalar(c.mScale));
            }
            Q2Jp.middleRows(pr, vr) = J;
            Q2r.segment(pr, vr) =
                step.mVisR +
                J * values.Delta(step.mOrder, dm::Vec3<Scalar>::Zero());
        }

        if (ir > 0) {
            ImuLinData<Scalar> ild = {g, gyroW, accelW, {}};
            Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<Scalar>> states;
            int row = pr + vr;
            for (const IntegratedImuMeasurement<Scalar>& meas : step.mvImu) {
                const int64_t t0 = meas.get_start_t_ns();
                const int64_t t1 = t0 + meas.get_dt_ns();
                states[t0] = values.State(t0);
                states[t1] = values.State(t1);
                ImuBlockGS<Scalar> block(&meas, &ild, step.mOrder, &gsImu,
                                         imuScaleFree, imuScaleFreeTol);
                block.linearizeImu(states);
                block.add_dense_Q2Jp_Q2r(Q2Jp, Q2r, row);
                row += POSE_VEL_BIAS_SIZE;
            }
        }

        dm::MatX<Scalar> H_new;
        dm::VecX<Scalar> b_new;
        MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(
            Q2Jp, Q2r, step.mIdxKeep, step.mIdxMarg, H_new, b_new,
            elim ? &elim->mJ : nullptr, elim ? &elim->mR : nullptr,
            elim ? &elim->mPivotRow : nullptr);

        prior.mH = H_new;
        prior.mOrder = step.mOrderNew;
        prior.mAnchor.mDir = gsImu.mDirLin;
        prior.mAnchor.mBasis = gsImu.mBasisLin;
        prior.mAnchor.mSigma = gsImu.mSigmaLin;
        prior.mB = b_new - prior.mH * values.Delta(step.mOrderNew,
                                                    gsImu.GetDelta());
    }
};

// The delayed store of the main thread, plans/delayed_marginalisation.md
// §3.2.
template <class Scalar_>
class DelayedMarginalisation {
   public:
    using Scalar = Scalar_;

    explicit DelayedMarginalisation(int delay) : mpDelay(delay) {}

    bool BoundarySet() const { return mpBoundarySet; }

    void SetBoundary(const DelayedMargPrior<Scalar>& prior) {
        mpBoundary = prior;
        mpBoundarySet = true;
    }

    const DelayedMargPrior<Scalar>& Boundary() const { return mpBoundary; }
    const std::deque<dm::StepPtr<Scalar>>& Steps() const { return mvpSteps; }
    DelayedMargValues<Scalar>& Values() { return mpValues; }
    const DelayedMargValues<Scalar>& Values() const { return mpValues; }
    int64_t NextIndex() const { return mpNextIndex; }

    // Keyframes of the period, the nodes of the coarse optimisation, and the
    // factors between them composed from the stored per frame factors.
    void AddKeyframe(int64_t id) { mvpKeyframes.insert(id); }
    const std::set<int64_t>& Keyframes() const { return mvpKeyframes; }
    const DelayedMargKfChain<Scalar>& KfChain() const { return mpKfChain; }

    // Records the coordinates that leave the window at this step.
    void RecordDeparture(int64_t id, const PoseVelBiasStateWithLin<Scalar>& st,
                         int64_t step) {
        DelayedMargRecord<Scalar>& rec = mpValues.mvRecords[id];
        rec.mState = Frozen(st);
        rec.mLastStep = step;
    }

    void RecordPoseDeparture(int64_t id, const PoseStateWithLin<Scalar>& ps,
                             int64_t step) {
        auto it = mpValues.mvRecords.find(id);
        PoseVelBiasState<Scalar> lin;
        typename PoseVelBiasStateWithLin<Scalar>::VecN delta;
        delta.setZero();
        if (it != mpValues.mvRecords.end()) {
            lin = it->second.mState.getStateLin();
            delta = it->second.mState.getDelta();
        }
        lin.t_ns = id;
        lin.T_w_i = ps.getPoseLin();
        delta.template head<POSE_SIZE>() = ps.getDelta();
        PoseVelBiasStateWithLin<Scalar> st(id, lin.T_w_i, lin.vel_w_i,
                                           lin.bias_gyro, lin.bias_accel,
                                           true);
        st.applyInc(delta);
        DelayedMargRecord<Scalar>& rec = mpValues.mvRecords[id];
        rec.mState = st;
        rec.mLastStep = step;
    }

    void Push(DelayedMargStep<Scalar>&& step) {
        for (IntegratedImuMeasurement<Scalar>& meas : step.mvImu) {
            // Fills the lazily computed whitening before the step is shared.
            meas.get_sqrt_cov_inv();
            mpKfChain.Add(meas, mvpKeyframes,
                          DelayedMargJob<Scalar>::kCoarseMaxKeyframes - 1);
        }
        mpNextIndex = step.mIndex + 1;
        mvpSteps.push_back(
            std::make_shared<const DelayedMargStep<Scalar>>(std::move(step)));
    }

    // Moves the oldest steps beyond the delay into the boundary prior, with
    // the scale free inertial rows of Change A when boundaryImu is set,
    // §2.3.5.
    int AdvanceBoundary(const GravityScaleStateWithLin<Scalar>& gs,
                        const dm::Vec3<Scalar>& g,
                        const dm::Vec3<Scalar>& gyroW,
                        const dm::Vec3<Scalar>& accelW,
                        const GravityScaleCorrectionLog* log, int64_t epoch,
                        bool boundaryImu, Scalar scaleFreeTol) {
        int advanced = 0;
        while (int(mvpSteps.size()) > mpDelay) {
            const dm::StepPtr<Scalar> step = mvpSteps.front();
            GravityScaleStateWithLin<Scalar> gsB = gs;
            gsB.Reanchor(mpBoundary.mAnchor.mDir, mpBoundary.mAnchor.mBasis,
                         gs.GetSigma());

            DelayedMargStep<Scalar> stepB = *step;
            if (!boundaryImu) stepB.mvImu.clear();
            DelayedMargJob<Scalar>::MarginaliseStep(
                stepB, mpValues, gsB, g, gyroW, accelW, log, epoch, true,
                scaleFreeTol, mpBoundary, nullptr);

            mvpSteps.pop_front();
            while (!mpKfChain.mvSpans.empty() &&
                   mpValues.mvRecords.count(mpKfChain.mvSpans.front().mA) > 0 &&
                   mpValues.mvRecords.at(mpKfChain.mvSpans.front().mA)
                           .mLastStep <= step->mIndex) {
                mpKfChain.mvSpans.pop_front();
            }
            for (auto it = mpValues.mvRecords.begin();
                 it != mpValues.mvRecords.end();) {
                if (it->second.mLastStep <= step->mIndex) {
                    mvpKeyframes.erase(it->first);
                    it = mpValues.mvRecords.erase(it);
                } else {
                    ++it;
                }
            }
            advanced++;
        }
        return advanced;
    }

    // Applies a re-centring to the boundary and the records. The steps keep
    // their own epoch and are mapped when read.
    void Transform(const dm::SO3<Scalar>& Q, Scalar s, Scalar sigmaShift) {
        DelayedMargTransformColumns(mpBoundary.mH, mpBoundary.mOrder, Q, s);
        mpBoundary.mAnchor.Transform(Q, sigmaShift);
        for (auto& [id, rec] : mpValues.mvRecords) {
            rec.mState.TransformBySim3(Q, s);
        }
    }

    static PoseVelBiasStateWithLin<Scalar> Frozen(
        const PoseVelBiasStateWithLin<Scalar>& st) {
        const PoseVelBiasState<Scalar>& lin = st.getStateLin();
        PoseVelBiasStateWithLin<Scalar> out(lin.t_ns, lin.T_w_i, lin.vel_w_i,
                                            lin.bias_gyro, lin.bias_accel,
                                            true);
        out.applyInc(st.getDelta());
        return out;
    }

   private:
    int mpDelay;
    bool mpBoundarySet = false;
    int64_t mpNextIndex = 0;
    DelayedMargPrior<Scalar> mpBoundary;
    std::deque<dm::StepPtr<Scalar>> mvpSteps;
    DelayedMargValues<Scalar> mpValues;
    std::set<int64_t> mvpKeyframes;
    DelayedMargKfChain<Scalar> mpKfChain;
};

}  // namespace basalt
