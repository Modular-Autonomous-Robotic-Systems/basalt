#pragma once

#include <basalt/imu/preintegration.h>
#include <basalt/optimization/accumulator.h>
#include <basalt/utils/imu_types.h>

namespace basalt {

template <class Scalar_>
class ImuBlock {
 public:
  using Scalar = Scalar_;

  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using VecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  using MatX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  ImuBlock(const IntegratedImuMeasurement<Scalar>* meas,
           const ImuLinData<Scalar>* imu_lin_data, const AbsOrderMap& aom)
      : imu_meas(meas), imu_lin_data(imu_lin_data), aom(aom) {
    Jp.resize(POSE_VEL_BIAS_SIZE, 2 * POSE_VEL_BIAS_SIZE);
    r.resize(POSE_VEL_BIAS_SIZE);
  }

  virtual ~ImuBlock() = default;

  virtual Scalar linearizeImu(
      const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<Scalar>>&
          frame_states) {
    Jp.setZero();
    r.setZero();

    const int64_t start_t = imu_meas->get_start_t_ns();
    const int64_t end_t = imu_meas->get_start_t_ns() + imu_meas->get_dt_ns();

    const size_t start_idx = 0;
    const size_t end_idx = POSE_VEL_BIAS_SIZE;

    PoseVelBiasStateWithLin<Scalar> start_state = frame_states.at(start_t);
    PoseVelBiasStateWithLin<Scalar> end_state = frame_states.at(end_t);

    typename IntegratedImuMeasurement<Scalar>::MatNN d_res_d_start, d_res_d_end;
    typename IntegratedImuMeasurement<Scalar>::MatN3 d_res_d_bg, d_res_d_ba;

    typename PoseVelState<Scalar>::VecN res = imu_meas->residual(
        start_state.getStateLin(), imu_lin_data->g, end_state.getStateLin(),
        start_state.getStateLin().bias_gyro,
        start_state.getStateLin().bias_accel, &d_res_d_start, &d_res_d_end,
        &d_res_d_bg, &d_res_d_ba);

    if (start_state.isLinearized() || end_state.isLinearized()) {
      res = imu_meas->residual(
          start_state.getState(), imu_lin_data->g, end_state.getState(),
          start_state.getState().bias_gyro, start_state.getState().bias_accel);
    }

    // error
    Scalar imu_error =
        Scalar(0.5) * (imu_meas->get_sqrt_cov_inv() * res).squaredNorm();

    // imu residual linearization
    Jp.template block<9, 9>(0, start_idx) =
        imu_meas->get_sqrt_cov_inv() * d_res_d_start;
    Jp.template block<9, 9>(0, end_idx) =
        imu_meas->get_sqrt_cov_inv() * d_res_d_end;

    Jp.template block<9, 3>(0, start_idx + 9) =
        imu_meas->get_sqrt_cov_inv() * d_res_d_bg;
    Jp.template block<9, 3>(0, start_idx + 12) =
        imu_meas->get_sqrt_cov_inv() * d_res_d_ba;

    r.template segment<9>(0) = imu_meas->get_sqrt_cov_inv() * res;

    const std::pair<Scalar, Scalar> bias_error =
        LinearizeBiasRandomWalk(start_state, end_state);

    return imu_error + bias_error.first + bias_error.second;
  }

  virtual void add_dense_Q2Jp_Q2r(MatX& Q2Jp, VecX& Q2r,
                                  size_t row_start_idx) const {
    int64_t start_t = imu_meas->get_start_t_ns();
    int64_t end_t = imu_meas->get_start_t_ns() + imu_meas->get_dt_ns();

    const size_t start_idx = aom.abs_order_map.at(start_t).first;
    const size_t end_idx = aom.abs_order_map.at(end_t).first;

    Q2Jp.template block<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(row_start_idx,
                                                                start_idx) +=
        Jp.template topLeftCorner<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>();

    Q2Jp.template block<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(row_start_idx,
                                                                end_idx) +=
        Jp.template topRightCorner<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>();

    Q2r.template segment<POSE_VEL_BIAS_SIZE>(row_start_idx) += r;
  }

  virtual void add_dense_H_b(DenseAccumulator<Scalar>& accum) const {
    int64_t start_t = imu_meas->get_start_t_ns();
    int64_t end_t = imu_meas->get_start_t_ns() + imu_meas->get_dt_ns();

    const size_t start_idx = aom.abs_order_map.at(start_t).first;
    const size_t end_idx = aom.abs_order_map.at(end_t).first;

    const MatX H = Jp.transpose() * Jp;
    const VecX b = Jp.transpose() * r;

    accum.template addH<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(
        start_idx, start_idx,
        H.template block<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(0, 0));

    accum.template addH<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(
        end_idx, start_idx,
        H.template block<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(
            POSE_VEL_BIAS_SIZE, 0));

    accum.template addH<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(
        start_idx, end_idx,
        H.template block<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(
            0, POSE_VEL_BIAS_SIZE));

    accum.template addH<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(
        end_idx, end_idx,
        H.template block<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>(
            POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE));

    accum.template addB<POSE_VEL_BIAS_SIZE>(
        start_idx, b.template segment<POSE_VEL_BIAS_SIZE>(0));
    accum.template addB<POSE_VEL_BIAS_SIZE>(
        end_idx, b.template segment<POSE_VEL_BIAS_SIZE>(POSE_VEL_BIAS_SIZE));
  }

  virtual void scaleJp_cols(const VecX& jacobian_scaling) {
    int64_t start_t = imu_meas->get_start_t_ns();
    int64_t end_t = imu_meas->get_start_t_ns() + imu_meas->get_dt_ns();

    const size_t start_idx = aom.abs_order_map.at(start_t).first;
    const size_t end_idx = aom.abs_order_map.at(end_t).first;

    Jp.template topLeftCorner<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>() *=
        jacobian_scaling.template segment<POSE_VEL_BIAS_SIZE>(start_idx)
            .asDiagonal();

    Jp.template topRightCorner<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>() *=
        jacobian_scaling.template segment<POSE_VEL_BIAS_SIZE>(end_idx)
            .asDiagonal();
  }

  virtual void addJp_diag2(VecX& res) const {
    int64_t start_t = imu_meas->get_start_t_ns();
    int64_t end_t = imu_meas->get_start_t_ns() + imu_meas->get_dt_ns();

    const size_t start_idx = aom.abs_order_map.at(start_t).first;
    const size_t end_idx = aom.abs_order_map.at(end_t).first;

    res.template segment<POSE_VEL_BIAS_SIZE>(start_idx) +=
        Jp.template topLeftCorner<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>()
            .colwise()
            .squaredNorm();

    res.template segment<POSE_VEL_BIAS_SIZE>(end_idx) +=
        Jp.template topRightCorner<POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE>()
            .colwise()
            .squaredNorm();
  }

  virtual void backSubstitute(const VecX& pose_inc, Scalar& l_diff) {
    int64_t start_t = imu_meas->get_start_t_ns();
    int64_t end_t = imu_meas->get_start_t_ns() + imu_meas->get_dt_ns();

    const size_t start_idx = aom.abs_order_map.at(start_t).first;
    const size_t end_idx = aom.abs_order_map.at(end_t).first;

    VecX pose_inc_reduced(2 * POSE_VEL_BIAS_SIZE);
    pose_inc_reduced.template head<POSE_VEL_BIAS_SIZE>() =
        pose_inc.template segment<POSE_VEL_BIAS_SIZE>(start_idx);
    pose_inc_reduced.template tail<POSE_VEL_BIAS_SIZE>() =
        pose_inc.template segment<POSE_VEL_BIAS_SIZE>(end_idx);

    // We want to compute the model cost change. The model function is
    //
    //     L(inc) = F(x) + incT JT r + 0.5 incT JT J inc
    //
    // and thus the expect decrease in cost for the computed increment is
    //
    //     l_diff = L(0) - L(inc)
    //            = - incT JT r - 0.5 incT JT J inc.
    //            = - incT JT (r + 0.5 J inc)
    //            = - (J inc)T (r + 0.5 (J inc))

    VecX Jinc = Jp * pose_inc_reduced;
    l_diff -= Jinc.transpose() * (Scalar(0.5) * Jinc + r);
  }

 protected:
  std::pair<Scalar, Scalar> LinearizeBiasRandomWalk(
      const PoseVelBiasStateWithLin<Scalar>& start_state,
      const PoseVelBiasStateWithLin<Scalar>& end_state) {
    const size_t start_idx = 0;
    const size_t end_idx = POSE_VEL_BIAS_SIZE;

    // difference between biases
    Scalar dt = imu_meas->get_dt_ns() * Scalar(1e-9);

    Vec3 gyro_bias_weight_dt =
        imu_lin_data->gyro_bias_weight_sqrt / std::sqrt(dt);
    Vec3 res_bg =
        start_state.getState().bias_gyro - end_state.getState().bias_gyro;

    Jp.template block<3, 3>(9, start_idx + 9) =
        gyro_bias_weight_dt.asDiagonal();
    Jp.template block<3, 3>(9, end_idx + 9) =
        (-gyro_bias_weight_dt).asDiagonal();

    r.template segment<3>(9) += gyro_bias_weight_dt.asDiagonal() * res_bg;

    Scalar bg_error =
        Scalar(0.5) * (gyro_bias_weight_dt.asDiagonal() * res_bg).squaredNorm();

    Vec3 accel_bias_weight_dt =
        imu_lin_data->accel_bias_weight_sqrt / std::sqrt(dt);
    Vec3 res_ba =
        start_state.getState().bias_accel - end_state.getState().bias_accel;

    Jp.template block<3, 3>(12, start_idx + 12) =
        accel_bias_weight_dt.asDiagonal();
    Jp.template block<3, 3>(12, end_idx + 12) =
        (-accel_bias_weight_dt).asDiagonal();

    r.template segment<3>(12) += accel_bias_weight_dt.asDiagonal() * res_ba;

    Scalar ba_error =
        Scalar(0.5) *
        (accel_bias_weight_dt.asDiagonal() * res_ba).squaredNorm();

    return {bg_error, ba_error};
  }

  std::array<FrameId, 2> frame_ids;
  MatX Jp;
  VecX r;

  const IntegratedImuMeasurement<Scalar>* imu_meas;
  const ImuLinData<Scalar>* imu_lin_data;
  const AbsOrderMap& aom;
};  // namespace basalt

template <class Scalar_>
class ImuBlockGS : public ImuBlock<Scalar_> {
 public:
  using Scalar = Scalar_;
  using Base = ImuBlock<Scalar>;
  using VecX = typename Base::VecX;
  using MatX = typename Base::MatX;
  using Vec3 = typename Base::Vec3;
  using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
  using MatGs = Eigen::Matrix<Scalar, POSE_VEL_BIAS_SIZE, GRAVITY_SCALE_SIZE>;

  // With scaleFreeRows, the factor keeps all rows with the scale column zeroed
  // when the whitened scale column norm is at most scaleFreeTol, and the
  // rotation rows weighted by their marginal covariance otherwise.
  ImuBlockGS(const IntegratedImuMeasurement<Scalar>* meas,
             const ImuLinData<Scalar>* imu_lin_data, const AbsOrderMap& aom,
             const GravityScaleStateWithLin<Scalar>* gs,
             bool scaleFreeRows = false, Scalar scaleFreeTol = Scalar(0))
      : Base(meas, imu_lin_data, aom),
        mpGs(gs),
        mpGsIdx(aom.abs_order_map.at(kGravityScaleStateId).first),
        mpScaleFreeRows(scaleFreeRows),
        mpScaleFreeTol(scaleFreeTol) {
    mpJgs.setZero();
  }

  Scalar linearizeImu(
      const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<Scalar>>&
          frame_states) override {
    this->Jp.setZero();
    this->r.setZero();
    mpJgs.setZero();

    const int64_t start_t = this->imu_meas->get_start_t_ns();
    const int64_t end_t =
        this->imu_meas->get_start_t_ns() + this->imu_meas->get_dt_ns();

    const size_t start_idx = 0;
    const size_t end_idx = POSE_VEL_BIAS_SIZE;

    PoseVelBiasStateWithLin<Scalar> start_state = frame_states.at(start_t);
    PoseVelBiasStateWithLin<Scalar> end_state = frame_states.at(end_t);

    typename IntegratedImuMeasurement<Scalar>::MatNN d_res_d_start, d_res_d_end;
    typename IntegratedImuMeasurement<Scalar>::MatN3 d_res_d_bg, d_res_d_ba,
        d_res_d_gD;
    typename PoseVelState<Scalar>::VecN d_res_d_logscale;

    const Scalar g_norm = this->imu_lin_data->g.norm();
    const Vec3 g_D_lin = g_norm * mpGs->mDirLin;

    typename PoseVelState<Scalar>::VecN res = this->imu_meas->residualGS(
        start_state.getStateLin(), g_D_lin, end_state.getStateLin(),
        start_state.getStateLin().bias_gyro,
        start_state.getStateLin().bias_accel, mpGs->GetScaleLin(),
        &d_res_d_start, &d_res_d_end, &d_res_d_bg, &d_res_d_ba, &d_res_d_gD,
        &d_res_d_logscale);

    if (start_state.isLinearized() || end_state.isLinearized() ||
        mpGs->mLinearized) {
      res = this->imu_meas->residualGS(
          start_state.getState(), Vec3(g_norm * mpGs->GetDir()),
          end_state.getState(), start_state.getState().bias_gyro,
          start_state.getState().bias_accel, mpGs->GetScale());
    }

    Scalar imu_error = Scalar(0.5) *
                       (this->imu_meas->get_sqrt_cov_inv() * res).squaredNorm();

    this->Jp.template block<9, 9>(0, start_idx) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_start;
    this->Jp.template block<9, 9>(0, end_idx) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_end;

    this->Jp.template block<9, 3>(0, start_idx + 9) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_bg;
    this->Jp.template block<9, 3>(0, start_idx + 12) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_ba;

    const Eigen::Matrix<Scalar, 3, 2> d_gD_d_theta =
        -Sophus::SO3<Scalar>::hat(g_D_lin) * mpGs->mBasisLin;

    mpJgs.template block<9, 2>(0, 0) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_gD * d_gD_d_theta;
    mpJgs.template block<9, 1>(0, 2) =
        this->imu_meas->get_sqrt_cov_inv() * d_res_d_logscale;

    this->r.template segment<9>(0) = this->imu_meas->get_sqrt_cov_inv() * res;

    if (mpScaleFreeRows) {
      mpScaleColNorm = mpJgs.col(2).norm();
      mpUsedFullRows = mpScaleColNorm <= mpScaleFreeTol;
      if (mpUsedFullRows) {
        mpJgs.col(2).setZero();
      } else {
        const Mat3 sqrt_inv_cov_rot =
            Eigen::LLT<Mat3>(
                this->imu_meas->get_cov().template block<3, 3>(3, 3))
                .matrixL()
                .solve(Mat3::Identity());

        this->Jp.template topRows<9>().setZero();
        this->r.template head<9>().setZero();
        mpJgs.setZero();

        this->Jp.template block<3, 9>(3, start_idx) =
            sqrt_inv_cov_rot * d_res_d_start.template middleRows<3>(3);
        this->Jp.template block<3, 9>(3, end_idx) =
            sqrt_inv_cov_rot * d_res_d_end.template middleRows<3>(3);
        this->Jp.template block<3, 3>(3, start_idx + 9) =
            sqrt_inv_cov_rot * d_res_d_bg.template middleRows<3>(3);
        this->Jp.template block<3, 3>(3, start_idx + 12) =
            sqrt_inv_cov_rot * d_res_d_ba.template middleRows<3>(3);
        this->r.template segment<3>(3) =
            sqrt_inv_cov_rot * res.template segment<3>(3);

        imu_error = Scalar(0.5) * this->r.template segment<3>(3).squaredNorm();
      }
    }

    const std::pair<Scalar, Scalar> bias_error =
        this->LinearizeBiasRandomWalk(start_state, end_state);

    return imu_error + bias_error.first + bias_error.second;
  }

  void add_dense_Q2Jp_Q2r(MatX& Q2Jp, VecX& Q2r,
                          size_t row_start_idx) const override {
    Base::add_dense_Q2Jp_Q2r(Q2Jp, Q2r, row_start_idx);

    Q2Jp.template block<POSE_VEL_BIAS_SIZE, GRAVITY_SCALE_SIZE>(row_start_idx,
                                                                mpGsIdx) +=
        mpJgs;
  }

  void add_dense_H_b(DenseAccumulator<Scalar>& accum) const override {
    Base::add_dense_H_b(accum);

    const int64_t start_t = this->imu_meas->get_start_t_ns();
    const int64_t end_t =
        this->imu_meas->get_start_t_ns() + this->imu_meas->get_dt_ns();

    const size_t start_idx = this->aom.abs_order_map.at(start_t).first;
    const size_t end_idx = this->aom.abs_order_map.at(end_t).first;

    const Eigen::Matrix<Scalar, GRAVITY_SCALE_SIZE, GRAVITY_SCALE_SIZE> H_gg =
        mpJgs.transpose() * mpJgs;
    const Eigen::Matrix<Scalar, GRAVITY_SCALE_SIZE, POSE_VEL_BIAS_SIZE> H_gs =
        mpJgs.transpose() *
        this->Jp.template leftCols<POSE_VEL_BIAS_SIZE>();
    const Eigen::Matrix<Scalar, GRAVITY_SCALE_SIZE, POSE_VEL_BIAS_SIZE> H_ge =
        mpJgs.transpose() *
        this->Jp.template rightCols<POSE_VEL_BIAS_SIZE>();
    const Eigen::Matrix<Scalar, GRAVITY_SCALE_SIZE, 1> b_g =
        mpJgs.transpose() * this->r;

    accum.template addH<GRAVITY_SCALE_SIZE, GRAVITY_SCALE_SIZE>(
        mpGsIdx, mpGsIdx, H_gg);
    accum.template addH<GRAVITY_SCALE_SIZE, POSE_VEL_BIAS_SIZE>(
        mpGsIdx, start_idx, H_gs);
    accum.template addH<POSE_VEL_BIAS_SIZE, GRAVITY_SCALE_SIZE>(
        start_idx, mpGsIdx, H_gs.transpose());
    accum.template addH<GRAVITY_SCALE_SIZE, POSE_VEL_BIAS_SIZE>(
        mpGsIdx, end_idx, H_ge);
    accum.template addH<POSE_VEL_BIAS_SIZE, GRAVITY_SCALE_SIZE>(
        end_idx, mpGsIdx, H_ge.transpose());
    accum.template addB<GRAVITY_SCALE_SIZE>(mpGsIdx, b_g);
  }

  void scaleJp_cols(const VecX& jacobian_scaling) override {
    Base::scaleJp_cols(jacobian_scaling);

    mpJgs *= jacobian_scaling.template segment<GRAVITY_SCALE_SIZE>(mpGsIdx)
                 .asDiagonal();
  }

  void addJp_diag2(VecX& res) const override {
    Base::addJp_diag2(res);

    res.template segment<GRAVITY_SCALE_SIZE>(mpGsIdx) +=
        mpJgs.colwise().squaredNorm().transpose();
  }

  void backSubstitute(const VecX& pose_inc, Scalar& l_diff) override {
    const int64_t start_t = this->imu_meas->get_start_t_ns();
    const int64_t end_t =
        this->imu_meas->get_start_t_ns() + this->imu_meas->get_dt_ns();

    const size_t start_idx = this->aom.abs_order_map.at(start_t).first;
    const size_t end_idx = this->aom.abs_order_map.at(end_t).first;

    VecX pose_inc_reduced(2 * POSE_VEL_BIAS_SIZE);
    pose_inc_reduced.template head<POSE_VEL_BIAS_SIZE>() =
        pose_inc.template segment<POSE_VEL_BIAS_SIZE>(start_idx);
    pose_inc_reduced.template tail<POSE_VEL_BIAS_SIZE>() =
        pose_inc.template segment<POSE_VEL_BIAS_SIZE>(end_idx);

    VecX Jinc = this->Jp * pose_inc_reduced +
                mpJgs * pose_inc.template segment<GRAVITY_SCALE_SIZE>(mpGsIdx);
    l_diff -= Jinc.transpose() * (Scalar(0.5) * Jinc + this->r);
  }

  // False when the last linearisation kept the rotation rows only.
  bool UsedFullRows() const { return mpUsedFullRows; }
  // Whitened scale column norm of the last scale free linearisation.
  Scalar ScaleColNorm() const { return mpScaleColNorm; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const GravityScaleStateWithLin<Scalar>* mpGs;
  size_t mpGsIdx;
  MatGs mpJgs;
  bool mpScaleFreeRows;
  Scalar mpScaleFreeTol;
  bool mpUsedFullRows = true;
  Scalar mpScaleColNorm = 0;
};

}  // namespace basalt
