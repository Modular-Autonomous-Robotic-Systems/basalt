

#include <basalt/imu/preintegration.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/utils/ba_utils.h>
#include <basalt/vi_estimator/sc_ba_base.h>
#include <basalt/linearization/imu_block.hpp>
#include <basalt/vi_estimator/delayed_marginalisation.h>
#include <basalt/vi_estimator/marg_helper.h>

#include <iostream>
#include <random>

#include "gtest/gtest.h"
#include "test_utils.h"

static const double accel_std_dev = 0.23;
static const double gyro_std_dev = 0.0027;

// Smaller noise for testing
// static const double accel_std_dev = 0.00023;
// static const double gyro_std_dev = 0.0000027;

std::random_device rd{};
std::mt19937 gen{rd()};

std::normal_distribution<> gyro_noise_dist{0, gyro_std_dev};
std::normal_distribution<> accel_noise_dist{0, accel_std_dev};

#ifdef BASALT_INSTANTIATIONS_DOUBLE
TEST(VioTestSuite, ImuNullspace2Test) {
  int num_knots = 15;

  Eigen::Vector3d bg, ba;
  bg = Eigen::Vector3d::Random() / 100;
  ba = Eigen::Vector3d::Random() / 10;

  basalt::IntegratedImuMeasurement<double> imu_meas(0, bg, ba);

  basalt::Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  basalt::PoseVelBiasState<double> state0, state1, state1_gt;

  state0.t_ns = 0;
  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));
  state0.bias_gyro = bg;
  state0.bias_accel = ba;

  Eigen::Vector3d accel_cov, gyro_cov;
  accel_cov.setConstant(accel_std_dev * accel_std_dev);
  gyro_cov.setConstant(gyro_std_dev * gyro_std_dev);

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e8);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - basalt::constants::g);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    basalt::ImuData<double> data;
    data.accel = accel_body + ba;
    data.gyro = rot_vel_body + bg;

    data.accel[0] += accel_noise_dist(gen);
    data.accel[1] += accel_noise_dist(gen);
    data.accel[2] += accel_noise_dist(gen);

    data.gyro[0] += gyro_noise_dist(gen);
    data.gyro[1] += gyro_noise_dist(gen);
    data.gyro[2] += gyro_noise_dist(gen);

    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas.integrate(data, accel_cov, gyro_cov);
  }

  state1.t_ns = imu_meas.get_dt_ns();
  state1.T_w_i = gt_spline.pose(imu_meas.get_dt_ns()) *
                 Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  state1.vel_w_i = gt_spline.transVelWorld(imu_meas.get_dt_ns()) +
                   Sophus::Vector3d::Random() / 10;
  state1.bias_gyro = bg;
  state1.bias_accel = ba;

  Eigen::Vector3d gyro_weight_sqrt;
  gyro_weight_sqrt.setConstant(1e3);

  Eigen::Vector3d accel_weight_sqrt;
  accel_weight_sqrt.setConstant(1e3);

  Eigen::aligned_map<int64_t, basalt::IntegratedImuMeasurement<double>>
      imu_meas_vec;
  Eigen::aligned_map<int64_t, basalt::PoseVelBiasStateWithLin<double>>
      frame_states;
  Eigen::aligned_map<int64_t, basalt::PoseStateWithLin<double>> frame_poses;

  imu_meas_vec[state0.t_ns] = imu_meas;
  frame_states[state0.t_ns] = basalt::PoseVelBiasStateWithLin<double>(state0);
  frame_states[state1.t_ns] = basalt::PoseVelBiasStateWithLin<double>(state1);

  int asize = 30;

  basalt::AbsOrderMap aom;
  aom.total_size = 30;
  aom.items = 2;
  aom.abs_order_map[state0.t_ns] = std::make_pair(0, 15);
  aom.abs_order_map[state1.t_ns] = std::make_pair(15, 15);

  basalt::ImuLinData<double> ild = {basalt::constants::g,
                                    gyro_weight_sqrt,
                                    accel_weight_sqrt,
                                    {std::make_pair(state0.t_ns, &imu_meas)}};

  basalt::DenseAccumulator<double> accum;
  accum.reset(aom.total_size);

  basalt::ImuBlock<double> ib(&imu_meas, &ild, aom);
  double e0 = ib.linearizeImu(frame_states);
  ib.add_dense_H_b(accum);

  // Check quadratic approximation
  for (int i = 0; i < 10; i++) {
    Eigen::VectorXd rand_inc;
    rand_inc.setRandom(asize);
    rand_inc.normalize();
    rand_inc /= 10000;

    auto frame_states_copy = frame_states;
    frame_states_copy[state0.t_ns].applyInc(rand_inc.segment<15>(0));
    frame_states_copy[state1.t_ns].applyInc(rand_inc.segment<15>(15));

    double imu_error_u, bg_error_u, ba_error_u;
    basalt::ScBundleAdjustmentBase<double>::computeImuError(
        aom, imu_error_u, bg_error_u, ba_error_u, frame_states_copy,
        imu_meas_vec, gyro_weight_sqrt.array().square(),
        accel_weight_sqrt.array().square(), basalt::constants::g);

    double e1 = imu_error_u + bg_error_u + ba_error_u - e0;

    double e2 = 0.5 * rand_inc.transpose() * accum.getH() * rand_inc;
    e2 += rand_inc.transpose() * accum.getB();

    EXPECT_LE(std::abs(e1 - e2), 2e-2) << "e1 " << e1 << " e2 " << e2;
  }

  std::cout << "=========================================" << std::endl;
  Eigen::VectorXd null_res =
      basalt::ScBundleAdjustmentBase<double>::checkNullspace(
          accum.getH(), accum.getB(), aom, frame_states, frame_poses);
  std::cout << "=========================================" << std::endl;

  EXPECT_LE(std::abs(null_res[0]), 1e-8);
  EXPECT_LE(std::abs(null_res[1]), 1e-8);
  EXPECT_LE(std::abs(null_res[2]), 1e-8);
  EXPECT_LE(std::abs(null_res[5]), 1e-6);
}
#endif

#ifdef BASALT_INSTANTIATIONS_DOUBLE
TEST(VioTestSuite, ImuNullspace3Test) {
  int num_knots = 15;

  Eigen::Vector3d bg, ba;
  bg = Eigen::Vector3d::Random() / 100;
  ba = Eigen::Vector3d::Random() / 10;

  basalt::IntegratedImuMeasurement<double> imu_meas1(0, bg, ba);

  basalt::Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  basalt::PoseVelBiasState<double> state0, state1, state2;

  state0.t_ns = 0;
  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));
  state0.bias_gyro = bg;
  state0.bias_accel = ba;

  Eigen::Vector3d accel_cov, gyro_cov;
  accel_cov.setConstant(accel_std_dev * accel_std_dev);
  gyro_cov.setConstant(gyro_std_dev * gyro_std_dev);

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - basalt::constants::g);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    basalt::ImuData<double> data;
    data.accel = accel_body + ba;
    data.gyro = rot_vel_body + bg;

    data.accel[0] += accel_noise_dist(gen);
    data.accel[1] += accel_noise_dist(gen);
    data.accel[2] += accel_noise_dist(gen);

    data.gyro[0] += gyro_noise_dist(gen);
    data.gyro[1] += gyro_noise_dist(gen);
    data.gyro[2] += gyro_noise_dist(gen);

    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas1.integrate(data, accel_cov, gyro_cov);
  }

  basalt::IntegratedImuMeasurement<double> imu_meas2(imu_meas1.get_dt_ns(), bg,
                                                     ba);
  for (int64_t t_ns = imu_meas1.get_dt_ns() + dt_ns / 2;
       t_ns < int64_t(2e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - basalt::constants::g);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    basalt::ImuData<double> data;
    data.accel = accel_body + ba;
    data.gyro = rot_vel_body + bg;

    data.accel[0] += accel_noise_dist(gen);
    data.accel[1] += accel_noise_dist(gen);
    data.accel[2] += accel_noise_dist(gen);

    data.gyro[0] += gyro_noise_dist(gen);
    data.gyro[1] += gyro_noise_dist(gen);
    data.gyro[2] += gyro_noise_dist(gen);

    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas2.integrate(data, accel_cov, gyro_cov);
  }

  state1.t_ns = imu_meas1.get_dt_ns();
  state1.T_w_i = gt_spline.pose(state1.t_ns) *
                 Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  state1.vel_w_i =
      gt_spline.transVelWorld(state1.t_ns) + Sophus::Vector3d::Random() / 10;
  state1.bias_gyro = bg;
  state1.bias_accel = ba;

  state2.t_ns = imu_meas1.get_dt_ns() + imu_meas2.get_dt_ns();
  state2.T_w_i = gt_spline.pose(state2.t_ns) *
                 Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  state2.vel_w_i =
      gt_spline.transVelWorld(state2.t_ns) + Sophus::Vector3d::Random() / 10;
  state2.bias_gyro = bg;
  state2.bias_accel = ba;

  Eigen::Vector3d gyro_weight_sqrt;
  gyro_weight_sqrt.setConstant(1e3);

  Eigen::Vector3d accel_weight_sqrt;
  accel_weight_sqrt.setConstant(1e3);

  Eigen::aligned_map<int64_t, basalt::IntegratedImuMeasurement<double>>
      imu_meas_vec;
  Eigen::aligned_map<int64_t, basalt::PoseVelBiasStateWithLin<double>>
      frame_states;
  Eigen::aligned_map<int64_t, basalt::PoseStateWithLin<double>> frame_poses;

  imu_meas_vec[imu_meas1.get_start_t_ns()] = imu_meas1;
  imu_meas_vec[imu_meas2.get_start_t_ns()] = imu_meas2;
  frame_states[state0.t_ns] = basalt::PoseVelBiasStateWithLin<double>(state0);
  frame_states[state1.t_ns] = basalt::PoseVelBiasStateWithLin<double>(state1);
  frame_states[state2.t_ns] = basalt::PoseVelBiasStateWithLin<double>(state2);

  int asize = 45;

  basalt::AbsOrderMap aom;
  aom.total_size = asize;
  aom.items = 2;
  aom.abs_order_map[state0.t_ns] = std::make_pair(0, 15);
  aom.abs_order_map[state1.t_ns] = std::make_pair(15, 15);
  aom.abs_order_map[state2.t_ns] = std::make_pair(30, 15);

  basalt::ImuLinData<double> ild = {basalt::constants::g,
                                    gyro_weight_sqrt,
                                    accel_weight_sqrt,
                                    {
                                        std::make_pair(state0.t_ns, &imu_meas1),
                                        std::make_pair(state1.t_ns, &imu_meas2),
                                    }};

  basalt::DenseAccumulator<double> accum;
  accum.reset(aom.total_size);

  basalt::ImuBlock<double> ib1(&imu_meas1, &ild, aom),
      ib2(&imu_meas2, &ild, aom);
  ib1.linearizeImu(frame_states);
  ib2.linearizeImu(frame_states);
  ib1.add_dense_H_b(accum);
  ib2.add_dense_H_b(accum);

  std::cout << "=========================================" << std::endl;
  Eigen::VectorXd null_res =
      basalt::ScBundleAdjustmentBase<double>::checkNullspace(
          accum.getH(), accum.getB(), aom, frame_states, frame_poses);
  std::cout << "=========================================" << std::endl;

  EXPECT_LE(std::abs(null_res[0]), 1e-8);
  EXPECT_LE(std::abs(null_res[1]), 1e-8);
  EXPECT_LE(std::abs(null_res[2]), 1e-8);
  EXPECT_LE(std::abs(null_res[5]), 1e-6);
}
#endif

#ifdef BASALT_INSTANTIATIONS_DOUBLE
TEST(VioTestSuite, ImuBlockGSScaleFreeRowsTest) {
  Eigen::Vector3d bg = Eigen::Vector3d::Random() / 100;
  Eigen::Vector3d ba = Eigen::Vector3d::Random() / 10;

  basalt::IntegratedImuMeasurement<double> imu_meas(0, bg, ba);

  basalt::Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(15);

  Eigen::Vector3d accel_cov, gyro_cov;
  accel_cov.setConstant(accel_std_dev * accel_std_dev);
  gyro_cov.setConstant(gyro_std_dev * gyro_std_dev);

  const int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2; t_ns < int64_t(1e8); t_ns += dt_ns) {
    const Sophus::SE3d pose = gt_spline.pose(t_ns);
    basalt::ImuData<double> data;
    data.accel = pose.so3().inverse() * (gt_spline.transAccelWorld(t_ns) -
                                         basalt::constants::g) +
                 ba;
    data.gyro = gt_spline.rotVelBody(t_ns) + bg;
    data.t_ns = t_ns + dt_ns / 2;
    imu_meas.integrate(data, accel_cov, gyro_cov);
  }

  basalt::PoseVelBiasState<double> state0, state1;
  state0.t_ns = 0;
  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));
  state0.bias_gyro = bg;
  state0.bias_accel = ba;

  state1.t_ns = imu_meas.get_dt_ns();
  state1.T_w_i = gt_spline.pose(state1.t_ns) *
                 Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  state1.vel_w_i =
      gt_spline.transVelWorld(state1.t_ns) + Sophus::Vector3d::Random() / 10;
  state1.bias_gyro = bg;
  state1.bias_accel = ba;

  Eigen::aligned_map<int64_t, basalt::PoseVelBiasStateWithLin<double>>
      frame_states;
  frame_states[state0.t_ns] = basalt::PoseVelBiasStateWithLin<double>(state0);
  frame_states[state1.t_ns] = basalt::PoseVelBiasStateWithLin<double>(state1);

  basalt::AbsOrderMap aom;
  aom.abs_order_map[basalt::kGravityScaleStateId] =
      std::make_pair(0, basalt::GRAVITY_SCALE_SIZE);
  aom.abs_order_map[state0.t_ns] = std::make_pair(3, 15);
  aom.abs_order_map[state1.t_ns] = std::make_pair(18, 15);
  aom.total_size = 33;
  aom.items = 3;

  Eigen::Vector3d weight_sqrt;
  weight_sqrt.setConstant(1e3);
  basalt::ImuLinData<double> ild = {basalt::constants::g,
                                    weight_sqrt,
                                    weight_sqrt,
                                    {std::make_pair(state0.t_ns, &imu_meas)}};

  basalt::GravityScaleStateWithLin<double> gs;

  // Rotation rows only, forced by a zero tolerance.
  basalt::ImuBlockGS<double> ib_rot(&imu_meas, &ild, aom, &gs, true, 0.0);
  const double e_rot = ib_rot.linearizeImu(frame_states);
  EXPECT_FALSE(ib_rot.UsedFullRows());

  const Eigen::Matrix<double, 9, 1> res =
      imu_meas.residual(state0, basalt::constants::g, state1, bg, ba);
  const Eigen::Vector3d r_rot = res.segment<3>(3);
  const Eigen::Matrix3d cov_rot = imu_meas.get_cov().block<3, 3>(3, 3);
  const double e_expected = 0.5 * r_rot.dot(cov_rot.ldlt().solve(r_rot));
  EXPECT_NEAR(e_rot, e_expected, 1e-9 * (1.0 + e_expected));

  basalt::DenseAccumulator<double> acc_rot;
  acc_rot.reset(aom.total_size);
  ib_rot.add_dense_H_b(acc_rot);
  EXPECT_LE(acc_rot.getH().topRows(3).norm(), 1e-12);
  EXPECT_LE(acc_rot.getB().head(3).norm(), 1e-12);
  EXPECT_LE(acc_rot.getH().block(9, 0, 3, 33).norm(), 1e-12);
  EXPECT_LE(acc_rot.getH().block(24, 0, 3, 33).norm(), 1e-12);

  // All rows with the scale column removed, forced by a large tolerance.
  basalt::ImuBlockGS<double> ib_full(&imu_meas, &ild, aom, &gs, true, 1e12);
  ib_full.linearizeImu(frame_states);
  EXPECT_TRUE(ib_full.UsedFullRows());

  basalt::ImuBlockGS<double> ib_ref(&imu_meas, &ild, aom, &gs);
  ib_ref.linearizeImu(frame_states);

  basalt::DenseAccumulator<double> acc_full, acc_ref;
  acc_full.reset(aom.total_size);
  acc_ref.reset(aom.total_size);
  ib_full.add_dense_H_b(acc_full);
  ib_ref.add_dense_H_b(acc_ref);

  Eigen::MatrixXd H_ref = acc_ref.getH();
  Eigen::VectorXd b_ref = acc_ref.getB();
  H_ref.row(2).setZero();
  H_ref.col(2).setZero();
  b_ref[2] = 0;
  EXPECT_LE((acc_full.getH() - H_ref).norm(), 1e-9 * (1.0 + H_ref.norm()));
  EXPECT_LE((acc_full.getB() - b_ref).norm(), 1e-9 * (1.0 + b_ref.norm()));
}
#endif

TEST(VioTestSuite, RelPoseTest) {
  Sophus::SE3d T_w_i_h = Sophus::se3_expd(Sophus::Vector6d::Random());
  Sophus::SE3d T_w_i_t = Sophus::se3_expd(Sophus::Vector6d::Random());

  Sophus::SE3d T_i_c_h = Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  Sophus::SE3d T_i_c_t = Sophus::se3_expd(Sophus::Vector6d::Random() / 10);

  Sophus::Matrix6d d_rel_d_h, d_rel_d_t;

  Sophus::SE3d T_t_h_sophus = basalt::computeRelPose(
      T_w_i_h, T_i_c_h, T_w_i_t, T_i_c_t, &d_rel_d_h, &d_rel_d_t);

  {
    Sophus::Vector6d x0;
    x0.setZero();
    test_jacobian(
        "d_rel_d_h", d_rel_d_h,
        [&](const Sophus::Vector6d& x) {
          Sophus::SE3d T_w_h_new = T_w_i_h;
          basalt::PoseState<double>::incPose(x, T_w_h_new);

          Sophus::SE3d T_t_h_sophus_new =
              basalt::computeRelPose(T_w_h_new, T_i_c_h, T_w_i_t, T_i_c_t);

          return Sophus::se3_logd(T_t_h_sophus_new * T_t_h_sophus.inverse());
        },
        x0);
  }

  {
    Sophus::Vector6d x0;
    x0.setZero();
    test_jacobian(
        "d_rel_d_t", d_rel_d_t,
        [&](const Sophus::Vector6d& x) {
          Sophus::SE3d T_w_t_new = T_w_i_t;
          basalt::PoseState<double>::incPose(x, T_w_t_new);

          Sophus::SE3d T_t_h_sophus_new =
              basalt::computeRelPose(T_w_i_h, T_i_c_h, T_w_t_new, T_i_c_t);
          return Sophus::se3_logd(T_t_h_sophus_new * T_t_h_sophus.inverse());
        },
        x0);
  }
}

TEST(VioTestSuite, LinearizePointsTest) {
  basalt::ExtendedUnifiedCamera<double> cam =
      basalt::ExtendedUnifiedCamera<double>::getTestProjections()[0];

  basalt::Keypoint<double> kpt_pos;

  Eigen::Vector4d point3d;
  cam.unproject(Eigen::Vector2d::Random() * 50, point3d);
  kpt_pos.direction = basalt::StereographicParam<double>::project(point3d);
  kpt_pos.inv_dist = 0.1231231;

  Sophus::SE3d T_w_h = Sophus::se3_expd(Sophus::Vector6d::Random() / 100);
  Sophus::SE3d T_w_t = Sophus::se3_expd(Sophus::Vector6d::Random() / 100);
  T_w_t.translation()[0] += 0.1;

  Sophus::SE3d T_t_h_sophus = T_w_t.inverse() * T_w_h;
  Eigen::Matrix4d T_t_h = T_t_h_sophus.matrix();

  Eigen::Vector4d p_trans;
  p_trans = basalt::StereographicParam<double>::unproject(kpt_pos.direction);
  p_trans(3) = kpt_pos.inv_dist;

  p_trans = T_t_h * p_trans;

  basalt::KeypointObservation<double> kpt_obs;
  cam.project(p_trans, kpt_obs.pos);

  Eigen::Vector2d res;
  Eigen::Matrix<double, 2, 6> d_res_d_xi;
  Eigen::Matrix<double, 2, 3> d_res_d_p;

  basalt::linearizePoint(kpt_obs.pos, kpt_pos, T_t_h, cam, res, &d_res_d_xi,
                         &d_res_d_p);

  {
    Sophus::Vector6d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_xi", d_res_d_xi,
        [&](const Sophus::Vector6d& x) {
          Eigen::Matrix4d T_t_h_new =
              (Sophus::se3_expd(x) * T_t_h_sophus).matrix();

          Eigen::Vector2d res;
          basalt::linearizePoint(kpt_obs.pos, kpt_pos, T_t_h_new, cam, res);

          return res;
        },
        x0);
  }

  {
    Eigen::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_p", d_res_d_p,
        [&](const Eigen::Vector3d& x) {
          basalt::Keypoint kpt_pos_new = kpt_pos;

          kpt_pos_new.direction += x.head<2>();
          kpt_pos_new.inv_dist += x[2];

          Eigen::Vector2d res;
          basalt::linearizePoint(kpt_obs.pos, kpt_pos_new, T_t_h, cam, res);

          return res;
        },
        x0);
  }
}

#ifdef BASALT_INSTANTIATIONS_DOUBLE
namespace {

// A chain of states 0..N-1 marginalised one per step, each step with one
// inertial factor and random visual rows on the two poses, for the tests of
// plans/delayed_marginalisation.md §3.2 Change 10.
struct DelayedMargChain {
  static constexpr int N = 8;
  static constexpr int64_t kDt = 50000000;

  basalt::Se3Spline<5> spline{int64_t(10e9)};
  basalt::DelayedMargValues<double> values;
  basalt::DelayedMargPrior<double> boundary;
  std::vector<std::shared_ptr<const basalt::DelayedMargStep<double>>> steps;
  basalt::GravityScaleStateWithLin<double> gs;
  Eigen::Vector3d weight = Eigen::Vector3d::Constant(1e2);
  basalt::GravityScaleCorrectionLog::Ptr log =
      std::make_shared<basalt::GravityScaleCorrectionLog>();

  static basalt::AbsOrderMap Order(std::initializer_list<int64_t> ids) {
    basalt::AbsOrderMap o;
    o.abs_order_map[basalt::kGravityScaleStateId] = {0, 3};
    o.total_size = 3;
    o.items = 1;
    for (int64_t id : ids) {
      o.abs_order_map[id] = {int(o.total_size), 15};
      o.total_size += 15;
      o.items++;
    }
    return o;
  }

  DelayedMargChain() {
    spline.genRandomTrajectory(15);
    const Eigen::Vector3d bg = Eigen::Vector3d::Random() / 100;
    const Eigen::Vector3d ba = Eigen::Vector3d::Random() / 10;

    for (int i = 0; i < N; i++) {
      const int64_t t = i * kDt;
      basalt::PoseVelBiasState<double> lin(
          t, spline.pose(t) * Sophus::se3_expd(Sophus::Vector6d::Random() / 50),
          spline.transVelWorld(t) + Eigen::Vector3d::Random() / 20, bg, ba);
      basalt::PoseVelBiasStateWithLin<double> st(t, lin.T_w_i, lin.vel_w_i,
                                                 bg, ba, true);
      Eigen::Matrix<double, 15, 1> d = Eigen::Matrix<double, 15, 1>::Random();
      st.applyInc(d / 100);
      if (i == N - 1) {
        values.mvWindowStates[t] = st;
      } else {
        values.mvRecords[t].mState = st;
        values.mvRecords[t].mLastStep = i;
      }
    }

    gs.ApplyInc(Eigen::Vector3d(0.03, -0.02, 0.1));
    gs.Reanchor(gs.GetDir(), gs.GetBasis(), gs.GetSigma());
    gs.mDelta = Eigen::Vector3d(0.01, 0.02, -0.05);

    boundary.mOrder = Order({0});
    boundary.mH.setZero(15, 18);
    boundary.mH.diagonal().setZero();
    for (int c : {3, 4, 5, 8, 12, 13, 14, 15, 16, 17}) boundary.mH(c - 3, c) = 1e2;
    boundary.mB = Eigen::VectorXd::Random(15);
    boundary.mAnchor.Set(gs);

    for (int i = 0; i + 1 < N; i++) {
      Eigen::Vector3d accel_cov = Eigen::Vector3d::Constant(0.04);
      Eigen::Vector3d gyro_cov = Eigen::Vector3d::Constant(1e-4);
      basalt::IntegratedImuMeasurement<double> meas(i * kDt, bg, ba);
      for (int64_t t = i * kDt + 2500000; t < (i + 1) * kDt; t += 5000000) {
        basalt::ImuData<double> data;
        const Sophus::SE3d pose = spline.pose(t);
        data.accel = pose.so3().inverse() *
                         (spline.transAccelWorld(t) - basalt::constants::g) +
                     ba;
        data.gyro = spline.rotVelBody(t) + bg;
        data.t_ns = t + 2500000;
        meas.integrate(data, accel_cov, gyro_cov);
      }

      basalt::DelayedMargStep<double> step;
      step.mIndex = i;
      step.mEpoch = 0;
      step.mOrder = Order({i * kDt, (i + 1) * kDt});
      step.mOrderNew = Order({(i + 1) * kDt});
      for (int c = 0; c < 33; c++) {
        (c >= 3 && c < 18 ? step.mIdxMarg : step.mIdxKeep).insert(c);
      }
      step.mVisJ.setZero(10, 33);
      step.mVisJ.middleCols(3, 6).setRandom();
      step.mVisJ.middleCols(18, 6).setRandom();
      step.mVisJ *= 30;
      step.mVisR = Eigen::VectorXd::Random(10);
      step.mvImu.push_back(meas);
      step.mvImu.back().get_sqrt_cov_inv();
      steps.push_back(
          std::make_shared<const basalt::DelayedMargStep<double>>(step));
    }
  }

  basalt::DelayedMargJob<double> Job() const {
    basalt::DelayedMargJob<double> job;
    job.mBoundary = boundary;
    job.mvSteps = steps;
    job.mValues = values;
    job.mGs = gs;
    job.mG = basalt::constants::g;
    job.mGyroBiasWeightSqrt = weight;
    job.mAccelBiasWeightSqrt = weight;
    job.mLog = log;
    job.mEpoch = log->LatestEpoch();
    return job;
  }

  // Every row of the chain at the current values over the joint order.
  void Joint(Eigen::MatrixXd& J, Eigen::VectorXd& r,
             basalt::AbsOrderMap& order) const {
    std::vector<int64_t> ids;
    for (int i = 0; i < N; i++) ids.push_back(i * kDt);
    order = Order({});
    for (int64_t id : ids) {
      order.abs_order_map[id] = {int(order.total_size), 15};
      order.total_size += 15;
      order.items++;
    }
    basalt::GravityScaleStateWithLin<double> gsNow = gs;
    gsNow.Reanchor(gs.GetDir(), gs.GetBasis(), gs.GetSigma());

    const int rows = 15 + int(steps.size()) * (10 + 15);
    J.setZero(rows, order.total_size);
    r.setZero(rows);

    basalt::GravityScaleStateWithLin<double> gsB = gsNow;
    gsB.Reanchor(boundary.mAnchor.mDir, boundary.mAnchor.mBasis,
                 boundary.mAnchor.mSigma);
    J.topLeftCorner(15, 18) = boundary.mH;
    r.head(15) = boundary.mB +
                 boundary.mH * values.Delta(boundary.mOrder, gsB.GetDelta());

    int row = 15;
    for (const auto& s : steps) {
      const int c0 = order.abs_order_map.at(s->mvImu[0].get_start_t_ns()).first;
      J.block(row, 0, 10, 3) = s->mVisJ.leftCols(3);
      J.block(row, c0, 10, 30) = s->mVisJ.rightCols(30);
      r.segment(row, 10) =
          s->mVisR + s->mVisJ * values.Delta(s->mOrder, Eigen::Vector3d::Zero());
      row += 10;

      basalt::ImuLinData<double> ild = {basalt::constants::g, weight, weight, {}};
      Eigen::aligned_map<int64_t, basalt::PoseVelBiasStateWithLin<double>> st;
      const int64_t t0 = s->mvImu[0].get_start_t_ns();
      const int64_t t1 = t0 + s->mvImu[0].get_dt_ns();
      st[t0] = values.State(t0);
      st[t1] = values.State(t1);
      basalt::ImuBlockGS<double> block(&s->mvImu[0], &ild, order, &gsNow);
      block.linearizeImu(st);
      block.add_dense_Q2Jp_Q2r(J, r, row);
      row += 15;
    }
  }
};

}  // namespace

TEST(VioTestSuite, DelayedMargReadvanceJointTest) {
  DelayedMargChain chain;
  basalt::DelayedMargJob<double> job = chain.Job();
  job.Forward(nullptr);

  Eigen::MatrixXd J;
  Eigen::VectorXd r;
  basalt::AbsOrderMap order;
  chain.Joint(J, r, order);

  const int last = order.abs_order_map.at((chain.N - 1) * chain.kDt).first;
  std::set<int> keep, marg;
  for (int c = 0; c < int(order.total_size); c++) {
    (c < 3 || c >= last ? keep : marg).insert(c);
  }
  Eigen::MatrixXd H;
  Eigen::VectorXd b;
  basalt::MargHelper<double>::marginalizeHelperSqrtToSqrt(J, r, keep, marg, H,
                                                          b);
  b -= H * chain.values.Delta(chain.steps.back()->mOrderNew,
                              Eigen::Vector3d::Zero());

  const Eigen::MatrixXd& Hs = job.mResult.mH;
  const Eigen::VectorXd& bs = job.mResult.mB;
  const Eigen::MatrixXd G_ref = H.transpose() * H;
  const Eigen::MatrixXd G_seq = Hs.transpose() * Hs;
  EXPECT_LE((G_seq - G_ref).norm(), 1e-8 * G_ref.norm());
  EXPECT_LE((Hs.transpose() * bs - H.transpose() * b).norm(),
            1e-8 * (H.transpose() * b).norm());
  EXPECT_LE(std::abs(job.mResult.mAnchor.mSigma - chain.gs.GetSigma()), 1e-12);

  // The prior depends on the block at which the inertial factors are
  // relinearised, which is what a replacement changes.
  basalt::DelayedMargJob<double> moved = chain.Job();
  moved.mGs.mDelta[2] += 0.2;
  moved.Forward(nullptr);
  const Eigen::MatrixXd G_moved = moved.mResult.mH.transpose() * moved.mResult.mH;
  EXPECT_GE((G_moved - G_ref).norm(), 1e-3 * G_ref.norm());
}

TEST(VioTestSuite, DelayedMargSmoothingJointTest) {
  DelayedMargChain chain;
  basalt::DelayedMargJob<double> job = chain.Job();
  std::vector<basalt::DelayedMargJob<double>::Elim> elims;
  job.Forward(&elims);
  job.Backward(elims);

  Eigen::MatrixXd J;
  Eigen::VectorXd r;
  basalt::AbsOrderMap order;
  chain.Joint(J, r, order);

  const int last = order.abs_order_map.at((chain.N - 1) * chain.kDt).first;
  const Eigen::MatrixXd Jd = J.middleCols(3, last - 3);
  const Eigen::VectorXd x = -(Jd.transpose() * Jd).ldlt().solve(Jd.transpose() * r);

  for (int i = 0; i + 1 < chain.N; i++) {
    const auto& inc = job.mvSmoothInc.at(i * chain.kDt);
    EXPECT_EQ(inc.second, (1u << 15) - 1);
    EXPECT_LE((inc.first - x.segment<15>(15 * i)).norm(),
              1e-6 * (1.0 + x.segment<15>(15 * i).norm()));
  }
}

TEST(VioTestSuite, DelayedMargRecentreInvarianceTest) {
  DelayedMargChain chain;
  basalt::DelayedMargJob<double> before = chain.Job();
  before.Forward(nullptr);

  const Sophus::SO3d Q = chain.gs.AlignmentRotation(basalt::constants::g);
  const double sigma_c = chain.gs.GetSigma();
  const double s = std::exp(sigma_c);
  chain.values.TransformBySim3(Q, s);
  basalt::DelayedMargTransformColumns(chain.boundary.mH,
                                      chain.boundary.mOrder, Q, s);
  chain.boundary.mAnchor.Transform(Q, sigma_c);
  chain.gs.TransformLin(Q, sigma_c);
  chain.log->Append(Q, s);

  basalt::DelayedMargJob<double> after = chain.Job();
  after.Forward(nullptr);

  Eigen::MatrixXd H_mapped = before.mResult.mH;
  basalt::DelayedMargTransformColumns(H_mapped, before.mResult.mOrder, Q, s);
  const Eigen::MatrixXd G_ref = H_mapped.transpose() * H_mapped;
  const Eigen::MatrixXd G_new =
      after.mResult.mH.transpose() * after.mResult.mH;
  EXPECT_LE((G_new - G_ref).norm(), 1e-7 * G_ref.norm());
  EXPECT_LE((after.mResult.mH.transpose() * after.mResult.mB -
             H_mapped.transpose() * before.mResult.mB)
                .norm(),
            1e-7 * (1.0 + (H_mapped.transpose() * before.mResult.mB).norm()));
  EXPECT_LE((after.mResult.mAnchor.mDir - Q * before.mResult.mAnchor.mDir)
                .norm(),
            1e-9);
}

// A coarse job over a random trajectory, or a vehicle at rest when moving is
// false, whose positions and velocities are shrunk by scale, whose velocities
// are perturbed by velError and whose positions carry a jitter of jitterStd,
// with inertial factors that carry the biases ba and bg and a keyframe every
// fourth frame, plans/delayed_marginalisation.md §2.3.11.
static basalt::DelayedMargJob<double> MakeCoarseTestJob(
    const basalt::Se3Spline<5>& spline, double scale, double velError,
    double jitterStd, const Eigen::Vector3d& ba, const Eigen::Vector3d& bg,
    bool moving = true) {
  constexpr int N = 41;
  constexpr int64_t kDt = 50000000;
  std::mt19937 gen(3);
  std::normal_distribution<double> noise(0.0, 1.0);

  basalt::DelayedMargJob<double> job;
  for (int i = 0; i < N; i += 4) job.mvKeyframes.insert(i * kDt);
  for (int i = 0; i < N; i++) {
    const int64_t t = i * kDt;
    Sophus::SE3d pose = moving ? spline.pose(t) : Sophus::SE3d();
    pose.translation() /= scale;
    pose.translation() +=
        jitterStd *
        Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
    const Eigen::Vector3d vel =
        (moving ? spline.transVelWorld(t) : Eigen::Vector3d::Zero().eval()) /
            scale +
        Eigen::Vector3d::Constant(velError);
    basalt::PoseVelBiasStateWithLin<double> st(
        t, pose, vel, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), true);
    if (i == N - 1) {
      job.mValues.mvWindowStates[t] = st;
    } else {
      job.mValues.mvRecords[t].mState = st;
    }
  }
  for (int i = 0; i + 1 < N; i++) {
    basalt::IntegratedImuMeasurement<double> meas(
        i * kDt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    for (int64_t t = i * kDt + 2500000; t < (i + 1) * kDt; t += 5000000) {
      basalt::ImuData<double> data;
      const Sophus::SE3d pose = moving ? spline.pose(t) : Sophus::SE3d();
      const Eigen::Vector3d accel =
          moving ? spline.transAccelWorld(t) : Eigen::Vector3d::Zero().eval();
      data.accel =
          pose.so3().inverse() * (accel - basalt::constants::g) + ba;
      data.gyro =
          (moving ? spline.rotVelBody(t) : Eigen::Vector3d::Zero().eval()) +
          bg;
      data.t_ns = t + 2500000;
      // The noise densities of data/sitl_calib.json at 200 Hz.
      meas.integrate(data, Eigen::Vector3d::Constant(1.28),
                     Eigen::Vector3d::Constant(1.1e-3));
    }
    auto step = std::make_shared<basalt::DelayedMargStep<double>>();
    job.mKfChain.Add(meas, job.mvKeyframes, 99);
    step->mvImu.push_back(meas);
    job.mvSteps.push_back(step);
  }

  job.mG = basalt::constants::g;
  job.mInitialDir = basalt::constants::g.normalized();
  job.mScaleVarMax = 1.0;
  job.mCoarse = true;
  return job;
}

// The coarse optimisation recovers the scale, the gravity direction, both
// biases and the velocities of exact data, plans/delayed_marginalisation.md
// §2.3.11.
TEST(VioTestSuite, DelayedMargCoarseScaleTest) {
  basalt::Se3Spline<5> spline(int64_t(1e9));
  spline.genRandomTrajectory(15);
  const double scale = 1.6;
  const Eigen::Vector3d ba(0.05, -0.03, 0.02);
  const Eigen::Vector3d bg(0.004, -0.003, 0.002);

  basalt::DelayedMargJob<double> job =
      MakeCoarseTestJob(spline, scale, 0.3, 0.0, ba, bg);
  job.Coarse();

  EXPECT_TRUE(job.mCoarseScaleFree);
  EXPECT_EQ(job.mCoarseSpans, 10);
  EXPECT_NEAR(job.mCoarseSigma, std::log(scale), 1e-3);
  EXPECT_LT(std::acos(std::min(1.0, job.mCoarseDir.dot(
                                        basalt::constants::g.normalized()))),
            1e-3);
  EXPECT_LT((job.mCoarseBa - ba).norm(), 1e-3);
  EXPECT_LT((job.mCoarseBg - bg).norm(), 1e-4);
  for (int i = 0; i < 41; i++) {
    const int64_t t = i * int64_t(50000000);
    EXPECT_LT((job.mValues.State(t).getState().vel_w_i -
               spline.transVelWorld(t) / scale)
                  .norm(),
              1e-2);
  }
}

// At rest the scale has no information, so the gate holds it, while the
// biases are still estimated.
TEST(VioTestSuite, DelayedMargCoarseStaticTest) {
  basalt::Se3Spline<5> spline(int64_t(1e9));
  spline.genRandomTrajectory(15);
  const Eigen::Vector3d ba(0.05, -0.03, 0.02);
  const Eigen::Vector3d bg(0.004, -0.003, 0.002);

  basalt::DelayedMargJob<double> job =
      MakeCoarseTestJob(spline, 1.6, 0.0, 0.02, ba, bg, false);
  job.Coarse();

  EXPECT_FALSE(job.mCoarseTimedOut);
  EXPECT_FALSE(job.mCoarseScaleFree);
  EXPECT_DOUBLE_EQ(job.mCoarseSigma, 0.0);
  EXPECT_LT((job.mCoarseBg - bg).norm(), 1e-4);
}

// A job that misses its deadline returns no correction.
TEST(VioTestSuite, DelayedMargCoarseDeadlineTest) {
  basalt::Se3Spline<5> spline(int64_t(1e9));
  spline.genRandomTrajectory(15);

  basalt::DelayedMargJob<double> job =
      MakeCoarseTestJob(spline, 1.6, 0.3, 0.0, Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero());
  job.mCoarseDeadlineSeconds = 1e-12;
  job.Coarse();

  EXPECT_TRUE(job.mCoarseTimedOut);
  EXPECT_TRUE(job.mvCoarseVelInc.empty());
  EXPECT_EQ(job.mCoarseIterations, 0);
}
#endif
