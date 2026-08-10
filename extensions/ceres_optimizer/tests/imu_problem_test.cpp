#include "kalibr/ceres_optimizer/ImuProblem.hpp"

#include <bsplines/BSpline.hpp>
#include <bsplines/BSplinePose.hpp>
#include <sm/kinematics/RotationVector.hpp>

#include <boost/make_shared.hpp>

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <vector>

int main() {
  constexpr int kOrder = 6;
  auto rotation = boost::make_shared<sm::kinematics::RotationVector>();
  bsplines::BSplinePose pose(kOrder, rotation);
  constexpr int kSegments = 12;
  const int knot_count = kSegments + 2 * kOrder - 1;
  const int control_count = knot_count - kOrder;
  Eigen::VectorXd knots(knot_count);
  for (int index = 0; index < knot_count; ++index)
    knots[index] = 0.01 * (index - kOrder + 1);
  Eigen::MatrixXd pose_controls(6, control_count);
  for (int column = 0; column < control_count; ++column) {
    pose_controls.col(column) <<
        0.02 * column, -0.015 * column, 0.002 * column * column,
        0.004 * column, -0.003 * column, 0.0005 * column * column;
  }
  pose.setKnotVectorAndCoefficients(knots, pose_controls);

  bsplines::BSpline gyroscope_bias(kOrder);
  bsplines::BSpline accelerometer_bias(kOrder);
  gyroscope_bias.initConstantSpline(pose.t_min(), pose.t_max(), kSegments,
                                    Eigen::Vector3d::Zero());
  accelerometer_bias.initConstantSpline(pose.t_min(), pose.t_max(), kSegments,
                                        Eigen::Vector3d::Zero());
  const Eigen::Vector3d gravity(0.1, -0.2, -9.80665);
  const Eigen::Vector3d true_gyroscope_bias(0.012, -0.018, 0.027);
  const Eigen::Vector3d true_accelerometer_bias(-0.08, 0.04, 0.025);
  const Eigen::Matrix3d gyroscope_inverse_covariance =
      (Eigen::Vector3d(2.0e4, 3.0e4, 4.0e4)).asDiagonal();
  const Eigen::Matrix3d accelerometer_inverse_covariance =
      (Eigen::Matrix3d() << 500.0, 20.0, 0.0,
                            20.0, 600.0, 10.0,
                             0.0, 10.0, 700.0).finished();

  std::vector<kalibr::ceres_optimizer::ImuObservation> observations;
  double native_initial_energy = 0.0;
  for (int index = 0; index < 120; ++index) {
    const double time = pose.t_min() +
        (pose.t_max() - pose.t_min()) * (index + 0.5) / 120.0;
    kalibr::ceres_optimizer::ImuObservation observation;
    observation.timestamp = time;
    observation.gyroscope =
        pose.angularVelocityBodyFrame(time) + true_gyroscope_bias;
    observation.accelerometer = pose.inverseOrientation(time) *
        (pose.linearAcceleration(time) - gravity) + true_accelerometer_bias;
    observation.gyroscope_inverse_covariance =
        gyroscope_inverse_covariance;
    observation.accelerometer_inverse_covariance =
        accelerometer_inverse_covariance;
    native_initial_energy +=
        observation.gyroscope.dot(gyroscope_inverse_covariance *
                                  observation.gyroscope);
    const Eigen::Vector3d initial_accelerometer_prediction =
        pose.inverseOrientation(time) *
        (pose.linearAcceleration(time) - gravity);
    const Eigen::Vector3d accelerometer_error =
        initial_accelerometer_prediction - observation.accelerometer;
    native_initial_energy += accelerometer_error.dot(
        accelerometer_inverse_covariance * accelerometer_error);
    // Gyroscope prediction is omega, not zero.
    const Eigen::Vector3d gyroscope_error =
        pose.angularVelocityBodyFrame(time) - observation.gyroscope;
    native_initial_energy -= observation.gyroscope.dot(
        gyroscope_inverse_covariance * observation.gyroscope);
    native_initial_energy += gyroscope_error.dot(
        gyroscope_inverse_covariance * gyroscope_error);
    observations.push_back(observation);
  }

  kalibr::ceres_optimizer::ImuProblemOptions problem_options;
  problem_options.optimize_pose = false;
  problem_options.optimize_gravity_direction = false;
  problem_options.optimize_intrinsics = false;
  problem_options.add_bias_motion_error = true;
  problem_options.gyroscope_random_walk = 1.0e-2;
  problem_options.accelerometer_random_walk = 1.0e-1;
  kalibr::ceres_optimizer::ImuProblem problem(
      pose, gyroscope_bias, accelerometer_bias, observations, gravity, {},
      problem_options);
  const int motion_blocks = gyroscope_bias.numValidTimeSegments() +
      accelerometer_bias.numValidTimeSegments();
  const int imu_blocks =
      problem.problem()->NumResidualBlocks() - motion_blocks;
  if (imu_blocks <= 0 ||
      imu_blocks >= static_cast<int>(observations.size()) ||
      problem.imuResidualBlockCount() != imu_blocks) {
    std::cerr << "Temporal IMU batching did not reduce residual blocks: "
              << imu_blocks << " for " << observations.size()
              << " samples\n";
    return 5;
  }

  auto robust_options = problem_options;
  robust_options.add_bias_motion_error = false;
  robust_options.gyroscope_huber_width = 1.0;
  robust_options.accelerometer_huber_width = 1.0;
  kalibr::ceres_optimizer::ImuProblem robust_problem(
      pose, gyroscope_bias, accelerometer_bias, observations, gravity, {},
      robust_options);
  if (robust_problem.problem()->NumResidualBlocks() !=
      2 * static_cast<int>(observations.size())) {
    std::cerr << "Robust IMU components must retain independent blocks\n";
    return 7;
  }

  const double ceres_initial_energy = 2.0 * problem.evaluateCost();
  const double energy_error = std::abs(ceres_initial_energy -
                                       native_initial_energy) /
      std::max(1.0, std::abs(native_initial_energy));
  if (!std::isfinite(energy_error) || energy_error > 1.0e-11) {
    std::cerr << "Native/Ceres IMU initial energy mismatch: native="
              << native_initial_energy << " ceres=" << ceres_initial_energy
              << " relative=" << energy_error << '\n';
    return 1;
  }
  const auto initial_report = problem.residualReport();
  if (initial_report.gyroscope_errors.rows() !=
          static_cast<int>(observations.size()) ||
      initial_report.accelerometer_errors.rows() !=
          static_cast<int>(observations.size()) ||
      (initial_report.gyroscope_errors.row(0).transpose() +
       true_gyroscope_bias).norm() > 1.0e-12 ||
      (initial_report.accelerometer_errors.row(0).transpose() +
       true_accelerometer_bias).norm() > 1.0e-12) {
    std::cerr << "IMU batched residual report is inconsistent\n";
    return 6;
  }

  kalibr::ceres_optimizer::Options solver_options;
  solver_options.max_iterations = 20;
  solver_options.num_threads = 4;
  const auto summary = problem.solve(solver_options);
  if (!summary.usable || summary.final_cost >= summary.initial_cost) {
    std::cerr << summary.full_report << '\n';
    return 2;
  }
  for (const auto& control : problem.gyroscopeBiasControls()) {
    const Eigen::Vector3d estimate(control.data());
    if ((estimate - true_gyroscope_bias).norm() > 1.0e-7) {
      std::cerr << "Gyroscope bias solve mismatch: " << estimate.transpose()
                << '\n';
      return 3;
    }
  }
  for (const auto& control : problem.accelerometerBiasControls()) {
    const Eigen::Vector3d estimate(control.data());
    if ((estimate - true_accelerometer_bias).norm() > 1.0e-7) {
      std::cerr << "Accelerometer bias solve mismatch: "
                << estimate.transpose() << '\n';
      return 4;
    }
  }
  return 0;
}
