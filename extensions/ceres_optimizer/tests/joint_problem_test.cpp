#include "kalibr/ceres_optimizer/ImuProblem.hpp"

#include <bsplines/BSpline.hpp>
#include <bsplines/BSplinePose.hpp>
#include <sm/kinematics/RotationVector.hpp>

#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <vector>

int main() {
  constexpr int kOrder = 6;
  const auto rotation = boost::make_shared<sm::kinematics::RotationVector>();
  bsplines::BSplinePose pose(kOrder, rotation);
  constexpr int kSegments = 24;
  const int knot_count = kSegments + 2 * kOrder - 1;
  const int control_count = knot_count - kOrder;
  Eigen::VectorXd knots(knot_count);
  for (int index = 0; index < knot_count; ++index)
    knots[index] = 0.02 * (index - kOrder + 1);
  Eigen::MatrixXd controls(6, control_count);
  for (int column = 0; column < control_count; ++column) {
    const double x = static_cast<double>(column);
    controls.col(column) <<
        0.02 * x, 0.01 * std::sin(0.4 * x), 0.004 * x * x,
        0.025 * std::sin(0.25 * x), 0.02 * std::cos(0.3 * x),
        0.006 * x;
  }
  pose.setKnotVectorAndCoefficients(knots, controls);
  bsplines::BSpline gyroscope_bias(kOrder), accelerometer_bias(kOrder);
  gyroscope_bias.initConstantSpline(pose.t_min(), pose.t_max(), kSegments,
                                    Eigen::Vector3d::Zero());
  accelerometer_bias.initConstantSpline(pose.t_min(), pose.t_max(), kSegments,
                                        Eigen::Vector3d::Zero());
  const Eigen::Vector3d gravity(0.0, 0.0, -9.80665);

  std::vector<kalibr::ceres_optimizer::ImuObservation> imu;
  for (int sample = 0; sample < 200; ++sample) {
    const double time = pose.t_min() +
        (pose.t_max() - pose.t_min()) * (sample + 0.5) / 200.0;
    kalibr::ceres_optimizer::ImuObservation observation;
    observation.timestamp = time;
    observation.gyroscope = pose.angularVelocityBodyFrame(time);
    observation.accelerometer = pose.inverseOrientation(time) *
        (pose.linearAcceleration(time) - gravity);
    observation.gyroscope_inverse_covariance =
        Eigen::Matrix3d::Identity() * 1.0e3;
    observation.accelerometer_inverse_covariance =
        Eigen::Matrix3d::Identity() * 1.0e2;
    imu.push_back(observation);
  }

  kalibr::ceres_optimizer::ImuProblemOptions imu_options;
  imu_options.optimize_pose = false;
  imu_options.optimize_biases = false;
  imu_options.optimize_gravity_direction = false;
  imu_options.add_bias_motion_error = false;
  kalibr::ceres_optimizer::ImuProblem problem(
      pose, gyroscope_bias, accelerometer_bias, imu, gravity, {}, imu_options);

  kalibr::ceres_optimizer::CameraChainEntry camera;
  camera.camera.intrinsics = {460.0, 455.0, 367.0, 248.0};
  camera.camera.distortion = {-0.1, 0.03, 0.0005, -0.0004, 0.0};
  const Eigen::Matrix3d true_rotation =
      Eigen::AngleAxisd(0.08, Eigen::Vector3d(0.3, -0.2, 0.1).normalized())
          .toRotationMatrix();
  const Eigen::Vector3d true_translation(0.04, -0.025, 0.018);
  constexpr double kTrueTimeOffset = 0.0035;
  std::vector<kalibr::ceres_optimizer::CameraCornerObservation> corners;
  const std::vector<Eigen::Vector3d> targets = {
      {-0.4, -0.3, 2.8}, {0.4, -0.3, 2.8},
      {-0.4, 0.3, 2.8}, {0.4, 0.3, 2.8}, {0.0, 0.0, 3.2}};
  for (int frame = 0; frame < 36; ++frame) {
    const double timestamp = pose.t_min() + 0.03 +
        (pose.t_max() - pose.t_min() - 0.06) * frame / 35.0;
    const double imu_time = timestamp + kTrueTimeOffset;
    const Eigen::Matrix<double, 6, 1> state = pose.evalD(imu_time, 0);
    for (const Eigen::Vector3d& target : targets) {
      const Eigen::Vector3d point_imu = pose.inverseOrientation(imu_time) *
          (target - state.head<3>());
      const Eigen::Vector3d point_camera =
          true_rotation * point_imu + true_translation;
      kalibr::ceres_optimizer::CameraCornerObservation observation;
      observation.source_index = static_cast<int>(corners.size());
      observation.timestamp = timestamp;
      observation.target_point = target;
      observation.image_measurement =
          kalibr::ceres_optimizer::projectPinhole(camera.camera, point_camera);
      observation.inverse_covariance = Eigen::Matrix2d::Identity() * 4.0;
      corners.push_back(observation);
    }
  }
  kalibr::ceres_optimizer::CameraProblemOptions camera_options;
  camera_options.time_offset_padding = 0.01;
  const Eigen::Matrix3d initial_rotation =
      Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitX()) * true_rotation;
  const Eigen::Vector3d initial_translation =
      true_translation + Eigen::Vector3d(0.02, -0.015, 0.01);
  const int residual_blocks_before_camera = problem.problem()->NumResidualBlocks();
  const int added_corners = problem.addCameraResiduals(
      {camera}, corners, initial_rotation, initial_translation, camera_options);
  const int camera_residual_blocks =
      problem.problem()->NumResidualBlocks() - residual_blocks_before_camera;
  if (added_corners != static_cast<int>(corners.size()) ||
      camera_residual_blocks != 36) {
    std::cerr << "Camera observation batching mismatch: corners="
              << added_corners << " blocks=" << camera_residual_blocks
              << '\n';
    return 3;
  }
  const auto initial_report = problem.residualReport();
  if (initial_report.camera_errors.rows() !=
      static_cast<int>(corners.size()) ||
      initial_report.camera_errors(0, 0) != 0.0 ||
      initial_report.camera_errors(corners.size() - 1, 0) !=
          static_cast<double>(corners.size() - 1)) {
    std::cerr << "Camera batched residual report lost corner rows\n";
    return 4;
  }

  kalibr::ceres_optimizer::Options solver_options;
  solver_options.max_iterations = 40;
  solver_options.num_threads = 4;
  const auto summary = problem.solve(solver_options);
  if (!summary.usable || summary.final_cost >= summary.initial_cost) {
    std::cerr << summary.full_report << '\n';
    return 1;
  }
  const Eigen::Matrix3d rotation_error =
      problem.camera0FromImuRotation() * true_rotation.transpose();
  const double angle_error =
      Eigen::AngleAxisd(rotation_error).angle();
  const double translation_error =
      (problem.camera0FromImuTranslation() - true_translation).norm();
  const double time_error =
      std::abs(problem.cameraTimeOffsets().front() - kTrueTimeOffset);
  if (angle_error > 1.0e-6 || translation_error > 1.0e-6 ||
      time_error > 1.0e-7) {
    std::cerr << "Joint solve mismatch: rotation=" << angle_error
              << " translation=" << translation_error
              << " time=" << time_error << '\n';
    return 2;
  }
  return 0;
}
