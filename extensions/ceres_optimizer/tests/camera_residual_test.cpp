#include "kalibr/ceres_optimizer/CameraResidual.hpp"

#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/RadialTangentialDistortion.hpp>
#include <bsplines/BSplinePose.hpp>
#include <sm/kinematics/RotationVector.hpp>

#include <ceres/dynamic_autodiff_cost_function.h>

#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
  constexpr int kOrder = 6;
  const auto rotation = boost::make_shared<sm::kinematics::RotationVector>();
  bsplines::BSplinePose spline(kOrder, rotation);
  constexpr int kSegments = 10;
  const int knot_count = kSegments + 2 * kOrder - 1;
  const int control_count = knot_count - kOrder;
  Eigen::VectorXd knots(knot_count);
  for (int index = 0; index < knot_count; ++index)
    knots[index] = 0.02 * (index - kOrder + 1);
  Eigen::MatrixXd controls(6, control_count);
  for (int column = 0; column < control_count; ++column) {
    controls.col(column) <<
        0.01 * column, -0.005 * column, 0.003 * column * column,
        0.002 * column, -0.001 * column, 0.0003 * column * column;
  }
  spline.setKnotVectorAndCoefficients(knots, controls);
  kalibr::ceres_optimizer::DynamicSpline dynamic_spline(spline);

  // The copied basis matrices must reproduce Kalibr for value and first two
  // derivatives before time offset is allowed to become a Jet.
  std::vector<std::array<double, 6>> control_blocks(control_count);
  for (int column = 0; column < control_count; ++column)
    for (int row = 0; row < 6; ++row)
      control_blocks[column][row] = controls(row, column);
  std::vector<const double*> all_parameters;
  for (const auto& control : control_blocks)
    all_parameters.push_back(control.data());
  for (int sample = 0; sample < 50; ++sample) {
    const double time = spline.t_min() +
        (spline.t_max() - spline.t_min()) * (sample + 0.5) / 50.0;
    for (int derivative = 0; derivative <= 2; ++derivative) {
      const Eigen::VectorXd actual = dynamic_spline.evaluateParameters(
          time, derivative, all_parameters.data(), 0, 6);
      const Eigen::VectorXd expected = spline.evalD(time, derivative);
      if ((actual - expected).norm() > 1.0e-11) {
        std::cerr << "Dynamic spline mismatch at derivative " << derivative
                  << '\n';
        return 1;
      }
    }
  }

  kalibr::ceres_optimizer::PinholeCamera camera;
  camera.intrinsics = {460.0, 455.0, 367.0, 248.0};
  camera.distortion = {-0.12, 0.035, 0.0008, -0.0004, 0.0};
  camera.distortion_count = 4;
  const double timestamp = 0.5 * (spline.t_min() + spline.t_max());
  const double padding = 0.01;
  const auto range = dynamic_spline.candidateControls(timestamp - padding,
                                                       timestamp + padding);
  kalibr::ceres_optimizer::CameraReprojectionResidual residual;
  residual.pose_spline = &dynamic_spline;
  residual.first_pose_control = range.first;
  residual.pose_control_count = range.second - range.first + 1;
  residual.camera_timestamp = timestamp;
  residual.target_point = Eigen::Vector3d(0.4, -0.2, 2.5);
  residual.image_measurement = Eigen::Vector2d::Zero();
  residual.camera = camera;
  residual.camera_from_camera0_rotation =
      Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY()).toRotationMatrix();
  residual.camera_from_camera0_translation =
      Eigen::Vector3d(-0.11, 0.001, -0.002);

  auto* cost = new ceres::DynamicAutoDiffCostFunction<
      kalibr::ceres_optimizer::CameraReprojectionResidual, 16>(
      new kalibr::ceres_optimizer::CameraReprojectionResidual(residual));
  std::vector<const double*> parameters;
  for (int control = range.first; control <= range.second; ++control) {
    cost->AddParameterBlock(6);
    parameters.push_back(control_blocks[control].data());
  }
  std::array<double, 4> quaternion = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> translation = {0.03, -0.01, 0.02};
  std::array<double, 1> time_offset = {0.0};
  cost->AddParameterBlock(4);
  cost->AddParameterBlock(3);
  cost->AddParameterBlock(1);
  parameters.push_back(quaternion.data());
  parameters.push_back(translation.data());
  parameters.push_back(time_offset.data());
  cost->SetNumResiduals(2);
  std::array<double, 2> output{};
  std::array<double, 2> time_jacobian{};
  std::vector<double*> jacobians(parameters.size(), nullptr);
  jacobians.back() = time_jacobian.data();
  if (!cost->Evaluate(parameters.data(), output.data(), jacobians.data()))
    return 2;

  const Eigen::Matrix<double, 6, 1> pose = spline.evalD(timestamp, 0);
  const Eigen::Vector3d point_imu = spline.inverseOrientation(timestamp) *
      (residual.target_point - pose.head<3>());
  const Eigen::Vector3d point_camera0 = point_imu +
      Eigen::Vector3d(translation.data());
  const Eigen::Vector3d point_camera =
      residual.camera_from_camera0_rotation * point_camera0 +
      residual.camera_from_camera0_translation;
  const aslam::cameras::RadialTangentialDistortion distortion(
      camera.distortion[0], camera.distortion[1], camera.distortion[2],
      camera.distortion[3]);
  aslam::cameras::PinholeProjection<
      aslam::cameras::RadialTangentialDistortion>
      native_projection(camera.intrinsics[0], camera.intrinsics[1],
                        camera.intrinsics[2], camera.intrinsics[3], 752, 480,
                        distortion);
  Eigen::Vector2d expected;
  native_projection.euclideanToKeypoint(point_camera, expected);
  if ((Eigen::Vector2d(output.data()) - expected).norm() > 1.0e-10) {
    std::cerr << "Native/Ceres reprojection mismatch\n";
    return 3;
  }

  const double epsilon = 1.0e-7;
  time_offset[0] = epsilon;
  std::array<double, 2> plus{};
  cost->Evaluate(parameters.data(), plus.data(), nullptr);
  time_offset[0] = -epsilon;
  std::array<double, 2> minus{};
  cost->Evaluate(parameters.data(), minus.data(), nullptr);
  const Eigen::Vector2d numerical =
      (Eigen::Vector2d(plus.data()) - Eigen::Vector2d(minus.data())) /
      (2.0 * epsilon);
  if ((Eigen::Vector2d(time_jacobian.data()) - numerical).norm() > 1.0e-4) {
    std::cerr << "Ceres time-offset Jacobian mismatch\n";
    return 4;
  }
  delete cost;
  return 0;
}
