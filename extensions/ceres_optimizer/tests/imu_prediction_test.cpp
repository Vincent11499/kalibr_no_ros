#include "kalibr/ceres_optimizer/ImuResidual.hpp"

#include <bsplines/BSplinePose.hpp>
#include <sm/kinematics/RotationVector.hpp>

#include <ceres/dynamic_autodiff_cost_function.h>

#include <boost/make_shared.hpp>

#include <Eigen/Core>

#include <cmath>
#include <array>
#include <iostream>
#include <memory>
#include <vector>

namespace {

using kalibr::ceres_optimizer::ImuModel;

Eigen::Matrix3d lowerMatrix(const std::array<double, 6>& values) {
  return kalibr::ceres_optimizer::lowerTriangularImuMatrix(values.data());
}

bool evaluateAutoDiff(ImuModel model) {
  using Residual = kalibr::ceres_optimizer::ImuResidual;
  auto* functor = new Residual();
  functor->model = model;
  functor->pose_weights[0] = 1.0;
  functor->pose_derivative_weights[0] = 1.0;
  functor->pose_second_derivative_weights[0] = 1.0;
  functor->gyroscope_bias_weights[0] = 1.0;
  functor->accelerometer_bias_weights[0] = 1.0;
  ceres::DynamicAutoDiffCostFunction<Residual, 16> cost(functor);
  std::array<std::array<double, 6>, Residual::kSplineOrder> pose_blocks{};
  std::array<std::array<double, 3>, Residual::kSplineOrder> gyro_blocks{};
  std::array<std::array<double, 3>, Residual::kSplineOrder> accel_blocks{};
  pose_blocks[0] = {0.1, -0.2, 0.3, 0.01, -0.02, 0.03};
  std::array<double, 3> gravity = {0.0, 0.0, -9.80665};
  std::array<double, 6> M_accel = {1.01, 0.01, 0.99, -0.02, 0.03, 1.02};
  std::array<double, 4> q_gyro_i = {0.999825, 0.01, -0.015, 0.005};
  std::array<double, 6> M_gyro = {0.98, 0.02, 1.01, -0.01, 0.015, 1.03};
  std::array<double, 9> A = {
      0.001, -0.002, 0.003, 0.004, -0.005,
      0.006, -0.007, 0.008, 0.009};
  std::array<double, 3> rx = {0.01, -0.02, 0.03};
  std::array<double, 3> ry = {-0.015, 0.025, 0.005};
  std::array<double, 3> rz = {0.02, 0.01, -0.01};
  std::vector<const double*> parameters;
  for (auto& block : pose_blocks) {
    cost.AddParameterBlock(6);
    parameters.push_back(block.data());
  }
  for (auto& block : gyro_blocks) {
    cost.AddParameterBlock(3);
    parameters.push_back(block.data());
  }
  for (auto& block : accel_blocks) {
    cost.AddParameterBlock(3);
    parameters.push_back(block.data());
  }
  cost.AddParameterBlock(3);
  parameters.push_back(gravity.data());
  if (model != ImuModel::kCalibrated) {
    cost.AddParameterBlock(6);
    cost.AddParameterBlock(4);
    cost.AddParameterBlock(6);
    cost.AddParameterBlock(9);
    parameters.push_back(M_accel.data());
    parameters.push_back(q_gyro_i.data());
    parameters.push_back(M_gyro.data());
    parameters.push_back(A.data());
    if (model == ImuModel::kScaleMisalignmentSizeEffect) {
      cost.AddParameterBlock(3);
      cost.AddParameterBlock(3);
      cost.AddParameterBlock(3);
      parameters.push_back(rx.data());
      parameters.push_back(ry.data());
      parameters.push_back(rz.data());
    }
  }
  cost.SetNumResiduals(6);
  std::array<double, 6> residuals{};
  std::vector<double> jacobian_storage;
  std::vector<double*> jacobians(parameters.size(), nullptr);
  // Request one Jacobian block so DynamicAutoDiff must instantiate Jet.
  jacobian_storage.resize(6 * 6);
  jacobians[0] = jacobian_storage.data();
  if (!cost.Evaluate(parameters.data(), residuals.data(), jacobians.data()))
    return false;
  for (const double residual : residuals)
    if (!std::isfinite(residual)) return false;
  for (const double derivative : jacobian_storage)
    if (!std::isfinite(derivative)) return false;
  return true;
}

}  // namespace

int main() {
  constexpr int kOrder = 6;
  const auto rotation =
      boost::make_shared<sm::kinematics::RotationVector>();
  bsplines::BSplinePose spline(kOrder, rotation);
  const int segment_count = 8;
  const int knot_count = segment_count + 2 * kOrder - 1;
  const int control_count = knot_count - kOrder;
  Eigen::VectorXd knots(knot_count);
  for (int index = 0; index < knot_count; ++index) {
    knots[index] = 0.01 * (index - kOrder + 1);
  }
  Eigen::MatrixXd controls(6, control_count);
  for (int column = 0; column < control_count; ++column) {
    controls.col(column) <<
        0.03 * column, -0.01 * column, 0.005 * column * column,
        0.003 * column, -0.002 * column, 0.001 * column * column;
  }
  spline.setKnotVectorAndCoefficients(knots, controls);

  const Eigen::Vector3d gravity(0.1, -0.2, -9.80665);
  const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.03);
  const Eigen::Vector3d accel_bias(-0.1, 0.05, 0.02);
  const std::array<double, 6> M_accel_values =
      {1.01, 0.01, 0.99, -0.02, 0.03, 1.02};
  const std::array<double, 6> M_gyro_values =
      {0.98, 0.02, 1.01, -0.01, 0.015, 1.03};
  const Eigen::Matrix3d M_accel = lowerMatrix(M_accel_values);
  const Eigen::Matrix3d M_gyro = lowerMatrix(M_gyro_values);
  const Eigen::Matrix3d A =
      (Eigen::Matrix3d() << 0.001, -0.002, 0.003,
                            0.004, -0.005, 0.006,
                           -0.007, 0.008, 0.009).finished();
  const Eigen::Matrix3d C_gyro_i =
      Eigen::AngleAxisd(0.035, Eigen::Vector3d(0.2, -0.3, 0.1).normalized())
          .toRotationMatrix();
  const Eigen::Vector3d rx(0.01, -0.02, 0.03);
  const Eigen::Vector3d ry(-0.015, 0.025, 0.005);
  const Eigen::Vector3d rz(0.02, 0.01, -0.01);
  for (int sample = 0; sample < 20; ++sample) {
    const double time = spline.t_min() +
        (spline.t_max() - spline.t_min()) * (sample + 0.5) / 20.0;
    const Eigen::Matrix<double, 6, 1> pose = spline.evalD(time, 0);
    const Eigen::Matrix<double, 6, 1> pose_d = spline.evalD(time, 1);
    const Eigen::Matrix<double, 6, 1> pose_dd = spline.evalD(time, 2);
    const Eigen::Vector3d omega = spline.angularVelocityBodyFrame(time);
    const Eigen::Vector3d omega_dot =
        spline.angularAccelerationBodyFrame(time);
    const Eigen::Vector3d body_force = spline.inverseOrientation(time) *
        (spline.linearAcceleration(time) - gravity);
    for (const ImuModel model : {
             ImuModel::kCalibrated, ImuModel::kScaleMisalignment,
             ImuModel::kScaleMisalignmentSizeEffect}) {
      const auto prediction = kalibr::ceres_optimizer::predictImuMeasurement(
          pose, pose_d, pose_dd, gyro_bias, accel_bias, gravity, model,
          M_accel, C_gyro_i, M_gyro, A, rx, ry, rz);
      Eigen::Vector3d expected_gyro;
      Eigen::Vector3d expected_accel;
      if (model == ImuModel::kCalibrated) {
        expected_gyro = omega + gyro_bias;
        expected_accel = body_force + accel_bias;
      } else {
        expected_gyro = M_gyro * (C_gyro_i * omega) +
            A * (C_gyro_i * body_force) + gyro_bias;
        Eigen::Vector3d force = body_force;
        if (model == ImuModel::kScaleMisalignmentSizeEffect) {
          const auto effect = [&](const Eigen::Vector3d& lever) {
            return (omega_dot.cross(lever) +
                    omega.cross(omega.cross(lever))).eval();
          };
          force.x() += effect(rx).x();
          force.y() += effect(ry).y();
          force.z() += effect(rz).z();
        }
        expected_accel = M_accel * force + accel_bias;
      }
      const double error =
          (prediction.head<3>() - expected_gyro).norm() +
          (prediction.tail<3>() - expected_accel).norm();
      if (!std::isfinite(error) || error > 1.0e-10) {
        std::cerr << "IMU prediction mismatch at t=" << time
                  << " model=" << static_cast<int>(model)
                  << ": " << error << '\n';
        return 1;
      }
    }
  }

  for (const ImuModel model : {
           ImuModel::kCalibrated, ImuModel::kScaleMisalignment,
           ImuModel::kScaleMisalignmentSizeEffect}) {
    if (!evaluateAutoDiff(model)) {
      std::cerr << "Ceres AutoDiff IMU residual failed for model "
                << static_cast<int>(model) << '\n';
      return 3;
    }
  }
  return 0;
}
