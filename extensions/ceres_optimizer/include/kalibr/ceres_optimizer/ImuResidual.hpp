#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <vector>

namespace kalibr::ceres_optimizer {

enum class ImuModel {
  kCalibrated,
  kScaleMisalignment,
  kScaleMisalignmentSizeEffect,
};

enum class ImuResidualComponent {
  kBoth,
  kGyroscope,
  kAccelerometer,
};

// Kalibr MatrixBasicDv activates the lower triangle with the mask
// [[1,0,0],[1,1,0],[1,1,1]]. Store only those six active coordinates in the
// same row order instead of carrying three constant upper-triangle entries.
template <typename T>
Eigen::Matrix<T, 3, 3> lowerTriangularImuMatrix(const T* parameters) {
  Eigen::Matrix<T, 3, 3> result = Eigen::Matrix<T, 3, 3>::Zero();
  result(0, 0) = parameters[0];
  result(1, 0) = parameters[1];
  result(1, 1) = parameters[2];
  result(2, 0) = parameters[3];
  result(2, 1) = parameters[4];
  result(2, 2) = parameters[5];
  return result;
}

template <typename T>
Eigen::Matrix<T, 3, 3> crossMatrix(
    const Eigen::Matrix<T, 3, 1>& vector) {
  Eigen::Matrix<T, 3, 3> result;
  result << T(0), -vector.z(), vector.y(), vector.z(), T(0), -vector.x(),
      -vector.y(), vector.x(), T(0);
  return result;
}

// This is Kalibr's sm::kinematics::RotationVector convention: Exp(-[r]x).
template <typename T>
Eigen::Matrix<T, 3, 3> rotationVectorToMatrix(
    const Eigen::Matrix<T, 3, 1>& parameters) {
  const T angle = parameters.norm();
  const T squared_angle = angle * angle;
  const Eigen::Matrix<T, 3, 3> cross = crossMatrix(parameters);
  if (squared_angle < T(1.0e-20)) {
    return Eigen::Matrix<T, 3, 3>::Identity() - cross +
           T(0.5) * cross * cross;
  }
  return Eigen::Matrix<T, 3, 3>::Identity() -
         (sin(angle) / angle) * cross +
         ((T(1) - cos(angle)) / squared_angle) * cross * cross;
}

template <typename T>
Eigen::Matrix<T, 3, 3> rotationVectorSMatrix(
    const Eigen::Matrix<T, 3, 1>& parameters) {
  const T angle = parameters.norm();
  if (angle * angle < T(1.0e-20)) {
    return Eigen::Matrix<T, 3, 3>::Identity();
  }
  const Eigen::Matrix<T, 3, 1> axis = parameters / angle;
  const Eigen::Matrix<T, 3, 3> cross = crossMatrix(axis);
  const T half_sine = sin(T(0.5) * angle);
  const T c1 = -T(2) * half_sine * half_sine / angle;
  const T c2 = (angle - sin(angle)) / angle;
  return Eigen::Matrix<T, 3, 3>::Identity() + c1 * cross +
         c2 * cross * cross;
}

template <typename T>
Eigen::Matrix<T, 3, 1> angularVelocityBody(
    const Eigen::Matrix<T, 3, 1>& rotation,
    const Eigen::Matrix<T, 3, 1>& rotation_derivative) {
  const Eigen::Matrix<T, 3, 3> world_from_body =
      rotationVectorToMatrix(rotation);
  return -world_from_body.transpose() * rotationVectorSMatrix(rotation) *
         rotation_derivative;
}

template <typename T>
Eigen::Matrix<T, 6, 1> predictImuMeasurement(
    const Eigen::Matrix<T, 6, 1>& pose,
    const Eigen::Matrix<T, 6, 1>& pose_derivative,
    const Eigen::Matrix<T, 6, 1>& pose_second_derivative,
    const Eigen::Matrix<T, 3, 1>& gyroscope_bias,
    const Eigen::Matrix<T, 3, 1>& accelerometer_bias,
    const Eigen::Matrix<T, 3, 1>& gravity, ImuModel model,
    const Eigen::Matrix<T, 3, 3>& accelerometer_matrix =
        Eigen::Matrix<T, 3, 3>::Identity(),
    const Eigen::Matrix<T, 3, 3>& gyro_from_imu =
        Eigen::Matrix<T, 3, 3>::Identity(),
    const Eigen::Matrix<T, 3, 3>& gyroscope_matrix =
        Eigen::Matrix<T, 3, 3>::Identity(),
    const Eigen::Matrix<T, 3, 3>& acceleration_sensitivity =
        Eigen::Matrix<T, 3, 3>::Zero(),
    const Eigen::Matrix<T, 3, 1>& lever_x =
        Eigen::Matrix<T, 3, 1>::Zero(),
    const Eigen::Matrix<T, 3, 1>& lever_y =
        Eigen::Matrix<T, 3, 1>::Zero(),
    const Eigen::Matrix<T, 3, 1>& lever_z =
        Eigen::Matrix<T, 3, 1>::Zero()) {
  const auto rotation = pose.template tail<3>();
  const auto rotation_derivative = pose_derivative.template tail<3>();
  const auto rotation_second_derivative =
      pose_second_derivative.template tail<3>();
  const Eigen::Matrix<T, 3, 3> world_from_imu =
      rotationVectorToMatrix(Eigen::Matrix<T, 3, 1>(rotation));
  const Eigen::Matrix<T, 3, 1> angular_velocity = angularVelocityBody(
      Eigen::Matrix<T, 3, 1>(rotation),
      Eigen::Matrix<T, 3, 1>(rotation_derivative));
  const Eigen::Matrix<T, 3, 1> body_origin_force =
      world_from_imu.transpose() *
      (pose_second_derivative.template head<3>() - gravity);
  Eigen::Matrix<T, 3, 1> accelerometer_force = body_origin_force;

  if (model == ImuModel::kScaleMisalignmentSizeEffect) {
    // This intentionally follows Kalibr's existing size-effect model, which
    // applies the RotationVector angular-velocity map to the second spline
    // derivative as its angular-acceleration expression.
    const Eigen::Matrix<T, 3, 1> angular_acceleration =
        angularVelocityBody(Eigen::Matrix<T, 3, 1>(rotation),
                            Eigen::Matrix<T, 3, 1>(
                                rotation_second_derivative));
    const auto centrifugal = [&](const Eigen::Matrix<T, 3, 1>& lever) {
      return (angular_acceleration.cross(lever) +
              angular_velocity.cross(angular_velocity.cross(lever)))
          .eval();
    };
    const Eigen::Matrix<T, 3, 1> effect_x = centrifugal(lever_x);
    const Eigen::Matrix<T, 3, 1> effect_y = centrifugal(lever_y);
    const Eigen::Matrix<T, 3, 1> effect_z = centrifugal(lever_z);
    accelerometer_force.x() += effect_x.x();
    accelerometer_force.y() += effect_y.y();
    accelerometer_force.z() += effect_z.z();
  }

  Eigen::Matrix<T, 6, 1> prediction;
  if (model == ImuModel::kCalibrated) {
    prediction.template head<3>() = angular_velocity + gyroscope_bias;
    prediction.template tail<3>() =
        accelerometer_force + accelerometer_bias;
  } else {
    prediction.template head<3>() =
        gyroscope_matrix * (gyro_from_imu * angular_velocity) +
        acceleration_sensitivity * (gyro_from_imu * body_origin_force) +
        gyroscope_bias;
    prediction.template tail<3>() =
        accelerometer_matrix * accelerometer_force + accelerometer_bias;
  }
  return prediction;
}

// A thread-safe residual for all three Kalibr IMU models. All observations and
// spline basis weights are immutable; Ceres owns independent parameter blocks.
// Parameter order is documented by the constants below and is intentionally
// shared by the problem builder and tests.
struct ImuResidual {
  static constexpr int kSplineOrder = 6;
  static constexpr int kGravityBlock = 3 * kSplineOrder;
  static constexpr int kAccelerometerMatrixBlock = kGravityBlock + 1;
  static constexpr int kGyroFromImuQuaternionBlock = kGravityBlock + 2;
  static constexpr int kGyroscopeMatrixBlock = kGravityBlock + 3;
  static constexpr int kAccelerationSensitivityBlock = kGravityBlock + 4;
  static constexpr int kLeverXBlock = kGravityBlock + 5;
  static constexpr int kLeverYBlock = kGravityBlock + 6;
  static constexpr int kLeverZBlock = kGravityBlock + 7;

  ImuModel model = ImuModel::kCalibrated;
  ImuResidualComponent component = ImuResidualComponent::kBoth;
  std::array<double, kSplineOrder> pose_weights{};
  std::array<double, kSplineOrder> pose_derivative_weights{};
  std::array<double, kSplineOrder> pose_second_derivative_weights{};
  std::array<double, kSplineOrder> gyroscope_bias_weights{};
  std::array<double, kSplineOrder> accelerometer_bias_weights{};
  Eigen::Vector3d gyroscope_measurement = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer_measurement = Eigen::Vector3d::Zero();
  // R satisfies R.transpose() * R == Kalibr's inverse covariance. Keeping the
  // full matrix preserves correlated measurements instead of assuming that
  // the YAML noise model is isotropic.
  Eigen::Matrix3d gyroscope_sqrt_information = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d accelerometer_sqrt_information =
      Eigen::Matrix3d::Identity();

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const {
    Eigen::Matrix<T, 6, 1> pose = Eigen::Matrix<T, 6, 1>::Zero();
    Eigen::Matrix<T, 6, 1> pose_d = Eigen::Matrix<T, 6, 1>::Zero();
    Eigen::Matrix<T, 6, 1> pose_dd = Eigen::Matrix<T, 6, 1>::Zero();
    Eigen::Matrix<T, 3, 1> gyro_bias = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> accel_bias = Eigen::Matrix<T, 3, 1>::Zero();
    for (int control = 0; control < kSplineOrder; ++control) {
      const Eigen::Map<const Eigen::Matrix<T, 6, 1>> pose_control(
          parameters[control]);
      pose += T(pose_weights[control]) * pose_control;
      pose_d += T(pose_derivative_weights[control]) * pose_control;
      pose_dd += T(pose_second_derivative_weights[control]) * pose_control;
      gyro_bias += T(gyroscope_bias_weights[control]) *
          Eigen::Map<const Eigen::Matrix<T, 3, 1>>(
              parameters[kSplineOrder + control]);
      accel_bias += T(accelerometer_bias_weights[control]) *
          Eigen::Map<const Eigen::Matrix<T, 3, 1>>(
              parameters[2 * kSplineOrder + control]);
    }
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> gravity(
        parameters[kGravityBlock]);
    const Eigen::Matrix<T, 3, 1> gravity_vector = gravity;
    Eigen::Matrix<T, 3, 3> accelerometer_matrix =
        Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 3> gyro_from_imu =
        Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 3> gyroscope_matrix =
        Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 3> acceleration_sensitivity =
        Eigen::Matrix<T, 3, 3>::Zero();
    Eigen::Matrix<T, 3, 1> lever_x = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> lever_y = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> lever_z = Eigen::Matrix<T, 3, 1>::Zero();
    if (model != ImuModel::kCalibrated) {
      accelerometer_matrix = lowerTriangularImuMatrix(
          parameters[kAccelerometerMatrixBlock]);
      const Eigen::Quaternion<T> gyro_from_imu_quaternion(
          parameters[kGyroFromImuQuaternionBlock][0],
          parameters[kGyroFromImuQuaternionBlock][1],
          parameters[kGyroFromImuQuaternionBlock][2],
          parameters[kGyroFromImuQuaternionBlock][3]);
      gyro_from_imu = gyro_from_imu_quaternion.normalized().toRotationMatrix();
      gyroscope_matrix = lowerTriangularImuMatrix(
          parameters[kGyroscopeMatrixBlock]);
      using RowMajorMatrix3 =
          Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;
      acceleration_sensitivity = Eigen::Map<const RowMajorMatrix3>(
          parameters[kAccelerationSensitivityBlock]);
      if (model == ImuModel::kScaleMisalignmentSizeEffect) {
        lever_x = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(
            parameters[kLeverXBlock]);
        lever_y = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(
            parameters[kLeverYBlock]);
        lever_z = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(
            parameters[kLeverZBlock]);
      }
    }
    const Eigen::Matrix<T, 6, 1> prediction = predictImuMeasurement(
        pose, pose_d, pose_dd, gyro_bias, accel_bias, gravity_vector,
        model, accelerometer_matrix, gyro_from_imu, gyroscope_matrix,
        acceleration_sensitivity, lever_x, lever_y, lever_z);
    const Eigen::Matrix<T, 3, 1> gyroscope_error =
        prediction.template head<3>() - gyroscope_measurement.cast<T>();
    const Eigen::Matrix<T, 3, 1> accelerometer_error =
        prediction.template tail<3>() - accelerometer_measurement.cast<T>();
    if (component == ImuResidualComponent::kBoth) {
      Eigen::Map<Eigen::Matrix<T, 3, 1>> gyroscope_residual(residuals);
      Eigen::Map<Eigen::Matrix<T, 3, 1>> accelerometer_residual(
          residuals + 3);
      gyroscope_residual =
          gyroscope_sqrt_information.cast<T>() * gyroscope_error;
      accelerometer_residual =
          accelerometer_sqrt_information.cast<T>() * accelerometer_error;
    } else if (component == ImuResidualComponent::kGyroscope) {
      Eigen::Map<Eigen::Matrix<T, 3, 1>> gyroscope_residual(residuals);
      gyroscope_residual =
          gyroscope_sqrt_information.cast<T>() * gyroscope_error;
    } else {
      Eigen::Map<Eigen::Matrix<T, 3, 1>> accelerometer_residual(residuals);
      accelerometer_residual =
          accelerometer_sqrt_information.cast<T>() * accelerometer_error;
    }
    return true;
  }
};

using CalibratedImuResidual = ImuResidual;

// Adjacent IMU samples often share the same order-6 pose and bias control
// blocks (for example a 200 Hz IMU with a 100 Hz pose spline). Combining those
// samples removes duplicate residual-block metadata while preserving the exact
// row-wise least-squares objective. Robustified components remain unbatched so
// each measurement keeps its independent loss function.
struct ImuBatchResidual {
  std::vector<ImuResidual> samples;

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const {
    for (std::size_t index = 0; index < samples.size(); ++index) {
      if (!samples[index](parameters, residuals + 6 * index)) return false;
    }
    return true;
  }
};

}  // namespace kalibr::ceres_optimizer
