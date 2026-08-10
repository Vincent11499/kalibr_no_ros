#pragma once

#include "kalibr/ceres_optimizer/DynamicSpline.hpp"
#include "kalibr/ceres_optimizer/ImuResidual.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <vector>

namespace kalibr::ceres_optimizer {

Eigen::Matrix2d squareRootInformation2(
    const Eigen::Matrix2d& inverse_covariance);

struct PinholeCamera {
  std::array<double, 4> intrinsics = {1, 1, 0, 0};  // fu, fv, cu, cv
  std::array<double, 5> distortion = {};
  int distortion_count = 4;
};

template <typename T>
Eigen::Matrix<T, 2, 1> projectPinhole(
    const PinholeCamera& camera, const Eigen::Matrix<T, 3, 1>& point) {
  const T x = point.x() / point.z();
  const T y = point.y() / point.z();
  const T radius_squared = x * x + y * y;
  const T radius_fourth = radius_squared * radius_squared;
  T radial = T(1) + T(camera.distortion[0]) * radius_squared +
      T(camera.distortion[1]) * radius_fourth;
  if (camera.distortion_count == 5)
    radial += T(camera.distortion[4]) * radius_fourth * radius_squared;
  const T distorted_x = x * radial +
      T(2 * camera.distortion[2]) * x * y +
      T(camera.distortion[3]) * (radius_squared + T(2) * x * x);
  const T distorted_y = y * radial +
      T(camera.distortion[2]) * (radius_squared + T(2) * y * y) +
      T(2 * camera.distortion[3]) * x * y;
  return {T(camera.intrinsics[0]) * distorted_x + T(camera.intrinsics[2]),
          T(camera.intrinsics[1]) * distorted_y + T(camera.intrinsics[3])};
}

// Parameter order matches the bridge ABI:
// [fu, fv, cu, cv, k1, k2, p1, p2, k3].
template <typename T>
Eigen::Matrix<T, 2, 1> projectPinholeParameters(
    const T* parameters, int distortion_count,
    const Eigen::Matrix<T, 3, 1>& point) {
  const T x = point.x() / point.z();
  const T y = point.y() / point.z();
  const T radius_squared = x * x + y * y;
  const T radius_fourth = radius_squared * radius_squared;
  T radial = T(1) + parameters[4] * radius_squared +
      parameters[5] * radius_fourth;
  if (distortion_count == 5)
    radial += parameters[8] * radius_fourth * radius_squared;
  const T distorted_x = x * radial + T(2) * parameters[6] * x * y +
      parameters[7] * (radius_squared + T(2) * x * x);
  const T distorted_y = y * radial +
      parameters[6] * (radius_squared + T(2) * y * y) +
      T(2) * parameters[7] * x * y;
  return {parameters[0] * distorted_x + parameters[2],
          parameters[1] * distorted_y + parameters[3]};
}

struct CameraReprojectionResidual {
  const DynamicSpline* pose_spline = nullptr;
  int first_pose_control = 0;
  int pose_control_count = 0;
  double camera_timestamp = 0.0;
  double time_shift_prior = 0.0;
  Eigen::Vector3d target_point = Eigen::Vector3d::Zero();
  Eigen::Vector2d image_measurement = Eigen::Vector2d::Zero();
  Eigen::Matrix2d sqrt_information = Eigen::Matrix2d::Identity();
  PinholeCamera camera;
  Eigen::Matrix3d camera_from_camera0_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d camera_from_camera0_translation = Eigen::Vector3d::Zero();

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const {
    const int quaternion_block = pose_control_count;
    const int translation_block = quaternion_block + 1;
    const int time_offset_block = translation_block + 1;
    const T timestamp = T(camera_timestamp + time_shift_prior) +
        parameters[time_offset_block][0];
    const Eigen::Matrix<T, 6, 1> pose =
        pose_spline
            ->evaluateParameters(timestamp, 0, parameters,
                                 first_pose_control, 6)
            .template cast<T>();
    const Eigen::Matrix<T, 3, 3> world_from_imu =
        rotationVectorToMatrix(Eigen::Matrix<T, 3, 1>(
            pose.template tail<3>()));
    const Eigen::Matrix<T, 3, 1> point_imu =
        world_from_imu.transpose() *
        (target_point.cast<T>() - pose.template head<3>());
    const Eigen::Quaternion<T> camera0_from_imu_quaternion(
        parameters[quaternion_block][0], parameters[quaternion_block][1],
        parameters[quaternion_block][2], parameters[quaternion_block][3]);
    const Eigen::Matrix<T, 3, 1> camera0_from_imu_translation =
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(
            parameters[translation_block]);
    const Eigen::Matrix<T, 3, 1> point_camera0 =
        camera0_from_imu_quaternion.normalized() * point_imu +
        camera0_from_imu_translation;
    const Eigen::Matrix<T, 3, 1> point_camera =
        camera_from_camera0_rotation.cast<T>() * point_camera0 +
        camera_from_camera0_translation.cast<T>();
    const Eigen::Matrix<T, 2, 1> projection =
        projectPinhole(camera, point_camera);
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(residuals);
    residual = sqrt_information.cast<T>() *
        (projection - image_measurement.cast<T>());
    return true;
  }
};

// One residual block per target observation.  Every corner in an image shares
// the spline timestamp, extrinsic and time offset, so evaluating them together
// avoids repeating the expensive spline and rotation work for every corner.
struct CameraObservationResidual {
  const DynamicSpline* pose_spline = nullptr;
  int first_pose_control = 0;
  int pose_control_count = 0;
  double camera_timestamp = 0.0;
  double time_shift_prior = 0.0;
  std::vector<Eigen::Vector3d> target_points;
  std::vector<Eigen::Vector2d> image_measurements;
  Eigen::Matrix2d sqrt_information = Eigen::Matrix2d::Identity();
  PinholeCamera camera;
  Eigen::Matrix3d camera_from_camera0_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d camera_from_camera0_translation = Eigen::Vector3d::Zero();

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const {
    const int quaternion_block = pose_control_count;
    const int translation_block = quaternion_block + 1;
    const int time_offset_block = translation_block + 1;
    const T timestamp = T(camera_timestamp + time_shift_prior) +
        parameters[time_offset_block][0];
    const Eigen::Matrix<T, 6, 1> pose =
        pose_spline
            ->evaluateParameters(timestamp, 0, parameters,
                                 first_pose_control, 6)
            .template cast<T>();
    const Eigen::Matrix<T, 3, 3> world_from_imu =
        rotationVectorToMatrix(Eigen::Matrix<T, 3, 1>(
            pose.template tail<3>()));
    const Eigen::Quaternion<T> camera0_from_imu_quaternion(
        parameters[quaternion_block][0], parameters[quaternion_block][1],
        parameters[quaternion_block][2], parameters[quaternion_block][3]);
    const Eigen::Matrix<T, 3, 1> camera0_from_imu_translation =
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(
            parameters[translation_block]);
    for (std::size_t index = 0; index < target_points.size(); ++index) {
      const Eigen::Matrix<T, 3, 1> point_imu =
          world_from_imu.transpose() *
          (target_points[index].cast<T>() - pose.template head<3>());
      const Eigen::Matrix<T, 3, 1> point_camera0 =
          camera0_from_imu_quaternion.normalized() * point_imu +
          camera0_from_imu_translation;
      const Eigen::Matrix<T, 3, 1> point_camera =
          camera_from_camera0_rotation.cast<T>() * point_camera0 +
          camera_from_camera0_translation.cast<T>();
      const Eigen::Matrix<T, 2, 1> projection =
          projectPinhole(camera, point_camera);
      Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(residuals + 2 * index);
      residual = sqrt_information.cast<T>() *
          (projection - image_measurements[index].cast<T>());
    }
    return true;
  }
};

}  // namespace kalibr::ceres_optimizer
