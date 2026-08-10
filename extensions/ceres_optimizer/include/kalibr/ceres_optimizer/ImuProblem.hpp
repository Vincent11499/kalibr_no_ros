#pragma once

#include "kalibr/ceres_optimizer/Backend.hpp"
#include "kalibr/ceres_optimizer/CameraResidual.hpp"
#include "kalibr/ceres_optimizer/DynamicSpline.hpp"
#include "kalibr/ceres_optimizer/ImuResidual.hpp"

#include <bsplines/BSpline.hpp>
#include <bsplines/BSplinePose.hpp>

#include <ceres/ceres.h>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <vector>

namespace kalibr::ceres_optimizer {

struct ImuObservation {
  double timestamp = 0.0;
  Eigen::Vector3d gyroscope = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerometer = Eigen::Vector3d::Zero();
  Eigen::Matrix3d gyroscope_inverse_covariance = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d accelerometer_inverse_covariance =
      Eigen::Matrix3d::Identity();
};

struct ImuIntrinsics {
  ImuModel model = ImuModel::kCalibrated;
  std::array<double, 6> accelerometer_matrix = {1, 0, 1, 0, 0, 1};
  // Quaternion order is w, x, y, z, matching ceres::QuaternionManifold.
  std::array<double, 4> gyro_from_imu_quaternion = {1, 0, 0, 0};
  std::array<double, 6> gyroscope_matrix = {1, 0, 1, 0, 0, 1};
  std::array<double, 9> acceleration_sensitivity = {};
  std::array<double, 3> lever_x = {};
  std::array<double, 3> lever_y = {};
  std::array<double, 3> lever_z = {};
};

struct ImuProblemOptions {
  bool optimize_pose = true;
  bool optimize_biases = true;
  bool optimize_gravity_direction = true;
  bool optimize_intrinsics = true;
  bool add_bias_motion_error = true;
  double gyroscope_noise_scale = 1.0;
  double accelerometer_noise_scale = 1.0;
  double gyroscope_huber_width = -1.0;
  double accelerometer_huber_width = -1.0;
  double gyroscope_random_walk = 1.0;
  double accelerometer_random_walk = 1.0;
};

struct CameraChainEntry {
  PinholeCamera camera;
  Eigen::Matrix3d camera_from_camera0_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d camera_from_camera0_translation = Eigen::Vector3d::Zero();
  double time_shift_prior = 0.0;
};

struct CameraCornerObservation {
  int source_index = -1;
  int camera_index = 0;
  double timestamp = 0.0;
  Eigen::Vector3d target_point = Eigen::Vector3d::Zero();
  Eigen::Vector2d image_measurement = Eigen::Vector2d::Zero();
  Eigen::Matrix2d inverse_covariance = Eigen::Matrix2d::Identity();
};

struct JointResidualReport {
  Eigen::MatrixXd gyroscope_errors;
  Eigen::MatrixXd gyroscope_normalized_errors;
  Eigen::MatrixXd gyroscope_measurements;
  Eigen::MatrixXd accelerometer_errors;
  Eigen::MatrixXd accelerometer_normalized_errors;
  Eigen::MatrixXd accelerometer_measurements;
  // Columns are: source corner row, camera index, x error, y error.
  Eigen::MatrixXd camera_errors;
  Eigen::MatrixXd camera_normalized_errors;
};

struct CameraProblemOptions {
  bool optimize_camera0_from_imu = true;
  bool estimate_time_offsets = true;
  double time_offset_padding = 0.02;
  // Kalibr's positive Blake-Zisserman setting is rejected until the matching
  // Ceres loss is connected. The CLI default is disabled (-1).
  double blake_zisserman_degrees_of_freedom = -1.0;
};

// Owns an immutable Ceres graph and all parameter memory for the IMU portion
// of Kalibr's joint problem. Native splines are read once during construction;
// residual evaluation never calls their mutable expression caches.
class ImuProblem {
 public:
  ImuProblem(const bsplines::BSplinePose& pose_spline,
             const bsplines::BSpline& gyroscope_bias_spline,
             const bsplines::BSpline& accelerometer_bias_spline,
             std::vector<ImuObservation> observations,
             const Eigen::Vector3d& gravity,
             const ImuIntrinsics& intrinsics = {},
             const ImuProblemOptions& options = {});

  double evaluateCost() const;
  JointResidualReport residualReport() const;
  Summary solve(const Options& options = {});

  int addCameraResiduals(
      const std::vector<CameraChainEntry>& cameras,
      const std::vector<CameraCornerObservation>& observations,
      const Eigen::Matrix3d& camera0_from_imu_rotation,
      const Eigen::Vector3d& camera0_from_imu_translation,
      const CameraProblemOptions& options = {});

  int observationCount() const { return observation_count_; }
  int imuResidualBlockCount() const { return imu_residual_block_count_; }
  int skippedObservationCount() const { return skipped_observation_count_; }
  ceres::Problem* problem() { return problem_.get(); }
  const ceres::Problem* problem() const { return problem_.get(); }

  const std::vector<std::array<double, 6>>& poseControls() const {
    return pose_controls_;
  }
  const std::vector<std::array<double, 3>>& gyroscopeBiasControls() const {
    return gyroscope_bias_controls_;
  }
  const std::vector<std::array<double, 3>>& accelerometerBiasControls() const {
    return accelerometer_bias_controls_;
  }
  Eigen::Vector3d gravity() const { return Eigen::Vector3d(gravity_.data()); }
  const ImuIntrinsics& intrinsics() const { return intrinsics_; }
  Eigen::Matrix3d camera0FromImuRotation() const;
  Eigen::Vector3d camera0FromImuTranslation() const {
    return Eigen::Vector3d(camera0_from_imu_translation_.data());
  }
  std::vector<double> cameraTimeOffsets() const;

  void copyStateTo(bsplines::BSplinePose* pose_spline,
                   bsplines::BSpline* gyroscope_bias_spline,
                   bsplines::BSpline* accelerometer_bias_spline) const;

 private:
  struct ImuResidualRecord {
    ceres::ResidualBlockId block = nullptr;
    Eigen::Matrix3d sqrt_information = Eigen::Matrix3d::Identity();
    Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
    int residual_offset = 0;
    int residual_count = 3;
  };
  struct CameraResidualRecord {
    ceres::ResidualBlockId block = nullptr;
    Eigen::Matrix2d sqrt_information = Eigen::Matrix2d::Identity();
    std::vector<int> source_indices;
    int camera_index = 0;
  };

  std::unique_ptr<ceres::Problem> problem_;
  std::unique_ptr<DynamicSpline> dynamic_pose_spline_;
  std::vector<std::array<double, 6>> pose_controls_;
  std::vector<std::array<double, 3>> gyroscope_bias_controls_;
  std::vector<std::array<double, 3>> accelerometer_bias_controls_;
  std::array<double, 3> gravity_{};
  ImuIntrinsics intrinsics_;
  std::array<double, 4> camera0_from_imu_quaternion_ = {1, 0, 0, 0};
  std::array<double, 3> camera0_from_imu_translation_ = {};
  std::vector<std::array<double, 1>> camera_time_offset_deltas_;
  std::vector<double> camera_time_shift_priors_;
  std::vector<ImuResidualRecord> gyroscope_residual_records_;
  std::vector<ImuResidualRecord> accelerometer_residual_records_;
  std::vector<CameraResidualRecord> camera_residual_records_;
  bool camera_residuals_added_ = false;
  int observation_count_ = 0;
  int imu_residual_block_count_ = 0;
  int skipped_observation_count_ = 0;
};

Eigen::Matrix3d squareRootInformation(
    const Eigen::Matrix3d& inverse_covariance);

}  // namespace kalibr::ceres_optimizer
