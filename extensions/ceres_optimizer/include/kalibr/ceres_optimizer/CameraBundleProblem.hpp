#pragma once

#include "kalibr/ceres_optimizer/Backend.hpp"
#include "kalibr/ceres_optimizer/CameraResidual.hpp"

#include <ceres/ceres.h>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <vector>

namespace kalibr::ceres_optimizer {

struct CameraBundleObservation {
  int source_index = -1;
  int view_index = 0;
  int camera_index = 0;
  Eigen::Vector3d target_point = Eigen::Vector3d::Zero();
  Eigen::Vector2d image_measurement = Eigen::Vector2d::Zero();
  Eigen::Matrix2d inverse_covariance = Eigen::Matrix2d::Identity();
};

struct CameraBundleReport {
  // Columns: source row, view index, camera index, x error, y error.
  Eigen::MatrixXd errors;
  Eigen::MatrixXd normalized_errors;
};

// Thread-safe bundle adjustment for fixed target points, per-view target
// poses, pinhole-radtan camera intrinsics and adjacent camera-chain baselines.
// The input/output baseline convention is T_c(n)_c(n-1). View transforms use
// T_c0_target internally and are returned in that same convention.
class CameraBundleProblem {
 public:
  CameraBundleProblem(
      const std::vector<PinholeCamera>& cameras,
      const std::vector<Eigen::Matrix4d>& camera_from_previous,
      const std::vector<Eigen::Matrix4d>& camera0_from_target,
      const std::vector<CameraBundleObservation>& observations);

  double evaluateCost() const;
  CameraBundleReport residualReport() const;
  Summary solve(const Options& options = {});

  int observationCount() const { return observation_count_; }
  int residualBlockCount() const {
    return static_cast<int>(residual_records_.size());
  }
  std::vector<PinholeCamera> cameras() const;
  std::vector<Eigen::Matrix4d> cameraFromPrevious() const;
  std::vector<Eigen::Matrix4d> camera0FromTarget() const;
  ceres::Problem* problem() { return problem_.get(); }
  const ceres::Problem* problem() const { return problem_.get(); }

  // Public only so translation-unit helpers can convert matrix conventions;
  // callers should use the matrix getters above.
  struct TransformParameters {
    std::array<double, 4> quaternion = {1, 0, 0, 0};
    std::array<double, 3> translation = {};
  };

 private:
  struct ResidualRecord {
    ceres::ResidualBlockId block = nullptr;
    Eigen::Matrix2d sqrt_information = Eigen::Matrix2d::Identity();
    std::vector<int> source_indices;
    int view_index = 0;
    int camera_index = 0;
  };

  std::unique_ptr<ceres::Problem> problem_;
  std::vector<std::array<double, 9>> camera_parameters_;
  std::vector<int> distortion_counts_;
  std::vector<TransformParameters> baselines_;
  std::vector<TransformParameters> views_;
  std::vector<ResidualRecord> residual_records_;
  int observation_count_ = 0;
};

}  // namespace kalibr::ceres_optimizer
