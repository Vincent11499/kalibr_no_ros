#include "kalibr/ceres_optimizer/CameraBundleProblem.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <vector>

namespace kco = kalibr::ceres_optimizer;

Eigen::Matrix4d transform(const Eigen::Matrix3d& rotation,
                          const Eigen::Vector3d& translation) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.topLeftCorner<3, 3>() = rotation;
  result.topRightCorner<3, 1>() = translation;
  return result;
}

int main() {
  std::vector<kco::PinholeCamera> truth(2);
  truth[0].intrinsics = {460.0, 455.0, 367.0, 248.0};
  truth[0].distortion = {-0.12, 0.035, 0.0005, -0.0004, 0.0};
  truth[1].intrinsics = {458.0, 454.0, 365.0, 247.0};
  truth[1].distortion = {-0.11, 0.03, -0.0003, 0.0002, 0.004};
  truth[1].distortion_count = 5;
  const Eigen::Matrix4d true_baseline = transform(
      Eigen::AngleAxisd(0.012, Eigen::Vector3d(0.2, 0.8, -0.1).normalized())
          .toRotationMatrix(),
      Eigen::Vector3d(-0.11, 0.001, -0.002));

  std::vector<Eigen::Matrix4d> true_views;
  std::vector<kco::CameraBundleObservation> observations;
  for (int view = 0; view < 12; ++view) {
    const Eigen::Vector3d axis(0.2 + 0.03 * view, -0.5 + 0.04 * view, 0.1);
    const Eigen::Matrix4d pose = transform(
        Eigen::AngleAxisd(-0.18 + 0.035 * view, axis.normalized())
            .toRotationMatrix(),
        Eigen::Vector3d(-0.15 + 0.025 * view,
                        0.08 * std::sin(0.4 * view), 2.7 + 0.06 * view));
    true_views.push_back(pose);
    for (int camera = 0; camera < 2; ++camera) {
      for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 8; ++column) {
          const Eigen::Vector3d target(0.055 * column, 0.055 * row, 0.0);
          Eigen::Vector3d point =
              pose.topLeftCorner<3, 3>() * target +
              pose.topRightCorner<3, 1>();
          if (camera == 1)
            point = true_baseline.topLeftCorner<3, 3>() * point +
                true_baseline.topRightCorner<3, 1>();
          kco::CameraBundleObservation observation;
          observation.source_index = observations.size();
          observation.view_index = view;
          observation.camera_index = camera;
          observation.target_point = target;
          observation.image_measurement =
              kco::projectPinhole(truth[camera], point);
          observations.push_back(observation);
        }
      }
    }
  }

  std::vector<kco::PinholeCamera> initial = truth;
  for (kco::PinholeCamera& camera : initial) {
    camera.intrinsics[0] *= 1.015;
    camera.intrinsics[1] *= 0.985;
    camera.intrinsics[2] += 2.0;
    camera.intrinsics[3] -= 1.5;
    camera.distortion[0] += 0.01;
  }
  std::vector<Eigen::Matrix4d> initial_views = true_views;
  for (Eigen::Matrix4d& view : initial_views) {
    view.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(0.008, Eigen::Vector3d::UnitY()).toRotationMatrix() *
        view.topLeftCorner<3, 3>();
    view.topRightCorner<3, 1>() += Eigen::Vector3d(0.01, -0.008, 0.015);
  }
  Eigen::Matrix4d initial_baseline = true_baseline;
  initial_baseline.topRightCorner<3, 1>() +=
      Eigen::Vector3d(0.012, -0.006, 0.004);

  kco::CameraBundleProblem problem(initial, {initial_baseline}, initial_views,
                                   observations);
  if (problem.observationCount() != static_cast<int>(observations.size()) ||
      problem.residualBlockCount() != 24 ||
      problem.residualReport().errors.rows() !=
          static_cast<int>(observations.size())) {
    std::cerr << "Camera bundle batching/report mismatch\n";
    return 1;
  }
  kco::Options options;
  options.max_iterations = 80;
  options.num_threads = 4;
  options.structure = kco::ProblemStructure::kBundleAdjustment;
  const auto summary = problem.solve(options);
  if (!summary.usable || summary.final_cost >= summary.initial_cost ||
      summary.final_cost > 1.0e-8) {
    std::cerr << summary.full_report << '\n';
    return 2;
  }
  const auto solved_cameras = problem.cameras();
  const auto solved_baselines = problem.cameraFromPrevious();
  if (std::abs(solved_cameras[0].intrinsics[0] -
               truth[0].intrinsics[0]) > 1.0e-4 ||
      (solved_baselines[0].topRightCorner<3, 1>() -
       true_baseline.topRightCorner<3, 1>()).norm() > 1.0e-6) {
    std::cerr << "Camera bundle state mismatch\n";
    return 3;
  }
  return 0;
}
