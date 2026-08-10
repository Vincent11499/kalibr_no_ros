#include "kalibr/ceres_optimizer/CameraBundleProblem.hpp"

#include <ceres/dynamic_autodiff_cost_function.h>
#include <ceres/manifold.h>

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace kalibr::ceres_optimizer {
namespace {

struct CameraBundleObservationResidual {
  int baseline_count = 0;
  int distortion_count = 4;
  std::vector<Eigen::Vector3d> target_points;
  std::vector<Eigen::Vector2d> image_measurements;
  Eigen::Matrix2d sqrt_information = Eigen::Matrix2d::Identity();

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const {
    const Eigen::Quaternion<T> camera0_from_target_rotation(
        parameters[0][0], parameters[0][1], parameters[0][2],
        parameters[0][3]);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>>
        camera0_from_target_translation(parameters[1]);
    const int camera_parameter_block = 2 + 2 * baseline_count;
    for (std::size_t index = 0; index < target_points.size(); ++index) {
      Eigen::Matrix<T, 3, 1> point =
          camera0_from_target_rotation.normalized() *
              target_points[index].cast<T>() +
          camera0_from_target_translation;
      for (int baseline = 0; baseline < baseline_count; ++baseline) {
        const Eigen::Quaternion<T> rotation(
            parameters[2 + 2 * baseline][0],
            parameters[2 + 2 * baseline][1],
            parameters[2 + 2 * baseline][2],
            parameters[2 + 2 * baseline][3]);
        const Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(
            parameters[3 + 2 * baseline]);
        point = rotation.normalized() * point + translation;
      }
      const Eigen::Matrix<T, 2, 1> projection = projectPinholeParameters(
          parameters[camera_parameter_block], distortion_count, point);
      Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(residuals + 2 * index);
      residual = sqrt_information.cast<T>() *
          (projection - image_measurements[index].cast<T>());
    }
    return true;
  }
};

CameraBundleProblem::TransformParameters transformParameters(
    const Eigen::Matrix4d& transform) {
  if (!transform.allFinite() ||
      !transform.bottomRows<1>().isApprox(
          Eigen::RowVector4d(0, 0, 0, 1), 1.0e-12)) {
    throw std::invalid_argument("Invalid homogeneous camera transform");
  }
  const Eigen::Quaterniond quaternion(transform.topLeftCorner<3, 3>());
  CameraBundleProblem::TransformParameters result;
  result.quaternion = {quaternion.w(), quaternion.x(), quaternion.y(),
                       quaternion.z()};
  result.translation = {transform(0, 3), transform(1, 3), transform(2, 3)};
  return result;
}

Eigen::Matrix4d transformMatrix(
    const CameraBundleProblem::TransformParameters& parameters) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.topLeftCorner<3, 3>() =
      Eigen::Quaterniond(parameters.quaternion[0], parameters.quaternion[1],
                         parameters.quaternion[2], parameters.quaternion[3])
          .normalized()
          .toRotationMatrix();
  result.topRightCorner<3, 1>() =
      Eigen::Vector3d(parameters.translation.data());
  return result;
}

}  // namespace

CameraBundleProblem::CameraBundleProblem(
    const std::vector<PinholeCamera>& cameras,
    const std::vector<Eigen::Matrix4d>& camera_from_previous,
    const std::vector<Eigen::Matrix4d>& camera0_from_target,
    const std::vector<CameraBundleObservation>& observations)
    : problem_(std::make_unique<ceres::Problem>()) {
  if (cameras.empty() || camera0_from_target.empty() ||
      camera_from_previous.size() + 1 != cameras.size()) {
    throw std::invalid_argument("Invalid camera bundle dimensions");
  }
  camera_parameters_.resize(cameras.size());
  distortion_counts_.resize(cameras.size());
  for (std::size_t index = 0; index < cameras.size(); ++index) {
    if ((cameras[index].distortion_count != 4 &&
         cameras[index].distortion_count != 5) ||
        !std::isfinite(cameras[index].intrinsics[0]) ||
        !std::isfinite(cameras[index].intrinsics[1])) {
      throw std::invalid_argument("Camera bundle supports radtan4/radtan5 only");
    }
    auto& parameters = camera_parameters_[index];
    for (int parameter = 0; parameter < 4; ++parameter)
      parameters[parameter] = cameras[index].intrinsics[parameter];
    for (int parameter = 0; parameter < 5; ++parameter)
      parameters[4 + parameter] = cameras[index].distortion[parameter];
    distortion_counts_[index] = cameras[index].distortion_count;
  }
  baselines_.reserve(camera_from_previous.size());
  for (const Eigen::Matrix4d& transform : camera_from_previous)
    baselines_.push_back(transformParameters(transform));
  views_.reserve(camera0_from_target.size());
  for (const Eigen::Matrix4d& transform : camera0_from_target)
    views_.push_back(transformParameters(transform));

  std::size_t begin = 0;
  while (begin < observations.size()) {
    const CameraBundleObservation& observation = observations[begin];
    if (observation.view_index < 0 ||
        observation.view_index >= static_cast<int>(views_.size()) ||
        observation.camera_index < 0 ||
        observation.camera_index >= static_cast<int>(cameras.size()) ||
        !observation.target_point.allFinite() ||
        !observation.image_measurement.allFinite()) {
      throw std::invalid_argument("Invalid camera bundle observation");
    }
    std::size_t end = begin + 1;
    while (end < observations.size() &&
           observations[end].view_index == observation.view_index &&
           observations[end].camera_index == observation.camera_index &&
           observations[end].inverse_covariance.isApprox(
               observation.inverse_covariance, 1.0e-12)) {
      if (!observations[end].target_point.allFinite() ||
          !observations[end].image_measurement.allFinite()) {
        throw std::invalid_argument("Invalid camera bundle observation");
      }
      ++end;
    }

    auto* residual = new CameraBundleObservationResidual();
    residual->baseline_count = observation.camera_index;
    residual->distortion_count =
        distortion_counts_[observation.camera_index];
    residual->sqrt_information =
        squareRootInformation2(observation.inverse_covariance);
    residual->target_points.reserve(end - begin);
    residual->image_measurements.reserve(end - begin);
    std::vector<int> source_indices;
    source_indices.reserve(end - begin);
    for (std::size_t index = begin; index < end; ++index) {
      residual->target_points.push_back(observations[index].target_point);
      residual->image_measurements.push_back(
          observations[index].image_measurement);
      source_indices.push_back(observations[index].source_index);
    }

    auto* cost = new ceres::DynamicAutoDiffCostFunction<
        CameraBundleObservationResidual, 16>(residual);
    std::vector<double*> parameters;
    TransformParameters& view = views_[observation.view_index];
    cost->AddParameterBlock(4);
    cost->AddParameterBlock(3);
    parameters.push_back(view.quaternion.data());
    parameters.push_back(view.translation.data());
    for (int baseline = 0; baseline < observation.camera_index; ++baseline) {
      cost->AddParameterBlock(4);
      cost->AddParameterBlock(3);
      parameters.push_back(baselines_[baseline].quaternion.data());
      parameters.push_back(baselines_[baseline].translation.data());
    }
    cost->AddParameterBlock(9);
    parameters.push_back(camera_parameters_[observation.camera_index].data());
    cost->SetNumResiduals(2 * static_cast<int>(end - begin));
    const ceres::ResidualBlockId block =
        problem_->AddResidualBlock(cost, nullptr, parameters);
    residual_records_.push_back(
        {block, residual->sqrt_information, std::move(source_indices),
         observation.view_index, observation.camera_index});
    observation_count_ += static_cast<int>(end - begin);
    begin = end;
  }
  if (observation_count_ == 0)
    throw std::invalid_argument("Camera bundle contains no observations");

  for (TransformParameters& view : views_)
    if (problem_->HasParameterBlock(view.quaternion.data()))
      problem_->SetManifold(view.quaternion.data(),
                            new ceres::QuaternionManifold());
  for (TransformParameters& baseline : baselines_)
    if (problem_->HasParameterBlock(baseline.quaternion.data()))
      problem_->SetManifold(baseline.quaternion.data(),
                            new ceres::QuaternionManifold());
  for (std::size_t index = 0; index < camera_parameters_.size(); ++index)
    if (distortion_counts_[index] == 4 &&
        problem_->HasParameterBlock(camera_parameters_[index].data()))
      problem_->SetManifold(camera_parameters_[index].data(),
                            new ceres::SubsetManifold(9, {8}));
}

double CameraBundleProblem::evaluateCost() const {
  double cost = 0.0;
  if (!problem_->Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr,
                          nullptr, nullptr))
    throw std::runtime_error("Ceres failed to evaluate camera bundle");
  return cost;
}

CameraBundleReport CameraBundleProblem::residualReport() const {
  CameraBundleReport report;
  report.errors.resize(observation_count_, 5);
  report.normalized_errors.resize(observation_count_, 5);
  int output_row = 0;
  for (const ResidualRecord& record : residual_records_) {
    std::vector<double> weighted_data(2 * record.source_indices.size(), 0.0);
    double cost = 0.0;
    if (!problem_->EvaluateResidualBlock(record.block, false, &cost,
                                         weighted_data.data(), nullptr)) {
      throw std::runtime_error("Ceres failed to evaluate camera residual");
    }
    for (std::size_t corner = 0; corner < record.source_indices.size();
         ++corner, ++output_row) {
      const Eigen::Vector2d weighted(weighted_data.data() + 2 * corner);
      const Eigen::Vector2d raw =
          record.sqrt_information.fullPivLu().solve(weighted);
      report.errors.row(output_row) << record.source_indices[corner],
          record.view_index, record.camera_index, raw.x(), raw.y();
      report.normalized_errors.row(output_row)
          << record.source_indices[corner], record.view_index,
          record.camera_index, weighted.x(), weighted.y();
    }
  }
  return report;
}

Summary CameraBundleProblem::solve(const Options& options) {
  Options bundle_options = options;
  bundle_options.structure = ProblemStructure::kBundleAdjustment;
  return kalibr::ceres_optimizer::solve(problem_.get(), bundle_options);
}

std::vector<PinholeCamera> CameraBundleProblem::cameras() const {
  std::vector<PinholeCamera> result(camera_parameters_.size());
  for (std::size_t index = 0; index < result.size(); ++index) {
    for (int parameter = 0; parameter < 4; ++parameter)
      result[index].intrinsics[parameter] =
          camera_parameters_[index][parameter];
    for (int parameter = 0; parameter < 5; ++parameter)
      result[index].distortion[parameter] =
          camera_parameters_[index][4 + parameter];
    result[index].distortion_count = distortion_counts_[index];
  }
  return result;
}

std::vector<Eigen::Matrix4d> CameraBundleProblem::cameraFromPrevious() const {
  std::vector<Eigen::Matrix4d> result;
  result.reserve(baselines_.size());
  for (const TransformParameters& baseline : baselines_)
    result.push_back(transformMatrix(baseline));
  return result;
}

std::vector<Eigen::Matrix4d> CameraBundleProblem::camera0FromTarget() const {
  std::vector<Eigen::Matrix4d> result;
  result.reserve(views_.size());
  for (const TransformParameters& view : views_)
    result.push_back(transformMatrix(view));
  return result;
}

}  // namespace kalibr::ceres_optimizer
