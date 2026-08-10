#include "kalibr/ceres_optimizer/ImuProblem.hpp"

#include "kalibr/ceres_optimizer/SplineMotionCost.hpp"

#include <ceres/dynamic_autodiff_cost_function.h>
#include <ceres/manifold.h>
#include <ceres/sphere_manifold.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace kalibr::ceres_optimizer {
namespace {

template <std::size_t Dimension>
std::vector<std::array<double, Dimension>> copyControls(
    const Eigen::MatrixXd& coefficients) {
  if (coefficients.rows() != static_cast<int>(Dimension)) {
    throw std::invalid_argument("Spline control dimension is incompatible");
  }
  std::vector<std::array<double, Dimension>> result(coefficients.cols());
  for (int column = 0; column < coefficients.cols(); ++column)
    for (std::size_t row = 0; row < Dimension; ++row)
      result[column][row] = coefficients(static_cast<int>(row), column);
  return result;
}

template <std::size_t Dimension>
Eigen::MatrixXd controlsToMatrix(
    const std::vector<std::array<double, Dimension>>& controls) {
  Eigen::MatrixXd result(Dimension, controls.size());
  for (std::size_t column = 0; column < controls.size(); ++column)
    for (std::size_t row = 0; row < Dimension; ++row)
      result(static_cast<int>(row), static_cast<int>(column)) =
          controls[column][row];
  return result;
}

template <typename Spline>
std::array<double, ImuResidual::kSplineOrder> basisWeights(
    const Spline& spline, double timestamp, int derivative_order,
    int dimension) {
  const Eigen::MatrixXd basis =
      spline.localBasisMatrix(timestamp, derivative_order);
  if (basis.rows() != dimension ||
      basis.cols() != ImuResidual::kSplineOrder * dimension) {
    throw std::runtime_error("Unexpected Kalibr B-spline basis dimensions");
  }
  std::array<double, ImuResidual::kSplineOrder> result{};
  for (int control = 0; control < ImuResidual::kSplineOrder; ++control) {
    result[control] = basis(0, control * dimension);
    for (int row = 1; row < dimension; ++row) {
      if (std::abs(basis(row, control * dimension + row) - result[control]) >
              1.0e-12 ||
          basis.row(row).segment(control * dimension, dimension)
                  .cwiseAbs().sum() -
                  std::abs(basis(row, control * dimension + row)) >
              1.0e-12) {
        throw std::runtime_error(
            "Kalibr B-spline basis is not component-separable");
      }
    }
  }
  return result;
}

ceres::LossFunction* makeHuberLoss(double width) {
  return width > 0.0 ? static_cast<ceres::LossFunction*>(
                           new ceres::HuberLoss(width))
                     : nullptr;
}

}  // namespace

Eigen::Matrix3d squareRootInformation(
    const Eigen::Matrix3d& inverse_covariance) {
  if (!inverse_covariance.allFinite() ||
      !inverse_covariance.isApprox(inverse_covariance.transpose(), 1.0e-12)) {
    throw std::invalid_argument(
        "IMU inverse covariance must be finite and symmetric");
  }
  Eigen::LLT<Eigen::Matrix3d> cholesky(inverse_covariance);
  if (cholesky.info() == Eigen::Success)
    return cholesky.matrixL().transpose();

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(
      inverse_covariance);
  if (eigen_solver.info() != Eigen::Success ||
      eigen_solver.eigenvalues().minCoeff() < -1.0e-12) {
    throw std::invalid_argument(
        "IMU inverse covariance must be positive semidefinite");
  }
  return eigen_solver.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
      eigen_solver.eigenvectors().transpose();
}

Eigen::Matrix2d squareRootInformation2(
    const Eigen::Matrix2d& inverse_covariance) {
  if (!inverse_covariance.allFinite() ||
      !inverse_covariance.isApprox(inverse_covariance.transpose(), 1.0e-12))
    throw std::invalid_argument(
        "Camera inverse covariance must be finite and symmetric");
  Eigen::LLT<Eigen::Matrix2d> cholesky(inverse_covariance);
  if (cholesky.info() != Eigen::Success)
    throw std::invalid_argument(
        "Camera inverse covariance must be positive definite");
  return cholesky.matrixL().transpose();
}

ImuProblem::ImuProblem(
    const bsplines::BSplinePose& pose_spline,
    const bsplines::BSpline& gyroscope_bias_spline,
    const bsplines::BSpline& accelerometer_bias_spline,
    std::vector<ImuObservation> observations, const Eigen::Vector3d& gravity,
    const ImuIntrinsics& intrinsics, const ImuProblemOptions& options)
    : problem_(std::make_unique<ceres::Problem>()),
      dynamic_pose_spline_(std::make_unique<DynamicSpline>(pose_spline)),
      pose_controls_(copyControls<6>(pose_spline.coefficients())),
      gyroscope_bias_controls_(
          copyControls<3>(gyroscope_bias_spline.coefficients())),
      accelerometer_bias_controls_(
          copyControls<3>(accelerometer_bias_spline.coefficients())),
      gravity_{gravity.x(), gravity.y(), gravity.z()},
      intrinsics_(intrinsics) {
  if (pose_spline.splineOrder() != ImuResidual::kSplineOrder ||
      gyroscope_bias_spline.splineOrder() != ImuResidual::kSplineOrder ||
      accelerometer_bias_spline.splineOrder() != ImuResidual::kSplineOrder) {
    throw std::invalid_argument("Kalibr Ceres IMU path currently requires order-6 splines");
  }
  if (!gravity.allFinite() || gravity.norm() <= 0.0 ||
      options.gyroscope_noise_scale <= 0.0 ||
      options.accelerometer_noise_scale <= 0.0 ||
      options.gyroscope_random_walk <= 0.0 ||
      options.accelerometer_random_walk <= 0.0) {
    throw std::invalid_argument("Invalid Ceres IMU problem options");
  }

  const double minimum_time = std::max(
      {pose_spline.t_min(), gyroscope_bias_spline.t_min(),
       accelerometer_bias_spline.t_min()});
  const double maximum_time = std::min(
      {pose_spline.t_max(), gyroscope_bias_spline.t_max(),
       accelerometer_bias_spline.t_max()});

  const auto make_residual = [&](const ImuObservation& observation,
                                 ImuResidualComponent component) {
    ImuResidual residual;
    residual.model = intrinsics_.model;
    residual.component = component;
    residual.pose_weights =
        basisWeights(pose_spline, observation.timestamp, 0, 6);
    residual.pose_derivative_weights =
        basisWeights(pose_spline, observation.timestamp, 1, 6);
    residual.pose_second_derivative_weights =
        basisWeights(pose_spline, observation.timestamp, 2, 6);
    residual.gyroscope_bias_weights = basisWeights(
        gyroscope_bias_spline, observation.timestamp, 0, 3);
    residual.accelerometer_bias_weights = basisWeights(
        accelerometer_bias_spline, observation.timestamp, 0, 3);
    residual.gyroscope_measurement = observation.gyroscope;
    residual.accelerometer_measurement = observation.accelerometer;
    residual.gyroscope_sqrt_information = squareRootInformation(
        observation.gyroscope_inverse_covariance /
        options.gyroscope_noise_scale);
    residual.accelerometer_sqrt_information = squareRootInformation(
        observation.accelerometer_inverse_covariance /
        options.accelerometer_noise_scale);
    return residual;
  };

  const auto append_record = [&](std::vector<ImuResidualRecord>* records,
                                 ceres::ResidualBlockId block,
                                 const Eigen::Matrix3d& sqrt_information,
                                 const Eigen::Vector3d& measurement,
                                 int residual_offset, int residual_count) {
    ImuResidualRecord record;
    record.block = block;
    record.sqrt_information = sqrt_information;
    record.measurement = measurement;
    record.residual_offset = residual_offset;
    record.residual_count = residual_count;
    records->push_back(record);
  };

  const auto add_parameter_blocks = [&](auto* cost,
                                        const Eigen::VectorXi& pose_indices,
                                        const Eigen::VectorXi& gyro_indices,
                                        const Eigen::VectorXi& accel_indices,
                                        std::vector<double*>* parameters) {
    for (int control = 0; control < ImuResidual::kSplineOrder; ++control) {
      cost->AddParameterBlock(6);
      parameters->push_back(pose_controls_[pose_indices[control]].data());
    }
    for (int control = 0; control < ImuResidual::kSplineOrder; ++control) {
      cost->AddParameterBlock(3);
      parameters->push_back(
          gyroscope_bias_controls_[gyro_indices[control]].data());
    }
    for (int control = 0; control < ImuResidual::kSplineOrder; ++control) {
      cost->AddParameterBlock(3);
      parameters->push_back(
          accelerometer_bias_controls_[accel_indices[control]].data());
    }
    cost->AddParameterBlock(3);
    parameters->push_back(gravity_.data());
    if (intrinsics_.model != ImuModel::kCalibrated) {
      cost->AddParameterBlock(6);
      cost->AddParameterBlock(4);
      cost->AddParameterBlock(6);
      cost->AddParameterBlock(9);
      parameters->push_back(intrinsics_.accelerometer_matrix.data());
      parameters->push_back(intrinsics_.gyro_from_imu_quaternion.data());
      parameters->push_back(intrinsics_.gyroscope_matrix.data());
      parameters->push_back(intrinsics_.acceleration_sensitivity.data());
      if (intrinsics_.model == ImuModel::kScaleMisalignmentSizeEffect) {
        cost->AddParameterBlock(3);
        cost->AddParameterBlock(3);
        cost->AddParameterBlock(3);
        parameters->push_back(intrinsics_.lever_x.data());
        parameters->push_back(intrinsics_.lever_y.data());
        parameters->push_back(intrinsics_.lever_z.data());
      }
    }
  };

  struct PendingBatch {
    Eigen::VectorXi pose_indices;
    Eigen::VectorXi gyroscope_indices;
    Eigen::VectorXi accelerometer_indices;
    std::vector<ImuResidual> samples;
  } pending_batch;
  const auto same_indices = [](const Eigen::VectorXi& left,
                               const Eigen::VectorXi& right) {
    return left.size() == right.size() &&
        (left.array() == right.array()).all();
  };
  const auto flush_batch = [&]() {
    if (pending_batch.samples.empty()) return;
    auto* residual = new ImuBatchResidual();
    residual->samples = std::move(pending_batch.samples);
    auto* cost = new ceres::DynamicAutoDiffCostFunction<
        ImuBatchResidual, 16>(residual);
    std::vector<double*> parameters;
    add_parameter_blocks(cost, pending_batch.pose_indices,
                         pending_batch.gyroscope_indices,
                         pending_batch.accelerometer_indices, &parameters);
    const int residual_count =
        6 * static_cast<int>(residual->samples.size());
    cost->SetNumResiduals(residual_count);
    const ceres::ResidualBlockId block =
        problem_->AddResidualBlock(cost, nullptr, parameters);
    ++imu_residual_block_count_;
    for (std::size_t sample = 0; sample < residual->samples.size(); ++sample) {
      const ImuResidual& item = residual->samples[sample];
      append_record(&gyroscope_residual_records_, block,
                    item.gyroscope_sqrt_information,
                    item.gyroscope_measurement, 6 * sample, residual_count);
      append_record(&accelerometer_residual_records_, block,
                    item.accelerometer_sqrt_information,
                    item.accelerometer_measurement, 6 * sample + 3,
                    residual_count);
    }
    pending_batch = PendingBatch();
  };

  for (const ImuObservation& observation : observations) {
    if (!(observation.timestamp > minimum_time &&
          observation.timestamp < maximum_time)) {
      ++skipped_observation_count_;
      continue;
    }
    if (!observation.gyroscope.allFinite() ||
        !observation.accelerometer.allFinite()) {
      throw std::invalid_argument("IMU observation contains non-finite values");
    }

    const Eigen::VectorXi pose_indices =
        pose_spline.localVvCoefficientVectorIndices(observation.timestamp);
    const Eigen::VectorXi gyroscope_indices =
        gyroscope_bias_spline.localVvCoefficientVectorIndices(
            observation.timestamp);
    const Eigen::VectorXi accelerometer_indices =
        accelerometer_bias_spline.localVvCoefficientVectorIndices(
            observation.timestamp);
    if (pose_indices.size() != ImuResidual::kSplineOrder ||
        gyroscope_indices.size() != ImuResidual::kSplineOrder ||
        accelerometer_indices.size() != ImuResidual::kSplineOrder) {
      throw std::runtime_error("Unexpected number of local spline controls");
    }

    auto add_component = [&](ImuResidualComponent component,
                             ceres::LossFunction* loss) {
      auto* residual = new ImuResidual(make_residual(observation, component));

      auto* cost =
          new ceres::DynamicAutoDiffCostFunction<ImuResidual, 16>(residual);
      std::vector<double*> parameters;
      add_parameter_blocks(cost, pose_indices, gyroscope_indices,
                           accelerometer_indices, &parameters);
      const int residual_count =
          component == ImuResidualComponent::kBoth ? 6 : 3;
      cost->SetNumResiduals(residual_count);
      const ceres::ResidualBlockId block =
          problem_->AddResidualBlock(cost, loss, parameters);
      ++imu_residual_block_count_;
      if (component == ImuResidualComponent::kGyroscope) {
        append_record(&gyroscope_residual_records_, block,
                      residual->gyroscope_sqrt_information,
                      observation.gyroscope, 0, residual_count);
      } else if (component == ImuResidualComponent::kAccelerometer) {
        append_record(&accelerometer_residual_records_, block,
                      residual->accelerometer_sqrt_information,
                      observation.accelerometer, 0, residual_count);
      } else {
        append_record(&gyroscope_residual_records_, block,
                      residual->gyroscope_sqrt_information,
                      observation.gyroscope, 0, residual_count);
        append_record(&accelerometer_residual_records_, block,
                      residual->accelerometer_sqrt_information,
                      observation.accelerometer, 3, residual_count);
      }
    };
    // With no robust loss, gyro and accelerometer terms have exactly the same
    // least-squares objective when represented as one six-dimensional block.
    // Keeping them separate when either Huber loss is active preserves
    // Kalibr's independent robustification of the two sensors.
    if (options.gyroscope_huber_width <= 0.0 &&
        options.accelerometer_huber_width <= 0.0) {
      if (!pending_batch.samples.empty() &&
          (!same_indices(pending_batch.pose_indices, pose_indices) ||
           !same_indices(pending_batch.gyroscope_indices,
                         gyroscope_indices) ||
           !same_indices(pending_batch.accelerometer_indices,
                         accelerometer_indices))) {
        flush_batch();
      }
      if (pending_batch.samples.empty()) {
        pending_batch.pose_indices = pose_indices;
        pending_batch.gyroscope_indices = gyroscope_indices;
        pending_batch.accelerometer_indices = accelerometer_indices;
      }
      pending_batch.samples.push_back(
          make_residual(observation, ImuResidualComponent::kBoth));
    } else {
      flush_batch();
      add_component(ImuResidualComponent::kGyroscope,
                    makeHuberLoss(options.gyroscope_huber_width));
      add_component(ImuResidualComponent::kAccelerometer,
                    makeHuberLoss(options.accelerometer_huber_width));
    }
    ++observation_count_;
  }
  flush_batch();

  if (observation_count_ == 0)
    throw std::invalid_argument("No IMU observations lie inside all spline domains");

  const auto add_motion_terms = [&](const bsplines::BSpline& spline,
                                    auto& controls, double random_walk) {
    const Eigen::Matrix3d information = Eigen::Matrix3d::Identity() /
        (random_walk * random_walk);
    for (int segment = 0; segment < spline.numValidTimeSegments(); ++segment) {
      const Eigen::MatrixXd factor =
          spline.segmentIntegral(segment, information, 1);
      auto* cost = new SplineSegmentMotionCost(factor, 3);
      const Eigen::VectorXi indices =
          spline.segmentVvCoefficientVectorIndices(segment);
      std::vector<double*> parameters;
      for (int index = 0; index < indices.size(); ++index)
        parameters.push_back(controls[indices[index]].data());
      problem_->AddResidualBlock(cost, nullptr, parameters);
    }
  };
  if (options.add_bias_motion_error) {
    add_motion_terms(gyroscope_bias_spline, gyroscope_bias_controls_,
                     options.gyroscope_random_walk);
    add_motion_terms(accelerometer_bias_spline, accelerometer_bias_controls_,
                     options.accelerometer_random_walk);
  }

  problem_->SetManifold(gravity_.data(), new ceres::SphereManifold<3>());
  if (intrinsics_.model != ImuModel::kCalibrated) {
    problem_->SetManifold(intrinsics_.gyro_from_imu_quaternion.data(),
                          new ceres::QuaternionManifold());
    if (intrinsics_.model == ImuModel::kScaleMisalignmentSizeEffect)
      problem_->SetParameterBlockConstant(intrinsics_.lever_x.data());
  }

  if (!options.optimize_pose)
    for (auto& control : pose_controls_)
      if (problem_->HasParameterBlock(control.data()))
        problem_->SetParameterBlockConstant(control.data());
  if (!options.optimize_biases) {
    for (auto& control : gyroscope_bias_controls_)
      if (problem_->HasParameterBlock(control.data()))
        problem_->SetParameterBlockConstant(control.data());
    for (auto& control : accelerometer_bias_controls_)
      if (problem_->HasParameterBlock(control.data()))
        problem_->SetParameterBlockConstant(control.data());
  }
  if (!options.optimize_gravity_direction)
    problem_->SetParameterBlockConstant(gravity_.data());
  if (!options.optimize_intrinsics &&
      intrinsics_.model != ImuModel::kCalibrated) {
    problem_->SetParameterBlockConstant(intrinsics_.accelerometer_matrix.data());
    problem_->SetParameterBlockConstant(
        intrinsics_.gyro_from_imu_quaternion.data());
    problem_->SetParameterBlockConstant(intrinsics_.gyroscope_matrix.data());
    problem_->SetParameterBlockConstant(
        intrinsics_.acceleration_sensitivity.data());
    if (intrinsics_.model == ImuModel::kScaleMisalignmentSizeEffect) {
      problem_->SetParameterBlockConstant(intrinsics_.lever_y.data());
      problem_->SetParameterBlockConstant(intrinsics_.lever_z.data());
    }
  }
}

int ImuProblem::addCameraResiduals(
    const std::vector<CameraChainEntry>& cameras,
    const std::vector<CameraCornerObservation>& observations,
    const Eigen::Matrix3d& camera0_from_imu_rotation,
    const Eigen::Vector3d& camera0_from_imu_translation,
    const CameraProblemOptions& options) {
  if (camera_residuals_added_)
    throw std::logic_error("Camera residuals can only be added once");
  if (cameras.empty() || !camera0_from_imu_rotation.allFinite() ||
      !camera0_from_imu_translation.allFinite() ||
      options.time_offset_padding < 0.0 ||
      options.blake_zisserman_degrees_of_freedom > 0.0) {
    throw std::invalid_argument("Invalid Ceres camera problem options");
  }
  for (const auto& camera : cameras) {
    if ((camera.camera.distortion_count != 4 &&
         camera.camera.distortion_count != 5) ||
        !camera.camera_from_camera0_rotation.allFinite() ||
        !camera.camera_from_camera0_translation.allFinite() ||
        !std::isfinite(camera.time_shift_prior)) {
      throw std::invalid_argument("Invalid pinhole camera chain entry");
    }
  }

  const Eigen::Quaterniond initial_quaternion(camera0_from_imu_rotation);
  camera0_from_imu_quaternion_ = {
      initial_quaternion.w(), initial_quaternion.x(), initial_quaternion.y(),
      initial_quaternion.z()};
  camera0_from_imu_translation_ = {
      camera0_from_imu_translation.x(), camera0_from_imu_translation.y(),
      camera0_from_imu_translation.z()};
  camera_time_offset_deltas_.assign(cameras.size(), {0.0});
  camera_time_shift_priors_.resize(cameras.size());
  for (std::size_t index = 0; index < cameras.size(); ++index)
    camera_time_shift_priors_[index] = cameras[index].time_shift_prior;

  int added = 0;
  std::size_t begin = 0;
  while (begin < observations.size()) {
    const CameraCornerObservation& observation = observations[begin];
    if (observation.camera_index < 0 ||
        observation.camera_index >= static_cast<int>(cameras.size()))
      throw std::invalid_argument("Camera observation index is out of range");
    if (!observation.target_point.allFinite() ||
        !observation.image_measurement.allFinite())
      throw std::invalid_argument("Camera observation is not finite");
    std::size_t end = begin + 1;
    while (end < observations.size() &&
           observations[end].camera_index == observation.camera_index &&
           observations[end].timestamp == observation.timestamp &&
           observations[end].inverse_covariance.isApprox(
               observation.inverse_covariance, 1.0e-12)) {
      ++end;
    }
    for (std::size_t index = begin + 1; index < end; ++index) {
      if (!observations[index].target_point.allFinite() ||
          !observations[index].image_measurement.allFinite()) {
        throw std::invalid_argument("Camera observation is not finite");
      }
    }

    const CameraChainEntry& camera = cameras[observation.camera_index];
    const double center_time = observation.timestamp + camera.time_shift_prior;
    const double minimum_time = center_time - options.time_offset_padding;
    const double maximum_time = center_time + options.time_offset_padding;
    if (minimum_time <= dynamic_pose_spline_->minimumTime() ||
        maximum_time >= dynamic_pose_spline_->maximumTime()) {
      begin = end;
      continue;
    }
    const auto range = dynamic_pose_spline_->candidateControls(
        minimum_time, maximum_time);
    auto* residual = new CameraObservationResidual();
    residual->pose_spline = dynamic_pose_spline_.get();
    residual->first_pose_control = range.first;
    residual->pose_control_count = range.second - range.first + 1;
    residual->camera_timestamp = observation.timestamp;
    residual->time_shift_prior = camera.time_shift_prior;
    residual->sqrt_information =
        squareRootInformation2(observation.inverse_covariance);
    residual->camera = camera.camera;
    residual->camera_from_camera0_rotation =
        camera.camera_from_camera0_rotation;
    residual->camera_from_camera0_translation =
        camera.camera_from_camera0_translation;
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
        CameraObservationResidual, 16>(residual);
    std::vector<double*> parameters;
    for (int control = range.first; control <= range.second; ++control) {
      cost->AddParameterBlock(6);
      parameters.push_back(pose_controls_[control].data());
    }
    cost->AddParameterBlock(4);
    cost->AddParameterBlock(3);
    cost->AddParameterBlock(1);
    parameters.push_back(camera0_from_imu_quaternion_.data());
    parameters.push_back(camera0_from_imu_translation_.data());
    parameters.push_back(
        camera_time_offset_deltas_[observation.camera_index].data());
    cost->SetNumResiduals(2 * static_cast<int>(end - begin));
    const ceres::ResidualBlockId block =
        problem_->AddResidualBlock(cost, nullptr, parameters);
    camera_residual_records_.push_back(
        {block, residual->sqrt_information, std::move(source_indices),
         observation.camera_index});
    added += static_cast<int>(end - begin);
    begin = end;
  }
  if (added == 0)
    throw std::invalid_argument(
        "No camera observations lie inside the padded pose spline domain");

  problem_->SetManifold(camera0_from_imu_quaternion_.data(),
                        new ceres::QuaternionManifold());
  if (!options.optimize_camera0_from_imu) {
    problem_->SetParameterBlockConstant(camera0_from_imu_quaternion_.data());
    problem_->SetParameterBlockConstant(camera0_from_imu_translation_.data());
  }
  for (auto& offset : camera_time_offset_deltas_) {
    if (problem_->HasParameterBlock(offset.data())) {
      problem_->SetParameterLowerBound(offset.data(), 0,
                                       -options.time_offset_padding);
      problem_->SetParameterUpperBound(offset.data(), 0,
                                       options.time_offset_padding);
      if (!options.estimate_time_offsets)
        problem_->SetParameterBlockConstant(offset.data());
    }
  }
  camera_residuals_added_ = true;
  return added;
}

Eigen::Matrix3d ImuProblem::camera0FromImuRotation() const {
  return Eigen::Quaterniond(camera0_from_imu_quaternion_[0],
                            camera0_from_imu_quaternion_[1],
                            camera0_from_imu_quaternion_[2],
                            camera0_from_imu_quaternion_[3])
      .normalized()
      .toRotationMatrix();
}

std::vector<double> ImuProblem::cameraTimeOffsets() const {
  std::vector<double> result(camera_time_offset_deltas_.size());
  for (std::size_t index = 0; index < result.size(); ++index)
    result[index] = camera_time_shift_priors_[index] +
        camera_time_offset_deltas_[index][0];
  return result;
}

double ImuProblem::evaluateCost() const {
  double cost = 0.0;
  if (!problem_->Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr,
                          nullptr, nullptr)) {
    throw std::runtime_error("Ceres failed to evaluate the IMU problem");
  }
  return cost;
}

JointResidualReport ImuProblem::residualReport() const {
  JointResidualReport report;
  const auto evaluate_imu = [&](const std::vector<ImuResidualRecord>& records,
                                Eigen::MatrixXd* errors,
                                Eigen::MatrixXd* normalized,
                                Eigen::MatrixXd* measurements) {
    errors->resize(records.size(), 3);
    normalized->resize(records.size(), 3);
    measurements->resize(records.size(), 3);
    ceres::ResidualBlockId evaluated_block = nullptr;
    std::vector<double> weighted_data;
    for (std::size_t index = 0; index < records.size(); ++index) {
      if (records[index].block != evaluated_block) {
        weighted_data.assign(records[index].residual_count, 0.0);
        double cost = 0.0;
        if (!problem_->EvaluateResidualBlock(records[index].block, false,
                                             &cost, weighted_data.data(),
                                             nullptr)) {
          throw std::runtime_error("Ceres failed to evaluate an IMU residual");
        }
        evaluated_block = records[index].block;
      }
      const Eigen::Vector3d weighted(
          weighted_data.data() + records[index].residual_offset);
      normalized->row(index) = weighted.transpose();
      errors->row(index) =
          records[index].sqrt_information.fullPivLu().solve(weighted).transpose();
      measurements->row(index) = records[index].measurement.transpose();
    }
  };
  evaluate_imu(gyroscope_residual_records_, &report.gyroscope_errors,
               &report.gyroscope_normalized_errors,
               &report.gyroscope_measurements);
  evaluate_imu(accelerometer_residual_records_, &report.accelerometer_errors,
               &report.accelerometer_normalized_errors,
               &report.accelerometer_measurements);

  std::size_t camera_corner_count = 0;
  for (const CameraResidualRecord& record : camera_residual_records_)
    camera_corner_count += record.source_indices.size();
  report.camera_errors.resize(camera_corner_count, 4);
  report.camera_normalized_errors.resize(camera_corner_count, 4);
  std::size_t output_row = 0;
  for (const CameraResidualRecord& record : camera_residual_records_) {
    std::vector<double> weighted_data(2 * record.source_indices.size(), 0.0);
    double cost = 0.0;
    if (!problem_->EvaluateResidualBlock(record.block, false, &cost,
                                         weighted_data.data(), nullptr)) {
      throw std::runtime_error("Ceres failed to evaluate a camera residual");
    }
    for (std::size_t corner = 0; corner < record.source_indices.size();
         ++corner, ++output_row) {
      const Eigen::Vector2d weighted(weighted_data.data() + 2 * corner);
      const Eigen::Vector2d raw =
          record.sqrt_information.fullPivLu().solve(weighted);
      report.camera_errors.row(output_row) << record.source_indices[corner],
          record.camera_index, raw.x(), raw.y();
      report.camera_normalized_errors.row(output_row)
          << record.source_indices[corner], record.camera_index,
          weighted.x(), weighted.y();
    }
  }
  return report;
}

Summary ImuProblem::solve(const Options& options) {
  return kalibr::ceres_optimizer::solve(problem_.get(), options);
}

void ImuProblem::copyStateTo(
    bsplines::BSplinePose* pose_spline,
    bsplines::BSpline* gyroscope_bias_spline,
    bsplines::BSpline* accelerometer_bias_spline) const {
  if (pose_spline == nullptr || gyroscope_bias_spline == nullptr ||
      accelerometer_bias_spline == nullptr) {
    throw std::invalid_argument("Output splines must not be null");
  }
  pose_spline->setCoefficientMatrix(controlsToMatrix(pose_controls_));
  gyroscope_bias_spline->setCoefficientMatrix(
      controlsToMatrix(gyroscope_bias_controls_));
  accelerometer_bias_spline->setCoefficientMatrix(
      controlsToMatrix(accelerometer_bias_controls_));
}

}  // namespace kalibr::ceres_optimizer
