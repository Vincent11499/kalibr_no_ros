#include <numpy_eigen/boost_python_headers.hpp>

#include "kalibr/ceres_optimizer/CameraBundleProblem.hpp"
#include "kalibr/ceres_optimizer/ImuProblem.hpp"

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>

#include <boost/python.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace bp = boost::python;
using kalibr::ceres_optimizer::CameraChainEntry;
using kalibr::ceres_optimizer::CameraBundleObservation;
using kalibr::ceres_optimizer::CameraBundleProblem;
using kalibr::ceres_optimizer::CameraBundleReport;
using kalibr::ceres_optimizer::CameraCornerObservation;
using kalibr::ceres_optimizer::CameraProblemOptions;
using kalibr::ceres_optimizer::ImuIntrinsics;
using kalibr::ceres_optimizer::ImuModel;
using kalibr::ceres_optimizer::ImuObservation;
using kalibr::ceres_optimizer::ImuProblem;
using kalibr::ceres_optimizer::ImuProblemOptions;
using kalibr::ceres_optimizer::JointResidualReport;

namespace {

template <typename T>
T option(const bp::dict& options, const char* name, const T& fallback) {
  return options.has_key(name) ? bp::extract<T>(options[name]) : fallback;
}

std::array<double, 6> lowerTriangle(const Eigen::Matrix3d& matrix) {
  return {matrix(0, 0), matrix(1, 0), matrix(1, 1),
          matrix(2, 0), matrix(2, 1), matrix(2, 2)};
}

Eigen::Matrix3d lowerTriangle(const std::array<double, 6>& values) {
  return kalibr::ceres_optimizer::lowerTriangularImuMatrix(values.data());
}

bp::dict reportToDict(const JointResidualReport& report) {
  bp::dict result;
  result["gyroscope_errors"] = report.gyroscope_errors;
  result["gyroscope_normalized_errors"] =
      report.gyroscope_normalized_errors;
  result["gyroscope_measurements"] = report.gyroscope_measurements;
  result["accelerometer_errors"] = report.accelerometer_errors;
  result["accelerometer_normalized_errors"] =
      report.accelerometer_normalized_errors;
  result["accelerometer_measurements"] = report.accelerometer_measurements;
  result["camera_errors"] = report.camera_errors;
  result["camera_normalized_errors"] = report.camera_normalized_errors;
  return result;
}

bp::dict reportToDict(const CameraBundleReport& report) {
  bp::dict result;
  result["errors"] = report.errors;
  result["normalized_errors"] = report.normalized_errors;
  return result;
}

bp::dict solveCameraBundle(
    const Eigen::MatrixXd& camera_matrix,
    const Eigen::MatrixXd& baseline_transforms,
    const Eigen::MatrixXd& view_transforms,
    const Eigen::MatrixXd& corner_matrix,
    const bp::dict& raw_options) {
  if (camera_matrix.rows() <= 0 || camera_matrix.cols() != 10 ||
      baseline_transforms.cols() != 4 ||
      baseline_transforms.rows() != 4 * (camera_matrix.rows() - 1) ||
      view_transforms.cols() != 4 || view_transforms.rows() <= 0 ||
      view_transforms.rows() % 4 != 0 || corner_matrix.cols() != 7) {
    throw std::invalid_argument("Invalid camera bundle bridge matrices");
  }

  std::vector<kalibr::ceres_optimizer::PinholeCamera> cameras(
      camera_matrix.rows());
  for (int camera_index = 0; camera_index < camera_matrix.rows();
       ++camera_index) {
    for (int parameter = 0; parameter < 4; ++parameter)
      cameras[camera_index].intrinsics[parameter] =
          camera_matrix(camera_index, parameter);
    for (int parameter = 0; parameter < 5; ++parameter)
      cameras[camera_index].distortion[parameter] =
          camera_matrix(camera_index, 4 + parameter);
    cameras[camera_index].distortion_count =
        static_cast<int>(camera_matrix(camera_index, 9));
  }
  std::vector<Eigen::Matrix4d> baselines;
  baselines.reserve(static_cast<std::size_t>(camera_matrix.rows() - 1));
  for (int index = 0; index + 1 < camera_matrix.rows(); ++index)
    baselines.push_back(baseline_transforms.block<4, 4>(4 * index, 0));
  const int view_count = view_transforms.rows() / 4;
  std::vector<Eigen::Matrix4d> views;
  views.reserve(view_count);
  for (int index = 0; index < view_count; ++index)
    views.push_back(view_transforms.block<4, 4>(4 * index, 0));

  std::vector<CameraBundleObservation> corners;
  corners.reserve(corner_matrix.rows());
  for (int row = 0; row < corner_matrix.rows(); ++row) {
    CameraBundleObservation observation;
    observation.source_index = row;
    observation.view_index = static_cast<int>(corner_matrix(row, 0));
    observation.camera_index = static_cast<int>(corner_matrix(row, 1));
    observation.target_point = corner_matrix.block<1, 3>(row, 2).transpose();
    observation.image_measurement =
        corner_matrix.block<1, 2>(row, 5).transpose();
    corners.push_back(observation);
  }

  CameraBundleProblem problem(cameras, baselines, views, corners);
  const CameraBundleReport initial_report = problem.residualReport();
  kalibr::ceres_optimizer::Options solver_options;
  solver_options.max_iterations = option(raw_options, "max_iterations", 50);
  solver_options.num_threads = option(raw_options, "num_threads", 0);
  solver_options.verbose = option(raw_options, "verbose", false);
  solver_options.structure =
      kalibr::ceres_optimizer::ProblemStructure::kBundleAdjustment;
  const auto summary = problem.solve(solver_options);
  if (!summary.usable)
    throw std::runtime_error(summary.full_report);

  const auto optimized_cameras = problem.cameras();
  Eigen::MatrixXd camera_output(camera_matrix.rows(), 10);
  for (int camera_index = 0; camera_index < camera_output.rows();
       ++camera_index) {
    for (int parameter = 0; parameter < 4; ++parameter)
      camera_output(camera_index, parameter) =
          optimized_cameras[camera_index].intrinsics[parameter];
    for (int parameter = 0; parameter < 5; ++parameter)
      camera_output(camera_index, 4 + parameter) =
          optimized_cameras[camera_index].distortion[parameter];
    camera_output(camera_index, 9) =
        optimized_cameras[camera_index].distortion_count;
  }
  const auto optimized_baselines = problem.cameraFromPrevious();
  Eigen::MatrixXd baseline_output(4 * optimized_baselines.size(), 4);
  for (std::size_t index = 0; index < optimized_baselines.size(); ++index)
    baseline_output.block<4, 4>(4 * index, 0) = optimized_baselines[index];
  const auto optimized_views = problem.camera0FromTarget();
  Eigen::MatrixXd view_output(4 * optimized_views.size(), 4);
  for (std::size_t index = 0; index < optimized_views.size(); ++index)
    view_output.block<4, 4>(4 * index, 0) = optimized_views[index];

  bp::dict result;
  result["usable"] = summary.usable;
  result["converged"] = summary.converged;
  result["iterations"] = summary.iterations;
  result["initial_cost"] = summary.initial_cost;
  result["final_cost"] = summary.final_cost;
  result["total_time_seconds"] = summary.total_time_seconds;
  result["num_threads"] = summary.num_threads;
  result["observations"] = problem.observationCount();
  result["residual_blocks"] = problem.residualBlockCount();
  result["initial_report"] = reportToDict(initial_report);
  result["final_report"] = reportToDict(problem.residualReport());
  result["cameras"] = camera_output;
  result["baselines"] = baseline_output;
  result["views"] = view_output;
  return result;
}

void applyControls(aslam::splines::BSplinePoseDesignVariable* spline,
                   const std::vector<std::array<double, 6>>& controls) {
  if (spline == nullptr || spline->numDesignVariables() != controls.size())
    throw std::runtime_error("Pose spline control count changed");
  for (std::size_t index = 0; index < controls.size(); ++index) {
    Eigen::Matrix<double, 6, 1> value(controls[index].data());
    spline->designVariable(index)->setParameters(value);
  }
}

void applyControls(aslam::splines::EuclideanBSplineDesignVariable* spline,
                   const std::vector<std::array<double, 3>>& controls) {
  if (spline == nullptr || spline->numDesignVariables() != controls.size())
    throw std::runtime_error("Bias spline control count changed");
  for (std::size_t index = 0; index < controls.size(); ++index) {
    Eigen::Vector3d value(controls[index].data());
    spline->designVariable(index)->setParameters(value);
  }
}

bp::dict solveJoint(
    aslam::splines::BSplinePoseDesignVariable* pose_spline,
    aslam::splines::EuclideanBSplineDesignVariable* gyroscope_bias_spline,
    aslam::splines::EuclideanBSplineDesignVariable* accelerometer_bias_spline,
    const Eigen::MatrixXd& imu_matrix,
    const Eigen::Matrix3d& gyroscope_inverse_covariance,
    const Eigen::Matrix3d& accelerometer_inverse_covariance,
    const Eigen::Vector3d& gravity, const Eigen::MatrixXd& camera_matrix,
    const Eigen::MatrixXd& camera_transforms,
    const Eigen::MatrixXd& corner_matrix,
    const Eigen::Matrix4d& initial_camera0_from_imu,
    const std::string& model_name, const Eigen::VectorXd& intrinsic_values,
    const bp::dict& raw_options) {
  if (pose_spline == nullptr || gyroscope_bias_spline == nullptr ||
      accelerometer_bias_spline == nullptr)
    throw std::invalid_argument("Spline design variables must not be null");
  if (imu_matrix.cols() != 7 || corner_matrix.cols() != 7)
    throw std::invalid_argument("IMU/corner bridge matrices must have 7 columns");

  ImuIntrinsics intrinsics;
  if (model_name == "calibrated") {
    intrinsics.model = ImuModel::kCalibrated;
  } else if (model_name == "scale-misalignment" ||
             model_name == "scale-misalignment-size-effect") {
    intrinsics.model = model_name == "scale-misalignment"
        ? ImuModel::kScaleMisalignment
        : ImuModel::kScaleMisalignmentSizeEffect;
    const int expected_size = intrinsics.model == ImuModel::kScaleMisalignment
        ? 36
        : 45;
    if (intrinsic_values.size() != expected_size)
      throw std::invalid_argument("Invalid flattened IMU intrinsic vector");
    using RowMajorMatrix3d =
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    const Eigen::Map<const RowMajorMatrix3d> M_accel(intrinsic_values.data());
    const Eigen::Map<const RowMajorMatrix3d> C_gyro_i(
        intrinsic_values.data() + 9);
    const Eigen::Map<const RowMajorMatrix3d> M_gyro(
        intrinsic_values.data() + 18);
    const Eigen::Map<const RowMajorMatrix3d> A(intrinsic_values.data() + 27);
    intrinsics.accelerometer_matrix = lowerTriangle(M_accel);
    intrinsics.gyroscope_matrix = lowerTriangle(M_gyro);
    const Eigen::Quaterniond quaternion(C_gyro_i);
    intrinsics.gyro_from_imu_quaternion = {
        quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> sensitivity(
        intrinsics.acceleration_sensitivity.data());
    sensitivity = A;
    if (intrinsics.model == ImuModel::kScaleMisalignmentSizeEffect) {
      std::copy(intrinsic_values.data() + 36,
                intrinsic_values.data() + 39, intrinsics.lever_x.begin());
      std::copy(intrinsic_values.data() + 39,
                intrinsic_values.data() + 42, intrinsics.lever_y.begin());
      std::copy(intrinsic_values.data() + 42,
                intrinsic_values.data() + 45, intrinsics.lever_z.begin());
    }
  } else {
    throw std::invalid_argument("Unsupported IMU model for Ceres bridge");
  }

  std::vector<ImuObservation> imu;
  imu.reserve(imu_matrix.rows());
  for (int row = 0; row < imu_matrix.rows(); ++row) {
    ImuObservation observation;
    observation.timestamp = imu_matrix(row, 0);
    observation.gyroscope = imu_matrix.block<1, 3>(row, 1).transpose();
    observation.accelerometer = imu_matrix.block<1, 3>(row, 4).transpose();
    observation.gyroscope_inverse_covariance = gyroscope_inverse_covariance;
    observation.accelerometer_inverse_covariance =
        accelerometer_inverse_covariance;
    imu.push_back(observation);
  }

  ImuProblemOptions imu_options;
  imu_options.optimize_pose = true;
  imu_options.optimize_biases = true;
  imu_options.optimize_gravity_direction = true;
  imu_options.optimize_intrinsics = intrinsics.model != ImuModel::kCalibrated;
  imu_options.add_bias_motion_error = true;
  imu_options.gyroscope_noise_scale =
      option(raw_options, "gyroscope_noise_scale", 1.0);
  imu_options.accelerometer_noise_scale =
      option(raw_options, "accelerometer_noise_scale", 1.0);
  imu_options.gyroscope_huber_width =
      option(raw_options, "gyroscope_huber_width", -1.0);
  imu_options.accelerometer_huber_width =
      option(raw_options, "accelerometer_huber_width", -1.0);
  imu_options.gyroscope_random_walk =
      option(raw_options, "gyroscope_random_walk", 1.0);
  imu_options.accelerometer_random_walk =
      option(raw_options, "accelerometer_random_walk", 1.0);

  ImuProblem problem(pose_spline->spline(), gyroscope_bias_spline->spline(),
                     accelerometer_bias_spline->spline(), std::move(imu),
                     gravity, intrinsics, imu_options);

  std::vector<CameraChainEntry> cameras;
  if (camera_matrix.cols() != 12 || camera_matrix.rows() <= 0 ||
      camera_transforms.cols() != 4 ||
      camera_transforms.rows() != 4 * camera_matrix.rows()) {
    throw std::invalid_argument("Invalid flattened camera bridge matrices");
  }
  const int camera_count = camera_matrix.rows();
  cameras.reserve(camera_count);
  for (int index = 0; index < camera_count; ++index) {
    CameraChainEntry camera;
    for (int parameter = 0; parameter < 4; ++parameter)
      camera.camera.intrinsics[parameter] = camera_matrix(index, parameter);
    camera.camera.distortion_count = static_cast<int>(camera_matrix(index, 9));
    for (int parameter = 0; parameter < 5; ++parameter)
      camera.camera.distortion[parameter] = camera_matrix(index, 4 + parameter);
    const Eigen::Matrix4d transform =
        camera_transforms.block<4, 4>(4 * index, 0);
    camera.camera_from_camera0_rotation = transform.topLeftCorner<3, 3>();
    camera.camera_from_camera0_translation = transform.topRightCorner<3, 1>();
    camera.time_shift_prior = camera_matrix(index, 10);
    cameras.push_back(camera);
  }

  std::vector<CameraCornerObservation> corners;
  corners.reserve(corner_matrix.rows());
  for (int row = 0; row < corner_matrix.rows(); ++row) {
    CameraCornerObservation observation;
    observation.source_index = row;
    observation.timestamp = corner_matrix(row, 0);
    observation.camera_index = static_cast<int>(corner_matrix(row, 1));
    observation.target_point = corner_matrix.block<1, 3>(row, 2).transpose();
    observation.image_measurement = corner_matrix.block<1, 2>(row, 5).transpose();
    const double sigma = camera_matrix(observation.camera_index, 11);
    observation.inverse_covariance = Eigen::Matrix2d::Identity() /
        (sigma * sigma);
    corners.push_back(observation);
  }
  CameraProblemOptions camera_options;
  camera_options.optimize_camera0_from_imu = true;
  camera_options.estimate_time_offsets =
      option(raw_options, "estimate_time_offsets", true);
  camera_options.time_offset_padding =
      option(raw_options, "time_offset_padding", 0.03);
  const int camera_residuals = problem.addCameraResiduals(
      cameras, corners, initial_camera0_from_imu.topLeftCorner<3, 3>(),
      initial_camera0_from_imu.topRightCorner<3, 1>(), camera_options);
  // The problem owns immutable copies required by its residuals. Release the
  // bridge-side observation structs before the linear solver allocates its
  // Jacobian and factorization workspace.
  cameras.clear();
  cameras.shrink_to_fit();
  corners.clear();
  corners.shrink_to_fit();

  const JointResidualReport initial_report = problem.residualReport();
  kalibr::ceres_optimizer::Options solver_options;
  solver_options.max_iterations = option(raw_options, "max_iterations", 30);
  solver_options.num_threads = option(raw_options, "num_threads", 0);
  solver_options.verbose = option(raw_options, "verbose", false);
  const auto summary = problem.solve(solver_options);
  if (!summary.usable)
    throw std::runtime_error(summary.full_report);

  applyControls(pose_spline, problem.poseControls());
  applyControls(gyroscope_bias_spline, problem.gyroscopeBiasControls());
  applyControls(accelerometer_bias_spline,
                problem.accelerometerBiasControls());

  bp::dict result;
  result["usable"] = summary.usable;
  result["converged"] = summary.converged;
  result["iterations"] = summary.iterations;
  result["initial_cost"] = summary.initial_cost;
  result["final_cost"] = summary.final_cost;
  result["total_time_seconds"] = summary.total_time_seconds;
  result["num_threads"] = summary.num_threads;
  result["imu_observations"] = problem.observationCount();
  result["imu_residual_blocks"] = problem.imuResidualBlockCount();
  result["camera_residuals"] = camera_residuals;
  result["initial_report"] = reportToDict(initial_report);
  result["final_report"] = reportToDict(problem.residualReport());
  result["gravity"] = problem.gravity();
  Eigen::Matrix4d camera0_from_imu = Eigen::Matrix4d::Identity();
  camera0_from_imu.topLeftCorner<3, 3>() =
      problem.camera0FromImuRotation();
  camera0_from_imu.topRightCorner<3, 1>() =
      problem.camera0FromImuTranslation();
  result["camera0_from_imu"] = camera0_from_imu;
  const std::vector<double> time_offsets = problem.cameraTimeOffsets();
  const Eigen::VectorXd time_offset_vector = Eigen::Map<const Eigen::VectorXd>(
      time_offsets.data(), time_offsets.size());
  result["time_offsets"] = time_offset_vector;
  const ImuIntrinsics& optimized_intrinsics = problem.intrinsics();
  if (optimized_intrinsics.model != ImuModel::kCalibrated) {
    result["M_accel"] = lowerTriangle(
        optimized_intrinsics.accelerometer_matrix);
    result["M_gyro"] = lowerTriangle(
        optimized_intrinsics.gyroscope_matrix);
    const Eigen::Matrix3d sensitivity =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            optimized_intrinsics.acceleration_sensitivity.data());
    result["A"] = sensitivity;
    const auto& q = optimized_intrinsics.gyro_from_imu_quaternion;
    result["C_gyro_i"] =
        Eigen::Quaterniond(q[0], q[1], q[2], q[3])
            .normalized()
            .toRotationMatrix();
    if (optimized_intrinsics.model ==
        ImuModel::kScaleMisalignmentSizeEffect) {
      result["rx"] = Eigen::Vector3d(optimized_intrinsics.lever_x.data());
      result["ry"] = Eigen::Vector3d(optimized_intrinsics.lever_y.data());
      result["rz"] = Eigen::Vector3d(optimized_intrinsics.lever_z.data());
    }
  }
  return result;
}

void setDesignVariableParameters(aslam::backend::DesignVariable* variable,
                                 const Eigen::MatrixXd& parameters) {
  if (variable == nullptr)
    throw std::invalid_argument("Design variable must not be null");
  variable->setParameters(parameters);
}

}  // namespace

BOOST_PYTHON_MODULE(libkalibr_ceres_optimizer_python) {
  bp::def("solve_camera_bundle", &solveCameraBundle);
  bp::def("solve_joint", &solveJoint);
  bp::def("set_design_variable_parameters", &setDesignVariableParameters);
}
