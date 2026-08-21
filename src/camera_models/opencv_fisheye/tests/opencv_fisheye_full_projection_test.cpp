#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>

#include <kalibr_no_ros/opencv_fisheye_full/OpenCvFisheyeProjection.hpp>

namespace {

void requireNear(double actual, double expected, double tolerance,
                 const char* message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(std::string(message) + ": actual=" +
                             std::to_string(actual) + " expected=" +
                             std::to_string(expected));
  }
}

template <typename A, typename B>
void requireMatrixNear(const A& actual, const B& expected, double tolerance,
                       const char* message) {
  if (actual.rows() != expected.rows() || actual.cols() != expected.cols() ||
      (actual - expected).cwiseAbs().maxCoeff() > tolerance) {
    throw std::runtime_error(message);
  }
}

Eigen::Vector2d project(
    const aslam::cameras::OpenCvFisheyeProjection& projection,
    const Eigen::Vector3d& point) {
  Eigen::Vector2d pixel;
  projection.euclideanToKeypoint(point, pixel);
  return pixel;
}

}  // namespace

int main() {
  using aslam::cameras::OpenCvFisheyeDistortion;
  using aslam::cameras::OpenCvFisheyeProjection;

  const double fu = 407.5;
  const double fv = 398.25;
  const double cu = 321.2;
  const double cv = 238.7;
  const double alpha = 0.037;
  const OpenCvFisheyeDistortion distortion(-0.021, 0.0042, -0.00071,
                                            0.000052);
  OpenCvFisheyeProjection projection(fu, fv, cu, cv, alpha, 1280, 960,
                                     distortion);
  const Eigen::Vector3d point(0.31, -0.17, 1.23);

  const Eigen::Vector2d actual = project(projection, point);
  std::vector<cv::Point3d> object_points{
      cv::Point3d(point[0], point[1], point[2])};
  std::vector<cv::Point2d> image_points;
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fu, 0.0, cu, 0.0,
                           fv, cv, 0.0, 0.0, 1.0);
  cv::Mat coefficients = (cv::Mat_<double>(4, 1) << distortion.k1(),
                          distortion.k2(), distortion.k3(), distortion.k4());
  cv::fisheye::projectPoints(object_points, image_points, cv::Vec3d(0, 0, 0),
                             cv::Vec3d(0, 0, 0), camera_matrix, coefficients,
                             alpha);
  requireNear(actual[0], image_points[0].x, 2e-12,
              "u differs from cv::fisheye::projectPoints");
  requireNear(actual[1], image_points[0].y, 2e-12,
              "v differs from cv::fisheye::projectPoints");

  Eigen::Vector2d projected;
  Eigen::Matrix<double, 2, 3> point_jacobian;
  projection.euclideanToKeypoint(point, projected, point_jacobian);
  Eigen::Matrix<double, 2, 3> point_fd;
  const double step = 1e-7;
  for (int column = 0; column < 3; ++column) {
    Eigen::Vector3d plus = point;
    Eigen::Vector3d minus = point;
    plus[column] += step;
    minus[column] -= step;
    point_fd.col(column) =
        (project(projection, plus) - project(projection, minus)) /
        (2.0 * step);
  }
  requireMatrixNear(point_jacobian, point_fd, 2e-6,
                    "point Jacobian differs from finite differences");

  Eigen::Matrix<double, 2, 5> intrinsic_jacobian;
  projection.euclideanToKeypointIntrinsicsJacobian(point,
                                                    intrinsic_jacobian);
  Eigen::Matrix<double, 2, 5> intrinsic_fd;
  Eigen::MatrixXd original_parameters;
  projection.getParameters(original_parameters);
  for (int column = 0; column < 5; ++column) {
    Eigen::MatrixXd plus = original_parameters;
    Eigen::MatrixXd minus = original_parameters;
    plus(column, 0) += step;
    minus(column, 0) -= step;
    projection.setParameters(plus);
    const Eigen::Vector2d plus_pixel = project(projection, point);
    projection.setParameters(minus);
    const Eigen::Vector2d minus_pixel = project(projection, point);
    intrinsic_fd.col(column) = (plus_pixel - minus_pixel) / (2.0 * step);
  }
  projection.setParameters(original_parameters);
  requireMatrixNear(intrinsic_jacobian, intrinsic_fd, 2e-6,
                    "intrinsic Jacobian differs from finite differences");

  Eigen::Matrix<double, 2, 4> distortion_jacobian;
  projection.euclideanToKeypointDistortionJacobian(point,
                                                    distortion_jacobian);
  Eigen::Matrix<double, 2, 4> distortion_fd;
  Eigen::MatrixXd original_distortion;
  projection.distortion().getParameters(original_distortion);
  for (int column = 0; column < 4; ++column) {
    Eigen::MatrixXd plus = original_distortion;
    Eigen::MatrixXd minus = original_distortion;
    plus(column, 0) += step;
    minus(column, 0) -= step;
    projection.distortion().setParameters(plus);
    const Eigen::Vector2d plus_pixel = project(projection, point);
    projection.distortion().setParameters(minus);
    const Eigen::Vector2d minus_pixel = project(projection, point);
    distortion_fd.col(column) = (plus_pixel - minus_pixel) / (2.0 * step);
  }
  projection.distortion().setParameters(original_distortion);
  requireMatrixNear(distortion_jacobian, distortion_fd, 2e-6,
                    "distortion Jacobian differs from finite differences");

  Eigen::Vector3d ray;
  Eigen::Matrix<double, 3, 2> back_jacobian;
  projection.keypointToEuclidean(actual, ray, back_jacobian);
  requireNear(ray[0] / ray[2], point[0] / point[2], 5e-11,
              "back-projected x differs");
  requireNear(ray[1] / ray[2], point[1] / point[2], 5e-11,
              "back-projected y differs");

  Eigen::Matrix<double, 3, 2> back_fd;
  for (int column = 0; column < 2; ++column) {
    Eigen::Vector2d plus = actual;
    Eigen::Vector2d minus = actual;
    plus[column] += step;
    minus[column] -= step;
    Eigen::Vector3d plus_ray;
    Eigen::Vector3d minus_ray;
    projection.keypointToEuclidean(plus, plus_ray);
    projection.keypointToEuclidean(minus, minus_ray);
    back_fd.col(column) = (plus_ray - minus_ray) / (2.0 * step);
  }
  requireMatrixNear(back_jacobian, back_fd, 2e-6,
                    "back-projection Jacobian differs from finite differences");

  requireNear(projection.getCameraMatrix()(0, 1), fu * alpha, 1e-15,
              "K skew does not equal fu*alpha");

  // OpenCV emits a fixed sentinel when Newton fails (or theta flips sign).
  // The projection must propagate that as `false`, never as a usable ray.
  const OpenCvFisheyeDistortion failing_distortion(
      0.00862475, 0.00525583, -0.00358072, -0.00621336);
  OpenCvFisheyeProjection failing_projection(
      fu, fv, cu, cv, alpha, 1280, 960, failing_distortion);
  const Eigen::Vector2d failing_distorted(0.95448856, 0.96440625);
  const Eigen::Vector2d failing_pixel(
      fu * (failing_distorted[0] + alpha * failing_distorted[1]) + cu,
      fv * failing_distorted[1] + cv);
  Eigen::Vector3d failed_ray;
  Eigen::Matrix<double, 3, 2> failed_jacobian;
  const bool backprojection_valid = failing_projection.keypointToEuclidean(
      failing_pixel, failed_ray, failed_jacobian);
  if (backprojection_valid) {
    throw std::runtime_error("failed OpenCV Newton solve returned a valid ray");
  }
  requireNear(failed_ray[0], -1000000.0, 0.0,
              "failed ray x is not OpenCV sentinel");
  requireNear(failed_ray[1], -1000000.0, 0.0,
              "failed ray y is not OpenCV sentinel");
  requireMatrixNear(failed_jacobian,
                    Eigen::Matrix<double, 3, 2>::Zero(), 0.0,
                    "failed inverse Jacobian must be zero");

  std::vector<cv::Point2d> distorted_points{
      cv::Point2d(failing_distorted[0], failing_distorted[1])};
  std::vector<cv::Point2d> undistorted_points;
  cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat failing_coefficients =
      (cv::Mat_<double>(4, 1) << failing_distortion.k1(),
       failing_distortion.k2(), failing_distortion.k3(),
       failing_distortion.k4());
  cv::fisheye::undistortPoints(distorted_points, undistorted_points, identity,
                               failing_coefficients);
  requireNear(undistorted_points[0].x, -1000000.0, 0.0,
              "OpenCV failure fixture no longer returns sentinel x");
  requireNear(undistorted_points[0].y, -1000000.0, 0.0,
              "OpenCV failure fixture no longer returns sentinel y");

  std::cout << "full OpenCV fisheye projection tests passed\n";
  return 0;
}
