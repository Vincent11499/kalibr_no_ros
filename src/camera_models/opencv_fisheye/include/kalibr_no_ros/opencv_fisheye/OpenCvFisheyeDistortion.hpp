#ifndef KALIBR_NO_ROS_OPENCV_FISHEYE_DISTORTION_HPP
#define KALIBR_NO_ROS_OPENCV_FISHEYE_DISTORTION_HPP

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>

#include <aslam/cameras/StaticAssert.hpp>

namespace aslam {
namespace cameras {

/// OpenCV's cv::fisheye four-coefficient distortion block:
/// D = [k1, k2, k3, k4].
///
/// The projection supplies the normalized point [x/z, y/z]. This class
/// applies OpenCV's theta polynomial before the projection applies alpha/skew:
///   theta_d = theta * (1 + k1 theta^2 + ... + k4 theta^8)
///   [x_d, y_d] = theta_d / r * [x, y], theta = atan(r).
///
/// The forward formula matches Kalibr's native equidistant model. This
/// distortion block preserves OpenCV's inverse convergence and failure
/// behavior for the nine-parameter OpenCV fisheye projection.
class OpenCvFisheyeDistortion {
 public:
  enum { IntrinsicsDimension = 4 };
  enum { DesignVariableDimension = IntrinsicsDimension };

  OpenCvFisheyeDistortion()
      : _k1(0.0), _k2(0.0), _k3(0.0), _k4(0.0) {}

  OpenCvFisheyeDistortion(double k1, double k2, double k3, double k4)
      : _k1(k1), _k2(k2), _k3(k3), _k4(k4) {}

  template <typename DERIVED_Y>
  void distort(const Eigen::MatrixBase<DERIVED_Y>& yconst) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_Y>, 2);
    Eigen::MatrixBase<DERIVED_Y>& y =
        const_cast<Eigen::MatrixBase<DERIVED_Y>&>(yconst);
    y.derived().resize(2);

    const double x = y[0];
    const double yy = y[1];
    const double r = std::sqrt(x * x + yy * yy);
    if (r <= axisEpsilon()) {
      return;
    }
    const double theta = std::atan(r);
    y *= distortedTheta(theta) / r;
  }

  template <typename DERIVED_Y, typename DERIVED_JY>
  void distort(const Eigen::MatrixBase<DERIVED_Y>& yconst,
               const Eigen::MatrixBase<DERIVED_JY>& outJy) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_Y>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JY>, 2, 2);
    Eigen::MatrixBase<DERIVED_Y>& y =
        const_cast<Eigen::MatrixBase<DERIVED_Y>&>(yconst);
    Eigen::MatrixBase<DERIVED_JY>& J =
        const_cast<Eigen::MatrixBase<DERIVED_JY>&>(outJy);
    y.derived().resize(2);
    J.derived().resize(2, 2);

    const double x = y[0];
    const double yy = y[1];
    const double r2 = x * x + yy * yy;
    const double r = std::sqrt(r2);
    if (r <= axisEpsilon()) {
      J.setIdentity();
      return;
    }

    const double theta = std::atan(r);
    const double theta_d = distortedTheta(theta);
    const double scale = theta_d / r;
    const double dtheta_d_dr = distortedThetaDerivative(theta) / (1.0 + r2);
    const double scale_slope_over_r =
        (dtheta_d_dr * r - theta_d) / (r2 * r);

    J(0, 0) = scale + scale_slope_over_r * x * x;
    J(0, 1) = scale_slope_over_r * x * yy;
    J(1, 0) = J(0, 1);
    J(1, 1) = scale + scale_slope_over_r * yy * yy;

    y[0] = x * scale;
    y[1] = yy * scale;
  }

  template <typename DERIVED>
  void undistort(const Eigen::MatrixBase<DERIVED>& yconst) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED>, 2);
    Eigen::MatrixBase<DERIVED>& y =
        const_cast<Eigen::MatrixBase<DERIVED>&>(yconst);
    y.derived().resize(2);

    undistortImpl(y);
  }

  template <typename DERIVED, typename DERIVED_JY>
  void undistort(const Eigen::MatrixBase<DERIVED>& yconst,
                 const Eigen::MatrixBase<DERIVED_JY>& outJy) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JY>, 2, 2);
    Eigen::MatrixBase<DERIVED>& y =
        const_cast<Eigen::MatrixBase<DERIVED>&>(yconst);
    const Eigen::Vector2d distorted = y;
    const double distorted_radius = distorted.norm();
    const bool converged = undistortImpl(y);
    Eigen::MatrixBase<DERIVED_JY>& J =
        const_cast<Eigen::MatrixBase<DERIVED_JY>&>(outJy);
    J.derived().resize(2, 2);
    if (!converged) {
      // OpenCV returns one constant sentinel for every failed Newton solve.
      // Its derivative is therefore zero away from the convergence boundary.
      J.setZero();
      return;
    }
    const double half_pi = 1.57079632679489661923;
    if (distorted_radius > half_pi) {
      // With OpenCV's theta_d clipping, the solved scale is constant outside
      // the pi/2 circle: undistort(p) = scale * p.  Inverting the forward
      // distortion Jacobian here is incorrect because distort(undistort(p))
      // no longer equals p in this clipped domain.
      const double scale =
          distorted.dot(y) / (distorted_radius * distorted_radius);
      J.setIdentity();
      J *= scale;
    } else {
      Eigen::Vector2d projected = y;
      Eigen::Matrix2d forward_jacobian;
      distort(projected, forward_jacobian);
      J = forward_jacobian.inverse();
    }
  }

  template <typename DERIVED_Y, typename DERIVED_JD>
  void distortParameterJacobian(
      const Eigen::MatrixBase<DERIVED_Y>& imageY,
      const Eigen::MatrixBase<DERIVED_JD>& outJd) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_Y>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JD>, 2, 4);
    Eigen::MatrixBase<DERIVED_JD>& J =
        const_cast<Eigen::MatrixBase<DERIVED_JD>&>(outJd);
    J.derived().resize(2, 4);
    J.setZero();

    const double x = imageY[0];
    const double y = imageY[1];
    const double r = std::sqrt(x * x + y * y);
    if (r <= axisEpsilon()) {
      return;
    }
    const double theta = std::atan(r);
    const double theta2 = theta * theta;
    const double theta3_over_r = theta * theta2 / r;
    const double theta5_over_r = theta3_over_r * theta2;
    const double theta7_over_r = theta5_over_r * theta2;
    const double theta9_over_r = theta7_over_r * theta2;
    J.row(0) << x * theta3_over_r, x * theta5_over_r,
        x * theta7_over_r, x * theta9_over_r;
    J.row(1) << y * theta3_over_r, y * theta5_over_r,
        y * theta7_over_r, y * theta9_over_r;
  }

  void update(const double* v) {
    _k1 += v[0];
    _k2 += v[1];
    _k3 += v[2];
    _k4 += v[3];
  }

  int minimalDimensions() const { return IntrinsicsDimension; }

  void getParameters(Eigen::MatrixXd& parameters) const {
    parameters.resize(4, 1);
    parameters << _k1, _k2, _k3, _k4;
  }

  void setParameters(const Eigen::MatrixXd& parameters) {
    _k1 = parameters(0, 0);
    _k2 = parameters(1, 0);
    _k3 = parameters(2, 0);
    _k4 = parameters(3, 0);
  }

  Eigen::Vector2i parameterSize() const { return Eigen::Vector2i(4, 1); }

  double k1() const { return _k1; }
  double k2() const { return _k2; }
  double k3() const { return _k3; }
  double k4() const { return _k4; }

  void clear() { _k1 = _k2 = _k3 = _k4 = 0.0; }

  bool isBinaryEqual(const OpenCvFisheyeDistortion& rhs) const {
    return _k1 == rhs._k1 && _k2 == rhs._k2 && _k3 == rhs._k3 &&
           _k4 == rhs._k4;
  }

  static OpenCvFisheyeDistortion getTestDistortion() {
    return OpenCvFisheyeDistortion(-0.01, 0.003, -0.0004, 0.00002);
  }

  enum { CLASS_SERIALIZATION_VERSION = 0 };
  BOOST_SERIALIZATION_SPLIT_MEMBER();

  template <class Archive>
  void save(Archive& archive, const unsigned int) const {
    archive << BOOST_SERIALIZATION_NVP(_k1);
    archive << BOOST_SERIALIZATION_NVP(_k2);
    archive << BOOST_SERIALIZATION_NVP(_k3);
    archive << BOOST_SERIALIZATION_NVP(_k4);
  }

  template <class Archive>
  void load(Archive& archive, const unsigned int) {
    archive >> BOOST_SERIALIZATION_NVP(_k1);
    archive >> BOOST_SERIALIZATION_NVP(_k2);
    archive >> BOOST_SERIALIZATION_NVP(_k3);
    archive >> BOOST_SERIALIZATION_NVP(_k4);
  }

 private:
  static double axisEpsilon() { return 1e-12; }

  template <typename DERIVED>
  bool undistortImpl(Eigen::MatrixBase<DERIVED>& y) const {
    const double distorted_radius = y.norm();
    if (distorted_radius <= axisEpsilon()) {
      return true;
    }

    // Mirror cv::fisheye::undistortPoints.  OpenCV clips the distorted angle
    // supplied to Newton (rather than the solved undistorted angle), uses ten
    // Newton steps with an 1e-8 step tolerance, rejects a root whose sign has
    // flipped, and emits a fixed sentinel when the solve does not converge.
    const double half_pi = 1.57079632679489661923;
    const double theta_d = std::min(distorted_radius, half_pi);
    const double newton_tolerance = 1e-8;
    double theta = theta_d;
    bool converged = false;
    for (int iteration = 0; iteration < 10; ++iteration) {
      const double derivative = distortedThetaDerivative(theta);
      if (!std::isfinite(derivative) || derivative == 0.0) {
        break;
      }
      const double step =
          (distortedTheta(theta) - theta_d) / derivative;
      theta -= step;
      if (!std::isfinite(theta) || !std::isfinite(step)) {
        break;
      }
      if (std::abs(step) < newton_tolerance) {
        converged = true;
        break;
      }
    }

    const bool theta_flipped = theta_d > 0.0 && theta < 0.0;
    if (!converged || theta_flipped) {
      y.setConstant(-1000000.0);
      return false;
    }

    // OpenCV applies the scale to the original (unclipped) distorted point,
    // while using the clipped theta_d in the denominator.
    y *= std::tan(theta) / theta_d;
    return true;
  }

  double distortedTheta(double theta) const {
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    return theta *
           (1.0 + _k1 * theta2 + _k2 * theta4 + _k3 * theta6 +
            _k4 * theta8);
  }

  double distortedThetaDerivative(double theta) const {
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    return 1.0 + 3.0 * _k1 * theta2 + 5.0 * _k2 * theta4 +
           7.0 * _k3 * theta6 + 9.0 * _k4 * theta8;
  }

  double _k1;
  double _k2;
  double _k3;
  double _k4;
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::OpenCvFisheyeDistortion);

#endif
