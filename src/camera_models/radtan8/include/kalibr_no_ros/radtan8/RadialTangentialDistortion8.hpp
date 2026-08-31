#ifndef KALIBR_NO_ROS_RADIAL_TANGENTIAL_DISTORTION8_HPP
#define KALIBR_NO_ROS_RADIAL_TANGENTIAL_DISTORTION8_HPP

#include <Eigen/Dense>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>

#include <aslam/cameras/StaticAssert.hpp>

namespace aslam {
namespace cameras {

/// OpenCV's rational pinhole model: [k1, k2, p1, p2, k3, k4, k5, k6].
class RadialTangentialDistortion8 {
 public:
  enum { IntrinsicsDimension = 8 };
  enum { DesignVariableDimension = IntrinsicsDimension };

  RadialTangentialDistortion8()
      : _k1(0.0), _k2(0.0), _p1(0.0), _p2(0.0), _k3(0.0),
        _k4(0.0), _k5(0.0), _k6(0.0) {}

  RadialTangentialDistortion8(double k1, double k2, double p1, double p2,
                              double k3, double k4, double k5, double k6)
      : _k1(k1), _k2(k2), _p1(p1), _p2(p2), _k3(k3),
        _k4(k4), _k5(k5), _k6(k6) {}

  template <typename DERIVED_Y>
  void distort(const Eigen::MatrixBase<DERIVED_Y>& yconst) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_Y>, 2);
    Eigen::MatrixBase<DERIVED_Y>& y =
        const_cast<Eigen::MatrixBase<DERIVED_Y>&>(yconst);
    y.derived().resize(2);

    const double x = y[0];
    const double yy = y[1];
    const double x2 = x * x;
    const double y2 = yy * yy;
    const double r2 = x2 + y2;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double numerator = 1.0 + _k1 * r2 + _k2 * r4 + _k3 * r6;
    const double denominator = 1.0 + _k4 * r2 + _k5 * r4 + _k6 * r6;
    const double radial = numerator / denominator;

    y[0] = x * radial + 2.0 * _p1 * x * yy +
           _p2 * (r2 + 2.0 * x2);
    y[1] = yy * radial + 2.0 * _p2 * x * yy +
           _p1 * (r2 + 2.0 * y2);
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
    const double x2 = x * x;
    const double y2 = yy * yy;
    const double r2 = x2 + y2;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double numerator = 1.0 + _k1 * r2 + _k2 * r4 + _k3 * r6;
    const double denominator = 1.0 + _k4 * r2 + _k5 * r4 + _k6 * r6;
    const double numerator_slope = _k1 + 2.0 * _k2 * r2 +
                                   3.0 * _k3 * r4;
    const double denominator_slope = _k4 + 2.0 * _k5 * r2 +
                                     3.0 * _k6 * r4;
    const double radial = numerator / denominator;
    const double twice_radial_slope =
        2.0 * (numerator_slope * denominator -
               numerator * denominator_slope) /
        (denominator * denominator);

    J(0, 0) = radial + x2 * twice_radial_slope + 2.0 * _p1 * yy +
              6.0 * _p2 * x;
    J(0, 1) = x * yy * twice_radial_slope + 2.0 * _p1 * x +
              2.0 * _p2 * yy;
    J(1, 0) = J(0, 1);
    J(1, 1) = radial + y2 * twice_radial_slope + 6.0 * _p1 * yy +
              2.0 * _p2 * x;

    y[0] = x * radial + 2.0 * _p1 * x * yy +
           _p2 * (r2 + 2.0 * x2);
    y[1] = yy * radial + 2.0 * _p2 * x * yy +
           _p1 * (r2 + 2.0 * y2);
  }

  template <typename DERIVED>
  void undistort(const Eigen::MatrixBase<DERIVED>& yconst) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED>, 2);
    Eigen::MatrixBase<DERIVED>& y =
        const_cast<Eigen::MatrixBase<DERIVED>&>(yconst);
    y.derived().resize(2);
    const Eigen::Vector2d measurement = y;
    Eigen::Vector2d estimate = measurement;
    for (int iteration = 0; iteration < 10; ++iteration) {
      Eigen::Vector2d projected = estimate;
      Eigen::Matrix2d J;
      distort(projected, J);
      const Eigen::Vector2d error = measurement - projected;
      estimate += J.colPivHouseholderQr().solve(error);
      if (error.squaredNorm() < 1e-15) {
        break;
      }
    }
    y = estimate;
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
    undistort(y);
    Eigen::Vector2d projected = y;
    Eigen::Matrix2d J;
    distort(projected, J);
    DERIVED_JY& output = const_cast<DERIVED_JY&>(outJy.derived());
    output = J.inverse();
  }

  template <typename DERIVED_Y, typename DERIVED_JD>
  void distortParameterJacobian(
      const Eigen::MatrixBase<DERIVED_Y>& imageY,
      const Eigen::MatrixBase<DERIVED_JD>& outJd) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_Y>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JD>, 2, 8);
    Eigen::MatrixBase<DERIVED_JD>& J =
        const_cast<Eigen::MatrixBase<DERIVED_JD>&>(outJd);
    J.derived().resize(2, 8);
    J.setZero();

    const double x = imageY[0];
    const double y = imageY[1];
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double numerator = 1.0 + _k1 * r2 + _k2 * r4 + _k3 * r6;
    const double denominator = 1.0 + _k4 * r2 + _k5 * r4 + _k6 * r6;
    const double inverse_denominator = 1.0 / denominator;
    const double inverse_denominator_squared =
        inverse_denominator * inverse_denominator;
    const double radial_powers[3] = {r2, r4, r6};

    J(0, 0) = x * radial_powers[0] * inverse_denominator;
    J(0, 1) = x * radial_powers[1] * inverse_denominator;
    J(0, 2) = 2.0 * x * y;
    J(0, 3) = r2 + 2.0 * x * x;
    J(0, 4) = x * radial_powers[2] * inverse_denominator;
    J(1, 0) = y * radial_powers[0] * inverse_denominator;
    J(1, 1) = y * radial_powers[1] * inverse_denominator;
    J(1, 2) = r2 + 2.0 * y * y;
    J(1, 3) = 2.0 * x * y;
    J(1, 4) = y * radial_powers[2] * inverse_denominator;

    for (int index = 0; index < 3; ++index) {
      const int column = 5 + index;
      const double derivative =
          -numerator * radial_powers[index] * inverse_denominator_squared;
      J(0, column) = x * derivative;
      J(1, column) = y * derivative;
    }
  }

  void update(const double* v) {
    _k1 += v[0];
    _k2 += v[1];
    _p1 += v[2];
    _p2 += v[3];
    _k3 += v[4];
    _k4 += v[5];
    _k5 += v[6];
    _k6 += v[7];
  }

  int minimalDimensions() const { return IntrinsicsDimension; }

  void getParameters(Eigen::MatrixXd& parameters) const {
    parameters.resize(8, 1);
    parameters << _k1, _k2, _p1, _p2, _k3, _k4, _k5, _k6;
  }

  void setParameters(const Eigen::MatrixXd& parameters) {
    _k1 = parameters(0, 0);
    _k2 = parameters(1, 0);
    _p1 = parameters(2, 0);
    _p2 = parameters(3, 0);
    _k3 = parameters(4, 0);
    _k4 = parameters(5, 0);
    _k5 = parameters(6, 0);
    _k6 = parameters(7, 0);
  }

  Eigen::Vector2i parameterSize() const { return Eigen::Vector2i(8, 1); }

  double k1() const { return _k1; }
  double k2() const { return _k2; }
  double p1() const { return _p1; }
  double p2() const { return _p2; }
  double k3() const { return _k3; }
  double k4() const { return _k4; }
  double k5() const { return _k5; }
  double k6() const { return _k6; }

  void clear() {
    _k1 = _k2 = _p1 = _p2 = _k3 = _k4 = _k5 = _k6 = 0.0;
  }

  bool isBinaryEqual(const RadialTangentialDistortion8& rhs) const {
    return _k1 == rhs._k1 && _k2 == rhs._k2 && _p1 == rhs._p1 &&
           _p2 == rhs._p2 && _k3 == rhs._k3 && _k4 == rhs._k4 &&
           _k5 == rhs._k5 && _k6 == rhs._k6;
  }

  static RadialTangentialDistortion8 getTestDistortion() {
    return RadialTangentialDistortion8(
        -0.2, 0.13, 0.0005, 0.0005, -0.02, 0.01, -0.005, 0.001);
  }

  enum { CLASS_SERIALIZATION_VERSION = 0 };
  BOOST_SERIALIZATION_SPLIT_MEMBER();

  template <class Archive>
  void save(Archive& archive, const unsigned int) const {
    archive << BOOST_SERIALIZATION_NVP(_k1);
    archive << BOOST_SERIALIZATION_NVP(_k2);
    archive << BOOST_SERIALIZATION_NVP(_p1);
    archive << BOOST_SERIALIZATION_NVP(_p2);
    archive << BOOST_SERIALIZATION_NVP(_k3);
    archive << BOOST_SERIALIZATION_NVP(_k4);
    archive << BOOST_SERIALIZATION_NVP(_k5);
    archive << BOOST_SERIALIZATION_NVP(_k6);
  }

  template <class Archive>
  void load(Archive& archive, const unsigned int) {
    archive >> BOOST_SERIALIZATION_NVP(_k1);
    archive >> BOOST_SERIALIZATION_NVP(_k2);
    archive >> BOOST_SERIALIZATION_NVP(_p1);
    archive >> BOOST_SERIALIZATION_NVP(_p2);
    archive >> BOOST_SERIALIZATION_NVP(_k3);
    archive >> BOOST_SERIALIZATION_NVP(_k4);
    archive >> BOOST_SERIALIZATION_NVP(_k5);
    archive >> BOOST_SERIALIZATION_NVP(_k6);
  }

 private:
  double _k1;
  double _k2;
  double _p1;
  double _p2;
  double _k3;
  double _k4;
  double _k5;
  double _k6;
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::RadialTangentialDistortion8);

#endif
