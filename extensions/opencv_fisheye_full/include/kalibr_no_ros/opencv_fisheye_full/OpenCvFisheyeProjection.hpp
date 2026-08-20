#ifndef KALIBR_NO_ROS_OPENCV_FISHEYE_FULL_PROJECTION_HPP
#define KALIBR_NO_ROS_OPENCV_FISHEYE_FULL_PROJECTION_HPP

#include <cmath>
#include <cstdlib>
#include <vector>

#include <Eigen/Dense>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sm/PropertyTree.hpp>
#include <sm/assert_macros.hpp>
#include <sm/boost/serialization.hpp>
#include <sm/kinematics/Transformation.hpp>

#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/StaticAssert.hpp>
#include <kalibr_no_ros/opencv_fisheye/OpenCvFisheyeDistortion.hpp>

namespace aslam {
namespace cameras {

/// Full OpenCV fisheye projection, including the dimensionless skew/alpha.
///
/// OpenCV defines the final pixel mapping as
///   u = fu * (xd + alpha * yd) + cu
///   v = fv * yd + cv
/// where [xd, yd] is the four-coefficient fisheye-distorted normalized point.
/// Consequently K(0, 1) = fu * alpha when the skew is encoded in K.
///
/// Unlike PinholeProjection, alpha is a projection design variable.  The
/// projection parameter block is [fu, fv, cu, cv, alpha]; distortion remains
/// the independent OpenCV D block [k1, k2, k3, k4].
class OpenCvFisheyeProjection {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { KeypointDimension = 2 };
  enum { IntrinsicsDimension = 5 };
  enum { DesignVariableDimension = IntrinsicsDimension };

  typedef OpenCvFisheyeDistortion distortion_t;
  typedef Eigen::Matrix<double, KeypointDimension, 1> keypoint_t;
  typedef Eigen::Matrix<double, KeypointDimension, IntrinsicsDimension>
      jacobian_intrinsics_t;

  OpenCvFisheyeProjection() : _base(), _alpha(0.0) {}

  OpenCvFisheyeProjection(double fu, double fv, double cu, double cv,
                          double alpha, int ru, int rv,
                          const distortion_t& distortion)
      : _base(fu, fv, cu, cv, ru, rv, distortion), _alpha(alpha) {}

  OpenCvFisheyeProjection(double fu, double fv, double cu, double cv,
                          double alpha, int ru, int rv)
      : _base(fu, fv, cu, cv, ru, rv), _alpha(alpha) {}

  explicit OpenCvFisheyeProjection(const sm::PropertyTree& config)
      : _base(
            config.getDouble("fu"), config.getDouble("fv"),
            config.getDouble("cu"), config.getDouble("cv"),
            config.getInt("ru"), config.getInt("rv"),
            distortion_t(
                sm::PropertyTree(config, "distortion").getDouble("k1", 0.0),
                sm::PropertyTree(config, "distortion").getDouble("k2", 0.0),
                sm::PropertyTree(config, "distortion").getDouble("k3", 0.0),
                sm::PropertyTree(config, "distortion").getDouble("k4", 0.0))),
        _alpha(config.getDouble("alpha", 0.0)) {}

  template <typename DERIVED_P, typename DERIVED_K>
  bool euclideanToKeypoint(const Eigen::MatrixBase<DERIVED_P>& p,
                           const Eigen::MatrixBase<DERIVED_K>& output) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 2);
    Eigen::MatrixBase<DERIVED_K>& keypoint =
        const_cast<Eigen::MatrixBase<DERIVED_K>&>(output);
    keypoint.derived().resize(2);

    const double inverse_z = 1.0 / p[2];
    keypoint[0] = p[0] * inverse_z;
    keypoint[1] = p[1] * inverse_z;
    _base.distortion().distort(keypoint);
    applyIntrinsics(keypoint);
    return isValid(keypoint) && p[2] > 0.0;
  }

  template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool euclideanToKeypoint(const Eigen::MatrixBase<DERIVED_P>& p,
                           const Eigen::MatrixBase<DERIVED_K>& output,
                           const Eigen::MatrixBase<DERIVED_JP>& output_jp) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JP>, 2, 3);

    Eigen::MatrixBase<DERIVED_K>& keypoint =
        const_cast<Eigen::MatrixBase<DERIVED_K>&>(output);
    Eigen::MatrixBase<DERIVED_JP>& jacobian =
        const_cast<Eigen::MatrixBase<DERIVED_JP>&>(output_jp);
    keypoint.derived().resize(2);
    jacobian.derived().resize(2, 3);

    const double inverse_z = 1.0 / p[2];
    const double inverse_z_squared = inverse_z * inverse_z;
    keypoint[0] = p[0] * inverse_z;
    keypoint[1] = p[1] * inverse_z;

    Eigen::Matrix2d distortion_jacobian;
    _base.distortion().distort(keypoint, distortion_jacobian);
    Eigen::Matrix<double, 2, 3> normalization_jacobian;
    normalization_jacobian << inverse_z, 0.0,
        -p[0] * inverse_z_squared, 0.0, inverse_z,
        -p[1] * inverse_z_squared;
    jacobian = pixelFromDistortedJacobian() * distortion_jacobian *
               normalization_jacobian;

    applyIntrinsics(keypoint);
    return isValid(keypoint) && p[2] > 0.0;
  }

  template <typename DERIVED_P, typename DERIVED_K>
  bool homogeneousToKeypoint(const Eigen::MatrixBase<DERIVED_P>& p,
                             const Eigen::MatrixBase<DERIVED_K>& output) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);
    return p[3] < 0.0
               ? euclideanToKeypoint(-p.derived().template head<3>(), output)
               : euclideanToKeypoint(p.derived().template head<3>(), output);
  }

  template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool homogeneousToKeypoint(
      const Eigen::MatrixBase<DERIVED_P>& p,
      const Eigen::MatrixBase<DERIVED_K>& output,
      const Eigen::MatrixBase<DERIVED_JP>& output_jp) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JP>, 2, 4);
    Eigen::MatrixBase<DERIVED_JP>& jacobian =
        const_cast<Eigen::MatrixBase<DERIVED_JP>&>(output_jp);
    jacobian.derived().resize(2, 4);
    jacobian.setZero();
    if (p[3] < 0.0) {
      const bool valid = euclideanToKeypoint(
          -p.derived().template head<3>(), output,
          jacobian.derived().template topLeftCorner<2, 3>());
      jacobian.derived().template topLeftCorner<2, 3>() *= -1.0;
      return valid;
    }
    return euclideanToKeypoint(
        p.derived().template head<3>(), output,
        jacobian.derived().template topLeftCorner<2, 3>());
  }

  template <typename DERIVED_K, typename DERIVED_P>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K>& keypoint,
                           const Eigen::MatrixBase<DERIVED_P>& output) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    keypoint_t distorted;
    removeIntrinsics(keypoint, distorted);
    _base.distortion().undistort(distorted);

    const bool undistortion_valid = isValidUndistortedPoint(distorted);

    Eigen::MatrixBase<DERIVED_P>& point =
        const_cast<Eigen::MatrixBase<DERIVED_P>&>(output);
    point.derived().resize(3);
    point << distorted[0], distorted[1], 1.0;
    return isValid(keypoint) && undistortion_valid;
  }

  template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K>& keypoint,
                           const Eigen::MatrixBase<DERIVED_P>& output,
                           const Eigen::MatrixBase<DERIVED_JK>& output_jk) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JK>, 3, 2);

    keypoint_t distorted;
    removeIntrinsics(keypoint, distorted);
    Eigen::Matrix2d undistortion_jacobian;
    _base.distortion().undistort(distorted, undistortion_jacobian);

    const bool undistortion_valid = isValidUndistortedPoint(distorted);

    Eigen::MatrixBase<DERIVED_P>& point =
        const_cast<Eigen::MatrixBase<DERIVED_P>&>(output);
    point.derived().resize(3);
    point << distorted[0], distorted[1], 1.0;

    Eigen::MatrixBase<DERIVED_JK>& jacobian =
        const_cast<Eigen::MatrixBase<DERIVED_JK>&>(output_jk);
    jacobian.derived().resize(3, 2);
    jacobian.setZero();
    if (undistortion_valid && undistortion_jacobian.allFinite()) {
      jacobian.derived().template topRows<2>() =
          undistortion_jacobian * distortedFromPixelJacobian();
    }
    return isValid(keypoint) && undistortion_valid &&
           undistortion_jacobian.allFinite();
  }

  template <typename DERIVED_K, typename DERIVED_P>
  bool keypointToHomogeneous(const Eigen::MatrixBase<DERIVED_K>& keypoint,
                             const Eigen::MatrixBase<DERIVED_P>& output) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);
    Eigen::MatrixBase<DERIVED_P>& point =
        const_cast<Eigen::MatrixBase<DERIVED_P>&>(output);
    point.derived().resize(4);
    point[3] = 0.0;
    return keypointToEuclidean(keypoint,
                               point.derived().template head<3>());
  }

  template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToHomogeneous(const Eigen::MatrixBase<DERIVED_K>& keypoint,
                             const Eigen::MatrixBase<DERIVED_P>& output,
                             const Eigen::MatrixBase<DERIVED_JK>& output_jk) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JK>, 4, 2);
    Eigen::MatrixBase<DERIVED_P>& point =
        const_cast<Eigen::MatrixBase<DERIVED_P>&>(output);
    Eigen::MatrixBase<DERIVED_JK>& jacobian =
        const_cast<Eigen::MatrixBase<DERIVED_JK>&>(output_jk);
    point.derived().resize(4);
    point[3] = 0.0;
    jacobian.derived().resize(4, 2);
    jacobian.setZero();
    return keypointToEuclidean(
        keypoint, point.derived().template head<3>(),
        jacobian.derived().template topLeftCorner<3, 2>());
  }

  template <typename DERIVED_P, typename DERIVED_JI>
  void euclideanToKeypointIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_P>& p,
      const Eigen::MatrixBase<DERIVED_JI>& output_ji) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JI>, 2, 5);
    Eigen::MatrixBase<DERIVED_JI>& jacobian =
        const_cast<Eigen::MatrixBase<DERIVED_JI>&>(output_ji);
    jacobian.derived().resize(2, 5);
    jacobian.setZero();

    keypoint_t distorted(p[0] / p[2], p[1] / p[2]);
    _base.distortion().distort(distorted);
    jacobian(0, 0) = distorted[0] + _alpha * distorted[1];
    jacobian(0, 2) = 1.0;
    jacobian(0, 4) = fu() * distorted[1];
    jacobian(1, 1) = distorted[1];
    jacobian(1, 3) = 1.0;
  }

  template <typename DERIVED_P, typename DERIVED_JD>
  void euclideanToKeypointDistortionJacobian(
      const Eigen::MatrixBase<DERIVED_P>& p,
      const Eigen::MatrixBase<DERIVED_JD>& output_jd) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    keypoint_t normalized(p[0] / p[2], p[1] / p[2]);
    Eigen::MatrixBase<DERIVED_JD>& jacobian =
        const_cast<Eigen::MatrixBase<DERIVED_JD>&>(output_jd);
    _base.distortion().distortParameterJacobian(normalized, jacobian);
    jacobian = pixelFromDistortedJacobian() * jacobian.derived();
  }

  template <typename DERIVED_P, typename DERIVED_JI>
  void homogeneousToKeypointIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_P>& p,
      const Eigen::MatrixBase<DERIVED_JI>& output_ji) const {
    if (p[3] < 0.0) {
      euclideanToKeypointIntrinsicsJacobian(
          -p.derived().template head<3>(), output_ji);
    } else {
      euclideanToKeypointIntrinsicsJacobian(
          p.derived().template head<3>(), output_ji);
    }
  }

  template <typename DERIVED_P, typename DERIVED_JD>
  void homogeneousToKeypointDistortionJacobian(
      const Eigen::MatrixBase<DERIVED_P>& p,
      const Eigen::MatrixBase<DERIVED_JD>& output_jd) const {
    if (p[3] < 0.0) {
      euclideanToKeypointDistortionJacobian(
          -p.derived().template head<3>(), output_jd);
    } else {
      euclideanToKeypointDistortionJacobian(
          p.derived().template head<3>(), output_jd);
    }
  }

  template <typename DERIVED_K>
  bool isValid(const Eigen::MatrixBase<DERIVED_K>& keypoint) const {
    return keypoint[0] >= 0.0 && keypoint[1] >= 0.0 &&
           keypoint[0] < static_cast<double>(ru()) &&
           keypoint[1] < static_cast<double>(rv());
  }

  template <typename DERIVED_P>
  bool isEuclideanVisible(const Eigen::MatrixBase<DERIVED_P>& point) const {
    keypoint_t keypoint;
    return euclideanToKeypoint(point, keypoint);
  }

  template <typename DERIVED_P>
  bool isHomogeneousVisible(const Eigen::MatrixBase<DERIVED_P>& point) const {
    keypoint_t keypoint;
    return homogeneousToKeypoint(point, keypoint);
  }

  void update(const double* update_vector) {
    _base.update(update_vector);
    _alpha += update_vector[4];
  }

  int minimalDimensions() const { return IntrinsicsDimension; }

  void getParameters(Eigen::MatrixXd& parameters) const {
    parameters.resize(5, 1);
    parameters << fu(), fv(), cu(), cv(), _alpha;
  }

  void setParameters(const Eigen::MatrixXd& parameters) {
    SM_ASSERT_EQ(std::runtime_error, parameters.rows(), 5,
                 "OpenCV fisheye projection requires five intrinsics");
    SM_ASSERT_EQ(std::runtime_error, parameters.cols(), 1,
                 "OpenCV fisheye intrinsics must be a column vector");
    _base.setParameters(parameters.topRows(4));
    _alpha = parameters(4, 0);
  }

  Eigen::Vector2i parameterSize() const { return Eigen::Vector2i(5, 1); }

  enum { CLASS_SERIALIZATION_VERSION = 0 };
  BOOST_SERIALIZATION_SPLIT_MEMBER();

  template <class Archive>
  void save(Archive& archive, const unsigned int) const {
    archive << boost::serialization::make_nvp("base", _base);
    archive << BOOST_SERIALIZATION_NVP(_alpha);
  }

  template <class Archive>
  void load(Archive& archive, const unsigned int version) {
    SM_ASSERT_LE(std::runtime_error, version,
                 static_cast<unsigned int>(CLASS_SERIALIZATION_VERSION),
                 "Unsupported OpenCV fisheye projection serialization version");
    archive >> boost::serialization::make_nvp("base", _base);
    archive >> BOOST_SERIALIZATION_NVP(_alpha);
  }

  Eigen::VectorXd createRandomKeypoint() const {
    return _base.createRandomKeypoint();
  }

  Eigen::Vector3d createRandomVisiblePoint(double depth = -1.0) const {
    Eigen::VectorXd keypoint = createRandomKeypoint();
    Eigen::Vector3d point;
    keypointToEuclidean(keypoint, point);
    if (depth < 0.0) {
      depth = (static_cast<double>(std::rand()) /
               static_cast<double>(RAND_MAX)) *
              100.0;
    }
    return point.normalized() * depth;
  }

  bool isProjectionInvertible() const { return false; }

  void setDistortion(const distortion_t& distortion) {
    _base.setDistortion(distortion);
  }
  distortion_t& distortion() { return _base.distortion(); }
  const distortion_t& distortion() const { return _base.distortion(); }

  Eigen::Matrix3d getCameraMatrix() const {
    Eigen::Matrix3d matrix;
    matrix << fu(), skew(), cu(), 0.0, fv(), cv(), 0.0, 0.0, 1.0;
    return matrix;
  }

  double focalLengthCol() const { return fu(); }
  double focalLengthRow() const { return fv(); }
  double opticalCenterCol() const { return cu(); }
  double opticalCenterRow() const { return cv(); }
  double fu() const { return _base.fu(); }
  double fv() const { return _base.fv(); }
  double cu() const { return _base.cu(); }
  double cv() const { return _base.cv(); }
  double alpha() const { return _alpha; }
  double skew() const { return fu() * _alpha; }
  int ru() const { return _base.ru(); }
  int rv() const { return _base.rv(); }
  int width() const { return ru(); }
  int height() const { return rv(); }
  int keypointDimension() const { return KeypointDimension; }

  bool isBinaryEqual(const OpenCvFisheyeProjection& rhs) const {
    return _base.isBinaryEqual(rhs._base) && _alpha == rhs._alpha;
  }

  static OpenCvFisheyeProjection getTestProjection() {
    return OpenCvFisheyeProjection(
        400.0, 410.0, 320.0, 240.0, 0.013, 640, 480,
        distortion_t::getTestDistortion());
  }

  void resizeIntrinsics(double scale) { _base.resizeIntrinsics(scale); }

  void getBorderRays(Eigen::MatrixXd& rays) {
    rays.resize(4, 8);
    keypointToHomogeneous(Eigen::Vector2d(0.0, 0.0), rays.col(0));
    keypointToHomogeneous(Eigen::Vector2d(0.0, rv() * 0.5), rays.col(1));
    keypointToHomogeneous(Eigen::Vector2d(0.0, rv() - 1.0), rays.col(2));
    keypointToHomogeneous(Eigen::Vector2d(ru() - 1.0, 0.0), rays.col(3));
    keypointToHomogeneous(Eigen::Vector2d(ru() - 1.0, rv() * 0.5),
                          rays.col(4));
    keypointToHomogeneous(Eigen::Vector2d(ru() - 1.0, rv() - 1.0),
                          rays.col(5));
    keypointToHomogeneous(Eigen::Vector2d(ru() * 0.5, 0.0), rays.col(6));
    keypointToHomogeneous(Eigen::Vector2d(ru() * 0.5, rv() - 1.0),
                          rays.col(7));
  }

  bool initializeIntrinsics(
      const std::vector<GridCalibrationTargetObservation>& observations) {
    _alpha = 0.0;
    return _base.initializeIntrinsics(observations);
  }

  size_t computeReprojectionError(
      const GridCalibrationTargetObservation& observation,
      const sm::kinematics::Transformation& target_from_camera,
      double& error) const {
    error = 0.0;
    size_t count = 0;
    const sm::kinematics::Transformation camera_from_target =
        target_from_camera.inverse();
    for (size_t index = 0; index < observation.target()->size(); ++index) {
      Eigen::Vector2d measured;
      Eigen::Vector2d predicted;
      if (observation.imagePoint(index, measured) &&
          euclideanToKeypoint(
              camera_from_target * observation.target()->point(index),
              predicted)) {
        error += (measured - predicted).norm();
        ++count;
      }
    }
    return count;
  }

  bool estimateTransformation(
      const GridCalibrationTargetObservation& observation,
      sm::kinematics::Transformation& target_from_camera) const {
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> target_points;
    observation.getCornersImageFrame(image_points);
    observation.getCornersTargetFrame(target_points);

    size_t accepted = 0;
    for (size_t index = 0; index < image_points.size(); ++index) {
      const Eigen::Vector2d image_point(image_points[index].x,
                                        image_points[index].y);
      Eigen::Vector3d ray;
      if (keypointToEuclidean(image_point, ray) &&
          ray.normalized()[2] > std::cos(80.0 * M_PI / 180.0)) {
        target_points[accepted] = target_points[index];
        image_points[accepted].x = static_cast<float>(ray[0] / ray[2]);
        image_points[accepted].y = static_cast<float>(ray[1] / ray[2]);
        ++accepted;
      }
    }
    target_points.resize(accepted);
    image_points.resize(accepted);
    if (accepted < 4) {
      return false;
    }

    cv::Mat rotation_vector(3, 1, CV_64F);
    cv::Mat translation_vector(3, 1, CV_64F);
    std::vector<double> no_distortion(4, 0.0);
    if (!cv::solvePnP(target_points, image_points,
                      cv::Mat::eye(3, 3, CV_64F), no_distortion,
                      rotation_vector, translation_vector)) {
      return false;
    }

    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Rodrigues(rotation_vector, rotation);
    Eigen::Matrix4d camera_from_target = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
      camera_from_target(row, 3) = translation_vector.at<double>(row, 0);
      for (int column = 0; column < 3; ++column) {
        camera_from_target(row, column) = rotation.at<double>(row, column);
      }
    }
    target_from_camera.set(camera_from_target.inverse());
    return true;
  }

 private:
  template <typename DERIVED_K>
  void applyIntrinsics(const Eigen::MatrixBase<DERIVED_K>& output) const {
    Eigen::MatrixBase<DERIVED_K>& keypoint =
        const_cast<Eigen::MatrixBase<DERIVED_K>&>(output);
    const double distorted_y = keypoint[1];
    keypoint[0] = fu() * (keypoint[0] + _alpha * distorted_y) + cu();
    keypoint[1] = fv() * distorted_y + cv();
  }

  template <typename DERIVED_K, typename DERIVED_D>
  void removeIntrinsics(const Eigen::MatrixBase<DERIVED_K>& keypoint,
                        const Eigen::MatrixBase<DERIVED_D>& output) const {
    Eigen::MatrixBase<DERIVED_D>& distorted =
        const_cast<Eigen::MatrixBase<DERIVED_D>&>(output);
    distorted.derived().resize(2);
    distorted[1] = (keypoint[1] - cv()) / fv();
    distorted[0] = (keypoint[0] - cu()) / fu() - _alpha * distorted[1];
  }

  Eigen::Matrix2d pixelFromDistortedJacobian() const {
    Eigen::Matrix2d jacobian;
    jacobian << fu(), fu() * _alpha, 0.0, fv();
    return jacobian;
  }

  Eigen::Matrix2d distortedFromPixelJacobian() const {
    Eigen::Matrix2d jacobian;
    jacobian << 1.0 / fu(), -_alpha / fv(), 0.0, 1.0 / fv();
    return jacobian;
  }

  static bool isValidUndistortedPoint(const Eigen::Vector2d& point) {
    if (!point.allFinite()) {
      return false;
    }
    // cv::fisheye::undistortPoints uses (-1e6,-1e6) to signal a Newton
    // failure or a theta sign flip.  Propagate that failure through Kalibr's
    // bool-returning back-projection API instead of treating it as a ray.
    const double sentinel = -1000000.0;
    return !(point[0] == sentinel && point[1] == sentinel);
  }

  PinholeProjection<distortion_t> _base;
  double _alpha;
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::OpenCvFisheyeProjection);

#endif
