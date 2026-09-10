#include <numpy_eigen/boost_python_headers.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/python/ExportFrame.hpp>
#include <sm/python/boost_serialization_pickle.hpp>
#include <sm/python/unique_register_ptr_to_python.hpp>

#include <kalibr_no_ros/opencv_fisheye/CameraTypes.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::OpenCvFisheyeCameraGeometry);

namespace bp = boost::python;
using aslam::cameras::CameraGeometryBase;
using aslam::cameras::OpenCvFisheyeCameraGeometry;
using aslam::cameras::OpenCvFisheyeDistortion;
using aslam::cameras::OpenCvFisheyeProjection;

namespace {

template <typename T>
Eigen::MatrixXd getParameters(T* object) {
  Eigen::MatrixXd parameters;
  object->getParameters(parameters);
  return parameters;
}

Eigen::Vector2d distort(OpenCvFisheyeDistortion* distortion,
                        Eigen::Vector2d point) {
  distortion->distort(point);
  return point;
}

bp::tuple distortWithInputJacobian(OpenCvFisheyeDistortion* distortion,
                                   Eigen::Vector2d point) {
  Eigen::Matrix2d jacobian;
  distortion->distort(point, jacobian);
  return bp::make_tuple(point, jacobian);
}

Eigen::Vector2d undistort(OpenCvFisheyeDistortion* distortion,
                         Eigen::Vector2d point) {
  distortion->undistort(point);
  return point;
}

bp::tuple undistortWithInputJacobian(
    OpenCvFisheyeDistortion* distortion, Eigen::Vector2d point) {
  Eigen::Matrix2d jacobian;
  distortion->undistort(point, jacobian);
  return bp::make_tuple(point, jacobian);
}

Eigen::MatrixXd distortionCoefficientJacobian(
    OpenCvFisheyeDistortion* distortion, Eigen::Vector2d point) {
  Eigen::MatrixXd jacobian;
  distortion->distortParameterJacobian(point, jacobian);
  return jacobian;
}

Eigen::VectorXd project(OpenCvFisheyeProjection* projection,
                        const Eigen::Vector3d& point) {
  Eigen::VectorXd keypoint;
  projection->euclideanToKeypoint(point, keypoint);
  return keypoint;
}

bp::tuple projectWithJacobian(OpenCvFisheyeProjection* projection,
                              const Eigen::Vector3d& point) {
  Eigen::VectorXd keypoint;
  Eigen::MatrixXd jacobian;
  const bool valid =
      projection->euclideanToKeypoint(point, keypoint, jacobian);
  return bp::make_tuple(keypoint, jacobian, valid);
}

Eigen::Vector3d backProject(OpenCvFisheyeProjection* projection,
                            const Eigen::VectorXd& keypoint) {
  Eigen::Vector3d point;
  projection->keypointToEuclidean(keypoint, point);
  return point;
}

bp::tuple backProjectWithJacobian(OpenCvFisheyeProjection* projection,
                                  const Eigen::VectorXd& keypoint) {
  Eigen::Vector3d point;
  Eigen::MatrixXd jacobian;
  const bool valid =
      projection->keypointToEuclidean(keypoint, point, jacobian);
  return bp::make_tuple(point, jacobian, valid);
}

Eigen::MatrixXd projectionParameterJacobian(
    OpenCvFisheyeProjection* projection, const Eigen::Vector3d& point) {
  Eigen::MatrixXd jacobian;
  projection->euclideanToKeypointIntrinsicsJacobian(point, jacobian);
  return jacobian;
}

Eigen::MatrixXd distortionParameterJacobian(
    OpenCvFisheyeProjection* projection, const Eigen::Vector3d& point) {
  Eigen::MatrixXd jacobian;
  projection->euclideanToKeypointDistortionJacobian(point, jacobian);
  return jacobian;
}

void exportDistortion() {
  bp::class_<OpenCvFisheyeDistortion,
             boost::shared_ptr<OpenCvFisheyeDistortion> > cls(
      "OpenCvFisheyeDistortion", bp::init<>());
  sm::python::unique_register_ptr_to_python<
      boost::shared_ptr<OpenCvFisheyeDistortion> >();
  cls.def(bp::init<double, double, double, double>())
      .def("distort", &distort)
      .def("distortWithInputJacobian", &distortWithInputJacobian)
      .def("undistort", &undistort)
      .def("undistortWithInputJacobian", &undistortWithInputJacobian)
      .def("distortParameterJacobian", &distortionCoefficientJacobian)
      .def("getParameters", &getParameters<OpenCvFisheyeDistortion>)
      .def("setParameters", &OpenCvFisheyeDistortion::setParameters)
      .def("minimalDimensions", &OpenCvFisheyeDistortion::minimalDimensions)
      .def("k1", &OpenCvFisheyeDistortion::k1)
      .def("k2", &OpenCvFisheyeDistortion::k2)
      .def("k3", &OpenCvFisheyeDistortion::k3)
      .def("k4", &OpenCvFisheyeDistortion::k4);
}

void exportProjection() {
  OpenCvFisheyeProjection::distortion_t&
      (OpenCvFisheyeProjection::*distortion)() =
          &OpenCvFisheyeProjection::distortion;
  bp::class_<OpenCvFisheyeProjection,
             boost::shared_ptr<OpenCvFisheyeProjection> > cls(
      "OpenCvFisheyeProjection", bp::init<>());
  sm::python::unique_register_ptr_to_python<
      boost::shared_ptr<OpenCvFisheyeProjection> >();
  cls.def(bp::init<double, double, double, double, double, int, int,
                   OpenCvFisheyeDistortion>())
      .def(bp::init<double, double, double, double, double, int, int>())
      .def("fu", &OpenCvFisheyeProjection::fu)
      .def("fv", &OpenCvFisheyeProjection::fv)
      .def("cu", &OpenCvFisheyeProjection::cu)
      .def("cv", &OpenCvFisheyeProjection::cv)
      .def("alpha", &OpenCvFisheyeProjection::alpha)
      .def("skew", &OpenCvFisheyeProjection::skew)
      .def("ru", &OpenCvFisheyeProjection::ru)
      .def("rv", &OpenCvFisheyeProjection::rv)
      .def("getCameraMatrix", &OpenCvFisheyeProjection::getCameraMatrix)
      .def("distortion", distortion, bp::return_internal_reference<>())
      .def("setDistortion", &OpenCvFisheyeProjection::setDistortion)
      .def("getParameters", &getParameters<OpenCvFisheyeProjection>)
      .def("setParameters", &OpenCvFisheyeProjection::setParameters)
      .def("minimalDimensions", &OpenCvFisheyeProjection::minimalDimensions)
      .def("euclideanToKeypoint", &project)
      .def("euclideanToKeypointJp", &projectWithJacobian)
      .def("keypointToEuclidean", &backProject)
      .def("keypointToEuclideanJk", &backProjectWithJacobian)
      .def("projectionParameterJacobian", &projectionParameterJacobian)
      .def("distortionParameterJacobian", &distortionParameterJacobian);
}

void exportGeometry() {
  using Geometry = OpenCvFisheyeCameraGeometry;
  Geometry::projection_t& (Geometry::*projection)() = &Geometry::projection;
  Geometry::shutter_t& (Geometry::*shutter)() = &Geometry::shutter;
  Geometry::mask_t& (Geometry::*mask)() = &Geometry::mask;
  bp::class_<Geometry, boost::shared_ptr<Geometry>,
             bp::bases<CameraGeometryBase> >(
      "OpenCvFisheyeCameraGeometry", bp::init<>())
      .def(bp::init<Geometry::projection_t>())
      .def(bp::init<Geometry::projection_t, Geometry::shutter_t>())
      .def("projection", projection, bp::return_internal_reference<>())
      .def("shutter", shutter, bp::return_internal_reference<>())
      .def("mask", mask, bp::return_internal_reference<>())
      .def_pickle(sm::python::pickle_suite<Geometry>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<Geometry> >();
  aslam::python::exportFrame<Geometry>("OpenCvFisheyeFrame");
}

}  // namespace

BOOST_PYTHON_MODULE(libkalibr_opencv_fisheye_cv_python) {
  exportDistortion();
  exportProjection();
  exportGeometry();
}
