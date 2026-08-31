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

#include <kalibr_no_ros/radtan8/CameraTypes.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::Radtan8PinholeCameraGeometry);

namespace bp = boost::python;
using aslam::cameras::CameraGeometryBase;
using aslam::cameras::RadialTangentialDistortion8;
using aslam::cameras::Radtan8PinholeCameraGeometry;
using aslam::cameras::Radtan8PinholeProjection;

namespace {

template <typename T>
Eigen::MatrixXd getParameters(T* object) {
  Eigen::MatrixXd parameters;
  object->getParameters(parameters);
  return parameters;
}

Eigen::Vector2d distort(RadialTangentialDistortion8* distortion,
                        Eigen::Vector2d point) {
  distortion->distort(point);
  return point;
}

bp::tuple distortWithInputJacobian(RadialTangentialDistortion8* distortion,
                                   Eigen::Vector2d point) {
  Eigen::Matrix2d jacobian;
  distortion->distort(point, jacobian);
  return bp::make_tuple(point, jacobian);
}

Eigen::Vector2d undistort(RadialTangentialDistortion8* distortion,
                          Eigen::Vector2d point) {
  distortion->undistort(point);
  return point;
}

Eigen::MatrixXd distortionParameterJacobian(
    RadialTangentialDistortion8* distortion, Eigen::Vector2d point) {
  Eigen::MatrixXd jacobian;
  distortion->distortParameterJacobian(point, jacobian);
  return jacobian;
}

Eigen::VectorXd project(Radtan8PinholeProjection* projection,
                        const Eigen::Vector3d& point) {
  Eigen::VectorXd keypoint;
  projection->euclideanToKeypoint(point, keypoint);
  return keypoint;
}

bp::tuple projectWithJacobian(Radtan8PinholeProjection* projection,
                              const Eigen::Vector3d& point) {
  Eigen::VectorXd keypoint;
  Eigen::MatrixXd jacobian;
  const bool valid = projection->euclideanToKeypoint(point, keypoint, jacobian);
  return bp::make_tuple(keypoint, jacobian, valid);
}

Eigen::Vector3d backProject(Radtan8PinholeProjection* projection,
                            const Eigen::VectorXd& keypoint) {
  Eigen::Vector3d point;
  projection->keypointToEuclidean(keypoint, point);
  return point;
}

void exportDistortion() {
  bp::class_<RadialTangentialDistortion8,
             boost::shared_ptr<RadialTangentialDistortion8> > cls(
      "RadialTangentialDistortion8", bp::init<>());
  sm::python::unique_register_ptr_to_python<
      boost::shared_ptr<RadialTangentialDistortion8> >();
  cls.def(bp::init<double, double, double, double, double, double, double,
                   double>())
      .def("distort", &distort)
      .def("distortWithInputJacobian", &distortWithInputJacobian)
      .def("undistort", &undistort)
      .def("distortParameterJacobian", &distortionParameterJacobian)
      .def("getParameters", &getParameters<RadialTangentialDistortion8>)
      .def("setParameters", &RadialTangentialDistortion8::setParameters)
      .def("minimalDimensions", &RadialTangentialDistortion8::minimalDimensions)
      .def("k1", &RadialTangentialDistortion8::k1)
      .def("k2", &RadialTangentialDistortion8::k2)
      .def("p1", &RadialTangentialDistortion8::p1)
      .def("p2", &RadialTangentialDistortion8::p2)
      .def("k3", &RadialTangentialDistortion8::k3)
      .def("k4", &RadialTangentialDistortion8::k4)
      .def("k5", &RadialTangentialDistortion8::k5)
      .def("k6", &RadialTangentialDistortion8::k6);
}

void exportProjection() {
  Radtan8PinholeProjection::distortion_t&
      (Radtan8PinholeProjection::*distortion)() =
          &Radtan8PinholeProjection::distortion;
  bp::class_<Radtan8PinholeProjection,
             boost::shared_ptr<Radtan8PinholeProjection> > cls(
      "Radtan8PinholeProjection", bp::init<>());
  sm::python::unique_register_ptr_to_python<
      boost::shared_ptr<Radtan8PinholeProjection> >();
  cls.def(bp::init<double, double, double, double, int, int,
                   RadialTangentialDistortion8>())
      .def(bp::init<double, double, double, double, int, int>())
      .def("fu", &Radtan8PinholeProjection::fu)
      .def("fv", &Radtan8PinholeProjection::fv)
      .def("cu", &Radtan8PinholeProjection::cu)
      .def("cv", &Radtan8PinholeProjection::cv)
      .def("ru", &Radtan8PinholeProjection::ru)
      .def("rv", &Radtan8PinholeProjection::rv)
      .def("distortion", distortion, bp::return_internal_reference<>())
      .def("setDistortion", &Radtan8PinholeProjection::setDistortion)
      .def("getParameters", &getParameters<Radtan8PinholeProjection>)
      .def("setParameters", &Radtan8PinholeProjection::setParameters)
      .def("euclideanToKeypoint", &project)
      .def("euclideanToKeypointJp", &projectWithJacobian)
      .def("keypointToEuclidean", &backProject);
}

void exportGeometry() {
  using Geometry = Radtan8PinholeCameraGeometry;
  Geometry::projection_t& (Geometry::*projection)() = &Geometry::projection;
  Geometry::shutter_t& (Geometry::*shutter)() = &Geometry::shutter;
  Geometry::mask_t& (Geometry::*mask)() = &Geometry::mask;
  bp::class_<Geometry, boost::shared_ptr<Geometry>, bp::bases<CameraGeometryBase> >(
      "Radtan8PinholeCameraGeometry", bp::init<>())
      .def(bp::init<Geometry::projection_t>())
      .def(bp::init<Geometry::projection_t, Geometry::shutter_t>())
      .def("projection", projection, bp::return_internal_reference<>())
      .def("shutter", shutter, bp::return_internal_reference<>())
      .def("mask", mask, bp::return_internal_reference<>())
      .def_pickle(sm::python::pickle_suite<Geometry>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<Geometry> >();
  aslam::python::exportFrame<Geometry>("Radtan8PinholeFrame");
}

}  // namespace

BOOST_PYTHON_MODULE(libkalibr_radtan8_cv_python) {
  exportDistortion();
  exportProjection();
  exportGeometry();
}
