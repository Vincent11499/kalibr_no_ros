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

#include <kalibr_no_ros/radtan5/CameraTypes.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::Radtan5PinholeCameraGeometry);

namespace bp = boost::python;
using aslam::cameras::CameraGeometryBase;
using aslam::cameras::RadialTangentialDistortion5;
using aslam::cameras::Radtan5PinholeCameraGeometry;
using aslam::cameras::Radtan5PinholeProjection;

namespace {

template <typename T>
Eigen::MatrixXd getParameters(T* object) {
  Eigen::MatrixXd parameters;
  object->getParameters(parameters);
  return parameters;
}

Eigen::Vector2d distort(RadialTangentialDistortion5* distortion,
                        Eigen::Vector2d point) {
  distortion->distort(point);
  return point;
}

bp::tuple distortWithInputJacobian(RadialTangentialDistortion5* distortion,
                                   Eigen::Vector2d point) {
  Eigen::Matrix2d jacobian;
  distortion->distort(point, jacobian);
  return bp::make_tuple(point, jacobian);
}

Eigen::Vector2d undistort(RadialTangentialDistortion5* distortion,
                          Eigen::Vector2d point) {
  distortion->undistort(point);
  return point;
}

Eigen::MatrixXd distortionParameterJacobian(
    RadialTangentialDistortion5* distortion, Eigen::Vector2d point) {
  Eigen::MatrixXd jacobian;
  distortion->distortParameterJacobian(point, jacobian);
  return jacobian;
}

Eigen::VectorXd project(Radtan5PinholeProjection* projection,
                        const Eigen::Vector3d& point) {
  Eigen::VectorXd keypoint;
  projection->euclideanToKeypoint(point, keypoint);
  return keypoint;
}

bp::tuple projectWithJacobian(Radtan5PinholeProjection* projection,
                              const Eigen::Vector3d& point) {
  Eigen::VectorXd keypoint;
  Eigen::MatrixXd jacobian;
  const bool valid = projection->euclideanToKeypoint(point, keypoint, jacobian);
  return bp::make_tuple(keypoint, jacobian, valid);
}

Eigen::Vector3d backProject(Radtan5PinholeProjection* projection,
                            const Eigen::VectorXd& keypoint) {
  Eigen::Vector3d point;
  projection->keypointToEuclidean(keypoint, point);
  return point;
}

void exportDistortion() {
  bp::class_<RadialTangentialDistortion5,
             boost::shared_ptr<RadialTangentialDistortion5> > cls(
      "RadialTangentialDistortion5", bp::init<>());
  sm::python::unique_register_ptr_to_python<
      boost::shared_ptr<RadialTangentialDistortion5> >();
  cls.def(bp::init<double, double, double, double, double>())
      .def("distort", &distort)
      .def("distortWithInputJacobian", &distortWithInputJacobian)
      .def("undistort", &undistort)
      .def("distortParameterJacobian", &distortionParameterJacobian)
      .def("getParameters", &getParameters<RadialTangentialDistortion5>)
      .def("setParameters", &RadialTangentialDistortion5::setParameters)
      .def("minimalDimensions", &RadialTangentialDistortion5::minimalDimensions)
      .def("k1", &RadialTangentialDistortion5::k1)
      .def("k2", &RadialTangentialDistortion5::k2)
      .def("p1", &RadialTangentialDistortion5::p1)
      .def("p2", &RadialTangentialDistortion5::p2)
      .def("k3", &RadialTangentialDistortion5::k3);
}

void exportProjection() {
  Radtan5PinholeProjection::distortion_t&
      (Radtan5PinholeProjection::*distortion)() =
          &Radtan5PinholeProjection::distortion;
  bp::class_<Radtan5PinholeProjection,
             boost::shared_ptr<Radtan5PinholeProjection> > cls(
      "Radtan5PinholeProjection", bp::init<>());
  sm::python::unique_register_ptr_to_python<
      boost::shared_ptr<Radtan5PinholeProjection> >();
  cls.def(bp::init<double, double, double, double, int, int,
                   RadialTangentialDistortion5>())
      .def(bp::init<double, double, double, double, int, int>())
      .def("fu", &Radtan5PinholeProjection::fu)
      .def("fv", &Radtan5PinholeProjection::fv)
      .def("cu", &Radtan5PinholeProjection::cu)
      .def("cv", &Radtan5PinholeProjection::cv)
      .def("ru", &Radtan5PinholeProjection::ru)
      .def("rv", &Radtan5PinholeProjection::rv)
      .def("distortion", distortion, bp::return_internal_reference<>())
      .def("setDistortion", &Radtan5PinholeProjection::setDistortion)
      .def("getParameters", &getParameters<Radtan5PinholeProjection>)
      .def("setParameters", &Radtan5PinholeProjection::setParameters)
      .def("euclideanToKeypoint", &project)
      .def("euclideanToKeypointJp", &projectWithJacobian)
      .def("keypointToEuclidean", &backProject);
}

void exportGeometry() {
  using Geometry = Radtan5PinholeCameraGeometry;
  Geometry::projection_t& (Geometry::*projection)() = &Geometry::projection;
  Geometry::shutter_t& (Geometry::*shutter)() = &Geometry::shutter;
  Geometry::mask_t& (Geometry::*mask)() = &Geometry::mask;
  bp::class_<Geometry, boost::shared_ptr<Geometry>, bp::bases<CameraGeometryBase> >(
      "Radtan5PinholeCameraGeometry", bp::init<>())
      .def(bp::init<Geometry::projection_t>())
      .def(bp::init<Geometry::projection_t, Geometry::shutter_t>())
      .def("projection", projection, bp::return_internal_reference<>())
      .def("shutter", shutter, bp::return_internal_reference<>())
      .def("mask", mask, bp::return_internal_reference<>())
      .def_pickle(sm::python::pickle_suite<Geometry>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<Geometry> >();
  aslam::python::exportFrame<Geometry>("Radtan5PinholeFrame");
}

}  // namespace

BOOST_PYTHON_MODULE(libkalibr_radtan5_cv_python) {
  exportDistortion();
  exportProjection();
  exportGeometry();
}
