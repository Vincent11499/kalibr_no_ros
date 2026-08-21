#include <numpy_eigen/boost_python_headers.hpp>

#include <kalibr_no_ros/radtan5/CameraTypes.hpp>

#include <aslam/ExportCameraDesignVariable.hpp>
#include <aslam/ExportReprojectionError.hpp>

BOOST_PYTHON_MODULE(libkalibr_radtan5_backend_python) {
  using aslam::cameras::RadialTangentialDistortion5;
  using aslam::cameras::Radtan5PinholeCameraGeometry;
  using aslam::cameras::Radtan5PinholeProjection;
  aslam::python::exportGenericProjectionDesignVariable<
      RadialTangentialDistortion5>("RadialTangentialDistortion5");
  aslam::python::exportGenericProjectionDesignVariable<
      Radtan5PinholeProjection>("Radtan5PinholeProjection");
  aslam::python::exportReprojectionErrors<Radtan5PinholeCameraGeometry>(
      "Radtan5Pinhole");
  aslam::python::exportCameraDesignVariables<Radtan5PinholeCameraGeometry>(
      "Radtan5PinholeCameraGeometry");
}
