#include <numpy_eigen/boost_python_headers.hpp>

#include <kalibr_no_ros/radtan8/CameraTypes.hpp>

#include <aslam/ExportCameraDesignVariable.hpp>
#include <aslam/ExportReprojectionError.hpp>

BOOST_PYTHON_MODULE(libkalibr_radtan8_backend_python) {
  using aslam::cameras::RadialTangentialDistortion8;
  using aslam::cameras::Radtan8PinholeCameraGeometry;
  using aslam::cameras::Radtan8PinholeProjection;
  aslam::python::exportGenericProjectionDesignVariable<
      RadialTangentialDistortion8>("RadialTangentialDistortion8");
  aslam::python::exportGenericProjectionDesignVariable<
      Radtan8PinholeProjection>("Radtan8PinholeProjection");
  aslam::python::exportReprojectionErrors<Radtan8PinholeCameraGeometry>(
      "Radtan8Pinhole");
  aslam::python::exportCameraDesignVariables<Radtan8PinholeCameraGeometry>(
      "Radtan8PinholeCameraGeometry");
}
