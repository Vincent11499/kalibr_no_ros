#include <numpy_eigen/boost_python_headers.hpp>

#include <kalibr_no_ros/opencv_fisheye/CameraTypes.hpp>

#include <aslam/ExportCameraDesignVariable.hpp>
#include <aslam/ExportReprojectionError.hpp>

BOOST_PYTHON_MODULE(libkalibr_opencv_fisheye_backend_python) {
  using aslam::cameras::OpenCvFisheyeDistortion;
  using aslam::cameras::OpenCvFisheyeCameraGeometry;
  using aslam::cameras::OpenCvFisheyeProjection;
  aslam::python::exportGenericProjectionDesignVariable<
      OpenCvFisheyeDistortion>("OpenCvFisheyeDistortion");
  aslam::python::exportGenericProjectionDesignVariable<
      OpenCvFisheyeProjection>("OpenCvFisheyeProjection");
  aslam::python::exportReprojectionErrors<OpenCvFisheyeCameraGeometry>(
      "OpenCvFisheye");
  aslam::python::exportCameraDesignVariables<OpenCvFisheyeCameraGeometry>(
      "OpenCvFisheyeCameraGeometry");
}
