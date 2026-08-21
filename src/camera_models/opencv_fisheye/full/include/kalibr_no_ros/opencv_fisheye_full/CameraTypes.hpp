#ifndef KALIBR_NO_ROS_OPENCV_FISHEYE_FULL_CAMERA_TYPES_HPP
#define KALIBR_NO_ROS_OPENCV_FISHEYE_FULL_CAMERA_TYPES_HPP

#include <boost/serialization/export.hpp>

#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/GlobalShutter.hpp>
#include <aslam/cameras/NoMask.hpp>

#include "OpenCvFisheyeProjection.hpp"

namespace aslam {
namespace cameras {

using OpenCvFisheyeCameraGeometry =
    CameraGeometry<OpenCvFisheyeProjection, GlobalShutter, NoMask>;

}  // namespace cameras
}  // namespace aslam

BOOST_CLASS_EXPORT_KEY(aslam::cameras::OpenCvFisheyeCameraGeometry);

#endif
