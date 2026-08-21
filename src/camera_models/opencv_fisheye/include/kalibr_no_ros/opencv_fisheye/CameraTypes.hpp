#ifndef KALIBR_NO_ROS_OPENCV_FISHEYE_CAMERA_TYPES_HPP
#define KALIBR_NO_ROS_OPENCV_FISHEYE_CAMERA_TYPES_HPP

#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/GlobalShutter.hpp>
#include <aslam/cameras/NoMask.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <boost/serialization/export.hpp>

#include "OpenCvFisheyeDistortion.hpp"

namespace aslam {
namespace cameras {

using OpenCvFisheyePinholeProjection =
    PinholeProjection<OpenCvFisheyeDistortion>;
using OpenCvFisheyePinholeCameraGeometry =
    CameraGeometry<OpenCvFisheyePinholeProjection, GlobalShutter, NoMask>;

}  // namespace cameras
}  // namespace aslam

BOOST_CLASS_EXPORT_KEY(aslam::cameras::OpenCvFisheyePinholeCameraGeometry);

#endif
