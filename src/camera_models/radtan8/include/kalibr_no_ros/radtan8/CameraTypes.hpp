#ifndef KALIBR_NO_ROS_RADTAN8_CAMERA_TYPES_HPP
#define KALIBR_NO_ROS_RADTAN8_CAMERA_TYPES_HPP

#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/GlobalShutter.hpp>
#include <aslam/cameras/NoMask.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <boost/serialization/export.hpp>

#include "RadialTangentialDistortion8.hpp"

namespace aslam {
namespace cameras {

using Radtan8PinholeProjection =
    PinholeProjection<RadialTangentialDistortion8>;
using Radtan8PinholeCameraGeometry =
    CameraGeometry<Radtan8PinholeProjection, GlobalShutter, NoMask>;

}  // namespace cameras
}  // namespace aslam

BOOST_CLASS_EXPORT_KEY(aslam::cameras::Radtan8PinholeCameraGeometry);

#endif
