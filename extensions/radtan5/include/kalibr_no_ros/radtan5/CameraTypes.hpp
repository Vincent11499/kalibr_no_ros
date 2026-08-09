#ifndef KALIBR_NO_ROS_RADTAN5_CAMERA_TYPES_HPP
#define KALIBR_NO_ROS_RADTAN5_CAMERA_TYPES_HPP

#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/GlobalShutter.hpp>
#include <aslam/cameras/NoMask.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <boost/serialization/export.hpp>

#include "RadialTangentialDistortion5.hpp"

namespace aslam {
namespace cameras {

using Radtan5PinholeProjection =
    PinholeProjection<RadialTangentialDistortion5>;
using Radtan5PinholeCameraGeometry =
    CameraGeometry<Radtan5PinholeProjection, GlobalShutter, NoMask>;

}  // namespace cameras
}  // namespace aslam

BOOST_CLASS_EXPORT_KEY(aslam::cameras::Radtan5PinholeCameraGeometry);

#endif
