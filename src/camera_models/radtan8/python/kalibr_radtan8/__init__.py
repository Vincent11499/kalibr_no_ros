"""OpenCV-compatible eight-parameter rational pinhole model for Kalibr."""

import aslam_cv

from .libkalibr_radtan8_cv_python import *

import aslam_backend  # noqa: F401 - registers backend base classes
import aslam_cv_backend

from .libkalibr_radtan8_backend_python import *


class PinholeRadtan8(object):
    geometry = Radtan8PinholeCameraGeometry
    reprojectionError = Radtan8PinholeReprojectionError
    reprojectionErrorSimple = Radtan8PinholeReprojectionErrorSimple
    designVariable = Radtan8PinholeCameraGeometryDesignVariable
    projectionType = Radtan8PinholeProjection
    distortionType = RadialTangentialDistortion8
    shutterType = aslam_cv.GlobalShutter
    frameType = Radtan8PinholeFrame


from .integration import install  # noqa: E402,F401


__all__ = [
    "PinholeRadtan8",
    "RadialTangentialDistortion8",
    "Radtan8PinholeProjection",
    "Radtan8PinholeCameraGeometry",
    "Radtan8PinholeFrame",
    "install",
]
