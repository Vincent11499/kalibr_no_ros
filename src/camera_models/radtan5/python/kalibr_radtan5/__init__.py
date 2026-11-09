"""OpenCV-compatible five-parameter pinhole model for Kalibr."""

import aslam_cv

from .libkalibr_radtan5_cv_python import *

import aslam_backend  # noqa: F401 - registers backend base classes
import aslam_cv_backend

from .libkalibr_radtan5_backend_python import *


class PinholeRadtan5(object):
    geometry = Radtan5PinholeCameraGeometry
    reprojectionError = Radtan5PinholeReprojectionError
    reprojectionErrorSimple = Radtan5PinholeReprojectionErrorSimple
    designVariable = Radtan5PinholeCameraGeometryDesignVariable
    projectionType = Radtan5PinholeProjection
    distortionType = RadialTangentialDistortion5
    shutterType = aslam_cv.GlobalShutter
    frameType = Radtan5PinholeFrame


class PinholeRadtan5Rs(object):
    cameraModel = "pinhole"
    distortionModel = "radtan5"
    geometry = Radtan5PinholeRsCameraGeometry
    reprojectionError = Radtan5PinholeRsReprojectionError
    reprojectionErrorSimple = Radtan5PinholeRsReprojectionErrorSimple
    reprojectionErrorAdaptiveCovariance = (
        Radtan5PinholeRsReprojectionErrorAdaptiveCovariance)
    designVariable = Radtan5PinholeRsCameraGeometryDesignVariable
    projectionType = Radtan5PinholeProjection
    distortionType = RadialTangentialDistortion5
    shutterType = aslam_cv.RollingShutter
    frameType = Radtan5PinholeRsFrame


from .integration import install  # noqa: E402,F401


__all__ = [
    "PinholeRadtan5",
    "PinholeRadtan5Rs",
    "RadialTangentialDistortion5",
    "Radtan5PinholeProjection",
    "Radtan5PinholeCameraGeometry",
    "Radtan5PinholeRsCameraGeometry",
    "Radtan5PinholeFrame",
    "Radtan5PinholeRsFrame",
    "install",
]
