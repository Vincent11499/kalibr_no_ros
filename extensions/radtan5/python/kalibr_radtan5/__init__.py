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


from .integration import install  # noqa: E402,F401


__all__ = [
    "PinholeRadtan5",
    "RadialTangentialDistortion5",
    "Radtan5PinholeProjection",
    "Radtan5PinholeCameraGeometry",
    "Radtan5PinholeFrame",
    "install",
]
