"""OpenCV fisheye model with five intrinsics and four distortion coefficients."""

import aslam_cv
import aslam_backend  # noqa: F401 - register backend base classes first
import aslam_cv_backend  # noqa: F401

from .libkalibr_opencv_fisheye_cv_python import *
from .libkalibr_opencv_fisheye_backend_python import *


class PinholeOpenCvFisheye(object):
    geometry = OpenCvFisheyeCameraGeometry
    reprojectionError = OpenCvFisheyeReprojectionError
    reprojectionErrorSimple = OpenCvFisheyeReprojectionErrorSimple
    designVariable = OpenCvFisheyeCameraGeometryDesignVariable
    projectionType = OpenCvFisheyeProjection
    distortionType = OpenCvFisheyeDistortion
    shutterType = aslam_cv.GlobalShutter
    frameType = OpenCvFisheyeFrame


from .integration import install  # noqa: E402,F401


__all__ = [
    "PinholeOpenCvFisheye",
    "OpenCvFisheyeDistortion",
    "OpenCvFisheyeProjection",
    "OpenCvFisheyeCameraGeometry",
    "OpenCvFisheyeCameraGeometryDesignVariable",
    "OpenCvFisheyeFrame",
    "install",
]
