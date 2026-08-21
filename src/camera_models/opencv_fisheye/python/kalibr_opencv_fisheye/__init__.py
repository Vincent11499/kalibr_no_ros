"""Opt-in OpenCV fisheye camera model and calibration-file conversion."""

import aslam_cv

from .libkalibr_opencv_fisheye_cv_python import *

import aslam_backend  # noqa: F401 - registers backend base classes
import aslam_cv_backend

from .libkalibr_opencv_fisheye_backend_python import *


class PinholeOpenCvFisheye(object):
    geometry = OpenCvFisheyePinholeCameraGeometry
    reprojectionError = OpenCvFisheyePinholeReprojectionError
    reprojectionErrorSimple = OpenCvFisheyePinholeReprojectionErrorSimple
    designVariable = OpenCvFisheyePinholeCameraGeometryDesignVariable
    projectionType = OpenCvFisheyePinholeProjection
    distortionType = OpenCvFisheyeDistortion
    shutterType = aslam_cv.GlobalShutter
    frameType = OpenCvFisheyePinholeFrame


from .integration import install  # noqa: E402,F401
from .yaml_io import (  # noqa: E402,F401
    DIRECT_TRANSFORM,
    INVERSE_TRANSFORM,
    OpenCvCamera,
    OpenCvStereo,
    export_kalibr_camchain,
    import_opencv_mono,
    import_opencv_stereo,
    read_opencv_camera,
    read_opencv_stereo,
)


__all__ = [
    "PinholeOpenCvFisheye",
    "OpenCvFisheyeDistortion",
    "OpenCvFisheyePinholeProjection",
    "OpenCvFisheyePinholeCameraGeometry",
    "OpenCvFisheyePinholeFrame",
    "OpenCvCamera",
    "OpenCvStereo",
    "DIRECT_TRANSFORM",
    "INVERSE_TRANSFORM",
    "read_opencv_camera",
    "read_opencv_stereo",
    "import_opencv_mono",
    "import_opencv_stereo",
    "export_kalibr_camchain",
    "install",
]
