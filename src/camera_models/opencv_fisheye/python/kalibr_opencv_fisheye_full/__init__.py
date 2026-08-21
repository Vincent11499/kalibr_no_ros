"""Full OpenCV fisheye model with optimizable alpha/skew."""

from kalibr_opencv_fisheye import OpenCvFisheyeDistortion

import aslam_backend  # noqa: F401 - register backend base classes first
import aslam_cv
import aslam_cv_backend

try:
    from .libkalibr_opencv_fisheye_full_cv_python import *
    from .libkalibr_opencv_fisheye_full_backend_python import *
except ImportError:
    # Keep the pure-Python YAML converter importable in source-tree tests.
    # Calibration use still fails clearly in install() if native bindings are
    # absent.
    pass


if "OpenCvFisheyeProjection" in globals():
    class PinholeOpenCvFisheyeFull(object):
        geometry = OpenCvFisheyeCameraGeometry
        reprojectionError = OpenCvFisheyeReprojectionError
        reprojectionErrorSimple = OpenCvFisheyeReprojectionErrorSimple
        designVariable = OpenCvFisheyeCameraGeometryDesignVariable
        projectionType = OpenCvFisheyeProjection
        distortionType = OpenCvFisheyeDistortion
        shutterType = aslam_cv.GlobalShutter
        frameType = OpenCvFisheyeFrame


from .integration import install  # noqa: E402,F401
from .yaml_io import (  # noqa: E402,F401
    CAMERA_MODEL,
    DIRECT_TRANSFORM,
    DISTORTION_MODEL,
    INVERSE_TRANSFORM,
    OpenCvFisheyeCamera,
    OpenCvFisheyeStereo,
    export_kalibr_camchain,
    import_opencv_mono,
    import_opencv_stereo,
    read_kalibr_camchain,
    read_opencv_camera,
    read_opencv_stereo,
    write_opencv_camera,
    write_opencv_stereo,
)


__all__ = [
    "CAMERA_MODEL",
    "DISTORTION_MODEL",
    "DIRECT_TRANSFORM",
    "INVERSE_TRANSFORM",
    "OpenCvFisheyeCamera",
    "OpenCvFisheyeStereo",
    "read_opencv_camera",
    "read_opencv_stereo",
    "read_kalibr_camchain",
    "import_opencv_mono",
    "import_opencv_stereo",
    "export_kalibr_camchain",
    "write_opencv_camera",
    "write_opencv_stereo",
    "install",
]

if "OpenCvFisheyeProjection" in globals():
    __all__ += [
        "PinholeOpenCvFisheyeFull",
        "OpenCvFisheyeProjection",
        "OpenCvFisheyeCameraGeometry",
        "OpenCvFisheyeFrame",
    ]
