"""Register the five-intrinsic OpenCV fisheye model in Kalibr's Python API."""

import sys
from pathlib import Path

import numpy as np


_INSTALLED = False
CAMERA_MODEL = "pinhole_opencv_fisheye"


def _write_camchain(calibrator, result_file, graph, full_model):
    import aslam_cv_backend as cvb
    from kalibr_common import ConfigReader as cr
    import sm

    camera_models = {
        cvb.DistortedPinhole: "pinhole",
        cvb.EquidistantPinhole: "pinhole",
        cvb.FovPinhole: "pinhole",
        cvb.Omni: "omni",
        cvb.DistortedOmni: "omni",
        cvb.ExtendedUnified: "eucm",
        cvb.DoubleSphere: "ds",
        full_model: CAMERA_MODEL,
    }
    distortion_models = {
        cvb.DistortedPinhole: "radtan",
        cvb.EquidistantPinhole: "equidistant",
        cvb.FovPinhole: "fov",
        cvb.Omni: "none",
        cvb.DistortedOmni: "radtan",
        cvb.ExtendedUnified: "none",
        cvb.DoubleSphere: "none",
        full_model: "opencv_fisheye",
    }

    try:
        from kalibr_radtan5 import PinholeRadtan5
    except ImportError:
        PinholeRadtan5 = None
    if PinholeRadtan5 is not None:
        camera_models[PinholeRadtan5] = "pinhole"
        distortion_models[PinholeRadtan5] = "radtan5"

    try:
        from kalibr_radtan8 import PinholeRadtan8
    except ImportError:
        PinholeRadtan8 = None
    if PinholeRadtan8 is not None:
        camera_models[PinholeRadtan8] = "pinhole"
        distortion_models[PinholeRadtan8] = "radtan8"

    try:
        from kalibr_opencv_fisheye import PinholeOpenCvFisheye
    except ImportError:
        PinholeOpenCvFisheye = None
    if PinholeOpenCvFisheye is not None:
        camera_models[PinholeOpenCvFisheye] = "pinhole"
        distortion_models[PinholeOpenCvFisheye] = "opencv_fisheye"

    chain = cr.CameraChainParameters(result_file, createYaml=True)
    for camera in calibrator.cameras:
        try:
            camera_model = camera_models[camera.model]
            distortion_model = distortion_models[camera.model]
        except KeyError:
            raise RuntimeError(
                "full OpenCV fisheye integration cannot serialize {}".format(
                    camera.model
                )
            )
        parameters = cr.CameraParameters(result_file, createYaml=True)
        parameters.setRosTopic(camera.dataset.topic)
        projection = camera.geometry.projection()
        if camera_model == CAMERA_MODEL:
            intrinsics = [
                projection.fu(),
                projection.fv(),
                projection.cu(),
                projection.cv(),
                projection.alpha(),
            ]
        elif camera_model == "omni":
            intrinsics = [
                projection.xi(),
                projection.fu(),
                projection.fv(),
                projection.cu(),
                projection.cv(),
            ]
        elif camera_model == "pinhole":
            intrinsics = [
                projection.fu(),
                projection.fv(),
                projection.cu(),
                projection.cv(),
            ]
        elif camera_model == "eucm":
            intrinsics = [
                projection.alpha(),
                projection.beta(),
                projection.fu(),
                projection.fv(),
                projection.cu(),
                projection.cv(),
            ]
        elif camera_model == "ds":
            intrinsics = [
                projection.xi(),
                projection.alpha(),
                projection.fu(),
                projection.fv(),
                projection.cu(),
                projection.cv(),
            ]
        else:
            raise RuntimeError("invalid camera model {}".format(camera_model))
        parameters.setIntrinsics(camera_model, intrinsics)
        parameters.setResolution([projection.ru(), projection.rv()])
        parameters.setDistortion(
            distortion_model,
            projection.distortion().getParameters().flatten(),
        )
        chain.addCameraAtEnd(parameters)

    for camera_id in range(len(calibrator.cameras)):
        chain.setCamOverlaps(camera_id, graph.getCamOverlaps(camera_id))
    for baseline_id, baseline in enumerate(calibrator.baselines):
        chain.setExtrinsicsLastCamToHere(
            baseline_id + 1, sm.Transformation(baseline.T())
        )
    chain.writeYaml()


def install():
    global _INSTALLED
    if _INSTALLED:
        return

    import aslam_cv as cv
    import kalibr_camera_calibration as kcc
    from kalibr_camera_calibration import CameraUtils
    from kalibr_common import ConfigReader as cr
    import kalibr_opencv_fisheye as zero_fisheye

    zero_fisheye.install()

    package = sys.modules[__package__]
    required = (
        "OpenCvFisheyeProjection",
        "OpenCvFisheyeCameraGeometry",
        "OpenCvFisheyeFrame",
        "OpenCvFisheyeReprojectionErrorSimple",
        "PinholeOpenCvFisheyeFull",
    )
    missing = [name for name in required if not hasattr(package, name)]
    if missing:
        raise ImportError(
            "full OpenCV fisheye native bindings are unavailable: {}".format(
                ", ".join(missing)
            )
        )

    full_model = package.PinholeOpenCvFisheyeFull
    original_check_intrinsics = cr.CameraParameters.checkIntrinsics

    def check_intrinsics(self, model, intrinsics):
        if model != CAMERA_MODEL:
            return original_check_intrinsics(self, model, intrinsics)
        values = np.asarray(intrinsics, dtype=np.float64).reshape(-1)
        if values.size != 5:
            self.raiseError(
                "{} intrinsics must be [fu,fv,cu,cv,alpha]".format(
                    CAMERA_MODEL
                )
            )
        if not np.all(np.isfinite(values)):
            self.raiseError("OpenCV fisheye intrinsics must be finite")
        if values[0] <= 0.0 or values[1] <= 0.0:
            self.raiseError("OpenCV fisheye focal lengths must be positive")

    cr.CameraParameters.checkIntrinsics = check_intrinsics

    original_aslam_camera_init = cr.AslamCamera.__init__

    def aslam_camera_init(
        self, camera_model, intrinsics, distortion_model, coefficients, resolution
    ):
        if camera_model != CAMERA_MODEL:
            return original_aslam_camera_init(
                self,
                camera_model,
                intrinsics,
                distortion_model,
                coefficients,
                resolution,
            )
        if distortion_model not in ("opencv_fisheye", "fisheye"):
            raise RuntimeError(
                "{} requires opencv_fisheye distortion".format(CAMERA_MODEL)
            )
        distortion = zero_fisheye.OpenCvFisheyeDistortion(*coefficients)
        projection = package.OpenCvFisheyeProjection(
            intrinsics[0],
            intrinsics[1],
            intrinsics[2],
            intrinsics[3],
            intrinsics[4],
            resolution[0],
            resolution[1],
            distortion,
        )
        self.geometry = package.OpenCvFisheyeCameraGeometry(projection)
        self.frameType = package.OpenCvFisheyeFrame
        self.keypointType = cv.Keypoint2
        self.reprojectionErrorType = package.OpenCvFisheyeReprojectionErrorSimple
        self.undistorterType = None

    cr.AslamCamera.__init__ = aslam_camera_init

    original_print_details = cr.CameraParameters.printDetails

    def print_details(self, dest=sys.stdout):
        camera_model, intrinsics = self.getIntrinsics()
        if camera_model != CAMERA_MODEL:
            return original_print_details(self, dest)
        distortion_model, coefficients = self.getDistortion()
        print("  Camera model: {}".format(camera_model), file=dest)
        print("  Focal length: {}".format(intrinsics[0:2]), file=dest)
        print("  Principal point: {}".format(intrinsics[2:4]), file=dest)
        print("  OpenCV fisheye alpha: {}".format(intrinsics[4]), file=dest)
        print("  Distortion model: {}".format(distortion_model), file=dest)
        print("  Distortion coefficients: {}".format(coefficients), file=dest)

    cr.CameraParameters.printDetails = print_details

    original_save = CameraUtils.saveChainParametersYaml

    def save_chain_parameters_yaml(calibrator, result_file, graph):
        if not any(camera.model is full_model for camera in calibrator.cameras):
            return original_save(calibrator, result_file, graph)
        _write_camchain(calibrator, result_file, graph, full_model)

        from .yaml_io import export_kalibr_camchain

        result_path = Path(result_file)
        suffix = "-camchain.yaml"
        result_text = str(result_path)
        prefix = (
            result_text[: -len(suffix)]
            if result_text.endswith(suffix)
            else str(result_path.with_suffix(""))
        )
        try:
            outputs = export_kalibr_camchain(result_file, prefix)
        except ValueError as error:
            print("  Skipping OpenCV sidecar export: {}".format(error))
        else:
            for output in outputs:
                print("  Saving OpenCV calibration to file: {}".format(output))

    CameraUtils.saveChainParametersYaml = save_chain_parameters_yaml
    kcc.saveChainParametersYaml = save_chain_parameters_yaml
    _INSTALLED = True
