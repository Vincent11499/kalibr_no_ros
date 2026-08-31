"""Register the opt-in OpenCV fisheye type with Kalibr's Python layer."""

from pathlib import Path


_INSTALLED = False


def _write_camchain(calibrator, result_file, graph, fisheye_model):
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
        fisheye_model: "pinhole",
    }
    distortion_models = {
        cvb.DistortedPinhole: "radtan",
        cvb.EquidistantPinhole: "equidistant",
        cvb.FovPinhole: "fov",
        cvb.Omni: "none",
        cvb.DistortedOmni: "radtan",
        cvb.ExtendedUnified: "none",
        cvb.DoubleSphere: "none",
        fisheye_model: "opencv_fisheye",
    }

    # The independent radtan5 extension may be installed in the same process.
    # Register it dynamically so a mixed pinhole chain remains writable.
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

    chain = cr.CameraChainParameters(result_file, createYaml=True)
    for camera_id, camera in enumerate(calibrator.cameras):
        try:
            camera_model = camera_models[camera.model]
            distortion_model = distortion_models[camera.model]
        except KeyError:
            raise RuntimeError(
                "OpenCV fisheye integration cannot serialize camera model {}".format(
                    camera.model
                )
            )
        parameters = cr.CameraParameters(result_file, createYaml=True)
        parameters.setRosTopic(camera.dataset.topic)
        projection = camera.geometry.projection()
        if camera_model == "omni":
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

    from . import (
        OpenCvFisheyeDistortion,
        OpenCvFisheyePinholeCameraGeometry,
        OpenCvFisheyePinholeFrame,
        OpenCvFisheyePinholeProjection,
        OpenCvFisheyePinholeReprojectionErrorSimple,
        PinholeOpenCvFisheye,
    )
    from .yaml_io import export_kalibr_camchain

    original_check_distortion = cr.CameraParameters.checkDistortion

    def check_distortion(self, model, coefficients):
        if model in ("opencv_fisheye", "fisheye"):
            if len(coefficients) != 4:
                self.raiseError(
                    "distortion model '{}' requires 4 coefficients; {} given".format(
                        model, len(coefficients)
                    )
                )
            return
        return original_check_distortion(self, model, coefficients)

    cr.CameraParameters.checkDistortion = check_distortion

    original_aslam_camera_init = cr.AslamCamera.__init__

    def aslam_camera_init(
        self, camera_model, intrinsics, distortion_model, coefficients, resolution
    ):
        if camera_model != "pinhole" or distortion_model not in (
            "opencv_fisheye",
            "fisheye",
        ):
            return original_aslam_camera_init(
                self,
                camera_model,
                intrinsics,
                distortion_model,
                coefficients,
                resolution,
            )
        distortion = OpenCvFisheyeDistortion(*coefficients)
        projection = OpenCvFisheyePinholeProjection(
            intrinsics[0],
            intrinsics[1],
            intrinsics[2],
            intrinsics[3],
            resolution[0],
            resolution[1],
            distortion,
        )
        self.geometry = OpenCvFisheyePinholeCameraGeometry(projection)
        self.frameType = OpenCvFisheyePinholeFrame
        self.keypointType = cv.Keypoint2
        self.reprojectionErrorType = OpenCvFisheyePinholeReprojectionErrorSimple
        self.undistorterType = None

    cr.AslamCamera.__init__ = aslam_camera_init

    original_save_chain_parameters_yaml = CameraUtils.saveChainParametersYaml

    def save_chain_parameters_yaml(calibrator, result_file, graph):
        if not any(
            camera.model is PinholeOpenCvFisheye for camera in calibrator.cameras
        ):
            return original_save_chain_parameters_yaml(calibrator, result_file, graph)

        _write_camchain(calibrator, result_file, graph, PinholeOpenCvFisheye)

        # The output prefix mirrors Kalibr's existing <bag>-camchain.yaml
        # convention and matches the radtan5 extension's filenames.
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
            # Kalibr supports mixed chains containing non-OpenCV central
            # models.  The Kalibr camchain remains valid; only the optional
            # OpenCV sidecar cannot represent that chain.
            print("  Skipping OpenCV sidecar export: {}".format(error))
        else:
            for output in outputs:
                print("  Saving OpenCV calibration to file: {}".format(output))

    CameraUtils.saveChainParametersYaml = save_chain_parameters_yaml
    kcc.saveChainParametersYaml = save_chain_parameters_yaml
    _INSTALLED = True
