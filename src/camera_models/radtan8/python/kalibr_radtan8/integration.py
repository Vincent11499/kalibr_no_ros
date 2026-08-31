"""Register the OpenCV rational radtan8 model with Kalibr's Python layer."""

from pathlib import Path

import cv2
import numpy as np


_INSTALLED = False


def _camera_matrix(projection):
    return np.array(
        [
            [projection.fu(), 0.0, projection.cu()],
            [0.0, projection.fv(), projection.cv()],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _rational_distortion(camera, distorted_pinhole, radtan5_model, radtan8_model):
    values = np.asarray(
        camera.geometry.projection().distortion().getParameters(), dtype=np.float64
    ).reshape(-1)
    if camera.model is distorted_pinhole:
        return np.pad(values, (0, 8 - values.size)).reshape(1, 8)
    if radtan5_model is not None and camera.model is radtan5_model:
        return np.pad(values, (0, 8 - values.size)).reshape(1, 8)
    if camera.model is radtan8_model:
        return values.reshape(1, 8)
    return None


def _write_matrix_file(path, values):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise RuntimeError("Could not open OpenCV output {}".format(path))
    try:
        for name, value in values:
            storage.write(name, value)
    finally:
        storage.release()


def _export_opencv_files(
    calibrator, result_file, distorted_pinhole, radtan5_model, radtan8_model
):
    if not any(camera.model is radtan8_model for camera in calibrator.cameras):
        return []

    result_path = Path(result_file)
    suffix = "-camchain.yaml"
    result_text = str(result_path)
    prefix = (
        result_text[: -len(suffix)]
        if result_text.endswith(suffix)
        else str(result_path.with_suffix(""))
    )
    exported = []
    camera_data = {}

    for index, camera in enumerate(calibrator.cameras):
        distortion = _rational_distortion(
            camera, distorted_pinhole, radtan5_model, radtan8_model
        )
        if distortion is None:
            continue
        projection = camera.geometry.projection()
        matrix = _camera_matrix(projection)
        path = Path("{}-cam{}-opencv.yaml".format(prefix, index))
        _write_matrix_file(
            path,
            [
                ("camera_name", "cam{}".format(index)),
                ("image_width", int(projection.ru())),
                ("image_height", int(projection.rv())),
                ("camera_model", "pinhole"),
                ("distortion_model", "rational_polynomial"),
                ("kalibr_distortion_model", "radtan8"),
                ("K", matrix),
                ("D", distortion),
                ("camera_matrix", matrix),
                ("distortion_coefficients", distortion),
            ],
        )
        camera_data[index] = (
            matrix,
            distortion,
            int(projection.ru()),
            int(projection.rv()),
        )
        exported.append(path)

    import sm

    for left_index, baseline in enumerate(calibrator.baselines):
        right_index = left_index + 1
        if left_index not in camera_data or right_index not in camera_data:
            continue
        transform = np.asarray(sm.Transformation(baseline.T()).T(), dtype=np.float64)
        rotation = transform[:3, :3]
        translation = transform[:3, 3:4]
        tx = np.array(
            [
                [0.0, -translation[2, 0], translation[1, 0]],
                [translation[2, 0], 0.0, -translation[0, 0]],
                [-translation[1, 0], translation[0, 0], 0.0],
            ],
            dtype=np.float64,
        )
        essential = tx.dot(rotation)
        left_matrix, left_distortion, left_width, left_height = camera_data[left_index]
        right_matrix, right_distortion, right_width, right_height = camera_data[
            right_index
        ]
        fundamental = np.linalg.inv(right_matrix).T.dot(essential).dot(
            np.linalg.inv(left_matrix)
        )
        path = Path(
            "{}-cam{}-cam{}-opencv-stereo.yaml".format(
                prefix, left_index, right_index
            )
        )
        _write_matrix_file(
            path,
            [
                ("left_camera", "cam{}".format(left_index)),
                ("right_camera", "cam{}".format(right_index)),
                ("left_image_width", left_width),
                ("left_image_height", left_height),
                ("right_image_width", right_width),
                ("right_image_height", right_height),
                ("camera_model", "pinhole"),
                ("distortion_model", "rational_polynomial"),
                ("kalibr_distortion_model", "radtan8"),
                ("transform_direction", "cam0-to-cam1"),
                ("translation_unit", "m"),
                ("translation_scale", 1.0),
                ("K1", left_matrix),
                ("D1", left_distortion),
                ("K2", right_matrix),
                ("D2", right_distortion),
                ("R", rotation),
                ("T", translation),
                ("E", essential),
                ("F", fundamental),
            ],
        )
        exported.append(path)

    for path in exported:
        print("  Saving OpenCV calibration to file: {}".format(path))
    return exported


def install():
    global _INSTALLED
    if _INSTALLED:
        return

    import aslam_cv as cv
    import aslam_cv_backend as cvb
    import kalibr_camera_calibration as kcc
    from kalibr_camera_calibration import CameraUtils
    from kalibr_common import ConfigReader as cr

    from . import (
        PinholeRadtan8,
        RadialTangentialDistortion8,
        Radtan8PinholeCameraGeometry,
        Radtan8PinholeFrame,
        Radtan8PinholeProjection,
        Radtan8PinholeReprojectionErrorSimple,
    )

    try:
        from kalibr_radtan5 import PinholeRadtan5
    except ImportError:
        PinholeRadtan5 = None

    original_check_distortion = cr.CameraParameters.checkDistortion

    def check_distortion(self, model, coeffs):
        if model in ("radtan8", "rational_polynomial"):
            if len(coeffs) != 8:
                self.raiseError(
                    "distortion model '{}' requires 8 coefficients; {} given".format(
                        model, len(coeffs)
                    )
                )
            return
        return original_check_distortion(self, model, coeffs)

    cr.CameraParameters.checkDistortion = check_distortion

    original_aslam_camera_init = cr.AslamCamera.__init__

    def aslam_camera_init(
        self, camera_model, intrinsics, dist_model, dist_coeff, resolution
    ):
        if camera_model != "pinhole" or dist_model not in (
            "radtan8",
            "rational_polynomial",
        ):
            return original_aslam_camera_init(
                self, camera_model, intrinsics, dist_model, dist_coeff, resolution
            )
        distortion = RadialTangentialDistortion8(*dist_coeff)
        projection = Radtan8PinholeProjection(
            intrinsics[0],
            intrinsics[1],
            intrinsics[2],
            intrinsics[3],
            resolution[0],
            resolution[1],
            distortion,
        )
        self.geometry = Radtan8PinholeCameraGeometry(projection)
        self.frameType = Radtan8PinholeFrame
        self.keypointType = cv.Keypoint2
        self.reprojectionErrorType = Radtan8PinholeReprojectionErrorSimple
        self.undistorterType = None

    cr.AslamCamera.__init__ = aslam_camera_init

    original_save_chain_parameters_yaml = CameraUtils.saveChainParametersYaml

    def save_chain_parameters_yaml(calibrator, result_file, graph):
        if not any(
            camera.model is PinholeRadtan8 for camera in calibrator.cameras
        ):
            return original_save_chain_parameters_yaml(calibrator, result_file, graph)

        camera_models = {
            cvb.DistortedPinhole: "pinhole",
            PinholeRadtan8: "pinhole",
            cvb.EquidistantPinhole: "pinhole",
            cvb.FovPinhole: "pinhole",
            cvb.Omni: "omni",
            cvb.DistortedOmni: "omni",
            cvb.ExtendedUnified: "eucm",
            cvb.DoubleSphere: "ds",
        }
        distortion_models = {
            cvb.DistortedPinhole: "radtan",
            PinholeRadtan8: "radtan8",
            cvb.EquidistantPinhole: "equidistant",
            cvb.FovPinhole: "fov",
            cvb.Omni: "none",
            cvb.DistortedOmni: "radtan",
            cvb.ExtendedUnified: "none",
            cvb.DoubleSphere: "none",
        }
        if PinholeRadtan5 is not None:
            camera_models[PinholeRadtan5] = "pinhole"
            distortion_models[PinholeRadtan5] = "radtan5"

        chain = cr.CameraChainParameters(result_file, createYaml=True)
        for camera in calibrator.cameras:
            try:
                model = camera_models[camera.model]
                distortion_model = distortion_models[camera.model]
            except KeyError:
                raise RuntimeError(
                    "radtan8 integration cannot serialize camera model {}".format(
                        camera.model
                    )
                )
            params = cr.CameraParameters(result_file, createYaml=True)
            params.setRosTopic(camera.dataset.topic)
            projection = camera.geometry.projection()
            if model == "omni":
                intrinsics = [
                    projection.xi(),
                    projection.fu(),
                    projection.fv(),
                    projection.cu(),
                    projection.cv(),
                ]
            elif model == "pinhole":
                intrinsics = [
                    projection.fu(),
                    projection.fv(),
                    projection.cu(),
                    projection.cv(),
                ]
            elif model == "eucm":
                intrinsics = [
                    projection.alpha(),
                    projection.beta(),
                    projection.fu(),
                    projection.fv(),
                    projection.cu(),
                    projection.cv(),
                ]
            elif model == "ds":
                intrinsics = [
                    projection.xi(),
                    projection.alpha(),
                    projection.fu(),
                    projection.fv(),
                    projection.cu(),
                    projection.cv(),
                ]
            else:
                raise RuntimeError("Invalid camera model {}.".format(model))
            params.setIntrinsics(model, intrinsics)
            params.setResolution([projection.ru(), projection.rv()])
            params.setDistortion(
                distortion_model,
                projection.distortion().getParameters().flatten(),
            )
            chain.addCameraAtEnd(params)
        for camera_id in range(len(calibrator.cameras)):
            chain.setCamOverlaps(camera_id, graph.getCamOverlaps(camera_id))
        for baseline_id, baseline_dv in enumerate(calibrator.baselines):
            import sm

            chain.setExtrinsicsLastCamToHere(
                baseline_id + 1, sm.Transformation(baseline_dv.T())
            )
        chain.writeYaml()
        _export_opencv_files(
            calibrator,
            result_file,
            cvb.DistortedPinhole,
            PinholeRadtan5,
            PinholeRadtan8,
        )

    CameraUtils.saveChainParametersYaml = save_chain_parameters_yaml
    kcc.saveChainParametersYaml = save_chain_parameters_yaml
    _INSTALLED = True
