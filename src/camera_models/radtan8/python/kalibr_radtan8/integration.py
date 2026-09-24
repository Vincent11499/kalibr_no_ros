"""Register the OpenCV rational radtan8 model with Kalibr's Python layer."""

from pathlib import Path

import numpy as np

from kalibr_radtan5.integration import (
    _camera_matrix,
    _install_camera_model,
    _write_camchain,
    _write_matrix_file,
)


_INSTALLED = False


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

    _install_camera_model(
        cr,
        cv,
        ("radtan8", "rational_polynomial"),
        8,
        RadialTangentialDistortion8,
        Radtan8PinholeProjection,
        Radtan8PinholeCameraGeometry,
        Radtan8PinholeFrame,
        Radtan8PinholeReprojectionErrorSimple,
    )

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

        _write_camchain(
            calibrator,
            result_file,
            graph,
            camera_models,
            distortion_models,
            lambda: _export_opencv_files(
                calibrator,
                result_file,
                cvb.DistortedPinhole,
                PinholeRadtan5,
                PinholeRadtan8,
            ),
            owner="radtan8",
        )

    CameraUtils.saveChainParametersYaml = save_chain_parameters_yaml
    kcc.saveChainParametersYaml = save_chain_parameters_yaml
    _INSTALLED = True
