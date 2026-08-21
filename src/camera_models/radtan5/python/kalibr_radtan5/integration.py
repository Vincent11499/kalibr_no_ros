"""Small runtime adapters that register radtan5 with Kalibr's Python layer."""

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


def _opencv_distortion(camera, distorted_pinhole, radtan5_model):
    values = np.asarray(
        camera.geometry.projection().distortion().getParameters(), dtype=np.float64
    ).reshape(-1)
    if camera.model is distorted_pinhole:
        values = np.concatenate((values, np.zeros(1, dtype=np.float64)))
    elif camera.model is not radtan5_model:
        return None
    return values.reshape(1, 5)


def _write_matrix_file(path, values):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise RuntimeError("Could not open OpenCV output {}".format(path))
    try:
        for name, value in values:
            storage.write(name, value)
    finally:
        storage.release()


def _export_opencv_files(calibrator, result_file, distorted_pinhole, radtan5_model):
    if not any(camera.model is radtan5_model for camera in calibrator.cameras):
        return []

    result_path = Path(result_file)
    suffix = "-camchain.yaml"
    result_text = str(result_path)
    bagtag = (
        result_text[: -len(suffix)]
        if result_text.endswith(suffix)
        else str(result_path.with_suffix(""))
    )
    exported = []
    camera_data = {}

    for index, camera in enumerate(calibrator.cameras):
        distortion = _opencv_distortion(camera, distorted_pinhole, radtan5_model)
        if distortion is None:
            continue
        projection = camera.geometry.projection()
        matrix = _camera_matrix(projection)
        path = Path("{}-cam{}-opencv.yaml".format(bagtag, index))
        _write_matrix_file(
            path,
            [
                ("camera_name", "cam{}".format(index)),
                ("image_width", int(projection.ru())),
                ("image_height", int(projection.rv())),
                ("distortion_model", "plumb_bob"),
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
        right_matrix, right_distortion, right_width, right_height = camera_data[right_index]
        fundamental = np.linalg.inv(right_matrix).T.dot(essential).dot(
            np.linalg.inv(left_matrix)
        )
        path = Path(
            "{}-cam{}-cam{}-opencv-stereo.yaml".format(
                bagtag, left_index, right_index
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
        PinholeRadtan5,
        RadialTangentialDistortion5,
        Radtan5PinholeCameraGeometry,
        Radtan5PinholeFrame,
        Radtan5PinholeProjection,
        Radtan5PinholeReprojectionErrorSimple,
    )

    original_check_distortion = cr.CameraParameters.checkDistortion

    def check_distortion(self, model, coeffs):
        if model == "radtan5":
            if len(coeffs) != 5:
                self.raiseError(
                    "distortion model 'radtan5' requires 5 coefficients; {} given".format(
                        len(coeffs)
                    )
                )
            return
        return original_check_distortion(self, model, coeffs)

    cr.CameraParameters.checkDistortion = check_distortion

    original_aslam_camera_init = cr.AslamCamera.__init__

    def aslam_camera_init(
        self, camera_model, intrinsics, dist_model, dist_coeff, resolution
    ):
        if camera_model != "pinhole" or dist_model != "radtan5":
            return original_aslam_camera_init(
                self, camera_model, intrinsics, dist_model, dist_coeff, resolution
            )
        distortion = RadialTangentialDistortion5(*dist_coeff)
        projection = Radtan5PinholeProjection(
            intrinsics[0],
            intrinsics[1],
            intrinsics[2],
            intrinsics[3],
            resolution[0],
            resolution[1],
            distortion,
        )
        self.geometry = Radtan5PinholeCameraGeometry(projection)
        self.frameType = Radtan5PinholeFrame
        self.keypointType = cv.Keypoint2
        self.reprojectionErrorType = Radtan5PinholeReprojectionErrorSimple
        self.undistorterType = None

    cr.AslamCamera.__init__ = aslam_camera_init

    original_save_chain_parameters_yaml = CameraUtils.saveChainParametersYaml

    def save_chain_parameters_yaml(calibrator, result_file, graph):
        # Preserve Kalibr's original four-parameter and non-pinhole paths
        # exactly.  The custom writer is needed only because upstream does not
        # know the radtan5 model name or its fifth design variable.
        if not any(
            camera.model is PinholeRadtan5 for camera in calibrator.cameras
        ):
            return original_save_chain_parameters_yaml(calibrator, result_file, graph)

        camera_models = {
            cvb.DistortedPinhole: "pinhole",
            PinholeRadtan5: "pinhole",
            cvb.EquidistantPinhole: "pinhole",
            cvb.FovPinhole: "pinhole",
            cvb.Omni: "omni",
            cvb.DistortedOmni: "omni",
            cvb.ExtendedUnified: "eucm",
            cvb.DoubleSphere: "ds",
        }
        distortion_models = {
            cvb.DistortedPinhole: "radtan",
            PinholeRadtan5: "radtan5",
            cvb.EquidistantPinhole: "equidistant",
            cvb.FovPinhole: "fov",
            cvb.Omni: "none",
            cvb.DistortedOmni: "radtan",
            cvb.ExtendedUnified: "none",
            cvb.DoubleSphere: "none",
        }
        chain = cr.CameraChainParameters(result_file, createYaml=True)
        for camera_id, camera in enumerate(calibrator.cameras):
            model = camera_models[camera.model]
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
                distortion_models[camera.model],
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
            calibrator, result_file, cvb.DistortedPinhole, PinholeRadtan5
        )

    CameraUtils.saveChainParametersYaml = save_chain_parameters_yaml
    kcc.saveChainParametersYaml = save_chain_parameters_yaml
    _INSTALLED = True
