"""Validate and project explicitly reused machine-facing calibration values."""

from datetime import datetime
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from ..task.initialization import (
    InitializationError, _matrix, _rotation, _transform,
)
from ..task.document import TaskError
from ..task.validation import IMU_MODELS
from ..version import (
    CALIBRATION_BEHAVIOR_VERSION,
    RESULT_SCHEMA_VERSION,
    SUPPORTED_CALIB_PARAMS_SCHEMA_VERSIONS,
    calibration_behavior_version,
    supports_schema_version,
)


_STAGE_FILES = {
    "camera": "vision/stereo_imu_calib.yaml",
    "camera_imu": "vision/stereo_imu_calib.yaml",
    "camera_baselink": "vision/stereo_imu_calib.yaml",
    "mono_camera": "mono_camera/usb_mono_calib.yaml",
    "mono_camera_baselink": "mono_camera/usb_mono_calib.yaml",
}
_CALIBRATED_FLAGS = {
    "camera_imu": "stereo_imu_calibrated",
    "camera_baselink": "stereo_baselink_calibrated",
    "mono_camera_baselink": "mono_baselink_calibrated",
}
_MODELS = {
    "pinhole-equidistant": ("pinhole", "equidistant", 4, 4),
    "pinhole-equi": ("pinhole", "equidistant", 4, 4),
    "pinhole-radtan": ("pinhole", "radtan", 4, 4),
    "pinhole-radtan5": ("pinhole", "radtan5", 4, 5),
    "pinhole-radtan8": ("pinhole", "radtan8", 4, 8),
    "pinhole-opencv-fisheye": (
        "pinhole_opencv_fisheye", "opencv_fisheye", 5, 4),
}
_IMU_INTRINSIC_FIELDS = {
    "M_accel", "M_gyro", "C_gyro_i", "A_gyro_accel",
    "rx_i_m", "ry_i_m", "rz_i_m",
}


def parameter_path(output, stage):
    """Return the one public parameter file owned by a workflow stage."""
    if stage not in _STAGE_FILES:
        raise TaskError("unknown parameter reuse stage: {}".format(stage))
    return Path(output) / "calib_params" / _STAGE_FILES[stage]


def semantic_sha256(value):
    """Hash parsed values, independently of YAML comments and formatting."""
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _mapping(value, label):
    if not isinstance(value, dict):
        raise TaskError("{} must be a mapping".format(label))
    return value


def _finite(value, label, *, minimum=None):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or minimum is not None and value < minimum):
        raise TaskError("{} must be a finite number{}".format(
            label, " >= {}".format(minimum) if minimum is not None else ""))
    return float(value)


def _vector(value, size, label):
    if not isinstance(value, list) or len(value) != size:
        raise TaskError("{} must contain {} values".format(label, size))
    return [_finite(element, "{}[{}]".format(label, index))
            for index, element in enumerate(value)]


def _scale_matrix(value, label):
    try:
        matrix = _matrix(value, 3, 3, label)
    except InitializationError as error:
        raise TaskError(str(error)) from error
    if any(abs(matrix[row][column]) > 1e-12
           for row in range(3) for column in range(row + 1, 3)):
        raise TaskError("{} must be lower triangular".format(label))
    if any(matrix[index][index] <= 0.0 for index in range(3)):
        raise TaskError("{} diagonal entries must be positive".format(label))
    return matrix


def validate_imu_parameters(value, label="imu"):
    """Validate the Camera-IMU fields carried by machine parameter reuse."""
    value = _mapping(value, label)
    rate = _finite(value.get("imu_data_rate"), label + ".imu_data_rate",
                   minimum=0.0)
    if rate == 0.0:
        raise TaskError(label + ".imu_data_rate must be positive")
    result = {
        "imu_data_rate": rate,
        "gyro_bias": _vector(value.get("gyro_bias"), 3, label + ".gyro_bias"),
        "gyro_scale": _vector(value.get("gyro_scale"), 3, label + ".gyro_scale"),
        "acc_bias": _vector(value.get("acc_bias"), 3, label + ".acc_bias"),
        "acc_scale": _vector(value.get("acc_scale"), 3, label + ".acc_scale"),
    }
    model = value.get("model")
    if model is None:
        if _IMU_INTRINSIC_FIELDS & set(value):
            raise TaskError(label + ".model is required with IMU intrinsics")
        return result
    if not isinstance(model, str) or model not in IMU_MODELS:
        raise TaskError(label + ".model is unsupported")
    result["model"] = model
    supplied = _IMU_INTRINSIC_FIELDS & set(value)
    if model == "calibrated":
        if supplied:
            raise TaskError(
                "{} model calibrated cannot contain {}".format(
                    label, ", ".join(sorted(supplied))))
        return result

    required = {"M_accel", "M_gyro", "C_gyro_i", "A_gyro_accel"}
    if not required <= supplied:
        raise TaskError("{} model {} requires {}".format(
            label, model, ", ".join(sorted(required - supplied))))
    result["M_accel"] = _scale_matrix(
        value["M_accel"], label + ".M_accel")
    result["M_gyro"] = _scale_matrix(
        value["M_gyro"], label + ".M_gyro")
    try:
        result["C_gyro_i"] = _rotation(
            value["C_gyro_i"], label + ".C_gyro_i")
        result["A_gyro_accel"] = _matrix(
            value["A_gyro_accel"], 3, 3, label + ".A_gyro_accel")
    except InitializationError as error:
        raise TaskError(str(error)) from error
    lever_fields = {"rx_i_m", "ry_i_m", "rz_i_m"}
    if model == "scale-misalignment-size-effect":
        if not lever_fields <= supplied:
            raise TaskError("{} model {} requires {}".format(
                label, model, ", ".join(sorted(lever_fields - supplied))))
        for field in ("rx_i_m", "ry_i_m", "rz_i_m"):
            result[field] = _vector(value[field], 3, label + "." + field)
    elif lever_fields & supplied:
        raise TaskError(
            "{} model scale-misalignment cannot contain size-effect parameters"
            .format(label))
    return result


def _header(document, identity):
    _mapping(document, "calib_params")
    for key in ("machine_version", "machine_num"):
        if not isinstance(document.get(key), str) or document[key] != identity[key]:
            raise TaskError("calib_params {} does not match the current machine".format(key))
    if not supports_schema_version(
            document.get("calib_file_version"),
            SUPPORTED_CALIB_PARAMS_SCHEMA_VERSIONS):
        raise TaskError(
            "calib_params calib_file_version must be one of "
            + ", ".join(sorted(SUPPORTED_CALIB_PARAMS_SCHEMA_VERSIONS)))
    if calibration_behavior_version({
            "version": document.get("algorithm_version"),
            "calibration_behavior_version": document.get(
                "calibration_behavior_version"),
    }) != CALIBRATION_BEHAVIOR_VERSION:
        raise TaskError("calib_params algorithm version is incompatible")
    try:
        datetime.strptime(document.get("time"), "%Y-%m-%d-%H:%M:%S")
    except (TypeError, ValueError) as error:
        raise TaskError("calib_params time must use YYYY-MM-DD-HH:MM:SS") from error


def _optics(value, label):
    value = _mapping(value, label)
    model = value.get("model")
    if not isinstance(model, str) or model not in _MODELS:
        raise TaskError("{} has an unsupported camera model".format(label))
    if model == "pinhole-equi":
        model = "pinhole-equidistant"
    unused_projection, unused_distortion, intrinsics, distortion = _MODELS[model]
    resolution = value.get("resolution")
    if (not isinstance(resolution, list) or len(resolution) != 2
            or any(type(element) is not int or element <= 0 for element in resolution)):
        raise TaskError("{}.resolution must be [positive width, positive height]".format(label))
    intrinsic = _vector(value.get("intrinsic"), intrinsics, label + ".intrinsic")
    if any(focal <= 0.0 for focal in intrinsic[:2]):
        raise TaskError("{}.intrinsic focal lengths must be positive".format(label))
    return {
        "model": model,
        "resolution": list(resolution),
        "intrinsic": intrinsic,
        "distortion": _vector(value.get("distortion"), distortion,
                              label + ".distortion"),
        "rms": _finite(value.get("rms"), label + ".rms", minimum=0.0),
    }


def _correction(value):
    value = _mapping(value, "camera.cam0_cam1_correct")
    result = {}
    for name in ("R1", "P1", "R2", "P2"):
        result[name] = (_rotation(value.get(name), name) if name.startswith("R")
                        else _matrix(value.get(name), 3, 4, name))
    for name in ("baseline", "alignment"):
        result[name] = _finite(value.get(name), name, minimum=0.0)
    return result


def _baselink(camera, label):
    camera = _mapping(camera, label)
    pitch = _finite(camera.get("pitch"), label + ".pitch", minimum=0.0)
    if pitch > 90.0:
        raise TaskError("{}.pitch must not exceed 90 degrees".format(label))
    return {
        "T_base_cam": _transform(camera.get("T_base_cam"), label + ".T_base_cam"),
        "pitch": pitch,
    }


def validate_reused_stage(document, stage, identity):
    """Return only the public fields explicitly accepted for *stage*.

    Numeric zeros and identity transforms are validated as values, not used to
    infer whether another computer actually completed a calibration stage.
    """
    parameter_path(".", stage)
    _header(document, identity)
    calibrated_flag = _CALIBRATED_FLAGS.get(stage)
    if calibrated_flag is not None and document.get(calibrated_flag) is not True:
        raise TaskError(
            "calib_params {} must be true to reuse {}"
            .format(calibrated_flag, stage))
    cameras = _mapping(document.get("camera"), "camera")
    try:
        if stage == "mono_camera":
            return {"camera": _optics(cameras, "camera")}
        if stage == "mono_camera_baselink":
            return {"camera": _baselink(cameras, "camera")}
        if stage == "camera_baselink":
            return {"camera": {"cam0": _baselink(cameras.get("cam0"), "camera.cam0")}}

        result = {"camera": {}}
        for camera_id in ("cam0", "cam1"):
            source = _mapping(cameras.get(camera_id), "camera." + camera_id)
            target = _optics(source, "camera." + camera_id)
            target["line_delay_s"] = _finite(
                source.get("line_delay_s"), camera_id + ".line_delay_s")
            if stage == "camera_imu":
                target["T_imu_cam"] = _transform(
                    source.get("T_imu_cam"), camera_id + ".T_imu_cam")
                target["td_cam_to_imu"] = _finite(
                    source.get("td_cam_to_imu"), camera_id + ".td_cam_to_imu")
            result["camera"][camera_id] = target
        result["camera"]["cam1"]["T_cn_cnm1"] = _transform(
            cameras["cam1"].get("T_cn_cnm1"), "cam1.T_cn_cnm1")
        result["camera"]["cam0_cam1_correct"] = _correction(
            cameras.get("cam0_cam1_correct"))
        if stage == "camera_imu":
            result["imu"] = validate_imu_parameters(document.get("imu"))
            # Both directions describe one rigid camera chain, even when its
            # baseline was refined by the Camera–IMU stage.
            expected = np.linalg.inv(np.asarray(
                result["camera"]["cam1"]["T_imu_cam"])) @ np.asarray(
                    result["camera"]["cam0"]["T_imu_cam"])
            if not np.allclose(expected, result["camera"]["cam1"]["T_cn_cnm1"],
                               rtol=0.0, atol=1e-6):
                raise TaskError("Camera-IMU transforms do not match T_cn_cnm1")
        return result
    except InitializationError as error:
        raise TaskError(str(error)) from error


def camera_seed(parameters, mono=False):
    """Convert accepted optics to ordinary low-level camera-result input."""
    public = _mapping(parameters.get("camera"), "camera")
    cameras = {"cam0": public} if mono else public
    result = {
        "schema_version": RESULT_SCHEMA_VERSION,
        "kind": "calibration_result",
        "calibration_type": "cameras",
        "transform_convention": "p_target = T_target_source * p_source",
        "cameras": [],
    }
    for camera_id in (("cam0",) if mono else ("cam0", "cam1")):
        camera = _optics(cameras.get(camera_id), camera_id)
        projection, distortion, unused_intrinsics, unused_coefficients = _MODELS[
            camera["model"]]
        native = {
            "id": camera_id,
            "camera_model": projection,
            "distortion_model": distortion,
            "intrinsics": camera["intrinsic"],
            "distortion_coeffs": camera["distortion"],
            "resolution": camera["resolution"],
            "rostopic": "/{}/image_raw".format(camera_id),
        }
        if camera_id == "cam1":
            try:
                native["T_cn_cnm1"] = _transform(
                    cameras[camera_id].get("T_cn_cnm1"), "cam1.T_cn_cnm1")
            except InitializationError as error:
                raise TaskError(str(error)) from error
        result["cameras"].append(native)
    return result


def assert_compatible_camera_geometry(first, second):
    """Require equal optical geometry while permitting a refined baseline."""
    first_cameras = _mapping(first.get("camera"), "first camera")
    second_cameras = _mapping(second.get("camera"), "second camera")
    for camera_id in ("cam0", "cam1"):
        first_camera = _optics(first_cameras.get(camera_id), camera_id)
        second_camera = _optics(second_cameras.get(camera_id), camera_id)
        if (_MODELS[first_camera["model"]][:2] != _MODELS[second_camera["model"]][:2]
                or first_camera["resolution"] != second_camera["resolution"]
                or any(not np.allclose(first_camera[field], second_camera[field],
                                       rtol=1e-9, atol=1e-12)
                       for field in ("intrinsic", "distortion"))):
            raise TaskError("reused camera geometry does not match {}".format(camera_id))
