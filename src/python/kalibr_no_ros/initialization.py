"""Validation for the public calibration-initialization YAML formats."""

from __future__ import annotations

import copy
import math
from pathlib import Path
import re

import yaml


INITIALIZATION_SCHEMA_VERSION = 1
INITIALIZATION_STRATEGIES = frozenset(("refine", "direct"))

_JOB_KINDS = {
    "camera_calibration": "camera_calibration_initialization",
    "camera_imu_calibration": "camera_imu_calibration_initialization",
}

# Public task model -> (intrinsic parameter count, distortion parameter count).
CAMERA_MODEL_DIMENSIONS = {
    "pinhole-radtan": (4, 4),
    "pinhole-radtan5": (4, 5),
    "pinhole-radtan8": (4, 8),
    "pinhole-equi": (4, 4),
    "pinhole-fov": (4, 1),
    "pinhole-opencv-fisheye": (5, 4),
    "omni-none": (5, 0),
    "omni-radtan": (5, 4),
    "eucm-none": (6, 0),
    "ds-none": (6, 0),
}

_CAMERA_FIELDS = {
    "intrinsics", "distortion_coeffs", "T_cam_from_previous",
}
_CAMERA_IMU_FIELDS = {
    "T_cam0_imu", "timeshift_cam_imu_s", "gravity_direction_target",
}
_IMU_COMMON_FIELDS = {
    "gyroscope_bias_rad_s",
    "accelerometer_bias_m_s2",
    "T_imu_from_reference",
    "time_offset_to_reference_s",
}
_IMU_INTRINSIC_FIELDS = {
    "M_accel", "M_gyro", "C_gyro_i", "A_gyro_accel",
}
_IMU_SIZE_EFFECT_FIELDS = {"ry_i_m", "rz_i_m"}
_INDEXED_CAMERA = re.compile(r"^cam(?:0|[1-9][0-9]*)$")


class InitializationError(ValueError):
    """Raised when a public initialization document is not valid."""


def _field_list(values):
    return ", ".join(sorted((str(value) for value in values)))


def _mapping(value, field):
    if not isinstance(value, dict):
        raise InitializationError("{} must be a mapping".format(field))
    return value


def _reject_unknown(value, allowed, field):
    unknown = set(value) - set(allowed)
    if unknown:
        raise InitializationError(
            "unknown fields in {}: {}".format(field, _field_list(unknown)))


def _finite_number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InitializationError("{} must be a finite number".format(field))
    result = float(value)
    if not math.isfinite(result):
        raise InitializationError("{} must be a finite number".format(field))
    return result


def _vector(value, length, field):
    if not isinstance(value, list) or len(value) != length:
        raise InitializationError(
            "{} must be a {}-element vector".format(field, length))
    return [
        _finite_number(element, "{}[{}]".format(field, index))
        for index, element in enumerate(value)
    ]


def _matrix(value, rows, columns, field):
    if not isinstance(value, list) or len(value) != rows:
        raise InitializationError(
            "{} must be a {}x{} matrix".format(field, rows, columns))
    result = []
    for row_index, row in enumerate(value):
        if not isinstance(row, list) or len(row) != columns:
            raise InitializationError(
                "{} must be a {}x{} matrix".format(field, rows, columns))
        result.append([
            _finite_number(
                element, "{}[{}][{}]".format(field, row_index, column_index))
            for column_index, element in enumerate(row)
        ])
    return result


def _determinant3(matrix):
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def _rotation(value, field):
    rotation = _matrix(value, 3, 3, field)
    tolerance = 1e-6
    for row in range(3):
        for column in range(3):
            dot = sum(rotation[index][row] * rotation[index][column]
                      for index in range(3))
            expected = 1.0 if row == column else 0.0
            if abs(dot - expected) > tolerance:
                raise InitializationError("{} rotation must be orthonormal".format(field))
    if abs(_determinant3(rotation) - 1.0) > tolerance:
        raise InitializationError(
            "{} rotation must have determinant +1".format(field))
    return rotation


def _transform(value, field):
    transform = _matrix(value, 4, 4, field)
    bottom = transform[3]
    if any(abs(bottom[index]) > 1e-9 for index in range(3)) or abs(bottom[3] - 1.0) > 1e-9:
        raise InitializationError(
            "{} bottom row must be [0, 0, 0, 1]".format(field))
    rotation = _rotation([row[:3] for row in transform[:3]], field)
    for row in range(3):
        transform[row][:3] = rotation[row]
    return transform


def validate_task_initialization(value):
    """Validate the optional ``initialization`` mapping in a task document."""
    value = _mapping(value, "initialization")
    _reject_unknown(value, {"path", "strategy"}, "initialization")
    missing = {"path"} - set(value)
    if missing:
        raise InitializationError(
            "initialization requires {}".format(_field_list(missing)))
    if not isinstance(value["path"], str) or not value["path"].strip():
        raise InitializationError("initialization.path must be a non-empty string")
    if ("strategy" in value
            and value["strategy"] not in INITIALIZATION_STRATEGIES):
        raise InitializationError(
            "initialization.strategy must be refine or direct")


def _validate_header(document, expected_job):
    if expected_job not in _JOB_KINDS:
        raise InitializationError(
            "unsupported initialization job: {}".format(expected_job))
    if (type(document.get("schema_version")) is not int
            or document.get("schema_version") != INITIALIZATION_SCHEMA_VERSION):
        raise InitializationError(
            "initialization schema_version must be {}".format(
                INITIALIZATION_SCHEMA_VERSION))
    expected_kind = _JOB_KINDS[expected_job]
    if document.get("kind") != expected_kind:
        raise InitializationError(
            "initialization kind must be {} for {}".format(
                expected_kind, expected_job))
    return expected_kind


def _camera_models(task):
    cameras = task.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise InitializationError("cameras must be a non-empty list")
    result = {}
    for index, camera in enumerate(cameras):
        field = "cameras[{}]".format(index)
        if not isinstance(camera, dict) or not isinstance(camera.get("model"), str):
            raise InitializationError("{}.model is required".format(field))
        model = camera["model"]
        if model not in CAMERA_MODEL_DIMENSIONS:
            raise InitializationError(
                "unsupported camera model for initialization: {}".format(model))
        result["cam{}".format(index)] = model
    return result


def _validate_camera_document(document, task, strategy):
    _reject_unknown(document, {"schema_version", "kind", "cameras"},
                    "camera initialization")
    if "cameras" not in document:
        raise InitializationError("camera initialization requires cameras")
    cameras = _mapping(document["cameras"], "cameras")
    models = _camera_models(task)
    unknown = set(cameras) - set(models)
    if unknown:
        raise InitializationError(
            "unknown camera initialization blocks: {}".format(
                _field_list(unknown)))

    result = {}
    for camera_id in models:
        if camera_id not in cameras:
            if strategy == "direct":
                raise InitializationError(
                    "direct initialization requires {}".format(camera_id))
            continue
        block = _mapping(cameras[camera_id], "cameras.{}".format(camera_id))
        _reject_unknown(block, _CAMERA_FIELDS, "cameras.{}".format(camera_id))
        if camera_id == "cam0" and "T_cam_from_previous" in block:
            raise InitializationError(
                "cameras.cam0 cannot contain T_cam_from_previous")
        intrinsic_count, distortion_count = CAMERA_MODEL_DIMENSIONS[models[camera_id]]
        validated = {}
        if "intrinsics" in block:
            validated["intrinsics"] = _vector(
                block["intrinsics"], intrinsic_count,
                "cameras.{}.intrinsics".format(camera_id))
        if "distortion_coeffs" in block:
            validated["distortion_coeffs"] = _vector(
                block["distortion_coeffs"], distortion_count,
                "cameras.{}.distortion_coeffs".format(camera_id))
        if "T_cam_from_previous" in block:
            validated["T_cam_from_previous"] = _transform(
                block["T_cam_from_previous"],
                "cameras.{}.T_cam_from_previous".format(camera_id))
        if strategy == "direct":
            for field in ("intrinsics", "distortion_coeffs"):
                if field not in validated:
                    raise InitializationError(
                        "direct initialization requires cameras.{}.{}".format(
                            camera_id, field))
            if camera_id != "cam0" and "T_cam_from_previous" not in validated:
                raise InitializationError(
                    "direct initialization requires cameras.{}.T_cam_from_previous".format(
                        camera_id))
        result[camera_id] = validated
    return {"cameras": result}


def _allowed_imu_fields(model):
    if model == "calibrated":
        return set(_IMU_COMMON_FIELDS)
    if model == "scale-misalignment":
        return set(_IMU_COMMON_FIELDS) | _IMU_INTRINSIC_FIELDS
    if model == "scale-misalignment-size-effect":
        return (set(_IMU_COMMON_FIELDS) | _IMU_INTRINSIC_FIELDS
                | _IMU_SIZE_EFFECT_FIELDS)
    raise InitializationError(
        "unsupported IMU model for initialization: {}".format(model))


def _validate_camera_imu_block(value, camera_ids):
    value = _mapping(value, "camera_imu")
    _reject_unknown(value, _CAMERA_IMU_FIELDS, "camera_imu")
    result = {}
    if "T_cam0_imu" in value:
        result["T_cam0_imu"] = _transform(
            value["T_cam0_imu"], "camera_imu.T_cam0_imu")
    if "timeshift_cam_imu_s" in value:
        shifts = _mapping(
            value["timeshift_cam_imu_s"],
            "camera_imu.timeshift_cam_imu_s")
        if camera_ids is None:
            invalid = [key for key in shifts
                       if not isinstance(key, str) or not _INDEXED_CAMERA.match(key)]
        else:
            invalid = set(shifts) - set(camera_ids)
        if invalid:
            raise InitializationError(
                "unknown camera time shifts: {}".format(_field_list(invalid)))
        ordered_ids = camera_ids if camera_ids is not None else sorted(shifts)
        result["timeshift_cam_imu_s"] = {
            camera_id: _finite_number(
                shifts[camera_id],
                "camera_imu.timeshift_cam_imu_s.{}".format(camera_id))
            for camera_id in ordered_ids if camera_id in shifts
        }
    if "gravity_direction_target" in value:
        gravity = _vector(
            value["gravity_direction_target"], 3,
            "camera_imu.gravity_direction_target")
        if math.sqrt(sum(element * element for element in gravity)) <= 1e-12:
            raise InitializationError(
                "camera_imu.gravity_direction_target must be nonzero")
        result["gravity_direction_target"] = gravity
    return result


def _validate_imu_blocks(value, task):
    value = _mapping(value, "imus")
    task_imus = task.get("imus")
    if not isinstance(task_imus, list) or not task_imus:
        raise InitializationError("imus must be a non-empty list")
    expected = ["imu{}".format(index) for index in range(len(task_imus))]
    unknown = set(value) - set(expected)
    if unknown:
        raise InitializationError(
            "unknown IMU initialization blocks: {}".format(_field_list(unknown)))

    result = {}
    for index, imu_id in enumerate(expected):
        if imu_id not in value:
            continue
        block = _mapping(value[imu_id], "imus.{}".format(imu_id))
        imu = task_imus[index]
        if not isinstance(imu, dict):
            raise InitializationError("imus[{}] must be a mapping".format(index))
        allowed = _allowed_imu_fields(imu.get("model", "calibrated"))
        _reject_unknown(block, allowed, "imus.{}".format(imu_id))
        if index == 0:
            reference_only = {
                "T_imu_from_reference", "time_offset_to_reference_s",
            } & set(block)
            if reference_only:
                raise InitializationError(
                    "imus.imu0 cannot contain {}".format(
                        _field_list(reference_only)))

        validated = {}
        for field in ("gyroscope_bias_rad_s", "accelerometer_bias_m_s2",
                      "ry_i_m", "rz_i_m"):
            if field in block:
                validated[field] = _vector(
                    block[field], 3, "imus.{}.{}".format(imu_id, field))
        for field in ("M_accel", "M_gyro", "A_gyro_accel"):
            if field in block:
                validated[field] = _matrix(
                    block[field], 3, 3, "imus.{}.{}".format(imu_id, field))
        for field in ("M_accel", "M_gyro"):
            if field not in validated:
                continue
            matrix = validated[field]
            if any(abs(matrix[row][column]) > 1e-12
                   for row in range(3) for column in range(row + 1, 3)):
                raise InitializationError(
                    "imus.{}.{} must be lower triangular".format(
                        imu_id, field))
            if any(matrix[index][index] <= 0.0 for index in range(3)):
                raise InitializationError(
                    "imus.{}.{} diagonal entries must be positive".format(
                        imu_id, field))
        if "C_gyro_i" in block:
            validated["C_gyro_i"] = _rotation(
                block["C_gyro_i"], "imus.{}.C_gyro_i".format(imu_id))
        if "T_imu_from_reference" in block:
            validated["T_imu_from_reference"] = _transform(
                block["T_imu_from_reference"],
                "imus.{}.T_imu_from_reference".format(imu_id))
        if "time_offset_to_reference_s" in block:
            validated["time_offset_to_reference_s"] = _finite_number(
                block["time_offset_to_reference_s"],
                "imus.{}.time_offset_to_reference_s".format(imu_id))
        result[imu_id] = validated
    return result


def _validate_camera_imu_document(document, task, camera_ids):
    _reject_unknown(
        document, {"schema_version", "kind", "camera_imu", "imus"},
        "camera-IMU initialization")
    result = {}
    if "camera_imu" in document:
        result["camera_imu"] = _validate_camera_imu_block(
            document["camera_imu"], camera_ids)
    if "imus" in document:
        result["imus"] = _validate_imu_blocks(document["imus"], task)
    return result


def _validate_strategy_requirements(body, task, strategy):
    if task.get("job") != "camera_imu_calibration" or strategy != "refine":
        return
    calibration = task.get("calibration") or {}
    camera_shifts = (body.get("camera_imu") or {}).get(
        "timeshift_cam_imu_s") or {}
    if (camera_shifts
            and calibration.get("calibrate_time_offset", True) is False):
        raise InitializationError(
            "refine camera time-shift initialization requires "
            "calibration.calibrate_time_offset: true")
    imu_time_seeds = [
        imu_id for imu_id, block in (body.get("imus") or {}).items()
        if "time_offset_to_reference_s" in block
    ]
    if (imu_time_seeds
            and not calibration.get("estimate_multi_imu_delay", False)):
        raise InitializationError(
            "refine inter-IMU time initialization for {} requires "
            "calibration.estimate_multi_imu_delay: true".format(
                _field_list(imu_time_seeds)))


def validate_initialization(document, expected_job, task, strategy="refine",
                            camera_ids=None):
    """Validate and canonicalize one public initialization document.

    ``strategy`` is supplied by the task/CLI rather than by the public document.
    The returned mapping intentionally does not yet contain it; callers add it to
    the ephemeral legacy-facing document.
    """
    document = _mapping(document, "initialization document")
    if strategy not in INITIALIZATION_STRATEGIES:
        raise InitializationError("initialization strategy must be refine or direct")
    kind = _validate_header(document, expected_job)
    if expected_job == "camera_calibration":
        body = _validate_camera_document(document, task, strategy)
    else:
        body = _validate_camera_imu_document(document, task, camera_ids)
    _validate_strategy_requirements(body, task, strategy)
    result = {
        "schema_version": INITIALIZATION_SCHEMA_VERSION,
        "kind": kind,
    }
    result.update(body)
    return result


def load_initialization(path, expected_job, task, strategy="refine",
                        camera_ids=None):
    """Load, strictly validate, and canonicalize an initialization YAML file."""
    path = Path(path)
    try:
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise InitializationError(
            "could not read initialization file {}: {}".format(path, error)) from error
    return validate_initialization(
        document, expected_job, task, strategy=strategy, camera_ids=camera_ids)


def canonical_initialization(document, strategy):
    """Add the effective strategy in stable top-level order for legacy code."""
    if strategy not in INITIALIZATION_STRATEGIES:
        raise InitializationError("initialization strategy must be refine or direct")
    result = {
        "schema_version": document["schema_version"],
        "kind": document["kind"],
        "strategy": strategy,
    }
    for field in ("cameras", "camera_imu", "imus"):
        if field in document:
            result[field] = document[field]
    return result


def initialization_stage_decisions(document, strategy, task, camera_ids=None):
    """Describe how supplied blocks alter initialization, not final activity.

    The report is deliberately derived from the validated canonical document.
    It therefore remains available even when the native command fails before
    producing a calibration result.
    """
    job = task["job"]
    decisions = []
    if job == "camera_calibration":
        cameras = document.get("cameras", {})
        for index, _camera in enumerate(task["cameras"]):
            camera_id = "cam{}".format(index)
            fields = sorted(cameras.get(camera_id, {}))
            decisions.append({
                "stage": "single_camera_intrinsics.{}".format(camera_id),
                "supplied_fields": [
                    field for field in fields
                    if field in {"intrinsics", "distortion_coeffs"}
                ],
                "analytic_projection": (
                    "skipped_seeded_geometry_from_observation_resolution"
                    if "intrinsics" in fields else "native"
                ),
                "single_camera_lm": "skipped" if strategy == "direct" else "run",
                "final_incremental_state": "active",
            })
        if len(task["cameras"]) > 1:
            supplied = [
                "cam{}".format(index)
                for index in range(1, len(task["cameras"]))
                if "T_cam_from_previous" in cameras.get(
                    "cam{}".format(index), {})
            ]
            complete = len(supplied) == len(task["cameras"]) - 1
            decisions.append({
                "stage": "camera_chain_baselines",
                "supplied_transforms": supplied,
                "pairwise_stereo_lm": (
                    "skipped" if strategy == "direct" or complete else "run"
                ),
                "full_batch_lm": "skipped" if strategy == "direct" else "run",
                "final_incremental_state": "active",
            })
        return decisions

    camera_ids = list(camera_ids or ())
    camera_imu = document.get("camera_imu", {})
    shifts = camera_imu.get("timeshift_cam_imu_s", {})
    time_calibration = (task.get("calibration") or {}).get(
        "calibrate_time_offset", True) is not False
    for camera_id in camera_ids:
        supplied = camera_id in shifts
        if not time_calibration:
            correlation = "disabled"
        elif supplied and strategy == "direct":
            correlation = "skipped"
        elif supplied:
            correlation = "residual_about_seed"
        else:
            correlation = "native"
        decisions.append({
            "stage": "camera_imu_time_offset.{}".format(camera_id),
            "seed_supplied": supplied,
            "cross_correlation": correlation,
            "final_joint_state": "active" if time_calibration else "inactive",
        })

    imu0 = document.get("imus", {}).get("imu0", {})
    transform_supplied = "T_cam0_imu" in camera_imu
    gyro_bias_supplied = "gyroscope_bias_rad_s" in imu0
    if strategy == "direct" and transform_supplied and gyro_bias_supplied:
        preliminary = "skipped"
    elif strategy == "direct" and transform_supplied:
        preliminary = "rotation_fixed_bias_solved"
    elif strategy == "direct" and gyro_bias_supplied:
        preliminary = "bias_fixed_rotation_solved"
    else:
        preliminary = "run_from_seed" if (
            transform_supplied or gyro_bias_supplied) else "native"
    decisions.append({
        "stage": "camera_imu_rotation_and_gyro_bias",
        "T_cam0_imu_supplied": transform_supplied,
        "imu0_gyro_bias_supplied": gyro_bias_supplied,
        "preliminary_lm": preliminary,
        "final_joint_state": "active",
    })

    for index, _imu in enumerate(task.get("imus") or ()):  # final joint seeds
        imu_id = "imu{}".format(index)
        block = document.get("imus", {}).get(imu_id, {})
        joint_fields = sorted(
            field for field in block
            if field != "time_offset_to_reference_s")
        decisions.append({
            "stage": "final_joint.{}".format(imu_id),
            "supplied_fields": joint_fields,
            "precomputed_fields": (
                ["time_offset_to_reference_s"]
                if "time_offset_to_reference_s" in block else []
            ),
            "seed_only": True,
            "fixed_or_prior": False,
        })
        if index:
            transform_supplied = "T_imu_from_reference" in block
            gyro_bias_supplied = "gyroscope_bias_rad_s" in block
            if strategy == "direct" and transform_supplied and gyro_bias_supplied:
                rotation_bias_lm = "skipped"
            elif strategy == "direct" and transform_supplied:
                rotation_bias_lm = "rotation_fixed_bias_solved"
            elif strategy == "direct" and gyro_bias_supplied:
                rotation_bias_lm = "bias_fixed_rotation_solved"
            elif transform_supplied or gyro_bias_supplied:
                rotation_bias_lm = "run_from_seed"
            else:
                rotation_bias_lm = "native"
            decisions.append({
                "stage": "multi_imu_preliminary.{}".format(imu_id),
                "reference_angular_velocity_spline": "always_estimated",
                "T_imu_from_reference_supplied": transform_supplied,
                "gyro_bias_supplied": gyro_bias_supplied,
                "rotation_and_gyro_bias_lm": rotation_bias_lm,
                "relative_rotation": (
                    "fixed_in_preliminary"
                    if strategy == "direct"
                    and transform_supplied else "solved"
                ),
                "time_correlation": (
                    "skipped"
                    if strategy == "direct"
                    and "time_offset_to_reference_s" in block
                    else ("residual_about_seed"
                          if "time_offset_to_reference_s" in block
                          else "native")
                ),
                "final_joint_state": "fixed_precomputed_offset",
            })
    return decisions


def build_initialization_report(document, strategy, task, *, source_path,
                                source_sha256, path_origin,
                                strategy_origin, camera_ids=None):
    """Build the stable provenance report written before native execution."""
    return {
        "schema_version": 1,
        "kind": "calibration_initialization_report",
        "job": task["job"],
        "status": "configured",
        "strategy": strategy,
        "source": {
            "path": str(source_path),
            "sha256": str(source_sha256),
            "path_origin": str(path_origin),
            "strategy_origin": str(strategy_origin),
        },
        "semantics": {
            "role": "initial_value_only",
            "fixed_parameter": False,
            "fixed_parameter_meaning": (
                "the seed adds no fixed state; native task activity still applies"
            ),
            "prior_error_term_added": False,
            "final_optimizer_activity_unchanged": True,
            "transform_convention": "p_target = T_target_source * p_source",
            "gravity_magnitude_m_s2": 9.80655,
        },
        "configured": copy.deepcopy(document),
        "stage_decisions": initialization_stage_decisions(
            document, strategy, task, camera_ids=camera_ids),
    }
