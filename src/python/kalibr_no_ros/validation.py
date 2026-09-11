"""Configuration and input validation without detection or optimization."""

import math
import re

from kalibr_bag_io import open_dataset, DirectoryReader
from .version import SCHEMA_VERSION
from .initialization import CAMERA_MODEL_DIMENSIONS, _transform
from .task import (
    TaskError, load_task, load_yaml, require_document_version,
    resolve_execution, resolve_initialization, resolve_task_path,
)


def _fields(value, allowed, label):
    if not isinstance(value, dict):
        raise TaskError("{} must be a mapping".format(label))
    unknown = set(value) - set(allowed)
    if unknown:
        raise TaskError("unknown fields in {}: {}".format(label, ", ".join(sorted(map(str, unknown)))))


def _positive(value, label, zero=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (value < 0 if zero else value <= 0):
        raise TaskError("{} must be a {}finite number".format(label, "non-negative " if zero else "positive "))


def _input_path(block, label):
    if isinstance(block, dict):
        _fields(block, {"path"}, label)
        path = block.get("path")
    else:
        path = block
    if not isinstance(path, str) or not path.strip():
        raise TaskError("{}.path must be a non-empty string".format(label))
    return path


def _camera_id(value, seen, label):
    if (not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", value)
            or value in seen):
        raise TaskError("{} must be a unique safe camera ID (letters, digits, '_' or '-')".format(label))
    seen.add(value)


def _quality_summary(value, label, rms=False):
    if value is None:
        return
    if type(value) in (int, float):
        if not math.isfinite(value) or value < 0:
            raise TaskError(label + ' must be a nonnegative finite pixel RMS or null')
        return
    numeric = {"value"} if rms else {"mean", "std", "min", "max", "median", "p95", "rms", "rms_px", "mean_abs_px"}
    _fields(value, {"status", "count", "unit", "reason"} | numeric, label)
    if value.get("status") not in ("available", "unavailable"):
        raise TaskError("{}.status must be available or unavailable".format(label))
    if "count" in value and (type(value["count"]) is not int or value["count"] < 0):
        raise TaskError("{}.count must be a non-negative integer".format(label))
    if "unit" in value and value["unit"] != "px":
        raise TaskError("{}.unit must be px".format(label))
    if "reason" in value and not isinstance(value["reason"], str):
        raise TaskError("{}.reason must be a string".format(label))
    for name in numeric & set(value):
        if value[name] is not None:
            _positive(value[name], label + "." + name, zero=True)
    if rms and value["status"] == "available" and value.get("value") is None:
        raise TaskError("{}.value is required when available".format(label))


def validate_options(task):
    from .rolling_shutter import JOB, validate_shutters
    if task["job"] == JOB:
        validate_shutters(task.get("rolling_shutter"))
    dataset = task["dataset"]
    _fields(dataset, {"type", "path", "time_range_s", "frequency_hz"}, "dataset")
    if not isinstance(dataset["path"], str) or not dataset["path"].strip():
        raise TaskError("dataset.path must be a non-empty string")
    if "frequency_hz" in dataset:
        _positive(dataset["frequency_hz"], "dataset.frequency_hz")
    if "time_range_s" in dataset:
        interval = dataset["time_range_s"]
        if not isinstance(interval, list) or len(interval) != 2:
            raise TaskError("dataset.time_range_s must contain [start, end]")
        for value in interval:
            _positive(value, "dataset.time_range_s", zero=True)
        if interval[1] <= interval[0]:
            raise TaskError("dataset.time_range_s end must be after start")
    calibration = task.get("calibration") or {}
    common = {"window_half_size_px", "max_displacement_px"}
    camera_fields = {
        "freeze_intrinsics", "focal_initialization_min_visible_corner_ratio",
        "synchronization_tolerance_s", "qr_tolerance", "information_gain_tolerance",
        "min_views_for_outlier_statistics", "shuffle", "remove_outliers",
        "final_filtering", "blake_zisserman",
    }
    imu_fields = {
        "max_iterations", "time_offset_padding_s", "reprojection_sigma_px",
        "synchronize_clocks", "estimate_multi_imu_delay", "calibrate_time_offset",
        "recover_covariance", "recompute_camera_chain_extrinsics",
    }
    _fields(calibration, common | (camera_fields if task["job"] == "camera_calibration" else imu_fields), "calibration")
    booleans = {
        "freeze_intrinsics", "shuffle", "remove_outliers", "final_filtering", "blake_zisserman",
        "synchronize_clocks", "estimate_multi_imu_delay", "calibrate_time_offset",
        "recover_covariance", "recompute_camera_chain_extrinsics",
    }
    integers = {"window_half_size_px", "min_views_for_outlier_statistics", "max_iterations"}
    for key, value in calibration.items():
        if key in booleans:
            if type(value) is not bool:
                raise TaskError("calibration.{} must be a boolean".format(key))
        elif key in integers:
            if type(value) is not int or value < 1:
                raise TaskError("calibration.{} must be a positive integer".format(key))
        else:
            if key == "information_gain_tolerance" and type(value) in (int, float) and value == -1:
                continue
            if key == "qr_tolerance" and type(value) in (int, float) and math.isfinite(value):
                continue
            _positive(value, "calibration." + key, zero=key in {"synchronization_tolerance_s", "information_gain_tolerance"})
            if key == "focal_initialization_min_visible_corner_ratio" and value > 1:
                raise TaskError("calibration.{} must be in (0, 1]".format(key))
    resolve_execution(task.get("execution"))
    if task["job"] == "camera_calibration":
        topics = set()
        camera_ids = set()
        for index, camera in enumerate(task["cameras"]):
            _fields(camera, {"id", "topic", "model"}, "cameras[{}]".format(index))
            _camera_id(camera.get("id", "cam{}".format(index)), camera_ids, "cameras[{}].id".format(index))
            topic = camera.get("topic")
            directory = dataset["type"] == "directory"
            if directory and "topic" not in camera and "id" not in camera:
                raise TaskError("directory cameras require an explicit id")
            if (not directory or "topic" in camera) and (not isinstance(topic, str) or not topic.strip() or topic in topics):
                raise TaskError("camera topics must be non-empty and unique")
            if not isinstance(camera.get("model"), str) or camera["model"] not in CAMERA_MODEL_DIMENSIONS:
                raise TaskError("unsupported camera model: {}".format(camera.get("model")))
            topics.add(topic)
    else:
        if "camera_calibration" in task:
            _input_path(task["camera_calibration"], "camera_calibration")
        for index, imu in enumerate(task["imus"]):
            _fields(imu, {"id", "path", "model"}, "imus")
            if imu.get("id", "imu{}".format(index)) != "imu{}".format(index):
                raise TaskError("IMU IDs must follow the task order imu0..imuN")
            if not isinstance(imu.get("path"), str) or not imu["path"].strip():
                raise TaskError("imus.path is required")
            if not isinstance(imu.get("model", "calibrated"), str) or imu.get("model", "calibrated") not in {"calibrated", "scale-misalignment", "scale-misalignment-size-effect"}:
                raise TaskError("unsupported IMU model")


def load_target(task):
    block = task["target"]
    _fields(block, {"path", "type", "parameters"}, "target")
    if "path" in block:
        if set(block) != {"path"}:
            raise TaskError("target.path is mutually exclusive with inline target")
        value = load_yaml(resolve_task_path(task, _input_path(block, "target")))
        require_document_version(value, "target", "calibration_target")
    else:
        if not isinstance(block.get("parameters"), dict):
            raise TaskError("target requires path or type plus parameters")
        value = dict(block["parameters"])
        if {"schema_version", "kind", "target_type"} & set(value):
            raise TaskError("inline target.parameters must contain only target-specific fields")
        value["target_type"] = block.get("type")
    target_type = value.get("target_type")
    fields = {
        "aprilgrid": {"tagRows", "tagCols", "tagSize", "tagSpacing", "tagStartId"},
        "checkerboard": {"targetRows", "targetCols", "rowSpacingMeters", "colSpacingMeters"},
        "circlegrid": {"targetRows", "targetCols", "spacingMeters", "asymmetricGrid"},
    }
    if not isinstance(target_type, str) or target_type not in fields:
        raise TaskError("unsupported target_type")
    _fields(value, {"schema_version", "kind", "target_type"} | fields[target_type], "target document")
    row_fields = ("tagRows", "tagCols") if target_type == "aprilgrid" else ("targetRows", "targetCols")
    for key in row_fields:
        if type(value.get(key)) is not int or value[key] < 3:
            raise TaskError("target.{} must be an integer >= 3".format(key))
    spacing_fields = {
        "aprilgrid": ("tagSize", "tagSpacing"),
        "checkerboard": ("rowSpacingMeters", "colSpacingMeters"),
        "circlegrid": ("spacingMeters",),
    }
    for key in spacing_fields[target_type]:
        # ConfigReader requires floats: rejecting integer YAML here avoids a
        # later native reader failure after input validation has succeeded.
        if type(value.get(key)) is not float:
            raise TaskError("target.{} must be a positive finite float".format(key))
        _positive(value[key], "target." + key)
    if target_type == "aprilgrid":
        start = value.get("tagStartId", 0)
        if type(start) is not int or start < 0 or start + value["tagRows"] * value["tagCols"] > 587:
            raise TaskError("AprilGrid tag IDs must be within tag36h11 [0, 587)")
    elif target_type == "circlegrid" and type(value.get("asymmetricGrid")) is not bool:
        raise TaskError("target.asymmetricGrid must be a boolean")
    return value


def _directory_reader(task):
    if task.get("dataset", {}).get("type") == "directory":
        return DirectoryReader(resolve_task_path(task, task["dataset"]["path"]))
    return None


def load_imu(task, block):
    _fields(block, {"id", "path", "model"}, "imus")
    path = _input_path(block.get("path"), "imus")
    value = load_yaml(resolve_task_path(task, path))
    require_document_version(value, "IMU", "imu_configuration")
    numeric_fields = {"accelerometer_noise_density", "accelerometer_random_walk", "gyroscope_noise_density", "gyroscope_random_walk", "update_rate"}
    _fields(value, {"schema_version", "kind", "rostopic"} | numeric_fields, "IMU document")
    for name in numeric_fields:
        _positive(value.get(name), "IMU." + name)
    reader = _directory_reader(task)
    if (reader is None or "rostopic" in value) and (not isinstance(value.get("rostopic"), str) or not value["rostopic"].strip()):
        raise TaskError("IMU.rostopic is required")
    if reader is not None:
        sensor_id = block["id"] if "id" in block else "imu{}".format(task.get("imus", []).index(block))
        value["rostopic"] = reader.sensor_topic(sensor_id, "imu", value.get("rostopic"))
    return value


def load_cameras(task):
    if task["job"] == "camera_calibration":
        reader = _directory_reader(task)
        if reader is not None:
            return [dict(camera, topic=reader.sensor_topic(camera.get("id", "cam{}".format(index)), "camera", camera.get("topic")))
                    for index, camera in enumerate(task["cameras"])]
        return task["cameras"]
    if "camera_calibration" not in task:
        raise TaskError("camera_calibration.path is required for standalone validation; calibrate imu-camera can discover a camera result in its output directory")
    path = _input_path(task["camera_calibration"], "camera_calibration")
    value = load_yaml(resolve_task_path(task, path))
    require_document_version(value, "camera calibration", "calibration_result")
    _fields(value, {"schema_version", "kind", "calibration_type", "transform_convention", "cameras", "imus"}, "camera calibration result")
    if "calibration_type" in value and value["calibration_type"] not in ("cameras", "camera_imu"):
        raise TaskError("camera calibration_type must be cameras or camera_imu")
    if "transform_convention" in value and value["transform_convention"] != "p_target = T_target_source * p_source":
        raise TaskError("unsupported camera result transform_convention")
    if "imus" in value and (not isinstance(value["imus"], list) or any(not isinstance(imu, dict) for imu in value["imus"])):
        raise TaskError("camera result imus must be a list of mappings")
    cameras = value.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise TaskError("camera calibration contains no cameras")
    topics = set()
    camera_ids = set()
    for index, camera in enumerate(cameras):
        _fields(camera, {"id", "camera_model", "distortion_model", "intrinsics", "distortion_coeffs",
                         "resolution", "rostopic", "T_cn_cnm1", "T_cam_imu", "timeshift_cam_imu",
                         "cam_overlaps", "line_delay", "from_camera", "rms", "alignment", "shutter"}, "camera result entry")
        _camera_id(camera.get("id"), camera_ids, "camera result id")
        for name in ("rms", "alignment"):
            if name in camera:
                _quality_summary(camera[name], "camera result " + name, rms=name == "rms")
        projection = camera.get("camera_model")
        distortion = camera.get("distortion_model")
        if not isinstance(projection, str) or not isinstance(distortion, str):
            raise TaskError("camera result camera_model and distortion_model must be strings")
        aliases = {"equidistant": "equi", "radtan": "radtan", "none": "none", "fov": "fov", "radtan5": "radtan5", "radtan8": "radtan8"}
        if projection == "pinhole_opencv_fisheye":
            model = "pinhole-opencv-fisheye"
        else:
            model = "{}-{}".format(projection, aliases.get(distortion, distortion))
        dimensions = CAMERA_MODEL_DIMENSIONS.get(model)
        if dimensions is None:
            raise TaskError("unsupported camera result model: {}".format(model))
        for field, count in zip(("intrinsics", "distortion_coeffs"), dimensions):
            values = camera.get(field)
            if not isinstance(values, list) or len(values) != count or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise TaskError("{}.{} must contain {} finite values".format(camera["id"], field, count))
        resolution = camera.get("resolution")
        if not isinstance(resolution, list) or len(resolution) != 2 or any(type(v) is not int or v <= 0 for v in resolution):
            raise TaskError("camera result resolution must be [positive width, positive height]")
        if "shutter" in camera:
            from .rolling_shutter import JOB
            if task["job"] != JOB:
                raise TaskError("a rolling-shutter result requires the rolling-shutter job")
            shutter = camera["shutter"]
            if not isinstance(shutter, dict) or shutter.get("type") != "rolling_shutter":
                raise TaskError("invalid camera shutter result")
            for field in ("line_delay_s", "reference_row_px", "first_to_last_row_span_s"):
                if type(shutter.get(field)) not in (int, float) or not math.isfinite(shutter[field]):
                    raise TaskError("invalid shutter." + field)
        topic = camera.get("rostopic")
        if not isinstance(topic, str) or not topic.strip() or topic in topics:
            raise TaskError("camera result rostopic must be non-empty and unique")
        topics.add(topic)
        if "cam_overlaps" in camera:
            overlaps = camera["cam_overlaps"]
            if not isinstance(overlaps, list) or any(type(v) is not int or not 0 <= v < len(cameras) for v in overlaps):
                raise TaskError("camera result cam_overlaps must contain valid camera indices")
        for field in ("timeshift_cam_imu", "line_delay"):
            if field in camera and (type(camera[field]) not in (int, float) or not math.isfinite(camera[field])):
                raise TaskError("camera result {} must be a finite number".format(field))
        if index > 0 and "T_cn_cnm1" not in camera:
            raise TaskError("camera result requires T_cn_cnm1 for {}".format(camera["id"]))
        if "from_camera" in camera and (index == 0 or "T_cn_cnm1" not in camera
                                        or camera["from_camera"] != cameras[index - 1]["id"]):
            raise TaskError("camera result from_camera must identify the preceding camera of T_cn_cnm1")
        for field in ("T_cn_cnm1", "T_cam_imu"):
            if field in camera:
                _transform(camera[field], field)
    reader = _directory_reader(task)
    return [dict(camera, topic=(reader.sensor_topic(camera["id"], "camera")
                                if reader is not None else camera["rostopic"]))
            for camera in cameras]


def validate_task(config, *, decode_images=True):
    """Return a versioned report; no target detector or native solver is loaded."""
    report = {"schema_version": SCHEMA_VERSION, "kind": "input_validation", "status": "passed", "errors": [], "warnings": [], "cameras": [], "imus": []}
    try:
        task = load_task(config)
        validate_options(task)
        load_target(task)
        cameras = load_cameras(task)
        from .rolling_shutter import JOB, validate_shutters
        if task["job"] == JOB:
            validate_shutters(task["rolling_shutter"], [camera["id"] for camera in cameras])
        imu_configs = [load_imu(task, block) for block in task.get("imus", [])]
        resolve_initialization(task)
        reader = open_dataset(resolve_task_path(task, task["dataset"]["path"]))
        is_directory = isinstance(reader, DirectoryReader)
        if (task["dataset"]["type"] == "directory") != is_directory:
            raise TaskError("dataset type mismatch")
        if is_directory:
            report["dataset_id"] = reader.dataset_id
        camera_windows = []
        for camera in cameras:
            topic = camera["topic"]
            with reader.index_images(topic) as dataset:
                indices = reader._decimate(reader._crop(dataset.index, task["dataset"].get("time_range_s")), task["dataset"].get("frequency_hz"))
                if not indices:
                    raise TaskError("camera {} has no selected images".format(topic))
                resolution = None
                if decode_images:
                    for entry in indices:
                        image = dataset.get_by_entry(entry).image
                        actual = [int(image.shape[1]), int(image.shape[0])]
                        if resolution is not None and actual != resolution:
                            raise TaskError("camera {} has inconsistent image resolution at {}".format(topic, entry.header_timestamp_ns))
                        resolution = actual
                    if camera.get("resolution") and resolution != camera["resolution"]:
                        raise TaskError("camera {} resolution does not match camera calibration".format(topic))
                lower, upper = indices[0].header_timestamp_ns, indices[-1].header_timestamp_ns
                camera_windows.append((lower, upper))
                report["cameras"].append({"topic": topic, "images": len(indices), "resolution": resolution, "first_timestamp_ns": str(lower), "last_timestamp_ns": str(upper)})
        for imu in imu_configs:
            records = reader.read_imu(imu["rostopic"], from_to=task["dataset"].get("time_range_s"))
            if len(records) < 2:
                raise TaskError("IMU {} requires at least two samples".format(imu["rostopic"]))
            lower, upper = records[0].header_timestamp_ns, records[-1].header_timestamp_ns
            if camera_windows and (lower > min(v[0] for v in camera_windows) or upper < max(v[1] for v in camera_windows)):
                raise TaskError("IMU time range does not cover selected camera observations")
            duration = (upper - lower) * 1e-9
            rate = (len(records) - 1) / duration if duration > 0 else None
            report["imus"].append({"topic": imu["rostopic"], "samples": len(records), "observed_rate_hz": rate, "configured_rate_hz": imu["update_rate"], "first_timestamp_ns": str(lower), "last_timestamp_ns": str(upper)})
        report["note"] = "Structure and timing checks only; calibration quality and observability are not evaluated."
    except (ValueError, RuntimeError, OSError, KeyError) as error:
        report["status"] = "failed"
        report["errors"].append(str(error))
    return report
