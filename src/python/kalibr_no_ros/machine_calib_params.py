"""Build machine-consumable parameter files from workflow deliveries."""

from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile

from .evaluation import ReportingError, stereo_geometry, validate_output_options
from .filesystem import file_sha256
from .task import TaskError, dump_yaml, load_yaml
from .version import SCHEMA_VERSION, VERSION


PROFILE_KIND = "machine_calib_params_profile"
CALIB_PARAM_OUTPUTS = {
    "vision": Path("vision") / "stereo_imu_calib.yaml",
    "mono": Path("mono_camera") / "usb_mono_calib.yaml",
}
_MACHINE_NAME = re.compile(
    r"^(?P<machine_version>[A-Za-z][A-Za-z0-9]*)-(?P<machine_num>[0-9]+)$")
_TIMESTAMP_FORMAT = "%Y-%m-%d-%H:%M:%S"
_IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def machine_identity(machine_dir):
    """Parse public identity from an exact ``MODEL-NUMBER`` basename."""
    match = _MACHINE_NAME.fullmatch(Path(machine_dir).name)
    if match is None:
        raise TaskError(
            "machine_dir basename must match <machine_version>-<machine_num>; "
            "for example EVT312-02")
    return {
        "machine_version": match.group("machine_version"),
        "machine_num": match.group("machine_num"),
    }


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


def _matrix(value, label):
    if (not isinstance(value, list) or len(value) != 4
            or any(not isinstance(row, list) or len(row) != 4
                   for row in value)):
        raise TaskError("{} must be a 4x4 matrix".format(label))
    return [[_finite(element, "{}[{}][{}]".format(label, row, column))
             for column, element in enumerate(values)]
            for row, values in enumerate(value)]


def _identity():
    return deepcopy(_IDENTITY)


def _load_profile(profile_dir):
    path = Path(profile_dir) / "mechine_base_param.yaml"
    if path.is_symlink() or not path.is_file():
        raise TaskError(
            "machine calib_params profile is missing: {}".format(path))
    value = load_yaml(path)
    if (value.get("schema_version") != SCHEMA_VERSION
            or value.get("kind") != PROFILE_KIND
            or set(value) != {"schema_version", "kind", "imu", "odom"}):
        raise TaskError("machine calib_params profile has an invalid schema")
    imu = value.get("imu")
    imu_keys = {
        "imu_type", "imu_data_rate", "arw", "vrw", "gb_std", "ab_std",
        "corr_time", "gravity", "gyro_scale", "acc_scale",
    }
    if not isinstance(imu, dict) or set(imu) != imu_keys:
        raise TaskError("machine calib_params profile imu fields are invalid")
    if not isinstance(imu["imu_type"], str) or not imu["imu_type"]:
        raise TaskError("machine calib_params profile imu_type is invalid")
    rate = _finite(imu["imu_data_rate"], "imu_data_rate", minimum=0.0)
    if rate == 0.0:
        raise TaskError("imu_data_rate must be positive")
    normalized_imu = {
        "imu_type": imu["imu_type"],
        "imu_data_rate": imu["imu_data_rate"],
    }
    for key in ("arw", "vrw", "gb_std", "ab_std", "corr_time", "gravity"):
        _finite(imu[key], key, minimum=0.0)
        normalized_imu[key] = imu[key]
    _vector(imu["gyro_scale"], 3, "gyro_scale")
    _vector(imu["acc_scale"], 3, "acc_scale")
    normalized_imu["gyro_scale"] = list(imu["gyro_scale"])
    normalized_imu["acc_scale"] = list(imu["acc_scale"])

    odom = value.get("odom")
    odom_keys = {
        "w_x_std", "w_y_std", "w_z_std", "w_z_std_vision_off",
        "v_x_std", "v_y_std", "v_z_std", "v_scale", "w_scale",
    }
    if not isinstance(odom, dict) or set(odom) != odom_keys:
        raise TaskError("machine calib_params profile odom fields are invalid")
    normalized_odom = {}
    for key in (
            "w_x_std", "w_y_std", "w_z_std", "w_z_std_vision_off",
            "v_x_std", "v_y_std", "v_z_std", "v_scale", "w_scale"):
        _finite(odom[key], "odom." + key, minimum=0.0)
        normalized_odom[key] = odom[key]
    return {
        "path": path.resolve(),
        "sha256": file_sha256(path),
        "imu": normalized_imu,
        "odom": normalized_odom,
    }


def _json_sha256(value):
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _delivery_source(deliveries, stage, expected_type):
    delivery = deliveries.get(stage)
    if delivery is None:
        return None
    if not isinstance(delivery, dict) or not isinstance(delivery.get("result"), str):
        raise TaskError("{} delivery is invalid".format(stage))
    path = Path(delivery["result"])
    if path.is_symlink() or not path.is_file():
        raise TaskError("{} result is missing: {}".format(stage, path))
    digest = file_sha256(path)
    expected_digest = (delivery.get("files") or {}).get("result")
    if expected_digest is not None and expected_digest != digest:
        raise TaskError("{} result hash changed".format(stage))
    document = load_yaml(path)
    if (document.get("schema_version") != SCHEMA_VERSION
            or document.get("kind") != "calibration_result"
            or document.get("calibration_type") != expected_type):
        raise TaskError("{} result has an unexpected schema".format(stage))
    return {
        "path": path.resolve(),
        "sha256": digest,
        "document": document,
    }


def _camera_map(source, expected_ids, label):
    cameras = source["document"].get("cameras")
    if (not isinstance(cameras, list)
            or any(not isinstance(camera, dict) for camera in cameras)
            or [camera.get("id") for camera in cameras] != expected_ids):
        raise TaskError("{} must contain cameras {} in order".format(
            label, expected_ids))
    return {camera["id"]: camera for camera in cameras}


def _public_model(camera, label):
    projection = camera.get("camera_model")
    distortion = camera.get("distortion_model")
    if projection == "pinhole_opencv_fisheye" and distortion == "opencv_fisheye":
        model, intrinsics, coefficients = "pinhole-opencv-fisheye", 5, 4
    elif projection == "pinhole" and distortion in {
            "equidistant", "radtan", "radtan5", "radtan8"}:
        model = "pinhole-" + distortion
        intrinsics = 4
        coefficients = {
            "equidistant": 4, "radtan": 4, "radtan5": 5, "radtan8": 8,
        }[distortion]
    else:
        raise TaskError("{} has an unsupported camera model".format(label))
    return (
        model,
        _vector(camera.get("intrinsics"), intrinsics, label + ".intrinsics"),
        _vector(camera.get("distortion_coeffs"), coefficients,
                label + ".distortion_coeffs"),
    )


def _camera_parameters(camera, label):
    model, intrinsics, distortion = _public_model(camera, label)
    resolution = camera.get("resolution")
    if (not isinstance(resolution, list) or len(resolution) != 2
            or any(type(value) is not int or value <= 0 for value in resolution)):
        raise TaskError("{}.resolution must be [width, height]".format(label))
    return {
        "model": model,
        "resolution": list(resolution),
        "intrinsic": intrinsics,
        "distortion": distortion,
    }


def _rms(camera, label):
    return _finite(camera.get("rms"), label + ".rms", minimum=0.0)


def _baselink_camera(source, camera_id="cam0"):
    if source is None:
        return None
    cameras = source["document"].get("cameras")
    if not isinstance(cameras, list):
        raise TaskError("Camera-BaseLink result cameras must be a list")
    matches = [camera for camera in cameras
               if isinstance(camera, dict) and camera.get("id") == camera_id]
    if len(matches) != 1:
        raise TaskError("Camera-BaseLink result must contain {}".format(camera_id))
    return matches[0]


def _required_line_delay(camera, label):
    shutter = camera.get("shutter") if isinstance(camera, dict) else None
    if not isinstance(shutter, dict) or "line_delay_s" not in shutter:
        raise TaskError("{} rolling-shutter result lacks line_delay_s".format(label))
    return _finite(shutter["line_delay_s"], label + ".line_delay_s")


def _line_delay(camera, fallback, label, *, camera_imu_available,
                camera_mode, camera_imu_mode):
    if camera_imu_available and camera_imu_mode == "rolling_shutter":
        return _required_line_delay(camera, "Camera-IMU " + label)
    if camera_mode == "rolling_shutter":
        return _required_line_delay(fallback, "camera " + label)
    return 0.0


def _metrics_path(source):
    result = source["path"]
    return (result.parent if result.parent.name == result.stem
            else result.parent / result.stem) / "metrics.json"


def _metrics(source, label):
    path = _metrics_path(source)
    if path.is_symlink() or not path.is_file():
        raise TaskError("{} metrics.json is required".format(label))
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise TaskError("invalid {} metrics.json: {}".format(label, error)) from error
    if (not isinstance(value, dict)
            or value.get("schema_version") != SCHEMA_VERSION
            or value.get("kind") != "calibration_metrics"
            or value.get("calibration_type")
            != source["document"].get("calibration_type")):
        raise TaskError("{} metrics.json has an unexpected schema".format(label))
    return value, {"path": str(path.resolve()), "sha256": file_sha256(path)}


def _biases(metrics):
    result = {}
    for kind, output in (("gyro", "gyro_bias"), ("accel", "acc_bias")):
        spline = (((metrics.get("imus") or {}).get("imu0") or {}).get(kind) or {}).get(
            "bias_spline")
        if not isinstance(spline, dict) or spline.get("status") != "available":
            raise TaskError("imu0 {} bias spline is unavailable".format(kind))
        axes = spline.get("axes")
        if not isinstance(axes, dict):
            raise TaskError("imu0 {} bias spline axes are invalid".format(kind))
        means = []
        for axis in ("x", "y", "z"):
            values = axes.get(axis)
            if not isinstance(values, dict) or values.get("status") != "available":
                raise TaskError("imu0 {} bias {} axis is unavailable".format(kind, axis))
            means.append(_finite(values.get("mean"), "imu0 {} bias {} mean".format(
                kind, axis)))
        result[output] = means
    return result


def _rectification(metrics, label):
    options = metrics.get("evaluation_options")
    if not isinstance(options, dict) or not isinstance(
            options.get("rectification"), dict):
        raise TaskError(
            "{} metrics.json lacks evaluation_options.rectification".format(label))
    try:
        return validate_output_options({
            "rectification": options["rectification"],
        })["rectification"]
    except ReportingError as error:
        raise TaskError("{} rectification is invalid: {}".format(
            label, error)) from error


def _header(identity, generated_at):
    return {
        "machine_version": identity["machine_version"],
        "machine_num": identity["machine_num"],
        "time": generated_at,
        "algorithm_version": VERSION,
        "calib_file_version": SCHEMA_VERSION,
    }


def _baselink_values(source):
    camera = _baselink_camera(source)
    if camera is None:
        return _identity(), 0.0
    transform = _matrix(camera.get("T_base_camera"), "cam0.T_base_camera")
    pitch = _finite(camera.get("pitch"), "cam0.pitch", minimum=0.0)
    if pitch > 90.0:
        raise TaskError("cam0.pitch must not exceed 90 degrees")
    return transform, pitch


def _stereo_correction(cameras, rectification):
    import cv2

    try:
        geometry = stereo_geometry(
            cameras["cam0"], cameras["cam1"], rectification)
    except (ReportingError, cv2.error) as error:
        raise TaskError("stereo rectification failed: {}".format(error)) from error
    return {
        "R1": geometry["R1"].tolist(),
        "P1": geometry["P1"].tolist(),
        "R2": geometry["R2"].tolist(),
        "P2": geometry["P2"].tolist(),
        "baseline": _finite(geometry["baseline_m"], "baseline", minimum=0.0),
        "alignment": _finite(
            cameras["cam1"].get("alignment"),
            "cam1.alignment", minimum=0.0),
    }


def _vision_body(profile, camera_source, camera_imu_source, baselink_source,
                 camera_mode, camera_imu_mode):
    base_cameras = _camera_map(
        camera_source, ["cam0", "cam1"], "stereo result")
    selected_source = camera_imu_source or camera_source
    selected_label = ("Camera-IMU" if camera_imu_source is not None
                      else "camera")
    selected = (_camera_map(camera_imu_source, ["cam0", "cam1"],
                            "Camera-IMU result")
                if camera_imu_source is not None else base_cameras)
    metrics, metric_source = _metrics(selected_source, selected_label)
    rectification = _rectification(metrics, selected_label)
    imu = deepcopy(profile["imu"])
    defaulted = []
    if camera_imu_source is None:
        imu["gyro_bias"] = [0.0, 0.0, 0.0]
        imu["acc_bias"] = [0.0, 0.0, 0.0]
        defaulted.extend(["imu.gyro_bias", "imu.acc_bias"])
    else:
        imus = camera_imu_source["document"].get("imus")
        if (not isinstance(imus, list) or len(imus) != 1
                or not isinstance(imus[0], dict) or imus[0].get("id") != "imu0"):
            raise TaskError("Camera-IMU result must contain imu0")
        rate = _finite(imus[0].get("update_rate"), "imu0.update_rate", minimum=0.0)
        if not math.isclose(rate, float(imu["imu_data_rate"]), rel_tol=0.0,
                            abs_tol=1e-9):
            raise TaskError("Camera-IMU update_rate does not match imu_data_rate")
        biases = _biases(metrics)
        imu["gyro_bias"] = biases["gyro_bias"]
        imu["acc_bias"] = biases["acc_bias"]
    # Preserve the consumer's requested key order around the two bias vectors.
    imu = {
        "imu_type": imu["imu_type"],
        "imu_data_rate": imu["imu_data_rate"],
        "arw": imu["arw"],
        "vrw": imu["vrw"],
        "gb_std": imu["gb_std"],
        "ab_std": imu["ab_std"],
        "corr_time": imu["corr_time"],
        "gravity": imu["gravity"],
        "gyro_bias": imu["gyro_bias"],
        "gyro_scale": imu["gyro_scale"],
        "acc_bias": imu["acc_bias"],
        "acc_scale": imu["acc_scale"],
    }
    imu_values = {}
    for camera_id, source_camera in selected.items():
        if camera_imu_source is None:
            imu_values[camera_id] = {
                "T_imu_cam": _identity(),
                "td_cam_to_imu": 0.0,
            }
            defaulted.extend([
                "camera.{}.td_cam_to_imu".format(camera_id),
                "camera.{}.T_imu_cam".format(camera_id),
            ])
        else:
            imu_values[camera_id] = {
                "T_imu_cam": _matrix(
                    source_camera.get("T_imu_cam"),
                    camera_id + ".T_imu_cam"),
                "td_cam_to_imu": _finite(
                    source_camera.get("timeshift_cam_imu"),
                    camera_id + ".timeshift_cam_imu"),
            }
        imu_values[camera_id]["line_delay_s"] = _line_delay(
            source_camera, base_cameras[camera_id], camera_id,
            camera_imu_available=camera_imu_source is not None,
            camera_mode=camera_mode, camera_imu_mode=camera_imu_mode)

    base_transform, pitch = _baselink_values(baselink_source)
    if baselink_source is None:
        defaulted.extend(["camera.cam0.T_base_cam", "camera.cam0.pitch"])

    cam0 = _camera_parameters(selected["cam0"], "cam0")
    cam0.update({
        "T_base_cam": base_transform,
        "T_imu_cam": imu_values["cam0"]["T_imu_cam"],
        "td_cam_to_imu": imu_values["cam0"]["td_cam_to_imu"],
        "line_delay_s": imu_values["cam0"]["line_delay_s"],
        "pitch": pitch,
        "rms": _rms(selected["cam0"], "cam0"),
    })
    cam1 = _camera_parameters(selected["cam1"], "cam1")
    cam1.update({
        "T_cn_cnm1": _matrix(
            selected["cam1"].get("T_cn_cnm1"), "cam1.T_cn_cnm1"),
        "T_imu_cam": imu_values["cam1"]["T_imu_cam"],
        "td_cam_to_imu": imu_values["cam1"]["td_cam_to_imu"],
        "line_delay_s": imu_values["cam1"]["line_delay_s"],
        "rms": _rms(selected["cam1"], "cam1"),
    })
    return {
        "imu": imu,
        "camera": {
            "cam0": cam0,
            "cam1": cam1,
            "cam0_cam1_correct": _stereo_correction(
                selected, rectification),
        },
        "odom": deepcopy(profile["odom"]),
    }, defaulted, metric_source


def _mono_body(camera_source, baselink_source):
    camera = _camera_map(camera_source, ["cam0"], "mono result")["cam0"]
    public = _camera_parameters(camera, "camera")
    transform, pitch = _baselink_values(baselink_source)
    defaulted = []
    if baselink_source is None:
        defaulted.extend(["camera.T_base_cam", "camera.pitch"])
    public.update({
        "T_base_cam": transform,
        "pitch": pitch,
        "rms": _rms(camera, "camera"),
    })
    return {"camera": public}, defaulted


def _source_record(source):
    return {"path": str(source["path"]), "sha256": source["sha256"]}


def _timestamp(value):
    if not isinstance(value, str):
        raise TaskError("generated_at must be a timestamp string")
    try:
        datetime.strptime(value, _TIMESTAMP_FORMAT)
    except ValueError as error:
        raise TaskError("generated_at must use YYYY-MM-DD-HH:MM:SS") from error
    return value


def _artifact(identity, generated_at, profile, body, defaulted,
              source_files, relative_path, signature_context=None):
    document = _header(identity, generated_at)
    document.update(body)
    defaulted = sorted(defaulted)
    signature = _json_sha256({
        "format_version": SCHEMA_VERSION,
        "identity": identity,
        "profile_sha256": profile["sha256"],
        "sources": source_files,
        "defaulted_fields": defaulted,
        "context": signature_context,
    })
    return {
        "relative_path": relative_path.as_posix(),
        "document": document,
        "defaulted_fields": defaulted,
        "profile": {
            "path": str(profile["path"]),
            "sha256": profile["sha256"],
        },
        "sources": source_files,
        "source_signature": signature,
    }


def build_calib_params(machine_dir, profile_dir, deliveries, generated_at,
                       *, camera_mode="standard",
                       camera_imu_mode="standard"):
    """Return active parameter documents derivable from current deliveries.

    Missing downstream stages intentionally produce numeric defaults.  A
    completed downstream stage with missing evidence instead raises an error.
    """
    identity = machine_identity(machine_dir)
    profile = _load_profile(profile_dir)
    generated_at = _timestamp(generated_at)
    if not isinstance(deliveries, dict):
        raise TaskError("workflow deliveries must be a mapping")
    for label, mode in (("camera_mode", camera_mode),
                        ("camera_imu_mode", camera_imu_mode)):
        if mode not in {"standard", "rolling_shutter"}:
            raise TaskError("{} must be standard or rolling_shutter".format(label))
    sources = {
        "camera": _delivery_source(deliveries, "camera", "cameras"),
        "camera_imu": _delivery_source(deliveries, "camera_imu", "camera_imu"),
        "camera_baselink": _delivery_source(
            deliveries, "camera_baselink", "cameras_baselink"),
        "mono_camera": _delivery_source(deliveries, "mono_camera", "cameras"),
        "mono_camera_baselink": _delivery_source(
            deliveries, "mono_camera_baselink", "cameras_baselink"),
    }
    artifacts = {}
    if sources["camera"] is not None:
        body, defaulted, metric_source = _vision_body(
            profile, sources["camera"], sources["camera_imu"],
            sources["camera_baselink"], camera_mode, camera_imu_mode)
        source_files = {"camera": _source_record(sources["camera"])}
        for key in ("camera_imu", "camera_baselink"):
            if sources[key] is not None:
                source_files[key] = _source_record(sources[key])
        metric_name = ("camera_imu_metrics" if sources["camera_imu"] is not None
                       else "camera_metrics")
        source_files[metric_name] = metric_source
        artifacts["vision"] = _artifact(
            identity, generated_at, profile, body, defaulted,
            source_files, CALIB_PARAM_OUTPUTS["vision"], {
                "artifact_format": "combined_vision_v1",
                "camera_mode": camera_mode,
                "camera_imu_mode": camera_imu_mode,
            })
    if sources["mono_camera"] is not None:
        body, defaulted = _mono_body(
            sources["mono_camera"], sources["mono_camera_baselink"])
        source_files = {
            "mono_camera": _source_record(sources["mono_camera"]),
        }
        if sources["mono_camera_baselink"] is not None:
            source_files["mono_camera_baselink"] = _source_record(
                sources["mono_camera_baselink"])
        artifacts["mono"] = _artifact(
            identity, generated_at, profile, body, defaulted,
            source_files, CALIB_PARAM_OUTPUTS["mono"], {
                "artifact_format": "usb_mono_v1",
            })
    return artifacts


def write_calib_param(output_dir, artifact):
    """Atomically write and read back one artifact beneath ``calib_params``."""
    if (not isinstance(artifact, dict)
            or not isinstance(artifact.get("relative_path"), str)
            or not isinstance(artifact.get("document"), dict)):
        raise TaskError("calib_params artifact is invalid")
    relative = Path(artifact["relative_path"])
    if relative.is_absolute() or ".." in relative.parts:
        raise TaskError("calib_params relative path escapes its root")
    root = Path(output_dir) / "calib_params"
    if root.is_symlink():
        raise TaskError("calib_params output must not be a symlink: {}".format(root))
    root.mkdir(parents=True, exist_ok=True)
    current = root
    for part in relative.parent.parts:
        current = current / part
        if current.is_symlink():
            raise TaskError("calib_params output must not contain symlinks: {}".format(
                current))
        if current.exists() and not current.is_dir():
            raise TaskError("calib_params output parent must be a directory: {}".format(
                current))
        current.mkdir(exist_ok=True)
    target = root / relative
    if target.is_symlink() or target.exists() and not target.is_file():
        raise TaskError("calib_params target must be a regular file: {}".format(target))
    candidate = None
    try:
        with tempfile.NamedTemporaryFile(
                dir=target.parent, prefix="." + target.name + ".",
                delete=False) as stream:
            candidate = Path(stream.name)
        dump_yaml(artifact["document"], candidate)
        if load_yaml(candidate) != artifact["document"]:
            raise TaskError(
                "calib_params YAML read-back changed semantics: {}".format(
                    target))
        os.replace(candidate, target)
        candidate = None
        descriptor = os.open(str(target.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if candidate is not None and candidate.exists():
            candidate.unlink()
    return {"path": str(target.resolve()), "sha256": file_sha256(target)}
