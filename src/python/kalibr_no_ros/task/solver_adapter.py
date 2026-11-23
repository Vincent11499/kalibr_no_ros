"""Adapt validated tasks to the existing native solver inputs and results."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import copy
import math
import os
import runpy
import shutil
import statistics
import sys
import traceback

from kalibr_bag_io import detect_dataset_format, open_dataset

from ..version import CONFIG_SCHEMA_VERSION
from .document import TaskError, CALIBRATION_RESULT_VERSION, dump_yaml, load_yaml, require_document_version, resolve_task_path
from .execution import resolve_execution
from .initialization import _camera_freeze_intrinsics, _camera_calibration_ids, _inverse_transform
from .jobs import CAMERA_RS_JOB, CAMERA_IMU_RS_JOB, CAMERA_CALIBRATION_JOBS
from .rolling_shutter_config import CAMERA_RS_MODELS, NATIVE_CAMERA_RS_MODELS

_NATIVE_RS_CALIBRATION_FIELDS = frozenset({
    "window_half_size_px", "max_displacement_px", "max_iterations",
    "feature_sigma_px", "freeze_intrinsics",
})


def _target_path(task, temporary):
    target = task.get("target")
    if not isinstance(target, dict):
        raise TaskError("target mapping is required")
    if target.get("path"):
        from .validation import load_target
        document = load_target(task)
        document.pop("schema_version")
        document.pop("kind", None)
        path = temporary / "target.yaml"
        dump_yaml(document, path)
        return path
    target_type = target.get("type")
    parameters = target.get("parameters")
    if not target_type or not isinstance(parameters, dict):
        raise TaskError("target requires path or type plus parameters")
    from .validation import load_target
    load_target(task)
    legacy = {"target_type": target_type}
    legacy.update(parameters)
    path = temporary / "target.yaml"
    dump_yaml(legacy, path)
    return path


def _dataset_alias(task, temporary):
    source = resolve_task_path(task, task["dataset"]["path"])
    if not source.exists():
        raise TaskError("dataset does not exist: {}".format(source))
    try:
        detected = detect_dataset_format(source)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        raise TaskError(str(error)) from error
    requested = task["dataset"]["type"]
    compatible = (
        detected in {"ros1", "ros2"}
        if requested == "bag"
        else detected == "directory"
    )
    if not compatible:
        raise TaskError(
            "dataset type mismatch: requested {}, detected {}".format(
                requested, detected))
    suffix = source.suffix if source.is_file() else ""
    alias = temporary / ("dataset" + suffix)
    alias.symlink_to(source, target_is_directory=source.is_dir())
    return alias


def _append_common_dataset(arguments, task):
    dataset = task["dataset"]
    interval = dataset.get("time_range_s")
    if interval is not None:
        if not isinstance(interval, list) or len(interval) != 2:
            raise TaskError("dataset.time_range_s must contain [start, end]")
        arguments.extend(["--bag-from-to", str(interval[0]), str(interval[1])])
    if dataset.get("frequency_hz") is not None:
        arguments.extend(["--bag-freq", str(dataset["frequency_hz"])])


def _append_execution(arguments, task, overrides):
    from kalibr_runtime import profiling_enabled

    execution = resolve_execution(task.get("execution"), overrides)
    for key, option in (
        ("detector_processes", "--detector-processes"),
        ("optimizer_threads", "--optimizer-threads"),
        ("detector_inflight_per_worker", "--detector-inflight-per-worker"),
        ("detector_opencv_threads", "--detector-opencv-threads"),
    ):
        value = execution.get(key)
        if value is not None:
            arguments.extend([option, str(value)])
    timing_json = overrides.get("timing_json")
    if timing_json:
        if not profiling_enabled():
            raise TaskError("timing output requires the project-profile build")
        arguments.extend(["--timing-json", str(timing_json)])
        interval = execution.get("profiling_memory_sample_interval_s")
        if interval is not None:
            arguments.extend(["--memory-sample-interval", str(interval)])


def _flag(arguments, enabled, name):
    if enabled:
        arguments.append(name)


def _append_corner_refinement(arguments, calibration):
    window = calibration.get("window_half_size_px")
    if window is not None:
        if type(window) is not int or window < 1:
            raise TaskError(
                "calibration.window_half_size_px must be a positive integer")
        arguments.extend(["--window-half-size-px", str(window)])

    displacement = calibration.get("max_displacement_px")
    if displacement is not None:
        if (not isinstance(displacement, (int, float))
                or isinstance(displacement, bool)
                or displacement <= 0.0
                or not math.isfinite(displacement)):
            raise TaskError(
                "calibration.max_displacement_px must be a positive finite number")
        arguments.extend(["--max-displacement-px", str(displacement)])


def _append_focal_initialization(arguments, calibration):
    ratio = calibration.get(
        "focal_initialization_min_visible_corner_ratio")
    if ratio is None:
        return
    if (not isinstance(ratio, (int, float))
            or isinstance(ratio, bool)
            or ratio <= 0.0
            or ratio > 1.0
            or not math.isfinite(ratio)):
        raise TaskError(
            "calibration.focal_initialization_min_visible_corner_ratio "
            "must be a finite number in (0, 1]")
    arguments.extend([
        "--focal-initialization-min-visible-corner-ratio", str(ratio)])


def _camera_arguments(task, bag, target, overrides, initialization_config=None):
    cameras = task.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise TaskError("cameras must be a non-empty list")
    if task.get("dataset", {}).get("type") == "directory":
        from .validation import load_cameras
        cameras = load_cameras(dict(task, job="camera_calibration"))
    topics = []
    models = []
    for index, camera in enumerate(cameras):
        if not isinstance(camera, dict) or not camera.get("topic") or not camera.get("model"):
            raise TaskError("camera {} requires topic and model".format(index))
        topics.append(str(camera["topic"]))
        models.append(str(camera["model"]))
    arguments = [
        "--bag", str(bag), "--target", str(target),
        "--topics", *topics, "--models", *models,
    ]
    _append_common_dataset(arguments, task)
    calibration = task.get("calibration") or {}
    freeze_intrinsics = _camera_freeze_intrinsics(task)
    if freeze_intrinsics and initialization_config is None:
        raise TaskError(
            "calibration.freeze_intrinsics: true requires an "
            "initialization path")
    _append_corner_refinement(arguments, calibration)
    _append_focal_initialization(arguments, calibration)
    for key, option in (
        ("synchronization_tolerance_s", "--approx-sync"),
        ("qr_tolerance", "--qr-tol"),
        ("information_gain_tolerance", "--mi-tol"),
        ("min_views_for_outlier_statistics", "--min-views-outlier"),
    ):
        if calibration.get(key) is not None:
            arguments.extend([option, str(calibration[key])])
    _flag(arguments, calibration.get("shuffle") is False, "--no-shuffle")
    arguments.extend([
        "--solver-mode", calibration.get("solver_mode", "incremental")])
    _flag(arguments, calibration.get("remove_outliers") is False, "--no-outliers-removal")
    _flag(arguments, calibration.get("final_filtering") is False, "--no-final-filtering")
    _flag(arguments, bool(calibration.get("blake_zisserman")), "--use-blakezisserman")
    # output.verbose belongs to the managed status channel.  Native --verbose
    # also opens detector/graph windows and changes extraction threading.
    _flag(arguments, bool((task.get("output") or {}).get("show_extraction")), "--show-extraction")
    _flag(arguments, bool((task.get("output") or {}).get("export_poses")), "--export-poses")
    _flag(arguments, freeze_intrinsics, "--freeze-intrinsics")
    if not (task.get("output") or {}).get("interactive_report", False):
        arguments.append("--dont-show-report")
    if initialization_config is not None:
        arguments.extend(["--initialization-config", str(initialization_config)])
    _append_execution(arguments, task, overrides)
    return arguments


def _rolling_shutter_config(task, temporary):
    """Translate public camera IDs to the native solver's positional IDs."""
    from .rolling_shutter_config import validate_shutters

    camera_ids = [
        camera.get("id", "cam{}".format(index))
        for index, camera in enumerate(task["cameras"])
    ]
    shutters = validate_shutters(task["rolling_shutter"], camera_ids)
    path = temporary / "rolling_shutter.yaml"
    dump_yaml(
        {
            "schema_version": CONFIG_SCHEMA_VERSION,
            "kind": "rolling_shutter_config",
            "cameras": {
                "cam{}".format(index): shutters[camera_id]
                for index, camera_id in enumerate(camera_ids)
            },
        },
        path,
    )
    return path, shutters


def _validate_camera_rs_backend(task, solver_backend):
    """Validate command-specific RS constraints without opening the dataset."""
    if solver_backend not in {"system", "native"}:
        raise TaskError(
            "rolling-shutter solver_backend must be system or native")
    if task.get("job") != CAMERA_RS_JOB:
        raise TaskError(
            "rolling-shutter solver_backend is only valid for {}".format(
                CAMERA_RS_JOB))
    if solver_backend == "system":
        return

    cameras = task.get("cameras")
    if not isinstance(cameras, list) or len(cameras) != 1:
        raise TaskError(
            "native-rs-cameras supports exactly one camera; use cameras-rs "
            "for stereo/multicamera")
    if bool((task.get("output") or {}).get("export_poses")):
        raise TaskError(
            "native-rs-cameras does not support output.export_poses; "
            "use cameras-rs when poses.csv is required")
    calibration = task.get("calibration") or {}
    unsupported = sorted(
        set(calibration) - _NATIVE_RS_CALIBRATION_FIELDS)
    if unsupported:
        raise TaskError(
            "native-rs-cameras does not implement these cameras-rs "
            "calibration fields: {}".format(
                ", ".join(unsupported)))


def _rs_task_cameras(task):
    cameras = task.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise TaskError("cameras must be a non-empty list")
    if task.get("dataset", {}).get("type") == "directory":
        from .validation import load_cameras
        cameras = load_cameras(task)
    for index, camera in enumerate(cameras):
        if (not isinstance(camera, dict) or not camera.get("topic")
                or not camera.get("model")):
            raise TaskError("camera {} requires topic and model".format(index))
    return cameras


def _rs_camera_arguments(task, bag, target, temporary, overrides,
                         initialization_config=None):
    """Build the private command line for mono/stereo/multicamera RS solving."""
    cameras = _rs_task_cameras(task)
    models = [str(camera["model"]) for camera in cameras]
    unsupported = sorted(set(models) - CAMERA_RS_MODELS)
    if unsupported:
        raise TaskError(
            "cameras-rs supports {}; got {}".format(
                ", ".join(sorted(CAMERA_RS_MODELS)),
                ", ".join(unsupported)))
    shutter_path, _ = _rolling_shutter_config(task, temporary)
    arguments = [
        "--bag", str(bag), "--target", str(target),
        "--topics", *[str(camera["topic"]) for camera in cameras],
        "--models", *models,
        "--rolling-shutter-config", str(shutter_path),
    ]
    _append_common_dataset(arguments, task)
    calibration = task.get("calibration") or {}
    freeze_intrinsics = _camera_freeze_intrinsics(task)
    if freeze_intrinsics and initialization_config is None:
        raise TaskError(
            "calibration.freeze_intrinsics: true requires an initialization path")
    _append_corner_refinement(arguments, calibration)
    _append_focal_initialization(arguments, calibration)
    for key, option in (
        ("synchronization_tolerance_s", "--approx-sync"),
        ("min_views_for_outlier_statistics", "--min-views-outlier"),
        ("max_iterations", "--max-iterations"),
        ("spline_order", "--spline-order"),
        ("time_padding_s", "--time-padding-s"),
        ("knots_per_second", "--knots-per-second"),
        ("feature_sigma_px", "--feature-sigma-px"),
        ("motion_translation_weight", "--motion-translation-weight"),
        ("motion_rotation_weight", "--motion-rotation-weight"),
    ):
        if calibration.get(key) is not None:
            arguments.extend([option, str(calibration[key])])
    _flag(arguments, calibration.get("remove_outliers") is False,
          "--no-outliers-removal")
    _flag(arguments, calibration.get("final_filtering") is False,
          "--no-final-filtering")
    _flag(arguments, bool(calibration.get("blake_zisserman")),
          "--use-blakezisserman")
    _flag(arguments, bool((task.get("output") or {}).get("show_extraction")),
          "--show-extraction")
    _flag(arguments, bool((task.get("output") or {}).get("export_poses")),
          "--export-poses")
    _flag(arguments, freeze_intrinsics, "--freeze-intrinsics")
    if not (task.get("output") or {}).get("interactive_report", False):
        arguments.append("--dont-show-report")
    if initialization_config is not None:
        arguments.extend(["--initialization-config", str(initialization_config)])
    _append_execution(arguments, task, overrides)
    return arguments


def _selected_frame_rate_hz(task, topic):
    """Estimate the rate of observations actually sent to the native solver."""
    reader = open_dataset(resolve_task_path(task, task["dataset"]["path"]))
    try:
        with reader.index_images(topic, grayscale=True) as dataset:
            records = list(dataset.index)
        interval = task["dataset"].get("time_range_s")
        if interval is not None and records:
            start = records[0].header_timestamp_ns
            lower = start + int(round(float(interval[0]) * 1e9))
            upper = start + int(round(float(interval[1]) * 1e9))
            records = [record for record in records
                       if lower <= record.header_timestamp_ns <= upper]
        frequency = task["dataset"].get("frequency_hz")
        if frequency is not None:
            minimum_delta_ns = 1e9 / float(frequency)
            selected = []
            previous = None
            for record in records:
                if (previous is None
                        or record.header_timestamp_ns - previous >= minimum_delta_ns):
                    selected.append(record)
                    previous = record.header_timestamp_ns
            records = selected
        intervals = [
            current.header_timestamp_ns - previous.header_timestamp_ns
            for previous, current in zip(records, records[1:])
            if current.header_timestamp_ns > previous.header_timestamp_ns
        ]
    finally:
        close = getattr(reader, "close", None)
        if close is not None:
            close()
    if not intervals:
        raise TaskError(
            "native-rs-cameras requires at least two selected timestamps to infer frame rate")
    rate = 1e9 / statistics.median(intervals)
    if not math.isfinite(rate) or rate <= 0.0:
        raise TaskError("could not infer a positive finite source frame rate")
    return float(rate)


def _native_rs_camera_arguments(task, bag, target, temporary, overrides,
                                initialization_config=None):
    """Adapt the shared RS task to Kalibr's native single-camera RS solver."""
    _validate_camera_rs_backend(task, "native")
    cameras = _rs_task_cameras(task)
    camera = cameras[0]
    if camera["model"] not in NATIVE_CAMERA_RS_MODELS:
        raise TaskError(
            "native-rs-cameras supports {}".format(
                ", ".join(sorted(NATIVE_CAMERA_RS_MODELS))))
    _, shutters = _rolling_shutter_config(task, temporary)
    camera_id = task["cameras"][0].get("id", "cam0")
    shutter = shutters[camera_id]
    calibration = task.get("calibration") or {}
    feature_sigma = float(calibration.get("feature_sigma_px", 1.0))
    arguments = [
        "--bag", str(bag), "--target", str(target),
        "--topic", str(camera["topic"]), "--model", str(camera["model"]),
        "--frame-rate", str(_selected_frame_rate_hz(task, camera["topic"])),
        "--inverse-feature-variance", str(feature_sigma * feature_sigma),
        "--line-delay-seed", str(shutter["line_delay_s"]),
    ]
    if "max_abs_line_delay_s" in shutter:
        arguments.extend([
            "--max-abs-line-delay", str(shutter["max_abs_line_delay_s"])])
    _flag(arguments, shutter["estimate"] is False, "--no-estimate-line-delay")
    _append_common_dataset(arguments, task)
    _append_corner_refinement(arguments, calibration)
    arguments.extend([
        "--max-iter", str(calibration.get("max_iterations", 80))])
    _flag(arguments, bool((task.get("output") or {}).get("show_extraction")),
          "--show-extraction")
    _flag(arguments, bool(calibration.get("freeze_intrinsics")),
          "--freeze-intrinsics")
    if initialization_config is not None:
        arguments.extend(["--initialization-config", str(initialization_config)])
    _append_execution(arguments, task, overrides)
    return arguments


def _legacy_camchain(task, temporary):
    value = task.get("camera_calibration")
    if isinstance(value, dict):
        value = value.get("path")
    if not value:
        raise TaskError("camera_calibration.path is required")
    source = resolve_task_path(task, value)
    data = load_yaml(source)
    require_document_version(
        data, "camera calibration", "calibration_result", result=True)
    from .validation import load_cameras
    cameras = load_cameras(task)
    if not isinstance(cameras, list) or not cameras:
        raise TaskError("camera calibration contains no cameras")
    legacy = {}
    for index, camera in enumerate(cameras):
        item = dict(camera)
        item.pop("id", None)
        for field in (
                "rms", "alignment", "rs_compensated_pair_residual",
                "from_camera", "shutter", "line_delay", "T_imu_cam"):
            item.pop(field, None)
        item["rostopic"] = item.pop("topic")
        legacy["cam{}".format(index)] = item
    path = temporary / "camchain.yaml"
    dump_yaml(legacy, path)
    return path


def _imu_arguments(task, bag, target, temporary, overrides,
                   initialization_config=None):
    imus = task.get("imus")
    if not isinstance(imus, list) or not imus:
        raise TaskError("imus must be a non-empty list")
    imu_paths = []
    imu_models = []
    for index, imu in enumerate(imus):
        if not isinstance(imu, dict) or not imu.get("path"):
            raise TaskError("imu {} requires path".format(index))
        from .validation import load_imu
        document = load_imu(task, imu)
        document.pop("schema_version")
        document.pop("kind", None)
        imu_path = temporary / "imu{}.yaml".format(index)
        dump_yaml(document, imu_path)
        imu_paths.append(str(imu_path))
        imu_models.append(str(imu.get("model", "calibrated")))
    arguments = [
        "--bag", str(bag), "--target", str(target),
        "--cams", str(_legacy_camchain(task, temporary)),
        "--imu", *imu_paths, "--imu-models", *imu_models,
    ]
    _append_common_dataset(arguments, task)
    calibration = task.get("calibration") or {}
    _append_corner_refinement(arguments, calibration)
    if task["job"] == CAMERA_IMU_RS_JOB:
        from .rolling_shutter_config import validate_shutters
        ids = _camera_calibration_ids(task)
        shutters = validate_shutters(task["rolling_shutter"], ids)
        path = temporary / "rolling_shutter.yaml"
        dump_yaml({"schema_version": CONFIG_SCHEMA_VERSION, "kind": "rolling_shutter_config",
                   "cameras": {"cam{}".format(i): shutters[key] for i, key in enumerate(ids)}}, path)
        arguments.extend(["--rolling-shutter-config", str(path)])
    for key, option in (
        ("max_iterations", "--max-iter"),
        ("time_offset_padding_s", "--timeoffset-padding"),
        ("reprojection_sigma_px", "--reprojection-sigma"),
    ):
        if calibration.get(key) is not None:
            arguments.extend([option, str(calibration[key])])
    _flag(arguments, bool(calibration.get("synchronize_clocks")), "--perform-synchronization")
    _flag(arguments, bool(calibration.get("estimate_multi_imu_delay")), "--imu-delay-by-correlation")
    _flag(arguments, calibration.get("calibrate_time_offset") is False, "--no-time-calibration")
    _flag(arguments, bool(calibration.get("recover_covariance")), "--recover-covariance")
    _flag(arguments, bool(calibration.get("recompute_camera_chain_extrinsics")), "--recompute-camera-chain-extrinsics")
    _flag(arguments, bool((task.get("output") or {}).get("show_extraction")), "--show-extraction")
    _flag(arguments, bool((task.get("output") or {}).get("extraction_stepping")), "--extraction-stepping")
    _flag(arguments, bool((task.get("output") or {}).get("export_poses")), "--export-poses")
    if not (task.get("output") or {}).get("interactive_report", False):
        arguments.append("--dont-show-report")
    if initialization_config is not None:
        arguments.extend(["--initialization-config", str(initialization_config)])
    _append_execution(arguments, task, overrides)
    return arguments


@contextmanager
def _arguments_and_directory(arguments, directory):
    previous_arguments = sys.argv
    previous_directory = Path.cwd()
    sys.argv = arguments
    os.chdir(str(directory))
    try:
        yield
    finally:
        os.chdir(str(previous_directory))
        sys.argv = previous_arguments


def _run_legacy(prefix, command, arguments, directory):
    script = Path(prefix) / "libexec" / "kalibr" / command
    if not script.is_file():
        raise TaskError("missing internal calibration command: {}".format(script))
    with _arguments_and_directory([command] + arguments, directory):
        try:
            runpy.run_path(str(script), run_name="__main__")
        except SystemExit as error:
            code = error.code if isinstance(error.code, int) else 1
            if code:
                raise TaskError("{} failed with exit code {}".format(command, code))


@contextmanager
def _capture_solver_logs(output, verbose=False):
    """Capture Python, native C++ and detector-worker output, restoring FDs even on failure."""
    import ctypes
    flush_native = ctypes.CDLL(None).fflush
    flush_native.argtypes = [ctypes.c_void_p]
    flush_native.restype = ctypes.c_int
    sys.stdout.flush()
    sys.stderr.flush()
    flush_native(None)
    saved = [os.dup(1), os.dup(2)]
    import kalibr_runtime as native_runtime
    try:
        with (Path(output) / "stdout.log").open("w", encoding="utf-8", buffering=1) as stdout, \
                (Path(output) / "stderr.log").open("w", encoding="utf-8", buffering=1) as stderr:
            os.dup2(stdout.fileno(), 1)
            os.dup2(stderr.fileno(), 2)
            native_runtime.set_status_output(saved[0], verbose=verbose)
            try:
                yield
            except BaseException:
                traceback.print_exc()
                raise
            finally:
                sys.stdout.flush()
                sys.stderr.flush()
                flush_native(None)
    finally:
        native_runtime.clear_status_output()
        for descriptor, original in zip((1, 2), saved):
            os.dup2(original, descriptor)
            os.close(original)


def _native_cameras_to_result(data, calibration_type, imus=None):
    cameras = []
    for key in sorted((key for key in data if key.startswith("cam")), key=lambda value: int(value[3:])):
        camera = {"id": key}
        camera.update(data[key])
        if "T_cam_imu" in camera:
            camera["T_imu_cam"] = _inverse_transform(
                camera["T_cam_imu"], "{}.T_cam_imu".format(key))
        cameras.append(camera)
    result = {
        "schema_version": CALIBRATION_RESULT_VERSION,
        "kind": "calibration_result",
        "calibration_type": calibration_type,
        "cameras": cameras,
    }
    if imus is not None:
        result["imus"] = [dict({"id": key}, **imus[key]) for key in sorted(imus)]
    return result


def _normalize_camera_rs_result(result, task, solver_backend):
    """Enforce the public row-0 timestamp semantics for RS camera outputs."""
    from .rolling_shutter_config import validate_shutters

    camera_ids = [camera.get("id", "cam{}".format(index))
                  for index, camera in enumerate(task["cameras"])]
    configured = validate_shutters(task["rolling_shutter"], camera_ids)
    result_cameras = result.get("cameras")
    if (not isinstance(result_cameras, list)
            or len(result_cameras) != len(camera_ids)):
        raise TaskError(
            "rolling-shutter solver camera count does not match the task")
    for index, camera in enumerate(result_cameras):
        shutter = dict(camera.get("shutter") or {})
        line_delay = shutter.get("line_delay_s", camera.pop("line_delay", None))
        resolution = camera.get("resolution")
        if (isinstance(line_delay, bool)
                or not isinstance(line_delay, (int, float))
                or not math.isfinite(line_delay)):
            raise TaskError(
                "rolling-shutter solver did not publish a finite line_delay_s for {}".format(
                    camera.get("id")))
        if (not isinstance(resolution, list) or len(resolution) != 2
                or any(type(value) is not int or value <= 0 for value in resolution)):
            raise TaskError("rolling-shutter result requires a valid camera resolution")
        configured_shutter = configured[camera_ids[index]]
        shutter.update({
            "type": "rolling_shutter",
            "line_delay_s": float(line_delay),
            "reference_row_px": 0.0,
            "first_to_last_row_span_s": abs(float(line_delay)) * (
                resolution[1] - 1),
            "estimated": bool(configured_shutter["estimate"]),
            "timestamp_reference": "row0_exposure_end",
            "corner_time_equation": (
                "t_corner_s = t_camera_timestamp_s + y_px * line_delay_s"),
        })
        if "max_abs_line_delay_s" in configured_shutter:
            shutter["max_abs_line_delay_s"] = float(
                configured_shutter["max_abs_line_delay_s"])
            shutter["bound_role"] = (
                "post_solve_admissibility"
                if solver_backend == "native"
                else "optimizer_parameterization")
        camera["shutter"] = shutter
    return result


def _overlay_structured_camera_snapshots(result, artifacts):
    """Prefer the managed final-state snapshot over legacy serializer copies."""
    native = artifacts.get("cameras", [])
    public = result.get("cameras", [])
    if len(native) != len(public):
        raise TaskError("native camera result and managed snapshot counts differ")
    for snapshot, camera in zip(native, public):
        for field in ("intrinsics", "distortion_coeffs", "resolution"):
            if field in snapshot:
                camera[field] = copy.deepcopy(snapshot[field])
        if "shutter" in snapshot:
            shutter = dict(camera.get("shutter") or {})
            shutter.update(copy.deepcopy(snapshot["shutter"]))
            line_delay = shutter.get("line_delay_s")
            resolution = camera.get("resolution")
            if (isinstance(line_delay, bool)
                    or not isinstance(line_delay, (int, float))
                    or not math.isfinite(line_delay)
                    or not isinstance(resolution, list)
                    or len(resolution) != 2
                    or any(type(value) is not int or value <= 0
                           for value in resolution)):
                raise TaskError(
                    "managed rolling-shutter snapshot is not finite")
            shutter.update({
                "reference_row_px": 0.0,
                "first_to_last_row_span_s": abs(float(line_delay)) * (
                    resolution[1] - 1),
                "timestamp_reference": "row0_exposure_end",
                "corner_time_equation": (
                    "t_corner_s = t_camera_timestamp_s + y_px * line_delay_s"),
            })
            camera["shutter"] = shutter
        camera.pop("line_delay", None)


def _collect_outputs(work, output, job, export_poses, task=None,
                     rs_solver_backend="system"):
    if job in CAMERA_CALIBRATION_JOBS:
        camchain = next(work.glob("*-camchain.yaml"))
        result_text = next(work.glob("*-results-cam.txt"))
        result = _native_cameras_to_result(load_yaml(camchain), "cameras")
        if job == CAMERA_RS_JOB:
            if task is None:
                raise TaskError("rolling-shutter output collection requires its task")
            result = _normalize_camera_rs_result(
                result, task, rs_solver_backend)
        pose_pattern = "*-poses-cam0.csv"
    else:
        camchain = next(work.glob("*-camchain-imucam.yaml"))
        imu_yaml = next(work.glob("*-imu.yaml"))
        result_text = next(work.glob("*-results-imucam.txt"))
        result = _native_cameras_to_result(
            load_yaml(camchain), "camera_imu", load_yaml(imu_yaml)
        )
        pose_pattern = "*-poses-imucam-imu0.csv"
    poses = next(work.glob(pose_pattern)) if export_poses else None
    dump_yaml(result, output / "calibration.yaml")
    shutil.move(str(result_text), str(output / "results.txt"))
    if export_poses:
        shutil.move(str(poses), str(output / "poses.csv"))
    for name in ("initialization_report.yaml", "observability.yaml"):
        sidecar = work / name
        if sidecar.is_file():
            shutil.move(str(sidecar), str(output / name))
    return result


def _staged_solver_invocation(
        task, expected_job, temporary, overrides, initialization_config,
        rs_backend):
    bag = _dataset_alias(task, temporary)
    target = _target_path(task, temporary)
    if expected_job == "camera_calibration":
        return "kalibr_calibrate_cameras", _camera_arguments(
            task, bag, target, overrides, initialization_config)
    if expected_job == CAMERA_RS_JOB:
        if rs_backend == "native":
            return "kalibr_calibrate_rs_cameras", _native_rs_camera_arguments(
                task, bag, target, temporary, overrides,
                initialization_config)
        return "kalibr_calibrate_rs_camera_system", _rs_camera_arguments(
            task, bag, target, temporary, overrides, initialization_config)
    return "kalibr_calibrate_imu_camera", _imu_arguments(
        task, bag, target, temporary, overrides, initialization_config)
