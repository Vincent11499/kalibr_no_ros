"""Versioned task I/O and algorithm-neutral legacy command adaptation."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import hashlib
import math
import os
import runpy
import shutil
import sys
import tempfile

import yaml

from kalibr_bag_io import detect_dataset_format

from .initialization import (
    InitializationError,
    build_initialization_report,
    canonical_initialization,
    load_initialization,
    validate_task_initialization,
)


TASK_SCHEMA_VERSION = 1
CALIBRATION_RESULT_VERSION = 2
STANDARD_EXECUTION = {
    "detector_processes": 4,
    "optimizer_threads": 4,
    "detector_inflight_per_worker": 2,
    "detector_opencv_threads": 1,
    "profiling_memory_sample_interval_s": 0.25,
}
MANAGED_OUTPUTS = {
    "calibration.yaml",
    "initialization_report.yaml",
    "observability.yaml",
    "results.txt",
    "report.pdf",
    "poses.csv",
    "timing.json",
}
_COMMON_TASK_KEYS = {
    "schema_version",
    "job",
    "dataset",
    "target",
    "calibration",
    "execution",
    "initialization",
}
_JOB_TASK_KEYS = {
    "camera_calibration": {"cameras"},
    "camera_imu_calibration": {"camera_calibration", "imus"},
}


class TaskError(ValueError):
    pass


class _StableDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        # Indent block sequences beneath their mapping key.  PyYAML's default
        # indentless style is valid YAML, but is harder to scan by eye.
        return super().increase_indent(flow, False)


class _FlowSequence(list):
    """Sequence that is rendered in YAML flow style."""


def _represent_float(dumper, value):
    text = format(value, ".17g")
    # Keep integral-valued floats visibly typed as floats without PyYAML's
    # explicit ``!!float`` annotation (for example, write 0.0 instead of 0).
    if math.isfinite(value) and not any(marker in text for marker in ".eE"):
        text += ".0"
    return dumper.represent_scalar("tag:yaml.org,2002:float", text)


def _represent_flow_sequence(dumper, value):
    return dumper.represent_sequence(
        "tag:yaml.org,2002:seq", value, flow_style=True)


_StableDumper.add_representer(float, _represent_float)
_StableDumper.add_representer(_FlowSequence, _represent_flow_sequence)


def _is_numeric_matrix(value):
    if not isinstance(value, list) or not value:
        return False
    if not all(isinstance(row, list) and row for row in value):
        return False
    column_count = len(value[0])
    return all(
        len(row) == column_count
        and all(type(element) in (int, float) for element in row)
        for row in value
    )


def _format_yaml_collections(value):
    """Keep mappings block-oriented while rendering compact numeric data inline."""
    if _is_numeric_matrix(value):
        return [_FlowSequence(row) for row in value]
    if isinstance(value, dict):
        return {
            key: _format_yaml_collections(element)
            for key, element in value.items()
        }
    if isinstance(value, list):
        if value and all(
                not isinstance(element, (dict, list)) for element in value):
            return _FlowSequence(value)
        return [_format_yaml_collections(element) for element in value]
    return value


def load_yaml(path):
    path = Path(path).resolve()
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise TaskError("YAML root must be a mapping: {}".format(path))
    return value


def dump_yaml(value, path):
    path = Path(path)
    with path.open("w", encoding="utf-8") as stream:
        yaml.dump(
            _format_yaml_collections(value),
            stream,
            Dumper=_StableDumper,
            allow_unicode=True,
            default_flow_style=False,
            sort_keys=False,
            width=2147483647,
        )


def load_task(path, expected_job=None):
    path = Path(path).resolve()
    task = load_yaml(path)
    if (type(task.get("schema_version")) is not int
            or task.get("schema_version") != TASK_SCHEMA_VERSION):
        raise TaskError(
            "task schema_version must be {}".format(TASK_SCHEMA_VERSION))
    job = task.get("job")
    if job not in {"camera_calibration", "camera_imu_calibration"}:
        raise TaskError("job must be camera_calibration or camera_imu_calibration")
    if expected_job is not None and job != expected_job:
        raise TaskError("expected job {}, got {}".format(expected_job, job))
    allowed = _COMMON_TASK_KEYS | _JOB_TASK_KEYS[job]
    unknown = sorted(set(task) - allowed)
    if unknown:
        raise TaskError("unknown fields for {}: {}".format(
            job, ", ".join(unknown)))
    if not isinstance(task.get("dataset"), dict) or not task["dataset"].get("path"):
        raise TaskError("dataset.path is required")
    dataset_type = str(task["dataset"].get("type", "")).lower()
    if dataset_type not in {"bag", "directory"}:
        raise TaskError("dataset.type must be bag or directory")
    task["dataset"]["type"] = dataset_type
    if not isinstance(task.get("target"), dict):
        raise TaskError("target mapping is required")
    if "initialization" in task:
        try:
            validate_task_initialization(task["initialization"])
        except InitializationError as error:
            raise TaskError(str(error)) from error
    if job == "camera_calibration":
        cameras = task.get("cameras")
        if not isinstance(cameras, list) or not cameras:
            raise TaskError("cameras must be a non-empty list")
    else:
        camera_calibration = task.get("camera_calibration")
        if not (
            isinstance(camera_calibration, str) and camera_calibration
            or isinstance(camera_calibration, dict)
            and camera_calibration.get("path")
        ):
            raise TaskError("camera_calibration.path is required")
        imus = task.get("imus")
        if not isinstance(imus, list) or not imus:
            raise TaskError("imus must be a non-empty list")
    task["_config_dir"] = str(path.parent)
    return task


def resolve_task_path(task, value):
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path(task["_config_dir"]) / path
    return path.resolve()


def _camera_calibration_ids(task):
    value = task.get("camera_calibration")
    if isinstance(value, dict):
        value = value.get("path")
    if not value:
        raise TaskError("camera_calibration.path is required")
    data = load_yaml(resolve_task_path(task, value))
    if data.get("schema_version") == CALIBRATION_RESULT_VERSION:
        cameras = data.get("cameras")
        if not isinstance(cameras, list) or not cameras:
            raise TaskError("v2 camera calibration contains no cameras")
        return ["cam{}".format(index) for index in range(len(cameras))]
    indices = sorted(
        int(key[3:]) for key in data
        if isinstance(key, str) and key.startswith("cam") and key[3:].isdigit()
    )
    if not indices or indices != list(range(len(indices))):
        raise TaskError("camera calibration must contain contiguous cam0..camN entries")
    return ["cam{}".format(index) for index in indices]


def resolve_initialization(task, expected_job=None, initialization=None,
                           initialization_strategy=None):
    """Resolve CLI-over-task initialization settings and validate the file.

    Task paths are relative to the task YAML.  A CLI path is resolved against
    the caller's current working directory before the legacy command changes it.
    """
    expected_job = expected_job or task.get("job")
    configured = task.get("initialization") or {}
    configured_path = configured.get("path")
    configured_strategy = configured.get("strategy")

    if initialization is not None:
        if not isinstance(initialization, (str, os.PathLike)) or not str(initialization):
            raise TaskError("--initialization requires a non-empty path")
        path = Path(initialization).expanduser().resolve()
        path_origin = "cli"
    elif configured_path is not None:
        path = resolve_task_path(task, configured_path)
        path_origin = "task"
    else:
        path = None
        path_origin = None

    strategy = initialization_strategy
    strategy_origin = "cli" if strategy is not None else None
    if strategy is None:
        strategy = configured_strategy
        if strategy is not None:
            strategy_origin = "task"
    if path is None:
        if strategy is not None:
            raise TaskError("initialization strategy requires an initialization path")
        return None
    if strategy is None:
        strategy = "refine"
        strategy_origin = "default"

    camera_ids = None
    if expected_job == "camera_imu_calibration":
        camera_ids = _camera_calibration_ids(task)
    try:
        document = load_initialization(
            path, expected_job, task, strategy=strategy, camera_ids=camera_ids)
        source_sha256 = _sha256(path)
    except OSError as error:
        raise TaskError(
            "could not hash initialization file {}: {}".format(
                path, error)) from error
    except InitializationError as error:
        raise TaskError(str(error)) from error
    return {
        "path": path,
        "strategy": strategy,
        "document": document,
        "path_origin": path_origin,
        "strategy_origin": strategy_origin,
        "camera_ids": camera_ids,
        "source_sha256": source_sha256,
    }


def _write_initialization_config(initialization, temporary):
    if initialization is None:
        return None
    path = temporary / "initialization.yaml"
    dump_yaml(
        canonical_initialization(
            initialization["document"], initialization["strategy"]),
        path,
    )
    return path


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_initialization_report(initialization, task, temporary):
    if initialization is None:
        return None
    report = build_initialization_report(
        initialization["document"],
        initialization["strategy"],
        task,
        source_path=initialization["path"],
        source_sha256=initialization["source_sha256"],
        path_origin=initialization["path_origin"],
        strategy_origin=initialization["strategy_origin"],
        camera_ids=initialization["camera_ids"],
    )
    path = temporary / "initialization_report.yaml"
    dump_yaml(report, path)
    return path


def prepare_output_directory(path, force=False):
    output = Path(path).expanduser().resolve()
    forbidden = {
        Path("/").resolve(),
        Path.home().resolve(),
        Path.cwd().resolve(),
    }
    if output in forbidden:
        raise TaskError("unsafe output directory: {}".format(output))
    output.mkdir(parents=True, exist_ok=True)
    entries = list(output.iterdir())
    unknown = [entry.name for entry in entries if entry.name not in MANAGED_OUTPUTS]
    if unknown:
        raise TaskError(
            "output directory contains unmanaged entries: {}".format(
                ", ".join(sorted(unknown))
            )
        )
    existing = [entry for entry in entries if entry.name in MANAGED_OUTPUTS]
    if existing and not force:
        raise TaskError("output files already exist; pass --force to replace them")
    for entry in existing:
        if entry.is_dir():
            raise TaskError("managed output path is unexpectedly a directory: {}".format(entry))
        entry.unlink()
    return output

def _target_path(task, temporary):
    target = task.get("target")
    if not isinstance(target, dict):
        raise TaskError("target mapping is required")
    if target.get("path"):
        return resolve_task_path(task, target["path"])
    target_type = target.get("type")
    parameters = target.get("parameters")
    if not target_type or not isinstance(parameters, dict):
        raise TaskError("target requires path or type plus parameters")
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
    execution = resolve_execution(task.get("execution"), overrides)
    for key, option in (
        ("detector_processes", "--detector-processes"),
        ("optimizer_threads", "--optimizer-threads"),
        ("detector_inflight_per_worker", "--detector-inflight-per-worker"),
        ("detector_opencv_threads", "--detector-opencv-threads"),
        ("profiling_memory_sample_interval_s", "--memory-sample-interval"),
    ):
        value = execution.get(key)
        if value is not None:
            arguments.extend([option, str(value)])
    timing_json = overrides.get("timing_json")
    if timing_json:
        arguments.extend(["--timing-json", str(timing_json)])


def resolve_execution(configured=None, overrides=None, standard_defaults=False):
    """Resolve CLI-over-YAML execution values without changing algorithms."""
    configured = dict(configured or {})
    overrides = dict(overrides or {})
    allowed = {
        "parallelism", "detector_processes", "optimizer_threads",
        "detector_inflight_per_worker", "detector_opencv_threads",
        "profiling_memory_sample_interval_s",
    }
    unknown = sorted(set(configured) - allowed)
    if unknown:
        raise TaskError(
            "unknown execution fields: {}".format(", ".join(unknown)))

    cli_common = overrides.get("parallelism")
    yaml_common = configured.get("parallelism")
    result = {}
    for key in ("detector_processes", "optimizer_threads"):
        value = overrides.get(key)
        if value is None:
            value = cli_common
        if value is None:
            value = configured.get(key)
        if value is None:
            value = yaml_common
        if value is None and standard_defaults:
            value = STANDARD_EXECUTION[key]
        if value is not None:
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise TaskError("execution.{} must be a positive integer".format(key))
            result[key] = value

    for key in ("detector_inflight_per_worker", "detector_opencv_threads"):
        value = overrides.get(key)
        if value is None:
            value = configured.get(key, STANDARD_EXECUTION[key])
        if not isinstance(value, int) or isinstance(value, bool) or value < 1:
            raise TaskError("execution.{} must be a positive integer".format(key))
        result[key] = value

    key = "profiling_memory_sample_interval_s"
    value = overrides.get(key)
    if value is None:
        value = configured.get(key, STANDARD_EXECUTION[key])
    if (not isinstance(value, (int, float)) or isinstance(value, bool)
            or value <= 0.0 or value == float("inf")):
        raise TaskError("execution.{} must be a positive finite number".format(key))
    result[key] = float(value)
    return result


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
    _flag(arguments, calibration.get("remove_outliers") is False, "--no-outliers-removal")
    _flag(arguments, calibration.get("final_filtering") is False, "--no-final-filtering")
    _flag(arguments, bool(calibration.get("blake_zisserman")), "--use-blakezisserman")
    _flag(arguments, bool(calibration.get("verbose")), "--verbose")
    _flag(arguments, bool(calibration.get("show_extraction")), "--show-extraction")
    _flag(arguments, bool(calibration.get("export_poses")), "--export-poses")
    if not calibration.get("interactive_report", False):
        arguments.append("--dont-show-report")
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
    if data.get("schema_version") != CALIBRATION_RESULT_VERSION:
        return source
    cameras = data.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise TaskError("v2 camera calibration contains no cameras")
    legacy = {}
    for index, camera in enumerate(cameras):
        item = dict(camera)
        item.pop("id", None)
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
        imu_paths.append(str(resolve_task_path(task, imu["path"])))
        imu_models.append(str(imu.get("model", "calibrated")))
    arguments = [
        "--bag", str(bag), "--target", str(target),
        "--cams", str(_legacy_camchain(task, temporary)),
        "--imu", *imu_paths, "--imu-models", *imu_models,
    ]
    _append_common_dataset(arguments, task)
    calibration = task.get("calibration") or {}
    _append_corner_refinement(arguments, calibration)
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
    _flag(arguments, bool(calibration.get("verbose")), "--verbose")
    _flag(arguments, bool(calibration.get("show_extraction")), "--show-extraction")
    _flag(arguments, bool(calibration.get("extraction_stepping")), "--extraction-stepping")
    _flag(arguments, bool(calibration.get("export_poses")), "--export-poses")
    if not calibration.get("interactive_report", False):
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


def _legacy_cameras_to_v2(data, calibration_type, imus=None):
    cameras = []
    for key in sorted((key for key in data if key.startswith("cam")), key=lambda value: int(value[3:])):
        camera = {"id": key}
        camera.update(data[key])
        cameras.append(camera)
    result = {
        "schema_version": CALIBRATION_RESULT_VERSION,
        "kind": "calibration_result",
        "calibration_type": calibration_type,
        "transform_convention": "p_target = T_target_source * p_source",
        "cameras": cameras,
    }
    if imus is not None:
        result["imus"] = [dict({"id": key}, **imus[key]) for key in sorted(imus)]
    return result


def _collect_outputs(work, output, job, export_poses):
    if job == "camera_calibration":
        camchain = next(work.glob("*-camchain.yaml"))
        result_text = next(work.glob("*-results-cam.txt"))
        report = next(work.glob("*-report-cam.pdf"))
        result = _legacy_cameras_to_v2(load_yaml(camchain), "cameras")
        pose_pattern = "*-poses-cam0.csv"
    else:
        camchain = next(work.glob("*-camchain-imucam.yaml"))
        imu_yaml = next(work.glob("*-imu.yaml"))
        result_text = next(work.glob("*-results-imucam.txt"))
        report = next(work.glob("*-report-imucam.pdf"))
        result = _legacy_cameras_to_v2(
            load_yaml(camchain), "camera_imu", load_yaml(imu_yaml)
        )
        pose_pattern = "*-poses-imucam-imu0.csv"
    poses = next(work.glob(pose_pattern)) if export_poses else None
    dump_yaml(result, output / "calibration.yaml")
    shutil.move(str(result_text), str(output / "results.txt"))
    shutil.move(str(report), str(output / "report.pdf"))
    if export_poses:
        shutil.move(str(poses), str(output / "poses.csv"))
    for name in ("initialization_report.yaml", "observability.yaml"):
        sidecar = work / name
        if sidecar.is_file():
            shutil.move(str(sidecar), str(output / name))


def _read_observability(path):
    path = Path(path)
    if not path.is_file():
        return None
    report = load_yaml(path)
    calibration = report.get("calibration") or {}
    return {
        "status": report.get("status"),
        "quality": report.get("quality"),
        "rank": calibration.get("rank"),
        "columns": calibration.get("columns"),
        "deficiency": calibration.get("deficiency"),
        "operational_rank": calibration.get("operational_rank"),
        "operational_deficiency": calibration.get(
            "operational_deficiency"),
    }


def _finish_initialization_report(path, status, *, error=None,
                                  observability_path=None,
                                  calibration_path=None):
    path = Path(path)
    if not path.is_file():
        return
    report = load_yaml(path)
    report["status"] = status
    if observability_path is not None:
        summary = _read_observability(observability_path)
        if summary is not None:
            report["observability"] = summary
    if calibration_path is not None and Path(calibration_path).is_file():
        report["result"] = {
            "path": "calibration.yaml",
            "sha256": _sha256(calibration_path),
        }
    if error is not None:
        report["failure"] = {
            "type": type(error).__name__,
            "message": str(error),
        }
    dump_yaml(report, path)


def _preserve_diagnostic_sidecars(work, output):
    for name in ("initialization_report.yaml", "observability.yaml"):
        source = Path(work) / name
        destination = Path(output) / name
        if source.is_file() and not destination.exists():
            shutil.move(str(source), str(destination))


def _remove_failed_seeded_outputs(output):
    """Keep diagnostics only when a seeded run does not complete."""
    diagnostic_names = {"initialization_report.yaml", "observability.yaml"}
    for name in MANAGED_OUTPUTS - diagnostic_names:
        path = Path(output) / name
        if path.is_file():
            path.unlink()


def run_task(prefix, config, output_dir, expected_job, force=False, **overrides):
    task = load_task(config, expected_job=expected_job)
    initialization = resolve_initialization(
        task,
        expected_job,
        initialization=overrides.get("initialization"),
        initialization_strategy=overrides.get("initialization_strategy"),
    )
    output = prepare_output_directory(output_dir, force=force)
    with tempfile.TemporaryDirectory(prefix="kalibr-noros-") as temporary_name:
        temporary = Path(temporary_name)
        initialization_config = _write_initialization_config(
            initialization, temporary)
        initialization_report = _write_initialization_report(
            initialization, task, temporary)
        try:
            bag = _dataset_alias(task, temporary)
            target = _target_path(task, temporary)
            if expected_job == "camera_calibration":
                command = "kalibr_calibrate_cameras"
                arguments = _camera_arguments(
                    task, bag, target, overrides, initialization_config)
            else:
                command = "kalibr_calibrate_imu_camera"
                arguments = _imu_arguments(
                    task, bag, target, temporary, overrides,
                    initialization_config)
            _run_legacy(prefix, command, arguments, temporary)
            if initialization is not None:
                observability_path = temporary / "observability.yaml"
                observability = _read_observability(observability_path)
                if observability is None:
                    raise TaskError(
                        "seeded calibration did not produce observability.yaml")
                if observability.get("status") != "full_rank":
                    raise TaskError(
                        "seeded calibration is rank deficient: rank {}/{} "
                        "(deficiency {})".format(
                            observability.get("rank"),
                            observability.get("columns"),
                            observability.get("deficiency"),
                        ))
            export_poses = bool(
                (task.get("calibration") or {}).get("export_poses"))
            _collect_outputs(temporary, output, expected_job, export_poses)
            if initialization_report is not None:
                _finish_initialization_report(
                    output / "initialization_report.yaml",
                    "completed",
                    observability_path=output / "observability.yaml",
                    calibration_path=output / "calibration.yaml",
                )
        except Exception as error:
            if initialization_report is not None:
                observability_path = temporary / "observability.yaml"
                observability = _read_observability(observability_path)
                status = (
                    "failed_rank_deficient"
                    if observability is not None
                    and observability.get("status") in {
                        "rank_deficient", "no_information"
                    }
                    else "failed"
                )
                report_path = (
                    initialization_report
                    if initialization_report.is_file()
                    else output / "initialization_report.yaml"
                )
                _finish_initialization_report(
                    report_path, status, error=error,
                    observability_path=observability_path)
                _preserve_diagnostic_sidecars(temporary, output)
                _remove_failed_seeded_outputs(output)
            raise
    return output
