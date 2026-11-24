"""Resolve machine templates and construct ordinary calibration tasks."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import hashlib
import json
import os

from ..filesystem import file_sha256 as _sha256
from ..task.document import (
    TaskError, canonical_document, load_yaml, resolve_task_path,
)
from ..task.execution import resolve_execution
from ..task.jobs import CAMERA_BASELINK_JOB, CAMERA_IMU_RS_JOB, CAMERA_RS_JOB
from ..task.validation import IMU_MODELS, load_task
from ..version import (
    CALIBRATION_BEHAVIOR_VERSION,
    CONFIG_SCHEMA_VERSION,
    SUPPORTED_CONFIG_SCHEMA_VERSIONS,
    VERSION,
    supports_schema_version,
)
from .config import (
    _MACHINE_CAMERA_MODELS, _RECALIBRATION_STAGES, _STAGE_ROLES, _TEMPLATES,
)

try:
    from .._build_info import (
        BUILD_FINGERPRINT, BUILD_TYPE, GIT_COMMIT, PROFILING_ENABLED,
        SOURCE_FINGERPRINT, SOURCE_VARIANT,
    )
except ImportError:
    BUILD_FINGERPRINT = None
    BUILD_TYPE = None
    GIT_COMMIT = None
    PROFILING_ENABLED = None
    SOURCE_FINGERPRINT = None
    SOURCE_VARIANT = "project"


def _profile_directory(prefix, template):
    machine_config = Path(prefix).resolve() / "config" / "mechine"
    path = machine_config / template
    if machine_config.is_symlink() or path.is_symlink():
        raise TaskError(
            "installed machine configuration must not use symlinks: {}"
            .format(machine_config))
    if not path.is_dir():
        raise TaskError("installed machine workflow profile is missing: {}".format(path))
    missing = [name for name in _TEMPLATES.values()
               if not (path / name).is_file()]
    for name in ("实验室_aprilgrid_8x10.yaml", "imu_noise.yaml",
                 "mechine_base_param.yaml", "initialization_stereo_camera.yaml",
                 "initialization_mono_camera.yaml"):
        if not (path / name).is_file():
            missing.append(name)
    if missing:
        raise TaskError("machine workflow profile is incomplete: {}".format(
            ", ".join(sorted(set(missing)))))
    return path


def _target_file(prefix, profile, target_stem):
    if target_stem is None:
        return None
    root = Path(profile).resolve()
    path = root / (target_stem + ".yaml")
    if path.is_symlink() or not path.is_file():
        raise TaskError(
            "installed machine calibration target is missing or not a regular file: {}"
            .format(path))
    from ..task.validation import load_target
    load_target({"_config_dir": str(path.parent),
                 "target": {"path": str(path)}})
    return path.resolve()


def _profile_relative_path(profile, path):
    return Path(os.path.relpath(path, profile)).as_posix()


def _profile_reference(profile, relative, label):
    """Resolve a task-relative YAML path inside one machine template."""
    if not isinstance(relative, str) or not relative.strip():
        raise TaskError("{} must be a non-empty relative path".format(label))
    value = Path(relative)
    if value.is_absolute():
        raise TaskError("{} must be relative to its task template".format(label))
    profile = Path(profile).resolve()
    config_root = profile
    lexical = profile
    for part in value.parts:
        if part in {"", "."}:
            continue
        lexical = lexical.parent if part == ".." else lexical / part
        try:
            lexical.relative_to(config_root)
        except ValueError as error:
            raise TaskError(
                "{} must stay inside {}".format(label, config_root)) from error
        if lexical.is_symlink():
            raise TaskError(
                "machine configuration path must not contain a symlink: {}"
                .format(lexical))
    resolved = lexical.resolve()
    try:
        relative_to_root = resolved.relative_to(config_root)
    except ValueError as error:
        raise TaskError(
            "{} must stay inside {}".format(label, config_root)) from error
    current = config_root
    for part in relative_to_root.parts:
        current = current / part
        if current.is_symlink():
            raise TaskError(
                "machine configuration path must not contain a symlink: {}"
                .format(current))
    if not resolved.is_file():
        raise TaskError("{} is not a file: {}".format(label, resolved))
    return resolved


def _active_template_names(config):
    templates = {
        "camera": _TEMPLATES[
            "camera" if config["camera_mode"] == "standard"
            else "camera_rolling_shutter"],
        "camera_imu": _TEMPLATES[
            "camera_imu_" + config["camera_imu_mode"]],
        "camera_baselink": _TEMPLATES["camera_baselink"],
        "mono_camera": _TEMPLATES["mono_camera"],
        "mono_camera_baselink": _TEMPLATES["mono_camera_baselink"],
    }
    return {stage: name for stage, name in templates.items()
            if not config.get("reuse_calib_params", {}).get(stage, False)}


def _effective_target_paths(profile, overrides, config):
    targets = {}
    for stage, template_name in _active_template_names(config).items():
        selected = overrides.get(stage)
        if selected is not None:
            targets[stage] = Path(selected).resolve()
            continue
        task = load_yaml(profile / template_name)
        block = task.get("target")
        if not isinstance(block, dict) or not block.get("path"):
            raise TaskError(
                "machine {} template must define target.path".format(stage))
        targets[stage] = _profile_reference(
            profile, block["path"], "{}.target.path".format(stage))
    return targets


def _profile_dependencies(profile, targets, config):
    dependencies = {}

    def add(path, stages):
        name = _profile_relative_path(profile, path)
        dependencies.setdefault(name, set()).update(stages)

    for stage, template_name in _active_template_names(config).items():
        stages = _RECALIBRATION_STAGES[_STAGE_ROLES[stage]]
        template_path = profile / template_name
        add(template_path, stages)
        task = load_yaml(template_path)
        add(targets[stage], stages)
        references = []
        references.append(("initialization.path", task.get("initialization")))
        references.extend(
            ("imus[{}].path".format(index), imu)
            for index, imu in enumerate(task.get("imus", [])))
        for label, block in references:
            if isinstance(block, dict) and block.get("path"):
                add(_profile_reference(
                    profile, block["path"], label), stages)
    return {name: [stage for stage in _STAGE_ROLES if stage in stages]
            for name, stages in sorted(dependencies.items())}


def _profile_target_record(profile, path):
    return {
        "path": _profile_relative_path(profile, path),
        "sha256": _semantic_yaml_sha256(path),
    }


def _semantic_yaml_sha256(path):
    payload = json.dumps(
        canonical_document(load_yaml(path)), ensure_ascii=False,
        sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _profile_provenance(profile, target_overrides, config):
    targets = _effective_target_paths(profile, target_overrides, config)
    files = {}
    semantic_files = {}
    config_root = profile
    for path in sorted(config_root.rglob("*.yaml")):
        if path.is_symlink() or not path.is_file():
            raise TaskError("profile YAML must be a regular file: {}".format(path))
        files[_profile_relative_path(profile, path)] = _sha256(path)
        semantic_files[_profile_relative_path(profile, path)] = (
            _semantic_yaml_sha256(path))
    return {
        "version": profile.name,
        "files": files,
        "semantic_files": semantic_files,
        "dependencies": _profile_dependencies(profile, targets, config),
        "targets": {
            stage: _profile_target_record(profile, path)
            for stage, path in targets.items()
        },
        "target_overrides": sorted(target_overrides),
    }


def _software_provenance():
    implementation = hashlib.sha256()
    package = Path(__file__).resolve().parent.parent
    paths = [*package.joinpath("machine").glob("*.py"),
             *package.joinpath("task").glob("*.py"),
             package / "results" / "delivery.py"]
    for path in sorted(paths):
        implementation.update(path.relative_to(package).as_posix().encode("utf-8"))
        implementation.update(_sha256(path).encode("ascii"))
    return {
        "name": "kalibr-noros",
        "version": VERSION,
        "calibration_behavior_version": CALIBRATION_BEHAVIOR_VERSION,
        "git_commit": GIT_COMMIT,
        "source_variant": SOURCE_VARIANT,
        "source_fingerprint": SOURCE_FINGERPRINT,
        "build_fingerprint": BUILD_FINGERPRINT,
        "build_type": BUILD_TYPE,
        "profiling_enabled": PROFILING_ENABLED,
        "implementation_sha256": implementation.hexdigest(),
    }


def _absolute_profile_paths(task, profile):
    for key in ("target", "initialization"):
        block = task.get(key)
        if isinstance(block, dict) and block.get("path"):
            block["path"] = str(_profile_reference(
                profile, block["path"], "{}.path".format(key)))
    for index, imu in enumerate(task.get("imus", [])):
        if not isinstance(imu, dict) or not imu.get("path"):
            raise TaskError("template imus[{}].path is required".format(index))
        imu["path"] = str(_profile_reference(
            profile, imu["path"], "imus[{}].path".format(index)))


def build_task(profile_dir, stage, dataset, output_name,
               camera_result=None, mode="standard", camera_baselink=None,
               camera_mode="standard", recompute_extrinsics=False,
               target_path=None, dataset_type="directory", bag_topics=None):
    """Materialize a normal low-level task from the fixed internal profile."""
    profile = Path(profile_dir).resolve()
    if dataset_type not in {"directory", "bag"}:
        raise TaskError("machine dataset_type must be directory or bag")
    if dataset_type == "bag" and stage in {
            "camera_baselink", "mono_camera_baselink"}:
        raise TaskError("machine BaseLink calibration requires a directory dataset")
    if stage == "camera" and camera_mode in {"standard", "rolling_shutter"}:
        template_key = ("camera" if camera_mode == "standard"
                        else "camera_rolling_shutter")
        expected_job = ("camera_calibration" if camera_mode == "standard"
                        else CAMERA_RS_JOB)
    elif stage == "mono_camera":
        template_key = stage
        expected_job = "camera_calibration"
    elif stage in {"camera_baselink", "mono_camera_baselink"}:
        template_key = stage
        expected_job = CAMERA_BASELINK_JOB
    elif stage == "camera_imu" and mode in {"standard", "rolling_shutter"}:
        template_key = "camera_imu_" + mode
        expected_job = (
            "camera_imu_calibration"
            if mode == "standard" else CAMERA_IMU_RS_JOB)
    else:
        raise TaskError("unsupported machine workflow stage or mode")
    task = deepcopy(load_yaml(profile / _TEMPLATES[template_key]))
    if not supports_schema_version(
            task.get("schema_version"), SUPPORTED_CONFIG_SCHEMA_VERSIONS):
        raise TaskError("machine task template has an invalid schema_version")
    task["schema_version"] = CONFIG_SCHEMA_VERSION
    if task.get("kind") != "calibration_task" or task.get("job") != expected_job:
        raise TaskError("machine task template has an unexpected kind or job")
    if stage in {"camera", "mono_camera"}:
        camera_ids = ["cam0"] if stage == "mono_camera" else ["cam0", "cam1"]
        cameras = task.get("cameras")
        if (not isinstance(cameras, list)
                or any(not isinstance(camera, dict) for camera in cameras)
                or [camera.get("id") for camera in cameras] != camera_ids
                or any(camera.get("model") not in _MACHINE_CAMERA_MODELS
                       for camera in cameras)):
            raise TaskError(
                "machine {} template must define {} in order using {}"
                .format(stage, camera_ids,
                        ", ".join(sorted(_MACHINE_CAMERA_MODELS))))
        if (expected_job == CAMERA_RS_JOB
                and set(task.get("rolling_shutter", {})) != {"cam0", "cam1"}):
            raise TaskError(
                "machine rolling-shutter camera template must define cam0 and cam1")
    elif stage == "camera_imu":
        imus = task.get("imus")
        if (not isinstance(imus, list) or len(imus) != 1
                or not isinstance(imus[0], dict)
                or imus[0].get("id") != "imu0"
                or imus[0].get("model", "calibrated") not in IMU_MODELS):
            raise TaskError(
                "machine Camera-IMU template must define imu0 using one of: {}"
                .format(", ".join(sorted(IMU_MODELS))))
        if mode == "rolling_shutter" and set(
                task.get("rolling_shutter", {})) != {"cam0", "cam1"}:
            raise TaskError(
                "machine rolling-shutter template must define cam0 and cam1")
    task["dataset"] = {
        "type": dataset_type, "path": str(Path(dataset).resolve())}
    output = task.setdefault("output", {})
    output["name"] = output_name
    if output.get("save_diagnostics") is not True:
        raise TaskError(
            "machine workflow templates must enable output.save_diagnostics")
    if stage in {"camera", "camera_imu"} and output.get(
            "save_metrics") is not True:
        raise TaskError(
            "machine workflow camera templates must enable output.save_metrics")
    if stage in {"camera_imu", "camera_baselink", "mono_camera_baselink"}:
        if camera_result is None:
            raise TaskError("camera_result is required for the {} stage".format(stage))
        task["camera_calibration"] = {
            "path": str(Path(camera_result).resolve())}
    if dataset_type == "bag":
        expected_ids = (
            ["cam0"] if stage == "mono_camera" else
            ["cam0", "cam1", "imu0"] if stage == "camera_imu" else
            ["cam0", "cam1"])
        if (not isinstance(bag_topics, dict)
                or set(bag_topics) != set(expected_ids)
                or any(not isinstance(bag_topics.get(sensor_id), str)
                       or not bag_topics[sensor_id].strip()
                       for sensor_id in expected_ids)
                or len({bag_topics[sensor_id]
                        for sensor_id in expected_ids}) != len(expected_ids)):
            raise TaskError(
                "machine {} bag_topics must define unique non-empty topics for {}"
                .format(stage, ", ".join(expected_ids)))
        if stage in {"camera", "mono_camera"}:
            for camera in task["cameras"]:
                camera["topic"] = bag_topics[camera["id"]]
        elif stage == "camera_imu":
            task["imus"][0]["rostopic"] = bag_topics["imu0"]
            task["camera_topics"] = {
                camera_id: bag_topics[camera_id]
                for camera_id in ("cam0", "cam1")}
    if stage in {"camera_baselink", "mono_camera_baselink"}:
        if camera_baselink is None:
            raise TaskError("camera_baselink configuration is required")
        task.update(deepcopy(camera_baselink))
        task["base_link"]["rotation_left_correction"] = (
            [[1.0, 0.0, 0.0],
             [0.0, -1.0, 0.0],
             [0.0, 0.0, -1.0]]
            if stage == "camera_baselink" else
            [[-1.0, 0.0, 0.0],
             [0.0, 1.0, 0.0],
             [0.0, 0.0, -1.0]])
    _absolute_profile_paths(task, profile)
    if target_path is not None:
        selected_target = Path(target_path).resolve()
        if selected_target.is_symlink() or not selected_target.is_file():
            raise TaskError(
                "machine calibration target must be a regular file: {}".format(
                    selected_target))
        task["target"] = {"path": str(selected_target)}
    if stage == "camera_imu":
        task.setdefault("calibration", {})[
            "recompute_camera_chain_extrinsics"] = recompute_extrinsics
    return task


def _resolved_task(task_path, expected_job):
    task = load_task(task_path, expected_job)
    document = deepcopy({key: value for key, value in task.items()
                         if not key.startswith("_")})
    document["execution"] = resolve_execution(task.get("execution"), {})
    document["dataset"] = dict(
        task["dataset"], path=str(resolve_task_path(
            task, task["dataset"]["path"])))
    for key in ("target", "camera_calibration"):
        block = document.get(key)
        if isinstance(block, dict) and block.get("path"):
            block["path"] = str(resolve_task_path(task, block["path"]))
        elif isinstance(block, str):
            document[key] = {"path": str(resolve_task_path(task, block))}
    for imu in document.get("imus", []):
        imu["path"] = str(resolve_task_path(task, imu["path"]))
    initialization = document.get("initialization")
    if isinstance(initialization, dict) and initialization.get("path"):
        initialization["path"] = str(
            resolve_task_path(task, initialization["path"]))
    if expected_job == CAMERA_RS_JOB:
        document["solver_backend"] = "system"
    return document
