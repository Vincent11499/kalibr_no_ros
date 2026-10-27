"""Machine-level staged calibration orchestration.

This module deliberately stays above the calibration solvers.  It discovers
well-known directory datasets, materializes ordinary versioned task files and
invokes the existing task runners without changing solver options.
"""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path
import csv
import fcntl
import hashlib
import json
import os
import re
import tempfile

from kalibr_bag_io.directory import DirectoryReader

from .rolling_shutter import (
    CAMERA_RS_JOB,
    JOB as CAMERA_IMU_RS_JOB,
)
from .jobs import CAMERA_BASELINK_JOB
from .filesystem import file_sha256
from .task import (
    TaskError,
    dump_yaml,
    load_task,
    load_yaml,
    resolve_execution,
    resolve_task_path,
)
from .version import SCHEMA_VERSION, VERSION

try:
    from ._build_info import (
        BUILD_FINGERPRINT,
        BUILD_TYPE,
        GIT_COMMIT,
        PROFILING_ENABLED,
        SOURCE_FINGERPRINT,
        SOURCE_VARIANT,
    )
except ImportError:
    BUILD_FINGERPRINT = None
    BUILD_TYPE = None
    GIT_COMMIT = None
    PROFILING_ENABLED = None
    SOURCE_FINGERPRINT = None
    SOURCE_VARIANT = "project"


WORKFLOW_KIND = "machine_calibration_workflow"
STATE_KIND = "machine_calibration_state"
PROFILE_VERSION = "workflow_v1.0.0"
CONTROL_DIRECTORY = ".kalibr-cedarobot"
BASE_REVISION = "__base__"

_DATASET_PATTERNS = {
    "stereo": re.compile(r"^stereo_\d{6}_\d{4}$"),
    "stereo_imu": re.compile(r"^stereo_imu_\d{6}_\d{4}$"),
    "stereo_baselink": re.compile(r"^stereo_base_?link_\d{6}_\d{4}$"),
    "mono": re.compile(r"^usb_\d{6}_\d{4}$"),
    "mono_baselink": re.compile(r"^usb_baselink_\d{6}_\d{4}$"),
}
_TEMPLATES = {
    "camera": "stereo_camera_calibration_task.yaml",
    "camera_rolling_shutter": (
        "stereo_camera_rolling_shutter_calibration_task.yaml"
    ),
    "camera_baselink": "stereo_camera_baselink_calibration_task.yaml",
    "mono_camera": "mono_camera_calibration_task.yaml",
    "mono_camera_baselink": "mono_camera_baselink_calibration_task.yaml",
    "camera_imu_standard": "camera_imu_calibration_task.yaml",
    "camera_imu_rolling_shutter": (
        "camera_imu_rolling_shutter_calibration_task.yaml"
    ),
}
_REVISION_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,31}$")
_TARGET_STEM_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
_DEFAULT_BASE_LINK = {
    "frame_id": "base_link",
    "position_scale_m": 0.01,
}


_sha256 = file_sha256


def _json_sha256(value):
    payload = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _tree_sha256(root):
    root = Path(root)
    digest = hashlib.sha256()
    for path in sorted(
            (item for item in root.rglob("*") if item.is_file()),
            key=lambda item: item.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix()
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(_sha256(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=path.parent,
                prefix="." + path.name + ".", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, ensure_ascii=False, indent=2,
                      allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        descriptor = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def load_workflow_config(path):
    """Load the strict developer-provided machine workflow document."""
    config_path = Path(path).expanduser().resolve()
    value = load_yaml(config_path)
    allowed = {
        "schema_version", "kind", "machine_dir", "datasets",
        "target", "camera", "camera_imu", "camera_baselink",
        "mono_camera_baselink", "revision",
    }
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise TaskError(
            "unknown machine workflow fields: {}".format(
                ", ".join(unknown)))
    if value.get("schema_version") != SCHEMA_VERSION:
        raise TaskError(
            "machine workflow schema_version must be {}".format(
                SCHEMA_VERSION))
    if value.get("kind") != WORKFLOW_KIND:
        raise TaskError("machine workflow kind must be " + WORKFLOW_KIND)

    configured_machine = value.get("machine_dir")
    if not isinstance(configured_machine, str) or not configured_machine.strip():
        raise TaskError("machine_dir must be a non-empty path")
    machine = Path(configured_machine).expanduser()
    if not machine.is_absolute():
        machine = config_path.parent / machine
    machine = machine.resolve()
    if not machine.is_dir():
        raise TaskError("machine_dir is not a directory: {}".format(machine))

    datasets = value.get("datasets", {})
    if not isinstance(datasets, dict):
        raise TaskError("datasets must be a mapping")
    unknown_datasets = sorted(set(datasets) - set(_DATASET_PATTERNS))
    if unknown_datasets:
        raise TaskError(
            "unknown datasets fields: {}".format(
                ", ".join(unknown_datasets)))
    selected = {}
    for role, name in datasets.items():
        if (not isinstance(name, str) or not name
                or Path(name).name != name
                or not _DATASET_PATTERNS[role].fullmatch(name)):
            raise TaskError(
                "datasets.{} must be a matching direct-child directory name".format(
                    role))
        selected[role] = name

    target = value.get("target")
    if target is not None and (
            not isinstance(target, str)
            or not _TARGET_STEM_PATTERN.fullmatch(target)):
        raise TaskError(
            "target must be a safe config/mechine YAML filename stem")

    camera = value.get("camera", {})
    if not isinstance(camera, dict):
        raise TaskError("camera must be a mapping")
    unknown_camera = sorted(set(camera) - {"mode"})
    if unknown_camera:
        raise TaskError("unknown camera fields: {}".format(
            ", ".join(unknown_camera)))
    camera_mode = camera.get("mode", "standard")
    if camera_mode not in {"standard", "rolling_shutter"}:
        raise TaskError("camera.mode must be standard or rolling_shutter")

    camera_imu = value.get("camera_imu")
    if not isinstance(camera_imu, dict):
        raise TaskError("camera_imu must be a mapping")
    unknown_camera_imu = sorted(
        set(camera_imu) - {"mode", "recompute_extrinsics"})
    if unknown_camera_imu:
        raise TaskError(
            "unknown camera_imu fields: {}".format(
                ", ".join(unknown_camera_imu)))
    mode = camera_imu.get("mode")
    if mode not in {"standard", "rolling_shutter"}:
        raise TaskError(
            "camera_imu.mode must be standard or rolling_shutter")
    recompute_extrinsics = camera_imu.get("recompute_extrinsics", False)
    if type(recompute_extrinsics) is not bool:
        raise TaskError("camera_imu.recompute_extrinsics must be a boolean")

    baselinks = {}
    for key, camera_ids in (("camera_baselink", ["cam0", "cam1"]),
                            ("mono_camera_baselink", ["cam0"])):
        if key not in value:
            continue
        from .validation import validate_options
        baselink = deepcopy(value[key])
        if (not isinstance(baselink, dict)
                or not set(baselink) <= {"base_link", "cameras"}
                or "cameras" not in baselink):
            raise TaskError(
                "{} must contain cameras and may contain base_link".format(key))
        baselink.setdefault("base_link", deepcopy(_DEFAULT_BASE_LINK))
        cameras = baselink["cameras"]
        actual_ids = ([camera.get("id") for camera in cameras]
                      if isinstance(cameras, list)
                      and all(isinstance(camera, dict) for camera in cameras)
                      else [])
        valid_ids = ([camera_ids] if key == "mono_camera_baselink"
                     else [["cam0"], camera_ids])
        if actual_ids not in valid_ids:
            expected = "[cam0]" if key == "mono_camera_baselink" else "[cam0] or [cam0, cam1]"
            raise TaskError("{}.cameras must define {} in order".format(
                key, expected))
        # Share the low-level unit, translation and frame-ID contract.
        validate_options({
            "job": CAMERA_BASELINK_JOB,
            "dataset": {"type": "directory", "path": "__DATASET_PATH__"},
            "camera_calibration": {"path": "__CAMERA_CALIBRATION_PATH__"},
            "output": {}, **baselink,
        })
        baselink["base_link"].setdefault("frame_id", "base_link")
        baselinks[key] = baselink

    revision = value.get("revision")
    if revision is not None and (
            not isinstance(revision, str)
            or not _REVISION_PATTERN.fullmatch(revision)):
        raise TaskError(
            "revision must match [A-Za-z0-9][A-Za-z0-9_-]{0,31}")

    canonical = {
        "schema_version": SCHEMA_VERSION,
        "kind": WORKFLOW_KIND,
        "machine_dir": str(machine),
        "datasets": selected,
        "target": target,
        "camera": {"mode": camera_mode},
        "camera_imu": {
            "mode": mode,
            "recompute_extrinsics": recompute_extrinsics,
        },
    }
    if revision is not None:
        canonical["revision"] = revision
    canonical.update(baselinks)
    return {
        "config_path": config_path,
        "machine_dir": machine,
        "datasets": selected,
        "target": target,
        "camera": {"mode": camera_mode},
        "camera_mode": camera_mode,
        "camera_imu": {
            "mode": mode,
            "recompute_extrinsics": recompute_extrinsics,
        },
        "camera_imu_mode": mode,
        "camera_imu_recompute_extrinsics": recompute_extrinsics,
        "camera_baselink": baselinks.get("camera_baselink"),
        "mono_camera_baselink": baselinks.get("mono_camera_baselink"),
        "revision": revision,
        "config_sha256": _json_sha256(canonical),
    }


def discover_dataset(machine_dir, role, selected=None):
    """Return the unique direct-child dataset for ``role`` or ``None``."""
    if role not in _DATASET_PATTERNS:
        raise TaskError("unsupported machine dataset role: {}".format(role))
    machine = Path(machine_dir).resolve()
    pattern = _DATASET_PATTERNS[role]
    if selected is not None:
        if (not isinstance(selected, str) or Path(selected).name != selected
                or not pattern.fullmatch(selected)):
            raise TaskError(
                "selected {} dataset is not a valid directory name".format(
                    role))
        candidate = machine / selected
        if candidate.is_symlink():
            raise TaskError("dataset must not be a symlink: {}".format(candidate))
        if not candidate.is_dir():
            raise TaskError("selected dataset does not exist: {}".format(candidate))
        candidate = candidate.resolve()
        _validate_dataset_role(candidate, role)
        return candidate

    candidates = []
    for candidate in machine.iterdir():
        if not pattern.fullmatch(candidate.name):
            continue
        if candidate.is_symlink():
            raise TaskError("dataset must not be a symlink: {}".format(candidate))
        if candidate.is_dir():
            candidates.append(candidate.resolve())
    candidates.sort(key=lambda item: item.name)
    if not candidates:
        return None
    if len(candidates) > 1:
        raise TaskError(
            "multiple {} datasets found; set datasets.{} in user.yaml: {}".format(
                role, role, ", ".join(item.name for item in candidates)))
    _validate_dataset_role(candidates[0], role)
    return candidates[0]


def _safe_child(root, relative, label, *, directory=False):
    if not isinstance(relative, str) or not relative.strip():
        raise TaskError("{} must be a non-empty relative path".format(label))
    value = Path(relative)
    if value.is_absolute() or ".." in value.parts:
        raise TaskError("{} must stay inside the dataset".format(label))
    path = root
    for part in value.parts:
        path = path / part
        if path.is_symlink():
            raise TaskError("input path must not contain a symlink: {}".format(path))
    resolved = path.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise TaskError("{} escapes the dataset".format(label)) from error
    valid = resolved.is_dir() if directory else resolved.is_file()
    if not valid:
        expected = "directory" if directory else "file"
        raise TaskError("{} is not a {}: {}".format(label, expected, resolved))
    return resolved


def _dataset_files(dataset, manifest):
    files = {_safe_child(dataset, "dataset.yaml", "dataset manifest")}
    for index, camera in enumerate(manifest.get("cameras", [])):
        label = "cameras[{}]".format(index)
        table = _safe_child(dataset, camera.get("timestamps"),
                            label + ".timestamps")
        image_root = _safe_child(dataset, camera.get("images"),
                                 label + ".images", directory=True)
        files.add(table)
        try:
            stream = table.open("r", encoding="utf-8-sig", newline="")
        except OSError as error:
            raise TaskError("could not read {}: {}".format(table, error)) from error
        with stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != ["timestamp_ns", "filename"]:
                raise TaskError(
                    "invalid camera timestamp CSV header: {}".format(table))
            for line, row in enumerate(reader, start=2):
                filename = row.get("filename")
                files.add(_safe_child(
                    image_root, filename,
                    "{} line {} filename".format(table, line)))
    for index, imu in enumerate(manifest.get("imus", [])):
        files.add(_safe_child(
            dataset, imu.get("data"), "imus[{}].data".format(index)))
    return sorted(files, key=lambda item: item.relative_to(dataset).as_posix())


def _validate_dataset_role(dataset, role):
    """Validate the directory manifest and its fixed workflow sensor role."""
    dataset = Path(dataset).resolve()
    _safe_child(dataset, "dataset.yaml", "dataset manifest")
    try:
        reader = DirectoryReader(dataset)
        # Force CSV header/path checks even when a completed result is reused.
        reader.topics()
    except (OSError, ValueError, RuntimeError) as error:
        raise TaskError("invalid directory dataset {}: {}".format(
            dataset, error)) from error
    manifest = reader.manifest
    camera_ids = [camera.get("id") for camera in manifest.get("cameras", [])]
    imu_ids = [imu.get("id") for imu in manifest.get("imus", [])]
    expected_cameras = ["cam0"] if role in {"mono", "mono_baselink"} else ["cam0", "cam1"]
    if camera_ids != expected_cameras:
        raise TaskError(
            "{} dataset must contain cameras {} in order".format(
                role, expected_cameras))
    expected_imus = ["imu0"] if role == "stereo_imu" else []
    if imu_ids != expected_imus:
        raise TaskError(
            "{} dataset must contain IMUs {}".format(role, expected_imus))
    return reader


def dataset_fingerprint(dataset, role):
    """Validate sensor composition and hash every solver-referenced input."""
    dataset = Path(dataset).resolve()
    reader = _validate_dataset_role(dataset, role)
    manifest = reader.manifest

    digest = hashlib.sha256()
    files = _dataset_files(dataset, manifest)
    for path in files:
        relative = path.relative_to(dataset).as_posix()
        file_digest = _sha256(path)
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(file_digest.encode("ascii"))
        digest.update(b"\n")
    return {
        "role": role,
        "path": str(dataset),
        "name": dataset.name,
        "dataset_id": reader.dataset_id,
        "sha256": digest.hexdigest(),
        "file_count": len(files),
    }


def _profile_directory(prefix):
    path = (Path(prefix).resolve() / "share" / "kalibr-noros" / "config"
            / "mechine" / PROFILE_VERSION)
    if not path.is_dir():
        raise TaskError("installed machine workflow profile is missing: {}".format(path))
    missing = [name for name in _TEMPLATES.values()
               if not (path / name).is_file()]
    for name in ("aprilgrid.yaml", "imu.yaml",
                 "initialization_pinhole_equi.yaml"):
        if not (path / name).is_file():
            missing.append(name)
    if missing:
        raise TaskError("machine workflow profile is incomplete: {}".format(
            ", ".join(sorted(set(missing)))))
    return path


def _target_file(prefix, profile, target_stem):
    if target_stem is None:
        path = Path(profile) / "aprilgrid.yaml"
    else:
        root = (Path(prefix).resolve() / "share" / "kalibr-noros"
                / "config" / "mechine")
        path = root / (target_stem + ".yaml")
    if path.is_symlink() or not path.is_file():
        raise TaskError(
            "installed machine calibration target is missing or not a regular file: {}"
            .format(path))
    from .validation import load_target
    load_target({"_config_dir": str(path.parent),
                 "target": {"path": str(path)}})
    return path.resolve()


def _profile_provenance(profile, target):
    files = {}
    for path in sorted(profile.glob("*.yaml")):
        if path.is_symlink() or not path.is_file():
            raise TaskError("profile YAML must be a regular file: {}".format(path))
        files[path.name] = _sha256(path)
    return {
        "version": PROFILE_VERSION,
        "files": files,
        "target": {"name": target.name, "sha256": _sha256(target)},
    }


def _software_provenance():
    implementation = hashlib.sha256()
    for path in sorted({Path(__file__).resolve(),
                        Path(__file__).with_name("task.py").resolve(),
                        Path(__file__).with_name("camera_baselink.py").resolve(),
                        Path(__file__).with_name("delivery.py").resolve()}):
        implementation.update(path.name.encode("utf-8"))
        implementation.update(_sha256(path).encode("ascii"))
    return {
        "name": "kalibr-noros",
        "version": VERSION,
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
            block["path"] = str(_safe_child(
                profile, block["path"], "{}.path".format(key)))
    for index, imu in enumerate(task.get("imus", [])):
        if not isinstance(imu, dict) or not imu.get("path"):
            raise TaskError("template imus[{}].path is required".format(index))
        imu["path"] = str(_safe_child(
            profile, imu["path"], "imus[{}].path".format(index)))


def build_task(profile_dir, stage, dataset, output_name,
               camera_result=None, mode="standard", camera_baselink=None,
               camera_mode="standard", recompute_extrinsics=False,
               target_path=None):
    """Materialize a normal low-level task from the fixed internal profile."""
    profile = Path(profile_dir).resolve()
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
    if task.get("schema_version") != SCHEMA_VERSION:
        raise TaskError("machine task template has an invalid schema_version")
    if task.get("kind") != "calibration_task" or task.get("job") != expected_job:
        raise TaskError("machine task template has an unexpected kind or job")
    if stage in {"camera", "mono_camera"}:
        camera_ids = ["cam0"] if stage == "mono_camera" else ["cam0", "cam1"]
        cameras = task.get("cameras")
        if (not isinstance(cameras, list)
                or any(not isinstance(camera, dict) for camera in cameras)
                or [(camera.get("id"), camera.get("model"))
                    for camera in cameras]
                != [(camera_id, "pinhole-equi") for camera_id in camera_ids]):
            raise TaskError(
                "machine {} template must define {} as pinhole-equi".format(stage, camera_ids))
        if (expected_job == CAMERA_RS_JOB
                and set(task.get("rolling_shutter", {})) != {"cam0", "cam1"}):
            raise TaskError(
                "machine rolling-shutter camera template must define cam0 and cam1")
    elif stage == "camera_imu":
        imus = task.get("imus")
        if (not isinstance(imus, list) or len(imus) != 1
                or imus[0].get("id") != "imu0"
                or imus[0].get("model") != "calibrated"):
            raise TaskError(
                "machine Camera-IMU template must define calibrated imu0")
        if mode == "rolling_shutter" and set(
                task.get("rolling_shutter", {})) != {"cam0", "cam1"}:
            raise TaskError(
                "machine rolling-shutter template must define cam0 and cam1")
    task["dataset"] = {"type": "directory", "path": str(Path(dataset).resolve())}
    output = task.setdefault("output", {})
    output["name"] = output_name
    if output.get("save_diagnostics") is not True:
        raise TaskError(
            "machine workflow templates must enable output.save_diagnostics")
    if stage in {"camera_imu", "camera_baselink", "mono_camera_baselink"}:
        if camera_result is None:
            raise TaskError("camera_result is required for the {} stage".format(stage))
        task["camera_calibration"] = {
            "path": str(Path(camera_result).resolve())}
    if stage in {"camera_baselink", "mono_camera_baselink"}:
        if camera_baselink is None:
            raise TaskError("camera_baselink configuration is required")
        task.update(deepcopy(camera_baselink))
    _absolute_profile_paths(task, profile)
    selected_target = Path(target_path or profile / "aprilgrid.yaml").resolve()
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


def _expected_result_paths(output, name):
    return {
        "result": output / (name + ".yaml"),
        "html": output / (name + ".report.html"),
        "pdf": output / (name + ".report.pdf"),
        "sidecar": output / name,
    }


def _delivery_exists(output, name):
    paths = _expected_result_paths(output, name)
    return any(path.exists() or path.is_symlink() for path in paths.values())


def _validate_delivery(output, name, expected_job, dataset_record,
                       task_path, camera_dependency=None):
    from .delivery import inspect_managed_delivery
    inspected = inspect_managed_delivery(output, name)
    paths = {key: inspected[key] for key in ("result", "html", "pdf", "sidecar")}
    sidecar = paths["sidecar"]

    result = load_yaml(paths["result"])
    expected_type = {
        "camera_calibration": "cameras",
        CAMERA_RS_JOB: "cameras",
        CAMERA_BASELINK_JOB: "cameras_baselink",
    }.get(expected_job, "camera_imu")
    if (result.get("schema_version") != SCHEMA_VERSION
            or result.get("kind") != "calibration_result"
            or result.get("calibration_type") != expected_type):
        raise TaskError("delivery result has an unexpected schema or calibration type")
    result_cameras = result.get("cameras")
    if expected_job == CAMERA_BASELINK_JOB:
        generated_task = load_yaml(task_path)
        camera_ids = [camera.get("id")
                      for camera in generated_task.get("cameras", [])]
    else:
        camera_ids = (["cam0"]
                      if dataset_record["role"] == "mono"
                      else ["cam0", "cam1"])
    if (not isinstance(result_cameras, list)
            or any(not isinstance(camera, dict)
                   for camera in result_cameras)
            or [camera.get("id") for camera in result_cameras]
            != camera_ids):
        raise TaskError("delivery result must contain cameras {} in order".format(camera_ids))
    if expected_type == "camera_imu":
        result_imus = result.get("imus")
        if (not isinstance(result_imus, list)
                or any(not isinstance(imu, dict) for imu in result_imus)
                or [imu.get("id") for imu in result_imus] != ["imu0"]):
            raise TaskError("Camera-IMU result must contain imu0")

    manifest_path = inspected["manifest_path"]
    task_resolved_path = inspected["task_resolved"]
    manifest = inspected["manifest"]
    if (manifest.get("schema_version") != SCHEMA_VERSION
            or manifest.get("kind") != "calibration_run"
            or manifest.get("status") != "completed"
            or manifest.get("job") != expected_job):
        raise TaskError("delivery run manifest is not a completed expected job")
    manifest_dataset = manifest.get("dataset", {})
    if (not isinstance(manifest_dataset, dict)
            or Path(str(manifest_dataset.get("path", ""))).resolve()
            != Path(dataset_record["path"]).resolve()
            or manifest_dataset.get("id") != dataset_record["dataset_id"]):
        raise TaskError("delivery dataset provenance does not match")
    if manifest.get("task_sha256") != _sha256(task_resolved_path):
        raise TaskError("delivery resolved task hash does not match its manifest")
    if load_yaml(task_resolved_path) != _resolved_task(task_path, expected_job):
        raise TaskError("delivery resolved task does not match the generated task")

    if camera_dependency is not None:
        expected_path = str(Path(camera_dependency["path"]).resolve())
        input_documents = manifest.get("input_documents")
        if (not isinstance(input_documents, list)
                or any(not isinstance(entry, dict)
                       for entry in input_documents)):
            raise TaskError(
                "delivery has invalid camera input document provenance")
        matches = [entry for entry in input_documents
                   if entry.get("role") == "camera_calibration"]
        if (len(matches) != 1
                or str(Path(matches[0].get("path", "")).resolve()) != expected_path
                or matches[0].get("sha256") != camera_dependency["sha256"]):
            raise TaskError("delivery references a different camera result")

    return {
        "result": str(paths["result"].resolve()),
        "files": {
            key: _sha256(paths[key]) for key in ("result", "html", "pdf")
        },
        "run_manifest_sha256": _sha256(manifest_path),
        "task_resolved_sha256": _sha256(task_resolved_path),
        "sidecar_sha256": _tree_sha256(sidecar),
    }


def _load_state(path, machine):
    if not path.exists():
        return {
            "schema_version": SCHEMA_VERSION,
            "kind": STATE_KIND,
            "machine_dir": str(machine),
            "runs": {},
        }
    if path.is_symlink() or not path.is_file():
        raise TaskError("workflow state must be a regular file: {}".format(path))
    try:
        state = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise TaskError("invalid workflow state: {}".format(error)) from error
    if (not isinstance(state, dict)
            or state.get("schema_version") != SCHEMA_VERSION
            or state.get("kind") != STATE_KIND
            or state.get("machine_dir") != str(machine)
            or not isinstance(state.get("runs"), dict)):
        raise TaskError("workflow state has an incompatible schema or machine")
    for revision, run in state["runs"].items():
        if (revision != BASE_REVISION
                and (not isinstance(revision, str)
                     or not _REVISION_PATTERN.fullmatch(revision))):
            raise TaskError("workflow state contains an invalid revision")
        if (not isinstance(run, dict)
                or run.get("binding") is not None
                and not isinstance(run.get("binding"), dict)
                or not isinstance(run.get("stages", {}), dict)
                or any(not isinstance(stage, dict)
                       for stage in run.get("stages", {}).values())):
            raise TaskError("workflow state contains an invalid run record")
        binding = run.get("binding")
        if (isinstance(binding, dict)
                and (not isinstance(binding.get("datasets", {}), dict)
                     or not isinstance(binding.get("profile", {}), dict)
                     or not isinstance(binding.get("software", {}), dict))):
            raise TaskError("workflow state contains an invalid binding record")
    return state


@contextmanager
def _machine_lock(control):
    if control.is_symlink():
        raise TaskError("workflow control directory must not be a symlink: {}".format(
            control))
    if control.exists() and not control.is_dir():
        raise TaskError("workflow control path must be a directory: {}".format(
            control))
    control.mkdir(parents=True, exist_ok=True)
    lock_path = control / "workflow.lock"
    if lock_path.is_symlink():
        raise TaskError("workflow lock must not be a symlink: {}".format(lock_path))
    with lock_path.open("a+", encoding="utf-8") as stream:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise TaskError(
                "another kalibr-cedarobot process is using this machine") from error
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def _ensure_managed_directory(path, root):
    """Create a directory while rejecting symlinks in every managed level."""
    root = Path(root)
    path = Path(path)
    try:
        relative = path.relative_to(root)
    except ValueError as error:
        raise TaskError("managed directory escapes its root: {}".format(path)) from error
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise TaskError(
                "managed directory must not contain symlinks: {}".format(current))
        if current.exists() and not current.is_dir():
            raise TaskError("managed path must be a directory: {}".format(current))
        current.mkdir(exist_ok=True)
    return path


def _output_name(dataset_name, stage, mode, revision, *,
                 camera_mode="standard", camera_ids=None):
    if stage == "camera":
        name = dataset_name + (
            "_rs_pinhole_equi" if camera_mode == "rolling_shutter"
            else "_pinhole_equi")
    elif stage == "mono_camera":
        name = dataset_name + "_pinhole_equi"
    elif stage in {"camera_baselink", "mono_camera_baselink"}:
        suffix = "_".join(camera_ids or [])
        name = dataset_name + "_camera_baselink"
        if suffix:
            name += "_" + suffix
        name += "_pinhole_equi"
    elif mode == "rolling_shutter":
        name = dataset_name + "_camera_imu_rs_pinhole_equi"
    else:
        name = dataset_name + "_camera_imu_pinhole_equi"
    if revision is not None:
        name += "_rev_" + revision
    return name


def _binding(config, profile, software, datasets):
    return {
        "revision": config["revision"],
        "camera_mode": config["camera_mode"],
        "camera_imu_mode": config["camera_imu_mode"],
        "camera_imu_recompute_extrinsics": config[
            "camera_imu_recompute_extrinsics"],
        "camera_baselink": config["camera_baselink"],
        "mono_camera_baselink": config["mono_camera_baselink"],
        "profile": profile,
        "software": software,
        "datasets": datasets,
    }


def _merge_binding(existing, current):
    if existing is None:
        return current
    for key in (
            "revision", "camera_mode", "camera_imu_mode",
            "camera_imu_recompute_extrinsics", "profile", "software"):
        if existing.get(key) != current.get(key):
            raise TaskError(
                "workflow provenance changed; set a new revision in user.yaml")
    merged = deepcopy(existing)
    for key in ("camera_baselink", "mono_camera_baselink"):
        old_baselink = existing.get(key)
        new_baselink = current.get(key)
        if old_baselink is not None and old_baselink != new_baselink:
            raise TaskError(
                "{} configuration changed; set a new revision in user.yaml".format(key))
        merged[key] = new_baselink
    old_datasets = existing.get("datasets", {})
    new_datasets = current.get("datasets", {})
    if not isinstance(old_datasets, dict) or not isinstance(new_datasets, dict):
        raise TaskError("workflow state contains invalid dataset provenance")
    for role in _DATASET_PATTERNS:
        old = old_datasets.get(role)
        new = new_datasets.get(role)
        if old is None:
            merged.setdefault("datasets", {})[role] = new
        elif new is None or old != new:
            raise TaskError(
                "{} dataset provenance changed or disappeared; set a new revision in user.yaml".format(
                    role))
    return merged


def _stage_summary(status, result=None, action=None, message=None):
    value = {"status": status, "result": result}
    if action is not None:
        value["action"] = action
    if message is not None:
        value["message"] = message
    return value


def _completed_stage(output, name, expected_job, dataset_record, task_path,
                     stored=None, camera_dependency=None):
    delivery = _validate_delivery(
        output, name, expected_job, dataset_record, task_path,
        camera_dependency=camera_dependency)
    if stored is not None:
        if (stored.get("status") != "completed"
                or stored.get("delivery") != delivery):
            raise TaskError(
                "completed output provenance changed; set a new revision in user.yaml")
    return delivery


def _verify_stage_inputs_unchanged(dataset_record, camera_dependency=None):
    """Reject a solve if any content-addressed input changed while it ran."""
    current_dataset = dataset_fingerprint(
        dataset_record["path"], dataset_record["role"])
    if current_dataset != dataset_record:
        raise TaskError(
            "dataset changed during calibration; keep capture files immutable "
            "and use a new revision")
    if camera_dependency is not None:
        current_camera_hash = _sha256(camera_dependency["path"])
        if current_camera_hash != camera_dependency["sha256"]:
            raise TaskError(
                "camera calibration changed during downstream calibration; "
                "use a new revision")


def _run_stage(*, prefix, runner, output, tasks, state_path, state,
               run_state, stage_name, task, output_name, expected_job,
               dataset_record, camera_dependency=None):
    _ensure_managed_directory(tasks, state_path.parent)
    task_path = tasks / (output_name + ".task.yaml")
    dump_yaml(task, task_path)
    stored = run_state.setdefault("stages", {}).get(stage_name)
    if stored is not None and stored.get("status") == "completed":
        delivery = _completed_stage(
            output, output_name, expected_job, dataset_record, task_path,
            stored=stored, camera_dependency=camera_dependency)
        return delivery, "reused"
    if _delivery_exists(output, output_name):
        if stored is None or stored.get("status") not in {"running", "failed"}:
            raise TaskError(
                "existing output has no matching workflow checkpoint; set a new revision in user.yaml")
        try:
            delivery = _completed_stage(
                output, output_name, expected_job, dataset_record, task_path,
                camera_dependency=camera_dependency)
        except (TaskError, OSError, ValueError) as error:
            raise TaskError(
                "existing output cannot be safely reused; set a new revision in user.yaml: {}".format(
                    error)) from error
        run_state["stages"][stage_name] = {
            "status": "completed", "delivery": delivery}
        _atomic_json(state_path, state)
        return delivery, "reused"

    run_state["stages"][stage_name] = {
        "status": "running", "task": str(task_path),
        "output_name": output_name,
    }
    _atomic_json(state_path, state)
    try:
        runner(prefix, task_path, output, expected_job, force=False)
        _verify_stage_inputs_unchanged(
            dataset_record, camera_dependency=camera_dependency)
        delivery = _completed_stage(
            output, output_name, expected_job, dataset_record, task_path,
            camera_dependency=camera_dependency)
    except Exception as error:
        run_state["stages"][stage_name] = {
            "status": "failed",
            "task": str(task_path),
            "output_name": output_name,
            "failure": {"type": type(error).__name__, "message": str(error)},
        }
        _atomic_json(state_path, state)
        raise
    run_state["stages"][stage_name] = {
        "status": "completed", "delivery": delivery}
    _atomic_json(state_path, state)
    return delivery, "executed"


def _run_task(prefix, config, output, expected_job, force=False):
    if expected_job == CAMERA_BASELINK_JOB:
        from .camera_baselink import run_camera_baselink
        return run_camera_baselink(config, output, force=force)
    from .task import run_task
    return run_task(prefix, config, output, expected_job, force=force)


def run_machine_workflow(prefix, config_path, runner=None):
    """Run or resume the staged calibration for one machine directory."""
    if runner is None:
        runner = _run_task
    config = load_workflow_config(config_path)
    machine = config["machine_dir"]
    output = machine / "result"
    if output.is_symlink():
        raise TaskError("machine result directory must not be a symlink: {}".format(
            output))
    if output.exists() and not output.is_dir():
        raise TaskError("machine result path must be a directory: {}".format(output))
    control = output / CONTROL_DIRECTORY
    profile_dir = _profile_directory(prefix)
    target_path = _target_file(prefix, profile_dir, config["target"])
    profile = _profile_provenance(profile_dir, target_path)
    software = _software_provenance()

    with _machine_lock(control):
        state_path = control / "state.json"
        state = _load_state(state_path, machine)
        datasets = {}
        for role in _DATASET_PATTERNS:
            dataset_path = discover_dataset(machine, role, config["datasets"].get(role))
            datasets[role] = (dataset_fingerprint(dataset_path, role)
                              if dataset_path is not None else None)

        run_key = config["revision"] or BASE_REVISION
        run_state = state["runs"].setdefault(run_key, {
            "user_config_sha256": config["config_sha256"],
            "binding": None,
            "stages": {},
        })
        current_binding = _binding(
            config, profile, software, datasets)
        run_state["binding"] = _merge_binding(
            run_state.get("binding"), current_binding)
        run_state["user_config_sha256"] = config["config_sha256"]
        state["active_revision"] = run_key
        _atomic_json(state_path, state)

        summary = {
            "schema_version": SCHEMA_VERSION,
            "kind": "machine_calibration_workflow_summary",
            "machine_dir": str(machine),
            "output_dir": str(output),
            "revision": config["revision"],
        }
        tasks = control / "tasks" / run_key
        deliveries = {}
        camera_job = (
            "camera_calibration"
            if config["camera_mode"] == "standard" else CAMERA_RS_JOB)
        imu_job = (
            "camera_imu_calibration"
            if config["camera_imu_mode"] == "standard"
            else CAMERA_IMU_RS_JOB)
        for stage, role, expected_job, parent in (
                ("camera", "stereo", camera_job, None),
                ("camera_imu", "stereo_imu", imu_job, "camera"),
                ("camera_baselink", "stereo_baselink", CAMERA_BASELINK_JOB, "camera"),
                ("mono_camera", "mono", "camera_calibration", None),
                ("mono_camera_baselink", "mono_baselink", CAMERA_BASELINK_JOB, "mono_camera")):
            record = datasets[role]
            camera_delivery = deliveries.get(parent)
            baselink_config = config.get(stage) if expected_job == CAMERA_BASELINK_JOB else None
            status = None
            message = None
            if expected_job == CAMERA_BASELINK_JOB and baselink_config is None:
                status = "disabled" if record is None else "blocked_missing_configuration"
                if record is not None:
                    message = (
                        "dataset {} exists, but {} is missing from user.yaml"
                        .format(record["name"], stage))
            elif record is None:
                status = "waiting_for_dataset"
                message = (
                    "no {} dataset found; add it and rerun the same command"
                    .format(role))
            elif parent is not None and camera_delivery is None:
                status = "blocked_missing_camera_calibration"
                message = (
                    "dataset {} exists, but the prerequisite {} stage has no "
                    "completed result; add its dataset and rerun"
                    .format(record["name"], parent))
            if status is not None:
                run_state["stages"][stage] = {"status": status}
                summary[stage] = _stage_summary(status, message=message)
                continue
            camera_dependency = ({
                "path": camera_delivery["result"],
                "sha256": camera_delivery["files"]["result"],
            } if camera_delivery is not None else None)
            camera_ids = ([camera["id"] for camera in baselink_config["cameras"]]
                          if baselink_config is not None else None)
            name = _output_name(
                record["name"], stage, config["camera_imu_mode"],
                config["revision"], camera_mode=config["camera_mode"],
                camera_ids=camera_ids)
            task = build_task(
                profile_dir, stage, record["path"], name,
                camera_result=camera_delivery["result"] if camera_delivery is not None else None,
                mode=config["camera_imu_mode"],
                camera_baselink=baselink_config,
                camera_mode=config["camera_mode"],
                recompute_extrinsics=config[
                    "camera_imu_recompute_extrinsics"],
                target_path=target_path)
            delivery, action = _run_stage(
                prefix=prefix, runner=runner, output=output, tasks=tasks,
                state_path=state_path, state=state, run_state=run_state,
                stage_name=stage, task=task, output_name=name,
                expected_job=expected_job, dataset_record=record,
                camera_dependency=camera_dependency)
            deliveries[stage] = delivery
            summary[stage] = _stage_summary("completed", delivery["result"], action)
        _atomic_json(state_path, state)
        return summary
