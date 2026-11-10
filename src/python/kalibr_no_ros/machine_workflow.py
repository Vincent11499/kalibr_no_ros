"""Machine-level staged calibration orchestration.

This module deliberately stays above the calibration solvers.  It discovers
well-known directory datasets, materializes ordinary versioned task files and
invokes the existing task runners without changing solver options.
"""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime
from pathlib import Path
import csv
import fcntl
import hashlib
import json
import os
import re
import tempfile
import time
from zoneinfo import ZoneInfo

from kalibr_bag_io.directory import DirectoryReader

from .rolling_shutter import (
    CAMERA_RS_JOB,
    JOB as CAMERA_IMU_RS_JOB,
)
from .jobs import CAMERA_BASELINK_JOB
from .delivery import MACHINE_LAYOUT
from .filesystem import file_sha256
from .machine_calib_params import (
    build_calib_params,
    machine_identity,
    write_calib_param,
)
from .machine_param_reuse import (
    camera_seed, parameter_path, semantic_sha256, validate_reused_stage,
)
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
CONTROL_DIRECTORY = ".kalibr-cedarobot"
INPUT_HASH_CACHE = "input_hash_cache.json"
INPUT_HASH_CACHE_KIND = "machine_input_hash_cache"
# Test-phase policy: accept cross-run provenance changes and recalibrate the
# affected stages. Production re-enablement criteria are documented in docs.
_STRICT_PROVENANCE = False

_DATASET_PATTERNS = {
    "stereo": re.compile(r"^stereo_\d{6}_\d{4}$"),
    "stereo_imu": re.compile(r"^stereo_imu_\d{6}_\d{4}$"),
    "stereo_baselink": re.compile(r"^stereo_base_link_\d{6}_\d{4}$"),
    "mono": re.compile(r"^usb_\d{6}_\d{4}$"),
    "mono_baselink": re.compile(r"^usb_base_link_\d{6}_\d{4}$"),
}
_STAGE_ROLES = {
    "camera": "stereo",
    "camera_imu": "stereo_imu",
    "camera_baselink": "stereo_baselink",
    "mono_camera": "mono",
    "mono_camera_baselink": "mono_baselink",
}
_TARGET_STAGES = {role: stage for stage, role in _STAGE_ROLES.items()}
_LEGACY_WORKFLOW_FIELDS = {
    "camera": "stereo",
    "camera_imu": "stereo_imu",
    "camera_baselink": "stereo_baselink",
    "mono_camera": "mono",
    "mono_camera_baselink": "mono_baselink",
}
_RECALIBRATION_STAGES = {
    "stereo": ("camera", "camera_imu", "camera_baselink"),
    "stereo_imu": ("camera_imu",),
    "stereo_baselink": ("camera_baselink",),
    "mono": ("mono_camera", "mono_camera_baselink"),
    "mono_baselink": ("mono_camera_baselink",),
}
_TEMPLATES = {
    "camera": "stereo_camera_calibration_task.yaml",
    "camera_rolling_shutter": (
        "stereo_camera_rs_calibration_task.yaml"
    ),
    "camera_baselink": "stereo_camera_baselink_calibration_task.yaml",
    "mono_camera": "mono_camera_calibration_task.yaml",
    "mono_camera_baselink": "mono_camera_baselink_calibration_task.yaml",
    "camera_imu_standard": "stereo_imu_calibration_task.yaml",
    "camera_imu_rolling_shutter": (
        "stereo_imu_rs_calibration_task.yaml"
    ),
}
_BATCH_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
_TARGET_STEM_PATTERN = re.compile(r"^[^\W_][\w-]{0,63}$", re.UNICODE)
_DEFAULT_BASE_LINK = {
    "frame_id": "base_link",
    "position_scale_m": 0.01,
}
_RETIRED_CALIB_PARAMS = {
    "stereo": Path("stereo_camera") / "stereo_calib.yaml",
}
_RENAMED_CALIB_PARAMS = {
    "vision": (
        Path("vision") / "vision_calib.yaml",
        Path("vision") / "stereo_imu_calib.yaml",
    ),
}
_MACHINE_CAMERA_MODELS = frozenset((
    "pinhole-equi",
    "pinhole-radtan5",
    "pinhole-radtan8",
))


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
        "schema_version", "kind", "template", "machine_dir", "datasets",
        "targets", "stereo", "stereo_imu", "stereo_baselink", "mono",
        "mono_baselink", "reuse_calib_params",
    }
    if "target" in value:
        raise TaskError(
            "target is no longer supported; use targets.<stage> instead")
    legacy = sorted(set(value) & set(_LEGACY_WORKFLOW_FIELDS))
    if legacy:
        raise TaskError("{} is no longer supported; use {} instead".format(
            legacy[0], _LEGACY_WORKFLOW_FIELDS[legacy[0]]))
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

    template = value.get("template")
    if (not isinstance(template, str)
            or not _BATCH_NAME_PATTERN.fullmatch(template)):
        raise TaskError(
            "template must match [A-Za-z0-9][A-Za-z0-9_-]{0,63}")

    configured_machine = value.get("machine_dir")
    if not isinstance(configured_machine, str) or not configured_machine.strip():
        raise TaskError("machine_dir must be a non-empty path")
    machine = Path(configured_machine).expanduser()
    if not machine.is_absolute():
        machine = config_path.parent / machine
    if machine.is_symlink():
        raise TaskError("machine batch directory must not be a symlink: {}".format(
            machine))
    machine = machine.resolve()
    if not machine.is_dir():
        raise TaskError("machine_dir is not a directory: {}".format(machine))
    if not _BATCH_NAME_PATTERN.fullmatch(machine.name):
        raise TaskError(
            "machine_dir batch name must match "
            "[A-Za-z0-9][A-Za-z0-9_-]{0,63}")
    machine_root = machine.parent
    identity = machine_identity(machine_root)

    reuse_config = value.get("reuse_calib_params", {})
    legacy_reuse = (sorted(set(reuse_config) & set(_LEGACY_WORKFLOW_FIELDS))
                    if isinstance(reuse_config, dict) else [])
    if legacy_reuse:
        raise TaskError(
            "reuse_calib_params.{} is no longer supported; use {} instead"
            .format(legacy_reuse[0], _LEGACY_WORKFLOW_FIELDS[legacy_reuse[0]]))
    if (not isinstance(reuse_config, dict)
            or set(reuse_config) - set(_TARGET_STAGES)
            or any(type(enabled) is not bool
                   for enabled in reuse_config.values())):
        raise TaskError(
            "reuse_calib_params must map the five task names to booleans")
    reuse = {stage: reuse_config.get(role, False)
             for role, stage in _TARGET_STAGES.items()}

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

    targets = value.get("targets", {})
    if not isinstance(targets, dict):
        raise TaskError("targets must be a mapping")
    unknown_targets = sorted(set(targets) - set(_TARGET_STAGES))
    if unknown_targets:
        raise TaskError("unknown targets fields: {}".format(
            ", ".join(unknown_targets)))
    for role, target in targets.items():
        if (not isinstance(target, str)
                or not _TARGET_STEM_PATTERN.fullmatch(target)):
            raise TaskError(
                "targets.{} must be a config/mechine YAML filename stem "
                "using Unicode letters/digits, underscores, or hyphens"
                .format(role))
    targets = {_TARGET_STAGES[role]: target
               for role, target in targets.items()}

    stereo = value.get("stereo", {})
    if not isinstance(stereo, dict):
        raise TaskError("stereo must be a mapping")
    unknown_stereo = sorted(set(stereo) - {"mode"})
    if unknown_stereo:
        raise TaskError("unknown stereo fields: {}".format(
            ", ".join(unknown_stereo)))
    camera_mode = stereo.get("mode", "standard")
    if camera_mode not in {"standard", "rolling_shutter"}:
        raise TaskError("stereo.mode must be standard or rolling_shutter")

    stereo_imu = value.get("stereo_imu")
    if not isinstance(stereo_imu, dict):
        raise TaskError("stereo_imu must be a mapping")
    unknown_stereo_imu = sorted(
        set(stereo_imu) - {"mode", "recompute_extrinsics"})
    if unknown_stereo_imu:
        raise TaskError(
            "unknown stereo_imu fields: {}".format(
                ", ".join(unknown_stereo_imu)))
    mode = stereo_imu.get("mode")
    if mode not in {"standard", "rolling_shutter"}:
        raise TaskError(
            "stereo_imu.mode must be standard or rolling_shutter")
    recompute_extrinsics = stereo_imu.get("recompute_extrinsics", False)
    if type(recompute_extrinsics) is not bool:
        raise TaskError("stereo_imu.recompute_extrinsics must be a boolean")

    mono = value.get("mono", {})
    if not isinstance(mono, dict) or mono:
        raise TaskError("mono must be an empty mapping")

    baselinks = {}
    for public_key, key, camera_ids in (
            ("stereo_baselink", "camera_baselink", ["cam0", "cam1"]),
            ("mono_baselink", "mono_camera_baselink", ["cam0"])):
        if public_key not in value:
            continue
        from .validation import validate_options
        baselink = deepcopy(value[public_key])
        if (not isinstance(baselink, dict)
                or not set(baselink) <= {"base_link", "cameras"}
                or "cameras" not in baselink):
            raise TaskError(
                "{} must contain cameras and may contain base_link".format(
                    public_key))
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
                public_key, expected))
        # Share the low-level unit, translation and frame-ID contract.
        validate_options({
            "job": CAMERA_BASELINK_JOB,
            "dataset": {"type": "directory", "path": "__DATASET_PATH__"},
            "camera_calibration": {"path": "__CAMERA_CALIBRATION_PATH__"},
            "output": {}, **baselink,
        })
        baselink["base_link"].setdefault("frame_id", "base_link")
        baselinks[key] = baselink

    canonical = {
        "schema_version": SCHEMA_VERSION,
        "kind": WORKFLOW_KIND,
        "template": template,
        "machine_dir": str(machine),
        "datasets": selected,
        "targets": targets,
        "reuse_calib_params": {
            role: reuse_config.get(role, False) for role in _TARGET_STAGES
        },
        "stereo": {"mode": camera_mode},
        "stereo_imu": {
            "mode": mode,
            "recompute_extrinsics": recompute_extrinsics,
        },
        "mono": {},
    }
    if "camera_baselink" in baselinks:
        canonical["stereo_baselink"] = baselinks["camera_baselink"]
    if "mono_camera_baselink" in baselinks:
        canonical["mono_baselink"] = baselinks["mono_camera_baselink"]
    return {
        "config_path": config_path,
        "template": template,
        "machine_dir": machine,
        "machine_root": machine_root,
        "batch_name": machine.name,
        "machine_version": identity["machine_version"],
        "machine_num": identity["machine_num"],
        "datasets": selected,
        "targets": targets,
        "reuse_calib_params": reuse,
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
        "config_sha256": _json_sha256(canonical),
    }


def _discover_dataset_path(machine_dir, role, selected=None, children=None):
    """Return a named direct child without opening or hashing its contents."""
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
        return candidate.resolve()

    candidates = []
    for candidate in (machine.iterdir() if children is None else children):
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
    return candidates[0]


def discover_dataset(machine_dir, role, selected=None):
    """Return the unique validated direct-child dataset or ``None``."""
    candidate = _discover_dataset_path(machine_dir, role, selected)
    if candidate is not None:
        _validate_dataset_role(candidate, role)
    return candidate


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


def _file_metadata(path):
    stat = Path(path).stat()
    return {
        "size": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "ctime_ns": stat.st_ctime_ns,
        "inode": stat.st_ino,
        "device": stat.st_dev,
    }


def _valid_cached_metadata(value):
    return (isinstance(value, dict)
            and all(isinstance(value.get(key), int)
                    and not isinstance(value.get(key), bool) for key in (
                        "size", "mtime_ns", "ctime_ns", "inode", "device")))


def _cached_file_hash(value):
    if (not _valid_cached_metadata(value)
            or not isinstance(value.get("sha256"), str)
            or re.fullmatch(r"[0-9a-f]{64}", value["sha256"]) is None):
        return None
    return value["sha256"]


def _metadata_fingerprint(dataset, role, dataset_id, snapshots):
    digest = hashlib.sha256()
    for relative in sorted(snapshots):
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        metadata = snapshots[relative]
        for key in ("size", "mtime_ns", "ctime_ns", "inode", "device"):
            digest.update(str(metadata[key]).encode("ascii"))
            digest.update(b"\0")
        digest.update(b"\n")
    return {
        "role": role,
        "path": str(dataset),
        "name": dataset.name,
        "dataset_id": dataset_id,
        "sha256": digest.hexdigest(),
        "fingerprint_type": "metadata",
        "file_count": len(snapshots),
    }


def _valid_cached_record(value, dataset, role, dataset_id, file_count):
    return (isinstance(value, dict)
            and value.get("role") == role
            and value.get("path") == str(dataset)
            and value.get("name") == dataset.name
            and value.get("dataset_id") == dataset_id
            and value.get("file_count") == file_count
            and value.get("fingerprint_type", "content")
            in {"content", "metadata"}
            and isinstance(value.get("sha256"), str)
            and re.fullmatch(r"[0-9a-f]{64}", value["sha256"]) is not None)


def _empty_input_hash_cache():
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": INPUT_HASH_CACHE_KIND,
        "datasets": {},
    }


def _load_input_hash_cache(path):
    path = Path(path)
    if path.is_symlink():
        raise TaskError("input hash cache must not be a symlink: {}".format(path))
    if not path.exists():
        return _empty_input_hash_cache()
    if not path.is_file():
        raise TaskError("input hash cache must be a regular file: {}".format(path))
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return _empty_input_hash_cache()
    if (not isinstance(value, dict)
            or value.get("schema_version") != SCHEMA_VERSION
            or value.get("kind") != INPUT_HASH_CACHE_KIND
            or not isinstance(value.get("datasets"), dict)):
        return _empty_input_hash_cache()
    return value


def _write_input_hash_cache(path, value):
    path = Path(path)
    if path.is_symlink():
        raise TaskError("input hash cache must not be a symlink: {}".format(path))
    _atomic_json(path, value)


def _fingerprint_dataset(
        dataset, role, cached=None, progress=None, expected_record=None,
        force_content=False):
    """Return a fast metadata fingerprint, hashing content only when needed."""
    started = time.monotonic()
    dataset = Path(dataset).resolve()
    reader = _validate_dataset_role(dataset, role)
    manifest = reader.manifest
    files = _dataset_files(dataset, manifest)
    cached_files = (cached.get("files", {})
                    if isinstance(cached, dict)
                    and cached.get("role") == role
                    and cached.get("dataset_id") == reader.dataset_id
                    and isinstance(cached.get("files"), dict) else {})
    snapshots = {}
    total_bytes = 0
    for path in files:
        relative = path.relative_to(dataset).as_posix()
        metadata = _file_metadata(path)
        snapshots[relative] = metadata
        total_bytes += metadata["size"]

    metadata_matches = (
        set(cached_files) == set(snapshots)
        and all(_valid_cached_metadata(cached_files[relative])
                and all(cached_files[relative][key] == metadata[key]
                        for key in metadata)
                for relative, metadata in snapshots.items()))
    cached_record = cached.get("record") if isinstance(cached, dict) else None
    if (metadata_matches
            and _valid_cached_record(
                cached_record, dataset, role, reader.dataset_id, len(files))):
        record = dict(cached_record)
        statistics = {
            "file_count": len(files),
            "total_bytes": total_bytes,
            "reused_files": len(files),
            "hashed_files": 0,
            "hashed_bytes": 0,
            "mode": "cache",
            "elapsed_s": time.monotonic() - started,
        }
        if progress is not None:
            progress("start", {
                "dataset": dataset.name, "role": role, **statistics})
            progress("complete", {
                "dataset": dataset.name, "role": role, **statistics})
        return record, cached, statistics

    reusable = {}
    if metadata_matches:
        for relative, candidate in cached_files.items():
            file_hash = _cached_file_hash(candidate)
            if file_hash is not None:
                reusable[relative] = file_hash
    legacy_cache_complete = (
        metadata_matches and len(reusable) == len(files))
    if legacy_cache_complete and cached_record is None:
        force_content = True

    expected_is_content = (
        isinstance(expected_record, dict)
        and expected_record.get("fingerprint_type", "content") == "content")
    needs_content_hash = (
        force_content or expected_is_content or legacy_cache_complete
        or bool(cached_files) and not metadata_matches)

    if not needs_content_hash:
        record = _metadata_fingerprint(
            dataset, role, reader.dataset_id, snapshots)
        cache_entry = {
            "role": role,
            "dataset_id": reader.dataset_id,
            "record": record,
            "files": dict(snapshots),
        }
        statistics = {
            "file_count": len(files),
            "total_bytes": total_bytes,
            "reused_files": 0,
            "hashed_files": 0,
            "hashed_bytes": 0,
            "mode": "metadata_snapshot",
            "elapsed_s": time.monotonic() - started,
        }
        if progress is not None:
            progress("start", {
                "dataset": dataset.name, "role": role, **statistics})
            progress("complete", {
                "dataset": dataset.name, "role": role, **statistics})
        return record, cache_entry, statistics

    hashed = [path for path in files
              if path.relative_to(dataset).as_posix() not in reusable]
    hashed_bytes = sum(
        snapshots[path.relative_to(dataset).as_posix()]["size"]
        for path in hashed)
    if progress is not None:
        progress("start", {
            "dataset": dataset.name,
            "role": role,
            "file_count": len(files),
            "total_bytes": total_bytes,
            "reused_files": len(reusable),
            "hashed_files": len(hashed),
            "hashed_bytes": hashed_bytes,
            "mode": "content_hash",
        })

    hashes = dict(reusable)
    processed_bytes = 0
    last_percent = 0
    for path in hashed:
        relative = path.relative_to(dataset).as_posix()
        before = _file_metadata(path)
        if before != snapshots[relative]:
            raise TaskError(
                "input changed while preparing its hash: {}".format(path))
        hashes[relative] = _sha256(path)
        after = _file_metadata(path)
        if after != before:
            raise TaskError(
                "input changed while it was being hashed: {}".format(path))
        processed_bytes += after["size"]
        percent = (100 if hashed_bytes == 0 else
                   min(100, int(processed_bytes * 100 / hashed_bytes)))
        milestone = min(100, (percent // 10) * 10)
        if progress is not None and milestone > last_percent:
            last_percent = milestone
            progress("progress", {
                "dataset": dataset.name,
                "role": role,
                "percent": milestone,
                "processed_bytes": processed_bytes,
                "hashed_bytes": hashed_bytes,
            })

    for path in files:
        relative = path.relative_to(dataset).as_posix()
        if _file_metadata(path) != snapshots[relative]:
            raise TaskError(
                "input changed during dataset verification: {}".format(path))

    digest = hashlib.sha256()
    cache_files = {}
    for path in files:
        relative = path.relative_to(dataset).as_posix()
        file_digest = hashes[relative]
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(file_digest.encode("ascii"))
        digest.update(b"\n")
        cache_files[relative] = {
            **snapshots[relative], "sha256": file_digest}
    record = {
        "role": role,
        "path": str(dataset),
        "name": dataset.name,
        "dataset_id": reader.dataset_id,
        "sha256": digest.hexdigest(),
        "file_count": len(files),
    }
    cache_entry = {
        "role": role,
        "dataset_id": reader.dataset_id,
        "record": record,
        "files": cache_files,
    }
    statistics = {
        "file_count": len(files),
        "total_bytes": total_bytes,
        "reused_files": len(reusable),
        "hashed_files": len(hashed),
        "hashed_bytes": hashed_bytes,
        "mode": "content_hash",
        "elapsed_s": time.monotonic() - started,
    }
    if progress is not None:
        progress("complete", {
            "dataset": dataset.name,
            "role": role,
            **statistics,
        })
    return record, cache_entry, statistics


def dataset_fingerprint(dataset, role):
    """Validate sensor composition and hash every solver-referenced input."""
    return _fingerprint_dataset(dataset, role, force_content=True)[0]


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
    from .validation import load_target
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
        "sha256": _sha256(path),
    }


def _profile_provenance(profile, target_overrides, config):
    targets = _effective_target_paths(profile, target_overrides, config)
    files = {}
    config_root = profile
    for path in sorted(config_root.rglob("*.yaml")):
        if path.is_symlink() or not path.is_file():
            raise TaskError("profile YAML must be a regular file: {}".format(path))
        files[_profile_relative_path(profile, path)] = _sha256(path)
    return {
        "version": profile.name,
        "files": files,
        "dependencies": _profile_dependencies(profile, targets, config),
        "targets": {
            stage: _profile_target_record(profile, path)
            for stage, path in targets.items()
        },
        "target_overrides": sorted(target_overrides),
    }


def _software_provenance():
    implementation = hashlib.sha256()
    for path in sorted({Path(__file__).resolve(),
                        Path(__file__).with_name("task.py").resolve(),
                        Path(__file__).with_name("camera_baselink.py").resolve(),
                        Path(__file__).with_name("machine_calib_params.py").resolve(),
                        Path(__file__).with_name("machine_param_reuse.py").resolve(),
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
    if stage in {"camera", "camera_imu"} and output.get(
            "save_metrics") is not True:
        raise TaskError(
            "machine workflow camera templates must enable output.save_metrics")
    if stage in {"camera_imu", "camera_baselink", "mono_camera_baselink"}:
        if camera_result is None:
            raise TaskError("camera_result is required for the {} stage".format(stage))
        task["camera_calibration"] = {
            "path": str(Path(camera_result).resolve())}
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


def _expected_result_paths(output, name):
    task_output = output / name
    return {
        "result": task_output / (name + ".yaml"),
        "html": output / (name + ".report.html"),
        "pdf": output / (name + ".report.pdf"),
        "sidecar": task_output,
    }


def _delivery_exists(output, name):
    paths = _expected_result_paths(output, name)
    legacy = [
        output / (name + suffix)
        for suffix in (".yaml", ".report.html", ".report.pdf", ".results.txt")
    ]
    return any(path.exists() or path.is_symlink()
               for path in [*paths.values(), *legacy])


def _delivery_summary(inspected):
    sidecar = inspected["sidecar"]
    return {
        "result": str(inspected["result"].resolve()),
        "files": {
            key: _sha256(inspected[key]) for key in ("result", "html", "pdf")
        },
        "run_manifest_sha256": _sha256(inspected["manifest_path"]),
        "task_resolved_sha256": _sha256(inspected["task_resolved"]),
        "sidecar_sha256": _tree_sha256(sidecar),
    }


def _managed_delivery_summary(output, name):
    from .delivery import inspect_managed_delivery
    inspected = inspect_managed_delivery(output, name, layout=MACHINE_LAYOUT)
    return _delivery_summary(inspected)


def _delivery_name(output, delivery):
    result = Path(delivery.get("result", ""))
    name = result.parent.name
    expected = _expected_result_paths(output, name)["result"]
    if not name or result.resolve() != expected.resolve():
        raise TaskError("workflow checkpoint has an invalid delivery path")
    return name


def _verify_previous_delivery(output, delivery):
    name = _delivery_name(output, delivery)
    if _managed_delivery_summary(output, name) != delivery:
        raise TaskError(
            "managed output changed outside the workflow: {}".format(name))
    return name


def _validate_delivery(output, name, expected_job, dataset_record,
                       task_path, camera_dependency=None):
    from .delivery import inspect_managed_delivery
    inspected = inspect_managed_delivery(output, name, layout=MACHINE_LAYOUT)
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
    resolved_task = _resolved_task(task_path, expected_job)
    if load_yaml(task_resolved_path) != resolved_task:
        raise TaskError("delivery resolved task does not match the generated task")

    input_documents = manifest.get("input_documents")
    if (not isinstance(input_documents, list)
            or any(not isinstance(entry, dict)
                   for entry in input_documents)):
        raise TaskError("delivery has invalid input document provenance")
    semantic_inputs = [
        (role, resolved_task.get(role))
        for role in ("target", "initialization")
    ]
    semantic_inputs.extend(
        ("imu{}".format(index), block)
        for index, block in enumerate(resolved_task.get("imus", [])))
    for role, block in semantic_inputs:
        if not isinstance(block, dict) or not block.get("path"):
            continue
        source = Path(block["path"]).resolve()
        matches = [entry for entry in input_documents
                   if entry.get("role") == role]
        if (len(matches) != 1
                or Path(str(matches[0].get("path", ""))).resolve() != source
                or matches[0].get("document") != load_yaml(source)):
            raise TaskError(
                "delivery {} input document does not match".format(role))

    if camera_dependency is not None:
        expected_path = str(Path(camera_dependency["path"]).resolve())
        matches = [entry for entry in input_documents
                   if entry.get("role") == "camera_calibration"]
        if (len(matches) != 1
                or str(Path(matches[0].get("path", "")).resolve()) != expected_path
                or matches[0].get("sha256") != camera_dependency["sha256"]):
            raise TaskError("delivery references a different camera result")

    return _delivery_summary(inspected)


def _load_state(path, machine):
    if not path.exists():
        return {
            "schema_version": SCHEMA_VERSION,
            "kind": STATE_KIND,
            "machine_dir": str(machine),
            "binding": None,
            "stages": {},
            "calib_params": {},
        }
    if path.is_symlink() or not path.is_file():
        raise TaskError("workflow state must be a regular file: {}".format(path))
    try:
        state = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise TaskError("invalid workflow state: {}".format(error)) from error
    if (not isinstance(state, dict)
            or state.get("schema_version") != SCHEMA_VERSION
            or state.get("kind") != STATE_KIND):
        raise TaskError("workflow state has an incompatible schema or machine")
    stored_machine = state.get("machine_dir")
    same_identity = (
        isinstance(stored_machine, str)
        and Path(stored_machine).name == machine.name
        and Path(stored_machine).parent.name == machine.parent.name)
    if stored_machine != str(machine):
        if _STRICT_PROVENANCE or not same_identity:
            raise TaskError("workflow state has an incompatible schema or machine")
        state["machine_dir"] = str(machine)
    if "runs" in state or "active_revision" in state:
        raise TaskError(
            "workflow state uses the retired revision layout; create a new "
            "batch directory and update machine_dir in user.yaml")
    binding = state.get("binding")
    stages = state.get("stages", {})
    calib_params = state.get("calib_params", {})
    parameter_reuse = state.get("parameter_reuse", {})
    pending = state.get("pending_recalibration")
    pending_valid = (
        pending is None
        or isinstance(pending, dict)
        and isinstance(pending.get("target_binding"), dict)
        and isinstance(pending.get("changed_roles"), list)
        and all(isinstance(value, str)
                for value in pending["changed_roles"])
        and set(pending["changed_roles"]) <= set(_DATASET_PATTERNS)
        and isinstance(pending.get("changed_profile_inputs", []), list)
        and all(isinstance(value, str)
                for value in pending.get("changed_profile_inputs", []))
        and isinstance(pending.get("affected_stages"), list)
        and all(isinstance(value, str)
                for value in pending["affected_stages"])
        and set(pending["affected_stages"]) <= set(_STAGE_ROLES)
        and isinstance(pending.get("previous_deliveries", {}), dict)
        and set(pending.get("previous_deliveries", {})) <= set(_STAGE_ROLES)
        and all(isinstance(value, dict) for value in
                pending.get("previous_deliveries", {}).values())
    )
    if (binding is not None and not isinstance(binding, dict)
            or isinstance(binding, dict)
            and (not isinstance(binding.get("datasets", {}), dict)
                 or not isinstance(binding.get("profile", {}), dict)
                 or not isinstance(binding.get("software", {}), dict))
            or not isinstance(stages, dict)
            or any(not isinstance(stage, dict) for stage in stages.values())
            or not isinstance(calib_params, dict)
            or any(not isinstance(value, dict)
                   for value in calib_params.values())
            or not isinstance(parameter_reuse, dict)
            or set(parameter_reuse) - set(_STAGE_ROLES)
            or any(not isinstance(value, dict) for value in parameter_reuse.values())
            or not pending_valid):
        raise TaskError("workflow state contains invalid records")
    state.setdefault("binding", None)
    state.setdefault("stages", {})
    state.setdefault("calib_params", {})
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


def _binding(config, profile, software, datasets):
    return {
        "camera_mode": config["camera_mode"],
        "camera_imu_mode": config["camera_imu_mode"],
        "camera_imu_recompute_extrinsics": config[
            "camera_imu_recompute_extrinsics"],
        "camera_baselink": config["camera_baselink"],
        "mono_camera_baselink": config["mono_camera_baselink"],
        "profile": profile,
        "software": software,
        "datasets": datasets,
        "reuse_calib_params": config.get("reuse_calib_params", {}),
        "parameter_inputs": config.get("parameter_inputs", {}),
    }


def _stored_profile_targets(profile):
    targets = profile.get("targets")
    if isinstance(targets, dict):
        return targets
    legacy = profile.get("target")
    if legacy is None:
        return None
    if (not isinstance(legacy, dict)
            or not isinstance(legacy.get("name"), str)
            or not isinstance(legacy.get("sha256"), str)):
        raise TaskError("workflow state contains invalid target provenance")
    record = {
        "path": "../" + legacy["name"],
        "sha256": legacy["sha256"],
    }
    return {stage: record for stage in _STAGE_ROLES}


def _profile_recalibration(existing, current, binding):
    if (not isinstance(existing, dict) or not isinstance(current, dict)
            or existing.get("version") != current.get("version")):
        if _STRICT_PROVENANCE:
            raise TaskError(
                "workflow profile version changed; create a new batch directory "
                "and update machine_dir in user.yaml")
        return {"@profile.version"}, list(_STAGE_ROLES)
    old_files = existing.get("files")
    new_files = current.get("files")
    if not isinstance(old_files, dict) or not isinstance(new_files, dict):
        raise TaskError("workflow state contains invalid profile provenance")
    changed = {
        name for name in set(old_files) | set(new_files)
        if old_files.get(name) != new_files.get(name)
    }
    affected = set()
    old_dependencies = existing.get("dependencies", {})
    new_dependencies = current.get("dependencies", {})
    if (not isinstance(old_dependencies, dict)
            or not isinstance(new_dependencies, dict)):
        old_dependencies = {}
        new_dependencies = {}
    camera_template = _TEMPLATES[
        "camera" if binding["camera_mode"] == "standard"
        else "camera_rolling_shutter"]
    imu_template = _TEMPLATES[
        "camera_imu_" + binding["camera_imu_mode"]]
    stage_files = {
        camera_template: _RECALIBRATION_STAGES["stereo"],
        "initialization_stereo_camera.yaml": _RECALIBRATION_STAGES["stereo"],
        "initialization_mono_camera.yaml": _RECALIBRATION_STAGES["mono"],
        imu_template: _RECALIBRATION_STAGES["stereo_imu"],
        "imu_noise.yaml": _RECALIBRATION_STAGES["stereo_imu"],
        _TEMPLATES["camera_baselink"]: _RECALIBRATION_STAGES[
            "stereo_baselink"],
        _TEMPLATES["mono_camera"]: _RECALIBRATION_STAGES["mono"],
        _TEMPLATES["mono_camera_baselink"]: _RECALIBRATION_STAGES[
            "mono_baselink"],
    }
    inactive = set()
    for stage, names in {
            "camera": (camera_template, "initialization_stereo_camera.yaml"),
            "camera_imu": (imu_template, "imu_noise.yaml"),
            "mono_camera": (_TEMPLATES["mono_camera"], "initialization_mono_camera.yaml"),
            "camera_baselink": (_TEMPLATES["camera_baselink"],),
            "mono_camera_baselink": (_TEMPLATES["mono_camera_baselink"],),
    }.items():
        if binding.get("reuse_calib_params", {}).get(stage, False):
            inactive.update(names)
    for name in changed:
        # Still honor a file explicitly referenced by another active template.
        if name in inactive and name not in new_dependencies:
            continue
        affected.update(stage_files.get(name, ()))
        affected.update(old_dependencies.get(name, ()))
        affected.update(new_dependencies.get(name, ()))
    old_targets = _stored_profile_targets(existing)
    new_targets = _stored_profile_targets(current)
    if new_targets is None:
        raise TaskError("workflow state contains invalid target provenance")
    compared_stages = (
        current.get("target_overrides", ())
        if old_targets is None else _STAGE_ROLES)
    for stage in compared_stages:
        if binding.get("reuse_calib_params", {}).get(stage, False):
            continue
        if old_targets is None or old_targets.get(stage) != new_targets.get(stage):
            changed.add("@target." + stage)
            affected.update(_RECALIBRATION_STAGES[_STAGE_ROLES[stage]])
    return changed, [stage for stage in _STAGE_ROLES if stage in affected]


def _merge_binding(existing, current):
    if existing is None:
        return current, set(), set(), []
    merged = deepcopy(existing)
    changed = set()
    strict_roles = {
        "camera_mode": {"stereo"},
        "camera_imu_mode": {"stereo_imu"},
        "camera_imu_recompute_extrinsics": {"stereo_imu"},
        "software": set(_DATASET_PATTERNS),
    }
    for key, roles in strict_roles.items():
        if existing.get(key) == current.get(key):
            continue
        if _STRICT_PROVENANCE:
            raise TaskError(
                "workflow provenance changed; create a new batch directory "
                "and update machine_dir in user.yaml")
        merged[key] = deepcopy(current.get(key))
        changed.update(roles)
    changed_profile, profile_stages = _profile_recalibration(
        existing.get("profile"), current.get("profile"), current)
    merged["profile"] = deepcopy(current["profile"])
    for key, role in (("camera_baselink", "stereo_baselink"),
                      ("mono_camera_baselink", "mono_baselink")):
        old_baselink = existing.get(key)
        new_baselink = current.get(key)
        if old_baselink is not None and old_baselink != new_baselink:
            if _STRICT_PROVENANCE:
                raise TaskError(
                    "{} configuration changed; create a new batch directory "
                    "and update machine_dir in user.yaml".format(key))
            changed.add(role)
        merged[key] = new_baselink
    old_datasets = existing.get("datasets", {})
    new_datasets = current.get("datasets", {})
    if not isinstance(old_datasets, dict) or not isinstance(new_datasets, dict):
        raise TaskError("workflow state contains invalid dataset provenance")
    for role in _DATASET_PATTERNS:
        stage = next(stage for stage, value in _STAGE_ROLES.items()
                     if value == role)
        reuse = current.get("reuse_calib_params", {}).get(stage, False)
        old_reuse = existing.get("reuse_calib_params", {}).get(stage, False)
        if reuse != old_reuse:
            changed.add(role)
        if stage in {"camera", "mono_camera"} and (
                existing.get("parameter_inputs", {}).get(stage)
                != current.get("parameter_inputs", {}).get(stage)):
            changed.add(role)
        old = old_datasets.get(role)
        new = new_datasets.get(role)
        if (old is not None and not isinstance(old, dict)
                or new is not None and not isinstance(new, dict)):
            raise TaskError("workflow state contains invalid dataset provenance")
        if old is None:
            merged.setdefault("datasets", {})[role] = new
        elif new is None and reuse:
            merged.setdefault("datasets", {})[role] = None
        elif new is None:
            if _STRICT_PROVENANCE:
                raise TaskError(
                    "{} dataset disappeared; add one strictly named replacement "
                    "dataset and rerun".format(role))
            merged.setdefault("datasets", {})[role] = None
            changed.add(role)
        elif old != new:
            if (old.get("name") == new.get("name")
                    or old.get("path") == new.get("path")):
                if _STRICT_PROVENANCE:
                    raise TaskError(
                        "{} dataset changed in place; restore it or use a new "
                        "YYMMDD_HHMM directory name".format(role))
            merged.setdefault("datasets", {})[role] = new
            changed.add(role)
    merged["reuse_calib_params"] = deepcopy(current.get("reuse_calib_params", {}))
    merged["parameter_inputs"] = deepcopy(current.get("parameter_inputs", {}))
    return merged, changed, changed_profile, profile_stages


def _affected_stages(changed_roles):
    affected = set()
    for role in changed_roles:
        affected.update(_RECALIBRATION_STAGES[role])
    return [stage for stage in _STAGE_ROLES if stage in affected]


def _start_recalibration(state, target_binding, changed_roles, datasets,
                         output, profile_stages=(), changed_profile_inputs=()):
    affected_set = set(_affected_stages(changed_roles)) | set(profile_stages)
    affected = [stage for stage in _STAGE_ROLES if stage in affected_set]
    previous = {}
    for stage in affected:
        target = datasets.get(_STAGE_ROLES[stage])
        stored = state.get("stages", {}).get(stage)
        if (target is None or not isinstance(stored, dict)
                or stored.get("status") != "completed"):
            continue
        delivery = stored.get("delivery")
        if not isinstance(delivery, dict):
            raise TaskError(
                "completed {} stage lacks its delivery checkpoint".format(
                    stage))
        if _delivery_name(output, delivery) == target["name"]:
            _verify_previous_delivery(output, delivery)
            previous[stage] = deepcopy(delivery)
    return {
        "target_binding": deepcopy(target_binding),
        "changed_roles": sorted(changed_roles),
        "changed_profile_inputs": sorted(changed_profile_inputs),
        "affected_stages": affected,
        "previous_deliveries": previous,
    }


def _preflight_profile_recalibration(profile, targets, config, datasets,
                                     output, affected):
    parents = {
        "camera_imu": "camera",
        "camera_baselink": "camera",
        "mono_camera_baselink": "mono_camera",
    }
    for stage in affected:
        if config.get("reuse_calib_params", {}).get(stage, False):
            continue
        record = datasets.get(_STAGE_ROLES[stage])
        if record is None:
            continue
        parent = parents.get(stage)
        camera_result = None
        if parent is not None:
            parent_record = datasets.get(_STAGE_ROLES[parent])
            if parent_record is None:
                continue
            parent_name = parent_record["name"]
            camera_result = output / parent_name / (parent_name + ".yaml")
        build_task(
            profile, stage, record["path"], record["name"],
            camera_result=camera_result,
            mode=config["camera_imu_mode"],
            camera_baselink=(
                config.get(stage)
                if stage in {"camera_baselink", "mono_camera_baselink"}
                else None),
            camera_mode=config["camera_mode"],
            recompute_extrinsics=config[
                "camera_imu_recompute_extrinsics"],
            target_path=targets.get(stage))


def _stage_summary(status, result=None, action=None, message=None):
    value = {"status": status, "result": result}
    if action is not None:
        value["action"] = action
    if message is not None:
        value["message"] = message
    return value


def _emit_progress(callback, event, **details):
    if callback is not None:
        callback(event, details)


def _completed_stage(output, name, expected_job, dataset_record, task_path,
                     stored=None, camera_dependency=None):
    delivery = _validate_delivery(
        output, name, expected_job, dataset_record, task_path,
        camera_dependency=camera_dependency)
    if stored is not None:
        if (stored.get("status") != "completed"
                or stored.get("delivery") != delivery):
            raise TaskError(
                "completed output provenance changed; create a new batch "
                "directory and update machine_dir in user.yaml")
    return delivery


def _verify_stage_inputs_unchanged(
        dataset_record, camera_dependency=None, hash_cache_path=None,
        progress=None, stage=None):
    """Reject a solve if any content-addressed input changed while it ran."""
    if hash_cache_path is None:
        current_dataset = dataset_fingerprint(
            dataset_record["path"], dataset_record["role"])
    else:
        cache = _load_input_hash_cache(hash_cache_path)
        key = str(Path(dataset_record["path"]).resolve())

        def report(event, details):
            if event != "complete":
                _emit_progress(
                    progress, "dataset_recheck_" + event,
                    stage=stage, **details)

        current_dataset, entry, statistics = _fingerprint_dataset(
            dataset_record["path"], dataset_record["role"],
            cached=cache["datasets"].get(key), progress=report,
            expected_record=dataset_record)
        if current_dataset == dataset_record:
            cache["datasets"][key] = entry
            _write_input_hash_cache(hash_cache_path, cache)
            _emit_progress(
                progress, "dataset_recheck_complete",
                stage=stage, dataset=dataset_record["name"], **statistics)
    if current_dataset != dataset_record:
        raise TaskError(
            "dataset changed during calibration; keep capture files immutable "
            "and use a new batch directory")
    if camera_dependency is not None:
        current_camera_hash = _sha256(camera_dependency["path"])
        if current_camera_hash != camera_dependency["sha256"]:
            raise TaskError(
                "camera calibration changed during downstream calibration; "
                "use a new batch directory")


def _run_stage(*, prefix, runner, output, tasks, state_path, state,
               stage_name, task, output_name, expected_job,
               dataset_record, camera_dependency=None, recalibrate=False,
               previous_delivery=None, force_recalibrate=False,
               hash_cache_path=None, progress=None):
    _ensure_managed_directory(tasks, state_path.parent)
    final_task_path = tasks / (output_name + ".task.yaml")
    task_path = (
        tasks / (output_name + ".pending.task.yaml")
        if previous_delivery is not None else final_task_path)
    if task_path.is_symlink() or final_task_path.is_symlink():
        raise TaskError("workflow task file must not be a symlink")
    dump_yaml(task, task_path)
    stored = state.setdefault("stages", {}).get(stage_name)

    existing = _delivery_exists(output, output_name)
    if existing:
        try:
            delivery = _completed_stage(
                output, output_name, expected_job, dataset_record, task_path,
                camera_dependency=camera_dependency)
        except (TaskError, OSError, ValueError) as error:
            if previous_delivery is None:
                raise TaskError(
                    "existing output cannot be safely reused: {}".format(
                        error)) from error
            _verify_previous_delivery(output, previous_delivery)
        else:
            if recalibrate:
                checkpoint_matches = (
                    isinstance(stored, dict)
                    and (stored.get("status") in {"running", "failed"}
                         and stored.get("output_name") == output_name
                         or stored.get("status") == "completed"
                         and stored.get("delivery") == delivery)
                )
            else:
                checkpoint_matches = (
                    isinstance(stored, dict)
                    and stored.get("status") in {
                        "completed", "running", "failed"})
            if not checkpoint_matches:
                raise TaskError(
                    "existing output has no matching workflow checkpoint; "
                    "use a new batch directory")
            if (not recalibrate and stored.get("status") == "completed"
                    and stored.get("delivery") != delivery):
                raise TaskError(
                    "completed output provenance changed; create a new batch "
                    "directory and update machine_dir in user.yaml")
            if not force_recalibrate:
                if task_path != final_task_path:
                    os.replace(task_path, final_task_path)
                state["stages"][stage_name] = {
                    "status": "completed", "delivery": delivery}
                _atomic_json(state_path, state)
                return delivery, "reused"
    elif previous_delivery is not None:
        raise TaskError(
            "managed output disappeared before recalibration: {}".format(
                output_name))
    elif (not recalibrate and isinstance(stored, dict)
          and stored.get("status") == "completed"):
        raise TaskError(
            "completed output is missing; create a new batch directory")

    state["stages"][stage_name] = {
        "status": "running", "task": str(task_path.resolve()),
        "output_name": output_name,
    }
    _atomic_json(state_path, state)
    try:
        runner(
            prefix, task_path, output, expected_job,
            force=previous_delivery is not None)
        _verify_stage_inputs_unchanged(
            dataset_record, camera_dependency=camera_dependency,
            hash_cache_path=hash_cache_path, progress=progress,
            stage=stage_name)
        delivery = _completed_stage(
            output, output_name, expected_job, dataset_record, task_path,
            camera_dependency=camera_dependency)
    except Exception as error:
        state["stages"][stage_name] = {
            "status": "failed",
            "task": str(task_path),
            "output_name": output_name,
            "failure": {"type": type(error).__name__, "message": str(error)},
        }
        _atomic_json(state_path, state)
        raise
    if task_path != final_task_path:
        os.replace(task_path, final_task_path)
    state["stages"][stage_name] = {
        "status": "completed", "delivery": delivery}
    _atomic_json(state_path, state)
    return delivery, "executed"


def _run_task(prefix, config, output, expected_job, force=False):
    if expected_job == CAMERA_BASELINK_JOB:
        from .camera_baselink import run_camera_baselink
        return run_camera_baselink(
            config, output, force=force, _delivery_layout=MACHINE_LAYOUT)
    from .task import run_task
    return run_task(
        prefix, config, output, expected_job, force=force,
        _delivery_layout=MACHINE_LAYOUT)


def _verify_calib_param_record(record, target):
    if (not isinstance(record, dict)
            or str(target.resolve()) != record.get("path")
            or not isinstance(record.get("sha256"), str)):
        raise TaskError(
            "calib_params output has no matching workflow checkpoint: {}"
            .format(target))
    if target.is_symlink() or not target.is_file():
        raise TaskError("managed calib_params output is missing: {}".format(target))
    if _sha256(target) != record["sha256"]:
        raise TaskError(
            "managed calib_params output changed outside the workflow: {}"
            .format(target))


def _parameter_edits(base, published, current):
    """Apply copied edits without feeding our refined output back as a seed."""
    if all(isinstance(value, dict) for value in (base, published, current)):
        return {key: _parameter_edits(base.get(key), published.get(key), value)
                for key, value in current.items()}
    return deepcopy(base if current == published else current)


def _prepare_parameter_reuse(output, control, state, config):
    """Accept copied parameter blocks once, not our own subsequent output."""
    flags = config["reuse_calib_params"]
    if not any(flags.values()):
        for record in state.get("parameter_reuse", {}).values():
            record["active"] = False
        return {}, {}
    identity = {key: config[key] for key in ("machine_version", "machine_num")}
    records = state.setdefault("parameter_reuse", {})
    if not isinstance(records, dict):
        raise TaskError("workflow state has invalid parameter reuse records")
    accepted = {}
    for key, stage in (("vision", "camera"), ("mono", "mono_camera")):
        path = parameter_path(output, stage)
        current = output
        for part in path.relative_to(output).parts:
            current = current / part
            if current.is_symlink():
                raise TaskError("reused calib_params must not contain symlinks: {}".format(current))
        if path.exists():
            if not path.is_file():
                raise TaskError("calib_params must be a regular file: {}".format(path))
            accepted[key] = _sha256(path)
    reused = {}
    for stage, enabled in flags.items():
        previous = records.get(stage, {})
        if not isinstance(previous, dict):
            raise TaskError("workflow state has invalid parameter reuse record")
        if not enabled:
            if previous:
                previous["active"] = False
            continue
        key = "mono" if stage.startswith("mono_") else "vision"
        path = parameter_path(output, stage)
        if key not in accepted:
            raise TaskError("reuse_calib_params.{} requires {}".format(stage, path))
        digest = accepted[key]
        published = state.get("calib_params", {}).get(key, {}).get("sha256")
        if (previous.get("active") is True
                and digest in {published, previous.get("accepted_sha256")}
                and isinstance(previous.get("parameters"), dict)):
            parameters = previous["parameters"]
        else:
            parameters = validate_reused_stage(load_yaml(path), stage, identity)
            if (previous.get("active") is True
                    and isinstance(previous.get("published_parameters"), dict)):
                parameters = _parameter_edits(
                    previous["parameters"], previous["published_parameters"], parameters)
        record = {
            "active": True, "path": str(path.resolve()),
            "sha256": semantic_sha256(parameters),
            "parameters": parameters,
            "accepted_sha256": digest,
        }
        if "published_parameters" in previous:
            record["published_parameters"] = previous["published_parameters"]
        if stage in {"camera", "mono_camera"}:
            seed = camera_seed(parameters, mono=stage == "mono_camera")
            geometry = semantic_sha256(seed)
            directory = _ensure_managed_directory(control / "parameter_inputs", control)
            seed_path = directory / (stage + "_" + geometry + ".yaml")
            if seed_path.is_symlink():
                raise TaskError("reused camera input must not be a symlink")
            if seed_path.exists():
                if not seed_path.is_file() or load_yaml(seed_path) != seed:
                    raise TaskError("reused camera input changed outside the workflow")
            else:
                dump_yaml(seed, seed_path)
            record["seed"] = {
                "result": str(seed_path.resolve()),
                "files": {"result": _sha256(seed_path)},
                "geometry_sha256": geometry,
            }
        records[stage] = record
        reused[stage] = record
    return reused, accepted


def _retire_legacy_calib_params(root, active, state, state_path):
    """Remove only obsolete aggregate files still owned by workflow state."""
    if root.is_symlink():
        raise TaskError("calib_params output must not be a symlink: {}".format(root))
    if root.exists() and not root.is_dir():
        raise TaskError("calib_params output must be a directory: {}".format(root))
    retired = []
    empty_parents = []
    for name, relative in _RETIRED_CALIB_PARAMS.items():
        target = root / relative
        parent = target.parent
        stored = active.get(name)
        if parent.is_symlink():
            raise TaskError(
                "obsolete calib_params directory must not be a symlink: {}"
                .format(parent))
        if parent.exists() and not parent.is_dir():
            raise TaskError(
                "obsolete calib_params parent must be a directory: {}"
                .format(parent))
        entries = list(parent.iterdir()) if parent.is_dir() else []
        if stored is None:
            if entries:
                raise TaskError(
                    "obsolete unmanaged calib_params output exists: {}"
                    .format(parent))
            if parent.is_dir():
                empty_parents.append(parent)
            continue

        _verify_calib_param_record(stored, target)
        if entries != [target]:
            raise TaskError(
                "obsolete calib_params directory contains unmanaged files: {}"
                .format(parent))
        retired.append((name, target, parent))

    for parent in empty_parents:
        parent.rmdir()
    for name, target, parent in retired:
        target.unlink()
        parent.rmdir()
        active.pop(name)
    if retired:
        _atomic_json(state_path, state)


def _migrate_renamed_calib_params(root, active, state, state_path):
    """Atomically rename aggregate files still owned by workflow state."""
    changed = False
    for name, (old_relative, new_relative) in _RENAMED_CALIB_PARAMS.items():
        old = root / old_relative
        new = root / new_relative
        stored = active.get(name)
        old_path = str(old.resolve())
        if stored is None or stored.get("path") != old_path:
            if old.exists() or old.is_symlink():
                raise TaskError(
                    "obsolete unmanaged calib_params output exists: {}"
                    .format(old))
            continue
        if new.exists() or new.is_symlink():
            raise TaskError(
                "renamed calib_params target already exists: {}".format(new))
        _verify_calib_param_record(stored, old)
        os.replace(old, new)
        descriptor = os.open(str(new.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        updated = deepcopy(stored)
        updated["path"] = str(new.resolve())
        active[name] = updated
        changed = True
    if changed:
        _atomic_json(state_path, state)


def _update_calib_params(*, machine, output, profile, deliveries,
                         camera_mode, camera_imu_mode, state, state_path,
                         reused=None, accepted_hashes=None):
    generated_at = datetime.now(ZoneInfo("Asia/Shanghai")).strftime(
        "%Y-%m-%d-%H:%M:%S")
    artifacts = build_calib_params(
        machine, profile, deliveries, generated_at,
        camera_mode=camera_mode, camera_imu_mode=camera_imu_mode,
        reused=reused)
    active = state.setdefault("calib_params", {})
    summary = {}
    root = output / "calib_params"
    _migrate_renamed_calib_params(root, active, state, state_path)
    _retire_legacy_calib_params(root, active, state, state_path)
    for name, artifact in artifacts.items():
        target = root / artifact["relative_path"]
        stored = active.get(name)
        accepted = (accepted_hashes or {}).get(name)
        if accepted is not None:
            if (target.is_symlink() or not target.is_file()
                    or _sha256(target) != accepted):
                raise TaskError("calib_params changed during workflow: {}".format(target))
        elif stored is not None:
            _verify_calib_param_record(stored, target)
        elif target.exists() or target.is_symlink():
            raise TaskError(
                "unmanaged calib_params output already exists: {}".format(target))

        if (stored is not None
                and stored.get("source_signature")
                == artifact["source_signature"]
                and (accepted is None or stored.get("sha256") == accepted)):
            record = deepcopy(stored)
            action = "reused"
        else:
            current = load_yaml(target) if accepted is not None else None
            if (isinstance(current, dict)
                    and {key: value for key, value in current.items() if key != "time"}
                    == {key: value for key, value in artifact["document"].items() if key != "time"}):
                written = {"path": str(target.resolve()), "sha256": accepted}
                artifact["document"]["time"] = current["time"]
            else:
                written = write_calib_param(output, artifact)
            record = {
                "status": "completed",
                "path": written["path"],
                "sha256": written["sha256"],
                "time": artifact["document"]["time"],
                "source_signature": artifact["source_signature"],
                "defaulted_fields": artifact["defaulted_fields"],
                "profile": artifact["profile"],
                "sources": artifact["sources"],
            }
            active[name] = record
            action = "updated" if stored is not None else "created"
            _atomic_json(state_path, state)
        summary[name] = {
            "status": "completed",
            "action": action,
            "result": record["path"],
            "defaulted_fields": record["defaulted_fields"],
        }
        for stage, reused_record in (reused or {}).items():
            if parameter_path(output, stage) == target:
                parameters = validate_reused_stage(
                    artifact["document"], stage, machine_identity(machine))
                reused_record["published_parameters"] = parameters
                state["parameter_reuse"][stage] = reused_record
        _atomic_json(state_path, state)
    return summary


def run_machine_workflow(prefix, config_path, runner=None, progress=None):
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
    profile_dir = _profile_directory(prefix, config["template"])
    target_paths = {
        stage: _target_file(prefix, profile_dir, stem)
        for stage, stem in config["targets"].items()
        if not config["reuse_calib_params"][stage]
    }
    profile = _profile_provenance(profile_dir, target_paths, config)
    software = _software_provenance()
    workflow_started = time.monotonic()
    _emit_progress(
        progress, "workflow_start", batch=config["batch_name"],
        machine="{}-{}".format(
            config["machine_version"], config["machine_num"]))

    with _machine_lock(control):
        state_path = control / "state.json"
        state = _load_state(state_path, machine)
        reused_parameters, accepted_hashes = _prepare_parameter_reuse(
            output, control, state, config)
        config["parameter_inputs"] = {
            stage: (record["seed"]["geometry_sha256"]
                    if "seed" in record else record["sha256"])
            for stage, record in reused_parameters.items()}
        children = tuple(machine.iterdir())
        dataset_paths = {}
        for index, role in enumerate(_DATASET_PATTERNS, 1):
            _emit_progress(
                progress, "dataset_start", index=index,
                total=len(_DATASET_PATTERNS), role=role)
            stage = next(stage for stage, value in _STAGE_ROLES.items()
                         if value == role)
            dataset_path = (None if config["reuse_calib_params"][stage]
                            else _discover_dataset_path(
                                machine, role, config["datasets"].get(role), children))
            dataset_paths[role] = dataset_path
            _emit_progress(
                progress, "dataset_complete", index=index,
                total=len(_DATASET_PATTERNS), role=role,
                parameters_reused=config["reuse_calib_params"][stage],
                dataset=(dataset_path.name
                         if dataset_path is not None else None))

        found = [(role, path) for role, path in dataset_paths.items()
                 if path is not None]
        _emit_progress(
            progress, "dataset_validation_begin", total=len(found),
            reused_count=len(reused_parameters))
        cache_path = control / INPUT_HASH_CACHE
        hash_cache = _load_input_hash_cache(cache_path)
        stored_binding = state.get("binding")
        pending = state.get("pending_recalibration")
        if pending is not None:
            stored_binding = pending.get("target_binding")
        expected_datasets = (
            stored_binding.get("datasets", {})
            if isinstance(stored_binding, dict)
            and isinstance(stored_binding.get("datasets", {}), dict)
            else {})
        datasets = {role: None for role in _DATASET_PATTERNS}
        current_cache_keys = set()
        for validation_index, (role, dataset_path) in enumerate(found, 1):
            key = str(dataset_path.resolve())
            current_cache_keys.add(key)

            def report(event, details, *, index=validation_index):
                _emit_progress(
                    progress, "dataset_validation_" + event,
                    index=index, total=len(found), **details)

            record, entry, unused_statistics = _fingerprint_dataset(
                dataset_path, role,
                cached=hash_cache["datasets"].get(key), progress=report,
                expected_record=expected_datasets.get(role))
            datasets[role] = record
            hash_cache["datasets"][key] = entry
            _write_input_hash_cache(cache_path, hash_cache)
        hash_cache["datasets"] = {
            key: value for key, value in hash_cache["datasets"].items()
            if key in current_cache_keys}
        _write_input_hash_cache(cache_path, hash_cache)
        _emit_progress(
            progress, "dataset_validation_end", total=len(found))
        _emit_progress(progress, "workflow_state_start")

        current_binding = _binding(
            config, profile, software, datasets)
        stored_software = (
            state.get("binding", {}).get("software")
            if isinstance(state.get("binding"), dict) else None)
        force_provenance_stages = {
            stage for stage, role in _STAGE_ROLES.items()
            if (not _STRICT_PROVENANCE
                and stored_software is not None
                and stored_software != current_binding["software"]
                and datasets[role] is not None)
        }
        pending = state.get("pending_recalibration")
        if pending is not None:
            if current_binding != pending["target_binding"]:
                if _STRICT_PROVENANCE:
                    raise TaskError(
                        "recalibration inputs changed while a replacement is "
                        "pending; restore the selected replacement datasets and "
                        "rerun")
                for stage, delivery in pending.get(
                        "previous_deliveries", {}).items():
                    _verify_previous_delivery(output, delivery)
                    state.setdefault("stages", {})[stage] = {
                        "status": "completed", "delivery": deepcopy(delivery)}
                state.pop("pending_recalibration", None)
                pending = None
            else:
                target_binding = pending["target_binding"]
        if pending is None:
            (target_binding, changed_roles, changed_profile_inputs,
             profile_stages) = _merge_binding(
                state.get("binding"), current_binding)
            if profile_stages:
                _preflight_profile_recalibration(
                    profile_dir, target_paths, config, datasets, output,
                    profile_stages)
            if changed_roles or profile_stages:
                pending = _start_recalibration(
                    state, target_binding, changed_roles, datasets, output,
                    profile_stages=profile_stages,
                    changed_profile_inputs=changed_profile_inputs)
                state["pending_recalibration"] = pending
            else:
                state["binding"] = target_binding
        recalibration_stages = set(
            pending["affected_stages"] if pending is not None else ())
        previous_deliveries = (
            pending.get("previous_deliveries", {})
            if pending is not None else {})
        state["user_config_sha256"] = config["config_sha256"]
        _atomic_json(state_path, state)

        summary = {
            "schema_version": SCHEMA_VERSION,
            "kind": "machine_calibration_workflow_summary",
            "machine_dir": str(machine),
            "machine_root": str(config["machine_root"]),
            "batch_name": config["batch_name"],
            "machine_version": config["machine_version"],
            "machine_num": config["machine_num"],
            "output_dir": str(output),
        }
        tasks = control / "tasks"
        deliveries = {}
        stage_actions = {}
        camera_job = (
            "camera_calibration"
            if config["camera_mode"] == "standard" else CAMERA_RS_JOB)
        imu_job = (
            "camera_imu_calibration"
            if config["camera_imu_mode"] == "standard"
            else CAMERA_IMU_RS_JOB)
        stage_specs = (
                ("camera", "stereo", camera_job, None),
                ("camera_imu", "stereo_imu", imu_job, "camera"),
                ("camera_baselink", "stereo_baselink", CAMERA_BASELINK_JOB, "camera"),
                ("mono_camera", "mono", "camera_calibration", None),
                ("mono_camera_baselink", "mono_baselink", CAMERA_BASELINK_JOB, "mono_camera"))
        counted_stages = [
            stage for stage, role, unused_job, unused_parent in stage_specs
            if datasets[role] is not None or stage in reused_parameters]
        stage_indices = {
            stage: index for index, stage in enumerate(counted_stages, 1)}
        for stage, role, expected_job, parent in stage_specs:
            record = datasets[role]
            stage_index = stage_indices.get(stage)
            stage_total = len(counted_stages)
            if stage in reused_parameters:
                reused = reused_parameters[stage]
                if "seed" in reused:
                    deliveries[stage] = reused["seed"]
                stored = state["stages"].get(stage)
                if not isinstance(stored, dict) or stored.get("status") != "completed":
                    state["stages"][stage] = {"status": "reused_parameters"}
                stage_actions[stage] = "reused_parameters"
                summary[stage] = _stage_summary(
                    "reused_parameters", reused["path"], "reused_parameters")
                _emit_progress(
                    progress, "stage_parameters_reused", index=stage_index,
                    total=stage_total, stage=stage)
                continue
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
                parent_role = _STAGE_ROLES[parent]
                if datasets[parent_role] is None:
                    status = "waiting_for_prerequisite_dataset"
                    message = (
                        "dataset {} is waiting for prerequisite {} dataset; "
                        "other independent branches continue"
                        .format(record["name"], parent_role))
                else:
                    status = "blocked_missing_camera_calibration"
                    message = (
                        "dataset {} exists, but the prerequisite {} stage has "
                        "no completed result"
                        .format(record["name"], parent))
            if status is not None:
                state["stages"][stage] = {"status": status}
                summary[stage] = _stage_summary(status, message=message)
                _emit_progress(
                    progress, "stage_skip", index=stage_index,
                    total=stage_total, stage=stage, status=status,
                    dataset=(record["name"] if record is not None else None),
                    message=message)
                continue
            camera_dependency = ({
                "path": camera_delivery["result"],
                "sha256": camera_delivery["files"]["result"],
            } if camera_delivery is not None else None)
            name = record["name"]
            stage_started = time.monotonic()
            _emit_progress(
                progress, "stage_start", index=stage_index,
                total=stage_total, stage=stage, dataset=name,
                job=expected_job)
            try:
                task = build_task(
                    profile_dir, stage, record["path"], name,
                    camera_result=(camera_delivery["result"]
                                   if camera_delivery is not None else None),
                    mode=config["camera_imu_mode"],
                    camera_baselink=baselink_config,
                    camera_mode=config["camera_mode"],
                    recompute_extrinsics=config[
                        "camera_imu_recompute_extrinsics"],
                    target_path=target_paths.get(stage))
                delivery, action = _run_stage(
                    prefix=prefix, runner=runner, output=output, tasks=tasks,
                    state_path=state_path, state=state,
                    stage_name=stage, task=task, output_name=name,
                    expected_job=expected_job, dataset_record=record,
                    camera_dependency=camera_dependency,
                    recalibrate=stage in recalibration_stages,
                    previous_delivery=previous_deliveries.get(stage),
                    force_recalibrate=(
                        stage in force_provenance_stages
                        or stage in recalibration_stages
                        and parent is not None
                        and stage_actions.get(parent) == "executed"),
                    hash_cache_path=cache_path, progress=progress)
            except Exception as error:
                _emit_progress(
                    progress, "stage_failed", index=stage_index,
                    total=stage_total, stage=stage, dataset=name,
                    elapsed_s=time.monotonic() - stage_started,
                    error=str(error))
                raise
            deliveries[stage] = delivery
            stage_actions[stage] = action
            summary[stage] = _stage_summary("completed", delivery["result"], action)
            _emit_progress(
                progress, "stage_complete", index=stage_index,
                total=stage_total, stage=stage, dataset=name,
                action=action, result=delivery["result"],
                elapsed_s=time.monotonic() - stage_started)
        try:
            _emit_progress(progress, "calib_params_start")
            summary["calib_params"] = _update_calib_params(
                machine=config["machine_root"], output=output,
                profile=profile_dir,
                deliveries={stage: delivery for stage, delivery in deliveries.items()
                            if stage not in reused_parameters}, state=state,
                camera_mode=config["camera_mode"],
                camera_imu_mode=config["camera_imu_mode"],
                state_path=state_path, reused=reused_parameters,
                accepted_hashes=accepted_hashes)
            state.pop("calib_params_failure", None)
            _emit_progress(
                progress, "calib_params_complete",
                artifacts={
                    name: artifact.get("action")
                    for name, artifact in summary["calib_params"].items()
                })
        except Exception as error:
            state["calib_params_failure"] = {
                "type": type(error).__name__, "message": str(error)}
            _atomic_json(state_path, state)
            raise
        if pending is not None:
            state["binding"] = deepcopy(pending["target_binding"])
            state.pop("pending_recalibration", None)
        _atomic_json(state_path, state)
        _emit_progress(
            progress, "workflow_complete",
            elapsed_s=time.monotonic() - workflow_started)
        return summary
