"""Machine workflow configuration, identities, and stage contracts."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import hashlib
import json
import re

from ..task.document import TaskError, load_yaml
from ..task.jobs import CAMERA_BASELINK_JOB
from ..version import (
    CONFIG_SCHEMA_VERSION,
    SUPPORTED_CONFIG_SCHEMA_VERSIONS,
    supports_schema_version,
)


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


_MACHINE_NAME = re.compile(
    r"^(?P<machine_version>[A-Za-z][A-Za-z0-9]*)-(?P<machine_num>[0-9]+)$")


WORKFLOW_KIND = "machine_calibration_workflow"


# Test-phase policy; production re-enablement is documented in docs.
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


_MACHINE_CAMERA_MODELS = frozenset((
    "pinhole-equi",
    "pinhole-radtan5",
    "pinhole-radtan8",
))


def _json_sha256(value):
    payload = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


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
    if not supports_schema_version(
            value.get("schema_version"), SUPPORTED_CONFIG_SCHEMA_VERSIONS):
        raise TaskError(
            "machine workflow schema_version must be one of {}".format(
                ", ".join(sorted(SUPPORTED_CONFIG_SCHEMA_VERSIONS))))
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
        from ..task.validation import validate_options
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
        "schema_version": CONFIG_SCHEMA_VERSION,
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
