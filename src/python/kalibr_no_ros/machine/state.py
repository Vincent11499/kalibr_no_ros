"""Durable machine checkpoints, locks, and recalibration decisions."""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path
import fcntl
import json
import os
import tempfile

from ..task.document import TaskError
from ..version import (
    STATE_SCHEMA_VERSION,
    SUPPORTED_STATE_SCHEMA_VERSIONS,
    calibration_behavior_version,
    supports_schema_version,
)
from .config import (
    _DATASET_PATTERNS, _RECALIBRATION_STAGES, _STAGE_ROLES,
    _STRICT_PROVENANCE, _TEMPLATES,
)
from .result_checks import _delivery_name, _verify_previous_delivery


STATE_KIND = "machine_calibration_state"


CONTROL_DIRECTORY = ".kalibr-cedarobot"


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


def _load_state(path, machine):
    if not path.exists():
        return {
            "schema_version": STATE_SCHEMA_VERSION,
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
            or not supports_schema_version(
                state.get("schema_version"), SUPPORTED_STATE_SCHEMA_VERSIONS)
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
        "path": legacy["name"],
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
    use_semantic = (
        isinstance(existing.get("semantic_files"), dict)
        and isinstance(current.get("semantic_files"), dict)
    )
    old_files = existing.get("semantic_files" if use_semantic else "files")
    new_files = current.get("semantic_files" if use_semantic else "files")
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
    old_software = existing.get("software")
    new_software = current.get("software")
    if (calibration_behavior_version(old_software)
            != calibration_behavior_version(new_software)):
        changed.update(_DATASET_PATTERNS)
    merged["software"] = deepcopy(new_software)
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
