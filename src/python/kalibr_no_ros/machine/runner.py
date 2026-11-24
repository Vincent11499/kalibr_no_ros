"""Run and resume machine calibration stages and parameter publication."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import os
import time

from ..filesystem import file_sha256 as _sha256
from ..results.delivery import MACHINE_LAYOUT
from ..task.document import TaskError, dump_yaml, load_yaml
from ..task.jobs import CAMERA_BASELINK_JOB, CAMERA_IMU_RS_JOB, CAMERA_RS_JOB
from ..version import (
    RESULT_SCHEMA_VERSION,
    STATE_SCHEMA_VERSION,
    calibration_behavior_version,
)
from .calib_params import build_calib_params, write_calib_param
from .config import (
    _DATASET_PATTERNS, _STAGE_ROLES, _STRICT_PROVENANCE,
    load_workflow_config, machine_identity,
)
from .datasets import (
    INPUT_HASH_CACHE, DatasetChangedError, _discover_dataset_path,
    _fingerprint_dataset, _load_input_hash_cache, _write_input_hash_cache,
    bind_cache_entry, cache_entry_can_migrate, cache_entry_is_bound,
)
from .parameter_reuse import (
    camera_seed, parameter_path, semantic_sha256, validate_reused_stage,
)
from .result_checks import (
    _delivery_exists, _validate_delivery, _verify_previous_delivery,
)
from .state import (
    CONTROL_DIRECTORY, _atomic_json, _binding, _ensure_managed_directory,
    _load_state, _machine_lock, _merge_binding, _start_recalibration,
)
from .task_templates import (
    _profile_directory, _profile_provenance, _software_provenance,
    _target_file, build_task,
)


_RETIRED_CALIB_PARAMS = {
    "stereo": Path("stereo_camera") / "stereo_calib.yaml",
}


_RENAMED_CALIB_PARAMS = {
    "vision": (
        Path("vision") / "vision_calib.yaml",
        Path("vision") / "stereo_imu_calib.yaml",
    ),
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
            target_path=targets.get(stage),
            dataset_type=record.get("dataset_type", "directory"),
            bag_topics=record.get("bag_topics"))


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
        dataset_record, camera_dependency=None, progress=None, stage=None):
    """Return a verified candidate without mutating the trusted cache."""
    def report(event, details):
        if event != "complete":
            _emit_progress(
                progress, "dataset_recheck_" + event,
                stage=stage, **details)

    current_dataset, entry, statistics = _fingerprint_dataset(
        dataset_record["path"], dataset_record["role"],
        bag_topics=dataset_record.get("bag_topics"), progress=report)
    if current_dataset != dataset_record:
        raise DatasetChangedError(
            "dataset changed during calibration; keep capture files immutable "
            "and use a new batch directory")
    if camera_dependency is not None:
        current_camera_hash = _sha256(camera_dependency["path"])
        if current_camera_hash != camera_dependency["sha256"]:
            raise DatasetChangedError(
                "camera calibration changed during downstream calibration; "
                "use a new batch directory")
    _emit_progress(
        progress, "dataset_recheck_complete",
        stage=stage, dataset=dataset_record["name"], **statistics)
    return entry


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
    verified = {}

    def before_publish():
        verified["cache_entry"] = _verify_stage_inputs_unchanged(
            dataset_record, camera_dependency=camera_dependency,
            progress=progress, stage=stage_name)

    try:
        runner(
            prefix, task_path, output, expected_job,
            force=previous_delivery is not None,
            _before_publish=before_publish)
        if "cache_entry" not in verified:
            raise TaskError(
                "calibration runner did not perform the required pre-publish "
                "input verification")
        delivery = _completed_stage(
            output, output_name, expected_job, dataset_record, task_path,
            camera_dependency=camera_dependency)
        if hash_cache_path is not None:
            cache = _load_input_hash_cache(hash_cache_path)
            key = str(Path(dataset_record["path"]).resolve())
            cache["datasets"][key] = bind_cache_entry(
                verified["cache_entry"], stage_name, delivery)
            _write_input_hash_cache(hash_cache_path, cache)
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


def _run_task(
        prefix, config, output, expected_job, force=False,
        _before_publish=None):
    overrides = {"_delivery_layout": MACHINE_LAYOUT}
    if _before_publish is not None:
        overrides["_before_publish"] = _before_publish
    if expected_job == CAMERA_BASELINK_JOB:
        from ..task.camera_baselink import run_camera_baselink
        return run_camera_baselink(
            config, output, force=force, **overrides)
    from ..task.runner import run_task
    return run_task(
        prefix, config, output, expected_job, force=force,
        **overrides)


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
        state["schema_version"] = STATE_SCHEMA_VERSION
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
        pending = state.get("pending_recalibration")
        stored_binding = state.get("binding")
        stored_datasets = (
            stored_binding.get("datasets", {})
            if isinstance(stored_binding, dict)
            and isinstance(stored_binding.get("datasets", {}), dict)
            else {})
        pending_datasets = (
            pending.get("target_binding", {}).get("datasets", {})
            if isinstance(pending, dict) else {})
        datasets = {role: None for role in _DATASET_PATTERNS}
        pending_completed_stages = set()
        cache_changed = False
        for validation_index, (role, dataset_path) in enumerate(found, 1):
            key = str(dataset_path.resolve())
            cached = hash_cache["datasets"].get(key)
            stage = next(
                name for name, value in _STAGE_ROLES.items()
                if value == role)

            def report(event, details, *, index=validation_index):
                _emit_progress(
                    progress, "dataset_validation_" + event,
                    index=index, total=len(found), **details)

            record, entry, unused_statistics = _fingerprint_dataset(
                dataset_path, role,
                bag_topics=config.get("bag_topics", {}).get(role),
                progress=report)
            datasets[role] = record

            expected = stored_datasets.get(role)
            trusted_delivery = None
            expected_source = "binding"
            stored_stage = state.get("stages", {}).get(stage)
            stage_delivery = (
                stored_stage.get("delivery")
                if isinstance(stored_stage, dict)
                and stored_stage.get("status") == "completed"
                and isinstance(stored_stage.get("delivery"), dict)
                else None)
            pending_record = (
                pending_datasets.get(role)
                if isinstance(pending_datasets, dict) else None)
            if (pending_record is not None and stage_delivery is not None
                    and cache_entry_is_bound(
                        cached, stage, stage_delivery, pending_record)):
                expected = pending_record
                trusted_delivery = stage_delivery
                expected_source = "pending"
                pending_completed_stages.add(stage)
            elif isinstance(pending, dict):
                trusted_delivery = pending.get(
                    "previous_deliveries", {}).get(stage)
            if trusted_delivery is None:
                trusted_delivery = stage_delivery

            equivalent = record == expected
            migrated = (
                not equivalent and expected is not None
                and cache_entry_can_migrate(cached, expected, entry))
            if (equivalent or migrated) and trusted_delivery is not None:
                _verify_previous_delivery(output, trusted_delivery)
                if migrated:
                    if expected_source == "pending":
                        pending["target_binding"]["datasets"][role] = record
                    else:
                        state["binding"]["datasets"][role] = record
                        stored_datasets[role] = record
                        if (isinstance(pending_datasets, dict)
                                and pending_datasets.get(role) == expected):
                            pending["target_binding"]["datasets"][role] = record
                hash_cache["datasets"][key] = bind_cache_entry(
                    entry, stage, trusted_delivery)
                cache_changed = True
        if cache_changed:
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
                and calibration_behavior_version(stored_software)
                != calibration_behavior_version(current_binding["software"])
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
            "schema_version": RESULT_SCHEMA_VERSION,
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
                    target_path=target_paths.get(stage),
                    dataset_type=record.get("dataset_type", "directory"),
                    bag_topics=record.get("bag_topics"))
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
                        and stage not in pending_completed_stages),
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
