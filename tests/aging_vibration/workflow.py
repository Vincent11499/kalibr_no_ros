"""Study manifest, safe organization, and release command orchestration."""

from __future__ import annotations

import copy
import datetime as dt
import json
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Iterable

import yaml


SCHEMA_VERSION = "1.0.0"
KIND = "stereo_aging_vibration_study"
VERIFY_OPTIONS = {
    "window_half_size_px": 7,
    "max_displacement_px": 2.0,
    "synchronization_tolerance_s": 0.0001,
    "detector_opencv_threads": 1,
    "visualizations": True,
    "max_frames_per_camera": 30,
    "max_pairs": 30,
}


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _write_yaml(path: Path, value: dict[str, Any]) -> None:
    _atomic_text(
        path,
        yaml.safe_dump(value, allow_unicode=True, sort_keys=False, width=120),
    )


def _write_json(path: Path, value: Any) -> None:
    _atomic_text(path, json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def load_manifest(path: Path | str) -> dict[str, Any]:
    manifest_path = Path(path).resolve()
    value = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("study manifest must be a YAML mapping")
    if value.get("schema_version") != SCHEMA_VERSION or value.get("kind") != KIND:
        raise ValueError("unsupported study manifest schema or kind")
    value["_manifest_path"] = manifest_path
    root_value = value.get("machine_root", ".")
    root = Path(root_value)
    if not root.is_absolute():
        root = (manifest_path.parent / root).resolve()
    value["_machine_root"] = root
    return value


def save_manifest(manifest: dict[str, Any]) -> None:
    path = Path(manifest["_manifest_path"])
    value = {key: copy.deepcopy(item) for key, item in manifest.items() if not key.startswith("_")}
    _write_yaml(path, value)


def machine_path(manifest: dict[str, Any], value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else Path(manifest["_machine_root"]) / path


def repository_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else repository_root() / path


def _relative(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def _calibration_result_paths(root: Path, dataset_id: str) -> tuple[str, str]:
    base = root / "result" / (dataset_id + "_equi")
    return _relative(root, base.with_suffix(".yaml")), _relative(root, base)


def create_manifest(
    machine_root: Path | str,
    machine_id: str,
    before_calibration: str,
    before_tests: list[str],
    after_calibration: str,
    after_tests: list[str],
) -> Path:
    root = Path(machine_root).resolve()
    manifest_path = root / "study_manifest.yaml"
    if manifest_path.exists():
        raise FileExistsError(f"study manifest already exists: {manifest_path}")
    if len(before_tests) != 3 or len(after_tests) != 3:
        raise ValueError("this workflow requires exactly three validation datasets per phase")

    before_result, before_artifacts = _calibration_result_paths(root, before_calibration)
    after_result, after_artifacts = _calibration_result_paths(root, after_calibration)
    phases: dict[str, Any] = {}
    for phase, calibration, tests, calibration_result, artifacts in (
        ("before", before_calibration, before_tests, before_result, before_artifacts),
        ("after", after_calibration, after_tests, after_result, after_artifacts),
    ):
        runs = []
        for repeat, dataset_id in enumerate(tests, 1):
            result_base = root / "result" / ("verification_" + dataset_id)
            results = {"before": _relative(root, result_base)}
            if phase == "after":
                results["after"] = _relative(root, Path(str(result_base) + "_after"))
            runs.append(
                {
                    "repeat": repeat,
                    "dataset_id": dataset_id,
                    "dataset": dataset_id,
                    "results": results,
                }
            )
        phases[phase] = {
            "calibration_dataset_id": calibration,
            "calibration_dataset": calibration,
            "calibration_result": calibration_result,
            "calibration_artifacts": artifacts,
            "validation_runs": runs,
        }

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "kind": KIND,
        "machine_id": str(machine_id),
        "machine_root": ".",
        "created_at": _utc_now(),
        "classification": {
            "source": "explicit_before_after_assignment_confirmed_by_capture_time",
            "before_latest_dataset": before_tests[-1],
            "after_earliest_dataset": after_tests[0],
        },
        "kalibr": {
            "binary": "install/release/bin/kalibr-noros",
            "task_template": "config/mechine/stereo_camera_calibration_task_pinhole_equi.yaml",
            "target": "config/mechine/aprilgrid.yaml",
            "calibration_label": "1316_equi",
            "verify_options": copy.deepcopy(VERIFY_OPTIONS),
        },
        "phases": phases,
        "report_output": "comparison_report",
    }
    _write_yaml(manifest_path, manifest)
    return manifest_path


def _completed_run(directory: Path) -> bool:
    manifest = directory / "run_manifest.json"
    if not manifest.is_file():
        return False
    try:
        return json.loads(manifest.read_text(encoding="utf-8")).get("status") == "completed"
    except (OSError, ValueError):
        return False


def inspect_study(manifest: dict[str, Any]) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    phases = manifest.get("phases", {})
    if set(phases) != {"before", "after"}:
        errors.append("manifest must contain exactly before and after phases")
    for phase_name in ("before", "after"):
        phase = phases.get(phase_name, {})
        calibration_dataset = machine_path(manifest, phase.get("calibration_dataset", ""))
        if not (calibration_dataset / "dataset.yaml").is_file():
            errors.append(f"missing {phase_name} calibration dataset: {calibration_dataset}")
        calibration_result = machine_path(manifest, phase.get("calibration_result", ""))
        if not calibration_result.is_file():
            errors.append(f"missing {phase_name} calibration result: {calibration_result}")
        artifacts = machine_path(manifest, phase.get("calibration_artifacts", ""))
        if not _completed_run(artifacts):
            errors.append(f"calibration run is not completed: {artifacts}")
        runs = phase.get("validation_runs", [])
        if len(runs) != 3:
            errors.append(f"{phase_name} must contain exactly three validation runs")
        for run in runs:
            dataset = machine_path(manifest, run.get("dataset", ""))
            if not (dataset / "dataset.yaml").is_file():
                errors.append(f"missing validation dataset: {dataset}")
            required = ("before",) if phase_name == "before" else ("before", "after")
            for calibration_phase in required:
                result = machine_path(manifest, run.get("results", {}).get(calibration_phase, ""))
                if not _completed_run(result):
                    errors.append(f"validation run is not completed: {result}")
                for filename in ("fixed_camera_validation.json", "fixed_camera_validation.csv"):
                    if not (result / filename).is_file():
                        errors.append(f"missing {filename}: {result}")
    if manifest.get("migration", {}).get("applied"):
        warnings.append("study directories are already organized")
        migration_path = Path(manifest["_machine_root"]) / manifest["migration"].get(
            "manifest", "migration_manifest.json"
        )
        if not migration_path.is_file():
            errors.append(f"missing migration manifest: {migration_path}")
        else:
            migration = json.loads(migration_path.read_text(encoding="utf-8"))
            for move in migration.get("moves", []):
                source = machine_path(manifest, move["source"])
                destination = machine_path(manifest, move["destination"])
                if source.exists():
                    errors.append(f"migration source still exists: {source}")
                if not destination.exists():
                    errors.append(f"migration destination is missing: {destination}")
                    continue
                current = _tree_stats(destination)
                expected = {
                    "file_count": int(move["file_count"]),
                    "total_bytes": int(move["total_bytes"]),
                }
                if current != expected:
                    errors.append(
                        f"migration destination statistics changed: {destination}; "
                        f"expected {expected}, found {current}"
                    )
    return {
        "status": "ok" if not errors else "error",
        "errors": errors,
        "warnings": warnings,
    }


def _tree_stats(path: Path) -> dict[str, int]:
    if path.is_file():
        return {"file_count": 1, "total_bytes": path.stat().st_size}
    count = 0
    total = 0
    for child in path.rglob("*"):
        if child.is_file() and not child.is_symlink():
            count += 1
            total += child.stat().st_size
    return {"file_count": count, "total_bytes": total}


def _assert_no_symlinks(path: Path) -> None:
    if path.is_symlink():
        raise ValueError(f"refusing to move symlink: {path}")
    if path.is_dir():
        for child in path.rglob("*"):
            if child.is_symlink():
                raise ValueError(f"refusing to move tree containing symlink: {child}")


def build_migration_plan(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    if manifest.get("migration", {}).get("applied"):
        return []
    root = Path(manifest["_machine_root"])
    entries: list[tuple[Path, Path]] = []
    locations = {
        "before": {
            "calibration": root / "01_before_aging_vibration" / "calibration_dataset",
            "validation": root / "01_before_aging_vibration" / "validation_datasets",
            "calibration_results": root / "result" / "01_before_calibration",
        },
        "after": {
            "calibration": root / "02_after_aging_vibration" / "calibration_dataset",
            "validation": root / "02_after_aging_vibration" / "validation_datasets",
            "calibration_results": root / "result" / "04_after_calibration",
        },
    }
    for phase_name in ("before", "after"):
        phase = manifest["phases"][phase_name]
        calibration_dataset = machine_path(manifest, phase["calibration_dataset"])
        entries.append((calibration_dataset, locations[phase_name]["calibration"] / calibration_dataset.name))
        for run in phase["validation_runs"]:
            source = machine_path(manifest, run["dataset"])
            destination = locations[phase_name]["validation"] / f"repeat_{run['repeat']:02d}_{run['dataset_id']}"
            entries.append((source, destination))
        result_group = locations[phase_name]["calibration_results"]
        result = machine_path(manifest, phase["calibration_result"])
        artifacts = machine_path(manifest, phase["calibration_artifacts"])
        entries.extend([(result, result_group / result.name), (artifacts, result_group / artifacts.name)])
        for suffix in (".report.html", ".report.pdf"):
            source = result.with_suffix(suffix)
            entries.append((source, result_group / source.name))

    result_groups = {
        ("before", "before"): root / "result" / "02_before_validation_with_before_calibration",
        ("after", "before"): root / "result" / "03_after_validation_with_before_calibration",
        ("after", "after"): root / "result" / "05_after_validation_with_after_calibration",
    }
    for phase_name in ("before", "after"):
        for run in manifest["phases"][phase_name]["validation_runs"]:
            for calibration_phase, result_value in run["results"].items():
                source = machine_path(manifest, result_value)
                group = result_groups[(phase_name, calibration_phase)]
                clean_name = "verification_" + run["dataset_id"] + f"_using_{calibration_phase}_calibration"
                destination = group / f"repeat_{run['repeat']:02d}_{clean_name}"
                entries.append((source, destination))

    seen_sources: set[Path] = set()
    seen_destinations: set[Path] = set()
    plan = []
    for source, destination in entries:
        source = source.resolve()
        destination = destination.resolve()
        if source in seen_sources or destination in seen_destinations:
            raise ValueError(f"duplicate migration source or destination: {source} -> {destination}")
        seen_sources.add(source)
        seen_destinations.add(destination)
        if not source.exists():
            raise FileNotFoundError(source)
        if destination.exists():
            raise FileExistsError(destination)
        _assert_no_symlinks(source)
        plan.append(
            {
                "source": _relative(root, source),
                "destination": _relative(root, destination),
                **_tree_stats(source),
            }
        )
    return plan


def _replace_manifest_paths(manifest: dict[str, Any], mapping: dict[str, str]) -> None:
    for phase in manifest["phases"].values():
        for key in ("calibration_dataset", "calibration_result", "calibration_artifacts"):
            phase[key] = mapping.get(phase[key], phase[key])
        for run in phase["validation_runs"]:
            run["dataset"] = mapping.get(run["dataset"], run["dataset"])
            for key, value in list(run["results"].items()):
                run["results"][key] = mapping.get(value, value)


def apply_migration(manifest: dict[str, Any], plan: list[dict[str, Any]]) -> dict[str, Any]:
    if not plan:
        return {"status": "already_organized", "moves": []}
    root = Path(manifest["_machine_root"])
    moved: list[tuple[Path, Path]] = []
    created_parents: set[Path] = set()
    try:
        for entry in plan:
            source = root / entry["source"]
            destination = root / entry["destination"]
            if not destination.parent.exists():
                destination.parent.mkdir(parents=True, exist_ok=True)
                created_parents.add(destination.parent)
            source.rename(destination)
            moved.append((source, destination))
    except Exception:
        for source, destination in reversed(moved):
            if destination.exists() and not source.exists():
                source.parent.mkdir(parents=True, exist_ok=True)
                destination.rename(source)
        for parent in sorted(created_parents, key=lambda item: len(item.parts), reverse=True):
            try:
                parent.rmdir()
            except OSError:
                pass
        raise

    mapping = {entry["source"]: entry["destination"] for entry in plan}
    _replace_manifest_paths(manifest, mapping)
    manifest["migration"] = {
        "applied": True,
        "applied_at": _utc_now(),
        "manifest": "migration_manifest.json",
    }
    save_manifest(manifest)
    migration = {
        "schema_version": SCHEMA_VERSION,
        "kind": "stereo_aging_vibration_migration",
        "machine_id": manifest["machine_id"],
        "applied_at": manifest["migration"]["applied_at"],
        "moves": plan,
        "totals": {
            "entries": len(plan),
            "file_count": sum(entry["file_count"] for entry in plan),
            "total_bytes": sum(entry["total_bytes"] for entry in plan),
        },
    }
    _write_json(root / "migration_manifest.json", migration)
    return {"status": "completed", "moves": plan}


def _release_binary(manifest: dict[str, Any]) -> Path:
    binary = repository_path(manifest["kalibr"]["binary"])
    if not os.access(binary, os.X_OK):
        raise FileNotFoundError(f"release binary is missing or not executable: {binary}")
    return binary


def build_calibrate_command(manifest: dict[str, Any], task: Path, output: Path, force: bool = False) -> list[str]:
    command = [
        str(_release_binary(manifest)),
        "calibrate",
        "cameras",
        "--config",
        str(task),
        "--output-dir",
        str(output),
    ]
    if force:
        command.append("--force")
    return command


def build_verify_command(
    manifest: dict[str, Any], calibration: Path, dataset: Path, output: Path, force: bool = False
) -> list[str]:
    options = manifest["kalibr"].get("verify_options", VERIFY_OPTIONS)
    command = [
        str(_release_binary(manifest)),
        "verify",
        "cameras",
        "--calibration",
        f"{manifest['kalibr'].get('calibration_label', '1316_equi')}={calibration}",
        "--dataset",
        str(dataset),
        "--target",
        str(repository_path(manifest["kalibr"]["target"])),
        "--output-dir",
        str(output),
        "--window-half-size-px",
        str(options["window_half_size_px"]),
        "--max-displacement-px",
        str(options["max_displacement_px"]),
        "--synchronization-tolerance-s",
        str(options["synchronization_tolerance_s"]),
        "--detector-opencv-threads",
        str(options["detector_opencv_threads"]),
    ]
    if options.get("visualizations", True):
        command.append("--visualizations")
    command.extend(
        [
            "--max-frames-per-camera",
            str(options["max_frames_per_camera"]),
            "--max-pairs",
            str(options["max_pairs"]),
        ]
    )
    if force:
        command.append("--force")
    return command


def prepare_runtime_task(
    manifest: dict[str, Any], phase_name: str, write: bool = True
) -> tuple[Path, Path, str]:
    phase = manifest["phases"][phase_name]
    template_path = repository_path(manifest["kalibr"]["task_template"])
    task = yaml.safe_load(template_path.read_text(encoding="utf-8"))
    dataset = machine_path(manifest, phase["calibration_dataset"])
    output_name = phase["calibration_dataset_id"] + "_equi"
    task["dataset"]["path"] = str(dataset)
    task["target"]["path"] = str((template_path.parent / task["target"]["path"]).resolve())
    if task.get("initialization", {}).get("path"):
        task["initialization"]["path"] = str(
            (template_path.parent / task["initialization"]["path"]).resolve()
        )
    task["output"]["name"] = output_name
    runtime_dir = Path(manifest["_machine_root"]) / "result" / "runtime_configs"
    runtime_task = runtime_dir / f"{phase_name}_camera_calibration_task_pinhole_equi.yaml"
    if write:
        _write_yaml(runtime_task, task)
    output = machine_path(manifest, phase["calibration_result"]).parent
    return runtime_task, output, output_name


def run_calibration(manifest: dict[str, Any], phase_name: str, dry_run: bool, force: bool) -> list[str]:
    phase = manifest["phases"][phase_name]
    current_result = machine_path(manifest, phase["calibration_result"])
    current_artifacts = machine_path(manifest, phase["calibration_artifacts"])
    if current_result.is_file() and _completed_run(current_artifacts) and not force:
        return ["reuse", str(current_result)]
    runtime_task, output, output_name = prepare_runtime_task(manifest, phase_name, write=not dry_run)
    command = build_calibrate_command(manifest, runtime_task, output, force)
    if dry_run:
        return command
    output.mkdir(parents=True, exist_ok=True)
    subprocess.run(command, cwd=repository_root(), check=True)
    result = output / (output_name + ".yaml")
    artifacts = output / output_name
    if not result.is_file() or not _completed_run(artifacts):
        raise RuntimeError("release calibration completed without a complete managed result")
    phase["calibration_result"] = _relative(Path(manifest["_machine_root"]), result)
    phase["calibration_artifacts"] = _relative(Path(manifest["_machine_root"]), artifacts)
    save_manifest(manifest)
    return command


def _verification_group(root: Path, test_phase: str, calibration_phase: str) -> Path:
    groups = {
        ("before", "before"): root / "result" / "02_before_validation_with_before_calibration",
        ("after", "before"): root / "result" / "03_after_validation_with_before_calibration",
        ("after", "after"): root / "result" / "05_after_validation_with_after_calibration",
    }
    if (test_phase, calibration_phase) not in groups:
        raise ValueError("unsupported test/calibration phase combination")
    return groups[(test_phase, calibration_phase)]


def run_verification(
    manifest: dict[str, Any], test_phase: str, calibration_phase: str, dry_run: bool, force: bool
) -> list[list[str]]:
    root = Path(manifest["_machine_root"])
    calibration = machine_path(manifest, manifest["phases"][calibration_phase]["calibration_result"])
    if not calibration.is_file():
        raise FileNotFoundError(calibration)
    commands: list[list[str]] = []
    for run in manifest["phases"][test_phase]["validation_runs"]:
        current = run.get("results", {}).get(calibration_phase)
        if current and _completed_run(machine_path(manifest, current)) and not force:
            commands.append(["reuse", str(machine_path(manifest, current))])
            continue
        dataset = machine_path(manifest, run["dataset"])
        name = (
            f"repeat_{run['repeat']:02d}_verification_{run['dataset_id']}"
            f"_using_{calibration_phase}_calibration"
        )
        output = _verification_group(root, test_phase, calibration_phase) / name
        command = build_verify_command(manifest, calibration, dataset, output, force)
        commands.append(command)
        if dry_run:
            continue
        output.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(command, cwd=repository_root(), check=True)
        if not _completed_run(output):
            raise RuntimeError(f"release verification did not produce a completed run: {output}")
        run.setdefault("results", {})[calibration_phase] = _relative(root, output)
        save_manifest(manifest)
    return commands


def shell_join(command: Iterable[str]) -> str:
    import shlex

    return shlex.join(list(command))
