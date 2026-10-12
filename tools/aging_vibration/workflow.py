"""Discovery, manifest validation, and serial execution for stereo aging studies."""

from __future__ import annotations

import copy
import datetime as dt
import json
import os
import shlex
import subprocess
import tempfile
import time
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
MEMORY_LIMITS = {
    "linux_available_gib": 12.0,
    "windows_free_virtual_gib": 16.0,
}


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def _atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            stream.write(value)
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
    _atomic_text(path, yaml.safe_dump(value, allow_unicode=True, sort_keys=False, width=120))


def _write_json(path: Path, value: Any) -> None:
    _atomic_text(path, json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def machine_path(manifest: dict[str, Any], value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else Path(manifest["_machine_root"]) / path


def repository_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else repository_root() / path


def _relative(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def _discover_phase(root: Path, phase_name: str) -> dict[str, Any] | None:
    phase_root = root / phase_name
    if not phase_root.exists():
        return None
    calibration_sets = sorted(path for path in (phase_root / "calibration").glob("stereo_*") if (path / "dataset.yaml").is_file())
    test_sets = sorted(path for path in (phase_root / "test").glob("stereo_*") if (path / "dataset.yaml").is_file())
    if len(calibration_sets) != 3 or len(test_sets) != 3:
        raise ValueError(
            f"{phase_name} requires exactly three calibration and three test datasets; "
            f"found {len(calibration_sets)} and {len(test_sets)}"
        )
    study = root / "result" / "aging_vibration_study"
    calibrations = []
    for repeat, dataset in enumerate(calibration_sets, 1):
        output_root = study / phase_name / "calibration" / f"repeat_{repeat:02d}_{dataset.name}"
        output_name = dataset.name + "_equi"
        calibrations.append({
            "repeat": repeat,
            "dataset_id": dataset.name,
            "dataset": _relative(root, dataset),
            "output_root": _relative(root, output_root),
            "result": _relative(root, output_root / (output_name + ".yaml")),
            "artifacts": _relative(root, output_root / output_name),
        })
    tests = []
    for repeat, dataset in enumerate(test_sets, 1):
        results = {
            "before": _relative(
                root,
                study / phase_name / "verification" / "using_before"
                / f"repeat_{repeat:02d}_{dataset.name}",
            )
        }
        if phase_name == "after":
            results["after"] = _relative(
                root,
                study / phase_name / "verification" / "using_after"
                / f"repeat_{repeat:02d}_{dataset.name}",
            )
        tests.append({
            "repeat": repeat,
            "dataset_id": dataset.name,
            "dataset": _relative(root, dataset),
            "results": results,
        })
    return {"representative_repeat": 1, "calibration_runs": calibrations, "test_runs": tests}


def create_manifest(machine_root: Path | str) -> Path:
    root = Path(machine_root).resolve()
    if not root.is_dir():
        raise FileNotFoundError(root)
    manifest_path = root / "study_manifest.yaml"
    if manifest_path.exists():
        raise FileExistsError(f"study manifest already exists: {manifest_path}")
    phases = {}
    for phase_name in ("before", "after"):
        phase = _discover_phase(root, phase_name)
        if phase is not None:
            phases[phase_name] = phase
    if "before" not in phases:
        raise ValueError("before/calibration and before/test are required")
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "kind": KIND,
        "machine_id": root.name,
        "machine_root": ".",
        "created_at": _utc_now(),
        "kalibr": {
            "binary": "install/release/bin/kalibr-noros",
            "task_template": "config/mechine/stereo_camera_calibration_task_pinhole_equi.yaml",
            "target": "config/mechine/aprilgrid.yaml",
            "calibration_label": "pinhole_equi",
            "verify_options": copy.deepcopy(VERIFY_OPTIONS),
        },
        "runtime": {"memory_limits": copy.deepcopy(MEMORY_LIMITS)},
        "phases": phases,
        "reports": {
            "before": "result/aging_vibration_study/reports/before_baseline",
            "comparison": "result/aging_vibration_study/reports/before_after_comparison",
        },
    }
    _write_yaml(manifest_path, manifest)
    return manifest_path


def _public_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    return {key: copy.deepcopy(value) for key, value in manifest.items() if not key.startswith("_")}


def refresh_manifest(manifest: dict[str, Any]) -> Path:
    """Register a newly captured after phase without changing existing phase assignments."""
    if "after" in manifest["phases"]:
        raise ValueError("after phase is already registered")
    after = _discover_phase(Path(manifest["_machine_root"]), "after")
    if after is None:
        raise ValueError("after/calibration and after/test are absent")
    manifest["phases"]["after"] = after
    _write_yaml(Path(manifest["_manifest_path"]), _public_manifest(manifest))
    return Path(manifest["_manifest_path"])


def load_manifest(path: Path | str) -> dict[str, Any]:
    manifest_path = Path(path).resolve()
    value = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("study manifest must be a YAML mapping")
    if value.get("schema_version") != SCHEMA_VERSION or value.get("kind") != KIND:
        raise ValueError("unsupported study manifest; regenerate it with the current init command")
    root = Path(value.get("machine_root", "."))
    value["_manifest_path"] = manifest_path
    value["_machine_root"] = root if root.is_absolute() else (manifest_path.parent / root).resolve()
    validate_manifest(value)
    return value


def validate_manifest(manifest: dict[str, Any]) -> None:
    phases = manifest.get("phases")
    if not isinstance(phases, dict) or "before" not in phases or set(phases) - {"before", "after"}:
        raise ValueError("phases must contain before and may contain after")
    for phase_name, phase in phases.items():
        calibrations = phase.get("calibration_runs")
        tests = phase.get("test_runs")
        if not isinstance(calibrations, list) or len(calibrations) != 3:
            raise ValueError(f"{phase_name}.calibration_runs must contain exactly three entries")
        if not isinstance(tests, list) or len(tests) != 3:
            raise ValueError(f"{phase_name}.test_runs must contain exactly three entries")
        if [run.get("repeat") for run in calibrations] != [1, 2, 3]:
            raise ValueError(f"{phase_name} calibration repeats must be 1, 2, 3")
        if [run.get("repeat") for run in tests] != [1, 2, 3]:
            raise ValueError(f"{phase_name} test repeats must be 1, 2, 3")
        if phase.get("representative_repeat") != 1:
            raise ValueError(f"{phase_name}.representative_repeat must be 1")


def _completed_run(directory: Path) -> bool:
    path = directory / "run_manifest.json"
    try:
        return path.is_file() and json.loads(path.read_text(encoding="utf-8")).get("status") == "completed"
    except (OSError, ValueError):
        return False


def inspect_study(manifest: dict[str, Any]) -> dict[str, Any]:
    errors = []
    state = []
    reference_signature = None
    for phase_name, phase in manifest["phases"].items():
        for kind, runs in (("calibration", phase["calibration_runs"]), ("test", phase["test_runs"])):
            for run in runs:
                dataset = machine_path(manifest, run["dataset"])
                if not (dataset / "dataset.yaml").is_file():
                    errors.append(f"missing dataset.yaml: {dataset}")
                else:
                    metadata = yaml.safe_load((dataset / "dataset.yaml").read_text(encoding="utf-8"))
                    metadata = metadata if isinstance(metadata, dict) else {}
                    cameras = metadata.get("cameras", [])
                    signature = tuple((camera.get("id"), tuple(camera.get("resolution", []))) for camera in cameras)
                    if metadata.get("schema_version") != SCHEMA_VERSION:
                        errors.append(f"unsupported dataset schema: {dataset}")
                    if tuple(camera[0] for camera in signature) != ("cam0", "cam1"):
                        errors.append(f"dataset camera IDs must be cam0, cam1: {dataset}")
                    if reference_signature is None:
                        reference_signature = signature
                    elif signature != reference_signature:
                        errors.append(f"camera IDs or resolution differ from other study datasets: {dataset}")
                row = {"phase": phase_name, "kind": kind, "repeat": run["repeat"], "dataset": str(dataset)}
                if kind == "calibration":
                    row["completed"] = machine_path(manifest, run["result"]).is_file() and _completed_run(machine_path(manifest, run["artifacts"]))
                else:
                    row["completed"] = {name: _completed_run(machine_path(manifest, path)) for name, path in run["results"].items()}
                state.append(row)
    return {"status": "ok" if not errors else "error", "errors": errors, "runs": state}


def _release_binary(manifest: dict[str, Any]) -> Path:
    binary = repository_path(manifest["kalibr"]["binary"])
    if not os.access(binary, os.X_OK):
        raise FileNotFoundError(f"release binary is missing or not executable: {binary}")
    return binary


def prepare_runtime_task(manifest: dict[str, Any], phase_name: str, run: dict[str, Any], write: bool = True) -> Path:
    template_path = repository_path(manifest["kalibr"]["task_template"])
    task = yaml.safe_load(template_path.read_text(encoding="utf-8"))
    task["dataset"]["path"] = str(machine_path(manifest, run["dataset"]))
    task["target"]["path"] = str((template_path.parent / task["target"]["path"]).resolve())
    task["initialization"]["path"] = str((template_path.parent / task["initialization"]["path"]).resolve())
    task.setdefault("calibration", {})["solver_mode"] = "full_batch"
    task["execution"] = {
        "detector_processes": 8,
        "optimizer_threads": 4,
        "detector_inflight_per_worker": 2,
        "detector_opencv_threads": 1,
        "profiling_memory_sample_interval_s": 0.25,
    }
    task["output"] = {
        "name": run["dataset_id"] + "_equi",
        "save_diagnostics": True,
        "save_metrics": True,
        "visualizations": {
            "enabled": True,
            "sampling": "uniform",
            "max_frames_per_camera": 30,
            "max_pairs": 30,
            "undistortion": {"enabled": True, "crop": False},
        },
    }
    path = machine_path(
        manifest,
        Path("result/aging_vibration_study/runtime_configs")
        / f"{phase_name}_calibration_repeat_{run['repeat']:02d}_{run['dataset_id']}.yaml",
    )
    if write:
        _write_yaml(path, task)
    return path


def build_calibrate_command(manifest: dict[str, Any], task: Path, run: dict[str, Any], force: bool = False) -> list[str]:
    command = [str(_release_binary(manifest)), "calibrate", "cameras", "--config", str(task), "--output-dir", str(machine_path(manifest, run["output_root"]))]
    if force:
        command.append("--force")
    return command


def representative_calibration(manifest: dict[str, Any], phase_name: str) -> dict[str, Any]:
    phase = manifest["phases"][phase_name]
    return next(run for run in phase["calibration_runs"] if run["repeat"] == phase["representative_repeat"])


def build_verify_command(manifest: dict[str, Any], calibration: Path, dataset: Path, output: Path, force: bool = False) -> list[str]:
    options = manifest["kalibr"]["verify_options"]
    command = [
        str(_release_binary(manifest)), "verify", "cameras",
        "--calibration", f"{manifest['kalibr']['calibration_label']}={calibration}",
        "--dataset", str(dataset), "--target", str(repository_path(manifest["kalibr"]["target"])),
        "--output-dir", str(output),
        "--window-half-size-px", str(options["window_half_size_px"]),
        "--max-displacement-px", str(options["max_displacement_px"]),
        "--synchronization-tolerance-s", str(options["synchronization_tolerance_s"]),
        "--detector-opencv-threads", str(options["detector_opencv_threads"]),
        "--max-frames-per-camera", str(options["max_frames_per_camera"]),
        "--max-pairs", str(options["max_pairs"]),
    ]
    if options["visualizations"]:
        command.append("--visualizations")
    if force:
        command.append("--force")
    return command


def _linux_available_gib() -> float:
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        if line.startswith("MemAvailable:"):
            return float(line.split()[1]) / 1024.0 / 1024.0
    raise RuntimeError("MemAvailable is absent from /proc/meminfo")


def _windows_free_virtual_gib() -> float | None:
    powershell = Path("/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe")
    if not powershell.is_file():
        return None
    process = subprocess.run(
        [str(powershell), "-NoProfile", "-NonInteractive", "-Command", "(Get-CimInstance Win32_OperatingSystem).FreeVirtualMemory"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False,
    )
    if process.returncode:
        return None
    decoded = process.stdout.replace(b"\x00", b"").decode("ascii", errors="ignore").strip()
    try:
        return float(decoded.splitlines()[-1]) / 1024.0 / 1024.0
    except (ValueError, IndexError):
        return None


def memory_status(manifest: dict[str, Any]) -> dict[str, Any]:
    limits = manifest["runtime"]["memory_limits"]
    linux = _linux_available_gib()
    windows = _windows_free_virtual_gib()
    errors = []
    if linux < float(limits["linux_available_gib"]):
        errors.append(f"Linux available memory {linux:.2f} GiB is below {limits['linux_available_gib']} GiB")
    if windows is not None and windows < float(limits["windows_free_virtual_gib"]):
        errors.append(f"Windows free virtual memory {windows:.2f} GiB is below {limits['windows_free_virtual_gib']} GiB")
    return {"linux_available_gib": linux, "windows_free_virtual_gib": windows, "errors": errors}


def _run_logged(manifest: dict[str, Any], command: list[str], log_path: Path, dry_run: bool) -> None:
    status = memory_status(manifest)
    prefix = [
        f"started_at: {_utc_now()}",
        f"command: {shlex.join(command)}",
        f"linux_available_gib: {status['linux_available_gib']:.3f}",
        f"windows_free_virtual_gib: {status['windows_free_virtual_gib']}",
    ]
    print("\n".join(prefix), flush=True)
    if status["errors"]:
        raise RuntimeError("; ".join(status["errors"]) + "; close memory-heavy applications such as Foxglove and retry")
    if dry_run:
        return
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with log_path.open("a", encoding="utf-8", buffering=1) as log:
        log.write("\n" + "\n".join(prefix) + "\n")
        process = subprocess.Popen(command, cwd=repository_root(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
        returncode = process.wait()
        finished = f"finished_at: {_utc_now()}\nwall_seconds: {time.monotonic() - started:.3f}\nreturncode: {returncode}\n"
        print(finished, end="", flush=True)
        log.write(finished)
    if returncode:
        raise subprocess.CalledProcessError(returncode, command)


def run_calibrations(manifest: dict[str, Any], phase_name: str, dry_run: bool = False, force: bool = False) -> list[list[str]]:
    commands = []
    logs = machine_path(manifest, "result/aging_vibration_study/logs")
    for run in manifest["phases"][phase_name]["calibration_runs"]:
        result = machine_path(manifest, run["result"])
        artifacts = machine_path(manifest, run["artifacts"])
        if result.is_file() and _completed_run(artifacts) and not force:
            print(f"reuse completed calibration: {result}")
            continue
        task = prepare_runtime_task(manifest, phase_name, run, write=not dry_run)
        command = build_calibrate_command(manifest, task, run, force)
        commands.append(command)
        if not dry_run:
            machine_path(manifest, run["output_root"]).mkdir(parents=True, exist_ok=True)
        _run_logged(manifest, command, logs / f"{phase_name}_calibration_repeat_{run['repeat']:02d}.log", dry_run)
        if not dry_run and (not result.is_file() or not _completed_run(artifacts)):
            raise RuntimeError(f"calibration did not produce a complete managed result: {artifacts}")
    return commands


def run_verifications(manifest: dict[str, Any], test_phase: str, calibration_phase: str, dry_run: bool = False, force: bool = False) -> list[list[str]]:
    calibration_run = representative_calibration(manifest, calibration_phase)
    calibration = machine_path(manifest, calibration_run["result"])
    if not calibration.is_file() and not dry_run:
        raise FileNotFoundError(calibration)
    commands = []
    logs = machine_path(manifest, "result/aging_vibration_study/logs")
    for run in manifest["phases"][test_phase]["test_runs"]:
        output = machine_path(manifest, run["results"][calibration_phase])
        if _completed_run(output) and not force:
            print(f"reuse completed verification: {output}")
            continue
        command = build_verify_command(manifest, calibration, machine_path(manifest, run["dataset"]), output, force)
        commands.append(command)
        _run_logged(manifest, command, logs / f"{test_phase}_verification_repeat_{run['repeat']:02d}_using_{calibration_phase}.log", dry_run)
        if not dry_run and not _completed_run(output):
            raise RuntimeError(f"verification did not produce a complete managed result: {output}")
    return commands


def run_phase(manifest: dict[str, Any], phase_name: str, dry_run: bool = False, force: bool = False) -> dict[str, Any]:
    if phase_name not in manifest["phases"]:
        raise ValueError(f"phase is not present: {phase_name}")
    inspection = inspect_study(manifest)
    if inspection["errors"]:
        raise ValueError("study input validation failed: " + "; ".join(inspection["errors"]))
    commands = []
    if phase_name == "after":
        commands.extend(run_verifications(manifest, "after", "before", dry_run, force))
    commands.extend(run_calibrations(manifest, phase_name, dry_run, force))
    commands.extend(run_verifications(manifest, phase_name, phase_name, dry_run, force))
    if not dry_run:
        from .reporting import generate_report
        report = generate_report(manifest, full=phase_name == "after", force=force)
    else:
        report = None
    return {"commands": commands, "report": report}


def shell_join(command: Iterable[str]) -> str:
    return shlex.join(list(command))
