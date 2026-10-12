"""Safe, serial orchestration for rolling-shutter data diagnosis."""

from __future__ import annotations

import copy
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
from typing import Any

import numpy as np
import yaml

from . import KIND, SCHEMA_VERSION
from .analysis import (
    compare_archived_residuals,
    load_run_summary,
    read_archive,
    screen_observations,
)


SUPPORTED_SOURCE_JOBS = {"camera_calibration", "camera_imu_calibration"}
CAMERA_MODELS = {"pinhole-equi", "pinhole-radtan"}
MANAGED_DIRECTORIES = ("configs", "runs", "logs", "charts")
MANAGED_FILES = (
    "diagnosis_manifest.json",
    "screening.json",
    "screening_frames_rejected.csv",
    "formal_comparison.json",
    "result.json",
    "summary.csv",
    "run_metrics.csv",
    "README_ZH.md",
    "report.html",
    "report.pdf",
    "suggested_rolling_shutter_task.yaml",
)


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
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


def write_json(path: Path, value: Any) -> None:
    _atomic_text(path, json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def write_yaml(path: Path, value: Any) -> None:
    _atomic_text(path, yaml.safe_dump(value, allow_unicode=True, sort_keys=False, width=120))


def _load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"YAML document must be a mapping: {path}")
    return value


def _resolve(base: Path, value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def _resolve_task_paths(task: dict[str, Any], task_path: Path) -> dict[str, Any]:
    task = copy.deepcopy(task)
    base = task_path.parent
    for section in ("dataset", "target", "initialization", "camera_calibration"):
        block = task.get(section)
        if isinstance(block, dict) and block.get("path"):
            block["path"] = str(_resolve(base, block["path"]))
    for imu in task.get("imus", []):
        if imu.get("path"):
            imu["path"] = str(_resolve(base, imu["path"]))
    return task


def _camera_model_from_result(camera: dict[str, Any]) -> str:
    if camera.get("camera_model") != "pinhole":
        raise ValueError("only pinhole-equi and pinhole-radtan are supported")
    distortion = camera.get("distortion_model")
    if distortion == "equidistant":
        return "pinhole-equi"
    if distortion == "radtan":
        return "pinhole-radtan"
    raise ValueError("only pinhole-equi and pinhole-radtan are supported")


def load_config(path: Path | str) -> dict[str, Any]:
    config_path = Path(path).expanduser().resolve()
    config = _load_yaml(config_path)
    allowed = {
        "schema_version", "kind", "source_task", "capture",
        "rolling_shutter", "screening", "formal_controls",
    }
    unknown = set(config) - allowed
    if unknown:
        raise ValueError("unknown diagnosis fields: " + ", ".join(sorted(unknown)))
    if config.get("schema_version") != SCHEMA_VERSION or config.get("kind") != KIND:
        raise ValueError('diagnosis requires schema_version: "1.0.0" and kind: rolling_shutter_diagnosis')
    source = config.get("source_task")
    if not isinstance(source, dict) or set(source) != {"path"} or not isinstance(source.get("path"), str):
        raise ValueError("source_task must contain exactly one string field: path")
    task_path = _resolve(config_path.parent, source["path"])
    if not task_path.is_file():
        raise FileNotFoundError(task_path)
    task = _resolve_task_paths(_load_yaml(task_path), task_path)
    if task.get("schema_version") != SCHEMA_VERSION or task.get("kind") != "calibration_task":
        raise ValueError("source_task.path must reference a v1.0.0 calibration task")
    if task.get("job") not in SUPPORTED_SOURCE_JOBS:
        raise ValueError("source task job must be camera_calibration or camera_imu_calibration")

    capture = config.get("capture")
    if not isinstance(capture, dict):
        raise ValueError("capture must be a mapping")
    capture_unknown = set(capture) - {"motion", "camera_timestamp_event"}
    if capture_unknown:
        raise ValueError("unknown capture fields: " + ", ".join(sorted(capture_unknown)))
    if capture.get("motion") not in {"continuous", "stop_and_go", "unknown"}:
        raise ValueError("capture.motion must be continuous, stop_and_go, or unknown")
    if "camera_timestamp_event" in capture and not isinstance(capture["camera_timestamp_event"], str):
        raise ValueError("capture.camera_timestamp_event must be a string")

    if task["job"] == "camera_calibration":
        cameras = task.get("cameras")
        if not isinstance(cameras, list) or not cameras:
            raise ValueError("camera source task must contain cameras")
        ids = [str(camera.get("id")) for camera in cameras]
        if any(camera.get("model") not in CAMERA_MODELS for camera in cameras):
            raise ValueError("diagnosis supports pinhole-equi and pinhole-radtan only")
        camera_result = None
    else:
        dependency = task.get("camera_calibration")
        if not isinstance(dependency, dict) or not dependency.get("path"):
            raise ValueError(
                "Camera-IMU diagnosis requires explicit camera_calibration.path because its isolated output directory cannot use automatic lookup"
            )
        camera_result = Path(dependency["path"])
        if not camera_result.is_file():
            raise FileNotFoundError(camera_result)
        result = _load_yaml(camera_result)
        result_cameras = result.get("cameras") or []
        ids = [str(camera.get("id")) for camera in result_cameras]
        for camera in result_cameras:
            _camera_model_from_result(camera)
        if not ids:
            raise ValueError("camera_calibration.path contains no cameras")

    shutters = config.get("rolling_shutter")
    if not isinstance(shutters, dict) or set(shutters) != set(ids):
        raise ValueError("rolling_shutter keys must exactly match source camera IDs")
    bounds: dict[str, float] = {}
    for camera_id in ids:
        entry = shutters[camera_id]
        if not isinstance(entry, dict) or set(entry) != {"max_abs_line_delay_s"}:
            raise ValueError(f"rolling_shutter.{camera_id} must contain exactly max_abs_line_delay_s")
        bound = entry["max_abs_line_delay_s"]
        if isinstance(bound, bool) or not isinstance(bound, (int, float)) or not math.isfinite(bound) or bound <= 0:
            raise ValueError(f"rolling_shutter.{camera_id}.max_abs_line_delay_s must be finite and > 0")
        bounds[camera_id] = float(bound)

    screening = config.get("screening") or {}
    if not isinstance(screening, dict) or set(screening) - {"bootstrap_samples", "bootstrap_seed"}:
        raise ValueError("screening supports bootstrap_samples and bootstrap_seed only")
    samples = screening.get("bootstrap_samples", 1000)
    seed = screening.get("bootstrap_seed", 0)
    if isinstance(samples, bool) or not isinstance(samples, int) or samples < 100:
        raise ValueError("screening.bootstrap_samples must be an integer >= 100")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("screening.bootstrap_seed must be a nonnegative integer")

    formal = config.get("formal_controls") or {}
    formal_unknown = set(formal) - {"freeze_intrinsics", "max_iterations"} if isinstance(formal, dict) else set()
    if not isinstance(formal, dict) or formal_unknown:
        raise ValueError(
            "formal_controls supports freeze_intrinsics and max_iterations only"
        )
    freeze_intrinsics = formal.get("freeze_intrinsics", True)
    max_iterations = formal.get("max_iterations")
    if type(freeze_intrinsics) is not bool:
        raise ValueError("formal_controls.freeze_intrinsics must be a boolean")
    if max_iterations is not None and (
        type(max_iterations) is not int or max_iterations < 1
    ):
        raise ValueError("formal_controls.max_iterations must be a positive integer")
    if task["job"] == "camera_imu_calibration" and "freeze_intrinsics" in formal:
        raise ValueError(
            "formal_controls.freeze_intrinsics only applies to camera_calibration sources"
        )

    return {
        "config_path": config_path,
        "raw": config,
        "source_task_path": task_path,
        "source_task": task,
        "source_job": task["job"],
        "camera_ids": ids,
        "camera_result": camera_result,
        "bounds": bounds,
        "capture": capture,
        "bootstrap_samples": samples,
        "bootstrap_seed": seed,
        "formal_controls": {
            "freeze_intrinsics": freeze_intrinsics,
            "max_iterations": max_iterations,
        },
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _config_digest(config: dict[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(config["raw"], sort_keys=True, ensure_ascii=False).encode("utf-8")
    ).hexdigest()


def _dependency_hashes(config: dict[str, Any]) -> dict[str, str]:
    task = config["source_task"]
    paths = {config["config_path"], config["source_task_path"]}
    for section in ("target", "initialization", "camera_calibration"):
        block = task.get(section)
        if isinstance(block, dict) and block.get("path"):
            paths.add(Path(block["path"]))
    for imu in task.get("imus", []):
        if imu.get("path"):
            paths.add(Path(imu["path"]))
    dataset = Path(task["dataset"]["path"])
    if dataset.is_dir():
        for name in ("dataset.yaml", "timestamps.csv", "imu.csv"):
            paths.update(dataset.rglob(name))
    elif dataset.is_file():
        paths.add(dataset)
    return {str(path): _sha256(path) for path in sorted(paths) if path.is_file()}


def _selected_image_hashes(observations: Path) -> dict[str, str]:
    result = {}
    for row in read_archive(observations / "frames.csv.gz"):
        if not row.get("source_path"):
            continue
        path = Path(row["source_path"])
        if path.is_file():
            result[str(path)] = _sha256(path)
    return dict(sorted(result.items()))


def _clear_managed_output(output: Path) -> None:
    manifest = output / "diagnosis_manifest.json"
    if not manifest.is_file():
        raise ValueError("--force refuses to clear a directory without this tool's diagnosis_manifest.json")
    value = json.loads(manifest.read_text(encoding="utf-8"))
    if value.get("kind") != KIND:
        raise ValueError("--force refuses to clear output owned by another workflow")
    for name in MANAGED_DIRECTORIES:
        path = output / name
        if path.exists():
            if path.is_symlink():
                raise ValueError(f"--force refuses to remove managed-directory symlink: {path}")
            shutil.rmtree(path)
    for name in MANAGED_FILES:
        path = output / name
        if path.exists():
            if path.is_symlink():
                raise ValueError(f"--force refuses to remove managed-file symlink: {path}")
            path.unlink()


def prepare_output(config: dict[str, Any], output: Path, force: bool) -> dict[str, Any]:
    output = output.expanduser().resolve()
    manifest_path = output / "diagnosis_manifest.json"
    digest = _config_digest(config)
    if output.exists() and any(output.iterdir()):
        if not manifest_path.is_file():
            raise ValueError("output directory is nonempty and is not owned by this tool")
        previous = json.loads(manifest_path.read_text(encoding="utf-8"))
        if previous.get("kind") != KIND:
            raise ValueError("output directory is owned by another workflow")
        if previous.get("config_sha256") != digest and not force:
            raise ValueError("diagnosis config changed; use a new output directory or --force")
        if force:
            _clear_managed_output(output)
    output.mkdir(parents=True, exist_ok=True)
    for name in ("configs", "runs", "logs", "charts"):
        (output / name).mkdir(exist_ok=True)
    if manifest_path.is_file():
        return json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "kind": KIND,
        "status": "prepared",
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "config_path": str(config["config_path"]),
        "config_sha256": digest,
        "source_task_path": str(config["source_task_path"]),
        "dependency_sha256": _dependency_hashes(config),
        "selected_image_sha256": {},
        "runs": {},
    }
    write_json(manifest_path, manifest)
    return manifest


def _ordinary_task(config: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    task = copy.deepcopy(config["source_task"])
    name = "ordinary_source"
    output = task.setdefault("output", {})
    output.update({
        "name": name,
        "save_diagnostics": True,
        "save_metrics": True,
        "archive_observations": True,
        "archive_selection_history": True,
        "copy_used_images": False,
        "export_opencv": False,
        "export_text": False,
        "export_poses": False,
        "verbose": False,
        "show_extraction": False,
        "extraction_stepping": False,
        "interactive_report": False,
        "visualizations": {"enabled": False},
    })
    return name, task


def _camera_initialization(result_path: Path, path: Path) -> None:
    result = _load_yaml(result_path)
    cameras = {}
    for camera in result.get("cameras", []):
        value = {
            "intrinsics": list(camera["intrinsics"]),
            "distortion_coeffs": list(camera["distortion_coeffs"]),
        }
        if camera.get("T_cn_cnm1") is not None:
            value["T_cn_cnm1"] = camera["T_cn_cnm1"]
        cameras[str(camera["id"])] = value
    write_yaml(path, {
        "schema_version": SCHEMA_VERSION,
        "kind": "camera_calibration_initialization",
        "cameras": cameras,
    })


def _rs_output(name: str) -> dict[str, Any]:
    return {
        "name": name,
        "save_diagnostics": True,
        "save_metrics": True,
        "export_text": False,
        "archive_observations": True,
        "archive_selection_history": True,
        "copy_used_images": False,
        "export_opencv": False,
        "export_poses": True,
        "verbose": False,
        "show_extraction": False,
        "extraction_stepping": False,
        "interactive_report": False,
        "evaluation_pairing_tolerance_s": 0.0002,
        "visualizations": {"enabled": False, "max_frames_per_camera": 5, "max_pairs": 5, "sampling": "uniform"},
        "rectification": {"balance": 0.0, "fov_scale": 1.0, "size": None},
        "assessment": {"reference_grading": False, "rules": []},
    }


def _rs_camera_calibration(
    source: dict[str, Any], camera_count: int, knots_per_second: float,
    time_padding_s: float, formal_controls: dict[str, Any],
) -> dict[str, Any]:
    original = source.get("calibration") or {}
    requested_iterations = formal_controls.get("max_iterations")
    return {
        "freeze_intrinsics": (
            camera_count >= 2 and formal_controls["freeze_intrinsics"]
        ),
        "window_half_size_px": original.get("window_half_size_px", 2),
        "max_displacement_px": original.get("max_displacement_px", math.sqrt(1.5)),
        "focal_initialization_min_visible_corner_ratio": original.get("focal_initialization_min_visible_corner_ratio", 1.0),
        "synchronization_tolerance_s": original.get("synchronization_tolerance_s", 0.02),
        "min_views_for_outlier_statistics": original.get("min_views_for_outlier_statistics", 20),
        "remove_outliers": False,
        "final_filtering": False,
        "blake_zisserman": False,
        "max_iterations": (
            int(requested_iterations) if requested_iterations is not None
            else max(int(original.get("max_iterations", 80)), 5000)
        ),
        "spline_order": 4,
        "time_padding_s": time_padding_s,
        "knots_per_second": float(knots_per_second),
        "feature_sigma_px": 1.0,
        "motion_translation_weight": 1e-5,
        "motion_rotation_weight": 1e-2,
    }


def _rs_imu_calibration(
    source: dict[str, Any], recover_covariance: bool,
    formal_controls: dict[str, Any],
) -> dict[str, Any]:
    original = source.get("calibration") or {}
    result = copy.deepcopy(original)
    requested_iterations = formal_controls.get("max_iterations")
    result.update({
        "max_iterations": (
            int(requested_iterations) if requested_iterations is not None
            else max(int(original.get("max_iterations", 30)), 100)
        ),
        "recover_covariance": recover_covariance,
    })
    return result


def _execution(source: dict[str, Any]) -> dict[str, Any]:
    original = source.get("execution") or {}
    return {
        "detector_processes": int(original.get("detector_processes", 4)),
        "optimizer_threads": int(original.get("optimizer_threads", 4)),
        "detector_inflight_per_worker": int(original.get("detector_inflight_per_worker", 2)),
        "detector_opencv_threads": int(original.get("detector_opencv_threads", 1)),
        "profiling_memory_sample_interval_s": float(original.get("profiling_memory_sample_interval_s", 0.25)),
    }


def _shutters(
    config: dict[str, Any], candidates: dict[str, float], seed_scale: float, estimate: bool
) -> dict[str, Any]:
    return {
        camera_id: {
            "line_delay_s": float(candidates[camera_id] * seed_scale),
            "estimate": estimate,
            "max_abs_line_delay_s": config["bounds"][camera_id],
        }
        for camera_id in config["camera_ids"]
    }


def build_formal_tasks(
    config: dict[str, Any], output: Path, ordinary_result: Path, screen: dict[str, Any]
) -> list[dict[str, Any]]:
    source = config["source_task"]
    candidates = {
        camera_id: float(screen["camera_results"][camera_id]["candidate_line_delay_s"])
        for camera_id in config["camera_ids"]
    }
    variants: list[tuple[str, float, bool, float | None, bool]] = []
    if config["source_job"] == "camera_calibration":
        frequencies = [
            screen["camera_results"][camera_id]["median_frame_frequency_hz"]
            for camera_id in config["camera_ids"]
        ]
        base_knots = float(np.median([value for value in frequencies if value and math.isfinite(value)]))
        ordinary_cameras = _load_yaml(ordinary_result).get("cameras", [])
        heights = {str(camera["id"]): int(camera["resolution"][1]) for camera in ordinary_cameras}
        required_padding = max(
            (heights[camera_id] - 1) * config["bounds"][camera_id]
            for camera_id in config["camera_ids"]
        )
        time_padding_s = max(0.5, required_padding * 1.1)
        variants = [
            ("free_seed0", 0.0, True, base_knots, False),
            ("fixed_zero", 0.0, False, base_knots, False),
            ("fixed_candidate", 1.0, False, base_knots, False),
            ("fixed_opposite", -1.0, False, base_knots, False),
            ("free_seed_candidate", 1.0, True, base_knots, False),
            ("free_seed_opposite", -1.0, True, base_knots, False),
            ("knot_half_free", 0.0, True, base_knots * 0.5, False),
            ("knot_half_zero", 0.0, False, base_knots * 0.5, False),
            ("knot_double_free", 0.0, True, base_knots * 2.0, False),
            ("knot_double_zero", 0.0, False, base_knots * 2.0, False),
        ]
        initialization = output / "configs" / "camera_initialization_from_ordinary.yaml"
        _camera_initialization(ordinary_result, initialization)
    else:
        variants = [
            ("free_seed0", 0.0, True, None, True),
            ("fixed_zero", 0.0, False, None, False),
            ("fixed_candidate", 1.0, False, None, False),
            ("fixed_opposite", -1.0, False, None, False),
            ("free_seed_candidate", 1.0, True, None, False),
            ("free_seed_opposite", -1.0, True, None, False),
        ]

    tasks = []
    for name, seed_scale, estimate, knots, covariance in variants:
        if config["source_job"] == "camera_calibration":
            task = {
                "schema_version": SCHEMA_VERSION,
                "kind": "calibration_task",
                "job": "camera_rolling_shutter_calibration",
                "dataset": copy.deepcopy(source["dataset"]),
                "target": copy.deepcopy(source["target"]),
                "cameras": copy.deepcopy(source["cameras"]),
                "initialization": {"path": str(initialization)},
                "rolling_shutter": _shutters(config, candidates, seed_scale, estimate),
                "calibration": _rs_camera_calibration(
                    source, len(config["camera_ids"]), float(knots),
                    time_padding_s, config["formal_controls"],
                ),
                "execution": _execution(source),
                "output": _rs_output(name),
            }
            command = "cameras-rs"
        else:
            task = copy.deepcopy(source)
            task["job"] = "camera_imu_rolling_shutter_calibration"
            task["rolling_shutter"] = _shutters(config, candidates, seed_scale, estimate)
            task["calibration"] = _rs_imu_calibration(
                source, covariance, config["formal_controls"]
            )
            task["execution"] = _execution(source)
            task["output"] = _rs_output(name)
            command = "imu-camera-rs"
        path = output / "configs" / f"{name}.yaml"
        write_yaml(path, task)
        tasks.append({"name": name, "path": path, "command": command})
    return tasks


def _completed(run_dir: Path, output_name: str) -> bool:
    manifest = run_dir / output_name / "run_manifest.json"
    result = run_dir / f"{output_name}.yaml"
    try:
        return result.is_file() and json.loads(manifest.read_text(encoding="utf-8")).get("status") == "completed"
    except (OSError, ValueError):
        return False


def _run_command(command: list[str], log_path: Path, dry_run: bool) -> int:
    print("$ " + shlex.join(command), flush=True)
    if dry_run:
        return 0
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8", newline="") as log:
        process = subprocess.Popen(
            command,
            cwd=repository_root(),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
        return process.wait()


def _execute_task(
    binary: Path,
    task_path: Path,
    run_dir: Path,
    output_name: str,
    command: str,
    output: Path,
    *,
    dry_run: bool,
    force: bool,
) -> tuple[list[list[str]], bool]:
    run_dir.mkdir(parents=True, exist_ok=True)
    commands = [
        [str(binary), "validate", "--config", str(task_path)],
        [str(binary), "calibrate", command, "--config", str(task_path), "--output-dir", str(run_dir)],
    ]
    if _completed(run_dir, output_name) and not force:
        print(f"resume: {output_name} already completed", flush=True)
        return commands, True
    if force or (run_dir / output_name).exists() or (run_dir / f"{output_name}.yaml").exists():
        commands[1].append("--force")
    for index, values in enumerate(commands):
        code = _run_command(values, output / "logs" / f"{output_name}_{index}.log", dry_run)
        if code:
            return commands, False
    return commands, dry_run or _completed(run_dir, output_name)


def _formal_evidence(output: Path, config: dict[str, Any], tasks: list[dict[str, Any]]) -> dict[str, Any]:
    summaries = {
        task["name"]: load_run_summary(output / "runs" / task["name"], task["name"])
        for task in tasks
    }
    comparisons = {}
    pairs = [("free_seed0", "fixed_zero"), ("fixed_candidate", "fixed_opposite")]
    if config["source_job"] == "camera_calibration":
        pairs += [("knot_half_free", "knot_half_zero"), ("knot_double_free", "knot_double_zero")]
    for candidate, baseline in pairs:
        candidate_dir = output / "runs" / candidate / candidate / "observations"
        baseline_dir = output / "runs" / baseline / baseline / "observations"
        if candidate_dir.is_dir() and baseline_dir.is_dir():
            comparisons[f"{candidate}_vs_{baseline}"] = compare_archived_residuals(
                baseline_dir,
                candidate_dir,
                bootstrap_samples=config["bootstrap_samples"],
                bootstrap_seed=config["bootstrap_seed"],
            )

    camera_gates = {}
    required = [task["name"] for task in tasks]
    for camera_index, camera_id in enumerate(config["camera_ids"]):
        base = summaries["free_seed0"].get("cameras", {}).get(camera_id, {})
        tau = base.get("line_delay_s")
        bound = config["bounds"][camera_id]
        multistart = [
            summaries[name].get("cameras", {}).get(camera_id, {}).get("line_delay_s")
            for name in ("free_seed0", "free_seed_candidate", "free_seed_opposite")
        ]
        common = comparisons.get("free_seed0_vs_fixed_zero", {}).get(camera_id, {})
        fixed_direction = comparisons.get("fixed_candidate_vs_fixed_opposite", {}).get(camera_id, {})
        observability = summaries["free_seed0"].get("observability", {})
        pair = summaries["free_seed0"].get("cameras", {}).get(camera_id, {})
        optimizer_healthy = all(
            isinstance(summaries[name].get("optimizer"), dict)
            and summaries[name]["optimizer"].get("linearSolverFailure") is False
            and isinstance(summaries[name]["optimizer"].get("iterations"), int)
            for name in required
        )
        gates: dict[str, bool] = {
            "all_required_runs_completed": all(summaries[name].get("completed") for name in required),
            "all_optimizer_summaries_healthy": optimizer_healthy,
            "finite_line_delay_below_95_percent_bound": tau is not None and math.isfinite(tau) and abs(tau) < 0.95 * bound,
            "hard_rank_full": observability.get("status") == "full_rank",
            "line_delay_not_dominant_in_operational_weak_mode": observability.get(
                "maximum_line_delay_contribution_in_weak_mode", 1.0
            ) <= 0.5,
            "free_improves_common_corner_residual_with_positive_bootstrap_lower_bound": bool(common) and common["bootstrap_95_percent"]["lower_px"] > 0.0,
            "fixed_candidate_better_than_fixed_opposite": bool(fixed_direction)
            and fixed_direction["relative_reduction"] > 0.0,
            "multistart_spread_within_10_percent_bound": (
                all(value is not None and math.isfinite(value) for value in multistart)
                and max(multistart) - min(multistart) <= 0.10 * bound
            ),
            "stereo_pair_metrics_finite": camera_index == 0 or all(
                value is not None and math.isfinite(value)
                for value in (
                    pair.get("alignment_rms_px"),
                    pair.get("rs_compensated_pair_residual_rms_px"),
                )
            ),
        }
        if config["source_job"] == "camera_calibration":
            knot_values = [
                summaries[name].get("cameras", {}).get(camera_id, {}).get("line_delay_s")
                for name in ("knot_half_free", "free_seed0", "knot_double_free")
            ]
            knot_improvements = [
                comparisons.get(name, {}).get(camera_id, {}).get("bootstrap_95_percent", {}).get("lower_px")
                for name in (
                    "knot_half_free_vs_knot_half_zero",
                    "free_seed0_vs_fixed_zero",
                    "knot_double_free_vs_knot_double_zero",
                )
            ]
            gates["knot_spread_within_20_percent_bound"] = (
                all(value is not None and math.isfinite(value) for value in knot_values)
                and max(knot_values) - min(knot_values) <= 0.20 * bound
            )
            gates["all_knot_controls_improve"] = all(value is not None and value > 0.0 for value in knot_improvements)
        else:
            std = base.get("line_delay_std_s")
            gates["line_delay_exceeds_1_96_local_std"] = (
                tau is not None
                and math.isfinite(tau)
                and std is not None
                and math.isfinite(std)
                and std > 0
                and abs(tau) > 1.96 * std
            )
            fixed_imu = summaries["fixed_zero"].get("imu_rms", {})
            free_imu = summaries["free_seed0"].get("imu_rms", {})
            comparisons_imu = []
            for imu_id in set(fixed_imu) & set(free_imu):
                values = (
                    free_imu[imu_id].get("accel"),
                    fixed_imu[imu_id].get("accel"),
                    free_imu[imu_id].get("gyro"),
                    fixed_imu[imu_id].get("gyro"),
                )
                if all(value is not None and math.isfinite(value) for value in values):
                    accel_worse = values[0] > values[1]
                    gyro_worse = values[2] > values[3]
                    comparisons_imu.append(not (accel_worse and gyro_worse))
            gates["gyro_and_accel_not_both_worse"] = bool(comparisons_imu) and all(comparisons_imu)
        camera_gates[camera_id] = {"passed": all(gates.values()), "gates": gates}

    classification = "important" if camera_gates and all(value["passed"] for value in camera_gates.values()) else "inconclusive"
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": "rolling_shutter_formal_comparison",
        "classification": classification,
        "completed_run_count": len(summaries),
        "required_run_count": len(tasks),
        "missing_runs": [],
        "runs": summaries,
        "common_corner_comparisons": comparisons,
        "camera_gates": camera_gates,
        "interpretation": "automatic evidence classification; not a hardware timing measurement or production approval",
    }


def _incomplete_formal_evidence(
    output: Path, tasks: list[dict[str, Any]]
) -> dict[str, Any]:
    """Keep readable metrics from completed controls without grading them."""
    summaries = {
        task["name"]: load_run_summary(
            output / "runs" / task["name"], task["name"]
        )
        for task in tasks
        if _completed(output / "runs" / task["name"], task["name"])
    }
    missing = [task["name"] for task in tasks if task["name"] not in summaries]
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": "rolling_shutter_formal_comparison",
        "classification": "inconclusive",
        "reason": "one or more mandatory formal controls failed",
        "completed_run_count": len(summaries),
        "required_run_count": len(tasks),
        "missing_runs": missing,
        "runs": summaries,
        "common_corner_comparisons": {},
        "camera_gates": {},
        "interpretation": (
            "completed-run metrics are retained for diagnosis only; no formal "
            "classification gates are evaluated until every mandatory control completes"
        ),
    }


def _suggested_task(
    config: dict[str, Any], formal: dict[str, Any], tasks: list[dict[str, Any]], output: Path
) -> Path:
    source = next(task for task in tasks if task["name"] == "free_seed0")
    suggested = _load_yaml(source["path"])
    runs = formal["runs"]
    for camera_id in config["camera_ids"]:
        estimates = [
            runs[name]["cameras"][camera_id]["line_delay_s"]
            for name in ("free_seed0", "free_seed_candidate", "free_seed_opposite")
        ]
        suggested["rolling_shutter"][camera_id]["line_delay_s"] = float(np.median(estimates))
        suggested["rolling_shutter"][camera_id]["estimate"] = True
    suggested["output"]["name"] = None
    path = output / "suggested_rolling_shutter_task.yaml"
    write_yaml(path, suggested)
    return path


def run_diagnosis(
    config_path: Path | str,
    output_dir: Path | str,
    *,
    binary: Path | str | None = None,
    dry_run: bool = False,
    force: bool = False,
) -> dict[str, Any]:
    config = load_config(config_path)
    output = Path(output_dir).expanduser().resolve()
    manifest = prepare_output(config, output, force)
    executable = Path(binary).expanduser().resolve() if binary else repository_root() / "install/release/bin/kalibr-noros"
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise FileNotFoundError(f"kalibr-noros is not executable: {executable}")

    ordinary_name, ordinary = _ordinary_task(config)
    ordinary_path = output / "configs" / "ordinary_source.yaml"
    write_yaml(ordinary_path, ordinary)
    ordinary_command = "cameras" if config["source_job"] == "camera_calibration" else "imu-camera"
    commands, completed = _execute_task(
        executable,
        ordinary_path,
        output / "runs" / ordinary_name,
        ordinary_name,
        ordinary_command,
        output,
        dry_run=dry_run,
        force=force,
    )
    manifest["runs"][ordinary_name] = {"commands": commands, "completed": completed}
    write_json(output / "diagnosis_manifest.json", manifest)
    if dry_run:
        manifest["status"] = "dry_run"
        write_json(output / "diagnosis_manifest.json", manifest)
        return {"status": "dry_run", "commands": commands, "output_dir": str(output)}
    if not completed:
        manifest["status"] = "failed"
        write_json(output / "diagnosis_manifest.json", manifest)
        raise RuntimeError(f"ordinary source calibration failed; see {output / 'logs'}")

    ordinary_result = output / "runs" / ordinary_name / f"{ordinary_name}.yaml"
    ordinary_observations = output / "runs" / ordinary_name / ordinary_name / "observations"
    screen, rejected = screen_observations(
        ordinary_result,
        ordinary_observations,
        config["bounds"],
        config["capture"]["motion"],
        bootstrap_samples=config["bootstrap_samples"],
        bootstrap_seed=config["bootstrap_seed"],
    )
    write_json(output / "screening.json", screen)
    _write_rejected(output / "screening_frames_rejected.csv", rejected)
    before_images = _selected_image_hashes(ordinary_observations)
    manifest["selected_image_sha256"] = before_images

    formal: dict[str, Any] | None = None
    all_commands = list(commands)
    tasks: list[dict[str, Any]] = []
    if screen["run_formal_controls"]:
        tasks = build_formal_tasks(config, output, ordinary_result, screen)
        all_completed = True
        for task in tasks:
            task_commands, task_completed = _execute_task(
                executable,
                task["path"],
                output / "runs" / task["name"],
                task["name"],
                task["command"],
                output,
                dry_run=False,
                force=force,
            )
            all_commands.extend(task_commands)
            manifest["runs"][task["name"]] = {"commands": task_commands, "completed": task_completed}
            write_json(output / "diagnosis_manifest.json", manifest)
            if not task_completed:
                all_completed = False
                break
        if all_completed:
            formal = _formal_evidence(output, config, tasks)
        else:
            formal = _incomplete_formal_evidence(output, tasks)
        write_json(output / "formal_comparison.json", formal)

    classification = formal["classification"] if formal else (
        "unsupported" if screen["classification"] == "unsupported" else "inconclusive"
    )
    result = {
        "schema_version": SCHEMA_VERSION,
        "kind": "rolling_shutter_diagnosis_result",
        "classification": classification,
        "screening": screen,
        "formal_comparison": formal,
        "suggested_task": None,
        "disclaimer": "The result diagnoses whether a row-time model is supported; it is not a hardware measurement or production approval.",
    }
    if classification == "important":
        result["suggested_task"] = str(_suggested_task(config, formal, tasks, output))
    after_images = _selected_image_hashes(ordinary_observations)
    if after_images != before_images:
        raise RuntimeError("selected input images changed during diagnosis")
    manifest["status"] = "reporting"
    write_json(output / "result.json", result)
    write_json(output / "diagnosis_manifest.json", manifest)

    from .reporting import generate_reports

    generate_reports(output, result, config)
    manifest.update({"status": "completed", "completed_at": dt.datetime.now(dt.timezone.utc).isoformat()})
    write_json(output / "diagnosis_manifest.json", manifest)
    return {"status": "completed", "classification": classification, "commands": all_commands, "output_dir": str(output)}


def _write_rejected(path: Path, rows: list[dict[str, Any]]) -> None:
    columns = ["camera_id", "frame_id", "reason", "corner_count"]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = __import__("csv").DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
