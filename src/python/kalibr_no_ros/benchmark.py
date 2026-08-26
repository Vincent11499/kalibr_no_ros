"""Immutable benchmark archives and comparison against frozen baselines."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import resource
import shutil
import statistics
import subprocess
import time

import yaml

from kalibr_native_optimizer import profiling_enabled

from .task import (
    STANDARD_EXECUTION,
    TaskError,
    dump_yaml,
    load_task,
    load_yaml,
    resolve_execution,
    resolve_task_path,
)


BENCHMARK_SCHEMA_VERSION = 1
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def _git_identity(root):
    def command(*arguments):
        result = subprocess.run(
            ["git", "-C", str(root), *arguments],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, check=False)
        return result.stdout.strip() if result.returncode == 0 else None

    revision = command("rev-parse", "HEAD")
    status = command("status", "--porcelain")
    return {"revision": revision, "dirty": bool(status), "status": status}


def _absolute_effective_task(config, overrides):
    task = load_task(config)
    value = load_yaml(config)
    value.setdefault("dataset", {})["path"] = str(
        resolve_task_path(task, task["dataset"]["path"]))
    value["dataset"]["type"] = task["dataset"]["type"]
    target = value.get("target") or {}
    if target.get("path"):
        target["path"] = str(resolve_task_path(task, target["path"]))
    camera_calibration = value.get("camera_calibration")
    if isinstance(camera_calibration, str):
        value["camera_calibration"] = str(
            resolve_task_path(task, camera_calibration))
    elif isinstance(camera_calibration, dict) and camera_calibration.get("path"):
        camera_calibration["path"] = str(
            resolve_task_path(task, camera_calibration["path"]))
    for imu in value.get("imus") or ():
        imu["path"] = str(resolve_task_path(task, imu["path"]))

    calibration = value.setdefault("calibration", {})
    calibration.setdefault("interactive_report", False)
    if task["job"] == "camera_calibration":
        calibration.setdefault("shuffle", False)
        calibration.setdefault("synchronization_tolerance_s", 0.02)
        calibration.setdefault("qr_tolerance", 0.02)
        calibration.setdefault("information_gain_tolerance", 0.2)
        calibration.setdefault("min_views_for_outlier_statistics", 20)
        calibration.setdefault("remove_outliers", True)
        calibration.setdefault("final_filtering", True)
        calibration.setdefault("blake_zisserman", False)
    else:
        calibration.setdefault("max_iterations", 30)
        calibration.setdefault("time_offset_padding_s", 0.03)
        calibration.setdefault("reprojection_sigma_px", 1.0)
        calibration.setdefault("synchronize_clocks", False)
        calibration.setdefault("estimate_multi_imu_delay", False)
        calibration.setdefault("calibrate_time_offset", True)
        calibration.setdefault("recover_covariance", False)
        calibration.setdefault("recompute_camera_chain_extrinsics", False)

    value["execution"] = resolve_execution(
        task.get("execution"), overrides, standard_defaults=True)
    return value


def _process_table():
    table = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text(encoding="utf-8")
            close = stat.rfind(")")
            fields = stat[close + 2:].split()
            table[int(entry.name)] = int(fields[1])
        except (OSError, ValueError, IndexError):
            continue
    return table


def _descendants(root_pid):
    parents = _process_table()
    selected = {int(root_pid)}
    changed = True
    while changed:
        changed = False
        for pid, parent in parents.items():
            if parent in selected and pid not in selected:
                selected.add(pid)
                changed = True
    return sorted(selected)


def _process_memory(pid):
    rss = None
    pss = None
    try:
        for line in Path("/proc/{}/status".format(pid)).read_text().splitlines():
            if line.startswith("VmRSS:"):
                rss = int(line.split()[1]) * 1024
                break
    except (OSError, ValueError, IndexError):
        pass
    try:
        for line in Path("/proc/{}/smaps_rollup".format(pid)).read_text().splitlines():
            if line.startswith("Pss:"):
                pss = int(line.split()[1]) * 1024
                break
    except (OSError, ValueError, IndexError):
        pass
    return rss, pss


class _TreeSampler:
    def __init__(self, interval):
        self.interval = float(interval)
        self.samples = 0
        self.max_processes = 0
        self.rss_peak = None
        self.pss_peak = None
        self.incomplete_rss = 0
        self.incomplete_pss = 0

    def sample(self, root_pid):
        pids = _descendants(root_pid)
        values = [_process_memory(pid) for pid in pids]
        rss_values = [item[0] for item in values]
        pss_values = [item[1] for item in values]
        self.samples += 1
        self.max_processes = max(self.max_processes, len(pids))
        if rss_values and all(value is not None for value in rss_values):
            total = sum(rss_values)
            self.rss_peak = total if self.rss_peak is None else max(self.rss_peak, total)
        else:
            self.incomplete_rss += 1
        if pss_values and all(value is not None for value in pss_values):
            total = sum(pss_values)
            self.pss_peak = total if self.pss_peak is None else max(self.pss_peak, total)
        else:
            self.incomplete_pss += 1

    def result(self):
        return {
            "sample_interval_seconds": self.interval,
            "samples": self.samples,
            "max_processes": self.max_processes,
            "aggregate_rss_peak_bytes": self.rss_peak,
            "aggregate_pss_peak_bytes": self.pss_peak,
            "incomplete_rss_samples": self.incomplete_rss,
            "incomplete_pss_samples": self.incomplete_pss,
        }


def _rusage_delta(before, after):
    return {
        "user_seconds": after.ru_utime - before.ru_utime,
        "system_seconds": after.ru_stime - before.ru_stime,
        "minor_page_faults": after.ru_minflt - before.ru_minflt,
        "major_page_faults": after.ru_majflt - before.ru_majflt,
        "voluntary_context_switches": after.ru_nvcsw - before.ru_nvcsw,
        "involuntary_context_switches": after.ru_nivcsw - before.ru_nivcsw,
    }


def _trial(executable, job, effective_config, trial_dir, interval):
    trial_dir.mkdir(parents=True)
    output = trial_dir / "outputs"
    log_path = trial_dir / "stdout-stderr.log"
    timing_path = trial_dir / "timing.json"
    command = [
        str(executable), "calibrate",
        "cameras" if job == "camera_calibration" else "imu-camera",
        "--config", str(effective_config),
        "--output-dir", str(output),
        "--timing-json", str(timing_path),
    ]
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    start = time.perf_counter()
    sampler = _TreeSampler(interval)
    with log_path.open("wb") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        while process.poll() is None:
            sampler.sample(process.pid)
            time.sleep(interval)
        sampler.sample(process.pid)
    wall = time.perf_counter() - start
    after = resource.getrusage(resource.RUSAGE_CHILDREN)
    measurement = {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "command": command,
        "exit_status": process.returncode,
        "wall_seconds": wall,
        "cpu": _rusage_delta(before, after),
        "memory": sampler.result(),
    }
    files = {}
    for path in sorted(trial_dir.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            files[path.relative_to(trial_dir).as_posix()] = {
                "bytes": path.stat().st_size,
                "sha256": _sha256(path),
            }
    measurement["files"] = files
    _write_json(trial_dir / "manifest.json", measurement)
    if process.returncode:
        raise TaskError(
            "benchmark calibration failed with exit code {}; archive: {}".format(
                process.returncode, trial_dir))
    return measurement


def _summary(values):
    values = [float(value) for value in values if value is not None]
    if not values:
        return None
    median = statistics.median(values)
    return {
        "count": len(values),
        "median": median,
        "minimum": min(values),
        "maximum": max(values),
        "mad": statistics.median(abs(value - median) for value in values),
    }


def run_benchmark(prefix, config, archive_dir, name, repeat=1, **overrides):
    if not _NAME.match(name):
        raise TaskError("benchmark name must contain only letters, digits, ., _, or -")
    if repeat < 1:
        raise TaskError("repeat must be a positive integer")
    if not profiling_enabled():
        raise TaskError("benchmark run requires the project-profile build")

    prefix = Path(prefix).resolve()
    executable = prefix / "bin" / "kalibr-noros"
    if not executable.is_file():
        raise TaskError("missing benchmark executable: {}".format(executable))
    effective = _absolute_effective_task(config, overrides)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    revision = _git_identity(prefix.parent.parent).get("revision")
    suffix = revision[:8] if revision else "unknown"
    run_dir = Path(archive_dir).expanduser().resolve() / name / (stamp + "-" + suffix)
    if run_dir.exists():
        raise TaskError("benchmark archive already exists: {}".format(run_dir))
    run_dir.mkdir(parents=True)
    shutil.copy2(str(Path(config).resolve()), str(run_dir / "task.yaml"))
    effective_path = run_dir / "effective-task.yaml"
    dump_yaml(effective, effective_path)
    metadata = {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "name": name,
        "created_utc": stamp,
        "job": effective["job"],
        "build_prefix": str(prefix),
        "git": _git_identity(prefix.parent.parent),
        "task_sha256": _sha256(run_dir / "task.yaml"),
        "effective_task_sha256": _sha256(effective_path),
        "execution": effective["execution"],
        "repeat": repeat,
    }
    _write_json(run_dir / "run.json", metadata)

    trials = []
    interval = effective["execution"]["profiling_memory_sample_interval_s"]
    for index in range(repeat):
        trials.append(_trial(
            executable, effective["job"], effective_path,
            run_dir / "trial-{:03d}".format(index + 1), interval))
    summary = {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "name": name,
        "trials": ["trial-{:03d}".format(index + 1) for index in range(repeat)],
        "wall_seconds": _summary(item["wall_seconds"] for item in trials),
        "aggregate_rss_peak_bytes": _summary(
            item["memory"]["aggregate_rss_peak_bytes"] for item in trials),
        "aggregate_pss_peak_bytes": _summary(
            item["memory"]["aggregate_pss_peak_bytes"] for item in trials),
        "cpu_seconds": _summary(
            item["cpu"]["user_seconds"] + item["cpu"]["system_seconds"]
            for item in trials),
    }
    _write_json(run_dir / "summary.json", summary)
    return run_dir


def _yaml_difference(reference, candidate, atol, rtol):
    left = yaml.safe_load(Path(reference).read_text(encoding="utf-8"))
    right = yaml.safe_load(Path(candidate).read_text(encoding="utf-8"))
    result = {"compatible": True, "max_absolute_difference": 0.0,
              "max_relative_difference": 0.0, "first_difference": None}

    def visit(a, b, path):
        if isinstance(a, bool) or isinstance(b, bool):
            if a != b and result["first_difference"] is None:
                result["compatible"] = False
                result["first_difference"] = path
            return
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            absolute = abs(float(a) - float(b))
            relative = absolute / max(abs(float(a)), abs(float(b)), 1e-300)
            result["max_absolute_difference"] = max(
                result["max_absolute_difference"], absolute)
            result["max_relative_difference"] = max(
                result["max_relative_difference"], relative)
            if not math.isclose(float(a), float(b), abs_tol=atol, rel_tol=rtol):
                if result["first_difference"] is None:
                    result["compatible"] = False
                    result["first_difference"] = path
            return
        if isinstance(a, dict) and isinstance(b, dict):
            if a.keys() != b.keys():
                result["compatible"] = False
                result["first_difference"] = result["first_difference"] or path + ".<keys>"
                return
            for key in a:
                visit(a[key], b[key], "{}.{}".format(path, key))
            return
        if isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b):
                result["compatible"] = False
                result["first_difference"] = result["first_difference"] or path + ".<length>"
                return
            for index, (first, second) in enumerate(zip(a, b)):
                visit(first, second, "{}[{}]".format(path, index))
            return
        if a != b and result["first_difference"] is None:
            result["compatible"] = False
            result["first_difference"] = path

    visit(left, right, "$")
    result["reference_sha256"] = _sha256(reference)
    result["candidate_sha256"] = _sha256(candidate)
    result["byte_identical"] = (
        Path(reference).read_bytes() == Path(candidate).read_bytes())
    return result


def _load_registry(path, baseline_id):
    value = load_yaml(path)
    if value.get("schema_version") != BENCHMARK_SCHEMA_VERSION:
        raise TaskError("unsupported benchmark baseline registry schema")
    try:
        return value["baselines"][baseline_id]
    except (KeyError, TypeError):
        raise TaskError("unknown benchmark baseline id: {}".format(baseline_id))


def _validate_frozen_baseline(baseline):
    project = Path(baseline["project_output"])
    reference = Path(baseline["reference_output"])
    paths = {
        "task": Path(baseline["task"]),
        "project_calibration": project / "calibration.yaml",
        "project_results": project / "results.txt",
        "project_timing": Path(baseline["project_timing"]),
        "reference_calibration": reference / "calibration.yaml",
    }
    expected = baseline.get("frozen_sha256") or {}
    for name, path in paths.items():
        if not path.is_file():
            raise TaskError("frozen baseline artifact is missing: {}".format(path))
        if expected.get(name) != _sha256(path):
            raise TaskError("frozen baseline artifact changed: {}".format(path))


def _validate_candidate_contract(baseline, candidate):
    contract = baseline.get("contract") or {}
    effective_path = Path(candidate) / "effective-task.yaml"
    if not effective_path.is_file():
        raise TaskError("candidate archive has no effective-task.yaml")
    task = load_yaml(effective_path)
    actual = {
        "job": task.get("job"),
        "dataset_path": str(Path(task.get("dataset", {}).get("path", "")).resolve()),
        "camera_models": [camera.get("model") for camera in task.get("cameras") or ()],
        "imu_models": [imu.get("model", "calibrated") for imu in task.get("imus") or ()],
        "detector_processes": task.get("execution", {}).get("detector_processes"),
        "optimizer_threads": task.get("execution", {}).get("optimizer_threads"),
    }
    for name, expected in contract.items():
        if actual.get(name) != expected:
            raise TaskError(
                "candidate does not match baseline contract: {}={!r}, expected {!r}".format(
                    name, actual.get(name), expected))
    if task.get("job") == "camera_calibration":
        if task.get("calibration", {}).get("shuffle") is not False:
            raise TaskError("camera benchmark requires calibration.shuffle: false")


def _category_totals(path):
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if document.get("status") != "ok":
        raise TaskError("timing report is not complete: {}".format(path))
    rows = document.get("summary", {}).get("category_totals", [])
    return {row["name"]: float(row["wall_seconds"]) for row in rows}


def compare_benchmark(registry, baseline_id, candidate, atol=1e-8, rtol=1e-8):
    baseline = _load_registry(registry, baseline_id)
    _validate_frozen_baseline(baseline)
    candidate = Path(candidate).resolve()
    _validate_candidate_contract(baseline, candidate)
    summary = json.loads((candidate / "summary.json").read_text(encoding="utf-8"))
    first_trial = candidate / summary["trials"][0]
    candidate_output = first_trial / "outputs"
    project_output = Path(baseline["project_output"])
    reference_output = Path(baseline["reference_output"])
    for path in (project_output, reference_output, candidate_output):
        if not (path / "calibration.yaml").is_file():
            raise TaskError("missing benchmark calibration output: {}".format(path))

    candidate_wall = summary["wall_seconds"]["median"]
    baseline_wall = float(baseline["wall_seconds"])
    delta_percent = 100.0 * (candidate_wall - baseline_wall) / baseline_wall
    candidate_rss = summary["aggregate_rss_peak_bytes"]["median"]
    baseline_rss = baseline.get("aggregate_rss_peak_bytes")
    baseline_categories = _category_totals(baseline["project_timing"])
    candidate_categories = _category_totals(first_trial / "timing.json")
    category_names = sorted(set(baseline_categories) | set(candidate_categories))
    categories = {}
    for name in category_names:
        old = baseline_categories.get(name)
        new = candidate_categories.get(name)
        categories[name] = {
            "baseline_wall_seconds": old,
            "candidate_wall_seconds": new,
            "delta_percent": (
                100.0 * (new - old) / old
                if old not in (None, 0.0) and new is not None else None),
        }
    comparison = {
        "schema_version": BENCHMARK_SCHEMA_VERSION,
        "baseline_id": baseline_id,
        "candidate": str(candidate),
        "performance": {
            "baseline_wall_seconds": baseline_wall,
            "candidate_wall_seconds": candidate_wall,
            "wall_delta_percent": delta_percent,
            "single_run_below_five_percent_is_inconclusive": (
                summary["wall_seconds"]["count"] == 1 and abs(delta_percent) < 5.0),
            "baseline_aggregate_rss_peak_bytes": baseline_rss,
            "candidate_aggregate_rss_peak_bytes": candidate_rss,
            "memory_values_are_not_directly_comparable": baseline_rss is None,
        },
        "category_comparison": categories,
        "project_numeric": _yaml_difference(
            project_output / "calibration.yaml",
            candidate_output / "calibration.yaml", atol, rtol),
        "reference_numeric": _yaml_difference(
            reference_output / "calibration.yaml",
            candidate_output / "calibration.yaml", atol, rtol),
    }
    json_path = candidate / "comparison-{}.json".format(baseline_id)
    _write_json(json_path, comparison)
    markdown = candidate / "comparison-{}.md".format(baseline_id)
    markdown.write_text(
        "# Benchmark comparison: {0}\n\n"
        "- Baseline wall: {1:.3f} s\n"
        "- Candidate wall: {2:.3f} s\n"
        "- Wall delta: {3:+.2f}%\n"
        "- Project numeric compatible: {4}\n"
        "- Reference numeric compatible: {5}\n"
        "- Single-run <5% inconclusive: {6}\n".format(
            baseline_id, baseline_wall, candidate_wall, delta_percent,
            comparison["project_numeric"]["compatible"],
            comparison["reference_numeric"]["compatible"],
            comparison["performance"][
                "single_run_below_five_percent_is_inconclusive"]),
        encoding="utf-8")
    return json_path, markdown
