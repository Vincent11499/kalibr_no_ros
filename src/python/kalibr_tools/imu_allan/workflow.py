"""Run the pinned CSV Allan analysis and publish a new noise configuration."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path

from kalibr_no_ros.task.document import dump_yaml
from kalibr_no_ros.version import CONFIG_SCHEMA_VERSION, RESULT_SCHEMA_VERSION

from .allan_core import UPSTREAM_REVISION, compute_curves, validate_curves
from .input_format import materialize_selected_samples, resolve_dataset, validate_input
from .upstream_analysis import analyze


NOISE_FIELDS = (
    "accelerometer_noise_density", "accelerometer_random_walk",
    "gyroscope_noise_density", "gyroscope_random_walk",
)


def _save_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def run_analysis(dataset, output_dir, *, topic="/imu/data", start_ns=0, end_ns=None, progress=None):
    if not isinstance(topic, str) or not topic or any(character.isspace() for character in topic):
        raise ValueError("topic must be a non-empty string without whitespace")
    dataset, csv_path = resolve_dataset(dataset)
    requested_output = Path(output_dir).expanduser().absolute()
    if requested_output.is_symlink():
        raise FileExistsError(f"output already exists: {requested_output}")
    output = requested_output.resolve()
    if any(output == root or root in output.parents for root in (dataset, csv_path.parent)):
        raise ValueError("output must not be inside the source dataset")
    staging = output.with_name(output.name + ".inprogress")
    failed = output.with_name(output.name + ".failed")
    for path in (output, staging, failed):
        if path.exists() or path.is_symlink():
            raise FileExistsError(f"output already exists: {path}")

    if progress:
        progress("Validating the complete source CSV...")
    source = validate_input(dataset, csv_path, start_ns, end_ns)
    actual_rate = float(source["effective_rate_hz"])
    measure_rate = math.floor(actual_rate)
    maximum_bin_size = int((9999 * 0.1) * measure_rate)
    if measure_rate < 10 or source["messages"] <= 2 * maximum_bin_size:
        raise ValueError(
            "the fixed 0.1..999.9 s grid requires >=10 Hz and more than two "
            "complete bins at tau=999.9 s"
        )
    run = {
        "schema_version": RESULT_SCHEMA_VERSION,
        "kind": "allan_variance_python_run",
        "status": "running",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "source": source,
        "compatibility_target": {
            "repository": "ori-drs/allan_variance_ros", "revision": UPSTREAM_REVISION,
        },
        "options": {
            "dataset": str(dataset), "output": str(output), "topic": topic,
            "start_offset_ns": start_ns, "end_offset_ns": end_ns,
        },
        "measure_rate_hz": measure_rate,
        "imu_skip": 1,
        "tau_scale_note": "Integer measure_rate sets nominal bin durations; nonuniform timestamps are not resampled.",
    }
    staging.mkdir(parents=True)
    try:
        work = staging / "work"
        work.mkdir()
        _save_json(work / "run.json", run)
        selected_path = work / "selected_samples.npy"
        if progress:
            progress(f"Materializing {source['messages']} selected samples...")
        materialize_selected_samples(csv_path, selected_path, source["messages"], start_ns, end_ns)
        curve_path = work / "allan_variance.csv"
        compute_curves(selected_path, curve_path, measure_rate, progress=progress)
        validation = validate_curves(selected_path, curve_path, measure_rate)
        if progress:
            progress("Curves validated; fitting parameters and rendering reports...")
        parameters, fit = analyze(curve_path, topic, actual_rate)
        if any(not math.isfinite(parameters[key]) or parameters[key] <= 0 for key in NOISE_FIELDS):
            raise RuntimeError("invalid fitted parameter")
        dump_yaml({"schema_version": CONFIG_SCHEMA_VERSION, "kind": "imu_configuration", **parameters}, staging / "imu_noise.yaml")
        validation["parameters"] = parameters
        _save_json(work / "validation.json", validation)
        _save_json(work / "fit.json", fit)
        from .reporting import generate_reports

        generate_reports(staging, source, parameters, fit, measure_rate)
        selected_path.unlink()
        run["status"] = "pass"
        run["finished_utc"] = datetime.now(timezone.utc).isoformat()
        _save_json(work / "run.json", run)
        if output.exists() or output.is_symlink():
            raise FileExistsError(f"output already exists: {output}")
        staging.rename(output)
    except BaseException as exc:
        run["status"] = "fail"
        run["error"] = f"{type(exc).__name__}: {exc}"
        run["finished_utc"] = datetime.now(timezone.utc).isoformat()
        if (staging / "work").is_dir():
            _save_json(staging / "work" / "run.json", run)
        if not failed.exists() and not failed.is_symlink():
            staging.rename(failed)
        raise
    return {
        "status": "pass", "output": str(output),
        "imu_noise": str(output / "imu_noise.yaml"),
        "report_html": str(output / "imu_allan.report.html"),
        "report_pdf": str(output / "imu_allan.report.pdf"),
    }
