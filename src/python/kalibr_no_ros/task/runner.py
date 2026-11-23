"""Stage, execute, and safely publish one calibration task."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from datetime import datetime, timezone
import copy
import json
import shutil
import tempfile
import time

from ..version import (
    CALIBRATION_BEHAVIOR_VERSION,
    CONFIG_SCHEMA_VERSION,
    RESULT_SCHEMA_VERSION,
    SUPPORTED_RESULT_SCHEMA_VERSIONS,
    VERSION,
    supports_schema_version,
)
from ..filesystem import file_sha256 as _sha256
from .document import TaskError, dump_yaml, load_yaml, resolve_task_path
from .validation import load_task
from .execution import resolve_execution
from .initialization import resolve_initialization, _camera_calibration_ids, _write_initialization_config, _write_initialization_report
from .jobs import CAMERA_RS_JOB, CAMERA_IMU_RS_JOB, CAMERA_IMU_JOBS, CAMERA_CALIBRATION_JOBS, ROLLING_SHUTTER_JOBS
from .solver_adapter import (_validate_camera_rs_backend, _staged_solver_invocation, _collect_outputs, _overlay_structured_camera_snapshots, _capture_solver_logs, _run_legacy)
from ..results.delivery import (FLAT_LAYOUT, _validate_layout, result_name, _check_owned, _delivery_paths, publish, _print_published_paths)

try:
    from .._build_info import GIT_COMMIT, SOURCE_VARIANT
except ImportError:
    GIT_COMMIT = None
    SOURCE_VARIANT = "project"

MANAGED_OUTPUTS = {
    "calibration.yaml",
    "initialization_report.yaml",
    "observability.yaml",
    "results.txt",
    "report.pdf",
    "poses.csv",
    "timing.json",
    "metrics.json", "assessment.json", "report.html", "run_manifest.json",
    "task_resolved.yaml", "validation.json", "stdout.log", "stderr.log",
}
MANAGED_DIRECTORIES = {"observations", "opencv", "visualizations", "images"}


def prepare_output_directory(path, force=False):
    output = Path(path).expanduser().resolve()
    forbidden = {
        Path("/").resolve(),
        Path.home().resolve(),
        Path.cwd().resolve(),
    }
    if output in forbidden:
        raise TaskError("unsafe output directory: {}".format(output))
    output.mkdir(parents=True, exist_ok=True)
    entries = list(output.iterdir())
    unknown = [entry.name for entry in entries if entry.name not in MANAGED_OUTPUTS | MANAGED_DIRECTORIES]
    if unknown:
        raise TaskError(
            "output directory contains unmanaged entries: {}".format(
                ", ".join(sorted(unknown))
            )
        )
    existing = entries
    if existing and not force:
        raise TaskError("output files already exist; pass --force to replace them")
    manifest_path = output / "run_manifest.json"
    registered = set()
    registered_directories = set()
    if manifest_path.is_file() and not manifest_path.is_symlink():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            registered = {entry["path"] if isinstance(entry, dict) else entry for entry in manifest.get("files", [])}
            registered_directories = set(manifest.get("directories", []))
        except (ValueError, TypeError, KeyError) as error:
            raise TaskError("invalid output manifest: {}".format(error)) from error
    descendants = list(output.rglob("*"))
    # Inspect the complete tree before removing anything.  A familiar top-level
    # directory name never authorizes deletion of unregistered user files.
    for entry in descendants:
        relative = entry.relative_to(output).as_posix()
        if entry.is_symlink():
            raise TaskError("managed output cannot contain symlinks: {}".format(relative))
        if entry.is_dir():
            if relative not in registered_directories:
                raise TaskError("unmanaged output directory: {}".format(relative))
        elif relative not in MANAGED_OUTPUTS and relative not in registered:
            raise TaskError("unmanaged output file: {}".format(relative))
    for entry in sorted(descendants, key=lambda item: len(item.parts), reverse=True):
        if entry.is_dir():
            entry.rmdir()
        else:
            entry.unlink()
    return output


def write_run_manifest(output, *, status, task=None, failure=None,
                       source_run=None, solver_backend=None):
    """Inventory only files created inside this run, including format metadata."""
    output = Path(output)
    document = {
        "schema_version": RESULT_SCHEMA_VERSION,
        "kind": "calibration_run",
        "software": {"name": "kalibr-noros", "version": VERSION,
                     "calibration_behavior_version": CALIBRATION_BEHAVIOR_VERSION,
                     "git_commit": GIT_COMMIT, "source_variant": SOURCE_VARIANT},
        "updated_at": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "files": [],
        "directories": [],
    }
    if solver_backend is not None:
        if solver_backend not in {"system", "native"}:
            raise TaskError(
                "solver_backend must be system or native")
        document["solver_backend"] = solver_backend
    if task is not None:
        document["job"] = task["job"]
        document["dataset"] = dict(task["dataset"])
        document["dataset"]["path"] = str(resolve_task_path(task, task["dataset"]["path"]))
        if task["dataset"]["type"] == "directory":
            manifest = Path(document["dataset"]["path"]) / "dataset.yaml"
            if manifest.is_file():
                try:
                    document["dataset"]["id"] = load_yaml(manifest).get("dataset_id")
                except (TaskError, OSError) as error:
                    document["dataset"]["metadata_error"] = str(error)
    effective_path = output / "task_resolved.yaml"
    if effective_path.is_file():
        document["task_sha256"] = _sha256(effective_path)
        effective = load_yaml(effective_path)
        inputs = [(key, effective.get(key)) for key in ("target", "camera_calibration", "initialization")]
        inputs += [("imu{}".format(i), block) for i, block in enumerate(effective.get("imus", []))]
        document["input_documents"] = []
        for role, block in inputs:
            if not isinstance(block, dict) or not block.get("path"):
                continue
            source = Path(block["path"])
            record = {"role": role, "path": str(source)}
            try:
                record["sha256"] = _sha256(source)
                record["document"] = load_yaml(source)
            except (TaskError, OSError) as error:
                record["error"] = str(error)
            document["input_documents"].append(record)
    if source_run is not None:
        document["source_run"] = str(Path(source_run).resolve())
    if failure is not None:
        document["failure"] = {"type": type(failure).__name__, "message": str(failure)}
    for path in sorted(output.rglob("*")):
        if path.is_symlink():
            raise TaskError("run output contains a symlink: {}".format(path))
        relative = path.relative_to(output).as_posix()
        if path.is_dir():
            document["directories"].append(relative)
        elif path.name != "run_manifest.json":
            document["files"].append({
                "path": relative,
                "size_bytes": path.stat().st_size,
                "sha256": _sha256(path),
                "schema_version": RESULT_SCHEMA_VERSION,
            })
    destination = output / "run_manifest.json"
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=output, prefix=".manifest-", delete=False) as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(destination)
    return document


def _read_observability(path):
    path = Path(path)
    if not path.is_file():
        return None
    report = load_yaml(path)
    calibration = report.get("calibration") or {}
    return {
        "status": report.get("status"),
        "quality": report.get("quality"),
        "rank": calibration.get("rank"),
        "columns": calibration.get("columns"),
        "deficiency": calibration.get("deficiency"),
        "operational_rank": calibration.get("operational_rank"),
        "operational_deficiency": calibration.get(
            "operational_deficiency"),
    }


def _finish_initialization_report(path, status, *, error=None,
                                  observability_path=None,
                                  calibration_path=None):
    path = Path(path)
    if not path.is_file():
        return
    report = load_yaml(path)
    report["status"] = status
    if observability_path is not None:
        summary = _read_observability(observability_path)
        if summary is not None:
            report["observability"] = summary
    if calibration_path is not None and Path(calibration_path).is_file():
        report["result"] = {
            "path": "calibration.yaml",
            "sha256": _sha256(calibration_path),
        }
    if error is not None:
        report["failure"] = {
            "type": type(error).__name__,
            "message": str(error),
        }
    dump_yaml(report, path)


def _preserve_diagnostic_sidecars(work, output):
    for name in ("initialization_report.yaml", "observability.yaml"):
        source = Path(work) / name
        destination = Path(output) / name
        if source.is_file() and not destination.exists():
            shutil.move(str(source), str(destination))


def _staged_dataset_path(task, config, output_dir, initialization):
    dataset_path = resolve_task_path(task, task["dataset"]["path"])
    output_path = Path(output_dir).expanduser().resolve()
    if (output_path == dataset_path or dataset_path in output_path.parents
            or output_path in dataset_path.parents):
        raise TaskError(
            "output directory must be separate from the input dataset")
    referenced_inputs = [Path(config).expanduser().resolve()]
    for block in [
            task.get("target"), task.get("camera_calibration"),
            *task.get("imus", []),
    ]:
        value = block.get("path") if isinstance(block, dict) else block
        if value:
            referenced_inputs.append(resolve_task_path(task, value))
    if initialization is not None:
        referenced_inputs.append(initialization["path"])
    for source in referenced_inputs:
        if source == output_path or output_path in source.parents:
            raise TaskError(
                "output directory contains an input file: {}".format(source))
    return dataset_path


def _resolved_staged_task(
        task, dataset_path, initialization, overrides, solver_backend):
    resolved = copy.deepcopy({
        key: value for key, value in task.items() if not key.startswith("_")
    })
    resolved["schema_version"] = CONFIG_SCHEMA_VERSION
    resolved["execution"] = resolve_execution(task.get("execution"), overrides)
    resolved["dataset"] = dict(task["dataset"], path=str(dataset_path))
    for key in ("target", "camera_calibration"):
        block = resolved.get(key)
        if isinstance(block, dict) and block.get("path"):
            block["path"] = str(resolve_task_path(task, block["path"]))
        elif isinstance(block, str):
            resolved[key] = {"path": str(resolve_task_path(task, block))}
    for imu in resolved.get("imus", []):
        imu["path"] = str(resolve_task_path(task, imu["path"]))
    if initialization is not None:
        resolved["initialization"] = {
            "path": str(initialization["path"]),
        }
    if solver_backend is not None:
        # Managed provenance; the portable task schema does not accept it.
        resolved["solver_backend"] = solver_backend
    return resolved


def _validate_staged_input(
        task, resolved_task, output, temporary, solver_backend):
    from .validation import validate_task

    if solver_backend is not None:
        _validate_camera_rs_backend(task, solver_backend)
        validation_task = copy.deepcopy(resolved_task)
        validation_task.pop("solver_backend", None)
        validation_path = temporary / "task_for_validation.yaml"
        dump_yaml(validation_task, validation_path)
    else:
        validation_path = output / "task_resolved.yaml"
    validation = validate_task(
        validation_path, solver_backend=solver_backend)
    if solver_backend is not None:
        validation["solver_backend"] = solver_backend
    (output / "validation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    if validation["status"] != "passed":
        raise TaskError("input validation failed: {}".format(
            "; ".join(validation["errors"])))


def _print_solver_start(task, resolved_task, initialization):
    dataset = resolved_task["dataset"]
    if task["job"] in CAMERA_CALIBRATION_JOBS:
        camera_ids = [
            camera.get("id", "cam{}".format(index))
            for index, camera in enumerate(task["cameras"])
        ]
    else:
        camera_ids = _camera_calibration_ids(task)
    calibration = task.get("calibration") or {}
    solver_mode = (
        calibration.get("solver_mode", "incremental")
        if task["job"] == "camera_calibration" else "joint_batch"
    )
    print("[kalibr] task={} dataset={} ({}) cameras={} [{}] solver_mode={}".format(
        task["job"], dataset["path"], dataset["type"], len(camera_ids),
        ", ".join(camera_ids), solver_mode))
    execution = resolved_task["execution"]
    print(
        "[kalibr] detector_processes={} detector_opencv_threads={} "
        "optimizer_threads={} detector_inflight_per_worker={}".format(
            execution.get("detector_processes", "native_default"),
            execution.get("detector_opencv_threads", "native_default"),
            execution.get("optimizer_threads", "native_default"),
            execution.get("detector_inflight_per_worker", "native_default"),
        ))
    if initialization is not None:
        print("[kalibr] initialization_seed={}".format(
            initialization["path"]))
    if solver_mode == "full_batch":
        incremental_fields = {
            "shuffle", "qr_tolerance", "information_gain_tolerance",
            "min_views_for_outlier_statistics", "remove_outliers",
            "final_filtering", "blake_zisserman",
        }
        ignored = sorted(incremental_fields & set(calibration))
        if ignored:
            print("[kalibr] warning: full_batch ignores incremental fields: {}".format(
                ", ".join("calibration." + key for key in ignored)))


def _attach_required_observability(
        artifacts, temporary, expected_job, rs_backend, initialization, task):
    full_batch = (
        expected_job == "camera_calibration"
        and (task.get("calibration") or {}).get(
            "solver_mode", "incremental") == "full_batch"
    )
    required = (
        expected_job == CAMERA_IMU_RS_JOB
        or expected_job == CAMERA_RS_JOB and rs_backend == "system"
        or initialization is not None and rs_backend != "native"
        or full_batch
    )
    if not required:
        return
    observability = _read_observability(temporary / "observability.yaml")
    if observability is None:
        raise TaskError("calibration did not produce observability.yaml")
    rank_required = (
        expected_job == CAMERA_IMU_RS_JOB
        or expected_job == CAMERA_RS_JOB and rs_backend == "system"
        or initialization is not None and rs_backend != "native"
    )
    if rank_required and observability.get("status") != "full_rank":
        raise TaskError(
            "calibration is rank deficient: rank {}/{} "
            "(deficiency {})".format(
                observability.get("rank"),
                observability.get("columns"),
                observability.get("deficiency"),
            ))
    if expected_job in ROLLING_SHUTTER_JOBS or full_batch:
        artifacts["observability"] = observability


def _record_staged_failure(
        initialization_report, temporary, output, task, error,
        solver_completed, input_validated, solver_backend):
    if initialization_report is not None:
        observability_path = temporary / "observability.yaml"
        observability = _read_observability(observability_path)
        status = (
            "failed_rank_deficient"
            if observability is not None
            and observability.get("status") in {
                "rank_deficient", "no_information"
            }
            else "failed"
        )
        report_path = (
            initialization_report
            if initialization_report.is_file()
            else output / "initialization_report.yaml"
        )
        _finish_initialization_report(
            report_path, status, error=error,
            observability_path=observability_path)
    _preserve_diagnostic_sidecars(temporary, output)
    status = (
        "output_failed" if solver_completed
        else "failed" if input_validated else "input_failed"
    )
    write_run_manifest(
        output, status=status, task=task, failure=error,
        solver_backend=solver_backend)


@contextmanager
def _output_status_stage(task, name):
    if not bool((task.get("output") or {}).get("verbose")):
        yield
        return
    started = time.perf_counter()
    print("[kalibr] {}: start".format(name))
    try:
        yield
    except BaseException:
        print("[kalibr] {}: failed ({:.3f} s)".format(
            name, time.perf_counter() - started))
        raise
    else:
        print("[kalibr] {}: done ({:.3f} s)".format(
            name, time.perf_counter() - started))


def _complete_staged_output(
        temporary, output, expected_job, task, artifacts, rs_backend,
        initialization_report, report_generator, solver_backend):
    export_poses = bool((task.get("output") or {}).get("export_poses"))
    with _output_status_stage(task, "result collection"):
        result = _collect_outputs(
            temporary, output, expected_job, export_poses, task=task,
            rs_solver_backend=rs_backend)
    if expected_job == CAMERA_RS_JOB:
        _overlay_structured_camera_snapshots(result, artifacts)
    if initialization_report is not None:
        _finish_initialization_report(
            output / "initialization_report.yaml",
            "completed",
            observability_path=output / "observability.yaml",
            calibration_path=output / "calibration.yaml",
        )
    from ..results.delivery import apply_camera_ids, enrich_result
    with _output_status_stage(task, "metrics and report"):
        apply_camera_ids(task, artifacts, result)
        report = report_generator(
            artifacts, result, output, task["output"])
        enrich_result(result, report["metrics"])
    with _output_status_stage(task, "result serialization"):
        dump_yaml(result, output / "calibration.yaml")
        write_run_manifest(
            output, status="completed", task=task,
            solver_backend=solver_backend)


def _run_task_staged(prefix, config, output_dir, expected_job, force=False, _task=None, **overrides):
    from ..native_bridge.evidence import run_context
    from ..results.reporting import generate_report

    task = _task if _task is not None else load_task(config, expected_job=expected_job)
    rs_backend = overrides.get("_rs_solver_backend", "system")
    public_solver_backend = (
        rs_backend if expected_job == CAMERA_RS_JOB else None)
    initialization = resolve_initialization(
        task,
        expected_job,
        initialization=overrides.get("initialization"),
    )
    dataset_path = _staged_dataset_path(
        task, config, output_dir, initialization)
    output = prepare_output_directory(output_dir, force=force)
    resolved_task = _resolved_staged_task(
        task, dataset_path, initialization, overrides, public_solver_backend)
    dump_yaml(resolved_task, output / "task_resolved.yaml")
    write_run_manifest(
        output, status="running", task=task,
        solver_backend=public_solver_backend)
    input_validated = False
    solver_completed = False
    with tempfile.TemporaryDirectory(prefix="kalibr-noros-") as temporary_name:
        temporary = Path(temporary_name)
        initialization_config = _write_initialization_config(
            initialization, temporary)
        initialization_report = _write_initialization_report(
            initialization, task, temporary,
            solver_backend=public_solver_backend)
        try:
            _validate_staged_input(
                task, resolved_task, output, temporary,
                public_solver_backend)
            if public_solver_backend != "native":
                input_validated = True
            command, arguments = _staged_solver_invocation(
                task, expected_job, temporary, overrides,
                initialization_config, rs_backend)
            _print_solver_start(task, resolved_task, initialization)
            if public_solver_backend == "native":
                # Native-only cardinality/options and timestamp-rate inference
                # are all input adaptation; no solver code has run yet.
                input_validated = True
            with run_context(expected_job, capture_history=task["output"]["archive_selection_history"]) as context:
                with _capture_solver_logs(
                        output, verbose=bool(task["output"].get("verbose"))):
                    import kalibr_runtime as native_runtime
                    with native_runtime.status_stage("native calibration solve"):
                        _run_legacy(prefix, command, arguments, temporary)
            artifacts = context.artifacts
            if artifacts.get("state") != "completed":
                raise TaskError("native calibration did not publish a completed result")
            # The numerical solve is complete once the native artifact snapshot
            # says so.  Any later failure belongs to output/assessment handling;
            # delivery still publishes only after the whole staging run succeeds.
            solver_completed = True
            artifacts["dataset"] = dict(resolved_task["dataset"])
            if expected_job == CAMERA_RS_JOB:
                artifacts["rolling_shutter_solver_backend"] = rs_backend
                if rs_backend == "native":
                    artifacts["observability"] = {
                        "status": "unavailable",
                        "quality": "unavailable",
                        "rank": None,
                        "columns": None,
                        "deficiency": None,
                        "operational_rank": None,
                        "operational_deficiency": None,
                        "reason": (
                            "the temporary native single-camera RS solver does not expose a final Hessian rank report"),
                    }
            _attach_required_observability(
                artifacts, temporary, expected_job, rs_backend,
                initialization, task)
            _complete_staged_output(
                temporary, output, expected_job, task, artifacts, rs_backend,
                initialization_report, generate_report,
                public_solver_backend)
        except Exception as error:
            _record_staged_failure(
                initialization_report, temporary, output, task, error,
                solver_completed, input_validated, public_solver_backend)
            raise
    return output


def discover_camera_result(task, output):
    """Never guess between multiple models/runs or silently replace an explicit input."""
    from .document import TaskError, load_yaml, resolve_task_path
    if task['job'] not in CAMERA_IMU_JOBS or task.get('camera_calibration'):
        return
    expected = None
    if task['dataset']['type'] == 'directory':
        from kalibr_bag_io.directory import DirectoryReader
        reader = DirectoryReader(resolve_task_path(task, task['dataset']['path']))
        expected = [c['id'] for c in reader.manifest.get('cameras', [])]
    matches = []
    for path in sorted(Path(output).glob('*.yaml')):
        if path.is_symlink() or not path.is_file():
            continue
        try:
            value = load_yaml(path)
        except (OSError, ValueError):
            continue
        if (not supports_schema_version(
                value.get('schema_version'), SUPPORTED_RESULT_SCHEMA_VERSIONS)
                or value.get('kind') != 'calibration_result'
                or value.get('calibration_type') != 'cameras'):
            continue
        cameras = value.get('cameras', [])
        if not isinstance(cameras, list) or len(cameras) < 2 or not all(isinstance(c, dict) for c in cameras):
            continue
        ids = [c.get('id') for c in cameras]
        if any(not isinstance(i, str) for i in ids) or len(set(ids)) != len(ids):
            continue
        if expected is not None and set(ids) != set(expected):
            continue
        try:
            from .validation import load_cameras
            candidate_task = dict(task, camera_calibration={'path': str(path.resolve())})
            load_cameras(candidate_task)
        except (ValueError, OSError, KeyError, TypeError):
            continue
        matches.append(path.resolve())
    if len(matches) != 1:
        reason = 'none found' if not matches else ', '.join(p.name for p in matches)
        raise TaskError('camera_calibration.path omitted: expected one matching stereo/multicamera result in output directory; {}. Specify camera_calibration.path explicitly.'.format(reason))
    task['camera_calibration'] = {'path': str(matches[0])}



def run_delivery(prefix, config, output_dir, expected_job, force, overrides):
    run_overrides = dict(overrides)
    layout = _validate_layout(
        run_overrides.pop('_delivery_layout', FLAT_LAYOUT))
    before_publish = run_overrides.pop('_before_publish', None)
    if before_publish is not None and not callable(before_publish):
        raise TaskError('_before_publish must be callable')
    task = load_task(config, expected_job)
    output = Path(output_dir).expanduser().resolve()
    dataset = resolve_task_path(task, task['dataset']['path'])
    if output in {Path('/'), Path.home(), Path.cwd()} or output == dataset or dataset in output.parents or output in dataset.parents:
        raise TaskError('output directory must be separate from the input dataset and workspace root')
    discover_camera_result(task, output)
    rs_solver_backend = overrides.get('_rs_solver_backend', 'system')
    name = result_name(task, rs_solver_backend)
    _check_owned(output, name, force, layout)
    inputs = [Path(config).resolve(), *([Path(run_overrides['initialization']).resolve()] if run_overrides.get('initialization') else [])]
    for block in [task.get('target'), task.get('camera_calibration'), task.get('initialization'), *task.get('imus', [])]:
        value = block.get('path') if isinstance(block, dict) else block
        if value:
            source = resolve_task_path(task, value)
            inputs.append(source)
    delivery_paths = _delivery_paths(output, name, layout)
    replaced = {delivery_paths[key] for key in ('result', 'html', 'pdf', 'text')}
    for source in inputs:
        if source in replaced or output / name in source.parents:
            raise TaskError('delivery would overwrite its input: {}'.format(source))
    with tempfile.TemporaryDirectory(prefix='kalibr-delivery-') as temporary:
        stage = Path(temporary) / 'run'
        if run_overrides.get('timing_json'):
            run_overrides['timing_json'] = str(stage / 'timing.json')
        try:
            _run_task_staged(prefix, config, stage, expected_job, _task=task, **run_overrides)
        except Exception:
            # Failed attempts never replace a previous successful delivery.
            if task['output'].get('save_diagnostics') and stage.exists():
                from uuid import uuid4
                failed_name = name + '_failed'
                while (output / failed_name).exists() or (output / failed_name).is_symlink():
                    failed_name = name + '_failed_' + uuid4().hex[:12]
                try:
                    publish(
                        stage, output, failed_name, task['output'], False,
                        failed=True, layout=layout)
                except Exception as diagnostic_error:
                    import sys
                    print('Could not preserve failure diagnostics: {}'.format(diagnostic_error), file=sys.stderr)
            raise
        if before_publish is not None:
            before_publish()
        published = publish(
            stage, output, name, task['output'], force, layout=layout)
        paths = _delivery_paths(output, name, layout)
        _print_published_paths(paths, layout)
        return published



def run_task(prefix, config, output_dir, expected_job, force=False, **overrides):
    """Solve privately, then publish one named delivery atomically per file."""
    return run_delivery(prefix, config, output_dir, expected_job, force, overrides)
