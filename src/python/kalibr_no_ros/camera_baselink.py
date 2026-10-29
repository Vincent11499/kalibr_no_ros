"""Camera-to-base_link rotation calibration from fixed-attitude platform motion."""

from __future__ import annotations

import csv
from copy import deepcopy
from pathlib import Path
import math
import re
import tempfile

import numpy as np

from .jobs import CAMERA_BASELINK_JOB


class CameraBaselinkError(RuntimeError):
    pass


_POSITION_STEM = re.compile(
    r"(?P<x>[+-]?(?:\d+(?:\.\d*)?|\.\d+))_"
    r"(?P<y>[+-]?(?:\d+(?:\.\d*)?|\.\d+))_"
    r"(?P<z>[+-]?(?:\d+(?:\.\d*)?|\.\d+))")

def _all_pair_indices(count):
    return np.triu_indices(count, k=1)


def platform_position(path, scale_m):
    """Parse ``<x>_<y>_<z>.<ext>`` and return a base-frame position in metres."""
    match = _POSITION_STEM.fullmatch(Path(path).stem)
    if match is None:
        raise CameraBaselinkError(
            "base_link image filename must be <x>_<y>_<z>.<ext>: {}"
            .format(Path(path).name))
    position = np.asarray([
        float(match.group("x")),
        float(match.group("y")),
        float(match.group("z")),
    ]) * float(scale_m)
    if not np.all(np.isfinite(position)):
        raise CameraBaselinkError(
            "base_link image filename contains a non-finite position: {}"
            .format(Path(path).name))
    return position


def _rank_diagnostics(positions, rank_ratio_min):
    positions = np.asarray(positions, dtype=float)
    if positions.ndim != 2 or positions.shape[1:] != (3,) or len(positions) < 3:
        raise CameraBaselinkError(
            "base_link calibration requires at least three platform positions")
    first, second = _all_pair_indices(len(positions))
    differences = positions[first] - positions[second]
    nonzero = np.linalg.norm(differences, axis=1) >= 1e-6
    if int(np.sum(nonzero)) < 2:
        raise CameraBaselinkError(
            "base_link calibration requires at least two nonzero movements")
    singular = np.linalg.svd(differences[nonzero].T, compute_uv=False)
    ratio = (
        float(singular[1] / singular[0])
        if len(singular) >= 2 and singular[0] >= 1e-12 else 0.0)
    if ratio < rank_ratio_min:
        raise CameraBaselinkError(
            "rank-1 platform motion / base_link rotation unobservable")
    return singular, ratio


def _optical_axis(rotation):
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise CameraBaselinkError("camera/base_link rotation must be a finite 3x3 matrix")
    axis = rotation @ np.asarray([0.0, 0.0, 1.0])
    norm = float(np.linalg.norm(axis))
    if not math.isfinite(norm) or norm < 1e-12:
        raise CameraBaselinkError("camera optical axis is invalid")
    return axis / norm


def optical_axis_pitch_deg(rotation):
    """Unsigned camera +Z optical-axis angle above the base horizontal plane."""
    axis = _optical_axis(rotation)
    return math.degrees(math.asin(np.clip(abs(axis[2]), 0.0, 1.0)))


def _correct_output_rotation(rotation, correction):
    """Left-multiply a fixed base-frame correction onto a solved rotation."""
    if (not isinstance(correction, list) or len(correction) != 3
            or any(not isinstance(row, list) or len(row) != 3
                   for row in correction)
            or any(type(value) not in (int, float)
                   or not math.isfinite(value)
                   for row in correction for value in row)):
        raise CameraBaselinkError(
            "base_link.rotation_left_correction must be a finite 3x3 matrix")
    correction = np.asarray(correction, dtype=float)
    if (not np.allclose(correction.T @ correction, np.eye(3),
                        atol=1e-6, rtol=0.0)
            or not math.isclose(float(np.linalg.det(correction)), 1.0,
                                abs_tol=1e-6)):
        raise CameraBaselinkError(
            "base_link.rotation_left_correction must belong to SO(3)")
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise CameraBaselinkError(
            "solved camera/base_link rotation must be a finite 3x3 matrix")
    return correction @ rotation


def relative_rotation_angles_rad(camera_rotations):
    """Return ``||log(R[j] @ R[i].T)||`` for every unordered PnP pair."""
    rotations = np.asarray(camera_rotations, dtype=float)
    if (rotations.ndim != 3 or rotations.shape[1:] != (3, 3)
            or len(rotations) < 2 or not np.all(np.isfinite(rotations))):
        raise CameraBaselinkError(
            "camera rotations must have shape (N,3,3), N>=2")
    orthogonality = np.linalg.norm(
        np.swapaxes(rotations, 1, 2) @ rotations - np.eye(3),
        axis=(1, 2))
    determinants = np.linalg.det(rotations)
    if (float(np.max(orthogonality)) > 1e-6
            or float(np.max(np.abs(determinants - 1.0))) > 1e-6):
        raise CameraBaselinkError("camera rotations must belong to SO(3)")
    first, second = _all_pair_indices(len(rotations))
    relative = rotations[second] @ np.swapaxes(rotations[first], 1, 2)
    cosine = (np.trace(relative, axis1=1, axis2=2) - 1.0) * 0.5
    return np.arccos(np.clip(cosine, -1.0, 1.0))


def _relative_rotation_pairs(frame_names, camera_rotations):
    angles = relative_rotation_angles_rad(camera_rotations)
    if len(frame_names) != len(camera_rotations):
        raise CameraBaselinkError(
            "frame names and camera rotations have different lengths")
    first, second = _all_pair_indices(len(frame_names))
    return [{
        "pair_index": index,
        "from_frame": str(frame_names[first[index]]),
        "to_frame": str(frame_names[second[index]]),
        "theta_rad": float(angle),
        "theta_deg": math.degrees(float(angle)),
    } for index, angle in enumerate(angles)]


def solve_direct_rotation(platform_positions_m, camera_target_translations_m,
                          rank_ratio_min=1e-4):
    """Solve ``R_base_camera`` using every direct translation-difference pair."""
    platform = np.asarray(platform_positions_m, dtype=float)
    camera = np.asarray(camera_target_translations_m, dtype=float)
    if (platform.ndim != 2 or platform.shape[1:] != (3,)
            or camera.shape != platform.shape or len(platform) < 3
            or not np.all(np.isfinite(platform))
            or not np.all(np.isfinite(camera))):
        raise CameraBaselinkError(
            "platform positions and PnP translations must have shape (N,3), N>=3")
    if (isinstance(rank_ratio_min, bool)
            or not isinstance(rank_ratio_min, (int, float))
            or not math.isfinite(rank_ratio_min)
            or not 0.0 < rank_ratio_min < 1.0):
        raise CameraBaselinkError("rank_ratio_min must be in (0,1)")

    first, second = _all_pair_indices(len(platform))
    a_full = platform[first] - platform[second]
    b_full = camera[second] - camera[first]
    kept = ((np.linalg.norm(a_full, axis=1) >= 1e-6)
            & (np.linalg.norm(b_full, axis=1) >= 1e-6))
    if int(np.sum(kept)) < 2:
        raise CameraBaselinkError(
            "insufficient nonzero platform/PnP movement pairs")

    a = a_full[kept].T
    b = b_full[kept].T
    singular_a = np.linalg.svd(a, compute_uv=False)
    singular_b = np.linalg.svd(b, compute_uv=False)
    for singular, label in ((singular_a, "platform"), (singular_b, "camera")):
        if (len(singular) < 2 or singular[0] < 1e-12
                or singular[1] / singular[0] < rank_ratio_min):
            raise CameraBaselinkError(
                "rank-1 {} motion / base_link rotation unobservable".format(label))

    left, singular_cross, right_transposed = np.linalg.svd(a @ b.T)
    reflected = np.linalg.det(left @ right_transposed) < 0.0
    rotation = (
        left
        @ np.diag([1.0, 1.0, np.linalg.det(left @ right_transposed)])
        @ right_transposed)
    residual = a_full[kept] - (rotation @ b_full[kept].T).T
    return rotation, {
        "effective_pairs": int(np.sum(kept)),
        "pair_indices": np.flatnonzero(kept).tolist(),
        "platform_singular_values": singular_a.tolist(),
        "camera_singular_values": singular_b.tolist(),
        "cross_singular_values": singular_cross.tolist(),
        "platform_second_first_ratio": float(singular_a[1] / singular_a[0]),
        "camera_second_first_ratio": float(singular_b[1] / singular_b[0]),
        "reflection_corrected": bool(reflected),
        "movement_residual_mm": float(
            1000.0 * np.sqrt(np.mean(np.sum(residual * residual, axis=1)))),
        "pitch": optical_axis_pitch_deg(rotation),
    }


def _write_rotation_summary(artifacts, path):
    rows = []
    for camera in artifacts["cameras"]:
        pairs = camera.get("relative_rotation_pairs") or []
        if not pairs:
            raise CameraBaselinkError(
                "relative rotation summary requires PnP frame pairs")
        degrees = np.asarray([pair["theta_deg"] for pair in pairs], dtype=float)
        statistics = [
            len(pairs),
            "{:.17g}".format(float(np.min(degrees))),
            "{:.17g}".format(float(np.median(degrees))),
            "{:.17g}".format(float(np.mean(degrees))),
            "{:.17g}".format(float(np.percentile(degrees, 95))),
            "{:.17g}".format(float(np.max(degrees))),
        ]
        rows.extend([
            camera["id"], pair["pair_index"], pair["from_frame"],
            pair["to_frame"], "{:.17g}".format(pair["theta_rad"]),
            "{:.17g}".format(pair["theta_deg"]), *statistics,
        ] for pair in pairs)
    with Path(path).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "camera", "pair_index", "from_frame", "to_frame", "theta_rad",
            "theta_deg", "pair_count", "theta_deg_min", "theta_deg_median",
            "theta_deg_mean", "theta_deg_p95", "theta_deg_max",
        ])
        writer.writerows(rows)


def _selected_entries(task, camera, reader):
    with reader.index_images(camera["topic"]) as dataset:
        entries = reader._crop(
            dataset.index, task["dataset"].get("time_range_s"))
        entries = reader._decimate(
            entries, task["dataset"].get("frequency_hz"))
        return list(entries)


def validate_platform_positions(task, cameras, reader):
    """Validate filename positions without importing the native detector."""
    scale = float(task["base_link"]["position_scale_m"])
    ratio_min = float((task.get("calibration") or {}).get(
        "rank_ratio_min", 1e-4))
    summaries = []
    for camera in cameras:
        positions = []
        for entry in _selected_entries(task, camera, reader):
            path = getattr(entry, "path", None)
            if path is None:
                raise CameraBaselinkError(
                    "base_link calibration requires filename-backed directory images")
            positions.append(platform_position(path, scale))
        singular, ratio = _rank_diagnostics(positions, ratio_min)
        summaries.append({
            "id": camera["id"],
            "positions": len(positions),
            "platform_singular_values": singular.tolist(),
            "platform_second_first_ratio": ratio,
        })
    return {
        "frame_id": task["base_link"].get("frame_id", "base_link"),
        "position_source": "image_filename_stem_x_y_z",
        "position_scale_m": scale,
        "cameras": summaries,
    }


def _observation_transform(observation):
    transform = observation.T_t_c()
    matrix = np.asarray(transform.T(), dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise CameraBaselinkError("native PnP returned an invalid transformation")
    return np.linalg.inv(matrix)


def _corner_records(observation, geometry):
    measured = np.asarray(observation.getCornersImageFrame(), dtype=float)
    predicted = np.asarray(
        observation.getCornerReprojection(geometry), dtype=float)
    identifiers = np.asarray(observation.getCornersIdx(), dtype=int).reshape(-1)
    if (measured.ndim != 2 or measured.shape[1:] != (2,)
            or predicted.shape != measured.shape
            or len(identifiers) != len(measured)):
        raise CameraBaselinkError(
            "native detector returned inconsistent target corners")
    records = []
    for identifier, measurement, prediction in zip(
            identifiers, measured, predicted):
        records.append({
            "corner_id": int(identifier),
            "target_xyz_m": np.asarray(
                observation.target().point(int(identifier)), dtype=float
            ).reshape(-1).tolist(),
            "measurement_px": measurement.tolist(),
            "prediction_px": prediction.tolist(),
            "residual_px": (measurement - prediction).tolist(),
            "used": True,
        })
    return records


def _public_source_camera(camera):
    fields = (
        "id", "camera_model", "distortion_model", "intrinsics",
        "distortion_coeffs", "resolution", "rostopic")
    return {key: deepcopy(camera[key]) for key in fields if key in camera}


def _solve(task, resolved_task, target_path):
    from kalibr_common import extractCornersFromDataset

    from .artifacts import run_context
    from .datasets import BagImageDatasetReader
    from .evaluation import camera_geometry
    from .fixed_camera_validation import _detector
    from .validation import load_cameras

    source_cameras = load_cameras(task)
    configured = {camera["id"]: camera for camera in task["cameras"]}
    dataset = resolved_task["dataset"]
    calibration = task.get("calibration") or {}
    scale = float(task["base_link"]["position_scale_m"])
    ratio_min = float(calibration.get("rank_ratio_min", 1e-4))
    result_cameras = []

    with run_context(CAMERA_BASELINK_JOB,
                     capture_history=task["output"]["archive_selection_history"]) as context:
        for source in source_cameras:
            geometry, detector = _detector(
                target_path, source,
                calibration.get("window_half_size_px", 2),
                calibration.get("max_displacement_px", math.sqrt(1.5)),
                show_extraction=task["output"].get("show_extraction", False),
                extraction_stepping=task["output"].get(
                    "extraction_stepping", False))
            image_dataset = BagImageDatasetReader(
                dataset["path"], source["topic"],
                bag_from_to=dataset.get("time_range_s"),
                bag_freq=dataset.get("frequency_hz"))
            try:
                observations = extractCornersFromDataset(
                    image_dataset, detector,
                    multithreading=not task["output"].get("show_extraction"),
                    clearImages=False)
            finally:
                image_dataset.close()
            if len(observations) < 3:
                raise CameraBaselinkError(
                    "{} requires at least three successful PnP observations"
                    .format(source["id"]))

            artifact = context.artifacts["cameras"][-1]
            collected_id = artifact["id"]
            artifact.update(_public_source_camera(source))
            if collected_id != source["id"]:
                prefix = collected_id + ":"
                for frame in artifact["frames"]:
                    if frame["frame_id"].startswith(prefix):
                        frame["frame_id"] = (
                            source["id"] + frame["frame_id"][len(collected_id):])
            artifact["model"] = camera_geometry(source)["model"]
            positions, translations, rotations, frame_names = [], [], [], []
            for observation in observations:
                frame = context.frame(observation)
                source_path = frame.get("source_relative_path") or frame.get(
                    "source_path")
                if not source_path:
                    raise CameraBaselinkError(
                        "base_link observation has no source filename")
                transform = _observation_transform(observation)
                positions.append(platform_position(source_path, scale))
                translations.append(transform[:3, 3])
                rotations.append(transform[:3, :3])
                frame_names.append(Path(source_path).name)
                frame.update({
                    "used": True,
                    "T_camera_target": transform.tolist(),
                    "corners": _corner_records(observation, geometry),
                })

            rotation, diagnostics = solve_direct_rotation(
                positions, translations, rank_ratio_min=ratio_min)
            correction = task["base_link"].get(
                "rotation_left_correction", np.eye(3).tolist())
            rotation = _correct_output_rotation(rotation, correction)
            diagnostics["pitch"] = optical_axis_pitch_deg(rotation)
            diagnostics["rotation_left_correction"] = np.asarray(
                correction, dtype=float).tolist()
            artifact["relative_rotation_pairs"] = _relative_rotation_pairs(
                frame_names, rotations)
            translation = np.asarray(
                configured[source["id"]]["translation_base_camera_m"],
                dtype=float)
            transform = np.eye(4)
            transform[:3, :3] = rotation
            transform[:3, 3] = translation
            artifact.update(diagnostics)
            artifact["T_base_camera"] = transform.tolist()
            result_camera = _public_source_camera(source)
            result_camera.update({
                "T_base_camera": transform.tolist(),
                "translation_source": "task_configuration",
                "rotation_method": "direct_all_pairs_translation_difference",
                **diagnostics,
            })
            result_cameras.append(result_camera)

        context.artifacts.update({
            "state": "completed",
            "dataset": dict(dataset),
            "base_frame": task["base_link"].get("frame_id", "base_link"),
            "position_source": "image_filename_stem_x_y_z",
            "position_scale_m": scale,
        })
        artifacts = context.artifacts

    result = {
        "schema_version": "1.0.0",
        "kind": "calibration_result",
        "calibration_type": "cameras_baselink",
        "base_frame": task["base_link"].get("frame_id", "base_link"),
        "position_source": {
            "type": "image_filename_stem_x_y_z",
            "scale_m": scale,
        },
        "cameras": result_cameras,
    }
    return result, artifacts


def _resolved_task(task, dataset_path, overrides):
    from .task import resolve_execution, resolve_task_path

    resolved = deepcopy({
        key: value for key, value in task.items() if not key.startswith("_")
    })
    resolved["dataset"] = dict(task["dataset"], path=str(dataset_path))
    for key in ("target", "camera_calibration"):
        block = resolved[key]
        value = block.get("path") if isinstance(block, dict) else block
        resolved[key] = {"path": str(resolve_task_path(task, value))}
    resolved["base_link"].setdefault("frame_id", "base_link")
    resolved["base_link"].setdefault(
        "rotation_left_correction", np.eye(3).tolist())
    resolved["execution"] = resolve_execution(task.get("execution"), overrides)
    return resolved


def _run_staged(config, stage, task, overrides):
    import json
    import kalibr_runtime as native_runtime

    from .delivery import enrich_result
    from .reporting import generate_report
    from .task import (
        _capture_solver_logs,
        _staged_dataset_path,
        _target_path,
        dump_yaml,
        prepare_output_directory,
        write_run_manifest,
    )
    from .validation import validate_task

    dataset_path = _staged_dataset_path(task, config, stage, None)
    output = prepare_output_directory(stage)
    resolved = _resolved_task(task, dataset_path, overrides)
    dump_yaml(resolved, output / "task_resolved.yaml")
    write_run_manifest(output, status="running", task=task)
    validation = validate_task(output / "task_resolved.yaml")
    (output / "validation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2, allow_nan=False)
        + "\n", encoding="utf-8")
    if validation["status"] != "passed":
        error = CameraBaselinkError(
            "input validation failed: {}".format(
                "; ".join(validation["errors"])))
        write_run_manifest(
            output, status="input_failed", task=task, failure=error)
        raise error

    runtime = dict(resolved["execution"])
    runtime["timing_json"] = overrides.get("timing_json")
    native_runtime.configure(
        runtime, command="kalibr-noros calibrate cameras-baselink")
    with tempfile.TemporaryDirectory(prefix="kalibr-baselink-") as folder:
        target_path = _target_path(task, Path(folder))
        try:
            with _capture_solver_logs(
                    output, verbose=bool(task["output"].get("verbose"))):
                result, artifacts = _solve(task, resolved, target_path)
            native_runtime.finish("ok")
            # Preserve a finite solve if report generation fails afterwards.
            _write_rotation_summary(artifacts, output / "rotation_summary.csv")
            dump_yaml(result, output / "calibration.yaml")
            report = generate_report(
                artifacts, result, output, task["output"],
                dataset=str(dataset_path))
            enrich_result(result, report["metrics"])
            dump_yaml(result, output / "calibration.yaml")
            write_run_manifest(output, status="completed", task=task)
        except Exception as error:
            native_runtime.finish("error")
            write_run_manifest(
                output, status="failed", task=task, failure=error)
            raise
    return output


def run_camera_baselink(config, output_dir, force=False, **overrides):
    """Run privately and publish one named camera/base_link delivery."""
    from .delivery import (
        FLAT_LAYOUT, _check_owned, _delivery_paths, _validate_layout,
        publish, result_name,
    )
    from .task import TaskError, load_task, resolve_task_path

    task = load_task(config, expected_job=CAMERA_BASELINK_JOB)
    run_overrides = dict(overrides)
    layout = _validate_layout(
        run_overrides.pop("_delivery_layout", FLAT_LAYOUT))
    output = Path(output_dir).expanduser().resolve()
    dataset = resolve_task_path(task, task["dataset"]["path"])
    if (output in {Path("/").resolve(), Path.home().resolve(), Path.cwd().resolve()}
            or output == dataset or dataset in output.parents
            or output in dataset.parents):
        raise TaskError(
            "output directory must be separate from the input dataset and workspace root")
    name = result_name(task)
    _check_owned(output, name, force, layout)
    inputs = [Path(config).expanduser().resolve()]
    for block in (task["target"], task["camera_calibration"]):
        value = block.get("path") if isinstance(block, dict) else block
        inputs.append(resolve_task_path(task, value))
    paths = _delivery_paths(output, name, layout)
    for source in inputs:
        if (source == paths["result"]
                or output / name in source.parents):
            raise TaskError("delivery would overwrite its input: {}".format(source))

    with tempfile.TemporaryDirectory(prefix="kalibr-baselink-delivery-") as folder:
        stage = Path(folder) / "run"
        if run_overrides.get("timing_json"):
            run_overrides["timing_json"] = str(stage / "timing.json")
        try:
            _run_staged(config, stage, task, run_overrides)
        except Exception:
            if task["output"].get("save_diagnostics") and stage.exists():
                from uuid import uuid4
                failed_name = name + "_failed"
                while ((output / failed_name).exists()
                       or (output / failed_name).is_symlink()):
                    failed_name = name + "_failed_" + uuid4().hex[:12]
                try:
                    publish(
                        stage, output, failed_name, task["output"],
                        failed=True, layout=layout)
                except Exception as diagnostic_error:
                    import sys
                    print(
                        "Could not preserve failure diagnostics: {}".format(
                            diagnostic_error),
                        file=sys.stderr)
            raise
        published = publish(
            stage, output, name, task["output"], force=force,
            layout=layout)
        paths = _delivery_paths(output, name, layout)
        print("[kalibr] result={}".format(paths["result"]))
        print("[kalibr] report_html={}".format(paths["html"]))
        print("[kalibr] report_pdf={}".format(paths["pdf"]))
        return published
