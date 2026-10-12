"""Offline row-timing screening and formal-run evidence comparison."""

from __future__ import annotations

from collections import defaultdict
import csv
import gzip
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.optimize import least_squares
import yaml


cv2.setNumThreads(1)


def read_archive(path: Path) -> list[dict[str, Any]]:
    """Read the project CSV.gz format, whose cells are individually JSON encoded."""
    with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
        return [
            {key: json.loads(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def rms(vectors: np.ndarray) -> float:
    values = np.asarray(vectors, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) == 0:
        raise ValueError("RMS requires a nonempty N x 2 array")
    return float(np.sqrt(np.mean(np.sum(values * values, axis=1))))


def _camera_family(camera: dict[str, Any]) -> str:
    model = camera.get("distortion_model")
    if camera.get("camera_model") != "pinhole":
        raise ValueError("diagnosis supports pinhole-equi and pinhole-radtan only")
    if model == "equidistant":
        return "pinhole-equi"
    if model == "radtan":
        return "pinhole-radtan"
    raise ValueError("diagnosis supports pinhole-equi and pinhole-radtan only")


def project(pose: np.ndarray, xyz: np.ndarray, camera: dict[str, Any]) -> np.ndarray:
    fx, fy, cx, cy = map(float, camera["intrinsics"][:4])
    matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    points = np.ascontiguousarray(np.asarray(xyz, dtype=float).reshape(-1, 1, 3))
    distortion = np.asarray(camera["distortion_coeffs"], dtype=float)
    if _camera_family(camera) == "pinhole-equi":
        projected = cv2.fisheye.projectPoints(
            points, pose[:3], pose[3:], matrix, distortion
        )[0]
    else:
        projected = cv2.projectPoints(
            points, pose[:3], pose[3:], matrix, distortion
        )[0]
    return projected.reshape(-1, 2)


def _pose_from_transform(transform: Any) -> np.ndarray:
    value = np.asarray(transform, dtype=float)
    if value.shape != (4, 4):
        raise ValueError("archived T_camera_target must be a 4 x 4 matrix")
    return np.r_[cv2.Rodrigues(value[:3, :3])[0].ravel(), value[:3, 3]]


def _remove_pose_directions(basis: np.ndarray, jacobian: np.ndarray) -> np.ndarray:
    flat = basis.ravel()
    return (flat - jacobian @ np.linalg.lstsq(jacobian, flat, rcond=None)[0]).reshape(-1, 2)


def _stack(frames: list[dict[str, Any]], key: str) -> np.ndarray:
    return np.concatenate([np.asarray(frame[key]) for frame in frames])


def fit_archived_frames(
    result_path: Path, observations: Path
) -> tuple[dict[str, dict[str, Any]], dict[str, list[dict[str, Any]]], list[dict[str, Any]]]:
    """Fit one target pose per used frame while holding final K/D fixed."""
    result = yaml.safe_load(result_path.read_text(encoding="utf-8"))
    if not isinstance(result, dict) or result.get("kind") != "calibration_result":
        raise ValueError(f"not a calibration result: {result_path}")
    cameras = {str(camera["id"]): camera for camera in result.get("cameras", [])}
    if not cameras:
        raise ValueError("calibration result contains no cameras")
    for camera in cameras.values():
        _camera_family(camera)

    grouped: dict[Any, list[dict[str, Any]]] = defaultdict(list)
    for corner in read_archive(observations / "corners.csv.gz"):
        if corner.get("used") and corner.get("measurement_px") is not None:
            grouped[corner["frame_id"]].append(corner)

    fitted: dict[str, list[dict[str, Any]]] = defaultdict(list)
    rejected: list[dict[str, Any]] = []
    for frame in read_archive(observations / "frames.csv.gz"):
        corners = grouped.get(frame["frame_id"], [])
        camera_id = str(frame["camera_id"])
        if not frame.get("used") or camera_id not in cameras or len(corners) < 6:
            rejected.append({
                "camera_id": camera_id,
                "frame_id": frame["frame_id"],
                "reason": "not_used_or_fewer_than_6_corners",
                "corner_count": len(corners),
            })
            continue
        xyz = np.asarray([item["target_xyz_m"] for item in corners], dtype=float)
        uv = np.asarray([item["measurement_px"] for item in corners], dtype=float)
        transform = frame.get("T_camera_target")
        if transform is None:
            rejected.append({
                "camera_id": camera_id,
                "frame_id": frame["frame_id"],
                "reason": "missing_T_camera_target",
                "corner_count": len(corners),
            })
            continue
        x0 = _pose_from_transform(transform)
        camera = cameras[camera_id]
        fit = least_squares(
            lambda value: (project(value, xyz, camera) - uv).ravel(),
            x0,
            xtol=1e-11,
            ftol=1e-11,
            gtol=1e-10,
            max_nfev=80,
            x_scale="jac",
        )
        if fit.status <= 0 or not np.all(np.isfinite(fit.x)):
            rejected.append({
                "camera_id": camera_id,
                "frame_id": frame["frame_id"],
                "reason": "pose_fit_failed",
                "corner_count": len(corners),
            })
            continue
        residual = uv - project(fit.x, xyz, camera)
        timestamp_ns = int(frame.get("source_timestamp_ns") or frame["observation_timestamp_ns"])
        fitted[camera_id].append({
            "camera_id": camera_id,
            "frame_id": frame["frame_id"],
            "source_index": int(frame["source_index"]),
            "timestamp_ns": timestamp_ns,
            "time_s": timestamp_ns * 1e-9,
            "xyz": xyz,
            "uv": uv,
            "residual": residual,
            "pose": fit.x,
            "jacobian": fit.jac,
            "corner_count": len(corners),
            "pose_rms_px": rms(residual),
        })
    for frames in fitted.values():
        frames.sort(key=lambda item: item["timestamp_ns"])
    return cameras, dict(fitted), rejected


def _add_velocity_basis(
    frames: list[dict[str, Any]], camera: dict[str, Any], *, exclude_edges: bool
) -> list[dict[str, Any]]:
    if len(frames) < 3:
        return []
    width, height = map(float, camera["resolution"])
    result = []
    for index, frame in enumerate(frames):
        if exclude_edges and index in (0, len(frames) - 1):
            continue
        previous = frames[max(0, index - 1)]
        following = frames[min(len(frames) - 1, index + 1)]
        delta_s = following["time_s"] - previous["time_s"]
        if delta_s <= 0:
            raise ValueError("observation timestamps must be strictly increasing")
        velocity = (
            project(following["pose"], frame["xyz"], camera)
            - project(previous["pose"], frame["xyz"], camera)
        ) / delta_s
        row_coordinate = frame["uv"][:, 1] - (height - 1.0) / 2.0
        column_coordinate = frame["uv"][:, 0] - (width - 1.0) / 2.0
        enriched = dict(frame)
        enriched.update({
            "speed_px_s": rms(velocity),
            "row_basis": _remove_pose_directions(
                velocity * row_coordinate[:, None], frame["jacobian"]
            ),
            "column_basis": _remove_pose_directions(
                velocity * column_coordinate[:, None], frame["jacobian"]
            ),
            "velocity_previous_index": previous["source_index"],
            "velocity_following_index": following["source_index"],
        })
        result.append(enriched)
    return result


def _fit_delay(frames: list[dict[str, Any]], basis: str) -> float:
    direction = _stack(frames, basis)
    residual = _stack(frames, "residual")
    information = float(np.sum(direction * direction))
    if not math.isfinite(information) or information <= 0.0:
        raise ValueError("zero timing information in selected frames")
    return float(np.sum(direction * residual) / information)


def _frame_losses(
    frames: list[dict[str, Any]], basis: str, delay_s: float
) -> list[tuple[int, float, float]]:
    losses = []
    for frame in frames:
        before = np.asarray(frame["residual"], dtype=float)
        after = before - np.asarray(frame[basis], dtype=float) * delay_s
        losses.append((len(before), float(np.sum(before * before)), float(np.sum(after * after))))
    return losses


def _loss_metrics(losses: list[tuple[int, float, float]]) -> dict[str, float]:
    points = sum(item[0] for item in losses)
    before = math.sqrt(sum(item[1] for item in losses) / points)
    after = math.sqrt(sum(item[2] for item in losses) / points)
    return {
        "before_rms_px": before,
        "after_rms_px": after,
        "absolute_improvement_px": before - after,
        "relative_reduction": (before - after) / before if before else 0.0,
    }


def _bootstrap_improvement(
    losses: list[tuple[int, float, float]], samples: int, rng: np.random.Generator
) -> dict[str, float]:
    values = []
    for _ in range(samples):
        indices = rng.integers(0, len(losses), len(losses))
        sampled = [losses[int(index)] for index in indices]
        values.append(_loss_metrics(sampled)["absolute_improvement_px"])
    lower, median, upper = np.percentile(values, [2.5, 50.0, 97.5])
    return {"lower_px": float(lower), "median_px": float(median), "upper_px": float(upper)}


def _held_out_direction(
    train: list[dict[str, Any]],
    test: list[dict[str, Any]],
    basis: str,
    bootstrap_samples: int,
    rng: np.random.Generator,
) -> dict[str, Any]:
    delay = _fit_delay(train, basis)
    losses = _frame_losses(test, basis, delay)
    return {
        "fitted_delay_s_per_pixel_axis": delay,
        "train_frames": len(train),
        "test_frames": len(test),
        "test_corners": sum(item[0] for item in losses),
        **_loss_metrics(losses),
        "bootstrap_95_percent": _bootstrap_improvement(losses, bootstrap_samples, rng),
    }


def _screen_camera(
    frames: list[dict[str, Any]],
    camera: dict[str, Any],
    bound_s: float,
    bootstrap_samples: int,
    rng: np.random.Generator,
) -> dict[str, Any]:
    split = len(frames) // 2
    first = _add_velocity_basis(frames[:split], camera, exclude_edges=True)
    second = _add_velocity_basis(frames[split:], camera, exclude_edges=True)
    summary: dict[str, Any] = {
        "input_used_frames": len(frames),
        "first_block_interior_frames": len(first),
        "second_block_interior_frames": len(second),
        "minimum_required": {"total_frames": 20, "interior_frames_per_block": 8},
        "max_abs_line_delay_s": bound_s,
        "median_frame_frequency_hz": (
            float(1.0 / np.median(np.diff([item["time_s"] for item in frames])))
            if len(frames) >= 2 else None
        ),
    }
    if len(frames) < 20 or len(first) < 8 or len(second) < 8:
        summary.update({"status": "insufficient_data", "supportive": False})
        return summary

    forward_row = _held_out_direction(first, second, "row_basis", bootstrap_samples, rng)
    reverse_row = _held_out_direction(second, first, "row_basis", bootstrap_samples, rng)
    forward_column = _held_out_direction(first, second, "column_basis", bootstrap_samples, rng)
    reverse_column = _held_out_direction(second, first, "column_basis", bootstrap_samples, rng)
    row = [forward_row, reverse_row]
    column = [forward_column, reverse_column]
    row_reduction = min(item["relative_reduction"] for item in row)
    column_reduction = max(0.0, max(item["relative_reduction"] for item in column))
    delays = [item["fitted_delay_s_per_pixel_axis"] for item in row]
    candidate = float(np.median(delays))
    gates = {
        "row_reduction_each_direction_at_least_10_percent": row_reduction >= 0.10,
        "bootstrap_lower_bound_each_direction_positive": all(
            item["bootstrap_95_percent"]["lower_px"] > 0.0 for item in row
        ),
        "row_improvement_at_least_twice_column": row_reduction >= 2.0 * column_reduction,
        "blocked_delay_same_sign": delays[0] * delays[1] > 0.0,
        "candidate_below_95_percent_of_bound": abs(candidate) < 0.95 * bound_s,
    }
    supportive = all(gates.values())
    no_row_specific_benefit = (
        row_reduction <= 0.0
        or (
            row_reduction < 0.10
            and not gates["bootstrap_lower_bound_each_direction_positive"]
            and row_reduction <= column_reduction
        )
    )
    summary.update({
        "status": "supportive" if supportive else ("unsupported" if no_row_specific_benefit else "inconclusive"),
        "supportive": supportive,
        "candidate_line_delay_s": candidate,
        "candidate_line_delay_us": candidate * 1e6,
        "first_to_last_row_span_ms": candidate * (int(camera["resolution"][1]) - 1) * 1e3,
        "blocked_forward": {"row": forward_row, "column": forward_column},
        "blocked_reverse": {"row": reverse_row, "column": reverse_column},
        "gates": gates,
    })
    return summary


def screen_observations(
    result_path: Path,
    observations: Path,
    bounds: dict[str, float],
    capture_motion: str,
    *,
    bootstrap_samples: int = 1000,
    bootstrap_seed: int = 0,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    cameras, frames, rejected = fit_archived_frames(result_path, observations)
    rng = np.random.default_rng(bootstrap_seed)
    summaries = {}
    for camera_id, camera in cameras.items():
        summaries[camera_id] = _screen_camera(
            frames.get(camera_id, []), camera, bounds[camera_id], bootstrap_samples, rng
        )
    supportive = all(item.get("supportive") for item in summaries.values())
    statuses = {item["status"] for item in summaries.values()}
    if supportive and capture_motion == "continuous":
        classification = "screen_supportive"
        run_formal = True
    elif statuses == {"unsupported"} and capture_motion == "continuous":
        classification = "unsupported"
        run_formal = False
    else:
        classification = "inconclusive"
        run_formal = False
    return ({
        "schema_version": "1.0.0",
        "kind": "rolling_shutter_offline_screen",
        "classification": classification,
        "run_formal_controls": run_formal,
        "capture_motion": capture_motion,
        "camera_results": summaries,
        "bootstrap": {"samples": bootstrap_samples, "seed": bootstrap_seed, "unit": "frame"},
        "metric_definition": "2D corner RMS = sqrt(sum(dx^2+dy^2)/corner_count)",
        "limits": [
            "The fitted delay is a finite-difference screening estimate, not a hardware measurement.",
            "Only a continuous-motion capture can reach the automatic important classification.",
        ],
    }, rejected)


def archived_residuals(observations: Path) -> dict[str, dict[tuple[int, int], np.ndarray]]:
    frames = {row["frame_id"]: row for row in read_archive(observations / "frames.csv.gz")}
    values: dict[str, dict[tuple[int, int], np.ndarray]] = defaultdict(dict)
    for corner in read_archive(observations / "corners.csv.gz"):
        frame = frames.get(corner["frame_id"])
        if not frame or not frame.get("used") or not corner.get("used") or corner.get("residual_px") is None:
            continue
        key = (int(frame["source_timestamp_ns"]), int(corner["corner_id"]))
        values[str(frame["camera_id"])][key] = np.asarray(corner["residual_px"], dtype=float)
    return dict(values)


def compare_archived_residuals(
    baseline_observations: Path,
    candidate_observations: Path,
    *,
    bootstrap_samples: int = 1000,
    bootstrap_seed: int = 0,
) -> dict[str, Any]:
    """Compare two formal runs on exactly their common camera/timestamp/corner keys."""
    baseline = archived_residuals(baseline_observations)
    candidate = archived_residuals(candidate_observations)
    rng = np.random.default_rng(bootstrap_seed)
    result = {}
    for camera_id in sorted(set(baseline) & set(candidate)):
        keys = sorted(set(baseline[camera_id]) & set(candidate[camera_id]))
        by_frame: dict[int, list[tuple[np.ndarray, np.ndarray]]] = defaultdict(list)
        for timestamp, corner_id in keys:
            by_frame[timestamp].append((baseline[camera_id][(timestamp, corner_id)], candidate[camera_id][(timestamp, corner_id)]))
        losses = []
        for pairs in by_frame.values():
            before = np.asarray([pair[0] for pair in pairs])
            after = np.asarray([pair[1] for pair in pairs])
            losses.append((len(pairs), float(np.sum(before * before)), float(np.sum(after * after))))
        if not losses:
            continue
        result[camera_id] = {
            "common_frames": len(losses),
            "common_corners": sum(item[0] for item in losses),
            **_loss_metrics(losses),
            "bootstrap_95_percent": _bootstrap_improvement(losses, bootstrap_samples, rng),
        }
    return result


def load_run_summary(run_dir: Path, output_name: str) -> dict[str, Any]:
    result_path = run_dir / f"{output_name}.yaml"
    artifact_dir = run_dir / output_name
    manifest_path = artifact_dir / "run_manifest.json"
    summary: dict[str, Any] = {
        "output_name": output_name,
        "result_path": str(result_path),
        "artifact_dir": str(artifact_dir),
        "completed": result_path.is_file(),
    }
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        summary["run_status"] = manifest.get("status")
        summary["completed"] = summary["completed"] and manifest.get("status") == "completed"
    if not result_path.is_file():
        return summary
    result = yaml.safe_load(result_path.read_text(encoding="utf-8"))
    summary["cameras"] = {
        str(camera["id"]): {
            "line_delay_s": (camera.get("shutter") or {}).get("line_delay_s"),
            "line_delay_std_s": (camera.get("shutter") or {}).get("line_delay_std_s"),
            "max_abs_line_delay_s": (camera.get("shutter") or {}).get("max_abs_line_delay_s"),
            "rms_px": camera.get("rms"),
            "alignment_rms_px": camera.get("alignment"),
            "rs_compensated_pair_residual_rms_px": camera.get("rs_compensated_pair_residual"),
        }
        for camera in result.get("cameras", [])
    }
    observability_path = artifact_dir / "observability.yaml"
    if observability_path.is_file():
        observability = yaml.safe_load(observability_path.read_text(encoding="utf-8")) or {}
        weak_line_delay = 0.0
        for mode in observability.get("operationally_truncated_modes", []):
            contribution = sum(
                float(block.get("contribution", 0.0))
                for block in mode.get("dominant_blocks", [])
                if "line_delay" in str(block.get("block", ""))
            )
            weak_line_delay = max(weak_line_delay, contribution)
        summary["observability"] = {
            "status": observability.get("status"),
            "quality": observability.get("quality"),
            "maximum_line_delay_contribution_in_weak_mode": weak_line_delay,
        }
    metrics_path = artifact_dir / "metrics.json"
    if metrics_path.is_file():
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        summary["optimizer"] = metrics.get("optimizer")
        summary["imu_rms"] = {
            imu_id: {
                "accel": (value.get("accel") or {}).get("rms"),
                "gyro": (value.get("gyro") or {}).get("rms"),
            }
            for imu_id, value in (metrics.get("imus") or {}).items()
        }
    return summary
