"""Validate fixed stereo calibration parameters on a separate directory dataset.

This module deliberately does not run a calibration optimizer.  It detects the
target on the validation images, estimates an independent target pose for each
camera/frame with fixed intrinsics, and evaluates the saved stereo transform by
rectifying common measured corners.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import json
import math
import os
import re
import shutil
import tempfile
import time

import numpy as np

from .evaluation import assess_metrics, rectified_points, stereo_geometry
from .task import load_yaml, require_document_version
from .validation import load_cameras, load_target
from .version import SCHEMA_VERSION, VERSION

try:
    from ._build_info import GIT_COMMIT, SOURCE_VARIANT
except ImportError:
    GIT_COMMIT = None
    SOURCE_VARIANT = "project"


class FixedCameraValidationError(ValueError):
    pass


_SAFE_LABEL = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}")
_OUTPUT_FILES = {
    "fixed_camera_validation.json",
    "README_ZH.md",
    "run_manifest.json",
}


def parse_calibration_specs(values):
    """Parse repeated ``LABEL=PATH`` or plain path arguments."""
    result = []
    labels = set()
    for value in values or []:
        if not isinstance(value, str) or not value.strip():
            raise FixedCameraValidationError(
                "--calibration must be LABEL=PATH or a non-empty path")
        if "=" in value:
            label, raw_path = value.split("=", 1)
        else:
            raw_path = value
            label = Path(raw_path).stem
        if not _SAFE_LABEL.fullmatch(label):
            raise FixedCameraValidationError(
                "calibration label must contain only letters, digits, underscores, "
                "or hyphens and start with a letter or digit: {}".format(label))
        if label in labels:
            raise FixedCameraValidationError(
                "duplicate calibration label: {}".format(label))
        path = Path(raw_path).expanduser().resolve()
        if not path.is_file() or path.is_symlink():
            raise FixedCameraValidationError(
                "calibration is not a regular file: {}".format(path))
        labels.add(label)
        result.append((label, path))
    if not result:
        raise FixedCameraValidationError(
            "at least one --calibration is required")
    return result


def combined_reprojection_rms(camera_rows):
    squared_error = sum(float(row["squared_error_px2"]) for row in camera_rows)
    corners = sum(int(row["corner_count"]) for row in camera_rows)
    return math.sqrt(squared_error / corners) if corners else None


def _native_model(camera):
    """Construct the native model class used by the original detector."""
    import aslam_cv_backend as acvb

    from .evaluation import camera_geometry

    model = camera_geometry(camera)["model"]
    models = {
        "pinhole-radtan": acvb.DistortedPinhole,
        "pinhole-equi": acvb.EquidistantPinhole,
        "pinhole-equidistant": acvb.EquidistantPinhole,
        "pinhole-fov": acvb.FovPinhole,
        "omni-none": acvb.Omni,
        "omni-radtan": acvb.DistortedOmni,
        "eucm-none": acvb.ExtendedUnified,
        "ds-none": acvb.DoubleSphere,
    }
    if model == "pinhole-radtan5":
        import kalibr_radtan5 as extension
        extension.install()
        return extension.PinholeRadtan5
    if model == "pinhole-radtan8":
        import kalibr_radtan8 as extension
        extension.install()
        return extension.PinholeRadtan8
    if model == "pinhole-opencv-fisheye":
        import kalibr_opencv_fisheye as extension
        extension.install()
        return extension.PinholeOpenCvFisheye
    try:
        return models[model]
    except KeyError as error:
        raise FixedCameraValidationError(
            "fixed-camera validation does not support model {}".format(
                model)) from error


def _native_geometry(camera):
    model = _native_model(camera)
    distortion = model.distortionType()
    distortion.setParameters(np.asarray(
        camera["distortion_coeffs"], dtype=float))
    projection = model.projectionType(*(
        list(map(float, camera["intrinsics"]))
        + list(map(int, camera["resolution"]))
        + [distortion]))
    return model.geometry(projection)


def _detector(target_path, camera, window_half_size_px,
              max_displacement_px):
    from kalibr_common.ConfigReader import CalibrationTargetParameters
    from kalibr_camera_calibration.CameraCalibrator import TargetDetector

    target = CalibrationTargetParameters(str(target_path))
    geometry = _native_geometry(camera)
    detector = TargetDetector(
        target,
        geometry,
        windowHalfSizePx=window_half_size_px,
        maxDisplacementPx=max_displacement_px,
    ).detector
    return geometry, detector


def _detect_camera(dataset_path, target_path, camera, window_half_size_px,
                   max_displacement_px):
    from .datasets import BagImageDatasetReader

    camera_id = camera["id"]
    geometry, detector = _detector(
        target_path, camera, window_half_size_px, max_displacement_px)
    reader = BagImageDatasetReader(str(dataset_path), camera_id)
    frames = []
    total_squared = 0.0
    total_corners = 0
    try:
        for source_index in map(int, reader.indices.copy()):
            stamp, image = reader.getImage(source_index)
            metadata = reader.source_metadata(source_index)
            success, observation = detector.findTarget(stamp, image)
            frame = {
                "frame_id": "{}:{}".format(camera_id, source_index),
                "source_index": source_index,
                "source_timestamp_ns": int(metadata["source_timestamp_ns"]),
                "source_relative_path": metadata.get("source_relative_path"),
                "detected": bool(success),
                "corner_count": 0,
                "rms_px": None,
            }
            if success:
                measured = np.asarray(
                    observation.getCornersImageFrame(), dtype=float)
                predicted = np.asarray(
                    observation.getCornerReprojection(geometry), dtype=float)
                corner_ids = np.asarray(
                    observation.getCornersIdx(), dtype=int)
                if (measured.ndim != 2 or measured.shape[1] != 2
                        or predicted.shape != measured.shape
                        or corner_ids.size != measured.shape[0]):
                    raise FixedCameraValidationError(
                        "native detector returned inconsistent corners for {}"
                        .format(frame["frame_id"]))
                residual = predicted - measured
                squared = float(np.sum(residual * residual))
                count = int(measured.shape[0])
                frame.update(
                    corner_count=count,
                    rms_px=(math.sqrt(squared / count) if count else None),
                )
                frame["_squared_error_px2"] = squared
                frame["_points"] = {
                    int(identifier): point
                    for identifier, point in zip(corner_ids, measured)
                }
                total_squared += squared
                total_corners += count
            frames.append(frame)
    finally:
        reader.close()
    return {
        "input_frames": len(frames),
        "detected_frames": sum(frame["detected"] for frame in frames),
        "corner_count": total_corners,
        "squared_error_px2": total_squared,
        "rms_px": (
            math.sqrt(total_squared / total_corners)
            if total_corners else None),
        "frames": frames,
    }


def _paired_frames(left, right, tolerance_ns):
    candidates = []
    for left_index, left_frame in enumerate(left):
        if not left_frame["detected"]:
            continue
        for right_index, right_frame in enumerate(right):
            if not right_frame["detected"]:
                continue
            delta = abs(
                left_frame["source_timestamp_ns"]
                - right_frame["source_timestamp_ns"])
            if delta <= tolerance_ns:
                candidates.append((
                    delta,
                    left_frame["source_index"],
                    right_frame["source_index"],
                    left_index,
                    right_index,
                ))
    consumed_left, consumed_right = set(), set()
    for delta, _, _, left_index, right_index in sorted(candidates):
        if left_index in consumed_left or right_index in consumed_right:
            continue
        consumed_left.add(left_index)
        consumed_right.add(right_index)
        yield delta, left[left_index], right[right_index]


def _stereo_metrics(cameras, detected, tolerance_s, rectification):
    left, right = cameras
    geometry = stereo_geometry(left, right, rectification)
    width, height = geometry["size"]
    disparity_axis = 0 if geometry["disparity_axis"] == "x" else 1
    errors = []
    pairs = []
    common_count = 0
    valid_count = 0
    outside = 0
    tolerance_ns = round(float(tolerance_s) * 1e9)
    for difference_ns, left_frame, right_frame in _paired_frames(
            detected[left["id"]]["frames"],
            detected[right["id"]]["frames"], tolerance_ns):
        left_points = left_frame["_points"]
        right_points = right_frame["_points"]
        common = sorted(set(left_points) & set(right_points))
        if not common:
            continue
        common_count += len(common)
        lp = rectified_points(
            [left_points[key] for key in common], geometry, "left")
        rp = rectified_points(
            [right_points[key] for key in common], geometry, "right")
        valid = np.all(np.isfinite(lp), axis=1) & np.all(
            np.isfinite(rp), axis=1)
        for points in (lp, rp):
            valid &= (points[:, 0] >= 0) & (points[:, 0] < width)
            valid &= (points[:, 1] >= 0) & (points[:, 1] < height)
        pair_errors = np.abs(
            (lp[valid] - rp[valid])[:, 1 - disparity_axis])
        errors.extend(pair_errors.tolist())
        valid_count += int(np.sum(valid))
        outside += int(np.sum(~valid))
        pairs.append({
            "left_frame_id": left_frame["frame_id"],
            "right_frame_id": right_frame["frame_id"],
            "timestamp_difference_ns": difference_ns,
            "common_corners": len(common),
            "valid_rectified_corners": int(np.sum(valid)),
            "alignment_rms_px": (
                float(np.sqrt(np.mean(pair_errors * pair_errors)))
                if pair_errors.size else None),
        })
    values = np.asarray(errors, dtype=float)
    return {
        "status": "available" if values.size else "unavailable",
        "used_pairs": len(pairs),
        "common_corners": common_count,
        "valid_rectified_corners": valid_count,
        "outside_rectified_domain_corners": outside,
        "baseline_m": geometry["baseline_m"],
        "translation_m": geometry["T"].tolist(),
        "disparity_axis": geometry["disparity_axis"],
        "alignment": {
            "status": "available" if values.size else "unavailable",
            "count": int(values.size),
            "unit": "px",
            "mean_abs_px": float(np.mean(values)) if values.size else None,
            "rms_px": (
                float(np.sqrt(np.mean(values * values)))
                if values.size else None),
        },
        "pairs": pairs,
    }


def _public_camera(camera):
    frames = []
    for frame in camera["frames"]:
        frames.append({
            key: value for key, value in frame.items()
            if not key.startswith("_")
        })
    result = {
        key: value for key, value in camera.items()
        if key not in {"frames", "squared_error_px2"}
    }
    result["frames"] = frames
    return result


def _assessment_metrics(cameras, stereo):
    return {
        "cameras": {
            camera_id: {
                "reprojection": {
                    "status": "available" if row["rms_px"] is not None else "unavailable",
                    "count": row["corner_count"],
                    "rms_px": row["rms_px"],
                }
            }
            for camera_id, row in cameras.items()
        },
        "stereo_pairs": {
            "{}_{}".format(*cameras): {
                "alignment": stereo["alignment"],
            }
        },
    }


def _load_calibration(path, dataset_path):
    document = load_yaml(path)
    require_document_version(
        document, "camera calibration", "calibration_result")
    if document.get("calibration_type") != "cameras":
        raise FixedCameraValidationError(
            "fixed-camera validation requires calibration_type: cameras")
    task = {
        "_config_dir": str(path.parent),
        "job": "camera_imu_calibration",
        "dataset": {"type": "directory", "path": str(dataset_path)},
        "camera_calibration": {"path": str(path)},
    }
    cameras = load_cameras(task)
    if len(cameras) != 2:
        raise FixedCameraValidationError(
            "verify cameras currently requires exactly two cameras")
    return document, cameras


def _validate_inputs(dataset_path, target_path):
    if not dataset_path.is_dir() or dataset_path.is_symlink():
        raise FixedCameraValidationError(
            "dataset is not a directory: {}".format(dataset_path))
    if not (dataset_path / "dataset.yaml").is_file():
        raise FixedCameraValidationError(
            "directory dataset requires dataset.yaml: {}".format(dataset_path))
    if not target_path.is_file() or target_path.is_symlink():
        raise FixedCameraValidationError(
            "target is not a regular file: {}".format(target_path))
    load_target({
        "_config_dir": str(target_path.parent),
        "target": {"path": str(target_path)},
    })


def _safe_replace_directory(stage, destination, force):
    if destination.exists():
        if destination.is_symlink() or not destination.is_dir():
            raise FixedCameraValidationError(
                "output exists and is not a regular directory: {}".format(
                    destination))
        entries = list(destination.rglob("*"))
        inventory = destination / ".inventory.json"
        if entries and not force:
            raise FixedCameraValidationError(
                "output directory is not empty; pass --force to replace its "
                "managed validation files: {}".format(destination))
        if entries:
            if not inventory.is_file() or inventory.is_symlink():
                raise FixedCameraValidationError(
                    "refusing to replace output without a regular .inventory.json")
            try:
                registered = set(json.loads(
                    inventory.read_text(encoding="utf-8")))
            except (OSError, ValueError, TypeError) as error:
                raise FixedCameraValidationError(
                    "invalid output inventory: {}".format(error)) from error
            actual = {
                path.relative_to(destination).as_posix()
                for path in entries if path.is_file()
            }
            if (any(path.is_symlink() for path in entries)
                    or actual != registered | {".inventory.json"}
                    or registered != _OUTPUT_FILES):
                raise FixedCameraValidationError(
                    "refusing to replace output with unmanaged entries")
        shutil.rmtree(destination)
    os.replace(stage, destination)


def _readme(report):
    left_id, right_id = report["camera_ids"]
    lines = [
        "# 固定双目参数测试集验证",
        "",
        "该验证在新数据集上重新检测标定板，但固定已有标定结果的 K/D/T；不重新标定内参、畸变或双目外参。",
        "",
        "- 测试集：`{}`".format(report["dataset"]),
        "- 标定板：`{}`".format(report["target"]),
        "- 每目 RMS：固定 K/D 后逐帧估计标定板位姿，计算二维角点重投影 RMS。",
        "- 双目综合 RMS：合并两目角点残差平方和后按角点总数归一化。",
        "- Alignment RMS：固定 K/D/T 校正共同角点后，统计非视差方向误差 RMS。",
        "- 基线：固定外参平移向量的模，测试集不重新估计。",
        "- 参考评级沿用项目显示阈值，不等同于生产验收。",
        "",
        "| 参数来源 | {} RMS [px] | {} RMS [px] | 双目综合 RMS [px] | Alignment RMS [px] | Alignment mean abs [px] | 基线 [mm] | 评级 |".format(left_id, right_id),
        "|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    for label, result in report["results"].items():
        cameras = result["cameras"]
        stereo = result["stereo"]
        lines.append(
            "| `{}` | {:.6f} | {:.6f} | {:.6f} | {:.6f} | {:.6f} | {:.3f} | `{}` |".format(
                label,
                cameras[left_id]["rms_px"],
                cameras[right_id]["rms_px"],
                result["stereo_combined_rms_px"],
                stereo["alignment"]["rms_px"],
                stereo["alignment"]["mean_abs_px"],
                stereo["baseline_m"] * 1000.0,
                result["assessment"]["reference_grading"]["status"],
            ))
    lines.extend(["", "## 数据覆盖", "",
                  "| 参数来源 | {} 检测/输入帧 | {} 检测/输入帧 | {} 角点 | {} 角点 | 双目帧对 | 共同有效角点 |".format(left_id, right_id, left_id, right_id),
                  "|---|---:|---:|---:|---:|---:|---:|"])
    for label, result in report["results"].items():
        cameras = result["cameras"]
        stereo = result["stereo"]
        lines.append(
            "| `{}` | {}/{} | {}/{} | {} | {} | {} | {} |".format(
                label,
                cameras[left_id]["detected_frames"], cameras[left_id]["input_frames"],
                cameras[right_id]["detected_frames"], cameras[right_id]["input_frames"],
                cameras[left_id]["corner_count"], cameras[right_id]["corner_count"],
                stereo["used_pairs"], stereo["valid_rectified_corners"],
            ))
    return "\n".join(lines) + "\n"


def verify_fixed_cameras(calibrations, dataset, target, output_dir, *,
                         window_half_size_px=2,
                         max_displacement_px=math.sqrt(1.5),
                         synchronization_tolerance_s=0.0002,
                         rectification_balance=0.0,
                         rectification_fov_scale=1.0,
                         detector_opencv_threads=1,
                         force=False):
    """Validate one or more saved stereo calibrations on a new dataset."""
    dataset_path = Path(dataset).expanduser().resolve()
    target_path = Path(target).expanduser().resolve()
    destination = Path(output_dir).expanduser().resolve()
    _validate_inputs(dataset_path, target_path)
    calibration_specs = parse_calibration_specs(calibrations)
    for source in [dataset_path, target_path] + [path for _, path in calibration_specs]:
        if destination == source or destination in source.parents or source in destination.parents:
            raise FixedCameraValidationError(
                "validation output must be separate from every input")
    if type(window_half_size_px) is not int or window_half_size_px < 1:
        raise FixedCameraValidationError(
            "window_half_size_px must be a positive integer")
    for value, name, minimum, exclusive in (
            (max_displacement_px, "max_displacement_px", 0.0, True),
            (synchronization_tolerance_s, "synchronization_tolerance_s", 0.0, False),
            (rectification_balance, "rectification_balance", 0.0, False),
            (rectification_fov_scale, "rectification_fov_scale", 0.0, True)):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or (value <= minimum if exclusive else value < minimum)):
            raise FixedCameraValidationError(
                "{} is outside its legal range".format(name))
    if rectification_balance > 1.0:
        raise FixedCameraValidationError(
            "rectification_balance must be in [0, 1]")
    if type(detector_opencv_threads) is not int or detector_opencv_threads < 1:
        raise FixedCameraValidationError(
            "detector_opencv_threads must be a positive integer")

    import cv2
    cv2.setNumThreads(detector_opencv_threads)
    started = time.monotonic()
    report = {
        "schema_version": SCHEMA_VERSION,
        "software_version": VERSION,
        "kind": "fixed_camera_validation",
        "dataset": str(dataset_path),
        "target": str(target_path),
        "method": {
            "intrinsics_extrinsics_fixed": True,
            "optimizer_run": False,
            "camera_pose": "independent per-frame target pose using fixed K/D",
            "reprojection_rms": "sqrt(sum(dx^2+dy^2)/number_of_corners)",
            "stereo_combined_rms": "sqrt((sum_cam0_squared_error+sum_cam1_squared_error)/(cam0_corners+cam1_corners))",
            "alignment": "fixed K/D/T rectification; RMS of the non-disparity coordinate",
            "window_half_size_px": window_half_size_px,
            "max_displacement_px": float(max_displacement_px),
            "synchronization_tolerance_s": float(synchronization_tolerance_s),
            "rectification": {
                "balance": float(rectification_balance),
                "fov_scale": float(rectification_fov_scale),
                "size": None,
            },
            "detector_opencv_threads": detector_opencv_threads,
        },
        "camera_ids": None,
        "results": {},
    }
    for label, calibration_path in calibration_specs:
        _, cameras = _load_calibration(calibration_path, dataset_path)
        camera_ids = [camera["id"] for camera in cameras]
        if report["camera_ids"] is None:
            report["camera_ids"] = camera_ids
        elif camera_ids != report["camera_ids"]:
            raise FixedCameraValidationError(
                "all calibration inputs must use the same ordered camera IDs")
        detected = {
            camera["id"]: _detect_camera(
                dataset_path, target_path, camera,
                window_half_size_px, max_displacement_px)
            for camera in cameras
        }
        missing = [
            camera["id"] for camera in cameras
            if not detected[camera["id"]]["corner_count"]
        ]
        if missing:
            raise FixedCameraValidationError(
                "no target corners detected for {} using calibration {}"
                .format(", ".join(missing), label))
        stereo = _stereo_metrics(
            cameras, detected, synchronization_tolerance_s,
            report["method"]["rectification"])
        if stereo["alignment"]["rms_px"] is None:
            raise FixedCameraValidationError(
                "no valid synchronized common corners for calibration {}"
                .format(label))
        public_cameras = {
            camera_id: _public_camera(row)
            for camera_id, row in detected.items()
        }
        assessment = assess_metrics(
            _assessment_metrics(detected, stereo),
            {"assessment": {"reference_grading": True, "rules": []}},
        )
        report["results"][label] = {
            "source_calibration": str(calibration_path),
            "cameras": public_cameras,
            "stereo_combined_rms_px": combined_reprojection_rms(
                detected.values()),
            "stereo": stereo,
            "assessment": assessment,
        }
    report["wall_seconds"] = time.monotonic() - started

    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(
        prefix="." + destination.name + ".", dir=destination.parent))
    try:
        (stage / "fixed_camera_validation.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2,
                       allow_nan=False) + "\n",
            encoding="utf-8")
        (stage / "README_ZH.md").write_text(
            _readme(report), encoding="utf-8")
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "kind": "fixed_camera_validation_run",
            "software": {
                "name": "kalibr-noros",
                "version": VERSION,
                "git_commit": GIT_COMMIT,
                "source_variant": SOURCE_VARIANT,
            },
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "status": "completed",
            "files": sorted(_OUTPUT_FILES - {"run_manifest.json"}),
        }
        (stage / "run_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2,
                       allow_nan=False) + "\n",
            encoding="utf-8")
        (stage / ".inventory.json").write_text(
            json.dumps(sorted(_OUTPUT_FILES), indent=2) + "\n",
            encoding="utf-8")
        _safe_replace_directory(stage, destination, force)
    finally:
        if stage.exists():
            shutil.rmtree(stage)
    return report
