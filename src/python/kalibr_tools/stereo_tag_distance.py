"""Measure the 3-D center distance between two AprilTags in stereo images.

The tool uses Kalibr's native ETHZ AprilTag2 detector without optional
cornerSubPix refinement.  It accepts both normal calibration-result YAML and
machine ``stereo_imu_calib.yaml`` files containing pinhole-equidistant K/D/T.
"""

from __future__ import annotations

import bisect
import csv
import math
import os
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np
import yaml


class TagDistanceError(ValueError):
    pass


def _vector(value, length, label):
    result = np.asarray(value, dtype=np.float64).reshape(-1)
    if result.size != length or not np.isfinite(result).all():
        raise TagDistanceError(
            "{} must contain {} finite values".format(label, length))
    return result


def _camera_entry(document, camera_id):
    cameras = document.get("cameras")
    if isinstance(cameras, list):
        matches = [item for item in cameras
                   if isinstance(item, dict) and item.get("id") == camera_id]
        if len(matches) != 1:
            raise TagDistanceError(
                "calibration must contain exactly one {}".format(camera_id))
        return matches[0]
    cameras = document.get("camera")
    if isinstance(cameras, dict) and isinstance(cameras.get(camera_id), dict):
        return cameras[camera_id]
    raise TagDistanceError(
        "calibration does not contain camera {}".format(camera_id))


def _parse_camera(document, camera_id):
    source = _camera_entry(document, camera_id)
    model = str(source.get("model", ""))
    if (not model and source.get("camera_model") == "pinhole"
            and source.get("distortion_model") == "equidistant"):
        model = "pinhole-equi"
    if model not in ("pinhole-equi", "pinhole-equidistant"):
        raise TagDistanceError(
            "{} model {} is unsupported; expected pinhole-equi or "
            "pinhole-equidistant".format(camera_id, model or "<missing>"))
    intrinsics = source.get("intrinsics", source.get("intrinsic"))
    distortion = source.get(
        "distortion_coeffs", source.get("distortion"))
    resolution = _vector(source.get("resolution"), 2,
                         camera_id + ".resolution").astype(int)
    if np.any(resolution <= 0):
        raise TagDistanceError(camera_id + ".resolution must be positive")
    intrinsics = _vector(intrinsics, 4, camera_id + ".intrinsics")
    distortion = _vector(distortion, 4, camera_id + ".distortion")
    fx, fy, cx, cy = intrinsics
    if fx <= 0.0 or fy <= 0.0:
        raise TagDistanceError(camera_id + " focal lengths must be positive")
    return {
        "id": camera_id,
        "model": "pinhole-equi",
        "intrinsics": intrinsics.tolist(),
        "distortion_coeffs": distortion.tolist(),
        "resolution": resolution.tolist(),
        "K": np.asarray(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=np.float64),
    }


def load_calibration(path):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise TagDistanceError("calibration YAML does not exist: {}".format(path))
    try:
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise TagDistanceError(
            "invalid calibration YAML {}: {}".format(path, error)) from error
    if not isinstance(document, dict):
        raise TagDistanceError("calibration YAML root must be a mapping")
    cam0 = _parse_camera(document, "cam0")
    cam1 = _parse_camera(document, "cam1")
    transform = np.asarray(
        _camera_entry(document, "cam1").get("T_cn_cnm1"),
        dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise TagDistanceError("cam1.T_cn_cnm1 must be a finite 4x4 matrix")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12):
        raise TagDistanceError("cam1.T_cn_cnm1 has an invalid final row")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not math.isclose(np.linalg.det(rotation), 1.0,
                                rel_tol=0.0, abs_tol=1e-6)):
        raise TagDistanceError("cam1.T_cn_cnm1 rotation is not rigid")
    return path, cam0, cam1, transform


def _manifest_camera(document, camera_id):
    entries = document.get("cameras")
    if not isinstance(entries, list):
        raise TagDistanceError("dataset.yaml cameras must be a list")
    matches = [item for item in entries
               if isinstance(item, dict) and item.get("id") == camera_id]
    if len(matches) != 1:
        raise TagDistanceError(
            "dataset.yaml must contain exactly one {}".format(camera_id))
    return matches[0]


def _dataset_file(root, value, label):
    if not isinstance(value, str) or not value.strip():
        raise TagDistanceError(label + " must be a non-empty path")
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = root / candidate
    return candidate.resolve()


def _read_camera_index(root, manifest, camera_id):
    entry = _manifest_camera(manifest, camera_id)
    csv_path = _dataset_file(root, entry.get("timestamps"),
                             camera_id + ".timestamps")
    image_root = _dataset_file(root, entry.get("images"),
                               camera_id + ".images")
    if not csv_path.is_file() or not image_root.is_dir():
        raise TagDistanceError(
            "{} timestamp CSV or image directory is missing".format(camera_id))
    records = []
    seen = set()
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row_number, row in enumerate(csv.DictReader(stream), 2):
            try:
                timestamp = int(row["timestamp_ns"])
                filename = row["filename"]
            except (KeyError, TypeError, ValueError) as error:
                raise TagDistanceError(
                    "invalid {} row {}".format(csv_path, row_number)) from error
            if timestamp in seen:
                raise TagDistanceError(
                    "duplicate {} timestamp {}".format(camera_id, timestamp))
            seen.add(timestamp)
            image_path = _dataset_file(image_root, filename,
                                       camera_id + ".filename")
            if not image_path.is_file():
                raise TagDistanceError("image does not exist: {}".format(image_path))
            try:
                display_path = image_path.relative_to(root).as_posix()
            except ValueError:
                display_path = str(image_path)
            records.append({
                "timestamp_ns": timestamp,
                "path": image_path,
                "display_path": display_path,
            })
    if not records:
        raise TagDistanceError(camera_id + " contains no images")
    records.sort(key=lambda item: item["timestamp_ns"])
    return records


def _key(row, candidates):
    for name in candidates:
        if name in row and row[name] not in (None, ""):
            return int(row[name])
    raise TagDistanceError(
        "pair CSV is missing one of: {}".format(", ".join(candidates)))


def _pairs_from_manifest(root, left, right):
    path = root / "meta" / "pairs.csv"
    if not path.is_file():
        return None
    left_by_time = {item["timestamp_ns"]: item for item in left}
    right_by_time = {item["timestamp_ns"]: item for item in right}
    pairs = []
    used_left = set()
    used_right = set()
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row_number, row in enumerate(csv.DictReader(stream), 2):
            try:
                left_time = _key(
                    row, ("left_timestamp_ns", "cam0_timestamp_ns"))
                right_time = _key(
                    row, ("right_timestamp_ns", "cam1_timestamp_ns"))
                left_item = left_by_time[left_time]
                right_item = right_by_time[right_time]
            except (KeyError, ValueError) as error:
                raise TagDistanceError(
                    "invalid {} row {}".format(path, row_number)) from error
            if left_time in used_left or right_time in used_right:
                raise TagDistanceError("pair CSV reuses a camera frame")
            used_left.add(left_time)
            used_right.add(right_time)
            pairs.append((left_item, right_item))
    return pairs


def _pairs_by_timestamp(left, right, tolerance_ns):
    right_times = [item["timestamp_ns"] for item in right]
    used = set()
    pairs = []
    for left_item in left:
        timestamp = left_item["timestamp_ns"]
        begin = bisect.bisect_left(right_times, timestamp - tolerance_ns)
        end = bisect.bisect_right(right_times, timestamp + tolerance_ns)
        candidates = [index for index in range(begin, end) if index not in used]
        if not candidates:
            continue
        index = min(candidates,
                    key=lambda candidate: abs(right_times[candidate] - timestamp))
        used.add(index)
        pairs.append((left_item, right[index]))
    return pairs


def load_dataset(path, tolerance_ns):
    root = Path(path).expanduser().resolve()
    manifest_path = root / "dataset.yaml"
    if not manifest_path.is_file():
        raise TagDistanceError("dataset.yaml does not exist: {}".format(root))
    try:
        with manifest_path.open("r", encoding="utf-8") as stream:
            manifest = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise TagDistanceError(
            "invalid dataset YAML {}: {}".format(
                manifest_path, error)) from error
    if not isinstance(manifest, dict):
        raise TagDistanceError("dataset.yaml root must be a mapping")
    left = _read_camera_index(root, manifest, "cam0")
    right = _read_camera_index(root, manifest, "cam1")
    pairs = _pairs_from_manifest(root, left, right)
    if pairs is None:
        pairs = _pairs_by_timestamp(left, right, tolerance_ns)
    if not pairs:
        raise TagDistanceError("no stereo pairs were found")
    return root, left, right, pairs


def read_grayscale(path):
    data = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    image = cv2.imdecode(
        data, cv2.IMREAD_GRAYSCALE | cv2.IMREAD_IGNORE_ORIENTATION)
    if image is None:
        raise TagDistanceError("cannot decode image: {}".format(path))
    return image


def _tag_corner_indices(local_tag_id, tag_columns):
    row, column = divmod(local_tag_id, tag_columns)
    base = row * 4 * tag_columns + 2 * column
    return (base, base + 1, base + 2 * tag_columns + 1,
            base + 2 * tag_columns)


class NativeTagDetector:
    """Extract selected raw tag corners even when a two-tag grid is not a valid board."""

    def __init__(self, camera, tag_ids, border_bits):
        import aslam_cameras_april as april
        import aslam_cv as acv
        from kalibr_no_ros.native_bridge.camera_geometry import (
            create_camera_geometry,
        )

        self.tag_ids = tuple(tag_ids)
        self.first_tag_id = min(tag_ids)
        self.tag_columns = max(tag_ids) - self.first_tag_id + 1
        options = april.AprilgridOptions()
        # Reproduce the adopted two-tag method; forced refinement reduced repeatability.
        options.doSubpixRefinement = False
        options.minTagsForValidObs = 1
        options.blackTagBorder = int(border_bits)
        target = april.GridCalibrationTargetAprilgrid(
            1, self.tag_columns, 1.0, 0.3, self.first_tag_id, options)
        detector_options = acv.GridDetectorOptions()
        detector_options.filterCornerOutliers = False
        self.detector = acv.GridDetector(
            create_camera_geometry(camera), target, detector_options)
        self.border_bits = int(border_bits)

    def detect(self, image):
        _accepted, observation = self.detector.findTargetNoTransformation(image)
        identifiers = np.asarray(observation.getCornersIdx(), dtype=int).reshape(-1)
        points = np.asarray(
            observation.getCornersImageFrame(), dtype=np.float64).reshape(-1, 2)
        by_identifier = {int(identifier): point
                         for identifier, point in zip(identifiers, points)}
        tags = {}
        for tag_id in self.tag_ids:
            local = tag_id - self.first_tag_id
            corner_ids = _tag_corner_indices(local, self.tag_columns)
            if all(identifier in by_identifier for identifier in corner_ids):
                tags[tag_id] = np.asarray(
                    [by_identifier[identifier] for identifier in corner_ids],
                    dtype=np.float64)
        return {
            "border_bits": self.border_bits,
            "corner_count": len(by_identifier),
            "tags": tags,
        }


class AutoBorderDetector:
    def __init__(self, camera, tag_ids, border):
        borders = (1, 2) if border == "auto" else (int(border),)
        self.detectors = [NativeTagDetector(camera, tag_ids, value)
                          for value in borders]

    def detect(self, image):
        candidates = [detector.detect(image) for detector in self.detectors]
        return max(candidates, key=lambda result: (
            len(result["tags"]), result["corner_count"],
            -result["border_bits"]))


def undistort_points(points, camera):
    return cv2.fisheye.undistortPoints(
        np.asarray(points, dtype=np.float64).reshape(-1, 1, 2),
        camera["K"], np.asarray(
            camera["distortion_coeffs"], dtype=np.float64)).reshape(-1, 2)


def triangulate_points(points0, points1, cam0, cam1, transform):
    normalized0 = undistort_points(points0, cam0)
    normalized1 = undistort_points(points1, cam1)
    projection0 = np.column_stack((np.eye(3), np.zeros(3)))
    projection1 = transform[:3]
    homogeneous = cv2.triangulatePoints(
        projection0, projection1, normalized0.T, normalized1.T)
    if np.any(np.abs(homogeneous[3]) < 1e-12):
        raise TagDistanceError("triangulation produced a point at infinity")
    points = (homogeneous[:3] / homogeneous[3]).T
    points1_camera = (transform[:3, :3] @ points.T).T + transform[:3, 3]
    if (not np.isfinite(points).all() or np.any(points[:, 2] <= 0.0)
            or np.any(points1_camera[:, 2] <= 0.0)):
        raise TagDistanceError("triangulation produced an invalid/negative depth")
    return points


def _project(points, camera):
    projected, _jacobian = cv2.fisheye.projectPoints(
        np.asarray(points, dtype=np.float64).reshape(-1, 1, 3),
        np.zeros(3), np.zeros(3), camera["K"],
        np.asarray(camera["distortion_coeffs"], dtype=np.float64))
    return projected.reshape(-1, 2)


def evaluate_detections(detected0, detected1, cam0, cam1, transform, tag_ids):
    for camera_id, detected in (("cam0", detected0), ("cam1", detected1)):
        missing = [tag_id for tag_id in tag_ids
                   if tag_id not in detected["tags"]]
        if missing:
            raise TagDistanceError(
                "{} did not detect complete tag(s): {}".format(
                    camera_id, ", ".join(map(str, missing))))
    points3d = {}
    measured0 = []
    measured1 = []
    reconstructed = []
    for tag_id in tag_ids:
        corners0 = detected0["tags"][tag_id]
        corners1 = detected1["tags"][tag_id]
        points = triangulate_points(
            corners0, corners1, cam0, cam1, transform)
        points3d[tag_id] = points
        measured0.append(corners0)
        measured1.append(corners1)
        reconstructed.append(points)
    measured0 = np.concatenate(measured0)
    measured1 = np.concatenate(measured1)
    reconstructed = np.concatenate(reconstructed)
    reconstructed1 = (
        (transform[:3, :3] @ reconstructed.T).T + transform[:3, 3])
    residuals = np.concatenate((
        np.linalg.norm(_project(reconstructed, cam0) - measured0, axis=1),
        np.linalg.norm(_project(reconstructed1, cam1) - measured1, axis=1),
    ))
    centers = {tag_id: np.mean(points3d[tag_id], axis=0)
               for tag_id in tag_ids}
    delta = centers[tag_ids[1]] - centers[tag_ids[0]]
    return {
        "border0": detected0["border_bits"],
        "border1": detected1["border_bits"],
        "center0_m": centers[tag_ids[0]],
        "center1_m": centers[tag_ids[1]],
        "delta_m": delta,
        "distance_mm": float(np.linalg.norm(delta) * 1000.0),
        "reprojection_rms_px": float(np.sqrt(np.mean(residuals * residuals))),
    }


def draw_detection(image, detected, camera_id, pair_index, timestamp_ns,
                   tag_ids, distance_mm=None, error=None):
    canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    scale = max(0.6, min(1.2, canvas.shape[1] / 3200.0))
    thickness = max(1, int(round(scale * 2.0)))
    colors = ((0, 220, 0), (0, 180, 255))
    for tag_index, tag_id in enumerate(tag_ids):
        corners = detected.get("tags", {}).get(tag_id)
        if corners is None:
            continue
        integer_corners = np.rint(corners).astype(np.int32)
        color = colors[tag_index % len(colors)]
        cv2.polylines(
            canvas, [integer_corners], True, color, thickness, cv2.LINE_AA)
        center = tuple(np.rint(np.mean(corners, axis=0)).astype(int))
        cv2.drawMarker(
            canvas, center, color, cv2.MARKER_CROSS,
            max(12, int(round(20 * scale))), thickness, cv2.LINE_AA)
        for corner_index, point in enumerate(integer_corners):
            location = tuple(point)
            cv2.circle(canvas, location, max(3, thickness + 1), color, -1,
                       cv2.LINE_AA)
            cv2.putText(
                canvas, str(corner_index), (location[0] + 6, location[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, scale * 0.55, color, thickness,
                cv2.LINE_AA)
        label_at = (
            max(5, int(np.min(integer_corners[:, 0]))),
            max(30, int(np.min(integer_corners[:, 1])) - 12),
        )
        cv2.putText(
            canvas, "tag {}".format(tag_id), label_at,
            cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

    status = ("distance={:.3f} mm".format(distance_mm)
              if distance_mm is not None else "FAILED: {}".format(error))
    header = "pair={} {} ts={} border={} {}".format(
        pair_index, camera_id, timestamp_ns,
        detected.get("border_bits", "?"), status)
    text_size, baseline = cv2.getTextSize(
        header, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    cv2.rectangle(
        canvas, (0, 0),
        (min(canvas.shape[1] - 1, text_size[0] + 20),
         text_size[1] + baseline + 20), (0, 0, 0), -1)
    cv2.putText(
        canvas, header, (10, text_size[1] + 10),
        cv2.FONT_HERSHEY_SIMPLEX, scale,
        (80, 255, 80) if distance_mm is not None else (0, 0, 255),
        thickness, cv2.LINE_AA)
    return canvas


def distance_statistics(values):
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    if not values.size or not np.isfinite(values).all():
        raise TagDistanceError("no finite distances are available")
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "sample_std": (float(np.std(values, ddof=1))
                       if values.size > 1 else None),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "range": float(np.ptp(values)),
    }


_PAIR_FIELDS = (
    "pair_index", "cam0_timestamp_ns", "cam1_timestamp_ns", "delta_us",
    "cam0_image", "cam1_image", "cam0_detection_image",
    "cam1_detection_image", "status", "error",
    "cam0_border_bits", "cam1_border_bits",
    "tag0_center_x_mm", "tag0_center_y_mm", "tag0_center_z_mm",
    "tag1_center_x_mm", "tag1_center_y_mm", "tag1_center_z_mm",
    "tag0_to_tag1_x_mm", "tag0_to_tag1_y_mm", "tag0_to_tag1_z_mm",
    "distance_mm", "stereo_reprojection_rms_px",
)


def _atomic_csv(path, fields, rows, overwrite):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not overwrite:
        raise TagDistanceError(
            "output already exists (use --overwrite): {}".format(path))
    handle = tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", newline="", prefix=path.name + ".",
        suffix=".tmp", dir=str(path.parent), delete=False)
    temporary = Path(handle.name)
    try:
        with handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        os.replace(str(temporary), str(path))
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def _atomic_png(path, image, overwrite):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not overwrite:
        raise TagDistanceError(
            "output already exists (use --overwrite): {}".format(path))
    encoded, payload = cv2.imencode(".png", image)
    if not encoded:
        raise TagDistanceError("cannot encode detection image: {}".format(path))
    handle = tempfile.NamedTemporaryFile(
        "wb", prefix=path.name + ".", suffix=".tmp",
        dir=str(path.parent), delete=False)
    temporary = Path(handle.name)
    try:
        with handle:
            handle.write(payload.tobytes())
        os.replace(str(temporary), str(path))
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def _format(value, digits=6):
    return "" if value is None else ("{:.{}f}".format(float(value), digits))


def _default_output_dir(dataset):
    parts = dataset.name.rsplit("_", 2)
    if (len(parts) != 3 or len(parts[1]) != 6 or len(parts[2]) != 4
            or not parts[1].isdigit() or not parts[2].isdigit()):
        raise TagDistanceError(
            "dataset directory must end with _YYMMDD_HHMM to derive the "
            "default distance output directory")
    return dataset.parent / "result" / "distance_{}_{}".format(
        parts[1], parts[2])


def _output_paths(dataset, output_csv, summary_csv, detection_dir):
    if output_csv:
        pairs_path = Path(output_csv).expanduser().resolve()
    else:
        pairs_path = _default_output_dir(dataset) / "distance.csv"
    if summary_csv:
        summary_path = Path(summary_csv).expanduser().resolve()
    else:
        summary_path = pairs_path.with_name(pairs_path.stem + "_summary.csv")
    if pairs_path == summary_path:
        raise TagDistanceError("pair CSV and summary CSV must be different files")
    if detection_dir:
        image_dir = Path(detection_dir).expanduser().resolve()
    else:
        image_dir = pairs_path.with_name(pairs_path.stem + "_detections")
    if image_dir.exists() and not image_dir.is_dir():
        raise TagDistanceError(
            "detection output is not a directory: {}".format(image_dir))
    return pairs_path, summary_path, image_dir


def run_distance(args, prefix):
    if args.tag0 < 0 or args.tag1 < 0 or args.tag0 > 586 or args.tag1 > 586:
        raise TagDistanceError("tag IDs must be within tag36h11 [0, 586]")
    if args.tag0 == args.tag1:
        raise TagDistanceError("tag0 and tag1 must be different")
    if not math.isfinite(args.max_pair_delta_ms) or args.max_pair_delta_ms < 0.0:
        raise TagDistanceError("max-pair-delta-ms must be finite and non-negative")

    runtime = Path(prefix).resolve()
    calibration_path, cam0, cam1, transform = load_calibration(args.calibration)
    dataset, left, right, pairs = load_dataset(
        args.dataset, int(round(args.max_pair_delta_ms * 1e6)))
    tag_ids = (args.tag0, args.tag1)
    pairs_path, summary_path, detection_dir = _output_paths(
        dataset, args.output_csv, args.summary_csv, args.detection_dir)
    for path in (pairs_path, summary_path):
        if path.exists() and not args.overwrite:
            raise TagDistanceError(
                "output already exists (use --overwrite): {}".format(path))

    detection_paths = []
    for pair_index, (left_item, right_item) in enumerate(pairs, 1):
        detection_paths.append((
            detection_dir / "cam0" / "pair_{:04d}_cam0_{}.png".format(
                pair_index, left_item["timestamp_ns"]),
            detection_dir / "cam1" / "pair_{:04d}_cam1_{}.png".format(
                pair_index, right_item["timestamp_ns"]),
        ))
    if not args.overwrite:
        existing = [path for paths in detection_paths for path in paths
                    if path.exists()]
        if existing:
            raise TagDistanceError(
                "output already exists (use --overwrite): {}".format(
                    existing[0]))
    detector0 = AutoBorderDetector(cam0, tag_ids, args.border)
    detector1 = AutoBorderDetector(cam1, tag_ids, args.border)
    rows = []
    distances = []
    failures = 0
    for pair_index, (left_item, right_item) in enumerate(pairs, 1):
        image0 = read_grayscale(left_item["path"])
        image1 = read_grayscale(right_item["path"])
        detected0 = {"tags": {}, "border_bits": "?"}
        detected1 = {"tags": {}, "border_bits": "?"}
        detection0_path, detection1_path = detection_paths[pair_index - 1]
        row = {
            "pair_index": pair_index,
            "cam0_timestamp_ns": left_item["timestamp_ns"],
            "cam1_timestamp_ns": right_item["timestamp_ns"],
            "delta_us": _format(abs(left_item["timestamp_ns"]
                                    - right_item["timestamp_ns"]) / 1000.0, 3),
            "cam0_image": left_item["display_path"],
            "cam1_image": right_item["display_path"],
            "cam0_detection_image": str(detection0_path),
            "cam1_detection_image": str(detection1_path),
            "status": "failed",
            "error": "",
        }
        result = None
        try:
            detected0 = detector0.detect(image0)
            detected1 = detector1.detect(image1)
            result = evaluate_detections(
                detected0, detected1, cam0, cam1, transform, tag_ids)
            center0 = result["center0_m"] * 1000.0
            center1 = result["center1_m"] * 1000.0
            delta = result["delta_m"] * 1000.0
            row.update({
                "status": "ok",
                "cam0_border_bits": result["border0"],
                "cam1_border_bits": result["border1"],
                "tag0_center_x_mm": _format(center0[0]),
                "tag0_center_y_mm": _format(center0[1]),
                "tag0_center_z_mm": _format(center0[2]),
                "tag1_center_x_mm": _format(center1[0]),
                "tag1_center_y_mm": _format(center1[1]),
                "tag1_center_z_mm": _format(center1[2]),
                "tag0_to_tag1_x_mm": _format(delta[0]),
                "tag0_to_tag1_y_mm": _format(delta[1]),
                "tag0_to_tag1_z_mm": _format(delta[2]),
                "distance_mm": _format(result["distance_mm"]),
                "stereo_reprojection_rms_px": _format(
                    result["reprojection_rms_px"]),
            })
            distances.append(result["distance_mm"])
            print("[{}/{}] {:.3f} mm".format(
                pair_index, len(pairs), result["distance_mm"]), flush=True)
        except Exception as error:  # Keep the per-image failure in the requested CSV.
            failures += 1
            row["error"] = str(error).replace("\n", " ")
            print("[{}/{}] 失败: {}".format(
                pair_index, len(pairs), row["error"]), file=sys.stderr,
                flush=True)
        _atomic_png(
            detection0_path,
            draw_detection(
                image0, detected0, "cam0", pair_index,
                left_item["timestamp_ns"], tag_ids,
                result["distance_mm"] if result is not None else None,
                row["error"]),
            args.overwrite)
        _atomic_png(
            detection1_path,
            draw_detection(
                image1, detected1, "cam1", pair_index,
                right_item["timestamp_ns"], tag_ids,
                result["distance_mm"] if result is not None else None,
                row["error"]),
            args.overwrite)
        rows.append(row)

    if not distances:
        raise TagDistanceError("all stereo pairs failed; no CSV was written")
    statistics = distance_statistics(distances)
    summary_rows = [
        {"metric": "mean", "value": _format(statistics["mean"]), "unit": "mm"},
        {"metric": "median", "value": _format(statistics["median"]), "unit": "mm"},
        {"metric": "sample_std", "value": _format(statistics["sample_std"]), "unit": "mm"},
        {"metric": "min", "value": _format(statistics["min"]), "unit": "mm"},
        {"metric": "max", "value": _format(statistics["max"]), "unit": "mm"},
        {"metric": "range", "value": _format(statistics["range"]), "unit": "mm"},
        {"metric": "valid_pair_count", "value": len(distances), "unit": "count"},
        {"metric": "failed_pair_count", "value": failures, "unit": "count"},
    ]
    _atomic_csv(pairs_path, _PAIR_FIELDS, rows, args.overwrite)
    _atomic_csv(summary_path, ("metric", "value", "unit"),
                summary_rows, args.overwrite)

    print("\n| 指标 | 距离 |")
    print("|---|---:|")
    labels = (
        ("平均值", "mean"), ("中位数", "median"),
        ("样本标准差", "sample_std"), ("最小值", "min"),
        ("最大值", "max"), ("极差", "range"),
    )
    for label, key in labels:
        value = statistics[key]
        display = "N/A" if value is None else "{:.2f} mm".format(value)
        print("| {} | {} |".format(label, display))
    print("\n有效图对: {}/{}；失败: {}".format(
        len(distances), len(pairs), failures))
    print("逐图对 CSV: {}".format(pairs_path))
    print("统计 CSV: {}".format(summary_path))
    print("检测标注图: {}".format(detection_dir))
    print("标定 YAML: {}".format(calibration_path))
    print("运行时: {}".format(runtime))
    print("坐标系: cam0；距离定义: 两个 tag 各自四个三角化角点均值之间的欧氏距离")
    return 1 if failures else 0
