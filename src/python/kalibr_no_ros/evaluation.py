"""Model-aware, read-only evaluation of the final native observations.

These statistics never feed back into calibration or observation selection.
Pixel RMS is the RMS of two-dimensional point distances, not of coordinates.
"""

from __future__ import annotations

import math
import re
from bisect import bisect_left, bisect_right

import numpy as np

from .version import SCHEMA_VERSION, VERSION


class ReportingError(ValueError):
    pass


def _mapping(value, name, keys):
    if not isinstance(value, dict):
        raise ReportingError("{} must be a mapping".format(name))
    unknown = set(value) - set(keys)
    if unknown:
        raise ReportingError("{}: unknown fields {}".format(name, sorted(unknown)))
    return value


def _number(value, name, minimum=None, maximum=None, exclusive=False):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ReportingError("{} must be a finite number".format(name))
    if minimum is not None and (value <= minimum if exclusive else value < minimum):
        raise ReportingError("{} is below its legal range".format(name))
    if maximum is not None and value > maximum:
        raise ReportingError("{} is above its legal range".format(name))
    return value


def validate_output_options(options=None):
    """Validate public output/evaluation settings without importing native code."""
    bool_defaults = {
        "archive_observations": False, "archive_selection_history": False,
        "copy_used_images": False, "export_opencv": False,
        "save_diagnostics": False, "save_metrics": False, "export_text": False,
        "export_poses": False, "interactive_report": False, "verbose": False,
        "show_extraction": False, "extraction_stepping": False,
    }
    options = {} if options is None else options
    _mapping(options, "output", set(bool_defaults) | {"name", "visualizations", "rectification", "assessment", "evaluation_pairing_tolerance_s"})
    result = {}
    name = options.get("name")
    if name is not None and (not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,159}", name)):
        raise ReportingError("output.name must be a safe filename stem (letters, digits, underscores, hyphens; max 160)")
    result["name"] = name
    for key, default in bool_defaults.items():
        value = options.get(key, default)
        if type(value) is not bool:
            raise ReportingError("output.{} must be boolean".format(key))
        result[key] = value
    result["evaluation_pairing_tolerance_s"] = _number(
        options.get("evaluation_pairing_tolerance_s", 0.0002),
        "output.evaluation_pairing_tolerance_s", 0.0)
    if result["archive_selection_history"] and not result["archive_observations"]:
        raise ReportingError("archive_selection_history requires archive_observations")
    visual = _mapping(options.get("visualizations", {}), "visualizations",
                      {"enabled", "max_frames_per_camera", "max_pairs", "sampling"})
    if type(visual.get("enabled", False)) is not bool:
        raise ReportingError("visualizations.enabled must be boolean")
    result["visualizations"] = {"enabled": visual.get("enabled", False), "sampling": "uniform"}
    if visual.get("sampling", "uniform") != "uniform":
        raise ReportingError("visualizations.sampling must be uniform")
    for key in ("max_frames_per_camera", "max_pairs"):
        value = visual.get(key, 30)
        if type(value) is not int or value < 1:
            raise ReportingError("visualizations.{} must be a positive integer".format(key))
        result["visualizations"][key] = value
    rect = _mapping(options.get("rectification", {}), "rectification", {"balance", "fov_scale", "size"})
    result["rectification"] = {
        "balance": _number(rect.get("balance", 0.0), "rectification.balance", 0.0, 1.0),
        "fov_scale": _number(rect.get("fov_scale", 1.0), "rectification.fov_scale", 0.0, exclusive=True),
        "size": rect.get("size"),
    }
    size = result["rectification"]["size"]
    if size is not None and (not isinstance(size, list) or len(size) != 2
                             or any(type(v) is not int or v <= 0 for v in size)):
        raise ReportingError("rectification.size must be [positive width, positive height]")
    assess = _mapping(options.get("assessment", {}), "assessment", {"rules", "reference_grading"})
    grading = assess.get("reference_grading", True)
    if type(grading) is not bool:
        raise ReportingError("assessment.reference_grading must be boolean")
    rules = assess.get("rules", [])
    if not isinstance(rules, list):
        raise ReportingError("assessment.rules must be a list")
    clean_rules = []
    for index, rule in enumerate(rules):
        label = "assessment.rules[{}]".format(index)
        _mapping(rule, label, {"metric", "min", "max", "required"})
        metric = rule.get("metric")
        if not isinstance(metric, str) or not re.fullmatch(r"[A-Za-z0-9_\-]+(?:\.[A-Za-z0-9_\-]+)*", metric):
            raise ReportingError(label + ".metric must be a dotted metric path")
        if not ({"min", "max"} & set(rule)):
            raise ReportingError(label + " requires min or max")
        item = {"metric": metric, "required": rule.get("required", True)}
        if type(item["required"]) is not bool:
            raise ReportingError(label + ".required must be boolean")
        for bound in ("min", "max"):
            if bound in rule:
                item[bound] = _number(rule[bound], label + "." + bound)
        if "min" in item and "max" in item and item["min"] > item["max"]:
            raise ReportingError(label + ": min exceeds max")
        clean_rules.append(item)
    result["assessment"] = {"reference_grading": grading, "rules": clean_rules}
    return result


def vector(value, length):
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=float).reshape(-1)
    except (ValueError, TypeError):
        return None
    if array.size != length or not np.all(np.isfinite(array)):
        return None
    return array


def distribution(values, unit="px"):
    data = np.asarray(list(values), dtype=float)
    data = data[np.isfinite(data)]
    if not len(data):
        result = {"status": "unavailable", "count": 0, "unit": unit, "rms": None,
                "mean": None, "std": None, "min": None, "max": None,
                "median": None, "p95": None}
        if unit == "px":
            result["rms_px"] = None
        return result
    result = {"status": "available", "count": int(len(data)), "unit": unit,
              "mean": float(np.mean(data)), "std": float(np.std(data)),
              "min": float(np.min(data)), "max": float(np.max(data)),
              "median": float(np.median(data)), "p95": float(np.percentile(data, 95)),
              "rms": float(np.sqrt(np.mean(data ** 2)))}
    if unit == "px":
        result["rms_px"] = result["rms"]
    return result


def corner_residual(corner):
    measurement = vector(corner.get("measurement_px"), 2)
    prediction = vector(corner.get("prediction_px"), 2)
    if measurement is not None and prediction is not None:
        return measurement - prediction
    return vector(corner.get("residual_px"), 2)


def normalized_statistics(records, dimensions):
    """Use explicitly exported whitening and robust loss; never infer weights."""
    norms, weights, losses = [], [], []
    for record in records:
        whitened = vector(record.get("whitened_residual"), dimensions)
        if whitened is not None:
            norms.append(float(np.linalg.norm(whitened)))
        for key, values in (("robust_weight", weights), ("weighted_squared_error", losses)):
            value = record.get(key)
            if not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value) and value >= 0:
                values.append(value)
    normalized = distribution(norms, "dimensionless")
    normalized["missing_residual_count"] = len(records) - len(norms)
    if norms and normalized["missing_residual_count"]:
        normalized["status"] = "incomplete"
    robust = {"status": "available" if losses and len(losses) == len(records) else "incomplete" if losses else "unavailable",
              "unit": "dimensionless", "count": len(losses),
              "missing_loss_count": len(records) - len(losses),
              "total_weighted_squared_error": float(sum(losses)) if losses else None,
              "weights": distribution(weights, "dimensionless")}
    return normalized, robust


def bias_spline_statistics(bias):
    """Describe actual spline values at retained measurement timestamps.

    Means/stds are sample statistics, not a constant-bias estimate or a
    time-integral of the spline. Full timestamps are retained for plotting.
    """
    samples = bias.get("samples", [])
    values, timestamps, series = [], [], []
    missing_times = 0
    for sample in samples:
        value = vector(sample.get("value"), 3)
        timestamp = sample.get("solver_timestamp_s")
        finite_time = (not isinstance(timestamp, bool) and isinstance(timestamp, (int, float))
                       and math.isfinite(timestamp))
        if value is not None:
            values.append(value)
        if finite_time:
            timestamps.append(timestamp)
        else:
            missing_times += 1
        entry = {"timestamp_ns": sample.get("timestamp_ns"),
                 "source_index": sample.get("source_index"),
                 "solver_timestamp_s": timestamp if finite_time else None,
                 "value": value.tolist() if value is not None else None}
        if sample.get("unavailable_reason"):
            entry["unavailable_reason"] = sample["unavailable_reason"]
        series.append(entry)
    status = "available" if values and len(values) == len(samples) and not missing_times else "incomplete" if values else "unavailable"
    unit = bias.get("units", "rad/s" if bias.get("kind") == "gyro" else "m/s^2")
    result = {"status": status, "count": len(values), "total_sample_count": len(samples),
              "missing_value_count": len(samples) - len(values), "missing_timestamp_count": missing_times,
              "unit": unit, "representation": bias.get("representation", "time_varying_spline"),
              "sampling": bias.get("sampling", "retained_imu_residual_timestamps"),
              "statistics_definition": "unweighted sample statistics of the time-varying spline at retained IMU residual timestamps",
              "start_solver_timestamp_s": min(timestamps) if timestamps else None,
              "end_solver_timestamp_s": max(timestamps) if timestamps else None,
              "axes": {}, "vector_norm": distribution([np.linalg.norm(v) for v in values], unit),
              "time_series": series}
    for index, axis in enumerate(("x", "y", "z")):
        result["axes"][axis] = distribution([v[index] for v in values], unit)
        if status == "incomplete":
            result["axes"][axis]["status"] = "incomplete"
    if status == "incomplete":
        result["vector_norm"]["status"] = "incomplete"
    return result


def merged_cameras(artifacts, calibration):
    final = {c.get("id", "cam{}".format(i)): c for i, c in enumerate(calibration.get("cameras", []))}
    result = []
    for index, camera in enumerate(artifacts.get("cameras", [])):
        identifier = camera.get("id", "cam{}".format(index))
        merged = dict(camera)
        merged.update(final.get(identifier, {}))
        merged["id"] = identifier
        merged["frames"] = camera.get("frames", [])
        result.append(merged)
    if not result:
        result = [dict(camera, frames=[]) for camera in final.values()]
    return result


def camera_geometry(camera):
    model = camera.get("model")
    if (camera.get("camera_model") == "pinhole_opencv_fisheye"
            or model == "pinhole_opencv_fisheye-opencv_fisheye"):
        model = "pinhole-opencv-fisheye"
    elif not model:
        model = "{}-{}".format(camera.get("camera_model"), camera.get("distortion_model"))
    intrinsic_count = 5 if model == "pinhole-opencv-fisheye" else 4
    intrinsics = vector(camera.get("intrinsics"), intrinsic_count)
    if intrinsics is None or np.any(intrinsics[:2] <= 0):
        raise ReportingError("{} requires {} finite pinhole intrinsics".format(
            camera["id"], intrinsic_count))
    if model in ("pinhole-equi", "pinhole-equidistant", "pinhole-opencv-fisheye"):
        family, length = "fisheye", 4
    elif model in ("pinhole-radtan", "pinhole-radtan5", "pinhole-radtan8"):
        family = "pinhole"
        length = {"pinhole-radtan": 4, "pinhole-radtan5": 5, "pinhole-radtan8": 8}[model]
    else:
        raise ReportingError("OpenCV rectification unsupported for {}".format(model))
    distortion = vector(camera.get("distortion_coeffs"), length)
    if distortion is None:
        raise ReportingError("invalid distortion coefficients for {}".format(model))
    size = camera.get("resolution")
    if not isinstance(size, (list, tuple)) or len(size) != 2 or any(type(v) is not int or v <= 0 for v in size):
        raise ReportingError("invalid camera resolution")
    fx, fy, cx, cy = intrinsics[:4]
    alpha = float(intrinsics[4]) if intrinsic_count == 5 else 0.0
    matrix = np.array([[fx, fx * alpha, cx], [0., fy, cy], [0., 0., 1.]])
    no_skew = matrix.copy() if alpha else matrix
    if alpha:
        no_skew[0, 1] = 0.0
    return {"K": matrix, "K_no_skew": no_skew, "alpha": alpha,
            "D": distortion, "size": tuple(size), "family": family, "model": model}


def stereo_geometry(left, right, rectification):
    import cv2

    first, second = camera_geometry(left), camera_geometry(right)
    if first["family"] != second["family"] or first["size"] != second["size"]:
        raise ReportingError("stereo rectification requires equal model families and source resolutions")
    transform = vector(right.get("T_cn_cnm1"), 16)
    if transform is None:
        raise ReportingError("stereo rectification requires T_cn_cnm1")
    transform = transform.reshape(4, 4)
    rotation, translation = transform[:3, :3], transform[:3, 3]
    if (not np.allclose(transform[3], [0., 0., 0., 1.], atol=1e-10)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)
            or np.linalg.norm(translation) <= 1e-12):
        raise ReportingError("invalid or zero-baseline stereo transform")
    size = tuple(rectification.get("size") or first["size"])
    arguments = (first["K_no_skew"], first["D"], second["K_no_skew"], second["D"], first["size"], rotation, translation)
    if first["family"] == "fisheye":
        R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
            *arguments, flags=cv2.CALIB_ZERO_DISPARITY, newImageSize=size,
            balance=rectification["balance"], fov_scale=rectification["fov_scale"])
    else:
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            *arguments, flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=rectification["balance"], newImageSize=size)
    if not all(np.all(np.isfinite(v)) for v in (R1, R2, P1, P2, Q)):
        raise ReportingError("rectification produced non-finite matrices")
    axis = "x" if abs(P2[0, 3]) >= abs(P2[1, 3]) else "y"
    return {"left": first, "right": second, "R": rotation, "T": translation,
            "R1": R1, "R2": R2, "P1": P1, "P2": P2, "Q": Q,
            "size": size, "disparity_axis": axis,
            "baseline_m": float(np.linalg.norm(translation))}


def rectified_points(points, geometry, side):
    import cv2

    camera = geometry[side]
    number = "1" if side == "left" else "2"
    points = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    if camera["alpha"]:
        # OpenCV fisheye undistortPoints ignores K[0,1]. Convert measured
        # pixels to the corresponding zero-skew sensor coordinates first.
        points = points.copy()
        points[..., 0] -= (camera["K"][0, 1] / camera["K"][1, 1]) * (
            points[..., 1] - camera["K"][1, 2])
    if camera["family"] == "fisheye":
        return cv2.fisheye.undistortPoints(
            points, camera["K_no_skew"], camera["D"],
            R=geometry["R" + number], P=geometry["P" + number]).reshape(-1, 2)
    # The five iterations used by undistortPoints can leave large inverse
    # errors near image edges for rational distortion. These errors must not
    # be counted as calibration/epipolar residuals. Only evaluation changes;
    # native calibration and forward image-remap models remain untouched.
    return cv2.undistortPointsIter(
        points, camera["K_no_skew"], camera["D"],
        geometry["R" + number], geometry["P" + number],
        (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-12),
    ).reshape(-1, 2)


def rectification_maps(geometry, side):
    """Map rectified pixels back to the original, potentially skewed sensor."""
    import cv2

    camera = geometry[side]
    number = "1" if side == "left" else "2"
    function = cv2.fisheye.initUndistortRectifyMap if camera["family"] == "fisheye" else cv2.initUndistortRectifyMap
    map_x, map_y = function(camera["K_no_skew"], camera["D"], geometry["R" + number],
                            geometry["P" + number], geometry["size"], cv2.CV_32FC1)
    if camera["alpha"]:
        map_x += (camera["K"][0, 1] / camera["K"][1, 1]) * (
            map_y - camera["K"][1, 2])
    return map_x, map_y


def paired_frames(artifacts, left, right):
    lframes = {f["frame_id"]: f for f in left.get("frames", []) if f.get("used")}
    rframes = {f["frame_id"]: f for f in right.get("frames", []) if f.get("used")}
    view_key = "evaluation_views" if artifacts.get("calibration_type") == "camera_imu" else "views"
    for view in artifacts.get(view_key, []):
        if not view.get("used", True):
            continue
        frame_ids = view.get("frame_ids", [])
        lf = [lframes[f] for f in frame_ids if f in lframes]
        rf = [rframes[f] for f in frame_ids if f in rframes]
        if len(lf) == 1 and len(rf) == 1:
            yield view, lf[0], rf[0]


def prepare_evaluation_artifacts(artifacts, options):
    """Pair asynchronous camera/IMU observations only for output diagnostics."""
    if artifacts.get("calibration_type") != "camera_imu":
        return artifacts
    result = dict(artifacts)
    views = []
    tolerance_ns = round(options["evaluation_pairing_tolerance_s"] * 1e9)
    cameras = artifacts.get("cameras", [])
    for left, right in zip(cameras, cameras[1:]):
        def ordered(camera):
            return sorted((frame for frame in camera.get("frames", [])
                           if frame.get("used") and type(frame.get("source_timestamp_ns")) is int),
                          key=lambda frame: (frame["source_timestamp_ns"], frame["source_index"]))
        lf, rf = ordered(left), ordered(right)
        timestamps = [frame["source_timestamp_ns"] for frame in rf]
        candidates = []
        for li, frame in enumerate(lf):
            stamp = frame["source_timestamp_ns"]
            lo, hi = bisect_left(timestamps, stamp - tolerance_ns), bisect_right(timestamps, stamp + tolerance_ns)
            for ri in range(lo, hi):
                candidates.append((abs(stamp - timestamps[ri]), frame["source_index"], rf[ri]["source_index"], li, ri))
        consumed_left, consumed_right, matched = set(), set(), []
        for _, _, _, li, ri in sorted(candidates):
            if li in consumed_left or ri in consumed_right:
                continue
            consumed_left.add(li)
            consumed_right.add(ri)
            matched.append((li, ri))
        for li, ri in sorted(matched):
            views.append({"view_id": "evaluation:{}:{}".format(lf[li]["frame_id"], rf[ri]["frame_id"]),
                          "frame_ids": [lf[li]["frame_id"], rf[ri]["frame_id"]], "used": True,
                          "pairing": "camera_imu_diagnostic_pairing"})
    result["evaluation_views"] = views
    result["evaluation_pairing_tolerance_s"] = options["evaluation_pairing_tolerance_s"]
    return result


def _paired_points(left, right):
    points = []
    for frame in (left, right):
        values = {}
        for corner in frame.get("corners", []):
            point = vector(corner.get("measurement_px"), 2)
            if corner.get("used", True) and point is not None:
                identifier = corner["corner_id"]
                if identifier in values:
                    raise ReportingError("duplicate corner ID in final observation")
                values[identifier] = point
        points.append(values)
    common = sorted(set(points[0]) & set(points[1]))
    return common, [points[0][key] for key in common], [points[1][key] for key in common]


def compute_metrics(artifacts, calibration, options=None):
    import cv2

    options = validate_output_options(options)
    artifacts = prepare_evaluation_artifacts(artifacts, options)
    cameras = merged_cameras(artifacts, calibration)
    metrics = {"schema_version": SCHEMA_VERSION, "software_version": VERSION,
               "kind": "calibration_metrics", "calibration_type": artifacts.get("calibration_type", calibration.get("calibration_type")),
               "population": "final_used_observations", "reprojection_definition": "sqrt(sum(dx^2 + dy^2) / number_of_corners)",
               "evaluation_options": {"rectification": options["rectification"],
                                      "evaluation_pairing_tolerance_s": options["evaluation_pairing_tolerance_s"]},
               "cameras": {}, "stereo_pairs": {}, "imus": {}}
    if artifacts.get("observability"):
        metrics["observability"] = dict(artifacts["observability"])
    for camera in cameras:
        frames = camera.get("frames", [])
        used = [f for f in frames if f.get("used")]
        norms, per_frame, missing, measured_points, final_corners = [], [], 0, [], []
        for frame in used:
            frame_norms = []
            for corner in frame.get("corners", []):
                if not corner.get("used", True):
                    continue
                final_corners.append(corner)
                point = vector(corner.get("measurement_px"), 2)
                if point is not None:
                    measured_points.append(point)
                residual = corner_residual(corner)
                if residual is None:
                    missing += 1
                else:
                    frame_norms.append(float(np.linalg.norm(residual)))
            norms.extend(frame_norms)
            per_frame.append({"frame_id": frame["frame_id"], **distribution(frame_norms)})
        reprojection = distribution(norms)
        reprojection["missing_residual_count"] = missing
        if missing:
            reprojection["status"] = "incomplete"
        metrics["cameras"][camera["id"]] = {
            "model": camera.get("model", camera.get("distortion_model")),
            "resolution": camera.get("resolution"), "input_frames": camera.get("source_frame_count"),
            "selected_frames": camera.get("selected_frame_count", len(frames)),
            "detected_frames": sum(bool(f.get("corners")) or f.get("detection_status") in ("detected", "success", "succeeded") for f in frames),
            "used_frames": len(used), "unused_frames": len(frames) - len(used),
            "reprojection": reprojection, "frames": per_frame}
        entry = metrics["cameras"][camera["id"]]
        entry["normalized_reprojection"], entry["robust_objective"] = normalized_statistics(final_corners, 2)
        entry["intrinsics"] = camera.get("intrinsics")
        entry["distortion_coeffs"] = camera.get("distortion_coeffs")
        if camera.get("shutter"):
            entry["shutter"] = dict(camera["shutter"])
        shift = camera.get("timeshift_cam_imu_s", camera.get("timeshift_cam_imu"))
        if shift is not None:
            entry["timeshift_cam_imu_s"] = shift
            entry["absolute_timeshift_cam_imu_s"] = abs(shift)
        resolution = vector(camera.get("resolution"), 2)
        entry["coverage"] = {"status": "unavailable", "grid_columns": 8, "grid_rows": 6,
                             "occupied_bin_ratio": None, "bounding_box_area_ratio": None}
        if measured_points and resolution is not None and np.all(resolution > 0):
            normalized = np.asarray(measured_points) / resolution
            inside = normalized[np.all((normalized >= 0) & (normalized < 1), axis=1)]
            if len(inside):
                bins = np.floor(inside * [8, 6]).astype(int)
                entry["coverage"].update(status="available",
                    occupied_bin_ratio=len(set(map(tuple, bins))) / 48.,
                    bounding_box_area_ratio=float(np.prod(np.max(inside, axis=0) - np.min(inside, axis=0))))
    for left, right in zip(cameras, cameras[1:]):
        identifier = left["id"] + "_" + right["id"]
        pair = {"left_camera": left["id"], "right_camera": right["id"], "pairs": [],
                "pairing": "native_target_view" if metrics["calibration_type"] == "cameras" else "camera_imu_diagnostic_pairing",
                "status": "unavailable", "rectification": dict(options["rectification"], zero_disparity=True)}
        if metrics["calibration_type"] == "camera_imu":
            pair["evaluation_pairing_tolerance_s"] = options["evaluation_pairing_tolerance_s"]
            pair["pairing_timebase"] = "source_timestamp_ns"
            pair["interpretation"] = "Spatial alignment diagnostic on time-separated observations; motion can contribute. Not an independent or strictly simultaneous validation."
        metrics["stereo_pairs"][identifier] = pair
        if left.get("shutter") or right.get("shutter"):
            pair["interpretation"] = "Optical rectification of the original rolling-shutter measurements, without row-time compensation. Motion and row sampling times can contribute; this is not the joint reprojection residual."
        try:
            geometry = stereo_geometry(left, right, options["rectification"])
        except (ReportingError, ValueError, RuntimeError, cv2.error) as error:
            pair["reason"] = str(error)
            continue
        pair.update({"baseline_m": geometry["baseline_m"], "translation_m": geometry["T"].tolist(),
                     "disparity_axis": geometry["disparity_axis"]})
        pair["rectification"]["size"] = list(geometry["size"])
        errors, disparities, removed = [], [], 0
        for view, lf, rf in paired_frames(artifacts, left, right):
            common, lp, rp = _paired_points(lf, rf)
            if not common:
                continue
            lp, rp = rectified_points(lp, geometry, "left"), rectified_points(rp, geometry, "right")
            width, height = geometry["size"]
            valid = np.all(np.isfinite(lp), axis=1) & np.all(np.isfinite(rp), axis=1)
            for points in (lp, rp):
                valid &= (points[:, 0] >= 0) & (points[:, 0] < width) & (points[:, 1] >= 0) & (points[:, 1] < height)
            removed += int(np.sum(~valid))
            d_axis = 0 if geometry["disparity_axis"] == "x" else 1
            delta = lp[valid] - rp[valid]
            alignment = np.abs(delta[:, 1 - d_axis])
            errors.extend(alignment.tolist())
            disparities.extend(delta[:, d_axis].tolist())
            timestamp0, timestamp1 = lf.get("source_timestamp_ns"), rf.get("source_timestamp_ns")
            solver0, solver1 = lf.get("solver_timestamp_s"), rf.get("solver_timestamp_s")
            pair["pairs"].append({"view_id": view.get("view_id"), "left_frame_id": lf["frame_id"],
                                  "right_frame_id": rf["frame_id"], "common_corners": len(common),
                                  "valid_rectified_corners": int(np.sum(valid)),
                                  "timestamp_difference_ns": abs(timestamp1 - timestamp0) if timestamp0 is not None and timestamp1 is not None else None,
                                  "solver_timestamp_difference_s": abs(solver1 - solver0) if solver0 is not None and solver1 is not None else None,
                                  "alignment": distribution(alignment)})
        pair["alignment"] = distribution(errors)
        pair["alignment"]["mean_abs_px"] = pair["alignment"]["mean"]
        pair["signed_disparity"] = distribution(disparities)
        pair["outside_rectified_domain_corners"] = removed
        pair["used_pairs"] = len(pair["pairs"])
        pair["status"] = "available" if errors else "unavailable"
        if not errors:
            pair["reason"] = "no common final corners in the rectified image domain"
    groups = {}
    for residual in artifacts.get("imu_residuals", []):
        key = (residual.get("imu_id", "imu0"), residual.get("kind"))
        groups.setdefault(key, []).append(residual)
    for (identifier, kind), residuals in groups.items():
        valid = [vector(r.get("residual"), 3) for r in residuals]
        values = [r for r in valid if r is not None]
        unit = "rad/s" if kind == "gyro" else "m/s^2"
        summary = distribution([np.linalg.norm(r) for r in values], unit)
        summary["missing_residual_count"] = len(residuals) - len(values)
        summary["rms_per_axis"] = np.sqrt(np.mean(np.asarray(values) ** 2, axis=0)).tolist() if values else None
        summary["normalized_residual"], summary["robust_objective"] = normalized_statistics(residuals, 3)
        if summary["missing_residual_count"]:
            summary["status"] = "incomplete"
        metrics["imus"].setdefault(identifier, {})[kind] = summary
    for bias in artifacts.get("imu_biases", []):
        identifier, kind = bias.get("imu_id", "imu0"), bias.get("kind")
        if kind not in ("gyro", "accel"):
            raise ReportingError("unknown IMU bias kind")
        imu = metrics["imus"].setdefault(identifier, {})
        if kind not in imu:
            imu[kind] = distribution([], "rad/s" if kind == "gyro" else "m/s^2")
        imu[kind]["bias_spline"] = bias_spline_statistics(bias)
    metrics["solver_state"] = artifacts.get("state")
    metrics["optimizer"] = artifacts.get("optimizer")
    metrics["objective"] = artifacts.get("objective")
    return metrics


def _metric_value(metrics, path):
    value = metrics
    for part in path.split("."):
        if not isinstance(value, dict):
            return None
        # A distribution can have an available partial value while missing
        # required samples. A pair's missing alignment must not hide its known
        # baseline, which can be evaluated without image observations.
        if ("count" in value and value.get("status") in ("incomplete", "unavailable")
                and not isinstance(value.get(part), dict)):
            return None
        value = value.get(part)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return value


def assess_metrics(metrics, options=None):
    options = validate_output_options(options)
    rules, results = options["assessment"]["rules"], []
    for rule in rules:
        value = _metric_value(metrics, rule["metric"])
        status = "unavailable" if value is None else "pass"
        if value is not None and (value < rule.get("min", -math.inf) or value > rule.get("max", math.inf)):
            status = "fail"
        results.append(dict(rule, value=value, status=status))
    if not rules or not any(r["status"] != "unavailable" or r["required"] for r in results):
        status = "not_evaluated"
    elif any(r["status"] == "fail" for r in results):
        status = "fail"
    elif any(r["status"] == "unavailable" and r["required"] for r in results):
        status = "incomplete"
    else:
        status = "pass"
    assessment = {"schema_version": SCHEMA_VERSION, "software_version": VERSION,
                  "kind": "calibration_assessment", "status": status, "rules": results,
                  "reference_grading": {"enabled": options["assessment"]["reference_grading"],
                                        "scope": "camera_reprojection_and_stereo_alignment_only",
                                        "status": "not_evaluated", "metrics": []}}
    if options["assessment"]["reference_grading"]:
        entries = []
        for name in metrics.get("cameras", {}):
            entries.append(("cameras.{}.reprojection.rms_px".format(name), (0.1, 0.5, 1.0)))
        for name in metrics.get("stereo_pairs", {}):
            entries.append(("stereo_pairs.{}.alignment.mean_abs_px".format(name), (0.3, 0.5, 1.0)))
        grades = []
        labels = ("excellent", "good", "acceptable", "poor")
        for path, thresholds in entries:
            value = _metric_value(metrics, path)
            grade = None if value is None else sum(value >= threshold for threshold in thresholds)
            grades.append(grade)
            assessment["reference_grading"]["metrics"].append({"metric": path, "value": value,
                "thresholds_px": list(thresholds), "grade": "unavailable" if grade is None else labels[grade]})
        assessment["reference_grading"]["status"] = (
            "incomplete" if not grades or None in grades else labels[max(grades)])
        assessment["reference_grading"]["note"] = (
            "Reference display only; corrected 2D RMS. This is not a validated production acceptance policy.")
    return assessment
