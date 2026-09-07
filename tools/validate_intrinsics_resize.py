#!/usr/bin/env python3
"""Validate transferring camera intrinsics across an image resize.

The experiment keeps one physical camera and one set of source images fixed.
It compares the common direct K scaling rule with OpenCV's pixel-center-aware
resize convention, for both radtan8 and the full OpenCV fisheye model.
"""

import argparse
import copy
import csv
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


CONVENTIONS = ("direct_scale", "opencv_half_pixel")
ENGINEERING_CHECK_THRESHOLDS = {
    "required_detected_frame_fraction": 1.0,
    "analytic_projection_max_px": 1.0e-9,
    "observed_corner_mapping_p95_px": 0.5,
    "fixed_pose_reprojection_rms_px": 0.5,
}


def load_mapping(path):
    path = Path(path).expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise RuntimeError("YAML root must be a mapping: {}".format(path))
    return path, document


def scale_intrinsics(intrinsics, source_size, target_size, convention):
    """Scale [fx, fy, cx, cy, ...] without changing dimensionless terms."""
    if convention not in CONVENTIONS:
        raise ValueError("unknown resize convention: {}".format(convention))
    source_width, source_height = map(int, source_size)
    target_width, target_height = map(int, target_size)
    if min(source_width, source_height, target_width, target_height) <= 0:
        raise ValueError("source and target dimensions must be positive")
    values = np.asarray(intrinsics, dtype=np.float64).reshape(-1).copy()
    if values.size < 4 or not np.all(np.isfinite(values)):
        raise ValueError("intrinsics must contain at least four finite values")
    scale_x = target_width / float(source_width)
    scale_y = target_height / float(source_height)
    values[0] *= scale_x
    values[1] *= scale_y
    if convention == "direct_scale":
        values[2] *= scale_x
        values[3] *= scale_y
    else:
        values[2] = scale_x * (values[2] + 0.5) - 0.5
        values[3] = scale_y * (values[3] + 0.5) - 0.5
    return values


def transform_pixels(points, source_size, target_size, convention):
    """Transform continuous source pixel coordinates to resized coordinates."""
    if convention not in CONVENTIONS:
        raise ValueError("unknown resize convention: {}".format(convention))
    values = np.asarray(points, dtype=np.float64)
    if values.shape[-1] != 2:
        raise ValueError("pixel coordinates must end in dimension two")
    source_width, source_height = map(int, source_size)
    target_width, target_height = map(int, target_size)
    scale = np.asarray([
        target_width / float(source_width),
        target_height / float(source_height),
    ])
    if convention == "direct_scale":
        return values * scale
    return (values + 0.5) * scale - 0.5


def vector_statistics(chunks):
    nonempty = [np.asarray(chunk, dtype=np.float64).reshape(-1, 2)
                for chunk in chunks if np.asarray(chunk).size]
    if not nonempty:
        return {
            "count": 0,
            "mean_xy_px": None,
            "rms_norm_px": None,
            "median_norm_px": None,
            "p95_norm_px": None,
            "max_norm_px": None,
        }
    values = np.concatenate(nonempty, axis=0)
    norms = np.linalg.norm(values, axis=1)
    return {
        "count": int(values.shape[0]),
        "mean_xy_px": [float(item) for item in np.mean(values, axis=0)],
        "rms_norm_px": float(np.sqrt(np.mean(np.sum(values * values, axis=1)))),
        "median_norm_px": float(np.median(norms)),
        "p95_norm_px": float(np.percentile(norms, 95.0)),
        "max_norm_px": float(np.max(norms)),
    }


def per_frame_statistics(frames):
    """Summarize residual norms without hiding the worst sampled frame."""
    values = []
    for frame in frames:
        residuals = np.asarray(frame["residuals"], dtype=np.float64).reshape(
            -1, 2)
        if residuals.size == 0:
            continue
        norms = np.linalg.norm(residuals, axis=1)
        values.append({
            "sample": int(frame["sample"]),
            "timestamp_ns": int(frame["timestamp_ns"]),
            "count": int(residuals.shape[0]),
            "rms_norm_px": float(np.sqrt(np.mean(norms * norms))),
            "max_norm_px": float(np.max(norms)),
        })
    if not values:
        return {
            "frame_count": 0,
            "mean_frame_rms_norm_px": None,
            "worst_frame_rms_norm_px": None,
            "worst_frame_sample": None,
            "worst_frame_timestamp_ns": None,
            "max_point_norm_px": None,
            "per_frame": [],
        }
    worst_frame = max(values, key=lambda item: item["rms_norm_px"])
    return {
        "frame_count": len(values),
        "mean_frame_rms_norm_px": float(np.mean([
            item["rms_norm_px"] for item in values])),
        "worst_frame_rms_norm_px": worst_frame["rms_norm_px"],
        "worst_frame_sample": worst_frame["sample"],
        "worst_frame_timestamp_ns": worst_frame["timestamp_ns"],
        "max_point_norm_px": float(max(
            item["max_norm_px"] for item in values)),
        "per_frame": values,
    }


def observed_pixel_coverage(chunks, resolution):
    nonempty = [np.asarray(chunk, dtype=np.float64).reshape(-1, 2)
                for chunk in chunks if np.asarray(chunk).size]
    if not nonempty:
        return {"count": 0, "bounding_box_px": None,
                "bounding_box_fraction": None}
    values = np.concatenate(nonempty, axis=0)
    minimum = np.min(values, axis=0)
    maximum = np.max(values, axis=0)
    denominators = np.asarray(resolution, dtype=np.float64) - 1.0
    return {
        "count": int(values.shape[0]),
        "bounding_box_px": {
            "min_x": float(minimum[0]),
            "max_x": float(maximum[0]),
            "min_y": float(minimum[1]),
            "max_y": float(maximum[1]),
        },
        "bounding_box_fraction": {
            "min_x": float(minimum[0] / denominators[0]),
            "max_x": float(maximum[0] / denominators[0]),
            "min_y": float(minimum[1] / denominators[1]),
            "max_y": float(maximum[1] / denominators[1]),
        },
    }


def calibration_cameras(document):
    cameras = document.get("cameras")
    if isinstance(cameras, list):
        return cameras
    keys = sorted(
        (key for key in document
         if isinstance(key, str) and key.startswith("cam")
         and key[3:].isdigit()),
        key=lambda key: int(key[3:]),
    )
    return [document[key] for key in keys]


def load_camera_spec(path, camera_index, expected_kind):
    calibration_path, document = load_mapping(path)
    cameras = calibration_cameras(document)
    if camera_index < 0 or camera_index >= len(cameras):
        raise RuntimeError(
            "camera index {} is absent from {}".format(
                camera_index, calibration_path))
    camera = copy.deepcopy(cameras[camera_index])
    resolution = tuple(map(int, camera.get("resolution", ())))
    intrinsics = np.asarray(camera.get("intrinsics"), dtype=np.float64)
    distortion = np.asarray(
        camera.get("distortion_coeffs"), dtype=np.float64)
    distortion_model = str(camera.get("distortion_model"))
    camera_model = str(camera.get("camera_model"))
    if expected_kind == "radtan8":
        valid = (camera_model == "pinhole"
                 and distortion_model in ("radtan8", "rational_polynomial")
                 and intrinsics.shape == (4,) and distortion.shape == (8,))
    else:
        valid = (camera_model == "pinhole_opencv_fisheye"
                 and distortion_model == "opencv_fisheye"
                 and intrinsics.shape == (5,) and distortion.shape == (4,))
    if (not valid or len(resolution) != 2
            or not np.all(np.isfinite(intrinsics))
            or not np.all(np.isfinite(distortion))):
        raise RuntimeError(
            "{} cam{} is not a valid {} calibration".format(
                calibration_path, camera_index, expected_kind))
    return {
        "kind": expected_kind,
        "path": str(calibration_path),
        "camera": camera,
        "resolution": resolution,
        "intrinsics": intrinsics,
        "distortion": distortion,
    }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Validate 4K camera intrinsics on resized images.")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--target", type=Path)
    parser.add_argument("--radtan8-calibration", type=Path, required=True)
    parser.add_argument("--fisheye-calibration", type=Path, required=True)
    parser.add_argument("--camera-index", type=int, default=1)
    parser.add_argument("--target-width", type=int, default=1920)
    parser.add_argument("--target-height", type=int, default=1080)
    parser.add_argument("--window-half-size-px", type=int, default=7)
    parser.add_argument("--max-displacement-px", type=float, default=2.0)
    parser.add_argument("--visualization-samples", type=int, default=5)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def create_target(acv_april, target_document, window, displacement):
    if target_document.get("target_type") != "aprilgrid":
        raise RuntimeError("the resize experiment currently requires aprilgrid")
    options = acv_april.AprilgridOptions()
    options.subpixWindowHalfSize = int(window)
    options.maxSubpixDisplacement2 = float(displacement) ** 2
    rows = int(target_document["tagRows"])
    cols = int(target_document["tagCols"])
    options.minTagsForValidObs = max(rows, cols) + 1
    return acv_april.GridCalibrationTargetAprilgrid(
        rows, cols, float(target_document["tagSize"]),
        float(target_document["tagSpacing"]),
        int(target_document.get("tagStartId", 0)), options)


def create_geometry(aslam_camera_type, spec, resolution, intrinsics):
    camera = spec["camera"]
    return aslam_camera_type(
        camera["camera_model"], intrinsics.tolist(),
        camera["distortion_model"], spec["distortion"].tolist(),
        list(resolution)).geometry


def create_detector(acv, geometry, target):
    options = acv.GridDetectorOptions()
    options.filterCornerOutliers = False
    return acv.GridDetector(geometry, target, options)


def observation_from_manifest(acv, target, gray, timestamp_ns, records):
    observation = acv.GridCalibrationTargetObservation(target)
    observation.setImage(gray)
    observation.setTime(acv.Time(float(timestamp_ns) * 1.0e-9))
    for record in records:
        index = int(record["index"])
        observation.updateImagePoint(
            index,
            np.asarray([record["x"], record["y"]], dtype=np.float64))
    return observation


def observation_pixels(observation):
    return {
        int(index): np.asarray(point, dtype=np.float64)
        for index, point in zip(
            observation.getCornersIdx(), observation.getCornersImageFrame())
    }


def estimate_pose(geometry, observation):
    success, transform = geometry.estimateTransformation(observation)
    if not success:
        return None
    matrix = np.asarray(transform.T(), dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        return None
    return matrix


def reprojection_residuals(geometry, target, pixels, target_from_camera,
                           indices=None):
    camera_from_target = np.linalg.inv(target_from_camera)
    selected = sorted(pixels) if indices is None else sorted(indices)
    residuals = {}
    predictions = {}
    for index in selected:
        point_target = np.asarray(target.point(index), dtype=np.float64)
        point_camera = camera_from_target.dot(
            np.concatenate((point_target, [1.0])))[:3]
        prediction = np.asarray(
            geometry.euclideanToKeypoint(point_camera), dtype=np.float64)
        if prediction.shape == (2,) and np.all(np.isfinite(prediction)):
            predictions[index] = prediction
            residuals[index] = prediction - pixels[index]
    return residuals, predictions


def projection_equivalence(high_geometry, low_geometry, source_size,
                           target_size, convention):
    residuals = []
    for x in np.linspace(-0.8, 0.8, 33):
        for y in np.linspace(-0.45, 0.45, 21):
            point = np.asarray([x, y, 1.0], dtype=np.float64)
            if (not high_geometry.isEuclideanVisible(point)
                    or not low_geometry.isEuclideanVisible(point)):
                continue
            high_pixel = np.asarray(
                high_geometry.euclideanToKeypoint(point), dtype=np.float64)
            low_pixel = np.asarray(
                low_geometry.euclideanToKeypoint(point), dtype=np.float64)
            expected = transform_pixels(
                high_pixel, source_size, target_size, convention)
            residuals.append(low_pixel - expected)
    return np.asarray(residuals, dtype=np.float64)


def draw_overlay(image, actual, predicted, label):
    canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for index in sorted(set(actual) & set(predicted)):
        measured = tuple(np.rint(actual[index]).astype(int))
        estimate = tuple(np.rint(predicted[index]).astype(int))
        cv2.circle(canvas, measured, 2, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.drawMarker(
            canvas, estimate, (0, 255, 0), cv2.MARKER_CROSS, 7, 1,
            cv2.LINE_AA)
    cv2.putText(
        canvas, label, (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
        (255, 255, 255), 2, cv2.LINE_AA)
    return canvas


def format_number(value, digits=6):
    if value is None:
        return "n/a"
    return ("{:.%df}" % digits).format(value)


def recommended_parameter_document(summary):
    models = {}
    conventions = set()
    for model_name, model in summary["models"].items():
        convention = model["recommended_convention"]
        conventions.add(convention)
        candidate = model["candidates"][convention]
        models[model_name] = {
            "source_calibration": model["source_calibration"],
            "camera_model": (
                "pinhole" if model_name == "radtan8"
                else "pinhole_opencv_fisheye"),
            "distortion_model": (
                "radtan8" if model_name == "radtan8"
                else "opencv_fisheye"),
            "resolution": list(summary["target_resolution"]),
            "intrinsics": list(candidate["scaled_intrinsics"]),
            "distortion_coeffs": list(
                model["distortion_coefficients_unchanged"]),
        }
    if len(conventions) != 1:
        raise RuntimeError(
            "models do not share one recommended principal-point convention")
    return {
        "schema_version": 1,
        "kind": "resized_camera_intrinsics",
        "camera_id": "cam{}".format(summary["camera_index"]),
        "topic": summary["topic"],
        "source_resolution": list(summary["source_resolution"]),
        "target_resolution": list(summary["target_resolution"]),
        "resize_interpolation": summary["resize_interpolation"],
        "principal_point_convention": conventions.pop(),
        "models": models,
    }


def write_report(path, summary):
    rows = []
    for model_name, model in summary["models"].items():
        for convention, candidate in model["candidates"].items():
            mapping = candidate["observed_corner_mapping"]
            fixed = candidate["low_resolution_fixed_pose_reprojection"]
            fixed_frames = candidate["low_resolution_fixed_pose_per_frame"]
            refit = candidate[
                "low_resolution_refit_pose_common_reprojection"]
            projection = candidate["analytic_projection_equivalence"]
            rows.append(
                "| {model} | {rule} | {count} | {proj} | {biasx} | "
                "{biasy} | {mapping} | {p95} | {fixed} | {worst} "
                "(sample {sample}) | {refit} |".format(
                    model=model_name,
                    rule=convention,
                    count=mapping["count"],
                    proj=format_number(projection["max_norm_px"], 12),
                    biasx=format_number(mapping["mean_xy_px"][0]),
                    biasy=format_number(mapping["mean_xy_px"][1]),
                    mapping=format_number(mapping["rms_norm_px"]),
                    p95=format_number(mapping["p95_norm_px"]),
                    fixed=format_number(fixed["rms_norm_px"]),
                    worst=format_number(
                        fixed_frames["worst_frame_rms_norm_px"]),
                    sample=fixed_frames["worst_frame_sample"],
                    refit=format_number(refit["rms_norm_px"]),
                ))
    recommended = [
        "- `{}`：`{}`".format(name, model["recommended_convention"])
        for name, model in summary["models"].items()
    ]
    recommended_parameters = recommended_parameter_document(summary)["models"]
    parameter_lines = []
    for model_name, parameters in recommended_parameters.items():
        parameter_lines.extend([
            "#### {}".format(model_name),
            "",
            "```yaml",
            "resolution: {}".format(parameters["resolution"]),
            "intrinsics: {}".format(parameters["intrinsics"]),
            "distortion_coeffs: {}".format(parameters["distortion_coeffs"]),
            "```",
            "",
        ])
    coverage = summary["source_corner_coverage"]
    coverage_px = coverage["bounding_box_px"]
    coverage_fraction = coverage["bounding_box_fraction"]
    text = """# cam1 4K 内参缩放到 1080p 验证报告

## 1. 固定输入

- 数据：`{dataset}`；
- topic：`{topic}`，只使用 cam1；
- 原分辨率：`{source_width}×{source_height}`；
- 目标分辨率：`{target_width}×{target_height}`；
- 缩放：`cv2.resize(..., INTER_AREA)`；
- 固定样本：{sample_count} 帧，不重新选择数据；
- 模型：radtan8、完整 OpenCV fisheye；
- 4K 角点覆盖范围：x={min_x:.1f}～{max_x:.1f} px
  （宽度坐标的 {min_x_fraction:.1%}～{max_x_fraction:.1%}），
  y={min_y:.1f}～{max_y:.1f} px
  （高度坐标的 {min_y_fraction:.1%}～{max_y_fraction:.1%}）。

畸变系数始终保持不变。fisheye 的 `alpha` 是无量纲量，也保持不变。

## 2. 两种候选规则

`direct_scale`：

$$
f'_x=s_xf_x,\quad f'_y=s_yf_y,\quad
c'_x=s_xc_x,\quad c'_y=s_yc_y.
$$

`opencv_half_pixel`：

$$
f'_x=s_xf_x,\quad f'_y=s_yf_y,
$$

$$
c'_x=s_x(c_x+0.5)-0.5,\quad
c'_y=s_y(c_y+0.5)-0.5.
$$

后者与 OpenCV resize 的像素中心采样约定一致。本实验同时测试二者，不预设结论。

## 3. 结果

| 模型 | 规则 | 共同角点 | 代数自检最大差/px | 角点偏差 x/px | 角点偏差 y/px | 角点映射 RMS/px | 角点映射 P95/px | 固定 4K pose 的 1080p 重投影 RMS/px | 最差帧 RMS/px | 1080p pose 重估后共同角点 RMS/px |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
{rows}

推荐规则：

{recommended}

### 推荐的 1920×1080 参数

{parameters}

工程检查要求 {sample_count}/{sample_count} 帧检测、4K pose 和 1080p pose 均成功，
代数自检最大差小于 $10^{{-9}}$ px、真实角点映射 P95 小于 0.5 px，并且固定 4K pose
的 1080p 重投影 RMS 小于 0.5 px。这些门限是本实验的工程判据，不是相机精度标准。

解析投影检查使用相同三维射线，分别通过 4K 和缩放后的相机模型投影。它是由候选公式
构造出的代数自检，只能检查实现是否自洽，不能在两种规则间作选择；规则选择来自真实
角点映射和固定 pose 残差。角点映射只由 resize 与检测结果决定，因此两个相机模型得到
相同数值是预期现象，不代表二者标定质量相同。

真实图像检查在 1080p 图像上重新运行 Kalibr AprilGrid 检测。固定 pose 指标先由 4K
角点估计标定板位姿，再直接用缩放后的内参投影到 1080p，没有让低分辨率 pose 吸收
内参转换误差。1080p pose 使用全部低分辨率角点拟合，但表中的重估残差只在与 4K
共有的角点上统计，因而可与固定 pose 指标按相同点集比较。

## 4. 判定边界

这 20 帧纯 OpenCV resize 实验支持：按推荐规则缩放的内参可以直接用于这些
1920×1080 图像。样本来自既有标定数据而非独立留出集，角点也没有覆盖最右侧约 21%
的视场；因此本实验不是相机全视场精度认证，也不独立验证 4K 源标定的绝对准确度。
两套源标定的配置和可观性并不完全相同，本报告也不能用于比较 radtan8 与 fisheye
哪个模型更好。

若实际 1080p 图像来自 ISP 的裁剪、binning、数字防抖、畸变校正、不同 readout ROI
或不同对焦状态，不能套用本结论，必须使用实际输出模式重新验证或重新标定。

完整聚合指标见 `summary.yaml`，每个角点的明细见 `corner_residuals.csv`，叠加图中红点
为 1080p 实测角点，绿色十字为使用 4K pose 和缩放内参得到的预测。
""".format(
        dataset=summary["dataset"], topic=summary["topic"],
        source_width=summary["source_resolution"][0],
        source_height=summary["source_resolution"][1],
        target_width=summary["target_resolution"][0],
        target_height=summary["target_resolution"][1],
        sample_count=summary["sample_count"],
        min_x=coverage_px["min_x"], max_x=coverage_px["max_x"],
        min_y=coverage_px["min_y"], max_y=coverage_px["max_y"],
        min_x_fraction=coverage_fraction["min_x"],
        max_x_fraction=coverage_fraction["max_x"],
        min_y_fraction=coverage_fraction["min_y"],
        max_y_fraction=coverage_fraction["max_y"],
        rows="\n".join(rows),
        recommended="\n".join(recommended),
        parameters="\n".join(parameter_lines).rstrip())
    path.write_text(text, encoding="utf-8")


def main():
    arguments = parse_arguments()
    if arguments.camera_index < 0:
        raise RuntimeError("--camera-index must be non-negative")
    if arguments.target_width <= 0 or arguments.target_height <= 0:
        raise RuntimeError("target dimensions must be positive")
    if arguments.window_half_size_px < 1:
        raise RuntimeError("--window-half-size-px must be positive")
    if (arguments.max_displacement_px <= 0.0
            or not math.isfinite(arguments.max_displacement_px)):
        raise RuntimeError("--max-displacement-px must be positive and finite")

    manifest_path, manifest = load_mapping(arguments.manifest)
    target_value = arguments.target or manifest.get("target")
    if target_value is None:
        raise RuntimeError("target path is absent from arguments and manifest")
    target_path, target_document = load_mapping(target_value)
    specs = {
        "radtan8": load_camera_spec(
            arguments.radtan8_calibration, arguments.camera_index, "radtan8"),
        "fisheye": load_camera_spec(
            arguments.fisheye_calibration, arguments.camera_index, "fisheye"),
    }
    source_resolutions = {spec["resolution"] for spec in specs.values()}
    if len(source_resolutions) != 1:
        raise RuntimeError("source calibrations have different resolutions")
    source_size = source_resolutions.pop()
    target_size = (arguments.target_width, arguments.target_height)

    output_dir = arguments.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    visualization_dir = output_dir / "overlays"
    visualization_dir.mkdir()

    import aslam_cameras_april as acv_april
    import aslam_cv as acv
    import kalibr_opencv_fisheye_full
    import kalibr_radtan8
    from kalibr_bag_io import open_dataset
    from kalibr_common.ConfigReader import AslamCamera

    kalibr_radtan8.install()
    kalibr_opencv_fisheye_full.install()
    target = create_target(
        acv_april, target_document, arguments.window_half_size_px,
        arguments.max_displacement_px)

    high_geometries = {}
    candidates = {}
    accumulators = {}
    for model_name, spec in specs.items():
        high_geometries[model_name] = create_geometry(
            AslamCamera, spec, source_size, spec["intrinsics"])
        for convention in CONVENTIONS:
            scaled = scale_intrinsics(
                spec["intrinsics"], source_size, target_size, convention)
            geometry = create_geometry(
                AslamCamera, spec, target_size, scaled)
            key = (model_name, convention)
            candidates[key] = {
                "intrinsics": scaled,
                "geometry": geometry,
                "detector": create_detector(acv, geometry, target),
            }
            accumulators[key] = {
                "projection": [projection_equivalence(
                    high_geometries[model_name], geometry, source_size,
                    target_size, convention)],
                "mapping": [],
                "mapping_frames": [],
                "high_scaled": [],
                "low_fixed": [],
                "low_fixed_frames": [],
                "low_refit_all": [],
                "low_refit_common": [],
                "low_refit_common_frames": [],
                "fixed_delta": [],
                "detected_frames": 0,
                "high_pose_frames": 0,
                "low_pose_frames": 0,
            }

    pairs = manifest.get("pairs")
    if not isinstance(pairs, list) or not pairs:
        raise RuntimeError("manifest contains no sampled pairs")
    topic = manifest["topics"][arguments.camera_index]
    timestamp_key = "cam{}_timestamp_ns".format(arguments.camera_index)
    detection_key = "cam{}_detection".format(arguments.camera_index)
    wanted_timestamps = {int(pair[timestamp_key]) for pair in pairs}
    visual_indices = set(
        np.rint(np.linspace(
            0, len(pairs) - 1,
            min(arguments.visualization_samples, len(pairs)))).astype(int).tolist()
    ) if arguments.visualization_samples > 0 else set()

    residual_rows = []
    source_observed_pixels = []
    reader = open_dataset(manifest["dataset"])
    with reader.index_images(topic, grayscale=True) as dataset:
        entries = {
            int(entry.header_timestamp_ns): entry for entry in dataset.index
            if int(entry.header_timestamp_ns) in wanted_timestamps
        }
        missing = sorted(wanted_timestamps - set(entries))
        if missing:
            raise RuntimeError(
                "{} manifest timestamps are absent from the bag".format(
                    len(missing)))

        for pair_index, pair in enumerate(pairs):
            sample_id = int(pair.get("sample", pair_index))
            timestamp_ns = int(pair[timestamp_key])
            high_image = np.asarray(
                dataset.get_by_entry(entries[timestamp_ns]).image)
            if high_image.ndim == 3:
                high_image = cv2.cvtColor(high_image, cv2.COLOR_BGR2GRAY)
            if high_image.shape[::-1] != source_size:
                raise RuntimeError(
                    "source image {} has resolution {}, expected {}".format(
                        timestamp_ns, high_image.shape[::-1], source_size))
            low_image = cv2.resize(
                high_image, target_size, interpolation=cv2.INTER_AREA)
            high_observation = observation_from_manifest(
                acv, target, high_image, timestamp_ns,
                pair[detection_key]["corners"])
            high_pixels = observation_pixels(high_observation)
            source_observed_pixels.append(np.asarray(
                list(high_pixels.values()), dtype=np.float64))

            high_state = {}
            for model_name, high_geometry in high_geometries.items():
                pose = estimate_pose(high_geometry, high_observation)
                if pose is None:
                    continue
                residuals, _ = reprojection_residuals(
                    high_geometry, target, high_pixels, pose)
                high_state[model_name] = (pose, residuals)

            for model_name in specs:
                for convention in CONVENTIONS:
                    key = (model_name, convention)
                    candidate = candidates[key]
                    accumulator = accumulators[key]
                    success, low_observation = candidate["detector"].findTargetNoTransformation(
                        acv.Time(float(timestamp_ns) * 1.0e-9), low_image)
                    if not success:
                        continue
                    accumulator["detected_frames"] += 1
                    low_pixels = observation_pixels(low_observation)
                    common = sorted(set(high_pixels) & set(low_pixels))
                    if not common:
                        continue

                    high_array = np.asarray([high_pixels[index]
                                             for index in common])
                    low_array = np.asarray([low_pixels[index]
                                            for index in common])
                    mapping = low_array - transform_pixels(
                        high_array, source_size, target_size, convention)
                    accumulator["mapping"].append(mapping)
                    accumulator["mapping_frames"].append({
                        "sample": sample_id,
                        "timestamp_ns": timestamp_ns,
                        "residuals": mapping,
                    })

                    low_refit = {}
                    low_refit_pose = estimate_pose(
                        candidate["geometry"], low_observation)
                    if low_refit_pose is not None:
                        accumulator["low_pose_frames"] += 1
                        low_refit_all, _ = reprojection_residuals(
                            candidate["geometry"], target, low_pixels,
                            low_refit_pose)
                        low_refit, _ = reprojection_residuals(
                            candidate["geometry"], target, low_pixels,
                            low_refit_pose, common)
                        accumulator["low_refit_all"].append(
                            np.asarray(list(low_refit_all.values())))

                    fixed_predictions = {}
                    if model_name in high_state:
                        high_pose, high_residual_map = high_state[model_name]
                        accumulator["high_pose_frames"] += 1
                        low_fixed, fixed_predictions = reprojection_residuals(
                            candidate["geometry"], target, low_pixels,
                            high_pose, common)
                        fixed_common = sorted(
                            set(common) & set(low_fixed) & set(high_residual_map))
                        scale = np.asarray([
                            target_size[0] / float(source_size[0]),
                            target_size[1] / float(source_size[1]),
                        ])
                        high_scaled = np.asarray([
                            high_residual_map[index] * scale
                            for index in fixed_common])
                        fixed_values = np.asarray([
                            low_fixed[index] for index in fixed_common])
                        accumulator["high_scaled"].append(high_scaled)
                        accumulator["low_fixed"].append(fixed_values)
                        accumulator["low_fixed_frames"].append({
                            "sample": sample_id,
                            "timestamp_ns": timestamp_ns,
                            "residuals": fixed_values,
                        })
                        refit_fixed_common = [
                            index for index in fixed_common
                            if index in low_refit]
                        refit_common_values = np.asarray([
                            low_refit[index] for index in refit_fixed_common])
                        accumulator["low_refit_common"].append(
                            refit_common_values)
                        accumulator["low_refit_common_frames"].append({
                            "sample": sample_id,
                            "timestamp_ns": timestamp_ns,
                            "residuals": refit_common_values,
                        })
                        accumulator["fixed_delta"].append(
                            fixed_values - high_scaled)

                        mapping_by_index = dict(zip(common, mapping))
                        for index, map_error, high_error, low_error in zip(
                                fixed_common,
                                [mapping_by_index[item]
                                 for item in fixed_common],
                                high_scaled, fixed_values):
                            refit_error = low_refit.get(index)
                            residual_rows.append({
                                "sample": sample_id,
                                "timestamp_ns": timestamp_ns,
                                "model": model_name,
                                "convention": convention,
                                "corner_index": index,
                                "mapping_dx_px": map_error[0],
                                "mapping_dy_px": map_error[1],
                                "high_reprojection_scaled_dx_px": high_error[0],
                                "high_reprojection_scaled_dy_px": high_error[1],
                                "low_fixed_pose_dx_px": low_error[0],
                                "low_fixed_pose_dy_px": low_error[1],
                                "low_refit_pose_dx_px": (
                                    None if refit_error is None
                                    else refit_error[0]),
                                "low_refit_pose_dy_px": (
                                    None if refit_error is None
                                    else refit_error[1]),
                            })

                    if pair_index in visual_indices and fixed_predictions:
                        model_dir = visualization_dir / model_name / convention
                        model_dir.mkdir(parents=True, exist_ok=True)
                        overlay = draw_overlay(
                            low_image, low_pixels, fixed_predictions,
                            "{} {} sample {}".format(
                                model_name, convention,
                                sample_id))
                        output_path = model_dir / "sample_{:02d}.jpg".format(
                            sample_id)
                        if not cv2.imwrite(
                                str(output_path), overlay,
                                [cv2.IMWRITE_JPEG_QUALITY, 95]):
                            raise RuntimeError(
                                "could not write {}".format(output_path))

    summary = {
        "schema_version": 1,
        "kind": "camera_intrinsics_resize_validation",
        "manifest": str(manifest_path),
        "target": str(target_path),
        "dataset": str(Path(manifest["dataset"]).resolve()),
        "camera_index": arguments.camera_index,
        "topic": topic,
        "source_resolution": list(source_size),
        "target_resolution": list(target_size),
        "resize_interpolation": "cv2.INTER_AREA",
        "sample_count": len(pairs),
        "source_corner_coverage": observed_pixel_coverage(
            source_observed_pixels, source_size),
        "engineering_check_thresholds": dict(
            ENGINEERING_CHECK_THRESHOLDS),
        "models": {},
    }
    for model_name, spec in specs.items():
        model_summary = {
            "source_calibration": spec["path"],
            "source_intrinsics": spec["intrinsics"].tolist(),
            "distortion_coefficients_unchanged": spec["distortion"].tolist(),
            "candidates": {},
        }
        for convention in CONVENTIONS:
            key = (model_name, convention)
            accumulator = accumulators[key]
            candidate_summary = {
                "scaled_intrinsics": candidates[key]["intrinsics"].tolist(),
                "detected_frames": accumulator["detected_frames"],
                "high_pose_frames": accumulator["high_pose_frames"],
                "low_pose_frames": accumulator["low_pose_frames"],
                "analytic_projection_equivalence": vector_statistics(
                    accumulator["projection"]),
                "observed_corner_mapping": vector_statistics(
                    accumulator["mapping"]),
                "observed_corner_mapping_per_frame": per_frame_statistics(
                    accumulator["mapping_frames"]),
                "source_reprojection_scaled_to_target": vector_statistics(
                    accumulator["high_scaled"]),
                "low_resolution_fixed_pose_reprojection": vector_statistics(
                    accumulator["low_fixed"]),
                "low_resolution_fixed_pose_per_frame": per_frame_statistics(
                    accumulator["low_fixed_frames"]),
                "low_resolution_refit_pose_common_reprojection":
                    vector_statistics(accumulator["low_refit_common"]),
                "low_resolution_refit_pose_common_per_frame":
                    per_frame_statistics(
                        accumulator["low_refit_common_frames"]),
                "low_resolution_refit_pose_all_reprojection":
                    vector_statistics(accumulator["low_refit_all"]),
                "fixed_minus_scaled_source_reprojection": vector_statistics(
                    accumulator["fixed_delta"]),
            }
            candidate_summary["passes_engineering_checks"] = bool(
                candidate_summary["detected_frames"] >= len(pairs)
                    * ENGINEERING_CHECK_THRESHOLDS[
                        "required_detected_frame_fraction"]
                and candidate_summary["high_pose_frames"] >= len(pairs)
                    * ENGINEERING_CHECK_THRESHOLDS[
                        "required_detected_frame_fraction"]
                and candidate_summary["low_pose_frames"] >= len(pairs)
                    * ENGINEERING_CHECK_THRESHOLDS[
                        "required_detected_frame_fraction"]
                and candidate_summary["analytic_projection_equivalence"]
                    ["max_norm_px"] < ENGINEERING_CHECK_THRESHOLDS[
                        "analytic_projection_max_px"]
                and candidate_summary["observed_corner_mapping"]
                    ["p95_norm_px"] < ENGINEERING_CHECK_THRESHOLDS[
                        "observed_corner_mapping_p95_px"]
                and candidate_summary["low_resolution_fixed_pose_reprojection"]
                    ["rms_norm_px"] < ENGINEERING_CHECK_THRESHOLDS[
                        "fixed_pose_reprojection_rms_px"])
            model_summary["candidates"][convention] = candidate_summary
        model_summary["recommended_convention"] = min(
            CONVENTIONS,
            key=lambda convention: model_summary["candidates"][convention]
                ["observed_corner_mapping"]["rms_norm_px"])
        summary["models"][model_name] = model_summary

    with (output_dir / "summary.yaml").open(
            "w", encoding="utf-8") as stream:
        yaml.safe_dump(
            summary, stream, allow_unicode=True, sort_keys=False,
            default_flow_style=False)
    with (output_dir / "recommended_parameters.yaml").open(
            "w", encoding="utf-8") as stream:
        yaml.safe_dump(
            recommended_parameter_document(summary), stream,
            allow_unicode=True, sort_keys=False, default_flow_style=False)
    with (output_dir / "corner_residuals.csv").open(
            "w", encoding="utf-8", newline="") as stream:
        fieldnames = list(residual_rows[0]) if residual_rows else []
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        if fieldnames:
            writer.writeheader()
            writer.writerows(residual_rows)
    write_report(output_dir / "report.md", summary)
    print("summary: {}".format(output_dir / "summary.yaml"))
    print("report: {}".format(output_dir / "report.md"))
    for model_name, model in summary["models"].items():
        print("{} recommended: {}".format(
            model_name, model["recommended_convention"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
