"""Generate calibration-repeatability and fixed-parameter validation reports."""

from __future__ import annotations

import base64
import csv
import html
import json
import math
import shutil
import statistics
import tempfile
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib import font_manager
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties

from .workflow import _write_json, machine_path


METRICS = (
    ("cam0_rms_px", "cam0 RMS [px]"),
    ("cam1_rms_px", "cam1 RMS [px]"),
    ("stereo_rms_px", "双目综合 RMS [px]"),
    ("alignment_rms_px", "Alignment RMS [px]"),
)


def _font() -> FontProperties:
    for candidate in (
        "/mnt/c/Windows/Fonts/msyh.ttc",
        "/mnt/c/Windows/Fonts/simhei.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    ):
        if Path(candidate).is_file():
            font_manager.fontManager.addfont(candidate)
            result = FontProperties(fname=candidate)
            plt.rcParams["font.family"] = result.get_name()
            plt.rcParams["axes.unicode_minus"] = False
            return result
    return FontProperties()


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _weighted_rms(values: list[tuple[int, float]]) -> float:
    count = sum(item[0] for item in values)
    return math.sqrt(sum(count_i * value * value for count_i, value in values) / count) if count else math.nan


def _camera(document: dict[str, Any], camera_id: str) -> dict[str, Any]:
    return next(camera for camera in document["cameras"] if camera["id"] == camera_id)


def _observability(path: Path) -> tuple[str, str]:
    if not path.is_file():
        return "unavailable", "unavailable"
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    calibration = value.get("calibration", {})
    return str(value.get("status", calibration.get("status", "unavailable"))), str(value.get("quality", calibration.get("quality", "unavailable")))


def collect_calibrations(manifest: dict[str, Any], phases: tuple[str, ...]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    summary = []
    parameters = []
    for phase_name in phases:
        for run in manifest["phases"][phase_name]["calibration_runs"]:
            result_path = machine_path(manifest, run["result"])
            artifacts = machine_path(manifest, run["artifacts"])
            document = yaml.safe_load(result_path.read_text(encoding="utf-8"))
            metrics = _load_json(artifacts / "metrics.json")
            cam_metrics = metrics["cameras"]
            cam0_count = int(cam_metrics["cam0"]["reprojection"]["count"])
            cam1_count = int(cam_metrics["cam1"]["reprojection"]["count"])
            cam0_rms = float(cam_metrics["cam0"]["reprojection"]["rms_px"])
            cam1_rms = float(cam_metrics["cam1"]["reprojection"]["rms_px"])
            pair = metrics.get("stereo_pairs", {}).get("cam0_cam1", {})
            alignment = pair.get("alignment", {})
            cam1 = _camera(document, "cam1")
            transform = np.asarray(cam1["T_cn_cnm1"], dtype=float)
            status, quality = _observability(artifacts / "observability.yaml")
            summary.append({
                "phase": phase_name,
                "repeat": int(run["repeat"]),
                "dataset_id": run["dataset_id"],
                "cam0_rms_px": cam0_rms,
                "cam1_rms_px": cam1_rms,
                "stereo_rms_px": _weighted_rms([(cam0_count, cam0_rms), (cam1_count, cam1_rms)]),
                "alignment_rms_px": float(alignment.get("rms_px", cam1.get("alignment", math.nan))),
                "cam0_used_frames": int(cam_metrics["cam0"]["used_frames"]),
                "cam1_used_frames": int(cam_metrics["cam1"]["used_frames"]),
                "baseline_mm": float(np.linalg.norm(transform[:3, 3]) * 1000.0),
                "observability_status": status,
                "observability_quality": quality,
                "result": str(result_path),
            })
            for camera_id in ("cam0", "cam1"):
                camera = _camera(document, camera_id)
                for name, value in zip(("fx", "fy", "cx", "cy"), camera["intrinsics"]):
                    parameters.append({"phase": phase_name, "repeat": run["repeat"], "camera": camera_id, "parameter": name, "value": float(value)})
                for index, value in enumerate(camera["distortion_coeffs"], 1):
                    parameters.append({"phase": phase_name, "repeat": run["repeat"], "camera": camera_id, "parameter": f"d{index}", "value": float(value)})
            for row, axis in enumerate(("x", "y", "z")):
                parameters.append({"phase": phase_name, "repeat": run["repeat"], "camera": "cam0_cam1", "parameter": "t" + axis + "_m", "value": float(transform[row, 3])})
            for row in range(3):
                for column in range(3):
                    parameters.append({"phase": phase_name, "repeat": run["repeat"], "camera": "cam0_cam1", "parameter": f"r{row}{column}", "value": float(transform[row, column])})
    return summary, parameters


def _result_for_label(document: dict[str, Any], label: str) -> dict[str, Any]:
    results = document["results"]
    if label in results:
        return results[label]
    if len(results) == 1:
        return next(iter(results.values()))
    raise KeyError(f"calibration label is absent: {label}")


def collect_verifications(manifest: dict[str, Any], groups: tuple[tuple[str, str], ...]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    label = manifest["kalibr"]["calibration_label"]
    summary = []
    details = []
    method = None
    for test_phase, calibration_phase in groups:
        group = f"{test_phase}_tests_using_{calibration_phase}_calibration"
        for run in manifest["phases"][test_phase]["test_runs"]:
            output = machine_path(manifest, run["results"][calibration_phase])
            document = _load_json(output / "fixed_camera_validation.json")
            current_method = json.dumps(document["method"], sort_keys=True, separators=(",", ":"))
            if method is None:
                method = current_method
            elif current_method != method:
                raise ValueError(f"verification method differs: {output}")
            result = _result_for_label(document, label)
            summary.append({
                "group": group,
                "test_phase": test_phase,
                "calibration_phase": calibration_phase,
                "repeat": int(run["repeat"]),
                "dataset_id": run["dataset_id"],
                "cam0_rms_px": float(result["cameras"]["cam0"]["rms_px"]),
                "cam1_rms_px": float(result["cameras"]["cam1"]["rms_px"]),
                "stereo_rms_px": float(result["stereo_combined_rms_px"]),
                "alignment_rms_px": float(result["stereo"]["alignment"]["rms_px"]),
                "cam0_frames": int(result["cameras"]["cam0"]["detected_frames"]),
                "cam1_frames": int(result["cameras"]["cam1"]["detected_frames"]),
                "stereo_pairs": int(result["stereo"]["used_pairs"]),
                "result_directory": str(output),
            })
            with (output / "fixed_camera_validation.csv").open(encoding="utf-8-sig", newline="") as stream:
                for row in csv.DictReader(stream):
                    details.append({"group": group, "repeat": run["repeat"], "dataset_id": run["dataset_id"], **row})
    return summary, details


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = list(rows[0]) if rows else []
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _html_table(rows: list[dict[str, Any]], columns: list[tuple[str, str]]) -> str:
    head = "".join(f"<th>{html.escape(title)}</th>" for _, title in columns)
    body = []
    for row in rows:
        cells = []
        for key, _ in columns:
            value = row.get(key, "")
            if isinstance(value, float):
                value = f"{value:.9f}"
            cells.append(f"<td>{html.escape(str(value))}</td>")
        body.append("<tr>" + "".join(cells) + "</tr>")
    return f"<table><thead><tr>{head}</tr></thead><tbody>{''.join(body)}</tbody></table>"


def _stats(rows: list[dict[str, Any]], key: str) -> tuple[float, float, float]:
    values = [float(row[key]) for row in rows]
    return statistics.mean(values), statistics.stdev(values), max(values) - min(values)


def _rotation_angle_degrees(rotation: np.ndarray) -> float:
    cosine = max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def build_parameter_comparison(
    calibrations: list[dict[str, Any]], parameters: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Compare the representative and three-repeat before/after parameters."""
    values = {
        (row["phase"], int(row["repeat"]), row["camera"], row["parameter"]): float(row["value"])
        for row in parameters
    }
    for row in calibrations:
        values[(row["phase"], int(row["repeat"]), "cam0_cam1", "baseline_mm")] = float(row["baseline_mm"])

    specs = []
    for camera in ("cam0", "cam1"):
        specs.extend((camera, name, name, "px", 1.0, True) for name in ("fx", "fy", "cx", "cy"))
        specs.extend((camera, name, name, "1", 1.0, False) for name in ("d1", "d2", "d3", "d4"))
    specs.extend(("cam0_cam1", "t" + axis + "_m", "t" + axis, "mm", 1000.0, False) for axis in "xyz")
    specs.extend(
        ("cam0_cam1", f"r{row}{column}", f"r{row}{column}", "1", 1.0, False)
        for row in range(3)
        for column in range(3)
    )
    specs.append(("cam0_cam1", "baseline_mm", "baseline", "mm", 1.0, True))

    comparison = []
    for block, source_name, display_name, unit, scale, show_percent in specs:
        before = [values[("before", repeat, block, source_name)] * scale for repeat in (1, 2, 3)]
        after = [values[("after", repeat, block, source_name)] * scale for repeat in (1, 2, 3)]
        before_mean, after_mean = statistics.mean(before), statistics.mean(after)
        representative_delta = after[0] - before[0]
        mean_delta = after_mean - before_mean
        comparison.append({
            "block": block,
            "parameter": display_name,
            "unit": unit,
            "before_representative": before[0],
            "after_representative": after[0],
            "representative_delta": representative_delta,
            "representative_delta_percent": representative_delta / before[0] * 100.0 if show_percent and before[0] else "",
            "before_mean": before_mean,
            "before_sample_std": statistics.stdev(before),
            "before_range": max(before) - min(before),
            "after_mean": after_mean,
            "after_sample_std": statistics.stdev(after),
            "after_range": max(after) - min(after),
            "mean_delta": mean_delta,
            "mean_delta_percent": mean_delta / before_mean * 100.0 if show_percent and before_mean else "",
        })

    def matrix(phase: str) -> np.ndarray:
        return np.asarray(
            [[values[(phase, 1, "cam0_cam1", f"r{row}{column}")] for column in range(3)] for row in range(3)],
            dtype=float,
        )

    def translation(phase: str) -> np.ndarray:
        return np.asarray([values[(phase, 1, "cam0_cam1", "t" + axis + "_m")] for axis in "xyz"], dtype=float)

    before_rotation, after_rotation = matrix("before"), matrix("after")
    before_translation, after_translation = translation("before"), translation("after")
    before_baseline = -before_rotation.T @ before_translation
    after_baseline = -after_rotation.T @ after_translation
    direction_cosine = float(
        np.dot(before_baseline, after_baseline)
        / (np.linalg.norm(before_baseline) * np.linalg.norm(after_baseline))
    )
    derived = (
        ("relative_rotation_angle", "deg", _rotation_angle_degrees(after_rotation @ before_rotation.T)),
        ("translation_vector_change", "mm", float(np.linalg.norm(after_baseline - before_baseline) * 1000.0)),
        ("baseline_direction_angle", "deg", math.degrees(math.acos(max(-1.0, min(1.0, direction_cosine))))),
    )
    for parameter, unit, change in derived:
        comparison.append({
            "block": "cam0_cam1",
            "parameter": parameter,
            "unit": unit,
            "before_representative": "",
            "after_representative": "",
            "representative_delta": change,
            "representative_delta_percent": "",
            "before_mean": "",
            "before_sample_std": "",
            "before_range": "",
            "after_mean": "",
            "after_sample_std": "",
            "after_range": "",
            "mean_delta": "",
            "mean_delta_percent": "",
        })
    return comparison


def _calibration_figure(rows: list[dict[str, Any]], font: FontProperties) -> plt.Figure:
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    phases = sorted({row["phase"] for row in rows})
    for axis, (key, label) in zip(axes.flat, METRICS):
        for index, phase in enumerate(phases):
            values = [row[key] for row in rows if row["phase"] == phase]
            axis.plot(range(1, len(values) + 1), values, "o-", label=phase if index == 0 else phase)
        axis.set_title(label, fontproperties=font)
        axis.set_xlabel("重复次数", fontproperties=font)
        axis.set_xticks((1, 2, 3))
        axis.grid(alpha=0.25)
    axes.flat[0].legend()
    figure.suptitle("三次标定结果重复性", fontproperties=font, fontsize=15)
    return figure


def _verification_figure(rows: list[dict[str, Any]], font: FontProperties) -> plt.Figure:
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    groups = list(dict.fromkeys(row["group"] for row in rows))
    for axis, (key, label) in zip(axes.flat, METRICS):
        for group in groups:
            values = [row[key] for row in rows if row["group"] == group]
            axis.plot(range(1, len(values) + 1), values, "o-", label=group)
        axis.set_title(label, fontproperties=font)
        axis.set_xlabel("测试重复次数", fontproperties=font)
        axis.set_xticks((1, 2, 3))
        axis.grid(alpha=0.25)
    axes.flat[0].legend(fontsize=7)
    figure.suptitle("固定参数验证重复性", fontproperties=font, fontsize=15)
    return figure


def _parameter_figure(rows: list[dict[str, Any]], font: FontProperties) -> plt.Figure:
    figure, axes = plt.subplots(3, 1, figsize=(13, 11), constrained_layout=True)
    for axis, camera in zip(axes, ("cam0", "cam1", "cam0_cam1")):
        selected = [row for row in rows if row["camera"] == camera]
        labels = list(dict.fromkeys(row["parameter"] for row in selected))
        x = np.arange(len(labels))
        reference = {row["parameter"]: row["value"] for row in selected if row["repeat"] == 1 and row["phase"] == "before"}
        for phase in dict.fromkeys(row["phase"] for row in selected):
            for repeat in (1, 2, 3):
                current = {row["parameter"]: row["value"] for row in selected if row["repeat"] == repeat and row["phase"] == phase}
                axis.plot(x, [current[name] - reference[name] for name in labels], "o-", label=f"{phase} R{repeat}")
        axis.set_xticks(x, labels)
        axis.set_title(f"{camera} 相对 before repeat 1 的参数差", fontproperties=font)
        axis.grid(alpha=0.25)
        axis.legend()
    return figure


def _detail_figure(rows: list[dict[str, Any]], font: FontProperties) -> plt.Figure:
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    definitions = (("cam0", "cam0 每帧 RMS", "reprojection_rms_px"), ("cam1", "cam1 每帧 RMS", "reprojection_rms_px"), (None, "双目每图对 RMS", "reprojection_rms_px"), (None, "每图对 Alignment RMS", "alignment_rms_px"))
    for axis, (camera, title, field) in zip(axes.flat, definitions):
        for group in dict.fromkeys(row["group"] for row in rows):
            selected = []
            for row in rows:
                if row["group"] != group or row.get(field) in (None, ""):
                    continue
                if camera is not None and not (row.get("record_type") == "camera_frame" and row.get("camera_id") == camera):
                    continue
                if camera is None and row.get("record_type") != "stereo_pair":
                    continue
                selected.append(float(row[field]))
            axis.plot(range(1, len(selected) + 1), selected, linewidth=1, marker=".", label=group)
        axis.set_title(title, fontproperties=font)
        axis.grid(alpha=0.2)
    axes.flat[0].legend(fontsize=7)
    figure.suptitle("逐帧与逐双目图对误差", fontproperties=font, fontsize=15)
    return figure


def _managed_replace(destination: Path, stage: Path, force: bool) -> None:
    if not destination.exists():
        stage.rename(destination)
        return
    manifest = destination / "report_manifest.json"
    inventory = destination / ".inventory.json"
    if not force:
        raise FileExistsError(f"report already exists: {destination}")
    if not manifest.is_file() or not inventory.is_file():
        raise RuntimeError(f"refusing to replace unmanaged report directory: {destination}")
    listed = set(_load_json(inventory))
    actual = {path.relative_to(destination).as_posix() for path in destination.rglob("*") if path.is_file()}
    if actual != listed:
        raise RuntimeError(f"refusing to replace report directory with unregistered files: {destination}")
    shutil.rmtree(destination)
    stage.rename(destination)


def generate_report(manifest: dict[str, Any], full: bool = False, force: bool = False) -> dict[str, Any]:
    phases = ("before", "after") if full else ("before",)
    if full and "after" not in manifest["phases"]:
        raise ValueError("after phase is absent")
    groups = (("before", "before"), ("after", "before"), ("after", "after")) if full else (("before", "before"),)
    calibrations, parameters = collect_calibrations(manifest, phases)
    verifications, details = collect_verifications(manifest, groups)
    parameter_comparison = build_parameter_comparison(calibrations, parameters) if full else []
    destination = machine_path(manifest, manifest["reports"]["comparison" if full else "before"])
    existing_manifest = destination / "report_manifest.json"
    if existing_manifest.is_file() and not force:
        stored = _load_json(existing_manifest)
        return {"output": str(destination), "html": str(destination / stored["html"]), "pdf": str(destination / stored["pdf"]), "reused": True}
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".aging-report-", dir=destination.parent))
    charts = stage / "charts"
    charts.mkdir()
    font = _font()
    figures = [
        ("calibration_repeatability.png", _calibration_figure(calibrations, font)),
        ("verification_repeatability.png", _verification_figure(verifications, font)),
        ("parameter_deviations.png", _parameter_figure(parameters, font)),
        ("per_frame_pair_errors.png", _detail_figure(details, font)),
    ]
    encoded = {}
    try:
        for name, figure in figures:
            path = charts / name
            figure.savefig(path, dpi=170, bbox_inches="tight")
            encoded[name] = base64.b64encode(path.read_bytes()).decode("ascii")
        _write_csv(stage / "calibration_repeatability.csv", calibrations)
        _write_csv(stage / "calibration_parameters.csv", parameters)
        _write_csv(stage / "verification_summary.csv", verifications)
        _write_csv(stage / "verification_details.csv", details)
        if full:
            _write_csv(stage / "parameter_before_after_comparison.csv", parameter_comparison)
        calibration_stats = []
        for phase in phases:
            phase_rows = [row for row in calibrations if row["phase"] == phase]
            for key, label in METRICS:
                mean, std, spread = _stats(phase_rows, key)
                calibration_stats.append({"phase": phase, "metric": label, "mean": mean, "sample_std": std, "range": spread})
        verification_stats = []
        for group in dict.fromkeys(row["group"] for row in verifications):
            group_rows = [row for row in verifications if row["group"] == group]
            for key, label in METRICS:
                mean, std, spread = _stats(group_rows, key)
                verification_stats.append({"group": group, "metric": label, "mean": mean, "sample_std": std, "range": spread})
        parameter_stats = []
        for phase in phases:
            for camera in ("cam0", "cam1", "cam0_cam1"):
                names = dict.fromkeys(row["parameter"] for row in parameters if row["phase"] == phase and row["camera"] == camera)
                for name in names:
                    values = [row["value"] for row in parameters if row["phase"] == phase and row["camera"] == camera and row["parameter"] == name]
                    parameter_stats.append({"phase": phase, "camera": camera, "parameter": name, "mean": statistics.mean(values), "sample_std": statistics.stdev(values), "range": max(values) - min(values)})
        _write_csv(stage / "parameter_repeatability.csv", parameter_stats)
        images = "".join(f'<figure><img src="data:image/png;base64,{encoded[name]}" alt="{name}"></figure>' for name, _ in figures)
        title = f"{manifest['machine_id']} 双目老化前基线报告" if not full else f"{manifest['machine_id']} 双目老化前后对比报告"
        comparison_html = ""
        if full:
            comparison_html = "<h2>老化前后标定参数对比</h2>" + _html_table(parameter_comparison, [
                ('block','参数块'),('parameter','参数'),('unit','单位'),
                ('before_representative','before 代表值'),('after_representative','after 代表值'),
                ('representative_delta','代表值变化'),('representative_delta_percent','代表值变化 [%]'),
                ('before_mean','before 均值'),('before_sample_std','before 样本标准差'),('before_range','before 极差'),
                ('after_mean','after 均值'),('after_sample_std','after 样本标准差'),('after_range','after 极差'),
                ('mean_delta','均值变化'),('mean_delta_percent','均值变化 [%]'),
            ])
        document = f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><title>{html.escape(title)}</title>
<style>body{{font-family:"Microsoft YaHei","Noto Sans CJK SC",sans-serif;max-width:1500px;margin:30px auto;padding:0 24px;color:#1a202c}}h1,h2{{color:#16324f}}.note{{background:#edf2f7;border-left:5px solid #2b6cb0;padding:12px 16px}}table{{border-collapse:collapse;width:100%;font-size:12px;margin:14px 0 24px}}th,td{{border:1px solid #cbd5e0;padding:6px 8px;text-align:right}}th:first-child,td:first-child{{text-align:left}}th{{background:#e2e8f0}}img{{max-width:100%}}</style></head><body>
<h1>{html.escape(title)}</h1><p class="note">三组标定用于估计参数重复性；按时间排序的第 1 组标定参数用于三组固定参数测试。报告只给出测量结果和离散程度，不自动作生产合格判定。</p>
<h2>标定指标</h2>{_html_table(calibrations, [('phase','阶段'),('repeat','重复'),('dataset_id','数据集'),('cam0_rms_px','cam0 RMS'),('cam1_rms_px','cam1 RMS'),('stereo_rms_px','双目综合 RMS'),('alignment_rms_px','Alignment RMS'),('baseline_mm','基线 [mm]'),('cam0_used_frames','cam0 使用帧'),('cam1_used_frames','cam1 使用帧'),('observability_status','可观性状态'),('observability_quality','可观性质量')])}
<h2>标定重复性统计</h2>{_html_table(calibration_stats, [('phase','阶段'),('metric','指标'),('mean','均值'),('sample_std','样本标准差'),('range','极差')])}
<h2>参数重复性统计</h2>{_html_table(parameter_stats, [('phase','阶段'),('camera','参数块'),('parameter','参数'),('mean','均值'),('sample_std','样本标准差'),('range','极差')])}
{comparison_html}
<h2>固定参数验证</h2>{_html_table(verifications, [('group','组合'),('repeat','重复'),('dataset_id','数据集'),('cam0_rms_px','cam0 RMS'),('cam1_rms_px','cam1 RMS'),('stereo_rms_px','双目综合 RMS'),('alignment_rms_px','Alignment RMS'),('cam0_frames','cam0 帧'),('cam1_frames','cam1 帧'),('stereo_pairs','双目对')])}
<h2>验证重复性统计</h2>{_html_table(verification_stats, [('group','组合'),('metric','指标'),('mean','均值'),('sample_std','样本标准差'),('range','极差')])}
<h2>图表</h2>{images}<p>逐参数与逐帧数值见随附 CSV。</p></body></html>'''
        stem = manifest["machine_id"] + ("_before_after_comparison" if full else "_before_baseline")
        html_name, pdf_name = stem + ".html", stem + ".pdf"
        (stage / html_name).write_text(document, encoding="utf-8")
        with PdfPages(stage / pdf_name) as pdf:
            page = plt.figure(figsize=(11.69, 8.27))
            page.text(0.06, 0.92, title, fontsize=20, fontproperties=font)
            lines = ["三次标定与三次固定参数验证；无生产合格判定。", ""]
            for row in calibration_stats:
                lines.append(f"{row['phase']} {row['metric']}: {row['mean']:.6f} ± {row['sample_std']:.6f}, range {row['range']:.6f}")
            page.text(0.06, 0.84, "\n".join(lines), va="top", fontsize=10, linespacing=1.4, fontproperties=font)
            pdf.savefig(page)
            plt.close(page)
            if full:
                for start in range(0, len(parameter_comparison), 16):
                    page = plt.figure(figsize=(11.69, 8.27))
                    page.text(0.05, 0.94, "老化前后标定参数对比", fontsize=16, fontproperties=font)
                    lines = []
                    for row in parameter_comparison[start:start + 16]:
                        before = row["before_representative"]
                        after = row["after_representative"]
                        delta = row["representative_delta"]
                        if before == "":
                            lines.append(f"{row['block']}.{row['parameter']}: 变化 {delta:.9f} {row['unit']}")
                        else:
                            lines.append(
                                f"{row['block']}.{row['parameter']}: {before:.9f} → {after:.9f}, "
                                f"变化 {delta:.9f} {row['unit']}; 均值 "
                                f"{row['before_mean']:.9f}±{row['before_sample_std']:.9f} → "
                                f"{row['after_mean']:.9f}±{row['after_sample_std']:.9f}"
                            )
                    page.text(0.05, 0.88, "\n".join(lines), va="top", fontsize=7.5, linespacing=1.4, fontproperties=font)
                    pdf.savefig(page)
                    plt.close(page)
            for _, figure in figures:
                pdf.savefig(figure)
        files = [html_name, pdf_name, "calibration_repeatability.csv", "calibration_parameters.csv", "parameter_repeatability.csv", "verification_summary.csv", "verification_details.csv"]
        if full:
            files.append("parameter_before_after_comparison.csv")
        files.extend([*("charts/" + name for name, _ in figures), "report_manifest.json", ".inventory.json"])
        report_manifest = {"schema_version": "1.0.0", "kind": "stereo_aging_vibration_report", "machine_id": manifest["machine_id"], "full_before_after": full, "html": html_name, "pdf": pdf_name, "files": files[:-2]}
        _write_json(stage / "report_manifest.json", report_manifest)
        _write_json(stage / ".inventory.json", sorted(files))
        _managed_replace(destination, stage, force)
        return {"output": str(destination), "html": str(destination / html_name), "pdf": str(destination / pdf_name), "reused": False}
    except Exception:
        shutil.rmtree(stage, ignore_errors=True)
        raise
    finally:
        for _, figure in figures:
            plt.close(figure)
