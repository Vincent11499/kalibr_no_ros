"""Generate self-contained HTML/PDF/CSV reports from existing study evidence."""

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
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib import font_manager
from matplotlib.font_manager import FontProperties

from .workflow import _write_json, machine_path


GROUPS = (
    ("before_with_before", "测试前数据＋测试前参数", "before", "before"),
    ("after_with_before", "测试后数据＋测试前参数", "after", "before"),
    ("after_with_after", "测试后数据＋测试后参数", "after", "after"),
)
METRICS = (
    ("cam0_rms_px", "cam0 RMS [px]"),
    ("cam1_rms_px", "cam1 RMS [px]"),
    ("stereo_rms_px", "双目综合 RMS [px]"),
    ("alignment_rms_px", "Alignment RMS [px]"),
)


def _font() -> FontProperties:
    candidates = (
        "/mnt/c/Windows/Fonts/msyh.ttc",
        "/mnt/c/Windows/Fonts/simhei.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    )
    for candidate in candidates:
        if Path(candidate).is_file():
            font_manager.fontManager.addfont(candidate)
            prop = FontProperties(fname=candidate)
            plt.rcParams["font.family"] = prop.get_name()
            plt.rcParams["axes.unicode_minus"] = False
            return prop
    return FontProperties()


def _value(result: dict[str, Any], dotted: str) -> Any:
    current: Any = result
    for key in dotted.split("."):
        current = current[key]
    return current


def _weighted_rms(items: list[tuple[int, float]]) -> float:
    count = sum(item[0] for item in items)
    if count <= 0:
        return math.nan
    return math.sqrt(sum(n * value * value for n, value in items) / count)


def _result_for_label(document: dict[str, Any], label: str) -> dict[str, Any]:
    results = document["results"]
    if label in results:
        return results[label]
    if len(results) == 1:
        return next(iter(results.values()))
    raise KeyError(f"calibration label {label!r} is absent")


def _reconcile(result: dict[str, Any]) -> None:
    for camera_id in ("cam0", "cam1"):
        camera = result["cameras"][camera_id]
        recomputed = _weighted_rms(
            [(int(frame["corner_count"]), float(frame["rms_px"])) for frame in camera["frames"] if frame["detected"]]
        )
        if not math.isclose(recomputed, float(camera["rms_px"]), rel_tol=1e-10, abs_tol=1e-12):
            raise ValueError(f"{camera_id} frame RMS does not reconcile with summary")
    combined = _weighted_rms(
        [
            (int(result["cameras"][camera_id]["corner_count"]), float(result["cameras"][camera_id]["rms_px"]))
            for camera_id in ("cam0", "cam1")
        ]
    )
    if not math.isclose(combined, float(result["stereo_combined_rms_px"]), rel_tol=1e-10, abs_tol=1e-12):
        raise ValueError("stereo combined RMS does not reconcile with camera summaries")
    pairs = result["stereo"]["pairs"]
    alignment = _weighted_rms(
        [(int(pair["alignment"]["count"]), float(pair["alignment"]["rms_px"])) for pair in pairs]
    )
    if not math.isclose(
        alignment, float(result["stereo"]["alignment"]["rms_px"]), rel_tol=1e-10, abs_tol=1e-12
    ):
        raise ValueError("pair alignment RMS does not reconcile with stereo summary")


def collect_evidence(manifest: dict[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    label = manifest["kalibr"].get("calibration_label", "1316_equi")
    summary: list[dict[str, Any]] = []
    details: list[dict[str, Any]] = []
    method_reference: str | None = None
    for group_id, group_label, test_phase, calibration_phase in GROUPS:
        runs = manifest["phases"][test_phase]["validation_runs"]
        for run in runs:
            result_dir = machine_path(manifest, run["results"][calibration_phase])
            document = json.loads((result_dir / "fixed_camera_validation.json").read_text(encoding="utf-8"))
            method = json.dumps(document["method"], sort_keys=True, separators=(",", ":"))
            if method_reference is None:
                method_reference = method
            elif method != method_reference:
                raise ValueError(f"verification method differs: {result_dir}")
            result = _result_for_label(document, label)
            _reconcile(result)
            row = {
                "group_id": group_id,
                "group": group_label,
                "test_phase": test_phase,
                "calibration_phase": calibration_phase,
                "repeat": int(run["repeat"]),
                "dataset_id": run["dataset_id"],
                "calibration_label": label,
                "cam0_rms_px": float(result["cameras"]["cam0"]["rms_px"]),
                "cam1_rms_px": float(result["cameras"]["cam1"]["rms_px"]),
                "stereo_rms_px": float(result["stereo_combined_rms_px"]),
                "alignment_rms_px": float(result["stereo"]["alignment"]["rms_px"]),
                "cam0_frames": int(result["cameras"]["cam0"]["detected_frames"]),
                "cam1_frames": int(result["cameras"]["cam1"]["detected_frames"]),
                "stereo_pairs": int(result["stereo"]["used_pairs"]),
                "alignment_count": int(result["stereo"]["alignment"]["count"]),
                "result_directory": str(result_dir),
            }
            summary.append(row)
            with (result_dir / "fixed_camera_validation.csv").open(encoding="utf-8-sig", newline="") as stream:
                for source in csv.DictReader(stream):
                    details.append(
                        {
                            "group_id": group_id,
                            "group": group_label,
                            "test_phase": test_phase,
                            "calibration_phase": calibration_phase,
                            "repeat": int(run["repeat"]),
                            "dataset_id": run["dataset_id"],
                            **source,
                        }
                    )
    return summary, details


def _group_stats(summary: list[dict[str, Any]]) -> list[dict[str, Any]]:
    output = []
    for group_id, label, _, _ in GROUPS:
        rows = [row for row in summary if row["group_id"] == group_id]
        item: dict[str, Any] = {"group_id": group_id, "group": label}
        for key, _ in METRICS:
            values = [float(row[key]) for row in rows]
            item[key + "_mean"] = statistics.mean(values)
            item[key + "_sample_std"] = statistics.stdev(values)
            item[key + "_min"] = min(values)
            item[key + "_max"] = max(values)
        output.append(item)
    return output


def _parameter_evidence(manifest: dict[str, Any]) -> dict[str, Any]:
    documents = {}
    observability = {}
    for phase in ("before", "after"):
        result = machine_path(manifest, manifest["phases"][phase]["calibration_result"])
        documents[phase] = yaml.safe_load(result.read_text(encoding="utf-8"))
        obs_path = machine_path(manifest, manifest["phases"][phase]["calibration_artifacts"]) / "observability.yaml"
        observability[phase] = yaml.safe_load(obs_path.read_text(encoding="utf-8")) if obs_path.is_file() else {}
    cameras = {}
    for camera_id in ("cam0", "cam1"):
        before = next(camera for camera in documents["before"]["cameras"] if camera["id"] == camera_id)
        after = next(camera for camera in documents["after"]["cameras"] if camera["id"] == camera_id)
        cameras[camera_id] = {
            "before_intrinsics": [float(value) for value in before["intrinsics"]],
            "after_intrinsics": [float(value) for value in after["intrinsics"]],
            "before_distortion": [float(value) for value in before["distortion_coeffs"]],
            "after_distortion": [float(value) for value in after["distortion_coeffs"]],
            "before_rms_px": float(before["rms"]),
            "after_rms_px": float(after["rms"]),
        }
    before_cam1 = next(camera for camera in documents["before"]["cameras"] if camera["id"] == "cam1")
    after_cam1 = next(camera for camera in documents["after"]["cameras"] if camera["id"] == "cam1")
    before_t = np.asarray(before_cam1["T_cn_cnm1"], dtype=float)
    after_t = np.asarray(after_cam1["T_cn_cnm1"], dtype=float)
    relative = after_t[:3, :3] @ before_t[:3, :3].T
    angle = math.acos(float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)))
    translation_delta = after_t[:3, 3] - before_t[:3, 3]
    baseline_before = float(np.linalg.norm(before_t[:3, 3]))
    baseline_after = float(np.linalg.norm(after_t[:3, 3]))
    return {
        "cameras": cameras,
        "T_before": before_t.tolist(),
        "T_after": after_t.tolist(),
        "translation_before_m": before_t[:3, 3].tolist(),
        "translation_after_m": after_t[:3, 3].tolist(),
        "translation_delta_m": translation_delta.tolist(),
        "translation_delta_norm_mm": float(np.linalg.norm(translation_delta) * 1000.0),
        "baseline_before_m": baseline_before,
        "baseline_after_m": baseline_after,
        "baseline_delta_mm": (baseline_after - baseline_before) * 1000.0,
        "relative_rotation_deg": math.degrees(angle),
        "calibration_alignment_before_px": float(before_cam1.get("alignment", math.nan)),
        "calibration_alignment_after_px": float(after_cam1.get("alignment", math.nan)),
        "observability": {
            phase: {
                "status": observability[phase].get("status", "unavailable"),
                "quality": observability[phase].get("quality", "unavailable"),
            }
            for phase in ("before", "after")
        },
    }


def _save_figure(figure: plt.Figure, path: Path) -> str:
    figure.savefig(path, dpi=170, bbox_inches="tight")
    return base64.b64encode(path.read_bytes()).decode("ascii")


def _summary_figure(summary: list[dict[str, Any]], font: FontProperties) -> plt.Figure:
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    colors = ("#2b6cb0", "#dd6b20", "#2f855a")
    for axis, (metric, title) in zip(axes.flat, METRICS):
        for index, ((group_id, label, _, _), color) in enumerate(zip(GROUPS, colors)):
            values = [float(row[metric]) for row in summary if row["group_id"] == group_id]
            offsets = np.linspace(-0.09, 0.09, len(values))
            axis.scatter(index + offsets, values, color=color, s=38, alpha=0.85, zorder=3)
            axis.errorbar(
                index,
                statistics.mean(values),
                yerr=statistics.stdev(values),
                fmt="D",
                color="black",
                capsize=5,
                markersize=5,
                zorder=4,
            )
        axis.set_title(title, fontproperties=font)
        axis.set_xticks(range(3), [group[1] for group in GROUPS], rotation=12, ha="right", fontproperties=font)
        axis.set_ylabel("px")
        axis.grid(axis="y", alpha=0.25)
    figure.suptitle("三次重复验证：单次值与均值 ± 样本标准差", fontproperties=font, fontsize=15)
    return figure


def _paired_figure(summary: list[dict[str, Any]], font: FontProperties) -> plt.Figure:
    figure, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
    for axis, (metric, title) in zip(axes.flat, METRICS):
        old = sorted(
            (row for row in summary if row["group_id"] == "after_with_before"), key=lambda row: row["repeat"]
        )
        new = sorted(
            (row for row in summary if row["group_id"] == "after_with_after"), key=lambda row: row["repeat"]
        )
        for old_row, new_row in zip(old, new):
            axis.plot([0, 1], [old_row[metric], new_row[metric]], marker="o", linewidth=1.8, label=f"重复 {old_row['repeat']}")
        axis.set_xticks([0, 1], ["测试前参数", "测试后参数"], fontproperties=font)
        axis.set_title(title, fontproperties=font)
        axis.set_ylabel("px")
        axis.grid(axis="y", alpha=0.25)
    axes.flat[0].legend(prop=font, fontsize=8)
    figure.suptitle("同一测试后图像：新旧标定参数成对比较", fontproperties=font, fontsize=15)
    return figure


def _detail_values(details: list[dict[str, Any]], group_id: str, metric: str) -> tuple[list[float], list[str]]:
    output: list[float] = []
    labels: list[str] = []
    rows = sorted(details, key=lambda row: (int(row["repeat"]), row.get("record_type", ""), int(row.get("source_index") or 0)))
    for row in rows:
        if row["group_id"] != group_id:
            continue
        record_type = row.get("record_type")
        camera_id = row.get("camera_id")
        field = "reprojection_rms_px"
        include = False
        if metric == "cam0" and record_type == "camera_frame" and camera_id == "cam0":
            include = True
        elif metric == "cam1" and record_type == "camera_frame" and camera_id == "cam1":
            include = True
        elif metric == "stereo" and record_type == "stereo_pair":
            include = True
        elif metric == "alignment" and record_type == "stereo_pair":
            include = True
            field = "alignment_rms_px"
        if include and row.get(field) not in (None, ""):
            output.append(float(row[field]))
            labels.append(f"R{row['repeat']}-{int(row.get('source_index') or 0) + 1}")
    return output, labels


def _detail_figure(details: list[dict[str, Any]], font: FontProperties) -> plt.Figure:
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    definitions = (("cam0", "cam0 每图 RMS"), ("cam1", "cam1 每图 RMS"), ("stereo", "每图对综合 RMS"), ("alignment", "每图对 Alignment RMS"))
    colors = ("#2b6cb0", "#dd6b20", "#2f855a")
    for axis, (metric, title) in zip(axes.flat, definitions):
        labels: list[str] = []
        for (group_id, group_label, _, _), color in zip(GROUPS, colors):
            values, current_labels = _detail_values(details, group_id, metric)
            labels = current_labels or labels
            axis.plot(range(len(values)), values, marker="o", markersize=3.5, linewidth=1.3, label=group_label, color=color)
        axis.set_title(title, fontproperties=font)
        axis.set_ylabel("px")
        axis.set_xticks(range(len(labels)), labels, rotation=60, fontsize=7)
        axis.grid(alpha=0.22)
    axes.flat[0].legend(prop=font, fontsize=7)
    figure.suptitle("逐图与逐双目图对精度", fontproperties=font, fontsize=15)
    return figure


def _parameter_figure(parameters: dict[str, Any], font: FontProperties) -> plt.Figure:
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    names = ("fx", "fy", "cx", "cy")
    x = np.arange(4)
    width = 0.35
    for camera_index, camera_id in enumerate(("cam0", "cam1")):
        camera = parameters["cameras"][camera_id]
        deltas = np.asarray(camera["after_intrinsics"]) - np.asarray(camera["before_intrinsics"])
        axes[0, 0].bar(x + (camera_index - 0.5) * width, deltas, width=width, label=camera_id)
    axes[0, 0].set_xticks(x, names)
    axes[0, 0].set_title("内参变化（测试后－测试前）", fontproperties=font)
    axes[0, 0].set_ylabel("px")
    axes[0, 0].legend()
    axes[0, 0].grid(axis="y", alpha=0.22)

    for camera_id, color in (("cam0", "#2b6cb0"), ("cam1", "#dd6b20")):
        camera = parameters["cameras"][camera_id]
        axes[0, 1].plot(range(1, 5), camera["before_distortion"], "--o", color=color, label=f"{camera_id} 测试前")
        axes[0, 1].plot(range(1, 5), camera["after_distortion"], "-o", color=color, label=f"{camera_id} 测试后")
    axes[0, 1].set_xticks(range(1, 5), ("k1", "k2", "k3", "k4"))
    axes[0, 1].set_title("等距畸变系数", fontproperties=font)
    axes[0, 1].legend(prop=font, fontsize=7)
    axes[0, 1].grid(alpha=0.22)

    before = np.asarray(parameters["translation_before_m"]) * 1000.0
    after = np.asarray(parameters["translation_after_m"]) * 1000.0
    axes[1, 0].bar(np.arange(3) - width / 2, before, width=width, label="测试前")
    axes[1, 0].bar(np.arange(3) + width / 2, after, width=width, label="测试后")
    axes[1, 0].set_xticks(range(3), ("tx", "ty", "tz"))
    axes[1, 0].set_ylabel("mm")
    axes[1, 0].set_title("T_cn_cnm1 平移分量", fontproperties=font)
    axes[1, 0].legend(prop=font)
    axes[1, 0].grid(axis="y", alpha=0.22)

    axes[1, 1].axis("off")
    text = (
        f"基线：{parameters['baseline_before_m'] * 1000:.6f} → {parameters['baseline_after_m'] * 1000:.6f} mm\n"
        f"基线变化：{parameters['baseline_delta_mm']:+.6f} mm\n"
        f"平移向量差模长：{parameters['translation_delta_norm_mm']:.6f} mm\n"
        f"相对旋转：{parameters['relative_rotation_deg']:.6f}°\n"
        f"标定集 Alignment：{parameters['calibration_alignment_before_px']:.6f} → "
        f"{parameters['calibration_alignment_after_px']:.6f} px"
    )
    axes[1, 1].text(0.02, 0.94, text, va="top", fontsize=12, linespacing=1.8, fontproperties=font)
    axes[1, 1].set_title("外参变化摘要", fontproperties=font)
    figure.suptitle("标定参数变化", fontproperties=font, fontsize=15)
    return figure


def _write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _html_table(rows: list[dict[str, Any]], columns: list[tuple[str, str]], digits: int = 6) -> str:
    head = "".join(f"<th>{html.escape(label)}</th>" for _, label in columns)
    body = []
    for row in rows:
        cells = []
        for key, _ in columns:
            value = row.get(key, "")
            if isinstance(value, float):
                value = f"{value:.{digits}f}"
            cells.append(f"<td>{html.escape(str(value))}</td>")
        body.append("<tr>" + "".join(cells) + "</tr>")
    return f"<table><thead><tr>{head}</tr></thead><tbody>{''.join(body)}</tbody></table>"


def _comparison_text(stats: list[dict[str, Any]]) -> list[str]:
    indexed = {row["group_id"]: row for row in stats}
    output = []
    for metric, label in METRICS:
        before = indexed["before_with_before"][metric + "_mean"]
        after_old = indexed["after_with_before"][metric + "_mean"]
        after_new = indexed["after_with_after"][metric + "_mean"]
        output.append(f"{label}：测试后沿用测试前参数相对测试前变化 {(after_old / before - 1) * 100:+.3f}%")
        output.append(f"{label}：测试后改用新参数相对沿用旧参数变化 {(after_new / after_old - 1) * 100:+.3f}%")
    return output


def generate_report(manifest: dict[str, Any], output: Path | None = None) -> dict[str, Any]:
    summary, details = collect_evidence(manifest)
    stats = _group_stats(summary)
    parameters = _parameter_evidence(manifest)
    root = Path(manifest["_machine_root"])
    destination = output.resolve() if output else machine_path(manifest, manifest.get("report_output", "comparison_report"))
    if destination.exists() and any(destination.iterdir()):
        raise FileExistsError(f"report output is not empty: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".comparison-report-", dir=str(destination.parent)))
    charts = stage / "charts"
    charts.mkdir()
    font = _font()
    figures = [
        ("summary_metrics.png", _summary_figure(summary, font)),
        ("post_paired_metrics.png", _paired_figure(summary, font)),
        ("per_image_pair_metrics.png", _detail_figure(details, font)),
        ("parameter_changes.png", _parameter_figure(parameters, font)),
    ]
    encoded: dict[str, str] = {}
    try:
        for name, figure in figures:
            encoded[name] = _save_figure(figure, charts / name)

        summary_fields = [
            "group_id", "group", "test_phase", "calibration_phase", "repeat", "dataset_id", "calibration_label",
            "cam0_rms_px", "cam1_rms_px", "stereo_rms_px", "alignment_rms_px", "cam0_frames", "cam1_frames",
            "stereo_pairs", "alignment_count", "result_directory",
        ]
        _write_csv(stage / "comparison_summary.csv", summary, summary_fields)
        detail_fields = list(details[0].keys()) if details else []
        _write_csv(stage / "comparison_details.csv", details, detail_fields)

        machine_name = root.name
        report_stem = machine_name + "_aging_vibration_comparison"
        stats_rows = []
        for row in stats:
            stats_rows.append(
                {
                    "group": row["group"],
                    **{metric: row[metric + "_mean"] for metric, _ in METRICS},
                    **{metric + "_std": row[metric + "_sample_std"] for metric, _ in METRICS},
                }
            )
        summary_table = _html_table(
            stats_rows,
            [("group", "数据与参数组合")]
            + [(metric, label + " 均值") for metric, label in METRICS]
            + [(metric + "_std", label + " 标准差") for metric, label in METRICS],
        )
        parameter_rows = []
        for camera_id in ("cam0", "cam1"):
            camera = parameters["cameras"][camera_id]
            for name, before, after in zip(("fx", "fy", "cx", "cy"), camera["before_intrinsics"], camera["after_intrinsics"]):
                parameter_rows.append({"camera": camera_id, "parameter": name, "before": before, "after": after, "delta": after - before})
            for index, (before, after) in enumerate(zip(camera["before_distortion"], camera["after_distortion"]), 1):
                parameter_rows.append({"camera": camera_id, "parameter": f"k{index}", "before": before, "after": after, "delta": after - before})
        detail_columns = [
            ("group", "组合"), ("repeat", "重复"), ("dataset_id", "数据集"), ("record_type", "记录类型"),
            ("camera_id", "相机"), ("source_index", "图像索引"), ("reprojection_rms_px", "重投影 RMS"),
            ("reprojection_p95_px", "重投影 P95"), ("reprojection_max_px", "重投影 Max"),
            ("alignment_rms_px", "Alignment RMS"), ("alignment_p95_px", "Alignment P95"),
            ("alignment_max_px", "Alignment Max"),
        ]
        observations = "".join(f"<li>{html.escape(item)}</li>" for item in _comparison_text(stats))
        images = "".join(
            f"<figure><img src=\"data:image/png;base64,{encoded[name]}\" alt=\"{html.escape(name)}\"></figure>"
            for name, _ in figures
        )
        document = f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>{html.escape(machine_name)} 老化振动双目精度对比</title>
<style>body{{font-family:"Microsoft YaHei","Noto Sans CJK SC",sans-serif;max-width:1500px;margin:30px auto;padding:0 24px;color:#1a202c}}h1,h2{{color:#16324f}}.note{{background:#edf2f7;border-left:5px solid #2b6cb0;padding:12px 16px}}table{{border-collapse:collapse;width:100%;font-size:12px;margin:14px 0 24px}}th,td{{border:1px solid #cbd5e0;padding:6px 8px;text-align:right}}th:first-child,td:first-child{{text-align:left}}th{{background:#e2e8f0;position:sticky;top:0}}figure{{margin:24px 0}}img{{max-width:100%;height:auto}}details{{margin:18px 0}}code{{background:#edf2f7;padding:2px 5px}}</style></head>
<body><h1>{html.escape(machine_name)} 老化振动前后双目精度对比报告</h1>
<p class="note">本报告仅使用已有标定结果和固定参数验证证据，不重新检测或优化，也不输出通过/不通过判定。跨阶段数据是相同名义位姿的独立采集；测试后新旧参数比较使用完全相同的测试图像。</p>
<h2>汇总</h2>{summary_table}<ul>{observations}</ul>
<h2>图表</h2>{images}
<h2>内参与畸变参数变化</h2>{_html_table(parameter_rows, [('camera','相机'),('parameter','参数'),('before','测试前'),('after','测试后'),('delta','变化')], 9)}
<h2>外参与可观性</h2><p>基线变化：{parameters['baseline_delta_mm']:+.6f} mm；平移向量差模长：{parameters['translation_delta_norm_mm']:.6f} mm；相对旋转：{parameters['relative_rotation_deg']:.6f}°。</p>
<p>测试前：<code>{html.escape(str(parameters['observability']['before']))}</code>；测试后：<code>{html.escape(str(parameters['observability']['after']))}</code>。</p>
<details><summary>逐图与逐图对明细（共 {len(details)} 行）</summary>{_html_table(details, detail_columns, 9)}</details>
<p>完整机器可读数据见 <code>comparison_summary.csv</code> 与 <code>comparison_details.csv</code>。</p></body></html>"""
        (stage / (report_stem + ".html")).write_text(document, encoding="utf-8")

        with PdfPages(stage / (report_stem + ".pdf")) as pdf:
            title_page = plt.figure(figsize=(11.69, 8.27))
            title_page.text(0.06, 0.90, f"{machine_name} 老化振动前后双目精度对比", fontsize=21, fontproperties=font)
            title_page.text(0.06, 0.84, "仅使用已有证据；无通过/不通过判定", fontsize=12, fontproperties=font)
            lines = []
            for row in stats:
                lines.append(row["group"])
                lines.append(
                    "  cam0 {0:.6f} | cam1 {1:.6f} | stereo {2:.6f} | alignment {3:.6f} px".format(
                        row["cam0_rms_px_mean"], row["cam1_rms_px_mean"], row["stereo_rms_px_mean"], row["alignment_rms_px_mean"]
                    )
                )
            lines.extend(["", "关键变化：", *_comparison_text(stats)])
            lines.extend(
                [
                    "",
                    f"外参：基线变化 {parameters['baseline_delta_mm']:+.6f} mm；平移差 {parameters['translation_delta_norm_mm']:.6f} mm；相对旋转 {parameters['relative_rotation_deg']:.6f}°",
                    f"可观性：测试前 {parameters['observability']['before']}；测试后 {parameters['observability']['after']}",
                ]
            )
            title_page.text(0.06, 0.77, "\n".join(lines), va="top", fontsize=10.5, linespacing=1.45, fontproperties=font)
            title_page.text(0.06, 0.05, "逐图和逐图对完整数值见随附 CSV。", fontsize=9, fontproperties=font)
            pdf.savefig(title_page)
            plt.close(title_page)
            for _, figure in figures:
                pdf.savefig(figure)

        report_manifest = {
            "schema_version": "1.0.0",
            "kind": "stereo_aging_vibration_comparison_report",
            "machine_id": manifest["machine_id"],
            "source_manifest": str(manifest["_manifest_path"]),
            "recalibration_or_verification_run": False,
            "summary_rows": len(summary),
            "detail_rows": len(details),
            "files": sorted(
                [
                    report_stem + ".html",
                    report_stem + ".pdf",
                    "comparison_summary.csv",
                    "comparison_details.csv",
                    *("charts/" + name for name, _ in figures),
                ]
            ),
            "parameter_comparison": parameters,
        }
        _write_json(stage / "report_manifest.json", report_manifest)
        report_manifest["files"].append("report_manifest.json")
        _write_json(stage / ".inventory.json", sorted(report_manifest["files"] + [".inventory.json"]))

        if destination.exists():
            destination.rmdir()
        stage.rename(destination)
        return {
            "output": str(destination),
            "html": str(destination / (report_stem + ".html")),
            "pdf": str(destination / (report_stem + ".pdf")),
            "summary_csv": str(destination / "comparison_summary.csv"),
            "details_csv": str(destination / "comparison_details.csv"),
            "summary_rows": len(summary),
            "detail_rows": len(details),
        }
    except Exception:
        shutil.rmtree(stage, ignore_errors=True)
        raise
    finally:
        for _, figure in figures:
            plt.close(figure)
