"""Human-readable reports and plots for rolling-shutter diagnosis."""

from __future__ import annotations

import csv
import html
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import yaml

from .workflow import _atomic_text


LABELS = {
    "important": "重要：数据支持逐行时间模型，仍需按建议任务正式标定并独立验收",
    "inconclusive": "证据不足：不能确认或排除滚动快门行时间",
    "unsupported": "不支持：当前数据没有显示稳定且独立于列方向的行时序收益",
}

ESTIMATED_LINE_DELAY_RUNS = {
    "free_seed0",
    "free_seed_candidate",
    "free_seed_opposite",
    "knot_half_free",
    "knot_double_free",
}

GATE_DESCRIPTIONS = {
    "all_required_runs_completed": "10 组正式 RS 对照均完成并产生可读取结果",
    "all_optimizer_summaries_healthy": "全部正式对照都有优化摘要、迭代数，且未报告线性求解失败",
    "finite_line_delay_below_95_percent_bound": "主估计值有限，且绝对值小于配置上界的 95%，没有贴边",
    "hard_rank_full": "主结果在机器精度秩判据下为满秩",
    "line_delay_not_dominant_in_operational_weak_mode": "行时间在运行判据的最弱模态中贡献不超过 50%",
    "free_improves_common_corner_residual_with_positive_bootstrap_lower_bound": "主自由估计相对同轨迹固定 0 改善公共角点残差，且按帧 bootstrap 95% 下界大于 0",
    "fixed_candidate_better_than_fixed_opposite": "固定正候选比固定反向候选的公共角点残差更小",
    "multistart_spread_within_10_percent_bound": "从 0、正候选和反向候选出发的三个自由估计跨度不超过行时间上界的 10%",
    "stereo_pair_metrics_finite": "双目参考目不适用；其余相机的 Optical Alignment 与 RS 补偿配对残差均为有限数",
    "knot_spread_within_20_percent_bound": "0.5×、1×、2× knot 下自由估计行时间的跨度不超过行时间上界的 20%",
    "all_knot_controls_improve": "0.5×、1×、2× knot 下，自由行时间相对同频率固定 0 的 bootstrap 95% 下界都必须大于 0",
    "line_delay_exceeds_1_96_local_std": "Camera–IMU 主估计的绝对值大于 1.96 倍局部标准差",
    "gyro_and_accel_not_both_worse": "Camera–IMU 自由行时间不会同时使 gyro 和 accel RMS 变差",
}


def _camera_rows(result: dict[str, Any]) -> list[dict[str, Any]]:
    screen = result["screening"]
    formal = result.get("formal_comparison") or {}
    rows = []
    for camera_id, value in screen["camera_results"].items():
        forward = value.get("blocked_forward", {}).get("row", {})
        reverse = value.get("blocked_reverse", {}).get("row", {})
        base = (formal.get("runs", {}).get("free_seed0", {}).get("cameras", {}).get(camera_id, {}))
        rows.append({
            "camera_id": camera_id,
            "screen_status": value["status"],
            "used_frames": value["input_used_frames"],
            "candidate_line_delay_us": value.get("candidate_line_delay_us"),
            "candidate_first_last_span_ms": value.get("first_to_last_row_span_ms"),
            "forward_before_rms_px": forward.get("before_rms_px"),
            "forward_after_rms_px": forward.get("after_rms_px"),
            "forward_reduction_percent": _percent(forward.get("relative_reduction")),
            "reverse_before_rms_px": reverse.get("before_rms_px"),
            "reverse_after_rms_px": reverse.get("after_rms_px"),
            "reverse_reduction_percent": _percent(reverse.get("relative_reduction")),
            "formal_line_delay_us": _scale(base.get("line_delay_s"), 1e6),
            "formal_line_delay_std_us": _scale(base.get("line_delay_std_s"), 1e6),
            "formal_rms_px": base.get("rms_px"),
            "formal_gates_passed": (formal.get("camera_gates", {}).get(camera_id, {}).get("passed")),
        })
    return rows


def _formal_run_rows(result: dict[str, Any]) -> list[dict[str, Any]]:
    runs = ((result.get("formal_comparison") or {}).get("runs") or {})
    rows = []
    for run_name, run in runs.items():
        for camera_id, camera in (run.get("cameras") or {}).items():
            rows.append({
                "run": run_name,
                "run_type": "formal_rs",
                "camera_id": camera_id,
                "line_delay_mode": (
                    "estimated" if run_name in ESTIMATED_LINE_DELAY_RUNS else "fixed"
                ),
                "line_delay_us": _scale(camera.get("line_delay_s"), 1e6),
                "rms_px": camera.get("rms_px"),
                "alignment_rms_px": camera.get("alignment_rms_px"),
                "rs_compensated_pair_rms_px": camera.get(
                    "rs_compensated_pair_residual_rms_px"
                ),
            })
    return rows


def _ordinary_run_rows(output: Path) -> list[dict[str, Any]]:
    result_path = output / "runs" / "ordinary_source" / "ordinary_source.yaml"
    if not result_path.is_file():
        return []
    document = yaml.safe_load(result_path.read_text(encoding="utf-8")) or {}
    rows = []
    for camera in document.get("cameras") or []:
        rows.append({
            "run": "ordinary_source",
            "run_type": "ordinary",
            "camera_id": camera.get("id"),
            "line_delay_mode": "not_applicable",
            "line_delay_us": None,
            "rms_px": camera.get("rms"),
            "alignment_rms_px": camera.get("alignment"),
            "rs_compensated_pair_rms_px": None,
        })
    return rows


def _knot_control_rows(result: dict[str, Any]) -> list[dict[str, Any]]:
    formal = result.get("formal_comparison") or {}
    runs = formal.get("runs") or {}
    comparisons = formal.get("common_corner_comparisons") or {}
    pairs = (
        ("0.5×", "knot_half_free", "knot_half_zero"),
        ("1×", "free_seed0", "fixed_zero"),
        ("2×", "knot_double_free", "knot_double_zero"),
    )
    rows = []
    for frequency, free_name, zero_name in pairs:
        comparison = comparisons.get(f"{free_name}_vs_{zero_name}") or {}
        camera_ids = (runs.get(free_name) or {}).get("cameras") or {}
        for camera_id, free_camera in camera_ids.items():
            zero_camera = ((runs.get(zero_name) or {}).get("cameras") or {}).get(camera_id, {})
            evidence = comparison.get(camera_id) or {}
            lower = (evidence.get("bootstrap_95_percent") or {}).get("lower_px")
            rows.append({
                "knot_frequency": frequency,
                "camera_id": camera_id,
                "free_rms_px": free_camera.get("rms_px"),
                "fixed_zero_rms_px": zero_camera.get("rms_px"),
                "relative_reduction_percent": _percent(evidence.get("relative_reduction")),
                "bootstrap_lower_px": lower,
                "passed": lower is not None and lower > 0.0,
            })
    return rows


def _scale(value: Any, factor: float) -> float | None:
    return None if value is None else float(value) * factor


def _percent(value: Any) -> float | None:
    return None if value is None else float(value) * 100.0


def _format(value: Any, digits: int = 6) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "是" if value else "否"
    if isinstance(value, float):
        return f"{value:.{digits}g}"
    return str(value)


def _run_metrics_title(result: dict[str, Any]) -> str:
    formal = result.get("formal_comparison") or {}
    completed = formal.get("completed_run_count")
    required = formal.get("required_run_count")
    if completed is None:
        completed = len(formal.get("runs") or {})
    if required is None:
        required = 10
    return (
        "已完成的真实运行指标"
        f"（1 组普通标定 + {completed}/{required} 组正式 RS 对照）"
    )


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _plots(output: Path, result: dict[str, Any]) -> list[Path]:
    chart_dir = output / "charts"
    chart_dir.mkdir(exist_ok=True)
    screen = result["screening"]["camera_results"]
    camera_ids = list(screen)
    paths = []

    figure, axis = plt.subplots(figsize=(8.0, 4.5))
    delays = [screen[camera].get("candidate_line_delay_us", np.nan) for camera in camera_ids]
    bounds = [screen[camera]["max_abs_line_delay_s"] * 1e6 for camera in camera_ids]
    positions = np.arange(len(camera_ids))
    axis.bar(positions, delays, color="#2b8cbe", label="screen candidate")
    axis.scatter(positions, bounds, marker="_", s=500, color="#d7301f", label="+ bound")
    axis.scatter(positions, [-value for value in bounds], marker="_", s=500, color="#d7301f", label="- bound")
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set_xticks(positions, camera_ids)
    axis.set_ylabel("Line delay [us/row]")
    axis.set_title("Offline row-time candidates and configured bounds")
    axis.legend(loc="best")
    figure.tight_layout()
    path = chart_dir / "screen_line_delay.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    paths.append(path)

    figure, axes = plt.subplots(1, max(1, len(camera_ids)), figsize=(5.0 * max(1, len(camera_ids)), 4.3), squeeze=False)
    for axis, camera_id in zip(axes[0], camera_ids):
        value = screen[camera_id]
        directions = ["forward", "reverse"]
        before = [value.get("blocked_forward", {}).get("row", {}).get("before_rms_px", np.nan), value.get("blocked_reverse", {}).get("row", {}).get("before_rms_px", np.nan)]
        after = [value.get("blocked_forward", {}).get("row", {}).get("after_rms_px", np.nan), value.get("blocked_reverse", {}).get("row", {}).get("after_rms_px", np.nan)]
        x = np.arange(2)
        axis.bar(x - 0.18, before, 0.36, label="before", color="#bdbdbd")
        axis.bar(x + 0.18, after, 0.36, label="row model", color="#31a354")
        axis.set_xticks(x, directions)
        axis.set_ylabel("Held-out 2D corner RMS [px]")
        axis.set_title(camera_id)
        axis.legend(loc="best")
    figure.tight_layout()
    path = chart_dir / "blocked_holdout_rms.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    paths.append(path)

    formal = result.get("formal_comparison") or {}
    runs = formal.get("runs") or {}
    if runs:
        names = [name for name in runs if runs[name].get("completed")]
        figure, axes = plt.subplots(len(camera_ids), 1, figsize=(10.0, max(4.0, 3.4 * len(camera_ids))), squeeze=False)
        for axis, camera_id in zip(axes[:, 0], camera_ids):
            values = [_scale(runs[name].get("cameras", {}).get(camera_id, {}).get("line_delay_s"), 1e6) for name in names]
            values = [np.nan if value is None else value for value in values]
            axis.plot(np.arange(len(names)), values, "o-", color="#756bb1")
            axis.set_xticks(np.arange(len(names)), names, rotation=35, ha="right")
            axis.set_ylabel("us/row")
            axis.set_title(f"{camera_id}: formal controls")
            axis.axhline(0.0, color="black", linewidth=0.8)
        figure.tight_layout()
        path = chart_dir / "formal_line_delay_sensitivity.png"
        figure.savefig(path, dpi=160)
        plt.close(figure)
        paths.append(path)
    return paths


def _markdown(
    result: dict[str, Any],
    config: dict[str, Any],
    rows: list[dict[str, Any]],
    run_rows: list[dict[str, Any]],
    knot_rows: list[dict[str, Any]],
    charts: list[Path],
) -> str:
    classification = result["classification"]
    lines = [
        "# 滚动快门行时间诊断报告",
        "",
        f"**自动结论：{LABELS[classification]}。**",
        "",
        "该结论判断当前数据是否支持逐行时间模型。它不是硬件行时间实测、快门类型认证或产线放行结果。",
        "",
        "## 输入与适用边界",
        "",
        f"- 普通任务：`{config['source_task_path']}`",
        f"- 任务类型：`{config['source_job']}`",
        f"- 运动方式：`{config['capture']['motion']}`",
        f"- 相机时间戳事件：`{config['capture'].get('camera_timestamp_event', '未声明')}`",
        f"- Bootstrap：按帧重采样 {config['bootstrap_samples']} 次，seed={config['bootstrap_seed']}。",
        "",
        "只有 `capture.motion: continuous`、离线筛查全部通过且正式对照全部通过时，自动结论才会达到 `important`。",
        "",
        "## 指标汇总",
        "",
        "| 相机 | 使用帧 | 筛查状态 | 候选行时间 [µs/行] | 首末行跨度 [ms] | 前→后 RMS [px] | 后→前 RMS [px] | 正式行时间 [µs/行] | 正式门槛 |",
        "|---|---:|---|---:|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        lines.append(
            "| {camera_id} | {used_frames} | {screen_status} | {candidate_line_delay_us} | "
            "{candidate_first_last_span_ms} | {forward_before_rms_px}→{forward_after_rms_px} | "
            "{reverse_before_rms_px}→{reverse_after_rms_px} | {formal_line_delay_us} | {formal_gates_passed} |".format(
                **{key: _format(value) for key, value in row.items()}
            )
        )
    lines += ["", "## 离线筛查方法", ""]
    lines += [
        "1. 从普通标定的最终观测归档读取已使用帧和角点；固定最终 K/D，对每帧只重新估计标定板六自由度位姿。",
        "2. 用同一分段内部的相邻位姿有限差分估计图像速度；丢弃每段首末帧，禁止速度预处理跨越训练/验证边界。",
        "3. 从候选行时间方向中投影掉每帧位姿 Jacobian 的六维列空间，减少行时间与独立 PnP 位姿重复解释同一残差。",
        "4. 前半段拟合、后半段验证，再反向验证；同时把图像列坐标作为负对照。",
        "5. 行方向两次相对改善均须至少 10%，按帧 bootstrap 的 95% 下界须大于 0，且行方向改善至少为列方向两倍。",
        "",
    ]
    formal = result.get("formal_comparison")
    if formal:
        lines += ["## 正式求解对照", ""]
        if run_rows:
            lines += [
                f"### {_run_metrics_title(result)}",
                "",
                "`estimated` 表示该组由优化器估计行时间；`fixed` 表示使用控制值；普通标定不含行时间。双目 Alignment 和 RS 补偿配对残差记录在相邻相机行；单目或 cam0 显示为 `—`。",
                "",
                "| 运行组 | 类型 | 相机 | 行时间模式 | 行时间 [µs/行] | 每目重投影 RMS [px] | Optical Alignment RMS [px] | RS 补偿配对残差 RMS [px] |",
                "|---|---|---|---|---:|---:|---:|---:|",
            ]
            for row in run_rows:
                lines.append(
                    "| {run} | {run_type} | {camera_id} | {line_delay_mode} | {line_delay_us} | {rms_px} | "
                    "{alignment_rms_px} | {rs_compensated_pair_rms_px} |".format(
                        **{key: _format(value) for key, value in row.items()}
                    )
                )
            lines.append("")
        if knot_rows:
            lines += [
                "### knot 配对控制",
                "",
                "每个 knot 频率都比较自由行时间与同频率固定 0。Bootstrap 95% 下界必须大于 0；三个频率全部通过时，`all_knot_controls_improve` 才通过。",
                "",
                "| knot 频率 | 相机 | 自由行时间 RMS [px] | 固定 0 RMS [px] | 相对下降 [%] | Bootstrap 95% 下界 [px] | 通过 |",
                "|---|---|---:|---:|---:|---:|---|",
            ]
            for row in knot_rows:
                lines.append(
                    "| {knot_frequency} | {camera_id} | {free_rms_px} | {fixed_zero_rms_px} | "
                    "{relative_reduction_percent} | {bootstrap_lower_px} | {passed} |".format(
                        **{key: _format(value) for key, value in row.items()}
                    )
                )
            lines.append("")
        if formal.get("camera_gates"):
            for camera_id, value in formal["camera_gates"].items():
                lines.append(f"### {camera_id}")
                lines.append("")
                for name, passed in value["gates"].items():
                    description = GATE_DESCRIPTIONS.get(name, "未提供说明")
                    lines.append(
                        f"- {'通过' if passed else '未通过'}：`{name}` — {description}。"
                    )
                lines.append("")
        else:
            completed = formal.get("completed_run_count", 0)
            required = formal.get("required_run_count", 10)
            missing = formal.get("missing_runs") or []
            lines.append(
                f"正式对照未完成：已完成 {completed}/{required} 组；"
                f"未完成组：{', '.join(missing) if missing else '未记录'}。"
                "因此不计算正式门槛，结论保持 `inconclusive`。"
            )
            lines.append("")
        lines += [
            "正式对照使用项目现有 `cameras-rs` 或 `imu-camera-rs` 求解器。固定 0、固定候选、固定反向候选、三个自由初值使用同一数据和参数；纯视觉任务还对 0.5×、1×、2× knot 频率分别设置匹配的固定 0 对照。",
            "",
        ]
    else:
        lines += [
            "## 正式求解对照",
            "",
            "离线筛查未达到启动条件，因此没有运行耗时的 RS 联合求解。此时不能把筛查候选值当作标定结果。",
            "",
        ]
    lines += ["## 图表", ""]
    for chart in charts:
        lines.append(f"![{chart.stem}](charts/{chart.name})")
        lines.append("")
    if result.get("suggested_task"):
        lines += [
            "## 建议后续任务",
            "",
            f"已生成 `{Path(result['suggested_task']).name}`。其中行时间仍为 `estimate: true`，候选值只作为初值；该任务输出必须另行验收。",
            "",
        ]
    lines += [
        "## 文件说明",
        "",
        "- `result.json`：最终分类及证据索引。",
        "- `screening.json`：分块筛查、列方向负对照、bootstrap 和门槛。",
        "- `formal_comparison.json`：正式对照运行、公共角点残差和逐项门槛；仅筛查通过时存在。",
        "- `summary.csv`：每目关键数值。",
        "- `run_metrics.csv`：1 组普通标定和所有已完成正式 RS 对照的逐目行时间、RMS、Alignment 与 RS 补偿残差。",
        "- `configs/`：普通任务副本、正式对照任务和相机初始化。",
        "- `runs/`：每个真实求解的原生输出。",
        "- `logs/`：validate 与 calibrate 终端日志。",
        "- `diagnosis_manifest.json`：配置、依赖、使用图像哈希和断点状态。",
        "",
    ]
    return "\n".join(lines)


def _html_table(rows: list[dict[str, Any]], labels: dict[str, str]) -> str:
    if not rows:
        return "<p>无可用数据。</p>"
    keys = list(rows[0])
    table = "<table><thead><tr>" + "".join(
        f"<th>{html.escape(labels.get(key, key))}</th>" for key in keys
    ) + "</tr></thead><tbody>"
    for row in rows:
        table += "<tr>" + "".join(
            f"<td>{html.escape(_format(row[key]))}</td>" for key in keys
        ) + "</tr>"
    return table + "</tbody></table>"


def _html(
    markdown_text: str,
    result: dict[str, Any],
    rows: list[dict[str, Any]],
    run_rows: list[dict[str, Any]],
    knot_rows: list[dict[str, Any]],
    charts: list[Path],
) -> str:
    classification = result["classification"]
    table = _html_table(rows, {})
    formal_table = _html_table(run_rows, {
        "run": "运行组",
        "run_type": "类型",
        "camera_id": "相机",
        "line_delay_mode": "行时间模式",
        "line_delay_us": "行时间 [µs/行]",
        "rms_px": "每目重投影 RMS [px]",
        "alignment_rms_px": "Optical Alignment RMS [px]",
        "rs_compensated_pair_rms_px": "RS 补偿配对残差 RMS [px]",
    })
    knot_table = _html_table(knot_rows, {
        "knot_frequency": "knot 频率",
        "camera_id": "相机",
        "free_rms_px": "自由行时间 RMS [px]",
        "fixed_zero_rms_px": "固定 0 RMS [px]",
        "relative_reduction_percent": "相对下降 [%]",
        "bootstrap_lower_px": "Bootstrap 95% 下界 [px]",
        "passed": "通过",
    })
    images = "".join(f'<figure><img src="charts/{chart.name}"><figcaption>{html.escape(chart.stem)}</figcaption></figure>' for chart in charts)
    escaped = html.escape(markdown_text)
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>滚动快门行时间诊断</title>
<style>body{{font-family:Arial,"Noto Sans CJK SC",sans-serif;max-width:1180px;margin:2rem auto;line-height:1.65;color:#222}}.status{{padding:1rem;border-left:6px solid #2b8cbe;background:#eef7fb}}table{{border-collapse:collapse;width:100%;font-size:.88rem}}th,td{{border:1px solid #bbb;padding:.4rem;text-align:left}}th{{background:#eee}}img{{max-width:100%}}pre{{white-space:pre-wrap;background:#f7f7f7;padding:1rem}}</style></head>
<body><h1>滚动快门行时间诊断报告</h1><p class="status"><strong>{html.escape(LABELS[classification])}</strong></p>
<h2>指标汇总</h2>{table}<h2>{html.escape(_run_metrics_title(result))}</h2>{formal_table}
<h2>knot 配对控制</h2>{knot_table}<h2>图表</h2>{images}
<h2>完整 Markdown 内容</h2><pre>{escaped}</pre></body></html>"""


def _pdf_table_pages(pdf: PdfPages, rows: list[dict[str, Any]], title: str) -> None:
    if not rows:
        return
    columns = list(rows[0])
    for start in range(0, len(rows), 18):
        page = rows[start:start + 18]
        figure, axis = plt.subplots(figsize=(11.69, 8.27))
        axis.axis("off")
        axis.set_title(title, fontsize=15, pad=14)
        values = [[_pdf_format(row[column]) for column in columns] for row in page]
        table = axis.table(cellText=values, colLabels=columns, loc="center", cellLoc="center")
        table.auto_set_font_size(False)
        table.set_fontsize(7.5)
        table.scale(1.0, 1.35)
        figure.tight_layout()
        pdf.savefig(figure)
        plt.close(figure)


def _pdf_format(value: Any) -> str:
    if isinstance(value, bool):
        return "yes" if value else "no"
    return _format(value)


def _pdf(
    path: Path,
    result: dict[str, Any],
    rows: list[dict[str, Any]],
    run_rows: list[dict[str, Any]],
    knot_rows: list[dict[str, Any]],
    charts: list[Path],
) -> None:
    with PdfPages(path) as pdf:
        figure = plt.figure(figsize=(11.69, 8.27))
        figure.text(0.06, 0.92, "Rolling-shutter row-time diagnosis", fontsize=20, weight="bold")
        figure.text(0.06, 0.84, f"Classification: {result['classification']}", fontsize=15)
        figure.text(0.06, 0.77, "This is an evidence diagnosis, not hardware timing metrology or production approval.", fontsize=10)
        y = 0.67
        for row in rows:
            figure.text(0.07, y, f"{row['camera_id']}: screen={row['screen_status']}, candidate={_format(row['candidate_line_delay_us'])} us/row, formal={_format(row['formal_line_delay_us'])} us/row", fontsize=10)
            y -= 0.06
        pdf.savefig(figure)
        plt.close(figure)
        _pdf_table_pages(pdf, run_rows, "Ordinary calibration and formal RS run metrics")
        _pdf_table_pages(pdf, knot_rows, "Matched knot controls")
        for chart in charts:
            image = plt.imread(chart)
            figure, axis = plt.subplots(figsize=(11.69, 8.27))
            axis.imshow(image)
            axis.axis("off")
            axis.set_title(chart.stem)
            figure.tight_layout()
            pdf.savefig(figure)
            plt.close(figure)


def generate_reports(output: Path, result: dict[str, Any], config: dict[str, Any]) -> None:
    rows = _camera_rows(result)
    run_rows = _ordinary_run_rows(output) + _formal_run_rows(result)
    knot_rows = _knot_control_rows(result)
    _write_csv(output / "summary.csv", rows)
    _write_csv(output / "run_metrics.csv", run_rows)
    charts = _plots(output, result)
    markdown_text = _markdown(result, config, rows, run_rows, knot_rows, charts)
    _atomic_text(output / "README_ZH.md", markdown_text + "\n")
    _atomic_text(
        output / "report.html",
        _html(markdown_text, result, rows, run_rows, knot_rows, charts),
    )
    _pdf(output / "report.pdf", result, rows, run_rows, knot_rows, charts)
