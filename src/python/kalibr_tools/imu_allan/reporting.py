"""Render Allan evidence in self-contained HTML and vector PDF reports."""

from __future__ import annotations

import base64
import html
from io import BytesIO
from pathlib import Path
import textwrap

from .allan_core import UPSTREAM_REVISION
from .upstream_analysis import generate_prediction


PARAMETERS = (
    ("accelerometer_noise_density", "m/s²/√Hz", "m/s² / √Hz"),
    ("accelerometer_random_walk", "m/s²/√s", "m/s² / √s"),
    ("gyroscope_noise_density", "rad/s/√Hz", "rad/s / √Hz"),
    ("gyroscope_random_walk", "rad/s/√s", "rad/s / √s"),
)
FIT_FIELDS = (
    ("white_noise_at_1s", "白噪声 @1 s", "White @1 s"),
    ("random_walk_at_3s", "随机游走 @3 s", "Walk @3 s"),
    ("minimum_allan_deviation", "最小 Allan 标准差", "Min ADEV"),
    ("minimum_tau_s", "最小值处 τ [s]", "Min tau [s]"),
    ("white_fit_slope", "白噪声斜率", "White slope"),
    ("random_walk_fit_slope", "随机游走斜率", "Walk slope"),
)
INK = "#18354a"
MUTED = "#596f7f"
ACCENT = "#087e8b"
PALE = "#eff5f8"
LINE = "#d6e2e9"
STYLE = {"font.family": "DejaVu Sans", "font.size": 10, "text.color": INK,
         "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
         "pdf.fonttype": 42}


def _format(value):
    return format(value, ".12g") if isinstance(value, float) else str(value)


def _table(headers, rows):
    head = "".join(f"<th>{html.escape(str(value))}</th>" for value in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{html.escape(_format(value))}</td>" for value in row) + "</tr>"
        for row in rows
    )
    return f'<div class="table-wrap"><table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'


def _selection(source):
    selected = source["selection"]
    end = "end" if selected["end_s"] is None else _format(selected["end_s"])
    return f"[{_format(selected['start_s'])}, {end}) s"


def _dataset_name(source):
    return source.get("dataset_id", Path(source["source"]).parent.name)


def _fit_rows(group):
    return [[axis.upper()] + [group[axis][key] for key, _, _ in FIT_FIELDS] for axis in ("x", "y", "z")]


def _html(source, parameters, fit, measure_rate, charts):
    summary = _table(("项目", "值"), (
        ("输入 CSV（只读）", source["source"]),
        ("选择区间（相对原始首条样本）", _selection(source)),
        ("计算 measure_rate [Hz]", measure_rate),
        ("输出 rostopic", parameters["rostopic"]),
        ("输出 update_rate [Hz]", parameters["update_rate"]),
    ))
    parameter_table = _table(("参数", "值", "单位"), (
        (key, parameters[key], unit) for key, unit, _ in PARAMETERS
    ))
    methods = _table(("计算项", "规则"), (
        ("Allan 曲线", "六轴各 9,999 点，τ = 0.1–999.9 s，步长 0.1 s；不重叠分箱"),
        ("白噪声拟合", "0.1 ≤ τ < 10 s；斜率 [-0.5, -0.499]；在 1 s 处取值"),
        ("随机游走拟合", "全部尺度；斜率强制约束为 [0.5, 0.501]；在 3 s 处取值"),
        ("轴聚合与单位", "四项输出取 X/Y/Z 最大值；陀螺仪拟合 deg/s，YAML 转回弧度制"),
    ))
    fit_tables = "".join(
        f"<h3>{title}</h3>" + _table(
            ("轴",) + tuple(label for _, label, _ in FIT_FIELDS), _fit_rows(fit[key]))
        for key, title in (("accelerometer", "加速度计：m/s²"), ("gyroscope_deg_per_s", "陀螺仪：deg/s"))
    )
    images = "".join(
        f'<figure><img alt="{title} Allan deviation" src="data:image/png;base64,{data}">'
        f'<figcaption>{title} · 实测曲线与已有拟合参数重建的拟合线</figcaption></figure>'
        for title, data in charts
    )
    cards = "".join(
        f'<div class="card"><span>{label}</span><strong>{html.escape(value)}</strong></div>'
        for label, value in (("所选样本数", f"{source['messages']:,}"),
                             ("所选时长", f"{source['duration_s'] / 3600:.3f} h"),
                             ("实际平均采样率", f"{source['effective_rate_hz']:.6f} Hz"))
    )
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>IMU Allan 方差分析报告</title>
<style>body{{margin:0;background:#f3f7fa;color:{INK};font:15px/1.65 system-ui,sans-serif}}main{{max-width:1120px;margin:0 auto;padding:36px 22px}}header{{border-top:5px solid {ACCENT};padding-top:24px}}.eyebrow{{color:{ACCENT};font-size:12px;letter-spacing:.14em;font-weight:700}}h1{{font-size:32px;margin:8px 0}}h2{{font-size:22px;margin:0 0 16px}}h3{{font-size:16px;margin-top:24px}}.dataset{{color:{MUTED};overflow-wrap:anywhere}}.cards{{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin:24px 0}}.card,section,figure{{background:white;border:1px solid {LINE};border-radius:10px;padding:24px}}.card span{{display:block;color:{MUTED};font-size:13px}}.card strong{{font-size:25px;line-height:1.7}}section,figure{{margin:20px 0}}.table-wrap{{overflow-x:auto}}table{{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;font-size:13px}}th,td{{border-bottom:1px solid {LINE};padding:12px;text-align:left;overflow-wrap:anywhere}}th{{background:{PALE};color:{MUTED};font-size:12px}}td:first-child{{font-weight:500}}tr:last-child td{{border-bottom:0}}img{{display:block;width:100%;height:auto}}figcaption,.muted{{color:{MUTED};font-size:13px}}.notice{{padding:16px 20px;border-left:4px solid #d7a43e;background:#fff7e8}}code{{overflow-wrap:anywhere}}footer{{color:{MUTED};font-size:12px;margin:24px 0}}@media(max-width:700px){{main{{padding:20px 12px}}.cards{{grid-template-columns:1fr}}section,figure{{padding:16px}}h1{{font-size:26px}}}}</style></head>
<body><main><header><div class="eyebrow">KALIBR · IMU NOISE ANALYSIS</div><h1>IMU Allan 方差分析报告</h1>
<div class="dataset">{html.escape(_dataset_name(source))}</div></header><div class="cards">{cards}</div>
<section><h2>imu_noise.yaml 参数</h2>{parameter_table}<p class="muted">参数按三轴最大拟合值输出。噪声密度与随机游走的时间维度不同，使用时请保留对应单位。</p></section>
<section><h2>输入与采样</h2>{summary}<p class="muted">按样本序号和向下取整的平均采样率分箱，不重采样、不插值；时间戳抖动会影响名义 τ 与实际时间尺度的对应。</p></section>
<section><h2>计算与拟合规则</h2>{methods}<p class="muted">兼容目标：<code>ori-drs/allan_variance_ros@{UPSTREAM_REVISION}</code>。</p>
<p class="notice">输入应为静止采集数据。本工具未自动判断静止状态，也未确认存在可信的 +1/2 随机游走区域；全尺度强制拟合得到的是参数候选，不等同于生产验收。</p></section>
<section><h2>逐轴拟合明细</h2>{fit_tables}<p class="muted">最小 Allan 标准差是曲线的实测最小值，不作为额外噪声标定参数写入 YAML。</p></section>
{images}<footer>曲线仅嵌入 HTML/PDF，不生成独立 PNG。详细输入、曲线复算验证和拟合记录见 work/ 下的 JSON 与 allan_variance.csv。HTML 可独立分享。</footer>
</main></body></html>"""


def _ascii(value):
    # PDF remains portable without a host-specific CJK font.
    return str(value).encode("ascii", "backslashreplace").decode("ascii")


def _page(title, subtitle, number):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    figure = plt.figure(figsize=(11.69, 8.27), facecolor="white")
    figure.add_artist(Line2D((.06, .94), (.955, .955), color=ACCENT, linewidth=3,
                             transform=figure.transFigure))
    figure.text(.06, .913, "KALIBR / IMU NOISE ANALYSIS", fontsize=9, color=ACCENT, weight="bold")
    figure.text(.06, .858, title, fontsize=23, weight="bold")
    figure.text(.06, .817, subtitle, fontsize=10, color=MUTED)
    figure.text(.06, .035, "kalibr-noros tools imu-allan  |  schema 1.0.0", fontsize=8, color=MUTED)
    figure.text(.94, .035, f"{number} / 4", ha="right", fontsize=8, color=MUTED)
    return figure


def _pdf_table(figure, bounds, headers, rows, widths=None, size=10):
    axis = figure.add_axes(bounds)
    axis.axis("off")
    table = axis.table(cellText=rows, colLabels=headers, colWidths=widths,
                       cellLoc="left", colLoc="left", bbox=(0, 0, 1, 1))
    table.auto_set_font_size(False)
    table.set_fontsize(size)
    for (row, _column), cell in table.get_celld().items():
        cell.set_edgecolor(LINE)
        cell.set_linewidth(.6)
        cell.set_facecolor(PALE if row == 0 else "white")
        cell.set_text_props(color=MUTED if row == 0 else INK,
                            weight="bold" if row == 0 else "normal")
        cell.PAD = .10
    return table


def _summary_figure(source, parameters, measure_rate):
    from matplotlib.patches import FancyBboxPatch

    figure = _page("IMU Allan deviation", "Noise parameter summary · " + _ascii(_dataset_name(source)), 1)
    for x, label, value in (
        (.06, "SELECTED SAMPLES", f"{source['messages']:,}"),
        (.36, "SELECTED DURATION", f"{source['duration_s'] / 3600:.3f} h"),
        (.66, "EFFECTIVE SAMPLE RATE", f"{source['effective_rate_hz']:.6f} Hz"),
    ):
        figure.add_artist(FancyBboxPatch((x, .674), .28, .105,
                          boxstyle="round,pad=0.007,rounding_size=0.01", facecolor=PALE,
                          edgecolor="none", transform=figure.transFigure))
        figure.text(x + .015, .748, label, fontsize=8, color=MUTED, weight="bold")
        figure.text(x + .015, .703, value, fontsize=19, color=INK, weight="bold")
    figure.text(.06, .621, "PARAMETERS FOR imu_noise.yaml", fontsize=10, weight="bold", color=ACCENT)
    _pdf_table(figure, (.06, .351, .88, .238), ("Parameter", "Value", "Unit"),
               [(key, f"{parameters[key]:.8g}", unit) for key, _, unit in PARAMETERS],
               widths=(.48, .23, .29), size=10)
    figure.text(.06, .304, "INPUT & SELECTION", fontsize=10, weight="bold", color=ACCENT)
    path_lines = textwrap.wrap(_ascii(source["source"]), width=132)
    figure.text(.06, .274, "Source CSV (read-only)", fontsize=8, color=MUTED)
    path_size = min(8.1, 35 / max(len(path_lines), 1))
    figure.text(.06, .251, "\n".join(path_lines), fontsize=path_size, family="DejaVu Sans Mono",
                va="top", linespacing=1.3)
    figure.text(.06, .148, f"Selected offsets: {_selection(source)}", fontsize=9)
    figure.text(.53, .148, f"Duration: {source['duration_s']:.6f} s", fontsize=9)
    figure.text(.06, .116, f"Calculation rate: {measure_rate} Hz (integer)", fontsize=9)
    figure.text(.53, .116, f"YAML update_rate: {parameters['update_rate']:.6f} Hz", fontsize=9)
    topic = textwrap.fill("rostopic: " + _ascii(parameters["rostopic"]), width=132)
    figure.text(.06, .084, topic, fontsize=8.3, color=MUTED, va="top")
    return figure


def _details_figure(fit):
    figure = _page("Fitting method & axis details", "Pinned upstream fitting rules; candidates for the IMU noise model", 2)
    _pdf_table(figure, (.06, .652, .88, .12), ("Fit", "Scales", "Slope bounds", "Evaluation"),
               [("White noise", "0.1 <= tau < 10 s", "[-0.5, -0.499]", "tau = 1 s"),
                ("Random walk", "All 9,999 scales", "[+0.5, +0.501]", "tau = 3 s")],
               widths=(.22, .31, .25, .22), size=9)
    for key, title, top in (("accelerometer", "ACCELEROMETER [m/s²]", .609),
                            ("gyroscope_deg_per_s", "GYROSCOPE [deg/s]", .405)):
        figure.text(.06, top, title, fontsize=10, color=ACCENT, weight="bold")
        rows = [[_format(value) if index == 0 else f"{value:.5g}"
                 for index, value in enumerate(row)] for row in _fit_rows(fit[key])]
        _pdf_table(figure, (.06, top - .157, .88, .13),
                   ("Axis",) + tuple(label for _, _, label in FIT_FIELDS), rows,
                   widths=(.06, .16, .16, .16, .14, .16, .16), size=8.2)
    figure.text(.06, .199, "METHOD NOTES", fontsize=9, color=ACCENT, weight="bold")
    figure.text(.06, .174, "9,999 non-overlapping scales: tau = 0.1..999.9 s. Output uses the maximum of X/Y/Z.\n"
                "Gyro curves/fits use deg/s; YAML parameters use radians. No timestamp resampling or interpolation.",
                fontsize=9, color=MUTED, va="top", linespacing=1.5)
    figure.text(.06, .110, "Stationary input is required; stationarity is not checked automatically. The forced all-scale +1/2 fit\n"
                "does not establish a credible random-walk region. Fitted candidates are not production acceptance.",
                fontsize=8.5, color=MUTED, va="top", linespacing=1.5)
    figure.text(.06, .061, "Compatibility: ori-drs/allan_variance_ros@" + UPSTREAM_REVISION, fontsize=7.5, color=MUTED)
    return figure


def _curve_figure(curve, fit, group, number):
    import numpy as np
    from matplotlib.lines import Line2D

    acceleration = group == "accelerometer"
    title = "Accelerometer" if acceleration else "Gyroscope"
    unit = "m/s²" if acceleration else "deg/s"
    figure = _page(f"{title} Allan deviation", "Measured curves and fits reconstructed from the saved coefficients", number)
    axis = figure.add_axes((.095, .165, .85, .594))
    period = curve[:, 0]
    values = curve[:, 1:4] if acceleration else curve[:, 4:7]
    details = fit[group]
    colors = ("#2563a6", "#d06b2f", "#16806a")
    for index, (name, color) in enumerate(zip(("x", "y", "z"), colors)):
        detail = details[name]
        axis.loglog(period, values[:, index], color=color, linewidth=1.1, label=name.upper())
        for prefix, style in (("white", "--"), ("random_walk", ":")):
            fitted = np.exp(detail[prefix + "_fit_slope"] * np.log(period)
                            + detail[prefix + "_fit_log_intercept"])
            axis.loglog(period, fitted, color=color, linestyle=style, linewidth=1, alpha=.8)
        for tau, value, marker in ((1, detail["white_noise_at_1s"], "o"),
                                   (3, detail["random_walk_at_3s"], "*"),
                                   (detail["minimum_tau_s"], detail["minimum_allan_deviation"], "^")):
            axis.plot(tau, value, marker, color=color, markersize=5)
    mean_model = generate_prediction(
        period, q_white=sum(details[name]["white_noise_at_1s"] for name in details) / 3,
        q_bias_instability=sum(details[name]["minimum_allan_deviation"] for name in details) / 3,
        q_walk=sum(details[name]["random_walk_at_3s"] for name in details) / 3)
    axis.loglog(period, mean_model, color=INK, linewidth=1.4)
    axis.set_xlabel("Averaging time, τ [s]", labelpad=10)
    axis.set_ylabel(f"Allan deviation [{unit}]", labelpad=10)
    axis.grid(which="major", color=LINE, linewidth=.8)
    axis.grid(which="minor", color=LINE, linewidth=.4, alpha=.45)
    axis.tick_params(labelsize=9)
    for spine in axis.spines.values():
        spine.set_color(LINE)
    handles, labels = axis.get_legend_handles_labels()
    handles.extend([Line2D([], [], color=MUTED, linestyle="--"),
                    Line2D([], [], color=MUTED, linestyle=":"),
                    Line2D([], [], color=INK, linewidth=1.4)])
    axis.legend(handles, labels + ["White fit", "Walk fit", "Mean fitted model"],
                ncol=6, loc="lower left", bbox_to_anchor=(0, 1.02), frameon=False, fontsize=9)
    figure.text(.095, .080, "Markers: circle = white @1 s   ·   star = walk @3 s   ·   triangle = measured minimum",
                fontsize=8.5, color=MUTED)
    return figure


def generate_reports(output: Path, source: dict, parameters: dict, fit: dict, measure_rate: int) -> None:
    """Render cached evidence without refitting or writing standalone image files."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    import numpy as np

    curve = np.loadtxt(output / "work" / "allan_variance.csv")
    if curve.shape != (9999, 7) or not np.all(np.isfinite(curve)) or not np.all(curve > 0):
        raise ValueError("expected positive finite 9999 by 7 cached Allan curve")
    charts = []
    with matplotlib.rc_context(STYLE), PdfPages(output / "imu_allan.report.pdf") as pdf:
        figures = (_summary_figure(source, parameters, measure_rate), _details_figure(fit))
        try:
            for figure in figures:
                pdf.savefig(figure)
        finally:
            for figure in figures:
                plt.close(figure)
        for number, group in enumerate(("accelerometer", "gyroscope_deg_per_s"), 3):
            figure = _curve_figure(curve, fit, group, number)
            try:
                pdf.savefig(figure)
                buffer = BytesIO()
                figure.savefig(buffer, format="png", dpi=140)
                charts.append(("Accelerometer" if number == 3 else "Gyroscope",
                               base64.b64encode(buffer.getvalue()).decode("ascii")))
            finally:
                plt.close(figure)
    (output / "imu_allan.report.html").write_text(
        _html(source, parameters, fit, measure_rate, charts), encoding="utf-8")
