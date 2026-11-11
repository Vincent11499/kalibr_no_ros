"""Diagnose stereo before/after extrinsic changes on held-out after tests."""

from __future__ import annotations

import base64
import copy
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

from .reporting import _font, _html_table, _managed_replace, _result_for_label, _write_csv
from .workflow import (
    _completed_run,
    _run_logged,
    _write_json,
    _write_yaml,
    build_verify_command,
    inspect_study,
    machine_path,
    representative_calibration,
)


METRICS = (
    ("cam0_rms_px", "cam0 RMS [px]"),
    ("cam1_rms_px", "cam1 RMS [px]"),
    ("stereo_rms_px", "双目综合 RMS [px]"),
    ("alignment_rms_px", "Alignment RMS [px]"),
)

VARIANT_ORDER = (
    "before_kd_before_t",
    "after_kd_after_t",
    "before_kd_after_t",
    "after_kd_before_t",
    "before_kd_rotation_all",
    "before_kd_baseline_all",
    "before_kd_rotation_x",
    "before_kd_rotation_y",
    "before_kd_rotation_z",
    "before_kd_baseline_x",
    "before_kd_baseline_y",
    "before_kd_baseline_z",
    "before_kd_baseline_length",
    "before_kd_baseline_direction",
)

VARIANT_DESCRIPTIONS = {
    "before_kd_before_t": "老化前 K/D + 老化前完整外参（已有验证）",
    "after_kd_after_t": "老化后 K/D + 老化后完整外参（已有验证）",
    "before_kd_after_t": "老化前 K/D + 老化后完整外参",
    "after_kd_before_t": "老化后 K/D + 老化前完整外参",
    "before_kd_rotation_all": "老化前 K/D/基线 + 老化后完整 R",
    "before_kd_baseline_all": "老化前 K/D/R + 老化后完整物理基线 b",
    "before_kd_rotation_x": "仅加入老化前后相对旋转向量的 X 分量",
    "before_kd_rotation_y": "仅加入老化前后相对旋转向量的 Y 分量",
    "before_kd_rotation_z": "仅加入老化前后相对旋转向量的 Z 分量",
    "before_kd_baseline_x": "仅加入 cam0 坐标系物理基线变化的 X 分量",
    "before_kd_baseline_y": "仅加入 cam0 坐标系物理基线变化的 Y 分量",
    "before_kd_baseline_z": "仅加入 cam0 坐标系物理基线变化的 Z 分量",
    "before_kd_baseline_length": "仅替换物理基线长度，保留老化前方向",
    "before_kd_baseline_direction": "仅替换物理基线方向，保留老化前长度",
}


def _camera(document: dict[str, Any], camera_id: str) -> dict[str, Any]:
    return next(camera for camera in document["cameras"] if camera["id"] == camera_id)


def _transform(document: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    value = np.asarray(_camera(document, "cam1")["T_cn_cnm1"], dtype=float)
    if value.shape != (4, 4):
        raise ValueError("cam1.T_cn_cnm1 must be 4x4")
    return value[:3, :3], value[:3, 3]


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.asarray(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)), dtype=float)


def _rotation_matrix(vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-14:
        return np.eye(3) + _skew(vector)
    generator = _skew(vector / angle)
    return np.eye(3) + math.sin(angle) * generator + (1.0 - math.cos(angle)) * generator @ generator


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    vee = np.asarray(
        (rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]),
        dtype=float,
    )
    if angle < 1.0e-10:
        return vee / 2.0
    if abs(math.pi - angle) < 1.0e-7:
        eigenvalues, eigenvectors = np.linalg.eig(rotation)
        axis = np.real(eigenvectors[:, int(np.argmin(np.abs(eigenvalues - 1.0)))])
        axis /= np.linalg.norm(axis)
        return axis * angle
    return vee * (angle / (2.0 * math.sin(angle)))


def _with_transform(document: dict[str, Any], rotation: np.ndarray, baseline_cam0: np.ndarray) -> dict[str, Any]:
    result = copy.deepcopy(document)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = -rotation @ baseline_cam0
    _camera(result, "cam1")["T_cn_cnm1"] = transform.tolist()
    return result


def build_extrinsic_variants(
    before_document: dict[str, Any], after_document: dict[str, Any]
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    """Build analysis-only calibrations while keeping R and physical baseline explicit."""
    before_rotation, before_translation = _transform(before_document)
    after_rotation, after_translation = _transform(after_document)
    before_baseline = -before_rotation.T @ before_translation
    after_baseline = -after_rotation.T @ after_translation
    delta_rotation = _rotation_vector(before_rotation.T @ after_rotation)
    delta_baseline = after_baseline - before_baseline

    variants: dict[str, dict[str, Any]] = {}
    variants["before_kd_after_t"] = copy.deepcopy(before_document)
    _camera(variants["before_kd_after_t"], "cam1")["T_cn_cnm1"] = _camera(after_document, "cam1")["T_cn_cnm1"]
    variants["after_kd_before_t"] = copy.deepcopy(after_document)
    _camera(variants["after_kd_before_t"], "cam1")["T_cn_cnm1"] = _camera(before_document, "cam1")["T_cn_cnm1"]
    variants["before_kd_rotation_all"] = _with_transform(before_document, after_rotation, before_baseline)
    variants["before_kd_baseline_all"] = _with_transform(before_document, before_rotation, after_baseline)

    for index, axis in enumerate("xyz"):
        component = np.zeros(3)
        component[index] = delta_rotation[index]
        variants[f"before_kd_rotation_{axis}"] = _with_transform(
            before_document, before_rotation @ _rotation_matrix(component), before_baseline
        )
        baseline = before_baseline.copy()
        baseline[index] += delta_baseline[index]
        variants[f"before_kd_baseline_{axis}"] = _with_transform(before_document, before_rotation, baseline)

    variants["before_kd_baseline_length"] = _with_transform(
        before_document, before_rotation, before_baseline / np.linalg.norm(before_baseline) * np.linalg.norm(after_baseline)
    )
    variants["before_kd_baseline_direction"] = _with_transform(
        before_document, before_rotation, after_baseline / np.linalg.norm(after_baseline) * np.linalg.norm(before_baseline)
    )
    metadata = {
        "schema_version": "1.0.0",
        "kind": "stereo_extrinsic_diagnosis_variants",
        "coordinate_system": "physical baseline b is expressed in cam0; b=-R^T*t and t=-R*b",
        "delta_rotation_vector_cam0_deg": np.degrees(delta_rotation).tolist(),
        "relative_rotation_angle_deg": math.degrees(float(np.linalg.norm(delta_rotation))),
        "before_baseline_cam0_mm": (before_baseline * 1000.0).tolist(),
        "after_baseline_cam0_mm": (after_baseline * 1000.0).tolist(),
        "delta_baseline_cam0_mm": (delta_baseline * 1000.0).tolist(),
        "before_baseline_length_mm": float(np.linalg.norm(before_baseline) * 1000.0),
        "after_baseline_length_mm": float(np.linalg.norm(after_baseline) * 1000.0),
        "variants": {name: VARIANT_DESCRIPTIONS[name] for name in variants},
    }
    return variants, metadata


def _load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected YAML mapping: {path}")
    return value


def _verification_row(manifest: dict[str, Any], variant: str, repeat: int, dataset_id: str, output: Path) -> dict[str, Any]:
    document = json.loads((output / "fixed_camera_validation.json").read_text(encoding="utf-8"))
    result = _result_for_label(document, manifest["kalibr"]["calibration_label"])
    return {
        "variant": variant,
        "description": VARIANT_DESCRIPTIONS[variant],
        "repeat": repeat,
        "dataset_id": dataset_id,
        "cam0_rms_px": float(result["cameras"]["cam0"]["rms_px"]),
        "cam1_rms_px": float(result["cameras"]["cam1"]["rms_px"]),
        "stereo_rms_px": float(result["stereo_combined_rms_px"]),
        "alignment_rms_px": float(result["stereo"]["alignment"]["rms_px"]),
        "cam0_frames": int(result["cameras"]["cam0"]["detected_frames"]),
        "cam1_frames": int(result["cameras"]["cam1"]["detected_frames"]),
        "stereo_pairs": int(result["stereo"]["used_pairs"]),
        "result_directory": str(output),
    }


def _reference_rows(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for variant, phase in (("before_kd_before_t", "before"), ("after_kd_after_t", "after")):
        for run in manifest["phases"]["after"]["test_runs"]:
            output = machine_path(manifest, run["results"][phase])
            rows.append(_verification_row(manifest, variant, int(run["repeat"]), run["dataset_id"], output))
    return rows


def _summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    old_alignment = statistics.mean(row["alignment_rms_px"] for row in rows if row["variant"] == "before_kd_before_t")
    new_alignment = statistics.mean(row["alignment_rms_px"] for row in rows if row["variant"] == "after_kd_after_t")
    denominator = old_alignment - new_alignment
    for variant in VARIANT_ORDER:
        selected = [row for row in rows if row["variant"] == variant]
        if not selected:
            continue
        entry: dict[str, Any] = {"variant": variant, "description": VARIANT_DESCRIPTIONS[variant]}
        for key, _ in METRICS:
            values = [row[key] for row in selected]
            entry[key + "_mean"] = statistics.mean(values)
            entry[key + "_sample_std"] = statistics.stdev(values)
            entry[key + "_range"] = max(values) - min(values)
        entry["alignment_recovery_percent"] = (
            (old_alignment - entry["alignment_rms_px_mean"]) / denominator * 100.0 if abs(denominator) > 1.0e-12 else math.nan
        )
        result.append(entry)
    return result


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    left, _, right = np.linalg.svd(sum(rotations))
    result = left @ right
    if np.linalg.det(result) < 0.0:
        left[:, -1] *= -1.0
        result = left @ right
    return result


def _parameter_evidence(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    phase_rotations: dict[str, list[np.ndarray]] = {"before": [], "after": []}
    phase_baselines: dict[str, list[np.ndarray]] = {"before": [], "after": []}
    for phase in ("before", "after"):
        for run in manifest["phases"][phase]["calibration_runs"]:
            rotation, translation = _transform(_load_yaml(machine_path(manifest, run["result"])))
            phase_rotations[phase].append(rotation)
            phase_baselines[phase].append(-rotation.T @ translation)
    mean_rotations = {phase: _mean_rotation(values) for phase, values in phase_rotations.items()}
    mean_baselines = {phase: np.mean(values, axis=0) for phase, values in phase_baselines.items()}
    rotation_change = np.degrees(_rotation_vector(mean_rotations["before"].T @ mean_rotations["after"]))
    rotation_std = {
        phase: np.std(
            [np.degrees(_rotation_vector(mean_rotations[phase].T @ value)) for value in phase_rotations[phase]], axis=0, ddof=1
        )
        for phase in ("before", "after")
    }
    baseline_change = (mean_baselines["after"] - mean_baselines["before"]) * 1000.0
    baseline_std = {
        phase: np.std(np.asarray(phase_baselines[phase]) * 1000.0, axis=0, ddof=1) for phase in ("before", "after")
    }
    rows = []
    for index, axis in enumerate("xyz"):
        for kind, unit, change, spread in (
            ("rotation", "deg", rotation_change, rotation_std),
            ("baseline_cam0", "mm", baseline_change, baseline_std),
        ):
            combined = math.hypot(float(spread["before"][index]), float(spread["after"][index]))
            rows.append({
                "parameter": f"{kind}_{axis}",
                "unit": unit,
                "before_sample_std": float(spread["before"][index]),
                "after_sample_std": float(spread["after"][index]),
                "change": float(change[index]),
                "combined_repeat_std": combined,
                "change_over_repeat_std": abs(float(change[index])) / combined if combined > 0.0 else math.inf,
            })
    return rows


def _alignment_figure(summary: list[dict[str, Any]], font: Any) -> plt.Figure:
    selected = [row for row in summary if row["variant"] in VARIANT_ORDER]
    figure, axis = plt.subplots(figsize=(12, 9), constrained_layout=True)
    positions = np.arange(len(selected))
    values = [row["alignment_rms_px_mean"] for row in selected]
    errors = [row["alignment_rms_px_sample_std"] for row in selected]
    axis.barh(positions, values, xerr=errors, alpha=0.8, capsize=3)
    axis.set_yticks(positions, [row["variant"] for row in selected])
    axis.invert_yaxis()
    axis.set_xlabel("三组 after 测试的 Alignment RMS 均值 [px]", fontproperties=font)
    axis.set_title("K/D、R 与物理基线 b 分量替换实验", fontproperties=font)
    axis.grid(axis="x", alpha=0.25)
    return figure


def _parameter_figure(rows: list[dict[str, Any]], font: Any) -> plt.Figure:
    figure, axes = plt.subplots(2, 1, figsize=(11, 8), constrained_layout=True)
    for axis, kind, label in ((axes[0], "rotation_", "旋转变化 [deg]"), (axes[1], "baseline_cam0_", "物理基线变化 [mm]")):
        selected = [row for row in rows if row["parameter"].startswith(kind)]
        names = [row["parameter"].rsplit("_", 1)[-1].upper() for row in selected]
        values = [row["change"] for row in selected]
        errors = [row["combined_repeat_std"] for row in selected]
        axis.bar(names, values, yerr=errors, capsize=5)
        axis.axhline(0.0, color="black", linewidth=0.8)
        axis.set_ylabel(label, fontproperties=font)
        axis.grid(axis="y", alpha=0.25)
    figure.suptitle("三次重复标定均值的外参变化；误差棒为前后样本标准差合成", fontproperties=font)
    return figure


def _conclusions(summary: list[dict[str, Any]], evidence: list[dict[str, Any]]) -> list[str]:
    by_name = {row["variant"]: row for row in summary}
    old = by_name["before_kd_before_t"]["alignment_rms_px_mean"]
    new = by_name["after_kd_after_t"]["alignment_rms_px_mean"]
    old_kd_new_t = by_name["before_kd_after_t"]["alignment_rms_px_mean"]
    new_kd_old_t = by_name["after_kd_before_t"]["alignment_rms_px_mean"]
    rotation_variants = [by_name[f"before_kd_rotation_{axis}"] for axis in "xyz"]
    best_axis = min(rotation_variants, key=lambda row: row["alignment_rms_px_mean"])["variant"].rsplit("_", 1)[-1].upper()
    strongest = max((row for row in evidence if row["parameter"].startswith("rotation_")), key=lambda row: row["change_over_repeat_std"])
    return [
        f"完整老化前参数在 after 测试上的 Alignment 均值为 {old:.6f} px，完整老化后参数为 {new:.6f} px。",
        f"保留老化前 K/D、只换老化后 T 后为 {old_kd_new_t:.6f} px；只换老化后 K/D、保留老化前 T 后为 {new_kd_old_t:.6f} px。因此当前退化主要由双目外参变化解释。",
        f"单轴旋转替换中 {best_axis} 轴给出最低 Alignment，说明它是最主要的一阶方向；完整 R 仍用于判断多轴耦合。",
        f"三次重复标定中最显著旋转分量为 {strongest['parameter']}，变化量是前后重复标准差合成值的 {strongest['change_over_repeat_std']:.2f} 倍。",
        "这些实验能定位数学参数块，不能仅凭图像把支架形变、相机内部光轴漂移、温度和重力姿态效应唯一分开；物理复测必须独立采集。",
    ]


def _physical_sop(machine_id: str) -> str:
    return f"""# {machine_id} 双目外参物理复测清单

本清单用于区分支架／安装链形变、相机内部光学倾斜、温度效应和重力姿态效应。当前报告中的混合参数实验只使用已有图像，不能替代这些采集。

## 固定条件

- 不拆卸、不调整螺丝；先静置 30 min，记录两相机壳体温度、环境温度、供电和安装姿态。
- 每个条件采集三组标定集和三组独立测试集；标定集与测试集不能共用图像。
- 保存相机原始时间戳、曝光、增益、温度、支撑方式和操作人；保持 AprilGrid、焦距和分辨率不变。
- 比较时固定 K/D，仅重新求双目 T；报告 R 的 X/Y/Z 旋转向量、cam0 坐标系物理基线 b、重复标准差及独立测试 Alignment。

## 实验顺序

1. **原姿态重复性**：同一支撑、同一温度再做 3+3 组，确认当前变化能否重复出现。
2. **距离分离**：近／中距离使用 AprilGrid，建议 2 m、4 m、6 m；远距离使用大于 30 m 的特征或准直目标。远距离对旋转敏感、对毫米级基线变化不敏感。
3. **温度循环**：冷机、稳定热机、冷却恢复各做 3+3 组，记录温度曲线。若参数随温度可逆变化，优先判断热效应。
4. **支撑与重力方向**：保持相机组件不拆卸，改变整机支撑或重力姿态后重复采集。若参数随姿态可逆变化，优先判断结构柔性。
5. **独立测量**：若条件允许，以不差于 0.005° 的角度测量和 0.02 mm 的位置测量检查相机壳体／安装面；再用远距离光轴测量区分壳体未动但光轴变化的情况。

## 建议目录

```text
{machine_id}/physical_diagnosis/
  stable_repeat/{{calibration,test}}/stereo_*
  distance_2m/{{calibration,test}}/stereo_*
  distance_4m/{{calibration,test}}/stereo_*
  distance_6m/{{calibration,test}}/stereo_*
  distance_far/{{calibration,test}}/stereo_*
  temperature_cold/{{calibration,test}}/stereo_*
  temperature_warm/{{calibration,test}}/stereo_*
  temperature_cooled/{{calibration,test}}/stereo_*
  gravity_reference/{{calibration,test}}/stereo_*
  gravity_rotated/{{calibration,test}}/stereo_*
  capture_conditions.csv
```

`capture_conditions.csv` 至少记录 `dataset_id,condition,ambient_temperature_c,cam0_temperature_c,cam1_temperature_c,exposure_us,gain,support,orientation,operator,notes`。
"""


def generate_diagnosis_report(
    manifest: dict[str, Any], rows: list[dict[str, Any]], metadata: dict[str, Any], force: bool = False
) -> dict[str, Any]:
    destination = machine_path(manifest, "result/aging_vibration_study/extrinsic_diagnosis/report")
    existing = destination / "report_manifest.json"
    if existing.is_file() and not force:
        stored = json.loads(existing.read_text(encoding="utf-8"))
        return {"output": str(destination), "html": str(destination / stored["html"]), "pdf": str(destination / stored["pdf"]), "reused": True}
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".extrinsic-diagnosis-", dir=destination.parent))
    (stage / "charts").mkdir()
    font = _font()
    summary = _summary(rows)
    evidence = _parameter_evidence(manifest)
    conclusions = _conclusions(summary, evidence)
    figures = (
        ("alignment_sensitivity.png", _alignment_figure(summary, font)),
        ("extrinsic_change.png", _parameter_figure(evidence, font)),
    )
    try:
        encoded = {}
        for name, figure in figures:
            path = stage / "charts" / name
            figure.savefig(path, dpi=180, bbox_inches="tight")
            encoded[name] = base64.b64encode(path.read_bytes()).decode("ascii")
        _write_csv(stage / "verification_results.csv", rows)
        _write_csv(stage / "verification_summary.csv", summary)
        _write_csv(stage / "parameter_evidence.csv", evidence)
        _write_json(stage / "variant_definitions.json", metadata)
        (stage / "PHYSICAL_FOLLOWUP_SOP_ZH.md").write_text(_physical_sop(manifest["machine_id"]), encoding="utf-8")

        summary_columns = [
            ("variant", "参数组合"), ("alignment_rms_px_mean", "Alignment 均值 [px]"),
            ("alignment_rms_px_sample_std", "样本标准差 [px]"), ("alignment_recovery_percent", "恢复比例 [%]"),
            ("cam0_rms_px_mean", "cam0 RMS"), ("cam1_rms_px_mean", "cam1 RMS"), ("stereo_rms_px_mean", "双目 RMS"),
        ]
        evidence_columns = [
            ("parameter", "分量"), ("change", "前后变化"), ("unit", "单位"),
            ("before_sample_std", "before 标准差"), ("after_sample_std", "after 标准差"),
            ("change_over_repeat_std", "变化/重复波动"),
        ]
        conclusions_html = "".join(f"<li>{line}</li>" for line in conclusions)
        images = "".join(f'<img src="data:image/png;base64,{encoded[name]}" alt="{name}">' for name, _ in figures)
        title = f"{manifest['machine_id']} 双目外参成因诊断"
        html_document = f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><title>{title}</title>
<style>body{{font-family:sans-serif;max-width:1480px;margin:32px auto;line-height:1.55}}table{{border-collapse:collapse;width:100%;font-size:12px}}th,td{{border:1px solid #bbb;padding:5px}}th{{background:#eef}}img{{max-width:100%;margin:12px 0}}code{{background:#eee;padding:2px 4px}}</style></head><body>
<h1>{title}</h1><p>所有参数组合均在同一组三个 after 独立测试集上重新检测角点并固定参数验证，没有重新优化标定参数。物理基线统一在 cam0 坐标系表示：<code>b=-Rᵀt</code>，构造变体时使用 <code>t=-Rb</code>。</p>
<h2>数据支持的结论</h2><ol>{conclusions_html}</ol>
<h2>参数组合验证</h2>{_html_table(summary, summary_columns)}
<h2>三次重复标定的参数证据</h2>{_html_table(evidence, evidence_columns)}
<h2>图表</h2>{images}
<h2>结论边界</h2><p>当前证据可判断 K/D 与外参贡献、R 与物理基线贡献及主要分量；不能唯一识别机械支架、相机内部光学、温度或重力原因。后续采集步骤见 <a href="PHYSICAL_FOLLOWUP_SOP_ZH.md">物理复测清单</a>。</p>
</body></html>'''
        stem = manifest["machine_id"] + "_extrinsic_diagnosis"
        html_name, pdf_name, markdown_name = stem + ".html", stem + ".pdf", stem + ".md"
        (stage / html_name).write_text(html_document, encoding="utf-8")
        markdown_lines = [f"# {title}", "", "## 数据支持的结论", ""] + [f"{index}. {line}" for index, line in enumerate(conclusions, 1)]
        markdown_lines += ["", "## 方法", "", "同一组三个 after 独立测试集固定参数验证；`b=-R^T t`，构造时 `t=-Rb`。完整数值见 `verification_summary.csv` 和 `parameter_evidence.csv`。", "", "![参数组合](charts/alignment_sensitivity.png)", "", "![外参变化](charts/extrinsic_change.png)", "", "物理复测见 [PHYSICAL_FOLLOWUP_SOP_ZH.md](PHYSICAL_FOLLOWUP_SOP_ZH.md)。", ""]
        (stage / markdown_name).write_text("\n".join(markdown_lines), encoding="utf-8")
        with PdfPages(stage / pdf_name) as pdf:
            page = plt.figure(figsize=(11.69, 8.27))
            page.text(0.06, 0.93, title, fontsize=20, fontproperties=font)
            page.text(0.06, 0.86, "\n\n".join(conclusions), va="top", fontsize=10, linespacing=1.4, fontproperties=font)
            page.text(0.06, 0.12, "参数组合在同一组三个 after 测试集上固定验证；未重新优化。", fontsize=9, fontproperties=font)
            pdf.savefig(page)
            plt.close(page)
            for _, figure in figures:
                pdf.savefig(figure)
        files = [
            html_name, pdf_name, markdown_name, "PHYSICAL_FOLLOWUP_SOP_ZH.md", "verification_results.csv",
            "verification_summary.csv", "parameter_evidence.csv", "variant_definitions.json",
            "charts/alignment_sensitivity.png", "charts/extrinsic_change.png", "report_manifest.json", ".inventory.json",
        ]
        report_manifest = {
            "schema_version": "1.0.0", "kind": "stereo_extrinsic_diagnosis_report", "machine_id": manifest["machine_id"],
            "html": html_name, "pdf": pdf_name, "markdown": markdown_name, "files": files[:-2],
        }
        _write_json(stage / "report_manifest.json", report_manifest)
        _write_json(stage / ".inventory.json", sorted(files))
        _managed_replace(destination, stage, force)
        return {"output": str(destination), "html": str(destination / html_name), "pdf": str(destination / pdf_name), "markdown": str(destination / markdown_name), "reused": False}
    except Exception:
        shutil.rmtree(stage, ignore_errors=True)
        raise
    finally:
        for _, figure in figures:
            plt.close(figure)


def run_extrinsic_diagnosis(
    manifest: dict[str, Any], dry_run: bool = False, force: bool = False
) -> dict[str, Any]:
    if "after" not in manifest["phases"]:
        raise ValueError("after phase is required for extrinsic diagnosis")
    inspection = inspect_study(manifest)
    if inspection["errors"]:
        raise ValueError("study input validation failed: " + "; ".join(inspection["errors"]))
    before_path = machine_path(manifest, representative_calibration(manifest, "before")["result"])
    after_path = machine_path(manifest, representative_calibration(manifest, "after")["result"])
    if not dry_run and (not before_path.is_file() or not after_path.is_file()):
        raise FileNotFoundError("representative before/after calibration result is missing")
    before_document = _load_yaml(before_path)
    after_document = _load_yaml(after_path)
    variants, metadata = build_extrinsic_variants(before_document, after_document)
    metadata["before_source"] = str(before_path)
    metadata["after_source"] = str(after_path)
    root = machine_path(manifest, "result/aging_vibration_study/extrinsic_diagnosis")
    calibrations = root / "calibrations"
    logs = root / "logs"
    if not dry_run:
        calibrations.mkdir(parents=True, exist_ok=True)
        for name, document in variants.items():
            _write_yaml(calibrations / f"{name}.yaml", document)
        _write_json(root / "variant_definitions.json", metadata)

    commands = []
    rows = _reference_rows(manifest) if not dry_run else []
    for name in (variant for variant in VARIANT_ORDER if variant in variants):
        calibration = calibrations / f"{name}.yaml"
        for run in manifest["phases"]["after"]["test_runs"]:
            output = root / "verification" / name / f"repeat_{int(run['repeat']):02d}_{run['dataset_id']}"
            if _completed_run(output) and not force:
                print(f"reuse completed extrinsic diagnosis: {output}")
            else:
                command = build_verify_command(
                    manifest, calibration, machine_path(manifest, run["dataset"]), output, force
                )
                commands.append(command)
                _run_logged(manifest, command, logs / f"{name}_repeat_{int(run['repeat']):02d}.log", dry_run)
                if not dry_run and not _completed_run(output):
                    raise RuntimeError(f"extrinsic diagnosis did not produce a complete result: {output}")
            if not dry_run:
                rows.append(_verification_row(manifest, name, int(run["repeat"]), run["dataset_id"], output))
    report = None if dry_run else generate_diagnosis_report(manifest, rows, metadata, force)
    return {"commands": commands, "report": report, "output": str(root)}
