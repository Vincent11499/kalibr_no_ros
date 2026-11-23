---
name: kalibr-stereo-aging-validation
description: Run or report this repository's pinhole-equi stereo aging/vibration study with three repeated calibration captures and three repeated fixed-parameter test captures per available phase. Do not use for Camera-IMU or rolling-shutter jobs.
---

# 双目老化／振动验证

使用正式入口 `kalibr-noros tools stereo-aging-validation`。实现保留在
`src/python/kalibr_tools/aging_vibration/`；不要恢复废弃的
`tools/kalibr_stereo_aging_validation.py`，也不要在 skill、临时脚本或测试目录复制实验编排和报告计算。

## 数据目录

```text
<machine>/
  before/
    calibration/stereo_*/   # 恰好三组
    test/stereo_*/          # 恰好三组
  after/                    # 老化后数据到位时再添加，结构相同
    calibration/stereo_*/
    test/stereo_*/
  study_manifest.yaml
  result/aging_vibration_study/
```

每个 `stereo_*` 必须是已有的目录数据集并包含 `dataset.yaml`。工具不会移动、改名或删除采集数据。新 manifest 只接受三组标定和三组测试结构，不读取旧版单标定集 manifest。

## 固定方法

- 只使用当前入口所属安装前缀的 `bin/kalibr-noros`。
- 固定参数双目验证由正式 `kalibr-noros tools fixed-stereo-validation` 执行，不复制检测或误差计算实现。
- 标定使用显式 pinhole-equi 初值和 `solver_mode: full_batch`。
- `detector_processes: 8`、`optimizer_threads: 4`，所有标定与验证逐项串行。
- 保存诊断、指标以及最多 30 帧／30 对的检测、去畸变和极线对齐图；去畸变不裁剪。
- 每个阶段按目录名排序，三组都用于标定重复性；第 1 组标定结果是该阶段唯一代表参数。
- before 阶段用 before 第 1 组结果验证三组 before 测试集。
- after 阶段先用 before 代表参数验证三组 after 测试集，再完成三组 after 标定，并用 after 第 1 组结果验证同一批 after 测试集。
- 对比 before标定参数与after标定参数。
- 没有用户验收阈值时只输出数值与趋势，不使用内建参考等级宣称生产合格。

## 汇总报告指标

before 基线及 before/after 对比的 HTML、PDF 和 CSV 均需列出每组每目的
`RMS [px]`、`P95 [px]`、`Line [px]`、`Line P95 [px]`，以及每组双目的
`Optical RMS [px]`、`Optical RMS P95 [px]`。重投影按二维角点距离统计；
Line 为去畸变后行列角点到 TLS 直线的距离 RMS；Optical 为极线对齐残差，
其 P95 指同一残差绝对值的第95百分位，不是逐帧 RMS 的百分位。
只读取已有结果；缺少汇总分布时注明未保存，不用逐帧百分位平均值代替，也不自动重跑标定或检测。
新执行的固定参数 test 必须保存逐目总体 P95 和直线度分布（RMS、P95）；
历史 test 缺少这些统计且用户明确授权重跑时，仅重跑固定参数验证，复用原标定参数并重建汇总报告。

`cam0_cam1` 外参保留完整 `T_cn_cnm1` 矩阵，同时增加单位旋转轴和角度（deg）。
方向为 cam0 → cam1，平移单位 m；零旋转的轴约定为 `[1,0,0]`。
仅更新报告时使用 `stereo-aging-validation report --manifest ... --force`，
替换前检查受管输出清单，不触发 `run` 或重新标定。

## 命令

```bash
kalibr-noros tools stereo-aging-validation init \
  --machine-root <machine>

kalibr-noros tools stereo-aging-validation inspect \
  --manifest <machine>/study_manifest.yaml

kalibr-noros tools stereo-aging-validation run \
  --manifest <machine>/study_manifest.yaml \
  --phase before

# 老化后目录完整时
kalibr-noros tools stereo-aging-validation refresh \
  --manifest <machine>/study_manifest.yaml

kalibr-noros tools stereo-aging-validation run \
  --manifest <machine>/study_manifest.yaml \
  --phase after

# before/after 已完成后，在三组 after 测试集上诊断 K/D、R 与物理基线分量
kalibr-noros tools stereo-aging-validation diagnose-extrinsics \
  --manifest <machine>/study_manifest.yaml
```

`run` 会复用有完整 `run_manifest.json` 的已完成步骤。不要使用 `--force`，除非用户明确要求替换受管输出。每步日志位于 `result/aging_vibration_study/logs/`。

`diagnose-extrinsics` 不重新标定。它以 before/after 第 1 组代表结果构造分析专用参数，
统一用 cam0 坐标系物理基线 `b=-R^T*t`，在三组 after 独立测试集上分别验证 K/D、
完整 R、旋转 X/Y/Z、物理基线 X/Y/Z、基线长度和方向的贡献。输出位于
`result/aging_vibration_study/extrinsic_diagnosis/`，其中报告还包含三次重复标定相对
重复波动的证据与待采集物理复测清单。分析专用 YAML 不能作为生产标定结果发布。

## 内存与停止条件

每一步默认要求 Linux 可用内存至少 12 GiB；能读取 Windows 状态时还要求可用虚拟内存至少 16 GiB。低于阈值时停止并提示关闭 Foxglove 等占用内存的软件，不主动终止外部程序。

相机 ID、分辨率、模型、标定板、阶段目录数量或验证方法不一致时停止，不自行猜测。完成后报告 HTML、PDF、CSV、图表、日志路径以及实际执行／复用的步骤。
