# 普通多目标定求解模式

本文说明 `camera_calibration` 在双目/多目任务中的阶段、可选求解模式、初始化与固定内参的组合，以及输出语义。这里的“多目”包含双目。

## 1. 原生阶段

普通相机标定按以下顺序准备联合问题：

1. **S0 角点检测与同步 view 建立**：每目独立检测，再按时间容差和共同全局角点 ID 建立多目 view。
2. **S1 单目初始化**：每目求 K/D；提供部分 K/D seed 时，以 seed 为起点运行同一单目 LM。
3. **S2 baseline 初始化**：缺少相邻外参 seed 时，通过共视相机对的 PnP 和 stereo LM 补齐相机链；相邻外参全部提供时跳过这一阶段。
4. **S3 full-batch refinement**：双目/多目将全部同步 view、全部检测成功角点、所有相机 K/D、相邻 baseline 和每帧标定板位姿放入一个联合 LM；单目不执行此多目阶段。
5. **S4 增量选择与优化**：仅 `incremental` 模式执行。按观测顺序逐个创建 batch，通过信息增益、秩检查、回滚和可选异常点处理确定最终 view。

S3 与 S4 使用同一批共享相机对象。因此 S3 更新的是随后输出或继续进入 S4 的 K/D 状态，而不是一份临时副本；S3 返回的相邻 baseline 同样是 S4 的起点。

## 2. `calibration.solver_mode`

```yaml
calibration:
  solver_mode: incremental
```

允许值：

| 值 | 支持范围 | 最终结果来源 | view/角点选择 |
|---|---|---|---|
| `incremental` | 普通单目、双目、多目 | S4 原生增量估计器 | 原生 shuffle、信息增益、回滚、异常角点和最终过滤 |
| `full_batch` | 仅普通双目、多目 | S3 联合 LM 的终值 | 全部同步 view 和其中全部检测成功角点 |

省略时默认 `incremental`，与升级前行为一致。`camera_rolling_shutter_calibration`、Camera–IMU 和 RS Camera–IMU 不接受此字段，因为它们有各自的联合求解器。

### 2.1 `incremental`

双目/多目在 S3 完成后创建原生 `IncrementalEstimator`，单目在 S1 后直接创建；随后都运行 S4。下列字段仅作用于这一阶段：

- `shuffle`
- `qr_tolerance`
- `information_gain_tolerance`
- `min_views_for_outlier_statistics`
- `remove_outliers`
- `final_filtering`
- `blake_zisserman`

`shuffle: true` 使用运行时随机顺序；项目不再提供 `shuffle_seed`。需要确定性顺序时配置 `shuffle: false`。

### 2.2 `full_batch`

S3 成功且在最大迭代数内收敛后，程序直接将该问题作为最终结果：

- 不创建 `IncrementalEstimator`；
- 不调用 `addBatch()`；
- 不重新求解；
- 不执行 shuffle、信息增益筛选、增量回滚、Blake-Zisserman 或增量异常点删除；
- 最终使用全部同步 view 和全部检测成功角点；
- 在终值处只读重线性化一次，用于协方差和可观性分析，参数值不会更新。

若 task 显式包含前述增量专用字段，配置仍可读取，终端会列出未应用的字段。这样同一模板可以在两种模式间切换，同时避免用户误以为异常点策略已经生效。

S3 的线性求解失败、返回失败状态或达到最大迭代数时，任务失败，不发布成功结果。可观性文件仍区分数学秩与工程弱模态；未提供 seed 的 `full_batch` 不会因为 `weakly_observable` 被误判为求解失败。

## 3. 初始化与参数活动状态

task 只接受：

```yaml
initialization:
  path: initialization_pinhole_equi.yaml
```

所有值都是可优化初值。旧 `initialization.strategy` 和 CLI `--initialization-strategy` 已删除；显式填写会报错。初始化文件本身也不包含 `strategy`。

`calibration.freeze_intrinsics` 独立控制 K/D 是否固定：

- 默认 `false`：提供的 K/D 在 S1、S3 和适用的 S4 中继续优化；
- `true`：仅普通/RS 双目或多目可用，且每目必须提供完整 K/D；S1 跳过，S3/S4 中 projection 和 distortion 固定；baseline 与每帧标定板位姿仍为 active。

相邻外参 seed `T_cam_from_previous` 或 `T_cn_cnm1` 仅决定 S2 是否需要补齐 baseline。它在 S3 中始终继续优化。

## 4. 可用组合

下表中“完整 baseline”表示按相机列表顺序提供全部 $N-1$ 个 `T_cn_cnm1`。

| 初值与配置 | S1 单目 LM | S2 pairwise baseline | S3 full-batch | S4 incremental | K/D 最终状态 |
|---|---:|---:|---:|---:|---|
| 无 seed，`incremental` | 执行 | 执行 | 执行 | 执行 | active |
| 部分 K/D seed，baseline 不完整，`incremental` | 执行 | 执行 | 执行 | 执行 | active |
| K/D seed，完整 baseline，`incremental` | 执行 | 跳过 | 执行 | 执行 | active |
| 完整 K/D，`freeze_intrinsics: true`，baseline 不完整，`incremental` | 跳过 | 执行 | 执行 | 执行 | fixed |
| 完整 K/D，`freeze_intrinsics: true`，完整 baseline，`incremental` | 跳过 | 跳过 | 执行 | 执行 | fixed |
| 无 seed，`full_batch` | 执行 | 执行 | 执行并作为终值 | 跳过 | active |
| 部分 K/D seed，`full_batch` | 执行 | 按缺失项执行 | 执行并作为终值 | 跳过 | active |
| 完整 K/D，`freeze_intrinsics: true`，`full_batch` | 跳过 | 按缺失项执行 | 执行并作为终值 | 跳过 | fixed |

`full_batch` 至少需要两台相机。单目任务必须使用 `incremental`。

## 5. 配置示例

使用全部同步 view 直接输出 S3 结果：

```yaml
calibration:
  solver_mode: full_batch
  freeze_intrinsics: false
  window_half_size_px: 7
  max_displacement_px: 1.0
  synchronization_tolerance_s: 0.0001
```

使用可靠 K/D，固定内参并继续求 baseline：

```yaml
initialization:
  path: initialization_pinhole_equi.yaml
calibration:
  solver_mode: full_batch
  freeze_intrinsics: true
```

保留原生增量选择：

```yaml
calibration:
  solver_mode: incremental
  shuffle: false
  information_gain_tolerance: 0.2
  remove_outliers: true
  final_filtering: true
  blake_zisserman: false
```

命令不变：

```bash
install/release/bin/kalibr-noros calibrate cameras \
  --config path/to/stereo_camera_calibration_task.yaml \
  --output-dir path/to/result
```

## 6. 输出与诊断

两种模式复用同一结果、指标、HTML/PDF、观测归档、poses 和可观性输出代码。`full_batch` 的只读适配层保留：

- 最终 K/D 与相邻 `T_cn_cnm1`；
- 全部最终 view 及每帧板位姿；
- 每目每角点测量、预测和残差；
- 完整优化问题与 S3 求解返回值；
- 终值协方差和 reduced Fisher information 可观性结果。

因此结果 YAML 的 `rms`、双目 `alignment`、报告和观测归档与 `incremental` 走同一套后处理。`full_batch` 不产生增量 selection event；输入 view 数应等于最终使用 view 数。

终端始终显示任务、数据集、相机数量、线程预算、每目检测汇总、同步 view 数和最终交付路径。`output.verbose: true` 额外显示各初始化/优化/可观性/报告阶段耗时、K/D、baseline、S3 的 view/角点数、迭代数、`JStart/JFinal` 及 active/fixed 参数块。完整原生日志仍写入诊断日志，不逐帧刷屏。

## 7. 后续图像与角点剔除

当前 `full_batch` 有意不定义新的剔除策略。最终 view、角点、残差和优化问题已经显式保留，后续可在 S3 前增加“生成保留掩码”步骤，再用同一 full-batch 构造与输出适配层求解。扩展时应满足：

1. 选择规则独立于输出后处理；
2. 明确记录输入、保留和剔除的 view/角点及原因；
3. 最终只求解一次，并在终值处只读评价；
4. 不把增量接受语义伪装成 full-batch 语义；
5. 默认未启用时保持本版本的全部 view/角点行为。
