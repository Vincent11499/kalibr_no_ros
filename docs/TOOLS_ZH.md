# `kalibr-noros tools` 工具总览

本文统一说明当前随安装包交付的诊断与实验工具。一级工具列表、参数和默认值以
`src/python/kalibr_tools/cli.py` 为准；运行时可随时核对：

```bash
kalibr-noros tools --help
kalibr-noros tools <工具名> --help
```

这些命令不是普通标定入口。普通相机、Camera–IMU 和滚动快门标定仍使用
`kalibr-noros calibrate ...`。

## 1. 当前工具

| 工具 | 主要用途 | 是否运行标定优化 |
|---|---|---|
| `imu-allan` | 从静止 IMU CSV 估计四项噪声参数并生成报告 | 只估计噪声，不做 Camera–IMU 标定 |
| `fixed-stereo-validation` | 在独立测试集上固定双目 K/D/T 评价重投影、直线度和极线对齐 | 否 |
| `stereo-tag-distance` | 使用固定双目 K/D/T 三角化两个 AprilTag 的三维中心距离 | 否 |
| `rolling-shutter-diagnosis` | 判断动态数据是否稳定支持逐行曝光时间模型 | 条件满足后编排多组正式 RS 对照 |
| `stereo-aging-validation` | 用三组标定和三组测试评价双目老化／振动前后稳定性 | 编排相机标定和固定参数验证 |

## 2. 共通规则

- 路径必须是当前操作系统可直接访问的路径。WSL 中使用 `/mnt/q/...`；工具不会把
  `Q:\...` 静默转换为 WSL 路径。
- 实际数据默认只读。输出应放在数据集外部或各工具规定的同级 `result/` 目录。
- `--force` 与 `--overwrite` 不是通用参数，覆盖规则见各工具说明。没有明确替换需求时不要使用。
- 报告中的参考等级不是生产放行规则。没有用户明确阈值时，只能报告数值、趋势和证据是否充分。
- 复杂工作流先运行 `--dry-run`；目前只有滚动快门诊断及老化工作流相关子命令提供该参数。

## 3. `imu-allan`

### 3.1 功能与命令

读取静止 IMU CSV，计算 Allan deviation，输出可供 Camera–IMU task 使用的
`imu_noise.yaml`，以及 HTML/PDF 报告。

```bash
kalibr-noros tools imu-allan \
  --dataset /data/imu_YYMMDD_HHMM \
  --output-dir /data/result/imu_allan
```

也可直接指定 CSV，并选择相对第一条样本的时间区间：

```bash
kalibr-noros tools imu-allan \
  --dataset /data/imu/imu0.csv \
  --output-dir /data/result/imu_allan_1h_23h \
  --start-h 1 --end-h 23 \
  --topic /imu/data
```

目录输入固定读取 `imu/imu0.csv`。CSV 前七列必须依次为：

```csv
timestamp_ns,wx,wy,wz,ax,ay,az
```

陀螺仪单位为 `rad/s`，加速度单位为 `m/s²`。时间戳必须是严格递增的整数纳秒。

### 3.2 输出与注意事项

```text
<output-dir>/
├── imu_noise.yaml
├── imu_allan.report.html
├── imu_allan.report.pdf
└── work/
```

- 数据必须是连续静止采集；工具不自动判断设备是否静止。
- 通常至少需要约 2000 s，最低样本数只保证能够计算，不保证长时间尺度拟合可信。
- 不读取 bag，也不根据 `dataset.yaml` 自动选择 IMU。
- 输出目录、同级 `.inprogress` 或 `.failed` 任一存在时都会拒绝运行；没有覆盖选项。
- 生成的噪声参数仍需人工审核，工具不会修改 task 或自动启动 Camera–IMU 标定。

完整算法、单位和拟合限制见 [IMU Allan 噪声分析](IMU_ALLAN_VARIANCE_ZH.md)。

## 4. `fixed-stereo-validation`

### 4.1 功能与命令

在独立目录测试集上重新检测标定板，固定已有标定结果中的 K、D 和
`T_cn_cnm1`。每帧只估计标定板位姿，不重新优化相机参数。

```bash
kalibr-noros tools fixed-stereo-validation \
  --calibration baseline=/data/camera_calibration.yaml \
  --dataset /data/stereo_test_YYMMDD_HHMM \
  --target /data/aprilgrid.yaml \
  --output-dir /data/result/fixed_validation \
  --visualizations \
  --max-frames-per-camera 30 \
  --max-pairs 30
```

`--calibration` 可重复，用不同标签在同一测试集上比较多套参数。

### 4.2 指标与输出

- 每目重投影 RMS、P95、Max；
- 去畸变后行列角点到 TLS 直线的直线度 RMS、P95；
- 两目独立 PnP 残差的合并 RMS/P95/Max；
- 固定 K/D/T 校正后，非视差方向的 Optical Alignment RMS/P95/Max；
- `fixed_camera_validation.json`、CSV、中文 README 和运行清单；
- 显式使用 `--visualizations` 时，保存角点、去畸变和双目极线对齐图。

### 4.3 注意事项

- 标定输入必须是 `calibration_type: cameras` 的正式 `calibration_result`，不是机器汇总
  `stereo_imu_calib.yaml`。
- `stereo_combined_rms_px` 允许左右目独立估计板位姿，不是共享板位姿的双目联合重投影 RMS，
  不能单独检验外参；外参主要看 Optical Alignment。
- P95 是所有有限逐角点二维误差范数的合并 P95，不是逐帧 P95 的平均值。
- `--visualizations` 只影响证据图数量，不减少统计所用帧。当前默认最多输出每目 20 帧、
  20 个双目图对；建议实验命令显式填写数量。
- 角点亚像素默认 `window_half_size_px=2`、`max_displacement_px=sqrt(1.5)`；验证训练结果时
  应与训练任务保持一致。同步容差默认 `0.0002 s`。
- `--force` 只覆盖具有完整 `.inventory.json` 且没有未登记文件的受管输出目录。

指标公式和边界见 [固定参数验证与标定诊断](FIXED_CAMERA_VALIDATION_ZH.md)。

## 5. `stereo-tag-distance`

### 5.1 功能与命令

使用固定的 pinhole-equi K/D/T 检测两个 tag36h11 AprilTag，分别三角化四个角点，并计算
两个三维中心在 cam0 坐标系中的欧氏距离。

```bash
kalibr-noros tools stereo-tag-distance \
  --dataset /data/stereo_YYMMDD_HHMM \
  --calibration /data/camera_calibration.yaml
```

默认测量 tag 0 到 tag 1。其他 ID 使用 `--tag0`、`--tag1`；默认配对容差为 `10 ms`。

### 5.2 输出与注意事项

```text
<dataset-parent>/result/distance_YYMMDD_HHMM/
├── distance.csv
├── distance_summary.csv
└── distance_detections/
    ├── cam0/
    └── cam1/
```

- 数据集必须包含 `dataset.yaml`，目录名必须以 `_YYMMDD_HHMM` 结束。
- 有 `meta/pairs.csv` 时优先使用其配对，否则按时间容差配对。
- 标定文件可以是正式相机标定结果，也可以是机器 `stereo_imu_calib.yaml`。
- 两目必须是 `pinhole-equi`／`pinhole-equidistant` 和四参数 KB4，且
  `T_cn_cnm1` 方向为 cam0 → cam1。
- 使用 Kalibr 原生 ETHZ AprilTag2 初始角点，不执行 `cornerSubPix`。
- 每个 tag 中心是四个三角化角点的均值；距离单位为 mm。汇总样本标准差使用 `ddof=1`。
- 使用 `--output-csv`、`--summary-csv`、`--detection-dir` 可修改输出位置；已有同名输出时
  默认停止，明确允许替换时才使用 `--overwrite`。
- 部分图对失败时仍保留逐图对信息，但命令返回非零；失败值不会写成 0。

## 6. `rolling-shutter-diagnosis`

### 6.1 功能与命令

从普通相机或普通 Camera–IMU task 出发，检查动态数据中的误差是否稳定支持
“角点时刻随图像行坐标变化”的解释。

```bash
kalibr-noros tools rolling-shutter-diagnosis \
  --config /data/rolling_shutter_diagnosis.yaml \
  --output-dir /data/result/rolling_shutter_diagnosis
```

建议先执行：

```bash
kalibr-noros tools rolling-shutter-diagnosis \
  --config /data/rolling_shutter_diagnosis.yaml \
  --output-dir /data/result/rolling_shutter_diagnosis \
  --dry-run
```

第一阶段执行普通任务，并进行严格前后分块、列方向负对照和按帧 bootstrap。只有所有相机
筛查通过且 `capture.motion: continuous` 时，第二阶段才串行运行固定零值、正反候选、
多初值和轨迹带宽等正式 RS 对照。

### 6.2 输出与注意事项

主要输出包括 `screening.json`、`formal_comparison.json`、`result.json`、CSV、HTML/PDF、
图表、配置、日志和各组原生结果。只有结论为 `important` 时才生成建议 RS task。

- 自动结论只有 `important`、`inconclusive`、`unsupported`；三者都不是快门硬件认证或
  生产放行结果。
- `stop_and_go` 或 `unknown` 采集最多得到 `inconclusive`。
- Camera–IMU 源 task 必须显式提供 `camera_calibration.path`。
- `--dry-run` 只能生成第一阶段普通任务命令，后续 seed 依赖第一阶段真实观测。
- 正式对照全部串行，但纯视觉最多约 10 组、Camera–IMU 约 6 组，仍可能耗时并占用较多内存。
- 默认断点复用完整结果；`--force` 只覆盖带本工具 manifest 的受管目录。
- `RS compensated pair residual` 使用同一批数据拟合出的轨迹和行时间，是样本内指标，不能
  单独证明参数能推广到独立数据。

配置、门槛及各指标公式见
[滚动快门行时间诊断工具](ROLLING_SHUTTER_DIAGNOSIS_TOOL_ZH.md)。

## 7. `stereo-aging-validation`

### 7.1 数据结构

用于 pinhole-equi 双目老化／振动实验，不用于 Camera–IMU 或滚动快门任务：

```text
<machine>/
├── before/
│   ├── calibration/stereo_*/   # 恰好三组
│   └── test/stereo_*/          # 恰好三组
├── after/                         # 数据到位后再添加
│   ├── calibration/stereo_*/   # 恰好三组
│   └── test/stereo_*/          # 恰好三组
└── result/aging_vibration_study/
```

每个 `stereo_*` 都必须是带 `dataset.yaml` 的目录数据集。

### 7.2 推荐流程

```bash
kalibr-noros tools stereo-aging-validation init \
  --machine-root /data/machine

kalibr-noros tools stereo-aging-validation inspect \
  --manifest /data/machine/study_manifest.yaml

kalibr-noros tools stereo-aging-validation run \
  --manifest /data/machine/study_manifest.yaml \
  --phase before

# after 数据到位后
kalibr-noros tools stereo-aging-validation refresh \
  --manifest /data/machine/study_manifest.yaml

kalibr-noros tools stereo-aging-validation run \
  --manifest /data/machine/study_manifest.yaml \
  --phase after

# 只根据已有证据重建完整报告；明确允许替换受管报告时才加 --force
kalibr-noros tools stereo-aging-validation report \
  --manifest /data/machine/study_manifest.yaml \
  --full --force

# 分解 K/D、旋转与 cam0 坐标系物理基线分量的贡献
kalibr-noros tools stereo-aging-validation diagnose-extrinsics \
  --manifest /data/machine/study_manifest.yaml
```

子命令含义：

| 子命令 | 功能 |
|---|---|
| `init` | 发现数据并创建新的 `study_manifest.yaml` |
| `inspect` | 检查数据结构和已完成证据 |
| `refresh` | 将新采集的 after 数据登记到 manifest |
| `run` | 完成指定阶段需要的标定、固定参数验证和报告 |
| `calibrate` | 只执行指定阶段的三组标定 |
| `verify` | 指定一阶段代表参数去验证指定阶段测试集 |
| `report` | 只读取已有证据生成报告，不重新检测或标定 |
| `diagnose-extrinsics` | 在三组 after 测试集上分解 K/D、旋转和物理基线分量贡献 |

### 7.3 固定口径与注意事项

- 每阶段按目录名排序，三组共同评价重复性；第 1 组结果是该阶段唯一代表参数。
- before 代表参数验证三组 before 测试集；after 阶段还会在相同 after 测试集上比较 before
  与 after 代表参数。
- 报告列出每目 RMS/P95、直线度 RMS/P95，每组双目 Optical RMS/P95，以及
  `T_cn_cnm1` 矩阵和单位旋转轴＋角度。
- `diagnose-extrinsics` 不重新标定。生成的分析专用 YAML 不能作为生产标定结果发布。
- 标定和验证逐项串行；内部固定使用 `detector_processes=8`、`optimizer_threads=4`。
- 每一步默认要求 Linux available 至少 12 GiB；能读取宿主状态时还要求 Windows 可用
  虚拟内存至少 16 GiB。
- `run` 支持断点复用。没有明确替换受管结果的要求时不要使用 `--force`。
- 没有用户验收阈值时只输出数值和趋势，不根据参考等级宣称生产合格。

## 8. 覆盖行为速查

| 工具 | 默认行为 | 明确替换时使用 |
|---|---|---|
| `imu-allan` | 输出目录或同级临时／失败目录已存在就停止 | 不支持覆盖，使用新目录 |
| `fixed-stereo-validation` | 非空输出目录停止 | `--force`，且目录必须完全受清单管理 |
| `stereo-tag-distance` | 任一目标 CSV 或检测图已存在就停止 | `--overwrite` |
| `rolling-shutter-diagnosis` | 复用完整步骤，拒绝非受管目录 | `--force` |
| `stereo-aging-validation` | 复用完整步骤，保护未登记文件 | 对相应子命令使用 `--force` |

## 9. 维护要求

新增、删除或修改正式工具时，同时检查：

1. `src/python/kalibr_tools/cli.py` 的命令、参数和默认值；
2. 本文的工具列表、命令示例、输出和覆盖行为；
3. [文档导航](README_ZH.md)；
4. 对应专题文档和仓库 skill；
5. CLI 解析测试及工具自身测试。

不要重新增加根目录 `tools/kalibr_*.py` 独立入口。正式实现保留在
`src/python/kalibr_tools/`，统一由 `kalibr-noros tools` 调用。
