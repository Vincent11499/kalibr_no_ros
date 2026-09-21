# 机器级一键标定工作流

本文说明面向产线的机器级入口：

```bash
kalibr-cedarobot machine --config user.yaml
```

它把数据集发现、task 生成、输入校验、双目标定、Camera–IMU 标定、结果复用和交付
串成一条可重复执行的命令。操作员不需要编辑普通 calibration task，也不需要手工把
双目结果路径填入 Camera–IMU task。底层 `kalibr-noros` 命令继续保留，供研发维护、
单项复现和排障使用。

机器工作流只编排既有求解器，不改变普通相机或 Camera–IMU 的阶段顺序、残差、活动
参数、优化器和异常点逻辑。

## 1. 操作员只需要做什么

工位维护人员随机器准备 `user.yaml`。操作员完成采集和数据复制后，在 WSL/Linux 中
重复执行同一条命令：

```bash
install/release/bin/kalibr-cedarobot machine \
  --config /path/to/user.yaml
```

不要求先同时准备双目和双目＋IMU 两套数据：

1. 机器目录中只有 `stereo_*` 时，命令完成双目内外参标定，随后以
   `waiting_for_dataset` 正常结束。
2. 以后把 `stereo_imu_*` 复制到同一个机器目录，再执行原命令。程序核对并复用此前
   的双目结果，只运行 Camera–IMU 阶段。
3. 两套数据一开始就齐全时，程序先完成双目，再把本次双目结果的精确路径交给
   Camera–IMU 阶段。
4. 两个阶段都已完成且数据、模板和软件来源一致时，命令复用结果，不重复求解。

`reference grade` 是参考显示，不是生产判定。双目求解和 YAML/HTML/PDF 交付完成后，
即使参考等级为 `poor` 或尚未配置生产规则，工作流仍可执行 Camera–IMU；最终是否放行
仍由正式规则和人工复核决定。

## 2. 机器目录约定

机器目录只扫描直接子目录，不递归进入 `result/`：

```text
mechine-01/
├── stereo_260916_2136/              双目内外参数据
│   ├── dataset.yaml
│   └── cameras/...
├── stereo_imu_260916_2200/          双目＋IMU 数据，可以稍后再加入
│   ├── dataset.yaml
│   ├── cameras/...
│   └── imu/imu0.csv
├── stereo_test_260916_2211/         独立验证数据，不参与自动标定
└── result/                           程序创建或复用
```

自动候选只接受严格名称：

| 名称 | 用途 |
|---|---|
| `stereo_YYMMDD_hhmm` | 双目相机标定 |
| `stereo_imu_YYMMDD_hhmm` | Camera–IMU 标定 |
| `stereo_test_YYMMDD_hhmm` | 固定参数验证，不被本入口选择 |

目录名只用于角色选择；传感器映射仍由各目录的 `dataset.yaml` 决定。双目数据必须提供
`cam0`、`cam1` 且没有 IMU；双目＋IMU 数据必须提供 `cam0`、`cam1`、`imu0`。
`.inprogress`、`.failed`、隐藏目录和其他前缀不会成为候选；命名匹配的符号链接会被
明确拒绝并停止运行，防止输入跳出机器目录。

每类只有一个候选时可以自动选择。出现两个或更多同类候选时，程序停止并列出候选，
不根据目录时间、mtime 或字典顺序猜测；开发者应在 `user.yaml` 中指定本工单使用的
目录名。

## 3. `user.yaml` 是开发者接口

`user.yaml` 由开发者或工位维护人员生成并随机器交付，不要求产线操作员填写。可从
[`config/examples/v1.0.0/machine_calibration_workflow.yaml`](../config/examples/v1.0.0/machine_calibration_workflow.yaml)
复制最小配置：

```yaml
schema_version: "1.0.0"
kind: machine_calibration_workflow
machine_dir: /mnt/q/File/machine_data/kalibr_no_ros/mechine-01
camera_imu:
  mode: standard
```

同类数据存在多个候选时，明确指定直接子目录名：

```yaml
datasets:
  stereo: stereo_260916_2136
  stereo_imu: stereo_imu_260916_2200
```

字段含义：

| 字段 | 含义 |
|---|---|
| `schema_version` | 固定为字符串 `"1.0.0"` |
| `kind` | 固定为 `machine_calibration_workflow` |
| `machine_dir` | 机器数据根目录；相对路径以 `user.yaml` 所在目录为基准 |
| `datasets.stereo` | 可选；指定双目数据的直接子目录名 |
| `datasets.stereo_imu` | 可选；指定双目＋IMU 数据的直接子目录名 |
| `camera_imu.mode` | 必填；`standard` 或 `rolling_shutter`，程序不自动切换 |
| `revision` | 可选；相同数据需用新模板、软件来源或工艺重新运行时使用 |

模板路径、target、IMU 参数、初始化、求解线程和输出开关属于已安装的机型 profile，
不放到操作员接口。当前发布 profile 使用 8×10 AprilGrid、`tagSize=0.07 m`、
`tagSpacing=0.3`、`tagStartId=0`、`pinhole-equi`、`cam0/cam1/imu0`。换标定板、相机
模型、IMU 噪声或 RS 行时间边界属于开发者发布新 profile，不应由操作员临时改值。

## 4. 普通与滚动快门 Camera–IMU

模式必须由开发者明确选择：

```yaml
camera_imu:
  mode: standard
```

或：

```yaml
camera_imu:
  mode: rolling_shutter
```

`standard` 调用既有 `camera_imu_calibration` 求解；`rolling_shutter` 调用
`camera_imu_rolling_shutter_calibration`，逐目联合估计行时间。程序不会根据 RMS、
相机型号或数据运动程度自动从普通模型切换为 RS 模型。

`stereo_imu_*` 存在但没有可用的 `stereo_*` 或可信双目结果时，工作流返回
`blocked_missing_camera_calibration`。它不会用动态 Camera–IMU 数据静默替代本流程
要求的静态双目内外参阶段。

## 5. 输出、状态与续跑

所有正式结果都写入：

```text
<machine_dir>/result/
```

名称由输入数据集和模式生成：

```text
<stereo目录名>_pinhole_equi.yaml
<stereo目录名>_pinhole_equi.report.html
<stereo目录名>_pinhole_equi.report.pdf

<stereo_imu目录名>_camera_imu_pinhole_equi.yaml
<stereo_imu目录名>_camera_imu_pinhole_equi.report.html
<stereo_imu目录名>_camera_imu_pinhole_equi.report.pdf

<stereo_imu目录名>_camera_imu_rs_pinhole_equi.yaml
<stereo_imu目录名>_camera_imu_rs_pinhole_equi.report.html
<stereo_imu目录名>_camera_imu_rs_pinhole_equi.report.pdf
```

当 `revision: v2` 时，结果名称增加 `_rev_v2`，形成新的完整标定谱系。Camera–IMU
结果总是记录并复核它实际引用的双目结果；存在多个历史双目结果时不会依赖“自动找
唯一文件”。

工作流私有状态位于：

```text
<machine_dir>/result/.kalibr-cedarobot/
├── state.json
├── workflow.lock
└── tasks/
```

`tasks/` 是程序按安装 profile 生成的可追溯 task，不需要操作员编辑。`.kalibr-cedarobot`
不是正式标定参数；归档整台机器时应与正式结果一起保留，以便安全续跑和追溯。

复用前会核对数据集、模板、软件、task、结果和上游双目结果的内容摘要，以及结果 schema、
任务类型、传感器 ID、YAML/HTML/PDF、运行清单和完成状态。只存在同名 YAML 不足以视为
已完成。

相同数据集在同一 revision 下出现模板、软件或来源变化时，程序停止，不自动覆盖，也不
隐式传递 `--force`。开发者确认要形成新结果后，在 `user.yaml` 写新的 `revision` 再运行。

## 6. 失败与并发规则

- 同一机器同一时间只允许一个工作流持有锁；第二个进程立即报错，不等待或并行写结果。
- 双目阶段失败时不启动 Camera–IMU。
- Camera–IMU 失败不删除或降级已完成的双目结果。未改变任何受指纹保护输入、且没有
  发布正式同名文件时，可重复原命令重试；修复或替换数据文件后必须填写新 `revision`。
- 进程中断后不会把半成品当作成功。尚未发布正式同名文件时，下次运行重新执行整个
  阶段；若中断发生在多文件发布过程中，程序保留现场并要求新 `revision`，避免静默删除
  或覆盖无法完整核验的文件。
- 受管结果目录存在未知文件、损坏清单或符号链接时拒绝覆盖，保留现场供维护人员检查。
- `waiting_for_dataset` 表示当前已完成可执行阶段，不是错误；`blocked_*` 或 `failed` 才需要
  处理后重试。

## 7. 开发者 profile 与维护接口

发布 profile 的源码位于 `config/mechine/workflow_v1.0.0/`，安装后位于：

```text
<prefix>/share/kalibr-noros/config/mechine/workflow_v1.0.0/
```

它包含 AprilGrid、双目初始化、双目 task、IMU 参数、普通和 RS Camera–IMU task。
profile 必须可移植，不能写机器名、数据绝对路径或输出绝对路径。机器入口以结构化 YAML
修改注入 `dataset.path`、`output.name` 和精确的 `camera_calibration.path`，不做文本替换。

研发需要复现单阶段时，使用 `.kalibr-cedarobot/tasks/` 中保存的 task 配合现有接口：

```bash
kalibr-noros validate --config <生成的task.yaml>
kalibr-noros calibrate cameras --config <相机task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate imu-camera --config <普通IMU task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate imu-camera-rs --config <RS IMU task.yaml> --output-dir <machine_dir>/result
```

这些是维护接口。直接运行时，使用者自行负责阶段依赖、结果路径、命名冲突和来源追溯；
产线常规操作继续使用 `kalibr-cedarobot machine`。
