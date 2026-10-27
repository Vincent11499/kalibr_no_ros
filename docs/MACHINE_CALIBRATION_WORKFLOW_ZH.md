# 机器级一键标定工作流

本文说明面向产线的机器级入口：

```bash
kalibr-cedarobot machine --config user.yaml
```

它把数据集发现、task 生成、输入校验、双目及单目标定、Camera–IMU、两模组的可选 Camera–BaseLink、结果复用和交付
串成一条可重复执行的命令。操作员不需要编辑普通 calibration task，也不需要手工把
对应模组的相机结果路径填入后续 task。底层 `kalibr-noros` 命令继续保留，供研发维护、
单项复现和排障使用。

机器工作流只编排既有求解器，不改变各任务的阶段顺序、残差、活动
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
4. 添加 `camera_baselink` 配置和 `stereo_base_link_*` 数据后，程序复用双目 K/D，
   运行 Camera–BaseLink。该阶段不需要 IMU 数据，不因缺少 `stereo_imu_*` 而等待。
5. 同一机器还可放入独立单目模组的 `usb_*` 数据。程序执行单目标定；加入
   `usb_baselink_*` 数据及 `mono_camera_baselink` 配置后，复用该单目结果求 BaseLink
   旋转。不需要等双目或 IMU 数据齐全。
6. 各阶段已完成且数据、模板和软件来源一致时，命令复用结果，不重复求解。

两条链路分别管理，不合成三目联合任务：

| 模组 | 工作流阶段名 | 任务及上游依赖 |
|---|---|---|
| 双目 `cam0/cam1` | `camera` | 普通或 RS 双目 K/D 和双目外参 |
| 双目 `cam0/cam1` | `camera_imu` | 仅复用 `camera` 结果，普通或 RS Camera–IMU |
| 双目 `cam0/cam1` | `camera_baselink` | 仅复用 `camera` 结果，逐目旋转和配置平移 |
| 独立单目 `cam0` | `mono_camera` | 单目 K/D，使用原生自动初始化，不套用双目初值 |
| 独立单目 `cam0` | `mono_camera_baselink` | 仅复用 `mono_camera` 结果，旋转和配置平移 |

单目中的 `cam0` 是该模组的局部 ID，不是双目的左相机。目录角色、独立 task、结果
文件名、上游精确路径和 SHA256 一起区分它们；不能因两个文件都有 `cam0` 就互换。
本入口不新增单目 Camera–IMU，也不直接求单目模组与双目模组之间的联合外参。

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
├── stereo_base_link_261021_1630/    可选：固定姿态平台平移数据
│   ├── dataset.yaml
│   └── cameras/...
├── stereo_test_260916_2211/         独立验证数据，不参与自动标定
├── usb_261021_1510/               独立单目模组内参数据，仅 cam0
│   ├── dataset.yaml
│   └── cameras/cam0/...
├── usb_baselink_261021_1640/      单目模组固定姿态平移数据，仅 cam0
│   ├── dataset.yaml
│   └── cameras/cam0/...
└── result/                           程序创建或复用
```

自动候选只接受严格名称：

| 名称 | 用途 |
|---|---|
| `stereo_YYMMDD_hhmm` | 双目相机标定 |
| `stereo_imu_YYMMDD_hhmm` | Camera–IMU 标定 |
| `stereo_base_link_YYMMDD_hhmm` | Camera–BaseLink；也接受 `stereo_baselink_YYMMDD_hhmm` |
| `usb_YYMMDD_HHMM` | 独立单目模组内参标定 |
| `usb_baselink_YYMMDD_HHMM` | 单目 Camera–BaseLink |
| `stereo_test_YYMMDD_hhmm` | 固定参数验证，不被本入口选择 |

目录名只用于角色选择；传感器映射仍由各目录的 `dataset.yaml` 决定。双目数据必须提供
`cam0`、`cam1` 且没有 IMU；BaseLink 数据相同；双目＋IMU 数据必须提供 `cam0`、`cam1`、`imu0`。
双目清单中的相机顺序必须是 `[cam0, cam1]`。`usb_*` 和 `usb_baselink_*` 清单
必须且只能包含 `[cam0]`，不能包含 IMU。当前机器 profile 不自动处理此前 USB 棋盘格、
bag 或三目任务；纯视觉 RS 双目通过 `camera.mode: rolling_shutter` 显式启用，其他任务继续使用底层 `kalibr-noros`。
`.inprogress`、`.failed`、隐藏目录和其他前缀不会成为候选；命名匹配的符号链接会被
明确拒绝并停止运行，防止输入跳出机器目录。

每类只有一个候选时可以自动选择。出现两个或更多同类候选时，程序停止并列出候选，
不根据目录时间、mtime 或字典顺序猜测；开发者应在 `user.yaml` 中指定本工单使用的
目录名。

### 2.1 每套目录数据的内部格式

内部路径可自定义，但必须与 `dataset.yaml` 一致。推荐：

```text
stereo_base_link_261021_1630/
├── dataset.yaml
└── cameras/
    ├── cam0/
    │   ├── timestamps.csv
    │   └── images/
    │       ├── 0_0_0.png
    │       └── 5_0_0.png
    └── cam1/
        ├── timestamps.csv
        └── images/...
```

无 IMU 的清单示例（`dataset_id` 改成实际目录标识）：

```yaml
schema_version: "1.0.0"
type: kalibr_directory_dataset
dataset_id: stereo_base_link_261021_1630
cameras:
  - id: cam0
    images: cameras/cam0/images
    timestamps: cameras/cam0/timestamps.csv
  - id: cam1
    images: cameras/cam1/images
    timestamps: cameras/cam1/timestamps.csv
imus: []
```

单目目录采用同样结构，但只保留 `cameras/cam0/`。例如单目 BaseLink 的完整清单：

```yaml
schema_version: "1.0.0"
type: kalibr_directory_dataset
dataset_id: usb_baselink_261021_1640
cameras:
  - id: cam0
    images: cameras/cam0/images
    timestamps: cameras/cam0/timestamps.csv
imus: []
```

每目 `timestamps.csv` 的表头和内容为：

```csv
timestamp_ns,filename
1000000000,0_0_0.png
2000000000,5_0_0.png
```

时间戳保留整数纳秒，文件名相对于对应的 `images` 目录。普通单目、双目数据的图像名无需包含
位置；BaseLink 数据则必须使用 `<x>_<y>_<z>.<png|jpg|jpeg|bmp>`，例如
`43_-5_0.png`。`[x,y,z] * position_scale_m` 是平台在 BaseLink 坐标系中的位置，
不能把随机帧号当位置。以上两帧只说明格式，不足以完成旋转求解。

Camera–IMU 的清单再加入：

```yaml
imus:
  - id: imu0
    data: imu/imu0.csv
```

IMU CSV 表头为 `timestamp_ns,wx,wy,wz,ax,ay,az`，角速度单位 `rad/s`，加速度单位
`m/s²`。BaseLink 采集时平台姿态不变、标定板固定在环境中，仅平移相机平台；至少三个有效位置且覆盖两个独立
平移方向；每目都需看到本 profile 指定的标定板。完整契约见
[目录数据说明](DATASET_WORKFLOW_ZH.md)和[BaseLink 任务说明](CAMERA_BASELINK_CALIBRATION_ZH.md)。

## 3. `user.yaml` 是开发者接口

`user.yaml` 由开发者或工位维护人员生成并随机器交付，不要求产线操作员填写。可从
[`config/examples/v1.0.0/machine_calibration_workflow.yaml`](../config/examples/v1.0.0/machine_calibration_workflow.yaml)
复制最小配置：

```yaml
schema_version: "1.0.0"
kind: machine_calibration_workflow
machine_dir: /mnt/q/File/machine_data/kalibr_no_ros/mechine-01
camera:
  mode: standard
camera_imu:
  mode: standard
  recompute_extrinsics: false
```

同类数据存在多个候选时，明确指定直接子目录名：

```yaml
datasets:
  stereo: stereo_260916_2136
  stereo_imu: stereo_imu_260916_2200
  stereo_baselink: stereo_base_link_261021_1630
  mono: usb_261021_1510
  mono_baselink: usb_baselink_261021_1640
```

显式指定的目录必须已存在；分阶段采集时，不要提前填写尚未复制到机器根目录的目录名。
未指定某角色时，程序自动发现该角色的唯一候选，没有候选则等待。

USB 单目目录统一使用上述 `usb_` / `usb_baselink_` 前缀，`HHMM` 为四位小时、分钟。
不再识别此前新增的 `mono_*` / `mono_base_link_*` / `mono_baselink_*` 目录名称。
这次只改目录约定；`datasets.mono`、`datasets.mono_baselink`、`mono_camera_baselink`
以及单目阶段名保持不变，标定板仍为两个模组共用的 AprilGrid。

字段含义：

| 字段 | 含义 |
|---|---|
| `schema_version` | 固定为字符串 `"1.0.0"` |
| `kind` | 固定为 `machine_calibration_workflow` |
| `machine_dir` | 机器数据根目录；相对路径以 `user.yaml` 所在目录为基准 |
| `target` | 可选；只填安全文件名stem，例如 `aprilgrid_1`；从安装的 `config/mechine/aprilgrid_1.yaml` 读取，省略时使用内部profile标定板 |
| `datasets.stereo` | 可选；指定双目数据的直接子目录名 |
| `datasets.stereo_imu` | 可选；指定双目＋IMU 数据的直接子目录名 |
| `datasets.stereo_baselink` | 可选；指定 BaseLink 数据的直接子目录名，兼容上述两种前缀 |
| `datasets.mono` | 可选；指定 `usb_YYMMDD_HHMM` 单目内参数据的直接子目录名 |
| `datasets.mono_baselink` | 可选；指定 `usb_baselink_YYMMDD_HHMM` 单目 BaseLink 数据的直接子目录名 |
| `camera.mode` | 可选；`standard` 或 `rolling_shutter`，默认 `standard`；控制双目相机阶段 |
| `camera_imu.mode` | 必填；`standard` 或 `rolling_shutter`，程序不自动切换 |
| `camera_imu.recompute_extrinsics` | 可选布尔值，默认 `false`；映射到底层 `recompute_camera_chain_extrinsics` |
| `camera_baselink` | 可选；省略且无对应数据时为 `disabled`；有数据但缺配置时报 `blocked_missing_configuration` |
| `mono_camera_baselink` | 可选；单目 BaseLink 配置，只能含 `cam0`；启用、等待与阻塞规则同上 |
| `revision` | 可选；相同数据需用新模板、软件来源或工艺重新运行时使用 |

任务模板、IMU 参数、初始化、求解线程和输出开关属于已安装的机型 profile，
不放到操作员接口。target 只暴露已安装标定板文件的安全stem，不接受任意路径。
省略 target 时，当前发布 profile 使用 8×10 AprilGrid、`tagSize=0.07 m`、
`tagSpacing=0.3`、`tagStartId=0`；两个模组均为 `pinhole-equi`，共用这份 AprilGrid。
双目为 `cam0/cam1`（可带 `imu0`），独立单目为局部 `cam0`。换标定板、相机
模型、IMU 噪声或 RS 行时间边界属于开发者发布新 profile，不应由操作员临时改值。

### 3.1 启用 Camera–BaseLink

在原 `user.yaml` 中增加以下块。平移值仅为格式示例，**必须换成该机器的实测值**：

```yaml
camera_baselink:
  cameras:
    - id: cam0
      translation_base_camera_m: [0.1, 0.2, 0.3]
```

| 字段 | 默认值、单位、范围与作用 |
|---|---|
| `camera_baselink.base_link` | 可省略；内部默认 `frame_id: base_link`、`position_scale_m: 0.01`；使用其他坐标系或文件名单位时显式覆盖 |
| `camera_baselink.base_link.frame_id` | 默认 `base_link`；匹配 `[A-Za-z0-9][A-Za-z0-9_/-]*` 的非空坐标系 ID；标记结果的基准坐标系 |
| `camera_baselink.base_link.position_scale_m` | 默认 `0.01` m/文件名单位；有限正数；毫米填 `0.001`，米填 `1.0` |
| `camera_baselink.cameras` | 必填；可为 `[cam0]` 或按顺序 `[cam0, cam1]` |
| `camera_baselink.cameras[].translation_base_camera_m` | 必填、无默认值；三个有限数，单位米；相机原点在 BaseLink 中的位置，即 `T_base_camera` 的平移列，不参与求解 |

上述配置用于双目模组，固定从本工单双目结果读取 K/D，只求配置中各目的 `R_base_camera`。
采用全部有效无序帧对的直接差分，不提供 `alpha` 参数。完整注释示例见
[`machine_calibration_workflow_full.yaml`](../config/examples/v1.0.0/all_params/machine_calibration_workflow_full.yaml)。
此前 USB 的 11×8、`square_size=0.05 m` 棋盘格数据不能直接套用当前 AprilGrid 机器
profile；单目 USB 仍使用独立的 `cameras`、`cameras-baselink` task。

### 3.2 独立单目模组的 BaseLink

单目内参数据只要放入 `usb_YYMMDD_HHMM` 就会自动执行 `mono_camera`，不需要新增
模型选择块。需要单目 BaseLink 时，再添加下列配置（平移为示意，必须换成实测值）：

```yaml
mono_camera_baselink:
  base_link:
    frame_id: base_link
    position_scale_m: 0.01
  cameras:
    - id: cam0
      translation_base_camera_m: [-0.1, 0.05, 0.3]
```

字段默认值、单位和范围与 §3.1 相同；`base_link` 也可整体省略，但 `cameras` 必须且只能为 `[cam0]`。
位置比例、平移和结果单独记录，不继承双目 `camera_baselink` 配置。
两个模组若都相对同一机体坐标系，应使用相同 `frame_id`，且平移均在该坐标系内测量。
`mono_camera_baselink` 只引用本次机器工作流中的单目标定结果，不搜索或回退到双目
`cam0`。单目标定数据缺失时，返回 `blocked_missing_camera_calibration`，即使双目结果存在。

## 4. 普通与滚动快门 Camera–IMU

双目相机阶段可由开发者明确选择普通或 RS：

```yaml
camera:
  mode: rolling_shutter
```

省略 `camera` 时保持原有普通双目标定。Camera–IMU 模式必须明确选择：

```yaml
camera_imu:
  mode: standard
```

或：

```yaml
camera_imu:
  mode: rolling_shutter
  recompute_extrinsics: true
```

`standard` 调用既有 `camera_imu_calibration` 求解；`rolling_shutter` 调用
`camera_imu_rolling_shutter_calibration`，逐目联合估计行时间。程序不会根据 RMS、
相机型号或数据运动程度自动从普通模型切换为 RS 模型。
`recompute_extrinsics: true` 仅让联合阶段重新优化相机链相邻外参，仍精确引用本次
工作流的相机结果，不会重做相机内参初始化。

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

<stereo目录名>_rs_pinhole_equi.yaml
<stereo目录名>_rs_pinhole_equi.report.html
<stereo目录名>_rs_pinhole_equi.report.pdf

<stereo_imu目录名>_camera_imu_pinhole_equi.yaml
<stereo_imu目录名>_camera_imu_pinhole_equi.report.html
<stereo_imu目录名>_camera_imu_pinhole_equi.report.pdf

<stereo_imu目录名>_camera_imu_rs_pinhole_equi.yaml
<stereo_imu目录名>_camera_imu_rs_pinhole_equi.report.html
<stereo_imu目录名>_camera_imu_rs_pinhole_equi.report.pdf

<stereo_base_link目录名>_camera_baselink_cam0_pinhole_equi.yaml
<stereo_base_link目录名>_camera_baselink_cam0_pinhole_equi.report.html
<stereo_base_link目录名>_camera_baselink_cam0_pinhole_equi.report.pdf
<stereo_base_link目录名>_camera_baselink_cam0_pinhole_equi/rotation_summary.csv

<usb目录名>_pinhole_equi.yaml
<usb目录名>_pinhole_equi.report.html
<usb目录名>_pinhole_equi.report.pdf

<usb_baselink目录名>_camera_baselink_cam0_pinhole_equi.yaml
<usb_baselink目录名>_camera_baselink_cam0_pinhole_equi.report.html
<usb_baselink目录名>_camera_baselink_cam0_pinhole_equi.report.pdf
<usb_baselink目录名>_camera_baselink_cam0_pinhole_equi/rotation_summary.csv
```

BaseLink YAML 每目含固定 K/D、`T_base_camera`、`pitch`（光轴相对水平面的无符号倾角，
度）、有效帧对数及运动残差；报告展示对应质量项。CSV 记录全部无序帧对的
`theta_rad/theta_deg` 与汇总值，用于检查固定姿态采集的 PnP 旋转变化，不影响求解。

当 `revision: v2` 时，结果名称增加 `_rev_v2`，形成新的完整标定谱系。Camera–IMU 和 BaseLink
结果总是记录并复核它实际引用的对应模组相机结果；存在多个历史相机结果时不会依赖“自动找
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

复用前会核对数据集、模板、软件、task、结果和上游对应模组相机结果的内容摘要，以及结果 schema、
任务类型、传感器 ID、YAML/HTML/PDF、运行清单和完成状态。只存在同名 YAML 不足以视为
已完成。

相同数据集在同一 revision 下出现模板、软件或来源变化，或已配置的 BaseLink 平移、
比例、坐标系发生变化时，程序停止，不自动覆盖，也不
隐式传递 `--force`。开发者确认要形成新结果后，在 `user.yaml` 写新的 `revision` 再运行。

## 6. 失败与并发规则

- 同一机器同一时间只允许一个工作流持有锁；第二个进程立即报错，不等待或并行写结果。
- 每个模组的 BaseLink 必须有本模组的可用相机结果，否则返回
  `blocked_missing_camera_calibration`；不跨模组回退。
- 阶段顺序为双目 → 双目 Camera–IMU → 双目 BaseLink → 单目 → 单目 BaseLink。
  缺少某阶段的数据不会阻止另一模组的可执行阶段，缺少 IMU 数据不阻止两种 BaseLink。
  已开始的求解若抛出异常，本次命令立即停止；修复后重复命令续跑。
- Camera–IMU 失败不删除或降级已完成的双目结果。未改变任何受指纹保护输入、且没有
  发布正式同名文件时，可重复原命令重试；修复或替换数据文件后必须填写新 `revision`。
- 进程中断后不会把半成品当作成功。尚未发布正式同名文件时，下次运行重新执行整个
  阶段；若中断发生在多文件发布过程中，程序保留现场并要求新 `revision`，避免静默删除
  或覆盖无法完整核验的文件。
- 受管结果目录存在未知文件、损坏清单或符号链接时拒绝覆盖，保留现场供维护人员检查。
- `waiting_for_dataset` 表示当前已完成可执行阶段，不是错误；终端会打印需要补充的目录角色。
  `blocked_missing_camera_calibration` 会同时指出已发现的下游数据和缺少的前置相机阶段；
  `blocked_*` 或 `failed` 才需要处理后重试。

## 7. 开发者 profile 与维护接口

发布 profile 的源码位于 `config/mechine/workflow_v1.0.0/`，安装后位于：

```text
<prefix>/share/kalibr-noros/config/mechine/workflow_v1.0.0/
```

它包含默认 AprilGrid、双目初始化、普通和 RS 双目 task、IMU 参数、普通和 RS Camera–IMU task、
双目 BaseLink task，以及单目和单目 BaseLink task。单目不引用双目初始化文件。
可选标定板 YAML 位于安装后的 `config/mechine/` 根目录，由 `user.yaml.target` 以stem选择。
profile 必须可移植，不能写机器名、数据绝对路径或输出绝对路径。机器入口以结构化 YAML
修改注入 `dataset.path`、`output.name`、精确的 `camera_calibration.path`，以及明确配置的
`base_link` 和每目平移，不做文本替换。

研发需要复现单阶段时，使用 `.kalibr-cedarobot/tasks/` 中保存的 task 配合现有接口：

```bash
kalibr-noros validate --config <生成的task.yaml>
kalibr-noros calibrate cameras --config <相机task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate cameras-rs --config <RS相机task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate imu-camera --config <普通IMU task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate imu-camera-rs --config <RS IMU task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate cameras-baselink --config <BaseLink task.yaml> --output-dir <machine_dir>/result
```

这些是维护接口。直接运行时，使用者自行负责阶段依赖、结果路径、命名冲突和来源追溯；
产线常规操作继续使用 `kalibr-cedarobot machine`。
