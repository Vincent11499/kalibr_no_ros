# 机器级一键标定工作流

本文说明面向产线的机器级入口：

```bash
kalibr-cedarobot machine
```

它把数据集发现、task 生成、输入校验、双目及单目标定、Camera–IMU、两模组的可选 Camera–BaseLink、结果复用和交付
串成一条可重复执行的命令。操作员不需要编辑普通 calibration task，也不需要手工把
对应模组的相机结果路径填入后续 task。底层 `kalibr-noros` 命令继续保留，供研发维护、
单项复现和排障使用。

机器工作流只编排既有求解器，不改变各任务的阶段顺序、残差、活动
参数、优化器和异常点逻辑。

## 1. 操作员只需要做什么

工位维护人员把 `user.yaml` 放到 `<prefix>/config/user.yaml`。操作员完成采集和数据复制后，在 WSL/Linux 中
重复执行同一条命令：

```bash
<prefix>/bin/kalibr-cedarobot machine
```

默认读取 `<prefix>/config/user.yaml`；临时使用另一份配置时再显式增加
`--config /path/to/user.yaml`。

不要求先同时准备双目和双目＋IMU 两套数据：

1. 当前批次目录中只有 `stereo_*` 时，命令完成双目内外参标定，随后以
   `waiting_for_dataset` 正常结束。
2. 以后把 `stereo_imu_*` 复制到同一个批次目录，再执行原命令。程序核对并复用此前
   的双目结果，只运行 Camera–IMU 阶段。
3. 两套数据一开始就齐全时，程序先完成双目，再把本次双目结果的精确路径交给
   Camera–IMU 阶段。
4. 添加 `camera_baselink` 配置和 `stereo_base_link_*` 数据后，程序复用双目 K/D，
   运行 Camera–BaseLink。该阶段不需要 IMU 数据，不因缺少 `stereo_imu_*` 而等待。
5. 同一机器还可放入独立单目模组的 `usb_*` 数据。程序执行单目标定；加入
   `usb_base_link_*` 数据及 `mono_camera_baselink` 配置后，复用该单目结果求 BaseLink
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

## 2. 机器与批次目录约定

机器目录名固定为 `<machine_version>-<machine_num>`；其下每次标定使用一个独立批次
目录。`machine_dir` 指向批次目录，workflow 只扫描该目录的直接子目录，不递归进入
`result/`：

```text
EVT312-02/
├── user.yaml
└── 1029/                             本次标定批次；名称可为安全字母、数字、下划线或连字符
    ├── stereo_260916_2136/           双目内外参数据
    ├── stereo_imu_260916_2200/       双目＋IMU 数据，可以稍后加入同一批次
    ├── stereo_base_link_261021_1630/ 可选：固定姿态平台平移数据
    ├── stereo_test_260916_2211/      独立验证数据，不参与自动标定
    ├── usb_261021_1510/              独立单目模组内参数据，仅 cam0
    ├── usb_base_link_261021_1640/    单目模组固定姿态平移数据，仅 cam0
    └── result/                        本批次程序创建或复用
```

同一工单内因精度不足重新采集时，删除旧数据集或把目录改成不满足下表的名称（例如
追加 `_bk`），再放入唯一的新规范目录；工作流会自动重跑对应任务和依赖任务。真正的
新工单仍新建批次目录并修改 `user.yaml.machine_dir`，不同批次不自动复用结果。批次名不承担时间解析，只需匹配
`[A-Za-z0-9][A-Za-z0-9_-]{0,63}`，例如 `1029`、`20261030` 或 `run_02`。

自动候选只接受严格名称：

| 名称 | 用途 |
|---|---|
| `stereo_YYMMDD_hhmm` | 双目相机标定 |
| `stereo_imu_YYMMDD_hhmm` | Camera–IMU 标定 |
| `stereo_base_link_YYMMDD_HHMM` | Camera–BaseLink |
| `usb_YYMMDD_HHMM` | 独立单目模组内参标定 |
| `usb_base_link_YYMMDD_HHMM` | 单目 Camera–BaseLink |
| `stereo_test_YYMMDD_hhmm` | 固定参数验证，不被本入口选择 |

目录名只用于角色选择；传感器映射仍由各目录的 `dataset.yaml` 决定。双目数据必须提供
`cam0`、`cam1` 且没有 IMU；BaseLink 数据相同；双目＋IMU 数据必须提供 `cam0`、`cam1`、`imu0`。
双目清单中的相机顺序必须是 `[cam0, cam1]`。`usb_*` 和 `usb_base_link_*` 清单
必须且只能包含 `[cam0]`，不能包含 IMU。当前机器 profile 不自动处理此前 USB 棋盘格、
bag 或三目任务；纯视觉 RS 双目通过 `camera.mode: rolling_shutter` 显式启用，其他任务继续使用底层 `kalibr-noros`。
`.inprogress`、`.failed`、`*_bk`、旧名、缺少时间后缀的目录、隐藏目录和其他任意
非规范名称都静默跳过，不影响正常标定；严格命名的符号链接会被明确拒绝并停止运行，
防止输入跳出机器目录。

每类只有一个候选时可以自动选择。出现两个或更多同类候选时，程序停止并列出候选，
不根据目录时间、mtime 或字典顺序猜测；开发者应清理或重命名旧候选，也可以在
`user.yaml` 中显式指定本工单使用的目录名。

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
dataset_id: usb_base_link_261021_1640
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
machine_dir: "1029"
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
  mono_baselink: usb_base_link_261021_1640
```

`datasets` 可完全省略或只填写部分角色。显式指定的角色强制使用该直接子目录，即使同类
还有其他合法候选；未指定角色仍自动发现唯一候选，没有候选则等待。指定目录必须已存在，
分阶段采集时不要提前填写尚未复制到当前批次目录的目录名。

USB 单目目录统一使用上述 `usb_` / `usb_base_link_` 前缀，`HHMM` 为四位小时、分钟。
不再识别 `stereo_baselink_*`、`usb_baselink_*`、`mono_*`、`mono_base_link_*` 或
`mono_baselink_*` 目录名称；这类旧名及形似任务但后缀不合法的目录会被静默跳过。
这次只改目录约定；`datasets.mono`、`datasets.mono_baselink`、`mono_camera_baselink`
以及单目阶段名保持不变，标定板仍为两个模组共用的 AprilGrid。

字段含义：

| 字段 | 含义 |
|---|---|
| `schema_version` | 固定为字符串 `"1.0.0"` |
| `kind` | 固定为 `machine_calibration_workflow` |
| `machine_dir` | 本次标定批次目录；相对路径以 `user.yaml` 所在目录为基准；父目录必须严格使用 `<machine_version>-<machine_num>`，批次名匹配安全名称规则；例如 `EVT312-02/1029` 从父目录得到 `EVT312` 和字符串编号 `"02"` |
| `target` | 可选；只填安全文件名stem，例如 `aprilgrid_1`；从安装的 `config/mechine/aprilgrid_1.yaml` 读取，省略时使用内部profile标定板 |
| `datasets.stereo` | 可选；指定双目数据的直接子目录名 |
| `datasets.stereo_imu` | 可选；指定双目＋IMU 数据的直接子目录名 |
| `datasets.stereo_baselink` | 可选；强制指定严格命名的双目 BaseLink 数据直接子目录 |
| `datasets.mono` | 可选；指定 `usb_YYMMDD_HHMM` 单目内参数据的直接子目录名 |
| `datasets.mono_baselink` | 可选；指定 `usb_base_link_YYMMDD_HHMM` 单目 BaseLink 数据的直接子目录名 |
| `camera.mode` | 可选；`standard` 或 `rolling_shutter`，默认 `standard`；控制双目相机阶段 |
| `camera_imu.mode` | 必填；`standard` 或 `rolling_shutter`，程序不自动切换 |
| `camera_imu.recompute_extrinsics` | 可选布尔值，默认 `false`；映射到底层 `recompute_camera_chain_extrinsics` |
| `camera_baselink` | 可选；省略且无对应数据时为 `disabled`；有数据但缺配置时在求解前报错 |
| `mono_camera_baselink` | 可选；单目 BaseLink 配置，只能含 `cam0`；有数据但缺配置时在求解前报错 |

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
采用全部有效无序帧对的直接差分，不提供 `alpha` 参数；写出前固定左乘绕 X 轴 180°
的旋转矩阵，配置平移不变。完整注释示例见
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
单目 BaseLink 结果写出前固定左乘绕 Y 轴 180° 的旋转矩阵，配置平移不变。
两个模组若都相对同一机体坐标系，应使用相同 `frame_id`，且平移均在该坐标系内测量。
`mono_camera_baselink` 只引用本次机器工作流中的单目标定结果，不搜索或回退到双目
`cam0`。若发现单目 BaseLink 数据却缺少单目数据，工作流在启动任何求解前报错，即使双目结果存在。

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

`stereo_imu_*` 存在但没有 `stereo_*` 时，工作流在启动任何求解前报错。它不会用动态
Camera–IMU 数据静默替代本流程要求的静态双目内外参阶段。

## 5. 输出、状态与续跑

机器入口使用数据集名作为输出基础名，不追加任务类型、相机 ID、模型、RS 或 revision
后缀。每个批次拥有独立 `result/`：

```text
<machine_root>/<batch>/
├── stereo_YYMMDD_HHMM/
├── stereo_imu_YYMMDD_HHMM/
├── stereo_base_link_YYMMDD_HHMM/
├── usb_YYMMDD_HHMM/
├── usb_base_link_YYMMDD_HHMM/
└── result/
    ├── <数据集名>.report.html
    ├── <数据集名>.report.pdf
    ├── <数据集名>/
    │   ├── .inventory.json
    │   ├── <数据集名>.yaml
    │   ├── task_resolved.yaml
    │   ├── run_manifest.json
    │   ├── validation.json
    │   ├── metrics.json
    │   ├── assessment.json
    │   ├── stdout.log
    │   ├── stderr.log
    │   ├── visualizations/       # 启用时
    │   ├── observations/         # 启用观测归档时
    │   ├── images/               # 启用原图复制时
    │   ├── opencv/               # 启用 OpenCV 导出时
    │   ├── poses.csv             # 支持且启用时
    │   └── rotation_summary.csv  # Camera–BaseLink 专有
    ├── .kalibr-cedarobot/
    │   ├── state.json
    │   ├── workflow.lock
    │   └── tasks/
    └── calib_params/
        ├── vision/stereo_imu_calib.yaml
        └── mono_camera/usb_mono_calib.yaml
```

`initialization_report.yaml`、`observability.yaml`、`results.txt`、`timing.json` 等未在简图
逐一列出的既有可选文件仍由原输出开关控制，并放在同一个任务目录。报告内链接均相对该
报告所在的 `result/` 根目录；`.inventory.json` 管理任务目录，`run_manifest.json` 还校验
根目录 HTML/PDF，防止强制覆盖时误删用户文件。底层 `kalibr-noros` 直接运行仍使用原有
的根目录平铺正式 YAML/报告和同名证据目录，机器入口采用上述混合布局。

同一批次替换数据集时，旧数据集对应的任务目录及根目录 HTML/PDF 报告原样保留；新
数据集使用新名称生成另一套结果。例如 `stereo_261029_1000` 改为 `_bk` 后加入
`stereo_261029_1500`，两套双目结果可以同时存在。仍使用原数据集名称但依赖已变化的
下游任务（例如 `stereo_imu_261029_1100`）先在临时目录完整求解，成功后才替换其同名
受管目录和报告；求解失败时旧结果和旧设备参数不变。替换期间状态记录
`pending_recalibration`，后续运行继续同一次替换，全部相关阶段成功后才切换活动绑定并
更新设备参数。不会创建 `active`、`archive` 或 revision 目录，也不会自动删除旧结果。

BaseLink YAML 每目含固定 K/D、修正后的 `T_base_camera`、`rotation_left_correction`、
`pitch`（光轴相对水平面的无符号倾角，度）、有效帧对数及运动残差；报告展示对应质量项。CSV 记录全部无序帧对的
`theta_rad/theta_deg` 与汇总值，用于检查固定姿态采集的 PnP 旋转变化，不影响求解。

### 5.1 设备参数文件

机器入口按已完成的模组生成两类设备消费文件：双目链路写
`result/calib_params/vision/stereo_imu_calib.yaml`，USB 单目链路写
`result/calib_params/mono_camera/usb_mono_calib.yaml`。不再生成
`stereo_camera/stereo_calib.yaml`。

文件首部为 `machine_version`、字符串 `machine_num`、`time`、`algorithm_version` 和
`calib_file_version`。`machine_version`/`machine_num` 从批次父目录名称拆分；
`EVT312-02` 得到 `EVT312` 和 `"02"`。两个版本字段当前均为 `"1.0.0"`。`time` 是
文件最后一次成功组装的 Asia/Shanghai 时间，格式为 `YYYY-MM-DD-HH:MM:SS`；来源没有
变化的续跑不重写文件或更新时间。

有 Camera–IMU 结果时，两目 K/D、分辨率、RMS、`T_cn_cnm1`、alignment 和双目校正
矩阵统一使用 Camera–IMU 的最终相机结果；否则统一使用双目标定结果，避免混用两套几何。
`cam0_cam1_correct.R1/P1/R2/P2` 使用对应任务 `metrics.json` 记录的 rectification 设置，
复用与报告/OpenCV 导出相同的 `stereo_geometry()` 计算；`baseline` 为
`T_cn_cnm1` 平移列的欧氏长度，单位米，`alignment` 为非视差方向误差 RMS，单位 px。

双目 Camera–BaseLink 的 cam0 `T_base_camera` 和 `pitch` 分别写入
`camera.cam0.T_base_cam` 与 `camera.cam0.pitch`。该阶段尚未完成时写单位阵和 `0.0`。
这里的旋转已经包含双目固定的 X 轴 180° 左乘修正；USB 单目设备参数中的对应旋转
已经包含 Y 轴 180° 左乘修正。
相机 `T_imu_cam` 直接取 Camera–IMU 同名字段，`td_cam_to_imu` 原值复制
`timeshift_cam_imu`，满足 `t_imu=t_cam+td_cam_to_imu`；Camera–IMU 尚未完成时使用
单位阵和 `0.0`。RS 行延迟优先使用 RS Camera–IMU 结果，否则使用 RS 双目结果；普通
任务写 `0.0`。

固定 IMU 参数为：`imu_type=soc`、`imu_data_rate=100 Hz`、`arw=0.514 deg/√h`、
`vrw=0.800 m/s/√h`、`gb_std=30 deg/h`、`ab_std=100 μg`、`corr_time=1.0 s`、
`gravity=9.81 m/s²`、`gyro_scale=[0.9934,0.9934,0.9934]`、
`acc_scale=[0.989362,0.989362,0.989362]`。最终 `gyro_bias`/`acc_bias` 是 bias spline
三轴样本均值；Camera–IMU 尚未完成时写零。

`odom` 只保留固定噪声和 scale：`w_x_std=0.1`、`w_y_std=0.1`、`w_z_std=2.0`、
`w_z_std_vision_off=20.0`、`v_x_std=0.05`、`v_y_std=0.05`、`v_z_std=0.001`、
`v_scale=1.0`、`w_scale=1.0`；不再包含 BaseLink 变换。完整结构如下：

```yaml
machine_version: EVT312
machine_num: "02"
time: "2026-10-10-19:58:00"
algorithm_version: "1.0.0"
calib_file_version: "1.0.0"
imu:
  imu_type: soc
  imu_data_rate: 100
  arw: 0.514
  vrw: 0.800
  gb_std: 30
  ab_std: 100
  corr_time: 1.0
  gravity: 9.81
  gyro_bias: [0.0, 0.0, 0.0]
  gyro_scale: [0.9934, 0.9934, 0.9934]
  acc_bias: [0.0, 0.0, 0.0]
  acc_scale: [0.989362, 0.989362, 0.989362]
camera:
  cam0:
    model: pinhole-equidistant
    resolution: [3840, 2160]
    intrinsic: [fu, fv, cu, cv]
    distortion: [k1, k2, k3, k4]
    T_base_cam:
      - [1.0, 0.0, 0.0, 0.0]
      - [0.0, 1.0, 0.0, 0.0]
      - [0.0, 0.0, 1.0, 0.0]
      - [0.0, 0.0, 0.0, 1.0]
    T_imu_cam:
      - [1.0, 0.0, 0.0, 0.0]
      - [0.0, 1.0, 0.0, 0.0]
      - [0.0, 0.0, 1.0, 0.0]
      - [0.0, 0.0, 0.0, 1.0]
    td_cam_to_imu: 0.0
    line_delay_s: 0.0
    pitch: 0.0
    rms: 0.22742579286357287
  cam1:
    model: pinhole-equidistant
    resolution: [3840, 2160]
    intrinsic: [fu, fv, cu, cv]
    distortion: [k1, k2, k3, k4]
    T_cn_cnm1:
      - [1.0, 0.0, 0.0, -0.1]
      - [0.0, 1.0, 0.0, 0.0]
      - [0.0, 0.0, 1.0, 0.0]
      - [0.0, 0.0, 0.0, 1.0]
    T_imu_cam:
      - [1.0, 0.0, 0.0, 0.0]
      - [0.0, 1.0, 0.0, 0.0]
      - [0.0, 0.0, 1.0, 0.0]
      - [0.0, 0.0, 0.0, 1.0]
    td_cam_to_imu: 0.0
    line_delay_s: 0.0
    rms: 0.22742579286357287
  cam0_cam1_correct:
    R1:
      - [1.0, 0.0, 0.0]
      - [0.0, 1.0, 0.0]
      - [0.0, 0.0, 1.0]
    P1:
      - [1000.0, 0.0, 640.0, 0.0]
      - [0.0, 1000.0, 400.0, 0.0]
      - [0.0, 0.0, 1.0, 0.0]
    R2:
      - [1.0, 0.0, 0.0]
      - [0.0, 1.0, 0.0]
      - [0.0, 0.0, 1.0]
    P2:
      - [1000.0, 0.0, 640.0, -100.0]
      - [0.0, 1000.0, 400.0, 0.0]
      - [0.0, 0.0, 1.0, 0.0]
    baseline: 0.1
    alignment: 0.25013276123650297
odom:
  w_x_std: 0.1
  w_y_std: 0.1
  w_z_std: 2.0
  w_z_std_vision_off: 20.0
  v_x_std: 0.05
  v_y_std: 0.05
  v_z_std: 0.001
  v_scale: 1.0
  w_scale: 1.0
```

上例的单位阵和零表示尚未完成 Camera–IMU/BaseLink 时的正式默认值；双目校正矩阵、
基线、K/D、RMS 和 alignment 始终写实际计算值。`fu` 等符号只表示字段位置，不是最终
文件中的字符串。

`usb_mono_calib.yaml` 的模型、分辨率、内参、畸变和 RMS 来自 USB 单目标定；
`T_base_cam` 与 `pitch` 来自 USB 单目 Camera–BaseLink。尚未完成 BaseLink 时分别写
单位阵和 `0.0`。文件结构如下：

```yaml
machine_version: EVT312
machine_num: "02"
time: "2026-10-10-19:58:00"
algorithm_version: "1.0.0"
calib_file_version: "1.0.0"
camera:
  model: pinhole-equidistant
  resolution: [1280, 800]
  intrinsic: [fu, fv, cu, cv]
  distortion: [k1, k2, k3, k4]
  T_base_cam:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  pitch: 0.0
  rms: 0.22742579286357287
```

聚合更新关系如下：

| 完成阶段 | 更新文件 |
|---|---|
| 双目标定 | 创建或更新 `stereo_imu_calib.yaml` 的两目参数、相邻外参和校正参数 |
| 双目 Camera–BaseLink | 更新 `camera.cam0.T_base_cam/pitch` |
| Camera–IMU | `stereo_imu_calib.yaml` |
| 单目标定 | 创建或更新 `usb_mono_calib.yaml` 的单目参数和 RMS |
| 单目 Camera–BaseLink | 更新 `usb_mono_calib.yaml` 的 `camera.T_base_cam/pitch` |

每次都按当前批次的全部已完成阶段重新组装目标文件，后完成的 Camera–IMU 不会
抹掉 BaseLink 字段，反之亦然。未运行的阶段使用上述明确默认值；某阶段已标记完成却
缺少必需字段、出现非有限数或不可用 bias 时则停止更新，保留此前文件，不用默认值掩盖
损坏结果。state 记录聚合文件的来源/输出 SHA-256 和仍使用默认值的字段。
若目录中存在上一聚合格式的受管 `stereo_calib.yaml`，下一次运行会先核对 state 中的
精确路径与 SHA-256，再删除该旧派生文件及空父目录。旧格式的受管
`usb_mono_calib.yaml` 不删除，而是在校验路径与 SHA-256 后按当前结构原子更新；发现
未知文件、内容被改动或符号链接时停止，不做清理或覆盖。

### 5.2 批次内复用

Camera–IMU 和 BaseLink 结果始终记录并复核实际引用的对应模组相机结果，不依赖
“自动找唯一文件”。同一批次可先完成双目，稍后补充 IMU、BaseLink 或 USB 数据后续跑；
已经完成且来源一致的阶段不会重复求解。

`tasks/` 是程序按安装 profile 生成的可追溯 task，不需要操作员编辑。
`.kalibr-cedarobot` 不是正式标定参数；归档整台机器时应与正式结果一起保留。复用前会
核对数据集、profile、软件、task、上游依赖和受管交付摘要。只存在同名 YAML 不足以
视为已完成。

批次一旦产生 state，同名数据目录的内容、软件、BaseLink 配置和受管结果必须保持不变。
已有数据目录消失并出现唯一的新规范目录时，程序把它识别为受控重标定。已安装 profile
的 YAML 字节变化时，程序比较最终生成 task 及其 target、initialization、IMU 输入文档
的 YAML 语义；只改注释、空白或键顺序仍复用，有效字段变化则按依赖重跑。名称不变的
交付在完整性校验后由内部使用 `force` 安全替换，失败时保留旧结果。改变软件、工艺配置
或原地修改数据仍需新建批次并修改 `user.yaml.machine_dir`。旧 revision 结构的 state
不自动接管，应使用新的批次目录运行。

## 6. 失败与并发规则

- 同一批次同一时间只允许一个工作流持有锁；第二个进程立即报错，不等待或并行写结果。
- 发现 Camera–IMU 或 BaseLink 下游数据时，必须同时发现同模组的相机数据；否则预检
  直接报错，不跨模组回退，也不启动任何求解。
- 阶段顺序为双目 → 双目 Camera–IMU → 双目 BaseLink → 单目 → 单目 BaseLink。
  缺少某阶段的数据不会阻止另一模组的可执行阶段，缺少 IMU 数据不阻止两种 BaseLink。
  已开始的求解若抛出异常，本次命令立即停止；修复后重复命令续跑。
- Camera–IMU 失败不删除或降级已完成的双目结果。未改变任何受指纹保护输入、且没有
  发布正式同名文件时，可重复原命令重试；需要替换数据时改用新的规范时间目录，不能
  在原目录内修改。
- 进程中断后不会把半成品当作成功。尚未发布正式同名文件时，下次运行重新执行整个
  阶段；若中断发生在多文件发布过程中，程序保留现场并要求新批次，避免静默删除
  或覆盖无法完整核验的文件。
- 受管结果目录存在未知文件、损坏清单或符号链接时拒绝覆盖，保留现场供维护人员检查。
- `waiting_for_dataset` 表示当前已完成可执行阶段，不是错误；终端会打印需要补充的目录角色。
  非规范目录静默跳过；重复的规范候选、缺少前置数据或缺少 BaseLink 配置在求解前返回
  非零状态；`failed` 表示已启动的任务运行失败，需要处理后重试。

## 7. 开发者 profile 与维护接口

发布 profile 的源码位于 `config/mechine/workflow_v1.0.0/`，安装后位于：

```text
<prefix>/config/mechine/workflow_v1.0.0/
```

它包含默认 AprilGrid、双目初始化、普通和 RS 双目 task、IMU 参数、设备参数固定模板、
普通和 RS Camera–IMU task、双目 BaseLink task，以及单目和单目 BaseLink task。单目不
引用双目初始化文件。
可选标定板 YAML 位于安装后的 `config/mechine/` 根目录，由 `user.yaml.target` 以stem选择。
profile 必须可移植，不能写机器名、数据绝对路径或输出绝对路径。机器入口以结构化 YAML
修改注入 `dataset.path`、`output.name`、精确的 `camera_calibration.path`，以及明确配置的
`base_link` 和每目平移，不做文本替换。

task 中的 `target.path`、`initialization.path` 和 `imus[].path` 必须写相对路径，以该 task
模板所在的 `workflow_v1.0.0/` 为基准，并且解析后必须仍位于 `config/mechine/` 内。例如：

```text
config/mechine/
├── aprilgrid_large.yaml
├── initialization_wide.yaml
└── workflow_v1.0.0/
    └── stereo_camera_calibration_task.yaml
```

模板中分别填写 `../aprilgrid_large.yaml` 和 `../initialization_wide.yaml`。保留在 profile
目录内的文件仍直接写文件名，例如 `实验室_aprilgrid_8x10.yaml`。绝对路径、逃出 `config/mechine/` 的 `../..` 和
符号链接都会在求解前拒绝。`user.yaml.target` 一旦填写，仍作为操作接口显式覆盖各 task
的 `target.path`。

产线精简包通过下列命令发布：

```bash
python3 tools/install_runtime.py \
  --build-dir build/release \
  --prefix /opt/kalibr_cedarobo_v1.0.0 \
  --user-config /path/to/user.yaml
```

精简包保留 `bin/`、`lib/`、`libexec/`、`config/`、根 `LICENSE` 和 `licenses/`，不含
`include/`、`share/`。更新已有安装时增加 `--replace`；现场 `config/user.yaml` 以及
`config/mechine/` 下与新包不重名的新增 YAML 会保留，正式发布包中的同名文件优先。

算法开发者可在已安装软件中临时修改现有 profile YAML。再次运行同一批次时，工作流按
实际语义变化重跑对应任务及其下游，并在新交付完整成功后覆盖同名旧结果；无效模板在
启动任何受影响求解前报错。`mechine_base_param.yaml` 变化只重新生成设备参数文件。安装目录的
临时修改会被后续安装覆盖，正式修改仍应回写源码 profile 并重新发布。

研发需要复现单阶段时，使用 `.kalibr-cedarobot/tasks/` 中保存的 task 配合现有接口：

```bash
kalibr-noros validate --config <生成的task.yaml>
kalibr-noros calibrate cameras --config <相机task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate cameras-rs --config <RS相机task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate imu-camera --config <普通IMU task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate imu-camera-rs --config <RS IMU task.yaml> --output-dir <machine_dir>/result
kalibr-noros calibrate cameras-baselink --config <BaseLink task.yaml> --output-dir <machine_dir>/result
```

这些是维护接口。直接运行时仍使用普通 CLI 的平铺正式文件＋同名证据目录；使用者自行
负责阶段依赖、结果路径、命名冲突和来源追溯。产线常规操作继续使用
`kalibr-cedarobot machine`，由机器入口生成数据集同名任务目录、根目录报告和 `calib_params`。
