# v1.0.0 标定配置示例

本目录独立提供各类任务的最简版与完整注释版。相机模型统一为 `pinhole-equi`，
cam0 表示左目、cam1 表示右目；单目示例只使用 cam0。
根目录所有 YAML 配置均不带注释；`all_params/` 提供带注释的完整任务和配套配置。

根目录 YAML 是简洁操作模板，可以显式采用非默认参数；`all_params/` 则按以下规则：

- 有固定程序默认值的字段直接填写该默认值；
- 没有固定数值默认的字段填写一个合法示例，并在行内列出可选集合或范围；
- 只能通过省略表示的自动行为保留为注释，并说明省略后的行为；
- 数据路径、标定板和 IMU 物理量仍必须按实际设备修改。

主要无固定默认值字段的合法选择如下；`all_params/` 已从中填写一个示例：

| 字段 | 合法选择或范围 | 示例值 |
|---|---|---|
| `schema_version` | 当前唯一合法字符串 `"1.0.0"` | `"1.0.0"` |
| `job` | `camera_calibration`、`camera_rolling_shutter_calibration`、`camera_imu_calibration`、`camera_imu_rolling_shutter_calibration` | 按文件用途选择 |
| `dataset.type` | `directory` 或 `bag` | `directory` |
| `dataset.path`、各类 `path` | 对应类型的非空有效路径 | 示例相对路径 |
| 普通相机 `model` | `pinhole-radtan`、`pinhole-radtan5`、`pinhole-radtan8`、`pinhole-equi`、`pinhole-fov`、`pinhole-opencv-fisheye`、`omni-none`、`omni-radtan`、`eucm-none`、`ds-none` | `pinhole-equi` |
| 纯视觉 RS 相机 `model` | `pinhole-equi` 或 `pinhole-radtan` | `pinhole-equi` |
| `parallelism`、`detector_processes`、`optimizer_threads` | 任意正整数 | `4` |
| `rolling_shutter.*.max_abs_line_delay_s` | 有限正数且严格大于行时间初值绝对值 | `0.00002 s/行` |
| `knots_per_second` | 任意有限正数 Hz；省略时自动取观测中位频率 | `1.0 Hz` |
| 标定板和 IMU 物理量 | 满足行内范围的实测正数 | 当前板／IMU 示例值 |

目录任务按 dataset.yaml 中的传感器 ID 读取，不需要 topic 或 IMU rostopic。
改成 bag 输入时，须补齐相机 topic 和 imu.yaml 的 rostopic，值对应 bag 中的话题。

| 任务 | 最简版 | 完整版 | 主要结果 |
|---|---|---|---|
| 单目内参 | mono_camera_calibration_task.yaml | all_params/mono_camera_calibration_task_full.yaml | cam0 内参、四项等距畸变 |
| 双目内外参 | stereo_camera_calibration_task.yaml | all_params/stereo_camera_calibration_task_full.yaml | 两目内参、畸变、相邻外参 |
| 双目＋IMU 外参 | camera_imu_calibration_task.yaml | all_params/camera_imu_calibration_task_full.yaml | 相机–IMU 外参、时间偏移与 IMU 参数 |
| 单目／多目滚动快门相机 | camera_rolling_shutter_calibration_task.yaml | all_params/camera_rolling_shutter_calibration_task_full.yaml | K/D、相邻外参、每目行时间与RS残差 |
| 滚动快门双目＋IMU | camera_imu_rolling_shutter_calibration_task.yaml | all_params/camera_imu_rolling_shutter_calibration_task_full.yaml | 每目行时间、相机–IMU 外参、整体时间偏移与 IMU 参数 |
| RS 行时间诊断 | rolling_shutter_diagnosis.yaml | all_params/rolling_shutter_diagnosis_full.yaml | 分块筛查、正式对照、证据分类和建议RS任务 |

普通单目和多目任务的 `solver_mode` 默认均为 `incremental`，也都可以改为
`full_batch`。单目 full-batch 联合优化全部 view 的 K/D 和板位姿；多目还联合优化相邻
baseline。`freeze_intrinsics: true` 仍只适用于至少双目且每目提供完整 K/D 的任务。

根目录还提供以下无注释配置：

| 文件 | 用途 |
|---|---|
| `aprilgrid.yaml` | 标定板尺寸、间距和起始 ID |
| `imu.yaml` | IMU 采样率、噪声密度和随机游走；目录任务按 ID 选择 IMU |
| `initialization_mono.yaml` | 单目初值，默认不启用 |
| `initialization_stereo.yaml` | 双目初值，默认不启用 |
| `initialization_imu.yaml` | Camera–IMU 初值，默认不启用 |
| `evaluation.yaml` | 从已归档观测重新生成指标、判定和图像 |
| `machine_calibration_workflow.yaml` | 机器级入口的最小 `user.yaml`；复制后填写机器目录和 Camera–IMU 模式 |
| `rolling_shutter_diagnosis.yaml` | 输入普通任务，自动判断数据是否支持逐行时间模型 |

`all_params/` 中的 `aprilgrid.yaml`、`imu.yaml`、`evaluation.yaml` 保留参数解释与规则示例。
相机与 Camera–IMU 的 job 分开，不把完整相机标定嵌入 Camera–IMU 任务。

`camera_rolling_shutter_calibration_task.yaml` 当前写成双目；做单目时同时删除第二个
`cameras` 项和对应的 `rolling_shutter.cam1`，做三目及以上时按相同 ID 规则成对增加。
正式 `cameras-rs` 可直接使用完整注释版。临时 `native-rs-cameras` 只支持单目，且不接受
完整注释版中的同步、样条、运动先验和异常点过滤字段；从简洁版改为单目后，还需删除
`calibration.synchronization_tolerance_s`。两条命令共用 task schema，不共用全部后端参数。

## 准备输入

将本目录复制到自己的可写运行目录，保持根目录与 `all_params/` 的相对位置。完整任务所需的板、IMU 和评价配置也保存在 `all_params/`。
task 的 `dataset.path` 都指向根目录下的占位数据目录
`data/stereo_imu_YYMMDD_hhmm`（完整版使用 `../data/…`），必须替换为本次采集的真实新格式目录。
可以把实际数据放到本目录下的 `data/`，也可以填写相对 task 文件的其他路径。
不要用旧整数 schema 数据改版本号假装新数据。

本示例目录及 `all_params/` 不再放置 `dataset.yaml` 模板。
真实数据集根目录中的 `dataset.yaml` 由采集工程自动生成，目录输入必须保留它。
它记录数据集 ID、相机／IMU 话题、图像目录、时间戳 CSV 和 IMU CSV 路径，
程序据此找到数据；它不是标定任务，也不保存标定结果。
task 的 `dataset.path` 应指向包含该清单的真实数据集目录，无需手动复制示例清单。

根目录 AprilGrid 参数来自现有 EVT3.1 数据旁的配置：8×8、tagSize 0.065 m、
tagSpacing 0.3、tagStartId 80。`all_params/aprilgrid.yaml` 为展示程序默认值，将
`tagStartId` 写为默认 `0`；其余板尺寸没有程序默认值。两份都必须在使用前核对实体板。
IMU 参数沿用同处现有 100 Hz 配置，仅作为无默认物理量的合法示例；本次升级没有
重新执行 Allan 噪声估计。

## 运行顺序

先将 `kalibr-noros` 加入 PATH，再从复制后的示例目录运行：

```bash
kalibr-noros --version
kalibr-noros validate --config mono_camera_calibration_task.yaml
kalibr-noros calibrate cameras --config mono_camera_calibration_task.yaml --output-dir output
kalibr-noros validate --config all_params/stereo_camera_calibration_task_full.yaml
kalibr-noros calibrate cameras --config all_params/stereo_camera_calibration_task_full.yaml --output-dir output
kalibr-noros calibrate cameras-rs --config all_params/camera_rolling_shutter_calibration_task_full.yaml --output-dir output
kalibr-noros calibrate imu-camera --config all_params/camera_imu_calibration_task_full.yaml --output-dir output
```

各任务可以使用同一输出目录 `output`，结果按任务与相机 ID 命名；临时原生 RS 单目
对照结果会增加 `native_` 前缀。
IMU task 省略 `camera_calibration`，运行时从输出目录查找唯一匹配双目/多目结果。
有多个结果时填写显式路径；单独运行 `validate` 时也需要该路径，因为该命令没有输出目录上下文。
IMU 任务要求相机结果的模型、尺寸、
相机顺序和话题与本次数据匹配。完整／最简 task 是可替换方案，不必重复标定；
应使用新的输出目录保留已有运行记录。

最简版关闭观测归档和图像可视化，仍生成结果、指标和判定文件。完整版打开最终
观测与选择历史归档，每相机最多可视化 30 帧、双目最多 30 对，不复制原图。
指标始终使用全部最终有效观测，30 只是图像展示上限。

## 初始化与离线评价

初始化默认未启用。`initialization_*.yaml` 中的 900 px 焦距、零畸变、
100 mm baseline 和单位 Camera–IMU 变换都是教学占位值，必须先换成可信初值。
然后才取消对应完整 task 顶层 `initialization` 的三行注释。
提供的数值始终作为原生初始化与后续联合优化的起点，不增加先验约束。
固定内参另由 `calibration.freeze_intrinsics` 控制，仅双目及以上并具备完整 seed
时可打开。修改初值和算法参数后需另建候选，不能复用历史数值一致性结论。

完整双目运行结束后，独立重新评价：

```bash
kalibr-noros evaluate --run output/stereo --config evaluation.yaml --output-dir output/stereo_evaluation
```

数据目录移动后可增加 `--dataset <新的数据集根目录>`。离线评价不检测角点、不运行
优化器，也不覆盖原运行。缺少 `observations/` 的最简运行只能按已保存的汇总指标
重新判定；无法重画逐角点证据或改变校正设置后重算极线误差。

`assessment.rules: []` 表示尚未配置业务验收门槛，正式状态是 `not_evaluated`。
参考等级仅供显示，不能代替产线判定。`all_params/evaluation.yaml` 注释中提供规则
语法示例，示例 0.5 px 阈值未经业务验证，默认未启用。

## 默认值与比较边界

以下非默认值只属于根目录简洁操作模板，不是 `all_params/` 的默认值。根目录简洁 task
显式使用 4 个检测进程、4 个优化线程；这两个字段没有统一数值默认，
省略时保留各原生阶段行为。普通相机简洁 task 还把 `shuffle` 从默认 `true` 改为
`false`，把同步容差从默认 0.02 s 改为 0.0002 s。RS 相机简洁 task 使用
`window_half_size_px=7`、`max_displacement_px=1.0 px` 和 0.0001 s 同步容差，
分别覆盖默认的 `2`、$\sqrt{1.5}$ px 和 0.02 s。

`all_params/` 中的固定默认值已经展开：亚像素半窗口 `2 px`、最大位移
$\sqrt{1.5}$ px、相机同步容差 `0.02 s`、评价配对容差 `0.0002 s`、
`shuffle: true`，观测归档、选择历史和可视化均为 `false`。detector、optimizer 和
公共 parallelism 没有统一数值默认，因此示例选用合法的 `4`，注释中明确允许任意
正整数。RS 行时间上界、轨迹 knot 频率等无固定默认数值的字段同样给出合法示例及范围。

根目录 `evaluation.yaml` 主动开启观测归档、选择历史、OpenCV 导出和可视化；这些开关
的程序默认均为 `false`。标定板行列、尺寸、间距以及 IMU 采样率和噪声参数属于实物
输入，没有通用程序默认值。AprilGrid 仅 `tagStartId` 可省略且默认 `0`；根目录简洁
模板的 `80` 是当前板参数。`output.evaluation_pairing_tolerance_s` 的程序默认是 0.0002 s，只影响
Camera–IMU 后处理配对，不反馈求解。

详见 [统一输入／求解／输出契约](../../../docs/V1_INTERFACE_ZH.md)。
