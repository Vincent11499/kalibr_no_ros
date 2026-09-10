# v1.0.0 标定配置示例

本目录独立提供三类任务的最简版与完整注释版。相机模型统一为 `pinhole-equi`，
cam0 表示左目、cam1 表示右目；单目示例只使用 cam0。
根目录所有 YAML 配置均不带注释；`all_params/` 提供带注释的完整任务和配套配置。

目录任务按 dataset.yaml 中的传感器 ID 读取，不需要 topic 或 IMU rostopic。
改成 bag 输入时，须补齐相机 topic 和 imu.yaml 的 rostopic，值对应 bag 中的话题。

| 任务 | 最简版 | 完整版 | 主要结果 |
|---|---|---|---|
| 单目内参 | mono_camera_calibration_task.yaml | all_params/mono_camera_calibration_task_full.yaml | cam0 内参、四项等距畸变 |
| 双目内外参 | stereo_camera_calibration_task.yaml | all_params/stereo_camera_calibration_task_full.yaml | 两目内参、畸变、相邻外参 |
| 双目＋IMU 外参 | camera_imu_calibration_task.yaml | all_params/camera_imu_calibration_task_full.yaml | 相机–IMU 外参、时间偏移与 IMU 参数 |

根目录还提供以下无注释配置：

| 文件 | 用途 |
|---|---|
| `aprilgrid.yaml` | 标定板尺寸、间距和起始 ID |
| `imu.yaml` | IMU 采样率、噪声密度和随机游走；目录任务按 ID 选择 IMU |
| `initialization_mono.yaml` | 单目初值，默认不启用 |
| `initialization_stereo.yaml` | 双目初值，默认不启用 |
| `initialization_imu.yaml` | Camera–IMU 初值，默认不启用 |
| `evaluation.yaml` | 从已归档观测重新生成指标、判定和图像 |

`all_params/` 中的 `aprilgrid.yaml`、`imu.yaml`、`evaluation.yaml` 保留参数解释与规则示例。
两种任务的 job 分开，不把完整相机标定嵌入 Camera–IMU 任务。

## 准备输入

将本目录复制到自己的可写运行目录，保持根目录与 `all_params/` 的相对位置。完整任务所需的板、IMU 和评价配置也保存在 `all_params/`。
六个 task 的 `dataset.path` 都指向根目录下的占位数据目录
`data/stereo_imu_YYMMDD_hhmm`（完整版使用 `../data/…`），必须替换为本次采集的真实新格式目录。
可以把实际数据放到本目录下的 `data/`，也可以填写相对 task 文件的其他路径。
不要用旧整数 schema 数据改版本号假装新数据。

本示例目录及 `all_params/` 不再放置 `dataset.yaml` 模板。
真实数据集根目录中的 `dataset.yaml` 由采集工程自动生成，目录输入必须保留它。
它记录数据集 ID、相机／IMU 话题、图像目录、时间戳 CSV 和 IMU CSV 路径，
程序据此找到数据；它不是标定任务，也不保存标定结果。
task 的 `dataset.path` 应指向包含该清单的真实数据集目录，无需手动复制示例清单。

AprilGrid 参数来自现有 EVT3.1 数据旁的配置：8×8、tagSize 0.065 m、
tagSpacing 0.3、tagStartId 80。使用前核对实体板。IMU 参数沿用同处现有
100 Hz 配置，仅作为已知输入示例；本次升级没有重新执行 Allan 噪声估计。

## 运行顺序

先将 `kalibr-noros` 加入 PATH，再从复制后的示例目录运行：

```bash
kalibr-noros --version
kalibr-noros validate --config mono_camera_calibration_task.yaml
kalibr-noros calibrate cameras --config mono_camera_calibration_task.yaml --output-dir output
kalibr-noros validate --config all_params/stereo_camera_calibration_task_full.yaml
kalibr-noros calibrate cameras --config all_params/stereo_camera_calibration_task_full.yaml --output-dir output
kalibr-noros calibrate imu-camera --config all_params/camera_imu_calibration_task_full.yaml --output-dir output
```

三个任务可以使用同一输出目录 `output`，结果按任务与相机 ID 命名。
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
`refine` 继续原生 refinement；`direct` 跳过部分初始化求解但不固定最终参数。
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

示例显式使用 4 个检测进程、4 个优化线程、每 worker 两个在途任务、一个 OpenCV
线程；相机 `shuffle: false`。相机同步容差 0.0002 s 对齐采集的 200 µs，
这是示例的显式配对策略；省略时原生默认仍为 0.02 s，不应混为同一数据选择条件。
`output.evaluation_pairing_tolerance_s` 只影响 Camera–IMU 后处理配对，不反馈求解。
默认完整帧焦距初始化、亚像素窗口和位移阈值保留原生行为。

详见 [统一输入／求解／输出契约](../../../docs/V1_INTERFACE_ZH.md)。
