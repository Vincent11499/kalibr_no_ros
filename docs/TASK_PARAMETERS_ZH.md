# 标定 Task 参数与超参数完整说明

本文基于当前 `docs/kalibr-native-v2.1` 分支源码，逐项说明
`camera_calibration` 与 `camera_imu_calibration` task 中真正受支持的字段、默认值、
CLI 映射、作用阶段、内部算法和有效范围。审计基准为提交
`39b5b8280f347a68cecf5c1cfe437ae4c75f0877` 加当前工作区改动，日期为
2026-08-26。

需要先区分五类配置：

| 类别 | 典型字段 | 是否可能改变数值结果 |
|---|---|---:|
| 数据选择 | `time_range_s`、`frequency_hz` | 是 |
| 算法超参数 | `information_gain_tolerance`、`reprojection_sigma_px` | 是 |
| 显式物理初值 | `initialization.path`、`initialization.strategy` | 是；改变前置阶段和初始点 |
| 执行资源 | `detector_processes`、`optimizer_threads` | 理论算法不变，但并行浮点归约可能造成末位差异 |
| 显示与输出 | `verbose`、`export_poses`、`interactive_report` | 不应改变目标函数，但会明显影响耗时，部分选项会关闭多进程检测 |

本文中的“默认值”指不在 task 和 CLI 中指定该字段时，当前程序最终采用的值。
“有效范围”分为：

- **已强制校验**：不满足时会在 task 适配层或数据读取层报错；
- **算法有效范围**：当前代码未必提前校验，但越界会导致无意义配置、矩阵奇异或运行期失败；
- **特殊值**：具有与普通数值不同的控制语义。

## 1. 两类 task 的完整示例

### 1.1 `camera_calibration`

下面列出当前适配器认识的全部字段。实际任务不需要把默认值全部展开。

```yaml
schema_version: 1
job: camera_calibration

dataset:
  type: bag
  path: /data/camera.bag
  time_range_s: [0.0, 60.0]
  frequency_hz: 20.0

target:
  path: aprilgrid.yaml

cameras:
  - {topic: /cam0/image_raw, model: pinhole-radtan5}
  - {topic: /cam1/image_raw, model: pinhole-radtan5}

calibration:
  window_half_size_px: 2
  max_displacement_px: 1.224744871391589
  focal_initialization_min_visible_corner_ratio: 1.0
  synchronization_tolerance_s: 0.02
  qr_tolerance: 0.02
  information_gain_tolerance: 0.2
  min_views_for_outlier_statistics: 20
  shuffle: false
  remove_outliers: true
  final_filtering: true
  blake_zisserman: false
  verbose: false
  show_extraction: false
  export_poses: false
  interactive_report: false

execution:
  detector_processes: 4
  optimizer_threads: 4
  detector_inflight_per_worker: 2
  detector_opencv_threads: 1
  profiling_memory_sample_interval_s: 0.25
```

### 1.2 `camera_imu_calibration`

```yaml
schema_version: 1
job: camera_imu_calibration

dataset:
  type: bag
  path: /data/imu_camera_bag
  time_range_s: [0.0, 60.0]
  frequency_hz: 20.0

target:
  path: aprilgrid.yaml

camera_calibration:
  path: camchain.yaml

imus:
  - {path: imu.yaml, model: scale-misalignment}

calibration:
  window_half_size_px: 2
  max_displacement_px: 1.224744871391589
  max_iterations: 30
  time_offset_padding_s: 0.03
  reprojection_sigma_px: 1.0
  synchronize_clocks: false
  estimate_multi_imu_delay: false
  calibrate_time_offset: true
  recover_covariance: false
  recompute_camera_chain_extrinsics: false
  verbose: false
  show_extraction: false
  extraction_stepping: false
  export_poses: false
  interactive_report: false

execution:
  detector_processes: 4
  optimizer_threads: 4
  detector_inflight_per_worker: 2
  detector_opencv_threads: 1
  profiling_memory_sample_interval_s: 0.25
```

以上两份“全部字段”示例有意不默认启用 `initialization`：未配置时保持原生自动初始化
基线。要启用时在 task 顶层增加第 2.2 节的块，或使用同节所述 CLI 覆盖。

## 2. 公共 task 字段

### 2.1 顶层字段

| 字段 | 类型与范围 | 作用 |
|---|---|---|
| `schema_version` | 整数，当前必须为 `1` | task 接口版本；不是结果 `calibration.yaml` 的版本 |
| `job` | `camera_calibration` 或 `camera_imu_calibration` | 选择任务及允许出现的顶层字段 |
| `dataset` | mapping，必填 | 数据来源和数据裁剪参数 |
| `target` | mapping，必填 | 标定板配置 |
| `initialization` | mapping，可省略 | 独立物理初值文件与 `refine`/`direct` 策略 |
| `calibration` | mapping，可省略 | 算法、诊断和结果显示参数 |
| `execution` | mapping，可省略 | 进程、线程、队列和 profiling 配置 |

相机任务额外要求 `cameras`；Camera–IMU 任务额外要求 `camera_calibration` 和
`imus`。顶层未知字段会被拒绝。

### 2.2 `initialization`

```yaml
initialization:
  path: camera_calibration_initialization.yaml
  strategy: refine
```

| 字段 | 默认值 | 范围与路径语义 | 作用 |
|---|---:|---|---|
| `path` | 无，配置该块时必填 | 非空路径；task 内相对路径以 task YAML 所在目录为基准 | 选择与当前 job 匹配的 schema v1 物理 seed |
| `strategy` | `refine` | `refine` 或 `direct` | 控制 seed 是进入原生前置 refinement，还是直接跳过已完整提供的初值阶段 |

CLI 的 `--initialization` 和 `--initialization-strategy` 分别覆盖对应字段；CLI seed
相对路径以当前工作目录为基准。只给策略而 task/CLI 都没有路径会被拒绝。

两类初值文件字段很多，且涉及严格坐标方向、相机模型维数和 IMU 模型门控，完整格式
见 [`INITIALIZATION_ZH.md`](INITIALIZATION_ZH.md)。需要特别注意：seed 不是 fixed
parameter，也不增加先验残差；无 seed 时不进入新路径。

### 2.3 `dataset`

| 字段 | 默认值 | 强制/有效范围 | 内部作用 |
|---|---:|---|---|
| `type` | 无，必填 | `bag` 或 `directory` | `bag` 自动识别 ROS1 文件或 ROS2 bag 目录；`directory` 要求根目录含 `dataset.yaml` |
| `path` | 无，必填 | 存在的文件或目录 | 相对路径以 task YAML 所在目录为基准 |
| `time_range_s` | 全部数据 | 长度为 2；读取层要求 `start < end` | 先裁剪数据，再做图像降采样 |
| `frequency_hz` | 不降采样 | 读取层强制 `> 0` | 按时间间隔选择图像；不对 IMU 流降采样 |

当前 no-ROS 读取层把 `time_range_s` 解释为相对于**每个数据流第一条 header
时间戳**的秒数，而不是绝对时间戳。`frequency_hz` 使用保序贪心选择：保留第一帧，
之后仅保留与上一保留帧间隔至少为 $1/f$ 的帧。它会减少检测和视觉误差项数量，
因此不仅影响耗时，也会改变最终数值。

### 2.4 `target`

可以引用现有文件：

```yaml
target: {path: aprilgrid.yaml}
```

也可以内联，适配器会临时生成原生 Kalibr target YAML：

```yaml
target:
  type: aprilgrid
  parameters:
    tagRows: 6
    tagCols: 6
    tagSize: 0.088
    tagSpacing: 0.3
    tagStartId: 100
```

`tagStartId` 是左上角标签的检测 ID，按先行后列的顺序连续递增。上例使用
$6\times6$ 标定板，因此有效 ID 区间为 $[100,136)$；检测到的 ID 100 映射到
标定板局部编号 0，ID 135 映射到局部编号 35。省略该字段时默认为 0，与原生
Kalibr 行为一致。

支持的标定板及原生检查范围为：

| `type` | 必填参数 | 范围 |
|---|---|---|
| `aprilgrid` | `tagRows`、`tagCols`、`tagSize`、`tagSpacing`；可选 `tagStartId` | 行列为整数且 `>= 3`；`tagSize` 为正数（米）；`tagSpacing` 为正的间距/边长比；`tagStartId` 为非负整数且 `tagStartId + tagRows * tagCols <= 587` |
| `checkerboard` | `targetRows`、`targetCols`、`rowSpacingMeters`、`colSpacingMeters` | 行列为整数且 `>= 3`；间距为正浮点数 |
| `circlegrid` | `targetRows`、`targetCols`、`spacingMeters`、`asymmetricGrid` | 行列为整数且 `>= 3`；间距为正浮点数；最后一项为布尔值 |

标定板尺寸直接决定 Camera–IMU 平移和相机 baseline 的米制尺度，不能作为“只影响
检测”的参数随意调整。

当前 AprilGrid 检测家族仍固定为 `tag36h11`，仅支持从 `tagStartId` 开始的连续
ID，不支持任意离散 ID 映射。其他 AprilTag family 留待后续版本扩展。

## 3. `camera_calibration` 专用配置

### 3.1 `cameras[]`

每项必须包含 `topic` 和 `model`：

```yaml
cameras:
  - {topic: /cam0/image_raw, model: pinhole-radtan5}
```

当前相机标定入口接受以下模型名：

| task 模型名 | 投影/畸变模型 |
|---|---|
| `pinhole-radtan` | pinhole + 4 参数 radtan |
| `pinhole-radtan5` | pinhole + OpenCV 顺序的 5 参数 radtan |
| `pinhole-radtan8` | pinhole + OpenCV rational 顺序的 8 参数 radtan |
| `pinhole-equi` | pinhole + 4 参数 equidistant |
| `pinhole-fov` | pinhole + FOV distortion |
| `pinhole-opencv-fisheye` | 完整 OpenCV fisheye 扩展 |
| `omni-none` | unified omni，无额外 distortion |
| `omni-radtan` | unified omni + radtan |
| `eucm-none` | Extended Unified Camera Model |
| `ds-none` | Double Sphere |

`id` 当前可以出现在条目中，但 task 适配器不会使用它；相机编号严格由列表顺序决定。
未配置 `initialization` 时，内参、畸变、每帧 target pose 和相邻相机 baseline 仍全部
由原生路径自动初始化。配置后，`refine` 可给部分内参/畸变/baseline 并保留相应前置
优化；相机 `direct` 要求每台相机内参和畸变、以及所有相邻 baseline 完整，然后跳过
这些前置初值优化。两种策略的最终增量标定参数仍按原生活动集合优化，详见
[`INITIALIZATION_ZH.md`](INITIALIZATION_ZH.md)。

`pinhole-radtan8` 的畸变顺序固定为
`[k1,k2,p1,p2,k3,k4,k5,k6]`。OpenCV YAML 中的
`distortion_model: rational_polynomial` 会在导入时规范化为 Kalibr camchain 的
`distortion_model: radtan8`；导出时执行反向映射。不能把 8 个系数截成 `radtan5`。

### 3.2 算法超参数总表

| 字段 | 默认值 | 程序接受/算法有效范围 | 作用阶段与影响 |
|---|---:|---|---|
| `window_half_size_px` | `2` px | 强制整数 `>= 1` | AprilGrid `cornerSubPix` 半窗口；实际搜索区域为 $(2w+1)\times(2w+1)$ |
| `max_displacement_px` | $\sqrt{1.5}\approx1.224745$ px | 强制有限数 `> 0` | AprilGrid 亚像素角点相对原始检测的最大允许位移 |
| `focal_initialization_min_visible_corner_ratio` | `1.0` | 强制有限数，范围 $(0,1]$ | 无显式内参初值时，筛选可进入 pinhole 焦距解析初始化的部分标定板观测；`1.0` 严格保持原生完整帧路径 |
| `synchronization_tolerance_s` | `0.02` s | 当前未前置校验；算法上应 `>= 0` | 多相机观测近似同步、相机连接图和 baseline 初始化 |
| `qr_tolerance` | `0.02` | 接受任意 float；**当前不生效** | 只生成 `--qr-tol`，解析后未写入线性求解器 |
| `information_gain_tolerance` | `0.2` | 特殊值 `-1`；常规值建议 `>= 0` | 决定增量估计器是否保留新 target view |
| `min_views_for_outlier_statistics` | `20` | 强制整数 `>= 1` | 决定何时开始建立重投影误差统计并删角点 |
| `shuffle` | `true` | 布尔值 | 是否随机打乱 target view 的增量加入顺序 |
| `remove_outliers` | `true` | 布尔值 | 是否运行显式 $4\sigma$ 角点删除流程 |
| `final_filtering` | `true` | 布尔值 | 所有 view 处理完后是否再检查全部已接受 batch |
| `blake_zisserman` | `false` | 布尔值 | 是否对每个二维重投影误差启用 Blake–Zisserman M-estimator |

#### `focal_initialization_min_visible_corner_ratio`

该字段只作用于没有提供相机内参初值时的 pinhole 系列焦距解析初始化。对一帧观测定义：

$$
\rho_{\mathrm{visible}}
=
\frac{N_{\mathrm{valid\ corners}}}
     {N_{\mathrm{all\ target\ corners}}}.
$$

其中，有效角点是经过标定板检测、ID 映射和亚像素有效性检查后仍可用的角点；分母是
标定板的理论角点总数。以 8×8 AprilGrid 为例，内部目标为 16×16 个角点，因此设置
`0.75` 表示一帧至少需要 192/256 个有效角点，才会进入候选检查。

```yaml
calibration:
  focal_initialization_min_visible_corner_ratio: 0.75
```

- 字段省略或设为 `1.0`：直接调用冻结 ETHZ 的完整标定板实现，保持原生运算顺序和
  数值行为；
- 小于 `1.0`：启用部分可见扩展，先按上述全局比例筛帧；
- 全局比例只是第一道门槛。内部还固定要求足够的行内角点数、角点跨度、有效行数、
  跨行覆盖范围、圆拟合条件和稳定圆交点，并使用中位数/MAD 排除焦距异常候选；
- 已通过 `initialization` 提供 `intrinsics` 时，解析焦距初始化被跳过，因此该字段不
  影响该相机；
- Camera–IMU task 读取既有相机标定，不重新执行相机焦距初始化；当前 task 解析器会忽略
  该字段，但不应配置它。

降低该值改变了可用于产生焦距初值的观测集合，只解决“没有完整帧导致无法产生 seed”
的问题，不等价于改善数据可观性。它也不会改变后续单相机 LM、双目 baseline 初始化、
full-batch refinement 或最终增量 GN 的超参数。

#### `synchronization_tolerance_s`

设一条新观测时间为 $t_i$，数据库已有 target view 时间为 $t_j$。若最近时间满足

$$
\left|t_i-t_j\right|\leq\Delta t_{\mathrm{sync}},
$$

则两条观测归入同一 target view；否则创建新 view。该字段不修改原始时间戳，只影响
多相机观测如何分组。

- 过小：双目同一时刻的观测不能配对，连接图边数减少，baseline 可能无法初始化；
- 过大：不同真实时刻可能被错误合并，甚至同一相机向同一 view 写入两次；
- 单目时一般保留默认值即可，因为不存在跨相机配对收益。

当前 task 层未拒绝负值，但负值会让任何观测都无法与已有 view 合并，没有合理用途。

#### `information_gain_tolerance`

每个候选 target view 先作为一个 batch 参与增量求解。增量估计器根据标定参数信息
矩阵的奇异值计算信息增益：

$$
\Delta I
=
\frac{1}{2}
\left(
\sum_k\log_2\sigma_{k,\mathrm{new}}
-
\sum_k\log_2\sigma_{k,\mathrm{old}}
\right).
$$

正常加入流程在下列条件之一满足时保留该 batch：

1. 解有效且 $\Delta I$ 大于 `information_gain_tolerance`；
2. 解有效且标定参数的数值秩提高。

阈值越高，保留的 view 通常越少，运行时间和内存下降，但覆盖不足会降低精度或
可观性。`-1` 是原生 CLI 约定的“尽量保留所有有效图像”特殊值；从实际代码看，
它并不是无条件 `force=true`，无效或严重不收敛的 batch 仍可能被拒绝。

#### `shuffle`

该字段只打乱**已完成角点检测后的 target view 加入顺序**，不打乱图像读取或标定板
检测顺序。由于信息增益、当前秩、异常值统计都依赖已经接受的 batch，顺序会改变
最终保留集合和浮点路径。

- `true`：ETHZ 原生默认行为；不同运行可能产生不同插入顺序；
- `false`：按数据库时间顺序处理，适合作为数值回归和 benchmark 基线。

#### `remove_outliers`、`min_views_for_outlier_statistics` 与 `final_filtering`

显式异常值流程在已接受 batch 数满足

$$
N_{\mathrm{active}}
>
N_{\mathrm{min}}N_{\mathrm{cam}}
$$

后启动。注意源码实际使用的是“每相机最少 view 数乘相机数”，不只是 CLI help 所说
的 raw view 数。程序统计每个相机全部当前重投影误差的均值和标准差，并删除任一
坐标分量满足下式的角点：

$$
|e_x|>4\sigma_x
\quad\text{或}\quad
|e_y|>4\sigma_y.
$$

删除后会重建对应 batch 并重新送入增量估计器。

- `remove_outliers: false`：完全关闭该显式删除流程；此时 `final_filtering` 无效；
- `final_filtering: true`：最后再次扫描所有已接受 batch；通常更稳健，但增加耗时；
- `min_views_for_outlier_statistics` 太小：早期统计不稳定，可能误删；太大：短数据集
  可能只在最终过滤时处理，若又关闭最终过滤则可能完全不删异常点。

#### `blake_zisserman`

启用后，每个二维重投影 error term 都使用 Blake–Zisserman 鲁棒权重。当前构造固定
使用二维误差，内部默认 `pCut=0.999`、`wCut=1e-6`；task 只暴露开关，不暴露这两个
参数。它与上面的显式 $4\sigma$ 删除是两套独立机制，可以同时启用。

该模式会改变目标函数权重，不能只视为性能选项。当前冻结数值基线使用
`blake_zisserman: false`。

#### `qr_tolerance` 的当前真实状态

task 会把它转换为 `--qr-tol`，原生参数解析器也会读入 `parsed.qrTol`；但是相机标定
入口没有执行

```python
linearSolverOptions.qrTol = parsed.qrTol
```

相邻位置仍保留注释掉的 `qrTol` 赋值。因此当前任何 `qr_tolerance` 数值都不会改变
QR 分解、秩判断或标定结果。真正使用的 QR tolerance 仍由增量线性求解器自动计算。
该字段应视为兼容接口，不应纳入有效调参矩阵。

### 3.3 显示和输出字段

| 字段 | 默认值 | 作用 | 性能注意事项 |
|---|---:|---|---|
| `verbose` | `false` | Debug 级日志 | 会关闭并行角点检测，明显增加终端 I/O |
| `show_extraction` | `false` | 显示标定板检测画面 | 会关闭并行角点检测，需要 GUI |
| `export_poses` | `false` | 额外输出 `poses.csv` | 只增加结果序列化 |
| `interactive_report` | `false` | 标定结束后在屏幕打开报告 | `false` 仍会生成 `report.pdf`；无桌面环境应保持关闭 |

## 4. `camera_imu_calibration` 专用配置

### 4.1 `camera_calibration` 输入及被优化的参数边界

```yaml
camera_calibration: {path: camchain.yaml}
```

这里既接受原生 Kalibr camchain，也接受本项目 `schema_version: 2` 的相机标定结果。
它提供：

- 每个相机的投影模型、内参、畸变和分辨率；
- 相机 topic；
- 相邻相机 baseline 初值。

Camera–IMU 标定中，相机内参和畸变始终固定，不会被联合优化。默认情况下，相邻相机
baseline 也固定；只有 `recompute_camera_chain_extrinsics: true` 才将 cam1 及后续
baseline 设为 active。无论该开关为何值，$\mathbf T_{\mathrm{cam0}\leftarrow\mathrm{imu}}$
始终是联合优化的标定状态。

可选 `camera_imu_calibration_initialization` 文件不会替代这个 camchain：它只给
${}^{C_0}_{I_0}\mathbf T$、每相机时间偏移、重力、bias、IMU intrinsic，以及多 IMU
相对状态的 seed。相机内参与相邻 baseline 仍来自 `camera_calibration.path`，baseline
是否 active 仍只由上述开关决定。

### 4.2 `imus[]`

```yaml
imus:
  - {path: imu.yaml, model: scale-misalignment}
```

`path` 指向包含 topic、噪声密度、随机游走和采样率的原生 IMU YAML。第一个 IMU
是参考 IMU。`id` 当前不参与编号，编号由列表顺序决定。

| `model` | 额外参与联合优化的 IMU 参数 | 初始化 |
|---|---|---|
| `calibrated` | 不估计尺度/非正交；仍估计 gyro/accel bias spline | 无 seed 时 bias 由旋转初值和零值初始化；可给两类 bias seed |
| `scale-misalignment` | accel、gyro 下三角标度/非正交矩阵；gyro 到 IMU 旋转；gyro 对加速度敏感矩阵 | 无 seed 时单位阵/零值；可显式给 `M_accel`、`M_gyro`、`C_gyro_i`、`A_gyro_accel` |
| `scale-misalignment-size-effect` | 上述参数加 accelerometer size-effect lever arms | 可再给 `ry_i_m`、`rz_i_m`；`rx_i` 保持原生零规范约束 |

这些矩阵参数当前没有上下界。高维模型需要充分的三轴旋转、角加速度和线加速度激励；
数据激励不足时，即使求解器报告收敛，也可能存在强相关或不物理的内参。

当前语法和原生路径允许多个 IMU。非参考 IMU 还会估计相对旋转和平移；
`estimate_multi_imu_delay` 仅控制它相对参考 IMU 的时间延迟初值/固定修正。

### 4.3 算法超参数总表

| 字段 | 默认值 | 程序接受/算法有效范围 | 作用阶段与影响 |
|---|---:|---|---|
| `window_half_size_px` | `2` px | 强制整数 `>= 1` | Camera–IMU 重新检测 AprilGrid 时的 `cornerSubPix` 半窗口 |
| `max_displacement_px` | $\sqrt{1.5}\approx1.224745$ px | 强制有限数 `> 0` | Camera–IMU 检测中亚像素角点允许偏离原始检测的最大距离 |
| `max_iterations` | `30` | 当前只解析整数；算法上应 `>= 1` | 最终 Camera–IMU 联合 LM 的最大迭代数 |
| `time_offset_padding_s` | `0.03` s | 当前未前置校验；启用时间标定时应 `> 0` | 扩展 pose spline，并预注册时间变化可能触及的 spline 系数 |
| `reprojection_sigma_px` | `1.0` px | 当前未前置校验；物理和数值上必须 `> 0` | 所有相机重投影残差的协方差/权重 |
| `synchronize_clocks` | `false` | 布尔值 | 数据读取预处理：拟合 header 时间到 record 时间的时钟映射 |
| `estimate_multi_imu_delay` | `false` | 布尔值 | 非参考 IMU 与参考 IMU 的角速度模长互相关和连续微调 |
| `calibrate_time_offset` | `true` | 布尔值 | Camera–IMU 时间偏移的互相关初值及联合优化开关 |
| `recover_covariance` | `false` | 布尔值 | 最终优化后额外执行标定组 covariance 恢复 |
| `recompute_camera_chain_extrinsics` | `false` | 布尔值 | 是否在联合 LM 中放开 cam1 及后续相邻 baseline |

#### `max_iterations`

它只控制最终联合 LM，不控制以下初始化优化：

- Camera–IMU rotation 与 gyro bias 初值：固定最多 50 次；
- 多 IMU 相对旋转初值：固定最多 50 次；
- 时间偏移互相关：不是 LM；
- pose/bias spline 初始化：不是最终联合 LM。

最终联合优化仍可能因固定停止条件提前结束：

```text
convergenceDeltaX = 1e-5
convergenceDeltaJ = 1e-2
LM initial lambda = 10
```

因此增大 `max_iterations` 只提高迭代上限，不保证实际运行更多轮。当前 task 层未拒绝
零或负数；为避免 C++ 边界行为，只应使用正整数。

#### `reprojection_sigma_px`

每个角点使用二维各向同性协方差

$$
\mathbf R_{\mathrm{cam}}
=
\sigma_{\mathrm{px}}^2\mathbf I_2,
\qquad
\mathbf W_{\mathrm{cam}}
=
\mathbf R_{\mathrm{cam}}^{-1}.
$$

相机误差项可写为

$$
\mathbf r_{c,k}
=
\mathbf z_{c,k}
-
\boldsymbol\pi_c
\left(
{}^{c}_{b}\mathbf T;
{}^{b}_{w}\mathbf T(t_k+\Delta t_c);
{}^{w}\mathbf p_k
\right).
$$

减小 sigma 会提高视觉误差相对 IMU 误差的权重；增大 sigma 会降低视觉约束权重。
它会影响外参、时间偏移、轨迹、bias 和 IMU intrinsic 的联合平衡，不只是最终报告中
重投影误差的显示尺度。`0` 会使协方差不可逆；负值虽然在平方后可能得到同一权重，
但物理意义错误，不能使用。

#### `calibrate_time_offset` 与 `time_offset_padding_s`

启用时间标定时，每个相机先用视觉角速度模长和参考 IMU gyro 模长做互相关，得到
离散时间偏移初值 $\Delta t_{c,0}$；随后为每个相机建立 active 标量
$\delta t_c$，联合优化中实际取样时间为

$$
t_{\mathrm{imu}}
=
t_{\mathrm{cam}}
+
\Delta t_{c,0}
+
\delta t_c.
$$

显式初始化会改变“互相关初值”这一步：`refine` 把给定时间作为 base，再计算剩余相关
修正；`direct` 对已经给出 `timeshift_cam_imu_s.camN` 的相机跳过互相关。两种策略下，
只要 `calibrate_time_offset: true`，最终 $\delta t_c$ 仍是 active；`direct` 不等于固定
时间偏移。`refine` 时间 seed 要求此开关为 `true`。

`time_offset_padding_s` 不是残差正则项，也不是显式 box constraint。它有两个作用：

1. 在 pose spline 两端增加可用时间；
2. 构造误差项时，把初值左右 padding 范围内可能涉及的 spline 控制点预先注册进
   Jacobian 稀疏结构。

若优化后的时间变化超出注册 buffer，表达式会报
`Spline Coefficient Buffer Exceeded`，而不是自动扩大范围。padding 太小会限制可容纳
的初值误差；太大会让每个误差项关联更多 spline 控制点，增大 Jacobian、Hessian、
内存和求解时间。应至少覆盖预计的**互相关初值剩余误差**，而不是盲目取很大值。

设置 `calibrate_time_offset: false` 时：

- 不执行 Camera–IMU 时间偏移互相关；
- 时间偏移 design variable inactive；
- 无显式初值时，结果 camchain 不写新的 `timeshift_cam_imu`；若 `direct` 给出了某相机
  的时间 seed，则把该固定预计算偏移写入结果，避免初值信息丢失；
- `time_offset_padding_s` 对时间参数不再有调节意义。

#### `synchronize_clocks`

该字段与 Camera–IMU 时间偏移标定不是同一个问题。它在数据读取时使用每条消息的
传感器 header 时间 $t_h$ 和 bag record 时间 $t_r$，拟合远端时钟到本地记录时钟的
映射，再把校正后的时间送入后续检测和优化：

$$
t_{\mathrm{used}}=f(t_h;t_r).
$$

适用于传感器时钟与主机记录时钟存在速率差或网络传输时钟问题的情况。错误启用可能
把真实硬件时间关系映射到主机到达时间，反而恶化标定。目录数据集的 record 时间与
header 时间相等，因此该开关对目录输入没有可学习的校正量。

#### `estimate_multi_imu_delay`

只有第二个及后续 IMU 受影响。算法先用两个 IMU 的角速度模长做互相关，再用
一维无导数优化细化延迟。所得 `timeOffset` 作为固定时间修正用于该 IMU 的误差项，
不是最终联合 LM 中的 active 时间 design variable。

单 IMU 任务中打开该字段没有实际作用。多 IMU 缺少共同动态时间段或角速度激励时，
互相关可能不可靠。

#### `recompute_camera_chain_extrinsics`

对于 $N$ 个相机，Camera–IMU 变换按相机链复合：

$$
{}^{c_N}_{i}\mathbf T
=
{}^{c_N}_{c_{N-1}}\mathbf T
\cdots
{}^{c_1}_{c_0}\mathbf T
{}^{c_0}_{i}\mathbf T.
$$

默认 `false` 时：

- ${}^{c_0}_{i}\mathbf T$ 的旋转和平移参与优化；
- 所有 ${}^{c_j}_{c_{j-1}}\mathbf T$ 取自输入 camchain 并固定；
- 相机内参和畸变固定。

设置为 `true` 时，只额外放开 cam1 及后续相邻 baseline 的旋转和平移。它不会：

- 重新估计相机内参或畸变；
- 重新运行 `camera_calibration` 的 baseline 初始化；
- 自动增加 stereo 同步约束；
- 保证弱激励数据下 baseline 可观。

ETHZ 原生 help 把该选项定位为排查相机链外参问题的 debugging 功能。若相机 camchain
已经由高质量双目标定得到，默认固定通常更稳定；若打开，Camera–IMU 轨迹、IMU 外参
和平移 baseline 之间会增加相关性，必须检查结果是否偏离物理尺寸。

#### `recover_covariance`

启用后，最终 LM 完成并释放 optimizer，再通过 `IncrementalEstimator` 强制加入完整
problem，恢复 `CALIBRATION_GROUP_ID` 的协方差标准差。当前实现实际覆盖：

1. ${}^{c_0}_{i}\mathbf T$ 的 6 个最小参数；
2. 启用时间标定时，每个相机的时间偏移标量。

它**不等于全状态 covariance**。pose spline、bias spline、gravity、IMU intrinsic、
非参考 IMU 外参，以及 `recompute_camera_chain_extrinsics` 放开的相机 baseline 都位于
helper group，不会出现在当前打印的标定 covariance 中。恢复过程会再次构造/分解大型
线性系统，可能显著增加尾部耗时和峰值内存。

### 4.4 显示和输出字段

| 字段 | 默认值 | 作用 | 性能注意事项 |
|---|---:|---|---|
| `verbose` | `false` | Debug 日志，并自动打开 extraction 显示 | 关闭多进程检测，需要 GUI，I/O 很多 |
| `show_extraction` | `false` | 显示角点检测和重投影视图 | 关闭多进程检测 |
| `extraction_stepping` | `false` | 每帧等待用户单步确认 | 关闭多进程检测，不适合 benchmark |
| `export_poses` | `false` | 输出优化轨迹到 `poses.csv` | 只增加结果序列化 |
| `interactive_report` | `false` | 标定后打开报告窗口 | `false` 仍生成 `report.pdf` |

## 5. `execution`：性能参数，不是目标函数超参数

### 5.1 全部字段

| 字段 | 普通运行默认 | 已强制范围 | 影响范围 |
|---|---:|---|---|
| `parallelism` | 无 | 正整数 | 同时为 detector 和 optimizer 提供公共并行数 |
| `detector_processes` | 保留各阶段 ETHZ 默认 | 正整数 | 图像读取/反序列化/解码/标定板检测 worker 进程数 |
| `optimizer_threads` | 保留各阶段 ETHZ 默认 | 正整数 | 覆盖所有已接入 Optimizer2 和 IncrementalEstimator 阶段的 `nThreads` |
| `detector_inflight_per_worker` | `2` | 正整数 | 有界队列最多允许 `进程数 × 本字段` 个在途任务 |
| `detector_opencv_threads` | `1` | 正整数 | 每个 detector worker 内部 OpenCV 线程数 |
| `profiling_memory_sample_interval_s` | `0.25` s | 有限正数 | 进程树 RSS/PSS 采样周期；只影响测量精度和少量开销 |

普通 `calibrate` 不指定 detector/optimizer 时，不会自动注入 4/4，而是保留原生各阶段
默认值。`benchmark run` 为了形成可比较归档，会把缺失值固定为标准执行配置 4/4、
inflight 2、OpenCV 1、memory interval 0.25 s。

### 5.2 优先级

detector 和 optimizer 分别按下列顺序取第一个非空值：

```text
组件专用 CLI
> CLI --parallelism
> task 中组件专用字段
> task execution.parallelism
> 原生阶段默认
```

例如：

```yaml
execution:
  parallelism: 4
  optimizer_threads: 8
```

再执行：

```bash
kalibr-noros calibrate cameras --config task.yaml --output-dir output \
  --detector-processes 2
```

最终为 detector 2、optimizer 8。

### 5.3 并行参数实际影响哪些阶段

`detector_processes` 控制每个相机依次进入检测阶段时的 worker 数。多相机之间目前不是
同时建立两个独立进程池；每个相机内部的图像读取、ROS 反序列化或目录文件读取、
OpenCV 解码和 target detection 才是并行流水线。

`optimizer_threads` 会覆盖：

- 相机内参初始化 LM；
- 双目/多相机 baseline 初始化 LM；
- 相机 full-batch refinement LM；
- 相机最终增量 `addBatch`/GN；
- Camera–IMU rotation/gyro bias 初值；
- 多 IMU rotation 初值；
- Camera–IMU 最终联合 LM；
- covariance 恢复使用的增量估计器。

但它不会把以下主进程 Python 阶段自动并行化：时间偏移互相关、相机连接图构建、
target pose/PnP 初值循环、pose/bias spline 初始化、误差项 Python 构建、统计和报告
生成。底层 `BlockCholeskyLinearSystemSolver` 的分解本身也没有多线程支持；线程主要
用于残差、Jacobian/Hessian 构建和误差求值。因此从 4 增加到 8 不保证线性加速。

`detector_processes × detector_opencv_threads` 是检测阶段的潜在线程预算。默认每个
worker 只给 OpenCV 1 线程，避免“多进程再套 OpenCV 多线程”造成过度订阅和内存上升。

### 5.4 task 与 `calibrate` CLI 如何分工

当前正式运行命令是：

```bash
kalibr-noros calibrate cameras \
  --config camera_calibration_task.yaml \
  --output-dir output

kalibr-noros calibrate imu-camera \
  --config camera_imu_calibration_task.yaml \
  --output-dir output
```

数据选择和算法超参数以 task YAML 为唯一正式入口；`calibrate` CLI 不提供
`--max-iter`、`--approx-sync` 或 `--recompute-camera-chain-extrinsics` 等算法覆盖参数。
CLI 只覆盖易变的执行资源参数，并额外控制：

| CLI 字段 | 作用 |
|---|---|
| `--output-dir` | 输出目录，必填；相对路径以当前工作目录为基准 |
| `--force` | 允许替换输出目录内由程序管理的既有结果文件 |
| `--initialization` | 字段级覆盖 task 的 seed 路径；相对路径以当前工作目录为基准 |
| `--initialization-strategy` | 字段级覆盖 task 策略，只能为 `refine` 或 `direct` |
| `--parallelism` 及组件并行参数 | 按第 5.2 节优先级覆盖 task `execution` |
| `--timing-json` | 仅 `project-profile` 构建可用；输出结构化阶段计时 |

`kalibr-noros convert job` 只是从旧式参数快速生成 task 的迁移工具，它只暴露常用字段，
不能用它的 help 判断 task 的完整能力。

## 6. 当前固定、没有通过 task 暴露的内部超参数

### 6.1 相机标定

| 阶段 | 固定设置 |
|---|---|
| 单相机内参/畸变初始化 LM | `DeltaX=1e-3`、`DeltaJ=1`、最多 200 次、LM lambda 10、原生 4 线程 |
| 相机对 baseline 初始化 LM | `DeltaX=1e-3`、`DeltaJ=1`、最多 200 次、LM lambda 10、原生 4 线程 |
| 全相机 full-batch refinement LM | `DeltaX=1e-3`、`DeltaJ=1`、最多 250 次、LM lambda 10、原生 4 线程 |
| 最终增量估计 | 每个 batch 最多 50 次；column scaling 开；`epsSVD=1e-6`；原生 `CPU-1` 线程 |
| 显式异常角点阈值 | 固定为各坐标分量 $4\sigma$ |
| Blake–Zisserman | 二维，内部默认 `pCut=0.999`、`wCut=1e-6` |

task 当前没有暴露原生相机 CLI 的 `--plot` 和 `--plot-outliers`。这是有意避免普通
task 混入高开销交互绘图，不代表底层入口删除了这些选项。

### 6.2 Camera–IMU 标定

| 项目 | 当前固定值 |
|---|---:|
| pose spline order | `6` |
| pose knots per second | `100` |
| bias knots per second | `50` |
| pose motion regularization | 关闭 |
| bias motion regularization | 开启，权重来自 IMU random walk |
| camera Blake–Zisserman | 关闭，`-1` |
| accelerometer Huber | 关闭，`-1` |
| gyroscope Huber | 关闭，`-1` |
| accel/gyro noise scale | `1.0` |
| rotation/gyro-bias 初值优化 | `DeltaX=1e-4`、`DeltaJ=1`、最多 50 次、原生 2 线程 |
| 最终联合 LM | `DeltaX=1e-5`、`DeltaJ=1e-2`、lambda 10、Block Cholesky、原生 `CPU-1` 线程 |

task 当前没有暴露底层 `--profile-optimizer`，也没有暴露 spline 阶数、knots/s、Huber
宽度、IMU noise scale、LM lambda、停止阈值或线性求解器选择。性能分析应使用
`project-profile` 构建和 `--timing-json`，而不是在 task 中写未知字段。

## 7. 参数校验现状与使用风险

当前实现对 task 顶层字段和 `execution` 字段执行严格白名单检查；但是
`calibration`、`dataset`、`target`、`cameras[]` 和 `imus[]` 尚未全部实行嵌套字段
白名单与统一类型检查。这带来两个重要结果：

1. `calibration` 中拼错的未知字段可能被静默忽略；
2. 部分数值虽然能通过 YAML 和 argparse，仍可能在更深的矩阵运算中失败。

`initialization` 是例外：task 初值块和独立 seed YAML 都执行嵌套严格白名单、有限数、
向量维数、齐次变换与模型门控校验。相机或 IMU 物理值是否足够接近真实设备，仍需要
由最终 residual、`calibration.yaml` 与 seed 的人工差值，以及 `observability.yaml`
判断；`initialization_report.yaml` 本身不直接计算物理参数变化量。

因此现阶段应只使用本文列出的字段，并坚持：

- 布尔字段写 YAML 原生 `true`/`false`，不要写字符串 `"false"`；
- 秒、像素 sigma、频率等物理量使用有限正数；
- `max_iterations` 和所有并行数使用正整数；
- benchmark 归档必须保存 effective task，避免把“省略默认值”和“显式固定值”混为
  一组；
- 修改任何会改变视图集合、残差权重或 active design variable 的字段后，都必须重新
  做数值一致性比较。

## 8. 哪些字段会直接破坏与冻结基线的可比性

下列字段改变数据、状态量、残差或增量加入路径，不能与原有结果只做耗时比较：

```text
dataset.time_range_s
dataset.frequency_hz
cameras[].model
target 物理尺寸
window_half_size_px
max_displacement_px
focal_initialization_min_visible_corner_ratio
synchronization_tolerance_s
information_gain_tolerance
shuffle
remove_outliers
final_filtering
blake_zisserman
imus[].model
max_iterations
time_offset_padding_s
reprojection_sigma_px
synchronize_clocks
estimate_multi_imu_delay
calibrate_time_offset
recompute_camera_chain_extrinsics
initialization.path 中的任一物理 seed
initialization.strategy
```

`recover_covariance` 不应改变最终最优点，但会增加后处理求解、耗时和内存；只比较
solver 主结果时可以分开统计。`detector_processes` 和 `optimizer_threads` 保持数学
问题不变，但不同并行归约顺序可能带来浮点末位差异，仍应使用既定 atol/rtol 验证。

## 9. 源码对应位置

本报告的主要事实可从以下文件复核：

- task 校验、字段到 CLI 的映射和执行优先级：
  `src/python/kalibr_no_ros/task.py`；
- 两类公共初值 YAML 的严格校验、相机模型维数和 IMU 模型门控：
  `src/python/kalibr_no_ros/initialization.py`；
- 最终标定块可观性报告、机器 epsilon hard rank 失败门和
  `epsSVD=1e-6` operational 弱可观警告：
  `src/python/kalibr_no_ros/observability.py`；
- 公共 CLI 和 task 生成器：`src/python/kalibr_no_ros/cli.py`；
- 相机标定参数、增量流程和 outlier 逻辑：
  `src/kalibr/calibration/kalibr/python/kalibr_calibrate_cameras`；
- 相机初始化各阶段固定 Optimizer2 设置：
  `src/kalibr/calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py`；
- 信息增益 batch 接受条件：
  `src/kalibr/calibration/incremental_calibration/src/core/IncrementalEstimator.cpp`；
- Camera–IMU CLI 与固定 spline/鲁棒核参数：
  `src/kalibr/calibration/kalibr/python/kalibr_calibrate_imu_camera`；
- Camera–IMU active design variable、时间偏移、IMU 模型和残差：
  `src/kalibr/calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py`；
- Camera–IMU 最终 LM 与 covariance 范围：
  `src/kalibr/calibration/kalibr/python/kalibr_imu_camera_calibration/IccCalibrator.py`；
- 并行覆盖和 profiling：`src/python/kalibr_native_optimizer/runtime.py`；
- 多进程图像读取、解码、检测和有界队列：
  `src/kalibr/calibration/kalibr/python/kalibr_common/TargetExtractor.py`。
