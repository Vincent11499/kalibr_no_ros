# Kalibr no-ROS 使用与配置指南

本文只说明日常使用需要修改的内容：任务 YAML、目录数据集、相机模型和运行命令。
算法原理与源码调用链见
[`SOURCE_CODE_DEEP_DIVE_ZH.md`](SOURCE_CODE_DEEP_DIVE_ZH.md)，固定基准测试见
[`BENCHMARK_ZH.md`](BENCHMARK_ZH.md)。两类 task 的全部可配置字段、默认值、
有效范围和内部算法影响见
[`TASK_PARAMETERS_ZH.md`](TASK_PARAMETERS_ZH.md)。

## 1. 各类 YAML 不要混淆

| 文件 | 谁创建 | 是否需要经常修改 | 作用 |
|---|---|---:|---|
| `task.yaml` | 用户 | 是 | 选择任务、数据、target、模型和可选参数；接口版本为字符串 `"1.0.0"` |
| `dataset.yaml` | 用户，仅目录数据集需要 | 很少 | 把图像/IMU 文件映射为逻辑数据流 |
| `*_initialization.yaml` | 用户，仅显式初值需要 | 按硬件维护 | 保存相机或 Camera–IMU 的物理 seed；接口版本为 `"1.0.0"` |
| `calibration.yaml` | 程序 | 否 | 保存标定结果；结果接口版本为字符串 `"1.0.0"` |
| `initialization_report.yaml` / `observability.yaml` | 程序，仅显式初值运行生成 | 否 | 记录初值应用情况和最终标定块可观性 |

`config/` 提供三份可直接复制的简洁模板；字段合法性由 CLI 在运行前检查。详见
[`../config/README_ZH.md`](../config/README_ZH.md)。

## 2. 最简任务配置

任务必须用 `dataset.type` 明确写 `bag` 或 `directory`。选择 `bag` 后，程序会根据
输入自动区分 ROS1 文件和 ROS2 目录；普通运行可以省略 `execution`，通过 CLI 临时
指定并行数。

双目相机标定的最简 task：

```yaml
schema_version: "1.0.0"
job: camera_calibration
dataset: {type: bag, path: /data/camera.bag}
target: {path: aprilgrid.yaml}
cameras:
  - {topic: /cam0/image_raw, model: pinhole-radtan5}
  - {topic: /cam1/image_raw, model: pinhole-radtan5}
calibration: {shuffle: false}
```

双目 Camera–IMU 标定的最简 task：

```yaml
schema_version: "1.0.0"
job: camera_imu_calibration
dataset: {type: bag, path: /data/imu_camera_bag}
target: {path: aprilgrid.yaml}
camera_calibration: {path: camchain.yaml}
imus:
  - {path: imu.yaml, model: scale-misalignment}
```

task 文件名只表达任务类型，例如 `camera_calibration_task_demo.yaml`、
`camera_imu_calibration_task.yaml`；bag 和目录输入都使用同一命名方式，由
`dataset.type` 区分。

task 中的 `dataset.path`、`target.path`、`camera_calibration.path`、
`imus[].path` 和 `initialization.path` 都支持绝对路径；相对路径统一以 task YAML
所在目录为基准。
`dataset.yaml` 内部的图像/CSV 路径以数据集根目录为基准。CLI 的相对
`--output-dir` 和相对 `--initialization` 则以命令执行时的当前目录为基准。

运行时再给容易变化的执行参数：

```bash
kalibr-noros calibrate cameras --config task.yaml --output-dir output \
  --detector-processes 4 --optimizer-threads 4
```

需要长期固定或用于 benchmark 时，才把执行参数写回 task：

```yaml
execution:
  detector_processes: 4
  optimizer_threads: 4
```

完整 benchmark 执行配置会额外记录检测队列、OpenCV 线程数和内存采样周期；这些
字段有稳定默认值，不要求普通任务逐项重复。

### 2.1 可选的显式物理初值

相机和 Camera–IMU task 都可增加：

```yaml
initialization:
  path: camera_calibration_initialization.yaml
  strategy: refine
```

`strategy` 省略时为 `refine`。也可以不改 task，用 CLI 字段级覆盖：

```bash
kalibr-noros calibrate cameras --config task.yaml --output-dir output \
  --initialization camera_calibration_initialization.yaml \
  --initialization-strategy direct
```

- `refine`：seed 作为原生前置初始化优化的起点，允许只给部分参数；
- `direct`：可信 seed 直接进入后续阶段；相机标定要求所有相机内参、畸变和相邻
  baseline 完整；
- 两种策略都只是初始化，最终联合问题的原有 active 参数不会因此被固定，也不会
  增加“靠近 seed”的先验残差；
- 相机任务若需要固定已知内参和畸变、只重新估计多相机 baseline，可另外设置
  `calibration.freeze_intrinsics: true`；它要求每台相机有完整内参和畸变 seed，默认
  `false`，且仅对至少两台相机的任务有意义；
- 显式初值任务会先输出 `initialization_report.yaml`；到达最终可观性分析后再输出
  `observability.yaml`。基于机器 epsilon 的 hard rank gate 不通过时任务失败，只保留这两份
  诊断，不会用 seed 填出一个假成功结果；更早失败时可能只有初值报告。
  `epsSVD=1e-6` 只用于 operational 弱可观性警告，不会阻止 `calibration.yaml`
  输出。

两类初始化 YAML 的全部字段、坐标变换方向、单位、相机模型向量长度、IMU 模型门控
和分阶段行为见
[`INITIALIZATION_ZH.md`](INITIALIZATION_ZH.md)。
[`config/euroc/`](../config/euroc/README_ZH.md) 提供无初值配置；
[`examples/v1.0.0/`](../examples/v1.0.0/README_ZH.md) 提供 `pinhole-equi` 的单目、
双目和双目＋IMU 完整 task 及匹配初值模板。模板中的数值是教学占位值，默认未启用；
先换成当前硬件的可信 seed，再启用完整 task 中已注释的 `initialization` 块。
输入准备、验证和双目到 Camera–IMU 的运行顺序见初始化指南第 8 节。

## 3. 相机模型

公开的 `calibrate cameras` task 当前支持 10 种模型。下表只列最常用选择；全部模型的
参数顺序、单相机参数量和各阶段 active 状态见
[`CAMERA_MODELS_ZH.md`](CAMERA_MODELS_ZH.md)。

常用选择如下：

| 模型 | 畸变参数 | 适用范围 |
|---|---|---|
| `pinhole-radtan` | `[k1,k2,p1,p2]` | 普通镜头，等价于 OpenCV `k3=0` |
| `pinhole-radtan5` | `[k1,k2,p1,p2,k3]` | 普通到较大广角，参数顺序与 OpenCV 一致 |
| `pinhole-radtan8` | `[k1,k2,p1,p2,k3,k4,k5,k6]` | OpenCV rational model；径向变化复杂且数据覆盖充分的普通/广角镜头 |
| `pinhole-equi` / OpenCV fisheye | `[k1,k2,k3,k4]` | 更大视场或鱼眼镜头 |

`pinhole-radtan8` 对应 OpenCV `CALIB_RATIONAL_MODEL` 的前 8 个系数，不包含
thin-prism 的 $s_1\ldots s_4$ 或 tilted sensor 的 $\tau_x,\tau_y$。它比
`pinhole-radtan5` 多三个分母径向参数，表达能力更强，但也更容易出现参数相关和过拟合。
普通镜头应优先从 `radtan5` 开始；只有画面边缘有稳定系统残差、标定板覆盖中心到四角且
姿态/距离变化充分时，才建议使用 `radtan8`。

OpenCV fisheye 内部保留 zero-skew 和 full 两个 projection 类型：前者兼容旧四内参
YAML，后者使用 `[fu,fv,cu,cv,alpha]` 并可无损保存非零 skew，满足
`K[0,1] = fu * alpha`。两者共享同一个扩展模块、distortion 顺序和转换入口。

OpenCV stereo 导入支持 `K1/D1/K2/D2`、`R/T` 或 4×4 `RT`。平移单位必须显式
给出，转换器不会猜测毫米或米。变换统一采用：

```text
p_target = T_target_source * p_source
```

## 4. 目录数据集的作用与边界

目录数据集让 `kalibr-noros` 直接读取图像文件和 IMU CSV，不需要先制作 ROS1
或 ROS2 bag。它只扩展 I/O 边界：标定板检测、相机初始化、误差项、
Optimizer2、IncrementalEstimator、超参数和停止条件均未改变。

根目录必须包含 `dataset.yaml`。自动检测规则是：

| 输入 | 检测结果 |
|---|---|
| 普通文件 | `ros1` |
| 含 `metadata.yaml` 的目录 | `ros2` |
| 含 `dataset.yaml` 的目录 | `directory` |

一个目录同时包含 `metadata.yaml` 和 `dataset.yaml` 会被判定为歧义并拒绝，防止
程序静默选择错误后端。

## 5. 推荐目录结构

```text
dataset/
├── dataset.yaml
├── cameras/
│   ├── cam0/
│   │   ├── timestamps.csv
│   │   └── images/
│   └── cam1/
│       ├── timestamps.csv
│       └── images/
└── imu/
    └── imu0.csv
```

文件夹名称本身没有语义；实际映射由 `dataset.yaml` 声明。因此也可以采用其他
内部目录名，只要所有路径合法且指向对应文件。

## 6. `dataset.yaml`

```yaml
schema_version: "1.0.0"
type: kalibr_directory_dataset
dataset_id: example_dataset

cameras:
  - id: cam0
    timestamps: cameras/cam0/timestamps.csv
    images: cameras/cam0/images
  - id: cam1
    timestamps: cameras/cam1/timestamps.csv
    images: cameras/cam1/images

imus:
  - id: imu0
    data: imu/imu0.csv
```

字段含义：

| 字段 | 含义 |
|---|---|
| `schema_version` | 当前固定为字符串 `"1.0.0"` |
| `type` | 当前固定为 `kalibr_directory_dataset` |
| `dataset_id` | 必填的稳定数据集身份 |
| `cameras[].id`、`imus[].id` | 必填且互不重复的传感器 ID |
| `cameras[].topic` | 可选的原话题元数据；目录任务通常不填写 |
| `cameras[].timestamps` | 相机时间戳表，相对根目录 |
| `cameras[].images` | 图像目录，相对根目录 |
| `imus[].topic` | 可选的原话题元数据；目录任务通常不填写 |
| `imus[].data` | IMU 表，相对根目录 |

目录任务通过 `id` 选择上述条目，图像目录与 CSV 路径只在 manifest 中定义一次。
相机任务使用 `id/model`，IMU task 使用 `id/path/model`，对应噪声 YAML 可省略
`rostopic`。bag 任务仍要求相机 `topic` 和 IMU `rostopic`。

目录 manifest 的 `topic` 仅是可选元数据。若填写，必须非空且各流唯一；若目录任务
仍显式填写 `topic` 或噪声 YAML 填写 `rostopic`，它必须与该 ID 的数据流一致，
冲突时报错。省略时内部自动生成读取键，不需要用户配置 ROS。

`timestamps`、`images` 和 `data` 只允许相对路径。绝对路径、`..` 越出数据集根
目录以及解析后指向根目录之外的符号链接都会被拒绝。

## 7. 相机时间戳 CSV

每个相机使用独立时间戳表：

```csv
timestamp_ns,filename
1403636579763555584,000000.png
1403636579813555456,000001.jpg
1403636579863555584,000002.bmp
```

约束如下：

- 表头必须是 `timestamp_ns,filename`，不能缺列或增加未知列；
- `timestamp_ns` 是区间 $[0,2^{63}-1]$ 内的十进制整数纳秒；
- `filename` 相对于该相机的 `images` 目录；
- 文件必须存在，同一个相机表内不能重复引用同一文件；
- 新目录数据的时间戳必须严格递增；
- 重复或回退时间戳报错，不静默排序或清洗源数据。

支持的实际格式由构建时 OpenCV 的 `imdecode` 编解码器决定。项目测试至少覆盖
PNG、JPEG/JPG 和 BMP。读取彩色图时使用 OpenCV 的 BGR/BGRA 灰度转换；16 位
图像按 Kalibr 原有 8 位检测输入约定缩放为 8 位。

目录后端仍是惰性读取：建立索引时不加载像素。多进程检测时，父进程只把时间戳
和图像路径放入有界任务队列；每个 detector worker 独立打开文件、读取编码字节、
执行 `cv2.imdecode` 并检测标定板。因此 `--detector-processes` 同时控制文件读取、
图像解码和标定板检测的并行度，队列不再复制整张图像的编码字节。串行检测以及
直接调用底层同步读取接口时，仍由调用进程读取并解码。

## 8. IMU CSV

基础格式为：

```csv
timestamp_ns,wx,wy,wz,ax,ay,az
1403636579763555584,0.01,-0.02,0.03,0.1,0.2,9.81
```

可选温度列为：

```csv
timestamp_ns,wx,wy,wz,ax,ay,az,temperature_c
1403636579763555584,0.01,-0.02,0.03,0.1,0.2,9.81,41.5
```

单位与原生 Kalibr IMU 输入一致：

| 列 | 单位 |
|---|---|
| `timestamp_ns` | ns |
| `wx,wy,wz` | rad/s |
| `ax,ay,az` | m/s² |
| `temperature_c` | °C |

所有物理量必须是有限数，拒绝 `NaN` 和无穷值。新目录数据的时间戳必须严格递增，
重复或回退时间戳报错。`temperature_c` 当前保存在中立 `ImuRecord` 中，
为后续温漂模型预留；原生 Kalibr 标定算法目前不会把温度加入残差或状态量。

对于目录数据集，`record_timestamp_ns` 与 `header_timestamp_ns` 相等。因此启用
`--perform-synchronization` 时，记录时间与传感器时间之间没有可学习的差值；这
不会替代 camera–IMU 联合标定中的相机时间偏移参数。

## 9. 目录数据集的 task 配置

相机标定 task 的数据集段：

```yaml
schema_version: "1.0.0"
job: camera_calibration

dataset:
  type: directory
  path: /data/my_dataset

target:
  path: aprilgrid.yaml

cameras:
  - id: cam0
    model: pinhole-radtan5
  - id: cam1
    model: pinhole-radtan5
```

运行方式不变：

```bash
kalibr-noros calibrate cameras \
  --config camera-task.yaml \
  --output-dir output \
  --detector-processes 4 \
  --optimizer-threads 4
```

`dataset.type` 是必填字段：目录数据集写 `directory`，ROS1/ROS2 bag 都写 `bag`。
类型与实际输入不一致时立即报错，不需要把 ROS1、ROS2 或 directory 写入 task
文件名。

Camera–IMU task 同样将 `dataset.path` 指向目录。相机结果和 IMU task 中的传感器
ID 必须对应 manifest；相机结果的原话题不会决定目录路径。程序读取相机结果的
内外参，并在内部按当前目录 ID 生成读取键，不改写被引用的相机结果。

AprilGrid 的标签若不是从 0 开始，可在 `target.path` 指向的 target YAML 中增加：

```yaml
target_type: aprilgrid
tagRows: 6
tagCols: 6
tagSize: 0.088
tagSpacing: 0.3
tagStartId: 100
```

标签按先行后列连续编号，因此上例有效 ID 区间为 $[100,136)$。省略
`tagStartId` 时默认为 0，与旧配置兼容。当前仍固定使用 `tag36h11`，不支持离散
ID 表或其他 AprilTag family。

## 10. 目录数据集计时字段

为保持既有 profiling 报告字段兼容，目录后端沿用下列名称：

| 字段 | 目录后端实际含义 |
|---|---|
| `bag_read_*` | 从图像文件读取编码字节 |
| `deserialize_*` | 固定为 0；目录格式没有 ROS 消息反序列化 |
| `decode_*` | OpenCV `imdecode` 与灰度/位深转换 |

因此报告中的 `bag_read` 是跨后端兼容名称，不表示目录输入内部存在 rosbag。
