# 数据输入、配置依赖与输出说明

## 1. 目录相机如何选择

目录任务只填写数据集根路径和相机 ID，不需要 topic，也不重复填写相机目录：

```yaml
dataset:
  type: directory
  path: ../data/stereo_imu_YYMMDD_hhmm
cameras:
  - id: cam0
    model: pinhole-equi
  - id: cam1
    model: pinhole-equi
```

根目录的 `dataset.yaml` 完成 ID 到路径的映射。例如：

```yaml
cameras:
  - id: cam0
    timestamps: cameras/cam0/timestamps.csv
    images: cameras/cam0
  - id: cam1
    timestamps: cameras/cam1/timestamps.csv
    images: cameras/cam1
imus:
  - id: imu0
    data: imu/imu0.csv
```

这些是完整文件的片段；根 manifest 还必须声明字符串版本 `1.0.0`、类型
`kalibr_directory_dataset` 和稳定的 `dataset_id`。目录名可以移动或改名，ID 不随之改变。
task 中相对路径以 task 所在目录为基准；manifest 路径以数据集根目录为基准；
相机 CSV 的 filename 则相对该相机 images 目录。相机编号仍按任务顺序为 cam0、cam1，
IMU 同理。ID 不存在、类型不对或显式 topic 与 ID 冲突时直接报错。

bag 输入没有这份目录映射，因此任务仍须填写相机 topic，IMU 噪声 YAML 仍须填写
rostopic。目录 manifest 可以保留可选 topic 元数据；新采集将原话题仅保留在 meta/
中，用于追溯和导出 bag。目录任务及噪声 YAML 推荐省略话题。

## 2. 四份 YAML 是否足够

没有显式初始化时，以下四份文件足够描述一次“双目 → Camera–IMU”的任务配置：

| 配置 | 用途 | 何时需要 |
|---|---|---|
| stereo_camera_calibration_task.yaml | 数据、相机模型、求解与输出选项 | 双目内外参标定 |
| aprilgrid.yaml | 标签行列、尺寸、间距、起始 ID | 两阶段共同使用 |
| camera_imu_calibration_task.yaml | 数据、相机结果引用、IMU 模型和输出选项 | Camera–IMU 标定 |
| imu.yaml | 采样率、噪声密度、随机游走 | Camera–IMU 标定 |

此外必须有实际数据。目录输入需要 dataset.yaml、图像与时间戳 CSV、IMU CSV；
bag 输入需要实际 bag，ROS2 bag 需保留整个目录。数据不能由这四份配置替代。

先运行双目任务，成功生成 `camera_calibration_cam0_cam1.yaml`；再将 Camera–IMU task 的
`camera_calibration.path` 指向该文件，然后运行第二阶段。相机结果是必需的上一阶段
产物，不是可选 initialization。省略 initialization 时使用原生自动初始化；
`calibrated` IMU 仍会估计 bias、相机–IMU 外参和默认开启的时间偏移。

可移植模板见[配置示例](../config/examples/v1.0.0/README_ZH.md)，默认使用
pinhole-equi 和 calibrated IMU；根据硬件与镜头选择模型。输出路径由 CLI 的
`--output-dir` 指定。

## 3. 输入数据目录

```text
dataset.yaml                  公共数据入口：身份、传感器、路径、尺寸、单位
cameras/cam0/timestamps.csv    cam0 时间戳与文件名的对应表
cameras/cam0/*.bmp             cam0 原始灰度图像（也可为 JPG/PNG）
cameras/cam1/timestamps.csv    cam1 时间戳与文件名的对应表
cameras/cam1/*.bmp             cam1 原始灰度图像
imu/imu0.csv                  原始角速度 rad/s、加速度 m/s²，时间戳为 ns
meta/capture_dataset.yaml     采集设备、接口、左右目、编码、原话题等
meta/session.json             实际采集配置、创建时间、统计、设备信息
meta/pairs.csv                采集时保存的左右图像配对与源序号
meta/imu_full.jsonl            原始完整 IMU 消息，含姿态、协方差和原 frame ID
meta/validation.json          采集端结构与时序校验，含质量警告
```

Kalibr 的目录读取、`validate` 和标定前输入校验只读取公共 `dataset.yaml` 与其
引用的数据，不读取或检查 `meta/`。`meta/` 缺失、采集报告中的警告或其中的文件损坏，
均不影响标定端的输入校验；公共 manifest、图像和 CSV 本身仍须满足格式、分辨率、
纳秒时间戳及 IMU 时间覆盖要求。

采集端自己的校验、追溯和 bag 导出仍依赖 `meta/`，因此完整采集交付保留这些文件。
仅提供标定输入时可以不包含 `meta/`。纯双目采集可以没有 IMU 文件。
`meta/imu_full.jsonl` 与 `imu0.csv` 不是两份不同的六轴数据，前者保留完整消息供导出。

新采集目录命名为 `<前缀>_YYMMDD_hhmm`。读取以 manifest 的身份和路径为准，
移动数据集不改变采集来源；迁移已有目录时保留原始采集元数据。

## 4. 命名与精简交付

多个任务可以使用同一个 `--output-dir`。默认文件名由任务类型与 task 中按顺序排列的
真实传感器 ID 组成；右目单目标定可以只写 `cameras: [{id: cam1, model: pinhole-equi}]`。

当前按设备目录组织输入与输出，结构如下；具体中间文件可继续细化：

```text
<设备目录>/
  stereo_YYMMDD_hhmm/             双目输入数据
  stereo_imu_YYMMDD_hhmm/         双目＋IMU 输入数据
  result/
    <任务名称>.yaml               最终标定参数
    <任务名称>.report.html        HTML 报告
    <任务名称>.report.pdf         PDF 报告
    <任务名称>/                  该任务启用保存的中间结果
      observations/
      visualizations/
      metrics.json
      ...
```

CLI 显式传入 `--output-dir <设备目录>/result`，两类任务使用同一个结果根目录。
启用的观测、可视化、指标及诊断直接归入任务同名子目录，无需在工程内另存一份
`result/<设备名>/`。目录建立和文件映射由 `delivery.py` 负责；`config/`、`execution/`
等额外归档若由外层流程保存，也归入对应任务子目录，不代表 CLI 默认都会生成。

`--output-dir` 当前表示最终结果目录，不会自动附加 `result`；设备根目录包含输入数据，
应传其下的 `result/`，以满足输入与输出目录的隔离检查。

| 任务 | 结果文件 |
|---|---|
| cam0 单目 | camera_calibration_cam0.yaml |
| cam1 单目 | camera_calibration_cam1.yaml |
| cam0 + cam1 双目 | camera_calibration_cam0_cam1.yaml |
| cam0 + cam1 + cam2 多目 | camera_calibration_cam0_cam1_cam2.yaml |
| 双目 + imu0 | camera_imu_calibration_cam0_cam1_imu0.yaml |

默认只交付 `<名称>.yaml`、`<名称>.report.html` 和 `<名称>.report.pdf`。可以用
`output.name` 显式覆盖名称，例如不同模型使用 `stereo_equi`、`stereo_radtan8`；只允许
字母、数字、下划线和连字符，最多 160 个字符。相同名称拒绝覆盖，只有显式 `--force`
才替换该任务的文件，其他任务和未登记的用户文件不受影响。

每目结果以标量 `rms: 0.24955518585492681` 表示最终保留角点的二维重投影 RMS，单位 px。
相邻外参旁的 `alignment: 0.22041488139290424` 为双目校正后非视差方向的像素误差 RMS。
缺少可用证据时写 null，不写为 0；数量、状态和误差分布保留在可选 metrics.json 中。
结果不再输出 `transform_convention` 文本；方向仍为点从源相机变换到目标相机：
相机列表为 `[cam0, cam1]` 时，cam1 条目的 `T_cn_cnm1` 表示 cam0 → cam1；不再输出重复的 from_camera 字段。

Camera–IMU task 省略 `camera_calibration` 时，先在 `--output-dir` 根目录寻找有效的
双目/多目相机结果。目录数据集必须匹配 manifest 中的相机 ID；零个或多个匹配均报错，
多个结果时请显式填写 `camera_calibration.path`。显式路径优先，不自动替代错误的路径。

报告采用分组表格、分页图形，包含 Camera system、Estimated poses、Polar error、
Azimuthal error 与 Reprojection errors。图形使用已有最终观测，不重跑求解；缺证据明确
标为不可用。上述统计图内嵌 HTML/PDF；启用可视化时两种报告使用相同的随机选取的最多
5 组图，随机选取后按原始帧顺序排列。每组上方为左右双目原图，下方为同一对图像的
极线对齐结果；不引用角点检测图或单目去畸变图。抽样使用固定随机种子以便复查，
极线辅助线为绿色、3 像素宽。PDF 每组单独一页，上原图、下对齐图，图像内嵌且可放大查看，
分享 PDF 无需附加图片。HTML 分享这些引用图时需一并保留对应的可视化子目录。

以下文件按需保存到 `<名称>/` 子目录。`output.save_diagnostics: true` 保存日志、
输入校验、可观性、初值报告、判定、运行清单及 task 快照，默认 false；
`output.save_metrics: true` 单独保留 metrics.json 供离线摘要评价，默认 false。
`output.export_opencv` 默认 false；观测归档和原图可视化默认 false，各自独立配置。
`output.export_text: true` 额外交付 `<名称>.results.txt`。内部求解始终执行所需输入校验
和可观性检查，这些开关只控制保存文件。失败时默认返回错误；启用诊断时保存到
`<名称>_failed/`，不会覆盖已有成功结果。

离线评价示例：

```bash
kalibr-noros evaluate --run output/camera_calibration_cam0_cam1.yaml --output-dir evaluated
```

启用 `archive_observations` 后可以重算指标；仅启用 `save_metrics` 时只能重做摘要判定，
不能改变角点配对或校正参数。两者都未保存时明确提示缺少证据。证据子目录中的
`.inventory.json` 仅用于检查安全覆盖；不要手工添加文件后强制覆盖整个子目录。

### 可选文件含义

| 文件 | 含义与使用场景 |
|---|---|
| <名称>.yaml | 最终数值参数；双目包含内参、畸变和相机间变换，Camera–IMU 另含 T_cam_imu、时间偏移和 IMU 参数；下游读取此文件 |
| results.txt | 人可读的标定结果和误差摘要 |
| metrics.json | 可量化的重投影、双目校正、IMU 残差、bias 及可用优化状态；包含总体和逐帧统计 |
| assessment.json | 根据显式规则判定 pass/fail/not_evaluated；参考评级独立保存，不能等同生产验收 |
| report.html | 浏览器查看的汇总、指标及图像链接；移动时保留相邻文件夹 |
| report.pdf | 可分享的 PDF 结果报告 |
| validation.json | 本次标定前的输入结构、分辨率和时间覆盖校验；不是标定精度验收 |
| task_resolved.yaml | 本次任务快照，含解析后的路径和执行配置 |
| run_manifest.json | 运行状态、软件版本、输入文档哈希和受管输出清单，用于追溯与安全覆盖 |
| stdout.log / stderr.log | 内部求解、检测工作进程的标准输出和错误日志；输入早期失败可能没有 |
| initialization_report.yaml | 使用初值时的初值来源、策略与初始化诊断 |
| observability.yaml | 启用相应分析时的秩和可观性诊断；不可用不能写成通过 |
| poses.csv | 显式请求时导出的标定板位姿序列 |
| timing.json | profile 构建且显式请求时的阶段计时 |

## 5. observations/：可重算的观测证据

通过 `output.archive_observations: true` 开启，默认关闭。CSV.gz 是 gzip 压缩的
CSV，每个单元格按 JSON 编码，保留整数纳秒、数组、布尔值和 null。

| 文件 | 每行／内容的含义 |
|---|---|
| manifest.json | 观测归档版本、数据来源、相机参数、总体状态、各表行数和 SHA-256；加载时校验 |
| frames.csv.gz | 相机帧记录：相机 ID、唯一帧 ID、原始序号与时间戳、源文件、检测状态、是否最终使用、板位姿；用 used 区分最终有效帧 |
| corners.csv.gz | 角点记录：所属帧、全局角点 ID、板坐标、检测像素、预测像素、残差和 used；用于 RMS、极线误差与叠加图 |
| views.csv.gz | 原生相机增量标定的 target view，关联同时观察标定板的相机和帧；Camera–IMU 中可能只有表头 |
| pairs.csv.gz | 可用双目帧的对应关系；相机标定来源于原生 view，Camera–IMU 为诊断配对，不能混为同一种算法观测 |
| selection_events.csv.gz | 开启 archive_selection_history 后记录增量接受、拒绝、异常点过滤等事件；Camera–IMU 不运行该增量选择，可能为空表 |
| imu_residuals.csv.gz | 最终保留的 gyro/accel 误差项，含时间来源、物理残差、白化残差及可用权重 |
| imu_biases.csv.gz | 存在 bias 数据时保存优化后 bias spline 的三轴采样时序；不是一个固定 bias 常量 |

不保存选择历史时，只保存最终有效角点；开启历史可保留更多筛选证据。是否选择帧、
是否最终使用角点，应读取表中的状态字段，不能仅凭“文件存在”判断。空表表示该流程
没有对应证据，不表示误差为零。不能独立修改这些表，否则校验和不再匹配。

## 6. visualizations/、images/ 与 opencv/

`output.visualizations.enabled: true` 才生成图像诊断，默认关闭：

| 路径 | 含义 |
|---|---|
| visualizations/cam0/corners_N.jpg | cam0 第 N 个源索引对应的角点叠加图：绿色圆为使用角点，红色圆为保留在历史中的未使用角点，蓝色线段连接测量点和预测点，表示像素残差 |
| visualizations/cam1/corners_N.jpg | cam1 对应的角点与残差图 |
| visualizations/cam0_cam1/rectified_NNNN.jpg | 去畸变并双目校正后的左右拼接图；绿色、3 像素宽参考线帮助目视检查极线对齐，同一空间点应落在相同极线上 |
| images/camX/N.png | copy_used_images 显式开启时复制的最终使用图像，便于离线移交；不等于上述带叠加的 JPG |
| opencv/cam0.yaml、cam1.yaml | OpenCV FileStorage 可读的相机矩阵 K、畸变 D 和模型信息 |
| opencv/cam0_cam1.yaml | 双目 R/T、R1/R2、P1/P2、Q 等；由 export_opencv 控制 |

九参数 `pinhole-opencv-fisheye` 的导出 K 保留 `K[0,1] = fu * alpha`。
极线指标和校正图按同一 alpha 处理角点与原图，非零 skew 不会被丢弃；
该语义同时适用于相机和 Camera–IMU 结果。

radtan 系列的角点反畸变采用最多 100 次迭代、收敛容差 1e-12，避免 OpenCV
默认五次迭代在强 rational 畸变的边缘点上留下反解误差而污染极线指标。
这只影响输出评价，不改变标定优化或图像重映射的正向模型。

默认展示每相机最多 30 帧、最多 30 对，均匀抽样。展示数量不限制标定或指标的计算
总体，文件名索引也不是“第 N 张入选图”。Camera–IMU 下图像 RMS 还受连续时间
位姿拟合约束影响，不必等于第一阶段相机标定的 RMS。

离线 `evaluate` 只从已有证据重算指标和展示，不重跑检测或优化。绘图需要原图或
images/ 副本；仅算已有残差指标时不需要原图。

## 7. 采集与标定交付边界

采集端交付公共 `dataset.yaml`、图像、时间戳 CSV 和 IMU CSV。两端共同使用
v1.0.0 的传感器 ID、纳秒时间戳与物理单位约定。采集配置、SDK、原始话题和完整
消息保存在 meta/ 中，标定求解不依赖采集端程序或 ROS 运行时。

目录任务按 ID 选择，不必重复设备话题；需要导出 bag 时，采集端可从 meta/
恢复原始话题与消息信息。标定板物理尺寸和 IMU 噪声参数由对应 YAML 提供，
不能从数据目录名猜测。
