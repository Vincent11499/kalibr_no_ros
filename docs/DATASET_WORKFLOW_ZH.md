# 数据输入、配置依赖与输出说明

## 1. 目录相机如何选择

目录任务只填写数据集根路径和相机 ID，不需要 topic，也不重复填写相机目录：

```yaml
dataset:
  type: directory
  path: /mnt/q/File/machine_data/evt3.1-data/directory_dataset
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

先运行双目任务，成功生成 `stereo/calibration.yaml`；再将 Camera–IMU task 的
`camera_calibration.path` 指向该文件，然后运行第二阶段。相机结果是必需的上一阶段
产物，不是可选 initialization。省略 initialization 时使用原生自动初始化；
`calibrated` IMU 仍会估计 bias、相机–IMU 外参和默认开启的时间偏移。

当前机器的可运行配置位于 `config/directory_dataset/` 和 `config/euroc/`，
机器专用路径配置保留本地。可移植模板见 [配置示例](../config/examples/v1.0.0/README_ZH.md)。
前者使用 pinhole-equi，后者使用 pinhole-radtan；
两者均使用 calibrated IMU。输出路径由 CLI 的 `--output-dir` 指定。

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

Kalibr 的目录读取只依赖公共 manifest 与其引用的数据；采集端校验、追溯和 bag
导出还依赖 meta/，因此完整交付保留这些文件。纯双目采集可以没有 IMU 文件。
meta/imu_full.jsonl 与 imu0.csv 不是两份不同的六轴数据，前者保留完整消息供导出。

新采集目录命名为 `<前缀>_YYMMDD_hhmm`。当前用户指定的 directory_dataset 保持原
文件夹位置；这不影响协议读取。对该已有目录的授权迁移另记在 meta/ 下，不能冒充
由新版本软件重新采集，也不能抹掉原有采集警告。

## 4. 输出根目录文件

| 文件 | 含义与使用场景 |
|---|---|
| calibration.yaml | 最终数值参数；双目包含内参、畸变和相机间变换，Camera–IMU 另含 T_cam_imu、时间偏移和 IMU 参数；下游读取此文件 |
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
| visualizations/cam0_cam1/rectified_NNNN.jpg | 去畸变并双目校正后的左右拼接图；红色参考线帮助目视检查极线对齐，同一空间点应落在相同极线上 |
| images/camX/N.png | copy_used_images 显式开启时复制的最终使用图像，便于离线移交；不等于上述带叠加的 JPG |
| opencv/cam0.yaml、cam1.yaml | OpenCV FileStorage 可读的相机矩阵 K、畸变 D 和模型信息 |
| opencv/cam0_cam1.yaml | 双目 R/T、R1/R2、P1/P2、Q 等；由 export_opencv 控制 |

默认展示每相机最多 30 帧、最多 30 对，均匀抽样。展示数量不限制标定或指标的计算
总体，文件名索引也不是“第 N 张入选图”。实际当前四次输出均为 60 张角点图和
30 张校正图。Camera–IMU 下图像 RMS 还受连续时间位姿拟合约束影响，不必等于第一
阶段相机标定的 RMS。

离线 `evaluate` 只从已有证据重算指标和展示，不重跑检测或优化。绘图需要原图或
images/ 副本；仅算已有残差指标时不需要原图。历史批次根目录的 verification/
是人工运行验收证据，不是求解器必须生成的目录；superseded 标记被后续配置替代的
历史运行，不能误当作当前结果。

## 7. 采集工程联动改动

工程位置为 `Q:\project\kalibr_cedarobo`。本轮目录 ID 改造修改了：

| 文件 | 本轮改动 |
|---|---|
| script/calib_capture/dataset.py | 新写出的公共 dataset.yaml 使用 ID 和路径，不重复 topic；内部采集元数据继续保留原话题 |
| script/calib_capture/validator.py | 接受无 topic 的公共清单；可选 topic 如填写则检查一致性；校验报告中的 manifest 版本修正为 1.0.0 |
| script/tests/test_dataset_contract.py | 核验新清单、传感器身份和自定义话题仍保留在采集元数据 |
| script/tests/test_kalibr_reader.py | 用真实 DirectoryReader 按 ID 读取 BMP/JPEG、时间戳和原始 IMU 数据 |
| AGENTS.md、doc/DATASET_FORMAT.md、doc/README.md、doc/使用操作说明.md | 同步必填字段和目录任务示例 |

此前 v1.0.0 联动已统一 config.py/cli.py 的版本、INI schema、目录前缀和
`_YYMMDD_hhmm` 命名；dataset.py/validator.py/ros1_export.py 的公共清单、会话、
IMU 消息及导出接口也已统一到新契约。当前 Git 工作区中还保留其他已有采集改动，
本轮没有改动 AXERA 取帧、同步、限频、线程池或原生标定求解代码。

本轮验证与原地迁移记录见
[目录 ID 契约验收](reports/DIRECTORY_ID_CONTRACT_VALIDATION_20260910_ZH.md)。
