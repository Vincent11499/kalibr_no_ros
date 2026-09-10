# v1.0.0 输入、求解与输出契约

本版本统一软件包、CLI、配置和项目结构化输出为字符串版本 `1.0.0`，并将输入验证、
原生求解和结果评价分开。新任务不兼容旧整数 schema，运行时不改写输入数据。
第三方 ROS bag、ROS 消息、OpenCV FileStorage
编码规则和 SDK ABI 不属于项目 schema 版本，保留其标准。

## 1. 公共文件与路径

| 文档 | 识别字段 | 用途 |
|---|---|---|
| 相机／Camera–IMU task | kind: calibration_task | 输入选择、求解参数、执行预算、输出选项 |
| dataset.yaml | type: kalibr_directory_dataset | dataset_id 与有身份的相机／IMU列表 |
| AprilGrid | kind: calibration_target | 标定板真实物理参数 |
| IMU 配置 | kind: imu_configuration | 话题、采样频率、噪声与随机游走 |
| 相机初值 | kind: camera_calibration_initialization | 单目或双目内参、畸变、baseline 初值 |
| Camera–IMU 初值 | kind: camera_imu_calibration_initialization | 安装外参、时间偏移、bias 等物理初值 |
| 离线评价 | kind: calibration_evaluation | 只改变报告、校正和判定规则 |
| <任务类型>_<传感器ID...>.yaml | kind: calibration_result | 最终物理参数，是下游权威输入 |
| metrics.json／assessment.json | calibration_metrics／calibration_assessment | 指标证据与单独判定 |

表内文档同时声明 `schema_version: "1.0.0"`。公共 task、模型、输入与输出选项拒绝
错误版本、未知字段和非法类型；禁止只改旧文件版本号冒充已满足新契约。

task 中的 dataset、target、IMU、相机结果和初始化路径，以 task YAML 所在目录为基准；
dataset manifest 内路径以数据集根为基准，相机 CSV 文件名再相对对应 images 目录。
CLI 的 `--output-dir`、初始化覆盖和重定位 `--dataset` 以执行命令时的当前目录为基准。
生成结果应写新目录，采集数据与标定输出分别管理。

完整可复制示例见 [config/examples/v1.0.0](../config/examples/v1.0.0/README_ZH.md)：
单目内参、双目内外参、双目＋IMU 各有最简和完整注释 task，并附板、IMU、
初值与评价示例。示例根目录的配置不带注释，带注释版本位于 `all_params/`；
数据集清单由采集工程生成，示例中不再重复提供。所有相机示例使用 `pinhole-equi`，内参 `[fu,fv,cu,cv]`、
畸变 `[k1,k2,k3,k4]`；它使用原生等距模型，不能和五项带 alpha 的
`pinhole-opencv-fisheye` 内参长度混用。

## 2. 采集数据与输入验证

采集命名统一为 `<dataset_name>_YYMMDD_hhmm`。前缀允许 1–64 个 ASCII 字母、数字、
下划线或连字符，首字符为字母或数字；默认双目 `stereo`、双目–IMU `stereo_imu`、
独立 IMU `imu`。日期是设备本地墙钟，只用于目录命名。同名前缀一分钟内冲突报错，
不追加秒数、不覆盖正式或 `.inprogress/.failed` 数据。session 另存 UTC 时间。

根 manifest 必须有 `schema_version/type/dataset_id/cameras/imus`；相机条目必需
`id/timestamps/images`，可带 `topic/frame_id/resolution/clock_source`；IMU 条目必需
`id/data`，可带 `topic/frame_id/clock_source/angular_velocity_unit/linear_acceleration_unit`。
根可选 `created_at/producer`。采集器写出完整身份、尺寸、时钟源和单位，不将相机内参
或求解超参数放入原始数据 manifest。

目录任务按传感器 `id` 关联路径，不需要填写相机 `topic` 或 IMU `rostopic`。
采集器将 ROS 话题保存在 meta/ 供 bag 导出使用；新生成的公共 manifest 不重复话题。
bag 标定仍需填写话题。文件依赖和输出含义见 [数据与输出说明](DATASET_WORKFLOW_ZH.md)。

规范同时固定以下语义：

- cam0 为左目，cam1 为右目；topic、frame ID 与设备身份分别保存，不能靠文件名猜轴向。
- 相机表头 `timestamp_ns,filename`；IMU 表头 `timestamp_ns,wx,wy,wz,ax,ay,az`。
  CSV 时间戳为 `[0,2^63-1]` 内严格递增整数纳秒，不用浮点秒，不静默重排或去重。
- 相机各自使用 `u64PTS×1000` ns，IMU 使用原 header stamp；保存原轴、原符号和原值，
  gyro 为 rad/s、accel 为 m/s²。`clock_source: axera_pts_us` 描述原始硬件源，CSV 仍为 ns。
- 图像使用原尺寸，默认无损 mono8 BMP，可选有损 JPEG；Kalibr 解码支持 PNG、JPEG/JPG、BMP。
  manifest 中尺寸必须与实际解码相符，不能仅修改尺寸字段替代重采集或正确的模型换算。
- 相机根 frame ID 描述相机坐标系，IMU 根 frame ID 描述会话有效坐标系。原始 IMU frame ID
  连同空值保留于采集 JSONL；空值回退产生警告，有效 frame ID 在会话内改变即失败。
- `meta/pairs.csv` 保留左右原始时间戳、序号和采集配对，不用平均时间覆盖原时间。
  双目采集默认同步容差 200 µs；Camera–IMU 默认保留图像窗前后各至少 1 s 的真实 IMU 覆盖。
- 默认不计算逐帧 SHA-256；结构检查不能保证发现所有等长度或仍可解码的像素修改。

`kalibr-noros validate --config task.yaml` 检查输入与任务契约、话题、尺寸、时间范围和
相机–IMU覆盖，不加载角点检测器或优化器。验证通过只证明输入结构与基本时序可用，
不能据此判定标定精度或可观性。现有 EVT3.1 目录包含多代旧格式，本版不会将其原地升级。

## 3. 求解边界与可选观测记录

camera 与 Camera–IMU 分别调用原生标定入口，保持阶段顺序、残差、
active 参数、优化器、超参数、停止与异常点逻辑。相机初始化、最终观测选择和残差评价
不是同一步骤；新增保存钩子只读取原生状态，不重新检测、替换图像或改变 view 顺序。

相机 `calibration.freeze_intrinsics` 默认 false；true 仅在至少双目、每台提供完整 seed
时跳过单相机内参 LM，并在后续阶段固定投影与畸变，baseline 与 target pose 保持 active。
Camera–IMU 读取既有相机结果，`recompute_camera_chain_extrinsics` 仅控制联合阶段是否
放开相邻 baseline，不执行完整相机标定。显式初值默认不启用，示例初值都是占位数据。

可重复示例明确 `shuffle: false`、检测 4 进程、优化 4 线程、每 worker 两个在途任务、
每 worker 一个 OpenCV 线程。相机同步示例为 0.0002 s，用于对齐采集；未配置时仍恢复
原生 0.02 s。二者可能形成不同配对，数值／性能比较必须保持显式配置一致。

`output.archive_observations` 默认 false；打开后保存最终实际使用的相机帧、view、
角点 ID、原始亚像素坐标、最终预测／残差、IMU 残差、bias spline 采样值及来源引用。
`output.archive_selection_history` 默认 false 且依赖前者，保存可用的接受／拒绝与
过滤事件。通过最终观测集合计算指标，不把检测到过但最终未使用的角点混入总体。
归档中的稳定来源索引与整数纳秒时间戳用于回溯图片，不能用格式化浮点时间进行关联。

显示和文件选项位于 `output`，包括 verbose、show_extraction、extraction_stepping、
interactive_report、export_poses；它们不属于 calibration 求解参数。
显式开启图像诊断和归档会增加 I/O 与内存，性能比较须记录并保持相同设置。

## 4. 指标、图像与判定

重投影统计使用最终有效角点的二维像素距离：

$$
\operatorname{RMS}_{2D}=\sqrt{\frac{1}{N}\sum_{i=1}^{N}(\Delta u_i^2+\Delta v_i^2)}.
$$

分母是角点数量，不是两个坐标维度的总数；它与某些旧工具逐坐标 RMSE 相差
`sqrt(2)`，不能混用阈值。报告提供按相机和每帧的统计及分布，单位明确为 px。
IMU 残差分别以 rad/s 和 m/s² 汇总，不把白化残差当物理单位误差。
归一化统计只使用原生误差项导出的白化向量，单位为无量纲；robust 权重和加权目标值
另行记录。无法取得归一化信息时标记 `unavailable`，不根据物理残差猜测噪声或权重。

所有指标使用最终实际参与求解并保留的全部有效观测。图像展示的 `max_frames_per_camera`
和 `max_pairs` 只限制展示数量，不抽样重投影、极线、IMU 残差或 bias 统计。相机统计中
`input_frames` 是源流总帧数，`selected_frames` 是经过 task 时间窗／频率选择后的帧数，
`used_frames` 是最终用于求解的帧数；源总数未知时明确留空。

IMU bias 位于 `metrics.json` 的 `imus.<imu_id>.<gyro|accel>.bias_spline`。这些值来自
优化后的真实随时间变化的 bias spline，在保留的 IMU 残差时刻求值；gyro bias 单位为
rad/s，accel bias 单位为 m/s²。`axes.x/y/z` 给出采样值的均值、标准差、最小／最大值
和 RMS，`vector_norm` 汇总三维模长。均值是这些时刻的等权样本统计，不代表常数 bias
估计，也不代表 spline 对时间积分后的平均值。

`bias_spline.time_series` 保留完整采样序列，包括原始整数 `timestamp_ns`、
`source_index`、求解器时间 `solver_timestamp_s` 和三轴 `value`；HTML／PDF 同时绘制
三轴随时间变化的曲线。`archive_observations: false` 时仍输出这些 bias 统计和时间序列，
不会因此保存逐角点或逐 IMU 残差的大表。无法读取某个 spline 值时保留 `value: null`
和原因；统计注明缺失数量，部分数据为 `incomplete`，不会补成零值或常数曲线。

双目极线指标只使用最终图对的共同全局角点 ID，基于模型一致的 OpenCV 校正计算。
`pinhole-equi` 使用 fisheye 校正，不能对带畸变原像素直接套普通 pinhole F 矩阵。
九参数 `pinhole-opencv-fisheye` 的导出 K 保留 alpha/skew；非零 alpha 在角点
校正和图像映射中显式处理，不会丢弃第五个内参。
报告给出校正后 y 差的均值、RMS、分布及有效样本数，域外或不可用点明确记录。

| output 参数 | 默认／范围 | 作用及比较边界 |
|---|---|---|
| archive_observations | false，布尔 | 保存可重算的最终观测；增加归档 I/O |
| archive_selection_history | false，布尔 | 保存选择历史，必须同时开观测归档 |
| copy_used_images | false，布尔 | 可选复制最终使用原图；数据量可很大 |
| export_opencv | false，布尔 | 显式导出相机／双目 OpenCV 参数，附模型与方向 |
| name | 自动，字符串 | 任务与传感器 ID 组成的文件名；可用安全名称区分不同模型 |
| save_diagnostics | false，布尔 | 保存日志、校验、可观性、初值报告、判定及追溯文件 |
| save_metrics | false，布尔 | 保存 metrics.json，支持离线摘要评价 |
| export_text | false，布尔 | 额外交付具名 results.txt |
| evaluation_pairing_tolerance_s | 0.0002 s，有限且 >=0 | 仅 Camera–IMU 最终观测后处理最近一对一配对，稳定来源索引打破平局 |
| visualizations.enabled | false，布尔 | 生成角点、残差与校正图 |
| visualizations.max_frames_per_camera | 30，正整数 | 每相机展示上限，不裁剪指标总体 |
| visualizations.max_pairs | 30，正整数 | 双目展示上限，不裁剪指标总体 |
| visualizations.sampling | uniform | 确定性均匀抽样 |
| rectification.balance | 0.0，无量纲，[0,1] | 校正视场设置；改变可影响像素极线误差 |
| rectification.fov_scale | 1.0，无量纲，>0 | fisheye 校正视场倍率 |
| rectification.size | null 或 [宽,高] 正整数 px | null 使用源尺寸；改变后不能直接混比像素误差 |
| assessment.reference_grading | true，布尔 | 独立参考显示等级，不作生产验收 |
| assessment.rules | []，规则列表 | 业务指定 metric、min/max、required；阈值单位随指标 |

正式判定与参考评级分开：无业务规则为 `not_evaluated`；已知规则失败为 `fail`；必要
指标缺失为 `incomplete`；必要规则均有证据且通过才为 `pass`。不得用零值或空统计代替
缺失证据自动放行。参考等级为 excellent/good/acceptable/poor 或 incomplete，仅显示
历史工具风格参考；示例门槛未经生产验证，默认业务规则为空。

优化器摘要必须连同 `scope` 阅读。相机增量路径的 `optimizer.scope: last_attempt`
描述最后一次候选求解，其 `JFinal` 可能对应后来被拒绝或回滚的候选；最终保留视图的
加权误差总和独立放在 `objective.final_camera_weighted_residual_sum`，其 scope 为
`final_retained_camera_views`。Camera–IMU 的 `optimizer.scope: final_joint_optimization`
描述最后联合问题，目标包含当次实际启用的 bias motion prior 等原生项；只把测量残差
相加不能代表完整联合目标。原生返回值没有明确提供的 `stop_reason` 和
`stopping_thresholds` 保持 `unavailable`。迭代次数、某个 `JFinal`、求解流程完成或
报告生成成功，都不能单独证明收敛或正式质量合格。

## 5. 生成文件和离线重评

默认交付 `<名称>.yaml`、`<名称>.report.pdf` 和 `<名称>.report.html`，名称由任务类型及
传感器 ID 组成，也可通过 `output.name` 指定。每目附标量 `rms`，相邻外参附
标量 `alignment`；两个数值均为 px 单位的 RMS，缺证据时为 null，不输出重复的 from_camera 字段。
结果不再写 transform_convention 文本。
诊断文件由 `output.save_diagnostics` 开启，指标 JSON 由 `output.save_metrics` 开启，
均默认 false；保存到 `<名称>/`，不会覆盖同目录其他任务。详细规范见
[输出说明](DATASET_WORKFLOW_ZH.md#4-命名与精简交付)。

标定命令在调用求解器前自动运行输入验证，启用诊断保存后才交付 `validation.json`；
不需要先手工执行一次 `validate`。输入验证失败会阻止
进入求解，不能把其结构检查通过状态当成 `assessment.json` 的质量判定。

进入内部求解阶段后，Python、原生 C++ 和检测 worker 的标准输出／错误输出分别写到
临时工作目录中的 `stdout.log`、`stderr.log`，启用诊断后保留到任务证据子目录。
求解异常时仅在诊断配置开启时保留日志；尚未进入求解的输入失败可能没有这两个日志。CLI 自身和后续报告
生成阶段的终端消息不属于这两个内部求解日志的完整采集范围。

OpenCV 导出放在 `opencv/`。观测归档包含 `observations/manifest.json` 和压缩 CSV
`frames.csv.gz`、`corners.csv.gz`、`views.csv.gz`、`pairs.csv.gz`、
`imu_residuals.csv.gz`；存在 IMU bias 数据时另存 `imu_biases.csv.gz`，保留完整 spline
采样时序及来源。各归档表的相对路径、行数和 SHA-256 均由 `observations/manifest.json`
登记，离线读取先校验。`selection_events.csv.gz` 仅在开启选择历史时保存。
图像展示在 `visualizations/`，仅显式复制时创建 `images/`。
`run_manifest.json` 描述阶段状态、可用产物和失败信息；求解成功、输出失败、
输入失败和正式质量不达标应分别理解，不能把生成报告成功等同于标定合格。

统一坐标约定为 `p_target = T_target_source * p_source`。`T_cam_imu` 为
${}^{C}_{I}\mathbf T$，`T_cn_cnm1` 为 ${}^{C_n}_{C_{n-1}}\mathbf T$；平移单位米。
结果 YAML 保持块映射、行内一维数组、矩阵逐行数组与 17 位有效数字，不用排版截断精度。

```bash
kalibr-noros evaluate --run output/stereo --config config/examples/v1.0.0/evaluation.yaml --output-dir output/stereo_evaluation
```

离线评价使用新的输出目录，原标定参数保持不变，不重新检测或求解。归档观测齐全时可
重算指标、改变校正设置和重绘；图像搬动后用 `--dataset` 重定位。只有汇总指标时仅可
按这些指标重新判定，不能补造逐角点证据。归档中未记录的历史事件也不能事后补造。
没有执行新真实数据标定或数值／性能比较时，应单独说明验证范围，不能用单元测试结果
替代数值一致或性能提升证据。
