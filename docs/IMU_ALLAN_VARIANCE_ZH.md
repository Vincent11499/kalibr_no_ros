# IMU Allan 噪声分析

`kalibr-noros tools imu-allan` 读取静止 IMU CSV，生成本工程可直接读取的
`imu_noise.yaml` 及 HTML/PDF 报告。实现迁自参考脚本的 `input_format.py`、
`allan_core.py` 和 `upstream_analysis.py`，保留其固定
`ori-drs/allan_variance_ros@1d54b602ee7f2ba0427865d63afe4945d913ed24` 数值策略。
使用工程已有的 NumPy、SciPy、Matplotlib 和 PyYAML，不需要 ROS 或额外安装 AllanTools。

## 命令

```bash
kalibr-noros tools imu-allan \
  --dataset /data/imu_YYMMDD_hhmm \
  --output-dir /data/results/imu_allan
```

也可直接指定 CSV，并选择相对原文件首条样本的第 1–23 小时：

```bash
kalibr-noros tools imu-allan \
  --dataset /data/imu_YYMMDD_hhmm/imu/imu0.csv \
  --output-dir /data/results/imu_allan_1h_23h \
  --start-h 1 --end-h 23 --topic /imu/data
```

| 参数 | 默认值 | 范围、单位与作用 |
|---|---|---|
| `--dataset` | 无，必填 | 已有目录或 CSV 文件；目录输入读取其 `imu/imu0.csv`，直接 CSV 不限制文件名 |
| `--output-dir` | 无，必填 | 全新输出目录；必须位于源数据目录之外 |
| `--topic` | `/imu/data` | 非空且无空白字符的字符串，写入 YAML 的 `rostopic`；不参与计算 |
| `--start-s` | `0` | 相对首条源样本的非负有限秒数，含起点；精度须为整数纳秒或更粗 |
| `--start-h` | 未设置 | 同一起点的小时表示，与 `--start-s` 互斥 |
| `--end-s` | 原文件结束 | 相对首条源样本的非负有限秒数，不含终点；须大于起点且为整数纳秒或更粗 |
| `--end-h` | 未设置 | 同一终点的小时表示，与 `--end-s` 互斥 |

未填写区间时分析全部样本。区间为 `[start,end)`，时间戳不归零。即使只分析中间一段，
也会验证完整源文件，包括区间外的行。本工具只读 CSV，不读取 bag，也不按
`dataset.yaml` 自动选择 IMU；其他 CSV 位置通过 `--dataset` 直接指定。

## 输入

CSV 首行必须为以下七列，并保持顺序：

```csv
timestamp_ns,wx,wy,wz,ax,ay,az
```

可在最后增加第八列 `temperature` 或 `temperature_c`。时间戳必须是非负、严格递增的
整数纳秒；角速度为 `rad/s`，加速度为 `m/s²`，所有数值须有限。温度列存在时也须有限，
但不参与 Allan 计算。不会自动转换设备原始计数、角度制输入或加速度 `g` 单位。

使用设备静止时采集的连续记录。固定网格要求 `floor(effective_rate_hz) >= 10`，且样本数
严格大于 `2 * int(999.9 * floor(effective_rate_hz))`，保证最大尺度至少有两个完整分箱。
这通常要求约 2,000 s 以上的数据；更长的静止记录可以增加长尺度的分箱数。最低样本门槛
只保证能够计算，不保证长尺度拟合可信。本工具不会自动判断设备是否静止。

## 计算与参数含义

加速度保持 `m/s²`，陀螺仪先由 `rad/s` 转为 `deg/s` 计算曲线。实际平均采样率为
`(N-1)/(末时间戳-首时间戳)`，计算频率 `measure_rate` 取其向下取整值。按样本序号分箱，
不对不均匀时间戳重采样或插值。

六轴各计算 9,999 个尺度：`tau = 0.1, 0.2, ..., 999.9 s`，分箱大小为
`int(tau * measure_rate)`。每个尺度使用 `(N-1)//bin_size` 个不重叠完整分箱，对相邻分箱
均值差的平方取平均后除以 2，再开平方，得到 Allan deviation。末尾不足分箱的样本不参与
该尺度计算；`work/allan_variance.csv` 使用参考 C++ 输出的七位有效数字曲线。
五个尺度会用直接分箱均值独立复算，核对计算与序列化结果。

拟合保留参考实现的规则：

| 拟合项 | 使用尺度 | 对数域斜率范围 | 参数取值位置 |
|---|---|---|---|
| 白噪声 | `0.1 <= tau < 10 s` | `[-0.5, -0.499]` | 拟合曲线在 `tau=1 s` 的值 |
| 随机游走 | 全部 9,999 个尺度 | `[0.5, 0.501]` | 拟合曲线在 `tau=3 s` 的值 |

每组取 X/Y/Z 三轴的最大拟合值写入 YAML，陀螺仪两个噪声参数再转回弧度制。
曲线最小值、对应尺度及各轴拟合斜率记录在报告和 `work/fit.json` 中；最小值不会额外
写成 Kalibr 噪声参数。

| YAML 字段 | 单位 | 来源与范围 |
|---|---|---|
| `accelerometer_noise_density` | `m/s²/√Hz` | 三轴白噪声候选值最大值，有限且大于 0 |
| `accelerometer_random_walk` | `m/s³/√Hz` | 三轴随机游走候选值最大值，有限且大于 0 |
| `gyroscope_noise_density` | `rad/s/√Hz` | 三轴白噪声候选值最大值，转为弧度制，有限且大于 0 |
| `gyroscope_random_walk` | `rad/s²/√Hz` | 三轴随机游走候选值最大值，转为弧度制，有限且大于 0 |
| `update_rate` | `Hz` | 所选区间的实际平均采样率，有限且大于 0；不是取整后的 `measure_rate` |
| `rostopic` | 无 | `--topic` 的值 |

这些字段没有额外的固定噪声默认值。YAML 同时写入字符串
`schema_version: "1.0.0"` 和 `kind: imu_configuration`，遵循工程 IMU 输入契约。

## 输出与使用

```text
<output-dir>/
├── imu_noise.yaml
├── imu_allan.report.html
├── imu_allan.report.pdf
└── work/
    ├── allan_variance.csv
    ├── run.json
    ├── validation.json
    └── fit.json
```

HTML 内嵌曲线，可独立复制和分享；PDF 直接绘制矢量曲线，不生成独立的
`acceleration.png` 或 `gyro.png`。报告首页按摘要、参数表和数据来源分区排版，方法与
逐轴拟合明细独立成页，最后两页展示曲线。HTML/PDF 报告包含数据来源、选择区间、
样本数、时长、实际采样率与计算频率、候选参数、各轴拟合明细和曲线。成功后删除临时磁盘数组
`selected_samples.npy`，保留曲线与 JSON 证据。报告失败时保留已经生成的 YAML 和诊断，
并记录失败状态。

将 Camera–IMU task 的 IMU 配置路径指向生成文件，例如：

```yaml
imus:
  - id: imu0
    path: /data/results/imu_allan/imu_noise.yaml
    model: calibrated
```

task 中的相对路径以 task 所在目录为基准。本工具不会修改 task、机器模板或已有
`imu_noise.yaml`，也不会自动启动 Camera–IMU 标定。
目录任务若配置了 IMU 话题，`--topic` 应与其 `dataset.yaml` 的 IMU 话题一致。

输出必须位于源数据目录之外：CSV 在 `imu/` 下时保护其数据集父目录，否则保护 CSV
所在目录。输出目录及其同级 `<output-dir>.inprogress`、`<output-dir>.failed` 任一存在时
均拒绝运行，包括已有符号链接；没有 `--force` 覆盖选项。计算在 `.inprogress` 中进行，
全部成功后移至正式输出目录；计算或报告失败时保留 `.failed` 和 `work/run.json` 中的
明确失败状态。输入预检查失败时不创建结果目录。

## 限制与源码

全尺度强制拟合 `+0.5` 斜率并不证明曲线上存在可信的 `+1/2` 随机游走区域。非静止运动、
温漂、采样间隙和有限采集时长都可能影响候选参数。报告中的计算校验通过只表示数值
检查通过，不代表 IMU 噪声模型有效、Camera–IMU 标定合格或产线放行。

实现位于 `src/python/kalibr_tools/imu_allan/`：`input_format.py` 验证和选择 CSV，
`allan_core.py` 计算与复核曲线，`upstream_analysis.py` 拟合参数，`workflow.py` 管理执行
和安全输出，`reporting.py` 从曲线与拟合证据生成 HTML/PDF。命令统一注册在 `kalibr_tools/cli.py`，
核心相机与 Camera–IMU 标定流程不依赖此工具。
