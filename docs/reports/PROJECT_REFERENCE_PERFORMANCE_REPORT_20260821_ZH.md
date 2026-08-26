# Project 与 Reference 公平并行性能测试报告

日期：2026-08-21  
提交：`2e85ad169f66c8e3c0aee98eb7b7a382b0bc2c5a`（`kalibr-native-v2`）

## 1. 测试目标

本报告使用相同输入、模型、数据区间、优化超参数和线程预算，对比：

| 测试组 | Project | Reference |
| --- | --- | --- |
| 4 并行 | detector=4、optimizer=4 | detector=4、optimizer=4 |
| 8 并行 | detector=8、optimizer=8 | detector=8、optimizer=8 |
| 原生默认观察组 | 不运行 | detector=31，各优化阶段2/4/31混合 |

需要回答：

1. 相同线程预算下，Project 的并行 Hessian 是否缩短 Camera–IMU 标定；
2. 从4增加到8后，图像流水线和最终优化分别获得多少收益；
3. Project 与 Reference 的标定参数是否保持数值一致；
4. 加速对应的内存成本是多少。

## 2. 测试对象与固定参数

### 2.1 EuRoC ROS1

- 相机 Bag：`cam_april.bag`；
- Camera–IMU Bag：`imu_april.bag`；
- 双目相机标定：`pinhole-radtan5`；
- 相机顺序：`shuffle: false`；
- Camera–IMU 固定输入：`output_cam2/cam_april-camchain.yaml`；
- IMU 模型：`scale-misalignment`；
- target、离群点过滤、时间偏移、LM/GN 超参数均保持默认且各组一致。

Camera–IMU 使用固定共同 camchain，而不是使用各相机 benchmark 的输出，防止相机
结果的微小差异级联进入 IMU 比较。

### 2.2 EVT ROS2

- Bag：`imu_mipi_in_stereo_calib_08_0730`；
- 图像：3339张1920×1080 `CompressedImage`；
- 双目输入：同一份 OpenCV K/D/RT 转换的 `radtan5` camchain；
- IMU 模型：`calibrated`；
- target、时间偏移、LM参数及输入数据范围均保持一致。

EVT 已有固定双目内外参，本轮不重新运行相机内外参标定，只测试
`calibrate imu-camera`。

### 2.3 运行环境

- 当前机器：32个逻辑 CPU；
- OpenCV detector worker 内部固定1线程；
- `OPENBLAS_NUM_THREADS=1`、`OMP_NUM_THREADS=1`、`MKL_NUM_THREADS=1`；
- 不并发运行不同 benchmark 组；
- Project 使用 `project-profile`，Reference 使用 `reference-release`；
- Project 保存 schema-3 timing JSON；
- 所有组使用 `/usr/bin/time -v` 记录完整进程 wall、CPU和最大RSS；
- PDF、YAML、TXT正常生成，没有通过跳过报告缩短测试时间。

## 3. 端到端性能

### 3.1 EuRoC 双目相机标定

| 配置 | wall | CPU | 最大RSS | 相对同线程Reference |
| --- | ---: | ---: | ---: | ---: |
| Project 4 | 1002.64 s | 124% | 1632.0 MiB | 慢9.58% |
| Reference 4 | 914.96 s | 126% | 1633.5 MiB | 基准 |
| Project 8 | 952.33 s | 129% | 1632.1 MiB | 快3.09% |
| Reference 8 | 982.71 s | 137% | 1631.5 MiB | 基准 |
| Reference 默认 | 894.46 s | 146% | 1630.9 MiB | 观察组 |

扩展性：

- Project 4→8：wall缩短5.02%；
- Reference 4→8：wall反而增加7.40%；
- Reference 默认相对Reference 4仅快2.24%。

判断：相机最终优化主要使用 IncrementalEstimator/GN/SPQR，而不是 Camera–IMU
使用的 BlockCholesky 并行 Hessian路径。提高线程数只能带来有限收益，甚至可能因
小 batch、SPQR/TBB调度和归约开销变慢。Project profile 还承担阶段采样开销，因此
Project 4慢于Reference 4不能解释为算法退化。

### 3.2 EuRoC Camera–IMU scale-misalignment

| 配置 | wall | CPU | 最大RSS | Project相对Reference |
| --- | ---: | ---: | ---: | ---: |
| Project 4 | 136.36 s | 165% | 1312.4 MiB | 快20.84% |
| Reference 4 | 172.25 s | 129% | 1139.1 MiB | 基准 |
| Project 8 | 130.86 s | 181% | 1425.2 MiB | 快22.27% |
| Reference 8 | 168.35 s | 136% | 1139.6 MiB | 基准 |
| Reference 默认 | 168.04 s | 146% | 1139.3 MiB | 观察组 |

扩展性和内存：

- Project 4→8：wall缩短4.03%；
- Reference 4→8：wall仅缩短2.26%；
- Project 4最大RSS比Reference 4高15.22%；
- Project 8最大RSS比Reference 8高25.07%。

判断：Project 的确定性 thread-local chunk Hessian带来约21%～22%的端到端收益，
但8线程相对4线程的边际收益只有约4%，并增加约113 MiB主进程最大RSS。

### 3.3 EVT ROS2 Camera–IMU calibrated

| 配置 | wall | CPU | 最大RSS | Project相对Reference |
| --- | ---: | ---: | ---: | ---: |
| Project 4 | 207.56 s | 217% | 1369.2 MiB | 快23.55% |
| Reference 4 | 271.50 s | 163% | 1040.4 MiB | 基准 |
| Project 8 | 194.92 s | 244% | 1479.6 MiB | 快20.81% |
| Reference 8 | 246.14 s | 186% | 1039.4 MiB | 基准 |
| Reference 默认 | 237.33 s | 206% | 1043.0 MiB | 观察组 |

扩展性和内存：

- Project 4→8：wall缩短6.09%；
- Reference 4→8：wall缩短9.34%，主要包含检测4→8的收益；
- Project 4最大RSS比Reference 4高31.60%；
- Project 8最大RSS比Reference 8高42.34%；
- Reference 默认相对Reference 4快12.59%，但使用31个检测进程和最终LM 31线程。

判断：Project 相同线程预算下快约21%～24%，但ROS2超过一半的内部wall仍位于
Bag/图像流水线，继续增加优化器线程无法线性改善端到端时间。

## 4. Project 阶段耗时与占比

以下占比统一使用 timing JSON内部wall。内部时钟与`/usr/bin/time`的命令边界略有
不同，不能混用分母。

### 4.1 EuRoC 相机标定

| 大阶段 | Project 4 | 占比 | Project 8 | 占比 |
| --- | ---: | ---: | ---: | ---: |
| Bag索引与图像检测流水线 | 66.39 s | 6.64% | 71.54 s | 7.54% |
| 内参与baseline初始化 | 14.27 s | 1.43% | 13.01 s | 1.37% |
| 最终增量 addBatch/GN | 605.51 s | 60.55% | 558.51 s | 58.89% |
| YAML/TXT/PDF输出 | 9.68 s | 0.97% | 9.55 s | 1.01% |
| 未插桩Python/批处理边界 | 304.17 s | 30.42% | 295.69 s | 31.18% |
| **内部总wall** | **1000.06 s** | **100%** | **948.33 s** | **100%** |

主要结论：固定`shuffle: false`后，相机时间主要消耗在最终逐视图增量和离群点
重加流程；检测不是本次相机标定的主瓶颈。

相机小阶段：

| 小阶段 | Project 4 | Project 8 |
| --- | ---: | ---: |
| 图像Bag索引 | 13.31 s | 15.12 s |
| 两路corner extraction pipeline | 53.08 s | 56.43 s |
| 单相机内参初始化（合计） | 6.36 s | 6.19 s |
| baseline/full-batch初始化 | 7.91 s | 6.82 s |
| 增量addBatch聚合 | 605.51 s | 558.51 s |
| PDF报告 | 9.38 s | 9.26 s |

### 4.2 EuRoC Camera–IMU

| 大阶段 | Project 4 | 占比 | Project 8 | 占比 |
| --- | ---: | ---: | ---: | ---: |
| 数据输入与图像流水线 | 53.10 s | 39.71% | 54.76 s | 42.50% |
| 初始化与residual图构建 | 40.89 s | 30.57% | 40.03 s | 31.06% |
| 最终联合LM | 25.15 s | 18.81% | 19.41 s | 15.07% |
| residual统计与结果输出 | 14.39 s | 10.76% | 14.46 s | 11.22% |
| 未覆盖边界 | 0.18 s | 0.14% | 0.17 s | 0.13% |
| **内部总wall** | **133.73 s** | **100%** | **128.85 s** | **100%** |

4→8后，最终联合LM从25.15秒降至19.41秒，缩短22.81%；但图像流水线没有
变快，因此端到端只缩短4.03%。

主要小阶段：

| 小阶段 | Project 4 | Project 8 |
| --- | ---: | ---: |
| IMU Bag读取+measurement构建 | 13.61 s | 12.79 s |
| 两路图像Bag索引 | 12.92 s | 12.39 s |
| 两路corner extraction pipeline | 26.57 s | 29.58 s |
| 时间偏移初值 | 1.66 s | 1.62 s |
| 旋转/gyro bias先验 | 0.57 s | 0.57 s |
| camera residual构建 | 7.21 s | 7.06 s |
| accel residual构建 | 15.57 s | 15.04 s |
| gyro residual构建 | 15.42 s | 15.28 s |
| 最终联合LM | 25.15 s | 19.41 s |
| PDF报告 | 10.19 s | 10.17 s |

### 4.3 EVT ROS2 Camera–IMU

| 大阶段 | Project 4 | 占比 | Project 8 | 占比 |
| --- | ---: | ---: | ---: | ---: |
| 数据输入与图像流水线 | 106.90 s | 51.88% | 101.76 s | 52.64% |
| 初始化与residual图构建 | 40.65 s | 19.73% | 41.79 s | 21.61% |
| 最终联合LM | 44.65 s | 21.67% | 35.99 s | 18.62% |
| residual统计与结果输出 | 13.37 s | 6.49% | 13.30 s | 6.88% |
| 未覆盖边界 | 0.45 s | 0.22% | 0.47 s | 0.24% |
| **内部总wall** | **206.03 s** | **100%** | **193.33 s** | **100%** |

主要小阶段：

| 小阶段 | Project 4 | Project 8 |
| --- | ---: | ---: |
| IMU Bag读取+measurement构建 | 5.98 s | 6.09 s |
| 两路rosbag2图像索引 | 33.96 s | 34.41 s |
| 两路corner extraction pipeline | 66.95 s | 61.26 s |
| worker decode服务需求 | 38.81 worker-s | 40.88 worker-s |
| worker target detection需求 | 218.92 worker-s | 227.94 worker-s |
| 时间偏移初值 | 1.60 s | 1.58 s |
| camera residual构建 | 8.18 s | 8.20 s |
| accel residual构建 | 14.94 s | 15.42 s |
| gyro residual构建 | 14.64 s | 15.28 s |
| 最终联合LM | 44.65 s | 35.99 s |
| PDF报告 | 10.05 s | 9.90 s |

worker服务时间是所有图像累计需求，可以大于用户等待wall，不能与pipeline wall
相加。EVT 4→8主要使extraction pipeline缩短8.50%、最终LM缩短19.39%，但图像
索引和residual建图基本不变。

## 5. 内存分析

### 5.1 `/usr/bin/time` 最大RSS

端到端表中的最大RSS不表示检测worker同时驻留内存的总和。它适合观察主命令
高水位，但不能代替进程树聚合PSS。

Project 使用thread-local Hessian chunk，会为每个优化线程保留局部块和RHS，因此
相对Reference增加内存。这是Camera–IMU获得20%～24%加速的主要空间换时间成本。

### 5.2 Project detector进程树峰值

| 数据/命令 | 并行 | 聚合RSS峰值 | 聚合PSS峰值 |
| --- | ---: | ---: | ---: |
| EuRoC cameras | 4 | 3713.4 MiB | 1372.4 MiB |
| EuRoC cameras | 8 | 6345.9 MiB | 1559.8 MiB |
| EuRoC Camera–IMU | 4 | 1200.8 MiB | 505.0 MiB |
| EuRoC Camera–IMU | 8 | 2102.9 MiB | 708.6 MiB |
| EVT Camera–IMU | 4 | 1078.6 MiB | 453.8 MiB |
| EVT Camera–IMU | 8 | 1855.6 MiB | 600.6 MiB |

RSS会在每个进程重复计算共享映射，因此8进程下增长很大；PSS按共享页比例分摊，
更适合衡量实际物理内存压力。检测8进程相对4进程的PSS增幅约14%～40%。

### 5.3 Project最终优化结束内存

| 数据 | 4线程RSS/PSS | 8线程RSS/PSS |
| --- | ---: | ---: |
| EuRoC Camera–IMU | 1263.9 / 1259.9 MiB | 1378.0 / 1374.0 MiB |
| EVT Camera–IMU | 1291.9 / 1287.9 MiB | 1311.6 / 1307.6 MiB |

EuRoC scale-misalignment参数块更多，8线程局部Hessian内存增量比EVT calibrated明显。

## 6. 数值一致性

所有比较均递归提取`calibration.yaml`中的对应数值字段。相同线程预算是主判据。

| 数据/命令 | 配对 | 最大绝对差 | 最大相对差 | 结果 |
| --- | --- | ---: | ---: | --- |
| EuRoC cameras | Project 4 vs Reference 4 | 0 | 0 | YAML字节一致 |
| EuRoC cameras | Project 8 vs Reference 8 | 0 | 0 | YAML字节一致 |
| EuRoC cameras | Project 4 vs Project 8 | `4.23e-12` | `1.07e-11` | 浮点归约差异 |
| EuRoC Camera–IMU | Project 4 vs Reference 4 | `1.45e-9` | `1.75e-5` | 数值一致 |
| EuRoC Camera–IMU | Project 8 vs Reference 8 | `1.82e-9` | `1.61e-5` | 数值一致 |
| EuRoC Camera–IMU | Project 4 vs Project 8 | `3.03e-10` | `1.84e-5` | 数值一致 |
| EVT Camera–IMU | Project 4 vs Reference 4 | `3.48e-9` | `1.20e-7` | 数值一致 |
| EVT Camera–IMU | Project 8 vs Reference 8 | `1.58e-9` | `5.38e-8` | 数值一致 |
| EVT Camera–IMU | Project 4 vs Project 8 | `2.15e-9` | `7.36e-8` | 数值一致 |
| EuRoC cameras | Reference默认 vs Reference 4 | 0 | 0 | YAML字节一致 |
| EuRoC Camera–IMU | Reference默认 vs Reference 4 | `1.10e-10` | `6.70e-6` | 数值一致 |
| EVT Camera–IMU | Reference默认 vs Reference 4 | `2.24e-9` | `7.76e-8` | 数值一致 |

EuRoC较大的相对差出现在接近零的IMU intrinsic或时间偏移项；绝对差仍为
`1e-9`量级。相机同线程Project/Reference字节一致，证明工程迁移和统一CLI没有
改变相机算法结果。

## 7. 结论

1. **数值一致性通过。** 相机同线程结果字节一致，Camera–IMU最大绝对差不超过
   `3.48e-9`。
2. **并行Hessian对Camera–IMU有效。** Project相对同线程Reference端到端快约
   20%～24%。
3. **8线程边际收益有限。** Project从4增至8，EuRoC/EVT端到端仅再快4.03%和
   6.09%，但内存明显增加。
4. **相机标定不应盲目增加线程。** Project 4→8只快5.02%，Reference 8反而比4
   慢7.40%；最终增量GN、batch管理和过滤是主瓶颈。
5. **ROS2首要瓶颈仍是数据与图像流水线。** EVT在4/8下约52%的内部wall位于
   数据输入和检测，继续只优化LM无法获得等比例端到端收益。
6. **推荐默认性能配置为4/4。** 8/8适合内存充足且更关注最低wall的场景；31进程
   原生默认收益有限，不适合作为资源受限机器的工程默认值。

## 8. 统计限制

- 每组只完整运行一次，wall会受文件系统缓存、CPU频率和系统调度影响；小于约5%的
  差异应通过多次重复后再作强结论。
- Project使用profile构建，Reference使用release构建；Project承担计时、PSS采样和
  JSON写入开销，因此性能数据对Project略不利。
- Reference没有schema-3阶段统计，Reference阶段差异不能由本报告直接拆分。
- 当前schema记录最终Optimizer2整体wall，没有在本轮单独输出Hessian、CHOLMOD
  factorize/solve和error evaluation明细；报告不根据旧数据虚构本轮内部占比。
- `/usr/bin/time`最大RSS与Project detector进程树聚合RSS/PSS口径不同，不能直接
  互相替代。

## 9. 原始产物

EuRoC：

```text
/mnt/q/File/kalibr/data/euroc_cam/performance_project_reference_v2
```

EVT：

```text
/mnt/q/File/kalibr/data/evt_8_0730/imu_mipi_in_stereo_calib_08_0730/
performance_project_reference_v2
```

每个运行目录包含`calibration.yaml`、`results.txt`和`report.pdf`；Project目录额外
包含`timing.json`。`logs/`保存完整控制台日志、GNU time报告和退出状态。
