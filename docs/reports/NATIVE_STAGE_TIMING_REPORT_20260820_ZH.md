# Native Kalibr 分层耗时与占比报告

日期：2026-08-20

分支：`opt/kalibr-native`

并行配置：`--detector-processes 4 --optimizer-threads 4`

构建配置：`-DKALIBR_ENABLE_PROFILING=ON`
统计配置：`--profile-optimizer --timing-json <path>`

上述统计接口属于显式 profiling 构建。默认构建不会注册优化器计时器、读取
`/proc` 内存统计或写 timing JSON；这份报告记录的是开启统计后的验证数据。

## 1. 报告目标

本报告回答三个问题：

1. 完整标定的时间主要消耗在哪些大阶段；
2. 每个大阶段内部最重要的 3～4 个子阶段各占多少；
3. ROS1 EuRoC 与 ROS2 EVT 的瓶颈为什么不同。

测试对象为：

- ROS1 EuRoC，双目相机 + IMU，`scale-misalignment`；
- ROS2 EVT，双目相机 + IMU，`calibrated`。

本报告使用 worker 解码版本的最新完整运行。原始产物位于：

- `/tmp/kalibr-timing-v3-20260820/ros1/worker-timing.json`；
- `/tmp/kalibr-timing-v3-20260820/ros1/worker-run.log`；
- `/tmp/kalibr-timing-v3-20260820/ros2/worker-timing.json`；
- `/tmp/kalibr-timing-v3-20260820/ros2/worker-run.log`。

## 2. 统计口径

### 2.1 Wall time

Wall time 是用户从阶段开始到阶段结束实际等待的时间。端到端阶段占比定义为：

\[
P_i = \frac{T_{i,\mathrm{wall}}}{T_{\mathrm{total,wall}}}\times 100\%.
\]

线程或进程并行时，所有 worker 的服务时间之和可能大于 wall time。例如四个
worker 各工作 30 秒，worker 累计服务时间约为 120 worker-s，但用户可能只等待
约 35 秒。因此只有关键路径 wall 可以直接计算端到端占比。

### 2.2 两套总时间

| 测量 | ROS1 | ROS2 | 用途 |
| --- | ---: | ---: | --- |
| `/usr/bin/time` 进程 wall | 144.18 s | 223.07 s | 用户实际端到端等待时间、版本间性能比较 |
| timing JSON 内部 wall | 147.12 s | 231.91 s | 阶段占比的统一分母 |

当前环境中两种时钟存在约 2%～4% 的差异。为了不混用计时边界，本报告所有阶段
占比都使用 timing JSON 内部 wall；端到端版本比较仍应使用 `/usr/bin/time`。

### 2.3 读取与检测为什么有两张表

正常运行采用有界流水线：父进程读取下一帧时，检测 worker 正在解码和检测上一
帧。因此读取与检测可以在逻辑和服务时间上完全分开，但不能把二者的服务时间
直接相加当成用户等待时间。

本报告同时给出：

- **联合流水线 wall**：用于端到端占比，保证总占比合理闭合；
- **读取服务时间**：定位 bag、payload 和反序列化瓶颈；
- **worker 服务时间**：定位 decode 和 target detection 瓶颈。

## 3. 端到端大阶段

“数据输入与图像流水线”在本表中保持联合关键路径，是为了避免重复计算读取与
检测的重叠时间。后续章节再将二者完全分栏诊断。

| 大阶段 | ROS1 wall | ROS1 占总 wall | ROS2 wall | ROS2 占总 wall |
| --- | ---: | ---: | ---: | ---: |
| 数据输入与图像流水线 | 70.54 s | 47.95% | 138.81 s | 59.86% |
| 初始化与 residual 图构建 | 21.94 s | 14.91% | 24.03 s | 10.36% |
| 最终联合优化 | 37.33 s | 25.37% | 52.93 s | 22.82% |
| 数值统计与结果输出 | 16.49 s | 11.21% | 14.97 s | 6.45% |
| 未覆盖的 CLI/阶段边界开销 | 0.82 s | 0.56% | 1.17 s | 0.50% |
| **timing JSON 总 wall** | **147.12 s** | **100.00%** | **231.91 s** | **100.00%** |

主要判断：

- ROS1 最大单项是数据输入与检测流水线，第二是最终优化；
- ROS2 约 60% 的内部 wall 位于数据输入与检测流水线，明显高于优化器的 22.82%；
- 所以只继续优化 Hessian/LM，不可能让 ROS2 端到端时间按相同比例下降。

## 4. 大阶段一：数据读取

下表是**读取服务需求**，用于比较读取内部构成。payload read 与 worker 检测存在
重叠，所以本表的 59.80/114.69 秒不是新的端到端独占阶段，不能与检测 wall
再次相加。

| 读取子阶段 | ROS1 服务时间 | 占 ROS1 读取服务 | ROS2 服务时间 | 占 ROS2 读取服务 |
| --- | ---: | ---: | ---: | ---: |
| IMU bag 读取与 measurement 构建 | 10.44 s | 17.45% | 1.95 s | 1.70% |
| 两路图像 bag 建索引 | 21.01 s | 35.13% | 58.11 s | 50.67% |
| 两路图像 payload 读取 | 28.00 s | 46.82% | 54.28 s | 47.32% |
| ROS 图像消息反序列化 | 0.36 s | 0.61% | 0.35 s | 0.31% |
| **读取服务需求合计** | **59.80 s** | **100.00%** | **114.69 s** | **100.00%** |

结论：

- ROS2 读取服务的 97.99% 位于图像建索引和 payload 读取；
- ROS 消息反序列化仅约 0.3%，不是当前优化重点；
- ROS1 也有建索引和二次 payload 读取，但 chunk/offset 定位和 raw Image 使代价低
  得多；
- ROS2 下一优先级应是消除逐帧 timestamp 查询或改成单遍、有界的顺序读取。

## 5. 大阶段二：图像解码与标定板检测

前三行是所有图像在 worker 中的累计服务时间，最后一行是用户实际等待的流水线
wall。它们不是同一口径，不能纵向相加。

| 图像子阶段/指标 | ROS1 | ROS1 worker 服务占比 | ROS2 | ROS2 worker 服务占比 |
| --- | ---: | ---: | ---: | ---: |
| 消息反序列化 | 0.36 worker-s | 0.29% | 0.35 worker-s | 0.12% |
| raw/compressed 图像 decode | 0.45 worker-s | 0.36% | 47.55 worker-s | 16.12% |
| target detection + 亚像素角点 | 123.03 worker-s | 99.34% | 247.11 worker-s | 83.76% |
| **worker 服务需求合计** | **123.84 worker-s** | **100.00%** | **295.02 worker-s** | **100.00%** |
| 四 worker 理想计算下界 | 30.96 s | — | 73.75 s | — |
| **两路实际 extraction pipeline wall** | **39.10 s** | — | **78.75 s** | — |
| 流水线计算效率（理想下界/实际 wall） | 79.19% | — | 93.65% | — |
| 处理吞吐率 | 73.61 image/s | — | 42.40 image/s | — |

解释：

- EuRoC 是 raw Image，decode 几乎没有成本，worker 时间几乎全部用于靶标检测；
- EVT 是 1920×1080 CompressedImage，decode 已占 16.12% worker 服务时间；
- 将 EVT decode 移入 worker 后，流水线 wall 已从 109.15 秒降到 78.75 秒；
- `TargetObservation` 打包、IPC、队列等待和按 index 归并目前包含在 pipeline wall
  中，尚无完全独立的子计时，因此报告不虚构一个“IPC 时间”。

## 6. 大阶段三：初始化

初始化位于 `problem_build_total` 内部，以下子阶段使用 wall，可直接计算阶段内
占比。

| 初始化子阶段 | ROS1 wall | 占 ROS1 初始化 | ROS2 wall | 占 ROS2 初始化 |
| --- | ---: | ---: | ---: | ---: |
| 两路相机–IMU 时间偏移初值 | 1.78 s | 60.41% | 2.05 s | 58.57% |
| 相机–IMU 旋转/gyro bias 先验 | 0.68 s | 23.14% | 0.82 s | 23.29% |
| pose spline 初始化 | 0.46 s | 15.68% | 0.62 s | 17.56% |
| bias spline 初始化 | 0.02 s | 0.76% | 0.02 s | 0.58% |
| **初始化合计** | **2.95 s** | **100.00%** | **3.51 s** | **100.00%** |

初始化仅占 ROS1/ROS2 总 wall 的约 2.01%/1.51%，不是当前端到端瓶颈。

## 7. 大阶段四：Residual 图构建

| 图构建子阶段 | ROS1 wall | 占 ROS1 图构建 | ROS2 wall | 占 ROS2 图构建 |
| --- | ---: | ---: | ---: | ---: |
| 设计变量注册 | 0.12 s | 0.66% | 0.03 s | 0.17% |
| 相机重投影 residual | 5.90 s | 31.12% | 6.22 s | 30.32% |
| accelerometer residual | 6.49 s | 34.23% | 7.08 s | 34.52% |
| gyroscope + bias-motion residual | 6.45 s | 33.99% | 7.18 s | 35.00% |
| **Residual 图构建合计** | **18.97 s** | **100.00%** | **20.51 s** | **100.00%** |

`problem_build_total` 还包含上一节初始化和约 0.02 秒包装开销，因此得到：

| Problem build inclusive | ROS1 | ROS2 |
| --- | ---: | ---: |
| 初始化 | 2.95 s | 3.51 s |
| Residual 图构建 | 18.97 s | 20.51 s |
| 包装/边界开销 | 0.02 s | 0.01 s |
| **总计** | **21.94 s** | **24.03 s** |

图构建主要是逐 measurement 创建 Python/Boost.Python residual 对象。设计变量
注册不到 1%，批量 C++ residual 构建才是下一步值得考虑的优化方向。

## 8. 大阶段五：最终联合优化

| 优化器子阶段 | ROS1 wall | 占 ROS1 优化器 | ROS2 wall | 占 ROS2 优化器 |
| --- | ---: | ---: | ---: | ---: |
| 并行 Jacobian/Hessian 构建 | 27.34 s | 73.25% | 34.87 s | 65.88% |
| CHOLMOD factorize/solve | 6.01 s | 16.09% | 11.34 s | 21.43% |
| 新状态 error evaluation | 2.52 s | 6.75% | 3.76 s | 7.11% |
| 参数更新、回滚、lambda/停止判断等 | 1.46 s | 3.90% | 2.95 s | 5.58% |
| **最终 Optimizer2** | **37.33 s** | **100.00%** | **52.93 s** | **100.00%** |

对应端到端占比：

| 优化器子阶段 | ROS1 占总 wall | ROS2 占总 wall |
| --- | ---: | ---: |
| Hessian 构建 | 18.59% | 15.04% |
| CHOLMOD factorize/solve | 4.08% | 4.89% |
| Error evaluation | 1.71% | 1.62% |
| 其他优化器开销 | 0.99% | 1.27% |

Hessian 构建仍是优化器内部最大项，但它只占完整 ROS2 wall 的约 15%。即使将其
理想地缩短一半，ROS2 端到端理论收益也只有约 7.5%，这符合 Amdahl 定律。

## 9. 大阶段六：数值统计与结果输出

| 输出子阶段 | ROS1 wall | 占 ROS1 输出 | ROS2 wall | 占 ROS2 输出 |
| --- | ---: | ---: | ---: | ---: |
| 优化前 residual statistics | 1.64 s | 9.93% | 1.23 s | 8.19% |
| 优化后 statistics + 结果格式化 | 1.60 s | 9.71% | 1.34 s | 8.98% |
| camchain/IMU YAML + TXT | 1.85 s | 11.21% | 1.26 s | 8.45% |
| PDF 报告生成 | 11.40 s | 69.15% | 11.13 s | 74.38% |
| **统计与输出合计** | **16.49 s** | **100.00%** | **14.97 s** | **100.00%** |

生成内容包括：

- 终端/日志中的优化前后相机、gyro、accel residual 统计；
- `*-camchain-imucam.yaml`；
- `*-imu.yaml`；
- `*-results-imucam.txt`；
- `*-report-imucam.pdf`。

`--dont-show-report` 只禁止弹窗，不跳过 PDF。PDF 占输出阶段约 69%～74%，如果
需要纯性能 benchmark，应新增显式 `--skip-report`；为了保持 ETHZ 默认行为，该
选项不能默认开启。

## 10. 最终瓶颈排序

### 10.1 ROS1 EuRoC

1. 数据输入与图像检测联合流水线：47.95%；
2. 最终联合优化：25.37%，其中 Hessian 占总 wall 18.59%；
3. 初始化与 residual 图构建：14.91%；
4. 数值统计与输出：11.21%，其中 PDF 约 7.75%。

ROS1 的读取与检测较均衡；后续优化 Hessian 和 residual 批量建图都能产生可见但
有限的端到端收益。

### 10.2 ROS2 EVT

1. 数据输入与图像检测联合流水线：59.86%；
2. 最终联合优化：22.82%，其中 Hessian 占总 wall 15.04%；
3. 初始化与 residual 图构建：10.36%；
4. 数值统计与输出：6.45%。

ROS2 第一瓶颈是 rosbag2 图像索引、payload 查询和高分辨率压缩图像处理，而不是
LM 状态机。优化优先级应为：

1. ROS2 单遍/row-id 有界读取，避免每帧 timestamp 二次查询；
2. 保持 decode 在 worker，继续降低队列复制与尾部等待；
3. 将 camera/gyro/accel residual 改为 C++ 批量构建；
4. 再优化 Hessian 并行效率与 CHOLMOD 路径；
5. 性能测试时提供显式跳过 PDF 的非默认选项。

## 11. 后续统计格式建议

后续每次基准应固定生成三份产物：

1. schema timing JSON：保存原始 stage、CPU、RSS/PSS 和并行配置；
2. 中文 Markdown：生成本文这种两级占比报告；
3. 控制台摘要：只显示大阶段和最重要瓶颈。

报告生成器必须保留以下字段，防止误读：

- `wall_seconds` 与 `percent_of_total`；
- `percent_of_parent`；
- `inclusive`；
- `aggregation: sum_over_images/sum_over_workers`；
- `overlaps_with`；
- `critical_path_wall_seconds`。

读取与检测只有在独立 benchmark 中才能获得严格互斥的 wall。正常流水线报告应
保持联合关键路径，同时提供拆开的服务需求；否则分阶段占比会错误地超过 100%。
