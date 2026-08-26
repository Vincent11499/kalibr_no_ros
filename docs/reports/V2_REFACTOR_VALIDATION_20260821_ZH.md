# Kalibr no-ROS v2 重构与一致性验证报告

## 1. 结论

本次升级完成了工程目录重构、统一命令行、ROS1/ROS2 无 ROS 读取、OpenCV
相机模型扩展、显式并行控制和可选分阶段统计。标定算法本身没有改成新的
求解器：各阶段仍使用 ETHZ Kalibr 的残差、参数激活、Optimizer2 / IncrementalEstimator、
LM/GN 超参数、接受与回退规则、离群点处理和终止条件。

截至 2026-08-21 的验证结果：

- 冻结参考快照为 1,630 个文件，SHA-256 为
  `9b9c8658d901b6b53772255686513a668aa8d5ca656d2beaaa7edf431d031f0a`；
- project 源码审计结果为 1,580 个未修改文件、17 个批准的修改、32 个明确省略项；
- project-test 完整构建成功，CTest 注册 26 项、禁用 1 项、实际执行 25 项，
  25/25 通过；
- project、reference、profile 三种完整构建均成功，所有构建并行数不超过 4；
- project 和 reference 安装树均可在未加载 ROS 环境时运行公共命令；
- ROS1、ROS2 camera–IMU 结果与旧 native 结果在浮点舍入误差范围内一致；
- 一个确定性双目标定窗口的 project/reference `calibration.yaml` 字节完全一致。

因此可以判定：目录迁移和应用层封装没有改变标定算法的数值语义。多线程会改变
浮点加法的结合顺序，故完整长数据集不承诺逐字节一致，但当前最大差异均在
`1.5e-9` 以内。

## 2. 工程结构

| 目录 | 职责 |
|---|---|
| `ref/kalibr` | 冻结 ETHZ 参考快照，只做校验和 reference 对照构建 |
| `src/kalibr` | 可维护的正式 Kalibr 源码，按基础、相机、优化、轨迹、标定分域 |
| `src/python` | 无 ROS bag I/O、v2 CLI、任务转换和运行期统计 |
| `src/camera_models` | OpenCV radtan5、zero-skew fisheye、完整 alpha/skew fisheye |
| `schemas` | v2 任务与结果 YAML 约束 |
| `tools` | 依赖准备、参考校验、源码差异审计、结果比较和 benchmark |
| `tests` | 应用层、相机模型、并行优化和回归测试 |

`ref/kalibr` 不参与 project 源码修改。`src/kalibr` 不是运行期覆盖层，而是已经
物化的工程源码；这避免了构建时文本替换和补丁顺序成为隐式行为。

## 3. 构建与安装验证

统一使用 Ninja/CMake Presets。为了限制内存压力，本报告所有编译最多使用四个
并行任务：

```bash
cmake --preset project-test
cmake --build --preset project-test --parallel 4
ctest --preset project-test -j4 --output-on-failure
```

验证矩阵：

| preset | 源码 | profiling | testing | 结果 |
|---|---|---:|---:|---|
| `project-test` | `src/kalibr` | 关 | 开 | 完整构建成功，25/25 执行测试通过 |
| `project-profile` | `src/kalibr` | 开 | 关 | 完整构建与安装成功 |
| `reference-release` | `ref/kalibr` | 关 | 关 | 549/549 构建步骤成功，安装成功 |

release 默认关闭 `KALIBR_ENABLE_PROFILING` 和
`KALIBR_ENABLE_DIAGNOSTIC_IO`，不会采集 `/proc` 内存、阶段计时或生成额外诊断
JSON。需要性能分析时显式使用 `project-profile`。

安装树只暴露 `kalibr-noros`，内部 ETHZ 入口位于 `libexec`，避免把实现细节当成
长期公共 API。已验证下列接口的 `--help` 或实际执行：

```text
kalibr-noros calibrate cameras
kalibr-noros calibrate imu-camera
kalibr-noros convert camera
kalibr-noros convert job
kalibr-noros reference verify
```

## 4. 数值一致性验证

### 4.1 ROS1 EuRoC camera–IMU，scale-misalignment

| 项目 | v2 project | 旧 native |
|---|---:|---:|
| 退出状态 | 0 | 0 |
| 最大 YAML 绝对差异 | `6.90285796981e-10` | 基准 |
| 非数值字段差异 | 0 | 基准 |
| v2 wall | 121.04 s | — |
| v2 最大 RSS | 1,354,200 KiB | — |

最大差异位于 cam0 时间偏移。量级属于线程归约顺序导致的末位舍入，不表示优化
阶段、模型或超参数发生变化。

### 4.2 ROS2 EVT camera–IMU，calibrated

| 项目 | v2 project | 旧 native |
|---|---:|---:|
| 退出状态 | 0 | 0 |
| camchain 最大绝对差异 | `1.43174694323e-9` | 基准 |
| IMU YAML | 字节一致 | 基准 |
| 非数值字段差异 | 0 | 基准 |
| v2 wall | 198.22 s | — |
| v2 最大 RSS | 1,350,352 KiB | — |

ROS2 并不存在另一套优化算法。与 ROS1 的主要区别在 bag 索引和消息反序列化：
ROS1 使用 chunk/offset，ROS2 使用 metadata、record timestamp 和 ordinal；完成解码
以后，检测和优化进入同一套 Kalibr 代码。

### 4.3 ROS1 双目标定 project/reference

为消除随机视图顺序影响，选用 EuRoC 20–23 s 窗口并启用 `no-shuffle`：

| 项目 | project | reference |
|---|---:|---:|
| 退出状态 | 0 | 0 |
| `calibration.yaml` | 字节完全一致 | 字节完全一致 |
| wall | 42.16 s | 40.35 s |
| 最大 RSS | 370,152 KiB | 377,792 KiB |

该验证覆盖单相机内参初始化、双目 baseline 初始化、全批量 refinement、逐视图增量
估计、信息增益判断和最终输出。project-profile 多出的约 1.8 s 包含统计开销，不能
解释为优化算法退化。

另完成了一次 1,450+1,450 帧的完整相机标定，退出状态为 0；在固定
`no-shuffle` 顺序下 wall 为 17:50.76。固定顺序会显著影响逐视图增量阶段的计算量，
该结果只证明全流程可运行，不作为常规性能基准。

## 5. 分阶段统计定义

profile 构建的 timing JSON 使用 schema 3。一级阶段按实际执行路径记录：

1. bag 索引/读取与消息反序列化；
2. 图像解码；
3. 标定板检测；
4. 初始化、图构建和优化；
5. 统计、YAML/TXT/PDF/CSV 输出。

其中优化细分为相机内参初始化、相邻相机 baseline 初始化、全批量 refinement、
逐视图增量 `addBatch`，以及 camera–IMU 的时间/旋转先验、样条初始化、最终联合 LM
和可选 covariance 恢复。重复的增量调用按调用点聚合，报告 count、总 wall/CPU、
min/max/mean，避免数百到数千条记录使 JSON 膨胀。一次三秒窗口的统计文件由约
970 KiB 降到约 16 KiB。

`wall_seconds` 是现实经过时间，包含 CPU 运算、锁等待、I/O、调度和子进程等待；
`cpu_seconds` 是当前进程实际消耗的 CPU 时间，多核执行时可能大于 wall。类别占比
使用 exclusive wall，父阶段会扣除已记录的子阶段，因而不会重复计数；每个小阶段
同时保留 inclusive 和 exclusive wall，inclusive 不能再与其子阶段简单相加。

检测阶段内存是父进程与存活 worker 的周期采样聚合峰值；优化阶段 RSS/PSS scope
是主进程。`lifetime_peak_at_end` 是进程生命周期高水位，不是该阶段独占峰值。

默认 release 不生成 timing JSON。profile 构建中可使用：

```bash
kalibr-noros calibrate imu-camera \
  --config task.yaml --output-dir output \
  --detector-processes 4 --optimizer-threads 4 \
  --timing-json timing.json
```

## 6. 算法不变量与允许差异

重构保持下列内容不变：

- 相机各初始化、full-batch、增量 GN 阶段和离群重加流程；
- camera–IMU 时间互相关、旋转先验、order-6 pose/bias spline 和最终联合 LM；
- LM 初始 lambda、更新律、绝对 `deltaX/deltaJ` 停止条件和回滚语义；
- 参数活动集合、残差定义、M-estimator 默认值和输出标定方向。

project 相对 reference 的批准修改只位于无 ROS I/O、OpenCV 模型、无头运行、显式
线程传递、确定性并行 Hessian 和可选诊断边界。并行 Hessian 使用 thread-local
chunk，worker 内保持原 error 顺序，主线程固定顺序归并；单线程路径保留原循环。

允许的差异是：不同线程数会改变浮点归约括号化；profiling 会增加少量计时、内存
采样和 JSON I/O 开销；OpenCV 扩展模型产生上游 ETHZ 不认识的新 schema。以上均不
改变既有模型在固定设置下的算法定义。

## 7. 复验命令

```bash
python3 tools/verify_reference.py
python3 tools/audit_source_delta.py
python3 tools/compare_kalibr_outputs.py old.yaml new.yaml
```

历史的 ROS1/ROS2 逐阶段性能分析见
[`NATIVE_STAGE_TIMING_REPORT_20260820_ZH.md`](NATIVE_STAGE_TIMING_REPORT_20260820_ZH.md)。
