# Kalibr no-ROS v2 工程架构

## 两套源码

`ref/kalibr` 是 ETHZ 原始快照。它有独立哈希清单，只用于 reference
preset，不接受补丁、生成代码和运行输出。

`src/kalibr` 是项目正式源码，按职责重排父目录，但保留原 package、C++
namespace、Python module 和 public include 名称：

```text
foundation/    Schweizer-Messer 与 Python/Eigen 基础设施
camera/        相机、图像、标定板与视觉误差项
optimization/  优化后端和稀疏矩阵
trajectory/    B-spline 连续时间轨迹
calibration/   增量相机标定和 camera–IMU 标定
third_party/   AprilTag
```

两套源码由 `KALIBR_SOURCE_VARIANT=project|reference` 选择，构建目录和安装
目录始终隔离。无 ROS bag I/O、统一 CLI 和相机模型注册属于共同的应用边界，
因此 reference/project 能读取完全相同的数据。

## 运行数据流

```text
v2 task YAML
  -> ROS1/ROS2 bag 索引与消息解码
  -> 标定板并行检测（有界队列、按输入序恢复顺序）
  -> ETHZ 相机或 camera–IMU 阶段机
  -> 原生 Optimizer2 / IncrementalEstimator
  -> 结果统计、PDF 与统一 calibration.yaml
```

统一 CLI 只校验配置、生成临时兼容输入、调用原生阶段机和规范化文件名，
不重写残差、迭代、接受/回退或终止行为。

## 构建与诊断

- `KALIBR_ENABLE_PROFILING=OFF`：默认不采集阶段计时和 PSS/RSS；
- `KALIBR_ENABLE_DIAGNOSTIC_IO=OFF`：默认不产生额外诊断文件；
- `KALIBR_ENABLE_SOURCE_AUDIT=ON`：默认校验 reference 和 source delta；
- `KALIBR_ENABLE_TESTING=OFF`：release 不构建历史测试目标。

项目只保留一个依赖准备脚本，其余验证通过 CMake Presets、CTest 和 `tools/`
中的单用途程序完成。

所有 build preset 固定 `jobs: 4`，test preset 固定 `execution.jobs: 4`；即使调用者
省略 `--parallel 4` 或 `-j4`，预设也不会启动超过四个并行任务。
