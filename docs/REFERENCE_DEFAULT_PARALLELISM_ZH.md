# Reference 默认并行配置

本文说明在不传入 `--detector-processes` 和 `--optimizer-threads` 时，
`reference-release` 保留的 ETHZ Kalibr 各阶段默认并行配置。

`CPU-1` 表示 `max(1, multiprocessing.cpu_count() - 1)`。当前测试机器有 32 个
逻辑 CPU，因此 `CPU-1 = 31`。更换机器后，该数值会随逻辑 CPU 数量变化。

## 总览

| 阶段 | Reference 默认 |
| --- | ---: |
| 图像标定板检测 | `CPU-1 = 31` 个进程 |
| 单相机内参初始化 LM | 4 线程 |
| 双目 baseline 初始化 LM | 4 线程 |
| 全批量相机 refinement LM | 4 线程 |
| 相机最终增量 GN | `CPU-1 = 31` 线程 |
| Camera–IMU 旋转/gyro bias 初值 | 2 线程 |
| Camera–IMU 最终联合 LM | `CPU-1 = 31` 线程 |
| 通用 `Optimizer2Options` 默认 | 4 线程 |

Reference 没有统一的默认优化线程数。各阶段显式保留 ETHZ 原有的 2、4 或
`CPU-1` 设置。

## 相机标定

| 阶段 | Reference 默认 | 简要说明 |
| --- | ---: | --- |
| Bag 索引、读取和任务提交 | 1 个主进程 | 读取图像并向检测 worker 提交任务 |
| 图像标定板检测 | `CPU-1 = 31` 个进程 | cam0、cam1 通常依次检测；每个 worker 内 OpenCV 为 1 线程 |
| 单相机内参初始化 LM | 4 线程 | 分别估计每台相机的投影、畸变和各视图 target pose |
| 双目 baseline 初始化 LM | 4 线程 | 初始化相邻相机外参和共同视图 target pose |
| 全批量 refinement LM | 4 线程 | 联合优化内参、畸变、baseline 和所有视图 pose |
| 最终逐视图增量 GN | `CPU-1 = 31` 线程 | 使用 IncrementalEstimator、SPQR/SVD 和信息增益筛选 |
| 离群点过滤后的重优化 | 继承对应增量阶段设置 | 删除角点后重新加入 batch 并优化 |
| YAML、TXT、PDF 输出 | 以主进程为主 | 不属于 Optimizer2 并行阶段 |

相机标定默认线程配置可简化为：

```text
检测：31 个进程
初始化 LM：4 线程
baseline LM：4 线程
full-batch LM：4 线程
最终增量 GN：31 线程
```

默认图像处理顺序会执行随机 shuffle。进行数值一致性和性能对比时，应明确设置
`shuffle: false`，从而固定逐视图增量优化的输入顺序。

## Camera–IMU

| 阶段 | Reference 默认 | 简要说明 |
| --- | ---: | --- |
| Bag 索引、图像和 IMU 读取 | 1 个主进程 | 建立图像观测和 IMU measurement |
| 图像标定板检测 | `CPU-1 = 31` 个进程 | 每个检测 worker 内 OpenCV 为 1 线程 |
| 相机–IMU 时间偏移互相关 | 主进程，不使用 Optimizer2 | 对视觉角速度模长和 gyro 模长做互相关，得到时间偏移初值 |
| Camera–IMU 旋转/gyro bias 初值 | 2 线程 | 固定视觉 pose spline，优化旋转和常量 gyro bias |
| pose/bias spline 初始化 | 主进程为主 | 默认 order 6；pose 100 knots/s，bias 50 knots/s |
| residual 图构建 | 主进程为主 | 创建 camera、gyro、accel 和 bias-motion residual |
| 最终联合 LM | `CPU-1 = 31` 线程 | 默认最大 30 次迭代，联合优化 spline、bias、重力、外参和时间偏移 |
| covariance 恢复（可选） | 通用默认 4 线程 | 只有启用 `recover_covariance` 时执行 |
| 统计及 YAML、TXT、PDF 输出 | 以主进程为主 | 不属于最终 LM 线程数 |

Camera–IMU 默认线程配置可简化为：

```text
检测：31 个进程
时间偏移互相关：主进程，无 Optimizer2
旋转/gyro bias 初值：2 线程
spline 初始化和 residual 建图：主进程为主
最终联合 LM：31 线程
covariance（可选）：4 线程
```

Reference 的最终 Camera–IMU LM 虽配置为 `CPU-1`，但原生
`BlockCholeskyLinearSystemSolver` 的 Hessian 构建不使用该参数并行，CHOLMOD
分解也不由该参数严格控制。因此这里的 31 线程主要作用于 error evaluation，不能
理解为整个最终 LM 都能占满 31 核。

## 显式参数覆盖

当前工程生成的 `reference-release` CLI 支持显式设置：

```bash
--detector-processes N --optimizer-threads N
```

其中：

- `--detector-processes` 只覆盖标定板检测 worker 数量；
- `--optimizer-threads` 统一覆盖已接入的 LM、GN 和 IncrementalEstimator 阶段；
- 没有显式传入时，保留本文列出的 ETHZ 各阶段默认值；
- 时间互相关、spline 初始化、residual 建图和报告输出不受
  `--optimizer-threads` 直接控制。

如果要比较 project 与 reference 的性能和数值，应让双方使用相同的检测进程数、
优化线程数、输入顺序、模型、数据区间及优化超参数。
