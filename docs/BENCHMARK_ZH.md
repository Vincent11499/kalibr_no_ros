# Kalibr no-ROS 固定基线与性能调优规范

## 1. 基线不再重新运行

`benchmarks/baselines-v1.yaml` 冻结三项 Project 性能基线：

| ID | 任务 | 性能基线 | 数值基线 |
|---|---|---|---|
| `euroc_camera_project_4` | EuRoC ROS1 双目相机 | `camera_project_4` | `camera_reference_default` |
| `euroc_imu_project_4` | EuRoC ROS1 双目–IMU scale-misalignment | `imu_project_4` | `imu_reference_default` |
| `evt_imu_project_4` | EVT ROS2 双目–IMU calibrated | `imu_project_4` | `imu_reference_default` |

这是四类逻辑结果：三项 Project 性能结果加一个 `reference_default` 数值参考组。
Reference 组在物理上包含三个任务各自的结果，所以实际保留六个运行目录。

注册表记录 task、Project calibration/results/timing 和 Reference calibration 的
SHA-256。比较前重新计算哈希；任何历史文件丢失或变化都会立即拒绝比较。benchmark
命令从不执行注册表中的基线命令，只运行新的 Candidate。

## 2. 公平比较条件

Candidate 必须与基线满足：

- 相同任务、数据集路径和模型；
- `detector_processes: 4`、`optimizer_threads: 4`；
- 相机标定必须 `shuffle: false`；
- Camera–IMU 沿用原生按时间戳顺序处理，不存在 shuffle 分支；
- 算法参数不因性能实验发生变化。

不满足上述契约时，`benchmark compare` 报错，不生成性能结论。

## 3. 参数归属与默认值

数据、target、topic、相机/IMU 模型和算法选项属于 task YAML。运行目录、归档名、
重复次数和比较容差属于 CLI。

标准执行配置为：

```yaml
execution:
  detector_processes: 4
  optimizer_threads: 4
  detector_inflight_per_worker: 2
  detector_opencv_threads: 1
  profiling_memory_sample_interval_s: 0.25
```

CLI 可以临时覆盖执行参数，优先级为：组件专用 CLI、CLI `--parallelism`、组件专用
YAML、YAML `parallelism`、标准默认值。所有最终值都会写进 `effective-task.yaml`。

新增性能参数的含义：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `detector_inflight_per_worker` | 2 | 每个 detector worker 允许的在途图像数；降低可节省队列内存，提高可能改善流水线饱和度 |
| `detector_opencv_threads` | 1 | 每个 detector 进程内部的 OpenCV 线程数；默认 1 防止进程与线程乘法超配 |
| `profiling_memory_sample_interval_s` | 0.25 s | benchmark 和 detector 进程树内存采样周期；更小更容易捕获尖峰但测量开销更高 |

LM/GN、QR、信息增益、异常点、同步和时间偏移参数不是本轮性能超参数。benchmark
会把其有效默认值显式写入 task，但不会改变 ETHZ 算法。

## 4. Reference 默认并行配置

不传 `--detector-processes` 和 `--optimizer-threads` 时，Reference 保留 ETHZ 各
阶段原有默认值，并不是统一的线程预算。`CPU-1` 表示
`max(1, multiprocessing.cpu_count() - 1)`；当前 32 逻辑 CPU 的机器上等于 31。

| 阶段 | Reference 默认 |
|---|---:|
| 图像标定板检测 | `CPU-1` 个进程 |
| 单相机内参初始化 LM | 4 线程 |
| 双目 baseline 初始化 LM | 4 线程 |
| 全批量相机 refinement LM | 4 线程 |
| 相机最终增量 GN | `CPU-1` 线程 |
| Camera–IMU 时间偏移互相关 | 主进程，不使用 Optimizer2 |
| Camera–IMU 旋转/gyro bias 初值 | 2 线程 |
| spline 初始化与 residual 建图 | 主进程为主 |
| Camera–IMU 最终联合 LM | `CPU-1` 线程 |
| covariance（可选） | 通用默认 4 线程 |

相机标定可简化为：检测 `CPU-1`，三个初始化/批量 LM 阶段 4 线程，最终增量 GN
使用 `CPU-1`。Camera–IMU 可简化为：检测 `CPU-1`，旋转初值 2 线程，最终联合
LM 使用 `CPU-1`，其余若干初始化和输出阶段主要由主进程执行。

Reference 最终 LM 的线程参数主要影响 error evaluation；原生
`BlockCholeskyLinearSystemSolver` 的 Hessian 构建并不使用该参数并行，CHOLMOD
分解也不由它严格控制。因此不能把 `CPU-1` 理解为整个优化阶段持续占满全部核心。

项目的统一 CLI 可给 Project 和 Reference 显式传入：

```text
--detector-processes N --optimizer-threads N
```

前者只控制检测 worker，后者覆盖已经接入的 LM、GN 和 IncrementalEstimator
阶段；时间互相关、spline 初始化、residual 建图和报告输出不受其直接控制。公平
比较必须显式使用相同线程预算，因此冻结基线统一采用 4/4，而不是 Reference 默认。

## 5. 运行与归档

必须使用 `project-profile`：

```bash
build/project-profile/bin/kalibr-noros benchmark run \
  --config camera-task.yaml \
  --archive-dir /data/benchmark-runs \
  --name camera-candidate \
  --repeat 1
```

每个运行生成唯一 UTC 时间戳目录，不允许覆盖：

```text
camera-candidate/<UTC>-<git>/
├── task.yaml
├── effective-task.yaml
├── run.json
├── summary.json
└── trial-001/
    ├── stdout-stderr.log
    ├── timing.json
    ├── manifest.json
    └── outputs/
        ├── calibration.yaml
        ├── results.txt
        └── report.pdf
```

`manifest.json` 保存完整命令、退出状态、wall、CPU、进程树聚合 RSS/PSS 和文件哈希。
`--repeat N` 会保存每次 trial，并输出中位数、最小值、最大值和 MAD。

比较示例：

```bash
build/project-profile/bin/kalibr-noros benchmark compare \
  --registry benchmarks/baselines-v1.yaml \
  --baseline-id euroc_camera_project_4 \
  --candidate /data/benchmark-runs/camera-candidate/<run-id>
```

默认数值容差为 `atol=1e-8`、`rtol=1e-8`，同时报告实际最大差异。单次运行的 wall
差异小于 5% 时标记为“结论不足”；若需确认，只增加 Candidate 的重复次数，仍不
重新运行历史基线。

## 6. 内存口径

新 benchmark 周期性采样 Candidate 的完整进程树并汇总 RSS/PSS。旧基线只保存了
GNU time 最大 RSS 和 detector 阶段进程树峰值，二者与新全流程聚合内存口径不同，
所以注册表只保留旧值供历史查阅，不伪造内存百分比。下一轮 Candidate 会成为具有
完整聚合内存口径的新阶段性基线。
