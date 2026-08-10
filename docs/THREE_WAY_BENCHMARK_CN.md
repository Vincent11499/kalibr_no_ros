# 三条 Kalibr 实现路线对比

## 1. 对比对象

后续性能和精度报告固定区分以下三条路线，并始终以路线 1 为基准：

| 路线 | 分支/构建 | 数值优化器 | 目的 |
| --- | --- | --- | --- |
| 1. 原生 Kalibr | ROS1 工作区 `devel/lib/kalibr` 原生 CLI | Kalibr LM + Block Cholesky | 精度、时间和内存基准 |
| 2. 原生优化版 | `opt/kalibr-native` | 仍为 Kalibr LM + Block Cholesky | 保持方程与求解策略不变，减少临时拷贝并优化前处理 |
| 3. Ceres 改造版 | `opt/ceres` | Ceres 2.2 LM + SuiteSparse | 利用现代并行求解并重新组织 residual graph |

路线 1 直接运行已经编译好的 ROS1 Kalibr CLI，不经过本项目的 no-ROS 启动器。
另行构建的 `main`/`install/native-baseline` 使用未修改的上游 Kalibr 数值源码和
no-ROS 数据接口；它与路线 1 的三个结果文件逐字节一致，只作为接口等价性旁证，
不占用独立路线编号。

路线 2 不是 Ceres，也没有改变状态量、误差项、LM 接受策略或 Block Cholesky
求解器。当前主要优化是：

- 使用有界多进程队列提取角点，避免 Manager 队列同时复制整包图像；
- 避免每个误差项构造 Jacobian `std::map` 深拷贝；
- 避免 B-spline motion error 每次迭代重复复制全部系数；
- 增加原生优化器分阶段计时能力。

路线 3 重新用线程安全的 Ceres residual 构造 IMU—相机联合问题。位姿/偏置
B-spline、重力、外参、时间偏移以及三种 IMU 内参模型都进入同一 Ceres 问题。

## 2. 统一测试条件

测试日期：2026-08-10。数据和参数如下：

- 数据：`data/euroc_cam/imu_april.bag`，完整 bag；
- target：`data/euroc_cam/april_6x6.yaml`；
- camchain：`data/euroc_cam/output_cam1/cam_april-camchain.yaml`；
- IMU：`data/euroc_cam/imu_adis16448.yaml`；
- IMU 模型：`calibrated`；
- 最大迭代：30；
- Ceres：15 个工作线程；
- 三条路线都检测到 1399/1439 帧 AprilGrid，使用 14381 条 IMU 数据；
- 三条命令串行执行，使用 `/usr/bin/time -v` 记录整个进程的墙钟和峰值 RSS。

路线 2 和路线 3 使用相同的有界多进程角点提取实现。路线 1 保留上游 Manager
队列实现。三者检测结果相同，但端到端数据包含了路线 2/3 的前处理优化收益，不能把
端到端差值全部解释成非线性求解器差值。Ceres 自身 solve 时间由 Ceres summary
单独给出。

## 3. 精度对比

| 指标 | 1. 原生 Kalibr | 2. 原生优化版 | 3. Ceres 改造版 |
| --- | ---: | ---: | ---: |
| 最终目标函数 | `J=43975.1` | `J=43975.1` | `2*cost=43975.1258` |
| 重投影均值 | 0.406838986 px | 0.406838986 px | 0.406839326 px |
| gyro 均值 | 0.007785401 rad/s | 0.007785401 rad/s | 0.007784987 rad/s |
| accel 均值 | 0.049746279 m/s² | 0.049746279 m/s² | 0.049745741 m/s² |
| 相对基准外参旋转差 | 0 | 0 | 6.85e-5 deg |
| 相对基准外参平移差 | 0 | 0 | 1.37e-6 m |
| 相对基准时间偏移差 | 0 | 0 | 9.09e-8 s |

路线 2 的 camchain YAML、IMU YAML 和结果文本与路线 1 逐字节一致。Ceres 的目标
函数定义带 `1/2`，所以必须使用 `2*cost` 与 Kalibr 的 `J` 比较。原生日志只显示
一位小数，Ceres 结果在该打印精度内一致。

## 4. 时间与内存

| 指标 | 1. 原生 Kalibr | 2. 原生优化版 | 3. Ceres 改造版 |
| --- | ---: | ---: | ---: |
| 墙钟时间 | 57.35 s | 54.83 s | 38.04 s |
| 相对原生 | 基准 | 快 4.39% | 快 33.67% |
| 峰值 RSS | 825016 KiB | 835596 KiB | 976380 KiB |
| 相对原生 | 基准 | +1.28% | +18.35% |
| 非线性求解时间 | 未单独记录 | 未在本轮单独记录 | 19.427 s |

这是一轮同机、同输入、串行执行的工程基准，不是多轮统计分布。墙钟时间会受页缓存
和系统调度影响，应优先看数量级和多轮中位数，不应把亚秒差异当成稳定收益。

结论：

- 路线 2 达到了严格数值兼容，当前端到端速度小幅提升；本轮 RSS 高 1.28%，
  尚不能证明整条流水线的峰值内存下降。
- 路线 3 在可接受的数值误差内收敛，并在统一前处理后明显更快；但峰值内存仍比
  原生高约 18%，内存目标尚未完成。
- 后续报告应同时给出端到端时间和求解器时间，并至少重复三轮取中位数。

本轮原始日志位于 `/tmp/kalibr_three_way_20260810`；`/tmp` 内容不是永久产物。

## 5. 双目 + IMU 测试条件

双目测试使用同一个完整 `imu_april.bag`，但 camchain 改为
`data/euroc_cam/baseline_kalibr/cam_stereo/cam_april-camchain.yaml`。它包含：

- `cam0`：`/cam0/image_raw`；
- `cam1`：`/cam1/image_raw`；
- 已由双目标定得到的固定 `T_cn_cnm1`；
- 两台相机分别检测到 1399/1439 帧 AprilGrid；
- 14381 条 IMU 数据；
- 三种模型均有 375703 个原生误差项；`calibrated`、
  `scale-misalignment`、`scale-misalignment-size-effect` 分别有
  14424/14428/14430 个设计变量，Jacobian 分别为
  `780166 x 64888`、`780166 x 64912`、`780166 x 64918`。

本次遵循 Kalibr 默认联合标定用法：双目 baseline 在 IMU 联合阶段保持为输入值，
两台相机的重投影都参与优化，优化 `cam0—IMU` 外参和每台相机的时间偏移；输出的
`cam1—IMU` 由固定 baseline 与 `cam0—IMU` 共同确定。没有使用调试选项
`--recompute-camera-chain-extrinsics`。

## 6. 双目 + IMU 精度

三个模型下，路线 2 的 camchain YAML、IMU YAML 和结果文本都与 ROS1 原生路线
逐字节一致。Ceres 相对原生的最大差值如下，“相机”一列取 cam0/cam1 中较大者：

| IMU 模型 | 原生 J | Ceres `2*cost` | 相机旋转差 | 相机平移差 | 时间偏移差 | IMU 内参最大差 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| calibrated | 171474 | 171474.4812 | 1.33e-4 deg | 3.93e-6 m | 2.53e-7 s | — |
| scale-misalignment | 170184 | 170183.7342 | 3.15e-5 deg | 6.27e-7 m | 6.09e-7 s | 1.06e-5 |
| scale-misalignment-size-effect | 169042 | 169041.7840 | 3.58e-5 deg | 9.88e-7 m | 1.26e-7 s | 9.76e-6 |

原生 `J` 只打印到整数，因此 Ceres 结果都在其日志显示精度内。最终残差均值如下，
括号中为 Ceres：

| IMU 模型 | cam0 px | cam1 px | gyro rad/s | accel m/s² |
| --- | ---: | ---: | ---: | ---: |
| calibrated | 0.455277 (0.455278) | 0.484755 (0.484756) | 0.0175814 (0.0175887) | 0.0781110 (0.0781105) |
| scale-misalignment | 0.456910 (0.456910) | 0.486217 (0.486217) | 0.0142069 (0.0142054) | 0.0794711 (0.0794708) |
| scale-misalignment-size-effect | 0.456263 (0.456263) | 0.485517 (0.485518) | 0.0137465 (0.0137449) | 0.0762231 (0.0762227) |

## 7. 双目 + IMU 时间

| IMU 模型 | 1. 原生 Kalibr | 2. 原生优化版 | 相对原生 | 3. Ceres 版 | 相对原生 | Ceres solve |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| calibrated | 105.15 s | 96.93 s | 快 7.82% | 58.06 s | 快 44.78% | 26.709 s |
| scale-misalignment | 100.92 s | 94.40 s | 快 6.46% | 65.32 s | 快 35.28% | 34.445 s |
| scale-misalignment-size-effect | 104.29 s | 97.85 s | 快 6.18% | 67.92 s | 快 34.87% | 36.960 s |

## 8. 双目 + IMU 峰值内存

| IMU 模型 | 1. 原生 Kalibr | 2. 原生优化版 | 相对原生 | 3. Ceres 版 | 相对原生 |
| --- | ---: | ---: | ---: | ---: | ---: |
| calibrated | 1034864 KiB | 1044220 KiB | +0.90% | 1462772 KiB | +41.35% |
| scale-misalignment | 1136276 KiB | 1146208 KiB | +0.87% | 1495348 KiB | +31.60% |
| scale-misalignment-size-effect | 1202484 KiB | 1211280 KiB | +0.73% | 1503976 KiB | +25.07% |

双目结论与单目一致但更明显：原生优化版严格保持数值结果并获得约 6%--8% 的
端到端提速；Ceres 获得约 35%--45% 的提速，但峰值内存高 25%--41%。当前下一项
优化重点应是减少双相机角点数据、Ceres residual graph 和 AutoDiff/Jacobian 在
求解阶段的同时驻留，而不是继续增加线程数。

双目原始日志位于 `/tmp/kalibr_three_way_stereo_20260810`。
