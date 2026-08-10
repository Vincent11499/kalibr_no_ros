# C++17 与 Ceres 2.2 优化后端

## 1. 实现边界

本仓库固定使用 C++17，并在 `opt/ceres` 分支提供 Ceres 2.2.0 后端。两个标定
命令都新增了 `--optimizer {native,ceres}`，默认仍为 `native`，因此已有脚本不加
参数时不会改变行为。

- `kalibr_calibrate_imu_camera --optimizer ceres`：联合问题的非线性优化由 Ceres
  完整求解，包括位姿/偏置样条、重力、相机—IMU 外参、时间偏移以及三种 IMU
  内参模型。
- `kalibr_calibrate_cameras --optimizer ceres`：保留 Kalibr 的相机初始化、增量
  view acceptance、`miTol`、QR 条件检查、rollback 和 outlier/batch rejection；这些
  步骤结束后，用 Ceres 对最终被接受的视图执行一次全局 BA。它目前不是完整替换
  Kalibr 的增量相机优化器。

上游快照 `upstream/kalibr` 没有修改。Ceres 实现位于
`extensions/ceres_optimizer`，CLI 接线由顶层 `CMakeLists.txt` 在生成安装脚本时完成。

## 2. 为什么不能只替换线性求解器

Kalibr 的 `ErrorTerm`、表达式节点和样条对象含共享缓存，直接让 Ceres 多线程调用
这些对象存在数据竞争。新后端因此重新构造线程安全的问题：观测和样条基函数只读，
每个状态使用独立参数块，残差直接从参数块计算，不在工作线程调用 Kalibr 表达式图。

这也解释了当前相机路径为什么先保留原生增量阶段：`miTol`、QR 可观性判断、视图
回滚和剔除时机属于数据选择策略，不只是一个求解器调用。最终 BA 已能安全并行，
若要获得相机全流程加速，还需把这套增量状态机迁移到 Ceres。

## 3. 依赖与构建

| 组件 | 固定版本/要求 |
| --- | --- |
| C++ | C++17 |
| Ceres Solver | 2.2.0 |
| Abseil | 20240116.2 |
| CMake | >= 3.16 |
| Eigen | >= 3.3.4；当前验证为 3.3.7 |
| 稀疏后端 | SuiteSparse |

依赖安装在仓库的 `.deps/install`，不会覆盖系统 Ceres。完整命令：

```bash
./scripts/bootstrap_ceres_2_2.sh

env -u CMAKE_PREFIX_PATH -u ROS_DISTRO -u ROS_ROOT \
  -u ROS_PACKAGE_PATH -u PYTHONPATH \
  cmake -S . -B build/ceres \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PWD/install/ceres" \
    -DCMAKE_PREFIX_PATH="$PWD/.deps/install" \
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF \
    -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF \
    -DKALIBR_ENABLE_CERES=ON \
    -DKALIBR_ENABLE_TESTING=OFF

cmake --build build/ceres --parallel 2
cmake --install build/ceres
cd build/ceres
ctest --output-on-failure
```

当前 8 GiB 主机建议 `--parallel 2`；`--parallel 4 --clean-first` 曾因大型
Boost.Python 编译单元触发 OOM。安装树自带 Ceres 运行库和分层 RPATH，不需要设置
`LD_LIBRARY_PATH`。

## 4. IMU—相机联合问题

### 4.1 状态与残差

`ImuProblem.cpp`/`DynamicSpline.cpp`/`SplineMotionCost.cpp` 构建以下状态：

- 六阶位姿 B-spline，100 knots/s；
- 陀螺仪和加速度计偏置样条，50 knots/s；
- 重力向量；
- 每个相机的外参和 scalar 时间偏移参数块；
- `calibrated`、`scale-misalignment` 或
  `scale-misalignment-size-effect` 的 IMU 内参块。

相机残差使用 Kalibr 相同的针孔 radtan4/radtan5 投影和观测协方差。IMU 残差使用
相同的旋转约定、角速度、重力/线加速度、偏置与三种内参修正公式。偏置随机游走
直接使用 Kalibr `BSpline::segmentIntegral()` 生成的矩阵，不使用数值 quadrature。

CLI 默认没有 gyro/accel Huber loss，因此先把同一时刻的 3 维陀螺仪和 3 维
加速度计合并为 6 维 residual；再把共享同一组位姿/偏置样条控制点的相邻 IMU
样本合并到同一个 block。200 Hz IMU 与 100 Hz 位姿样条通常能合并约两个样本。
如果启用任一独立 Huber，则恢复为每个测量、每个传感器各自的 block，以保持
Kalibr 独立 robust loss 的语义。相机按“每帧每相机”批量组织所有角点。

时间偏移遵循 Kalibr 的初始化策略：建图时保持固定，进入联合求解后在允许范围内
释放；`--no-time-calibration` 时始终固定。相机链外参默认固定，与原命令一致，只有
`--recompute-camera-chain-extrinsics` 时才释放。

### 4.2 求解配置

- 信赖域：Levenberg–Marquardt；
- 线性求解器：`SPARSE_NORMAL_CHOLESKY`；
- 稀疏库：SuiteSparse；
- 默认线程：逻辑 CPU 数减 1，至少 1；
- 可用 `--ceres-threads N` 显式限制线程；`0` 表示上述自动选择；
- CLI 的 `--max-iter`、`--timeoffset-padding` 和三种 IMU model 直接传入 Ceres。

Kalibr 报告的目标函数 `J` 是平方残差和；Ceres 的 `cost` 定义为其一半，因此比较
时应使用 `J = 2 * cost`。

### 4.3 EuRoC 10 秒窗口对比

统一输入：`imu_april.bag`、`april_6x6.yaml`、已有单目 camchain，窗口 15--25 s，
最大 30 次迭代。两端初始重投影/陀螺仪/加速度计均值分别为
`1.24655355 px`、`0.08593912 rad/s`、`0.62565242 m/s²`。

| IMU 模型 | Kalibr J | Ceres 2*cost | 旋转差 | 平移差 | 时间偏移差 |
| --- | ---: | ---: | ---: | ---: | ---: |
| calibrated | 4183.71 | 4183.70538 | 2.56e-5 deg | 2.79e-7 m | 5.99e-8 s |
| scale-misalignment | 4118.33 | 4118.3308 | 1.85e-4 deg | 1.39e-5 m | 5.12e-7 s |
| scale-misalignment-size-effect | 4105.92 | 4105.91452 | 5.69e-4 deg | 4.57e-6 m | 4.17e-8 s |

三类模型的最终重投影均约 `0.34 px`，gyro 均约 `0.008 rad/s`，accel 均约
`0.035--0.038 m/s²`。两类 IMU 内参模型相对原生结果的最大绝对参数差分别为
`4.89e-5` 和 `1.12e-5`。

第一层 gyro/accel 合并将 Ceres solve 从 5.102 s 降到 3.621 s；第二层相邻时间
样本合并把 2001 个 IMU 测量进一步压缩为 1050 个 block，solve 降至 2.821 s。
相对最初实现总计减少约 44.7%。峰值 RSS 从 430716 降到 382608 KiB，端到端从
13.24 降到 11.72 s，解不变。

### 4.4 EuRoC 完整 bag 对比

三组测试都使用完全相同的 `output_cam1/cam_april-camchain.yaml`、完整
`imu_april.bag` 和 30 次最大迭代。Ceres 自动使用 15 个线程；14381 个 IMU 测量
在三组测试中都合并为 7822 个 residual block。

| IMU 模型 | Kalibr J | Ceres `2*cost` | 旋转差 | 平移差 | 时间偏移差 | IMU 内参最大元素差 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| calibrated | 43975.1 | 43975.1258 | 6.85e-5 deg | 1.37e-6 m | 9.09e-8 s | — |
| scale-misalignment | 43761.5 | 43761.5150 | 3.10e-4 deg | 3.45e-6 m | 3.14e-7 s | 1.53e-5 |
| scale-misalignment-size-effect | 43348.5 | 43348.4696 | 1.27e-5 deg | 9.47e-7 m | 4.57e-8 s | 6.02e-6 |

原生日志只把 `J` 打印到小数点后一位；上表三组 `2*cost` 都落在该打印精度内。
最终残差均值如下，括号中是 Ceres：

| IMU 模型 | 重投影 px | gyro rad/s | accel m/s² |
| --- | ---: | ---: | ---: |
| calibrated | 0.40683899 (0.40683933) | 0.00778540 (0.00778499) | 0.04974628 (0.04974574) |
| scale-misalignment | 0.40638815 (0.40638962) | 0.00653120 (0.00653010) | 0.04880608 (0.04880416) |
| scale-misalignment-size-effect | 0.40535033 (0.40535030) | 0.00626397 (0.00626319) | 0.04636805 (0.04636805) |

| IMU 模型 | Native 墙钟 | Ceres 墙钟 | Ceres solve | 加速 | Native RSS | Ceres RSS | RSS 变化 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| calibrated | 57.35 s | 38.04 s | 19.427 s | 33.7% | 825016 KiB | 976380 KiB | +18.3% |
| scale-misalignment | 89.97 s | 75.82 s | 23.835 s | 15.7% | 920040 KiB | 993236 KiB | +8.0% |
| scale-misalignment-size-effect | 93.01 s | 78.08 s | 26.130 s | 16.1% | 984828 KiB | 1002392 KiB | +1.8% |

三类模型都满足“同一残差原理、目标函数收敛、解与原生接近”的基线要求。桥接层
已经在求解前释放临时 C++ 观测数组，但 Python 输入矩阵、Ceres 图和 AutoDiff 状态
仍同时存在，因此速度目标已经达到，内存目标尚未完全达到。

上表每一行都是对应模型在同一批次内的成对测试；不同模型所在批次的机器负载和
页缓存状态不同，不应横向比较绝对秒数。包含“原生、原生优化版、Ceres 改造版”
三条路线的统一 calibrated 对比见 `THREE_WAY_BENCHMARK_CN.md`。
该文档同时包含双目 + IMU、三种 IMU 内参模型的完整九组测试；双目 Ceres
端到端快约 35%--45%，但峰值 RSS 高约 25%--41%。

批处理前曾做 4 线程控制实验：RSS 仅从 981396 降至 972936 KiB，端到端却从
76.83 增至 100.57 s。这说明主要额外内存不是线程工作区，后续仍应优先减少数据和
图结构的重复存储。`--ceres-threads` 保留用于资源受限机器，但不作为默认省内存方案。

## 5. 相机 BA

`CameraBundleProblem.cpp` 优化每个相机的投影/畸变参数、相邻相机 baseline 和每个
被接受视图的 `T_c0_target`。radtan4 参数块固定未使用的 `k3` 槽；radtan5 优化
OpenCV 顺序 `[k1,k2,p1,p2,k3]`。求解器使用 `SPARSE_SCHUR` 和多线程。

每次求解前后都会用 Kalibr 原生 `ErrorTerm::evaluateError()` 复核 Ceres 目标函数。
两条浮点路径在四元数转矩阵等最后运算上略有差异，EuRoC 实测初始/最终相对差为
`1.151e-5`/`1.107e-5`；运行时允许上限为 `1e-4`，超过即中止而不是继续输出结果。

15--18 s 的确定性窗口结果：

| 场景 | 原生与 Ceres 行为 |
| --- | --- |
| 单目 radtan4 | 都接受 40/61 视图；Ceres 初始点已收敛，0 次有效迭代；YAML 最大差 6.14e-12 |
| 双目 radtan4 | 都接受 60/61 视图并剔除 65 个角点；Ceres 7 次迭代，cost 80.19258 -> 78.45061，15 线程约 0.03--0.04 s |

双目 Ceres 找到更低的同一重投影目标，因此参数不会被强制复制成原生结果。最终
标准差为 cam0 `[0.070844,0.067929] px`、cam1 `[0.069523,0.067805] px`；原生
分别约 `[0.072642,0.068486] px`、`[0.070467,0.067566] px`。baseline 平移从
原生 `[-0.11077854,0.00142952,-0.00475299] m` 变为
`[-0.11075958,0.00123034,-0.00404181] m`。

当前相机端到端耗时与原生基本相同（约 27.4 s），因为原生增量流程仍完整执行，
Ceres 只增加约 0.04 s 的最终 BA。这里证明的是残差一致、并行求解可用且收敛，
不是相机全链路性能提升。

## 6. 使用方式

```bash
# 相机：原生增量筛选 + Ceres 最终 BA
./install/ceres/bin/kalibr_calibrate_cameras \
  --target /path/to/aprilgrid.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw \
  --bag /path/to/camera.bag \
  --optimizer ceres --ceres-threads 0

# IMU—相机：Ceres 联合优化
./install/ceres/bin/kalibr_calibrate_imu_camera \
  --target /path/to/aprilgrid.yaml \
  --imu /path/to/imu.yaml \
  --imu-models scale-misalignment-size-effect \
  --cams /path/to/camchain.yaml \
  --bag /path/to/imu_camera.bag \
  --optimizer ceres --ceres-threads 0
```

ROS1 `.bag` 与 ROS2 rosbag2 目录使用同一数据接口；优化核心不感知 bag 类型。

## 7. 已知限制

- IMU—相机 Ceres 路径当前只支持一个 IMU。
- 相机 Ceres 残差当前只支持 pinhole-radtan4/radtan5。
- 相机 Ceres 路径不支持 `--use-blakezisserman`，会明确报错。
- 相机最终打印的协方差仍来自 Ceres refinement 之前的 Kalibr 原生 estimator，
  不能解释为 Ceres 最终解的协方差。
- IMU Ceres 路径尚未恢复 Kalibr `--recover-covariance` 的完整协方差输出。
- 相机初始化、增量 view acceptance、`miTol`、QR、rollback 和 rejection 仍是原生
  单线程路径；要提升相机端到端性能，需要继续迁移这部分状态机。
- Ceres 完整 bag 已覆盖三种 IMU 内参模型；它们都收敛到与原生 Kalibr 极接近的
  解，但峰值 RSS 仍分别比原生高约 17.4%、8.0% 和 1.8%。
