# Kalibr no-ROS v2 源码深入导读

本文面向希望真正理解本项目，而不只是会运行命令的工程人员。默认读者熟悉
Python、C++、线性代数和最小二乘，但不要求预先掌握 Kalibr。

本文描述的是当前工程源码，而不是一个抽象的“标准 Kalibr”。涉及算法行为时，
以 `src/kalibr` 中的实现为准；`ref/kalibr` 只用于核对 ETHZ 原始行为。

## 0. 怎样最快读懂这份代码

不要从 1,630 个历史文件依次阅读。建议分三遍：

| 阅读层级 | 目标 | 建议时间 | 到达标准 |
| --- | --- | ---: | --- |
| 第一遍 | 看懂输入、阶段和输出 | 30 分钟 | 能解释一次标定从 task YAML 到结果文件的路径 |
| 第二遍 | 跟踪两条标定调用链 | 半天 | 能指出每阶段的状态量、残差和优化器 |
| 第三遍 | 深入数学与求解实现 | 2～3 天 | 能修改模型或性能实现，同时知道如何验证数值不变 |

第一遍只读以下入口：

1. [`src/python/kalibr_no_ros/cli.py`](../src/python/kalibr_no_ros/cli.py)：公共命令入口；
2. [`src/python/kalibr_no_ros/task.py`](../src/python/kalibr_no_ros/task.py)：task v1 到原生参数的适配；
3. [`src/python/kalibr_no_ros/initialization.py`](../src/python/kalibr_no_ros/initialization.py)：可选物理初值的严格校验和规范化；
4. [`kalibr_calibrate_cameras`](../src/kalibr/calibration/kalibr/python/kalibr_calibrate_cameras)：相机标定阶段机；
5. [`kalibr_calibrate_imu_camera`](../src/kalibr/calibration/kalibr/python/kalibr_calibrate_imu_camera)：Camera–IMU 阶段机。

第二遍再读：

- [`CameraIntializers.py`](../src/kalibr/calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py)：单相机、双目和全批量初值；
- [`IncrementalEstimator.cpp`](../src/kalibr/calibration/incremental_calibration/src/core/IncrementalEstimator.cpp)：最终逐 view 增量估计；
- [`IccCalibrator.py`](../src/kalibr/calibration/kalibr/python/kalibr_imu_camera_calibration/IccCalibrator.py)：Camera–IMU 问题建图和最终求解；
- [`IccSensors.py`](../src/kalibr/calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py)：相机与 IMU 的状态量、初值和误差项。
- [`src/python/kalibr_no_ros/observability.py`](../src/python/kalibr_no_ros/observability.py)：最终物理标定块的只读重线性化、秩和零空间报告。

第三遍再进入：

- [`Optimizer2.cpp`](../src/kalibr/optimization/aslam_backend/src/Optimizer2.cpp)；
- [`LevenbergMarquardtTrustRegionPolicy.cpp`](../src/kalibr/optimization/aslam_backend/src/LevenbergMarquardtTrustRegionPolicy.cpp)；
- [`BlockCholeskyLinearSystemSolver.cpp`](../src/kalibr/optimization/aslam_backend/src/BlockCholeskyLinearSystemSolver.cpp)；
- `src/kalibr/camera`、`src/kalibr/trajectory` 和 `src/camera_models` 中的具体模型。

## 1. 坐标系和符号约定

这是阅读标定代码最重要的一章。看到一个 `T` 时，先确定它把哪个坐标系下的
坐标变换到哪个坐标系，不要只根据变量名猜测“外参方向”。

### 1.1 符号表

| 符号 | 含义 | 维度或单位 |
| --- | --- | --- |
| $\{W\}$ | 世界或标定板参考坐标系 | — |
| $\{B\}$ | body 坐标系，单 IMU 时通常与参考 IMU 绑定 | — |
| $\{I\}$ | IMU 测量坐标系 | — |
| $\{C_i\}$ | 第 $i$ 台相机坐标系 | — |
| $\{T\}$ | 标定板坐标系 | — |
| ${}^{A}\mathbf p$ | 点在 $\{A\}$ 中的坐标 | $3\times1$，m |
| ${}^{A}_{B}\mathbf R$ | 把 $\{B\}$ 坐标表示旋转到 $\{A\}$ | $3\times3$ |
| ${}^{A}\mathbf t_B$ | $\{B\}$ 原点在 $\{A\}$ 中的位置 | $3\times1$，m |
| ${}^{A}_{B}\mathbf T$ | 把齐次坐标从 $\{B\}$ 变换到 $\{A\}$ | $4\times4$ |
| $\boldsymbol\pi_i$ | 第 $i$ 台相机的投影函数 | $\mathbb R^3\rightarrow\mathbb R^2$ |
| $\mathbf z_{ij}$ | 相机 $i$ 中角点 $j$ 的观测像素 | $2\times1$，px |
| $\mathbf e_i$ | 一个误差项的残差 | 依误差类型而定 |
| $\mathbf R_i$ | 测量协方差 | 残差维度的方阵 |
| $\Delta\mathbf x$ | 设计变量的最小局部增量 | 参数相关 |
| $\Delta t$ | 相机时间到 IMU 时间的偏移 | s |

### 1.2 齐次变换

本文统一采用左上标和左下标，不使用容易产生方向歧义的箭头别名：

$$
{}^{A}_{B}\mathbf T
=
\begin{bmatrix}
{}^{A}_{B}\mathbf R & {}^{A}\mathbf t_B\\
\mathbf 0^\mathrm T & 1
\end{bmatrix}.
$$

其中：

- ${}^{A}_{B}\mathbf R$ 表示坐标系 $\{B\}$ 的坐标轴在坐标系 $\{A\}$ 下的描述；
- ${}^{A}\mathbf t_B$ 表示坐标系 $\{B\}$ 的原点在坐标系 $\{A\}$ 下的坐标。

同一个点在两个坐标系中的坐标满足：

$$
{}^{A}\mathbf p
=
{}^{A}_{B}\mathbf T\,{}^{B}\mathbf p.
$$

连续坐标变换从右向左作用：

$$
{}^{A}_{C}\mathbf T
=
{}^{A}_{B}\mathbf T
{}^{B}_{C}\mathbf T.
$$

相邻的 $B$ 坐标系匹配，因此结果直接把 $C$ 系坐标变换到 $A$ 系。旋转和平移
分别满足：

$$
{}^{A}_{C}\mathbf R
=
{}^{A}_{B}\mathbf R
{}^{B}_{C}\mathbf R,
$$

$$
{}^{A}\mathbf t_C
=
{}^{A}_{B}\mathbf R\,{}^{B}\mathbf t_C
+
{}^{A}\mathbf t_B.
$$

逆变换为：

$$
{}^{B}_{A}\mathbf T
=
\left({}^{A}_{B}\mathbf T\right)^{-1}
=
\begin{bmatrix}
\left({}^{A}_{B}\mathbf R\right)^\mathrm T &
-\left({}^{A}_{B}\mathbf R\right)^\mathrm T{}^{A}\mathbf t_B\\
\mathbf 0^\mathrm T & 1
\end{bmatrix}.
$$

### 1.3 在 YAML 和源码中验证方向

项目统一结果格式明确写出：

```text
p_target = T_target_source * p_source
```

也就是说，矩阵名中的第一个坐标系是结果表达坐标系，第二个是输入表达坐标系。

Kalibr camchain 中 `cam1.T_cn_cnm1` 是 ${}^{C_1}_{C_0}\mathbf T$：

$$
{}^{C_1}\mathbf p
=
{}^{C_1}_{C_0}\mathbf T\,{}^{C_0}\mathbf p.
$$

`T_cam_imu` 是 ${}^{C}_{I}\mathbf T$：

$$
{}^{C}\mathbf p
=
{}^{C}_{I}\mathbf T\,{}^{I}\mathbf p.
$$

调试外参时，可以拿原点 ${}^{I}\mathbf p=\mathbf0$ 代入。输出应为 IMU 原点在
相机系中的坐标，而不是相机原点在 IMU 系中的坐标。

### 1.4 旋转局部更新

优化器不会直接对九个旋转矩阵元素做无约束加法，而是在最小三维扰动上更新。
以左扰动为示意：

$$
\mathbf R(\Delta\boldsymbol\theta)
=
\operatorname{Exp}\!\left([\Delta\boldsymbol\theta]_\times\right)
\mathbf R,
$$

其中 $[\cdot]_\times$ 把三维向量变为反对称矩阵。实际更新左右侧应以对应
DesignVariable 实现为准；阅读 Jacobian 时不能把两种约定混用。

## 2. 工程代码地图

### 2.1 两套 Kalibr 源码

```text
ref/kalibr                 冻结的 ETHZ 1,630 文件快照
src/kalibr                 可维护的项目源码
  foundation/              Schweizer-Messer、Python/Eigen 基础设施
  camera/                  相机模型、图像、标定板和视觉误差项
  optimization/            Optimizer2、稀疏矩阵和线性求解器
  trajectory/              连续时间 B-spline
  calibration/             相机及 Camera–IMU 标定阶段机
  third_party/             AprilTag
src/python                 无 ROS I/O、公共 CLI、task 和性能插桩
src/camera_models          radtan5、radtan8 与 OpenCV fisheye 扩展
config                     可直接复制的任务与目录数据集模板
tools                      依赖、审计、比较和 benchmark 工具
```

`KALIBR_SOURCE_VARIANT=project|reference` 只选择底层 Kalibr 源码。两种 preset 共用
无 ROS 数据集边界和统一 CLI，所以可以用相同 task 对比算法结果。

### 2.2 总体数据流

```text
task v1 YAML + 可选 initialization schema v1 YAML
  |
  v
kalibr-noros CLI
  |
  +-- 校验任务/初值、解析相对路径、检查输出目录
  +-- 创建临时工作目录和数据集符号链接
  +-- 把 task 转换为原生 Kalibr 参数；初值规范化为内部临时 YAML
  |
  v
原生 Python 阶段机
  |
  +-- ROS1/ROS2 bag 或目录数据集索引、必要的反序列化、图像解码
  +-- 并行标定板检测
  +-- 构造 Boost.Python/C++ 设计变量和误差项
  +-- Optimizer2 或 IncrementalEstimator
  |
  v
原生 camchain/IMU YAML、TXT、PDF
  |
  v
统一 calibration.yaml、results.txt、report.pdf、可选 poses.csv
+ 显式初值运行的 initialization_report.yaml、observability.yaml
```

### 2.3 公共 CLI 没有重写算法

[`task.py`](../src/python/kalibr_no_ros/task.py) 的职责是适配，不是重新实现标定：

1. 严格读取 `schema_version: 1`，并校验 `dataset.type`；
2. 若配置初值，由 `initialization.py` 按 job、相机模型和 IMU 模型严格校验，再生成
   只供内部阶段机读取的规范化临时 YAML；
3. 把 task 字段转换成原生 CLI 参数；
4. 用临时 `sys.argv` 和工作目录执行内部阶段机；
5. 收集原生输出并规范化文件名。

因此要理解数值行为，应继续跟进 `kalibr_calibrate_cameras` 或
`kalibr_calibrate_imu_camera`，而不是停在公共 CLI。

初始化文件的公共接口、坐标方向和 `refine`/`direct` 分阶段差异见
[`INITIALIZATION_ZH.md`](INITIALIZATION_ZH.md)。内部阶段机只接收已经规范化的 seed，
不会自行猜测缺失维数、变换方向或模型兼容性。

## 3. 输入、图像检测和并行边界

### 3.1 数据集索引不是完整解码

[`reader.py`](../src/python/kalibr_bag_io/reader.py) 对 ROS1 和 ROS2 bag 先建立
轻量索引；[`directory.py`](../src/python/kalibr_bag_io/directory.py) 对
`dataset.yaml`、相机时间戳 CSV 和图像路径建立等价索引：

- topic；
- header timestamp；
- record/chunk 定位信息；
- 消息类型。

真正的读取、反序列化和图像解码延迟到取出该帧时发生。Bag 接口支持
`sensor_msgs/Image`、`CompressedImage` 和 `Imu`；目录接口支持 OpenCV 能解码的
图像文件及严格 CSV。两者都不依赖 ROS 运行时，并由
[`factory.py`](../src/python/kalibr_bag_io/factory.py) 自动选择。

### 3.2 图像检测流水线

[`TargetExtractor.py`](../src/kalibr/calibration/kalibr/python/kalibr_common/TargetExtractor.py)
采用多进程而不是 Python 线程：

```text
父进程按索引提交有限数量任务
        |
        v
N个worker：读取/反序列化/解码/AprilGrid检测
        |
        v
结果队列返回(index, observation或error)
        |
        v
父进程按index排序，恢复确定的输入顺序
```

这里有四个重要的软件技巧：

1. 队列容量有限，避免把全部图像和检测结果同时堆在内存；
2. worker 结果先显式序列化，避免 `multiprocessing.Queue` feeder 静默丢失不可
   pickle 对象而导致父进程永久等待；
3. 正常和异常退出都执行有界 join，必要时 terminate/kill；
4. 多进程完成顺序可以变化，但最终 observation 顺序按原始 index 恢复。

`--detector-processes` 只控制标定板检测进程数，不等于优化器线程数。

## 4. 双目相机标定调用链

### 4.1 阶段总览

```text
每个topic读取bag并检测标定板
  -> 每台相机单独初始化内参/畸变
  -> 构建多相机共同观测图
  -> 相邻相机baseline初值LM
  -> 全批量相机refinement LM
  -> 每个候选view逐个addBatch并运行GN
  -> 信息增益/rank决定接受或回滚
  -> 离群角点删除、batch移除和重新加入
  -> YAML/TXT/PDF/poses输出
```

主控制流位于
[`kalibr_calibrate_cameras`](../src/kalibr/calibration/kalibr/python/kalibr_calibrate_cameras)。

显式配置 `calibration.freeze_intrinsics: true` 时，主控制流要求完整相机内参/畸变
seed，并跳过单相机内参 LM。固定状态不是靠各阶段各自记住一个松散开关，而是保存在
每个 `CameraGeometry.freezeIntrinsics` 中；所有阶段调用 `setDvActiveStatus()` 时，策略
闸门都会把 projection 和 distortion 强制设为 inactive。相机对、full-batch、最终
batch 工厂以及异常点删除后的 batch 重建都经过这一入口，因此不会在重建问题时意外
解冻。默认 `false` 时，该入口原样执行调用者请求的 active 状态，保持原生流程。

### 4.2 单相机内参初始化

每台相机先独立运行 `calibrateIntrinsics()`。设计变量包括：

- 投影内参；
- 畸变参数；
- 每张有效标定板图像的 ${}^{C}_{T}\mathbf T$。

每个有效角点形成一个二维重投影残差。原生阶段设置为：

| 设置 | 数值 |
| --- | ---: |
| 优化器 | Optimizer2 + LM |
| 初始 $\lambda$ | 10 |
| `nThreads` | 4，显式覆盖时使用指定值 |
| `convergenceDeltaX` | $10^{-3}$ |
| `convergenceDeltaJ` | 1 |
| `maxIterations` | 200 |

### 4.3 双目 baseline 初值

多相机图选择同时看到足够共同角点的观测对。对相邻相机求解
${}^{C_1}_{C_0}\mathbf T$，同时允许两台相机投影内参和每个 view 的标定板位姿
参与优化，畸变在该阶段保持不活动。

初值来自每帧 PnP 外参关系的中值，然后使用与单相机初始化相同的 LM 阈值，
最多 200 次迭代。

### 4.4 全批量 refinement

`solveFullBatch()` 把以下状态放进同一个问题：

- 所有相机投影和畸变；
- 所有相邻 baseline；
- 所有共同时间点的标定板位姿。

仍使用 $\lambda_0=10$、`convergenceDeltaX=1e-3`、
`convergenceDeltaJ=1`，最大迭代数提高到 250。

### 4.5 最终逐 view 增量估计

最终阶段不是“一次最终 BA”。每个候选时间点构造一个 batch，然后调用：

```text
IncrementalEstimator.addBatch(batch)
```

第 $k$ 个候选 view 到来时，优化问题包含此前所有已接受 batch 和当前 batch。
所以问题不断变大，并被重复线性化与求解：

```text
view 1 -> 优化问题1
view 2 -> 优化问题1+2
view 3 -> 优化问题1+2+3
...
view N -> 优化所有已接受view和当前view
```

该阶段使用 Gauss–Newton、专用稀疏 QR/SVD 线性求解和列归一化。CLI 把最大
迭代数设为 50；`convergenceDeltaX` 和 `convergenceDeltaJ` 保持通用默认值
$10^{-3}$。`--optimizer-threads` 会覆盖线程数，但不把所有 SPQR/BLAS 内部线程
强制限制为相同数字。

### 4.6 信息增益、rank和回滚

设边缘标定参数系统的有效奇异值为 $\sigma_i$。源码保存奇异值二进制对数之和，
新 batch 的信息增益为：

$$
\Delta I
=
\frac{1}{2}
\left(
\sum_i\log_2\sigma_i^{\mathrm{new}}
-
\sum_i\log_2\sigma_i^{\mathrm{old}}
\right).
$$

默认阈值为 $0.2$。batch 被接受需要：

$$
\left(
\Delta I>0.2
\quad\text{或}\quad
\operatorname{rank}_{\mathrm{new}}>\operatorname{rank}_{\mathrm{old}}
\right)
\quad\text{且解有效}.
$$

“解有效”要求没有达到最大迭代数，并且最终代价低于初始代价。拒绝时恢复所有
设计变量、删除当前 batch，并恢复线性求解器状态。

### 4.7 离群点不是单独后处理

当有效 batch 数超过 `minViewOutlier * numCams` 后，程序按每台相机全局
重投影误差标准差构造阈值，逐坐标删除超过四倍标准差的角点。删除角点后会：

1. 移除原 batch；
2. 用保留角点重建 batch；
3. 再次调用 `addBatch()`；
4. 如果重建 batch 被拒绝，则删除整个 view。

所有候选 view 处理完还会再进行一轮全局过滤。因此最终阶段耗时包含多次移除、
重建和重复优化。

### 4.8 EuRoC 为什么只留下158组

当前性能验证日志给出的实际数据为：

| 口径 | 数量 |
| --- | ---: |
| cam0 bag图像 | 1450 |
| cam1 bag图像 | 1449 |
| cam0成功检测 | 1038 |
| cam1成功检测 | 1034 |
| 进入增量流程的候选view | 1038 |
| 增量过程中最多保留 | 238 |
| 增量处理结束保留 | 237 |
| 最终离群清理后使用 | 158组双目观测 |

所以最终 `addBatch/GN` 很慢，不是因为只优化158组，而是因为尝试了1038个候选
view，并在问题增长、拒绝和离群重加过程中进行了大量重复求解。

## 5. 相机投影与重投影残差

### 5.1 从三维点到归一化平面

设相机坐标系中的点为：

$$
{}^{C}\mathbf p
=
\begin{bmatrix}X&Y&Z\end{bmatrix}^{\mathrm T},
\qquad Z>0.
$$

归一化坐标为：

$$
x=\frac{X}{Z},
\qquad
y=\frac{Y}{Z}.
$$

其 Jacobian 为：

$$
\frac{\partial(x,y)}{\partial(X,Y,Z)}
=
\begin{bmatrix}
1/Z & 0 & -X/Z^2\\
0 & 1/Z & -Y/Z^2
\end{bmatrix}.
$$

对应实现位于
[`PinholeProjection.hpp`](../src/kalibr/camera/aslam_cameras/include/aslam/cameras/implementation/PinholeProjection.hpp)。

### 5.2 Radtan4、Radtan5 与 Radtan8

令 $r^2=x^2+y^2$。Radtan5 的径向项为：

$$
d_r
=
k_1r^2+k_2r^4+k_3r^6.
$$

畸变坐标为：

$$
x_d
=
x(1+d_r)+2p_1xy+p_2(r^2+2x^2),
$$

$$
y_d
=
y(1+d_r)+p_1(r^2+2y^2)+2p_2xy.
$$

Radtan4 就是令 $k_3=0$。参数顺序分别为：

```text
radtan4: [k1, k2, p1, p2]
radtan5: [k1, k2, p1, p2, k3]
```

Radtan8 是 OpenCV rational model。令 $s=r^2$，其径向倍率为：

$$
q(s)
=
\frac{1+k_1s+k_2s^2+k_3s^3}
{1+k_4s+k_5s^2+k_6s^3}.
$$

对应的归一化畸变坐标为：

$$
x_d
=
xq(s)+2p_1xy+p_2(s+2x^2),
$$

$$
y_d
=
yq(s)+p_1(s+2y^2)+2p_2xy.
$$

参数顺序严格采用 OpenCV：

```text
radtan8: [k1, k2, p1, p2, k3, k4, k5, k6]
```

设：

$$
N(s)=1+k_1s+k_2s^2+k_3s^3,
\qquad
D(s)=1+k_4s+k_5s^2+k_6s^3,
$$

则输入 Jacobian 中使用的径向导数为：

$$
\frac{\mathrm d q}{\mathrm d s}
=
\frac{N'(s)D(s)-N(s)D'(s)}{D(s)^2}.
$$

优化器对 8 个畸变参数使用解析 Jacobian；反投影通过 Newton 迭代求逆。分母在有效
视场内必须远离零。由于分子与分母参数可能高度相关，Radtan8 需要比 Radtan5 更充分的
边缘覆盖、距离变化和姿态激励。

最终像素为：

$$
u=f_u x_d+c_u,
\qquad
v=f_v y_d+c_v.
$$

### 5.3 Equidistant和OpenCV fisheye

令：

$$
r=\sqrt{x^2+y^2},
\qquad
\theta=\arctan(r).
$$

OpenCV fisheye四参数模型为：

$$
\theta_d
=
\theta
\left(
1+k_1\theta^2+k_2\theta^4+k_3\theta^6+k_4\theta^8
\right),
$$

$$
\begin{bmatrix}x_d\\y_d\end{bmatrix}
=
\begin{cases}
\dfrac{\theta_d}{r}
\begin{bmatrix}x\\y\end{bmatrix}, & r>\epsilon,\\
\begin{bmatrix}x\\y\end{bmatrix}, & r\le\epsilon.
\end{cases}
$$

零skew模型最后仍使用普通四内参。完整OpenCV模型额外优化无量纲
$\alpha$：

$$
u=f_u(x_d+\alpha y_d)+c_u,
\qquad
v=f_vy_d+c_v,
$$

因此内参矩阵中的 $K_{01}=f_u\alpha$。完整投影参数顺序为
`[fu,fv,cu,cv,alpha]`，畸变仍为`[k1,k2,k3,k4]`。

### 5.4 重投影残差

标定板点 ${}^{T}\mathbf p_j$ 投影到相机 $i$：

$$
{}^{C_i}\mathbf p_j
=
{}^{C_i}_{T}\mathbf T\,{}^{T}\mathbf p_j,
$$

$$
\widehat{\mathbf z}_{ij}
=
\boldsymbol\pi_i\!\left({}^{C_i}\mathbf p_j\right).
$$

相机的 `ReprojectionError` 和 `SimpleReprojectionError` 使用“测量减预测”的方向：

$$
\mathbf e_{ij}
=
\mathbf z_{ij}-\widehat{\mathbf z}_{ij}.
$$

而 Camera–IMU 中使用的 `kalibr_errorterms::EuclideanError` 采用“预测减测量”。
两种符号的平方代价相同，但 Jacobian 和残差均值的符号相反，阅读具体误差类时
必须区分，不能假设整个工程只有一种残差方向。

若像素协方差为 $\mathbf R_{ij}$，代价为：

$$
J(\mathbf x)
=
\sum_{i,j}
\mathbf e_{ij}^{\mathrm T}
\mathbf R_{ij}^{-1}
\mathbf e_{ij}.
$$

注意原生 ErrorTerm 返回完整加权平方误差，没有 Ceres 常见的 $1/2$ 系数。

## 6. Camera–IMU 标定调用链

### 6.1 固定的建图设置

当前入口固定使用：

| 设置 | 数值 |
| --- | ---: |
| 位姿和bias spline阶数 | 6 |
| 位姿knots/s | 100 |
| bias knots/s | 50 |
| 位姿运动正则 | 关闭 |
| bias运动正则 | 开启 |
| Blake–Zisserman | 关闭 |
| accel/gyro Huber | 关闭 |
| 默认最终LM最大迭代 | 30 |

### 6.2 时间偏移初值没有使用Optimizer2

相机位姿先拟合为连续时间 spline。程序比较视觉预测角速度模长和陀螺仪测量
模长：

$$
s_v(t)=\left\|\boldsymbol\omega_v(t)\right\|,
\qquad
s_g(t)=\left\|\boldsymbol\omega_m(t)\right\|.
$$

离散互相关为：

$$
c[k]
=
\sum_n s_v[n]s_g[n+k].
$$

最大相关位置给出离散偏移，再乘平均 IMU 采样周期得到时间偏移初值。代码明确
采用：

$$
t_{\mathrm{imu}}
=
t_{\mathrm{cam}}+\Delta t.
$$

这一步主要是 NumPy 互相关和 spline 求值，没有 Optimizer2，也没有线性系统求解。

### 6.3 camera–IMU旋转和gyro bias初值

固定视觉位姿 spline，只优化 camera–IMU旋转和常量陀螺仪bias：

$$
\widehat{\boldsymbol\omega}_I(t)
=
{}^{I}_{C}\mathbf R\,
{}^{C}\boldsymbol\omega_v(t)
+
\mathbf b_g.
$$

该阶段使用 Optimizer2、BlockCholesky 和默认LM初值 $10^{-3}$：

| 设置 | 数值 |
| --- | ---: |
| `nThreads` | 2，显式覆盖时使用指定值 |
| `convergenceDeltaX` | $10^{-4}$ |
| `convergenceDeltaJ` | 1 |
| `maxIterations` | 50 |

重力方向初值由旋转到世界系的负加速度测量平均后归一化得到，长度固定为
$9.80655\,\mathrm{m/s^2}$。

### 6.4 连续时间位姿和bias

代码中的位姿 spline 表示 ${}^{W}_{B}\mathbf T(t)$。相机观测位姿先变换到 body
轨迹，再用六阶 B-spline 拟合。抽象地，对一个欧式分量可写为：

$$
\mathbf s(t)
=
\sum_{j=0}^{K-1}B_{j,6}(t)\mathbf c_j,
$$

其中 $B_{j,6}(t)$ 是六阶基函数，$\mathbf c_j$ 是控制点。导数由基函数导数获得：

$$
\dot{\mathbf s}(t)
=
\sum_j\dot B_{j,6}(t)\mathbf c_j,
\qquad
\ddot{\mathbf s}(t)
=
\sum_j\ddot B_{j,6}(t)\mathbf c_j.
$$

旋转部分不是简单对旋转矩阵元素做欧式插值，而由 `BSplinePose` 在其旋转参数化
上构造连续轨迹。阅读具体导数时从
[`BSplinePose.cpp`](../src/kalibr/trajectory/bsplines/src/BSplinePose.cpp) 开始。

gyro和accelerometer bias各自使用六阶欧式spline。bias运动项约束其一阶导数，
权重由随机游走参数决定。

### 6.5 相机误差项

对相机时间 $t_c$，实际 spline 查询时间为：

$$
t
=
t_c+\Delta t_{\mathrm{prior}}+\delta t,
$$

其中 $\Delta t_{\mathrm{prior}}$ 来自互相关，$\delta t$ 是联合优化中的标量设计变量。

由 spline 得到 ${}^{W}_{B}\mathbf T(t)$，取逆得到 ${}^{B}_{W}\mathbf T(t)$，再与
${}^{C_i}_{B}\mathbf T$ 复合：

$$
{}^{C_i}_{W}\mathbf T(t)
=
{}^{C_i}_{B}\mathbf T
{}^{B}_{W}\mathbf T(t).
$$

标定板点随后通过相机模型形成二维重投影残差。

### 6.6 Calibrated IMU残差

设 spline 给出的世界系线加速度为 ${}^{W}\mathbf a(t)$，重力为
${}^{W}\mathbf g$，body到世界的旋转为 ${}^{W}_{B}\mathbf R(t)$。则：

$$
{}^{B}\mathbf f(t)
=
{}^{B}_{W}\mathbf R(t)
\left({}^{W}\mathbf a(t)-{}^{W}\mathbf g\right).
$$

基础加速度计预测为：

$$
\widehat{\mathbf a}_m(t)
=
{}^{B}\mathbf f(t)+\mathbf b_a(t).
$$

基础陀螺仪预测为：

$$
\widehat{\boldsymbol\omega}_m(t)
=
{}^{B}\boldsymbol\omega(t)+\mathbf b_g(t).
$$

残差均为预测减测量，并按IMU YAML提供的逆协方差加权。

### 6.7 Scale-misalignment模型

该模型增加：

- 加速度计下三角矩阵 $\mathbf M_a$，6个活动参数；
- 陀螺仪下三角矩阵 $\mathbf M_g$，6个活动参数；
- IMU系到gyro系旋转 ${}^{G}_{I}\mathbf R$，3个活动参数；
- g-sensitivity矩阵 $\mathbf A_g$，9个活动参数。

考虑 IMU 相对 body 的旋转 ${}^{I}_{B}\mathbf R$ 和杆臂 ${}^{B}\mathbf r_I$，
IMU位置处的body系比力为：

$$
{}^{B}\mathbf f_I
=
{}^{B}_{W}\mathbf R
\left({}^{W}\mathbf a-{}^{W}\mathbf g\right)
+
{}^{B}\dot{\boldsymbol\omega}\times{}^{B}\mathbf r_I
+
{}^{B}\boldsymbol\omega\times
\left({}^{B}\boldsymbol\omega\times{}^{B}\mathbf r_I\right).
$$

加速度计预测为：

$$
\widehat{\mathbf a}_m
=
\mathbf M_a,{}^{I}_{B}\mathbf R,{}^{B}\mathbf f_I
+
\mathbf b_a.
$$

定义 ${}^{G}_{B}\mathbf R={}^{G}_{I}\mathbf R{}^{I}_{B}\mathbf R$，陀螺仪预测为：

$$
\widehat{\boldsymbol\omega}_m
=
\mathbf M_g,{}^{G}_{B}\mathbf R,{}^{B}\boldsymbol\omega
+
\mathbf A_g,{}^{G}_{B}\mathbf R,{}^{B}\mathbf f_I
+
\mathbf b_g.
$$

这两式直接对应 `IccScaledMisalignedImu.addAccelerometerErrorTerms()` 和
`addGyroscopeErrorTerms()`，矩阵乘法顺序不能随意交换。

### 6.8 Size-effect模型

`scale-misalignment-size-effect`在上述模型上增加加速度计各敏感轴的等效杆臂。
代码中`rx_i`保持固定，`ry_i`和`rz_i`活动；三个轴选择矩阵保持固定：

$$
\mathbf I_x=\operatorname{diag}(1,0,0),
\qquad
\mathbf I_y=\operatorname{diag}(0,1,0),
\qquad
\mathbf I_z=\operatorname{diag}(0,0,1).
$$

每个敏感轴使用自己的杆臂加速度，再由选择矩阵提取对应输出分量。这个模型参数
更多、可观性要求更高，不应只因为残差更低就默认选用。

### 6.9 最终联合LM

最终问题的主要活动状态包括：

- 位姿spline控制点；
- 重力方向；
- gyro和accelerometer bias spline；
- ${}^{C_0}_{I}\mathbf T$；
- 每台相机时间偏移，除非关闭时间标定；
- scale-misalignment内参；
- 相机链baseline默认固定，只有显式重算时活动。

最终求解配置为：

| 设置 | 数值 |
| --- | ---: |
| 优化器 | Optimizer2 + LM |
| 线性求解器 | BlockCholesky/CHOLMOD |
| 初始 $\lambda$ | 10 |
| `convergenceDeltaX` | $10^{-5}$ |
| `convergenceDeltaJ` | $10^{-2}$ |
| 默认最大迭代 | 30 |
| 默认线程 | CPU-1，显式覆盖时使用指定值 |

## 7. 原生优化器数学与实际语义

### 7.1 线性化

将所有白化残差堆叠为 $\mathbf r(\mathbf x)$，在当前状态线性化：

$$
\mathbf r(\mathbf x\boxplus\Delta\mathbf x)
\approx
\mathbf r(\mathbf x)+\mathbf J\Delta\mathbf x.
$$

Gauss–Newton正规方程为：

$$
\mathbf H\Delta\mathbf x
=
-\mathbf g,
\qquad
\mathbf H=\mathbf J^\mathrm T\mathbf J,
\qquad
\mathbf g=\mathbf J^\mathrm T\mathbf r.
$$

源码把线性系统右端保存为与求解器符号约定一致的`rhs`；读代码时应根据
`buildHessian()`和状态更新一起判断正负号，不要只看变量名`b`或`rhs`。

### 7.2 原生LM的平方阻尼

LM conditioner作为增广Jacobian的一部分进入求解器。BlockCholesky中先计算
conditioner的逐元素平方，再加到Hessian对角线，因此实际求解近似为：

$$
\left(\mathbf H+\lambda^2\mathbf I\right)
\Delta\mathbf x
=
-\mathbf g.
$$

这就是为什么不能简单把 Ceres 的初始 trust-region 半径或对角阻尼数字设成相同
的10，就声称逐步复刻了原生 Kalibr。

还应注意当前原生实现的历史语义：求解后恢复Hessian对角线时，代码减去的是
conditioner本身而不是其平方。若目标是与ETHZ结果一致，分析和测试都应以实际
代码路径为准，而不能默默“修正”后继续声称数值等价。

### 7.3 lambda更新和接受/回滚

初始超参数为：

```text
gamma = 3, beta = 2, p = 3, mu = 2
```

收益比为：

$$
\rho
=
\frac{\Delta J}
{\Delta\mathbf x^\mathrm T
\left(\lambda\Delta\mathbf x+\mathbf b\right)}.
$$

注意分母使用 $\lambda$，而实际对角阻尼是 $\lambda^2$；这是源码的历史行为。

若上一步回归，$\mu\leftarrow2\mu$，再令 $\lambda\leftarrow\mu\lambda$；若
$\rho\le0$，则$\mu\leftarrow10\mu$后增大$\lambda$。成功时：

$$
\lambda
\leftarrow
\lambda\max\left(
\frac{1}{3},
1-(2\rho-1)^3
\right),
$$

并令 $\mu=2$。实际代价上升即 $\Delta J<0$ 时，Optimizer2恢复所有设计变量。

### 7.4 停止条件

Optimizer2循环要求两个绝对量同时大于阈值：

$$
\Delta x_{\max}>\epsilon_x
\quad\text{且}\quad
|\Delta J|>\epsilon_J.
$$

所以任意一个绝对阈值满足就停止。这里：

$$
\Delta x_{\max}
=
\max_k|\Delta x_k|.
$$

它是求解向量的最大绝对分量，不是相对参数变化，也不是统一物理单位。不同设计
变量还可能有自己的scaling。因此不能只用迭代次数判断哪种优化器“更收敛”。

### 7.5 Schur complement

将待边缘化变量 $\boldsymbol\psi$ 和标定变量 $\boldsymbol\theta$ 分块：

$$
\begin{bmatrix}
\mathbf H_{\psi\psi} & \mathbf H_{\psi\theta}\\
\mathbf H_{\theta\psi} & \mathbf H_{\theta\theta}
\end{bmatrix}
\begin{bmatrix}
\Delta\boldsymbol\psi\\
\Delta\boldsymbol\theta
\end{bmatrix}
=
-
\begin{bmatrix}
\mathbf g_\psi\\
\mathbf g_\theta
\end{bmatrix}.
$$

消去 $\boldsymbol\psi$ 后：

$$
\mathbf S
=
\mathbf H_{\theta\theta}
-
\mathbf H_{\theta\psi}
\mathbf H_{\psi\psi}^{-1}
\mathbf H_{\psi\theta}.
$$

$\mathbf S$描述消去每帧位姿等辅助变量后，标定参数仍获得的信息。增量估计器对
相应系统做QR/SVD以分析rank、零空间和covariance。

## 8. 复杂的软件工程技巧

### 8.1 冻结reference和差异白名单

[`verify_reference.py`](../tools/verify_reference.py) 校验冻结快照的文件数和哈希；
[`audit_source_delta.py`](../tools/audit_source_delta.py) 将重排后的工程源码映射回
reference，只允许明确列出的修改。

这使“目录大改”和“算法不变”成为可机器检查的约束，而不是口头承诺。

### 8.2 fake-catkin不是ROS运行时

历史包的CMake依赖catkin宏。工程通过`cmake/fake_catkin`提供最小构建兼容层，
同时不发现、不链接ROS库。它解决的是构建描述兼容，不模拟roscore、rosbag或
消息运行时。

### 8.3 用适配器复用原生阶段机

公共CLI没有复制几百行标定逻辑，而是：

- 把公开 task 转换为旧参数；
- 在临时目录中调用原生脚本；
- 用`runpy.run_path()`保持同一Python进程；
- 结束后收集并规范化输出。

优点是算法路径一致；代价是必须严格管理`sys.argv`、当前目录、模块搜索路径和
原生脚本的`SystemExit`。

### 8.4 Boost.Python类型身份

Optimizer2、IncrementalEstimator、DesignVariable和ErrorTerm是C++对象的
Boost.Python包装。若用普通Python proxy替换类或实例，C++参数转换可能不再识别
对象。当前运行时控制在原调用点显式执行`apply_optimizer_threads()`、
`run_optimizer()`和`run_incremental_batch()`，不改变原生对象类型。

### 8.5 编译期开关实现默认零诊断I/O

`project-release`关闭profiling和diagnostic I/O；`project-profile`才生成并启用
`--timing-json`。因此release默认不会因为时钟、PSS/RSS采样和JSON写入改变热路径。

### 8.6 确定性并行Hessian

Camera–IMU最终LM的主要修改位于BlockCholesky系统构建：

1. 保留原error term顺序；
2. 把连续error区间分给固定数量worker；
3. 每个worker写自己的thread-local稀疏Hessian和rhs；
4. worker内部仍调用原始`ErrorTerm::buildHessian()`；
5. 主线程按chunk index固定顺序归并。

这避免并发写共享稀疏矩阵，也使固定线程数下归并顺序稳定。线程数改变时浮点
加法分组仍会改变，所以只承诺数值容差一致，不承诺跨线程数bitwise一致。

代价是每个worker持有局部稀疏结构和rhs，线程越多内存越高；这解释了8线程
通常比4线程占用更多RSS/PSS。

### 8.7 稀疏求解器的职责边界

- Hessian assembly：把每个误差项贡献累积到稀疏块矩阵；
- CHOLMOD：Camera–IMU BlockCholesky的主要分解/求解后端；
- SPQR：相机增量系统的稀疏QR；
- SVD：分析边缘标定系统的rank、零空间和covariance。

`--optimizer-threads=N`不等于这些库的每个内部阶段都严格使用N线程，因此看到
CPU利用率低于N核或4到8扩展不线性，并不直接说明线程参数没有生效。

### 8.8 安装树和私有依赖

工程使用repository-private sysroot准备部分SuiteSparse和Python依赖，通过RPATH
和launcher隔离构建/安装树。构建并行数在CMake Presets中固定为4，防止高并发
C++编译耗尽内存。

### 8.9 安全输出和稳定浮点YAML

公共CLI先拒绝根目录、home、当前工程目录等危险输出目标；已有输出必须显式
`--force`，且不会删除未知文件。浮点YAML用17位有效数字保存，避免转换过程无意
损失double精度。

## 9. EuRoC源码跟踪实验

### 9.1 准备profile构建

构建并行不要超过4：

```bash
cmake --preset project-profile
cmake --build --preset project-profile --parallel 4
```

以下示例使用：

```text
/mnt/q/File/kalibr/data/euroc_cam/performance_project_reference_v2/euroc-camera.yaml
/mnt/q/File/kalibr/data/euroc_cam/performance_project_reference_v2/euroc-imu.yaml
```

### 9.2 跟踪双目相机标定

```bash
build/project-profile/bin/kalibr-noros calibrate cameras \
  --config /mnt/q/File/kalibr/data/euroc_cam/performance_project_reference_v2/euroc-camera.yaml \
  --output-dir /mnt/q/File/kalibr/data/euroc_cam/source-study-camera \
  --detector-processes 4 \
  --optimizer-threads 4 \
  --timing-json timing.json
```

输出目录必须为空或使用`--force`覆盖由本工具管理的结果。建议依次设置断点：

1. `kalibr_no_ros.cli.main`；
2. `kalibr_no_ros.task.run_task`；
3. `kalibr_calibrate_cameras.main`；
4. `CameraIntializers.calibrateIntrinsics`；
5. `CameraIntializers.stereoCalibrate`；
6. `CameraIntializers.solveFullBatch`；
7. `IncrementalEstimator::addBatch`；
8. `Optimizer2::optimize`。

重点观察：

- task如何变成原生参数；
- `ObservationDatabase`怎样按时间组织多相机观测；
- 每个batch包含哪些位姿、相机参数和重投影误差；
- `batchAccepted`、`informationGain`和`rankTheta`如何变化；
- 离群过滤为什么再次调用`addBatch()`。

### 9.3 跟踪Camera–IMU标定

```bash
build/project-profile/bin/kalibr-noros calibrate imu-camera \
  --config /mnt/q/File/kalibr/data/euroc_cam/performance_project_reference_v2/euroc-imu.yaml \
  --output-dir /mnt/q/File/kalibr/data/euroc_cam/source-study-imu \
  --detector-processes 4 \
  --optimizer-threads 4 \
  --timing-json timing.json
```

建议依次设置断点：

1. `IccCalibrator.buildProblem`；
2. `IccCamera.findTimeshiftCameraImuPrior`；
3. `IccCamera.findOrientationPriorCameraToImu`；
4. `IccCamera.addCameraErrorTerms`；
5. `IccScaledMisalignedImu.addAccelerometerErrorTerms`；
6. `IccScaledMisalignedImu.addGyroscopeErrorTerms`；
7. `IccCalibrator.optimize`；
8. `BlockCholeskyLinearSystemSolver::buildSystem`。

重点观察每种DesignVariable是否active、每种ErrorTerm数量、spline有效时间范围和被
跳过的越界IMU测量。

### 9.4 读 timing JSON

`wall_seconds`是现实世界经过的时间；`cpu_seconds`是主进程或被统计执行单元实际
消耗的CPU时间。多进程检测中，各图像detect wall求和可以大于流水线总wall。

- RSS：驻留物理内存，包含共享页，跨进程相加可能重复计算；
- PSS：共享页按共享进程数分摊，更适合比较进程树真实内存压力；
- `lifetime_peak_at_end`：进程生命周期高水位在阶段结束时的采样，不是严格归属于
  该阶段的独立峰值。

### 9.5 数值一致性检查

使用工程工具比较project和reference输出：

```bash
python3 tools/compare_kalibr_outputs.py --help
```

公平比较必须固定：

- 完全相同的bag时间范围、频率、topic和模型；
- 相同的shuffle设置，推荐`shuffle: false`；
- 相同detector进程数和optimizer线程数；
- 相同IMU模型、最大迭代和时间标定开关；
- 相同初始camchain和IMU YAML。

结果判定应同时检查参数、重投影/IMU残差、接受view数和终止原因。不同线程数会
改变浮点归并顺序，不能把字节不一致自动判断成算法不一致。

## 10. 从异常现象反向定位源码

| 现象 | 优先检查 | 典型原因 |
| --- | --- | --- |
| 标定板成功率低 | bag/decode/TargetExtractor | topic、分辨率、曝光、target尺寸 |
| baseline方向或长度异常 | 坐标系和YAML转换 | 变换取逆错误、mm/m混用 |
| 相机增量阶段极慢 | addBatch和离群重加 | 候选view太多、重复求解、SPQR/SVD |
| Camera–IMU初始旋转错误 | 时间互相关和orientation prior | 时间不重叠、转动激励不足 |
| 时间偏移顶到spline边界 | timestamp和padding | 偏移符号错误、初值太远 |
| 加速度残差很大 | 重力、外参、IMU模型 | 重力方向错误、运动不足、矩阵顺序错误 |
| 4到8线程收益很小 | timing中的build/factor/solve | 串行阶段、内存带宽、底层库线程边界 |
| 参数变化小但不停止 | deltaX/deltaJ | 两个绝对条件、参数scaling不同 |
| 达到maxIterations | 初值和可观性 | 并不等价于“已经充分收敛” |

## 11. 推荐的继续阅读顺序

1. 先完整跟踪一次EuRoC相机任务，不进入模板实现；
2. 阅读`CameraIntializers.py`并手写状态量和残差表；
3. 单步进入`IncrementalEstimator::addBatch`，核对接受与回滚；
4. 跟踪一次Camera–IMU `buildProblem()`，统计每类DesignVariable和ErrorTerm；
5. 对照本文公式阅读`IccSensors.py`中的表达式乘法顺序；
6. 阅读`Optimizer2.cpp`和LM policy，手工复算一次lambda更新；
7. 最后阅读稀疏矩阵、SPQR/SVD、CHOLMOD和并行Hessian实现。

每读完一个阶段，建议回答四个问题：

1. 这一阶段优化什么，固定什么？
2. 每个残差的预测量、测量量、坐标系和单位是什么？
3. 线性系统怎样构造，步长怎样接受或回滚？
4. 哪些修改会改变算法，哪些只改变工程边界或执行速度？

能稳定回答这四个问题，就已经从“会使用Kalibr”进入了“能够维护Kalibr”的阶段。
