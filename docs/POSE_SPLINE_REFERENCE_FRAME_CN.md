# Kalibr 位姿样条到底描述谁：参考 IMU/body 轨迹的选择、原理与源码实现

本文集中回答以下问题：

1. Kalibr 对谁的位姿进行样条化？
2. 为什么选择参考 IMU/body 的位姿，而不是相机、AprilGrid 或每个传感器各自的位姿？
3. 相机观测怎样初始化 IMU/body 位姿样条？
4. 相机、陀螺仪和加速度计怎样共同使用并优化这条样条？
5. 原生 Kalibr 和当前 Ceres 后端分别怎样实现？

公式使用 GitHub Markdown 支持的格式：行内公式使用 `$...$`，独立公式使用 `$$...$$`。可直接在 GitHub、支持 MathJax/KaTeX 的 Markdown 预览器中查看。

---

## 1. 先给出结论

Kalibr 样条化的是：

> 参考 IMU，也就是 body/imu0，相对于固定标定板世界坐标系的连续时间位姿。

记为：

$$
\mathbf T_{wb}(t)
=
\begin{bmatrix}
\mathbf C_{wb}(t) & \mathbf p_{wb}(t)\\
\mathbf 0^T & 1
\end{bmatrix}.
$$

单 IMU 时 body 与 imu0 重合，因此也可以写成：

$$
\mathbf T_{wi}(t)=\mathbf T_{wb}(t).
$$

其中：

- $w$：由固定 AprilGrid 定义的世界/标定板坐标系；
- $b$：传感器系统的 body 坐标系；
- $i$：参考 IMU，即 imu0；
- $\mathbf C_{wb}(t)$：body 到世界的旋转；
- $\mathbf p_{wb}(t)$：body 原点在世界坐标系中的位置。

原生源码已经直接注明：

```python
# initialize a pose spline using camera poses (pose spline = T_wb)
```

并且在注册优化变量时再次注明：

```python
# Initialize the system pose spline (always attached to imu0)
```

所以需要区分两个概念：

- **样条初始数据来自相机**；
- **样条最终描述的是参考 IMU/body 的位姿**。

它不是相机独立轨迹，也不是标定板的运动轨迹。

直接对应的源码入口：

- 原生轨迹初始化和相机/IMU residual：[IccSensors.py](../upstream/kalibr/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py)；
- 原生联合问题和 pose DV 注册：[IccCalibrator.py](../upstream/kalibr/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccCalibrator.py)；
- 原生 rotation-vector 运动学：[BSplinePose.cpp](../upstream/kalibr/aslam_nonparametric_estimation/bsplines/src/BSplinePose.cpp)；
- Ceres 初始化桥接：[integration.py](../extensions/ceres_optimizer/python/kalibr_ceres_optimizer/integration.py)；
- Ceres 问题构造：[ImuProblem.cpp](../extensions/ceres_optimizer/src/ImuProblem.cpp)；
- Ceres 动态时间样条：[DynamicSpline.hpp](../extensions/ceres_optimizer/include/kalibr/ceres_optimizer/DynamicSpline.hpp)；
- Ceres 相机 residual：[CameraResidual.hpp](../extensions/ceres_optimizer/include/kalibr/ceres_optimizer/CameraResidual.hpp)；
- Ceres IMU residual：[ImuResidual.hpp](../extensions/ceres_optimizer/include/kalibr/ceres_optimizer/ImuResidual.hpp)。

---

## 2. 坐标变换约定

本文使用：

$$
\mathbf T_{ab}: b\rightarrow a,
$$

即把 $b$ 坐标系中的点变换到 $a$ 坐标系：

$$
\mathbf p_a
=\mathbf C_{ab}\mathbf p_b+\mathbf t_{ab}.
$$

常用变换为：

| 变换 | 含义 |
|---|---|
| $\mathbf T_{wb}(t)$ | body/参考 IMU 到世界，随时间变化 |
| $\mathbf T_{bw}(t)=\mathbf T_{wb}(t)^{-1}$ | 世界到 body，随时间变化 |
| $\mathbf T_{c_0b}$ | body/IMU 到 cam0，待标定的常量外参 |
| $\mathbf T_{c_nc_0}$ | cam0 到 camN，由相机链标定给出的常量基线 |
| $\mathbf T_{c_nb}$ | body 到 camN，由外参链组合得到 |

对于一个固定在标定板上的三维点 $\mathbf P_w$，它在 camN 中的位置是：

$$
\mathbf P_{c_n}(t)
=\mathbf T_{c_nc_0}
\mathbf T_{c_0b}
\mathbf T_{bw}(t)
\mathbf P_w.
$$

其中只有 $\mathbf T_{wb}(t)$ 是随时间变化的运动状态；传感器之间的外参是刚性安装产生的常量。

---

## 3. 位姿样条内部保存什么

Kalibr 的 `BSplinePose` 将位姿写成一条 6 维欧氏曲线：

$$
\mathbf x(t)
=
\begin{bmatrix}
\mathbf p_{wb}(t)\\
\mathbf r_{wb}(t)
\end{bmatrix}
\in\mathbb R^6,
$$

其中 $\mathbf r_{wb}(t)$ 是 Kalibr rotation-vector 参数。

对阶数为 $k$ 的 B-spline：

$$
\mathbf x(t)=\sum_{j=0}^{k-1}B_j(t)\mathbf c_j,
$$

$\mathbf c_j$ 是当前时间附近的局部控制点。当前 IMU－相机 CLI 使用：

```text
splineOrder = 6
poseKnotsPerSecond = 100
```

因此一个时间点只与附近 6 个位姿控制点相连。

由同一条曲线可以获得：

$$
\mathbf p_{wb}(t),\qquad
\dot{\mathbf p}_{wb}(t),\qquad
\ddot{\mathbf p}_{wb}(t),
$$

以及：

$$
\mathbf r_{wb}(t),\qquad
\dot{\mathbf r}_{wb}(t),\qquad
\ddot{\mathbf r}_{wb}(t).
$$

这些量分别服务于：

| 样条量 | 使用者 |
|---|---|
| 位姿 $\mathbf T_{wb}(t)$ | 相机重投影 |
| 姿态一阶导/角速度 | 陀螺仪 |
| 位置二阶导 | 加速度计 |
| 姿态二阶导 | size-effect 旋转加速度项 |

---

## 4. 为什么选择参考 IMU/body 的位姿

### 4.1 刚性系统实际上只有一条运动轨迹

IMU、cam0、cam1 固定安装在同一个刚体上。它们的位姿满足：

$$
\mathbf T_{wc_n}(t)
=\mathbf T_{wb}(t)\mathbf T_{bc_n},
$$

或者按本文前述方向，通过逆变换写成相机投影链。

只要选择任意一个刚体坐标系作为参考，再加上传感器间常量外参，就能得到全部传感器的运动。理论上可以选择相机或 IMU，但不能给每个传感器建立互相独立的自由轨迹，否则：

- 重复表达同一个刚体运动；
- 引入额外 gauge freedom；
- 不再自动满足刚性约束；
- 相机轨迹和 IMU 轨迹可能分别拟合自己的数据，却无法唯一确定外参。

因此首先应当选择一条公共 body 轨迹。

### 4.2 IMU 测量天然是位姿的一、二阶导数

陀螺仪直接约束 body 姿态的时间导数：

$$
\tilde{\boldsymbol\omega}(t)
=\boldsymbol\omega_b(t)+\mathbf b_g(t)+\mathbf n_g(t).
$$

加速度计约束 body 原点的二阶运动：

$$
\tilde{\mathbf a}(t)
=\mathbf C_{wb}(t)^T
\left(
\ddot{\mathbf p}_{wb}(t)-\mathbf g_w
\right)
+\mathbf b_a(t)+\mathbf n_a(t).
$$

当样条直接绑定参考 IMU 时：

- gyro 可直接使用样条姿态导数；
- accel 可直接使用样条位置二阶导；
- 参考 IMU 的旋转外参为单位阵；
- 参考 IMU 的杆臂为零；
- IMU residual 最简单，参数耦合最少。

这就是选择参考 IMU/body 最重要的原因。

### 4.3 如果选择相机位姿作为公共样条会怎样

从数学上说，可以选择 cam0 位姿 $\mathbf T_{wc_0}(t)$ 作为轨迹。但陀螺仪和加速度计预测必须先转换到 IMU：

$$
\mathbf T_{wi}(t)
=\mathbf T_{wc_0}(t)\mathbf T_{c_0i}.
$$

外参 $\mathbf T_{c_0i}$ 本身又是待估变量。于是 IMU 预测中的：

- 姿态；
- 角速度；
- 线加速度；
- 旋转杆臂加速度；

都会更直接地依赖未知外参。

尤其加速度计不只需要旋转转换。若相机原点和 IMU 原点相距 $\mathbf r$，还必须加入：

$$
\dot{\boldsymbol\omega}\times\mathbf r
+\boldsymbol\omega\times
(\boldsymbol\omega\times\mathbf r).
$$

这会让轨迹导数和待标定外参平移更强耦合。选择 IMU 原点作为轨迹原点，可以使参考 IMU 的 $\mathbf r=\mathbf0$，避免这组不必要的转换。

所以“相机轨迹样条”不是错误的数学模型，但对于以 IMU 模型为核心的联合标定，它不是最自然、最简洁的状态定义。

### 4.4 为什么不对 AprilGrid 位姿做样条

标准 Kalibr 采集方式是：

- AprilGrid 固定；
- 相机和 IMU 组成的刚体运动。

因此直接把 AprilGrid 坐标系定义为世界系：

$$
\mathbf T_{w\,	ext{target}}=\mathbf I.
$$

标定板不运动，自然没有必要给它建立时间样条。这样还固定了整个问题的世界坐标 gauge。

如果反过来把传感器固定、移动标定板，理论上可以对标定板运动建模，但那已经是另一种实验和状态定义。标准 Kalibr 代码不是按这种场景设计的。

### 4.5 为什么不直接对速度或 IMU 测量做样条

如果只拟合角速度和线加速度：

- 还需要积分才能得到相机重投影所需的位姿；
- 两次积分会引入初值和漂移状态；
- 时间偏移改变后需要重新保持积分一致性；
- 相机位姿约束与 IMU 状态之间的关系更复杂。

位姿 B-spline 可以同时提供位姿、一阶导和二阶导，使相机与 IMU 在同一个连续状态上建立 residual，避免额外的数值积分轨迹。

### 4.6 为什么 body 绑定 imu0，而不是多个 IMU 的平均坐标系

原生 Kalibr 支持多 IMU，但将系统 pose spline 固定绑定到 imu0。其余 IMU 通过：

$$
\mathbf T_{i_mb}
$$

以及时间偏移连接到 body 轨迹。

如果使用一个人为定义的“平均 IMU 坐标系”：

- 它没有直接测量；
- 每个 IMU 都需要额外外参和杆臂；
- 需要另外固定坐标 gauge；
- 不会增加任何观测信息。

选择真实存在的 imu0 作为参考，可以固定 body 定义，并使至少一个 IMU 的外参严格为单位变换。

---

## 5. 相机怎样初始化参考 IMU 位姿样条

### 5.1 第一步：相机观测给出离散相机位姿

AprilGrid 的三维角点坐标已知，相机内参也由相机标定给出。每张有效图像通过角点得到相机相对标定板的位姿：

$$
\mathbf T_{wc}(t_k).
$$

这些是离散时刻 $t_k$ 的相机位姿，不是最终样条状态。

### 5.2 第二步：用外参初值转换成 body 位姿

给定当前 body 到相机外参初值 $\mathbf T_{cb}$，可得：

$$
\mathbf T_{wb}(t_k)
=\mathbf T_{wc}(t_k)\mathbf T_{cb}.
$$

原生 Kalibr 对应代码：

```python
T_c_b = self.T_extrinsic.T()

curve = np.matrix([
    pose.transformationToCurveValue(
        np.dot(obs.T_t_c().T(), T_c_b)
    )
    for obs in self.targetObservations
]).T
```

其中 `transformationToCurveValue()` 将每个 $\mathbf T_{wb}(t_k)$ 转成：

$$
\begin{bmatrix}
\mathbf p_{wb}(t_k)\\
\mathbf r_{wb}(t_k)
\end{bmatrix}.
$$

源码位置：

`upstream/kalibr/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py`

函数：

```text
IccCamera.initPoseSplineFromCamera()
```

### 5.3 第三步：统一相机和 IMU 时间

初始化样条使用：

$$
t_k=t_{c,k}+\Delta t_{c\rightarrow i}^{\text{prior}}.
$$

源码：

```python
times = np.array([
    obs.time().toSec() + self.timeshiftCamToImuPrior
    for obs in self.targetObservations
])
```

这样初始化的 body spline 已经大致处于 IMU 时间轴上。最终的连续时间偏移修正仍由联合优化估计。

### 5.4 第四步：rotation vector 连续化

同一个三维旋转可以写成：

$$
\mathbf r_s=\mathbf a(\theta+2\pi s).
$$

如果相邻帧分别选择了不同的 $2\pi$ 分支，rotation vector 会突然跳变，样条导数会产生假的巨大角速度。

Kalibr 在 $s\in[-3,3]$ 中寻找与前一个 rotation vector 最近的表示，再进行样条拟合。

### 5.5 第五步：添加时间边界并拟合

为了允许后续时间偏移变化，Kalibr 在首尾添加：

$$
2\times\texttt{timeOffsetPadding}
$$

的边界样本，然后计算控制点数量：

$$
N_{knots}
=\operatorname{round}
\left(
(t_{end}-t_{start})
\cdot\texttt{poseKnotsPerSecond}
\right).
$$

最后执行：

```python
pose.initPoseSplineSparse(times, curve, knots, 1e-4)
```

这里 `1e-4` 是初始样条稀疏拟合参数。

---

## 6. 初始化以后，相机怎样使用这条样条

对一帧相机观测，其 IMU 时间为：

$$
t=t_c+\Delta t^{prior}+\delta t.
$$

在该时间查询：

$$
\mathbf T_{wb}(t),
$$

取逆：

$$
\mathbf T_{bw}(t)=\mathbf T_{wb}(t)^{-1},
$$

再组合相机外参：

$$
\mathbf T_{c_nw}(t)
=\mathbf T_{c_nb}\mathbf T_{bw}(t).
$$

原生代码：

```python
frameTime = (
    self.cameraTimeToImuTimeDv.toExpression()
    + obs.time().toSec()
    + self.timeshiftCamToImuPrior
)

T_w_b = poseSplineDv.transformationAtTime(
    frameTime, timeOffsetPadding, timeOffsetPadding
)
T_b_w = T_w_b.inverse()
T_c_w = T_cN_b * T_b_w
```

所以相机 residual 并没有使用一条“相机样条”。它始终查询 body spline，再通过常量/待估外参得到相机位姿。

---

## 7. IMU 怎样使用这条样条

### 7.1 陀螺仪

Kalibr 从位姿样条旋转部分的一阶导得到 body-frame 角速度：

$$
\boldsymbol\omega_b(t)
=-\mathbf C_{wb}(t)^T
\mathbf S(\mathbf r(t))
\dot{\mathbf r}(t).
$$

参考 IMU 的 calibrated 模型预测：

$$
\hat{\boldsymbol\omega}(t)
=\boldsymbol\omega_b(t)+\mathbf b_g(t).
$$

对应 residual：

$$
\mathbf r_g(t)
=\mathbf L_g
\left(
\hat{\boldsymbol\omega}(t)
-\tilde{\boldsymbol\omega}(t)
\right).
$$

原生实现调用：

```python
w_b = poseSplineDv.angularVelocityBodyFrame(tk)
```

### 7.2 加速度计

位姿样条平移部分的二阶导是 body 原点的世界系加速度：

$$
\mathbf a_w(t)=\ddot{\mathbf p}_{wb}(t).
$$

转换成参考 IMU 测量的比力：

$$
\mathbf f_b(t)
=\mathbf C_{wb}(t)^T
\left(
\ddot{\mathbf p}_{wb}(t)-\mathbf g_w
\right).
$$

预测和 residual 为：

$$
\hat{\mathbf a}(t)
=\mathbf f_b(t)+\mathbf b_a(t),
$$

$$
\mathbf r_a(t)
=\mathbf L_a
\left(
\hat{\mathbf a}(t)-\tilde{\mathbf a}(t)
\right).
$$

原生实现调用：

```python
C_b_w = poseSplineDv.orientation(tk).inverse()
a_w = poseSplineDv.linearAcceleration(tk)
```

### 7.3 为什么一条样条能够连接三种传感器

```mermaid
flowchart TD
    S[参考 IMU/body 位姿样条 T_wb(t)]
    S --> P[位姿值 T_wb(t)]
    S --> W[旋转一阶导 omega_b(t)]
    S --> A[平移二阶导 a_w(t)]
    P --> C[相机重投影 residual]
    W --> G[gyro residual]
    A --> I[accel residual]
    P --> I
    E[IMU-cam 外参] --> C
    D[相机时间偏移] --> C
    R[重力方向] --> I
```

相机约束位姿，gyro 约束姿态变化率，accel 约束位置二阶变化和重力方向。它们通过共享控制点真正形成联合优化，而不是三个独立结果在优化结束后才做对齐。

---

## 8. 双目情况下仍然只有一条位姿样条

cam0 使用：

$$
\mathbf T_{c_0w}(t)
=\mathbf T_{c_0b}\mathbf T_{bw}(t).
$$

cam1 使用：

$$
\mathbf T_{c_1w}(t)
=\mathbf T_{c_1c_0}
\mathbf T_{c_0b}
\mathbf T_{bw}(t).
$$

相机数量增加的只是：

- 每个相机的观测 residual；
- 相机链基线；
- 每个相机的时间偏移变量。

不会增加新的系统运动样条。所有相机和参考 IMU 共享 $\mathbf T_{wb}(t)$。

在当前 Ceres 联合路径中，cam0 到其他相机的基线默认固定，联合优化的是公共 $\mathbf T_{c_0b}$ 和每个相机的时间偏移。

---

## 9. bias 样条不是位姿样条

Kalibr 还建立两条独立的欧氏 B-spline：

$$
\mathbf b_g(t),\qquad\mathbf b_a(t).
$$

它们分别描述 gyro bias 和 accel bias 的缓慢变化，默认：

```text
biasKnotsPerSecond = 50
```

完整的三类 spline 是：

| spline | 描述对象 | 默认 knot 密度 |
|---|---|---:|
| pose spline | $\mathbf T_{wb}(t)$ | 100/s |
| gyro bias spline | $\mathbf b_g(t)$ | 50/s |
| accel bias spline | $\mathbf b_a(t)$ | 50/s |

bias spline 还受到随机游走积分先验约束，避免 bias 高频变化并吸收真实运动。

---

## 10. 原生 Kalibr 的完整实现链路

```mermaid
sequenceDiagram
    participant Cam as cam0 AprilGrid observations
    participant Sensor as IccSensors.py
    participant Cal as IccCalibrator.py
    participant Pose as BSplinePose
    participant Problem as CalibrationOptimizationProblem
    participant Opt as Optimizer2

    Cam->>Sensor: 每帧 T_wc(t_k)
    Sensor->>Sensor: 加入时间偏移先验
    Sensor->>Sensor: T_wb(t_k) = T_wc(t_k) T_cb
    Sensor->>Sensor: rotation-vector unwrap
    Sensor->>Pose: initPoseSplineSparse
    Sensor-->>Cal: poseSpline = T_wb(t)
    Cal->>Problem: BSplinePoseDesignVariable
    Cal->>Problem: 加入相机外参、时间、重力和 bias DVs
    Cal->>Problem: 相机 residual 查询 T_wb(t)
    Cal->>Problem: gyro residual 查询姿态一阶导
    Cal->>Problem: accel residual 查询平移二阶导
    Opt->>Problem: 联合更新同一批 pose controls 和标定参数
```

关键文件与函数：

| 阶段 | 文件 | 函数 |
|---|---|---|
| cam0 生成公共样条 | `.../kalibr_imu_camera_calibration/IccSensors.py` | `IccCameraChain.initializePoseSplineFromCameraChain()` |
| 相机位姿转 body 位姿 | 同上 | `IccCamera.initPoseSplineFromCamera()` |
| 注册位姿样条 DV | `.../IccCalibrator.py` | `initDesignVariables()` |
| 相机查询样条 | `.../IccSensors.py` | `addCameraErrorTerms()` |
| gyro 查询一阶导 | 同上 | `addGyroscopeErrorTerms()` |
| accel 查询二阶导 | 同上 | `addAccelerometerErrorTerms()` |
| 旋转导数映射 | `upstream/.../bsplines/src/BSplinePose.cpp` | `angularVelocityBodyFrame()` |
| 联合问题编排 | `.../IccCalibrator.py` | `buildProblem()` |

最能说明样条归属的代码位于 `IccCalibrator.py`：

```python
# Initialize the system pose spline (always attached to imu0)
self.poseDv = asp.BSplinePoseDesignVariable(poseSpline)
addSplineDesignVariables(problem, self.poseDv)
```

---

## 11. 当前 Ceres 后端怎样实现同一状态定义

当前 Ceres 后端没有重新选择轨迹参考系，而是保持同一个 $\mathbf T_{wb}(t)$ 定义。

### 11.1 继续复用原生初始化

Python 入口：

`extensions/ceres_optimizer/python/kalibr_ceres_optimizer/integration.py`

`initialize_problem()` 依次调用：

```python
pose_spline = calibrator.CameraChain.initializePoseSplineFromCameraChain(
    spline_order, pose_knots_per_second, time_offset_padding
)

calibrator.initDesignVariables(
    design_variable_problem,
    pose_spline,
    ...
)
```

因此两套后端的：

- 轨迹参考系；
- 相机生成的离散 body 位姿；
- rotation-vector unwrap；
- spline order；
- knot 数量；
- 初始控制点；

全部来自同一套原生逻辑。

### 11.2 C++ 复制原生控制点和 basis

`ImuProblem` 构造时：

```cpp
dynamic_pose_spline_(std::make_unique<DynamicSpline>(pose_spline)),
pose_controls_(copyControls<6>(pose_spline.coefficients()))
```

`DynamicSpline` 复制原生 spline 的：

- knot vector；
- 每段 basis matrix；
- order；
- 有效时间范围。

它不是重新拟合另一条轨迹，而是为 Ceres 建立同一条轨迹的可自动求导求值器。

### 11.3 IMU residual 使用 0、1、2 阶样条权重

`ImuProblem.cpp` 中：

```cpp
residual.pose_weights =
    basisWeights(pose_spline, observation.timestamp, 0, 6);

residual.pose_derivative_weights =
    basisWeights(pose_spline, observation.timestamp, 1, 6);

residual.pose_second_derivative_weights =
    basisWeights(pose_spline, observation.timestamp, 2, 6);
```

分别表示：

$$
\mathbf x(t),\qquad
\dot{\mathbf x}(t),\qquad
\ddot{\mathbf x}(t).
$$

### 11.4 相机 residual 使用带时间偏移的 body 位姿

`CameraObservationResidual` 中：

```cpp
const T timestamp =
    T(camera_timestamp + time_shift_prior)
    + parameters[time_offset_block][0];

const Eigen::Matrix<T, 6, 1> pose =
    pose_spline->evaluateParameters(
        timestamp, 0, parameters,
        first_pose_control, 6);
```

然后把世界点先变换到 IMU/body：

```cpp
point_imu = world_from_imu.transpose() *
    (target_point - pose.head<3>());
```

再通过待估 IMU－cam0 外参以及固定相机链基线变换到相机。

### 11.5 求解后回写同一条原生样条

`python_module.cpp` 中：

```cpp
applyControls(pose_spline, problem.poseControls());
```

Ceres 优化后的控制点被写回原生 `BSplinePoseDesignVariable`，所以后续原生报告和 YAML 输出读取的是同一条更新后的 body 轨迹与标定变量。

---

## 12. 两套后端在位姿样条上的对照

| 项目 | 原生 Kalibr | 当前 Ceres |
|---|---|---|
| 样条描述对象 | imu0/body 的 $\mathbf T_{wb}(t)$ | 相同 |
| 初始观测来源 | cam0 的 AprilGrid 位姿 | 复用原生 |
| spline order | 6 | 6 |
| pose knots/s | 100 | 100 |
| 控制点初值 | `initPoseSplineSparse()` | 复制原生控制点 |
| rotation convention | Kalibr RotationVector | 显式复制该 convention |
| 相机查询 | ASLAM time expression | Ceres Jet timestamp |
| gyro 查询 | 原生 analytic expression | 相同公式的 AutoDiff residual |
| accel 查询 | 原生 analytic expression | 相同公式的 AutoDiff residual |
| 优化结果 | 原生 DV 内更新 | Ceres controls 回写原生 DV |

因此两者的差别不在“对谁做样条化”，而在：

- residual 怎样组织；
- Jacobian 怎样生成；
- 稀疏法方程怎样求解；
- LM 接受/拒绝和停止条件。

---

## 13. 常见误解

### 误解一：样条是相机轨迹，因为它由图像初始化

不正确。相机只提供离散初值；每个相机位姿先通过外参转换成 body 位姿，拟合的是 $\mathbf T_{wb}(t)$。

### 误解二：双目应该有两条样条

不正确。双目刚性连接，共享一条 body 轨迹，通过基线获得各自相机位姿。

### 误解三：IMU 数据也被拟合成一条测量样条

不正确。gyro 和 accel 原始测量仍作为离散 residual。被样条化的是 body 位姿以及两个慢变 bias。

### 误解四：样条控制点就是输出轨迹采样点

不正确。控制点是 B-spline 参数，不一定落在真实轨迹上。任意时刻的位姿由附近控制点按基函数加权得到。

### 误解五：相机初始化以后，轨迹只由视觉决定

不正确。联合优化中 pose controls 同时连接相机、gyro、accel residual。最终轨迹是三者共同作用的结果。

### 误解六：选择哪个刚体坐标系完全没有影响

最终物理关系可以通过坐标变换等价表达，但参数耦合、残差复杂度和数值条件会受参考系选择影响。将 body 绑定参考 IMU 能让主 IMU 外参和杆臂为零，是联合标定最自然的参数化。

---

## 14. 建议怎样结合代码阅读

建议按以下顺序观察：

1. 在 `initPoseSplineFromCamera()` 查看每帧 `obs.T_t_c()`；
2. 查看它与 `T_c_b` 相乘后的 $\mathbf T_{wb}(t_k)$；
3. 查看 `curve` 的前 3 行平移和后 3 行 rotation vector；
4. 查看 `initPoseSplineSparse()` 生成的控制点；
5. 在 `addCameraErrorTerms()` 查看同一时刻的 `T_w_b` 和 `T_c_w`；
6. 在 `addGyroscopeErrorTerms()` 查看由同一 spline 得到的 `w_b`；
7. 在 `addAccelerometerErrorTerms()` 查看 `a_w` 和 `C_b_w`；
8. 在 Ceres `ImuProblem.cpp` 对比 0/1/2 阶 basis weights；
9. 在 `CameraResidual.hpp` 改变时间偏移，观察查询到的 body pose 怎样变化；
10. 优化前后比较同一批 pose controls，而不是只比较最终外参 YAML。

最关键的验证关系是：

$$
\mathbf T_{c_nw}(t)
=\mathbf T_{c_nb}
\mathbf T_{wb}(t)^{-1}.
$$

只要在原生与 Ceres 中确认这条关系、相同的 spline controls 和相同的查询时间，就能判断两边是否在使用同一条参考 IMU/body 轨迹。

---

## 15. 总结

Kalibr 选择参考 IMU/body 位姿 $\mathbf T_{wb}(t)$ 作为唯一的连续运动样条，原因是：

1. 刚性相机－IMU 系统本来只有一条公共运动轨迹；
2. gyro 和 accel 天然对应 IMU 位姿的一阶、二阶导数；
3. 绑定参考 IMU 后，其自身外参为单位阵、杆臂为零；
4. 所有相机可通过常量外参从这条轨迹得到位姿；
5. 固定 AprilGrid 可直接作为世界系，不需要运动样条；
6. 单一公共样条避免每传感器独立轨迹造成的重复状态和 gauge；
7. 原生 Kalibr 与当前 Ceres 使用相同的轨迹定义、初始化控制点和 spline basis。

最简洁的一句话是：

> 相机负责给参考 IMU/body 轨迹提供初值和位姿约束，gyro 约束它的旋转一阶导，accel 约束它的平移二阶导；三者共同优化同一条 $\mathbf T_{wb}(t)$。
