# 纯视觉滚动快门相机标定源码实现说明

本文以当前源码为准，逐项解释以下两个入口实际执行的算法：

- 正式入口：`kalibr-noros calibrate cameras-rs`；
- 对照入口：`kalibr-noros calibrate native-rs-cameras`，它包装项目内的
  `kalibr_calibrate_rs_cameras`。

审计日期为 2026-10-08。若本文与后续代码发生冲突，应以代码和完整注释配置为准。本文只
讨论纯视觉相机 RS；`imu-camera-rs` 是另一条包含 IMU 残差的求解链。

## 1. 先给出结论

两个入口都采用逐角点时间模型，也都会联合优化相机参数、行时间和连续目标轨迹，但它们
不是同一个数学问题：

| 项目 | 正式 `cameras-rs` | 对照 `native-rs-cameras` |
|---|---|---|
| 相机数 | 单目、双目、多目 | 公共 CLI 恰好一台 |
| 相机模型 | `pinhole-equi`、`pinhole-radtan` | 公共 CLI 同左，并映射到原生 RS geometry |
| 外参 | 联合优化相邻 `T_cn_cnm1` | 单目，无相机间外参 |
| 行时间状态 | `tau = bound * tanh(q)`，求解中有界 | 原生 shutter 标量，求解中无界，结束后检查范围 |
| 重投影权重 | 固定 `feature_sigma_px² I` | 随当前轨迹运动更新的 adaptive covariance |
| 轨迹网格 | 观测率感知的固定网格 | 初始均匀网格，随后按残差自适应改 knot |
| 求解策略 | Levenberg–Marquardt | DogLeg |
| 异常点 | 可选一次最终四倍标准差过滤并重解 | 没有等价最终过滤 |
| 多目同步 | 时间容差且必须有共同全局角点 ID | 不适用 |
| 可观性 | 最终状态 reduced Fisher，full-rank 门控 | 没有等价输出 |
| 参数后验协方差 | 当前正式流程不计算、不输出 | 不计算、不输出 |

因此，相同数据得到不同的 K、D、行时间或 RMS 并不自动说明某一方实现错误。两边的轨迹
自由度、残差权重、信赖域策略和行时间参数化均不同。

## 2. 代码入口与调用链

### 2.1 公共 CLI 适配层

公共 task、目录数据集转接、模型门控和参数映射位于：

- [`src/python/kalibr_no_ros/cli.py`](../src/python/kalibr_no_ros/cli.py)；
- [`src/python/kalibr_no_ros/task.py`](../src/python/kalibr_no_ros/task.py)；
- [`src/python/kalibr_no_ros/rolling_shutter.py`](../src/python/kalibr_no_ros/rolling_shutter.py)。

两条命令使用同一个 `job: camera_rolling_shutter_calibration`。CLI 子命令决定后端：

```text
calibrate cameras-rs        -> system 后端
calibrate native-rs-cameras -> native 单目后端
```

目录输入会先按 `dataset.yaml` 暂存为私有求解器能读取的 bag 兼容层；相机 ID 会按 task
列表顺序映射为 `cam0`、`cam1`……。这些步骤只适配输入，不应改变原始整数纳秒时间戳、
图像内容或角点数学模型。

### 2.2 正式 system 后端

入口与核心实现分别是：

- [`kalibr_calibrate_rs_camera_system`](../src/kalibr/calibration/kalibr/python/kalibr_calibrate_rs_camera_system)；
- [`SystemCalibrator.py`](../src/kalibr/calibration/kalibr/python/kalibr_rs_camera_calibration/SystemCalibrator.py)。

实际顺序为：

```text
读图和每目 AprilGrid 检测
  -> 每目 K/D 初始化或安装 seed
  -> 按时间容差和共同角点 ID 建立多目 view
  -> 初始化相邻 baseline
  -> 用逐 view PnP 位姿初始化共享 pose spline
  -> 构建 K/D/T/行时间/轨迹联合问题
  -> LM 求解
  -> 可选最终角点过滤、重建同一问题并再次求解
  -> 最终只读可观性分析
  -> 行时间边界检查
  -> 结果、残差、RMS、报告和可选归档
```

### 2.3 原生对照后端

入口与核心实现分别是：

- [`kalibr_calibrate_rs_cameras`](../src/kalibr/calibration/kalibr/python/kalibr_calibrate_rs_cameras)；
- [`RsCalibrator.py`](../src/kalibr/calibration/kalibr/python/kalibr_rs_camera_calibration/RsCalibrator.py)；
- [`ReprojectionErrorKnotSequenceUpdateStrategy.py`](../src/kalibr/calibration/kalibr/python/kalibr_rs_camera_calibration/ReprojectionErrorKnotSequenceUpdateStrategy.py)。

实际顺序为：

```text
单目检测
  -> 初始化 K/D、逐帧 PnP 和原生 shutter
  -> 初始化均匀 pose spline
  -> 构建 K/D/shutter/轨迹联合问题
  -> DogLeg 求解
  -> 根据分段残差调整 knot、重建问题、继续求解
  -> 行时间范围和收敛检查
  -> 结构化单目结果与物理像素残差
```

这个入口保留了上游算法的主要结构，但项目已增加 no-ROS 输入、公开模型映射、显式 seed、
线程预算、失败检查、观测归档和 v1.0.0 输出，所以不能把它描述为完全未改动的上游脚本。

## 3. 正式 `cameras-rs` 的输入与初始化

### 3.1 每台相机独立检测和 K/D 初始化

每台相机先独立检测 AprilGrid。未冻结内参时：

- 没有 seed：走现有单目内参/畸变初始化；
- 有部分或完整 K/D seed：用 seed 作为起点，再执行现有内参初始化 LM；
- `freeze_intrinsics: true`：至少需要两台相机且每目必须有完整 K/D，后续 projection 和
  distortion design variable 固定。

seed 只是初值，不产生先验残差。正式 RS 求解没有冻结相邻外参的公开字段。

### 3.2 view 和 baseline

`ObservationDatabase` 使用 `synchronization_tolerance_s` 建立 view。多目不仅要满足时间差，
还必须存在共同的全局标定板角点 ID。相机图不连通时任务失败。

双目或多目通过 `MulticamCalibrationGraph.getInitialGuesses()` 初始化相邻 baseline。初始化
YAML 中的 `T_cn_cnm1` 只覆盖对应的起点，联合阶段仍会优化它。单目没有 baseline。

### 3.3 初始目标轨迹

每个 view 按检测角点数从多到少尝试 PnP，采用第一个有效结果。如果该观测属于第 i 目，
代码通过完整累计相机链换到 cam0 参考系：

$$
{}^{c_i}_{c_0}\mathbf T
= {}^{c_i}_{c_{i-1}}\mathbf T\cdots{}^{c_1}_{c_0}\mathbf T.
$$

因此 cam2 使用 `T_c2_c1 * T_c1_c0`，不会只使用最后一段 `T_c2_c1`。保存的时间戳是执行
PnP 的那台相机自己的源时间戳。

## 4. 正式联合问题的状态

正式后端注册以下 design variable：

| 状态 | 是否 active | 说明 |
|---|---|---|
| 每目 projection K | 默认 active | `freeze_intrinsics: true` 时 fixed |
| 每目 distortion D | 默认 active | `freeze_intrinsics: true` 时 fixed |
| 相邻 `T_cn_cnm1` | active | 单目为空 |
| 每目行时间 latent `q_i` | `estimate: true` 时 active | 物理行时间由 tanh 映射 |
| pose spline 控制系数 | active | 表示共享目标连续轨迹 |
| 标定板三维角点 | fixed | 注册为 landmark，但不是估计参数 |
| RS geometry 内部 shutter DV | inactive | 只为 camera error term 的 DV 集合完整注册；物理时间使用独立 latent |

将所有活动变量记为：

$$
\mathbf x =
\left\{\mathbf K_i,\mathbf D_i,
{}^{c_i}_{c_{i-1}}\mathbf T,q_i,\mathbf c_k\right\},
$$

其中 $\mathbf c_k$ 是 spline 控制系数。

## 5. 逐角点时间与有界行时间

### 5.1 时间模型

当前结果契约把相机图像时间戳定义为第 0 行曝光结束：

$$
t_{ij}=t_i+y_{ij}\tau_i,
$$

其中 $t_i$ 是该相机自己的源时间戳，$y_{ij}$ 是角点纵坐标，$\tau_i$ 单位为 s/row。
正 $\tau$ 表示 y 越大，角点时刻越晚。首末行跨度为：

$$
\Delta t_{first,last}=(H-1)|\tau|.
$$

多目 view 只建立观测关联，不会把各目的 $t_i$ 强行替换为 cam0 时间。

### 5.2 光滑有界状态

正式后端不直接优化 $\tau$，而是优化无量纲 latent $q$：

$$
\tau_i=b_i\tanh(q_i),\qquad
b_i=\mathtt{max\_abs\_line\_delay\_s}
$$
s
初值转换为：

$$
q_{i,0}=\operatorname{atanh}(\tau_{i,0}/b_i).
$$

其导数为：

$$
\frac{\partial\tau_i}{\partial q_i}
=b_i\left(1-\tanh^2(q_i)\right).
$$

该导数由 `ScalarExpressionNodeTanh` 的 C++ 表达式节点直接实现。它保证求解试探步始终满足
$|\tau_i|<b_i$，同时让实际更新和 Jacobian 使用同一条光滑函数。`estimate: false` 时，行
时间表达式为常数，对应 scalar DV inactive。

求解完成后，active 行时间若达到边界的 99.5% 仍会失败。原因是 tanh 靠近饱和区时导数
趋近 0，边界解通常表示范围、运动激励、轨迹自由度或可观性存在问题。

## 6. 正式重投影残差

对第 i 台相机、第 j 个标定板角点，代码先在 $t_{ij}$ 查询目标轨迹，再组合完整相机链：

$$
{}^{c_i}_{t}\mathbf T(t_{ij})
= {}^{c_i}_{c_0}\mathbf T
\left({}^{t}_{c_0}\mathbf T(t_{ij})\right)^{-1}.
$$

预测像素为：

$$
\hat{\mathbf u}_{ij}
=\pi_i\!\left(
\mathbf K_i,\mathbf D_i,
{}^{c_i}_{t}\mathbf T(t_{ij})\mathbf p_j
\right),
$$

物理像素残差为：

$$
\mathbf e_{ij}=\mathbf u_{ij}-\hat{\mathbf u}_{ij}.
$$

基础逆协方差固定为：

$$
\mathbf R^{-1}=\frac{1}{\sigma_{feature}^{2}}\mathbf I_2,
\qquad \sigma_{feature}=\texttt{feature\_sigma\_px}.
$$

开启 `blake_zisserman` 时，误差项额外安装常数为 2.0 的 Blake–Zisserman M-estimator。
概念目标函数为：

$$
J(\mathbf x)=
\sum_{i,j}\rho\!\left(
\mathbf e_{ij}^{\mathsf T}\mathbf R^{-1}\mathbf e_{ij}
\right)+J_{motion}(\mathbf c).
$$

结果中的每目 RMS 不含鲁棒核和运动先验，也不使用加权 objective：

$$
RMS_i=\sqrt{\frac{1}{N_i}\sum_{j=1}^{N_i}
\left(\Delta u_{ij}^{2}+\Delta v_{ij}^{2}\right)}.
$$

分母是角点数 $N_i$，不是坐标分量数 $2N_i$。

## 7. 正式 Jacobian 如何传递

核心联合问题使用表达式图的解析 Jacobian，不以有限差分作为主路径。

### 7.1 投影与 K/D

`ReprojectionError` 先由相机模型计算投影 Jacobian $\partial\pi/\partial\mathbf p$。因为残差
是“测量减预测”，点表达式接收负的投影 Jacobian；K/D 则由
`CameraDesignVariable.evaluateJacobians()` 加入相机模型自身的解析导数。

### 7.2 spline 控制系数和时间

`TransformationTimeOffsetExpressionNode` 在当前角点时间调用：

- `evalDAndJacobian(t, 0)`：得到 spline 值及其对局部控制系数的 Jacobian；
- `evalD(t, 1)`：得到曲线对时间的一阶导数；
- `curveValueToTransformationAndJacobian()`：把 6 维曲线值映射为位姿并给出 Jacobian。

时间链的核心项在代码中为 `JT * Phi_dot_c`，对应：

$$
\frac{\partial\mathbf T(t)}{\partial t}.
$$

对行时间 latent 的完整链可概念化为：

$$
\frac{\partial\mathbf e}{\partial q_i}
=\frac{\partial\mathbf e}{\partial\mathbf p}
\frac{\partial\mathbf p}{\partial\mathbf T}
\frac{\partial\mathbf T}{\partial t}
\underbrace{y_{ij}}_{\partial t/\partial\tau_i}
\underbrace{b_i(1-\tanh^2q_i)}_{\partial\tau_i/\partial q_i}.
$$

相邻 baseline 的 Jacobian由 transformation expression 的乘法链继续传播。cam2 及以后也
会经过完整累计链中的每一段外参。

### 7.3 为什么时间表达式需要 margin

角点时间会随 $q$ 更新并跨越相邻 spline segment。表达式建立时必须预注册可能涉及的控制
系数，以保持优化问题的稀疏结构不在迭代中突变。正式后端按所有相机可能的最大时间移动
设置左右 margin，并在构造时要求 `time_padding_s` 严格大于最大可能首末行跨度。

## 8. 正式 pose spline

### 8.1 order 和初始样本

默认 `spline_order: 4`，对应分段三次多项式。二阶运动先验要求 order 至少为 3。有效、
有限且时间唯一的 PnP 样本至少为：

```text
max(4, spline_order)
```

旋转向量会在相差 $2\pi$ 的等价表示中选择与前一项最近的一项，避免数值上的正负 $\pi$
跳变。

### 8.2 时间集合

代码区分三组时间：

| 时间集合 | 用途 |
|---|---|
| PnP pose sample 时间 | 拟合初始轨迹 |
| 每个 view 的参考时间，优先 cam0 | 估计实际入选观测频率 |
| 所有进入问题的各目源时间 | 确定必须覆盖的支持范围 |

支持区间的起止取 PnP 与全部观测时间的并集，再在两端各放置一个位于
`2 * time_padding_s` 之外的重复端点样本。

### 8.3 自动 segment 数

省略 `knots_per_second` 时：

$$
f_{obs}=\frac{1}{\operatorname{median}(\Delta t_{selected})},
$$

$$
N_{requested}=\operatorname{round}(T_{padded}f_{obs}),
$$

$$
N_{segment}=\max(2\,\texttt{spline\_order},N\_{requested}).
$$

显式填写 `knots_per_second` 时只把 $f_{obs}$ 换成指定 rate，仍保留相同下限。字段名称是
`knots_per_second`，当前实现实际用它计算目标 segment 数；最终底层 knot vector 元素数不
应直接等同于该字段乘时长。

正式后端不会自适应插入 knot，也不会按 PnP 位姿数截断 segment 数。报告中的 trajectory
元数据会保存策略、入选频率、请求/实际段数、下限、有效 rate 和时间范围。

### 8.4 正式和 native 是否使用同一种基函数

是。正式 `cameras-rs` 和原生 `kalibr_calibrate_rs_cameras` 都实例化同一个
`bsplines.BSplinePose`，其底层是标准非有理 B-spline。基矩阵由 `BSpline::M()` 按 Qin 的
递归基矩阵算法从 knot 序列生成；它与标准 Cox–de Boor B-spline 基函数表达的是同一组
分段多项式基。

两条路径默认都是：

```text
spline order = 4
polynomial degree = order - 1 = 3
```

对均匀 knot 的某个 segment，定义：

$$
u=\frac{t-t_i}{t_{i+1}-t_i},\qquad 0\leq u\leq1.
$$

该段只受四个局部控制系数影响：

$$
\mathbf s(t)=\sum_{k=0}^{3}B_k(u)\mathbf c_k,
$$

其中三次均匀 B-spline 基函数为：

$$
B_0(u)=\frac{(1-u)^3}{6},
\qquad
B_1(u)=\frac{3u^3-6u^2+4}{6},
$$

$$
B_2(u)=\frac{-3u^3+3u^2+3u+1}{6},
\qquad
B_3(u)=\frac{u^3}{6}.
$$

源码用等价矩阵形式求值：

$$
\mathbf s(t)=
\begin{bmatrix}\mathbf c_0&\mathbf c_1&\mathbf c_2&\mathbf c_3\end{bmatrix}
\mathbf M_i^{\mathsf T}
\begin{bmatrix}1&u&u^2&u^3\end{bmatrix}^{\mathsf T}.
$$

`BSplinePose` 的每个控制系数是六维量。前三维形成平移曲线，后三维形成旋转向量曲线，再
由 `RotationVector` 映射为旋转矩阵：

$$
\mathbf T(t)=
\begin{bmatrix}
\operatorname{Exp}(\mathbf r(t))&\mathbf p(t)\\
\mathbf0^{\mathsf T}&1
\end{bmatrix}.
$$

所以它不是直接对 $4\times4$ 变换矩阵逐元素插值，也不是 cumulative Lie-group B-spline。
二阶运动先验求导的对象也是这条六维曲线；旋转部分是旋转向量曲线的二阶导数，不应严格
解释为物理角加速度。

两条后端的区别在 knot 序列，而不是 B-spline 家族：

| 后端 | 初始 knot | 求解期间 | 基矩阵 |
|---|---|---|---|
| 正式 `cameras-rs` | `initPoseSplineSparse()` 生成均匀 knot | 固定，不插入 knot | 均匀网格下各内部段使用同形三次基函数 |
| native | 初始同样由 `initPoseSplineSparse()` 生成均匀 knot | adaptive 策略可能插入 knot，并用 `initPoseSplineSparseKnots()` 重建 | 非均匀后由每段相邻 knot 间距生成不同的 $\mathbf M_i$ |

因此可以说两者“使用同一种 B-spline 基函数实现”，不能说两者“使用相同的基矩阵、knot
位置或轨迹自由度”。native 自适应后即使 order 仍为 4，非均匀 knot 也会改变局部基函数的
时间支撑和每段曲线形状。

### 8.5 正式后端怎样求初始控制系数

正式后端将 padding 后的 PnP 六维位姿样本传给：

```python
spline.initPoseSplineSparse(
    padded_times, padded_curve, segment_count, 1e-4)
```

`initPoseSplineSparse()` 先在完整 padding 时间域生成均匀 knot。对 order 为 $k$、有效段数为
$N_s$ 的样条：

$$
N_{coefficient}=N_s+k-1,
\qquad
N_{knot}=N_s+2k-1.
$$

所以传入的第三个参数是 **segment 数**，不是最终 knot-vector 元素数。默认 order 4 时，
例如 60 段对应 63 个六维控制系数和 67 个 knot。

初始化器用样条基矩阵组成 $\mathbf A$，把 padding 后的六维 PnP 样本堆成 $\mathbf b$，并
求解：

$$
\min_{\mathbf c}
\|\mathbf A\mathbf c-\mathbf b\|^2
+\mathbf c^{\mathsf T}\mathbf Q_{init}\mathbf c,
$$

等价正规方程为：

$$
(\mathbf A^{\mathsf T}\mathbf A+\mathbf Q_{init})\mathbf c
=\mathbf A^{\mathsf T}\mathbf b.
$$

order 大于 2 时，$\mathbf Q_{init}$ 是权重 `lambda=1e-4` 的六维曲线二阶导数积分。它只用
来得到平滑、有限的初始控制系数，不是最终 RS 联合问题中的运动先验；联合阶段会另外使用
配置的 `motion_translation_weight` 和 `motion_rotation_weight`。

若稀疏 Cholesky 求解失败，初始化器退回 dense LDLT。初始样条建立后，所有控制系数在
RS 联合问题中 active，因此 PnP 样本和 `lambda=1e-4` 都不会把最终轨迹固定在初始曲线上。

## 9. 正式二阶运动先验

代码使用：

```python
asp.addMotionErrorTerms(problem, spline_dv, W, 2)
```

其中：

$$
\mathbf W=\operatorname{diag}
\left(w_t,w_t,w_t,w_r,w_r,w_r\right),
$$

默认 $w_t=10^{-5}$、$w_r=10^{-2}$。概念上它对每个有效 segment 累积：

$$
J_{motion}
=\sum_s\int_{t_s}^{t_{s+1}}
\mathbf c^{(2)}(t)^{\mathsf T}\mathbf W
\mathbf c^{(2)}(t)\,dt.
$$

底层 `segmentIntegral(..., W, 2)` 为每段生成等价的二次项，再以
`MarginalizationPriorErrorTerm` 加入问题。因此它有解析 Jacobian，也会进入最终 Fisher
信息。它不是 IMU 测量，也不是“行时间先验”；它约束的是无图像区间和端部的轨迹二阶
变化，防止过密 spline 在缺少观测处自由摆动。

轨迹更灵活时，轨迹形状和行时间可能互相吸收像素误差；轨迹过平滑时，未建模运动又可能
被 K/D/T/行时间吸收。因此行时间必须做 knot rate 和先验权重敏感性分析，不能只看训练集
RMS 最小值。

## 10. 正式异常点与求解器

### 10.1 最终角点过滤

只有以下条件同时满足才执行：

- `remove_outliers: true`；
- `final_filtering: true`；
- 对应相机中实际含残差的 observation/frame 数不少于
  `min_views_for_outlier_statistics`，默认 20。

代码按每台相机的残差坐标统计标准差，逐坐标阈值为：

$$
\max(4\sigma,\texttt{feature\_sigma\_px}).
$$

一旦角点任一坐标超过阈值就作为候选，但每个 observation 至少保留 4 个角点。若确实删除
角点，代码保留第一次求解后的轨迹系数，重建同一联合问题并再次求解。它没有普通相机增量
估计器的 view 信息增益筛选语义。

### 10.2 Optimizer2 设置

正式后端固定使用：

| 设置 | 值 |
|---|---|
| 信赖域 | Levenberg–Marquardt，初始参数 10 |
| 线性求解 | Block Cholesky |
| Schur complement | 开启 |
| `convergenceDeltaX` | `1e-5` |
| `convergenceDeltaJ` | `1e-4` |
| `max_iterations` | 公共默认 `80` |
| 线程 | task/CLI `optimizer_threads` |

Optimizer2 的原生停止语义是参数增量 `dX` 或目标函数绝对变化 `|dJ|` 任一达到阈值即可。
线性求解失败或实际迭代数达到 `max_iterations` 均视为失败，不发布貌似成功的结果。

## 11. 正式可观性与“协方差”

### 11.1 当前实际执行什么

正式入口调用 `observability.analyze_write_require()`，在最终最优点只读线性化。选入的物理
标定块包括：

- active 的 K/D；
- 相邻 baseline 的旋转和平移；
- active 的行时间 latent `q`。

轨迹控制系数和固定 landmark 作为 nuisance 参数消去，随后分析 reduced Fisher
information 的奇异值、hard rank、operational rank 和弱模态贡献。hard rank 不满时任务
失败；只有 operational 弱模态时报告 `weakly_observable`，提示参数组合不稳定。

可观性文件中行时间块是无量纲 latent `q`，不是 s/row。因为
$d\tau/dq=b(1-\tanh^2q)$，接近边界时从 latent 信息强度直接推断物理行时间精度尤其需要
谨慎。

### 11.2 当前没有输出参数后验协方差

`observability.py` 确实另有 `analyze_with_covariance()`，但正式 `cameras-rs` 当前调用的是
不请求 covariance 的 `analyze()` 路径。因此当前结果和 `observability.yaml` 不包含 K/D/T/
行时间的完整后验 covariance 或标准差。

必须区分三种概念：

| 名称 | 当前用途 | 是否是参数不确定度 |
|---|---|---|
| `feature_sigma_px² I` | 正式后端观测噪声/残差权重 | 否 |
| native adaptive covariance | 原生后端随运动改变残差权重 | 否 |
| reduced parameter covariance | Hessian/Fisher 逆意义上的参数后验近似 | 是，但正式 RS 当前未请求输出 |

## 12. 原生 `kalibr_calibrate_rs_cameras` 的状态与初始化

公共 `native-rs-cameras` 只允许一台 `pinhole-equi` 或 `pinhole-radtan`。内部脚本还保留
其他历史模型 alias，但它们不是当前公共 task 契约的一部分。

活动状态为：

```text
K、D、原生 shutter lineDelay、pose spline 控制系数
```

标定板角点默认 fixed；没有相机间 baseline。公共包装器总是把 task 中的
`line_delay_s` 作为显式 seed 传入，所以公共默认是 `0.0 s/row`。只有绕过包装器直接调用
内部脚本且不传 seed 时，才使用：

$$
\tau_0=\frac{1}{f_{selected}H}.
$$

native 路径没有正式 system 路径那种“每目先完成独立内参 LM，再进入相机系统联合问题”的
阶段划分。没有 K/D seed 时，`initializeIntrinsics()` 只负责生成联合问题的起点；有 seed
时，代码先取得图像元数据并安装 seed。K、D、shutter 和轨迹随后在同一个 native 联合问题
中调整。

原生 shutter 直接优化 $\tau$，其时间 Jacobian 为：

$$
\frac{\partial t}{\partial\tau}=y.
$$

`max_abs_line_delay_s` 在公共 task 中仍为估计模式的必填范围，但 native 内部只在求解结束
后检查 $|\tau|$，不会约束 DogLeg 中间试探步。

## 13. 原生 spline 与 adaptive knot

### 13.1 初始 spline

有效 PnP 位姿会按时间排序，失败、非有限和重复时间样本被移除，至少保留
`max(4, splineOrder)` 项。公共包装器下：

- `splineOrder = 4`；
- `timeOffsetPadding = 0.5 s`；
- 首尾实际各增加 `2 * 0.5 s` 的重复端点；
- `timeOffsetConstantSparsityPattern = 0.5 s`；
- 帧率由入选源时间戳正间隔的中位数推断。

源码局部变量名为 `knots`，打印文本也写成 knots，但它实际作为
`initPoseSplineSparse(..., numSegments, ...)` 的第三个参数传入。因此下式得到的是**初始有效
segment 数**：

$$
N_{segment,0}=\operatorname{round}(T_{padded}f_{selected}/3).
$$

默认 order 4 时，底层实际生成：

$$
N_{coefficient}=N_{segment,0}+3,
\qquad
N_{knot}=N_{segment,0}+7.
$$

native 初始样条同样调用 `initPoseSplineSparse()`，拟合目标是全部保留的单目 PnP 六维位姿，
初始化二阶正则 `lambda` 同样为 `1e-4`。源码打印中的 `100 knots per second` 是硬编码显示
值，既不是实际 segment rate，也不是实际 knot rate，不能引用为算法参数。

### 13.2 自适应更新的准确语义

初次求解后，`ReprojectionErrorKnotSequenceUpdateStrategy` 最多运行 10 次。每轮：

1. 按角点时间把当前加权误差和误差项数累计到 segment；
2. 对连续无观测的内部区域，从本轮误差分段使用的 filtered knot 列表中移除对应项，并把
   时间范围加入禁用区；
3. 若某段误差和大于该段误差项数量，尝试在段中点插入 knot；
4. 若半段间距不大于 `1/(2 * framerate)`，不再细分；
5. 若上一轮某段误差没有至少约 20% 的改善，把对应时间段禁用；
6. 除第一轮外，只有 knot 数相对上一轮增加超过 1% 才继续重建和求解。

当前实现把 filtered knot 列表用于重新统计分段误差，但生成候选更新序列时仍传入旧 knot
序列。因此“无观测移除”不能简单解释为最终 knot vector 必然删除对应 knot；确定发生的
行为是该时间范围被加入禁用区，后续不再细分。

源码变量 `__maxKnotsPerSecond` 实际保存的是秒单位的最小半段间距
`1/(2*framerate)`，变量名容易误导。该后端没有正式 system 的
`knots_per_second` 配置语义。

发生 adaptive 更新时，native 不会重新运行 PnP。`getUpdatedSpline()` 会：

1. 在旧样条的 `[t_min,t_max]` 内按新的 knot 列表长度均匀取样时间；
2. 在这些时间上计算上一轮已优化样条的六维曲线值；
3. 保留覆盖首尾时间域所需的旧端部 knot；
4. 调用 `initPoseSplineSparseKnots()`，用候选非均匀 knot 和 `lambda=1e-6` 拟合新控制系数；
5. 用新样条重建 K/D/shutter/轨迹联合问题并再次优化。

这里的 `1e-6` 也是样条重建初始化正则，不是联合问题运动先验。

### 13.3 两条初始化路径对比

| 项目 | 正式 `cameras-rs` | native `kalibr_calibrate_rs_cameras` |
|---|---|---|
| 初始 PnP 样本 | 每个多目 view 选择角点最多的有效观测，再经完整 baseline 链换到 cam0 | 每个成功单目 observation 都做 PnP |
| 时间去重 | 按时间排序，`1e-12 s` 内只留一个 | 按时间排序，`1e-12 s` 内只留一个 |
| 最少样本 | `max(4, order)` | `max(4, order)` |
| 时间支持范围 | PnP 时间与所有各目观测时间的并集 | 保留的单目 PnP 首末时间 |
| 两端 padding | 各扩展 `2 * time_padding_s`，复制首末 PnP 位姿 | 各扩展 `2 * timeOffsetPadding`，复制首末 PnP 位姿 |
| 默认 padding | `0.5 s` | 公共包装器覆盖为 `0.5 s` |
| 旋转处理 | 展开相差 $2\pi$ 的等价旋转向量 | 相同处理 |
| 初始有效段数 | `max(2*order, round(T_padded*f_selected))` | `round(T_padded*f_selected/3)` |
| 初始拟合 | 均匀 knot，`initPoseSplineSparse(..., 1e-4)` | 相同初始化器和 `1e-4` |
| 联合求解后的 knot | 保持固定 | 最多 10 轮 adaptive 更新 |
| adaptive 重建 | 不适用 | 采样上一轮轨迹，用候选 knot 和 `1e-6` 重拟合，不重做 PnP |

正式后端若执行最终角点过滤，会复制第一次联合求解后的完整 knot vector 和控制系数，再
重建残差问题；它也不会回到 PnP 初始样条。

## 14. 原生 adaptive covariance 残差

物理残差仍是：

$$
\mathbf e=\mathbf u-\hat{\mathbf u}.
$$

区别在于 `CovarianceReprojectionError` 每次评价误差时根据当前轨迹更新权重。代码先计算
局部运动投影项：

$$
\mathbf j_m
=\mathbf J_{proj}\,\mathbf J_{T\!p}\,
\mathbf J_{curve\rightarrow T}\,\dot{\boldsymbol\Phi}(t)\mathbf c\,\tau.
$$

随后构造：

$$
\mathbf A=
\begin{bmatrix}
1 & j_{m,u}\\
0 & 1+j_{m,v}
\end{bmatrix},
$$

并把基础像素 covariance $\mathbf R_0$ 映射为：

$$
\mathbf R_{eff}=\mathbf A\mathbf R_0\mathbf A^{\mathsf T},
\qquad
\mathbf R_{eff}^{-1}
=\left(\mathbf A\mathbf R_0\mathbf A^{\mathsf T}\right)^{-1}.
$$

它的目的，是让行坐标噪声通过“行坐标改变采样时间、采样时间又经过当前运动改变预测像素”
这条链影响有效二维测量权重。

这里有一个实现细节：`evaluateError()` 会刷新 $\mathbf R_{eff}^{-1}$，但
`evaluateJacobians()` 只对物理残差调用点表达式和相机解析 Jacobian，没有继续对
$\mathbf A(\mathbf x)$ 的变化求导。换句话说，每次线性化使用当前状态更新权重，再把该权重
视为这次残差线性化中的固定量。

这仍然不是参数后验 covariance，也不会直接给出 K/D/行时间的置信区间。

## 15. 原生 Jacobian、运动先验和求解器

原生残差同样通过表达式树传播解析 Jacobian：

- `ScalarExpressionNodeKeypointTime` 对 shutter 的导数为 y；
- `TransformationTimeOffsetExpressionNode` 对 spline 控制系数和时间求导；
- camera model 对投影、K 和 D 求导；
- adaptive covariance 的权重按上一节所述在误差评价时刷新。

原生默认运动先验矩阵最终同样为：

$$
\operatorname{diag}(10^{-5},10^{-5},10^{-5},10^{-2},10^{-2},10^{-2}).
$$

它通过一个 `BSplineMotionError` 加入；正式后端则显式按 segment 添加等价二次项，便于最终
可观性分析展开。两者数值对角权重相同，不表示轨迹网格和完整目标函数相同。

公共入口实际使用：

| 设置 | 值 |
|---|---|
| 信赖域 | DogLeg |
| 线性求解 | Block Cholesky |
| Schur complement | 开启 |
| `convergenceDeltaX` | `1e-8` |
| `convergenceDeltaJ` | `1e-4` |
| `max_iterations` | 公共包装器默认 `80` |
| adaptive knot 最大轮次 | `10` |

初始求解和每次 adaptive knot 重求解都执行线性失败及迭代预算检查。达到最大迭代数视为
未收敛，不发布正式结构化结果。

## 16. 公共默认值与内部默认值

### 16.1 用户实际可见的公共默认值

| 字段 | 正式 system | native 公共包装器 |
|---|---:|---:|
| `line_delay_s` | `0.0 s/row` | `0.0 s/row`，总是显式传入 |
| `estimate` | `true` | `true` |
| `max_abs_line_delay_s` | 估计时必填 | 估计时由公共契约必填，内部只作结束检查 |
| `window_half_size_px` | `2 px` | `2 px` |
| `max_displacement_px` | `sqrt(1.5) px` | `sqrt(1.5) px` |
| `focal_initialization_min_visible_corner_ratio` | `1.0` | 不作为 native 专用字段公开 |
| `synchronization_tolerance_s` | `0.02 s` | 不适用 |
| `min_views_for_outlier_statistics` | `20` | 不适用 |
| `remove_outliers` / `final_filtering` | `true` / `true` | 不支持 |
| `blake_zisserman` | `false` | 不支持 |
| `max_iterations` | `80` | `80` |
| `spline_order` | `4` | 内部固定 `4`，公共 task 不接受该字段 |
| `time_padding_s` | `0.5 s` | 内部包装固定 `0.5 s`，公共 task 不接受该字段 |
| `knots_per_second` | 自动观测中位频率 | 不支持；内部先 `/3` 后 adaptive |
| `feature_sigma_px` | `1.0 px` | `1.0 px` |
| 运动先验平移/旋转权重 | `1e-5` / `1e-2` | 内部固定同一对角值 |
| `freeze_intrinsics` | 默认 false；至少双目且完整 K/D seed 才能 true | 公共命令恰好单目，因此有效语义只能 false |

### 16.2 为什么直接看 `RsCalibratorConfiguration` 会看错默认值

原生配置类仍保留上游类属性：最大迭代 20、时间稀疏 buffer 0.08 s、padding 0.05 s、帧率
30 Hz、基础 inverse covariance `1/0.26`。内部脚本自己的 argparse 最大迭代默认又是 30。

项目公共包装器运行 `native-rs-cameras` 时会覆盖这些值：

- 最大迭代覆盖为 task 默认 80；
- buffer 和 padding 均覆盖为 0.5 s；
- 帧率由最终入选源时间戳推断；
- inverse covariance 由 `feature_sigma_px` 转换为 `1/sigma²`；
- 行时间 seed 总是显式传入。

所以用户实际行为必须从 `task.py -> kalibr_calibrate_rs_cameras -> RsCalibrator.py` 整条调用
链判断，不能只读配置类属性。

## 17. 输出指标和限制

两条后端都重新计算未加权物理像素 RMS，便于使用相同单位比较。正式多目结果还会输出：

- 每目 `shutter.line_delay_s` 和首末行跨度；
- 相邻 `T_cn_cnm1`；
- `alignment`：只用 K/D/T 做普通静态光学校正后的极线诊断；
- `rs_compensated_pair_residual`：减去当前 RS 轨迹模型预测后的样本内残差。

`alignment` 不包含逐行轨迹补偿；`rs_compensated_pair_residual` 又依赖当前拟合轨迹，不能当
作独立验证。参数可信度仍需通过固定延迟对照、轨迹带宽敏感性、独立动态测试集和重复采集
判断。

native 公共入口没有多目外参、alignment、正式 reduced-Fisher 可观性，也不支持
`output.export_poses`。它适合做单目算法对照，不应替代正式多目交付。

## 18. 文档一致性审计

本次按代码核对了以下文档：

- [滚动快门相机标定](ROLLING_SHUTTER_CAMERA_CALIBRATION_ZH.md)；
- [`cameras-rs` 连续轨迹、样条与运动先验入门](ROLLING_SHUTTER_SPLINE_TUTORIAL_ZH.md)；
- [任务参数](TASK_PARAMETERS_ZH.md)；
- [源码导读](SOURCE_CODE_DEEP_DIVE_ZH.md)；
- [完整注释 RS task](../config/examples/v1.0.0/all_params/camera_rolling_shutter_calibration_task_full.yaml)。

当前关键默认值、正式固定轨迹网格、native adaptive knot、行时间参数化、公开模型范围和
停止失败语义与代码一致。需要额外澄清、并由本文补足的内容是：

1. native adaptive covariance 的矩阵构造及其 Jacobian处理方式；
2. 观测 covariance、adaptive covariance 与参数后验 covariance 的区别；
3. 正式 `cameras-rs` 当前只请求可观性谱，不请求参数 covariance；
4. 原生配置类默认、内部脚本默认和公共包装器最终默认存在三层覆盖；
5. 原生 adaptive knot 中若干变量名和打印值不能按字面解释。

后续修改求解代码时，应同步更新本文、主 RS 文档、`TASK_PARAMETERS_ZH.md` 和完整注释配置，
并以公开包装器最终传给私有脚本的参数作为“用户默认行为”。

## 19. C++ 数学实现索引

| 文件 | 对应内容 |
|---|---|
| [`ReprojectionError.hpp`](../src/kalibr/camera/aslam_cv_error_terms/include/aslam/backend/implementation/ReprojectionError.hpp) | 正式固定权重重投影残差和点/相机 Jacobian |
| [`CovarianceReprojectionError.hpp`](../src/kalibr/camera/aslam_cv_error_terms/include/aslam/backend/implementation/CovarianceReprojectionError.hpp) | native adaptive covariance 的 A 矩阵和有效权重 |
| [`ScalarExpressionNodeKeypointTime.hpp`](../src/kalibr/camera/aslam_cv_backend/include/aslam/backend/implementation/ScalarExpressionNodeKeypointTime.hpp) | 原生 `dt/dtau = y` |
| [`ScalarExpressionNode.cpp`](../src/kalibr/optimization/aslam_backend_expressions/src/ScalarExpressionNode.cpp) | 正式 tanh 表达式及 `1-tanh²(q)` Jacobian |
| [`BSplineExpressions.cpp`](../src/kalibr/trajectory/aslam_splines/src/BSplineExpressions.cpp) | spline 对控制系数和时间的解析 Jacobian |
| [`BSplineMotionErrorFactory.hpp`](../src/kalibr/trajectory/aslam_splines/include/aslam/backend/implementation/BSplineMotionErrorFactory.hpp) | 正式逐 segment 二阶运动误差 |
| [`BSplinePose.hpp`](../src/kalibr/trajectory/bsplines/include/bsplines/BSplinePose.hpp) | 位姿 spline、order 和变换接口 |
