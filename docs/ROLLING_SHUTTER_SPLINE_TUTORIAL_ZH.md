# `cameras-rs` 连续轨迹、样条与运动先验入门

本文面向第一次接触连续时间标定、B-spline 和运动先验的读者。目标是解释正式
`kalibr-noros calibrate cameras-rs` 中这些术语为什么存在、彼此有什么区别，以及当前代码
究竟怎样使用它们。

本文只描述正式 system 后端。临时 `native-rs-cameras` 使用自适应 knot，流程不同，不能把
两者的 spline 参数直接混用。

两条后端的底层基函数家族相同：都使用 `bsplines.BSplinePose` 的标准非有理 B-spline，
默认 `order=4`，即分段三次多项式；六维曲线的前三维表示平移，后三维表示旋转向量。区别
在 knot 网格：正式后端始终使用均匀固定网格，native 初始均匀，但 adaptive knot 后可能
成为非均匀网格。此时 order 没变，具体每段基矩阵、时间支撑和轨迹自由度已经不同。公式、
代码位置和两种后端的逐项对照见
[纯视觉 RS 源码实现说明](ROLLING_SHUTTER_IMPLEMENTATION_REFERENCE_ZH.md#84-正式和-native-是否使用同一种基函数)。

## 1. 为什么滚动快门标定需要连续轨迹

普通全局快门近似认为整幅图像在同一时刻成像，因此一幅图只需要一个标定板位姿：

```text
一幅图 → 一个时间 → 一个目标位姿
```

滚动快门逐行读出。正式任务采用：

```text
t_corner = t_frame + y × line_delay
```

其中：

- `t_frame` 是该相机源图像的第 0 行时间戳；
- `y` 是角点的纵坐标，单位 px；
- `line_delay` 是相邻两行采样参考时刻之差，单位 s/row。

同一幅图上方和下方的角点具有不同时间。例如图像高 1080 px、行时间为 8 us/row：

```text
第 0 行：   t_frame
第 500 行： t_frame + 4.000 ms
第 1079 行：t_frame + 8.632 ms
```

求解器必须能在任意角点时间查询标定板位姿：

$$
{}^{c_0}_{t}\mathbf T(t).
$$

离散的“一帧一个位姿”无法直接回答帧内任意时刻的位姿，所以正式 `cameras-rs` 用一条连续
位姿 spline 表示目标运动。

```mermaid
flowchart LR
    A[图像时间戳] --> B[角点行坐标 y]
    B --> C[t_corner = t_frame + y tau]
    C --> D[在连续 spline 上查询目标位姿]
    D --> E[组合完整相机外参链]
    E --> F[投影角点并计算像素残差]
```

## 2. 先分清这些术语

| 术语 | 含义 | 当前代码中的角色 |
|---|---|---|
| observation | 一台相机在一幅图中检测到的 AprilGrid 角点 | 提供角点像素和源时间戳 |
| view | 时间接近且有共同全局角点 ID 的多目观测集合 | 建立多目共视关系 |
| pose sample | 由某个有效 observation 做 PnP 得到的离散目标位姿 | 只用于初始化 spline |
| trajectory | 标定板位姿随时间变化的连续函数 | 联合优化中的共享运动状态 |
| control point / coefficient | 决定 spline 形状的参数 | active design variable |
| knot | knot vector 中的时间节点 | 划分分段多项式的时间结构 |
| segment | 相邻有效 knot 之间的一段多项式区间 | 报告中的 `segment_count` |
| spacing | 相邻 segment 的时间长度 | 近似为 `1 / effective_segments_per_second` |
| spline order | 分段多项式的阶数 | 默认 4，即三次多项式 |
| knot/segment rate | 每秒希望布置多少段 | task 字段 `knots_per_second` |
| bandwidth | 轨迹表示快速变化的能力 | rate 越高通常越灵活 |
| padding | 实际观测区间两端的额外时间支持 | task 字段 `time_padding_s` |
| motion prior | 对轨迹二阶导数的平滑约束 | 正式目标函数的一部分 |

最容易混淆的是以下三点：

1. pose sample 是初始化观测，control point 是求解变量，两者不是一一对应；
2. knot 是时间节点，segment 是两个有效节点间的区间；
3. 当前字段虽然叫 `knots_per_second`，代码实际用它计算目标 `segment_count`，不能把它理解
   成最终原始 knot vector 每秒恰好包含多少个元素。

## 3. 正式 `cameras-rs` 联合优化什么

system 后端的主要活动状态为：

```text
每目 K/D
相邻外参 T_cn_cnm1
每目有界行时间
共享目标位姿 spline 的控制系数
```

标定板三维角点固定，不作为自由 landmark。对每个最终使用角点，代码执行：

```text
角点像素 y
  → 计算该角点时间
  → 在 spline 上取 T_c0_target(t)
  → 累计 T_ci_c0
  → 得到 T_ci_target(t)
  → 投影标定板三维角点
  → 与检测像素相减
```

概念目标函数为：

$$
J = J_{\text{reprojection}} + J_{\text{motion}}.
$$

`J_reprojection` 让预测像素靠近检测像素；`J_motion` 防止观测之间和轨迹端部出现无依据的
剧烈弯曲。

## 4. 初始轨迹怎样从图像得到

实现位于
[`SystemCalibrator.py`](../src/kalibr/calibration/kalibr/python/kalibr_rs_camera_calibration/SystemCalibrator.py)
的 `_initial_target_poses()`、`_observation_timestamps()` 和 `make_pose_spline()`。

### 4.1 先生成离散 PnP 位姿样本

对每个共视 view：

1. 按检测角点数量从多到少检查各目观测；
2. 用当前 K/D 对第一个有效观测执行 PnP；
3. 如果该观测不是 cam0，累计完整相机链，将位姿换算到 cam0 参考系；
4. 保存该观测自己的源时间戳和目标位姿；
5. 该 view 的所有 PnP 都失败时，不生成 pose sample。

成功样本至少需要：

```text
max(4, spline_order)
```

默认四阶时至少需要 4 个不同时间的有效 PnP 位姿。它只是建立初始曲线所需的最低数量，
不是足以保证行时间可观的生产数据量。

### 4.2 三组时间戳不要混为一谈

当前代码区分：

| 名称 | 来源 | 用途 |
|---|---|---|
| `pose_samples` 时间 | 每个 view 中成功 PnP 的那台相机 | 拟合初始轨迹 |
| `selected_timestamps` | 每个 view 优先取 cam0，否则取该 view 第一目 | 估计实际入选观测频率 |
| `support_timestamps` | 所有进入 view 的各目源时间戳 | 确定轨迹需要覆盖的真实时间范围 |

这样多目同步只负责组成 view，不会把各目的真实时间戳改成 cam0 时间。

### 4.3 旋转向量为什么要展开

同一个旋转可以用相差 $2\pi$ 的旋转向量表示。如果相邻位姿一个接近 $+\pi$、另一个接近
$-\pi$，物理运动很小，但数值曲线看起来会突然跳变。`_continuous_rotation_vectors()` 会在
等价旋转向量中选择最接近前一个样本的表示，避免初始化 spline 穿过不必要的大跳变。

## 5. B-spline 的 order、degree、knot 和 segment

### 5.1 `spline_order: 4` 不是四次多项式

本项目使用的 B-spline 定义为：

```text
degree = spline_order - 1
```

所以：

| `spline_order` | 分段多项式 degree | 直观性质 |
|---:|---:|---|
| 2 | 1，一次 | 分段直线，不能提供有意义的二阶曲率 |
| 3 | 2，二次 | 二阶导数可按段计算，但段间不够平滑 |
| 4 | 3，三次 | 默认值，通常具有连续的一、二阶变化 |

当前联合问题始终加入二阶运动先验，所以 `spline_order` 强制至少为 3。默认选择 4，是为了
让轨迹既能计算二阶导数，又保持较平滑的局部运动。

### 5.2 knot 是时间边界，control point 决定形状

用一维位置示意：

```text
时间轴     |--------|--------|--------|
knot       k0       k1       k2       k3
segment       s0       s1       s2
control       ●    ●    ●    ●    ●
```

- knot 决定每段多项式在什么时间区间有效；
- segment 是相邻有效 knot 之间的时间区间；
- control point/系数决定曲线形状，一般不会正好落在测量点上；
- 四阶 B-spline 在一个局部 segment 上由附近 4 组系数共同影响。

底层 knot vector 为了提供边界支持会包含额外 knot，因此不要用“原始 knot 数减一”推断
报告中的 `segment_count`。报告值表示初始化时请求并实际采用的有效 segment 数。

### 5.3 spacing、rate 和 bandwidth

若有效时间域长度是 $D$ 秒，使用 $N$ 个 segment，则：

$$
\text{effective segments per second}=\frac{N}{D},
$$

平均 segment 间距约为：

$$
\Delta t_{segment}\approx\frac{D}{N}.
$$

例如 10 秒使用 50 段：

```text
effective rate = 5 segments/s
average spacing = 0.2 s
```

rate 越高、spacing 越小，轨迹越能表示快速变化；但自由度也更多，更可能吸收角点噪声或
行时间误差。rate 越低，轨迹越平滑；但真实快速运动可能无法表示，剩余误差可能被 K/D、
外参或行时间错误吸收。

这里的“带宽”是建模能力的直观说法，不表示代码显式设计了一个理想低通滤波器。

## 6. 当前代码怎样自动选择段数

### 6.1 自动观测频率

省略 `knots_per_second` 时，代码先计算最终入选参考时间戳的正时间差：

$$
\Delta t_i=t_{i+1}-t_i,
$$

再使用中位数：

$$
f_{obs}=\frac{1}{\operatorname{median}(\Delta t_i)}.
$$

中位数比平均数不容易被少量丢帧或长间隔拉偏。如果原始相机为 30 Hz，但任务抽帧后实际
入选 5 Hz，自动值跟随约 5 Hz，不会继续使用原始 30 Hz。

### 6.2 padding 后的完整时间域

代码先取：

```text
support_start = min(首个 PnP 时间, 最早的各目观测时间)
support_end   = max(最后 PnP 时间, 最晚的各目观测时间)
```

然后当前实现把初始化端点扩展为：

```text
padded_start = support_start - 2 × time_padding_s
padded_end   = support_end   + 2 × time_padding_s
```

两端额外样本分别复制首、末 PnP 位姿，作为初始稀疏拟合的端部条件。联合优化开始后，
spline 控制系数仍然 active，并不会把端部永久固定在复制值上。

初始控制系数通过 `initPoseSplineSparse(..., lambda=1e-4)` 求得。它建立均匀 knot，并求解
“六维 PnP 曲线拟合误差 + 很弱的二阶曲线正则”。这里的 `1e-4` 只用于生成平滑初值；进入
RS 联合问题后，控制系数全部 active，最终运动先验使用 task 中独立配置的平移、旋转权重。
第三个实参是有效 segment 数，不是 knot-vector 元素数。order 为 $k$、segment 数为 $N_s$
时，控制系数数为 $N_s+k-1$，knot 数为 $N_s+2k-1$。

`time_padding_s` 本身还必须严格大于任一相机可能的首末行跨度：

$$
time\_padding\_s>(H-1)\max|line\_delay|.
$$

这保证行时间在允许范围内变化时，第一幅和最后一幅图的角点仍能查询到有效 spline。

### 6.3 请求段数和实际段数

完整 padding 时间域长度为：

$$
D=padded\_end-padded\_start.
$$

代码计算：

```text
requested_segment_count = round(D × requested_rate_hz)
minimum_segment_count   = 2 × spline_order
segment_count           = max(minimum_segment_count,
                              requested_segment_count)
```

其中：

- 自动模式的 `requested_rate_hz = f_obs`；
- 显式模式的 `requested_rate_hz = knots_per_second`；
- 默认四阶时最少为 8 段。

### 6.4 一个完整数值例子

假设：

```text
最终入选时间：0.0, 0.2, 0.4, ..., 10.0 s
中位间隔：0.2 s
自动频率：5 Hz
time_padding_s：0.5 s
spline_order：4
```

当前代码在两端各扩展 `2 × 0.5 = 1.0 s`：

```text
padded domain = [-1.0, 11.0]
D = 12.0 s
requested segments = round(12 × 5) = 60
minimum segments = 2 × 4 = 8
actual segment_count = 60
effective rate = 60 / 12 = 5 segments/s
```

有效 PnP pose sample 可能只有 51 个，但段数仍可为 60。pose sample 是初始拟合数据，
segment 是连续模型的时间划分，它们没有一一对应关系。联合问题还有每帧大量逐行角点残差
和运动先验，不能按 PnP 样本数量硬截断 segment 数。

### 6.5 为什么按 pose 数硬截断会改变结果

若把上例强行限制为 51 段：

```text
effective rate = 51 / 12 = 4.25 segments/s
```

轨迹已经比请求的 5 Hz 更平滑，knot 相对观测时间的位置也会改变。角点在不同 spline
segment 中求值，行时间与控制系数的 Jacobian 随之变化。优化器可能在轨迹和行时间之间
重新分配误差，使结果随段数和 knot 网格位置明显变化。

文档中的“数值共振”指这种参数相关性和对 knot 网格相位的敏感性，不是机械结构发生物理
共振。

## 7. 为什么联合问题计算轨迹二阶导数

### 7.1 一阶和二阶导数的直观含义

先看一维位置 $x(t)$：

| 数学量 | 直观含义 |
|---|---|
| $x(t)$ | 位置 |
| $\dot x(t)$ | 速度 |
| $\ddot x(t)$ | 加速度，表示速度变化有多快 |

如果只要求曲线穿过离散位姿样本，观测之间可以存在很多不同形状：

```text
平滑通过： ●───────●
不必要摆动：●─∿─∿─●
```

两条曲线都可能在图像时间戳处产生相似投影，但第二条在图像之间包含没有数据支持的快速
加减速。RS 角点又会在帧内不同时间查询这条曲线，因此这种自由度会直接影响行时间估计。

二阶运动先验惩罚过大的二阶变化，使无观测区间倾向于局部匀速，而不是任意弯曲。它允许
真实加速运动，只是要求明显弯曲必须由足够的像素证据支持。

用每隔 1 秒记录一次的一维位置举例：

| 时间 | 轨迹 A 的位置 | A 的相邻位移 | 轨迹 B 的位置 | B 的相邻位移 |
|---:|---:|---:|---:|---:|
| `0 s` | `0 m` | — | `0 m` | — |
| `1 s` | `1 m` | `1 m` | `1 m` | `1 m` |
| `2 s` | `2 m` | `1 m` | `3 m` | `2 m` |

轨迹 A 每秒都移动 1 m，速度不变，二阶变化为 0；轨迹 B 的相邻位移从 1 m 变为 2 m，
存在加速度，二阶变化不为 0。运动先验会给轨迹 B 更大的代价，但如果逐行角点确实支持这次
加速，重投影误差的改善仍可以超过这部分代价，优化器仍会保留加速运动。

### 7.2 二阶导数从哪里来

它不是通过图像直接“检测”出的加速度，也不是新增一个需要估计的独立参数。B-spline 在每个
segment 上都是已知形式的多项式；给定当前控制系数后，代码可以对该多项式解析求导，得到
任意时刻的一阶和二阶导数。优化器每次改变控制系数，轨迹、轨迹二阶导数、运动先验代价和
它们对控制系数的 Jacobian 都会一起更新。

因此联合问题中真正发生的是：

```text
优化变量：K/D/T、行时间、spline 控制系数
                         │
                         ├─ 逐角点投影 → 重投影残差
                         └─ spline 解析二阶导数 → 运动先验残差
```

二阶导数用于约束同一组 spline 控制系数，不作为一项新的标定结果输出。

### 7.3 当前代码实际加入什么

在 `_add_design_variables()` 中，代码构造：

```python
weights = diag([
    motion_translation_weight,
    motion_translation_weight,
    motion_translation_weight,
    motion_rotation_weight,
    motion_rotation_weight,
    motion_rotation_weight,
])
addMotionErrorTerms(problem, spline_dv, weights, 2)
```

最后一个参数 `2` 表示对 spline 的二阶导数建立运动误差。底层
[`BSplineMotionErrorFactory.hpp`](../src/kalibr/trajectory/aslam_splines/include/aslam/backend/implementation/BSplineMotionErrorFactory.hpp)
逐个有效 segment 调用 `segmentIntegral(..., W, 2)`，构造与下式概念等价的二次项：

$$
J_{motion}
\approx
\sum_{segment}
\int
\left(\frac{d^2\boldsymbol\xi(t)}{dt^2}\right)^T
\mathbf W
\left(\frac{d^2\boldsymbol\xi(t)}{dt^2}\right)dt.
$$

$\boldsymbol\xi(t)$ 是 pose spline 使用的 6 维局部曲线坐标，前三维对应平移，后三维对应
旋转向量。因此：

- 平移部分可直观理解为对加速度进行平滑；
- 旋转部分用于平滑角运动；
- 它不是 IMU 测得的物理加速度或角加速度，不应当当作 IMU 测量使用。

### 7.4 为什么 `spline_order` 至少为 3

order 为 2 时是一次分段多项式，每段二阶导数为零，连接处又发生不光滑跳变，无法为当前
积分型二阶先验提供合适模型。order 为 3 时是二次多项式，二阶导数可以按段计算；默认
order 4 的三次曲线具有更平滑的一、二阶变化，所以当前选择：

```text
最低允许：3
默认：4
```

### 7.5 运动先验约束哪些地方

它对每个有效 segment 都加入目标函数，主要帮助：

- 两幅图之间没有直接观测的时间区间；
- 轨迹开始和结束处；
- knot 较密时抑制局部摆动；
- 防止 spline 完全吸收角点噪声或行时间效应。

它不是初始化结束后就丢弃的临时约束。第一次联合求解和最终角点过滤后的重建求解都使用
同样的运动先验。

### 7.6 两个默认权重怎样理解

默认值为：

```text
motion_translation_weight = 1e-5
motion_rotation_weight    = 1e-2
```

它们是二阶运动误差的数值信息权重：

- 增大权重：轨迹更倾向平滑，快速变化需要更强图像证据；
- 减小权重：轨迹更灵活，更容易拟合快速运动，也更容易吸收噪声；
- 当前校验要求两者为有限正数，不能设为零。

平移和旋转使用不同坐标、单位和数值尺度，不能因为 `1e-2 / 1e-5 = 1000`，就宣称旋转
物理上被约束得比平移强 1000 倍。

## 8. 行时间为什么会与轨迹、K/D 和外参互相补偿

单个角点预测可概括为：

$$
\hat{\mathbf u}_{ij}
=\pi\!\left(
\mathbf K_i,\mathbf D_i,
{}^{c_i}_{c_0}\mathbf T,
{}^{c_0}_{t}\mathbf T(t_{ij}),
\mathbf p_j
\right),
$$

其中：

$$
t_{ij}=t_i+y_{ij}\tau_i.
$$

改变 $\tau_i$ 会改变查询轨迹的时刻；改变 spline 控制系数会改变该时刻的位姿。某些数据
中，两种改变可以产生相似像素移动：

```text
错误行时间 + 更灵活轨迹 ≈ 相似重投影
```

轨迹太平滑时，未表示的真实运动也可能被 K/D、外参或行时间吸收：

```text
轨迹带宽不足 → 结构化像素残差 → K/D/T/tau 被迫补偿
```

因此不能只选择 RMS 最小的 knot 设置。还必须检查物理参数在合理 knot 率和先验变化下是否
稳定，以及可观性是否显示行时间与其他参数形成弱模态。

## 9. 每个配置字段改变什么

| 字段 | 默认 | 增大后的主要效果 | 太小/太大的风险 |
|---|---:|---|---|
| `spline_order` | `4` | 每段多项式阶次和局部支持增加 | 太低不能稳定表达二阶运动；太高增加局部耦合和计算量 |
| `knots_per_second` | 自动 | segment 更密、轨迹更灵活 | 太低欠拟合运动；太高吸收噪声或行时间 |
| `time_padding_s` | `0.5 s` | 端部时间支持更宽 | 太小角点时间越界；过大增加时间域和 segment 数 |
| `motion_translation_weight` | `1e-5` | 平移轨迹更平滑 | 太小局部摆动；太大压制真实平移加速度 |
| `motion_rotation_weight` | `1e-2` | 旋转轨迹更平滑 | 太小局部摆动；太大压制真实角运动变化 |
| `max_abs_line_delay_s` | 估计时必填 | 行时间搜索和支持范围更宽 | 太小产生边界解；过大加重行时间/轨迹相关性并要求更大 padding |

修改这些字段会改变数学问题，不属于单纯性能参数。

## 10. 怎样做 spline 敏感性实验

推荐先保留自动设置作为基准，再选择少量有物理意义的显式 rate。例如以自动观测率
$f_{obs}$ 为中心：

```text
0.5 × f_obs
1.0 × f_obs
2.0 × f_obs
```

所有组固定：

- 输入帧和最终角点集合；
- K/D/T 初始化；
- 行时间 seed 和 `max_abs_line_delay_s`；
- `spline_order` 和 `time_padding_s`；
- 两个运动先验权重；
- 特征 sigma、鲁棒核和过滤策略。

每组至少比较：

| 类别 | 需要检查 |
|---|---|
| 轨迹结构 | 请求 rate、请求段数、最小段数、实际段数、有效 rate |
| RS 参数 | 每目 `line_delay_s`、首末行跨度、距边界余量 |
| 光学与外参 | K/D、`T_cn_cnm1` 是否漂移 |
| 拟合 | 每目 RMS、逐帧残差、残差随行坐标和运动速度的关系 |
| 可观性 | rank、弱模态、行时间与轨迹/K/D/T 的相关方向 |
| 泛化 | 独立测试集指标，而不只看训练数据 RMS |

可信结果应在一段合理的轨迹带宽内保持物理参数基本稳定。若 RMS 变化很小，但行时间随着
segment 密度大幅变化，应判定行时间与轨迹尚未分离，而不是挑选 RMS 最小的一组作为真值。

## 11. 代码中的完整调用链

正式入口：

[`kalibr_calibrate_rs_camera_system`](../src/kalibr/calibration/kalibr/python/kalibr_calibrate_rs_camera_system)

调用顺序为：

```text
读取 task 与 rolling_shutter 配置
  → 各目检测角点并初始化 K/D
  → 建多目共视图并初始化相邻 baseline
  → SystemRsCalibrator(...)
      → _observation_timestamps()
      → _initial_target_poses()
      → make_pose_spline()
  → build_problem()
      → 注册 spline/K/D/T/line-delay design variables
      → addMotionErrorTerms(..., order=2)
      → 按逐角点时间建立重投影残差
  → optimize()
  → 可选角点过滤、重建同一问题并再次求解
  → 可观性、边界检查、指标与结果发布
```

关键文件：

| 文件 | 作用 |
|---|---|
| [`SystemCalibrator.py`](../src/kalibr/calibration/kalibr/python/kalibr_rs_camera_calibration/SystemCalibrator.py) | spline 初始化、联合状态、逐角点残差、运动先验与求解 |
| [`BSplinePose.hpp`](../src/kalibr/trajectory/bsplines/include/bsplines/BSplinePose.hpp) | 位姿 B-spline 底层接口，order 对应 degree `order-1` |
| [`BSplineMotionErrorFactory.hpp`](../src/kalibr/trajectory/aslam_splines/include/aslam/backend/implementation/BSplineMotionErrorFactory.hpp) | 按 segment 构造二阶运动误差 |
| [`task.py`](../src/python/kalibr_no_ros/task.py) | task 参数转换为正式后端命令行参数 |
| [`validation.py`](../src/python/kalibr_no_ros/validation.py) | order、padding、行时间支持范围等输入检查 |

## 12. 常见误解

### “每幅图已经有 PnP 位姿，为什么还要 spline？”

PnP 只给出离散图像时刻的初值。RS 每个角点都有不同时间，必须在帧内连续查询位姿。

### “一个 pose sample 就对应一个 knot 吗？”

不对应。pose sample 用于初始化拟合，knot/segment 定义连续模型，control coefficient 才是
联合优化变量。

### “knot 越多越准确吗？”

不一定。更多 segment 可以表示更快运动，也更容易吸收噪声和行时间误差。

### “运动先验是不是 IMU？”

不是。它没有读取加速度计或陀螺仪，只对视觉轨迹的二阶变化增加平滑代价。

### “二阶先验是不是强制匀速？”

不是。它惩罚过大的二阶变化，但允许图像证据支持的加速和转动变化。

### “RMS 最小的 `knots_per_second` 就最好吗？”

不能这样判断。还要检查行时间、K/D/T、可观性及独立测试结果是否对 spline 设置稳定。

### “`time_padding_s` 只是为了多保存一点轨迹吗？”

不是。它保证正负行时间更新后，序列首尾角点仍位于 spline 可求值的时间域内，并参与
segment 数计算。

## 13. 推荐阅读顺序

1. 先阅读本文第 1～7 节，理解时间、segment 和二阶先验；
2. 再阅读[滚动快门相机标定](ROLLING_SHUTTER_CAMERA_CALIBRATION_ZH.md)的正式流程和配置；
3. 配置字段可直接对照[完整注释示例](../config/examples/v1.0.0/all_params/camera_rolling_shutter_calibration_task_full.yaml)；
4. 需要修改代码时查看 `SystemCalibrator.py`；
5. 需要判断行时间可信度时执行固定行时间、不同 knot rate 和独立测试集对照。
