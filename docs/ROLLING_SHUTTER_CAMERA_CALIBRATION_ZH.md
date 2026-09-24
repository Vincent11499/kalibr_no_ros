# Rolling Shutter 单目／多目相机标定

本页说明纯视觉 `camera_rolling_shutter_calibration`。正式入口
`kalibr-noros calibrate cameras-rs` 支持单目、双目和按列表排列的多目系统；临时对照入口
`kalibr-noros calibrate native-rs-cameras` 只支持单目，用于核对 Kalibr 上游
`kalibr_calibrate_rs_cameras` 的行为。两条入口共用 task、数据集、标定板和初始化格式，
普通 `calibrate cameras` 以及 Camera–IMU 任务不受影响。

## 1. 使用方法

复制[简洁示例](../config/examples/v1.0.0/camera_rolling_shutter_calibration_task.yaml)，
把数据路径、相机 ID、模型和行时间范围换成本机实际值。目录数据集按 `dataset.yaml`
映射相机 ID，不填写 topic；bag 输入必须在每个相机项中填写 topic。

```bash
kalibr-noros validate \
  --config camera_rolling_shutter_calibration_task.yaml
kalibr-noros calibrate cameras-rs \
  --config camera_rolling_shutter_calibration_task.yaml \
  --output-dir result
```

同一个 task 若只有一台相机，可以执行原生单目对照：

```bash
kalibr-noros calibrate native-rs-cameras \
  --config camera_rolling_shutter_calibration_task.yaml \
  --output-dir result
```

执行前需从示例中删去 cam1 及其 `rolling_shutter.cam1`，并删去
`synchronization_tolerance_s`、轨迹样条、运动先验、异常点过滤等正式求解器专用字段。
该临时入口的 `calibration` 只接受 `window_half_size_px`、`max_displacement_px`、
`max_iterations`、`feature_sigma_px` 和值为 `false` 的 `freeze_intrinsics`。
`native-rs-cameras` 遇到两台及以上相机会在求解前拒绝输入，输出名称增加 `native_`
前缀，避免覆盖正式 `cameras-rs` 结果。该临时入口不实现轨迹导出，必须保持
`output.export_poses: false`；需要 `poses.csv` 时使用正式 `cameras-rs`。两条入口共用
task schema，并不表示任一具体 YAML 中的后端专用参数都可直接互换。

两条项目公开入口都只支持下列两种模型：

| task 模型名 | `native-rs-cameras` 中的原生 RS geometry |
|---|---|
| `pinhole-equi` | `EquidistantPinholeRs` |
| `pinhole-radtan` | `DistortedPinholeRs` |

因此原生 Kalibr RS 单目对照不接受 `pinhole-radtan5`、`pinhole-radtan8`、
`pinhole-opencv-fisheye` 或 omni 模型。内部脚本为上游兼容保留了
`pinhole-equi-rs`、`pinhole-radtan-rs` 等别名，但它们不是
`kalibr-noros calibrate native-rs-cameras` 的公开 task 契约。在 RS 专用入口中，
快门类型由 job 决定，task 不使用 `pinhole-equi-rs` 这类内部名称。

`native-rs-cameras` 的“临时”指用途是过渡性的算法对照，不表示代码位于
临时目录。它当前仍是项目源码和安装内容的一部分：

- 原生命令入口：
  [`kalibr_calibrate_rs_cameras`](../src/kalibr/calibration/kalibr/python/kalibr_calibrate_rs_cameras)；
- 原生单目求解器：
  [`RsCalibrator.py`](../src/kalibr/calibration/kalibr/python/kalibr_rs_camera_calibration/RsCalibrator.py)；
- 公开 CLI 路由和 task 适配：
  [`cli.py`](../src/python/kalibr_no_ros/cli.py) 和
  [`task.py`](../src/python/kalibr_no_ros/task.py)；
- 构建时由 [`ProjectBuild.cmake`](../cmake/ProjectBuild.cmake) 复制并安装到
  `libexec/kalibr`。

它属于 `src/kalibr` 中保留的原生标定实现，但不是项目正式的多目 RS
求解路径。正式单目／双目／多目 RS 任务使用 `cameras-rs`；只有明确做上游单目
对照时才使用 `native-rs-cameras`。

## 2. `validate` 与 `evaluate` 的作用和边界

### 2.1 `validate`：求解前检查输入

```bash
kalibr-noros validate \
  --config camera_rolling_shutter_calibration_task.yaml
```

`validate` 读取 task、标定板、初始化文件和数据集，但不检测 AprilGrid 角点、不建立
重投影问题，也不启动任一 RS 优化器。它检查：

- `schema_version`、`kind`、`job` 和字段集合；
- `dataset.type/path` 与实际 reader 类型是否一致；
- 目录数据集中的相机 ID、图像路径和时间戳，或 bag 的 topic；
- 每目是否有入选图像、图像分辨率是否一致、显式分辨率是否匹配；
- `rolling_shutter` 的键集合是否与 `cameras[].id` 完全一致；
- `line_delay_s`、`estimate`、`max_abs_line_delay_s` 的类型、范围和关系；
- 正式 system 后端的 `time_padding_s` 是否覆盖允许的首末行跨度；
- 初始化文件的版本、模型、相机数量和必要字段。

校验结果打印为 `kind: input_validation` 的 JSON；需要留档时可以使用：

```bash
kalibr-noros validate \
  --config camera_rolling_shutter_calibration_task.yaml \
  --output validation.json
```

退出码 0 表示结构和时序检查通过，2 表示检查失败。通过不代表能够检测到标定板，也不代表
运动足以辨识行时间，更不代表优化会收敛、可观性为满秩或结果达到精度要求。

公开的独立 `validate` 命令没有 `--backend` 参数，因此对
`camera_rolling_shutter_calibration` 按正式 `cameras-rs` 的公共结构和样条支持做检查；它
不会提前应用 `native-rs-cameras` 的“必须单目、禁止 `export_poses`、只允许部分
`calibration` 字段”等限制。实际执行 `calibrate native-rs-cameras` 时，CLI 会在解码图像和
运行求解器之前补做这些 native 专用检查。因此，“独立 `validate` 通过”不等于“同一份
完整 system 配置可以直接交给 native 后端”。

### 2.2 `evaluate`：利用已有证据重新评价和生成报告

```bash
kalibr-noros evaluate \
  --run result/camera_rolling_shutter_calibration_cam0.yaml \
  --config evaluation.yaml \
  --dataset /new/location/of/the/same/dataset \
  --output-dir evaluated
```

`evaluate` 不重新检测角点、不重新初始化样条、不重新估计 K/D/行时间，也不调用
`SystemRsCalibrator` 或 `RsCalibrator`。它接受具名结果 YAML，或能够唯一定位结果的运行
目录，并要求评价输出目录与源结果、替代数据集相互独立。

它有两种证据模式：

| 标定时保存的证据 | `evaluate` 能做什么 | 不能做什么 |
|---|---|---|
| `output.archive_observations: true` | 校验归档 checksum；读取最终使用帧、角点、预测、残差、逐角点 RS 时刻和位姿；重新计算指标、判定、HTML/PDF、普通去畸变和双目图 | 不能修改最终预测、轨迹、K/D 或行时间 |
| 只有 `output.save_metrics: true` | 读取已有 `metrics.json`，修改验收规则并重新生成汇总报告 | 不能改变校正参数和配对容差，不能生成需要角点或原图的可视化 |

`--dataset` 只在原数据移动后为可视化提供**同一数据集**的图像。程序会核对归档中的
`dataset_id`、帧索引和整数纳秒时间戳，不能用另一组数据替换后冒充离线重评价。

对 RS 结果，归档充分时可以重新汇总：

- 每目最终二维重投影 RMS；
- 普通光学 Alignment；
- 双目／多目存在共同角点时的 `rs_compensated_pair_residual`；
- 新的业务 `assessment.rules` 和参考等级；
- 报告中的已保存行时间、首末行跨度、可观性摘要和求解器信息。

`evaluate` 复用的是求解时保存的逐角点预测，因此不能用它测试新的行时间初值、固定
$\tau=0$、反向扫描、不同 knot 密度或另一个 RS 后端。做这些算法对照必须创建新的标定
运行。

## 3. 两条 RS 命令的实际调用链

`cameras-rs` 与 `native-rs-cameras` 使用相同的公开 job：

```yaml
job: camera_rolling_shutter_calibration
```

后端由 CLI 子命令选择，不写入 task：

```text
kalibr-noros calibrate cameras-rs
  → cli.py
  → task.run_task(..., solver_backend=system)
  → task._rs_camera_arguments()
  → kalibr_calibrate_rs_camera_system
  → SystemRsCalibrator
  → 联合 LM、可观性检查、正式结果

kalibr-noros calibrate native-rs-cameras
  → cli.py 设置 _rs_solver_backend=native
  → task.run_task(..., solver_backend=native)
  → task._native_rs_camera_arguments()
  → kalibr_calibrate_rs_cameras
  → RsCalibrator
  → 原生 DogLeg、adaptive knot、单目对照结果
```

所以答案是：**是，`native-rs-cameras` 直接调用项目源码中的
`src/kalibr/calibration/kalibr/python/kalibr_calibrate_rs_cameras`**。CLI 适配层把公开
task 转换成该脚本的原生参数，包括实际入选帧率、模型、行时间 seed、是否估计行时间、
检测参数、线程预算和初始化文件；它没有重新实现一套名字相似的求解器。

正式 system 后端对应另一支独立实现
`kalibr_calibrate_rs_camera_system`/`SystemRsCalibrator`。两者共享底层 aslam 相机表达式、
连续时间 spline 和 Optimizer2 基础设施，但状态参数化、误差权重、轨迹节点策略、信赖域
算法和多目能力不同。

## 4. `pinhole-equi` 与 `pinhole-radtan` 单目投影

两种后端对公开模型的映射相同：

| task 模型 | 原生 RS geometry | 内参 | 畸变 |
|---|---|---|---|
| `pinhole-equi` | `EquidistantPinholeRs` | $[f_x,f_y,c_x,c_y]$ | $[k_1,k_2,k_3,k_4]$ |
| `pinhole-radtan` | `DistortedPinholeRs` | $[f_x,f_y,c_x,c_y]$ | $[k_1,k_2,p_1,p_2]$ |

设目标角点在自己的坐标系中为 $\mathbf P_j$，该角点时刻的相机位姿将其转换为：

$$
\begin{bmatrix}X\\Y\\Z\end{bmatrix}
= {}^{cam}_{target}\mathbf T(t_{ij})\mathbf P_j,
\qquad x=\frac XZ,\quad y=\frac YZ.
$$

`pinhole-radtan` 令 $r^2=x^2+y^2$：

$$
L(r)=1+k_1r^2+k_2r^4,
$$

$$
\begin{aligned}
x_d &=xL(r)+2p_1xy+p_2(r^2+2x^2),\\
y_d &=yL(r)+p_1(r^2+2y^2)+2p_2xy.
\end{aligned}
$$

`pinhole-equi` 令 $r=\sqrt{x^2+y^2}$、$\theta=\arctan(r)$：

$$
\theta_d=\theta(1+k_1\theta^2+k_2\theta^4+k_3\theta^6+k_4\theta^8),
$$

$$
\begin{bmatrix}x_d\\y_d\end{bmatrix}
=
\begin{cases}
\dfrac{\theta_d}{r}\begin{bmatrix}x\\y\end{bmatrix},&r>0,\\[5pt]
\begin{bmatrix}x\\y\end{bmatrix},&r=0.
\end{cases}
$$

最后都投影为：

$$
u=f_xx_d+c_x,\qquad v=f_yy_d+c_y.
$$

Rolling Shutter 并没有改变以上光学投影公式；它改变的是计算
${}^{cam}_{target}\mathbf T$ 时使用的时间。检测角点为
$\mathbf z_{ij}=[u_{ij},v_{ij}]^\mathsf T$，两种后端的物理像素残差都可以写成：

$$
\mathbf e_{ij}
=\mathbf z_{ij}-
\pi\!\left(K,D,
{}^{cam}_{target}\mathbf T(t_{ij})\mathbf P_j\right),
$$

$$
t_{ij}=t_i+v_{ij}\tau.
$$

因此 `pinhole-equi` 与 `pinhole-radtan` 的 RS 求解架构相同，区别只在 $\pi(K,D,\cdot)$
内部采用角度多项式还是径向－切向畸变。模型选择应由镜头成像特性和独立测试决定，不能
根据哪个模型把训练 RMS 压得更低就直接认定。

### 4.1 单目 `pinhole-equi` task

```yaml
schema_version: "1.0.0"
kind: calibration_task
job: camera_rolling_shutter_calibration
dataset:
  type: directory
  path: /data/mono_rs_YYMMDD_hhmm
target:
  path: aprilgrid.yaml
cameras:
  - id: cam0
    model: pinhole-equi
rolling_shutter:
  cam0:
    line_delay_s: 8.0e-6
    estimate: true
    max_abs_line_delay_s: 2.0e-5
calibration:
  window_half_size_px: 7
  max_displacement_px: 1.0
  max_iterations: 80
  feature_sigma_px: 1.0
output:
  archive_observations: true
  export_poses: false
```

正式求解：

```bash
kalibr-noros calibrate cameras-rs --config mono_equi_rs.yaml --output-dir result
```

原生对照可直接使用以上精简 task：

```bash
kalibr-noros calibrate native-rs-cameras --config mono_equi_rs.yaml --output-dir result
```

### 4.2 单目 `pinhole-radtan` task

配置结构相同，只把模型改为：

```yaml
cameras:
  - id: cam0
    model: pinhole-radtan
```

如果提供初始化，`distortion_coeffs` 必须严格使用四参数顺序：

```yaml
cameras:
  cam0:
    intrinsics: [fx, fy, cx, cy]
    distortion_coeffs: [k1, k2, p1, p2]
```

分别运行：

```bash
kalibr-noros calibrate cameras-rs --config mono_radtan_rs.yaml --output-dir result
kalibr-noros calibrate native-rs-cameras --config mono_radtan_rs.yaml --output-dir result
```

同一输出根目录中，native 对照结果会增加 `native_` 前缀，不覆盖正式 system 结果。

## 5. 时间语义

纯视觉 RS 任务把相机时间戳定义为原图第 0 行的曝光结束时刻。原图角点坐标为
$(x,y)$，每行时间为 $\tau$，角点使用的时刻为：

```text
t_corner = t_camera_timestamp + y × tau
```

正 $\tau$ 表示图像 y 增大时采样更晚，负值表示扫描方向相反。高度为 $H$ 时：

```text
首末行跨度 = (H − 1) × |tau|
```

该跨度不包含垂直消隐，也不是单行曝光时长。以 2160 行、8 μs/行为例，首末行跨度为
`2159 × 8 μs = 17.272 ms`。若曝光时长为 1 ms，第 0 行曝光中点比“第 0 行曝光结束”
早 0.5 ms；固定曝光造成的公共时间平移在纯视觉自由轨迹中属于时间参考重参数化，
不应重复当成每行时间。

有些分析采用中间行 $y_c=(H-1)/2$：

```text
t_corner = t_center + (y − y_c) × tau
t_center = t_row0 + y_c × tau
```

在纯视觉且轨迹时间原点可以同步平移时，两式给出相同角点相对时差；结果文件仍统一写
`reference_row_px: 0.0` 和 `timestamp_reference: row0_exposure_end`，保证与驱动 PTS
定义一致。Camera–IMU RS 任务还包含跨传感器整体时移，使用它自己的参考行约定，不能把
两类任务的 timeshift 数字直接互换。

## 6. 正式 `cameras-rs` 的实现原理

`cameras-rs` 为所有相机建立一条共享的连续标定板轨迹，同时联合优化：

- 每目 pinhole 投影参数和畸变；
- 相邻相机外参 `T_cn_cnm1`；
- 每目独立的行时间；
- 连续目标位姿样条控制点。

每台相机使用自己的源时间戳。对第 i 台相机，先按列表顺序累计相邻外参：

```text
T_ci_c0 = T_ci_c(i-1) × ... × T_c1_c0
T_ci_target(t) = T_ci_c0 × T_c0_target(t)
```

因此第三台及后续相机不会错误地只使用最后一段 baseline。多目建 view 时仍要求时间差
在 `synchronization_tolerance_s` 内并具有共同全局角点 ID。

估计模式使用光滑有界状态：

```text
tau = max_abs_line_delay_s × tanh(q)
```

$q$ 是优化变量。该写法对更新和 Jacobian 一致，不在更新后硬截断参数，也没有给行时间
增加先验残差。初值按 `atanh(seed / bound)` 转换；估计接近范围边缘时任务失败并要求
检查激励、范围和可观性。`estimate: false` 固定 `line_delay_s`，可构造同一求解器的
0、正值和反向值对照。要让这些控制实验的样条支持完全相同，应保留相同的
`max_abs_line_delay_s`。

目标轨迹默认使用四阶样条。由于联合问题始终加入二阶运动先验，`spline_order` 至少为 3。
正式 system 求解器没有原生后端的 adaptive knot 插入，自动
knot 率因此直接取实际入选观测时间戳的中位频率，即
`1 / median(diff(selected_timestamps))`。自动段数先按含两端 padding 的时间跨度乘以上述
频率计算，最少为 `2 * spline_order`。段数覆盖包含两端 padding 的完整样条时间域，因此
可以略多于成功初始化的目标位姿数；不能再按位姿数硬截断。硬截断会改变实际 knot 间距及
knot 网格相对观测的相位，可能使行时间与轨迹出现数值共振。这样每个实际入选观测间隔及
端部支持域都有一致轨迹带宽，同时不会因原始相机频率高就对抽帧数据建立过密轨迹。旧的
“入选频率除以 3”会在低频抽样数据上进一步压低轨迹带宽，使未建模运动进入重投影误差，
甚至由 K/D 或行时间吸收，因此不再作为默认值。

样条加入原生分段二阶运动先验，默认平移权重 `1e-5`、旋转权重 `1e-2`。这些权重约束无
图像区间和端部运动，行时间、0 延迟及固定延迟对照必须使用相同样条和先验。需要做敏感性
分析时显式设置 `knots_per_second`；显式值同样保留 `2 * spline_order` 下限。报告中应
同时保存请求频率、请求段数、实际段数和下限，不能只保留最有利的设置。
`time_padding_s` 必须严格大于任一相机在配置范围内的首末行跨度，防止优化更新后角点
时刻离开样条支持域。

联合求解沿用 Optimizer2 的原生停止条件：参数增量 `dX` 或目标函数变化绝对值 `|dJ|`
达到阈值即可停止。若实际迭代数达到 `max_iterations`，则表示预算耗尽而不是满足上述
停止条件；任务会失败，不发布正式结果，错误中同时记录 `iterations`、`dXFinal` 和
`dJFinal`。临时原生单目入口对初始求解和每次 adaptive knot 重求解执行相同检查；线性
求解失败或耗尽迭代预算时也不会序列化标定结果。单纯调大迭代上限不能证明物理参数
可信，仍需检查行时间范围、样条／先验敏感性和可观性。

完整字段、默认值、单位和范围见
[完整注释示例](../config/examples/v1.0.0/all_params/camera_rolling_shutter_calibration_task_full.yaml)。

## 7. 初始化

RS 相机任务复用 `camera_calibration_initialization`：

```yaml
initialization:
  path: initialization_stereo.yaml
```

相机块按 task 列表映射到 cam0、cam1……，相邻外参仍使用 `T_cn_cnm1`，表示前一相机
到当前相机。可以只提供部分 K/D 或外参，所有值都作为联合求解的起点。
`freeze_intrinsics: true` 另行控制最终联合求解是否固定 K/D，并要求至少双目与
完整 K/D seed。初始化文件不包含行时间；行时间只在 task 的 `rolling_shutter` 中配置，
便于用同一光学初值执行 0、固定和自由行时间对照。

## 8. 原生 Kalibr RS 单目实现与异同

### 8.1 正式 `cameras-rs` 的状态与目标函数

单目时，正式 system 问题的活动状态为：

```text
K、D、有界行时间 latent q、目标位姿 spline 控制点
```

双目／多目时再增加相邻 `T_cn_cnm1`。标定板三维角点固定，不作为自由 landmark。行时间
使用：

$$
\tau=\tau_{max}\tanh(q),
$$

因此整个优化过程中都有 $|\tau|<\tau_{max}$，且对 $q$ 的 Jacobian 连续。像素协方差固定
为 `feature_sigma_px² I`，可选 Blake–Zisserman M-estimator。目标函数概念上为：

$$
J_{system}
=\sum_{i,j}\rho\!\left(
\frac{\|\mathbf e_{ij}\|^2}{\sigma_{feature}^2}
\right)
+J_{motion}^{(2)}.
$$

$J_{motion}^{(2)}$ 是 spline 的二阶平移和旋转运动先验，用于约束图像间隔和端部，而不是
行时间先验。求解器使用 Optimizer2、Schur complement、Block Cholesky 和
Levenberg–Marquardt。可选最终四倍标准差角点过滤会重建问题并重新求解。

完成后，正式后端还会在最终状态上执行只读可观性分析。报告的物理块包括 K、D、相邻
外参以及 active 的行时间 latent；hard rank 失败不发布成功结果，行时间达到配置范围
99.5% 也作为不可信边界解拒绝。

### 8.2 原生 `kalibr_calibrate_rs_cameras` 的状态与目标函数

原生单目后端的活动状态为：

```text
K、D、原生 shutter lineDelay、相机/目标位姿 spline 控制点
```

它直接使用 RS camera geometry 的 `shutterDesignVariable`。公开包装器总是把 task 中的
`line_delay_s` 显式传给原生脚本；若绕过包装器且不提供 seed，上游默认才会用
`1 / (实际入选帧率 × 图像高度)` 初始化。

原生行时间是无界标量。`max_abs_line_delay_s` 只在求解完成后检查：

$$
|\hat\tau|\leq\tau_{max}.
$$

它不等价于 system 后端的 $\tau_{max}\tanh(q)$，优化中的 DogLeg 试探值可能越过最终
允许范围。为保证时间表达式稀疏模式和 spline 端部支持，包装器保留固定时间 buffer 和
padding，但不向 $\tau$ 增加先验。

原生每个 RS 角点使用 `reprojectionErrorAdaptiveCovariance`。该误差项根据连续轨迹下的
局部图像运动调整有效协方差；目标函数仍包含 spline motion prior，但权重、误差标准化和
正式 system 后端不同。求解器使用 Optimizer2、Schur complement、Block Cholesky 和
DogLeg。初次联合求解后，`ReprojectionErrorKnotSequenceUpdateStrategy` 根据残差决定是否
重排 spline knot；每次变化后重建完整问题并再次求解，直至无需更新或达到次数限制。

原生后端没有 system 后端等价的多目外参状态、最终角点四倍标准差过滤、Blake–Zisserman
开关和 reduced-Fisher 可观性输出。它仍会重新计算未加权二维像素残差，用于与正式后端
统一报告：

$$
RMS=\sqrt{\frac1N\sum_{i,j}\|\mathbf e_{ij}\|^2}.
$$

### 8.3 共同点与差异

| 项目 | `cameras-rs` | `native-rs-cameras` |
|---|---|---|
| 相机数量 | 1 台或多台 | 恰好 1 台 |
| 多目外参 | 与 K/D、轨迹、每目行时间联合优化 | 无 |
| 行时间范围 | `bound × tanh(q)`，求解过程中光滑有界 | 原生无界标量，求解后检查配置范围 |
| 像素误差 | 固定 `feature_sigma_px`，可选鲁棒核 | 保留原生随运动自适应协方差 |
| 轨迹 | 观测率感知的共享样条，分段二阶运动先验 | 原生自适应 knot、运动先验与 DogLeg |
| 输出 | 多目正式结果、真实逐角点残差和可观性 | 单目结构化 K/D/tau/RMS；原生 Hessian 秩报告不可用 |

共同点包括：

- 同一公开 task schema、目录/bag reader、AprilGrid 和亚像素检测参数；
- 同一 `pinhole-equi`/`pinhole-radtan` 底层光学投影与解析 Jacobian；
- 每个角点用自己的 $t_i+v\tau$ 查询连续轨迹；
- K、D、行时间和轨迹在同一个非线性问题中联合调整；
- 均使用 Optimizer2 基础设施并在输出时重算未加权物理像素 RMS；
- 都可以固定行时间，并可从相同 K/D seed 开始联合优化。当前公开 task 只允许至少双目时
  使用 `freeze_intrinsics: true`；单目 system/native 对照不能通过公开 CLI 固定 K/D。

不能直接逐数值比较的原因包括：

- LM 与 DogLeg 的更新路径不同；
- 固定像素协方差与 adaptive covariance 的代价权重不同；
- system 使用观测率感知的固定 spline 网格，native 会 adaptive knot；
- 行时间有界 latent 与原生无界标量的参数化不同；
- 运动先验、端部支持和异常点处理不同。

因此两者相同输入下得到不同的 K/D/$\tau$/RMS，并不自动表示某个实现错误。严谨的公开
单目对照必须固定帧集合、角点参数、相机模型、K/D seed、行时间 seed，并让两边使用相同
的活动参数；当前两边的 K/D 都参与联合估计。随后同时检查：

1. 最终使用帧与角点数；
2. K、D 和 $\tau$ 差异；
3. 未加权逐角点 RMS，而不只比较内部 objective；
4. 行时间是否依赖 knot 密度、时间段或初值；
5. 留出数据和独立动态序列，而不只看训练集；
6. system 的可观性和 native 的范围/收敛状态。

临时入口直接包装
`src/kalibr/calibration/kalibr/python/kalibr_calibrate_rs_cameras`。本工程只移除了它对
`rosbag`/`cv_bridge` 的顶层依赖，补上 no-ROS 数据读取、`pinhole-equi` RS 映射、
初始化、线程预算和结构化结果。它的 adaptive covariance、运动先验、DogLeg、adaptive
knot 与无界行时间仍保持原生语义。`max_abs_line_delay_s` 对该入口是求解后的物理可接受性
检查，不应写成优化器内部边界。包装器把原生时间表达式的固定稀疏 buffer 设为 `0.5 s`，
样条两端各提供 `2 × 0.5 s` 支持，并在建残差前检查端部支持严格大于
`buffer + 初始首末行跨度`。该 buffer 只允许 DogLeg 试探时角点时间移动，不会把无界 shutter
改成有界参数或增加行时间先验。

两种求解器的 RMS 都由最终未归一化二维像素残差重算：

```text
RMS = sqrt(mean(dx² + dy²))
```

由于轨迹、协方差和行时间参数化不同，即使输入和初值相同，两者也不是逐位数等价算法。
单目对照应固定相同帧、角点检测参数、K/D seed 和行时间 seed，并同时比较 K/D、tau、RMS、
使用帧数及范围状态。

## 9. 输出指标如何解释

每目结果包含真实 `rms` 和 `shutter`；相邻外参包含 `alignment`。其中：

- `rms` 是联合 RS 模型的二维逐角点残差，是拟合质量主指标；
- `alignment` 使用最终 K/D/T 对原始左右角点做普通 OpenCV 光学校正，只衡量非视差轴
  差异，不使用行时间、轨迹或目标深度；
- `rs_compensated_pair_residual` 在相同校正域内，从实测左右差中减去联合模型预测的左右
  差，是依赖拟合轨迹和标定板的样本内诊断。

保留普通 `alignment` 是为了让静态普通、动态普通和动态 RS 结果使用同一 K/D/T 口径。
一般六自由度运动下，不同行需要不同姿态，没有一组只依赖 K/D/T 的固定校正映射能同时
补偿整幅 RS 图像。因此 `alignment` 不能称作“逐行补偿后的联合残差”，也不能与二维
`rms` 直接相减。新增的 `rs_compensated_pair_residual` 更接近“模型解释后还剩多少左右
非视差残差”，但它使用了同一批角点拟合出的轨迹，不是独立外部精度证明。

输出观测归档开启后，每个最终角点还保存：

- 相机原始时间戳、角点实际求解时刻和 `row_time_offset_s`；
- 实测像素、预测像素和二维残差；
- 该角点时刻的 `T_camera_target`。

`evaluate` 只读这些已有证据重算指标，不重新检测、估轨迹或优化。没有预测或共同角点时，
对应指标写明 unavailable 原因，不写 0。

Camera–IMU 可以从纯视觉 RS 结果读取 K/D/T 和相邻外参，但结果中的 `shutter` 只是来源
元数据，不会自动变成下游求解状态。普通 `imu-camera` 在内部 camchain 适配时去掉该块，
保持全局快门残差；需要逐行时间模型时必须选择 `imu-camera-rs`，并在它自己的 task 中
显式配置完整 `rolling_shutter`。这样不会因引用了带快门元数据的结果而静默切换算法。

## 10. 数据要求和结论边界

静止数据适合建立稳定 K/D/T 参考，却几乎不能单独辨识行时间。RS 数据应包含不同方向的
平移和旋转、充分的图像行覆盖、距离与倾角变化，并尽量避免严重运动模糊。低频保存或再次
抽帧会降低帧间运动约束；行时间可能与样条自由度、畸变、外参和运动先验耦合。

判断 RS 是否造成动态重投影误差，至少同时比较同求解器 `tau=0`、固定理论正值、反向值、
自由估计与多初值，并做时间段或 tag 留出验证、knot 敏感性和成块重采样。只有训练误差
降低不足以证明原因；报告应把结论分为“强支持、重要贡献、证据不足、不支持”，并保留
模糊、板面、光学模型、左右固定时差和运动近似等竞争解释。
