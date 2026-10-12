# Rolling Shutter 单目／多目相机标定

本页说明纯视觉 `camera_rolling_shutter_calibration`。它同时估计光学参数、连续运动轨迹
和逐行采样时间。普通 `calibrate cameras` 使用 Global Shutter（GS）观测模型；
Camera–IMU Rolling Shutter 联合标定见
[ROLLING_SHUTTER_ZH.md](ROLLING_SHUTTER_ZH.md)，不要把三种任务的参数或指标混用。

需要逐行核对当前实现中的 design variable、残差、解析 Jacobian、样条、运动先验、
adaptive covariance、可观性和三层默认值覆盖时，参见
[纯视觉 RS 源码实现说明](ROLLING_SHUTTER_IMPLEMENTATION_REFERENCE_ZH.md)。该说明以当前源码
调用链为准；本文保留面向使用者的配置、流程和结果解释。

项目保留两条 RS 相机入口：

| 入口 | 用途 | 相机数量 | 正式交付建议 |
|---|---|---:|---|
| `kalibr-noros calibrate cameras-rs` | 项目 RS system 求解器 | 1 台或多台 | 使用 |
| `kalibr-noros calibrate native-rs-cameras` | 对照 Kalibr 原生单目 RS 流程 | 恰好 1 台 | 只用于算法对照 |

两条入口共用 task、数据集、标定板和初始化文件格式，但不共用完整的 `calibration` 参数集，
也不是同一个优化问题切换后端。下文把它们分别称为 **system** 和 **native**。

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

同一个**精简的单目 task** 可以执行 native 对照：

```bash
kalibr-noros calibrate native-rs-cameras \
  --config camera_rolling_shutter_calibration_task.yaml \
  --output-dir result
```

若从双目完整示例开始，执行前需删去 cam1 及其 `rolling_shutter.cam1`，并删去
`synchronization_tolerance_s`、轨迹样条、运动先验和异常点过滤等 system 专用字段。
native 入口的 `calibration` 只接受 `window_half_size_px`、`max_displacement_px`、
`max_iterations`、`feature_sigma_px` 和 `freeze_intrinsics`。公共 CLI 目前不允许单目
`freeze_intrinsics: true`，因此这里必须省略该字段或填写 `false`。
`native-rs-cameras` 遇到两台及以上相机会在求解前拒绝输入，输出名称增加 `native_`
前缀，避免覆盖正式 `cameras-rs` 结果。该临时入口不实现轨迹导出，必须保持
`output.export_poses: false`；需要 `poses.csv` 时使用正式 `cameras-rs`。两条入口共用
task schema，并不表示任一具体 YAML 中的后端专用参数都可直接互换。

| `calibration` 字段组 | system | native |
|---|---:|---:|
| `window_half_size_px`、`max_displacement_px` | 支持 | 支持 |
| `max_iterations`、`feature_sigma_px` | 支持 | 支持，但内部代价权重不同 |
| `freeze_intrinsics` | 支持；`true` 仅双目／多目完整 seed | 字段可读；公共单目入口只能为 `false` |
| `focal_initialization_min_visible_corner_ratio`、`synchronization_tolerance_s` | 支持 | 不支持 |
| `spline_order`、`time_padding_s`、`knots_per_second`、运动先验权重 | 支持 | 不支持公开配置 |
| `min_views_for_outlier_statistics`、`remove_outliers`、`final_filtering`、`blake_zisserman` | 支持 | 不支持 |

“不支持”表示 task 在启动 native 求解器前明确报错，不是静默忽略。

两条公开入口都只支持下列两种模型：

| task 模型名 | 两条入口使用的底层 RS geometry |
|---|---|
| `pinhole-equi` | `EquidistantPinholeRs` |
| `pinhole-radtan` | `DistortedPinholeRs` |

因此 RS 相机任务不接受 `pinhole-radtan5`、`pinhole-radtan8`、
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

它属于 `src/kalibr` 中保留的标定实现，但不是项目正式的多目 RS
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

## 4. `pinhole-equi` 与 `pinhole-radtan` 光学投影

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

项目输出契约把数据集中的相机时间戳解释为**原图第 0 行曝光结束时刻**。程序不会从
图像内容判断驱动 PTS 究竟对应曝光开始、中点还是结束；采集端必须保证该语义，或在进入
标定前统一转换时间戳。结果中的 `timestamp_reference: row0_exposure_end` 是契约声明，
不是程序对硬件打点位置的测量结论。

原图检测角点坐标为 $(x,y)$，每行时间为 $\tau$，求解器查询轨迹的时刻为：

```text
t_corner = t_camera_timestamp + y × tau
```

正 $\tau$ 表示图像 y 增大时采样更晚，负值表示扫描方向相反。高度为 $H$ 时：

```text
首末行跨度 = (H − 1) × |tau|
```

这里的 $y$ 是**检测所得的原始图像亚像素纵坐标**，不是去畸变、裁剪或极线校正后的坐标。
`line_delay_s` 是相邻两条像素行的采样时刻差，不是某一行的曝光时长。首末行跨度不包含
垂直消隐，也不等同于整帧周期。以 2160 行、8 μs/行为例，首末行跨度为
`2159 × 8 μs = 17.272 ms`。若曝光时长为 1 ms，第 0 行曝光中点比“第 0 行曝光结束”
早 0.5 ms。这个 `-0.5 ms` 对所有图像行相同，属于公共时间平移，不应重复当成每行时间。
纯视觉为什么不能单独辨识这个常量、与 IMU 联合时为什么又必须处理，见第 5.1 节。

有些分析采用中间行 $y_c=(H-1)/2$：

```text
t_corner = t_center + (y − y_c) × tau
t_center = t_row0 + y_c × tau
```

在纯视觉且轨迹时间原点可以同步平移时，两式给出相同角点相对时差；结果文件仍统一写
`reference_row_px: 0.0` 和 `timestamp_reference: row0_exposure_end`。这保证输出口径固定，
并不自动保证驱动 PTS 已符合该口径。Camera–IMU RS 任务还包含跨传感器整体时移，不能把
纯视觉的轨迹时间原点或普通 Camera–IMU 的 `timeshift_cam_imu` 当作每行时间。

三个量应始终分开记录：

| 量 | 含义 | 典型单位 |
|---|---|---|
| 曝光时长 | 每一行光电积分持续多久 | ms |
| `line_delay_s` | 相邻图像行采样参考时刻之差 | s/row |
| `first_to_last_row_span_s` | $(H-1)\lvert\tau\rvert$，第 0 行到末行的采样跨度 | s |

### 5.1 什么是时间参考“重参数化”

“重参数化”的“重”读 **chóng**，意思是“重新”。它不是重复标定，也不是给残差重新加权；
它表示：**同一条物理运动轨迹换一种时间坐标或参数写法后，仍产生完全相同的图像预测**。

设图像时间戳 $t_i$ 是第 0 行曝光结束时刻，固定曝光时长为 $E$。若投影使用曝光中点，
第 $y$ 行的物理采样参考时刻为：

$$
t_{iy}^{physical}=t_i-\frac{E}{2}+y\tau.
$$

令公共常量 $\Delta=-E/2$。真实轨迹为 $\mathbf T(t)$ 时，像素预测使用：

$$
\pi\!\left(\mathbf T(t_i+\Delta+y\tau)\mathbf P\right).
$$

纯视觉任务没有 IMU 或其他绝对时间参考。定义一条重新计时、但空间运动相同的轨迹：

$$
\mathbf T'(t)=\mathbf T(t+\Delta),
$$

求解器即使按曝光结束时刻查询：

$$
\pi\!\left(\mathbf T'(t_i+y\tau)\mathbf P\right),
$$

也会得到完全相同的像素。换句话说，数据只能确定“某个角点对应什么姿态”，不能仅凭图像
确定整条轨迹的绝对时间原点。因此固定的曝光结束／曝光中点差可以由轨迹整体平移吸收，
这就是时间参考重参数化。

每行时间 $\tau$ 不能这样消除，因为它产生随 $y$ 变化的偏移。一个常量轨迹平移无法同时
消除第 0 行的 $0\tau$、第 1000 行的 $1000\tau$ 和末行的 $(H-1)\tau$。因此运动和行覆盖
充分时，纯视觉可以约束逐行相对时间，却不能单独确定公共绝对时间原点。

这里的“公共”有严格含义：

- 单目中，所有帧都具有相同的固定偏移时，它可以被一条自由轨迹吸收；
- 双目／多目中，只有各相机共有的同一个偏移可以被共享轨迹吸收。各相机曝光时长、PTS
  参考点或管线延迟不同形成的**相对偏移**不能由一条共享轨迹同时吸收；
- 自动曝光导致 $E$ 随帧变化时，$-E/2$ 不再是公共常量，也不能由一次整体平移消除。

Camera–IMU 联合任务中，IMU 测量把轨迹绑定到 IMU 时间轴。角点时间关系可概念性写为：

$$
t_{corner}^{imu}
=t_{camera}+\delta_{cam\text{-}imu}+\Delta_{exposure}+y\tau.
$$

图像和 IMU 实际约束的是 $\delta_{cam\text{-}imu}+\Delta_{exposure}$ 的组合。如果 task 把
相机时间戳按曝光结束解释，却在物理上想使用曝光中点，那么
$\Delta_{exposure}=-E/2$；没有显式建模它时，估计出的 Camera–IMU 整体时移会把该项吸收。
因此该时移数字可能同时包含传感器时差、驱动／传输延迟和曝光参考点差异，不能自动解释成
单一硬件延迟。具体正负号必须按结果中定义的 Camera–IMU 时移方向读取。

以 $E=1\,\mathrm{ms}$、$\tau=8\,\mu\mathrm{s/row}$ 为例：曝光中点相对结束时刻的
`-0.5 ms` 是每一行共有的常量；2160 行的 `17.272 ms` 是第 0 行到末行的相对采样跨度。
前者在纯视觉自由轨迹中可被重新计时吸收，后者不能，两者不能相加后再除以图像高度来
定义新的 `line_delay_s`。

### 5.2 图像预处理会改变行时间参数的含义

求解器只看到数据集里保存的图像坐标，因此 `line_delay_s` 的单位是“每个当前图像的像素
行”，不一定等于传感器原始行时间。若纵坐标经过仿射变换 $y'=s_y y+b$，保持同一物理
角点时刻需要：

$$
\tau'=\frac{\tau}{s_y},\qquad
t'_{row0}=t_{row0}-\frac{b\tau}{s_y}.
$$

常见情况如下：

- 只做水平缩放或水平翻转不会改变 $y$，因此不改变行时间；
- 纵向缩放到原来的 $s_y$ 倍时，每个新像素行对应 $1/s_y$ 个原始行，配置初值应使用
  $\tau' = \tau/s_y$；
- 从原图第 $y_0$ 行开始裁剪时，行时间不变，但新第 0 行时间戳应为
  $t'_{row0}=t_{row0}+y_0\tau$；
- 高度为 $H$ 的图像上下翻转后 $y'=(H-1)-y$，等价于
  $\tau'=-\tau$、$t'_{row0}=t_{row0}+(H-1)\tau$。

如果只变换像素而不相应调整行时间 seed 和第 0 行时间戳，优化器可能用轨迹或行时间吸收
这部分时间定义错误。普通纯视觉任务对公共常量时移不敏感，但扫描方向和行间相对时差仍会
改变；Camera–IMU 联合任务对公共时移也敏感。上述换算把预处理视为理想坐标变换；纵向
插值会混合相邻传感器行的像素，严格来说不再满足“一个输出像素行对应一个采样时刻”。

## 6. 正式 `cameras-rs` 的实现原理

system 流程按以下顺序执行：

1. 每台相机独立检测 AprilGrid，并用观测或初始化 seed 建立 K/D 初值；
2. 按 `synchronization_tolerance_s` 和共同全局角点 ID 建立多目共视关系；
3. 单目任务跳过 baseline；双目／多目任务用共视图初始化相邻 baseline，初始化文件中的
   外参只覆盖对应的初值；
4. 用有效 PnP 位姿初始化一条共享的连续标定板轨迹；
5. 构建一个联合问题，同时优化 K/D、相邻外参、逐目行时间和轨迹；
6. 满足配置时按最终像素残差执行一次角点过滤，重建同一个问题并再次求解；
7. 在最终解处做可观性分析、行时间边界检查、指标计算和结果发布。

这条流程没有普通相机任务的 `calibration.solver_mode: incremental|full_batch`。不要在 RS
task 中加入 `solver_mode`；`cameras-rs` 的最终核心求解本身就是连续轨迹上的联合批量问题。

联合阶段的状态包括：

- 每目 pinhole 投影参数和畸变；
- 相邻相机外参 `T_cn_cnm1`；
- 每目独立的行时间；
- 连续目标位姿样条控制点。

### 6.1 完整相机链与各目实际时间

每台相机使用自己的源时间戳，不把同一 view 内的各目时间戳强行改成 cam0 时间。对第 i
台相机，先按列表顺序累计相邻外参：

```text
T_ci_c0 = T_ci_c(i-1) × ... × T_c1_c0
T_ci_target(t) = T_ci_c0 × T_c0_target(t)
```

以三目为例，结果保存的是两段相邻外参：

```text
cam0 --T_c1_c0--> cam1 --T_c2_c1--> cam2
```

把 cam0 坐标中的点转换到 cam2 时必须依次经过两段变换：

```text
T_c2_c0 = T_c2_c1 × T_c1_c0
T_c2_target(t) = T_c2_c1 × T_c1_c0 × T_c0_target(t)
```

只使用最后一段 `T_c2_c1`，等于把 cam0 坐标中的点错误地当成 cam1 坐标中的点。双目只有
一段 baseline，看不出这个区别；第三台及后续相机必须累计从 cam0 到当前相机的完整链。

`synchronization_tolerance_s` 只用于判断不同相机的图像能否组成同一个共视 view。例如：

```text
cam0 图像时间戳：1.000000 s
cam1 图像时间戳：1.000050 s
同步容差：       0.000100 s
```

两幅图相差 50 us，可以组成同一个 view，但求解器仍分别保留 `1.000000 s` 和
`1.000050 s`。同一个物理角点在两幅图中的纵坐标通常也不同，因此实际角点时刻分别是：

```text
t_cam0_corner = 1.000000 + y_cam0 × tau_cam0
t_cam1_corner = 1.000050 + y_cam1 × tau_cam1
```

它们不必相等。共同全局角点 ID 表示两目检测的是 AprilGrid 上的同一个物理角点；只有
时间接近但没有共同角点的图像不能提供双目几何约束。

### 6.2 行时间为什么使用有界内部变量

估计模式使用光滑有界状态：

```text
tau = max_abs_line_delay_s × tanh(q)
```

这里的 `tau` 是最终物理行时间，单位为 s/row；`q` 是优化器内部使用的无量纲变量；
`max_abs_line_delay_s` 是允许的最大绝对行时间，下面简称 `bound`。因为
`-1 < tanh(q) < 1`，所以无论优化器怎样更新 `q`，始终满足：

```text
-bound < tau < bound
```

例如 `bound = 20 us/row`：

| `q` | `tau` |
|---:|---:|
| `0` | `0 us/row` |
| `0.4236` | `8 us/row` |
| `1` | `15.23 us/row` |
| `2` | `19.28 us/row` |
| 趋近正无穷 | 趋近 `20 us/row` |

如果初值是 `8 us/row`、边界是 `20 us/row`，内部初值为：

```text
q_seed = atanh(8 / 20) = atanh(0.4) ≈ 0.4236
```

最终结果仍输出物理量 `tau`，不会向用户输出 `q`。该参数化的导数为：

```text
d tau / d q = bound × (1 - tanh(q)^2)
```

因此实际参数更新和 Jacobian 使用同一个连续公式。若直接优化 `tau`，再把越界值硬裁剪到
正负边界，裁剪点会不光滑，而且优化器计算的更新与真正应用的更新不一致；system 后端不
采用这种做法。

有界参数化只限制搜索范围，不是行时间先验。目标函数中没有
`(tau - tau_seed)^2 / sigma^2` 之类把结果拉回初值的残差；行时间由重投影数据、K/D、
外参和共享轨迹共同决定。接近边界时 `d tau / d q` 会变小，而且结果可能只是被搜索范围
挡住，并不能证明真实行时间位于边界。因此估计值进入配置范围的边缘区域时任务失败，要求
检查范围、扫描方向、时间戳语义、运动激励、轨迹设置和可观性。

### 6.3 固定行时间对照

`estimate: false` 时不优化 `q`，`line_delay_s` 固定为配置值。可以使用同一个求解器做：

| 对照 | 配置 | 用途 |
|---|---|---|
| 零行时间 | `line_delay_s: 0` | 近似全局快门模型 |
| 正向固定值 | 例如 `+8e-6` | 验证已知正向扫描理论值 |
| 反向固定值 | 例如 `-8e-6` | 检查扫描方向或图像上下翻转语义 |
| 自由估计 | `estimate: true` | 由数据估计行时间 |

公平对比时应保持 K/D/T 初值、帧和角点集合、`spline_order`、`knots_per_second`、
`time_padding_s`、运动先验权重及 `max_abs_line_delay_s` 相同，只改变行时间值和
`estimate`。这样各组使用相同的轨迹带宽、边界定义和时间支持检查，自由估计组的搜索范围
也保持不变，误差变化才主要对应行时间模型。

### 6.4 轨迹带宽与运动先验

本节涉及的 pose sample、control point、knot、segment、order、轨迹二阶导数和运动先验
可先阅读[连续轨迹、样条与运动先验入门](ROLLING_SHUTTER_SPLINE_TUTORIAL_ZH.md)。该文按
当前 `SystemCalibrator.py` 实现给出数值例子和代码调用链。

目标轨迹默认使用四阶样条。由于联合问题始终加入二阶运动先验，`spline_order` 至少为 3。
system 求解器没有 native 后端的 adaptive knot 插入。省略 `knots_per_second` 时，自动
knot 率取进入联合问题的共视序列参考时间戳的中位频率，即
`1 / median(diff(selected_timestamps))`。自动段数先按含两端 padding 的时间跨度乘以上述
频率计算，最少为 `2 * spline_order`。段数覆盖包含两端 padding 的完整样条时间域，因此
可以略多于成功初始化的目标位姿数；不能再按位姿数硬截断。硬截断会改变实际 knot 间距及
knot 网格相对观测的相位，可能使行时间与轨迹出现数值共振。这样每个实际入选观测间隔及
端部支持域都有一致轨迹带宽，同时不会因原始相机频率高就对抽帧数据建立过密轨迹。旧的
“入选频率除以 3”会在低频抽样数据上进一步压低轨迹带宽，使未建模运动进入重投影误差，
甚至由 K/D 或行时间吸收，因此不再作为默认值。

样条加入分段二阶运动先验，默认平移权重 `1e-5`、旋转权重 `1e-2`。这些权重约束无
图像区间和端部运动，也是目标函数的一部分。行时间、0 延迟及固定延迟对照必须使用相同
样条和先验。需要做敏感性
分析时显式设置 `knots_per_second`；显式值同样保留 `2 * spline_order` 下限。报告中应
同时保存请求频率、请求段数、实际段数和下限，不能只保留最有利的设置。
`time_padding_s` 必须严格大于任一相机在配置范围内的首末行跨度，防止优化更新后角点
时刻离开样条支持域。

### 6.5 求解停止与失败条件

联合求解沿用 Optimizer2 的原生停止条件：参数增量 `dX` 或目标函数变化绝对值 `|dJ|`
达到阈值即可停止。若实际迭代数达到 `max_iterations`，则表示预算耗尽而不是满足上述
停止条件；任务会失败，不发布正式结果，错误中同时记录 `iterations`、`dXFinal` 和
`dJFinal`。临时原生单目入口对初始求解和每次 adaptive knot 重求解执行相同检查；线性
求解失败或耗尽迭代预算时也不会序列化标定结果。单纯调大迭代上限不能证明物理参数
可信，仍需检查行时间范围、样条／先验敏感性和可观性。

完整字段、默认值、单位和范围见
[完整注释示例](../config/examples/v1.0.0/all_params/camera_rolling_shutter_calibration_task_full.yaml)。

## 7. 初始化

### 7.1 四类初值来自哪里

RS 相机任务的“初始化”包含四类状态，它们的来源不同：

| 状态 | 初值来源 | 能否由初始化文件提供 | 联合阶段默认状态 |
|---|---|---:|---|
| 每目 K/D | 图像观测，或 `camera_calibration_initialization` | 可以 | active |
| 相邻 `T_cn_cnm1` | 多目共视图，或 `camera_calibration_initialization` | 可以 | active |
| 每目 `line_delay_s` | task 的 `rolling_shutter.<camera_id>` | 不可以 | 由 `estimate` 决定 |
| 目标位姿 spline | 使用当前 K/D/T 对有效观测做 PnP 后自动拟合 | 不可以 | active |

因此 `initialization.path` 只负责相机光学参数和相邻外参，不包含行时间或轨迹。RS 相机任务
复用 `camera_calibration_initialization`：

```yaml
initialization:
  path: initialization_stereo.yaml
```

一个双目初始化文件可以写成：

```yaml
schema_version: "1.0.0"
kind: camera_calibration_initialization
cameras:
  cam0:
    intrinsics: [2400.0, 2400.0, 1919.5, 1079.5]
    distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  cam1:
    intrinsics: [2400.0, 2400.0, 1919.5, 1079.5]
    distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
    T_cn_cnm1:
      - [1.0, 0.0, 0.0, -0.38]
      - [0.0, 1.0, 0.0, 0.0]
      - [0.0, 0.0, 1.0, 0.0]
      - [0.0, 0.0, 0.0, 1.0]
```

参数维数必须与 task 中的模型一致。`pinhole-equi` 和项目公开的 `pinhole-radtan` 都使用
四个投影参数 `[fu,fv,cu,cv]`；本任务支持的畸变参数维数由所选模型严格检查，不能通过
补零或截断把另一模型的 seed 直接套用。

初始化文件中的键使用 task 的相机 ID，加载后再按 `task.cameras` 的列表顺序映射为内部
cam0、cam1……。例如 task 的 ID 是 `left`、`right`，初始化文件也必须写 `left`、
`right`；内部求解时它们分别成为 cam0、cam1。初始化文件中出现 task 未声明的相机 ID、
重复 ID 或模型维数不符都会在检测和优化之前报错。

### 7.2 K/D 初值怎样使用

未设置 `calibration.freeze_intrinsics` 时，每台相机按以下规则初始化：

1. K/D 都未提供：从该相机的 AprilGrid 观测执行原生单目初始化和单目 LM；
2. 只提供 K：跳过解析焦距估计，先从观测取得图像分辨率，再以给定 K 和模型默认 D
   启动单目 LM；
3. 只提供 D：先从观测初始化 K，再覆盖给定 D，随后执行单目 LM；
4. K/D 都提供：以完整 seed 启动单目 LM。

这里的单目 LM 仍会更新 K/D。完成多目 baseline 初始化后，传统全 view refinement 还会
同时更新共享的相机几何对象；随后 RS 联合问题继续把同一组 K/D 作为 active 状态。因此
初始化文件中的数值可能在三个位置发生变化：单目 LM、baseline 的全 view refinement 和
最终 RS 联合求解。seed 的作用是给非线性求解器一个起点，并不要求最终结果保持靠近 seed。

`calibration.freeze_intrinsics: true` 是另一种活动状态策略。它要求：

- 至少有两台相机；
- 初始化文件为每台相机提供完整 K 和 D；
- 参数维数与各目模型完全一致。

此时跳过单目内参 LM，K/D 在 baseline 初始化、传统全 view refinement 和 RS 联合阶段都
固定；相邻外参、目标轨迹和配置为 `estimate: true` 的行时间仍然 active。它适合验证一套
可信内参下的外参和行时间，不代表“完全固定所有相机参数”。当前公共 CLI 不支持 RS 单目
固定 K/D。

### 7.3 相邻外参初值怎样使用

推荐在第 2 台及后续相机块使用 `T_cn_cnm1`，它表示前一相机到当前相机：

$$
\mathbf p_{c_n}={}^{c_n}_{c_{n-1}}\mathbf T\,\mathbf p_{c_{n-1}}.
$$

兼容键 `T_cam_from_previous` 含义相同，但同一个相机块不能同时写两个键。外参只能属于
第 2 台及后续相机。cam0 没有前一台相机，因此不能包含该字段。三目时应分别提供
`cam1.T_cn_cnm1 = T_c1_c0` 和 `cam2.T_cn_cnm1 = T_c2_c1`，不能把 `T_c2_c0`
写进 cam2。

baseline 初始化的实际顺序是：

1. 始终先检查所有相机能否通过“时间同步且具有共同全局角点 ID”的观测形成连通图；外参
   seed 不会绕过该数据检查；
2. 没有 seed 时，选择共视信息较强的相机对执行成对 stereo 初始化，再换算为相邻相机链；
3. 只提供部分相邻外参时，缺失项由上述共视图生成，提供项覆盖对应链段；
4. 提供完整相邻链时，跳过成对 stereo 初始化，直接采用完整链 seed；
5. 无论外参 seed 是部分还是完整，随后都执行包含全部共视 view 的传统全批量 refinement；
6. refinement 后的 baseline 才用于 PnP 轨迹初始化，并作为 RS 联合问题的 active 起点。

所以“提供完整 baseline”只会跳过成对初始化，不会固定 baseline，也不会跳过全 view
refinement。当前 `cameras-rs` 没有冻结外参的配置。若 seed 方向写反、量级错误或偏差过大，
全 view refinement 及后续 RS 求解仍可能进入错误局部解，不能把“成功收敛”视为 seed 正确。

### 7.4 目标轨迹初值怎样生成

初始化文件不能直接写目标轨迹。完成 K/D 和 baseline 初始化后，程序遍历共视 view：

1. 优先选择检测角点较多的相机观测；
2. 用该相机当前 K/D 对标定板执行 PnP；
3. 使用完整相机链把该目标位姿换算到 cam0 参考系；
4. 将成功的 `(相机原始时间戳, 目标位姿)` 样本拟合为连续位姿 spline。

PnP 失败的 view 不提供轨迹样本；有效样本必须至少达到 `max(4, spline_order)`。K/D seed
会影响 PnP，baseline seed 会影响非 cam0 位姿换算，因此不合理的光学或外参初值会继续
污染 spline 初值。进入联合问题后 spline 控制点是 active 的，初始 PnP 位姿不是固定测量，
也不是额外先验；真正保留在目标函数中的轨迹约束是第 6 节所述二阶运动先验。

### 7.5 行时间初值与初始化文件无关

每目的行时间从 task 读取，例如：

```yaml
rolling_shutter:
  cam0:
    line_delay_s: 8.0e-6
    estimate: true
    max_abs_line_delay_s: 2.0e-5
```

当 `estimate: true` 时，`line_delay_s` 是有界 latent 状态的初值，并按
$q_0=\operatorname{atanh}(\tau_0/\tau_{max})$ 转换；它必须严格位于正负边界内部。设置
`estimate: false` 时，`line_delay_s` 在联合问题中固定，`max_abs_line_delay_s` 若存在则
只用于支持域和合法性检查。为了公平比较 $\tau=0$、固定理论值和自由估计，应使用同一份
K/D/T 初始化文件、同一帧集合、同一 spline 和运动先验，只修改行时间配置。

### 7.6 怎样选择初始化方式

| 已知条件 | 推荐配置 | 含义 |
|---|---|---|
| 没有可信历史参数 | 省略 `initialization` | K/D/T 全部由当前观测初始化 |
| 有粗略 K/D 或某段 baseline | 只填写可信字段 | 其余字段由观测补齐，所有字段继续优化 |
| 有完整历史 K/D/T，希望重新联合标定 | 提供完整文件，`freeze_intrinsics: false` | 全部作为起点，K/D/T 都可变化 |
| K/D 已单独验证可信，只估计外参、轨迹和行时间 | 提供完整 K/D/T，`freeze_intrinsics: true` | K/D 固定，T、轨迹和可估计行时间 active |
| 比较 0 延迟、理论延迟和自由 RS | 复用同一初始化文件 | 行时间只在 task 中改变，避免同时改变光学起点 |

所有 seed 都不增加隐式先验残差。`initialization.strategy` 已删除，不要填写 `refine` 或
`direct`。若保存了运行诊断，`initialization_report.yaml` 会记录实际提供字段、K/D 是否
固定、baseline 使用共视图还是 seed 覆盖，以及每目行时间是 active 还是 fixed；它用于
追溯初始化路径，不代表这些 seed 最终被求解器接受为真值。

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

这里的可观性分析调用 `analyze_write_require()`，当前只输出 reduced Fisher information
的谱、秩和弱模态，不请求或序列化 K/D/T/行时间的后验 covariance。`feature_sigma_px² I`
只是观测噪声权重；它与参数 covariance 不是同一概念。

### 8.2 原生 `kalibr_calibrate_rs_cameras` 的状态与目标函数

原生单目后端的活动状态为：

```text
K、D、原生 shutter lineDelay、相机与标定板相对位姿 spline 控制点
```

它直接使用 RS camera geometry 的 `shutterDesignVariable`。公开包装器总是把 task 中的
`line_delay_s` 显式传给原生脚本，因此通过 `kalibr-noros` 运行时不会采用上游的隐式初值。
只有绕过包装器直接调用内部脚本且不传 `--line-delay-seed`，才会使用
`1 / (入选帧率 × 图像高度)`。

原生行时间是无界标量。`max_abs_line_delay_s` 只在求解完成后检查：

$$
|\hat\tau|\leq\tau_{max}.
$$

它不等价于 system 后端的 $\tau_{max}\tanh(q)$，优化中的 DogLeg 试探值可能越过最终
允许范围。native 的固定时间表达式 buffer 为 `0.5 s`，样条首尾各扩展
`2 × 0.5 s`；建残差前还检查端部支持严格大于 `buffer + 初始首末行跨度`。这些值用于
维持时间表达式稀疏结构和样条端部支持，不会限制 shutter 变量，也不会增加行时间先验。

原生每个 RS 角点使用 `reprojectionErrorAdaptiveCovariance`。该误差项根据连续轨迹下的
局部图像运动调整有效协方差；目标函数仍包含 spline motion prior，但权重、误差标准化和
正式 system 后端不同。求解器使用 Optimizer2、Schur complement、Block Cholesky 和
DogLeg。初次联合求解后，`ReprojectionErrorKnotSequenceUpdateStrategy` 根据残差决定是否
重排 spline knot；每次变化后重建完整问题并再次求解，直至无需更新或达到次数限制。

这个 adaptive covariance 是残差的运动相关权重，不是参数后验 covariance。当前 C++ 实现
在每次误差评价时按当前状态刷新有效逆协方差，但残差 Jacobian 不继续对该权重矩阵本身
求导；本次线性化把刷新后的权重视为固定量。矩阵公式和源码位置见
[纯视觉 RS 源码实现说明](ROLLING_SHUTTER_IMPLEMENTATION_REFERENCE_ZH.md)。

native 后端没有 system 后端等价的多目外参状态、最终角点四倍标准差过滤、
Blake–Zisserman 开关和 reduced-Fisher 可观性输出。项目包装器在保留上述核心求解流程的
同时增加了 no-ROS 数据读取、公开模型映射、初始化 seed、线程预算、失败检查、观测归档
和结构化结果；不要把它描述成未经修改的上游可执行文件。输出 RMS 由最终未加权二维像素
残差重算，以便使用相同物理单位比较：

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

## 9. 输出指标如何解释

每目结果包含 `rms` 和 `shutter`；双目／多目结果的第 2 台及后续相机还包含对应相邻相机
对的 `T_cn_cnm1`、`alignment` 和 `rs_compensated_pair_residual`。单目结果没有相机对指标。

`shutter` 中最关键的字段是：

```yaml
shutter:
  type: rolling_shutter
  line_delay_s: 8.0e-6
  reference_row_px: 0.0
  first_to_last_row_span_s: 0.017272
  estimated: true
  max_abs_line_delay_s: 2.0e-5
  timestamp_reference: row0_exposure_end
  corner_time_equation: t_corner_s = t_camera_timestamp_s + y_px * line_delay_s
```

数值只是格式示例。实际输出还可能包含距边界余量及后端中的边界用途。不要只摘录
`line_delay_s`；至少同时保存图像高度、首末行跨度、`estimated`、上限和求解后端。

三个像素指标的定义不同：

- `rms` 是联合 RS 模型的二维逐角点残差：
  $\sqrt{\sum_j(\Delta u_j^2+\Delta v_j^2)/N}$。$N$ 是角点数，不是二维坐标分量数；
  它是样本内拟合质量的主指标。
- `alignment` 使用最终 K/D/T 对原始左右角点做普通 OpenCV 光学校正，只衡量非视差轴
  差异的分布和 RMS，不使用行时间、轨迹或目标深度。实现直接校正已检测的共同角点，
  不会先生成校正图像再重新检测角点；
- `rs_compensated_pair_residual` 在相同校正域内，从实测左右差中减去联合模型预测的左右
  差，再统计非视差轴绝对残差。公式可写为
  $|(m_L-m_R)_{\perp}-(\hat m_L-\hat m_R)_{\perp}|$；它是依赖拟合轨迹和标定板的样本内
  诊断。

保留普通 `alignment` 是为了让静态普通、动态普通和动态 RS 结果使用同一 K/D/T 口径。
一般六自由度运动下，不同行需要不同姿态，没有一组只依赖 K/D/T 的固定校正映射能同时
补偿整幅 RS 图像。因此 `alignment` 不能称作“逐行补偿后的联合残差”，也不能与二维
`rms` 直接相减。`rs_compensated_pair_residual` 更接近“模型解释后还剩多少左右
非视差残差”，但它使用了同一批角点拟合出的轨迹，不是独立外部精度证明。

输出观测归档开启后，每个最终角点还保存：

- 相机原始时间戳、角点实际求解时刻和 `row_time_offset_s`；
- 实测像素、预测像素和二维残差；
- 该角点时刻的 `T_camera_target`。

`evaluate` 只读这些已有证据重算指标，不重新检测、估轨迹或优化。没有预测、配对 view、
共同角点或有效校正域时，对应指标写明 `unavailable` 原因，不写 0。`alignment` 和
`rs_compensated_pair_residual` 的详细统计位于 `metrics.json`；结果 YAML 只保留对应 RMS
标量或 `null`。

Camera–IMU 可以从纯视觉 RS 结果读取 K/D/T 和相邻外参，但结果中的 `shutter` 只是来源
元数据，不会自动变成下游求解状态。普通 `imu-camera` 在内部 camchain 适配时去掉该块，
保持全局快门残差；需要逐行时间模型时必须选择 `imu-camera-rs`，并在它自己的 task 中
显式配置完整 `rolling_shutter`。这样不会因引用了带快门元数据的结果而静默切换算法。

### 9.1 运行时如何用于双目三角化

`line_delay_s` 不能单独把 RS 图像变成全局快门图像。它只把原始像素行转换为采样时刻；运行
时还需要该时刻的相机运动。以 Camera–IMU 输出口径为例，对相机 $i$ 的原始像素
$(u_i,v_i)$：

$$
t_i=t_{i,\mathrm{frame}}
+\Delta t_{\mathrm{cam\rightarrow imu},i}
+(v_i-y_{\mathrm{ref},i})\tau_i.
$$

这里 $\tau_i$ 是 `line_delay_s`，$y_{\mathrm{ref},i}$ 是 `reference_row_px`，
$\Delta t_{\mathrm{cam\rightarrow imu},i}$ 是结果中的 `timeshift_cam_imu`；实现内部和诊断指标
有时用带单位后缀的名称 `timeshift_cam_imu_s` 表示同一数值。当前 Camera–IMU 输出明确采用
`t_row_imu = t_frame + timeshift_cam_imu + (y-reference_row_px)*line_delay_s`。必须使用畸变、
缩放和极线校正前的原始传感器行号；若只能取得缩放后的行号，必须同时换算行坐标和每行时间，
不能直接套用原分辨率的 $\tau$。

设 ${}^{W}_{I}\mathbf T(t)$ 是 IMU 到世界的运行时连续位姿，标定输出
`T_cam_imu` 为 ${}^{C_i}_{I}\mathbf T$，则：

$$
{}^{W}_{C_i}\mathbf T(t_i)
={}^{W}_{I}\mathbf T(t_i)
\left({}^{C_i}_{I}\mathbf T\right)^{-1}.
$$

对左右观测分别执行以下步骤：

1. 用各自 K/D 反投影原始像素，得到相机坐标系单位射线 $\mathbf r_i$；
2. 用各自角点时刻 $t_i$ 插值 IMU/视觉惯性轨迹；不能只取整帧时间戳处的位姿；
3. 将射线变换到共同世界坐标：

$$
\mathbf o_i=\operatorname{trans}({}^{W}_{C_i}\mathbf T(t_i)),\qquad
\mathbf d_i=\operatorname{R}({}^{W}_{C_i}\mathbf T(t_i))\mathbf r_i;
$$

4. 对两个不同时刻、不同光心的世界射线做最近点三角化：

$$
\min_{\lambda_L,\lambda_R}
\left\|
(\mathbf o_L+\lambda_L\mathbf d_L)
-(\mathbf o_R+\lambda_R\mathbf d_R)
\right\|^2;
$$

5. 取两条射线最近点的中点作为静态场景点 $\mathbf p_W$。若业务需要参考时刻 $t_*$ 下的
   cam0 坐标，再计算
   $\mathbf p_{C_0(t_*)}=({}^{W}_{C_0}\mathbf T(t_*))^{-1}\mathbf p_W$。

这种“异步射线三角化”直接使用左右像素各自的真实采样时刻，通常比先把像素强行搬到同一
时刻更清楚。若选择先做图像或角点运动补偿，平移造成的像素位移依赖未知深度，通常需要：
静态双目给出初始深度、按轨迹搬移到共同时间、重新匹配/三角化，再迭代。仅使用陀螺仪可以
较好补偿旋转，不能完整补偿平移视差。

标准 `stereoRectify + disparity + Q` 默认左右观测来自同一时刻且一组固定 R/T 对整幅图像
成立。相机运动明显、首末行跨度较大时，直接使用这条全局快门管线不会自动使用
`line_delay_s`。稠密深度需要逐行位姿、运动补偿或专用 RS stereo 算法；稀疏特征可直接使用
上面的异步射线方法。

还需满足以下边界：

- 设备静止时，不同行虽然时间不同，但位姿不变，普通三角化已经足够；
- 对静态场景，两个不同时刻射线仍指向同一世界点；对运动物体还需要物体运动模型，IMU 只能
  修正相机自身运动；
- 相机帧时间戳事件、扫描方向和 `timeshift_cam_imu_s` 符号必须与标定输出一致；
- 若运行时没有 IMU、视觉惯性轨迹或其他运动估计，只有 K/D/T/$\tau$ 仍无法完整修正动态 RS
  三角化误差。

## 10. 数据要求和结论边界

需要从一份普通相机或普通 Camera–IMU task 自动完成分块筛查、列方向负对照、bootstrap、
多初值及 knot 敏感性对照时，使用
[滚动快门行时间诊断工具](ROLLING_SHUTTER_DIAGNOSIS_TOOL_ZH.md)。该工具只在证据通过时
生成建议 RS task，不会把离线拟合值直接发布成正式标定结果。

静止数据适合建立稳定 K/D/T 参考，却几乎不能单独辨识行时间。RS 数据应满足：

- 标定板覆盖图像上、中、下区域，否则行时间缺少行坐标杠杆；
- 同时包含绕水平轴、垂直轴和光轴的双向旋转，以及不同方向的平移；
- 距离和倾角有变化，但每帧仍有足够角点并避免严重运动模糊；
- 双目／多目各相机硬件同步关系稳定，数据时间戳没有跳变或重复；
- 尽量保留连续动态帧。再次抽帧会削弱轨迹约束，使行时间更容易与轨迹形状、畸变和外参
  互相吸收误差。

### 10.1 不要只看一次自由估计

建议至少执行以下同口径实验；所有组应固定帧集合、角点参数、K/D/T seed、样条设置、
运动先验和异常点策略：

| 组别 | `line_delay_s` | `estimate` | 目的 |
|---|---:|---:|---|
| GS 零延迟 | `0.0` | `false` | 建立不含逐行时间的动态基线 |
| 理论正值 | 硬件理论值 | `false` | 检查已知扫描方向和量级能否解释残差 |
| 反向值 | 理论值取反 | `false` | 检查残差改善是否具有正确方向性 |
| 自由估计 | 多个合理初值 | `true` | 检查是否收敛到一致且远离边界的结果 |

自由估计还应改变 `knots_per_second`、按连续时间段留出数据、按 tag 或板面区域留出角点，
并用独立动态序列验证。若 $\tau$ 随 knot 密度或时间段大幅变化，说明优化器仍可在“轨迹
形状”和“行时间”之间重新分配误差，不能把某一次最低训练 RMS 当成硬件行时间。

### 10.2 常见症状与检查方向

| 症状 | 首先检查 | 不能直接得出的结论 |
|---|---|---|
| 自由估计贴近 `max_abs_line_delay_s` | 搜索范围、扫描方向、运动激励、时间戳语义 | 行时间就是边界值 |
| RMS 降低但可观性弱 | 轨迹密度敏感性、留出集、K/D 与 $\tau$ 耦合 | RS 已被唯一识别 |
| `alignment` 仍大 | 左右采样时差、运动、K/D/T、普通校正视场 | RS 联合残差也一定大 |
| `rs_compensated_pair_residual` 很小 | 是否为同一批拟合角点、留出结果是否一致 | 已获得独立外部精度证明 |
| 正值和反向值同样改善 | 轨迹自由度、模糊、坏角点、时间戳或模型错误 | 扫描方向已经确认 |

只有训练误差降低不足以证明 RS 是动态重投影误差的原因。报告应把结论限定为“强支持、
重要贡献、证据不足、不支持”之一，并保留运动模糊、标定板平面误差、光学模型不匹配、
左右固定时差和连续轨迹近似等竞争解释。
