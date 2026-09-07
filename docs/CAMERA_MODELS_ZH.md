# 相机模型与优化参数

本文说明当前 Project 版本在相机 task 中可直接填写的全部相机模型、每个模型的参数
顺序，以及这些参数在 `camera_calibration` 和 `camera_imu_calibration` 中是否参与优化。
模型注册表以
[`kalibr_calibrate_cameras`](../src/kalibr/calibration/kalibr/python/kalibr_calibrate_cameras)
中的 `cameraModels` 为准，而不是以 OpenCV 转换工具接受的别名为准。

## 1. 公开模型与优化参数总表

当前 `cameras[].model` 支持以下 10 个值：

| task 模型 | 投影参数 `intrinsics`（维数） | 畸变参数 `distortion_coeffs`（维数） | 单相机参数量 | 典型用途 |
|---|---|---|---:|---|
| `pinhole-radtan` | $[f_u,f_v,c_u,c_v]$（4） | $[k_1,k_2,p_1,p_2]$（4） | 8 | 普通镜头、轻中度广角；原生 Kalibr radtan |
| `pinhole-radtan5` | $[f_u,f_v,c_u,c_v]$（4） | $[k_1,k_2,p_1,p_2,k_3]$（5） | 9 | 需要三阶径向项的普通/较广角镜头 |
| `pinhole-radtan8` | $[f_u,f_v,c_u,c_v]$（4） | $[k_1,k_2,p_1,p_2,k_3,k_4,k_5,k_6]$（8） | 12 | OpenCV rational 8 参数；数据覆盖充分时拟合复杂畸变 |
| `pinhole-equi` | $[f_u,f_v,c_u,c_v]$（4） | $[k_1,k_2,k_3,k_4]$（4） | 8 | 原生 Kalibr equidistant 鱼眼模型 |
| `pinhole-fov` | $[f_u,f_v,c_u,c_v]$（4） | $[w]$（1） | 5 | 单参数 FOV 畸变模型 |
| `pinhole-opencv-fisheye` | $[f_u,f_v,c_u,c_v,\alpha]$（5） | $[k_1,k_2,k_3,k_4]$（4） | 9 | 完整 OpenCV fisheye，并显式标定 skew/alpha |
| `omni-none` | $[\xi,f_u,f_v,c_u,c_v]$（5） | `[]`（0） | 5 | Unified omnidirectional，无附加畸变 |
| `omni-radtan` | $[\xi,f_u,f_v,c_u,c_v]$（5） | $[k_1,k_2,p_1,p_2]$（4） | 9 | Unified omnidirectional + radtan |
| `eucm-none` | $[\alpha,\beta,f_u,f_v,c_u,c_v]$（6） | `[]`（0） | 6 | Extended Unified Camera Model |
| `ds-none` | $[\xi,\alpha,f_u,f_v,c_u,c_v]$（6） | `[]`（0） | 6 | Double Sphere，大视场/鱼眼 |

表中的“单相机参数量”只统计投影参数和畸变参数，不包含相机之间的外参、标定板
位姿、Camera–IMU 外参或时间偏移。

默认 `camera_calibration` 流程中，这 10 种模型的全部投影参数和畸变参数都会进入每台
相机的单目标定 LM，并在全相机 full-batch 与最终增量联合优化中保持 active。相机对
baseline 初始化 LM 是一个原生例外：该阶段投影参数 active、畸变参数暂时 fixed，随后
会重新放开。只有显式设置 `calibration.freeze_intrinsics: true`，才会从头到尾同时
固定这两个参数块。

最容易混淆的两点是：

- `pinhole-radtan` 是 4 个畸变参数，不是 `radtan5` 的别名；
- task 公开的是 `pinhole-opencv-fisheye`。源码中为兼容和实现复用保留的零 skew
  类型不是另一个 task 模型，不能在 `cameras[].model` 中写成
  `opencv_fisheye` 或 `opencv_fisheye_full`。

## 2. 参数含义和投影公式

### 2.1 公共记号

相机坐标系中的空间点记为

$$
{}^{C}\mathbf p=
\begin{bmatrix}X&Y&Z\end{bmatrix}^{\mathrm T}.
$$

对 pinhole 系列，先得到归一化坐标：

$$
x=\frac{X}{Z},\qquad y=\frac{Y}{Z}.
$$

畸变后的归一化坐标记为 $(x_d,y_d)$。除完整 OpenCV fisheye 外，像素投影为：

$$
u=f_u x_d+c_u,\qquad v=f_v y_d+c_v.
$$

其中：

- $f_u,f_v$：横向和纵向焦距，单位为 pixel；
- $c_u,c_v$：主点坐标，单位为 pixel；
- $k_i$：径向或角度多项式系数；
- $p_1,p_2$：切向畸变系数；
- $\xi,\alpha,\beta,w$：对应模型的无量纲形状参数。

图像分辨率 `resolution: [width, height]` 是投影有效域和结果元数据，不是优化变量。

### 2.2 `pinhole-radtan`、`radtan5` 与 `radtan8`

令

$$
r^2=x^2+y^2.
$$

三个模型的切向部分相同：

$$
\begin{aligned}
x_d &= xL(r)+2p_1xy+p_2\left(r^2+2x^2\right),\\
y_d &= yL(r)+p_1\left(r^2+2y^2\right)+2p_2xy.
\end{aligned}
$$

不同点是径向倍率 $L(r)$：

$$
\begin{aligned}
L_{\mathrm{radtan}}(r)
  &=1+k_1r^2+k_2r^4,\\
L_{\mathrm{radtan5}}(r)
  &=1+k_1r^2+k_2r^4+k_3r^6,\\
L_{\mathrm{radtan8}}(r)
  &=\frac{1+k_1r^2+k_2r^4+k_3r^6}
          {1+k_4r^2+k_5r^4+k_6r^6}.
\end{aligned}
$$

`pinhole-radtan8` 与 OpenCV `rational_polynomial` 的前 8 个参数顺序一致，但不包含
thin-prism 的 $s_1\ldots s_4$ 和 tilted-sensor 的 $\tau_x,\tau_y$。它比 radtan5 多
3 个分母系数，因此表达能力更强，也更依赖图像边缘覆盖、标定板姿态和数据量；参数
更多不等于结果一定更好。

### 2.3 `pinhole-equi` 与 `pinhole-opencv-fisheye`

两者都使用 OpenCV fisheye/equidistant 的角度多项式。令

$$
r=\sqrt{x^2+y^2},\qquad \theta=\arctan(r),
$$

则

$$
\theta_d
=\theta\left(1+k_1\theta^2+k_2\theta^4+k_3\theta^6+k_4\theta^8\right),
$$

$$
\begin{bmatrix}x_d\\y_d\end{bmatrix}
=
\begin{cases}
\dfrac{\theta_d}{r}
\begin{bmatrix}x\\y\end{bmatrix}, & r>0,\\[6pt]
\begin{bmatrix}x\\y\end{bmatrix}, & r=0.
\end{cases}
$$

`pinhole-equi` 的投影矩阵不含 skew。`pinhole-opencv-fisheye` 额外优化无量纲参数
$\alpha$：

$$
u=f_u\left(x_d+\alpha y_d\right)+c_u,\qquad
v=f_vy_d+c_v,
$$

因此 OpenCV 相机矩阵中的 skew 为

$$
K_{01}=f_u\alpha.
$$

当 $\alpha=0$ 时，两者的主要投影公式相同；完整 OpenCV 模型的区别是将 alpha/skew
作为第五个投影设计变量，并提供相应的 OpenCV YAML 双向转换。

### 2.4 `pinhole-fov`

FOV 模型只优化一个畸变参数 $w$。令 $r=\sqrt{x^2+y^2}$，其径向缩放为：

$$
s(r,w)=
\frac{\arctan\!\left(2r\tan\dfrac{w}{2}\right)}{rw},
$$

$$
x_d=sx,\qquad y_d=sy.
$$

在 $r\rightarrow0$ 或 $w\rightarrow0$ 时，代码使用对应的稳定极限。当前实现仅接受
$w=0$，或 $0.5\leq w\leq1.5$。

### 2.5 `omni-none` 与 `omni-radtan`

令

$$
d=\sqrt{X^2+Y^2+Z^2},
$$

Unified omnidirectional 模型先计算：

$$
x=\frac{X}{Z+\xi d},\qquad
y=\frac{Y}{Z+\xi d}.
$$

`omni-none` 直接把 $(x,y)$ 映射到像素；`omni-radtan` 再应用 4 参数 radtan
$[k_1,k_2,p_1,p_2]$。这里的 $\xi$ 属于投影参数，不属于 `distortion_coeffs`。

### 2.6 `eucm-none`

Extended Unified Camera Model 使用：

$$
d=\sqrt{\beta\left(X^2+Y^2\right)+Z^2},
$$

$$
x=\frac{X}{\alpha d+(1-\alpha)Z},\qquad
y=\frac{Y}{\alpha d+(1-\alpha)Z}.
$$

随后使用 $f_u,f_v,c_u,c_v$ 映射到像素。$\alpha$ 和 $\beta$ 都在投影参数块中优化，
没有独立畸变参数块。

### 2.7 `ds-none`

Double Sphere 先后使用两个球面距离：

$$
d_1=\sqrt{X^2+Y^2+Z^2},\qquad z_1=\xi d_1+Z,
$$

$$
d_2=\sqrt{X^2+Y^2+z_1^2}.
$$

归一化坐标为：

$$
x=\frac{X}{\alpha d_2+(1-\alpha)z_1},\qquad
y=\frac{Y}{\alpha d_2+(1-\alpha)z_1}.
$$

$\xi$ 和 $\alpha$ 都在投影参数块中优化；该 task 组合没有独立畸变参数块。

## 3. `camera_calibration` 实际优化哪些变量

### 3.1 最终输出参数

对 $N$ 台相机，最终相机标定的物理标定参数包括：

- 每台相机：上表列出的全部投影参数和畸变参数；
- 每对相邻相机：一个 6 自由度外参
  ${}^{C_i}_{C_{i-1}}\mathbf T$，即输出中的 `T_cn_cnm1`；
- 单目任务没有相机间 baseline。

若只计算这些最终物理参数，模型自由度总数为：

$$
n_{\mathrm{calib}}
=\sum_{i=0}^{N-1}
\left(n_{\mathrm{projection},i}+n_{\mathrm{distortion},i}\right)
+6(N-1).
$$

每个已接受 target view 还会建立一个 6 自由度的标定板位姿。这些位姿是为形成重投影
误差而联合求解的 nuisance variables，不是相机标定 YAML 中的相机内外参。

### 3.2 各阶段 active 状态

| 阶段 | 投影参数 | 畸变参数 | 相机间外参 | 每帧标定板位姿 | 说明 |
|---|---|---|---|---|---|
| 几何/解析初值 | 产生初值 | 通常从默认值开始 | 尚未联合求解 | PnP 估计 | 不是最终 BA |
| 单相机内参 LM | active | active | 不存在 | active | 每台相机独立；`omni-radtan` 先额外执行一次畸变 fixed 的 LM |
| 相机对 baseline LM | active | **fixed** | active | active | `stereoCalibrate(..., distortionActive=False)` |
| 全相机 full-batch LM | active | active | active | active | 联合细化全部相机及相邻 baseline |
| 最终增量标定 | active | active | active | active | 按 view 加入、信息增益选择和异常点处理 |

此外：

- 标定板三维点默认 fixed，即 AprilGrid 的 `tagSize`、`tagSpacing` 和角点布局不会被
  优化；
- shutter design variable 在这些阶段为 fixed，当前 task 没有把 rolling-shutter
  参数作为相机标定变量；
- 图像分辨率 fixed；
- `direct`/`refine` 只改变初值如何产生以及跳过哪些前置初值阶段，不会自行把最终
  增量问题中的内参、畸变或 baseline 固定住；需要固定相机模型时显式使用
  `calibration.freeze_intrinsics: true`。无初始化、`refine`、`direct` 以及冻结模式的
  逐阶段 active 变量表见
  [`INITIALIZATION_ZH.md`](INITIALIZATION_ZH.md)。

因此，“给了内外参初值”不等于“固定内外参”。若最终阶段有足够观测，结果仍可以
离开初值。

## 4. `camera_imu_calibration` 实际优化哪些变量

Camera–IMU task 从 `camera_calibration.path` 读入相机链。它会按相机模型计算每一个
角点的重投影残差，但不会把相机模型的 projection/distortion design variable 加入
Camera–IMU 优化问题。因此对上述全部 10 个模型，状态一致：

| 参数块 | 默认状态 | 控制方式 |
|---|---|---|
| 每台相机的 `intrinsics` | fixed | 当前 Camera–IMU task 不提供重新优化开关 |
| 每台相机的 `distortion_coeffs` | fixed | 当前 Camera–IMU task 不提供重新优化开关 |
| IMU 到 cam0 的 ${}^{C_0}_{I}\mathbf T$ | active | 始终作为 Camera–IMU 主外参求解 |
| cam0 之后的相邻相机 baseline | fixed | `recompute_camera_chain_extrinsics: true` 时 active |
| camera–IMU 时间偏移 | active | `no_time_calibration: true` 时 fixed |
| 连续时间位姿 spline 控制点 | active | Camera–IMU 轨迹 nuisance variables |
| gyro bias spline | active | 不是单个常量零偏 |
| accel bias spline | active | 不是单个常量零偏 |
| 重力方向 | active | 默认固定重力模长，只优化方向 |

也就是说，`recompute_camera_chain_extrinsics` 只重新优化相机链外参，不会重新优化双目
内参或畸变。若 Camera–IMU task 使用 `scale-misalignment` IMU 模型，还会增加 IMU 的
尺度、轴不正交和陀螺仪对加速度敏感项；这些与所选相机模型无关，详见
[`TASK_PARAMETERS_ZH.md`](TASK_PARAMETERS_ZH.md)。

## 5. task 与结果 YAML 的名称对应

task 中的模型组合会被拆成结果中的 `camera_model` 和 `distortion_model`：

| task `model` | 结果 `camera_model` | 结果 `distortion_model` |
|---|---|---|
| `pinhole-radtan` | `pinhole` | `radtan` |
| `pinhole-radtan5` | `pinhole` | `radtan5` |
| `pinhole-radtan8` | `pinhole` | `radtan8` |
| `pinhole-equi` | `pinhole` | `equidistant` |
| `pinhole-fov` | `pinhole` | `fov` |
| `pinhole-opencv-fisheye` | `pinhole_opencv_fisheye` | `opencv_fisheye` |
| `omni-none` | `omni` | `none` |
| `omni-radtan` | `omni` | `radtan` |
| `eucm-none` | `eucm` | `none` |
| `ds-none` | `ds` | `none` |

这两个结果字段描述已经标定好的相机，不应原样拼接后当作 task 的模型名。例如结果
中的 `camera_model: pinhole` 和 `distortion_model: radtan5`，在新 task 中应写为
`model: pinhole-radtan5`。

无独立畸变块的 `omni-none`、`eucm-none` 和 `ds-none` 仍输出
`distortion_coeffs: []`。在 `direct` 初始化中也必须显式提供这个空列表，用来区分
“完整的零维参数块”和“遗漏字段”。

## 6. 选型建议

- 普通镜头或尚未达到鱼眼的广角镜头，优先从 `pinhole-radtan5` 开始；若边缘仍有
  明显系统残差且数据覆盖充分，再比较 `pinhole-radtan8`。
- `radtan8` 的优化参数更多。标定板只覆盖中心、姿态单一或图像数量少时，它更容易
  出现参数相关性强、边缘外推不稳定的问题，应同时比较重投影残差、参数可观性和
  去畸变图像，而不能只看平均误差。
- 真实鱼眼优先比较 `pinhole-equi`、`pinhole-opencv-fisheye`、`eucm-none` 和
  `ds-none`。需要与 OpenCV fisheye YAML 完整往返且需要保留 skew 时，使用
  `pinhole-opencv-fisheye`。
- 折反射或 unified omnidirectional 成像使用 `omni-none`/`omni-radtan`；不要仅因
  普通广角镜头视场较大就默认选择 omni。
- 双目两侧允许选择不同模型，但同型号、同镜头的量产双目通常应使用相同模型，并
  检查两侧参数是否符合物理一致性。

## 7. 源码定位

| 内容 | 实现位置 |
|---|---|
| task 可选模型注册表 | [`kalibr_calibrate_cameras`](../src/kalibr/calibration/kalibr/python/kalibr_calibrate_cameras) |
| 原生相机投影与畸变 | [`aslam_cameras`](../src/kalibr/camera/aslam_cameras) |
| radtan5 扩展 | [`radtan5`](../src/camera_models/radtan5) |
| radtan8 扩展 | [`radtan8`](../src/camera_models/radtan8) |
| OpenCV fisheye 扩展 | [`opencv_fisheye`](../src/camera_models/opencv_fisheye) |
| 相机标定 active 参数 | [`CameraIntializers.py`](../src/kalibr/calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py)、[`CameraCalibrator.py`](../src/kalibr/calibration/kalibr/python/kalibr_camera_calibration/CameraCalibrator.py) |
| Camera–IMU active 参数 | [`IccSensors.py`](../src/kalibr/calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py)、[`IccCalibrator.py`](../src/kalibr/calibration/kalibr/python/kalibr_imu_camera_calibration/IccCalibrator.py) |
