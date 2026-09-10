# 显式物理初值、分阶段初始化与可观性诊断

本文说明 `camera_calibration` 和 `camera_imu_calibration` 如何从独立 YAML 接收
物理参数初值。这个接口解决的是“已有可信近似值时，如何绕过脆弱的自动初值阶段，
或从更好的位置开始优化”。`initialization` 本身不会增加固定约束或先验残差；相机
内参是否固定由独立的 `calibration.freeze_intrinsics` 活动策略决定。

需要先明确三个边界：

1. task、初始化和标定结果的 `schema_version` 统一为字符串 `"1.0.0"`，
   各文档通过 `job`/`kind` 区分语义，不接受旧项目整数版本；
2. 只有显式配置初始化文件时才进入新路径；未配置时仍走原生 Kalibr 的自动初始化、
   优化活动集合和输出路径；
3. 初值只能改善初始点，不能从缺少激励的数据中创造信息。显式初值运行会在最终状态
   重新线性化并检查标定参数秩。只有基于机器 epsilon 的结构/数值 hard rank
   gate 不通过时任务失败；更保守的 operational 阈值只用于弱可观性警告。

## 1. 接入 task 和 CLI

### 1.1 task 写法

在原有 task 顶层添加一个可选块：

```yaml
initialization:
  path: camera_calibration_initialization.yaml
  strategy: refine
```

`path` 相对于 **task YAML 所在目录**。`strategy` 可省略，缺省为 `refine`：

```yaml
initialization: {path: camera_calibration_initialization.yaml}
```

只写 `strategy` 而没有初始化路径会被拒绝。初始化文件不把 `strategy` 写在自身内部，
因为同一份物理初值应该可以由不同 task 分别选择 `refine` 或 `direct`。

### 1.2 CLI 写法和覆盖优先级

两个标定子命令都支持：

```text
--initialization PATH
--initialization-strategy refine|direct
```

例如，完成第 8 节的输入和可信初值准备后，从复制后的示例目录运行：

```bash
kalibr-noros calibrate cameras \
  --config all_params/stereo_camera_calibration_task_full.yaml \
  --output-dir output/stereo \
  --initialization initialization_stereo.yaml \
  --initialization-strategy refine
```

CLI 的 `--initialization` 相对路径以**执行命令时的当前目录**为基准。覆盖采用字段级
合并，而不是整块替换：

```text
CLI --initialization
> task initialization.path

CLI --initialization-strategy
> task initialization.strategy
> refine
```

因此，task 若配置 `strategy: direct`，CLI 只替换 `--initialization new-seed.yaml` 时，
有效策略仍是 `direct`。反过来，CLI 只给策略而 task 也没有路径时会立即报错。

## 2. 公共校验与坐标约定

初始化 YAML 执行严格白名单校验：未知字段、错误向量长度、`NaN`、无穷值、错误相机
编号和不匹配的 `kind` 都会在运行原生算法前被拒绝。非法初值也不会先清空已有输出。

所有外参都必须给完整 $4\times4$ 齐次矩阵。本文使用：

$$
{}^{A}_{B}\mathbf T
=
\begin{bmatrix}
{}^{A}_{B}\mathbf R & {}^{A}\mathbf t_B\\
\mathbf 0^{\mathrm T} & 1
\end{bmatrix},
\qquad
{}^{A}\mathbf p
=
{}^{A}_{B}\mathbf T\,{}^{B}\mathbf p.
$$

也就是说，矩阵左上标对应目标坐标系，右下标对应源坐标系。旋转块必须满足：

$$
{}^{A}_{B}\mathbf R^{\mathrm T}
{}^{A}_{B}\mathbf R
=\mathbf I,
\qquad
\det\left({}^{A}_{B}\mathbf R\right)=1,
$$

最后一行必须为 $[0,0,0,1]$。接口不会接受四元数、旋转向量或“只给平移”的缩写，
从而避免单位、四元数排列和变换方向被静默猜错。

## 3. `camera_calibration` 初值

### 3.1 文件结构

```yaml
schema_version: "1.0.0"
kind: camera_calibration_initialization

cameras:
  cam0:
    intrinsics: [460.98, 459.72, 369.03, 247.06]
    distortion_coeffs: [-0.317, 0.140, 0.00032, -0.00083, -0.0342]

  cam1:
    intrinsics: [459.65, 458.16, 382.79, 253.97]
    distortion_coeffs: [-0.312, 0.130, -0.00003, -0.00073, -0.0290]
    T_cn_cnm1:
      - [0.999995, 0.002444, -0.001775, -0.110214]
      - [-0.002419, 0.999902, 0.013785, 0.000581]
      - [0.001808, -0.013780, 0.999903, -0.000902]
      - [0.0, 0.0, 0.0, 1.0]
```

`cameras` 的键使用 task 中的实际相机 `id`。例如只标定
`id: cam1` 时，内参写在 `cameras.cam1`；使用 `left`、`right` 时也采用这两个 ID。
ID 必须唯一，由字母、数字、下划线或连字符组成，首字符为字母或数字。bag task
未填写 ID 时，按列表顺序默认 `cam0`、`cam1`；目录 task 应明确写出 ID。

`T_cn_cnm1` 放在当前相机条目下，将 task 列表中前一相机的点变换到当前相机。
上述 `cameras.cam1.T_cn_cnm1` 表示 cam0 → cam1：

$$
{}^{C_k}_{C_{k-1}}\mathbf T,
\qquad
{}^{C_k}\mathbf p
=
{}^{C_k}_{C_{k-1}}\mathbf T\,{}^{C_{k-1}}\mathbf p.
$$

程序保持 task `cameras[]` 的顺序，不按 ID 名称、初始化映射顺序或 topic 重排。
每条外参连接列表中相邻两台相机。三目列表 `[left, right, rear]` 中，
`right.T_cn_cnm1` 表示 left → right，`rear.T_cn_cnm1` 表示 right → rear。
第一台相机不得填写此字段。输入方向相反时，应先对矩阵求逆。

多目链连续复合为：

$$
{}^{C_k}_{C_0}\mathbf T
=
{}^{C_k}_{C_{k-1}}\mathbf T
\cdots
{}^{C_1}_{C_0}\mathbf T.
$$

平移单位是米。不要把 OpenCV stereo 中方向相反的 $\mathbf R,\mathbf t$ 未取逆就写入。
方向不能根据相机 ID 的数字大小推断，始终以 task 列表顺序为准。

内部仍按 task 顺序使用 `cam0`、`cam1` 和 `T_cam_from_previous`，这是输入适配后的
原生格式，不改变求解问题。旧版 `cameras.<id>.T_cam_from_previous` 仍可读取，含义
是 task 列表前一相机到当前相机；列表中的第一台相机禁止填写。对于同一条边，旧字段
与 `T_cn_cnm1` 不能同时出现，即使矩阵数值相同也会报冲突。
旧顶层 `extrinsics` 仅兼容读取，新示例不再使用它。

### 3.2 内参与畸变向量长度

向量长度由 task 中每个相机的 `model` 决定：

| task 模型 | `intrinsics` 顺序 | `distortion_coeffs` 顺序 | 单相机参数量 |
|---|---|---|---:|
| `pinhole-radtan` | $[f_u,f_v,c_u,c_v]$ | $[k_1,k_2,p_1,p_2]$ | 8 |
| `pinhole-radtan5` | $[f_u,f_v,c_u,c_v]$ | $[k_1,k_2,p_1,p_2,k_3]$ | 9 |
| `pinhole-radtan8` | $[f_u,f_v,c_u,c_v]$ | $[k_1,k_2,p_1,p_2,k_3,k_4,k_5,k_6]$ | 12 |
| `pinhole-equi` | $[f_u,f_v,c_u,c_v]$ | $[k_1,k_2,k_3,k_4]$ | 8 |
| `pinhole-fov` | $[f_u,f_v,c_u,c_v]$ | $[w]$ | 5 |
| `pinhole-opencv-fisheye` | $[f_u,f_v,c_u,c_v,\alpha]$ | $[k_1,k_2,k_3,k_4]$ | 9 |
| `omni-none` | $[\xi,f_u,f_v,c_u,c_v]$ | `[]` | 5 |
| `omni-radtan` | $[\xi,f_u,f_v,c_u,c_v]$ | $[k_1,k_2,p_1,p_2]$ | 9 |
| `eucm-none` | $[\alpha,\beta,f_u,f_v,c_u,c_v]$ | `[]` | 6 |
| `ds-none` | $[\xi,\alpha,f_u,f_v,c_u,c_v]$ | `[]` | 6 |

其中 OpenCV fisheye 的 skew 满足 $K_{01}=f_u\alpha$。无独立畸变块的模型仍应在
`direct` 中显式写 `distortion_coeffs: []`，因为“完整 seed”和“漏写字段”必须可区分。
各模型的公式、使用范围和默认优化阶段统一见
[`CAMERA_MODELS_ZH.md`](CAMERA_MODELS_ZH.md)。

### 3.3 `refine` 的实际阶段

`refine` 允许只提供部分相机、部分参数或部分相邻外参；只有 `extrinsics` 时可省略
`cameras`。各阶段行为为：

1. 若给出某相机 `intrinsics`，跳过会覆盖它的解析式内参估计；若未给则保留原生
   自动内参估计；
2. 将显式 `intrinsics`/`distortion_coeffs` 写入相机模型，然后仍运行原生单相机
   内参与畸变 LM；
3. 未给出的相邻 baseline 仍由原生相机对 stereo LM 生成；显式 baseline 覆盖对应
   相邻链初值；
4. 仍运行原生全相机 full-batch refinement；
5. 最终增量 GN/LM、信息增益筛选、异常值处理和报告生成保持原有流程。

若所有 baseline 都已提供，pairwise stereo LM 可以省去，但 full-batch refinement 仍
执行。因此 `refine` 的目标是保留原生收敛路径，同时避免明显错误的解析初值。

### 3.4 `direct` 的实际阶段

相机 `direct` 要求完整提供：

- 每个相机的 `intrinsics`；
- 每个相机的 `distortion_coeffs`，无畸变模型写空列表；
- 覆盖 task 相机列表全部相邻边的 `extrinsics`，共 $N-1$ 条；单目不需要外参。

完整 seed 会跳过单相机内参 LM、相机对 baseline LM 和 full-batch 初值优化，但仍会：

- 读取图像并检测标定板；
- 建立多相机共同观测图并检查连通性；
- 为每个 target view 计算 PnP/位姿初值；
- 执行最终增量标定、信息增益选择和异常值流程。

最终增量问题中的相机内参、畸变和 baseline 仍是原有 active 状态，可以离开 seed。
`direct` 不是“固定相机参数”，只是声明这些值足够可信，可以省略前置初值优化。

### 3.5 初始化策略与参数活动状态

为避免把“提供初值”“跳过前置阶段”和“冻结参数”混为一谈，本节统一使用以下记号：

- $\mathbf I$：每台相机的投影内参，例如 pinhole 的
  $[f_u,f_v,c_u,c_v]$；
- $\mathbf D$：每台相机的畸变参数；
- $\mathbf B$：相邻相机间的 baseline
  ${}^{C_k}_{C_{k-1}}\mathbf T$；
- $\mathbf P$：每个 target view 的标定板位姿。

这里的 `active` 表示对应 design variable 会被加入当前优化问题并允许产生增量；
`fixed` 表示本阶段使用其当前值计算残差，但不允许优化器更新。$\mathbf P$ 是形成
重投影误差所必需的 nuisance variable，不是最终输出中的相机内外参。

#### 3.5.1 不提供初始化文件

不配置 `initialization` 时，程序保持原生 Kalibr 的完整自动初始化和优化流程：

| 阶段 | active 变量 | fixed 或不参与的相机参数 |
|---|---|---|
| 单相机内参 LM | $\mathbf I,\mathbf D,\mathbf P$ | 该阶段尚无 $\mathbf B$ |
| 相机对 stereo LM | $\mathbf I,\mathbf B,\mathbf P$ | $\mathbf D$ 暂时 fixed |
| 全相机 full-batch LM | $\mathbf I,\mathbf D,\mathbf B,\mathbf P$ | — |
| 最终增量优化 | $\mathbf I,\mathbf D,\mathbf B,\mathbf P$ | — |

相机对阶段暂时固定畸变，是原生
`stereoCalibrate(..., distortionActive=False)` 的行为；畸变会在后续 full-batch 和
最终增量问题中重新放开。

#### 3.5.2 `refine`

`refine` 只改变部分参数的起点，并保留原生前置 refinement：

| 阶段 | 行为和 active 变量 |
|---|---|
| 解析焦距初始化 | 已提供 $\mathbf I$ 的相机可跳过；未提供的相机仍自动估计 |
| 单相机 LM | $\mathbf I,\mathbf D,\mathbf P$ 仍 active，允许离开 seed |
| 相机对 stereo LM | 缺失 baseline 时继续运行；运行时 $\mathbf I,\mathbf B,\mathbf P$ active，$\mathbf D$ fixed；所有 baseline 均已提供时可跳过该阶段 |
| full-batch | $\mathbf I,\mathbf D,\mathbf B,\mathbf P$ 全部 active |
| 最终增量优化 | $\mathbf I,\mathbf D,\mathbf B,\mathbf P$ 全部 active |

因此，`refine` 不会永久冻结任何相机参数。若把显式 seed 记为
$\boldsymbol\theta_0$，其语义只是设置优化起点：

$$
\boldsymbol\theta\leftarrow\boldsymbol\theta_0,
\qquad
\delta\boldsymbol\theta\ \text{仍可由优化器更新}.
$$

#### 3.5.3 `direct`

`direct` 要求完整的相机内参、畸变和 baseline seed，并跳过三个前置优化阶段，但不
改变最终问题的参数活动策略：

| 阶段 | 行为和 active 变量 |
|---|---|
| 单相机内参 LM | 跳过 |
| 相机对 stereo LM | 跳过 |
| full-batch 初始化 LM | 跳过 |
| 最终增量优化 | $\mathbf I,\mathbf D,\mathbf B,\mathbf P$ 重新全部 active |

所以不能用 `direct` 实现“固定内参，只标定外参”。它仅表示信任完整 seed，允许程序
从最终增量阶段开始；一旦进入该阶段，内参和畸变仍可能变化。

#### 3.5.4 `freeze_intrinsics`

`camera_calibration` 支持把参数活动策略与 `refine`/`direct` 初始化策略正交配置：

```yaml
initialization:
  path: camera_calibration_initialization.yaml
  strategy: refine

calibration:
  freeze_intrinsics: true
```

`freeze_intrinsics: true` 同时固定 $\mathbf I$ 和 $\mathbf D$。只固定焦距、主点而允许
畸变继续变化，不能称为“固定相机模型”。其约束语义应为：

$$
\mathbf I=\mathbf I_0,
\qquad
\mathbf D=\mathbf D_0,
\qquad
\delta\mathbf I=\mathbf 0,
\qquad
\delta\mathbf D=\mathbf 0.
$$

该字段默认是 `false`；省略或显式设为 `false` 时，阶段和 active 状态与原生路径完全
相同。设为 `true` 时，必须配置 `initialization.path`，而且初始化文件必须为每台相机
完整提供与模型维数一致的 `intrinsics` 和 `distortion_coeffs`。baseline seed 可在
`refine` 中省略；`direct` 仍要求完整 baseline。该模式至少需要两台相机，因为单目
任务冻结相机模型后不存在需要估计的相机外参。

实际分阶段行为为：

| 阶段 | `freeze_intrinsics: true` 后的行为 |
|---|---|
| 输入校验 | 每台相机必须已有完整 $\mathbf I,\mathbf D$，且长度与相机模型严格一致 |
| 单相机内参 LM | 跳过 |
| PnP pose 初始化 | 使用固定 $\mathbf I,\mathbf D$ 计算，$\mathbf P$ 仍需初始化 |
| 相机对 stereo LM | 只优化 $\mathbf B,\mathbf P$；允许由该阶段生成缺失的 baseline 初值 |
| full-batch | 只优化 $\mathbf B,\mathbf P$ |
| 最终增量优化 | 物理 calibration block 只优化 $\mathbf B$，同时保持每帧 $\mathbf P$ active |
| 异常点删除后重建问题 | 必须继续保持 $\mathbf I,\mathbf D$ fixed，不能被工厂默认值重新激活 |

在这种配置下，“只优化外参”的准确含义是：**相机标定的物理参数中只有
$\mathbf B$ active**；每帧 $\mathbf P$ 仍必须参与联合求解。单目任务不存在
$\mathbf B$，因此配置 `freeze_intrinsics: true` 会在运行前被明确拒绝。

这个概念不要套用到 `camera_imu_calibration`：该任务当前已经始终固定相机
$\mathbf I,\mathbf D$。其相邻 baseline 默认也 fixed；只有
`recompute_camera_chain_extrinsics: true` 才会放开 $\mathbf B$，而 cam0–IMU 外参、
轨迹、bias、重力和按配置启用的时间偏移等状态仍按 Camera–IMU 联合问题求解。

## 4. `camera_imu_calibration` 初值

### 4.1 文件结构

Camera–IMU 初值与 camera task 的相机内参初值是两个独立文件类型：

```yaml
schema_version: "1.0.0"
kind: camera_imu_calibration_initialization

camera_imu:
  T_cam0_imu:
    - [0.0, 1.0, 0.0, 0.06]
    - [-1.0, 0.0, 0.0, -0.02]
    - [0.0, 0.0, 1.0, -0.01]
    - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu_s:
    cam0: -0.0001
    cam1: -0.0001
  gravity_direction_target: [1.0, 0.0, 0.0]

imus:
  imu0:
    gyroscope_bias_rad_s: [0.0, 0.0, 0.0]
    accelerometer_bias_m_s2: [0.0, 0.0, 0.0]
    M_accel:
      - [1.0, 0.0, 0.0]
      - [0.0, 1.0, 0.0]
      - [0.0, 0.0, 1.0]
    M_gyro:
      - [1.0, 0.0, 0.0]
      - [0.0, 1.0, 0.0]
      - [0.0, 0.0, 1.0]
    C_gyro_i:
      - [1.0, 0.0, 0.0]
      - [0.0, 1.0, 0.0]
      - [0.0, 0.0, 1.0]
    A_gyro_accel:
      - [0.0, 0.0, 0.0]
      - [0.0, 0.0, 0.0]
      - [0.0, 0.0, 0.0]
```

两个顶层参数块都可省略，也允许块内只给部分字段。`camera_imu` 描述相机链与参考
IMU 的关系；`imus.imu0`、`imus.imu1` 按 task 的 `imus[]` 顺序匹配。

### 4.2 `camera_imu` 字段

| 字段 | 形状和单位 | 含义 |
|---|---|---|
| `T_cam0_imu` | $4\times4$，平移 m | ${}^{C_0}_{I_0}\mathbf T$，把参考 IMU 坐标变到相机结果列表中的第一台相机 |
| `timeshift_cam_imu_s` | `camera_id: float` mapping，s | 每个相机的 $t_{I_0}=t_{C_k}+\Delta t_k$ |
| `gravity_direction_target` | 非零三向量 | 标定板/世界坐标系中的重力方向，长度会归一化到 $9.80655\ \mathrm{m/s^2}$ |

时间偏移必须按相机写成 mapping，单个 scalar 会被拒绝。这样多相机任务不会把“只给
cam0”静默解释为“所有相机相同”。相机键必须使用输入相机结果中的真实 ID，例如
`front`、`rear`。程序按该结果的列表顺序映射到原生 `cam0`、`cam1`。

`initialization_report.yaml` 的 `camera_id_mapping` 记录真实 ID 到原生编号的对应关系。
报告中的 `configured` 是完成映射后的求解器初值，源文件保持不变。

Camera–IMU task 输入的 camchain 仍负责相机内参、畸变和相邻 baseline。该初始化文件
不重复这些字段；是否在最终联合优化中放开相邻 baseline 仍由
`calibration.recompute_camera_chain_extrinsics` 控制。

### 4.3 每个 IMU 的字段

| 字段 | 形状和单位 | 初始化的内部状态 |
|---|---|---|
| `gyroscope_bias_rad_s` | 3，rad/s | gyro bias spline 的常量初值，并参与旋转初值阶段 |
| `accelerometer_bias_m_s2` | 3，m/s² | accel bias spline 的常量初值 |
| `M_accel` | $3\times3$ | accelerometer 下三角尺度/非正交矩阵 |
| `M_gyro` | $3\times3$ | gyroscope 下三角尺度/非正交矩阵 |
| `C_gyro_i` | $3\times3$ 旋转矩阵 | IMU 坐标到 gyro sensitive axes 的旋转 |
| `A_gyro_accel` | $3\times3$，$(\mathrm{rad/s})/(\mathrm{m/s^2})$ | gyro 对加速度敏感矩阵 |
| `ry_i_m` | 3，m | size-effect 模型的 y 轴 accelerometer lever arm |
| `rz_i_m` | 3，m | size-effect 模型的 z 轴 accelerometer lever arm |
| `T_imu_from_reference` | $4\times4$，平移 m | 非参考 IMU 的 ${}^{I_k}_{I_0}\mathbf T$ |
| `time_offset_to_reference_s` | scalar，s | 非参考 IMU 的 $t_{I_0}=t_{I_k}+\Delta t_{I_k}$ |

`imu0` 是参考 IMU，禁止出现 `T_imu_from_reference` 和
`time_offset_to_reference_s`。它们只对 `imu1` 及后续 IMU 有意义。

`M_accel` 和 `M_gyro` 必须是对角线为正的下三角矩阵：

$$
\mathbf M
=
\begin{bmatrix}
m_{00} & 0 & 0\\
m_{10} & m_{11} & 0\\
m_{20} & m_{21} & m_{22}
\end{bmatrix},
\qquad
m_{00},m_{11},m_{22}>0.
$$

`A_gyro_accel` 是完整矩阵，`C_gyro_i` 必须满足旋转矩阵约束。scale-misalignment 模型
中简化的测量关系为：

$$
\widetilde{\mathbf a}
=
\mathbf M_a\,{}^{I}\mathbf a
+\mathbf b_a,
$$

$$
\widetilde{\boldsymbol\omega}
=
\mathbf M_g\,{}^{G}_{I}\mathbf R\,{}^{I}\boldsymbol\omega
+\mathbf A_{ga}\,{}^{G}_{I}\mathbf R\,{}^{I}\mathbf a
+\mathbf b_g.
$$

size-effect 模型进一步给各 accelerometer sensitive axis 使用不同 lever arm。由于共同
平移规范自由度，`rx_i` 固定为零；公共 seed 只接受 `ry_i_m` 和 `rz_i_m`。

### 4.4 IMU 模型门控

初始化字段必须与 task 中相应 `imus[].model` 一致：

| IMU 模型 | 可用 seed |
|---|---|
| `calibrated` | 两种 bias；非参考 IMU 的相对变换和时间偏移 |
| `scale-misalignment` | 上述字段，加 `M_accel`、`M_gyro`、`C_gyro_i`、`A_gyro_accel` |
| `scale-misalignment-size-effect` | 上述全部字段，加 `ry_i_m`、`rz_i_m` |

例如，在 `calibrated` 模型下误写 `M_accel` 不会被忽略，而会报模型不匹配。这个设计
避免用户以为尺度矩阵已经生效，实际状态量却根本不存在。

### 4.5 `refine` 与 `direct` 的分阶段差异

两种策略最终都进入原有 Camera–IMU 联合 LM；最终 active 状态由算法和 task 开关
决定，不由 seed 永久固定。差异只存在于联合 LM 之前：

| 初始化阶段 | `refine` | `direct` |
|---|---|---|
| 相机–IMU 时间偏移 | 从 seed 开始，用角速度模长互相关求剩余修正 | 已给的相机直接采用 seed，跳过其互相关 |
| cam0–IMU 旋转与 gyro bias 小型 LM | 已给值作为 active 起点继续联合求初值 | 已给量在该小型 LM 暂时固定；两者都给时可跳过小型 LM |
| cam0–IMU 平移 | 写入 pose spline/最终联合问题的初始外参 | 相同 |
| 重力方向 | 归一化到 $9.80655\ \mathrm{m/s^2}$ 后初始化 gravity state | 相同 |
| gyro/accel bias | 作为常量初始化 bias splines | 相同；最终 bias splines 仍 active |
| IMU intrinsic/lever arm | 直接初始化相应 design variable | 相同；最终仍 active |
| 非参考 IMU 时间偏移 | seed 加互相关/连续优化的剩余修正 | 已给时跳过偏移相关初始化 |

使用 `refine` 的相机时间 seed 时，`calibration.calibrate_time_offset` 必须为 `true`；
否则没有后续时间标定路径。非参考 IMU 的 `refine` 时间 seed 同理要求
`calibration.estimate_multi_imu_delay: true`。`direct` 可以在跳过相关初始化后使用给定
时间值，Camera–IMU 每相机时间偏移是否在最终联合 LM 中 active，仍取决于
`calibrate_time_offset`。

## 5. 如何选择策略

推荐按初值可信程度选择：

| 场景 | 推荐策略 | 原因 |
|---|---|---|
| 厂商参数、上次标定或粗略手工测量，误差仍可能明显 | `refine` | 保留原生前置 LM/相关估计来修正初值 |
| 同一硬件的高质量历史标定，目标是缩短重复标定前置阶段 | `direct` | 避免重复运行脆弱或昂贵的初值优化 |
| 数据运动弱、自动相机内参或外参初始化失败 | 先 `refine` | 好 seed 可让优化进入正确吸引域，但仍接受前置修正 |
| 没有可信物理 seed | 不配置初始化 | 使用原生自动初始化，检查运动与标定板覆盖 |

若 seed 的旋转方向写反、平移单位误用毫米、焦距超出图像尺度或时间偏移符号错误，
`direct` 更容易把问题带到错误吸引域。它不会自动回退到原生初始化；失败应由用户修正
seed，而不是静默换一条算法路径。

不要为了让文件看起来“完整”而对未知物理量填零。阶段机只根据字段是否存在决定
是否跳过 preliminary 阶段，不能判断该数值是测量值还是占位值。例如 Camera–IMU
`direct` 同时收到 `T_cam0_imu` 和 `gyroscope_bias_rad_s` 时，会跳过旋转/gyro
bias 前置 LM；若 bias 只是零占位，应优先用 `refine`。

## 6. 两份诊断 sidecar

显式初值通过校验且输出目录准备完成后，会先生成初值 provenance；只有运行到最终解的
可观性重线性化阶段，才会同时生成两份诊断文件：

```text
initialization_report.yaml
observability.yaml
```

### 6.1 `initialization_report.yaml`

该文件用于回答“本次运行究竟用了哪份 seed、采用什么策略、哪些块被显式给出、哪些
阶段被使用、跳过或只在 preliminary 阶段暂时固定”。它是运行 provenance/诊断信息，
不是下一次标定要直接喂回的 seed 文件。

它包含：

- seed 的绝对源路径、SHA-256，以及路径和策略分别来自 task、CLI 还是默认值；
- 完整的规范化 `configured` 初值块和 `initial_value_only` 语义声明；
- 每个初始化阶段的 `run`、`skipped`、`fixed_in_preliminary` 等决策；
- `configured`、`completed`、`failed` 或 `failed_rank_deficient` 状态；
- 成功时最终 `calibration.yaml` 的 SHA-256，以及 observability 的 `quality`、hard rank
  和 operational rank 摘要；
- 失败时异常类型和消息。

`quality: weakly_observable` 是成功但需要复核的工程警告，对应的
`initialization_report.yaml` 仍为 `status: completed`，不是 `failed_rank_deficient`。

其中 `fixed_parameter: false` 表示 **seed 本身没有额外增加固定状态或先验**，不表示 task
原本 inactive 的参数会被强制改成 active。例如关闭时间标定时，`direct` 时间 seed 仍按
原任务规则作为固定预计算偏移使用。

若数据集、target 或内部命令在最终重线性化之前失败，输出目录只保留
`initialization_report.yaml`；此时不存在 `observability.yaml`。非法 seed 会在输出目录
清理之前被拒绝，因此不会改动已有结果。

应重点检查：

- 有效 `kind` 和 `strategy` 是否符合预期；
- 每个参数块的来源是显式 seed 还是原生自动初始化；
- `direct` 是否按预期跳过了初值阶段；
- `status`、结果 hash 和 observability 摘要是否完整。

该报告**不直接列出最终物理参数相对 seed 的变化量**。最终值读取
`calibration.yaml`，并与 `configured` 块按相同坐标方向自行比较；秩、谱和零空间参数
块贡献读取 `observability.yaml`。

### 6.2 `observability.yaml`

该文件在最终解处重新线性化：普通 residual 保留其 M-estimator 权重，不加 LM
阻尼，并做 $L_2$ 列尺度归一化。把轨迹、每帧 target pose、bias spline 等记作
nuisance 参数 $\mathbf x_n$，把需要报告的物理标定块记作 $\mathbf x_c$。

Camera–IMU 中的 B-spline motion regularization 是 Hessian-only 二次项：原生优化器通过
`buildHessian()` 直接累加精确稀疏矩阵 $\mathbf Q$，它本身不提供普通 residual
Jacobian。为了不在 Jacobian 型的 QR/SVD 可观性分析中丢掉这部分 nuisance 信息，
诊断路径按 spline segment 构造等价平方根：

$$
\mathbf c_s
=
\mathbf P_s\mathbf c,
\qquad
\mathbf Q_s
=
\mathbf R_s^{\mathrm T}\mathbf R_s,
$$

$$
\mathbf Q
=
\sum_s
\mathbf P_s^{\mathrm T}
\mathbf Q_s
\mathbf P_s,
$$

其中 $\mathbf P_s$ 从全局 spline 系数 $\mathbf c$ 中选出第 $s$ 段的局部系数。诊断路径
为每段创建临时 prior error term，其等价全局 Jacobian 为：

$$
\mathbf J_s
=
\mathbf R_s\mathbf P_s,
\qquad
\sum_s \mathbf J_s^{\mathrm T}\mathbf J_s
=
\mathbf Q.
$$

按段分解不需要形成或分解全局稠密 $\mathbf Q$。

这里的“等价”严格指对可观性有用的局部 Hessian/Fisher 信息等价，不要将这些
诊断 residual 解释为主优化器新的代价函数或更新方向。这是**只读诊断中的等价
平方根分段展开**：临时 residual 不会写回原优化问题，
不更新任何参数，且分析后恢复设计变量与误差项的 ordering metadata。主标定仍
使用原生 Hessian-only `buildHessian()` 路径，所以展开只让可观性分析看到等价局部
信息，**不改变优化路径或标定结果**。报告中的 `original_error_terms`、
`analyzed_error_terms`、`expanded_quadratic_error_terms` 和
`quadratic_expansion: segment_sqrt_information` 用于记录这个展开。

概念上的降维信息矩阵为 Schur 补：

$$
\mathbf H_{c\mid n}
=
\mathbf H_{cc}
-
\mathbf H_{cn}
\mathbf H_{nn}^{-1}
\mathbf H_{nc}.
$$

实现使用稀疏 QR/SVD 处理消元和秩判断，不会把上式中的逆显式形成。设降维谱为
$\sigma_1\geq\cdots\geq\sigma_{n_c}$。秩诊断使用同一个原生 `rankTol` 公式，但有两层
明确不同的语义：

$$
\tau_{\mathrm{hard}}
=
\sigma_1\epsilon_{\mathrm{machine}}n_c,
\qquad
\operatorname{rank}_{\mathrm{hard}}
=
\#\{i\mid\sigma_i>\tau_{\mathrm{hard}}\},
$$

其中 double 机器 epsilon 约为 $2.22\times10^{-16}$。这是原生 `LinearSolverOptions`
默认的结构/数值 hard rank gate；只有
$\operatorname{rank}_{\mathrm{hard}}<n_c$ 才会阻止有效结果输出。

$$
\tau_{\mathrm{operational}}
=
\sigma_1\cdot10^{-6}\cdot n_c,
\qquad
\operatorname{rank}_{\mathrm{operational}}
=
\#\{i\mid\sigma_i>\tau_{\mathrm{operational}}\}.
$$

$10^{-6}$ 来自原生相机最终增量流程的 `epsSVD`，在这里只是 operational
弱可观性标尺。若 hard rank 完整但 operational rank 不完整，任务仍成功并
保留 `calibration.yaml`，同时在诊断中给出警告。因此 operational rank 不能被当成
“标定失败”的判定条件。

报告包含：

- 顶层 `status` 表示 hard gate：`full_rank`、`rank_deficient`、`no_information`；
  分析本身异常时为 `analysis_failed` 并附异常信息；
- 分析成功时，顶层 `quality` 表示质量分类：`nominal`、`weakly_observable`、
  `rank_deficient`、`no_information`；
- 每个物理参数块的维数、active 维数、单位和坐标约定；
- nuisance QR 的列数、秩、缺失维数和 tolerance；
- `calibration.rank`、`calibration.deficiency`、`calibration.svd_tolerance` 是
  hard rank 结果，`calibration.rank_policy` 为
  `native_rank_tol_machine_epsilon`；
- `calibration.operational_rank`、`calibration.operational_deficiency`、
  `calibration.operational_svd_tolerance`、`calibration.operational_eps_svd: 1e-6` 和
  `calibration.operational_spectral_gap` 描述弱可观工程阈值；
- `nullspace_modes` 报告 hard 零空间的物理块贡献，顶层
  `operationally_truncated_modes` 报告 operational 阈值下被截断的方向，其中也可能
  包含 hard 零空间方向。

相机任务的 calibration block 包含每台相机 active 的投影内参、畸变和相邻 baseline
旋转/平移。Camera–IMU 任务包含 cam0–IMU 外参、每相机 active 时间偏移、被
`recompute_camera_chain_extrinsics` 放开的 baseline、多 IMU 相对外参，以及所选 IMU
模型存在的尺度/非正交、gyro-axis、g-sensitivity 和 size-effect 参数。轨迹 spline、
target pose、bias spline 和重力等随时间或规范相关状态作为 nuisance 消元；inactive
块仍可列在 `parameter_blocks`，但 `active_dimension` 为零，不计入 calibration rank。

分析只读最终状态，不应用求解出的更新。若 hard rank gate 不通过，显式初值任务
视为失败：不会把看似稳定但由 seed 支撑的数值当成有效 `calibration.yaml`；
`initialization_report.yaml` 和 `observability.yaml` 会保留用于定位问题。只有 operational
弱可观性警告时不会阻止输出，但用户仍应检查弱方向并增加运动激励。

需要注意，列尺度归一化让米、弧度、秒和无量纲 intrinsic 可以在同一秩测试中比较，
但 condition number 仍是局部线性诊断，不是全局唯一性证明，也不能替代不同时间窗口
和不同运动激励下的重复标定。

## 7. 初值不能修复的数据问题

下列情况即使提供“准确答案”也可能触发 hard rank 失败或 operational 弱可观
警告：

- 标定板只出现在图像很小区域，焦距、主点和畸变强相关；
- 双目没有足够共同观测，连接图不连通；
- Camera–IMU 基本没有三轴转动，外参旋转、gyro bias 和时间偏移不可分；
- 缺少线加速度和角加速度，平移、accelerometer intrinsic 或 size-effect lever arm
  不可观；
- 相机与 IMU 有效时间范围不重叠；
- 选择高维 IMU 模型，但数据长度和激励只够 calibrated 模型。

正确处理顺序是：先检查 topic/时间戳/坐标方向和单位，再增加标定板覆盖与运动激励，
必要时降低 IMU 模型维数。不要通过关闭秩诊断、增加 LM 迭代次数或把最终参数锁在 seed
附近来掩盖信息不足。

## 8. v1.0.0 完整示例流程

[`config/examples/v1.0.0`](../config/examples/v1.0.0/README_ZH.md) 提供匹配的 task 与初始化模板：

| 完整 task | 初始化模板 | 匹配模型 |
|---|---|---|
| [`all_params/mono_camera_calibration_task_full.yaml`](../config/examples/v1.0.0/all_params/mono_camera_calibration_task_full.yaml) | [`initialization_mono.yaml`](../config/examples/v1.0.0/initialization_mono.yaml) | 单目 `pinhole-equi` |
| [`all_params/stereo_camera_calibration_task_full.yaml`](../config/examples/v1.0.0/all_params/stereo_camera_calibration_task_full.yaml) | [`initialization_stereo.yaml`](../config/examples/v1.0.0/initialization_stereo.yaml) | 双目 `pinhole-equi` |
| [`all_params/camera_imu_calibration_task_full.yaml`](../config/examples/v1.0.0/all_params/camera_imu_calibration_task_full.yaml) | [`initialization_imu.yaml`](../config/examples/v1.0.0/initialization_imu.yaml) | 读取双目结果；IMU 为 `calibrated` |

先把整个示例目录复制到可写运行目录，保留 task 和辅助 YAML 的相对位置，并将
`kalibr-noros` 加入 PATH。按示例 README 将 task 的
`data/stereo_imu_YYMMDD_hhmm` 占位路径替换为真实新格式数据目录，核对 AprilGrid、
IMU 噪声参数和话题。

所有初始化默认未启用。模板中的 900 px 焦距、零畸变、100 mm baseline、单位
Camera–IMU 变换和零 bias 是教学占位值。先替换为本次硬件的可信物理初值，再取消
所需完整 task 顶层 `initialization` 的三行注释，保留 `strategy: refine`。
`direct` 的使用仍须满足前文的完整性和可信度要求。

从复制后的示例目录运行；单目与双目任务可按需求选择，Camera–IMU 任务需要先完成
下面的双目标定：

```bash
kalibr-noros validate --config all_params/mono_camera_calibration_task_full.yaml
kalibr-noros calibrate cameras --config all_params/mono_camera_calibration_task_full.yaml --output-dir output

kalibr-noros validate --config all_params/stereo_camera_calibration_task_full.yaml
kalibr-noros calibrate cameras --config all_params/stereo_camera_calibration_task_full.yaml --output-dir output

kalibr-noros calibrate imu-camera --config all_params/camera_imu_calibration_task_full.yaml --output-dir output
```

`all_params/camera_imu_calibration_task_full.yaml` 默认省略相机结果路径；两个任务使用同一个输出目录时自动查找唯一匹配双目/多目结果；
有多个候选或独立运行 validate 时请显式填写 camera_calibration.path。相同任务再次运行请使用新输出名称或目录保留既有记录。
同一示例的 task 与 seed 使用一致的模型和相机顺序；这些等距畸变 seed 不能直接用于
采用 `pinhole-radtan` 或其他畸变模型的相机任务。

也可以保留 task 中初始化块的注释，通过第 1.2 节的 CLI 参数一次性启用初值。
task 内的相对初始化路径以 task YAML 所在目录为基准；CLI 相对路径以当前目录为基准。
