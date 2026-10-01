# 标定初始化文件

初始化文件为求解器提供物理参数起点。它不增加先验残差，也不会因为数值来自文件就自动固定参数。需要固定相机 K/D 时使用 `calibration.freeze_intrinsics: true`。

## 1. task 与 CLI

task 中引用初始化文件：

```yaml
initialization:
  path: initialization_pinhole_equi.yaml
```

相对路径以 task YAML 所在目录为基准。CLI 可替换路径：

```bash
kalibr-noros calibrate cameras \
  --config stereo_camera_calibration_task.yaml \
  --initialization /absolute/path/initialization_pinhole_equi.yaml \
  --output-dir result
```

CLI 路径相对于执行命令时的工作目录解析，并优先于 task 路径。旧 `initialization.strategy`、初始化文件中的 `strategy` 和 CLI `--initialization-strategy` 已删除；所有 seed 都按本页所述继续 refinement，旧字段会明确报错。

## 2. 相机初始化

文件头：

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

初始化文件中的键使用 task 的相机 ID；内部按 task 列表顺序映射为 `cam0`、`cam1`……。也可使用 `T_cam_from_previous`，含义与 `T_cn_cnm1` 相同：

$$
{}^{C_n}_{C_{n-1}}\mathbf T,qquad
\mathbf p_{C_n} = {}^{C_n}_{C_{n-1}}\mathbf T\,\mathbf p_{C_{n-1}}.
$$

首台相机没有前一台相机，不能填写该字段。多目需为每个相邻相机对分别提供，不能用一条跨相机链变换代替。

另一种等价写法使用明确 ID：

```yaml
extrinsics:
  - from: cam0
    to: cam1
    T:
      - [1.0, 0.0, 0.0, -0.38]
      - [0.0, 1.0, 0.0, 0.0]
      - [0.0, 0.0, 1.0, 0.0]
      - [0.0, 0.0, 0.0, 1.0]
```

反方向 `from/to` 可用，读取时会求逆并规范成相机列表方向。每条边必须连接相邻相机，同一相邻对不能重复。

### 2.1 参数维数

| 模型 | `intrinsics` | `distortion_coeffs` |
|---|---:|---:|
| `pinhole-radtan` | 4 | 4 |
| `pinhole-radtan5` | 4 | 5 |
| `pinhole-radtan8` | 4 | 8 |
| `pinhole-equi` | 4 | 4 |
| `pinhole-fov` | 4 | 1 |
| `pinhole-opencv-fisheye` | 5 | 4 |
| `omni-none` | 5 | 0 |
| `omni-radtan` | 5 | 4 |
| `eucm-none` | 6 | 0 |
| `ds-none` | 6 | 0 |

零维畸变模型显式使用 `distortion_coeffs: []`。

### 2.2 阶段行为

- 省略 K：运行原生解析焦距初始化，再运行单目 LM。
- 提供部分或完整 K/D：先将参数写入相机对象，再运行同一单目 LM；未提供字段保留原生初始值。
- 相邻 baseline 不完整：对缺失链运行原生相机对初始化，再用显式 seed 覆盖对应边。
- 相邻 baseline 完整：跳过相机对 baseline 初始化。
- 多目无论 baseline 是否完整都会运行 S3 full-batch refinement；单目只在
  `solver_mode: full_batch` 下运行 S3。
- 多目 `solver_mode: incremental` 在 S3 后运行 S4；单目 incremental 在 S1 后直接
  运行 S4；`solver_mode: full_batch` 统一采用 S3 终值。

### 2.3 固定内参

```yaml
calibration:
  freeze_intrinsics: true
```

要求至少两台相机，并且每目提供完整、维数正确的 K/D。该配置跳过单目内参 LM，并在后续问题中将 projection/distortion 设为 inactive。相邻 baseline 和每帧板位姿仍继续优化。它不固定 baseline。

## 3. Camera–IMU 初始化

```yaml
schema_version: "1.0.0"
kind: camera_imu_calibration_initialization
camera_imu:
  T_cam0_imu:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu_s:
    cam0: 0.0
  gravity_direction_target: [0.0, 0.0, -1.0]
imus:
  imu0:
    gyroscope_bias_rad_s: [0.0, 0.0, 0.0]
    accelerometer_bias_m_s2: [0.0, 0.0, 0.0]
```

`T_cam0_imu` 的方向为 IMU 到 cam0：

$$
\mathbf p_{C_0} = {}^{C_0}_{I}\mathbf T\,\mathbf p_I.
$$

这些值均为后续原生流程的起点：

- 相机时间 seed 作为互相关结果的中心/偏置继续估计；若 `calibrate_time_offset: false`，不能提供该 seed。
- `T_cam0_imu` 和 gyro bias seed 作为旋转/gyro bias 前置 LM 的起点，该 LM 仍执行。
- 重力方向会归一化到 $9.80655\,\mathrm{m/s^2}$。
- 最终连续时间 spline 联合 LM 的参数活动状态保持原生设置。

多 IMU 可为非参考 IMU 提供：

- `T_imu_from_reference`
- `time_offset_to_reference_s`
- `gyroscope_bias_rad_s`
- `accelerometer_bias_m_s2`
- 与所选 IMU 模型一致的 `M_accel`、`M_gyro`、`C_gyro_i`、`A_gyro_accel`、`ry_i_m`、`rz_i_m`

`time_offset_to_reference_s` 要求 `calibration.estimate_multi_imu_delay: true`。参考 `imu0` 不能填写相对于参考 IMU 的变换或时间偏移。

## 4. Rolling-shutter 初始化

视觉 RS 任务复用相机初始化文件。系统 RS 求解器仍联合优化 K/D、相邻 baseline、共享连续轨迹和按 task 配置为 active 的行时间；`freeze_intrinsics` 只固定 K/D。原生 Kalibr RS 单目对照命令也把 seed 作为 joint solver 的起点，不提供跳过整个前置问题的模式。

RS Camera–IMU 复用 Camera–IMU 初始化文件；行时间来自 task 的 `rolling_shutter` 块，不写入 initialization YAML。

## 5. 校验与追溯

读取时检查：

- `schema_version` 必须是字符串 `"1.0.0"`；
- `kind` 必须与任务一致；
- 向量维数、有限性和模型一致性；
- 4×4 齐次变换底行为 `[0, 0, 0, 1]`；
- 旋转正交且行列式为 +1；
- 相机/IMU ID、相邻边及重复项；
- freeze/time-offset 等配置与 seed 的组合约束。

启用 `output.save_diagnostics: true` 时，`initialization_report.yaml` 保存源路径、SHA-256、ID 映射、参数作用和实际阶段决策。它描述 seed 如何进入求解流程，不代表参数已经通过生产验收。
