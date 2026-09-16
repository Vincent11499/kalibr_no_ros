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

## 2. 时间语义

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

## 3. 正式多目联合模型

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

## 4. 初始化

RS 相机任务复用 `camera_calibration_initialization`：

```yaml
initialization:
  path: initialization_stereo.yaml
  strategy: refine
```

相机块按 task 列表映射到 cam0、cam1……，相邻外参仍使用 `T_cn_cnm1`，表示前一相机
到当前相机。`refine` 可只提供部分 K/D 或外参；`direct` 要求每目完整 K/D 和全部相邻
外参。`freeze_intrinsics: true` 另行控制最终联合求解是否固定 K/D，并要求至少双目与
完整 K/D seed。初始化文件不包含行时间；行时间只在 task 的 `rolling_shutter` 中配置，
便于用同一光学初值执行 0、固定和自由行时间对照。

## 5. 与 Kalibr 原生 RS 单目求解器的区别

| 项目 | `cameras-rs` | `native-rs-cameras` |
|---|---|---|
| 相机数量 | 1 台或多台 | 恰好 1 台 |
| 多目外参 | 与 K/D、轨迹、每目行时间联合优化 | 无 |
| 行时间范围 | `bound × tanh(q)`，求解过程中光滑有界 | 原生无界标量，求解后检查配置范围 |
| 像素误差 | 固定 `feature_sigma_px`，可选鲁棒核 | 保留原生随运动自适应协方差 |
| 轨迹 | 观测率感知的共享样条，分段二阶运动先验 | 原生自适应 knot、运动先验与 DogLeg |
| 输出 | 多目正式结果、真实逐角点残差和可观性 | 单目结构化 K/D/tau/RMS；原生 Hessian 秩报告不可用 |

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

## 6. 输出指标如何解释

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

## 7. 数据要求和结论边界

静止数据适合建立稳定 K/D/T 参考，却几乎不能单独辨识行时间。RS 数据应包含不同方向的
平移和旋转、充分的图像行覆盖、距离与倾角变化，并尽量避免严重运动模糊。低频保存或再次
抽帧会降低帧间运动约束；行时间可能与样条自由度、畸变、外参和运动先验耦合。

判断 RS 是否造成动态重投影误差，至少同时比较同求解器 `tau=0`、固定理论正值、反向值、
自由估计与多初值，并做时间段或 tag 留出验证、knot 敏感性和成块重采样。只有训练误差
降低不足以证明原因；报告应把结论分为“强支持、重要贡献、证据不足、不支持”，并保留
模糊、板面、光学模型、左右固定时差和运动近似等竞争解释。
