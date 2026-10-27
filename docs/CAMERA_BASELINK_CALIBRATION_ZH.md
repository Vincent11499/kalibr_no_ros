# 相机到 `base_link` 旋转标定

`kalibr-noros calibrate cameras-baselink` 使用固定姿态 XY 平台上全部任意两帧的平移差，估计每台
相机到 `base_link` 的旋转。该任务不重新标定相机 K/D，也不能从这组运动中独立估计
外参平移；K/D 读取已有相机结果，平移由 task 明确给出。

产线入口也支持双目和独立单目模组的该阶段：`kalibr-cedarobot machine --config user.yaml`。
数据放在机器根目录的 `stereo_base_link_YYMMDD_hhmm` 或 `stereo_baselink_YYMMDD_hhmm`，
在 `user.yaml` 配置 `camera_baselink`；工作流自动引用本工单双目结果，不依赖 IMU。
独立单目模组则使用 `usb_YYMMDD_HHMM`、`usb_baselink_YYMMDD_HHMM`
和 `mono_camera_baselink` 配置，只引用本工单的单目结果。
两个模组的 `cam0` 是局部 ID，不能互换；当前机器 profile 的两模组均使用同一份 AprilGrid。
配置和目录示例见[机器级一键标定](MACHINE_CALIBRATION_WORKFLOW_ZH.md)。

```bash
kalibr-noros validate --config stereo_camera_baselink_calibration_task.yaml
kalibr-noros calibrate cameras-baselink \
  --config stereo_camera_baselink_calibration_task.yaml \
  --output-dir output \
  --detector-processes 4
```

双目模板是
[stereo_camera_baselink_calibration_task.yaml](../config/examples/v1.0.0/stereo_camera_baselink_calibration_task.yaml)，
单目／USB 模板是
[mono_camera_baselink_calibration_task.yaml](../config/examples/v1.0.0/mono_camera_baselink_calibration_task.yaml)，
全部字段见
[完整注释模板](../config/examples/v1.0.0/all_params/camera_baselink_calibration_task_full.yaml)。

## 输入契约

当前任务只接受 `dataset.type: directory`。`dataset.yaml` 与普通目录数据集相同；双目
任务的 `cameras` 选择 `cam0`、`cam1`，USB 任务只选择 `cam0`。任务引用的
`camera_calibration.path` 必须包含这些相机的固定内参、畸变和分辨率。

时间戳 CSV 的 `filename` 必须为：

```text
<x>_<y>_<z>.<png|jpg|jpeg|bmp>
```

例如 `43_-5_0.bmp`。程序按 CSV 的时间戳顺序读取位置，并执行：

```text
p_base_m = [x, y, z] * base_link.position_scale_m
```

如果文件名使用厘米，配置 `position_scale_m: 0.01`；如果使用毫米，配置 `0.001`。
该比例必填且必须为有限正数，程序不从数值大小猜单位。

每台相机必须配置：

```yaml
cameras:
  - id: cam0
    translation_base_camera_m: [0.1976, 0.19, 0.39292]
```

它表示 ${}^{B}t_C$，即最终 `T_base_camera` 的平移列，单位米。示例模板中的零只是
可移植占位值，运行真实标定前必须换成该机器的实测值。

## 求解公式

每个成功观测先用固定 K/D 做 AprilGrid PnP，得到标定板在相机系中的平移
$t_i$。对全部成功观测的无序帧对 $i<j$ 构造：

\[
a_{ij}=p_i-p_j,\qquad b_{ij}=t_j-t_i
\]

然后求：

\[
\min_{{}^{B}R_C\in SO(3)}
\sum_{i<j}\left\|a_{ij}-{}^{B}R_C b_{ij}\right\|^2
\]

因此 $N$ 帧最多生成 $N(N-1)/2$ 对，结果不依赖输入帧的排列顺序。实现使用等权 SVD
和反射修正。这就是直接差分 `alpha=0`；公共配置没有 `alpha`、
权重、角度阈值或其他候选算法。平台差分与 PnP 差分都至少需要二维独立激励；第二／
第一奇异值比例低于 `calibration.rank_ratio_min` 时拒绝结果，不能用单一直线运动求旋转。

双目目录中的 cam0、cam1 使用各自固定 K/D 和各自观测独立求解。当前实现不把双目
baseline、同步共视或 stereo 残差加入旋转方程。

## 输出

默认名称为：

```text
camera_baselink_calibration_cam0.yaml
camera_baselink_calibration_cam0_cam1.yaml
```

结果 `calibration_type` 为 `cameras_baselink`。每目包含固定 K/D、完整
`T_base_camera`、`rotation_method`、配置平移来源，以及有效差分对数、激励奇异值和
`movement_residual_mm`。每目还包含 `pitch`，单位度，定义为相机 `+Z` 光轴经
`R_base_camera` 变换后与 `base_link` 水平面的无符号夹角：

```text
optical_axis_base = R_base_camera * [0, 0, 1]
pitch = asin(abs(optical_axis_base.z))
```

`pitch` 范围为 `[0, 90]`：`0` 表示光轴水平，`90` 表示光轴竖直；它不区分光轴朝上
或朝下，也不是欧拉角分解中的有符号 pitch。每次成功交付还会在
`<output.name>/rotation_summary.csv` 按成功 PnP 观测索引的字典序写入全部无序帧对：

```text
Q_ij = R_j R_i^T, i < j
theta_ij = ||log(Q_ij)||
```

CSV 每行对应一对图像，列为 `camera`、`pair_index`、`from_frame`、`to_frame`、
`theta_rad`、更便于人工理解的 `theta_deg`，以及该相机的 `pair_count`、角度最小值、
中位数、均值、P95 和最大值。固定姿态平移实验的理想值为 `0 deg`，数值越小表示两帧
PnP 姿态越稳定。当前算法固定 `alpha=0`，所以这些角度只用于诊断；实际求解仍使用
`Q_tilde_ij=I` 和 `b_ij=t_j-t_i`。坐标方向仍遵循项目统一约定：

```text
p_base = T_base_camera * p_camera
```

报告中的 PnP 重投影 RMS 和运动残差都是样本内拟合诊断，没有独立真值时不能解释为
相机到 `base_link` 的绝对旋转精度，也不能自动作为生产放行结论。
