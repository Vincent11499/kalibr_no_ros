# 机器标定工作流内部模板

本目录由 `kalibr-cedarobot machine` 使用，不是可直接执行的任务配置。工作流会在运行时
以结构化方式替换数据集路径、相机标定结果路径和输出名称，再把生成的任务保存到机器
输出目录的受管工作区。

模板内 `target.path`、`initialization.path`、`imus[].path` 均相对于当前目录。需要新增
标定板或初始化时，可把 YAML 放到上一级 `config/mechine/`，并在模板中写
`../文件名.yaml`。解析结果必须留在 `config/mechine/` 内，绝对路径、越界路径和符号
链接会被拒绝；这些外部 YAML 也会进入任务指纹，语义变化时重跑实际依赖阶段。

模板固定用于以下传感器组合：

- `cam0`、`cam1` 双目相机，模型为 `pinhole-equi`；
- 另一个独立单目模组，其局部 ID 为 `cam0`，模型为 `pinhole-equi`；
- 可选 `imu0`，模型为 `calibrated`；
- 省略 `user.yaml.target` 时，两个模组共用内部 AprilGrid：8×10、`tagSize=0.07 m`、
  `tagSpacing=0.3`、`tagStartId=0`；显式 target 从已安装 `config/mechine` 根目录按文件名stem选择。

固定辅助文件为 `实验室_aprilgrid_8x10.yaml`、`initialization_stereo_camera.yaml`、
`initialization_mono_camera.yaml`、`imu_noise.yaml` 和 `mechine_base_param.yaml`。任务文件为
`stereo_camera_calibration_task.yaml`、`stereo_camera_rs_calibration_task.yaml`、
`stereo_imu_calibration_task.yaml`、`stereo_imu_rs_calibration_task.yaml` 及对应 BaseLink/单目任务。

占位符 `__DATASET_PATH__`、`__CAMERA_CALIBRATION_PATH__` 和
`__OUTPUT_NAME__` 只用于防止模板被误当作真实任务运行。不要在产线手工替换这些值；
单次标定批次目录和数据集选择由开发者提供的 `user.yaml` 控制；批次父目录提供
`<机型>-<编号>`。同一工单精度不足时可删除旧数据集或把它改成非规范名称，再加入
唯一的新规范目录；真正的新工单才使用新的批次目录。
算法开发者临时修改已安装模板后可重复运行同一批次：最终 task 或其外部 YAML 输入的
有效字段变化会触发对应任务及下游安全重标，只改注释、空白或键顺序仍复用旧结果。
`datasets` 可省略或部分填写；未填写角色从 `machine_dir` 的直接子目录自动选择唯一候选，
填写的角色才强制使用指定目录。目录只接受严格的 `stereo_*`、`stereo_imu_*`、
`stereo_base_link_*`、`usb_*` 和 `usb_base_link_*` 时间后缀名称；`*_bk`、旧名及其他
非规范目录静默跳过。同类唯一的新规范数据会触发对应任务和依赖任务重跑，旧数据集结果
保留，同名下游结果仅在新求解成功后替换。

`stereo_camera_baselink_calibration_task.yaml` 另外由工作流注入
`user.yaml` 的 `camera_baselink.cameras`。内部默认 `frame_id=base_link`、
`position_scale_m=0.01`；其他文件名位置单位才需显式覆盖 `camera_baselink.base_link`。
每目实测平移仍必须明确配置；内部模板不提供零平移默认值。
该可选阶段复用双目 K/D，按全部无序帧对直接差分求旋转，不依赖 IMU 数据。
工作流在输出前固定左乘绕 X 轴 180° 的旋转修正，配置平移保持不变。

`mono_camera_calibration_task.yaml` 只标定独立单目模组的 K/D，使用独立的
`initialization_mono_camera.yaml`，不套用双目初始化。
`mono_camera_baselink_calibration_task.yaml` 只引用它的结果，由工作流注入
`user.yaml` 的 `mono_camera_baselink.base_link` 和 `mono_camera_baselink.cameras`。
该配置只接受 `[cam0]`，不得引用双目相机结果或继承双目的配置平移。
省略其 `base_link` 时同样使用 `frame_id=base_link`、`position_scale_m=0.01`。
工作流在输出前固定左乘绕 Y 轴 180° 的旋转修正，配置平移保持不变。
两个模组的结果通过数据集角色、独立阶段和精确上游路径区分，不合成三目联合标定。
