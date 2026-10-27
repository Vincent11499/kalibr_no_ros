# 机器标定工作流内部模板

本目录由 `kalibr-cedarobot machine` 使用，不是可直接执行的任务配置。工作流会在运行时
以结构化方式替换数据集路径、相机标定结果路径和输出名称，再把生成的任务保存到机器
输出目录的受管工作区。

模板固定用于以下传感器组合：

- `cam0`、`cam1` 双目相机，模型为 `pinhole-equi`；
- 另一个独立单目模组，其局部 ID 为 `cam0`，模型为 `pinhole-equi`；
- 可选 `imu0`，模型为 `calibrated`；
- 省略 `user.yaml.target` 时，两个模组共用内部 AprilGrid：8×10、`tagSize=0.07 m`、
  `tagSpacing=0.3`、`tagStartId=0`；显式 target 从已安装 `config/mechine` 根目录按文件名stem选择。

占位符 `__DATASET_PATH__`、`__CAMERA_CALIBRATION_PATH__` 和
`__OUTPUT_NAME__` 只用于防止模板被误当作真实任务运行。不要在产线手工替换这些值；
机器目录和数据集选择由开发者提供的 `user.yaml` 控制。

`stereo_camera_baselink_calibration_task.yaml` 另外由工作流注入
`user.yaml` 的 `camera_baselink.cameras`。内部默认 `frame_id=base_link`、
`position_scale_m=0.01`；其他文件名位置单位才需显式覆盖 `camera_baselink.base_link`。
每目实测平移仍必须明确配置；内部模板不提供零平移默认值。
该可选阶段复用双目 K/D，按全部无序帧对直接差分求旋转，不依赖 IMU 数据。

`mono_camera_calibration_task.yaml` 只标定独立单目模组的 K/D，不套用双目初始化。
`mono_camera_baselink_calibration_task.yaml` 只引用它的结果，由工作流注入
`user.yaml` 的 `mono_camera_baselink.base_link` 和 `mono_camera_baselink.cameras`。
该配置只接受 `[cam0]`，不得引用双目相机结果或继承双目的配置平移。
省略其 `base_link` 时同样使用 `frame_id=base_link`、`position_scale_m=0.01`。
两个模组的结果通过数据集角色、独立阶段和精确上游路径区分，不合成三目联合标定。
