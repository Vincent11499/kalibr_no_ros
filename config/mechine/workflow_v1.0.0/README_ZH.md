# 机器标定工作流内部模板

本目录由 `kalibr-cedarobot machine` 使用，不是可直接执行的任务配置。工作流会在运行时
以结构化方式替换数据集路径、相机标定结果路径和输出名称，再把生成的任务保存到机器
输出目录的受管工作区。

模板固定用于以下传感器组合：

- `cam0`、`cam1` 双目相机，模型为 `pinhole-equi`；
- 可选 `imu0`，模型为 `calibrated`；
- AprilGrid 为 8×10、`tagSize=0.07 m`、`tagSpacing=0.3`、`tagStartId=0`。

占位符 `__DATASET_PATH__`、`__CAMERA_CALIBRATION_PATH__` 和
`__OUTPUT_NAME__` 只用于防止模板被误当作真实任务运行。不要在产线手工替换这些值；
机器目录和数据集选择由开发者提供的 `user.yaml` 控制。
