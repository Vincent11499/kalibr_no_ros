# euroc_cam 标定

数据目录：`Q:\File\machine_data\evt3.1-data\euroc_cam`。
双目使用 `stoero_camara.bag`，Camera–IMU 使用 `camara_imu.bag`，保留实际文件名。
两目均使用 `pinhole-radtan`；沿用已有 6×6 AprilGrid 和 200 Hz IMU 噪声配置。
IMU 模型按本次要求设为 `calibrated`，没有重新估计噪声参数。

从工程根目录运行，先相机、后 Camera–IMU：

```bash
build/project-release/bin/kalibr-noros calibrate cameras --config config/euroc/stereo_camera_calibration_task.yaml --output-dir /mnt/q/File/machine_data/evt3.1-data/output/euroc_cam/260909_2018/stereo
build/project-release/bin/kalibr-noros calibrate imu-camera --config config/euroc/camera_imu_calibration_task.yaml --output-dir /mnt/q/File/machine_data/evt3.1-data/output/euroc_cam/260909_2018/camera_imu
```

Camera–IMU 配置在双目标定成功后引用本次 `stereo/calibration.yaml`，不会使用旧的
camchain 或手工初值替代相机结果。后续另开运行目录时，需要同步更新该引用。
现有结果不使用 `--force` 覆盖。

先前完成的 `scale-misalignment` 运行单独保存在本批次
`camera_imu_scale_misalignment_superseded/` 中；`camera_imu/` 为当前的
`calibrated` 运行结果，继续引用同一份双目标定结果。

Camera–IMU bag 的首个 IMU 样本比首张图像晚 15 ms。任务使用
`dataset.time_range_s: [0.02, 72.0]`：按既有读取规则分别相对各话题起点裁剪，
保留的首张相机图像为原起点后 50 ms，首个 IMU 为相机原起点后 35 ms。
该选择使 IMU 覆盖选用图像；不修改原始 bag、时间戳或测量值。
实际保留每目 1438 张图像和 14377 个 IMU 样本；输入原有每目 1439 张图像和
14381 个 IMU 样本。
相机内外参标定使用完整双目 bag，不应用此裁剪。

本次输出开启最终观测、选择历史、OpenCV 参数和可视化；每相机最多展示 30 帧、双目
最多 30 对，指标使用全部最终有效观测。没有业务验收门槛，正式判定为
`not_evaluated`；参考评级仅供显示。
