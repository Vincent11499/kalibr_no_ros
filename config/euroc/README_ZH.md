# EuRoC 可运行配置

这组配置对应本机已经验证过的 EuRoC 数据：

```text
/mnt/q/File/kalibr/data/euroc_cam/cam_april.bag
/mnt/q/File/kalibr/data/euroc_cam/imu_april.bag
```

双目相机内外参标定：

```bash
build/project-profile/bin/kalibr-noros calibrate cameras \
  --config config/euroc/camera_calibration_task.yaml \
  --output-dir /mnt/q/File/kalibr/data/euroc_cam/output_config_camera
```

双目 Camera–IMU 标定，使用 `scale-misalignment`：

```bash
build/project-profile/bin/kalibr-noros calibrate imu-camera \
  --config config/euroc/camera_imu_calibration_task.yaml \
  --output-dir /mnt/q/File/kalibr/data/euroc_cam/output_config_imu_camera
```

两份 task 已固定 `detector_processes: 4` 和 `optimizer_threads: 4`。CLI 仍可临时
覆盖，例如追加 `--detector-processes 8 --optimizer-threads 8`。

相机 task 使用双目 `pinhole-radtan5` 并固定 `shuffle: false`。Camera–IMU task
引用本目录中已经验证的双目 `radtan` camchain 和 ADIS16448 IMU 噪声参数，因此它
可以独立运行，不要求先重新执行相机标定。两个输出目录若已经存在结果，程序会
拒绝覆盖；确认需要替换时才添加 `--force`。

## 与初值配置的边界

本目录只包含**无初值** task，用于原生自动初始化和冻结基线对比。需要初值时，不必在
命令行追加参数，直接选择相应目录的 task：

- [`../euroc_init/`](../euroc_init/README_ZH.md)：正常或 `10%` 扰动初值，默认 `refine`；
- [`../euroc_bad_init/`](../euroc_bad_init/README_ZH.md)：差内参或大外参测试，默认 `refine`。

初值坐标方向、单位和策略语义见
[`../../docs/INITIALIZATION_ZH.md`](../../docs/INITIALIZATION_ZH.md)。
