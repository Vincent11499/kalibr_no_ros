# 配置模板

这里是可以复制后直接修改的 YAML，不是 JSON Schema，也不需要理解
`properties`、`required` 等描述语法。

| 文件 | 用途 |
|---|---|
| `camera_calibration_task.yaml` | 单目或多目相机内外参标定 |
| `camera_imu_calibration_task.yaml` | 相机与 IMU 联合标定 |
| `directory_dataset.yaml` | 图像文件和 IMU CSV 目录的数据清单 |

`euroc/` 保存本机 EuRoC 数据集的完整可运行配置，包括 task、AprilGrid、IMU 参数和
已验证 camchain，运行命令见 [`euroc/README_ZH.md`](euroc/README_ZH.md)。根目录
三份 YAML 只是用于复制的通用模板。

task 中只需要注意：

- `schema_version` 固定为 `1`；
- `job` 与运行的子命令一致；
- `dataset.type` 只能是 `bag` 或 `directory`；
- 相对路径以 task YAML 所在目录为基准；
- 普通运行不必写 `execution`，并行数可从命令行传入。

例如：

```bash
cp config/camera_calibration_task.yaml camera_calibration_task_demo.yaml
kalibr-noros calibrate cameras \
  --config camera_calibration_task_demo.yaml \
  --output-dir output \
  --detector-processes 4 \
  --optimizer-threads 4
```

选择 ROS1 或 ROS2 输入时都写 `type: bag`，程序自动识别具体 bag 格式；选择图像和
CSV 目录时写 `type: directory`，并在数据集根目录放置由
`directory_dataset.yaml` 复制得到的 `dataset.yaml`。
