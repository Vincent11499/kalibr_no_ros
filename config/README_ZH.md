# 配置模板

这里是可以复制后直接修改的 YAML，不是 JSON Schema，也不需要理解
`properties`、`required` 等描述语法。

| 文件 | 用途 |
|---|---|
| `camera_calibration_task.yaml` | 单目或多目相机内外参标定 |
| `camera_imu_calibration_task.yaml` | 相机与 IMU 联合标定 |
| `directory_dataset.yaml` | 图像文件和 IMU CSV 目录的数据清单 |

EuRoC 配置按是否带初值明确分开：

| 目录 | 用途 |
|---|---|
| [`euroc/`](euroc/README_ZH.md) | 不带初值；原生自动初始化和冻结基线对比 |
| [`euroc_init/`](euroc_init/README_ZH.md) | 带正常/`10%` 扰动初值；task 默认 `refine` |
| [`euroc_bad_init/`](euroc_bad_init/README_ZH.md) | 带差内参或大外参；只用于鲁棒性测试 |

三个目录的 task 都能直接传给 `--config`。`euroc_init` 和 `euroc_bad_init` 已在 task
内部选择初值，不要求额外的初始化 CLI 参数。根目录三份 YAML 只是用于复制的通用模板。

task 中只需要注意：

- task 的 `schema_version` 固定为整数 `1`；可选 initialization 文件也使用自己的
  schema v1，而程序输出的 `calibration.yaml` 是结果 schema v2；
- `job` 与运行的子命令一致；
- `dataset.type` 只能是 `bag` 或 `directory`；
- 相对路径以 task YAML 所在目录为基准；
- 普通运行不必写 `execution`，并行数可从命令行传入。

显式初值也是可选 task 顶层块：

```yaml
initialization:
  path: camera_calibration_initialization.yaml
  strategy: refine
```

`strategy` 可省略并默认为 `refine`。初值文件的格式、`direct` 的完整性要求和 CLI
覆盖规则见
[`../docs/INITIALIZATION_ZH.md`](../docs/INITIALIZATION_ZH.md)。

多相机任务若要固定 seed 中的内参和畸变、只重新估计相机间 baseline，可增加：

```yaml
calibration:
  freeze_intrinsics: true
```

该开关默认 `false`，要求至少两台相机和每台相机完整的内参、畸变 seed；详细阶段
行为见 [`../docs/TASK_PARAMETERS_ZH.md`](../docs/TASK_PARAMETERS_ZH.md)。

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
