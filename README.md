# Kalibr no-ROS v1.0.0

不依赖 ROS 运行时的相机与 Camera–IMU 标定工程。输入验证、原生 Kalibr 求解和结果
评价分别维护，保留 ETHZ-ASL Kalibr 的阶段顺序、残差、活动参数与优化器默认行为。

## 构建与安装

主机需要编译器、CMake 3.21 及以上、Ninja、Eigen、OpenCV、Boost、TBB，以及
Python 3 开发环境、NumPy、SciPy、PyYAML 和 Matplotlib。依赖脚本在仓库
`.deps/` 中准备 bag、igraph 和 SuiteSparse 依赖，不修改系统 Python：

```bash
./tools/bootstrap_deps.sh
cmake --preset release
cmake --build --preset release --parallel 4
cmake --install build/release
```

默认安装入口为 `install/release/bin/kalibr-noros`。下面示例中的 `kalibr-noros`
可替换为这个路径，也可将安装的 `bin` 目录加入 PATH 后直接使用。

只提供两种构建，最多使用 4 个并行 job：

| Preset | 用途 |
|---|---|
| `release` | Release 优化，关闭性能采样和诊断计时，不编译或安装测试 |
| `project-profile` | Release 优化，允许显式启用阶段计时和进程树内存采样 |

需要分析运行阶段时：

```bash
cmake --preset project-profile
cmake --build --preset project-profile --parallel 4
```

功能与模型检查使用现有 profile 构建，显式执行；不增加第三种构建，也不随默认
构建或安装运行：

```bash
cmake --build --preset project-profile --target check --parallel 4
```

修改 Python 源码后重新执行对应 `cmake --preset ...`，使构建目录中的包同步。

## 运行

从[配置示例](config/examples/v1.0.0/README_ZH.md)复制单目、双目或 Camera–IMU
任务，修改数据路径、标定板和 IMU 参数。示例根目录 YAML 无注释，完整参数说明位于
`all_params/`。

```bash
kalibr-noros validate --config stereo_camera_calibration_task.yaml
kalibr-noros calibrate cameras \
  --config stereo_camera_calibration_task.yaml --output-dir output \
  --detector-processes 4 --optimizer-threads 4
```

Camera–IMU 任务可以省略 `camera_calibration`，从同一个输出目录自动查找唯一匹配的
双目/多目结果；有多个模型结果时显式指定 `camera_calibration.path`。

```bash
kalibr-noros calibrate imu-camera \
  --config camera_imu_calibration_task.yaml --output-dir output
```

目录数据集通过 `dataset.yaml` 将相机 ID 映射到图像与 CSV 路径，任务中可省略相机
`topic` 和 IMU `rostopic`。ROS1/ROS2 bag 输入仍需填写话题。任务内相对路径以任务
YAML 所在目录为基准。软件、任务、初始化、目录清单和结果版本统一为字符串
`"1.0.0"`。

公开命令：

```text
kalibr-noros calibrate cameras
kalibr-noros calibrate imu-camera
kalibr-noros validate
kalibr-noros evaluate
kalibr-noros convert camera
kalibr-noros convert job
```

`convert camera` 提供 OpenCV/Kalibr 参数转换；`evaluate` 只利用已有观测证据重算
指标、判定和报告，不重新检测或优化。九参数 fisheye 的 alpha/skew 无损转换使用
`kalibr-noros convert camera --full-fisheye ...`。

## 模型与输出

支持原生 Kalibr 模型以及 OpenCV 参数顺序的 radtan5/radtan8。鱼眼使用原生
`pinhole-equi`，或包含 `[fu,fv,cu,cv,alpha]` 与四个畸变系数的九参数
`pinhole-opencv-fisheye`。后者保留 `K[0,1] = fu * alpha`，使用统一的
`kalibr_opencv_fisheye` 扩展包。

默认输出按任务与传感器 ID 命名的结果 YAML 及 HTML/PDF 报告，例如
`camera_calibration_cam0_cam1.yaml`。每目附 RMS，外参附 alignment 误差。
左右单目、双目和 Camera–IMU 可以共用输出目录；`output.name` 可以自定义名称。
`save_diagnostics`、`save_metrics`、`export_text` 默认关闭。普通 release 可通过 `output` 配置启用详细观测归档、
角点图、双目校正图和 OpenCV 导出。阶段 `timing.json` 仅在 profile 构建且显式
请求时生成。

## 源码与文档

| 目录 | 职责 |
|---|---|
| `src/kalibr` | 原生相机、轨迹、优化器、标定和基础库 |
| `src/python/kalibr_no_ros` | 输入契约、CLI、初始化、正式输出与评价 |
| `src/python/kalibr_bag_io` | 无 ROS bag 和目录 I/O |
| `src/python/kalibr_runtime` | 执行预算与可选运行诊断 |
| `src/camera_models` | radtan5/radtan8 和九参数 OpenCV fisheye 扩展 |
| `config/examples` | 可移植任务与辅助配置 |
| `tests` | 精简的输入、模型、初始化和输出正确性检查 |

从[文档导航](docs/README_ZH.md)开始；使用见[使用指南](docs/USER_GUIDE_ZH.md)，
输出文件见[数据与输出说明](docs/DATASET_WORKFLOW_ZH.md)，模型见
[模型说明](docs/CAMERA_MODELS_ZH.md)，维护代码见
[源码导读](docs/SOURCE_CODE_DEEP_DIVE_ZH.md)与
[输入输出代码修改指南](docs/IO_CODE_MAINTENANCE_ZH.md)。

This product includes software developed by the Autonomous Systems Lab and
Skybotix AG. 上游版权与许可条款见 [LICENSE](LICENSE)。
