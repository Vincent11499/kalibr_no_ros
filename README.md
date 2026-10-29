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
python3 tools/install_runtime.py \
  --build-dir build/release \
  --prefix /opt/kalibr_cedarobo_v1.0.0
```

上述命令发布可搬移的精简运行包，只含 `bin/`、`lib/`、`libexec/`、`config/`、
根 `LICENSE` 和随私有运行库携带的 `licenses/`；不发布开发用 `include/`、`share/`。
更新已有安装时显式增加 `--replace`，发布器会保留 `config/user.yaml` 和
`config/mechine/` 下与新版不重名的现场新增 YAML。研发需要完整头文件安装时仍可执行
`cmake --install build/release`，它不是产线交付结构。

安装提供两个入口：产线机器级工作流使用
`install/release/bin/kalibr-cedarobot`，研发维护与单任务运行使用
`install/release/bin/kalibr-noros`。下面示例中的命令也可在将安装的 `bin` 目录加入
PATH 后直接使用。

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

产线常规操作由开发者把 `user.yaml` 放到安装根的 `config/`，操作员只运行：

```bash
kalibr-cedarobot machine
```

也可用 `--config /path/to/user.yaml` 临时覆盖默认文件。

当前批次目录只有 `stereo_*` 时先完成双目标定并正常等待；以后加入 `stereo_imu_*` 后重复
同一命令，程序复用双目结果并继续 Camera–IMU。新增可选 Camera–BaseLink 阶段：配置
`camera_baselink` 并提供 `stereo_base_link_*` 后复用双目 K/D 求旋转，不需要 IMU 数据。
同一机器的独立单目模组使用 `usb_*` 标定内参，使用 `usb_base_link_*` 和
`mono_camera_baselink` 配置复用单目结果求旋转；不会混用双目 `cam0`。两模组共用当前
AprilGrid，均使用 `pinhole-equi`。
`user.yaml` 可用 `camera.mode: rolling_shutter` 选择 RS 双目，用
`camera_imu.recompute_extrinsics: true` 控制 Camera–IMU 联合阶段重新优化相机链外参；
`target` 只填写安装在 `config/mechine` 下的标定板文件名stem。
算法开发者也可在 `config/mechine/` 增加初始化或标定板 YAML，并在
`config/mechine/workflow_v1.0.0/*.task.yaml` 中填写相对路径。路径以 task 所在目录为
基准，例如根目录文件写成 `../custom_initialization.yaml`，且不能逃出
`config/mechine/`。
数据发现、批次目录、重名处理和输出
位置见[机器级一键标定工作流](docs/MACHINE_CALIBRATION_WORKFLOW_ZH.md)。
`datasets` 可省略或部分填写：未填写的双目、Camera–IMU、双目 BaseLink、USB 单目和
USB BaseLink 角色都从 `machine_dir` 自动选择唯一候选；填写的角色才强制使用指定目录。
只有严格匹配 `*_YYMMDD_HHMM` 约定的直接子目录才是候选，`*_bk`、旧名和其他非规范
目录静默跳过。重复的规范候选、缺少上游数据或 BaseLink 配置会在任何求解启动前报错。
精度不足时可删除旧数据集或将它改为非规范名称，再加入唯一的新规范目录；工作流自动
重跑该任务及其下游依赖，保留旧数据集结果，同名下游结果仅在新求解成功后安全替换。
`machine_dir` 指向 `<机型>-<编号>/<批次名>`，例如 `EVT312-02/1029`；新一批数据新建
批次目录并修改该字段，不跨批次复用。机器入口把每个任务的
正式 YAML 和诊断写入 `result/<数据集名>/`，HTML/PDF 报告平铺在 `result/`；并在
`result/calib_params/` 按已完成模组分别生成 `vision/stereo_imu_calib.yaml` 和
`mono_camera/usb_mono_calib.yaml`；不再生成 `stereo_calib.yaml`。
双目、Camera–IMU 与双目 BaseLink 更新 vision，独立单目及其 BaseLink 更新 usb 文件。

下面的 `kalibr-noros` 是研发维护与单任务接口。

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
kalibr-noros calibrate cameras-baselink
kalibr-noros calibrate cameras-rs
kalibr-noros calibrate native-rs-cameras
kalibr-noros calibrate imu-camera
kalibr-noros calibrate imu-camera-rs
kalibr-noros validate
kalibr-noros evaluate
kalibr-noros verify cameras
kalibr-noros convert camera
kalibr-noros convert job
```

`convert camera` 提供 OpenCV/Kalibr 参数转换；`evaluate` 只利用已有观测证据重算
指标、判定和报告，不重新检测或优化。九参数 fisheye 的 alpha/skew 无损转换使用
`kalibr-noros convert camera --full-fisheye ...`。

固定姿态 XY 平台上的相机到 `base_link` 旋转使用
`calibrate cameras-baselink`。它读取已有相机 K/D，以图像文件名中的 `x_y_z` 平台位置
对全部任意两帧做等权直接差分，仅估计旋转；平移在 task 中明确给出。输入和公式见
[相机到 base_link 旋转标定](docs/CAMERA_BASELINK_CALIBRATION_ZH.md)。
机器 workflow 写出时，双目 BaseLink 旋转固定左乘 `Rx(180°)`，USB 单目 BaseLink
旋转固定左乘 `Ry(180°)`；配置平移保持不变。

需要在新的目录测试集上固定已有双目 K/D/T 计算每目 RMS、双目综合 RMS 和
Alignment RMS 时，使用 `kalibr-noros verify cameras`。该命令会重新检测测试图像，
但不调用标定优化器；显式加入 `--visualizations` 可按参数标签输出测试集角点、单目
去畸变和双目极线对齐图。验证同时输出 JSON 汇总和逐图像/逐双目图对的 CSV，包含
重投影与对齐误差的 RMS/P95/Max。参数、公式和输出见
[固定参数验证与标定诊断](docs/FIXED_CAMERA_VALIDATION_ZH.md)。

滚动快门相机＋IMU 使用独立任务 `camera_imu_rolling_shutter_calibration` 和
`calibrate imu-camera-rs`，逐目联合估计行时间；配置和架构见
[滚动快门联合标定](docs/ROLLING_SHUTTER_ZH.md)。普通任务的默认行为不变。

纯视觉滚动快门相机使用 `camera_rolling_shutter_calibration` 和 `calibrate cameras-rs`，
支持单目、双目及多目联合 K/D、相邻外参、连续轨迹和每目行时间。临时
`calibrate native-rs-cameras` 只支持单目，用于与 Kalibr 原生 RS 求解器对照；它使用
同一 task schema，但只接受文档列出的原生后端参数子集。时间基准、配置和两种算法差异见
[滚动快门相机标定](docs/ROLLING_SHUTTER_CAMERA_CALIBRATION_ZH.md)。

不确定一组动态数据是否足以支持行时间模型时，先使用
`python3 tools/kalibr_rolling_shutter_diagnosis.py`。它从普通标定观测做严格分块筛查，
只有证据通过才运行正式 RS 正反值、多初值及轨迹带宽对照；配置、门槛与输出见
[滚动快门行时间诊断工具](docs/ROLLING_SHUTTER_DIAGNOSIS_TOOL_ZH.md)。

## 模型与输出

支持原生 Kalibr 模型以及 OpenCV 参数顺序的 radtan5/radtan8。鱼眼使用原生
`pinhole-equi`，或包含 `[fu,fv,cu,cv,alpha]` 与四个畸变系数的九参数
`pinhole-opencv-fisheye`。后者保留 `K[0,1] = fu * alpha`，使用统一的
`kalibr_opencv_fisheye` 扩展包。

默认输出按任务与传感器 ID 命名的结果 YAML 及 HTML/PDF 报告，例如
`camera_calibration_cam0_cam1.yaml`。每目附 RMS，外参附 alignment 误差。
左右单目、双目和 Camera–IMU 可以共用输出目录；`output.name` 可以自定义名称。
上述为普通 `kalibr-noros` 的兼容平铺布局；`kalibr-cedarobot machine` 使用
`result/<数据集名>/<数据集名>.yaml` 保存结果和证据，并把同名 HTML/PDF 报告放在
`result/` 根目录。
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

从[文档导航](docs/README_ZH.md)开始；产线使用见
[机器级一键标定](docs/MACHINE_CALIBRATION_WORKFLOW_ZH.md)和
[标定 SOP](docs/CALIBRATION_SOP_ZH.md)，底层 task 使用见[使用指南](docs/USER_GUIDE_ZH.md)，
输出文件见[数据与输出说明](docs/DATASET_WORKFLOW_ZH.md)，模型见
[模型说明](docs/CAMERA_MODELS_ZH.md)，维护代码见
[源码导读](docs/SOURCE_CODE_DEEP_DIVE_ZH.md)与
[输入输出代码修改指南](docs/IO_CODE_MAINTENANCE_ZH.md)。

This product includes software developed by the Autonomous Systems Lab and
Skybotix AG. 上游版权与许可条款见 [LICENSE](LICENSE)。
