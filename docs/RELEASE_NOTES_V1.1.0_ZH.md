# Kalibr no-ROS v1.1.0 版本更新说明

发布日期：2026-11-23
运行包：`/home/gs/kalibr_cedarobo/install/kalibr_cedarobo_v1.1.0`

本说明以已安装的 v1.0.1 为比较基线。变更点汇总工程组织、机器标定、
诊断工具、IMU 工作流和输入校验性能更新。
本次版本封装没有修改原生标定阶段顺序、残差、活动参数、优化器、默认阈值或数学模型。

## 主要更新

### 1. 源码和命令入口

- `kalibr_no_ros` 按 `task/`、`native_bridge/`、`results/`、`conversion/`、`machine/`
  划分职责，核心标定不依赖实验工具实现。
- 随运行包交付的诊断与实验工作流统一位于 `kalibr_tools`，通过
  `kalibr-noros tools <功能>` 调用。
- 当前工具入口包括 `imu-allan`、`fixed-stereo-validation`、
  `rolling-shutter-diagnosis`、`stereo-aging-validation` 和
  `stereo-tag-distance`。

### 2. 机器级 Camera–IMU 模型

普通快门和滚动快门 Camera–IMU 模板均支持以下三种模型：

- `calibrated`
- `scale-misalignment`
- `scale-misalignment-size-effect`

机器结果汇总、跨电脑复用和 HTML/PDF 报告会继续传递模型对应的 IMU 内参：
`M_accel`、`M_gyro`、`C_gyro_i`、`A_gyro_accel`，以及尺寸效应模型的
`rx_i_m`、`ry_i_m`、`rz_i_m`。原有 `gyro_scale`、`acc_scale` 继续保持机型 profile
的下游兼容语义，不作为上述矩阵对角线的别名。

### 3. IMU Allan 噪声分析

新增 `kalibr-noros tools imu-allan`，读取静止 IMU CSV，生成 Camera–IMU 可用的
`imu_noise.yaml` 和独立 HTML/PDF 报告。默认不生成 `gyro.png` 或
`acceleration.png`，也不会自动改写现场配置。

### 4. 结果与辅助工作流

- 固定双目参数验证统一使用 `kalibr-noros tools fixed-stereo-validation`。
- 机器工作流按任务保存正式结果、报告和可选诊断，并在汇总参数中保留相机、IMU、
  BaseLink 标定状态及可复用参数。
- 新增仓库级 `kalibr-runtime-release` skill：发布版本必须由用户明确指定，覆盖时保护
  现场 `config/`，并要求安装根携带本版本说明。
- 机器数据集以结构文件 SHA-256 和图像清单共同标识：不读取图像像素，也不因 `touch`
  重标定；staging 完成后、正式结果替换前再次复核，变化时保留旧成功交付。仅含文件
  metadata 的旧缓存会自动重标定一次，不能把当前 CSV 绑定到无法证明的历史结果。
- 相机内参、畸变、相邻外参和 IMU 内参在任务结果、机器汇总、复用校验和报告之间
  保持明确传递，不从旧字段推断或补造参数。

## 版本与兼容性

软件版本升级为 `1.1.0`，但软件版本不再充当数据格式版本。配置、task、初始化、
目录清单、标定结果、机器状态和 `calib_params` 的结构没有发生破坏性变化，因此
规范 schema 继续为字符串 `"1.0.0"`。v1.1.0 读取时也接受曾经误写的字符串
`"1.1.0"`，并在新生成或更新文档时规范化为 `"1.0.0"`；整数 schema 仍不接受。

机器续跑与参数复用另按 `calibration_behavior_version: "1.0.0"` 判断求解语义。
本次重构没有改变标定数学、默认值、数据选择或坐标方向，所以 v1.0.0 结果可在输入、
profile 与依赖文档语义一致时继续复用。安装路径变化、YAML 排版变化和上述兼容 schema
别名本身不触发重标定。旧状态若缺少语义 profile 哈希且 profile 字节也被修改，无法
证明等价时仍会保守重算一次。第三方 ROS、OpenCV YAML 版本不受此规则影响。完整规则见
[版本兼容说明](VERSION_COMPATIBILITY_ZH.md)。

升级自定义配置时：

1. 优先从 `config/examples/v1.1.0/` 复制同类模板，再迁入数据路径、模型和现场参数。
2. 已符合当前字段结构的手写 YAML 保持 `schema_version: "1.0.0"`，先运行
   `kalibr-noros validate`；已经写成 `"1.1.0"` 的兼容文档无需手工批量修改。
3. 旧标定结果、机器状态、目录清单和 `calib_params` 不应只改版本号；程序会按文档
   schema、标定行为版本、内容哈希和依赖关系分别校验，满足条件才续跑或复用。
4. v1.1.0 示例目录已由 `config/examples/v1.0.0/` 调整为
   `config/examples/v1.1.0/`。

## 发布与验收

本版本使用 `release` 构建发布为精简可搬移运行包，只包含 `bin/`、`lib/`、
`libexec/`、`config/`、`LICENSE`、`licenses/` 和另行生成的简洁根
`README_ZH.md`。本次覆盖发布验证结果：

- `kalibr-noros --version`：`kalibr-noros 1.1.0`
- `kalibr-cedarobot --version`：`kalibr-cedarobot 1.1.0`
- `release` 和 `project-profile` 重新 configure/build：通过
- `project-profile` 完整 `check`：8/8 CTest 通过
- 安装包版本与五个 `kalibr-noros tools` 子命令冒烟检查：通过
- 同一批挂载盘数据输入校验实测如下；优化前后三组数据指纹完全一致：

  | 数据集 | 优化前 | 优化后 | 提升 |
  |---|---:|---:|---:|
  | 40 张双目 | 1.389 s | 0.465 s | 约 3.0× |
  | 364 张双目＋IMU | 8.261 s | 0.736 s | 约 11.2× |
  | 269 张 USB | 7.496 s | 0.458 s | 约 16.4× |
  | 合计 | 17.146 s | 1.659 s | 约 10.3× |

- `touch`、CSV/名称/顺序/数量/大小检查、结构文件 SHA-256 和发布前复核语义均未改变。
- 本轮没有运行真实标定求解；输入校验优化不改变标定数学、参数默认值、筛查门槛、
  输出格式或结果复用语义。

## 已知限制与源码状态

- 图像身份采用相机、CSV 顺序、时间戳、相对路径和文件大小；同路径、同时间戳、同大小
  的像素内容替换不会被发现，适用于采集完成后不编辑图像的生产约束。
- 挂载盘仍需完成一次目录扫描，这是保留图像清单与大小校验的必要成本。
- 正式源码按功能拆分提交，并使用附注 tag `v1.1.0` 标记发布版本。
- 本次覆盖不更新现场 `config/`，发布前后必须保持文件集合与内容哈希完全一致。
