# 文档导航

文档保留当前版本的使用方法、输入输出契约、模型与源码说明。

| 文档 | 内容 |
|---|---|
| [机器级一键标定](MACHINE_CALIBRATION_WORKFLOW_ZH.md) | 产线主入口、标定批次目录、分阶段续跑、数据发现与结果复用规则 |
| [产线标定 SOP](CALIBRATION_SOP_ZH.md) | 采集、机器入口、结果检查、评估、放行与封存 |
| [使用指南](USER_GUIDE_ZH.md) | 任务配置、目录数据集、运行与转换 |
| [输入与输出说明](DATASET_WORKFLOW_ZH.md) | 相机 ID、四份配置依赖、输出各文件含义 |
| [v1.0.0 契约](V1_INTERFACE_ZH.md) | 统一版本、数据格式、观测归档、指标与判定 |
| [任务参数](TASK_PARAMETERS_ZH.md) | 字段、默认值、单位、范围与算法作用 |
| [相机模型](CAMERA_MODELS_ZH.md) | 参数顺序、投影公式、各阶段是否参与优化 |
| [初始化](INITIALIZATION_ZH.md) | 物理初值、固定内参、坐标约定与可观性 |
| [标定可观性入门](CALIBRATION_OBSERVABILITY_TUTORIAL_ZH.md) | 面向初学者推导残差、Jacobian、临时状态消元和信息谱，并分别解释单目、双目、Camera–IMU 与 RS 任务 |
| [普通单目与多目标定求解模式](MULTICAMERA_SOLVER_MODES_AND_FULL_BATCH_PLAN_ZH.md) | 单目/多目在不同初值、固定内参与求解模式下的阶段和输出语义 |
| [滚动快门联合标定](ROLLING_SHUTTER_ZH.md) | 独立任务、逐角点时间模型、架构与输出含义 |
| [滚动快门相机标定](ROLLING_SHUTTER_CAMERA_CALIBRATION_ZH.md) | validate/evaluate、单目／多目纯视觉RS任务、两种光学模型、正式 system 与原生单目后端原理和指标 |
| [纯视觉 RS 源码实现说明](ROLLING_SHUTTER_IMPLEMENTATION_REFERENCE_ZH.md) | 按当前代码详解正式与原生后端的状态、残差、Jacobian、样条、运动先验、协方差、默认值和差异 |
| [`cameras-rs` 连续轨迹、样条与运动先验入门](ROLLING_SHUTTER_SPLINE_TUTORIAL_ZH.md) | 面向初学者解释 pose sample、knot、segment、order、轨迹带宽、二阶导数、运动先验及正式实现代码 |
| [滚动快门行时间诊断工具](ROLLING_SHUTTER_DIAGNOSIS_TOOL_ZH.md) | 从普通标定任务自动执行分块筛查、负对照、正式RS敏感性实验并生成证据报告 |
| [源码导读](SOURCE_CODE_DEEP_DIVE_ZH.md) | 输入、求解、输出调用链与维护边界 |
| [输入输出代码修改指南](IO_CODE_MAINTENANCE_ZH.md) | 文件与函数定位、常见修改步骤、报告重生成、构建与验证 |
| [固定参数验证与标定诊断](FIXED_CAMERA_VALIDATION_ZH.md) | 独立测试集 K/D/T 验证、shuffle、可观性与异常点过滤 |
| [相机到 base_link 旋转标定](CAMERA_BASELINK_CALIBRATION_ZH.md) | XY平台文件名位置、固定K/D PnP、全部帧对直接差分旋转与配置平移 |

三组重复标定与三组固定参数测试的双目老化／振动实验使用
`python3 tools/kalibr_stereo_aging_validation.py`。机器目录组织、串行运行和断点复用规则见
仓库技能 `.agents/skills/kalibr-stereo-aging-validation/SKILL.md`。
before/after 完成后可用同一工具的 `diagnose-extrinsics` 在三组 after 测试集上固定参数
验证 K/D、相对旋转与 cam0 坐标系物理基线分量的贡献；该诊断不重新优化标定参数。

产线常规运行使用 `kalibr-cedarobot machine`（默认读取安装根的 `config/user.yaml`），由程序依次完成或复用
双目、双目 Camera–IMU、双目 BaseLink，以及独立单目与单目 BaseLink 阶段，见
[机器级一键标定](MACHINE_CALIBRATION_WORKFLOW_ZH.md)。
研发维护时再按[工程 README](../README.md)构建 `release`、复制
[配置示例](../config/examples/v1.0.0/README_ZH.md)，并直接使用 `kalibr-noros`。
需要定位耗时时使用 `project-profile`，通过 task 或 CLI 显式请求计时。
