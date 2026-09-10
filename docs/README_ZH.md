# 文档导航

文档保留当前版本的使用方法、输入输出契约、模型与源码说明。

| 文档 | 内容 |
|---|---|
| [使用指南](USER_GUIDE_ZH.md) | 任务配置、目录数据集、运行与转换 |
| [输入与输出说明](DATASET_WORKFLOW_ZH.md) | 相机 ID、四份配置依赖、输出各文件含义 |
| [v1.0.0 契约](V1_INTERFACE_ZH.md) | 统一版本、数据格式、观测归档、指标与判定 |
| [任务参数](TASK_PARAMETERS_ZH.md) | 字段、默认值、单位、范围与算法作用 |
| [相机模型](CAMERA_MODELS_ZH.md) | 参数顺序、投影公式、各阶段是否参与优化 |
| [初始化](INITIALIZATION_ZH.md) | direct/refine、物理初值、坐标约定与可观性 |
| [源码导读](SOURCE_CODE_DEEP_DIVE_ZH.md) | 输入、求解、输出调用链与维护边界 |
| [输入输出代码修改指南](IO_CODE_MAINTENANCE_ZH.md) | 文件与函数定位、常见修改步骤、报告重生成、构建与验证 |

先按[工程 README](../README.md)构建 `release`，再复制
[配置示例](../config/examples/v1.0.0/README_ZH.md)。运行时先完成相机标定，
Camera–IMU 再引用生成的相机结果。需要定位耗时时使用 `project-profile`，
通过任务或 CLI 显式请求计时。
