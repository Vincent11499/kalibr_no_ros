# 文档导航

文档按“日常使用、源码理解、性能验证、历史证据”分为四层，避免同一内容在多个
文件中重复维护。

| 文档 | 适合什么时候看 | 主要内容 |
|---|---|---|
| [`V1_INTERFACE_ZH.md`](V1_INTERFACE_ZH.md) | 使用统一 v1.0.0 输入与输出 | 双工程数据契约、完整与最简示例、离线指标/报告和质量判定 |
| [`DATASET_WORKFLOW_ZH.md`](DATASET_WORKFLOW_ZH.md) | 配置、数据和输出文件分别做什么 | 目录按 ID 选择、四份配置依赖、observations 与 visualizations 逐文件说明 |
| [`USER_GUIDE_ZH.md`](USER_GUIDE_ZH.md) | 准备和运行标定 | 最简 task、目录数据集、CSV、相机模型 |
| [`CAMERA_MODELS_ZH.md`](CAMERA_MODELS_ZH.md) | 选择相机模型或核对优化变量 | 全部模型、参数顺序、投影公式，以及 camera/Camera–IMU 各阶段 active 状态 |
| [`TASK_PARAMETERS_ZH.md`](TASK_PARAMETERS_ZH.md) | 配置或调参 | 两类 task 的全部字段、默认值、算法作用、有效范围和固定参数 |
| [`INITIALIZATION_ZH.md`](INITIALIZATION_ZH.md) | 已有物理初值或弱数据需要诊断 | 两类 seed YAML、refine/direct、坐标单位、模型门控、秩诊断 |
| [`INITIALIZATION_ROBUSTNESS_VALIDATION_ZH.md`](INITIALIZATION_ROBUSTNESS_VALIDATION_ZH.md) | 评估初值可信范围 | EuRoC 10% 随机扰动、差内参、大外参的 direct/refine 实测结果 |
| [`SOURCE_CODE_DEEP_DIVE_ZH.md`](SOURCE_CODE_DEEP_DIVE_ZH.md) | 阅读或修改源码 | 工程架构、坐标约定、相机与 Camera–IMU 调用链、公式 |
| [`BENCHMARK_ZH.md`](BENCHMARK_ZH.md) | 优化耗时和内存 | 冻结基线、公平线程预算、Reference 默认值、归档与比较 |
| [`reports/README_ZH.md`](reports/README_ZH.md) | 追溯已有结论 | 带日期的构建、数值和性能验证原始报告 |
| [`v1.0.0 升级验证`](reports/V1_0_0_UPGRADE_VALIDATION_20260909_ZH.md) | 核对本次升级验收 | 三类真实数据对照、归档开关一致性、离线重评和测试证据 |
| [`两组指定数据集实测`](reports/TWO_DATASETS_VALIDATION_20260909_ZH.md) | 查阅 directory_dataset 和 EuRoC 结果 | 双目到 Camera–IMU 串联、calibrated IMU、指标与输出位置 |

可直接复制和修改的任务、目录数据集模板位于
[`../config/examples/v1.0.0/README_ZH.md`](../config/examples/v1.0.0/README_ZH.md)。字段合法性由 CLI 在运行前
检查，不要求用户阅读 JSON Schema。

推荐阅读顺序：

1. 只运行标定：阅读使用指南的第 1–3 节，再按输入格式选择后续章节；需要修改
   `calibration` 或 `execution` 时查阅 task 参数报告；已有标定初值时再阅读显式初值
   文档；
2. 做性能优化：先读 Benchmark 规范，确保不会重跑或污染冻结基线；
3. 修改算法或并行实现：阅读源码深入导读，再从历史报告核对数值不变量。
