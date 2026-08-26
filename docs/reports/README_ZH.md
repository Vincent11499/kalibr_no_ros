# 历史验证报告

这里保存已经完成且带日期的验证证据。它们记录当时的构建、数据集、耗时和数值
结果，不作为当前接口说明，也不因后续代码优化而改写历史数字。

| 报告 | 内容 |
|---|---|
| [`NATIVE_STAGE_TIMING_REPORT_20260820_ZH.md`](NATIVE_STAGE_TIMING_REPORT_20260820_ZH.md) | ROS1/ROS2 原生流程的分阶段耗时定义与瓶颈 |
| [`NATIVE_VALIDATION_20260820.md`](NATIVE_VALIDATION_20260820.md) | 原生并行实现的构建、数值和模型验证 |
| [`V2_REFACTOR_VALIDATION_20260821_ZH.md`](V2_REFACTOR_VALIDATION_20260821_ZH.md) | v2 目录重构、安装及算法一致性 |
| [`PROJECT_REFERENCE_PERFORMANCE_REPORT_20260821_ZH.md`](PROJECT_REFERENCE_PERFORMANCE_REPORT_20260821_ZH.md) | Project/Reference 公平线程预算性能对比 |

后续性能实验应使用
[`../BENCHMARK_ZH.md`](../BENCHMARK_ZH.md) 规定的冻结注册表和 Candidate 归档流程，
不要直接覆盖本目录中的报告或其对应数据目录。
