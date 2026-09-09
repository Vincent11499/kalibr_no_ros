# v1.0.0 升级验证记录（2026-09-09）

## 1. 实现与对照范围

本次同时升级标定工程和 `/mnt/q/project/kalibr_cedarobo` 采集工程。软件与项目配置／
结构化输出统一为字符串 `1.0.0`，采集目录使用 `<前缀>_YYMMDD_hhmm`；新项目输入
拒绝旧整数 schema，不原地迁移历史数据。使用入口见
[统一契约](../V1_INTERFACE_ZH.md) 和 [完整／最简示例](../../config/examples/v1.0.0/README_ZH.md)。

标定工程主要新增 `validation.py`、`artifacts.py`、`evaluation.py`、`reporting.py`、
`conversion.py`，分别承担输入验证、原生状态快照、指标／判定、文件／图像报告及
公共格式转换。原生求解只增加观测来源、最终状态和选择历史的读取钩子；阶段顺序、
残差、活动参数、优化器及默认超参数不变。逐角点归档和图像诊断默认关闭。

升级前 Project 对照来自提交 `9b7b3ce2a8cafca4d17aec86cb23876a07260ae2` 的独立
`git archive` 构建。Candidate 是 `docs/kalibr-native-v2.1` 分支上的未提交工作区。
本次没有提交、推送、打 tag 或修改冻结 benchmark；该对照不是重新运行历史基线。

## 2. 输入与执行条件

原始数据为 `/mnt/q/File/machine_data/evt3.1-data/2026-09-09_15-17-18`：
每目 301 张 1920×1080 图像，IMU 3221 个样本。只在当前工作区创建验证副本，
新／旧接口各用自己的 manifest。605 个复制后的原始图像与 CSV 文件逐个 SHA-256
核对一致，原始外部数据未改写。

三类任务均使用同一输入、板、模型及参数：

- 单目：cam0，`pinhole-equi`；双目：cam0/cam1，均为 `pinhole-equi`。
- 双目＋IMU：读取对应双目输出，IMU 使用 `calibrated`，未启用显式 seed。
- AprilGrid：8×8，tagSize 0.065 m，tagSpacing 0.3，tagStartId 80。
- 检测 4 进程、优化 4 线程、每 worker 两个在途任务、一个 OpenCV 线程。
- 相机 `shuffle=false`、同步容差 0.0002 s；其他未指定求解参数保持原生默认。
- 每类 Candidate 各执行一次归档关闭和一次归档开启。开启时同时保存选择历史；
  两组均关闭图像可视化，保持 OpenCV 导出开启。

最终严格输入校验针对六份真实 task 全部通过。最后补充的输入字段校验不改变求解
或数据选择，因此只重新验证输入，没有重复六次已完成的数值运行。

## 3. 数值结果

比较完整 `calibration.yaml` 的相机、畸变、外参、时间偏移、IMU 参数及其他语义字段，
仅在离线比较时排除 schema 版本标签，容差 `atol=rtol=1e-8`。

| 任务 | 升级前／Candidate 最大绝对差 | 最大相对差 | Candidate 归档开／关 |
|---|---:|---:|---|
| 单目 | 0 | 0 | calibration.yaml 逐字节相同，metrics.json 语义完全相同 |
| 双目 | 0 | 0 | calibration.yaml 逐字节相同，metrics.json 语义完全相同 |
| 双目＋IMU | 0 | 0 | calibration.yaml 逐字节相同，metrics.json 语义完全相同 |

以上是这份数据及这些明确配置下的数值一致证据，不代表所有数据集均已验证。

新指标以最终有效角点计算二维点距离 RMS，单位 px，分母为角点数。

| 任务／相机 | 最终使用帧数 | 最终角点数 | 重投影 RMS / px |
|---|---:|---:|---:|
| 单目 cam0 | 27 | 5187 | 1.1192114593311104 |
| 双目 cam0 | 47 | 8134 | 1.4250992880427118 |
| 双目 cam1 | 46 | 8357 | 1.7005569704412444 |
| Camera–IMU cam0 | 65 | 9302 | 1.5757927912009322 |
| Camera–IMU cam1 | 60 | 9391 | 1.920958713749731 |

双目标定最终共 46 对、6497 个可用共同角点，校正后极线方向差的平均绝对值为
0.8982892482583718 px，RMS 为 1.1818370134558793 px。OpenCV 导出的 R/T 与结果中
左目到右目的 `T_cn_cnm1` 逐元素相等，且 R1/R2/P1/P2/Q 均可由 FileStorage 读回。

Camera–IMU 各保留 2966 个 gyro／accel 残差：向量 RMS 分别为
0.00219152069910466 rad/s 和 0.1061833189531341 m/s²。两条优化后的三轴 bias spline
均保存 2966 个采样值，无缺值；白化残差单独使用无量纲统计。
原生联合优化返回 30 次迭代，JFinal 为 64918.793942471224；返回值没有直接提供停止
条件，输出明确标记停止原因不可用，未将“运行完成”推断为“收敛”。

三类任务均未配置业务阈值，正式判定均为 `not_evaluated`；参考评级为 `poor`。
这些结果验证软件升级与数值一致性，不能作为这套设备的标定精度验收结论。

## 4. 归档与离线评价

归档校验包含压缩表哈希、帧 ID 唯一性、整数纳秒时间戳、最终角点残差及 OpenCV
矩阵方向。双目归档保存 602 条输入帧记录、197 条选择／过滤事件；Camera–IMU
归档保存 5932 条物理残差和两条 bias 时序。ICC 后处理图对保持其诊断配对含义，
不冒充原生双目 view。

离线 `evaluate` 未重新检测或求解：

- 双目与 Camera–IMU 各重算一次，指标与在线结果完全相同，calibration.yaml 保持
  逐字节相同；各生成 60 张角点／残差图和 30 张校正图，输出诊断无缺失项。
- 已目视检查实际角点叠加和双目校正图。
- 将工作区数据副本重定位后再次运行双目评价，指标仍完全一致，6 张抽样图成功生成。
- 无观测归档的单目运行可按已保存汇总重评，模式为 `saved_summary_only`。
- 以示例 0.5 px 门槛单独验证规则路径，汇总重评正确返回正式状态 `fail`；此门槛
  仅用于验证判定功能，没有写回原运行或作为生产验收配置。

## 5. 构建与测试

| 检查 | 结果 |
|---|---|
| project-test configure／最多 4 jobs 构建 | 成功 |
| `ctest --preset project-test -j4 --output-on-failure` | 27 项启用测试通过，0 失败 |
| CTest 内 `python_no_ros_tests` | 223 个测试通过，无跳过 |
| project-release configure／最多 4 jobs 构建 | 成功；最终 configure 已同步 Python |
| `kalibr-noros --version` | kalibr-noros 1.0.0 |
| Reference verify | 1630 个冻结文件验证通过 |
| source delta audit | 1564 未变、33 项获准差异、32 项既有省略，审计通过 |
| `git diff --check` | 通过 |
| 采集工程离线 unittest／self-test | 161 个测试通过，0 失败、0 跳过 |

保留的 `aslam_cameras_tests` 是原有 Disabled 测试：它会向源目录写 bin/xml，且包含
受 OpenCV 版本影响的检测断言。未通过临时启用它污染冻结源码。
采集测试覆盖 BMP/JPEG 的真实 DirectoryReader 读回和 ROS1 bag 导出／读回；没有
连接或部署到板端，没有执行新硬件采集或重新估计 IMU Allan 噪声。

## 6. 单次耗时与资源记录

以下为 `/usr/bin/time` 的单次墙钟与最大 RSS，包含本版输入验证、检测、求解和输出。
图像展示均关闭。没有重复统计或受控冷热缓存实验，不据此宣称速度提升。

| 任务 | 归档关 / s | 归档开 / s | GNU max RSS 关 / KiB | GNU max RSS 开 / KiB | 输出大小关 / bytes | 输出大小开 / bytes |
|---|---:|---:|---:|---:|---:|---:|
| 单目 | 29.55 | 28.64 | 408104 | 415352 | 132490 | 692629 |
| 双目 | 78.13 | 77.52 | 589232 | 615520 | 258049 | 1786553 |
| 双目＋IMU | 47.55 | 47.43 | 502504 | 515260 | 2891396 | 5656985 |

GNU max RSS 不是全进程树同时聚合峰值，也不是 PSS。没有运行 project-profile 性能
分析，亦未与冻结 benchmark 的历史 RSS/PSS 口径混算改善百分比。

## 7. 本地证据与工作区状态

完整新输出位于仓库忽略目录 `validation-runs/v1.0.0/`；以下链接只在本工作区有效，
不会把数据集或运行产物提交到仓库：

- [数值／归档比较](../../validation-runs/v1.0.0/numerical-validation.json)、
  [离线重评记录](../../validation-runs/v1.0.0/offline-validation.json)、
  [输入最终检查](../../validation-runs/v1.0.0/input-validation-final.json)、
  [原始文件核对](../../validation-runs/v1.0.0/data-copy-verification.json)。
- [完整 CTest 结果](../../validation-runs/v1.0.0/ctest-final.log)、
  [测试细节](../../validation-runs/v1.0.0/ctest-details.log)、
  [双目可视化报告](../../validation-runs/v1.0.0/evaluation-stereo/report.html)、
  [Camera–IMU 可视化报告](../../validation-runs/v1.0.0/evaluation-imu/report.html)。
- 原生 stdout/stderr、有效 task、输入文档哈希和运行状态保留在各运行目录中。
  新旧对照和失败的准备阶段尝试各有独立日志，没有覆盖成成功记录。

主工程开始时没有未提交修改；采集工程已有用户修改，已在其基础上完成精确文件
更新并保留原内容备份。两工程所有修改保持未提交状态；没有清理用户数据、历史
标定输出或冻结基线。
