# directory_dataset 与 EuRoC 实测记录（2026-09-09）

本次按指定数据集分别完成双目相机标定，再引用本次相机结果完成 Camera–IMU
标定。两组 IMU 最终均使用 `calibrated`。本次调整任务、数据清单及文档，没有修改
原生求解算法或默认超参数；这是指定配置的运行验证，没有重新执行冻结 benchmark。

## 1. 数据与配置

输入根目录为 `Q:\File\machine_data\evt3.1-data`，Linux 对应
`/mnt/q/File/machine_data/evt3.1-data`。

| 数据 | 相机模型 | 输入 | 配置 |
|---|---|---|---|
| directory_dataset | pinhole-equi | 每目 301 张 1920×1080 图像；3221 个 IMU 样本 | 本地 config/directory_dataset/，批次 verification/ 留有快照 |
| euroc_cam 双目 | pinhole-radtan | stoero_camara.bag，cam0/cam1 各 1450/1449 张 752×480 图像 | 本地 config/euroc/，批次 verification/ 留有快照 |
| euroc_cam Camera–IMU | pinhole-radtan | camara_imu.bag，原始每目 1439 张图像、14381 个 IMU 样本 | 同上 |

保留 EuRoC 两个 bag 的实际拼写。任务文件统一为
`stereo_camera_calibration_task.yaml` 和 `camera_imu_calibration_task.yaml`，不带注释。
AprilGrid 和 IMU 噪声配置放在对应配置目录，沿用已有参数，没有估计新的噪声参数。
检测进程数和优化线程数均为 4，相机 `shuffle: false`；双目同步容差分别为
0.0002 s 和 0.02 s。

directory_dataset 的 `dataset.yaml` 补齐 v1.0.0 版本、数据集及传感器 ID、frame ID、
图像分辨率和 IMU 单位。原清单保存在数据目录的 `dataset.schema-1.yaml.bak`，
程序断言原有流路径和字段保持一致；图像、CSV、时间戳和采集元数据未改写。

EuRoC Camera–IMU bag 的首个 IMU 比首张图像晚 15 ms。任务使用
`dataset.time_range_s: [0.02, 72.0]`，按既有规则相对各话题自己的起点裁剪。
实际保留每目 1438 张图像、14377 个 IMU 样本；首张保留图像位于相机原起点后
50 ms，首个保留 IMU 位于该起点后 35 ms，满足时间覆盖要求。原始 bag 未改写。
双目标定使用完整的 stoero_camara.bag，不应用此裁剪。

## 2. 输出和串联关系

输出根目录为 `Q:\File\machine_data\evt3.1-data\output`，本批次目录为：

```text
directory_dataset/260909_2018/
  stereo/
  camera_imu/
  verification/
euroc_cam/260909_2018/
  stereo/
  camera_imu/
  camera_imu_scale_misalignment_superseded/
  verification/
```

各 Camera–IMU task 的 `camera_calibration.path` 均在对应双目标定成功后写入，
指向本批次 `stereo/calibration.yaml`。核对运行清单中的引用路径和 SHA-256 一致；
Camera–IMU 结果中的相机内参、畸变、分辨率和相机间变换保持相机结果，逐元素比较
`atol=1e-12, rtol=0` 通过。

EuRoC 最初的 `scale-misalignment` 任务在切换模型时已经完成，结果和日志保留在
`camera_imu_scale_misalignment_superseded/`。随后仅重跑 `calibrated` Camera–IMU，
复用同一份双目结果；下文所有 EuRoC Camera–IMU 数值均来自新的 `camera_imu/`。

每次运行生成 calibration.yaml、results.txt、metrics.json、assessment.json、
HTML/PDF 报告、输入校验、任务快照、运行清单、stdout/stderr、OpenCV 参数及带
校验和的观测归档。每次另有 60 张角点图和 30 张双目校正图，指标使用全部最终
有效角点。已经抽看 directory_dataset 的角点图和 EuRoC 的双目校正图。

## 3. 数值结果

RMS 为最终有效角点的二维重投影距离 RMS，单位 px，分母为角点数。

| 数据／任务 | cam0/cam1 有效帧 | cam0/cam1 有效角点 | cam0 RMS | cam1 RMS | 参考评级 |
|---|---|---|---:|---:|---|
| directory_dataset 双目 | 47 / 46 | 8134 / 8357 | 1.425099 | 1.700557 | poor |
| directory_dataset Camera–IMU | 65 / 60 | 9302 / 9391 | 1.575793 | 1.920959 | poor |
| EuRoC 双目 | 143 / 143 | 18237 / 19263 | 0.244191 | 0.291162 | good |
| EuRoC Camera–IMU，calibrated | 1398 / 1398 | 174161 / 173094 | 0.493612 | 0.523419 | acceptable |

两组双目标定的基线长度分别为 0.5114411059688072 m、0.10986836245902386 m；
校正后极线方向差平均绝对值分别为 0.8982892482583718 px、0.10159422153600135 px，
RMS 分别为 1.1818370134558793 px、0.13530312184289656 px。
Camera–IMU 的图像配对是后处理诊断配对，不等同于原生双目增量 view。

| Camera–IMU 数据 | gyro / accel 各自残差数 | gyro 向量 RMS，rad/s | accel 向量 RMS，m/s² |
|---|---:|---:|---:|
| directory_dataset | 2966 | 0.00219152069910466 | 0.1061833189531341 |
| EuRoC，calibrated | 14377 | 0.009287688551589336 | 0.06100461390656106 |

EuRoC calibrated 联合优化返回 5 次迭代、无失败迭代，最终目标值为
92792.5494737653。原生返回值未说明具体停止条件，报告保持 `stop_reason: unavailable`，
不能将任务执行完成直接解释为满足某个收敛条件。

所有任务未配置业务验收规则，正式判定均为 `not_evaluated`。参考评级仅覆盖相机
重投影和双目校正误差，不能替代 Camera–IMU 或设备生产验收。

## 4. 验证与运行记录

四份 task 均通过加载检查，四次运行均通过严格输入预检并以 `completed` 结束。
核验通过观测表校验和、整数纳秒来源时间戳、两类相机模型及四个畸变参数、结果
版本、必需输出文件，以及 OpenCV R/T 与结果变换的方向和数值一致性
（`atol=1e-15, rtol=0`）。四次输出诊断列表均为空。

| 运行 | 全流程墙钟时间，s | GNU time 最大 RSS，KiB |
|---|---:|---:|
| directory_dataset 双目 | 129.25 | 613976 |
| directory_dataset Camera–IMU | 124.72 | 530600 |
| EuRoC 双目 | 1041.60 | 1833896 |
| EuRoC calibrated Camera–IMU | 262.00 | 1546716 |

以上为 release 构建下含预检、检测、求解和报告的单次运行记录；RSS 不是进程树
聚合 PSS，也不构成性能提升对照。

本次重新 configure 并以最多 4 jobs 构建 project-release，安装到临时前缀后验证
CLI 版本为 1.0.0，移动后的示例及 calibrated 配置正确安装。`git diff --check`
和源码差异审计通过（1564 未变、33 项已批准修改、32 项省略 Reference 文件）。
本轮是配置、文档和配置安装位置调整，未重新执行完整 CTest；升级阶段的完整测试
证据见此前的 [v1.0.0 升级验证](V1_0_0_UPGRADE_VALIDATION_20260909_ZH.md)。

每个批次的 `verification/` 保存数值汇总、输入哈希、清单适配／时间覆盖记录、
核验脚本与日志、配置快照及本报告。当前工作区原有未提交修改与删除均保留，
本次没有提交、推送或打 tag。
