# 目录按 ID 读取与两端格式验收（2026-09-10）

## 改动范围

目录相机任务可省略 topic，使用 id/model；目录 IMU 噪声配置可省略 rostopic。
目录 manifest 的相机必需字段为 id/timestamps/images，IMU 为 id/data；话题为
可选元数据，显式填写时仍校验一致性。bag 任务继续要求话题。

改动贯通 DirectoryReader、任务参数生成、Camera–IMU 内部 camchain 适配和离线
图像读取。相机结果按 ID 关联当前目录，生成内部读取键时不改写引用的结果文件。
原生求解代码、默认数学问题、数据选择规则和优化超参数没有修改。

采集工程新清单不再重复写话题，原话题仍保存在 meta/session.json 和
meta/capture_dataset.yaml 供追溯及 bag 导出。两端的任务模板和使用文档已同步。
接口、配置依赖和输出含义见 [数据与输出说明](../DATASET_WORKFLOW_ZH.md)。

## 指定数据的授权迁移

仅在用户已备份并明确授权的目录中执行：
`Q:\File\machine_data\evt3.1-data\directory_dataset`。没有迁移其他历史数据或改写
此前完成的标定输出及冻结基线。

根清单此前已为 v1.0.0，但内部采集 manifest/session 仍为旧 schema 3，不能被新版
采集校验器完整接受。本次统一这些元数据为 v1.0.0，增加 schema/kind、稳定数据集
ID 和时钟来源等必要声明，保留实际原始采集软件版本 1.9.2、UTC 时间、原配置值和
实际采集统计。新增的 dataset_name 仅是迁移后的命名元数据，不改变当前目录名。

- 原始图像 602 张，两目各 301 张，均为 1920×1080 BMP。
- 602 张图像、两份相机 CSV、一份 IMU CSV、一份采集配对表，共 606 个文件的
  SHA-256 在迁移前后逐项相同。
- 3221 条 IMU JSONL 仅增加字符串版本和消息 kind；剔除这两个新增字段后，时间戳、
  六轴、姿态、协方差、frame ID 和到达时刻均与原消息语义完全相同。
- 原始根清单及内部元数据保存为 meta/migration_260910_original.zip；根目录的
  dataset.schema-1.yaml.bak 已归入该归档。迁移记录为 meta/migration_260910.json。
- 采集端无结构错误，状态为 warning；保留“采集时缺少 SDK 头文件检查”和“源相机
  序号缺口 8 帧”两项原有警告。
- Release CLI 对无 topic 的双目和 Camera–IMU task 均预检通过，分别读取 cam0、
  cam1 各 301 帧，Camera–IMU 读取 imu0 的 3221 条样本，时间覆盖满足要求。
- Camera–IMU 内部生成的 camchain 与此前双目结果在所有数值字段上语义相同，
  仅读取键由原话题换为 cam0/cam1；引用的原相机结果没有改写。

采集端结果写入 meta/validation.json；两类标定输入检查保存在
meta/calibration_input_validation.json。原图与测量值不因格式迁移而重新生成。

## 构建与测试

| 检查 | 结果 |
|---|---|
| project-test configure / 最多 4 jobs 构建 | 成功 |
| ctest --preset project-test -j4 --output-on-failure | 27 项启用测试通过，0 失败；aslam_cameras_tests 保持原有 Disabled |
| Python no-ROS 测试 | 228 项通过；新增 5 项覆盖省略话题、ID 冲突、bag 必填、相机结果衔接和原始数据读取一致性 |
| 既有离线重评测试 | 改为无 topic 的替换目录，原归档仍可按 ID 找回准确帧；错误 dataset_id 继续拒绝 |
| project-release configure / 最多 4 jobs 构建 | 成功，Python 已重新同步 |
| 采集工程 unittest / self-test | 161 项通过、0 失败、0 跳过；self-test 为 pass |
| 采集联动 | BMP/JPEG 真实 Kalibr reader 读取、身份/原始 IMU 数值、ROS1 导出与回读均执行 |
| 两个工作区 git diff --check | 通过 |
| 主工程源码差异审计 | 1564 未变、33 项已批准修改、32 项省略 Reference 文件 |

采集导出测试在临时依赖目录使用 rosbags 0.11.0，并复用现有 OpenCV 兼容的 NumPy；
没有升级主工程固定的 rosbags 0.9.x、系统依赖或板端依赖。

本轮只改输入关联与元数据，没有重新执行真实数据的角点检测或优化；之前两阶段的
真实标定结果保留，当前检查证明数据读取、路径解析和参数传递一致，不将其宣称为
一次新的求解数值对照或性能提升。没有执行板端部署、硬件采集、Git 提交、推送或
打 tag。两工程已有的其他未提交修改均保留。
