# Kalibr no-ROS Repository Instructions

本文件作用于整个仓库。执行前先阅读本文件、README.md、docs/README_ZH.md 与目标模块。
更深层 AGENTS.md 对其目录补充约束。现有修改属于用户，开始前运行 `git status --short`。

## 1. 发布工程边界

工程只维护输入、原生标定求解、正式输出以及可选运行诊断。当前源码位于
`src/kalibr`；不重新引入冻结源码副本、历史基线、性能比较工具或历史验证报告。
保留根 LICENSE 与第三方源码版权头，安装时携带许可证。

| 路径 | 职责 |
|---|---|
| `src/kalibr` | 原生基础库、相机、轨迹、优化器与标定实现 |
| `src/python/kalibr_no_ros` | CLI、输入契约、初始化、输出与评价 |
| `src/python/kalibr_bag_io` | 无 ROS bag/目录读取 |
| `src/python/kalibr_runtime` | 运行线程预算与可选 profiling |
| `src/camera_models` | radtan5/radtan8 和九参数 OpenCV fisheye |
| `config/examples` | 可移植的简洁模板与完整注释示例 |
| `docs` | 当前使用、参数、模型和源码说明 |
| `tests` | 精简的功能、模型数学与接口正确性检查 |
| `tools` | 依赖准备与启动入口 |
| `build`、`install`、`.deps` | 构建产物与私有依赖，不提交 |

搜索优先使用 `rg`；编辑源码使用 `apply_patch`，不要手工修改构建目录里的复制文件。
不要将工作区缓存、数据集、标定输出或机器专用配置写入可移植示例。

## 2. 构建和检查

只保留 `release` 与 `project-profile`。所有编译和检查最多 4 个并行 job；
不得使用裸 `-j`、`-j8`、`nproc` 或 `CPU-1` 编译。

```bash
cmake --preset release
cmake --build --preset release --parallel 4
cmake --preset project-profile
cmake --build --preset project-profile --parallel 4
cmake --build --preset project-profile --target check --parallel 4
```

`release` 关闭性能采样与诊断计时，不编译或安装测试。`project-profile` 允许显式
请求阶段计时和进程树内存采样；`check` 是显式测试目标，不属于默认构建或安装。
不要新建独立测试/参考构建版本。

Python 包在 configure 阶段复制。修改 Python 后必须重新运行对应
`cmake --preset ...`；不能仅凭 `ninja: no work to do` 判断源码已同步。
运行期 detector/optimizer 线程数遵循 task 与 CLI，不受编译 job 限制自动覆盖。

普通发布路径不得无条件采样性能、写诊断图像或打印大量调试日志。正式观测归档、
指标与报告由 `output` 独立控制，不依赖 profile；详细归档和原图可视化默认关闭。

## 3. 原生算法不变量

未得到明确算法修改要求时，保持阶段顺序、残差、活动参数、优化器、超参数、停止
条件和异常点逻辑。不引入 Ceres。I/O、模型、初始化与诊断适配不得静默改变数学问题。

相机标定顺序：各相机独立检测 → 单相机内参/畸变 LM → 按时间容差建 view 与共视图
→ PnP 相对位姿中值与成对 stereo LM → 全批量联合 refinement → 原生增量接受、
信息增益与异常点过滤。同步还必须满足共同全局角点 ID，不能只检查时间戳。

`calibration.freeze_intrinsics` 默认 `false`。仅至少两台相机、显式启用且每台相机有
完整内参与畸变 seed 时，才能跳过单目内参 LM，并在所有后续阶段固定 projection/
distortion；baseline 与 target pose 仍 active。

Camera–IMU 保持时间偏移互相关、旋转/gyro bias 初值、连续时间 spline、建图和最终
联合 LM。`recompute_camera_chain_extrinsics` 仅控制联合阶段 baseline 是否 active，
不意味着重做相机内参标定。初始化 `direct`/`refine` 语义见
[初始化说明](docs/INITIALIZATION_ZH.md)；seed 不应增加隐式先验或掩盖秩亏。

统一坐标方向：

```text
p_target = T_target_source * p_source
```

`T_cam_imu` 为 IMU 到相机，`T_cn_cnm1` 为前一相机到当前相机。文档统一
使用 ${}^{A}_{B}\mathbf T$。修改变换读写时检查组合、求逆和原点，禁止只凭变量名猜方向。

## 4. 输入与模型契约

软件、CLI、task、初始化、目录清单及结果版本统一为字符串 `"1.0.0"`。
旧整数 schema 不能作为项目运行输入；第三方 ROS/OpenCV 编码独立于本项目版本。

task 必须显式指定 `dataset.type: bag|directory` 和 `dataset.path`。
相对路径以 task 所在目录为基准。目录数据集用 `dataset.yaml` 将 ID 映射到图像、
时间戳 CSV 和 IMU CSV；相机 topic/IMU rostopic 可省略，bag 输入仍必须有话题。
图像至少支持 PNG/JPEG/JPG/BMP，时间戳保留整数纳秒；新数据目录后缀为
`_YYMMDD_hhmm`。相机与 Camera–IMU 是分开的 job；后者的
`camera_calibration.path` 必须读取上一阶段新版相机结果。

保留原生模型和 radtan5/radtan8。radtan8 的顺序固定为
`[k1,k2,p1,p2,k3,k4,k5,k6]`，不得截断。鱼眼扩展只保留九参数
`pinhole-opencv-fisheye`，投影为 `[fu,fv,cu,cv,alpha]`、畸变为四系数，
`K[0,1]=fu*alpha`，唯一扩展包名 `kalibr_opencv_fisheye`。八参数等距鱼眼使用
原生 `pinhole-equi`。

模型改动至少检查投影/反投影、解析 Jacobian、维数顺序、YAML/OpenCV 往返及
Boost.Python pickle，不得仅凭编译成功判定数学正确。

AprilGrid 固定 tag36h11，支持连续非零 tagStartId；满足
`0 <= tagStartId` 且 `tagStartId + tagRows * tagCols <= 587`。
亚像素参数只在 calibration 暴露：
`window_half_size_px=2`、`max_displacement_px=sqrt(1.5)`，省略必须保留原生值；
需同时贯通两类标定、绑定和多进程 pickle。
`focal_initialization_min_visible_corner_ratio` 合法范围 `(0,1]`，默认 1 直接走
原生完整帧路径；放宽候选帧不能绕过行覆盖和拟合稳定性检查。Camera–IMU 不执行它。

新增配置必须注明默认值、单位、合法范围及内部作用；省略保持原有行为。
示例根目录与 initialization YAML 不写注释，详细注释仅放 `all_params/`。

## 5. 输出契约

结果为 `schema_version: "1.0.0"`、`kind: calibration_result`。
YAML 映射使用块状与稳定顺序；一维数组行内，矩阵外层块状、每行行内；双精度
17 位有效数字，整数浮点写为 `0.0`；禁止默认 80 列折断数组。
任何已有结果的格式迁移必须先读回验证语义相等，再原子替换。

正式交付为按任务与传感器 ID 命名的结果 YAML 和 HTML/PDF 报告；每目含 rms，
相邻外参含 alignment，方向按相机列表与 T_cn_cnm1 表达，不输出 from_camera。默认不交付诊断，output.save_diagnostics、
save_metrics、export_text 分别控制可选文件。结果可共用输出根目录，但不得覆盖其他任务。
输入验证、初值、可观性、poses 和 profile timing 按运行情况及保存配置生成。
详细观测、筛选历史、原图与 OpenCV
导出独立配置。`evaluate` 只用已有证据，不重新检测或优化；无法评价不得写成 0
或判定通过。参考等级与用户显式生产判定规则保持分离。

覆盖受管输出前，必须检查清单内全部文件，禁止删除未登记子文件、符号链接指向的
外部数据或本次输入结果。配置/报告失败应保留可用诊断与明确运行状态。

## 6. 并行与运行诊断

建议显式预算为 detector_processes=4、optimizer_threads=4，
detector_inflight_per_worker=2、detector_opencv_threads=1。
不指定线程预算时保留原生阶段默认。并行不应改变数据选择、顺序或算法参数；
浮点归约差异需通过相关数学测试核验。

优化器预算只覆盖已接入 Optimizer2、增量估计器与 Hessian 路径，不能宣称整个流程
均使用此线程数。profile 内存采样周期默认 0.25 s；RSS 与 PSS 含义不同，不能混算。
报告、时间互相关、部分 spline 初始化与 Python 调度仍主要在主进程。

## 7. 数据保护、Git 与验收

实际数据与外部输出默认只读，只写用户指定的新输出目录。删除外部数据、历史结果
或迁移目录前确认精确目标和可恢复性；用户已明确授权的目标不重复索取授权。
不要重跑能够通过已有结果回答的真实标定，也不改采集硬件配置。

现有修改必须保留，不用 `git reset --hard` 或会覆盖用户工作的 checkout。
仅用户明确要求时提交、推送或 tag；暂存使用精确路径，提交前列出暂存文件并运行
`git diff --cached --check`。config 下机器专用路径与本地 README 默认不提交。
tag 使用附注 tag，先检查本地/远端同名项，禁止覆盖或 force-push。

按风险执行目标检查、`git diff --check`、重新 configure/build 和显式 `check`。
仅算法、模型或数据选择变化且用户授权时运行真实数据；不要硬编码历史测试数量，
验收关注失败与 skip 原因。文档用中文，保留标识原文，链接和 Markdown 格式有效。

完成时报告功能变化、算法/默认数值行为是否改变、构建与测试结果、真实数据验证
是否执行及原因、保留的未提交用户修改；若已提交/推送，给出分支和完整 commit。
