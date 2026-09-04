# Kalibr no-ROS Repository Instructions

本文件作用于整个仓库。执行任务前先阅读本文件，再阅读与任务直接相关的
`README.md`、`docs/README_ZH.md` 及目标模块源码。若更深层目录以后增加自己的
`AGENTS.md`，则以更深层文件对该目录的补充约束为准。

## 1. 项目目标

本项目把冻结的 ETHZ-ASL Kalibr 改造成不依赖 ROS 运行时的现代工程，同时保留原生
标定算法，并增加 OpenCV 相机模型兼容、显式初始化、目录数据集、可控并行和性能
分析能力。

默认行为必须满足以下原则：

1. 相机标定和 Camera–IMU 标定的阶段顺序、残差、活动参数、优化器、超参数、停止
   条件和异常点逻辑与冻结 ETHZ 实现一致。
2. 新能力应作为 I/O、模型适配、初始化、诊断或并行执行扩展；除非任务明确要求
   改算法，否则不得改变原生数学问题。
3. 新增超参数必须有明确物理单位、合法范围和文档；省略时必须恢复原生默认值。
4. 并行化不得静默改变数据选择、观测顺序或算法超参数。并行浮点归约造成的末位差异
   必须通过数值对比报告说明。
5. 本项目使用原生增量 LM/GN/Optimizer2 路径，不引入 Ceres，也不复用其他分支遗留
   的 Ceres 构建或安装产物。

## 2. 不可修改的 Reference

- `ref/kalibr` 是 ETHZ Kalibr commit
  `1f60227442d25e36365ef5f72cd80b9666d73467` 的 1630 文件冻结快照。
- Reference 只用于验证、构建和数值比较，禁止直接编辑、格式化、补丁覆盖或把生产
  功能写入其中。
- 生产改动写入 `src/kalibr`、`src/python` 或 `src/camera_models`。
- 修改从 Reference 迁移出的 Kalibr 源文件时，检查并按真实原因维护
  `tools/audit_source_delta.py` 的允许列表；不得用宽泛规则绕过审计。
- Reference 验证命令：

  ```bash
  build/project-release/bin/kalibr-noros reference verify
  python3 tools/audit_source_delta.py --root /home/gs/kalibr_no_ros
  ```

## 3. 目录职责

| 路径 | 职责 |
|---|---|
| `ref/kalibr` | 只读 ETHZ 快照和 Reference 构建输入 |
| `src/kalibr` | 可修改的原生 Kalibr C++/Python 生产源码 |
| `src/python` | `kalibr-noros` CLI、task、无 ROS bag/目录 I/O、benchmark 和诊断 |
| `src/camera_models` | radtan5、radtan8、OpenCV fisheye 等相机模型扩展 |
| `cmake`、`CMakeLists.txt` | 工程构建、Reference/Project 选择和安装规则 |
| `config` | 简洁模板与可运行示例；不要写入临时输出或提交机器专用数据路径 |
| `docs` | 中文使用、参数、初始化、源码和历史验证文档 |
| `benchmarks` | 冻结基线注册表；不是临时运行目录 |
| `tests` | Python、C++、模型、序列化、数值和 CLI 契约测试 |
| `tools` | 依赖、审计、比较和维护工具 |
| `build`、`install`、`.deps` | 生成物或私有依赖；禁止手工修改或提交 |

搜索文件和文本优先使用 `rg`、`rg --files`。编辑源码使用 `apply_patch`，不要直接编辑
`build/` 中的复制文件。

## 4. 构建与内存限制

所有编译和测试最多使用 4 个并行 job。不得使用裸 `-j`、`-j8`、`nproc` 或
`CPU-1` 编译；这台机器可能因并发编译耗尽内存。

生产构建：

```bash
cmake --preset project-release
cmake --build --preset project-release --parallel 4
```

测试构建：

```bash
cmake --preset project-test
cmake --build --preset project-test --parallel 4
ctest --preset project-test -j4
```

性能构建：

```bash
cmake --preset project-profile
cmake --build --preset project-profile --parallel 4
```

Reference 构建只在明确需要比较时执行：

```bash
cmake --preset reference-release
cmake --build --preset reference-release --parallel 4
```

注意：部分 Python package 通过 CMake configure 阶段复制到 build tree。修改
`src/python` 或迁移的 Kalibr Python 后，要先重新运行对应的 `cmake --preset ...`，
不能仅凭 `ninja: no work to do` 判断 build tree 已同步。

编译 4 jobs 的限制不等同于标定运行时线程数。标定的 detector/optimizer 并行数必须
遵循 task、CLI 或 benchmark 契约，不得为了提速擅自改变。

## 5. 构建预设边界

| Preset | 用途 |
|---|---|
| `project-release` | 生产版本；测试、profiling 和诊断 I/O 默认关闭 |
| `project-test` | Project 源码及完整 C++/Python 测试 |
| `project-profile` | 阶段计时、进程树内存和诊断 I/O |
| `reference-release` | 冻结 ETHZ 算法经过相同外层 no-ROS I/O 运行 |
| `reference-test` | Reference 可适用测试 |

普通发布路径禁止无条件打印大量日志、写中间图像或采集性能数据。新增计时、内存、
调试打印和诊断文件应接入既有 profiling/diagnostic 编译开关，默认关闭。

## 6. 标定算法不变量

### 6.1 双目相机标定

默认阶段不得重排：

1. 同一 bag 按相机 topic 分别读取，独立检测标定板。
2. 每台相机分别解析初始化并执行单相机内参/畸变 LM。
3. 按时间容差构建 target view 和相机共视图。
4. 通过 PnP 相对位姿中值和成对 stereo LM 初始化 baseline。
5. 全批量联合 refinement 同时优化内参、畸变、baseline 和 target pose。
6. 最终按原生增量流程加入 view、计算信息增益并执行异常点过滤。

`shuffle: false` 是可重复 benchmark 的必要条件。同步成功只表示两路有效观测的时间差
满足 `synchronization_tolerance_s`；相机连接图还要求共同全局角点 ID。

### 6.2 Camera–IMU 标定

保持原生时间偏移互相关、旋转/gyro bias 初值、连续时间 spline、残差建图和最终联合
LM 顺序。`recompute_camera_chain_extrinsics` 仅控制联合阶段是否放开相机链 baseline，
不要误解为 Camera–IMU 默认会重做完整相机标定。

### 6.3 坐标变换约定

统一结果声明：

```text
p_target = T_target_source * p_source
```

例如：

$$
{}^{C}\mathbf p
=
{}^{C}_{I}\mathbf T\,{}^{I}\mathbf p.
$$

- `T_cam_imu` 是 ${}^{C}_{I}\mathbf T$，从 IMU 坐标变换到相机坐标。
- `T_cn_cnm1` 是 ${}^{C_n}_{C_{n-1}}\mathbf T$。
- 不得只凭变量名猜方向；修改读写或转换代码时必须做组合、求逆和原点测试。

## 7. 数据、模型与 AprilGrid

支持的输入包括 ROS1 bag、ROS2 bag 目录和带 `dataset.yaml` 的目录数据集。目录图像由
OpenCV 解码，至少覆盖 PNG、JPEG/JPG 和 BMP；时间戳保持纳秒精度。

task 中必须显式写：

```yaml
dataset:
  type: bag       # 或 directory
  path: ...
```

相对路径以 task YAML 所在目录为基准。不要依赖当前工作目录偶然解析成功。

当前常用模型包括：

- `pinhole-radtan`：`[k1,k2,p1,p2]`；
- `pinhole-radtan5`：`[k1,k2,p1,p2,k3]`；
- `pinhole-radtan8`：OpenCV rational `[k1,k2,p1,p2,k3,k4,k5,k6]`；
- 原生 equidistant/FOV/omni/EUCM/DoubleSphere 组合；
- `pinhole-opencv-fisheye` 及完整 alpha/skew 转换扩展。

`pinhole-radtan8` 必须始终保持 OpenCV rational 的 8 参数顺序，不得截断为 radtan5。
新增模型必须包含投影/反投影、解析 Jacobian、design variable、误差项、绑定、配置
读写、OpenCV 转换和数值测试。

AprilGrid 当前固定 tag36h11，支持连续的非零 `tagStartId`，但不支持离散 ID 表或其他
tag family。必须验证：

$$
0\leq\text{tagStartId},\qquad
\text{tagStartId}+\text{tagRows}\cdot\text{tagCols}\leq587.
$$

AprilGrid 亚像素细化只在 `calibration` 层暴露两个扁平字段：

```yaml
calibration:
  window_half_size_px: 2
  max_displacement_px: 1.224744871391589
```

- `window_half_size_px=w` 对应 OpenCV 实际搜索区域 $(2w+1)\times(2w+1)$；
- `max_displacement_px` 在内部平方后对应原生 `maxSubpixDisplacement2`；
- 两项省略时必须严格保留原生默认 `2` 和 `sqrt(1.5)`；
- 新字段必须同时贯通 camera、Camera–IMU、Python 绑定及多进程 pickle。

Pinhole 系列无内参 seed 时可显式放宽焦距初始化的完整帧要求：

```yaml
calibration:
  focal_initialization_min_visible_corner_ratio: 0.75
```

- 比例定义为单帧有效角点数除以目标理论总角点数，合法范围为 $(0,1]$；
- 省略或设为 `1.0` 时必须直接走冻结 ETHZ 的完整帧实现；
- 小于 `1.0` 时，全局比例只负责候选筛选，行内覆盖、拟合条件和圆交点稳定性仍是不可
  绕过的内部检查；
- Camera–IMU 从 camchain 读取内参，不执行该焦距初始化，因此不使用此字段。

## 8. Task、初始化和结果契约

- camera task 与 Camera–IMU task 的 `schema_version` 当前固定为整数 `1`。
- 两类 task 必须分开，不能把 `camera_calibration` 和
  `camera_imu_calibration` 混成一个 job。
- 初始化文件也使用各自接口的 schema v1；`direct` 和 `refine` 的语义及完整性要求见
  `docs/INITIALIZATION_ZH.md`。
- 程序输出 `calibration.yaml` 使用独立的结果 `schema_version: 2`。不要因为 task 是
  v1 就把结果版本改成 1；这会破坏已有结果、benchmark 和下游读取。
- Camera–IMU 的 `camera_calibration.path` 同时接受原生 Kalibr camchain 和项目结果
  schema v2。

结果 YAML 排版是稳定接口的一部分：

- 映射保持块状格式和既定字段顺序；
- 一维标量数组使用行内形式，例如 `intrinsics: [fu, fv, cu, cv]`；
- 矩阵外层保持块状、每行使用行内数组；
- 双精度按 17 位有效数字输出；整数值浮点写成 `0.0`，不写成
  `!!float '0'`；
- 禁止默认 80 列把 `distortion_coeffs` 拆行；
- 任何排版迁移必须先将新旧 YAML 读回并断言语义完全相等，再原子替换旧文件。

默认管理输出为：

```text
calibration.yaml
results.txt
report.pdf
initialization_report.yaml   # 使用初值或产生相应诊断时
observability.yaml           # 运行可观性分析时
poses.csv                    # 显式请求时
timing.json                  # profile 构建并显式请求时
```

## 9. 并行与数值一致性

标准执行配置为：

```yaml
execution:
  detector_processes: 4
  optimizer_threads: 4
  detector_inflight_per_worker: 2
  detector_opencv_threads: 1
  profiling_memory_sample_interval_s: 0.25
```

- detector 使用多进程；每个 worker 默认只允许一个 OpenCV 线程，防止进程数乘线程数
  造成超配。
- optimizer 线程数只覆盖已接入的 Optimizer2、增量估计器和并行 Hessian 路径，不能
  宣称整个流程都以该线程数运行。
- 时间偏移互相关、部分 spline 初始化、Python 调度和报告输出主要仍在主进程。
- 默认未指定并行时的 Reference 保留 ETHZ 各阶段 2/4/CPU-1 混合行为；公平测试必须
  为 Project 与 Reference 显式传入相同线程预算。
- 不能用迭代次数单独判断哪一端更收敛；比较最终目标、停止原因、参数变化和残差。

## 10. Benchmark 和外部数据保护

`benchmarks/baselines-v1.yaml` 冻结三项 Project 性能基线：

- `euroc_camera_project_4`；
- `euroc_imu_project_4`；
- `evt_imu_project_4`。

另有 `reference_default_group` 作为数值参考。这些历史结果、日志、报告、task、哈希和
输入 manifest 禁止重跑、覆盖、改写或删除。新优化版本只作为 Candidate 与其比较。

公平比较必须保持：

- 同一数据集、topic、target、相机/IMU 模型和算法参数；
- `detector_processes=4`、`optimizer_threads=4`；
- 相机标定 `shuffle=false`；
- 数值比较默认 `atol=1e-8`、`rtol=1e-8`；
- 性能分析使用 `project-profile`，保留 stdout/stderr、`timing.json`、manifest、结果和
  报告。

除非用户明确要求，不要重新执行能由已有结果回答的问题。`/mnt/q/File/kalibr/data`
下的 bag、目录数据集和冻结输出默认只读；只允许写入用户明确指定的新输出目录。
删除外部数据、历史结果或 benchmark 前必须再次确认精确目标和可恢复性。

内存报告区分 RSS 与 PSS：旧 GNU time 最大 RSS、detector 进程树峰值和新全流程进程树
聚合 PSS 不是同一口径，不得直接伪造百分比比较。

## 11. 测试与验收

按风险由小到大验证：

1. 运行目标模块的单元测试或最小数值测试。
2. `git diff --check`。
3. `python3 tools/audit_source_delta.py --root /home/gs/kalibr_no_ros`。
4. 重新 configure、最多 4 jobs 编译 `project-test`。
5. 运行 `ctest --preset project-test -j4`。
6. 影响生产路径时重新 configure、最多 4 jobs 编译 `project-release`。
7. 只有算法、数据选择或模型发生变化时，才运行用户授权的真实数据 Candidate 验证。

当前完整 Project 测试基线是 26 项通过、0 项失败，另有 1 项长期 Disabled。测试数量
可以随新增测试增加，因此验收应同时检查失败数和 disabled 原因，不能只硬编码数量。

模型扩展至少验证：

- 投影/反投影往返；
- 解析 Jacobian 对有限差分；
- 参数维数与顺序；
- YAML/OpenCV 往返；
- Boost/Python pickle（若 detector worker 会跨进程传递）；
- 默认配置与冻结原生结果的一致性。

初始化或可观性改动至少验证 direct、refine、坏初值、缺字段、维数错误、hard rank 和
operational rank。不能用初值掩盖真实秩亏而宣称数据已经充分可观。

## 12. 文档和公式

- 面向用户的工程文档优先使用中文；代码标识、CLI 名称和 YAML 字段保持源码拼写。
- Markdown 标题、表格、列表前后保留空行，确保渲染正确。
- 坐标记号统一使用 ${}^{A}_{B}\mathbf T$，并明确点变换方向。
- 参数文档必须说明默认值、单位、合法范围、内部算法作用和是否破坏 benchmark
  可比性。
- 历史实测结论放入 `docs/reports`；日常使用和当前契约放入主指南，避免重复维护多份
  相互矛盾的文档。

## 13. 工作区与 Git 规则

- 开始前运行 `git status --short`。现有修改属于用户，必须保留；不要顺手格式化或
  暂存无关文件。
- 不直接修改、提交或删除 `build`、`install`、`.deps`、数据集和标定输出。
- 不使用 `git reset --hard`、`git checkout --` 或其他会丢失用户改动的命令。
- 只在用户明确要求时提交、推送或打 tag。提交前显式列出暂存文件，运行测试和
  `git diff --cached --check`。
- 使用精确路径 `git add`，不要用 `git add -A` 将本地配置、数据或输出误提交。
- `config` 下机器专用路径和本地 README 改动默认不提交，除非用户明确要求纳入。
- tag 使用附注 tag，创建前检查本地和远端同名 tag；禁止覆盖或 force-push 已发布
  tag。

## 14. 完成任务时的报告

最终说明应包含：

- 实际修改的功能和文件；
- 是否改变算法或默认数值行为；
- 执行过的构建、测试及其结果；
- 未执行的真实数据验证及原因；
- 仍保留的未提交用户改动；
- 若已提交/推送，给出分支、完整 commit 和 tag。

不要把“编译成功”“单元测试通过”“真实数据数值一致”“性能提升”混成一个结论；四者
必须分别给出证据。
