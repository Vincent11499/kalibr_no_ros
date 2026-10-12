# 滚动快门行时间诊断工具

`tools/kalibr_rolling_shutter_diagnosis.py` 用一个普通相机或普通 Camera–IMU 标定任务检查：
当前动态数据中的较大重投影误差，是否稳定地支持“角点时刻随图像行坐标变化”的解释。
它把 `mechine-00/result/verification` 中一次性的分析方法固化为可复用工具，同时补上严格
分块、列方向负对照、按帧 bootstrap、多初值、正反方向和轨迹带宽敏感性检查。

工具不会把筛查值直接发布为标定结果。自动结论只有三种：

| 结论 | 含义 |
|---|---|
| `important` | 离线筛查及全部正式对照都通过；逐行时间是当前数据中重要且稳定的解释，建议使用生成的 RS task 正式标定并独立验收 |
| `inconclusive` | 数据量、连续运动、求解收敛、可观性、多初值或轨迹带宽等证据不足，不能确认或排除 |
| `unsupported` | 数据充分且为连续运动，但没有显示稳定、可重复且明显强于列方向负对照的行方向收益 |

以上都不是硬件快门类型认证、行时间示波器测量或生产放行结果。

## 1. 输入配置

最简配置见
[`config/examples/v1.0.0/rolling_shutter_diagnosis.yaml`](../config/examples/v1.0.0/rolling_shutter_diagnosis.yaml)，
所有字段、默认值和范围见
[`all_params/rolling_shutter_diagnosis_full.yaml`](../config/examples/v1.0.0/all_params/rolling_shutter_diagnosis_full.yaml)。

```yaml
schema_version: "1.0.0"
kind: rolling_shutter_diagnosis
source_task:
  path: stereo_camera_calibration_task.yaml
capture:
  motion: continuous
  camera_timestamp_event: row0_exposure_end
rolling_shutter:
  cam0:
    max_abs_line_delay_s: 0.00002
  cam1:
    max_abs_line_delay_s: 0.00002
```

`source_task.path` 必须指向当前可直接执行的普通 `camera_calibration` 或
`camera_imu_calibration` task。工具会把其中的目录／bag、标定板、相机、IMU、初值、
检测参数和线程预算解析为绝对路径，再在自己的隔离输出目录执行。Camera–IMU 输入必须显式
提供 `camera_calibration.path`；新的隔离输出目录不能可靠猜测原任务曾经依赖哪一份相机结果。

只支持 `pinhole-equi` 和 `pinhole-radtan`。`rolling_shutter` 只填写每目搜索上界，不能
填写预期行时间。离线筛查从 0 开始，只有证据满足门槛才用候选值构造正式对照。

`capture.motion` 必须根据真实采集方式填写：

- `continuous`：图像序列在时间上连续，邻帧差分可以近似局部运动；
- `stop_and_go`：每个姿态停住拍摄，邻帧差分不代表曝光时的瞬时运动；
- `unknown`：采集过程无法确认。

后两类最多得到 `inconclusive`，不会自动判成 `important`。

## 2. 运行

```bash
python3 tools/kalibr_rolling_shutter_diagnosis.py \
  --config /path/to/rolling_shutter_diagnosis.yaml \
  --output-dir /path/to/result/rolling_shutter_diagnosis
```

默认使用 `install/release/bin/kalibr-noros`。可显式指定其他已安装入口：

```bash
python3 tools/kalibr_rolling_shutter_diagnosis.py \
  --config diagnosis.yaml \
  --output-dir diagnosis_output \
  --kalibr-noros install/project-profile/bin/kalibr-noros
```

`--dry-run` 验证配置、创建清单并只打印第一阶段普通任务命令；由于正式对照的 seed 和 knot
频率来自第一阶段观测，未真实执行普通任务前不能生成第二阶段配置。默认支持断点续跑：存在
结果 YAML 且对应 `run_manifest.json` 为 `completed` 的运行不会重复执行。`--force` 只会
覆盖带本工具 `diagnosis_manifest.json` 的受管输出；非空且不属于本工具的目录会被拒绝。

真实运行全部串行，避免多个求解器同时占用大量内存。检测和优化器内部线程仍遵循源 task 的
`execution` 配置。

## 3. 第一阶段：普通任务和离线筛查

工具先执行源普通任务，并强制开启：

- `save_diagnostics: true`；
- `save_metrics: true`；
- `archive_observations: true`；
- `archive_selection_history: true`。

这不会更改源 task 文件，只在 `configs/ordinary_source.yaml` 写一份解析后的副本。

### 3.1 固定 K/D 的逐帧位姿残差

对每个最终使用帧，固定普通标定结果中的 K/D，只重新拟合该帧标定板六自由度位姿。设角点
测量为 $\mathbf z_{ij}$，固定光学模型的预测为 $\hat{\mathbf z}_{ij}$：

$$
\mathbf r_{ij}=\mathbf z_{ij}-\hat{\mathbf z}_{ij},\qquad
\operatorname{RMS}=\sqrt{\frac{\sum_{ij}\lVert\mathbf r_{ij}\rVert^2}{N}}.
$$

一个二维角点计一个样本。这样先移除了每帧普通刚体位姿能解释的误差，再检查剩余误差是否
具有行坐标和运动速度共同决定的结构。

### 3.2 行方向因子和位姿方向消除

工具用同一时间段内前后邻帧位姿的投影有限差分估计角点图像速度
$\dot{\mathbf u}_{ij}$。行时间 $\tau$ 的一阶像素影响近似为：

$$
\Delta\mathbf u_{ij}\approx
\dot{\mathbf u}_{ij}\left(y_{ij}-\frac{H-1}{2}\right)\tau.
$$

中心行只选择时间参考，不改变逐行斜率。随后把这个方向投影到逐帧 PnP 位姿 Jacobian 的
六维列空间之外，避免把仍可由该帧位姿解释的分量再次记到行时间名下。

列方向负对照把 $y$ 换成 $x$：

$$
\Delta\mathbf u^{col}_{ij}\approx
\dot{\mathbf u}_{ij}\left(x_{ij}-\frac{W-1}{2}\right)\tau_{col}.
$$

真实逐行扫描应主要与行坐标相关；若列方向取得相近收益，更可能是未建模运动、板面误差、
模糊、光学模型或每帧位姿近似共同造成的通用空间趋势。

### 3.3 严格前后分块

按时间顺序把每目帧分成前半和后半。每一半单独构造邻帧速度并丢弃本段首末帧，因此预处理
不会跨越训练／验证边界。先在前半拟合一个 $\tau$、在后半固定验证，再反向执行一次。

最低数据要求为每目 20 个有效帧，并且去掉边界后每半至少 8 帧。每个验证方向按“帧”为
单位进行 1000 次有放回 bootstrap；角点不是相互独立样本，因此不能把几千个角点当成
几千次独立拍摄。

筛查通过要求每目同时满足：

1. 两个方向的留出 RMS 相对下降都不少于 10%；
2. 两个方向的 RMS 绝对改善 bootstrap 95% 下界都大于 0；
3. 两方向中较弱的行方向相对改善仍至少为最强列方向改善的两倍；
4. 两段估计的行时间同号；
5. 候选行时间绝对值小于配置上界的 95%。

候选 seed 是两个分块方向估计的中位数。它是有限差分统计值，不是最终联合估计。

## 4. 第二阶段：正式同求解器对照

仅当所有相机筛查通过且 `capture.motion: continuous` 时，工具才串行运行第二阶段。

### 4.1 纯视觉单目／双目／多目

使用正式 `kalibr-noros calibrate cameras-rs`。普通结果转换为 K/D/T 初始化；双目和
多目固定 K/D，以减少光学参数对行时间的吸收，单目因当前公开契约不允许
`freeze_intrinsics: true` 而继续联合优化 K/D。所有组关闭最终异常点过滤，保持共同输入角点。

基础 knot 频率取最终使用观测时间差中位数的倒数。实际运行：

| 运行 | 行时间 | 是否估计 | knot 频率 |
|---|---:|---:|---:|
| `free_seed0` | 0 | 是 | 1× |
| `fixed_zero` | 0 | 否 | 1× |
| `fixed_candidate` | 筛查候选 | 否 | 1× |
| `fixed_opposite` | 反向候选 | 否 | 1× |
| `free_seed_candidate` | 筛查候选 | 是 | 1× |
| `free_seed_opposite` | 反向候选 | 是 | 1× |
| `knot_half_free` / `knot_half_zero` | 0 | 是／否 | 0.5× |
| `knot_double_free` / `knot_double_zero` | 0 | 是／否 | 2× |

每个轨迹带宽都有自己的固定 0 对照，避免拿不同样条能力的两个目标函数直接比较。

### 4.2 Camera–IMU

使用正式 `kalibr-noros calibrate imu-camera-rs`，执行基础的六组 1× 对照。主运行
`free_seed0` 开启 `recover_covariance: true`；其他组不重复昂贵的协方差恢复。Camera–IMU
没有公开的目标轨迹 `knots_per_second` 控制，因此不伪造纯视觉 knot 敏感性实验。

### 4.3 正式门槛

自动 `important` 要求全部相机和全部强制运行同时满足：

- 所有运行成功、结果有限且没有被任务失败条件拒绝；
- 自由行时间低于上界 95%；
- 最终可观性为 `full_rank`；在任何 operational weak mode 中，行时间块贡献不超过 50%；
- 自由行时间相对固定 0 的公共帧、公共角点残差改善，其按帧 bootstrap 95% 下界大于 0；
- 固定候选优于固定反向候选；
- 三个自由初值的结果跨度不超过上界的 10%；
- 纯视觉 0.5×／1×／2× knot 的行时间跨度不超过上界的 20%，且每个频率均优于自己的固定 0 对照；
- Camera–IMU 的 $|\tau|>1.96\sigma_\tau$，并且 gyro 与 accel RMS 不会同时比固定 0 更差。

任一正式运行失败、缺文件、不可观、贴边或敏感性不通过，最终结论都是 `inconclusive`，
不会因某一次 RMS 最低而提升结论。

正式视觉 RS 对照默认固定普通源任务得到的 K/D，使控制实验只研究轨迹与行时间。需要研究
动态 RS 内参与普通或静态内参的一致性时，可显式设置
`formal_controls.freeze_intrinsics: false`；此时 10 组正式对照都把 K/D 作为 active 状态，
报告中的内参才是各组重新优化的结果。`formal_controls.max_iterations` 可显式限制每组迭代
预算；省略时保持原有相机 RS 至少 5000 次、Camera–IMU RS 至少 100 次的诊断默认值。
达到显式上限仍未满足停止条件时任务失败，不应把未收敛结果用于一致性判断。

## 5. 输出目录

```text
output/
├── diagnosis_manifest.json
├── screening.json
├── screening_frames_rejected.csv
├── formal_comparison.json              # 仅启动正式阶段时
├── result.json
├── summary.csv
├── run_metrics.csv                     # 普通标定与已完成正式RS对照的逐目指标
├── README_ZH.md
├── report.html
├── report.pdf
├── suggested_rolling_shutter_task.yaml # 仅important时
├── charts/
├── configs/
├── logs/
└── runs/
```

`diagnosis_manifest.json` 保存输入配置和依赖文件 SHA-256。普通任务完成后还会记录观测
归档列出的全部输入图像 SHA-256，并在全部正式对照结束后重新计算；输入变化会使任务失败。

`runs/` 保留现有 `kalibr-noros` 的原生结果、观测、指标、可观性和报告，工具不重新定义
它们。`formal_comparison.json` 的残差比较只使用两次运行中相机 ID、源时间戳和 corner ID
都相同的公共角点。

`run_metrics.csv` 将 1 组普通标定与所有已完成的正式 RS 对照放在同一张表中。每目记录
行时间模式（`estimated`、`fixed` 或 `not_applicable`）、行时间、重投影 RMS、可用的双目
Optical Alignment RMS 和 RS 补偿配对残差，便于直接比较和绘图。全部 10 组完成时，报告
显示 `1 + 10` 组；若某个正式求解失败，报告保留此前已完成组的指标，同时列出完成数和未完成
组，结论保持 `inconclusive`，也不计算正式门槛。若离线筛查没有启动正式阶段，报告只有普通
标定和筛查证据。因此，不同数据或不同完成状态下的 `report.html` 内容会不同。

### 5.1 Optical Alignment RMS 如何计算

实现位于 `src/python/kalibr_no_ros/evaluation.py` 的 `stereo_geometry()`、
`rectified_points()` 和 `_stereo_pair_metrics()`：

1. 从最终使用的左右帧中按共同 `corner_id` 配对检测角点；
2. 用最终 K、D、`T_cn_cnm1` 调用 OpenCV `stereoRectify`；`pinhole-equi` 使用
   `cv2.fisheye.stereoRectify()` 和 `cv2.fisheye.undistortPoints()`；
3. 将原始检测角点直接变换到校正坐标。这里不是先校正整幅图像再重新检测角点；
4. 水平基线取校正后左右角点的纵坐标差，垂直基线取横坐标差。这个方向称为
   non-disparity axis；
5. 丢弃非有限值和落在校正图像范围外的角点，在全部有效共同角点上计算 RMS。

水平双目的单角点误差及汇总为：

$$
e_{\mathrm{opt},k}=\left|v^{\mathrm{rect}}_{L,k}-v^{\mathrm{rect}}_{R,k}\right|,
\qquad
\operatorname{RMS}_{\mathrm{opt}}=
\sqrt{\frac{1}{N}\sum_{k=1}^{N}e_{\mathrm{opt},k}^{2}}.
$$

垂直双目把 $v$ 换为 $u$。该指标只使用 K/D/T 和 `output.rectification` 选项，不使用
行时间、连续轨迹、标定板深度或逐角点时刻。因此，动态 RS 数据即使已经正确估计行时间，
Optical Alignment 仍可能较大；它回答的是“把这些动态观测当作同一时刻做静态光学校正后，
还剩多少非视差方向偏差”。

### 5.2 RS compensated pair residual 如何计算

实现位于同一文件的 `_rolling_shutter_pair_residual()`。它对同一个共同角点读取：

- `measurement_px`：检测到的真实像素 $\mathbf m$；
- `prediction_px`：最终 RS 联合求解器按该角点实际行时刻，用 K/D/T、行时间和连续轨迹
  投影得到的预测像素 $\hat{\mathbf m}$。

测量点和预测点都使用同一套 K/D/T 校正映射 $r(\cdot)$。在非视差方向 $p$ 上先分别求
左右差，再从测量差中减去模型预测差：

$$
e_{\mathrm{RS},k}=
\left|
\left[r_L(\mathbf m_{L,k})-r_R(\mathbf m_{R,k})\right]_p
-
\left[r_L(\hat{\mathbf m}_{L,k})-r_R(\hat{\mathbf m}_{R,k})\right]_p
\right|,
$$

$$
\operatorname{RMS}_{\mathrm{RS}}=
\sqrt{\frac{1}{N}\sum_{k=1}^{N}e_{\mathrm{RS},k}^{2}}.
$$

这里的校正映射本身仍只由 K/D/T 决定；行时间和连续轨迹通过 `prediction_px` 进入第二项。
所以约 `0.601 px` 的 Optical Alignment 表示原始动态观测中仍含有逐行运动造成的左右非视差
差，而约 `0.219 px` 表示减去最终 RS 模型预测的这部分差异后剩余的配对残差。两者不是同一
指标，也不要求数值接近。

`RS compensated pair residual` 使用了当前数据拟合出的轨迹和行时间，属于
`fit_dependent: true` 的诊断量，`independent_validation: false`。它能说明联合模型解释了多少
已拟合观测，却不能单独证明模型可泛化；仍需固定 0、反方向、多初值、knot 敏感性以及独立
测试数据。最终结果 YAML 只保存两个 RMS 标量，完整 count、mean、p95、max 等统计保存在各组
`metrics.json`。诊断工具的 `analysis.load_run_summary()` 读取这些结果，`reporting.py` 只负责
汇总展示，不重新计算这两个指标。

### 5.3 能否先按行时间修正角点，再计算 Alignment

可以，但不能只用行时间 $\tau$。行时间只能给出角点的采样时刻：

$$
t_{i,k}=t_{i,\mathrm{frame}}+(y_{i,k}-y_{i,\mathrm{ref}})\tau_i.
$$

要把检测角点从 $t_{i,k}$ 搬到统一参考时刻 $t_*$，还必须知道该时间段的连续运动轨迹、
该角点对应的标定板三维点以及 K/D/T。设联合模型在实际角点时刻和参考时刻的预测分别为
$\hat{\mathbf m}_i(t_{i,k})$、$\hat{\mathbf m}_i(t_*)$，在校正坐标中可定义：

$$
\mathbf c_{i,k}=
r_i(\mathbf m_{i,k})+
\left[r_i\!\left(\hat{\mathbf m}_i(t_*)\right)
-r_i\!\left(\hat{\mathbf m}_i(t_{i,k})\right)\right].
$$

然后计算修正后的非视差方向对齐误差：

$$
e_{\mathrm{motion\text{-}comp},k}
=\left|[\mathbf c_{L,k}-\mathbf c_{R,k}]_p\right|.
$$

同一三维角点在同一参考时刻经过理想双目校正后满足：

$$
\left[
r_L\!\left(\hat{\mathbf m}_L(t_*)\right)
-r_R\!\left(\hat{\mathbf m}_R(t_*)\right)
\right]_p\approx 0.
$$

把它代入上式，就得到 5.2 节当前代码计算的
`RS compensated pair residual`。因此，当前实现本质上已经是在校正坐标中执行“逐行运动补偿
后再算 Alignment”，只是省略了显式生成 $\mathbf c_{i,k}$ 和参考时刻预测点的步骤。两种写法
不应作为两项相互独立的证据重复计分。

如果要生成可视化的逐行补偿图像或显式的修正角点，求解输出还需要保留或重新加载可在任意
$t_*$ 求值的最终轨迹。目前观测归档保存了每个角点实际时刻的 `prediction_px`、
`solver_timestamp_s` 和 `T_camera_target`，足够计算当前紧凑指标；它没有序列化完整样条控制点，
不能在离线报告阶段严格重建任意统一参考时刻的整幅图像。

无论采用紧凑写法还是显式角点修正，只要轨迹和行时间由同一批角点联合拟合，结果仍是
fit-dependent。更独立的验证应固定已标定的 K/D/T/$\tau$，用 IMU 或训练帧估计运动轨迹，
再在未参与轨迹或行时间拟合的测试帧上计算 motion-compensated Alignment。

### 5.4 怎样做有验收意义的独立验证

“训练帧”是参与估计 K/D/T、行时间或连续轨迹的帧；“测试帧”是完全不向这些参数提供残差
的另一组帧。若先用某帧角点拟合该帧位姿，再用同一批角点评价，只能说明样本内拟合质量。
独立验证至少按下面步骤执行：

1. 用动态标定集 A 求出 K/D/T、每目 $\tau$、Camera–IMU 外参和时间偏移；
2. 冻结这些参数，不再优化；
3. 使用独立动态测试集 B。最好由外部位姿系统提供轨迹；Camera–IMU 系统可由 IMU 和不包含
   被测角点残差的状态估计器提供连续轨迹；纯视觉只能用训练帧估轨迹，并把留出的短连续时间段
   作为测试段，独立性较弱；
4. 对测试角点按原始传感器行号计算实际时刻，将左右观测变换到同一世界坐标或参考时刻；
5. 在完全相同的测试帧、轨迹和 K/D/T 上分别使用标定 $\tau$、$\tau=0$ 和 $-\tau$；
6. 比较下面的测试集指标，并按帧 bootstrap 给出改善量的 95% 区间。

建议分三层阅读指标：

| 层级 | 指标 | 能回答的问题 |
|---|---|---|
| 样本内拟合 | 每目 RS reprojection RMS、`rs_compensated_pair_residual` | 当前联合模型能否解释参与拟合的观测 |
| 识别控制 | 固定 0、反方向、多初值、不同 knot、可观性 | 改善是否确实依赖正确行时间，而不是轨迹自由度或初值 |
| 独立验收 | 测试集 motion-compensated Alignment、测试集重投影、三角化三维误差 | 参数能否推广到新数据并改善最终用途 |

用于双目测距时，最有业务意义的是测试集三维指标：AprilGrid 角点三角化后到已知板面的距离、
已知点间距误差、深度 bias/RMS/P95，以及这些指标相对 $\tau=0$ 的改善。静态
Optical Alignment 主要验证 K/D/T；动态测试集的 motion-compensated Alignment 验证时间模型
和运动补偿；两者不能互相替代。

验收不应只规定“RMS 小于某个通用数值”。应先用多次独立采集建立正常分布，再规定：标定
$\tau$ 在测试集上稳定优于 0 和反方向、改善的 bootstrap 95% 下界大于 0、重复标定得到的
$\tau$ 和三维误差在业务容差内，并且不同运动速度下没有系统性退化。当前诊断工具完成前两层
的大部分检查，但还没有自动执行带独立轨迹和真实三维基准的第三层生产验收。

当结论为 `important` 时，建议 task 的 seed 取三个 1× 自由初值结果的中位数，并保留
`estimate: true`。它只是后续正式任务模板，不替换普通结果，也不自动进入生产配置。

## 6. 仍需人工检查的竞争解释

即使得到 `important`，仍应检查运动模糊、AprilGrid 平面度、角点系统偏差、左右固定采样
时差、错误时间戳事件语义和相机模型不匹配。若相机时间戳记录首行曝光结束，则正
`line_delay_s` 表示图像 $y$ 增大时角点时刻更晚；若真实扫描方向或时间戳事件不同，必须先
修正输入语义，不能只靠把估计符号翻转来解释。

静态数据适合 K/D/T，却不能提供足够曝光期间运动。已经大幅抽帧的数据会降低连续轨迹约束，
容易让轨迹形状、K/D、外参和行时间重新分配误差。工具可以指出证据不足，但不能从不足数据
恢复丢失的时间信息。
