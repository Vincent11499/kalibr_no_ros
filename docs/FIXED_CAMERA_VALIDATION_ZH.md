# 固定相机参数测试集验证与标定诊断

本文说明如何用独立测试集验证已有双目标定结果，以及如何判断增量顺序、可观性和异常点
过滤对结果的影响。

## 1. 独立测试集验证

`kalibr-noros verify cameras` 在测试集上重新检测标定板，但固定已有结果中的相机内参
$\mathbf K$、畸变 $\mathbf D$ 和相邻外参 $\mathbf T$。该命令不调用标定优化器，因而
不会把测试集重新拟合成一套相机参数。

```bash
kalibr-noros verify cameras \
  --calibration 2136=/path/to/camera_calibration_2136.yaml \
  --calibration 2152=/path/to/camera_calibration_2152.yaml \
  --dataset /path/to/stereo_test_YYMMDD_hhmm \
  --target /path/to/aprilgrid.yaml \
  --output-dir /path/to/verification \
  --window-half-size-px 7 \
  --max-displacement-px 1 \
  --synchronization-tolerance-s 0.0001 \
  --visualizations \
  --max-frames-per-camera 30 \
  --max-pairs 30
```

`--calibration` 可重复。推荐写成 `标签=结果YAML`；省略标签时使用文件名。输出包括：

- `fixed_camera_validation.json`：汇总指标、逐帧 RMS、配对和共同角点数量；
- `fixed_camera_validation.csv`：每张图像及每个同步双目图对的误差明细；
- `README_ZH.md`：便于人工检查的汇总表；
- `run_manifest.json`：软件版本和运行状态；
- `.inventory.json`：受管输出清单，供 `--force` 安全覆盖时核对。

CSV 使用 `record_type` 区分两类行：

- `camera_frame`：每张相机图像的 `corner_count`，以及二维重投影误差范数的
  `reprojection_rms_px`、`reprojection_p95_px`、`reprojection_max_px`；检测失败的图像
  仍保留一行，角点数为 0，误差字段为空。
- `stereo_pair`：同步左右图的各自角点数、合并两目独立 PnP 重投影残差后的 RMS/P95/Max，
  以及共同角点在校正域非视差方向上的 Alignment RMS/P95/Max。

P95 使用所有有限逐角点误差范数的线性 95 百分位；Max 为最大逐角点误差。多套参数由
`calibration_label` 区分。CSV 是 JSON 的表格化明细，原有 JSON 继续完整保留，并增加
对应的 P95/Max 字段。

`--visualizations` 显式开启测试集证据图，默认关闭，不影响检测、配对和指标。多套参数按
`--calibration` 标签隔离，目录结构为：

```text
visualizations/<标签>/
├── cam0_det/       # 原图角点；左上角显示检测、保留和标定板总角点数
├── cam0_dist/      # cam0 单目去畸变图
├── cam1_det/       # cam1 原图角点
├── cam1_dist/      # cam1 单目去畸变图
└── cam0_cam1/      # 固定该标签 K/D/T 后的双目极线对齐图及逐对 Alignment RMS
```

`--max-frames-per-camera` 和 `--max-pairs` 均为正整数，默认 30，采用稳定的时间顺序均匀
抽样，只限制图片数量，不限制统计使用的数据。单目去畸变默认保留最大视场，可能有黑边；
传入 `--undistortion-crop` 后缩放并裁去无效边。`cam0_cam1` 使用绿色 3 px 极线，左上角
显示该图对的 Alignment RMS 和有效共同角点数。

默认 `window_half_size_px=2`、`max_displacement_px=sqrt(1.5)`，保持原生检测默认值。
验证训练标定时建议显式填写与训练任务相同的两个数值。同步容差默认 0.0002 s；单位是秒。
`rectification_balance` 默认 0，合法范围 `[0,1]`；`rectification_fov_scale` 默认 1，必须
大于 0。

### 1.1 每目 RMS

对每台相机、每帧图像，用固定 $\mathbf K,\mathbf D$ 做标定板 PnP，只估计该帧
标定板位姿。设检测点为 $\mathbf z_{ij}$，固定相机模型投影为
$\hat{\mathbf z}_{ij}$，则

$$
\operatorname{RMS}_c=
\sqrt{\frac{\sum_{i,j}\lVert\hat{\mathbf z}_{ij}-\mathbf z_{ij}\rVert_2^2}
{N_c}}.
$$

这里每个二维角点计为一个样本，不把 x、y 当成两个样本。

### 1.2 双目综合 RMS

双目综合 RMS 只合并两目的二维重投影残差：

$$
\operatorname{RMS}_{stereo}=
\sqrt{\frac{N_0\operatorname{RMS}_0^2+N_1\operatorname{RMS}_1^2}{N_0+N_1}}.
$$

右目板位姿由右目测试图像独立 PnP 得到；该指标用于验证两套内参在新图像上的拟合，
不会用待测外参重新构造右目位姿。双目外参由下面的 Alignment RMS 单独验证。

### 1.3 Alignment RMS

按时间容差一对一配对左右帧，按全局 `corner_id` 取共同角点。使用固定
$\mathbf K_0,\mathbf D_0,\mathbf K_1,\mathbf D_1,{}^{cam1}_{cam0}\mathbf T$
进行 OpenCV 双目校正。若校正后的视差轴为 x，则只统计 y 差：

$$
e_k=y_{0,k}^{rect}-y_{1,k}^{rect},\qquad
\operatorname{RMS}_{align}=\sqrt{\frac{1}{N}\sum_k e_k^2}.
$$

基线为固定外参平移向量的模，不从测试集重新估计。报告中的参考评级沿用项目显示阈值，
不是生产判定规则。

## 2. shuffle 与增量优化

普通相机标定并非只执行一次 LM。当前原生路径分为下面四步：

| 阶段 | 参与的数据 | 活动参数 | 该阶段的用途 |
|---|---|---|---|
| 单目初始化 | 每台相机自己的有效观测 | 该目内参、畸变、每帧板位姿 | 给每台相机建立可用的内参和畸变初值 |
| 相机对 baseline 初始化 | 有共同角点的相机对 | 两目投影内参、相对位姿、每帧板位姿；畸变固定 | 得到相机对之间的相对位姿初值 |
| full-batch refinement | 当时检测到的全部 view、全部相机 | 所有相机内参、畸变、相邻 baseline、每帧板位姿；shutter 固定 | 把分别得到的初值放进同一个多目问题中，消除各阶段之间的不一致 |
| 原生增量估计 | 依次尝试加入 view | 已接纳 view 的板位姿和全部相机物理参数 | 选择有效且有信息的 view，生成最终问题、最终参数和可观性状态 |

### 2.1 baseline 初始化具体做了什么

多目图首先按共同观测建立相机连接图。边的权重取共同角点总数的倒数，因此共同角点越多，
越倾向于被用来建立相机链。对选中的每个相机对，程序执行以下步骤：

1. 分别对左右观测做 PnP，得到标定板相对于两台相机的位姿；
2. 由同一时刻的两个 PnP 位姿计算一组相机间相对位姿，并对所有 view 的旋转向量和平移
   分别取中位数，作为 baseline 初值；
3. 建立相机对的 bundle adjustment，同时优化 baseline、两目的投影内参和每帧板位姿，
   此时畸变参数保持单目初始化值；
4. 多目情况下，把选中图边的变换连乘为相邻相机的
   ${}^{cam_n}_{cam_{n-1}}\mathbf T$。

这一阶段解决的是“先让相机之间的方向和平移落到正确解附近”。如果一开始 baseline 偏差
太大，后面的联合非线性优化可能进入错误的局部最小值，甚至无法构造可靠的板位姿初值。

### 2.2 full-batch refinement 具体做了什么

`solveFullBatch()` 建立一个同时包含全部 view 的优化问题。每个有效角点贡献一个二维
重投影残差，假设角点标准差为 1 px。它联合调整：

- 每台相机的投影内参；
- 每台相机的畸变参数；
- 每一对相邻相机的 6 自由度 baseline；
- 每个 view 的 6 自由度标定板位姿。

该阶段使用 LM，最多 250 次迭代。它更新共享的相机对象，并返回 refinement 后的
baseline；每帧板位姿、优化问题、信息矩阵和 view 选择状态都不会返回。随后代码创建新的
`CameraCalibration` 和新的 `IncrementalEstimator`。因此 full-batch 的 K、D、T 数值会作为
更好的起点进入下一阶段，但这个 full-batch 问题本身不是最终输出问题。

### 2.3 为什么还要逐个加入 view

这里的“逐个加入”不表示只优化新加入的一帧。每次尝试一个新 view 时，增量估计器都会对
“此前已经接纳的全部 view + 当前候选 view”重新联合优化，然后检查：

1. 优化是否在 50 次迭代内结束；
2. 最终代价是否低于开始代价；
3. 标定物理参数的秩是否增加，或信息增益是否超过阈值。

满足条件才保留这个 view；否则恢复加入前的全部参数，并从问题中删除候选 view。后续的
角点异常值过滤也围绕这些已接纳 batch 重建问题。最终 YAML、RMS、观测归档以及
`observability.yaml` 都对应这套最终保留下来的问题。

所以两次联合求解承担不同职责：full-batch 负责把分阶段初值带入一致的解域；增量阶段负责
确定最终使用哪些 view，并保存原生的秩、信息增益、回滚和异常点处理语义。增量阶段还可使用
Blake-Zisserman M-estimator，而初始化 full-batch 没有这套接纳机制。它们虽然使用相同的
角点投影模型，但数值路径和最终数据集合并不相同。

### 2.4 full-batch 能否直接作为最终结果

数学上可以设计“全量 batch 作为最终解”的求解模式；并不存在 full-batch 结果天然不能
输出的限制。但当前 `solveFullBatch()` 只是初始化接口，尚不具备正式最终求解器所需的完整
行为：

- 只以线性求解器是否失败判断成功，没有增量阶段的代价下降和信息增益接纳；
- 不返回每帧板位姿、最终问题、可观性状态和 view 使用清单；
- 不执行当前的 batch 回滚及角点异常值重建流程；
- 会无条件纳入全部 view，包括重复、弱约束或质量较差的观测。

因此不能仅把现有初始化函数的返回值改名为“最终结果”。若要减少顺序敏感性，可以新增明确
的 final full-batch solver mode，在相同角点和相同初值下定义异常点处理、失败条件、最终
可观性和输出证据，再与原生增量模式比较。全量优化通常较少受加入顺序影响，但它不会自动
排除坏帧，也不保证比经过独立测试集验证的增量结果更准确。

`shuffle: false` 按时间顺序加入 view。连续采集的静态标定图像常把相近位置、距离或倾角
集中在一起，前几个 batch 对焦距、畸变和板位姿的约束可能高度相关。当某个新 view 加入
后，LM 在 50 次迭代内未收敛，原生代码就抛出 `OptimizationDiverged` 并整体重启；三次
使用相同顺序会在同一点再次失败。

`shuffle: true` 只改变增量加入顺序。若前几个 view 更早覆盖中心/边缘、近/远和不同倾角，
Hessian 会更早获得互补方向，后续 view 更容易收敛。因此同一组观测可能从失败变为成功。
这说明数据和增量路径对顺序敏感，并不证明随机顺序产生的参数必然更准确。应同时检查
独立测试集和可观性。

## 3. observability.yaml

`observability.yaml` 在最终解处重新线性化，不再更新参数。它先消去每帧板位姿等 nuisance
变量，再分析相机物理参数的 reduced Fisher information。

### 3.1 先解释几个专用名词

- **残差（residual）**：检测角点与模型预测角点之间的二维像素差。残差越小，模型对这条
  观测的解释越好。
- **线性化（linearization）**：在当前参数附近只考虑很小的参数变化，用一阶导数近似残差
  如何变化。导数组成 Jacobian 矩阵。
- **物理标定参数**：最终需要交付的 K、D 和相邻相机外参。
- **nuisance variables**：求解时必需、但不属于交付标定值的辅助未知量。普通相机标定中
  主要是每个 view 的标定板位姿，每帧 3 个旋转和 3 个平移自由度。这里的 nuisance 是
  “本次分析不关心”，不是“噪声”或“错误参数”。
- **Fisher information**：在当前解附近，残差对参数变化的敏感程度。某个参数组合稍微变化
  就使许多像素残差显著增大，信息强；变化后几乎可由其他变量抵消，信息弱。
- **可观（observable）**：数据能否把某个参数组合与其他组合区分开。可观性描述“能不能
  稳定确定”，不直接等同于参数“准不准”。

### 3.2 “在最终解处重新线性化”是什么意思

设白化后的像素残差为 $\mathbf r$，每帧板位姿为 $\mathbf p$，需要分析的相机参数为
$\boldsymbol\theta$。在最终解附近做一阶展开：

$$
\mathbf r(\mathbf p+\delta\mathbf p,
          \boldsymbol\theta+\delta\boldsymbol\theta)
\approx
\mathbf r+
\mathbf J_p\delta\mathbf p+
\mathbf J_\theta\delta\boldsymbol\theta.
$$

“重新线性化”是用最终 K、D、T 和最终板位姿重新计算 $\mathbf J_p$ 与
$\mathbf J_\theta$，而不是沿用优化早期的导数。“不再更新参数”是指原生求解器虽然计算了
一个线性更新向量，但诊断代码主动丢弃它；K、D、T 和板位姿不会被这次分析改变。

代码会按最终误差项的协方差和 M-estimator 权重建立 Jacobian；`uses_m_estimator` 记录是否
实际存在稳健权重。`damping_applied: false` 表示没有把 LM 阻尼当成虚假的观测信息。

### 3.3 为什么要消去每帧板位姿

一张标定板图像之所以落在当前像素位置，既与相机 K/D/T 有关，也与该帧板放在哪里有关。
例如，焦距略变后，把板沿深度方向移动一点，投影尺寸可能仍然很接近。如果分析时把板位姿
固定不动，会错误地认为数据对焦距约束很强；现实中板位姿本来就是未知的，它应被允许作出
最有利的补偿。

把完整 Jacobian 写成

$$
\mathbf J=[\mathbf J_p\;\mathbf J_\theta],
$$

完整的 Gauss-Newton 信息矩阵为

$$
\mathbf H=\mathbf J^\mathsf T\mathbf J=
\begin{bmatrix}
\mathbf H_{pp} & \mathbf H_{p\theta}\\
\mathbf H_{\theta p} & \mathbf H_{\theta\theta}
\end{bmatrix}.
$$

让每帧板位姿对给定的 $\delta\boldsymbol\theta$ 作出最佳调整后，相机参数真正剩下的信息为

$$
\mathbf H_{reduced}=
\mathbf H_{\theta\theta}-
\mathbf H_{\theta p}\mathbf H_{pp}^{+}\mathbf H_{p\theta},
$$

其中 $+$ 表示可处理秩亏的广义逆。这个式子叫 Schur complement。当前实现不会直接计算
矩阵逆，而是先对 $\mathbf J_p$ 做稀疏 QR 分解，再把 $\mathbf J_\theta$ 中能够被板位姿解释
的方向投影掉，得到等价的

$$
\boldsymbol\Omega=
\mathbf J_\theta^\mathsf T
(\mathbf I-\mathbf P_p)
\mathbf J_\theta,
$$

其中 $\mathbf P_p$ 是投影到板位姿 Jacobian 列空间的算子。这样数值上比显式求逆稳定。
`nuisance.rank` 就是 QR 分解判断出的板位姿列秩；若它自己已经亏秩，表示某些板位姿自由度
都不能从图像确定，后续相机参数可观性需要谨慎解释。

### 3.4 如何从 reduced information 判断强弱

内参像素、畸变无量纲、旋转弧度和平移米的单位差别很大。代码先对每一列做 L2 归一化，
因此 `scaling: l2` 下比较的是归一化参数方向，不会仅因为焦距数字大、平移数字小而产生
错误排序。随后对 $\boldsymbol\Omega$ 做 SVD：

$$
\boldsymbol\Omega=
\mathbf U\operatorname{diag}(\sigma_1,\ldots,\sigma_n)\mathbf V^\mathsf T,
\qquad \sigma_1\geq\cdots\geq\sigma_n\geq0.
$$

每一列 $\mathbf V_i$ 是一种“多个参数一起变化”的方向，$\sigma_i$ 是该方向的信息强度：

- 大奇异值：很小的参数变化就会留下无法由板位姿吸收的像素变化；
- 小奇异值：这组参数可以一起变化，而重投影残差变化很小；
- 接近零：当前数据局部无法区分这组参数。

文件同时给出两套判定：

1. `rank/deficiency` 使用接近机器精度的严格阈值，判断是否存在结构性或数值上的真正零方向；
2. `operational_rank/operational_deficiency` 使用相机增量求解器的
   `epsSVD=1e-6` 工程阈值，标出理论非零但过弱、不适合稳定更新的方向。

因此 `status: full_rank` 与 `quality: weakly_observable` 可以同时成立：所有方向在数学上都有
一点信息，但其中一些信息比最强方向小很多，数据扰动后相应参数组合容易明显变化。

### 3.5 各字段怎样读

| 字段 | 含义 |
|---|---|
| `status` | 按机器精度阈值判断是否存在严格秩亏；`full_rank` 表示没有精确零方向 |
| `quality` | 工程阈值下的强弱；`weakly_observable` 表示虽满秩，但有非常弱的方向 |
| `linearization.original_error_terms` | 纳入分析的原生误差项数量 |
| `parameter_blocks` | 相机内参、畸变、外参旋转/平移的维数、活动状态和偏移 |
| `nuisance` | 每帧板位姿等临时变量的列数、秩和亏秩 |
| `calibration.columns` | 被分析的物理标定自由度总数 |
| `rank/deficiency` | 机器精度判定的秩与亏秩 |
| `operational_rank/deficiency` | 使用 `operational_eps_svd` 的工程秩与弱方向数量 |
| `singular_values` | 从强到弱的信息特征值；越小表示相应参数组合越难区分 |
| `condition_information_observable` | 最大/最小可观信息特征值 |
| `condition_jacobian_equivalent` | 上一条件数的平方根，便于按 Jacobian 尺度理解 |
| `nullspace_modes` | 严格不可观方向的参数块贡献 |
| `operationally_truncated_modes` | 工程上过弱方向的参数块贡献 |

`parameter_blocks[].offset` 是该参数块在 $\boldsymbol\theta$ 中的起始列；它不是图像偏移或
时间偏移。`operationally_truncated_modes[].dominant_blocks[].contribution` 是弱方向向量在
各参数块上的平方占比。它适合判断弱方向主要落在内参、畸变还是外参，但当前文件没有保存
向量各元素的正负号，所以不能仅凭该占比反推出“具体哪两个系数一增一减”。

双目 pinhole-equi 的 22 个物理自由度来自两目各 4 个内参、各 4 个畸变参数和 6 个相邻
外参自由度。20 个 view 的 nuisance 是 20×6=120 个板位姿自由度。

`2136` 与 `2152` 都是 `rank=22/22`，说明没有严格不可解的方向；但二者都是
`operational_rank=20/22`。最后两个弱模态中，cam0/cam1 distortion 的贡献均约占 99%，
所以结论是：外参和主要内参可解，但某些高阶等距畸变参数组合可以互相补偿，数据不足以
稳定地区分它们。它不是“标定失败”，但表示换一组相似的 20 帧图像时畸变系数可能变化，
即使训练 RMS 仍然较低。

可观性只是在最终解附近的一阶局部诊断。它不能证明模型没有系统偏差，也不能替代独立测试
集 RMS、Alignment RMS、清晰度和覆盖度检查。满秩但独立测试误差大，仍然不是合格标定；
独立测试误差低但存在弱模态，则表示当前整体投影可能可用，但某些单独系数不稳定。

改进采集时应增加清晰且互补的姿态：让角点稳定覆盖四角和图像边缘；增加近、中、远距离；
增加绕水平轴、垂直轴和光轴的倾角；避免大量正视、居中或相邻重复姿态。增加帧数只有在
新增姿态带来不同成像半径和透视关系时才有效。若设备内参已由高质量主数据集确定，可在
后续设备外参任务中使用完整 seed 并设置 `freeze_intrinsics: true`，但这代表使用先验，
不能替代首次内参标定的数据覆盖。

## 4. 异常点过滤

原生普通相机标定的 `remove_outliers` 和 `final_filtering` 默认均为 true。过滤对象是单个
角点，不是整帧。对每台相机汇总当前所有 view 的 x/y 重投影误差标准差，阈值为每轴
`4*std`；超过任一轴阈值的角点会被移除，然后重建对应 batch 并重新求解。

过滤还有启动门槛：

```text
numActiveBatches > min_views_for_outlier_statistics * numCameras
```

默认 `min_views_for_outlier_statistics=20`。双目 20 个 view 时要求 active batch 数大于 40，
因此这两组数据从未进入过滤阶段；日志中的 `Removed 0 outlier corners` 是门槛未满足后的
结果，并不表示高残差角点经过过滤检查后被保留。

`2152` 的 `cam0:6` 有 59 个最终角点，整帧 RMS 约 1.492 px，最大单角点误差约
8.239 px；`cam0:7` RMS 约 0.664 px。由于启用了 shuffle，`cam0:6` 是源数据索引，
不是优化器第 7 个加入的 view。

小数据集可显式降低 `min_views_for_outlier_statistics`，但阈值统计也会更不稳定。对 20 个
双目 view，设置 10 仍因严格大于条件而不会触发；设置 9 或更小才可能在末尾触发。修改前
应先查看角点图和逐帧残差，确认是模糊、反光、板弯曲或误检测；若整帧普遍较差，角点级
过滤不等价于合理的整帧剔除，应重新采集或显式构造经审核的数据子集。

## 5. Allan 方差工具状态

当前项目内部没有 `kalibr_allan` 功能、CLI 子命令、CMake target 或安装后的同名可执行文件。
`kalibr-noros` 当前也没有 Allan 分析入口。仓库只在文档中提到 Allan 数据和噪声估计，并在
Camera-IMU 标定时读取 `imu.yaml` 中已经准备好的噪声密度、随机游走等参数；它不会从静态
IMU 数据自动生成这些参数。

因此 Allan 方差分析目前是标定工程外部的前置流程。若以后集成，应作为独立的 IMU 噪声
分析命令输出候选参数和分析报告，再由人工审核后写入 `imu.yaml`，不要把 Allan 静态数据
误当作同步 Camera-IMU 标定数据。
