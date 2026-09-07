# 固定相机内参、扰动双目外参验证报告

日期：2026-09-05

## 1. 验证目的

本轮验证针对 `camera_calibration` 新增的：

```yaml
calibration:
  freeze_intrinsics: true
```

需要确认：

1. 两台相机的投影内参 $\mathbf I$ 和畸变参数 $\mathbf D$ 在所有阶段严格固定；
2. 双目 baseline $\mathbf B={}^{C_1}_{C_0}\mathbf T$ 仍为 active；
3. 外参初值加入确定性旋转、平移噪声后能够回到可信解；
4. `refine` 和 `direct` 的阶段差异不会改变最终参数活动策略；
5. 固定较多 calibration block 后，原生增量求解器仍能处理早期不可观批次；
6. 省略该配置或设为 `false` 时，原有内参、畸变和外参联合优化路径不变。

本报告是功能和鲁棒性验证，不替代全量性能 benchmark。

## 2. 数据和冻结基准

采用 EuRoC 双目 AprilGrid 数据：

- Bag：`/mnt/q/File/kalibr/data/euroc_cam/cam_april.bag`；
- 标定板：`/mnt/q/File/kalibr/data/euroc_cam/april_6x6.yaml`；
- 相机模型：两路均为 `pinhole-radtan5`；
- 冻结基准：`performance_project_reference_v2/camera_project_4/calibration.yaml`；
- 冻结基准 SHA-256：
  `64f6fde32724642e4e1742d89f5165e32223eb1ea14922876299678182fc1a8e`。

为了使多组功能测试可快速重复，本次统一使用代表性子集：

```yaml
dataset:
  time_range_s: [0.0, 60.0]
  frequency_hz: 5.0

calibration:
  freeze_intrinsics: true
  shuffle: false

execution:
  detector_processes: 4
  optimizer_threads: 4
  detector_opencv_threads: 1
```

每路读取 301 张图像；cam0 检测成功 240 张，cam1 检测成功 239 张；同步后的候选
target view 为 240 个。固定 `shuffle: false` 保证不同初始化策略使用相同的视图顺序。

这 60 秒子集与冻结基准所用的全量数据不同，因此最终外参允许出现很小的数据子集
差异；比较初始化策略时，主要使用三次子集运行之间的差异。

## 3. 外参噪声设计

设冻结基线为：

$$
{}^{C_1}_{C_0}\mathbf T_{\mathrm{ref}}
=
\begin{bmatrix}
\mathbf R_{\mathrm{ref}} & \mathbf t_{\mathrm{ref}}\\
\mathbf 0^{\mathrm T} & 1
\end{bmatrix}.
$$

在 cam1 坐标侧左乘确定性欧拉角扰动，并对平移直接加偏差：

$$
\mathbf R_{\mathrm{seed}}
=
\mathbf R_z(5^\circ)
\mathbf R_y(-3^\circ)
\mathbf R_x(2^\circ)
\mathbf R_{\mathrm{ref}},
$$

$$
\mathbf t_{\mathrm{seed}}
=
\mathbf t_{\mathrm{ref}}
+
\begin{bmatrix}
0.020 & -0.010 & 0.005
\end{bmatrix}^{\mathrm T}\ \mathrm m.
$$

由旋转测地距离和欧氏距离计算，实际注入误差为：

$$
\Delta\theta
=
\cos^{-1}\!\left(
\frac{\operatorname{tr}(\mathbf R_{\mathrm{seed}}
\mathbf R_{\mathrm{ref}}^{\mathrm T})-1}{2}
\right)
=6.20599996^\circ,
$$

$$
\Delta t
=
\left\|\mathbf t_{\mathrm{seed}}-\mathbf t_{\mathrm{ref}}\right\|_2
=22.9128785\ \mathrm{mm}.
$$

两路 $\mathbf I,\mathbf D$ 从冻结基准逐值复制，没有加入噪声。

## 4. 测试矩阵

| 组别 | 初始外参 | 策略 | `freeze_intrinsics` | 目的 |
|---|---|---|---:|---|
| A | 冻结基准精确值 | `direct` | `true` | 正常起点控制组 |
| B | $6.206^\circ/22.913\ \mathrm{mm}$ 扰动 | `refine` | `true` | 验证 full-batch 外参/pose 初始化和最终增量 |
| C | 同 B | `direct` | `true` | 验证跳过前置 full-batch 后的最终增量吸引域 |
| D | 冻结基准精确值 | `direct` | `false` | 确认默认联合优化仍会更新 $\mathbf I,\mathbf D$ |

三组固定内参运行都启用了正常的异常点移除和最终过滤。因此 K/D 最终不变也同时验证了
“删除角点后重建 batch”不会意外重新激活相机参数。

## 5. 真实数据首先暴露的问题

### 5.1 修复前现象

第一次运行 A 和 B 时，两组都只接受了 2/240 个视图，随后 baseline 和重投影统计变为
NaN；最终可观性检查报告 `rank 5/6` 并以状态 2 退出。由于精确 baseline 控制组同样
失败，可以排除外参噪声或变换方向是根因。

### 5.2 根因

固定两台相机的 K/D 后，calibration group 从原来的 24 维变为只含 baseline 的 6 维。
早期某个 batch 可能对该 6 维 block 没有信息，此时消元后的奇异值谱可以为：

$$
\boldsymbol\sigma
=
\begin{bmatrix}
0&0&0&0&0&0
\end{bmatrix}^{\mathrm T}.
$$

原生 `estimateNumericalRank()` 的倒序循环停止在索引 1，不检查索引 0，所以把全零谱
错误报告为 rank 1。随后的 SVD 求解会计算：

$$
\frac{1}{\sigma_0}=\frac{1}{0},
$$

从而把 NaN 写入 baseline。这个原生边界条件在所有相机参数均 active 时通常不会触发，
但固定 K/D 后会稳定暴露。

### 5.3 修复

数值秩现在从最大奇异值开始计数，并允许合法的 rank 0；rank 0 的谱间隙使用无穷大
哨兵，SVD 更新使用空可观子空间，因此该 batch 对 baseline 产生零更新，而不是
`1/0`。对任意正常正秩谱，秩、谱间隙和更新公式完全不变。

新增 `incremental_rank_tests` 覆盖：

- 全零 6 维谱必须返回 rank 0；
- 部分秩谱的阈值行为不变；
- 正奇异值满秩谱仍返回满秩；
- rank 0 和满秩的谱间隙均使用既定无穷大哨兵。

## 6. 最终结果

### 6.1 参数、秩、耗时和内存

| 组别 | 状态 | K/D 最大绝对变化 | 相对全量基线旋转差 | 相对全量基线平移差 | Hard rank | Operational rank | 最终使用视图 | wall | 最大 RSS |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| A：精确 `direct`、冻结 | 成功 | 0 | $0.00334073^\circ$ | $0.077221\ \mathrm{mm}$ | 6/6 | 6/6 | 40/240 | 48.59 s | 483.73 MiB |
| B：带噪 `refine`、冻结 | 成功 | 0 | $0.00334072^\circ$ | $0.077221\ \mathrm{mm}$ | 6/6 | 6/6 | 40/240 | 45.45 s | 523.87 MiB |
| C：带噪 `direct`、冻结 | 成功 | 0 | $0.00334073^\circ$ | $0.077221\ \mathrm{mm}$ | 6/6 | 6/6 | 40/240 | 47.07 s | 482.81 MiB |
| D：精确 `direct`、不冻结 | 成功 | 内参最大 2.16458；畸变最大 0.00196259 | $0.0304063^\circ$ | $0.483179\ \mathrm{mm}$ | 24/24 | 24/24 | 125/240 | 228.21 s | 635.91 MiB |

组 D 的 K/D 差异包含像素和模型原生单位，不能合并解释为像素误差。它只用于证明
`freeze_intrinsics: false` 仍走原有的全参数 active 路径。不同 active 维数会改变信息
增益筛选和最终保留视图数量，因此组 D 的 125 个视图不能与冻结组的 40 个视图直接
作为数据质量优劣比较。

三组冻结运行均报告：

```text
cameras.cam0.intrinsics   active: false, active_dimension: 0
cameras.cam0.distortion   active: false, active_dimension: 0
cameras.cam1.intrinsics   active: false, active_dimension: 0
cameras.cam1.distortion   active: false, active_dimension: 0
camera_chain.cam1         rotation 3 + translation 3 active
```

最终 nuisance pose 为 240 维且满秩；baseline calibration block 为 6 维且 hard、
operational 两种判据都满秩。条件数为：

$$
\kappa(\mathbf H_{\mathrm{reduced}})=89.9930,
\qquad
\kappa(\mathbf J_{\mathrm{equivalent}})=9.48647.
$$

### 6.2 不同策略的终值一致性

三组冻结结果之间的最大差异为：

| 比较 | 旋转矩阵 Frobenius 差 | 平移向量差 | $4\times4$ 矩阵最大绝对差 |
|---|---:|---:|---:|
| B `refine` vs A 精确 `direct` | $7.69\times10^{-15}$ | $6.42\times10^{-16}\ \mathrm m$ | $4.44\times10^{-15}$ |
| C 带噪 `direct` vs A 精确 `direct` | $7.83\times10^{-16}$ | $1.41\times10^{-16}\ \mathrm m$ | $4.44\times10^{-16}$ |
| B `refine` vs C 带噪 `direct` | $6.92\times10^{-15}$ | $5.31\times10^{-16}\ \mathrm m$ | $4.00\times10^{-15}$ |

三份 `results.txt` 的 SHA-256 完全相同：

```text
706435e2740eb74993b7c6bedfdfe5015e12b45025545596338911f549d8708d
```

这说明本数据和本级别噪声下，`refine` 与 `direct` 最终进入了同一个数值解。不能据此
推断所有更大误差都适合 `direct`；`refine` 仍会多运行一次固定 K/D 下的 full-batch
外参/pose 修正，面对未知质量的历史外参时更稳妥。

### 6.3 外参噪声恢复量

带噪 seed 相对全量冻结基线的误差为 $6.206^\circ/22.913\ \mathrm{mm}$；终值相对
该基线只剩 $0.00334^\circ/0.0772\ \mathrm{mm}$。按误差幅值计算，分别消除了：

$$
99.9462\%\quad\text{旋转误差},
$$

$$
99.6630\%\quad\text{平移误差}.
$$

剩余差异主要来自本轮仅使用 60 秒子集，而冻结基线使用全量 Bag。

### 6.4 重投影误差

三组冻结运行的最终重投影统计完全一致：

| 相机 | 均值 $(x,y)$ | 标准差 $(x,y)$ | 二维 RMS |
|---|---:|---:|---:|
| cam0 | $(-0.000085,\ 0.000073)$ px | $(0.103927,\ 0.097893)$ px | 0.142772 px |
| cam1 | $(-0.000132,\ 0.000067)$ px | $(0.104411,\ 0.092797)$ px | 0.139689 px |

二维 RMS 按下式由报告中的均值和标准差计算：

$$
\operatorname{RMS}_{2D}
=
\sqrt{\mu_x^2+\mu_y^2+\sigma_x^2+\sigma_y^2}.
$$

## 7. 结论

1. `freeze_intrinsics: true` 的实际语义已经验证：两路投影内参和畸变参数逐值不变，
   相机标定物理参数中只有双目 baseline active；每帧 target pose 仍正常联合求解。
2. 异常点移除累计删除 1527 个角点并重建 batch 后，K/D 最大变化仍严格为 0，说明
   固定策略贯穿了最终增量和重建路径。
3. 本次确定性的 $6.206^\circ/22.913\ \mathrm{mm}$ 外参扰动在 `refine`、`direct`
   两种策略下都恢复到同一个解；未知或更差外参仍优先推荐 `refine`。
4. 固定参数暴露了原生增量求解器的 rank-0 边界缺陷。修复后精确、带噪和两种策略均
   成功，baseline 的 hard/operational rank 都为 6/6。
5. 正常正秩求解公式和默认未冻结路径没有改变；`freeze_intrinsics` 省略或设为 `false`
   时仍联合优化 $\mathbf I,\mathbf D,\mathbf B,\mathbf P$。
6. 这组 60 秒验证适合回归功能与初值鲁棒性，不应替换既有全量
   `camera_project_4` 冻结性能基线。

## 8. 构建和自动化复验

本轮使用不超过 4 个编译任务：

```bash
cmake --preset project-test
cmake --build --preset project-test --parallel 4
ctest --test-dir build/project-test --output-on-failure

cmake --preset project-release
cmake --build --preset project-release --parallel 4
```

另外运行源码差异审计和格式检查：

```bash
python3 tools/audit_source_delta.py
git diff --check
```
