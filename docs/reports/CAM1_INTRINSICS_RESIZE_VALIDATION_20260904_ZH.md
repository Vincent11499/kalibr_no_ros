# cam1 4K 内参迁移到 1080p 详细验证报告

验证日期：2026-09-04。

## 1. 实验问题和最终结论

本实验回答的问题是：cam1 已有的 $3840\times2160$ 内参，在同一幅原图通过
`cv2.resize(..., INTER_AREA)` 缩小为 $1920\times1080$ 后，能否经过确定的参数变换
直接用于低分辨率图像。

最终结论分为三层：

1. 数学层面：radtan8 和完整 OpenCV fisheye 的畸变都定义在归一化坐标域。纯 resize
   不改变归一化畸变参数，只需要对像素尺度的投影参数进行仿射变换。
2. 本次数据层面：基于 20 帧 cam1 图像和 4341 个高低分辨率共同角点观测的聚合统计，
   `opencv_half_pixel` 规则得到支持。它把角点映射 RMS 从 0.4220 px 降至
   0.1984 px，并把固定 4K pose 的重投影 RMS 从约 0.43 px 降至约 0.22 px。
3. 产品层面：该结论只覆盖“同一张 4K 图像执行纯 OpenCV resize”。它不证明相机 ISP
   直接输出的 1080p 模式可以使用相同参数，也不等价于重新完成了一次 1080p 标定。

若要严格匹配本次 OpenCV resize 的像素中心坐标，推荐：

$$
f'_x=0.5f_x,
\qquad
f'_y=0.5f_y,
$$

$$
c'_x=0.5(c_x+0.5)-0.5,
\qquad
c'_y=0.5(c_y+0.5)-0.5.
$$

所有畸变系数保持不变；完整 OpenCV fisheye 的无量纲 `alpha` 也保持不变。简单地将
$f_x,f_y,c_x,c_y$ 全部除以 2 是常见近似，但在本次数据上会留下约
$(-0.25,-0.25)$ px 的系统偏差。该近似是否仍可接受，取决于下游应用容差；它没有
通过本实验采用的 P95 小于 0.5 px 的工程判据。

## 2. 实验比较了什么

本实验不是在比较 radtan8 和 fisheye 哪个模型更好，而是在两个模型上分别重复相同的
内参迁移验证。

| 对比 | 保持不变的内容 | 改变的内容 | 主要目的 | 本次结论 |
|---|---|---|---|---|
| A. 代数投影自检 | 三维射线、模型及畸变参数 | 4K 投影与变换后的 1080p 投影 | 排除公式或代码实现错误 | 四组最大差均为机器精度下的 0；只能说明实现自洽 |
| B. 真实角点映射 | 同一原图、同一角点 ID、同一检测参数 | 简单缩放与半像素缩放 | 不引入 pose，直接识别 resize 的像素坐标约定 | 半像素 RMS/P95 为 0.1984/0.3232 px，明显更好 |
| C. 固定 4K pose 重投影 | 4K 估计的标定板 pose、三维角点及畸变 | 两套 1080p 内参 | 防止低分辨率 pose 吸收错误，验证参数能否直接迁移 | 半像素约 0.22 px，简单缩放约 0.43 px |
| D. 1080p pose 重估 | 转换后的内参和畸变固定 | 只重新估计每帧标定板 pose | 观察 pose 能吸收多少内参迁移误差，并给出参考残差水平 | 两种规则都约 0.20 px，因此该指标不能用于选规则 |
| E. 跨模型重复 | 相同图像、resize、角点和评价流程 | radtan8 与完整 fisheye | 检查结论是否只在一个投影模型上成立 | 两种模型都推荐半像素规则，但不能据此进行模型排名 |

其中 B 和 C 是决定转换规则的主要证据；A 是代码自检，D 是对照组，E 是重复性检查。

## 3. 固定输入与控制变量

### 3.1 数据与标定板

| 项目 | 固定值 |
|---|---|
| ROS2 bag | `/mnt/q/File/machine_data/evt3.0-12/mipi_in_stereo_calib_12_0902` |
| 相机 | 只使用 cam1 |
| topic | `/mipi_image_data_rgb1` |
| 源分辨率 | $3840\times2160$ |
| 目标分辨率 | $1920\times1080$ |
| 图像变换 | 解码为灰度图后执行 `cv2.resize(..., INTER_AREA)` |
| manifest 原配置范围 | 10～70 s、5 Hz |
| manifest 可用同步对 | 79 对；本实验只读取其中 cam1 数据 |
| 固定样本 | 均匀选出的 20 个唯一 cam1 时间戳，不重新抽样 |
| 标定板 | 8×8 AprilGrid，Tag ID 为 `[80, 144)` |
| Tag 参数 | `tagSize=0.065 m`、`tagSpacing=0.3`，实际间隔为 0.0195 m |
| 亚像素窗口 | `window_half_size_px=7` |
| 最大细化位移 | `max_displacement_px=2` |

高分辨率角点不在本实验中重新选择，而是读取固定 manifest 中保存的检测结果。每个
1080p 候选组都重新运行 Kalibr AprilGrid 检测，避免直接把 4K 角点数值缩放后当成
“低分辨率实测值”。

### 3.2 三种角点数量

设第 $i$ 帧的 4K 有效角点 ID 集合为 $\mathcal H_i$，1080p 重新检测得到的集合为
$\mathcal L_i$，真正用于高低分辨率一一比较的集合为：

$$
\mathcal C_i
=
\mathcal H_i\cap\mathcal L_i.
$$

20 帧累计观测数为：

| 集合 | 累计角点观测数 | 用途 |
|---|---:|---|
| $\mathcal H_i$ | 4447 | 估计 4K pose |
| $\mathcal L_i$ | 4750 | 1080p pose 重估 |
| $\mathcal C_i$ | 4341 | mapping、缩放后的源残差、固定 pose 和重估 pose 的公平对比 |

这里的数量是跨帧累计的“角点观测”，同一个 AprilGrid 角点 ID 可以在多帧重复出现，
不能把 4341 理解为 4341 个相互独立的物理角点。resize 的抗混叠可能让某些边缘在低
分辨率上更容易通过检测或亚像素细化门限，因此 1080p 的累计检测数大于 4K 并不矛盾，
也不表示低分辨率包含更多原始信息。

### 3.3 图像覆盖范围

4447 个 4K 角点的外接框为：

$$
u\in[278.2,3016.3]\ \mathrm{px},
\qquad
v\in[4.4,1974.3]\ \mathrm{px}.
$$

归一化到像素坐标范围后约为：

$$
u:7.2\%\sim78.6\%,
\qquad
v:0.2\%\sim91.4\%.
$$

这只是所有角点的 bounding box，不表示框内均匀覆盖。最右侧约 21% 没有角点观测，
左侧约 7% 和底部约 8.6% 也没有覆盖，因此本实验不能给出全视场保证。

### 3.4 两套 4K 源标定

| 模型 | cam1 源重投影误差标准差 x/y | Hard rank | Operational rank | 源结果状态 |
|---|---:|---:|---:|---|
| radtan8 | 0.249740/0.258505 px | 30/30 | 21/30 | full rank、weakly observable |
| 完整 OpenCV fisheye | 0.249168/0.236045 px | 24/24 | 22/24 | full rank、weakly observable |

radtan8 来源为 `output_radtan8_init_mi05/calibration.yaml`，fisheye 来源为
`output_5hz_w7/calibration.yaml`。两次源标定的模型维数、进入最终系统的误差项数量、
初始化和信息筛选配置并不完全相同，所以本实验不能用 0.2234 px 与 0.2268 px 的细小
差异判断模型优劣。转换参数还会继承各自 4K 源标定中的系统误差和弱可观方向。

## 4. 参数转换原理

### 4.1 统一成像链

设三维射线归一化坐标为：

$$
\mathbf q
=
\begin{bmatrix}x&y\end{bmatrix}^{\mathrm T}.
$$

相机模型先在归一化域中执行畸变：

$$
\mathbf q_d
=
\mathcal D(\mathbf q;\boldsymbol\delta),
$$

然后用像素投影参数映射到图像：

$$
\mathbf p
=
\begin{bmatrix}u&v&1\end{bmatrix}^{\mathrm T}
=
\mathbf K
\begin{bmatrix}x_d&y_d&1\end{bmatrix}^{\mathrm T}.
$$

resize 只对最终像素坐标 $\mathbf p$ 做仿射变换，不改变 $\mathbf q$ 或
$\mathbf q_d$。因此有：

$$
\mathbf p'
=
\mathbf H\mathbf p
=
\mathbf H\mathbf K
\begin{bmatrix}x_d&y_d&1\end{bmatrix}^{\mathrm T},
$$

从而：

$$
\boxed{\mathbf K'=\mathbf H\mathbf K}.
$$

这就是“缩放像素参数、保持归一化畸变参数不变”的根本原因。

### 4.2 radtan8 为什么只缩放四个内参

radtan8 的严格参数顺序是：

$$
\boldsymbol\delta
=
\begin{bmatrix}
k_1&k_2&p_1&p_2&k_3&k_4&k_5&k_6
\end{bmatrix}^{\mathrm T}.
$$

令 $r^2=x^2+y^2$，rational 径向因子为：

$$
L(r)
=
\frac{1+k_1r^2+k_2r^4+k_3r^6}
     {1+k_4r^2+k_5r^4+k_6r^6}.
$$

畸变后的归一化坐标为：

$$
x_d
=
xL(r)+2p_1xy+p_2(r^2+2x^2),
$$

$$
y_d
=
yL(r)+p_1(r^2+2y^2)+2p_2xy.
$$

最终投影为：

$$
u=f_xx_d+c_x,
\qquad
v=f_yy_d+c_y.
$$

$k_i,p_i$ 作用在无量纲的 $x,y,r$ 上，所以 resize 时全部保持不变；只有单位为 pixel
的 $f_x,f_y,c_x,c_y$ 需要转换。

### 4.3 fisheye 为什么畸变和 `alpha` 都不缩放

完整 OpenCV fisheye 使用：

$$
r=\sqrt{x^2+y^2},
\qquad
\theta=\arctan(r),
$$

$$
\theta_d
=
\theta
\left(
1+k_1\theta^2+k_2\theta^4+k_3\theta^6+k_4\theta^8
\right).
$$

当 $r>0$ 时：

$$
x_d=\frac{\theta_d}{r}x,
\qquad
y_d=\frac{\theta_d}{r}y.
$$

本项目完整模型的像素投影为：

$$
u=f_x(x_d+\alpha y_d)+c_x,
\qquad
v=f_yy_d+c_y.
$$

$k_1\ldots k_4$ 和 $\alpha$ 都是无量纲量，因此保持不变。OpenCV 矩阵中的实际 skew
元素为：

$$
K_{01}=f_x\alpha.
$$

虽然 $\alpha$ 不变，但 $f_x$ 缩放后，$K_{01}$ 会自动按相同比例缩放。

### 4.4 OpenCV 半像素公式如何得到

对缩放率 $s_x$，OpenCV resize 的像素中心逆采样关系可以写为：

$$
u_{\mathrm{src}}
=
\frac{u_{\mathrm{dst}}+0.5}{s_x}-0.5.
$$

解出目标坐标：

$$
u_{\mathrm{dst}}
=
s_x(u_{\mathrm{src}}+0.5)-0.5.
$$

纵向同理。因此像素仿射矩阵为：

$$
\mathbf H_{\mathrm{half}}
=
\begin{bmatrix}
s_x&0&0.5s_x-0.5\\
0&s_y&0.5s_y-0.5\\
0&0&1
\end{bmatrix}.
$$

结合 $\mathbf K'=\mathbf H\mathbf K$ 可得：

$$
f'_x=s_xf_x,
\qquad
f'_y=s_yf_y,
$$

$$
c'_x=s_x(c_x+0.5)-0.5,
\qquad
c'_y=s_y(c_y+0.5)-0.5.
$$

作为对照，简单缩放假设 $\mathbf H_{\mathrm{direct}}=\operatorname{diag}(s_x,s_y,1)$，
所以 $c'_x=s_xc_x,c'_y=s_yc_y$。本次 $s_x=s_y=0.5$，两种规则的焦距完全相同，
只在主点上相差：

$$
\Delta c_x=\Delta c_y=-0.25\ \mathrm{px}.
$$

如果实际流程是先从 4K 图像左上角裁掉 $(o_x,o_y)$，再执行 resize，则应改为：

$$
c'_x=s_x(c_x-o_x+0.5)-0.5,
$$

$$
c'_y=s_y(c_y-o_y+0.5)-0.5.
$$

当前实验中 $o_x=o_y=0$，没有验证 crop、letterbox 或非等比例 ISP 变换。

## 5. 实际执行流程

```text
固定 manifest 中的 20 个 cam1 时间戳
                 │
                 ├──读取已保存的 4K 角点──按模型估计 4K 标定板 pose
                 │                              │
ROS2 bag 解码 4K 灰度图                         │（保持固定）
                 │                              │
                 └──INTER_AREA resize──1080p 重新检测角点
                                                │
                           ┌────────────────────┼────────────────────┐
                           │                    │                    │
                       角点映射             固定 pose 投影       1080p pose 重估
                           │                    │                    │
                           └────────共同角点 4341 个统一统计────────┘
```

对每个模型和每种转换规则，工具执行以下步骤：

1. 从源标定 YAML 读取 cam1 的 4K 内参、畸变、分辨率和模型类型；
2. 分别生成 `direct_scale`、`opencv_half_pixel` 两套 1080p 内参，畸变参数不变；
3. 建立 4K 相机几何和两套 1080p 相机几何；
4. 对归一化平面中的 $33\times21=693$ 条射线进行代数投影自检，采样范围为
   $x\in[-0.8,0.8]$、$y\in[-0.45,0.45]$，并经过可见性检查；
5. 按 manifest 时间戳读取 4K 原图，转换为灰度图后使用 `INTER_AREA` resize；
6. 每个候选组在 1080p 图像上独立运行 AprilGrid 检测；
7. 按全局角点 ID 建立 $\mathcal C_i$，计算映射、固定 pose 和 pose 重估残差；
8. 输出角点级 CSV、逐帧统计、推荐参数和预测叠加图。

四组候选均为 20/20 帧检测成功、20/20 帧 4K pose 成功、20/20 帧 1080p pose 成功。

## 6. 指标定义

### 6.1 角点映射残差

对共同角点 $j$，定义：

$$
\mathbf e^{\mathrm{map}}_{ij}
=
\mathbf u^{1080}_{ij,\mathrm{detect}}
-
\mathcal A
\left(
\mathbf u^{4K}_{ij,\mathrm{detect}}
\right),
$$

其中 $\mathcal A$ 是正在测试的 direct 或 half-pixel 像素变换。它完全不使用标定板
pose，因此可以直接检查图像 resize 后的实际角点坐标更符合哪种像素约定。

均值 $\overline e_x,\overline e_y$ 用来识别系统方向偏差。本实验残差定义是“1080p
实测角点减去 4K 角点的理论变换位置”，所以负值表示理论位置整体偏右或偏下。

### 6.2 二维 RMS、P95 和最差值

对 $N$ 个二维残差 $\mathbf e_n=[e_{x,n},e_{y,n}]^{\mathrm T}$：

$$
\operatorname{RMS}_{2D}
=
\sqrt{
\frac{1}{N}
\sum_{n=1}^{N}
\left(e_{x,n}^2+e_{y,n}^2\right)
}.
$$

该值是二维欧氏范数的 RMS，不是 x/y 单轴标准差。P95 表示 95% 的角点残差范数不超过
该数值；median 表示典型角点；max 表示单个最差角点。

聚合 RMS 按角点加权，因此检测角点多的帧权重更大。报告另外给出逐帧 RMS 和最差帧，
用于避免总体数字掩盖局部退化。不同角点来自同一图像和同一标定板，也高度相关，不能
把 4341 个角点当成 4341 个独立统计样本来构造虚假的显著性结论。

### 6.3 固定 4K pose 残差

定义 ${}^{C}_{T}\mathbf T_i$ 为第 $i$ 帧从标定板坐标系到相机坐标系的变换：

$$
{}^{C}\mathbf p
=
{}^{C}_{T}\mathbf T_i {}^{T}\mathbf p.
$$

先用 4K 模型和 4K 角点估计 ${}^{C}_{T}\mathbf T_i$，随后保持它不变：

$$
\mathbf e^{\mathrm{fixed}}_{ij}
=
\pi
\left(
\mathbf K_{1080},
\boldsymbol\delta,
{}^{C}_{T}\mathbf T_i {}^{T}\mathbf P_j
\right)
-
\mathbf u^{1080}_{ij,\mathrm{detect}}.
$$

如果 1080p 内参转换正确，固定同一个 4K 估计 pose 后仍应能解释低分辨率观测。如果转换
错误，该误差不能被重新估计 pose 吸收，所以它是本实验最重要的模型相关指标。

源 4K 重投影残差定义为：

$$
\mathbf e^{\mathrm{source}}_{ij}
=
\pi
\left(
\mathbf K_{4K},
\boldsymbol\delta,
{}^{C}_{T}\mathbf T_i {}^{T}\mathbf P_j
\right)
-
\mathbf u^{4K}_{ij,\mathrm{detect}}.
$$

工具输出的 `fixed_minus_scaled_source_reprojection` 与 mapping 项应满足以下闭环关系，
可供独立审计：

$$
\mathbf e^{\mathrm{fixed}}_{ij}
-
\mathbf S\mathbf e^{\mathrm{source}}_{ij}
=
-\mathbf e^{\mathrm{map}}_{ij},
$$

其中 $\mathbf S=\operatorname{diag}(s_x,s_y)$。它把“角点坐标变换”和“模型重投影”两条
计算链连接起来，可用于审计残差符号和参数缩放是否一致。

### 6.4 1080p pose 重估残差

该对照组固定 $\mathbf K_{1080}$ 和 $\boldsymbol\delta$，只用 4750 个低分辨率角点重新
估计每帧 ${}^{C}_{T}\mathbf T_i$。为与固定 pose 公平比较，最终表格只在 4341 个共同
角点上统计残差。

pose 有旋转和平移自由度，能够吸收一部分全局主点偏差。因此它表示“允许位姿重新适应
后的参考残差水平”，不是内参迁移正确性的独立证据，也不是重新标定了 1080p 内参。

## 7. 分项对比结果与解释

### 7.1 对比 A：代数投影自检

对每条归一化射线，分别计算：

$$
\mathbf u_{1080}^{\mathrm{model}}
=
\pi(\mathbf K_{1080},\boldsymbol\delta,\mathbf q),
$$

$$
\mathbf u_{1080}^{\mathrm{expected}}
=
\mathcal A
\left(
\pi(\mathbf K_{4K},\boldsymbol\delta,\mathbf q)
\right).
$$

| 模型 | 规则 | 射线数 | 最大差 |
|---|---|---:|---:|
| radtan8 | direct | 693 | 0.0 px |
| radtan8 | half-pixel | 693 | 0.0 px |
| fisheye | direct | 693 | 0.0 px |
| fisheye | half-pixel | 693 | 0.0 px |

目的和结论：该检查证明两套候选规则都被代码正确实现，并且畸变参数保持不变时模型
内部是代数自洽的。但每个候选的“期望值”本来就由自身公式生成，所以四组均为 0
不能说明哪一种更符合 `cv2.resize`；规则选择必须看真实角点。

### 7.2 对比 B：真实角点映射

| 规则 | 共同角点 | 均值 x/y | RMS | Median | P95 | 最大单点 | 最差帧 RMS |
|---|---:|---:|---:|---:|---:|---:|---:|
| 简单乘 0.5 | 4341 | -0.2581/-0.2693 px | 0.4220 px | 0.3996 px | 0.5766 px | 1.8593 px | 0.6195 px |
| OpenCV 半像素 | 4341 | -0.0081/-0.0193 px | 0.1984 px | 0.1265 px | 0.3232 px | 1.8269 px | 0.4799 px |

角点映射不依赖相机模型，所以 radtan8 和 fisheye 的这组数字完全相同是预期现象。

对比结论：

- 半像素规则将映射 RMS 降低约 53.0%；
- P95 降低约 43.9%；
- 简单缩放的均值非常接近理论上的 $(-0.25,-0.25)$ px；
- 半像素规则基本消除了该系统偏差，剩余误差主要包括 resize 插值、AprilTag 检测和
  亚像素细化差异。

因此，若目标是匹配本次 `INTER_AREA` resize 的像素坐标，半像素规则得到真实图像
直接支持。

### 7.3 对比 C：固定 4K pose 重投影

| 模型 | 缩放后的源 4K 残差 RMS | 简单缩放固定 pose RMS | 半像素固定 pose RMS | 半像素相对简单缩放改善 | 半像素最差帧 RMS | 半像素最大单点 |
|---|---:|---:|---:|---:|---:|---:|
| radtan8 | 0.1857 px | 0.4332 px | 0.2234 px | 48.4% | 0.5087 px | 1.8461 px |
| fisheye | 0.1879 px | 0.4358 px | 0.2268 px | 48.0% | 0.5002 px | 1.8401 px |

“缩放后的源 4K 残差”是把 4K 模型原有残差乘以 0.5 后得到的参考水平。半像素固定
pose 结果只比这个参考高约 0.038～0.039 px；简单缩放则因为主点系统偏差升至约
0.43 px。

对比结论：在不允许 pose 重新适应的条件下，半像素转换仍接近源模型的残差水平，说明
它不仅拟合了角点坐标变换，也保持了“4K 模型—物理 pose—1080p 观测”之间的一致性。

最差帧都是 sample 11。其 RMS 略高于 0.5 px，而聚合 RMS 明显低于 0.5 px；这不违反
本实验按聚合 RMS 定义的门限，但说明不能把“整体通过”解释为每帧、每点都小于
0.5 px。如果生产验收要求逐帧小于 0.5 px，需要单独制定并执行逐帧门限。

### 7.4 对比 D：为什么 1080p pose 重估不能选规则

| 模型 | 规则 | 固定 4K pose RMS | 1080p pose 重估后共同角点 RMS | 两者差值 |
|---|---|---:|---:|---:|
| radtan8 | 简单乘 0.5 | 0.4332 px | 0.2012 px | 0.2320 px |
| radtan8 | 半像素 | 0.2234 px | 0.2009 px | 0.0225 px |
| fisheye | 简单乘 0.5 | 0.4358 px | 0.2035 px | 0.2323 px |
| fisheye | 半像素 | 0.2268 px | 0.2039 px | 0.0229 px |

只看最后一列重估结果，direct 与 half-pixel 都约为 0.20 px，几乎无法区分。这不是
两种转换同样正确，而是重估后的 pose 吸收了简单缩放造成的主点平移误差。

对比结论：

- 简单缩放的固定 pose 与重估 pose 相差约 0.232 px，存在明显的可吸收系统误差；
- 半像素规则只相差约 0.023 px，已经接近该分辨率下重估 pose 的参考水平；
- 因而不能用“重新拟合 pose 后误差也很小”证明内参转换正确。固定 pose 对比是必要的。

### 7.5 对比 E：radtan8 与 fisheye 重复验证

| 模型 | 推荐规则 | 角点映射 RMS | 固定 pose RMS | 重估 pose RMS | 本实验工程判定 |
|---|---|---:|---:|---:|---|
| radtan8 | half-pixel | 0.1984 px | 0.2234 px | 0.2009 px | 通过 |
| fisheye | half-pixel | 0.1984 px | 0.2268 px | 0.2039 px | 通过 |

对比结论：像素坐标变换规则不依赖具体畸变模型，两种模型都支持 half-pixel。但是两套
源标定不是严格受控的模型 A/B 实验，且参数维数和可观性不同，所以 0.0034 px 的固定
pose 差值没有模型排名意义。

## 8. 工程门限与总判定

本实验使用以下门限：

| 检查 | 门限 | 用途 |
|---|---:|---|
| 检测、4K pose、1080p pose 成功率 | 20/20 | 避免只对容易成功的帧报告结果 |
| 代数投影最大差 | $<10^{-9}$ px | 捕获公式和实现错误 |
| 真实角点映射 P95 | $<0.5$ px | 限制绝大多数角点的坐标迁移误差 |
| 固定 4K pose 聚合 RMS | $<0.5$ px | 限制不重估 pose 时的模型迁移误差 |

这些值是本次实验的工程容差，不是统计置信区间，也不是行业统一的相机精度标准。

| 候选 | 20/20 全成功 | 代数检查 | Mapping P95 | 固定 pose 聚合 RMS | 总判定 |
|---|---|---|---|---|---|
| radtan8 direct | 通过 | 通过 | 0.5766 px，未通过 | 0.4332 px，通过 | 未通过 |
| radtan8 half-pixel | 通过 | 通过 | 0.3232 px，通过 | 0.2234 px，通过 | 通过 |
| fisheye direct | 通过 | 通过 | 0.5766 px，未通过 | 0.4358 px，通过 | 未通过 |
| fisheye half-pixel | 通过 | 通过 | 0.3232 px，通过 | 0.2268 px，通过 | 通过 |

## 9. 推荐的 1920×1080 参数

### 9.1 radtan8

```yaml
camera_model: pinhole
distortion_model: radtan8
resolution: [1920, 1080]
intrinsics: [1181.4081151293556, 1182.2459666336356,
             949.080243557361, 574.022094684965]
distortion_coeffs: [-0.03387416646859061, 2.820223690151636,
                    -0.0002046555023107088, -0.00017604858652260457,
                    0.3919510145340939, 0.3818108661654195,
                    2.766627369119955, 1.5205862240566517]
```

作为对照，简单缩放得到的主点是
`[949.330243557361, 574.272094684965]`，恰好比推荐值各大 0.25 px。

### 9.2 完整 OpenCV fisheye

```yaml
camera_model: pinhole_opencv_fisheye
distortion_model: opencv_fisheye
resolution: [1920, 1080]
intrinsics: [1184.527020361704, 1184.8331603306171,
             947.2676747217236, 570.7106737938465,
             -0.0002594685359216402]
distortion_coeffs: [-0.07998069898479412, -0.1004873708160048,
                    0.17527742094867615, -0.10377456102222345]
```

最后一个内参是无量纲 `alpha`，未缩放。简单缩放得到的主点是
`[947.5176747217236, 570.9606737938465]`，同样比推荐值各大 0.25 px。

## 10. 结果文件与复现

### 10.1 结果目录

```text
/mnt/q/File/machine_data/evt3.0-12/output_cam1_intrinsics_resize_4k_to_1080p_20260904
```

其中：

- `report.md`：由验证工具生成的结果摘要；
- `summary.yaml`：四组候选的聚合、逐帧指标和工程判定；
- `recommended_parameters.yaml`：两套推荐的 1080p 参数；
- `corner_residuals.csv`：$4341\times2\times2=17364$ 条角点明细；
- `overlays/`：每组 5 张，共 20 张检测/预测叠加图；红点是 1080p 实测角点，绿色十字
  是固定 4K pose 和转换内参的预测。

### 10.2 实际复现命令

从仓库根目录运行；下面给出的示例输出目录必须事先不存在：

```bash
env \
  PYTHONPATH=build/project-release/python \
  LD_LIBRARY_PATH=build/project-release/lib \
  MPLCONFIGDIR=/tmp/kalibr-cam1-resize-matplotlib \
  python3 tools/validate_intrinsics_resize.py \
  --manifest /mnt/q/File/machine_data/evt3.0-12/output_5hz_w7/stereo_sample_manifest.yaml \
  --target /mnt/q/File/machine_data/aprilgrid.yaml \
  --radtan8-calibration /mnt/q/File/machine_data/evt3.0-12/output_radtan8_init_mi05/calibration.yaml \
  --fisheye-calibration /mnt/q/File/machine_data/evt3.0-12/output_5hz_w7/calibration.yaml \
  --camera-index 1 \
  --target-width 1920 \
  --target-height 1080 \
  --window-half-size-px 7 \
  --max-displacement-px 2 \
  --visualization-samples 5 \
  --output-dir /tmp/cam1_resize_validation_candidate
```

本次实验没有启用专门的 profile 计时，因此不把交互运行观察到的 wall time 当作性能
结论。该实验只评价几何和数值迁移。

### 10.3 版本与输入哈希

- 仓库基准 HEAD：`c83f083c113b91c6a9cc05f9eae64916ee09aa52`；
- 验证工具当次工作区 SHA256：
  `63eba88b502d4af6ce164caf5ad3a34bacc39d0f70ec1e6bb8838a7fdc74c56b`；
- OpenCV：`4.5.4`；
- manifest SHA256：
  `4a2e51b4b456cde8ac9ccd366d45b57fbb4614b900e35ea44d65b5746a2bfbf9`；
- AprilGrid YAML SHA256：
  `3205958a1983f13929cf5bc028814e3eac5b06336fe8d4e27bf62c3cfc5a3583`；
- radtan8 源结果 SHA256：
  `65c7d1434ce62f007b2187516ce8b707d68df1a496a25f911cb7eeb01c1efdb3`；
- fisheye 源结果 SHA256：
  `4b55b5bdf83c05d5ba7b5a0d0b117cc3f1935df10767ab4ca511cac02d280aba`；
- 最终 `summary.yaml` SHA256：
  `e7c2c1e93a0ecc6748503292476e2e4cb0088d1868b370910ad49437195260be`；
- 最终 `corner_residuals.csv` SHA256：
  `51c56377312437f32484e146ab58cea5f85a26d684c3d9778fc14b4051b8684c`。

验证工具和本报告在记录上述 HEAD 时仍是未提交的工作区文件，因此复现应以工具 SHA256
为准；不能只依赖基准 commit 推断工具内容。

## 11. 适用边界和后续建议

### 11.1 当前结论覆盖的情况

- 同一物理相机、同一焦距和对焦状态；
- 4K 图像已经形成后，使用 OpenCV `INTER_AREA` 纯软件缩小；
- 分辨率严格从 $3840\times2160$ 变为 $1920\times1080$；
- 没有 crop、letterbox、旋转、数字防抖或预去畸变。

### 11.2 当前结论不覆盖的情况

- sensor binning、subsampling 或不同 readout ROI；
- ISP 裁剪、缩放、畸变校正、EIS 或未知像素坐标变换；
- 4K 与 1080p 使用不同焦距、对焦位置或镜头状态；
- 右侧未覆盖区域和真正的全视场精度；
- 4K 源内参本身是否为绝对真值；
- radtan8 与 fisheye 哪个模型更适合该镜头。

### 11.3 如果要升级为产品验收

建议按以下顺序扩展：

1. 使用独立于标定数据的留出序列重复本实验；
2. 补充图像右侧和四角覆盖，按图像半径/位置统计残差；
3. 直接采集相机真实 1080p 输出，与软件 resize 结果逐点比较；
4. 如果真实模式含 crop，使用带 $(o_x,o_y)$ 的通用主点公式；
5. 建立逐帧 RMS、P95 和最大误差的产品门限；
6. 需要计量级统计时，以帧为重采样单位计算置信区间，而不是把同帧相关角点当成独立
   样本；
7. 可补充“4K 先去畸变再 resize”与“先 resize 再用转换内参去畸变”的图像差异，但
   应注意两条链包含的插值次数不同，不能预期逐像素完全相等。

当前可以用于工程的结论是：对于这批 cam1 图像的纯 OpenCV 0.5 倍 resize，radtan8
和完整 OpenCV fisheye 都应采用半像素主点变换，畸变参数不变；在进入真实 ISP 1080p
产品模式前，还需要用该模式的实际图像做一次独立验证。
