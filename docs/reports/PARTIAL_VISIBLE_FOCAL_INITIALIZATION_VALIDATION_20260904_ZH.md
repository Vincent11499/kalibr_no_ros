# 部分可见标定板焦距初始化验证报告

验证日期：2026-09-04。

## 1. 目标与接口

本次只新增一个公开 task 参数：

```yaml
calibration:
  focal_initialization_min_visible_corner_ratio: 0.75
```

它只作用于未提供相机内参初值时的 pinhole 系列解析焦距初始化。对单帧观测：

$$
\rho_{\mathrm{visible}}
=
\frac{N_{\mathrm{valid\ corners}}}
     {N_{\mathrm{all\ target\ corners}}}.
$$

参数必须是有限数且满足 $0<\rho_{\min}\leq1$。省略时默认为 `1.0`；该值直接调用
原有单参数实现，不改变 ETHZ 原生初始化的运算路径和数值顺序。显式内参 seed 会跳过
解析焦距初始化，因此不受此参数影响。Camera–IMU task 读取已有 camchain，也不使用该
参数。

当参数小于 `1.0` 时，全局可见比例只是第一道筛选。为避免用短圆弧产生病态焦距，
程序内部还固定检查：

- 每行至少 `min(cols, max(6, ceil(cols/2)))` 个角点；
- 行内角点索引跨度至少覆盖标定板宽度的 50%；
- 至少三条有效行，且行索引跨度至少覆盖标定板高度的 50%；
- 归一化圆拟合必须非退化，拟合残差和圆交点必须有限、稳定；
- 至少产生三个正的焦距候选，并使用 median/MAD 去除异常候选。

这些条件是内部安全门槛，没有继续暴露成 task 超参数。

## 2. 代码和测试覆盖

参数链路为：task 校验 → 隐藏的内部 CLI 参数 → Python 相机初始化器 → Boost.Python
绑定 → C++ 投影模型。当前覆盖 `pinhole-radtan5`、`pinhole-radtan8`、零 skew 和完整
OpenCV fisheye 投影。

验证结果：

- `project-test` 配置、最多四线程编译成功；
- CTest：26 个已启用测试全部通过，0 失败；原有 `aslam_cameras_tests` 仍为 disabled；
- 新增的四个 C++ 初始化测试单独执行，4/4 通过，覆盖部分观测成功、默认路径逐值
  一致、退化直线和非法比例；
- Python 测试覆盖普通及仅畸变 seed 的参数转发、低于可见阈值、退化行，以及
  radtan8 和完整 OpenCV fisheye 的部分观测公共绑定；
- `project-release` 配置、最多四线程编译成功；
- source-delta audit 通过：1565 个冻结文件不变，32 个批准修改；
- `ref/kalibr` 未修改。

## 3. 真实数据条件

数据集：

```text
/mnt/q/File/machine_data/evt3.0-12/mipi_in_stereo_calib_12_0902
```

两次实验均采用：

- ROS2 bag 时间段 `[10, 70]` s；
- 图像频率 `3.0` Hz；
- 双目同步容差 `0.00002` s；
- `information_gain_tolerance: 0.5`；
- `shuffle: true`；
- `window_half_size_px: 7`、`max_displacement_px: 2`；
- detector 32 进程、optimizer 8 线程、每个 detector 的 OpenCV 1 线程；
- 无 `initialization` 段；
- `focal_initialization_min_visible_corner_ratio: 0.75`。

每个结果目录保存了实际输入 `input_task.yaml`。两次均写入新目录，没有使用
`--force`，没有覆盖既有结果。

## 4. 标定结果

| 模型 | cam0 检测 | cam1 检测 | 最终使用 view | 删除离群角点 | cam0 重投影标准差 | cam1 重投影标准差 | baseline |
|---|---:|---:|---:|---:|---:|---:|---:|
| `pinhole-radtan8` | 136/137 | 142/142 | 36/258 | 198 | [0.3270, 0.2911] px | [0.2259, 0.2060] px | 0.381154 m |
| `pinhole-opencv-fisheye` | 136/137 | 142/142 | 36/258 | 147 | [0.3121, 0.3009] px | [0.2321, 0.2083] px | 0.380633 m |

两份输出中的全部数值均有限；双目旋转矩阵满足
$\det(\mathbf R)\approx1$，且最大正交误差不超过 $6\times10^{-15}$。

输出目录：

```text
/mnt/q/File/machine_data/evt3.0-12/output_radtan8_no_init_partial075_3hz_mi05
/mnt/q/File/machine_data/evt3.0-12/output_fisheye_no_init_partial075_3hz_mi05
```

## 5. 与已有结果的交叉检查

现有结果使用了显式初值，且部分实验的频率或角点窗口不同，因此这里只做健全性
交叉检查，不把差异解释为严格的算法精度排名。

- 新 fisheye 与既有 3 Hz 显式初值结果相比：焦距最大差 1.26 px，主点最大差
  5.55 px，baseline 长度差 0.141 mm；
- 新 fisheye 与既有 5 Hz、窗口 7 结果相比：焦距最大差 3.57 px，主点最大差
  3.47 px，baseline 长度差 0.077 mm；
- 新 radtan8 与既有 5 Hz 显式初值结果相比：焦距最大差 2.63 px，主点最大差
  9.67 px，baseline 长度差 0.269 mm。

radtan8 的八个原始系数差异明显，因为 rational radial 模型的分子和分母存在强参数
耦合。按 OpenCV rational 形式定义径向缩放：

$$
s(r)
=
\frac{1+k_1r^2+k_2r^4+k_3r^6}
     {1+k_4r^2+k_5r^4+k_6r^6}.
$$

在归一化半径 $r\in[0,1]$ 上对 10001 个等距采样点检查，新旧 cam0 的最大
$|\Delta s|$ 为 0.001407，cam1 为 0.000495；两份新结果的分母在该区间始终为正。
因此不能只按单个 rational 系数大小判断标定是否发散。但大系数仍表明该数据对八个
径向参数的独立可辨识性较弱，量产使用时应结合去畸变图、视场边缘误差和独立验证集
决定选用 radtan8 还是参数更少的模型。

## 6. 结论

`0.75` 部分可见路径已经在 radtan8 和完整 OpenCV fisheye 上完成无初值、真实 ROS2
双目标定，两次都成功产生数值健全且重投影误差较低的结果。它解决了“没有完整标定板
帧时无法生成焦距 seed”的软件限制，但不会弥补视场覆盖、姿态多样性或参数可观性
不足；默认 `1.0` 仍是保持原生可比性的路径。
