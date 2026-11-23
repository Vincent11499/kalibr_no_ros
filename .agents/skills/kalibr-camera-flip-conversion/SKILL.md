---
name: kalibr-camera-flip-conversion
description: Convert or validate calibration parameters for this project's manual-confirmed IMX678 horizontal-and-vertical camera flip configuration. Use for EEPROM or calibrated K/D orientation transfer; do not apply these constants to arbitrary sensors or a single-axis mirror.
---

# 相机双翻转参数转换

先读仓库 [内参翻转规则](../../../docs/CAMERA_FLIP_CONVERSION_ZH.md)，该文档是当前规则来源。

## 固定规则

用户已根据手册确认当前 IMX678 模组读出配置。源与目标相差一次水平、垂直同时翻转时：

```text
fx' = fx
fy' = fy
cx' = 3856 - cx
cy' = 2184 - cy
```

3856、2184 直接使用，不减 1，不按输出宽高重新计算，不叠加历史拟合偏移。
当前输出分辨率仍是 3840×2160。每目按源参数方向与目标图像状态判定，同状态不转换，
相差双翻转才转换一次；正反向使用同一公式。参数中的焦距保持正值。

## 畸变参数

- `pinhole-equi`、径向 KB4、`pinhole-opencv-fisheye`：四个径向系数不变；
  完整 OpenCV fisheye 的投影 `alpha/skew` 也不变。
- `pinhole-radtan`、`radtan5`、`radtan8`、`omni-radtan`：径向系数不变，`p1/p2` 同时取反，
  保持原系数顺序和维数。
- `pinhole-fov`：`w` 不变；`omni-none/eucm-none/ds-none` 的 D 仍为空；
  对应投影形状参数 `xi/alpha/beta` 不变，按模型字段定位焦距和主点。
- 不按 D 的长度猜模型；未列模型先核对投影定义。

## 验证和输出

只执行用户要求的转换或评价，不因套用规则自动重跑标定或覆盖 EEPROM。
需要验证时固定转换后的 K/D，以已有角点优先计算重投影 RMS/P95/MAX 和去畸变直线度；
没有可复用角点且用户要求真实数据验证时再检测。
记录源参数、目标图像配置、模型及转换后参数，历史报告不得改写为新规则的实测结果。
历史实验脚本中的旧转换和偏移拟合不作为后续转换入口。
