---
name: kalibr-stereo-tag-distance
description: Measure or report the 3-D center distance between two tag36h11 AprilTags in this repository's directory-format stereo captures using fixed pinhole-equi K/D/T. Use for per-pair distance CSV and annotated detection images; do not use to calibrate cameras or infer AprilGrid board geometry.
---

# 双目 AprilTag 距离测量

使用正式入口 `kalibr-noros tools stereo-tag-distance`。实现位于
`src/python/kalibr_tools/stereo_tag_distance.py`；不要恢复独立 `tools/` 启动脚本，也不要在
skill、临时脚本或报告中复制检测、去畸变或三角化实现。

## 输入边界

- 数据集必须是包含 `dataset.yaml` 的目录数据集，并且存在 `cam0`、`cam1` 图像与时间戳；
  有 `meta/pairs.csv` 时以其为配对依据，否则按工具的时间容差配对。
- 标定文件可以是普通 `calibration_result` 的 `cameras` 列表，也可以是机器汇总
  `stereo_imu_calib.yaml` 的 `camera` 映射；不为这两种布局额外生成转换文件。
- 两目必须是 `pinhole-equi`／`pinhole-equidistant`，各自具有完整 K、四项 KB4 D、分辨率，
  `cam1.T_cn_cnm1` 必须表示 cam0 → cam1。标定文件必须属于被测双目模组。
- 默认测量 tag 0 到 tag 1；其他 ID 使用 `--tag0`、`--tag1` 显式指定。
- 命令只接受当前操作系统可直接访问的路径。不得把 `Q:\...` 等 Windows 路径静默转换为
  `/mnt/q/...`；路径语法需要改变时，先向用户明确给出实际路径并确认。

如果用户只要求汇总已有结果，直接读取现有 CSV，不重新检测或三角化。已有输出没有可信的
标定文件来源证据时，不推断它使用了当前指定 YAML；需要重算或覆盖时先取得明确授权。

## 固定计算口径

- 使用 Kalibr 原生 ETHZ AprilTag2 检测器的初始角点，不执行 `cornerSubPix`。
  这是当前两标签实验采用的重复性口径；不要宣称它必然更接近物理真值。
- 若用户要求亚像素对照，先说明当前入口没有该模式；只有用户明确要求修改工具时才增加，
  并保留当前默认结果，不能静默替换。
- 对两目同一 tag 的四个解码顺序角点使用 KB4 K/D 去畸变，以
  `P0=[I|0]`、`P1=[R|t]` 三角化；每个 tag 的三维中心为四角均值。
- 距离为 cam0 坐标系中两个三维中心的欧氏距离，输出单位 mm。
- 逐图对保留时间戳、配对时差、两 tag 中心、中心差向量、距离和双目重投影 RMS；
  汇总只统计成功图对，样本标准差使用 `ddof=1`。

## 执行与输出

数据集名必须以 `_YYMMDD_HHMM` 结尾。默认输出不能写入输入数据集，而是：

```text
<dataset-parent>/result/distance_YYMMDD_HHMM/
  distance.csv
  distance_summary.csv
  distance_detections/
    cam0/
    cam1/
```

检测图保持全分辨率，标出 tag 轮廓、四角编号、中心、相机、时间戳、黑边模式和图对距离。
`distance.csv` 中的 `cam0_detection_image`、`cam1_detection_image` 必须指向相应子目录。

```bash
kalibr-noros tools stereo-tag-distance \
  --dataset /absolute/path/stereo_YYMMDD_HHMM \
  --calibration /absolute/path/camera_calibration.yaml
```

需要自定义位置时使用 `--output-csv`、`--summary-csv`、`--detection-dir`。
已有同名输出时工具默认停止；只有用户明确授权替换本次结果时才使用 `--overwrite`。
不得顺带移动、删除或改名原始图像和数据集清单。

## 验收与报告

运行后至少核对：

- 成功／失败图对数，逐图对 CSV 行数是否对应实际配对数；
- `distance_detections/cam0`、`cam1` 的 PNG 数量是否各等于已处理图对数；
- CSV 中检测图路径是否存在且落在正确相机子目录；
- 平均值、中位数、样本标准差、最小值、最大值、极差；
- 使用的标定 YAML、tag ID、cam0 坐标系及未使用 `cornerSubPix` 的口径。

完成时给出可点击的 `distance.csv`、`distance_summary.csv` 和检测图目录。任一相机缺少指定
tag、模型或外参不完整、没有可配对图像、三角化为负深度时，保留明确失败信息，不把失败值
写成 0，也不据此宣称测距有效。
