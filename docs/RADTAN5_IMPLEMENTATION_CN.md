# OpenCV 五参数针孔相机模型修改说明

本文档说明 `kalibr_no_ros` 为支持 OpenCV 五参数针孔畸变模型所做的代码修改、修改原因、接入方式、对原 Kalibr 标定链路的影响，以及对应的验证结果。

对应实现提交：

```text
cea5594 feat: add OpenCV five-coefficient camera model
```

## 1. 修改目标

原生 Kalibr 的 `pinhole-radtan` 使用四个畸变参数：

```text
[k1, k2, p1, p2]
```

本次增加一个独立模型，使相机标定能够优化 OpenCV 常用的五个畸变参数：

```text
[k1, k2, p1, p2, k3]
```

新模型的命名约定如下：

| 使用位置 | 名称 |
| --- | --- |
| 相机标定 CLI | `pinhole-radtan5` |
| camchain 中的相机模型 | `camera_model: pinhole` |
| camchain 中的畸变模型 | `distortion_model: radtan5` |
| OpenCV 输出模型 | `distortion_model: plumb_bob` |

本次修改遵守以下原则：

1. 不修改 `upstream/kalibr` 中的原生 Kalibr 源码。
2. 不改变原四参数 `pinhole-radtan` 的行为。
3. 相机、双目和 IMU–相机标定继续使用 Kalibr 原生优化链路。
4. 只新增五参数畸变模型及其必要的类型注册、配置和输出适配。
5. 复用 Kalibr 的 `PinholeProjection`、设计变量和重投影误差模板，避免复制已有算法。

## 2. 总体结构

新增代码集中在：

```text
extensions/radtan5/
├── include/kalibr_no_ros/radtan5/
│   ├── CameraTypes.hpp
│   └── RadialTangentialDistortion5.hpp
├── python/kalibr_radtan5/
│   ├── __init__.py
│   └── integration.py
└── src/
    ├── backend_module.cpp
    └── cv_module.cpp
```

整体接入关系为：

```text
kalibr_calibrate_cameras / kalibr_calibrate_imu_camera
                         │
                         ▼
                kalibr_radtan5.install()
                         │
            ┌────────────┴────────────┐
            ▼                         ▼
  配置、CLI、YAML 适配          C++/Python 类型注册
    integration.py        cv_module.cpp/backend_module.cpp
            │                         │
            └────────────┬────────────┘
                         ▼
           Radtan5PinholeCameraGeometry
                         │
                         ▼
              Kalibr 原生标定和优化流程
```

## 3. 新增五参数畸变数学模型

实现文件：

- [`RadialTangentialDistortion5.hpp`](../extensions/radtan5/include/kalibr_no_ros/radtan5/RadialTangentialDistortion5.hpp)

新增 C++ 类型：

```cpp
aslam::cameras::RadialTangentialDistortion5
```

参数顺序固定为：

```text
[k1, k2, p1, p2, k3]
```

对于归一化图像坐标 `(x, y)`：

```text
r² = x² + y²
r⁴ = r² · r²
r⁶ = r⁴ · r²
```

径向部分为：

```text
radial = k1·r² + k2·r⁴ + k3·r⁶
```

畸变后的归一化坐标为：

```text
xd = x + x·radial + 2·p1·x·y + p2·(r² + 2·x²)
yd = y + y·radial + p1·(r² + 2·y²) + 2·p2·x·y
```

这与 OpenCV 五参数 `plumb_bob` 模型的计算顺序和参数顺序一致。

### 为什么不直接修改原四参数类

原生 Kalibr 的 `RadialTangentialDistortion` 将参数维度固定为 4。如果直接增加 `k3`：

- 原 `pinhole-radtan` 的优化维度会从 4 变成 5；
- 旧 camchain 的四参数数据将无法保持原语义；
- 原相机标定结果和序列化格式会发生变化；
- 无法同时支持严格四参数模型和五参数模型。

因此新增独立类型，让两个模型并存：

```text
pinhole-radtan  → 原生四参数类型
pinhole-radtan5 → 新增五参数类型
```

当 `k3=0` 且前四个参数相同时，新旧模型会退化为同一个投影模型。实际数值验证中，两者最大归一化坐标差约为：

```text
1.11e-16
```

## 4. 增加解析雅可比矩阵

Kalibr 的非线性优化依赖误差项对各设计变量的雅可比矩阵（Jacobian）。因此，五参数模型不仅需要实现投影公式，还必须提供坐标雅可比矩阵和参数雅可比矩阵。

### 4.1 对归一化坐标的雅可比矩阵

径向项对半径的导数增加了 `k3` 部分：

```text
radial_slope = 2·k1 + 4·k2·r² + 6·k3·r⁴
```

该导数进入：

```text
∂(xd, yd) / ∂(x, y)
```

它会继续通过链式法则进入：

```text
三维点
→ 归一化坐标
→ 畸变坐标
→ 像素坐标
→ 重投影残差
```

### 4.2 对五个畸变参数的雅可比矩阵

参数雅可比矩阵为：

```text
∂xd/∂k1 = x·r²
∂xd/∂k2 = x·r⁴
∂xd/∂p1 = 2·x·y
∂xd/∂p2 = r² + 2·x²
∂xd/∂k3 = x·r⁶

∂yd/∂k1 = y·r²
∂yd/∂k2 = y·r⁴
∂yd/∂p1 = r² + 2·y²
∂yd/∂p2 = 2·x·y
∂yd/∂k3 = y·r⁶
```

### 为什么使用解析雅可比矩阵

- Kalibr 会在增量标定过程中多次重新线性化。
- 双目标定需要联合优化两台相机的内参、畸变、相机间外参和每个批次位姿。
- 解析雅可比矩阵比运行时有限差分更快、更稳定。
- 这种实现方式与 Kalibr 原生相机模型保持一致。

测试中使用中心有限差分对坐标雅可比矩阵和参数雅可比矩阵进行了独立校验。

## 5. 增加五维设计变量接口

在 `RadialTangentialDistortion5` 中实现了 Kalibr 模板要求的统一接口：

```cpp
void update(const double* v);
int minimalDimensions() const;
void getParameters(Eigen::MatrixXd& parameters) const;
void setParameters(const Eigen::MatrixXd& parameters);
Eigen::Vector2i parameterSize() const;
```

参数增量更新为：

```text
k1 ← k1 + Δk1
k2 ← k2 + Δk2
p1 ← p1 + Δp1
p2 ← p2 + Δp2
k3 ← k3 + Δk3
```

五参数模型的最小维度为：

```text
minimalDimensions() = 5
```

因此，相机标定阶段单台相机参与优化的内部参数为：

```text
[fu, fv, cu, cv, k1, k2, p1, p2, k3]
```

共 9 个参数。

## 6. 复用 Kalibr 的针孔投影和相机几何

实现文件：

- [`CameraTypes.hpp`](../extensions/radtan5/include/kalibr_no_ros/radtan5/CameraTypes.hpp)

新增两个类型别名：

```cpp
using Radtan5PinholeProjection =
    PinholeProjection<RadialTangentialDistortion5>;

using Radtan5PinholeCameraGeometry =
    CameraGeometry<Radtan5PinholeProjection, GlobalShutter, NoMask>;
```

### 为什么采用模板组合

Kalibr 的 `PinholeProjection` 本身已经实现：

- 针孔投影和反投影；
- `fu/fv/cu/cv` 参数管理；
- 投影有效性检查；
- 相机内参初始化；
- 投影雅可比矩阵的链式组合；
- 与 CameraGeometry、Frame 和重投影误差的接口。

畸变模型是 `PinholeProjection` 的模板参数，因此只需增加新的畸变策略，不需要重新迁移整套针孔相机代码。

当前组合为：

```text
PinholeProjection
+ RadialTangentialDistortion5
+ GlobalShutter
+ NoMask
```

本次没有扩展滚动快门、Omni、鱼眼、OpenCV 有理多项式、薄棱镜或倾斜传感器模型。

## 7. 增加 C++ 到 Python 的相机绑定

实现文件：

- [`cv_module.cpp`](../extensions/radtan5/src/cv_module.cpp)

生成模块：

```text
libkalibr_radtan5_cv_python.so
```

该模块向 Python 导出：

- `RadialTangentialDistortion5`
- `Radtan5PinholeProjection`
- `Radtan5PinholeCameraGeometry`
- `Radtan5PinholeFrame`

同时导出以下主要接口：

```text
distort
undistort
distortWithInputJacobian
distortParameterJacobian
getParameters
setParameters
euclideanToKeypoint
keypointToEuclidean
```

### 为什么必须绑定到 Python

Kalibr 的高级流程由 Python 调度，而相机投影、设计变量和误差项由 C++ 执行。没有该绑定，Python 层无法创建五参数相机，也无法把它传入原生 `CameraCalibrator` 和 IMU–相机标定流程。

## 8. 增加相机几何序列化

涉及文件：

- [`CameraTypes.hpp`](../extensions/radtan5/include/kalibr_no_ros/radtan5/CameraTypes.hpp)
- [`cv_module.cpp`](../extensions/radtan5/src/cv_module.cpp)

增加了：

```cpp
BOOST_CLASS_EXPORT_KEY(...)
BOOST_CLASS_EXPORT_IMPLEMENT(...)
.def_pickle(...)
```

畸变参数本身也实现了 Boost 序列化的 `save/load`。

### 为什么序列化是必要功能

Kalibr 的 AprilGrid 多进程角点提取会复制 `GridDetector`：

```python
copy.copy(detector)
```

`GridDetector` 内部通过基类指针持有具体相机几何。复制过程中需要序列化并恢复实际派生类型。如果没有注册五参数相机，会出现：

```text
unregistered class - derived class not registered or exported
```

因此序列化注册是多进程 AprilGrid 检测正常运行的必要条件。

## 9. 接入 Kalibr 优化后端

实现文件：

- [`backend_module.cpp`](../extensions/radtan5/src/backend_module.cpp)

生成模块：

```text
libkalibr_radtan5_backend_python.so
```

通过 Kalibr 已有模板注册：

```text
RadialTangentialDistortion5DesignVariable
Radtan5PinholeProjectionDesignVariable
Radtan5PinholeCameraGeometryDesignVariable
Radtan5PinholeReprojectionError
Radtan5PinholeReprojectionErrorSimple
```

### 为什么必须注册后端类型

相机标定会调用：

```python
camera.dv.projectionDesignVariable()
camera.dv.distortionDesignVariable()
camera.dv.shutterDesignVariable()
```

每个有效 AprilGrid 角点还需要建立对应的重投影误差项。如果只有投影公式而没有设计变量和误差类型，新模型只能用于计算像素坐标，不能进入 Kalibr 优化器。

这里复用了原生模板：

```cpp
exportGenericProjectionDesignVariable
exportReprojectionErrors
exportCameraDesignVariables
```

因此以下机制仍由原生 Kalibr 实现：

- 增量视图接收；
- `miTol` 信息增益筛选；
- `qrTol` 条件检查；
- 增量重优化和回滚（rollback）；
- 异常点剔除（outlier rejection）；
- 批次拒绝（batch rejection）；
- 最终协方差和重投影统计。

## 10. 定义统一的 Python 相机模型描述

实现文件：

- [`__init__.py`](../extensions/radtan5/python/kalibr_radtan5/__init__.py)

新增模型描述：

```python
class PinholeRadtan5:
    geometry = Radtan5PinholeCameraGeometry
    reprojectionError = Radtan5PinholeReprojectionError
    reprojectionErrorSimple = Radtan5PinholeReprojectionErrorSimple
    designVariable = Radtan5PinholeCameraGeometryDesignVariable
    projectionType = Radtan5PinholeProjection
    distortionType = RadialTangentialDistortion5
    shutterType = aslam_cv.GlobalShutter
    frameType = Radtan5PinholeFrame
```

Kalibr 的 Python 标定逻辑通过这些统一成员访问具体相机类型，因此不需要复制 `CameraCalibrator` 的实现。

## 11. 扩展 camchain 配置读取

实现文件：

- [`integration.py`](../extensions/radtan5/python/kalibr_radtan5/integration.py)

首先扩展 `CameraParameters.checkDistortion`，允许：

```yaml
distortion_model: radtan5
distortion_coeffs: [k1, k2, p1, p2, k3]
```

并严格要求参数数量为 5。随后扩展 `AslamCamera` 构造过程：

```text
camchain 数据
→ RadialTangentialDistortion5
→ Radtan5PinholeProjection
→ Radtan5PinholeCameraGeometry
```

### 为什么需要扩展 ConfigReader

原生 Kalibr 只识别：

```text
radtan、equidistant、fov、none
```

如果不增加该适配，IMU–相机标定读取五参数 camchain 时会直接报告未知畸变模型。

## 12. 接入相机和 IMU 两个 CLI

修改文件：

- [`CMakeLists.txt`](../CMakeLists.txt)

在生成无 ROS 版本 CLI 时加入：

```python
import kalibr_radtan5 as kr5
kr5.install()
```

相机标定模型表增加：

```python
'pinhole-radtan5': kr5.PinholeRadtan5
```

所以相机标定可以使用：

```bash
--models pinhole-radtan5
```

IMU–相机 CLI 没有 `--models` 参数，但加载适配器后可以读取 `distortion_model: radtan5` 的 camchain。

### 为什么采用生成期注入

- 不直接修改上游 Kalibr CLI 文件；
- 上游快照仍能通过哈希验证；
- 扩展逻辑集中在独立模块；
- 后续替换 Kalibr 快照时更容易检查差异。

## 13. 修改 camchain 保存逻辑

实现文件：

- [`integration.py`](../extensions/radtan5/python/kalibr_radtan5/integration.py)

原生保存函数不知道 `PinholeRadtan5` 和 `radtan5`，因此增加映射：

```text
PinholeRadtan5 → camera_model: pinhole
PinholeRadtan5 → distortion_model: radtan5
```

五参数结果保存为：

```yaml
camera_model: pinhole
distortion_model: radtan5
distortion_coeffs: [k1, k2, p1, p2, k3]
```

对于没有使用五参数模型的标定，代码会立即回退到原生保存函数：

```python
if not any(camera.model is PinholeRadtan5 for camera in calibrator.cameras):
    return original_save_chain_parameters_yaml(...)
```

这样可以保证原四参数和其他相机模型的输出路径不被新扩展改变。

## 14. 增加 OpenCV 输出

实现文件：

- [`integration.py`](../extensions/radtan5/python/kalibr_radtan5/integration.py)

### 14.1 单目文件

文件名：

```text
<bag>-camN-opencv.yaml
```

主要字段：

```text
camera_matrix
distortion_coefficients
image_width
image_height
distortion_model: plumb_bob
```

其中 `distortion_coefficients` 是 `1×5` 矩阵，顺序为：

```text
[k1, k2, p1, p2, k3]
```

### 14.2 双目文件

文件名：

```text
<bag>-camN-camN+1-opencv-stereo.yaml
```

主要字段：

```text
K1、D1、K2、D2、R、T、E、F
```

计算关系为：

```text
E = [T]× · R
F = K2⁻ᵀ · E · K1⁻¹
```

`R/T` 的方向与 Kalibr camchain 中的 `T_cn_cnm1` 一致，即把相机 `N` 中的点变换到相机 `N+1`。

### 为什么增加 OpenCV 输出

Kalibr camchain 仍是完整、权威的标定结果，但 OpenCV 用户通常希望直接调用：

```text
cv::FileStorage
cv::undistort
cv::stereoRectify
```

单独输出 OpenCV YAML 可以避免用户自行转换参数顺序和双目外参方向。

## 15. 构建系统修改

修改文件：

- [`CMakeLists.txt`](../CMakeLists.txt)

主要修改包括：

1. 复制并安装 `kalibr_radtan5` Python 包；
2. 构建 `libkalibr_radtan5_cv_python.so`；
3. 构建 `libkalibr_radtan5_backend_python.so`；
4. 链接 `aslam_cameras`、`aslam_cv_python`、`aslam_cv_backend`、`aslam_backend`、`numpy_eigen` 等已有库；
5. 在相机和 IMU CLI 中注册插件；
6. 将五参数测试加入 CTest。

相机类型模块和优化后端模块分开，是为了保持与 Kalibr 原有结构一致，并确保 Boost.Python 按正确顺序注册基础类型和后端派生类型。

## 16. 对相机和双目标定链路的影响

相机标定调用链没有被重写：

```text
读取 ROS1/ROS2 数据
→ AprilGrid 角点检测
→ 相机内参初始化
→ 创建投影和畸变设计变量
→ 增量视图筛选
→ 建立重投影残差
→ Kalibr IncrementalEstimator
→ 异常点和批次拒绝
→ 最终优化
→ 输出 camchain
```

相对于四参数模型，主要变化只有：

```text
畸变参数：4 维 → 5 维
相机内部参数：8 维 → 9 维
重投影公式：增加 k3·r⁶
参数雅可比矩阵：2×4 → 2×5
```

双目标定中的相机间外参、每个批次位姿、视图筛选和异常点逻辑仍由原生 Kalibr 处理。

## 17. 对 IMU–相机联合标定链路的影响

IMU–相机标定调用链为：

```text
读取 radtan5 camchain
→ 创建五参数相机几何
→ 提取 AprilGrid 观测
→ 使用五参数投影建立视觉残差
→ 初始化相机与 IMU 时间关系
→ 初始化 T_cam_imu
→ 构造位姿 B-spline
→ 加入视觉、陀螺仪和加速度计残差
→ 联合优化外参、时间偏移、B-spline 和 IMU 参数
→ 输出 camchain-imucam.yaml
```

按照原生 Kalibr 的 IMU–相机逻辑，相机内参和畸变参数在该阶段保持固定。因此：

- 相机标定阶段会优化 `k3`；
- IMU–相机阶段不会再次优化 `k3`；
- 视觉残差会使用固定的五参数投影；
- 最终 IMU camchain 会保留全部五个畸变系数。

IMU 的 `calibrated`、`scale-misalignment` 和 `scale-misalignment-size-effect` 模型共用同一个五参数相机适配器。

## 18. 测试和验证

测试文件：

- [`test_radtan5_native.py`](../scripts/test_radtan5_native.py)

自动测试覆盖：

- 参数顺序与五维参数数量；
- 与 `cv2.projectPoints()` 的投影一致性；
- 坐标解析雅可比矩阵与有限差分对比；
- 参数解析雅可比矩阵与有限差分对比；
- 畸变和反畸变互逆；
- PinholeProjection 与 OpenCV 对比；
- 相机几何序列化；
- projection/distortion/shutter 设计变量；
- camchain 读取和五参数数量检查；
- OpenCV 单目和双目 YAML；
- 双目 `R/T/E/F` 的尺寸和方向。

CTest 结果：

```text
3/3 通过
```

真实 EuRoC 数据验证包括：

- ROS1 单目五参数标定；
- ROS2 单目五参数标定；
- ROS1/ROS2 结果一致性；
- ROS1 双目五参数标定；
- 全量单目五参数标定；
- 全量 `calibrated` IMU–相机联合标定；
- 原四参数模型回归验证。

其中原四参数标定在加入插件后的文本结果保持不变，YAML 只有约 `1e-15` 的浮点末位波动。

## 19. 文件修改汇总

| 文件 | 类型 | 主要功能 | 修改原因 |
| --- | --- | --- | --- |
| `RadialTangentialDistortion5.hpp` | 新增 | 五参数公式、雅可比矩阵、更新和序列化 | 提供可被 Kalibr 优化的 OpenCV 五参数模型 |
| `CameraTypes.hpp` | 新增 | 组合 Pinhole、radtan5、GlobalShutter | 复用 Kalibr 模板，避免重写投影和几何 |
| `cv_module.cpp` | 新增 | 相机、投影、Frame 和序列化绑定 | 让 Python 标定流程可以使用新 C++ 类型 |
| `backend_module.cpp` | 新增 | 设计变量和重投影误差绑定 | 让新模型进入 Kalibr 优化器 |
| `kalibr_radtan5/__init__.py` | 新增 | 定义统一相机模型描述 | 对接 `CameraCalibrator` 所需接口 |
| `integration.py` | 新增 | 配置读取、相机构造、YAML 保存和 OpenCV 输出 | 在不修改上游源码的前提下注册新模型 |
| `CMakeLists.txt` | 修改 | 构建、安装、CLI 注册和 CTest | 将扩展接入无 ROS 构建产物 |
| `test_radtan5_native.py` | 新增 | 数学、绑定、配置和输出测试 | 防止公式、维度和注册回归 |
| `README.md` | 修改 | 增加基本使用入口 | 告知用户如何选择五参数模型 |
| `docs/RADTAN5.md` | 修改 | 简版模型与使用说明 | 提供面向使用者的快速文档 |
| `docs/STATUS.md` | 修改 | 记录验证结果 | 明确已验证范围和已知限制 |

## 20. 与原生 Kalibr 的差异边界

保持不变的部分：

- AprilGrid 检测算法；
- 相机初始化总体流程；
- Kalibr 增量估计器；
- `miTol` 和 `qrTol`；
- 回滚和批次管理；
- 相机/双目重投影误差结构；
- IMU 位姿 B-spline；
- 陀螺仪和加速度计误差项；
- IMU–相机外参和时间偏移优化；
- 三种 IMU 内参模型；
- ROS1/ROS2 数据适配边界。

新增或变化的部分：

- 新增 `k3·r⁶` 径向畸变项；
- 畸变设计变量由 4 维扩展为 5 维；
- 新增五参数类型的 Python 和优化后端注册；
- 新增 `radtan5` camchain 读写；
- 新增 OpenCV 单目和双目输出；
- 新增相关测试。

已知限制：

- 仅支持全局快门针孔相机；
- 不支持 OpenCV 超过五参数的扩展模型；
- 没有为新几何类型绑定专用的 Kalibr 去畸变器（undistorter）；当前离线相机和 IMU 标定链路不依赖该对象；
- 相机标定中增加一个自由度后，增量视图选择和异常点集合可能与四参数模型不同，因此最终标定结果不要求逐值相同。

## 21. 结论

本次实现不是调用 `cv::calibrateCamera` 替换 Kalibr，而是将 OpenCV 五参数畸变公式实现为一个符合 Kalibr 相机策略接口的新类型。这样既获得了 OpenCV `[k1,k2,p1,p2,k3]` 兼容性，又保留了 Kalibr 原生的增量相机标定、双目外参优化以及 IMU–相机联合优化链路。

原四参数模型和上游 Kalibr 快照均未被修改。
