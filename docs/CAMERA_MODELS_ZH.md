# OpenCV 相机模型

## Radtan

- `pinhole-radtan`：`[k1,k2,p1,p2]`，等价于 OpenCV k3=0；
- `pinhole-radtan5`：`[k1,k2,p1,p2,k3]`，参数顺序与 OpenCV 一致。

## Fisheye

工程目录只保留一个 `src/camera_models/opencv_fisheye` 模块：

- 公共 distortion 为 OpenCV `[k1,k2,k3,k4]`；
- zero-skew 类型兼容旧四内参 pinhole schema；
- full 类型使用 `[fu,fv,cu,cv,alpha]`，满足
  `K[0,1] = fu * alpha`，可无损保存非零 skew。

内部保留两个 projection 类型是因为设计变量维度分别为 4 和 5，不能使用同一
C++ 类型而不破坏旧 YAML/序列化 ABI；但 Python 注册、YAML 转换、公共 distortion
和构建目录已经合并，不再存在两个顶层扩展工程。

OpenCV stereo 导入支持 K1/D1/K2/D2、R/T 或 4x4 RT。平移必须显式给出单位或
缩放，转换器不会猜测毫米和米。所有变换统一说明方向：

```text
p_target = T_target_source * p_source
```
