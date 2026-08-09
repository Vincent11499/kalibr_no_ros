# OpenCV five-coefficient pinhole model

更详细的中文代码修改和设计原因说明见
[`RADTAN5_IMPLEMENTATION_CN.md`](RADTAN5_IMPLEMENTATION_CN.md)。

`kalibr_no_ros` adds an out-of-tree global-shutter pinhole model that is
compatible with OpenCV's five-coefficient `plumb_bob` convention. Kalibr's
upstream snapshot is not modified, and its existing four-coefficient
`pinhole-radtan` path remains unchanged.

## Model and parameter order

Select the model on the camera-calibration CLI with `pinhole-radtan5`. Its
camchain representation is:

```yaml
camera_model: pinhole
distortion_model: radtan5
distortion_coeffs: [k1, k2, p1, p2, k3]
```

For normalized image coordinates `(x, y)` and `r2 = x*x + y*y`, the model is:

```text
radial = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
xd = x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)
yd = y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y
```

All nine camera parameters `[fu, fv, cu, cv, k1, k2, p1, p2, k3]` are active
during camera calibration. The implementation supplies analytic Jacobians for
the normalized coordinates and all five distortion coefficients, so it uses
Kalibr's original incremental estimator, outlier handling, and reprojection
error pipeline.

This model does not implement OpenCV's rational `k4..k6`, thin-prism, or tilted
sensor extensions. It is also intentionally limited to global-shutter pinhole
cameras.

## Commands

Mono camera calibration:

```bash
env -u ROS_DISTRO -u ROS_ROOT -u ROS_PACKAGE_PATH \
  -u CMAKE_PREFIX_PATH -u PYTHONPATH MPLBACKEND=Agg \
  ./install/bin/kalibr_calibrate_cameras \
  --target ../../data/euroc_cam/april_6x6.yaml \
  --models pinhole-radtan5 \
  --topics /cam0/image_raw \
  --bag ../../data/euroc_cam/cam_april.bag \
  --no-shuffle --dont-show-report
```

Stereo camera calibration:

```bash
env -u ROS_DISTRO -u ROS_ROOT -u ROS_PACKAGE_PATH \
  -u CMAKE_PREFIX_PATH -u PYTHONPATH MPLBACKEND=Agg \
  ./install/bin/kalibr_calibrate_cameras \
  --target ../../data/euroc_cam/april_6x6.yaml \
  --models pinhole-radtan5 pinhole-radtan5 \
  --topics /cam0/image_raw /cam1/image_raw \
  --bag ../../data/euroc_cam/cam_april.bag \
  --no-shuffle --dont-show-report
```

The same commands accept a ROS2 recording directory in `--bag`.

IMU-camera calibration reads the resulting `radtan5` camchain directly:

```bash
env -u ROS_DISTRO -u ROS_ROOT -u ROS_PACKAGE_PATH \
  -u CMAKE_PREFIX_PATH -u PYTHONPATH MPLBACKEND=Agg \
  ./install/bin/kalibr_calibrate_imu_camera \
  --target ../../data/euroc_cam/april_6x6.yaml \
  --imu ../../data/euroc_cam/imu_adis16448.yaml \
  --imu-models calibrated \
  --cams /path/to/cam_april-camchain.yaml \
  --bag ../../data/euroc_cam/imu_april.bag \
  --dont-show-report
```

`calibrated`, `scale-misalignment`, and
`scale-misalignment-size-effect` use the same camera model adapter. Camera
intrinsics and distortion are held fixed by the original IMU-camera pipeline;
the fifth coefficient is preserved in the output camchain.

## Output files

The normal Kalibr camchain is still the authoritative calibration output. A
run containing at least one `pinhole-radtan5` camera also writes OpenCV
FileStorage YAML:

- `<bag>-camN-opencv.yaml`: `camera_matrix`, five-element
  `distortion_coefficients`, image size, and `plumb_bob` model name.
- `<bag>-camN-camN+1-opencv-stereo.yaml`: `K1`, `D1`, `K2`, `D2`, `R`, `T`,
  `E`, and `F` for each adjacent stereo pair.

The stereo `R` and `T` have the same direction as Kalibr's
`T_cn_cnm1`: they transform a point from camera `N` into camera `N+1`.

## Implementation map

- `extensions/radtan5/include/kalibr_no_ros/radtan5/`: distortion formula and
  concrete pinhole geometry types.
- `extensions/radtan5/src/cv_module.cpp`: camera, projection, frame,
  serialization, and Python bindings.
- `extensions/radtan5/src/backend_module.cpp`: reprojection errors and Kalibr
  design-variable bindings.
- `extensions/radtan5/python/kalibr_radtan5/integration.py`: CLI registration,
  camchain support, and OpenCV exports.
- `scripts/test_radtan5_native.py`: OpenCV equivalence, Jacobian,
  serialization, design-variable, config, and output tests.
