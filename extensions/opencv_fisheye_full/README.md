# Full OpenCV fisheye (`alpha`/skew included)

This extension augments `extensions/opencv_fisheye` with the part that a
`PinholeProjection<Distortion>` cannot represent: OpenCV fisheye's nonzero
dimensionless `alpha` (pixel skew `K[0,1] = fx * alpha`).  It does not modify
the frozen ETHZ source tree and does not change any native optimizer stage,
stopping rule, damping rule, or default thread count.

## Exact parameterization

The projection design-variable block is

```text
[fu, fv, cu, cv, alpha]
```

and the independent distortion design-variable block remains

```text
D = [k1, k2, k3, k4].
```

For a normalized point, the projection is exactly OpenCV's model:

```text
r       = sqrt(x*x + y*y)
theta   = atan(r)
theta_d = theta * (1 + k1*theta^2 + k2*theta^4
                         + k3*theta^6 + k4*theta^8)
[xd,yd] = theta_d/r * [x,y]
u       = fu * (xd + alpha*yd) + cu
v       = fv * yd + cv
```

The implementation supplies analytic Jacobians with respect to the 3-D
point, all five projection parameters, and all four D coefficients.  The
inverse first removes the triangular intrinsic matrix, then solves the
fisheye theta polynomial.  It also preserves OpenCV's failed-Newton sentinel:
back-projection returns `false` and a zero Jacobian instead of allowing the
`(-1e6,-1e6)` sentinel to enter Kalibr as a physical ray.

## Lossless Kalibr schema

Legacy Kalibr declares `camera_model: pinhole` to have exactly four
intrinsics.  Storing nonzero alpha under that name would either fail schema
validation or silently discard it.  This extension therefore uses:

```yaml
camera_model: pinhole_opencv_fisheye
intrinsics: [fu, fv, cu, cv, alpha]
distortion_model: opencv_fisheye
distortion_coeffs: [k1, k2, k3, k4]
```

The Python integration validates this schema, constructs the five-parameter
projection for camera/IMU calibration, and writes the same schema back.  It
also reads the previous zero-skew `pinhole + opencv_fisheye` form as
`alpha = 0` during conversion.

OpenCV YAML import accepts alpha encoded in any of these ways:

- `K[0,1]`, interpreted as pixel skew and divided by `K[0,0]`;
- `alpha` / `alpha1` / `alpha2`, interpreted as dimensionless alpha;
- `skew` / `skew1` / `skew2`, interpreted as pixel skew.

When more than one representation is present, they must agree.  Export writes
all three unambiguously: the full K, dimensionless alpha, and pixel skew.

OpenCV stereo `R,T` continues to mean `p_cam1 = R*p_cam0 + T`, which is the
same direction as Kalibr `T_c1_c0` / `T_cn_cnm1`.

OpenCV does not prescribe the physical unit of `T`, while Kalibr camera/IMU
extrinsics are in metres.  Import therefore never guesses: without metadata it
preserves the numeric values, and it accepts `translation_unit: m|cm|mm|um` or
`translation_scale` (stored-units-to-metres).  The CLI exposes mutually
exclusive `--translation-unit` and `--translation-scale` overrides.  Exported
files explicitly state `translation_unit: m` and `translation_scale: 1.0`.

## Calibration model name

After build integration, use the existing user-facing name:

```text
--models pinhole-opencv-fisheye
```

but map it to `kalibr_opencv_fisheye_full.PinholeOpenCvFisheyeFull`, not the
zero-skew compatibility class.  Initialization deliberately starts alpha at
zero, as OpenCV does when skew is not preseeded; native Kalibr then optimizes
alpha in the ordinary projection parameter block.

## Tests

The C++ test compares nonzero-alpha projection directly against
`cv::fisheye::projectPoints`, checks point/projection/distortion/inverse
Jacobians by central finite differences, and verifies `K[0,1] = fu*alpha`.

The Python tests cover mono and stereo YAML round trips, nontrivial stereo
`R,T`, flat K/R/RT vendor YAML, transform direction, translation units, the
five-intrinsic Kalibr schema, D coefficient ordering, conflicting alpha/model
rejection, and legacy zero-skew import.

## Top-level CMake integration

Keep the existing zero-skew extension because this module reuses its tested
four-coefficient distortion class.  Add the following beside its two Python
targets:

```cmake
copy_python_package(
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/python"
  kalibr_opencv_fisheye_full)

add_python_export_library(kalibr_opencv_fisheye_full_cv_python
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/python/kalibr_opencv_fisheye_full"
  extensions/opencv_fisheye_full/src/cv_module.cpp)
target_include_directories(kalibr_opencv_fisheye_full_cv_python PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/include")
target_link_libraries(kalibr_opencv_fisheye_full_cv_python
  aslam_cv_python aslam_cameras aslam_cv_serialization sm_python numpy_eigen
  ${Boost_LIBRARIES})

add_python_export_library(kalibr_opencv_fisheye_full_backend_python
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/python/kalibr_opencv_fisheye_full"
  extensions/opencv_fisheye_full/src/backend_module.cpp)
target_include_directories(kalibr_opencv_fisheye_full_backend_python PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/include")
target_link_libraries(kalibr_opencv_fisheye_full_backend_python
  aslam_cv_backend_python aslam_cv_backend aslam_backend_python
  aslam_backend aslam_backend_expressions aslam_cv_python aslam_cameras
  aslam_splines sm_python numpy_eigen ${Boost_LIBRARIES})
```

In each generated calibration CLI, import and install the full package after
the base fisheye package:

```python
import kalibr_opencv_fisheye_full as koff
koff.install()
```

and map:

```python
'pinhole-opencv-fisheye': koff.PinholeOpenCvFisheyeFull
```

For a fisheye-only YAML conversion entry point, invoke
`kalibr_opencv_fisheye_full.__main__.main`.  The existing converter remains
appropriate for radtan/radtan5 files; a unified front end can dispatch its
fisheye subcommands to this module.
