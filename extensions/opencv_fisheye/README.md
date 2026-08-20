# OpenCV fisheye extension

This out-of-tree extension adds the opt-in, **zero-skew OpenCV fisheye**
Kalibr camera model
`pinhole-opencv-fisheye`.  It does not replace Kalibr's native
`pinhole-equi` model or modify the upstream source snapshot.

The distortion coefficients use OpenCV order:

```text
D = [k1, k2, k3, k4]
theta_d = theta * (1 + k1*theta^2 + k2*theta^4
                         + k3*theta^6 + k4*theta^8)
```

Kalibr's native pinhole projection has no skew/alpha design variable, so
importing a matrix with a nonzero `K[0,1]` or a nonzero `alpha`/`skew` field
is rejected instead of silently discarding it.  Consequently this is not a
claim of unrestricted compatibility with every `cv::fisheye` configuration;
it is complete for the zero-skew model Kalibr can represent.  The model is
mathematically equivalent to Kalibr's equidistant model; the distinct name
preserves OpenCV model intent across YAML round-trips and provides finite
Jacobians on the optical axis.

## YAML conversion

After the native modules are built, use:

```bash
python3 -m kalibr_opencv_fisheye import-mono \
  --input camera.yaml --output camchain.yaml \
  --topic /cam0/image_raw --resolution 1280 720

python3 -m kalibr_opencv_fisheye import-stereo \
  --stereo stereo.yaml --output camchain.yaml \
  --topics /cam0/image_raw /cam1/image_raw

python3 -m kalibr_opencv_fisheye import-stereo \
  --left left.yaml --right right.yaml --stereo extrinsics.yaml \
  --output camchain.yaml

python3 -m kalibr_opencv_fisheye export \
  --input camchain.yaml --output-prefix calibration
```

Mono input accepts either `K/D` or
`camera_matrix/distortion_coefficients`, including OpenCV
`%YAML:1.0`/`!!opencv-matrix` and ROS-style `rows/cols/data` mappings. Stereo input accepts
`K1/D1/K2/D2/R/T` (and `M1/M2` as aliases).  When image dimensions are not
stored in the input, pass them explicitly.

OpenCV's standard stereo convention is used by default:

```text
p_cam1 = R * p_cam0 + T
```

This is the same direction as Kalibr's `cam1.T_cn_cnm1`.  For a vendor file
that explicitly stores the reverse transform, set
`transform_direction: cam1-to-cam0` in the YAML or add the CLI override:

```text
--transform-direction cam1-to-cam0
```

OpenCV does not define a unit for stereo `T`, while Kalibr uses meters.  The
converter honors `translation_unit` (`m`, `cm`, `mm`, or `um`) and
`translation_scale` (the multiplier from stored values to meters) when those
fields are present.  For files without unit metadata, values are preserved;
use one of these explicit CLI overrides when needed:

```text
--translation-unit mm
--translation-scale 0.001
```

Export writes one OpenCV file per camera and one file per adjacent stereo
pair.  Fisheye `D` matrices are written as 4-by-1, and stereo files contain
`K1/D1/K2/D2/R/T/E/F` plus explicit transform-direction and meter-unit
metadata.

The converter intentionally supports OpenCV-compatible pinhole models only:
`opencv_fisheye`/`equidistant`, `radtan`, and `radtan5`.  OpenCV YAML has no
standard representation for Kalibr IMU extrinsics, time offsets, omni, EUCM,
or DoubleSphere fields; those are not reconstructed from OpenCV files.
