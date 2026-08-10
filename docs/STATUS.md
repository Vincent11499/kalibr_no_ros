# Migration and validation status

## Implemented boundary

- The upstream Kalibr snapshot is fixed at commit
  `1f60227442d25e36365ef5f72cd80b9666d73467` and verified by SHA-256.
- Kalibr's native calibration libraries and Python algorithms are built without
  catkin or ROS libraries. The numerical calibration source is unchanged.
- `bag_io` reads and writes standard ROS1 and ROS2 `sensor_msgs/Image`,
  `sensor_msgs/CompressedImage`, and `sensor_msgs/Imu` messages through
  `rosbags`. It does not import Kalibr, ROS1, or ROS2.
- A `.bag` file selects the ROS1 backend. A directory containing
  `metadata.yaml` selects the ROS2 backend. Rosbag2 SQLite3 and MCAP storage are
  accepted through the same interface; SQLite3 has full dataset validation.
- `core` adapts `bag_io` to Kalibr's original `BagImageDatasetReader` and
  `BagImuDatasetReader` contracts, including header timestamps, optional clock
  synchronization, cropping, frequency selection, and in-place shuffle
  semantics.
- Phase-one installed commands are `kalibr_calibrate_cameras` and
  `kalibr_calibrate_imu_camera`.
- An out-of-tree `pinhole-radtan5` model optimizes OpenCV-order
  `[k1, k2, p1, p2, k3]` using native Kalibr design variables and analytic
  Jacobians. It is available to both phase-one commands without changing the
  upstream snapshot.

## EuRoC validation

Dataset: `/home/czh/kalibr_workspace/data/euroc_cam`.

All deterministic comparisons below used the same bag, target, camera chain,
CLI defaults, and executable build. ROS environment variables were removed for
the no-ROS run.

| Case | Scope | Result |
| --- | --- | --- |
| Mono camera | 15--18 s, `--no-shuffle` | YAML and text byte-identical |
| Stereo camera | 15--18 s, `--no-shuffle` | Text byte-identical; YAML max difference about `4.2e-13` |
| Mono camera | Full bag, `--no-shuffle` | Text byte-identical; YAML only last-digit float serialization differences |
| IMU `calibrated` | Full bag | camchain, IMU YAML, and text byte-identical |
| IMU `scale-misalignment` | Full bag | camchain, IMU YAML, and text byte-identical |
| IMU `scale-misalignment-size-effect` | Full bag | camchain, IMU YAML, and text byte-identical |

## ROS2 validation

The two ROS1 EuRoC recordings were converted losslessly with
`python3 -m rosbags.convert` and stored under
`/home/czh/kalibr_workspace/data/euroc_cam_ros2` as standard rosbag2 SQLite3
directories.

| Check | Result |
| --- | --- |
| `cam_april`: 2 image topics | All 2,899 image headers and pixel payloads exactly equal |
| `cam_april`: IMU | All 14,490 messages and numeric fields exactly equal |
| `imu_april`: 2 image topics | All 2,878 image headers and pixel payloads exactly equal |
| `imu_april`: IMU | All 14,381 messages and numeric fields exactly equal |
| Mono camera, 15--18 s | Results text byte-identical; YAML differs only at about `2.8e-12` |
| Stereo camera, 15--18 s | Results text byte-identical; YAML differs only at about `4.3e-13` |
| IMU-camera `calibrated`, full bag | camchain, IMU YAML, and results text byte-identical |

The full ROS2 IMU-camera run constructed the same 14,423 design variables,
203,045 error terms, and `434850 x 64887` Jacobian as ROS1. Its final cost and
camera-to-IMU shift were respectively `43569.3` and
`-9.666847237544294e-05 s`.

The full no-ROS stereo run with Kalibr's default shuffle converged using 72
views. Reprojection standard deviations were about `[0.305, 0.258]` pixels for
cam0 and `[0.300, 0.256]` pixels for cam1. The estimated baseline translation
was `[-0.10996, 0.00040, -0.00061]` metres.

The full IMU runs constructed 14,423 design variables and 203,045 error terms.
All 14,381 gyro and accelerometer samples were added. The three estimated
camera-to-IMU time shifts were approximately:

- `calibrated`: `-9.66685e-05 s`
- `scale-misalignment`: `-8.31345e-05 s`
- `scale-misalignment-size-effect`: `-8.89441e-05 s`

## OpenCV radtan5 validation

The five-parameter implementation was compared directly against OpenCV 4.2
projection, and its coordinate and parameter Jacobians were checked by finite
differences. Geometry serialization, all three design-variable adapters,
camchain parsing, and OpenCV mono/stereo exports are covered by CTest.

| Case | Result |
| --- | --- |
| ROS1 mono, 15--18 s | Converged; 46 of 61 views used |
| ROS2 mono, 15--18 s | Camchain and text exactly equal to ROS1 |
| ROS1 stereo, 15--18 s | Converged; all 61 views used |
| ROS1 mono, full bag | Converged; 90 views used; reprojection std about `[0.0966, 0.0893]` px |
| IMU-camera `calibrated`, full bag | Converged from cost `1.53e6` to `3.91e4`; all 14,381 gyro and accelerometer samples used |
| Existing four-parameter mono path | Text result unchanged; YAML differs only at about `1e-15` numerical roundoff |

The full five-parameter IMU-camera run constructed 14,423 design variables and
202,739 error terms with a `434238 x 64887` Jacobian. Its final camera-to-IMU
time shift was about `-3.00e-05 s`; the output camchain retained all five fixed
camera distortion coefficients.

Use `scripts/compare_kalibr_outputs.py` to repeat the result-file comparison.

## Ceres 2.2 backend validation

The optional C++17/Ceres build exposes `--optimizer {native,ceres}` on both
phase-one commands; `native` remains the default. The IMU-camera Ceres path
constructs and solves the complete joint problem for all three supported IMU
models. On the EuRoC 15--25 s window, `2 * Ceres cost` matched Kalibr's final
objective within about `6e-3` absolute, and the camera-to-IMU transform differed
by at most `5.7e-4 deg`, `1.4e-5 m`, and `5.2e-7 s` across the three models.

Full-bag comparisons for all three IMU models used the same input camchain and
30-iteration limit. Temporal batching reduced 14381 IMU measurements to 7822
residual blocks without changing the solution. Native `J` versus Ceres
`2*cost` was `43975.1/43975.1258` for `calibrated`,
`43761.5/43761.5150` for `scale-misalignment`, and
`43348.5/43348.4696` for `scale-misalignment-size-effect`. Across the three
models, transform differences were at most `3.10e-4 deg`, `3.45e-6 m`, and
`3.14e-7 s`; IMU-intrinsic matrix/vector elements differed by at most
`1.53e-5`.

Ceres wall-clock time was respectively 70.92, 75.82, and 78.08 seconds versus
native 91.62, 89.97, and 93.01 seconds: a 15.7--22.6% full-pipeline speedup.
Peak RSS was still 1.8--17.4% higher than native. A 4-thread control run before
batching saved less than 1% RSS and was slower than native, so graph/AutoDiff
storage rather than per-thread workspace remains the next memory target.

A later controlled three-way `calibrated` run compared the untouched native
numerical core, the native-compatible optimization branch, and Ceres after
giving the latter two the same bounded multiprocessing corner extractor.
Outputs from the native-compatible branch were byte-identical to baseline.
The baseline was the ROS1 workspace CLI itself. Wall times were
57.35/54.83/38.04 seconds and peak RSS values were
825016/835596/976380 KiB. Ceres was 33.7% faster in that run but used 18.3%
more peak memory. See `docs/THREE_WAY_BENCHMARK_CN.md` for scope and caveats.

The same three-way matrix was also completed with the calibrated stereo
camchain and both camera reprojection streams for all three IMU models. The
native-compatible outputs were byte-identical to ROS1 Kalibr in every model.
Ceres camera-to-IMU differences were at most `1.33e-4 deg`, `3.93e-6 m`, and
`6.09e-7 s`; IMU-intrinsic elements differed by at most `1.06e-5`. Ceres was
34.9--44.8% faster end to end but used 25.1--41.4% more peak RSS. Full tables
are in `docs/THREE_WAY_BENCHMARK_CN.md`.

The camera Ceres path currently preserves Kalibr's native initialization,
incremental view acceptance, information/QR checks, rollback, and rejection
timing, then runs a parallel global Ceres BA on the accepted observations. In
the deterministic 15--18 s stereo run, both paths accepted 60 of 61 views and
removed the same 65 corners. Ceres reduced its matching reprojection cost from
`80.19258` to `78.45061` in 7 iterations. Runtime checks compare Kalibr and
Ceres objective values before and after every camera refinement and abort when
their relative mismatch exceeds `1e-4`.

Ten CTest regressions cover parallel execution, spline/IMU equations, all IMU
models, camera residuals, joint state mapping, mono/stereo camera BA, Python
integration, radtan5, and the immutable upstream snapshot. Detailed equations,
performance numbers, commands, and current limits are in
`docs/CERES_OPTIMIZER_CN.md`.

## Known limits

- The Ceres IMU-camera path currently supports one IMU and pinhole
  radtan4/radtan5 cameras. Its full covariance recovery is not implemented.
- The Ceres camera path is a final global BA, not yet a replacement for the
  native incremental camera state machine. Blake-Zisserman loss is rejected,
  and the covariance printed after refinement is the native pre-refinement
  approximation.
- Ceres IMU-camera A/B validation covers both a deterministic 10-second EuRoC
  window and a full-bag run for all three IMU models. Peak memory remains above
  native even though all three full runs are faster.

- The custom Snappy image message used by some Kalibr datasets is not yet
  supported. Standard raw and compressed image messages are supported.
- Rolling-shutter calibration is intentionally outside phase one.
- A full stereo run forced to sequential `--no-shuffle` accepted 242 batches
  before final filtering and was killed with exit code 137 on this 8 GiB host.
  The original default shuffle kept the incremental problem smaller and
  completed. Short-window deterministic stereo comparison remains available.
- The lazy ROS1 image reader uses the indexed ROS1-bag API of `rosbags` 0.9.x;
  the Python dependency is therefore constrained to `<0.10` until its 0.10 API
  is reviewed. ROS2 uses public timestamp-bounded rosbag2 queries.
- A normal ROS2 recording directory with `metadata.yaml` is required. Passing
  a standalone `.db3` or `.mcap` without rosbag2 metadata is not supported.
