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

Use `scripts/compare_kalibr_outputs.py` to repeat the result-file comparison.

## Known limits

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
