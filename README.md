# kalibr_no_ros

`kalibr_no_ros` keeps the Kalibr calibration algorithms at upstream commit
`1f60227442d25e36365ef5f72cd80b9666d73467`, while removing ROS from the data
boundary and runtime environment. The same calibration commands accept ROS1
bag files and ROS2 bag directories without sourcing either ROS version.

The repository contains two intentionally separate packages:

- `bag_io`: ROS1 and ROS2 bag reading/writing based on `rosbags`. It does not
  import Kalibr or ROS.
- `core`: adapters that expose Kalibr's original dataset-reader API and feed
  data from `bag_io` into the unchanged calibration pipeline.

The copy below `upstream/kalibr` is immutable. No-ROS changes live outside the
snapshot so that algorithm changes cannot be hidden in a porting patch.

## Dependencies

The native build is currently validated on Ubuntu 20.04 with Python 3.8,
OpenCV 4.2, Boost 1.71, Eigen 3, SuiteSparse, TBB, and NumPy. Install the
ROS-free Python bag dependency with:

```bash
python3 -m pip install --user 'rosbags>=0.9.20,<0.10'
```

OpenCV is consumed from the system Python installation (`python3-opencv`), so
no wheel-specific OpenCV ABI is introduced.

## Build and test

From this repository:

```bash
cmake -S . -B build/linux-release \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build/linux-release --parallel 4
cmake --install build/linux-release
cd build/linux-release
ctest --output-on-failure
```

The install is relocatable and contains only the two phase-one commands. It
does not need a sourced ROS environment.

## Run

Inspect a ROS1 bag file or ROS2 bag directory through the standalone package:

```bash
python3 -m pip install --user -e ./bag_io
kalibr-bag-info ../../data/euroc_cam/cam_april.bag
kalibr-bag-info ../../data/euroc_cam_ros2/cam_april_ros2
```

`--bag` uses the same syntax in both installed calibration commands:

- ROS1: pass a `.bag` file.
- ROS2: pass the recording directory containing `metadata.yaml` and SQLite3
  (`.db3`) or MCAP storage. SQLite3 is covered by the full EuRoC regression;
  MCAP is handled by the same `rosbags` rosbag2 reader.

Run deterministic mono camera calibration:

```bash
env -u ROS_DISTRO -u ROS_ROOT -u ROS_PACKAGE_PATH \
  -u CMAKE_PREFIX_PATH -u PYTHONPATH MPLBACKEND=Agg \
  ./install/bin/kalibr_calibrate_cameras \
  --target ../../data/euroc_cam/april_6x6.yaml \
  --models pinhole-radtan \
  --topics /cam0/image_raw \
  --bag ../../data/euroc_cam/cam_april.bag \
  --no-shuffle --dont-show-report
```

The equivalent ROS2 invocation changes only the bag path:

```bash
env -u ROS_DISTRO -u ROS_ROOT -u ROS_PACKAGE_PATH \
  -u CMAKE_PREFIX_PATH -u PYTHONPATH MPLBACKEND=Agg \
  ./install/bin/kalibr_calibrate_cameras \
  --target ../../data/euroc_cam/april_6x6.yaml \
  --models pinhole-radtan \
  --topics /cam0/image_raw \
  --bag ../../data/euroc_cam_ros2/cam_april_ros2 \
  --no-shuffle --dont-show-report
```

An OpenCV-compatible five-coefficient pinhole model is available as
`pinhole-radtan5`. It optimizes distortion in OpenCV order
`[k1, k2, p1, p2, k3]`, writes `distortion_model: radtan5` to the Kalibr
camchain, and additionally produces OpenCV FileStorage YAML containing `K/D`
and, for stereo, `R/T/E/F`:

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

The resulting camchain can be passed unchanged to
`kalibr_calibrate_imu_camera`. See [docs/RADTAN5.md](docs/RADTAN5.md) for the
formula, stereo transform convention, output names, and implementation map.

Run IMU-camera calibration with one of the three upstream models:

```bash
env -u ROS_DISTRO -u ROS_ROOT -u ROS_PACKAGE_PATH \
  -u CMAKE_PREFIX_PATH -u PYTHONPATH MPLBACKEND=Agg \
  ./install/bin/kalibr_calibrate_imu_camera \
  --target ../../data/euroc_cam/april_6x6.yaml \
  --imu ../../data/euroc_cam/imu_adis16448.yaml \
  --imu-models scale-misalignment \
  --cams /path/to/cam_april-camchain.yaml \
  --bag ../../data/euroc_cam/imu_april.bag \
  --dont-show-report
```

Compare native and no-ROS outputs:

```bash
python3 scripts/compare_kalibr_outputs.py \
  /path/to/native-results /path/to/no-ros-results
```

Compare the complete ROS1 and ROS2 input streams as seen by Kalibr:

```bash
python3 scripts/compare_bag_inputs.py \
  ../../data/euroc_cam/imu_april.bag \
  ../../data/euroc_cam_ros2/imu_april_ros2 \
  --image-topic /cam0/image_raw \
  --image-topic /cam1/image_raw \
  --imu-topic /imu0
```

The command compares every image header, every pixel payload, and every IMU
sample by default. Use `--sample-images N` only when a faster sampled check is
preferred.

To create a rosbag2 SQLite3 copy without installing ROS2:

```bash
python3 -m rosbags.convert input.bag --dst output_ros2
```

See `docs/STATUS.md` for the validated boundary, numerical baseline, and known
limits.
