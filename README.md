# kalibr_no_ros

`kalibr_no_ros` keeps the Kalibr calibration algorithms at upstream commit
`1f60227442d25e36365ef5f72cd80b9666d73467`, while removing ROS from the data
boundary and runtime environment.

The repository contains two intentionally separate packages:

- `bag_io`: ROS1 bag reading/writing based on `rosbags`. It does not import
  Kalibr or ROS.
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

Inspect a bag through the standalone package:

```bash
python3 -m pip install --user -e ./bag_io
kalibr-bag-info ../../data/euroc_cam/cam_april.bag
```

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

See `docs/STATUS.md` for the validated boundary, numerical baseline, and known
limits.
