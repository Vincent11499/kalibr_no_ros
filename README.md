# kalibr_no_ros

`kalibr_no_ros` is a ROS-free packaging of the native ETHZ-ASL Kalibr
calibration pipeline. The frozen upstream snapshot is commit
`1f60227442d25e36365ef5f72cd80b9666d73467`; its complete 1,630-file tree is
checked by `scripts/verify_upstream.py` against the repository SHA-256
manifest. Portability, camera-model and parallel-execution changes are kept
outside `upstream/kalibr` and applied to build/install artifacts.

The installed calibration surface intentionally contains only:

- `kalibr_calibrate_cameras`;
- `kalibr_calibrate_imu_camera`.

Both commands accept a ROS1 `.bag` file or a ROS2 recording directory without
installing or sourcing ROS1/ROS2. The calibration phases, native optimizer
state machine, per-phase options and numerical hyperparameters remain those of
the pinned ETHZ source. Parallel flags change execution resources, not the LM
damping policy, stopping rules, parameter scaling or phase ordering.

## ROS-free input boundary

The repository has two independent adapter packages:

- `bag_io` reads ROS1 and ROS2 recordings with `rosbags`; it does not import
  Kalibr or ROS.
- `core` presents those messages through Kalibr's original
  `BagImageDatasetReader` and `BagImuDatasetReader` interfaces.

Supported calibration input is deliberately narrow:

- ROS1: one `.bag` file;
- ROS2: a directory containing `metadata.yaml` and SQLite3 (`.db3`) or MCAP
  storage;
- messages: standard `sensor_msgs/Image`, `sensor_msgs/CompressedImage` and
  `sensor_msgs/Imu`.

A standalone `.db3`/`.mcap` without rosbag2 metadata and custom Snappy image
messages are outside this boundary. The remaining upstream utilities are not
installed as partially supported commands.

## Dependencies

Ubuntu 20.04 uses the native Python 3.8/Boost.Python 3.8 toolchain and Ubuntu
22.04 uses Python 3.10/Boost.Python 3.10. CMake selects the matching
Boost.Python component instead of hard-coding either ABI; a local compatibility
header covers the legacy `boost/detail/endian.hpp` include on newer Boost.

Prepare repository-private dependencies without modifying site-packages or the
host package database:

```bash
./scripts/bootstrap_deps.sh
```

The single bootstrap installs `rosbags` under `.deps/python`. On Ubuntu 22.04
it additionally uses `apt-get download` and `dpkg-deb --extract` to prepare
SuiteSparse and igraph/texttable under `.deps/sysroot`; Ubuntu 20.04 uses its
system C++ dependencies.

With the default `-DKALIBR_USE_PRIVATE_DEPS=ON`, CMake detects `.deps/sysroot`
and `.deps/python`, then stages the required private Python modules and
SuiteSparse runtime symlink chains in the build/install tree. Compiler,
CMake, OpenCV, Eigen, Boost, TBB and the standard C/C++ runtime are still host
toolchain dependencies.

## Build, install and test

```bash
cmake -S . -B build/linux-release \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build/linux-release --parallel 4
cmake --install build/linux-release
```

The launchers resolve their Python modules and bundled shared libraries
relative to the install prefix; no ROS environment is required. Test targets
and CTest registration are disabled by default and are enabled together with
`-DKALIBR_ENABLE_TESTING=ON`:

```bash
cmake -S . -B build/testing \
  -DCMAKE_BUILD_TYPE=Release \
  -DKALIBR_ENABLE_TESTING=ON
cmake --build build/testing --parallel 4
ctest --test-dir build/testing --output-on-failure
```

The lightweight Python suite and immutable snapshot check are also available
directly:

```bash
python3 scripts/run_tests.py
python3 scripts/verify_upstream.py
```

The legacy `aslam_cameras_tests` executable is still buildable in test mode,
but is disabled in CTest: it writes generated files into the frozen upstream
source directory and contains detector assertions that vary across OpenCV
versions. The standalone regression suite covers the camera extensions and
YAML conversion paths separately.

Set `-DKALIBR_ENABLE_NATIVE_OPTIMIZATIONS=OFF` for a reference build without
the out-of-tree Hessian/parallel overlay.

## Headless operation

`wxPython` is loaded only for an interactive report window. Calibration and
PDF generation work without an X server when the report window is disabled:

```bash
env -u ROS_DISTRO -u ROS_ROOT -u ROS_PACKAGE_PATH \
  -u CMAKE_PREFIX_PATH -u PYTHONPATH MPLBACKEND=Agg \
  ./install/bin/kalibr_calibrate_cameras --help
```

Use `--dont-show-report` for actual non-interactive calibration runs. Asking
to display the interactive window without wxPython produces an explicit error.

## Optional parallel execution and profiling

Both calibration commands expose the same parallel controls:

- `--parallelism N` sets detector processes and optimizer threads together;
- `--detector-processes N` overrides only calibration-target extraction;
- `--optimizer-threads N` overrides the native optimizer thread option.

Diagnostic printing and file I/O are compiled out by default. Enable them in a
dedicated profiling build:

```bash
cmake -S . -B build/profile \
  -DCMAKE_BUILD_TYPE=Release \
  -DKALIBR_ENABLE_PROFILING=ON
```

That build additionally exposes `--profile-optimizer` on IMU-camera
calibration and `--timing-json PATH` on both commands. The JSON splits image
processing into bag payload read, ROS-message deserialization, pixel decode and
target detection; calibration timing also separates initialization, problem
construction, optimization, residual statistics, report generation and result
serialization.

Component-specific flags take precedence over `--parallelism`. If none of
the three parallel flags is supplied, every ETHZ per-stage thread default is
left unchanged.

Target extraction uses bounded queues and at most `2*N` in-flight images,
disables nested OpenCV worker threads inside each process, and restores
observations to dataset order. Worker exceptions and hard exits are surfaced
to the parent instead of becoming an unbounded wait.

For BlockCholesky, `nThreads=0/1` retains the original serial
`ErrorTerm::buildHessian()` loop. With `nThreads>1`, workers calculate the
unchanged per-error Hessian/RHS contributions in private buffers and the caller
merges them strictly in original error-term order. Shared Hessian writes,
CHOLMOD/SPQR factorization, LM policy and optimizer state transitions remain
unchanged. Repeated runs with one fixed parallel configuration are designed
to be scheduling-independent, but bitwise identity is not promised across
different thread counts, compilers or numerical libraries; serial/parallel
equivalence is a strict numerical comparison.

`--optimizer-threads` controls Kalibr's optimizer option and the parallel
Hessian/error evaluation path. It does not fully control thread pools internal
to SuiteSparse, BLAS, OpenCV or other factorization/runtime libraries.

Example:

```bash
./install/bin/kalibr_calibrate_cameras \
  --bag /data/cam.bag \
  --target /data/aprilgrid.yaml \
  --models pinhole-radtan5 pinhole-radtan5 \
  --topics /cam0/image_raw /cam1/image_raw \
  --parallelism 8 \
  --detector-processes 6 \
  --dont-show-report
```

When profiling is enabled, the JSON contains wall and CPU time plus RSS/PSS
snapshots for extraction and native optimization stages. With profiling off,
the commands do not register optimizer timers, sample `/proc`, or create timing
files.

## Camera models and OpenCV YAML

In addition to the pinned upstream models, this branch provides:

- `pinhole-radtan5`: OpenCV-order distortion
  `[k1, k2, p1, p2, k3]` with four pinhole intrinsics;
- `pinhole-opencv-fisheye`: the complete OpenCV fisheye projection with
  intrinsics `[fu, fv, cu, cv, alpha]`, where pixel skew is
  `K[0,1] = fu * alpha`, and distortion `[k1, k2, k3, k4]`.

The five-intrinsic fisheye model uses a lossless Kalibr schema because native
`camera_model: pinhole` accepts only four intrinsics:

```yaml
camera_model: pinhole_opencv_fisheye
intrinsics: [fu, fv, cu, cv, alpha]
distortion_model: opencv_fisheye
distortion_coeffs: [k1, k2, k3, k4]
```

Two installed conversion commands keep the compatibility boundary explicit:

- `kalibr_convert_camera_yaml` handles radtan, radtan5 and legacy zero-skew
  OpenCV fisheye (`camera_model: pinhole`) in both directions;
- `kalibr_convert_opencv_fisheye_yaml` preserves the full fisheye K/D model,
  including nonzero alpha/skew, in both directions.

Both accept mono K/D and stereo K1/D1/K2/D2 plus R/T or a flat 4x4 `RT`.
Stereo export writes R/T/E/F, transform direction and translation-unit
metadata. Import never guesses a missing image resolution or translation
unit: provide the resolution and, when the stored T is not already metres,
`--translation-unit` or `--translation-scale`.

Generic radtan5 import and export:

```bash
./install/bin/kalibr_convert_camera_yaml import-stereo \
  --stereo opencv-stereo.yaml \
  --output camchain.yaml \
  --topics /cam0/image_raw /cam1/image_raw \
  --left-resolution 1920 1080 \
  --right-resolution 1920 1080 \
  --distortion-model radtan5 \
  --transform-direction cam0-to-cam1 \
  --translation-unit m

./install/bin/kalibr_convert_camera_yaml export \
  --input camchain.yaml --output-prefix opencv-export
```

Full OpenCV fisheye import and export use the same subcommand shape:

```bash
./install/bin/kalibr_convert_opencv_fisheye_yaml import-stereo \
  --stereo opencv-fisheye-stereo.yaml \
  --output fisheye-camchain.yaml \
  --topics /cam0/image_raw /cam1/image_raw \
  --left-resolution 1920 1080 \
  --right-resolution 1920 1080 \
  --translation-unit m

./install/bin/kalibr_convert_opencv_fisheye_yaml export \
  --input fisheye-camchain.yaml --output-prefix fisheye-export
```

For the inspected ROS2 recording in this workspace, the vendor stereo YAML
contains flat K1/K2 and flat 4x4 RT arrays and the images are 1920x1080. Its
radtan5 conversion is therefore:

```bash
dataset=/mnt/q/File/kalibr/data/evt_8_0730/imu_mipi_in_stereo_calib_08_0730
./install/bin/kalibr_convert_camera_yaml import-stereo \
  --stereo "$dataset/mipi_in_stereo_calib_08_0730.yaml" \
  --output /tmp/evt_8_0730-camchain.yaml \
  --topics /mipi_image_data_rgb0 /mipi_image_data_rgb1 \
  --left-resolution 1920 1080 \
  --right-resolution 1920 1080 \
  --distortion-model radtan5 \
  --transform-direction cam0-to-cam1 \
  --translation-unit m
```

The final branch was also exercised on the complete ROS2 `calibrated`
IMU-camera dataset and the ROS1 EuRoC `scale-misalignment` dataset.  See
[`docs/NATIVE_VALIDATION_20260820.md`](docs/NATIVE_VALIDATION_20260820.md) for
the exact thread counts, stage timings, memory trade-offs and numeric comparison
against the fixed native references.
The independent ROS1/ROS2 bag-read benchmark and the complete bottleneck
analysis are available in Chinese at
[`docs/NATIVE_STAGE_TIMING_REPORT_20260820_ZH.md`](docs/NATIVE_STAGE_TIMING_REPORT_20260820_ZH.md).
See [the radtan5 notes](docs/RADTAN5.md),
[the zero-skew compatibility notes](extensions/opencv_fisheye/README.md) and
[the full-fisheye notes](extensions/opencv_fisheye_full/README.md) for formulas
and schemas.

## Calibration examples

ROS1 and ROS2 use identical command syntax; only `--bag` changes from a file to
a recording directory:

```bash
env -u ROS_DISTRO -u ROS_ROOT -u ROS_PACKAGE_PATH \
  -u CMAKE_PREFIX_PATH -u PYTHONPATH MPLBACKEND=Agg \
  ./install/bin/kalibr_calibrate_cameras \
  --target /data/aprilgrid.yaml \
  --models pinhole-radtan5 pinhole-radtan5 \
  --topics /cam0/image_raw /cam1/image_raw \
  --bag /data/stereo.bag \
  --dont-show-report

env -u ROS_DISTRO -u ROS_ROOT -u ROS_PACKAGE_PATH \
  -u CMAKE_PREFIX_PATH -u PYTHONPATH MPLBACKEND=Agg \
  ./install/bin/kalibr_calibrate_imu_camera \
  --target /data/aprilgrid.yaml \
  --imu /data/imu.yaml \
  --imu-models scale-misalignment \
  --cams /data/camchain.yaml \
  --bag /data/ros2_recording \
  --dont-show-report
```

Use `scripts/compare_bag_inputs.py` to compare the ROS1/ROS2 streams presented
to Kalibr and `scripts/compare_kalibr_outputs.py` to compare result trees.
Detailed implementation and validation boundaries are recorded in
[docs/STATUS.md](docs/STATUS.md).
