# Kalibr no-ROS

ROS-free packaging of the native ETHZ-ASL Kalibr camera and camera–IMU
calibration algorithms. The optimizer state machines, residual definitions,
parameter activation, numerical hyperparameters and stopping conditions remain
compatible with the pinned ETHZ implementation.

## Source layout

- `ref/kalibr`: immutable 1,630-file ETHZ snapshot at commit
  `1f60227442d25e36365ef5f72cd80b9666d73467`, used only by reference builds.
- `src/kalibr`: editable production source, grouped into foundation, camera,
  optimization, trajectory, calibration and third-party domains.
- `src/python`: ROS-free bag I/O, public CLI and runtime instrumentation.
- `src/camera_models`: OpenCV radtan5 and fisheye model extensions.
- `schemas`: versioned task and calibration-result contracts.
- `tools`: dependency bootstrap, source audit, comparison and benchmark tools.

`tools/verify_reference.py` verifies the frozen snapshot.
`tools/audit_source_delta.py` rejects unapproved differences in copied Kalibr
sources.

## Dependencies

On Ubuntu 20.04/22.04, prepare repository-private Python and SuiteSparse
dependencies without changing system site-packages:

```bash
./tools/bootstrap_deps.sh
```

The host still provides the compiler, CMake, Ninja, Eigen, OpenCV, Boost and
TBB. No ROS installation or sourced ROS environment is required.

## Build

All build and test presets are capped at four parallel jobs. The explicit
flags below document the same memory-safety limit:

```bash
cmake --preset project-release
cmake --build --preset project-release --parallel 4
cmake --install build/project-release
```

Available presets:

- `project-release`: production project source, diagnostics disabled;
- `project-test`: project source plus C++/Python tests;
- `project-profile`: stage timing and diagnostic I/O enabled;
- `reference-release`: immutable ETHZ native source behind the same outer I/O;
- `reference-test`: reference source plus applicable tests.

Run tests with:

```bash
cmake --preset project-test
cmake --build --preset project-test --parallel 4
ctest --preset project-test -j4
```

## Public interface

The install exposes one executable:

```text
kalibr-noros calibrate cameras
kalibr-noros calibrate imu-camera
kalibr-noros convert camera
kalibr-noros convert job
kalibr-noros reference verify
```

Calibration accepts a strict `schema_version: 2` task:

```bash
kalibr-noros calibrate cameras \
  --config camera-task.yaml --output-dir output \
  --detector-processes 4 --optimizer-threads 4
```

Both calibration commands support `--parallelism`,
`--detector-processes`, `--optimizer-threads` and safe `--force` overrides.
`--timing-json` is operational only in a `project-profile` build. Default
outputs are `calibration.yaml`, `results.txt` and `report.pdf`.

Convert legacy arguments into a v2 task without running calibration:

```bash
kalibr-noros convert job --type cameras \
  --bag data.bag --target aprilgrid.yaml \
  --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-radtan5 pinhole-radtan5 \
  --output camera-task.yaml
```

Camera YAML conversion delegates to the common OpenCV/Kalibr converter:

```bash
kalibr-noros convert camera import-stereo \
  --stereo opencv.yaml --output camchain.yaml \
  --topics /cam0/image_raw /cam1/image_raw \
  --left-resolution 1920 1080 --right-resolution 1920 1080 \
  --distortion-model radtan5 --translation-unit m
```

Add `--full-fisheye` immediately after `camera` for lossless nonzero
alpha/skew OpenCV fisheye conversion.

## Supported data and models

- ROS1 `.bag` and ROS2 directories containing `metadata.yaml`;
- standard `sensor_msgs/Image`, `CompressedImage` and `Imu`;
- native Kalibr camera models plus OpenCV-order radtan5;
- zero-skew and full `[fu,fv,cu,cv,alpha]` OpenCV fisheye;
- calibrated, scale-misalignment and scale-misalignment-size-effect IMU models.

See [architecture](docs/ARCHITECTURE_ZH.md),
[camera models](docs/CAMERA_MODELS_ZH.md) and
[v2 重构验证报告](docs/V2_REFACTOR_VALIDATION_20260821_ZH.md).
