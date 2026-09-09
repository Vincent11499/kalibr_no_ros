# Kalibr no-ROS v1.0.0

ROS-free packaging of the native ETHZ-ASL Kalibr camera and camera–IMU
calibration algorithms. The optimizer state machines, residual definitions,
parameter activation, numerical hyperparameters and stopping conditions remain
compatible with the pinned ETHZ implementation.

## Source layout

- `ref/kalibr`: immutable 1,630-file ETHZ snapshot at commit
  `1f60227442d25e36365ef5f72cd80b9666d73467`, used only by reference builds.
- `src/kalibr`: editable production source, grouped into foundation, camera,
  optimization, trajectory, calibration and third-party domains.
- `src/python`: ROS-free dataset I/O, public CLI and runtime instrumentation.
- `src/camera_models`: OpenCV radtan5/radtan8 and fisheye model extensions.
- `config`: concise camera, camera–IMU and directory-dataset YAML templates.
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
kalibr-noros validate
kalibr-noros evaluate
```

Performance candidates can be archived and compared without rerunning the
frozen baseline:

```bash
kalibr-noros benchmark run \
  --config task.yaml --archive-dir benchmark-runs --name candidate
kalibr-noros benchmark compare \
  --registry benchmarks/baselines-v1.yaml \
  --baseline-id euroc_camera_project_4 \
  --candidate benchmark-runs/candidate/<run-id>
```

See `docs/BENCHMARK_ZH.md` for the fixed inputs, parameter precedence and
archive format.

Calibration accepts a strict `schema_version: "1.0.0"` task. Project datasets,
initialization and result documents use the same version; historical project
schemas are not accepted as runtime input. The dataset type is
explicitly `bag` or `directory`; ROS1/ROS2 bag storage is detected internally:

```bash
kalibr-noros calibrate cameras \
  --config camera-task.yaml --output-dir output \
  --detector-processes 4 --optimizer-threads 4
```

Both calibration commands support `--parallelism`,
`--detector-processes`, `--optimizer-threads` and safe `--force` overrides.
`--timing-json` is operational only in a `project-profile` build. Default
outputs include `calibration.yaml`, `metrics.json`, `assessment.json`,
`results.txt`, `report.html`, `report.pdf`, `run_manifest.json` and
`task_resolved.yaml`. Detailed observations and image visualizations can be
enabled in an ordinary release build through the task's `output` block.

Annotated and minimal `pinhole-equi` examples for mono, stereo and stereo–IMU
calibration live in [`config/examples/v1.0.0`](config/examples/v1.0.0/README_ZH.md).
The [v1.0.0 interface guide](docs/V1_INTERFACE_ZH.md) describes the capture
contract, quality rules and offline report regeneration.

Directory tasks select cameras by `id` from `dataset.yaml`; camera `topic`
and IMU `rostopic` can be omitted. Bag tasks still require topics. See the
[input and output file guide](docs/DATASET_WORKFLOW_ZH.md) for configuration
dependencies and observation/visualization file meanings.

Convert legacy arguments into a task without running calibration:

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

- ROS1 `.bag`、ROS2 directories containing `metadata.yaml`, and manifest-based
  directory datasets containing `dataset.yaml`;
- standard `sensor_msgs/Image`, `CompressedImage` and `Imu`;
- directory images supported by the installed OpenCV codecs, including PNG,
  JPEG/JPG and BMP, plus strict nanosecond camera/IMU CSV tables;
- native Kalibr camera models plus OpenCV-order radtan5 and rational radtan8;
- zero-skew and full `[fu,fv,cu,cv,alpha]` OpenCV fisheye;
- calibrated, scale-misalignment and scale-misalignment-size-effect IMU models.

中文资料从[文档导航](docs/README_ZH.md)开始；日常配置见
[使用与配置指南](docs/USER_GUIDE_ZH.md)，源码原理见
[源码深入导读](docs/SOURCE_CODE_DEEP_DIVE_ZH.md)，历史验证结果集中保存在
[`docs/reports`](docs/reports)。可直接复制的最简配置见
[`config/examples/v1.0.0/README_ZH.md`](config/examples/v1.0.0/README_ZH.md)。
