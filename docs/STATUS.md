# Native branch implementation and validation status

This document describes `opt/kalibr-native`. The objective is the native
ETHZ-ASL Kalibr algorithm with a ROS-free data boundary, OpenCV-compatible
camera models and optional parallel execution. It is not a Ceres solver port.

## Preservation contract

- The upstream snapshot is commit
  `1f60227442d25e36365ef5f72cd80b9666d73467`.
- All 1,630 upstream files, including ignored test images and `.orig` fixtures,
  are present. `python3 scripts/verify_upstream.py` verifies the complete tree
  against SHA-256
  `9b9c8658d901b6b53772255686513a668aa8d5ca656d2beaaa7edf431d031f0a`.
- `upstream/kalibr` is not edited for the no-ROS port. CMake stages Python
  adapters and applies the native optimizer patches to copies in the build
  tree.
- The original camera and IMU-camera calibration phases, design-variable
  construction, error terms, robustification, native LM/trust-region behavior,
  per-phase damping/tolerance values, state transitions and output flow are
  retained.
- Omitting `--parallelism`, `--detector-processes` and
  `--optimizer-threads` preserves each ETHZ per-stage thread setting. The new
  flags do not select a different optimizer or change its hyperparameters.
- `-DKALIBR_ENABLE_NATIVE_OPTIMIZATIONS=OFF` removes the Hessian/parallel CLI
  overlay for a reference build from the same checkout.
- Diagnostic timers, RSS/PSS sampling, timing JSON I/O and optimizer timing
  printing require `-DKALIBR_ENABLE_PROFILING=ON`; this option is off by
  default and does not affect the parallel algorithm.

The two supported calibration commands are `kalibr_calibrate_cameras` and
`kalibr_calibrate_imu_camera`. Other upstream programs are intentionally not
presented as supported executables.

## Platform and dependency status

| Platform | Toolchain handling | Repository-private dependency path |
| --- | --- | --- |
| Ubuntu 20.04 | Python 3.8, matching Boost.Python component and system SuiteSparse/OpenCV toolchain | `scripts/bootstrap_deps.sh` installs `rosbags` under `.deps/python` |
| Ubuntu 22.04 | Python 3.10, matching Boost.Python component and compatibility for removed `boost/detail/endian.hpp` | The same bootstrap also extracts SuiteSparse and igraph/texttable under `.deps/sysroot` |

The Ubuntu 22.04 bootstrap does not call `apt install` or change the host
package database. It obtains repository candidates with `apt-get download` and
extracts explicit `.deb` files. It is version-gated and must not be described
as an Ubuntu 20.04 bootstrap.

`KALIBR_USE_PRIVATE_DEPS` is on by default. When the private trees exist, CMake
uses them automatically and copies the required Python packages and complete
SuiteSparse runtime symlink chains into build/install artifacts. This solves
the non-inherited ELF RUNPATH issue for transitive SuiteSparse dependencies.
The install still relies on ABI-compatible host fundamentals such as glibc,
libstdc++, OpenCV, Boost and TBB; “relocatable” does not mean a universal static
binary.

## ROS-free boundary

`kalibr_bag_io` uses `rosbags`; it does not load `rosbag`, `rclpy`,
`cv_bridge`, roscpp or a sourced ROS environment. `kalibr_no_ros` adapts the
neutral records to the original Kalibr reader contracts.

| Input | Supported form |
| --- | --- |
| ROS1 | `.bag` file |
| ROS2 | Recording directory with `metadata.yaml`; SQLite3 and MCAP storage are accepted through `rosbags` |
| Camera messages | `sensor_msgs/Image`, `sensor_msgs/CompressedImage` |
| IMU messages | `sensor_msgs/Imu` |

Timestamp ordering, header time, optional clock synchronization, time cropping,
frequency selection, lazy image payload decoding and Kalibr's in-place shuffle
semantics are preserved at the adapter boundary.

Out of scope are a bare `.db3`/`.mcap` with no rosbag2 metadata, custom Snappy
image messages, arbitrary ROS message types and the upstream utilities that
are not one of the two installed calibration commands.

## Headless status

The `sm.PlotCollection` overlay imports wxPython only when an interactive
window is actually requested. Consequently both command parsers and
calibration with `MPLBACKEND=Agg --dont-show-report` work without wxPython,
`DISPLAY` or a sourced desktop/ROS environment. PDF report generation remains
available. Interactive display still requires wxPython and reports a clear
runtime error when it is missing.

## Parallel execution

### CLI precedence and unchanged defaults

Both calibration commands provide:

| Option | Effect |
| --- | --- |
| `--parallelism N` | Common detector-process and optimizer-thread value |
| `--detector-processes N` | Target-extraction process count; overrides the common value |
| `--optimizer-threads N` | Native `Optimizer2`/`IncrementalEstimator` thread option; overrides the common value |
| `--timing-json PATH` | Profiling builds only; structured measurements without changing thread settings |

All counts must be positive. When no parallel option is present, the runtime
hook does not replace ETHZ stage defaults. Explicit optimizer counts are
applied at construction and through the incremental estimator option proxy,
including covariance recovery paths that never fetch the options separately.

### Detection process behavior

- The producer and result queues are bounded.
- At most `2*N` decoded images are in flight.
- Each worker calls `cv2.setNumThreads(1)` to avoid nested OpenCV
  oversubscription.
- Results are restored to original dataset index order.
- Task/result objects are serialized before entering multiprocessing queues,
  so serialization failures are synchronous diagnostics rather than feeder
  thread loss.
- Worker exceptions, malformed results and hard exits terminate with a bounded
  cleanup path instead of waiting indefinitely.

This parallelism applies to image decode/target detection orchestration. It
does not change the detector mathematics or calibration-view order passed to
later stages.

### Deterministic native Hessian construction

For BlockCholesky:

- `nThreads=0/1` executes the original serial
  `ErrorTerm::buildHessian()` loop.
- `nThreads>1` partitions the original error sequence into contiguous chunks;
  each worker accumulates its unchanged error terms into one private sparse
  block matrix and RHS.
- The caller merges contributions strictly in original error-term order;
  shared Hessian/RHS writes remain single-threaded.
- Native square-root covariance, M-estimator weights, design-variable scaling
  and direct-quadratic `BSplineMotionError` handling remain on their original
  expression paths.
- Temporary memory is bounded by one accumulated normal-equation chunk per
  worker and is independent of the number of measurements. It scales with the
  thread count and each chunk's Hessian sparsity/RHS size.

Repeated execution for one fixed parallel configuration is tested for
scheduling-independent output. The serial and parallel paths are tested with a
strict numerical tolerance. There is deliberately no blanket bitwise promise
when the thread count, compiler, CPU, SuiteSparse, BLAS or other numerical
library changes.

Only contribution/Jacobian/Hessian construction is parallelized by this
overlay. CHOLMOD/SPQR factorization and the LM/state-machine logic are
unchanged. `--optimizer-threads` does not fully govern internal threads of
SuiteSparse, BLAS, OpenCV or TBB, so total process thread count can differ from
the requested value.

## Timing JSON

When configured with `-DKALIBR_ENABLE_PROFILING=ON`, `--timing-json` records
command status, total wall time, effective parallel
settings and a stage list. Schema 3 separates image bag read, ROS-message
deserialize, pixel decode, target detection, initialization, residual/problem
construction, optimization and output/report stages. Optimizer2 calls and
incremental `addBatch` calls carry
invocation/batch metadata. Measurements include wall time, CPU time, RSS and,
on Linux when `/proc` exposes it, PSS.

Important interpretation boundaries:

- detector CPU is an aggregation over individual images/workers;
- multiprocessing extraction RSS/PSS has
  `scope: parent_and_detector_workers` and a peak defined as the maximum of
  periodic process-tree samples, so a short peak between samples can be missed;
- optimizer-stage PSS has `scope: main_process`;
- optimizer RSS/PSS deltas are endpoint samples, and optimizer
  `lifetime_peak_at_end` is a process-lifetime RSS high-water mark rather than
  a peak attributable to that stage; and
- stages carrying `metadata.inclusive=true` contain nested stage records and
  must not be added to their children when calculating a total; and
- timing instrumentation reports execution, not convergence quality.

## Camera-model extensions

| CLI model | Kalibr schema | Parameters | OpenCV mapping |
| --- | --- | --- | --- |
| `pinhole-radtan5` | `camera_model: pinhole`, `distortion_model: radtan5` | Intrinsics `[fu,fv,cu,cv]`; D `[k1,k2,p1,p2,k3]` | OpenCV pinhole five-coefficient order |
| `pinhole-opencv-fisheye` | `camera_model: pinhole_opencv_fisheye`, `distortion_model: opencv_fisheye` | Intrinsics `[fu,fv,cu,cv,alpha]`; D `[k1,k2,k3,k4]` | Complete `cv::fisheye` projection, including `K[0,1]=fu*alpha` |

The complete fisheye class has analytic point, projection-parameter and
distortion-parameter Jacobians. Its inverse removes the triangular intrinsic
matrix and solves the fisheye theta polynomial. A separate five-intrinsic
schema is necessary because ETHZ `camera_model: pinhole` validates exactly four
intrinsics and cannot losslessly retain alpha.

The earlier zero-skew fisheye implementation remains as a compatibility layer
and supplies the shared four-coefficient distortion type. The calibration CLI
name `pinhole-opencv-fisheye` maps to the complete five-intrinsic class.

## OpenCV YAML conversion

Two bidirectional commands are installed:

| Command | Intended input/output |
| --- | --- |
| `kalibr_convert_camera_yaml` | radtan, radtan5 and legacy zero-skew OpenCV fisheye |
| `kalibr_convert_opencv_fisheye_yaml` | full OpenCV fisheye with alpha/skew preserved |

Supported input forms include OpenCV FileStorage matrices, ROS-style
rows/cols/data matrices, ordinary YAML sequences, flat 3x3 K/R and flat 4x4
RT. Stereo R/T direction is explicit: OpenCV
`p_cam1 = R*p_cam0 + T` is Kalibr `T_c1_c0`. The converter accepts an inverse
declaration/override and performs the inversion when requested. It also
requires explicit unit metadata or an override when T is not already in
metres. Export emits mono K/D files and adjacent stereo K1/D1/K2/D2/R/T/E/F
files with direction and metre metadata.

### Inspected ROS2 vendor file

The recording
`/mnt/q/File/kalibr/data/evt_8_0730/imu_mipi_in_stereo_calib_08_0730` is a ROS2
SQLite3 bag. Its image topics are `/mipi_image_data_rgb0` and
`/mipi_image_data_rgb1`; the images are 1920x1080. The accompanying
`mipi_in_stereo_calib_08_0730.yaml` is ordinary YAML with flat K1/D1/K2/D2 and
a flat 4x4 RT. D1 and D2 contain five coefficients, so this particular file is
radtan5, not fisheye.

The parser has been exercised against that file and reconstructs the two 3x3
camera matrices and the 4x4 cam0-to-cam1 transform. A reproducible conversion
is:

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

The complete ROS2 `calibrated` IMU-camera optimization and the ROS1 EuRoC
`scale-misalignment` optimization were subsequently rerun with the final
native-parallel implementation.  Their commands, stage timings, memory and
numeric comparisons are recorded in
[`NATIVE_VALIDATION_20260820.md`](NATIVE_VALIDATION_20260820.md).

## Automated coverage

The current CMake/CTest graph includes checks for:

- the full upstream snapshot hash and Python ROS-free adapters;
- headless PlotCollection import and python-igraph compatibility;
- radtan5 projection, analytic Jacobians, geometry/design-variable bindings
  and serialization;
- zero-skew and full-alpha OpenCV fisheye projection against OpenCV, inverse
  behavior and analytic finite-difference Jacobians;
- mono/stereo OpenCV YAML round trips, flat vendor matrices, transform
  direction and translation units;
- parallel CLI precedence, unchanged-default behavior and timing JSON;
- bounded target extraction, result ordering, serialization errors, worker
  exceptions and hard exits; and
- BlockCholesky 0/1 serial identity, 2/4/8-thread repeatability, strict
  serial/parallel numerical agreement, non-unit scaling, M-estimators,
  direct-quadratic errors and observed concurrent evaluation.

The Release native tree has been configured and built on Ubuntu 22.04 with the
repository-private dependency sysroot; install staging and ROS-free/headless
`--help` for both calibration commands have also completed.  The final tree
passed 12/12 CTest cases, relocated-install smoke tests and both dataset-scale
IMU-camera validations.  Dataset comparisons still record the exact build,
thread counts and input/output paths; component-test success alone is not an
end-to-end calibration claim.

## Recorded numerical reference baseline

The following results were recorded earlier for the ROS-free adapter against
the same pinned ETHZ native path. They remain regression reference data, not a
statement that those full bags were rerun after every extension in this branch.

| Case | Recorded comparison |
| --- | --- |
| EuRoC mono camera, deterministic window | YAML and text byte-identical |
| EuRoC stereo camera, deterministic window | Text byte-identical; YAML maximum difference about `4.2e-13` |
| EuRoC mono camera, full bag | Text byte-identical; only final-digit YAML serialization differences |
| EuRoC IMU-camera, `calibrated` | camchain, IMU YAML and text byte-identical |
| EuRoC IMU-camera, `scale-misalignment` | camchain, IMU YAML and text byte-identical |
| EuRoC IMU-camera, `scale-misalignment-size-effect` | camchain, IMU YAML and text byte-identical |

A lossless ROS1-to-ROS2 EuRoC conversion previously compared all 2,899/2,878
camera-recording image messages and all 14,490/14,381 IMU messages, including
pixel payloads and numeric fields. Short-window camera outputs agreed at
approximately `1e-12`, and the full `calibrated` IMU-camera output was
byte-identical. Use `scripts/compare_bag_inputs.py` and
`scripts/compare_kalibr_outputs.py` to reproduce these comparisons for a
specific current build.

## Scope and non-guarantees

- The branch accelerates target extraction and native Hessian/error evaluation;
  it does not replace the native optimizer, change its convergence criteria or
  guarantee that every phase scales linearly with cores.
- Factorization thread control, cross-thread-count bitwise identity and an
  exact continuously sampled whole-process-tree peak PSS are not claimed.
- Only the two stated calibration commands and two YAML converters are
  installed. Rolling-shutter tools, visualization utilities and other upstream
  programs are not part of the supported executable surface.
- Standard raw/compressed camera and IMU messages are supported; custom message
  codecs remain outside the ROS-free boundary.
