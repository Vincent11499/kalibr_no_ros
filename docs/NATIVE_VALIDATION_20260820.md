# Native parallel validation — 2026-08-20

This report validates the ROS-free native optimizer on the two fixed datasets
used for development.  All candidate outputs were written below
`/tmp/kalibr-native-e2e.uiZ7vx`; none of the reference output directories were
modified.

## Build and invariants

- Branch: `opt/kalibr-native`
- Host: Ubuntu 22.04, Python 3.10, 32 logical CPUs
- Release build: `/tmp/kalibr-native-audit`
- Parallel settings for the full IMU-camera runs:
  `--detector-processes 4 --optimizer-threads 4`
- CTest: 12/12 passed after the final Hessian implementation was built.
- The installed and relocated camera, IMU-camera and two YAML-converter
  commands all completed `--help` with ROS variables and `PYTHONPATH` unset.

The parallel implementation does not replace `Optimizer2`,
`IncrementalEstimator`, LM, GN, BlockCholesky or SPQR.  It retains the ETHZ
problem construction, stage order, active parameters, lambda values,
accept/revert policy and absolute stopping conditions.  With more than one
optimizer thread, fixed contiguous error chunks are assembled concurrently;
each chunk preserves original error order and chunks are reduced in fixed
index order.  This changes floating-point parenthesization at chunk boundaries,
so comparison across different thread counts is numerical, not bytewise.

## Full-dataset results

| Dataset and model | Reference native wall | Candidate wall | Candidate measured stages | Reference/candidate max RSS |
| --- | ---: | ---: | --- | ---: |
| ROS1 EuRoC, `scale-misalignment` | 150.64 s | 136.44 s | extraction 13.57 + 19.32 s; final optimizer 37.03 s | 1,163,896 / 1,342,532 KiB |
| ROS2 EVT, `calibrated` | 262.33 s | 263.81 s | extraction 61.29 + 54.61 s; final optimizer 54.80 s | 1,081,592 / 1,335,520 KiB |

The ROS1 candidate is about 9.4% faster with only four optimizer threads.  Its
peak RSS is about 15.3% higher.  The ROS2 candidate is within 0.6% of the old
total wall time even though the old run used the host-dependent upstream
`CPU-1` defaults and the candidate deliberately limited target detection to
four processes.  Its peak RSS is about 23.5% higher.  Increasing only
`--detector-processes` is the appropriate next performance setting for the
1920x1080 ROS2 images; `--optimizer-threads 4` already reduces the final solve
to 54.80 s.

`/usr/bin/time` supplies the comparable process-wall and peak-RSS values in the
table.  Timing JSON schema 3 additionally records stage CPU/wall values and
separates bag read, deserialization, image decode, target detection,
initialization, problem construction, optimization and output. It also records
effective worker counts and Linux PSS. During this WSL/virtualized run its
ROS2 command-total monotonic measurement was 271.53 s while `/usr/bin/time`
reported 263.81 s; the stage measurements and process-wall measurement are
therefore reported separately rather than silently mixing the two clocks.

### ROS1 EuRoC numerical comparison

- Problem: 14,428 design variables, 375,517 error terms,
  Jacobian 779,794 x 64,912.
- Both runs stopped at accepted iteration 13 because absolute
  `dJ < 0.01`; neither reached the iteration limit.
- Cost path: `2.44447e6 -> 74693.5` in both runs.
- Candidate residuals:
  - camera means: 0.368134 / 0.388292 px;
  - gyro mean: 0.00548025 rad/s;
  - accelerometer mean: 0.0473168 m/s2.
- Against `output_native`:
  - camchain YAML maximum absolute difference: `6.90e-10`;
  - each camera translation L2 difference: `2.70e-10 m`;
  - rotation-matrix Frobenius difference: `7.36e-11` (geodesic angle
    rounds to zero at double precision);
  - time-shift absolute differences: `6.90e-10 s` and `8.78e-11 s`;
  - IMU YAML maximum absolute difference: `2.38e-10`, with aggregate numeric
    L2 difference `5.55e-10`.

### ROS2 EVT numerical comparison

- Problem: 22,300 design variables, 306,009 error terms,
  Jacobian 633,640 x 100,330.
- Both runs follow the same 30 displayed LM steps, including every rejected
  step and rollback.
- Cost path: `2.78974e7 -> 172132` at the final displayed step in both runs.
- Candidate residuals:
  - camera means: 0.512383 / 0.478737 px;
  - gyro mean: 0.00132308 rad/s;
  - accelerometer mean: 0.0596595 m/s2.
- Candidate time shifts: 13.5209 ms and 14.8606 ms.
- Against `output_native`:
  - camchain YAML maximum absolute difference: `1.43e-9`;
  - each camera translation L2 difference: `1.54e-9 m`;
  - rotation-matrix Frobenius difference: `1.29e-10` (geodesic angle
    rounds to zero at double precision);
  - time-shift absolute differences: `2.92e-12 s` and `1.97e-12 s`;
  - IMU YAML: exact byte match.

## Hessian performance correction

An initial deterministic ring implementation produced correct ROS2 results
but spent 693.66 s in the final optimizer.  It allocated/deallocated sparse
blocks and scanned the complete block/RHS layout once per error term.  The
final contiguous-chunk implementation reduced the same four-thread optimizer
stage to 54.80 s, about 12.7x faster, while retaining the same LM trajectory.

The independent 200,000-error assembly benchmark recorded:

| Threads | Median assembly | Speedup | Peak RSS |
| ---: | ---: | ---: | ---: |
| 1 | 0.880 s | 1.00x | 265.5 MiB |
| 2 | 0.562 s | 1.57x | 277.1 MiB |
| 4 | 0.308 s | 2.85x | 290.2 MiB |
| 8 | 0.218 s | 4.04x | 313.7 MiB |

Tests cover the exact 0/1-thread native path, fixed-thread repeatability,
strict serial/parallel numerical agreement, M-estimators, non-unit design
variable scaling, direct-quadratic terms, worker exceptions and a 10,000-error
stress case.

## Camera-model smoke tests

- EuRoC stereo `pinhole-radtan5`, deterministic 3 s window: 61/61 views per
  camera, final reprojection standard deviations about 0.070/0.065 px for cam0
  and 0.066/0.062 px for cam1; total recorded wall 46.66 s.
- EuRoC mono full OpenCV fisheye, the same window: 61 detections and 46 accepted
  views; reprojection standard deviation about 0.058/0.060 px; total wall
  15.58 s.  The result contains five intrinsics
  `[fu,fv,cu,cv,alpha]`, and the OpenCV sidecar preserves
  `K[0,1] = fu*alpha`.

The full fisheye C++ regression also compares projection against
`cv::fisheye`, exercises analytic finite-difference Jacobians and failure
sentinels, while the YAML tests cover ordinary YAML, OpenCV FileStorage,
flat matrices, stereo `RT`, transform direction, translation units and
bidirectional alpha/skew round trips.

## Reproduction outputs

The subsequent schema-3 rerun splits bag access, deserialization, image decode,
target detection, initialization, graph construction, optimization and output.
Its full Chinese analysis and optimization priority are recorded in
[`NATIVE_STAGE_TIMING_REPORT_20260820_ZH.md`](NATIVE_STAGE_TIMING_REPORT_20260820_ZH.md).

The temporary result set includes:

- `euroc-imu-scale-misalignment-timing.json` and
  `euroc-imu-scale-misalignment-time.txt`;
- `imu_april-{camchain-imucam,imu,results-imucam}`;
- `evt-chunk-timing.json` and `evt-chunk-time.txt`; and
- `evt_ros2_chunk-{camchain-imucam,imu,results-imucam}`.

Use `tools/compare_kalibr_outputs.py` for YAML comparisons.  Text reports
contain full-precision floating-point formatting and are expected to show the
same last-digit differences as the YAML when thread count changes.
