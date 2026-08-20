# Native optimizer Python integration

The native runtime is an independent package under
`extensions/native_optimizer/python/kalibr_native_optimizer`. It leaves every
ETHZ thread default unchanged until a user explicitly supplies
`--parallelism`, `--detector-processes`, or `--optimizer-threads`.

The build integration needs three small operations when
`KALIBR_ENABLE_NATIVE_OPTIMIZATIONS` is enabled:

1. Copy and install the `kalibr_native_optimizer` package next to the existing
   Kalibr Python packages.
2. Keep copying `python/kalibr_common/TargetExtractor.py` over the staged
   upstream module.
3. After `prepare_cli` has performed the ROS/radtan model substitutions and
   written each libexec command, run `cli_overlay.py` in place:

```cmake
execute_process(
  COMMAND "${PYTHON_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/native_optimizer/python/kalibr_native_optimizer/cli_overlay.py"
    --command "${name}" --input "${KALIBR_LIBEXEC_DIR}/${name}"
  RESULT_VARIABLE native_cli_result)
if(NOT native_cli_result EQUAL 0)
  message(FATAL_ERROR "Failed to generate native CLI overlay for ${name}")
endif()
```

The transformer supports exactly `kalibr_calibrate_cameras` and
`kalibr_calibrate_imu_camera`, validates all source anchors, and is idempotent.
It adds the three parallel flags and configures the runtime immediately after
argument parsing. A build generated with `KALIBR_ENABLE_PROFILING=ON` also adds
`--timing-json`; only that build flushes timing data around `main()`.

Four staged calibration modules are additionally passed through
`source_overlay.py`: `CameraIntializers.py`, `CameraCalibrator.py`,
`IccSensors.py`, and `IccCalibrator.py`.  The overlay inserts calls to
`apply_optimizer_threads`, `run_optimizer`, and `run_incremental_batch` at the
original callsites.  It does not replace `aslam_backend.Optimizer2`,
`incremental_calibration.IncrementalEstimator`, their options, or their
instances, so all objects retain their native Boost.Python types.

For an optimization phase that does not construct `Optimizer2` directly and
does not call `IncrementalEstimator.addBatch`, wrap it explicitly without
altering its options:

```python
with native_runtime.stage("phase_name", category="optimization"):
    run_phase()
```

Direct `Optimizer2` phases and incremental `addBatch` calls in the two supported
calibration pipelines are timed by the explicit source callsites.  Thread
overrides are applied to the real options object immediately before the
corresponding optimization, after upstream code has assigned its normal
per-stage default.
