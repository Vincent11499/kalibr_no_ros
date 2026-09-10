"""Parallel execution and optional profiling for Kalibr calibration.

The module is deliberately small and has no ROS dependency.  Calibration
entry points configure it after parsing their command line. Calibration stages
query it without changing any of Kalibr's numerical defaults.
"""

from .runtime import (
    add_parallelism_arguments,
    apply_optimizer_threads,
    configure,
    detector_inflight_per_worker,
    detector_opencv_threads,
    detector_processes,
    finish,
    get_parallelism,
    optimizer_threads,
    optimizer_threads_or,
    positive_int,
    positive_float,
    profiling_memory_sample_interval_s,
    profiling_enabled,
    record_corner_extraction,
    run_incremental_batch,
    run_optimizer,
    stage,
    timed_call,
    timing_enabled,
)

__all__ = [
    "add_parallelism_arguments",
    "apply_optimizer_threads",
    "configure",
    "detector_inflight_per_worker",
    "detector_opencv_threads",
    "detector_processes",
    "finish",
    "get_parallelism",
    "optimizer_threads",
    "optimizer_threads_or",
    "positive_int",
    "positive_float",
    "profiling_memory_sample_interval_s",
    "profiling_enabled",
    "record_corner_extraction",
    "run_incremental_batch",
    "run_optimizer",
    "stage",
    "timed_call",
    "timing_enabled",
]
