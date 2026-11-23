"""Resolve task and CLI execution budgets without changing solver defaults."""

from .document import TaskError

STANDARD_EXECUTION = {
    "detector_processes": 4,
    "optimizer_threads": 4,
    "detector_inflight_per_worker": 2,
    "detector_opencv_threads": 1,
    "profiling_memory_sample_interval_s": 0.25,
}


def resolve_execution(configured=None, overrides=None, standard_defaults=False):
    """Resolve CLI-over-YAML execution values without changing algorithms."""
    configured = dict(configured or {})
    overrides = dict(overrides or {})
    allowed = {
        "parallelism", "detector_processes", "optimizer_threads",
        "detector_inflight_per_worker", "detector_opencv_threads",
        "profiling_memory_sample_interval_s",
    }
    unknown = sorted(set(configured) - allowed)
    if unknown:
        raise TaskError(
            "unknown execution fields: {}".format(", ".join(unknown)))

    cli_common = overrides.get("parallelism")
    yaml_common = configured.get("parallelism")
    result = {}
    for key in ("detector_processes", "optimizer_threads"):
        value = overrides.get(key)
        if value is None:
            value = cli_common
        if value is None:
            value = configured.get(key)
        if value is None:
            value = yaml_common
        if value is None and standard_defaults:
            value = STANDARD_EXECUTION[key]
        if value is not None:
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise TaskError("execution.{} must be a positive integer".format(key))
            result[key] = value

    for key in ("detector_inflight_per_worker", "detector_opencv_threads"):
        value = overrides.get(key)
        if value is None:
            value = configured.get(key, STANDARD_EXECUTION[key])
        if not isinstance(value, int) or isinstance(value, bool) or value < 1:
            raise TaskError("execution.{} must be a positive integer".format(key))
        result[key] = value

    key = "profiling_memory_sample_interval_s"
    value = overrides.get(key)
    if value is None:
        value = configured.get(key, STANDARD_EXECUTION[key])
    if (not isinstance(value, (int, float)) or isinstance(value, bool)
            or value <= 0.0 or value == float("inf")):
        raise TaskError("execution.{} must be a positive finite number".format(key))
    result[key] = float(value)
    return result
