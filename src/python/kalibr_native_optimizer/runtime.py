"""Parallelism selection and structured timing for native Kalibr.

Nothing in this module changes an upstream default unless the user explicitly
passes one of the parallelism flags.  Runtime hooks are installed only when an
optimizer override or timing output was requested.
"""

from __future__ import print_function

import argparse
import atexit
import contextlib
import inspect
import json
import os
import resource
import sys
import threading
import time

try:
    from ._build_config import PROFILING_ENABLED as _BUILD_PROFILING_ENABLED
except ImportError:
    # Source-tree imports intentionally model the production default. CMake
    # generates _build_config.py in build/install trees.
    _BUILD_PROFILING_ENABLED = False

_SCHEMA_VERSION = 3
_profiling_enabled = bool(_BUILD_PROFILING_ENABLED)
_parallelism = {
    "parallelism": None,
    "detector_processes": None,
    "optimizer_threads": None,
}
_recorder = None
_atexit_registered = False


def positive_int(value):
    """Argparse type accepting strictly positive integers."""
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("must be a positive integer")
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def add_parallelism_arguments(argument_container):
    """Add controls enabled by this build to a parser or argument group."""
    argument_container.add_argument(
        "--parallelism",
        type=positive_int,
        default=None,
        help=("Set detector process and optimizer thread counts together. "
              "A component-specific option takes precedence."),
    )
    argument_container.add_argument(
        "--detector-processes",
        type=positive_int,
        default=None,
        help=("Number of calibration-target detector worker processes. "
              "The upstream default is retained when omitted."),
    )
    argument_container.add_argument(
        "--optimizer-threads",
        type=positive_int,
        default=None,
        help=("Number of threads for native optimizer phases. The upstream "
              "per-phase defaults are retained when omitted."),
    )
    if _profiling_enabled:
        argument_container.add_argument(
            "--timing-json",
            default=None,
            metavar="PATH",
            help=("Write schema-3 I/O, extraction, initialization, "
                  "problem-build, optimization and output timings to PATH."),
        )


def _argument_value(arguments, name, default=None):
    if isinstance(arguments, dict):
        return arguments.get(name, default)
    return getattr(arguments, name, default)


def _resolve_parallelism(arguments):
    common = _argument_value(arguments, "parallelism")
    detector = _argument_value(arguments, "detector_processes")
    optimizer = _argument_value(arguments, "optimizer_threads")
    if detector is None:
        detector = common
    if optimizer is None:
        optimizer = common
    for name, value in (("parallelism", common),
                        ("detector_processes", detector),
                        ("optimizer_threads", optimizer)):
        if value is not None and (not isinstance(value, int) or value < 1):
            raise ValueError("{0} must be a positive integer".format(name))
    return {
        "parallelism": common,
        "detector_processes": detector,
        "optimizer_threads": optimizer,
    }


def configure(arguments, command=None):
    """Configure this process from parsed CLI arguments.

    This should be called once, immediately after ``parse_args``.  It returns a
    plain dictionary so callers can log the effective settings without taking
    a dependency on implementation classes.
    """
    global _parallelism, _recorder, _atexit_registered
    _parallelism = _resolve_parallelism(arguments)
    _recorder = None

    timing_path = _argument_value(arguments, "timing_json")
    if timing_path and not _profiling_enabled:
        raise RuntimeError(
            "timing output is disabled in this build; configure with "
            "-DKALIBR_ENABLE_PROFILING=ON"
        )
    if timing_path:
        recorder = _TimingRecorder(
            timing_path,
            command=command or os.path.basename(sys.argv[0]),
            parallelism=_parallelism,
        )
        # Publish the recorder only after validating that the destination is
        # writable.  A failed configure must not make later cleanup mask the
        # original I/O error.
        recorder.write()
        _recorder = recorder
        if not _atexit_registered:
            atexit.register(_flush_at_exit)
            _atexit_registered = True
    install_runtime_hooks()
    return get_parallelism()


def get_parallelism():
    return dict(_parallelism)


def detector_processes():
    return _parallelism["detector_processes"]


def optimizer_threads():
    return _parallelism["optimizer_threads"]


def timing_enabled():
    """Return whether structured timing was explicitly requested."""

    return _recorder is not None


def profiling_enabled():
    """Return whether optional diagnostic I/O was compiled into this build."""

    return _profiling_enabled


def optimizer_threads_or(upstream_default):
    """Return an explicit override or the exact upstream stage default."""
    override = optimizer_threads()
    return upstream_default if override is None else override


def apply_optimizer_threads(options):
    """Apply an explicit thread override to an optimizer options object."""
    override = optimizer_threads()
    if override is not None:
        options.nThreads = override
    return options


def _callsite(skip=1):
    frame = inspect.currentframe()
    try:
        for unused_index in range(skip + 1):
            if frame is None:
                break
            frame = frame.f_back
        if frame is None:
            return {"function": "unknown", "file": "unknown"}
        return {
            "function": frame.f_code.co_name,
            "file": os.path.basename(frame.f_code.co_filename),
        }
    finally:
        # Frames participate in reference cycles if retained.
        del frame


def _actual_thread_count(options):
    try:
        return int(options.nThreads)
    except (AttributeError, TypeError, ValueError):
        return None


def run_optimizer(optimizer, *args, **kwargs):
    """Run a real Boost.Python ``Optimizer2`` with optional timing.

    Source overlays call this function at the original optimization callsites.
    No native class or instance is wrapped, so type identity and C++ converter
    behavior remain unchanged.
    """

    if _recorder is None:
        return optimizer.optimize(*args, **kwargs)
    metadata = _callsite(skip=1)
    metadata["optimizer_threads"] = _actual_thread_count(optimizer.options)
    with stage("optimizer2", category="optimization", metadata=metadata):
        return optimizer.optimize(*args, **kwargs)


def run_incremental_batch(estimator, *args, **kwargs):
    """Run ``IncrementalEstimator.addBatch`` without proxying the estimator."""

    if _recorder is None:
        return estimator.addBatch(*args, **kwargs)
    metadata = _callsite(skip=1)
    metadata["optimizer_threads"] = _actual_thread_count(
        estimator.getOptimizerOptions()
    )
    with stage(
        "incremental_estimator.add_batch",
        category="optimization",
        metadata=metadata,
    ):
        return estimator.addBatch(*args, **kwargs)


def timed_call(name, category, metadata, function, *args, **kwargs):
    """Call ``function`` inside a named stage without changing its result."""
    if _recorder is None:
        return function(*args, **kwargs)
    with stage(name, category=category, metadata=metadata):
        return function(*args, **kwargs)


def install_runtime_hooks():
    """Compatibility no-op retained for callers of the former hook API.

    Optimizer integration is now injected into the staged Kalibr Python source
    at explicit callsites.  In particular, this function never replaces a
    Boost.Python class with a Python factory or proxy.
    """


@contextlib.contextmanager
def stage(name, category="optimization", metadata=None):
    """Measure one stage when ``--timing-json`` is active.

    The no-recorder path is intentionally a near-zero-cost context manager and
    remains useful as a clean hook around phases not reached by the automatic
    Optimizer2/IncrementalEstimator wrappers.
    """
    if _recorder is None:
        yield
        return

    token = _recorder.begin_stage(name, category, metadata)
    try:
        yield
    except BaseException:
        _recorder.end_stage(token, "error")
        raise
    else:
        _recorder.end_stage(token, "ok")


def record_corner_extraction(**measurement):
    if _recorder is not None:
        _recorder.record_corner_extraction(measurement)


def finish(status="ok"):
    """Mark and flush timing output. Entry points should call this on success."""
    if _recorder is not None:
        _recorder.finish(status)


def _flush_at_exit():
    if _recorder is not None:
        try:
            _recorder.write()
        except Exception as error:
            sys.stderr.write(
                "warning: failed to flush Kalibr timing JSON: {0}\n".format(
                    error))


def _rss_snapshot():
    current = None
    pss = None
    try:
        with open("/proc/self/status", "r") as status_file:
            for line in status_file:
                if line.startswith("VmRSS:"):
                    current = int(line.split()[1]) * 1024
                    break
    except (IOError, OSError, ValueError, IndexError):
        pass
    try:
        with open("/proc/self/smaps_rollup", "r") as smaps_file:
            for line in smaps_file:
                if line.startswith("Pss:"):
                    pss = int(line.split()[1]) * 1024
                    break
    except (IOError, OSError, ValueError, IndexError):
        pass

    usage = resource.getrusage(resource.RUSAGE_SELF)
    lifetime_peak = int(usage.ru_maxrss)
    # Linux and the BSDs expose KiB; macOS exposes bytes.
    if sys.platform != "darwin":
        lifetime_peak *= 1024
    return {
        "current": current,
        "lifetime_peak": lifetime_peak,
        "pss": pss,
    }


def _measurement_start():
    return {
        "wall": time.perf_counter(),
        "cpu": time.process_time(),
        "rss": _rss_snapshot(),
    }


def _measurement_finish(start):
    end_rss = _rss_snapshot()
    start_current = start["rss"]["current"]
    end_current = end_rss["current"]
    delta = None
    if start_current is not None and end_current is not None:
        delta = end_current - start_current
    start_pss = start["rss"]["pss"]
    end_pss = end_rss["pss"]
    pss_delta = None
    if start_pss is not None and end_pss is not None:
        pss_delta = end_pss - start_pss
    return {
        "wall_seconds": time.perf_counter() - start["wall"],
        "cpu_seconds": time.process_time() - start["cpu"],
        "rss_bytes": {
            "start": start_current,
            "end": end_current,
            "delta": delta,
            # getrusage() is a process-lifetime high-water mark sampled at the
            # end of this stage, not a peak attributable to this stage alone.
            "lifetime_peak_at_end": end_rss["lifetime_peak"],
            "scope": "main_process",
        },
        "pss_bytes": {
            "start": start_pss,
            "end": end_pss,
            "delta": pss_delta,
            "scope": "main_process",
        },
    }


def _json_safe(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return str(value)


class _TimingRecorder(object):
    def __init__(self, path, command, parallelism):
        self.path = os.path.abspath(os.path.expanduser(str(path)))
        self.lock = threading.RLock()
        self.started_wall = time.time()
        self.started_monotonic = time.perf_counter()
        self.data = {
            "schema_version": _SCHEMA_VERSION,
            "command": command,
            "status": "running",
            "started_unix_seconds": self.started_wall,
            "parallelism": dict(parallelism),
            "stages": [],
        }
        self._repeated_stages = {}
        self._stage_local = threading.local()

    def begin_stage(self, name, category, metadata):
        stack = getattr(self._stage_local, "stack", None)
        if stack is None:
            stack = []
            self._stage_local.stack = stack
        token = {
            "name": str(name),
            "category": str(category),
            "metadata": _json_safe(metadata or {}),
            "measurement": _measurement_start(),
            "depth": len(stack),
            "child_wall_seconds": 0.0,
            "parent": stack[-1] if stack else None,
        }
        stack.append(token)
        return token

    def end_stage(self, token, status):
        result = _measurement_finish(token["measurement"])
        stack = getattr(self._stage_local, "stack", [])
        if not stack or stack[-1] is not token:
            raise RuntimeError("timing stages ended out of order")
        stack.pop()
        if token["parent"] is not None:
            token["parent"]["child_wall_seconds"] += result["wall_seconds"]
        exclusive_wall = max(
            0.0, result["wall_seconds"] - token["child_wall_seconds"])
        entry = {
            "name": token["name"],
            "category": token["category"],
            "status": status,
            "wall_seconds": result["wall_seconds"],
            "exclusive_wall_seconds": exclusive_wall,
            "cpu_seconds": result["cpu_seconds"],
            "rss_bytes": result["rss_bytes"],
            "pss_bytes": result["pss_bytes"],
            "metadata": token["metadata"],
            "nesting_depth": token["depth"],
        }
        with self.lock:
            if token["name"] == "incremental_estimator.add_batch":
                self._merge_repeated_stage(entry)
            else:
                self.data["stages"].append(entry)

    def _merge_repeated_stage(self, entry):
        """Aggregate per-view calls so profile files stay compact."""
        key = (
            entry["category"],
            entry["name"],
            json.dumps(entry["metadata"], sort_keys=True),
        )
        aggregate = self._repeated_stages.get(key)
        if aggregate is None:
            entry["aggregation"] = {
                "kind": "repeated_calls",
                "count": 1,
                "wall_seconds_min": entry["wall_seconds"],
                "wall_seconds_max": entry["wall_seconds"],
                "wall_seconds_mean": entry["wall_seconds"],
            }
            self._repeated_stages[key] = entry
            self.data["stages"].append(entry)
            return

        count = aggregate["aggregation"]["count"] + 1
        wall = entry["wall_seconds"]
        aggregate["wall_seconds"] += wall
        aggregate["exclusive_wall_seconds"] += entry["exclusive_wall_seconds"]
        aggregate["cpu_seconds"] += entry["cpu_seconds"]
        aggregate["aggregation"]["count"] = count
        aggregate["aggregation"]["wall_seconds_min"] = min(
            aggregate["aggregation"]["wall_seconds_min"], wall)
        aggregate["aggregation"]["wall_seconds_max"] = max(
            aggregate["aggregation"]["wall_seconds_max"], wall)
        aggregate["aggregation"]["wall_seconds_mean"] = (
            aggregate["wall_seconds"] / count)
        if entry["status"] != "ok":
            aggregate["status"] = entry["status"]
        for field in ("rss_bytes", "pss_bytes"):
            first = aggregate[field]
            last = entry[field]
            first["end"] = last.get("end")
            if first.get("start") is not None and first.get("end") is not None:
                first["delta"] = first["end"] - first["start"]
        first_peak = aggregate["rss_bytes"].get("lifetime_peak_at_end")
        last_peak = entry["rss_bytes"].get("lifetime_peak_at_end")
        if first_peak is not None and last_peak is not None:
            aggregate["rss_bytes"]["lifetime_peak_at_end"] = max(
                first_peak, last_peak)

    def record_corner_extraction(self, measurement):
        measurement = dict(measurement)
        phases = {
            "bag_read": {
                "wall_seconds": float(measurement.pop(
                    "bag_read_wall_seconds", 0.0)),
                "cpu_seconds": float(measurement.pop(
                    "bag_read_cpu_seconds", 0.0)),
            },
            "deserialize": {
                "wall_seconds": float(measurement.pop(
                    "deserialize_wall_seconds", 0.0)),
                "cpu_seconds": float(measurement.pop(
                    "deserialize_cpu_seconds", 0.0)),
            },
            "decode": {
                "wall_seconds": float(measurement.pop("decode_wall_seconds", 0.0)),
                "cpu_seconds": float(measurement.pop("decode_cpu_seconds", 0.0)),
            },
            "detect": {
                "wall_seconds": float(measurement.pop("detect_wall_seconds", 0.0)),
                "cpu_seconds": float(measurement.pop("detect_cpu_seconds", 0.0)),
                "aggregation": measurement.pop(
                    "detect_aggregation", "sum_over_images"),
            },
            "total": {
                "wall_seconds": float(measurement.pop("total_wall_seconds", 0.0)),
                "cpu_seconds": float(measurement.pop("total_cpu_seconds", 0.0)),
            },
        }
        entry = {
            "name": "corner_extraction",
            "category": "extraction",
            "status": measurement.pop("status", "ok"),
            "phases": phases,
            "rss_bytes": _json_safe(measurement.pop("rss_bytes", {})),
            "pss_bytes": _json_safe(measurement.pop("pss_bytes", {})),
            "metadata": _json_safe(measurement),
        }
        with self.lock:
            self.data["stages"].append(entry)

    def finish(self, status):
        with self.lock:
            self.data["status"] = str(status)
            self.data["finished_unix_seconds"] = time.time()
            self.data["total_wall_seconds"] = (
                time.perf_counter() - self.started_monotonic)
            self.data["summary"] = self._summary()
            self.write()

    def _summary(self):
        total = self.data.get("total_wall_seconds", 0.0)
        categories = {}
        stage_totals = {}
        for entry in self.data["stages"]:
            wall = entry.get("wall_seconds")
            if wall is None:
                wall = entry.get("phases", {}).get("total", {}).get(
                    "wall_seconds", 0.0)
            wall = float(wall or 0.0)
            exclusive_wall = float(entry.get("exclusive_wall_seconds", wall) or 0.0)
            category = entry["category"]
            name = entry["name"]
            categories[category] = categories.get(category, 0.0) + exclusive_wall
            stage_key = "{0}/{1}".format(category, name)
            aggregate = stage_totals.setdefault(
                stage_key, {"inclusive": 0.0, "exclusive": 0.0})
            aggregate["inclusive"] += wall
            aggregate["exclusive"] += exclusive_wall

        def category_rows(values):
            return [
                {
                    "name": name,
                    "wall_seconds": wall,
                    "percent_of_total": (
                        100.0 * wall / total if total > 0.0 else 0.0),
                }
                for name, wall in sorted(
                    values.items(), key=lambda item: (-item[1], item[0]))
            ]

        def stage_rows(values):
            return [
                {
                    "name": name,
                    "wall_seconds": value["inclusive"],
                    "exclusive_wall_seconds": value["exclusive"],
                    "percent_of_total": (
                        100.0 * value["inclusive"] / total
                        if total > 0.0 else 0.0),
                    "exclusive_percent_of_total": (
                        100.0 * value["exclusive"] / total
                        if total > 0.0 else 0.0),
                }
                for name, value in sorted(
                    values.items(),
                    key=lambda item: (-item[1]["inclusive"], item[0]))
            ]

        instrumented = sum(categories.values())

        return {
            "category_totals": category_rows(categories),
            "stage_totals": stage_rows(stage_totals),
            "instrumented_exclusive_wall_seconds": instrumented,
            "unattributed_wall_seconds": max(0.0, total - instrumented),
            "instrumented_percent_of_total": (
                100.0 * instrumented / total if total > 0.0 else 0.0),
            "timing_semantics": (
                "category totals use exclusive wall time and do not double-count "
                "nested stages; stage totals expose both inclusive and exclusive "
                "wall time"
            ),
        }

    def write(self):
        with self.lock:
            directory = os.path.dirname(self.path) or os.curdir
            if not os.path.isdir(directory):
                raise IOError(
                    "timing output directory does not exist: {0}".format(directory))
            temporary = "{0}.tmp.{1}".format(self.path, os.getpid())
            try:
                with open(temporary, "w") as output:
                    json.dump(self.data, output, indent=2, sort_keys=True)
                    output.write("\n")
                os.replace(temporary, self.path)
            except Exception:
                try:
                    os.unlink(temporary)
                except OSError:
                    pass
                raise


def _reset_for_tests():
    """Clear process-global runtime state."""
    global _parallelism, _profiling_enabled, _recorder
    _parallelism = {
        "parallelism": None,
        "detector_processes": None,
        "optimizer_threads": None,
    }
    _profiling_enabled = bool(_BUILD_PROFILING_ENABLED)
    _recorder = None


def _set_profiling_for_tests(enabled):
    """Override the generated build setting in isolated unit tests only."""

    global _profiling_enabled
    _profiling_enabled = bool(enabled)
