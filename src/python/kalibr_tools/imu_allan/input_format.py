"""Validate IMU CSV input and select a relative half-open time interval."""

from __future__ import annotations

import csv
from decimal import Decimal, DecimalException
import math
from pathlib import Path

from kalibr_no_ros.version import RESULT_SCHEMA_VERSION


COLUMNS = ["timestamp_ns", "wx", "wy", "wz", "ax", "ay", "az"]


def add_time_arguments(parser):
    start = parser.add_mutually_exclusive_group()
    start.add_argument("--start-s", help="inclusive offset from the first sample, seconds (default: 0)")
    start.add_argument("--start-h", help="inclusive offset from the first sample, hours")
    end = parser.add_mutually_exclusive_group()
    end.add_argument("--end-s", help="exclusive offset from the first sample, seconds (default: end of file)")
    end.add_argument("--end-h", help="exclusive offset from the first sample, hours")


def time_bounds(args):
    def convert(seconds, hours):
        value = hours if hours is not None else seconds
        if value is None:
            return None
        try:
            number = Decimal(value)
            if not number.is_finite() or number < 0:
                raise ValueError("time bounds must be finite and non-negative")
            nanoseconds = number * (3600 if hours is not None else 1) * 1_000_000_000
            if nanoseconds != nanoseconds.to_integral_value():
                raise ValueError("time bounds must have nanosecond precision or coarser")
            return int(nanoseconds)
        except DecimalException as exc:
            raise ValueError("invalid time bound: {}".format(value)) from exc

    start = convert(args.start_s, args.start_h)
    end = convert(args.end_s, args.end_h)
    start = 0 if start is None else start
    if end is not None and end <= start:
        raise ValueError("end must be later than start")
    return start, end


def resolve_dataset(path):
    source = Path(path).expanduser().resolve()
    if source.is_file():
        dataset = source.parent.parent if source.parent.name == "imu" else source.parent
        return dataset, source
    csv_path = source / "imu" / "imu0.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)
    return source, csv_path.resolve()


def rows(csv_path):
    with csv_path.open(encoding="utf-8", newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader, None)
        if header not in (COLUMNS, COLUMNS + ["temperature"], COLUMNS + ["temperature_c"]):
            raise ValueError("expected IMU CSV columns: " + ",".join(COLUMNS) + "[,temperature|temperature_c]")
        previous = None
        for line_number, row in enumerate(reader, 2):
            if len(row) != len(header):
                raise ValueError(f"wrong column count at line {line_number}")
            try:
                stamp = int(row[0])
                values = tuple(float(value) for value in row[1:])
            except ValueError as exc:
                raise ValueError(f"invalid numeric value at line {line_number}") from exc
            if stamp < 0:
                raise ValueError(f"negative timestamp at line {line_number}")
            if previous is not None and stamp <= previous:
                raise ValueError(f"non-increasing timestamp at line {line_number}")
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"non-finite value at line {line_number}")
            previous = stamp
            yield stamp, values


def selected_rows(csv_path, start_ns=0, end_ns=None):
    origin = None
    for stamp, values in rows(csv_path):
        if origin is None:
            origin = stamp
        offset = stamp - origin
        if end_ns is not None and offset >= end_ns:
            break
        if offset >= start_ns:
            yield stamp, values


def validate_input(dataset, csv_path, start_ns=0, end_ns=None):
    """Validate every row, including samples outside the selected interval."""
    if type(start_ns) is not int or (end_ns is not None and type(end_ns) is not int):
        raise ValueError("selection offsets must be integer nanoseconds")
    if start_ns < 0 or (end_ns is not None and end_ns <= start_ns):
        raise ValueError("invalid selection range")
    first = last = origin = source_last = None
    count = total = before = after = 0
    minimum_interval_ns = math.inf
    maximum_interval_ns = 0
    for stamp, _values in rows(csv_path):
        if origin is None:
            origin = stamp
        total += 1
        source_last = stamp
        offset = stamp - origin
        if offset < start_ns:
            before += 1
            continue
        if end_ns is not None and offset >= end_ns:
            after += 1
            continue
        if first is None:
            first = stamp
        if last is not None:
            interval = stamp - last
            minimum_interval_ns = min(minimum_interval_ns, interval)
            maximum_interval_ns = max(maximum_interval_ns, interval)
        last = stamp
        count += 1
    if count < 2:
        raise ValueError("at least two samples are required in the selected interval")
    duration_s = (last - first) / 1e9
    return {
        "schema_version": RESULT_SCHEMA_VERSION,
        "kind": "allan_variance_python_input_validation",
        "status": "pass",
        "source": str(csv_path),
        "dataset_id": dataset.name,
        "messages": count,
        "first_timestamp_ns": first,
        "last_timestamp_ns": last,
        "duration_s": duration_s,
        "effective_rate_hz": (count - 1) / duration_s,
        "minimum_interval_ns": int(minimum_interval_ns),
        "maximum_interval_ns": int(maximum_interval_ns),
        "selection": {
            "origin_timestamp_ns": origin,
            "start_offset_ns": start_ns,
            "end_offset_ns": end_ns,
            "start_s": start_ns / 1e9,
            "end_s": None if end_ns is None else end_ns / 1e9,
            "interval": "[start,end)",
            "source_messages": total,
            "excluded_before": before,
            "excluded_after": after,
            "source_last_timestamp_ns": source_last,
            "source_duration_s": (source_last - origin) / 1e9,
            "complete_source_checked": True,
            "timestamps_rebased": False,
        },
        "sampling_note": (
            "The original algorithm bins by sample index at floor(effective_rate_hz); "
            "timestamps are not resampled or interpolated."
        ),
        "temperature_note": "temperature is validated when present but is not used in the Allan calculation",
    }


def materialize_selected_samples(csv_path, target, count, start_ns, end_ns):
    """Write [ax, ay, az, wx_deg, wy_deg, wz_deg] to a disk-backed array."""
    import numpy as np

    data = np.lib.format.open_memmap(target, mode="w+", dtype=np.float64, shape=(count, 6))
    radians_to_degrees = 180.0 / np.pi
    written = 0
    try:
        for _stamp, values in selected_rows(csv_path, start_ns, end_ns):
            if written >= count:
                raise RuntimeError("selected sample count changed while reading the source")
            wx, wy, wz, ax, ay, az = values[:6]
            data[written] = (ax, ay, az, wx * radians_to_degrees, wy * radians_to_degrees, wz * radians_to_degrees)
            written += 1
        data.flush()
    finally:
        del data
    if written != count:
        raise RuntimeError(f"selected sample count changed: expected {count}, wrote {written}")
