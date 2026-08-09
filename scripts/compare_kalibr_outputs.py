#!/usr/bin/env python3
"""Compare deterministic Kalibr result files or result directories."""

import argparse
import math
from pathlib import Path
import sys

import yaml


RESULT_SUFFIXES = {".txt", ".yaml", ".yml"}


def _compare_value(reference, candidate, path, atol, rtol, differences):
    if isinstance(reference, bool) or isinstance(candidate, bool):
        if reference != candidate:
            differences.append((path, reference, candidate, None))
        return

    if isinstance(reference, (int, float)) and isinstance(candidate, (int, float)):
        reference = float(reference)
        candidate = float(candidate)
        delta = abs(reference - candidate)
        if not math.isclose(reference, candidate, abs_tol=atol, rel_tol=rtol):
            differences.append((path, reference, candidate, delta))
        return

    if isinstance(reference, dict) and isinstance(candidate, dict):
        if reference.keys() != candidate.keys():
            differences.append((path + ".<keys>", sorted(reference), sorted(candidate), None))
            return
        for key in reference:
            _compare_value(reference[key], candidate[key], "{}.{}".format(path, key), atol, rtol, differences)
        return

    if isinstance(reference, list) and isinstance(candidate, list):
        if len(reference) != len(candidate):
            differences.append((path + ".<length>", len(reference), len(candidate), None))
            return
        for index, (left, right) in enumerate(zip(reference, candidate)):
            _compare_value(left, right, "{}[{}]".format(path, index), atol, rtol, differences)
        return

    if reference != candidate:
        differences.append((path, reference, candidate, None))


def compare_file(reference, candidate, atol=1e-12, rtol=1e-12):
    reference = Path(reference)
    candidate = Path(candidate)
    if reference.read_bytes() == candidate.read_bytes():
        return True, "exact byte match"

    if reference.suffix.lower() not in {".yaml", ".yml"}:
        return False, "text differs"

    with reference.open("r", encoding="utf-8") as stream:
        left = yaml.safe_load(stream)
    with candidate.open("r", encoding="utf-8") as stream:
        right = yaml.safe_load(stream)
    differences = []
    _compare_value(left, right, "$", atol, rtol, differences)
    if differences:
        path, expected, actual, delta = differences[0]
        detail = "{}: {!r} != {!r}".format(path, expected, actual)
        if delta is not None:
            detail += " (abs={:.6g})".format(delta)
        return False, detail

    numeric_deltas = []

    def collect_deltas(a, b):
        if isinstance(a, bool) or isinstance(b, bool):
            return
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            numeric_deltas.append(abs(float(a) - float(b)))
        elif isinstance(a, dict):
            for key in a:
                collect_deltas(a[key], b[key])
        elif isinstance(a, list):
            for left_item, right_item in zip(a, b):
                collect_deltas(left_item, right_item)

    collect_deltas(left, right)
    maximum = max(numeric_deltas, default=0.0)
    return True, "numeric YAML match (max abs delta {:.6g})".format(maximum)


def _result_files(path):
    path = Path(path)
    if path.is_file():
        return {path.name: path}
    return {
        item.relative_to(path).as_posix(): item
        for item in sorted(path.rglob("*"))
        if item.is_file() and item.suffix.lower() in RESULT_SUFFIXES
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Compare Kalibr YAML numerically and text reports byte-for-byte"
    )
    parser.add_argument("reference")
    parser.add_argument("candidate")
    parser.add_argument("--atol", type=float, default=1e-12)
    parser.add_argument("--rtol", type=float, default=1e-12)
    args = parser.parse_args(argv)

    reference_files = _result_files(args.reference)
    candidate_files = _result_files(args.candidate)
    if reference_files.keys() != candidate_files.keys():
        missing = sorted(reference_files.keys() - candidate_files.keys())
        extra = sorted(candidate_files.keys() - reference_files.keys())
        if missing:
            print("missing candidate files: {}".format(", ".join(missing)), file=sys.stderr)
        if extra:
            print("extra candidate files: {}".format(", ".join(extra)), file=sys.stderr)
        return 1

    success = True
    for name in reference_files:
        matched, detail = compare_file(
            reference_files[name], candidate_files[name], args.atol, args.rtol
        )
        print("{} {}: {}".format("PASS" if matched else "FAIL", name, detail))
        success = success and matched
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
