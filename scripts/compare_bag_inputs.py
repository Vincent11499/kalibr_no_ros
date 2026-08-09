#!/usr/bin/env python3
"""Compare calibration inputs exposed by two supported bag backends."""

import argparse
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bag_io" / "src"))

from kalibr_bag_io import BagReader  # noqa: E402


def _assert_equal(actual, expected, label):
    if actual != expected:
        raise AssertionError("{} differs: {!r} != {!r}".format(label, actual, expected))


def _assert_array_equal(actual, expected, label):
    try:
        np.testing.assert_array_equal(actual, expected)
    except AssertionError as error:
        raise AssertionError("{} differs: {}".format(label, error)) from error


def compare_topics(reference, candidate):
    lhs = [(x.name, x.msgtype, x.message_count) for x in reference.topics()]
    rhs = [(x.name, x.msgtype, x.message_count) for x in candidate.topics()]
    _assert_equal(rhs, lhs, "topic inventory")
    print("PASS topics: {} connection(s)".format(len(lhs)))


def compare_images(reference, candidate, topic, sample_count=None):
    with reference.index_images(topic, grayscale=False) as lhs, candidate.index_images(
        topic, grayscale=False
    ) as rhs:
        _assert_equal(len(rhs.index), len(lhs.index), "{} image count".format(topic))
        ignored = {"sequence", "connection_id", "chunk_position", "chunk_offset", "record_ordinal"}
        for position, (left, right) in enumerate(zip(lhs.index, rhs.index)):
            for name in left.__dataclass_fields__:
                if name not in ignored:
                    _assert_equal(
                        getattr(right, name),
                        getattr(left, name),
                        "{} image {} {}".format(topic, position, name),
                    )

        count = len(lhs.index)
        if sample_count is None or sample_count >= count:
            positions = range(count)
        elif sample_count <= 0:
            positions = []
        else:
            positions = sorted(set(np.linspace(0, count - 1, sample_count, dtype=int)))
        checked = 0
        for position in positions:
            left = lhs.get(position)
            right = rhs.get(position)
            _assert_array_equal(right.image, left.image, "{} image {} pixels".format(topic, position))
            checked += 1
        print(
            "PASS {}: {} headers, {} pixel payloads".format(topic, count, checked)
        )


def compare_imu(reference, candidate, topic):
    lhs = reference.read_imu(topic)
    rhs = candidate.read_imu(topic)
    _assert_equal(len(rhs), len(lhs), "{} IMU count".format(topic))
    scalar_fields = ("header_timestamp_ns", "record_timestamp_ns", "frame_id")
    array_fields = (
        "angular_velocity",
        "linear_acceleration",
        "orientation",
        "orientation_covariance",
        "angular_velocity_covariance",
        "linear_acceleration_covariance",
    )
    for position, (left, right) in enumerate(zip(lhs, rhs)):
        for name in scalar_fields:
            _assert_equal(
                getattr(right, name),
                getattr(left, name),
                "{} IMU {} {}".format(topic, position, name),
            )
        for name in array_fields:
            left_value = getattr(left, name)
            right_value = getattr(right, name)
            if left_value is None or right_value is None:
                _assert_equal(right_value, left_value, "{} IMU {} {}".format(topic, position, name))
            else:
                _assert_array_equal(
                    right_value,
                    left_value,
                    "{} IMU {} {}".format(topic, position, name),
                )
    print("PASS {}: {} IMU messages".format(topic, len(lhs)))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Compare ROS1/ROS2 data as seen by the Kalibr input boundary"
    )
    parser.add_argument("reference")
    parser.add_argument("candidate")
    parser.add_argument("--image-topic", action="append", default=[])
    parser.add_argument("--imu-topic", action="append", default=[])
    parser.add_argument(
        "--sample-images",
        type=int,
        help="compare this many evenly-spaced pixel payloads; default compares every image",
    )
    args = parser.parse_args(argv)

    reference = BagReader(args.reference)
    candidate = BagReader(args.candidate)
    compare_topics(reference, candidate)
    for topic in args.image_topic:
        compare_images(reference, candidate, topic, args.sample_images)
    for topic in args.imu_topic:
        compare_imu(reference, candidate, topic)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
