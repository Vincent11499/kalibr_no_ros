#!/usr/bin/env python3
"""Benchmark bag indexing, payload reads, deserialization and image decoding."""

import argparse
import json
import resource
import sys
import time

from kalibr_bag_io import BagReader


def _peak_rss_bytes():
    value = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    return value if sys.platform == "darwin" else value * 1024


def benchmark_topic(bag_path, topic):
    reader = BagReader(bag_path)

    wall_start = time.perf_counter()
    cpu_start = time.process_time()
    dataset = reader.index_images(topic)
    index_wall = time.perf_counter() - wall_start
    index_cpu = time.process_time() - cpu_start

    bag_read_wall = 0.0
    bag_read_cpu = 0.0
    deserialize_wall = 0.0
    deserialize_cpu = 0.0
    decode_wall = 0.0
    decode_cpu = 0.0
    raw_bytes = 0
    decoded_bytes = 0
    checksum = 0
    total_wall_start = time.perf_counter()
    total_cpu_start = time.process_time()
    try:
        for entry in dataset.index:
            payload, read_timing = (
                dataset.get_deferred_by_entry_with_timing(entry))
            image, image_timing = payload.decode_for_kalibr(True)
            bag_read_wall += read_timing["bag_read_wall_seconds"]
            bag_read_cpu += read_timing["bag_read_cpu_seconds"]
            deserialize_wall += image_timing["deserialize_wall_seconds"]
            deserialize_cpu += image_timing["deserialize_cpu_seconds"]
            decode_wall += image_timing["decode_wall_seconds"]
            decode_cpu += image_timing["decode_cpu_seconds"]
            raw_bytes += len(payload.rawdata)
            decoded_bytes += int(image.nbytes)
            if image.size:
                checksum = (checksum + int(image.flat[0])) & 0xFFFFFFFF
    finally:
        dataset.close()

    retrieval_wall = time.perf_counter() - total_wall_start
    retrieval_cpu = time.process_time() - total_cpu_start
    return {
        "topic": topic,
        "storage_format": reader.storage_format,
        "messages": len(dataset.index),
        "raw_payload_bytes": raw_bytes,
        "decoded_image_bytes": decoded_bytes,
        "checksum": checksum,
        "index": {
            "wall_seconds": index_wall,
            "cpu_seconds": index_cpu,
        },
        "payload_read": {
            "wall_seconds": bag_read_wall,
            "cpu_seconds": bag_read_cpu,
        },
        "deserialize": {
            "wall_seconds": deserialize_wall,
            "cpu_seconds": deserialize_cpu,
        },
        "decode": {
            "wall_seconds": decode_wall,
            "cpu_seconds": decode_cpu,
        },
        "retrieval_decode_loop": {
            "wall_seconds": retrieval_wall,
            "cpu_seconds": retrieval_cpu,
        },
        "bag_only_wall_seconds": index_wall + bag_read_wall,
        "peak_rss_bytes_at_end": _peak_rss_bytes(),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Measure image bag indexing, raw-payload retrieval, ROS-message "
            "deserialization and pixel decode without target detection or "
            "calibration."))
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topics", nargs="+", required=True)
    parser.add_argument("--output")
    arguments = parser.parse_args(argv)

    started = time.time()
    result = {
        "schema_version": "1.0.0",
        "bag": arguments.bag,
        "started_unix_seconds": started,
        "topics": [
            benchmark_topic(arguments.bag, topic)
            for topic in arguments.topics
        ],
    }
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if arguments.output:
        with open(arguments.output, "w") as output:
            output.write(encoded)
    else:
        sys.stdout.write(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
