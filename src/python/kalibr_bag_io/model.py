"""Neutral records passed across the bag/core boundary."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


def crop_records(records, from_to):
    if from_to is None:
        return records
    start_offset, end_offset = map(float, from_to)
    if start_offset >= end_offset:
        raise RuntimeError("Bag start time must be smaller than end time.")
    if not records:
        return records
    start = records[0].header_timestamp_ns
    lower = start + int(round(start_offset * 1e9))
    upper = start + int(round(end_offset * 1e9))
    return [item for item in records if lower <= item.header_timestamp_ns <= upper]


def decimate_records(records, frequency):
    if frequency is None:
        return records
    frequency = float(frequency)
    if frequency <= 0.0:
        raise RuntimeError("Frequency must be greater than zero")
    minimum_delta = 1e9 / frequency
    selected = []
    last = None
    for record in records:
        if last is None or record.header_timestamp_ns - last >= minimum_delta:
            selected.append(record)
            last = record.header_timestamp_ns
    return selected


@dataclass(frozen=True)
class TopicInfo:
    name: str
    msgtype: str
    message_count: int


@dataclass(frozen=True)
class ImageIndex:
    """Small random-access descriptor; image payload remains in the bag."""

    header_timestamp_ns: int
    record_timestamp_ns: int
    connection_id: int
    chunk_position: int
    chunk_offset: int
    encoding: str
    frame_id: str = ""
    sequence: int = 0
    compressed_format: Optional[str] = None
    # Position among messages that share the same connection and record time.
    # ROS1 uses chunk_position/chunk_offset for direct lookup; ROS2 SQLite3 and
    # MCAP use record_timestamp_ns plus this ordinal for backend-neutral lookup.
    record_ordinal: int = 0


@dataclass(frozen=True)
class FileImageIndex:
    """Random-access descriptor for an image stored as a regular file."""

    header_timestamp_ns: int
    record_timestamp_ns: int
    path: str
    encoding: str = "file"
    frame_id: str = ""
    sequence: int = 0
    compressed_format: Optional[str] = None


@dataclass(frozen=True)
class ImageRecord:
    header_timestamp_ns: int
    record_timestamp_ns: int
    encoding: str
    image: np.ndarray
    frame_id: str = ""
    sequence: int = 0
    compressed_format: Optional[str] = None


@dataclass(frozen=True)
class ImuRecord:
    header_timestamp_ns: int
    record_timestamp_ns: int
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray
    frame_id: str = ""
    sequence: int = 0
    orientation: Optional[np.ndarray] = None
    orientation_covariance: Optional[np.ndarray] = None
    angular_velocity_covariance: Optional[np.ndarray] = None
    linear_acceleration_covariance: Optional[np.ndarray] = None
    temperature_c: Optional[float] = None

    def vectors(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.angular_velocity, self.linear_acceleration
