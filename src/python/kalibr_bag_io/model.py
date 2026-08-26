"""Neutral records passed across the bag/core boundary."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


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
