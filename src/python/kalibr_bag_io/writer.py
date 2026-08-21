"""Write the standard ROS1 or ROS2 messages consumed by the readers."""

from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
from rosbags.rosbag1 import Writer as Ros1Writer
from rosbags.rosbag2 import Writer as Ros2Writer
from rosbags.typesys import Stores, get_typestore

from .image_codec import encode_raw_image
from .model import ImageRecord, ImuRecord


class BagWriter:
    def __init__(self, bagfile, *, storage_format=None):
        self.path = Path(bagfile).expanduser().resolve()
        if storage_format is None:
            storage_format = "ros1" if self.path.suffix == ".bag" else "ros2"
        if storage_format not in ("ros1", "ros2"):
            raise ValueError("storage_format must be 'ros1' or 'ros2'")
        self.storage_format = storage_format
        store = Stores.ROS1_NOETIC if storage_format == "ros1" else Stores.ROS2_HUMBLE
        self._typestore = get_typestore(store)
        self._writer = None
        self._connections: Dict[tuple, object] = {}

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        writer_type = Ros1Writer if self.storage_format == "ros1" else Ros2Writer
        self._writer = writer_type(self.path)
        self._writer.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self._writer is not None:
            self._writer.close()
            self._writer = None

    def _require_open(self):
        if self._writer is None:
            raise RuntimeError("BagWriter must be used as a context manager")

    def _connection(self, topic, msgtype):
        self._require_open()
        key = (topic, msgtype)
        if key not in self._connections:
            self._connections[key] = self._writer.add_connection(
                topic, msgtype, typestore=self._typestore
            )
        return self._connections[key]

    def _header(self, timestamp_ns: int, sequence: int, frame_id: str):
        stamp_type = self._typestore.types["builtin_interfaces/msg/Time"]
        header_type = self._typestore.types["std_msgs/msg/Header"]
        seconds, nanoseconds = divmod(int(timestamp_ns), 1_000_000_000)
        stamp = stamp_type(seconds, nanoseconds)
        if self.storage_format == "ros1":
            return header_type(int(sequence), stamp, frame_id)
        return header_type(stamp, frame_id)

    def _write(self, topic, msgtype, message, record_timestamp_ns):
        connection = self._connection(topic, msgtype)
        if self.storage_format == "ros1":
            raw = self._typestore.serialize_ros1(message, msgtype)
        else:
            raw = self._typestore.serialize_cdr(message, msgtype)
        self._writer.write(connection, int(record_timestamp_ns), raw)

    def write_image(self, topic: str, record: ImageRecord, *, encoding: Optional[str] = None):
        msgtype = "sensor_msgs/msg/Image"
        image_type = self._typestore.types[msgtype]
        selected_encoding = encoding or record.encoding
        raw, step, is_bigendian = encode_raw_image(record.image, selected_encoding)
        height, width = record.image.shape[:2]
        message = image_type(
            self._header(record.header_timestamp_ns, record.sequence, record.frame_id),
            int(height),
            int(width),
            selected_encoding,
            is_bigendian,
            step,
            raw,
        )
        self._write(topic, msgtype, message, record.record_timestamp_ns)

    def write_compressed_image(
        self, topic: str, record: ImageRecord, *, image_format: str = ".png"
    ):
        msgtype = "sensor_msgs/msg/CompressedImage"
        compressed_type = self._typestore.types[msgtype]
        ok, payload = cv2.imencode(image_format, np.asarray(record.image))
        if not ok:
            raise RuntimeError("OpenCV failed to encode image as {}".format(image_format))
        fmt = record.compressed_format or image_format.lstrip(".")
        message = compressed_type(
            self._header(record.header_timestamp_ns, record.sequence, record.frame_id),
            fmt,
            np.ascontiguousarray(payload, dtype=np.uint8).reshape(-1),
        )
        self._write(topic, msgtype, message, record.record_timestamp_ns)

    def write_imu(self, topic: str, record: ImuRecord):
        msgtype = "sensor_msgs/msg/Imu"
        imu_type = self._typestore.types[msgtype]
        quat_type = self._typestore.types["geometry_msgs/msg/Quaternion"]
        vector_type = self._typestore.types["geometry_msgs/msg/Vector3"]
        orientation = record.orientation if record.orientation is not None else [0, 0, 0, 1]
        oc = record.orientation_covariance if record.orientation_covariance is not None else np.zeros(9)
        wc = record.angular_velocity_covariance if record.angular_velocity_covariance is not None else np.zeros(9)
        ac = record.linear_acceleration_covariance if record.linear_acceleration_covariance is not None else np.zeros(9)
        omega = record.angular_velocity
        alpha = record.linear_acceleration
        message = imu_type(
            self._header(record.header_timestamp_ns, record.sequence, record.frame_id),
            quat_type(*map(float, orientation)),
            np.asarray(oc, dtype=np.float64),
            vector_type(*map(float, omega)),
            np.asarray(wc, dtype=np.float64),
            vector_type(*map(float, alpha)),
            np.asarray(ac, dtype=np.float64),
        )
        self._write(topic, msgtype, message, record.record_timestamp_ns)
