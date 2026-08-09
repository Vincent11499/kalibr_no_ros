"""Read standard camera and IMU messages from ROS1 bags."""

from pathlib import Path
from io import BytesIO
import os
from typing import Iterator, List, Optional, Sequence, Tuple

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.rosbag1.reader import Header, RecordType, read_bytes, read_uint32
from rosbags.typesys import Stores, get_typestore

from .image_codec import decode_compressed_image, decode_raw_image
from .model import ImageIndex, ImageRecord, ImuRecord, TopicInfo


IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
IMU_TYPE = "sensor_msgs/msg/Imu"


def _header_timestamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class BagReader:
    """Stateless ROS1 bag reader whose returned records are sorted by header time."""

    def __init__(self, bagfile):
        self.path = Path(bagfile).expanduser().resolve()
        if not self.path.is_file():
            raise FileNotFoundError(str(self.path))
        self._typestore = get_typestore(Stores.ROS1_NOETIC)

    def topics(self) -> List[TopicInfo]:
        with Reader(self.path) as reader:
            return sorted(
                (TopicInfo(c.topic, c.msgtype, int(c.msgcount)) for c in reader.connections),
                key=lambda info: (info.name, info.msgtype),
            )

    def _connections(self, reader: Reader, topic: str, allowed_types: Sequence[str]):
        found = [c for c in reader.connections if c.topic == topic]
        if not found:
            raise RuntimeError("Could not find topic {} in {}.".format(topic, self.path))
        unsupported = [c.msgtype for c in found if c.msgtype not in allowed_types]
        if unsupported:
            raise RuntimeError(
                "Topic {} has unsupported type(s): {}".format(topic, ", ".join(unsupported))
            )
        return found

    @staticmethod
    def _crop(records, from_to: Optional[Tuple[float, float]]):
        if from_to is None:
            return records
        start_offset, end_offset = map(float, from_to)
        if start_offset >= end_offset:
            raise RuntimeError("Bag start time must be smaller than end time.")
        if not records:
            return records
        bag_start = records[0].header_timestamp_ns
        lower = bag_start + int(round(start_offset * 1e9))
        upper = bag_start + int(round(end_offset * 1e9))
        return [r for r in records if lower <= r.header_timestamp_ns <= upper]

    @staticmethod
    def _decimate(records, frequency: Optional[float]):
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

    def read_images(
        self,
        topic: str,
        from_to: Optional[Tuple[float, float]] = None,
        frequency: Optional[float] = None,
        *,
        grayscale: bool = True,
    ) -> List[ImageRecord]:
        with self.index_images(topic, grayscale=grayscale) as dataset:
            selected = self._decimate(self._crop(dataset.index, from_to), frequency)
            return [dataset.get_by_entry(entry) for entry in selected]

    def iter_images(self, *args, **kwargs) -> Iterator[ImageRecord]:
        return iter(self.read_images(*args, **kwargs))

    def index_images(self, topic: str, *, grayscale: bool = True):
        """Open a lazy, random-access image dataset.

        Only headers are decoded while building the index. Pixel payloads are
        decoded on demand, matching rosbag's memory behavior during Kalibr runs.
        """
        return IndexedImageDataset(self.path, topic, self._typestore, grayscale)

    def read_imu(
        self, topic: str, from_to: Optional[Tuple[float, float]] = None
    ) -> List[ImuRecord]:
        records = []
        with Reader(self.path) as reader:
            connections = self._connections(reader, topic, (IMU_TYPE,))
            for connection, record_ns, rawdata in reader.messages(connections=connections):
                message = self._typestore.deserialize_ros1(rawdata, connection.msgtype)
                vec = message.angular_velocity
                acc = message.linear_acceleration
                quat = message.orientation
                records.append(
                    ImuRecord(
                        _header_timestamp_ns(message),
                        int(record_ns),
                        np.array([vec.x, vec.y, vec.z], dtype=float),
                        np.array([acc.x, acc.y, acc.z], dtype=float),
                        message.header.frame_id,
                        int(message.header.seq),
                        np.array([quat.x, quat.y, quat.z, quat.w], dtype=float),
                        np.asarray(message.orientation_covariance, dtype=float).copy(),
                        np.asarray(message.angular_velocity_covariance, dtype=float).copy(),
                        np.asarray(message.linear_acceleration_covariance, dtype=float).copy(),
                    )
                )
        records.sort(key=lambda item: item.header_timestamp_ns)
        return self._crop(records, from_to)

    def iter_imu(self, *args, **kwargs) -> Iterator[ImuRecord]:
        return iter(self.read_imu(*args, **kwargs))


class IndexedImageDataset:
    """Persistent random-access view over one image topic."""

    def __init__(self, path, topic, typestore, grayscale=True):
        self.path = Path(path)
        self.topic = topic
        self.typestore = typestore
        self.grayscale = grayscale
        self.reader = Reader(self.path)
        self.reader.open()
        self.connections = [c for c in self.reader.connections if c.topic == topic]
        if not self.connections:
            self.close()
            raise RuntimeError("Could not find topic {} in {}.".format(topic, path))
        unsupported = [c.msgtype for c in self.connections if c.msgtype not in IMAGE_TYPES]
        if unsupported:
            self.close()
            raise RuntimeError("Topic {} has unsupported type(s): {}".format(topic, unsupported))
        self.connection_by_id = {c.id: c for c in self.connections}
        entries = []
        for connection in self.connections:
            entries.extend((connection.id, item) for item in self.reader.indexes[connection.id])
        entries.sort(key=lambda item: item[1].time)
        self._entries = entries
        self._entry_by_location = {
            (connection_id, int(entry.chunk_pos), int(entry.offset)): entry
            for connection_id, entry in entries
        }
        self.index = []
        for connection_id, entry in entries:
            message = self._deserialize(connection_id, entry)
            msgtype = self.connection_by_id[connection_id].msgtype
            self.index.append(
                ImageIndex(
                    _header_timestamp_ns(message),
                    int(entry.time),
                    int(connection_id),
                    int(entry.chunk_pos),
                    int(entry.offset),
                    message.encoding if msgtype == "sensor_msgs/msg/Image" else "compressed",
                    message.header.frame_id,
                    int(message.header.seq),
                    None if msgtype == "sensor_msgs/msg/Image" else message.format,
                )
            )
        self.index.sort(key=lambda item: item.header_timestamp_ns)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __del__(self):
        self.close()

    def close(self):
        reader = getattr(self, "reader", None)
        if reader is not None:
            reader.close()
            self.reader = None

    def _raw(self, entry):
        reader = self.reader
        if reader is None:
            raise RuntimeError("IndexedImageDataset is closed")
        if reader.current_chunk[0] != entry.chunk_pos:
            reader.current_chunk[1].close()
            chunk_header = reader.chunks[entry.chunk_pos]
            reader.bio.seek(chunk_header.datapos)
            rawbytes = chunk_header.decompressor(read_bytes(reader.bio, chunk_header.datasize))
            reader.current_chunk = (entry.chunk_pos, BytesIO(rawbytes))
        chunk = reader.current_chunk[1]
        chunk.seek(entry.offset)
        while True:
            header = Header.read(chunk)
            operation = header.get_uint8("op")
            if operation != RecordType.CONNECTION:
                break
            chunk.seek(read_uint32(chunk), os.SEEK_CUR)
        if operation != RecordType.MSGDATA:
            raise RuntimeError("Expected image message data at indexed bag position")
        return read_bytes(chunk, read_uint32(chunk))

    def _deserialize(self, connection_id, entry):
        connection = self.connection_by_id[connection_id]
        return self.typestore.deserialize_ros1(self._raw(entry), connection.msgtype)

    def get(self, position: int) -> ImageRecord:
        return self.get_by_entry(self.index[int(position)])

    def get_by_entry(self, selected: ImageIndex) -> ImageRecord:
        connection_id = selected.connection_id
        entry = self._entry_by_location[
            (connection_id, selected.chunk_position, selected.chunk_offset)
        ]
        message = self._deserialize(connection_id, entry)
        msgtype = self.connection_by_id[connection_id].msgtype
        if msgtype == "sensor_msgs/msg/Image":
            image = decode_raw_image(message, grayscale=self.grayscale)
        else:
            image = decode_compressed_image(message, grayscale=self.grayscale)
        return ImageRecord(
            selected.header_timestamp_ns,
            selected.record_timestamp_ns,
            selected.encoding,
            image,
            selected.frame_id,
            selected.sequence,
            selected.compressed_format,
        )
