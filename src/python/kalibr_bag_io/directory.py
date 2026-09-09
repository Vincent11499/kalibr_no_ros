"""Reader for the ROS-independent Kalibr directory dataset format."""

from __future__ import annotations

import csv
import math
from pathlib import Path
import time
from typing import Dict, Iterator, List, Optional, Tuple

import cv2
import numpy as np
import yaml

from kalibr_no_ros.version import SCHEMA_VERSION

from .model import FileImageIndex, ImageRecord, ImuRecord, TopicInfo


MANIFEST_NAME = "dataset.yaml"
MANIFEST_TYPE = "kalibr_directory_dataset"
_CAMERA_FIELDS = {"id", "timestamps", "images"}
_IMU_FIELDS = {"id", "data"}
_CAMERA_METADATA = {"topic", "frame_id", "resolution", "clock_source"}
_IMU_METADATA = {"topic", "frame_id", "clock_source", "angular_velocity_unit", "linear_acceleration_unit"}
_IMU_REQUIRED_COLUMNS = (
    "timestamp_ns", "wx", "wy", "wz", "ax", "ay", "az",
)
_IMU_OPTIONAL_COLUMNS = ("temperature_c",)
_MAX_TIMESTAMP_NS = (1 << 63) - 1


class DirectoryDatasetError(ValueError):
    """The directory dataset manifest or one of its tables is invalid."""


def _load_mapping(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream)
    except OSError as error:
        raise DirectoryDatasetError("could not read {}: {}".format(path, error)) from error
    except yaml.YAMLError as error:
        raise DirectoryDatasetError("invalid YAML {}: {}".format(path, error)) from error
    if not isinstance(value, dict):
        raise DirectoryDatasetError("YAML root must be a mapping: {}".format(path))
    return value


def _resolve_child(root: Path, value, label: str, *, expected: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise DirectoryDatasetError("{} must be a non-empty relative path".format(label))
    relative = Path(value)
    if relative.is_absolute():
        raise DirectoryDatasetError("{} must be relative to the dataset root".format(label))
    resolved = (root / relative).resolve()
    if resolved != root and root not in resolved.parents:
        raise DirectoryDatasetError("{} escapes the dataset root: {}".format(label, value))
    if expected == "file" and not resolved.is_file():
        raise DirectoryDatasetError("{} is not a file: {}".format(label, resolved))
    if expected == "directory" and not resolved.is_dir():
        raise DirectoryDatasetError("{} is not a directory: {}".format(label, resolved))
    return resolved


def _strict_fields(value, allowed, label, optional=()):
    if not isinstance(value, dict):
        raise DirectoryDatasetError("{} must be a mapping".format(label))
    missing = sorted(allowed - set(value))
    unknown = sorted(set(value) - allowed - set(optional))
    if missing:
        raise DirectoryDatasetError(
            "{} is missing field(s): {}".format(label, ", ".join(missing)))
    if unknown:
        raise DirectoryDatasetError(
            "{} has unknown field(s): {}".format(label, ", ".join(unknown)))


def _topic(value, label):
    if not isinstance(value, str) or not value.strip():
        raise DirectoryDatasetError("{}.topic must be a non-empty string".format(label))
    return value


def _parse_timestamp(value, label):
    try:
        timestamp = int(value)
    except (TypeError, ValueError) as error:
        raise DirectoryDatasetError("{} must be an integer nanosecond timestamp".format(label)) from error
    if str(value).strip() != str(timestamp):
        raise DirectoryDatasetError("{} must be an integer nanosecond timestamp".format(label))
    if timestamp < 0 or timestamp > _MAX_TIMESTAMP_NS:
        raise DirectoryDatasetError("{} is outside [0, 2^63-1]".format(label))
    return timestamp


def _parse_finite(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise DirectoryDatasetError("{} must be a finite number".format(label)) from error
    if not math.isfinite(result):
        raise DirectoryDatasetError("{} must be a finite number".format(label))
    return result


def _read_csv(path: Path, required, optional=()):
    try:
        stream = path.open("r", encoding="utf-8-sig", newline="")
    except OSError as error:
        raise DirectoryDatasetError("could not read {}: {}".format(path, error)) from error
    with stream:
        reader = csv.DictReader(stream)
        actual = reader.fieldnames
        if actual is None:
            raise DirectoryDatasetError("CSV has no header: {}".format(path))
        if len(actual) != len(set(actual)):
            raise DirectoryDatasetError("CSV has duplicate column names: {}".format(path))
        required = list(required)
        optional = list(optional)
        if actual not in (required, required + optional):
            expected = ",".join(required)
            if optional:
                expected += "[,{}]".format(",".join(optional))
            raise DirectoryDatasetError(
                "invalid CSV header in {}: expected {}".format(path, expected))
        rows = []
        for line, row in enumerate(reader, start=2):
            if None in row:
                raise DirectoryDatasetError(
                    "too many columns in {} at line {}".format(path, line))
            if all(value is None or not value.strip() for value in row.values()):
                raise DirectoryDatasetError(
                    "empty row in {} at line {}".format(path, line))
            rows.append((line, row))
    return rows


def _decode_image(rawdata: bytes, source: str, grayscale: bool):
    encoded = np.frombuffer(rawdata, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise DirectoryDatasetError("OpenCV could not decode image: {}".format(source))
    if image.dtype == np.uint16:
        image = (image / 256.0).astype(np.uint8)
    elif image.dtype != np.uint8:
        raise DirectoryDatasetError(
            "unsupported decoded image depth {}: {}".format(image.dtype, source))
    if grayscale:
        if image.ndim == 2:
            return np.ascontiguousarray(image)
        if image.ndim != 3:
            raise DirectoryDatasetError(
                "unsupported decoded image shape {}: {}".format(image.shape, source))
        channels = image.shape[2]
        if channels == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif channels == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        elif channels == 1:
            image = image[:, :, 0]
        else:
            raise DirectoryDatasetError(
                "unsupported decoded image channel count {}: {}".format(channels, source))
    return np.ascontiguousarray(image)


def _check_resolution(image, expected, source):
    actual = [int(image.shape[1]), int(image.shape[0])]
    if expected is not None and actual != list(expected):
        raise DirectoryDatasetError("image resolution {} does not match manifest {}: {}".format(actual, expected, source))


class DeferredFileImagePayload:
    """Picklable image path read and decoded later by a detector worker."""

    def __init__(self, source, grayscale, resolution=None):
        self.source = source
        self.grayscale = grayscale
        self.resolution = resolution

    def decode_for_kalibr(self, collect_timing=False):
        if collect_timing:
            read_wall_start = time.perf_counter()
            read_cpu_start = time.process_time()
        try:
            rawdata = Path(self.source).read_bytes()
        except OSError as error:
            raise DirectoryDatasetError(
                "could not read image {}: {}".format(self.source, error)) from error
        if collect_timing:
            read_wall = time.perf_counter() - read_wall_start
            read_cpu = time.process_time() - read_cpu_start
            decode_wall_start = time.perf_counter()
            decode_cpu_start = time.process_time()
        image = _decode_image(rawdata, self.source, self.grayscale)
        _check_resolution(image, self.resolution, self.source)
        if not collect_timing:
            return image, {}
        return image, {
            "bag_read_wall_seconds": read_wall,
            "bag_read_cpu_seconds": read_cpu,
            "deserialize_wall_seconds": 0.0,
            "deserialize_cpu_seconds": 0.0,
            "decode_wall_seconds": time.perf_counter() - decode_wall_start,
            "decode_cpu_seconds": time.process_time() - decode_cpu_start,
        }


class IndexedDirectoryImageDataset:
    """Lazy random-access view over one camera stream in a directory dataset."""

    def __init__(self, root, stream, grayscale=True):
        self.root = Path(root)
        self.topic = stream["topic"]
        self.grayscale = grayscale
        table = stream["timestamps"]
        images = stream["images"]
        rows = _read_csv(table, ("timestamp_ns", "filename"))
        seen_paths = set()
        self.index = []
        previous_timestamp = None
        for line, row in rows:
            timestamp = _parse_timestamp(
                row["timestamp_ns"], "{} line {} timestamp_ns".format(table, line))
            if previous_timestamp is not None and timestamp <= previous_timestamp:
                raise DirectoryDatasetError("timestamps must be strictly increasing in {} at line {}".format(table, line))
            previous_timestamp = timestamp
            filename = row["filename"]
            image_path = _resolve_child(
                images, filename,
                "{} line {} filename".format(table, line), expected="file")
            normalized = str(image_path)
            if normalized in seen_paths:
                raise DirectoryDatasetError(
                    "duplicate image filename in {} at line {}: {}".format(
                        table, line, filename))
            seen_paths.add(normalized)
            self.index.append(
                FileImageIndex(
                    timestamp,
                    timestamp,
                    normalized,
                    frame_id=stream.get("frame_id", ""),
                    sequence=len(self.index),
                    compressed_format=image_path.suffix.lower().lstrip(".") or None,
                )
            )
        self.resolution = stream.get("resolution")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def close(self):
        return None

    def get(self, position: int):
        return self.get_by_entry(self.index[int(position)])

    def get_by_entry(self, selected):
        return self._get_by_entry(selected, collect_timing=False)

    def get_by_entry_with_timing(self, selected):
        return self._get_by_entry(selected, collect_timing=True)

    def get_deferred_by_entry(self, selected):
        return self._get_deferred_by_entry(selected, collect_timing=False)

    def get_deferred_by_entry_with_timing(self, selected):
        return self._get_deferred_by_entry(selected, collect_timing=True)

    @staticmethod
    def _read_bytes(selected, collect_timing):
        if collect_timing:
            wall_start = time.perf_counter()
            cpu_start = time.process_time()
        try:
            rawdata = Path(selected.path).read_bytes()
        except OSError as error:
            raise DirectoryDatasetError(
                "could not read image {}: {}".format(selected.path, error)) from error
        if not collect_timing:
            return rawdata, None
        return rawdata, {
            "bag_read_wall_seconds": time.perf_counter() - wall_start,
            "bag_read_cpu_seconds": time.process_time() - cpu_start,
        }

    def _get_deferred_by_entry(self, selected, collect_timing=False):
        # Keep the producer lightweight: only the path and decode policy cross
        # the process queue.  File I/O and OpenCV decoding happen together in
        # the detector worker, so multiple workers can overlap both phases.
        payload = DeferredFileImagePayload(selected.path, self.grayscale, self.resolution)
        timing = {
            "bag_read_wall_seconds": 0.0,
            "bag_read_cpu_seconds": 0.0,
        }
        return (payload, timing) if collect_timing else payload

    def _get_by_entry(self, selected, collect_timing=False):
        rawdata, timing = self._read_bytes(selected, collect_timing)
        if collect_timing:
            timing["deserialize_wall_seconds"] = 0.0
            timing["deserialize_cpu_seconds"] = 0.0
            wall_start = time.perf_counter()
            cpu_start = time.process_time()
        image = _decode_image(rawdata, selected.path, self.grayscale)
        _check_resolution(image, self.resolution, selected.path)
        if collect_timing:
            timing["decode_wall_seconds"] = time.perf_counter() - wall_start
            timing["decode_cpu_seconds"] = time.process_time() - cpu_start
        record = ImageRecord(
            selected.header_timestamp_ns,
            selected.record_timestamp_ns,
            selected.encoding,
            image,
            selected.frame_id,
            selected.sequence,
            selected.compressed_format,
        )
        return (record, timing) if collect_timing else record


class DirectoryReader:
    """Read image and IMU streams described by a root ``dataset.yaml``."""

    storage_format = "directory"

    def __init__(self, dataset):
        self.path = Path(dataset).expanduser().resolve()
        if not self.path.is_dir():
            if not self.path.exists():
                raise FileNotFoundError(str(self.path))
            raise DirectoryDatasetError("directory dataset path is not a directory: {}".format(self.path))
        manifest_path = self.path / MANIFEST_NAME
        if not manifest_path.is_file():
            raise DirectoryDatasetError("directory dataset is missing {}".format(manifest_path))
        manifest = _load_mapping(manifest_path)
        allowed = {"schema_version", "type", "dataset_id", "created_at", "producer", "cameras", "imus"}
        unknown = sorted(set(manifest) - allowed)
        if unknown:
            raise DirectoryDatasetError(
                "dataset manifest has unknown field(s): {}".format(", ".join(unknown)))
        if manifest.get("schema_version") != SCHEMA_VERSION:
            raise DirectoryDatasetError(
                "dataset schema_version must be {}".format(SCHEMA_VERSION))
        if manifest.get("type") != MANIFEST_TYPE:
            raise DirectoryDatasetError("dataset type must be {}".format(MANIFEST_TYPE))
        if not isinstance(manifest.get("dataset_id"), str) or not manifest["dataset_id"].strip():
            raise DirectoryDatasetError("dataset.dataset_id must be a non-empty string")
        self.manifest = manifest
        self.dataset_id = manifest["dataset_id"]
        cameras = manifest.get("cameras", [])
        imus = manifest.get("imus", [])
        if not isinstance(cameras, list) or not isinstance(imus, list):
            raise DirectoryDatasetError("dataset cameras and imus must be lists")
        if not cameras and not imus:
            raise DirectoryDatasetError("dataset must contain at least one camera or IMU stream")
        self._cameras: Dict[str, dict] = {}
        self._imus: Dict[str, dict] = {}
        topics = set()
        sensor_ids = set()
        for index, camera in enumerate(cameras):
            label = "cameras[{}]".format(index)
            _strict_fields(camera, _CAMERA_FIELDS, label, _CAMERA_METADATA)
            self._validate_sensor(camera, label, sensor_ids)
            topic = _topic(camera.get("topic", camera["id"]), label)
            if topic in topics:
                raise DirectoryDatasetError("duplicate topic: {}".format(topic))
            topics.add(topic)
            self._cameras[topic] = {
                **camera,
                "topic": topic,
                "timestamps": _resolve_child(
                    self.path, camera["timestamps"], label + ".timestamps", expected="file"),
                "images": _resolve_child(
                    self.path, camera["images"], label + ".images", expected="directory"),
            }
        for index, imu in enumerate(imus):
            label = "imus[{}]".format(index)
            _strict_fields(imu, _IMU_FIELDS, label, _IMU_METADATA)
            self._validate_sensor(imu, label, sensor_ids)
            for field, expected in (("angular_velocity_unit", "rad/s"), ("linear_acceleration_unit", "m/s^2")):
                if field in imu and imu[field] != expected:
                    raise DirectoryDatasetError("{}.{} must be {}".format(label, field, expected))
            topic = _topic(imu.get("topic", imu["id"]), label)
            if topic in topics:
                raise DirectoryDatasetError("duplicate topic: {}".format(topic))
            topics.add(topic)
            self._imus[topic] = {
                **imu,
                "topic": topic,
                "data": _resolve_child(
                    self.path, imu["data"], label + ".data", expected="file"),
            }

    def sensor_topic(self, sensor_id, kind, topic=None):
        """Resolve an explicit sensor ID to the native reader's stream key."""
        streams = {"camera": self._cameras, "imu": self._imus}[kind]
        for stream in streams.values():
            if stream["id"] == sensor_id:
                if topic is not None and topic != stream["topic"]:
                    raise DirectoryDatasetError(
                        "{} {} topic disagrees with dataset manifest".format(kind, sensor_id))
                return stream["topic"]
        raise DirectoryDatasetError("unknown {} id in dataset: {}".format(kind, sensor_id))

    @staticmethod
    def _validate_sensor(sensor, label, seen):
        sensor_id = sensor["id"]
        if not isinstance(sensor_id, str) or not sensor_id.strip() or sensor_id in seen:
            raise DirectoryDatasetError("{}.id must be a unique non-empty string".format(label))
        seen.add(sensor_id)
        for field in ("frame_id", "clock_source"):
            if field in sensor and not isinstance(sensor[field], str):
                raise DirectoryDatasetError("{}.{} must be a string".format(label, field))
        if "resolution" in sensor:
            value = sensor["resolution"]
            if not isinstance(value, list) or len(value) != 2 or any(type(v) is not int or v < 1 for v in value):
                raise DirectoryDatasetError("{}.resolution must be [positive width, positive height]".format(label))

    def topics(self) -> List[TopicInfo]:
        result = []
        for topic, stream in self._cameras.items():
            count = len(_read_csv(
                stream["timestamps"], ("timestamp_ns", "filename")))
            result.append(TopicInfo(topic, "sensor_msgs/msg/Image", count))
        for topic, stream in self._imus.items():
            count = len(_read_csv(
                stream["data"], _IMU_REQUIRED_COLUMNS, _IMU_OPTIONAL_COLUMNS))
            result.append(TopicInfo(topic, "sensor_msgs/msg/Imu", count))
        return sorted(result, key=lambda info: (info.name, info.msgtype))

    @staticmethod
    def _crop(records, from_to: Optional[Tuple[float, float]]):
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

    def index_images(self, topic: str, *, grayscale: bool = True):
        stream = self._cameras.get(topic)
        if stream is None:
            if topic in self._imus:
                raise RuntimeError("Topic {} is not an image stream".format(topic))
            raise RuntimeError("Could not find topic {} in {}.".format(topic, self.path))
        return IndexedDirectoryImageDataset(self.path, stream, grayscale)

    def read_images(self, topic, from_to=None, frequency=None, *, grayscale=True):
        with self.index_images(topic, grayscale=grayscale) as dataset:
            selected = self._decimate(self._crop(dataset.index, from_to), frequency)
            return [dataset.get_by_entry(entry) for entry in selected]

    def iter_images(self, *args, **kwargs) -> Iterator[ImageRecord]:
        return iter(self.read_images(*args, **kwargs))

    def read_imu(self, topic, from_to=None) -> List[ImuRecord]:
        stream = self._imus.get(topic)
        if stream is None:
            if topic in self._cameras:
                raise RuntimeError("Topic {} is not an IMU stream".format(topic))
            raise RuntimeError("Could not find topic {} in {}.".format(topic, self.path))
        table = stream["data"]
        records = []
        previous_timestamp = None
        for line, row in _read_csv(table, _IMU_REQUIRED_COLUMNS, _IMU_OPTIONAL_COLUMNS):
            timestamp = _parse_timestamp(
                row["timestamp_ns"], "{} line {} timestamp_ns".format(table, line))
            if previous_timestamp is not None and timestamp <= previous_timestamp:
                raise DirectoryDatasetError("timestamps must be strictly increasing in {} at line {}".format(table, line))
            previous_timestamp = timestamp
            values = [
                _parse_finite(row[name], "{} line {} {}".format(table, line, name))
                for name in ("wx", "wy", "wz", "ax", "ay", "az")
            ]
            temperature = None
            if "temperature_c" in row and row["temperature_c"].strip():
                temperature = _parse_finite(
                    row["temperature_c"],
                    "{} line {} temperature_c".format(table, line),
                )
            records.append(
                ImuRecord(
                    timestamp,
                    timestamp,
                    np.asarray(values[:3], dtype=float),
                    np.asarray(values[3:], dtype=float),
                    frame_id=stream.get("frame_id", ""),
                    temperature_c=temperature,
                )
            )
        return self._crop(records, from_to)

    def iter_imu(self, *args, **kwargs) -> Iterator[ImuRecord]:
        return iter(self.read_imu(*args, **kwargs))
