import pickle
import tempfile
import unittest
import unittest.mock
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_bag_io import (
    DirectoryDatasetError,
    DirectoryReader,
    detect_dataset_format,
    open_dataset,
)
from kalibr_no_ros.datasets import BagImageDatasetReader, BagImuDatasetReader
from kalibr_no_ros.validation import validate_task


class FakeTime:
    def __init__(self, seconds, nanoseconds=None):
        self.seconds = seconds
        self.nanoseconds = nanoseconds


class DirectoryIoTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / "dataset"
        self.images = self.root / "cameras" / "cam0" / "images"
        self.images.mkdir(parents=True)
        (self.root / "imu").mkdir()

    def tearDown(self):
        self.temporary.cleanup()

    def _write_manifest(self, cameras=None, imus=None):
        value = {
            "schema_version": "1.0.0",
            "type": "kalibr_directory_dataset",
            "dataset_id": "test-dataset",
        }
        if cameras is not None:
            value["cameras"] = cameras
        if imus is not None:
            value["imus"] = imus
        (self.root / "dataset.yaml").write_text(
            yaml.safe_dump(value, sort_keys=False), encoding="utf-8")

    def _camera_stream(self):
        return {
            "id": "cam0",
            "topic": "/cam0/image_raw",
            "timestamps": "cameras/cam0/timestamps.csv",
            "images": "cameras/cam0/images",
        }

    def _imu_stream(self):
        return {"id": "imu0", "topic": "/imu0", "data": "imu/imu0.csv"}

    def _write_images(self):
        color = np.zeros((6, 8, 3), dtype=np.uint8)
        color[:, :, 0] = 15
        color[:, :, 1] = 90
        color[:, :, 2] = 220
        for name in ("first.png", "second.jpg", "third.bmp"):
            self.assertTrue(cv2.imwrite(str(self.images / name), color))
        table = self.root / "cameras" / "cam0" / "timestamps.csv"
        table.write_text(
            "timestamp_ns,filename\n"
            "1000000000,first.png\n"
            "2000000000,second.jpg\n"
            "3000000000,third.bmp\n",
            encoding="utf-8",
        )
        return color

    def _write_imu(self):
        (self.root / "imu" / "imu0.csv").write_text(
            "timestamp_ns,wx,wy,wz,ax,ay,az,temperature_c\n"
            "1000000000,1,2,3,4,5,6,\n"
            "3000000000,3,4,5,6,7,8,41.5\n",
            encoding="utf-8",
        )

    def test_calibration_validation_does_not_depend_on_capture_meta(self):
        self._write_images()
        self._write_imu()
        camera, imu = self._camera_stream(), self._imu_stream()
        del camera["topic"], imu["topic"]
        self._write_manifest([camera], [imu])
        common = {
            "schema_version": "1.0.0", "kind": "calibration_task",
            "dataset": {"type": "directory", "path": str(self.root)},
            "target": {"type": "aprilgrid", "parameters": {
                "tagRows": 3, "tagCols": 3, "tagSize": 0.04, "tagSpacing": 0.3,
            }},
        }
        tasks = {
            "camera": {**common, "job": "camera_calibration",
                       "cameras": [{"id": "cam0", "model": "pinhole-equi"}]},
            "camera_imu": {**common, "job": "camera_imu_calibration",
                           "camera_calibration": {"path": "calibration.yaml"},
                           "imus": [{"id": "imu0", "path": "imu.yaml", "model": "calibrated"}]},
        }
        documents = {
            "calibration.yaml": {
                "schema_version": "1.0.0", "kind": "calibration_result",
                "calibration_type": "cameras", "cameras": [{
                    "id": "cam0", "rostopic": "cam0", "camera_model": "pinhole",
                    "distortion_model": "equidistant", "intrinsics": [7., 7., 4., 3.],
                    "distortion_coeffs": [0., 0., 0., 0.], "resolution": [8, 6],
                }],
            },
            "imu.yaml": {
                "schema_version": "1.0.0", "kind": "imu_configuration",
                "update_rate": 1., "accelerometer_noise_density": 0.01,
                "accelerometer_random_walk": 0.001, "gyroscope_noise_density": 0.001,
                "gyroscope_random_walk": 0.0001,
            },
            **{name + ".yaml": task for name, task in tasks.items()},
        }
        for name, document in documents.items():
            (self.root.parent / name).write_text(yaml.safe_dump(document), encoding="utf-8")

        meta = self.root / "meta"
        self.assertFalse(meta.exists())
        expected = {}
        for name in tasks:
            expected[name] = validate_task(self.root.parent / (name + ".yaml"))
            self.assertEqual(expected[name]["status"], "passed", expected[name])
            self.assertEqual(expected[name]["cameras"][0]["images"], 3)
        self.assertEqual(expected["camera_imu"]["imus"][0]["samples"], 2)

        meta.mkdir()
        for name in ("capture_dataset.yaml", "session.json", "pairs.csv", "imu_full.jsonl",
                     "validation.json"):
            (meta / name).write_bytes(b"\xff\xfe invalid capture metadata\x00")
        for name in tasks:
            with self.subTest(job=name, capture_meta="corrupt"):
                actual = validate_task(self.root.parent / (name + ".yaml"))
                self.assertEqual(actual, expected[name])

    def test_id_only_manifest_reads_identical_images_timestamps_and_imu_values(self):
        self._write_images()
        self._write_imu()
        camera, imu = self._camera_stream(), self._imu_stream()
        self._write_manifest([camera], [imu])
        before = DirectoryReader(self.root)
        images = before.read_images(camera['topic'])
        measurements = before.read_imu(imu['topic'])
        del camera['topic'], imu['topic']
        self._write_manifest([camera], [imu])
        after = DirectoryReader(self.root)
        self.assertEqual(after.sensor_topic('cam0', 'camera'), 'cam0')
        self.assertEqual(after.sensor_topic('imu0', 'imu'), 'imu0')
        for old, new in zip(images, after.read_images('cam0')):
            self.assertEqual(old.header_timestamp_ns, new.header_timestamp_ns)
            np.testing.assert_array_equal(old.image, new.image)
        for old, new in zip(measurements, after.read_imu('imu0')):
            self.assertEqual(old.header_timestamp_ns, new.header_timestamp_ns)
            np.testing.assert_array_equal(old.angular_velocity, new.angular_velocity)
            np.testing.assert_array_equal(old.linear_acceleration, new.linear_acceleration)
        with self.assertRaisesRegex(DirectoryDatasetError, 'unknown imu id'):
            after.sensor_topic('cam0', 'imu')

    def test_png_jpeg_bmp_manifest_imu_and_selection(self):
        original = self._write_images()
        self._write_imu()
        self._write_manifest([self._camera_stream()], [self._imu_stream()])

        reader = open_dataset(self.root)
        self.assertIsInstance(reader, DirectoryReader)
        self.assertEqual(reader.storage_format, "directory")
        self.assertEqual(detect_dataset_format(self.root), "directory")
        self.assertEqual(
            [(item.name, item.msgtype, item.message_count) for item in reader.topics()],
            [
                ("/cam0/image_raw", "sensor_msgs/msg/Image", 3),
                ("/imu0", "sensor_msgs/msg/Imu", 2),
            ],
        )

        images = reader.read_images("/cam0/image_raw", grayscale=False)
        self.assertEqual(
            [item.header_timestamp_ns for item in images],
            [1_000_000_000, 2_000_000_000, 3_000_000_000],
        )
        np.testing.assert_array_equal(images[0].image, original)
        selected = reader.read_images(
            "/cam0/image_raw", from_to=(0.5, 2.0), frequency=1.0)
        self.assertEqual(
            [item.header_timestamp_ns for item in selected],
            [2_000_000_000, 3_000_000_000],
        )

        imu = reader.read_imu("/imu0")
        self.assertEqual([item.header_timestamp_ns for item in imu], [1_000_000_000, 3_000_000_000])
        self.assertIsNone(imu[0].temperature_c)
        self.assertEqual(imu[1].temperature_c, 41.5)
        np.testing.assert_array_equal(imu[1].angular_velocity, [3.0, 4.0, 5.0])
        np.testing.assert_array_equal(imu[1].linear_acceleration, [6.0, 7.0, 8.0])

    def test_lazy_deferred_payload_is_picklable_and_timed(self):
        self._write_images()
        self._write_manifest([self._camera_stream()], [])
        reader = DirectoryReader(self.root)
        with reader.index_images("/cam0/image_raw") as dataset:
            deferred, io_timing = dataset.get_deferred_by_entry_with_timing(dataset.index[0])
            restored = pickle.loads(pickle.dumps(deferred))
            self.assertFalse(hasattr(restored, "rawdata"))
            image, worker_timing = restored.decode_for_kalibr(True)
            direct, direct_timing = dataset.get_by_entry_with_timing(dataset.index[0])
        np.testing.assert_array_equal(image, direct.image)
        self.assertEqual(set(io_timing), {
            "bag_read_wall_seconds", "bag_read_cpu_seconds",
        })
        self.assertEqual(io_timing["bag_read_wall_seconds"], 0.0)
        self.assertEqual(io_timing["bag_read_cpu_seconds"], 0.0)
        self.assertEqual(set(worker_timing), {
            "bag_read_wall_seconds", "bag_read_cpu_seconds",
            "deserialize_wall_seconds", "deserialize_cpu_seconds",
            "decode_wall_seconds", "decode_cpu_seconds",
        })
        self.assertEqual(worker_timing["deserialize_wall_seconds"], 0.0)
        self.assertEqual(set(direct_timing), {
            "bag_read_wall_seconds", "bag_read_cpu_seconds",
            "deserialize_wall_seconds", "deserialize_cpu_seconds",
            "decode_wall_seconds", "decode_cpu_seconds",
        })

    def test_deferred_payload_opens_image_only_when_worker_decodes(self):
        self._write_images()
        self._write_manifest([self._camera_stream()], [])
        dataset = DirectoryReader(self.root).index_images("/cam0/image_raw")
        selected = dataset.index[0]
        original_read_bytes = Path.read_bytes
        with unittest.mock.patch.object(
                Path, "read_bytes", autospec=True,
                side_effect=original_read_bytes) as read_bytes:
            deferred = dataset.get_deferred_by_entry(selected)
            read_bytes.assert_not_called()
            deferred.decode_for_kalibr()
            read_bytes.assert_called_once_with(Path(selected.path))

    def test_legacy_dataset_adapters_accept_directory_dataset(self):
        self._write_images()
        self._write_imu()
        self._write_manifest([self._camera_stream()], [self._imu_stream()])
        cameras = BagImageDatasetReader(
            self.root, "/cam0/image_raw", time_factory=FakeTime)
        imus = BagImuDatasetReader(self.root, "/imu0", time_factory=FakeTime)
        try:
            self.assertEqual(cameras.numImages(), 3)
            self.assertEqual(imus.numMessages(), 2)
            self.assertEqual(list(cameras)[0][0].seconds, 1)
            self.assertEqual(list(imus)[0][0].seconds, 1)
        finally:
            cameras.close()

    def test_explicit_format_mismatch_and_ambiguous_markers_are_rejected(self):
        self._write_images()
        self._write_manifest([self._camera_stream()], [])
        with self.assertRaisesRegex(RuntimeError, "format mismatch"):
            open_dataset(self.root, format="ros2")
        (self.root / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
        with self.assertRaisesRegex(RuntimeError, "ambiguous"):
            detect_dataset_format(self.root)

    def test_factory_detects_ros1_file_and_ros2_marker(self):
        ros1 = Path(self.temporary.name) / "input.bag"
        ros1.write_bytes(b"")
        ros2 = Path(self.temporary.name) / "ros2"
        ros2.mkdir()
        (ros2 / "metadata.yaml").write_text("version: 5\n", encoding="utf-8")
        self.assertEqual(detect_dataset_format(ros1), "ros1")
        self.assertEqual(detect_dataset_format(ros2), "ros2")
        self.assertEqual(open_dataset(ros1).storage_format, "ros1")
        self.assertEqual(open_dataset(ros2).storage_format, "ros2")

    def test_manifest_path_escape_duplicate_topic_and_bad_csv_are_rejected(self):
        self._write_images()
        escaped = self._camera_stream()
        escaped["timestamps"] = "../outside.csv"
        self._write_manifest([escaped], [])
        with self.assertRaisesRegex(DirectoryDatasetError, "escapes"):
            DirectoryReader(self.root)

        self._write_manifest([self._camera_stream()], [{
            "id": "imu0",
            "topic": "/cam0/image_raw", "data": "imu/imu0.csv",
        }])
        self._write_imu()
        with self.assertRaisesRegex(DirectoryDatasetError, "duplicate topic"):
            DirectoryReader(self.root)

        table = self.root / "cameras" / "cam0" / "timestamps.csv"
        table.write_text("timestamp,filename\n1,first.png\n", encoding="utf-8")
        self._write_manifest([self._camera_stream()], [])
        reader = DirectoryReader(self.root)
        with self.assertRaisesRegex(DirectoryDatasetError, "invalid CSV header"):
            reader.index_images("/cam0/image_raw")

    def test_invalid_timestamp_numeric_and_duplicate_filename_are_rejected(self):
        self._write_images()
        self._write_manifest([self._camera_stream()], [])
        table = self.root / "cameras" / "cam0" / "timestamps.csv"
        table.write_text(
            "timestamp_ns,filename\n1.5,first.png\n", encoding="utf-8")
        with self.assertRaisesRegex(DirectoryDatasetError, "integer nanosecond"):
            DirectoryReader(self.root).index_images("/cam0/image_raw")

        table.write_text(
            "timestamp_ns,filename\n1,first.png\n2,first.png\n", encoding="utf-8")
        with self.assertRaisesRegex(DirectoryDatasetError, "duplicate image filename"):
            DirectoryReader(self.root).index_images("/cam0/image_raw")

        (self.root / "imu" / "imu0.csv").write_text(
            "timestamp_ns,wx,wy,wz,ax,ay,az\n1,nan,0,0,0,0,0\n",
            encoding="utf-8",
        )
        self._write_manifest([self._camera_stream()], [self._imu_stream()])
        with self.assertRaisesRegex(DirectoryDatasetError, "finite number"):
            DirectoryReader(self.root).read_imu("/imu0")

    def test_unordered_timestamps_are_rejected_and_bad_image_is_lazy_error(self):
        (self.images / "first.png").write_bytes(b"not an image")
        valid = np.full((3, 4), 12, dtype=np.uint8)
        self.assertTrue(cv2.imwrite(str(self.images / "second.png"), valid))
        self.assertTrue(cv2.imwrite(str(self.images / "third.png"), valid))
        table = self.root / "cameras" / "cam0" / "timestamps.csv"
        table.write_text(
            "timestamp_ns,filename\n"
            "2000000000,first.png\n"
            "1000000000,second.png\n"
            "2000000000,third.png\n",
            encoding="utf-8",
        )
        self._write_manifest([self._camera_stream()], [])
        with self.assertRaisesRegex(DirectoryDatasetError, "strictly increasing"):
            DirectoryReader(self.root).index_images("/cam0/image_raw")
        table.write_text("timestamp_ns,filename\n1000000000,second.png\n2000000000,first.png\n3000000000,third.png\n", encoding="utf-8")
        dataset = DirectoryReader(self.root).index_images("/cam0/image_raw")
        with self.assertRaisesRegex(DirectoryDatasetError, "could not decode"):
            dataset.get_by_entry(dataset.index[1])
        with self.assertRaisesRegex(RuntimeError, "Could not find topic"):
            DirectoryReader(self.root).read_images("/missing")


if __name__ == "__main__":
    unittest.main()
