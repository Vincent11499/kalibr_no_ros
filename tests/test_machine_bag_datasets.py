import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))


from kalibr_bag_io import BagWriter, ImageRecord, ImuRecord
from kalibr_bag_io.model import TopicInfo
from kalibr_no_ros.machine import datasets as machine_datasets
from kalibr_no_ros.machine.config import load_workflow_config
from kalibr_no_ros.task.document import TaskError, dump_yaml


class _FakeBagReader:
    topics_value = []
    topics_calls = 0

    def __init__(self, path):
        self.path = Path(path)

    def topics(self):
        type(self).topics_calls += 1
        return list(self.topics_value)


class MachineBagDatasetTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.machine = self.root / "EVT312-02" / "1029"
        self.machine.mkdir(parents=True)

    def tearDown(self):
        self.temporary.cleanup()

    def _config(self, bag_topics, datasets=None):
        document = {
            "schema_version": "1.0.0",
            "kind": "machine_calibration_workflow",
            "template": "EVT320",
            "machine_dir": str(self.machine),
            "stereo_imu": {"mode": "standard"},
            "bag_topics": bag_topics,
        }
        if datasets is not None:
            document["datasets"] = datasets
        path = self.root / "user.yaml"
        dump_yaml(document, path)
        return path

    @staticmethod
    def _stereo_topics():
        return {"cam0": "/left/image", "cam1": "/right/image"}

    @staticmethod
    def _stereo_imu_topics():
        return {
            "cam0": "/left/image",
            "cam1": "/right/image",
            "imu0": "/imu/data",
        }

    def _write_directory_dataset(self, name):
        dataset = self.machine / name
        cameras = []
        for camera_id in ("cam0", "cam1"):
            images = dataset / camera_id
            images.mkdir(parents=True)
            timestamps = dataset / (camera_id + ".csv")
            timestamps.write_text(
                "timestamp_ns,filename\n", encoding="utf-8")
            cameras.append({
                "id": camera_id,
                "images": camera_id,
                "timestamps": timestamps.name,
            })
        dump_yaml({
            "schema_version": "1.0.0",
            "type": "kalibr_directory_dataset",
            "dataset_id": name,
            "cameras": cameras,
            "imus": [],
        }, dataset / "dataset.yaml")
        return dataset

    def test_workflow_accepts_strict_bag_topics_and_ros1_names(self):
        configured = load_workflow_config(self._config(
            {
                "stereo": self._stereo_topics(),
                "stereo_imu": self._stereo_imu_topics(),
                "mono": {"cam0": "/usb/image"},
            },
            datasets={
                "stereo": "stereo_261124_1000.bag",
                "stereo_imu": "stereo_imu_261124_1010.bag",
                "mono": "usb_261124_1020.bag",
            }))
        self.assertEqual(
            configured["bag_topics"]["stereo"], self._stereo_topics())
        self.assertTrue(configured["datasets"]["stereo"].endswith(".bag"))

    def test_workflow_rejects_incomplete_duplicate_or_unknown_bag_topics(self):
        invalid = (
            {"stereo": {"cam0": "/left/image"}},
            {"stereo": {"cam0": "/same", "cam1": "/same"}},
            {"stereo_baselink": {"cam0": "/left", "cam1": "/right"}},
        )
        for index, value in enumerate(invalid):
            with self.subTest(index=index):
                with self.assertRaisesRegex(TaskError, "bag_topics"):
                    load_workflow_config(self._config(value))

    def test_discovery_accepts_ros1_file_and_ros2_or_directory_roots(self):
        ros1 = self.machine / "stereo_261124_1000.bag"
        ros1.write_bytes(b"ros1")
        self.assertEqual(
            machine_datasets._discover_dataset_path(
                self.machine, "stereo", ros1.name),
            ros1.resolve())
        ros1.unlink()

        ros2 = self.machine / "stereo_261124_1010"
        ros2.mkdir()
        (ros2 / "metadata.yaml").write_text("version: 5\n", encoding="utf-8")
        self.assertEqual(
            machine_datasets._discover_dataset_path(self.machine, "stereo"),
            ros2.resolve())
        ros2.rename(self.root / "saved_ros2")

        directory = self._write_directory_dataset("stereo_261124_1020")
        self.assertEqual(
            machine_datasets._discover_dataset_path(self.machine, "stereo"),
            directory.resolve())

    def test_baselink_does_not_accept_bag_files(self):
        bag = self.machine / "stereo_base_link_261124_1000.bag"
        bag.write_bytes(b"not-supported")
        self.assertIsNone(machine_datasets._discover_dataset_path(
            self.machine, "stereo_baselink"))
        with self.assertRaisesRegex(TaskError, "valid directory name"):
            machine_datasets._discover_dataset_path(
                self.machine, "stereo_baselink", bag.name)

    def test_directory_record_preserves_inventory_and_declares_format(self):
        dataset = self._write_directory_dataset("stereo_261124_1000")
        record = machine_datasets.dataset_fingerprint(dataset, "stereo")
        self.assertEqual(record["dataset_type"], "directory")
        self.assertEqual(record["storage_format"], "directory")
        self.assertEqual(record["dataset_id"], dataset.name)

    def test_ros1_bag_requires_topics_and_hashes_complete_file(self):
        bag = self.machine / "stereo_261124_1000.bag"
        bag.write_bytes(b"first bag payload")
        with self.assertRaisesRegex(TaskError, "requires bag_topics.stereo"):
            machine_datasets.dataset_fingerprint(bag, "stereo")

        _FakeBagReader.topics_value = [
            TopicInfo("/left/image", "sensor_msgs/msg/Image", 4),
            TopicInfo("/right/image", "sensor_msgs/msg/CompressedImage", 5),
        ]
        with mock.patch.object(machine_datasets, "BagReader", _FakeBagReader):
            first = machine_datasets.dataset_fingerprint(
                bag, "stereo", self._stereo_topics())
            bag.write_bytes(b"second bag payload")
            second = machine_datasets.dataset_fingerprint(
                bag, "stereo", self._stereo_topics())
        self.assertEqual(first["name"], "stereo_261124_1000")
        self.assertIsNone(first["dataset_id"])
        self.assertEqual(first["dataset_type"], "bag")
        self.assertEqual(first["storage_format"], "ros1")
        self.assertEqual(first["image_counts"], {"cam0": 4, "cam1": 5})
        self.assertNotEqual(first["sha256"], second["sha256"])

    def test_ros2_bag_hashes_all_regular_files_and_rejects_symlinks(self):
        bag = self.machine / "stereo_imu_261124_1000"
        bag.mkdir()
        (bag / "metadata.yaml").write_text("version: 5\n", encoding="utf-8")
        (bag / "data.db3").write_bytes(b"sqlite payload")
        nested = bag / "attachments"
        nested.mkdir()
        (nested / "note.txt").write_text("trace", encoding="utf-8")
        _FakeBagReader.topics_value = [
            TopicInfo("/left/image", "sensor_msgs/msg/Image", 3),
            TopicInfo("/right/image", "sensor_msgs/msg/Image", 3),
            TopicInfo("/imu/data", "sensor_msgs/msg/Imu", 30),
        ]
        with mock.patch.object(machine_datasets, "BagReader", _FakeBagReader):
            record, entry, statistics = machine_datasets._fingerprint_dataset(
                bag, "stereo_imu", bag_topics=self._stereo_imu_topics())
            self.assertEqual(record["storage_format"], "ros2")
            self.assertEqual(record["file_count"], 3)
            self.assertEqual(record["imu_counts"], {"imu0": 30})
            self.assertEqual(statistics["mode"], "bag_content_hash")
            self.assertEqual(set(entry["files"]), {
                "metadata.yaml", "data.db3", "attachments/note.txt"})

            external = self.root / "external.db3"
            external.write_bytes(b"external")
            (bag / "external.db3").symlink_to(external)
            _FakeBagReader.topics_calls = 0
            with self.assertRaisesRegex(TaskError, "symlink"):
                machine_datasets.dataset_fingerprint(
                    bag, "stereo_imu", self._stereo_imu_topics())
            self.assertEqual(_FakeBagReader.topics_calls, 0)

    def test_bag_topic_types_are_checked_before_hashing(self):
        bag = self.machine / "usb_261124_1000.bag"
        bag.write_bytes(b"payload")
        _FakeBagReader.topics_value = [
            TopicInfo("/usb/image", "custom_msgs/msg/Image", 2),
        ]
        with mock.patch.object(machine_datasets, "BagReader", _FakeBagReader):
            with self.assertRaisesRegex(TaskError, "unsupported type"):
                machine_datasets.dataset_fingerprint(
                    bag, "mono", {"cam0": "/usb/image"})

    def test_current_bag_topic_change_is_not_a_legacy_cache_migration(self):
        old_record = {
            "role": "stereo",
            "path": str(self.machine / "stereo_261124_1000.bag"),
            "name": "stereo_261124_1000",
            "dataset_id": None,
            "dataset_type": "bag",
            "storage_format": "ros1",
            "bag_topics": self._stereo_topics(),
            "sha256": "a" * 64,
            "fingerprint_type": "content",
            "file_count": 1,
        }
        new_record = dict(old_record, bag_topics={
            "cam0": "/right/image", "cam1": "/left/image"})
        file_record = {
            "size": 1, "mtime_ns": 1, "ctime_ns": 1,
            "inode": 1, "device": 1, "kind": "structured",
            "sha256": "b" * 64,
        }
        cached = {
            "record": old_record,
            "files": {"stereo_261124_1000.bag": file_record},
        }
        current = {
            "record": new_record,
            "files": {"stereo_261124_1000.bag": file_record},
        }

        self.assertFalse(machine_datasets.cache_entry_can_migrate(
            cached, old_record, current))

    def test_real_ros1_and_ros2_bags_are_fingerprinted(self):
        pixels = np.arange(20, dtype=np.uint8).reshape(4, 5)
        topics = self._stereo_imu_topics()
        inputs = (
            (self.machine / "stereo_imu_261124_1100.bag", "ros1"),
            (self.machine / "stereo_imu_261124_1110", "ros2"),
        )
        for path, storage_format in inputs:
            with self.subTest(storage_format=storage_format):
                image = ImageRecord(
                    1_000_000_000, 1_000_000_100, "mono8", pixels)
                imu = ImuRecord(
                    900_000_000, 900_000_100,
                    np.array([0.1, 0.2, 0.3]),
                    np.array([0.0, 0.0, 9.81]))
                with BagWriter(path) as writer:
                    writer.write_image(topics["cam0"], image)
                    writer.write_compressed_image(topics["cam1"], image)
                    writer.write_imu(topics["imu0"], imu)

                record = machine_datasets.dataset_fingerprint(
                    path, "stereo_imu", topics)
                self.assertEqual(record["storage_format"], storage_format)
                self.assertEqual(record["image_counts"], {
                    "cam0": 1, "cam1": 1})
                self.assertEqual(record["imu_counts"], {"imu0": 1})
                self.assertGreater(record["file_count"], 0)


if __name__ == "__main__":
    unittest.main()
