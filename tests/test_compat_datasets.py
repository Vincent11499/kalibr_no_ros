import numpy as np
import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "bag_io" / "src"), str(ROOT / "core" / "src")]

from kalibr_bag_io import ImageRecord, ImuRecord
from kalibr_no_ros.datasets import BagImageDatasetReader, BagImuDatasetReader


class FakeTime:
    def __init__(self, seconds, nanoseconds=None):
        self.seconds = seconds
        self.nanoseconds = nanoseconds


class FakeReader:
    def __init__(self):
        self.images = [
            ImageRecord(i * 1_000_000_000, i * 1_000_000_000 + 10, "mono8", np.zeros((2, 3), np.uint8))
            for i in range(1, 5)
        ]
        self.imu = [
            ImuRecord(
                i * 1_000_000_000,
                i * 1_000_000_000 + 10,
                np.full(3, i, dtype=float),
                np.full(3, -i, dtype=float),
            )
            for i in range(1, 5)
        ]

    def read_images(self, topic):
        return self.images

    def index_images(self, topic):
        return FakeImageDataset(self.images)

    def read_imu(self, topic):
        return self.imu

    @staticmethod
    def _crop(records, from_to):
        return BagReaderSelection.crop(records, from_to)

    @staticmethod
    def _decimate(records, frequency):
        return BagReaderSelection.decimate(records, frequency)


class BagReaderSelection:
    @staticmethod
    def crop(records, from_to):
        if from_to is None:
            return records
        lower = records[0].header_timestamp_ns + int(from_to[0] * 1e9)
        upper = records[0].header_timestamp_ns + int(from_to[1] * 1e9)
        return [x for x in records if lower <= x.header_timestamp_ns <= upper]

    @staticmethod
    def decimate(records, frequency):
        if frequency is None:
            return records
        selected = []
        last = None
        for record in records:
            if last is None or record.header_timestamp_ns - last >= 1e9 / frequency:
                selected.append(record)
                last = record.header_timestamp_ns
        return selected


class FakeImageDataset:
    def __init__(self, images):
        self.index = images

    def get_by_entry(self, entry):
        return entry

    def close(self):
        pass


class CompatibilityTest(unittest.TestCase):
  def test_image_reader_contract(self):
    dataset = BagImageDatasetReader(
        "unused", "/cam", bag_from_to=(1.0, 3.0), reader=FakeReader(), time_factory=FakeTime
    )
    self.assertEqual(len(dataset.index), 4)
    self.assertEqual(dataset.numImages(), 3)
    samples = list(dataset)
    self.assertEqual([sample[0].seconds for sample in samples], [2, 3, 4])
    self.assertTrue(all(sample[1].shape == (2, 3) for sample in samples))


  def test_imu_reader_contract(self):
    dataset = BagImuDatasetReader(
        "unused", "/imu", bag_from_to=(0.0, 2.0), reader=FakeReader(), time_factory=FakeTime
    )
    self.assertEqual(len(dataset.index), 4)
    self.assertEqual(dataset.numMessages(), 3)
    samples = list(dataset)
    self.assertEqual([sample[0].seconds for sample in samples], [1, 2, 3])
    np.testing.assert_array_equal(samples[-1][1], [3.0, 3.0, 3.0])


  def test_shuffle_matches_kalibr_in_place_semantics(self):
    dataset = BagImageDatasetReader(
        "unused", "/cam", reader=FakeReader(), time_factory=FakeTime
    )
    original = dataset.indices.copy()
    np.random.seed(7)
    iterator = dataset.readDatasetShuffle()

    self.assertIs(iterator.indices, dataset.indices)
    self.assertFalse(np.array_equal(dataset.indices, original))
