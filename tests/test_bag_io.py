import numpy as np
import tempfile
import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "bag_io" / "src"), str(ROOT / "core" / "src")]

from kalibr_bag_io import BagReader, BagWriter, ImageRecord, ImuRecord


class BagIoTest(unittest.TestCase):
  def setUp(self):
    self.temporary = tempfile.TemporaryDirectory()
    self.directory = Path(self.temporary.name)

  def tearDown(self):
    self.temporary.cleanup()

  def test_standard_message_round_trip(self):
    path = self.directory / "roundtrip.bag"
    pixels = np.arange(20, dtype=np.uint8).reshape(4, 5)
    image = ImageRecord(1_000_000_123, 1_000_000_456, "mono8", pixels, "cam", 7)
    imu = ImuRecord(
        2_000_000_123,
        2_000_000_456,
        np.array([1.0, 2.0, 3.0]),
        np.array([4.0, 5.0, 6.0]),
        "imu",
        8,
    )
    with BagWriter(path) as writer:
        writer.write_image("/cam", image)
        writer.write_compressed_image("/compressed", image)
        writer.write_imu("/imu", imu)

    reader = BagReader(path)
    assert [(x.name, x.message_count) for x in reader.topics()] == [
        ("/cam", 1),
        ("/compressed", 1),
        ("/imu", 1),
    ]
    raw = reader.read_images("/cam")[0]
    compressed = reader.read_images("/compressed")[0]
    measured = reader.read_imu("/imu")[0]
    np.testing.assert_array_equal(raw.image, pixels)
    np.testing.assert_array_equal(compressed.image, pixels)
    np.testing.assert_array_equal(measured.angular_velocity, imu.angular_velocity)
    np.testing.assert_array_equal(measured.linear_acceleration, imu.linear_acceleration)
    self.assertEqual((raw.header_timestamp_ns, raw.record_timestamp_ns), (
        image.header_timestamp_ns,
        image.record_timestamp_ns,
    ))
    self.assertEqual((measured.header_timestamp_ns, measured.record_timestamp_ns), (
        imu.header_timestamp_ns,
        imu.record_timestamp_ns,
    ))


  def test_crop_and_decimation_preserve_header_time_order(self):
    path = self.directory / "selection.bag"
    times = [3_000_000_000, 1_000_000_000, 2_000_000_000, 4_000_000_000]
    with BagWriter(path) as writer:
        for sequence, stamp in enumerate(times):
            record = ImageRecord(
                stamp,
                10_000_000_000 + sequence,
                "mono8",
                np.full((2, 2), sequence, dtype=np.uint8),
            )
            writer.write_image("/cam", record)
    reader = BagReader(path)
    selected = reader.read_images("/cam", from_to=(0.5, 3.0), frequency=1.0)
    self.assertEqual([x.header_timestamp_ns for x in selected], [2_000_000_000, 3_000_000_000, 4_000_000_000])


  def test_invalid_topic_and_selection(self):
    path = self.directory / "empty.bag"
    with BagWriter(path):
        pass
    reader = BagReader(path)
    with self.assertRaisesRegex(RuntimeError, "Could not find topic"):
        reader.read_images("/missing")
    self.assertEqual(reader._crop([], (0.0, 1.0)), [])
    with self.assertRaisesRegex(RuntimeError, "smaller"):
        reader._crop([], (2.0, 1.0))
    with self.assertRaisesRegex(RuntimeError, "greater than zero"):
        reader._decimate([], 0)
