import json
import tempfile
import unittest
from pathlib import Path

from kalibr_no_ros.fixed_camera_validation import (
    FixedCameraValidationError,
    _distribution,
    _paired_frames,
    _safe_replace_directory,
    combined_reprojection_rms,
    parse_calibration_specs,
)


class FixedCameraValidationTest(unittest.TestCase):
    def test_distribution_reports_rms_p95_and_max_of_point_norms(self):
        statistics = _distribution([1.0, 2.0, 3.0, 4.0])
        self.assertEqual(statistics["count"], 4)
        self.assertAlmostEqual(statistics["rms_px"], (30.0 / 4.0) ** 0.5)
        self.assertAlmostEqual(statistics["p95_px"], 3.85)
        self.assertEqual(statistics["max_px"], 4.0)

    def test_combined_rms_uses_corner_weighted_squared_error(self):
        rows = [
            {"squared_error_px2": 4.0, "corner_count": 4},
            {"squared_error_px2": 12.0, "corner_count": 12},
        ]
        self.assertAlmostEqual(combined_reprojection_rms(rows), 1.0)

    def test_calibration_specs_support_labels_and_reject_duplicates(self):
        with tempfile.TemporaryDirectory() as directory:
            first = Path(directory) / "first.yaml"
            second = Path(directory) / "second.yaml"
            first.write_text("{}\n", encoding="utf-8")
            second.write_text("{}\n", encoding="utf-8")
            parsed = parse_calibration_specs([
                "reference={}".format(first), str(second)])
            self.assertEqual([row[0] for row in parsed], ["reference", "second"])
            with self.assertRaisesRegex(
                    FixedCameraValidationError, "duplicate"):
                parse_calibration_specs([
                    "same={}".format(first), "same={}".format(second)])

    def test_pairing_is_one_to_one_and_uses_smallest_time_difference(self):
        left = [
            {"detected": True, "source_timestamp_ns": 100,
             "source_index": 0},
            {"detected": True, "source_timestamp_ns": 200,
             "source_index": 1},
        ]
        right = [
            {"detected": True, "source_timestamp_ns": 103,
             "source_index": 0},
            {"detected": True, "source_timestamp_ns": 198,
             "source_index": 1},
        ]
        pairs = list(_paired_frames(left, right, 5))
        self.assertEqual([row[0] for row in pairs], [2, 3])
        self.assertEqual(len(pairs), 2)

    def test_force_refuses_unmanaged_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            destination = root / "output"
            destination.mkdir()
            (destination / "user.txt").write_text("keep", encoding="utf-8")
            stage = root / "stage"
            stage.mkdir()
            with self.assertRaisesRegex(
                    FixedCameraValidationError, "inventory"):
                _safe_replace_directory(stage, destination, True)
            self.assertEqual(
                (destination / "user.txt").read_text(encoding="utf-8"),
                "keep")

    def test_force_replaces_legacy_managed_inventory_without_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            destination = root / "output"
            destination.mkdir()
            managed = {
                "fixed_camera_validation.json",
                "README_ZH.md",
                "run_manifest.json",
            }
            for name in managed:
                (destination / name).write_text("old", encoding="utf-8")
            (destination / ".inventory.json").write_text(
                json.dumps(sorted(managed)), encoding="utf-8")
            stage = root / "stage"
            stage.mkdir()
            (stage / "marker").write_text("new", encoding="utf-8")
            _safe_replace_directory(stage, destination, True)
            self.assertEqual(
                (destination / "marker").read_text(encoding="utf-8"),
                "new")

    def test_force_accepts_registered_visualization_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            destination = root / "output"
            image = "visualizations/reference/cam0_det/corners_0.jpg"
            managed = {
                "fixed_camera_validation.json",
                "fixed_camera_validation.csv",
                "README_ZH.md",
                "run_manifest.json",
                image,
            }
            for name in managed:
                path = destination / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("old", encoding="utf-8")
            (destination / ".inventory.json").write_text(
                json.dumps(sorted(managed)), encoding="utf-8")
            stage = root / "stage"
            stage.mkdir()
            (stage / "marker").write_text("new", encoding="utf-8")
            _safe_replace_directory(stage, destination, True)
            self.assertEqual(
                (destination / "marker").read_text(encoding="utf-8"),
                "new")


if __name__ == "__main__":
    unittest.main()
