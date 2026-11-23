import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import yaml

from kalibr_no_ros.native_bridge.camera_geometry import create_camera_geometry
from kalibr_no_ros.native_bridge.target_detector import create_target_detector

from kalibr_tools.fixed_stereo_validation import (
    FixedCameraValidationError,
    _distribution,
    _camera_quality,
    _paired_frames,
    _safe_replace_directory,
    combined_reprojection_rms,
    parse_calibration_specs,
)


class FixedCameraValidationTest(unittest.TestCase):
    def test_native_geometry_preserves_complete_intrinsics_and_distortion(self):
        try:
            import aslam_cv_backend
        except ImportError:
            self.skipTest("native camera bindings are unavailable")
        for model, distortion_count, skew in (
                ("pinhole-radtan", 4, False),
                ("pinhole-equi", 4, False),
                ("pinhole-radtan5", 5, False),
                ("pinhole-radtan8", 8, False),
                ("pinhole-opencv-fisheye", 4, True)):
            with self.subTest(model=model):
                camera = {
                    "id": "cam0", "model": model,
                    "intrinsics": [400.0, 410.0, 320.0, 240.0]
                    + ([0.02] if skew else []),
                    "distortion_coeffs": [0.001 * (index + 1)
                                          for index in range(distortion_count)],
                    "resolution": [640, 480],
                }
                geometry = create_camera_geometry(camera)
                np.testing.assert_array_equal(
                    geometry.projection().getParameters().flatten(),
                    camera["intrinsics"])
                np.testing.assert_array_equal(
                    geometry.projection().distortion().getParameters().flatten(),
                    camera["distortion_coeffs"])
                self.assertEqual(geometry.projection().ru(), 640)
                self.assertEqual(geometry.projection().rv(), 480)

    def test_detector_uses_supplied_geometry_and_board_parameters(self):
        try:
            from kalibr_camera_calibration import CameraCalibrator
        except ImportError:
            self.skipTest("native target detector bindings are unavailable")
        targets = (
            {"target_type": "aprilgrid", "tagRows": 6, "tagCols": 6,
             "tagSize": 0.04, "tagSpacing": 0.3, "tagStartId": 80},
            {"target_type": "checkerboard", "targetRows": 6,
             "targetCols": 6, "rowSpacingMeters": 0.04,
             "colSpacingMeters": 0.04},
            {"target_type": "circlegrid", "targetRows": 6,
             "targetCols": 6, "spacingMeters": 0.04,
             "asymmetricGrid": False},
        )
        geometry = object()
        for target in targets:
            with self.subTest(target=target["target_type"]), \
                    tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "target.yaml"
                path.write_text(yaml.safe_dump(target), encoding="utf-8")
                with mock.patch.object(CameraCalibrator, "TargetDetector") as factory:
                    detector = create_target_detector(
                        path, geometry, 7, 2.0,
                        show_extraction=True, extraction_stepping=True)
                arguments, keywords = factory.call_args
                self.assertEqual(arguments[0].getTargetType(), target["target_type"])
                self.assertIs(arguments[1], geometry)
                self.assertEqual(keywords, {
                    "showCorners": True, "showOneStep": True,
                    "windowHalfSizePx": 7, "maxDisplacementPx": 2.0,
                })
                self.assertIs(detector, factory.return_value.detector)

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

    def test_camera_quality_pools_points_and_measures_straight_lines(self):
        camera = {"model": "pinhole-equi", "intrinsics": [400., 400., 320., 240.],
                  "distortion_coeffs": [0., 0., 0., 0.], "resolution": [640, 480]}
        import cv2
        xyz = np.array([[x, y, 0.] for y in range(3) for x in range(3)], dtype=float)
        image, _ = cv2.fisheye.projectPoints(xyz.reshape(-1, 1, 3), np.zeros(3),
                    np.array([0., 0., 10.]), np.array([[400., 0., 320.], [0., 400., 240.], [0., 0., 1.]]), np.zeros(4))
        corners = [{"measurement_px": p.tolist(), "target_xyz_m": t.tolist()}
                   for p, t in zip(image.reshape(-1, 2), xyz)]
        frames = [{"detected": True, "_residual_norms_px": np.array(values), "_line_corners": corners}
                  for values in ([1.], [2., 3., 4., 5.])]
        quality = _camera_quality(camera, frames)
        self.assertAlmostEqual(quality["p95_px"], 4.8)
        self.assertEqual(quality["max_px"], 5.)
        self.assertEqual(quality["line_straightness"]["count"], 36)
        self.assertLess(quality["line_straightness"]["rms_px"], 1e-10)

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
