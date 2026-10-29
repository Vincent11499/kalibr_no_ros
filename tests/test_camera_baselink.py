"""Contracts for camera-to-base_link direct-difference calibration."""

import csv
import tempfile
import unittest
from contextlib import redirect_stderr
from io import StringIO
from pathlib import Path

import cv2
import numpy as np

from kalibr_no_ros.camera_baselink import (
    CameraBaselinkError,
    _correct_output_rotation,
    _relative_rotation_pairs,
    _write_rotation_summary,
    optical_axis_pitch_deg,
    platform_position,
    relative_rotation_angles_rad,
    solve_direct_rotation,
)
from kalibr_no_ros.artifacts import RunContext
from kalibr_no_ros.cli import build_parser
from kalibr_no_ros.evaluation import compute_metrics
from kalibr_no_ros.jobs import CAMERA_BASELINK_JOB
from kalibr_no_ros.report_plots import _summary_sections
from kalibr_no_ros.reporting import _summary
from kalibr_no_ros.task import dump_yaml, load_task
from kalibr_no_ros.validation import validate_task


class CameraBaselinkTest(unittest.TestCase):
    def test_cli_and_task_job_are_public(self):
        parsed = build_parser().parse_args([
            "calibrate", "cameras-baselink", "--config", "task.yaml",
            "--output-dir", "result",
        ])
        self.assertEqual(parsed.calibration, "cameras-baselink")
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            build_parser().parse_args([
                "calibrate", "cameras-baselink", "--config", "task.yaml",
                "--output-dir", "result", "--initialization", "seed.yaml",
            ])

    def test_artifacts_and_metrics_use_baselink_type_without_stereo_pair(self):
        context = RunContext(CAMERA_BASELINK_JOB)
        self.assertEqual(
            context.artifacts["calibration_type"], "cameras_baselink")
        camera = {
            "id": "cam0", "camera_model": "pinhole",
            "distortion_model": "equidistant",
            "intrinsics": [40.0, 40.0, 32.0, 24.0],
            "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
            "resolution": [64, 48], "frames": [],
            "source_frame_count": 0, "selected_frame_count": 0,
            "movement_residual_mm": 0.2, "effective_pairs": 2,
            "pitch": 90.0,
            "rotation_left_correction": [
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
            ],
        }
        artifacts = dict(context.artifacts, state="completed", cameras=[camera])
        result_camera = {key: value for key, value in camera.items()
                         if key not in {"frames", "source_frame_count",
                                        "selected_frame_count"}}
        result_camera["T_base_camera"] = np.eye(4).tolist()
        metrics = compute_metrics(artifacts, {
            "schema_version": "1.0.0", "kind": "calibration_result",
            "calibration_type": "cameras_baselink",
            "cameras": [result_camera],
        })
        self.assertEqual(metrics["calibration_type"], "cameras_baselink")
        self.assertEqual(metrics["stereo_pairs"], {})
        self.assertEqual(metrics["cameras"]["cam0"]["movement_residual_mm"], 0.2)
        self.assertEqual(metrics["cameras"]["cam0"]["pitch"], 90.0)
        self.assertEqual(
            metrics["cameras"]["cam0"]["rotation_left_correction"],
            [[1.0, 0.0, 0.0],
             [0.0, -1.0, 0.0],
             [0.0, 0.0, -1.0]])
        assessment = {
            "status": "not_evaluated",
            "reference_grading": {"status": "unavailable"},
            "rules": [],
        }
        self.assertIn("pitch=90 deg", _summary(metrics, assessment))
        sections = {
            title: (headers, rows)
            for title, headers, rows in _summary_sections(
                metrics, assessment, {"cameras": [result_camera]})
        }
        headers, rows = sections["base_link direct-difference solve"]
        self.assertEqual(headers[1], "Pitch [deg]")
        self.assertEqual(rows[0][1], 90.0)
        self.assertEqual(rows[0][2], [
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

    def test_pitch_is_unsigned_optical_axis_angle_above_horizontal(self):
        self.assertAlmostEqual(optical_axis_pitch_deg(np.eye(3)), 90.0)
        horizontal = np.asarray([
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ])
        self.assertAlmostEqual(optical_axis_pitch_deg(horizontal), 0.0)
        self.assertAlmostEqual(
            optical_axis_pitch_deg(np.diag([1.0, -1.0, -1.0])), 90.0)

    def test_output_rotation_corrections_are_left_multiplied(self):
        solved = np.asarray([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(
            _correct_output_rotation(solved, [
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
            ]),
            np.diag([1.0, -1.0, -1.0]) @ solved)
        np.testing.assert_allclose(
            _correct_output_rotation(solved, [
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0],
            ]),
            np.diag([-1.0, 1.0, -1.0]) @ solved)
        np.testing.assert_allclose(
            _correct_output_rotation(solved, np.eye(3).tolist()), solved)
        with self.assertRaisesRegex(CameraBaselinkError, r"SO\(3\)"):
            _correct_output_rotation(solved, [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0],
            ])

    def test_rotation_summary_records_all_pnp_pair_rotation_angles(self):
        angle10 = np.deg2rad(10.0)
        angle30 = np.deg2rad(30.0)
        rotations = np.asarray([
            np.eye(3),
            [[np.cos(angle10), -np.sin(angle10), 0.0],
             [np.sin(angle10), np.cos(angle10), 0.0],
             [0.0, 0.0, 1.0]],
            [[np.cos(angle30), -np.sin(angle30), 0.0],
             [np.sin(angle30), np.cos(angle30), 0.0],
             [0.0, 0.0, 1.0]],
        ])
        np.testing.assert_allclose(
            np.rad2deg(relative_rotation_angles_rad(rotations)),
            [10.0, 30.0, 20.0], atol=1e-12)
        artifacts = {
            "cameras": [{
                "id": "cam0",
                "relative_rotation_pairs": _relative_rotation_pairs(
                    ["0_0_0.png", "10_0_0.png", "10_10_0.png"],
                    rotations),
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rotation_summary.csv"
            _write_rotation_summary(artifacts, path)
            with path.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[0]["camera"], "cam0")
            self.assertEqual(rows[0]["from_frame"], "0_0_0.png")
            self.assertEqual(rows[0]["to_frame"], "10_0_0.png")
            self.assertAlmostEqual(float(rows[0]["theta_deg"]), 10.0)
            self.assertEqual(rows[1]["from_frame"], "0_0_0.png")
            self.assertEqual(rows[1]["to_frame"], "10_10_0.png")
            self.assertAlmostEqual(float(rows[1]["theta_deg"]), 30.0)
            self.assertEqual(rows[2]["from_frame"], "10_0_0.png")
            self.assertEqual(rows[2]["to_frame"], "10_10_0.png")
            self.assertAlmostEqual(float(rows[2]["theta_deg"]), 20.0)
            self.assertTrue(all(row["pair_count"] == "3" for row in rows))
            for row in rows:
                self.assertAlmostEqual(
                    float(row["theta_deg_median"]), 20.0)
            self.assertNotIn("pitch", rows[0])

    def test_filename_positions_are_scaled_and_strict(self):
        np.testing.assert_allclose(
            platform_position("cameras/cam0/43_-5_0.bmp", 0.01),
            [0.43, -0.05, 0.0])
        with self.assertRaisesRegex(CameraBaselinkError, "<x>_<y>_<z>"):
            platform_position("frame_001.bmp", 0.01)

    def test_direct_difference_recovers_rotation_without_alpha(self):
        angle = np.deg2rad(31.0)
        rotation = np.asarray([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        platform = np.asarray([
            [0.0, 0.0, 0.0],
            [0.2, 0.0, 0.0],
            [0.2, 0.15, 0.0],
            [-0.1, 0.15, 0.0],
        ])
        translations = [np.zeros(3)]
        for difference in platform[:-1] - platform[1:]:
            translations.append(translations[-1] + rotation.T @ difference)
        estimate, diagnostics = solve_direct_rotation(
            platform, np.asarray(translations))
        np.testing.assert_allclose(estimate, rotation, atol=1e-12)
        self.assertEqual(diagnostics["effective_pairs"], 6)
        self.assertLess(diagnostics["movement_residual_mm"], 1e-9)
        self.assertAlmostEqual(diagnostics["pitch"], 90.0)
        self.assertNotIn("alpha", diagnostics)
        order = [2, 0, 3, 1]
        reordered, reordered_diagnostics = solve_direct_rotation(
            platform[order], np.asarray(translations)[order])
        np.testing.assert_allclose(reordered, rotation, atol=1e-12)
        self.assertEqual(reordered_diagnostics["effective_pairs"], 6)

    def test_rank_one_motion_is_rejected(self):
        values = np.asarray([
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [0.2, 0.0, 0.0],
        ])
        with self.assertRaisesRegex(CameraBaselinkError, "rank-1"):
            solve_direct_rotation(values, -values)

    def test_structural_validation_uses_fixed_camera_result_and_filename_positions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            dataset = root / "platform_261021_1630"
            images = dataset / "cameras/cam0"
            images.mkdir(parents=True)
            names = ["0_0_0.bmp", "10_0_0.bmp", "10_10_0.bmp"]
            for name in names:
                self.assertTrue(cv2.imwrite(
                    str(images / name), np.zeros((48, 64), dtype=np.uint8)))
            (images / "timestamps.csv").write_text(
                "timestamp_ns,filename\n" + "".join(
                    "{},{}\n".format(index + 1, name)
                    for index, name in enumerate(names)),
                encoding="utf-8")
            dump_yaml({
                "schema_version": "1.0.0",
                "type": "kalibr_directory_dataset",
                "dataset_id": dataset.name,
                "cameras": [{
                    "id": "cam0",
                    "timestamps": "cameras/cam0/timestamps.csv",
                    "images": "cameras/cam0",
                    "resolution": [64, 48],
                    "clock_source": "test",
                }],
                "imus": [],
            }, dataset / "dataset.yaml")
            dump_yaml({
                "schema_version": "1.0.0",
                "kind": "calibration_target",
                "target_type": "aprilgrid",
                "tagRows": 3,
                "tagCols": 3,
                "tagSize": 0.04,
                "tagSpacing": 0.3,
                "tagStartId": 0,
            }, root / "target.yaml")
            dump_yaml({
                "schema_version": "1.0.0",
                "kind": "calibration_result",
                "calibration_type": "cameras",
                "cameras": [{
                    "id": "cam0",
                    "camera_model": "pinhole",
                    "distortion_model": "equidistant",
                    "intrinsics": [40.0, 40.0, 32.0, 24.0],
                    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                    "resolution": [64, 48],
                    "rostopic": "/cam0",
                }],
            }, root / "cameras.yaml")
            task = {
                "schema_version": "1.0.0",
                "kind": "calibration_task",
                "job": CAMERA_BASELINK_JOB,
                "dataset": {"type": "directory", "path": dataset.name},
                "target": {"path": "target.yaml"},
                "camera_calibration": {"path": "cameras.yaml"},
                "cameras": [{
                    "id": "cam0",
                    "translation_base_camera_m": [0.1, 0.2, 0.3],
                }],
                "base_link": {
                    "frame_id": "base_link",
                    "position_scale_m": 0.01,
                    "rotation_left_correction": [
                        [1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, -1.0],
                    ],
                },
                "calibration": {"rank_ratio_min": 1e-4},
            }
            task_path = root / "task.yaml"
            dump_yaml(task, task_path)
            self.assertEqual(load_task(task_path)["job"], CAMERA_BASELINK_JOB)
            report = validate_task(task_path)
            self.assertEqual(report["status"], "passed", report)
            self.assertEqual(report["base_link"]["cameras"][0]["positions"], 3)
            task["base_link"]["rotation_left_correction"] = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0],
            ]
            dump_yaml(task, task_path)
            with self.assertRaisesRegex(
                    ValueError, "rotation_left_correction"):
                load_task(task_path)


if __name__ == "__main__":
    unittest.main()
