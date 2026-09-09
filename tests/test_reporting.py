import copy
import gzip
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_no_ros.evaluation import (
    ReportingError, assess_metrics, compute_metrics, prepare_evaluation_artifacts,
    stereo_geometry, validate_output_options,
)
from kalibr_no_ros.reporting import (
    evaluate_run, export_opencv, generate_report, load_archive, write_archive,
)
from kalibr_no_ros.version import SCHEMA_VERSION


def synthetic_stereo():
    K = np.array([[220., 0., 320.], [0., 225., 240.], [0., 0., 1.]])
    D = np.array([0.02, -0.005, 0.001, -0.0001])
    points = np.array([[-.2, -.15, 2.], [.2, -.15, 2.], [-.2, .15, 2.], [.2, .15, 2.]])
    transform = np.eye(4)
    transform[0, 3] = -.12
    cameras, results = [], []
    stamp = 1751234567890123456
    for index in range(2):
        pixels, _ = cv2.fisheye.projectPoints(points.reshape(-1, 1, 3), np.zeros(3),
                                             np.array([-.12 * index, 0., 0.]), K, D)
        corners = [{"corner_id": i + 40, "target_xyz_m": point.tolist(),
                    "measurement_px": pixel.tolist(), "prediction_px": pixel.tolist(),
                    "residual_px": [0., 0.], "used": True}
                   for i, (point, pixel) in enumerate(zip(points, pixels.reshape(-1, 2)))]
        result = {"id": "cam{}".format(index), "camera_model": "pinhole",
                  "distortion_model": "equidistant", "intrinsics": [220., 225., 320., 240.],
                  "distortion_coeffs": D.tolist(), "resolution": [640, 480]}
        if index:
            result["T_cn_cnm1"] = transform.tolist()
        frame = {"frame_id": "cam{}:7".format(index), "source_index": 7,
                 "source_timestamp_ns": stamp + index * 1000, "record_timestamp_ns": stamp + index * 1000,
                 "observation_timestamp_ns": stamp + index * 1000,
                 "source_path": "/nonexistent/calibration/image.png", "detection_status": "succeeded",
                 "used": True, "view_id": "view0", "T_camera_target": np.eye(4).tolist(), "corners": corners}
        cameras.append(dict(result, model="pinhole-equi", topic="/cam{}/image_raw".format(index), frames=[frame]))
        results.append(result)
    artifacts = {"schema_version": SCHEMA_VERSION, "kind": "run_artifacts", "state": "completed",
                 "calibration_type": "cameras", "history_captured": False,
                 "residual_convention": "measurement - prediction", "cameras": cameras,
                 "views": [{"view_id": "view0", "frame_ids": ["cam0:7", "cam1:7"], "used": True}],
                 "imu_residuals": [], "events": []}
    calibration = {"schema_version": SCHEMA_VERSION, "kind": "calibration_result",
                   "calibration_type": "cameras", "cameras": results}
    return artifacts, calibration


class ReportingTest(unittest.TestCase):
    def test_time_varying_bias_statistics_and_full_series_are_preserved(self):
        artifacts, calibration = synthetic_stereo()
        stamp = artifacts["cameras"][0]["frames"][0]["source_timestamp_ns"]
        bias = {"imu_id": "imu0", "kind": "gyro", "units": "rad/s",
                "representation": "time_varying_spline", "sampling": "retained_imu_residual_timestamps",
                "samples": [
                    {"timestamp_ns": stamp, "source_index": 7, "solver_timestamp_s": 10., "value": [1., 2., 3.]},
                    {"timestamp_ns": stamp + 1000000, "source_index": 8, "solver_timestamp_s": 10.001, "value": [3., 4., 5.]},
                ]}
        artifacts["imu_biases"] = [bias]
        artifacts["optimizer"] = {"scope": "last_attempt", "JFinal": 123., "stop_reason": "unavailable"}
        artifacts["objective"] = {"scope": "final_retained_camera_views", "final_camera_weighted_residual_sum": 2.}
        metrics = compute_metrics(artifacts, calibration)
        summary = metrics["imus"]["imu0"]["gyro"]["bias_spline"]
        self.assertEqual(summary["representation"], "time_varying_spline")
        self.assertEqual(summary["axes"]["x"]["mean"], 2.)
        self.assertEqual(summary["axes"]["x"]["std"], 1.)
        self.assertEqual(summary["axes"]["z"]["max"], 5.)
        self.assertEqual(summary["time_series"], bias["samples"])
        self.assertEqual(metrics["optimizer"]["scope"], "last_attempt")
        self.assertEqual(metrics["objective"]["final_camera_weighted_residual_sum"], 2.)
        with tempfile.TemporaryDirectory() as temporary:
            files = write_archive(artifacts, temporary)
            self.assertIn("observations/imu_biases.csv.gz", files)
            restored = load_archive(temporary)
            self.assertEqual(restored["imu_biases"], artifacts["imu_biases"])
            result = generate_report(artifacts, calibration, Path(temporary) / "report", {"export_opencv": False})
            text = (Path(temporary) / "report/results.txt").read_text()
            self.assertIn("Bias: time-varying spline", text)
            self.assertIn("scope: last_attempt", text)
            self.assertIn("scope: final_retained_camera_views", text)
            self.assertIn("imu0 gyro time-varying bias spline", (Path(temporary) / "report/report.html").read_text())
            self.assertEqual(result["metrics"]["imus"]["imu0"]["gyro"]["bias_spline"], summary)

    def test_missing_bias_samples_do_not_appear_as_constant_zero(self):
        artifacts, calibration = synthetic_stereo()
        artifacts["imu_biases"] = [{"imu_id": "imu0", "kind": "accel", "units": "m/s^2",
                                   "representation": "time_varying_spline", "samples": [
                                       {"timestamp_ns": 123, "source_index": 0, "solver_timestamp_s": 1.,
                                        "value": None, "unavailable_reason": "outside spline domain"},
                                       {"timestamp_ns": 456, "source_index": 1, "solver_timestamp_s": 2.,
                                        "value": [1., 2., 3.]},
                                   ]}]
        metrics = compute_metrics(artifacts, calibration)
        summary = metrics["imus"]["imu0"]["accel"]["bias_spline"]
        self.assertEqual(summary["status"], "incomplete")
        self.assertEqual(summary["count"], 1)
        self.assertEqual(summary["missing_value_count"], 1)
        self.assertIsNone(summary["time_series"][0]["value"])
        self.assertEqual(summary["axes"]["x"]["mean"], 1.)
        options = {"assessment": {"rules": [{"metric": "imus.imu0.accel.bias_spline.axes.x.mean", "max": 10.}]}}
        self.assertEqual(assess_metrics(metrics, options)["status"], "incomplete")

    def test_rms_is_two_dimensional_and_only_final_corners(self):
        artifacts, calibration = synthetic_stereo()
        frame = artifacts["cameras"][0]["frames"][0]
        for corner in frame["corners"]:
            corner["prediction_px"] = (np.array(corner["measurement_px"]) - [3., 4.]).tolist()
        frame["corners"].append({"corner_id": 400, "residual_px": [300., 400.], "used": False})
        unused = copy.deepcopy(frame)
        unused.update(frame_id="cam0:8", source_index=8, used=False)
        artifacts["cameras"][0]["frames"].append(unused)
        metrics = compute_metrics(artifacts, calibration)
        self.assertEqual(metrics["cameras"]["cam0"]["reprojection"]["rms_px"], 5.)
        self.assertEqual(metrics["cameras"]["cam0"]["reprojection"]["count"], 4)
        self.assertEqual(metrics["cameras"]["cam0"]["used_frames"], 1)
        self.assertEqual(metrics["cameras"]["cam0"]["detected_frames"], 2)

    def test_fisheye_rectification_uses_common_ids_and_source_resolution(self):
        artifacts, calibration = synthetic_stereo()
        artifacts["cameras"][1]["frames"][0]["corners"].reverse()
        metrics = compute_metrics(artifacts, calibration)
        pair = metrics["stereo_pairs"]["cam0_cam1"]
        self.assertEqual(pair["alignment"]["count"], 4)
        self.assertLess(pair["alignment"]["mean_abs_px"], 1e-9)
        self.assertEqual(pair["rectification"]["size"], [640, 480])
        self.assertEqual(pair["pairs"][0]["timestamp_difference_ns"], 1000)
        self.assertAlmostEqual(pair["baseline_m"], .12)
        self.assertEqual(pair["disparity_axis"], "x")

    def test_absent_residual_is_incomplete_not_zero(self):
        artifacts, calibration = synthetic_stereo()
        for corner in artifacts["cameras"][0]["frames"][0]["corners"]:
            corner.update(prediction_px=None, residual_px=None)
        metrics = compute_metrics(artifacts, calibration)
        self.assertIsNone(metrics["cameras"]["cam0"]["reprojection"]["rms_px"])
        options = {"assessment": {"rules": [{"metric": "cameras.cam0.reprojection.rms_px", "max": 1.}]}}
        assessment = assess_metrics(metrics, options)
        self.assertEqual(assessment["status"], "incomplete")
        self.assertEqual(assessment["reference_grading"]["status"], "incomplete")

    def test_inclusive_acceptance_and_reference_grading_are_separate(self):
        artifacts, calibration = synthetic_stereo()
        metrics = compute_metrics(artifacts, calibration)
        self.assertEqual(assess_metrics(metrics)["status"], "not_evaluated")
        metrics["cameras"]["cam0"]["reprojection"]["rms_px"] = 1.
        rules = [{"metric": "cameras.cam0.reprojection.rms_px", "max": 1.}]
        assessment = assess_metrics(metrics, {"assessment": {"rules": rules}})
        self.assertEqual(assessment["status"], "pass")
        self.assertEqual(assessment["reference_grading"]["status"], "poor")
        metrics["cameras"]["cam0"]["reprojection"]["rms_px"] = 1.001
        rules.append({"metric": "imus.imu0.gyro.rms", "max": 1.})
        self.assertEqual(assess_metrics(metrics, {"assessment": {"rules": rules}})["status"], "fail")

    def test_unused_detected_frame_is_counted_without_history_corners(self):
        artifacts, calibration = synthetic_stereo()
        frame = copy.deepcopy(artifacts["cameras"][0]["frames"][0])
        frame.update(frame_id="cam0:8", source_index=8, used=False, corners=[])
        artifacts["cameras"][0]["frames"].append(frame)
        self.assertEqual(compute_metrics(artifacts, calibration)["cameras"]["cam0"]["detected_frames"], 2)

    def test_imu_residuals_have_physical_units_and_vector_rms(self):
        artifacts, calibration = synthetic_stereo()
        artifacts["imu_residuals"] = [
            {"imu_id": "imu0", "kind": "gyro", "residual": [3., 4., 0.]},
            {"imu_id": "imu0", "kind": "accel", "residual": [0., 0., 2.]},
        ]
        metrics = compute_metrics(artifacts, calibration)
        self.assertEqual(metrics["imus"]["imu0"]["gyro"]["rms"], 5.)
        self.assertEqual(metrics["imus"]["imu0"]["gyro"]["unit"], "rad/s")
        self.assertEqual(metrics["imus"]["imu0"]["accel"]["unit"], "m/s^2")

    def test_normalized_residuals_use_exported_information_only(self):
        artifacts, calibration = synthetic_stereo()
        camera = artifacts["cameras"][0]
        camera.update(source_frame_count=1000, selected_frame_count=1)
        for corner in camera["frames"][0]["corners"]:
            corner.update(whitened_residual=[3., 4.], robust_weight=.5, weighted_squared_error=12.5)
        artifacts["imu_residuals"] = [{"imu_id": "imu0", "kind": "gyro", "residual": [0., 0., 1.],
                                     "whitened_residual": [0., 0., 10.]}]
        metrics = compute_metrics(artifacts, calibration)
        camera = metrics["cameras"]["cam0"]
        self.assertEqual(camera["input_frames"], 1000)
        self.assertEqual(camera["selected_frames"], 1)
        self.assertEqual(camera["normalized_reprojection"]["rms"], 5.)
        self.assertEqual(camera["robust_objective"]["total_weighted_squared_error"], 50.)
        self.assertEqual(metrics["cameras"]["cam1"]["normalized_reprojection"]["status"], "unavailable")
        self.assertEqual(metrics["imus"]["imu0"]["gyro"]["normalized_residual"]["rms"], 10.)
        self.assertEqual(metrics["imus"]["imu0"]["gyro"]["rms"], 1.)

    def test_camera_imu_diagnostic_pairing_is_one_to_one_and_bounded(self):
        artifacts, calibration = synthetic_stereo()
        artifacts.update(calibration_type="camera_imu", views=[])
        duplicate = copy.deepcopy(artifacts["cameras"][0]["frames"][0])
        duplicate.update(frame_id="cam0:8", source_index=8)
        artifacts["cameras"][0]["frames"].append(duplicate)
        options = validate_output_options()
        prepared = prepare_evaluation_artifacts(artifacts, options)
        self.assertEqual(len(prepared["evaluation_views"]), 1)
        self.assertEqual(prepared["evaluation_views"][0]["frame_ids"], ["cam0:7", "cam1:7"])
        metrics = compute_metrics(artifacts, calibration)
        self.assertEqual(metrics["stereo_pairs"]["cam0_cam1"]["pairing"], "camera_imu_diagnostic_pairing")
        artifacts["cameras"][1]["frames"][0]["source_timestamp_ns"] += 300000
        self.assertFalse(prepare_evaluation_artifacts(artifacts, options)["evaluation_views"])

    def test_output_options_reject_typos_and_invalid_ranges(self):
        invalid = [
            {"unexpected": True}, {"export_opencv": 1}, {"archive_selection_history": True},
            {"evaluation_pairing_tolerance_s": -1}, {"rectification": {"balance": 1.1}},
            {"rectification": {"fov_scale": 0}}, {"rectification": {"size": [640, True]}},
            {"visualizations": {"max_pairs": 0}}, {"visualizations": {"sampling": "random"}},
            {"assessment": {"rules": [{"metric": "cameras.cam0.rms", "max": float("nan")}] }},
            {"assessment": {"rules": [{"metric": "cameras.cam0.rms", "min": 2, "max": 1}] }},
        ]
        for options in invalid:
            with self.subTest(options=options), self.assertRaises(ReportingError):
                validate_output_options(options)

    def test_archive_roundtrip_preserves_ns_corners_and_optional_events(self):
        artifacts, _ = synthetic_stereo()
        artifacts["history_captured"] = True
        artifacts["events"] = [{"kind": "corner_removed", "corner_id": 123}]
        with tempfile.TemporaryDirectory() as temporary:
            files = write_archive(artifacts, temporary)
            self.assertIn("observations/selection_events.csv.gz", files)
            loaded = load_archive(temporary)
            original = artifacts["cameras"][0]["frames"][0]
            restored = loaded["cameras"][0]["frames"][0]
            self.assertEqual(restored["source_timestamp_ns"], original["source_timestamp_ns"])
            self.assertIs(type(restored["source_timestamp_ns"]), int)
            self.assertEqual(restored["corners"], original["corners"])
            self.assertEqual(loaded["events"], artifacts["events"])
            self.assertEqual(loaded["views"], artifacts["views"])
        artifacts["history_captured"] = False
        with tempfile.TemporaryDirectory() as temporary:
            files = write_archive(artifacts, temporary)
            self.assertNotIn("observations/selection_events.csv.gz", files)

    def test_archive_checksum_and_path_escape_are_rejected(self):
        artifacts, _ = synthetic_stereo()
        with tempfile.TemporaryDirectory() as temporary:
            write_archive(artifacts, temporary)
            corners = Path(temporary) / "observations/corners.csv.gz"
            corners.write_bytes(gzip.compress(b"modified"))
            with self.assertRaisesRegex(ReportingError, "checksum"):
                load_archive(temporary)
            manifest_path = Path(temporary) / "observations/manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["tables"]["frames.csv.gz"]["path"] = "../../outside.gz"
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ReportingError, "contained"):
                load_archive(temporary)

    def test_opencv_fisheye_export_roundtrip_and_transform_direction(self):
        artifacts, calibration = synthetic_stereo()
        with tempfile.TemporaryDirectory() as temporary:
            files, statuses = export_opencv(artifacts, calibration, temporary)
            self.assertEqual(len(files), 3)
            self.assertTrue(all(row["status"] == "written" for row in statuses))
            storage = cv2.FileStorage(str(Path(temporary) / "opencv/cam0_cam1.yaml"), cv2.FILE_STORAGE_READ)
            try:
                R, T = storage.getNode("R").mat(), storage.getNode("T").mat().reshape(3)
                self.assertEqual(storage.getNode("schema_version").string(), SCHEMA_VERSION)
                np.testing.assert_allclose(R @ np.zeros(3) + T, [-.12, 0., 0.])
                np.testing.assert_allclose(R.T @ (-T), [.12, 0., 0.])
                self.assertEqual(storage.getNode("D1").mat().size, 4)
                geometry = stereo_geometry(artifacts["cameras"][0], artifacts["cameras"][1], validate_output_options()["rectification"])
                np.testing.assert_allclose(storage.getNode("Q").mat(), geometry["Q"])
            finally:
                storage.release()

    def test_generate_and_offline_evaluate_without_original_images(self):
        artifacts, calibration = synthetic_stereo()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "original"
            output.mkdir()
            (output / "calibration.yaml").write_text(yaml.safe_dump(calibration))
            original_yaml = (output / "calibration.yaml").read_bytes()
            result = generate_report(artifacts, calibration, output, {"archive_observations": True})
            self.assertEqual(result["assessment"]["status"], "not_evaluated")
            self.assertEqual((output / "calibration.yaml").read_bytes(), original_yaml)
            self.assertTrue((output / "report.pdf").read_bytes().startswith(b"%PDF"))
            self.assertIn("2D corner RMS", (output / "report.html").read_text())
            evaluated = evaluate_run(output, Path(temporary) / "evaluated", {"export_opencv": False})
            self.assertEqual(evaluated["metrics"]["cameras"], result["metrics"]["cameras"])
            self.assertEqual(evaluated["metrics"]["stereo_pairs"], result["metrics"]["stereo_pairs"])
            self.assertFalse((output / "images").exists())
            self.assertFalse((output / "observations/selection_events.csv.gz").exists())

    def test_summary_only_evaluation_cannot_invent_corner_evidence(self):
        artifacts, calibration = synthetic_stereo()
        with tempfile.TemporaryDirectory() as temporary:
            original = Path(temporary) / "source"
            result = generate_report(artifacts, calibration, original, {"export_opencv": False})
            assessment = {"assessment": {"rules": [{"metric": "cameras.cam0.reprojection.rms_px", "max": .5}]}}
            evaluated = evaluate_run(original, Path(temporary) / "output", assessment)
            self.assertEqual(evaluated["assessment"]["status"], "pass")
            self.assertEqual(evaluated["metrics"]["evaluation_mode"], "saved_summary_only")
            self.assertEqual(evaluated["metrics"]["cameras"], result["metrics"]["cameras"])
            with self.assertRaisesRegex(ReportingError, "requires an observation archive"):
                evaluate_run(original, Path(temporary) / "invalid", {"visualizations": {"enabled": True}})
            with self.assertRaisesRegex(ReportingError, "separate"):
                evaluate_run(original, original)

    def test_missing_images_are_diagnostic_not_fake_numeric_failures(self):
        artifacts, calibration = synthetic_stereo()
        with tempfile.TemporaryDirectory() as temporary:
            result = generate_report(artifacts, calibration, temporary,
                                     {"export_opencv": False, "visualizations": {"enabled": True}})
            self.assertTrue(result["metrics"]["output_diagnostics"])
            self.assertEqual(result["metrics"]["cameras"]["cam0"]["reprojection"]["rms_px"], 0.)
            self.assertFalse(any(path.endswith(".jpg") for path in result["files"]))

    def test_frozen_reference_without_hooks_has_unavailable_statistics(self):
        _, calibration = synthetic_stereo()
        metrics = compute_metrics({"calibration_type": "cameras", "state": "completed"}, calibration)
        self.assertEqual(metrics["cameras"]["cam0"]["reprojection"]["status"], "unavailable")
        self.assertIsNone(metrics["cameras"]["cam0"]["reprojection"]["rms_px"])
        self.assertEqual(assess_metrics(metrics)["reference_grading"]["status"], "incomplete")
        assessment = assess_metrics(metrics, {"assessment": {"rules": [
            {"metric": "stereo_pairs.cam0_cam1.baseline_m", "min": .1, "max": .2}]}})
        self.assertEqual(assessment["status"], "pass")

    def test_copied_images_support_portable_offline_visualization(self):
        artifacts, calibration = synthetic_stereo()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            image = root / "original.png"
            self.assertTrue(cv2.imwrite(str(image), np.full((480, 640, 3), 120, dtype=np.uint8)))
            for camera in artifacts["cameras"]:
                camera["frames"][0]["source_path"] = str(image)
            original = root / "run"
            original.mkdir()
            (original / "calibration.yaml").write_text(yaml.safe_dump(calibration))
            result = generate_report(artifacts, calibration, original,
                                     {"archive_observations": True, "copy_used_images": True,
                                      "visualizations": {"enabled": True}, "export_opencv": False})
            self.assertIn("images/cam0/7.png", result["files"])
            self.assertIn("visualizations/cam0_cam1/rectified_0000.jpg", result["files"])
            self.assertNotIn("copied_image", artifacts["cameras"][0]["frames"][0])
            image.unlink()
            evaluated = evaluate_run(original, root / "evaluated",
                                     {"visualizations": {"enabled": True}, "export_opencv": False})
            self.assertIn("visualizations/cam0/corners_7.jpg", evaluated["files"])
            self.assertFalse(evaluated["metrics"]["output_diagnostics"])

    def test_offline_dataset_override_checks_exact_frame_timestamp(self):
        artifacts, calibration = synthetic_stereo()
        for camera in artifacts["cameras"]:
            camera["frames"][0]["dataset_id"] = "test_capture_260909_1200"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            original = root / "run"
            original.mkdir()
            (original / "calibration.yaml").write_text(yaml.safe_dump(calibration))
            generate_report(artifacts, calibration, original,
                            {"archive_observations": True, "export_opencv": False})
            dataset = root / "moved_dataset"
            dataset.mkdir()
            manifest = {"schema_version": SCHEMA_VERSION, "type": "kalibr_directory_dataset",
                        "dataset_id": "test_capture_260909_1200", "cameras": []}
            for camera in artifacts["cameras"]:
                directory = dataset / camera["id"]
                directory.mkdir()
                rows = ["timestamp_ns,filename"]
                stamp = camera["frames"][0]["source_timestamp_ns"]
                for index in range(8):
                    filename = "{}.png".format(index)
                    cv2.imwrite(str(directory / filename), np.zeros((480, 640, 3), dtype=np.uint8))
                    rows.append("{},{}".format(stamp + (index - 7) * 1000, filename))
                (directory / "timestamps.csv").write_text("\n".join(rows) + "\n")
                manifest["cameras"].append({"id": camera["id"], "images": camera["id"],
                                             "timestamps": camera["id"] + "/timestamps.csv"})
            (dataset / "dataset.yaml").write_text(yaml.safe_dump(manifest))
            result = evaluate_run(original, root / "evaluated", {"export_opencv": False,
                                  "visualizations": {"enabled": True}}, dataset=dataset)
            self.assertIn("visualizations/cam0/corners_7.jpg", result["files"])
            self.assertFalse(result["metrics"]["output_diagnostics"])
            manifest["dataset_id"] = "wrong_device_260909_1200"
            (dataset / "dataset.yaml").write_text(yaml.safe_dump(manifest))
            mismatched = evaluate_run(original, root / "wrong_device", {"export_opencv": False,
                                     "visualizations": {"enabled": True}}, dataset=dataset)
            self.assertTrue(any("dataset_id differs" in row["reason"] for row in mismatched["metrics"]["output_diagnostics"]))
            self.assertFalse(any(path.endswith(".jpg") for path in mismatched["files"]))


if __name__ == "__main__":
    unittest.main()
