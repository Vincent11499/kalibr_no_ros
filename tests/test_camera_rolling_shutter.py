"""Pure-Python contracts for the visual rolling-shutter camera task."""

import copy
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from kalibr_no_ros.cli import build_parser
from kalibr_no_ros.delivery import enrich_result
from kalibr_no_ros.evaluation import compute_metrics
from kalibr_no_ros.initialization import validate_initialization
from kalibr_no_ros.rolling_shutter import CAMERA_RS_JOB
from kalibr_no_ros.task import (
    TaskError,
    _legacy_camchain,
    _native_rs_camera_arguments,
    _normalize_camera_rs_result,
    _overlay_structured_camera_snapshots,
    _rs_camera_arguments,
    dump_yaml,
    load_task,
    load_yaml,
)
from kalibr_no_ros.validation import _validate_shutter_result, load_cameras


def _task(camera_count=2):
    camera_ids = ["left", "right", "rear"][:camera_count]
    return {
        "schema_version": "1.0.0",
        "kind": "calibration_task",
        "job": CAMERA_RS_JOB,
        "dataset": {"type": "bag", "path": "input.bag"},
        "target": {"path": "aprilgrid.yaml"},
        "cameras": [
            {"id": camera_id, "topic": "/{}".format(camera_id),
             "model": "pinhole-equi"}
            for camera_id in camera_ids
        ],
        "rolling_shutter": {
            camera_id: {
                "line_delay_s": (index + 1) * 1e-6,
                "estimate": True,
                "max_abs_line_delay_s": 2e-5,
            }
            for index, camera_id in enumerate(camera_ids)
        },
        "calibration": {},
    }


def _write_task(root, document, name="task.yaml"):
    path = Path(root) / name
    dump_yaml(document, path)
    return path


class VisualRollingShutterContractTest(unittest.TestCase):
    def test_persisted_visual_shutter_metadata_is_strict_and_consistent(self):
        shutter = {
            "type": "rolling_shutter",
            "line_delay_s": 8e-6,
            "reference_row_px": 0.0,
            "first_to_last_row_span_s": 479 * 8e-6,
            "estimated": True,
            "timestamp_reference": "row0_exposure_end",
            "corner_time_equation": (
                "t_corner_s = t_camera_timestamp_s + y_px * line_delay_s"),
            "max_abs_line_delay_s": 2e-5,
            "distance_to_bound_s": 1.2e-5,
            "bound_role": "optimizer_parameterization",
        }
        _validate_shutter_result(shutter, [640, 480], "cameras")

        invalid = []
        for field, value in (
                ("estimated", 1),
                ("reference_row_px", 239.5),
                ("timestamp_reference", "frame_end"),
                ("first_to_last_row_span_s", 0.1),
                ("max_abs_line_delay_s", 4e-6),
                ("distance_to_bound_s", 1e-6)):
            candidate = dict(shutter)
            candidate[field] = value
            invalid.append((candidate, field))
        unknown = dict(shutter)
        unknown["unexpected"] = True
        invalid.append((unknown, "unknown fields"))
        for candidate, message in invalid:
            with self.subTest(message=message), self.assertRaisesRegex(
                    TaskError, message):
                _validate_shutter_result(candidate, [640, 480], "cameras")

        camera_imu = {
            "type": "rolling_shutter",
            "line_delay_s": 8e-6,
            "reference_row_px": 239.5,
            "first_to_last_row_span_s": 479 * 8e-6,
            "estimated": True,
            "time_reference": (
                "t_row_imu = t_frame + timeshift_cam_imu + "
                "(y - reference_row_px) * line_delay_s"),
            "max_abs_line_delay_s": 2e-5,
            "line_delay_std_s": 1e-7,
        }
        _validate_shutter_result(camera_imu, [640, 480], "camera_imu")

    def test_cli_has_system_and_native_comparison_commands(self):
        for command in ("cameras-rs", "native-rs-cameras"):
            parsed = build_parser().parse_args([
                "calibrate", command, "--config", "task.yaml",
                "--output-dir", "result",
            ])
            self.assertEqual(parsed.group, "calibrate")
            self.assertEqual(parsed.calibration, command)

    def test_job_is_strict_about_models_and_camera_only_parameters(self):
        invalid = []
        unsupported_model = _task()
        unsupported_model["cameras"][1]["model"] = "pinhole-radtan8"
        invalid.append((unsupported_model, "supports pinhole-equi and pinhole-radtan"))
        ordinary_only_parameter = _task()
        ordinary_only_parameter["calibration"]["qr_tolerance"] = 1e-8
        invalid.append((ordinary_only_parameter, "unknown fields"))
        unknown_parameter = _task()
        unknown_parameter["calibration"]["row_reference_px"] = 1080.0
        invalid.append((unknown_parameter, "unknown fields"))
        missing_bound = _task()
        missing_bound["rolling_shutter"]["left"].pop(
            "max_abs_line_delay_s")
        invalid.append((missing_bound, "max_abs_line_delay_s"))
        insufficient_spline_order = _task()
        insufficient_spline_order["calibration"]["spline_order"] = 2
        invalid.append((
            insufficient_spline_order,
            "spline_order must be at least 3"))

        with tempfile.TemporaryDirectory() as directory:
            for index, (document, message) in enumerate(invalid):
                with self.subTest(index=index), self.assertRaisesRegex(
                        TaskError, message):
                    load_task(_write_task(
                        directory, document, "invalid-{}.yaml".format(index)))

    def test_system_arguments_preserve_camera_order_ids_and_parameters(self):
        document = _task(3)
        document["calibration"].update({
            "synchronization_tolerance_s": 0.0001,
            "spline_order": 5,
            "knots_per_second": 1.25,
            "feature_sigma_px": 0.75,
        })
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            task = load_task(_write_task(root, document))
            arguments = _rs_camera_arguments(
                task, root / "dataset.bag", root / "target.yaml", root, {})
            topics = arguments.index("--topics")
            models = arguments.index("--models")
            self.assertEqual(
                arguments[topics + 1:models], ["/left", "/right", "/rear"])
            shutter_path = Path(arguments[
                arguments.index("--rolling-shutter-config") + 1])
            from kalibr_no_ros.task import load_yaml
            shutter = load_yaml(shutter_path)
            self.assertEqual(list(shutter["cameras"]), ["cam0", "cam1", "cam2"])
            self.assertEqual(shutter["cameras"]["cam2"]["line_delay_s"], 3e-6)
            self.assertEqual(arguments[
                arguments.index("--spline-order") + 1], "5")
            self.assertEqual(arguments[
                arguments.index("--knots-per-second") + 1], "1.25")

    def test_native_adapter_rejects_multicamera_and_system_only_controls(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            task = load_task(_write_task(root, _task(2), "multi.yaml"))
            with self.assertRaisesRegex(TaskError, "exactly one camera"):
                _native_rs_camera_arguments(
                    task, root / "input.bag", root / "target.yaml", root, {})

            mono = _task(1)
            mono["calibration"]["spline_order"] = 4
            task = load_task(_write_task(root, mono, "mono.yaml"))
            with self.assertRaisesRegex(TaskError, "does not implement.*spline_order"):
                _native_rs_camera_arguments(
                    task, root / "input.bag", root / "target.yaml", root, {})

            export = _task(1)
            export["output"] = {"export_poses": True}
            task = load_task(_write_task(root, export, "export.yaml"))
            with self.assertRaisesRegex(
                    TaskError,
                    r"does not support output\.export_poses.*use cameras-rs"):
                _native_rs_camera_arguments(
                    task, root / "input.bag", root / "target.yaml", root, {})

    def test_camera_initialization_is_shared_and_keeps_three_camera_chain(self):
        task = _task(3)
        identity = np.eye(4).tolist()
        first = np.eye(4)
        first[0, 3] = -0.3
        second = np.eye(4)
        second[1, 3] = 0.2
        initialization = {
            "schema_version": "1.0.0",
            "kind": "camera_calibration_initialization",
            "cameras": {
                "left": {
                    "intrinsics": [400.0, 401.0, 320.0, 240.0],
                    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                },
                "right": {
                    "intrinsics": [402.0, 403.0, 320.0, 240.0],
                    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                    "T_cn_cnm1": first.tolist(),
                },
                "rear": {
                    "intrinsics": [404.0, 405.0, 320.0, 240.0],
                    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                    "T_cn_cnm1": second.tolist(),
                },
            },
        }
        canonical = validate_initialization(
            initialization, CAMERA_RS_JOB, task, strategy="direct")
        self.assertEqual(list(canonical["cameras"]), ["cam0", "cam1", "cam2"])
        self.assertNotIn("T_cam_from_previous", canonical["cameras"]["cam0"])
        self.assertEqual(
            canonical["cameras"]["cam1"]["T_cam_from_previous"],
            first.tolist())
        self.assertEqual(
            canonical["cameras"]["cam2"]["T_cam_from_previous"],
            second.tolist())
        self.assertEqual(identity[3], canonical["cameras"]["cam2"]
                         ["T_cam_from_previous"][3])

    def test_rs_camera_result_is_readable_without_activating_rs_in_ordinary_imu_job(self):
        camera = {
            "id": "cam0",
            "camera_model": "pinhole",
            "distortion_model": "equidistant",
            "intrinsics": [400.0, 401.0, 320.0, 240.0],
            "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
            "resolution": [640, 480],
            "rostopic": "/cam0",
            "rms": 0.25,
            "line_delay": 8e-6,
            "rs_compensated_pair_residual": 0.2,
            "shutter": {
                "type": "rolling_shutter",
                "line_delay_s": 8e-6,
                "reference_row_px": 0.0,
                "first_to_last_row_span_s": 479 * 8e-6,
                "estimated": True,
                "timestamp_reference": "row0_exposure_end",
            },
        }
        result = {
            "schema_version": "1.0.0",
            "kind": "calibration_result",
            "calibration_type": "cameras",
            "cameras": [camera],
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            dump_yaml(result, root / "rs-camera.yaml")
            task = {
                "job": "camera_imu_calibration",
                "dataset": {"type": "bag"},
                "camera_calibration": {"path": "rs-camera.yaml"},
                "_config_dir": str(root),
            }
            loaded = load_cameras(task)
            self.assertEqual(loaded[0]["shutter"]["line_delay_s"], 8e-6)

            task["job"] = "camera_imu_rolling_shutter_calibration"
            self.assertEqual(
                load_cameras(task)[0]["shutter"]["timestamp_reference"],
                "row0_exposure_end")

            stage = root / "stage"
            stage.mkdir()
            legacy = load_yaml(_legacy_camchain(task, stage))
            self.assertNotIn("shutter", legacy["cam0"])
            self.assertNotIn("line_delay", legacy["cam0"])
            self.assertNotIn("rs_compensated_pair_residual", legacy["cam0"])

    def test_formal_shutter_output_uses_row_zero_and_preserves_solver_metadata(self):
        task = _task(1)
        result = {
            "cameras": [{
                "id": "cam0", "resolution": [640, 480],
                "line_delay": 8e-6,
            }],
        }
        normalized = _normalize_camera_rs_result(result, task, "system")
        shutter = normalized["cameras"][0]["shutter"]
        self.assertNotIn("line_delay", normalized["cameras"][0])
        self.assertEqual(shutter["reference_row_px"], 0.0)
        self.assertEqual(shutter["timestamp_reference"], "row0_exposure_end")
        self.assertAlmostEqual(shutter["first_to_last_row_span_s"], 479 * 8e-6)
        self.assertEqual(shutter["bound_role"], "optimizer_parameterization")

        artifacts = {"cameras": [{
            "intrinsics": [410.0, 411.0, 320.0, 240.0],
            "distortion_coeffs": [0.1, 0.0, 0.0, 0.0],
            "resolution": [640, 480],
            "shutter": {"line_delay_s": 7.5e-6,
                        "distance_to_bound_s": 12.5e-6},
        }]}
        _overlay_structured_camera_snapshots(normalized, artifacts)
        shutter = normalized["cameras"][0]["shutter"]
        self.assertEqual(shutter["line_delay_s"], 7.5e-6)
        self.assertEqual(shutter["reference_row_px"], 0.0)
        self.assertAlmostEqual(
            shutter["first_to_last_row_span_s"], 479 * 7.5e-6)
        self.assertEqual(shutter["bound_role"], "optimizer_parameterization")
        self.assertEqual(shutter["distance_to_bound_s"], 12.5e-6)
        self.assertEqual(normalized["cameras"][0]["intrinsics"][0], 410.0)

        with self.assertRaisesRegex(TaskError, "camera count"):
            _normalize_camera_rs_result({"cameras": []}, task, "system")

    def test_rs_compensated_metric_subtracts_fitted_pair_motion(self):
        transform = np.eye(4)
        transform[0, 3] = -0.12
        result_cameras = []
        artifact_cameras = []
        for index, (measurement, prediction) in enumerate((
                ([10.0, 20.0], [9.0, 18.0]),
                ([15.0, 24.0], [14.0, 22.0]))):
            result = {
                "id": "cam{}".format(index),
                "camera_model": "pinhole",
                "distortion_model": "equidistant",
                "intrinsics": [300.0, 300.0, 320.0, 240.0],
                "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                "resolution": [640, 480],
                "shutter": {"type": "rolling_shutter", "line_delay_s": 8e-6},
            }
            if index:
                result["T_cn_cnm1"] = transform.tolist()
            frame = {
                "frame_id": "cam{}:0".format(index), "used": True,
                "source_index": 0, "source_timestamp_ns": index,
                "corners": [{
                    "corner_id": 7, "used": True,
                    "measurement_px": measurement,
                    "prediction_px": prediction,
                    "residual_px": (np.asarray(measurement)
                                    - np.asarray(prediction)).tolist(),
                }],
            }
            result_cameras.append(result)
            artifact_cameras.append(dict(
                result, model="pinhole-equi", frames=[frame]))
        artifacts = {
            "calibration_type": "cameras",
            "cameras": artifact_cameras,
            "views": [{"view_id": "view0", "used": True,
                       "frame_ids": ["cam0:0", "cam1:0"]}],
        }
        calibration = {
            "calibration_type": "cameras", "cameras": result_cameras}
        fake_geometry = {
            "baseline_m": 0.12, "T": transform[:3, 3],
            "disparity_axis": "x", "size": (640, 480),
        }
        with mock.patch(
                "kalibr_no_ros.evaluation.stereo_geometry",
                return_value=fake_geometry), mock.patch(
                    "kalibr_no_ros.evaluation.rectified_points",
                    side_effect=lambda points, unused_geometry, unused_side:
                    np.asarray(points, dtype=float)):
            pair = compute_metrics(artifacts, calibration)[
                "stereo_pairs"]["cam0_cam1"]
        self.assertEqual(pair["alignment"]["mean_abs_px"], 4.0)
        self.assertEqual(
            pair["rs_compensated_pair_residual"]["mean_abs_px"], 0.0)
        self.assertTrue(pair["rs_compensated_pair_residual"]["fit_dependent"])
        self.assertFalse(
            pair["rs_compensated_pair_residual"]["independent_validation"])

    def test_indirect_three_camera_graph_keeps_unavailable_adjacent_metrics(self):
        transform = np.eye(4)
        transform[0, 3] = -0.12

        def frame(camera_id, index, corner_id, measurement, prediction):
            return {
                "frame_id": "{}:{}".format(camera_id, index),
                "used": True,
                "source_index": index,
                "source_timestamp_ns": index,
                "corners": [{
                    "corner_id": corner_id,
                    "used": True,
                    "measurement_px": measurement,
                    "prediction_px": prediction,
                    "residual_px": (np.asarray(measurement)
                                    - np.asarray(prediction)).tolist(),
                }],
            }

        frames = {
            "cam0": [frame("cam0", 0, 7, [10.0, 20.0], [9.0, 18.0])],
            "cam1": [frame("cam1", 0, 8, [15.0, 24.0], [14.0, 22.0])],
            "cam2": [
                frame("cam2", 0, 7, [16.0, 23.0], [15.0, 21.0]),
                frame("cam2", 1, 8, [20.0, 28.0], [19.0, 26.0]),
            ],
        }
        result_cameras = []
        artifact_cameras = []
        for index, camera_id in enumerate(("cam0", "cam1", "cam2")):
            camera = {
                "id": camera_id,
                "camera_model": "pinhole",
                "distortion_model": "equidistant",
                "intrinsics": [300.0, 300.0, 320.0, 240.0],
                "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                "resolution": [640, 480],
                "shutter": {"type": "rolling_shutter",
                            "line_delay_s": 8e-6},
            }
            if index:
                camera["T_cn_cnm1"] = transform.tolist()
            result_cameras.append(dict(camera))
            artifact_cameras.append(dict(
                camera, model="pinhole-equi", frames=frames[camera_id]))
        artifacts = {
            "calibration_type": "cameras",
            "cameras": artifact_cameras,
            # cam0 and cam1 are connected only through cam2.  The public chain
            # still contains cam0->cam1->cam2 transforms, while cam0_cam1 has
            # no direct final observation from which to compute alignment.
            "views": [
                {"view_id": "view02", "used": True,
                 "frame_ids": ["cam0:0", "cam2:0"]},
                {"view_id": "view12", "used": True,
                 "frame_ids": ["cam1:0", "cam2:1"]},
            ],
        }
        calibration = {
            "calibration_type": "cameras", "cameras": result_cameras}
        fake_geometry = {
            "baseline_m": 0.12, "T": transform[:3, 3],
            "disparity_axis": "x", "size": (640, 480),
        }
        with mock.patch(
                "kalibr_no_ros.evaluation.stereo_geometry",
                return_value=fake_geometry), mock.patch(
                    "kalibr_no_ros.evaluation.rectified_points",
                    side_effect=lambda points, unused_geometry, unused_side:
                    np.asarray(points, dtype=float)):
            metrics = compute_metrics(artifacts, calibration)

        missing = metrics["stereo_pairs"]["cam0_cam1"]
        self.assertEqual(missing["status"], "unavailable")
        self.assertIsNone(missing["alignment"]["rms_px"])
        self.assertIn(
            "no directly paired final views", missing["alignment"]["reason"])
        self.assertIsNone(
            missing["rs_compensated_pair_residual"]["rms_px"])
        self.assertIn(
            "no directly paired final views",
            missing["rs_compensated_pair_residual"]["reason"])

        invalid_metrics = copy.deepcopy(metrics)
        invalid_metrics["stereo_pairs"]["cam1_cam2"]["alignment"][
            "rms_px"] = float("nan")
        with self.assertRaisesRegex(TaskError, "nonnegative finite rms_px"):
            enrich_result(copy.deepcopy(calibration), invalid_metrics)

        enrich_result(calibration, metrics)
        unavailable_camera = calibration["cameras"][1]
        self.assertIsNone(unavailable_camera["alignment"])
        self.assertIsNone(
            unavailable_camera["rs_compensated_pair_residual"])
        self.assertEqual(calibration["cameras"][2]["alignment"], 4.0)
        self.assertEqual(
            calibration["cameras"][2]["rs_compensated_pair_residual"],
            0.0)


if __name__ == "__main__":
    unittest.main()
