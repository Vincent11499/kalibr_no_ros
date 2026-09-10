import os
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_no_ros.cli import _runtime_overrides, build_parser
from kalibr_no_ros.initialization import (
    InitializationError,
    build_initialization_report,
    canonical_initialization,
    validate_initialization,
)
from kalibr_no_ros.task import (
    TaskError,
    _camera_arguments,
    load_task,
    resolve_initialization,
    run_task,
)
import kalibr_no_ros.task as task_module
from kalibr_no_ros import artifacts as artifact_module
from kalibr_no_ros import reporting


IDENTITY4 = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
]


class InitializationValidationTest(unittest.TestCase):
    def test_chain_named_transform_is_converted_without_changing_direction(self):
        task = self.camera_task('pinhole-equi', 'pinhole-equi', 'pinhole-equi')
        transform = np.eye(4)
        transform[0, 3] = -0.38
        doc = {'schema_version': '1.0.0', 'kind': 'camera_calibration_initialization',
               'cameras': {'cam1': {'T_cn_cnm1': transform.tolist()},
                           'cam2': {'T_cn_cnm1': transform.tolist()}}}
        value = validate_initialization(doc, 'camera_calibration', task, strategy='refine')
        np.testing.assert_array_equal(value['cameras']['cam1']['T_cam_from_previous'], transform)
        np.testing.assert_array_equal(value['cameras']['cam2']['T_cam_from_previous'], transform)
        doc['cameras']['cam0'] = {'T_cn_cnm1': transform.tolist()}
        with self.assertRaises(InitializationError):
            validate_initialization(doc, 'camera_calibration', task, strategy='refine')

    def camera_task(self, *models):
        return {
            "job": "camera_calibration",
            "cameras": [
                {"topic": "/cam{}".format(index), "model": model}
                for index, model in enumerate(models)
            ],
        }

    def test_explicit_multicamera_edges_preserve_direction_and_task_id_order(self):
        task = self.camera_task("pinhole-equi", "pinhole-equi", "pinhole-equi")
        ids = ["right", "left", "rear"]
        for camera, identifier in zip(task["cameras"], ids):
            camera["id"] = identifier
        first = np.array([[0., -1., 0., -.1], [1., 0., 0., .02],
                          [0., 0., 1., .03], [0., 0., 0., 1.]])
        second = np.array([[1., 0., 0., .04], [0., 0., -1., -.2],
                           [0., 1., 0., .01], [0., 0., 0., 1.]])
        document = {"schema_version": "1.0.0", "kind": "camera_calibration_initialization",
                    "cameras": {identifier: {"intrinsics": [400. + index, 401., 320., 240.],
                                              "distortion_coeffs": [0.] * 4}
                                for index, identifier in reversed(list(enumerate(ids)))},
                    "extrinsics": [
                        {"from": "rear", "to": "left", "T": np.linalg.inv(second).tolist()},
                        {"from": "right", "to": "left", "T": first.tolist()},
                    ]}
        validated = validate_initialization(document, "camera_calibration", task, strategy="direct")
        self.assertEqual(list(validated["cameras"]), ["cam0", "cam1", "cam2"])
        self.assertEqual(validated["cameras"]["cam0"]["intrinsics"][0], 400.)
        actual_first = np.asarray(validated["cameras"]["cam1"]["T_cam_from_previous"])
        actual_second = np.asarray(validated["cameras"]["cam2"]["T_cam_from_previous"])
        np.testing.assert_array_equal(actual_first, first)
        np.testing.assert_allclose(actual_second, second, atol=1e-15, rtol=0.)
        origin = np.array([0., 0., 0., 1.])
        np.testing.assert_allclose(actual_second @ actual_first @ origin, second @ first @ origin)
        np.testing.assert_allclose(np.linalg.inv(actual_first) @ actual_first @ origin, origin)
        self.assertNotIn("extrinsics", canonical_initialization(validated, "direct"))
        report = build_initialization_report(validated, "direct", task, source_path="seed.yaml",
                                             source_sha256="abc", path_origin="task", strategy_origin="task")
        self.assertEqual(report["camera_id_mapping"], dict(zip(ids, ["cam0", "cam1", "cam2"])))
        self.assertEqual([stage["stage"] for stage in report["stage_decisions"][:3]],
                         ["single_camera_intrinsics." + identifier for identifier in ids])
        self.assertEqual(report["stage_decisions"][3]["supplied_transforms"], ["left", "rear"])

    def test_mono_named_cam1_seed_maps_to_first_native_camera(self):
        task = self.camera_task("pinhole-equi")
        task["cameras"][0]["id"] = "cam1"
        document = {"schema_version": "1.0.0", "kind": "camera_calibration_initialization",
                    "cameras": {"cam1": {"intrinsics": [400., 401., 320., 240.],
                                           "distortion_coeffs": [0.] * 4}}}
        self.assertEqual(list(validate_initialization(document, "camera_calibration", task,
                                                       strategy="direct")["cameras"]), ["cam0"])

    def test_refine_accepts_only_explicit_extrinsics(self):
        task = self.camera_task("pinhole-equi", "pinhole-equi")
        document = {"schema_version": "1.0.0", "kind": "camera_calibration_initialization",
                    "extrinsics": [{"from": "cam0", "to": "cam1", "T": IDENTITY4}]}
        validated = validate_initialization(document, "camera_calibration", task)
        self.assertEqual(validated["cameras"], {"cam1": {"T_cam_from_previous": IDENTITY4}})
        with self.assertRaisesRegex(InitializationError, "direct initialization requires"):
            validate_initialization(document, "camera_calibration", task, strategy="direct")

    def test_explicit_extrinsics_reject_ambiguous_or_invalid_edges(self):
        task = self.camera_task("pinhole-equi", "pinhole-equi", "pinhole-equi")
        edge = {"from": "cam0", "to": "cam1", "T": IDENTITY4}
        invalid = [
            ({"extrinsics": {}}, "extrinsics must be a list"),
            ({"extrinsics": [{"from": "cam0", "T": IDENTITY4}]}, "requires to"),
            ({"extrinsics": [dict(edge, to="missing")]}, "camera ID"),
            ({"extrinsics": [dict(edge, to="cam0")]}, "adjacent"),
            ({"extrinsics": [dict(edge, to="cam2")]}, "adjacent"),
            ({"extrinsics": [edge, dict(edge, **{"from": "cam1", "to": "cam0"})]}, "duplicate"),
            ({"extrinsics": [dict(edge, T=[[1.]])]}, "4x4"),
            ({"extrinsics": [dict(edge, unexpected=True)]}, "unknown fields"),
            ({"extrinsics": [edge], "cameras": {"cam1": {"T_cam_from_previous": IDENTITY4}}}, "conflicting"),
        ]
        for body, message in invalid:
            with self.subTest(body=body), self.assertRaisesRegex(InitializationError, message):
                validate_initialization(dict(schema_version="1.0.0", kind="camera_calibration_initialization", **body),
                                        "camera_calibration", task)

    def test_legacy_transform_and_named_camera_remain_supported(self):
        task = self.camera_task("pinhole-equi", "pinhole-equi")
        task["cameras"][0]["id"], task["cameras"][1]["id"] = "right", "left"
        document = {"schema_version": "1.0.0", "kind": "camera_calibration_initialization",
                    "cameras": {"left": {"T_cam_from_previous": IDENTITY4}}}
        self.assertEqual(validate_initialization(document, "camera_calibration", task)["cameras"]["cam1"],
                         {"T_cam_from_previous": IDENTITY4})
        document["cameras"] = {"right": {"T_cam_from_previous": IDENTITY4}}
        with self.assertRaisesRegex(InitializationError, "first task camera"):
            validate_initialization(document, "camera_calibration", task)

    def test_camera_imu_time_seeds_use_real_ids_and_native_order(self):
        task = {"job": "camera_imu_calibration", "imus": [{"path": "imu.yaml"}]}
        document = {"schema_version": "1.0.0", "kind": "camera_imu_calibration_initialization",
                    "camera_imu": {"timeshift_cam_imu_s": {"rear": .002, "front": -.001}}}
        validated = validate_initialization(document, "camera_imu_calibration", task,
                                            strategy="direct", camera_ids=["front", "rear"])
        self.assertEqual(validated["camera_imu"]["timeshift_cam_imu_s"], {"cam0": -.001, "cam1": .002})
        report = build_initialization_report(validated, "direct", task, camera_ids=["front", "rear"],
                                             source_path="seed.yaml", source_sha256="abc",
                                             path_origin="task", strategy_origin="task")
        self.assertEqual(report["camera_id_mapping"], {"front": "cam0", "rear": "cam1"})
        self.assertTrue(all(stage["seed_supplied"] for stage in report["stage_decisions"]
                            if stage["stage"].startswith("camera_imu_time_offset.")))

    def test_camera_refine_is_partial_and_model_dimensions_are_strict(self):
        task = self.camera_task("pinhole-radtan5", "omni-none")
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_calibration_initialization",
            "cameras": {
                "cam0": {"intrinsics": [400, 401, 320, 240]},
                "cam1": {
                    "distortion_coeffs": [],
                    "T_cam_from_previous": IDENTITY4,
                },
            },
        }
        validated = validate_initialization(
            document, "camera_calibration", task, strategy="refine")
        self.assertEqual(validated["cameras"]["cam1"]["distortion_coeffs"], [])
        document["cameras"]["cam0"]["distortion_coeffs"] = [0, 0, 0, 0]
        with self.assertRaisesRegex(InitializationError, "5-element"):
            validate_initialization(
                document, "camera_calibration", task, strategy="refine")

    def test_radtan8_initialization_requires_eight_coefficients(self):
        task = self.camera_task("pinhole-radtan8")
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_calibration_initialization",
            "cameras": {
                "cam0": {
                    "distortion_coeffs": [0.0] * 8,
                },
            },
        }
        validated = validate_initialization(
            document, "camera_calibration", task, strategy="refine")
        self.assertEqual(
            validated["cameras"]["cam0"]["distortion_coeffs"], [0.0] * 8)
        document["cameras"]["cam0"]["distortion_coeffs"] = [0.0] * 5
        with self.assertRaisesRegex(InitializationError, "8-element"):
            validate_initialization(
                document, "camera_calibration", task, strategy="refine")

    def test_initialization_schema_version_requires_release_string(self):
        task = self.camera_task("pinhole-radtan")
        for invalid_version in (True, 1, 1.0, 2, "1"):
            with self.subTest(schema_version=invalid_version):
                document = {
                    "schema_version": invalid_version,
                    "kind": "camera_calibration_initialization",
                    "cameras": {},
                }
                with self.assertRaisesRegex(
                        InitializationError, "schema_version must be 1.0.0"):
                    validate_initialization(
                        document, "camera_calibration", task)

    def test_camera_direct_requires_complete_seed(self):
        task = self.camera_task("pinhole-radtan", "pinhole-equi")
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_calibration_initialization",
            "cameras": {"cam0": {"intrinsics": [400, 400, 320, 240]}},
        }
        with self.assertRaisesRegex(
                InitializationError, "direct initialization requires"):
            validate_initialization(
                document, "camera_calibration", task, strategy="direct")

    def test_transform_rotation_and_bottom_row_are_checked(self):
        task = self.camera_task("pinhole-radtan", "pinhole-radtan")
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_calibration_initialization",
            "cameras": {"cam1": {"T_cam_from_previous": IDENTITY4}},
        }
        document["cameras"]["cam1"]["T_cam_from_previous"] = [
            [2, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]
        ]
        with self.assertRaisesRegex(InitializationError, "orthonormal"):
            validate_initialization(document, "camera_calibration", task)
        document["cameras"]["cam1"]["T_cam_from_previous"] = [
            [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 1, 1]
        ]
        with self.assertRaisesRegex(InitializationError, "bottom row"):
            validate_initialization(document, "camera_calibration", task)

    def test_camera_imu_fields_are_gated_by_task_imu_model(self):
        task = {
            "job": "camera_imu_calibration",
            "imus": [{"path": "imu.yaml", "model": "calibrated"}],
        }
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_imu_calibration_initialization",
            "imus": {"imu0": {"M_accel": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}},
        }
        with self.assertRaisesRegex(InitializationError, "unknown fields.*M_accel"):
            validate_initialization(
                document, "camera_imu_calibration", task,
                camera_ids=["cam0"])
        task["imus"][0]["model"] = "scale-misalignment-size-effect"
        document["imus"]["imu0"] = {
            "M_accel": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "C_gyro_i": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "ry_i_m": [0, 0, 0],
            "rz_i_m": [0, 0, 0],
        }
        validated = validate_initialization(
            document, "camera_imu_calibration", task,
            camera_ids=["cam0"])
        self.assertIn("ry_i_m", validated["imus"]["imu0"])

    def test_imu_scale_matrices_are_lower_triangular_with_positive_diagonal(self):
        task = {
            "job": "camera_imu_calibration",
            "imus": [{"path": "imu.yaml", "model": "scale-misalignment"}],
        }
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_imu_calibration_initialization",
            "imus": {"imu0": {
                "M_accel": [[1, 0.1, 0], [0, 1, 0], [0, 0, 1]],
            }},
        }
        with self.assertRaisesRegex(InitializationError, "lower triangular"):
            validate_initialization(
                document, "camera_imu_calibration", task,
                camera_ids=["cam0"])
        document["imus"]["imu0"]["M_accel"] = [
            [1, 0, 0], [0.1, 0, 0], [0.2, 0.3, 1],
        ]
        with self.assertRaisesRegex(InitializationError, "must be positive"):
            validate_initialization(
                document, "camera_imu_calibration", task,
                camera_ids=["cam0"])

    def test_camera_imu_global_fields_are_finite_and_indexed(self):
        task = {
            "job": "camera_imu_calibration",
            "imus": [{"path": "imu.yaml"}],
        }
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_imu_calibration_initialization",
            "camera_imu": {
                "T_cam0_imu": IDENTITY4,
                "timeshift_cam_imu_s": {"cam0": 0.001},
                "gravity_direction_target": [0, 0, -1],
            },
        }
        validated = validate_initialization(
            document, "camera_imu_calibration", task,
            strategy="direct", camera_ids=["cam0"])
        self.assertEqual(
            validated["camera_imu"]["gravity_direction_target"],
            [0.0, 0.0, -1.0])
        document["camera_imu"]["timeshift_cam_imu_s"] = {"cam1": 0.0}
        with self.assertRaisesRegex(InitializationError, "unknown camera time shifts"):
            validate_initialization(
                document, "camera_imu_calibration", task,
                camera_ids=["cam0"])

    def test_refine_time_seeds_require_the_corresponding_estimator(self):
        task = {
            "job": "camera_imu_calibration",
            "calibration": {"calibrate_time_offset": False},
            "imus": [
                {"path": "imu0.yaml"},
                {"path": "imu1.yaml"},
            ],
        }
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_imu_calibration_initialization",
            "camera_imu": {"timeshift_cam_imu_s": {"cam0": 0.001}},
        }
        with self.assertRaisesRegex(
                InitializationError, "calibrate_time_offset"):
            validate_initialization(
                document, "camera_imu_calibration", task,
                strategy="refine", camera_ids=["cam0"])
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_imu_calibration_initialization",
            "imus": {"imu1": {"time_offset_to_reference_s": 0.001}},
        }
        task["calibration"]["calibrate_time_offset"] = True
        with self.assertRaisesRegex(
                InitializationError, "estimate_multi_imu_delay"):
            validate_initialization(
                document, "camera_imu_calibration", task,
                strategy="refine", camera_ids=["cam0"])
        task["calibration"]["estimate_multi_imu_delay"] = True
        validated = validate_initialization(
            document, "camera_imu_calibration", task,
            strategy="refine", camera_ids=["cam0"])
        self.assertEqual(
            validated["imus"]["imu1"]["time_offset_to_reference_s"],
            0.001)

    def test_canonical_document_adds_only_effective_strategy(self):
        source = {
            "schema_version": "1.0.0",
            "kind": "camera_imu_calibration_initialization",
            "camera_imu": {},
        }
        canonical = canonical_initialization(source, "direct")
        self.assertEqual(list(canonical)[:3], ["schema_version", "kind", "strategy"])
        self.assertEqual(canonical["strategy"], "direct")

    def test_report_records_seed_only_semantics_and_stage_policy(self):
        task = self.camera_task("pinhole-radtan", "pinhole-radtan")
        task["job"] = "camera_calibration"
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_calibration_initialization",
            "cameras": {
                "cam0": {
                    "intrinsics": [400.0, 400.0, 320.0, 240.0],
                    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                },
                "cam1": {
                    "intrinsics": [400.0, 400.0, 320.0, 240.0],
                    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                    "T_cam_from_previous": IDENTITY4,
                },
            },
        }
        report = build_initialization_report(
            document, "direct", task, source_path="seed.yaml",
            source_sha256="abc", path_origin="task",
            strategy_origin="task")
        self.assertEqual(report["status"], "configured")
        self.assertEqual(report["semantics"]["role"], "initial_value_only")
        baseline = next(
            item for item in report["stage_decisions"]
            if item["stage"] == "camera_chain_baselines")
        self.assertEqual(baseline["pairwise_stereo_lm"], "skipped")
        self.assertEqual(baseline["full_batch_lm"], "skipped")
        self.assertNotIn("camera_intrinsics_state", baseline)
        self.assertFalse(report["semantics"]["fixed_parameter"])
        self.assertTrue(
            report["semantics"]["final_optimizer_activity_unchanged"])

    def test_report_records_frozen_intrinsics_activity_policy(self):
        task = self.camera_task("pinhole-radtan", "pinhole-radtan")
        task["job"] = "camera_calibration"
        task["calibration"] = {"freeze_intrinsics": True}
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_calibration_initialization",
            "cameras": {
                "cam0": {
                    "intrinsics": [400.0, 400.0, 320.0, 240.0],
                    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                },
                "cam1": {
                    "intrinsics": [400.0, 400.0, 320.0, 240.0],
                    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                },
            },
        }

        report = build_initialization_report(
            document, "refine", task, source_path="seed.yaml",
            source_sha256="abc", path_origin="task",
            strategy_origin="task")

        semantics = report["semantics"]
        self.assertEqual(
            semantics["role"], "initial_value_and_activity_policy")
        self.assertTrue(semantics["fixed_parameter"])
        self.assertFalse(semantics["final_optimizer_activity_unchanged"])
        camera_stages = [
            item for item in report["stage_decisions"]
            if item["stage"].startswith("single_camera_intrinsics.")
        ]
        self.assertTrue(camera_stages)
        for stage in camera_stages:
            self.assertEqual(stage["single_camera_lm"], "skipped")
            self.assertEqual(stage["final_incremental_state"], "fixed")
        baseline = next(
            item for item in report["stage_decisions"]
            if item["stage"] == "camera_chain_baselines")
        self.assertEqual(baseline["pairwise_stereo_lm"], "run")
        self.assertEqual(baseline["full_batch_lm"], "run")
        self.assertEqual(baseline["camera_intrinsics_state"], "fixed")
        self.assertEqual(baseline["final_incremental_state"], "active")

    def test_report_marks_disabled_camera_time_correlation(self):
        task = {
            "job": "camera_imu_calibration",
            "calibration": {"calibrate_time_offset": False},
            "imus": [
                {"path": "imu0.yaml"},
                {"path": "imu1.yaml"},
            ],
        }
        document = {
            "schema_version": "1.0.0",
            "kind": "camera_imu_calibration_initialization",
            "camera_imu": {
                "timeshift_cam_imu_s": {"cam0": 0.001},
            },
            "imus": {
                "imu1": {
                    "gyroscope_bias_rad_s": [0.0, 0.0, 0.0],
                    "T_imu_from_reference": IDENTITY4,
                    "time_offset_to_reference_s": 0.002,
                },
            },
        }
        report = build_initialization_report(
            document, "direct", task, source_path="seed.yaml",
            source_sha256="abc", path_origin="task",
            strategy_origin="default", camera_ids=["cam0"])
        camera_time = next(
            item for item in report["stage_decisions"]
            if item["stage"] == "camera_imu_time_offset.cam0")
        self.assertEqual(camera_time["cross_correlation"], "disabled")
        self.assertEqual(camera_time["final_joint_state"], "inactive")
        final_imu1 = next(
            item for item in report["stage_decisions"]
            if item["stage"] == "final_joint.imu1")
        self.assertEqual(
            final_imu1["supplied_fields"],
            ["T_imu_from_reference", "gyroscope_bias_rad_s"])
        self.assertEqual(
            final_imu1["precomputed_fields"],
            ["time_offset_to_reference_s"])
        preliminary_imu1 = next(
            item for item in report["stage_decisions"]
            if item["stage"] == "multi_imu_preliminary.imu1")
        self.assertEqual(
            preliminary_imu1["rotation_and_gyro_bias_lm"], "skipped")


class InitializationTaskIntegrationTest(unittest.TestCase):
    def _write_camera_task(self, directory, initialization=None,
                           num_cameras=1, save_diagnostics=False):
        directory = Path(directory)
        lines = [
            "schema_version: 1.0.0",
            "job: camera_calibration",
            "dataset: {type: bag, path: missing.bag}",
            "target: {path: target.yaml}",
            "cameras: [" + ", ".join(
                "{{topic: /cam{0}, model: pinhole-radtan}}".format(index)
                for index in range(num_cameras)
            ),
        ]
        # Keep the fixture readable while producing a flow-style YAML list.
        lines[-1] += "]"
        if save_diagnostics:
            lines.append("output: {save_diagnostics: true}")
        if initialization is not None:
            lines.extend([
                "initialization:",
                "  path: {}".format(initialization[0]),
                "  strategy: {}".format(initialization[1]),
            ])
        path = directory / "task.yaml"
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return path

    def _write_camera_seed(self, path, focal_length):
        Path(path).write_text(
            "schema_version: 1.0.0\n"
            "kind: camera_calibration_initialization\n"
            "cameras:\n"
            "  cam0:\n"
            "    intrinsics: [{0}, {0}, 320, 240]\n".format(focal_length),
            encoding="utf-8",
        )

    def test_task_initialization_mapping_is_strict(self):
        with tempfile.TemporaryDirectory() as directory:
            task_path = self._write_camera_task(
                directory, ("seed.yaml", "refine"))
            task = load_task(task_path)
            self.assertEqual(task["initialization"]["strategy"], "refine")
            text = task_path.read_text(encoding="utf-8")
            task_path.write_text(text + "  extra: true\n", encoding="utf-8")
            with self.assertRaisesRegex(TaskError, "unknown fields in initialization"):
                load_task(task_path)

    def test_task_schema_version_requires_release_string(self):
        with tempfile.TemporaryDirectory() as directory:
            task_path = self._write_camera_task(directory)
            original = task_path.read_text(encoding="utf-8")
            for invalid_version in ("true", "1", "1.0", "2"):
                with self.subTest(schema_version=invalid_version):
                    task_path.write_text(
                        original.replace(
                            "schema_version: 1.0.0",
                            "schema_version: {}".format(invalid_version),
                            1,
                        ),
                        encoding="utf-8",
                    )
                    with self.assertRaisesRegex(
                            TaskError, "task schema_version must be 1.0.0"):
                        load_task(task_path)

    def test_task_initialization_strategy_is_optional(self):
        with tempfile.TemporaryDirectory() as directory:
            seed = Path(directory) / "seed.yaml"
            self._write_camera_seed(seed, 100)
            task_path = self._write_camera_task(directory)
            with task_path.open("a", encoding="utf-8") as stream:
                stream.write("initialization: {path: seed.yaml}\n")
            task = load_task(task_path)
            resolved = resolve_initialization(task)
            self.assertEqual(resolved["strategy"], "refine")

            task_path = self._write_camera_task(directory)
            task_path.write_text(
                task_path.read_text(encoding="utf-8") + "initialization: null\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(TaskError, "must be a mapping"):
                load_task(task_path)

    def test_cli_path_overrides_task_path_and_is_cwd_relative(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            task_dir = root / "task"
            cli_dir = root / "cli"
            task_dir.mkdir()
            cli_dir.mkdir()
            self._write_camera_seed(task_dir / "seed.yaml", 100)
            self._write_camera_seed(cli_dir / "seed.yaml", 200)
            task = load_task(self._write_camera_task(
                task_dir, ("seed.yaml", "direct")))
            previous = Path.cwd()
            os.chdir(str(cli_dir))
            try:
                resolved = resolve_initialization(
                    task, initialization="seed.yaml",
                    initialization_strategy="refine")
            finally:
                os.chdir(str(previous))
            self.assertEqual(resolved["path"], (cli_dir / "seed.yaml").resolve())
            self.assertEqual(
                resolved["document"]["cameras"]["cam0"]["intrinsics"][0],
                200.0)

    def test_initialization_path_without_strategy_defaults_to_refine(self):
        with tempfile.TemporaryDirectory() as directory:
            seed = Path(directory) / "seed.yaml"
            self._write_camera_seed(seed, 100)
            task = load_task(self._write_camera_task(directory))
            resolved = resolve_initialization(task, initialization=seed)
            self.assertEqual(resolved["strategy"], "refine")
            with self.assertRaisesRegex(TaskError, "strategy requires"):
                resolve_initialization(
                    task, initialization_strategy="direct")

    def test_self_contained_task_families_resolve_refine_and_direct_initialization(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            unseeded = load_task(self._write_camera_task(root))
            self.assertIsNone(resolve_initialization(unseeded))
            camera_result = {
                "schema_version": "1.0.0", "kind": "calibration_result",
                "calibration_type": "cameras", "cameras": [{
                    "id": "cam0", "camera_model": "pinhole",
                    "distortion_model": "equidistant", "intrinsics": [400., 401., 320., 240.],
                    "distortion_coeffs": [0., 0., 0., 0.],
                    "resolution": [640, 480], "rostopic": "/cam0",
                }],
            }
            task_module.dump_yaml(camera_result, root / "calibration.yaml")
            for job in ("camera_calibration", "camera_imu_calibration"):
                for strategy in ("refine", "direct"):
                    with self.subTest(job=job, strategy=strategy):
                        if job == "camera_calibration":
                            seed = {"schema_version": "1.0.0",
                                    "kind": "camera_calibration_initialization",
                                    "cameras": {"cam0": {"intrinsics": [400., 401., 320., 240.]}}}
                            if strategy == "direct":
                                seed["cameras"]["cam0"]["distortion_coeffs"] = [0.] * 4
                            task_path = self._write_camera_task(root, ("seed.yaml", strategy))
                        else:
                            seed = {"schema_version": "1.0.0",
                                    "kind": "camera_imu_calibration_initialization",
                                    "camera_imu": {"T_cam0_imu": IDENTITY4}}
                            if strategy == "direct":
                                seed["camera_imu"].update({
                                    "timeshift_cam_imu_s": {"cam0": 0.001},
                                    "gravity_direction_target": [0., 0., -1.],
                                })
                            task_path = root / "task.yaml"
                            task_module.dump_yaml({
                                "schema_version": "1.0.0", "job": job,
                                "dataset": {"type": "bag", "path": "missing.bag"},
                                "target": {"path": "target.yaml"},
                                "camera_calibration": {"path": "calibration.yaml"},
                                "imus": [{"path": "imu.yaml", "model": "calibrated"}],
                                "initialization": {"path": "seed.yaml", "strategy": strategy},
                            }, task_path)
                        task_module.dump_yaml(seed, root / "seed.yaml")
                        resolved = resolve_initialization(load_task(task_path))
                        self.assertEqual(resolved["strategy"], strategy)
                        self.assertEqual(resolved["path_origin"], "task")
                        self.assertEqual(resolved["strategy_origin"], "task")
                        self.assertEqual(resolved["path"], root / "seed.yaml")

    def test_invalid_initialization_does_not_clean_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            task_path = self._write_camera_task(
                root, ("invalid.yaml", "refine"))
            (root / "invalid.yaml").write_text(
                "schema_version: 1.0.0\n"
                "kind: camera_imu_calibration_initialization\n",
                encoding="utf-8",
            )
            output = root / "output"
            output.mkdir()
            result = output / "camera_calibration_cam0.yaml"
            result.write_text("preserve\n", encoding="utf-8")
            with self.assertRaisesRegex(TaskError, "initialization kind"):
                run_task(ROOT, task_path, output, "camera_calibration", force=True)
            self.assertEqual(result.read_text(encoding="utf-8"), "preserve\n")

    def test_rank_failure_preserves_only_diagnostic_sidecars(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._write_camera_seed(root / "seed.yaml", 100)
            task_path = self._write_camera_task(
                root, ("seed.yaml", "refine"), save_diagnostics=True)
            output = root / "output"

            def fail_with_diagnostics(prefix, command, arguments, work):
                del prefix, command, arguments
                (Path(work) / "observability.yaml").write_text(
                    "schema_version: 1.0.0\n"
                    "kind: calibration_diagnostics\n"
                    "status: rank_deficient\n"
                    "calibration: {columns: 4, rank: 3, deficiency: 1}\n",
                    encoding="utf-8")
                raise RuntimeError("rank deficient")

            with mock.patch.object(
                    task_module, "_dataset_alias", return_value="bag"), \
                    mock.patch.object(
                        task_module, "_target_path", return_value="target"), \
                    mock.patch.object(
                        task_module, "_run_legacy",
                        side_effect=fail_with_diagnostics), \
                    mock.patch("kalibr_no_ros.validation.validate_task", return_value={"status": "passed"}):
                with self.assertRaisesRegex(RuntimeError, "rank deficient"):
                    run_task(
                        ROOT, task_path, output,
                        "camera_calibration", force=True)

            diagnostics = output / "camera_calibration_cam0_failed"
            self.assertFalse((output / "camera_calibration_cam0.yaml").exists())
            self.assertFalse((diagnostics / "result.yaml").exists())
            self.assertTrue((diagnostics / "observability.yaml").is_file())
            report = task_module.load_yaml(
                diagnostics / "initialization_report.yaml")
            self.assertEqual(report["status"], "failed_rank_deficient")
            self.assertEqual(report["observability"]["rank"], 3)

    def test_early_dataset_failure_still_preserves_initialization_report(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._write_camera_seed(root / "seed.yaml", 100)
            task_path = self._write_camera_task(
                root, ("seed.yaml", "refine"), save_diagnostics=True)
            output = root / "output"

            with self.assertRaisesRegex(TaskError, "input validation failed"):
                run_task(
                    ROOT, task_path, output,
                    "camera_calibration", force=True)

            diagnostics = output / "camera_calibration_cam0_failed"
            self.assertFalse((output / "camera_calibration_cam0.yaml").exists())
            self.assertFalse((diagnostics / "observability.yaml").exists())
            report = task_module.load_yaml(
                diagnostics / "initialization_report.yaml")
            self.assertEqual(report["status"], "failed")
            self.assertEqual(report["failure"]["type"], "TaskError")

    def test_seeded_report_failure_preserves_valid_calibration_and_failure_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._write_camera_seed(root / "seed.yaml", 100)
            task_path = self._write_camera_task(
                root, ("seed.yaml", "refine"), save_diagnostics=True)
            output = root / "output"

            def finish_native(prefix, command, arguments, work):
                del prefix, command, arguments
                artifact_module.current_context().artifacts["state"] = "completed"
                task_module.dump_yaml({"cam0": {
                    "camera_model": "pinhole", "distortion_model": "radtan",
                    "intrinsics": [100., 100., 320., 240.],
                    "distortion_coeffs": [0., 0., 0., 0.],
                    "resolution": [640, 480], "rostopic": "/cam0",
                }}, Path(work) / "test-camchain.yaml")
                (Path(work) / "test-results-cam.txt").write_text(
                    "Native calibration completed.\n", encoding="utf-8")
                (Path(work) / "observability.yaml").write_text(
                    "schema_version: 1.0.0\n"
                    "kind: calibration_diagnostics\n"
                    "status: full_rank\n"
                    "calibration: {columns: 4, rank: 4, deficiency: 0}\n",
                    encoding="utf-8")

            with mock.patch.object(
                    task_module, "_dataset_alias", return_value="bag"), \
                    mock.patch.object(
                        task_module, "_target_path", return_value="target"), \
                    mock.patch.object(
                        task_module, "_run_legacy", side_effect=finish_native), \
                    mock.patch.object(
                        reporting, "generate_report",
                        side_effect=RuntimeError("postprocessing failed")), \
                    mock.patch("kalibr_no_ros.validation.validate_task", return_value={"status": "passed"}):
                with self.assertRaisesRegex(
                        RuntimeError, "postprocessing failed"):
                    run_task(
                        ROOT, task_path, output,
                        "camera_calibration", force=True)

            diagnostics = output / "camera_calibration_cam0_failed"
            self.assertFalse((output / "camera_calibration_cam0.yaml").exists())
            result = task_module.load_yaml(diagnostics / "result.yaml")
            self.assertEqual(result["schema_version"], "1.0.0")
            self.assertEqual(result["kind"], "calibration_result")
            self.assertEqual(result["cameras"][0]["intrinsics"], [100., 100., 320., 240.])
            self.assertTrue((diagnostics / "observability.yaml").is_file())
            manifest = json.loads((diagnostics / "run_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["status"], "output_failed")
            self.assertEqual(manifest["failure"]["message"], "postprocessing failed")
            self.assertIn("result.yaml", {entry["path"] for entry in manifest["files"]})
            report = task_module.load_yaml(
                diagnostics / "initialization_report.yaml")
            self.assertEqual(report["status"], "failed")
            saved_result = diagnostics / report["result"]["path"]
            self.assertEqual(task_module.load_yaml(saved_result), result)
            self.assertEqual(report["result"]["sha256"], task_module._sha256(saved_result))

    def test_legacy_flag_is_absent_without_initialization(self):
        task = {
            "dataset": {},
            "cameras": [{"topic": "/cam0", "model": "pinhole-radtan"}],
        }
        arguments = _camera_arguments(task, "bag", "target", {})
        self.assertNotIn("--initialization-config", arguments)
        arguments = _camera_arguments(
            task, "bag", "target", {}, Path("canonical.yaml"))
        index = arguments.index("--initialization-config")
        self.assertEqual(arguments[index + 1], "canonical.yaml")

    def test_corner_refinement_is_forwarded_and_validated(self):
        task = {
            "dataset": {},
            "cameras": [{"topic": "/cam0", "model": "pinhole-radtan"}],
            "calibration": {
                "window_half_size_px": 3,
                "max_displacement_px": 1.75,
            },
        }
        arguments = _camera_arguments(task, "bag", "target", {})
        self.assertEqual(
            arguments[arguments.index("--window-half-size-px") + 1], "3")
        self.assertEqual(
            arguments[arguments.index("--max-displacement-px") + 1], "1.75")

        for key, value in (
                ("window_half_size_px", 0),
                ("window_half_size_px", 1.5),
                ("max_displacement_px", 0.0),
                ("max_displacement_px", float("inf"))):
            invalid = dict(task)
            invalid["calibration"] = {key: value}
            with self.subTest(key=key, value=value):
                with self.assertRaises(TaskError):
                    _camera_arguments(invalid, "bag", "target", {})

    def test_partial_focal_initialization_ratio_is_forwarded_and_validated(self):
        task = {
            "dataset": {},
            "cameras": [{"topic": "/cam0", "model": "pinhole-radtan8"}],
            "calibration": {
                "focal_initialization_min_visible_corner_ratio": 0.75,
            },
        }
        arguments = _camera_arguments(task, "bag", "target", {})
        option = "--focal-initialization-min-visible-corner-ratio"
        self.assertEqual(arguments[arguments.index(option) + 1], "0.75")

        without_setting = dict(task)
        without_setting["calibration"] = {}
        self.assertNotIn(
            option, _camera_arguments(
                without_setting, "bag", "target", {}))

        for value in (
                0.0, -0.1, 1.01, float("inf"), float("nan"), True,
                "0.75"):
            invalid = dict(task)
            invalid["calibration"] = {
                "focal_initialization_min_visible_corner_ratio": value,
            }
            with self.subTest(value=value):
                with self.assertRaisesRegex(TaskError, r"in \(0, 1\]"):
                    _camera_arguments(invalid, "bag", "target", {})

    def test_freeze_intrinsics_is_forwarded_only_when_enabled(self):
        base = {
            "dataset": {},
            "cameras": [
                {"topic": "/cam0", "model": "pinhole-radtan"},
                {"topic": "/cam1", "model": "pinhole-radtan"},
            ],
        }
        self.assertNotIn(
            "--freeze-intrinsics",
            _camera_arguments(base, "bag", "target", {}))

        disabled = dict(base)
        disabled["calibration"] = {"freeze_intrinsics": False}
        self.assertNotIn(
            "--freeze-intrinsics",
            _camera_arguments(disabled, "bag", "target", {}))

        enabled = dict(base)
        enabled["calibration"] = {"freeze_intrinsics": True}
        arguments = _camera_arguments(
            enabled, "bag", "target", {}, Path("initialization.yaml"))
        self.assertIn("--freeze-intrinsics", arguments)

    def test_freeze_intrinsics_requires_boolean_and_initialization(self):
        base = {
            "dataset": {},
            "cameras": [
                {"topic": "/cam0", "model": "pinhole-radtan"},
                {"topic": "/cam1", "model": "pinhole-radtan"},
            ],
        }
        for value in (0, 1, "true", None):
            invalid = dict(base)
            invalid["calibration"] = {"freeze_intrinsics": value}
            with self.subTest(value=value):
                with self.assertRaisesRegex(TaskError, "must be a boolean"):
                    _camera_arguments(invalid, "bag", "target", {})

        enabled = dict(base)
        enabled["calibration"] = {"freeze_intrinsics": True}
        with self.assertRaisesRegex(TaskError, "requires an initialization"):
            _camera_arguments(enabled, "bag", "target", {})

        monocular = {
            "dataset": {},
            "cameras": [{"topic": "/cam0", "model": "pinhole-radtan"}],
            "calibration": {"freeze_intrinsics": True},
        }
        with self.assertRaisesRegex(TaskError, "at least two cameras"):
            _camera_arguments(
                monocular, "bag", "target", {}, Path("initialization.yaml"))

    def test_freeze_intrinsics_requires_complete_camera_models(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            task_path = self._write_camera_task(
                root, ("seed.yaml", "refine"), num_cameras=2)
            with task_path.open("a", encoding="utf-8") as stream:
                stream.write("calibration: {freeze_intrinsics: true}\n")
            (root / "seed.yaml").write_text(
                "schema_version: 1.0.0\n"
                "kind: camera_calibration_initialization\n"
                "cameras:\n"
                "  cam0:\n"
                "    intrinsics: [400, 400, 320, 240]\n"
                "  cam1:\n"
                "    intrinsics: [400, 400, 320, 240]\n",
                encoding="utf-8")
            task = load_task(task_path)
            with self.assertRaisesRegex(
                    TaskError, r"missing: cam0\.distortion_coeffs"):
                resolve_initialization(task)

            (root / "seed.yaml").write_text(
                "schema_version: 1.0.0\n"
                "kind: camera_calibration_initialization\n"
                "cameras:\n"
                "  cam0:\n"
                "    intrinsics: [400, 400, 320, 240]\n"
                "    distortion_coeffs: [0, 0, 0, 0]\n"
                "  cam1:\n"
                "    intrinsics: [400, 400, 320, 240]\n"
                "    distortion_coeffs: [0, 0, 0, 0]\n",
                encoding="utf-8")
            resolved = resolve_initialization(task)
            self.assertEqual(
                resolved["document"]["cameras"]["cam0"]
                ["distortion_coeffs"],
                [0.0, 0.0, 0.0, 0.0])

    def test_public_cli_flags_and_cwd_resolution(self):
        arguments = build_parser().parse_args([
            "calibrate", "cameras", "--config", "task.yaml",
            "--output-dir", "out", "--initialization", "seed.yaml",
            "--initialization-strategy", "direct",
        ])
        overrides = _runtime_overrides(arguments, "out")
        self.assertEqual(overrides["initialization"], Path("seed.yaml").resolve())
        self.assertEqual(overrides["initialization_strategy"], "direct")


if __name__ == "__main__":
    unittest.main()
