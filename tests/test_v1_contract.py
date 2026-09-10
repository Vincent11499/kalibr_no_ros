"""Public 1.0.0 contracts using self-contained inputs and real file I/O."""

import copy
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/python"))

from kalibr_no_ros import artifacts, reporting
from kalibr_no_ros.task import (
    TaskError, dump_yaml, load_task, load_yaml, prepare_output_directory,
    require_document_version, resolve_initialization, run_task, write_run_manifest,
)
from kalibr_no_ros.validation import load_cameras, load_imu, load_target, validate_options, validate_task


class V1ContractTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.dataset = self.root / "input_260909_1234"
        images = self.dataset / "cam0/images"
        images.mkdir(parents=True)
        for index in range(2):
            self.assertTrue(cv2.imwrite(str(images / "{}.png".format(index)),
                                       np.full((8, 10), index, dtype=np.uint8)))
        (self.dataset / "cam0/timestamps.csv").write_text(
            "timestamp_ns,filename\n1000000001,0.png\n2000000001,1.png\n", encoding="utf-8")
        (self.dataset / "imu.csv").write_text(
            "timestamp_ns,wx,wy,wz,ax,ay,az\n"
            "500000001,0,0,0,0,0,9.81\n"
            "1500000001,0,0,0,0,0,9.81\n"
            "2500000001,0,0,0,0,0,9.81\n", encoding="utf-8")
        self.manifest = {
            "schema_version": "1.0.0", "type": "kalibr_directory_dataset",
            "dataset_id": self.dataset.name,
            "cameras": [{"id": "cam0", "topic": "/cam0/image_raw",
                         "timestamps": "cam0/timestamps.csv", "images": "cam0/images",
                         "resolution": [10, 8], "clock_source": "device"}],
            "imus": [{"id": "imu0", "topic": "/imu0", "data": "imu.csv",
                      "angular_velocity_unit": "rad/s", "linear_acceleration_unit": "m/s^2"}],
        }
        self.task = {
            "schema_version": "1.0.0", "kind": "calibration_task",
            "job": "camera_calibration",
            "dataset": {"type": "directory", "path": self.dataset.name},
            "target": {"type": "aprilgrid", "parameters": {
                "tagRows": 3, "tagCols": 3, "tagSize": 0.04, "tagSpacing": 0.3,
                "tagStartId": 0,
            }},
            "cameras": [{"id": "cam0", "topic": "/cam0/image_raw", "model": "pinhole-equi"}],
            "calibration": {"shuffle": False},
        }
        self.task_path = self.root / "task.yaml"
        self.write_inputs()

    def write_inputs(self):
        dump_yaml(self.manifest, self.dataset / "dataset.yaml")
        dump_yaml(self.task, self.task_path)

    def camera_imu_task(self):
        self.task.pop("cameras", None)
        self.task["job"] = "camera_imu_calibration"
        self.task["calibration"] = {}
        self.task["camera_calibration"] = {"path": "calibration.yaml"}
        self.task["imus"] = [{"path": "imu.yaml", "model": "calibrated"}]
        camera = {
            "id": "cam0", "rostopic": "/cam0/image_raw", "camera_model": "pinhole",
            "distortion_model": "equidistant", "intrinsics": [7., 7., 5., 4.],
            "distortion_coeffs": [0., 0., 0., 0.], "resolution": [10, 8],
        }
        dump_yaml({"schema_version": "1.0.0", "kind": "calibration_result",
                   "calibration_type": "cameras", "cameras": [camera]}, self.root / "calibration.yaml")
        dump_yaml({"schema_version": "1.0.0", "kind": "imu_configuration", "rostopic": "/imu0",
                   "update_rate": 1., "accelerometer_noise_density": 0.01,
                   "accelerometer_random_walk": 0.001, "gyroscope_noise_density": 0.001,
                   "gyroscope_random_walk": 0.0001}, self.root / "imu.yaml")
        self.write_inputs()

    def assert_validation_failed(self, pattern):
        report = validate_task(self.task_path)
        self.assertEqual(report["status"], "failed", report)
        self.assertRegex("; ".join(report["errors"]), pattern)

    def test_directory_topics_can_be_omitted_and_native_arguments_select_same_stream(self):
        from kalibr_no_ros.task import _camera_arguments
        original = _camera_arguments(load_task(self.task_path), self.dataset, 'target.yaml', {})
        del self.task['cameras'][0]['topic']
        self.write_inputs()
        self.assertEqual(validate_task(self.task_path)['status'], 'passed')
        self.assertEqual(_camera_arguments(load_task(self.task_path), self.dataset, 'target.yaml', {}), original)
        del self.manifest['cameras'][0]['topic']
        self.write_inputs()
        actual = _camera_arguments(load_task(self.task_path), self.dataset, 'target.yaml', {})
        self.assertEqual(actual[actual.index('--topics') + 1], 'cam0')
        self.assertEqual(validate_task(self.task_path)['status'], 'passed')

    def test_directory_id_is_checked_even_when_a_topic_is_supplied(self):
        self.manifest['cameras'][0]['id'] = 'different_camera'
        self.write_inputs()
        self.assert_validation_failed('unknown camera id')
        self.manifest['cameras'][0]['id'] = 'cam0'
        self.task['cameras'][0]['topic'] = '/wrong'
        self.write_inputs()
        self.assert_validation_failed('topic disagrees')

    def test_bag_still_requires_camera_and_imu_topics(self):
        self.task['dataset']['type'] = 'bag'
        del self.task['cameras'][0]['topic']
        self.write_inputs()
        self.assert_validation_failed('camera topics')
        self.camera_imu_task()
        p = self.root / 'imu.yaml'
        imu = load_yaml(p)
        del imu['rostopic']
        dump_yaml(imu, p)
        self.assert_validation_failed('IMU.rostopic')

    def test_directory_imu_and_camera_chain_resolve_ids_without_modifying_input_result(self):
        from kalibr_no_ros.task import _imu_arguments
        self.camera_imu_task()
        self.task['imus'][0]['id'] = 'imu0'
        for streams in (self.manifest['cameras'], self.manifest['imus']):
            for stream in streams:
                del stream['topic']
        p = self.root / 'imu.yaml'
        imu = load_yaml(p)
        del imu['rostopic']
        dump_yaml(imu, p)
        self.write_inputs()
        task = load_task(self.task_path)
        self.assertEqual(validate_task(self.task_path)['status'], 'passed')
        source = (self.root / 'calibration.yaml').read_bytes()
        _imu_arguments(task, self.dataset, 'target.yaml', self.root, {})
        self.assertEqual(load_yaml(self.root / 'imu0.yaml')['rostopic'], 'imu0')
        self.assertEqual(load_yaml(self.root / 'camchain.yaml')['cam0']['rostopic'], 'cam0')
        self.assertEqual((self.root / 'calibration.yaml').read_bytes(), source)

    def test_only_release_string_is_accepted_for_task_dataset_and_result(self):
        for old in (1, 2, True, 1.0, "1", "2"):
            with self.subTest(version=old):
                self.task["schema_version"] = old
                self.write_inputs()
                with self.assertRaisesRegex(TaskError, "schema_version"):
                    load_task(self.task_path)
                self.task["schema_version"] = "1.0.0"
                self.manifest["schema_version"] = old
                self.write_inputs()
                self.assert_validation_failed("schema_version")
                self.manifest["schema_version"] = "1.0.0"
                with self.assertRaisesRegex(TaskError, "schema_version"):
                    require_document_version({"schema_version": old, "kind": "calibration_result"},
                                             "camera calibration", "calibration_result")
        self.write_inputs()
        self.assertEqual(validate_task(self.task_path)["status"], "passed")

    def test_validate_cli_decodes_directory_without_native_detector_or_ros_imports(self):
        output = self.root / "validation.json"
        script = (
            "import sys\n"
            "from kalibr_no_ros.cli import main\n"
            "code = main(sys.argv[1:])\n"
            "blocked = ('rospy', 'rosbag', 'aslam_cv', 'aslam_backend', 'kalibr_common', 'kalibr_camera_calibration')\n"
            "assert not [name for name in sys.modules if any(name == prefix or name.startswith(prefix + '.') for prefix in blocked)]\n"
            "raise SystemExit(code)\n"
        )
        command = [sys.executable, "-c", script, "validate", "--config", str(self.task_path),
                   "--output", str(output)]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join([str(ROOT / "src/python"), str(ROOT / ".deps/python")])
        result = subprocess.run(command, cwd=self.root, env=environment, text=True,
                                capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        report = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(report["schema_version"], "1.0.0")
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["cameras"][0]["images"], 2)
        self.assertEqual(report["cameras"][0]["resolution"], [10, 8])
        self.assertEqual(int(report["cameras"][0]["first_timestamp_ns"]), 1000000001)

    def test_bad_imu_units_and_nonmonotonic_timestamps_are_rejected(self):
        for field, invalid in (("angular_velocity_unit", "deg/s"), ("linear_acceleration_unit", "g")):
            with self.subTest(field=field):
                original = self.manifest["imus"][0][field]
                self.manifest["imus"][0][field] = invalid
                self.write_inputs()
                self.assert_validation_failed(field)
                self.manifest["imus"][0][field] = original
        self.write_inputs()
        table = self.dataset / "cam0/timestamps.csv"
        for stamps in ((2000000001, 1000000001), (1000000001, 1000000001)):
            with self.subTest(stamps=stamps):
                table.write_text("timestamp_ns,filename\n{},0.png\n{},1.png\n".format(*stamps), encoding="utf-8")
                self.assert_validation_failed("strictly increasing|timestamp")

    def test_mixed_and_incorrect_declared_image_dimensions_are_rejected(self):
        self.manifest["cameras"][0]["resolution"] = [11, 8]
        self.write_inputs()
        self.assert_validation_failed("resolution")
        self.manifest["cameras"][0]["resolution"] = [10, 8]
        self.write_inputs()
        self.assertTrue(cv2.imwrite(str(self.dataset / "cam0/images/1.png"),
                                   np.zeros((9, 10), dtype=np.uint8)))
        self.assert_validation_failed("resolution")

    def test_camera_result_model_dimensions_and_time_coverage_are_checked(self):
        self.camera_imu_task()
        self.assertEqual(validate_task(self.task_path)["status"], "passed")
        result_path = self.root / "calibration.yaml"
        original = load_yaml(result_path)
        for field, value in (("intrinsics", [7., 5., 4.]),
                             ("distortion_coeffs", [0., 0., 0.]),
                             ("intrinsics", [float("nan"), 7., 5., 4.]),
                             ("resolution", [10])):
            with self.subTest(field=field, value=value):
                result = copy.deepcopy(original)
                result["cameras"][0][field] = value
                dump_yaml(result, result_path)
                self.assert_validation_failed("intrinsics|distortion|resolution|finite")
        dump_yaml(original, result_path)
        (self.dataset / "imu.csv").write_text(
            "timestamp_ns,wx,wy,wz,ax,ay,az\n"
            "1100000001,0,0,0,0,0,9.81\n1900000001,0,0,0,0,0,9.81\n", encoding="utf-8")
        self.assert_validation_failed("cover")

    @staticmethod
    def tree_bytes(directory):
        return {path.relative_to(directory).as_posix(): path.read_bytes()
                for path in directory.rglob("*") if path.is_file() and not path.is_symlink()}

    def test_force_inspects_entire_managed_subtree_before_deleting_any_file(self):
        output = self.root / "output"
        (output / "observations").mkdir(parents=True)
        (output / "calibration.yaml").write_text("known result\n", encoding="utf-8")
        (output / "observations/corners.csv.gz").write_bytes(b"known archive")
        write_run_manifest(output, status="completed")
        unexpected = output / "observations/my_notes.txt"
        unexpected.write_text("user data\n", encoding="utf-8")
        before = self.tree_bytes(output)
        with self.assertRaisesRegex(TaskError, "unmanaged"):
            prepare_output_directory(output, force=True)
        self.assertEqual(self.tree_bytes(output), before)
        unexpected.unlink()
        self.assertEqual(prepare_output_directory(output, force=True), output)
        self.assertEqual(list(output.iterdir()), [])

    def test_force_rejects_symlink_and_preserves_external_target(self):
        output = self.root / "output"
        (output / "observations").mkdir(parents=True)
        (output / "calibration.yaml").write_text("known\n", encoding="utf-8")
        write_run_manifest(output, status="completed")
        external = self.root / "external.txt"
        external.write_text("external\n", encoding="utf-8")
        (output / "observations/link.txt").symlink_to(external)
        before = self.tree_bytes(output)
        with self.assertRaisesRegex(TaskError, "symlink"):
            prepare_output_directory(output, force=True)
        self.assertEqual(self.tree_bytes(output), before)
        self.assertEqual(external.read_text(encoding="utf-8"), "external\n")
        self.assertTrue((output / "observations/link.txt").is_symlink())

    def test_force_cannot_delete_camera_calibration_used_as_current_input(self):
        self.camera_imu_task()
        output = self.root / "existing_stereo"
        output.mkdir()
        (self.root / "calibration.yaml").rename(output / "camera_input.yaml")
        (output / "results.txt").write_text("previous stereo result\n", encoding="utf-8")
        write_run_manifest(output, status="completed")
        self.task["camera_calibration"]["path"] = "existing_stereo/camera_input.yaml"
        self.task["output"] = {"name": "camera_input"}
        self.write_inputs()
        before = self.tree_bytes(output)
        with mock.patch("kalibr_no_ros.task._run_legacy") as solver:
            with self.assertRaisesRegex(TaskError, "input|overlap|contain|separate"):
                run_task(self.root / "unused_prefix", self.task_path, output,
                         "camera_imu_calibration", force=True)
            solver.assert_not_called()
        self.assertEqual(self.tree_bytes(output), before)

    def test_resolved_snapshot_reloads_after_relocation_with_cli_seed_override(self):
        self.camera_imu_task()
        inputs = self.root / "configuration"
        inputs.mkdir()
        target = dict(self.task["target"]["parameters"], schema_version="1.0.0",
                      kind="calibration_target", target_type="aprilgrid")
        dump_yaml(target, inputs / "target.yaml")
        self.task["target"] = {"path": "configuration/target.yaml"}
        initial = {
            "schema_version": "1.0.0", "kind": "camera_imu_calibration_initialization",
            "camera_imu": {"T_cam0_imu": np.eye(4).tolist(), "timeshift_cam_imu_s": {"cam0": 0.0}},
            "imus": {"imu0": {"gyroscope_bias_rad_s": [0.0, 0.0, 0.0]}},
        }
        dump_yaml(initial, inputs / "task_seed.yaml")
        cli_seed = copy.deepcopy(initial)
        cli_seed["camera_imu"]["timeshift_cam_imu_s"]["cam0"] = 0.0123
        cli_seed["imus"]["imu0"]["gyroscope_bias_rad_s"] = [0.1, -0.2, 0.3]
        cli_seed_path = inputs / "cli_seed.yaml"
        dump_yaml(cli_seed, cli_seed_path)
        self.task["initialization"] = {"path": "configuration/task_seed.yaml", "strategy": "refine"}
        self.task["output"] = {"save_diagnostics": True}
        self.write_inputs()
        task_bytes = self.task_path.read_bytes()
        output = self.root / "snapshot_run"
        # Stop at the native boundary: exercise real resolution and persistence
        # without detection, optimization, or constructing synthetic results.
        with mock.patch("kalibr_no_ros.task._run_legacy", side_effect=RuntimeError("snapshot boundary")):
            with self.assertRaisesRegex(RuntimeError, "snapshot boundary"):
                run_task(self.root / "unused_prefix", self.task_path, output,
                         "camera_imu_calibration", initialization=cli_seed_path,
                         initialization_strategy="direct", optimizer_threads=2)
        relocated = self.root / "relocated" / "task.yaml"
        relocated.parent.mkdir()
        relocated.write_bytes((output / "camera_imu_calibration_cam0_imu0_failed/task_resolved.yaml").read_bytes())
        task = load_task(relocated)
        self.assertEqual(load_target(task), target)
        self.assertEqual(load_imu(task, task["imus"][0])["rostopic"], "/imu0")
        self.assertEqual(load_cameras(task)[0]["topic"], "/cam0/image_raw")
        seed = resolve_initialization(task)
        self.assertEqual(seed["path"], cli_seed_path.resolve())
        self.assertEqual(seed["strategy"], "direct")
        self.assertEqual(seed["document"]["camera_imu"]["timeshift_cam_imu_s"]["cam0"], 0.0123)
        self.assertEqual(task["execution"]["optimizer_threads"], 2)
        for value in (task["dataset"]["path"], task["target"]["path"],
                      task["camera_calibration"]["path"], task["imus"][0]["path"],
                      task["initialization"]["path"]):
            self.assertTrue(Path(value).is_absolute(), value)
        self.assertEqual(self.task_path.read_bytes(), task_bytes)

    def test_bad_yaml_is_a_structured_validation_failure(self):
        from kalibr_no_ros.cli import main
        self.task_path.write_text("schema_version: [\n", encoding="utf-8")
        report = validate_task(self.task_path)
        self.assertEqual(report["status"], "failed")
        self.assertTrue(report["errors"])
        destination = self.root / "invalid_yaml_validation.json"
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            code = main(["validate", "--config", str(self.task_path), "--output", str(destination)])
        self.assertEqual(code, 2)
        document = json.loads(destination.read_text(encoding="utf-8"))
        self.assertEqual(document["schema_version"], "1.0.0")
        self.assertEqual(document["kind"], "input_validation")
        self.assertEqual(document["status"], "failed")

    def test_non_string_target_path_is_a_structured_validation_failure(self):
        from kalibr_no_ros.cli import main
        for invalid in (123, ["target.yaml"], {"path": "target.yaml"}):
            with self.subTest(path=invalid):
                self.task["target"] = {"path": invalid}
                self.write_inputs()
                report = validate_task(self.task_path)
                self.assertEqual(report["status"], "failed", report)
                self.assertRegex("; ".join(report["errors"]), "target.path|path.*string")
                stdout = io.StringIO()
                with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(io.StringIO()):
                    code = main(["validate", "--config", str(self.task_path)])
                self.assertEqual(code, 2)
                self.assertEqual(json.loads(stdout.getvalue())["status"], "failed")

    def test_target_files_and_inline_parameters_reject_unknown_or_invalid_fields(self):
        targets = {
            "aprilgrid": {"tagRows": 3, "tagCols": 4, "tagSize": 0.04, "tagSpacing": 0.3},
            "checkerboard": {"targetRows": 6, "targetCols": 7,
                             "rowSpacingMeters": 0.04, "colSpacingMeters": 0.05},
            "circlegrid": {"targetRows": 6, "targetCols": 7,
                           "spacingMeters": 0.04, "asymmetricGrid": False},
        }
        for kind, parameters in targets.items():
            for external in (False, True):
                with self.subTest(kind=kind, external=external):
                    def configure(values):
                        if external:
                            dump_yaml(dict(values, schema_version="1.0.0", kind="calibration_target",
                                           target_type=kind), self.root / "target.yaml")
                            self.task["target"] = {"path": "target.yaml"}
                        else:
                            self.task["target"] = {"type": kind, "parameters": values}
                        self.write_inputs()
                    configure(parameters)
                    self.assertEqual(validate_task(self.task_path)["status"], "passed")
                    configure(dict(parameters, unexpected=1))
                    self.assert_validation_failed("unknown.*unexpected")
                    row = "tagRows" if kind == "aprilgrid" else "targetRows"
                    for invalid in (2, True, 3.0):
                        configure(dict(parameters, **{row: invalid}))
                        self.assert_validation_failed(row)
                    spacing = {"aprilgrid": "tagSize", "checkerboard": "rowSpacingMeters",
                               "circlegrid": "spacingMeters"}[kind]
                    for invalid in (0.0, -0.1, 1, float("nan"), "0.04"):
                        configure(dict(parameters, **{spacing: invalid}))
                        self.assert_validation_failed(spacing)
                    incomplete = dict(parameters)
                    del incomplete[spacing]
                    configure(incomplete)
                    self.assert_validation_failed(spacing)
                    if kind == "circlegrid":
                        configure(dict(parameters, asymmetricGrid="false"))
                        self.assert_validation_failed("asymmetricGrid")

    def test_imu_file_unknown_fields_and_missing_statistics_are_rejected(self):
        self.camera_imu_task()
        path = self.root / "imu.yaml"
        original = load_yaml(path)
        for field, value in (("unexpected", 1), ("gyroscope_noise_density", "0.001"),
                             ("update_rate", False), ("rostopic", " ")):
            with self.subTest(field=field):
                dump_yaml(dict(original, **{field: value}), path)
                self.assert_validation_failed(field)
        del original["accelerometer_random_walk"]
        dump_yaml(original, path)
        self.assert_validation_failed("accelerometer_random_walk")

    def test_camera_result_paths_and_documents_are_strict(self):
        self.camera_imu_task()
        for invalid in ({}, {"path": 123}, {"path": ["calibration.yaml"]},
                        {"path": "calibration.yaml", "unexpected": True}, " "):
            with self.subTest(path=invalid):
                self.task["camera_calibration"] = invalid
                self.write_inputs()
                self.assert_validation_failed("camera_calibration|unknown")
                with self.assertRaises(TaskError):
                    load_cameras(dict(self.task, _config_dir=str(self.root)))
        self.task["camera_calibration"] = {"path": "calibration.yaml"}
        self.write_inputs()
        path = self.root / "calibration.yaml"
        original = load_yaml(path)
        for scope, field, invalid in (
                ("root", "unexpected", True), ("root", "calibration_type", []),
                ("root", "transform_convention", "p_source = T * p_target"),
                ("root", "imus", {}), ("camera", "unexpected", True),
                ("camera", "camera_model", []), ("camera", "distortion_model", {}),
                ("camera", "cam_overlaps", [True]), ("camera", "cam_overlaps", [1]),
                ("camera", "timeshift_cam_imu", float("inf")), ("camera", "line_delay", "0")):
            with self.subTest(scope=scope, field=field):
                document = copy.deepcopy(original)
                destination = document if scope == "root" else document["cameras"][0]
                destination[field] = invalid
                dump_yaml(document, path)
                self.assert_validation_failed(field)
        valid = copy.deepcopy(original)
        valid["cameras"][0].update(cam_overlaps=[], T_cam_imu=np.eye(4).tolist(),
                                   timeshift_cam_imu=0.001, line_delay=0.0)
        dump_yaml(valid, path)
        self.assertEqual(validate_task(self.task_path)["status"], "passed")
        duplicate = dict(valid["cameras"][0], id="cam1", T_cn_cnm1=np.eye(4).tolist())
        valid["cameras"].append(duplicate)
        dump_yaml(valid, path)
        self.assert_validation_failed("rostopic.*unique")

    def test_camera_ids_select_one_named_directory_camera_and_reject_unsafe_ids(self):
        self.manifest["cameras"][0]["id"] = "cam1"
        self.task["cameras"][0]["id"] = "cam1"
        self.write_inputs()
        self.assertEqual(validate_task(self.task_path)["status"], "passed")
        loaded = load_cameras(dict(self.task, _config_dir=str(self.root)))
        self.assertEqual([camera["id"] for camera in loaded], ["cam1"])
        for identifier in ("", "../cam1", "cam/1", "cam.1", True):
            with self.subTest(identifier=identifier):
                task = copy.deepcopy(self.task)
                task["cameras"][0]["id"] = identifier
                with self.assertRaisesRegex(TaskError, "safe camera ID"):
                    validate_options(task)
        task = copy.deepcopy(self.task)
        task["cameras"].append(dict(task["cameras"][0], topic="/other"))
        with self.assertRaisesRegex(TaskError, "unique safe camera ID"):
            validate_options(task)

    def test_camera_result_keeps_real_ids_and_validates_quality_summaries(self):
        self.camera_imu_task()
        path = self.root / "calibration.yaml"
        result = load_yaml(path)
        first = result["cameras"][0]
        first.update(id="right", rms={"status": "available", "value": .25, "unit": "px", "count": 16})
        second = copy.deepcopy(first)
        second.update(id="left", rostopic="/cam1/image_raw", T_cn_cnm1=np.eye(4).tolist(),
                      from_camera="right", alignment={"status": "unavailable", "reason": "no paired corners"})
        result["cameras"].append(second)
        dump_yaml(result, path)
        task = dict(self.task, dataset={"type": "bag", "path": "unused.bag"}, _config_dir=str(self.root))
        self.assertEqual([camera["id"] for camera in load_cameras(task)], ["right", "left"])
        for field, value in (("id", "right"), ("from_camera", "missing"),
                             ("rms", {"status": "available", "value": float("nan")}),
                             ("alignment", {"status": "available", "rms": "0.1"}),
                             ("alignment", {"status": "available", "count": True})):
            with self.subTest(field=field):
                invalid = copy.deepcopy(result)
                invalid["cameras"][1][field] = value
                dump_yaml(invalid, path)
                with self.assertRaises(TaskError):
                    load_cameras(task)

    def test_camera_imu_without_result_path_needs_run_output_context(self):
        self.camera_imu_task()
        del self.task["camera_calibration"]
        validate_options(self.task)
        with self.assertRaisesRegex(TaskError, "standalone validation.*output directory"):
            load_cameras(self.task)

    def test_run_task_rejects_bad_image_before_solver_and_records_input_failure(self):
        (self.dataset / "cam0/images/1.png").write_bytes(b"not an encoded image")
        self.task["output"] = {"save_diagnostics": True}
        self.write_inputs()
        output = self.root / "bad_image_run"
        with mock.patch("kalibr_no_ros.task._run_legacy") as solver:
            with self.assertRaisesRegex(TaskError, "input validation failed"):
                run_task(self.root / "unused_prefix", self.task_path, output, "camera_calibration")
            solver.assert_not_called()
        diagnostics = output / "camera_calibration_cam0_failed"
        validation = json.loads((diagnostics / "validation.json").read_text(encoding="utf-8"))
        manifest = json.loads((diagnostics / "run_manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(validation["status"], "failed")
        self.assertEqual(manifest["status"], "input_failed")
        self.assertTrue((diagnostics / "task_resolved.yaml").is_file())
        self.assertFalse((output / "camera_calibration_cam0.yaml").exists())

    def test_camera_reader_preserves_all_supported_models_and_native_optional_fields(self):
        self.camera_imu_task()
        models = (
            ("pinhole", "radtan", [7., 7., 5., 4.], 4),
            ("pinhole", "radtan5", [7., 7., 5., 4.], 5),
            ("pinhole", "radtan8", [7., 7., 5., 4.], 8),
            ("pinhole", "equidistant", [7., 7., 5., 4.], 4),
            ("pinhole", "fov", [7., 7., 5., 4.], 1),
            ("pinhole_opencv_fisheye", "opencv_fisheye", [7., 7., 5., 4., 0.027123456789012345], 4),
            ("omni", "none", [0.5, 7., 7., 5., 4.], 0),
            ("omni", "radtan", [0.5, 7., 7., 5., 4.], 4),
            ("eucm", "none", [0.5, 1.0, 7., 7., 5., 4.], 0),
            ("ds", "none", [0.5, 0.5, 7., 7., 5., 4.], 0),
        )
        path = self.root / "calibration.yaml"
        for projection, distortion, intrinsics, count in models:
            with self.subTest(projection=projection, distortion=distortion):
                camera = {
                    "id": "cam0", "rostopic": "/cam0/image_raw", "camera_model": projection,
                    "distortion_model": distortion, "intrinsics": intrinsics,
                    "distortion_coeffs": [0.0012345678901234567] * count, "resolution": [10, 8],
                    "cam_overlaps": [], "T_cam_imu": np.eye(4).tolist(),
                    "timeshift_cam_imu": -0.0012345678901234567, "line_delay": 0.00001,
                }
                dump_yaml({"schema_version": "1.0.0", "kind": "calibration_result",
                           "calibration_type": "camera_imu", "cameras": [camera], "imus": []}, path)
                actual = load_cameras(load_task(self.task_path))[0]
                self.assertEqual(actual, dict(camera, topic=camera["rostopic"]))

    def test_solver_logs_capture_python_and_native_writes_and_restore_after_exception(self):
        output = self.root / "logs"
        output.mkdir()
        script = """
import os
from pathlib import Path
import sys
from kalibr_no_ros.task import _capture_solver_logs
before_streams = (sys.stdout, sys.stderr)
before_fds = [(os.fstat(fd).st_dev, os.fstat(fd).st_ino) for fd in (1, 2)]
try:
    with _capture_solver_logs(Path(sys.argv[1])):
        print('python stdout inside')
        print('python stderr inside', file=sys.stderr)
        os.write(1, b'native stdout inside\\n')
        os.write(2, b'native stderr inside\\n')
        raise RuntimeError('capture sentinel failure')
except RuntimeError:
    pass
assert (sys.stdout, sys.stderr) == before_streams
assert [(os.fstat(fd).st_dev, os.fstat(fd).st_ino) for fd in (1, 2)] == before_fds
print('python stdout restored')
print('python stderr restored', file=sys.stderr)
os.write(1, b'native stdout restored\\n')
os.write(2, b'native stderr restored\\n')
"""
        environment = dict(os.environ)
        environment["PYTHONPATH"] = str(ROOT / "src/python") + os.pathsep + environment.get("PYTHONPATH", "")
        completed = subprocess.run([sys.executable, "-c", script, str(output)], env=environment,
                                   text=True, capture_output=True, timeout=30)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        for name, outer in (("stdout", completed.stdout), ("stderr", completed.stderr)):
            with self.subTest(stream=name):
                saved = (output / (name + ".log")).read_text(encoding="utf-8")
                for source in ("python", "native"):
                    self.assertIn(source + " " + name + " inside", saved)
                    self.assertIn(source + " " + name + " restored", outer)
                self.assertNotIn("restored", saved)
                self.assertNotIn("inside", outer)
        self.assertIn("RuntimeError: capture sentinel failure",
                      (output / "stderr.log").read_text(encoding="utf-8"))

    def test_sequential_runpy_tasks_restore_outer_context_arguments_and_directory(self):
        prefix = self.root / "prefix"
        script = prefix / "libexec/kalibr/kalibr_calibrate_cameras"
        script.parent.mkdir(parents=True)
        script.write_text(
            "from pathlib import Path\n"
            "from kalibr_no_ros.artifacts import current_context\n"
            "from kalibr_no_ros.task import dump_yaml\n"
            "context = current_context()\n"
            "assert context is not None and context.artifacts['events'] == []\n"
            "context.artifacts['events'].append({'native_run': True})\n"
            "context.artifacts['state'] = 'completed'\n"
            "dump_yaml({'cam0': {'camera_model': 'pinhole', 'distortion_model': 'equidistant', "
            "'intrinsics': [7., 7., 5., 4.], 'distortion_coeffs': [0., 0., 0., 0.], "
            "'resolution': [10, 8], 'rostopic': '/cam0/image_raw'}}, Path('fixture-camchain.yaml'))\n"
            "Path('fixture-results-cam.txt').write_text('Completed native fixture.\\n')\n",
            encoding="utf-8")
        previous_arguments, previous_directory = sys.argv, Path.cwd()
        with artifacts.run_context("outer") as outer:
            outer.artifacts["events"].append({"owner": "outer"})
            summary = {"metrics": {"cameras": {"cam0": {"reprojection": {
                "status": "unavailable", "count": 0, "rms_px": None,
            }}}}}
            with mock.patch.object(reporting, "generate_report", return_value=summary) as generate:
                for index in range(2):
                    output = self.root / "output{}".format(index)
                    run_task(prefix, self.task_path, output, "camera_calibration")
                    self.assertIs(artifacts.current_context(), outer)
                    self.assertIs(sys.argv, previous_arguments)
                    self.assertEqual(Path.cwd(), previous_directory)
                    self.assertEqual(load_yaml(output / "camera_calibration_cam0.yaml")["schema_version"], "1.0.0")
                self.assertEqual(generate.call_count, 2)
            self.assertEqual(outer.artifacts["events"], [{"owner": "outer"}])
        self.assertIsNone(artifacts.current_context())


if __name__ == "__main__":
    unittest.main()
