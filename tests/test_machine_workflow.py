"""Machine-level staged calibration orchestration without invoking a solver."""

import json
import ast
from copy import deepcopy
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest import mock

from kalibr_no_ros import machine_workflow
from kalibr_no_ros import machine_cli
from kalibr_no_ros.machine_workflow import (
    build_task,
    discover_dataset,
    load_workflow_config,
    run_machine_workflow,
)
from kalibr_no_ros.task import TaskError, dump_yaml, load_yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "config" / "mechine" / "workflow_v1.0.0"
BASELINK_CONFIG = {
    "base_link": {"position_scale_m": 0.01},
    "cameras": [
        {"id": "cam0", "translation_base_camera_m": [0.1, 0.2, 0.3]},
        {"id": "cam1", "translation_base_camera_m": [0.2, 0.2, 0.3]},
    ],
}
MONO_BASELINK_CONFIG = {
    "base_link": {"position_scale_m": 0.001},
    "cameras": [
        {"id": "cam0", "translation_base_camera_m": [-0.1, 0.05, 0.3]},
    ],
}


class MachineWorkflowTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.machine_root = self.root / "EVT312-02"
        self.machine = self.machine_root / "1029"
        self.machine.mkdir(parents=True)
        self.prefix = self.root / "prefix"
        installed = self.prefix / "config" / "mechine"
        installed.parent.mkdir(parents=True)
        # The test installation is independent from build/install outputs.
        shutil.copytree(ROOT / "config" / "mechine", installed)
        shutil.copy2(
            installed / "workflow_v1.0.0" / "测试场_aprilgrid_8x8.yaml",
            installed / "aprilgrid_1.yaml")

    def test_native_config_printers_resolve_stdout_at_call_time(self):
        from kalibr_camera_calibration.CameraUtils import printParameters
        from kalibr_common.ConfigReader import (
            CalibrationTargetParameters,
            CameraChainParameters,
            CameraParameters,
            ImuParameters,
        )
        for config_type in (
                CameraParameters, ImuParameters,
                CalibrationTargetParameters, CameraChainParameters):
            self.assertEqual(config_type.printDetails.__defaults__, (None,))
        self.assertEqual(printParameters.__defaults__, (None,))
        for path in (ROOT / "src").rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                defaults = list(node.args.defaults) + [
                    value for value in node.args.kw_defaults if value is not None]
                for default in defaults:
                    with self.subTest(path=path, function=node.name):
                        self.assertFalse(
                            isinstance(default, ast.Attribute)
                            and isinstance(default.value, ast.Name)
                            and default.value.id == "sys"
                            and default.attr in {"stdout", "stderr"},
                            "bind stdout/stderr at call time, not import time")

    def write_dataset(self, name, *, imu=False, cameras=("cam0", "cam1")):
        dataset = self.machine / name
        camera_entries = []
        for camera_id in cameras:
            image_dir = dataset / camera_id
            image_dir.mkdir(parents=True)
            timestamps = dataset / (camera_id + ".csv")
            timestamps.write_text("timestamp_ns,filename\n", encoding="utf-8")
            camera_entries.append({
                "id": camera_id,
                "images": camera_id,
                "timestamps": timestamps.name,
            })
        imu_entries = []
        if imu:
            table = dataset / "imu0.csv"
            table.write_text(
                "timestamp_ns,wx,wy,wz,ax,ay,az\n", encoding="utf-8")
            imu_entries.append({"id": "imu0", "data": table.name})
        dump_yaml({
            "schema_version": "1.0.0",
            "type": "kalibr_directory_dataset",
            "dataset_id": name,
            "cameras": camera_entries,
            "imus": imu_entries,
        }, dataset / "dataset.yaml")
        return dataset

    def write_config(self, *, stereo=None, stereo_imu=None,
                     mode="standard", camera_mode=None,
                     recompute_extrinsics=False, target=None,
                     extra=None):
        document = {
            "schema_version": "1.0.0",
            "kind": "machine_calibration_workflow",
            "machine_dir": str(self.machine),
            "camera_imu": {
                "mode": mode,
                "recompute_extrinsics": recompute_extrinsics,
            },
        }
        if camera_mode is not None:
            document["camera"] = {"mode": camera_mode}
        if target is not None:
            document["target"] = target
        datasets = {}
        if stereo is not None:
            datasets["stereo"] = stereo
        if stereo_imu is not None:
            datasets["stereo_imu"] = stereo_imu
        if datasets:
            document["datasets"] = datasets
        if extra:
            document.update(extra)
        path = self.root / "user.yaml"
        dump_yaml(document, path)
        return path

    def profile_file(self, name):
        return (self.prefix / "config" / "mechine"
                / "workflow_v1.0.0" / name)

    def test_workflow_yaml_is_strict_and_resolves_machine_relative_to_config(self):
        relative_machine = self.root / "EVT312-03" / "run_01"
        relative_machine.mkdir(parents=True)
        path = self.root / "relative.yaml"
        dump_yaml({
            "schema_version": "1.0.0",
            "kind": "machine_calibration_workflow",
            "machine_dir": "EVT312-03/run_01",
            "target": "aprilgrid_1",
            "camera": {"mode": "rolling_shutter"},
            "camera_imu": {"mode": "rolling_shutter"},
            "datasets": {"stereo": "stereo_260916_2136"},
        }, path)
        loaded = load_workflow_config(path)
        self.assertEqual(Path(loaded["machine_dir"]), relative_machine.resolve())
        self.assertEqual(loaded["camera_imu"]["mode"], "rolling_shutter")
        self.assertEqual(loaded["camera"]["mode"], "rolling_shutter")
        self.assertEqual(loaded["target"], "aprilgrid_1")
        self.assertEqual(Path(loaded["machine_root"]), relative_machine.parent)
        self.assertEqual(loaded["batch_name"], "run_01")
        self.assertEqual(loaded["machine_version"], "EVT312")
        self.assertEqual(loaded["machine_num"], "03")

        invalid_documents = [
            ({"schema_version": "1", "kind": "machine_calibration_workflow",
              "machine_dir": str(self.machine), "camera_imu": {"mode": "standard"}},
             "schema_version"),
            ({"schema_version": "1.0.0", "kind": "calibration_task",
              "machine_dir": str(self.machine), "camera_imu": {"mode": "standard"}},
             "kind"),
            ({"schema_version": "1.0.0", "kind": "machine_calibration_workflow",
              "machine_dir": str(self.machine)}, "camera_imu"),
            ({"schema_version": "1.0.0", "kind": "machine_calibration_workflow",
              "machine_dir": str(self.machine), "camera_imu": {"mode": "auto"}},
             "mode"),
            ({"schema_version": "1.0.0", "kind": "machine_calibration_workflow",
              "machine_dir": str(self.machine), "camera": {"mode": "auto"},
              "camera_imu": {"mode": "standard"}}, "camera.mode"),
            ({"schema_version": "1.0.0", "kind": "machine_calibration_workflow",
              "machine_dir": str(self.machine), "target": "../target",
              "camera_imu": {"mode": "standard"}}, "target"),
            ({"schema_version": "1.0.0", "kind": "machine_calibration_workflow",
              "machine_dir": str(self.machine),
              "camera_imu": {"mode": "standard", "recompute_extrinsics": 1}},
             "recompute_extrinsics"),
            ({"schema_version": "1.0.0", "kind": "machine_calibration_workflow",
              "machine_dir": str(self.machine), "camera_imu": {"mode": "standard"},
              "revision": "../escape"}, "revision"),
            ({"schema_version": "1.0.0", "kind": "machine_calibration_workflow",
              "machine_dir": str(self.machine), "camera_imu": {"mode": "standard"},
              "templates": "private"}, "unknown"),
        ]
        for index, (document, message) in enumerate(invalid_documents):
            with self.subTest(index=index):
                invalid = self.root / ("invalid_{}.yaml".format(index))
                dump_yaml(document, invalid)
                with self.assertRaisesRegex((TaskError, ValueError), message):
                    load_workflow_config(invalid)

        invalid_machine = self.root / "mechine-EVT31202" / "1029"
        invalid_machine.mkdir(parents=True)
        with self.assertRaisesRegex(TaskError, "machine_dir basename"):
            load_workflow_config(self.write_config(extra={
                "machine_dir": str(invalid_machine),
            }))

        invalid_batch = self.machine_root / "bad batch"
        invalid_batch.mkdir()
        with self.assertRaisesRegex(TaskError, "batch name"):
            load_workflow_config(self.write_config(extra={
                "machine_dir": str(invalid_batch),
            }))

    def test_dataset_discovery_is_direct_strict_and_role_aware(self):
        stereo = self.write_dataset("stereo_260916_2136")
        stereo_imu = self.write_dataset("stereo_imu_260916_2200", imu=True)
        self.write_dataset("stereo_test_260916_2211")
        nested = self.machine / "archive"
        nested.mkdir()
        (nested / "stereo_260916_9999").mkdir()
        self.assertEqual(discover_dataset(self.machine, "stereo"), stereo.resolve())
        self.assertEqual(
            discover_dataset(self.machine, "stereo_imu"), stereo_imu.resolve())
        self.assertEqual(
            discover_dataset(self.machine, "stereo", stereo.name), stereo.resolve())

        another = self.write_dataset("stereo_260916_2152")
        with self.assertRaisesRegex((TaskError, ValueError),
                                    "2136.*2152|2152.*2136"):
            discover_dataset(self.machine, "stereo")
        self.assertEqual(
            discover_dataset(self.machine, "stereo", another.name),
            another.resolve())
        with self.assertRaisesRegex((TaskError, ValueError), "stereo_test|name|selected"):
            discover_dataset(self.machine, "stereo", "stereo_test_260916_2211")

    def test_workflow_preflight_rejects_duplicate_datasets_before_runner(self):
        self.write_dataset("stereo_260916_2136")
        self.write_dataset("stereo_260916_2152")
        calls = []
        with self.assertRaisesRegex(
                TaskError, "multiple stereo datasets.*2136.*2152"):
            run_machine_workflow(
                self.prefix, self.write_config(),
                runner=self._mock_runner(calls))
        self.assertEqual(calls, [])

    def test_workflow_ignores_noncanonical_dataset_directory_names(self):
        self.write_dataset(
            "usb_baselink_261027_1954", cameras=("cam0",))
        self.write_dataset("stereo_base_link_230320_2359_nomal")
        self.write_dataset("usb_261027_1949_bk", cameras=("cam0",))
        calls = []
        summary = run_machine_workflow(
            self.prefix, self.write_config(),
            runner=self._mock_runner(calls))
        self.assertEqual(calls, [])
        self.assertEqual(summary["camera"]["status"], "waiting_for_dataset")
        self.assertEqual(
            summary["mono_camera"]["status"], "waiting_for_dataset")

    def test_partial_dataset_selection_still_auto_discovers_usb(self):
        stereo = self.write_dataset("stereo_260916_2136")
        mono = self.write_dataset("usb_261027_1949", cameras=("cam0",))
        calls = []
        run_machine_workflow(
            self.prefix,
            self.write_config(extra={
                "datasets": {"stereo": stereo.name},
            }),
            runner=self._mock_runner(calls))
        self.assertEqual(
            [Path(call["task"]["dataset"]["path"]) for call in calls],
            [stereo.resolve(), mono.resolve()])

    def test_named_target_must_exist_in_installed_machine_config(self):
        self.write_dataset("stereo_260916_2136")
        with self.assertRaisesRegex(TaskError, "target.*missing|missing.*target"):
            run_machine_workflow(
                self.prefix, self.write_config(target="not_installed"),
                runner=self._mock_runner([]))

    def test_discovery_validates_sensor_composition_and_rejects_symlink(self):
        wrong_camera = self.write_dataset(
            "stereo_260916_2136", cameras=("cam0", "cam2"))
        with self.assertRaisesRegex((TaskError, ValueError), "cam0.*cam1|camera"):
            discover_dataset(self.machine, "stereo", wrong_camera.name)
        shutil.rmtree(wrong_camera)
        wrong_imu = self.write_dataset("stereo_imu_260916_2200", imu=False)
        with self.assertRaisesRegex((TaskError, ValueError), "imu0|IMU"):
            discover_dataset(self.machine, "stereo_imu", wrong_imu.name)
        shutil.rmtree(wrong_imu)
        external = self.root / "external"
        external.mkdir()
        (self.machine / "stereo_260916_2136").symlink_to(external,
                                                          target_is_directory=True)
        with self.assertRaisesRegex((TaskError, ValueError), "symlink"):
            discover_dataset(self.machine, "stereo", "stereo_260916_2136")

    def test_dataset_manifest_and_workflow_parents_must_not_be_symlinks(self):
        dataset = self.write_dataset("stereo_260916_2136")
        manifest = dataset / "dataset.yaml"
        external_manifest = self.root / "external_dataset.yaml"
        manifest.replace(external_manifest)
        manifest.symlink_to(external_manifest)
        with self.assertRaisesRegex((TaskError, ValueError), "symlink"):
            discover_dataset(self.machine, "stereo", dataset.name)

        manifest.unlink()
        external_manifest.replace(manifest)
        control = self.machine / "result" / ".kalibr-cedarobot"
        control.mkdir(parents=True)
        external_tasks = self.root / "external_tasks"
        external_tasks.mkdir()
        (control / "tasks").symlink_to(external_tasks, target_is_directory=True)
        calls = []
        with self.assertRaisesRegex((TaskError, ValueError), "symlink"):
            run_machine_workflow(
                self.prefix, self.write_config(),
                runner=self._mock_runner(calls))
        self.assertEqual(calls, [])

    def test_build_task_uses_absolute_inputs_and_exact_camera_dependency(self):
        stereo = self.write_dataset("stereo_260916_2136")
        camera = build_task(
            PROFILE, "camera", stereo, "stereo_260916_2136_pinhole_equi")
        self.assertEqual(camera["job"], "camera_calibration")
        self.assertEqual(camera["dataset"]["path"], str(stereo.resolve()))
        self.assertEqual(camera["output"]["name"],
                         "stereo_260916_2136_pinhole_equi")
        self.assertEqual(Path(camera["target"]["path"]),
                         (PROFILE / "测试场_aprilgrid_8x8.yaml").resolve())
        self.assertEqual(Path(camera["initialization"]["path"]),
                         (PROFILE / "initialization_stereo_camera.yaml").resolve())
        self.assertTrue(camera["output"]["save_metrics"])
        rolling_camera = build_task(
            PROFILE, "camera", stereo,
            "stereo_260916_2136_rs_pinhole_equi",
            camera_mode="rolling_shutter",
            target_path=(self.prefix / "config" / "mechine"
                         / "aprilgrid_1.yaml"))
        self.assertEqual(
            rolling_camera["job"], "camera_rolling_shutter_calibration")
        self.assertEqual(set(rolling_camera["rolling_shutter"]), {"cam0", "cam1"})
        self.assertEqual(
            Path(rolling_camera["target"]["path"]),
            (self.prefix / "config" / "mechine"
             / "aprilgrid_1.yaml").resolve())

        mono_dataset = self.write_dataset(
            "usb_260916_2137", cameras=("cam0",))
        mono = build_task(
            PROFILE, "mono_camera", mono_dataset, mono_dataset.name)
        self.assertEqual(
            Path(mono["initialization"]["path"]),
            (PROFILE / "initialization_mono_camera.yaml").resolve())

        imu_dataset = self.write_dataset("stereo_imu_260916_2200", imu=True)
        exact_result = self.machine / "result" / "selected_camera.yaml"
        standard = build_task(
            PROFILE, "camera_imu", imu_dataset, "standard_result",
            camera_result=exact_result, mode="standard")
        rolling = build_task(
            PROFILE, "camera_imu", imu_dataset, "rolling_result",
            camera_result=exact_result, mode="rolling_shutter",
            recompute_extrinsics=True)
        self.assertEqual(standard["job"], "camera_imu_calibration")
        self.assertEqual(
            rolling["job"], "camera_imu_rolling_shutter_calibration")
        self.assertIs(
            rolling["calibration"]["recompute_camera_chain_extrinsics"], True)
        for task in (standard, rolling):
            self.assertEqual(task["camera_calibration"]["path"],
                             str(exact_result.resolve()))
            self.assertEqual(Path(task["imus"][0]["path"]),
                             (PROFILE / "imu_noise.yaml").resolve())
            self.assertNotIn("__DATASET_PATH__", json.dumps(task))
            self.assertTrue(task["output"]["save_diagnostics"])
            self.assertTrue(task["output"]["save_metrics"])

    def test_task_auxiliary_paths_are_relative_and_confined_to_machine_config(self):
        stereo = self.write_dataset("stereo_260916_2136")
        machine_config = self.prefix / "config" / "mechine"
        profile = machine_config / "workflow_v1.0.0"
        shutil.copy2(profile / "实验室_aprilgrid_8x10.yaml",
                     machine_config / "custom_target.yaml")
        shutil.copy2(profile / "initialization_stereo_camera.yaml",
                     machine_config / "custom_initialization.yaml")
        template = profile / "stereo_camera_calibration_task.yaml"
        document = load_yaml(template)
        document["target"]["path"] = "../custom_target.yaml"
        document["initialization"]["path"] = "../custom_initialization.yaml"
        dump_yaml(document, template)

        task = build_task(profile, "camera", stereo, stereo.name)
        self.assertEqual(Path(task["target"]["path"]),
                         (machine_config / "custom_target.yaml").resolve())
        self.assertEqual(
            Path(task["initialization"]["path"]),
            (machine_config / "custom_initialization.yaml").resolve())

        document["initialization"]["path"] = "../../outside.yaml"
        dump_yaml(document, template)
        with self.assertRaisesRegex(TaskError, "must stay inside"):
            build_task(profile, "camera", stereo, stereo.name)

    def _mock_runner(self, calls):
        def run(prefix, config, output_dir, expected_job, force=False, **overrides):
            task = load_yaml(config)
            calls.append({
                "prefix": Path(prefix),
                "config": Path(config),
                "output": Path(output_dir),
                "job": expected_job,
                "task": task,
                "force": force,
                "overrides": overrides,
            })
            name = task["output"]["name"]
            output = Path(output_dir)
            output.mkdir(parents=True, exist_ok=True)
            if force:
                side = output / name
                if side.is_dir():
                    shutil.rmtree(side)
                for suffix in (".report.html", ".report.pdf"):
                    path = output / (name + suffix)
                    if path.is_file():
                        path.unlink()
            result = {
                "schema_version": "1.0.0",
                "kind": "calibration_result",
                "calibration_type": (
                    "cameras" if expected_job in {
                        "camera_calibration",
                        "camera_rolling_shutter_calibration",
                    }
                    else "cameras_baselink" if expected_job == "camera_baselink_calibration"
                    else "camera_imu"),
                "cameras": [
                    {"id": "cam0", "intrinsics": [1.0, 1.0, 1.0, 1.0],
                     "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                     "camera_model": "pinhole",
                     "distortion_model": "equidistant",
                     "resolution": [640, 480], "rms": 0.2},
                    {"id": "cam1", "intrinsics": [1.0, 1.0, 1.0, 1.0],
                     "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                     "camera_model": "pinhole",
                     "distortion_model": "equidistant",
                     "resolution": [640, 480], "rms": 0.3,
                     "alignment": 0.4,
                     "T_cn_cnm1": [[1.0, 0.0, 0.0, -0.1],
                                      [0.0, 1.0, 0.0, 0.0],
                                      [0.0, 0.0, 1.0, 0.0],
                                      [0.0, 0.0, 0.0, 1.0]]},
                ],
            }
            if "cameras" in task:
                camera_ids = [camera["id"] for camera in task["cameras"]]
                result["cameras"] = [camera for camera in result["cameras"]
                                     if camera["id"] in camera_ids]
            identity = [[1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0]]
            if expected_job == "camera_baselink_calibration":
                for camera in result["cameras"]:
                    camera["T_base_camera"] = [
                        [1.0, 0.0, 0.0, 0.2],
                        [0.0, 1.0, 0.0, 0.1],
                        [0.0, 0.0, 1.0, 0.3],
                        [0.0, 0.0, 0.0, 1.0],
                    ]
                    camera["pitch"] = 8.0
            if expected_job in {"camera_imu_calibration", "camera_imu_rolling_shutter_calibration"}:
                for index, camera in enumerate(result["cameras"]):
                    camera["T_imu_cam"] = identity
                    camera["timeshift_cam_imu"] = 0.01 + index * 0.001
                    if expected_job == "camera_imu_rolling_shutter_calibration":
                        camera["shutter"] = {"line_delay_s": 1e-5 + index * 1e-6}
                result["imus"] = [{"id": "imu0", "update_rate": 100.0}]
            elif expected_job == "camera_rolling_shutter_calibration":
                for index, camera in enumerate(result["cameras"]):
                    camera["shutter"] = {"line_delay_s": 1e-5 + index * 1e-6}
            side = output / name
            side.mkdir()
            dump_yaml(result, side / (name + ".yaml"))
            (output / (name + ".report.html")).write_text(
                "<html></html>", encoding="utf-8")
            (output / (name + ".report.pdf")).write_bytes(b"%PDF fixture")
            delivery_paths = [
                side / (name + ".yaml"),
                output / (name + ".report.html"),
                output / (name + ".report.pdf"),
            ]
            resolved_path = side / "task_resolved.yaml"
            dump_yaml(
                machine_workflow._resolved_task(config, expected_job),
                resolved_path)
            dataset_manifest = load_yaml(
                Path(task["dataset"]["path"]) / "dataset.yaml")
            input_documents = []
            resolved_document = load_yaml(resolved_path)
            inputs = [
                (role, resolved_document.get(role))
                for role in ("target", "camera_calibration", "initialization")
            ]
            inputs += [
                ("imu{}".format(index), block)
                for index, block in enumerate(resolved_document.get("imus", []))
            ]
            for role, block in inputs:
                if not isinstance(block, dict) or not block.get("path"):
                    continue
                source = block["path"]
                input_documents.append({
                    "role": role,
                    "path": source,
                    "sha256": machine_workflow._sha256(source),
                    "document": load_yaml(source),
                })
            if expected_job in {
                    "camera_calibration",
                    "camera_rolling_shutter_calibration",
                    "camera_imu_calibration",
                    "camera_imu_rolling_shutter_calibration"}:
                metrics = {
                    "schema_version": "1.0.0",
                    "kind": "calibration_metrics",
                    "calibration_type": result["calibration_type"],
                    "evaluation_options": {"rectification": {
                        "balance": 0.0,
                        "fov_scale": 1.0,
                        "size": None,
                    }},
                }
                if expected_job in {
                        "camera_imu_calibration",
                        "camera_imu_rolling_shutter_calibration"}:
                    metrics["imus"] = {"imu0": {
                        "gyro": {"bias_spline": {"status": "available", "axes": {
                            axis: {"status": "available", "mean": value}
                            for axis, value in zip(
                                ("x", "y", "z"), (0.001, 0.002, 0.003))}}},
                        "accel": {"bias_spline": {"status": "available", "axes": {
                            axis: {"status": "available", "mean": value}
                            for axis, value in zip(
                                ("x", "y", "z"), (0.01, 0.02, 0.03))}}},
                    }}
                (side / "metrics.json").write_text(
                    json.dumps(metrics), encoding="utf-8")
            (side / "run_manifest.json").write_text(json.dumps({
                "schema_version": "1.0.0",
                "kind": "calibration_run",
                "status": "completed",
                "job": expected_job,
                "dataset": {
                    "path": task["dataset"]["path"],
                    "id": dataset_manifest["dataset_id"],
                },
                "task_sha256": machine_workflow._sha256(resolved_path),
                "input_documents": input_documents,
                "files": [{
                    "path": "task_resolved.yaml",
                    "size_bytes": resolved_path.stat().st_size,
                    "sha256": machine_workflow._sha256(resolved_path),
                    "schema_version": "1.0.0",
                }] + ([{
                    "path": "metrics.json",
                    "size_bytes": (side / "metrics.json").stat().st_size,
                    "sha256": machine_workflow._sha256(side / "metrics.json"),
                    "schema_version": "1.0.0",
                }] if (side / "metrics.json").is_file() else []),
                "directories": [],
                "delivery_files": [{
                    "path": path.name,
                    "size_bytes": path.stat().st_size,
                    "sha256": machine_workflow._sha256(path),
                    "schema_version": "1.0.0",
                } for path in delivery_paths],
            }), encoding="utf-8")
            (side / ".inventory.json").write_text(json.dumps(sorted([
                name + ".yaml", "run_manifest.json", "task_resolved.yaml"
            ]
                + (["metrics.json"] if (side / "metrics.json").is_file() else []))),
                encoding="utf-8")
            return output
        return run

    def test_stereo_then_later_imu_reuses_camera_and_injects_exact_result(self):
        stereo = self.write_dataset("stereo_260916_2136")
        config = self.write_config()
        calls = []
        first = run_machine_workflow(
            self.prefix, config, runner=self._mock_runner(calls))
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]["job"], "camera_calibration")
        self.assertEqual(Path(calls[0]["task"]["dataset"]["path"]), stereo)
        self.assertIn("waiting_for_dataset", json.dumps(first, default=str))

        imu = self.write_dataset("stereo_imu_260916_2200", imu=True)
        second = run_machine_workflow(
            self.prefix, config, runner=self._mock_runner(calls))
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[1]["job"], "camera_imu_calibration")
        self.assertEqual(Path(calls[1]["task"]["dataset"]["path"]), imu)
        expected_camera = (
            self.machine / "result"
            / "stereo_260916_2136"
            / "stereo_260916_2136.yaml"
        ).resolve()
        self.assertEqual(
            Path(calls[1]["task"]["camera_calibration"]["path"]),
            expected_camera)
        self.assertIn("reused", json.dumps(second, default=str))

        third = run_machine_workflow(
            self.prefix, config, runner=self._mock_runner(calls))
        self.assertEqual(len(calls), 2)
        self.assertGreater((self.machine / "result" / ".kalibr-cedarobot"
                            / "state.json").stat().st_size, 0)
        self.assertIn("reused", json.dumps(third, default=str))

        self.write_dataset("stereo_base_link_261021_1630")
        with_baselink = run_machine_workflow(
            self.prefix,
            self.write_config(extra={"camera_baselink": BASELINK_CONFIG}),
            runner=self._mock_runner(calls))
        self.assertEqual(len(calls), 3)
        vision = load_yaml(with_baselink["calib_params"]["vision"]["result"])
        self.assertEqual(vision["imu"]["gyro_bias"], [0.001, 0.002, 0.003])
        self.assertEqual(vision["camera"]["cam0"]["T_imu_cam"][0][0], 1.0)
        self.assertEqual(vision["camera"]["cam0"]["T_base_cam"][0][3], 0.2)

    def test_imu_without_stereo_fails_preflight_before_runner(self):
        self.write_dataset("stereo_imu_260916_2200", imu=True)
        config = self.write_config()
        calls = []
        with self.assertRaisesRegex(
                TaskError, "stereo_imu.*requires a stereo"):
            run_machine_workflow(
                self.prefix, config, runner=self._mock_runner(calls))
        self.assertEqual(calls, [])

    def test_baselink_config_uses_the_low_level_validation_contract(self):
        config = self.write_config(extra={"camera_baselink": BASELINK_CONFIG})
        loaded = load_workflow_config(config)
        self.assertEqual(loaded["camera_baselink"]["base_link"]["frame_id"], "base_link")
        for update, message in (
                ({"position_scale_m": 0}, "position_scale_m"),
                ({"position_scale_m": True}, "position_scale_m"),
                ({"position_scale_m": float("nan")}, "position_scale_m"),
                ({"frame_id": "../base"}, "frame_id"),
                ({"alpha": 0}, "unknown")):
            block = deepcopy(BASELINK_CONFIG)
            block["base_link"].update(update)
            with self.subTest(update=update):
                with self.assertRaisesRegex(TaskError, message):
                    load_workflow_config(self.write_config(extra={"camera_baselink": block}))
        for block in (None, {}, {"base_link": {}, "cameras": []},
                      {**BASELINK_CONFIG, "alpha": 0}):
            with self.subTest(block=block), self.assertRaises(TaskError):
                load_workflow_config(self.write_config(extra={"camera_baselink": block}))
        for translation in ([0.0, 1.0], [0.0, True, 0.0], [0.0, float("inf"), 0.0], None):
            block = deepcopy(BASELINK_CONFIG)
            block["cameras"][0]["translation_base_camera_m"] = translation
            with self.subTest(translation=translation), self.assertRaisesRegex(TaskError, "translation"):
                load_workflow_config(self.write_config(extra={"camera_baselink": block}))

    def test_baselink_discovery_uses_strict_name_and_rejects_imus(self):
        first = self.write_dataset("stereo_base_link_261021_1630")
        self.assertEqual(discover_dataset(self.machine, "stereo_baselink"), first)
        second = self.write_dataset("stereo_base_link_261021_1741")
        with self.assertRaisesRegex(TaskError, "multiple.*stereo_baselink"):
            discover_dataset(self.machine, "stereo_baselink")
        self.assertEqual(discover_dataset(self.machine, "stereo_baselink", second.name), second)
        config = self.write_config(extra={"datasets": {"stereo_baselink": second.name}})
        self.assertEqual(load_workflow_config(config)["datasets"]["stereo_baselink"], second.name)
        wrong = self.write_dataset("stereo_base_link_261021_1800", imu=True)
        with self.assertRaisesRegex(TaskError, "IMUs"):
            discover_dataset(self.machine, "stereo_baselink", wrong.name)

    def test_baselink_runs_without_imu_and_reuses_exact_camera_result(self):
        self.write_dataset("stereo_260916_2136")
        config = self.write_config(extra={"camera_baselink": BASELINK_CONFIG})
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(first["camera_baselink"]["status"], "waiting_for_dataset")
        first_vision = load_yaml(first["calib_params"]["vision"]["result"])
        self.assertEqual(first_vision["camera"]["cam0"]["pitch"], 0.0)
        self.assertEqual(
            first_vision["camera"]["cam0"]["T_base_cam"][0][3], 0.0)
        self.write_dataset("stereo_base_link_261021_1630")
        second = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual([call["job"] for call in calls], [
            "camera_calibration", "camera_baselink_calibration"])
        task = calls[-1]["task"]
        self.assertEqual(task["camera_calibration"]["path"], second["camera"]["result"])
        self.assertEqual(task["cameras"], BASELINK_CONFIG["cameras"])
        self.assertEqual(task["base_link"]["position_scale_m"], 0.01)
        self.assertEqual(task["base_link"]["frame_id"], "base_link")
        self.assertEqual(
            task["base_link"]["rotation_left_correction"],
            [[1.0, 0.0, 0.0],
             [0.0, -1.0, 0.0],
             [0.0, 0.0, -1.0]])
        self.assertNotIn("initialization", task)
        self.assertNotIn("imus", task)
        self.assertEqual(second["camera_imu"]["status"], "waiting_for_dataset")
        self.assertEqual(second["camera_baselink"]["action"], "executed")
        self.assertTrue(second["camera_baselink"]["result"].endswith(
            "stereo_base_link_261021_1630/stereo_base_link_261021_1630.yaml"))
        self.assertEqual(second["calib_params"]["vision"]["action"], "updated")
        vision_params = load_yaml(second["calib_params"]["vision"]["result"])
        self.assertEqual(vision_params["camera"]["cam0"]["pitch"], 8.0)
        self.assertEqual(
            vision_params["camera"]["cam0"]["T_base_cam"][0][3], 0.2)
        third = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(third["camera_baselink"]["action"], "reused")
        self.assertEqual(len(calls), 2)
        block = deepcopy(BASELINK_CONFIG)
        block["cameras"][0]["translation_base_camera_m"][0] += 0.01
        with self.assertRaisesRegex(TaskError, "new batch"):
            run_machine_workflow(self.prefix, self.write_config(
                extra={"camera_baselink": block}), runner=runner)
        self.assertEqual(len(calls), 2)

    def test_all_three_stages_and_rolling_shutter_mode(self):
        self.write_dataset("stereo_260916_2136")
        self.write_dataset("stereo_imu_260916_2200", imu=True)
        self.write_dataset("stereo_base_link_261021_1741")
        cam0_baselink = {
            "cameras": [{
                "id": "cam0",
                "translation_base_camera_m": [0.1, 0.2, 0.3],
            }],
        }
        config = self.write_config(
            mode="rolling_shutter", camera_mode="rolling_shutter",
            recompute_extrinsics=True, target="aprilgrid_1",
            extra={"camera_baselink": cam0_baselink})
        calls = []
        summary = run_machine_workflow(self.prefix, config, runner=self._mock_runner(calls))
        self.assertEqual([call["job"] for call in calls], [
            "camera_rolling_shutter_calibration",
            "camera_imu_rolling_shutter_calibration",
            "camera_baselink_calibration"])
        self.assertTrue(calls[1]["task"]["calibration"][
            "recompute_camera_chain_extrinsics"])
        self.assertNotIn("time_range_s", calls[1]["task"]["dataset"])
        self.assertEqual(calls[2]["task"]["cameras"], cam0_baselink["cameras"])
        self.assertEqual(calls[2]["task"]["base_link"], {
            "frame_id": "base_link", "position_scale_m": 0.01,
            "rotation_left_correction": [
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
            ]})
        for call in calls:
            self.assertEqual(
                Path(call["task"]["target"]["path"]).name,
                "aprilgrid_1.yaml")
        for stage in ("camera", "camera_imu", "camera_baselink"):
            self.assertEqual(summary[stage]["status"], "completed")
        self.assertTrue(summary["camera"]["result"].endswith(
            "stereo_260916_2136/stereo_260916_2136.yaml"))
        self.assertTrue(summary["camera_baselink"]["result"].endswith(
            "stereo_base_link_261021_1741/"
            "stereo_base_link_261021_1741.yaml"))

    def test_baselink_disabled_missing_configuration_and_missing_camera(self):
        calls = []
        runner = self._mock_runner(calls)
        config = self.write_config()
        self.assertEqual(run_machine_workflow(self.prefix, config, runner=runner)[
            "camera_baselink"]["status"], "disabled")
        self.write_dataset("stereo_base_link_261021_1630")
        with self.assertRaisesRegex(
                TaskError, "stereo_baselink.*camera_baselink"):
            run_machine_workflow(self.prefix, config, runner=runner)
        config = self.write_config(extra={"camera_baselink": BASELINK_CONFIG})
        with self.assertRaisesRegex(
                TaskError, "stereo_baselink.*requires a stereo"):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(calls, [])

    def test_default_runner_dispatches_baselink_without_changing_solver_interface(self):
        with mock.patch("kalibr_no_ros.camera_baselink.run_camera_baselink") as run:
            machine_workflow._run_task(self.prefix, "task.yaml", "output",
                                       "camera_baselink_calibration", force=False)
            run.assert_called_once_with(
                "task.yaml", "output", force=False,
                _delivery_layout="machine")
        with mock.patch("kalibr_no_ros.task.run_task") as run:
            machine_workflow._run_task(self.prefix, "task.yaml", "output",
                                       "camera_calibration", force=False)
            run.assert_called_once_with(
                self.prefix, "task.yaml", "output", "camera_calibration",
                force=False, _delivery_layout="machine")

    def test_mono_discovery_and_configuration_are_module_local(self):
        mono = self.write_dataset("usb_261021_1500", cameras=("cam0",))
        baselink = self.write_dataset("usb_base_link_261021_1630", cameras=("cam0",))
        self.write_dataset("usb_test_261021_1700", cameras=("cam0",))
        self.assertEqual(discover_dataset(self.machine, "mono"), mono)
        self.assertEqual(discover_dataset(self.machine, "mono_baselink"), baselink)
        another = self.write_dataset("usb_base_link_261021_1741", cameras=("cam0",))
        with self.assertRaisesRegex(TaskError, "multiple.*mono_baselink"):
            discover_dataset(self.machine, "mono_baselink")
        config = self.write_config(extra={
            "datasets": {"mono": mono.name, "mono_baselink": another.name},
            "mono_camera_baselink": MONO_BASELINK_CONFIG,
        })
        loaded = load_workflow_config(config)
        self.assertEqual(loaded["datasets"]["mono_baselink"], another.name)
        self.assertEqual(loaded["mono_camera_baselink"]["base_link"]["frame_id"], "base_link")
        self.assertIsNone(loaded["camera_baselink"])
        self.assertEqual(discover_dataset(self.machine, "mono_baselink", another.name), another)
        with self.assertRaisesRegex(TaskError, "mono_camera_baselink.cameras"):
            load_workflow_config(self.write_config(extra={"mono_camera_baselink": BASELINK_CONFIG}))
        bad_units = deepcopy(MONO_BASELINK_CONFIG)
        bad_units["base_link"]["position_scale_m"] = 0
        with self.assertRaisesRegex(TaskError, "position_scale_m"):
            load_workflow_config(self.write_config(extra={"mono_camera_baselink": bad_units}))
        bad_translation = deepcopy(MONO_BASELINK_CONFIG)
        bad_translation["cameras"][0]["translation_base_camera_m"] = [0, float("inf"), 0]
        with self.assertRaisesRegex(TaskError, "translation"):
            load_workflow_config(self.write_config(extra={"mono_camera_baselink": bad_translation}))

    def test_mono_datasets_reject_wrong_sensor_composition(self):
        for name, cameras, imu, message in (
                ("usb_261021_1500", ("cam0", "cam1"), False, "cameras"),
                ("usb_261021_1510", ("cam1",), False, "cameras"),
                ("usb_base_link_261021_1630", ("cam0",), True, "IMUs")):
            with self.subTest(name=name):
                dataset = self.write_dataset(name, cameras=cameras, imu=imu)
                role = "mono" if name.startswith("usb_261021") else "mono_baselink"
                with self.assertRaisesRegex(TaskError, message):
                    discover_dataset(self.machine, role, dataset.name)

    def test_usb_directory_names_replace_the_previous_mono_names(self):
        for role, old_names in (
                ("mono", ("mono_261021_1510", "usb_intrinsics_261021_1510")),
                ("mono_baselink", ("mono_base_link_261021_1640", "mono_baselink_261021_1640",
                                   "usb_baselink_261021_1640"))):
            for name in old_names:
                with self.subTest(role=role, name=name):
                    self.write_dataset(name, cameras=("cam0",))
                    self.assertIsNone(discover_dataset(self.machine, role))
                    with self.assertRaisesRegex(TaskError, "valid directory name"):
                        discover_dataset(self.machine, role, name)
                    with self.assertRaisesRegex(TaskError, "matching direct-child"):
                        load_workflow_config(self.write_config(extra={"datasets": {role: name}}))

    def test_all_five_stages_keep_stereo_and_mono_dependencies_separate(self):
        self.write_dataset("stereo_261021_1500")
        self.write_dataset("stereo_imu_261021_1600", imu=True)
        self.write_dataset("stereo_base_link_261021_1630")
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        self.write_dataset("usb_base_link_261021_1640", cameras=("cam0",))
        config = self.write_config(extra={
            "camera_baselink": BASELINK_CONFIG,
            "mono_camera_baselink": MONO_BASELINK_CONFIG,
        })
        calls = []
        runner = self._mock_runner(calls)
        summary = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual([call["job"] for call in calls], [
            "camera_calibration", "camera_imu_calibration", "camera_baselink_calibration",
            "camera_calibration", "camera_baselink_calibration"])
        stereo_result = summary["camera"]["result"]
        mono_result = summary["mono_camera"]["result"]
        self.assertNotEqual(stereo_result, mono_result)
        output = self.machine / "result"
        for name in (
                "stereo_261021_1500", "stereo_imu_261021_1600",
                "stereo_base_link_261021_1630", "usb_261021_1510",
                "usb_base_link_261021_1640"):
            self.assertTrue((output / name / (name + ".yaml")).is_file())
            self.assertTrue((output / (name + ".report.html")).is_file())
            self.assertTrue((output / (name + ".report.pdf")).is_file())
            self.assertFalse((output / name / (name + ".report.html")).exists())
        self.assertEqual(calls[1]["task"]["camera_calibration"]["path"], stereo_result)
        self.assertEqual(calls[2]["task"]["camera_calibration"]["path"], stereo_result)
        self.assertEqual(calls[4]["task"]["camera_calibration"]["path"], mono_result)
        self.assertEqual(calls[4]["task"]["cameras"], MONO_BASELINK_CONFIG["cameras"])
        self.assertEqual(calls[2]["task"]["cameras"], BASELINK_CONFIG["cameras"])
        self.assertEqual(
            calls[2]["task"]["base_link"]["rotation_left_correction"],
            [[1.0, 0.0, 0.0],
             [0.0, -1.0, 0.0],
             [0.0, 0.0, -1.0]])
        self.assertEqual(
            calls[4]["task"]["base_link"]["rotation_left_correction"],
            [[-1.0, 0.0, 0.0],
             [0.0, 1.0, 0.0],
             [0.0, 0.0, -1.0]])
        for index in (3, 4):
            self.assertEqual([camera["id"] for camera in calls[index]["task"]["cameras"]], ["cam0"])
            self.assertNotIn("imus", calls[index]["task"])
            self.assertEqual(calls[index]["task"]["target"], calls[0]["task"]["target"])
        self.assertEqual(
            Path(calls[3]["task"]["initialization"]["path"]),
            (self.prefix / "config" / "mechine" / "workflow_v1.0.0"
             / "initialization_mono_camera.yaml").resolve())
        self.assertNotIn("initialization", calls[4]["task"])
        self.assertEqual(load_yaml(mono_result)["cameras"][0]["id"], "cam0")
        self.assertEqual(len(load_yaml(mono_result)["cameras"]), 1)
        self.assertEqual(set(summary["calib_params"]), {"vision", "mono"})
        self.assertFalse((self.machine / "result" / "calib_params"
                          / "stereo_camera").exists())
        self.assertTrue((self.machine / "result" / "calib_params"
                         / "vision" / "stereo_imu_calib.yaml").is_file())
        self.assertTrue((self.machine / "result" / "calib_params"
                         / "mono_camera" / "usb_mono_calib.yaml").is_file())
        parameter_bytes = {
            key: Path(value["result"]).read_bytes()
            for key, value in summary["calib_params"].items()
        }
        vision = load_yaml(summary["calib_params"]["vision"]["result"])
        self.assertEqual(vision["imu"]["gyro_bias"], [0.001, 0.002, 0.003])
        self.assertEqual(vision["camera"]["cam0"]["T_base_cam"][0][3], 0.2)
        self.assertEqual(vision["camera"]["cam0"]["pitch"], 8.0)
        self.assertIn("cam0_cam1_correct", vision["camera"])
        self.assertNotIn("T_BS", vision["imu"])
        self.assertNotIn("T_BS", vision["odom"])
        mono = load_yaml(summary["calib_params"]["mono"]["result"])
        self.assertEqual(mono["camera"]["T_base_cam"][0][3], 0.2)
        self.assertEqual(mono["camera"]["pitch"], 8.0)
        reused = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 5)
        for stage in ("camera", "camera_imu", "camera_baselink", "mono_camera", "mono_camera_baselink"):
            self.assertEqual(reused[stage]["action"], "reused")
        for key, before in parameter_bytes.items():
            self.assertEqual(reused["calib_params"][key]["action"], "reused")
            self.assertEqual(Path(reused["calib_params"][key]["result"]).read_bytes(), before)

    def test_calib_params_tampering_is_not_silently_overwritten(self):
        self.write_dataset("stereo_261021_1500")
        calls = []
        runner = self._mock_runner(calls)
        config = self.write_config()
        summary = run_machine_workflow(self.prefix, config, runner=runner)
        params = Path(summary["calib_params"]["vision"]["result"])
        params.write_text("tampered\n", encoding="utf-8")
        with self.assertRaisesRegex(TaskError, "changed outside"):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 1)

    def test_previous_managed_stereo_param_is_safely_retired(self):
        self.write_dataset("stereo_261021_1500")
        calls = []
        runner = self._mock_runner(calls)
        config = self.write_config()
        first = run_machine_workflow(self.prefix, config, runner=runner)
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        root = self.machine / "result" / "calib_params"
        path = root / "stereo_camera" / "stereo_calib.yaml"
        path.parent.mkdir()
        path.write_text("stereo\n", encoding="utf-8")
        record = {
            "path": str(path.resolve()),
            "sha256": machine_workflow._sha256(path),
        }
        state["calib_params"]["stereo"] = record
        state_path.write_text(json.dumps(state), encoding="utf-8")

        second = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(second["calib_params"]["vision"]["action"], "reused")
        self.assertEqual(len(calls), 1)
        self.assertFalse((root / "stereo_camera").exists())
        restored = json.loads(state_path.read_text(encoding="utf-8"))
        self.assertEqual(set(restored["calib_params"]), {"vision"})
        self.assertEqual(
            Path(first["calib_params"]["vision"]["result"]).read_bytes(),
            Path(second["calib_params"]["vision"]["result"]).read_bytes())

    def test_previous_managed_vision_name_is_safely_migrated(self):
        self.write_dataset("stereo_261021_1500")
        calls = []
        runner = self._mock_runner(calls)
        config = self.write_config()
        first = run_machine_workflow(self.prefix, config, runner=runner)
        current = Path(first["calib_params"]["vision"]["result"])
        before = current.read_bytes()
        legacy = current.with_name("vision_calib.yaml")
        current.replace(legacy)
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        state["calib_params"]["vision"]["path"] = str(legacy.resolve())
        state_path.write_text(json.dumps(state), encoding="utf-8")

        second = run_machine_workflow(self.prefix, config, runner=runner)
        migrated = Path(second["calib_params"]["vision"]["result"])
        self.assertEqual(second["calib_params"]["vision"]["action"], "reused")
        self.assertEqual(migrated.name, "stereo_imu_calib.yaml")
        self.assertEqual(migrated.read_bytes(), before)
        self.assertFalse(legacy.exists())
        self.assertEqual(len(calls), 1)

    def test_previous_vision_format_is_not_reused(self):
        self.write_dataset("stereo_261021_1500")
        calls = []
        runner = self._mock_runner(calls)
        config = self.write_config()
        first = run_machine_workflow(self.prefix, config, runner=runner)
        vision_path = Path(first["calib_params"]["vision"]["result"])
        vision_path.write_text(
            "machine_version: EVT312\ncalib_version: 1.0.0\n",
            encoding="utf-8")
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        active = state["calib_params"]["vision"]
        legacy_signature = machine_workflow._json_sha256({
            "format_version": "1.0.0",
            "identity": {"machine_version": "EVT312", "machine_num": "02"},
            "revision": "__base__",
            "profile_sha256": active["profile"]["sha256"],
            "sources": active["sources"],
            "defaulted_fields": active["defaulted_fields"],
            "context": {
                "camera_mode": "standard",
                "camera_imu_mode": "standard",
            },
        })
        active["sha256"] = machine_workflow._sha256(vision_path)
        active["source_signature"] = legacy_signature
        state_path.write_text(json.dumps(state), encoding="utf-8")

        second = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(second["calib_params"]["vision"]["action"], "updated")
        restored = load_yaml(vision_path)
        self.assertEqual(restored["algorithm_version"], "1.0.0")
        self.assertEqual(restored["calib_file_version"], "1.0.0")
        self.assertNotIn("calib_version", restored)
        self.assertEqual(len(calls), 1)

    def test_previous_usb_mono_format_is_not_reused(self):
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        self.write_dataset("usb_base_link_261021_1640", cameras=("cam0",))
        calls = []
        runner = self._mock_runner(calls)
        config = self.write_config(extra={
            "mono_camera_baselink": MONO_BASELINK_CONFIG,
        })
        first = run_machine_workflow(self.prefix, config, runner=runner)
        mono_path = Path(first["calib_params"]["mono"]["result"])
        mono_path.write_text(
            "machine_version: EVT312\ncalib_version: 1.0.0\n",
            encoding="utf-8")
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        active = state["calib_params"]["mono"]
        legacy_signature = machine_workflow._json_sha256({
            "format_version": "1.0.0",
            "identity": {"machine_version": "EVT312", "machine_num": "02"},
            "revision": "__base__",
            "profile_sha256": active["profile"]["sha256"],
            "sources": active["sources"],
            "defaulted_fields": active["defaulted_fields"],
            "context": None,
        })
        active["sha256"] = machine_workflow._sha256(mono_path)
        active["source_signature"] = legacy_signature
        state_path.write_text(json.dumps(state), encoding="utf-8")

        second = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(second["calib_params"]["mono"]["action"], "updated")
        restored = load_yaml(mono_path)
        self.assertEqual(restored["algorithm_version"], "1.0.0")
        self.assertEqual(restored["calib_file_version"], "1.0.0")
        self.assertIn("T_base_cam", restored["camera"])
        self.assertNotIn("T_base_cam0", restored["camera"])
        self.assertEqual(len(calls), 2)

    def test_unmanaged_obsolete_param_directory_is_not_deleted(self):
        self.write_dataset("stereo_261021_1500")
        calls = []
        runner = self._mock_runner(calls)
        config = self.write_config()
        run_machine_workflow(self.prefix, config, runner=runner)
        unknown = (self.machine / "result" / "calib_params"
                   / "stereo_camera" / "keep.txt")
        unknown.parent.mkdir()
        unknown.write_text("keep\n", encoding="utf-8")
        with self.assertRaisesRegex(TaskError, "obsolete unmanaged"):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(unknown.read_text(encoding="utf-8"), "keep\n")
        self.assertEqual(len(calls), 1)

    def test_mono_can_run_without_stereo_and_resume_baselink_later(self):
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        config = self.write_config(extra={"mono_camera_baselink": MONO_BASELINK_CONFIG})
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 1)
        self.assertEqual(first["mono_camera"]["status"], "completed")
        self.assertEqual(first["mono_camera_baselink"]["status"], "waiting_for_dataset")
        self.assertEqual(set(first["calib_params"]), {"mono"})
        first_mono = load_yaml(first["calib_params"]["mono"]["result"])
        self.assertEqual(first_mono["camera"]["T_base_cam"][0][3], 0.0)
        self.assertEqual(first_mono["camera"]["pitch"], 0.0)
        self.write_dataset("usb_base_link_261021_1640", cameras=("cam0",))
        second = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 2)
        self.assertEqual(second["mono_camera"]["action"], "reused")
        self.assertEqual(second["mono_camera_baselink"]["status"], "completed")
        self.assertEqual(second["camera"]["status"], "waiting_for_dataset")
        self.assertEqual(second["calib_params"]["mono"]["action"], "updated")
        second_mono = load_yaml(second["calib_params"]["mono"]["result"])
        self.assertEqual(second_mono["camera"]["T_base_cam"][0][3], 0.2)
        self.assertEqual(second_mono["camera"]["pitch"], 8.0)
        self.assertTrue(second["mono_camera"]["result"].endswith(
            "usb_261021_1510/usb_261021_1510.yaml"))
        self.assertTrue(second["mono_camera_baselink"]["result"].endswith(
            "usb_base_link_261021_1640/usb_base_link_261021_1640.yaml"))

    def test_mono_baselink_never_falls_back_to_stereo_camera(self):
        self.write_dataset("stereo_261021_1500")
        self.write_dataset("usb_base_link_261021_1640", cameras=("cam0",))
        calls = []
        with self.assertRaisesRegex(
                TaskError, "mono_baselink.*requires a mono"):
            run_machine_workflow(self.prefix, self.write_config(extra={
                "mono_camera_baselink": MONO_BASELINK_CONFIG,
            }), runner=self._mock_runner(calls))
        self.assertEqual(calls, [])

    def test_stereo_base_link_never_falls_back_to_mono_camera(self):
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        self.write_dataset("stereo_base_link_261021_1630")
        calls = []
        with self.assertRaisesRegex(
                TaskError, "stereo_baselink.*requires a stereo"):
            run_machine_workflow(self.prefix, self.write_config(extra={
                "camera_baselink": BASELINK_CONFIG,
            }), runner=self._mock_runner(calls))
        self.assertEqual(calls, [])

    def test_add_mono_after_stereo_and_require_new_batch_for_config_changes(self):
        self.write_dataset("stereo_261021_1500")
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, self.write_config(), runner=runner)
        self.assertEqual(first["mono_camera_baselink"]["status"], "disabled")
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        self.write_dataset("usb_base_link_261021_1640", cameras=("cam0",))
        with self.assertRaisesRegex(
                TaskError, "mono_baselink.*mono_camera_baselink"):
            run_machine_workflow(self.prefix, self.write_config(), runner=runner)
        self.assertEqual(len(calls), 1)
        config = self.write_config(extra={"mono_camera_baselink": MONO_BASELINK_CONFIG})
        completed = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(completed["camera"]["action"], "reused")
        self.assertEqual(completed["mono_camera"]["action"], "executed")
        self.assertEqual(completed["mono_camera_baselink"]["action"], "executed")
        before = Path(completed["mono_camera_baselink"]["result"]).read_bytes()
        changed = deepcopy(MONO_BASELINK_CONFIG)
        changed["cameras"][0]["translation_base_camera_m"][0] += 0.01
        with self.assertRaisesRegex(TaskError, "mono_camera_baselink.*new batch"):
            run_machine_workflow(self.prefix, self.write_config(extra={
                "mono_camera_baselink": changed}), runner=runner)
        self.assertEqual(len(calls), 3)
        self.assertEqual(Path(completed["mono_camera_baselink"]["result"]).read_bytes(), before)

    def test_machine_cli_returns_nonzero_for_a_blocked_stage(self):
        summary = {
            "machine_dir": str(self.machine),
            "output_dir": str(self.machine / "result"),
            "camera": {"status": "waiting_for_dataset", "result": None},
            "camera_imu": {
                "status": "blocked_missing_camera_calibration",
                "result": None,
            },
        }
        with mock.patch.object(
                machine_cli, "run_machine_workflow", return_value=summary):
            self.assertEqual(machine_cli.main(
                ["machine", "--config", "user.yaml"], prefix=self.prefix), 2)
        summary["camera_imu"] = {"status": "waiting_for_dataset", "result": None}
        summary["camera_baselink"] = {"status": "blocked_missing_configuration", "result": None}
        with mock.patch.object(machine_cli, "run_machine_workflow", return_value=summary):
            self.assertEqual(machine_cli.main(
                ["machine", "--config", "user.yaml"], prefix=self.prefix), 2)
        summary["camera_baselink"] = {"status": "disabled", "result": None}
        summary["mono_camera"] = {"status": "completed", "result": "mono.yaml"}
        summary["mono_camera_baselink"] = {"status": "blocked_missing_configuration", "result": None}
        with mock.patch.object(machine_cli, "run_machine_workflow", return_value=summary):
            with mock.patch("builtins.print") as printed:
                self.assertEqual(machine_cli.main(
                    ["machine", "--config", "user.yaml"], prefix=self.prefix), 2)
                printed.assert_any_call("mono_camera: completed -> mono.yaml")
                printed.assert_any_call("mono_camera_baselink: blocked_missing_configuration")

    def test_machine_cli_defaults_to_installed_user_config(self):
        summary = {
            "machine_dir": str(self.machine),
            "output_dir": str(self.machine / "result"),
            "camera": {"status": "waiting_for_dataset", "result": None},
        }
        with mock.patch.object(
                machine_cli, "run_machine_workflow", return_value=summary) as run:
            self.assertEqual(machine_cli.main(["machine"], prefix=self.prefix), 0)
        run.assert_called_once_with(
            self.prefix.resolve(), self.prefix / "config" / "user.yaml")

    def test_failed_stage_is_recorded_and_the_whole_stage_is_retried(self):
        self.write_dataset("stereo_260916_2136")
        config = self.write_config()
        attempts = []

        def fail(prefix, task, output, expected_job, force=False):
            attempts.append(expected_job)
            raise RuntimeError("solver sentinel")

        with self.assertRaisesRegex(RuntimeError, "solver sentinel"):
            run_machine_workflow(self.prefix, config, runner=fail)
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        self.assertEqual(
            state["stages"]["camera"]["status"],
            "failed")

        calls = []
        summary = run_machine_workflow(
            self.prefix, config, runner=self._mock_runner(calls))
        self.assertEqual(attempts, ["camera_calibration"])
        self.assertEqual([call["job"] for call in calls],
                         ["camera_calibration"])
        self.assertEqual(summary["camera"]["action"], "executed")
        self.assertEqual(summary["camera"]["status"], "completed")

    def test_recalibration_dependency_map_covers_all_dataset_roles(self):
        self.assertEqual(
            machine_workflow._affected_stages({"stereo"}),
            ["camera", "camera_imu", "camera_baselink"])
        self.assertEqual(
            machine_workflow._affected_stages({"stereo_imu"}),
            ["camera_imu"])
        self.assertEqual(
            machine_workflow._affected_stages({"stereo_baselink"}),
            ["camera_baselink"])
        self.assertEqual(
            machine_workflow._affected_stages({"mono"}),
            ["mono_camera", "mono_camera_baselink"])
        self.assertEqual(
            machine_workflow._affected_stages({"mono_baselink"}),
            ["mono_camera_baselink"])

        files = {
            name: "old" for name in [
                *machine_workflow._TEMPLATES.values(),
                "initialization_stereo_camera.yaml",
                "initialization_mono_camera.yaml", "imu_noise.yaml",
                "mechine_base_param.yaml",
            ]
        }
        profile = {
            "version": machine_workflow.PROFILE_VERSION,
            "files": files,
            "target": {"name": "aprilgrid_1.yaml", "sha256": "target"},
        }
        binding = {
            "camera_mode": "standard",
            "camera_imu_mode": "standard",
        }
        cases = {
            "stereo_camera_calibration_task.yaml": [
                "camera", "camera_imu", "camera_baselink"],
            "stereo_camera_rs_calibration_task.yaml": [],
            "initialization_stereo_camera.yaml": [
                "camera", "camera_imu", "camera_baselink"],
            "initialization_mono_camera.yaml": [
                "mono_camera", "mono_camera_baselink"],
            "stereo_imu_calibration_task.yaml": ["camera_imu"],
            "stereo_imu_rs_calibration_task.yaml": [],
            "imu_noise.yaml": ["camera_imu"],
            "stereo_camera_baselink_calibration_task.yaml": [
                "camera_baselink"],
            "mono_camera_calibration_task.yaml": [
                "mono_camera", "mono_camera_baselink"],
            "mono_camera_baselink_calibration_task.yaml": [
                "mono_camera_baselink"],
            "mechine_base_param.yaml": [],
        }
        for name, expected in cases.items():
            changed = deepcopy(profile)
            changed["files"][name] = "new"
            with self.subTest(profile=name):
                self.assertEqual(
                    machine_workflow._profile_recalibration(
                        profile, changed, binding)[1],
                    expected)
        changed_target = deepcopy(profile)
        changed_target["target"]["sha256"] = "new"
        self.assertEqual(
            machine_workflow._profile_recalibration(
                profile, changed_target, binding)[1],
            list(machine_workflow._STAGE_ROLES))

    def test_installed_camera_template_uses_semantic_change_and_reruns_dependents(self):
        self.write_dataset("stereo_261029_1000")
        self.write_dataset("stereo_imu_261029_1100", imu=True)
        self.write_dataset("stereo_base_link_261029_1200")
        config = self.write_config(extra={"camera_baselink": BASELINK_CONFIG})
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 3)

        template = self.profile_file("stereo_camera_calibration_task.yaml")
        template.write_text(
            template.read_text(encoding="utf-8") + "\n# onsite comment\n",
            encoding="utf-8")
        comment_only = run_machine_workflow(
            self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 3)
        self.assertEqual(
            [comment_only[stage]["action"] for stage in (
                "camera", "camera_imu", "camera_baselink")],
            ["reused", "reused", "reused"])

        document = load_yaml(template)
        document["calibration"]["shuffle"] = False
        dump_yaml(document, template)
        changed = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(
            [(call["job"], call["force"]) for call in calls[3:]],
            [("camera_calibration", True),
             ("camera_imu_calibration", True),
             ("camera_baselink_calibration", True)])
        self.assertEqual(
            [changed[stage]["action"] for stage in (
                "camera", "camera_imu", "camera_baselink")],
            ["executed", "executed", "executed"])
        state = json.loads((self.machine / "result" / ".kalibr-cedarobot"
                            / "state.json").read_text(encoding="utf-8"))
        self.assertNotIn("pending_recalibration", state)

    def test_installed_profile_changes_are_scoped_to_active_tasks(self):
        self.write_dataset("stereo_261029_1000")
        self.write_dataset("stereo_imu_261029_1100", imu=True)
        self.write_dataset("stereo_base_link_261029_1200")
        self.write_dataset("usb_261029_1300", cameras=("cam0",))
        self.write_dataset(
            "usb_base_link_261029_1400", cameras=("cam0",))
        config = self.write_config(extra={
            "camera_baselink": BASELINK_CONFIG,
            "mono_camera_baselink": MONO_BASELINK_CONFIG,
        })
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 5)

        inactive = self.profile_file(
            "stereo_camera_rs_calibration_task.yaml")
        inactive_document = load_yaml(inactive)
        inactive_document["calibration"]["window_half_size_px"] += 1
        dump_yaml(inactive_document, inactive)
        run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 5)

        mono = self.profile_file("mono_camera_calibration_task.yaml")
        mono_document = load_yaml(mono)
        mono_document["calibration"]["shuffle"] = not mono_document[
            "calibration"]["shuffle"]
        dump_yaml(mono_document, mono)
        run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(
            [(call["job"], call["force"]) for call in calls[5:]],
            [("camera_calibration", True),
             ("camera_baselink_calibration", True)])

        profile = self.profile_file("mechine_base_param.yaml")
        profile_document = load_yaml(profile)
        profile_document["imu"]["arw"] += 0.001
        dump_yaml(profile_document, profile)
        before = len(calls)
        params_only = run_machine_workflow(
            self.prefix, config, runner=runner)
        self.assertEqual(len(calls), before)
        self.assertEqual(params_only["calib_params"]["vision"]["action"],
                         "updated")
        self.assertEqual(params_only["calib_params"]["mono"]["action"],
                         "updated")

    def test_external_initialization_change_reruns_its_dependent_chain(self):
        self.write_dataset("stereo_261029_1000")
        self.write_dataset("stereo_imu_261029_1100", imu=True)
        machine_config = self.prefix / "config" / "mechine"
        custom = machine_config / "custom_initialization.yaml"
        shutil.copy2(
            self.profile_file("initialization_stereo_camera.yaml"), custom)
        template = self.profile_file("stereo_camera_calibration_task.yaml")
        document = load_yaml(template)
        document["initialization"]["path"] = "../custom_initialization.yaml"
        dump_yaml(document, template)

        calls = []
        runner = self._mock_runner(calls)
        config = self.write_config()
        run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 2)

        custom.write_text(
            custom.read_text(encoding="utf-8") + "\n# onsite comment\n",
            encoding="utf-8")
        comment_only = run_machine_workflow(
            self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 2)
        self.assertEqual(comment_only["camera"]["action"], "reused")

        initialization = load_yaml(custom)
        initialization["cameras"]["cam0"]["intrinsics"][0] += 1.0
        dump_yaml(initialization, custom)
        changed = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(
            [(call["job"], call["force"]) for call in calls[2:]],
            [("camera_calibration", True),
             ("camera_imu_calibration", True)])
        self.assertEqual(changed["camera"]["action"], "executed")
        self.assertEqual(changed["camera_imu"]["action"], "executed")

    def test_invalid_changed_profile_is_rejected_before_overwriting_results(self):
        self.write_dataset("stereo_261029_1000")
        self.write_dataset("stereo_imu_261029_1100", imu=True)
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        camera_result = Path(first["camera"]["result"])
        camera_before = camera_result.read_bytes()

        camera_template = self.profile_file(
            "stereo_camera_calibration_task.yaml")
        camera_document = load_yaml(camera_template)
        camera_document["calibration"]["shuffle"] = False
        dump_yaml(camera_document, camera_template)
        imu_template = self.profile_file("stereo_imu_calibration_task.yaml")
        imu_document = load_yaml(imu_template)
        imu_document["kind"] = "invalid"
        dump_yaml(imu_document, imu_template)

        with self.assertRaisesRegex(TaskError, "unexpected kind or job"):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 2)
        self.assertEqual(camera_result.read_bytes(), camera_before)
        state = json.loads((self.machine / "result" / ".kalibr-cedarobot"
                            / "state.json").read_text(encoding="utf-8"))
        self.assertNotIn("pending_recalibration", state)

    def test_changed_template_failure_preserves_old_result_and_retries(self):
        self.write_dataset("stereo_261029_1000")
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        result = Path(first["camera"]["result"])
        result_before = result.read_bytes()
        report = self.machine / "result" / "stereo_261029_1000.report.html"
        report_before = report.read_bytes()

        template = self.profile_file("stereo_camera_calibration_task.yaml")
        document = load_yaml(template)
        document["calibration"]["shuffle"] = False
        dump_yaml(document, template)
        failed_calls = []

        def fail(prefix, task, output, expected_job, force=False):
            failed_calls.append((expected_job, force))
            raise RuntimeError("template recalibration sentinel")

        with self.assertRaisesRegex(RuntimeError, "template recalibration sentinel"):
            run_machine_workflow(self.prefix, config, runner=fail)
        self.assertEqual(failed_calls, [("camera_calibration", True)])
        self.assertEqual(result.read_bytes(), result_before)
        self.assertEqual(report.read_bytes(), report_before)
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        self.assertIn(
            "pending_recalibration",
            json.loads(state_path.read_text(encoding="utf-8")))

        retry_calls = []
        completed = run_machine_workflow(
            self.prefix, config, runner=self._mock_runner(retry_calls))
        self.assertEqual(
            [(call["job"], call["force"]) for call in retry_calls],
            [("camera_calibration", True)])
        self.assertEqual(completed["camera"]["action"], "executed")
        self.assertNotIn(
            "pending_recalibration",
            json.loads(state_path.read_text(encoding="utf-8")))

    def test_replacing_stereo_keeps_old_result_and_reruns_dependents(self):
        old = self.write_dataset("stereo_261029_1000")
        self.write_dataset("stereo_imu_261029_1100", imu=True)
        self.write_dataset("stereo_base_link_261029_1200")
        config = self.write_config(extra={"camera_baselink": BASELINK_CONFIG})
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        old_result = Path(first["camera"]["result"])
        old_bytes = old_result.read_bytes()

        old.rename(self.machine / (old.name + "_bk"))
        self.write_dataset("stereo_261029_1500")
        second = run_machine_workflow(self.prefix, config, runner=runner)

        self.assertEqual([call["force"] for call in calls[3:]], [False, True, True])
        self.assertEqual([call["job"] for call in calls[3:]], [
            "camera_calibration", "camera_imu_calibration",
            "camera_baselink_calibration"])
        self.assertEqual(old_result.read_bytes(), old_bytes)
        self.assertTrue((self.machine / "result"
                         / "stereo_261029_1000.report.html").is_file())
        self.assertTrue(second["camera"]["result"].endswith(
            "stereo_261029_1500/stereo_261029_1500.yaml"))
        self.assertEqual(
            calls[4]["task"]["camera_calibration"]["path"],
            second["camera"]["result"])
        self.assertEqual(
            calls[5]["task"]["camera_calibration"]["path"],
            second["camera"]["result"])
        state = json.loads((self.machine / "result" / ".kalibr-cedarobot"
                            / "state.json").read_text(encoding="utf-8"))
        self.assertNotIn("pending_recalibration", state)
        self.assertEqual(
            state["binding"]["datasets"]["stereo"]["name"],
            "stereo_261029_1500")

    def test_replacing_usb_reruns_only_mono_chain(self):
        old = self.write_dataset("usb_261029_1000", cameras=("cam0",))
        self.write_dataset(
            "usb_base_link_261029_1100", cameras=("cam0",))
        config = self.write_config(extra={
            "mono_camera_baselink": MONO_BASELINK_CONFIG,
        })
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        old_result = Path(first["mono_camera"]["result"])
        old_bytes = old_result.read_bytes()

        old.rename(self.machine / (old.name + "_bk"))
        self.write_dataset("usb_261029_1500", cameras=("cam0",))
        second = run_machine_workflow(self.prefix, config, runner=runner)

        self.assertEqual([call["job"] for call in calls[2:]], [
            "camera_calibration", "camera_baselink_calibration"])
        self.assertEqual([call["force"] for call in calls[2:]], [False, True])
        self.assertEqual(old_result.read_bytes(), old_bytes)
        self.assertTrue(second["mono_camera"]["result"].endswith(
            "usb_261029_1500/usb_261029_1500.yaml"))
        self.assertEqual(second["camera"]["status"], "waiting_for_dataset")

    def test_replacing_stereo_imu_reruns_only_camera_imu(self):
        self.write_dataset("stereo_261029_1000")
        old = self.write_dataset("stereo_imu_261029_1100", imu=True)
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        old_result = Path(first["camera_imu"]["result"])
        old_bytes = old_result.read_bytes()

        old.rename(self.machine / (old.name + "_bk"))
        self.write_dataset("stereo_imu_261029_1500", imu=True)
        second = run_machine_workflow(self.prefix, config, runner=runner)

        self.assertEqual(
            [(call["job"], call["force"]) for call in calls[2:]],
            [("camera_imu_calibration", False)])
        self.assertEqual(second["camera"]["action"], "reused")
        self.assertEqual(old_result.read_bytes(), old_bytes)
        self.assertTrue(second["camera_imu"]["result"].endswith(
            "stereo_imu_261029_1500/stereo_imu_261029_1500.yaml"))

    def test_replacing_stereo_baselink_reruns_only_that_stage(self):
        self.write_dataset("stereo_261029_1000")
        old = self.write_dataset("stereo_base_link_261029_1100")
        config = self.write_config(extra={"camera_baselink": BASELINK_CONFIG})
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        old_result = Path(first["camera_baselink"]["result"])
        old_bytes = old_result.read_bytes()

        old.rename(self.machine / (old.name + "_bk"))
        self.write_dataset("stereo_base_link_261029_1500")
        second = run_machine_workflow(self.prefix, config, runner=runner)

        self.assertEqual(
            [(call["job"], call["force"]) for call in calls[2:]],
            [("camera_baselink_calibration", False)])
        self.assertEqual(second["camera"]["action"], "reused")
        self.assertEqual(old_result.read_bytes(), old_bytes)
        self.assertTrue(second["camera_baselink"]["result"].endswith(
            "stereo_base_link_261029_1500/stereo_base_link_261029_1500.yaml"))

    def test_replacing_usb_baselink_reruns_only_that_stage(self):
        self.write_dataset("usb_261029_1000", cameras=("cam0",))
        old = self.write_dataset(
            "usb_base_link_261029_1100", cameras=("cam0",))
        config = self.write_config(extra={
            "mono_camera_baselink": MONO_BASELINK_CONFIG,
        })
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        old_result = Path(first["mono_camera_baselink"]["result"])
        old_bytes = old_result.read_bytes()

        old.rename(self.machine / (old.name + "_bk"))
        self.write_dataset(
            "usb_base_link_261029_1500", cameras=("cam0",))
        second = run_machine_workflow(self.prefix, config, runner=runner)

        self.assertEqual(
            [(call["job"], call["force"]) for call in calls[2:]],
            [("camera_baselink_calibration", False)])
        self.assertEqual(second["mono_camera"]["action"], "reused")
        self.assertEqual(old_result.read_bytes(), old_bytes)
        self.assertTrue(second["mono_camera_baselink"]["result"].endswith(
            "usb_base_link_261029_1500/usb_base_link_261029_1500.yaml"))

    def test_bound_dataset_cannot_disappear_without_a_replacement(self):
        old = self.write_dataset("stereo_261029_1000")
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        result = Path(first["camera"]["result"])
        before = result.read_bytes()

        old.rename(self.machine / (old.name + "_bk"))
        with self.assertRaisesRegex(TaskError, "stereo dataset disappeared"):
            run_machine_workflow(self.prefix, config, runner=runner)

        self.assertEqual(len(calls), 1)
        self.assertEqual(result.read_bytes(), before)

    def test_explicit_dataset_can_switch_while_old_capture_remains(self):
        old = self.write_dataset("stereo_261029_1000")
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(
            self.prefix, self.write_config(), runner=runner)
        new = self.write_dataset("stereo_261029_1500")
        second = run_machine_workflow(
            self.prefix,
            self.write_config(extra={"datasets": {"stereo": new.name}}),
            runner=runner)
        self.assertEqual(len(calls), 2)
        self.assertFalse(calls[1]["force"])
        self.assertTrue(Path(first["camera"]["result"]).is_file())
        self.assertTrue(Path(second["camera"]["result"]).is_file())
        self.assertTrue(old.is_dir())

    def test_recalibration_failure_preserves_old_downstream_and_params(self):
        old = self.write_dataset("stereo_261029_1000")
        self.write_dataset("stereo_imu_261029_1100", imu=True)
        config = self.write_config()
        initial_calls = []
        first = run_machine_workflow(
            self.prefix, config, runner=self._mock_runner(initial_calls))
        imu_result = Path(first["camera_imu"]["result"])
        imu_before = imu_result.read_bytes()
        report = self.machine / "result" / "stereo_imu_261029_1100.report.html"
        report_before = report.read_bytes()
        params = Path(first["calib_params"]["vision"]["result"])
        params_before = params.read_bytes()

        old.rename(self.machine / (old.name + "_bk"))
        self.write_dataset("stereo_261029_1500")
        replacement_calls = []
        camera_runner = self._mock_runner(replacement_calls)

        def fail_imu(prefix, task, output, expected_job, force=False):
            if expected_job == "camera_imu_calibration":
                replacement_calls.append({"job": expected_job, "force": force})
                raise RuntimeError("replacement sentinel")
            return camera_runner(
                prefix, task, output, expected_job, force=force)

        with self.assertRaisesRegex(RuntimeError, "replacement sentinel"):
            run_machine_workflow(self.prefix, config, runner=fail_imu)
        self.assertEqual(
            [(call["job"], call["force"]) for call in replacement_calls],
            [("camera_calibration", False),
             ("camera_imu_calibration", True)])
        self.assertEqual(imu_result.read_bytes(), imu_before)
        self.assertEqual(report.read_bytes(), report_before)
        self.assertEqual(params.read_bytes(), params_before)
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        failed_state = json.loads(state_path.read_text(encoding="utf-8"))
        self.assertIn("pending_recalibration", failed_state)
        self.assertEqual(
            failed_state["binding"]["datasets"]["stereo"]["name"],
            "stereo_261029_1000")

        retry_calls = []
        completed = run_machine_workflow(
            self.prefix, config, runner=self._mock_runner(retry_calls))
        self.assertEqual(
            [(call["job"], call["force"]) for call in retry_calls],
            [("camera_imu_calibration", True)])
        self.assertEqual(completed["camera"]["action"], "reused")
        self.assertNotIn(
            "pending_recalibration",
            json.loads(state_path.read_text(encoding="utf-8")))

    def test_pending_recalibration_rejects_another_replacement(self):
        old = self.write_dataset("stereo_261029_1000")
        self.write_dataset("stereo_imu_261029_1100", imu=True)
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)

        old.rename(self.machine / (old.name + "_bk"))
        replacement = self.write_dataset("stereo_261029_1500")
        replacement_calls = []
        delegate = self._mock_runner(replacement_calls)

        def fail_imu(prefix, task, output, expected_job, force=False):
            if expected_job == "camera_imu_calibration":
                replacement_calls.append({"job": expected_job, "force": force})
                raise RuntimeError("replacement sentinel")
            return delegate(prefix, task, output, expected_job, force=force)

        with self.assertRaisesRegex(RuntimeError, "replacement sentinel"):
            run_machine_workflow(self.prefix, config, runner=fail_imu)

        replacement.rename(self.machine / (replacement.name + "_bk"))
        self.write_dataset("stereo_261029_1600")
        with self.assertRaisesRegex(TaskError, "inputs changed.*pending"):
            run_machine_workflow(self.prefix, config, runner=runner)

        self.assertEqual(
            [(call["job"], call["force"]) for call in replacement_calls],
            [("camera_calibration", False),
             ("camera_imu_calibration", True)])
        self.assertEqual(len(calls), 2)

    def test_tampered_downstream_blocks_recalibration_before_solver(self):
        old = self.write_dataset("stereo_261029_1000")
        self.write_dataset("stereo_imu_261029_1100", imu=True)
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)
        report = self.machine / "result" / "stereo_imu_261029_1100.report.html"
        report.write_text("tampered", encoding="utf-8")
        old.rename(self.machine / (old.name + "_bk"))
        self.write_dataset("stereo_261029_1500")

        with self.assertRaises(TaskError):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 2)
        self.assertFalse((self.machine / "result" / "stereo_261029_1500").exists())

    def test_legacy_revision_state_requires_a_new_batch(self):
        config = self.write_config()
        run_machine_workflow(
            self.prefix, config, runner=self._mock_runner([]))
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        state["runs"] = {"__base__": {}}
        state_path.write_text(json.dumps(state), encoding="utf-8")
        with self.assertRaisesRegex(TaskError, "retired revision layout"):
            run_machine_workflow(
                self.prefix, config, runner=self._mock_runner([]))

    def test_dataset_changed_in_place_requires_a_new_directory_name(self):
        dataset = self.write_dataset("stereo_260916_2136")
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)
        manifest_path = dataset / "dataset.yaml"
        manifest = load_yaml(manifest_path)
        manifest["dataset_id"] = "stereo_260916_2136_changed"
        dump_yaml(manifest, manifest_path)
        with self.assertRaisesRegex((TaskError, ValueError), "changed in place"):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 1)

    def test_selected_target_uses_semantic_change_for_recalibration(self):
        self.write_dataset("stereo_260916_2136")
        config = self.write_config(target="aprilgrid_1")
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)
        target = (self.prefix / "config" / "mechine"
                  / "aprilgrid_1.yaml")
        target.write_text(
            target.read_text(encoding="utf-8") + "\n# onsite comment\n",
            encoding="utf-8")
        comment_only = run_machine_workflow(
            self.prefix, config, runner=runner)
        self.assertEqual(comment_only["camera"]["action"], "reused")
        self.assertEqual(len(calls), 1)

        document = load_yaml(target)
        document["tagSize"] = 0.066
        dump_yaml(document, target)
        changed = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(changed["camera"]["action"], "executed")
        self.assertEqual(len(calls), 2)
        self.assertTrue(calls[-1]["force"])

    def test_missing_state_never_adopts_an_existing_result(self):
        dataset = self.write_dataset("stereo_260916_2136")
        image = dataset / "cam0" / "frame.png"
        image.write_bytes(b"first")
        (dataset / "cam0.csv").write_text(
            "timestamp_ns,filename\n100,frame.png\n", encoding="utf-8")
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)

        image.write_bytes(b"other")
        state = (self.machine / "result" / ".kalibr-cedarobot"
                 / "state.json")
        state.unlink()
        with self.assertRaisesRegex((TaskError, ValueError), "batch|checkpoint"):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 1)

    def test_failed_post_publish_checkpoint_revalidates_completed_delivery(self):
        self.write_dataset("stereo_260916_2136")
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        state["stages"]["camera"] = {
            "status": "failed",
            "failure": {
                "type": "TaskError",
                "message": "post-publish delivery validation sentinel",
            },
        }
        state_path.write_text(json.dumps(state), encoding="utf-8")
        second = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 1)
        self.assertEqual(first["camera"]["result"], second["camera"]["result"])
        self.assertEqual(second["camera"]["action"], "reused")

    def test_running_checkpoint_does_not_adopt_tampered_published_files(self):
        self.write_dataset("stereo_260916_2136")
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)

        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        state["stages"]["camera"] = {
            "status": "running",
        }
        state_path.write_text(
            json.dumps(state, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")

        report = (self.machine / "result"
                  / "stereo_260916_2136.report.html")
        report.write_text("X" * report.stat().st_size, encoding="utf-8")
        with self.assertRaisesRegex(
                (TaskError, ValueError), "safely reused|published delivery"):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 1)

    def test_dataset_change_during_solver_is_not_accepted(self):
        dataset = self.write_dataset("stereo_260916_2136")
        image = dataset / "cam0" / "frame.png"
        image.write_bytes(b"before")
        (dataset / "cam0.csv").write_text(
            "timestamp_ns,filename\n100,frame.png\n", encoding="utf-8")
        config = self.write_config()
        calls = []
        delegate = self._mock_runner(calls)

        def mutate_after_solve(*args, **kwargs):
            result = delegate(*args, **kwargs)
            image.write_bytes(b"after")
            return result

        with self.assertRaisesRegex(
                (TaskError, ValueError), "changed during calibration"):
            run_machine_workflow(
                self.prefix, config, runner=mutate_after_solve)
        self.assertEqual(len(calls), 1)
        state_path = (self.machine / "result" / ".kalibr-cedarobot"
                      / "state.json")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        self.assertEqual(
            state["stages"]["camera"]["status"],
            "failed")


if __name__ == "__main__":
    unittest.main()
