"""Machine-level staged calibration orchestration without invoking a solver."""

import json
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
        self.machine = self.root / "mechine-01"
        self.machine.mkdir()
        self.prefix = self.root / "prefix"
        installed = (
            self.prefix / "share" / "kalibr-noros" / "config" / "mechine"
            / "workflow_v1.0.0"
        )
        installed.parent.mkdir(parents=True)
        # The test installation is independent from build/install outputs.
        shutil.copytree(PROFILE, installed)
        shutil.copy2(
            ROOT / "config" / "mechine" / "aprilgrid_1.yaml",
            installed.parent / "aprilgrid_1.yaml")

    def test_native_config_printers_resolve_stdout_at_call_time(self):
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
                     revision=None, extra=None):
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
        if revision is not None:
            document["revision"] = revision
        if extra:
            document.update(extra)
        path = self.root / "user.yaml"
        dump_yaml(document, path)
        return path

    def test_workflow_yaml_is_strict_and_resolves_machine_relative_to_config(self):
        relative_machine = self.root / "relative-machine"
        relative_machine.mkdir()
        path = self.root / "relative.yaml"
        dump_yaml({
            "schema_version": "1.0.0",
            "kind": "machine_calibration_workflow",
            "machine_dir": "relative-machine",
            "target": "aprilgrid_1",
            "camera": {"mode": "rolling_shutter"},
            "camera_imu": {"mode": "rolling_shutter"},
            "datasets": {"stereo": "stereo_260916_2136"},
            "revision": "rerun_2",
        }, path)
        loaded = load_workflow_config(path)
        self.assertEqual(Path(loaded["machine_dir"]), relative_machine.resolve())
        self.assertEqual(loaded["camera_imu"]["mode"], "rolling_shutter")
        self.assertEqual(loaded["camera"]["mode"], "rolling_shutter")
        self.assertEqual(loaded["target"], "aprilgrid_1")
        self.assertEqual(loaded["revision"], "rerun_2")

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
                         (PROFILE / "aprilgrid.yaml").resolve())
        self.assertEqual(Path(camera["initialization"]["path"]),
                         (PROFILE / "initialization_pinhole_equi.yaml").resolve())
        rolling_camera = build_task(
            PROFILE, "camera", stereo,
            "stereo_260916_2136_rs_pinhole_equi",
            camera_mode="rolling_shutter",
            target_path=ROOT / "config" / "mechine" / "aprilgrid_1.yaml")
        self.assertEqual(
            rolling_camera["job"], "camera_rolling_shutter_calibration")
        self.assertEqual(set(rolling_camera["rolling_shutter"]), {"cam0", "cam1"})
        self.assertEqual(
            Path(rolling_camera["target"]["path"]),
            (ROOT / "config" / "mechine" / "aprilgrid_1.yaml").resolve())

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
                             (PROFILE / "imu.yaml").resolve())
            self.assertNotIn("__DATASET_PATH__", json.dumps(task))
            self.assertTrue(task["output"]["save_diagnostics"])

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
            self.assertFalse(force)
            name = task["output"]["name"]
            output = Path(output_dir)
            output.mkdir(parents=True, exist_ok=True)
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
                     "distortion_coeffs": [0.0, 0.0, 0.0, 0.0]},
                    {"id": "cam1", "intrinsics": [1.0, 1.0, 1.0, 1.0],
                     "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
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
            if expected_job in {"camera_imu_calibration", "camera_imu_rolling_shutter_calibration"}:
                result["imus"] = [{"id": "imu0"}]
            dump_yaml(result, output / (name + ".yaml"))
            (output / (name + ".report.html")).write_text(
                "<html></html>", encoding="utf-8")
            (output / (name + ".report.pdf")).write_bytes(b"%PDF fixture")
            delivery_paths = [
                output / (name + ".yaml"),
                output / (name + ".report.html"),
                output / (name + ".report.pdf"),
            ]
            side = output / name
            side.mkdir()
            resolved_path = side / "task_resolved.yaml"
            dump_yaml(
                machine_workflow._resolved_task(config, expected_job),
                resolved_path)
            dataset_manifest = load_yaml(
                Path(task["dataset"]["path"]) / "dataset.yaml")
            input_documents = []
            camera_input = task.get("camera_calibration", {}).get("path")
            if camera_input:
                input_documents.append({
                    "role": "camera_calibration",
                    "path": camera_input,
                    "sha256": machine_workflow._sha256(camera_input),
                })
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
                }],
                "directories": [],
                "delivery_files": [{
                    "path": path.name,
                    "size_bytes": path.stat().st_size,
                    "sha256": machine_workflow._sha256(path),
                    "schema_version": "1.0.0",
                } for path in delivery_paths],
            }), encoding="utf-8")
            (side / ".inventory.json").write_text(
                json.dumps(["run_manifest.json", "task_resolved.yaml"]),
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
            / "stereo_260916_2136_pinhole_equi.yaml"
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

    def test_imu_without_stereo_or_managed_camera_result_is_blocked(self):
        self.write_dataset("stereo_imu_260916_2200", imu=True)
        config = self.write_config()
        calls = []
        status = run_machine_workflow(
            self.prefix, config, runner=self._mock_runner(calls))
        self.assertEqual(calls, [])
        self.assertIn(
            "blocked_missing_camera_calibration", json.dumps(status, default=str))
        self.assertIn("prerequisite camera", status["camera_imu"]["message"])

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

    def test_baselink_discovery_supports_existing_names_and_rejects_imus(self):
        first = self.write_dataset("stereo_base_link_261021_1630")
        self.assertEqual(discover_dataset(self.machine, "stereo_baselink"), first)
        second = self.write_dataset("stereo_baselink_261021_1741")
        with self.assertRaisesRegex(TaskError, "multiple.*stereo_baselink"):
            discover_dataset(self.machine, "stereo_baselink")
        self.assertEqual(discover_dataset(self.machine, "stereo_baselink", second.name), second)
        config = self.write_config(extra={"datasets": {"stereo_baselink": second.name}})
        self.assertEqual(load_workflow_config(config)["datasets"]["stereo_baselink"], second.name)
        wrong = self.write_dataset("stereo_baselink_261021_1800", imu=True)
        with self.assertRaisesRegex(TaskError, "IMUs"):
            discover_dataset(self.machine, "stereo_baselink", wrong.name)

    def test_baselink_runs_without_imu_and_reuses_exact_camera_result(self):
        self.write_dataset("stereo_260916_2136")
        config = self.write_config(extra={"camera_baselink": BASELINK_CONFIG})
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(first["camera_baselink"]["status"], "waiting_for_dataset")
        self.write_dataset("stereo_base_link_261021_1630")
        second = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual([call["job"] for call in calls], [
            "camera_calibration", "camera_baselink_calibration"])
        task = calls[-1]["task"]
        self.assertEqual(task["camera_calibration"]["path"], second["camera"]["result"])
        self.assertEqual(task["cameras"], BASELINK_CONFIG["cameras"])
        self.assertEqual(task["base_link"]["position_scale_m"], 0.01)
        self.assertEqual(task["base_link"]["frame_id"], "base_link")
        self.assertNotIn("initialization", task)
        self.assertNotIn("imus", task)
        self.assertEqual(second["camera_imu"]["status"], "waiting_for_dataset")
        self.assertEqual(second["camera_baselink"]["action"], "executed")
        self.assertTrue(second["camera_baselink"]["result"].endswith(
            "stereo_base_link_261021_1630_camera_baselink_cam0_cam1_pinhole_equi.yaml"))
        third = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(third["camera_baselink"]["action"], "reused")
        self.assertEqual(len(calls), 2)
        block = deepcopy(BASELINK_CONFIG)
        block["cameras"][0]["translation_base_camera_m"][0] += 0.01
        with self.assertRaisesRegex(TaskError, "revision"):
            run_machine_workflow(self.prefix, self.write_config(
                extra={"camera_baselink": block}), runner=runner)
        self.assertEqual(len(calls), 2)

    def test_all_three_stages_and_rolling_shutter_mode(self):
        self.write_dataset("stereo_260916_2136")
        self.write_dataset("stereo_imu_260916_2200", imu=True)
        self.write_dataset("stereo_baselink_261021_1741")
        cam0_baselink = {
            "cameras": [{
                "id": "cam0",
                "translation_base_camera_m": [0.1, 0.2, 0.3],
            }],
        }
        config = self.write_config(
            mode="rolling_shutter", camera_mode="rolling_shutter",
            recompute_extrinsics=True, target="aprilgrid_1", revision="v2",
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
            "frame_id": "base_link", "position_scale_m": 0.01})
        for call in calls:
            self.assertEqual(
                Path(call["task"]["target"]["path"]).name,
                "aprilgrid_1.yaml")
        for stage in ("camera", "camera_imu", "camera_baselink"):
            self.assertEqual(summary[stage]["status"], "completed")
            self.assertIn("_rev_v2.yaml", summary[stage]["result"])
        self.assertIn("_rs_pinhole_equi_rev_v2.yaml",
                      summary["camera"]["result"])
        self.assertIn("_camera_baselink_cam0_pinhole_equi_rev_v2.yaml",
                      summary["camera_baselink"]["result"])

    def test_baselink_disabled_missing_configuration_and_missing_camera(self):
        calls = []
        runner = self._mock_runner(calls)
        config = self.write_config()
        self.assertEqual(run_machine_workflow(self.prefix, config, runner=runner)[
            "camera_baselink"]["status"], "disabled")
        self.write_dataset("stereo_base_link_261021_1630")
        missing_config = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(missing_config["camera_baselink"]["status"], "blocked_missing_configuration")
        config = self.write_config(extra={"camera_baselink": BASELINK_CONFIG})
        missing_camera = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(missing_camera["camera_baselink"]["status"], "blocked_missing_camera_calibration")
        self.assertEqual(calls, [])

    def test_default_runner_dispatches_baselink_without_changing_solver_interface(self):
        with mock.patch("kalibr_no_ros.camera_baselink.run_camera_baselink") as run:
            machine_workflow._run_task(self.prefix, "task.yaml", "output",
                                       "camera_baselink_calibration", force=False)
            run.assert_called_once_with("task.yaml", "output", force=False)
        with mock.patch("kalibr_no_ros.task.run_task") as run:
            machine_workflow._run_task(self.prefix, "task.yaml", "output",
                                       "camera_calibration", force=False)
            run.assert_called_once_with(self.prefix, "task.yaml", "output",
                                        "camera_calibration", force=False)

    def test_mono_discovery_and_configuration_are_module_local(self):
        mono = self.write_dataset("usb_261021_1500", cameras=("cam0",))
        baselink = self.write_dataset("usb_baselink_261021_1630", cameras=("cam0",))
        self.write_dataset("usb_test_261021_1700", cameras=("cam0",))
        self.assertEqual(discover_dataset(self.machine, "mono"), mono)
        self.assertEqual(discover_dataset(self.machine, "mono_baselink"), baselink)
        another = self.write_dataset("usb_baselink_261021_1741", cameras=("cam0",))
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
                ("usb_baselink_261021_1630", ("cam0",), True, "IMUs")):
            with self.subTest(name=name):
                dataset = self.write_dataset(name, cameras=cameras, imu=imu)
                role = "mono" if name.startswith("usb_261021") else "mono_baselink"
                with self.assertRaisesRegex(TaskError, message):
                    discover_dataset(self.machine, role, dataset.name)

    def test_usb_directory_names_replace_the_previous_mono_names(self):
        for role, old_names in (
                ("mono", ("mono_261021_1510", "usb_intrinsics_261021_1510")),
                ("mono_baselink", ("mono_base_link_261021_1640", "mono_baselink_261021_1640",
                                   "usb_base_link_261021_1640"))):
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
        self.write_dataset("usb_baselink_261021_1640", cameras=("cam0",))
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
        self.assertEqual(calls[1]["task"]["camera_calibration"]["path"], stereo_result)
        self.assertEqual(calls[2]["task"]["camera_calibration"]["path"], stereo_result)
        self.assertEqual(calls[4]["task"]["camera_calibration"]["path"], mono_result)
        self.assertEqual(calls[4]["task"]["cameras"], MONO_BASELINK_CONFIG["cameras"])
        self.assertEqual(calls[2]["task"]["cameras"], BASELINK_CONFIG["cameras"])
        for index in (3, 4):
            self.assertEqual([camera["id"] for camera in calls[index]["task"]["cameras"]], ["cam0"])
            self.assertNotIn("initialization", calls[index]["task"])
            self.assertNotIn("imus", calls[index]["task"])
            self.assertEqual(calls[index]["task"]["target"], calls[0]["task"]["target"])
        self.assertEqual(load_yaml(mono_result)["cameras"][0]["id"], "cam0")
        self.assertEqual(len(load_yaml(mono_result)["cameras"]), 1)
        reused = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 5)
        for stage in ("camera", "camera_imu", "camera_baselink", "mono_camera", "mono_camera_baselink"):
            self.assertEqual(reused[stage]["action"], "reused")

    def test_mono_can_run_without_stereo_and_resume_baselink_later(self):
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        config = self.write_config(extra={"mono_camera_baselink": MONO_BASELINK_CONFIG})
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 1)
        self.assertEqual(first["mono_camera"]["status"], "completed")
        self.assertEqual(first["mono_camera_baselink"]["status"], "waiting_for_dataset")
        self.write_dataset("usb_baselink_261021_1640", cameras=("cam0",))
        second = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 2)
        self.assertEqual(second["mono_camera"]["action"], "reused")
        self.assertEqual(second["mono_camera_baselink"]["status"], "completed")
        self.assertEqual(second["camera"]["status"], "waiting_for_dataset")
        self.assertTrue(second["mono_camera"]["result"].endswith("usb_261021_1510_pinhole_equi.yaml"))
        self.assertTrue(second["mono_camera_baselink"]["result"].endswith(
            "usb_baselink_261021_1640_camera_baselink_cam0_pinhole_equi.yaml"))

    def test_mono_baselink_never_falls_back_to_stereo_camera(self):
        self.write_dataset("stereo_261021_1500")
        self.write_dataset("usb_baselink_261021_1640", cameras=("cam0",))
        calls = []
        summary = run_machine_workflow(self.prefix, self.write_config(extra={
            "mono_camera_baselink": MONO_BASELINK_CONFIG,
        }), runner=self._mock_runner(calls))
        self.assertEqual(len(calls), 1)
        self.assertEqual(summary["camera"]["status"], "completed")
        self.assertEqual(summary["mono_camera_baselink"]["status"], "blocked_missing_camera_calibration")

    def test_stereo_baselink_never_falls_back_to_mono_camera(self):
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        self.write_dataset("stereo_base_link_261021_1630")
        calls = []
        summary = run_machine_workflow(self.prefix, self.write_config(extra={
            "camera_baselink": BASELINK_CONFIG,
        }), runner=self._mock_runner(calls))
        self.assertEqual(len(calls), 1)
        self.assertEqual(summary["mono_camera"]["status"], "completed")
        self.assertEqual(summary["camera_baselink"]["status"], "blocked_missing_camera_calibration")

    def test_add_mono_after_stereo_and_require_revision_for_mono_config_changes(self):
        self.write_dataset("stereo_261021_1500")
        calls = []
        runner = self._mock_runner(calls)
        first = run_machine_workflow(self.prefix, self.write_config(), runner=runner)
        self.assertEqual(first["mono_camera_baselink"]["status"], "disabled")
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        self.write_dataset("usb_baselink_261021_1640", cameras=("cam0",))
        missing_config = run_machine_workflow(self.prefix, self.write_config(), runner=runner)
        self.assertEqual(missing_config["mono_camera_baselink"]["status"], "blocked_missing_configuration")
        self.assertEqual(len(calls), 2)
        config = self.write_config(extra={"mono_camera_baselink": MONO_BASELINK_CONFIG})
        completed = run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(completed["camera"]["action"], "reused")
        self.assertEqual(completed["mono_camera"]["action"], "reused")
        self.assertEqual(completed["mono_camera_baselink"]["action"], "executed")
        before = Path(completed["mono_camera_baselink"]["result"]).read_bytes()
        changed = deepcopy(MONO_BASELINK_CONFIG)
        changed["cameras"][0]["translation_base_camera_m"][0] += 0.01
        with self.assertRaisesRegex(TaskError, "mono_camera_baselink.*revision"):
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
            state["runs"]["__base__"]["stages"]["camera"]["status"],
            "failed")

        calls = []
        summary = run_machine_workflow(
            self.prefix, config, runner=self._mock_runner(calls))
        self.assertEqual(attempts, ["camera_calibration"])
        self.assertEqual([call["job"] for call in calls],
                         ["camera_calibration"])
        self.assertEqual(summary["camera"]["action"], "executed")
        self.assertEqual(summary["camera"]["status"], "completed")

    def test_changed_provenance_requires_revision_and_never_forces(self):
        dataset = self.write_dataset("stereo_260916_2136")
        config = self.write_config()
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)
        manifest_path = dataset / "dataset.yaml"
        manifest = load_yaml(manifest_path)
        manifest["dataset_id"] = "stereo_260916_2136_changed"
        dump_yaml(manifest, manifest_path)
        with self.assertRaisesRegex((TaskError, ValueError), "revision"):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 1)

        revised = self.write_config(revision="changed_capture")
        run_machine_workflow(self.prefix, revised, runner=runner)
        self.assertEqual(len(calls), 2)
        self.assertIn("rev_changed_capture", calls[1]["task"]["output"]["name"])

    def test_changed_selected_target_requires_revision(self):
        self.write_dataset("stereo_260916_2136")
        config = self.write_config(target="aprilgrid_1")
        calls = []
        runner = self._mock_runner(calls)
        run_machine_workflow(self.prefix, config, runner=runner)
        target = (self.prefix / "share" / "kalibr-noros" / "config"
                  / "mechine" / "aprilgrid_1.yaml")
        document = load_yaml(target)
        document["tagSize"] = 0.066
        dump_yaml(document, target)
        with self.assertRaisesRegex(TaskError, "revision"):
            run_machine_workflow(self.prefix, config, runner=runner)
        self.assertEqual(len(calls), 1)

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
        with self.assertRaisesRegex((TaskError, ValueError), "revision|checkpoint"):
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
        state["runs"]["__base__"]["stages"]["camera"] = {
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
        state["runs"]["__base__"]["stages"]["camera"] = {
            "status": "running",
        }
        state_path.write_text(
            json.dumps(state, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")

        report = (self.machine / "result"
                  / "stereo_260916_2136_pinhole_equi.report.html")
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
            state["runs"]["__base__"]["stages"]["camera"]["status"],
            "failed")


if __name__ == "__main__":
    unittest.main()
