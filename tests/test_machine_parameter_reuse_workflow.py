"""Cross-computer machine parameters can replace individual local stages."""

from copy import deepcopy
import json
from pathlib import Path
import unittest

import test_machine_workflow as workflow_fixtures
from test_machine_param_reuse import public_parameters

from kalibr_no_ros.initialization import _inverse_transform
from kalibr_no_ros.machine_param_reuse import parameter_path
from kalibr_no_ros.machine_workflow import load_workflow_config, run_machine_workflow
from kalibr_no_ros.task import TaskError, dump_yaml, load_yaml
from kalibr_no_ros.filesystem import file_sha256


STAGES = (
    "camera", "camera_imu", "camera_baselink",
    "mono_camera", "mono_camera_baselink",
)


class MachineParameterReuseWorkflowTest(unittest.TestCase):
    setUp = workflow_fixtures.MachineWorkflowTest.setUp
    write_dataset = workflow_fixtures.MachineWorkflowTest.write_dataset
    write_config = workflow_fixtures.MachineWorkflowTest.write_config
    profile_file = workflow_fixtures.MachineWorkflowTest.profile_file
    _mock_runner = workflow_fixtures.MachineWorkflowTest._mock_runner

    def _parameters(self, mono=False):
        document = public_parameters(mono=mono)
        cameras = ([document["camera"]] if mono else
                   [document["camera"][camera_id] for camera_id in ("cam0", "cam1")])
        for camera in cameras:
            camera.update({
                "resolution": [640, 480],
                "intrinsic": [1.0, 1.0, 1.0, 1.0],
                "distortion": [0.0, 0.0, 0.0, 0.0],
            })
        cameras[0]["T_base_cam"] = (
            [[-1.0, 0.0, 0.0, -0.21306], [0.0, 1.0, 0.0, 0.0],
             [0.0, 0.0, -1.0, 0.30887], [0.0, 0.0, 0.0, 1.0]]
            if mono else
            [[1.0, 0.0, 0.0, 0.2], [0.0, -1.0, 0.0, 0.1],
             [0.0, 0.0, -1.0, 0.3], [0.0, 0.0, 0.0, 1.0]])
        cameras[0]["pitch"] = 90.0
        if not mono:
            document["camera"]["cam1"]["T_imu_cam"][0][3] = 0.1
            document["camera"]["cam1"]["T_cn_cnm1"][0][3] = -0.1
            document["camera"]["cam0_cam1_correct"]["baseline"] = 0.1
        return document

    def _copy_parameters(self):
        for stage, mono in (("camera", False), ("mono_camera", True)):
            target = parameter_path(self.machine / "result", stage)
            target.parent.mkdir(parents=True, exist_ok=True)
            dump_yaml(self._parameters(mono=mono), target)

    def _config(self, reuse):
        return self.write_config(extra={
            "reuse_calib_params": reuse,
            "camera_baselink": workflow_fixtures.BASELINK_CONFIG,
            "mono_camera_baselink": workflow_fixtures.MONO_BASELINK_CONFIG,
        })

    def _all_local_datasets(self):
        self.write_dataset("stereo_261021_1500")
        self.write_dataset("stereo_imu_261021_1600", imu=True)
        self.write_dataset("stereo_base_link_261021_1630")
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        self.write_dataset("usb_base_link_261021_1640", cameras=("cam0",))

    def _runner(self, calls):
        mock_runner = self._mock_runner(calls)

        def run(prefix, config, output_dir, expected_job, force=False, **overrides):
            result = mock_runner(prefix, config, output_dir, expected_job,
                                 force=force, **overrides)
            task = load_yaml(config)
            if "camera_calibration" not in task:
                return result
            seed = load_yaml(task["camera_calibration"]["path"])
            by_id = {camera["id"]: camera for camera in seed["cameras"]}
            name = task["output"]["name"]
            side = Path(output_dir) / name
            path = side / (name + ".yaml")
            document = load_yaml(path)
            for camera in document["cameras"]:
                source = by_id[camera["id"]]
                for field in ("camera_model", "distortion_model", "intrinsics",
                              "distortion_coeffs", "resolution", "T_cn_cnm1"):
                    if field in source:
                        camera[field] = deepcopy(source[field])
                if camera["id"] == "cam1" and "T_imu_cam" in camera:
                    camera["T_imu_cam"] = _inverse_transform(camera["T_cn_cnm1"])
            dump_yaml(document, path)
            manifest_path = side / "run_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            for entry in manifest["delivery_files"]:
                if entry["path"] == path.name:
                    entry["size_bytes"] = path.stat().st_size
                    entry["sha256"] = file_sha256(path)
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            return result

        return run

    def _state(self):
        return json.loads((self.machine / "result" / ".kalibr-cedarobot"
                           / "state.json").read_text(encoding="utf-8"))

    def _independent_switch(self, stage):
        self._copy_parameters()
        self._all_local_datasets()
        calls = []
        summary = run_machine_workflow(
            self.prefix, self._config({stage: True}), runner=self._runner(calls))
        self.assertEqual(len(calls), 4)
        self.assertEqual(summary[stage]["status"], "reused_parameters")
        self.assertEqual(summary[stage]["action"], "reused_parameters")
        ignored_name = {
            "camera": "stereo_261021_1500",
            "camera_imu": "stereo_imu_261021_1600",
            "camera_baselink": "stereo_base_link_261021_1630",
            "mono_camera": "usb_261021_1510",
            "mono_camera_baselink": "usb_base_link_261021_1640",
        }[stage]
        self.assertNotIn(ignored_name, [call["task"]["output"]["name"] for call in calls])
        if stage in {"camera_baselink", "mono_camera_baselink"}:
            mono = stage == "mono_camera_baselink"
            key = "mono" if mono else "vision"
            final = load_yaml(summary["calib_params"][key]["result"])["camera"]
            selected = final if mono else final["cam0"]
            source = self._parameters(mono=mono)["camera"]
            source = source if mono else source["cam0"]
            self.assertEqual(selected["T_base_cam"], source["T_base_cam"])
            self.assertEqual(selected["pitch"], source["pitch"])

    def test_camera_reuse_skips_its_local_dataset(self):
        self._independent_switch("camera")

    def test_camera_imu_reuse_skips_its_local_dataset(self):
        self._independent_switch("camera_imu")

    def test_camera_baselink_reuse_skips_its_local_dataset(self):
        self._independent_switch("camera_baselink")

    def test_mono_camera_reuse_skips_its_local_dataset(self):
        self._independent_switch("mono_camera")

    def test_mono_baselink_reuse_skips_its_local_dataset(self):
        self._independent_switch("mono_camera_baselink")

    def test_all_reused_stages_need_no_datasets_or_solver(self):
        self._copy_parameters()
        calls = []
        config = self._config({stage: True for stage in STAGES})
        first = run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual(calls, [])
        for stage in STAGES:
            self.assertEqual(first[stage]["status"], "reused_parameters")
        self.assertEqual(set(first["calib_params"]), {"vision", "mono"})
        before = {key: Path(record["result"]).read_bytes()
                  for key, record in first["calib_params"].items()}
        second = run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual(calls, [])
        for key, payload in before.items():
            self.assertEqual(Path(second["calib_params"][key]["result"]).read_bytes(), payload)
        self.assertEqual(set(self._state()["parameter_reuse"]), set(STAGES))

    def test_reused_camera_drives_imu_and_baselink_without_source_captures(self):
        self._copy_parameters()
        self.write_dataset("stereo_imu_261021_1600", imu=True)
        self.write_dataset("stereo_base_link_261021_1630")
        calls = []
        config = self._config({"camera": True})
        summary = run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual([call["job"] for call in calls], [
            "camera_imu_calibration", "camera_baselink_calibration"])
        seed_path = Path(calls[0]["task"]["camera_calibration"]["path"])
        self.assertEqual(calls[1]["task"]["camera_calibration"]["path"], str(seed_path))
        self.assertTrue(seed_path.is_file())
        self.assertFalse((self.machine / "stereo_261021_1500").exists())
        vision = load_yaml(summary["calib_params"]["vision"]["result"])
        self.assertEqual(vision["imu"]["gyro_bias"], [0.001, 0.002, 0.003])
        self.assertEqual(vision["camera"]["cam0"]["T_base_cam"][0][3], 0.2)
        run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual(len(calls), 2)

    def test_reused_mono_drives_only_usb_baselink(self):
        self._copy_parameters()
        self.write_dataset("usb_base_link_261021_1640", cameras=("cam0",))
        calls = []
        summary = run_machine_workflow(
            self.prefix, self._config({"mono_camera": True}),
            runner=self._runner(calls))
        self.assertEqual([call["job"] for call in calls], ["camera_baselink_calibration"])
        self.assertEqual([camera["id"] for camera in calls[0]["task"]["cameras"]], ["cam0"])
        self.assertEqual(summary["camera"]["status"], "waiting_for_dataset")
        mono = load_yaml(summary["calib_params"]["mono"]["result"])
        self.assertEqual(mono["camera"]["T_base_cam"][0][3], 0.2)

    def test_missing_reused_parameters_fail_before_starting_local_solver(self):
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        calls = []
        with self.assertRaises(TaskError):
            run_machine_workflow(self.prefix, self._config({"camera": True}),
                                 runner=self._runner(calls))
        self.assertEqual(calls, [])

    def test_invalid_copied_parameters_are_not_replaced_or_used(self):
        self._copy_parameters()
        target = parameter_path(self.machine / "result", "camera")
        document = load_yaml(target)
        document["machine_num"] = "12"
        dump_yaml(document, target)
        before = target.read_bytes()
        self.write_dataset("usb_261021_1510", cameras=("cam0",))
        calls = []
        with self.assertRaisesRegex(TaskError, "machine"):
            run_machine_workflow(self.prefix, self._config({"camera": True}),
                                 runner=self._runner(calls))
        self.assertEqual(calls, [])
        self.assertEqual(target.read_bytes(), before)

    def test_new_copied_geometry_reruns_consumers_and_preserves_old_seed(self):
        self._copy_parameters()
        self.write_dataset("stereo_imu_261021_1600", imu=True)
        self.write_dataset("stereo_base_link_261021_1630")
        calls = []
        config = self._config({"camera": True})
        first = run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual(len(calls), 2)
        old_seed = Path(calls[0]["task"]["camera_calibration"]["path"])
        old_bytes = old_seed.read_bytes()
        target = Path(first["calib_params"]["vision"]["result"])
        document = load_yaml(target)
        document["camera"]["cam0"]["intrinsic"][0] = 1.25
        dump_yaml(document, target)
        second = run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual(len(calls), 4)
        self.assertEqual(second["camera_imu"]["action"], "executed")
        self.assertEqual(second["camera_baselink"]["action"], "executed")
        new_seed = Path(calls[2]["task"]["camera_calibration"]["path"])
        self.assertNotEqual(new_seed, old_seed)
        self.assertEqual(old_seed.read_bytes(), old_bytes)
        self.assertEqual(load_yaml(new_seed)["cameras"][0]["intrinsics"][0], 1.25)
        run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual(len(calls), 4)

    def test_bias_baselink_and_comment_changes_do_not_rerun_geometry_consumers(self):
        self._copy_parameters()
        self.write_dataset("stereo_imu_261021_1600", imu=True)
        calls = []
        config = self._config({"camera": True})
        first = run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        target = Path(first["calib_params"]["vision"]["result"])
        document = load_yaml(target)
        document["imu"]["gyro_bias"][0] += 0.01
        document["camera"]["cam0"]["T_base_cam"][0][3] += 0.01
        dump_yaml(document, target)
        with target.open("a", encoding="utf-8") as stream:
            stream.write("\n# copied parameter annotations\n")
        run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual(len(calls), 1)

    def test_annotations_do_not_reimport_a_locally_refined_baseline(self):
        self._copy_parameters()
        self.write_dataset("stereo_imu_261021_1600", imu=True)
        calls = []
        base_runner = self._runner(calls)

        def refine_baseline(prefix, config, output, job, force=False, **overrides):
            result = base_runner(prefix, config, output, job,
                                 force=force, **overrides)
            name = load_yaml(config)["output"]["name"]
            side = Path(output) / name
            path = side / (name + ".yaml")
            document = load_yaml(path)
            document["cameras"][1]["T_cn_cnm1"][0][3] -= 0.001
            document["cameras"][1]["T_imu_cam"][0][3] += 0.001
            dump_yaml(document, path)
            manifest_path = side / "run_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            for entry in manifest["delivery_files"]:
                if entry["path"] == path.name:
                    entry.update(size_bytes=path.stat().st_size,
                                 sha256=file_sha256(path))
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            return result

        config = self._config({"camera": True})
        first = run_machine_workflow(self.prefix, config, runner=refine_baseline)
        seed = Path(calls[0]["task"]["camera_calibration"]["path"])
        target = Path(first["calib_params"]["vision"]["result"])
        self.assertEqual(load_yaml(seed)["cameras"][1]["T_cn_cnm1"][0][3], -0.1)
        self.assertAlmostEqual(
            load_yaml(target)["camera"]["cam1"]["T_cn_cnm1"][0][3], -0.101)
        with target.open("a", encoding="utf-8") as stream:
            stream.write("\n# only an annotation after local baseline refinement\n")
        second = run_machine_workflow(self.prefix, config, runner=refine_baseline)
        self.assertEqual(len(calls), 1)
        self.assertEqual(second["camera_imu"]["action"], "reused")
        document = load_yaml(target)
        document["imu"]["gyro_bias"][0] += 0.01
        document["camera"]["cam0"]["T_base_cam"][0][3] += 0.01
        dump_yaml(document, target)
        third = run_machine_workflow(self.prefix, config, runner=refine_baseline)
        self.assertEqual(len(calls), 1)
        self.assertEqual(third["camera_imu"]["action"], "reused")
        self.assertEqual(load_yaml(seed)["cameras"][1]["T_cn_cnm1"][0][3], -0.1)
        for field, increment in (("rms", 0.01), ("line_delay_s", 0.000001)):
            with self.subTest(field=field):
                document = load_yaml(target)
                document["camera"]["cam0"][field] += increment
                dump_yaml(document, target)
                current = run_machine_workflow(
                    self.prefix, config, runner=refine_baseline)
                self.assertEqual(len(calls), 1)
                self.assertEqual(current["camera_imu"]["action"], "reused")
                self.assertEqual(
                    load_yaml(seed)["cameras"][1]["T_cn_cnm1"][0][3], -0.1)

    def test_equi_alias_is_compatible_with_a_local_imu_result(self):
        self._copy_parameters()
        self.write_dataset("stereo_imu_261021_1600", imu=True)
        target = parameter_path(self.machine / "result", "camera")
        document = load_yaml(target)
        for camera_id in ("cam0", "cam1"):
            document["camera"][camera_id]["model"] = "pinhole-equi"
        dump_yaml(document, target)
        calls = []
        summary = run_machine_workflow(
            self.prefix, self._config({"camera": True}),
            runner=self._runner(calls))
        self.assertEqual(len(calls), 1)
        self.assertEqual(summary["camera_imu"]["status"], "completed")
        self.assertEqual(
            load_yaml(target)["camera"]["cam0"]["model"],
            "pinhole-equidistant")

    def test_failed_consumer_preserves_copied_values_and_retries_normally(self):
        self._copy_parameters()
        self.write_dataset("stereo_imu_261021_1600", imu=True)
        target = parameter_path(self.machine / "result", "camera")
        before = target.read_bytes()
        config = self._config({"camera": True})

        def failed_runner(*unused_arguments, **unused_overrides):
            raise RuntimeError("consumer solver failure")

        with self.assertRaisesRegex(RuntimeError, "consumer solver failure"):
            run_machine_workflow(self.prefix, config, runner=failed_runner)
        self.assertEqual(target.read_bytes(), before)
        calls = []
        summary = run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual([call["job"] for call in calls], ["camera_imu_calibration"])
        self.assertEqual(summary["camera"]["status"], "reused_parameters")
        run_machine_workflow(self.prefix, config, runner=self._runner(calls))
        self.assertEqual(len(calls), 1)

    def test_disabling_reuse_with_original_data_restores_local_stages(self):
        self._copy_parameters()
        self.write_dataset("stereo_imu_261021_1600", imu=True)
        calls = []
        run_machine_workflow(self.prefix, self._config({"camera": True}),
                             runner=self._runner(calls))
        self.assertEqual([call["job"] for call in calls], ["camera_imu_calibration"])
        self.write_dataset("stereo_261021_1500")
        summary = run_machine_workflow(
            self.prefix, self._config({}), runner=self._runner(calls))
        self.assertEqual(summary["camera"]["action"], "executed")
        self.assertEqual([call["job"] for call in calls], [
            "camera_imu_calibration", "camera_calibration", "camera_imu_calibration"])

    def test_switches_are_independent_strict_booleans_and_default_false(self):
        config = self.write_config()
        self.assertEqual(load_workflow_config(config)["reuse_calib_params"],
                         {stage: False for stage in STAGES})
        for value in (True, [], {"unknown": True}, {"camera": 1}, {"camera": "true"}):
            with self.subTest(value=value):
                with self.assertRaises(TaskError):
                    load_workflow_config(self.write_config(extra={"reuse_calib_params": value}))


if __name__ == "__main__":
    unittest.main()
