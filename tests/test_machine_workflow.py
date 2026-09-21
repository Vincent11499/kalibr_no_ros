"""Machine-level staged calibration orchestration without invoking a solver."""

import json
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
                     mode="standard", revision=None, extra=None):
        document = {
            "schema_version": "1.0.0",
            "kind": "machine_calibration_workflow",
            "machine_dir": str(self.machine),
            "camera_imu": {"mode": mode},
        }
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
            "camera_imu": {"mode": "rolling_shutter"},
            "datasets": {"stereo": "stereo_260916_2136"},
            "revision": "rerun_2",
        }, path)
        loaded = load_workflow_config(path)
        self.assertEqual(Path(loaded["machine_dir"]), relative_machine.resolve())
        self.assertEqual(loaded["camera_imu"]["mode"], "rolling_shutter")
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

        imu_dataset = self.write_dataset("stereo_imu_260916_2200", imu=True)
        exact_result = self.machine / "result" / "selected_camera.yaml"
        standard = build_task(
            PROFILE, "camera_imu", imu_dataset, "standard_result",
            camera_result=exact_result, mode="standard")
        rolling = build_task(
            PROFILE, "camera_imu", imu_dataset, "rolling_result",
            camera_result=exact_result, mode="rolling_shutter")
        self.assertEqual(standard["job"], "camera_imu_calibration")
        self.assertEqual(
            rolling["job"], "camera_imu_rolling_shutter_calibration")
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
                    "cameras" if expected_job == "camera_calibration"
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
            if expected_job != "camera_calibration":
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
