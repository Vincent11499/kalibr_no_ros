"""Named deliveries and input selection, using temporary files and no native solver."""

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/python"))

from kalibr_no_ros.delivery import (
    apply_camera_ids, discover_camera_result, enrich_result, publish, result_name,
)
from kalibr_no_ros.evaluation import validate_output_options
from kalibr_no_ros.task import TaskError, dump_yaml, load_task, run_task


class DeliveryTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.output = self.root / "output"
        self.output.mkdir()
        self.stage = self.root / "stage"
        self.stage.mkdir()
        self.dataset = self.root / "dataset"
        cameras = []
        for camera_id in ("cam1", "cam8"):
            images = self.dataset / camera_id
            images.mkdir(parents=True)
            (images / "timestamps.csv").write_text("timestamp_ns,filename\n")
            cameras.append({"id": camera_id, "images": camera_id,
                            "timestamps": camera_id + "/timestamps.csv"})
        dump_yaml({"schema_version": "1.0.0", "type": "kalibr_directory_dataset",
                   "dataset_id": "capture", "cameras": cameras, "imus": []},
                  self.dataset / "dataset.yaml")

    def camera_result(self, ids=("cam1", "cam8")):
        cameras = []
        for index, camera_id in enumerate(ids):
            camera = {"id": camera_id, "rostopic": camera_id, "camera_model": "pinhole",
                      "distortion_model": "equidistant", "intrinsics": [200., 205., 320., 240.],
                      "distortion_coeffs": [0., 0., 0., 0.], "resolution": [640, 480]}
            if index:
                camera["T_cn_cnm1"] = [[1., 0., 0., -0.1], [0., 1., 0., 0.],
                                         [0., 0., 1., 0.], [0., 0., 0., 1.]]
            cameras.append(camera)
        return {"schema_version": "1.0.0", "kind": "calibration_result",
                "calibration_type": "cameras", "cameras": cameras}

    def task(self, *, imu=False, ids=("cam1", "cam8"), output=None):
        value = {"schema_version": "1.0.0", "kind": "calibration_task",
                 "job": "camera_imu_calibration" if imu else "camera_calibration",
                 "dataset": {"type": "directory", "path": str(self.dataset)},
                 "target": {"type": "aprilgrid", "parameters": {
                     "tagRows": 3, "tagCols": 3, "tagSize": 0.04, "tagSpacing": 0.3}},
                 "output": output or {}}
        if imu:
            value["imus"] = [{"id": "imu0", "path": "imu.yaml", "model": "calibrated"}]
        else:
            value["cameras"] = [{"id": camera_id, "model": "pinhole-equi"} for camera_id in ids]
        path = self.root / ("imu_task.yaml" if imu else "camera_task.yaml")
        dump_yaml(value, path)
        return path, load_task(path)

    def populate_stage(self, stage=None, result=None):
        stage = self.stage if stage is None else Path(stage)
        stage.mkdir(parents=True, exist_ok=True)
        dump_yaml(self.camera_result() if result is None else result, stage / "calibration.yaml")
        (stage / "report.html").write_text(
            '<html><ul><li><a href="calibration.yaml">Result</a></li>'
            '<li><a href="metrics.json">Metrics</a></li></ul></html>')
        (stage / "report.pdf").write_bytes(b"%PDF-1.4\nfixture")
        (stage / "results.txt").write_text("Native result summary")
        (stage / "metrics.json").write_text('{"cameras": {}}')
        (stage / "validation.json").write_text('{"status": "passed"}')
        (stage / "stdout.log").write_text("Private solver output")
        return stage

    def test_result_names_preserve_mono_stereo_and_imu_ids(self):
        _, mono = self.task(ids=("cam1",))
        self.assertEqual(result_name(mono), "camera_calibration_cam1")
        _, stereo = self.task()
        self.assertEqual(result_name(stereo), "camera_calibration_cam1_cam8")
        result = self.root / "stereo.yaml"
        dump_yaml(self.camera_result(), result)
        _, imu = self.task(imu=True)
        imu["camera_calibration"] = {"path": str(result)}
        self.assertEqual(result_name(imu), "camera_imu_calibration_cam1_cam8_imu0")
        imu["output"]["name"] = "custom_run"
        self.assertEqual(result_name(imu), "custom_run")

    def test_default_delivery_has_only_named_result_and_two_reports(self):
        self.populate_stage()
        publish(self.stage, self.output, "mono_cam1", validate_output_options())
        self.assertEqual({p.name for p in self.output.iterdir()},
                         {"mono_cam1.yaml", "mono_cam1.report.html", "mono_cam1.report.pdf"})
        html = (self.output / "mono_cam1.report.html").read_text()
        self.assertIn('href="mono_cam1.yaml"', html)
        self.assertNotIn('href="metrics.json"', html)

    def test_independent_deliveries_and_force_preserve_other_tasks_and_user_files(self):
        self.populate_stage()
        publish(self.stage, self.output, "mono_cam1", {})
        publish(self.stage, self.output, "stereo_cam1_cam8", {})
        other = {p.name: p.read_bytes() for p in self.output.glob("stereo*")}
        note = self.output / "notes.txt"
        note.write_text("Do not delete")
        with self.assertRaisesRegex(TaskError, "already exists"):
            publish(self.stage, self.output, "mono_cam1", {})
        (self.stage / "report.pdf").write_bytes(b"%PDF-new")
        publish(self.stage, self.output, "mono_cam1", {}, force=True)
        self.assertEqual((self.output / "mono_cam1.report.pdf").read_bytes(), b"%PDF-new")
        self.assertEqual({p.name: p.read_bytes() for p in self.output.glob("stereo*")}, other)
        self.assertEqual(note.read_text(), "Do not delete")

    def test_metrics_and_text_can_be_saved_without_diagnostics(self):
        self.populate_stage()
        publish(self.stage, self.output, "stereo", {"save_metrics": True, "export_text": True})
        side = self.output / "stereo"
        self.assertEqual({p.name for p in side.iterdir()}, {"metrics.json", ".inventory.json"})
        self.assertEqual((self.output / "stereo.results.txt").read_text(), "Native result summary")
        self.assertIn('href="stereo/metrics.json"', (self.output / "stereo.report.html").read_text())

    def test_diagnostics_and_visualizations_are_kept_in_task_subdirectory(self):
        self.populate_stage()
        image = self.stage / "visualizations" / "cam1" / "corners_3.jpg"
        image.parent.mkdir(parents=True)
        image.write_bytes(b"fixture image")
        (self.stage / "report.html").write_text('<img src="visualizations/cam1/corners_3.jpg">')
        publish(self.stage, self.output, "stereo", {"save_diagnostics": True,
                                                     "visualizations": {"enabled": True}})
        self.assertTrue((self.output / "stereo/validation.json").is_file())
        self.assertTrue((self.output / "stereo/stdout.log").is_file())
        self.assertEqual((self.output / "stereo/visualizations/cam1/corners_3.jpg").read_bytes(),
                         b"fixture image")
        self.assertIn('src="stereo/visualizations/cam1/corners_3.jpg"',
                      (self.output / "stereo.report.html").read_text())

    def test_force_rejects_unregistered_sidecar_without_modifying_delivery(self):
        self.populate_stage()
        publish(self.stage, self.output, "stereo", {"save_metrics": True})
        foreign = self.output / "stereo/user-note.txt"
        foreign.write_text("owned by user")
        previous = (self.output / "stereo.yaml").read_bytes()
        with self.assertRaisesRegex(TaskError, "unregistered"):
            publish(self.stage, self.output, "stereo", {}, force=True)
        self.assertEqual(foreign.read_text(), "owned by user")
        self.assertEqual((self.output / "stereo.yaml").read_bytes(), previous)

    def test_force_rejects_main_or_sidecar_symlink_and_preserves_target(self):
        self.populate_stage()
        external = self.root / "external.txt"
        external.write_text("outside data")
        main = self.output / "mono.yaml"
        main.symlink_to(external)
        with self.assertRaisesRegex(TaskError, "symlink"):
            publish(self.stage, self.output, "mono", {}, force=True)
        self.assertEqual(external.read_text(), "outside data")
        main.unlink()
        publish(self.stage, self.output, "stereo", {"save_metrics": True})
        metrics = self.output / "stereo/metrics.json"
        metrics.unlink()
        metrics.symlink_to(external)
        with self.assertRaisesRegex(TaskError, "unregistered|symlink"):
            publish(self.stage, self.output, "stereo", {}, force=True)
        self.assertTrue(metrics.is_symlink())
        self.assertEqual(external.read_text(), "outside data")

    def test_discovery_selects_unique_compatible_result_without_modifying_explicit_input(self):
        _, task = self.task(imu=True)
        candidate = self.output / "stereo.yaml"
        dump_yaml(self.camera_result(), candidate)
        dump_yaml(self.camera_result(("other0", "other1")), self.output / "other_rig.yaml")
        malformed = self.camera_result()
        malformed["cameras"][0]["id"] = ["not", "a", "sensor", "id"]
        dump_yaml(malformed, self.output / "invalid_id.yaml")
        (self.output / "invalid.yaml").write_text("[invalid yaml")
        discover_camera_result(task, self.output)
        self.assertEqual(task["camera_calibration"], {"path": str(candidate)})
        task["camera_calibration"] = {"path": "explicit.yaml"}
        discover_camera_result(task, self.output)
        self.assertEqual(task["camera_calibration"], {"path": "explicit.yaml"})

    def test_discovery_rejects_missing_wrong_ids_and_ambiguous_results(self):
        _, task = self.task(imu=True)
        for wrong in (None, ("cam0", "cam2"), ("cam1",)):
            with self.subTest(candidate_ids=wrong):
                if wrong is not None:
                    dump_yaml(self.camera_result(wrong), self.output / "wrong.yaml")
                with self.assertRaisesRegex(TaskError, "none found"):
                    discover_camera_result(task, self.output)
                self.assertNotIn("camera_calibration", task)
        for name in ("first.yaml", "second.yaml"):
            dump_yaml(self.camera_result(), self.output / name)
        with self.assertRaisesRegex(TaskError, "first.yaml.*second.yaml"):
            discover_camera_result(task, self.output)
        self.assertNotIn("camera_calibration", task)

    def test_discovery_ignores_nonfinite_candidate_and_uses_only_valid_result(self):
        _, task = self.task(imu=True)
        invalid = self.camera_result()
        invalid["cameras"][0]["intrinsics"][0] = float("nan")
        dump_yaml(invalid, self.output / "invalid.yaml")
        with self.assertRaisesRegex(TaskError, "none found"):
            discover_camera_result(task, self.output)
        candidate = self.output / "valid.yaml"
        dump_yaml(self.camera_result(), candidate)
        discover_camera_result(task, self.output)
        self.assertEqual(task["camera_calibration"], {"path": str(candidate)})

    def test_run_resolves_camera_result_before_staging_and_keeps_configuration(self):
        config, _ = self.task(imu=True)
        before = config.read_bytes()
        camera_path = self.output / "camera_calibration_cam1_cam8.yaml"
        dump_yaml(self.camera_result(), camera_path)
        camera_bytes = camera_path.read_bytes()

        def staged(prefix, config, stage, job, **kwargs):
            self.assertEqual(job, "camera_imu_calibration")
            self.assertEqual(kwargs["_task"]["camera_calibration"], {"path": str(camera_path)})
            result = self.camera_result()
            result["calibration_type"] = "camera_imu"
            self.populate_stage(stage, result)

        with mock.patch("kalibr_no_ros.task._run_task_staged", side_effect=staged) as solver:
            run_task(self.root / "unused", config, self.output, "camera_imu_calibration")
        solver.assert_called_once()
        self.assertEqual(config.read_bytes(), before)
        self.assertEqual(camera_path.read_bytes(), camera_bytes)
        self.assertTrue((self.output / "camera_imu_calibration_cam1_cam8_imu0.yaml").is_file())

    def test_force_must_not_overwrite_the_task_configuration(self):
        _, task = self.task(ids=("cam1",), output={"name": "task"})
        config = self.output / "task.yaml"
        dump_yaml({key: value for key, value in task.items() if not key.startswith("_")}, config)
        before = config.read_bytes()
        with mock.patch("kalibr_no_ros.task._run_task_staged",
                        side_effect=lambda prefix, config, stage, job, **kw: self.populate_stage(stage)) as solver:
            with self.assertRaisesRegex(TaskError, "input"):
                run_task(self.root / "unused", config, self.output, "camera_calibration", force=True)
        solver.assert_not_called()
        self.assertEqual(config.read_bytes(), before)

    def test_input_yaml_with_report_or_text_extension_cannot_be_overwritten(self):
        for suffix in (".report.html", ".results.txt"):
            with self.subTest(input_suffix=suffix):
                _, task = self.task(ids=("cam1",), output={"name": "task"})
                config = self.output / ("task" + suffix)
                dump_yaml({key: value for key, value in task.items() if not key.startswith("_")}, config)
                before = config.read_bytes()
                with mock.patch("kalibr_no_ros.task._run_task_staged") as solver:
                    with self.assertRaisesRegex(TaskError, "overwrite its input"):
                        run_task(self.root / "unused", config, self.output, "camera_calibration", force=True)
                solver.assert_not_called()
                self.assertEqual(config.read_bytes(), before)

    def test_repeated_failures_keep_each_diagnostic_and_the_original_exception(self):
        config, _ = self.task(ids=("cam1",), output={"name": "repeat", "save_diagnostics": True})
        self.populate_stage()
        publish(self.stage, self.output, "repeat", {})
        success = {p.name: p.read_bytes() for p in self.output.iterdir()}
        failures = [RuntimeError("first sentinel"), RuntimeError("second sentinel")]

        def fail(prefix, config, stage, job, **kwargs):
            stage = Path(stage)
            stage.mkdir()
            error = failures[solver.call_count - 1]
            (stage / "stdout.log").write_text(str(error))
            (stage / "run_manifest.json").write_text(json.dumps({
                "schema_version": "1.0.0", "status": "failed",
                "failure": {"type": "RuntimeError", "message": str(error)},
            }))
            raise error

        first = self.output / "repeat_failed"
        with mock.patch("kalibr_no_ros.task._run_task_staged", side_effect=fail) as solver:
            for index, expected in enumerate(failures):
                with self.assertRaises(RuntimeError) as caught:
                    run_task(self.root / "unused", config, self.output, "camera_calibration", force=True)
                self.assertIs(caught.exception, expected)
                if index == 0:
                    first_files = {p.relative_to(first).as_posix(): p.read_bytes()
                                   for p in first.rglob("*") if p.is_file()}
        self.assertEqual(solver.call_count, 2)
        self.assertEqual({p.relative_to(first).as_posix(): p.read_bytes()
                          for p in first.rglob("*") if p.is_file()}, first_files)
        diagnostic_directories = list(self.output.glob("repeat_failed*"))
        self.assertEqual(len(diagnostic_directories), 2)
        second = next(p for p in diagnostic_directories if p != first)
        self.assertTrue(second.name.startswith("repeat_failed_"))
        self.assertEqual((first / "stdout.log").read_text(), "first sentinel")
        self.assertEqual((second / "stdout.log").read_text(), "second sentinel")
        self.assertEqual({name: (self.output / name).read_bytes() for name in success}, success)

    def test_public_ids_and_quality_summaries_preserve_missing_evidence(self):
        _, task = self.task()
        result = self.camera_result(("cam0", "cam1"))
        result["transform_convention"] = "p_target = T_target_source * p_source"
        artifacts = {"cameras": [{"id": "cam0", "frames": [{"frame_id": "cam0:7"}]},
                                   {"id": "cam1", "frames": [{"frame_id": "cam1:9"}]}],
                     "views": [{"frame_ids": ["cam0:7", "cam1:9"]}],
                     "events": [{"frame_id": "cam0:7"}]}
        apply_camera_ids(task, artifacts, result)
        self.assertEqual([c["id"] for c in result["cameras"]], ["cam1", "cam8"])
        self.assertEqual(artifacts["views"][0]["frame_ids"], ["cam1:7", "cam8:9"])
        self.assertEqual(artifacts["events"][0]["frame_id"], "cam1:7")
        alignment = {"status": "available", "count": 4, "unit": "px", "rms_px": 0.3}
        metrics = {"cameras": {
            "cam1": {"reprojection": {"status": "available", "count": 4, "rms_px": 0.5}},
            "cam8": {"reprojection": {"status": "unavailable", "count": 0, "rms_px": None}}},
            "stereo_pairs": {"cam1_cam8": {"alignment": alignment}}}
        enrich_result(result, metrics)
        self.assertNotIn("transform_convention", result)
        self.assertEqual(result["cameras"][0]["rms"], 0.5)
        self.assertIsNone(result["cameras"][1]["rms"])
        self.assertNotIn("from_camera", result["cameras"][1])
        self.assertEqual(result["cameras"][1]["alignment"], 0.3)
        no_pairs = copy.deepcopy(metrics)
        no_pairs["stereo_pairs"] = {}
        enrich_result(result, no_pairs)
        self.assertIsNone(result["cameras"][1]["alignment"])


if __name__ == "__main__":
    unittest.main()
