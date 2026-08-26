import tempfile
import unittest
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_no_ros.cli import build_parser, main
from kalibr_no_ros.reference import verify_snapshot
from kalibr_no_ros.task import (
    STANDARD_EXECUTION,
    TaskError,
    load_task,
    prepare_output_directory,
    resolve_execution,
)


class TaskCliTest(unittest.TestCase):
    def test_convert_camera_help_is_forwarded(self):
        arguments = build_parser().parse_args(["convert", "camera", "--help"])
        self.assertTrue(arguments.converter_help)
        self.assertEqual(arguments.arguments, [])

    def test_reference_snapshot_is_exact(self):
        result = verify_snapshot(
            ROOT / "ref" / "kalibr", ROOT / "ref" / "kalibr.sha256", 1630
        )
        self.assertTrue(result["valid"])

    def test_convert_camera_job_to_strict_task(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "camera-task.yaml"
            code = main([
                "convert", "job", "--type", "cameras",
                "--bag", "/data/input.bag",
                "--target", "/data/target.yaml",
                "--topics", "/cam0", "/cam1",
                "--models", "pinhole-radtan5", "pinhole-radtan5",
                "--no-shuffle", "--output", str(output),
            ], prefix=ROOT)
            self.assertEqual(code, 0)
            task = load_task(output, "camera_calibration")
            self.assertEqual(len(task["cameras"]), 2)
            self.assertEqual(task["schema_version"], 1)
            self.assertEqual(task["dataset"]["type"], "bag")
            self.assertFalse(task["calibration"]["shuffle"])
            self.assertEqual(task["execution"], STANDARD_EXECUTION)

    def test_convert_imu_job_preserves_model(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "imu-task.yaml"
            code = main([
                "convert", "job", "--type", "imu-camera",
                "--bag", "/data/input",
                "--target", "/data/target.yaml",
                "--cams", "/data/camchain.yaml",
                "--imu", "/data/imu.yaml",
                "--imu-models", "scale-misalignment",
                "--output", str(output),
            ], prefix=ROOT)
            self.assertEqual(code, 0)
            task = load_task(output, "camera_imu_calibration")
            self.assertEqual(task["imus"][0]["model"], "scale-misalignment")
            self.assertEqual(task["execution"], STANDARD_EXECUTION)

    def test_execution_cli_precedence_and_defaults(self):
        effective = resolve_execution({
            "parallelism": 3,
            "detector_processes": 5,
        }, {
            "parallelism": 6,
            "detector_processes": None,
            "optimizer_threads": 7,
        })
        self.assertEqual(effective["detector_processes"], 6)
        self.assertEqual(effective["optimizer_threads"], 7)
        self.assertEqual(effective["detector_inflight_per_worker"], 2)

    def test_benchmark_command_contract(self):
        arguments = build_parser().parse_args([
            "benchmark", "run", "--config", "task.yaml",
            "--archive-dir", "archive", "--name", "candidate",
            "--repeat", "2", "--detector-processes", "4",
        ])
        self.assertEqual(arguments.benchmark_command, "run")
        self.assertEqual(arguments.repeat, 2)
        self.assertEqual(arguments.detector_processes, 4)

    def test_unknown_task_field_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.yaml"
            path.write_text(
                "schema_version: 1\njob: camera_calibration\n"
                "dataset: {type: bag, path: data.bag}\n"
                "target: {path: target.yaml}\n"
                "unexpected: true\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(TaskError, "unknown fields"):
                load_task(path)

    def test_dataset_type_is_explicit_and_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.yaml"
            path.write_text(
                "schema_version: 1\njob: camera_calibration\n"
                "dataset: {type: directory, path: dataset}\n"
                "target: {path: target.yaml}\n"
                "cameras: [{topic: /cam0, model: pinhole-radtan5}]\n",
                encoding="utf-8",
            )
            task = load_task(path)
            self.assertEqual(task["dataset"]["type"], "directory")
            path.write_text(
                "schema_version: 1\njob: camera_calibration\n"
                "dataset: {type: zip, path: dataset}\n"
                "target: {path: target.yaml}\n"
                "cameras: [{topic: /cam0, model: pinhole-radtan5}]\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(TaskError, "dataset.type"):
                load_task(path)

    def test_dataset_type_is_required(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.yaml"
            path.write_text(
                "schema_version: 1\njob: camera_calibration\n"
                "dataset: {path: dataset}\n"
                "target: {path: target.yaml}\n"
                "cameras: [{topic: /cam0, model: pinhole-radtan5}]\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(TaskError, "dataset.type"):
                load_task(path)

    def test_job_specific_fields_are_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.yaml"
            path.write_text(
                "schema_version: 1\njob: camera_calibration\n"
                "dataset: {type: bag, path: data.bag}\n"
                "target: {path: target.yaml}\n"
                "cameras: [{topic: /cam0, model: pinhole-radtan5}]\n"
                "imus: [{path: imu.yaml}]\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(TaskError, "unknown fields.*imus"):
                load_task(path)

    def test_force_does_not_remove_unmanaged_files(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "output"
            output.mkdir()
            protected = output / "user.txt"
            protected.write_text("keep", encoding="utf-8")
            with self.assertRaisesRegex(TaskError, "unmanaged"):
                prepare_output_directory(output, force=True)
            self.assertEqual(protected.read_text(encoding="utf-8"), "keep")


if __name__ == "__main__":
    unittest.main()
