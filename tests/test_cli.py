import tempfile
import unittest
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_no_ros.cli import build_parser, main
from kalibr_no_ros.task import (
    STANDARD_EXECUTION,
    TaskError,
    dump_yaml,
    load_yaml,
    load_task,
    prepare_output_directory,
    resolve_execution,
)


class TaskCliTest(unittest.TestCase):
    def test_result_yaml_renders_matrix_rows_inline(self):
        document = {
            "T_cam_imu": [
                [-1.25, -0.5, 0.25, 2.5],
                [0.5, 0.25, -2.5, 1.25],
                [0.0, 0.0, 0.0, 1.0],
            ],
            "cam_overlaps": [1],
            "distortion_coeffs": [
                -0.41650678563187626,
                0.21732347443595471,
                0.0007757569830541829,
                0.00026089763511342616,
                -0.06624241154672733,
            ],
            "intrinsics": [1.25, 2.5, 3.75, 5.0],
        }
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "calibration.yaml"
            dump_yaml(document, output)
            text = output.read_text(encoding="utf-8")

            self.assertIn(
                "T_cam_imu:\n"
                "  - [-1.25, -0.5, 0.25, 2.5]\n"
                "  - [0.5, 0.25, -2.5, 1.25]\n"
                "  - [0.0, 0.0, 0.0, 1.0]\n",
                text,
            )
            self.assertIn("cam_overlaps: [1]\n", text)
            self.assertIn(
                "intrinsics: [1.25, 2.5, 3.75, 5.0]\n", text)
            distortion_line = next(
                line for line in text.splitlines()
                if line.startswith("distortion_coeffs:"))
            self.assertTrue(distortion_line.endswith("]"))
            self.assertEqual(distortion_line.count(","), 4)
            self.assertEqual(load_yaml(output), document)

    def test_convert_camera_help_is_forwarded(self):
        arguments = build_parser().parse_args(["convert", "camera", "--help"])
        self.assertTrue(arguments.converter_help)
        self.assertEqual(arguments.arguments, [])

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
            self.assertEqual(task["schema_version"], "1.0.0")
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

    def test_unknown_task_field_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "task.yaml"
            path.write_text(
                "schema_version: 1.0.0\njob: camera_calibration\n"
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
                "schema_version: 1.0.0\njob: camera_calibration\n"
                "dataset: {type: directory, path: dataset}\n"
                "target: {path: target.yaml}\n"
                "cameras: [{topic: /cam0, model: pinhole-radtan5}]\n",
                encoding="utf-8",
            )
            task = load_task(path)
            self.assertEqual(task["dataset"]["type"], "directory")
            path.write_text(
                "schema_version: 1.0.0\njob: camera_calibration\n"
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
                "schema_version: 1.0.0\njob: camera_calibration\n"
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
                "schema_version: 1.0.0\njob: camera_calibration\n"
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
