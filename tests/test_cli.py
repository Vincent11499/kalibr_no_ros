import tempfile
import unittest
from pathlib import Path
import sys
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_no_ros.cli import build_parser, main
from kalibr_no_ros.task.execution import STANDARD_EXECUTION
from kalibr_no_ros.task.document import TaskError
from kalibr_no_ros.task.document import dump_yaml
from kalibr_no_ros.task.document import load_yaml
from kalibr_no_ros.task.validation import load_task
from kalibr_no_ros.task.runner import prepare_output_directory
from kalibr_no_ros.task.execution import resolve_execution


class TaskCliTest(unittest.TestCase):
    def test_installed_tools_cli_accepts_diagnostic_and_study_commands(self):
        diagnosis = build_parser().parse_args([
            "tools", "rolling-shutter-diagnosis",
            "--config", "/data/diagnosis.yaml",
            "--output-dir", "/data/output",
            "--dry-run",
        ])
        self.assertEqual(diagnosis.group, "tools")
        self.assertEqual(diagnosis.tool, "rolling-shutter-diagnosis")
        self.assertEqual(diagnosis.config, "/data/diagnosis.yaml")
        self.assertTrue(diagnosis.dry_run)

        aging = build_parser().parse_args([
            "tools", "stereo-aging-validation", "run",
            "--manifest", "/data/study_manifest.yaml",
            "--phase", "after",
        ])
        self.assertEqual(aging.group, "tools")
        self.assertEqual(aging.tool, "stereo-aging-validation")
        self.assertEqual(aging.tool_command, "run")
        self.assertEqual(aging.phase, "after")

        distance = build_parser().parse_args([
            "tools", "stereo-tag-distance",
            "--dataset", "/data/stereo_261118_1429",
            "--calibration", "/data/cameras.yaml",
            "--tag0", "10", "--tag1", "11",
        ])
        self.assertEqual(distance.group, "tools")
        self.assertEqual(distance.tool, "stereo-tag-distance")
        self.assertEqual(distance.tag0, 10)
        self.assertEqual(distance.tag1, 11)

    @mock.patch("kalibr_tools.stereo_tag_distance.run_distance", return_value=0)
    def test_stereo_tag_distance_dispatches_through_installed_tool(
            self, run_distance):
        result = main([
            "tools", "stereo-tag-distance",
            "--dataset", "/data/stereo_261118_1429",
            "--calibration", "/data/cameras.yaml",
        ], prefix="/opt/kalibr")
        self.assertEqual(result, 0)
        arguments, prefix = run_distance.call_args.args
        self.assertEqual(arguments.dataset, "/data/stereo_261118_1429")
        self.assertEqual(prefix, Path("/opt/kalibr"))

    @mock.patch(
        "kalibr_tools.rolling_shutter_diagnosis.workflow.run_diagnosis",
        return_value={"status": "dry_run"},
    )
    def test_installed_tool_uses_current_installation_binary(self, run_diagnosis):
        result = main([
            "tools", "rolling-shutter-diagnosis",
            "--config", "/data/diagnosis.yaml",
            "--output-dir", "/data/output",
            "--dry-run",
        ], prefix="/opt/kalibr")
        self.assertEqual(result, 0)
        self.assertEqual(
            Path(run_diagnosis.call_args.kwargs["binary"]),
            Path("/opt/kalibr/bin/kalibr-noros"),
        )

    def test_fixed_camera_validation_cli_accepts_repeated_calibrations(self):
        arguments = build_parser().parse_args([
            "tools", "fixed-stereo-validation",
            "--calibration", "first=/data/first.yaml",
            "--calibration", "second=/data/second.yaml",
            "--dataset", "/data/test",
            "--target", "/data/aprilgrid.yaml",
            "--output-dir", "/data/verification",
            "--window-half-size-px", "7",
            "--max-displacement-px", "1",
            "--visualizations",
            "--max-frames-per-camera", "12",
            "--max-pairs", "8",
            "--undistortion-crop",
        ])
        self.assertEqual(arguments.group, "tools")
        self.assertEqual(arguments.tool, "fixed-stereo-validation")
        self.assertEqual(arguments.calibration, [
            "first=/data/first.yaml", "second=/data/second.yaml"])
        self.assertEqual(arguments.window_half_size_px, 7)
        self.assertEqual(arguments.max_displacement_px, 1.0)
        self.assertTrue(arguments.visualizations)
        self.assertEqual(arguments.max_frames_per_camera, 12)
        self.assertEqual(arguments.max_pairs, 8)
        self.assertTrue(arguments.undistortion_crop)
        defaults = build_parser().parse_args([
            "tools", "fixed-stereo-validation", "--calibration", "/data/camera.yaml",
            "--dataset", "/data/test", "--target", "/data/aprilgrid.yaml",
            "--output-dir", "/data/verification",
        ])
        self.assertEqual(defaults.max_frames_per_camera, 20)
        self.assertEqual(defaults.max_pairs, 20)
        self.assertEqual(defaults.window_half_size_px, 2)
        self.assertEqual(defaults.max_displacement_px, 1.5 ** 0.5)
        self.assertEqual(defaults.synchronization_tolerance_s, 0.0002)
        self.assertEqual(defaults.rectification_balance, 0.0)
        self.assertEqual(defaults.rectification_fov_scale, 1.0)
        self.assertEqual(defaults.detector_opencv_threads, 1)
        self.assertFalse(defaults.visualizations)
        self.assertFalse(defaults.undistortion_crop)
        self.assertFalse(defaults.force)

    @mock.patch("kalibr_tools.fixed_stereo_validation.verify_fixed_cameras")
    def test_fixed_stereo_tool_dispatches_existing_validation_parameters(self, verify):
        result = main([
            "tools", "fixed-stereo-validation",
            "--calibration", "/data/camera.yaml",
            "--dataset", "/data/test",
            "--target", "/data/aprilgrid.yaml",
            "--output-dir", "/data/verification",
        ], prefix="/opt/kalibr")
        self.assertEqual(result, 0)
        self.assertEqual(verify.call_args.args, (
            ["/data/camera.yaml"], "/data/test", "/data/aprilgrid.yaml",
            Path("/data/verification"),
        ))
        self.assertEqual(verify.call_args.kwargs["max_displacement_px"], 1.5 ** 0.5)
        self.assertEqual(verify.call_args.kwargs["synchronization_tolerance_s"], 0.0002)

    def test_previous_verify_command_is_not_a_second_public_entrypoint(self):
        with mock.patch("sys.stderr"), self.assertRaises(SystemExit) as failure:
            build_parser().parse_args(["verify", "cameras"])
        self.assertEqual(failure.exception.code, 2)

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
