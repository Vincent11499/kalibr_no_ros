"""Public rolling-shutter task isolation and camera-ID adaptation."""
from pathlib import Path
import sys
import tempfile
import unittest
import copy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/python"))
from kalibr_no_ros.task import TaskError, dump_yaml, load_yaml, load_task, _imu_arguments
from kalibr_no_ros.rolling_shutter import JOB, validate_shutters
from kalibr_no_ros.cli import build_parser
from kalibr_no_ros.delivery import result_name


class RollingShutterContractTest(unittest.TestCase):
    def test_independent_job_and_defaults(self):
        parsed = build_parser().parse_args([
            "calibrate", "imu-camera-rs", "--config", "task.yaml", "--output-dir", "result"])
        self.assertEqual(parsed.calibration, "imu-camera-rs")
        self.assertEqual(validate_shutters({"left": {"max_abs_line_delay_s": 2e-5}}),
                         {"left": {"line_delay_s": 0.0, "estimate": True, "max_abs_line_delay_s": 2e-5}})
        fixed = validate_shutters({"right": {"estimate": False, "line_delay_s": -8e-6}})
        self.assertEqual(fixed["right"]["line_delay_s"], -8e-6)

    def test_invalid_bounds_ids_and_types(self):
        for value in [None, {}, {"cam0": {}}, {"cam0": {"estimate": 1}},
                      {"cam0": {"estimate": False, "line_delay_s": float('nan')}},
                      {"cam0": {"line_delay_s": 1e-5, "max_abs_line_delay_s": 1e-5}},
                      {"cam0": {"max_abs_line_delay_s": True}},
                      {"cam0": {"estimate": False, "reference_row_px": 1080}}]:
            with self.subTest(value=value), self.assertRaises(TaskError):
                validate_shutters(value)
        with self.assertRaises(TaskError):
            validate_shutters({"cam0": {"estimate": False}}, ["left"])

    def test_saved_observability_reaches_metrics_and_both_report_formats(self):
        from kalibr_no_ros.evaluation import compute_metrics
        from kalibr_no_ros.report_plots import _summary_sections
        summary = {"status": "full_rank", "quality": "weakly_observable",
                   "rank": 10, "columns": 10, "operational_rank": 9}
        metrics = compute_metrics({"calibration_type": "camera_imu", "cameras": [],
                                   "observability": summary}, {"cameras": []})
        self.assertEqual(metrics["observability"], summary)
        sections = {title: rows for title, _, rows in _summary_sections(metrics, {}, {"cameras": []})}
        self.assertIn(["Quality", "weakly_observable"], sections["Local observability"])
        self.assertIn(["Operational rank", "9 / 10"], sections["Local observability"])

    def test_three_public_ids_map_to_native_order_without_changing_original_job(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cameras = []
            for index, name in enumerate(["left", "right", "rear"]):
                camera = {"id": name, "camera_model": "pinhole", "distortion_model": "equidistant",
                          "intrinsics": [400., 400., 320., 240.], "distortion_coeffs": [0., 0., 0., 0.],
                          "resolution": [640, 480], "rostopic": "/" + name}
                if index:
                    camera["T_cn_cnm1"] = [[1.,0.,0.,-.3],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
                cameras.append(camera)
            dump_yaml({"schema_version": "1.0.0", "kind": "calibration_result",
                       "calibration_type": "cameras", "cameras": cameras}, root / "cameras.yaml")
            dump_yaml({"schema_version": "1.0.0", "kind": "imu_configuration", "rostopic": "/imu",
                       "update_rate": 100., "accelerometer_noise_density": .01,
                       "accelerometer_random_walk": .001, "gyroscope_noise_density": .001,
                       "gyroscope_random_walk": .0001}, root / "imu.yaml")
            task = {"schema_version": "1.0.0", "kind": "calibration_task", "job": JOB,
                    "dataset": {"type": "bag", "path": "input.bag"}, "target": {"path": "target.yaml"},
                    "camera_calibration": {"path": "cameras.yaml"}, "imus": [{"id": "imu0", "path": "imu.yaml"}],
                    "rolling_shutter": {name: {"estimate": False, "line_delay_s": (i+1)*1e-6}
                                        for i, name in enumerate(["left", "right", "rear"])}}
            dump_yaml(task, root / "task.yaml")
            resolved = load_task(root / "task.yaml", JOB)
            self.assertEqual(result_name(resolved), JOB + "_left_right_rear_imu0")
            argv = _imu_arguments(resolved, root / "input.bag", root / "target.yaml", root, {})
            shutter = load_yaml(argv[argv.index("--rolling-shutter-config") + 1])
            self.assertEqual(shutter["cameras"]["cam2"]["line_delay_s"], 3e-6)
            old = copy.deepcopy(task);old["job"] = "camera_imu_calibration"
            dump_yaml(old, root / "old.yaml")
            with self.assertRaisesRegex(TaskError, "unknown fields"):
                load_task(root / "old.yaml")
            old.pop("rolling_shutter");dump_yaml(old, root / "old.yaml")
            argv = _imu_arguments(load_task(root / "old.yaml"), root / "input.bag", root / "target.yaml", root, {})
            self.assertNotIn("--rolling-shutter-config", argv)


if __name__ == "__main__":unittest.main()
