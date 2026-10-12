import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import yaml

from tools.rolling_shutter_diagnosis import analysis, reporting
from tools.rolling_shutter_diagnosis.workflow import (
    _execute_task,
    _formal_evidence,
    _incomplete_formal_evidence,
    build_formal_tasks,
    load_config,
)


def _write_yaml(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


class RollingShutterDiagnosisToolTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def _config(self, motion="continuous"):
        task = self.root / "task.yaml"
        _write_yaml(task, {
            "schema_version": "1.0.0",
            "kind": "calibration_task",
            "job": "camera_calibration",
            "dataset": {"type": "directory", "path": "dataset"},
            "target": {"path": "aprilgrid.yaml"},
            "cameras": [{"id": "cam0", "model": "pinhole-equi"}],
        })
        path = self.root / "diagnosis.yaml"
        _write_yaml(path, {
            "schema_version": "1.0.0",
            "kind": "rolling_shutter_diagnosis",
            "source_task": {"path": "task.yaml"},
            "capture": {"motion": motion},
            "rolling_shutter": {"cam0": {"max_abs_line_delay_s": 2e-5}},
        })
        return path

    def test_config_resolves_source_paths_and_applies_screen_defaults(self):
        config = load_config(self._config())
        self.assertEqual(config["capture"]["motion"], "continuous")
        self.assertEqual(config["bootstrap_samples"], 1000)
        self.assertEqual(config["bootstrap_seed"], 0)
        self.assertEqual(config["source_task"]["dataset"]["path"], str((self.root / "dataset").resolve()))

    def test_unknown_motion_and_camera_ids_are_rejected(self):
        path = self._config("fast")
        with self.assertRaisesRegex(ValueError, "capture.motion"):
            load_config(path)
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
        value["capture"]["motion"] = "continuous"
        value["rolling_shutter"] = {"other": {"max_abs_line_delay_s": 2e-5}}
        _write_yaml(path, value)
        with self.assertRaisesRegex(ValueError, "exactly match"):
            load_config(path)

    @staticmethod
    def _synthetic_frames(count=20, delay=8e-6):
        rng = np.random.default_rng(9)
        frames = []
        for index in range(count):
            row_basis = rng.normal(0.0, 30000.0, (24, 2))
            column_basis = rng.normal(0.0, 30000.0, (24, 2))
            residual = row_basis * delay + rng.normal(0.0, 0.003, (24, 2))
            frames.append({
                "time_s": index * 0.05,
                "timestamp_ns": index * 50_000_000,
                "source_index": index,
                "row_basis": row_basis,
                "column_basis": column_basis,
                "residual": residual,
                "corner_count": len(residual),
            })
        return frames

    def test_blocked_screen_detects_row_specific_delay(self):
        frames = self._synthetic_frames()
        with mock.patch.object(analysis, "_add_velocity_basis", side_effect=lambda values, _camera, exclude_edges: values[1:-1]):
            result = analysis._screen_camera(
                frames,
                {"resolution": [3840, 2160]},
                2e-5,
                200,
                np.random.default_rng(0),
            )
        self.assertTrue(result["supportive"])
        self.assertAlmostEqual(result["candidate_line_delay_s"], 8e-6, delta=3e-8)
        self.assertTrue(all(result["gates"].values()))

    def test_stop_and_go_never_starts_formal_controls(self):
        camera = {
            "id": "cam0",
            "camera_model": "pinhole",
            "distortion_model": "equidistant",
            "intrinsics": [100.0, 100.0, 50.0, 50.0],
            "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
            "resolution": [100, 100],
        }
        with mock.patch.object(analysis, "fit_archived_frames", return_value=({"cam0": camera}, {"cam0": self._synthetic_frames()}, [])), mock.patch.object(
            analysis, "_add_velocity_basis", side_effect=lambda values, _camera, exclude_edges: values[1:-1]
        ):
            result, _ = analysis.screen_observations(
                Path("result.yaml"), Path("observations"), {"cam0": 2e-5}, "stop_and_go", bootstrap_samples=200
            )
        self.assertEqual(result["classification"], "inconclusive")
        self.assertFalse(result["run_formal_controls"])

    def test_visual_formal_plan_has_matched_zero_knot_controls(self):
        config = load_config(self._config())
        result = self.root / "ordinary.yaml"
        _write_yaml(result, {
            "kind": "calibration_result",
            "cameras": [{
                "id": "cam0",
                "intrinsics": [100.0, 100.0, 50.0, 50.0],
                "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                "resolution": [100, 100],
            }],
        })
        output = self.root / "output"
        (output / "configs").mkdir(parents=True)
        screen = {"camera_results": {"cam0": {"candidate_line_delay_s": 8e-6, "median_frame_frequency_hz": 20.0}}}
        tasks = build_formal_tasks(config, output, result, screen)
        names = {task["name"] for task in tasks}
        self.assertEqual(len(tasks), 10)
        self.assertTrue({"knot_half_free", "knot_half_zero", "knot_double_free", "knot_double_zero"} <= names)
        half = yaml.safe_load((output / "configs" / "knot_half_free.yaml").read_text(encoding="utf-8"))
        zero = yaml.safe_load((output / "configs" / "knot_half_zero.yaml").read_text(encoding="utf-8"))
        self.assertEqual(half["calibration"]["knots_per_second"], zero["calibration"]["knots_per_second"])
        self.assertTrue(half["rolling_shutter"]["cam0"]["estimate"])
        self.assertFalse(zero["rolling_shutter"]["cam0"]["estimate"])

    def test_formal_controls_can_keep_intrinsics_active_and_bound_iterations(self):
        path = self._config()
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        document["formal_controls"] = {
            "freeze_intrinsics": False,
            "max_iterations": 200,
        }
        _write_yaml(path, document)
        config = load_config(path)
        result = self.root / "ordinary.yaml"
        _write_yaml(result, {
            "kind": "calibration_result",
            "cameras": [{
                "id": "cam0",
                "intrinsics": [100.0, 100.0, 50.0, 50.0],
                "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
                "resolution": [100, 100],
            }],
        })
        output = self.root / "formal-output"
        (output / "configs").mkdir(parents=True)
        screen = {"camera_results": {"cam0": {
            "candidate_line_delay_s": 8e-6,
            "median_frame_frequency_hz": 20.0,
        }}}
        tasks = build_formal_tasks(config, output, result, screen)
        task = yaml.safe_load(tasks[0]["path"].read_text(encoding="utf-8"))
        self.assertFalse(task["calibration"]["freeze_intrinsics"])
        self.assertEqual(task["calibration"]["max_iterations"], 200)

    def test_report_rows_include_every_formal_run_rms_and_knot_control(self):
        result = {
            "formal_comparison": {
                "runs": {
                    "free_seed0": {
                        "cameras": {"cam0": {"line_delay_s": 8e-6, "rms_px": 0.2}}
                    },
                    "fixed_zero": {
                        "cameras": {"cam0": {"line_delay_s": 0.0, "rms_px": 1.2}}
                    },
                },
                "common_corner_comparisons": {
                    "free_seed0_vs_fixed_zero": {
                        "cam0": {
                            "relative_reduction": 0.8,
                            "bootstrap_95_percent": {"lower_px": 0.5},
                        }
                    }
                },
            }
        }
        run_rows = reporting._formal_run_rows(result)
        self.assertEqual([row["run"] for row in run_rows], ["free_seed0", "fixed_zero"])
        self.assertEqual([row["rms_px"] for row in run_rows], [0.2, 1.2])
        self.assertEqual(
            [row["line_delay_mode"] for row in run_rows], ["estimated", "fixed"]
        )
        knot_rows = reporting._knot_control_rows(result)
        base = next(row for row in knot_rows if row["knot_frequency"] == "1×")
        self.assertEqual(base["free_rms_px"], 0.2)
        self.assertEqual(base["fixed_zero_rms_px"], 1.2)
        self.assertTrue(base["passed"])

    @mock.patch("tools.rolling_shutter_diagnosis.workflow.load_run_summary")
    @mock.patch("tools.rolling_shutter_diagnosis.workflow._completed")
    def test_incomplete_formal_evidence_retains_completed_run_metrics(
        self, completed, load_summary
    ):
        tasks = [{"name": name} for name in ("free_seed0", "fixed_zero", "knot_half_free")]
        completed.side_effect = lambda _path, name: name != "knot_half_free"
        load_summary.side_effect = lambda _path, name: {
            "completed": True,
            "cameras": {"cam0": {"rms_px": 0.2 if name == "free_seed0" else 1.2}},
        }

        result = _incomplete_formal_evidence(self.root, tasks)

        self.assertEqual(result["classification"], "inconclusive")
        self.assertEqual(result["completed_run_count"], 2)
        self.assertEqual(result["required_run_count"], 3)
        self.assertEqual(result["missing_runs"], ["knot_half_free"])
        self.assertEqual(list(result["runs"]), ["free_seed0", "fixed_zero"])
        self.assertEqual(load_summary.call_count, 2)

    def test_ordinary_run_rows_report_rms_without_line_delay(self):
        result_dir = self.root / "output" / "runs" / "ordinary_source"
        result_dir.mkdir(parents=True)
        (result_dir / "ordinary_source.yaml").write_text(
            yaml.safe_dump({
                "cameras": [
                    {"id": "cam0", "rms": 0.3},
                    {"id": "cam1", "rms": 0.4, "alignment": 0.2},
                ]
            }),
            encoding="utf-8",
        )
        rows = reporting._ordinary_run_rows(self.root / "output")
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["line_delay_mode"], "not_applicable")
        self.assertIsNone(rows[0]["line_delay_us"])
        self.assertEqual(rows[1]["alignment_rms_px"], 0.2)

    def test_resume_replaces_only_an_incomplete_managed_run(self):
        run_dir = self.root / "output" / "runs" / "failed"
        (run_dir / "failed").mkdir(parents=True)
        commands, completed = _execute_task(
            Path("/bin/true"),
            self.root / "task.yaml",
            run_dir,
            "failed",
            "cameras-rs",
            self.root / "output",
            dry_run=True,
            force=False,
        )
        self.assertTrue(completed)
        self.assertIn("--force", commands[1])

    @mock.patch("tools.rolling_shutter_diagnosis.workflow.compare_archived_residuals")
    @mock.patch("tools.rolling_shutter_diagnosis.workflow.load_run_summary")
    def test_formal_classification_requires_all_registered_controls(self, load_summary, compare):
        names = [
            "free_seed0", "fixed_zero", "fixed_candidate", "fixed_opposite",
            "free_seed_candidate", "free_seed_opposite", "knot_half_free",
            "knot_half_zero", "knot_double_free", "knot_double_zero",
        ]
        tasks = [{"name": name} for name in names]
        for name in names:
            (self.root / "runs" / name / name / "observations").mkdir(parents=True)

        delays = {
            "free_seed0": 8.0e-6,
            "free_seed_candidate": 8.1e-6,
            "free_seed_opposite": 7.9e-6,
            "knot_half_free": 7.8e-6,
            "knot_double_free": 8.2e-6,
        }

        def summary(_run_dir, name):
            return {
                "completed": True,
                "cameras": {"cam0": {"line_delay_s": delays.get(name, 0.0), "rms_px": 0.4}},
                "observability": {"status": "full_rank", "maximum_line_delay_contribution_in_weak_mode": 0.1},
                "optimizer": {"linearSolverFailure": False, "iterations": 20},
            }

        load_summary.side_effect = summary
        compare.return_value = {
            "cam0": {"relative_reduction": 0.2, "bootstrap_95_percent": {"lower_px": 0.01}}
        }
        config = {
            "source_job": "camera_calibration",
            "camera_ids": ["cam0"],
            "bounds": {"cam0": 2e-5},
            "bootstrap_samples": 200,
            "bootstrap_seed": 0,
        }
        result = _formal_evidence(self.root, config, tasks)
        self.assertEqual(result["classification"], "important")
        self.assertTrue(result["camera_gates"]["cam0"]["passed"])
        self.assertEqual(load_summary.call_count, 10)


if __name__ == "__main__":
    unittest.main()
