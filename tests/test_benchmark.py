import json
import tempfile
import unittest
from pathlib import Path
import sys
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_no_ros import benchmark


class BenchmarkTest(unittest.TestCase):
    def _output(self, root, value):
        root.mkdir(parents=True)
        (root / "calibration.yaml").write_text(
            yaml.safe_dump({"value": value}), encoding="utf-8")
        (root / "results.txt").write_text("result\n", encoding="utf-8")

    def test_effective_task_records_deterministic_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "task.yaml"
            config.write_text(
                "schema_version: 1.0.0\n"
                "job: camera_calibration\n"
                "dataset: {type: bag, path: input.bag}\n"
                "target: {path: target.yaml}\n"
                "cameras: [{topic: /cam0, model: pinhole-radtan5}]\n"
                "initialization: {path: seed.yaml, strategy: refine}\n",
                encoding="utf-8",
            )
            effective = benchmark._absolute_effective_task(config, {})
            self.assertFalse(effective["calibration"]["shuffle"])
            self.assertEqual(effective["execution"]["detector_processes"], 4)
            self.assertEqual(effective["execution"]["optimizer_threads"], 4)
            self.assertTrue(Path(effective["dataset"]["path"]).is_absolute())
            self.assertEqual(
                Path(effective["initialization"]["path"]),
                (root / "seed.yaml").resolve(),
            )

    def test_compare_validates_frozen_files_without_rerunning_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            project = root / "project"
            reference = root / "reference"
            candidate = root / "candidate"
            trial = candidate / "trial-001"
            self._output(project, 1.0)
            self._output(reference, 1.0)
            self._output(trial / "outputs", 1.0 + 1e-10)
            task = root / "task.yaml"
            task.write_text("schema_version: 1.0.0\n", encoding="utf-8")
            timing = root / "timing.json"
            timing_document = {
                "status": "ok",
                "summary": {"category_totals": [
                    {"name": "optimization", "wall_seconds": 50.0},
                ]},
            }
            timing.write_text(json.dumps(timing_document), encoding="utf-8")
            (trial / "timing.json").write_text(
                json.dumps(timing_document), encoding="utf-8")
            (candidate / "effective-task.yaml").write_text(yaml.safe_dump({
                "job": "camera_calibration",
                "dataset": {"path": str(root / "input.bag")},
                "cameras": [{"model": "pinhole-radtan5"}],
                "calibration": {"shuffle": False},
                "execution": {
                    "detector_processes": 4,
                    "optimizer_threads": 4,
                },
            }), encoding="utf-8")
            (candidate / "summary.json").write_text(json.dumps({
                "trials": ["trial-001"],
                "wall_seconds": {"count": 1, "median": 102.0},
                "aggregate_rss_peak_bytes": {"median": 1000.0},
                "aggregate_pss_peak_bytes": {"median": 900.0},
            }), encoding="utf-8")
            hashes = {
                "task": benchmark._sha256(task),
                "project_calibration": benchmark._sha256(
                    project / "calibration.yaml"),
                "project_results": benchmark._sha256(project / "results.txt"),
                "project_timing": benchmark._sha256(timing),
                "reference_calibration": benchmark._sha256(
                    reference / "calibration.yaml"),
            }
            registry = root / "registry.yaml"
            registry.write_text(yaml.safe_dump({
                "schema_version": 1,
                "baselines": {"sample": {
                    "project_output": str(project),
                    "reference_output": str(reference),
                    "project_timing": str(timing),
                    "task": str(task),
                    "wall_seconds": 100.0,
                    "contract": {
                        "job": "camera_calibration",
                        "dataset_path": str((root / "input.bag").resolve()),
                        "camera_models": ["pinhole-radtan5"],
                        "detector_processes": 4,
                        "optimizer_threads": 4,
                    },
                    "frozen_sha256": hashes,
                }},
            }), encoding="utf-8")
            with mock.patch.object(
                    benchmark.subprocess, "Popen",
                    side_effect=AssertionError("baseline must not run")):
                json_path, markdown = benchmark.compare_benchmark(
                    registry, "sample", candidate)
            result = json.loads(json_path.read_text())
            self.assertTrue(result["project_numeric"]["compatible"])
            self.assertTrue(result["reference_numeric"]["compatible"])
            self.assertTrue(result["performance"][
                "single_run_below_five_percent_is_inconclusive"])
            self.assertTrue(markdown.is_file())

    def test_changed_frozen_artifact_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "output"
            self._output(output, 1.0)
            task = root / "task.yaml"
            task.write_text("task\n", encoding="utf-8")
            timing = root / "timing.json"
            timing.write_text("{}\n", encoding="utf-8")
            baseline = {
                "project_output": str(output),
                "reference_output": str(output),
                "project_timing": str(timing),
                "task": str(task),
                "frozen_sha256": {
                    "task": "invalid",
                    "project_calibration": benchmark._sha256(output / "calibration.yaml"),
                    "project_results": benchmark._sha256(output / "results.txt"),
                    "project_timing": benchmark._sha256(timing),
                    "reference_calibration": benchmark._sha256(output / "calibration.yaml"),
                },
            }
            with self.assertRaisesRegex(Exception, "changed"):
                benchmark._validate_frozen_baseline(baseline)


if __name__ == "__main__":
    unittest.main()
