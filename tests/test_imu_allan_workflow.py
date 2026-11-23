"""Exercise CSV selection, the public command, and failure-safe Allan delivery."""

import argparse
import contextlib
import csv
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from kalibr_no_ros.cli import build_parser, main
from kalibr_no_ros.task.document import load_yaml
from kalibr_no_ros.task.validation import load_imu
from kalibr_tools.imu_allan import input_format, workflow


class ImuAllanWorkflowTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.dataset = self.root / "imu_261111_1200"
        self.csv_path = self.dataset / "imu" / "imu0.csv"
        self.csv_path.parent.mkdir(parents=True)
        self.output = self.root / "results"

    def write_csv(self, count=4, temperature=None):
        header = input_format.COLUMNS + ([] if temperature is None else [temperature])
        random = np.random.default_rng(2026)
        with self.csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            for index, sample in enumerate(random.normal(size=(count, 6))):
                values = (sample * [.001, .001, .001, .01, .01, .01]).tolist()
                values[5] += 9.81
                writer.writerow([1_700_000_000_000_000_000 + index * 100_000_000]
                                + values + ([] if temperature is None else [25.0]))

    def bounds(self, **values):
        options = dict(start_s=None, start_h=None, end_s=None, end_h=None)
        options.update(values)
        return input_format.time_bounds(argparse.Namespace(**options))

    def test_time_bounds_preserve_nanoseconds_and_reject_invalid_ranges(self):
        self.assertEqual(self.bounds(), (0, None))
        self.assertEqual(self.bounds(start_s="0.000000001", end_h="1"), (1, 3_600_000_000_000))
        for options in ({"start_s": "invalid"}, {"start_s": "NaN"}, {"end_h": "Infinity"},
                        {"start_s": "-1"}, {"start_s": "0.0000000001"},
                        {"start_h": "1", "end_s": "3600"}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.bounds(**options)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            build_parser().parse_args([
                "tools", "imu-allan", "--dataset", "data", "--output-dir", "out",
                "--start-s", "1", "--start-h", "1",
            ])

    def test_csv_formats_selection_and_axis_unit_conversion(self):
        for temperature in (None, "temperature", "temperature_c"):
            with self.subTest(temperature=temperature):
                self.write_csv(temperature=temperature)
                source = input_format.validate_input(self.dataset, self.csv_path, 100_000_000, 300_000_000)
                self.assertEqual(source["messages"], 2)
                self.assertEqual(source["first_timestamp_ns"], 1_700_000_000_100_000_000)
                self.assertEqual(source["selection"]["excluded_before"], 1)
                self.assertEqual(source["selection"]["excluded_after"], 1)
                self.assertEqual(source["selection"]["source_messages"], 4)
                array = self.root / "samples.npy"
                input_format.materialize_selected_samples(self.csv_path, array, 2, 100_000_000, 300_000_000)
                values = list(input_format.rows(self.csv_path))[1][1]
                np.testing.assert_allclose(np.load(array)[0],
                                           list(values[3:6]) + [value * 180 / np.pi for value in values[:3]])

    def test_complete_source_is_validated_outside_selected_range(self):
        self.write_csv()
        with self.csv_path.open("a", encoding="utf-8") as stream:
            stream.write("1700000000400000000,0,0,0,0,0,nan\n")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            input_format.validate_input(self.dataset, self.csv_path, 0, 200_000_000)
        self.write_csv()
        with self.csv_path.open("a", encoding="utf-8") as stream:
            stream.write("1700000000300000000,0,0,0,0,0,0\n")
        with self.assertRaisesRegex(ValueError, "non-increasing"):
            input_format.validate_input(self.dataset, self.csv_path)

    def test_existing_outputs_and_symlinks_are_preserved(self):
        self.write_csv()
        for suffix in ("", ".inprogress", ".failed"):
            path = self.output.with_name(self.output.name + suffix)
            path.mkdir()
            marker = path / "keep.txt"
            marker.write_text("keep", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                workflow.run_analysis(self.dataset, self.output)
            self.assertEqual(marker.read_text(encoding="utf-8"), "keep")
            marker.unlink()
            path.rmdir()
        self.output.symlink_to(self.root / "missing", target_is_directory=True)
        with self.assertRaises(FileExistsError):
            workflow.run_analysis(self.dataset, self.output)
        self.assertTrue(self.output.is_symlink())

    def test_invalid_inputs_do_not_create_output(self):
        self.write_csv()
        for output, options, error in ((self.dataset / "result", {}, "source dataset"),
                                        (self.output, {}, "complete bins"),
                                        (self.output, {"topic": "bad topic"}, "topic")):
            with self.subTest(options=options, output=output), self.assertRaisesRegex(ValueError, error):
                workflow.run_analysis(self.dataset, output, **options)
            self.assertFalse(output.exists())
        self.assertFalse(self.output.with_name(self.output.name + ".inprogress").exists())

    def test_failure_retains_diagnostics_and_source(self):
        self.write_csv(count=20001)
        original = self.csv_path.read_bytes()
        with mock.patch.object(workflow, "compute_curves", side_effect=RuntimeError("calculation failed")):
            with self.assertRaisesRegex(RuntimeError, "calculation failed"):
                workflow.run_analysis(self.dataset, self.output)
        failed = self.output.with_name(self.output.name + ".failed")
        run = json.loads((failed / "work" / "run.json").read_text(encoding="utf-8"))
        self.assertEqual(run["status"], "fail")
        self.assertIn("calculation failed", run["error"])
        self.assertTrue((failed / "work" / "selected_samples.npy").is_file())
        self.assertFalse(self.output.exists())
        self.assertEqual(self.csv_path.read_bytes(), original)

    def test_public_cli_generates_readable_noise_and_both_reports(self):
        self.write_csv(count=20001, temperature="temperature")
        original = hashlib.sha256(self.csv_path.read_bytes()).digest()
        with contextlib.redirect_stdout(io.StringIO()):
            status = main([
                "tools", "imu-allan", "--dataset", str(self.csv_path),
                "--output-dir", str(self.output), "--topic", "/sensor/imu",
                "--start-s", "0", "--end-s", "2000.1",
            ], prefix=self.root)
        self.assertEqual(status, 0)
        noise = load_yaml(self.output / "imu_noise.yaml")
        self.assertEqual(noise["schema_version"], "1.0.0")
        self.assertEqual(noise["kind"], "imu_configuration")
        block = {"path": str(self.output / "imu_noise.yaml")}
        self.assertEqual(load_imu({"_config_dir": str(self.root)}, block), noise)
        self.assertEqual(noise["rostopic"], "/sensor/imu")
        self.assertEqual(noise["update_rate"], 10.0)
        self.assertTrue(all(noise[name] > 0 for name in workflow.NOISE_FIELDS))
        self.assertTrue((self.output / "imu_allan.report.pdf").read_bytes().startswith(b"%PDF-"))
        self.assertIn("data:image/png;base64,", (self.output / "imu_allan.report.html").read_text(encoding="utf-8"))
        self.assertEqual(np.loadtxt(self.output / "work" / "allan_variance.csv").shape, (9999, 7))
        self.assertEqual(json.loads((self.output / "work" / "run.json").read_text())["status"], "pass")
        self.assertFalse((self.output / "work" / "selected_samples.npy").exists())
        self.assertFalse(list(self.output.rglob("*.png")))
        self.assertEqual(hashlib.sha256(self.csv_path.read_bytes()).digest(), original)


if __name__ == "__main__":
    unittest.main()
