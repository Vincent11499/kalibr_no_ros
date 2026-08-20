import argparse
import json
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NATIVE_PYTHON = ROOT / "extensions" / "native_optimizer" / "python"
sys.path.insert(0, str(NATIVE_PYTHON))

import kalibr_native_optimizer as native_runtime
from kalibr_native_optimizer import runtime
from kalibr_native_optimizer.cli_overlay import transform_cli_source
from kalibr_native_optimizer.source_overlay import transform_source


class FakeOptions:
    def __init__(self, threads=4):
        self.nThreads = threads


class FakeOptimizer:
    instances = []

    def __init__(self, options):
        self.options = options
        self.optimizations = 0
        self.__class__.instances.append(self)

    def optimize(self):
        self.optimizations += 1
        return "optimized"


class FakeIncrementalEstimator:
    instances = []

    def __init__(self, unused_group):
        self.options = FakeOptions(7)
        self.batches = 0
        self.__class__.instances.append(self)

    def getOptimizerOptions(self):
        return self.options

    def addBatch(self, value):
        self.batches += 1
        return value


class NativeOptimizerRuntimeTest(unittest.TestCase):
    def setUp(self):
        runtime._reset_for_tests()
        FakeOptimizer.instances = []
        FakeIncrementalEstimator.instances = []
        self.previous_backend = sys.modules.get("aslam_backend")
        self.previous_incremental = sys.modules.get("incremental_calibration")
        self.backend = types.ModuleType("aslam_backend")
        self.backend.Optimizer2 = FakeOptimizer
        self.incremental = types.ModuleType("incremental_calibration")
        self.incremental.IncrementalEstimator = FakeIncrementalEstimator
        sys.modules["aslam_backend"] = self.backend
        sys.modules["incremental_calibration"] = self.incremental

    def tearDown(self):
        runtime._reset_for_tests()
        if self.previous_backend is None:
            sys.modules.pop("aslam_backend", None)
        else:
            sys.modules["aslam_backend"] = self.previous_backend
        if self.previous_incremental is None:
            sys.modules.pop("incremental_calibration", None)
        else:
            sys.modules["incremental_calibration"] = self.previous_incremental

    @staticmethod
    def arguments(**overrides):
        values = {
            "parallelism": None,
            "detector_processes": None,
            "optimizer_threads": None,
            "timing_json": None,
        }
        values.update(overrides)
        return argparse.Namespace(**values)

    def test_parser_defaults_do_not_override_upstream_values(self):
        # Exercise the production-default interface even when this test is
        # launched from an explicitly profiling-enabled build tree.
        runtime._set_profiling_for_tests(False)
        parser = argparse.ArgumentParser()
        native_runtime.add_parallelism_arguments(parser)
        parsed = parser.parse_args([])
        self.assertTrue({
            "--parallelism",
            "--detector-processes",
            "--optimizer-threads",
        }.issubset(parser._option_string_actions))
        self.assertNotIn("--timing-json", parser._option_string_actions)
        self.assertFalse(native_runtime.profiling_enabled())
        effective = native_runtime.configure(parsed, command="test")

        self.assertEqual(effective, {
            "parallelism": None,
            "detector_processes": None,
            "optimizer_threads": None,
        })
        self.assertIs(self.backend.Optimizer2, FakeOptimizer)
        self.assertIs(
            self.incremental.IncrementalEstimator,
            FakeIncrementalEstimator,
        )
        self.assertEqual(native_runtime.optimizer_threads_or(11), 11)
        self.assertFalse(native_runtime.timing_enabled())

    def test_common_parallelism_and_component_precedence(self):
        effective = native_runtime.configure(self.arguments(
            parallelism=6,
            detector_processes=2,
        ))
        self.assertEqual(effective["detector_processes"], 2)
        self.assertEqual(effective["optimizer_threads"], 6)

    def test_explicit_threads_cover_direct_and_incremental_optimizers(self):
        native_runtime.configure(self.arguments(optimizer_threads=3))
        self.assertIs(self.backend.Optimizer2, FakeOptimizer)
        self.assertIs(
            self.incremental.IncrementalEstimator, FakeIncrementalEstimator
        )

        direct_options = FakeOptions(4)
        native_runtime.apply_optimizer_threads(direct_options)
        optimizer = self.backend.Optimizer2(direct_options)
        self.assertEqual(direct_options.nThreads, 3)
        self.assertIs(type(optimizer), FakeOptimizer)
        self.assertEqual(native_runtime.run_optimizer(optimizer), "optimized")

        estimator = self.incremental.IncrementalEstimator(0)
        self.assertIs(type(estimator), FakeIncrementalEstimator)
        # A phase may call addBatch immediately (recover-covariance does this)
        # after applying the override to its real options object.
        native_runtime.apply_optimizer_threads(estimator.getOptimizerOptions())
        self.assertEqual(estimator.getOptimizerOptions().nThreads, 3)
        self.assertEqual(
            native_runtime.run_incremental_batch(estimator, "direct"), "direct"
        )
        incremental_options = estimator.getOptimizerOptions()
        # Simulate the upstream CLI assigning its normal stage default later.
        incremental_options.nThreads = 12
        native_runtime.apply_optimizer_threads(incremental_options)
        self.assertEqual(incremental_options.nThreads, 3)
        self.assertEqual(
            native_runtime.run_incremental_batch(estimator, "accepted"),
            "accepted",
        )

    def test_timing_json_has_extraction_and_optimizer_phase_schema(self):
        runtime._set_profiling_for_tests(True)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "timing.json"
            native_runtime.configure(
                self.arguments(timing_json=str(output)),
                command="kalibr_calibrate_cameras",
            )
            self.assertTrue(native_runtime.timing_enabled())
            self.assertIs(self.backend.Optimizer2, FakeOptimizer)
            self.assertIs(
                self.incremental.IncrementalEstimator, FakeIncrementalEstimator
            )

            options = FakeOptions(4)
            optimizer = self.backend.Optimizer2(options)
            self.assertIs(type(optimizer), FakeOptimizer)
            self.assertEqual(native_runtime.run_optimizer(optimizer), "optimized")
            native_runtime.record_corner_extraction(
                topic="/cam0",
                images_reported=10,
                images_submitted=10,
                observations_succeeded=8,
                bag_read_wall_seconds=0.1,
                bag_read_cpu_seconds=0.08,
                deserialize_wall_seconds=0.12,
                deserialize_cpu_seconds=0.1,
                decode_wall_seconds=0.3,
                decode_cpu_seconds=0.2,
                detect_wall_seconds=1.2,
                detect_cpu_seconds=1.0,
                total_wall_seconds=0.9,
                total_cpu_seconds=1.3,
                rss_bytes={"start": 1, "end": 2, "peak": 3},
            )
            with native_runtime.stage("report", category="output"):
                time.sleep(0.001)
            native_runtime.finish()

            document = json.loads(output.read_text())
            self.assertEqual(document["schema_version"], 3)
            self.assertEqual(document["status"], "ok")
            self.assertEqual(document["command"], "kalibr_calibrate_cameras")
            categories = [stage["category"] for stage in document["stages"]]
            self.assertEqual(categories, ["optimization", "extraction", "output"])
            optimizer_stage = document["stages"][0]
            self.assertGreaterEqual(optimizer_stage["wall_seconds"], 0.0)
            self.assertIn(
                "lifetime_peak_at_end", optimizer_stage["rss_bytes"]
            )
            self.assertEqual(
                optimizer_stage["rss_bytes"]["scope"], "main_process"
            )
            self.assertEqual(
                optimizer_stage["pss_bytes"]["scope"], "main_process"
            )
            extraction = document["stages"][1]
            self.assertEqual(set(extraction["phases"]), {
                "bag_read", "deserialize", "decode", "detect", "total",
            })
            self.assertEqual(
                extraction["phases"]["detect"]["aggregation"],
                "sum_over_images",
            )
            self.assertEqual(
                extraction["phases"]["bag_read"]["wall_seconds"], 0.1)

    def test_all_optimizer_callsites_receive_type_preserving_source_overlay(self):
        upstream_directory = (
            ROOT
            / "upstream"
            / "kalibr"
            / "aslam_offline_calibration"
            / "kalibr"
            / "python"
        )
        expectations = {
            "kalibr_camera_calibration/CameraIntializers.py": (3, 0, 3),
            "kalibr_camera_calibration/CameraCalibrator.py": (0, 1, 1),
            "kalibr_imu_camera_calibration/IccSensors.py": (3, 0, 2),
            "kalibr_imu_camera_calibration/IccCalibrator.py": (1, 1, 2),
        }
        for relative, expected_counts in expectations.items():
            source = (upstream_directory / relative).read_text()
            transformed = transform_source(source, Path(relative).name)
            compile(transformed, relative, "exec")
            counts = (
                transformed.count("native_runtime.run_optimizer"),
                transformed.count("native_runtime.run_incremental_batch"),
                transformed.count("native_runtime.apply_optimizer_threads"),
            )
            self.assertEqual(counts, expected_counts, relative)
            if Path(relative).name == "IccCalibrator.py":
                for stage_name in (
                    "pose_spline_init",
                    "bias_spline_init",
                    "camera_error_build",
                    "accel_error_build",
                    "gyro_error_build",
                ):
                    self.assertIn(stage_name, transformed)
            self.assertEqual(
                transform_source(transformed, Path(relative).name), transformed
            )

    def test_non_positive_parallelism_is_rejected(self):
        parser = argparse.ArgumentParser()
        native_runtime.add_parallelism_arguments(parser)
        with self.assertRaises(SystemExit):
            parser.parse_args(["--parallelism", "0"])

    def test_invalid_timing_destination_does_not_poison_cleanup(self):
        runtime._set_profiling_for_tests(True)
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing" / "timing.json"
            with self.assertRaisesRegex(IOError, "does not exist"):
                native_runtime.configure(
                    self.arguments(timing_json=str(missing)))
            # There is no half-configured recorder left to mask the I/O error.
            native_runtime.finish("error")

    def test_disabled_profiling_rejects_diagnostic_io(self):
        runtime._set_profiling_for_tests(False)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "timing.json"
            with self.assertRaisesRegex(
                    RuntimeError, "KALIBR_ENABLE_PROFILING=ON"):
                native_runtime.configure(
                    self.arguments(timing_json=str(output)))
            self.assertFalse(output.exists())

    def test_both_calibration_commands_receive_the_same_cli_overlay(self):
        upstream_directory = (
            ROOT / "upstream" / "kalibr" / "aslam_offline_calibration"
            / "kalibr" / "python"
        )
        for command in (
                "kalibr_calibrate_cameras",
                "kalibr_calibrate_imu_camera"):
            source = (upstream_directory / command).read_text()
            transformed = transform_cli_source(source, command)
            compile(transformed, command, "exec")
            # The actual option strings live in the shared helper; each CLI
            # calls it once rather than duplicating four parser snippets.
            self.assertEqual(
                transformed.count("native_runtime.add_parallelism_arguments"),
                1,
            )
            self.assertIn(
                "native_runtime.configure(parsed, command={0!r})".format(
                    command),
                transformed,
            )
            self.assertIn(
                "parsed = parser.parse_args()\n    except Exception:",
                transformed,
            )
            self.assertNotIn(
                "parsed = parser.parse_args()\n    except:", transformed
            )
            if command == "kalibr_calibrate_cameras":
                self.assertIn(
                    "native_runtime.run_incremental_batch(\n"
                    "                                calibrator.estimator, "
                    "new_batch, False)",
                    transformed,
                )
                self.assertNotIn(
                    "calibrator.estimator.addBatch( new_batch, False )",
                    transformed,
                )
                self.assertIn("report_generation", transformed)
            else:
                self.assertIn("problem_build_total", transformed)
                self.assertIn("report_generation", transformed)
            self.assertEqual(
                transform_cli_source(transformed, command), transformed)


if __name__ == "__main__":
    unittest.main()
