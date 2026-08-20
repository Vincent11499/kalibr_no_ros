import argparse
import tempfile
import unittest
from pathlib import Path

try:
    import aslam_backend
    import incremental_calibration
    import kalibr_native_optimizer
    from kalibr_native_optimizer import runtime
except ImportError:
    aslam_backend = None
    incremental_calibration = None
    kalibr_native_optimizer = None
    runtime = None


@unittest.skipIf(
    aslam_backend is None,
    "the native build and its runtime libraries are not available",
)
class NativeBoostTypeCompatibilityTest(unittest.TestCase):
    def tearDown(self):
        runtime._reset_for_tests()

    def test_thread_and_timing_controls_preserve_native_types(self):
        optimizer_type = aslam_backend.Optimizer2
        estimator_type = incremental_calibration.IncrementalEstimator
        with tempfile.TemporaryDirectory() as directory:
            timing_path = Path(directory) / "timing.json"
            timing_output = (
                str(timing_path)
                if kalibr_native_optimizer.profiling_enabled() else None
            )
            kalibr_native_optimizer.configure(
                argparse.Namespace(
                    parallelism=None,
                    detector_processes=None,
                    optimizer_threads=3,
                    timing_json=timing_output,
                ),
                command="boost-type-smoke",
            )

            self.assertIs(aslam_backend.Optimizer2, optimizer_type)
            self.assertIs(
                incremental_calibration.IncrementalEstimator, estimator_type
            )

            options = aslam_backend.Optimizer2Options()
            options.nThreads = 4
            kalibr_native_optimizer.apply_optimizer_threads(options)
            optimizer = aslam_backend.Optimizer2(options)
            self.assertIs(type(optimizer), optimizer_type)
            self.assertEqual(optimizer.options.nThreads, 3)

            estimator = incremental_calibration.IncrementalEstimator(0)
            self.assertIs(type(estimator), estimator_type)
            estimator_options = estimator.getOptimizerOptions()
            estimator_options.nThreads = 12
            kalibr_native_optimizer.apply_optimizer_threads(estimator_options)
            self.assertEqual(estimator_options.nThreads, 3)

            kalibr_native_optimizer.finish()
            self.assertEqual(
                timing_path.is_file(),
                kalibr_native_optimizer.profiling_enabled(),
            )
            runtime._reset_for_tests()


if __name__ == "__main__":
    unittest.main(verbosity=2)
