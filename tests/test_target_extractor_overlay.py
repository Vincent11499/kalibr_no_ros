import importlib.util
import json
import multiprocessing
import os
import queue
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
NATIVE_PYTHON = ROOT / "extensions" / "native_optimizer" / "python"
sys.path.insert(0, str(NATIVE_PYTHON))
from kalibr_native_optimizer import runtime as native_optimizer_runtime

OVERLAY = (
    ROOT
    / "extensions"
    / "native_optimizer"
    / "python"
    / "kalibr_common"
    / "TargetExtractor.py"
)
RESULT_UNPICKLED_EVENT = None


class FakeProgress:
    def __init__(self, unused_total):
        self.samples = 0

    def sample(self, count=1):
        self.samples += count


class FakeObservation:
    def __init__(self, index, image):
        self.index = index
        self.image = image

    def clearImage(self):
        self.image = None


class FakeDetector:
    def __init__(self, failing_index=None):
        self.failing_index = failing_index

    def findTarget(self, stamp, image):
        if stamp == self.failing_index:
            raise ValueError("synthetic detector failure")
        return stamp % 3 != 0, FakeObservation(stamp, image)

    def findTargetNoTransformation(self, stamp, image):
        return self.findTarget(stamp, image)


class CrashingDetector(FakeDetector):
    def findTarget(self, stamp, image):
        if stamp == 5:
            os._exit(23)
        return super().findTarget(stamp, image)


class UnpicklableObservationDetector(FakeDetector):
    def findTarget(self, stamp, image):
        observation = FakeObservation(stamp, image)
        observation.unpicklable = lambda: None
        return True, observation


class FakeDataset:
    topic = "/camera"

    def __init__(self, count):
        self.count = count

    def numImages(self):
        return self.count

    def readDataset(self):
        for index in range(self.count):
            yield index, np.full((4, 5), index, dtype=np.uint8)


class TimedFakeDataset(FakeDataset):
    def readDatasetWithTiming(self):
        for index in range(self.count):
            yield (
                index,
                np.full((4, 5), index, dtype=np.uint8),
                {
                    "bag_read_wall_seconds": 0.01,
                    "bag_read_cpu_seconds": 0.008,
                    "deserialize_wall_seconds": 0.02,
                    "deserialize_cpu_seconds": 0.015,
                    "decode_wall_seconds": 0.03,
                    "decode_cpu_seconds": 0.025,
                },
            )


class DeferredImage:
    def __init__(self, index):
        self.index = index

    def decode_for_kalibr(self, collect_timing=False):
        timing = {}
        if collect_timing:
            timing = {
                "deserialize_wall_seconds": 0.02,
                "deserialize_cpu_seconds": 0.015,
                "decode_wall_seconds": 0.03,
                "decode_cpu_seconds": 0.025,
            }
        return np.full((4, 5), self.index, dtype=np.uint8), timing


class DeferredFakeDataset(FakeDataset):
    def readDatasetDeferred(self):
        for index in range(self.count):
            yield index, DeferredImage(index)

    def readDatasetDeferredWithTiming(self):
        for index in range(self.count):
            yield index, DeferredImage(index), {
                "bag_read_wall_seconds": 0.01,
                "bag_read_cpu_seconds": 0.008,
            }


class AcknowledgedObservation(FakeObservation):
    def __setstate__(self, state):
        self.__dict__.update(state)
        # Queue unpickling happens in the consumer process.  This lets the
        # dataset verify that result draining starts before all images decode.
        if RESULT_UNPICKLED_EVENT is not None:
            RESULT_UNPICKLED_EVENT.set()


class AcknowledgedDetector:
    def findTarget(self, stamp, image):
        return True, AcknowledgedObservation(stamp, image)

    def findTargetNoTransformation(self, stamp, image):
        return self.findTarget(stamp, image)


class DrainAwareDataset(FakeDataset):
    def __init__(self, count, first_drain_index, acknowledgement):
        super().__init__(count)
        self.first_drain_index = first_drain_index
        self.acknowledgement = acknowledgement

    def readDataset(self):
        for index in range(self.count):
            if index == self.first_drain_index:
                if not self.acknowledgement.wait(timeout=3.0):
                    raise RuntimeError("result queue was not drained")
            yield index, np.full((4, 5), index, dtype=np.uint8)


class StubbornProcess:
    pid = 4321
    exitcode = None

    def __init__(self):
        self.terminate_calls = 0
        self.kill_calls = 0
        self.join_calls = 0

    def is_alive(self):
        return True

    def terminate(self):
        self.terminate_calls += 1

    def kill(self):
        self.kill_calls += 1

    def join(self, unused_timeout=None):
        self.join_calls += 1


def load_overlay():
    fake_sm = types.ModuleType("sm")
    fake_sm.Progress2 = FakeProgress
    fake_sm.logFatal = lambda message: (_ for _ in ()).throw(RuntimeError(message))
    previous_sm = sys.modules.get("sm")
    sys.modules["sm"] = fake_sm
    try:
        spec = importlib.util.spec_from_file_location(
            "kalibr_target_extractor_overlay", OVERLAY
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        if previous_sm is None:
            del sys.modules["sm"]
        else:
            sys.modules["sm"] = previous_sm


class TargetExtractorOverlayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_overlay()
        cls.module.cv2.destroyAllWindows = lambda: None

    def tearDown(self):
        native_optimizer_runtime._reset_for_tests()

    def test_parallel_results_are_ordered_and_images_are_released(self):
        observations = self.module.extractCornersFromDataset(
            FakeDataset(25), FakeDetector(), multithreading=True,
            numProcesses=3, clearImages=True
        )
        self.assertEqual(
            [observation.index for observation in observations],
            [index for index in range(25) if index % 3 != 0],
        )
        self.assertTrue(all(observation.image is None for observation in observations))

    def test_worker_exception_is_reported_without_deadlock(self):
        with self.assertRaisesRegex(RuntimeError, "synthetic detector failure"):
            self.module.extractCornersFromDataset(
                FakeDataset(12), FakeDetector(failing_index=5),
                multithreading=True, numProcesses=2
            )

    def test_hard_worker_exit_is_reported_without_deadlock(self):
        with self.assertRaisesRegex(RuntimeError, "exited unexpectedly"):
            self.module.extractCornersFromDataset(
                FakeDataset(12), CrashingDetector(),
                multithreading=True, numProcesses=2,
            )

    def test_unpicklable_worker_result_is_reported_without_deadlock(self):
        with self.assertRaisesRegex(RuntimeError, "pickle|Pickl"):
            self.module.extractCornersFromDataset(
                FakeDataset(3), UnpicklableObservationDetector(),
                multithreading=True, numProcesses=1,
            )

    def test_submission_and_result_drain_are_interleaved(self):
        global RESULT_UNPICKLED_EVENT
        acknowledgement = multiprocessing.Event()
        RESULT_UNPICKLED_EVENT = acknowledgement
        try:
            observations = self.module.extractCornersFromDataset(
                DrainAwareDataset(20, 4, acknowledgement),
                AcknowledgedDetector(),
                multithreading=True,
                numProcesses=2,
            )
        finally:
            RESULT_UNPICKLED_EVENT = None
        self.assertEqual(
            [observation.index for observation in observations],
            list(range(20)),
        )

    def test_worker_limits_opencv_to_one_thread(self):
        taskq = queue.Queue()
        resultq = queue.Queue()
        taskq.put(None)
        with mock.patch.object(self.module.cv2, "setNumThreads") as setter:
            self.module.multicoreExtractionWrapper(
                FakeDetector(), taskq, resultq, True, False)
        setter.assert_called_once_with(1)

    def test_disabled_timing_avoids_clocks_and_memory_probes(self):
        with mock.patch.object(
                self.module.time, "perf_counter",
                side_effect=AssertionError("unexpected wall-clock probe")), \
             mock.patch.object(
                 self.module.time, "process_time",
                 side_effect=AssertionError("unexpected CPU probe")), \
             mock.patch.object(
                 self.module, "_readProcessMemory",
                 side_effect=AssertionError("unexpected memory probe")):
            observations = self.module.extractCornersFromDataset(
                FakeDataset(4), FakeDetector(), multithreading=False)
        self.assertEqual(
            [observation.index for observation in observations], [1, 2])

    def test_worker_fast_path_does_not_read_measurement_clocks(self):
        taskq = queue.Queue()
        resultq = queue.Queue()
        taskq.put(self.module.pickle.dumps(
            (0, 1, np.zeros((2, 2), dtype=np.uint8))))
        taskq.put(None)
        with mock.patch.object(
                self.module.time, "perf_counter",
                side_effect=AssertionError("unexpected wall-clock probe")), \
             mock.patch.object(
                 self.module.time, "process_time",
                 side_effect=AssertionError("unexpected CPU probe")):
            self.module.multicoreExtractionWrapper(
                FakeDetector(), taskq, resultq, True, False,
                collectTiming=False)
        result = self.module.pickle.loads(resultq.get_nowait())
        status, idx, unused_payload, wall, cpu = result[:5]
        self.assertEqual((status, idx, wall, cpu), ("result", 0, 0.0, 0.0))

    def test_worker_error_is_reported_before_invalid_index(self):
        with self.assertRaisesRegex(RuntimeError, "invalid serialized task"):
            self.module._validateWorkerResult(
                "error",
                -1,
                {"message": "invalid serialized task", "traceback": "trace"},
                submitted=1,
                observationsByIndex={},
            )

    def test_join_timeout_does_not_stop_processes_twice(self):
        process = StubbornProcess()
        with self.assertRaisesRegex(RuntimeError, "did not stop"):
            self.module._joinProcesses([process], timeout=0.0)
        self.assertEqual(process.terminate_calls, 0)
        self.assertEqual(process.kill_calls, 0)

    def test_stop_rechecks_survivors_after_kill(self):
        process = StubbornProcess()
        with self.assertRaisesRegex(RuntimeError, "could not be stopped"):
            self.module._stopProcesses([process], timeout=0.0)
        self.assertEqual(process.terminate_calls, 1)
        self.assertEqual(process.kill_calls, 1)

    def test_process_tree_tracker_sums_parent_and_live_workers(self):
        process = mock.Mock()
        process.pid = 20
        process.is_alive.return_value = True
        measurements = {
            10: {"rss": 100, "pss": 60},
            20: {"rss": 80, "pss": 40},
        }
        tracker = self.module._ProcessTreeMemoryTracker(
            multithreading=True, sampleInterval=0.0)
        with mock.patch.object(self.module.os, "getpid", return_value=10), \
             mock.patch.object(
                 self.module, "_readProcessMemory",
                 side_effect=lambda pid: measurements[pid]):
            tracker.sample([process], force=True)
            tracker.sample([], force=True)
        rss, pss = tracker.fields()
        self.assertEqual(rss["peak"], 180)
        self.assertEqual(pss["peak"], 100)
        self.assertEqual(rss["max_processes"], 2)
        self.assertEqual(rss["scope"], "parent_and_detector_workers")
        self.assertEqual(
            rss["peak_semantics"],
            "maximum_of_periodic_process_tree_samples",
        )

    def test_runtime_process_override_and_extraction_timing(self):
        native_optimizer_runtime._set_profiling_for_tests(True)
        with tempfile.TemporaryDirectory() as directory:
            timing_path = Path(directory) / "extract.json"
            native_optimizer_runtime.configure({
                "parallelism": None,
                "detector_processes": 1,
                "optimizer_threads": None,
                "timing_json": str(timing_path),
            }, command="kalibr_calibrate_cameras")
            observations = self.module.extractCornersFromDataset(
                FakeDataset(8), FakeDetector(), multithreading=True)
            native_optimizer_runtime.finish()

            self.assertEqual(
                [observation.index for observation in observations],
                [1, 2, 4, 5, 7],
            )
            document = json.loads(timing_path.read_text())
            extraction = document["stages"][0]
            self.assertEqual(extraction["category"], "extraction")
            self.assertEqual(extraction["metadata"]["worker_processes"], 1)
            self.assertEqual(
                extraction["pss_bytes"]["scope"],
                "parent_and_detector_workers",
            )
            self.assertGreaterEqual(extraction["pss_bytes"]["max_processes"], 2)
            self.assertEqual(
                extraction["pss_bytes"]["peak_semantics"],
                "maximum_of_periodic_process_tree_samples",
            )
            self.assertEqual(set(extraction["phases"]), {
                "bag_read", "deserialize", "decode", "detect", "total",
            })

    def test_detailed_image_phases_are_recorded_independently(self):
        native_optimizer_runtime._set_profiling_for_tests(True)
        with tempfile.TemporaryDirectory() as directory:
            timing_path = Path(directory) / "extract.json"
            native_optimizer_runtime.configure({
                "parallelism": None,
                "detector_processes": None,
                "optimizer_threads": None,
                "timing_json": str(timing_path),
            }, command="kalibr_calibrate_cameras")
            self.module.extractCornersFromDataset(
                TimedFakeDataset(4), FakeDetector(), multithreading=False)
            native_optimizer_runtime.finish()

            extraction = json.loads(timing_path.read_text())["stages"][0]
            self.assertTrue(
                extraction["metadata"]["image_phase_detail_available"])
            self.assertAlmostEqual(
                extraction["phases"]["bag_read"]["wall_seconds"], 0.04)
            self.assertAlmostEqual(
                extraction["phases"]["deserialize"]["wall_seconds"], 0.08)
            self.assertAlmostEqual(
                extraction["phases"]["decode"]["wall_seconds"], 0.12)

    def test_parallel_deferred_decode_runs_in_workers_and_is_timed(self):
        native_optimizer_runtime._set_profiling_for_tests(True)
        with tempfile.TemporaryDirectory() as directory:
            timing_path = Path(directory) / "extract.json"
            native_optimizer_runtime.configure({
                "parallelism": None,
                "detector_processes": 2,
                "optimizer_threads": None,
                "timing_json": str(timing_path),
            }, command="kalibr_calibrate_cameras")
            observations = self.module.extractCornersFromDataset(
                DeferredFakeDataset(4), FakeDetector(), multithreading=True)
            native_optimizer_runtime.finish()

            self.assertEqual([item.index for item in observations], [1, 2])
            extraction = json.loads(timing_path.read_text())["stages"][0]
            self.assertAlmostEqual(
                extraction["phases"]["bag_read"]["wall_seconds"], 0.04)
            self.assertAlmostEqual(
                extraction["phases"]["deserialize"]["wall_seconds"], 0.08)
            self.assertAlmostEqual(
                extraction["phases"]["decode"]["wall_seconds"], 0.12)


if __name__ == "__main__":
    unittest.main()
