import importlib.util
import sys
import types
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OVERLAY = (
    ROOT
    / "extensions"
    / "native_optimizer"
    / "python"
    / "kalibr_common"
    / "TargetExtractor.py"
)


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


class FakeDataset:
    topic = "/camera"

    def __init__(self, count):
        self.count = count

    def numImages(self):
        return self.count

    def readDataset(self):
        for index in range(self.count):
            yield index, np.full((4, 5), index, dtype=np.uint8)


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


if __name__ == "__main__":
    unittest.main()
