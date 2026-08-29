import pickle
import tempfile
import unittest
from pathlib import Path

import yaml

try:
    import aslam_cameras_april as acv_april
    from kalibr_common.ConfigReader import CalibrationTargetParameters
except ImportError:
    acv_april = None
    CalibrationTargetParameters = None


@unittest.skipIf(
    acv_april is None or CalibrationTargetParameters is None,
    "the native build-tree Python packages are not on PYTHONPATH",
)
class AprilgridIdOffsetTest(unittest.TestCase):
    def _target(self, **updates):
        document = {
            "target_type": "aprilgrid",
            "tagRows": 3,
            "tagCols": 4,
            "tagSize": 0.04,
            "tagSpacing": 0.3,
        }
        document.update(updates)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "target.yaml"
        path.write_text(yaml.safe_dump(document), encoding="utf-8")
        return CalibrationTargetParameters(str(path)).getTargetParams()

    def test_missing_start_id_preserves_legacy_zero_default(self):
        self.assertEqual(self._target()["tagStartId"], 0)

    def test_nonzero_start_id_is_returned_and_mapped_row_major(self):
        target = self._target(tagStartId=100)
        options = acv_april.AprilgridOptions()
        grid = acv_april.GridCalibrationTargetAprilgrid(
            target["tagRows"], target["tagCols"], target["tagSize"],
            target["tagSpacing"], target["tagStartId"], options)

        self.assertEqual(grid.tagStartId(), 100)
        self.assertEqual(grid.localTagId(100), 0)
        self.assertEqual(grid.localTagId(103), 3)
        self.assertEqual(grid.localTagId(104), 4)
        self.assertEqual(grid.localTagId(111), 11)
        with self.assertRaises(RuntimeError):
            grid.localTagId(99)
        with self.assertRaises(RuntimeError):
            grid.localTagId(112)

    def test_start_id_validation_is_strict(self):
        for invalid in (-1, True, 1.5):
            with self.subTest(value=invalid):
                with self.assertRaisesRegex(RuntimeError, "tagStartId"):
                    self._target(tagStartId=invalid)

        with self.assertRaisesRegex(RuntimeError, "587 IDs"):
            self._target(tagStartId=576)

    def test_cpp_constructor_rejects_family_capacity_overflow(self):
        options = acv_april.AprilgridOptions()
        with self.assertRaises(RuntimeError):
            acv_april.GridCalibrationTargetAprilgrid(
                3, 4, 0.04, 0.3, 576, options)

    def test_pickle_preserves_start_id_for_detector_workers(self):
        options = acv_april.AprilgridOptions()
        self.assertEqual(options.subpixWindowHalfSize, 2)
        self.assertEqual(options.maxSubpixDisplacement2, 1.5)
        options.subpixWindowHalfSize = 4
        options.maxSubpixDisplacement2 = 6.25
        restored_options = pickle.loads(pickle.dumps(options))
        self.assertEqual(restored_options.subpixWindowHalfSize, 4)
        self.assertEqual(restored_options.maxSubpixDisplacement2, 6.25)
        grid = acv_april.GridCalibrationTargetAprilgrid(
            3, 4, 0.04, 0.3, 100, options)
        restored = pickle.loads(pickle.dumps(grid))
        self.assertEqual(restored.tagStartId(), 100)
        self.assertEqual(restored.localTagId(111), 11)


if __name__ == "__main__":
    unittest.main()
