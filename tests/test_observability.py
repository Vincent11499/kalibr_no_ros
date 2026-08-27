import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_no_ros import observability


class _Dv:
    def __init__(self, dimension):
        self.dimension = dimension

    def minimalDimensions(self):
        return self.dimension


class _Transformation:
    def __init__(self):
        self.q = _Dv(3)
        self.t = _Dv(3)


class ObservabilityReportTest(unittest.TestCase):
    def setUp(self):
        self.previous = sys.modules.get("incremental_calibration")
        module = types.ModuleType("incremental_calibration")
        module.analyzeObservability = self._analyze
        sys.modules["incremental_calibration"] = module

    def tearDown(self):
        if self.previous is None:
            sys.modules.pop("incremental_calibration", None)
        else:
            sys.modules["incremental_calibration"] = self.previous

    @staticmethod
    def _analyze(problem, selected, threads):
        del problem, threads
        return {
            "state": "final_relinearized",
            "uses_m_estimator": True,
            "damping_applied": False,
            "column_scaling": "l2",
            "nuisance_dimensions": 12,
            "calibration_dimensions": 3,
            "rank": 2,
            "deficiency": 1,
            "qr_rank": 12,
            "qr_deficiency": 0,
            "qr_tolerance": 1e-12,
            "svd_tolerance": 1e-6,
            "singular_values": np.array([4.0, 1.0, 0.0]),
            "nullspace": np.array([[0.0], [0.6], [0.8]]),
            "selected_active": [True for _ in selected],
        }

    def test_rank_deficiency_and_block_contributions(self):
        report = observability.analyze(
            object(),
            [
                {"name": "rotation", "design_variables": [_Dv(1)]},
                {"name": "translation", "design_variables": [_Dv(2)]},
            ],
            job="camera_imu_calibration",
            num_threads=4,
        )
        self.assertEqual(report["status"], "rank_deficient")
        self.assertEqual(report["calibration"]["rank"], 2)
        self.assertEqual(
            report["calibration"]["condition_jacobian_equivalent"], 2.0)
        dominant = report["nullspace_modes"][0]["dominant_blocks"]
        self.assertEqual(dominant[0]["block"], "translation")
        self.assertAlmostEqual(dominant[0]["contribution"], 1.0)
        with self.assertRaises(observability.RankDeficiencyError):
            observability.require_full_rank(report)

    def test_operational_truncation_is_a_warning_not_a_hard_failure(self):
        def analyze_weak(problem, selected, threads):
            del problem, threads
            return {
                "state": "final_relinearized",
                "uses_m_estimator": True,
                "damping_applied": False,
                "column_scaling": "l2",
                "nuisance_dimensions": 12,
                "calibration_dimensions": 3,
                "rank": 3,
                "deficiency": 0,
                "qr_rank": 12,
                "qr_deficiency": 0,
                "qr_tolerance": 1e-12,
                "svd_tolerance": 1e-14,
                "singular_values": np.array([4.0, 1.0, 1e-5]),
                "nullspace": np.empty((3, 0)),
                "operational_eps_svd": 1e-6,
                "operational_rank": 2,
                "operational_deficiency": 1,
                "operational_svd_tolerance": 1.2e-5,
                "operational_nullspace": np.array(
                    [[0.0], [0.6], [0.8]]),
                "selected_active": [True for _ in selected],
            }

        sys.modules["incremental_calibration"].analyzeObservability = analyze_weak
        report = observability.analyze(
            object(),
            [
                {"name": "rotation", "design_variables": [_Dv(1)]},
                {"name": "translation", "design_variables": [_Dv(2)]},
            ],
            job="camera_imu_calibration",
        )

        self.assertEqual(report["status"], "full_rank")
        self.assertEqual(report["quality"], "weakly_observable")
        self.assertEqual(report["calibration"]["rank"], 3)
        self.assertEqual(report["calibration"]["operational_rank"], 2)
        self.assertEqual(
            report["operationally_truncated_modes"][0]
            ["dominant_blocks"][0]["block"],
            "translation",
        )
        self.assertIs(observability.require_full_rank(report), report)

    def test_report_uses_yaml_null_for_undefined_spectral_gap(self):
        report = observability.analyze(
            object(),
            [{"name": "calibration", "design_variables": [_Dv(3)]}],
            job="camera_calibration",
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "observability.yaml"
            observability.write_report(report, output)
            loaded = yaml.safe_load(output.read_text(encoding="utf-8"))
        self.assertEqual(loaded["calibration"]["smallest"], 0.0)
        self.assertIsNone(loaded["calibration"]["spectral_gap_at_rank"])

    def test_camera_parameter_blocks_have_physical_order_and_units(self):
        geometry_dv = type("GeometryDv", (), {
            "projectionDesignVariable": lambda self: _Dv(4),
            "distortionDesignVariable": lambda self: _Dv(5),
        })()
        camera = type("Camera", (), {"dv": geometry_dv})()
        calibrator = type("Calibrator", (), {
            "cameras": [camera, camera],
            "baselines": [_Transformation()],
        })()
        blocks = observability.camera_parameter_blocks(calibrator)
        self.assertEqual(blocks[0]["name"], "cameras.cam0.intrinsics")
        self.assertEqual(blocks[-1]["name"],
                         "camera_chain.cam1.translation")
        self.assertEqual(blocks[-1]["units"], "metres")

    def test_camera_imu_block_names_match_public_seed_keys(self):
        camera = type("Camera", (), {
            "T_c_b_Dv": _Transformation(),
            "cameraTimeToImuTimeDv": _Dv(1),
        })()
        calibrator = type("Calibrator", (), {
            "CameraChain": type("Chain", (), {"camList": [camera]})(),
            "ImuList": [],
        })()
        blocks = observability.camera_imu_parameter_blocks(calibrator)
        self.assertEqual(
            blocks[0]["name"], "camera_imu.T_cam0_imu.rotation")
        self.assertEqual(
            blocks[1]["name"], "camera_imu.T_cam0_imu.translation")

    def test_failed_native_analysis_is_written_before_reraising(self):
        sys.modules["incremental_calibration"].analyzeObservability = (
            lambda *unused: (_ for _ in ()).throw(RuntimeError("linear failure"))
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "observability.yaml"
            with self.assertRaisesRegex(RuntimeError, "linear failure"):
                observability.analyze_write_require(
                    object(),
                    [{"name": "calibration", "design_variables": [_Dv(1)]}],
                    job="camera_calibration", path=output)
            loaded = yaml.safe_load(output.read_text(encoding="utf-8"))
        self.assertEqual(loaded["status"], "analysis_failed")
        self.assertEqual(loaded["failure"]["message"], "linear failure")


if __name__ == "__main__":
    unittest.main()
