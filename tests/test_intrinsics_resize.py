import importlib.util
from pathlib import Path
import unittest

import numpy as np


TOOL_PATH = Path(__file__).resolve().parents[1] / "tools" / "validate_intrinsics_resize.py"
TOOL_SPEC = importlib.util.spec_from_file_location(
    "validate_intrinsics_resize", TOOL_PATH)
TOOL = importlib.util.module_from_spec(TOOL_SPEC)
TOOL_SPEC.loader.exec_module(TOOL)
scale_intrinsics = TOOL.scale_intrinsics
transform_pixels = TOOL.transform_pixels
vector_statistics = TOOL.vector_statistics
per_frame_statistics = TOOL.per_frame_statistics
observed_pixel_coverage = TOOL.observed_pixel_coverage
recommended_parameter_document = TOOL.recommended_parameter_document


class IntrinsicsResizeTest(unittest.TestCase):
    def test_direct_scaling_preserves_dimensionless_fisheye_alpha(self):
        scaled = scale_intrinsics(
            [2400.0, 2390.0, 1919.5, 1079.5, 0.001],
            (3840, 2160), (1920, 1080), "direct_scale")
        np.testing.assert_allclose(
            scaled, [1200.0, 1195.0, 959.75, 539.75, 0.001])

    def test_half_pixel_rule_matches_opencv_resize_coordinates(self):
        scaled = scale_intrinsics(
            [2400.0, 2390.0, 1919.5, 1079.5],
            (3840, 2160), (1920, 1080), "opencv_half_pixel")
        np.testing.assert_allclose(
            scaled, [1200.0, 1195.0, 959.5, 539.5])
        points = np.asarray([[0.0, 0.0], [1919.5, 1079.5]])
        np.testing.assert_allclose(
            transform_pixels(
                points, (3840, 2160), (1920, 1080),
                "opencv_half_pixel"),
            [[-0.25, -0.25], [959.5, 539.5]],
        )

    def test_vector_statistics_use_two_dimensional_norm(self):
        statistics = vector_statistics([
            np.asarray([[3.0, 4.0], [0.0, 0.0]])])
        self.assertEqual(statistics["count"], 2)
        self.assertAlmostEqual(statistics["rms_norm_px"], 5.0 / np.sqrt(2.0))
        self.assertEqual(statistics["max_norm_px"], 5.0)

    def test_per_frame_statistics_report_worst_sample(self):
        statistics = per_frame_statistics([
            {
                "sample": 3,
                "timestamp_ns": 30,
                "residuals": np.asarray([[0.0, 1.0], [0.0, 1.0]]),
            },
            {
                "sample": 7,
                "timestamp_ns": 70,
                "residuals": np.asarray([[3.0, 4.0]]),
            },
        ])
        self.assertEqual(statistics["frame_count"], 2)
        self.assertEqual(statistics["worst_frame_sample"], 7)
        self.assertEqual(statistics["worst_frame_rms_norm_px"], 5.0)
        self.assertEqual(statistics["max_point_norm_px"], 5.0)

    def test_observed_pixel_coverage_uses_pixel_extent(self):
        coverage = observed_pixel_coverage(
            [np.asarray([[0.0, 0.0], [1919.0, 1079.0]])],
            (1920, 1080))
        self.assertEqual(coverage["count"], 2)
        self.assertEqual(
            coverage["bounding_box_fraction"],
            {"min_x": 0.0, "max_x": 1.0, "min_y": 0.0, "max_y": 1.0},
        )

    def test_invalid_convention_is_rejected(self):
        with self.assertRaises(ValueError):
            scale_intrinsics(
                [1.0, 1.0, 0.0, 0.0], (2, 2), (1, 1), "unknown")

    def test_recommended_parameter_document_keeps_distortion(self):
        summary = {
            "camera_index": 1,
            "topic": "/cam1",
            "source_resolution": [3840, 2160],
            "target_resolution": [1920, 1080],
            "resize_interpolation": "cv2.INTER_AREA",
            "models": {
                "radtan8": {
                    "source_calibration": "source.yaml",
                    "distortion_coefficients_unchanged": list(range(8)),
                    "recommended_convention": "opencv_half_pixel",
                    "candidates": {
                        "opencv_half_pixel": {
                            "scaled_intrinsics": [1.0, 2.0, 3.0, 4.0],
                        },
                    },
                },
            },
        }
        document = recommended_parameter_document(summary)
        self.assertEqual(document["camera_id"], "cam1")
        self.assertEqual(
            document["models"]["radtan8"]["distortion_coeffs"],
            list(range(8)),
        )
        self.assertEqual(
            document["models"]["radtan8"]["intrinsics"],
            [1.0, 2.0, 3.0, 4.0],
        )
        self.assertEqual(
            document["principal_point_convention"], "opencv_half_pixel")

    def test_recommended_document_rejects_mixed_conventions(self):
        summary = {
            "camera_index": 1,
            "topic": "/cam1",
            "source_resolution": [3840, 2160],
            "target_resolution": [1920, 1080],
            "resize_interpolation": "cv2.INTER_AREA",
            "models": {},
        }
        for index, convention in enumerate(TOOL.CONVENTIONS):
            summary["models"]["model{}".format(index)] = {
                "source_calibration": "source.yaml",
                "distortion_coefficients_unchanged": [],
                "recommended_convention": convention,
                "candidates": {
                    convention: {"scaled_intrinsics": [1.0] * 4},
                },
            }
        with self.assertRaises(RuntimeError):
            recommended_parameter_document(summary)


if __name__ == "__main__":
    unittest.main()
