from pathlib import Path
import sys
import unittest

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_tools import stereo_tag_distance as TOOL


class StereoTagDistanceToolTest(unittest.TestCase):
    def camera(self):
        intrinsics = np.array([500.0, 505.0, 320.0, 240.0])
        return {
            "K": np.array([[intrinsics[0], 0.0, intrinsics[2]],
                           [0.0, intrinsics[1], intrinsics[3]],
                           [0.0, 0.0, 1.0]]),
            "distortion_coeffs": np.array([-0.03, 0.005, -0.001, 0.0001]),
        }

    def test_parses_both_calibration_layouts(self):
        transform = np.eye(4)
        transform[0, 3] = -0.38
        common = {
            "model": "pinhole-equidistant",
            "resolution": [3840, 2160],
            "intrinsic": [2400, 2401, 1920, 1080],
            "distortion": [-0.1, -0.02, 0.03, -0.01],
        }
        machine = {"camera": {
            "cam0": dict(common),
            "cam1": {**common, "T_cn_cnm1": transform.tolist()},
        }}
        self.assertEqual(TOOL._parse_camera(machine, "cam0")["K"].shape, (3, 3))

        result_common = {
            "camera_model": "pinhole",
            "distortion_model": "equidistant",
            "resolution": [3840, 2160],
            "intrinsics": [2400, 2401, 1920, 1080],
            "distortion_coeffs": [-0.1, -0.02, 0.03, -0.01],
        }
        result = {"cameras": [
            {"id": "cam0", **result_common},
            {"id": "cam1", **result_common,
             "T_cn_cnm1": transform.tolist()},
        ]}
        self.assertEqual(TOOL._parse_camera(result, "cam1")["K"].shape, (3, 3))

    def test_fisheye_triangulation_recovers_points(self):
        cam0 = self.camera()
        cam1 = self.camera()
        transform = np.eye(4)
        transform[0, 3] = -0.38
        points = np.array([
            [-0.2, -0.1, 3.0], [0.0, -0.1, 3.0],
            [0.0, 0.1, 3.0], [-0.2, 0.1, 3.0],
        ])
        points1 = points + transform[:3, 3]
        pixels0, _ = cv2.fisheye.projectPoints(
            points.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
            cam0["K"], cam0["distortion_coeffs"])
        pixels1, _ = cv2.fisheye.projectPoints(
            points1.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
            cam1["K"], cam1["distortion_coeffs"])
        recovered = TOOL.triangulate_points(
            pixels0.reshape(-1, 2), pixels1.reshape(-1, 2),
            cam0, cam1, transform)
        np.testing.assert_allclose(recovered, points, atol=1e-9)

    def test_distance_statistics_use_sample_standard_deviation(self):
        statistics = TOOL.distance_statistics([1000.0, 1002.0, 1004.0])
        self.assertEqual(statistics["mean"], 1002.0)
        self.assertEqual(statistics["median"], 1002.0)
        self.assertEqual(statistics["sample_std"], 2.0)
        self.assertEqual(statistics["range"], 4.0)

    def test_detection_image_marks_tags(self):
        image = np.zeros((240, 320), dtype=np.uint8)
        detected = {
            "border_bits": 1,
            "tags": {
                0: np.array([[40, 180], [100, 180], [100, 120], [40, 120]]),
                1: np.array([[180, 100], [240, 100], [240, 40], [180, 40]]),
            },
        }
        canvas = TOOL.draw_detection(
            image, detected, "cam0", 1, 123, (0, 1), 1002.42)
        self.assertEqual(canvas.shape, (240, 320, 3))
        self.assertGreater(int(np.max(canvas)), 0)

    def test_default_output_is_outside_dataset(self):
        dataset = Path("/data/intrinsic-02/stereo_261118_1429")
        pairs, summary, detections = TOOL._output_paths(
            dataset, None, None, None)
        output = Path("/data/intrinsic-02/result/distance_261118_1429")
        self.assertEqual(pairs, output / "distance.csv")
        self.assertEqual(summary, output / "distance_summary.csv")
        self.assertEqual(detections, output / "distance_detections")


if __name__ == "__main__":
    unittest.main()
