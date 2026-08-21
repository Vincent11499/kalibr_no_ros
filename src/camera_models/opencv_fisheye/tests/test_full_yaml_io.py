import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "python"
    / "kalibr_opencv_fisheye_full"
    / "yaml_io.py"
)
SPEC = importlib.util.spec_from_file_location("opencv_fisheye_full_yaml", MODULE_PATH)
yaml_io = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = yaml_io
SPEC.loader.exec_module(yaml_io)


class FullOpenCvFisheyeYamlTest(unittest.TestCase):
    def camera(self, alpha=0.027, name="cam0", topic="/left"):
        fu = 411.2
        K = np.array(
            [
                [fu, fu * alpha, 319.7],
                [0.0, 407.8, 241.1],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        return yaml_io.OpenCvFisheyeCamera(
            K,
            np.array([-0.02, 0.003, -0.0004, 0.00003]),
            (640, 480),
            alpha,
            name,
            topic,
        )

    def assertCameraEqual(self, actual, expected, compare_name=True):
        np.testing.assert_allclose(actual.K, expected.K, rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(actual.D, expected.D, rtol=0.0, atol=1e-12)
        self.assertEqual(actual.resolution, expected.resolution)
        self.assertAlmostEqual(actual.alpha, expected.alpha, places=14)
        if compare_name:
            self.assertEqual(actual.name, expected.name)
        self.assertEqual(actual.topic, expected.topic)

    def test_mono_opencv_k_d_alpha_roundtrip(self):
        camera = self.camera()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mono.yaml"
            yaml_io.write_opencv_camera(camera, path)
            loaded = yaml_io.read_opencv_camera(path)
            self.assertCameraEqual(loaded, camera)

    def test_camchain_preserves_fifth_intrinsic_and_d_order(self):
        camera = self.camera(alpha=-0.019)
        with tempfile.TemporaryDirectory() as directory:
            camchain = Path(directory) / "camchain.yaml"
            yaml_io.write_kalibr_camchain(
                yaml_io.opencv_mono_to_camchain(camera), camchain
            )
            raw = yaml.safe_load(camchain.read_text(encoding="utf-8"))
            self.assertEqual(
                raw["cam0"]["camera_model"], "pinhole_opencv_fisheye"
            )
            self.assertEqual(len(raw["cam0"]["intrinsics"]), 5)
            self.assertAlmostEqual(raw["cam0"]["intrinsics"][4], -0.019)
            self.assertEqual(
                raw["cam0"]["distortion_coeffs"], camera.D.tolist()
            )
            loaded, transforms = yaml_io.read_kalibr_camchain(camchain)
            self.assertEqual(transforms, [])
            self.assertCameraEqual(loaded[0], camera)

    def test_nontrivial_stereo_roundtrip(self):
        left = self.camera(alpha=0.013, name="left", topic="/left")
        right = self.camera(alpha=-0.008, name="right", topic="/right")
        angle = 0.07
        R = np.array(
            [
                [np.cos(angle), 0.0, np.sin(angle)],
                [0.0, 1.0, 0.0],
                [-np.sin(angle), 0.0, np.cos(angle)],
            ]
        )
        T = np.array([-0.121, 0.003, -0.002])
        stereo = yaml_io.OpenCvFisheyeStereo(left, right, R, T)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stereo.yaml"
            yaml_io.write_opencv_stereo(stereo, path)
            loaded = yaml_io.read_opencv_stereo(path)
            self.assertCameraEqual(loaded.left, left)
            self.assertCameraEqual(loaded.right, right)
            np.testing.assert_allclose(loaded.R, R, rtol=0.0, atol=1e-12)
            np.testing.assert_allclose(loaded.T, T, rtol=0.0, atol=1e-12)

            camchain = Path(directory) / "camchain.yaml"
            yaml_io.write_kalibr_camchain(
                yaml_io.opencv_stereo_to_camchain(stereo), camchain
            )
            cameras, transforms = yaml_io.read_kalibr_camchain(camchain)
            self.assertCameraEqual(cameras[0], left, compare_name=False)
            self.assertCameraEqual(cameras[1], right, compare_name=False)
            np.testing.assert_allclose(transforms[0][0], R, atol=1e-12)
            np.testing.assert_allclose(transforms[0][1], T, atol=1e-12)

    def test_rejects_conflicting_k_and_explicit_alpha(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "conflict.yaml")
            storage = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
            K = np.array(
                [[400.0, 8.0, 320.0], [0.0, 400.0, 240.0], [0.0, 0.0, 1.0]]
            )
            storage.write("image_width", 640)
            storage.write("image_height", 480)
            storage.write("K", K)
            storage.write("D", np.zeros((4, 1)))
            storage.write("alpha", 0.03)
            storage.release()
            with self.assertRaisesRegex(ValueError, "inconsistent"):
                yaml_io.read_opencv_camera(path)

    def test_zero_skew_legacy_camchain_is_importable(self):
        data = {
            "cam0": {
                "camera_model": "pinhole",
                "intrinsics": [400.0, 401.0, 320.0, 240.0],
                "distortion_model": "opencv_fisheye",
                "distortion_coeffs": [0.1, 0.2, 0.3, 0.4],
                "resolution": [640, 480],
                "rostopic": "/cam0",
            }
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.yaml"
            path.write_text(yaml.safe_dump(data), encoding="utf-8")
            cameras, _ = yaml_io.read_kalibr_camchain(path)
            self.assertEqual(cameras[0].alpha, 0.0)
            self.assertEqual(cameras[0].K[0, 1], 0.0)

    def test_plain_yaml_flat_k_and_d(self):
        data = {
            "image_width": 640,
            "image_height": 480,
            "distortion_model": "fisheye",
            "K": [400.0, 8.0, 320.0, 0.0, 405.0, 240.0, 0.0, 0.0, 1.0],
            "D": [-0.1, 0.01, -0.001, 0.0001],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plain.yaml"
            path.write_text(yaml.safe_dump(data), encoding="utf-8")
            camera = yaml_io.read_opencv_camera(path)
            self.assertAlmostEqual(camera.alpha, 0.02)
            self.assertEqual(camera.D.tolist(), data["D"])

    def test_plain_stereo_flat_r_and_generic_alpha(self):
        R = np.array(
            [[0.9998, 0.0, 0.0199986667], [0.0, 1.0, 0.0], [-0.0199986667, 0.0, 0.9998]]
        )
        # Make the rounded example exactly orthonormal for strict validation.
        angle = 0.02
        R = np.array(
            [
                [np.cos(angle), 0.0, np.sin(angle)],
                [0.0, 1.0, 0.0],
                [-np.sin(angle), 0.0, np.cos(angle)],
            ]
        )
        T = np.array([-0.12, 0.003, 0.001])
        K = [400.0, 0.0, 320.0, 0.0, 402.0, 240.0, 0.0, 0.0, 1.0]
        data = {
            "left_image_width": 640,
            "left_image_height": 480,
            "right_image_width": 640,
            "right_image_height": 480,
            "distortion_model": "equidistant",
            "K1": K,
            "D1": [0.01, 0.02, 0.03, 0.04],
            "K2": K,
            "D2": [-0.01, -0.02, -0.03, -0.04],
            "alpha": 0.015,
            "R": R.reshape(-1).tolist(),
            "T": T.tolist(),
            "transform_direction": yaml_io.INVERSE_TRANSFORM,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plain-stereo.yaml"
            path.write_text(yaml.safe_dump(data), encoding="utf-8")
            stereo = yaml_io.read_opencv_stereo(path)
            self.assertAlmostEqual(stereo.left.alpha, 0.015)
            self.assertAlmostEqual(stereo.right.alpha, 0.015)
            expected_R, expected_T = yaml_io._zero.invert_stereo_transform(R, T)
            np.testing.assert_allclose(stereo.R, expected_R, atol=1e-12)
            np.testing.assert_allclose(
                stereo.T, expected_T.reshape(3), atol=1e-12
            )

    def test_stereo_4x4_rt_fallback(self):
        left = self.camera(alpha=0.0)
        transform = np.eye(4)
        transform[:3, 3] = [-0.1, 0.002, 0.003]
        data = {
            "left_image_width": 640,
            "left_image_height": 480,
            "right_image_width": 640,
            "right_image_height": 480,
            "distortion_model": "fisheye",
            "K1": left.K.reshape(-1).tolist(),
            "D1": left.D.tolist(),
            "K2": left.K.reshape(-1).tolist(),
            "D2": left.D.tolist(),
            "RT": transform.reshape(-1).tolist(),
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rt.yaml"
            path.write_text(yaml.safe_dump(data), encoding="utf-8")
            stereo = yaml_io.read_opencv_stereo(path)
            np.testing.assert_allclose(stereo.R, np.eye(3), atol=1e-12)
            np.testing.assert_allclose(stereo.T, transform[:3, 3], atol=1e-12)

    def test_rejects_explicit_plumb_bob_model(self):
        data = {
            "image_width": 640,
            "image_height": 480,
            "distortion_model": "plumb_bob",
            "K": [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0],
            "D": [0.1, 0.2, 0.3, 0.4],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "radtan.yaml"
            path.write_text(yaml.safe_dump(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "not an OpenCV fisheye"):
                yaml_io.read_opencv_camera(path)

    def test_translation_unit_metadata_converts_to_kalibr_meters(self):
        camera = self.camera(alpha=0.0)
        data = {
            "left_image_width": 640,
            "left_image_height": 480,
            "right_image_width": 640,
            "right_image_height": 480,
            "distortion_model": "fisheye",
            "K1": camera.K.reshape(-1).tolist(),
            "D1": camera.D.tolist(),
            "K2": camera.K.reshape(-1).tolist(),
            "D2": camera.D.tolist(),
            "R": np.eye(3).reshape(-1).tolist(),
            "T": [-120.0, 2.0, 1.0],
            "translation_unit": "mm",
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "millimetres.yaml"
            path.write_text(yaml.safe_dump(data), encoding="utf-8")
            stereo = yaml_io.read_opencv_stereo(path)
            np.testing.assert_allclose(
                stereo.T, [-0.12, 0.002, 0.001], atol=1e-15
            )

            # An explicit caller override takes precedence over file metadata.
            overridden = yaml_io.read_opencv_stereo(
                path, translation_scale=1e-2
            )
            np.testing.assert_allclose(
                overridden.T, [-1.2, 0.02, 0.01], atol=1e-15
            )


if __name__ == "__main__":
    unittest.main()
