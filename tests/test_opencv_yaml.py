import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    ROOT
    / "extensions"
    / "opencv_fisheye"
    / "python"
    / "kalibr_opencv_fisheye"
    / "yaml_io.py"
)
SPEC = importlib.util.spec_from_file_location("kalibr_opencv_yaml_test_module", MODULE_PATH)
opencv_yaml = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = opencv_yaml
SPEC.loader.exec_module(opencv_yaml)


class OpenCvYamlRoundTripTest(unittest.TestCase):
    def setUp(self):
        self.K0 = np.array(
            [[421.3, 0.0, 319.2], [0.0, 420.7, 241.1], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.K1 = np.array(
            [[425.8, 0.0, 322.4], [0.0, 424.2, 238.6], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.D0 = np.array([-0.031, 0.0042, -0.00071, 0.000052])
        self.D1 = np.array([-0.027, 0.0035, -0.00052, 0.000041])
        self.R = cv2.Rodrigues(np.array([0.12, -0.08, 0.045]))[0]
        self.T = np.array([[-0.118], [0.0043], [0.0071]])
        self.left = opencv_yaml.OpenCvCamera(
            self.K0,
            self.D0,
            (640, 480),
            "opencv_fisheye",
            "left",
            "/left/image_raw",
        )
        self.right = opencv_yaml.OpenCvCamera(
            self.K1,
            self.D1,
            (648, 486),
            "opencv_fisheye",
            "right",
            "/right/image_raw",
        )
        self.stereo = opencv_yaml.OpenCvStereo(self.left, self.right, self.R, self.T)

    def _write_embedded_stereo(self, path, rotation, translation, metadata=()):
        opencv_yaml._write_values(
            path,
            [
                ("left_image_width", self.left.resolution[0]),
                ("left_image_height", self.left.resolution[1]),
                ("right_image_width", self.right.resolution[0]),
                ("right_image_height", self.right.resolution[1]),
                ("distortion_model", "fisheye"),
                ("K1", self.K0),
                ("D1", self.D0.reshape(4, 1)),
                ("K2", self.K1),
                ("D2", self.D1.reshape(4, 1)),
                ("R", rotation),
                ("T", translation),
            ]
            + list(metadata),
        )

    def test_nontrivial_stereo_direction_and_file_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "stereo.yaml"
            opencv_yaml.write_opencv_stereo(self.stereo, output)
            loaded = opencv_yaml.read_opencv_stereo(output)

            np.testing.assert_allclose(loaded.R, self.R, rtol=0.0, atol=1e-15)
            np.testing.assert_allclose(loaded.T, self.T, rtol=0.0, atol=1e-15)
            np.testing.assert_allclose(loaded.transform[:3, :3], self.R)
            np.testing.assert_allclose(loaded.transform[:3, 3:4], self.T)
            self.assertEqual(loaded.left.resolution, (640, 480))
            self.assertEqual(loaded.right.resolution, (648, 486))

            storage = cv2.FileStorage(str(output), cv2.FILE_STORAGE_READ)
            try:
                self.assertEqual(storage.getNode("D1").mat().shape, (4, 1))
                self.assertEqual(storage.getNode("D2").mat().shape, (4, 1))
                self.assertEqual(
                    storage.getNode("transform_direction").string(),
                    opencv_yaml.DIRECT_TRANSFORM,
                )
                self.assertEqual(storage.getNode("translation_unit").string(), "m")
                self.assertEqual(storage.getNode("translation_scale").real(), 1.0)
                essential = storage.getNode("E").mat()
                fundamental = storage.getNode("F").mat()
            finally:
                storage.release()
            expected_essential = opencv_yaml._skew(self.T).dot(self.R)
            expected_fundamental = np.linalg.inv(self.K1).T.dot(
                expected_essential
            ).dot(np.linalg.inv(self.K0))
            np.testing.assert_allclose(essential, expected_essential)
            np.testing.assert_allclose(fundamental, expected_fundamental)

    def test_explicit_reverse_transform_is_inverted(self):
        inverse_R, inverse_T = opencv_yaml.invert_stereo_transform(self.R, self.T)
        reversed_stereo = opencv_yaml.OpenCvStereo(
            self.left, self.right, inverse_R, inverse_T
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vendor-reverse.yaml"
            opencv_yaml.write_opencv_stereo(reversed_stereo, path)
            loaded = opencv_yaml.read_opencv_stereo(
                path, transform_direction=opencv_yaml.INVERSE_TRANSFORM
            )
        np.testing.assert_allclose(loaded.R, self.R, rtol=0.0, atol=2e-15)
        np.testing.assert_allclose(loaded.T, self.T, rtol=0.0, atol=2e-15)

    def test_transform_direction_from_file_is_honored_and_can_be_overridden(self):
        inverse_R, inverse_T = opencv_yaml.invert_stereo_transform(self.R, self.T)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "declared-reverse.yaml"
            self._write_embedded_stereo(
                path,
                inverse_R,
                inverse_T,
                [("transform_direction", opencv_yaml.INVERSE_TRANSFORM)],
            )
            loaded = opencv_yaml.read_opencv_stereo(path)
            overridden = opencv_yaml.read_opencv_stereo(
                path, transform_direction=opencv_yaml.DIRECT_TRANSFORM
            )

        np.testing.assert_allclose(loaded.R, self.R, rtol=0.0, atol=2e-15)
        np.testing.assert_allclose(loaded.T, self.T, rtol=0.0, atol=2e-15)
        np.testing.assert_allclose(overridden.R, inverse_R, rtol=0.0, atol=2e-15)
        np.testing.assert_allclose(overridden.T, inverse_T, rtol=0.0, atol=2e-15)

    def test_translation_unit_metadata_and_explicit_scale_convert_to_meters(self):
        millimeter_translation = self.T * 1000.0
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "millimeter-stereo.yaml"
            self._write_embedded_stereo(
                path,
                self.R,
                millimeter_translation,
                [("translation_unit", "mm")],
            )
            loaded = opencv_yaml.read_opencv_stereo(path)
            overridden = opencv_yaml.read_opencv_stereo(
                path, translation_scale=1e-2
            )

        np.testing.assert_allclose(loaded.T, self.T, rtol=0.0, atol=2e-15)
        np.testing.assert_allclose(
            overridden.T, millimeter_translation * 1e-2, rtol=0.0, atol=2e-14
        )

    def test_conflicting_translation_metadata_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "conflicting-units.yaml"
            self._write_embedded_stereo(
                path,
                self.R,
                self.T,
                [("translation_unit", "mm"), ("translation_scale", 1e-2)],
            )
            with self.assertRaisesRegex(ValueError, "conflicts"):
                opencv_yaml.read_opencv_stereo(path)

    def test_camchain_opencv_camchain_round_trip(self):
        chain = opencv_yaml.opencv_stereo_to_camchain(self.stereo)
        with tempfile.TemporaryDirectory() as directory:
            chain_path = Path(directory) / "input-camchain.yaml"
            opencv_yaml.write_kalibr_camchain(chain, chain_path)
            outputs = opencv_yaml.export_kalibr_camchain(
                chain_path, Path(directory) / "exported"
            )
            self.assertEqual(len(outputs), 3)
            stereo_path = Path(directory) / "exported-cam0-cam1-opencv-stereo.yaml"
            restored_path = Path(directory) / "restored-camchain.yaml"
            opencv_yaml.import_opencv_stereo(stereo_path, restored_path)
            restored = yaml.safe_load(restored_path.read_text(encoding="utf-8"))

        for index in range(2):
            key = "cam{}".format(index)
            self.assertEqual(restored[key]["camera_model"], "pinhole")
            self.assertEqual(restored[key]["distortion_model"], "opencv_fisheye")
            np.testing.assert_allclose(restored[key]["intrinsics"], chain[key]["intrinsics"])
            np.testing.assert_allclose(
                restored[key]["distortion_coeffs"], chain[key]["distortion_coeffs"]
            )
            self.assertEqual(restored[key]["resolution"], chain[key]["resolution"])
            self.assertEqual(restored[key]["rostopic"], chain[key]["rostopic"])
        np.testing.assert_allclose(
            restored["cam1"]["T_cn_cnm1"], chain["cam1"]["T_cn_cnm1"]
        )

    def test_separate_mono_intrinsics_and_stereo_extrinsics(self):
        with tempfile.TemporaryDirectory() as directory:
            left_path = Path(directory) / "left.yaml"
            right_path = Path(directory) / "right.yaml"
            extrinsics_path = Path(directory) / "extrinsics.yaml"
            opencv_yaml.write_opencv_camera(self.left, left_path)
            opencv_yaml.write_opencv_camera(self.right, right_path)
            opencv_yaml._write_values(
                extrinsics_path, [("R", self.R), ("T", self.T)]
            )
            loaded = opencv_yaml.read_opencv_stereo(
                extrinsics_path,
                left_path=left_path,
                right_path=right_path,
                topics=("/override/left", "/override/right"),
            )
        np.testing.assert_allclose(loaded.left.K, self.K0)
        np.testing.assert_allclose(loaded.right.D, self.D1)
        self.assertEqual(loaded.left.topic, "/override/left")
        self.assertEqual(loaded.right.topic, "/override/right")

    def test_nonzero_skew_is_rejected(self):
        skewed = self.K0.copy()
        skewed[0, 1] = 0.2
        with self.assertRaisesRegex(ValueError, "zero-skew"):
            opencv_yaml.OpenCvCamera(
                skewed, self.D0, (640, 480), "opencv_fisheye"
            )

    def test_nonzero_explicit_alpha_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "alpha.yaml"
            opencv_yaml._write_values(
                path,
                [
                    ("K", self.K0),
                    ("D", self.D0.reshape(4, 1)),
                    ("image_width", 640),
                    ("image_height", 480),
                    ("alpha", 0.015),
                ],
            )
            with self.assertRaisesRegex(ValueError, "requires alpha=0"):
                opencv_yaml.read_opencv_camera(path)

    def test_ros_rows_cols_data_yaml_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ros-camera.yaml"
            path.write_text(
                "%YAML:1.0\n"
                "---\n"
                "image_width: 640\n"
                "image_height: 480\n"
                "distortion_model: equidistant\n"
                "camera_matrix:\n"
                "  rows: 3\n"
                "  cols: 3\n"
                "  data: [421.3, 0., 319.2, 0., 420.7, 241.1, 0., 0., 1.]\n"
                "distortion_coefficients:\n"
                "  rows: 1\n"
                "  cols: 4\n"
                "  data: [-0.031, 0.0042, -0.00071, 0.000052]\n",
                encoding="utf-8",
            )
            camera = opencv_yaml.read_opencv_camera(path)
        np.testing.assert_allclose(camera.K, self.K0)
        np.testing.assert_allclose(camera.D, self.D0)
        self.assertEqual(camera.distortion_model, "opencv_fisheye")

    def test_plain_flat_radtan5_and_rt_yaml_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vendor-stereo.yaml"
            transform = np.eye(4)
            transform[:3, :3] = self.R
            transform[:3, 3:4] = self.T
            path.write_text(
                yaml.safe_dump(
                    {
                        "K1": self.K0.reshape(-1).tolist(),
                        "D1": [-0.42, 0.26, 0.0013, -0.00012, -0.12],
                        "K2": self.K1.reshape(-1).tolist(),
                        "D2": [-0.41, 0.23, 0.0007, -0.00001, -0.08],
                        "RT": transform.reshape(-1).tolist(),
                    },
                    sort_keys=False,
                ),
                encoding="utf-8",
            )
            stereo = opencv_yaml.read_opencv_stereo(
                path,
                resolutions=((640, 480), (648, 486)),
            )

        self.assertEqual(stereo.left.distortion_model, "radtan5")
        self.assertEqual(stereo.right.distortion_model, "radtan5")
        np.testing.assert_allclose(stereo.left.K, self.K0)
        np.testing.assert_allclose(stereo.right.K, self.K1)
        np.testing.assert_allclose(stereo.R, self.R)
        np.testing.assert_allclose(stereo.T, self.T)

    def test_native_equidistant_camchain_exports_as_cv_fisheye(self):
        chain = opencv_yaml.opencv_mono_to_camchain(self.left)
        chain["cam0"]["distortion_model"] = "equidistant"
        with tempfile.TemporaryDirectory() as directory:
            chain_path = Path(directory) / "native-equi.yaml"
            opencv_yaml.write_kalibr_camchain(chain, chain_path)
            outputs = opencv_yaml.export_kalibr_camchain(
                chain_path, Path(directory) / "native-equi"
            )
            self.assertEqual(len(outputs), 1)
            storage = cv2.FileStorage(str(outputs[0]), cv2.FILE_STORAGE_READ)
            try:
                self.assertEqual(storage.getNode("distortion_model").string(), "fisheye")
                self.assertEqual(storage.getNode("D").mat().shape, (4, 1))
                self.assertEqual(storage.getNode("alpha").real(), 0.0)
            finally:
                storage.release()

    def test_wrong_fisheye_coefficient_count_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "requires 4 coefficients"):
            opencv_yaml.OpenCvCamera(
                self.K0, np.r_[self.D0, 0.0], (640, 480), "opencv_fisheye"
            )

    def test_plumb_bob_constructor_resolves_four_and_five_coefficients(self):
        radtan = opencv_yaml.OpenCvCamera(
            self.K0, [-0.4, 0.2, 0.001, -0.002], (640, 480), "plumb_bob"
        )
        radtan5 = opencv_yaml.OpenCvCamera(
            self.K0,
            [-0.4, 0.2, 0.001, -0.002, -0.08],
            (640, 480),
            "plumb_bob",
        )
        self.assertEqual(radtan.distortion_model, "radtan")
        self.assertEqual(radtan5.distortion_model, "radtan5")


if __name__ == "__main__":
    unittest.main(verbosity=2)
