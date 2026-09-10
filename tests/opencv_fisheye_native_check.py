#!/usr/bin/env python3

import pickle
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml

import aslam_cv_backend as cvb
import kalibr_camera_calibration as kcc
import kalibr_opencv_fisheye as fisheye
from kalibr_common import ConfigReader


class OpenCvFisheyeNativeIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fisheye.install()

    def test_projection_and_design_variable_keep_fifth_intrinsic(self):
        distortion = fisheye.OpenCvFisheyeDistortion(
            -0.02, 0.003, -0.0004, 0.00003
        )
        projection = fisheye.OpenCvFisheyeProjection(
            400.0, 405.0, 320.0, 240.0, 0.025, 640, 480, distortion
        )
        geometry = fisheye.OpenCvFisheyeCameraGeometry(projection)
        design_variable = (
            fisheye.OpenCvFisheyeCameraGeometryDesignVariable(geometry)
        )

        self.assertEqual(projection.getParameters().shape, (5, 1))
        self.assertEqual(projection.distortion().getParameters().shape, (4, 1))
        self.assertAlmostEqual(projection.getCameraMatrix()[0, 1], 10.0)
        self.assertIsNotNone(design_variable)
        self.assertIsNotNone(design_variable.projectionDesignVariable())
        self.assertIsNotNone(design_variable.distortionDesignVariable())
        restored = pickle.loads(pickle.dumps(geometry))
        np.testing.assert_array_equal(
            restored.projection().getParameters(), projection.getParameters()
        )
        np.testing.assert_array_equal(
            restored.projection().distortion().getParameters(),
            distortion.getParameters(),
        )

    def test_config_reader_builds_model_without_losing_alpha(self):
        camera = ConfigReader.AslamCamera(
            "pinhole_opencv_fisheye",
            [400.0, 405.0, 320.0, 240.0, -0.017],
            "opencv_fisheye",
            [-0.02, 0.003, -0.0004, 0.00003],
            [640, 480],
        )

        projection = camera.geometry.projection()
        self.assertIsInstance(
            projection, fisheye.OpenCvFisheyeProjection
        )
        self.assertAlmostEqual(projection.alpha(), -0.017)
        np.testing.assert_allclose(
            projection.distortion().getParameters().reshape(-1),
            [-0.02, 0.003, -0.0004, 0.00003],
        )

    def test_wrong_distortion_dimension_is_rejected(self):
        parameters = ConfigReader.CameraParameters("unused.yaml", createYaml=True)
        with self.assertRaisesRegex(RuntimeError, "requires 4 coefficients"):
            parameters.setDistortion("opencv_fisheye", [0.0, 0.0, 0.0])

    def test_native_equi_and_nine_parameter_chain_preserves_parameters_and_transform(self):
        coefficients = [-0.02, 0.003, -0.0004, 0.00003]
        native = ConfigReader.AslamCamera(
            "pinhole", [400.0, 405.0, 320.0, 240.0],
            "equidistant", coefficients, [640, 480],
        )
        extended = ConfigReader.AslamCamera(
            "pinhole_opencv_fisheye", [401.0, 406.0, 321.0, 241.0, 0.025],
            "opencv_fisheye", coefficients, [640, 480],
        )
        cameras = [
            SimpleNamespace(model=model, geometry=camera.geometry,
                            dataset=SimpleNamespace(topic="cam{}".format(index)))
            for index, (model, camera) in enumerate([
                (cvb.EquidistantPinhole, native),
                (fisheye.PinholeOpenCvFisheye, extended),
            ])
        ]
        transform = np.eye(4)
        transform[:3, :3] = cv2.Rodrigues(np.array([0.12, -0.08, 0.045]))[0]
        transform[:3, 3] = [-0.118, 0.0043, 0.0071]
        calibrator = SimpleNamespace(
            cameras=cameras, baselines=[SimpleNamespace(T=lambda: transform)]
        )
        graph = SimpleNamespace(getCamOverlaps=lambda index: [1 - index])
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "unit-camchain.yaml"
            kcc.saveChainParametersYaml(calibrator, str(result), graph)
            chain = yaml.safe_load(result.read_text(encoding="utf-8"))
            self.assertEqual(chain["cam0"]["distortion_model"], "equidistant")
            self.assertEqual(chain["cam1"]["camera_model"], "pinhole_opencv_fisheye")
            self.assertEqual(chain["cam1"]["intrinsics"],
                             [401.0, 406.0, 321.0, 241.0, 0.025])
            np.testing.assert_allclose(chain["cam1"]["T_cn_cnm1"], transform)
            storage = cv2.FileStorage(
                str(Path(directory) / "unit-cam0-cam1-opencv-stereo.yaml"),
                cv2.FILE_STORAGE_READ,
            )
            try:
                self.assertTrue(storage.isOpened())
                np.testing.assert_allclose(storage.getNode("R").mat(), transform[:3, :3])
                np.testing.assert_allclose(storage.getNode("T").mat().reshape(3),
                                           transform[:3, 3])
            finally:
                storage.release()


if __name__ == "__main__":
    unittest.main(verbosity=2)
