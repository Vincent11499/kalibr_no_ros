#!/usr/bin/env python3

import unittest

import numpy as np

import kalibr_opencv_fisheye as zero_fisheye
import kalibr_opencv_fisheye_full as full_fisheye
from kalibr_common import ConfigReader


class FullOpenCvFisheyeNativeIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        zero_fisheye.install()
        full_fisheye.install()

    def test_projection_and_design_variable_keep_fifth_intrinsic(self):
        distortion = zero_fisheye.OpenCvFisheyeDistortion(
            -0.02, 0.003, -0.0004, 0.00003
        )
        projection = full_fisheye.OpenCvFisheyeProjection(
            400.0, 405.0, 320.0, 240.0, 0.025, 640, 480, distortion
        )
        geometry = full_fisheye.OpenCvFisheyeCameraGeometry(projection)
        design_variable = (
            full_fisheye.OpenCvFisheyeCameraGeometryDesignVariable(geometry)
        )

        self.assertEqual(projection.getParameters().shape, (5, 1))
        self.assertEqual(projection.distortion().getParameters().shape, (4, 1))
        self.assertAlmostEqual(projection.getCameraMatrix()[0, 1], 10.0)
        self.assertIsNotNone(design_variable)

    def test_config_reader_builds_full_model_without_losing_alpha(self):
        camera = ConfigReader.AslamCamera(
            "pinhole_opencv_fisheye",
            [400.0, 405.0, 320.0, 240.0, -0.017],
            "opencv_fisheye",
            [-0.02, 0.003, -0.0004, 0.00003],
            [640, 480],
        )

        projection = camera.geometry.projection()
        self.assertIsInstance(
            projection, full_fisheye.OpenCvFisheyeProjection
        )
        self.assertAlmostEqual(projection.alpha(), -0.017)
        np.testing.assert_allclose(
            projection.distortion().getParameters().reshape(-1),
            [-0.02, 0.003, -0.0004, 0.00003],
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
