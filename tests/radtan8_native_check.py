#!/usr/bin/env python3
"""Numerical and integration checks for the native radtan8 extension."""

import pickle
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml

import kalibr_camera_calibration as kcc
import kalibr_radtan5 as radtan5
import kalibr_radtan8 as radtan8
from kalibr_common import ConfigReader as cr


radtan5.install()
radtan8.install()


COEFFICIENTS = np.array(
    [-0.18, 0.052, 0.0012, -0.0007, -0.008, 0.014, -0.003, 0.0004],
    dtype=float,
)


class Radtan8ModelTest(unittest.TestCase):
    def setUp(self):
        self.distortion = radtan8.RadialTangentialDistortion8(*COEFFICIENTS)

    @staticmethod
    def opencv_distort(point):
        object_point = np.array([[point[0], point[1], 1.0]], dtype=np.float64)
        return cv2.projectPoints(
            object_point,
            np.zeros(3),
            np.zeros(3),
            np.eye(3),
            COEFFICIENTS,
        )[0].reshape(2)

    def test_parameter_order_and_dimension(self):
        np.testing.assert_array_equal(
            self.distortion.getParameters().reshape(-1), COEFFICIENTS
        )
        self.assertEqual(self.distortion.minimalDimensions(), 8)

    def test_distortion_matches_opencv(self):
        for point in (
            np.array([0.2, -0.3]),
            np.array([0.55, 0.4]),
            np.array([-0.72, 0.61]),
        ):
            np.testing.assert_allclose(
                self.distortion.distort(point),
                self.opencv_distort(point),
                rtol=0.0,
                atol=2e-15,
            )

    def test_input_jacobian_matches_finite_difference(self):
        point = np.array([0.31, -0.27], dtype=float)
        _, analytic = self.distortion.distortWithInputJacobian(point)
        numeric = np.empty((2, 2), dtype=float)
        epsilon = 1e-7
        for column in range(2):
            delta = np.zeros(2)
            delta[column] = epsilon
            numeric[:, column] = (
                self.distortion.distort(point + delta)
                - self.distortion.distort(point - delta)
            ) / (2.0 * epsilon)
        np.testing.assert_allclose(analytic, numeric, rtol=1e-8, atol=1e-9)

    def test_parameter_jacobian_matches_finite_difference(self):
        point = np.array([0.31, -0.27], dtype=float)
        analytic = self.distortion.distortParameterJacobian(point)
        numeric = np.empty((2, 8), dtype=float)
        epsilon = 1e-7
        for column in range(8):
            plus = COEFFICIENTS.copy()
            minus = COEFFICIENTS.copy()
            plus[column] += epsilon
            minus[column] -= epsilon
            numeric[:, column] = (
                radtan8.RadialTangentialDistortion8(*plus).distort(point)
                - radtan8.RadialTangentialDistortion8(*minus).distort(point)
            ) / (2.0 * epsilon)
        np.testing.assert_allclose(analytic, numeric, rtol=2e-8, atol=1e-9)

    def test_undistortion_inverts_distortion(self):
        for point in (
            np.array([0.4, -0.35], dtype=float),
            np.array([-0.75, 0.55], dtype=float),
        ):
            distorted = self.distortion.distort(point)
            np.testing.assert_allclose(
                self.distortion.undistort(distorted), point, rtol=0.0, atol=2e-12
            )

    def test_projection_and_projection_jacobian(self):
        projection = radtan8.Radtan8PinholeProjection(
            458.2, 457.8, 367.1, 248.3, 752, 480, self.distortion
        )
        point = np.array([0.4, -0.3, 1.7], dtype=float)
        actual, analytic, valid = projection.euclideanToKeypointJp(point)
        matrix = np.array(
            [[458.2, 0.0, 367.1], [0.0, 457.8, 248.3], [0.0, 0.0, 1.0]]
        )
        expected = cv2.projectPoints(
            point.reshape(1, 3),
            np.zeros(3),
            np.zeros(3),
            matrix,
            COEFFICIENTS,
        )[0].reshape(2)
        self.assertTrue(valid)
        np.testing.assert_allclose(actual, expected, rtol=0.0, atol=2e-13)

        numeric = np.empty((2, 3), dtype=float)
        epsilon = 1e-7
        for column in range(3):
            delta = np.zeros(3)
            delta[column] = epsilon
            numeric[:, column] = (
                projection.euclideanToKeypoint(point + delta)
                - projection.euclideanToKeypoint(point - delta)
            ) / (2.0 * epsilon)
        np.testing.assert_allclose(analytic, numeric, rtol=2e-8, atol=2e-7)

    def test_geometry_serialization_and_design_variables(self):
        projection = radtan8.Radtan8PinholeProjection(
            458.2, 457.8, 367.1, 248.3, 752, 480, self.distortion
        )
        geometry = radtan8.Radtan8PinholeCameraGeometry(projection)
        restored = pickle.loads(pickle.dumps(geometry))
        np.testing.assert_array_equal(
            restored.projection().distortion().getParameters().reshape(-1),
            COEFFICIENTS,
        )

        design_variable = radtan8.PinholeRadtan8.designVariable(geometry)
        self.assertIsNotNone(design_variable.projectionDesignVariable())
        self.assertIsNotNone(design_variable.distortionDesignVariable())
        self.assertIsNotNone(design_variable.shutterDesignVariable())


class Radtan8ConfigTest(unittest.TestCase):
    def test_camchain_model_constructs_native_geometry(self):
        parameters = cr.CameraParameters("unused.yaml", createYaml=True)
        parameters.setRosTopic("/cam0/image_raw")
        parameters.setIntrinsics("pinhole", [458.2, 457.8, 367.1, 248.3])
        parameters.setDistortion("radtan8", COEFFICIENTS)
        parameters.setResolution([752, 480])
        camera = cr.AslamCamera.fromParameters(parameters)
        self.assertEqual(
            camera.geometry.projection().distortion().minimalDimensions(), 8
        )
        np.testing.assert_array_equal(
            camera.geometry.projection().distortion().getParameters().reshape(-1),
            COEFFICIENTS,
        )

    def test_radtan8_alias_and_wrong_count(self):
        parameters = cr.CameraParameters("unused.yaml", createYaml=True)
        parameters.setDistortion("rational_polynomial", COEFFICIENTS)
        with self.assertRaisesRegex(RuntimeError, "requires 8 coefficients"):
            parameters.setDistortion("radtan8", COEFFICIENTS[:5])

    def test_camchain_and_opencv_stereo_exports(self):
        cameras = []
        for index in range(2):
            distortion = radtan8.RadialTangentialDistortion8(*COEFFICIENTS)
            projection = radtan8.Radtan8PinholeProjection(
                458.2 + index,
                457.8 + index,
                367.1 + index,
                248.3 + index,
                752,
                480,
                distortion,
            )
            cameras.append(
                SimpleNamespace(
                    model=radtan8.PinholeRadtan8,
                    geometry=radtan8.Radtan8PinholeCameraGeometry(projection),
                    dataset=SimpleNamespace(topic="/cam{}/image_raw".format(index)),
                )
            )

        transform = np.eye(4)
        transform[0, 3] = -0.11
        calibrator = SimpleNamespace(
            cameras=cameras,
            baselines=[SimpleNamespace(T=lambda: transform)],
        )

        class Graph:
            @staticmethod
            def getCamOverlaps(camera_id):
                return [1 - camera_id]

        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "unit-camchain.yaml"
            kcc.saveChainParametersYaml(calibrator, str(result), Graph())
            chain = yaml.safe_load(result.read_text(encoding="utf-8"))
            self.assertEqual(chain["cam0"]["distortion_model"], "radtan8")
            self.assertEqual(len(chain["cam0"]["distortion_coeffs"]), 8)

            stereo = Path(directory) / "unit-cam0-cam1-opencv-stereo.yaml"
            self.assertTrue(stereo.is_file())
            storage = cv2.FileStorage(str(stereo), cv2.FILE_STORAGE_READ)
            try:
                self.assertEqual(
                    storage.getNode("distortion_model").string(),
                    "rational_polynomial",
                )
                self.assertEqual(storage.getNode("D1").mat().shape, (1, 8))
                self.assertEqual(storage.getNode("D2").mat().shape, (1, 8))
                np.testing.assert_array_equal(
                    storage.getNode("R").mat(), transform[:3, :3]
                )
                np.testing.assert_array_equal(
                    storage.getNode("T").mat().reshape(3), transform[:3, 3]
                )
            finally:
                storage.release()


if __name__ == "__main__":
    unittest.main(verbosity=2)
