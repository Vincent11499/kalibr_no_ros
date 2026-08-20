#!/usr/bin/env python3
"""Numerical and Kalibr-integration checks for OpenCV fisheye support."""

import pickle
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml

import kalibr_camera_calibration as kcc
import kalibr_opencv_fisheye as fisheye
from kalibr_common import ConfigReader as cr


fisheye.install()

COEFFICIENTS = np.array([-0.031, 0.0042, -0.00071, 0.000052], dtype=float)
K = np.array(
    [[421.3, 0.0, 319.2], [0.0, 420.7, 241.1], [0.0, 0.0, 1.0]],
    dtype=float,
)


class OpenCvFisheyeModelTest(unittest.TestCase):
    def setUp(self):
        self.distortion = fisheye.OpenCvFisheyeDistortion(*COEFFICIENTS)

    @staticmethod
    def opencv_project(point, camera_matrix=np.eye(3)):
        object_points = np.asarray(point, dtype=np.float64).reshape(1, 1, 3)
        image_points, _ = cv2.fisheye.projectPoints(
            object_points,
            np.zeros(3),
            np.zeros(3),
            np.asarray(camera_matrix, dtype=np.float64),
            COEFFICIENTS.reshape(4, 1),
            alpha=0.0,
        )
        return image_points.reshape(2)

    def test_parameter_order_and_dimension(self):
        np.testing.assert_array_equal(
            self.distortion.getParameters().reshape(-1), COEFFICIENTS
        )
        self.assertEqual(self.distortion.minimalDimensions(), 4)

    def test_distortion_matches_cv_fisheye(self):
        for point in (
            np.array([0.2, -0.3]),
            np.array([0.9, 0.5]),
            np.array([-1.3, 0.8]),
        ):
            expected = self.opencv_project([point[0], point[1], 1.0])
            np.testing.assert_allclose(
                self.distortion.distort(point), expected, rtol=0.0, atol=5e-15
            )

    def test_axis_is_finite_and_identity(self):
        distorted, jacobian = self.distortion.distortWithInputJacobian(
            np.zeros(2)
        )
        np.testing.assert_array_equal(distorted, np.zeros(2))
        np.testing.assert_array_equal(jacobian, np.eye(2))
        np.testing.assert_array_equal(
            self.distortion.distortParameterJacobian(np.zeros(2)),
            np.zeros((2, 4)),
        )
        np.testing.assert_array_equal(self.distortion.undistort(np.zeros(2)), np.zeros(2))

    def test_input_jacobian_matches_finite_difference(self):
        point = np.array([0.63, -0.41], dtype=float)
        _, analytic = self.distortion.distortWithInputJacobian(point)
        numeric = np.empty((2, 2))
        epsilon = 1e-7
        for column in range(2):
            step = np.zeros(2)
            step[column] = epsilon
            numeric[:, column] = (
                self.distortion.distort(point + step)
                - self.distortion.distort(point - step)
            ) / (2.0 * epsilon)
        np.testing.assert_allclose(analytic, numeric, rtol=1e-8, atol=1e-9)

    def test_parameter_jacobian_matches_finite_difference(self):
        point = np.array([0.63, -0.41], dtype=float)
        analytic = self.distortion.distortParameterJacobian(point)
        numeric = np.empty((2, 4))
        epsilon = 1e-7
        for column in range(4):
            plus = COEFFICIENTS.copy()
            minus = COEFFICIENTS.copy()
            plus[column] += epsilon
            minus[column] -= epsilon
            numeric[:, column] = (
                fisheye.OpenCvFisheyeDistortion(*plus).distort(point)
                - fisheye.OpenCvFisheyeDistortion(*minus).distort(point)
            ) / (2.0 * epsilon)
        np.testing.assert_allclose(analytic, numeric, rtol=1e-8, atol=1e-9)

    def test_undistortion_inverts_distortion(self):
        for point in (np.array([0.4, -0.35]), np.array([1.1, 0.6])):
            distorted = self.distortion.distort(point)
            np.testing.assert_allclose(
                self.distortion.undistort(distorted), point, rtol=0.0, atol=2e-12
            )

    def test_large_radius_undistortion_matches_opencv(self):
        for point in (
            np.array([1.6, 0.0]),
            np.array([1.7, -0.8]),
            np.array([-2.4, 1.8]),
        ):
            expected = cv2.fisheye.undistortPoints(
                point.reshape(1, 1, 2),
                np.eye(3),
                COEFFICIENTS.reshape(4, 1),
            ).reshape(2)
            actual = self.distortion.undistort(point)
            self.assertTrue(np.all(np.isfinite(actual)))
            np.testing.assert_allclose(actual, expected, rtol=2e-13, atol=2e-12)

    def test_large_radius_undistortion_jacobian(self):
        point = np.array([1.7, -0.8], dtype=float)
        actual, analytic = self.distortion.undistortWithInputJacobian(point)
        numeric = np.empty((2, 2))
        epsilon = 1e-7
        for column in range(2):
            step = np.zeros(2)
            step[column] = epsilon
            numeric[:, column] = (
                self.distortion.undistort(point + step)
                - self.distortion.undistort(point - step)
            ) / (2.0 * epsilon)
        self.assertTrue(np.all(np.isfinite(actual)))
        self.assertTrue(np.all(np.isfinite(analytic)))
        np.testing.assert_allclose(analytic, numeric, rtol=2e-8, atol=2e-8)

    def _assert_failed_undistortion_matches_opencv(self, coefficients, point):
        coefficients = np.asarray(coefficients, dtype=np.float64)
        point = np.asarray(point, dtype=np.float64)
        expected = cv2.fisheye.undistortPoints(
            point.reshape(1, 1, 2),
            np.eye(3),
            coefficients.reshape(4, 1),
        ).reshape(2)
        distortion = fisheye.OpenCvFisheyeDistortion(*coefficients)
        actual, jacobian = distortion.undistortWithInputJacobian(point)

        # OpenCV deliberately exposes a fixed sentinel when Newton fails or
        # converges to a root on the opposite side of the optical axis.
        np.testing.assert_array_equal(expected, [-1000000.0, -1000000.0])
        np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(jacobian, np.zeros((2, 2)))

    def test_nonconvergent_undistortion_matches_opencv_failure(self):
        self._assert_failed_undistortion_matches_opencv(
            [0.00862475, 0.00525583, -0.00358072, -0.00621336],
            [0.95448856, 0.96440625],
        )

    def test_theta_flipped_undistortion_matches_opencv_failure(self):
        self._assert_failed_undistortion_matches_opencv(
            [
                0.05899982024444536,
                -0.04604087474406485,
                -0.03139836604133024,
                0.0011502896239130866,
            ],
            [2.0948835443795017, 0.0],
        )

    def test_projection_matches_cv_fisheye(self):
        projection = fisheye.OpenCvFisheyePinholeProjection(
            K[0, 0], K[1, 1], K[0, 2], K[1, 2], 640, 480, self.distortion
        )
        point = np.array([0.8, -0.4, 1.7])
        actual = projection.euclideanToKeypoint(point)
        expected = self.opencv_project(point, K)
        np.testing.assert_allclose(actual, expected, rtol=0.0, atol=2e-13)

    def test_geometry_serialization_and_design_variables(self):
        projection = fisheye.OpenCvFisheyePinholeProjection(
            K[0, 0], K[1, 1], K[0, 2], K[1, 2], 640, 480, self.distortion
        )
        geometry = fisheye.OpenCvFisheyePinholeCameraGeometry(projection)
        restored = pickle.loads(pickle.dumps(geometry))
        np.testing.assert_array_equal(
            restored.projection().distortion().getParameters().reshape(-1),
            COEFFICIENTS,
        )
        design_variable = fisheye.PinholeOpenCvFisheye.designVariable(geometry)
        self.assertIsNotNone(design_variable.projectionDesignVariable())
        self.assertIsNotNone(design_variable.distortionDesignVariable())
        self.assertIsNotNone(design_variable.shutterDesignVariable())


class OpenCvFisheyeIntegrationTest(unittest.TestCase):
    def test_camchain_model_constructs_geometry(self):
        parameters = cr.CameraParameters("unused.yaml", createYaml=True)
        parameters.setRosTopic("/cam0/image_raw")
        parameters.setIntrinsics("pinhole", [421.3, 420.7, 319.2, 241.1])
        parameters.setDistortion("opencv_fisheye", COEFFICIENTS)
        parameters.setResolution([640, 480])
        camera = cr.AslamCamera.fromParameters(parameters)
        np.testing.assert_array_equal(
            camera.geometry.projection().distortion().getParameters().reshape(-1),
            COEFFICIENTS,
        )

    def test_wrong_coefficient_count_is_rejected(self):
        parameters = cr.CameraParameters("unused.yaml", createYaml=True)
        with self.assertRaisesRegex(RuntimeError, "requires 4 coefficients"):
            parameters.setDistortion("opencv_fisheye", COEFFICIENTS[:3])

    def test_calibrator_export_preserves_nontrivial_transform(self):
        cameras = []
        for index in range(2):
            distortion = fisheye.OpenCvFisheyeDistortion(*COEFFICIENTS)
            projection = fisheye.OpenCvFisheyePinholeProjection(
                K[0, 0] + index,
                K[1, 1] + index,
                K[0, 2] + index,
                K[1, 2] + index,
                640,
                480,
                distortion,
            )
            cameras.append(
                SimpleNamespace(
                    model=fisheye.PinholeOpenCvFisheye,
                    geometry=fisheye.OpenCvFisheyePinholeCameraGeometry(projection),
                    dataset=SimpleNamespace(topic="/cam{}/image_raw".format(index)),
                )
            )
        transform = np.eye(4)
        transform[:3, :3] = cv2.Rodrigues(np.array([0.12, -0.08, 0.045]))[0]
        transform[:3, 3] = [-0.118, 0.0043, 0.0071]
        calibrator = SimpleNamespace(
            cameras=cameras, baselines=[SimpleNamespace(T=lambda: transform)]
        )

        class Graph:
            @staticmethod
            def getCamOverlaps(camera_id):
                return [1 - camera_id]

        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory) / "unit-camchain.yaml"
            kcc.saveChainParametersYaml(calibrator, str(result), Graph())
            chain = yaml.safe_load(result.read_text(encoding="utf-8"))
            self.assertEqual(chain["cam0"]["distortion_model"], "opencv_fisheye")
            stereo = Path(directory) / "unit-cam0-cam1-opencv-stereo.yaml"
            storage = cv2.FileStorage(str(stereo), cv2.FILE_STORAGE_READ)
            try:
                np.testing.assert_allclose(storage.getNode("R").mat(), transform[:3, :3])
                np.testing.assert_allclose(
                    storage.getNode("T").mat().reshape(3), transform[:3, 3]
                )
                self.assertEqual(storage.getNode("D1").mat().shape, (4, 1))
            finally:
                storage.release()


if __name__ == "__main__":
    unittest.main(verbosity=2)
