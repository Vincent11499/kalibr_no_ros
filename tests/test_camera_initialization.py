import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import yaml

try:
    import aslam_cv as acv
    import aslam_cv_backend as acvb
    import igraph
    import sm
    import kalibr_camera_calibration.CameraCalibrator as camera_calibrator
    import kalibr_camera_calibration.MulticamGraph as multicam_graph
    from kalibr_camera_calibration.CameraCalibrator import CameraGeometry
except ImportError:
    acv = None
    acvb = None
    igraph = None
    sm = None
    camera_calibrator = None
    multicam_graph = None
    CameraGeometry = None


@unittest.skipIf(
    CameraGeometry is None,
    "the native build-tree Python packages are not on PYTHONPATH",
)
class CameraInitializationTest(unittest.TestCase):
    def write_initialization(self, document):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "initialization.yaml"
        path.write_text(yaml.safe_dump(document), encoding="utf-8")
        return path

    def test_partial_focal_initialization_ratio_reaches_native_geometry(self):
        camera = CameraGeometry.__new__(CameraGeometry)
        camera.geometry = mock.Mock()
        camera.geometry.initializeIntrinsics.return_value = True
        camera.model = object()
        camera.dataset = type("Dataset", (), {"topic": "/cam0"})()

        with mock.patch.object(
            camera_calibrator.kcc, "calibrateIntrinsics", return_value=True
        ):
            success = camera.initGeometryFromObservations(
                ["observation"], 0.75)

        self.assertTrue(success)
        camera.geometry.initializeIntrinsics.assert_called_once_with(
            ["observation"], 0.75)

    def test_partial_focal_ratio_reaches_distortion_only_seed_path(self):
        camera = CameraGeometry.__new__(CameraGeometry)
        camera.geometry = mock.Mock()
        camera.geometry.initializeIntrinsics.return_value = True
        camera.model = object()
        camera.dataset = type("Dataset", (), {"topic": "/cam0"})()
        seed = {"distortion_coeffs": (0.1, -0.02, 0.003, -0.004)}

        with mock.patch.object(
            camera_calibrator.kcc, "calibrateIntrinsics", return_value=True
        ), mock.patch.object(
            camera, "_restoreUnseededInitializationDefaults"
        ), mock.patch.object(camera, "_applyInitializationSeed"):
            success = camera.initGeometryFromObservationsWithSeed(
                ["observation"], "cam0", seed, 0.75
            )

        self.assertTrue(success)
        camera.geometry.initializeIntrinsics.assert_called_once_with(
            ["observation"], 0.75
        )

    @staticmethod
    def _partial_circle_grid_observation():
        options = acv.CheckerboardOptions()
        target = acv.GridCalibrationTargetCheckerboard(
            8, 8, 0.04, 0.04, options
        )
        observation = acv.GridCalibrationTargetObservation(target)
        observation.setImage(np.zeros((1200, 2000), dtype=np.uint8))
        first_vanishing_point = np.asarray([250.0, 300.0])
        second_vanishing_point = np.asarray([1500.0, 300.0])
        center_x = 0.5 * (
            first_vanishing_point[0] + second_vanishing_point[0]
        )
        for row in range(8):
            center = np.asarray([center_x, -700.0 + 200.0 * row])
            first_angle = math.atan2(
                first_vanishing_point[1] - center[1],
                first_vanishing_point[0] - center[0],
            )
            second_angle = math.atan2(
                second_vanishing_point[1] - center[1],
                second_vanishing_point[0] - center[0],
            )
            angle_difference = second_angle - first_angle
            while angle_difference > math.pi:
                angle_difference -= 2.0 * math.pi
            while angle_difference < -math.pi:
                angle_difference += 2.0 * math.pi
            radius = np.linalg.norm(first_vanishing_point - center)
            for column in range(1, 7):
                fraction = column / 7.0
                angle = first_angle + fraction * angle_difference
                observation.updateImagePoint(
                    target.gridCoordinatesToPoint(row, column),
                    center + radius * np.asarray(
                        [math.cos(angle), math.sin(angle)]
                    ),
                )
        return observation

    def test_partial_focal_initialization_covers_public_extended_models(self):
        import kalibr_opencv_fisheye_full as opencv_fisheye
        import kalibr_radtan8 as radtan8

        observation = self._partial_circle_grid_observation()
        expected_focal_length = 1250.0 / math.pi
        for name, model in (
            ("pinhole-radtan8", radtan8.PinholeRadtan8),
            ("pinhole-opencv-fisheye",
             opencv_fisheye.PinholeOpenCvFisheyeFull),
        ):
            with self.subTest(model=name):
                geometry = model.geometry()
                self.assertTrue(
                    geometry.initializeIntrinsics([observation], 0.75)
                )
                parameters = geometry.projection().getParameters().flatten()
                self.assertAlmostEqual(
                    parameters[0], expected_focal_length, places=7
                )
                self.assertAlmostEqual(
                    parameters[1], expected_focal_length, places=7
                )

    def test_partial_focal_initialization_rejects_below_threshold_view(self):
        import kalibr_radtan8 as radtan8

        geometry = radtan8.PinholeRadtan8.geometry()
        with mock.patch.dict("os.environ", {}, clear=True):
            self.assertFalse(
                geometry.initializeIntrinsics(
                    [self._partial_circle_grid_observation()], 0.76
                )
            )

    def test_partial_focal_initialization_rejects_degenerate_rows(self):
        import kalibr_radtan8 as radtan8

        options = acv.CheckerboardOptions()
        target = acv.GridCalibrationTargetCheckerboard(
            8, 8, 0.04, 0.04, options
        )
        observation = acv.GridCalibrationTargetObservation(target)
        observation.setImage(np.zeros((1200, 2000), dtype=np.uint8))
        for row in range(8):
            for column in range(8):
                observation.updateImagePoint(
                    target.gridCoordinatesToPoint(row, column),
                    np.asarray([200.0 + 100.0 * column, 100.0 + 80.0 * row]),
                )

        geometry = radtan8.PinholeRadtan8.geometry()
        with mock.patch.dict("os.environ", {}, clear=True):
            self.assertFalse(
                geometry.initializeIntrinsics([observation], 0.75)
            )

    @staticmethod
    def base_document(strategy="refine"):
        return {
            "schema_version": 1,
            "kind": "camera_calibration_initialization",
            "strategy": strategy,
            "cameras": {},
        }

    def test_refine_accepts_partial_camera_blocks(self):
        document = self.base_document()
        document["cameras"] = {
            "cam0": {"intrinsics": [400.0, 401.0, 320.0, 240.0]},
            "cam1": {
                "distortion_coeffs": [0.1, -0.01, 0.0, 0.0],
                "T_cam_from_previous": np.eye(4).tolist(),
            },
        }

        loaded = camera_calibrator.loadCameraCalibrationInitialization(
            str(self.write_initialization(document)), 2)

        self.assertEqual(loaded["strategy"], "refine")
        self.assertEqual(
            loaded["cameras"]["cam0"]["intrinsics"],
            (400.0, 401.0, 320.0, 240.0),
        )
        self.assertNotIn(
            "distortion_coeffs", loaded["cameras"]["cam0"])

    def test_direct_rejects_an_incomplete_seed(self):
        document = self.base_document("direct")
        document["cameras"] = {
            "cam0": {
                "intrinsics": [400.0, 400.0, 320.0, 240.0],
                "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
            },
            "cam1": {
                "intrinsics": [400.0, 400.0, 320.0, 240.0],
                "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
            },
        }

        with self.assertRaisesRegex(
            RuntimeError, r"cam1\.T_cam_from_previous"
        ):
            camera_calibrator.loadCameraCalibrationInitialization(
                str(self.write_initialization(document)), 2)

    def test_loader_rejects_unknown_fields_and_non_4x4_transforms(self):
        unknown = self.base_document()
        unknown["extra"] = True
        with self.assertRaisesRegex(RuntimeError, "unknown top-level"):
            camera_calibrator.loadCameraCalibrationInitialization(
                str(self.write_initialization(unknown)), 1)

        bad_transform = self.base_document()
        bad_transform["cameras"] = {
            "cam1": {"T_cam_from_previous": np.eye(3).tolist()}
        }
        with self.assertRaisesRegex(RuntimeError, "full 4x4"):
            camera_calibrator.loadCameraCalibrationInitialization(
                str(self.write_initialization(bad_transform)), 2)

    def test_loader_checks_vectors_against_selected_native_models(self):
        document = self.base_document()
        document["cameras"] = {
            "cam0": {"intrinsics": [400.0, 401.0, 320.0]}
        }
        with self.assertRaisesRegex(RuntimeError, "requires 4"):
            camera_calibrator.loadCameraCalibrationInitialization(
                str(self.write_initialization(document)),
                1,
                [acvb.DistortedPinhole],
            )

    def test_loader_rejects_boolean_and_string_parameter_values(self):
        for invalid_value in (True, "400.0"):
            document = self.base_document()
            document["cameras"] = {
                "cam0": {
                    "intrinsics": [invalid_value, 401.0, 320.0, 240.0]
                }
            }
            with self.assertRaisesRegex(RuntimeError, "only numbers"):
                camera_calibrator.loadCameraCalibrationInitialization(
                    str(self.write_initialization(document)), 1
                )

    def test_seed_parameter_lengths_are_checked_against_native_model(self):
        camera = CameraGeometry.__new__(CameraGeometry)
        camera.geometry = acvb.DistortedPinhole.geometry()

        with self.assertRaisesRegex(RuntimeError, "requires 4"):
            camera._applyInitializationSeed(
                "cam0", {"intrinsics": (400.0, 401.0, 320.0)}
            )

        camera._applyInitializationSeed(
            "cam0",
            {
                "intrinsics": (400.0, 401.0, 320.0, 240.0),
                "distortion_coeffs": (0.1, -0.02, 0.003, -0.004),
            },
        )
        np.testing.assert_allclose(
            camera.geometry.projection().getParameters().flatten(),
            [400.0, 401.0, 320.0, 240.0],
        )
        np.testing.assert_allclose(
            camera.geometry.projection().distortion().getParameters().flatten(),
            [0.1, -0.02, 0.003, -0.004],
        )

    def test_seed_geometry_is_constructed_with_observation_resolution(self):
        options = acv.CheckerboardOptions()
        target = acv.GridCalibrationTargetCheckerboard(
            3, 3, 0.04, 0.04, options
        )
        observation = acv.GridCalibrationTargetObservation(target)
        observation.setImage(np.zeros((480, 640), dtype=np.uint8))
        seed = {
            "intrinsics": (400.0, 401.0, 320.0, 240.0),
            "distortion_coeffs": (0.1, -0.02, 0.003, -0.004),
        }

        geometry = camera_calibrator.cameraGeometryFromInitialization(
            acvb.DistortedPinhole, "cam0", seed, [observation]
        )

        self.assertEqual(geometry.projection().ru(), 640)
        self.assertEqual(geometry.projection().rv(), 480)
        np.testing.assert_allclose(
            geometry.projection().getParameters().flatten(),
            seed["intrinsics"],
        )
        np.testing.assert_allclose(
            geometry.projection().distortion().getParameters().flatten(),
            seed["distortion_coeffs"],
        )

    def test_seed_geometry_construction_covers_every_public_camera_model(self):
        import kalibr_opencv_fisheye_full as opencv_fisheye
        import kalibr_radtan5 as radtan5
        import kalibr_radtan8 as radtan8

        options = acv.CheckerboardOptions()
        target = acv.GridCalibrationTargetCheckerboard(
            3, 3, 0.04, 0.04, options
        )
        observation = acv.GridCalibrationTargetObservation(target)
        observation.setImage(np.zeros((480, 640), dtype=np.uint8))

        cases = (
            ("pinhole-radtan", acvb.DistortedPinhole,
             [400.0, 401.0, 320.0, 240.0], [0.1, -0.02, 0.003, -0.004]),
            ("pinhole-radtan5", radtan5.PinholeRadtan5,
             [400.0, 401.0, 320.0, 240.0],
             [0.1, -0.02, 0.003, -0.004, 0.001]),
            ("pinhole-radtan8", radtan8.PinholeRadtan8,
             [400.0, 401.0, 320.0, 240.0],
             [0.1, -0.02, 0.003, -0.004, 0.001, 0.01, -0.002, 0.0003]),
            ("pinhole-equi", acvb.EquidistantPinhole,
             [400.0, 401.0, 320.0, 240.0], [0.1, -0.02, 0.003, -0.004]),
            ("pinhole-fov", acvb.FovPinhole,
             [400.0, 401.0, 320.0, 240.0], [0.8]),
            ("pinhole-opencv-fisheye",
             opencv_fisheye.PinholeOpenCvFisheyeFull,
             [400.0, 401.0, 320.0, 240.0, 0.0],
             [0.1, -0.02, 0.003, -0.004]),
            ("omni-none", acvb.Omni,
             [1.0, 400.0, 401.0, 320.0, 240.0], []),
            ("omni-radtan", acvb.DistortedOmni,
             [1.0, 400.0, 401.0, 320.0, 240.0],
             [0.1, -0.02, 0.003, -0.004]),
            ("eucm-none", acvb.ExtendedUnified,
             [0.5, 1.0, 400.0, 401.0, 320.0, 240.0], []),
            ("ds-none", acvb.DoubleSphere,
             [0.5, 0.5, 400.0, 401.0, 320.0, 240.0], []),
        )

        for name, model, intrinsics, distortion in cases:
            with self.subTest(model=name):
                geometry = camera_calibrator.cameraGeometryFromInitialization(
                    model,
                    "cam0",
                    {
                        "intrinsics": tuple(intrinsics),
                        "distortion_coeffs": tuple(distortion),
                    },
                    [observation],
                )
                self.assertEqual(geometry.projection().ru(), 640)
                self.assertEqual(geometry.projection().rv(), 480)
                np.testing.assert_allclose(
                    geometry.projection().getParameters().flatten(),
                    intrinsics,
                )
                np.testing.assert_allclose(
                    geometry.projection().distortion().getParameters().flatten(),
                    distortion,
                )

    def test_resolution_bootstrap_cannot_overwrite_supplied_intrinsics(self):
        camera = CameraGeometry.__new__(CameraGeometry)
        camera.geometry = acvb.DistortedPinhole.geometry()
        camera.model = acvb.DistortedPinhole
        camera.dataset = type("Dataset", (), {"topic": "/cam0"})()

        options = acv.CheckerboardOptions()
        target = acv.GridCalibrationTargetCheckerboard(
            3, 3, 0.04, 0.04, options
        )
        observation = acv.GridCalibrationTargetObservation(target)
        observation.setImage(np.zeros((480, 640), dtype=np.uint8))
        with mock.patch.object(
            camera_calibrator.kcc, "calibrateIntrinsics", return_value=True
        ) as refine:
            success = camera.initGeometryFromObservationsWithSeed(
                [observation],
                "cam0",
                {"intrinsics": (400.0, 401.0, 320.0, 240.0)},
            )

        self.assertTrue(success)
        refine.assert_called_once_with(camera, [observation])
        self.assertEqual(camera.geometry.projection().ru(), 640)
        self.assertEqual(camera.geometry.projection().rv(), 480)
        np.testing.assert_allclose(
            camera.geometry.projection().getParameters().flatten(),
            [400.0, 401.0, 320.0, 240.0],
        )
        self.assertTrue(camera.isGeometryInitialized)

    def test_direct_seed_uses_metadata_bootstrap_without_intrinsic_lm(self):
        camera = CameraGeometry.__new__(CameraGeometry)
        camera.geometry = acvb.DistortedPinhole.geometry()

        options = acv.CheckerboardOptions()
        target = acv.GridCalibrationTargetCheckerboard(
            3, 3, 0.04, 0.04, options
        )
        observation = acv.GridCalibrationTargetObservation(target)
        observation.setImage(np.zeros((480, 640), dtype=np.uint8))
        seed = {
            "intrinsics": (410.0, 411.0, 321.0, 241.0),
            "distortion_coeffs": (0.1, -0.02, 0.003, -0.004),
        }

        with mock.patch.object(
            camera_calibrator.kcc,
            "calibrateIntrinsics",
            side_effect=AssertionError("direct mode ran intrinsic LM"),
        ):
            success = camera.initGeometryFromSeed(
                [observation], "cam0", seed
            )

        self.assertTrue(success)
        self.assertEqual(camera.geometry.projection().ru(), 640)
        self.assertEqual(camera.geometry.projection().rv(), 480)
        np.testing.assert_allclose(
            camera.geometry.projection().getParameters().flatten(),
            seed["intrinsics"],
        )
        np.testing.assert_allclose(
            camera.geometry.projection().distortion().getParameters().flatten(),
            seed["distortion_coeffs"],
        )

    def test_refine_restart_restores_explicit_and_implicit_seed_values(self):
        options = acv.CheckerboardOptions()
        target = acv.GridCalibrationTargetCheckerboard(
            3, 3, 0.04, 0.04, options
        )
        observation = acv.GridCalibrationTargetObservation(target)
        observation.setImage(np.zeros((480, 640), dtype=np.uint8))
        seed = {"intrinsics": (400.0, 401.0, 320.0, 240.0)}

        camera = CameraGeometry.__new__(CameraGeometry)
        camera.geometry = camera_calibrator.cameraGeometryFromInitialization(
            acvb.DistortedPinhole, "cam0", seed, [observation]
        )
        camera.model = acvb.DistortedPinhole
        camera.dataset = type("Dataset", (), {"topic": "/cam0"})()
        starts = []

        def mutate_after_recording(owner, observations):
            starts.append((
                owner.geometry.projection().getParameters().flatten().copy(),
                owner.geometry.projection().distortion().getParameters()
                .flatten().copy(),
            ))
            owner.geometry.projection().setParameters(
                np.asarray([900.0, 901.0, 902.0, 903.0])
            )
            owner.geometry.projection().distortion().setParameters(
                np.asarray([9.0, 9.0, 9.0, 9.0])
            )
            return True

        with mock.patch.object(
            camera_calibrator.kcc,
            "calibrateIntrinsics",
            side_effect=mutate_after_recording,
        ):
            camera.initGeometryFromObservationsWithSeed(
                [observation], "cam0", seed
            )
            camera.initGeometryFromObservationsWithSeed(
                [observation], "cam0", seed
            )

        for intrinsics, distortion in starts:
            np.testing.assert_allclose(intrinsics, seed["intrinsics"])
            np.testing.assert_allclose(distortion, np.zeros(4))

    def test_injected_geometry_is_used_by_design_variable_and_detector(self):
        class Geometry:
            pass

        injected = Geometry()
        design_variable = object()

        class Model:
            geometry = Geometry

            @staticmethod
            def designVariable(geometry):
                self.assertIs(geometry, injected)
                return design_variable

        dataset = type("Dataset", (), {})()
        target_config = object()
        with mock.patch.object(
            camera_calibrator, "TargetDetector", return_value="detector"
        ), mock.patch.object(CameraGeometry, "setDvActiveStatus"):
            camera = CameraGeometry(
                Model, target_config, dataset, geometry=injected
            )

        self.assertIs(camera.geometry, injected)
        self.assertIs(camera.dv, design_variable)
        self.assertEqual(camera.ctarget, "detector")

    def test_complete_refine_baselines_skip_pair_stereo_but_run_full_batch(self):
        graph = multicam_graph.MulticamCalibrationGraph.__new__(
            multicam_graph.MulticamCalibrationGraph
        )
        graph.G = igraph.Graph(2, [(0, 1)])
        graph.G.es["weight"] = [10]
        graph.numCams = 2
        graph.isGraphConnected = lambda: True
        transform = np.eye(4)
        transform[0, 3] = 0.12
        baseline = sm.Transformation(transform)

        with mock.patch.object(
            multicam_graph.kcc,
            "stereoCalibrate",
            side_effect=AssertionError("complete seed ran pair stereo"),
        ), mock.patch.object(
            multicam_graph.kcc,
            "solveFullBatch",
            side_effect=lambda cameras, baselines, owner: (True, baselines),
        ) as full_batch:
            result = graph.getInitialGuesses(
                ["cam0", "cam1"], [baseline]
            )

        full_batch.assert_called_once()
        self.assertEqual(graph.optimal_baseline_edges, set())
        np.testing.assert_allclose(result[0].T(), transform)

    def test_partial_refine_baselines_overlay_before_full_batch(self):
        class ObservationDatabase:
            @staticmethod
            def getAllObsTwoCams(cam_a, cam_b):
                return [("obs{0}".format(cam_a), "obs{0}".format(cam_b))]

        graph = multicam_graph.MulticamCalibrationGraph.__new__(
            multicam_graph.MulticamCalibrationGraph
        )
        graph.G = igraph.Graph(3, [(0, 1), (1, 2)])
        graph.G.es["weight"] = [10, 10]
        graph.numCams = 3
        graph.obs_db = ObservationDatabase()
        graph.isGraphConnected = lambda: True

        def transformation(x_translation):
            matrix = np.eye(4)
            matrix[0, 3] = x_translation
            return sm.Transformation(matrix)

        captured = []

        def full_batch(cameras, baselines, owner):
            captured.extend(baselines)
            return True, baselines

        with mock.patch.object(
            multicam_graph.kcc,
            "stereoCalibrate",
            return_value=(True, transformation(1.0)),
        ), mock.patch.object(
            multicam_graph.kcc, "solveFullBatch", side_effect=full_batch
        ):
            graph.getInitialGuesses(
                ["cam0", "cam1", "cam2"],
                [transformation(7.0), None],
            )

        self.assertEqual([baseline.T()[0, 3] for baseline in captured],
                         [7.0, 1.0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
