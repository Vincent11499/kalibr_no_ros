import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
SENSORS_SOURCE = (
    ROOT / "src" / "kalibr" / "calibration" / "kalibr" / "python" /
    "kalibr_imu_camera_calibration" / "IccSensors.py"
)


class _Expression:
    def __init__(self, value):
        self.value = np.asarray(value, dtype=float)

    def toEuclidean(self):
        return self.value


class _Transformation:
    def __init__(self, value=None):
        self.value = np.eye(4) if value is None else np.asarray(value, dtype=float)

    def T(self):
        return self.value

    def t(self):
        return self.value[:3, 3]

    def q(self):
        return np.array([0.0, 0.0, 0.0, 1.0])


class _BSpline:
    instances = []

    def __init__(self, order):
        self.order = order
        self.constant = None
        self.__class__.instances.append(self)

    def initConstantSpline(self, start, end, knots, value):
        self.constant = np.asarray(value, dtype=float)


def _load_sensors_source():
    package_name = "_kalibr_imu_seed_test"
    package = types.ModuleType(package_name)
    package.__path__ = []

    calibrator = types.ModuleType(package_name + ".IccCalibrator")
    calibrator.CALIBRATION_GROUP_ID = 0
    calibrator.HELPER_GROUP_ID = 1
    calibrator.addSplineDesignVariables = lambda *args, **kwargs: None
    package.IccCalibrator = calibrator

    util = types.ModuleType(package_name + ".IccUtil")
    package.IccUtil = util

    common = types.ModuleType("kalibr_common")
    common.ImuParameters = type("ImuParameters", (), {})
    common.AslamCamera = type("AslamCamera", (), {})

    sm = types.ModuleType("sm")
    sm.Transformation = _Transformation
    sm.r2quat = lambda rotation: np.asarray(rotation, dtype=float).copy()
    sm.RotationVector = type("RotationVector", (), {})

    backend = types.ModuleType("aslam_backend")
    backend.EuclideanExpression = _Expression

    bsplines = types.ModuleType("bsplines")
    bsplines.BSpline = _BSpline
    bsplines.BSplinePose = type("BSplinePose", (), {})

    stubs = {
        package_name: package,
        package_name + ".IccCalibrator": calibrator,
        package_name + ".IccUtil": util,
        "kalibr_native_optimizer": types.ModuleType("kalibr_native_optimizer"),
        "sm": sm,
        "aslam_cv": types.ModuleType("aslam_cv"),
        "aslam_cameras_april": types.ModuleType("aslam_cameras_april"),
        "aslam_splines": types.ModuleType("aslam_splines"),
        "aslam_backend": backend,
        "bsplines": bsplines,
        "kalibr_common": common,
        "kalibr_errorterms": types.ModuleType("kalibr_errorterms"),
        "incremental_calibration": types.ModuleType("incremental_calibration"),
    }
    module_name = package_name + ".IccSensors"
    spec = importlib.util.spec_from_file_location(module_name, SENSORS_SOURCE)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, stubs):
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    return module


SENSORS = _load_sensors_source()


class CameraImuInitializationLoaderTest(unittest.TestCase):
    def write_document(self, document):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "initialization.yaml"
        path.write_text(yaml.safe_dump(document), encoding="utf-8")
        return path

    @staticmethod
    def base_document(strategy="refine"):
        return {
            "schema_version": 1,
            "kind": "camera_imu_calibration_initialization",
            "strategy": strategy,
        }

    def test_partial_direct_document_is_valid(self):
        document = self.base_document("direct")
        document["camera_imu"] = {
            "timeshift_cam_imu_s": {"cam1": 0.002},
            "gravity_direction_target": [0.0, 0.0, -2.0],
        }
        document["imus"] = {
            "imu1": {"gyroscope_bias_rad_s": [0.1, 0.2, 0.3]},
        }

        loaded = SENSORS.loadCameraImuCalibrationInitialization(
            str(self.write_document(document)), num_cameras=2,
            imu_models=["calibrated", "calibrated"])

        self.assertEqual(loaded["strategy"], "direct")
        self.assertNotIn("T_cam0_imu", loaded["camera_imu"])

    def test_loader_rejects_scalar_time_map_and_bad_model_fields(self):
        document = self.base_document()
        document["camera_imu"] = {"timeshift_cam_imu_s": 0.001}
        with self.assertRaisesRegex(RuntimeError, "must be a mapping"):
            SENSORS.loadCameraImuCalibrationInitialization(
                str(self.write_document(document)))

        document = self.base_document()
        document["imus"] = {"imu0": {"M_accel": np.eye(3).tolist()}}
        with self.assertRaisesRegex(RuntimeError, "not valid.*calibrated"):
            SENSORS.loadCameraImuCalibrationInitialization(
                str(self.write_document(document)),
                imu_models=["calibrated"])

    def test_loader_rejects_nonrigid_transforms_and_boolean_schema(self):
        document = self.base_document()
        transform = np.eye(4)
        transform[0, 0] = 2.0
        document["camera_imu"] = {"T_cam0_imu": transform.tolist()}
        with self.assertRaisesRegex(RuntimeError, "proper rotation"):
            SENSORS.loadCameraImuCalibrationInitialization(
                str(self.write_document(document)))

    def test_loader_rejects_boolean_arrays_and_invalid_scale_structure(self):
        document = self.base_document()
        document["imus"] = {
            "imu0": {"gyroscope_bias_rad_s": [True, 0.0, 0.0]},
        }
        with self.assertRaisesRegex(RuntimeError, "only numbers"):
            SENSORS.loadCameraImuCalibrationInitialization(
                str(self.write_document(document)),
                imu_models=["calibrated"])

        document["imus"] = {
            "imu0": {"M_accel": [[1, 0.1, 0], [0, 1, 0], [0, 0, 1]]},
        }
        with self.assertRaisesRegex(RuntimeError, "lower triangular"):
            SENSORS.loadCameraImuCalibrationInitialization(
                str(self.write_document(document)),
                imu_models=["scale-misalignment"])

        document = self.base_document()
        document["schema_version"] = True
        with self.assertRaisesRegex(RuntimeError, "integer 1"):
            SENSORS.loadCameraImuCalibrationInitialization(
                str(self.write_document(document)))


class CameraImuInitializationHookTest(unittest.TestCase):
    @staticmethod
    def new_imu(model="calibrated", reference=False, estimate_delay=True,
                number=1):
        imu = SENSORS.IccImu.__new__(SENSORS.IccImu)
        imu.isReferenceImu = reference
        imu.estimateTimedelay = estimate_delay
        imu.imuConfig = types.SimpleNamespace(
            imuNr=number, data={"model": model})
        imu.GyroBiasPrior = np.zeros(3)
        imu.AccelBiasPrior = np.zeros(3)
        imu.q_i_b_prior = np.array([0.0, 0.0, 0.0, 1.0])
        imu.r_b_prior = np.zeros(3)
        imu.timeOffset = 0.0
        imu.M_accel_prior = np.eye(3)
        imu.M_gyro_prior = np.eye(3)
        imu.C_gyro_i_prior = np.eye(3)
        imu.A_gyro_accel_prior = np.zeros((3, 3))
        imu.ry_i_prior = np.zeros(3)
        imu.rz_i_prior = np.zeros(3)
        imu.initializationStrategy = None
        imu.initializationFields = set()
        imu.hasGyroBiasInitialization = False
        imu.hasTransformInitialization = False
        imu.hasTimeInitialization = False
        return imu

    def test_imu_seed_converts_full_transform_and_keeps_all_priors(self):
        imu = self.new_imu(
            "scale-misalignment-size-effect", number=1)
        transform = np.eye(4)
        transform[:3, 3] = [1.0, -2.0, 3.0]
        imu.applyInitialization({
            "gyroscope_bias_rad_s": [0.1, 0.2, 0.3],
            "accelerometer_bias_m_s2": [1.0, 2.0, 3.0],
            "T_imu_from_reference": transform.tolist(),
            "time_offset_to_reference_s": 0.02,
            "M_accel": (np.eye(3) * 2.0).tolist(),
            "M_gyro": (np.eye(3) * 3.0).tolist(),
            "C_gyro_i": np.eye(3).tolist(),
            "A_gyro_accel": (np.eye(3) * 0.1).tolist(),
            "ry_i_m": [0.0, 0.01, 0.0],
            "rz_i_m": [0.0, 0.0, 0.02],
        }, "direct")

        np.testing.assert_allclose(imu.r_b_prior, [-1.0, 2.0, -3.0])
        np.testing.assert_allclose(imu.GyroBiasPrior, [0.1, 0.2, 0.3])
        np.testing.assert_allclose(imu.AccelBiasPrior, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(imu.M_accel_prior, np.eye(3) * 2.0)
        np.testing.assert_allclose(imu.ry_i_prior, [0.0, 0.01, 0.0])
        self.assertEqual(imu.timeOffset, 0.02)
        self.assertTrue(imu.hasTransformInitialization)

    def test_reference_and_refine_delay_constraints_are_explicit(self):
        reference = self.new_imu(reference=True, number=0)
        with self.assertRaisesRegex(ValueError, "only valid for non-reference"):
            reference.applyInitialization({
                "T_imu_from_reference": np.eye(4).tolist(),
            }, "direct")

        imu = self.new_imu(estimate_delay=False)
        with self.assertRaisesRegex(ValueError, "imu-delay-by-correlation"):
            imu.applyInitialization({
                "time_offset_to_reference_s": 0.01,
            }, "refine")

    def test_multi_imu_time_seed_uses_reference_clock_spline_domain(self):
        stamp = lambda value: types.SimpleNamespace(toSec=lambda: value)
        reference = self.new_imu(reference=True, number=0)
        reference.imuData = [
            types.SimpleNamespace(stamp=stamp(10.0)),
            types.SimpleNamespace(stamp=stamp(20.0)),
        ]
        imu = self.new_imu(number=1)
        imu.imuData = [
            types.SimpleNamespace(stamp=stamp(110.0)),
            types.SimpleNamespace(stamp=stamp(120.0)),
        ]

        self.assertEqual(
            imu._referenceAngularVelocitySplineBounds(reference),
            (110.0, 120.0),
        )
        imu.applyInitialization(
            {"time_offset_to_reference_s": -100.0}, "direct")
        self.assertEqual(
            imu._referenceAngularVelocitySplineBounds(reference),
            (10.0, 20.0),
        )

    def test_multi_imu_reference_bias_keeps_legacy_zero_without_seed(self):
        class StopAfterReferenceBias(Exception):
            pass

        class DesignVariable:
            def setActive(self, active):
                self.active = active

        stamp = lambda value: types.SimpleNamespace(toSec=lambda: value)
        samples = [
            types.SimpleNamespace(stamp=stamp(0.0)),
            types.SimpleNamespace(stamp=stamp(1.0)),
        ]
        reference = self.new_imu(reference=True, number=0)
        reference.imuData = samples
        reference.GyroBiasPrior = np.array([0.1, 0.2, 0.3])
        imu = self.new_imu(number=1)
        imu.imuData = samples
        problem = types.SimpleNamespace(
            addDesignVariable=lambda design_variable: None)
        spline_design_variable = types.SimpleNamespace(
            numDesignVariables=lambda: 0)

        starts = []
        with mock.patch.object(
                SENSORS.aopt, "OptimizationProblem", return_value=problem,
                create=True), mock.patch.object(
                SENSORS.aopt, "RotationQuaternionDv",
                    side_effect=lambda value: DesignVariable(), create=True), \
                mock.patch.object(
                    SENSORS.asp, "EuclideanBSplineDesignVariable",
                    return_value=spline_design_variable, create=True), \
                mock.patch.object(
                    SENSORS.aopt, "EuclideanPointDv",
                    side_effect=lambda value: (
                        starts.append(np.asarray(value).copy()) or
                        (_ for _ in ()).throw(StopAfterReferenceBias())),
                    create=True):
            with self.assertRaises(StopAfterReferenceBias):
                imu.findOrientationPrior(reference)

        np.testing.assert_array_equal(starts[-1], np.zeros(3))

        reference.hasGyroBiasInitialization = True
        with mock.patch.object(
                SENSORS.aopt, "OptimizationProblem", return_value=problem,
                create=True), mock.patch.object(
                SENSORS.aopt, "RotationQuaternionDv",
                    side_effect=lambda value: DesignVariable(), create=True), \
                mock.patch.object(
                    SENSORS.asp, "EuclideanBSplineDesignVariable",
                    return_value=spline_design_variable, create=True), \
                mock.patch.object(
                    SENSORS.aopt, "EuclideanPointDv",
                    side_effect=lambda value: (
                        starts.append(np.asarray(value).copy()) or
                        (_ for _ in ()).throw(StopAfterReferenceBias())),
                    create=True):
            with self.assertRaises(StopAfterReferenceBias):
                imu.findOrientationPrior(reference)

        np.testing.assert_array_equal(starts[-1], reference.GyroBiasPrior)

    def test_bias_seeds_expand_only_when_bias_splines_are_initialized(self):
        _BSpline.instances = []
        imu = self.new_imu()
        imu.GyroBiasPrior = np.array([0.1, 0.2, 0.3])
        imu.AccelBiasPrior = np.array([1.0, 2.0, 3.0])
        pose = types.SimpleNamespace(t_min=lambda: 0.0, t_max=lambda: 2.0)

        imu.initBiasSplines(pose, splineOrder=4, biasKnotsPerSecond=5)

        np.testing.assert_allclose(
            _BSpline.instances[0].constant, imu.GyroBiasPrior)
        np.testing.assert_allclose(
            _BSpline.instances[1].constant, imu.AccelBiasPrior)

    def test_camera_time_direct_skips_and_refine_adds_residual(self):
        camera = SENSORS.IccCamera.__new__(SENSORS.IccCamera)
        camera.hasTimeshiftInitialization = True
        camera.initializationStrategy = "direct"
        camera.timeshiftCamToImuPrior = 0.25
        camera.initPoseSplineFromCamera = mock.Mock(
            side_effect=AssertionError("correlation spline was constructed"))
        camera.findTimeshiftCameraImuPrior(object())
        camera.initPoseSplineFromCamera.assert_not_called()

        pose = types.SimpleNamespace(
            t_min=lambda: -10.0,
            t_max=lambda: 10.0,
            angularVelocityBodyFrame=lambda time: np.array([1.0, 0.0, 0.0]),
        )
        seen_priors = []
        camera.initializationStrategy = "refine"
        camera.initPoseSplineFromCamera = lambda timeOffsetPadding=0.0: (
            seen_priors.append(camera.timeshiftCamToImuPrior) or pose)
        stamp = lambda value: types.SimpleNamespace(toSec=lambda: value)
        imu = types.SimpleNamespace(imuData=[
            types.SimpleNamespace(stamp=stamp(0.0), omega=np.ones(3)),
            types.SimpleNamespace(stamp=stamp(1.0), omega=np.ones(3)),
        ])
        with mock.patch.object(
                SENSORS.np, "correlate", return_value=np.array([1.0, 0.0, 0.0])):
            camera.findTimeshiftCameraImuPrior(imu)

        self.assertEqual(seen_priors, [0.25])
        self.assertEqual(camera.timeshiftCamToImuPrior, 1.25)

    def test_gravity_seed_is_normalized_to_kalibr_magnitude(self):
        camera = SENSORS.IccCamera.__new__(SENSORS.IccCamera)
        camera.setGravityInitialization([0.0, 0.0, -2.0], "direct")
        np.testing.assert_allclose(camera.gravity_w, [0.0, 0.0, -9.80655])

    def test_transform_seed_is_not_applied_twice_to_orientation_spline(self):
        class StopAfterSplineSelection(Exception):
            pass

        class DesignVariable:
            def setActive(self, active):
                self.active = active

        problem = types.SimpleNamespace(
            addDesignVariable=lambda design_variable: None)
        imu = types.SimpleNamespace(
            GyroBiasPrior=np.zeros(3),
            initializationStrategy=None,
            hasGyroBiasInitialization=False,
        )

        camera = SENSORS.IccCamera.__new__(SENSORS.IccCamera)
        transform = np.eye(4)
        transform[:3, :3] = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        camera.T_extrinsic = _Transformation(transform)
        camera.hasTransformInitialization = True
        camera.initializationStrategy = "direct"
        camera.initPoseSplineFromCamera = mock.Mock(
            side_effect=StopAfterSplineSelection)

        rotation_starts = []
        with mock.patch.object(
                SENSORS.aopt, "OptimizationProblem", return_value=problem,
                create=True), mock.patch.object(
                    SENSORS.aopt, "RotationQuaternionDv",
                    side_effect=lambda value: (
                        rotation_starts.append(np.asarray(value).copy()) or
                        DesignVariable()), create=True), \
                mock.patch.object(
                    SENSORS.aopt, "EuclideanPointDv",
                    side_effect=lambda value: DesignVariable(), create=True):
            with self.assertRaises(StopAfterSplineSelection):
                camera.findOrientationPriorCameraToImu(imu)

        call = camera.initPoseSplineFromCamera.call_args
        self.assertEqual(call.kwargs["timeOffsetPadding"], 0.0)
        np.testing.assert_array_equal(
            call.kwargs["T_c_b_override"], np.eye(4))
        np.testing.assert_array_equal(
            rotation_starts[0], transform[:3, :3].transpose())

        camera.hasTransformInitialization = False
        camera.initPoseSplineFromCamera.reset_mock()
        with mock.patch.object(
                SENSORS.aopt, "OptimizationProblem", return_value=problem,
                create=True), mock.patch.object(
                    SENSORS.aopt, "RotationQuaternionDv",
                    side_effect=lambda value: DesignVariable(), create=True), \
                mock.patch.object(
                    SENSORS.aopt, "EuclideanPointDv",
                    side_effect=lambda value: DesignVariable(), create=True):
            with self.assertRaises(StopAfterSplineSelection):
                camera.findOrientationPriorCameraToImu(imu)

        self.assertEqual(
            camera.initPoseSplineFromCamera.call_args.kwargs,
            {"timeOffsetPadding": 0.0})


if __name__ == "__main__":
    unittest.main(verbosity=2)
