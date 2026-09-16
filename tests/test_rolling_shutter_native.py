"""Native row-time Jacobians and noiseless recovery using actual camera terms."""
import ctypes
import ctypes.util
import math
import types
import unittest
from unittest import mock
import numpy as np
import cv2

library = ctypes.util.find_library("cholmod")
if library:ctypes.CDLL(library, mode=ctypes.RTLD_GLOBAL)
try:
    import aslam_backend as backend
    import aslam_splines
    import bsplines
    import sm
    import kalibr_common as common
    import incremental_calibration
    from kalibr_imu_camera_calibration.IccRollingShutter import IccRollingShutterCamera, IccRollingShutterCalibrator
    from kalibr_imu_camera_calibration.IccSensors import IccCamera
    from kalibr_rs_camera_calibration.RsCalibrator import (
        NATIVE_TIME_EXPRESSION_BUFFER_S,
        NATIVE_TIME_PADDING_S,
        RsCalibrator,
        RsCalibratorConfiguration,
        requireOptimizerSuccess,
    )
    import kalibr_rs_camera_calibration.SystemCalibrator as system_calibrator
except ImportError:
    backend = None


@unittest.skipIf(backend is None, "native calibration modules are unavailable")
class RollingShutterNativeTest(unittest.TestCase):
    def test_native_visual_time_buffer_fits_initial_row_span(self):
        config = RsCalibratorConfiguration()
        config.timeOffsetConstantSparsityPattern = \
            NATIVE_TIME_EXPRESSION_BUFFER_S
        config.timeOffsetPadding = NATIVE_TIME_PADDING_S
        self.assertEqual(config.timeOffsetConstantSparsityPattern, 0.5)
        self.assertEqual(config.timeOffsetPadding, 0.5)
        config.validate(True)
        config.validateTimeSupport(2160, 20e-6)

        config.timeOffsetPadding = 0.25
        with self.assertRaisesRegex(
                ValueError, "must strictly exceed.*initial row span"):
            config.validateTimeSupport(2160, 20e-6)

    def test_native_visual_rejects_solver_failure_and_iteration_exhaustion(self):
        failed = types.SimpleNamespace(
            iterations=3, linearSolverFailure=True,
            dXFinal=1.0, dJFinal=1.0)
        with self.assertRaisesRegex(RuntimeError, "failed during initial solve"):
            requireOptimizerSuccess(failed, 20, "initial solve")

        exhausted = types.SimpleNamespace(
            iterations=20, linearSolverFailure=False,
            dXFinal=0.2, dJFinal=0.01)
        with self.assertRaisesRegex(
                RuntimeError,
                r"adaptive knot iteration 2 within max_iterations=20 .*iterations=20"):
            requireOptimizerSuccess(
                exhausted, 20, "adaptive knot iteration 2")

        converged = types.SimpleNamespace(
            iterations=19, linearSolverFailure=False,
            dXFinal=1e-9, dJFinal=1e-5)
        self.assertIs(
            requireOptimizerSuccess(converged, 20, "initial solve"),
            converged)

    def test_native_visual_filters_failed_pnp_before_spline_initialization(self):
        class Observation:
            def __init__(self, timestamp, pnp_success=True):
                self.timestamp = timestamp
                self.pnp_success = pnp_success
                self.pose = None

            def time(self):
                return types.SimpleNamespace(toSec=lambda: self.timestamp)

            def set_T_t_c(self, pose):
                self.pose = pose

        class Camera:
            @staticmethod
            def estimateTransformation(observation):
                return observation.pnp_success, sm.Transformation()

        observations = [
            Observation(float(index), pnp_success=(index != 2))
            for index in range(5)
        ]
        calibrator = RsCalibrator()
        calibrator._RsCalibrator__observations = observations
        calibrator._RsCalibrator__camera = Camera()
        calibrator._RsCalibrator__config = types.SimpleNamespace(splineOrder=4)

        calibrator._RsCalibrator__generateExtrinsicsInitialGuess()

        retained = calibrator._RsCalibrator__observations
        self.assertEqual(retained,
                         [observations[0], observations[1],
                          observations[3], observations[4]])
        self.assertTrue(all(observation.pose is not None
                            for observation in retained))

        calibrator._RsCalibrator__observations = observations[:4]
        with self.assertRaisesRegex(
                RuntimeError,
                r"requires at least 4 finite, unique PnP target poses; "
                r"retained 3 of 4 observations \(1 PnP failures"):
            calibrator._RsCalibrator__generateExtrinsicsInitialGuess()

    def test_visual_system_rejects_iteration_budget_exhaustion(self):
        calibrator = system_calibrator.SystemRsCalibrator.__new__(
            system_calibrator.SystemRsCalibrator)
        calibrator.problem = object()
        calibrator.max_iterations = 80
        calibrator.verbose = False
        result = types.SimpleNamespace(
            iterations=80, failedIterations=0,
            linearSolverFailure=False, dXFinal=0.21867490780318674,
            dJFinal=0.01368354311512121)

        optimizer = mock.Mock()
        with mock.patch.object(
                system_calibrator.aopt, "Optimizer2Options",
                return_value=types.SimpleNamespace()), mock.patch.object(
                system_calibrator.aopt, "BlockCholeskyLinearSystemSolver",
                return_value=object()), mock.patch.object(
                system_calibrator.aopt, "LevenbergMarquardtTrustRegionPolicy",
                return_value=object()), mock.patch.object(
                system_calibrator.aopt, "Optimizer2", return_value=optimizer), \
                mock.patch.object(
                    system_calibrator.native_runtime, "apply_optimizer_threads"), \
                mock.patch.object(
                    system_calibrator.native_runtime, "run_optimizer",
                    return_value=result):
            with self.assertRaisesRegex(
                    RuntimeError,
                    r"max_iterations=80 .*iterations=80, dXFinal=0\.218674.*dJFinal=0\.0136835"):
                calibrator.optimize()
        self.assertIs(calibrator.optimizer_result, result)

    def test_visual_system_rejects_nonfinite_trajectory_and_pixel_residuals(self):
        class Projection:
            @staticmethod
            def getParameters():
                return np.array([400.0, 400.0, 320.0, 240.0])

            @staticmethod
            def distortion():
                return types.SimpleNamespace(
                    getParameters=lambda: np.zeros(4))

        class Error:
            def __init__(self, prediction):
                self.prediction = np.asarray(prediction, dtype=float)

            @staticmethod
            def evaluateError():
                return 0.0

            @staticmethod
            def getMeasurement():
                return np.array([10.0, 20.0])

            def getPredictedMeasurement(self):
                return self.prediction

        calibrator = system_calibrator.SystemRsCalibrator.__new__(
            system_calibrator.SystemRsCalibrator)
        calibrator.cameras = [types.SimpleNamespace(
            geometry=types.SimpleNamespace(projection=lambda: Projection()))]
        calibrator.baselines = []
        calibrator.line_delays = [types.SimpleNamespace(
            camera_id="cam0", value_s=lambda: 8e-6)]
        calibrator.spline_dv = types.SimpleNamespace(
            spline=lambda: types.SimpleNamespace(
                coefficients=lambda: np.array([[float("nan")]])))
        calibrator.views = []

        with self.assertRaisesRegex(RuntimeError,
                                    "trajectory coefficients are not finite"):
            calibrator._validate_solution()

        calibrator.spline_dv = types.SimpleNamespace(
            spline=lambda: types.SimpleNamespace(
                coefficients=lambda: np.zeros((6, 4))))
        calibrator.views = [types.SimpleNamespace(
            rig_observations=[(0, object())],
            rerrs={0: [Error([float("nan"), 20.0])]})]
        with self.assertRaisesRegex(
                RuntimeError,
                "non-finite two-dimensional final reprojection"):
            calibrator._validate_solution()

        calibrator.views[0].rerrs[0] = [Error([9.0, 18.0])]
        self.assertAlmostEqual(
            calibrator.reprojection_rms()[0], math.sqrt(5.0))

    def test_visual_line_delay_state_is_bounded_and_uses_row_zero(self):
        class Projection:
            @staticmethod
            def rv():
                return 480

        class Shutter:
            def __init__(self):
                self.parameters = None

            def setParameters(self, value):
                self.parameters = np.asarray(value, dtype=float)

        native_shutter = Shutter()
        geometry = types.SimpleNamespace(geometry=types.SimpleNamespace(
            projection=lambda: Projection(), shutter=lambda: native_shutter))
        state = system_calibrator.LineDelayState(
            "cam0", geometry,
            {"line_delay_s": 8e-6, "estimate": True,
             "max_abs_line_delay_s": 2e-5})
        self.assertAlmostEqual(state.value_s(), 8e-6, places=15)
        self.assertAlmostEqual(state.support_extent_s, 479 * 2e-5)
        self.assertAlmostEqual(
            state.expression_margin_s, 479 * (2e-5 + 8e-6))
        state.dv.update(np.array([30.0]))
        self.assertLess(abs(state.value_s()), 2e-5 + 1e-18)
        state.dv.revertUpdate()
        result = state.result()
        self.assertEqual(result["reference_row_px"], 0.0)
        self.assertEqual(result["timestamp_reference"], "row0_exposure_end")
        self.assertEqual(
            result["corner_time_equation"],
            "t_corner_s = t_camera_timestamp_s + y_px * line_delay_s")
        self.assertAlmostEqual(result["first_to_last_row_span_s"], 479 * 8e-6)
        state.sync_native_shutter()
        np.testing.assert_allclose(native_shutter.parameters, [8e-6])

        fixed = system_calibrator.LineDelayState(
            "cam0", geometry,
            {"line_delay_s": -8e-6, "estimate": False})
        self.assertFalse(fixed.dv.isActive())
        self.assertNotIn("max_abs_line_delay_s", fixed.result())
        self.assertAlmostEqual(fixed.support_extent_s, 479 * 8e-6)
        self.assertAlmostEqual(fixed.expression_margin_s, 479 * 8e-6)

    def test_system_spline_default_tracks_selected_observation_rate(self):
        pose_samples = []
        for index in range(26):
            matrix = np.eye(4)
            matrix[0, 3] = 0.01 * index
            pose_samples.append((1.2 * index, sm.Transformation(matrix)))

        with self.assertRaisesRegex(
                RuntimeError, "at least 3 for the second-order motion prior"):
            system_calibrator.make_pose_spline(
                pose_samples, order=2, padding_s=0.5)
        with self.assertRaisesRegex(
                RuntimeError, "at least 3 for the second-order motion prior"):
            system_calibrator.SystemRsCalibrator(
                [], None, [], {}, spline_order=2)

        _, metadata = system_calibrator.make_pose_spline(
            pose_samples, order=4, padding_s=0.5)
        _, explicit_metadata = system_calibrator.make_pose_spline(
            pose_samples, order=4, padding_s=0.5,
            knots_per_second=1.0 / 1.2)

        self.assertEqual(metadata["strategy"], "selected_observation_rate")
        self.assertAlmostEqual(
            metadata["selected_frame_rate_hz"], 1.0 / 1.2)
        self.assertAlmostEqual(
            metadata["requested_knots_per_second"], 1.0 / 1.2)
        self.assertEqual(metadata["requested_segment_count"], 27)
        self.assertEqual(metadata["minimum_segment_count"], 8)
        self.assertNotIn("automatic_maximum_segment_count", metadata)
        self.assertEqual(metadata["segment_count"], 27)
        self.assertEqual(explicit_metadata["segment_count"], 27)
        self.assertEqual(
            explicit_metadata["strategy"], "explicit_knots_per_second")

        support = [-0.02] + [1.2 * index for index in range(26)] + [30.02]
        extended, extended_metadata = system_calibrator.make_pose_spline(
            pose_samples[1:-1], order=4, padding_s=0.5,
            selected_timestamps=[1.2 * index for index in range(26)],
            support_timestamps=support)
        self.assertEqual(extended_metadata["selected_timestamp_count"], 26)
        self.assertEqual(extended_metadata["support_timestamp_count"], 28)
        self.assertAlmostEqual(
            extended_metadata["selected_frame_rate_hz"], 1.0 / 1.2)
        self.assertLessEqual(float(extended.t_min()), min(support))
        self.assertGreaterEqual(float(extended.t_max()), max(support))

    def test_system_observation_timestamps_include_views_without_pnp(self):
        def observation(timestamp):
            return types.SimpleNamespace(
                time=lambda: types.SimpleNamespace(toSec=lambda: timestamp))

        views = {
            0: [(1, observation(0.002)), (0, observation(0.0))],
            1: [(1, observation(1.003))],
            2: [(0, observation(2.0)), (1, observation(2.004))],
        }
        calibrator = system_calibrator.SystemRsCalibrator.__new__(
            system_calibrator.SystemRsCalibrator)
        calibrator.obsdb = types.SimpleNamespace(
            getAllViewTimestamps=lambda: list(views),
            getAllObsAtTimestamp=lambda timestamp: views[timestamp])

        selected, support = calibrator._observation_timestamps()
        self.assertEqual(selected, [0.0, 1.003, 2.0])
        self.assertEqual(
            support, [0.002, 0.0, 1.003, 2.0, 2.004])

    def test_saturated_error_lists_estimate_and_bound_per_camera(self):
        calibrator = system_calibrator.SystemRsCalibrator.__new__(
            system_calibrator.SystemRsCalibrator)
        calibrator.line_delays = [
            types.SimpleNamespace(
                camera_id="cam0", estimate=True, bound_s=2e-5,
                value_s=lambda: 1.999e-5),
            types.SimpleNamespace(
                camera_id="cam1", estimate=True, bound_s=3e-5,
                value_s=lambda: -2.999e-5),
            types.SimpleNamespace(
                camera_id="cam2", estimate=False, bound_s=None,
                value_s=lambda: 0.0),
        ]

        with self.assertRaises(RuntimeError) as raised:
            calibrator.require_interior_line_delays()
        message = str(raised.exception)
        self.assertIn("cam0: estimate=", message)
        self.assertIn("bound=2", message)
        self.assertIn("cam1: estimate=", message)
        self.assertIn("bound=3", message)
        self.assertNotIn("cam2: estimate=", message)
        self.assertIn("s/row", message)

    def test_three_camera_baseline_chain_is_previous_to_current(self):
        first_matrix = np.eye(4)
        first_matrix[:3, :3] = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        first_matrix[:3, 3] = [1.0, 2.0, 0.0]
        second_matrix = np.eye(4)
        second_matrix[:3, :3] = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ])
        second_matrix[:3, 3] = [-0.5, 0.0, 3.0]
        first = sm.Transformation(first_matrix)
        second = sm.Transformation(second_matrix)
        cumulative = system_calibrator.SystemRsCalibrator._cumulative_baseline(
            [first, second], 2)
        np.testing.assert_allclose(
            cumulative.T(), second_matrix @ first_matrix, atol=1e-14)

    def test_visual_system_uses_each_camera_timestamp_and_row(self):
        class FakeProblem:
            def __init__(self):
                self.errors = []

            def addErrorTerm(self, error):
                self.errors.append(error)

        class FakeTime:
            def __init__(self, value):
                self.value = value

            def toSec(self):
                return self.value

        class FakeObservation:
            def __init__(self, timestamp, row):
                self.timestamp = timestamp
                self.row = row

            def time(self):
                return FakeTime(self.timestamp)

            @staticmethod
            def getCornersIdx():
                return [0]

            def imagePoint(self, unused_corner_id):
                return True, np.array([50.0, self.row])

        class FakeTarget:
            @staticmethod
            def size():
                return 1

        class FakeModel:
            @staticmethod
            def reprojectionError(*unused_arguments):
                return types.SimpleNamespace()

        class FakeSpline:
            @staticmethod
            def transformationAtTime(time, unused_left, unused_right):
                return types.SimpleNamespace(
                    toTransformationMatrix=lambda: np.eye(4))

        observations = [
            (0, FakeObservation(1.0, 100.0)),
            (1, FakeObservation(1.002, 200.0)),
        ]
        calibrator = system_calibrator.SystemRsCalibrator.__new__(
            system_calibrator.SystemRsCalibrator)
        calibrator.obsdb = types.SimpleNamespace(
            getAllViewTimestamps=lambda: [1.0],
            getAllObsAtTimestamp=lambda unused_timestamp: observations)
        calibrator.line_delays = [
            types.SimpleNamespace(
                expression=backend.ScalarExpression(1e-5),
                support_extent_s=0.01, expression_margin_s=0.012),
            types.SimpleNamespace(
                expression=backend.ScalarExpression(-2e-5),
                support_extent_s=0.01, expression_margin_s=0.023),
        ]
        calibrator.cameras = [
            types.SimpleNamespace(model=FakeModel(), dv=object()),
            types.SimpleNamespace(model=FakeModel(), dv=object()),
        ]
        calibrator.spline_dv = FakeSpline()
        calibrator.feature_sigma_px = 1.0
        calibrator.use_blake_zisserman = False
        calibrator._active_corners = None
        calibrator._add_design_variables = lambda unused_problem: None
        calibrator._target_landmarks = lambda unused_problem: (
            FakeTarget(), [object()])
        evaluated_times = []

        # Special methods are resolved on the class, so use a tiny explicit
        # transform class rather than relying on SimpleNamespace.__mul__.
        class FakeTransform:
            def __mul__(self, point):
                return point

        def camera_target_expression(camera_id, timestamp, margin):
            evaluated_times.append((camera_id, timestamp.toScalar(), margin))
            return FakeTransform()

        calibrator._camera_target_expression = camera_target_expression
        original_problem = system_calibrator.inc.CalibrationOptimizationProblem
        system_calibrator.inc.CalibrationOptimizationProblem = FakeProblem
        try:
            calibrator.build_problem()
        finally:
            system_calibrator.inc.CalibrationOptimizationProblem = original_problem

        self.assertTrue(any(camera == 0 and abs(time - 1.0) < 1e-15
                            for camera, time, margin in evaluated_times))
        self.assertTrue(any(camera == 1 and abs(time - 1.002) < 1e-15
                            for camera, time, margin in evaluated_times))
        self.assertTrue(any(camera == 0 and abs(time - 1.001) < 1e-15
                            for camera, time, margin in evaluated_times))
        self.assertTrue(any(camera == 1 and abs(time - 0.998) < 1e-15
                            for camera, time, margin in evaluated_times))
        self.assertTrue(all(abs(margin - 0.023001) < 1e-15
                            for camera, time, margin in evaluated_times))

    def test_bounded_scalar_value_and_jacobians(self):
        for value in [-2., 0., .8, 2.]:
            dv = backend.Scalar(value);dv.setActive(True);dv.setBlockIndex(0)
            expression = dv.toExpression().tanh() * 2e-5
            jac = backend.JacobianContainer(1);expression.evaluateJacobians(jac)
            step = 1e-6
            dv.update(np.array([step]));plus = expression.toScalar();dv.revertUpdate()
            dv.update(np.array([-step]));minus = expression.toScalar();dv.revertUpdate()
            self.assertAlmostEqual(expression.toScalar(), math.tanh(value)*2e-5, places=15)
            np.testing.assert_allclose(jac.asDenseMatrix(), [[(plus-minus)/(2*step)]], rtol=1e-8, atol=1e-14)
            chained = backend.JacobianContainer(2)
            expression.evaluateJacobians(chained, np.array([[2.],[-3.]]))
            np.testing.assert_allclose(chained.asDenseMatrix(), np.array([[2.],[-3.]]) @ jac.asDenseMatrix())

    def _problem(self, true_delay, estimate, seed=0.0, calibrate_transform=False):
        spline = bsplines.BSplinePose(4, sm.RotationVector())
        times = np.linspace(0., 2., 61)
        curve = np.array([.3*np.sin(5*times), .2*np.cos(3*times), .05*np.sin(times),
                          .1*np.sin(4*times), .2*np.sin(2*times), .1*np.cos(3*times)])
        spline.initPoseSplineSparse(times, curve, 35, 1e-8)
        spline_dv = aslam_splines.BSplinePoseDesignVariable(spline)
        problem = incremental_calibration.CalibrationOptimizationProblem() if calibrate_transform else backend.OptimizationProblem()
        # This synthetic check uses known motion. Joint IMU/trajectory behavior
        # is exercised separately by the real-data validation.
        for i in range(spline_dv.numDesignVariables()):
            dv=spline_dv.designVariable(i);dv.setActive(False)
            if calibrate_transform:problem.addDesignVariable(dv,1)
            else:problem.addDesignVariable(dv)
        camera = IccRollingShutterCamera.__new__(IccRollingShutterCamera)
        camera.timing = {"estimate": estimate,"line_delay_s":seed,"max_abs_line_delay_s":4e-5}
        camera.imageHeight=480;camera.referenceRow=239.5
        camera.camera=common.AslamCamera('pinhole',[400.,400.,320.,240.], 'equidistant',[0.,0.,0.,0.],[640,480])
        camera.T_extrinsic=sm.Transformation();camera.timeshiftCamToImuPrior=0.;camera.cornerUncertainty=1.
        # IccCamera uses grouped insertion; adapt the native problem signature.
        grouped=problem if calibrate_transform else types.SimpleNamespace(addDesignVariable=lambda dv, group:problem.addDesignVariable(dv))
        camera.addDesignVariables(grouped, noExtrinsics=not calibrate_transform,
                                  noTimeCalibration=False, baselinedv_group_id=0)
        xyz=np.array([[x,y,3.] for y in np.linspace(-.8,.8,5) for x in np.linspace(-1.,1.,5)])
        intrinsic=np.array([[400.,0.,320.],[0.,400.,240.],[0.,0.,1.]])
        observations=[]
        for t in np.linspace(.3,1.7,9):
            measurements=[]
            for point in xyz:
                y=239.5
                for _ in range(12):
                    T=np.linalg.inv(spline.transformation(t+(y-239.5)*true_delay))
                    rv=cv2.Rodrigues(T[:3,:3])[0]
                    uv=cv2.fisheye.projectPoints(point.reshape(1,1,3),rv,T[:3,3],intrinsic,np.zeros(4))[0].reshape(2)
                    y=uv[1]
                measurements.append(uv)
            observations.append(types.SimpleNamespace(time=lambda t=t:types.SimpleNamespace(toSec=lambda:t),
                getCornersImageFrame=lambda points=np.array(measurements):points,
                getCornersTargetFrame=lambda:xyz))
        camera.targetObservations=observations
        camera.dataset=types.SimpleNamespace(topic='/camera')
        camera.addCameraErrorTerms(problem,spline_dv,camera.T_c_b_Dv.toExpression(),timeOffsetPadding=.02)
        return camera,problem,spline_dv

    def test_positive_and_negative_line_time_recovery(self):
        for truth in [1.2e-5,-1.2e-5]:
            camera,problem,spline=self._problem(truth,True)
            options=backend.Optimizer2Options();options.nThreads=1;options.maxIterations=40
            options.convergenceDeltaJ=1e-14;options.convergenceDeltaX=1e-10
            options.trustRegionPolicy=backend.LevenbergMarquardtTrustRegionPolicy(1e-3)
            options.linearSolver=backend.BlockCholeskyLinearSystemSolver()
            optimizer=backend.Optimizer2(options);optimizer.setProblem(problem);result=optimizer.optimize()
            self.assertFalse(result.linearSolverFailure)
            self.assertAlmostEqual(camera.lineDelayExpression.toScalar(),truth,delta=1e-9)
            self.assertAlmostEqual(camera.cameraTimeToImuTimeDv.toScalar(),0.,delta=1e-8)
            error=camera.allReprojectionErrors[0][0];error.evaluateError()
            jac=backend.JacobianContainer(2);error.evaluateJacobians(jac)
            analytic=np.asarray(jac.Jacobian(camera.lineDelayDv)).reshape(2)
            h=1e-5
            camera.lineDelayDv.update(np.array([h]));error.evaluateError();plus=np.asarray(error.error()).copy();camera.lineDelayDv.revertUpdate()
            camera.lineDelayDv.update(np.array([-h]));error.evaluateError();minus=np.asarray(error.error()).copy();camera.lineDelayDv.revertUpdate()
            np.testing.assert_allclose(analytic,(plus-minus)/(2*h),atol=1e-7,rtol=1e-4)

    def test_fixed_zero_delay_matches_original_camera_residuals(self):
        camera,problem,spline=self._problem(0.,False)
        rs=np.array([e.evaluateError() for frame in camera.allReprojectionErrors for e in frame])
        IccCamera.addCameraErrorTerms(camera,problem,spline,camera.T_c_b_Dv.toExpression(),timeOffsetPadding=.02)
        gs=np.array([e.evaluateError() for frame in camera.allReprojectionErrors for e in frame])
        np.testing.assert_allclose(rs,gs,atol=1e-20)
        self.assertFalse(camera.lineDelayDv.isActive())

    def test_native_covariance_keeps_time_and_row_parameter_order(self):
        camera,problem,spline=self._problem(1.2e-5,True,calibrate_transform=True)
        # The native incremental covariance solver requires a nonempty helper
        # block to marginalize (real tasks have trajectory and IMU states).
        helper=backend.TransformationDv(sm.Transformation())
        problem.addDesignVariable(helper.q,1);problem.addDesignVariable(helper.t,1)
        problem.addErrorTerm(backend.ErrorTermTransformation(
            helper.toExpression(),sm.Transformation(),1.,1.))
        calibrator=IccRollingShutterCalibrator()
        calibrator.CameraChain=types.SimpleNamespace(camList=[camera])
        calibrator.problem=problem
        options=backend.Optimizer2Options();options.nThreads=1;options.maxIterations=40
        options.convergenceDeltaJ=1e-14;options.convergenceDeltaX=1e-10
        options.trustRegionPolicy=backend.LevenbergMarquardtTrustRegionPolicy(1e-3)
        options.linearSolver=backend.BlockCholeskyLinearSystemSolver()
        calibrator.optimize(options=options,recoverCov=True)
        self.assertEqual(len(calibrator.std_trafo_ic),6)
        self.assertEqual(len(calibrator.std_times),1)
        self.assertTrue(np.isfinite(calibrator.std_times).all())
        self.assertGreater(camera.lineDelayStd,0.)
        self.assertAlmostEqual(camera.lineDelayExpression.toScalar(),1.2e-5,delta=1e-9)


if __name__ == "__main__":unittest.main()
