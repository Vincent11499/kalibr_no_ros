"""Native row-time Jacobians and noiseless recovery using actual camera terms."""
import ctypes
import ctypes.util
import math
import types
import unittest
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
except ImportError:
    backend = None


@unittest.skipIf(backend is None, "native calibration modules are unavailable")
class RollingShutterNativeTest(unittest.TestCase):
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
