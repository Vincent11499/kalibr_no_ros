"""Opt-in row-time camera sensors using the native continuous-time IMU problem.

Projection/distortion stay fixed. A signed row delay changes only the time at
which each measured corner queries the trajectory. Existing IccCamera is intact.
"""
import math
import numpy as np
import aslam_backend as aopt
import incremental_calibration as inc
import kalibr_runtime as runtime
from kalibr_no_ros import artifacts
from kalibr_no_ros.rolling_shutter import validate_shutters
from kalibr_no_ros.task import load_yaml, require_document_version
from .IccSensors import IccCamera, IccCameraChain
from .IccCalibrator import IccCalibrator, CALIBRATION_GROUP_ID, HELPER_GROUP_ID


class IccRollingShutterCamera(IccCamera):
    def __init__(self, *args, **kwargs):
        self.timing = kwargs.pop("timing")
        super().__init__(*args, **kwargs)
        self.imageHeight = self.camConfig.getResolution()[1]
        self.referenceRow = (self.imageHeight - 1.0) / 2.0

    def rowTimeExtent(self):
        bound = (self.timing["max_abs_line_delay_s"] if self.timing["estimate"]
                 else abs(self.timing["line_delay_s"]))
        return self.referenceRow * bound

    def addDesignVariables(self, problem, noExtrinsics=True,
                           noTimeCalibration=True, baselinedv_group_id=HELPER_GROUP_ID):
        super().addDesignVariables(problem, noExtrinsics, noTimeCalibration, baselinedv_group_id)
        seed = self.timing["line_delay_s"]
        latent = math.atanh(seed / self.timing["max_abs_line_delay_s"]) if self.timing["estimate"] else 0.0
        self.lineDelayDv = aopt.Scalar(latent)
        self.lineDelayDv.setActive(self.timing["estimate"])
        problem.addDesignVariable(self.lineDelayDv, CALIBRATION_GROUP_ID)
        self.lineDelayExpression = (self.lineDelayDv.toExpression().tanh() * self.timing["max_abs_line_delay_s"]
                                    if self.timing["estimate"] else aopt.ScalarExpression(seed))

    def getShutterResult(self):
        delay = float(self.lineDelayExpression.toScalar())
        result = {"type": "rolling_shutter", "line_delay_s": delay,
                  "reference_row_px": self.referenceRow,
                  "first_to_last_row_span_s": abs(delay) * (self.imageHeight - 1),
                  "estimated": self.timing["estimate"],
                  "time_reference": "t_row_imu = t_frame + timeshift_cam_imu + (y - reference_row_px) * line_delay_s"}
        if self.timing["estimate"]:
            result["max_abs_line_delay_s"] = self.timing["max_abs_line_delay_s"]
        if hasattr(self, "lineDelayStd"):
            result["line_delay_std_s"] = self.lineDelayStd
        return result

    def addCameraErrorTerms(self, problem, poseSplineDv, T_cN_b,
                            blakeZissermanDf=0.0, timeOffsetPadding=0.0):
        all_errors = []
        context = artifacts.current_context()
        # The expression sparsity support must cover every allowed row-delay
        # update, in addition to the existing frame-offset search interval.
        row_margin = self.referenceRow * (
            self.timing["max_abs_line_delay_s"] + abs(self.timing["line_delay_s"])) if self.timing["estimate"] else 0.0
        margin = timeOffsetPadding + row_margin
        for obs in self.targetObservations:
            frame_time = (self.cameraTimeToImuTimeDv.toExpression()
                          + obs.time().toSec() + self.timeshiftCamToImuPrior)
            center = frame_time.toScalar()
            if (center - self.rowTimeExtent() <= poseSplineDv.spline().t_min()
                    or center + self.rowTimeExtent() >= poseSplineDv.spline().t_max()):
                if context is not None:
                    context.record_camera_skipped(obs, "rolling_shutter_outside_spline_support")
                continue
            image_points = np.asarray(obs.getCornersImageFrame())
            target_points = np.asarray(obs.getCornersTargetFrame())
            frame = self.camera.frameType()
            frame.setGeometry(self.camera.geometry)
            invR = np.eye(2) / (self.cornerUncertainty ** 2)
            for measured in image_points:
                keypoint = self.camera.keypointType()
                keypoint.setMeasurement(measured)
                keypoint.setInverseMeasurementCovariance(invR)
                frame.addKeypoint(keypoint)
            errors, times, transforms = [], [], []
            for index, (measured, target) in enumerate(zip(image_points, target_points)):
                time = frame_time + self.lineDelayExpression * float(measured[1] - self.referenceRow)
                T_c_w = T_cN_b * poseSplineDv.transformationAtTime(time, margin, margin).inverse()
                point = T_c_w * aopt.HomogeneousExpression(np.r_[target, 1.0])
                error = self.camera.reprojectionErrorType(frame, index, point)
                if blakeZissermanDf > 0:
                    error.setMEstimatorPolicy(aopt.BlakeZissermanMEstimator(blakeZissermanDf))
                problem.addErrorTerm(error)
                errors.append(error); times.append(time); transforms.append(T_c_w)
            all_errors.append(errors)
            if context is not None:
                reference = T_cN_b * poseSplineDv.transformationAtTime(
                    frame_time, timeOffsetPadding, timeOffsetPadding).inverse()
                context.record_camera_terms(self, obs, errors, reference, frame_time,
                                            corner_times=times, corner_transforms=transforms)
        if not all_errors:
            raise RuntimeError("No rolling-shutter frames inside spline support")
        self.allReprojectionErrors = all_errors


class IccRollingShutterChain(IccCameraChain):
    def __init__(self, chainConfig, targetConfig, parsed):
        document = load_yaml(parsed.rolling_shutter_config)
        require_document_version(document, "rolling shutter", "rolling_shutter_config")
        self.shutters = validate_shutters(document.get("cameras"),
                                         ["cam{}".format(i) for i in range(chainConfig.numCameras())])
        super().__init__(chainConfig, targetConfig, parsed)

    def _make_camera(self, camera_index, *args, **kwargs):
        return IccRollingShutterCamera(*args, timing=self.shutters["cam{}".format(camera_index)], **kwargs)

    def initializePoseSplineFromCameraChain(self, splineOrder=6, poseKnotsPerSecond=100,
                                          timeOffsetPadding=0.02):
        return super().initializePoseSplineFromCameraChain(
            splineOrder, poseKnotsPerSecond,
            timeOffsetPadding + max(cam.rowTimeExtent() for cam in self.camList))


class IccRollingShutterCalibrator(IccCalibrator):
    def optimize(self, *args, **kwargs):
        super().optimize(*args, **kwargs)
        for cam in self.CameraChain.camList:
            value = cam.lineDelayExpression.toScalar()
            if not math.isfinite(value):
                raise RuntimeError("Non-finite rolling-shutter line delay")
            if cam.timing["estimate"] and abs(value) >= .995 * cam.timing["max_abs_line_delay_s"]:
                raise RuntimeError("Rolling-shutter estimate reached its search bound; inspect the data or increase max_abs_line_delay_s")

    def saveCamChainParametersYaml(self, resultFile):
        for index, cam in enumerate(self.CameraChain.camList):
            self.CameraChain.chainConfig.data["cam{}".format(index)]["shutter"] = cam.getShutterResult()
        super().saveCamChainParametersYaml(resultFile)

    def recoverCovariance(self):
        # The CALIBRATION group contains cam0 T, then each camera's frame
        # offset and row latent variable in insertion order. Baselines and IMU
        # states are HELPER variables and are marginalized, as in the native path.
        estimator = inc.IncrementalEstimator(CALIBRATION_GROUP_ID)
        runtime.apply_optimizer_threads(estimator.getOptimizerOptions())
        runtime.run_incremental_batch(estimator, self.problem, True)
        variances = np.asarray(estimator.getSigma2Theta().diagonal())
        expected = 6 + sum(int(cam.cameraTimeToImuTimeDv.isActive())
                           + int(cam.lineDelayDv.isActive())
                           for cam in self.CameraChain.camList)
        if len(variances) != expected:
            raise RuntimeError("Rolling-shutter covariance unavailable: expected {} parameters, got {}".format(
                expected, len(variances)))
        if np.any(variances < 0) or not np.all(np.isfinite(variances)):
            raise RuntimeError("Rolling-shutter covariance is not finite positive semidefinite")
        std = np.sqrt(variances)
        self.std_trafo_ic = std[:6]
        cursor = 6
        self.std_times = []
        for cam in self.CameraChain.camList:
            if cam.cameraTimeToImuTimeDv.isActive():
                self.std_times.append(std[cursor]); cursor += 1
            else:
                self.std_times.append(float('nan'))
            if cam.lineDelayDv.isActive():
                scale = cam.timing["max_abs_line_delay_s"] * (1 - math.tanh(cam.lineDelayDv.toScalar()) ** 2)
                cam.lineDelayStd = float(std[cursor] * scale); cursor += 1
        if cursor != len(std):
            raise RuntimeError("Unexpected calibration covariance ordering")
