"""Inject native runtime calls into staged Kalibr Python modules.

The frozen ETHZ sources stay untouched.  These small, anchor-checked rewrites
apply thread overrides only after each upstream stage has assigned its native
defaults and time the original Boost.Python objects without replacing their
types with Python factories or proxies.
"""

from __future__ import print_function

import argparse
import os


SUPPORTED_SOURCES = (
    "CameraIntializers.py",
    "CameraCalibrator.py",
    "IccSensors.py",
    "IccCalibrator.py",
)
_MARKER = "# kalibr-native-source-overlay"


def _replace(source, old, new, expected, description):
    occurrences = source.count(old)
    if occurrences != expected:
        raise RuntimeError(
            "expected {0} {1} anchor(s), found {2}".format(
                expected, description, occurrences
            )
        )
    return source.replace(old, new)


def _inject_import(source, anchor):
    return _replace(
        source,
        anchor,
        anchor + "import kalibr_native_optimizer as native_runtime\n" + _MARKER + "\n",
        1,
        "runtime import",
    )


def _transform_camera_initializers(source):
    source = _inject_import(source, "import sm\n")
    source = _replace(
        source,
        "    optimizer = aopt.Optimizer2(options)\n",
        "    native_runtime.apply_optimizer_threads(options)\n"
        "    optimizer = aopt.Optimizer2(options)\n",
        3,
        "CameraIntializers Optimizer2 construction",
    )
    return _replace(
        source,
        "optimizer.optimize()",
        "native_runtime.run_optimizer(optimizer)",
        3,
        "CameraIntializers optimize call",
    )


def _transform_camera_calibrator(source):
    source = _inject_import(
        source, "from __future__ import print_function #handle print in 2.x python\n"
    )
    return _replace(
        source,
        "        self.estimator_return_value = self.estimator.addBatch(batch_problem, force)\n",
        "        native_runtime.apply_optimizer_threads(self.optimizerOptions)\n"
        "        self.estimator_return_value = native_runtime.run_incremental_batch(\n"
        "            self.estimator, batch_problem, force)\n",
        1,
        "CameraCalibrator incremental batch",
    )


def _transform_icc_sensors(source):
    source = _inject_import(
        source, "from __future__ import print_function #handle print in 2.x python\n"
    )
    source = _replace(
        source,
        "    reader = kc.BagImageDatasetReader(bagfile, topic, bag_from_to=from_to, bag_freq=freq, \\\n"
        "                                      perform_synchronization=perform_synchronization)\n",
        "    reader = native_runtime.timed_call(\n"
        "        \"image_bag_index\", \"io\", {\"topic\": topic},\n"
        "        kc.BagImageDatasetReader, bagfile, topic,\n"
        "        bag_from_to=from_to, bag_freq=freq,\n"
        "        perform_synchronization=perform_synchronization)\n",
        1,
        "IccSensors image bag index",
    )
    source = _replace(
        source,
        "    reader = kc.BagImuDatasetReader(bagfile, topic, bag_from_to=from_to, \\\n"
        "                                      perform_synchronization=perform_synchronization)\n",
        "    reader = native_runtime.timed_call(\n"
        "        \"imu_bag_read\", \"io\", {\"topic\": topic},\n"
        "        kc.BagImuDatasetReader, bagfile, topic,\n"
        "        bag_from_to=from_to,\n"
        "        perform_synchronization=perform_synchronization)\n",
        1,
        "IccSensors IMU bag read",
    )
    source = _replace(
        source,
        "        self.loadImuData()\n",
        "        native_runtime.timed_call(\n"
        "            \"imu_measurement_build\", \"problem_build\",\n"
        "            {\"topic\": self.dataset.topic}, self.loadImuData)\n",
        1,
        "IccSensors IMU measurement build",
    )
    source = _replace(
        source,
        "        optimizer = aopt.Optimizer2(options)\n",
        "        native_runtime.apply_optimizer_threads(options)\n"
        "        optimizer = aopt.Optimizer2(options)\n",
        2,
        "IccSensors Optimizer2 construction",
    )
    return _replace(
        source,
        "optimizer.optimize()",
        "native_runtime.run_optimizer(optimizer)",
        3,
        "IccSensors optimize call",
    )


def _transform_icc_calibrator(source):
    source = _inject_import(source, "import aslam_backend as aopt\n")
    source = _replace(
        source,
        "                cam.findTimeshiftCameraImuPrior(self.ImuList[0], verbose)\n",
        "                native_runtime.timed_call(\n"
        "                    \"time_offset_initialization\", \"initialization\",\n"
        "                    {\"topic\": cam.dataset.topic},\n"
        "                    cam.findTimeshiftCameraImuPrior,\n"
        "                    self.ImuList[0], verbose)\n",
        1,
        "IccCalibrator time offset initialization",
    )
    source = _replace(
        source,
        "        self.CameraChain.findOrientationPriorCameraChainToImu(self.ImuList[0])\n",
        "        native_runtime.timed_call(\n"
        "            \"orientation_prior_initialization\", \"initialization\", {},\n"
        "            self.CameraChain.findOrientationPriorCameraChainToImu,\n"
        "            self.ImuList[0])\n",
        1,
        "IccCalibrator orientation initialization",
    )
    source = _replace(
        source,
        "        poseSpline = self.CameraChain.initializePoseSplineFromCameraChain(splineOrder, poseKnotsPerSecond, timeOffsetPadding)\n",
        "        poseSpline = native_runtime.timed_call(\n"
        "            \"pose_spline_init\", \"initialization\",\n"
        "            {\"spline_order\": splineOrder,\n"
        "             \"knots_per_second\": poseKnotsPerSecond},\n"
        "            self.CameraChain.initializePoseSplineFromCameraChain,\n"
        "            splineOrder, poseKnotsPerSecond, timeOffsetPadding)\n",
        1,
        "IccCalibrator pose spline initialization",
    )
    source = _replace(
        source,
        "            imu.initBiasSplines(poseSpline, splineOrder, biasKnotsPerSecond)\n",
        "            native_runtime.timed_call(\n"
        "                \"bias_spline_init\", \"initialization\",\n"
        "                {\"topic\": imu.dataset.topic,\n"
        "                 \"spline_order\": splineOrder,\n"
        "                 \"knots_per_second\": biasKnotsPerSecond},\n"
        "                imu.initBiasSplines, poseSpline, splineOrder,\n"
        "                biasKnotsPerSecond)\n",
        1,
        "IccCalibrator bias spline initialization",
    )
    source = _replace(
        source,
        "        self.initDesignVariables(problem, poseSpline, noTimeCalibration, noChainExtrinsics, initialGravityEstimate = estimatedGravity)\n",
        "        native_runtime.timed_call(\n"
        "            \"design_variable_initialization\", \"problem_build\", {},\n"
        "            self.initDesignVariables, problem, poseSpline,\n"
        "            noTimeCalibration, noChainExtrinsics,\n"
        "            initialGravityEstimate=estimatedGravity)\n",
        1,
        "IccCalibrator design variables",
    )
    source = _replace(
        source,
        "        self.CameraChain.addCameraChainErrorTerms(problem, self.poseDv, blakeZissermanDf=blakeZisserCam, timeOffsetPadding=timeOffsetPadding)\n",
        "        native_runtime.timed_call(\n"
        "            \"camera_error_build\", \"problem_build\", {},\n"
        "            self.CameraChain.addCameraChainErrorTerms,\n"
        "            problem, self.poseDv, blakeZissermanDf=blakeZisserCam,\n"
        "            timeOffsetPadding=timeOffsetPadding)\n",
        1,
        "IccCalibrator camera errors",
    )
    source = _replace(
        source,
        "            imu.addAccelerometerErrorTerms(problem, self.poseDv, self.gravityExpression, mSigma=huberAccel, accelNoiseScale=accelNoiseScale)\n"
        "            imu.addGyroscopeErrorTerms(problem, self.poseDv, mSigma=huberGyro, gyroNoiseScale=gyroNoiseScale, g_w=self.gravityExpression)\n",
        "            native_runtime.timed_call(\n"
        "                \"accel_error_build\", \"problem_build\",\n"
        "                {\"topic\": imu.dataset.topic},\n"
        "                imu.addAccelerometerErrorTerms, problem, self.poseDv,\n"
        "                self.gravityExpression, mSigma=huberAccel,\n"
        "                accelNoiseScale=accelNoiseScale)\n"
        "            native_runtime.timed_call(\n"
        "                \"gyro_error_build\", \"problem_build\",\n"
        "                {\"topic\": imu.dataset.topic},\n"
        "                imu.addGyroscopeErrorTerms, problem, self.poseDv,\n"
        "                mSigma=huberGyro, gyroNoiseScale=gyroNoiseScale,\n"
        "                g_w=self.gravityExpression)\n",
        1,
        "IccCalibrator IMU errors",
    )
    source = _replace(
        source,
        "                imu.addBiasMotionTerms(problem)\n",
        "                native_runtime.timed_call(\n"
        "                    \"bias_motion_error_build\", \"problem_build\",\n"
        "                    {\"topic\": imu.dataset.topic},\n"
        "                    imu.addBiasMotionTerms, problem)\n",
        1,
        "IccCalibrator bias motion errors",
    )
    source = _replace(
        source,
        "        self.optimizer = aopt.Optimizer2(options)\n",
        "        native_runtime.apply_optimizer_threads(options)\n"
        "        self.optimizer = aopt.Optimizer2(options)\n",
        1,
        "IccCalibrator Optimizer2 construction",
    )
    source = _replace(
        source,
        "self.optimizer.optimize()",
        "native_runtime.run_optimizer(self.optimizer)",
        1,
        "IccCalibrator optimize call",
    )
    return _replace(
        source,
        "        estimator = inc.IncrementalEstimator(CALIBRATION_GROUP_ID)\n"
        "        rval = estimator.addBatch(self.problem, True)    \n",
        "        estimator = inc.IncrementalEstimator(CALIBRATION_GROUP_ID)\n"
        "        native_runtime.apply_optimizer_threads(\n"
        "            estimator.getOptimizerOptions())\n"
        "        rval = native_runtime.run_incremental_batch(\n"
        "            estimator, self.problem, True)\n",
        1,
        "IccCalibrator covariance batch",
    )


_TRANSFORMERS = {
    "CameraIntializers.py": _transform_camera_initializers,
    "CameraCalibrator.py": _transform_camera_calibrator,
    "IccSensors.py": _transform_icc_sensors,
    "IccCalibrator.py": _transform_icc_calibrator,
}


def transform_source(source, source_name):
    """Return one staged module with its explicit runtime integration."""

    source_name = os.path.basename(str(source_name))
    if source_name not in _TRANSFORMERS:
        raise ValueError("unsupported Kalibr source overlay: {}".format(source_name))
    if _MARKER in source:
        return source
    return _TRANSFORMERS[source_name](source)


def transform_file(path, source_name=None, output=None):
    source_name = source_name or os.path.basename(path)
    output = output or path
    with open(path, "r") as source_file:
        transformed = transform_source(source_file.read(), source_name)
    temporary = "{0}.tmp.{1}".format(output, os.getpid())
    with open(temporary, "w") as output_file:
        output_file.write(transformed)
    os.replace(temporary, output)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Add type-preserving native runtime calls to Kalibr Python"
    )
    parser.add_argument("--source", choices=SUPPORTED_SOURCES, required=True)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output")
    arguments = parser.parse_args(argv)
    transform_file(arguments.input, arguments.source, arguments.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
