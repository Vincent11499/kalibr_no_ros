"""Translate an initialized upstream IccCalibrator into immutable Ceres data."""

import multiprocessing

import incremental_calibration as inc
import numpy as np
import sm

from .libkalibr_ceres_optimizer_python import (
    set_design_variable_parameters,
    solve_camera_bundle,
    solve_joint,
)


def _ceres_thread_count(requested):
    requested = int(requested)
    if requested < 0:
        raise ValueError("Ceres thread count must be non-negative")
    if requested == 0:
        return max(1, multiprocessing.cpu_count() - 1)
    return requested


def _column(values):
    return np.asarray(values, dtype=np.float64).reshape((-1, 1))


def _set(variable, values):
    set_design_variable_parameters(variable, _column(values))


def _gravity_parameters(vector):
    vector = np.asarray(vector, dtype=np.float64)
    unit = vector / np.linalg.norm(vector)
    helper = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(helper, unit)) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    first = helper - unit * np.dot(helper, unit)
    first /= np.linalg.norm(first)
    second = np.cross(unit, first)
    return np.column_stack((first, second, unit))


def initialize_problem(
    calibrator,
    spline_order=6,
    pose_knots_per_second=100,
    bias_knots_per_second=50,
    no_time_calibration=False,
    no_chain_extrinsics=True,
    time_offset_padding=0.03,
    verbose=False,
):
    """Run Kalibr's native initialization without creating its error graph."""
    if len(calibrator.ImuList) != 1:
        raise RuntimeError("Ceres CLI bridge currently supports one reference IMU")
    if not no_chain_extrinsics:
        raise RuntimeError(
            "Ceres CLI bridge does not yet recompute camera-chain baselines"
        )
    calibrator.noTimeCalibration = bool(no_time_calibration)
    if not no_time_calibration:
        for camera in calibrator.CameraChain.camList:
            camera.findTimeshiftCameraImuPrior(calibrator.ImuList[0], verbose)
    calibrator.CameraChain.findOrientationPriorCameraChainToImu(
        calibrator.ImuList[0]
    )
    gravity = calibrator.CameraChain.getEstimatedGravity()
    pose_spline = calibrator.CameraChain.initializePoseSplineFromCameraChain(
        spline_order, pose_knots_per_second, time_offset_padding
    )
    for imu in calibrator.ImuList:
        imu.initBiasSplines(pose_spline, spline_order, bias_knots_per_second)
    design_variable_problem = inc.CalibrationOptimizationProblem()
    calibrator.initDesignVariables(
        design_variable_problem,
        pose_spline,
        no_time_calibration,
        no_chain_extrinsics,
        initialGravityEstimate=gravity,
    )
    # Keep the lightweight owner alive because the native design variables are
    # passed into the C++ bridge. It intentionally contains no error terms.
    calibrator.problem = design_variable_problem


class _ResidualAdapter:
    def __init__(self, error, normalized_error, measurement=None):
        self._error = np.asarray(error, dtype=np.float64)
        self._normalized_error = np.asarray(normalized_error, dtype=np.float64)
        self._measurement = None if measurement is None else np.asarray(
            measurement, dtype=np.float64
        )

    def error(self):
        return self._error

    def evaluateError(self):
        return float(np.dot(self._normalized_error, self._normalized_error))

    def getMeasurement(self):
        return self._measurement

    def getPredictedMeasurement(self):
        return self._measurement + self._error


def _install_final_residuals(calibrator, report):
    imu = calibrator.ImuList[0]
    gyro_error = np.asarray(report["gyroscope_errors"])
    gyro_normalized = np.asarray(report["gyroscope_normalized_errors"])
    gyro_measurement = np.asarray(report["gyroscope_measurements"])
    accel_error = np.asarray(report["accelerometer_errors"])
    accel_normalized = np.asarray(report["accelerometer_normalized_errors"])
    accel_measurement = np.asarray(report["accelerometer_measurements"])
    imu.gyroErrors = [
        _ResidualAdapter(error, normalized, measurement)
        for error, normalized, measurement in zip(
            gyro_error, gyro_normalized, gyro_measurement
        )
    ]
    imu.accelErrors = [
        _ResidualAdapter(error, normalized, measurement)
        for error, normalized, measurement in zip(
            accel_error, accel_normalized, accel_measurement
        )
    ]

    groups = []
    source_to_group = {}
    source_index = 0
    for camera in calibrator.CameraChain.camList:
        camera_groups = []
        for observation in camera.targetObservations:
            group = []
            camera_groups.append(group)
            image = np.asarray(observation.getCornersImageFrame())
            corner_count = image.shape[1] if image.shape[0] == 2 else image.shape[0]
            for _ in range(corner_count):
                source_to_group[source_index] = group
                source_index += 1
        camera.allReprojectionErrors = camera_groups
        groups.append(camera_groups)
    raw = np.asarray(report["camera_errors"])
    normalized = np.asarray(report["camera_normalized_errors"])
    for raw_row, normalized_row in zip(raw, normalized):
        group = source_to_group[int(raw_row[0])]
        group.append(_ResidualAdapter(raw_row[2:4], normalized_row[2:4]))


def print_residual_statistics(report, dest=None):
    """Print Kalibr-style aggregate statistics from lightweight Ceres data."""
    import sys

    if dest is None:
        dest = sys.stdout

    def stats(values):
        norms = np.linalg.norm(np.asarray(values), axis=1)
        return np.mean(norms), np.median(norms), np.std(norms)

    camera_raw = np.asarray(report["camera_errors"])
    camera_normalized = np.asarray(report["camera_normalized_errors"])
    print("Normalized Residuals\n----------------------------", file=dest)
    for camera_index in np.unique(camera_raw[:, 1].astype(int)):
        values = camera_normalized[camera_normalized[:, 1] == camera_index, 2:4]
        print(
            "Reprojection error (cam{0}):     mean {1}, median {2}, std: {3}".format(
                camera_index, *stats(values)
            ),
            file=dest,
        )
    print(
        "Gyroscope error (imu0):        mean {0}, median {1}, std: {2}".format(
            *stats(report["gyroscope_normalized_errors"])
        ),
        file=dest,
    )
    print(
        "Accelerometer error (imu0):    mean {0}, median {1}, std: {2}".format(
            *stats(report["accelerometer_normalized_errors"])
        ),
        file=dest,
    )
    print("\nResiduals\n----------------------------", file=dest)
    for camera_index in np.unique(camera_raw[:, 1].astype(int)):
        values = camera_raw[camera_raw[:, 1] == camera_index, 2:4]
        print(
            "Reprojection error (cam{0}) [px]:     mean {1}, median {2}, std: {3}".format(
                camera_index, *stats(values)
            ),
            file=dest,
        )
    print(
        "Gyroscope error (imu0) [rad/s]:     mean {0}, median {1}, std: {2}".format(
            *stats(report["gyroscope_errors"])
        ),
        file=dest,
    )
    print(
        "Accelerometer error (imu0) [m/s^2]: mean {0}, median {1}, std: {2}".format(
            *stats(report["accelerometer_errors"])
        ),
        file=dest,
    )


def _camera_data(calibrator):
    rows = []
    transforms = []
    cumulative = np.eye(4)
    for index, camera in enumerate(calibrator.CameraChain.camList):
        if index > 0:
            cumulative = np.asarray(camera.T_c_b_Dv.T()) @ cumulative
        projection = camera.camera.geometry.projection()
        intrinsics = np.asarray(projection.getParameters()).reshape(-1)
        distortion = np.asarray(
            projection.distortion().getParameters(), dtype=np.float64
        ).reshape(-1)
        if intrinsics.size != 4 or distortion.size not in (4, 5):
            raise RuntimeError(
                "Ceres IMU-camera path currently supports only pinhole-radtan/radtan5"
            )
        padded_distortion = np.zeros(5, dtype=np.float64)
        padded_distortion[: distortion.size] = distortion
        rows.append(
            np.concatenate(
                (
                    intrinsics,
                    padded_distortion,
                    [
                        float(distortion.size),
                        float(camera.timeshiftCamToImuPrior),
                        float(camera.cornerUncertainty),
                    ],
                )
            )
        )
        transforms.append(cumulative.copy())
    return np.asarray(rows, dtype=np.float64), np.vstack(transforms)


def _corners(calibrator):
    rows = []
    for camera_index, camera in enumerate(calibrator.CameraChain.camList):
        for observation in camera.targetObservations:
            image = np.asarray(observation.getCornersImageFrame(), dtype=np.float64)
            target = np.asarray(observation.getCornersTargetFrame(), dtype=np.float64)
            if image.shape[0] == 2:
                image = image.T
            if target.shape[0] == 3:
                target = target.T
            if image.shape[0] != target.shape[0]:
                raise RuntimeError("AprilGrid image/target corner counts differ")
            timestamp = observation.time().toSec()
            for target_point, image_point in zip(target, image):
                rows.append(
                    [timestamp, camera_index]
                    + target_point[:3].tolist()
                    + image_point[:2].tolist()
                )
    return np.asarray(rows, dtype=np.float64).reshape((-1, 7))


def _imu_data(imu):
    rows = []
    for measurement in imu.imuData:
        rows.append(
            [measurement.stamp.toSec() + imu.timeOffset]
            + np.asarray(measurement.omega).reshape(3).tolist()
            + np.asarray(measurement.alpha).reshape(3).tolist()
        )
    return np.asarray(rows, dtype=np.float64).reshape((-1, 7))


def _intrinsics(imu, model):
    if model == "calibrated":
        return np.empty((0, 1), dtype=np.float64)
    values = [
        np.asarray(imu.M_accel_Dv.toMatrix3x3()).reshape(-1),
        np.asarray(imu.q_gyro_i_Dv.toRotationMatrix()).reshape(-1),
        np.asarray(imu.M_gyro_Dv.toMatrix3x3()).reshape(-1),
        np.asarray(imu.M_accel_gyro_Dv.toMatrix3x3()).reshape(-1),
    ]
    if model == "scale-misalignment-size-effect":
        values.extend(
            (
                np.asarray(imu.rx_i_Dv.toEuclidean()).reshape(-1),
                np.asarray(imu.ry_i_Dv.toEuclidean()).reshape(-1),
                np.asarray(imu.rz_i_Dv.toEuclidean()).reshape(-1),
            )
        )
    return _column(np.concatenate(values))


def _apply_result(calibrator, result, model):
    set_design_variable_parameters(
        calibrator.gravityDv, _gravity_parameters(result["gravity"])
    )
    transform = np.asarray(result["camera0_from_imu"])
    camera0 = calibrator.CameraChain.camList[0]
    _set(camera0.T_c_b_Dv.q, sm.r2quat(transform[:3, :3]))
    _set(camera0.T_c_b_Dv.t, transform[:3, 3])
    offsets = np.asarray(result["time_offsets"]).reshape(-1)
    for camera, offset in zip(calibrator.CameraChain.camList, offsets):
        _set(
            camera.cameraTimeToImuTimeDv,
            [float(offset) - float(camera.timeshiftCamToImuPrior)],
        )
    imu = calibrator.ImuList[0]
    if model != "calibrated":
        set_design_variable_parameters(imu.M_accel_Dv, result["M_accel"])
        _set(imu.q_gyro_i_Dv, sm.r2quat(result["C_gyro_i"]))
        set_design_variable_parameters(imu.M_gyro_Dv, result["M_gyro"])
        set_design_variable_parameters(imu.M_accel_gyro_Dv, result["A"])
    if model == "scale-misalignment-size-effect":
        _set(imu.rx_i_Dv, result["rx"])
        _set(imu.ry_i_Dv, result["ry"])
        _set(imu.rz_i_Dv, result["rz"])


def optimize(
    calibrator,
    model,
    max_iterations=30,
    recover_covariance=False,
    estimate_time_offsets=True,
    time_offset_padding=0.03,
    verbose=False,
    num_threads=0,
):
    if len(calibrator.ImuList) != 1:
        raise RuntimeError("Ceres CLI bridge currently supports one reference IMU")
    if recover_covariance:
        raise RuntimeError("Ceres covariance recovery is not connected yet")
    imu = calibrator.ImuList[0]
    imu_matrix = _imu_data(imu)
    first_measurement = imu.imuData[0]
    cameras, camera_transforms = _camera_data(calibrator)
    corners = _corners(calibrator)
    initial_transform = np.asarray(calibrator.CameraChain.camList[0].T_c_b_Dv.T())
    options = {
        "max_iterations": int(max_iterations),
        "num_threads": _ceres_thread_count(num_threads),
        "verbose": bool(verbose),
        "estimate_time_offsets": bool(estimate_time_offsets),
        "time_offset_padding": float(time_offset_padding),
        "gyroscope_random_walk": float(imu.gyroRandomWalk),
        "accelerometer_random_walk": float(imu.accelRandomWalk),
    }
    result = solve_joint(
        calibrator.poseDv,
        imu.gyroBiasDv,
        imu.accelBiasDv,
        imu_matrix,
        np.asarray(first_measurement.omegaInvR),
        np.asarray(first_measurement.alphaInvR),
        np.asarray(calibrator.gravityDv.toEuclidean()),
        cameras,
        camera_transforms,
        corners,
        initial_transform,
        model,
        _intrinsics(imu, model),
        options,
    )
    _apply_result(calibrator, result, model)
    _install_final_residuals(calibrator, result["final_report"])
    return result


def _camera_bundle_data(calibrator):
    camera_rows = []
    for camera in calibrator.cameras:
        projection = camera.geometry.projection()
        intrinsics = np.asarray(
            projection.getParameters(), dtype=np.float64
        ).reshape(-1)
        distortion = np.asarray(
            projection.distortion().getParameters(), dtype=np.float64
        ).reshape(-1)
        if intrinsics.size != 4 or distortion.size not in (4, 5):
            raise RuntimeError(
                "Ceres camera BA currently supports only pinhole-radtan/radtan5"
            )
        padded = np.zeros(5, dtype=np.float64)
        padded[: distortion.size] = distortion
        camera_rows.append(
            np.concatenate((intrinsics, padded, [float(distortion.size)]))
        )

    baseline_matrices = [
        np.asarray(baseline.T(), dtype=np.float64)
        for baseline in calibrator.baselines
    ]
    if baseline_matrices:
        baseline_matrix = np.vstack(baseline_matrices)
    else:
        baseline_matrix = np.empty((0, 4), dtype=np.float64)

    view_matrices = []
    corner_rows = []
    for view_index, view in enumerate(calibrator.views):
        target_from_camera0 = np.asarray(
            view.dv_T_target_camera.T(), dtype=np.float64
        )
        view_matrices.append(np.linalg.inv(target_from_camera0))
        for camera_index, observation in view.rig_observations:
            image = np.asarray(
                observation.getCornersImageFrame(), dtype=np.float64
            )
            target = np.asarray(
                observation.getCornersTargetFrame(), dtype=np.float64
            )
            if image.shape[0] == 2:
                image = image.T
            if target.shape[0] == 3:
                target = target.T
            if image.shape[0] != target.shape[0]:
                raise RuntimeError("Camera BA image/target corner counts differ")
            for target_point, image_point in zip(target, image):
                corner_rows.append(
                    [view_index, camera_index]
                    + target_point[:3].tolist()
                    + image_point[:2].tolist()
                )
    return (
        np.asarray(camera_rows, dtype=np.float64).reshape((-1, 10)),
        baseline_matrix,
        np.vstack(view_matrices),
        np.asarray(corner_rows, dtype=np.float64).reshape((-1, 7)),
    )


def _apply_camera_bundle_result(calibrator, result):
    camera_parameters = np.asarray(result["cameras"], dtype=np.float64)
    for camera, row in zip(calibrator.cameras, camera_parameters):
        count = int(round(row[9]))
        _set(camera.dv.projectionDesignVariable(), row[:4])
        _set(camera.dv.distortionDesignVariable(), row[4 : 4 + count])

    baseline_matrices = np.asarray(result["baselines"], dtype=np.float64)
    for index, baseline in enumerate(calibrator.baselines):
        transform = baseline_matrices[4 * index : 4 * index + 4, :]
        _set(baseline.q, sm.r2quat(transform[:3, :3]))
        _set(baseline.t, transform[:3, 3])

    view_matrices = np.asarray(result["views"], dtype=np.float64)
    for index, view in enumerate(calibrator.views):
        camera0_from_target = view_matrices[4 * index : 4 * index + 4, :]
        target_from_camera0 = np.linalg.inv(camera0_from_target)
        _set(view.dv_T_target_camera.q, sm.r2quat(target_from_camera0[:3, :3]))
        _set(view.dv_T_target_camera.t, target_from_camera0[:3, 3])


def optimize_camera_bundle(
    calibrator,
    max_iterations=50,
    use_blake_zisserman=False,
    verbose=False,
    num_threads=0,
):
    """Refine Kalibr's accepted camera views with a parallel Ceres BA."""
    if use_blake_zisserman:
        raise RuntimeError(
            "Ceres camera BA does not yet implement Kalibr Blake-Zisserman loss"
        )
    native_initial_energy = sum(
        error.evaluateError()
        for view in calibrator.views
        for camera_errors in view.rerrs.values()
        for error in camera_errors
        if error is not None
    )
    cameras, baselines, views, corners = _camera_bundle_data(calibrator)
    options = {
        "max_iterations": int(max_iterations),
        "num_threads": _ceres_thread_count(num_threads),
        "verbose": bool(verbose),
    }
    result = solve_camera_bundle(
        cameras, baselines, views, corners, options
    )
    relative_initial_error = abs(
        native_initial_energy - 2.0 * float(result["initial_cost"])
    ) / max(1.0, abs(native_initial_energy))
    # The legacy ErrorTerm path and the exported homogeneous matrices differ
    # at the last few floating-point operations (notably quaternion-to-matrix
    # evaluation). Real EuRoC runs agree within about 1.2e-5 relatively.
    if relative_initial_error > 1.0e-4:
        raise RuntimeError(
            "Kalibr/Ceres camera objective mismatch: relative error {0}".format(
                relative_initial_error
            )
        )
    _apply_camera_bundle_result(calibrator, result)
    native_final_energy = sum(
        error.evaluateError()
        for view in calibrator.views
        for camera_errors in view.rerrs.values()
        for error in camera_errors
        if error is not None
    )
    relative_final_error = abs(
        native_final_energy - 2.0 * float(result["final_cost"])
    ) / max(1.0, abs(native_final_energy))
    if relative_final_error > 1.0e-4:
        raise RuntimeError(
            "Ceres result/Kalibr camera objective mismatch: relative error {0}".format(
                relative_final_error
            )
        )
    result["kalibr_initial_energy"] = native_initial_energy
    result["kalibr_final_energy"] = native_final_energy
    result["initial_objective_relative_error"] = relative_initial_error
    result["final_objective_relative_error"] = relative_final_error
    return result
