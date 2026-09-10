"""Read-only adapters from native solver state to portable run evidence.

The active context belongs to the public task runner. Native hooks do not
write files, select data, or evaluate new optimization problems. Only the
finished native residuals are evaluated when publishing a result.
"""

from contextlib import contextmanager
from contextvars import ContextVar
import os

import numpy as np

from .version import SCHEMA_VERSION


_ACTIVE = ContextVar("kalibr_run_artifacts", default=None)


def current_context():
    context = _ACTIVE.get()
    # Detector children must never mutate their inherited parent collector.
    return context if context is not None and context.pid == os.getpid() else None


def managed_output_enabled():
    return current_context() is not None


@contextmanager
def run_context(calibration_type, capture_history=False):
    """Collect one run; restore nested contexts even on SystemExit/failure."""
    context = RunContext(calibration_type, capture_history=capture_history)
    token = _ACTIVE.set(context)
    try:
        yield context
    except BaseException as error:
        context.artifacts["state"] = "failed"
        context.artifacts["failure"] = {
            "type": type(error).__name__, "message": str(error),
        }
        raise
    finally:
        if context.artifacts["state"] == "running":
            context.artifacts["state"] = "incomplete"
        context.release_native_objects()
        _ACTIVE.reset(token)


def _array(value):
    return np.asarray(value, dtype=float).tolist()


def _vector(value):
    return np.asarray(value, dtype=float).reshape(-1).tolist()


def _corner(observation, corner_id):
    valid, point = observation.imagePoint(int(corner_id))
    if not valid:
        raise ValueError("cannot snapshot a missing target corner")
    return {
        "corner_id": int(corner_id),
        "target_xyz_m": _vector(observation.target().point(int(corner_id))),
        "measurement_px": _vector(point),
        "prediction_px": None,
        "residual_px": None,
        "used": False,
    }


def _optimizer_summary(result, scope):
    # IncrementalEstimator and Optimizer2 expose different return types. In
    # particular, an incremental candidate can be rolled back after its solve.
    summary = {
        "scope": scope,
        "return_value_status": "available" if result is not None else "unavailable",
        "stop_reason": "unavailable",
        "stopping_thresholds": "unavailable",
        "unavailable_reason": "native return values do not identify the stopping condition; optimizer thresholds were not captured",
    }
    for name in ("numIterations", "iterations", "failedIterations",
                 "linearSolverFailure", "batchAccepted", "dXFinal", "dJFinal",
                 "JStart", "JFinal", "informationGain"):
        value = getattr(result, name, None)
        if isinstance(value, (bool, int, float)):
            summary[name] = value if np.isfinite(value) else None
    if scope == "last_attempt":
        summary["objective_terms"] = "last_incremental_candidate_problem_before_acceptance_or_rollback"
    else:
        summary["objective_terms"] = "all_native_joint_problem_terms_including_enabled_bias_motion_priors"
        summary["measurement_residual_sum_is_full_objective"] = False
    return summary


def _normalized_error(error, residual):
    """Export noise normalization separately from physical and robust errors."""
    try:
        precision = np.asarray(error.invR(), dtype=float)
        whitened = np.linalg.cholesky(precision).T @ np.asarray(residual, dtype=float)
        return {
            "whitened_residual": _vector(whitened),
            "whitened_squared_error": float(np.dot(whitened, whitened)),
            "robust_weight": float(error.getCurrentMEstimatorWeight()),
            "weighted_squared_error": float(error.getWeightedSquaredError()),
        }
    except (AttributeError, ValueError, RuntimeError, np.linalg.LinAlgError) as error:
        return {"whitened_residual": None, "whitened_squared_error": None,
                "robust_weight": None, "weighted_squared_error": None,
                "normalization_unavailable": str(error)}


def _bias_sample(imu, kind, source, timestamp):
    sample = {"timestamp_ns": source["source_timestamp_ns"],
              "source_index": source["source_index"],
              "solver_timestamp_s": timestamp, "value": None}
    try:
        # The DV owns the optimized spline. The initialization spline on imu
        # is a different object and does not contain the final coefficients.
        variable = getattr(imu, "gyroBiasDv" if kind == "gyro" else "accelBiasDv")
        # This native getter calls the owned spline's evalD(t, 0) directly,
        # avoiding a complete spline copy for every retained measurement.
        value = np.asarray(variable.toEuclidean(timestamp, 0), dtype=float).reshape(-1)
        if value.size != 3 or not np.all(np.isfinite(value)):
            raise ValueError("bias spline must evaluate to three finite components")
        sample["value"] = _vector(value)
    except (AttributeError, ValueError, RuntimeError) as error:
        sample["unavailable_reason"] = str(error)
    return sample


class RunContext:
    def __init__(self, calibration_type, capture_history=False):
        calibration_type = {
            "camera_calibration": "cameras",
            "camera_imu_calibration": "camera_imu",
        }.get(calibration_type, calibration_type)
        self.pid = os.getpid()
        self.capture_history = bool(capture_history)
        self.artifacts = {
            "schema_version": SCHEMA_VERSION,
            "kind": "run_artifacts",
            "calibration_type": calibration_type,
            "state": "running",
            "residual_convention": "measurement - prediction",
            "transform_convention": "p_target = T_target_source * p_source",
            "history_captured": self.capture_history,
            "cameras": [], "views": [], "events": [], "imu_residuals": [],
            "imu_biases": [],
        }
        self._datasets = {}
        self._frames = {}
        self._observations = {}
        self._view_keys = {}
        self._camera_terms = []
        self._imu_sources = {}
        self._imu_terms = []
        self._solver_summary = None

    def release_native_objects(self):
        self._datasets.clear()
        self._observations.clear()
        self._camera_terms.clear()
        self._imu_sources.clear()
        self._imu_terms.clear()

    def register_dataset(self, dataset):
        """Capture selected source indices before the extractor consumes them."""
        key = id(dataset)
        if key in self._datasets:
            return self._datasets[key]
        camera_id = "cam{}".format(len(self.artifacts["cameras"]))
        camera = {"id": camera_id, "topic": dataset.topic, "frames": [],
                  "source_frame_count": len(dataset.index),
                  "selected_frame_count": len(dataset.indices)}
        self.artifacts["cameras"].append(camera)
        indices = [int(index) for index in dataset.indices]
        entry = (dataset, camera, indices)
        self._datasets[key] = entry
        for source_index in indices:
            metadata = dataset.source_metadata(source_index)
            frame = dict(metadata)
            frame.update({
                "frame_id": "{}:{}".format(camera_id, source_index),
                "detection_status": "not_processed", "used": False,
                "view_id": None, "T_camera_target": None,
                "corners": [], "detected_corner_count": 0,
            })
            camera["frames"].append(frame)
            self._frames[(key, source_index)] = frame
        return entry

    def record_detection(self, dataset, submitted_index, observation):
        _, camera, indices = self.register_dataset(dataset)
        frame = self._frames[(id(dataset), indices[int(submitted_index)])]
        if observation is None:
            frame["detection_status"] = "failed"
            return
        frame["detection_status"] = "succeeded"
        frame["observation_timestamp_ns"] = int(observation.time().toNSec())
        frame["resolution"] = [int(observation.imCols()), int(observation.imRows())]
        camera.setdefault("resolution", frame["resolution"])
        corner_ids = [int(value) for value in observation.getCornersIdx()]
        frame["detected_corner_count"] = len(corner_ids)
        if self.capture_history:
            frame["corners"] = [_corner(observation, index) for index in corner_ids]
        # Keep the object alive: Python ids must not be reused during this run.
        self._observations[id(observation)] = (observation, frame)

    def frame(self, observation):
        try:
            return self._observations[id(observation)][1]
        except KeyError as error:
            raise RuntimeError("solver observation has no recorded source frame") from error

    def view(self, timestamp, observations):
        frame_ids = [self.frame(obs)["frame_id"] for _, obs in observations]
        key = tuple(sorted(frame_ids))
        if key not in self._view_keys:
            record = {"view_id": "view{}".format(len(self._view_keys)),
                      "solver_timestamp_s": float(timestamp),
                      "frame_ids": frame_ids, "used": False}
            self._view_keys[key] = record
            self.artifacts["views"].append(record)
        return self._view_keys[key]

    def record_view(self, timestamp, observations, accepted, stage, result=None):
        view = self.view(timestamp, observations)
        view["used"] = bool(accepted)
        self._solver_summary = _optimizer_summary(result, "last_attempt")
        if self.capture_history:
            self.artifacts["events"].append({
                "kind": "view_decision", "view_id": view["view_id"],
                "stage": stage, "accepted": bool(accepted),
                "reason": None if accepted else "{}_rejected".format(stage),
                "optimizer": self._solver_summary,
            })

    def record_removed_corners(self, batch, camera_id, corner_ids, thresholds):
        if not self.capture_history:
            return
        observations = dict(batch.rig_observations)
        frame = self.frame(observations[camera_id])
        for corner_id in corner_ids:
            error = batch.rerrs[camera_id][corner_id]
            self.artifacts["events"].append({
                "kind": "corner_removed", "reason": "reprojection_outlier",
                "frame_id": frame["frame_id"], "corner_id": int(corner_id),
                "residual_at_removal_px": _vector(
                    error.getMeasurement() - error.getPredictedMeasurement()),
                "axis_thresholds_px": _vector(thresholds),
            })

    def record_camera_terms(self, camera, observation, errors, transform, time_expression):
        self._camera_terms.append((camera, observation, list(errors), transform,
                                   time_expression,
                                   [int(i) for i in observation.getCornersIdx()]))

    def record_camera_skipped(self, observation, reason):
        frame = self.frame(observation)
        frame["exclusion_reason"] = reason
        if self.capture_history:
            self.artifacts["events"].append({
                "kind": "frame_excluded", "frame_id": frame["frame_id"],
                "reason": reason,
            })

    def record_imu_sources(self, imu):
        if len(imu.imuData) != len(imu.dataset.indices):
            raise RuntimeError("IMU input index and native measurement counts differ")
        for source_index, measurement in zip(imu.dataset.indices, imu.imuData):
            self._imu_sources[id(measurement)] = (
                measurement, imu.dataset.source_metadata(int(source_index)))

    def record_imu_term(self, imu, measurement, kind, error):
        self._imu_terms.append((imu, measurement, kind, error))

    def record_optimizer(self, result):
        self._solver_summary = _optimizer_summary(result, "final_joint_optimization")

    @staticmethod
    def _final_corners(frame, observation, indexed_errors, simple=False):
        corners = {corner["corner_id"]: corner for corner in frame["corners"]}
        for corner_id, error in indexed_errors:
            if error is None:
                continue
            corner = corners.setdefault(int(corner_id), _corner(observation, corner_id))
            # Native error() is cached; explicitly evaluate at the final state.
            error.evaluateError()
            measurement = np.asarray(corner["measurement_px"], dtype=float)
            if simple:
                residual = np.asarray(error.error(), dtype=float).reshape(-1)
                prediction = measurement - residual
            else:
                prediction = np.asarray(error.getPredictedMeasurement(), dtype=float).reshape(-1)
                residual = measurement - prediction
            corner.update({"prediction_px": _vector(prediction),
                           "residual_px": _vector(residual), "used": True})
            corner.update(_normalized_error(error, residual))
        frame["corners"] = [corners[index] for index in sorted(corners)]

    def publish_camera(self, calibrator, models):
        for camera_id, native in enumerate(calibrator.cameras):
            camera = self._datasets[id(native.dataset)][1]
            projection = native.geometry.projection()
            camera.update({
                "model": models[camera_id],
                "intrinsics": _vector(projection.getParameters()),
                "distortion_coeffs": _vector(projection.distortion().getParameters()),
                "resolution": [int(projection.ru()), int(projection.rv())],
            })
            if camera_id:
                camera["T_cn_cnm1"] = _array(calibrator.baselines[camera_id - 1].T())
        for view in self.artifacts["views"]:
            view["used"] = False
        for batch in calibrator.views:
            view = self.view(batch.timestamp, batch.rig_observations)
            view["used"] = True
            T_camera_target = np.linalg.inv(np.asarray(batch.dv_T_target_camera.T(), dtype=float))
            for camera_id, observation in batch.rig_observations:
                transform = T_camera_target.copy()
                for index in range(camera_id):
                    transform = np.asarray(calibrator.baselines[index].T(), dtype=float) @ transform
                frame = self.frame(observation)
                frame.update({"used": any(error is not None for error in batch.rerrs[camera_id]),
                              "view_id": view["view_id"],
                              "T_camera_target": _array(transform)})
                self._final_corners(frame, observation, enumerate(batch.rerrs[camera_id]))
        final_corners = [corner for camera in self.artifacts["cameras"]
                         for frame in camera["frames"] if frame["used"]
                         for corner in frame["corners"] if corner["used"]]
        values = [corner.get("weighted_squared_error") for corner in final_corners]
        unavailable = sum(value is None for value in values)
        self.artifacts["objective"] = {
            "scope": "final_retained_camera_views",
            "final_camera_weighted_residual_sum": None if unavailable else float(sum(values)),
            "used_error_term_count": len(values),
            "unavailable_error_term_count": unavailable,
            "definition": "sum(weight(s) * s), s = residual^T * invR * residual; no factor of one half",
        }
        self.artifacts["optimizer"] = self._solver_summary or _optimizer_summary(None, "last_attempt")
        self.artifacts["state"] = "completed"

    def publish_imu_camera(self, calibrator):
        chain = calibrator.CameraChain
        for camera_id, native in enumerate(chain.camList):
            camera = self._datasets[id(native.dataset)][1]
            camera_model, intrinsics = native.camConfig.getIntrinsics()
            distortion_model, distortion = native.camConfig.getDistortion()
            model = ("pinhole-opencv-fisheye" if camera_model == "pinhole_opencv_fisheye"
                     else "{}-{}".format(camera_model, "equi" if distortion_model == "equidistant" else distortion_model))
            camera.update({
                "model": model, "intrinsics": _vector(intrinsics),
                "distortion_coeffs": _vector(distortion),
                "resolution": [int(v) for v in native.camConfig.getResolution()],
                "T_cam_imu": _array(chain.getResultTrafoImuToCam(camera_id).T()),
                "timeshift_cam_imu_s": float(chain.getResultTimeShift(camera_id)),
            })
            if camera_id:
                camera["T_cn_cnm1"] = _array(chain.getResultBaseline(camera_id - 1, camera_id)[0].T())
        for camera, observation, errors, transform, time_expression, corner_ids in self._camera_terms:
            if len(corner_ids) != len(errors):
                raise RuntimeError("camera corner IDs and native residual counts differ")
            frame = self.frame(observation)
            frame.update({"used": bool(errors), "solver_timestamp_s": float(time_expression.toScalar()),
                          "T_camera_target": _array(transform.toTransformationMatrix())})
            self._final_corners(frame, observation, zip(corner_ids, errors), simple=True)
        imu_ids = {id(imu): "imu{}".format(index) for index, imu in enumerate(calibrator.ImuList)}
        bias_series = {}
        for imu in calibrator.ImuList:
            for kind, units in (("gyro", "rad/s"), ("accel", "m/s^2")):
                series = {"imu_id": imu_ids[id(imu)], "kind": kind, "units": units,
                          "representation": "time_varying_spline",
                          "sampling": "retained_imu_residual_timestamps", "samples": []}
                bias_series[(id(imu), kind)] = series
                self.artifacts["imu_biases"].append(series)
        for imu, measurement, kind, error in self._imu_terms:
            _, source = self._imu_sources[id(measurement)]
            error.evaluateError()
            # Kalibr's EuclideanError uses prediction - measurement, opposite
            # to its camera errors. Normalize only the exported representation.
            residual = -np.asarray(error.error(), dtype=float).reshape(-1)
            measured = np.asarray(measurement.omega if kind == "gyro" else measurement.alpha).reshape(-1)
            record = dict(source)
            record.update({
                "imu_id": imu_ids[id(imu)], "kind": kind,
                "timestamp_ns": source["source_timestamp_ns"],
                "solver_timestamp_s": float(measurement.stamp.toSec() + imu.timeOffset),
                "measurement": _vector(measured), "prediction": _vector(measured - residual),
                "residual": _vector(residual), "units": "rad/s" if kind == "gyro" else "m/s^2",
            })
            record.update(_normalized_error(error, residual))
            self.artifacts["imu_residuals"].append(record)
            bias_series[(id(imu), kind)]["samples"].append(
                _bias_sample(imu, kind, source, record["solver_timestamp_s"]))
        self.artifacts["optimizer"] = self._solver_summary or _optimizer_summary(None, "final_joint_optimization")
        self.artifacts["state"] = "completed"


def record_detection(dataset, submitted_index, observation):
    context = current_context()
    if context is not None:
        context.record_detection(dataset, submitted_index, observation)


def publish_camera(calibrator, models):
    context = current_context()
    if context is not None:
        context.publish_camera(calibrator, models)


def publish_imu_camera(calibrator):
    context = current_context()
    if context is not None:
        context.publish_imu_camera(calibrator)


def record_imu_term(imu, measurement, kind, error):
    context = current_context()
    if context is not None:
        context.record_imu_term(imu, measurement, kind, error)
