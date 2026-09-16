"""Joint mono and multi-camera rolling-shutter calibration.

This module deliberately keeps the rolling-shutter time state separate from
the native camera shutter design variable.  The physical line delay is the
smooth bounded expression ``bound * tanh(latent)`` and the camera timestamp is
the time of row zero::

    t_corner = t_camera + y * line_delay

The target trajectory, camera intrinsics/distortion, adjacent camera
transforms and one line delay per camera are optimized in one native Kalibr
problem.  A weak second-order spline prior regularizes motion between the
sparsely sampled target poses.
"""

from __future__ import print_function

import math
from types import SimpleNamespace

import numpy as np

import aslam_backend as aopt
import aslam_splines as asp
import bsplines
import incremental_calibration as inc
import kalibr_runtime as native_runtime
import sm


CALIBRATION_GROUP_ID = 0
TRAJECTORY_GROUP_ID = 1
LANDMARK_GROUP_ID = 2


def _finite(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise RuntimeError("{} must be a number, not boolean".format(name))
    value = float(value)
    if not math.isfinite(value):
        raise RuntimeError("{} must be finite".format(name))
    return value


def _continuous_rotation_vectors(curve):
    """Modify one 6xN pose curve so rotation vectors do not wrap at pi."""
    for index in range(1, curve.shape[1]):
        previous = np.asarray(curve[3:6, index - 1], dtype=float).reshape(3)
        current = np.asarray(curve[3:6, index], dtype=float).reshape(3)
        angle = float(np.linalg.norm(current))
        if angle < 1e-15:
            continue
        axis = current / angle
        candidates = [axis * (angle + 2.0 * math.pi * turn)
                      for turn in range(-3, 4)]
        best = min(candidates, key=lambda value: np.linalg.norm(value - previous))
        curve[3:6, index] = np.asarray(best).reshape(3, 1)


def make_pose_spline(pose_samples, order=4, padding_s=0.5,
                     knots_per_second=None, selected_timestamps=None,
                     support_timestamps=None):
    """Initialize a pose spline and return it with deterministic metadata.

    This solver does not adaptively insert knots.  Its automatic rate therefore
    follows the median rate of the observations that actually entered the
    solve, with at least two spline orders of segments.  Segment count covers
    the padded spline domain and may therefore exceed the number of initialized
    poses.  An explicit rate remains available for trajectory-model
    sensitivity runs.
    """
    order = int(order)
    if order < 3:
        raise RuntimeError(
            "spline order must be at least 3 for the second-order motion prior")
    padding_s = _finite(padding_s, "time padding")
    if padding_s <= 0.0:
        raise RuntimeError("time padding must be positive")

    ordered = sorted(pose_samples, key=lambda item: float(item[0]))
    unique = []
    for timestamp, transform in ordered:
        timestamp = _finite(timestamp, "pose timestamp")
        if unique and abs(timestamp - unique[-1][0]) <= 1e-12:
            continue
        unique.append((timestamp, transform))
    if len(unique) < max(4, order):
        raise RuntimeError(
            "rolling-shutter calibration requires at least {} initialized "
            "target poses".format(max(4, order)))

    spline = bsplines.BSplinePose(order, sm.RotationVector())
    times = np.asarray([item[0] for item in unique], dtype=float)
    curve = np.matrix([
        spline.transformationToCurveValue(item[1].T()) for item in unique
    ]).T
    if not np.all(np.isfinite(curve)):
        raise RuntimeError("initial target poses contain non-finite values")

    def finite_unique_timestamps(values, name):
        ordered_values = sorted(_finite(value, name) for value in values)
        unique_values = []
        for value in ordered_values:
            if (not unique_values or
                    abs(value - unique_values[-1]) > 1e-12):
                unique_values.append(value)
        return np.asarray(unique_values, dtype=float)

    rate_times = (times if selected_timestamps is None else
                  finite_unique_timestamps(
                      selected_timestamps, "selected observation timestamp"))
    if rate_times.size < 2:
        raise RuntimeError(
            "rolling-shutter calibration requires at least two selected "
            "observation timestamps")
    support_times = (rate_times if support_timestamps is None else
                     finite_unique_timestamps(
                         support_timestamps, "observation support timestamp"))
    if not support_times.size:
        raise RuntimeError(
            "rolling-shutter calibration requires observation support timestamps")

    support_start = min(float(times[0]), float(support_times[0]))
    support_end = max(float(times[-1]), float(support_times[-1]))
    padded_times = np.hstack((support_start - 2.0 * padding_s, times,
                              support_end + 2.0 * padding_s))
    padded_curve = np.hstack((curve[:, 0], curve, curve[:, -1]))
    _continuous_rotation_vectors(padded_curve)

    positive_steps = np.diff(rate_times)
    positive_steps = positive_steps[positive_steps > 1e-12]
    if not positive_steps.size:
        raise RuntimeError("selected observation timestamps must increase")
    observed_rate_hz = 1.0 / float(np.median(positive_steps))
    duration_s = float(padded_times[-1] - padded_times[0])
    minimum_segment_count = 2 * order
    if knots_per_second is None:
        requested_rate_hz = observed_rate_hz
        requested_segment_count = int(round(
            duration_s * requested_rate_hz))
        # Keep the selected observation rate across the complete padded time
        # domain.  Capping this value at the pose count shortens the knot grid,
        # changes its phase relative to the measurements and can create a
        # strong line-delay/trajectory resonance.
        segment_count = max(minimum_segment_count, requested_segment_count)
        strategy = "selected_observation_rate"
    else:
        requested_rate_hz = _finite(knots_per_second, "knots per second")
        if requested_rate_hz <= 0.0:
            raise RuntimeError("knots per second must be positive")
        requested_segment_count = int(round(
            duration_s * requested_rate_hz))
        segment_count = max(minimum_segment_count, requested_segment_count)
        strategy = "explicit_knots_per_second"

    spline.initPoseSplineSparse(
        padded_times, padded_curve, segment_count, 1e-4)
    metadata = {
        "strategy": strategy,
        "order": order,
        "initialized_pose_count": len(unique),
        "selected_timestamp_count": int(rate_times.size),
        "support_timestamp_count": int(support_times.size),
        "selected_frame_rate_hz": observed_rate_hz,
        "requested_knots_per_second": requested_rate_hz,
        "requested_segment_count": requested_segment_count,
        "minimum_segment_count": minimum_segment_count,
        "segment_count": segment_count,
        "effective_segments_per_second": segment_count / duration_s,
        "padding_s": padding_s,
        "time_min_s": float(spline.t_min()),
        "time_max_s": float(spline.t_max()),
    }
    return spline, metadata


class LineDelayState(object):
    """One signed, optionally estimated line delay with a smooth bound."""

    def __init__(self, camera_id, geometry, config):
        self.camera_id = str(camera_id)
        self.geometry = geometry
        self.height = int(geometry.geometry.projection().rv())
        if self.height <= 0:
            raise RuntimeError("{} has no valid image height".format(camera_id))
        self.estimate = config.get("estimate", True)
        if type(self.estimate) is not bool:
            raise RuntimeError("{}.estimate must be boolean".format(camera_id))
        self.seed_s = _finite(config.get("line_delay_s", 0.0),
                              "{}.line_delay_s".format(camera_id))
        bound = config.get("max_abs_line_delay_s")
        if self.estimate:
            if bound is None:
                raise RuntimeError(
                    "{}.max_abs_line_delay_s is required when estimating"
                    .format(camera_id))
            self.bound_s = _finite(
                bound, "{}.max_abs_line_delay_s".format(camera_id))
            if self.bound_s <= 0.0 or abs(self.seed_s) >= self.bound_s:
                raise RuntimeError(
                    "{} line-delay seed must be strictly inside a positive "
                    "bound".format(camera_id))
            latent = math.atanh(self.seed_s / self.bound_s)
            self.dv = aopt.Scalar(latent)
            self.dv.setActive(True)
            self.expression = self.dv.toExpression().tanh() * self.bound_s
        else:
            self.bound_s = (_finite(
                bound, "{}.max_abs_line_delay_s".format(camera_id))
                if bound is not None else None)
            if self.bound_s is not None and self.bound_s < abs(self.seed_s):
                raise RuntimeError(
                    "{} fixed line delay lies outside its support bound"
                    .format(camera_id))
            self.dv = aopt.Scalar(0.0)
            self.dv.setActive(False)
            self.expression = aopt.ScalarExpression(self.seed_s)

    @property
    def support_extent_s(self):
        support_delay = (abs(self.seed_s) if self.bound_s is None else
                         max(self.bound_s, abs(self.seed_s)))
        return (self.height - 1.0) * support_delay

    @property
    def expression_margin_s(self):
        """Time-expression buffer around the current linearization point.

        A bounded active delay can move from its current value to either signed
        bound, so the worst displacement is ``bound + abs(current)``.  A fixed
        delay never moves and only needs its absolute row-zero span.
        """
        if self.estimate:
            return ((self.height - 1.0) *
                    (self.bound_s + abs(self.value_s())))
        return (self.height - 1.0) * abs(self.seed_s)

    def value_s(self):
        return float(self.expression.toScalar())

    def sync_native_shutter(self):
        self.geometry.geometry.shutter().setParameters(
            np.asarray([self.value_s()], dtype=float))

    def result(self):
        delay = self.value_s()
        result = {
            "type": "rolling_shutter",
            "line_delay_s": delay,
            "reference_row_px": 0.0,
            "first_to_last_row_span_s": abs(delay) * (self.height - 1),
            "estimated": self.estimate,
            "timestamp_reference": "row0_exposure_end",
            "corner_time_equation": (
                "t_corner_s = t_camera_timestamp_s + y_px * line_delay_s"),
        }
        if self.bound_s is not None:
            result["max_abs_line_delay_s"] = self.bound_s
            result["distance_to_bound_s"] = self.bound_s - abs(delay)
        return result


class _PoseProxy(object):
    """Expose a spline expression through CameraUtils' pose interface."""

    def __init__(self, expression):
        self.expression = expression

    def T(self):
        return self.expression.toTransformationMatrix()


class SystemRsCalibrator(object):
    """Build and solve one joint rolling-shutter camera-system problem."""

    def __init__(self, cameras, observation_database, baseline_guesses,
                 shutter_configs, *, spline_order=4, time_padding_s=0.5,
                 knots_per_second=None, feature_sigma_px=1.0,
                 motion_translation_weight=1e-5,
                 motion_rotation_weight=1e-2,
                 freeze_intrinsics=False, use_blake_zisserman=False,
                 max_iterations=80, verbose=False):
        spline_order = int(spline_order)
        if spline_order < 3:
            raise RuntimeError(
                "spline order must be at least 3 for the second-order "
                "motion prior")
        if not cameras:
            raise RuntimeError("rolling-shutter calibration needs a camera")
        if len(baseline_guesses) != len(cameras) - 1:
            raise RuntimeError("adjacent baseline count does not match cameras")
        expected = {"cam{}".format(index) for index in range(len(cameras))}
        if set(shutter_configs) != expected:
            raise RuntimeError(
                "rolling-shutter config must contain exactly {}".format(
                    ", ".join(sorted(expected))))
        self.cameras = list(cameras)
        self.obsdb = observation_database
        self.freeze_intrinsics = bool(freeze_intrinsics)
        self.use_blake_zisserman = bool(use_blake_zisserman)
        self.max_iterations = int(max_iterations)
        self.verbose = bool(verbose)
        if self.max_iterations < 1:
            raise RuntimeError("max iterations must be positive")
        self.feature_sigma_px = _finite(feature_sigma_px,
                                        "feature sigma")
        if self.feature_sigma_px <= 0.0:
            raise RuntimeError("feature sigma must be positive")
        self.motion_translation_weight = _finite(
            motion_translation_weight, "motion translation weight")
        self.motion_rotation_weight = _finite(
            motion_rotation_weight, "motion rotation weight")
        if (self.motion_translation_weight <= 0.0 or
                self.motion_rotation_weight <= 0.0):
            raise RuntimeError("motion-prior weights must be positive")

        self.baselines = [aopt.TransformationDv(value)
                          for value in baseline_guesses]
        self.cumulative_baseline_expressions = []
        cumulative = None
        for camera_index in range(len(self.cameras)):
            if camera_index == 0:
                self.cumulative_baseline_expressions.append(None)
                continue
            baseline = self.baselines[camera_index - 1].toExpression()
            cumulative = baseline if cumulative is None else baseline * cumulative
            self.cumulative_baseline_expressions.append(cumulative)

        self.line_delays = [
            LineDelayState("cam{}".format(index), camera,
                           shutter_configs["cam{}".format(index)])
            for index, camera in enumerate(self.cameras)
        ]
        minimum_padding = max(state.support_extent_s
                              for state in self.line_delays)
        effective_padding = _finite(time_padding_s, "time padding")
        if effective_padding <= minimum_padding:
            raise RuntimeError(
                "time padding {:.9g} s must exceed the maximum possible "
                "row span {:.9g} s".format(
                    effective_padding, minimum_padding))
        selected_timestamps, support_timestamps = self._observation_timestamps()
        pose_samples = self._initial_target_poses(baseline_guesses)
        self.spline, self.spline_metadata = make_pose_spline(
            pose_samples, order=spline_order, padding_s=effective_padding,
            knots_per_second=knots_per_second,
            selected_timestamps=selected_timestamps,
            support_timestamps=support_timestamps)
        self.problem = None
        self.spline_dv = None
        self.views = []
        self.optimizer_result = None
        self.removed_corners = []
        self._active_corners = None

    @staticmethod
    def _cumulative_baseline(baselines, camera_id):
        value = sm.Transformation()
        for baseline in baselines[:camera_id]:
            value = baseline * value
        return value

    def _initial_target_poses(self, baseline_guesses):
        samples = []
        for view_timestamp in sorted(self.obsdb.getAllViewTimestamps()):
            observations = self.obsdb.getAllObsAtTimestamp(view_timestamp)
            observations = sorted(
                observations,
                key=lambda item: (-len(item[1].getCornersIdx()), item[0]))
            for camera_id, observation in observations:
                try:
                    success, T_target_camera = self.cameras[
                        camera_id].geometry.estimateTransformation(observation)
                except (RuntimeError, ValueError):
                    success = False
                if not success:
                    continue
                T_camera_camera0 = self._cumulative_baseline(
                    baseline_guesses, camera_id)
                T_target_camera0 = T_target_camera * T_camera_camera0
                samples.append((observation.time().toSec(),
                                T_target_camera0))
                break
        return samples

    def _observation_timestamps(self):
        """Return per-view rate samples and every camera timestamp in the solve."""
        selected = []
        support = []
        for view_timestamp in sorted(self.obsdb.getAllViewTimestamps()):
            observations = self.obsdb.getAllObsAtTimestamp(view_timestamp)
            if not observations:
                continue
            observation_times = [
                float(observation.time().toSec())
                for unused_camera_id, observation in observations]
            support.extend(observation_times)
            selected.append(next(
                (float(observation.time().toSec())
                 for camera_id, observation in observations
                 if camera_id == 0),
                observation_times[0]))
        return selected, support

    def _add_design_variables(self, problem):
        # BSplinePoseDesignVariable owns a copy of the spline.  Preserve its
        # optimized coefficients before rebuilding after final outlier
        # filtering; otherwise a second solve would silently restart from the
        # pre-optimization trajectory.
        if self.spline_dv is not None:
            optimized = self.spline_dv.spline()
            snapshot = bsplines.BSplinePose(
                int(optimized.splineOrder()), sm.RotationVector())
            snapshot.setKnotVectorAndCoefficients(
                np.asarray(optimized.knots(), dtype=float),
                np.asarray(optimized.coefficients(), dtype=float))
            self.spline = snapshot
        self.spline_dv = asp.BSplinePoseDesignVariable(self.spline)
        for index in range(self.spline_dv.numDesignVariables()):
            variable = self.spline_dv.designVariable(index)
            variable.setActive(True)
            problem.addDesignVariable(variable, TRAJECTORY_GROUP_ID)

        for baseline in self.baselines:
            for index in range(baseline.numDesignVariables()):
                variable = baseline.getDesignVariable(index)
                variable.setActive(True)
                problem.addDesignVariable(variable, CALIBRATION_GROUP_ID)

        for camera, line_delay in zip(self.cameras, self.line_delays):
            camera.setDvActiveStatus(not self.freeze_intrinsics,
                                     not self.freeze_intrinsics, False)
            problem.addDesignVariable(
                camera.dv.projectionDesignVariable(), CALIBRATION_GROUP_ID)
            problem.addDesignVariable(
                camera.dv.distortionDesignVariable(), CALIBRATION_GROUP_ID)
            # ReprojectionError lists all camera DVs, including the inactive
            # native shutter.  Register it even though the physical row state
            # is represented by the separate bounded scalar below.
            problem.addDesignVariable(
                camera.dv.shutterDesignVariable(), CALIBRATION_GROUP_ID)
            problem.addDesignVariable(line_delay.dv, CALIBRATION_GROUP_ID)

        weights = np.diag(
            [self.motion_translation_weight] * 3 +
            [self.motion_rotation_weight] * 3)
        # Segment-wise square-root information terms provide analytic
        # Jacobians to both Optimizer2 and the observability analyzer.
        asp.addMotionErrorTerms(problem, self.spline_dv, weights, 2)

    def _target_landmarks(self, problem):
        target = self.cameras[0].ctarget.detector.target()
        expressions = []
        for corner_id in range(target.size()):
            variable = aopt.HomogeneousPointDv(
                sm.toHomogeneous(target.point(corner_id)))
            variable.setActive(False)
            problem.addDesignVariable(variable, LANDMARK_GROUP_ID)
            expressions.append(variable.toExpression())
        return target, expressions

    def _camera_target_expression(self, camera_id, timestamp_expression,
                                  margin_s):
        T_target_camera0 = self.spline_dv.transformationAtTime(
            timestamp_expression, margin_s, margin_s)
        T_camera_target = T_target_camera0.inverse()
        cumulative = self.cumulative_baseline_expressions[camera_id]
        if cumulative is not None:
            T_camera_target = cumulative * T_camera_target
        return T_camera_target

    def build_problem(self, active_corners=None):
        problem = inc.CalibrationOptimizationProblem()
        self._add_design_variables(problem)
        target, landmarks = self._target_landmarks(problem)
        invR = np.eye(2) / (self.feature_sigma_px ** 2)
        views = []
        total_errors = 0
        maximum_margin = max(state.expression_margin_s
                             for state in self.line_delays) + 1e-6

        for view_timestamp in sorted(self.obsdb.getAllViewTimestamps()):
            observations = self.obsdb.getAllObsAtTimestamp(view_timestamp)
            if not observations:
                continue
            reference_time = next(
                (obs.time().toSec() for camera_id, obs in observations
                 if camera_id == 0),
                observations[0][1].time().toSec())
            reference_expression = self.spline_dv.transformationAtTime(
                aopt.ScalarExpression(reference_time),
                maximum_margin, maximum_margin)
            batch = SimpleNamespace(
                timestamp=float(reference_time),
                rig_observations=list(observations),
                dv_T_target_camera=_PoseProxy(reference_expression),
                rerrs={}, corner_times={}, corner_transforms={},
                reference_transforms={})

            for camera_id, observation in observations:
                state = self.line_delays[camera_id]
                frame_time_s = float(observation.time().toSec())
                frame_time = aopt.ScalarExpression(frame_time_s)
                reference_transform = self._camera_target_expression(
                    camera_id, frame_time, maximum_margin)
                batch.reference_transforms[camera_id] = reference_transform
                errors = [None] * target.size()
                times = [None] * target.size()
                transforms = [None] * target.size()
                allowed = (None if active_corners is None else
                           active_corners.get((camera_id, id(observation)), set()))
                for corner_id in observation.getCornersIdx():
                    corner_id = int(corner_id)
                    if allowed is not None and corner_id not in allowed:
                        continue
                    valid, measurement = observation.imagePoint(corner_id)
                    if not valid:
                        continue
                    measurement = np.asarray(measurement, dtype=float)
                    corner_time = (frame_time + state.expression *
                                   float(measurement[1]))
                    transform = self._camera_target_expression(
                        camera_id, corner_time, maximum_margin)
                    point = transform * landmarks[corner_id]
                    error = self.cameras[camera_id].model.reprojectionError(
                        measurement, invR, point,
                        self.cameras[camera_id].dv)
                    if self.use_blake_zisserman:
                        error.setMEstimatorPolicy(
                            aopt.BlakeZissermanMEstimator(2.0))
                    problem.addErrorTerm(error)
                    errors[corner_id] = error
                    times[corner_id] = corner_time
                    transforms[corner_id] = transform
                    total_errors += 1
                batch.rerrs[camera_id] = errors
                batch.corner_times[camera_id] = times
                batch.corner_transforms[camera_id] = transforms
            views.append(batch)

        if total_errors == 0:
            raise RuntimeError("no rolling-shutter reprojection terms were built")
        self.problem = problem
        self.views = views
        self._active_corners = active_corners
        return problem

    def optimize(self):
        if self.problem is None:
            self.build_problem(self._active_corners)
        options = aopt.Optimizer2Options()
        options.verbose = self.verbose
        options.maxIterations = self.max_iterations
        options.convergenceDeltaX = 1e-5
        options.convergenceDeltaJ = 1e-4
        options.doSchurComplement = True
        options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()
        options.trustRegionPolicy = (
            aopt.LevenbergMarquardtTrustRegionPolicy(10))
        native_runtime.apply_optimizer_threads(options)
        optimizer = aopt.Optimizer2(options)
        optimizer.setProblem(self.problem)
        result = native_runtime.run_optimizer(optimizer)
        self.optimizer_result = result
        if result.linearSolverFailure:
            raise RuntimeError(
                "rolling-shutter camera-system optimization failed")
        # Optimizer2 preserves its native stop rule: either dX or |dJ| may end
        # the solve.  Exhausting the configured iteration budget satisfies
        # neither rule and must not be published as a completed calibration.
        if int(result.iterations) >= self.max_iterations:
            raise RuntimeError(
                "rolling-shutter camera-system optimization did not converge "
                "within max_iterations={} (iterations={}, dXFinal={:.17g}, "
                "dJFinal={:.17g})".format(
                    self.max_iterations, int(result.iterations),
                    float(result.dXFinal), float(result.dJFinal)))
        self._validate_solution()
        return result

    def _validate_solution(self):
        for state in self.line_delays:
            value = state.value_s()
            if not math.isfinite(value):
                raise RuntimeError(
                    "{} line-delay estimate is not finite".format(
                        state.camera_id))
        for camera in self.cameras:
            projection = np.asarray(
                camera.geometry.projection().getParameters(), dtype=float)
            distortion = np.asarray(
                camera.geometry.projection().distortion().getParameters(),
                dtype=float)
            if not (np.all(np.isfinite(projection)) and
                    np.all(np.isfinite(distortion))):
                raise RuntimeError("camera parameters are not finite")
        for baseline in self.baselines:
            if not np.all(np.isfinite(np.asarray(baseline.T(), dtype=float))):
                raise RuntimeError("camera baseline is not finite")
        coefficients = np.asarray(
            self.spline_dv.spline().coefficients(), dtype=float)
        if coefficients.size == 0 or not np.all(np.isfinite(coefficients)):
            raise RuntimeError(
                "rolling-shutter target trajectory coefficients are not finite")
        # Optimizer2 can theoretically terminate on a small update even after
        # an invalid model evaluation.  Validate the physical pixel residuals,
        # rather than relying only on finite calibration design variables.
        self._reprojection_squared_by_camera()

    def _reprojection_squared_by_camera(self):
        values = [[] for _ in self.cameras]
        for batch in self.views:
            for camera_id, _ in batch.rig_observations:
                for error in batch.rerrs[camera_id]:
                    if error is None:
                        continue
                    error.evaluateError()
                    measurement = np.asarray(
                        error.getMeasurement(), dtype=float).reshape(-1)
                    prediction = np.asarray(
                        error.getPredictedMeasurement(),
                        dtype=float).reshape(-1)
                    if (measurement.size != 2 or prediction.size != 2 or
                            not np.all(np.isfinite(measurement)) or
                            not np.all(np.isfinite(prediction))):
                        raise RuntimeError(
                            "cam{} has a non-finite two-dimensional final "
                            "reprojection measurement or prediction".format(
                                camera_id))
                    residual = measurement - prediction
                    squared = float(np.dot(residual, residual))
                    if not math.isfinite(squared):
                        raise RuntimeError(
                            "cam{} has a non-finite final reprojection "
                            "residual".format(camera_id))
                    values[camera_id].append(squared)
        return values

    def require_interior_line_delays(self, fraction=0.995):
        """Reject saturated estimates after observability has been recorded."""
        fraction = _finite(fraction, "line-delay interior fraction")
        if fraction <= 0.0 or fraction >= 1.0:
            raise RuntimeError(
                "line-delay interior fraction must be in (0, 1)")
        saturated = []
        for state in self.line_delays:
            if not state.estimate:
                continue
            estimate = state.value_s()
            if abs(estimate) >= fraction * state.bound_s:
                saturated.append(
                    (state.camera_id, estimate, state.bound_s))
        if saturated:
            details = ", ".join(
                "{}: estimate={:.17g} s/row, bound={:.17g} s/row".format(
                    camera_id, estimate, bound)
                for camera_id, estimate, bound in saturated)
            raise RuntimeError(
                "rolling-shutter estimate reached its search bound ({}); "
                "inspect observability or increase max_abs_line_delay_s"
                .format(details))

    def filter_outliers(self, min_views=20):
        """Apply one native-style four-sigma final corner filtering pass."""
        if self.optimizer_result is None:
            raise RuntimeError("optimize before filtering outliers")
        min_views = int(min_views)
        residuals = {index: [] for index in range(len(self.cameras))}
        records = {index: [] for index in range(len(self.cameras))}
        frame_counts = {index: 0 for index in range(len(self.cameras))}
        for batch in self.views:
            for camera_id, observation in batch.rig_observations:
                used = False
                for corner_id, error in enumerate(batch.rerrs[camera_id]):
                    if error is None:
                        continue
                    error.evaluateError()
                    residual = (np.asarray(error.getMeasurement(), dtype=float)
                                - np.asarray(error.getPredictedMeasurement(),
                                             dtype=float))
                    residuals[camera_id].append(residual)
                    records[camera_id].append(
                        (batch, observation, corner_id, residual))
                    used = True
                frame_counts[camera_id] += int(used)

        removals = []
        thresholds = {}
        for camera_id, values in residuals.items():
            if frame_counts[camera_id] < min_views or not values:
                continue
            values = np.asarray(values, dtype=float)
            sigma = np.std(values, axis=0)
            threshold = np.maximum(4.0 * sigma,
                                   self.feature_sigma_px)
            thresholds[camera_id] = threshold
            candidates = {}
            for batch, observation, corner_id, residual in records[camera_id]:
                if np.any(np.abs(residual) > threshold):
                    candidates.setdefault(id(observation), []).append(
                        (batch, observation, corner_id,
                         float(np.dot(residual, residual))))
            for entries in candidates.values():
                batch, observation = entries[0][0], entries[0][1]
                currently_used = sum(
                    error is not None for error in batch.rerrs[camera_id])
                maximum_remove = max(0, currently_used - 4)
                for entry in sorted(entries, key=lambda value: value[3],
                                    reverse=True)[:maximum_remove]:
                    removals.append((camera_id, entry[0], entry[1],
                                     entry[2]))

        if not removals:
            return 0

        active = {}
        for batch in self.views:
            for camera_id, observation in batch.rig_observations:
                active[(camera_id, id(observation))] = {
                    corner_id for corner_id, error in
                    enumerate(batch.rerrs[camera_id]) if error is not None}
        by_batch_camera = {}
        for camera_id, batch, observation, corner_id in removals:
            active[(camera_id, id(observation))].discard(corner_id)
            by_batch_camera.setdefault((id(batch), camera_id),
                                       (batch, []) )[1].append(corner_id)

        try:
            from kalibr_no_ros import artifacts as run_artifacts
            context = run_artifacts.current_context()
        except ImportError:
            context = None
        if context is not None:
            for (_, camera_id), (batch, corner_ids) in by_batch_camera.items():
                context.record_removed_corners(
                    batch, camera_id, corner_ids, thresholds[camera_id])

        self.removed_corners = [
            {"camera_id": camera_id,
             "timestamp_s": observation.time().toSec(),
             "corner_id": corner_id}
            for camera_id, _, observation, corner_id in removals]
        self.build_problem(active)
        self.optimize()
        return len(removals)

    def line_delay_results(self):
        for state in self.line_delays:
            state.sync_native_shutter()
        return [state.result() for state in self.line_delays]

    def reprojection_rms(self):
        values = self._reprojection_squared_by_camera()
        result = []
        for camera_id, squared in enumerate(values):
            if not squared:
                raise RuntimeError(
                    "cam{} has no final reprojection residuals".format(
                        camera_id))
            rms = float(math.sqrt(np.mean(squared)))
            if not math.isfinite(rms):
                raise RuntimeError(
                    "cam{} final reprojection RMS is not finite".format(
                        camera_id))
            result.append(rms)
        return result

    def observability_parameter_blocks(self):
        from kalibr_no_ros import observability
        blocks = observability.camera_parameter_blocks(self)
        for index, state in enumerate(self.line_delays):
            if state.dv.isActive():
                blocks.append({
                    "name": "cameras.cam{}.line_delay_latent".format(index),
                    "design_variables": [state.dv],
                    "units": "dimensionless",
                    "coordinate_convention": (
                        "line_delay_s = max_abs_line_delay_s * tanh(latent); "
                        "t_corner = t_camera_row0 + y_px * line_delay_s"),
                })
        return blocks

    def publish_artifacts(self, models):
        """Publish final residuals, then attach RS per-corner timing evidence."""
        from kalibr_no_ros import artifacts as run_artifacts
        results = self.line_delay_results()
        context = run_artifacts.current_context()
        if context is None:
            return
        for batch in self.views:
            context.record_view(batch.timestamp, batch.rig_observations,
                                True, "joint_rolling_shutter",
                                self.optimizer_result)
        context.record_optimizer(self.optimizer_result)
        context.publish_camera(self, models)
        for index, result in enumerate(results):
            context.artifacts["cameras"][index]["shutter"] = result
        context.artifacts["rolling_shutter_solver"] = {
            "timestamp_reference": "camera_row0",
            "per_camera_source_timestamps": True,
            "trajectory": dict(self.spline_metadata),
            "motion_prior": {
                "order": 2,
                "translation_weight": self.motion_translation_weight,
                "rotation_weight": self.motion_rotation_weight,
            },
            "removed_corner_count": len(self.removed_corners),
        }
