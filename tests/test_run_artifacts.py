"""Exercise provenance and final evidence against native observations/errors."""

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_bag_io.model import FileImageIndex, ImageRecord, ImuRecord
from kalibr_no_ros import artifacts
from kalibr_no_ros.datasets import BagImageDatasetReader, BagImuDatasetReader

try:
    import aslam_cv as acv
    import aslam_cv_backend as acvb
    import aslam_backend as aopt
    import kalibr_errorterms as ket
    import aslam_splines as asp
    import bsplines
    import incremental_calibration as inc
    import sm
except ImportError:
    acv = None


class _ImageIndex:
    def __init__(self, records):
        self.index = records

    def get_by_entry(self, entry):
        return ImageRecord(entry.header_timestamp_ns, entry.record_timestamp_ns,
                           "mono8", np.zeros((80, 100), dtype=np.uint8))

    def close(self):
        pass


class _Reader:
    def __init__(self):
        # All three values collapse onto the same double-precision seconds.
        self.records = [FileImageIndex(1700000000000000000 + i,
                                      1700000000000000010 + i,
                                      "/tmp/kalibr-input/{}.png".format(i))
                        for i in range(3)]

    def index_images(self, topic):
        return _ImageIndex(self.records)

    def read_imu(self, topic):
        return [ImuRecord(row.header_timestamp_ns, row.record_timestamp_ns,
                          np.asarray([1., 2., 3.]), np.asarray([4., 5., 6.]))
                for row in self.records]

    @staticmethod
    def _crop(records, bounds):
        return records if bounds is None else records[1:]

    @staticmethod
    def _decimate(records, frequency):
        return records


class _NativeDetector:
    def __init__(self, target):
        self.target = target

    def findTargetNoTransformation(self, stamp, image):
        index = stamp.toNSec() - 1700000000000000000
        if index == 1:
            return False, None
        observation = acv.GridCalibrationTargetObservation(self.target)
        observation.setImage(image)
        observation.setTime(stamp)
        observation.updateImagePoint(0, np.asarray([50. + index, 40.]))
        return True, observation


class RunContextTest(unittest.TestCase):
    def test_nested_failure_restores_context_and_does_not_reuse_artifacts(self):
        self.assertIsNone(artifacts.current_context())
        with artifacts.run_context("camera_calibration") as outer:
            with self.assertRaisesRegex(ValueError, "failed input"):
                with artifacts.run_context("camera_imu_calibration") as inner:
                    raise ValueError("failed input")
            self.assertIs(artifacts.current_context(), outer)
            self.assertEqual(inner.artifacts["state"], "failed")
            self.assertEqual(inner.artifacts["failure"]["type"], "ValueError")
        self.assertEqual(outer.artifacts["state"], "incomplete")
        self.assertIsNone(artifacts.current_context())
        with artifacts.run_context("cameras") as later:
            self.assertEqual(later.artifacts["cameras"], [])
            self.assertIsNot(later.artifacts, outer.artifacts)

    def test_source_metadata_preserves_crop_indices_and_nanoseconds(self):
        dataset = BagImageDatasetReader(
            "/tmp/kalibr-input", "/cam0", bag_from_to=(0, 1),
            reader=_Reader(), time_factory=lambda *args: args)
        with artifacts.run_context("cameras") as context:
            context.register_dataset(dataset)
            frames = context.artifacts["cameras"][0]["frames"]
            self.assertEqual([frame["source_index"] for frame in frames], [1, 2])
            self.assertEqual([frame["source_timestamp_ns"] for frame in frames],
                             [1700000000000000001, 1700000000000000002])
            self.assertEqual([frame["frame_id"] for frame in frames], ["cam0:1", "cam0:2"])
            self.assertTrue(frames[0]["source_path"].endswith("/1.png"))
            camera = context.artifacts["cameras"][0]
            self.assertEqual(camera["source_frame_count"], 3)
            self.assertEqual(camera["selected_frame_count"], 2)


@unittest.skipIf(acv is None, "native build-tree Python packages are unavailable")
class NativeRunArtifactsTest(unittest.TestCase):
    def setUp(self):
        self.dataset = BagImageDatasetReader(
            "/tmp/kalibr-input", "/cam0", reader=_Reader(), time_factory=acv.Time)
        self.target = acv.GridCalibrationTargetCheckerboard(
            2, 2, 0.04, 0.04, acv.CheckerboardOptions())
        projection = acv.EquidistantPinholeProjection(
            70., 72., 50., 40., 100, 80, acv.EquidistantDistortion(0., 0., 0., 0.))
        self.geometry = acv.EquidistantDistortedPinholeCameraGeometry(projection)
        self.dv = acvb.EquidistantPinhole.designVariable(self.geometry)

    def observation(self, source_index, corners=(0, 3)):
        observation = acv.GridCalibrationTargetObservation(self.target)
        observation.setImage(np.zeros((80, 100), dtype=np.uint8))
        stamp, _ = self.dataset.getImage(source_index)
        observation.setTime(stamp)
        for corner_id in corners:
            observation.updateImagePoint(corner_id, np.asarray([51. + corner_id, 42.]))
        return observation

    def error(self, observation, corner_id, simple=False):
        _, measurement = observation.imagePoint(int(corner_id))
        point = aopt.HomogeneousExpression(np.asarray([0., 0., 1., 1.]))
        if simple:
            return acvb.EquidistantDistortedPinholeReprojectionErrorSimple(
                measurement, np.eye(2), point, self.geometry)
        return acvb.EquidistantPinhole.reprojectionError(measurement, np.eye(2), point, self.dv)

    def test_extractor_records_failed_frame_without_shifting_source_indices(self):
        path = ROOT / "src/kalibr/calibration/kalibr/python/kalibr_common/TargetExtractor.py"
        specification = importlib.util.spec_from_file_location("artifact_target_extractor", path)
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)
        with artifacts.run_context("cameras") as context:
            observations = module.extractCornersFromDataset(
                self.dataset, _NativeDetector(self.target), noTransformation=True)
            self.assertEqual(len(observations), 2)
            frames = context.artifacts["cameras"][0]["frames"]
            self.assertEqual([f["detection_status"] for f in frames],
                             ["succeeded", "failed", "succeeded"])
            self.assertEqual(context.frame(observations[1])["source_index"], 2)
            self.assertEqual(context.frame(observations[1])["source_timestamp_ns"],
                             1700000000000000002)

    def test_camera_pose_composes_baseline_in_target_to_camera_direction(self):
        right_dataset = BagImageDatasetReader(
            "/tmp/kalibr-input", "/cam1", reader=_Reader(), time_factory=acv.Time)
        left, right = self.observation(0, (0,)), self.observation(0, (0,))
        rotation = np.asarray([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        baseline = np.eye(4)
        baseline[:3, :3] = rotation
        baseline[:3, 3] = [0.1, 0.2, 0.3]
        target_camera = np.eye(4)
        target_camera[:3, 3] = [0.3, 0.4, 0.5]
        batch = SimpleNamespace(
            timestamp=left.time().toSec(), rig_observations=[(0, left), (1, right)],
            dv_T_target_camera=aopt.TransformationDv(sm.Transformation(target_camera)),
            rerrs={0: [self.error(left, 0)], 1: [self.error(right, 0)]})
        native = SimpleNamespace(
            cameras=[SimpleNamespace(dataset=self.dataset, geometry=self.geometry),
                     SimpleNamespace(dataset=right_dataset, geometry=self.geometry)],
            baselines=[aopt.TransformationDv(sm.Transformation(baseline))], views=[batch])
        with artifacts.run_context("cameras") as context:
            context.record_detection(self.dataset, 0, left)
            context.record_detection(right_dataset, 0, right)
            context.publish_camera(native, ["pinhole-equi", "pinhole-equi"])
            camera = context.artifacts["cameras"][1]
            expected = baseline @ np.linalg.inv(target_camera)
            np.testing.assert_allclose(camera["frames"][0]["T_camera_target"], expected, atol=1e-14)
            np.testing.assert_allclose(camera["T_cn_cnm1"], baseline, atol=1e-14)
            np.testing.assert_allclose(np.asarray(camera["frames"][0]["T_camera_target"]) @ [0., 0., 0., 1.],
                                       expected[:, 3])

    def test_final_camera_mapping_uses_source_identity_and_retained_corners(self):
        early = self.observation(0)
        late = self.observation(2)
        with artifacts.run_context("cameras", capture_history=True) as context:
            context.register_dataset(self.dataset)
            # Mimic out-of-order worker completion, including a failed frame.
            context.record_detection(self.dataset, 2, late)
            context.record_detection(self.dataset, 1, None)
            context.record_detection(self.dataset, 0, early)
            native_camera = SimpleNamespace(dataset=self.dataset, geometry=self.geometry)
            errors = [self.error(late, 0), None, None, self.error(late, 3)]
            target_camera = np.eye(4)
            target_camera[0, 3] = 0.25
            batch = SimpleNamespace(timestamp=late.time().toSec(), rig_observations=[(0, late)],
                                    dv_T_target_camera=aopt.TransformationDv(sm.Transformation(target_camera)),
                                    rerrs={0: errors})
            context.record_view(early.time().toSec(), [(0, early)], False, "incremental")
            context.record_view(batch.timestamp, batch.rig_observations, True, "incremental")
            context.record_removed_corners(batch, 0, [3], np.asarray([2., 2.]))
            late.removeImagePoint(3)
            batch.rerrs[0][3] = None
            # The final attempted candidate is rejected and its JFinal is
            # unrelated to the one retained observation below.
            rejected = inc.IncrementalEstimatorReturnValue()
            rejected.batchAccepted = False
            rejected.numIterations = 7
            rejected.JStart, rejected.JFinal = 123., 100.
            context.record_view(early.time().toSec(), [(0, early)], False,
                                "incremental", rejected)
            native = SimpleNamespace(cameras=[native_camera], baselines=[], views=[batch])
            context.publish_camera(native, ["pinhole-equi"])
            frames = context.artifacts["cameras"][0]["frames"]
            self.assertEqual([f["used"] for f in frames], [False, False, True])
            self.assertEqual(frames[2]["source_timestamp_ns"], 1700000000000000002)
            self.assertEqual(frames[2]["observation_timestamp_ns"], 1700000000000000002)
            self.assertEqual(frames[2]["T_camera_target"][0][3], -0.25)
            retained, removed = frames[2]["corners"]
            np.testing.assert_allclose(retained["prediction_px"], [50., 40.])
            np.testing.assert_allclose(retained["residual_px"], [1., 2.])
            self.assertTrue(retained["used"])
            self.assertFalse(removed["used"])
            self.assertIsNone(removed["residual_px"])
            removals = [event for event in context.artifacts["events"]
                        if event["kind"] == "corner_removed"]
            self.assertEqual(removals[0]["corner_id"], 3)
            self.assertEqual([v["used"] for v in context.artifacts["views"]], [False, True])
            optimizer = context.artifacts["optimizer"]
            self.assertEqual(optimizer["scope"], "last_attempt")
            self.assertEqual(optimizer["JFinal"], 100.)
            self.assertEqual(optimizer["numIterations"], 7)
            self.assertFalse(optimizer["batchAccepted"])
            self.assertEqual(optimizer["stop_reason"], "unavailable")
            objective = context.artifacts["objective"]
            self.assertEqual(objective["scope"], "final_retained_camera_views")
            self.assertEqual(objective["used_error_term_count"], 1)
            self.assertAlmostEqual(objective["final_camera_weighted_residual_sum"], 5.)
            json.dumps(context.artifacts, allow_nan=False)
        self.assertEqual(context.artifacts["state"], "completed")

    def test_imu_mapping_skips_camera_frame_and_normalizes_native_imu_sign(self):
        skipped, used = self.observation(0), self.observation(2)
        configuration = SimpleNamespace(
            getIntrinsics=lambda: ("pinhole", [70., 72., 50., 40.]),
            getDistortion=lambda: ("equidistant", [0., 0., 0., 0.]),
            getResolution=lambda: [100, 80])
        camera = SimpleNamespace(dataset=self.dataset, camConfig=configuration)
        imu_dataset = BagImuDatasetReader(
            "/tmp/kalibr-input", "/imu0", reader=_Reader(), time_factory=acv.Time)
        samples = [SimpleNamespace(stamp=stamp, omega=omega, alpha=alpha)
                   for stamp, omega, alpha in imu_dataset]
        imu = SimpleNamespace(dataset=imu_dataset, imuData=samples, timeOffset=0.)
        chain = SimpleNamespace(camList=[camera],
                                getResultTrafoImuToCam=lambda index: sm.Transformation(np.eye(4)),
                                getResultTimeShift=lambda index: 0.)
        native = SimpleNamespace(CameraChain=chain, ImuList=[imu])
        with artifacts.run_context("camera_imu") as context:
            context.record_detection(self.dataset, 0, skipped)
            context.record_detection(self.dataset, 1, None)
            context.record_detection(self.dataset, 2, used)
            context.record_camera_skipped(skipped, "outside_spline_support")
            errors = [self.error(used, index, simple=True) for index in used.getCornersIdx()]
            context.record_camera_terms(camera, used, errors,
                                        aopt.TransformationExpression(np.eye(4)),
                                        aopt.ScalarExpression(used.time().toSec()))
            context.record_imu_sources(imu)
            prediction = np.asarray([0.5, 2.5, 2.])
            error = ket.EuclideanError(samples[2].omega, np.eye(3), aopt.EuclideanExpression(prediction))
            context.record_imu_term(imu, samples[2], "gyro", error)
            context.publish_imu_camera(native)
            frames = context.artifacts["cameras"][0]["frames"]
            self.assertFalse(frames[0]["used"])
            self.assertEqual(frames[0]["exclusion_reason"], "outside_spline_support")
            self.assertTrue(frames[2]["used"])
            self.assertEqual([c["corner_id"] for c in frames[2]["corners"]], [0, 3])
            self.assertEqual(context.artifacts["views"], [])
            row = context.artifacts["imu_residuals"][0]
            self.assertEqual(row["timestamp_ns"], 1700000000000000002)
            self.assertEqual(row["source_index"], 2)
            np.testing.assert_allclose(row["prediction"], prediction)
            np.testing.assert_allclose(row["residual"], samples[2].omega - prediction)
            missing_bias = context.artifacts["imu_biases"][0]["samples"][0]
            self.assertIsNone(missing_bias["value"])
            self.assertIn("unavailable_reason", missing_bias)
            json.dumps(context.artifacts, allow_nan=False)

    def test_rolling_shutter_archive_keeps_final_corner_times_and_poses(self):
        import tempfile
        from kalibr_no_ros.reporting import write_archive, load_archive
        observation = self.observation(0)
        shutter = {"type": "rolling_shutter", "line_delay_s": 8e-6,
                   "reference_row_px": 39.5, "first_to_last_row_span_s": 79*8e-6,
                   "estimated": True}
        camera = SimpleNamespace(dataset=self.dataset, getShutterResult=lambda: shutter,
            camConfig=SimpleNamespace(getIntrinsics=lambda: ("pinhole", [70.,72.,50.,40.]),
                getDistortion=lambda: ("equidistant", [0.,0.,0.,0.]), getResolution=lambda: [100,80]))
        chain = SimpleNamespace(camList=[camera],
            getResultTrafoImuToCam=lambda _: sm.Transformation(), getResultTimeShift=lambda _: 0.)
        time_dv = aopt.Scalar(2.0)
        with artifacts.run_context("camera_imu_rolling_shutter_calibration") as context:
            context.record_detection(self.dataset, 0, observation)
            errors = [self.error(observation, index, simple=True) for index in observation.getCornersIdx()]
            poses = [np.eye(4), np.eye(4)]; poses[1][0,3] = .01
            context.record_camera_terms(camera, observation, errors,
                aopt.TransformationExpression(np.eye(4)), time_dv.toExpression(),
                corner_times=[time_dv.toExpression()-.001, time_dv.toExpression()+.002],
                corner_transforms=[aopt.TransformationExpression(pose) for pose in poses])
            time_dv.update(np.array([.5]))
            context.publish_imu_camera(SimpleNamespace(CameraChain=chain, ImuList=[]))
            with tempfile.TemporaryDirectory() as directory:
                write_archive(context.artifacts, directory)
                restored = load_archive(directory)
            self.assertEqual(restored["cameras"][0]["shutter"], shutter)
            frame = restored["cameras"][0]["frames"][0]
            self.assertEqual(frame["pose_time_reference"], "shutter_reference_row")
            for corner, expected_time, pose in zip(frame["corners"], [2.499,2.502], poses):
                self.assertAlmostEqual(corner["solver_timestamp_s"], expected_time)
                self.assertAlmostEqual(corner["row_time_offset_s"], expected_time-2.5)
                np.testing.assert_array_equal(corner["T_camera_target"], pose)

    def test_optimizer2_summary_reads_actual_native_return_fields(self):
        result = aopt.SolutionReturnValue()
        result.iterations, result.failedIterations = 9, 2
        result.dXFinal, result.dJFinal = 1.5e-6, 0.004
        result.JStart, result.JFinal = 50., 25.
        with artifacts.run_context("camera_imu") as context:
            context.record_optimizer(result)
            context.publish_imu_camera(SimpleNamespace(
                CameraChain=SimpleNamespace(camList=[]), ImuList=[]))
            summary = context.artifacts["optimizer"]
            self.assertEqual(summary["scope"], "final_joint_optimization")
            for name in ("iterations", "failedIterations", "dXFinal", "dJFinal",
                         "JStart", "JFinal", "linearSolverFailure"):
                self.assertEqual(summary[name], getattr(result, name))
            self.assertNotIn("numIterations", summary)
            self.assertNotIn("lmLambdaFinal", summary)  # Optimizer2 never sets it.
            self.assertEqual(summary["stop_reason"], "unavailable")
            self.assertEqual(summary["stopping_thresholds"], "unavailable")
            self.assertFalse(summary["measurement_residual_sum_is_full_objective"])
            self.assertIn("bias_motion_priors", summary["objective_terms"])

    def test_native_normalization_separates_noise_and_robust_weight(self):
        measurement = np.asarray([1., 2., 3.])
        prediction = np.asarray([4., 0., 5.])
        precision = np.asarray([[4., 1., .5], [1., 3., .2], [.5, .2, 2.]])
        residual = measurement - prediction
        error = ket.EuclideanError(measurement, precision,
                                   aopt.EuclideanExpression(prediction))
        squared = float(residual @ precision @ residual)
        previous_whitened = None
        for policy in (aopt.NoMEstimator(), aopt.HuberMEstimator(2.)):
            error.setMEstimatorPolicy(policy)
            objective = error.evaluateError()
            normalized = artifacts._normalized_error(error, residual)
            self.assertAlmostEqual(normalized["whitened_squared_error"], squared)
            self.assertAlmostEqual(error.getRawSquaredError(), squared)
            self.assertAlmostEqual(normalized["weighted_squared_error"], objective)
            self.assertAlmostEqual(normalized["robust_weight"] * squared, objective)
            if previous_whitened is not None:
                np.testing.assert_allclose(normalized["whitened_residual"], previous_whitened)
                self.assertLess(normalized["robust_weight"], 1.)
            previous_whitened = normalized["whitened_residual"]
        # Native camera and IMU signs differ; scalar normalization is invariant.
        reversed_sign = artifacts._normalized_error(error, -residual)
        np.testing.assert_allclose(reversed_sign["whitened_residual"], -np.asarray(previous_whitened))
        self.assertAlmostEqual(reversed_sign["whitened_squared_error"], squared)

    def test_bias_snapshot_reads_final_native_spline_without_changing_it(self):
        reader = _Reader()
        reader.records = [FileImageIndex(1700000000000000000 + index * 1000000000,
                                        1700000000000000010 + index * 1000000000,
                                        "/tmp/kalibr-input/{}.png".format(index))
                          for index in range(3)]
        dataset = BagImuDatasetReader("/tmp/kalibr-input", "/imu0", reader=reader,
                                      time_factory=acv.Time)
        measurements = [SimpleNamespace(stamp=stamp, omega=omega, alpha=alpha)
                        for stamp, omega, alpha in dataset]
        gyro = bsplines.BSpline(4)
        gyro.initConstantSpline(-1., 3., 8, np.asarray([.1, .2, .3]))
        accel = bsplines.BSpline(4)
        accel.initConstantSpline(-1., 3., 8, np.asarray([1., 2., 3.]))
        gyro_dv, accel_dv = (asp.EuclideanBSplineDesignVariable(spline)
                             for spline in (gyro, accel))
        # Native DVs own a copy of the initialization spline. Change one
        # coefficient as optimization would; exporting the seed would be wrong.
        gyro_dv.designVariable(3).update(np.asarray([.9, 1.8, 2.7]))
        imu = SimpleNamespace(dataset=dataset, imuData=measurements,
                              timeOffset=-1700000000., gyroBias=gyro, accelBias=accel,
                              gyroBiasDv=gyro_dv, accelBiasDv=accel_dv)
        before = gyro_dv.spline().coefficients().copy()
        self.assertFalse(np.allclose(gyro.evalD(0., 0), gyro_dv.toEuclidean(0., 0)))
        with artifacts.run_context("camera_imu") as context:
            context.record_imu_sources(imu)
            for sample in measurements:
                for kind in ("gyro", "accel"):
                    measured = sample.omega if kind == "gyro" else sample.alpha
                    context.record_imu_term(imu, sample, kind, ket.EuclideanError(
                        measured, np.eye(3), aopt.EuclideanExpression(np.zeros(3))))
            context.publish_imu_camera(SimpleNamespace(
                CameraChain=SimpleNamespace(camList=[]), ImuList=[imu]))
            series = context.artifacts["imu_biases"]
            self.assertEqual([entry["units"] for entry in series], ["rad/s", "m/s^2"])
            for entry, variable in zip(series, (gyro_dv, accel_dv)):
                self.assertEqual(entry["representation"], "time_varying_spline")
                self.assertEqual(len(entry["samples"]), 3)
                for index, row in enumerate(entry["samples"]):
                    self.assertEqual(row["timestamp_ns"], reader.records[index].header_timestamp_ns)
                    self.assertEqual(row["source_index"], index)
                    self.assertEqual(row["solver_timestamp_s"], float(index))
                    np.testing.assert_allclose(row["value"], variable.toEuclidean(float(index), 0))
            json.dumps(context.artifacts, allow_nan=False)
        np.testing.assert_array_equal(before, gyro_dv.spline().coefficients())


if __name__ == "__main__":
    unittest.main()
