"""Check the pinned Allan bin boundary, fits, and Kalibr parameter units."""

import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from kalibr_tools.imu_allan import allan_core, upstream_analysis


class AllanCoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def test_nonoverlapping_bins_keep_the_upstream_last_sample_boundary(self):
        samples = np.array([1., 2., 4., 8., 16., 32., 64., 128., 4096.])
        prefix = np.concatenate(([0.], np.cumsum(samples)))
        for count in (8, 9):
            for size in (1, 2, 3):
                bins = (count - 1) // size
                means = [sum(samples[i * size:(i + 1) * size]) / size
                         for i in range(bins)]
                expected = math.sqrt(sum((b - a) ** 2
                                         for a, b in zip(means, means[1:]))
                                     / (2 * (bins - 1)))
                self.assertAlmostEqual(
                    allan_core._deviation(prefix, count, size), expected)
        with self.assertRaisesRegex(ValueError, "positive integer"):
            allan_core._deviation(prefix, len(samples), 0)

    def test_fixed_grid_serialization_and_independent_reference_scales(self):
        samples = np.random.default_rng(14).normal(size=(20001, 6))
        source, target = self.root / "samples.npy", self.root / "curves.csv"
        np.save(source, samples)
        curves = allan_core.compute_curves(source, target, 10)
        self.assertEqual(curves.shape, (9999, 7))
        np.testing.assert_array_equal(
            curves[:, 0], np.arange(1, 10000, dtype=float) * .1)
        serialized = np.loadtxt(target)
        for index in allan_core.REFERENCE_INDICES:
            size = int(curves[index, 0] * 10)
            bins = (len(samples) - 1) // size
            means = samples[:bins * size].reshape(bins, size, 6).mean(axis=1)
            expected = np.sqrt(np.mean(np.diff(means, axis=0) ** 2, axis=0) / 2)
            np.testing.assert_allclose(curves[index, 1:], expected, rtol=1e-10)
            self.assertEqual(serialized[index, 1], float(f"{curves[index, 1]:.7g}"))
        validation = allan_core.validate_curves(source, target, 10)
        self.assertEqual(validation["status"], "pass")
        self.assertEqual(len(validation["reference_checks"]), 5)
        serialized[99, 1] *= 2
        np.savetxt(target, serialized)
        with self.assertRaisesRegex(RuntimeError, "independent Allan check failed"):
            allan_core.validate_curves(source, target, 10)

    def test_invalid_rate_short_or_nonfinite_samples_fail_before_writing(self):
        source, target = self.root / "samples.npy", self.root / "curves.csv"
        np.save(source, np.ones((20001, 6)))
        for rate in (0, 9, 10.5, True):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                allan_core.compute_curves(source, target, rate)
        for samples, error in ((np.ones((4, 6)), "complete bins"),
                               (np.ones((20001, 5)), "shaped"),
                               (np.full((20001, 6), np.nan), "finite")):
            np.save(source, samples)
            with self.assertRaisesRegex(ValueError, error):
                allan_core.compute_curves(source, target, 10)
        self.assertFalse(target.exists())

    def test_zero_deviation_is_not_exported_as_a_valid_curve(self):
        source, target = self.root / "samples.npy", self.root / "curves.csv"
        np.save(source, np.zeros((20001, 6)))
        with self.assertRaisesRegex(RuntimeError, "non-positive"):
            allan_core.compute_curves(source, target, 10)
        self.assertFalse(target.exists())

    def test_pinned_reference_fits_axis_maximum_and_gyro_radians(self):
        period = np.arange(1, 10000, dtype=float) * .1
        base = np.sqrt(.006 ** 2 / period + .0002 ** 2 * period / 3)
        curve = np.column_stack([period] + [base * scale for scale in (1, 2, 3, 4, 5, 6)])
        path = self.root / "curve.csv"
        np.savetxt(path, curve)
        parameters, details = upstream_analysis.analyze(path, "/imu/data", 100.)
        # Values from the supplied upstream_analysis.py, using this analytic fixture.
        reference = {
            "accelerometer_noise_density": .018085713111962012,
            "accelerometer_random_walk": .0006498909666774076,
            "gyroscope_noise_density": .0006313104827496009,
            "gyroscope_random_walk": 2.2685474294646374e-05,
        }
        for key, expected in reference.items():
            self.assertAlmostEqual(parameters[key], expected, delta=expected * 1e-7)
        acceleration = details["accelerometer"]
        gyroscope = details["gyroscope_deg_per_s"]
        self.assertEqual(parameters["accelerometer_noise_density"],
                         max(axis["white_noise_at_1s"] for axis in acceleration.values()))
        self.assertEqual(parameters["gyroscope_random_walk"],
                         max(axis["random_walk_at_3s"] for axis in gyroscope.values())
                         * math.pi / 180.)
        self.assertEqual(parameters["update_rate"], 100.)
        self.assertEqual(parameters["rostopic"], "/imu/data")
        self.assertFalse((self.root / "imu.yaml").exists())
        self.assertFalse(list(self.root.glob("*.png")))

    def test_fit_rejects_invalid_curves_and_preserves_bounded_log_fit(self):
        period = np.arange(1, 100, dtype=float) * .1
        expected_slope, expected_intercept = -.4995, .03
        values = expected_intercept * period ** expected_slope
        value, fitted, coefficients = upstream_analysis.get_intercept(
            period, values, -.5, 1.)
        self.assertAlmostEqual(value, expected_intercept, places=9)
        self.assertAlmostEqual(coefficients[0], expected_slope, places=7)
        np.testing.assert_allclose(fitted(period), values, rtol=1e-7)
        for invalid in (np.zeros_like(values), np.full_like(values, np.nan)):
            with self.assertRaisesRegex(ValueError, "finite positive"):
                upstream_analysis.get_intercept(period, invalid, -.5, 1.)
        path = self.root / "invalid.csv"
        np.savetxt(path, np.zeros((9999, 7)))
        with self.assertRaisesRegex(ValueError, "positive finite"):
            upstream_analysis.analyze(path, "/imu/data", 100.)


if __name__ == "__main__":
    unittest.main()
