from copy import deepcopy
from pathlib import Path
import re
import sys
import tempfile
import unittest

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_tools.imu_allan.allan_core import UPSTREAM_REVISION
from kalibr_tools.imu_allan.reporting import generate_reports, _summary_figure, _curve_figure, STYLE


class ImuAllanReportingTest(unittest.TestCase):
    def setUp(self):
        self.source = {
            "source": '/data/<script>alert("input")</script>&imu.csv',
            "dataset_id": "imu_260910_2209",
            "messages": 20001, "duration_s": 2000.0, "effective_rate_hz": 10.0,
            "selection": {"start_s": 1.5, "end_s": None},
        }
        self.parameters = {
            "accelerometer_noise_density": .01, "accelerometer_random_walk": .0001,
            "gyroscope_noise_density": .001, "gyroscope_random_walk": .00001,
            "rostopic": "/imu/<&>", "update_rate": 10.0,
        }
        axis = {
            "white_noise_at_1s": .01, "white_fit_slope": -.5,
            "white_fit_log_intercept": np.log(.01),
            "random_walk_at_3s": .0001, "random_walk_fit_slope": .5,
            "random_walk_fit_log_intercept": np.log(.0001 / np.sqrt(3)),
            "minimum_allan_deviation": .001, "minimum_tau_s": 40.0,
        }
        self.fit = {key: {name: dict(axis) for name in ("x", "y", "z")}
                    for key in ("accelerometer", "gyroscope_deg_per_s")}
        period = np.arange(1, 10000) * .1
        self.curve = np.column_stack([period] + [.01 / np.sqrt(period)] * 6)

    def test_reports_use_cached_evidence_without_standalone_images_or_refitting(self):
        original_figures = plt.get_fignums()
        original_parameters, original_fit = deepcopy(self.parameters), deepcopy(self.fit)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "work").mkdir()
            np.savetxt(output / "work" / "allan_variance.csv", self.curve)
            generate_reports(output, self.source, self.parameters, self.fit, 10)
            html = (output / "imu_allan.report.html").read_text(encoding="utf-8")
            self.assertIn("&lt;script&gt;", html)
            self.assertIn("/imu/&lt;&amp;&gt;", html)
            self.assertNotIn("<script>", html)
            self.assertEqual(html.count("data:image/png;base64,"), 2)
            self.assertIn(UPSTREAM_REVISION, html)
            self.assertIn("[1.5, end) s", html)
            self.assertIn("accelerometer_random_walk", html)
            self.assertIn("rad/s/√Hz", html)
            self.assertIn("未自动判断静止状态", html)
            self.assertIn("斜率强制约束为 [0.5, 0.501]", html)
            pdf = (output / "imu_allan.report.pdf").read_bytes()
            self.assertTrue(pdf.startswith(b"%PDF-"))
            self.assertEqual(len(re.findall(rb"/Type\s*/Page\b", pdf)), 4)
            self.assertNotRegex(pdf, rb"/Subtype\s*/Image\b")
            self.assertEqual(list(output.glob("*.png")), [])
            self.assertEqual({path.name for path in output.iterdir()},
                             {"work", "imu_allan.report.html", "imu_allan.report.pdf"})
        self.assertEqual(self.parameters, original_parameters)
        self.assertEqual(self.fit, original_fit)
        self.assertEqual(plt.get_fignums(), original_figures)

    def test_summary_keeps_long_source_paths_and_parameter_table_inside_page(self):
        source = dict(self.source, source="/mnt/q/" + "测试场/" * 18 + "imu_260910_2209/imu/imu0.csv")
        with matplotlib.rc_context(STYLE):
            figure = _summary_figure(source, self.parameters, 10)
            try:
                figure.canvas.draw()
                renderer = figure.canvas.get_renderer()
                for text in figure.texts:
                    with self.subTest(text=text.get_text()[:50]):
                        bounds = text.get_window_extent(renderer)
                        self.assertGreaterEqual(bounds.x0, 0)
                        self.assertLessEqual(bounds.x1, figure.bbox.x1)
                        self.assertGreaterEqual(bounds.y0, 0)
                        self.assertLessEqual(bounds.y1, figure.bbox.y1)
                self.assertEqual(len(figure.axes[0].tables), 1)
            finally:
                plt.close(figure)

    def test_curve_fit_lines_are_reconstructed_from_saved_coefficients(self):
        with matplotlib.rc_context(STYLE):
            figure = _curve_figure(self.curve, self.fit, "accelerometer", 3)
            try:
                axis = figure.axes[0]
                period = self.curve[:, 0]
                np.testing.assert_array_equal(axis.lines[0].get_ydata(), self.curve[:, 1])
                expected = np.exp(-.5 * np.log(period) + np.log(.01))
                np.testing.assert_array_equal(axis.lines[1].get_ydata(), expected)
                expected_walk = np.exp(.5 * np.log(period) + np.log(.0001 / np.sqrt(3)))
                np.testing.assert_array_equal(axis.lines[2].get_ydata(), expected_walk)
            finally:
                plt.close(figure)


if __name__ == "__main__":
    unittest.main()
