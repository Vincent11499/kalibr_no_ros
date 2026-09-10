"""Direction and ray mathematics used by portable report plots."""

from pathlib import Path
import sys
import unittest
import tempfile
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "python"))

from kalibr_no_ros.report_plots import camera_frames, camera_observations, observation_rays, render_reports, stereo_comparison_page


class ReportPlotsTest(unittest.TestCase):
    def test_stereo_pdf_page_places_original_above_rectified_without_resampling(self):
        original = np.full((10, 40, 3), 35, dtype=np.uint8)
        corrected = np.full((10, 40, 3), 180, dtype=np.uint8)
        page = stereo_comparison_page(original, corrected, 'Test source pair')
        self.assertGreater(page.axes[0].get_position().y0, page.axes[1].get_position().y1)
        for axis, expected in zip(page.axes, (original, corrected)):
            np.testing.assert_array_equal(axis.images[0].get_array(), expected)
            self.assertEqual(axis.images[0].get_interpolation(), 'none')

    def test_multicamera_rig_uses_inverse_and_composed_baselines(self):
        transforms = []
        for rotation, translation in (([.13, -.2, .07], [-.4, .03, .01]),
                                      ([-.08, .12, .3], [.02, -.21, .05])):
            transform = np.eye(4)
            transform[:3, :3] = cv2.Rodrigues(np.asarray(rotation))[0]
            transform[:3, 3] = translation
            transforms.append(transform)
        frames = camera_frames([{"id": "left"},
                                {"id": "right", "T_cn_cnm1": transforms[0].tolist()},
                                {"id": "top", "T_cn_cnm1": transforms[1].tolist()}])
        self.assertEqual([name for name, _ in frames], ["left", "right", "top"])
        # Mapping a plotted camera frame back along the physical chain must
        # recover that camera's origin and basis, including noncommuting R.
        np.testing.assert_allclose(transforms[0] @ frames[1][1], np.eye(4), atol=1e-14)
        np.testing.assert_allclose(transforms[1] @ transforms[0] @ frames[2][1], np.eye(4), atol=1e-14)
        invalid = camera_frames([{"id": "left"}, {"id": "right"},
                                 {"id": "top", "T_cn_cnm1": transforms[1].tolist()}])
        self.assertIsNone(invalid[1][1])
        self.assertIsNone(invalid[2][1])

    def test_observed_rays_recover_independent_forward_projection(self):
        points = np.array([[-1.1, -.65, 1.], [.8, -.5, 1.], [.2, .1, 1.], [0., 0., 1.]])
        expected = points / np.linalg.norm(points, axis=1)[:, None]
        matrix = np.array([[2360., 0., 1920.], [0., 2330., 1080.], [0., 0., 1.]])
        for model, alpha, distortion in (
                ("pinhole-equi", 0., [.02, -.005, .001, -.0001]),
                ("pinhole-opencv-fisheye", .17, [.02, -.005, .001, -.0001]),
                ("pinhole-radtan8", 0., [6.1, 1.9, .00001, -.00008, -.03, 6.55, 4.4, .2])):
            with self.subTest(model=model):
                camera = {"id": "left", "model": model, "resolution": [3840, 2160],
                          "intrinsics": [2360., 2330., 1920., 1080.] + ([alpha] if alpha else []),
                          "distortion_coeffs": distortion}
                if model == "pinhole-radtan8":
                    pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), matrix, np.array(distortion))
                else:
                    pixels, _ = cv2.fisheye.projectPoints(points.reshape(-1, 1, 3), np.zeros(3),
                                                         np.zeros(3), matrix, np.array(distortion), alpha=alpha)
                actual = observation_rays(camera, pixels)
                np.testing.assert_allclose(actual, expected, atol=1e-10, rtol=0.)

    def test_plots_exclude_rejected_frames_and_removed_corners(self):
        good = {"used": True, "measurement_px": [10., 20.], "residual_px": [3., 4.]}
        removed = dict(good, used=False, residual_px=[1e6, 1e6])
        camera = {"frames": [{"used": True, "corners": [good, removed]},
                              {"used": False, "corners": [good]}]}
        pixels, residuals, indices, rms = camera_observations(camera)
        np.testing.assert_array_equal(pixels, [[10., 20.]])
        np.testing.assert_array_equal(residuals, [[3., 4.]])
        np.testing.assert_array_equal(indices, [0])
        np.testing.assert_array_equal(rms, [5.])

    def test_summary_only_report_does_not_invent_missing_plots(self):
        metrics = {"calibration_type": "cameras", "cameras": {
            "left": {"model": "pinhole-equi", "resolution": [640, 480], "used_frames": 1,
                     "selected_frames": 1, "intrinsics": [220., 225., 320., 240.],
                     "distortion_coeffs": [0., 0., 0., 0.], "reprojection": {"count": 0},
                     "frames": []}}, "stereo_pairs": {}, "imus": {}}
        assessment = {"status": "not_configured", "reference_grading": {"status": "unavailable"}, "rules": []}
        files = ['visualizations/cam0_cam1/rectified_{:04d}.jpg'.format(i) for i in range(12)]
        files += ['visualizations/cam0_cam1/original_{:04d}.jpg'.format(i) for i in range(12)]
        files += ['visualizations/cam0/corners_0.jpg', 'images/undistorted.jpg']
        document, pdf = render_reports(metrics, assessment, files)
        import re
        referenced = re.findall(r'<img[^>]+src="(visualizations/[^\"]+)"', document)
        self.assertEqual(len(referenced), 10)
        self.assertEqual(len(set(referenced)), 10)
        self.assertEqual(referenced[1::2], sorted(referenced[1::2]))
        self.assertTrue(all('/rectified_' in path for path in referenced[1::2]))
        self.assertEqual(referenced[::2], [p.replace('/rectified_', '/original_') for p in referenced[1::2]])
        self.assertNotIn('corners_0.jpg', document)
        self.assertNotIn('undistorted.jpg', document)
        self.assertTrue(pdf.startswith(b"%PDF"))
        self.assertIn("<table>", document)
        self.assertNotIn("<pre>", document)
        for title in ("Camera system", "Estimated poses", "Polar error", "Azimuthal error", "Reprojection errors"):
            self.assertIn(title, document)
        self.assertIn("No final corner measurements and residuals were saved", document)
        self.assertIn("No final camera-to-target pose evidence was saved", document)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for i in range(12):
                for kind, offset in [('original', 0), ('rectified', 30)]:
                    path = root / 'visualizations/cam0_cam1' / ('{}_{:04d}.jpg'.format(kind, i))
                    path.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(path), np.full((12, 48, 3), i + offset, dtype=np.uint8))
            with mock.patch('kalibr_no_ros.report_plots.stereo_comparison_page', wraps=stereo_comparison_page) as pages:
                html_with_images, pdf_with_images = render_reports(metrics, assessment, files, output_dir=root)
            self.assertEqual(pages.call_count, 5)
            expected = [int(Path(path).stem.split('_')[-1]) for path in referenced[1::2]]
            for call, index in zip(pages.call_args_list, expected):
                self.assertTrue(np.all(call.args[0] == index))
                self.assertTrue(np.all(call.args[1] == index + 30))
            self.assertGreaterEqual(pdf_with_images.count(b'/Subtype /Image'), 10)
            self.assertEqual(re.findall(r'<img[^>]+src="(visualizations/[^\"]+)"', html_with_images), referenced)


if __name__ == "__main__":
    unittest.main()
