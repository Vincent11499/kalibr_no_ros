"""Real OpenCV/public-result conversion without native calibration imports."""

from contextlib import redirect_stdout, redirect_stderr
import io
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/python"))

from kalibr_no_ros.cli import main as cli_main
from kalibr_no_ros.conversion import converter_modules, main as convert
from kalibr_no_ros.task import TaskError, dump_yaml, load_yaml


class ConversionV1Test(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.zero, _ = converter_modules(False)
        self.full, _ = converter_modules(True)
        self.K = np.asarray([[421.31234567890123, 0., 319.24567890123456],
                             [0., 420.79876543210987, 241.19876543210987], [0., 0., 1.]])
        self.D = np.asarray([-0.031234567890123456, 0.004212345678901234,
                             -0.0007123456789012345, 0.000052123456789012345])
        self.rotation = cv2.Rodrigues(np.asarray([0.12, -0.08, 0.045]))[0]
        self.translation = np.asarray([[-0.11812345678901234], [0.0043], [0.0071]])

    def camera(self, topic="/cam0", coefficients=None, model="opencv_fisheye"):
        return self.zero.OpenCvCamera(self.K, self.D if coefficients is None else coefficients,
                                     (640, 480), model, topic.strip("/"), topic)

    def run_conversion(self, *arguments, full=False):
        with redirect_stdout(io.StringIO()):
            self.assertEqual(convert(list(map(str, arguments)), full_fisheye=full), 0)

    def assert_metadata(self, path, kind):
        self.assertTrue(path.read_text(encoding="utf-8").startswith("%YAML:1.0"))
        storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        try:
            self.assertEqual(storage.getNode("schema_version").string(), "1.0.0")
            self.assertEqual(storage.getNode("software_version").string(), "1.0.0")
            self.assertEqual(storage.getNode("kind").string(), kind)
        finally:
            storage.release()

    def test_mono_import_and_export_preserve_double_values_and_equidistant_model(self):
        source, result = self.root / "vendor.yaml", self.root / "calibration.yaml"
        self.zero.write_opencv_camera(self.camera(), source)
        self.run_conversion("import-mono", "--input", source, "--output", result)
        document = load_yaml(result)
        self.assertEqual(document["schema_version"], "1.0.0")
        self.assertEqual(document["kind"], "calibration_result")
        camera = document["cameras"][0]
        self.assertEqual(camera["id"], "cam0")
        self.assertEqual(camera["distortion_model"], "equidistant")
        self.assertEqual(camera["intrinsics"], [self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]])
        self.assertEqual(camera["distortion_coeffs"], self.D.tolist())
        self.assertIn("intrinsics: [", result.read_text(encoding="utf-8"))
        prefix = self.root / "export.yaml"
        self.run_conversion("export", "--input", result, "--output-prefix", prefix)
        exported = self.root / "export-cam0-opencv.yaml"
        restored = self.zero.read_opencv_camera(exported)
        np.testing.assert_array_equal(restored.K, self.K)
        np.testing.assert_array_equal(restored.D, self.D)
        self.assert_metadata(exported, "opencv_camera")

    def test_stereo_direction_composition_and_multi_file_export(self):
        source, result = self.root / "stereo.yaml", self.root / "calibration.yaml"
        self.zero.write_opencv_stereo(self.zero.OpenCvStereo(
            self.camera(), self.camera("/cam1"), self.rotation, self.translation), source)
        self.run_conversion("import-stereo", "--stereo", source, "--output", result)
        camera = load_yaml(result)["cameras"][1]
        transform = np.asarray(camera["T_cn_cnm1"])
        np.testing.assert_array_equal(transform[:3, :3], self.rotation)
        np.testing.assert_array_equal(transform[:3, 3:4], self.translation)
        np.testing.assert_allclose(transform @ [0., 0., 0., 1.], np.r_[self.translation[:, 0], 1.])
        self.run_conversion("export", "--input", result, "--output-prefix", self.root / "export")
        exported = self.root / "export-cam0-cam1-opencv-stereo.yaml"
        restored = self.zero.read_opencv_stereo(exported)
        np.testing.assert_array_equal(restored.R, self.rotation)
        np.testing.assert_array_equal(restored.T, self.translation)
        self.assert_metadata(exported, "opencv_stereo")
        self.assertTrue((self.root / "export-cam0-opencv.yaml").is_file())
        self.assertTrue((self.root / "export-cam1-opencv.yaml").is_file())

    def test_reverse_transform_and_millimeter_translation_are_explicit(self):
        source, result = self.root / "reverse.yaml", self.root / "calibration.yaml"
        inverse_R, inverse_T = self.zero.invert_stereo_transform(self.rotation, self.translation)
        self.zero.write_opencv_stereo(self.zero.OpenCvStereo(
            self.camera(), self.camera("/cam1"), inverse_R, inverse_T * 1000.), source)
        self.run_conversion("import-stereo", "--stereo", source, "--output", result,
                            "--transform-direction", "cam1-to-cam0", "--translation-unit", "mm")
        transform = np.asarray(load_yaml(result)["cameras"][1]["T_cn_cnm1"])
        np.testing.assert_allclose(transform[:3, :3], self.rotation, atol=2e-15, rtol=0.)
        np.testing.assert_allclose(transform[:3, 3:4], self.translation, atol=2e-15, rtol=0.)

    def test_radtan8_retains_eight_coefficients(self):
        source, result = self.root / "rational.yaml", self.root / "calibration.yaml"
        coefficients = np.asarray([0.1, -0.02, 0.003, -0.004, 0.005, 0.006, -0.007, 0.008])
        self.zero.write_opencv_camera(self.camera(coefficients=coefficients, model="radtan8"), source)
        self.run_conversion("import-mono", "--input", source, "--output", result, "--distortion-model", "radtan8")
        self.assertEqual(load_yaml(result)["cameras"][0]["distortion_coeffs"], coefficients.tolist())
        self.run_conversion("export", "--input", result, "--output-prefix", self.root / "export")
        restored = self.zero.read_opencv_camera(self.root / "export-cam0-opencv.yaml")
        np.testing.assert_array_equal(restored.D, coefficients)

    def test_full_fisheye_cli_preserves_nonzero_alpha_and_skew(self):
        source, result = self.root / "full.yaml", self.root / "calibration.yaml"
        alpha = 0.027123456789012345
        intrinsic = self.K.copy()
        intrinsic[0, 1] = intrinsic[0, 0] * alpha
        self.full.write_opencv_camera(self.full.OpenCvFisheyeCamera(
            intrinsic, self.D, (640, 480), alpha, "cam0", "/cam0"), source)
        with redirect_stdout(io.StringIO()):
            self.assertEqual(cli_main(["convert", "camera", "--full-fisheye", "import-mono",
                                       "--input", str(source), "--output", str(result)]), 0)
        camera = load_yaml(result)["cameras"][0]
        self.assertEqual(camera["camera_model"], "pinhole_opencv_fisheye")
        self.assertEqual(len(camera["intrinsics"]), 5)
        self.assertEqual(camera["intrinsics"][4], alpha)
        self.run_conversion("export", "--input", result, "--output-prefix", self.root / "export", full=True)
        exported = self.root / "export-cam0-opencv.yaml"
        restored = self.full.read_opencv_camera(exported)
        np.testing.assert_array_equal(restored.K, intrinsic)
        np.testing.assert_array_equal(restored.D, self.D)
        self.assert_metadata(exported, "opencv_camera")

    def test_old_project_versions_and_unversioned_camchain_are_rejected(self):
        source = self.root / "old.yaml"
        for version in (1, 2, True, "2"):
            with self.subTest(version=version):
                dump_yaml({"schema_version": version, "kind": "calibration_result", "cameras": []}, source)
                with self.assertRaisesRegex(TaskError, "schema_version"):
                    convert(["export", "--input", str(source), "--output-prefix", str(self.root / "out")])
        self.zero.write_kalibr_camchain(self.zero.opencv_mono_to_camchain(self.camera()), source)
        with self.assertRaisesRegex(TaskError, "schema_version"):
            convert(["export", "--input", str(source), "--output-prefix", str(self.root / "out")])
        self.zero.write_opencv_camera(self.camera(), source)
        with source.open("a", encoding="utf-8") as stream:
            stream.write('schema_version: "2"\n')
        with self.assertRaisesRegex(TaskError, "schema_version"):
            convert(["import-mono", "--input", str(source), "--output", str(self.root / "result.yaml")])
        self.assertFalse((self.root / "result.yaml").exists())

    def test_existing_output_prevents_all_export_files_and_preserves_import_target(self):
        source, result = self.root / "source.yaml", self.root / "calibration.yaml"
        self.zero.write_opencv_stereo(self.zero.OpenCvStereo(
            self.camera(), self.camera("/cam1"), self.rotation, self.translation), source)
        self.run_conversion("import-stereo", "--stereo", source, "--output", result)
        before = result.read_bytes()
        with self.assertRaisesRegex(TaskError, "already exists"):
            convert(["import-stereo", "--stereo", str(source), "--output", str(result)])
        self.assertEqual(result.read_bytes(), before)
        protected = self.root / "out-cam0-cam1-opencv-stereo.yaml"
        protected.write_text("user file\n", encoding="utf-8")
        with self.assertRaisesRegex(TaskError, "already exists"):
            convert(["export", "--input", str(result), "--output-prefix", str(self.root / "out")])
        self.assertEqual(protected.read_text(encoding="utf-8"), "user file\n")
        self.assertFalse((self.root / "out-cam0-opencv.yaml").exists())
        self.assertFalse((self.root / "out-cam1-opencv.yaml").exists())

    def test_help_is_forwarded_for_both_converter_variants(self):
        for full in (False, True):
            arguments = ["convert", "camera"] + (["--full-fisheye"] if full else []) + ["--help"]
            output = io.StringIO()
            with redirect_stdout(output), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as stopped:
                    cli_main(arguments)
            self.assertEqual(stopped.exception.code, 0)
            self.assertIn("import-mono", output.getvalue())
            self.assertIn("import-stereo", output.getvalue())


if __name__ == "__main__":
    unittest.main()
