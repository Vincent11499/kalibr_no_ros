"""Machine-facing aggregate calibration parameter files."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from kalibr_no_ros.evaluation import stereo_geometry
from kalibr_no_ros.filesystem import file_sha256
from kalibr_no_ros.machine_calib_params import (
    build_calib_params,
    machine_identity,
    write_calib_param,
)
from kalibr_no_ros.task import TaskError, dump_yaml, load_yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "config" / "mechine" / "workflow_v1.0.0"
IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]
BASE = [
    [1.0, 0.0, 0.0, 0.2],
    [0.0, 1.0, 0.0, 0.1],
    [0.0, 0.0, 1.0, 0.3],
    [0.0, 0.0, 0.0, 1.0],
]
IMU_STEREO = [
    [1.0, 0.0, 0.0, -0.35],
    [0.0, 1.0, 0.0, 0.02],
    [0.0, 0.0, 1.0, 0.01],
    [0.0, 0.0, 0.0, 1.0],
]


class MachineCalibParamsTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.machine = self.root / "EVT312-02"
        self.output = self.machine / "result"
        self.output.mkdir(parents=True)

    @staticmethod
    def camera(camera_id, *, rms, shutter=None):
        value = {
            "id": camera_id,
            "camera_model": "pinhole",
            "distortion_model": "equidistant",
            "intrinsics": [1000.0, 1001.0, 640.0, 400.0],
            "distortion_coeffs": [-0.1, 0.02, -0.003, 0.0004],
            "resolution": [1280, 800],
            "rms": rms,
        }
        if shutter is not None:
            value["shutter"] = {"line_delay_s": shutter}
        return value

    def result(self, name, calibration_type, cameras, *, imus=None,
               metrics=None):
        side = self.output / name
        side.mkdir()
        path = side / (name + ".yaml")
        document = {
            "schema_version": "1.0.0",
            "kind": "calibration_result",
            "calibration_type": calibration_type,
            "cameras": cameras,
        }
        if imus is not None:
            document["imus"] = imus
        dump_yaml(document, path)
        if metrics is not None:
            (side / "metrics.json").write_text(
                json.dumps(metrics), encoding="utf-8")
        return {
            "result": str(path.resolve()),
            "files": {"result": file_sha256(path)},
        }

    def stereo(self):
        cam0 = self.camera("cam0", rms=0.21, shutter=1.0e-6)
        cam1 = self.camera("cam1", rms=0.22, shutter=1.1e-6)
        cam1["T_cn_cnm1"] = BASE
        cam1["alignment"] = 0.31
        return self.result(
            "stereo", "cameras", [cam0, cam1],
            metrics=self.evaluation_metrics())

    def mono(self):
        return self.result(
            "mono", "cameras", [self.camera("cam0", rms=0.41)])

    def baselink(self, name="baselink"):
        camera = self.camera("cam0", rms=9.9)
        camera.update({"T_base_camera": BASE, "pitch": 8.5})
        return self.result(name, "cameras_baselink", [camera])

    @staticmethod
    def bias_metrics():
        def spline(values):
            return {
                "status": "available",
                "axes": {
                    axis: {"status": "available", "mean": value}
                    for axis, value in zip(("x", "y", "z"), values)
                },
            }
        return {"imus": {"imu0": {
            "gyro": {"bias_spline": spline([0.01, 0.02, 0.03])},
            "accel": {"bias_spline": spline([0.1, -0.2, 0.3])},
        }}}

    @staticmethod
    def evaluation_metrics(calibration_type="cameras"):
        return {
            "schema_version": "1.0.0",
            "kind": "calibration_metrics",
            "calibration_type": calibration_type,
            "evaluation_options": {"rectification": {
                "balance": 0.0,
                "fov_scale": 1.0,
                "size": None,
            }},
        }

    def camera_imu(self, *, metrics=True):
        cameras = []
        for index, camera_id in enumerate(("cam0", "cam1")):
            camera = self.camera(camera_id, rms=0.5 + index, shutter=2.0e-6 + index * 1e-7)
            camera.update({
                "timeshift_cam_imu": 0.013 + index * 0.001,
                "T_imu_cam": BASE,
            })
            if camera_id == "cam1":
                camera["T_cn_cnm1"] = IMU_STEREO
                camera["alignment"] = 0.41
            cameras.append(camera)
        metric_value = self.evaluation_metrics("camera_imu")
        metric_value.update(self.bias_metrics())
        return self.result(
            "camera_imu", "camera_imu", cameras,
            imus=[{"id": "imu0", "update_rate": 100.0}],
            metrics=metric_value if metrics else None)

    def test_machine_identity_is_strict_and_preserves_leading_zero(self):
        self.assertEqual(machine_identity(self.machine), {
            "machine_version": "EVT312", "machine_num": "02"})
        with self.assertRaisesRegex(TaskError, "EVT312-02"):
            machine_identity(self.root / "mechine-EVT31202")

    def test_primary_camera_results_create_numeric_defaults(self):
        artifacts = build_calib_params(
            self.machine, PROFILE,
            {"camera": self.stereo(), "mono_camera": self.mono()},
            "2026-10-10-19:58:00", camera_mode="rolling_shutter")
        self.assertEqual(set(artifacts), {"vision", "mono"})

        vision = artifacts["vision"]["document"]
        self.assertEqual(list(vision), [
            "machine_version", "machine_num", "time", "algorithm_version",
            "calib_file_version", "imu", "camera", "odom",
        ])
        self.assertEqual(vision["algorithm_version"], "1.0.0")
        self.assertEqual(vision["calib_file_version"], "1.0.0")
        self.assertNotIn("calib_version", vision)
        self.assertNotIn("T_BS", vision["imu"])
        self.assertEqual(vision["imu"]["gyro_bias"], [0.0, 0.0, 0.0])
        self.assertEqual(vision["imu"]["gyro_scale"], [0.9934] * 3)
        self.assertEqual(vision["imu"]["acc_scale"], [0.989362] * 3)
        self.assertEqual(vision["camera"]["cam0"]["T_base_cam"], IDENTITY)
        self.assertEqual(list(vision["camera"]["cam0"]), [
            "model", "resolution", "intrinsic", "distortion", "T_base_cam",
            "T_imu_cam", "td_cam_to_imu", "line_delay_s", "pitch", "rms",
        ])
        self.assertEqual(vision["camera"]["cam0"]["pitch"], 0.0)
        self.assertEqual(vision["camera"]["cam0"]["rms"], 0.21)
        self.assertEqual(vision["camera"]["cam0"]["T_imu_cam"], IDENTITY)
        self.assertEqual(vision["camera"]["cam0"]["line_delay_s"], 1.0e-6)
        self.assertEqual(vision["camera"]["cam1"]["T_cn_cnm1"], BASE)
        self.assertEqual(list(vision["camera"]["cam1"]), [
            "model", "resolution", "intrinsic", "distortion", "T_cn_cnm1",
            "T_imu_cam", "td_cam_to_imu", "line_delay_s", "rms",
        ])
        correction = vision["camera"]["cam0_cam1_correct"]
        self.assertEqual(list(correction), [
            "R1", "P1", "R2", "P2", "baseline", "alignment",
        ])
        self.assertEqual(len(correction["R1"]), 3)
        self.assertEqual(len(correction["P1"]), 3)
        self.assertAlmostEqual(correction["baseline"], 14 ** 0.5 / 10.0)
        self.assertEqual(correction["alignment"], 0.31)
        self.assertNotIn("T_BS", vision["odom"])
        self.assertNotIn("T_base_cam", vision["odom"])
        written = write_calib_param(self.output, artifacts["vision"])
        restored = load_yaml(written["path"])
        self.assertEqual(restored, vision)
        self.assertIsInstance(restored["machine_num"], str)
        self.assertEqual(restored["machine_num"], "02")

        mono = artifacts["mono"]["document"]
        self.assertEqual(list(mono), [
            "machine_version", "machine_num", "time", "algorithm_version",
            "calib_file_version", "camera",
        ])
        self.assertEqual(list(mono["camera"]), [
            "model", "resolution", "intrinsic", "distortion", "T_base_cam",
            "pitch", "rms",
        ])
        self.assertEqual(mono["camera"]["resolution"], [1280, 800])
        self.assertEqual(mono["camera"]["T_base_cam"], IDENTITY)
        self.assertEqual(mono["camera"]["pitch"], 0.0)
        self.assertEqual(mono["camera"]["rms"], 0.41)
        self.assertEqual(artifacts["mono"]["defaulted_fields"], [
            "camera.T_base_cam", "camera.pitch",
        ])

    def test_mono_baselink_updates_usb_extrinsics(self):
        artifacts = build_calib_params(
            self.machine, PROFILE, {
                "mono_camera": self.mono(),
                "mono_camera_baselink": self.baselink("mono_baselink"),
            }, "2026-10-11-20:01:02")
        self.assertEqual(set(artifacts), {"mono"})
        mono = artifacts["mono"]["document"]
        self.assertEqual(mono["camera"]["T_base_cam"], BASE)
        self.assertEqual(mono["camera"]["pitch"], 8.5)
        self.assertEqual(mono["camera"]["rms"], 0.41)
        self.assertEqual(artifacts["mono"]["defaulted_fields"], [])
        self.assertIn("mono_camera_baselink", artifacts["mono"]["sources"])

    def test_baselink_updates_vision_and_camera_imu_wins_rs(self):
        artifacts = build_calib_params(
            self.machine, PROFILE, {
                "camera": self.stereo(),
                "camera_baselink": self.baselink(),
                "camera_imu": self.camera_imu(),
            }, "2026-10-11-20:01:02",
            camera_mode="rolling_shutter",
            camera_imu_mode="rolling_shutter")
        vision = artifacts["vision"]["document"]
        self.assertEqual(vision["camera"]["cam0"]["T_base_cam"], BASE)
        self.assertEqual(vision["camera"]["cam0"]["pitch"], 8.5)
        self.assertEqual(vision["camera"]["cam0"]["rms"], 0.5)
        self.assertEqual(vision["camera"]["cam1"]["rms"], 1.5)
        self.assertEqual(vision["imu"]["gyro_bias"], [0.01, 0.02, 0.03])
        self.assertEqual(vision["imu"]["acc_bias"], [0.1, -0.2, 0.3])
        self.assertEqual(vision["camera"]["cam0"]["td_cam_to_imu"], 0.013)
        self.assertEqual(vision["camera"]["cam0"]["T_imu_cam"], BASE)
        self.assertEqual(vision["camera"]["cam0"]["line_delay_s"], 2.0e-6)
        self.assertEqual(vision["camera"]["cam1"]["T_cn_cnm1"], IMU_STEREO)
        self.assertAlmostEqual(
            vision["camera"]["cam0_cam1_correct"]["baseline"],
            (0.35 ** 2 + 0.02 ** 2 + 0.01 ** 2) ** 0.5)
        self.assertEqual(
            vision["camera"]["cam0_cam1_correct"]["alignment"], 0.41)
        self.assertEqual(artifacts["vision"]["defaulted_fields"], [])
        self.assertIn("camera_imu_metrics", artifacts["vision"]["sources"])

    def test_rectification_matrices_use_saved_evaluation_options(self):
        delivery = self.stereo()
        metrics_path = Path(delivery["result"]).parent / "metrics.json"
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        rectification = {
            "balance": 0.35,
            "fov_scale": 0.9,
            "size": [640, 400],
        }
        metrics["evaluation_options"]["rectification"] = rectification
        metrics_path.write_text(json.dumps(metrics), encoding="utf-8")

        vision = build_calib_params(
            self.machine, PROFILE, {"camera": delivery},
            "2026-10-10-19:58:00")["vision"]["document"]
        cameras = load_yaml(delivery["result"])["cameras"]
        expected = stereo_geometry(cameras[0], cameras[1], rectification)
        actual = vision["camera"]["cam0_cam1_correct"]
        for name in ("R1", "P1", "R2", "P2"):
            np.testing.assert_allclose(actual[name], expected[name], atol=0.0)

    def test_line_delay_requires_the_completed_rs_source(self):
        stereo = self.stereo()
        document = load_yaml(stereo["result"])
        for camera in document["cameras"]:
            camera.pop("shutter")
        dump_yaml(document, stereo["result"])
        stereo["files"]["result"] = file_sha256(stereo["result"])

        standard = build_calib_params(
            self.machine, PROFILE, {"camera": stereo},
            "2026-10-10-19:58:00")
        self.assertEqual(
            standard["vision"]["document"]["camera"]["cam0"]["line_delay_s"],
            0.0)
        with self.assertRaisesRegex(TaskError, "lacks line_delay_s"):
            build_calib_params(
                self.machine, PROFILE, {"camera": stereo},
                "2026-10-10-19:58:00",
                camera_mode="rolling_shutter")

        camera_imu = self.camera_imu()
        imu_document = load_yaml(camera_imu["result"])
        imu_document["cameras"][0].pop("shutter")
        dump_yaml(imu_document, camera_imu["result"])
        camera_imu["files"]["result"] = file_sha256(camera_imu["result"])
        with self.assertRaisesRegex(TaskError, "Camera-IMU cam0.*line_delay"):
            build_calib_params(
                self.machine, PROFILE, {
                    "camera": stereo, "camera_imu": camera_imu},
                "2026-10-10-19:58:00",
                camera_imu_mode="rolling_shutter")

    def test_completed_camera_imu_requires_available_bias_metrics(self):
        with self.assertRaisesRegex(TaskError, "metrics.json"):
            build_calib_params(
                self.machine, PROFILE, {
                    "camera": self.stereo(),
                    "camera_imu": self.camera_imu(metrics=False),
                }, "2026-10-10-19:58:00")

    def test_completed_camera_requires_rectification_metrics(self):
        delivery = self.stereo()
        metrics_path = Path(delivery["result"]).parent / "metrics.json"
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        metrics.pop("evaluation_options")
        metrics_path.write_text(json.dumps(metrics), encoding="utf-8")
        with self.assertRaisesRegex(TaskError, "evaluation_options.rectification"):
            build_calib_params(
                self.machine, PROFILE, {"camera": delivery},
                "2026-10-10-19:58:00")

    def test_inconsistent_equidistant_length_is_rejected(self):
        delivery = self.stereo()
        result = load_yaml(delivery["result"])
        result["cameras"][1]["distortion_coeffs"].append(0.5)
        dump_yaml(result, delivery["result"])
        delivery["files"]["result"] = file_sha256(delivery["result"])
        with self.assertRaisesRegex(TaskError, "four|4 values"):
            build_calib_params(
                self.machine, PROFILE, {"camera": delivery},
                "2026-10-10-19:58:00")

    def test_failed_yaml_readback_keeps_previous_parameter_file(self):
        artifact = {
            "relative_path": "vision/stereo_imu_calib.yaml",
            "document": {"value": 1},
        }
        written = write_calib_param(self.output, artifact)
        before = Path(written["path"]).read_bytes()
        replacement = dict(artifact, document={"value": 2})
        with mock.patch(
                "kalibr_no_ros.machine_calib_params.load_yaml",
                return_value={"value": "changed"}):
            with self.assertRaisesRegex(TaskError, "read-back"):
                write_calib_param(self.output, replacement)
        self.assertEqual(Path(written["path"]).read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
