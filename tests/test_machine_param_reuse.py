"""Selected machine parameter blocks are reusable without original captures."""

from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from kalibr_no_ros.machine_param_reuse import (
    assert_compatible_camera_geometry,
    camera_seed,
    parameter_path,
    semantic_sha256,
    validate_reused_stage,
)
from kalibr_no_ros.task import TaskError, dump_yaml, load_yaml
from kalibr_no_ros.validation import load_cameras


IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]
MACHINE = {"machine_version": "EVT312", "machine_num": "02"}


def public_parameters(mono=False):
    def camera():
        return {
            "model": "pinhole-equidistant",
            "resolution": [1280, 800],
            "intrinsic": [1000.0, 1001.0, 640.0, 400.0],
            "distortion": [-0.1, 0.02, -0.003, 0.0004],
            "rms": 0.2,
            "line_delay_s": 1e-6,
            "T_base_cam": deepcopy(IDENTITY),
            "pitch": 0.0,
            "T_imu_cam": deepcopy(IDENTITY),
            "td_cam_to_imu": 0.016,
        }

    cam0, cam1 = camera(), camera()
    cam1["T_imu_cam"][0][3] = 0.3
    cam1["T_cn_cnm1"] = deepcopy(IDENTITY)
    cam1["T_cn_cnm1"][0][3] = -0.3
    rotation = [row[:3] for row in IDENTITY[:3]]
    projection = [
        [1000.0, 0.0, 640.0, 0.0],
        [0.0, 1000.0, 400.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ]
    correction = {
        "R1": rotation,
        "P1": projection,
        "R2": deepcopy(rotation),
        "P2": deepcopy(projection),
        "baseline": 0.3,
        "alignment": 0.25,
    }
    correction["P2"][0][3] = -300.0
    result = {
        **MACHINE,
        "time": "2026-10-10-19:58:00",
        "algorithm_version": "1.0.0",
        "calib_file_version": "1.0.0",
        "camera": (cam0 if mono else {
            "cam0": cam0, "cam1": cam1, "cam0_cam1_correct": correction}),
    }
    if mono:
        result["mono_baselink_calibrated"] = True
    else:
        result["stereo_imu_calibrated"] = True
        result["stereo_baselink_calibrated"] = True
        result["imu"] = {
            "imu_data_rate": 100,
            "gyro_bias": [0.001, -0.002, 0.003],
            "acc_bias": [0.01, -0.02, 0.03],
            "gyro_scale": [0.99, 0.99, 0.99],
        }
    return result


class MachineParamReuseTest(unittest.TestCase):
    def test_only_selected_stage_fields_are_projected(self):
        vision = public_parameters()
        mono = public_parameters(mono=True)
        stereo = validate_reused_stage(vision, "camera", MACHINE)
        self.assertNotIn("imu", stereo)
        self.assertNotIn("T_imu_cam", stereo["camera"]["cam0"])
        self.assertNotIn("T_base_cam", stereo["camera"]["cam0"])
        self.assertEqual(stereo["camera"]["cam0"]["line_delay_s"], 1e-6)

        imu = validate_reused_stage(vision, "camera_imu", MACHINE)
        self.assertEqual(imu["imu"]["gyro_bias"], [0.001, -0.002, 0.003])
        self.assertNotIn("gyro_scale", imu["imu"])
        self.assertEqual(imu["camera"]["cam1"]["T_imu_cam"][0][3], 0.3)

        base = validate_reused_stage(vision, "camera_baselink", MACHINE)
        self.assertEqual(set(base["camera"]["cam0"]), {"T_base_cam", "pitch"})
        usb = validate_reused_stage(mono, "mono_camera", MACHINE)
        self.assertNotIn("T_base_cam", usb["camera"])
        usb_base = validate_reused_stage(mono, "mono_camera_baselink", MACHINE)
        self.assertEqual(set(usb_base["camera"]), {"T_base_cam", "pitch"})

        stereo["camera"]["cam0"]["intrinsic"][0] = 999.0
        self.assertEqual(vision["camera"]["cam0"]["intrinsic"][0], 1000.0)

    def test_unselected_invalid_fields_do_not_claim_another_stage(self):
        document = public_parameters()
        document["camera"]["cam0"]["T_imu_cam"] = "not supplied"
        document["imu"] = None
        validate_reused_stage(document, "camera", MACHINE)
        with self.assertRaises(TaskError):
            validate_reused_stage(document, "camera_imu", MACHINE)

    def test_downstream_reuse_requires_explicit_calibrated_status(self):
        for mono, stage, flag in (
                (False, "camera_imu", "stereo_imu_calibrated"),
                (False, "camera_baselink", "stereo_baselink_calibrated"),
                (True, "mono_camera_baselink", "mono_baselink_calibrated")):
            with self.subTest(stage=stage):
                document = public_parameters(mono=mono)
                document[flag] = False
                with self.assertRaisesRegex(TaskError, flag):
                    validate_reused_stage(document, stage, MACHINE)
                document.pop(flag)
                with self.assertRaisesRegex(TaskError, flag):
                    validate_reused_stage(document, stage, MACHINE)

    def test_baselink_is_not_corrected_again_or_rejected_for_zero_values(self):
        document = public_parameters(mono=True)
        document["camera"]["T_base_cam"] = [
            [-1.0, 0.0, 0.0, -0.21306],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.30887],
            [0.0, 0.0, 0.0, 1.0],
        ]
        patch = validate_reused_stage(document, "mono_camera_baselink", MACHINE)
        self.assertEqual(patch["camera"]["T_base_cam"], document["camera"]["T_base_cam"])
        self.assertEqual(patch["camera"]["pitch"], 0.0)
        document["camera"]["T_base_cam"] = deepcopy(IDENTITY)
        self.assertEqual(validate_reused_stage(
            document, "mono_camera_baselink", MACHINE)["camera"]["T_base_cam"], IDENTITY)

    def test_header_machine_and_version_are_required(self):
        for field, value in (
                ("machine_num", "12"), ("machine_num", 2),
                ("machine_version", "OTHER"), ("calib_file_version", 1),
                ("algorithm_version", "0.9.0"), ("time", "not a time")):
            with self.subTest(field=field, value=value):
                document = public_parameters()
                document[field] = value
                with self.assertRaises(TaskError):
                    validate_reused_stage(document, "camera", MACHINE)

    def test_required_geometry_and_numeric_values_are_checked(self):
        invalid = (
            ("intrinsic", [1000.0, 1000.0]),
            ("intrinsic", [-1.0, 1000.0, 640.0, 400.0]),
            ("distortion", [0.0, 0.0, 0.0, float("nan")]),
            ("resolution", [1280]),
            ("resolution", [1280, True]),
            ("rms", -0.2),
            ("line_delay_s", float("inf")),
        )
        for field, value in invalid:
            with self.subTest(field=field, value=value):
                document = public_parameters()
                document["camera"]["cam0"][field] = value
                with self.assertRaises(TaskError):
                    validate_reused_stage(document, "camera", MACHINE)

    def test_invalid_rotation_reflection_and_homogeneous_row_are_rejected(self):
        for entry, value in (((0, 0), 2.0), ((0, 0), -1.0), ((3, 0), 0.1)):
            document = public_parameters()
            row, column = entry
            document["camera"]["cam1"]["T_cn_cnm1"][row][column] = value
            with self.subTest(entry=entry, value=value):
                with self.assertRaises(TaskError):
                    validate_reused_stage(document, "camera", MACHINE)

    def test_camera_imu_chain_biases_and_rate_are_checked(self):
        document = public_parameters()
        document["camera"]["cam1"]["T_imu_cam"][0][3] = 0.4
        with self.assertRaisesRegex(TaskError, "T_cn_cnm1"):
            validate_reused_stage(document, "camera_imu", MACHINE)
        for field, value in (("imu_data_rate", 0), ("gyro_bias", [0.1, 0.2]),
                             ("acc_bias", [False, 0.0, 0.0])):
            document = public_parameters()
            document["imu"][field] = value
            with self.subTest(field=field):
                with self.assertRaises(TaskError):
                    validate_reused_stage(document, "camera_imu", MACHINE)

    def test_native_seed_is_accepted_by_existing_low_level_reader(self):
        for mono, stage in ((False, "camera"), (True, "mono_camera")):
            with self.subTest(mono=mono), tempfile.TemporaryDirectory() as temporary:
                patch = validate_reused_stage(public_parameters(mono), stage, MACHINE)
                seed = camera_seed(patch, mono=mono)
                self.assertEqual(seed["calibration_type"], "cameras")
                self.assertNotIn("imus", seed)
                self.assertNotIn("T_imu_cam", seed["cameras"][0])
                self.assertNotIn("shutter", seed["cameras"][0])
                self.assertNotIn("rms", seed["cameras"][0])
                path = Path(temporary) / "已有参数.yaml"
                dump_yaml(seed, path)
                cameras = load_cameras({
                    "job": "camera_imu_calibration",
                    "dataset": {"type": "bag", "path": "unused.bag"},
                    "camera_calibration": {"path": str(path)},
                })
                self.assertEqual(len(cameras), 1 if mono else 2)
                self.assertEqual(cameras[0]["distortion_coeffs"], patch["camera"]["distortion"]
                                 if mono else patch["camera"]["cam0"]["distortion"])

    def test_seed_geometry_is_independent_from_quality_and_line_time(self):
        patch = validate_reused_stage(public_parameters(), "camera", MACHINE)
        original = camera_seed(patch)
        patch["camera"]["cam0"]["rms"] += 0.5
        patch["camera"]["cam0"]["line_delay_s"] *= 2.0
        patch["camera"]["cam0_cam1_correct"]["alignment"] += 0.5
        self.assertEqual(camera_seed(patch), original)

    def test_radtan8_and_fisheye_parameter_dimensions_are_preserved(self):
        for model, intrinsics, distortion in (
                ("pinhole-radtan8", [1000.0, 1001.0, 640.0, 400.0],
                 [float(index) / 100 for index in range(8)]),
                ("pinhole-opencv-fisheye", [1000.0, 1001.0, 640.0, 400.0, 0.02],
                 [-0.1, 0.02, -0.003, 0.0004])):
            with self.subTest(model=model):
                document = public_parameters(mono=True)
                document["camera"].update({"model": model, "intrinsic": intrinsics,
                                           "distortion": distortion})
                patch = validate_reused_stage(document, "mono_camera", MACHINE)
                native = camera_seed(patch, mono=True)["cameras"][0]
                self.assertEqual(native["intrinsics"], intrinsics)
                self.assertEqual(native["distortion_coeffs"], distortion)
                document["camera"]["distortion"] = distortion[:-1]
                with self.assertRaises(TaskError):
                    validate_reused_stage(document, "mono_camera", MACHINE)

    def test_geometry_comparison_allows_only_baseline_refinement(self):
        first = validate_reused_stage(public_parameters(), "camera", MACHINE)
        second = deepcopy(first)
        second["camera"]["cam1"]["T_cn_cnm1"][0][3] -= 0.001
        assert_compatible_camera_geometry(first, second)
        second["camera"]["cam0"]["intrinsic"][0] += 1.0
        with self.assertRaisesRegex(TaskError, "cam0"):
            assert_compatible_camera_geometry(first, second)

    def test_fixed_paths_and_semantic_signatures(self):
        root = Path("/tmp/复用结果")
        self.assertEqual(parameter_path(root, "camera"),
                         root / "calib_params/vision/stereo_imu_calib.yaml")
        self.assertEqual(parameter_path(root, "mono_camera_baselink"),
                         root / "calib_params/mono_camera/usb_mono_calib.yaml")
        with self.assertRaises(TaskError):
            parameter_path(root, "unknown")
        first = public_parameters()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "params.yaml"
            dump_yaml(first, path)
            loaded = load_yaml(path)
        self.assertEqual(semantic_sha256(first), semantic_sha256(loaded))
        changed = deepcopy(first)
        changed["camera"]["cam0"]["intrinsic"][0] += 1.0
        self.assertNotEqual(semantic_sha256(first), semantic_sha256(changed))


if __name__ == "__main__":
    unittest.main()
