import tempfile
import unittest
from pathlib import Path
from unittest import mock
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_no_ros.machine import runner as machine_runner
from kalibr_no_ros.machine.task_templates import build_task
from kalibr_no_ros.task.document import TaskError, dump_yaml
from kalibr_no_ros.task.validation import load_cameras, load_imu


PROFILE = ROOT / "config" / "mechine" / "EVT320"


def _camera(camera_id, topic, *, chained=False):
    value = {
        "id": camera_id,
        "rostopic": topic,
        "camera_model": "pinhole",
        "distortion_model": "equidistant",
        "intrinsics": [100.0, 100.0, 320.0, 240.0],
        "distortion_coeffs": [0.0, 0.0, 0.0, 0.0],
        "resolution": [640, 480],
    }
    if chained:
        value["T_cn_cnm1"] = [
            [1.0, 0.0, 0.0, 0.1],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    return value


class MachineBagTaskTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.camera_result = self.root / "camera.yaml"
        dump_yaml({
            "schema_version": "1.0.0",
            "kind": "calibration_result",
            "calibration_type": "cameras",
            "cameras": [
                _camera("cam0", "/left/image_raw"),
                _camera("cam1", "/right/image_raw", chained=True),
            ],
        }, self.camera_result)

    def test_camera_and_mono_bag_tasks_receive_explicit_topics(self):
        stereo = build_task(
            PROFILE, "camera", self.root / "stereo.bag", "stereo",
            dataset_type="bag", bag_topics={
                "cam0": "/left/image_raw",
                "cam1": "/right/image_raw",
            })
        mono = build_task(
            PROFILE, "mono_camera", self.root / "usb_bag", "usb",
            dataset_type="bag", bag_topics={"cam0": "/usb/image_raw"})

        self.assertEqual(stereo["dataset"]["type"], "bag")
        self.assertEqual(
            [camera["topic"] for camera in stereo["cameras"]],
            ["/left/image_raw", "/right/image_raw"])
        self.assertEqual(mono["cameras"][0]["topic"], "/usb/image_raw")

        directory = build_task(
            PROFILE, "camera", self.root / "stereo_directory", "directory")
        self.assertEqual(directory["dataset"]["type"], "directory")
        self.assertTrue(all("topic" not in camera
                            for camera in directory["cameras"]))

    def test_camera_imu_bag_topics_explicitly_remap_camera_result(self):
        topics = {
            "cam0": "/bag/left/image_raw",
            "cam1": "/bag/right/image_raw",
            "imu0": "/imu/data",
        }
        for mode in ("standard", "rolling_shutter"):
            with self.subTest(mode=mode):
                task = build_task(
                    PROFILE, "camera_imu", self.root / "stereo_imu.bag", "imu",
                    camera_result=self.camera_result, dataset_type="bag",
                    bag_topics=topics, mode=mode)

                self.assertEqual(task["dataset"]["type"], "bag")
                self.assertEqual(task["imus"][0]["rostopic"], "/imu/data")
                self.assertEqual(task["camera_topics"], {
                    "cam0": "/bag/left/image_raw",
                    "cam1": "/bag/right/image_raw",
                })
                cameras = load_cameras(task)
                self.assertEqual(
                    [camera["topic"] for camera in cameras],
                    ["/bag/left/image_raw", "/bag/right/image_raw"])
                self.assertEqual(
                    [camera["rostopic"] for camera in cameras],
                    ["/left/image_raw", "/right/image_raw"])
                self.assertEqual(cameras[1]["T_cn_cnm1"], _camera(
                    "cam1", "/right/image_raw", chained=True)["T_cn_cnm1"])

    def test_bag_topics_are_complete_unique_and_not_used_by_baselink(self):
        for topics in (
                None,
                {"cam0": "/same", "cam1": "/same"},
                {"cam0": "/left/image_raw"},
                {"cam0": "/left/image_raw", "cam1": "/right/image_raw",
                 "extra": "/extra"}):
            with self.subTest(topics=topics):
                with self.assertRaisesRegex(TaskError, "bag_topics"):
                    build_task(
                        PROFILE, "camera", self.root / "stereo.bag", "stereo",
                        dataset_type="bag", bag_topics=topics)

        with self.assertRaisesRegex(TaskError, "BaseLink.*directory"):
            build_task(
                PROFILE, "camera_baselink", self.root / "baselink.bag",
                "baselink", camera_result=self.camera_result,
                camera_baselink={}, dataset_type="bag")

    def test_task_imu_topic_override_allows_same_and_rejects_conflict(self):
        noise = {
            "schema_version": "1.0.0",
            "kind": "imu_configuration",
            "update_rate": 100.0,
            "accelerometer_noise_density": 0.01,
            "accelerometer_random_walk": 0.001,
            "gyroscope_noise_density": 0.001,
            "gyroscope_random_walk": 0.0001,
        }
        noise_path = self.root / "imu.yaml"
        dump_yaml(noise, noise_path)
        task = {
            "_config_dir": str(self.root),
            "dataset": {"type": "bag", "path": str(self.root / "data.bag")},
            "imus": [],
        }
        block = {
            "id": "imu0", "path": "imu.yaml", "model": "calibrated",
            "rostopic": "/imu/data",
        }
        task["imus"] = [block]

        self.assertEqual(load_imu(task, block)["rostopic"], "/imu/data")
        dump_yaml(dict(noise, rostopic="/imu/data"), noise_path)
        self.assertEqual(load_imu(task, block)["rostopic"], "/imu/data")
        dump_yaml(dict(noise, rostopic="/other/imu"), noise_path)
        with self.assertRaisesRegex(TaskError, "conflicts with IMU.rostopic"):
            load_imu(task, block)

    def test_dataset_recheck_reuses_bound_bag_topics(self):
        record = {
            "name": "stereo_261124_1200",
            "path": str(self.root / "stereo.bag"),
            "role": "stereo",
            "dataset_type": "bag",
            "bag_topics": {
                "cam0": "/left/image_raw",
                "cam1": "/right/image_raw",
            },
        }
        with mock.patch.object(
                machine_runner, "_fingerprint_dataset",
                return_value=(record, {"record": record}, {})) as fingerprint:
            machine_runner._verify_stage_inputs_unchanged(record)

        fingerprint.assert_called_once_with(
            record["path"], "stereo", bag_topics=record["bag_topics"],
            progress=mock.ANY)


if __name__ == "__main__":
    unittest.main()
