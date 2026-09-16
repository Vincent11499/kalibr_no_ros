"""Focused contracts for the public camera rolling-shutter command adapters."""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

from kalibr_no_ros.rolling_shutter import CAMERA_RS_JOB
from kalibr_no_ros.task import (
    TaskError,
    _native_rs_camera_arguments,
    _run_task_staged,
    dump_yaml,
    load_task,
    load_yaml,
)
from kalibr_no_ros.validation import validate_task


def _task(camera_count=1):
    return {
        "schema_version": "1.0.0",
        "kind": "calibration_task",
        "job": CAMERA_RS_JOB,
        "dataset": {"type": "bag", "path": "input.bag"},
        "target": {
            "type": "aprilgrid",
            "parameters": {
                "tagRows": 3,
                "tagCols": 3,
                "tagSize": 0.04,
                "tagSpacing": 0.3,
                "tagStartId": 0,
            },
        },
        "cameras": [
            {"topic": "/cam{}/image_raw".format(index),
             "model": "pinhole-equi"}
            for index in range(camera_count)
        ],
        "rolling_shutter": {
            "cam{}".format(index): {
                "line_delay_s": 0.0,
                "estimate": True,
                "max_abs_line_delay_s": 2e-5,
            }
            for index in range(camera_count)
        },
        "calibration": {},
    }


class _ImageDataset:
    def __init__(self, records):
        self.index = records
        self.get_calls = 0

    def __enter__(self):
        return self

    def __exit__(self, unused_type, unused_value, unused_traceback):
        return False

    def get_by_entry(self, entry):
        self.get_calls += 1
        return entry


class _BagReader:
    def __init__(self, height=100):
        records = [
            SimpleNamespace(
                header_timestamp_ns=1_000_000_000 + index * 1_000_000_000,
                image=np.zeros((height, 120), dtype=np.uint8))
            for index in range(2)
        ]
        self.dataset = _ImageDataset(records)

    def index_images(self, unused_topic):
        return self.dataset

    @staticmethod
    def _crop(records, unused_interval):
        return records

    @staticmethod
    def _decimate(records, unused_frequency):
        return records


class RollingShutterInputContractTest(unittest.TestCase):
    def _load(self, root, document, name="task.yaml"):
        path = Path(root) / name
        dump_yaml(document, path)
        return path, load_task(path)

    def test_bag_camera_ids_are_materialized_as_positional_cam_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            for job in ("camera_calibration", CAMERA_RS_JOB):
                with self.subTest(job=job):
                    document = _task(2)
                    document["job"] = job
                    if job == "camera_calibration":
                        document.pop("rolling_shutter")
                    unused_path, task = self._load(
                        directory, document,
                        "{}.yaml".format(job))
                    self.assertEqual(
                        [camera["id"] for camera in task["cameras"]],
                        ["cam0", "cam1"])
                    if job == CAMERA_RS_JOB:
                        self.assertEqual(
                            set(task["rolling_shutter"]), {"cam0", "cam1"})

    def test_native_backend_contract_fails_before_dataset_validation(self):
        document = _task(2)
        document["output"] = {"save_diagnostics": True}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path, task = self._load(root, document)
            output = root / "stage"
            with self.assertRaisesRegex(TaskError, "exactly one camera"):
                _run_task_staged(
                    root, path, output, CAMERA_RS_JOB, _task=task,
                    _rs_solver_backend="native")

            self.assertFalse((output / "validation.json").exists())
            resolved = load_yaml(output / "task_resolved.yaml")
            self.assertEqual(resolved["solver_backend"], "native")
            manifest = json.loads(
                (output / "run_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["status"], "input_failed")
            self.assertEqual(manifest["solver_backend"], "native")

    def test_native_public_default_passes_eighty_iterations(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            unused_path, task = self._load(root, _task())
            with mock.patch(
                    "kalibr_no_ros.task._selected_frame_rate_hz",
                    return_value=2.0):
                arguments = _native_rs_camera_arguments(
                    task, root / "dataset.bag", root / "target.yaml",
                    root, {})
        index = arguments.index("--max-iter")
        self.assertEqual(arguments[index + 1], "80")

    def test_time_padding_is_checked_from_first_decoded_image_height(self):
        document = _task()
        document["calibration"]["time_padding_s"] = 0.1
        document["rolling_shutter"]["cam0"][
            "max_abs_line_delay_s"] = 0.002
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path, unused_task = self._load(root, document)
            reader = _BagReader(height=100)
            with mock.patch(
                    "kalibr_no_ros.validation.open_dataset",
                    return_value=reader):
                report = validate_task(path)

            native_reader = _BagReader(height=100)
            with mock.patch(
                    "kalibr_no_ros.validation.open_dataset",
                    return_value=native_reader):
                native_report = validate_task(
                    path, solver_backend="native")

        self.assertEqual(report["status"], "failed")
        self.assertIn("time_padding_s", report["errors"][0])
        self.assertIn("image_height=100", report["errors"][0])
        self.assertEqual(reader.dataset.get_calls, 1)
        self.assertEqual(native_report["status"], "passed")
        self.assertIn("native backend", native_report["note"])


if __name__ == "__main__":
    unittest.main()
