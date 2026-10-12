import csv
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import yaml

from tools.aging_vibration.diagnosis import VARIANT_ORDER, build_extrinsic_variants, generate_diagnosis_report
from tools.aging_vibration.reporting import build_parameter_comparison, collect_calibrations, collect_verifications, generate_report
from tools.aging_vibration.workflow import (
    KIND,
    VERIFY_OPTIONS,
    build_verify_command,
    create_manifest,
    inspect_study,
    load_manifest,
    memory_status,
    prepare_runtime_task,
    refresh_manifest,
    representative_calibration,
    run_calibrations,
)


CALIBRATIONS = ("stereo_260929_1617", "stereo_260929_1620", "stereo_260929_1622")
TESTS = ("stereo_260929_1627", "stereo_260929_1629", "stereo_260929_1630")


def _yaml(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def _calibration(run, offset):
    result = {
        "schema_version": "1.0.0",
        "kind": "calibration_result",
        "cameras": [
            {"id": "cam0", "intrinsics": [2380 + offset, 2381 + offset, 1920, 1080], "distortion_coeffs": [-0.1, -0.02, 0.03, -0.01], "rms": 0.25 + offset / 1000},
            {"id": "cam1", "intrinsics": [2375 + offset, 2376 + offset, 1910, 1090], "distortion_coeffs": [-0.1, -0.02, 0.03, -0.01], "rms": 0.26 + offset / 1000, "alignment": 0.2 + offset / 1000, "T_cn_cnm1": [[1, 0, 0, -0.38 - offset / 10000], [0, 1, 0, 0.003], [0, 0, 1, 0.004], [0, 0, 0, 1]]},
        ],
    }
    _yaml(run["result"], result)
    metrics = {
        "cameras": {
            "cam0": {"used_frames": 20, "reprojection": {"count": 100, "rms_px": 0.25 + offset / 1000}},
            "cam1": {"used_frames": 20, "reprojection": {"count": 120, "rms_px": 0.26 + offset / 1000}},
        },
        "stereo_pairs": {"cam0_cam1": {"alignment": {"rms_px": 0.2 + offset / 1000}}},
    }
    _json(run["artifacts"] / "metrics.json", metrics)
    _json(run["artifacts"] / "run_manifest.json", {"status": "completed"})
    _yaml(run["artifacts"] / "observability.yaml", {"status": "full_rank", "quality": "weakly_observable"})


def _verification(path, base):
    cam0, cam1 = base, base + 0.01
    stereo = math.sqrt((cam0 * cam0 + cam1 * cam1) / 2)
    result = {
        "cameras": {
            "cam0": {"detected_frames": 1, "corner_count": 4, "rms_px": cam0},
            "cam1": {"detected_frames": 1, "corner_count": 4, "rms_px": cam1},
        },
        "stereo_combined_rms_px": stereo,
        "stereo": {"used_pairs": 1, "alignment": {"count": 4, "rms_px": base + 0.2}},
    }
    _json(path / "fixed_camera_validation.json", {"method": {"window": 7}, "results": {"pinhole_equi": result}})
    fields = ["calibration_label", "record_type", "camera_id", "source_index", "corner_count", "reprojection_rms_px", "reprojection_p95_px", "reprojection_max_px", "alignment_rms_px", "alignment_p95_px", "alignment_max_px"]
    path.mkdir(parents=True, exist_ok=True)
    with (path / "fixed_camera_validation.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow({"record_type": "camera_frame", "camera_id": "cam0", "source_index": 0, "reprojection_rms_px": cam0})
        writer.writerow({"record_type": "camera_frame", "camera_id": "cam1", "source_index": 0, "reprojection_rms_px": cam1})
        writer.writerow({"record_type": "stereo_pair", "source_index": 0, "reprojection_rms_px": stereo, "alignment_rms_px": base + 0.2})
    _json(path / "run_manifest.json", {"status": "completed"})


class AgingVibrationWorkflowTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / "mechine-fixture"
        dataset_manifest = {"schema_version": "1.0.0", "cameras": [{"id": "cam0", "resolution": [3840, 2160]}, {"id": "cam1", "resolution": [3840, 2160]}]}
        for dataset in (*CALIBRATIONS, *TESTS):
            group = "calibration" if dataset in CALIBRATIONS else "test"
            _yaml(self.root / "before" / group / dataset / "dataset.yaml", dataset_manifest)
        self.manifest_path = create_manifest(self.root)
        self.manifest = load_manifest(self.manifest_path)

    def tearDown(self):
        self.temporary.cleanup()

    def _evidence(self):
        for index, run in enumerate(self.manifest["phases"]["before"]["calibration_runs"]):
            converted = {"result": self.root / run["result"], "artifacts": self.root / run["artifacts"]}
            _calibration(converted, float(index))
        for index, run in enumerate(self.manifest["phases"]["before"]["test_runs"]):
            _verification(self.root / run["results"]["before"], 0.1 + index * 0.01)

    def test_init_discovers_sorted_before_only_runs(self):
        self.assertEqual(self.manifest["kind"], KIND)
        self.assertEqual(set(self.manifest["phases"]), {"before"})
        runs = self.manifest["phases"]["before"]["calibration_runs"]
        self.assertEqual([run["dataset_id"] for run in runs], list(CALIBRATIONS))
        self.assertEqual(representative_calibration(self.manifest, "before")["dataset_id"], CALIBRATIONS[0])
        self.assertEqual(len(self.manifest["phases"]["before"]["test_runs"]), 3)

    def test_old_single_calibration_manifest_is_rejected(self):
        value = yaml.safe_load(self.manifest_path.read_text(encoding="utf-8"))
        value["phases"]["before"] = {"calibration_dataset": "old", "validation_runs": []}
        _yaml(self.manifest_path, value)
        with self.assertRaisesRegex(ValueError, "calibration_runs"):
            load_manifest(self.manifest_path)

    def test_refresh_registers_new_after_phase(self):
        dataset_manifest = {"schema_version": "1.0.0", "cameras": [{"id": "cam0", "resolution": [3840, 2160]}, {"id": "cam1", "resolution": [3840, 2160]}]}
        for dataset in (*CALIBRATIONS, *TESTS):
            group = "calibration" if dataset in CALIBRATIONS else "test"
            _yaml(self.root / "after" / group / (dataset + "_after") / "dataset.yaml", dataset_manifest)
        refresh_manifest(self.manifest)
        refreshed = load_manifest(self.manifest_path)
        self.assertEqual(set(refreshed["phases"]), {"before", "after"})
        self.assertEqual(len(refreshed["phases"]["after"]["calibration_runs"]), 3)

    def test_runtime_task_has_required_full_batch_and_output_options(self):
        run = representative_calibration(self.manifest, "before")
        path = prepare_runtime_task(self.manifest, "before", run)
        task = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.assertEqual(task["calibration"]["solver_mode"], "full_batch")
        self.assertEqual(task["execution"]["detector_processes"], 8)
        self.assertEqual(task["execution"]["optimizer_threads"], 4)
        self.assertTrue(task["output"]["save_diagnostics"])
        self.assertTrue(task["output"]["save_metrics"])
        self.assertFalse(task["output"]["visualizations"]["undistortion"]["crop"])

    def test_verify_command_has_fixed_method(self):
        command = build_verify_command(self.manifest, Path("/tmp/a.yaml"), Path("/tmp/data"), Path("/tmp/out"))
        self.assertIn("pinhole_equi=/tmp/a.yaml", command)
        self.assertIn("--visualizations", command)
        for option, expected in (("--window-half-size-px", "7"), ("--max-displacement-px", "2.0"), ("--max-pairs", "30")):
            self.assertEqual(command[command.index(option) + 1], expected)
        self.assertEqual(self.manifest["kalibr"]["verify_options"], VERIFY_OPTIONS)

    @mock.patch("tools.aging_vibration.workflow._run_logged")
    def test_calibration_runs_are_serial_and_completed_runs_resume(self, runner):
        run_calibrations(self.manifest, "before", dry_run=True)
        self.assertEqual(runner.call_count, 3)
        runner.reset_mock()
        first = self.manifest["phases"]["before"]["calibration_runs"][0]
        _json(self.root / first["artifacts"] / "run_manifest.json", {"status": "completed"})
        _yaml(self.root / first["result"], {"kind": "calibration_result"})
        run_calibrations(self.manifest, "before", dry_run=True)
        self.assertEqual(runner.call_count, 2)

    @mock.patch("tools.aging_vibration.workflow._windows_free_virtual_gib", return_value=8.0)
    @mock.patch("tools.aging_vibration.workflow._linux_available_gib", return_value=20.0)
    def test_memory_status_blocks_low_windows_virtual_memory(self, _linux, _windows):
        self.assertEqual(len(memory_status(self.manifest)["errors"]), 1)

    def test_before_report_contains_all_calibrations_tests_and_charts(self):
        self._evidence()
        self.assertEqual(inspect_study(self.manifest)["status"], "ok")
        calibrations, parameters = collect_calibrations(self.manifest, ("before",))
        verifications, details = collect_verifications(self.manifest, (("before", "before"),))
        self.assertEqual((len(calibrations), len(parameters)), (3, 84))
        self.assertEqual((len(verifications), len(details)), (3, 9))
        result = generate_report(self.manifest)
        output = Path(result["output"])
        self.assertTrue(Path(result["pdf"]).read_bytes().startswith(b"%PDF"))
        self.assertEqual(len(list((output / "charts").glob("*.png"))), 4)
        self.assertTrue((output / "parameter_repeatability.csv").is_file())
        self.assertIn("三组标定", Path(result["html"]).read_text(encoding="utf-8"))

    def test_full_report_accepts_new_after_phase(self):
        self._evidence()
        dataset_manifest = {"schema_version": "1.0.0", "cameras": [{"id": "cam0", "resolution": [3840, 2160]}, {"id": "cam1", "resolution": [3840, 2160]}]}
        for dataset in (*CALIBRATIONS, *TESTS):
            group = "calibration" if dataset in CALIBRATIONS else "test"
            _yaml(self.root / "after" / group / (dataset + "_after") / "dataset.yaml", dataset_manifest)
        refresh_manifest(self.manifest)
        self.manifest = load_manifest(self.manifest_path)
        for index, run in enumerate(self.manifest["phases"]["after"]["calibration_runs"]):
            _calibration({"result": self.root / run["result"], "artifacts": self.root / run["artifacts"]}, 2.0 + index)
        for index, run in enumerate(self.manifest["phases"]["after"]["test_runs"]):
            _verification(self.root / run["results"]["before"], 0.2 + index * 0.01)
            _verification(self.root / run["results"]["after"], 0.12 + index * 0.01)
        result = generate_report(self.manifest, full=True)
        self.assertTrue(Path(result["pdf"]).is_file())
        output = Path(result["output"])
        with (output / "verification_summary.csv").open(encoding="utf-8-sig") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 9)
        with (output / "parameter_before_after_comparison.csv").open(encoding="utf-8-sig") as stream:
            comparison = list(csv.DictReader(stream))
        fx = next(row for row in comparison if row["block"] == "cam0" and row["parameter"] == "fx")
        self.assertAlmostEqual(float(fx["representative_delta"]), 2.0)
        self.assertTrue(any(row["parameter"] == "relative_rotation_angle" for row in comparison))
        self.assertIn("老化前后标定参数对比", Path(result["html"]).read_text(encoding="utf-8"))

    def test_baseline_direction_is_compared_in_cam0_coordinates(self):
        parameters = []
        rotations = {
            "before": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            "after": ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        }
        translations = {"before": (-1.0, 0.0, 0.0), "after": (0.0, -1.0, 0.0)}
        for phase in ("before", "after"):
            for repeat in (1, 2, 3):
                for camera in ("cam0", "cam1"):
                    for name in ("fx", "fy", "cx", "cy", "d1", "d2", "d3", "d4"):
                        parameters.append({"phase": phase, "repeat": repeat, "camera": camera, "parameter": name, "value": 1.0})
                for row in range(3):
                    for column in range(3):
                        parameters.append({"phase": phase, "repeat": repeat, "camera": "cam0_cam1", "parameter": f"r{row}{column}", "value": rotations[phase][row][column]})
                for axis, value in zip("xyz", translations[phase]):
                    parameters.append({"phase": phase, "repeat": repeat, "camera": "cam0_cam1", "parameter": "t" + axis + "_m", "value": value})
        calibrations = [
            {"phase": phase, "repeat": repeat, "baseline_mm": 1000.0}
            for phase in ("before", "after")
            for repeat in (1, 2, 3)
        ]
        rows = {row["parameter"]: row for row in build_parameter_comparison(calibrations, parameters)}
        self.assertAlmostEqual(rows["relative_rotation_angle"]["representative_delta"], 90.0)
        self.assertAlmostEqual(rows["translation_vector_change"]["representative_delta"], 0.0)
        self.assertAlmostEqual(rows["baseline_direction_angle"]["representative_delta"], 0.0)

    def test_extrinsic_diagnosis_splits_rotation_and_physical_baseline(self):
        angle = math.radians(2.0)
        after_rotation = [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(angle), -math.sin(angle)],
            [0.0, math.sin(angle), math.cos(angle)],
        ]
        before_baseline = [0.38, 0.0, 0.0]
        after_baseline = [0.381, 0.002, -0.003]

        def document(intrinsics, rotation, baseline):
            rotation_array = np.asarray(rotation, dtype=float)
            transform = np.eye(4)
            transform[:3, :3] = rotation_array
            transform[:3, 3] = -rotation_array @ np.asarray(baseline, dtype=float)
            return {
                "schema_version": "1.0.0",
                "kind": "calibration_result",
                "cameras": [
                    {"id": "cam0", "intrinsics": intrinsics, "distortion_coeffs": [0.1, 0.2, 0.3, 0.4]},
                    {"id": "cam1", "intrinsics": intrinsics, "distortion_coeffs": [0.1, 0.2, 0.3, 0.4], "T_cn_cnm1": transform.tolist()},
                ],
            }

        before = document([100.0, 101.0, 50.0, 51.0], np.eye(3), before_baseline)
        after = document([200.0, 201.0, 60.0, 61.0], after_rotation, after_baseline)
        variants, metadata = build_extrinsic_variants(before, after)

        def rotation_baseline(name):
            transform = np.asarray(variants[name]["cameras"][1]["T_cn_cnm1"], dtype=float)
            return transform[:3, :3], -transform[:3, :3].T @ transform[:3, 3]

        rotation, baseline = rotation_baseline("before_kd_rotation_all")
        np.testing.assert_allclose(rotation, after_rotation, atol=1.0e-12)
        np.testing.assert_allclose(baseline, before_baseline, atol=1.0e-12)
        rotation, baseline = rotation_baseline("before_kd_baseline_all")
        np.testing.assert_allclose(rotation, np.eye(3), atol=1.0e-12)
        np.testing.assert_allclose(baseline, after_baseline, atol=1.0e-12)
        rotation, baseline = rotation_baseline("before_kd_rotation_x")
        np.testing.assert_allclose(rotation, after_rotation, atol=1.0e-12)
        np.testing.assert_allclose(baseline, before_baseline, atol=1.0e-12)
        np.testing.assert_allclose(
            variants["before_kd_after_t"]["cameras"][1]["T_cn_cnm1"],
            after["cameras"][1]["T_cn_cnm1"],
        )
        self.assertEqual(variants["after_kd_before_t"]["cameras"][0]["intrinsics"][0], 200.0)
        self.assertAlmostEqual(metadata["delta_rotation_vector_cam0_deg"][0], 2.0)
        np.testing.assert_allclose(metadata["delta_baseline_cam0_mm"], [1.0, 2.0, -3.0], atol=1.0e-10)

    def test_extrinsic_diagnosis_report_contains_evidence_and_followup_sop(self):
        dataset_manifest = {"schema_version": "1.0.0", "cameras": [{"id": "cam0", "resolution": [3840, 2160]}, {"id": "cam1", "resolution": [3840, 2160]}]}
        for dataset in (*CALIBRATIONS, *TESTS):
            group = "calibration" if dataset in CALIBRATIONS else "test"
            _yaml(self.root / "after" / group / (dataset + "_after") / "dataset.yaml", dataset_manifest)
        refresh_manifest(self.manifest)
        self.manifest = load_manifest(self.manifest_path)
        for phase, offset in (("before", 0.0), ("after", 2.0)):
            for index, run in enumerate(self.manifest["phases"][phase]["calibration_runs"]):
                _calibration({"result": self.root / run["result"], "artifacts": self.root / run["artifacts"]}, offset + index)
        rows = []
        for variant_index, variant in enumerate(VARIANT_ORDER):
            for repeat in (1, 2, 3):
                base = 0.5 - variant_index * 0.01 + repeat * 0.001
                rows.append({
                    "variant": variant,
                    "description": variant,
                    "repeat": repeat,
                    "dataset_id": f"test_{repeat}",
                    "cam0_rms_px": base,
                    "cam1_rms_px": base + 0.01,
                    "stereo_rms_px": base + 0.005,
                    "alignment_rms_px": base + 0.2,
                    "cam0_frames": 10,
                    "cam1_frames": 10,
                    "stereo_pairs": 10,
                    "result_directory": "/tmp/result",
                })
        result = generate_diagnosis_report(self.manifest, rows, {"kind": "fixture"})
        output = Path(result["output"])
        self.assertTrue(Path(result["pdf"]).read_bytes().startswith(b"%PDF"))
        self.assertTrue((output / "PHYSICAL_FOLLOWUP_SOP_ZH.md").is_file())
        self.assertIn("外参变化", Path(result["html"]).read_text(encoding="utf-8"))
        with (output / "verification_summary.csv").open(encoding="utf-8-sig") as stream:
            self.assertEqual(len(list(csv.DictReader(stream))), len(VARIANT_ORDER))


if __name__ == "__main__":
    unittest.main()
