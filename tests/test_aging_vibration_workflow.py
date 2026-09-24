import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

from aging_vibration.reporting import collect_evidence, generate_report
from aging_vibration.workflow import (
    VERIFY_OPTIONS,
    apply_migration,
    build_migration_plan,
    build_verify_command,
    create_manifest,
    inspect_study,
    load_manifest,
    prepare_runtime_task,
)


BEFORE_CALIBRATION = "stereo_260917_2206"
AFTER_CALIBRATION = "stereo_260920_1729"
BEFORE_TESTS = ["stereo_test_260917_2209", "stereo_test_260918_1009", "stereo_test_260918_1010"]
AFTER_TESTS = ["stereo_test_260920_1640", "stereo_test_260920_1641", "stereo_test_260920_1642"]


def _write_yaml(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def _calibration(path, offset):
    document = {
        "schema_version": "1.0.0",
        "kind": "calibration_result",
        "calibration_type": "cameras",
        "cameras": [
            {
                "id": "cam0",
                "intrinsics": [2380.0 + offset, 2381.0 + offset, 1920.0, 1080.0],
                "distortion_coeffs": [-0.09, -0.02 - offset * 0.001, 0.03, -0.01],
                "rms": 0.25 + offset * 0.001,
            },
            {
                "id": "cam1",
                "intrinsics": [2375.0 + offset, 2376.0 + offset, 1910.0, 1090.0],
                "distortion_coeffs": [-0.10, -0.02 - offset * 0.001, 0.03, -0.01],
                "rms": 0.24 + offset * 0.001,
                "alignment": 0.19 + offset * 0.001,
                "T_cn_cnm1": [
                    [1.0, 0.0, 0.0, -0.378 - offset * 0.0001],
                    [0.0, 1.0, 0.0, 0.003],
                    [0.0, 0.0, 1.0, 0.004],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
        ],
    }
    _write_yaml(path, document)


def _verification(directory, base):
    directory.mkdir(parents=True, exist_ok=True)
    cam0 = base
    cam1 = base + 0.01
    stereo = math.sqrt((cam0 * cam0 + cam1 * cam1) / 2.0)
    result = {
        "source_calibration": "fixture.yaml",
        "cameras": {
            "cam0": {
                "input_frames": 1,
                "detected_frames": 1,
                "corner_count": 4,
                "rms_px": cam0,
                "frames": [{"source_index": 0, "detected": True, "corner_count": 4, "rms_px": cam0}],
            },
            "cam1": {
                "input_frames": 1,
                "detected_frames": 1,
                "corner_count": 4,
                "rms_px": cam1,
                "frames": [{"source_index": 0, "detected": True, "corner_count": 4, "rms_px": cam1}],
            },
        },
        "stereo_combined_rms_px": stereo,
        "stereo": {
            "used_pairs": 1,
            "alignment": {"count": 4, "rms_px": base + 0.2},
            "pairs": [{"alignment": {"count": 4, "rms_px": base + 0.2}}],
        },
    }
    _write_json(
        directory / "fixed_camera_validation.json",
        {
            "method": {"window_half_size_px": 7, "max_displacement_px": 2},
            "results": {"1316_equi": result},
        },
    )
    fields = [
        "calibration_label", "record_type", "camera_id", "source_index", "corner_count",
        "reprojection_rms_px", "reprojection_p95_px", "reprojection_max_px", "alignment_rms_px",
        "alignment_p95_px", "alignment_max_px",
    ]
    with (directory / "fixed_camera_validation.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow({"calibration_label": "1316_equi", "record_type": "camera_frame", "camera_id": "cam0", "source_index": 0, "corner_count": 4, "reprojection_rms_px": cam0, "reprojection_p95_px": cam0, "reprojection_max_px": cam0})
        writer.writerow({"calibration_label": "1316_equi", "record_type": "camera_frame", "camera_id": "cam1", "source_index": 0, "corner_count": 4, "reprojection_rms_px": cam1, "reprojection_p95_px": cam1, "reprojection_max_px": cam1})
        writer.writerow({"calibration_label": "1316_equi", "record_type": "stereo_pair", "camera_id": "cam0", "source_index": 0, "corner_count": 4, "reprojection_rms_px": stereo, "reprojection_p95_px": stereo, "reprojection_max_px": stereo, "alignment_rms_px": base + 0.2, "alignment_p95_px": base + 0.2, "alignment_max_px": base + 0.2})
    _write_json(directory / "run_manifest.json", {"status": "completed"})


class AgingVibrationWorkflowTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / "mechine-fixture"
        self.root.mkdir()
        for dataset in [BEFORE_CALIBRATION, AFTER_CALIBRATION, *BEFORE_TESTS, *AFTER_TESTS]:
            _write_yaml(self.root / dataset / "dataset.yaml", {"schema_version": "1.0.0"})
        for dataset, offset in ((BEFORE_CALIBRATION, 0.0), (AFTER_CALIBRATION, 1.0)):
            base = self.root / "result" / (dataset + "_equi")
            _calibration(base.with_suffix(".yaml"), offset)
            (base.with_suffix(".report.html")).write_text("report", encoding="utf-8")
            (base.with_suffix(".report.pdf")).write_bytes(b"%PDF-fixture")
            _write_json(base / "run_manifest.json", {"status": "completed"})
            _write_yaml(base / "observability.yaml", {"status": "full_rank", "quality": "weakly_observable"})
        for index, dataset in enumerate(BEFORE_TESTS):
            _verification(self.root / "result" / ("verification_" + dataset), 0.10 + index * 0.01)
        for index, dataset in enumerate(AFTER_TESTS):
            _verification(self.root / "result" / ("verification_" + dataset), 0.20 + index * 0.01)
            _verification(self.root / "result" / ("verification_" + dataset + "_after"), 0.12 + index * 0.01)
        self.manifest_path = create_manifest(
            self.root,
            "fixture",
            BEFORE_CALIBRATION,
            BEFORE_TESTS,
            AFTER_CALIBRATION,
            AFTER_TESTS,
        )

    def tearDown(self):
        self.temporary.cleanup()

    def test_inspect_and_reconcile_existing_evidence(self):
        manifest = load_manifest(self.manifest_path)
        self.assertEqual(inspect_study(manifest)["status"], "ok")
        summary, details = collect_evidence(manifest)
        self.assertEqual(len(summary), 9)
        self.assertEqual(len(details), 27)

    def test_migration_is_explicit_and_preserves_evidence(self):
        manifest = load_manifest(self.manifest_path)
        plan = build_migration_plan(manifest)
        self.assertEqual(len(plan), 25)
        self.assertTrue((self.root / BEFORE_CALIBRATION).exists())
        result = apply_migration(manifest, plan)
        self.assertEqual(result["status"], "completed")
        migrated = load_manifest(self.manifest_path)
        self.assertEqual(inspect_study(migrated)["status"], "ok")
        self.assertFalse((self.root / BEFORE_CALIBRATION).exists())
        self.assertTrue((self.root / "migration_manifest.json").is_file())
        self.assertEqual(build_migration_plan(migrated), [])

    def test_verify_command_uses_release_reference_parameters(self):
        manifest = load_manifest(self.manifest_path)
        manifest["kalibr"]["binary"] = sys.executable
        command = build_verify_command(
            manifest,
            Path("/tmp/calibration.yaml"),
            Path("/tmp/dataset"),
            Path("/tmp/output"),
        )
        self.assertEqual(command[1:3], ["verify", "cameras"])
        self.assertIn("1316_equi=/tmp/calibration.yaml", command)
        for option, expected in (
            ("--window-half-size-px", "7"),
            ("--max-displacement-px", "2.0"),
            ("--synchronization-tolerance-s", "0.0001"),
            ("--detector-opencv-threads", "1"),
            ("--max-frames-per-camera", "30"),
            ("--max-pairs", "30"),
        ):
            self.assertEqual(command[command.index(option) + 1], expected)
        self.assertIn("--visualizations", command)
        self.assertEqual(manifest["kalibr"]["verify_options"], VERIFY_OPTIONS)

    def test_calibration_dry_run_task_preparation_does_not_write(self):
        manifest = load_manifest(self.manifest_path)
        runtime_task, _, output_name = prepare_runtime_task(manifest, "before", write=False)
        self.assertEqual(output_name, BEFORE_CALIBRATION + "_equi")
        self.assertFalse(runtime_task.exists())

    def test_report_contains_html_pdf_and_csv(self):
        manifest = load_manifest(self.manifest_path)
        report = generate_report(manifest)
        self.assertTrue(Path(report["html"]).is_file())
        self.assertTrue(Path(report["pdf"]).read_bytes().startswith(b"%PDF"))
        self.assertEqual(report["summary_rows"], 9)
        self.assertEqual(report["detail_rows"], 27)
        document = Path(report["html"]).read_text(encoding="utf-8")
        self.assertIn("不输出通过/不通过判定", document)
        self.assertIn("data:image/png;base64,", document)


if __name__ == "__main__":
    unittest.main()
