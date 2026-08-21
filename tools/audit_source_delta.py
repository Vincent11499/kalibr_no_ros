#!/usr/bin/env python3
"""Audit the editable source against the frozen ETHZ snapshot."""

from pathlib import Path
import argparse
import hashlib
import json
import sys


ROOT = Path(__file__).resolve().parents[1]
MAPPINGS = (
    ("foundation", "Schweizer-Messer"),
    ("camera", "aslam_cv"),
    ("optimization", "aslam_optimizer"),
    ("trajectory", "aslam_nonparametric_estimation"),
    ("calibration/incremental_calibration", "aslam_incremental_calibration/incremental_calibration"),
    ("calibration/incremental_calibration_python", "aslam_incremental_calibration/incremental_calibration_python"),
    ("calibration/kalibr", "aslam_offline_calibration/kalibr"),
    ("third_party/ethz_apriltag2", "aslam_offline_calibration/ethz_apriltag2"),
)
ALLOWED_MODIFICATIONS = {
    "foundation/sm_python/python/sm/PlotCollection.py": "headless plotting",
    "camera/aslam_cv_backend_python/python/aslam_cv_backend/__init__.py": "remove rosbuild import",
    "optimization/aslam_backend/include/aslam/backend/Optimizer2.hpp": "compile-time profiling",
    "optimization/aslam_backend/src/BlockCholeskyLinearSystemSolver.cpp": "deterministic parallel assembly",
    "optimization/aslam_backend/src/JacobianContainer.cpp": "allocation reduction",
    "trajectory/aslam_splines/include/aslam/backend/implementation/BSplineMotionError.hpp": "zero-copy coefficient view",
    "calibration/incremental_calibration_python/src/incremental_calibration/__init__.py": "remove rosbuild import",
    "calibration/kalibr/python/kalibr_calibrate_cameras": "ROS-free models/runtime integration",
    "calibration/kalibr/python/kalibr_calibrate_imu_camera": "ROS-free models/runtime integration",
    "calibration/kalibr/python/kalibr_camera_calibration/CameraCalibrator.py": "runtime integration",
    "calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py": "runtime integration",
    "calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py": "igraph compatibility",
    "calibration/kalibr/python/kalibr_common/ImageDatasetReader.py": "ROS-free bag adapter",
    "calibration/kalibr/python/kalibr_common/ImuDatasetReader.py": "ROS-free bag adapter",
    "calibration/kalibr/python/kalibr_common/TargetExtractor.py": "bounded parallel detector",
    "calibration/kalibr/python/kalibr_imu_camera_calibration/IccCalibrator.py": "runtime integration",
    "calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py": "runtime integration",
}


def _digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(root=ROOT):
    root = Path(root)
    project = root / "src" / "kalibr"
    reference = root / "ref" / "kalibr"
    relocated = []
    modified = []
    added = []
    failures = []
    represented_reference = set()
    for project_prefix, reference_prefix in MAPPINGS:
        project_root = project / project_prefix
        reference_root = reference / reference_prefix
        for path in sorted(
            item for item in project_root.rglob("*")
            if item.is_file()
            and "__pycache__" not in item.parts
            and item.suffix != ".pyc"
        ):
            project_relative = path.relative_to(project).as_posix()
            relative = path.relative_to(project_root)
            reference_path = reference_root / relative
            if not reference_path.is_file():
                added.append(project_relative)
                failures.append("unlisted project source: {}".format(project_relative))
                continue
            represented_reference.add(reference_path.relative_to(reference).as_posix())
            if _digest(path) == _digest(reference_path):
                relocated.append(project_relative)
            elif project_relative in ALLOWED_MODIFICATIONS:
                modified.append({
                    "path": project_relative,
                    "reason": ALLOWED_MODIFICATIONS[project_relative],
                })
            else:
                failures.append("unapproved source modification: {}".format(project_relative))
    reference_files = {
        path.relative_to(reference).as_posix()
        for path in reference.rglob("*")
        if path.is_file()
    }
    omitted = sorted(reference_files - represented_reference)
    missing_allowed = sorted(set(ALLOWED_MODIFICATIONS) - {item["path"] for item in modified})
    if missing_allowed:
        failures.append(
            "allowlisted modifications no longer differ: {}".format(
                ", ".join(missing_allowed)
            )
        )
    return {
        "schema_version": 1,
        "relocated_unchanged": len(relocated),
        "approved_modifications": modified,
        "project_only_files": added,
        "omitted_reference_files": len(omitted),
        "valid": not failures,
        "failures": failures,
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--json")
    arguments = parser.parse_args(argv)
    result = audit(arguments.root)
    if arguments.json:
        Path(arguments.json).write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(
        "source audit: {} unchanged, {} approved modifications, {} omitted reference files".format(
            result["relocated_unchanged"],
            len(result["approved_modifications"]),
            result["omitted_reference_files"],
        )
    )
    for failure in result["failures"]:
        print(failure, file=sys.stderr)
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
