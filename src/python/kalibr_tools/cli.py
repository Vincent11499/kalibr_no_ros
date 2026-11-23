"""Command hierarchy for installed Kalibr diagnostic tools."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _positive_integer(value):
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def add_tools_parser(commands):
    tools = commands.add_parser(
        "tools", help="run installed diagnostics and calibration studies")
    tool_commands = tools.add_subparsers(dest="tool", required=True)

    allan = tool_commands.add_parser(
        "imu-allan", help="estimate IMU noise from static CSV and generate HTML/PDF reports")
    allan.add_argument("--dataset", required=True, help="session directory or IMU CSV (rad/s, m/s^2)")
    allan.add_argument("--output-dir", required=True, help="new result directory outside the source dataset")
    allan.add_argument("--topic", default="/imu/data", help="rostopic in imu_noise.yaml (default: /imu/data)")
    from .imu_allan.input_format import add_time_arguments

    add_time_arguments(allan)

    fixed = tool_commands.add_parser(
        "fixed-stereo-validation",
        help="validate fixed stereo K/D/T on a directory dataset")
    fixed.add_argument(
        "--calibration", action="append", required=True,
        help="repeatable LABEL=calibration-result.yaml (LABEL= may be omitted)")
    fixed.add_argument("--dataset", required=True)
    fixed.add_argument("--target", required=True)
    fixed.add_argument("--output-dir", required=True)
    fixed.add_argument("--window-half-size-px", type=int, default=2)
    fixed.add_argument(
        "--max-displacement-px", type=float, default=1.5 ** 0.5)
    fixed.add_argument(
        "--synchronization-tolerance-s", type=float, default=0.0002)
    fixed.add_argument("--rectification-balance", type=float, default=0.0)
    fixed.add_argument("--rectification-fov-scale", type=float, default=1.0)
    fixed.add_argument(
        "--detector-opencv-threads", type=_positive_integer, default=1)
    fixed.add_argument(
        "--visualizations", action="store_true",
        help="save detected corners, undistorted images, and rectified stereo pairs")
    fixed.add_argument(
        "--max-frames-per-camera", type=_positive_integer, default=20)
    fixed.add_argument("--max-pairs", type=_positive_integer, default=20)
    fixed.add_argument(
        "--undistortion-crop", action="store_true",
        help="crop invalid borders in per-camera undistorted images")
    fixed.add_argument("--force", action="store_true")

    distance = tool_commands.add_parser(
        "stereo-tag-distance",
        help="measure the 3-D center distance between two AprilTags",
    )
    distance.add_argument("--dataset", required=True)
    distance.add_argument(
        "--calibration", required=True,
        help="camera calibration result or stereo_imu_calib.yaml")
    distance.add_argument("--tag0", type=int, default=0)
    distance.add_argument("--tag1", type=int, default=1)
    distance.add_argument(
        "--border", choices=("auto", "1", "2"), default="auto")
    distance.add_argument("--max-pair-delta-ms", type=float, default=10.0)
    distance.add_argument(
        "--output-csv",
        help=("per-pair CSV; defaults to the dataset sibling "
              "result/distance_YYMMDD_HHMM/"))
    distance.add_argument("--summary-csv")
    distance.add_argument("--detection-dir")
    distance.add_argument("--overwrite", action="store_true")

    rolling = tool_commands.add_parser(
        "rolling-shutter-diagnosis",
        help="test whether dynamic data supports a rolling-shutter row-time model",
    )
    rolling.add_argument(
        "--config", required=True, help="rolling_shutter_diagnosis YAML")
    rolling.add_argument(
        "--output-dir", required=True,
        help="new or tool-owned output directory")
    rolling.add_argument(
        "--kalibr-noros",
        help="kalibr-noros executable; defaults to this installation")
    rolling.add_argument(
        "--dry-run", action="store_true",
        help="validate and print the ordinary source command only")
    rolling.add_argument(
        "--force", action="store_true",
        help="replace this tool's managed outputs and rerun")

    aging = tool_commands.add_parser(
        "stereo-aging-validation",
        help="run a three-calibration/three-test stereo aging or vibration study",
    )
    aging_commands = aging.add_subparsers(
        dest="tool_command", required=True)
    init = aging_commands.add_parser(
        "init", help="discover datasets and create a new manifest")
    init.add_argument("--machine-root", required=True)
    inspect = aging_commands.add_parser(
        "inspect", help="check datasets and completed evidence")
    inspect.add_argument("--manifest", required=True)
    refresh = aging_commands.add_parser(
        "refresh", help="register newly captured after datasets")
    refresh.add_argument("--manifest", required=True)
    for name in ("run", "calibrate"):
        command = aging_commands.add_parser(name)
        command.add_argument("--manifest", required=True)
        command.add_argument(
            "--phase", choices=("before", "after"), required=True)
        command.add_argument("--dry-run", action="store_true")
        command.add_argument("--force", action="store_true")
    verify = aging_commands.add_parser("verify")
    verify.add_argument("--manifest", required=True)
    verify.add_argument(
        "--test-phase", choices=("before", "after"), required=True)
    verify.add_argument(
        "--calibration-phase", choices=("before", "after"), required=True)
    verify.add_argument("--dry-run", action="store_true")
    verify.add_argument("--force", action="store_true")
    report = aging_commands.add_parser("report")
    report.add_argument("--manifest", required=True)
    report.add_argument("--full", action="store_true")
    report.add_argument("--force", action="store_true")
    diagnose = aging_commands.add_parser(
        "diagnose-extrinsics",
        help=("attribute before/after changes to K/D, rotation, and physical "
              "baseline components"),
    )
    diagnose.add_argument("--manifest", required=True)
    diagnose.add_argument("--dry-run", action="store_true")
    diagnose.add_argument("--force", action="store_true")


def _print_commands(commands, shell_join):
    for command in commands:
        print(shell_join(command))


def run_tool(arguments, prefix):
    prefix = Path(prefix).resolve()
    if arguments.tool == "imu-allan":
        from .imu_allan.input_format import time_bounds
        from .imu_allan.workflow import run_analysis

        start_ns, end_ns = time_bounds(arguments)
        result = run_analysis(
            arguments.dataset, arguments.output_dir, topic=arguments.topic,
            start_ns=start_ns, end_ns=end_ns,
            progress=lambda message: print(message, flush=True),
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0

    if arguments.tool == "fixed-stereo-validation":
        from .fixed_stereo_validation import verify_fixed_cameras

        output = Path(arguments.output_dir).expanduser().resolve()
        verify_fixed_cameras(
            arguments.calibration,
            arguments.dataset,
            arguments.target,
            output,
            window_half_size_px=arguments.window_half_size_px,
            max_displacement_px=arguments.max_displacement_px,
            synchronization_tolerance_s=arguments.synchronization_tolerance_s,
            rectification_balance=arguments.rectification_balance,
            rectification_fov_scale=arguments.rectification_fov_scale,
            detector_opencv_threads=arguments.detector_opencv_threads,
            visualizations=arguments.visualizations,
            max_frames_per_camera=arguments.max_frames_per_camera,
            max_pairs=arguments.max_pairs,
            undistortion_crop=arguments.undistortion_crop,
            force=arguments.force,
        )
        print(output)
        return 0

    if arguments.tool == "stereo-tag-distance":
        from .stereo_tag_distance import run_distance

        return run_distance(arguments, prefix)

    if arguments.tool == "rolling-shutter-diagnosis":
        from .rolling_shutter_diagnosis.workflow import run_diagnosis

        binary = arguments.kalibr_noros or prefix / "bin" / "kalibr-noros"
        result = run_diagnosis(
            arguments.config,
            arguments.output_dir,
            binary=binary,
            dry_run=arguments.dry_run,
            force=arguments.force,
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0

    if arguments.tool != "stereo-aging-validation":
        raise AssertionError(arguments.tool)

    from .aging_vibration.workflow import (
        create_manifest,
        inspect_study,
        load_manifest,
        refresh_manifest,
        run_calibrations,
        run_phase,
        run_verifications,
        shell_join,
    )

    if arguments.tool_command == "init":
        print(create_manifest(arguments.machine_root))
        return 0
    manifest = load_manifest(arguments.manifest, prefix)
    if arguments.tool_command == "refresh":
        print(refresh_manifest(manifest))
        return 0
    if arguments.tool_command == "inspect":
        result = inspect_study(manifest)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["status"] == "ok" else 2
    if arguments.tool_command == "calibrate":
        _print_commands(
            run_calibrations(
                manifest, arguments.phase, arguments.dry_run, arguments.force),
            shell_join,
        )
        return 0
    if arguments.tool_command == "verify":
        _print_commands(
            run_verifications(
                manifest,
                arguments.test_phase,
                arguments.calibration_phase,
                arguments.dry_run,
                arguments.force,
            ),
            shell_join,
        )
        return 0
    if arguments.tool_command == "run":
        result = run_phase(
            manifest, arguments.phase, arguments.dry_run, arguments.force)
        _print_commands(result["commands"], shell_join)
        if result["report"]:
            print(json.dumps(result["report"], ensure_ascii=False, indent=2))
        return 0
    if arguments.tool_command == "report":
        from .aging_vibration.reporting import generate_report

        result = generate_report(
            manifest, arguments.full, arguments.force)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    if arguments.tool_command == "diagnose-extrinsics":
        from .aging_vibration.diagnosis import run_extrinsic_diagnosis

        result = run_extrinsic_diagnosis(
            manifest, arguments.dry_run, arguments.force)
        _print_commands(result["commands"], shell_join)
        if result["report"]:
            print(json.dumps(result["report"], ensure_ascii=False, indent=2))
        return 0
    raise AssertionError(arguments.tool_command)
