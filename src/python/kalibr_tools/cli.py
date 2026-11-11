"""Command hierarchy for installed Kalibr diagnostic tools."""

from __future__ import annotations

import json
from pathlib import Path


def add_tools_parser(commands):
    tools = commands.add_parser(
        "tools", help="run installed diagnostics and calibration studies")
    tool_commands = tools.add_subparsers(dest="tool", required=True)

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
