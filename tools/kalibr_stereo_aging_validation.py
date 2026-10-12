#!/usr/bin/env python3
"""Run a three-calibration/three-test stereo aging or vibration study."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aging_vibration.workflow import (
    create_manifest,
    inspect_study,
    load_manifest,
    refresh_manifest,
    run_calibrations,
    run_phase,
    run_verifications,
    shell_join,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stereo aging/vibration calibration study")
    commands = parser.add_subparsers(dest="command", required=True)
    init = commands.add_parser("init", help="discover datasets and create a new manifest")
    init.add_argument("--machine-root", required=True)
    inspect = commands.add_parser("inspect", help="check datasets and completed evidence")
    inspect.add_argument("--manifest", required=True)
    refresh = commands.add_parser("refresh", help="register newly captured after datasets")
    refresh.add_argument("--manifest", required=True)
    for name in ("run", "calibrate"):
        command = commands.add_parser(name)
        command.add_argument("--manifest", required=True)
        command.add_argument("--phase", choices=("before", "after"), required=True)
        command.add_argument("--dry-run", action="store_true")
        command.add_argument("--force", action="store_true")
    verify = commands.add_parser("verify")
    verify.add_argument("--manifest", required=True)
    verify.add_argument("--test-phase", choices=("before", "after"), required=True)
    verify.add_argument("--calibration-phase", choices=("before", "after"), required=True)
    verify.add_argument("--dry-run", action="store_true")
    verify.add_argument("--force", action="store_true")
    report = commands.add_parser("report")
    report.add_argument("--manifest", required=True)
    report.add_argument("--full", action="store_true")
    report.add_argument("--force", action="store_true")
    diagnose = commands.add_parser(
        "diagnose-extrinsics",
        help="attribute before/after changes to K/D, rotation, and physical baseline components",
    )
    diagnose.add_argument("--manifest", required=True)
    diagnose.add_argument("--dry-run", action="store_true")
    diagnose.add_argument("--force", action="store_true")
    return parser


def _print_commands(commands: list[list[str]]) -> None:
    for command in commands:
        print(shell_join(command))


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "init":
        print(create_manifest(args.machine_root))
        return 0
    manifest = load_manifest(args.manifest)
    if args.command == "refresh":
        print(refresh_manifest(manifest))
        return 0
    if args.command == "inspect":
        result = inspect_study(manifest)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["status"] == "ok" else 2
    if args.command == "calibrate":
        _print_commands(run_calibrations(manifest, args.phase, args.dry_run, args.force))
        return 0
    if args.command == "verify":
        _print_commands(run_verifications(manifest, args.test_phase, args.calibration_phase, args.dry_run, args.force))
        return 0
    if args.command == "run":
        result = run_phase(manifest, args.phase, args.dry_run, args.force)
        _print_commands(result["commands"])
        if result["report"]:
            print(json.dumps(result["report"], ensure_ascii=False, indent=2))
        return 0
    if args.command == "report":
        from aging_vibration.reporting import generate_report

        print(json.dumps(generate_report(manifest, args.full, args.force), ensure_ascii=False, indent=2))
        return 0
    if args.command == "diagnose-extrinsics":
        from aging_vibration.diagnosis import run_extrinsic_diagnosis

        result = run_extrinsic_diagnosis(manifest, args.dry_run, args.force)
        _print_commands(result["commands"])
        if result["report"]:
            print(json.dumps(result["report"], ensure_ascii=False, indent=2))
        return 0
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
