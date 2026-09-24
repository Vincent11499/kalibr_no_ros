#!/usr/bin/env python3
"""CLI for repeatable stereo aging/vibration calibration studies."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from aging_vibration.workflow import (
        apply_migration,
        build_migration_plan,
        create_manifest,
        inspect_study,
        load_manifest,
        run_calibration,
        run_verification,
        shell_join,
    )
else:
    from .workflow import (
        apply_migration,
        build_migration_plan,
        create_manifest,
        inspect_study,
        load_manifest,
        run_calibration,
        run_verification,
        shell_join,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stereo aging/vibration calibration study workflow")
    subparsers = parser.add_subparsers(dest="command", required=True)

    init = subparsers.add_parser("init", help="create a study manifest from explicit phase assignments")
    init.add_argument("--machine-root", required=True)
    init.add_argument("--machine-id", required=True)
    init.add_argument("--before-calibration", required=True)
    init.add_argument("--before-test", action="append", required=True)
    init.add_argument("--after-calibration", required=True)
    init.add_argument("--after-test", action="append", required=True)

    inspect = subparsers.add_parser("inspect", help="validate data and completed evidence without mutation")
    inspect.add_argument("--manifest", required=True)

    organize = subparsers.add_parser("organize", help="plan or apply the phase-oriented directory migration")
    organize.add_argument("--manifest", required=True)
    organize.add_argument("--apply", action="store_true", help="apply the printed migration plan")

    calibrate = subparsers.add_parser("calibrate", help="run release camera calibration for one phase")
    calibrate.add_argument("--manifest", required=True)
    calibrate.add_argument("--phase", choices=("before", "after"), required=True)
    calibrate.add_argument("--dry-run", action="store_true")
    calibrate.add_argument("--force", action="store_true")

    verify = subparsers.add_parser("verify", help="run release fixed-camera validation")
    verify.add_argument("--manifest", required=True)
    verify.add_argument("--test-phase", choices=("before", "after"), required=True)
    verify.add_argument("--calibration-phase", choices=("before", "after"), required=True)
    verify.add_argument("--dry-run", action="store_true")
    verify.add_argument("--force", action="store_true")

    report = subparsers.add_parser("report", help="generate HTML/PDF/CSV from existing evidence only")
    report.add_argument("--manifest", required=True)
    report.add_argument("--output")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "init":
        path = create_manifest(
            args.machine_root,
            args.machine_id,
            args.before_calibration,
            args.before_test,
            args.after_calibration,
            args.after_test,
        )
        print(path)
        return 0

    manifest = load_manifest(args.manifest)
    if args.command == "inspect":
        result = inspect_study(manifest)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["status"] == "ok" else 2
    if args.command == "organize":
        plan = build_migration_plan(manifest)
        print(json.dumps({"moves": plan}, ensure_ascii=False, indent=2))
        if args.apply:
            result = apply_migration(manifest, plan)
            print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    if args.command == "calibrate":
        command = run_calibration(manifest, args.phase, args.dry_run, args.force)
        print(shell_join(command))
        return 0
    if args.command == "verify":
        commands = run_verification(
            manifest,
            args.test_phase,
            args.calibration_phase,
            args.dry_run,
            args.force,
        )
        for command in commands:
            print(shell_join(command))
        return 0
    if args.command == "report":
        if __package__ in (None, ""):
            from aging_vibration.reporting import generate_report
        else:
            from .reporting import generate_report

        output = Path(args.output).resolve() if args.output else None
        result = generate_report(manifest, output)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
