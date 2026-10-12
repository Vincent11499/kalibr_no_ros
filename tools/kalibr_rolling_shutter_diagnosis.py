#!/usr/bin/env python3
"""Diagnose whether a calibration dataset supports a rolling-shutter row-time model."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rolling_shutter_diagnosis.workflow import run_diagnosis


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="rolling_shutter_diagnosis YAML")
    parser.add_argument("--output-dir", required=True, help="new or tool-owned output directory")
    parser.add_argument("--kalibr-noros", help="installed kalibr-noros executable; defaults to install/release/bin")
    parser.add_argument("--dry-run", action="store_true", help="validate and print the ordinary source command only")
    parser.add_argument("--force", action="store_true", help="replace this tool's managed outputs and rerun")
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    try:
        result = run_diagnosis(
            arguments.config,
            arguments.output_dir,
            binary=arguments.kalibr_noros,
            dry_run=arguments.dry_run,
            force=arguments.force,
        )
    except (OSError, ValueError, RuntimeError) as error:
        print(f"rolling-shutter-diagnosis: error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
