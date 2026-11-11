#!/usr/bin/env python3
"""Compatibility entry point for the installed stereo aging tool."""

from __future__ import annotations

import os
from pathlib import Path
import sys


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    candidates = (
        project_root / "build" / "release" / "bin" / "kalibr-noros",
        project_root / "install" / "release" / "bin" / "kalibr-noros",
    )
    binary = next(
        (path for path in candidates
         if path.is_file() and os.access(path, os.X_OK)),
        None,
    )
    replacement = "kalibr-noros tools stereo-aging-validation"
    print(
        "deprecated: use '{}' instead".format(replacement),
        file=sys.stderr,
        flush=True,
    )
    if binary is None:
        print(
            "compatibility entry point requires a configured release build",
            file=sys.stderr,
        )
        return 2
    os.execv(
        str(binary),
        [str(binary), "tools", "stereo-aging-validation", *sys.argv[1:]],
    )
    raise AssertionError("unreachable")


if __name__ == "__main__":
    raise SystemExit(main())
