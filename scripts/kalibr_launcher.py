#!/usr/bin/env python3
"""Relocatable launcher for an unchanged Kalibr command."""

import runpy
import sys
from pathlib import Path


prefix = Path(__file__).resolve().parents[1]
python_candidates = [prefix / "python", prefix / "lib" / "python3" / "dist-packages"]
for candidate in python_candidates:
    if candidate.is_dir():
        sys.path.insert(0, str(candidate))

command = Path(__file__).name
script = prefix / "libexec" / "kalibr" / command
if not script.is_file():
    raise SystemExit("missing Kalibr program: {}".format(script))
runpy.run_path(str(script), run_name="__main__")
