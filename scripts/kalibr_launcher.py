#!/usr/bin/env python3
"""Relocatable launcher for an unchanged Kalibr command."""

import os
import runpy
import sys
from pathlib import Path


prefix = Path(__file__).resolve().parents[1]
library_dir = str(prefix / "lib")
library_paths = [
    path for path in os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
    if path
]

# Python extensions are loaded later with dlopen(). Re-exec the interpreter so
# glibc sees this install tree before a sourced ROS/catkin devel space. Merely
# changing LD_LIBRARY_PATH inside the current process is not reliable because
# the dynamic loader reads it during process startup.
if not library_paths or library_paths[0] != library_dir:
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        [library_dir] + [path for path in library_paths if path != library_dir]
    )
    os.execve(
        sys.executable,
        [sys.executable, str(Path(__file__).resolve())] + sys.argv[1:],
        environment,
    )

python_candidates = [prefix / "python", prefix / "lib" / "python3" / "dist-packages"]
for candidate in python_candidates:
    if candidate.is_dir():
        sys.path.insert(0, str(candidate))

command = Path(__file__).name
script = prefix / "libexec" / "kalibr" / command
if not script.is_file():
    raise SystemExit("missing Kalibr program: {}".format(script))
runpy.run_path(str(script), run_name="__main__")
