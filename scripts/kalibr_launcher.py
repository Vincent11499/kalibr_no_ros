#!/usr/bin/env python3
"""Relocatable launcher for an unchanged Kalibr command."""

import os
import runpy
import sys
from pathlib import Path


# A shell that sourced ROS/catkin places another Kalibr build ahead of this
# standalone tree in LD_LIBRARY_PATH.  Python would then import modules from
# this prefix while resolving libaslam_*/libbsplines from the catkin devel
# space, which is an ABI mismatch and can end in allocator corruption.  Re-exec
# before importing any extension so the build/install RPATH is authoritative.
if os.environ.get("KALIBR_NO_ROS_LAUNCHER_READY") != "1":
    environment = os.environ.copy()
    environment.pop("LD_LIBRARY_PATH", None)
    environment["KALIBR_NO_ROS_LAUNCHER_READY"] = "1"
    os.execve(
        sys.executable,
        [sys.executable, str(Path(__file__).resolve())] + sys.argv[1:],
        environment,
    )


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
