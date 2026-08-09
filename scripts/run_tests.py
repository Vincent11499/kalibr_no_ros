#!/usr/bin/env python3
"""Run the repository tests without third-party test frameworks."""

import sys
import unittest
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "bag_io" / "src"), str(ROOT / "core" / "src")]

subprocess.run([sys.executable, str(ROOT / "scripts" / "verify_upstream.py")], check=True)

suite = unittest.defaultTestLoader.discover(str(ROOT / "tests"))
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(0 if result.wasSuccessful() else 1)
