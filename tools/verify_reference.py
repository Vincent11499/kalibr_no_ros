#!/usr/bin/env python3
"""Verify the immutable ETHZ Kalibr snapshot from the source tree."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "python"))

from kalibr_no_ros.reference import verify_snapshot


def main():
    result = verify_snapshot(
        ROOT / "ref" / "kalibr", ROOT / "ref" / "kalibr.sha256", 1630
    )
    print(
        "reference snapshot verified: {} files, {}".format(
            result["actual_files"], result["actual_sha256"]
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
