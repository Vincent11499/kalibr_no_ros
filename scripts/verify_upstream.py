#!/usr/bin/env python3
"""Fail when the immutable upstream Kalibr snapshot has been changed."""

import hashlib
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = ROOT / "upstream" / "kalibr"
EXPECTED, EXPECTED_COUNT = (ROOT / "upstream" / "kalibr.sha256").read_text().split()

digest = hashlib.sha256()
count = 0
for path in sorted(item for item in SNAPSHOT.rglob("*") if item.is_file() or item.is_symlink()):
    relative = path.relative_to(SNAPSHOT).as_posix().encode()
    digest.update(len(relative).to_bytes(8, "big"))
    digest.update(relative)
    if path.is_symlink():
        data = ("link:" + os.readlink(str(path))).encode()
    else:
        data = path.read_bytes()
    digest.update(len(data).to_bytes(8, "big"))
    digest.update(data)
    count += 1

actual = digest.hexdigest()
if actual != EXPECTED or count != int(EXPECTED_COUNT):
    raise SystemExit(
        "upstream snapshot changed: expected {} files / {}, got {} files / {}".format(
            EXPECTED_COUNT, EXPECTED, count, actual
        )
    )
print("upstream snapshot verified: {} files, {}".format(count, actual))
