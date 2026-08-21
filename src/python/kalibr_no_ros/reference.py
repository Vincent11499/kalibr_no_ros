"""Integrity checks for the immutable ETHZ reference snapshot."""

from pathlib import Path
import hashlib
import json
import os


def snapshot_digest(snapshot):
    snapshot = Path(snapshot)
    digest = hashlib.sha256()
    count = 0
    for path in sorted(
        item for item in snapshot.rglob("*") if item.is_file() or item.is_symlink()
    ):
        relative = path.relative_to(snapshot).as_posix().encode()
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        if path.is_symlink():
            data = ("link:" + os.readlink(str(path))).encode()
        else:
            data = path.read_bytes()
        digest.update(len(data).to_bytes(8, "big"))
        digest.update(data)
        count += 1
    return count, digest.hexdigest()


def verify_snapshot(snapshot, manifest, expected_files=None, json_path=None):
    snapshot = Path(snapshot).resolve()
    manifest = Path(manifest).resolve()
    expected_digest, manifest_count = manifest.read_text(encoding="utf-8").split()
    expected_count = int(manifest_count)
    if expected_files is not None and int(expected_files) != expected_count:
        raise ValueError(
            "--expected-files {} disagrees with manifest {}".format(
                expected_files, expected_count
            )
        )
    actual_count, actual_digest = snapshot_digest(snapshot)
    result = {
        "schema_version": 1,
        "snapshot": str(snapshot),
        "manifest": str(manifest),
        "expected_files": expected_count,
        "actual_files": actual_count,
        "expected_sha256": expected_digest,
        "actual_sha256": actual_digest,
        "valid": actual_count == expected_count and actual_digest == expected_digest,
    }
    if json_path:
        Path(json_path).write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    if not result["valid"]:
        raise ValueError(
            "reference snapshot changed: expected {} files / {}, got {} files / {}".format(
                expected_count, expected_digest, actual_count, actual_digest
            )
        )
    return result
