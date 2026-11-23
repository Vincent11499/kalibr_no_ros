"""Shared filesystem primitives with no calibration or reporting dependencies."""

from pathlib import Path
import hashlib
import os
import tempfile


def contained_path(root, relative):
    """Resolve a relative artifact path without following external symlinks."""
    root = Path(root).resolve()
    relative = Path(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("artifact path must be relative and contained: {}".format(relative))
    path = root / relative
    if path.is_symlink():
        raise ValueError("artifact path must not be a symlink: {}".format(relative))
    try:
        path.resolve().relative_to(root)
    except ValueError as error:
        raise ValueError("artifact path escapes its root: {}".format(relative)) from error
    return path


def file_sha256(path):
    """Return the SHA-256 digest of a file without loading it into memory."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write_bytes(path, payload):
    """Atomically replace *path* with bytes written in the same directory."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix="." + path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
