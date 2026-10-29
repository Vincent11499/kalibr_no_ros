#!/usr/bin/env python3
"""Publish the minimal relocatable Kalibr runtime from a configured build."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import uuid


RUNTIME_DIRECTORIES = ("bin", "lib", "libexec")


def _copy_tree(source, destination):
    if source.is_symlink() or not source.is_dir():
        raise RuntimeError("required runtime directory is missing: {}".format(source))
    shutil.copytree(source, destination, symlinks=True)


def _copy_file(source, destination):
    if source.is_symlink() or not source.is_file():
        raise RuntimeError("required runtime file is missing: {}".format(source))
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _preserve_local_machine_yaml(existing, destination):
    root = existing / "config" / "mechine"
    if not root.exists():
        return
    if root.is_symlink() or not root.is_dir():
        raise RuntimeError(
            "existing machine configuration must be a directory: {}".format(root))
    for current, directories, filenames in os.walk(root, followlinks=False):
        current_path = Path(current)
        for name in directories:
            if (current_path / name).is_symlink():
                raise RuntimeError(
                    "existing machine configuration contains a symlink: {}"
                    .format(current_path / name))
        for name in filenames:
            source = current_path / name
            if source.suffix.lower() != ".yaml":
                continue
            if source.is_symlink() or not source.is_file():
                raise RuntimeError(
                    "existing machine configuration is not a regular file: {}"
                    .format(source))
            relative = source.relative_to(root)
            if (relative.parts
                    and relative.parts[0] == "workflow_v1.0.0"):
                continue
            target = destination / "config" / "mechine" / relative
            if not target.exists():
                _copy_file(source, target)


def assemble_runtime(source, destination, *, existing=None, user_config=None):
    """Copy only runtime files into an empty destination directory."""
    source = Path(source).resolve()
    destination = Path(destination)
    if any(destination.iterdir()):
        raise RuntimeError("runtime staging directory must be empty: {}".format(
            destination))
    for name in RUNTIME_DIRECTORIES:
        _copy_tree(source / name, destination / name)
    _copy_tree(
        source / "config" / "mechine",
        destination / "config" / "mechine")
    _copy_file(source / "LICENSE", destination / "LICENSE")
    license_directory = source / "licenses"
    if license_directory.is_dir() and not license_directory.is_symlink():
        _copy_tree(license_directory, destination / "licenses")
    else:
        (destination / "licenses").mkdir()

    if existing is not None:
        existing = Path(existing)
        _preserve_local_machine_yaml(existing, destination)
    selected_user = Path(user_config) if user_config is not None else None
    if selected_user is None and existing is not None:
        candidate = existing / "config" / "user.yaml"
        if candidate.exists() or candidate.is_symlink():
            selected_user = candidate
    if selected_user is not None:
        _copy_file(selected_user, destination / "config" / "user.yaml")


def _safe_prefix(value):
    prefix = Path(value).expanduser()
    if prefix.is_symlink():
        raise RuntimeError("runtime prefix must not be a symlink: {}".format(prefix))
    prefix = prefix.resolve()
    forbidden = {Path("/").resolve(), Path.home().resolve()}
    if prefix in forbidden or prefix.parent == prefix:
        raise RuntimeError("refusing unsafe runtime prefix: {}".format(prefix))
    return prefix


def publish_runtime(build_dir, prefix, *, replace=False, user_config=None):
    build_dir = Path(build_dir).resolve()
    if not (build_dir / "cmake_install.cmake").is_file():
        raise RuntimeError("build directory is not configured: {}".format(build_dir))
    prefix = _safe_prefix(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    if prefix.exists() and not replace:
        raise RuntimeError(
            "runtime prefix already exists; pass --replace to replace it safely: {}"
            .format(prefix))

    with tempfile.TemporaryDirectory(prefix="kalibr-full-install-") as temporary:
        full = Path(temporary) / "full"
        subprocess.run(
            ["cmake", "--install", str(build_dir), "--prefix", str(full)],
            check=True, stdout=subprocess.DEVNULL)
        staging = Path(tempfile.mkdtemp(
            prefix=".{}-staging-".format(prefix.name), dir=str(prefix.parent)))
        try:
            assemble_runtime(
                full, staging,
                existing=prefix if prefix.exists() else None,
                user_config=user_config)
            backup = None
            if prefix.exists():
                backup = prefix.parent / ".{}-backup-{}".format(
                    prefix.name, uuid.uuid4().hex)
                os.replace(prefix, backup)
            try:
                os.replace(staging, prefix)
            except Exception:
                if backup is not None and not prefix.exists():
                    os.replace(backup, prefix)
                raise
            if backup is not None:
                shutil.rmtree(backup)
        except Exception:
            if staging.exists():
                shutil.rmtree(staging)
            raise
    return prefix


def build_parser():
    parser = argparse.ArgumentParser(
        description="install a minimal relocatable Kalibr runtime")
    parser.add_argument("--build-dir", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument(
        "--replace", action="store_true",
        help="atomically replace an existing runtime prefix")
    parser.add_argument(
        "--user-config",
        help="copy this workflow configuration to config/user.yaml")
    return parser


def main(argv=None):
    arguments = build_parser().parse_args(argv)
    try:
        prefix = publish_runtime(
            arguments.build_dir, arguments.prefix,
            replace=arguments.replace, user_config=arguments.user_config)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print("install_runtime: error: {}".format(error))
        return 2
    print("runtime installed: {}".format(prefix))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
