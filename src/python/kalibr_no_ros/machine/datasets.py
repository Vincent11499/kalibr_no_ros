"""Machine dataset discovery, validation, and content fingerprints."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import csv
import hashlib
import json
import os
import re
import stat as stat_mode
import time

from kalibr_bag_io.directory import DirectoryReader

from ..filesystem import file_sha256 as _sha256
from ..task.document import TaskError
from ..version import (
    STATE_SCHEMA_VERSION,
    SUPPORTED_STATE_SCHEMA_VERSIONS,
    supports_schema_version,
)
from .config import _DATASET_PATTERNS
from .state import _atomic_json


INPUT_HASH_CACHE = "input_hash_cache.json"


INPUT_HASH_CACHE_KIND = "machine_input_hash_cache"


INVENTORY_FINGERPRINT_TYPE = "structured_content_image_inventory"


class DatasetChangedError(TaskError):
    """A staged solve no longer matches the inputs checked before it ran."""


def _discover_dataset_path(machine_dir, role, selected=None, children=None):
    """Return a named direct child without opening or hashing its contents."""
    if role not in _DATASET_PATTERNS:
        raise TaskError("unsupported machine dataset role: {}".format(role))
    machine = Path(machine_dir).resolve()
    pattern = _DATASET_PATTERNS[role]
    if selected is not None:
        if (not isinstance(selected, str) or Path(selected).name != selected
                or not pattern.fullmatch(selected)):
            raise TaskError(
                "selected {} dataset is not a valid directory name".format(
                    role))
        candidate = machine / selected
        if candidate.is_symlink():
            raise TaskError("dataset must not be a symlink: {}".format(candidate))
        if not candidate.is_dir():
            raise TaskError("selected dataset does not exist: {}".format(candidate))
        return candidate.resolve()

    candidates = []
    for candidate in (machine.iterdir() if children is None else children):
        if not pattern.fullmatch(candidate.name):
            continue
        if candidate.is_symlink():
            raise TaskError("dataset must not be a symlink: {}".format(candidate))
        if candidate.is_dir():
            candidates.append(candidate.resolve())
    candidates.sort(key=lambda item: item.name)
    if not candidates:
        return None
    if len(candidates) > 1:
        raise TaskError(
            "multiple {} datasets found; set datasets.{} in user.yaml: {}".format(
                role, role, ", ".join(item.name for item in candidates)))
    return candidates[0]


def discover_dataset(machine_dir, role, selected=None):
    """Return the unique validated direct-child dataset or ``None``."""
    candidate = _discover_dataset_path(machine_dir, role, selected)
    if candidate is not None:
        _validate_dataset_role(candidate, role)
    return candidate


def _safe_child(root, relative, label, *, directory=False):
    if not isinstance(relative, str) or not relative.strip():
        raise TaskError("{} must be a non-empty relative path".format(label))
    value = Path(relative)
    if value.is_absolute() or ".." in value.parts:
        raise TaskError("{} must stay inside the dataset".format(label))
    path = root
    for part in value.parts:
        path = path / part
        if path.is_symlink():
            raise TaskError("input path must not contain a symlink: {}".format(path))
    resolved = path.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise TaskError("{} escapes the dataset".format(label)) from error
    valid = resolved.is_dir() if directory else resolved.is_file()
    if not valid:
        expected = "directory" if directory else "file"
        raise TaskError("{} is not a {}: {}".format(label, expected, resolved))
    return resolved


def _validate_dataset_role(dataset, role):
    """Validate the directory manifest and its fixed workflow sensor role."""
    dataset = Path(dataset).resolve()
    _safe_child(dataset, "dataset.yaml", "dataset manifest")
    try:
        reader = DirectoryReader(dataset)
    except (OSError, ValueError, RuntimeError) as error:
        raise TaskError("invalid directory dataset {}: {}".format(
            dataset, error)) from error
    manifest = reader.manifest
    camera_ids = [camera.get("id") for camera in manifest.get("cameras", [])]
    imu_ids = [imu.get("id") for imu in manifest.get("imus", [])]
    expected_cameras = ["cam0"] if role in {"mono", "mono_baselink"} else ["cam0", "cam1"]
    if camera_ids != expected_cameras:
        raise TaskError(
            "{} dataset must contain cameras {} in order".format(
                role, expected_cameras))
    expected_imus = ["imu0"] if role == "stereo_imu" else []
    if imu_ids != expected_imus:
        raise TaskError(
            "{} dataset must contain IMUs {}".format(role, expected_imus))
    return reader


def _metadata_from_stat(value):
    return {
        "size": value.st_size,
        "mtime_ns": value.st_mtime_ns,
        "ctime_ns": value.st_ctime_ns,
        "inode": value.st_ino,
        "device": value.st_dev,
    }


def _file_metadata(path):
    return _metadata_from_stat(Path(path).stat())


def _scan_dataset_files(dataset):
    """Collect file entries with one non-following stat per tree entry."""
    files = {}
    pending = [Path(dataset)]
    while pending:
        directory = pending.pop()
        try:
            entries = os.scandir(directory)
        except OSError as error:
            raise TaskError(
                "could not scan input directory {}: {}".format(
                    directory, error)) from error
        with entries:
            for entry in entries:
                path = Path(entry.path)
                try:
                    info = entry.stat(follow_symlinks=False)
                except OSError as error:
                    raise TaskError(
                        "could not inspect input path {}: {}".format(
                            path, error)) from error
                relative = path.relative_to(dataset).as_posix()
                if stat_mode.S_ISLNK(info.st_mode):
                    files[relative] = {"path": path, "symlink": True}
                elif stat_mode.S_ISDIR(info.st_mode):
                    pending.append(path)
                elif stat_mode.S_ISREG(info.st_mode):
                    files[relative] = {
                        "path": path,
                        "metadata": _metadata_from_stat(info),
                    }
    return files


def _valid_cached_metadata(value):
    return (isinstance(value, dict)
            and all(isinstance(value.get(key), int)
                    and not isinstance(value.get(key), bool) for key in (
                        "size", "mtime_ns", "ctime_ns", "inode", "device")))


def _cached_file_hash(value):
    if (not _valid_cached_metadata(value)
            or not isinstance(value.get("sha256"), str)
            or re.fullmatch(r"[0-9a-f]{64}", value["sha256"]) is None):
        return None
    return value["sha256"]


def _read_csv_rows(path, expected, optional=()):
    try:
        stream = Path(path).open("r", encoding="utf-8-sig", newline="")
    except OSError as error:
        raise TaskError("could not read {}: {}".format(path, error)) from error
    with stream:
        reader = csv.DictReader(stream)
        allowed = (list(expected), list(expected) + list(optional))
        if reader.fieldnames not in allowed:
            raise TaskError(
                "invalid CSV header in {}: expected {}".format(
                    path, ",".join(expected)))
        try:
            return list(reader)
        except csv.Error as error:
            raise TaskError("invalid CSV {}: {}".format(path, error)) from error


def _count_csv_rows(path, expected, optional=()):
    """Validate a large CSV and count rows without retaining them in memory."""
    try:
        stream = Path(path).open("r", encoding="utf-8-sig", newline="")
    except OSError as error:
        raise TaskError("could not read {}: {}".format(path, error)) from error
    with stream:
        reader = csv.DictReader(stream)
        allowed = (list(expected), list(expected) + list(optional))
        if reader.fieldnames not in allowed:
            raise TaskError(
                "invalid CSV header in {}: expected {}".format(
                    path, ",".join(expected)))
        count = 0
        try:
            for row in reader:
                if (None in row
                        or any(value is None for value in row.values())
                        or all(not value.strip() for value in row.values())):
                    raise TaskError(
                        "invalid CSV row in {} at line {}".format(
                            path, count + 2))
                count += 1
        except csv.Error as error:
            raise TaskError("invalid CSV {}: {}".format(path, error)) from error
    return count


def _capture_session(dataset):
    path = dataset / "meta" / "session.json"
    if not path.exists():
        return None
    path = _safe_child(dataset, "meta/session.json", "capture session")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise TaskError("invalid capture session {}: {}".format(
            path, error)) from error
    if not isinstance(value, dict):
        raise TaskError("capture session root must be a mapping: {}".format(path))
    if value.get("capture_status") == "failed":
        raise TaskError("capture session marks the dataset as failed")
    if value.get("validation_status") == "fail":
        raise TaskError("capture validation marks the dataset as failed")
    return value


def _validate_capture_contract(
        dataset, session, camera_rows, camera_files, scanned_files,
        imu_count):
    if session is None:
        return
    if dataset.name.endswith(".inprogress") or ".failed" in dataset.name:
        raise TaskError("unfinished capture dataset cannot be calibrated: {}".format(
            dataset))

    camera_ids = list(camera_rows)
    if len(camera_ids) == 2:
        counts = [len(camera_rows[camera]) for camera in camera_ids]
        if len(set(counts)) != 1:
            raise TaskError(
                "capture stereo image counts disagree: {}".format(
                    ", ".join("{}={}".format(camera, len(camera_rows[camera]))
                              for camera in camera_ids)))
        pairs_path = dataset / "meta" / "pairs.csv"
        if pairs_path.is_file():
            fields = (
                "pair_index", "left_timestamp_ns", "right_timestamp_ns",
                "left_sequence", "right_sequence", "delta_us",
                "left_format", "right_format",
            )
            pairs = _read_csv_rows(pairs_path, fields)
            if len(pairs) != counts[0]:
                raise TaskError(
                    "capture pairs.csv count does not match camera images")
            for index, row in enumerate(pairs):
                try:
                    left = int(row["left_timestamp_ns"])
                    right = int(row["right_timestamp_ns"])
                except (TypeError, ValueError) as error:
                    raise TaskError(
                        "capture pairs.csv contains an invalid timestamp") from error
                if (left != camera_rows[camera_ids[0]][index]["timestamp_ns"]
                        or right != camera_rows[camera_ids[1]][index]["timestamp_ns"]):
                    raise TaskError(
                        "capture pairs.csv timestamps disagree with camera CSV files")
        if "pair_count" in session:
            try:
                pair_count = int(session["pair_count"])
            except (TypeError, ValueError) as error:
                raise TaskError("capture session pair_count is invalid") from error
            if pair_count != counts[0]:
                raise TaskError(
                    "capture session pair_count does not match camera images")
    elif camera_ids and "frame_count" in session:
        try:
            frame_count = int(session["frame_count"])
        except (TypeError, ValueError) as error:
            raise TaskError("capture session frame_count is invalid") from error
        if frame_count != len(camera_rows[camera_ids[0]]):
            raise TaskError(
                "capture session frame_count does not match camera images")

    if "imu_count" in session:
        try:
            session_imu_count = int(session["imu_count"])
        except (TypeError, ValueError) as error:
            raise TaskError("capture session imu_count is invalid") from error
        if session_imu_count != imu_count:
            raise TaskError("capture session imu_count does not match imu0.csv")

    temporary = sorted(
        relative for relative in scanned_files
        if Path(relative).suffix == ".tmp")
    if temporary:
        raise TaskError(
            "capture dataset contains temporary files: {}".format(
                ", ".join(temporary[:5])))
    for camera, rows in camera_rows.items():
        referenced = {row["filename"] for row in rows}
        actual = {
            relative for relative in camera_files[camera]
            if Path(relative).suffix.lower() == ".bmp"
        }
        extras = sorted(actual - referenced)
        if extras:
            raise TaskError(
                "capture dataset contains unreferenced {} images: {}".format(
                    camera, ", ".join(extras[:5])))


def _inventory(dataset, role):
    reader = _validate_dataset_role(dataset, role)
    manifest = reader.manifest
    session = _capture_session(dataset)
    scanned_files = _scan_dataset_files(dataset)
    structured = [_safe_child(dataset, "dataset.yaml", "dataset manifest")]
    snapshots = {}
    camera_rows = {}
    camera_files = {}
    ordered_images = []

    for index, camera in enumerate(manifest.get("cameras", [])):
        camera_id = camera["id"]
        label = "cameras[{}]".format(index)
        table = _safe_child(dataset, camera.get("timestamps"),
                            label + ".timestamps")
        image_root = _safe_child(dataset, camera.get("images"),
                                 label + ".images", directory=True)
        structured.append(table)
        topic = reader.sensor_topic(
            camera_id, "camera", camera.get("topic"))
        root_relative = image_root.relative_to(dataset)
        known_files = {}
        for relative, scanned in scanned_files.items():
            try:
                camera_relative = Path(relative).relative_to(root_relative)
            except ValueError:
                continue
            known_files[camera_relative.as_posix()] = scanned["path"]
        indexed = reader.index_images(topic, _known_files=known_files)
        rows = []
        try:
            for entry in indexed.index:
                path = Path(entry.path)
                try:
                    filename = path.relative_to(image_root).as_posix()
                except ValueError as error:
                    raise TaskError(
                        "camera image escapes its declared directory: {}".format(
                            path)) from error
                relative = path.relative_to(dataset).as_posix()
                scanned = scanned_files[relative]
                if scanned.get("symlink"):
                    raise TaskError(
                        "input path must not contain a symlink: {}".format(
                            path))
                metadata = scanned["metadata"]
                snapshot = {
                    **metadata,
                    "kind": "image",
                    "camera_id": camera_id,
                    "timestamp_ns": int(entry.header_timestamp_ns),
                }
                snapshots[relative] = snapshot
                row = {
                    "timestamp_ns": int(entry.header_timestamp_ns),
                    "filename": filename,
                    "relative_path": relative,
                    "size": metadata["size"],
                }
                rows.append(row)
                ordered_images.append((camera_id, len(rows) - 1, row))
        finally:
            indexed.close()
        camera_rows[camera_id] = rows
        camera_files[camera_id] = known_files

    imu_count = 0
    for index, imu in enumerate(manifest.get("imus", [])):
        path = _safe_child(
            dataset, imu.get("data"), "imus[{}].data".format(index))
        structured.append(path)
        imu_count += _count_csv_rows(
            path,
            ("timestamp_ns", "wx", "wy", "wz", "ax", "ay", "az"),
            ("temperature_c",))

    for path in structured:
        relative = path.relative_to(dataset).as_posix()
        before = scanned_files[relative]["metadata"]
        digest = _sha256(path)
        after = _file_metadata(path)
        if after != before:
            raise DatasetChangedError(
                "structured input changed while it was being hashed: {}".format(
                    path))
        snapshots[relative] = {
            **after, "kind": "structured", "sha256": digest}

    _validate_capture_contract(
        dataset, session, camera_rows, camera_files, scanned_files,
        imu_count)

    digest = hashlib.sha256()
    for relative in sorted(
            key for key, value in snapshots.items()
            if value["kind"] == "structured"):
        digest.update(b"S\0")
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(snapshots[relative]["sha256"].encode("ascii"))
        digest.update(b"\n")
    for camera_id, order, row in ordered_images:
        digest.update(b"I\0")
        digest.update(camera_id.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(order).encode("ascii"))
        digest.update(b"\0")
        digest.update(str(row["timestamp_ns"]).encode("ascii"))
        digest.update(b"\0")
        digest.update(row["relative_path"].encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(row["size"]).encode("ascii"))
        digest.update(b"\n")

    image_counts = {
        camera: len(rows) for camera, rows in camera_rows.items()}
    record = {
        "role": role,
        "path": str(dataset),
        "name": dataset.name,
        "dataset_id": reader.dataset_id,
        "sha256": digest.hexdigest(),
        "fingerprint_type": INVENTORY_FINGERPRINT_TYPE,
        "file_count": len(snapshots),
        "structured_file_count": len(structured),
        "image_count": sum(image_counts.values()),
        "image_counts": image_counts,
    }
    entry = {
        "role": role,
        "dataset_id": reader.dataset_id,
        "record": record,
        "files": snapshots,
    }
    total_bytes = sum(value["size"] for value in snapshots.values())
    structured_bytes = sum(
        value["size"] for value in snapshots.values()
        if value["kind"] == "structured")
    return record, entry, {
        "file_count": len(snapshots),
        "total_bytes": total_bytes,
        "image_count": record["image_count"],
        "image_counts": image_counts,
        "hashed_files": len(structured),
        "hashed_bytes": structured_bytes,
        "image_hashed_files": 0,
        "image_hashed_bytes": 0,
        "mode": "structured_hash_image_inventory",
    }


def _metadata_fingerprint(dataset, role, dataset_id, snapshots):
    digest = hashlib.sha256()
    for relative in sorted(snapshots):
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        metadata = snapshots[relative]
        for key in ("size", "mtime_ns", "ctime_ns", "inode", "device"):
            digest.update(str(metadata[key]).encode("ascii"))
            digest.update(b"\0")
        digest.update(b"\n")
    return {
        "role": role,
        "path": str(dataset),
        "name": dataset.name,
        "dataset_id": dataset_id,
        "sha256": digest.hexdigest(),
        "fingerprint_type": "metadata",
        "file_count": len(snapshots),
    }


def _valid_cached_record(value, dataset, role, dataset_id, file_count):
    return (isinstance(value, dict)
            and value.get("role") == role
            and value.get("path") == str(dataset)
            and value.get("name") == dataset.name
            and value.get("dataset_id") == dataset_id
            and value.get("file_count") == file_count
            and value.get("fingerprint_type", "content")
            in {"content", "metadata", INVENTORY_FINGERPRINT_TYPE}
            and isinstance(value.get("sha256"), str)
            and re.fullmatch(r"[0-9a-f]{64}", value["sha256"]) is not None)


def _empty_input_hash_cache():
    return {
        "schema_version": STATE_SCHEMA_VERSION,
        "kind": INPUT_HASH_CACHE_KIND,
        "datasets": {},
    }


def _load_input_hash_cache(path):
    path = Path(path)
    if path.is_symlink():
        raise TaskError("input hash cache must not be a symlink: {}".format(path))
    if not path.exists():
        return _empty_input_hash_cache()
    if not path.is_file():
        raise TaskError("input hash cache must be a regular file: {}".format(path))
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return _empty_input_hash_cache()
    if (not isinstance(value, dict)
            or not supports_schema_version(
                value.get("schema_version"), SUPPORTED_STATE_SCHEMA_VERSIONS)
            or value.get("kind") != INPUT_HASH_CACHE_KIND
            or not isinstance(value.get("datasets"), dict)):
        return _empty_input_hash_cache()
    return value


def _write_input_hash_cache(path, value):
    path = Path(path)
    if path.is_symlink():
        raise TaskError("input hash cache must not be a symlink: {}".format(path))
    _atomic_json(path, value)


def cache_entry_can_migrate(cached, expected_record, current):
    """Prove an old successful record from per-file historical evidence."""
    if (not isinstance(cached, dict)
            or cached.get("record") != expected_record
            or not isinstance(cached.get("files"), dict)
            or not isinstance(current, dict)
            or not isinstance(current.get("files"), dict)):
        return False
    old_files = cached["files"]
    new_files = current["files"]
    # Retired caches inventoried every file, including non-input meta/ files.
    # Every current algorithm input must be proven; harmless legacy extras do
    # not require a solve.
    if not set(new_files) <= set(old_files):
        return False
    for relative, present in new_files.items():
        historical = old_files.get(relative)
        if not isinstance(historical, dict):
            return False
        if present.get("kind") == "structured":
            if (_cached_file_hash(historical) is None
                    or historical["sha256"] != present.get("sha256")):
                return False
        elif (type(historical.get("size")) is not int
              or historical["size"] != present.get("size")):
            return False
    return True


def bind_cache_entry(entry, stage, delivery):
    value = deepcopy(entry)
    value["trust"] = "bound"
    value["bound_stage"] = stage
    value["bound_delivery"] = deepcopy(delivery)
    return value


def cache_entry_is_bound(entry, stage, delivery, record):
    return (isinstance(entry, dict)
            and entry.get("trust") == "bound"
            and entry.get("bound_stage") == stage
            and entry.get("bound_delivery") == delivery
            and entry.get("record") == record)


def _fingerprint_dataset(
        dataset, role, cached=None, progress=None, expected_record=None,
        force_content=False):
    """Hash structured inputs and inventory images without reading image bytes."""
    started = time.monotonic()
    dataset = Path(dataset).resolve()
    if progress is not None:
        progress("start", {"dataset": dataset.name, "role": role})
    record, entry, statistics = _inventory(dataset, role)
    statistics["elapsed_s"] = time.monotonic() - started
    if progress is not None:
        progress("complete", {
            "dataset": dataset.name, "role": role, **statistics})
    return record, entry, statistics


def dataset_fingerprint(dataset, role):
    """Validate structured content and image inventory without reading images."""
    return _fingerprint_dataset(dataset, role)[0]
