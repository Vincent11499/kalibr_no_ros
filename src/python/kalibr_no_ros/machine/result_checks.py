"""Inspect machine-stage deliveries and verify their provenance."""

from __future__ import annotations

from pathlib import Path
import hashlib

from ..filesystem import file_sha256 as _sha256
from ..results.delivery import MACHINE_LAYOUT
from ..task.document import (
    TaskError, canonical_document, load_yaml,
)
from ..task.jobs import CAMERA_BASELINK_JOB, CAMERA_RS_JOB
from ..version import (
    SUPPORTED_RESULT_SCHEMA_VERSIONS,
    supports_schema_version,
)
from .task_templates import _resolved_task


def _tree_sha256(root):
    root = Path(root)
    digest = hashlib.sha256()
    for path in sorted(
            (item for item in root.rglob("*") if item.is_file()),
            key=lambda item: item.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix()
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(_sha256(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _expected_result_paths(output, name):
    task_output = output / name
    return {
        "result": task_output / (name + ".yaml"),
        "html": output / (name + ".report.html"),
        "pdf": output / (name + ".report.pdf"),
        "sidecar": task_output,
    }


def _delivery_exists(output, name):
    paths = _expected_result_paths(output, name)
    legacy = [
        output / (name + suffix)
        for suffix in (".yaml", ".report.html", ".report.pdf", ".results.txt")
    ]
    return any(path.exists() or path.is_symlink()
               for path in [*paths.values(), *legacy])


def _delivery_summary(inspected):
    sidecar = inspected["sidecar"]
    return {
        "result": str(inspected["result"].resolve()),
        "files": {
            key: _sha256(inspected[key]) for key in ("result", "html", "pdf")
        },
        "run_manifest_sha256": _sha256(inspected["manifest_path"]),
        "task_resolved_sha256": _sha256(inspected["task_resolved"]),
        "sidecar_sha256": _tree_sha256(sidecar),
    }


def _managed_delivery_summary(output, name):
    from ..results.delivery import inspect_managed_delivery
    inspected = inspect_managed_delivery(output, name, layout=MACHINE_LAYOUT)
    return _delivery_summary(inspected)


def _delivery_name(output, delivery):
    result = Path(delivery.get("result", ""))
    name = result.parent.name
    expected = _expected_result_paths(output, name)["result"]
    if not name or result.resolve() != expected.resolve():
        raise TaskError("workflow checkpoint has an invalid delivery path")
    return name


def _verify_previous_delivery(output, delivery):
    name = _delivery_name(output, delivery)
    if _managed_delivery_summary(output, name) != delivery:
        raise TaskError(
            "managed output changed outside the workflow: {}".format(name))
    return name


def _manifest_input(manifest, role):
    entries = manifest.get("input_documents")
    if not isinstance(entries, list):
        return None
    matches = [entry for entry in entries
               if isinstance(entry, dict) and entry.get("role") == role]
    return matches[0].get("document") if len(matches) == 1 else None


def _semantic_task(document, manifest=None):
    """Compare tasks across compatible schemas and relocated installations."""
    value = canonical_document(document)
    for role, key in (("target", "target"),
                      ("initialization", "initialization")):
        block = value.get(key)
        if not isinstance(block, dict) or not block.get("path"):
            continue
        source = (_manifest_input(manifest, role) if manifest is not None
                  else load_yaml(block["path"]))
        if not isinstance(source, dict):
            raise TaskError("delivery {} input document is missing".format(role))
        block["path"] = canonical_document(source)
    for index, imu in enumerate(value.get("imus", [])):
        if not isinstance(imu, dict) or not imu.get("path"):
            continue
        role = "imu{}".format(index)
        source = (_manifest_input(manifest, role) if manifest is not None
                  else load_yaml(imu["path"]))
        if not isinstance(source, dict):
            raise TaskError("delivery {} input document is missing".format(role))
        imu["path"] = canonical_document(source)
    camera = value.get("camera_calibration")
    if isinstance(camera, dict) and camera.get("path"):
        camera["path"] = "__CAMERA_CALIBRATION__"
    return value


def _validate_delivery(output, name, expected_job, dataset_record,
                       task_path, camera_dependency=None):
    from ..results.delivery import inspect_managed_delivery
    inspected = inspect_managed_delivery(output, name, layout=MACHINE_LAYOUT)
    paths = {key: inspected[key] for key in ("result", "html", "pdf", "sidecar")}

    result = load_yaml(paths["result"])
    expected_type = {
        "camera_calibration": "cameras",
        CAMERA_RS_JOB: "cameras",
        CAMERA_BASELINK_JOB: "cameras_baselink",
    }.get(expected_job, "camera_imu")
    if (not supports_schema_version(
            result.get("schema_version"), SUPPORTED_RESULT_SCHEMA_VERSIONS)
            or result.get("kind") != "calibration_result"
            or result.get("calibration_type") != expected_type):
        raise TaskError("delivery result has an unexpected schema or calibration type")
    result_cameras = result.get("cameras")
    if expected_job == CAMERA_BASELINK_JOB:
        generated_task = load_yaml(task_path)
        camera_ids = [camera.get("id")
                      for camera in generated_task.get("cameras", [])]
    else:
        camera_ids = (["cam0"]
                      if dataset_record["role"] == "mono"
                      else ["cam0", "cam1"])
    if (not isinstance(result_cameras, list)
            or any(not isinstance(camera, dict)
                   for camera in result_cameras)
            or [camera.get("id") for camera in result_cameras]
            != camera_ids):
        raise TaskError("delivery result must contain cameras {} in order".format(camera_ids))
    if expected_type == "camera_imu":
        result_imus = result.get("imus")
        if (not isinstance(result_imus, list)
                or any(not isinstance(imu, dict) for imu in result_imus)
                or [imu.get("id") for imu in result_imus] != ["imu0"]):
            raise TaskError("Camera-IMU result must contain imu0")

    task_resolved_path = inspected["task_resolved"]
    manifest = inspected["manifest"]
    if (not supports_schema_version(
            manifest.get("schema_version"), SUPPORTED_RESULT_SCHEMA_VERSIONS)
            or manifest.get("kind") != "calibration_run"
            or manifest.get("status") != "completed"
            or manifest.get("job") != expected_job):
        raise TaskError("delivery run manifest is not a completed expected job")
    manifest_dataset = manifest.get("dataset", {})
    if (not isinstance(manifest_dataset, dict)
            or Path(str(manifest_dataset.get("path", ""))).resolve()
            != Path(dataset_record["path"]).resolve()
            or manifest_dataset.get("id") != dataset_record["dataset_id"]):
        raise TaskError("delivery dataset provenance does not match")
    if manifest.get("task_sha256") != _sha256(task_resolved_path):
        raise TaskError("delivery resolved task hash does not match its manifest")
    resolved_task = _resolved_task(task_path, expected_job)
    if (_semantic_task(load_yaml(task_resolved_path), manifest)
            != _semantic_task(resolved_task)):
        raise TaskError("delivery resolved task does not match the generated task")
    if expected_type == "camera_imu":
        expected_model = resolved_task["imus"][0].get("model", "calibrated")
        if result_imus[0].get("model") != expected_model:
            raise TaskError(
                "Camera-IMU result model does not match the generated task")

    input_documents = manifest.get("input_documents")
    if (not isinstance(input_documents, list)
            or any(not isinstance(entry, dict)
                   for entry in input_documents)):
        raise TaskError("delivery has invalid input document provenance")
    semantic_inputs = [
        (role, resolved_task.get(role))
        for role in ("target", "initialization")
    ]
    semantic_inputs.extend(
        ("imu{}".format(index), block)
        for index, block in enumerate(resolved_task.get("imus", [])))
    for role, block in semantic_inputs:
        if not isinstance(block, dict) or not block.get("path"):
            continue
        matches = [entry for entry in input_documents
                   if entry.get("role") == role]
        if (len(matches) != 1
                or not isinstance(matches[0].get("document"), dict)
                or canonical_document(matches[0]["document"])
                != canonical_document(load_yaml(block["path"]))):
            raise TaskError(
                "delivery {} input document does not match".format(role))

    if camera_dependency is not None:
        expected_path = str(Path(camera_dependency["path"]).resolve())
        matches = [entry for entry in input_documents
                   if entry.get("role") == "camera_calibration"]
        if (len(matches) != 1
                or str(Path(matches[0].get("path", "")).resolve()) != expected_path
                or matches[0].get("sha256") != camera_dependency["sha256"]):
            raise TaskError("delivery references a different camera result")

    return _delivery_summary(inspected)
