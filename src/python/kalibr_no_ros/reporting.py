"""Portable observation archives and reports, independent of native solvers."""

from __future__ import annotations

from contextlib import ExitStack
import copy
import csv
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile

import numpy as np

from .version import SCHEMA_VERSION, VERSION
from .evaluation import (
    ReportingError, assess_metrics, camera_geometry, compute_metrics,
    merged_cameras, paired_frames, stereo_geometry, validate_output_options,
    vector, prepare_evaluation_artifacts, rectification_maps,
)


def _child(root, relative):
    """Prevent artifact names and archive members from escaping their root."""
    root = Path(root).resolve()
    relative = Path(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise ReportingError("artifact path must be relative and contained: {}".format(relative))
    path = root / relative
    if path.is_symlink():
        raise ReportingError("artifact path must not be a symlink: {}".format(relative))
    try:
        path.resolve().relative_to(root)
    except ValueError as error:
        raise ReportingError("artifact path escapes its root: {}".format(relative)) from error
    return path


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError("unsupported report value: {}".format(type(value).__name__))


def _json(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, default=_json_default)


def _atomic_bytes(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _write_json(path, value):
    _atomic_bytes(path, (json.dumps(value, indent=2, ensure_ascii=False,
                                  allow_nan=False, default=_json_default) + "\n").encode("utf-8"))


def _csv_gzip(path, columns, rows):
    # JSON-valued cells preserve null, integers (including nanoseconds),
    # booleans and vector dimensions without platform-specific inference.
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=str(path.parent))
    count = 0
    try:
        with os.fdopen(descriptor, "wb") as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
                with io.TextIOWrapper(compressed, encoding="utf-8", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
                    writer.writeheader()
                    for row in rows:
                        writer.writerow({key: _json(row.get(key)) for key in columns})
                        count += 1
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return count


def _read_csv_gzip(path):
    with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
        return [{key: json.loads(value) for key, value in row.items()} for row in csv.DictReader(stream)]


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


_FRAME_COLUMNS = ["camera_id", "frame_id", "source_index", "source_timestamp_ns",
                  "record_timestamp_ns", "observation_timestamp_ns", "source_path",
                  "copied_image", "detection_status", "used", "view_id", "T_camera_target", "extra"]
_CORNER_COLUMNS = ["frame_id", "corner_id", "target_xyz_m", "measurement_px",
                   "prediction_px", "residual_px", "used", "extra"]


def write_archive(artifacts, output_dir):
    """Write lossless JSON-valued CSV.gz tables and a checksummed manifest."""
    output = Path(output_dir).resolve()
    cameras = [{key: value for key, value in camera.items() if key != "frames"}
               for camera in artifacts.get("cameras", [])]

    def frame_rows():
        for camera in artifacts.get("cameras", []):
            for frame in camera.get("frames", []):
                record = {key: frame.get(key) for key in _FRAME_COLUMNS if key not in ("camera_id", "extra")}
                record["camera_id"] = camera["id"]
                record["extra"] = {key: value for key, value in frame.items()
                                   if key not in _FRAME_COLUMNS and key != "corners"}
                yield record

    def corner_rows():
        for camera in artifacts.get("cameras", []):
            for frame in camera.get("frames", []):
                for corner in frame.get("corners", []):
                    if not artifacts.get("history_captured") and (not frame.get("used") or not corner.get("used", True)):
                        continue
                    record = {key: corner.get(key) for key in _CORNER_COLUMNS if key not in ("frame_id", "extra")}
                    record["frame_id"] = frame["frame_id"]
                    record["extra"] = {key: value for key, value in corner.items() if key not in _CORNER_COLUMNS}
                    yield record
    views = artifacts.get("views", [])
    pairs = []
    for left, right in zip(artifacts.get("cameras", []), artifacts.get("cameras", [])[1:]):
        for view, lf, rf in paired_frames(artifacts, left, right):
            pairs.append({"pair_id": left["id"] + "_" + right["id"], "view_id": view.get("view_id"),
                          "left_frame_id": lf["frame_id"], "right_frame_id": rf["frame_id"],
                          "pairing": "native_target_view" if artifacts.get("calibration_type") == "cameras" else "camera_imu_diagnostic_pairing"})
    tables = {
        "frames.csv.gz": (_FRAME_COLUMNS, frame_rows()),
        "corners.csv.gz": (_CORNER_COLUMNS, corner_rows()),
        "views.csv.gz": (["view"], [{"view": view} for view in views]),
        "pairs.csv.gz": (["pair_id", "view_id", "left_frame_id", "right_frame_id", "pairing"], pairs),
        "imu_residuals.csv.gz": (["residual"], ({"residual": r} for r in artifacts.get("imu_residuals", []))),
    }
    if artifacts.get("history_captured"):
        tables["selection_events.csv.gz"] = (["event"], [{"event": event} for event in artifacts.get("events", [])])
    biases = artifacts.get("imu_biases", [])
    if biases:
        tables["imu_biases.csv.gz"] = (["imu_id", "kind", "sample"],
            ({"imu_id": bias["imu_id"], "kind": bias["kind"], "sample": sample}
             for bias in biases for sample in bias.get("samples", [])))
    files = []
    manifest = {key: value for key, value in artifacts.items()
                if key not in ("cameras", "views", "events", "imu_residuals", "imu_biases")}
    manifest.update({"schema_version": SCHEMA_VERSION, "software_version": VERSION,
                     "kind": "run_artifacts_archive", "cameras": cameras,
                     "csv_cell_encoding": "json", "tables": {}})
    if biases:
        manifest["imu_biases"] = [{key: value for key, value in bias.items() if key != "samples"}
                                  for bias in biases]
    for name, (columns, rows) in tables.items():
        relative = "observations/" + name
        path = _child(output, relative)
        count = _csv_gzip(path, columns, rows)
        manifest["tables"][name] = {"path": name, "sha256": _sha256(path), "rows": count}
        files.append(relative)
    relative = "observations/manifest.json"
    _write_json(_child(output, relative), manifest)
    return files + [relative]


def load_archive(input_dir):
    """Load and verify an archive without touching images or importing Kalibr."""
    root = Path(input_dir).resolve()
    if root.name == "observations" and (root / "manifest.json").is_file():
        archive = root
    else:
        archive = _child(root, "observations")
    with _child(archive, "manifest.json").open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    if manifest.get("schema_version") != SCHEMA_VERSION or manifest.get("kind") != "run_artifacts_archive":
        raise ReportingError("unsupported observation archive version or kind")
    tables = {}
    names = ["frames.csv.gz", "corners.csv.gz", "views.csv.gz", "pairs.csv.gz", "imu_residuals.csv.gz"]
    if manifest.get("history_captured"):
        names.append("selection_events.csv.gz")
    if manifest.get("imu_biases"):
        names.append("imu_biases.csv.gz")
    for name in names:
        spec = manifest.get("tables", {}).get(name)
        if not isinstance(spec, dict):
            raise ReportingError("missing archive table " + name)
        path = _child(archive, spec.get("path", ""))
        if _sha256(path) != spec.get("sha256"):
            raise ReportingError("observation archive checksum mismatch: " + name)
        rows = _read_csv_gzip(path)
        if len(rows) != spec.get("rows"):
            raise ReportingError("observation archive row count mismatch: " + name)
        tables[name] = rows
    artifacts = {key: value for key, value in manifest.items()
                 if key not in ("tables", "csv_cell_encoding", "software_version")}
    artifacts["kind"] = "run_artifacts"
    cameras = {camera["id"]: dict(camera, frames=[]) for camera in manifest["cameras"]}
    frames = {}
    for row in tables["frames.csv.gz"]:
        if row["camera_id"] not in cameras or row["frame_id"] in frames:
            raise ReportingError("invalid camera reference or duplicate frame ID in archive")
        for timestamp in ("source_timestamp_ns", "record_timestamp_ns", "observation_timestamp_ns"):
            if row.get(timestamp) is not None and type(row[timestamp]) is not int:
                raise ReportingError("archived {} must be integer nanoseconds".format(timestamp))
        frame = {key: value for key, value in row.items() if key not in ("extra", "camera_id")}
        frame.update(row.get("extra") or {})
        frame["corners"] = []
        frames[frame["frame_id"]] = frame
        cameras[row["camera_id"]]["frames"].append(frame)
    for row in tables["corners.csv.gz"]:
        if row["frame_id"] not in frames:
            raise ReportingError("unknown frame ID in corners archive")
        corner = {key: value for key, value in row.items() if key not in ("extra", "frame_id")}
        corner.update(row.get("extra") or {})
        frames[row["frame_id"]]["corners"].append(corner)
    artifacts["cameras"] = list(cameras.values())
    artifacts["views"] = [row["view"] for row in tables["views.csv.gz"]]
    artifacts["imu_residuals"] = [row["residual"] for row in tables["imu_residuals.csv.gz"]]
    artifacts["events"] = [row["event"] for row in tables.get("selection_events.csv.gz", [])]
    biases = {(bias["imu_id"], bias["kind"]): dict(bias, samples=[]) for bias in manifest.get("imu_biases", [])}
    for row in tables.get("imu_biases.csv.gz", []):
        key = (row["imu_id"], row["kind"])
        if key not in biases:
            raise ReportingError("unknown IMU reference in bias archive")
        timestamp = row["sample"].get("timestamp_ns")
        if timestamp is not None and type(timestamp) is not int:
            raise ReportingError("bias timestamp_ns must be integer nanoseconds")
        biases[key]["samples"].append(row["sample"])
    artifacts["imu_biases"] = list(biases.values())
    return artifacts


def export_opencv(artifacts, calibration, output_dir, options=None):
    """Export OpenCV FileStorage matrices with explicit model and direction."""
    import cv2

    options = validate_output_options(options)
    cameras = merged_cameras(artifacts, calibration)
    files, status = [], []

    def save(relative, matrices, metadata):
        path = _child(output_dir, relative)
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(suffix=".yaml", dir=str(path.parent))
        os.close(descriptor)
        try:
            storage = cv2.FileStorage(temporary, cv2.FILE_STORAGE_WRITE)
            try:
                if not storage.isOpened():
                    raise ReportingError("cannot open OpenCV export")
                storage.write("schema_version", SCHEMA_VERSION)
                storage.write("software_version", VERSION)
                for name, value in metadata.items():
                    storage.write(name, value)
                for name, value in matrices.items():
                    storage.write(name, np.asarray(value, dtype=np.float64))
            finally:
                storage.release()
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        files.append(relative)

    for camera in cameras:
        try:
            geometry = camera_geometry(camera)
            metadata = {"kind": "opencv_camera", "model": geometry["model"],
                        "image_width": geometry["size"][0], "image_height": geometry["size"][1],
                        "distortion_order": "k1,k2,k3,k4" if geometry["family"] == "fisheye" else "opencv_radtan"}
            if geometry["model"] == "pinhole-opencv-fisheye":
                metadata.update(camera_model="fisheye", distortion_model="fisheye",
                                kalibr_camera_model="pinhole_opencv_fisheye",
                                kalibr_distortion_model="opencv_fisheye",
                                alpha=geometry["alpha"], skew=float(geometry["K"][0, 1]))
            save("opencv/{}.yaml".format(camera["id"]), {"K": geometry["K"], "D": geometry["D"].reshape(-1, 1)},
                 metadata)
            status.append({"camera": camera["id"], "status": "written"})
        except (ReportingError, cv2.error) as error:
            status.append({"camera": camera["id"], "status": "unavailable", "reason": str(error)})
    for left, right in zip(cameras, cameras[1:]):
        identifier = left["id"] + "_" + right["id"]
        try:
            geometry = stereo_geometry(left, right, options["rectification"])
            matrices = {name: geometry[name] for name in ("R", "T", "R1", "R2", "P1", "P2", "Q")}
            matrices["T"] = matrices["T"].reshape(3, 1)
            for prefix, side in (("1", "left"), ("2", "right")):
                matrices["K" + prefix] = geometry[side]["K"]
                matrices["D" + prefix] = geometry[side]["D"].reshape(-1, 1)
            metadata = {"kind": "opencv_stereo", "left_camera": left["id"], "right_camera": right["id"],
                        "model": geometry["left"]["model"], "translation_unit": "m",
                        "transform_convention": "p_right = R * p_left + T",
                        "rectified_width": geometry["size"][0], "rectified_height": geometry["size"][1],
                        "balance": options["rectification"]["balance"], "fov_scale": options["rectification"]["fov_scale"],
                        "zero_disparity": 1, "disparity_axis": geometry["disparity_axis"]}
            if any(geometry[side]["model"] == "pinhole-opencv-fisheye" for side in ("left", "right")):
                metadata.update(camera_model="fisheye", distortion_model="fisheye",
                                kalibr_camera_model="pinhole_opencv_fisheye",
                                kalibr_distortion_model="opencv_fisheye",
                                transform_direction="cam0-to-cam1",
                                rectification_skew_handling="unskew_input_points_and_reskew_source_maps")
                for side, number in (("left", "1"), ("right", "2")):
                    source = geometry[side]
                    metadata["alpha" + number] = source["alpha"]
                    metadata["skew" + number] = float(source["K"][0, 1])
                    metadata[side + "_image_width"] = source["size"][0]
                    metadata[side + "_image_height"] = source["size"][1]
                    matrices["K" + number + "_no_skew"] = source["K_no_skew"]
            save("opencv/{}.yaml".format(identifier), matrices, metadata)
            status.append({"pair": identifier, "status": "written"})
        except (ReportingError, cv2.error) as error:
            status.append({"pair": identifier, "status": "unavailable", "reason": str(error)})
    return files, status


class _Images:
    def __init__(self, artifacts, input_dir=None, dataset=None):
        self.stack = ExitStack()
        self.input_dir = Path(input_dir).resolve() if input_dir else None
        self.dataset = dataset or (artifacts.get("dataset") or {}).get("path")
        self.override = dataset is not None
        self.reader = None
        self.streams = {}

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return self.stack.__exit__(*args)

    def read(self, camera, frame):
        import cv2

        if not self.override:
            copied = frame.get("copied_image")
            if copied and self.input_dir:
                path = _child(self.input_dir, copied)
                if path.is_file():
                    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
                    if image is not None:
                        return image
            source = frame.get("source_path")
            if source and Path(source).is_file():
                image = cv2.imread(str(source), cv2.IMREAD_COLOR)
                if image is not None:
                    return image
        if not self.dataset:
            return None
        from kalibr_bag_io.factory import open_dataset

        if self.reader is None:
            self.reader = open_dataset(self.dataset)
        expected_id = frame.get("dataset_id")
        if expected_id is not None and getattr(self.reader, "dataset_id", None) != expected_id:
            raise ReportingError("replacement dataset_id differs from archived observation")
        topic = camera.get("topic", camera.get("rostopic"))
        from kalibr_bag_io.directory import DirectoryReader
        if isinstance(self.reader, DirectoryReader):
            topic = self.reader.sensor_topic(camera["id"], "camera")
        if topic not in self.streams:
            self.streams[topic] = self.stack.enter_context(self.reader.index_images(topic, grayscale=False))
        stream = self.streams[topic]
        index = frame.get("source_index")
        timestamp = frame.get("source_timestamp_ns")
        if type(index) is not int or index < 0 or index >= len(stream.index):
            return None
        entry = stream.index[index]
        if timestamp is not None and entry.header_timestamp_ns != timestamp:
            # A replacement dataset must describe the same observation stream;
            # never silently associate a nearby frame with archived corners.
            return None
        return stream.get(index).image


def _uniform(items, maximum):
    if len(items) <= maximum:
        return items
    return [items[int(i)] for i in np.linspace(0, len(items) - 1, maximum).round().astype(int)]


def create_images(artifacts, calibration, output_dir, options=None, input_dir=None, dataset=None):
    import cv2

    options = validate_output_options(options)
    cameras = merged_cameras(artifacts, calibration)
    files, problems = [], []

    def save(relative, pixels):
        success, data = cv2.imencode(Path(relative).suffix, pixels)
        if not success:
            raise ReportingError("image encoding failed")
        _atomic_bytes(_child(output_dir, relative), data.tobytes())
        files.append(relative)

    def read(images, camera, frame):
        try:
            image = images.read(camera, frame)
        except (OSError, ValueError, RuntimeError, ImportError, cv2.error) as error:
            problems.append({"frame_id": frame["frame_id"], "reason": str(error)})
            return None
        if image is None:
            problems.append({"frame_id": frame["frame_id"], "reason": "source image unavailable"})
        elif list(image.shape[1::-1]) != list(camera.get("resolution", image.shape[1::-1])):
            problems.append({"frame_id": frame["frame_id"], "reason": "source resolution differs from calibrated observation"})
            return None
        return image

    visual = options["visualizations"]
    with _Images(artifacts, input_dir=input_dir, dataset=dataset) as images:
        for camera in cameras:
            frames = [frame for frame in camera.get("frames", []) if frame.get("used")]
            if options["copy_used_images"]:
                for frame in frames:
                    source = read(images, camera, frame)
                    if source is None:
                        continue
                    relative = "images/{}/{}.png".format(camera["id"], frame["source_index"])
                    save(relative, source)
                    frame["copied_image"] = relative
            if not visual["enabled"]:
                continue
            for frame in _uniform(frames, visual["max_frames_per_camera"]):
                pixels = read(images, camera, frame)
                if pixels is None:
                    continue
                if pixels.ndim == 2:
                    pixels = cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
                pixels = pixels.copy()
                for corner in frame.get("corners", []):
                    measurement = vector(corner.get("measurement_px"), 2)
                    prediction = vector(corner.get("prediction_px"), 2)
                    if measurement is not None:
                        color = (0, 200, 0) if corner.get("used", True) else (0, 0, 220)
                        cv2.circle(pixels, tuple(np.rint(measurement).astype(int)), 3, color, 1)
                    if measurement is not None and prediction is not None:
                        cv2.line(pixels, tuple(np.rint(measurement).astype(int)), tuple(np.rint(prediction).astype(int)), (255, 100, 0), 1)
                save("visualizations/{}/corners_{}.jpg".format(camera["id"], frame["source_index"]), pixels)
        if visual["enabled"]:
            for left, right in zip(cameras, cameras[1:]):
                try:
                    geometry = stereo_geometry(left, right, options["rectification"])
                except (ReportingError, cv2.error) as error:
                    problems.append({"pair": left["id"] + "_" + right["id"], "reason": str(error)})
                    continue
                maps = [rectification_maps(geometry, side) for side in ("left", "right")]
                pairs = sorted(paired_frames(artifacts, left, right),
                               key=lambda pair: (pair[1]['source_index'], pair[2]['source_index']))
                for index, (_, lf, rf) in enumerate(_uniform(pairs, visual["max_pairs"])):
                    originals = [read(images, left, lf), read(images, right, rf)]
                    if any(p is None for p in originals):
                        continue
                    originals = [cv2.cvtColor(p, cv2.COLOR_GRAY2BGR) if p.ndim == 2 else p for p in originals]
                    height = max(p.shape[0] for p in originals)
                    raw_pair = np.hstack([np.pad(p, ((0, height - p.shape[0]), (0, 0), (0, 0))) for p in originals])
                    save("visualizations/{}_{}/original_{:04d}.jpg".format(left["id"], right["id"], index), raw_pair)
                    corrected = [cv2.remap(p, *mapping, interpolation=cv2.INTER_LINEAR) for p, mapping in zip(originals, maps)]
                    canvas = np.hstack(corrected)
                    if geometry["disparity_axis"] == "x":
                        for y in range(0, canvas.shape[0], max(1, canvas.shape[0] // 12)):
                            cv2.line(canvas, (0, y), (canvas.shape[1] - 1, y), (0, 255, 0), 3)
                    else:
                        width = geometry["size"][0]
                        for x in range(0, width, max(1, width // 12)):
                            for offset in (0, width):
                                cv2.line(canvas, (x + offset, 0), (x + offset, canvas.shape[0] - 1), (0, 255, 0), 3)
                    save("visualizations/{}_{}/rectified_{:04d}.jpg".format(left["id"], right["id"], index), canvas)
    return files, problems


def _format(value):
    if value is None:
        return "unavailable"
    if isinstance(value, float):
        return "{:.6g}".format(value)
    return str(value)


def _summary(metrics, assessment):
    lines = ["Calibration report v{}".format(VERSION),
             "Type: {}".format(metrics.get("calibration_type")),
             "Formal assessment: {}".format(assessment["status"]),
             "Reference grade: {} (display only)".format(assessment["reference_grading"]["status"]),
             "Population: final used observations; 2D corner RMS in pixels.", ""]
    for name, camera in metrics.get("cameras", {}).items():
        values = camera["reprojection"]
        lines.extend(["Camera {}: {} / {} selected frames used; total source frames={}".format(
                         name, camera["used_frames"], camera.get("selected_frames", camera["input_frames"]), _format(camera["input_frames"])),
                      "  corners={}, RMS={} px, mean={} px, P95={} px, max={} px".format(values["count"], _format(values.get("rms_px")), _format(values["mean"]), _format(values["p95"]), _format(values["max"]))])
        lines.append("  intrinsics={} distortion={}".format(camera.get("intrinsics"), camera.get("distortion_coeffs")))
        lines.append("  whitened residual vector RMS={} (dimensionless)".format(_format(camera.get("normalized_reprojection", {}).get("rms"))))
        if "timeshift_cam_imu_s" in camera:
            lines.append("  camera/IMU time shift={} s".format(_format(camera["timeshift_cam_imu_s"])))
    for name, pair in metrics.get("stereo_pairs", {}).items():
        lines.append("Stereo {}: {}, pairing={}".format(name, pair["status"], pair["pairing"]))
        if pair.get("alignment"):
            lines.append("  common corners={}, mean alignment={} px, baseline={} m".format(pair["alignment"]["count"], _format(pair["alignment"]["mean"]), _format(pair.get("baseline_m"))))
        if pair.get("reason"):
            lines.append("  " + pair["reason"])
        if pair.get("interpretation"):
            lines.append("  " + pair["interpretation"])
    for name, imu in metrics.get("imus", {}).items():
        for kind, values in imu.items():
            lines.append("IMU {} {}: samples={}, vector RMS={} {}".format(name, kind, values["count"], _format(values.get("rms")), values["unit"]))
            lines.append("  whitened residual vector RMS={} (dimensionless)".format(_format(values.get("normalized_residual", {}).get("rms"))))
            bias = values.get("bias_spline")
            if bias:
                lines.append("  Bias: time-varying spline sampled at retained IMU timestamps; {} / {} values available".format(
                    bias["count"], bias["total_sample_count"]))
                for axis in ("x", "y", "z"):
                    stats = bias["axes"][axis]
                    lines.append("    {} sample mean={}, std={}, min={}, max={} {}".format(
                        axis, _format(stats["mean"]), _format(stats["std"]), _format(stats["min"]), _format(stats["max"]), bias["unit"]))
    if assessment["rules"]:
        lines.extend(["", "Formal rules (inclusive bounds):"])
        for rule in assessment["rules"]:
            lines.append("  {}: value={}, min={}, max={}, {}".format(rule["metric"], _format(rule["value"]), _format(rule.get("min")), _format(rule.get("max")), rule["status"]))
    if metrics.get("optimizer"):
        lines.extend(["", "Native optimizer summary; scope: {}".format(
            metrics["optimizer"].get("scope", "unavailable")), _json(metrics["optimizer"])])
    if metrics.get("objective"):
        lines.extend(["", "Final retained objective; scope: {}".format(
            metrics["objective"].get("scope", "unavailable")), _json(metrics["objective"])])
    for problem in metrics.get("output_diagnostics", []):
        lines.append("Output diagnostic: " + _json(problem))
    return "\n".join(lines) + "\n"


def _write_reports(metrics, assessment, output_dir, files, native_text=None, *,
                   artifacts=None, calibration=None):
    from .report_plots import render_reports

    summary = _summary(metrics, assessment)
    text = summary + ("\nNative solver results\n" + native_text if native_text else "")
    _atomic_bytes(_child(output_dir, "results.txt"), text.encode("utf-8"))
    document, pdf = render_reports(metrics, assessment, files,
                                   artifacts=artifacts, calibration=calibration, output_dir=output_dir)
    _atomic_bytes(_child(output_dir, "report.pdf"), pdf)
    _atomic_bytes(_child(output_dir, "report.html"), document.encode("utf-8"))
    return ["results.txt", "report.html", "report.pdf"]


def generate_report(artifacts, calibration, output_dir, options=None, *, input_dir=None, dataset=None):
    """Produce metrics and optional evidence; never alter calibration.yaml."""
    options = validate_output_options(options)
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    artifacts = dict(artifacts)
    # Final public calibration remains authoritative over seed-time metadata.
    artifacts["cameras"] = merged_cameras(artifacts, calibration)
    if options["copy_used_images"]:
        for camera in artifacts["cameras"]:
            camera["frames"] = [dict(frame) for frame in camera["frames"]]
    artifacts = prepare_evaluation_artifacts(artifacts, options)
    metrics = compute_metrics(artifacts, calibration, options)
    files, diagnostics = [], []
    if options["export_opencv"]:
        generated, statuses = export_opencv(artifacts, calibration, output, options)
        files.extend(generated)
        diagnostics.extend(status for status in statuses if status["status"] != "written")
    if options["copy_used_images"] or options["visualizations"]["enabled"]:
        generated, problems = create_images(artifacts, calibration, output, options, input_dir=input_dir, dataset=dataset)
        files.extend(generated)
        diagnostics.extend(problems)
    if options["archive_observations"]:
        if not options["archive_selection_history"]:
            artifacts["events"] = []
            artifacts["history_captured"] = False
        files.extend(write_archive(artifacts, output))
    metrics["output_diagnostics"] = diagnostics
    assessment = assess_metrics(metrics, options)
    _write_json(_child(output, "metrics.json"), metrics)
    _write_json(_child(output, "assessment.json"), assessment)
    files.extend(["metrics.json", "assessment.json"])
    native_path = _child(output, "results.txt")
    native_text = native_path.read_text(encoding="utf-8") if native_path.is_file() else None
    files.extend(_write_reports(metrics, assessment, output, files, native_text=native_text,
                                artifacts=artifacts, calibration=calibration))
    return {"metrics": metrics, "assessment": assessment, "files": sorted(files)}


def evaluate_run(input_dir, output_dir, options=None, dataset=None, force=False):
    """Accept a named result YAML (or an unambiguous run directory)."""
    import tempfile
    import shutil
    from .delivery import locate_result, publish, published_paths
    from .task import load_yaml, require_document_version
    if Path(input_dir).is_dir() and (Path(input_dir) / 'metrics.json').is_file() and not list(Path(input_dir).glob('*.yaml')):
        source = Path(input_dir).resolve()
        destination = Path(output_dir).resolve()
        if source == destination or source in destination.parents or destination in source.parents:
            raise ReportingError('evaluation output must be separate from the source run')
        options = validate_output_options(options)
        with tempfile.TemporaryDirectory(prefix='kalibr-evaluation-') as folder:
            target = Path(folder) / 'evaluated'
            result = _evaluate_staged(source, target, options, dataset=dataset)
            name = options.get('name') or 'evaluation'
            publish(target, destination, name, options, force=force)
            result['files'] = published_paths(target, destination, name, options)
            return result
    source_result = locate_result(input_dir)
    require_document_version(load_yaml(source_result), 'calibration', 'calibration_result')
    options = validate_output_options(options)
    source_root = source_result.parent
    evidence = source_root if source_result.name == 'calibration.yaml' else source_root / source_result.stem
    output = Path(output_dir).resolve()
    if output == source_root or output in source_root.parents or source_root in output.parents:
        raise ReportingError('evaluation output must be separate from the source run')
    name = options.get('name') or (source_result.stem if source_result.name != 'calibration.yaml' else 'camera_calibration_' + '_'.join(c['id'] for c in load_yaml(source_result)['cameras']))
    with tempfile.TemporaryDirectory(prefix='kalibr-evaluation-') as folder:
        stage = Path(folder)
        source, target = stage / 'source', stage / 'evaluated'
        source.mkdir()
        shutil.copy2(source_result, source / 'calibration.yaml')
        for item in ('observations', 'metrics.json', 'opencv', 'images'):
            path = evidence / item
            if path.is_dir():
                shutil.copytree(path, source / item)
            elif path.is_file():
                shutil.copy2(path, source / item)
        if not (source / 'metrics.json').is_file() and not (source / 'observations/manifest.json').is_file():
            raise ReportingError('offline evaluation requires saved evidence: enable output.archive_observations or output.save_metrics during calibration')
        result = _evaluate_staged(source, target, options, dataset=dataset)
        publish(target, output, name, options, force=force)
        result['files'] = published_paths(target, output, name, options)
        return result


def _evaluate_staged(input_dir, output_dir, options=None, dataset=None):
    """Re-evaluate archived corners, or re-assess saved numeric summaries."""
    import yaml

    options = validate_output_options(options)
    source, output = Path(input_dir).resolve(), Path(output_dir).resolve()
    if source == output or source in output.parents or output in source.parents:
        raise ReportingError("evaluation output must be separate from the source run")
    if (source / "observations" / "manifest.json").is_file():
        artifacts = load_archive(source)
        with _child(source, "calibration.yaml").open(encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream)
        if not isinstance(calibration, dict):
            raise ReportingError("calibration.yaml must contain a mapping")
        output.mkdir(parents=True, exist_ok=True)
        # Copy the original result byte-for-byte, including numeric formatting.
        _atomic_bytes(_child(output, "calibration.yaml"), _child(source, "calibration.yaml").read_bytes())
        result = generate_report(artifacts, calibration, output, options, input_dir=source, dataset=dataset)
        result["files"].append("calibration.yaml")
        return result
    with _child(source, "metrics.json").open(encoding="utf-8") as stream:
        metrics = json.load(stream)
    if metrics.get("schema_version") != SCHEMA_VERSION or metrics.get("kind") != "calibration_metrics":
        raise ReportingError("unsupported metrics version or kind")
    if options["archive_observations"] or options["archive_selection_history"] or options["copy_used_images"] or options["visualizations"]["enabled"]:
        raise ReportingError("image/evidence export requires an observation archive")
    saved_options = metrics.get("evaluation_options", {})
    defaults = validate_output_options()
    if options["rectification"] != saved_options.get("rectification", defaults["rectification"]):
        raise ReportingError("changing rectification requires an observation archive")
    if options["evaluation_pairing_tolerance_s"] != saved_options.get("evaluation_pairing_tolerance_s", defaults["evaluation_pairing_tolerance_s"]):
        raise ReportingError("changing evaluation pairing requires an observation archive")
    metrics = copy.deepcopy(metrics)
    metrics["evaluation_mode"] = "saved_summary_only"
    assessment = assess_metrics(metrics, options)
    output.mkdir(parents=True, exist_ok=True)
    files = ["metrics.json", "assessment.json"]
    calibration = _child(source, "calibration.yaml")
    if calibration.is_file():
        _atomic_bytes(_child(output, "calibration.yaml"), calibration.read_bytes())
        files.append("calibration.yaml")
    if options["export_opencv"]:
        exports = sorted(_child(source, "opencv").glob("*.yaml"))
        for path in exports:
            relative = "opencv/" + path.name
            _atomic_bytes(_child(output, relative), _child(source, relative).read_bytes())
            files.append(relative)
        if not exports:
            metrics.setdefault("output_diagnostics", []).append({"export_opencv": "unavailable",
                "reason": "summary-only evaluation preserves saved OpenCV exports; none were saved"})
        metrics["opencv_evaluation_mode"] = "saved_exports_only"
    _write_json(_child(output, "metrics.json"), metrics)
    _write_json(_child(output, "assessment.json"), assessment)
    files.extend(_write_reports(metrics, assessment, output, files))
    return {"metrics": metrics, "assessment": assessment, "files": sorted(files)}
