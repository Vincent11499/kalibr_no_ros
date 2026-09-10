"""Versioned public camera conversion around the existing numerical adapters."""

from pathlib import Path
import tempfile

from .task import TaskError, dump_yaml, load_yaml, require_document_version
from .version import SCHEMA_VERSION, VERSION


def converter_modules(full_fisheye=False):
    """Select pure Python OpenCV I/O without importing calibration bindings."""
    if full_fisheye:
        from . import opencv_fisheye_io as adapter
    else:
        from . import opencv_io as adapter
    return adapter, adapter


def _result_from_camchain(camchain):
    cameras = []
    for index in range(len(camchain)):
        key = "cam{}".format(index)
        entry = camchain.get(key)
        if not isinstance(entry, dict):
            raise TaskError("converted camera IDs must be contiguous cam0..camN")
        camera = dict(entry)
        cameras.append(dict(id=key, **camera))
    return {
        "schema_version": SCHEMA_VERSION, "kind": "calibration_result",
        "calibration_type": "cameras",
        "cameras": cameras,
    }


def _camchain_from_result(document):
    require_document_version(document, "camera calibration", "calibration_result")
    cameras = document.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise TaskError("camera calibration contains no cameras")
    result = {}
    for index, camera in enumerate(cameras):
        key = "cam{}".format(index)
        if not isinstance(camera, dict) or not isinstance(camera.get("id"), str):
            raise TaskError("camera result requires sensor IDs")
        result[key] = {name: value for name, value in camera.items()
                       if name not in {"id", "rms", "alignment", "from_camera"}}
    return result


def _validate_opencv_version(path):
    """Third-party OpenCV files may omit metadata; old project files may not."""
    import cv2

    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    try:
        if not storage.isOpened():
            raise TaskError("could not read OpenCV YAML: {}".format(path))
        version = storage.getNode("schema_version")
        if not version.empty() and (not version.isString() or version.string() != SCHEMA_VERSION):
            raise TaskError("OpenCV project schema_version must be {}".format(SCHEMA_VERSION))
    finally:
        storage.release()


def _add_opencv_metadata(path):
    kind = "opencv_stereo" if "stereo" in path.name else "opencv_camera"
    # Append scalar metadata without reserializing existing double matrices or
    # modifying OpenCV's independent %YAML:1.0 FileStorage format declaration.
    with path.open("a", encoding="utf-8") as stream:
        stream.write('schema_version: "{}"\nkind: "{}"\nsoftware_version: "{}"\n'.format(
            SCHEMA_VERSION, kind, VERSION))
    _validate_opencv_version(path)


def _publish_new_files(pairs):
    destinations = [Path(destination).expanduser().absolute() for _, destination in pairs]
    if len(destinations) != len(set(destinations)):
        raise TaskError("conversion output paths collide")
    for destination in destinations:
        if destination.exists() or destination.is_symlink():
            raise TaskError("conversion output already exists: {}".format(destination))
    created = []
    try:
        for (source, _), destination in zip(pairs, destinations):
            content = Path(source).read_bytes()
            destination.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive creation also protects against files appearing after
            # the complete multi-file preflight. Roll back only our new files.
            with destination.open("xb") as stream:
                created.append(destination)
                stream.write(content)
    except BaseException:
        for destination in created:
            destination.unlink()
        raise
    return destinations


def main(argv=None, *, full_fisheye=False):
    adapter, parser_module = converter_modules(full_fisheye)
    arguments = parser_module.build_argument_parser().parse_args(argv)
    with tempfile.TemporaryDirectory(prefix="kalibr-convert-") as directory:
        temporary = Path(directory)
        native_path = temporary / "camchain.yaml"
        if arguments.command == "export":
            dump_yaml(_camchain_from_result(load_yaml(arguments.input)), native_path)
            generated = adapter.export_kalibr_camchain(native_path, temporary / "converted")
            prefix = Path(arguments.output_prefix).expanduser()
            if prefix.suffix in {".yaml", ".yml"}:
                prefix = prefix.with_suffix("")
            pairs = []
            for path in generated:
                _add_opencv_metadata(path)
                pairs.append((path, Path(str(prefix) + path.name[len("converted"):])))
        else:
            optional = {} if full_fisheye else {"distortion_model": arguments.distortion_model}
            if arguments.command == "import-mono":
                _validate_opencv_version(arguments.input)
                adapter.import_opencv_mono(
                    arguments.input, native_path,
                    resolution=tuple(arguments.resolution) if arguments.resolution else None,
                    topic=arguments.topic, **optional)
            else:
                for source in (arguments.stereo, arguments.left, arguments.right):
                    if source:
                        _validate_opencv_version(source)
                adapter.import_opencv_stereo(
                    arguments.stereo, native_path, left_path=arguments.left, right_path=arguments.right,
                    resolutions=(tuple(arguments.left_resolution) if arguments.left_resolution else None,
                                 tuple(arguments.right_resolution) if arguments.right_resolution else None),
                    topics=tuple(arguments.topics) if arguments.topics else (None, None),
                    transform_direction=arguments.transform_direction,
                    translation_scale=arguments.translation_scale,
                    translation_unit=arguments.translation_unit, **optional)
            result = _result_from_camchain(load_yaml(native_path))
            destination = temporary / "calibration.yaml"
            dump_yaml(result, destination)
            if load_yaml(destination) != result:
                raise TaskError("camera conversion serialization changed parameter values")
            pairs = [(destination, arguments.output)]
        outputs = _publish_new_files(pairs)
    for output in outputs:
        print(output)
    return 0
