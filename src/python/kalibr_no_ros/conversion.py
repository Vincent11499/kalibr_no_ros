"""Versioned public camera conversion around the existing numerical adapters."""

from contextlib import contextmanager
from functools import lru_cache
import importlib.machinery
import importlib.util
from pathlib import Path
import sys
import tempfile
import types

from .task import TaskError, dump_yaml, load_yaml, require_document_version
from .version import SCHEMA_VERSION, VERSION


def _package_directory(package):
    # Finding the package does not execute its native-binding __init__.py.
    specification = importlib.machinery.PathFinder.find_spec(package)
    if specification and specification.submodule_search_locations:
        return Path(next(iter(specification.submodule_search_locations)))
    source = Path(__file__).resolve().parents[2] / "camera_models/opencv_fisheye/python" / package
    if source.is_dir():
        return source
    raise TaskError("camera converter package is unavailable: {}".format(package))


def _load_module(name, path):
    specification = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    try:
        specification.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module


@contextmanager
def _module_alias(name, module):
    previous = sys.modules.get(name)
    sys.modules[name] = module
    try:
        yield
    finally:
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


@lru_cache(maxsize=2)
def converter_modules(full_fisheye=False):
    """Load pure YAML helpers while leaving native package imports untouched."""
    zero_path = _package_directory("kalibr_opencv_fisheye")
    zero_name = "kalibr_no_ros._conversion_zero"
    zero = sys.modules.get(zero_name) or _load_module(zero_name, zero_path / "yaml_io.py")
    if not full_fisheye:
        return zero, zero
    full_path = _package_directory("kalibr_opencv_fisheye_full")
    private_name = "kalibr_no_ros._conversion_full"
    private_package = types.ModuleType(private_name)
    private_package.__path__ = [str(full_path)]
    sys.modules[private_name] = private_package
    # The full helper imports the zero-skew helper via its public package.
    # Provide that one dependency for module loading, then restore the caller's
    # package object; subsequent calibration may need its actual C++ bindings.
    zero_package = types.ModuleType("kalibr_opencv_fisheye")
    zero_package.yaml_io = zero
    with _module_alias("kalibr_opencv_fisheye", zero_package):
        full = _load_module(private_name + ".yaml_io", full_path / "yaml_io.py")
    parser_module = _load_module(private_name + ".__main__", full_path / "__main__.py")
    return full, parser_module


def _result_from_camchain(camchain):
    cameras = []
    for index in range(len(camchain)):
        key = "cam{}".format(index)
        entry = camchain.get(key)
        if not isinstance(entry, dict):
            raise TaskError("converted camera IDs must be contiguous cam0..camN")
        camera = dict(entry)
        if camera.get("camera_model") == "pinhole" and camera.get("distortion_model") == "opencv_fisheye":
            camera["distortion_model"] = "equidistant"
        cameras.append(dict(id=key, **camera))
    return {
        "schema_version": SCHEMA_VERSION, "kind": "calibration_result",
        "calibration_type": "cameras",
        "transform_convention": "p_target = T_target_source * p_source",
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
        if not isinstance(camera, dict) or camera.get("id") != key:
            raise TaskError("camera result IDs must be contiguous cam0..camN")
        result[key] = {name: value for name, value in camera.items() if name != "id"}
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
