"""OpenCV FileStorage I/O for native equidistant and radial-tangential models.

OpenCV does not prescribe field names for calibration files.  This module
accepts the common ``K/D`` and ``camera_matrix/distortion_coefficients`` mono
forms and the conventional stereo ``K1/D1/K2/D2/R/T`` form.  The transform
stored by OpenCV and Kalibr has the same default direction: it maps a point in
camera 0 (or camera n-1) into camera 1 (or camera n).
"""

from __future__ import print_function

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import yaml


DIRECT_TRANSFORM = "cam0-to-cam1"
INVERSE_TRANSFORM = "cam1-to-cam0"

_TRANSLATION_UNIT_TO_METERS = {
    "m": 1.0,
    "meter": 1.0,
    "meters": 1.0,
    "cm": 1e-2,
    "centimeter": 1e-2,
    "centimeters": 1e-2,
    "mm": 1e-3,
    "millimeter": 1e-3,
    "millimeters": 1e-3,
    "um": 1e-6,
    "micrometer": 1e-6,
    "micrometers": 1e-6,
}


@dataclass(frozen=True)
class OpenCvCamera:
    K: np.ndarray
    D: np.ndarray
    resolution: Tuple[int, int]
    distortion_model: str = "opencv_fisheye"
    name: str = "camera"
    topic: str = ""

    def __post_init__(self):
        coefficient_count = np.asarray(self.D).size
        object.__setattr__(
            self,
            "distortion_model",
            _normalise_model_for_coefficients(
                self.distortion_model, coefficient_count
            ),
        )
        object.__setattr__(self, "K", _validate_camera_matrix(self.K))
        object.__setattr__(
            self,
            "D",
            _validate_distortion(self.D, self.distortion_model),
        )
        object.__setattr__(self, "resolution", _validate_resolution(self.resolution))


@dataclass(frozen=True)
class OpenCvStereo:
    left: OpenCvCamera
    right: OpenCvCamera
    R: np.ndarray
    T: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, "R", _validate_rotation(self.R))
        object.__setattr__(self, "T", _validate_translation(self.T))

    @property
    def transform(self):
        result = np.eye(4, dtype=np.float64)
        result[:3, :3] = self.R
        result[:3, 3] = self.T.reshape(3)
        return result


def _normalise_model(model):
    if model is None:
        raise ValueError("distortion model is required")
    model = str(model).strip().lower().replace("-", "_")
    if model in {"opencv_fisheye", "fisheye", "equidistant"}:
        return "opencv_fisheye"
    if model in {"radtan5", "plumb_bob"}:
        return "radtan5"
    if model in {"radtan8", "rational_polynomial"}:
        return "radtan8"
    if model == "radtan":
        return "radtan"
    raise ValueError("unsupported distortion model {!r}".format(model))


def _normalise_model_for_coefficients(model, coefficient_count):
    """Resolve OpenCV's ambiguous ``plumb_bob`` name from D length."""

    normalized = str(model).strip().lower().replace("-", "_")
    if normalized == "plumb_bob":
        if coefficient_count == 4:
            return "radtan"
        if coefficient_count == 5:
            return "radtan5"
        raise ValueError(
            "plumb_bob requires 4 or 5 supported coefficients, got {}".format(
                coefficient_count
            )
        )
    if normalized == "rational_polynomial":
        if coefficient_count != 8:
            raise ValueError(
                "rational_polynomial requires 8 coefficients, got {}".format(
                    coefficient_count
                )
            )
        return "radtan8"
    return _normalise_model(model)


def _validate_camera_matrix(matrix, skew_tolerance=1e-12):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("camera matrix K must have shape (3, 3), got {}".format(matrix.shape))
    if not np.all(np.isfinite(matrix)):
        raise ValueError("camera matrix K contains non-finite values")
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    if abs(matrix[0, 1]) > skew_tolerance:
        raise ValueError(
            "Kalibr's pinhole projection is zero-skew, but K[0,1]={}".format(
                matrix[0, 1]
            )
        )
    expected_last_rows = np.array([0.0, 0.0, 1.0])
    if not np.allclose(matrix[2], expected_last_rows, rtol=0.0, atol=skew_tolerance):
        raise ValueError("camera matrix K must end in [0, 0, 1]")
    if abs(matrix[1, 0]) > skew_tolerance:
        raise ValueError("camera matrix K[1,0] must be zero")
    result = matrix.copy()
    result[0, 1] = 0.0
    result[1, 0] = 0.0
    result[2] = expected_last_rows
    return result


def _validate_distortion(coefficients, model):
    coefficients = np.asarray(coefficients, dtype=np.float64).reshape(-1)
    model = _normalise_model_for_coefficients(model, coefficients.size)
    expected = {
        "opencv_fisheye": 4,
        "radtan": 4,
        "radtan5": 5,
        "radtan8": 8,
    }[model]
    if coefficients.size != expected:
        raise ValueError(
            "distortion model {} requires {} coefficients, got {}".format(
                model, expected, coefficients.size
            )
        )
    if not np.all(np.isfinite(coefficients)):
        raise ValueError("distortion coefficients contain non-finite values")
    return coefficients.copy()


def _validate_resolution(resolution):
    if resolution is None or len(resolution) != 2:
        raise ValueError("image resolution [width, height] is required")
    width, height = int(resolution[0]), int(resolution[1])
    if width <= 0 or height <= 0:
        raise ValueError("image resolution must be positive")
    return width, height


def _validate_rotation(rotation, tolerance=1e-7):
    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("stereo rotation R must have shape (3, 3)")
    if not np.all(np.isfinite(rotation)):
        raise ValueError("stereo rotation contains non-finite values")
    if not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=tolerance, rtol=0.0):
        raise ValueError("stereo rotation R is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=tolerance, rtol=0.0):
        raise ValueError("stereo rotation R must have determinant +1")
    return rotation.copy()


def _validate_translation(translation):
    translation = np.asarray(translation, dtype=np.float64).reshape(-1)
    if translation.size != 3:
        raise ValueError("stereo translation T must contain three values")
    if not np.all(np.isfinite(translation)):
        raise ValueError("stereo translation contains non-finite values")
    return translation.reshape(3, 1).copy()


def _open_storage(path, flags):
    if flags != cv2.FILE_STORAGE_READ:
        storage = cv2.FileStorage(str(path), flags)
        if not storage.isOpened():
            raise RuntimeError("could not open OpenCV YAML {}".format(path))
        return storage

    # cv::FileStorage requires its own YAML directive.  A large amount of
    # production calibration software instead writes ordinary YAML containing
    # flat K/D/RT lists.  Try the native reader first, then fall back to a tiny
    # FileNode-compatible adapter backed by PyYAML.
    storage = None
    try:
        storage = cv2.FileStorage(str(path), flags)
        if storage.isOpened():
            return storage
    except (cv2.error, SystemError):
        storage = None
    if storage is not None:
        storage.release()
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError("could not open camera YAML {}: {}".format(path, error))
    if not isinstance(document, dict):
        raise ValueError("camera YAML {} must contain a mapping".format(path))
    return _PlainYamlStorage(document)


_MISSING = object()


class _PlainYamlNode:
    """Subset of cv2.FileNode used by the converter."""

    def __init__(self, value=_MISSING):
        self.value = value

    def empty(self):
        return self.value is _MISSING or self.value is None

    def mat(self):
        if self.empty():
            return None
        value = self.value
        if isinstance(value, dict):
            if not {"rows", "cols", "data"}.issubset(value):
                return None
            rows = int(value["rows"])
            cols = int(value["cols"])
            data = np.asarray(value["data"], dtype=np.float64).reshape(-1)
            if rows <= 0 or cols <= 0 or data.size != rows * cols:
                raise ValueError("inconsistent rows/cols/data matrix")
            return data.reshape(rows, cols)
        if isinstance(value, (list, tuple)):
            return np.asarray(value, dtype=np.float64)
        return None

    def string(self):
        return self.value if isinstance(self.value, str) else ""

    def real(self):
        if self.empty():
            return 0.0
        return float(self.value)

    def isMap(self):
        return isinstance(self.value, dict)

    def isSeq(self):
        return isinstance(self.value, (list, tuple))

    def getNode(self, name):
        if not isinstance(self.value, dict):
            return _PlainYamlNode()
        return _PlainYamlNode(self.value.get(name, _MISSING))

    def at(self, index):
        if not self.isSeq():
            return _PlainYamlNode()
        return _PlainYamlNode(self.value[index])

    def size(self):
        return len(self.value) if self.isSeq() else 0


class _PlainYamlStorage:
    def __init__(self, document):
        self.document = document

    def getNode(self, name):
        return _PlainYamlNode(self.document.get(name, _MISSING))

    def release(self):
        pass


def _node(storage, names):
    for name in names:
        candidate = storage.getNode(name)
        if not candidate.empty():
            return candidate
    return None


def _read_matrix(storage, names, required=True):
    node = _node(storage, names)
    if node is None:
        if required:
            raise ValueError("missing OpenCV matrix {}; tried {}".format(names[0], names))
        return None
    matrix = None
    try:
        matrix = node.mat()
    except cv2.error:
        # ROS camera-calibration YAML uses the same rows/cols/data mapping as
        # an OpenCV matrix but omits the ``dt`` tag required by FileNode.mat().
        # Plain YAML sequences are also common for D.
        if node.isMap():
            rows_node = node.getNode("rows")
            cols_node = node.getNode("cols")
            data_node = node.getNode("data")
            if not rows_node.empty() and not cols_node.empty() and data_node.isSeq():
                rows = int(round(rows_node.real()))
                cols = int(round(cols_node.real()))
                values = [data_node.at(index).real() for index in range(data_node.size())]
                if rows <= 0 or cols <= 0 or len(values) != rows * cols:
                    raise ValueError(
                        "OpenCV node {} has inconsistent rows/cols/data".format(
                            names[0]
                        )
                    )
                matrix = np.asarray(values, dtype=np.float64).reshape(rows, cols)
        elif node.isSeq():
            matrix = np.asarray(
                [node.at(index).real() for index in range(node.size())],
                dtype=np.float64,
            )
    if matrix is None:
        raise ValueError(
            "OpenCV node {} is not an opencv-matrix, rows/cols/data map, or sequence".format(
                names[0]
            )
        )
    return np.asarray(matrix, dtype=np.float64)


def _read_string(storage, names, default=""):
    node = _node(storage, names)
    if node is None:
        return default
    value = node.string()
    return value if value else default


def _read_integer(storage, names):
    node = _node(storage, names)
    if node is None:
        return None
    return int(round(node.real()))


def _read_real(storage, names, default=0.0):
    node = _node(storage, names)
    return float(node.real()) if node is not None else float(default)


def _optional_real(storage, names):
    node = _node(storage, names)
    if node is None:
        return None
    value = float(node.real())
    if not np.isfinite(value):
        raise ValueError("{} must be finite".format(names[0]))
    return value


def _translation_unit_scale(unit):
    normalized = str(unit).strip().lower().replace("µ", "u")
    try:
        return _TRANSLATION_UNIT_TO_METERS[normalized]
    except KeyError:
        raise ValueError(
            "unsupported translation unit {!r}; use m, cm, mm, or um".format(
                unit
            )
        )


def _translation_scale_to_meters(
    storage, override_scale=None, override_unit=None, tolerance=1e-12
):
    """Return the multiplier from stored T units to Kalibr meters."""

    if override_scale is not None and override_unit is not None:
        raise ValueError("specify translation_scale or translation_unit, not both")
    if override_scale is not None:
        scale = float(override_scale)
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError("translation_scale must be finite and positive")
        return scale
    if override_unit is not None:
        return _translation_unit_scale(override_unit)

    stored_scale = _optional_real(storage, ("translation_scale", "T_scale"))
    stored_unit = _read_string(
        storage, ("translation_unit", "T_unit", "baseline_unit"), ""
    )
    unit_scale = _translation_unit_scale(stored_unit) if stored_unit else None
    if stored_scale is not None:
        if stored_scale <= 0.0:
            raise ValueError("stored translation_scale must be positive")
        if unit_scale is not None and not np.isclose(
            stored_scale, unit_scale, rtol=tolerance, atol=tolerance
        ):
            raise ValueError(
                "translation_scale={} conflicts with translation_unit={!r}".format(
                    stored_scale, stored_unit
                )
            )
        return stored_scale
    return unit_scale if unit_scale is not None else 1.0


def _check_zero_skew_alpha(storage, prefix="", tolerance=1e-12):
    names = (
        (
            "alpha{}".format(prefix),
            "skew{}".format(prefix),
            "alpha",
            "skew",
        )
        if prefix
        else ("alpha", "skew")
    )
    alpha = _read_real(storage, names)
    if abs(alpha) > tolerance:
        raise ValueError(
            "zero-skew OpenCV fisheye requires alpha=0, but {}={}".format(
                names[0], alpha
            )
        )


def _read_resolution(storage, prefix, fallback):
    if prefix == "1":
        width_names = ("left_image_width", "image_width1", "image_width")
        height_names = ("left_image_height", "image_height1", "image_height")
    elif prefix == "2":
        width_names = ("right_image_width", "image_width2", "image_width")
        height_names = ("right_image_height", "image_height2", "image_height")
    else:
        width_names = ("image_width", "camera_width", "width")
        height_names = ("image_height", "camera_height", "height")
    width = _read_integer(storage, width_names)
    height = _read_integer(storage, height_names)
    if width is None or height is None:
        return _validate_resolution(fallback)
    return _validate_resolution((width, height))


def _model_from_storage(storage, default_model, coefficient_count):
    declared = _read_string(
        storage,
        ("kalibr_distortion_model", "distortion_model"),
        "",
    )
    if not declared:
        camera_model = _read_string(storage, ("camera_model",), "")
        if camera_model.lower() not in ("", "pinhole"):
            declared = camera_model
    if declared:
        return _normalise_model_for_coefficients(declared, coefficient_count)
    # Five and eight coefficients unambiguously select this project's OpenCV
    # radial extensions.  Four coefficients remain ambiguous between
    # plumb-bob and fisheye, so retain the caller's explicit/default choice.
    if coefficient_count == 5:
        return "radtan5"
    if coefficient_count == 8:
        return "radtan8"
    return _normalise_model_for_coefficients(default_model, coefficient_count)


def _reshape_matrix(matrix, shape, field):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape == shape:
        return matrix
    if matrix.size == int(np.prod(shape)):
        return matrix.reshape(shape)
    raise ValueError(
        "{} must have shape {} or contain {} values, got {}".format(
            field, shape, int(np.prod(shape)), matrix.shape
        )
    )


def read_opencv_camera(
    path,
    *,
    prefix="",
    resolution=None,
    distortion_model="opencv_fisheye",
    topic="",
    name=None
):
    """Read one camera from an OpenCV FileStorage YAML file.

    ``prefix`` may be ``"1"`` or ``"2"`` for a stereo file containing
    ``K1/D1`` and ``K2/D2``.  A four-element D is interpreted as OpenCV
    fisheye by default; pass ``distortion_model="radtan"`` for the distinct
    four-parameter plumb-bob model.
    """

    storage = _open_storage(path, cv2.FILE_STORAGE_READ)
    try:
        _check_zero_skew_alpha(storage, prefix)
        if prefix:
            K = _read_matrix(storage, ("K{}".format(prefix), "M{}".format(prefix)))
            D = _read_matrix(storage, ("D{}".format(prefix),))
            camera_name = _read_string(
                storage,
                (("left_camera", "camera_name1") if prefix == "1" else ("right_camera", "camera_name2")),
                name or "cam{}".format(int(prefix) - 1),
            )
            camera_topic = topic or _read_string(
                storage,
                (("left_topic", "ros_topic1") if prefix == "1" else ("right_topic", "ros_topic2")),
            )
        else:
            K = _read_matrix(storage, ("K", "camera_matrix"))
            D = _read_matrix(storage, ("D", "distortion_coefficients"))
            camera_name = name or _read_string(storage, ("camera_name",), "camera")
            camera_topic = topic or _read_string(storage, ("ros_topic", "topic"))
        K = _reshape_matrix(K, (3, 3), "K{}".format(prefix))
        model = _model_from_storage(
            storage, distortion_model, np.asarray(D).size
        )
        return OpenCvCamera(
            K=K,
            D=D,
            resolution=_read_resolution(storage, prefix, resolution),
            distortion_model=model,
            name=camera_name,
            topic=camera_topic,
        )
    finally:
        storage.release()


def invert_stereo_transform(rotation, translation):
    """Invert ``p_to = R p_from + T`` without changing coordinate handedness."""

    rotation = _validate_rotation(rotation)
    translation = _validate_translation(translation)
    inverse_rotation = rotation.T
    inverse_translation = -inverse_rotation.dot(translation)
    return inverse_rotation, inverse_translation


def read_opencv_stereo(
    stereo_path,
    *,
    left_path=None,
    right_path=None,
    resolutions=None,
    topics=(None, None),
    distortion_model="opencv_fisheye",
    transform_direction=None,
    translation_scale=None,
    translation_unit=None,
):
    """Read a conventional OpenCV stereo calibration.

    ``stereo_path`` may contain all of ``K1,D1,K2,D2,R,T``.  If it contains
    only extrinsics, supply ``left_path`` and ``right_path`` mono files.
    OpenCV's normal direction is camera 0 to camera 1 and maps directly to
    Kalibr's ``T_c1_c0``/``T_cn_cnm1``.  A ``transform_direction`` stored in
    the file is honored unless the caller supplies an override.  OpenCV does
    not standardize the unit of T; absent metadata or an override, values are
    preserved and therefore assumed to already be meters.
    """

    if resolutions is None:
        resolutions = (None, None)
    if len(resolutions) != 2 or len(topics) != 2:
        raise ValueError("stereo conversion requires two resolutions and two topics")

    storage = _open_storage(stereo_path, cv2.FILE_STORAGE_READ)
    try:
        has_embedded_intrinsics = _node(storage, ("K1", "M1")) is not None
        rotation = _read_matrix(storage, ("R",), required=False)
        translation = _read_matrix(storage, ("T",), required=False)
        if rotation is None or translation is None:
            transform = _read_matrix(
                storage, ("RT", "T_cn_cnm1"), required=False
            )
            if transform is None:
                raise ValueError(
                    "stereo YAML requires R/T or a 4x4 RT transform"
                )
            transform = _reshape_matrix(transform, (4, 4), "RT")
            if not np.allclose(
                transform[3], [0.0, 0.0, 0.0, 1.0],
                rtol=0.0, atol=1e-12,
            ):
                raise ValueError("stereo RT has an invalid final row")
            rotation = transform[:3, :3]
            translation = transform[:3, 3:4]
        else:
            rotation = _reshape_matrix(rotation, (3, 3), "R")
            translation = _reshape_matrix(translation, (3, 1), "T")
        declared_direction = _read_string(
            storage, ("transform_direction",), ""
        )
        scale_to_meters = _translation_scale_to_meters(
            storage, translation_scale, translation_unit
        )
    finally:
        storage.release()

    if has_embedded_intrinsics:
        left = read_opencv_camera(
            stereo_path,
            prefix="1",
            resolution=resolutions[0],
            distortion_model=distortion_model,
            topic=topics[0],
            name="cam0",
        )
        right = read_opencv_camera(
            stereo_path,
            prefix="2",
            resolution=resolutions[1],
            distortion_model=distortion_model,
            topic=topics[1],
            name="cam1",
        )
    else:
        if left_path is None or right_path is None:
            raise ValueError(
                "stereo YAML has no K1/K2; both left_path and right_path are required"
            )
        left = read_opencv_camera(
            left_path,
            resolution=resolutions[0],
            distortion_model=distortion_model,
            topic=topics[0],
            name="cam0",
        )
        right = read_opencv_camera(
            right_path,
            resolution=resolutions[1],
            distortion_model=distortion_model,
            topic=topics[1],
            name="cam1",
        )

    translation = np.asarray(translation, dtype=np.float64) * scale_to_meters
    direction = (
        transform_direction
        if transform_direction is not None
        else declared_direction or DIRECT_TRANSFORM
    )
    if direction == INVERSE_TRANSFORM:
        rotation, translation = invert_stereo_transform(rotation, translation)
    elif direction != DIRECT_TRANSFORM:
        raise ValueError(
            "transform_direction must be {!r} or {!r}, got {!r}".format(
                DIRECT_TRANSFORM, INVERSE_TRANSFORM, direction
            )
        )
    return OpenCvStereo(left, right, rotation, translation)


def _camera_to_kalibr(camera, index, overlaps):
    model = _normalise_model(camera.distortion_model)
    if model == "opencv_fisheye":
        model = "equidistant"
    return {
        "camera_model": "pinhole",
        "intrinsics": [
            float(camera.K[0, 0]),
            float(camera.K[1, 1]),
            float(camera.K[0, 2]),
            float(camera.K[1, 2]),
        ],
        "distortion_model": model,
        "distortion_coeffs": [float(value) for value in camera.D],
        "resolution": [int(camera.resolution[0]), int(camera.resolution[1])],
        "rostopic": camera.topic or "/cam{}/image_raw".format(index),
        "cam_overlaps": list(overlaps),
    }


def opencv_mono_to_camchain(camera):
    return {"cam0": _camera_to_kalibr(camera, 0, [])}


def opencv_stereo_to_camchain(stereo):
    cam0 = _camera_to_kalibr(stereo.left, 0, [1])
    cam1 = _camera_to_kalibr(stereo.right, 1, [0])
    cam1["T_cn_cnm1"] = stereo.transform.tolist()
    return {"cam0": cam0, "cam1": cam1}


def write_kalibr_camchain(data, output_path):
    output_path = Path(output_path)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, default_flow_style=None, sort_keys=False)
    return output_path


def import_opencv_mono(
    input_path,
    output_path,
    *,
    resolution=None,
    topic=None,
    distortion_model="opencv_fisheye"
):
    camera = read_opencv_camera(
        input_path,
        resolution=resolution,
        topic=topic,
        distortion_model=distortion_model,
        name="cam0",
    )
    return write_kalibr_camchain(opencv_mono_to_camchain(camera), output_path)


def import_opencv_stereo(
    stereo_path,
    output_path,
    *,
    left_path=None,
    right_path=None,
    resolutions=None,
    topics=(None, None),
    distortion_model="opencv_fisheye",
    transform_direction=None,
    translation_scale=None,
    translation_unit=None,
):
    stereo = read_opencv_stereo(
        stereo_path,
        left_path=left_path,
        right_path=right_path,
        resolutions=resolutions,
        topics=topics,
        distortion_model=distortion_model,
        transform_direction=transform_direction,
        translation_scale=translation_scale,
        translation_unit=translation_unit,
    )
    return write_kalibr_camchain(opencv_stereo_to_camchain(stereo), output_path)


def _camera_from_kalibr(entry, index):
    if entry.get("camera_model") != "pinhole":
        raise ValueError("OpenCV export supports only pinhole camchain entries")
    intrinsics = np.asarray(entry.get("intrinsics"), dtype=np.float64).reshape(-1)
    if intrinsics.size != 4:
        raise ValueError("cam{} pinhole intrinsics must contain four values".format(index))
    K = np.array(
        [
            [intrinsics[0], 0.0, intrinsics[2]],
            [0.0, intrinsics[1], intrinsics[3]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    model = _normalise_model(entry.get("distortion_model"))
    coefficients = np.asarray(entry.get("distortion_coeffs"), dtype=np.float64)
    return OpenCvCamera(
        K,
        coefficients,
        tuple(entry.get("resolution", ())),
        model,
        "cam{}".format(index),
        str(entry.get("rostopic", "")),
    )


def read_kalibr_camchain(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or "cam0" not in data:
        raise ValueError("{} is not a Kalibr camchain".format(path))
    cameras = []
    transforms = []
    index = 0
    while "cam{}".format(index) in data:
        entry = data["cam{}".format(index)]
        cameras.append(_camera_from_kalibr(entry, index))
        if index:
            transform = np.asarray(entry.get("T_cn_cnm1"), dtype=np.float64)
            if transform.shape != (4, 4):
                raise ValueError("cam{} is missing a 4x4 T_cn_cnm1".format(index))
            if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0):
                raise ValueError("cam{} T_cn_cnm1 has an invalid final row".format(index))
            transforms.append(
                (_validate_rotation(transform[:3, :3]), _validate_translation(transform[:3, 3]))
            )
        index += 1
    if len(cameras) != len(data):
        unexpected = sorted(set(data) - {"cam{}".format(i) for i in range(len(cameras))})
        if unexpected:
            raise ValueError("non-contiguous or unknown camchain keys: {}".format(unexpected))
    return cameras, transforms


def _opencv_model_name(model):
    normalized = _normalise_model(model)
    if normalized == "opencv_fisheye":
        return "fisheye"
    if normalized == "radtan8":
        return "rational_polynomial"
    return "plumb_bob"


def _write_values(path, values):
    storage = _open_storage(path, cv2.FILE_STORAGE_WRITE)
    try:
        for name, value in values:
            storage.write(name, value)
    finally:
        storage.release()


def write_opencv_camera(camera, path):
    model = _normalise_model(camera.distortion_model)
    D = camera.D.reshape((-1, 1) if model == "opencv_fisheye" else (1, -1))
    _write_values(
        path,
        [
            ("camera_name", camera.name),
            ("ros_topic", camera.topic),
            ("image_width", int(camera.resolution[0])),
            ("image_height", int(camera.resolution[1])),
            ("camera_model", "fisheye" if model == "opencv_fisheye" else "pinhole"),
            ("distortion_model", _opencv_model_name(model)),
            ("kalibr_distortion_model", model),
            ("alpha", 0.0),
            ("K", camera.K),
            ("D", D),
            # These aliases make the same file consumable by tools following
            # ROS camera-calibration naming rather than OpenCV sample naming.
            ("camera_matrix", camera.K),
            ("distortion_coefficients", D),
        ],
    )
    return Path(path)


def _skew(vector):
    x, y, z = np.asarray(vector, dtype=np.float64).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def write_opencv_stereo(stereo, path):
    left_model = _normalise_model(stereo.left.distortion_model)
    right_model = _normalise_model(stereo.right.distortion_model)
    if left_model != right_model:
        raise ValueError("OpenCV stereo export requires the same model on both cameras")
    D1 = stereo.left.D.reshape((-1, 1) if left_model == "opencv_fisheye" else (1, -1))
    D2 = stereo.right.D.reshape((-1, 1) if right_model == "opencv_fisheye" else (1, -1))
    essential = _skew(stereo.T).dot(stereo.R)
    fundamental = np.linalg.inv(stereo.right.K).T.dot(essential).dot(
        np.linalg.inv(stereo.left.K)
    )
    _write_values(
        path,
        [
            ("left_camera", stereo.left.name),
            ("right_camera", stereo.right.name),
            ("left_topic", stereo.left.topic),
            ("right_topic", stereo.right.topic),
            ("left_image_width", int(stereo.left.resolution[0])),
            ("left_image_height", int(stereo.left.resolution[1])),
            ("right_image_width", int(stereo.right.resolution[0])),
            ("right_image_height", int(stereo.right.resolution[1])),
            ("camera_model", "fisheye" if left_model == "opencv_fisheye" else "pinhole"),
            ("distortion_model", _opencv_model_name(left_model)),
            ("kalibr_distortion_model", left_model),
            ("transform_direction", DIRECT_TRANSFORM),
            ("translation_unit", "m"),
            ("translation_scale", 1.0),
            ("alpha1", 0.0),
            ("alpha2", 0.0),
            ("K1", stereo.left.K),
            ("D1", D1),
            ("K2", stereo.right.K),
            ("D2", D2),
            ("R", stereo.R),
            ("T", stereo.T),
            ("E", essential),
            ("F", fundamental),
        ],
    )
    return Path(path)


def export_kalibr_camchain(camchain_path, output_prefix):
    """Export every camera and adjacent baseline from a Kalibr camchain."""

    cameras, transforms = read_kalibr_camchain(camchain_path)
    prefix = str(output_prefix)
    if prefix.endswith(".yaml") or prefix.endswith(".yml"):
        prefix = str(Path(prefix).with_suffix(""))
    outputs = []
    for index, camera in enumerate(cameras):
        path = Path("{}-cam{}-opencv.yaml".format(prefix, index))
        outputs.append(write_opencv_camera(camera, path))
    for index, (rotation, translation) in enumerate(transforms):
        stereo = OpenCvStereo(cameras[index], cameras[index + 1], rotation, translation)
        path = Path("{}-cam{}-cam{}-opencv-stereo.yaml".format(prefix, index, index + 1))
        outputs.append(write_opencv_stereo(stereo, path))
    return outputs


def build_argument_parser(*, full_fisheye=False):
    parser = argparse.ArgumentParser(
        description=(
            "Convert OpenCV fisheye YAML including alpha/skew"
            if full_fisheye else
            "Convert zero-skew OpenCV calibration YAML and Kalibr camchains"
        )
    )
    commands = parser.add_subparsers(dest="command", required=True)

    mono = commands.add_parser("import-mono", help="OpenCV K/D -> Kalibr camchain")
    mono.add_argument("--input", required=True)
    mono.add_argument("--output", required=True)
    mono.add_argument(
        "--topic",
        help="Kalibr image topic; preserves ros_topic from input when omitted",
    )
    mono.add_argument("--resolution", nargs=2, type=int, metavar=("WIDTH", "HEIGHT"))
    if not full_fisheye:
        mono.add_argument(
            "--distortion-model",
            choices=("opencv_fisheye", "radtan", "radtan5", "radtan8"),
            default="opencv_fisheye",
        )

    stereo = commands.add_parser(
        "import-stereo", help="OpenCV K1/D1/K2/D2/R/T -> Kalibr camchain"
    )
    stereo.add_argument("--stereo", required=True)
    stereo.add_argument("--left")
    stereo.add_argument("--right")
    stereo.add_argument("--output", required=True)
    stereo.add_argument(
        "--topics",
        nargs=2,
        help="Kalibr image topics; preserves left_topic/right_topic when omitted",
    )
    stereo.add_argument("--left-resolution", nargs=2, type=int, metavar=("WIDTH", "HEIGHT"))
    stereo.add_argument("--right-resolution", nargs=2, type=int, metavar=("WIDTH", "HEIGHT"))
    if not full_fisheye:
        stereo.add_argument(
            "--distortion-model",
            choices=("opencv_fisheye", "radtan", "radtan5", "radtan8"),
            default="opencv_fisheye",
        )
    stereo.add_argument(
        "--transform-direction",
        choices=(DIRECT_TRANSFORM, INVERSE_TRANSFORM),
        default=None,
        help="override transform_direction stored in the input file",
    )
    translation = stereo.add_mutually_exclusive_group()
    translation.add_argument(
        "--translation-scale",
        type=float,
        help="multiply stored T by this value to obtain Kalibr meters",
    )
    translation.add_argument(
        "--translation-unit",
        choices=("m", "cm", "mm", "um"),
        help="unit of stored T; converted to Kalibr meters",
    )

    export = commands.add_parser("export", help="Kalibr camchain -> OpenCV YAML")
    export.add_argument("--input", required=True)
    export.add_argument("--output-prefix", required=True)
    return parser
