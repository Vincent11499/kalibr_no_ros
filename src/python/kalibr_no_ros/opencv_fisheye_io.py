"""Nine-parameter OpenCV fisheye YAML conversion, including alpha/skew.

The Kalibr-side schema is intentionally explicit::

    camera_model: pinhole_opencv_fisheye
    intrinsics: [fu, fv, cu, cv, alpha]
    distortion_model: opencv_fisheye
    distortion_coeffs: [k1, k2, k3, k4]

``alpha`` is dimensionless.  OpenCV's pixel skew is ``K[0, 1] = fu*alpha``.
The distinct camera-model name prevents legacy Kalibr readers from silently
dropping the fifth intrinsic.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from . import opencv_io as _common


CAMERA_MODEL = "pinhole_opencv_fisheye"
DISTORTION_MODEL = "opencv_fisheye"
DIRECT_TRANSFORM = _common.DIRECT_TRANSFORM
INVERSE_TRANSFORM = _common.INVERSE_TRANSFORM


def build_argument_parser():
    return _common.build_argument_parser(full_fisheye=True)

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


def _finite(name, values):
    values = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("{} contains a non-finite value".format(name))
    return values


def _reshape_matrix(name, matrix, rows, columns):
    matrix = _finite(name, matrix)
    if matrix.shape == (rows, columns):
        return matrix.copy()
    if matrix.size == rows * columns:
        return matrix.reshape(rows, columns).copy()
    raise ValueError(
        "{} must be {}x{} or a flat {}-element sequence, got {}".format(
            name, rows, columns, rows * columns, matrix.shape
        )
    )


def _validate_k(matrix):
    matrix = _reshape_matrix("K", matrix, 3, 3)
    if not np.allclose(matrix[2], [0.0, 0.0, 1.0], rtol=0.0, atol=1e-12):
        raise ValueError("K final row must be [0, 0, 1]")
    if abs(matrix[1, 0]) > 1e-12:
        raise ValueError("OpenCV fisheye requires K[1,0]=0")
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("OpenCV fisheye focal lengths must be positive")
    return matrix.copy()


def _validate_d(coefficients):
    coefficients = _finite("D", coefficients).reshape(-1)
    if coefficients.size != 4:
        raise ValueError(
            "OpenCV fisheye D must be [k1,k2,k3,k4], got {} coefficients".format(
                coefficients.size
            )
        )
    return coefficients


def _optional_real(storage, names):
    node = _common._node(storage, names)
    if node is None:
        return None
    value = float(node.real())
    if not np.isfinite(value):
        raise ValueError("{} is not finite".format(names[0]))
    return value


def _extract_alpha(storage, K, prefix="", tolerance=1e-10):
    """Resolve alpha from K, alpha*, and pixel-skew fields without loss."""

    if prefix:
        alpha_names = ("alpha{}".format(prefix), "alpha")
        skew_names = ("skew{}".format(prefix), "skew")
    else:
        alpha_names = ("alpha",)
        skew_names = ("skew",)
    explicit_alpha = _optional_real(storage, alpha_names)
    explicit_skew = _optional_real(storage, skew_names)
    matrix_skew = float(K[0, 1])
    matrix_alpha = matrix_skew / float(K[0, 0])

    candidates = []
    if abs(matrix_skew) > tolerance:
        candidates.append(("K[0,1]", matrix_alpha))
    if explicit_alpha is not None:
        candidates.append((alpha_names[0], explicit_alpha))
    if explicit_skew is not None:
        candidates.append((skew_names[0], explicit_skew / float(K[0, 0])))
    if not candidates:
        return 0.0

    alpha = candidates[0][1]
    for name, candidate in candidates[1:]:
        scale = max(1.0, abs(alpha), abs(candidate))
        if abs(candidate - alpha) > tolerance * scale:
            raise ValueError(
                "inconsistent OpenCV fisheye skew: {} implies alpha={}, "
                "but {} implies alpha={}".format(
                    candidates[0][0], alpha, name, candidate
                )
            )
    return float(alpha)


def _validate_fisheye_declaration(storage):
    """Reject an explicitly declared non-fisheye distortion model."""

    for field in ("kalibr_distortion_model", "distortion_model"):
        value = _common._read_string(storage, (field,), "")
        if not value:
            continue
        normalized = value.strip().lower().replace("-", "_")
        if normalized not in ("opencv_fisheye", "fisheye", "equidistant"):
            raise ValueError(
                "{}={!r} is not an OpenCV fisheye declaration".format(
                    field, value
                )
            )

    # camera_model is secondary: "pinhole" can legitimately accompany an
    # explicit fisheye distortion_model, while plumb_bob cannot.
    camera_model = _common._read_string(storage, ("camera_model",), "")
    if camera_model:
        normalized = camera_model.strip().lower().replace("-", "_")
        if normalized not in (
            "fisheye",
            "opencv_fisheye",
            "equidistant",
            "pinhole",
            CAMERA_MODEL,
        ):
            raise ValueError(
                "camera_model={!r} is not compatible with OpenCV fisheye".format(
                    camera_model
                )
            )


def _read_stereo_transform(storage):
    rotation_node = _common._node(storage, ("R", "rotation_matrix"))
    translation_node = _common._node(
        storage, ("T", "translation", "translation_vector")
    )
    if rotation_node is not None and translation_node is not None:
        rotation = _reshape_matrix(
            "R",
            _common._read_matrix(storage, ("R", "rotation_matrix")),
            3,
            3,
        )
        translation = _finite(
            "T",
            _common._read_matrix(
                storage, ("T", "translation", "translation_vector")
            ),
        ).reshape(-1)
        if translation.size != 3:
            raise ValueError("T must contain exactly three values")
        return rotation, translation

    transform_node = _common._node(
        storage, ("RT", "T_cn_cnm1", "transform")
    )
    if transform_node is None:
        raise ValueError("stereo YAML requires R/T or a 4x4 RT/T_cn_cnm1")
    transform = _reshape_matrix(
        "RT",
        _common._read_matrix(storage, ("RT", "T_cn_cnm1", "transform")),
        4,
        4,
    )
    if not np.allclose(
        transform[3], [0.0, 0.0, 0.0, 1.0], rtol=0.0, atol=1e-12
    ):
        raise ValueError("stereo 4x4 transform has an invalid final row")
    return transform[:3, :3], transform[:3, 3]


def _unit_scale(unit):
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
    """Return the multiplier from stored T units to Kalibr meters.

    OpenCV itself does not prescribe a translation unit.  With no metadata or
    override, values are preserved (scale 1).  This avoids guessing; callers
    importing millimetres can explicitly request ``translation_unit="mm"``.
    """

    if override_scale is not None and override_unit is not None:
        raise ValueError("specify translation_scale or translation_unit, not both")
    if override_scale is not None:
        scale = float(override_scale)
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError("translation_scale must be finite and positive")
        return scale
    if override_unit is not None:
        return _unit_scale(override_unit)

    stored_scale = _optional_real(
        storage, ("translation_scale", "T_scale")
    )
    stored_unit = _common._read_string(
        storage, ("translation_unit", "T_unit", "baseline_unit"), ""
    )
    unit_scale = _unit_scale(stored_unit) if stored_unit else None
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


@dataclass
class OpenCvFisheyeCamera:
    K: np.ndarray
    D: np.ndarray
    resolution: tuple
    alpha: float = None
    name: str = "camera"
    topic: str = ""

    def __post_init__(self):
        self.K = _validate_k(self.K)
        self.D = _validate_d(self.D)
        self.resolution = _common._validate_resolution(self.resolution)
        if self.alpha is None:
            self.alpha = float(self.K[0, 1] / self.K[0, 0])
        self.alpha = float(self.alpha)
        if not np.isfinite(self.alpha):
            raise ValueError("alpha is not finite")
        expected_skew = self.K[0, 0] * self.alpha
        if not np.isclose(
            self.K[0, 1], expected_skew, rtol=1e-10, atol=1e-10
        ):
            raise ValueError(
                "K[0,1]={} conflicts with fu*alpha={}".format(
                    self.K[0, 1], expected_skew
                )
            )
        self.K[0, 1] = expected_skew
        self.name = str(self.name)
        self.topic = str(self.topic)

    @property
    def intrinsics(self):
        return [
            float(self.K[0, 0]),
            float(self.K[1, 1]),
            float(self.K[0, 2]),
            float(self.K[1, 2]),
            float(self.alpha),
        ]


@dataclass
class OpenCvFisheyeStereo:
    left: OpenCvFisheyeCamera
    right: OpenCvFisheyeCamera
    R: np.ndarray
    T: np.ndarray

    def __post_init__(self):
        self.R = _common._validate_rotation(self.R)
        self.T = _common._validate_translation(self.T).reshape(3)

    @property
    def transform(self):
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = self.R
        transform[:3, 3] = self.T
        return transform


def read_opencv_camera(
    path, *, prefix="", resolution=None, topic="", name=None
):
    storage = _common._open_storage(path, cv2.FILE_STORAGE_READ)
    try:
        _validate_fisheye_declaration(storage)
        if prefix:
            K = _common._read_matrix(
                storage, ("K{}".format(prefix), "M{}".format(prefix))
            )
            D = _common._read_matrix(storage, ("D{}".format(prefix),))
            camera_name = _common._read_string(
                storage,
                (
                    ("left_camera", "camera_name1")
                    if prefix == "1"
                    else ("right_camera", "camera_name2")
                ),
                name or "cam{}".format(int(prefix) - 1),
            )
            camera_topic = topic or _common._read_string(
                storage,
                (
                    ("left_topic", "ros_topic1")
                    if prefix == "1"
                    else ("right_topic", "ros_topic2")
                ),
            )
        else:
            K = _common._read_matrix(storage, ("K", "camera_matrix"))
            D = _common._read_matrix(
                storage, ("D", "distortion_coefficients")
            )
            camera_name = name or _common._read_string(
                storage, ("camera_name",), "camera"
            )
            camera_topic = topic or _common._read_string(
                storage, ("ros_topic", "topic")
            )
        K = _validate_k(K)
        alpha = _extract_alpha(storage, K, prefix)
        K[0, 1] = K[0, 0] * alpha
        return OpenCvFisheyeCamera(
            K,
            D,
            _common._read_resolution(storage, prefix, resolution),
            alpha,
            camera_name,
            camera_topic,
        )
    finally:
        storage.release()


def read_opencv_stereo(
    stereo_path,
    *,
    left_path=None,
    right_path=None,
    resolutions=None,
    topics=(None, None),
    transform_direction=None,
    translation_scale=None,
    translation_unit=None
):
    if resolutions is None:
        resolutions = (None, None)
    if len(resolutions) != 2 or len(topics) != 2:
        raise ValueError("stereo conversion requires two resolutions and topics")
    storage = _common._open_storage(stereo_path, cv2.FILE_STORAGE_READ)
    try:
        _validate_fisheye_declaration(storage)
        embedded = _common._node(storage, ("K1", "M1")) is not None
        rotation, translation = _read_stereo_transform(storage)
        declared_direction = _common._read_string(
            storage, ("transform_direction",), ""
        )
        scale_to_meters = _translation_scale_to_meters(
            storage, translation_scale, translation_unit
        )
    finally:
        storage.release()

    if embedded:
        left = read_opencv_camera(
            stereo_path,
            prefix="1",
            resolution=resolutions[0],
            topic=topics[0] or "",
            name="cam0",
        )
        right = read_opencv_camera(
            stereo_path,
            prefix="2",
            resolution=resolutions[1],
            topic=topics[1] or "",
            name="cam1",
        )
    else:
        if left_path is None or right_path is None:
            raise ValueError(
                "stereo YAML without K1/K2 requires left_path and right_path"
            )
        left = read_opencv_camera(
            left_path,
            resolution=resolutions[0],
            topic=topics[0] or "",
            name="cam0",
        )
        right = read_opencv_camera(
            right_path,
            resolution=resolutions[1],
            topic=topics[1] or "",
            name="cam1",
        )

    translation = np.asarray(translation, dtype=np.float64) * scale_to_meters
    direction = transform_direction or declared_direction or DIRECT_TRANSFORM
    if direction == INVERSE_TRANSFORM:
        rotation, translation = _common.invert_stereo_transform(
            rotation, translation
        )
    elif direction != DIRECT_TRANSFORM:
        raise ValueError(
            "transform_direction must be {!r} or {!r}, got {!r}".format(
                DIRECT_TRANSFORM, INVERSE_TRANSFORM, direction
            )
        )
    return OpenCvFisheyeStereo(left, right, rotation, translation)


def _camera_to_entry(camera, index, overlaps):
    return {
        "camera_model": CAMERA_MODEL,
        "intrinsics": camera.intrinsics,
        "distortion_model": DISTORTION_MODEL,
        "distortion_coeffs": camera.D.astype(float).tolist(),
        "resolution": [int(camera.resolution[0]), int(camera.resolution[1])],
        "rostopic": camera.topic or "/cam{}/image_raw".format(index),
        "cam_overlaps": list(overlaps),
    }


def opencv_mono_to_camchain(camera):
    return {"cam0": _camera_to_entry(camera, 0, [])}


def opencv_stereo_to_camchain(stereo):
    cam0 = _camera_to_entry(stereo.left, 0, [1])
    cam1 = _camera_to_entry(stereo.right, 1, [0])
    cam1["T_cn_cnm1"] = stereo.transform.tolist()
    return {"cam0": cam0, "cam1": cam1}


def write_kalibr_camchain(data, output_path):
    output_path = Path(output_path)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, default_flow_style=None, sort_keys=False)
    return output_path


def import_opencv_mono(input_path, output_path, *, resolution=None, topic=None):
    camera = read_opencv_camera(
        input_path, resolution=resolution, topic=topic or "", name="cam0"
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
    transform_direction=None,
    translation_scale=None,
    translation_unit=None
):
    stereo = read_opencv_stereo(
        stereo_path,
        left_path=left_path,
        right_path=right_path,
        resolutions=resolutions,
        topics=topics,
        transform_direction=transform_direction,
        translation_scale=translation_scale,
        translation_unit=translation_unit,
    )
    return write_kalibr_camchain(opencv_stereo_to_camchain(stereo), output_path)


def _camera_from_entry(entry, index):
    model = entry.get("camera_model")
    intrinsics = _finite("intrinsics", entry.get("intrinsics", [])).reshape(-1)
    distortion_model = entry.get("distortion_model")
    if model == CAMERA_MODEL:
        if intrinsics.size != 5:
            raise ValueError(
                "cam{} {} requires [fu,fv,cu,cv,alpha]".format(
                    index, CAMERA_MODEL
                )
            )
        fu, fv, cu, cv, alpha = intrinsics
    elif model == "pinhole" and distortion_model in (
        "opencv_fisheye",
        "fisheye",
        "equidistant",
    ):
        # Native equidistant has no alpha; OpenCV uses the same D ordering.
        if intrinsics.size != 4:
            raise ValueError("pinhole equidistant intrinsics require 4 values")
        fu, fv, cu, cv = intrinsics
        alpha = 0.0
    else:
        raise ValueError(
            "cam{} is not an OpenCV fisheye-compatible camera (model={!r})".format(
                index, model
            )
        )
    if distortion_model not in ("opencv_fisheye", "fisheye", "equidistant"):
        raise ValueError("cam{} does not use OpenCV fisheye D".format(index))
    K = np.array(
        [
            [fu, fu * alpha, cu],
            [0.0, fv, cv],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return OpenCvFisheyeCamera(
        K,
        entry.get("distortion_coeffs", []),
        tuple(entry.get("resolution", ())),
        alpha,
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
        cameras.append(_camera_from_entry(entry, index))
        if index:
            transform = _finite(
                "T_cn_cnm1", entry.get("T_cn_cnm1", [])
            )
            if transform.shape != (4, 4):
                raise ValueError("cam{} requires a 4x4 T_cn_cnm1".format(index))
            if not np.allclose(
                transform[3], [0.0, 0.0, 0.0, 1.0], rtol=0.0, atol=1e-12
            ):
                raise ValueError("cam{} has invalid transform final row".format(index))
            transforms.append(
                (
                    _common._validate_rotation(transform[:3, :3]),
                    _common._validate_translation(transform[:3, 3]).reshape(3),
                )
            )
        index += 1
    expected = {"cam{}".format(i) for i in range(len(cameras))}
    if set(data) != expected:
        raise ValueError("camchain keys must be contiguous camera entries")
    return cameras, transforms


def _write_values(path, values):
    storage = _common._open_storage(path, cv2.FILE_STORAGE_WRITE)
    try:
        for name, value in values:
            storage.write(name, value)
    finally:
        storage.release()


def write_opencv_camera(camera, path):
    _write_values(
        path,
        [
            ("camera_name", camera.name),
            ("ros_topic", camera.topic),
            ("image_width", int(camera.resolution[0])),
            ("image_height", int(camera.resolution[1])),
            ("camera_model", "fisheye"),
            ("distortion_model", "fisheye"),
            ("kalibr_camera_model", CAMERA_MODEL),
            ("kalibr_distortion_model", DISTORTION_MODEL),
            ("alpha", float(camera.alpha)),
            ("skew", float(camera.K[0, 1])),
            ("K", camera.K),
            ("D", camera.D.reshape(-1, 1)),
            ("camera_matrix", camera.K),
            ("distortion_coefficients", camera.D.reshape(-1, 1)),
        ],
    )
    return Path(path)


def _skew_matrix(vector):
    x, y, z = np.asarray(vector, dtype=np.float64).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def write_opencv_stereo(stereo, path):
    essential = _skew_matrix(stereo.T).dot(stereo.R)
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
            ("camera_model", "fisheye"),
            ("distortion_model", "fisheye"),
            ("kalibr_camera_model", CAMERA_MODEL),
            ("kalibr_distortion_model", DISTORTION_MODEL),
            ("transform_direction", DIRECT_TRANSFORM),
            ("translation_unit", "m"),
            ("translation_scale", 1.0),
            ("alpha1", float(stereo.left.alpha)),
            ("alpha2", float(stereo.right.alpha)),
            ("skew1", float(stereo.left.K[0, 1])),
            ("skew2", float(stereo.right.K[0, 1])),
            ("K1", stereo.left.K),
            ("D1", stereo.left.D.reshape(-1, 1)),
            ("K2", stereo.right.K),
            ("D2", stereo.right.D.reshape(-1, 1)),
            ("R", stereo.R),
            ("T", stereo.T.reshape(3, 1)),
            ("E", essential),
            ("F", fundamental),
        ],
    )
    return Path(path)


def export_kalibr_camchain(camchain_path, output_prefix):
    cameras, transforms = read_kalibr_camchain(camchain_path)
    prefix = str(output_prefix)
    if prefix.endswith((".yaml", ".yml")):
        prefix = str(Path(prefix).with_suffix(""))
    outputs = []
    for index, camera in enumerate(cameras):
        output = Path("{}-cam{}-opencv.yaml".format(prefix, index))
        outputs.append(write_opencv_camera(camera, output))
    for index, (rotation, translation) in enumerate(transforms):
        stereo = OpenCvFisheyeStereo(
            cameras[index], cameras[index + 1], rotation, translation
        )
        output = Path(
            "{}-cam{}-cam{}-opencv-stereo.yaml".format(
                prefix, index, index + 1
            )
        )
        outputs.append(write_opencv_stereo(stereo, output))
    return outputs
