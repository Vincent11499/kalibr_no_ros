"""sensor_msgs image conversion without cv_bridge."""

import sys
from typing import Tuple

import cv2
import numpy as np


_RAW_ENCODINGS = {
    "mono8": (np.dtype("u1"), 1),
    "8UC1": (np.dtype("u1"), 1),
    "mono16": (np.dtype("u2"), 1),
    "16UC1": (np.dtype("u2"), 1),
    "bgr8": (np.dtype("u1"), 3),
    "8UC3": (np.dtype("u1"), 3),
    "rgb8": (np.dtype("u1"), 3),
    "bgra8": (np.dtype("u1"), 4),
    "8UC4": (np.dtype("u1"), 4),
    "bayer_rggb8": (np.dtype("u1"), 1),
    "bayer_bggr8": (np.dtype("u1"), 1),
    "bayer_gbrg8": (np.dtype("u1"), 1),
    "bayer_grbg8": (np.dtype("u1"), 1),
}

_GRAY_CONVERSIONS = {
    "bgr8": cv2.COLOR_BGR2GRAY,
    "8UC3": cv2.COLOR_BGR2GRAY,
    "rgb8": cv2.COLOR_RGB2GRAY,
    "bgra8": cv2.COLOR_BGRA2GRAY,
    "8UC4": cv2.COLOR_BGRA2GRAY,
    # These intentionally mirror Kalibr's ImageDatasetReader/cv_bridge path.
    "bayer_rggb8": cv2.COLOR_BAYER_BG2GRAY,
    "bayer_bggr8": cv2.COLOR_BAYER_RG2GRAY,
    "bayer_gbrg8": cv2.COLOR_BAYER_GR2GRAY,
    "bayer_grbg8": cv2.COLOR_BAYER_GB2GRAY,
}


def decode_raw_image(message, *, grayscale: bool = True) -> np.ndarray:
    """Decode a ROS Image, respecting row stride and message endianness."""
    if message.encoding not in _RAW_ENCODINGS:
        supported = ", ".join(sorted(_RAW_ENCODINGS))
        raise RuntimeError(
            "Unsupported Image Encoding: {!r}. Supported: {}".format(
                message.encoding, supported
            )
        )
    dtype, channels = _RAW_ENCODINGS[message.encoding]
    itemsize = dtype.itemsize
    packed_step = int(message.width) * channels * itemsize
    if int(message.step) < packed_step:
        raise RuntimeError("Image row step is shorter than its encoded width")
    expected = int(message.step) * int(message.height)
    raw = np.asarray(message.data, dtype=np.uint8)
    if raw.size < expected:
        raise RuntimeError("Image payload is shorter than height * step")
    rows = raw[:expected].reshape(int(message.height), int(message.step))
    packed = np.ascontiguousarray(rows[:, :packed_step])
    byteorder = ">" if int(message.is_bigendian) else "<"
    typed = packed.view(dtype.newbyteorder(byteorder))
    if itemsize > 1 and ((sys.byteorder == "little") == bool(message.is_bigendian)):
        typed = typed.byteswap().newbyteorder()
    shape = (int(message.height), int(message.width))
    if channels > 1:
        shape += (channels,)
    image = typed.reshape(shape)
    if not grayscale:
        return np.array(image, copy=True)
    if message.encoding in ("mono16", "16UC1"):
        return (image / 256).astype(np.uint8)
    conversion = _GRAY_CONVERSIONS.get(message.encoding)
    if conversion is not None:
        return cv2.cvtColor(image, conversion)
    return np.array(image, copy=True)


def decode_compressed_image(message, *, grayscale: bool = True) -> np.ndarray:
    flags = cv2.IMREAD_UNCHANGED
    image = cv2.imdecode(np.asarray(message.data, dtype=np.uint8), flags)
    if image is None:
        raise RuntimeError("OpenCV failed to decode CompressedImage payload")
    if grayscale and image.ndim == 3 and image.shape[2] == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    elif grayscale and image.ndim == 3 and image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    return image


def encode_raw_image(image: np.ndarray, encoding: str) -> Tuple[np.ndarray, int, int]:
    if encoding not in _RAW_ENCODINGS:
        raise RuntimeError("Unsupported Image encoding for writing: {!r}".format(encoding))
    dtype, channels = _RAW_ENCODINGS[encoding]
    array = np.asarray(image)
    wanted_shape = array.shape[:2] if channels == 1 else array.shape[:2] + (channels,)
    if tuple(array.shape) != tuple(wanted_shape):
        raise ValueError("Image shape does not match encoding {!r}".format(encoding))
    native = np.ascontiguousarray(array, dtype=dtype)
    return native.view(np.uint8).reshape(-1), int(native.strides[0]), 0
