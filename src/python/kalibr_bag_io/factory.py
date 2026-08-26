"""Dataset backend detection and construction."""

from pathlib import Path


SUPPORTED_DATASET_FORMATS = ("auto", "ros1", "ros2", "directory")


def detect_dataset_format(dataset):
    path = Path(dataset).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(str(path))
    if path.is_file():
        return "ros1"
    if not path.is_dir():
        raise RuntimeError("unsupported dataset path: {}".format(path))
    markers = []
    if (path / "metadata.yaml").is_file():
        markers.append("ros2")
    if (path / "dataset.yaml").is_file():
        markers.append("directory")
    if len(markers) > 1:
        raise RuntimeError(
            "ambiguous dataset directory {}: contains both metadata.yaml and dataset.yaml".format(path))
    if markers:
        return markers[0]
    raise RuntimeError(
        "unsupported dataset directory {}: expected metadata.yaml or dataset.yaml".format(path))


def open_dataset(dataset, format="auto"):
    requested = str(format).lower()
    if requested not in SUPPORTED_DATASET_FORMATS:
        raise ValueError(
            "dataset format must be one of: {}".format(", ".join(SUPPORTED_DATASET_FORMATS)))
    detected = detect_dataset_format(dataset)
    if requested != "auto" and requested != detected:
        raise RuntimeError(
            "dataset format mismatch: requested {}, detected {}".format(requested, detected))
    if detected == "directory":
        from .directory import DirectoryReader
        return DirectoryReader(dataset)
    from .reader import BagReader
    return BagReader(dataset)
