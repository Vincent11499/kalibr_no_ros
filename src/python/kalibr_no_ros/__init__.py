"""ROS-free Kalibr application and lazily loaded dataset adapters."""

__all__ = ["BagImageDatasetReader", "BagImuDatasetReader"]


def __getattr__(name):
    if name in __all__:
        from .datasets import BagImageDatasetReader, BagImuDatasetReader

        return {
            "BagImageDatasetReader": BagImageDatasetReader,
            "BagImuDatasetReader": BagImuDatasetReader,
        }[name]
    raise AttributeError(name)
