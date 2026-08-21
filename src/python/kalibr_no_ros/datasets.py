"""Drop-in replacements for Kalibr's ROS-backed dataset readers."""

from typing import Optional

import numpy as np

from kalibr_bag_io import BagReader


class _Iterator:
    def __init__(self, dataset, getter, indices=None):
        self.dataset = dataset
        self.getter = getter
        count = len(dataset.indices)
        self.indices = np.arange(count) if indices is None else indices
        self.iter = iter(self.indices)

    def __iter__(self):
        return self

    def next(self):
        return self.getter(int(next(self.iter)))

    def __next__(self):
        return self.getter(int(next(self.iter)))


class _TimeBridge:
    def __init__(self, perform_synchronization, time_factory=None, corrector=None):
        self.perform_synchronization = bool(perform_synchronization)
        if time_factory is None:
            try:
                import aslam_cv as acv
            except ImportError as error:
                raise RuntimeError(
                    "aslam_cv is required by the Kalibr adapter; use BagReader directly "
                    "when only bag access is needed"
                ) from error
            time_factory = acv.Time
        self.time_factory = time_factory
        if self.perform_synchronization and corrector is None:
            try:
                import sm
            except ImportError as error:
                raise RuntimeError("sm.DoubleTimestampCorrector is required for --synchronization") from error
            corrector = sm.DoubleTimestampCorrector()
        self.corrector = corrector

    def train(self, records):
        if self.perform_synchronization:
            for record in records:
                self.corrector.correctTimestamp(
                    record.header_timestamp_ns * 1e-9,
                    record.record_timestamp_ns * 1e-9,
                )

    def convert(self, header_timestamp_ns):
        if self.perform_synchronization:
            seconds = self.corrector.getLocalTime(header_timestamp_ns * 1e-9)
            return self.time_factory(seconds)
        seconds, nanoseconds = divmod(int(header_timestamp_ns), 1_000_000_000)
        return self.time_factory(seconds, nanoseconds)


class BagImageDatasetReader:
    """Kalibr-compatible image reader with the original constructor contract."""

    def __init__(
        self,
        bagfile,
        imagetopic,
        bag_from_to=None,
        perform_synchronization=False,
        bag_freq=None,
        *,
        reader: Optional[BagReader] = None,
        time_factory=None,
        timestamp_corrector=None,
    ):
        if imagetopic is None:
            raise RuntimeError("Please pass in a topic name referring to the image stream")
        self.bagfile = bagfile
        self.topic = imagetopic
        self.perform_synchronization = perform_synchronization
        self.bag = reader or BagReader(bagfile)
        self._dataset = self.bag.index_images(imagetopic)
        all_records = self._dataset.index
        self.index = all_records
        self._time = _TimeBridge(
            perform_synchronization, time_factory, timestamp_corrector
        )
        self._time.train(all_records)
        selected = self.bag._crop(all_records, bag_from_to)
        selected = self.bag._decimate(selected, bag_freq)
        positions = {id(record): position for position, record in enumerate(all_records)}
        self.indices = np.asarray([positions[id(record)] for record in selected], dtype=int)

    def __iter__(self):
        return self.readDataset()

    def readDataset(self):
        return _Iterator(self, self.getImage, self.indices.copy())

    def readDatasetWithTiming(self):
        """Iterate images with bag-read, deserialize, and decode timings."""
        return _Iterator(self, self.getImageWithTiming, self.indices.copy())

    def readDatasetDeferred(self):
        """Iterate raw picklable messages for detector-worker decoding."""
        return _Iterator(self, self.getDeferredImage, self.indices.copy())

    def readDatasetDeferredWithTiming(self):
        return _Iterator(
            self, self.getDeferredImageWithTiming, self.indices.copy())

    def readDatasetShuffle(self):
        # Kalibr deliberately shuffles the dataset's index array in place.
        # Preserve that observable behavior for repeated iterator creation.
        indices = self.indices
        np.random.shuffle(indices)
        return _Iterator(self, self.getImage, indices)

    def numImages(self):
        return len(self.indices)

    def getImage(self, idx):
        metadata = self.index[int(idx)]
        record = self._dataset.get_by_entry(metadata)
        return self._time.convert(record.header_timestamp_ns), record.image

    def getImageWithTiming(self, idx):
        metadata = self.index[int(idx)]
        record, timing = self._dataset.get_by_entry_with_timing(metadata)
        return (
            self._time.convert(record.header_timestamp_ns),
            record.image,
            timing,
        )

    def getDeferredImage(self, idx):
        metadata = self.index[int(idx)]
        payload = self._dataset.get_deferred_by_entry(metadata)
        return self._time.convert(metadata.header_timestamp_ns), payload

    def getDeferredImageWithTiming(self, idx):
        metadata = self.index[int(idx)]
        payload, timing = self._dataset.get_deferred_by_entry_with_timing(
            metadata)
        return self._time.convert(metadata.header_timestamp_ns), payload, timing

    def close(self):
        self._dataset.close()

    def __del__(self):
        dataset = getattr(self, "_dataset", None)
        if dataset is not None:
            dataset.close()


class BagImuDatasetReader:
    """Kalibr-compatible IMU reader with the original constructor contract."""

    def __init__(
        self,
        bagfile,
        imutopic,
        bag_from_to=None,
        perform_synchronization=False,
        *,
        reader: Optional[BagReader] = None,
        time_factory=None,
        timestamp_corrector=None,
    ):
        if imutopic is None:
            raise RuntimeError("Please pass in a topic name referring to the imu stream")
        self.bagfile = bagfile
        self.topic = imutopic
        self.perform_synchronization = perform_synchronization
        self.bag = reader or BagReader(bagfile)
        all_records = self.bag.read_imu(imutopic)
        self.index = all_records
        self._time = _TimeBridge(
            perform_synchronization, time_factory, timestamp_corrector
        )
        self._time.train(all_records)
        selected = self.bag._crop(all_records, bag_from_to)
        positions = {id(record): position for position, record in enumerate(all_records)}
        self.indices = np.asarray([positions[id(record)] for record in selected], dtype=int)

    def __iter__(self):
        return self.readDataset()

    def readDataset(self):
        return _Iterator(self, self.getMessage, self.indices.copy())

    def readDatasetShuffle(self):
        indices = self.indices
        np.random.shuffle(indices)
        return _Iterator(self, self.getMessage, indices)

    def numMessages(self):
        return len(self.indices)

    def getMessage(self, idx):
        record = self.index[int(idx)]
        return (
            self._time.convert(record.header_timestamp_ns),
            record.angular_velocity,
            record.linear_acceleration,
        )
