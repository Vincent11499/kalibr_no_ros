from __future__ import print_function

import copy
import multiprocessing
import os
import pickle
import signal
import sys
import time
import traceback

import cv2
import numpy as np
import sm
from kalibr_no_ros import artifacts as run_artifacts

try:
    import queue
except ImportError:
    import Queue as queue  # python 2.x

try:
    import kalibr_runtime as _native_runtime
except ImportError:
    # The overlay can still be imported in an upstream-only environment.  In
    # a native install the companion package is always staged next to Kalibr.
    _native_runtime = None


def _timingEnabled():
    """Return whether structured extraction timing is active.

    Newer native-runtime packages expose this as a public query.  The private
    module fallback keeps this overlay usable while an older build tree is
    being reconfigured; it can be removed once all staged runtimes provide the
    public helper.
    """
    if _native_runtime is None:
        return False
    query = getattr(_native_runtime, "timing_enabled", None)
    if query is not None:
        return bool(query())
    runtimeModule = sys.modules.get("kalibr_runtime.runtime")
    return (
        runtimeModule is not None
        and getattr(runtimeModule, "_recorder", None) is not None
    )


def multicoreExtractionWrapper(detector, taskq, resultq, clearImages,
                               noTransformation, collectTiming=False,
                               opencvThreads=1):
    """Consume tasks and report one result, including timing, per image."""
    # OpenCV may otherwise create its own thread pool in every process.  One
    # native thread per worker avoids processes x OpenCV threads oversubscription.
    cv2.setNumThreads(int(opencvThreads))

    while True:
        encodedTask = taskq.get()
        if encodedTask is None:
            return
        try:
            idx, stamp, image = pickle.loads(encodedTask)
        except Exception as error:
            # Task serialization is normally validated by the parent before
            # enqueueing. Keep a synchronous diagnostic if bytes are damaged.
            encodedError = pickle.dumps((
                "error",
                -1,
                {
                    "message": "invalid serialized extraction task: {!r}".format(error),
                    "traceback": traceback.format_exc(),
                },
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ), protocol=pickle.HIGHEST_PROTOCOL)
            resultq.put(encodedError)
            return
        detect_wall_start = None
        detect_cpu_start = None
        bagReadWall = 0.0
        bagReadCpu = 0.0
        deserializeWall = 0.0
        deserializeCpu = 0.0
        decodeWall = 0.0
        decodeCpu = 0.0
        try:
            deferredDecoder = getattr(image, "decode_for_kalibr", None)
            if deferredDecoder is not None:
                decodedImage, decodeTiming = deferredDecoder(collectTiming)
                bagReadWall = decodeTiming.get(
                    "bag_read_wall_seconds", 0.0)
                bagReadCpu = decodeTiming.get(
                    "bag_read_cpu_seconds", 0.0)
                deserializeWall = decodeTiming.get(
                    "deserialize_wall_seconds", 0.0)
                deserializeCpu = decodeTiming.get(
                    "deserialize_cpu_seconds", 0.0)
                decodeWall = decodeTiming.get("decode_wall_seconds", 0.0)
                decodeCpu = decodeTiming.get("decode_cpu_seconds", 0.0)
            else:
                decodedImage = image
                deserializeWall = 0.0
                deserializeCpu = 0.0
                decodeWall = 0.0
                decodeCpu = 0.0
            if collectTiming:
                detect_wall_start = time.perf_counter()
                detect_cpu_start = time.process_time()
            else:
                detect_wall_start = None
                detect_cpu_start = None
            converted_image = np.array(decodedImage)
            if noTransformation:
                success, obs = detector.findTargetNoTransformation(
                    stamp, converted_image)
            else:
                success, obs = detector.findTarget(stamp, converted_image)

            if clearImages and obs is not None:
                obs.clearImage()
            # multiprocessing.Queue otherwise serializes in a background
            # feeder thread. A non-picklable observation would be dropped
            # there while this worker remained alive, leaving the parent in a
            # permanent wait. Serialize synchronously before queueing bytes.
            detectWall = (
                time.perf_counter() - detect_wall_start
                if collectTiming and detect_wall_start is not None else 0.0
            )
            detectCpu = (
                time.process_time() - detect_cpu_start
                if collectTiming and detect_cpu_start is not None else 0.0
            )
            encodedResult = pickle.dumps((
                "result",
                idx,
                obs if success else None,
                detectWall,
                detectCpu,
                bagReadWall,
                bagReadCpu,
                deserializeWall,
                deserializeCpu,
                decodeWall,
                decodeCpu,
            ), protocol=pickle.HIGHEST_PROTOCOL)
            resultq.put(encodedResult)
        except Exception as error:
            # Include the failing index and traceback.  The parent can stop all
            # workers immediately instead of waiting forever for a lost result.
            detectWall = (
                time.perf_counter() - detect_wall_start
                if collectTiming and detect_wall_start is not None else 0.0
            )
            detectCpu = (
                time.process_time() - detect_cpu_start
                if collectTiming and detect_cpu_start is not None else 0.0
            )
            resultq.put(pickle.dumps((
                "error",
                idx,
                {
                    "message": repr(error),
                    "traceback": traceback.format_exc(),
                },
                detectWall,
                detectCpu,
                locals().get("bagReadWall", 0.0),
                locals().get("bagReadCpu", 0.0),
                locals().get("deserializeWall", 0.0),
                locals().get("deserializeCpu", 0.0),
                locals().get("decodeWall", 0.0),
                locals().get("decodeCpu", 0.0),
            ), protocol=pickle.HIGHEST_PROTOCOL))


def _stopProcesses(processes, timeout=5.0):
    for process in processes:
        if process.is_alive():
            process.terminate()
    deadline = time.monotonic() + timeout
    for process in processes:
        process.join(max(0.0, deadline - time.monotonic()))
    stubborn = [process for process in processes if process.is_alive()]
    for process in stubborn:
        if hasattr(process, "kill"):
            process.kill()
        elif process.pid is not None:
            # Python 2's multiprocessing.Process has no kill().
            try:
                os.kill(process.pid, signal.SIGKILL)
            except OSError:
                pass
    deadline = time.monotonic() + timeout
    for process in stubborn:
        process.join(max(0.0, deadline - time.monotonic()))
    survivors = [process for process in stubborn if process.is_alive()]
    if survivors:
        details = ", ".join(
            "pid={0}".format(process.pid) for process in survivors
        )
        raise RuntimeError(
            "Corner extraction workers could not be stopped ({0})".format(
                details))


def _assertWorkersRunning(processes):
    failed = [
        process for process in processes
        if process.exitcode is not None
    ]
    if failed:
        details = ", ".join(
            "pid={0} exitcode={1}".format(process.pid, process.exitcode)
            for process in failed
        )
        raise RuntimeError(
            "Corner extraction worker exited unexpectedly ({0})".format(details))


def _putWithWorkerHealth(queueObject, item, processes, onWait=None):
    while True:
        try:
            queueObject.put(item, timeout=0.25)
            return
        except queue.Full:
            if onWait is not None:
                onWait()
            _assertWorkersRunning(processes)


def _getWithWorkerHealth(queueObject, processes, onWait=None):
    while True:
        try:
            return queueObject.get(timeout=0.25)
        except queue.Empty:
            if onWait is not None:
                onWait()
            _assertWorkersRunning(processes)


def _validateWorkerResult(resultStatus, idx, payload, submitted,
                          observationsByIndex):
    """Validate status before index so worker diagnostics are not obscured."""
    if resultStatus == "error":
        message = payload.get("message", repr(payload))
        trace = payload.get("traceback", "")
        raise RuntimeError(
            "Corner extraction failed for image {0}: {1}\n{2}".format(
                idx, message, trace))
    if resultStatus != "result":
        raise RuntimeError(
            "Corner extraction worker returned unknown status {0}".format(
                resultStatus))
    if idx < 0 or idx >= submitted or idx in observationsByIndex:
        raise RuntimeError(
            "Corner extraction worker returned invalid index {0}".format(idx))


def _joinProcesses(processes, timeout=5.0):
    deadline = time.monotonic() + timeout
    for process in processes:
        process.join(max(0.0, deadline - time.monotonic()))
    remaining = [process for process in processes if process.is_alive()]
    if remaining:
        raise RuntimeError(
            "Corner extraction workers did not stop after completing tasks")


def _closeQueue(queueObject, cancelPending=False):
    if cancelPending:
        try:
            queueObject.cancel_join_thread()
        except (AttributeError, ValueError):
            pass
    try:
        queueObject.close()
    except (AttributeError, ValueError):
        pass
    if not cancelPending:
        try:
            queueObject.join_thread()
        except (AttributeError, AssertionError, ValueError):
            pass


def _readProcessMemory(pid):
    """Read current RSS/PSS for one Linux process, in bytes."""
    rss = None
    pss = None
    try:
        with open("/proc/{0}/status".format(pid), "r") as statusFile:
            for line in statusFile:
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1]) * 1024
                    break
    except (IOError, OSError, ValueError, IndexError):
        pass
    try:
        with open("/proc/{0}/smaps_rollup".format(pid), "r") as smapsFile:
            for line in smapsFile:
                if line.startswith("Pss:"):
                    pss = int(line.split()[1]) * 1024
                    break
    except (IOError, OSError, ValueError, IndexError):
        pass
    return {"rss": rss, "pss": pss}


class _ProcessTreeMemoryTracker(object):
    """Sample aggregate memory of the parent and live detector workers.

    PSS is additive across processes because shared pages are proportionally
    charged.  The peak is explicitly a sampled high-water mark rather than the
    process-lifetime ``ru_maxrss`` value previously reported as a stage peak.
    """
    def __init__(self, multithreading, sampleInterval=0.25):
        self.multithreading = bool(multithreading)
        self.sampleInterval = float(sampleInterval)
        self.first = None
        self.last = None
        self.rssPeak = None
        self.pssPeak = None
        self.samples = 0
        self.incompleteRssSamples = 0
        self.incompletePssSamples = 0
        self.maxProcesses = 0
        self.lastSampleTime = None

    @staticmethod
    def _livePids(processes):
        pids = [os.getpid()]
        for process in processes or ():
            try:
                alive = process.is_alive()
            except (AssertionError, ValueError):
                alive = False
            if alive and process.pid is not None:
                pids.append(process.pid)
        return list(dict.fromkeys(pids))

    def sample(self, processes=(), force=False):
        now = time.monotonic()
        if (not force and self.lastSampleTime is not None
                and now - self.lastSampleTime < self.sampleInterval):
            return self.last
        self.lastSampleTime = now

        pids = self._livePids(processes)
        measurements = [_readProcessMemory(pid) for pid in pids]
        rssValues = [item["rss"] for item in measurements]
        pssValues = [item["pss"] for item in measurements]
        rssComplete = all(value is not None for value in rssValues)
        pssComplete = all(value is not None for value in pssValues)
        rss = sum(rssValues) if rssComplete else None
        pss = sum(pssValues) if pssComplete else None
        snapshot = {
            "rss": rss,
            "pss": pss,
            "processes": len(pids),
        }

        if self.first is None:
            self.first = snapshot
        self.last = snapshot
        self.samples += 1
        self.maxProcesses = max(self.maxProcesses, len(pids))
        if rss is None:
            self.incompleteRssSamples += 1
        else:
            self.rssPeak = rss if self.rssPeak is None else max(self.rssPeak, rss)
        if pss is None:
            self.incompletePssSamples += 1
        else:
            self.pssPeak = pss if self.pssPeak is None else max(self.pssPeak, pss)
        return snapshot

    @staticmethod
    def _delta(first, last, key):
        if first is None or last is None:
            return None
        if first[key] is None or last[key] is None:
            return None
        return last[key] - first[key]

    def fields(self):
        scope = (
            "parent_and_detector_workers"
            if self.multithreading else "main_process"
        )
        common = {
            "scope": scope,
            "peak_semantics": "maximum_of_periodic_process_tree_samples",
            "sample_interval_seconds": self.sampleInterval,
            "samples": self.samples,
            "max_processes": self.maxProcesses,
        }
        rss = dict(common)
        rss.update({
            "start": None if self.first is None else self.first["rss"],
            "end": None if self.last is None else self.last["rss"],
            "delta": self._delta(self.first, self.last, "rss"),
            "peak": self.rssPeak,
            "incomplete_samples": self.incompleteRssSamples,
        })
        pss = dict(common)
        pss.update({
            "start": None if self.first is None else self.first["pss"],
            "end": None if self.last is None else self.last["pss"],
            "delta": self._delta(self.first, self.last, "pss"),
            "peak": self.pssPeak,
            "incomplete_samples": self.incompletePssSamples,
        })
        return rss, pss


def _recordExtractionTiming(dataset, multithreading, numProcesses, numImages,
                            submitted, succeeded, bagReadWall, bagReadCpu,
                            deserializeWall, deserializeCpu,
                            decodeWall, decodeCpu,
                            detectWall, detectCpu, totalWallStart,
                            totalCpuStart, memoryTracker, status,
                            detailedImageTiming, deferredImageLoading,
                            error=None):
    if not _timingEnabled():
        return

    rssFields, pssFields = memoryTracker.fields()
    totalCpu = time.process_time() - totalCpuStart
    if multithreading:
        # process_time() does not include worker processes.  Their reported CPU
        # service time is additive and gives a useful total CPU cost.
        totalCpu += detectCpu
    _native_runtime.record_corner_extraction(
        topic=getattr(dataset, "topic", None),
        mode="multiprocessing" if multithreading else "serial",
        worker_processes=numProcesses if multithreading else 0,
        images_reported=numImages,
        images_submitted=submitted,
        observations_succeeded=succeeded,
        pipeline_capacity=(
            (_native_runtime.detector_inflight_per_worker()
             if _native_runtime is not None else 2) * numProcesses
            if multithreading else 1),
        image_phase_detail_available=detailedImageTiming,
        image_decode_location=(
            "detector_workers" if deferredImageLoading else "parent_process"),
        bag_read_wall_seconds=bagReadWall,
        bag_read_cpu_seconds=bagReadCpu,
        deserialize_wall_seconds=deserializeWall,
        deserialize_cpu_seconds=deserializeCpu,
        decode_wall_seconds=decodeWall,
        decode_cpu_seconds=decodeCpu,
        detect_wall_seconds=detectWall,
        detect_cpu_seconds=detectCpu,
        detect_aggregation="sum_over_images",
        total_wall_seconds=time.perf_counter() - totalWallStart,
        total_cpu_seconds=totalCpu,
        rss_bytes=rssFields,
        pss_bytes=pssFields,
        status=status,
        error=error,
    )


def extractCornersFromDataset(dataset, detector, multithreading=False,
                              numProcesses=None, clearImages=True,
                              noTransformation=False):
    print("Extracting calibration target corners")
    targetObservations = []
    numImages = dataset.numImages()
    artifact_context = run_artifacts.current_context()
    if artifact_context is not None:
        artifact_context.register_dataset(dataset)

    iProgress = sm.Progress2(numImages)
    iProgress.sample()

    timingActive = _timingEnabled()
    totalWallStart = time.perf_counter() if timingActive else None
    totalCpuStart = time.process_time() if timingActive else None
    memoryTracker = (
        _ProcessTreeMemoryTracker(
            multithreading,
            sampleInterval=(
                _native_runtime.profiling_memory_sample_interval_s()
                if _native_runtime is not None else 0.25))
        if timingActive else None
    )
    if memoryTracker is not None:
        memoryTracker.sample(force=True)
    decodeWall = 0.0
    decodeCpu = 0.0
    bagReadWall = 0.0
    bagReadCpu = 0.0
    deserializeWall = 0.0
    deserializeCpu = 0.0
    detectWall = 0.0
    detectCpu = 0.0
    submitted = 0
    deferredImageLoading = (
        multithreading and hasattr(dataset, "readDatasetDeferred")
    )
    detailedImageTiming = (
        timingActive and (
            hasattr(dataset, "readDatasetWithTiming")
            or deferredImageLoading
        )
    )

    if multithreading:
        configuredProcesses = (
            _native_runtime.detector_processes()
            if _native_runtime is not None else None
        )
        if numProcesses is None and configuredProcesses is not None:
            numProcesses = configuredProcesses
        if not numProcesses:
            numProcesses = max(1, multiprocessing.cpu_count() - 1)
        numProcesses = max(1, min(int(numProcesses), max(1, numImages)))

        # Both queues and the number of in-flight tasks are bounded.  The
        # producer alternates submission with result draining, so a fast reader
        # cannot decode the complete dataset into memory ahead of detection.
        inflightPerWorker = (
            _native_runtime.detector_inflight_per_worker()
            if _native_runtime is not None else 2)
        opencvThreads = (
            _native_runtime.detector_opencv_threads()
            if _native_runtime is not None else 1)
        maxPending = max(1, int(inflightPerWorker) * numProcesses)
        taskq = multiprocessing.Queue(maxsize=maxPending)
        resultq = multiprocessing.Queue(maxsize=maxPending)
        processes = []
        completedNormally = False
        try:
            for unused_idx in range(numProcesses):
                detectorCopy = copy.copy(detector)
                process = multiprocessing.Process(
                    target=multicoreExtractionWrapper,
                    args=(detectorCopy, taskq, resultq, clearImages,
                          noTransformation, timingActive, opencvThreads))
                process.start()
                processes.append(process)

            if memoryTracker is not None:
                memoryTracker.sample(processes, force=True)

            sampleMemory = (
                (lambda: memoryTracker.sample(processes))
                if memoryTracker is not None else None
            )

            if deferredImageLoading:
                datasetIterator = iter(
                    dataset.readDatasetDeferredWithTiming()
                    if timingActive else dataset.readDatasetDeferred())
            else:
                datasetIterator = iter(
                    dataset.readDatasetWithTiming()
                    if detailedImageTiming else dataset.readDataset())
            exhausted = False
            inFlight = 0
            observationsByIndex = {}

            while not exhausted or inFlight:
                while not exhausted and inFlight < maxPending:
                    if timingActive and not detailedImageTiming:
                        decodeWallStart = time.perf_counter()
                        decodeCpuStart = time.process_time()
                    try:
                        item = next(datasetIterator)
                    except StopIteration:
                        exhausted = True
                        break
                    finally:
                        if timingActive and not detailedImageTiming:
                            decodeWall += time.perf_counter() - decodeWallStart
                            decodeCpu += time.process_time() - decodeCpuStart

                    if detailedImageTiming:
                        timestamp, image, imageTiming = item
                        bagReadWall += imageTiming.get(
                            "bag_read_wall_seconds", 0.0)
                        bagReadCpu += imageTiming.get(
                            "bag_read_cpu_seconds", 0.0)
                        deserializeWall += imageTiming.get(
                            "deserialize_wall_seconds", 0.0)
                        deserializeCpu += imageTiming.get(
                            "deserialize_cpu_seconds", 0.0)
                        decodeWall += imageTiming.get(
                            "decode_wall_seconds", 0.0)
                        decodeCpu += imageTiming.get(
                            "decode_cpu_seconds", 0.0)
                    else:
                        timestamp, image = item

                    if submitted >= numImages:
                        raise RuntimeError(
                            "Dataset reported {0} images but yielded more".format(
                                numImages))
                    # Validate serialization in the producer process. Queue's
                    # own asynchronous pickler cannot report failures to us.
                    encodedTask = pickle.dumps(
                        (submitted, timestamp, image),
                        protocol=pickle.HIGHEST_PROTOCOL,
                    )
                    _putWithWorkerHealth(
                        taskq, encodedTask, processes, onWait=sampleMemory)
                    submitted += 1
                    inFlight += 1

                if not inFlight:
                    continue

                encodedResult = _getWithWorkerHealth(
                    resultq, processes, onWait=sampleMemory)
                try:
                    result = pickle.loads(encodedResult)
                except Exception as error:
                    raise RuntimeError(
                        "Corner extraction worker returned an invalid "
                        "serialized result: {!r}".format(error)
                    )
                if not isinstance(result, tuple) or len(result) != 11:
                    raise RuntimeError(
                        "Corner extraction worker returned a malformed result")
                (resultStatus, idx, payload, itemDetectWall, itemDetectCpu,
                 itemBagReadWall, itemBagReadCpu,
                 itemDeserializeWall, itemDeserializeCpu, itemDecodeWall,
                 itemDecodeCpu) = result
                inFlight -= 1
                detectWall += itemDetectWall
                detectCpu += itemDetectCpu
                bagReadWall += itemBagReadWall
                bagReadCpu += itemBagReadCpu
                deserializeWall += itemDeserializeWall
                deserializeCpu += itemDeserializeCpu
                decodeWall += itemDecodeWall
                decodeCpu += itemDecodeCpu
                iProgress.sample()

                _validateWorkerResult(
                    resultStatus, idx, payload, submitted,
                    observationsByIndex)

                # Store a sentinel for unsuccessful detections as well; this
                # makes duplicate result detection independent of payload.
                observationsByIndex[idx] = payload
                run_artifacts.record_detection(dataset, idx, payload)
                if memoryTracker is not None:
                    memoryTracker.sample(processes)

            if submitted != numImages:
                raise RuntimeError(
                    "Dataset reported {0} images but yielded {1}".format(
                        numImages, submitted))

            for unused_idx in range(numProcesses):
                _putWithWorkerHealth(
                    taskq, None, processes, onWait=sampleMemory)
            _joinProcesses(processes)
            completedNormally = True
            if memoryTracker is not None:
                memoryTracker.sample(force=True)
            targetObservations = [
                observationsByIndex[idx]
                for idx in range(submitted)
                if observationsByIndex[idx] is not None
            ]
        except Exception as error:
            if memoryTracker is not None:
                memoryTracker.sample(processes, force=True)
            _recordExtractionTiming(
                dataset, True, numProcesses, numImages, submitted,
                len(targetObservations), bagReadWall, bagReadCpu,
                deserializeWall, deserializeCpu, decodeWall, decodeCpu, detectWall,
                detectCpu, totalWallStart, totalCpuStart, memoryTracker, "error",
                detailedImageTiming, deferredImageLoading, repr(error))
            raise RuntimeError(
                "Exception during multithreaded extraction: {0}".format(error))
        finally:
            try:
                if not completedNormally:
                    _stopProcesses(processes)
            finally:
                # Queue feeder threads must be detached even if a pathological
                # worker survives SIGKILL and cleanup reports an error.
                _closeQueue(taskq, cancelPending=not completedNormally)
                _closeQueue(resultq, cancelPending=not completedNormally)

    else:
        datasetIterator = iter(
            dataset.readDatasetWithTiming()
            if detailedImageTiming else dataset.readDataset())
        try:
            while True:
                if timingActive and not detailedImageTiming:
                    decodeWallStart = time.perf_counter()
                    decodeCpuStart = time.process_time()
                try:
                    item = next(datasetIterator)
                except StopIteration:
                    break
                finally:
                    if timingActive and not detailedImageTiming:
                        decodeWall += time.perf_counter() - decodeWallStart
                        decodeCpu += time.process_time() - decodeCpuStart

                if detailedImageTiming:
                    timestamp, image, imageTiming = item
                    bagReadWall += imageTiming.get(
                        "bag_read_wall_seconds", 0.0)
                    bagReadCpu += imageTiming.get(
                        "bag_read_cpu_seconds", 0.0)
                    deserializeWall += imageTiming.get(
                        "deserialize_wall_seconds", 0.0)
                    deserializeCpu += imageTiming.get(
                        "deserialize_cpu_seconds", 0.0)
                    decodeWall += imageTiming.get(
                        "decode_wall_seconds", 0.0)
                    decodeCpu += imageTiming.get(
                        "decode_cpu_seconds", 0.0)
                else:
                    timestamp, image = item

                if submitted >= numImages:
                    raise RuntimeError(
                        "Dataset reported {0} images but yielded more".format(
                            numImages))
                submitted += 1
                if timingActive:
                    detectWallStart = time.perf_counter()
                    detectCpuStart = time.process_time()
                convertedImage = np.array(image)
                if noTransformation:
                    success, observation = detector.findTargetNoTransformation(
                        timestamp, convertedImage)
                else:
                    success, observation = detector.findTarget(
                        timestamp, convertedImage)
                if timingActive:
                    detectWall += time.perf_counter() - detectWallStart
                    detectCpu += time.process_time() - detectCpuStart
                if clearImages and observation is not None:
                    observation.clearImage()
                if success == 1:
                    targetObservations.append(observation)
                run_artifacts.record_detection(
                    dataset, submitted - 1, observation if success == 1 else None)
                iProgress.sample()
                if memoryTracker is not None:
                    memoryTracker.sample()

            if submitted != numImages:
                raise RuntimeError(
                    "Dataset reported {0} images but yielded {1}".format(
                        numImages, submitted))
        except Exception as error:
            if memoryTracker is not None:
                memoryTracker.sample(force=True)
            _recordExtractionTiming(
                dataset, False, 0, numImages, submitted,
                len(targetObservations), bagReadWall, bagReadCpu,
                deserializeWall, deserializeCpu, decodeWall, decodeCpu, detectWall,
                detectCpu, totalWallStart, totalCpuStart, memoryTracker, "error",
                detailedImageTiming, deferredImageLoading, repr(error))
            raise

    if memoryTracker is not None:
        memoryTracker.sample(force=True)
    extractionStatus = "ok" if targetObservations else "empty"
    _recordExtractionTiming(
        dataset, multithreading, numProcesses or 0, numImages, submitted,
        len(targetObservations), bagReadWall, bagReadCpu,
        deserializeWall, deserializeCpu, decodeWall, decodeCpu,
        detectWall, detectCpu, totalWallStart, totalCpuStart, memoryTracker,
        extractionStatus, detailedImageTiming, deferredImageLoading)

    if len(targetObservations) == 0:
        print("\r")
        sm.logFatal(
            "No corners could be extracted for camera {0}! Check the calibration target configuration and dataset.".format(
                dataset.topic))
    else:
        print("\r  Extracted corners for %d images (of %d images)                              " %
              (len(targetObservations), numImages))

    cv2.destroyAllWindows()
    return targetObservations
