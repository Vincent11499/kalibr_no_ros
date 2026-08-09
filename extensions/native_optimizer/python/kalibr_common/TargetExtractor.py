import sm

import numpy as np
import sys
import multiprocessing
try:
   import queue
except ImportError:
   import Queue as queue # python 2.x
import copy
import cv2


def multicoreExtractionWrapper(detector, taskq, resultq, clearImages, noTransformation):
    """Consume a bounded task stream and report one result for every image."""
    while True:
        task = taskq.get()
        if task is None:
            return

        idx, stamp, image = task
        try:
            if noTransformation:
                success, obs = detector.findTargetNoTransformation(stamp, np.array(image))
            else:
                success, obs = detector.findTarget(stamp, np.array(image))

            if clearImages:
                obs.clearImage()
            resultq.put(("result", idx, obs if success else None))
        except Exception as error:
            # Report the failing index instead of silently terminating a worker
            # and leaving the parent blocked waiting for a missing result.
            resultq.put(("error", idx, repr(error)))


def _stopProcesses(processes):
    for process in processes:
        if process.is_alive():
            process.terminate()
    for process in processes:
        process.join()


def extractCornersFromDataset(dataset, detector, multithreading=False,
                              numProcesses=None, clearImages=True,
                              noTransformation=False):
    print("Extracting calibration target corners")
    targetObservations = []
    numImages = dataset.numImages()

    iProgress = sm.Progress2(numImages)
    iProgress.sample()

    if multithreading:
        if not numProcesses:
            numProcesses = max(1, multiprocessing.cpu_count() - 1)
        numProcesses = max(1, min(numProcesses, numImages))

        # At most two pending decoded images per worker are retained. The
        # upstream implementation filled two Manager queues with the complete
        # dataset before starting any worker, which duplicated all images.
        taskq = multiprocessing.Queue(maxsize=max(1, 2 * numProcesses))
        resultq = multiprocessing.Queue()
        processes = []
        try:
            for unused_idx in range(numProcesses):
                detectorCopy = copy.copy(detector)
                process = multiprocessing.Process(
                    target=multicoreExtractionWrapper,
                    args=(detectorCopy, taskq, resultq, clearImages,
                          noTransformation))
                process.start()
                processes.append(process)

            submitted = 0
            for idx, (timestamp, image) in enumerate(dataset.readDataset()):
                taskq.put((idx, timestamp, image))
                submitted += 1

            if submitted != numImages:
                raise RuntimeError(
                    "Dataset reported {0} images but yielded {1}".format(
                        numImages, submitted))

            for unused_idx in range(numProcesses):
                taskq.put(None)

            observationsByIndex = []
            firstError = None
            remaining = numImages
            while remaining:
                try:
                    status, idx, payload = resultq.get(timeout=0.5)
                except queue.Empty:
                    if all(not process.is_alive() for process in processes):
                        raise RuntimeError(
                            "Corner extraction workers exited with {0} images pending".format(
                                remaining))
                    continue

                remaining -= 1
                iProgress.sample()
                if status == "error":
                    if firstError is None:
                        firstError = (idx, payload)
                elif payload is not None:
                    observationsByIndex.append((idx, payload))

            for process in processes:
                process.join()
            if firstError is not None:
                raise RuntimeError(
                    "Corner extraction failed for image {0}: {1}".format(
                        firstError[0], firstError[1]))

            targetObservations = [
                observation for unused_idx, observation in
                sorted(observationsByIndex, key=lambda item: item[0])]
        except Exception as error:
            _stopProcesses(processes)
            raise RuntimeError(
                "Exception during multithreaded extraction: {0}".format(error))
        finally:
            taskq.close()
            resultq.close()

    else:
        for timestamp, image in dataset.readDataset():
            if noTransformation:
                success, observation = detector.findTargetNoTransformation(
                    timestamp, np.array(image))
            else:
                success, observation = detector.findTarget(
                    timestamp, np.array(image))
            if clearImages:
                observation.clearImage()
            if success == 1:
                targetObservations.append(observation)
            iProgress.sample()

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
