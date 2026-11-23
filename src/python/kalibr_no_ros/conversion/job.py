"""Convert legacy command options to a versioned calibration task."""

from pathlib import Path

from ..task.document import TASK_SCHEMA_VERSION, TaskError, dump_yaml
from ..task.execution import STANDARD_EXECUTION


def convert_job(arguments):
    output = Path(arguments.output).expanduser().resolve()
    if output.exists() and not arguments.force:
        raise TaskError("output exists; pass --force to replace it: {}".format(output))
    dataset = {
        "type": "bag",
        "path": str(Path(arguments.bag).expanduser().resolve()),
    }
    if arguments.bag_from_to:
        dataset["time_range_s"] = list(arguments.bag_from_to)
    if arguments.bag_freq is not None:
        dataset["frequency_hz"] = arguments.bag_freq
    task = {
        "schema_version": TASK_SCHEMA_VERSION,
        "job": (
            "camera_calibration"
            if arguments.type == "cameras"
            else "camera_imu_calibration"
        ),
        "dataset": dataset,
        "target": {"path": str(Path(arguments.target).expanduser().resolve())},
    }
    calibration = {}
    task["kind"] = "calibration_task"
    task["output"] = {"interactive_report": False}
    if arguments.type == "cameras":
        if not arguments.topics or not arguments.models:
            raise TaskError("camera conversion requires --topics and --models")
        if len(arguments.topics) != len(arguments.models):
            raise TaskError("--topics and --models must have equal lengths")
        task["cameras"] = [
            {"id": "cam{}".format(index), "topic": topic, "model": model}
            for index, (topic, model) in enumerate(zip(arguments.topics, arguments.models))
        ]
        if arguments.no_shuffle:
            calibration["shuffle"] = False
        if arguments.approx_sync is not None:
            calibration["synchronization_tolerance_s"] = arguments.approx_sync
    else:
        if not arguments.cams or not arguments.imu:
            raise TaskError("IMU conversion requires --cams and --imu")
        models = arguments.imu_models or ["calibrated"] * len(arguments.imu)
        if len(models) != len(arguments.imu):
            raise TaskError("--imu and --imu-models must have equal lengths")
        task["camera_calibration"] = {
            "path": str(Path(arguments.cams).expanduser().resolve())
        }
        task["imus"] = [
            {"id": "imu{}".format(index), "path": str(Path(path).expanduser().resolve()), "model": model}
            for index, (path, model) in enumerate(zip(arguments.imu, models))
        ]
        if arguments.max_iter is not None:
            calibration["max_iterations"] = arguments.max_iter
        if arguments.timeoffset_padding is not None:
            calibration["time_offset_padding_s"] = arguments.timeoffset_padding
        if arguments.no_time_calibration:
            calibration["calibrate_time_offset"] = False
        if arguments.recover_covariance:
            calibration["recover_covariance"] = True
        if arguments.recompute_camera_chain_extrinsics:
            calibration["recompute_camera_chain_extrinsics"] = True
    task["calibration"] = calibration
    task["execution"] = dict(STANDARD_EXECUTION)
    output.parent.mkdir(parents=True, exist_ok=True)
    dump_yaml(task, output)
    print(output)
    return 0
