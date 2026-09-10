"""Public command hierarchy for the ROS-free Kalibr application."""

from pathlib import Path
import argparse
import json
import sys

from .version import VERSION

from kalibr_runtime import profiling_enabled
from .task import (
    TASK_SCHEMA_VERSION,
    STANDARD_EXECUTION,
    TaskError,
    dump_yaml,
    run_task,
)


def _positive_integer(value):
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def _add_execution_arguments(parser):
    parser.add_argument("--parallelism", type=_positive_integer)
    parser.add_argument("--detector-processes", type=_positive_integer)
    parser.add_argument("--optimizer-threads", type=_positive_integer)
    parser.add_argument("--detector-inflight-per-worker", type=_positive_integer)
    parser.add_argument("--detector-opencv-threads", type=_positive_integer)
    if profiling_enabled():
        parser.add_argument("--memory-sample-interval", type=float,
                            dest="profiling_memory_sample_interval_s")


def _add_runtime_arguments(parser):
    parser.add_argument("--config", required=True, help="task schema-version 1.0.0 YAML")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--initialization",
        help="camera or camera-IMU initialization YAML (relative to the current directory)",
    )
    parser.add_argument(
        "--initialization-strategy",
        choices=("refine", "direct"),
        help="use seeds as refinement starting points or direct initial values",
    )
    _add_execution_arguments(parser)
    if profiling_enabled():
        parser.add_argument("--timing-json", help="profile-build timing output")
    parser.add_argument("--force", action="store_true")


def build_parser():
    parser = argparse.ArgumentParser(prog="kalibr-noros")
    parser.add_argument("--version", action="version", version="kalibr-noros " + VERSION)
    commands = parser.add_subparsers(dest="group", required=True)

    validate = commands.add_parser("validate", help="check task and dataset without detection or optimization")
    validate.add_argument("--config", required=True)
    validate.add_argument("--output", help="write the validation JSON to a new file")

    evaluate = commands.add_parser("evaluate", help="recompute metrics and reports from saved observations")
    evaluate.add_argument("--run", required=True, help="existing calibration run")
    evaluate.add_argument("--config", help="evaluation schema-version 1.0.0 YAML")
    evaluate.add_argument("--dataset", help="relocated directory dataset")
    evaluate.add_argument("--output-dir", required=True)
    evaluate.add_argument("--force", action="store_true")

    calibrate = commands.add_parser("calibrate", help="run calibration")
    calibration_commands = calibrate.add_subparsers(dest="calibration", required=True)
    cameras = calibration_commands.add_parser("cameras", help="camera calibration")
    _add_runtime_arguments(cameras)
    imu_camera = calibration_commands.add_parser(
        "imu-camera", help="camera to IMU calibration"
    )
    _add_runtime_arguments(imu_camera)

    convert = commands.add_parser("convert", help="convert configuration formats")
    conversion_commands = convert.add_subparsers(dest="conversion", required=True)
    camera = conversion_commands.add_parser(
        "camera",
        help="OpenCV/Kalibr camera YAML conversion",
        add_help=False,
    )
    camera.add_argument("-h", "--help", dest="converter_help", action="store_true")
    camera.add_argument("--full-fisheye", action="store_true")
    camera.add_argument("arguments", nargs=argparse.REMAINDER)

    job = conversion_commands.add_parser("job", help="create a task YAML")
    job.add_argument("--type", choices=("cameras", "imu-camera"), required=True)
    job.add_argument("--bag", required=True)
    job.add_argument("--target", required=True)
    job.add_argument("--output", required=True)
    job.add_argument("--topics", nargs="+")
    job.add_argument("--models", nargs="+")
    job.add_argument("--cams")
    job.add_argument("--imu", nargs="+")
    job.add_argument("--imu-models", nargs="+")
    job.add_argument("--bag-from-to", type=float, nargs=2)
    job.add_argument("--bag-freq", type=float)
    job.add_argument("--no-shuffle", action="store_true")
    job.add_argument("--approx-sync", type=float)
    job.add_argument("--max-iter", type=int)
    job.add_argument("--timeoffset-padding", type=float)
    job.add_argument("--no-time-calibration", action="store_true")
    job.add_argument("--recover-covariance", action="store_true")
    job.add_argument("--recompute-camera-chain-extrinsics", action="store_true")
    job.add_argument("--force", action="store_true")

    return parser


def _runtime_overrides(arguments, output_dir):
    timing = getattr(arguments, "timing_json", None)
    if timing is not None:
        timing = Path(timing)
        if not timing.is_absolute():
            timing = Path(output_dir).resolve() / timing
    initialization = getattr(arguments, "initialization", None)
    if initialization is not None:
        initialization = Path(initialization).expanduser().resolve()
    return {
        "initialization": initialization,
        "initialization_strategy": getattr(
            arguments, "initialization_strategy", None),
        "parallelism": getattr(arguments, "parallelism", None),
        "detector_processes": getattr(arguments, "detector_processes", None),
        "optimizer_threads": getattr(arguments, "optimizer_threads", None),
        "detector_inflight_per_worker": getattr(
            arguments, "detector_inflight_per_worker", None),
        "detector_opencv_threads": getattr(
            arguments, "detector_opencv_threads", None),
        "profiling_memory_sample_interval_s":
            getattr(arguments, "profiling_memory_sample_interval_s", None),
        "timing_json": timing,
    }


def _convert_job(arguments):
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


def main(argv=None, prefix=None):
    arguments = build_parser().parse_args(argv)
    prefix = Path(prefix or Path(__file__).resolve().parents[3])
    try:
        if arguments.group == "validate":
            from .validation import validate_task
            report = validate_task(arguments.config)
            serialized = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
            if arguments.output:
                destination = Path(arguments.output).expanduser().resolve()
                destination.parent.mkdir(parents=True, exist_ok=True)
                with destination.open("x", encoding="utf-8") as stream:
                    stream.write(serialized)
            print(serialized, end="")
            return 0 if report["status"] == "passed" else 2
        if arguments.group == "evaluate":
            from .reporting import evaluate_run, validate_output_options
            from .task import load_yaml, require_document_version, prepare_output_directory, write_run_manifest
            options = None
            if arguments.config:
                document = load_yaml(arguments.config)
                require_document_version(document, "evaluation", "calibration_evaluation")
                unknown = set(document) - {"schema_version", "kind", "output"}
                if unknown:
                    raise TaskError("unknown evaluation fields: {}".format(", ".join(sorted(unknown))))
                options = document.get("output", {})
            options = validate_output_options(options)
            source = Path(arguments.run).expanduser().resolve()
            destination = Path(arguments.output_dir).expanduser().resolve()
            if source == destination or source in destination.parents or destination in source.parents:
                raise TaskError("evaluation output must be separate from the source run")
            from .delivery import locate_result
            require_document_version(load_yaml(locate_result(source)), "calibration", "calibration_result")
            if arguments.config:
                config_path = Path(arguments.config).expanduser().resolve()
                if destination == config_path or destination in config_path.parents:
                    raise TaskError("evaluation output contains its configuration file")
            if arguments.dataset:
                dataset_path = Path(arguments.dataset).expanduser().resolve()
                if destination == dataset_path or destination in dataset_path.parents or dataset_path in destination.parents:
                    raise TaskError("evaluation output must be separate from the relocated dataset")
            output = destination
            evaluate_run(source, output, options, dataset=arguments.dataset, force=arguments.force)
            print(output)
            return 0
        if arguments.group == "calibrate":
            job = (
                "camera_calibration"
                if arguments.calibration == "cameras"
                else "camera_imu_calibration"
            )
            output = run_task(
                prefix,
                arguments.config,
                arguments.output_dir,
                job,
                force=arguments.force,
                **_runtime_overrides(arguments, arguments.output_dir)
            )
            print(output)
            return 0
        if arguments.group == "convert" and arguments.conversion == "job":
            return _convert_job(arguments)
        if arguments.group == "convert":
            from .conversion import main as converter
            converter_arguments = list(arguments.arguments)
            if arguments.converter_help:
                converter_arguments.insert(0, "--help")
            return converter(converter_arguments, full_fisheye=arguments.full_fisheye)
    except (TaskError, ValueError, RuntimeError, OSError, StopIteration) as error:
        print("kalibr-noros: error: {}".format(error), file=sys.stderr)
        return 2
