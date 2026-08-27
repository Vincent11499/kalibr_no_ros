"""Public command hierarchy for the ROS-free Kalibr application."""

from pathlib import Path
import argparse
import sys

from .reference import verify_snapshot
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
    parser.add_argument("--memory-sample-interval", type=float,
                        dest="profiling_memory_sample_interval_s")


def _add_runtime_arguments(parser):
    parser.add_argument("--config", required=True, help="task schema-version 1 YAML")
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
    parser.add_argument("--timing-json", help="profile-build timing output")
    parser.add_argument("--force", action="store_true")


def build_parser():
    parser = argparse.ArgumentParser(prog="kalibr-noros")
    parser.add_argument("--version", action="version", version="kalibr-noros 2.0.0")
    commands = parser.add_subparsers(dest="group", required=True)

    calibrate = commands.add_parser("calibrate", help="run calibration")
    calibration_commands = calibrate.add_subparsers(dest="calibration", required=True)
    cameras = calibration_commands.add_parser("cameras", help="camera calibration")
    _add_runtime_arguments(cameras)
    imu_camera = calibration_commands.add_parser(
        "imu-camera", help="camera to IMU calibration"
    )
    _add_runtime_arguments(imu_camera)

    benchmark = commands.add_parser(
        "benchmark", help="run and compare immutable performance archives")
    benchmark_commands = benchmark.add_subparsers(
        dest="benchmark_command", required=True)
    benchmark_run = benchmark_commands.add_parser(
        "run", help="run only a new candidate and archive all measurements")
    benchmark_run.add_argument("--config", required=True)
    benchmark_run.add_argument("--archive-dir", required=True)
    benchmark_run.add_argument("--name", required=True)
    benchmark_run.add_argument("--repeat", type=_positive_integer, default=1)
    _add_execution_arguments(benchmark_run)
    benchmark_compare = benchmark_commands.add_parser(
        "compare", help="compare a candidate archive without rerunning baseline")
    benchmark_compare.add_argument("--registry", required=True)
    benchmark_compare.add_argument("--baseline-id", required=True)
    benchmark_compare.add_argument("--candidate", required=True)
    benchmark_compare.add_argument("--atol", type=float, default=1e-8)
    benchmark_compare.add_argument("--rtol", type=float, default=1e-8)

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

    reference = commands.add_parser("reference", help="reference snapshot tools")
    reference_commands = reference.add_subparsers(dest="reference", required=True)
    verify = reference_commands.add_parser("verify", help="verify the ETHZ snapshot")
    verify.add_argument("--snapshot")
    verify.add_argument("--manifest")
    verify.add_argument("--expected-files", type=int)
    verify.add_argument("--json")
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


def _default_reference_path(prefix, name):
    candidates = (
        Path(prefix) / "ref" / name,
        Path(prefix).parent.parent / "ref" / name,
        Path.cwd() / "ref" / name,
    )
    return next((path for path in candidates if path.exists()), candidates[0])


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
    calibration = {"interactive_report": False}
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
        if arguments.group == "benchmark":
            from .benchmark import compare_benchmark, run_benchmark
            if arguments.benchmark_command == "run":
                output = run_benchmark(
                    prefix,
                    arguments.config,
                    arguments.archive_dir,
                    arguments.name,
                    repeat=arguments.repeat,
                    **_runtime_overrides(arguments, arguments.archive_dir)
                )
                print(output)
                return 0
            json_path, markdown_path = compare_benchmark(
                arguments.registry,
                arguments.baseline_id,
                arguments.candidate,
                arguments.atol,
                arguments.rtol,
            )
            print(json_path)
            print(markdown_path)
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
            if arguments.full_fisheye:
                from kalibr_opencv_fisheye_full.__main__ import main as converter
            else:
                from kalibr_opencv_fisheye.yaml_io import main as converter
            converter_arguments = list(arguments.arguments)
            if arguments.converter_help:
                converter_arguments.insert(0, "--help")
            return converter(converter_arguments)
        snapshot = arguments.snapshot or _default_reference_path(prefix, "kalibr")
        manifest = arguments.manifest or _default_reference_path(prefix, "kalibr.sha256")
        result = verify_snapshot(
            snapshot,
            manifest,
            expected_files=arguments.expected_files,
            json_path=arguments.json,
        )
        print(
            "reference snapshot verified: {} files, {}".format(
                result["actual_files"], result["actual_sha256"]
            )
        )
        return 0
    except (TaskError, ValueError, RuntimeError, OSError, StopIteration) as error:
        print("kalibr-noros: error: {}".format(error), file=sys.stderr)
        return 2
