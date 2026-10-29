"""Concise command-line interface for staged machine calibration."""

from pathlib import Path
import argparse
import sys

from .machine_workflow import run_machine_workflow
from .task import TaskError
from .version import VERSION


def build_parser():
    parser = argparse.ArgumentParser(prog="kalibr-cedarobot")
    parser.add_argument(
        "--version", action="version", version="kalibr-cedarobot " + VERSION)
    commands = parser.add_subparsers(dest="command", required=True)
    machine = commands.add_parser(
        "machine", help="run or resume one machine calibration workflow")
    machine.add_argument(
        "--config",
        help=("developer-provided machine workflow user.yaml; defaults to "
              "<install-prefix>/config/user.yaml"))
    return parser


def _print_stage(label, stage):
    text = "{}: {}".format(label, stage["status"])
    if stage.get("action"):
        text += " ({})".format(stage["action"])
    if stage.get("result"):
        text += " -> {}".format(stage["result"])
    print(text)
    if stage.get("message"):
        print("  {}".format(stage["message"]))


def main(argv=None, prefix=None):
    arguments = build_parser().parse_args(argv)
    prefix = Path(prefix or Path(__file__).resolve().parents[3]).resolve()
    config = (Path(arguments.config) if arguments.config is not None
              else prefix / "config" / "user.yaml")
    try:
        summary = run_machine_workflow(
            prefix, config)
        print("machine: {}".format(summary["machine_dir"]))
        stages = {key: summary[key] for key in (
            "camera", "camera_imu", "camera_baselink", "mono_camera", "mono_camera_baselink")
            if key in summary}
        for label, stage in stages.items():
            _print_stage(label, stage)
        for label, artifact in summary.get("calib_params", {}).items():
            _print_stage("calib_params." + label, artifact)
        print("output: {}".format(summary["output_dir"]))
        return (2 if any(
            stage.get("status", "").startswith(("blocked", "failed"))
            for stage in stages.values()) else 0)
    except (TaskError, ValueError, RuntimeError, OSError) as error:
        print("kalibr-cedarobot: error: {}".format(error), file=sys.stderr)
        return 2


__all__ = ["build_parser", "main"]
