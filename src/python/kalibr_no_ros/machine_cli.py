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
        "--config", required=True,
        help="developer-provided machine workflow user.yaml")
    return parser


def _print_stage(label, stage):
    text = "{}: {}".format(label, stage["status"])
    if stage.get("action"):
        text += " ({})".format(stage["action"])
    if stage.get("result"):
        text += " -> {}".format(stage["result"])
    print(text)


def main(argv=None, prefix=None):
    arguments = build_parser().parse_args(argv)
    prefix = Path(prefix or Path(__file__).resolve().parents[3]).resolve()
    try:
        summary = run_machine_workflow(
            prefix, arguments.config)
        print("machine: {}".format(summary["machine_dir"]))
        _print_stage("camera", summary["camera"])
        _print_stage("camera_imu", summary["camera_imu"])
        print("output: {}".format(summary["output_dir"]))
        return (2 if any(
            stage.get("status", "").startswith(("blocked", "failed"))
            for stage in (summary["camera"], summary["camera_imu"])) else 0)
    except (TaskError, ValueError, RuntimeError, OSError) as error:
        print("kalibr-cedarobot: error: {}".format(error), file=sys.stderr)
        return 2


__all__ = ["build_parser", "main"]
