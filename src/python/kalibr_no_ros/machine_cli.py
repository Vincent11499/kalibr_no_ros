"""Human-readable command-line interface for staged machine calibration."""

from pathlib import Path
import argparse
import json
import math
import sys

from .machine_workflow import run_machine_workflow
from .task import TaskError, load_yaml
from .version import VERSION


_STAGE_LABELS = {
    "camera": "双目相机标定",
    "camera_imu": "双目 Camera–IMU 标定",
    "camera_baselink": "双目 Camera–BaseLink 标定",
    "mono_camera": "USB 单目相机标定",
    "mono_camera_baselink": "USB Camera–BaseLink 标定",
}
_ROLE_LABELS = {
    "stereo": "双目",
    "stereo_imu": "双目＋IMU",
    "stereo_baselink": "双目 BaseLink",
    "mono": "USB 单目",
    "mono_baselink": "USB BaseLink",
}
_COLORS = {
    "green": "\033[32m",
    "red": "\033[31m",
    "yellow": "\033[33m",
    "cyan": "\033[36m",
    "reset": "\033[0m",
}


def _is_tty(stream):
    try:
        return bool(stream.isatty())
    except (AttributeError, OSError):
        return False


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


def _finite_number(value):
    return (not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(float(value)))


def _number(value, digits=3):
    if not _finite_number(value):
        return None
    return ("{:.{}f}".format(float(value), digits).rstrip("0").rstrip(".")
            or "0")


def _byte_size(value):
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        return "未知容量"
    units = ("B", "KiB", "MiB", "GiB", "TiB")
    amount = float(value)
    for unit in units:
        if amount < 1024.0 or unit == units[-1]:
            digits = 0 if unit == "B" else 3
            return "{} {}".format(_number(amount, digits), unit)
        amount /= 1024.0
    return "{} B".format(value)


def _metric_lines(result_path):
    """Return concise final statistics without making reporting a dependency."""
    result_path = Path(result_path)
    try:
        result = load_yaml(result_path)
    except (OSError, ValueError, TypeError, TaskError):
        result = {}
    if not isinstance(result, dict):
        result = {}
    try:
        metrics = json.loads(
            (result_path.parent / "metrics.json").read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        metrics = {}
    if not isinstance(metrics, dict):
        metrics = {}

    camera_metrics = metrics.get("cameras", {})
    if not isinstance(camera_metrics, dict):
        camera_metrics = {}
    cameras = result.get("cameras", [])
    if not isinstance(cameras, list):
        cameras = []
    lines = []
    for camera in cameras:
        if not isinstance(camera, dict):
            continue
        camera_id = str(camera.get("id", "camera"))
        values = camera_metrics.get(camera_id, {})
        if not isinstance(values, dict):
            values = {}
        reprojection = values.get("reprojection", {})
        if not isinstance(reprojection, dict):
            reprojection = {}
        frame_parts = []
        for label, key in (
                ("输入", "input_frames"), ("检出", "detected_frames"),
                ("使用", "used_frames")):
            value = values.get(key)
            if isinstance(value, int) and not isinstance(value, bool):
                frame_parts.append("{} {}".format(label, value))
        details = []
        corners = reprojection.get("count")
        if isinstance(corners, int) and not isinstance(corners, bool):
            details.append("角点 {}".format(corners))
        rms = reprojection.get("rms_px", camera.get("rms"))
        rms_text = _number(rms)
        if rms_text is not None:
            details.append("RMS {} px".format(rms_text))
        description = []
        if frame_parts:
            description.append("图像 " + " / ".join(frame_parts))
        description.extend(details)
        if description:
            lines.append("{}: {}".format(camera_id, "，".join(description)))

        shift = values.get(
            "timeshift_cam_imu_s",
            camera.get("timeshift_cam_imu_s", camera.get("timeshift_cam_imu")))
        if _finite_number(shift):
            lines.append("{}: Camera→IMU 时间修正 {} ms".format(
                camera_id, _number(float(shift) * 1000.0, 4)))
        shutter = values.get("shutter", camera.get("shutter"))
        if isinstance(shutter, dict) and _finite_number(
                shutter.get("line_delay_s")):
            lines.append("{}: 行延时 {} μs/row".format(
                camera_id,
                _number(float(shutter["line_delay_s"]) * 1e6, 4)))
        pitch = values.get("pitch", camera.get("pitch"))
        if _finite_number(pitch):
            lines.append("{}: pitch {}°".format(camera_id, _number(pitch, 4)))
        pairs = values.get("effective_pairs")
        residual = _number(values.get("movement_residual_mm"), 4)
        if pairs is not None or residual is not None:
            parts = []
            if isinstance(pairs, int) and not isinstance(pairs, bool):
                parts.append("有效帧对 {}".format(pairs))
            if residual is not None:
                parts.append("平移差分 RMS {} mm".format(residual))
            lines.append("{}: {}".format(camera_id, "，".join(parts)))

    stereo_pairs = metrics.get("stereo_pairs", {})
    if isinstance(stereo_pairs, dict):
        for pair_id, pair in stereo_pairs.items():
            if not isinstance(pair, dict):
                continue
            alignment = pair.get("alignment")
            if not isinstance(alignment, dict):
                alignment = {}
            parts = []
            count = alignment.get("count")
            if isinstance(count, int) and not isinstance(count, bool):
                parts.append("共同角点 {}".format(count))
            alignment_value = alignment.get("rms_px", alignment.get("mean"))
            alignment_text = _number(alignment_value)
            if alignment_text is not None:
                parts.append("对齐误差 {} px".format(alignment_text))
            baseline_text = _number(pair.get("baseline_m"), 4)
            if baseline_text is not None:
                parts.append("基线 {} m".format(baseline_text))
            if parts:
                lines.append("{}: {}".format(pair_id, "，".join(parts)))

    imus = metrics.get("imus", {})
    if isinstance(imus, dict):
        for imu_id, imu in imus.items():
            if not isinstance(imu, dict):
                continue
            for kind, label in (("gyro", "陀螺仪"), ("accel", "加速度计")):
                values = imu.get(kind)
                if not isinstance(values, dict):
                    continue
                parts = []
                count = values.get("count")
                if isinstance(count, int) and not isinstance(count, bool):
                    parts.append("样本 {}".format(count))
                rms_text = _number(values.get("rms"), 5)
                if rms_text is not None:
                    unit = values.get("unit", "")
                    parts.append("RMS {}{}".format(
                        rms_text, " " + str(unit) if unit else ""))
                if parts:
                    lines.append("{} {}: {}".format(
                        imu_id, label, "，".join(parts)))

    optimizer = metrics.get("optimizer")
    if isinstance(optimizer, dict):
        parts = []
        iterations = optimizer.get("iterations")
        if isinstance(iterations, int) and not isinstance(iterations, bool):
            parts.append("迭代 {}".format(iterations))
        for label, key in (("初始目标", "JStart"), ("最终目标", "JFinal")):
            value = _number(optimizer.get(key), 5)
            if value is not None:
                parts.append("{} {}".format(label, value))
        if parts:
            lines.append("优化器: " + "，".join(parts))
    return lines


class _ConsoleProgress:
    def __init__(self, stream=None):
        self.stream = stream if stream is not None else sys.stdout
        self.color = _is_tty(self.stream)
        self.task_sections = 0

    def _paint(self, text, color):
        if not self.color:
            return text
        return _COLORS[color] + text + _COLORS["reset"]

    def _write(self, text=""):
        print(text, file=self.stream, flush=True)

    def _stage_header(self, details):
        if self.task_sections:
            self._write()
            self._write()
        else:
            self._write()
        self.task_sections += 1
        stage = details["stage"]
        self._write("=" * 72)
        self._write("[{}/{}] {}".format(
            details["index"], details["total"],
            _STAGE_LABELS.get(stage, stage)))

    def __call__(self, event, details):
        if event == "workflow_start":
            self._write("机器标定：{}，批次 {}".format(
                details["machine"], details["batch"]))
            self._write("正在搜索数据集目录…")
        elif event == "dataset_complete":
            prefix = "  数据集搜索 [{}/{}] {}：".format(
                details["index"], details["total"],
                _ROLE_LABELS.get(details["role"], details["role"]))
            if details.get("dataset"):
                self._write(prefix + self._paint("[找到]", "cyan") + " "
                            + details["dataset"])
            else:
                pattern = {
                    "stereo": "stereo_YYMMDD_HHMM",
                    "stereo_imu": "stereo_imu_YYMMDD_HHMM",
                    "stereo_baselink": "stereo_base_link_YYMMDD_HHMM",
                    "mono": "usb_YYMMDD_HHMM",
                    "mono_baselink": "usb_base_link_YYMMDD_HHMM",
                }.get(details["role"], details["role"])
                self._write(prefix + self._paint("[未发现]", "yellow")
                            + " " + pattern)
        elif event == "dataset_validation_begin":
            total = details.get("total", 0)
            if total:
                self._write()
                self._write("正在校验 {} 个输入数据集…".format(total))
            else:
                self._write()
                self._write(self._paint(
                    "[未发现] 当前批次没有可执行的数据集", "yellow"))
        elif event == "dataset_validation_start":
            self._write("  输入校验 [{}/{}] {}（{} 个文件，{}）".format(
                details["index"], details["total"], details["dataset"],
                details["file_count"], _byte_size(details["total_bytes"])))
        elif event == "dataset_validation_progress":
            self._write("    哈希进度 {}%（{} / {}）".format(
                details["percent"],
                _byte_size(details["processed_bytes"]),
                _byte_size(details["hashed_bytes"])))
        elif event == "dataset_validation_complete":
            elapsed = float(details.get("elapsed_s", 0.0))
            reused = details.get("reused_files", 0)
            hashed = details.get("hashed_files", 0)
            if details.get("mode") == "metadata_snapshot":
                label = self._paint("[快照]", "green")
                text = "已记录 {} 个文件元数据，未读取图像内容".format(
                    details.get("file_count", 0))
            elif hashed == 0:
                label = self._paint("[缓存]", "yellow")
                text = "{} 个文件未变化，无需重新读取图像".format(reused)
            else:
                label = self._paint("[完成]", "green")
                text = "复用 {} 个、重新哈希 {} 个（{}）".format(
                    reused, hashed, _byte_size(details.get("hashed_bytes")))
            self._write("    {} {}，耗时 {:.1f} s".format(
                label, text, elapsed))
        elif event == "workflow_state_start":
            self._write("正在比较 workflow 状态和既有结果…")
        elif event == "dataset_recheck_start":
            self._write("  [校验] 正在确认求解期间输入未变化…")
        elif event == "dataset_recheck_progress":
            self._write("    复查哈希 {}%".format(details["percent"]))
        elif event == "dataset_recheck_complete":
            self._write("  " + self._paint(
                "[通过] 求解期间输入数据未变化", "green"))
        elif event == "stage_start":
            self._stage_header(details)
            self._write("数据集：{}".format(details["dataset"]))
            self._write(self._paint("[进行中] 已开始", "cyan"))
        elif event == "stage_complete":
            action = details.get("action")
            if action == "reused":
                status = self._paint("[复用] 既有结果检查通过", "yellow")
            else:
                status = self._paint("[成功] 标定和交付完成", "green")
            self._write("{}（{:.1f} s）".format(
                status, float(details.get("elapsed_s", 0.0))))
            lines = _metric_lines(details["result"])
            if lines:
                self._write("结果摘要：")
                for line in lines:
                    self._write("  " + line)
            else:
                self._write("结果摘要：详细统计不可用，请查看任务日志。")
        elif event == "stage_skip":
            if details.get("index") is None:
                return
            self._stage_header(details)
            status = details["status"]
            if status.startswith("blocked"):
                label = self._paint("[失败] 前置条件不满足", "red")
            elif status == "disabled":
                label = self._paint("[跳过] 未启用", "yellow")
            else:
                label = self._paint("[等待] 尚无数据集", "yellow")
            self._write(label)
            if details.get("message"):
                self._write("  " + details["message"])
        elif event == "stage_failed":
            self._write("{}（{:.1f} s）".format(
                self._paint("[失败] 标定未完成", "red"),
                float(details.get("elapsed_s", 0.0))))
            if details.get("error"):
                self._write("  " + details["error"])
        elif event == "calib_params_start":
            if self.task_sections:
                self._write()
                self._write()
            else:
                self._write()
            self._write("=" * 72)
            self._write("正在汇总机器标定参数…")
        elif event == "calib_params_complete":
            artifacts = details.get("artifacts", {})
            if artifacts:
                rendered = "，".join(
                    "{} {}".format(name, action)
                    for name, action in sorted(artifacts.items()))
                self._write(self._paint("[成功] 参数汇总完成", "green")
                            + "（{}）".format(rendered))
            else:
                self._write(self._paint("[完成] 无可汇总参数", "yellow"))
        elif event == "workflow_complete":
            self._write("=" * 72)
            self._write(self._paint(
                "[成功] Workflow 完成（{:.1f} s）".format(
                    float(details.get("elapsed_s", 0.0))),
                "green"))


def main(argv=None, prefix=None):
    arguments = build_parser().parse_args(argv)
    prefix = Path(prefix or Path(__file__).resolve().parents[3]).resolve()
    config = (Path(arguments.config) if arguments.config is not None
              else prefix / "config" / "user.yaml")
    progress = _ConsoleProgress()
    try:
        summary = run_machine_workflow(
            prefix, config, progress=progress)
        stages = [summary[key] for key in _STAGE_LABELS if key in summary]
        return (2 if any(
            stage.get("status", "").startswith(("blocked", "failed"))
            for stage in stages) else 0)
    except (TaskError, ValueError, RuntimeError, OSError) as error:
        message = "kalibr-cedarobot: error: {}".format(error)
        if _is_tty(sys.stderr):
            message = _COLORS["red"] + message + _COLORS["reset"]
        print(message, file=sys.stderr, flush=True)
        return 2


__all__ = ["build_parser", "main"]
