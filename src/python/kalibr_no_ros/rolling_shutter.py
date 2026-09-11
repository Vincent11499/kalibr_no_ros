"""Input contract for the opt-in rolling-shutter camera/IMU job (no solver imports)."""
import math

JOB = "camera_imu_rolling_shutter_calibration"
CAMERA_IMU_JOBS = frozenset(("camera_imu_calibration", JOB))


def validate_shutters(value, camera_ids=None):
    from .task import TaskError
    if not isinstance(value, dict) or not value:
        raise TaskError("rolling_shutter must map every camera ID to its timing configuration")
    if camera_ids is not None and set(value) != set(camera_ids):
        raise TaskError("rolling_shutter IDs must exactly match camera_calibration cameras")
    result = {}
    for camera_id, block in value.items():
        if not isinstance(camera_id, str) or not isinstance(block, dict):
            raise TaskError("rolling_shutter entries must be camera ID mappings")
        if set(block) - {"line_delay_s", "estimate", "max_abs_line_delay_s"}:
            raise TaskError("unknown rolling_shutter fields for " + camera_id)
        estimate = block.get("estimate", True)
        if type(estimate) is not bool:
            raise TaskError("rolling_shutter.estimate must be boolean")
        seed = block.get("line_delay_s", 0.0)
        if type(seed) not in (int, float) or not math.isfinite(seed):
            raise TaskError("rolling_shutter.line_delay_s must be finite seconds per row")
        bound = block.get("max_abs_line_delay_s")
        if estimate or bound is not None:
            if (type(bound) not in (int, float) or not math.isfinite(bound)
                    or bound <= 0 or abs(seed) >= bound):
                raise TaskError("estimated line_delay_s requires a positive max_abs_line_delay_s strictly above abs(seed)")
        result[camera_id] = {"line_delay_s": float(seed), "estimate": estimate}
        if bound is not None:
            result[camera_id]["max_abs_line_delay_s"] = float(bound)
    return result
