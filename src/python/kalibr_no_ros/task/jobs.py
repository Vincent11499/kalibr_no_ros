"""Stable calibration job identifiers and job families."""

CAMERA_JOB = "camera_calibration"
CAMERA_IMU_JOB = "camera_imu_calibration"
CAMERA_BASELINK_JOB = "camera_baselink_calibration"
CAMERA_RS_JOB = "camera_rolling_shutter_calibration"
CAMERA_IMU_RS_JOB = "camera_imu_rolling_shutter_calibration"
CAMERA_IMU_JOBS = frozenset((CAMERA_IMU_JOB, CAMERA_IMU_RS_JOB))
CAMERA_CALIBRATION_JOBS = frozenset((CAMERA_JOB, CAMERA_RS_JOB))
ROLLING_SHUTTER_JOBS = frozenset((CAMERA_RS_JOB, CAMERA_IMU_RS_JOB))
