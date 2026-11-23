"""Construct a native calibration-board detector for existing camera geometry."""


def create_target_detector(target_path, geometry, window_half_size_px,
                           max_displacement_px, show_extraction=False,
                           extraction_stepping=False):
    from kalibr_common.ConfigReader import CalibrationTargetParameters
    from kalibr_camera_calibration.CameraCalibrator import TargetDetector

    target = CalibrationTargetParameters(str(target_path))
    return TargetDetector(
        target,
        geometry,
        showCorners=show_extraction,
        showOneStep=extraction_stepping,
        windowHalfSizePx=window_half_size_px,
        maxDisplacementPx=max_displacement_px,
    ).detector
