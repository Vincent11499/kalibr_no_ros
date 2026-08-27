"""Read-only calibration observability analysis and stable sidecar output."""

from __future__ import annotations

from pathlib import Path
import math
import time

import numpy as np
import yaml


OBSERVABILITY_SCHEMA_VERSION = 1


class ObservabilityError(RuntimeError):
    pass


class RankDeficiencyError(ObservabilityError):
    def __init__(self, report):
        calibration = report.get("calibration", {})
        super().__init__(
            "calibration is rank deficient: rank {}/{} (deficiency {})".format(
                calibration.get("rank", 0),
                calibration.get("columns", 0),
                calibration.get("deficiency", 0),
            )
        )
        self.report = report


def _finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def _as_serializable(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _as_serializable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_as_serializable(item) for item in value]
    return value


def _normalize_blocks(parameter_blocks):
    normalized = []
    flat_design_variables = []
    for block in parameter_blocks:
        name = str(block["name"])
        design_variables = list(block.get("design_variables") or ())
        if not design_variables:
            raise ObservabilityError(
                "parameter block {} has no design variables".format(name))
        dimensions = [int(dv.minimalDimensions()) for dv in design_variables]
        normalized.append({
            "name": name,
            "units": str(block.get("units", "model_native")),
            "coordinate_convention": block.get("coordinate_convention"),
            "design_variables": design_variables,
            "dimensions": dimensions,
        })
        flat_design_variables.extend(design_variables)
    return normalized, flat_design_variables


def _parameter_block(name, design_variable, units,
                     coordinate_convention=None):
    block = {
        "name": name,
        "design_variables": [design_variable],
        "units": units,
    }
    if coordinate_convention is not None:
        block["coordinate_convention"] = coordinate_convention
    return block


def _mode_contributions(nullspace, columns, deficiency, block_active_ranges):
    modes = []
    if nullspace.ndim != 2 or nullspace.shape != (columns, deficiency):
        return modes
    for mode_index in range(deficiency):
        contributions = []
        for name, begin, end in block_active_ranges:
            if end <= begin:
                continue
            contribution = float(np.dot(
                nullspace[begin:end, mode_index],
                nullspace[begin:end, mode_index],
            ))
            contributions.append({
                "block": name,
                "contribution": contribution,
            })
        contributions.sort(
            key=lambda item: (-item["contribution"], item["block"]))
        modes.append({
            "index": mode_index,
            "dominant_blocks": contributions,
        })
    return modes


def camera_parameter_blocks(calibrator):
    """Return physical camera calibration blocks in deterministic order."""
    blocks = []
    for index, camera in enumerate(calibrator.cameras):
        projection = camera.dv.projectionDesignVariable()
        if int(projection.minimalDimensions()) > 0:
            blocks.append(_parameter_block(
                "cameras.cam{}.intrinsics".format(index), projection,
                "pixels_and_model_native"))
        distortion = camera.dv.distortionDesignVariable()
        if int(distortion.minimalDimensions()) > 0:
            blocks.append(_parameter_block(
                "cameras.cam{}.distortion".format(index), distortion,
                "model_native"))
    for index, baseline in enumerate(calibrator.baselines, start=1):
        convention = (
            "p_cam{0} = T_cam{0}_cam{1} * p_cam{1}".format(
                index, index - 1))
        blocks.extend([
            _parameter_block(
                "camera_chain.cam{}.rotation".format(index), baseline.q,
                "radians", convention),
            _parameter_block(
                "camera_chain.cam{}.translation".format(index), baseline.t,
                "metres", convention),
        ])
    return blocks


def camera_imu_parameter_blocks(calibrator):
    """Return camera-IMU, multi-IMU and IMU-intrinsic calibration blocks."""
    blocks = []
    for index, camera in enumerate(calibrator.CameraChain.camList):
        if index == 0:
            convention = "p_cam0 = T_cam0_imu * p_imu0"
            prefix = "camera_imu.T_cam0_imu"
        else:
            convention = (
                "p_cam{0} = T_cam{0}_cam{1} * p_cam{1}".format(
                    index, index - 1))
            prefix = "camera_chain.cam{}".format(index)
        blocks.extend([
            _parameter_block(
                prefix + ".rotation", camera.T_c_b_Dv.q,
                "radians", convention),
            _parameter_block(
                prefix + ".translation", camera.T_c_b_Dv.t,
                "metres", convention),
            _parameter_block(
                "camera_imu.cam{}.time_offset".format(index),
                camera.cameraTimeToImuTimeDv, "seconds",
                "t_imu = t_cam + timeshift_cam_imu_s"),
        ])

    for index, imu in enumerate(calibrator.ImuList):
        if index:
            convention = (
                "p_imu{0} = T_imu{0}_imu0 * p_imu0".format(index))
            blocks.extend([
                _parameter_block(
                    "imus.imu{}.relative_rotation".format(index),
                    imu.q_i_b_Dv, "radians", convention),
                _parameter_block(
                    "imus.imu{}.relative_lever_arm_in_imu0".format(index),
                    imu.r_b_Dv, "metres",
                    convention + "; t_imuN_imu0 = -R_imuN_imu0 * r_imu0"),
            ])
        if hasattr(imu, "q_gyro_i_Dv"):
            blocks.extend([
                _parameter_block(
                    "imus.imu{}.C_gyro_i".format(index),
                    imu.q_gyro_i_Dv, "radians"),
                _parameter_block(
                    "imus.imu{}.M_accel".format(index),
                    imu.M_accel_Dv, "dimensionless"),
                _parameter_block(
                    "imus.imu{}.M_gyro".format(index),
                    imu.M_gyro_Dv, "dimensionless"),
                _parameter_block(
                    "imus.imu{}.A_gyro_accel".format(index),
                    imu.M_accel_gyro_Dv, "(rad/s)/(m/s^2)"),
            ])
        for axis in ("x", "y", "z"):
            attribute = "r{}_i_Dv".format(axis)
            if hasattr(imu, attribute):
                blocks.append(_parameter_block(
                    "imus.imu{}.r{}_i".format(index, axis),
                    getattr(imu, attribute), "metres"))
    return blocks


def analyze(problem, parameter_blocks, *, job, num_threads=1):
    """Analyze the final weighted linearization without applying an update."""
    try:
        import incremental_calibration as incremental
    except ImportError as error:
        raise ObservabilityError(
            "native incremental_calibration module is unavailable") from error

    blocks, selected = _normalize_blocks(parameter_blocks)
    started = time.monotonic()
    native = incremental.analyzeObservability(
        problem, selected, max(1, int(num_threads)))
    elapsed = time.monotonic() - started
    singular_values = np.asarray(native["singular_values"], dtype=float).reshape(-1)
    nullspace = np.asarray(native["nullspace"], dtype=float)
    active_flags = [bool(value) for value in native["selected_active"]]
    rank = int(native["rank"])
    columns = int(native["calibration_dimensions"])
    deficiency = int(native["deficiency"])
    operational_rank = int(native.get("operational_rank", rank))
    operational_deficiency = int(
        native.get("operational_deficiency", deficiency))
    operational_nullspace = np.asarray(
        native.get("operational_nullspace", native["nullspace"]),
        dtype=float,
    )

    parameter_rows = []
    active_offset = 0
    selected_offset = 0
    block_active_ranges = []
    for block in blocks:
        active_dimension = 0
        active = []
        for dimension in block["dimensions"]:
            is_active = active_flags[selected_offset]
            selected_offset += 1
            active.append(is_active)
            if is_active:
                active_dimension += dimension
        row = {
            "name": block["name"],
            "active": any(active),
            "dimension": sum(block["dimensions"]),
            "active_dimension": active_dimension,
            "offset": active_offset if active_dimension else None,
            "units": block["units"],
        }
        if block["coordinate_convention"]:
            row["coordinate_convention"] = block["coordinate_convention"]
        parameter_rows.append(row)
        block_active_ranges.append(
            (block["name"], active_offset, active_offset + active_dimension))
        active_offset += active_dimension

    nullspace_modes = _mode_contributions(
        nullspace, columns, deficiency, block_active_ranges)
    operational_modes = _mode_contributions(
        operational_nullspace, columns, operational_deficiency,
        block_active_ranges)

    largest = singular_values[0] if singular_values.size else None
    smallest = singular_values[-1] if singular_values.size else None
    smallest_observable = (
        singular_values[rank - 1] if rank > 0 and singular_values.size >= rank
        else None
    )
    condition_information = None
    condition_jacobian = None
    if largest is not None and smallest_observable is not None \
            and smallest_observable > 0.0:
        condition_information = float(largest / smallest_observable)
        condition_jacobian = math.sqrt(condition_information)
    spectral_gap = None
    if 0 < rank < singular_values.size and singular_values[rank] > 0.0:
        spectral_gap = float(singular_values[rank - 1] / singular_values[rank])
    operational_spectral_gap = None
    if (0 < operational_rank < singular_values.size
            and singular_values[operational_rank] > 0.0):
        operational_spectral_gap = float(
            singular_values[operational_rank - 1]
            / singular_values[operational_rank])

    if columns == 0 or singular_values.size == 0 or rank == 0:
        status = "no_information"
        quality = "no_information"
    elif deficiency:
        status = "rank_deficient"
        quality = "rank_deficient"
    else:
        status = "full_rank"
        quality = (
            "weakly_observable" if operational_deficiency else "nominal")

    return {
        "schema_version": OBSERVABILITY_SCHEMA_VERSION,
        "kind": "calibration_diagnostics",
        "job": str(job),
        "status": status,
        "quality": quality,
        "state": str(native["state"]),
        "uses_m_estimator": bool(native["uses_m_estimator"]),
        "damping_applied": bool(native["damping_applied"]),
        "scaling": str(native["column_scaling"]),
        "analysis_wall_seconds": elapsed,
        "linearization": {
            "original_error_terms": int(
                native.get("original_error_terms", 0)),
            "analyzed_error_terms": int(
                native.get("linearized_error_terms", 0)),
            "expanded_quadratic_error_terms": int(
                native.get("expanded_quadratic_error_terms", 0)),
            "quadratic_expansion": "segment_sqrt_information",
        },
        "parameter_blocks": parameter_rows,
        "nuisance": {
            "columns": int(native["nuisance_dimensions"]),
            "rank": int(native["qr_rank"]),
            "deficiency": int(native["qr_deficiency"]),
            "qr_tolerance": _finite_or_none(native["qr_tolerance"]),
        },
        "calibration": {
            "columns": columns,
            "rank": rank,
            "deficiency": deficiency,
            "svd_tolerance": _finite_or_none(native["svd_tolerance"]),
            "rank_policy": "native_rank_tol_machine_epsilon",
            "operational_rank": operational_rank,
            "operational_deficiency": operational_deficiency,
            "operational_svd_tolerance": _finite_or_none(
                native.get(
                    "operational_svd_tolerance", native["svd_tolerance"])),
            "operational_eps_svd": _finite_or_none(
                native.get("operational_eps_svd", np.finfo(float).eps)),
            "spectrum_type": "reduced_fisher_information",
            "singular_values": singular_values.tolist(),
            "largest": _finite_or_none(largest) if largest is not None else None,
            "smallest_observable": (
                _finite_or_none(smallest_observable)
                if smallest_observable is not None else None
            ),
            "smallest": (
                _finite_or_none(smallest) if smallest is not None else None
            ),
            "condition_information_observable": (
                _finite_or_none(condition_information)
                if condition_information is not None else None
            ),
            "condition_jacobian_equivalent": (
                _finite_or_none(condition_jacobian)
                if condition_jacobian is not None else None
            ),
            "spectral_gap_at_rank": (
                _finite_or_none(spectral_gap) if spectral_gap is not None else None
            ),
            "operational_spectral_gap": (
                _finite_or_none(operational_spectral_gap)
                if operational_spectral_gap is not None else None
            ),
        },
        "nullspace_modes": nullspace_modes,
        "operationally_truncated_modes": operational_modes,
    }


def write_report(report, path):
    path = Path(path)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            _as_serializable(report),
            stream,
            allow_unicode=True,
            default_flow_style=False,
            sort_keys=False,
        )
    return path


def require_full_rank(report):
    if report.get("status") != "full_rank":
        raise RankDeficiencyError(report)
    return report


def analyze_write_require(problem, parameter_blocks, *, job, path,
                          num_threads=1):
    """Write diagnostics before raising on an analysis or rank failure."""
    try:
        report = analyze(
            problem, parameter_blocks, job=job, num_threads=num_threads)
    except Exception as error:
        report = {
            "schema_version": OBSERVABILITY_SCHEMA_VERSION,
            "kind": "calibration_diagnostics",
            "job": str(job),
            "status": "analysis_failed",
            "failure": {
                "type": type(error).__name__,
                "message": str(error),
            },
            "parameter_blocks": [
                {
                    "name": str(block["name"]),
                    "dimension": sum(
                        int(dv.minimalDimensions())
                        for dv in block.get("design_variables") or ()),
                    "units": str(block.get("units", "model_native")),
                }
                for block in parameter_blocks
            ],
            "calibration": {
                "columns": 0,
                "rank": 0,
                "deficiency": 0,
            },
        }
        write_report(report, path)
        raise
    write_report(report, path)
    require_full_rank(report)
    return report
