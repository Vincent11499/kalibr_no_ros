"""Pinned upstream analysis.py fitting; reports render the saved fit evidence."""

from __future__ import annotations

import math
from pathlib import Path


def line_func(x, slope, intercept):
    return slope * x + intercept


def get_intercept(x, y, slope, evaluation_tau):
    import numpy as np
    from scipy.optimize import curve_fit

    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    if (x.ndim != 1 or y.shape != x.shape or x.size < 2
            or not np.all(np.isfinite(x)) or not np.all(x > 0)
            or not np.all(np.isfinite(y)) or not np.all(y > 0)
            or not math.isfinite(slope)
            or not math.isfinite(evaluation_tau) or evaluation_tau <= 0):
        raise ValueError("fit requires finite positive samples and evaluation tau")
    log_x = np.log(x)
    log_y = np.log(y)
    coefficients, _ = curve_fit(
        line_func,
        log_x,
        log_y,
        bounds=([slope, -np.inf], [slope + 0.001, np.inf]),
    )
    polynomial = np.poly1d(coefficients)

    def fitted(values):
        return np.exp(polynomial(np.log(values)))

    return float(fitted(evaluation_tau)), fitted, [float(value) for value in coefficients]


def generate_prediction(
    tau,
    q_quantization=0.0,
    q_white=0.0,
    q_bias_instability=0.0,
    q_walk=0.0,
    q_ramp=0.0,
):
    import numpy as np

    matrix = np.empty((len(tau), 5))
    matrix[:, 0] = 3 / tau**2
    matrix[:, 1] = 1 / tau
    matrix[:, 2] = 2 * np.log(2) / np.pi
    matrix[:, 3] = tau / 3
    matrix[:, 4] = tau**2 / 2
    parameters = np.array(
        [q_quantization**2, q_white**2, q_bias_instability**2, q_walk**2, q_ramp**2]
    )
    return np.sqrt(matrix.dot(parameters))


def _fit_group(period, values):
    import numpy as np

    breakpoint = int(np.where(period == 10)[0][0])
    white = []
    walk = []
    minimums = []
    white_functions = []
    walk_functions = []
    details = []
    for axis in range(3):
        white_value, white_function, white_coefficients = get_intercept(
            period[:breakpoint], values[:breakpoint, axis], -0.5, 1.0
        )
        walk_value, walk_function, walk_coefficients = get_intercept(
            period, values[:, axis], 0.5, 3.0
        )
        minimum_index = int(np.argmin(values[:, axis]))
        white.append(white_value)
        walk.append(walk_value)
        minimums.append((float(values[minimum_index, axis]), minimum_index))
        white_functions.append(white_function)
        walk_functions.append(walk_function)
        details.append(
            {
                "white_noise_at_1s": white_value,
                "white_fit_slope": white_coefficients[0],
                "white_fit_log_intercept": white_coefficients[1],
                "random_walk_at_3s": walk_value,
                "random_walk_fit_slope": walk_coefficients[0],
                "random_walk_fit_log_intercept": walk_coefficients[1],
                "minimum_allan_deviation": minimums[-1][0],
                "minimum_tau_s": float(period[minimum_index]),
            }
        )
    return {
        "white": white,
        "walk": walk,
        "minimums": minimums,
        "white_functions": white_functions,
        "walk_functions": walk_functions,
        "details": details,
    }


def analyze(curve_path: Path, topic: str, update_rate: float) -> tuple[dict, dict]:
    import numpy as np

    curve = np.loadtxt(curve_path)
    expected_tau = np.arange(1, 10000, dtype=np.float64) * 0.1
    if (curve.shape != (9999, 7) or not np.all(np.isfinite(curve))
            or not np.all(curve > 0)
            or not np.allclose(curve[:, 0], expected_tau, rtol=0, atol=1e-10)):
        raise ValueError("expected positive finite 9999 by 7 Allan curve on the fixed tau grid")
    if not math.isfinite(update_rate) or update_rate <= 0:
        raise ValueError("update_rate must be finite and positive")
    if not isinstance(topic, str) or not topic or any(c.isspace() for c in topic):
        raise ValueError("topic must be a non-empty string without whitespace")
    period = curve[:, 0]
    acceleration = curve[:, 1:4]
    rotation_rate = curve[:, 4:7]
    acceleration_fit = _fit_group(period, acceleration)
    gyroscope_fit = _fit_group(period, rotation_rate)
    parameters = {
        "accelerometer_noise_density": float(max(acceleration_fit["white"])),
        "accelerometer_random_walk": float(max(acceleration_fit["walk"])),
        "gyroscope_noise_density": float(max(gyroscope_fit["white"])) * math.pi / 180.0,
        "gyroscope_random_walk": float(max(gyroscope_fit["walk"])) * math.pi / 180.0,
        "rostopic": topic,
        "update_rate": float(update_rate),
    }
    details = {
        "fit_contract": {
            "white_tau_s": "[0.1, 10.0)",
            "white_slope_bounds": [-0.5, -0.499],
            "white_evaluation_tau_s": 1.0,
            "random_walk_tau_s": "all 9999 points",
            "random_walk_slope_bounds": [0.5, 0.501],
            "random_walk_evaluation_tau_s": 3.0,
            "axis_aggregation": "maximum of x/y/z",
        },
        "accelerometer": dict(zip(("x", "y", "z"), acceleration_fit["details"])),
        "gyroscope_deg_per_s": dict(zip(("x", "y", "z"), gyroscope_fit["details"])),
    }
    return parameters, details
