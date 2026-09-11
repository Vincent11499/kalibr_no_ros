"""Report presentation from final solver evidence, without native solver imports.

The camera-system, target-frame poses and angular-error definitions follow
kalibr_camera_calibration.CameraUtils. Only presentation is changed: all plots
use retained observations and are embedded into the HTML/PDF documents.
"""

from __future__ import annotations

import base64
import html
import io
import textwrap

import numpy as np

from .evaluation import ReportingError, camera_geometry, corner_residual, merged_cameras, vector
from .version import VERSION


_BLUE = "#2563a6"
_INK = "#183249"
_MUTED = "#617486"


def _number(value):
    if value is None:
        return "unavailable"
    if isinstance(value, (float, np.floating)):
        return "{:.6g}".format(value) if np.isfinite(value) else "unavailable"
    return str(value)


def _matrix(value):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        return None
    if not np.allclose(result[3], [0., 0., 0., 1.], atol=1e-9, rtol=0.):
        return None
    rotation = result[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0.) or np.linalg.det(rotation) <= 0:
        return None
    return result


def camera_frames(cameras):
    """Return T_first_camera; baseline input is T_current_previous."""
    result = []
    previous = np.eye(4)
    for index, camera in enumerate(cameras):
        if index:
            baseline = _matrix(camera.get("T_cn_cnm1"))
            if previous is None or baseline is None:
                previous = None
            else:
                previous = previous @ np.linalg.inv(baseline)
        result.append((camera["id"], previous))
    return result


def observation_rays(camera, pixels):
    """Observed unit rays, including fisheye alpha and rational distortion.

    Angles are acos(ray.z) and atan2(ray.y, ray.x), as in native Kalibr.
    An unsupported inverse model is reported as unavailable, never approximated
    with the pinhole formula or with a predicted target-point ray.
    """
    import cv2

    geometry = camera_geometry(camera)
    pixels = np.asarray(pixels, dtype=float).reshape(-1, 1, 2).copy()
    if geometry["alpha"]:
        fx, fy = geometry["K"][0, 0], geometry["K"][1, 1]
        cy = geometry["K"][1, 2]
        pixels[:, 0, 0] -= fx * geometry["alpha"] * (pixels[:, 0, 1] - cy) / fy
    criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-12)
    if geometry["family"] == "fisheye":
        # The supported OpenCV 4.5 Python binding does not expose criteria for
        # fisheye. Its native Newton inverse is distinct from radtan's default
        # five fixed-point iterations, for which explicit criteria are needed.
        normalized = cv2.fisheye.undistortPoints(
            pixels, geometry["K_no_skew"], geometry["D"])
    else:
        normalized = cv2.undistortPointsIter(
            pixels, geometry["K_no_skew"], geometry["D"], None, None, criteria)
    normalized = normalized.reshape(-1, 2)
    # OpenCV returns a large negative sentinel for a failed fisheye inverse.
    valid = np.all(np.isfinite(normalized), axis=1) & np.all(np.abs(normalized) < 1e6, axis=1)
    rays = np.c_[normalized, np.ones(len(normalized))]
    rays /= np.linalg.norm(rays, axis=1)[:, None]
    rays[~valid] = np.nan
    return rays


def camera_observations(camera):
    pixels, residuals, indices, frame_rms = [], [], [], []
    for index, frame in enumerate(frame for frame in camera.get("frames", []) if frame.get("used")):
        norms = []
        for corner in frame.get("corners", []):
            if not corner.get("used", True):
                continue
            point = vector(corner.get("measurement_px"), 2)
            error = corner_residual(corner)
            if point is None or error is None:
                continue
            pixels.append(point)
            residuals.append(error)
            indices.append(index)
            norms.append(float(error @ error))
        frame_rms.append(np.sqrt(np.mean(norms)) if norms else np.nan)
    return (np.asarray(pixels).reshape(-1, 2), np.asarray(residuals).reshape(-1, 2),
            np.asarray(indices), np.asarray(frame_rms))


def _figure(title, size=(10.5, 7.4)):
    from matplotlib.figure import Figure

    figure = Figure(figsize=size, facecolor="white")
    figure.suptitle(textwrap.fill(title, 42 if size[0] < 9 else 68), x=.075, y=.97,
                   ha="left", fontsize=18, color=_INK, weight="bold")
    figure.subplots_adjust(left=.085, right=.9, bottom=.2, top=.86, wspace=.32, hspace=.52)
    figure.text(.075, .025, "Kalibr no-ROS  |  v{}  |  Final retained observations".format(VERSION),
                fontsize=8, color=_MUTED)
    return figure


def _unavailable(title, reason):
    figure = _figure(title)
    axis = figure.subplots()
    axis.set_axis_off()
    axis.text(.03, .72, "Unavailable", fontsize=22, color=_MUTED, transform=axis.transAxes)
    axis.text(.03, .58, textwrap.fill(reason, 90), fontsize=12, color=_INK,
              va="top", linespacing=1.6, transform=axis.transAxes)
    return figure


def _coordinate_frame(axis, transform, size, name=None, alpha=1.):
    origin = transform[:3, 3]
    for index, color in enumerate(("#cf554c", "#3d9968", "#397abb")):
        endpoint = origin + size * transform[:3, index]
        axis.plot(*np.array([origin, endpoint]).T, color=color, linewidth=1.5, alpha=alpha)
    if name:
        axis.text(*origin, " " + str(name), color=_INK, fontsize=10)


def _equal_3d(axis, points, padding):
    points = np.asarray(points).reshape(-1, 3)
    midpoint = (points.max(axis=0) + points.min(axis=0)) / 2.
    extent = max(float(np.ptp(points, axis=0).max()) / 2. + padding, .05)
    axis.set(xlim=(midpoint[0] - extent, midpoint[0] + extent),
             ylim=(midpoint[1] - extent, midpoint[1] + extent),
             zlim=(midpoint[2] - extent, midpoint[2] + extent),
             xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
    axis.set_box_aspect((1, 1, 1))
    axis.view_init(elev=23, azim=-58)


def _rig_plot(cameras):
    title = "Camera system"
    frames = camera_frames(cameras)
    available = [(name, transform) for name, transform in frames if transform is not None]
    if not available:
        return title, "No camera extrinsic evidence was saved.", _unavailable(title, "No camera extrinsic evidence was saved.")
    figure = _figure(title)
    axis = figure.add_subplot(111, projection="3d")
    origins = np.array([transform[:3, 3] for _, transform in available])
    lengths = np.linalg.norm(np.diff(origins, axis=0), axis=1)
    positive = lengths[lengths > 1e-12]
    size = .35 * float(positive.min()) if len(positive) else .1
    for name, transform in available:
        _coordinate_frame(axis, transform, size, name)
    if len(origins) > 1:
        axis.plot(*origins.T, color=_MUTED, linestyle="--", linewidth=1.)
    _equal_3d(axis, origins, size * 1.5)
    caption = "Camera origins and axes expressed in {}. Red = x, green = y, blue = z; lengths in metres.".format(cameras[0]["id"])
    missing = [name for name, transform in frames if transform is None]
    if missing:
        caption += " Missing baseline chain: {}.".format(", ".join(missing))
    return title, caption, figure


def _pose_plot(camera):
    title = "{}: Estimated poses".format(camera["id"])
    frames = [frame for frame in camera.get("frames", []) if frame.get("used")]
    frames.sort(key=lambda frame: (frame.get("source_timestamp_ns") or 0, frame.get("frame_id", "")))
    transforms, target_points = [], []
    for frame in frames:
        transform = _matrix(frame.get("T_camera_target"))
        if transform is not None:
            transforms.append(np.linalg.inv(transform))
        for corner in frame.get("corners", []):
            point = vector(corner.get("target_xyz_m"), 3)
            if point is not None:
                target_points.append(point)
    caption = ("Camera poses in target coordinates, computed by inverting the final T_camera_target. "
               "The line follows source timestamp order; it is not a continuous-time trajectory estimate.")
    if not transforms:
        reason = "No final camera-to-target pose evidence was saved. Re-evaluation does not rerun pose estimation."
        return title, reason, _unavailable(title, reason)
    figure = _figure(title)
    axis = figure.add_subplot(111, projection="3d")
    origins = np.array([transform[:3, 3] for transform in transforms])
    size = max(float(np.ptp(origins, axis=0).max()) * .035, .025)
    axis.plot(*origins.T, color=_BLUE, linewidth=1., marker=".", markersize=4, label="Camera origins")
    selected = np.unique(np.linspace(0, len(transforms) - 1, min(30, len(transforms))).astype(int))
    for index in selected:
        _coordinate_frame(axis, transforms[index], size, alpha=.65)
    all_points = origins
    if target_points:
        points = np.unique(np.asarray(target_points), axis=0)
        axis.scatter(*points.T, s=2, c="#94a3af", alpha=.5, label="Target corners")
        all_points = np.r_[origins, points]
    _equal_3d(axis, all_points, size * 1.5)
    axis.legend(loc="upper left", fontsize=9)
    caption += " All {} saved poses are shown; orientation axes are drawn for {} poses for readability.".format(len(transforms), len(selected))
    if len(transforms) < len(frames):
        caption += " {} used frames have no valid saved pose.".format(len(frames) - len(transforms))
    return title, caption, figure


def _angular_plot(camera, pixels, residuals, kind):
    title = "{}: {} error".format(camera["id"], kind.capitalize())
    try:
        if not len(pixels):
            raise ReportingError("No final corner measurements and residuals were saved.")
        rays = observation_rays(camera, pixels)
        valid = np.all(np.isfinite(rays), axis=1)
        if not np.any(valid):
            raise ReportingError("No observed pixels could be inverted to finite camera rays.")
    except (ReportingError, ValueError, RuntimeError) as error:
        return title, str(error), _unavailable(title, str(error))
    rays, residuals = rays[valid], residuals[valid]
    if kind == "polar":
        angles = np.degrees(np.arccos(np.clip(rays[:, 2], -1., 1.)))
        definition = "Polar angle = acos(ray.z), measured from the camera +z optical axis."
    else:
        angles = np.degrees(np.arctan2(rays[:, 1], rays[:, 0]))
        definition = "Azimuthal angle = atan2(ray.y, ray.x), around the camera +z optical axis."
    figure = _figure(title)
    axes = figure.subplots(1, 2)
    axes[0].scatter(angles, np.linalg.norm(residuals, axis=1), s=4, alpha=.35, color=_BLUE, rasterized=True)
    axes[0].set(xlabel="{} angle [deg]".format(kind.capitalize()), ylabel="2D reprojection error [px]")
    axes[1].hist(angles, bins=30, color=_BLUE, alpha=.8)
    axes[1].set(xlabel="{} angle [deg]".format(kind.capitalize()), ylabel="Retained corners")
    for axis in axes:
        axis.grid(alpha=.18)
        axis.set_axisbelow(True)
    caption = definition + " Angles use observed-pixel inverse projection; error is the norm of the final pixel residual."
    if not np.all(valid):
        caption += " {} failed inverse projections are omitted; no zero values are substituted.".format(int((~valid).sum()))
    return title, caption, figure


def _reprojection_plot(camera, data, summary):
    pixels, residuals, indices, frame_rms = data
    title = "{}: Reprojection errors".format(camera["id"])
    if not len(pixels):
        values = [frame.get("rms_px") for frame in summary.get("frames", [])]
        values = [value for value in values if value is not None]
        reason = "Corner-level evidence is unavailable."
        if not values:
            return title, reason, _unavailable(title, reason)
        figure = _figure(title)
        axis = figure.subplots()
        axis.plot(values, ".-", color=_BLUE)
        axis.set(xlabel="Final used frame index", ylabel="2D RMS [px]")
        return title, reason + " Only the saved per-frame RMS summary is shown.", figure
    figure = _figure(title, size=(10.5, 8.3))
    axes = figure.subplots(2, 2)
    scatter = axes[0, 0].scatter(*pixels.T, c=indices, s=3, alpha=.45, cmap="viridis", rasterized=True)
    resolution = camera.get("resolution")
    if resolution:
        axes[0, 0].set(xlim=(0, resolution[0]), ylim=(resolution[1], 0))
    else:
        axes[0, 0].invert_yaxis()
    axes[0, 0].set(xlabel="Image x [px]", ylabel="Image y [px]", title="Used corner coverage")
    axes[0, 0].set_aspect("equal", adjustable="box")
    axes[0, 1].scatter(*residuals.T, c=indices, s=3, alpha=.45, cmap="viridis", rasterized=True)
    axes[0, 1].set(xlabel="Residual x [px]", ylabel="Residual y [px]", title="Measurement - prediction")
    axes[0, 1].set_aspect("equal", adjustable="datalim")
    figure.colorbar(scatter, ax=axes[0, 1], label="Used frame index", fraction=.045, pad=.035)
    norms = np.linalg.norm(residuals, axis=1)
    axes[1, 0].hist(norms, bins=40, color=_BLUE, alpha=.8)
    axes[1, 0].set(xlabel="2D reprojection error [px]", ylabel="Retained corners", title="Corner error distribution")
    axes[1, 1].plot(frame_rms, ".-", color=_BLUE, linewidth=.8, markersize=4)
    axes[1, 1].set(xlabel="Final used frame index", ylabel="2D RMS [px]", title="Per-frame RMS")
    for axis in axes.flat:
        axis.grid(alpha=.18)
        axis.set_axisbelow(True)
    return title, "{} final retained corners with measurements and residuals. 2D corner RMS = sqrt(mean(dx² + dy²)); no discarded observations are included.".format(len(pixels)), figure


def report_figures(metrics, artifacts=None, calibration=None):
    cameras = merged_cameras(artifacts or {}, calibration or {})
    if not cameras:
        cameras = [dict(camera, id=name, frames=[]) for name, camera in metrics.get("cameras", {}).items()]
    yield _rig_plot(cameras)
    for camera in cameras:
        yield _pose_plot(camera)
        data = camera_observations(camera)
        for kind in ("polar", "azimuthal"):
            yield _angular_plot(camera, data[0], data[1], kind)
        yield _reprojection_plot(camera, data, metrics.get("cameras", {}).get(camera["id"], {}))
    for imu_id, imu in metrics.get("imus", {}).items():
        for kind, values in imu.items():
            bias = values.get("bias_spline")
            if not bias:
                continue
            samples = [sample for sample in bias["time_series"] if sample["solver_timestamp_s"] is not None]
            if not samples:
                continue
            origin = min(sample["solver_timestamp_s"] for sample in samples)
            times = [sample["solver_timestamp_s"] - origin for sample in samples]
            trajectory = np.asarray([sample["value"] if sample["value"] is not None else [np.nan] * 3 for sample in samples])
            title = "{} {} time-varying bias spline".format(imu_id, kind)
            figure = _figure(title)
            axis = figure.subplots()
            for index, name in enumerate(("x", "y", "z")):
                axis.plot(times, trajectory[:, index], label=name, linewidth=1.)
            axis.set(xlabel="Solver time since {:.9f} [s]".format(origin), ylabel="Bias [{}]".format(bias["unit"]))
            axis.legend()
            axis.grid(alpha=.2)
            yield title, "Actual bias spline samples at retained IMU timestamps. Missing samples remain gaps.", figure


def _html_table(headers, rows):
    head = "".join("<th>{}</th>".format(html.escape(str(value))) for value in headers)
    body = "".join("<tr>" + "".join("<td>{}</td>".format(html.escape(_number(value))) for value in row) + "</tr>" for row in rows)
    return '<div class="table-scroll"><table><thead><tr>{}</tr></thead><tbody>{}</tbody></table></div>'.format(head, body)


def _summary_sections(metrics, assessment, calibration):
    """Shared table content makes the HTML and PDF summaries consistent."""
    rows = []
    for name, camera in metrics.get("cameras", {}).items():
        error = camera.get("reprojection", {})
        rows.append([name, camera.get("used_frames"), camera.get("selected_frames"), error.get("count"),
                     error.get("rms_px"), error.get("p95"), error.get("max")])
    yield "Calibration overview", ["Camera", "Used frames", "Selected", "Corners", "RMS [px]", "P95 [px]", "Max [px]"], rows
    if metrics.get("observability"):
        observability = metrics["observability"]
        yield "Local observability", ["Parameter", "Value"], [
            ["Quality", observability.get("quality")],
            ["Native numerical rank", "{} / {}".format(observability.get("rank"), observability.get("columns"))],
            ["Operational rank", "{} / {}".format(observability.get("operational_rank"), observability.get("columns"))],
            ["Interpretation", "Full numerical rank can still have weakly constrained directions. This is not a hardware accuracy guarantee."],
        ]
    if metrics.get("stereo_pairs"):
        rows = []
        for name, pair in metrics["stereo_pairs"].items():
            alignment = pair.get("alignment") or {}
            baseline = pair.get("baseline_m")
            rows.append([name, baseline * 1000. if baseline is not None else None,
                         alignment.get("mean_abs_px", alignment.get("mean")), alignment.get("rms_px"),
                         alignment.get("count"), pair.get("status")])
        yield "Stereo alignment", ["Pair", "Baseline [mm]", "Mean abs [px]", "RMS [px]", "Corners", "Status"], rows
    result_cameras = (calibration or {}).get("cameras", [])
    results = {camera.get("id", "cam{}".format(index)): camera for index, camera in enumerate(result_cameras)}
    previous_ids = {camera.get("id", "cam{}".format(index)): result_cameras[index - 1].get("id", "cam{}".format(index - 1))
                    for index, camera in enumerate(result_cameras) if index}
    for name, camera in metrics.get("cameras", {}).items():
        final = results.get(name, {})
        rows = [["Model", camera.get("model")], ["Resolution [px]", camera.get("resolution")]]
        intrinsics = camera.get("intrinsics") or []
        pinhole = str(camera.get("model", "")).startswith("pinhole")
        for index, value in enumerate(intrinsics):
            label = ("fu", "fv", "cu", "cv", "alpha")[index] if pinhole and index < 5 else "projection[{}]".format(index)
            rows.append([label, value])
        for index, value in enumerate(camera.get("distortion_coeffs") or []):
            rows.append(["distortion[{}]".format(index), value])
        if "timeshift_cam_imu_s" in camera:
            rows.append(["Camera/IMU time shift [s]", camera["timeshift_cam_imu_s"]])
        yield name + " · Camera parameters", ["Parameter", "Value"], rows
        shutter = camera.get("shutter")
        if shutter:
            rows = [["Shutter model", shutter["type"]],
                         ["Signed line delay [s/row]", shutter["line_delay_s"]],
                         ["Reference row [px]", shutter["reference_row_px"]],
                         ["First-to-last row span [s]", shutter["first_to_last_row_span_s"]],
                         ["Line delay estimated", shutter["estimated"]],
                         ["Frame time shift reference", "Effective sample at reference row"]]
            if "line_delay_std_s" in shutter:
                rows.append(["Local line-delay std [s/row]", shutter["line_delay_std_s"]])
            yield name + " · Rolling shutter timing", ["Parameter", "Value"], rows
        for field, source in (("T_cn_cnm1", "previous camera"), ("T_cam_imu", "IMU")):
            transform = _matrix(final.get(field))
            if transform is not None:
                source = final.get("from_camera", final.get("source_camera", previous_ids.get(name, source))) if field == "T_cn_cnm1" else source
                yield "{} → {} · {} (translation in m)".format(source, name, field), ["", "x", "y", "z", "translation"], [[str(index), *row] for index, row in enumerate(transform)]
    if assessment.get("rules"):
        yield "Configured assessment rules", ["Metric", "Value", "Minimum", "Maximum", "Status"], [
            [rule["metric"], rule.get("value"), rule.get("min"), rule.get("max"), rule.get("status")]
            for rule in assessment["rules"]]
    for imu_id, imu in metrics.get("imus", {}).items():
        yield imu_id + " · IMU residuals", ["Kind", "Samples", "Vector RMS", "Unit"], [
            [kind, values.get("count"), values.get("rms"), values.get("unit")] for kind, values in imu.items()]


def _pdf_table_page(title, headers, rows, subtitle):
    figure = _figure(title, size=(8.27, 11.69))
    figure.text(.085, .9, textwrap.fill(subtitle, 85), color=_MUTED, fontsize=10, va="top", linespacing=1.4)
    axis = figure.add_axes([.085, .13, .85, .69])
    axis.set_axis_off()
    widths = [.44, .56] if len(headers) == 2 else None
    text_rows = [["\n".join(textwrap.wrap(_number(value), width=42 if len(headers) == 2 else 19)) for value in row] for row in rows]
    table = axis.table(cellText=text_rows or [["unavailable"] + [""] * (len(headers) - 1)],
                       colLabels=headers, colWidths=widths, loc="upper center", cellLoc="left", colLoc="left")
    table.auto_set_font_size(False)
    table.set_fontsize(10 if len(headers) <= 5 else 8.5)
    for (row, column), cell in table.get_celld().items():
        line_count = max((value.count("\n") + 1 for value in text_rows[row - 1]), default=1) if row else 1
        cell.set_height(.04 * max(1.4, line_count))
        cell.set_edgecolor("#dce5ed")
        cell.set_linewidth(.5)
        cell.PAD = .12
        cell.set_text_props(color=_INK)
        cell.set_facecolor("#e8f0f7" if row == 0 else "#f6f9fb" if row % 2 else "white")
        if row == 0:
            cell.set_text_props(weight="bold")
    return figure


_STYLE = """
:root{color-scheme:light;--ink:#183249;--muted:#617486;--line:#dce5ed;--blue:#2563a6}
*{box-sizing:border-box}body{margin:0;background:#f3f6f9;color:var(--ink);font:16px/1.65 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1180px;margin:auto;padding:40px 32px 72px}header{padding:30px 0 34px;border-bottom:1px solid var(--line);margin-bottom:32px}
.eyebrow{font-size:12px;letter-spacing:.14em;color:var(--blue);font-weight:700}h1{font-size:38px;line-height:1.2;margin:10px 0 18px}h2{font-size:25px;margin:0 0 20px;line-height:1.3}
p{margin:12px 0;color:var(--muted)}.badges{display:flex;gap:10px;flex-wrap:wrap}.badge{background:#e5edf5;border-radius:6px;padding:6px 12px;font-size:13px}
section,figure{margin:28px 0;padding:30px;background:white;border:1px solid var(--line);border-radius:12px;box-shadow:0 2px 8px #18324905}
.table-scroll{overflow:auto}table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;font-size:14px}td,th{text-align:left;padding:13px 16px;border-bottom:1px solid var(--line);vertical-align:top}
th{font-weight:600;background:#edf3f8;white-space:nowrap}tr:last-child td{border-bottom:0}td:first-child{font-weight:500}tbody tr:nth-child(even){background:#fafcfd}
img{display:block;width:100%;height:auto}figcaption{padding:16px 8px 0;color:var(--muted);font-size:14px}a{color:var(--blue);text-underline-offset:3px}summary{cursor:pointer;font-weight:600}
.stereo-comparison{margin:28px 0;padding:20px;border:1px solid var(--line);border-radius:10px}.stereo-comparison figure{margin:12px 0;padding:12px;box-shadow:none}.stereo-comparison h3{margin:0 0 16px}.gallery{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:20px}.gallery figure{margin:0;padding:12px}.gallery figcaption{overflow-wrap:anywhere}footer{padding:22px 0;color:var(--muted);font-size:13px}
@media(max-width:760px){main{padding:20px 14px}h1{font-size:30px}section,figure{padding:18px}.gallery{grid-template-columns:1fr}td,th{padding:10px}}
@media print{body{background:white}main{max-width:none;padding:0}section,figure{break-inside:avoid;box-shadow:none}details{display:none}}
"""


def stereo_comparison_page(original, rectified, title):
    """One landscape page, preserving original pixels for zoomable PDF images."""
    from matplotlib.figure import Figure
    figure = Figure(figsize=(11.69, 8.27))
    figure.text(.06, .95, title, fontsize=17, weight='bold', color=_INK)
    for pixels, bounds, label in (
            (original, [.06, .52, .88, .34], 'Original stereo pair'),
            (rectified, [.06, .10, .88, .34], 'Epipolar alignment')):
        axis = figure.add_axes(bounds)
        axis.imshow(pixels, interpolation='none')
        axis.set_title(label, loc='left', fontsize=11, pad=10)
        axis.set_axis_off()
    figure.text(.06, .035, 'Same source pair above and below. Green guide lines: 3 source-image pixels.', fontsize=9, color=_MUTED)
    return figure


def render_reports(metrics, assessment, files, *, artifacts=None, calibration=None, output_dir=None):
    """Return a self-contained HTML document and a paginated PDF byte string."""
    import matplotlib
    from matplotlib.backends.backend_pdf import PdfPages
    import random
    candidates = sorted(path for path in files if "visualizations/" in path
                        and path.rsplit('/', 1)[-1].startswith('rectified_')
                        and path.rsplit('/', 1)[0] + '/' + path.rsplit('/', 1)[1].replace('rectified_', 'original_', 1) in files
                        and path.endswith((".jpg", ".png")))
    images = sorted(random.Random(0).sample(candidates, min(5, len(candidates))))

    status = assessment.get("status", "unavailable")
    grade = assessment.get("reference_grading", {}).get("status", "unavailable")
    subtitle = "Final used observations; 2D corner RMS in pixels. Missing evidence is unavailable."
    if any(c.get("shutter") for c in metrics.get("cameras", {}).values()):
        subtitle += " Rolling shutter: reprojection uses per-corner times; alignment and images use optical rectification without row-time compensation."
    document = ['<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">',
                '<title>Calibration report</title><style>' + _STYLE + '</style></head><body><main>',
                '<header><div class="eyebrow">KALIBR NO-ROS · V{}</div><h1>Calibration report</h1><div class="badges">'.format(VERSION),
                '<span class="badge">{}</span><span class="badge">Assessment: {}</span><span class="badge">Reference grade: {} · display only</span></div><p>{}</p></header>'.format(
                    html.escape(str(metrics.get("calibration_type"))), html.escape(status), html.escape(grade), subtitle)]
    buffer = io.BytesIO()
    style = {"font.family": "DejaVu Sans", "font.size": 10, "axes.labelcolor": _INK,
             "axes.edgecolor": "#aab8c4", "xtick.color": _MUTED, "ytick.color": _MUTED,
             "axes.titlepad": 12, "pdf.fonttype": 42}
    with matplotlib.rc_context(style), PdfPages(buffer) as pdf:
        pdf.infodict().update(Title="Kalibr no-ROS calibration report", Author="Kalibr no-ROS", Subject=subtitle)
        for title, headers, rows in _summary_sections(metrics, assessment, calibration):
            document.append("<section><h2>{}</h2>{}</section>".format(html.escape(title), _html_table(headers, rows)))
            for start in range(0, max(1, len(rows)), 16):
                page_title = title if start == 0 else title + " (continued)"
                pdf_title = page_title.replace("→", "to").replace("·", "|")
                page = _pdf_table_page(pdf_title, headers, rows[start:start + 16], subtitle + " Assessment: {}.".format(status))
                pdf.savefig(page)
                page.clear()
        for title, caption, figure in report_figures(metrics, artifacts, calibration):
            # Reserve a separate footer for a readable description on every PDF plot.
            wrapped = textwrap.fill(caption.replace("²", "^2"), 122)
            figure.text(.075, .11, wrapped, fontsize=8, color=_MUTED, va="top", linespacing=1.3)
            pdf.savefig(figure)
            raster = io.BytesIO()
            figure.savefig(raster, format="png", dpi=125)
            encoded = base64.b64encode(raster.getvalue()).decode("ascii")
            document.append('<figure><h2>{}</h2><img loading="lazy" alt="{}" src="data:image/png;base64,{}"><figcaption>{}</figcaption></figure>'.format(
                html.escape(title), html.escape(title, quote=True), encoded, html.escape(caption)))
            figure.clear()
        if output_dir is not None:
            from .reporting import _child
            import matplotlib.image as mpimg
            for number, path in enumerate(images, 1):
                original = path.rsplit('/', 1)[0] + '/' + path.rsplit('/', 1)[1].replace('rectified_', 'original_', 1)
                figure = stereo_comparison_page(
                    mpimg.imread(_child(output_dir, original)),
                    mpimg.imread(_child(output_dir, path)),
                    'Stereo comparison {} / {} - {}'.format(number, len(images), path.rsplit('/', 1)[-1]))
                pdf.savefig(figure)
                figure.clear()
    if images:
        document.append('<section><h2>Epipolar alignment · original and rectified</h2><p>Up to five random samples, sorted by frame order. Each group shows original stereo images above the rectified pair. Green guide lines: 3 pixels.</p><div class="stereo-comparisons">')
        for number, path in enumerate(images, 1):
            original = path.rsplit('/', 1)[0] + '/' + path.rsplit('/', 1)[1].replace('rectified_', 'original_', 1)
            document.append('<article class="stereo-comparison"><h3>Sample {}</h3>'.format(number))
            for image_path, label in ((original, 'Original stereo pair'), (path, 'Epipolar alignment')):
                safe = html.escape(image_path, quote=True)
                document.append('<figure><figcaption>{0}</figcaption><a href="{1}"><img loading="lazy" src="{1}" alt="{0}"></a></figure>'.format(label, safe))
            document.append('</article>')
        document.append('</div></section>')
    if files:
        links = "".join('<li><a href="{}">{}</a></li>'.format(html.escape(path, quote=True), html.escape(path))
                        for path in files if not path.endswith(('.jpg', '.png', '.jpeg')))
        document.append('<section><details><summary>Saved files</summary><ul>{}</ul></details></section>'.format(links))
    for problem in metrics.get("output_diagnostics", []):
        document.append('<section><h2>Unavailable output</h2><p>{}</p></section>'.format(html.escape(str(problem))))
    document.append('<footer>Pixel reprojection and stereo alignment describe the saved observations. They do not by themselves establish absolute geometric accuracy or production acceptance.</footer></main></body></html>')
    return "\n".join(document), buffer.getvalue()
