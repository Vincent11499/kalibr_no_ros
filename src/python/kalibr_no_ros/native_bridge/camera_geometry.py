"""Construct native camera geometry from complete public calibration values."""

import numpy as np


def native_camera_model(camera):
    """Select the native model class used by fixed-parameter detection."""
    import aslam_cv_backend as acvb

    from ..results.evaluation import camera_geometry

    model = camera_geometry(camera)["model"]
    models = {
        "pinhole-radtan": acvb.DistortedPinhole,
        "pinhole-equi": acvb.EquidistantPinhole,
        "pinhole-equidistant": acvb.EquidistantPinhole,
        "pinhole-fov": acvb.FovPinhole,
        "omni-none": acvb.Omni,
        "omni-radtan": acvb.DistortedOmni,
        "eucm-none": acvb.ExtendedUnified,
        "ds-none": acvb.DoubleSphere,
    }
    if model == "pinhole-radtan5":
        import kalibr_radtan5 as extension
        extension.install()
        return extension.PinholeRadtan5
    if model == "pinhole-radtan8":
        import kalibr_radtan8 as extension
        extension.install()
        return extension.PinholeRadtan8
    if model == "pinhole-opencv-fisheye":
        import kalibr_opencv_fisheye as extension
        extension.install()
        return extension.PinholeOpenCvFisheye
    try:
        return models[model]
    except KeyError as error:
        raise ValueError(
            "fixed-camera validation does not support model {}".format(
                model)) from error


def create_camera_geometry(camera):
    """Build native projection and distortion using the supplied K/D values."""
    model = native_camera_model(camera)
    distortion = model.distortionType()
    distortion.setParameters(np.asarray(
        camera["distortion_coeffs"], dtype=float))
    projection = model.projectionType(*(
        list(map(float, camera["intrinsics"]))
        + list(map(int, camera["resolution"]))
        + [distortion]))
    return model.geometry(projection)
