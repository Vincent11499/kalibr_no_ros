"""Small deterministic image helpers shared by reports and verification."""

import numpy as np


def uniform_sample(items, maximum):
    """Return at most *maximum* items at stable, evenly spaced indices."""
    if len(items) <= maximum:
        return list(items)
    indices = np.linspace(0, len(items) - 1, maximum).round().astype(int)
    return [items[int(index)] for index in indices]


def draw_image_label(image, label):
    """Draw the standard readable label used by exported evidence images."""
    import cv2

    font, scale, thickness, margin = cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2, 8
    (width, height), baseline = cv2.getTextSize(
        label, font, scale, thickness)
    cv2.rectangle(
        image,
        (0, 0),
        (
            min(image.shape[1] - 1, width + margin * 2),
            min(image.shape[0] - 1, height + baseline + margin * 2),
        ),
        (0, 0, 0),
        cv2.FILLED,
    )
    cv2.putText(
        image,
        label,
        (margin, margin + height),
        font,
        scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )
