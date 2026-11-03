"""The head frame H, and where the ZED's rectified left camera sits in it.

H is fixed to the skull and turns with the head: origin midway between the eye-socket
centres, x forward out of the face, y to the skull's left, z up. The camera bracket's
design will give each sensor's pose in H; until then the ZED is assumed level, its left
lens at the left eye socket, half a baseline from the origin.
"""
import numpy as np

from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification

HEAD_FRAME = "H: origin midway between the eye-socket centres, x forward, y to the skull's left, z up"
# Optical axes (x image right, y image down, z out of the lens) in H, for a camera that
# looks straight out of the face: right is -y, down is -z, forward is +x.
OPTICAL_TO_HEAD = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def nominal_camera_to_head(rectification: StereoRectification) -> dict:
    """The rectified left camera's pose in H, nominal: `rotation` and `translation_m` take a
    point in the camera's frame into H."""
    rectified_to_left = np.array(rectification.left_rotation).T  # Rectified frame to the left camera's own
    rotation = OPTICAL_TO_HEAD @ rectified_to_left
    return {
        "rotation": [[round(float(value), 6) for value in row] for row in rotation],
        "translation_m": [0.0, rectification.baseline / 2, 0.0],
        "head_frame": HEAD_FRAME,
        "status": "nominal",
    }
