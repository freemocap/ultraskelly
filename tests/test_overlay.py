import numpy as np
import pytest

pytest.importorskip("cv2")  # OpenCV is in the hardware extra; this runs where it is installed

from ultraskelly.ui.overlay import draw_skeleton  # noqa: E402


def person_with_spread_keypoints():
    keypoints = np.zeros((17, 3), np.float32)
    keypoints[:, 0] = np.arange(17) * 30 + 40  # Keypoints 30 px apart along y = 60
    keypoints[:, 1] = 60
    keypoints[:, 2] = 0.9
    keypoints[16] = [600, 450, 0.1]  # Too uncertain to draw
    return keypoints


def test_skeleton_draws_confident_keypoints_in_place():
    frame = np.zeros((480, 640, 3), np.uint8)

    draw_skeleton(frame, person_with_spread_keypoints())

    assert frame[60, 40].any() and frame[60, 40 + 15 * 30].any()  # Keypoints drawn
    assert not frame[450, 600].any()  # Low-confidence keypoint skipped
