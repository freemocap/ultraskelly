import numpy as np
import pytest

pytest.importorskip("cv2")  # OpenCV is in the hardware extra; this runs where it is installed

from ultraskelly.ui.overlay import draw_tracking_overlay  # noqa: E402


def person_with_spread_keypoints():
    keypoints = np.zeros((1, 17, 3), np.float32)
    keypoints[0, :, 0] = np.arange(17) * 30 + 40  # Keypoints 30 px apart along y = 60
    keypoints[0, :, 1] = 60
    keypoints[0, :, 2] = 0.9
    keypoints[0, 16] = [600, 450, 0.1]  # Too uncertain to draw
    return keypoints


def test_overlay_draws_pose_target_and_guides_on_a_copy():
    frame = np.zeros((480, 640, 3), np.uint8)

    annotated = draw_tracking_overlay(
        frame, keypoints=person_with_spread_keypoints(), target=(500, 400), aim=(320, 240), jaw_zone=100
    )

    assert frame.max() == 0  # The caller's frame is untouched
    assert annotated[60, 40].any() and annotated[60, 40 + 15 * 30].any()  # Keypoints drawn
    assert not annotated[450, 600].any()  # Low-confidence keypoint skipped
    assert annotated[400, 520].any()  # Target circle, radius 20
    assert annotated[240, 320].any()  # Aim crosshair
    assert annotated[240, 220].any()  # Jaw zone edge, 100 px left of the aim


def test_overlay_without_people_still_draws_the_guides():
    annotated = draw_tracking_overlay(
        np.zeros((480, 640, 3), np.uint8), keypoints=None, target=None, aim=(320, 240), jaw_zone=100
    )

    assert annotated[240, 320].any()
    assert not annotated[60, 40].any()
