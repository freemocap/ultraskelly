"""MediaPipe hands, Pi side: palm decoding, the hand crop and the landmarks' way back to the image."""
import numpy as np
import pytest

from ultraskelly.core.bot.sensory.mediapipe_hands import HandCrop, decode_palms, hand_crop, hand_points, palm_anchors


def test_anchors_match_the_rows_of_the_zoo_palm_detectors_outputs():
    assert len(palm_anchors(128)) == 896 and len(palm_anchors(192)) == 2016  # (1, 896, 18) and (1, 2016, 18)


def test_a_confident_anchor_decodes_to_a_palm_in_image_pixels_through_the_letterbox():
    anchors = palm_anchors(128)
    regressors, scores = np.zeros((896, 18), np.float32), np.full(896, -10.0, np.float32)
    row = int(np.argmin(np.abs(anchors[:, 0] - 0.5) + np.abs(anchors[:, 1] - 0.5)))
    scores[row] = 3.0  # Sigmoid 0.95
    regressors[row, 2:4] = [32, 16]  # 32 x 16 input pixels
    regressors[row, 4:6] = [0, 8]  # The wrist 8 input pixels below the centre
    cx, cy = anchors[row] * 128

    palms = decode_palms(regressors, scores, image_size=(640, 480))

    assert palms.count == 1 and palms.scores[0] == pytest.approx(0.9526, abs=1e-3)
    # 640x480 letterboxes into 128x96 at a fifth, 16 rows down: input (x, y) is image (5x, 5(y - 16)).
    assert palms.boxes[0] == pytest.approx([5 * (cx - 16), 5 * (cy - 8 - 16), 5 * (cx + 16), 5 * (cy + 8 - 16)])
    assert palms.keypoints[0, 0] == pytest.approx([5 * cx, 5 * (cy + 8 - 16)])


def test_nothing_above_the_threshold_finds_no_palms():
    assert decode_palms(np.zeros((896, 18)), np.full(896, -10.0), image_size=(640, 480)).count == 0


def palm(wrist, middle_finger, box=(100.0, 100.0, 140.0, 140.0)):
    keypoints = np.zeros((7, 2), np.float32)
    keypoints[0], keypoints[2] = wrist, middle_finger
    return np.array(box, np.float32), keypoints


def test_an_upright_hand_gets_an_unturned_crop_shifted_toward_its_fingers():
    crop = hand_crop(*palm(wrist=(120, 135), middle_finger=(120, 105)))

    assert crop.angle == pytest.approx(0.0)
    assert crop.center == pytest.approx((120, 120 - 0.5 * 40))
    assert crop.side == pytest.approx(2.6 * 40)


def test_a_hand_pointing_right_is_turned_a_quarter_clockwise():
    crop = hand_crop(*palm(wrist=(105, 120), middle_finger=(135, 120)))

    assert crop.angle == pytest.approx(90.0)
    assert crop.center == pytest.approx((120 + 0.5 * 40, 120))


def test_landmarks_map_from_the_crop_back_to_the_image():
    crop = HandCrop(center=(300.0, 200.0), side=224.0, angle=90.0)  # One input pixel per image pixel
    points = np.zeros((21, 3), np.float32)
    points[0] = [112, 112, 0]  # The crop's centre
    points[1] = [112, 0, 5]  # Top middle of the crop: up the turned crop is right in the image

    mapped = hand_points(points.reshape(1, 63), crop)

    assert mapped[0] == pytest.approx([300, 200, 0])
    assert mapped[1] == pytest.approx([412, 200, 5])
