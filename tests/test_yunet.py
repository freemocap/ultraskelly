"""YuNet's raw outputs decode into face boxes, landmarks and scores, as Luxonis' parser decodes them."""
import numpy as np
import pytest

from ultraskelly.core.bot.sensory.yunet import decode_yunet, yunet_priors


def test_priors_match_the_rows_of_the_zoo_models_outputs():
    # Output shapes from the archives' config.json: (4385, 14), (13150, 14), (17610, 14).
    assert [len(yunet_priors(*size)) for size in [(320, 240), (640, 360), (640, 480)]] == [4385, 13150, 17610]


def blank_outputs(width=320, height=240):
    rows = len(yunet_priors(width, height))
    conf = np.tile(np.array([1.0, 0.0], np.float32), (rows, 1))  # Background everywhere
    return np.zeros((rows, 14), np.float32), conf, np.zeros((rows, 1), np.float32)


def prior_near(x, y, size, width=320):
    """The row of the prior centred nearest (x, y) with the given side in input pixels."""
    priors = yunet_priors(320, 240)
    return int(np.argmin(np.abs(priors[:, 0] - x) + np.abs(priors[:, 1] - y) + np.abs(priors[:, 2] - size / width)))


def test_a_confident_prior_decodes_to_its_box_landmarks_and_score():
    loc, conf, iou = blank_outputs()
    row = prior_near(0.5, 0.5, 48)
    conf[row], iou[row] = [0.19, 0.81], [1.0]
    loc[row, 0:2] = [1.0, 0.0]  # Centre shifted right by a tenth of the prior's width
    loc[row, 4:6] = [-1.0, -2.0]  # Right eye left of and above the centre
    cx, cy, w, h = yunet_priors(320, 240)[row]

    faces = decode_yunet(loc, conf, iou, input_size=(320, 240))

    assert faces.count == 1
    assert faces.scores[0] == pytest.approx(0.9)  # sqrt(0.81 x 1.0)
    assert faces.boxes[0] == pytest.approx([cx + 0.1 * w - w / 2, cy - h / 2, cx + 0.1 * w + w / 2, cy + h / 2])
    assert faces.landmarks[0, 0] == pytest.approx([cx - 0.1 * w, cy - 0.2 * h])
    assert faces.landmarks[0, 1] == pytest.approx([cx, cy])  # Zero offsets sit on the prior's centre


def test_overlapping_faces_keep_the_best_and_weak_ones_are_dropped():
    loc, conf, iou = blank_outputs()
    best, overlapping, weak = prior_near(0.3, 0.5, 16), prior_near(0.3, 0.5, 24), prior_near(0.8, 0.5, 16)
    for row, probability in ((best, 0.9), (overlapping, 0.8), (weak, 0.3)):
        conf[row], iou[row] = [1 - probability, probability], [probability]

    faces = decode_yunet(loc, conf, iou, input_size=(320, 240), score_threshold=0.6, iou_threshold=0.3)

    assert faces.count == 1 and faces.scores[0] == pytest.approx(0.9)  # 16 and 24 px squares overlap 0.44


def test_no_score_above_the_threshold_finds_no_faces():
    faces = decode_yunet(*blank_outputs(), input_size=(320, 240))
    assert faces.count == 0 and faces.boxes.shape == (0, 4) and faces.landmarks.shape == (0, 5, 2)
