import numpy as np
import pytest

from ultraskelly.core.bot.sensory.pose_common import (
    CocoKeypoint,
    body_angle_from_shoulders,
    locate_target,
    select_target,
)


def person(*, nose=(100.0, 50.0, 0.9), left_shoulder=(80.0, 120.0, 0.9), right_shoulder=(120.0, 120.0, 0.9)):
    keypoints = np.zeros((17, 3), np.float32)
    keypoints[CocoKeypoint.NOSE] = nose
    keypoints[CocoKeypoint.LEFT_SHOULDER] = left_shoulder
    keypoints[CocoKeypoint.RIGHT_SHOULDER] = right_shoulder
    return keypoints


@pytest.mark.parametrize(
    ("left_shoulder", "right_shoulder", "expected"),
    [
        ((80, 120, 0.9), (120, 120, 0.9), 90.0),  # Level shoulders
        ((100, 200, 0.9), (200, 100, 0.9), 45.0),
        ((100, 100, 0.9), (200, 200, 0.9), -45.0),
    ],
)
def test_body_angle_from_shoulder_line(left_shoulder, right_shoulder, expected):
    keypoints = person(left_shoulder=left_shoulder, right_shoulder=right_shoulder)

    assert body_angle_from_shoulders(keypoints=keypoints, keypoint_threshold=0.4) == pytest.approx(expected)


def test_body_angle_needs_both_shoulders():
    keypoints = person(left_shoulder=(80, 120, 0.1))

    assert body_angle_from_shoulders(keypoints=keypoints, keypoint_threshold=0.4) is None


def test_select_target_tracks_the_highest_scoring_person():
    keypoints = np.stack([person(nose=(10, 20, 0.9)), person(nose=(300, 200, 0.9))])

    x, y, angle = select_target(
        keypoints=keypoints,
        scores=np.array([0.6, 0.8]),
        target_keypoint=CocoKeypoint.NOSE,
        keypoint_threshold=0.4,
    )

    assert (x, y, angle) == (300, 200, pytest.approx(90.0))


def test_select_target_defaults_angle_to_zero_without_shoulders():
    keypoints = person(left_shoulder=(80, 120, 0.0))[None]

    assert select_target(
        keypoints=keypoints, scores=np.array([0.9]), target_keypoint=CocoKeypoint.NOSE, keypoint_threshold=0.4
    ) == (100, 50, 0.0)


@pytest.mark.parametrize(
    ("keypoints", "scores"),
    [
        (None, None),
        (np.empty((0, 17, 3)), np.empty(0)),
        (person(nose=(100, 50, 0.1))[None], np.array([0.9])),  # Target keypoint not confident
    ],
)
def test_select_target_reports_no_target(keypoints, scores):
    assert select_target(
        keypoints=keypoints, scores=scores, target_keypoint=CocoKeypoint.NOSE, keypoint_threshold=0.4
    ) == (None, None, None)


def test_locate_target_names_the_followed_persons_row_with_the_same_target():
    keypoints = np.stack([person(nose=(10, 20, 0.9)), person(nose=(300, 200, 0.9))])
    scores = np.array([0.6, 0.8])

    row, target = locate_target(
        keypoints=keypoints, scores=scores, target_keypoint=CocoKeypoint.NOSE, keypoint_threshold=0.4
    )

    assert row == 1
    assert target == select_target(
        keypoints=keypoints, scores=scores, target_keypoint=CocoKeypoint.NOSE, keypoint_threshold=0.4
    )


def test_locate_target_follows_nobody_when_the_best_persons_target_keypoint_is_unsure():
    keypoints = np.stack([person(nose=(10, 20, 0.9)), person(nose=(300, 200, 0.1))])

    assert locate_target(
        keypoints=keypoints, scores=np.array([0.6, 0.8]), target_keypoint=CocoKeypoint.NOSE, keypoint_threshold=0.4
    ) == (None, (None, None, None))
