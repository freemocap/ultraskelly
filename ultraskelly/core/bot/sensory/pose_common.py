"""COCO keypoints and target selection shared by every pose detector. No hardware imports."""
import math
from enum import IntEnum

import numpy as np


class CocoKeypoint(IntEnum):
    """COCO pose estimation keypoint indices."""
    NOSE = 0
    LEFT_EYE = 1
    RIGHT_EYE = 2
    LEFT_EAR = 3
    RIGHT_EAR = 4
    LEFT_SHOULDER = 5
    RIGHT_SHOULDER = 6
    LEFT_ELBOW = 7
    RIGHT_ELBOW = 8
    LEFT_WRIST = 9
    RIGHT_WRIST = 10
    LEFT_HIP = 11
    RIGHT_HIP = 12
    LEFT_KNEE = 13
    RIGHT_KNEE = 14
    LEFT_ANKLE = 15
    RIGHT_ANKLE = 16


SKELETON_CONNECTIONS: list[tuple[CocoKeypoint, CocoKeypoint]] = [
    # Head
    (CocoKeypoint.NOSE, CocoKeypoint.LEFT_EYE),
    (CocoKeypoint.NOSE, CocoKeypoint.RIGHT_EYE),
    (CocoKeypoint.LEFT_EYE, CocoKeypoint.LEFT_EAR),
    (CocoKeypoint.RIGHT_EYE, CocoKeypoint.RIGHT_EAR),
    # Torso
    (CocoKeypoint.LEFT_SHOULDER, CocoKeypoint.RIGHT_SHOULDER),
    (CocoKeypoint.LEFT_SHOULDER, CocoKeypoint.LEFT_HIP),
    (CocoKeypoint.RIGHT_SHOULDER, CocoKeypoint.RIGHT_HIP),
    (CocoKeypoint.LEFT_HIP, CocoKeypoint.RIGHT_HIP),
    # Left arm
    (CocoKeypoint.LEFT_SHOULDER, CocoKeypoint.LEFT_ELBOW),
    (CocoKeypoint.LEFT_ELBOW, CocoKeypoint.LEFT_WRIST),
    # Right arm
    (CocoKeypoint.RIGHT_SHOULDER, CocoKeypoint.RIGHT_ELBOW),
    (CocoKeypoint.RIGHT_ELBOW, CocoKeypoint.RIGHT_WRIST),
    # Left leg
    (CocoKeypoint.LEFT_HIP, CocoKeypoint.LEFT_KNEE),
    (CocoKeypoint.LEFT_KNEE, CocoKeypoint.LEFT_ANKLE),
    # Right leg
    (CocoKeypoint.RIGHT_HIP, CocoKeypoint.RIGHT_KNEE),
    (CocoKeypoint.RIGHT_KNEE, CocoKeypoint.RIGHT_ANKLE),
]


def body_angle_from_shoulders(*, keypoints: np.ndarray, keypoint_threshold: float) -> float | None:
    """Body orientation from the shoulder line in degrees (0° = vertical), normalized to [-90, 90].

    Args:
        keypoints: One person's keypoints, shape (17, 3) as [x, y, confidence].
        keypoint_threshold: Minimum confidence for both shoulders.
    """
    left_shoulder = keypoints[CocoKeypoint.LEFT_SHOULDER]
    right_shoulder = keypoints[CocoKeypoint.RIGHT_SHOULDER]

    if left_shoulder[2] < keypoint_threshold or right_shoulder[2] < keypoint_threshold:
        return None

    dx = right_shoulder[0] - left_shoulder[0]
    dy = right_shoulder[1] - left_shoulder[1]
    angle = np.degrees(np.arctan2(dx, -dy))

    while angle > 90:
        angle -= 180
    while angle < -90:
        angle += 180

    return float(angle)


FOLLOW_SCORE_SHARE = 0.5  # People scoring at least this share of the best score are credible to follow


def choose_person(
    points: list[tuple[float, float] | None], scores: list[float], aim: tuple[float, float],
) -> int | None:
    """Index of the person to follow: of those with a target point and a score of at least
    FOLLOW_SCORE_SHARE of the best, the one whose target point is nearest `aim`.

    The head steers whoever it follows toward the aim point, so the person already being
    followed sits nearest it, and the choice stays with them instead of flipping to whoever
    scores a little higher on a given frame. None when nobody has a target point.
    """
    credible = [score for point, score in zip(points, scores) if point is not None]
    if not credible:
        return None
    floor = FOLLOW_SCORE_SHARE * max(credible)
    candidates = [
        (math.dist(point, aim), index)
        for index, (point, score) in enumerate(zip(points, scores))
        if point is not None and score >= floor
    ]
    return min(candidates)[1]


def locate_target(
    *,
    keypoints: np.ndarray | None,
    scores: np.ndarray | None,
    target_keypoint: CocoKeypoint,
    keypoint_threshold: float,
    aim: tuple[float, float] | None = None,
) -> tuple[int | None, tuple[int | None, int | None, float | None]]:
    """The person the head follows and their target.

    Args:
        keypoints: All people's keypoints, shape (num_people, 17, 3), or None.
        scores: Person confidences, shape (num_people,), or None.
        target_keypoint: Body part to track.
        keypoint_threshold: Minimum confidence for the target keypoint.
        aim: Where in the frame the head steers its target. With it, the person is chosen by
            `choose_person`; without it, the highest-scoring person is.

    Returns:
        (row, (x, y, angle)): the person's row in `keypoints`, their target keypoint in frame
        pixels and their body angle in degrees, 0.0 when the shoulders are not visible.
        (None, (None, None, None)) when no person or no confident target keypoint.
    """
    if keypoints is None or scores is None or len(scores) == 0:
        return None, (None, None, None)

    if aim is None:
        row = int(np.argmax(scores))
    else:
        points = [
            (float(person[target_keypoint][0]), float(person[target_keypoint][1]))
            if person[target_keypoint][2] >= keypoint_threshold else None
            for person in keypoints
        ]
        row = choose_person(points, [float(score) for score in scores], aim)
        if row is None:
            return None, (None, None, None)
    person = keypoints[row]
    target = person[target_keypoint]
    if target[2] < keypoint_threshold:
        return None, (None, None, None)

    angle = body_angle_from_shoulders(keypoints=person, keypoint_threshold=keypoint_threshold)
    return row, (int(target[0]), int(target[1]), angle if angle is not None else 0.0)


def select_target(
    *,
    keypoints: np.ndarray | None,
    scores: np.ndarray | None,
    target_keypoint: CocoKeypoint,
    keypoint_threshold: float,
) -> tuple[int | None, int | None, float | None]:
    """Target keypoint position and body angle of the highest-scoring person (see `locate_target`).

    Returns:
        (x, y, angle) in frame pixels and degrees; angle is 0.0 when the shoulders are
        not visible. All None when no person or no confident target keypoint.
    """
    return locate_target(
        keypoints=keypoints, scores=scores, target_keypoint=target_keypoint, keypoint_threshold=keypoint_threshold
    )[1]
