"""COCO keypoints and target selection shared by every pose detector. No hardware imports."""
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


def select_target(
    *,
    keypoints: np.ndarray | None,
    scores: np.ndarray | None,
    target_keypoint: CocoKeypoint,
    keypoint_threshold: float,
) -> tuple[int | None, int | None, float | None]:
    """Target keypoint position and body angle of the highest-scoring person.

    Args:
        keypoints: All people's keypoints, shape (num_people, 17, 3), or None.
        scores: Person confidences, shape (num_people,), or None.
        target_keypoint: Body part to track.
        keypoint_threshold: Minimum confidence for the target keypoint.

    Returns:
        (x, y, angle) in frame pixels and degrees; angle is 0.0 when the shoulders are
        not visible. All None when no person or no confident target keypoint.
    """
    if keypoints is None or scores is None or len(scores) == 0:
        return (None, None, None)

    person = keypoints[int(np.argmax(scores))]
    target = person[target_keypoint]
    if target[2] < keypoint_threshold:
        return (None, None, None)

    angle = body_angle_from_shoulders(keypoints=person, keypoint_threshold=keypoint_threshold)
    return (int(target[0]), int(target[1]), angle if angle is not None else 0.0)
