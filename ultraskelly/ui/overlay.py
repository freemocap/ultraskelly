"""Draw pose skeletons, the tracked target, and centering guides onto RGB camera frames.

Shared by the OpenCV window and the dashboard's annotated video stream. Colors are RGB.
"""
import cv2
import numpy as np

from ultraskelly.core.bot.sensory.pose_common import SKELETON_CONNECTIONS

BONE_COLOR = (0, 255, 255)
HEAD_COLOR = (255, 0, 0)
ARM_COLOR = (0, 255, 0)
LEG_COLOR = (0, 0, 255)
OUTLINE_COLOR = (255, 255, 255)
AIM_COLOR = (141, 227, 194)
JAW_ZONE_COLOR = (255, 178, 105)
TARGET_COLOR = (255, 178, 105)


def draw_skeleton(frame: np.ndarray, keypoints: np.ndarray, *, confidence_threshold: float = 0.3) -> None:
    """Draw one person's bones and keypoints in place on an RGB frame.

    Args:
        frame: Image to draw on, modified in place.
        keypoints: One person's keypoints, shape (17, 3) as [x, y, confidence].
        confidence_threshold: Minimum confidence for a keypoint or bone to be drawn.
    """
    for kp1, kp2 in SKELETON_CONNECTIONS:
        pt1 = keypoints[kp1]
        pt2 = keypoints[kp2]
        if pt1[2] > confidence_threshold and pt2[2] > confidence_threshold:
            cv2.line(frame, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), BONE_COLOR, 2)

    for index, kp in enumerate(keypoints):
        if kp[2] > confidence_threshold:
            x, y = int(kp[0]), int(kp[1])
            color = HEAD_COLOR if index <= 4 else ARM_COLOR if index <= 10 else LEG_COLOR
            cv2.circle(frame, (x, y), 5, color, -1)
            cv2.circle(frame, (x, y), 6, OUTLINE_COLOR, 1)


def draw_tracking_overlay(
    frame: np.ndarray,
    *,
    keypoints: np.ndarray | None,
    target: tuple[int, int] | None,
    aim: tuple[int, int],
    jaw_zone: int,
) -> np.ndarray:
    """A copy of an RGB frame with every skeleton, the tracked target, and the centering guides.

    Args:
        frame: Camera image; it is not modified.
        keypoints: All people's keypoints, shape (num_people, 17, 3), or None.
        target: Tracked point in pixels, or None when there is no target.
        aim: Pixel the head steers the target toward.
        jaw_zone: Half-width in pixels of the square around the aim that opens the jaw.
    """
    annotated = np.ascontiguousarray(frame[..., :3]).copy()
    aim_x, aim_y = aim
    cv2.rectangle(annotated, (aim_x - jaw_zone, aim_y - jaw_zone), (aim_x + jaw_zone, aim_y + jaw_zone), JAW_ZONE_COLOR, 1)
    cv2.line(annotated, (aim_x - 20, aim_y), (aim_x + 20, aim_y), AIM_COLOR, 2)
    cv2.line(annotated, (aim_x, aim_y - 20), (aim_x, aim_y + 20), AIM_COLOR, 2)
    for person in keypoints if keypoints is not None else []:
        draw_skeleton(annotated, person)
    if target is not None:
        cv2.circle(annotated, target, 20, TARGET_COLOR, 3)
    return annotated
