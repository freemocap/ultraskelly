"""Draw pose skeletons onto RGB camera frames for the OpenCV window.

Colors are RGB. The dashboard draws the streamed poses in the browser with the same colors.
"""
import cv2
import numpy as np

from ultraskelly.core.bot.sensory.pose_common import SKELETON_CONNECTIONS

BONE_COLOR = (0, 255, 255)
HEAD_COLOR = (255, 0, 0)
ARM_COLOR = (0, 255, 0)
LEG_COLOR = (0, 0, 255)
OUTLINE_COLOR = (255, 255, 255)


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
