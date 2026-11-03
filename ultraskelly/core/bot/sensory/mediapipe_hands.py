"""MediaPipe hands on the OAK, Pi side: palms from the palm detector, the hand crop each needs, and
the hand landmarks mapped back to the image. No hardware imports.

The palm detector (Luxonis Model Zoo `mediapipe-palm-detection:128x128`) sees the whole frame
letterboxed to 128 x 128 and answers per SSD anchor: a palm box, seven palm keypoints and a
score. Decoding follows MediaPipe's palm detection graph as Luxonis' MPPalmDetectionParser
(depthai-nodes 0.7) implements it. Each palm becomes a square crop, turned so the fingers point
up and shifted toward them, as MediaPipe's palm-to-hand rectangle does (scale 2.6, shift 0.5 of
the palm box); the hand landmarker (`mediapipe-hand-landmarker:224x224`) then finds 21 points in
it. Angles are clockwise on screen, as both MediaPipe and the OAK's ImageManip take them.
"""
import math
from dataclasses import dataclass
from functools import cache

import numpy as np

from ultraskelly.core.bot.sensory.yolov8_pose import Letterbox, _non_max_suppression

PALM_INPUT = 128
HAND_INPUT = 224
HAND_SCALE = 2.6  # Hand crop side over the palm box's longer side
HAND_SHIFT = 0.5  # Toward the fingers, in palm box heights
WRIST, MIDDLE_FINGER = 0, 2  # Palm keypoints that set the hand's direction
HAND_POINTS = (
    "wrist", "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
    "index_mcp", "index_pip", "index_dip", "index_tip", "middle_mcp", "middle_pip", "middle_dip", "middle_tip",
    "ring_mcp", "ring_pip", "ring_dip", "ring_tip", "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip",
)
HAND_BONES = (  # The archive's skeleton edges
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16), (0, 17), (17, 18), (18, 19), (19, 20),
)


@dataclass(frozen=True)
class Palms:
    """Palms in one frame, best first, in image pixels."""

    boxes: np.ndarray  # (N, 4) x1, y1, x2, y2
    keypoints: np.ndarray  # (N, 7, 2)
    scores: np.ndarray  # (N,)

    @property
    def count(self) -> int:
        return len(self.scores)


@dataclass(frozen=True)
class HandCrop:
    """A square to cut out of the image for the hand landmarker, in pixels; `angle` in degrees,
    clockwise on screen, turns the crop so the fingers point up in it."""

    center: tuple[float, float]
    side: float
    angle: float


@cache
def palm_anchors(size: int = PALM_INPUT) -> np.ndarray:
    """MediaPipe's palm detection anchors, (A, 2) centres normalized; every anchor is 1 x 1."""
    centres = []
    for stride, per_cell in ((8, 2), (16, 6)):  # Strides 8, 16, 16, 16 merge into two layers
        cells = math.ceil(size / stride)
        y, x = np.mgrid[0:cells, 0:cells]
        centre = np.stack([(x + 0.5) / cells, (y + 0.5) / cells], axis=-1).reshape(-1, 1, 2)
        centres.append(np.repeat(centre, per_cell, axis=1).reshape(-1, 2))
    return np.concatenate(centres).astype(np.float32)


def decode_palms(
    regressors: np.ndarray, scores: np.ndarray, *, image_size: tuple[int, int], score_threshold: float = 0.5,
    iou_threshold: float = 0.5, max_palms: int = 2, size: int = PALM_INPUT,
) -> Palms:
    """Palms from the detector's raw outputs for the image letterboxed into its `size` input."""
    anchors = palm_anchors(size)
    raw = np.asarray(regressors, np.float32).reshape(len(anchors), 18)
    logits = np.asarray(scores, np.float32).reshape(len(anchors))
    probability = 1.0 / (1.0 + np.exp(-np.clip(logits, -100, 100)))
    keep = probability > score_threshold
    if not np.any(keep):
        return Palms(np.zeros((0, 4), np.float32), np.zeros((0, 7, 2), np.float32), np.zeros(0, np.float32))
    raw, anchor, probability = raw[keep] / size, anchors[keep], probability[keep]
    centres, sides = raw[:, :2] + anchor, raw[:, 2:4]
    boxes = np.concatenate([centres - sides / 2, centres + sides / 2], axis=1)
    keypoints = raw[:, 4:].reshape(-1, 7, 2) + anchor[:, None, :]
    kept = _non_max_suppression(boxes=boxes, scores=probability, iou_threshold=iou_threshold)[:max_palms]
    width, height = image_size
    letterbox = Letterbox.fit(width=width, height=height, size=size)
    offset = np.array([letterbox.pad_x, letterbox.pad_y], np.float32)

    def to_image(points: np.ndarray) -> np.ndarray:
        return (points * size - offset) / letterbox.scale

    return Palms(
        boxes=to_image(boxes[kept].reshape(-1, 2, 2)).reshape(-1, 4),
        keypoints=to_image(keypoints[kept]),
        scores=probability[kept],
    )


def hand_crop(box: np.ndarray, keypoints: np.ndarray) -> HandCrop:
    """MediaPipe's hand rectangle for one palm: square, fingers up, shifted toward them."""
    direction = keypoints[MIDDLE_FINGER] - keypoints[WRIST]
    angle = math.atan2(float(direction[0]), float(-direction[1]))  # 0 when the fingers point up the image
    up = np.array([math.sin(angle), -math.cos(angle)])
    width, height = float(box[2] - box[0]), float(box[3] - box[1])
    centre = (box[:2] + box[2:]) / 2 + HAND_SHIFT * height * up
    return HandCrop(center=(float(centre[0]), float(centre[1])), side=HAND_SCALE * max(width, height),
                    angle=math.degrees(angle))


def hand_points(values: np.ndarray, crop: HandCrop, size: int = HAND_INPUT) -> np.ndarray:
    """The hand landmarker's 21 (x, y, z) in its input pixels, as image pixels (21, 3); z keeps the
    model's relative depth, in image pixels."""
    points = np.asarray(values, np.float32).reshape(21, 3)
    angle = math.radians(crop.angle)
    x_axis, y_axis = np.array([math.cos(angle), math.sin(angle)]), np.array([-math.sin(angle), math.cos(angle)])
    scale = crop.side / size
    local = points[:, :2] / size - 0.5
    image = np.array(crop.center) + crop.side * (local[:, :1] * x_axis + local[:, 1:] * y_axis)
    return np.column_stack([image, points[:, 2] * scale]).astype(np.float32)
