"""Decode YuNet face detections (Luxonis Model Zoo `yunet`): boxes, five landmarks and scores. No hardware imports.

The network's three outputs hold one row per prior box on four feature maps (strides 8 to
64): `loc` the box and landmark offsets, `conf` the probabilities of background and face,
`iou` the predicted box quality. Decoding follows Luxonis' YuNetParser (depthai-nodes 0.7),
taken from libfacedetection's reference: score = sqrt(face probability x IoU), offsets
scaled by the variances 0.1 and 0.2, then non-maximum suppression.
"""
from dataclasses import dataclass
from functools import cache

import numpy as np

from ultraskelly.core.bot.sensory.yolov8_pose import _non_max_suppression

# The five landmarks in YuNet's order. Right and left are the person's own: their right eye
# appears on the left of an image of them facing the camera.
LANDMARKS = ("right_eye", "left_eye", "nose", "right_mouth", "left_mouth")
MIN_SIZES = ((10, 16, 24), (32, 48), (64, 96), (128, 192, 256))
STRIDES = (8, 16, 32, 64)
VARIANCES = (0.1, 0.2)


@dataclass(frozen=True)
class FaceDetections:
    """Faces in one image, best first, in coordinates normalized to the image (0 to 1)."""

    boxes: np.ndarray  # (N, 4) x1, y1, x2, y2
    landmarks: np.ndarray  # (N, 5, 2) x, y in LANDMARKS order
    scores: np.ndarray  # (N,)

    @property
    def count(self) -> int:
        return len(self.scores)

    @classmethod
    def empty(cls) -> "FaceDetections":
        return cls(np.zeros((0, 4), np.float32), np.zeros((0, 5, 2), np.float32), np.zeros(0, np.float32))


@cache
def yunet_priors(width: int, height: int) -> np.ndarray:
    """The network's prior boxes for a `width` x `height` input: (P, 4) centre x, centre y,
    width, height, normalized. Feature maps halve the input five times, rounding down."""
    rows, columns = int(int((height + 1) / 2) / 2), int(int((width + 1) / 2) / 2)  # Stride 4
    priors = []
    for min_sizes, stride in zip(MIN_SIZES, STRIDES):
        rows, columns = rows // 2, columns // 2
        y, x = np.mgrid[0:rows, 0:columns]
        centres = np.stack([(x + 0.5) * stride / width, (y + 0.5) * stride / height], axis=-1).reshape(-1, 1, 2)
        sizes = np.array([[size / width, size / height] for size in min_sizes])  # (sizes, 2)
        layer = np.concatenate(np.broadcast_arrays(centres, sizes[None, :, :]), axis=-1)  # (cells, sizes, 4)
        priors.append(layer.reshape(-1, 4))
    return np.concatenate(priors).astype(np.float32)


def decode_yunet(
    loc: np.ndarray,
    conf: np.ndarray,
    iou: np.ndarray,
    *,
    input_size: tuple[int, int],
    score_threshold: float = 0.6,
    iou_threshold: float = 0.3,
    max_faces: int = 10,
) -> FaceDetections:
    """Faces from YuNet's raw outputs for an input of `input_size` (width, height).

    Args:
        loc: (P, 14) per prior: box centre and size offsets, then five landmark offsets.
        conf: (P, 2) background and face probabilities.
        iou: (P, 1) predicted IoU of each box.
        score_threshold: Smallest kept score, sqrt(face probability x IoU).
        iou_threshold: Boxes overlapping a better one by more than this are dropped.
        max_faces: Most faces returned.
    """
    priors = yunet_priors(*input_size)
    loc, conf, iou = (np.asarray(array, np.float32).reshape(len(priors), -1) for array in (loc, conf, iou))
    scores = np.sqrt(conf[:, 1].clip(min=0.0) * iou[:, 0].clip(0.0, 1.0))
    keep = scores > score_threshold
    if not np.any(keep):
        return FaceDetections.empty()
    loc, priors, scores = loc[keep], priors[keep], scores[keep]
    centres = priors[:, :2] + loc[:, :2] * VARIANCES[0] * priors[:, 2:]
    sizes = priors[:, 2:] * np.exp(loc[:, 2:4] * VARIANCES[1])
    boxes = np.concatenate([centres - sizes / 2, centres + sizes / 2], axis=1)
    landmarks = priors[:, None, :2] + loc[:, 4:14].reshape(-1, 5, 2) * VARIANCES[0] * priors[:, None, 2:]
    kept = _non_max_suppression(boxes=boxes, scores=scores, iou_threshold=iou_threshold)[:max_faces]
    return FaceDetections(
        boxes=boxes[kept].clip(0.0, 1.0), landmarks=landmarks[kept].clip(0.0, 1.0), scores=scores[kept],
    )
