"""Decode YOLOv8 pose outputs, as laid out by Hailo's yolov8*_pose HEFs, into people.

For each stride (8, 16, 32) the model emits three tensors on an (input_size / stride)² grid:
box side distances as 4 x 16 distribution bins, a person score (sigmoid already applied on
the chip), and 17 COCO keypoints as raw (x, y, visibility logit). Pure NumPy, so it runs
and is tested anywhere.
"""
from dataclasses import dataclass

import numpy as np

STRIDES: tuple[int, ...] = (8, 16, 32)
DFL_BINS = 16
NUM_KEYPOINTS = 17


@dataclass(frozen=True)
class Letterbox:
    """How a frame was scaled and padded to fit the square model input."""
    scale: float
    pad_x: int
    pad_y: int
    resized_width: int
    resized_height: int

    @classmethod
    def fit(cls, *, width: int, height: int, size: int) -> "Letterbox":
        """Largest aspect-preserving scale of a width x height frame that fits size x size, centered."""
        scale = min(size / width, size / height)
        resized_width = round(width * scale)
        resized_height = round(height * scale)
        return cls(
            scale=scale,
            pad_x=(size - resized_width) // 2,
            pad_y=(size - resized_height) // 2,
            resized_width=resized_width,
            resized_height=resized_height,
        )


@dataclass(frozen=True)
class PoseDetections:
    """People found in one frame, in frame pixel coordinates, best first."""
    keypoints: np.ndarray  # (num_people, 17, 3) as [x, y, visibility]
    scores: np.ndarray  # (num_people,)
    boxes: np.ndarray  # (num_people, 4) as [x1, y1, x2, y2]

    @property
    def count(self) -> int:
        return len(self.scores)


def decode_yolov8_pose(
    outputs: list[np.ndarray],
    *,
    input_size: int,
    letterbox: Letterbox,
    score_threshold: float,
    iou_threshold: float,
    max_people: int,
) -> PoseDetections:
    """Turn the nine raw output tensors into scored people with boxes and keypoints.

    Args:
        outputs: The model's output tensors in any order, each (grid, grid, channels).
        input_size: Side of the square model input in pixels.
        letterbox: How the frame was fitted into the model input; results are mapped back.
        score_threshold: Minimum person confidence.
        iou_threshold: Overlap above which a lower-scoring box is dropped as a duplicate.
        max_people: Most people to return.
    """
    by_shape = {(output.shape[0], output.shape[-1]): output for output in outputs}
    boxes: list[np.ndarray] = []
    scores: list[np.ndarray] = []
    keypoints: list[np.ndarray] = []

    for stride in STRIDES:
        grid = input_size // stride
        cell_scores = by_shape[(grid, 1)].reshape(-1)
        cells = np.flatnonzero(cell_scores >= score_threshold)
        if cells.size == 0:
            continue

        box_bins = by_shape[(grid, 4 * DFL_BINS)].reshape(-1, 4, DFL_BINS)[cells]
        raw_keypoints = by_shape[(grid, NUM_KEYPOINTS * 3)].reshape(-1, NUM_KEYPOINTS, 3)[cells]
        grid_x = (cells % grid).astype(np.float32)
        grid_y = (cells // grid).astype(np.float32)

        # Expected distance, in cells, from the cell center to the left, top, right, bottom sides.
        distances = _softmax(box_bins) @ np.arange(DFL_BINS, dtype=np.float32)
        center_x = grid_x + 0.5
        center_y = grid_y + 0.5
        boxes.append(
            np.stack(
                [
                    center_x - distances[:, 0],
                    center_y - distances[:, 1],
                    center_x + distances[:, 2],
                    center_y + distances[:, 3],
                ],
                axis=1,
            )
            * stride
        )
        keypoints.append(
            np.stack(
                [
                    (raw_keypoints[..., 0] * 2.0 + grid_x[:, None]) * stride,
                    (raw_keypoints[..., 1] * 2.0 + grid_y[:, None]) * stride,
                    _sigmoid(raw_keypoints[..., 2]),
                ],
                axis=-1,
            )
        )
        scores.append(cell_scores[cells])

    if not scores:
        return PoseDetections(
            keypoints=np.empty((0, NUM_KEYPOINTS, 3), dtype=np.float32),
            scores=np.empty(0, dtype=np.float32),
            boxes=np.empty((0, 4), dtype=np.float32),
        )

    all_boxes = np.concatenate(boxes)
    all_scores = np.concatenate(scores)
    all_keypoints = np.concatenate(keypoints)
    keep = _non_max_suppression(boxes=all_boxes, scores=all_scores, iou_threshold=iou_threshold)[:max_people]

    # Map from model-input pixels back to frame pixels.
    offset = np.array([letterbox.pad_x, letterbox.pad_y], dtype=np.float32)
    frame_keypoints = all_keypoints[keep]
    frame_keypoints[..., :2] = (frame_keypoints[..., :2] - offset) / letterbox.scale
    frame_boxes = (all_boxes[keep] - np.tile(offset, 2)) / letterbox.scale
    return PoseDetections(keypoints=frame_keypoints, scores=all_scores[keep], boxes=frame_boxes)


def _softmax(x: np.ndarray) -> np.ndarray:
    exp = np.exp(x - x.max(axis=-1, keepdims=True))
    return exp / exp.sum(axis=-1, keepdims=True)


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def _non_max_suppression(*, boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> np.ndarray:
    """Indices of the boxes to keep, best first, dropping any that overlap a better box too much."""
    order = np.argsort(scores)[::-1]
    areas = (boxes[:, 2] - boxes[:, 0]).clip(min=0) * (boxes[:, 3] - boxes[:, 1]).clip(min=0)
    keep: list[int] = []
    while order.size:
        best, rest = order[0], order[1:]
        keep.append(int(best))
        overlap_width = (np.minimum(boxes[best, 2], boxes[rest, 2]) - np.maximum(boxes[best, 0], boxes[rest, 0])).clip(min=0)
        overlap_height = (np.minimum(boxes[best, 3], boxes[rest, 3]) - np.maximum(boxes[best, 1], boxes[rest, 1])).clip(min=0)
        intersection = overlap_width * overlap_height
        iou = intersection / (areas[best] + areas[rest] - intersection + 1e-9)
        order = rest[iou <= iou_threshold]
    return np.array(keep, dtype=int)
