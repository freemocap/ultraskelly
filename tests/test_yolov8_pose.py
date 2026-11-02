import numpy as np
import pytest

from ultraskelly.core.bot.sensory.yolov8_pose import (
    DFL_BINS,
    NUM_KEYPOINTS,
    STRIDES,
    Letterbox,
    decode_yolov8_pose,
)

INPUT_SIZE = 640
NO_LETTERBOX = Letterbox(scale=1.0, pad_x=0, pad_y=0, resized_width=INPUT_SIZE, resized_height=INPUT_SIZE)


def empty_outputs() -> dict[tuple[int, int], np.ndarray]:
    """Raw outputs with no person anywhere, keyed by (stride, channels)."""
    outputs = {}
    for stride in STRIDES:
        grid = INPUT_SIZE // stride
        outputs[(stride, 4 * DFL_BINS)] = np.zeros((grid, grid, 4 * DFL_BINS), np.float32)
        outputs[(stride, 1)] = np.zeros((grid, grid, 1), np.float32)
        outputs[(stride, NUM_KEYPOINTS * 3)] = np.zeros((grid, grid, NUM_KEYPOINTS * 3), np.float32)
    return outputs


def place_person(
    outputs: dict[tuple[int, int], np.ndarray],
    *,
    stride: int,
    cell: tuple[int, int],
    score: float,
    sides: tuple[int, int, int, int],
    keypoint_offset: tuple[float, float] = (0.25, -0.25),
    visibility_logit: float = 4.0,
) -> None:
    """Write one person at grid cell (x, y): box sides in cells (left, top, right, bottom)."""
    x, y = cell
    outputs[(stride, 1)][y, x, 0] = score
    bins = np.full((4, DFL_BINS), -50.0, np.float32)
    for side, distance in enumerate(sides):
        bins[side, distance] = 50.0  # Softmax is one-hot, so the expected distance is exact
    outputs[(stride, 4 * DFL_BINS)][y, x] = bins.reshape(-1)
    keypoints = np.tile([*keypoint_offset, visibility_logit], NUM_KEYPOINTS).astype(np.float32)
    outputs[(stride, NUM_KEYPOINTS * 3)][y, x] = keypoints


def decode(outputs, *, letterbox=NO_LETTERBOX, score_threshold=0.5, iou_threshold=0.65, max_people=10):
    return decode_yolov8_pose(
        list(outputs.values()),
        input_size=INPUT_SIZE,
        letterbox=letterbox,
        score_threshold=score_threshold,
        iou_threshold=iou_threshold,
        max_people=max_people,
    )


def test_no_person_returns_empty_arrays_with_pose_shapes():
    detections = decode(empty_outputs())

    assert detections.count == 0
    assert detections.keypoints.shape == (0, NUM_KEYPOINTS, 3)
    assert detections.boxes.shape == (0, 4)


def test_decodes_box_keypoints_and_score_for_one_person():
    outputs = empty_outputs()
    place_person(outputs, stride=16, cell=(10, 12), score=0.9, sides=(3, 4, 5, 6))

    detections = decode(outputs)

    assert detections.count == 1
    assert detections.scores[0] == pytest.approx(0.9)
    # Cell center (10.5, 12.5) cells; sides 3, 4, 5, 6 cells; 16 px per cell.
    np.testing.assert_allclose(detections.boxes[0], [120, 136, 248, 296], atol=1e-3)
    # Keypoint x = (2 * 0.25 + 10) * 16, y = (2 * -0.25 + 12) * 16, visibility = sigmoid(4).
    np.testing.assert_allclose(detections.keypoints[0, :, 0], 168.0, atol=1e-3)
    np.testing.assert_allclose(detections.keypoints[0, :, 1], 184.0, atol=1e-3)
    np.testing.assert_allclose(detections.keypoints[0, :, 2], 1 / (1 + np.exp(-4.0)), atol=1e-6)


def test_people_below_score_threshold_are_dropped():
    outputs = empty_outputs()
    place_person(outputs, stride=8, cell=(5, 5), score=0.3, sides=(1, 1, 1, 1))

    assert decode(outputs, score_threshold=0.5).count == 0


def test_overlapping_duplicates_collapse_to_the_best_and_distinct_people_survive():
    outputs = empty_outputs()
    place_person(outputs, stride=32, cell=(5, 5), score=0.8, sides=(3, 3, 3, 3))
    place_person(outputs, stride=32, cell=(6, 5), score=0.9, sides=(3, 3, 3, 3))  # Same person, shifted
    place_person(outputs, stride=32, cell=(15, 15), score=0.7, sides=(2, 2, 2, 2))  # Someone else

    detections = decode(outputs)

    np.testing.assert_allclose(detections.scores, [0.9, 0.7], atol=1e-6)


def test_max_people_keeps_the_highest_scores():
    outputs = empty_outputs()
    for index, score in enumerate([0.6, 0.9, 0.7]):
        place_person(outputs, stride=32, cell=(2 + 6 * index, 10), score=score, sides=(1, 1, 1, 1))

    detections = decode(outputs, max_people=2)

    np.testing.assert_allclose(detections.scores, [0.9, 0.7], atol=1e-6)


def test_letterbox_fit_centers_a_landscape_frame():
    letterbox = Letterbox.fit(width=640, height=480, size=640)

    assert (letterbox.scale, letterbox.pad_x, letterbox.pad_y) == (1.0, 0, 80)
    assert (letterbox.resized_width, letterbox.resized_height) == (640, 480)


def test_results_are_mapped_back_through_the_letterbox():
    outputs = empty_outputs()
    place_person(outputs, stride=16, cell=(10, 12), score=0.9, sides=(3, 4, 5, 6))
    # A 1280x720 frame scaled by 0.5 into 640x360, with 140 px of padding above and below.
    letterbox = Letterbox.fit(width=1280, height=720, size=INPUT_SIZE)

    detections = decode(outputs, letterbox=letterbox)

    np.testing.assert_allclose(detections.boxes[0], [240, -8, 496, 312], atol=1e-3)
    np.testing.assert_allclose(detections.keypoints[0, 0, :2], [336, 88], atol=1e-3)
