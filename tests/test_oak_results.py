"""Joining what the OAK's chip finds in a frame: faces and their measurements, and hands."""
import numpy as np
import pytest

from ultraskelly.core.bot.sensory.mediapipe_hands import palm_anchors
from ultraskelly.core.bot.sensory.oak_faces import CropKind
from ultraskelly.core.bot.sensory.oak_results import ResultsAssembler
from ultraskelly.core.bot.sensory.yunet import FaceDetections

from test_oak_faces import FAR, NEAR, SIZE, detections


def results(*rows):
    return np.array(rows, np.float32)


def test_a_frame_waits_for_its_answers_and_carries_them_per_face():
    assembler = ResultsAssembler(SIZE, max_faces=3)
    crops, done = assembler.detected(10, 1.5, detections(NEAR, FAR), now=0.0)
    assert done == [] and [crop.kind for crop in crops].count(CropKind.MESH) == 1

    assert assembler.answered(10, 0.01, results=results([0, 30, -10, 5, 0, 0, -2], [1, np.nan, np.nan, np.nan] + [np.nan] * 3)) == []
    [message] = assembler.answered(10, 0.02, mesh=np.zeros(1404, np.float32))

    assert (message.frame_number, message.timestamp, message.image_size) == (10, 1.5, SIZE)
    near, far = message.faces
    assert near.box == pytest.approx((256, 96, 384, 240)) and near.score == pytest.approx(0.9)
    assert near.landmarks[1] == pytest.approx([352, 144])  # The person's left eye, in pixels
    assert near.head_pose == pytest.approx((30, -10, 5))
    assert near.gaze == pytest.approx((0, 0, -1))  # Scaled to unit length
    assert near.mesh.shape == (468, 3)
    assert (far.head_pose, far.gaze, far.mesh) == (None, None, None)  # NaN: not measured


def test_a_frame_without_faces_goes_out_at_once_while_an_older_one_is_measured():
    assembler = ResultsAssembler(SIZE, max_faces=3, mesh_every=None)
    assembler.detected(20, None, detections(NEAR), now=0.0)

    _, done = assembler.detected(21, None, FaceDetections.empty(), now=0.03)
    assert [(message.frame_number, message.faces) for message in done] == [(21, [])]  # Found by its number

    [measured] = assembler.answered(20, 0.05, results=results([0, 1, 2, 3, 0, 0, -1]))
    assert measured.frame_number == 20 and measured.faces[0].head_pose == pytest.approx((1, 2, 3))


def test_head_pose_and_landmarks_are_measured_at_their_own_rates():
    assembler = ResultsAssembler(SIZE, max_faces=3, every=2, mesh_every=4)
    kinds = [{crop.kind for crop in assembler.detected(number, None, detections(NEAR), now=0.0)[0]} for number in range(4)]

    assert kinds == [{CropKind.HEAD_POSE, CropKind.LEFT_EYE, CropKind.RIGHT_EYE, CropKind.MESH}, set(),
                     {CropKind.HEAD_POSE, CropKind.LEFT_EYE, CropKind.RIGHT_EYE}, set()]
    assert [number for number in assembler.pending] == [0, 2]  # The unmeasured frames went out with their boxes


def test_measuring_no_faces_asks_for_nothing():
    assembler = ResultsAssembler(SIZE, max_faces=0)
    crops, [message] = assembler.detected(5, None, detections(NEAR), now=0.0)

    assert crops == [] and message.faces[0].head_pose is None and message.faces[0].box == pytest.approx((256, 96, 384, 240))


def test_unanswered_frames_go_out_after_the_timeout_with_what_came():
    assembler = ResultsAssembler(SIZE, max_faces=3, timeout=0.5)
    assembler.detected(30, None, detections(NEAR), now=0.0)
    assembler.answered(30, 0.1, results=results([0, 5, 6, 7, 1, 0, 0]))
    assert assembler.finished(0.4) == []  # Still waiting for the face landmarks

    [message] = assembler.finished(0.6)
    assert message.faces[0].head_pose == pytest.approx((5, 6, 7)) and message.faces[0].mesh is None
    assert assembler.incomplete == 1
    assert assembler.answered(30, 0.7, mesh=np.zeros(1404)) == []  # A late answer finds nothing waiting


def palm_outputs(found: bool):
    """Palm detector outputs with one palm in the middle of the image, or none."""
    anchors = palm_anchors(128)
    regressors, scores = np.zeros((896, 18), np.float32), np.full(896, -10.0, np.float32)
    if found:
        row = int(np.argmin(np.abs(anchors[:, 0] - 0.5) + np.abs(anchors[:, 1] - 0.5)))
        scores[row] = 3.0
        regressors[row, 2:4] = [16, 16]
        regressors[row, 4:6], regressors[row, 8:10] = [0, 6], [0, -6]  # Wrist below, middle finger above
    return regressors, scores


def test_a_frame_looked_at_for_hands_waits_for_its_palms_and_their_landmarks():
    assembler = ResultsAssembler(SIZE, max_faces=3, mesh_every=None, hands_every=1)
    crops, done = assembler.detected(40, None, FaceDetections.empty(), now=0.0)
    assert [crop.kind for crop in crops] == [CropKind.PALM] and done == []

    hand_crops, done = assembler.palms(40, *palm_outputs(found=True), now=0.02)
    assert done == [] and [crop.kind for crop in hand_crops] == [CropKind.HAND]
    hand = hand_crops[0].hand
    assert hand.angle == pytest.approx(0.0) and hand.center[1] < 240  # Fingers up: the crop sits above the palm

    points = np.zeros((21, 3), np.float32)
    points[:] = [112, 112, 0]  # The crop's centre
    [message] = assembler.hand(40, points.reshape(1, 63), presence=0.9, handedness=0.2, now=0.04)

    [found] = message.hands
    assert found.points[0, :2] == pytest.approx(hand.center)
    assert (found.presence, found.handedness) == pytest.approx((0.9, 0.2))
    assert found.palm_box[0] < 320 < found.palm_box[2]


def test_hands_are_none_where_not_looked_for_empty_where_none_and_weak_ones_dropped():
    assembler = ResultsAssembler(SIZE, max_faces=3, mesh_every=None, hands_every=2)
    _, waiting = assembler.detected(0, None, FaceDetections.empty(), now=0.0)  # The first detection looks for hands
    _, [unlooked] = assembler.detected(1, None, FaceDetections.empty(), now=0.0)  # The next does not
    assert waiting == [] and (unlooked.frame_number, unlooked.hands) == (1, None)
    assembler = ResultsAssembler(SIZE, max_faces=3, mesh_every=None, hands_every=1)
    assembler.detected(2, None, FaceDetections.empty(), now=0.0)
    _, [empty] = assembler.palms(2, *palm_outputs(found=False), now=0.01)
    assert empty.hands == []

    assembler.detected(3, None, FaceDetections.empty(), now=0.0)
    assembler.palms(3, *palm_outputs(found=True), now=0.01)
    [weak] = assembler.hand(3, np.zeros(63, np.float32), presence=0.2, handedness=0.9, now=0.02)
    assert weak.hands == []  # The palm was there, but the landmarker saw no hand
