"""The Pi's side of the OAK's face chain: which crops to ask for, and joining what comes back."""
import numpy as np
import pytest

from ultraskelly.core.bot.sensory.oak_faces import (
    EYE_CROP,
    MESH_MARGIN,
    CropKind,
    FaceAssembler,
    face_crops,
    mesh_points,
)
from ultraskelly.core.bot.sensory.yunet import FaceDetections

SIZE = (640, 480)


def detections(*faces):
    """FaceDetections from (box, right eye, left eye, score), normalized."""
    boxes = np.array([box for box, _, _, _ in faces], np.float32).reshape(-1, 4)
    landmarks = np.zeros((len(faces), 5, 2), np.float32)
    for row, (_, right_eye, left_eye, _) in enumerate(faces):
        landmarks[row, 0], landmarks[row, 1] = right_eye, left_eye
    return FaceDetections(boxes=boxes, landmarks=landmarks, scores=np.array([score for *_, score in faces], np.float32))


NEAR = ((0.4, 0.2, 0.6, 0.5), (0.45, 0.3), (0.55, 0.3), 0.9)  # 128 x 144 px
FAR = ((0.1, 0.1, 0.15, 0.17), (0.11, 0.12), (0.14, 0.12), 0.8)  # 32 x 34 px


def test_each_face_gets_its_box_for_head_pose_and_eye_boxes_a_quarter_its_size():
    crops = face_crops(detections(NEAR, FAR), SIZE, max_faces=3, mesh=False)

    assert [(crop.kind, crop.face) for crop in crops] == [
        (CropKind.HEAD_POSE, 0), (CropKind.LEFT_EYE, 0), (CropKind.RIGHT_EYE, 0),
        (CropKind.HEAD_POSE, 1), (CropKind.LEFT_EYE, 1), (CropKind.RIGHT_EYE, 1),
    ]
    assert crops[0].box == pytest.approx(NEAR[0])
    left_eye = crops[1].box
    assert ((left_eye[0] + left_eye[2]) / 2, (left_eye[1] + left_eye[3]) / 2) == pytest.approx(NEAR[2])
    assert (left_eye[2] - left_eye[0], left_eye[3] - left_eye[1]) == pytest.approx((EYE_CROP * 0.2, EYE_CROP * 0.3))


def test_the_largest_face_alone_gets_a_square_landmark_crop_with_a_margin():
    crops = face_crops(detections(FAR, NEAR), SIZE, max_faces=1, mesh=True)

    assert [crop.kind for crop in crops] == [CropKind.HEAD_POSE, CropKind.LEFT_EYE, CropKind.RIGHT_EYE, CropKind.MESH]
    assert crops[0].face == 0 and crops[3].face == 0  # Only the best face is measured; the mesh goes to it too
    both = face_crops(detections(FAR, NEAR), SIZE, max_faces=2, mesh=True)
    mesh = both[-1]
    assert mesh.face == 1
    x1, y1, x2, y2 = np.array(mesh.box) * (640, 480, 640, 480)
    assert x2 - x1 == pytest.approx(y2 - y1) == pytest.approx(MESH_MARGIN * 144)  # Square in pixels
    assert ((x1 + x2) / 2, (y1 + y2) / 2) == pytest.approx((320, 168))


def test_crops_near_the_edge_stay_inside_the_image():
    edge = ((0.0, 0.0, 0.1, 0.15), (0.01, 0.02), (0.06, 0.02), 0.9)
    for crop in face_crops(detections(edge), SIZE, max_faces=1, mesh=True):
        assert all(0.0 <= value <= 1.0 for value in crop.box)


def test_mesh_points_map_from_the_network_input_back_to_image_pixels():
    points = np.zeros((468, 3), np.float32)
    points[0], points[1] = [0, 0, 0], [192, 96, -19.2]

    mapped = mesh_points(points.reshape(1, 1, 1, 1404), (0.25, 0.25, 0.5, 0.75), SIZE)

    assert mapped[0] == pytest.approx([160, 120, 0])
    assert mapped[1] == pytest.approx([320, 240, -16])  # z scales like x: 160 px wide crop / 192


def results(*rows):
    return np.array(rows, np.float32)


def test_a_frame_waits_for_its_answers_and_carries_them_per_face():
    assembler = FaceAssembler(SIZE, max_faces=3)
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
    assembler = FaceAssembler(SIZE, max_faces=3, mesh_every=None)
    assembler.detected(20, None, detections(NEAR), now=0.0)

    _, done = assembler.detected(21, None, FaceDetections.empty(), now=0.03)
    assert [(message.frame_number, message.faces) for message in done] == [(21, [])]  # Found by its number

    [measured] = assembler.answered(20, 0.05, results=results([0, 1, 2, 3, 0, 0, -1]))
    assert measured.frame_number == 20 and measured.faces[0].head_pose == pytest.approx((1, 2, 3))


def test_head_pose_and_landmarks_are_measured_at_their_own_rates():
    assembler = FaceAssembler(SIZE, max_faces=3, every=2, mesh_every=4)
    kinds = [{crop.kind for crop in assembler.detected(number, None, detections(NEAR), now=0.0)[0]} for number in range(4)]

    assert kinds == [{CropKind.HEAD_POSE, CropKind.LEFT_EYE, CropKind.RIGHT_EYE, CropKind.MESH}, set(),
                     {CropKind.HEAD_POSE, CropKind.LEFT_EYE, CropKind.RIGHT_EYE}, set()]
    assert [number for number in assembler.pending] == [0, 2]  # The unmeasured frames went out with their boxes


def test_measuring_no_faces_asks_for_nothing():
    assembler = FaceAssembler(SIZE, max_faces=0)
    crops, [message] = assembler.detected(5, None, detections(NEAR), now=0.0)

    assert crops == [] and message.faces[0].head_pose is None and message.faces[0].box == pytest.approx((256, 96, 384, 240))


def test_unanswered_frames_go_out_after_the_timeout_with_what_came():
    assembler = FaceAssembler(SIZE, max_faces=3, timeout=0.5)
    assembler.detected(30, None, detections(NEAR), now=0.0)
    assembler.answered(30, 0.1, results=results([0, 5, 6, 7, 1, 0, 0]))
    assert assembler.finished(0.4) == []  # Still waiting for the face landmarks

    [message] = assembler.finished(0.6)
    assert message.faces[0].head_pose == pytest.approx((5, 6, 7)) and message.faces[0].mesh is None
    assert assembler.incomplete == 1
    assert assembler.answered(30, 0.7, mesh=np.zeros(1404)) == []  # A late answer finds nothing waiting
