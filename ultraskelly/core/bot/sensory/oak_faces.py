"""The OAK's crops, Pi side: which parts of a frame the OAK cuts out for its second-stage models.

YuNet finds faces on the OAK. The Pi decodes them (`yunet.decode_yunet`) and sends back, for
the same frame, the boxes the OAK's Script node cuts out of it (`oak_script.py`): the face for
head pose, each eye for gaze, and the nearest face with a margin for MediaPipe's face
landmarks. Crops follow Luxonis' own gaze and face landmark examples (oak-examples,
neural-networks/face-detection): the face box as found, and eye boxes a quarter of the face
in size, centred on YuNet's eye points. On frames where hands are looked for, the whole frame
also goes, letterboxed, to the palm detector, and each palm it finds comes back as a turned
square for the hand landmarker (`mediapipe_hands.py`). No hardware imports.
"""
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from ultraskelly.core.bot.sensory.mediapipe_hands import HandCrop
from ultraskelly.core.bot.sensory.yunet import FaceDetections

EYE_CROP = 0.25  # Eye box size as a fraction of the face box, as in Luxonis' gaze example
MESH_MARGIN = 1.25  # The face landmark crop: a square this many times the face box's longer side
MESH_POINTS = 468
MESH_INPUT = 192  # The face landmark network's input side, in pixels


class CropKind(IntEnum):
    """What a crop feeds; it travels as the crop request's label."""

    HEAD_POSE = 0
    LEFT_EYE = 1  # The person's left eye, YuNet's second landmark
    RIGHT_EYE = 2
    MESH = 3
    PALM = 4  # The whole frame, letterboxed for the palm detector
    HAND = 5  # A turned square around one palm, for the hand landmarker


@dataclass(frozen=True)
class Crop:
    """One part of a frame for the OAK to cut out: a normalized box x1, y1, x2, y2, or for a hand
    the turned square `hand` in pixels; `row` is the face's or hand's row."""

    kind: CropKind
    row: int
    box: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0)
    hand: HandCrop | None = None


def clip_box(box: np.ndarray) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = (float(value) for value in np.clip(box, 0.0, 1.0))
    return x1, y1, x2, y2


def mesh_box(box: np.ndarray, image_size: tuple[int, int]) -> np.ndarray:
    """A square in pixels, MESH_MARGIN times the face box's longer side, centred on it; normalized.
    It may reach past the image's edge."""
    width, height = image_size
    x1, y1, x2, y2 = box * (width, height, width, height)
    side = MESH_MARGIN * max(x2 - x1, y2 - y1)
    centre_x, centre_y = (x1 + x2) / 2, (y1 + y2) / 2
    return np.array([(centre_x - side / 2) / width, (centre_y - side / 2) / height,
                     (centre_x + side / 2) / width, (centre_y + side / 2) / height])


def eye_box(eye: np.ndarray, face_box: np.ndarray) -> np.ndarray:
    """A box EYE_CROP the face box's size, centred on the eye point; normalized."""
    half = EYE_CROP * (face_box[2:] - face_box[:2]) / 2
    return np.concatenate([eye - half, eye + half])


def face_crops(
    faces: FaceDetections, image_size: tuple[int, int], *, max_faces: int, mesh: bool, head_pose: bool = True,
) -> list[Crop]:
    """The crops for the best `max_faces` faces, in face order: with `head_pose`, head pose, left
    eye and right eye for each; with `mesh`, the face landmark crop of the largest, last."""
    crops = []
    measured = min(faces.count, max_faces)
    for face in range(measured if head_pose else 0):
        box, landmarks = faces.boxes[face], faces.landmarks[face]
        crops.append(Crop(CropKind.HEAD_POSE, face, clip_box(box)))
        crops.append(Crop(CropKind.LEFT_EYE, face, clip_box(eye_box(landmarks[1], box))))
        crops.append(Crop(CropKind.RIGHT_EYE, face, clip_box(eye_box(landmarks[0], box))))
    if mesh and measured:
        areas = (faces.boxes[:, 2] - faces.boxes[:, 0]) * (faces.boxes[:, 3] - faces.boxes[:, 1])
        largest = int(np.argmax(areas[:measured]))
        # Clipped at the image's edge: the crop is then not square, and mesh_points maps it back as cut.
        crops.append(Crop(CropKind.MESH, largest, clip_box(mesh_box(faces.boxes[largest], image_size))))
    return crops


def mesh_points(values: np.ndarray, crop_box: tuple[float, float, float, float], image_size: tuple[int, int]) -> np.ndarray:
    """The face landmark network's 468 (x, y, z) in input pixels, as image pixels (468, 3).
    z keeps the network's scale, relative depth with negative toward the camera, in image pixels."""
    points = np.asarray(values, np.float32).reshape(MESH_POINTS, 3)
    width, height = image_size
    x1, y1, x2, y2 = crop_box
    scale_x, scale_y = (x2 - x1) * width / MESH_INPUT, (y2 - y1) * height / MESH_INPUT
    return np.stack([x1 * width + points[:, 0] * scale_x, y1 * height + points[:, 1] * scale_y,
                     points[:, 2] * scale_x], axis=1)


def unit(vector: np.ndarray) -> tuple[float, float, float] | None:
    """`vector` scaled to length 1, or None when it has no direction."""
    length = float(np.linalg.norm(vector))
    if not np.isfinite(length) or length < 1e-6:
        return None
    x, y, z = (float(value) / length for value in vector)
    return x, y, z
