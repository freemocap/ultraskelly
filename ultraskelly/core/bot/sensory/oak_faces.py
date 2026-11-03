"""The OAK's face chain, Pi side: which crops the OAK cuts for each face, and reading what comes back.

YuNet finds faces on the OAK. The Pi decodes them (`yunet.decode_yunet`) and sends back, for
the same frame, the boxes the OAK's Script node cuts out of it (`oak_script.py`): the face for
head pose, each eye for gaze, and the nearest face with a margin for MediaPipe's face
landmarks. Crops follow Luxonis' own gaze and face landmark examples (oak-examples,
neural-networks/face-detection): the face box as found, and eye boxes a quarter of the face
in size, centred on YuNet's eye points. No hardware imports.
"""
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from ultraskelly.core.bot.sensory.oak_script import RESULT_FIELDS
from ultraskelly.core.bot.sensory.yunet import FaceDetections
from ultraskelly.core.pubsub.bot_topics import FaceObservation, FacesMessage

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


@dataclass(frozen=True)
class Crop:
    """One box for the OAK to cut out of a frame, normalized x1, y1, x2, y2, for face `face`."""

    kind: CropKind
    face: int
    box: tuple[float, float, float, float]


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


def face_crops(faces: FaceDetections, image_size: tuple[int, int], *, max_faces: int, mesh: bool) -> list[Crop]:
    """The crops for the best `max_faces` faces, in face order: head pose, left eye, right eye,
    and for the largest face, when `mesh`, the face landmark crop last."""
    crops = []
    for face in range(min(faces.count, max_faces)):
        box, landmarks = faces.boxes[face], faces.landmarks[face]
        crops.append(Crop(CropKind.HEAD_POSE, face, clip_box(box)))
        crops.append(Crop(CropKind.LEFT_EYE, face, clip_box(eye_box(landmarks[1], box))))
        crops.append(Crop(CropKind.RIGHT_EYE, face, clip_box(eye_box(landmarks[0], box))))
    if mesh and faces.count:
        areas = (faces.boxes[:, 2] - faces.boxes[:, 0]) * (faces.boxes[:, 3] - faces.boxes[:, 1])
        largest = int(np.argmax(areas[:max_faces]))
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


@dataclass
class Pending:
    """One frame's faces, waiting for what the OAK measures on them."""

    sequence: int
    timestamp: float | None
    faces: FaceDetections
    mesh_crop: Crop | None
    since: float
    results: np.ndarray | None = None  # (faces, RESULT_FIELDS) from the Script node
    mesh: np.ndarray | None = None  # (468, 3) image pixels


class FaceAssembler:
    """Joins, frame by frame, YuNet's faces with the head pose, gaze and face landmarks the OAK
    sends back for them, and says which crops to ask for. A frame whose answers do not all come
    within `timeout` seconds is published with what did."""

    def __init__(self, image_size: tuple[int, int], *, max_faces: int = 3, mesh: bool = True, timeout: float = 0.5) -> None:
        self.image_size, self.max_faces, self.mesh, self.timeout = image_size, max_faces, mesh, timeout
        self.pending: dict[int, Pending] = {}
        self.incomplete = 0  # Frames published without every answer

    def detected(
        self, sequence: int, timestamp: float | None, faces: FaceDetections, now: float,
    ) -> tuple[list[Crop], list[FacesMessage]]:
        """A frame's faces: the crops to request, and any frames now finished (this one, if it has no faces)."""
        crops = face_crops(faces, self.image_size, max_faces=self.max_faces, mesh=self.mesh)
        mesh_crop = next((crop for crop in crops if crop.kind == CropKind.MESH), None)
        self.pending[sequence] = Pending(sequence, timestamp, faces, mesh_crop, now)
        return crops, self.finished(now)

    def answered(self, sequence: int, now: float, *, results: np.ndarray | None = None,
                 mesh: np.ndarray | None = None) -> list[FacesMessage]:
        """The Script's per-face results or the face landmark network's output for a frame."""
        frame = self.pending.get(sequence)
        if frame is not None:
            if results is not None:
                frame.results = np.asarray(results, np.float32).reshape(-1, RESULT_FIELDS)
            if mesh is not None and frame.mesh_crop is not None:
                frame.mesh = mesh_points(mesh, frame.mesh_crop.box, self.image_size)
        return self.finished(now)

    def complete(self, frame: Pending) -> bool:
        if frame.faces.count == 0:
            return True
        return frame.results is not None and (frame.mesh_crop is None or frame.mesh is not None)

    def finished(self, now: float) -> list[FacesMessage]:
        """Frames done or given up on, in frame order: a frame waits for every older one."""
        messages = []
        for sequence in sorted(self.pending):
            frame = self.pending[sequence]
            if not self.complete(frame) and now - frame.since <= self.timeout:
                break
            del self.pending[sequence]
            self.incomplete += not self.complete(frame)
            messages.append(self.message(frame))
        return messages

    def message(self, frame: Pending) -> FacesMessage:
        width, height = self.image_size
        scale = np.array([width, height], np.float32)
        measured = {}
        if frame.results is not None:
            for row in frame.results:
                if np.isfinite(row[0]):
                    measured[int(row[0])] = row
        faces = []
        for face in range(frame.faces.count):
            row = measured.get(face)
            angles = row[1:4] if row is not None and np.all(np.isfinite(row[1:4])) else None
            gaze = unit(row[4:7]) if row is not None and np.all(np.isfinite(row[4:7])) else None
            mesh = frame.mesh if frame.mesh_crop is not None and frame.mesh_crop.face == face else None
            faces.append(FaceObservation(
                box=tuple(float(value) for value in frame.faces.boxes[face] * np.tile(scale, 2)),
                score=float(frame.faces.scores[face]),
                landmarks=frame.faces.landmarks[face] * scale,
                head_pose=None if angles is None else tuple(float(value) for value in angles),
                gaze=gaze,
                mesh=mesh,
            ))
        return FacesMessage(faces=faces, image_size=self.image_size, frame_number=frame.sequence, timestamp=frame.timestamp)
