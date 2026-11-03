"""Joining, frame by frame, what the models on the OAK's own chip find: faces with their head
pose, gaze and face landmarks, and on some frames hands. Pi side; no hardware imports.

YuNet's faces for a frame arrive first. The assembler answers with the crops to ask the OAK
for (`oak_faces.face_crops`), and on the frames hands are looked for, the palm detector's crop
too. Each palm found becomes a hand crop, asked for in a second request for the same frame. A
frame is published as soon as every answer it waits for is in, or after `timeout` seconds
with what came.
"""
from dataclasses import dataclass, field

import numpy as np

from ultraskelly.core.bot.sensory.mediapipe_hands import HandCrop, Palms, decode_palms, hand_crop, hand_points
from ultraskelly.core.bot.sensory.oak_faces import Crop, CropKind, face_crops, mesh_points, unit
from ultraskelly.core.bot.sensory.oak_script import RESULT_FIELDS
from ultraskelly.core.bot.sensory.yunet import FaceDetections
from ultraskelly.core.pubsub.bot_topics import FaceObservation, HandObservation, OakResultsMessage


@dataclass
class Pending:
    """One frame's faces, waiting for what the OAK measures on them, and maybe for its hands."""

    sequence: int
    timestamp: float | None
    faces: FaceDetections
    since: float
    mesh_crop: Crop | None = None
    awaits_results: bool = False  # Head pose crops were asked for, so the Script answers
    results: np.ndarray | None = None  # (faces, RESULT_FIELDS) from the Script node
    mesh: np.ndarray | None = None  # (468, 3) image pixels
    awaits_palms: bool = False  # The palm detector was asked to look at this frame
    palms: Palms | None = None
    hand_crops: list[HandCrop] = field(default_factory=list)
    hands: list[tuple[np.ndarray, float, float]] = field(default_factory=list)  # Points, presence, handedness


class ResultsAssembler:
    """Says which crops to ask the OAK for and joins its answers into one message per frame.

    Head pose and gaze are measured in one face detection of `every`, face landmarks in one of
    `mesh_every`, and hands looked for in one of `hands_every` (None: never); the other frames'
    faces carry their boxes and landmarks only."""

    def __init__(
        self, image_size: tuple[int, int], *, max_faces: int = 3, every: int = 1, mesh_every: int | None = 1,
        hands_every: int | None = None, max_hands: int = 2, hand_threshold: float = 0.5, timeout: float = 0.5,
    ) -> None:
        self.image_size, self.max_faces, self.timeout = image_size, max_faces, timeout
        self.every, self.mesh_every, self.hands_every = every, mesh_every, hands_every
        self.max_hands, self.hand_threshold = max_hands, hand_threshold
        self.detections = 0
        self.pending: dict[int, Pending] = {}
        self.incomplete = 0  # Frames published without every answer

    def detected(
        self, sequence: int, timestamp: float | None, faces: FaceDetections, now: float,
    ) -> tuple[list[Crop], list[OakResultsMessage]]:
        """A frame's faces: the crops to request, and any frames now finished (this one, if nothing is measured in it)."""
        head_pose = self.detections % self.every == 0
        mesh = self.mesh_every is not None and self.detections % self.mesh_every == 0
        hands = self.hands_every is not None and self.detections % self.hands_every == 0
        self.detections += 1
        crops = face_crops(faces, self.image_size, max_faces=self.max_faces, mesh=mesh, head_pose=head_pose)
        if hands:
            crops.append(Crop(CropKind.PALM, 0))
        self.pending[sequence] = Pending(
            sequence, timestamp, faces, now,
            mesh_crop=next((crop for crop in crops if crop.kind == CropKind.MESH), None),
            awaits_results=any(crop.kind == CropKind.HEAD_POSE for crop in crops),
            awaits_palms=hands,
        )
        return crops, self.finished(now)

    def answered(self, sequence: int, now: float, *, results: np.ndarray | None = None,
                 mesh: np.ndarray | None = None) -> list[OakResultsMessage]:
        """The Script's per-face results or the face landmark network's output for a frame."""
        frame = self.pending.get(sequence)
        if frame is not None:
            if results is not None:
                frame.results = np.asarray(results, np.float32).reshape(-1, RESULT_FIELDS)
            if mesh is not None and frame.mesh_crop is not None:
                frame.mesh = mesh_points(mesh, frame.mesh_crop.box, self.image_size)
        return self.finished(now)

    def palms(self, sequence: int, regressors: np.ndarray, scores: np.ndarray,
              now: float) -> tuple[list[Crop], list[OakResultsMessage]]:
        """The palm detector's output for a frame: a hand crop to request per palm, and finished frames."""
        frame = self.pending.get(sequence)
        crops = []
        if frame is not None and frame.awaits_palms and frame.palms is None:
            frame.palms = decode_palms(regressors, scores, image_size=self.image_size, max_palms=self.max_hands)
            frame.hand_crops = [hand_crop(box, keypoints) for box, keypoints in zip(frame.palms.boxes, frame.palms.keypoints)]
            crops = [Crop(CropKind.HAND, row, hand=hand) for row, hand in enumerate(frame.hand_crops)]
        return crops, self.finished(now)

    def hand(self, sequence: int, points: np.ndarray, presence: float, handedness: float,
             now: float) -> list[OakResultsMessage]:
        """The hand landmarker's output for the frame's next hand crop, in the order they were asked for."""
        frame = self.pending.get(sequence)
        if frame is not None and len(frame.hands) < len(frame.hand_crops):
            crop = frame.hand_crops[len(frame.hands)]
            frame.hands.append((hand_points(points, crop), float(presence), float(handedness)))
        return self.finished(now)

    def complete(self, frame: Pending) -> bool:
        faces = (not frame.awaits_results or frame.results is not None) and (frame.mesh_crop is None or frame.mesh is not None)
        hands = not frame.awaits_palms or (frame.palms is not None and len(frame.hands) == len(frame.hand_crops))
        return faces and hands

    def finished(self, now: float) -> list[OakResultsMessage]:
        """Frames done or given up on, oldest first. A done frame need not wait for an older
        one: consumers find a frame's results by its number."""
        messages = []
        for sequence in sorted(self.pending):
            frame = self.pending[sequence]
            if self.complete(frame) or now - frame.since > self.timeout:
                del self.pending[sequence]
                self.incomplete += not self.complete(frame)
                messages.append(self.message(frame))
        return messages

    def message(self, frame: Pending) -> OakResultsMessage:
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
            mesh = frame.mesh if frame.mesh_crop is not None and frame.mesh_crop.row == face else None
            faces.append(FaceObservation(
                box=tuple(float(value) for value in frame.faces.boxes[face] * np.tile(scale, 2)),
                score=float(frame.faces.scores[face]),
                landmarks=frame.faces.landmarks[face] * scale,
                head_pose=None if angles is None else tuple(float(value) for value in angles),
                gaze=gaze,
                mesh=mesh,
            ))
        hands = None
        if frame.awaits_palms:
            hands = [
                HandObservation(palm_box=tuple(float(value) for value in frame.palms.boxes[row]),
                                palm_score=float(frame.palms.scores[row]), points=points,
                                presence=presence, handedness=handedness)
                for row, (points, presence, handedness) in enumerate(frame.hands)
                if presence >= self.hand_threshold
            ]
        return OakResultsMessage(faces=faces, hands=hands, image_size=self.image_size,
                                 frame_number=frame.sequence, timestamp=frame.timestamp)
