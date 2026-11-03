"""What the OAK's chip finds, as FreeMoCap tracker observations in the OAK's frame messages.

Each face is a tracker `face-<row>` (best YuNet score first) carrying, in pixels of the frame
it was found in: its box (BOXES_2D), YuNet's five landmarks (OVERLAY_2D, so any FreeMoCap
viewer draws them), and when measured, custom numeric channels for head pose, gaze and
MediaPipe's 468 face landmarks. Each hand, on the frames hands are looked for, is a tracker
`hand-<row>` with its palm box and MediaPipe's 21 hand landmarks (OVERLAY_2D) and a
handedness channel. docs/stream.md describes every column.
"""
import numpy as np
from freemocap_stream import BOXES_2D, OVERLAY_2D, channel_block, connection_group, tracker_observation

from ultraskelly.core.bot.sensory.mediapipe_hands import HAND_BONES, HAND_POINTS
from ultraskelly.core.bot.sensory.yunet import LANDMARKS
from ultraskelly.core.pubsub.bot_topics import OakResultsMessage

FACE_DETECTOR, HAND_DETECTOR = "yunet", "mediapipe_hands"
BOX_COLUMNS = ("x1", "y1", "x2", "y2", "confidence", "detector_ran")
LANDMARK_COLUMNS = ("x", "y", "visibility")
HEAD_POSE_CHANNEL, HEAD_POSE_COLUMNS = "ultraskelly.head_pose", ("yaw_deg", "pitch_deg", "roll_deg")
GAZE_CHANNEL, GAZE_COLUMNS = "ultraskelly.gaze", ("x", "y", "z")
MESH_CHANNEL, MESH_COLUMNS = "ultraskelly.face_mesh", ("x", "y", "z")
HAND_DEPTH_CHANNEL = "ultraskelly.hand_depth"
HANDEDNESS_CHANNEL, HANDEDNESS_COLUMNS = "ultraskelly.handedness", ("right",)
FACE = connection_group("face", [("right_eye", "nose"), ("left_eye", "nose"), ("nose", "right_mouth"),
                                 ("nose", "left_mouth"), ("right_mouth", "left_mouth")], "#4fc3f7")
HAND = connection_group("hand", [(HAND_POINTS[start], HAND_POINTS[end]) for start, end in HAND_BONES], "#ffb74d")


def oak_trackers(camera: str, results: OakResultsMessage | None) -> list[dict]:
    """One tracker observation per face and per hand, in pixels of `camera`'s image."""
    if results is None:
        return []
    return face_trackers(camera, results) + hand_trackers(camera, results)


def face_trackers(camera: str, results: OakResultsMessage) -> list[dict]:
    size = results.image_size
    trackers = []
    for row, face in enumerate(results.faces):
        channels = [
            channel_block(BOXES_2D, BOX_COLUMNS, [[*face.box, face.score, 1.0]], names=["face"],
                          camera_id=camera, image_size=size),
            channel_block(OVERLAY_2D, LANDMARK_COLUMNS,
                          np.column_stack([face.landmarks, np.full(len(LANDMARKS), face.score)]),
                          names=list(LANDMARKS), camera_id=camera, image_size=size),
        ]
        if face.head_pose is not None:
            channels.append(channel_block(HEAD_POSE_CHANNEL, HEAD_POSE_COLUMNS, [face.head_pose], names=["head"],
                                          camera_id=camera, image_size=size))
        if face.gaze is not None:
            channels.append(channel_block(GAZE_CHANNEL, GAZE_COLUMNS, [face.gaze], names=["gaze"],
                                          camera_id=camera, image_size=size))
        if face.mesh is not None:
            channels.append(channel_block(MESH_CHANNEL, MESH_COLUMNS, face.mesh, camera_id=camera, image_size=size))
        trackers.append(tracker_observation(f"face-{row}", channels, detector_type=FACE_DETECTOR, connection_groups=[FACE]))
    return trackers


def hand_trackers(camera: str, results: OakResultsMessage) -> list[dict]:
    size = results.image_size
    trackers = []
    for row, hand in enumerate(results.hands or []):
        channels = [
            channel_block(BOXES_2D, BOX_COLUMNS, [[*hand.palm_box, hand.palm_score, 1.0]], names=["palm"],
                          camera_id=camera, image_size=size),
            channel_block(OVERLAY_2D, LANDMARK_COLUMNS,
                          np.column_stack([hand.points[:, :2], np.full(len(HAND_POINTS), hand.presence)]),
                          names=list(HAND_POINTS), camera_id=camera, image_size=size),
            channel_block(HAND_DEPTH_CHANNEL, ("z",), hand.points[:, 2:], names=list(HAND_POINTS),
                          camera_id=camera, image_size=size),
            channel_block(HANDEDNESS_CHANNEL, HANDEDNESS_COLUMNS, [[hand.handedness]], names=["hand"],
                          camera_id=camera, image_size=size),
        ]
        trackers.append(tracker_observation(f"hand-{row}", channels, detector_type=HAND_DETECTOR, connection_groups=[HAND]))
    return trackers
