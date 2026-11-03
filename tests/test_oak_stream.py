"""The OAK's faces travel in the OAK's frame messages, with the camera's own JPEG."""
import asyncio

import cbor2
import numpy as np
import pytest
from aiohttp.test_utils import TestClient, TestServer

from ultraskelly.core.pubsub.bot_topics import (
    FaceObservation,
    HandObservation,
    OakResultsMessage,
    OakResultsTopic,
    FrameMessage,
    FrameTopic,
    PoseDataMessage,
    PoseDataTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.ui.oak_stream import face_trackers, hand_trackers, oak_trackers
from ultraskelly.ui.runtime_server import encode_camera_frame

from test_runtime_server import connect_stream, frame, make_server, read_image_payload, receive_stream

LANDMARKS = np.array([[300, 150], [340, 150], [320, 175], [305, 200], [335, 200]], np.float32)


def face(**measured):
    return FaceObservation(box=(280.0, 110.0, 360.0, 220.0), score=0.9, landmarks=LANDMARKS, **measured)


def values(channel):
    return np.frombuffer(channel["data"], "<f4").reshape(-1, len(channel["columns"]))


def test_each_face_carries_its_box_landmarks_and_what_was_measured():
    mesh = np.arange(468 * 3, dtype=np.float32).reshape(468, 3)
    faces = OakResultsMessage(faces=[face(head_pose=(20.0, -5.0, 3.0), gaze=(0.0, 0.6, -0.8), mesh=mesh), face()],
                         image_size=(640, 480), frame_number=8)

    measured, bare = face_trackers("oak", faces)

    assert (measured["tracker_id"], measured["detector_type"], bare["tracker_id"]) == ("face-0", "yunet", "face-1")
    kinds = [channel["kind"] for channel in measured["channels"]]
    assert kinds == ["BOXES_2D", "OVERLAY_2D", "ultraskelly.head_pose", "ultraskelly.gaze", "ultraskelly.face_mesh"]
    assert [channel["kind"] for channel in bare["channels"]] == ["BOXES_2D", "OVERLAY_2D"]  # Nothing unmeasured is sent
    box, landmarks, head_pose, gaze, face_mesh = measured["channels"]
    assert box["columns"] == ["x1", "y1", "x2", "y2", "confidence", "detector_ran"]
    assert values(box).tolist() == [pytest.approx([280, 110, 360, 220, 0.9, 1])]
    assert landmarks["names"] == ["right_eye", "left_eye", "nose", "right_mouth", "left_mouth"]
    assert values(landmarks)[:, :2].tolist() == LANDMARKS.tolist()
    assert (landmarks["camera_id"], landmarks["image_size"]) == ("oak", [640, 480])
    assert head_pose["columns"] == ["yaw_deg", "pitch_deg", "roll_deg"] and values(head_pose).tolist() == [[20, -5, 3]]
    assert values(gaze).tolist() == [pytest.approx([0.0, 0.6, -0.8])]
    assert values(face_mesh).shape == (468, 3) and "names" not in face_mesh


def test_faces_and_people_share_the_frame_message_they_were_found_in():
    decoded = cbor2.loads(encode_camera_frame(
        [("oak", 0, frame(1))], frame_number=8, timestamp=1.0,
        poses=PoseDataMessage(keypoints=np.zeros((1, 17, 3), np.float32), scores=np.array([0.7])),
        encode_jpeg=lambda image, quality: b"JPEG", jpeg_quality=80,
        results=OakResultsMessage(faces=[face()], image_size=(640, 480), frame_number=8),
    ))

    assert [tracker["tracker_id"] for tracker in decoded["trackers"]] == ["person-0", "face-0"]


def test_each_hand_carries_its_palm_box_landmarks_depth_and_handedness():
    points = np.column_stack([np.arange(21) * 2.0, np.arange(21) * 3.0, np.arange(21) * -0.5]).astype(np.float32)
    results = OakResultsMessage(faces=[face()], image_size=(640, 480), frame_number=8, hands=[HandObservation(
        palm_box=(100.0, 200.0, 140.0, 240.0), palm_score=0.8, points=points, presence=0.95, handedness=0.3)])

    [hand] = hand_trackers("oak", results)

    assert (hand["tracker_id"], hand["detector_type"]) == ("hand-0", "mediapipe_hands")
    assert hand["connection_groups"][0]["name"] == "hand" and len(hand["connection_groups"][0]["pairs"]) == 20
    box, landmarks, depth, handedness = hand["channels"]
    assert values(box).tolist() == [pytest.approx([100, 200, 140, 240, 0.8, 1])]
    assert landmarks["names"][0] == "wrist" and landmarks["names"][-1] == "pinky_tip"
    assert values(landmarks)[:, :2].tolist() == points[:, :2].tolist()
    assert (depth["kind"], values(depth)[:, 0].tolist()) == ("ultraskelly.hand_depth", points[:, 2].tolist())
    assert (handedness["kind"], handedness["columns"]) == ("ultraskelly.handedness", ["right"])
    assert [tracker["tracker_id"] for tracker in oak_trackers("oak", results)] == ["face-0", "hand-0"]


def test_the_oak_frame_waits_for_its_faces_and_keeps_the_cameras_own_jpeg():
    async def scenario():
        bus = PubSubTopicManager.create()
        encoded, faces = [], {}
        server = make_server(bus, encoded=encoded, pose_camera="oak", results_camera="oak", results_for=faces.get,
                             oak_status=lambda: {"rates": {"face_detection": 30.0}})
        pump = asyncio.create_task(server.pump())
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await connect_stream(client, server)
            camera_frame = FrameMessage(frame=frame(7), frame_number=41, timestamp=12.5, jpeg=b"OAK JPEG")
            await bus.publish(FrameTopic, camera_frame)
            await bus.publish(PoseDataTopic, PoseDataMessage(keypoints=np.zeros((1, 17, 3), np.float32),
                                                             scores=np.array([0.8]), frame=camera_frame))
            await asyncio.sleep(0.03)  # The pose is in, the faces are still on their way
            faces[41] = OakResultsMessage(faces=[face()], image_size=(640, 480), frame_number=41)
            received = await receive_stream(
                socket, lambda got: got["robot_state"] and any(message["frame_number"] == 41 for message in got["frame"]))
            await socket.close()
        server.stop()
        await pump
        bus.close()
        return encoded, received

    encoded, received = asyncio.run(scenario())

    posed = next(message for message in received["frame"] if message["frame_number"] == 41)
    assert [tracker["tracker_id"] for tracker in posed["trackers"]] == ["person-0", "face-0"]
    assert read_image_payload(posed["image"]) == (41, "oak", b"OAK JPEG")
    assert encoded == []  # The Pi encoded nothing: the OAK had
    state = received["robot_state"][-1]
    assert state["oak"] == {"rates": {"face_detection": 30.0}}


def stream_frames(faces_after: dict[int, float | None], count: int = 12) -> list[dict]:
    """Frames and poses at 30 fps; each frame's faces arrive `faces_after[number]` seconds after its
    pose (never for None). Returns the frame messages a viewer received."""
    async def scenario():
        bus = PubSubTopicManager.create()
        faces = {}
        server = make_server(bus, encoded=[], pose_camera="oak", results_camera="oak", results_for=faces.get)
        pump = asyncio.create_task(server.pump())

        async def faces_later(number, delay):
            await asyncio.sleep(delay)
            box = (float(number), 110.0, 360.0, 220.0)
            faces[number] = OakResultsMessage(faces=[FaceObservation(box=box, score=0.9, landmarks=LANDMARKS)],
                                         image_size=(640, 480), frame_number=number)

        async with TestClient(TestServer(server.build_app())) as client:
            socket = await connect_stream(client, server)
            for number in range(1, count + 1):
                camera_frame = FrameMessage(frame=frame(number), frame_number=number, timestamp=float(number))
                await bus.publish(FrameTopic, camera_frame)
                await bus.publish(PoseDataTopic, PoseDataMessage(keypoints=np.zeros((1, 17, 3), np.float32),
                                                                 scores=np.array([0.8]), frame=camera_frame))
                if faces_after.get(number, 0.0) is not None:
                    asyncio.create_task(faces_later(number, faces_after.get(number, 0.0)))
                await asyncio.sleep(1 / 30)
            received = await receive_stream(socket, lambda got: any(m["frame_number"] == count for m in got["frame"]))
            await socket.close()
        server.stop()
        await pump
        bus.close()
        return received["frame"]

    return asyncio.run(scenario())


def face_box_x(message):
    tracker = next(tracker for tracker in message["trackers"] if tracker["tracker_id"] == "face-0")
    return values(tracker["channels"][0])[0, 0]


def test_frames_keep_flowing_with_their_own_faces_when_faces_lag_the_poses():
    # Every other frame's faces take 50 ms, longer than a frame: they are measured, the others not.
    frames = stream_frames({number: 0.05 if number % 2 else 0.005 for number in range(1, 13)})

    numbers = [message["frame_number"] for message in frames]
    assert len(with_faces(frames)) >= 11 and numbers == sorted(numbers)  # The slow frames are not skipped
    assert all(face_box_x(message) == message["frame_number"] for message in with_faces(frames))  # Never a neighbour's


def test_a_frame_whose_faces_never_come_goes_bare_after_the_wait_and_the_rest_follow():
    frames = stream_frames({3: None})

    assert 3 in [message["frame_number"] for message in frames] and 3 not in [m["frame_number"] for m in with_faces(frames)]
    # The frames held behind it catch up newest-wins, then every frame comes with its own faces again.
    assert {9, 10, 11, 12} <= {message["frame_number"] for message in with_faces(frames)}
    assert all(face_box_x(message) == message["frame_number"] for message in with_faces(frames))


def with_faces(frames):
    return [message for message in frames if any(tracker["tracker_id"] == "face-0" for tracker in message["trackers"])]
