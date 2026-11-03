"""The OAK's faces travel in the OAK's frame messages, with the camera's own JPEG."""
import asyncio

import cbor2
import numpy as np
import pytest
from aiohttp.test_utils import TestClient, TestServer

from ultraskelly.core.pubsub.bot_topics import (
    FaceObservation,
    FacesMessage,
    FacesTopic,
    FrameMessage,
    FrameTopic,
    PoseDataMessage,
    PoseDataTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.ui.face_stream import face_trackers
from ultraskelly.ui.runtime_server import encode_camera_frame

from test_runtime_server import connect_stream, frame, make_server, read_image_payload, receive_stream

LANDMARKS = np.array([[300, 150], [340, 150], [320, 175], [305, 200], [335, 200]], np.float32)


def face(**measured):
    return FaceObservation(box=(280.0, 110.0, 360.0, 220.0), score=0.9, landmarks=LANDMARKS, **measured)


def values(channel):
    return np.frombuffer(channel["data"], "<f4").reshape(-1, len(channel["columns"]))


def test_each_face_carries_its_box_landmarks_and_what_was_measured():
    mesh = np.arange(468 * 3, dtype=np.float32).reshape(468, 3)
    faces = FacesMessage(faces=[face(head_pose=(20.0, -5.0, 3.0), gaze=(0.0, 0.6, -0.8), mesh=mesh), face()],
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
        faces=FacesMessage(faces=[face()], image_size=(640, 480), frame_number=8),
    ))

    assert [tracker["tracker_id"] for tracker in decoded["trackers"]] == ["person-0", "face-0"]


def test_the_oak_frame_waits_for_its_faces_and_keeps_the_cameras_own_jpeg():
    async def scenario():
        bus = PubSubTopicManager.create()
        encoded = []
        server = make_server(bus, encoded=encoded, pose_camera="oak", faces_camera="oak",
                             oak_status=lambda: {"rates": {"face_detection": 30.0}})
        pump = asyncio.create_task(server.pump())
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await connect_stream(client, server)
            await bus.publish(FacesTopic, FacesMessage(faces=[], image_size=(640, 480), frame_number=40))  # Faces flow
            camera_frame = FrameMessage(frame=frame(7), frame_number=41, timestamp=12.5, jpeg=b"OAK JPEG")
            await bus.publish(FrameTopic, camera_frame)
            await bus.publish(PoseDataTopic, PoseDataMessage(keypoints=np.zeros((1, 17, 3), np.float32),
                                                             scores=np.array([0.8]), frame=camera_frame))
            await asyncio.sleep(0.03)  # The pose is in, the faces are still on their way
            await bus.publish(FacesTopic, FacesMessage(faces=[face()], image_size=(640, 480), frame_number=41))
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
    assert "faces" in state["rates"]
