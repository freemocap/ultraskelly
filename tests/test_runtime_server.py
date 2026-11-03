import asyncio
import struct

import cbor2
import numpy as np
import pytest
from aiohttp.test_utils import TestClient, TestServer

from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder
from ultraskelly.core.pubsub.bot_topics import (
    FrameMessage,
    FrameTopic,
    PoseDataMessage,
    PoseDataTopic,
    ServoStateMessage,
    ServoStateTopic,
    StereoFrameMessage,
    StereoFrameTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams, TrackingOverlay, encode_camera_frame


def make_server(bus, *, encoded, **options):
    recorder = CommandRecorder()
    recorder.record("pan", 100.0)

    def fake_encode(frame, quality):
        encoded.append(int(frame[0, 0, 0]))
        return b"JPEG%d" % int(frame[0, 0, 0])

    return RuntimeServerNode.create(
        pubsub=bus,
        params=RuntimeServerParams(state_hz=50),
        cameras={"oak": FrameTopic},
        recorder=recorder,
        motors_simulated=True,
        node_status=lambda: [{"name": "FakeNode", "state": "running"}],
        encode_jpeg=fake_encode,
        **options,
    )


def frame(value):
    return np.full((480, 640, 3), value, dtype=np.uint8)


def read_image_payload(payload: bytes) -> tuple[int, str, bytes]:
    """Frame number, first camera ID and its JPEG, read straight from SkellyCam's layout."""
    frame_number = struct.unpack_from("<q", payload, 8)[0]
    camera_id = payload[24 + 16:24 + 32].rstrip(b"\0").decode()
    length = struct.unpack_from("<i", payload, 24 + 48)[0]
    return frame_number, camera_id, payload[24 + 56:24 + 56 + length]


async def connect_stream(client, server):
    socket = await client.ws_connect("/stream")
    while server.clients == 0:
        await asyncio.sleep(0.005)
    return socket


async def receive_stream(socket, until) -> dict[str, list[dict]]:
    received: dict[str, list[dict]] = {"frame": [], "robot_state": []}
    while not until(received):
        message = await asyncio.wait_for(socket.receive(), timeout=5)
        decoded = cbor2.loads(message.data)
        received[decoded["kind"]].append(decoded)
    return received


def test_robot_state_reports_motor_commands_bot_state_guides_and_nodes():
    async def scenario():
        bus = PubSubTopicManager.create()
        server = make_server(bus, encoded=[], overlays={"oak": TrackingOverlay(aim=(320, 240), jaw_zone=100)})
        pump = asyncio.create_task(server.pump())
        await bus.publish(
            ServoStateTopic,
            ServoStateMessage(
                pan_angle=100, tilt_angle=80, roll_angle=90,
                is_locked_x=True, is_locked_y=False, is_locked_roll=False,
            ),
        )
        await bus.publish(TargetLocationTopic, TargetLocationMessage(x=330, y=200, angle=85.0))
        await bus.publish(FrameTopic, FrameMessage(frame=frame(1)))
        await asyncio.sleep(0.05)
        state = server.robot_state()
        server.stop()
        await pump
        bus.close()
        return state

    state = asyncio.run(scenario())

    assert state["motors"]["simulated"] is True
    assert state["motors"]["commands"]["pan"]["value"] == 100.0
    assert state["servo_state"]["pan"] == 100 and state["servo_state"]["locked_x"] is True
    assert (state["target"]["x"], state["target"]["y"]) == (330, 200)
    assert (state["cameras"]["oak"]["width"], state["cameras"]["oak"]["height"]) == (640, 480)
    assert state["overlays"] == {"oak": {"aim": [320, 240], "jaw_zone": 100}}
    assert state["nodes"] == [{"name": "FakeNode", "state": "running"}]
    assert "pose" not in state  # Poses travel with their image in frame messages


def test_dashboard_assets_are_served_locally_with_explicit_types():
    async def scenario():
        bus = PubSubTopicManager.create()
        server = make_server(bus, encoded=[])
        served = {}
        async with TestClient(TestServer(server.build_app())) as client:
            for path in ["/", "/dashboard.js", "/dashboard.css", "/scene.js", "/tiles.js", "/depth.js", "/depth3d.js", "/gaze.js", "/timeseries.js", "/depth-worker.js", "/colormap.js", "/heatmap.js", "/dom.js", "/frameclock.js", "/style.css", "/skeleton.js",
                         "/vendor/three.module.js", "/vendor/three.core.js", "/vendor/OrbitControls.js",
                         "/vendor/freemocap-client.js", "/vendor/freemocap-client.js.map"]:
                response = await client.get(path)
                served[path] = (response.status, response.content_type, len(await response.read()))
            retired = [(await client.get(path)).status for path in ("/api/state", "/ws", "/video/oak.mjpg")]
        bus.close()
        return served, retired

    served, retired = asyncio.run(scenario())

    assert all(status == 200 and size > 100 for status, _, size in served.values())
    assert served["/"][1] == "text/html"
    assert {served[path][1] for path in served if path.endswith(".js")} == {"text/javascript"}
    assert served["/vendor/freemocap-client.js.map"][1] == "application/json"
    assert retired == [404, 404, 404]  # JSON and MJPEG endpoints were replaced by /stream


def test_stream_sends_each_frame_with_its_own_poses_and_the_robot_state():
    async def scenario():
        bus = PubSubTopicManager.create()
        encoded = []
        server = make_server(bus, encoded=encoded, pose_camera="oak",
                             overlays={"oak": TrackingOverlay(aim=(320, 240), jaw_zone=100)})
        pump = asyncio.create_task(server.pump())
        await bus.publish(FrameTopic, FrameMessage(frame=frame(5)))
        await asyncio.sleep(0.05)
        encoded_unwatched = list(encoded)
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await connect_stream(client, server)
            camera_frame = FrameMessage(frame=frame(7), frame_number=41, timestamp=12.5)
            await bus.publish(FrameTopic, camera_frame)
            keypoints = np.zeros((2, 17, 3), np.float32)
            keypoints[0, 0], keypoints[1, 0] = [330, 200, 0.9], [100, 50, 0.4]
            await bus.publish(PoseDataTopic, PoseDataMessage(keypoints=keypoints, scores=np.array([0.8, 0.6]),
                                                             frame=camera_frame))
            received = await receive_stream(
                socket, lambda got: got["robot_state"] and any(message["trackers"] for message in got["frame"]))
            await socket.send_str("ping")
            pong = await asyncio.wait_for(socket.receive_str(), timeout=5)
            await socket.close()
        server.stop()
        await pump
        bus.close()
        return encoded_unwatched, received, pong

    encoded_unwatched, received, pong = asyncio.run(scenario())

    assert encoded_unwatched == []  # Nothing is encoded while nobody watches
    posed = next(message for message in received["frame"] if message["trackers"])
    assert (posed["frame_number"], posed["sequence"], posed["timestamp"]) == (41, 41, 12.5)
    assert posed["cameras"][0]["id"] == "oak" and posed["cameras"][0]["image_size"] == [640, 480]
    assert len(posed["trackers"]) == 2
    tracker = posed["trackers"][0]
    channel = tracker["channels"][0]
    assert (channel["kind"], channel["camera_id"], channel["names"][0], channel["image_size"]) == (
        "OVERLAY_2D", "oak", "nose", [640, 480])
    assert np.frombuffer(channel["data"], "<f4").reshape(17, 3)[0].tolist() == pytest.approx([330, 200, 0.9])
    assert tracker["model_id"] is None and tracker["connection_groups"][0]["name"] == "body"
    assert read_image_payload(posed["image"]) == (41, "oak", b"JPEG7")
    state = received["robot_state"][0]
    assert state["overlays"]["oak"] == {"aim": [320, 240], "jaw_zone": 100}
    assert state["motors"]["commands"]["pan"]["value"] == 100.0
    assert pong == "pong"


def read_payload_images(payload: bytes) -> list[tuple[str, int, int, int, bytes]]:
    """Each camera's ID, index, width, height and JPEG, read straight from SkellyCam's layout."""
    count = struct.unpack_from("<i", payload, 16)[0]
    images, offset = [], 24
    for _ in range(count):
        camera_id = payload[offset + 16:offset + 32].rstrip(b"\0").decode()
        index, width, height, _, length = struct.unpack_from("<iiiii", payload, offset + 32)
        images.append((camera_id, index, width, height, payload[offset + 56:offset + 56 + length]))
        offset += 56 + length
    return images


def test_a_stereo_pair_travels_as_one_frame_with_both_cameras():
    async def scenario():
        bus = PubSubTopicManager.create()
        server = make_server(bus, encoded=[], stereo_cameras={"zed": StereoFrameTopic})
        pump = asyncio.create_task(server.pump())
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await connect_stream(client, server)
            yuyv = np.full((376, 2 * 672, 2), 128, np.uint8)  # As the ZED sends it: the eyes' Y are 3 and 4
            yuyv[:, :672, 0], yuyv[:, 672:, 0] = 3, 4
            await bus.publish(StereoFrameTopic, StereoFrameMessage(yuyv=yuyv, frame_number=8, timestamp=2.5))
            received = await receive_stream(socket, lambda got: got["frame"] and any(
                state["cameras"]["zed_left"]["width"] for state in got["robot_state"]))
            await socket.close()
        server.stop()
        await pump
        bus.close()
        return received

    received = asyncio.run(scenario())

    pair = received["frame"][0]
    assert (pair["frame_number"], pair["timestamp"], pair["trackers"]) == (8, 2.5, [])
    assert [(camera["id"], camera["index"], camera["image_size"]) for camera in pair["cameras"]] == [
        ("zed_left", 1, [672, 376]), ("zed_right", 2, [672, 376])]  # After the OAK's index 0
    assert read_payload_images(pair["image"]) == [
        ("zed_left", 1, 672, 376, b"JPEG3"), ("zed_right", 2, 672, 376, b"JPEG4")]
    cameras = received["robot_state"][-1]["cameras"]
    assert set(cameras) == {"oak", "zed_left", "zed_right"}
    assert (cameras["zed_left"]["width"], cameras["zed_right"]["height"]) == (672, 376)


def test_stream_keeps_video_flowing_without_poses():
    async def scenario():
        bus = PubSubTopicManager.create()
        server = make_server(bus, encoded=[], pose_camera="oak")
        pump = asyncio.create_task(server.pump())
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await connect_stream(client, server)
            await bus.publish(FrameTopic, FrameMessage(frame=frame(9)))
            received = await receive_stream(socket, lambda got: got["frame"])
            await socket.close()
        server.stop()
        await pump
        bus.close()
        return received["frame"][0]

    bare = asyncio.run(scenario())

    assert bare["trackers"] == [] and bare["frame_number"] == 1
    assert read_image_payload(bare["image"])[1:] == ("oak", b"JPEG9")


def people(*noses):
    """Keypoints of one person per nose (x, y, confidence), every other keypoint zero."""
    keypoints = np.zeros((len(noses), 17, 3), np.float32)
    for row, nose in enumerate(noses):
        keypoints[row, 0] = nose
    return keypoints


def encode_people(keypoints, **pose):
    """One frame message carrying these people, decoded the way a viewer receives it."""
    return cbor2.loads(encode_camera_frame(
        [("oak", 0, frame(1))], frame_number=3, timestamp=1.0, poses=PoseDataMessage(keypoints=keypoints, **pose),
        encode_jpeg=lambda image, quality: b"JPEG", jpeg_quality=80,
    ))


def channel_values(channel):
    return np.frombuffer(channel["data"], "<f4").reshape(-1, len(channel["columns"]))


def test_people_get_declared_ids_and_the_followed_person_carries_the_target():
    target = TargetLocationMessage(x=330, y=200, angle=85.0)
    decoded = encode_people(people((10, 20, 0.9), (330, 200, 0.9), (500, 60, 0.8)), target=target, target_person=1)

    trackers = decoded["trackers"]
    assert [tracker["tracker_id"] for tracker in trackers] == ["person-0", "target", "person-2"]
    assert {tracker["detector_type"] for tracker in trackers} == {"yolov8_pose"}
    assert [len(tracker["channels"]) for tracker in trackers] == [1, 2, 1]
    channel = trackers[1]["channels"][1]
    assert (channel["kind"], channel["columns"], channel["names"], channel["camera_id"], channel["image_size"]) == (
        "ultraskelly.target", ["x", "y", "body_angle_deg"], ["target"], "oak", [640, 480])
    assert channel_values(channel).tolist() == [[330.0, 200.0, 85.0]]


def test_ids_follow_the_people_when_their_order_changes():
    first, second = (10, 20, 0.9), (330, 200, 0.9)
    target = TargetLocationMessage(x=330, y=200, angle=0.0)
    before = encode_people(people(first, second), target=target, target_person=1)
    after = encode_people(people(second, first), target=target, target_person=0)

    def nose_of(decoded, tracker_id):
        tracker = next(tracker for tracker in decoded["trackers"] if tracker["tracker_id"] == tracker_id)
        return channel_values(tracker["channels"][0])[0].tolist()

    assert nose_of(before, "target") == nose_of(after, "target") == pytest.approx([330, 200, 0.9])
    assert [tracker["tracker_id"] for tracker in after["trackers"]] == ["target", "person-1"]


def test_without_a_followed_person_nobody_is_the_target():
    decoded = encode_people(people((10, 20, 0.9)), target=TargetLocationMessage(), target_person=None)

    assert [tracker["tracker_id"] for tracker in decoded["trackers"]] == ["person-0"]
    assert [channel["kind"] for channel in decoded["trackers"][0]["channels"]] == ["OVERLAY_2D"]


def test_a_target_without_a_body_angle_sends_nan():
    decoded = encode_people(people((330, 200, 0.9)), target=TargetLocationMessage(x=330, y=200), target_person=0)

    values = channel_values(decoded["trackers"][0]["channels"][1])[0]
    assert values[:2].tolist() == [330, 200] and np.isnan(values[2])


def test_stream_marks_the_followed_person_in_the_frame_they_were_found_in():
    async def scenario():
        bus = PubSubTopicManager.create()
        server = make_server(bus, encoded=[], pose_camera="oak")
        pump = asyncio.create_task(server.pump())
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await connect_stream(client, server)
            camera_frame = FrameMessage(frame=frame(9), frame_number=77, timestamp=3.5)
            await bus.publish(FrameTopic, camera_frame)
            target = TargetLocationMessage(x=100, y=50, angle=90.0)
            await bus.publish(PoseDataTopic, PoseDataMessage(
                keypoints=people((330, 200, 0.9), (100, 50, 0.9)), scores=np.array([0.6, 0.8]),
                frame=camera_frame, target=target, target_person=1))
            received = await receive_stream(socket, lambda got: any(message["trackers"] for message in got["frame"]))
            await socket.close()
        server.stop()
        await pump
        bus.close()
        return received

    received = asyncio.run(scenario())

    posed = next(message for message in received["frame"] if message["trackers"])
    assert posed["frame_number"] == 77
    assert [tracker["tracker_id"] for tracker in posed["trackers"]] == ["person-0", "target"]
    assert channel_values(posed["trackers"][1]["channels"][1]).tolist() == [[100.0, 50.0, 90.0]]
