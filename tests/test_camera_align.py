"""Aligning the OAK to the ZED on the person standing in front of the robot (the dashboard's Align button)."""
import asyncio
import json

import numpy as np
import pytest

from ultraskelly.core.bot.sensory.face_locator_node import (
    CameraAligner,
    FaceLocatorNode,
    FaceLocatorParams,
    nominal_alignment,
    turn,
)
from ultraskelly.core.pubsub.bot_topics import DepthFrameMessage, FrameMessage, PoseDataMessage, TargetLocationMessage
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from tests.test_face_locator import HALF_VGA

DISTANCE = 1.6  # Metres from the ZED to the person
# A person seen from the front, in the ZED's frame at DISTANCE (x right, y down): COCO's 17 keypoints, with the
# shoulders, elbows and wrists a few centimetres inside the sides of their outline, as a person's are
KEYPOINTS = np.array([
    (0.0, -0.60), (-0.03, -0.63), (0.03, -0.63), (-0.07, -0.60), (0.07, -0.60),  # Nose, eyes, ears
    (-0.19, -0.40), (0.19, -0.40), (-0.20, -0.10), (0.20, -0.10), (-0.19, 0.15), (0.19, 0.15),  # Arms
    (-0.12, 0.20), (0.12, 0.20), (-0.10, 0.45), (0.10, 0.45), (-0.10, 0.70), (0.10, 0.70),  # Hips, knees, ankles
])


def person(top: float, bottom: float = 0.73, half_width: float = 0.22) -> np.ndarray:
    """uint16 millimetre ZED depth: a wall 3 m away, and the person's outline at DISTANCE before it."""
    columns, rows = np.meshgrid(np.arange(HALF_VGA.width), np.arange(HALF_VGA.height))
    x = DISTANCE * (columns - HALF_VGA.cx) / HALF_VGA.focal_length
    y = DISTANCE * (rows - HALF_VGA.cy) / HALF_VGA.focal_length
    on = (np.abs(x) <= half_width) & (y >= top) & (y <= bottom)
    return np.where(on, DISTANCE * 1000, 3000).astype(np.uint16)


def seen_by_oak(yaw: float, pitch: float) -> np.ndarray:
    """The keypoints' OAK pixels (and scores) when the OAK is truly turned (yaw, pitch) from the nominal alignment."""
    base = nominal_alignment()
    rotation = turn(yaw, pitch) @ base.rotation
    points = np.column_stack([KEYPOINTS, np.full(len(KEYPOINTS), DISTANCE)])
    in_oak = (points - base.translation) @ rotation  # rotation.T applied to each row
    pixels = (base.oak_camera_matrix @ (in_oak / in_oak[:, 2:3]).T).T[:, :2]
    return np.column_stack([pixels, np.full(len(pixels), 0.9)])


def align(depth: np.ndarray, keypoints: np.ndarray, frames: int = 8) -> tuple[CameraAligner, dict]:
    aligner = CameraAligner(nominal_alignment(), path=None)
    totals = sum(aligner.score(keypoints, depth, HALF_VGA) for _ in range(frames))
    return aligner, aligner.adopt(totals, frames)


def test_it_finds_how_far_the_oak_is_turned_left_or_right():
    aligner, result = align(person(top=-0.85), seen_by_oak(2.0, -1.0))
    assert result["changed"] and result["after"] > result["before"]
    assert aligner.yaw == pytest.approx(2.0, abs=0.4)
    assert aligner.pitch == 0.0 and "pitch left as it was" in result["message"]  # The head's top is far above the eyes


def test_it_finds_the_pitch_too_where_the_outline_bounds_it():
    aligner, result = align(person(top=-0.66), seen_by_oak(2.0, -1.0))  # Keypoints near the outline's top and bottom
    assert aligner.yaw == pytest.approx(2.0, abs=0.4) and aligner.pitch == pytest.approx(-1.0, abs=0.4)


def test_an_aligned_oak_is_left_alone():
    aligner, result = align(person(top=-0.66), seen_by_oak(0.0, 0.0))
    assert not result["changed"] and (aligner.yaw, aligner.pitch) == (0.0, 0.0) and "Already aligned" in result["message"]


def test_too_few_sure_keypoints_cannot_tell():
    keypoints = seen_by_oak(2.0, 0.0)
    keypoints[:10, 2] = 0.1
    assert CameraAligner(nominal_alignment(), path=None).score(keypoints, person(top=-0.85), HALF_VGA) is None


def test_the_turn_is_kept_and_read_back(tmp_path):
    path = tmp_path / "oak-zed-learned.json"
    aligner = CameraAligner(nominal_alignment(), path=path)
    totals = sum(aligner.score(seen_by_oak(2.0, 0.0), person(top=-0.85), HALF_VGA) for _ in range(5))
    aligner.adopt(totals, 5)
    assert json.loads(path.read_text(encoding="utf-8"))["yaw_deg"] == pytest.approx(aligner.yaw, abs=1e-4)
    again = CameraAligner(nominal_alignment(), path=path)
    assert again.yaw == pytest.approx(aligner.yaw, abs=1e-4)
    assert np.allclose(again.alignment().rotation, turn(aligner.yaw, 0.0), atol=1e-5)


def test_the_node_aligns_to_the_person_it_follows_and_places_them_with_it(tmp_path):
    async def scenario():
        bus = PubSubTopicManager.create()
        node = FaceLocatorNode.create(pubsub=bus, params=FaceLocatorParams(align_seconds=0.4, learned_file=tmp_path / "t.json"))
        node.latest_depth = DepthFrameMessage(depth=person(top=-0.85), color=np.zeros((188, 336, 3), np.uint8),
                                              rectification=HALF_VGA, frame_number=5, timestamp=10.0)
        keypoints = seen_by_oak(2.0, 0.0)[None]

        async def feed():  # A pose result every 30 ms, as the detector gives them
            for n in range(20):
                frame = FrameMessage(frame=np.zeros((480, 640, 3), np.uint8), frame_number=n, timestamp=10.0 + 0.001 * n)
                node.latest_pose = PoseDataMessage(frame=frame, keypoints=keypoints, target_person=0,
                                                   target=TargetLocationMessage(x=320, y=240))
                await asyncio.sleep(0.03)

        result, _ = await asyncio.gather(node.align(), feed())
        busy = await node.align() if node.aligning else None
        bus.close()
        return node, result, busy

    node, result, busy = asyncio.run(scenario())
    assert result["ok"] and result["changed"] and node.aligner.yaw == pytest.approx(2.0, abs=0.4)
    assert "turned +2" in node.alignment_source or "turned +1" in node.alignment_source
    assert busy is None


def test_aligning_with_nobody_in_view_says_so():
    async def scenario():
        bus = PubSubTopicManager.create()
        node = FaceLocatorNode.create(pubsub=bus, params=FaceLocatorParams(align_seconds=0.2, learned_file=None))
        result = await node.align()
        bus.close()
        return result
    result = asyncio.run(scenario())
    assert not result["ok"] and "Not aligned" in result["message"]


def test_the_align_route_runs_the_face_locators_align_and_says_how_it_went():
    from aiohttp.test_utils import TestClient, TestServer

    from tests.test_motor_test_routes import make_server

    class Aligner:
        def __init__(self, answer):
            self.answer, self.calls = answer, 0

        async def align(self):
            self.calls += 1
            return self.answer

    async def scenario():
        server, recorder, bus = make_server()
        replies = []
        for aligner in (None, Aligner({"ok": True, "changed": True, "yaw": 1.5, "message": "Aligned"}),
                        Aligner({"ok": False, "busy": True, "message": "Already aligning"})):
            server.aligner = aligner
            async with TestClient(TestServer(server.build_app())) as client:
                reply = await client.post("/api/align", json={})
                replies.append((reply.status, await reply.json()))
                form = await client.post("/api/align", data="now")  # A plain form, as any page could send
                replies.append((form.status, None))
        bus.close()
        return replies

    (none, none_form, done, done_form, busy, busy_form) = asyncio.run(scenario())
    assert none[0] == 503 and "no depth" in none[1]["message"]
    assert done == (200, {"ok": True, "changed": True, "yaw": 1.5, "message": "Aligned"})
    assert busy[0] == 409
    assert none_form[0] == done_form[0] == busy_form[0] == 415
