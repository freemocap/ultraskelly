"""The OAK node turns what the OAK sends into frames and faces; run here with stand-ins for depthai."""
import asyncio
import time
from datetime import timedelta
from types import SimpleNamespace

import numpy as np
import pytest

from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams, OakFaceParams
from ultraskelly.core.bot.sensory.oak_faces import FaceAssembler
from ultraskelly.core.bot.sensory.yunet import yunet_priors
from ultraskelly.core.pubsub.bot_topics import FacesMessage, FacesTopic, FrameMessage, FrameTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


class Stamped:
    def __init__(self, sequence, seconds=2.5):
        self.sequence, self.seconds = sequence, seconds

    def getSequenceNum(self):
        return self.sequence

    def getTimestamp(self):
        return timedelta(seconds=self.seconds)


class Image(Stamped):
    def __init__(self, sequence):
        super().__init__(sequence)
        self.pixels = np.zeros((480, 640, 3), np.uint8)

    def getFrame(self):
        return self.pixels


class Encoded(Stamped):
    def getData(self):
        return np.frombuffer(b"\xff\xd8 OAK JPEG", np.uint8)


class Tensors(Stamped):
    def __init__(self, sequence, **tensors):
        super().__init__(sequence)
        self.tensors = tensors

    def getTensor(self, name):
        return self.tensors[name]

    def getFirstTensor(self):
        return next(iter(self.tensors.values()))


class Group(dict):
    """depthai's MessageGroup: Sync's pair of a frame and its JPEG."""


class Detection(SimpleNamespace):
    pass


class Detections:
    def setSequenceNum(self, sequence):
        self.sequence = sequence


dai = SimpleNamespace(MessageGroup=Group, ImgDetection=Detection, ImgDetections=Detections)


class Loop:
    """The event loop's thread-safe call, made at once."""

    def call_soon_threadsafe(self, function, *args):
        function(*args)


def node_with_faces():
    node = OakCameraNode(pubsub=PubSubTopicManager.create(), params=OakCameraNodeParams(faces=OakFaceParams()))
    node.outbox = asyncio.Queue(maxsize=16)
    node.faces = FaceAssembler((640, 480), max_faces=3)
    node.crop_requests = SimpleNamespace(sent=[])
    node.crop_requests.send = node.crop_requests.sent.append
    return node


def published(node):
    items = []
    while not node.outbox.empty():
        items.append(node.outbox.get_nowait())
    return items


def yunet_outputs(sequence, *, face=True):
    priors = yunet_priors(320, 240)
    loc, iou = np.zeros((len(priors), 14), np.float32), np.zeros((len(priors), 1), np.float32)
    conf = np.tile(np.array([1.0, 0.0], np.float32), (len(priors), 1))
    if face:
        row = int(np.argmin(np.abs(priors[:, 0] - 0.5) + np.abs(priors[:, 1] - 0.4) + np.abs(priors[:, 2] - 48 / 320)))
        conf[row], iou[row] = [0.1, 0.9], [0.9]
    return Tensors(sequence, loc=loc, conf=conf, iou=iou)


def test_each_frame_arrives_with_the_oaks_jpeg_number_and_capture_time():
    node = node_with_faces()
    image = Image(12)

    node._on_frames(dai, Group(rgb=image, jpeg=Encoded(12)), 0.0, Loop())

    [(topic, message)] = published(node)
    assert topic is FrameTopic and isinstance(message, FrameMessage)
    assert message.frame is image.pixels and message.jpeg == b"\xff\xd8 OAK JPEG"
    assert (message.frame_number, message.timestamp) == (12, 2.5)


def test_a_face_asks_the_oak_for_its_crops_and_is_published_once_measured():
    node = node_with_faces()

    node._on_face_detection(dai, yunet_outputs(7), 0.0, Loop())

    [request] = node.crop_requests.sent
    assert request.sequence == 7
    assert [(detection.label, detection.confidence) for detection in request.detections] == [(0, 0.0), (1, 0.0), (2, 0.0), (3, 0.0)]
    assert published(node) == []  # Waiting for head pose, gaze and landmarks
    node._on_face_results(dai, Tensors(7, faces=np.array([[0, 12, -3, 1, 0, 0, -1]], np.float32)), 0.02, Loop())
    node._on_face_landmarks(dai, Tensors(7, mesh=np.zeros((1, 1, 1, 1404), np.float32)), 0.03, Loop())

    [(topic, message)] = published(node)
    assert topic is FacesTopic and isinstance(message, FacesMessage)
    assert (message.frame_number, message.timestamp, len(message.faces)) == (7, 2.5, 1)
    assert message.faces[0].head_pose == pytest.approx((12, -3, 1)) and message.faces[0].mesh.shape == (468, 3)


def test_a_frame_without_faces_is_published_at_once_and_asks_for_nothing():
    node = node_with_faces()

    node._on_face_detection(dai, yunet_outputs(8, face=False), 0.0, Loop())

    assert node.crop_requests.sent == []
    [(topic, message)] = published(node)
    assert topic is FacesTopic and message.faces == [] and message.frame_number == 8


def test_a_busy_event_loop_drops_the_oldest_waiting_message():
    node = node_with_faces()
    node.outbox = asyncio.Queue(maxsize=2)
    for number in range(3):
        node._deliver(FrameTopic, number)

    assert [message for _, message in published(node)] == [1, 2] and node.dropped == 1


def test_status_reports_output_rates_and_the_chip():
    node = node_with_faces()
    now = time.monotonic()
    for arrival in range(60):
        node._count("face_detection", now - 3.0 + arrival / 20)  # 20 per second, the last 2 s of them recent
    node._on_system(dai, SimpleNamespace(
        leonCssCpuUsage=SimpleNamespace(average=0.25), leonMssCpuUsage=SimpleNamespace(average=0.5),
        ddrMemoryUsage=SimpleNamespace(used=30 * 2**20, total=333 * 2**20), chipTemperature=SimpleNamespace(average=48.0),
    ), 0.0, Loop())

    status = node.status()
    assert status["rates"]["face_detection"] == pytest.approx(20, abs=1)
    assert status["chip"] == {"leon_os_cpu": 0.25, "leon_rt_cpu": 0.5, "ddr_used_mib": 30.0, "ddr_total_mib": 333.0,
                              "temperature_c": 48.0}
