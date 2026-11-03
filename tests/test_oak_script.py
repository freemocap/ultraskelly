"""The OAK's face-chain Script, run on the Pi against stand-ins for depthai's on-device objects."""
import math
from types import SimpleNamespace

import pytest

from ultraskelly.core.bot.sensory.oak_faces import CropKind
from ultraskelly.core.bot.sensory.oak_script import RESULT_FIELDS, RESULT_LAYER, SCRIPT


class Finished(Exception):
    """No more crop requests: ends the Script's loop."""


class Message:
    def __init__(self, sequence: int):
        self.sequence, self.timestamp = sequence, None

    def getSequenceNum(self):
        return self.sequence

    def setSequenceNum(self, sequence):
        self.sequence = sequence

    def setTimestamp(self, timestamp):
        self.timestamp = timestamp

    def getTimestamp(self):
        return ("time", self.sequence)


class Frame(Message):
    def getWidth(self):
        return 640

    def getHeight(self):
        return 480


class NNData(Message):
    def __init__(self, size=0, **layers):
        super().__init__(0)
        self.layers = {name: list(values) for name, values in layers.items()}

    def setLayer(self, name, values):
        self.layers[name] = list(values)

    def getLayerFp16(self, name):
        return self.layers[name]


class ImageManipConfig:
    ResizeMode = SimpleNamespace(STRETCH="stretch")

    def __init__(self):
        self.crop = self.size = self.type = None

    def addCrop(self, x, y, w, h):
        self.crop = (x, y, w, h)

    def setOutputSize(self, w, h, mode):
        self.size = (w, h, mode)

    def setFrameType(self, frame_type):
        self.type = frame_type


class Queue:
    def __init__(self, messages=(), on_empty=None):
        self.messages, self.on_empty = list(messages), on_empty

    def get(self):
        if not self.messages:
            raise self.on_empty()
        return self.messages.pop(0)

    def tryGet(self):
        return self.messages.pop(0) if self.messages else None


class Output:
    def __init__(self, name, network):
        self.name, self.network, self.sent = name, network, []

    def send(self, message):
        self.sent.append(message)
        self.network(self.name, message)


def crop(kind, face, box):
    return SimpleNamespace(label=int(kind), confidence=float(face), xmin=box[0], ymin=box[1], xmax=box[2], ymax=box[3])


def request(sequence, *crops):
    message = Message(sequence)
    message.detections = list(crops)
    return message


def run(requests, frames, *, answer_head_pose=lambda count: True):
    """Run the Script until its requests run out. Fake networks answer as the Script feeds them:
    head pose for each head pose crop (unless `answer_head_pose(count)` says no), gaze for each angles message."""
    inputs = {
        "crops": Queue(requests, on_empty=Finished),
        "frame": Queue(frames, on_empty=Finished),
        "head_pose": Queue(),
        "gaze": Queue(),
    }
    counts = {"head_pose": 0, "gaze": 0}

    def network(name, message):
        if name == "head_pose_image":
            counts["head_pose"] += 1
            if answer_head_pose(counts["head_pose"]):
                yaw = 10.0 * counts["head_pose"]
                answer = NNData(**{"tf.identity": [yaw], "tf.identity_1": [-5.0], "tf.identity_2": [2.0]})
                answer.setSequenceNum(message.getSequenceNum())
                inputs["head_pose"].messages.append(answer)
        if name == "angles":
            counts["gaze"] += 1
            answer = NNData(Identity0=[0.0, 0.0, -float(counts["gaze"])])
            answer.setSequenceNum(message.getSequenceNum())
            inputs["gaze"].messages.append(answer)

    outputs = {}

    class Outputs(dict):
        def __missing__(self, name):
            self[name] = outputs[name] = Output(name, network)
            return self[name]

    node = SimpleNamespace(inputs=inputs, outputs=Outputs(), warn=print)
    with pytest.raises(Finished):
        exec(compile(SCRIPT, "oak_script", "exec"), {
            "node": node, "NNData": NNData, "ImageManipConfig": ImageManipConfig,
            "ImgFrame": SimpleNamespace(Type=SimpleNamespace(BGR888p="bgr888p")),
        })
    return outputs


def face_rows(result):
    values = result.layers[RESULT_LAYER]
    return [values[index:index + RESULT_FIELDS] for index in range(0, len(values), RESULT_FIELDS)]


def test_each_crop_is_cut_from_its_own_frame_and_every_face_gets_head_pose_and_gaze():
    face = (0.4, 0.2, 0.6, 0.5)
    outputs = run(
        [request(6, crop(CropKind.HEAD_POSE, 0, face), crop(CropKind.LEFT_EYE, 0, (0.5, 0.3, 0.55, 0.35)),
                 crop(CropKind.RIGHT_EYE, 0, (0.45, 0.3, 0.5, 0.35)), crop(CropKind.HEAD_POSE, 1, (0.1, 0.1, 0.2, 0.2)),
                 crop(CropKind.LEFT_EYE, 1, (0.1, 0.1, 0.12, 0.12)), crop(CropKind.RIGHT_EYE, 1, (0.15, 0.1, 0.17, 0.12)),
                 crop(CropKind.MESH, 0, (0.3, 0.1, 0.7, 0.6)))],
        [Frame(5), Frame(6), Frame(7)],
    )

    head_pose_config = outputs["head_pose_config"].sent[0]
    assert head_pose_config.crop == (256, 96, 128, 144) and head_pose_config.size == (60, 60, "stretch")
    assert [frame.getSequenceNum() for frame in outputs["head_pose_image"].sent] == [6, 6]  # The requested frame
    assert outputs["mesh_config"].sent[0].size[:2] == (192, 192)
    assert len(outputs["left_eye_image"].sent) == len(outputs["right_eye_image"].sent) == 2
    [result] = outputs["results"].sent
    assert result.getSequenceNum() == 6 and result.timestamp == ("time", 6)
    assert face_rows(result) == [[0, 10.0, -5.0, 2.0, 0.0, 0.0, -1.0], [1, 20.0, -5.0, 2.0, 0.0, 0.0, -2.0]]


def test_a_lost_head_pose_leaves_its_frame_unmeasured_and_the_next_frame_in_step():
    two_faces = (crop(CropKind.HEAD_POSE, 0, (0.1, 0.1, 0.2, 0.2)), crop(CropKind.HEAD_POSE, 1, (0.5, 0.1, 0.6, 0.2)))
    outputs = run(
        [request(3, *two_faces), request(4, crop(CropKind.HEAD_POSE, 0, (0.1, 0.1, 0.2, 0.2)))],
        [Frame(3), Frame(4)],
        answer_head_pose=lambda count: count != 1,  # Frame 3's first head pose never comes back
    )

    assert len(outputs["angles"].sent) == 3  # One per face, in step with their eye crops
    lost, found = outputs["results"].sent
    assert [row[0] for row in face_rows(lost)] == [0, 1]
    assert all(math.isnan(value) for row in face_rows(lost) for value in row[1:])  # Neither answer can be placed
    assert face_rows(found) == [[0, 30.0, -5.0, 2.0, 0.0, 0.0, -3.0]]  # Frame 4's own answers


def test_an_answer_for_a_later_frame_waits_for_it_instead_of_being_lost():
    one_face = (crop(CropKind.HEAD_POSE, 0, (0.1, 0.1, 0.2, 0.2)),)
    outputs = run([request(3, *one_face), request(4, *one_face)], [Frame(3), Frame(4)],
                  answer_head_pose=lambda count: count != 1)  # Frame 3's head pose never comes back

    lost, found = outputs["results"].sent
    assert found.getSequenceNum() == 4 and face_rows(found)[0][1] == 20.0  # Frame 4's own head pose


def test_a_landmark_crop_alone_sends_no_results():
    outputs = run([request(5, crop(CropKind.MESH, 0, (0.3, 0.1, 0.7, 0.6)))], [Frame(5)])

    assert len(outputs["mesh_image"].sent) == 1 and "results" not in outputs


def test_a_request_for_a_frame_that_already_left_is_skipped():
    outputs = run(
        [request(2, crop(CropKind.HEAD_POSE, 0, (0.1, 0.1, 0.2, 0.2))),
         request(4, crop(CropKind.HEAD_POSE, 0, (0.1, 0.1, 0.2, 0.2)))],
        [Frame(3), Frame(4)],
    )

    assert [result.getSequenceNum() for result in outputs["results"].sent] == [4]
