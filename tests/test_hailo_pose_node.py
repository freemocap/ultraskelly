"""The Hailo pose node publishes each target together with the pose data it came from."""
import asyncio

import numpy as np
import pytest

pytest.importorskip("cv2", reason="the Hailo pose node imports OpenCV")

from ultraskelly.core.bot.sensory.hailo_pose_node import HailoPoseDetectorNode, HailoPoseDetectorParams  # noqa: E402
from ultraskelly.core.bot.sensory.pose_common import CocoKeypoint  # noqa: E402
from ultraskelly.core.bot.sensory.yolov8_pose import PoseDetections  # noqa: E402
from ultraskelly.core.pubsub.bot_topics import (  # noqa: E402
    FrameMessage,
    PoseDataTopic,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager  # noqa: E402


def people(*noses: tuple[float, float, float]) -> np.ndarray:
    keypoints = np.zeros((len(noses), 17, 3), np.float32)
    for row, nose in enumerate(noses):
        keypoints[row, CocoKeypoint.NOSE] = nose
    return keypoints


def publish(detections: PoseDetections | None, frame: FrameMessage):
    """Run the node's publish step without a Hailo-8 and return what reached each topic."""
    async def scenario():
        bus = PubSubTopicManager.create()
        targets, poses = bus.get_subscription(TargetLocationTopic), bus.get_subscription(PoseDataTopic)
        node = HailoPoseDetectorNode(pubsub=bus, params=HailoPoseDetectorParams())
        await node._publish(detections, frame)
        published = targets.get_nowait(), poses.get_nowait()
        bus.close()
        return published

    return asyncio.run(scenario())


def test_pose_data_names_the_followed_person_and_carries_the_published_target():
    frame = FrameMessage(frame=np.zeros((480, 640, 3), np.uint8), frame_number=7, timestamp=1.25)
    keypoints = people((300, 200, 0.9), (10, 20, 0.9))
    detections = PoseDetections(keypoints=keypoints, scores=np.array([0.5, 0.9]), boxes=np.zeros((2, 4)))

    target, pose = publish(detections, frame)

    assert pose.target_person == 1  # The highest-scoring person, whatever their row
    assert pose.target == target
    assert (target.x, target.y) == (10, 20)
    assert pose.frame is frame and pose.keypoints is keypoints


def test_pose_data_without_people_follows_nobody():
    frame = FrameMessage(frame=np.zeros((480, 640, 3), np.uint8), frame_number=8)

    target, pose = publish(None, frame)

    assert pose.target_person is None
    assert pose.target == target and (target.x, target.y, target.angle) == (None, None, None)
