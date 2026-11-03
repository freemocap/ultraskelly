import asyncio
import time

import numpy as np
import pytest

from ultraskelly.core.bot.stall_monitor_node import StallMonitorNode, StallMonitorParams, StreamStalledError
from ultraskelly.core.pubsub.bot_topics import FrameMessage, FrameTopic, PoseDataMessage, PoseDataTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

FAST = StallMonitorParams(
    frame_timeout_seconds=0.1, pose_timeout_seconds=0.1, startup_seconds=0.2, check_interval_seconds=0.01,
)


async def publish(pubsub, seconds, *, frames=True, poses=True, pose_frames=True):
    """Publish at about 100 Hz; a pose result without its frame is the detector's "camera silent" report."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        frame = FrameMessage(frame=np.zeros((2, 2, 3), dtype=np.uint8))
        if frames:
            await pubsub.topics[FrameTopic].publish(frame)
        if poses:
            await pubsub.topics[PoseDataTopic].publish(PoseDataMessage(frame=frame if pose_frames else None))
        await asyncio.sleep(0.01)


def run_monitor(scenario, *, watch_poses=True):
    async def main():
        pubsub = PubSubTopicManager.create()
        monitor = StallMonitorNode.create(pubsub=pubsub, params=FAST, watch_poses=watch_poses)
        task = asyncio.create_task(monitor.run())
        try:
            await scenario(pubsub)
            if not task.done():
                monitor.stop()
            return await asyncio.wait_for(task, timeout=1)
        finally:
            pubsub.close()

    return asyncio.run(main())


def test_flowing_streams_keep_the_bot_running():
    run_monitor(lambda pubsub: publish(pubsub, 0.5))  # Returns once stopped, without raising


def test_a_camera_that_goes_quiet_stops_the_bot():
    async def scenario(pubsub):
        await publish(pubsub, 0.2)
        await asyncio.sleep(0.3)

    with pytest.raises(StreamStalledError, match="No camera frames for"):
        run_monitor(scenario)


def test_a_detector_that_stops_answering_stops_the_bot():
    async def scenario(pubsub):
        await publish(pubsub, 0.2)
        await publish(pubsub, 0.3, pose_frames=False)  # Frames keep coming; the detector only says "no camera"

    with pytest.raises(StreamStalledError, match="No pose results for"):
        run_monitor(scenario)


def test_streams_that_never_start_stop_the_bot_after_the_startup_allowance():
    with pytest.raises(StreamStalledError, match="No camera frames within 0.2 s of starting"):
        run_monitor(lambda pubsub: asyncio.sleep(0.4))


def test_pose_results_are_ignored_unless_watched():
    run_monitor(lambda pubsub: publish(pubsub, 0.5, poses=False), watch_poses=False)
