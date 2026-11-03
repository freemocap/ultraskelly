"""Measure where the OAK sits relative to the ZED, for the face locator. Redo it after re-seating either camera.

Uses the OAK, so run it under the robot lock (motors are not touched):

    flock -w 1200 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.oak_zed_alignment --check ~/oak-zed

Point the head at a scene with plenty of detail 0.5 to 3 m away, seen by both cameras, and
keep it still for the few seconds this takes. It captures OAK frames and ZED depth frames
taken within 20 ms of each other, matches features between the OAK image and the ZED's
rectified left image, and fits the OAK's pose to the ZED depth under those features
(ultraskelly/core/bot/sensory/oak_zed_alignment.py). It writes
ultraskelly/calibration/oak-zed-alignment.json, which the face locator then uses.
`--check` saves the OAK image with the ZED's depth drawn onto it through the new
alignment: the colours should follow the scene's shapes.
"""
import argparse
import asyncio
import time
from collections import deque
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams
from ultraskelly.core.bot.sensory.oak_zed_alignment import ALIGNMENT_FILE, estimate_alignment, points_from_depth
from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNode, StereoDepthNodeParams
from ultraskelly.core.bot.sensory.zed_camera_node import ZED_MODES, ZedCameraNode, ZedCameraNodeParams
from ultraskelly.core.pubsub.bot_topics import DepthFrameTopic, FrameTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

SAME_MOMENT = 0.02  # Seconds between an OAK frame and a depth frame taken together
SETTLE_SECONDS = 2.0
GIVE_UP_SECONDS = 60.0


async def keep_recent(frames, recent: deque) -> None:
    """Every frame from a subscription, newest last: its queue holds only the latest one."""
    while True:
        recent.append(await frames.get())


async def capture_views(count: int) -> list:
    """(OAK frame, depth frame) pairs taken at about the same moment."""
    bus = PubSubTopicManager.create()
    zed = ZedCameraNodeParams()
    width, height = ZED_MODES[zed.resolution][0]
    nodes = [
        OakCameraNode.create(pubsub=bus, params=OakCameraNodeParams(flip_camera=True)),
        ZedCameraNode.create(pubsub=bus, params=zed),
        StereoDepthNode.create(pubsub=bus, params=StereoDepthNodeParams(), eye_size=(width // 2, height)),
    ]
    recent, views = deque(maxlen=30), []  # A second of OAK frames
    depth_frames = bus.get_subscription(DepthFrameTopic)
    tasks = [asyncio.create_task(node.run()) for node in nodes]
    collector = asyncio.create_task(keep_recent(bus.get_subscription(FrameTopic), recent))
    try:
        await asyncio.sleep(SETTLE_SECONDS)
        deadline = time.monotonic() + GIVE_UP_SECONDS
        while len(views) < count:
            if time.monotonic() > deadline:
                raise TimeoutError(f"Only {len(views)} of {count} depth frames had an OAK frame within "
                                   f"{SAME_MOMENT * 1000:.0f} ms in {GIVE_UP_SECONDS:.0f} s")
            depth = await asyncio.wait_for(depth_frames.get(), timeout=10)
            await asyncio.sleep(0.1)  # Let the OAK frames taken just after it arrive too
            timed = [frame for frame in recent if frame.timestamp is not None]
            if not timed or depth.timestamp is None:
                continue
            oak = min(timed, key=lambda frame: abs(frame.timestamp - depth.timestamp))
            if abs(oak.timestamp - depth.timestamp) <= SAME_MOMENT:
                views.append((oak, depth))
                await asyncio.sleep(0.3)  # Spread the views over a little time
    finally:
        collector.cancel()
        for node in nodes:
            node.stop()
        await asyncio.gather(collector, *tasks, return_exceptions=True)
        bus.close()
    return views


def check_image(oak_image: np.ndarray, depth, alignment) -> np.ndarray:
    """The OAK image with ZED depth points drawn where the alignment puts them (BGR)."""
    import cv2

    from ultraskelly.ui.runtime_server import depth_to_bgr

    rows, columns = np.nonzero(depth.depth[::2, ::2])
    rows, columns = rows * 2, columns * 2
    points, _ = points_from_depth(np.stack([columns, rows], 1).astype(float), depth.depth, depth.rectification)
    in_oak = (points - alignment.translation) @ alignment.rotation  # rotation.T @ (point - translation)
    pixels = (in_oak @ alignment.oak_camera_matrix.T)[:, :2] / in_oak[:, 2:]
    colours = depth_to_bgr(depth.depth[rows, columns].reshape(1, -1)).reshape(-1, 3)
    image = cv2.cvtColor(oak_image, cv2.COLOR_RGB2BGR) // 2
    for (x, y), colour in zip(pixels.astype(int), colours):
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            cv2.circle(image, (int(x), int(y)), 1, tuple(int(value) for value in colour), -1)
    return image


def main() -> None:
    import cv2

    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--views", type=int, default=8, help="Moments to use (default 8)")
    parser.add_argument("--out", type=Path, default=ALIGNMENT_FILE)
    parser.add_argument("--check", type=Path, help="Folder for the check image")
    args = parser.parse_args()

    views = asyncio.run(capture_views(args.views))
    rectification = views[0][1].rectification
    alignment = estimate_alignment([(oak.frame, depth.color, depth.depth) for oak, depth in views], rectification)
    alignment.save(args.out)
    angles = np.degrees(cv2.Rodrigues(alignment.rotation)[0].ravel())
    print(f"Wrote {args.out}: {alignment.points} points agree, median error {alignment.error_px:.2f} px; "
          f"OAK at {np.round(alignment.translation * 1000).astype(int).tolist()} mm in the ZED frame, "
          f"turned {np.round(angles, 2).tolist()} degrees")
    if args.check:
        args.check.mkdir(parents=True, exist_ok=True)
        path = args.check / "oak-zed-alignment-check.jpg"
        cv2.imwrite(str(path), check_image(views[-1][0].frame, views[-1][1], alignment))
        print(f"Check image: {path}")


if __name__ == "__main__":
    main()
