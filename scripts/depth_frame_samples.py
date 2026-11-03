"""Write sample `depth_frame` messages (docs/depth-stream.md) for building and checking viewers.

    .venv/bin/python -m scripts.depth_frame_samples --synthetic synthetic.cbor
    .venv/bin/python -m scripts.depth_frame_samples --live live.cbor      # On the Pi, with the ZED

The synthetic scene has known geometry, in the camera frame (x right, y down, z forward,
metres): a wall 3 m ahead, a floor 0.4 m below the camera, and a ball of radius 0.12 m
centred at (0.2, 0, 1.0). The live sample is one depth frame from the ZED with OpenCV's
matcher. Each file holds one encoded message, exactly as /stream sends it.
"""
import argparse
import asyncio
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification
from ultraskelly.core.pubsub.bot_topics import DepthFrameMessage
from ultraskelly.ui.depth_stream import encode_depth_frame, nominal_camera_to_head
from ultraskelly.ui.runtime_server import encode_jpeg_with_opencv

# The ZED's rectified VGA camera.
VGA = StereoRectification(width=672, height=376, focal_length=339.8, cx=331.1, cy=193.4, baseline=0.063,
                          left_rotation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
WALL, FLOOR, BALL_CENTRE, BALL_RADIUS = 3.0, 0.4, np.array([0.2, 0.0, 1.0]), 0.12


def synthetic_frame() -> DepthFrameMessage:
    """Depth along z and a colour image of the synthetic scene."""
    columns, rows = np.meshgrid(np.arange(VGA.width), np.arange(VGA.height))
    rays = np.stack([(columns - VGA.cx) / VGA.focal_length, (rows - VGA.cy) / VGA.focal_length,
                     np.ones(columns.shape)], axis=-1)  # Directions with z = 1, so the distance along a ray is its depth
    depth = np.full(columns.shape, WALL)
    color = np.zeros(columns.shape + (3,), np.uint8)
    color[:] = (120, 120, 130)
    with np.errstate(divide="ignore", invalid="ignore"):
        floor = np.where(rays[..., 1] > 0, FLOOR / rays[..., 1], np.inf)
        checks = ((np.floor(rays[..., 0] * floor * 4) + np.floor(floor * 4)) % 2) == 1  # 25 cm floor tiles
    on_floor = floor < depth
    depth[on_floor] = floor[on_floor]
    color[on_floor & checks], color[on_floor & ~checks] = (200, 190, 160), (90, 80, 60)
    # The ball: the nearer root of |t * ray - centre| = radius.
    a = (rays ** 2).sum(-1)
    b = -2 * (rays @ BALL_CENTRE)
    c = BALL_CENTRE @ BALL_CENTRE - BALL_RADIUS ** 2
    discriminant = b ** 2 - 4 * a * c
    hit = discriminant >= 0
    t = np.where(hit, (-b - np.sqrt(np.maximum(discriminant, 0))) / (2 * a), np.inf)
    on_ball = hit & (t < depth)
    depth[on_ball] = t[on_ball]
    color[on_ball] = (240, 120, 40)
    return DepthFrameMessage(depth=np.rint(depth * 1000).astype(np.uint16), color=color, rectification=VGA,
                             frame_number=1, timestamp=0.0, method="synthetic")


async def live_frame() -> DepthFrameMessage:
    from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNode, StereoDepthNodeParams
    from ultraskelly.core.bot.sensory.zed_camera_node import ZedCameraNode, ZedCameraNodeParams
    from ultraskelly.core.pubsub.bot_topics import DepthFrameTopic
    from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

    bus = PubSubTopicManager.create()
    zed = ZedCameraNode.create(pubsub=bus, params=ZedCameraNodeParams())
    depth = StereoDepthNode.create(pubsub=bus, params=StereoDepthNodeParams(), eye_size=(672, 376))
    frames = bus.get_subscription(DepthFrameTopic)
    tasks = [asyncio.create_task(node.run()) for node in (zed, depth)]
    try:
        for _ in range(10):  # Let exposure settle
            frame = await asyncio.wait_for(frames.get(), timeout=10)
    finally:
        for node in (zed, depth):
            node.stop()
        await asyncio.gather(*tasks)
        bus.close()
    return frame


def write(frame: DepthFrameMessage, path: Path) -> None:
    encoded = encode_depth_frame(frame, color_jpeg=encode_jpeg_with_opencv(frame.color, 80), sequence=1,
                                 camera_to_head=nominal_camera_to_head(frame.rectification))
    path.write_bytes(encoded)
    known = (frame.depth > 0).mean()
    print(f"Wrote {path}: {len(encoded) / 1024:.0f} KiB, {100 * known:.0f}% of pixels with depth")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--synthetic", type=Path, help="Write the synthetic scene here")
    parser.add_argument("--live", type=Path, help="Write a depth frame from the live ZED here")
    args = parser.parse_args()
    if args.synthetic:
        write(synthetic_frame(), args.synthetic)
    if args.live:
        write(asyncio.run(live_frame()), args.live)


if __name__ == "__main__":
    main()
