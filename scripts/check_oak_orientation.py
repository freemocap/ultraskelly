"""Check on the robot that `flip_camera` turns the OAK's image 180 degrees from its board's default.

Uses the OAK, so stop the bot and run it under the robot lock, with the camera looking at
something with detail rather than a blank wall:

    flock -w 60 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.check_oak_orientation --save ~/oak-orientation.jpg

Neither depthai's camera features nor a frame's transformation tell the sensor's readouts apart,
so this compares frames. It builds the OAK pipeline as the bot does, without models, once
without `flip_camera` (the board's default) and once with it, and keeps each one's last frame
once the exposure has settled. The second frame must show the first one's scene turned 180
degrees, and its hardware JPEG must show the frame; the two readouts may start a few pixels
apart, so shifts of up to 16 pixels are allowed. Prints a JSON summary and exits 0 when the
image is turned, 1 when it is not, and 2 when the view has too little detail to tell.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.oak_models import without_telemetry

SETTLE_SECONDS = 4.0  # Auto-exposure and white balance settle in this
REACH = 4  # Cells of the 160x120 grey image the two readouts may be apart: 16 pixels
TURNED = 0.7  # Least similarity of two views of one scene
MARGIN = 0.3  # Least lead of the matching orientation over the other
LEAST_DETAIL = 6.0  # Grey levels of spread below which a view cannot tell


def grey(image: np.ndarray) -> np.ndarray:
    """An RGB or grey image as grey at a quarter of its size."""
    height, width = image.shape[:2]
    return image.reshape(height // 4, 4, width // 4, 4, -1).mean(axis=(1, 3, 4))


def similarity(a: np.ndarray, b: np.ndarray) -> float:
    a, b = a - a.mean(), b - b.mean()
    scale = np.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / scale) if scale > 0 else 0.0


def best_similarity(a: np.ndarray, b: np.ndarray, reach: int = REACH) -> float:
    """The highest similarity of the middle of a with b's, shifted up to `reach` cells either way."""
    height, width = a.shape
    middle = a[reach:height - reach, reach:width - reach]
    return max(similarity(middle, b[reach + dy:height - reach + dy, reach + dx:width - reach + dx])
               for dy in range(-reach, reach + 1) for dx in range(-reach, reach + 1))


def verdict(default: np.ndarray, flipped: np.ndarray) -> dict:
    """Whether the grey view `flipped` shows the grey view `default` turned 180 degrees."""
    turned, unturned = best_similarity(flipped, default[::-1, ::-1]), best_similarity(flipped, default)
    detail = float(default.std())
    if detail >= LEAST_DETAIL and turned >= TURNED and turned - unturned >= MARGIN:
        result = "turned"
    elif detail >= LEAST_DETAIL and unturned >= TURNED and unturned - turned >= MARGIN:
        result = "not turned"
    else:
        result = "inconclusive"
    return {"result": result, "turned": round(turned, 3), "unturned": round(unturned, 3), "detail": round(detail, 1)}


def grab(flip_camera: bool) -> tuple[np.ndarray, bytes]:
    """The last RGB frame and its JPEG after SETTLE_SECONDS, from the OAK pipeline the bot builds."""
    from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams
    from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

    node = OakCameraNode.create(pubsub=PubSubTopicManager.create(), params=OakCameraNodeParams(flip_camera=flip_camera))
    try:
        node.pipeline.start()
        frames, end = node.queues["frames"], time.monotonic() + SETTLE_SECONDS
        group = frames.get()
        while time.monotonic() < end:
            group = frames.get()
        return group["rgb"].getFrame().copy(), bytes(group["jpeg"].getData())
    finally:
        node.pipeline.stop()
        node.pipeline.getDefaultDevice().close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Check that flip_camera turns the OAK's image 180 degrees.")
    parser.add_argument("--save", type=Path, default=None, help="Write the views side by side, labelled, to this JPEG")
    args = parser.parse_args()
    without_telemetry()
    import cv2

    default_rgb, _ = grab(flip_camera=False)
    flipped_rgb, flipped_jpeg = grab(flip_camera=True)
    jpeg_bgr = cv2.imdecode(np.frombuffer(flipped_jpeg, np.uint8), cv2.IMREAD_COLOR)
    frames = verdict(grey(default_rgb), grey(flipped_rgb))
    jpeg_is_its_frame = round(similarity(grey(jpeg_bgr), grey(flipped_rgb)), 3)
    print(json.dumps({"frames": frames, "jpeg_is_its_frame": jpeg_is_its_frame}))
    if args.save:
        views = [("board default (flip_camera off)", default_rgb), ("default turned on the Pi (old main)", default_rgb[::-1, ::-1]),
                 ("flip_camera: the bot's frame", flipped_rgb), ("flip_camera: its hardware JPEG", jpeg_bgr[:, :, ::-1])]
        tiles = []
        for label, rgb in views:
            tile = cv2.cvtColor(cv2.resize(np.ascontiguousarray(rgb), (320, 240)), cv2.COLOR_RGB2BGR)
            cv2.rectangle(tile, (0, 0), (320, 22), (0, 0, 0), -1)
            cv2.putText(tile, label, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            tiles.append(tile)
        cv2.imwrite(str(args.save), np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])]))
    if frames["result"] == "inconclusive":
        sys.exit(2)
    sys.exit(0 if frames["result"] == "turned" and jpeg_is_its_frame >= 0.9 else 1)


if __name__ == "__main__":
    main()
