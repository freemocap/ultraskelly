"""Check the ZED's rectification: do features matched between its eyes sit on the same rows?

Captures a few pairs, matches SIFT features between the eyes, and reports the vertical
offset dy (right row minus left row) before and after rectifying with the factory
calibration, plus the disparity of the matches. Run on the Pi; the ZED needs no robot lock:

    .venv/bin/python -m scripts.zed_row_alignment --resolution VGA --save ~/zed-check

`--save` writes the last rectified pair with a line every 24 rows.
"""
import argparse
import asyncio
from pathlib import Path
from typing import NamedTuple

import numpy as np

from ultraskelly.core.bot.sensory.stereo_rectification import zed_rectifier
from ultraskelly.core.bot.sensory.zed_camera_node import (
    ZedCameraNode,
    ZedCameraNodeParams,
    ZedResolution,
    yuyv_to_rgb_with_opencv,
)
from ultraskelly.core.pubsub.bot_topics import StereoFrameTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

SETTLE_FRAMES = 30  # Let exposure and white balance settle first
FRAME_SPACING = 10  # Use every tenth frame after that
OUTLIER_PIXELS = 8  # Matches this far from the median row offset are mismatches


class ColourPair(NamedTuple):
    left: np.ndarray  # RGB
    right: np.ndarray
    frame_number: int | None


async def capture_pairs(resolution: ZedResolution, count: int) -> list[ColourPair]:
    """`count` live pairs, both eyes converted to RGB."""
    bus = PubSubTopicManager.create()
    node = ZedCameraNode.create(pubsub=bus, params=ZedCameraNodeParams(resolution=resolution))
    pairs = bus.get_subscription(StereoFrameTopic)
    running = asyncio.create_task(node.run())
    kept = []
    try:
        for index in range(SETTLE_FRAMES + count * FRAME_SPACING):
            pair = await asyncio.wait_for(pairs.get(), timeout=5)
            if index >= SETTLE_FRAMES and (index - SETTLE_FRAMES) % FRAME_SPACING == 0:
                kept.append(ColourPair(*(yuyv_to_rgb_with_opencv(pair.eye(eye)) for eye in (0, 1)), pair.frame_number))
    finally:
        node.stop()
        await running
        bus.close()
    return kept


def matched_points(left: np.ndarray, right: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Pixel positions of SIFT features matched between two RGB images (Lowe's ratio test)."""
    import cv2

    sift = cv2.SIFT_create(nfeatures=4000)
    found = [sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY), None) for image in (left, right)]
    (left_points, left_descriptors), (right_points, right_descriptors) = found
    if left_descriptors is None or right_descriptors is None:
        return np.zeros((0, 2)), np.zeros((0, 2))
    pairs = cv2.BFMatcher(cv2.NORM_L2).knnMatch(left_descriptors, right_descriptors, k=2)
    good = [best for best, second in (pair for pair in pairs if len(pair) == 2) if best.distance < 0.7 * second.distance]
    return (np.array([left_points[match.queryIdx].pt for match in good]).reshape(-1, 2),
            np.array([right_points[match.trainIdx].pt for match in good]).reshape(-1, 2))


def describe(name: str, left: np.ndarray, right: np.ndarray) -> str:
    if len(left) < 8:
        return f"{name}: only {len(left)} matches"
    dy, disparity = right[:, 1] - left[:, 1], left[:, 0] - right[:, 0]
    kept = np.abs(dy - np.median(dy)) < OUTLIER_PIXELS
    dy, disparity = dy[kept], disparity[kept]
    return (f"{name}: {kept.sum()} of {len(kept)} matches; dy median {np.median(dy):+.2f} px, "
            f"|dy| median {np.median(np.abs(dy)):.2f} px, 90th percentile {np.percentile(np.abs(dy), 90):.2f} px; "
            f"disparity median {np.median(disparity):.1f} px")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--resolution", type=ZedResolution, default=ZedResolution.VGA, choices=list(ZedResolution))
    parser.add_argument("--pairs", type=int, default=3, help="How many pairs to pool (default 3)")
    parser.add_argument("--save", type=Path, help="Folder for the last rectified pair with row lines")
    args = parser.parse_args()

    pairs = asyncio.run(capture_pairs(args.resolution, args.pairs))
    height, width = pairs[0].left.shape[:2]
    rectifier = zed_rectifier((width, height))
    shared = rectifier.rectification
    print(f"{width}x{height} per eye; rectified focal length {shared.focal_length:.1f} px, "
          f"centre ({shared.cx:.1f}, {shared.cy:.1f}), baseline {1000 * shared.baseline:.1f} mm")
    found = {"raw": [], "rectified": []}
    for pair in pairs:
        found["raw"].append(matched_points(pair.left, pair.right))
        found["rectified"].append(matched_points(*rectifier.rectify(pair.left, pair.right)))
    for name, matches in found.items():
        print(describe(name, np.concatenate([left for left, _ in matches]), np.concatenate([right for _, right in matches])))
    if args.save:
        import cv2

        canvas = np.hstack(rectifier.rectify(pairs[-1].left, pairs[-1].right))
        canvas[::24] = (0, 255, 0)
        args.save.mkdir(parents=True, exist_ok=True)
        path = args.save / f"rectified-{args.resolution.lower()}.jpg"
        cv2.imwrite(str(path), cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
        print(f"Saved {path}")


if __name__ == "__main__":
    main()
