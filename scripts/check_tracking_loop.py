"""Close the tracking loop on the robot without a person: the real head controller centres a patch of the scene.

Picks a detailed patch of the OAK's view off the middle, follows it with template matching on
every frame, and gives where it is to OrientationNode's head update as the target, which drives
the real pan and tilt servos (the waist stays still). If the picture-to-servo mapping is right,
the patch comes to the middle (320, 240); an axis mapped wrong (sign or channel) runs away
instead. `--delay-frames` hands the controller old positions, as a slow detector would.

Prints each run's track and a verdict per axis; --save writes the first and last picture of each
run with the patch marked.

Moves the head. Stop the bot and run under the robot lock:

    flock -w 60 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.check_tracking_loop --save ~/tracking-loop.jpg
"""
import argparse
import json
import time
from collections import deque
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.oak_models import without_telemetry

PATCH = 48  # Pixels on a side
SEARCH = 90  # Pixels the patch may move between frames
LEAST_MATCH = 0.5  # Normalized correlation below which the patch is lost
RUN_SECONDS = 6.0
STARTS = ((130, 70), (-130, -70))  # Patch offsets from the middle: right and down, then left and up


def grey(cv2, rgb: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2GRAY)


def newest(frames) -> np.ndarray:
    group = frames.get()
    while frames.has():
        group = frames.get()
    return group["rgb"].getFrame().copy()


def detailed_patch(image: np.ndarray, centre: tuple[int, int]) -> tuple[int, int]:
    """The centre of the most detailed PATCH-sized square within 40 pixels of `centre`."""
    best, best_detail = centre, -1.0
    half = PATCH // 2
    height, width = image.shape
    for dy in range(-40, 41, 8):
        for dx in range(-40, 41, 8):
            x, y = centre[0] + dx, centre[1] + dy
            if half <= x < width - half and half <= y < height - half:
                detail = float(image[y - half:y + half, x - half:x + half].std())
                if detail > best_detail:
                    best, best_detail = (x, y), detail
    return best


def find(cv2, image: np.ndarray, template: np.ndarray, near: tuple[float, float]) -> tuple[tuple[float, float], float]:
    """Where the template's centre is now, searching around `near`, and the match score."""
    half = PATCH // 2
    height, width = image.shape
    x0, y0 = max(0, int(near[0]) - half - SEARCH), max(0, int(near[1]) - half - SEARCH)
    x1, y1 = min(width, int(near[0]) + half + SEARCH), min(height, int(near[1]) + half + SEARCH)
    scores = cv2.matchTemplate(image[y0:y1, x0:x1], template, cv2.TM_CCOEFF_NORMED)
    _, score, _, (bx, by) = cv2.minMaxLoc(scores)
    return (x0 + bx + half, y0 + by + half), float(score)


def judge(track: list[dict], axis: str, middle: float) -> str:
    start, end = track[0][axis] - middle, track[-1][axis] - middle
    worst = max(abs(row[axis] - middle) for row in track)
    if abs(end) <= max(25.0, 0.4 * abs(start)):
        return f"came to the middle ({start:+.0f} -> {end:+.0f} px)"
    if worst > abs(start) + 40 and np.sign(end) == np.sign(start):
        return f"RAN AWAY ({start:+.0f} -> {end:+.0f} px): this axis is mapped wrong"
    return f"did not settle ({start:+.0f} -> {end:+.0f} px)"


def main() -> None:
    parser = argparse.ArgumentParser(description="Close the head's tracking loop on a patch of the scene.")
    parser.add_argument("--gain", type=float, default=None, help="Head gain (degrees per pixel per frame); the bot's by default")
    parser.add_argument("--delay-frames", type=int, default=0, help="Hand the controller positions this many frames old")
    parser.add_argument("--save", type=Path, default=None, help="Write the first and last picture of each run to this JPEG")
    args = parser.parse_args()
    without_telemetry()
    import cv2

    from ultraskelly.core.bot.motor.motor_boards import open_servo_board
    from ultraskelly.core.bot.motor.orientation_node import ROLL_LEVEL, OrientationNode, OrientationNodeParams
    from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams
    from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

    params = OrientationNodeParams() if args.gain is None else OrientationNodeParams(head_gain=args.gain)
    servos = open_servo_board()
    bus = PubSubTopicManager.create()
    camera = OakCameraNode.create(pubsub=bus, params=OakCameraNodeParams(flip_camera=True))
    middle_x, middle_y = params.target_x + params.x_offset, params.target_y + params.y_offset
    runs, pictures = [], []
    try:
        for channel, angle in ((params.pan_channel, 90.0), (params.tilt_channel, 90.0), (params.roll_channel, ROLL_LEVEL)):
            servos.servo[channel].angle = angle
        camera.pipeline.start()
        frames = camera.queues["frames"]
        end = time.monotonic() + 4.0  # Exposure settles
        while time.monotonic() < end:
            frames.get()
        for offset in STARTS:
            head = OrientationNode(pubsub=bus, params=params, servo_kit=servos)
            head.command_start_angles()
            time.sleep(1.0)
            first = newest(frames)
            image = grey(cv2, first)
            centre = detailed_patch(image, (middle_x + offset[0], middle_y + offset[1]))
            half = PATCH // 2
            template = image[centre[1] - half:centre[1] + half, centre[0] - half:centre[0] + half].copy()
            position, delayed, track = (float(centre[0]), float(centre[1])), deque(), []
            start, last = time.monotonic(), first
            while time.monotonic() - start < RUN_SECONDS:
                last = newest(frames)
                position, score = find(cv2, grey(cv2, last), template, position)
                if score < LEAST_MATCH:
                    track.append({"t": round(time.monotonic() - start, 2), "lost": True})
                    break
                delayed.append(position)
                if len(delayed) > args.delay_frames:
                    seen_x, seen_y = delayed.popleft()
                    head._update_head_servos(target_x=int(round(seen_x)), target_y=int(round(seen_y)))
                track.append({"t": round(time.monotonic() - start, 2), "x": round(position[0]), "y": round(position[1]),
                              "pan": round(head.pan_angle, 1), "tilt": round(head.tilt_angle, 1), "match": round(score, 2)})
            kept = [row for row in track if "x" in row]
            run = {"start_offset": offset, "frames": len(kept), "lost": any(row.get("lost") for row in track)}
            if kept:
                run["pan_axis"] = judge(kept, "x", middle_x)
                run["tilt_axis"] = judge(kept, "y", middle_y)
                run["track"] = kept[:: max(1, len(kept) // 12)] + [kept[-1]]
            runs.append(run)
            pictures.append((offset, first, centre, last, kept[-1] if kept else None))
    finally:
        for channel in (params.pan_channel, params.tilt_channel):
            servos.servo[channel].angle = 90.0
        camera.pipeline.stop()
        camera.pipeline.getDefaultDevice().close()
    print(json.dumps({"gain": params.head_gain, "delay_frames": args.delay_frames, "runs": runs}, indent=1))
    if args.save:
        rows = []
        for offset, first, centre, last, end_row in pictures:
            tiles = []
            for label, rgb, point in (("start", first, centre), ("end", last, (end_row["x"], end_row["y"]) if end_row else None)):
                tile = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR)
                cv2.drawMarker(tile, (middle_x, middle_y), (255, 255, 255), cv2.MARKER_CROSS, 40, 2)
                if point is not None:
                    half = PATCH // 2
                    cv2.rectangle(tile, (int(point[0]) - half, int(point[1]) - half),
                                  (int(point[0]) + half, int(point[1]) + half), (0, 255, 255), 3)
                tile = cv2.resize(tile, (320, 240))
                cv2.rectangle(tile, (0, 0), (320, 22), (0, 0, 0), -1)
                cv2.putText(tile, f"patch started {offset[0]:+d},{offset[1]:+d}: {label}", (6, 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
                tiles.append(tile)
            rows.append(np.hstack(tiles))
        cv2.imwrite(str(args.save), np.vstack(rows))


if __name__ == "__main__":
    main()
