"""Measure a head servo's real travel: the camera's view angle against the servo angle over its whole range.

Steps the pan (or with --servo tilt, the tilt) servo from the middle down to `--low`, up to `--high` and back to the middle, `--step`
servo degrees at a time, holding each step while the OAK (set up as the bot sets it up) watches a
still scene, and integrates each step's picture shift (phase correlation, dx) into the view angle at
PX_PER_DEGREE (dx for pan, dy for tilt). Steps that move the picture less than DEAD_PX are dead: the linkage's slack after a
reversal, or an end stop the servo presses against. Prints a table and JSON with the servo range
over which the head actually turns, each way.

The robot moves. Stop the bot, keep people out of the OAK's view, and run under the robot lock:

    flock -w 60 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.check_servo_travel [--servo tilt]

On 2026-11-06 it found pan steady over 10-170 (0.27 degrees of view per servo degree) but jammed after
being held at 5, and tilt steady over 10-140 with its linkage going over centre past ~140.
"""
import argparse
import json

import numpy as np

from scripts.check_motor_directions import SETTLE, latest, shift
from ultraskelly.core.bot.sensory.oak_models import without_telemetry

PX_PER_DEGREE = 8.1  # The OAK-1's 640 px over 69 degrees, near the middle
DEAD_PX = 3.0  # A step moving the picture less than this turned nothing


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure a head servo's view angle over its whole range.")
    parser.add_argument("--low", type=float, default=10.0)
    parser.add_argument("--high", type=float, default=170.0)
    parser.add_argument("--step", type=float, default=10.0)
    parser.add_argument("--servo", choices=["pan", "tilt"], default="pan", help="Which servo to sweep")
    args = parser.parse_args()
    without_telemetry()
    import cv2

    from ultraskelly.core.bot.motor.motor_boards import open_servo_board
    from ultraskelly.core.bot.motor.orientation_node import ROLL_LEVEL, OrientationNodeParams
    from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams
    from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

    head = OrientationNodeParams()
    channel_under_test = head.pan_channel if args.servo == "pan" else head.tilt_channel
    axis = "dx" if args.servo == "pan" else "dy"
    servos = open_servo_board()
    camera = OakCameraNode.create(pubsub=PubSubTopicManager.create(), params=OakCameraNodeParams(flip_camera=True))
    down = list(np.arange(90.0 - args.step, args.low - 1e-6, -args.step))
    up = list(np.arange(args.low + args.step, args.high + 1e-6, args.step))
    back = list(np.arange(args.high - args.step, 90.0 - 1e-6, -args.step))
    rows, view = [], 0.0
    try:
        for channel, angle in ((head.pan_channel, 90.0), (head.tilt_channel, 90.0), (head.roll_channel, ROLL_LEVEL)):
            servos.servo[channel].angle = angle
        camera.pipeline.start()
        frames = camera.queues["frames"]
        before, servo = latest(frames, 4.0), 90.0
        for target in down + up + back:
            servos.servo[channel_under_test].angle = float(target)
            after = latest(frames, SETTLE)
            moved = shift(cv2, before, after)
            view += -moved[axis] / PX_PER_DEGREE  # The scene moving left (up) means the view turned right (down)
            rows.append({"from": servo, "to": float(target), "dx": moved[axis], "response": moved["response"],
                         "view": round(view, 2), "dead": abs(moved[axis]) < DEAD_PX})
            before, servo = after, float(target)
    finally:
        servos.servo[channel_under_test].angle = 90.0
        camera.pipeline.stop()
        camera.pipeline.getDefaultDevice().close()
    for row in rows:
        print(f"{args.servo} {row['from']:5.1f} -> {row['to']:5.1f}: dx {row['dx']:+6.1f} px (r {row['response']:.2f})  "
              f"view {row['view']:+6.1f} deg{'  DEAD' if row['dead'] else ''}")
    live = [row for row in rows if not row["dead"]]
    summary = {"live_servo_range": [min(min(r["from"], r["to"]) for r in live), max(max(r["from"], r["to"]) for r in live)]
               if live else None,
               "view_range_deg": [min(r["view"] for r in rows), max(r["view"] for r in rows)],
               "dead_steps": [(r["from"], r["to"]) for r in rows if r["dead"]]}
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
