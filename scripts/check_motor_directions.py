"""Measure which way each head motor turns the OAK's view, and how far.

Moves the pan servo, the tilt servo and the waist motor a little, one at a time, while the OAK
(set up as the bot sets it up, upside down and without models) watches a still scene, and finds
how far the picture shifted after each move by phase correlation. The tracking controller
(OrientationNode) assumes:

- pan: more degrees turns the view right, so the scene moves LEFT in the picture (dx < 0); it
  raises pan for a person right of the middle;
- tilt: more degrees turns the view down, so the scene moves UP (dy < 0); it raises tilt for a
  person below the middle;
- waist: negative drive turns the body the way more pan turns the head (WaistDrive drives
  negative to follow positive pan), so it moves the scene the way more pan does.

Prints JSON: each move's shift (dx right, dy down, in pixels of the 640x480 picture; `response`
is the correlation peak, low when the view has too little detail), the pixels per degree, and
whether each motor turns the way the controller assumes. --save writes labelled pictures.

The robot moves. Stop the bot, keep people out of the OAK's view, and run under the robot lock:

    flock -w 60 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.check_motor_directions --save ~/motor-directions.jpg
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.oak_models import without_telemetry

SERVO_STEP = 12.0  # Degrees each way from the middle
SETTLE = 1.0  # Seconds for a servo or the waist to stop and the picture to catch up
WAIST_DRIVE, WAIST_SECONDS = 0.7, 0.5  # Within the dashboard's motor test limits
LEAST_SHIFT = 3.0  # Pixels; less is no movement
LEAST_RESPONSE = 0.05  # Correlation peaks below this are no measurement


def latest(frames, seconds: float) -> np.ndarray:
    """The newest RGB frame once `seconds` have passed."""
    end = time.monotonic() + seconds
    group = frames.get()
    while time.monotonic() < end or frames.has():
        group = frames.get()
    return group["rgb"].getFrame().copy()


def shift(cv2, before: np.ndarray, after: np.ndarray) -> dict:
    """How far the scene moved from `before` to `after`: dx right and dy down, in pixels."""
    grey = [cv2.cvtColor(np.ascontiguousarray(image), cv2.COLOR_RGB2GRAY).astype(np.float32) for image in (before, after)]
    height, width = grey[0].shape
    (dx, dy), response = cv2.phaseCorrelate(grey[0], grey[1], cv2.createHanningWindow((width, height), cv2.CV_32F))
    return {"dx": round(float(dx), 1), "dy": round(float(dy), 1), "response": round(float(response), 3)}


def seen(move: dict, axis: str) -> bool:
    return move["response"] >= LEAST_RESPONSE and abs(move[axis]) >= LEAST_SHIFT


def judge(results: dict) -> dict:
    """Per motor: the direction it turns the view, and whether the controller assumes that direction."""
    up, down = results[f"pan +{SERVO_STEP:g} deg"], results[f"pan -{SERVO_STEP:g} deg"]
    verdicts = {}
    if seen(up, "dx") and seen(down, "dx") and np.sign(up["dx"]) != np.sign(down["dx"]):
        verdicts["pan"] = {"more_degrees_moves_scene": "left" if up["dx"] < 0 else "right",
                           "px_per_degree": round((abs(up["dx"]) + abs(down["dx"])) / (2 * SERVO_STEP), 2),
                           "matches_controller": bool(up["dx"] < 0)}
    else:
        verdicts["pan"] = {"result": "no clear movement"}
    up, down = results[f"tilt +{SERVO_STEP:g} deg"], results[f"tilt -{SERVO_STEP:g} deg"]
    if seen(up, "dy") and seen(down, "dy") and np.sign(up["dy"]) != np.sign(down["dy"]):
        verdicts["tilt"] = {"more_degrees_moves_scene": "up" if up["dy"] < 0 else "down",
                            "px_per_degree": round((abs(up["dy"]) + abs(down["dy"])) / (2 * SERVO_STEP), 2),
                            "matches_controller": bool(up["dy"] < 0)}
    else:
        verdicts["tilt"] = {"result": "no clear movement"}
    negative = results[f"waist -{WAIST_DRIVE:g} for {WAIST_SECONDS:g} s"]
    if seen(negative, "dx") and "more_degrees_moves_scene" in verdicts["pan"]:
        pan_left = verdicts["pan"]["more_degrees_moves_scene"] == "left"
        verdicts["waist"] = {"negative_drive_moves_scene": "left" if negative["dx"] < 0 else "right",
                             "matches_controller": bool((negative["dx"] < 0) == pan_left)}
    else:
        verdicts["waist"] = {"result": "no clear movement" if not seen(negative, "dx") else "pan unclear"}
    return verdicts


def save(cv2, path: Path, pictures: list[tuple[str, np.ndarray, np.ndarray]], results: dict) -> None:
    """Each move as a row: before, after, and the measured shift drawn on the after picture."""
    rows = []
    for label, before, after in pictures:
        move = results[label]
        tiles = []
        for name, rgb in (("before", before), ("after", after)):
            tile = cv2.cvtColor(cv2.resize(np.ascontiguousarray(rgb), (320, 240)), cv2.COLOR_RGB2BGR)
            cv2.rectangle(tile, (0, 0), (320, 22), (0, 0, 0), -1)
            text = f"{label}: {name}" if name == "before" else f"scene moved dx {move['dx']:+.0f}, dy {move['dy']:+.0f} px"
            cv2.putText(tile, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            if name == "after":
                start = (160, 130)
                end = (int(160 + move["dx"] / 2), int(130 + move["dy"] / 2))  # Half size, as the tile is
                cv2.arrowedLine(tile, start, end, (0, 255, 255), 2, cv2.LINE_AA, tipLength=0.25)
            tiles.append(tile)
        rows.append(np.hstack(tiles))
    cv2.imwrite(str(path), np.vstack(rows))


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure which way each head motor turns the OAK's view.")
    parser.add_argument("--save", type=Path, default=None, help="Write labelled before/after pictures to this JPEG")
    args = parser.parse_args()
    without_telemetry()
    import cv2

    from ultraskelly.bot_launcher import LaunchConfig
    from ultraskelly.core.bot.motor.bridge_motor import BridgeMotor
    from ultraskelly.core.bot.motor.motor_boards import open_motor_board, open_servo_board
    from ultraskelly.core.bot.motor.orientation_node import ROLL_LEVEL, OrientationNodeParams
    from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams
    from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

    head = OrientationNodeParams()
    servos = open_servo_board()
    waist = BridgeMotor.from_motorkit(open_motor_board(LaunchConfig.model_fields["motor_pwm_hz"].default),
                                      head.waist_motor_channel)
    node = OakCameraNode.create(pubsub=PubSubTopicManager.create(), params=OakCameraNodeParams(flip_camera=True))
    results, pictures = {}, []
    try:
        for channel, angle in ((head.pan_channel, 90.0), (head.tilt_channel, 90.0), (head.roll_channel, ROLL_LEVEL)):
            servos.servo[channel].angle = angle
        node.pipeline.start()
        frames = node.queues["frames"]
        middle = latest(frames, 4.0)  # Exposure and white balance settle meanwhile
        for name, channel in (("pan", head.pan_channel), ("tilt", head.tilt_channel)):
            for step in (SERVO_STEP, -SERVO_STEP):
                label = f"{name} {step:+g} deg"
                servos.servo[channel].angle = 90.0 + step
                moved = latest(frames, SETTLE)
                servos.servo[channel].angle = 90.0
                back = latest(frames, SETTLE)
                results[label] = shift(cv2, middle, moved) | {"back_to_middle": shift(cv2, middle, back)}
                pictures.append((label, middle, moved))
                middle = back
        for drive in (-WAIST_DRIVE, WAIST_DRIVE):  # Out and back
            label = f"waist {drive:+g} for {WAIST_SECONDS:g} s"
            before = latest(frames, 0.3)
            waist.throttle = drive
            time.sleep(WAIST_SECONDS)
            waist.throttle = 0.0  # Brake
            after = latest(frames, SETTLE)
            results[label] = shift(cv2, before, after)
            pictures.append((label, before, after))
    finally:
        waist.throttle = 0.0
        for channel in (head.pan_channel, head.tilt_channel):
            servos.servo[channel].angle = 90.0
        node.pipeline.stop()
        node.pipeline.getDefaultDevice().close()
    print(json.dumps({"moves": results, "verdict": judge(results)}, indent=1))
    if args.save:
        save(cv2, args.save, pictures, results)


if __name__ == "__main__":
    main()
