"""Measure which way each head motor turns the OAK's view, and how far.

Moves the pan, tilt and roll servos and the waist motor a little, one at a time, while the OAK
(set up as the bot sets it up, upside down and without models) watches a still scene, and finds
how far the picture shifted after each move by phase correlation, and for roll how far it
turned, from matched features. The tracking controller (OrientationNode) assumes:

- pan: more degrees turns the view right, so the scene moves LEFT in the picture (dx < 0); it
  raises pan for a person right of the middle;
- tilt: more degrees turns the view down, so the scene moves UP (dy < 0); it raises tilt for a
  person below the middle;
- waist: negative drive turns the body the way more pan turns the head (WaistDrive drives
  negative to follow positive pan), so it moves the scene the way more pan does;
- roll: more degrees turns the picture's content anticlockwise (RollLevelParams.direction +1),
  by RollLevelParams.ratio degrees per servo degree. Neither is measured yet: this measures them.

Prints JSON: each move's shift (dx right, dy down, in pixels of the 640x480 picture; `response`
is the correlation peak, low when the view has too little detail), for roll the picture's turn
(`degrees`, clockwise on screen, with how many matched features agreed), the pixels or degrees
per servo degree, and whether each motor turns the way the controller assumes. --save writes
labelled pictures.

The robot moves. Stop the bot, keep people out of the OAK's view, and run under the robot lock:

    flock -w 60 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.check_motor_directions --save ~/motor-directions.jpg
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.oak_models import without_telemetry

SERVO_STEP = 12.0  # Degrees each way from the middle
SETTLE = 1.0  # Seconds for a servo or the waist to stop and the picture to catch up
WAIST_DRIVE, WAIST_SECONDS = 0.7, 0.5  # Within the dashboard's motor test limits
LEAST_SHIFT = 3.0  # Pixels; less is no movement
LEAST_RESPONSE = 0.05  # Correlation peaks below this are no measurement
LEAST_MATCHES = 12  # Matched features that must agree on a turn for it to count
LEAST_TURN = 0.5  # Degrees; less is no turn


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


def rotation(cv2, before: np.ndarray, after: np.ndarray) -> dict:
    """How far the scene turned from `before` to `after`, degrees clockwise on screen (image y points
    down), from matched ORB features: a turn, a uniform scale and a shift fitted with RANSAC."""
    grey = [cv2.cvtColor(np.ascontiguousarray(image), cv2.COLOR_RGB2GRAY) for image in (before, after)]
    orb = cv2.ORB_create(nfeatures=1500)
    (points0, features0), (points1, features1) = (orb.detectAndCompute(image, None) for image in grey)
    nothing = {"degrees": None, "dx": 0.0, "dy": 0.0, "matches": 0, "inliers": 0}
    if features0 is None or features1 is None:
        return nothing
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(features0, features1)
    if len(matches) < LEAST_MATCHES:
        return nothing | {"matches": len(matches)}
    source = np.float32([points0[match.queryIdx].pt for match in matches])
    target = np.float32([points1[match.trainIdx].pt for match in matches])
    fitted, inliers = cv2.estimateAffinePartial2D(source, target, method=cv2.RANSAC, ransacReprojThreshold=3.0)
    if fitted is None:
        return nothing | {"matches": len(matches)}
    agreed = int(inliers.sum())
    degrees = math.degrees(math.atan2(fitted[1, 0], fitted[0, 0]))
    return {"degrees": round(degrees, 2) if agreed >= LEAST_MATCHES else None,
            "dx": round(float(fitted[0, 2]), 1), "dy": round(float(fitted[1, 2]), 1),
            "matches": len(matches), "inliers": agreed}


def turned(move: dict) -> bool:
    return move["degrees"] is not None and abs(move["degrees"]) >= LEAST_TURN


def seen(move: dict, axis: str) -> bool:
    return move["response"] >= LEAST_RESPONSE and abs(move[axis]) >= LEAST_SHIFT


def judge(results: dict, roll_direction: int = 1) -> dict:
    """Per motor: the direction it turns the view, and whether the controller assumes that direction
    (`roll_direction`: +1 if more roll should turn the picture's content anticlockwise)."""
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
    up, down = results.get(f"roll +{SERVO_STEP:g} deg"), results.get(f"roll -{SERVO_STEP:g} deg")
    if up is not None and down is not None and turned(up) and turned(down) and np.sign(up["degrees"]) != np.sign(down["degrees"]):
        anticlockwise = up["degrees"] < 0  # Clockwise on screen is positive
        verdicts["roll"] = {"more_degrees_turns_content": "anticlockwise" if anticlockwise else "clockwise",
                            "picture_degrees_per_servo_degree": round((abs(up["degrees"]) + abs(down["degrees"])) / (2 * SERVO_STEP), 3),
                            "matches_controller": bool(anticlockwise == (roll_direction == 1))}
    elif up is not None:
        verdicts["roll"] = {"result": "no clear turn"}
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
            if name == "before":
                text = f"{label}: {name}"
            elif "degrees" in move:
                text = "scene turned " + ("?" if move["degrees"] is None else f"{move['degrees']:+.1f} deg clockwise")
            else:
                text = f"scene moved dx {move['dx']:+.0f}, dy {move['dy']:+.0f} px"
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
    from ultraskelly.core.bot.motor.roll_level import RollLevelParams
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
        for name, channel in (("pan", head.pan_channel), ("tilt", head.tilt_channel), ("roll", head.roll_channel)):
            measure = rotation if name == "roll" else shift
            for step in (SERVO_STEP, -SERVO_STEP):
                label = f"{name} {step:+g} deg"
                servos.servo[channel].angle = 90.0 + step
                moved = latest(frames, SETTLE)
                servos.servo[channel].angle = 90.0
                back = latest(frames, SETTLE)
                results[label] = measure(cv2, middle, moved) | {"back_to_middle": measure(cv2, middle, back)}
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
        for channel in (head.pan_channel, head.tilt_channel, head.roll_channel):
            servos.servo[channel].angle = 90.0
        node.pipeline.stop()
        node.pipeline.getDefaultDevice().close()
    print(json.dumps({"moves": results, "verdict": judge(results, RollLevelParams().direction)}, indent=1))
    if args.save:
        save(cv2, args.save, pictures, results)


if __name__ == "__main__":
    main()
