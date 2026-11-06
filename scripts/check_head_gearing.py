"""Measure the head's gearing each way, the slack in its linkage, and the waist's speed each way.

check_motor_directions found 12 servo degrees of pan moving the picture 11 px one way and 35 px
the other (2026-11-06): either slack in the linkage or a ratio that depends on direction. The
head's controller assumes one ratio per servo (HeadAimParams.pan_ratio, tilt_ratio) and one
waist speed (WaistDriveParams.degrees_per_second), and its closed-loop scenarios pass only
within a range of each (docs/audit-2026.md, "Calm head tracking").

Steps the pan servo up from the middle, down past it and back, then the tilt servo, while the
OAK (set up as the bot sets it up, upright, without models) watches a still scene, and measures
each step's picture shift by phase correlation. A step in the same direction as the one before
gives the ratio that way (degrees of view per servo degree, at 8.1 px per degree near the
middle); the first step after a reversal comes up short by the slack. Then drives the waist each
way at 0.6 for two durations and back: the longer move's extra travel gives the speed, free of
breaking loose and stopping.

Prints JSON: every move, then per servo the ratio each way and the slack in servo degrees, and
the waist's speed each way at 0.6 and at full drive if speed grows with drive. Within the
dashboard's motor test limits: servos within 30 degrees of the middle, the waist at most 0.6 for
0.5 s, as far each way.

The robot moves. Stop the bot, keep people out of the OAK's view, and run under the robot lock:

    flock -w 60 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.check_head_gearing
"""
import json
import statistics
import time

from scripts.check_motor_directions import LEAST_RESPONSE, SETTLE, latest, shift
from ultraskelly.core.bot.sensory.oak_models import without_telemetry

PX_PER_DEGREE = 8.1  # The OAK-1's 640 px over 69 degrees, a pinhole, near the middle
MIDDLE = 90.0
SWEEPS = {"pan": (10.0, 30.0, "dx"), "tilt": (15.0, 30.0, "dy")}  # Step and span in servo degrees, picture axis
WAIST_DRIVE = 0.6
WAIST_SECONDS = (0.25, 0.5)


def sweep(step: float, span: float, middle: float = MIDDLE) -> list[float]:
    """Servo angles after each step: up to middle + span, down to middle - span, back to the middle."""
    count = round(span / step)
    up = [middle + step * i for i in range(1, count + 1)]
    down = [middle + span - step * i for i in range(1, 2 * count + 1)]
    back = [middle - span + step * i for i in range(1, count + 1)]
    return up + down + back


def gearing(angles: list[float], shifts: list[float | None], px_per_degree: float = PX_PER_DEGREE) -> dict:
    """Ratio each way and slack from a sweep: `angles` starts at the middle, `shifts[i]` is the picture's
    shift in pixels during the move from angles[i] to angles[i + 1] (None where nothing was measured)."""
    steady = {"up": [], "down": []}
    after_reversal = []
    for i, (start, end, moved) in enumerate(zip(angles, angles[1:], shifts)):
        if moved is None or end == start:
            continue
        way = "up" if end > start else "down"
        view = abs(moved) / px_per_degree
        if i > 0 and (angles[i] - angles[i - 1]) * (end - start) > 0:
            steady[way].append(view / abs(end - start))
        elif i > 0:
            after_reversal.append((way, abs(end - start), view))
    ratios = {way: round(statistics.median(values), 3) if values else None for way, values in steady.items()}
    slack = [step - view / ratios[way] for way, step, view in after_reversal if ratios[way]]
    return {"ratio_up": ratios["up"], "ratio_down": ratios["down"],
            "slack_servo_degrees": round(statistics.median(slack), 1) if slack else None}


def waist_speeds(moves: dict[float, dict[float, float]], drive: float = WAIST_DRIVE,
                 px_per_degree: float = PX_PER_DEGREE) -> dict:
    """Speed each way from moves[signed drive][seconds] = picture shift in pixels, at `drive` and scaled to full drive."""
    speeds = {}
    for signed, by_time in moves.items():
        (short, near), (long, far) = sorted(by_time.items())
        speed = (abs(far) - abs(near)) / px_per_degree / (long - short)
        way = "negative_drive" if signed < 0 else "positive_drive"
        speeds[way] = {"deg_per_s": round(speed, 1), "at_full_drive": round(speed / drive, 1)}
    return speeds


def main() -> None:
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
    channels = {"pan": head.pan_channel, "tilt": head.tilt_channel}
    moves, summary = {}, {}
    try:
        for channel, angle in ((head.pan_channel, MIDDLE), (head.tilt_channel, MIDDLE), (head.roll_channel, ROLL_LEVEL)):
            servos.servo[channel].angle = angle
        node.pipeline.start()
        frames = node.queues["frames"]
        before = latest(frames, 4.0)  # Exposure and white balance settle meanwhile
        for name, (step, span, axis) in SWEEPS.items():
            angles, shifts, rows = [MIDDLE], [], []
            for angle in sweep(step, span):
                servos.servo[channels[name]].angle = angle
                after = latest(frames, SETTLE)
                moved = shift(cv2, before, after)
                shifts.append(moved[axis] if moved["response"] >= LEAST_RESPONSE else None)
                rows.append({"to": angle} | moved)
                angles.append(angle)
                before = after
            servos.servo[channels[name]].angle = MIDDLE
            moves[name] = rows
            summary[name] = gearing(angles, shifts)
            before = latest(frames, SETTLE)
        waist_moves = {}
        for seconds in WAIST_SECONDS:
            for drive in (-WAIST_DRIVE, WAIST_DRIVE):  # Out and back, so the body ends where it started
                before = latest(frames, SETTLE)
                waist.throttle = drive
                time.sleep(seconds)
                waist.throttle = 0.0  # Brake
                after = latest(frames, SETTLE)
                moved = shift(cv2, before, after)
                moves[f"waist {drive:+g} for {seconds:g} s"] = moved
                waist_moves.setdefault(drive, {})[seconds] = moved["dx"]
        summary["waist"] = waist_speeds(waist_moves)
    finally:
        waist.throttle = 0.0
        for channel in (head.pan_channel, head.tilt_channel):
            servos.servo[channel].angle = MIDDLE
        node.pipeline.stop()
        node.pipeline.getDefaultDevice().close()
    print(json.dumps({"moves": moves, "summary": summary, "controller_assumes": {
        "pan_ratio": head.aim.pan_ratio, "tilt_ratio": head.aim.tilt_ratio,
        "waist_degrees_per_second": head.waist.degrees_per_second}}, indent=1))


if __name__ == "__main__":
    main()
