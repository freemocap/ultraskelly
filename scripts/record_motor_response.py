"""Record how each head motor's commands turn the camera: for building a simulated head that matches the robot.

Runs a fixed script on the pan, tilt and roll servos and the waist motor, while a thread keeps every
OAK frame (grey, 320x240, with its capture time on time.monotonic()'s clock) and the main thread logs
every command with its time. Per servo: steps of several sizes from the middle and back, a staircase
up and down (gearing each way, slack at reversals), ramps at several speeds (lag, top speed) and big
steps (delay, rise, overshoot, settling). Then short waist pulses each way. Servos stay inside the
windows where their linkages turn steadily (pan 20-160, tilt 20-130, roll 60-120; check_servo_travel.py).

Saves to ~/ultraskelly-recordings/motor-response-<time>/: frames.npz (frames, captured, received),
commands.csv (time, actuator, value) and segments.json (each part's name and start and end time).

The robot moves. Stop the bot, have the scene still (a person may stand in view), and run under the lock:

    flock -w 60 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.record_motor_response
"""
import csv
import json
import threading
import time
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.oak_models import without_telemetry

MIDDLE = 90.0
WINDOWS = {"pan": (20.0, 160.0), "tilt": (20.0, 130.0), "roll": (60.0, 120.0)}
STEP_HOLD = 1.2  # Seconds a step is held: the head settles well within this
TICK = 0.01  # Seconds between ramp commands


class Recorder:
    """Keeps every frame the OAK sends, grey and halved, with its capture and arrival times."""

    def __init__(self, cv2, frames) -> None:
        self.cv2, self.frames = cv2, frames
        self.images, self.captured, self.received = [], [], []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self.stop.is_set():
            group = self.frames.get()
            rgb = group["rgb"]
            grey = self.cv2.cvtColor(rgb.getFrame(), self.cv2.COLOR_RGB2GRAY)
            self.images.append(self.cv2.resize(grey, (320, 240), interpolation=self.cv2.INTER_AREA))
            self.captured.append(rgb.getTimestamp().total_seconds())
            self.received.append(time.monotonic())


class Script:
    """Sends commands and logs them, and marks the parts of the run."""

    def __init__(self, servos, channels: dict[str, int], waist) -> None:
        self.servos, self.channels, self.waist = servos, channels, waist
        self.angles = {name: MIDDLE for name in channels}
        self.commands: list[tuple[float, str, float]] = []
        self.segments: list[dict] = []

    def send(self, name: str, value: float) -> None:
        if name == "waist":
            self.waist.throttle = value
        else:
            low, high = WINDOWS[name]
            value = min(max(value, low), high)
            self.servos.servo[self.channels[name]].angle = value
            self.angles[name] = value
        self.commands.append((time.monotonic(), name, float(value)))

    def segment(self, name: str):
        script = self

        class Mark:
            def __enter__(self):
                script.segments.append({"name": name, "start": time.monotonic()})

            def __exit__(self, *exc):
                script.segments[-1]["end"] = time.monotonic()

        return Mark()

    def step(self, name: str, angle: float, hold: float = STEP_HOLD) -> None:
        self.send(name, angle)
        time.sleep(hold)

    def ramp(self, name: str, angle: float, speed: float) -> None:
        start, distance = self.angles[name], angle - self.angles[name]
        moves = max(1, int(abs(distance) / speed / TICK))
        for tick in range(1, moves + 1):
            self.send(name, start + distance * tick / moves)
            time.sleep(TICK)


def servo_script(script: Script, name: str, amplitudes: list[float], stairs: list[float], ramp_path: list[float],
                 big_steps: list[float]) -> None:
    with script.segment(f"{name} settle"):
        script.step(name, MIDDLE, 1.5)
    for amplitude in amplitudes:
        for sign in (1, -1):
            with script.segment(f"{name} step {sign * amplitude:+g}"):
                script.step(name, MIDDLE + sign * amplitude)
                script.step(name, MIDDLE)
    with script.segment(f"{name} staircase"):
        for angle in stairs:
            script.step(name, angle, 1.0)
    for speed in (20.0, 60.0, 150.0):
        with script.segment(f"{name} ramp {speed:g} deg/s"):
            for angle in ramp_path:
                script.ramp(name, angle, speed)
                time.sleep(0.6)
    with script.segment(f"{name} big steps"):
        for angle in big_steps:
            script.step(name, angle, 1.5)


def main() -> None:
    without_telemetry()
    import cv2

    from ultraskelly.bot_launcher import LaunchConfig
    from ultraskelly.core.bot.motor.bridge_motor import BridgeMotor
    from ultraskelly.core.bot.motor.motor_boards import open_motor_board, open_servo_board
    from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
    from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams
    from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

    head = OrientationNodeParams()
    channels = {"pan": head.pan_channel, "tilt": head.tilt_channel, "roll": head.roll_channel}
    servos = open_servo_board()
    waist = BridgeMotor.from_motorkit(open_motor_board(LaunchConfig.model_fields["motor_pwm_hz"].default),
                                      head.waist_motor_channel)
    camera = OakCameraNode.create(pubsub=PubSubTopicManager.create(), params=OakCameraNodeParams(flip_camera=True))
    script = Script(servos, channels, waist)
    folder = Path.home() / "ultraskelly-recordings" / time.strftime("motor-response-%Y%m%d-%H%M%S")
    recorder = None
    try:
        for name in channels:
            script.send(name, MIDDLE)
        camera.pipeline.start()
        recorder = Recorder(cv2, camera.queues["frames"])
        recorder.thread.start()
        time.sleep(3.0)  # Exposure settles
        servo_script(script, "pan", [5, 10, 20, 40, 60], [70, 50, 30, 50, 70, 90, 110, 130, 150, 130, 110, 90],
                     [150, 30, 90], [140, 40, 90])
        servo_script(script, "tilt", [5, 10, 20, 35], [60, 30, 60, 90, 110, 125, 110, 90], [125, 30, 90], [125, 40, 90])
        servo_script(script, "roll", [5, 10, 20, 30], [75, 60, 75, 90, 105, 120, 105, 90], [120, 60, 90], [120, 60, 90])
        for drive, seconds in ((-0.6, 0.25), (0.6, 0.25), (-0.6, 0.5), (0.6, 0.5), (-0.7, 0.5), (0.7, 0.5)):
            with script.segment(f"waist {drive:+g} for {seconds:g} s"):
                script.send("waist", drive)
                time.sleep(seconds)
                script.send("waist", 0.0)
                time.sleep(1.5)
    finally:
        script.send("waist", 0.0)
        for name in channels:
            script.send(name, MIDDLE)
        time.sleep(0.5)
        if recorder is not None:
            recorder.stop.set()
            recorder.thread.join(timeout=2.0)
        camera.pipeline.stop()
        camera.pipeline.getDefaultDevice().close()
    folder.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(folder / "frames.npz", frames=np.array(recorder.images), captured=np.array(recorder.captured),
                        received=np.array(recorder.received))
    with open(folder / "commands.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time", "actuator", "value"])
        writer.writerows(script.commands)
    (folder / "segments.json").write_text(json.dumps(script.segments, indent=1))
    span = recorder.captured[-1] - recorder.captured[0] if recorder.captured else 0.0
    print(f"saved {folder}: {len(recorder.images)} frames over {span:.0f} s "
          f"({len(recorder.images) / max(span, 1e-9):.1f} fps), {len(script.commands)} commands, {len(script.segments)} parts")


if __name__ == "__main__":
    main()
