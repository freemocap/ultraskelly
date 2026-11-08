"""Record how each motor turns the camera, with nothing in the loop: the data the simulated robot is fitted to.

An open-loop, scripted session, short (about seven minutes of motion), with the bot off and under the
robot lock. The camera's exposure, focus and white balance are locked (exposure 4 ms or less where the
room's light allows, so fast turns blur little), and every frame is kept as the OAK's own JPEG with its
stamp, exposure, sensitivity and lens position; the device's calibration is saved beside them. Then:

- 20 s with everything still: the measurement's noise floor;
- pan, tilt and roll, each: a staircase with reversals (gain each way, slack at every reversal), a
  ladder of steps of several sizes in random order with 0-40 ms of random wait before each (dead time
  and slew, sampled finer than a frame), small steps all one way (the head's ringing mode, its gears
  kept in contact), and a random staircase kept back to validate the fit;
- the waist, the head still at its middle: triangle ramps of drive up and down each way (breakaway and
  stall), holds at constant drives each way after a short kick (speed against drive), each ended once
  by coasting and once by braking (stopping distance against speed), and kick-and-hold moves to
  validate the fit.

Servos stay inside WINDOWS, well clear of where their linkages end (tilt never past 130, never held at
an end: holding tilt at its end preceded the robot's power cuts). The waist's turn is measured online
from the camera (simulator/camera_rotation.py, features only, five times a second): after each waist
part it is driven back toward where it started, and any waist motion past WAIST_LIMIT degrees is
braked at once. Without that measurement for a second, the waist stops and its parts are skipped.

Saves to ~/ultraskelly-recordings/motor-response-<time>/: oak.avi (the OAK's Motion JPEG, 640x480),
oak_frames.csv (per frame: index, sequence number, stamp and device stamp in seconds, exposure in
seconds, ISO, lens position, colour temperature, arrival time), commands.csv (time, actuator, value:
each command as its write to the board returned), segments.json (each part's name, start, end and
settings), monitor.csv (the online waist turn) and session.json (camera settings and calibration,
windows, motor settings). scripts/analyze_motor_response.py measures the camera's turn in each frame.

The robot moves. With the bot stopped and the robot clear:

    flock -n /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.record_motor_response
"""
import argparse
import csv
import json
import math
import random
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

MIDDLE = 90.0
WINDOWS = {"pan": (30.0, 150.0), "tilt": (30.0, 130.0), "roll": (60.0, 120.0)}
SIZE = (640, 480)
JPEG_QUALITY = 95
NOISE_FLOOR = 20.0  # Seconds everything is still, first
JITTER = 0.04  # Seconds of random wait before each ladder step: the 20 ms PWM period and the 33 ms frame
LONGEST_EXPOSURE = 0.004  # Seconds; ISO rises to 1600 before exposure grows past this
WAIST_LIMIT = 60.0  # Degrees from the start the waist may be seen turned; past it, the waist brakes
RECENTRE_WITHIN = 8.0  # Degrees: close enough to the start to begin the next waist part
WAIST_KICK = (0.8, 0.06)  # Drive and seconds that break the waist loose before a hold
MONITOR_STALE = 1.0  # Seconds without a measured waist turn that stop the waist
LENS_DEGREES = 67.2  # The OAK-1's view across its 640x480 frames, from its frames (its spec says 66)
CHECK_PAN = (110.0, 3.0, 8.0)  # Pan the head here before the waist moves: the monitor must see 3-8 degrees

LADDERS = {"pan": [2, 5, 10, 20, 40], "tilt": [5, 10, 20, 40], "roll": [10, 20, 30, 45]}  # Step sizes, servo degrees

Step = tuple[float, float, float]  # (seconds of wait before, servo degrees, seconds held after)


def staircase(levels: list[float], hold: float) -> list[Step]:
    return [(0.0, level, hold) for level in levels]


def ladder(start: float, sizes: list[float], repeats: int, window: tuple[float, float], hold: float,
           rng: random.Random) -> list[Step]:
    """Each size each way `repeats` times in random order from `start`, a random 0-JITTER s wait before
    each. A step that would leave `window` waits until the walk has room for it."""
    waiting = [(size, way) for size in sizes for way in (1, -1) for _ in range(repeats)]
    rng.shuffle(waiting)
    steps, at, stuck = [], start, 0
    while waiting:
        size, way = waiting.pop(0)
        if window[0] <= at + way * size <= window[1]:
            at += way * size
            steps.append((rng.uniform(0.0, JITTER), at, hold))
            stuck = 0
            continue
        waiting.append((size, way))
        stuck += 1
        if stuck >= len(waiting):  # No step fits from here: move to the end the next one needs
            size, way = waiting[0]
            at = window[0] if way > 0 else window[1]
            steps.append((rng.uniform(0.0, JITTER), at, hold))
            stuck = 0
    return steps


def one_way_steps(low: float, high: float, size: float, hold: float) -> list[Step]:
    """Small steps up from `low` to `high`, then down again: within each run the gears stay in contact."""
    count = int((high - low) // size)
    ups = [low + size * k for k in range(count + 1)]
    return [(0.0, level, hold) for level in ups + ups[-2::-1]]


def random_staircase(window: tuple[float, float], count: int, dwell: tuple[float, float], rng: random.Random) -> list[Step]:
    return [(0.0, round(rng.uniform(*window), 1), rng.uniform(*dwell)) for _ in range(count)]


def head_plan(rng: random.Random) -> list[tuple[str, str, list[Step]]]:
    """(servo, part, steps) for the three head servos, in order."""
    pan = [90, 100, 110, 100, 110, 120, 130, 120, 130, 140, 130, 120, 110, 120, 110, 100, 90, 80, 70, 80, 70, 60, 50,
           60, 50, 40, 50, 40, 50, 60, 70, 80, 90]
    tilt = [90, 100, 110, 100, 110, 120, 125, 120, 125, 115, 105, 95, 90, 80, 70, 80, 70, 60, 50, 60, 50, 40, 50, 40, 50,
            60, 70, 80, 90]
    roll = [90, 100, 110, 120, 110, 100, 90, 80, 70, 60, 70, 80, 90, 100, 110, 100, 90, 80, 70, 80, 90]
    return [
        ("pan", "staircase", staircase(pan, 1.0)),
        ("pan", "ladder", ladder(90.0, LADDERS["pan"], 4, (45.0, 135.0), 1.0, rng)),
        ("pan", "one way", one_way_steps(66.0, 114.0, 4.0, 0.8)),
        ("pan", "validation", random_staircase((50.0, 130.0), 20, (0.1, 1.0), rng)),
        ("tilt", "staircase", staircase(tilt, 1.0)),
        ("tilt", "ladder", ladder(90.0, LADDERS["tilt"], 4, (40.0, 128.0), 0.8, rng)),
        ("tilt", "one way", one_way_steps(60.0, 120.0, 5.0, 0.6)),
        ("tilt", "validation", random_staircase((45.0, 125.0), 15, (0.1, 0.8), rng)),
        ("roll", "staircase", staircase(roll, 1.0)),
        ("roll", "ladder", ladder(90.0, LADDERS["roll"], 3, (62.0, 118.0), 0.8, rng)),
        ("roll", "one way", one_way_steps(65.0, 115.0, 5.0, 0.5)),
        ("roll", "validation", random_staircase((62.0, 118.0), 10, (0.1, 0.8), rng)),
    ]


@dataclass
class WaistPart:
    name: str
    way: int  # 1: positive drive, -1: negative
    profile: list[tuple[float, float | None]]  # (seconds, drive magnitude; None coasts, 0 brakes)


def waist_plan(top: float) -> list[WaistPart]:
    """Ramps, holds with their stops, and validation moves, alternating ways."""
    parts = []
    for repeat in range(3):
        for way in (-1, 1):
            rise = [(0.1, 0.3 + 0.025 * k) for k in range(1, int((top - 0.3) / 0.025) + 1)]  # 0.25 drive a second
            parts.append(WaistPart(f"waist ramp {way:+d} #{repeat + 1}", way, rise + rise[-2::-1] + [(1.5, 0.0)]))
    levels = [round(0.5 + 0.1 * k, 1) for k in range(int(round((top - 0.5) / 0.1)) + 1)]
    for level in levels:
        for way in (-1, 1):
            for stop, after in (("coast", None), ("brake", 0.0)):
                hold = 1.2 if way < 0 else 2.0  # Negative drive turns faster
                parts.append(WaistPart(f"waist hold {way * level:+.1f} then {stop}", way,
                                       [(WAIST_KICK[1], WAIST_KICK[0]), (hold, level), (1.5, after), (0.1, 0.0)]))
    for repeat in range(2):
        for way in (-1, 1):
            parts.append(WaistPart(f"waist validation {way:+d} #{repeat + 1}", way,
                                   [(0.08, 0.8), (1.0, 0.55), (1.5, 0.0)]))
    return parts


class Clock:
    """Real time; tests give a fake one that advances as it sleeps."""

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


@dataclass
class Session:
    """Runs the plan through `send(actuator, value)`, marking its parts; `turn()` gives the waist's measured
    turn from the start (degrees, positive the view's right) or None, and its age in seconds."""
    send: Callable[[str, float | None], None]
    turn: Callable[[], tuple[float | None, float]]
    clock: Clock = field(default_factory=Clock)
    segments: list[dict] = field(default_factory=list)
    waist_way: int = 1  # The drive sign that turns the view toward smaller yaw, learned from the first waist part
    waist_ok: bool = True

    def part(self, name: str, **settings):
        session = self

        class Mark:
            def __enter__(self):
                session.segments.append({"name": name, "start": session.clock.now(), **settings})

            def __exit__(self, *exc):
                session.segments[-1]["end"] = session.clock.now()

        return Mark()

    def head(self, plan) -> None:
        for name in WINDOWS:
            self.send(name, MIDDLE)
        for servo, part, steps in plan:
            with self.part(f"{servo} {part}", servo=servo):
                for wait, angle, hold in steps:
                    self.clock.sleep(wait)
                    self.send(servo, angle)
                    self.clock.sleep(hold)
            self.send(servo, MIDDLE)
            self.clock.sleep(1.0)

    def _waist_guard(self) -> bool:
        """False, having braked, once the waist is past WAIST_LIMIT or its turn has not been seen lately."""
        seen, age = self.turn()
        if seen is None or age > MONITOR_STALE:
            self.send("waist", 0.0)
            return False
        if abs(seen) > WAIST_LIMIT:
            self.send("waist", 0.0)
            return False
        return True

    def _drive(self, value: float | None, seconds: float) -> bool:
        self.send("waist", value)
        end = self.clock.now() + seconds
        while end - self.clock.now() > 1e-4:
            self.clock.sleep(min(0.02, end - self.clock.now()))
            if value not in (None, 0.0) and not self._waist_guard():
                return False
        return True

    def _fresh_turn(self, patience: float = 3.0) -> float | None:
        """The waist's measured turn, waiting up to `patience` seconds for a fresh one; None ends the waist's parts."""
        waited = 0.0
        while True:
            seen, age = self.turn()
            if seen is not None and age <= MONITOR_STALE:
                return seen
            if waited >= patience:
                self.waist_ok = False
                return None
            self.clock.sleep(0.1)
            waited += 0.1

    def recentre(self, timeout: float = 4.0) -> None:
        """Drive the waist back toward where it started, braking within RECENTRE_WITHIN degrees. A turn that
        grows instead (the drive's sign not what the first part showed) brakes and ends the waist's parts."""
        seen = self._fresh_turn()
        if seen is None or abs(seen) <= RECENTRE_WITHIN:
            return
        began, worst = self.clock.now(), abs(seen)
        way = self.waist_way if seen > 0 else -self.waist_way
        self.send("waist", way * WAIST_KICK[0])
        self.clock.sleep(WAIST_KICK[1])
        self.send("waist", way * 0.65)
        while self.clock.now() - began < timeout:
            self.clock.sleep(0.02)
            now, age = self.turn()
            if now is None or age > MONITOR_STALE:
                self.waist_ok = False
                break
            if abs(now) <= RECENTRE_WITHIN or (now > 0) != (seen > 0):
                break  # Close enough, or across the start
            if self.clock.now() - began < 0.6:  # The reading lags and the waist spins up: wait for it
                worst = max(worst, abs(now))
            elif abs(now) > worst + 5.0:
                self.waist_ok = False  # Going the wrong way
                break
        self.send("waist", 0.0)
        self.clock.sleep(1.0)

    def monitor_sees(self) -> bool:
        """Whether the waist monitor sees a known turn: the head panned to CHECK_PAN's angle shows as a few
        degrees, and back at its middle as none. A monitor that cannot, cannot guard the waist either."""
        angle, low, high = CHECK_PAN
        self.send("pan", angle)
        self.clock.sleep(1.5)
        turned = self._fresh_turn()
        self.send("pan", MIDDLE)
        self.clock.sleep(1.5)
        back = self._fresh_turn()
        seen = turned is not None and back is not None and low <= turned - back <= high and abs(back) < 1.5
        self.segments.append({"name": "monitor check", "start": self.clock.now(), "end": self.clock.now(),
                              "turned": turned, "back": back, "passed": seen})
        return seen

    def waist(self, plan: list[WaistPart]) -> None:
        if not self.monitor_sees():
            self.waist_ok = False
            self.send("waist", 0.0)
            return
        learned = False
        for part in plan:
            if not self.waist_ok:
                break
            self.recentre()
            before = self._fresh_turn()
            if before is None:
                break
            with self.part(part.name, way=part.way, profile=part.profile):
                for seconds, drive in part.profile:
                    if not self._drive(None if drive is None else part.way * drive, seconds):
                        self.segments[-1]["stopped"] = "waist past its limit or unseen"
                        break
            self.send("waist", 0.0)
            after = self._fresh_turn()
            if after is None:
                break
            if not learned and abs(after - before) > 5.0:  # Which way the drive turns the view, from a clear move
                self.waist_way = part.way if after < before else -part.way
                learned = True
        self.send("waist", 0.0)
        if self.waist_ok:
            self.recentre()
        self.send("waist", 0.0)


def plan_seconds(head, waist_parts) -> float:
    """Seconds of scripted motion, without recentring."""
    head_time = sum(wait + hold for _, _, steps in head for wait, _, hold in steps) + len(head) * 1.0
    return head_time + sum(seconds for part in waist_parts for seconds, _ in part.profile)


class FrameKeeper:
    """Keeps every frame's JPEG in an AVI and its metadata in rows; the newest JPEG for the waist monitor."""

    def __init__(self, queue, folder: Path) -> None:
        from ultraskelly.core.bot.recording.mjpeg_avi import MjpegAviWriter

        self.queue, self.stop = queue, threading.Event()
        self.video = MjpegAviWriter(folder / "oak.avi", SIZE, 30.0)
        self.rows: list[tuple] = []
        self.latest: tuple[float, bytes] | None = None
        self.calibration: dict = {}
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self.stop.is_set():
            group = self.queue.tryGet()
            if group is None:
                time.sleep(0.002)
                continue
            arrived = time.monotonic()
            rgb, jpeg = group["rgb"], bytes(group["jpeg"].getData())
            self.video.write(jpeg)
            self.rows.append((len(self.rows), rgb.getSequenceNum(), rgb.getTimestamp().total_seconds(),
                              rgb.getTimestampDevice().total_seconds(), rgb.getExposureTime().total_seconds(),
                              rgb.getSensitivity(), rgb.getLensPosition(), rgb.getColorTemperature(), arrived))
            self.latest = (rgb.getTimestamp().total_seconds(), jpeg)
            if not self.calibration:
                self.calibration = frame_transformation(rgb)


def frame_transformation(frame) -> dict:
    """What the frame says of its own geometry: its intrinsics and the sensor area it came from."""
    out = {}
    try:
        transformation = frame.getTransformation()
        out["matrix"] = [list(map(float, row)) for row in transformation.getIntrinsicMatrix()]
        out["distortion"] = list(map(float, transformation.getDistortionCoefficients()))
        out["size"] = list(transformation.getSize())
        out["source_size"] = list(transformation.getSourceSize())
        out["source_matrix"] = [list(map(float, row)) for row in transformation.getSourceIntrinsicMatrix()]
        out["horizontal_fov"] = float(transformation.getHFov())
        out["crops"] = repr(transformation.getSrcCrops())
    except Exception as error:  # Older depthai, or no calibration on the device
        out["error"] = repr(error)
    return out


class WaistMonitor:
    """The waist's turn from the start, measured about four times a second from the newest frame, features only,
    the robot's own parts in the picture masked (`mask`: 0 there, at the frames' size)."""

    def __init__(self, keeper: FrameKeeper, camera, mask=None) -> None:
        self.keeper, self.camera, self.mask = keeper, camera, mask
        self.tracker, self.reading = None, (None, math.inf)
        self.rows: list[tuple] = []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def turn(self) -> tuple[float | None, float]:
        seen, at = self.reading
        return seen, time.monotonic() - at

    def _run(self) -> None:
        import cv2
        import numpy as np

        from ultraskelly.simulator.camera_rotation import RotationTracker

        while not self.stop.is_set():
            time.sleep(0.2)
            if self.keeper.latest is None:
                continue
            stamp, jpeg = self.keeper.latest
            grey = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
            if self.tracker is None:  # In a dim room a half-size picture has too few features to hold on to
                self.tracker = RotationTracker(self.camera, grey, self_mask=self.mask, refine=False,
                                               new_keyframe_beyond=12.0, features=1500)
                self.reading = (0.0, time.monotonic())
                continue
            found = self.tracker.measure(grey)
            self.rows.append((stamp, found.yaw, found.pitch, found.roll, found.inliers, found.ok, len(self.tracker.keyframes)))
            if found.ok:
                self.reading = (found.yaw, time.monotonic())


def plausible_lens(matrix):
    """The camera for the waist monitor: the device's intrinsics if they are a real lens's, else the OAK-1's
    67.2-degree pinhole (its frames' own calibration, 2026-11-07). An uncalibrated OAK-1 reports placeholder
    intrinsics (a focal length under a pixel), with which every turn reads as none."""
    import numpy as np

    from ultraskelly.simulator.camera_rotation import Camera

    if matrix is not None and 200.0 <= float(matrix[0][0]) <= 2000.0:
        return Camera(np.array(matrix, dtype=float), SIZE)
    return Camera.from_field_of_view(SIZE, LENS_DEGREES)


def camera_settings(rows: list[tuple], cap: float = 0.010) -> dict:
    """Manual settings from auto-exposure's recent frames (FrameKeeper rows): the same brightness with the
    exposure at most LONGEST_EXPOSURE, ISO raised to 1600 before the exposure grows (to `cap` at most)."""
    middle = sorted(rows, key=lambda row: row[4])[len(rows) // 2]
    exposure, iso = middle[4], middle[5]
    light = exposure * iso
    short = min(exposure, LONGEST_EXPOSURE)
    needed = light / short
    if needed > 1600:
        needed, short = 1600, min(light / 1600, cap)
    return {"auto_exposure": exposure, "auto_iso": iso, "exposure": short, "iso": int(round(max(100, needed))),
            "lens_position": int(middle[6]), "colour_temperature": int(middle[7])}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--waist-top", type=float, default=0.8, help="Largest waist drive held (the limits allow 0.8)")
    parser.add_argument("--seed", type=int, default=20261107)
    parser.add_argument("--dry-run", action="store_true", help="Print the plan's length and exit")
    parser.add_argument("--exposure-cap", type=float, default=0.010,
                        help="Longest exposure (s) the camera may lock at in a dim room: sharper below, darker")
    parser.add_argument("--waist-only", action="store_true", help="The still part and the waist's parts, no head parts")
    parser.add_argument("--self-mask", type=Path, default=None,
                        help="A 640x480 image, black where the robot itself is in the picture (analyze_motor_response's "
                             "self-mask.png): kept out of the waist monitor's features")
    parser.add_argument("--camera-only", action="store_true",
                        help="Lock the camera and record 5 s still, without opening the motor boards")
    args = parser.parse_args()
    rng = random.Random(args.seed)
    head, waist_parts = head_plan(rng), waist_plan(args.waist_top)
    if args.waist_only:
        head = []
    print(f"plan: {plan_seconds(head, waist_parts) / 60:.1f} min of scripted motion, "
          f"{sum(len(s) for _, _, s in head)} head steps, {len(waist_parts)} waist parts")
    if args.dry_run:
        return

    from ultraskelly.core.bot.sensory.oak_models import without_telemetry

    without_telemetry()
    import cv2
    import depthai as dai
    import numpy as np

    from ultraskelly.bot_launcher import LaunchConfig
    from ultraskelly.core.bot.motor.actuator_limits import ActuatorLimits
    from ultraskelly.core.bot.motor.bridge_motor import SLOW_DECAY, BridgeMotor
    from ultraskelly.core.bot.motor.motor_boards import open_motor_board, open_servo_board
    from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
    from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder, RecordingMotor, RecordingServoKit
    from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams
    from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
    from ultraskelly.simulator.camera_rotation import Camera

    params = OrientationNodeParams()
    limits = ActuatorLimits(pan_min_degrees=WINDOWS["pan"][0], pan_max_degrees=WINDOWS["pan"][1],
                            tilt_min_degrees=WINDOWS["tilt"][0], tilt_max_degrees=WINDOWS["tilt"][1],
                            roll_min_degrees=WINDOWS["roll"][0], roll_max_degrees=WINDOWS["roll"][1])
    recorder = CommandRecorder()
    pwm_hz = LaunchConfig.model_fields["motor_pwm_hz"].default
    if args.camera_only:
        def send(name: str, value: float | None) -> None:
            recorder.record(name, value)  # Nothing moves
    else:
        servos = RecordingServoKit(kit=open_servo_board(), recorder=recorder, names=params.servo_names, limits=limits)
        bridge = BridgeMotor.from_motorkit(open_motor_board(pwm_hz), params.waist_motor_channel)
        bridge.decay_mode = SLOW_DECAY  # As the bot drives it
        waist = RecordingMotor(motor=bridge, name="waist", recorder=recorder, max_throttle=limits.waist_max_throttle)
        channels = {name: channel for channel, name in params.servo_names.items()}

        def send(name: str, value: float | None) -> None:
            if name == "waist":
                waist.throttle = value
            else:
                servos.servo[channels[name]].angle = value

    camera_node = OakCameraNode.create(pubsub=PubSubTopicManager.create(), params=OakCameraNodeParams(
        width=SIZE[0], height=SIZE[1], fps=30.0, flip_camera=True, jpeg_quality=JPEG_QUALITY, models=None))
    pipeline = camera_node.pipeline
    sensor = next(node for node in pipeline.getAllNodes() if isinstance(node, dai.node.Camera))
    controls = sensor.inputControl.createInputQueue()
    kind = "camera-check" if args.camera_only else "motor-response"
    folder = Path.home() / "ultraskelly-recordings" / time.strftime(f"{kind}-%Y%m%d-%H%M%S")
    folder.mkdir(parents=True)
    keeper = FrameKeeper(camera_node.queues["frames"], folder)
    session_info = {"windows": WINDOWS, "waist_limit": WAIST_LIMIT, "waist_top": args.waist_top, "seed": args.seed,
                    "motor_pwm_hz": pwm_hz, "waist_decay": "slow", "servo_names": params.servo_names,
                    "jpeg_quality": JPEG_QUALITY, "size": SIZE}
    session = None
    monitor = None
    try:
        for name in WINDOWS:
            send(name, MIDDLE)
        send("waist", 0.0)
        pipeline.start()
        keeper.thread.start()
        device = pipeline.getDefaultDevice()
        try:
            calibration = device.readCalibration()
            socket = dai.CameraBoardSocket.CAM_A
            session_info["device_calibration"] = {
                "matrix": [list(map(float, row)) for row in calibration.getCameraIntrinsics(socket, *SIZE)],
                "distortion": list(map(float, calibration.getDistortionCoefficients(socket))),
                "model": str(calibration.getDistortionModel(socket)), "fov": float(calibration.getFov(socket)),
            }
        except Exception as error:
            session_info["device_calibration"] = {"error": repr(error)}
        time.sleep(4.0)  # Auto-exposure, focus and white balance settle
        settings = camera_settings(keeper.rows[-15:], args.exposure_cap)
        control = dai.CameraControl()
        control.setManualExposure(int(settings["exposure"] * 1e6), settings["iso"])
        if settings["lens_position"] > 0:  # A lens position the frames report: hold it
            control.setAutoFocusMode(dai.CameraControl.AutoFocusMode.OFF)
            control.setManualFocus(settings["lens_position"])
        if settings["colour_temperature"] > 0:
            control.setManualWhiteBalance(settings["colour_temperature"])
        controls.send(control)
        time.sleep(1.5)
        settings["applied"] = {"exposure": keeper.rows[-1][4], "iso": keeper.rows[-1][5], "lens_position": keeper.rows[-1][6]}
        session_info["camera"] = settings
        session_info["frame_transformation"] = keeper.calibration
        print("camera:", settings)
        lens = plausible_lens(session_info["device_calibration"].get("matrix") or keeper.calibration.get("matrix"))
        robot = None if args.self_mask is None else cv2.imread(str(args.self_mask), cv2.IMREAD_GRAYSCALE)
        monitor = WaistMonitor(keeper, lens, robot)
        session = Session(send=send, turn=monitor.turn)
        with session.part("still", seconds=5.0 if args.camera_only else NOISE_FLOOR):
            time.sleep(5.0 if args.camera_only else NOISE_FLOOR)
        if args.camera_only:
            return
        monitor.thread.start()
        if not args.waist_only:
            session.head(head)
        for name in ("pan", "tilt", "roll"):  # Settle the head's slack one way before the waist turns it
            send(name, MIDDLE - 5.0)
        time.sleep(0.5)
        for name in ("pan", "tilt", "roll"):
            send(name, MIDDLE)
        time.sleep(1.0)
        session.waist(waist_parts)
    finally:
        send("waist", 0.0)
        for name in WINDOWS:
            send(name, MIDDLE)
        time.sleep(0.5)
        if monitor is not None:
            monitor.stop.set()
        keeper.stop.set()
        keeper.thread.join(timeout=2.0)
        keeper.video.release()
        pipeline.stop()
        pipeline.getDefaultDevice().close()
        with open(folder / "oak_frames.csv", "w", newline="") as file:
            rows = csv.writer(file)
            rows.writerow(["index", "sequence", "stamp", "device_stamp", "exposure", "iso", "lens_position",
                           "colour_temperature", "arrived"])
            rows.writerows(keeper.rows)
        with open(folder / "commands.csv", "w", newline="") as file:
            rows = csv.writer(file)
            rows.writerow(["time", "actuator", "value"])
            rows.writerows((c.at, c.actuator, "" if c.value is None else c.value) for c in recorder.history)
        if monitor is not None:
            with open(folder / "monitor.csv", "w", newline="") as file:
                rows = csv.writer(file)
                rows.writerow(["stamp", "yaw", "pitch", "roll", "inliers", "ok", "keyframes"])
                rows.writerows(monitor.rows)
        session_info["segments"] = len(session.segments) if session else 0
        session_info["waist_way"] = session.waist_way if session else None
        session_info["waist_completed"] = session.waist_ok if session else False
        (folder / "segments.json").write_text(json.dumps(session.segments if session else [], indent=1))
        (folder / "session.json").write_text(json.dumps(session_info, indent=1, default=str))
        sequence = [row[1] for row in keeper.rows]
        missed = sum(b - a - 1 for a, b in zip(sequence, sequence[1:]) if b - a > 1)
        print(f"saved {folder}: {len(keeper.rows)} frames ({missed} missed), {len(recorder.history)} commands, "
              f"{len(session.segments) if session else 0} parts")


if __name__ == "__main__":
    main()
