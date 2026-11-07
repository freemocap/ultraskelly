"""Deterministic plant and synthetic camera around the production node steps.

The camera behaves like the robot's: the OAK's field of view as a pinhole, 30 samples a
second, each delivered after the pose pipeline's delay with the time it was captured,
optional pixel noise and dropped detections, and any number of people, of whom it reports
the one the pose node would follow. Servos lag their commands as well as being limited in
speed, and either servo can be set to turn the camera the wrong way, to test polarity. Faces
carry a tilt: the roll servo turns the whole picture, and each sample also reports every
visible face's eyes, nose and mouth as the OAK's face detector would.
"""
import math
import random
from collections import deque
from dataclasses import dataclass
from typing import Callable, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from ultraskelly.core.bot.sensory.pose_common import choose_person
from ultraskelly.simulator import plant
from ultraskelly.simulator.plant import Axis, AxisModel, Waist, WaistModel, play
from ultraskelly.core.pubsub.bot_topics import (
    FaceObservation,
    OakResultsMessage,
    OakResultsTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.replay import create_controllers

WIDTH, HEIGHT = 640, 480  # The OAK frame the pose detector sees
AIM = (WIDTH / 2, HEIGHT / 2)  # Where the head steers its target, and where the pose node looks for it


class Settings(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    scenario: Literal["manual", "sweep", "loss"] = "sweep"
    yaw: float = Field(default=25, ge=-120, le=120)
    pitch: float = Field(default=10, ge=-60, le=60)
    visible: bool = True
    sensor_hz: Literal[10, 20, 30, 50, 100] = 30
    # Pan's top turning speed, degrees of view per second; None keeps the controller's own (HeadAimParams.pan_speed)
    head_speed: float | None = Field(default=None, gt=0, le=150)
    head_deadzone: int = Field(default=100, ge=0, le=240)
    # The head's settled pan offset that starts a waist move, degrees of view (WaistDriveParams.start_offset)
    waist_start: float = Field(default=10.0, ge=0, le=60)
    # The waist as measured on 2026-11-06 (plant.WAIST): it needs more than 0.6 drive to break loose
    # and turns 50 deg/s at full negative drive but 21.5 at full positive drive.
    waist_speed: float = Field(default=plant.WAIST.speed_negative, ge=0, le=180)  # At full negative drive
    waist_speed_positive: float = Field(default=plant.WAIST.speed_positive, ge=0, le=180)
    waist_breakaway: float = Field(default=plant.WAIST.breakaway, ge=0, le=1)
    waist_stall: float = Field(default=plant.WAIST.stall, ge=0, lt=1)
    waist_time_constant: float = Field(default=plant.WAIST.time_constant, gt=0, le=2)  # Seconds to reach its speed
    waist_brake_time: float = Field(default=plant.WAIST.brake_time, gt=0, le=2)  # Seconds to stop, braked
    waist_coast_time: float = Field(default=plant.WAIST.coast_time, gt=0, le=5)  # And released to coast
    # The camera as on the robot: the OAK-1 sees 69 degrees across its 4:3 sensor, and pose results
    # reach the controller about 0.06 s after capture (2026-11-06).
    camera_hfov: float = Field(default=69.0, gt=10, lt=170)
    camera_latency: float = Field(default=0.06, ge=0, le=0.5)
    # The head's servos as measured on 2026-11-06 (plant.PAN, TILT, ROLL; tests/test_plant_fidelity.py):
    # each turns the view by its gain per servo degree once it has crossed its slack, a dead time after
    # the command, no faster than its slew, through the head's own ringing mode. These settings vary
    # the measured models, for robustness tests: gains (`*_ratio`), slacks (`*_backlash`), every dead
    # time (`delay_scale`), and pan's mode, the one that rings (1.9 Hz, 22-32 % overshoot).
    pan_sign: Literal[1, -1] = 1  # -1: a larger pan command turns the camera the other way
    tilt_sign: Literal[1, -1] = 1
    pan_ratio: float = Field(default=plant.PAN.gain, gt=0, le=1)
    tilt_ratio: float = Field(default=plant.TILT.gain, gt=0, le=1)
    pan_backlash: float = Field(default=plant.PAN.slack, ge=0, le=30)  # Servo degrees
    tilt_backlash: float = Field(default=plant.TILT.slack, ge=0, le=30)
    pan_frequency: float = Field(default=plant.PAN.frequency, gt=0, le=200)  # Rad/s
    pan_damping: float = Field(default=plant.PAN.damping, gt=0, le=2)  # Turning toward larger views
    pan_damping_down: float = Field(default=plant.PAN.damping_down, gt=0, le=2)  # And toward smaller ones
    delay_scale: float = Field(default=1.0, ge=0, le=5)  # Multiplies every servo's dead time
    # The roll turns the picture's content clockwise for a larger angle (roll_sign -1; +1 turns it
    # anticlockwise), and nods the camera down roll_nod degrees per servo degree as it does (the OAK
    # sits off the roll's axis): 25 servo degrees moved the picture 27-29 px (2026-11-06).
    roll_ratio: float = Field(default=plant.ROLL.gain, gt=0, le=1)
    roll_sign: Literal[1, -1] = -1
    roll_backlash: float = Field(default=plant.ROLL.slack, ge=0, le=40)  # Servo degrees
    roll_nod: float = Field(default=0.18, ge=-1, le=1)
    noise_px: float = Field(default=0.0, ge=0, le=50)  # Detection jitter, standard deviation in pixels
    landmark_noise_px: float = Field(default=0.0, ge=0, le=20)  # The face detector's own jitter on eyes, nose, mouth
    dropout: float = Field(default=0.0, ge=0, lt=1)  # Share of samples in which the detector finds nobody
    seed: int = 0


@dataclass(frozen=True)
class Person:
    """Someone in front of the robot: bearing from its base (degrees) and detector score."""
    yaw: float
    pitch: float
    score: float = 0.9
    roll: float = 0.0  # The face's tilt as an upright camera would see it: degrees clockwise on screen
    distance: float = 2.5  # Metres from the robot's base


def approach(value, target, amount):
    return value + max(-amount, min(amount, target - value))


class Simulation:
    dt = 0.01

    def __init__(self, settings: Settings | None = None, people: Callable[[float], list[Person]] | None = None):
        self.settings = settings or Settings()
        self.people = people  # Who is in front of the robot at each time; None uses the built-in scenarios
        self.running = False
        self.error = None
        self.reset()

    def reset(self):
        if hasattr(self, "bus"):
            self.close()
        self.bus, self.head, self.mouth = create_controllers()
        self.tick = 0
        self.axes = self.build_axes()
        self.waist_plant = self.build_waist()
        self.waist = self.jaw = 0.0
        self.last_camera = {"x": None, "y": None, "visible": False, "time": None}
        self.history = deque(maxlen=600)
        self.pending: deque[tuple[float, type, object]] = deque()  # (due, topic, message) still in the pipelines
        self.next_sample_at = 0.0
        self.rng = random.Random(self.settings.seed)
        self.face_rng = random.Random(self.settings.seed + 1)  # The face detector's own noise and misses
        self.head.clock = self.mouth.clock = self.now
        self.running = False
        self.error = None
        self.configure(self.settings.model_dump())

    def now(self) -> float:
        return self.tick * self.dt

    def build_axes(self) -> dict[str, Axis]:
        """The head's servos, the measured models (plant.py) as the settings vary them."""
        s, scale = self.settings, self.settings.delay_scale
        def varied(model: AxisModel, **changes) -> Axis:
            return Axis(AxisModel(**{**model.__dict__, "dead_time": model.dead_time * scale, **changes}))
        return {
            "pan": varied(plant.PAN, gain=s.pan_ratio, slack=s.pan_backlash, frequency=s.pan_frequency,
                          damping=s.pan_damping, damping_down=s.pan_damping_down),
            "tilt": varied(plant.TILT, gain=s.tilt_ratio, slack=s.tilt_backlash),
            "roll": varied(plant.ROLL, gain=s.roll_ratio, slack=s.roll_backlash),
        }

    def build_waist(self) -> Waist:
        s = self.settings
        return Waist(WaistModel(breakaway=s.waist_breakaway, stall=s.waist_stall, speed_negative=s.waist_speed,
                                speed_positive=s.waist_speed_positive, time_constant=s.waist_time_constant,
                                brake_time=s.waist_brake_time, coast_time=s.waist_coast_time))

    @property
    def pan(self) -> float:
        return self.axes["pan"].servo  # The servos' actual angles

    @property
    def tilt(self) -> float:
        return self.axes["tilt"].servo

    @property
    def roll(self) -> float:
        return self.axes["roll"].servo

    def configure(self, values: dict):
        # Validate the complete proposed change before mutating anything.
        settings = Settings.model_validate(self.settings.model_dump() | values)
        self.settings = settings
        if settings.head_speed is not None:
            self.head.params.aim.pan_speed = settings.head_speed
        self.head.params.head_deadzone = settings.head_deadzone
        self.head.params.waist.start_offset = settings.waist_start
        if hasattr(self, "axes"):  # Plant settings take effect from here on, the axes where they stand
            for name, axis in self.build_axes().items():
                self.axes[name].model = axis.model
            self.waist_plant.model = self.build_waist().model

    def target(self):
        t = self.now()
        yaw, pitch = self.settings.yaw, self.settings.pitch
        visible = self.settings.visible
        if self.settings.scenario in ("sweep", "loss"):
            yaw = 50 * math.sin(t * 0.45)
            pitch = 18 * math.sin(t * 0.7)
        if self.settings.scenario == "loss":
            visible = visible and t % 8 < 5
        return yaw, pitch, visible

    def present(self) -> list[Person]:
        """Everyone in front of the robot now."""
        if self.people is not None:
            return self.people(self.now())
        yaw, pitch, visible = self.target()
        return [Person(yaw, pitch)] if visible else []

    @staticmethod
    def position(person: Person) -> list[float]:
        y, p, r = math.radians(person.yaw), math.radians(person.pitch), person.distance
        return [r * math.cos(p) * math.sin(y), 2.10 + r * math.sin(p), r * math.cos(p) * math.cos(y)]

    def target_position(self):
        people = self.present()
        if not people:
            yaw, pitch, _ = self.target()
            return self.position(Person(yaw, pitch))
        return self.position(people[0])

    def head_turn(self) -> tuple[float, float]:
        """How far the head has turned from the body (yaw) and nodded up (pitch), degrees: the plant's views."""
        s = self.settings
        return s.pan_sign * self.axes["pan"].view, -s.tilt_sign * self.axes["tilt"].view

    def picture_roll(self) -> float:
        """How far the roll servo has turned the picture's content anticlockwise, degrees."""
        return self.settings.roll_sign * self.axes["roll"].view

    def camera_pose(self) -> tuple[float, float]:
        """The camera's bearing from the base (yaw, pitch in degrees), from the plant's views."""
        yaw, pitch = self.head_turn()
        return self.waist + yaw, pitch - self.settings.roll_nod * (self.axes["roll"].linkage - 90)

    def project(self, person: Person) -> tuple[float, float] | None:
        """Where a person appears in the frame, in pixels, or None outside it."""
        seen = self.sighting(person)
        return None if seen is None else seen[:2]

    def sighting(self, person: Person) -> tuple[float, float, float] | None:
        """Where a person appears in the frame (pixels) and pixels per metre there, or None outside it.
        Same yaw/tilt hierarchy and face mount as the viewer's frustum."""
        camera_yaw, camera_pitch = self.camera_pose()
        yaw, pitch = math.radians(camera_yaw), math.radians(camera_pitch)
        right = [math.cos(yaw), 0, -math.sin(yaw)]
        up = [-math.sin(yaw)*math.sin(pitch), math.cos(pitch), -math.cos(yaw)*math.sin(pitch)]
        forward = [math.sin(yaw)*math.cos(pitch), math.sin(pitch), math.cos(yaw)*math.cos(pitch)]
        origin = [0.18*up[i] + 0.20*forward[i] + (1.92 if i == 1 else 0) for i in range(3)]
        delta = [v - o for v, o in zip(self.position(person), origin)]
        dot = lambda axis: sum(v*a for v, a in zip(delta, axis))
        depth = dot(forward)
        if depth <= 0:
            return None
        focal = (WIDTH / 2) / math.tan(math.radians(self.settings.camera_hfov / 2))  # Pixels; square pixels
        across, down = focal * dot(right) / depth, -focal * dot(up) / depth
        turn = math.radians(self.picture_roll())  # A rolled head turns the whole picture about its middle
        across, down = (across * math.cos(turn) + down * math.sin(turn), -across * math.sin(turn) + down * math.cos(turn))
        x, y = AIM[0] + across, AIM[1] + down
        return (x, y, focal / depth) if 0 <= x < WIDTH and 0 <= y < HEIGHT else None

    def camera(self):
        """One detector result for the current view: the person the pose node would follow, with noise and dropouts."""
        people = self.present()
        points = [self.project(person) for person in people]
        chosen = choose_person(points, [person.score for person in people], AIM) if people else None
        if chosen is not None and self.settings.dropout and self.rng.random() < self.settings.dropout:
            chosen = None
        x = y = None
        if chosen is not None:
            x, y = points[chosen]
            if self.settings.noise_px:
                x, y = x + self.rng.gauss(0, self.settings.noise_px), y + self.rng.gauss(0, self.settings.noise_px)
            x, y = round(min(max(x, 0), WIDTH - 1)), round(min(max(y, 0), HEIGHT - 1))
        return {"x": x, "y": y, "visible": chosen is not None, "time": self.now()}

    def faces(self) -> list[FaceObservation]:
        """The faces the OAK's face detector would find now: each visible person's eyes, nose and mouth around
        where the pose detector puts them, tilted in the picture by the person's own tilt less the head's roll."""
        s = self.settings
        if s.dropout and self.face_rng.random() < s.dropout:
            return []
        found = []
        for person in self.present():
            seen = self.sighting(person)
            if seen is None:
                continue
            x, y, scale = seen  # Pixels per metre at the person: an adult's eyes are about 63 mm apart
            tilt = math.radians(person.roll - self.picture_roll())
            along, down = np.array([math.cos(tilt), math.sin(tilt)]), np.array([-math.sin(tilt), math.cos(tilt)])
            nose = np.array([x, y])
            eyes = nose - 0.025 * scale * down
            landmarks = np.array([eyes - 0.0315 * scale * along, eyes + 0.0315 * scale * along,  # Right eye on the left
                                  nose, nose + 0.03 * scale * (down - along), nose + 0.03 * scale * (down + along)])
            if s.landmark_noise_px:
                landmarks = landmarks + np.array([[self.face_rng.gauss(0, s.landmark_noise_px) for _ in range(2)] for _ in range(5)])
            half_width, height = 0.08 * scale, 0.11 * scale
            found.append(FaceObservation(box=(x - half_width, y - height, x + half_width, y + height),
                                         score=person.score, landmarks=landmarks))
        return found

    async def step(self, count=1):
        for _ in range(count):
            now = self.now()
            if now + 1e-9 >= self.next_sample_at:  # Capture a frame; its result arrives after the pipeline's delay
                self.next_sample_at += 1 / self.settings.sensor_hz
                self.last_camera = self.camera()
                due = now + self.settings.camera_latency
                self.pending.append((due, TargetLocationTopic, TargetLocationMessage(
                    x=self.last_camera["x"], y=self.last_camera["y"], captured=now,
                )))
                self.pending.append((due, OakResultsTopic, OakResultsMessage(
                    faces=self.faces(), image_size=(WIDTH, HEIGHT), timestamp=now,
                )))
            while self.pending and self.pending[0][0] <= now + 1e-9:
                _, topic, message = self.pending.popleft()
                await self.bus.publish(topic, message)
            # Shared controllers hold fresh lock state between sensor samples.
            await self.head.step()
            await self.mouth.step()
            for name, command in (("pan", self.head.pan_angle), ("tilt", self.head.tilt_angle), ("roll", self.head.roll_angle)):
                self.axes[name].command(now, command)
                self.axes[name].advance(now + self.dt, self.dt)
            self.waist = max(-100.0, min(100.0, self.waist_plant.advance(self.waist_drive(), self.dt)))
            # Illustrative spring-return jaw: negative drive opens, zero releases.
            self.jaw = approach(self.jaw, max(0, -self.mouth.mouth_motor.throttle) * 28, 90 * self.dt)
            self.tick += 1
            if self.tick % 5 == 0:
                self.history.append(self.sample())

    def waist_drive(self):
        """The waist motor's command: a throttle, 0.0 braked, or None released to coast."""
        return self.head.waist_motor.throttle

    def sample(self):
        head_yaw, head_pitch = self.head_turn()
        return {"time": round(self.now(), 3), "pan": self.pan,
                "head_yaw": head_yaw, "head_pitch": head_pitch,
                "pan_command": self.head.pan_angle, "tilt": self.tilt,
                "roll": self.roll, "roll_command": self.head.roll_angle, "picture_roll": self.picture_roll(),
                "tilt_command": self.head.tilt_angle, "waist": self.waist, "jaw": self.jaw,
                "waist_throttle": self.waist_drive() or 0.0,
                "mouth_throttle": self.mouth.mouth_motor.throttle,
                "mouth_state": self.mouth.mouth_state.value,
                "jaw_energized": self.mouth.mouth_motor.throttle < 0}

    def snapshot(self):
        yaw, pitch, visible = self.target()
        return {"tick": self.tick, "running": self.running, "error": self.error,
                "settings": self.settings.model_dump(), "state": self.sample(),
                "target": {"yaw": yaw, "pitch": pitch, "enabled": visible, "position": self.target_position()},
                "camera": self.last_camera, "waist_mode": self.head.waist_mode.value,
                "mouth_state": self.mouth.mouth_state.value,
                "history": list(self.history)}

    def close(self):
        self.running = False
        self.head.waist_motor.throttle = 0
        self.mouth.mouth_motor.throttle = 0
        self.bus.close()
