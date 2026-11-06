"""Deterministic plant and synthetic camera around the production node steps.

The camera behaves like the robot's: the OAK's field of view as a pinhole, 30 samples a
second, each delivered after the pose pipeline's delay with the time it was captured,
optional pixel noise and dropped detections, and any number of people, of whom it reports
the one the pose node would follow. Servos lag their commands as well as being limited in
speed, and either servo can be set to turn the camera the wrong way, to test polarity.
"""
import math
import random
from collections import deque
from dataclasses import dataclass
from typing import Callable, Literal

from pydantic import BaseModel, ConfigDict, Field

from ultraskelly.core.bot.sensory.pose_common import choose_person
from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic
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
    head_gain: float = Field(default=0.05, gt=0, le=1)
    head_deadzone: int = Field(default=100, ge=0, le=240)
    waist_gain: float = Field(default=0.008, gt=0, le=0.05)
    servo_speed: float = Field(default=180, ge=1, le=720)
    waist_speed: float = Field(default=45, ge=0, le=180)
    # Optional friction: a still waist needs this much drive to break loose, and a turning
    # waist stops below waist_stall. Both 0 keep the plain linear model.
    waist_breakaway: float = Field(default=0.0, ge=0, le=1)
    waist_stall: float = Field(default=0.0, ge=0, lt=1)
    # The camera and servos as on the robot. The OAK-1 sees 69 degrees across its 4:3
    # sensor; frames reach the controller 50-70 ms after capture (measured on the Pi,
    # 2026-11-02); the servo lag is an assumption until measured.
    camera_hfov: float = Field(default=69.0, gt=10, lt=170)
    camera_latency: float = Field(default=0.055, ge=0, le=0.5)
    servo_lag: float = Field(default=0.04, ge=0, le=0.5)
    pan_sign: Literal[1, -1] = 1  # -1: a larger pan command turns the camera the other way
    tilt_sign: Literal[1, -1] = 1
    noise_px: float = Field(default=0.0, ge=0, le=50)  # Detection jitter, standard deviation in pixels
    dropout: float = Field(default=0.0, ge=0, lt=1)  # Share of samples in which the detector finds nobody
    seed: int = 0


@dataclass(frozen=True)
class Person:
    """Someone in front of the robot: bearing from its base (degrees) and detector score."""
    yaw: float
    pitch: float
    score: float = 0.9


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
        self.pan = self.tilt = 90.0
        self.waist = self.jaw = 0.0
        self.waist_moving = False
        self.last_camera = {"x": None, "y": None, "visible": False, "time": None}
        self.history = deque(maxlen=600)
        self.pending: deque[tuple[float, TargetLocationMessage]] = deque()  # Samples still in the pose pipeline
        self.next_sample_at = 0.0
        self.rng = random.Random(self.settings.seed)
        self.head.clock = self.mouth.clock = self.now
        self.running = False
        self.error = None
        self.configure(self.settings.model_dump())

    def now(self) -> float:
        return self.tick * self.dt

    def configure(self, values: dict):
        # Validate the complete proposed change before mutating anything.
        settings = Settings.model_validate(self.settings.model_dump() | values)
        self.settings = settings
        self.head.params.head_gain = settings.head_gain
        self.head.params.head_deadzone = settings.head_deadzone
        self.head.params.waist.gain = settings.waist_gain

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
        y, p = math.radians(person.yaw), math.radians(person.pitch)
        return [2.5 * math.cos(p) * math.sin(y), 2.10 + 2.5 * math.sin(p), 2.5 * math.cos(p) * math.cos(y)]

    def target_position(self):
        people = self.present()
        if not people:
            yaw, pitch, _ = self.target()
            return self.position(Person(yaw, pitch))
        return self.position(people[0])

    def camera_pose(self) -> tuple[float, float]:
        """The camera's bearing from the base (yaw, pitch in degrees), from the servos' actual angles."""
        return (self.waist + self.settings.pan_sign * (self.pan - 90),
                self.settings.tilt_sign * (90 - self.tilt))

    def project(self, person: Person) -> tuple[float, float] | None:
        """Where a person appears in the frame, in pixels, or None outside it.
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
        x, y = AIM[0] + focal * dot(right) / depth, AIM[1] - focal * dot(up) / depth
        return (x, y) if 0 <= x < WIDTH and 0 <= y < HEIGHT else None

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

    async def step(self, count=1):
        for _ in range(count):
            now = self.now()
            if now + 1e-9 >= self.next_sample_at:  # Capture a frame; its result arrives after the pipeline's delay
                self.next_sample_at += 1 / self.settings.sensor_hz
                self.last_camera = self.camera()
                self.pending.append((now + self.settings.camera_latency, TargetLocationMessage(
                    x=self.last_camera["x"], y=self.last_camera["y"], captured=now,
                )))
            while self.pending and self.pending[0][0] <= now + 1e-9:
                await self.bus.publish(TargetLocationTopic, self.pending.popleft()[1])
            # Shared controllers hold fresh lock state between sensor samples.
            await self.head.step()
            await self.mouth.step()
            self.pan = self.servo(self.pan, self.head.pan_angle)
            self.tilt = self.servo(self.tilt, self.head.tilt_angle)
            self.waist = max(-100, min(100, self.waist - self.waist_velocity() * self.dt))
            # Illustrative spring-return jaw: negative drive opens, zero releases.
            self.jaw = approach(self.jaw, max(0, -self.mouth.mouth_motor.throttle) * 28, 90 * self.dt)
            self.tick += 1
            if self.tick % 5 == 0:
                self.history.append(self.sample())

    def servo(self, angle: float, command: float) -> float:
        """A servo's next angle: it closes on its command with a first-order lag, no faster than its top speed."""
        lag = self.settings.servo_lag
        wanted = (command - angle) * min(1.0, self.dt / lag) if lag else command - angle
        return approach(angle, angle + wanted, self.settings.servo_speed * self.dt)

    def waist_drive(self):
        # A released (coasting) waist motor has no drive; the plant has no inertia to carry it on.
        return self.head.waist_motor.throttle or 0.0

    def waist_velocity(self):
        """Degrees per second for positive drive (negative drive increases yaw), with optional friction."""
        drive, s = self.waist_drive(), self.settings
        self.waist_moving = abs(drive) > s.waist_stall if self.waist_moving else abs(drive) >= max(s.waist_breakaway, 1e-9)
        if not self.waist_moving:
            return 0.0
        return math.copysign((abs(drive) - s.waist_stall) / (1 - s.waist_stall), drive) * s.waist_speed

    def sample(self):
        return {"time": round(self.now(), 3), "pan": self.pan,
                "pan_command": self.head.pan_angle, "tilt": self.tilt,
                "tilt_command": self.head.tilt_angle, "waist": self.waist, "jaw": self.jaw,
                "waist_throttle": self.waist_drive(),
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
