"""Deterministic plant and synthetic camera around the production node steps."""
import math
from collections import deque
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic
from ultraskelly.replay import create_controllers


class Settings(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    scenario: Literal["manual", "sweep", "loss"] = "sweep"
    yaw: float = Field(default=25, ge=-120, le=120)
    pitch: float = Field(default=10, ge=-60, le=60)
    visible: bool = True
    sensor_hz: Literal[10, 20, 50, 100] = 10
    head_gain: float = Field(default=0.05, gt=0, le=1)
    head_deadzone: int = Field(default=100, ge=0, le=240)
    waist_gain: float = Field(default=0.008, gt=0, le=0.05)
    servo_speed: float = Field(default=180, ge=1, le=720)
    waist_speed: float = Field(default=45, ge=0, le=180)
    # Optional friction: a still waist needs this much drive to break loose, and a turning
    # waist stops below waist_stall. Both 0 keep the plain linear model.
    waist_breakaway: float = Field(default=0.0, ge=0, le=1)
    waist_stall: float = Field(default=0.0, ge=0, lt=1)


def approach(value, target, amount):
    return value + max(-amount, min(amount, target - value))


class Simulation:
    dt = 0.01

    def __init__(self, settings: Settings | None = None):
        self.settings = settings or Settings()
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
        self.head.clock = self.mouth.clock = lambda: self.tick * self.dt
        self.running = False
        self.error = None
        self.configure(self.settings.model_dump())

    def configure(self, values: dict):
        # Validate the complete proposed change before mutating anything.
        settings = Settings.model_validate(self.settings.model_dump() | values)
        self.settings = settings
        self.head.params.head_gain = settings.head_gain
        self.head.params.head_deadzone = settings.head_deadzone
        self.head.params.waist.gain = settings.waist_gain

    def target(self):
        t = self.tick * self.dt
        yaw, pitch = self.settings.yaw, self.settings.pitch
        visible = self.settings.visible
        if self.settings.scenario in ("sweep", "loss"):
            yaw = 50 * math.sin(t * 0.45)
            pitch = 18 * math.sin(t * 0.7)
        if self.settings.scenario == "loss":
            visible = visible and t % 8 < 5
        return yaw, pitch, visible

    def target_position(self):
        yaw, pitch, _ = self.target()
        y, p = math.radians(yaw), math.radians(pitch)
        return [2.5 * math.cos(p) * math.sin(y), 2.10 + 2.5 * math.sin(p),
                2.5 * math.cos(p) * math.cos(y)]

    def camera(self):
        # Same yaw/tilt hierarchy and face mount as the viewer's frustum.
        yaw = math.radians(self.waist + self.pan - 90)
        pitch = math.radians(90 - self.tilt)
        right = [math.cos(yaw), 0, -math.sin(yaw)]
        up = [-math.sin(yaw)*math.sin(pitch), math.cos(pitch), -math.cos(yaw)*math.sin(pitch)]
        forward = [math.sin(yaw)*math.cos(pitch), math.sin(pitch), math.cos(yaw)*math.cos(pitch)]
        origin = [0.18*up[i] + 0.20*forward[i] + (1.92 if i == 1 else 0) for i in range(3)]
        delta = [v - o for v, o in zip(self.target_position(), origin)]
        dot = lambda axis: sum(v*a for v, a in zip(delta, axis))
        depth = dot(forward)
        nx = dot(right) / (depth * math.tan(math.radians(60))) if depth > 0 else 2
        ny = dot(up) / depth if depth > 0 else 2
        visible = self.target()[2] and depth > 0 and abs(nx) < 1 and abs(ny) < 1
        return {"x": round(320 + 320*nx) if visible else None,
                "y": round(240 - 240*ny) if visible else None,
                "visible": visible, "time": self.tick * self.dt}

    async def step(self, count=1):
        for _ in range(count):
            if self.tick % (100 // self.settings.sensor_hz) == 0:
                self.last_camera = self.camera()
                await self.bus.publish(TargetLocationTopic, TargetLocationMessage(
                    x=self.last_camera["x"], y=self.last_camera["y"],
                ))
            # Shared controllers hold fresh lock state between sensor samples.
            await self.head.step()
            await self.mouth.step()
            self.pan = approach(self.pan, self.head.pan_angle, self.settings.servo_speed * self.dt)
            self.tilt = approach(self.tilt, self.head.tilt_angle, self.settings.servo_speed * self.dt)
            self.waist = max(-100, min(100, self.waist - self.waist_velocity() * self.dt))
            # Illustrative spring-return jaw: negative drive opens, zero releases.
            self.jaw = approach(self.jaw, max(0, -self.mouth.mouth_motor.throttle) * 28, 90 * self.dt)
            self.tick += 1
            if self.tick % 5 == 0:
                self.history.append(self.sample())

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
        return {"time": round(self.tick * self.dt, 3), "pan": self.pan,
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
