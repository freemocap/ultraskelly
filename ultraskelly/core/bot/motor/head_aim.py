"""Calm head aiming: turn toward where the person is, allowing for the camera's delay.

The head used to add `head_gain` times the pixel error to its servo angles on every
camera sample. Each sample is measured on a frame captured some 50-100 ms earlier, while
the head was still turning toward the previous ones, so those corrections piled up and
the head swung past the person and back (see docs/audit-2026.md, "Calm head tracking").

HeadAim instead works out where the person is. It keeps its goal relative to the robot's
base: the camera's direction when the frame was captured (the head's commanded angles
then, less the servos' lag, plus how far the body had turned) plus `reach` of the person's
offset in that frame, converted to degrees through the camera's pinhole geometry. Aiming
85% of the way means neither a pixels-per-degree scale up to 17% off nor the parallax of
a camera mounted ahead of the pan axis can carry the head past the person; the frames
that follow close the rest. The goal is smoothed, and one far from the current goal
(someone new, or a stray detection) must show up `jump_samples` times in a row before the
head believes it.

Each tick the head turns toward the goal less the body's turn, so it counter-rotates as
soon as the waist moves, without waiting for the camera. It turns no faster than
`max_speed`, speeding up and slowing down at `max_acceleration`, and while the body is
still it moves only once its goal is more than `deadband` away, so jitter never moves it.
"""
import math
from collections import deque
from typing import Literal

from pydantic import Field

from ultraskelly.core.bot.base_abcs import NodeParams
from ultraskelly.core.bot.motor.waist_drive import MAX_STEP, s_curve_step

SERVO_RANGE = (0.0, 180.0)
HISTORY_SECONDS = 1.0  # How far back the head remembers where the camera pointed


class HeadAimParams(NodeParams):
    """How the head turns toward its target. Angles in degrees, times in seconds."""

    px_per_degree: float = Field(
        default=8.1, gt=0.0,
        description="Image pixels per degree at the frame's centre: the OAK-1's 640 px over 69 degrees, a pinhole",
    )
    pan_direction: Literal[1, -1] = Field(default=1, description="+1: a target right of the aim point needs a larger pan angle")
    tilt_direction: Literal[1, -1] = Field(default=1, description="+1: a target below the aim point needs a larger tilt angle")
    smoothing: float = Field(default=0.5, gt=0.0, le=1.0, description="Share of each new estimate taken into the goal")
    deadband: float = Field(default=1.5, ge=0.0, le=10.0, description="A still head turns only when its goal is further than this")
    max_speed: float = Field(default=40.0, gt=0.0, le=600.0, description="Top turning speed, degrees per second")
    max_acceleration: float = Field(default=300.0, gt=0.0, description="Degrees per second squared, speeding up and slowing down")
    jump_degrees: float = Field(default=8.0, gt=0.0, description="A goal further than this from the current one is a jump")
    jump_samples: int = Field(default=4, ge=1, description="Samples in a row a jump must show up in before the head turns to it")
    reach: float = Field(
        default=0.85, gt=0.0, le=1.0,
        description="Share of a target's offset each frame aims for: a scale error under 1/reach - 1 cannot overshoot, "
                    "and later frames close the rest",
    )
    servo_lag: float = Field(default=0.04, ge=0.0, le=0.5, description="How far the servos trail their commands")
    assumed_latency: float = Field(
        default=0.06, ge=0.0, le=0.5, description="Camera-to-target delay assumed for targets that do not say when they were captured",
    )


def _clip(angle: float) -> float:
    return min(max(angle, SERVO_RANGE[0]), SERVO_RANGE[1])


class HeadAim:
    """Feed `observe` each fresh target and call `update` every control tick for the servo commands.

    `body` is how far the body has turned (degrees, in pan's sense) as the caller estimates
    it; only its changes over the last second matter.
    """

    def __init__(self, params: HeadAimParams, pan: float = 90.0, tilt: float = 90.0) -> None:
        self.params = params
        self.angles = [pan, tilt]  # Commanded pan and tilt
        self.rates = [0.0, 0.0]  # Their rates of change, degrees per second
        self.turning = [False, False]
        self.body = 0.0  # The body's turn at the last update
        self.goal: tuple[float, float] | None = None  # Pan relative to the base (body + pan), and tilt
        self.candidate: tuple[float, float] | None = None  # A jump waiting to be believed
        self.candidate_count = 0
        self.history: deque[tuple[float, float, float]] = deque()  # (time, body + pan, tilt) commanded
        self.last_update: float | None = None

    @property
    def pan(self) -> float:
        return self.angles[0]

    @property
    def tilt(self) -> float:
        return self.angles[1]

    def place(self, pan: float, tilt: float) -> None:
        """Something else moved the head here (a motor test): hold it here and forget the old goal."""
        self.angles, self.rates, self.turning = [pan, tilt], [0.0, 0.0], [False, False]
        self.goal, self.candidate, self.candidate_count = None, None, 0
        self.history.clear()

    def pointing_at(self, when: float) -> tuple[float, float]:
        """Where the camera pointed at time `when` (body + pan, and tilt), from the recent commands."""
        chosen = None
        for time, pan, tilt in self.history:
            if time > when:
                break
            chosen = (pan, tilt)
        if chosen is None:
            chosen = (self.history[0][1], self.history[0][2]) if self.history else (self.body + self.pan, self.tilt)
        return chosen

    def observe(self, *, now: float, error_x: float, error_y: float, captured: float | None = None) -> None:
        """One detection: the target's offset in pixels from the aim point, in a frame captured at `captured`."""
        p = self.params
        seen = (captured if captured is not None else now - p.assumed_latency) - p.servo_lag
        pan_then, tilt_then = self.pointing_at(seen)
        # Aim short of the target: an offset measured from afar carries the scale's error and the
        # parallax of a camera mounted ahead of the pan axis, both in proportion to the offset.
        focal = p.px_per_degree * 180.0 / math.pi  # The pinhole's focal length in pixels
        estimate = (
            pan_then + p.reach * p.pan_direction * math.degrees(math.atan(error_x / focal)),
            _clip(tilt_then + p.reach * p.tilt_direction * math.degrees(math.atan(error_y / focal))),
        )
        if self.goal is not None and math.dist(estimate, self.goal) <= p.jump_degrees:
            self.candidate, self.candidate_count = None, 0
            self.goal = tuple(goal + p.smoothing * (new - goal) for goal, new in zip(self.goal, estimate))
            return
        # Someone new or somewhere new: believe it once it has held for jump_samples samples in a row.
        if self.candidate is not None and math.dist(estimate, self.candidate) <= p.jump_degrees:
            self.candidate_count += 1
            self.candidate = tuple((old + new) / 2 for old, new in zip(self.candidate, estimate))
        else:
            self.candidate, self.candidate_count = estimate, 1
        if self.candidate_count >= p.jump_samples:
            self.goal, self.candidate, self.candidate_count = self.candidate, None, 0

    def update(self, *, now: float, body: float = 0.0) -> tuple[float, float]:
        """This tick's pan and tilt commands, for a body turned `body` degrees."""
        p = self.params
        dt = 0.0 if self.last_update is None else min(max(now - self.last_update, 0.0), MAX_STEP)
        self.last_update = now
        body_turning = body != self.body
        self.body = body
        if self.goal is not None:
            wanted = (_clip(self.goal[0] - body), self.goal[1])
            for axis in (0, 1):
                if not self.turning[axis] and (abs(wanted[axis] - self.angles[axis]) > p.deadband
                                               or (axis == 0 and body_turning)):
                    self.turning[axis] = True  # Counter-rotate against the body at once
                if self.turning[axis]:
                    self.angles[axis], self.rates[axis] = s_curve_step(
                        self.angles[axis], self.rates[axis], wanted[axis],
                        up=p.max_speed, down=p.max_speed, jerk=p.max_acceleration, dt=dt,
                    )
                    if self.angles[axis] == wanted[axis] and self.rates[axis] == 0.0:
                        self.turning[axis] = False
        self.history.append((now, body + self.pan, self.tilt))
        while self.history and self.history[0][0] < now - HISTORY_SECONDS:
            self.history.popleft()
        return self.pan, self.tilt
