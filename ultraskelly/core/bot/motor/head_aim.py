"""Calm head aiming: turn toward where the person is, allowing for the camera's delay and the head's gearing.

The head used to add `head_gain` times the pixel error to its servo angles on every camera
sample. Each sample is measured on a frame captured some 50-100 ms earlier, while the head
was still turning toward the previous ones, so those corrections piled up. And the head's
linkage turns the camera only a fraction of a degree per servo degree (`pan_ratio`,
`tilt_ratio`, measured on the robot), so the servos wound toward their end stops while the
waist, which followed the pan servo's angle, hunted back and forth (docs/audit-2026.md,
"Calm head tracking").

HeadAim works in degrees of the camera's view and converts to servo degrees only for the
commands. It keeps its goal relative to the robot's base: where the camera pointed when the
frame was captured (the head's commanded view then, less the servos' lag, plus how far the
body had turned) plus `reach` of the person's offset in that frame, converted to degrees
through the camera's pinhole geometry. Aiming short means neither a wrong scale (pixels per
degree, or the gear ratio) nor the parallax of a camera mounted ahead of the pan axis can
carry the head past the person; the frames that follow close the rest. The goal is smoothed,
and one far from the current goal (someone new, or a stray detection) must show up
`jump_samples` times in a row before the head believes it.

Each tick the head turns toward the goal less the body's turn, so it counter-rotates as soon
as the waist moves, without waiting for the camera. It turns no faster than `max_speed` (and
its servos no faster than `servo_speed`), speeding up and slowing down at `max_acceleration`,
keeps `end_margin` servo degrees clear of its end stops, and while the body is still it moves
only once its goal is more than `deadband` away, so jitter never moves it. Each servo is
commanded half its linkage's slack (`pan_slack`, `tilt_slack`) past the view in the way it
moves, so a reversal takes up the play at once. The goal's bearing from the body tells the
waist where the person is, even beyond the head's reach.
"""
import math
from collections import deque
from typing import Literal

from pydantic import Field

from ultraskelly.core.bot.base_abcs import NodeParams
from ultraskelly.core.bot.motor.waist_drive import MAX_STEP, s_curve_step

SERVO_RANGE = (0.0, 180.0)
MIDDLE = 90.0  # Servo degrees at which the head faces the way the body does
HISTORY_SECONDS = 1.0  # How far back the head remembers where the camera pointed
SLACK_TURN = 0.2  # Degrees of view the head must turn back before its servo takes up the slack the other way


class HeadAimParams(NodeParams):
    """How the head turns toward its target. Angles are degrees of the camera's view unless they say servo degrees."""

    px_per_degree: float = Field(
        default=8.1, gt=0.0,
        description="Image pixels per degree at the frame's centre: the OAK-1's 640 px over 69 degrees, a pinhole",
    )
    pan_ratio: float = Field(
        default=0.33, gt=0.0, le=1.0,
        description="Degrees the view turns per pan servo degree: 12 servo degrees moved the picture 35 px on 2026-11-06 "
                    "(0.36 at 8.1 px per degree); a little under, so the head errs toward turning short",
    )
    tilt_ratio: float = Field(
        default=0.1, gt=0.0, le=1.0,
        description="Degrees the view nods per tilt servo degree: 12 servo degrees moved the picture 9-10 px on 2026-11-06",
    )
    pan_slack: float = Field(
        default=5.0, ge=0.0, le=20.0,
        description="Servo degrees of play in the pan linkage taken up at each reversal: returning pan from 78 to 90 "
                    "undid only 17.7 of 34.7 px on 2026-11-06, about 6; a little under, as too much makes the head jump",
    )
    tilt_slack: float = Field(
        default=2.0, ge=0.0, le=20.0, description="Servo degrees of play in the tilt linkage: tilt came back within 2 px",
    )
    pan_direction: Literal[1, -1] = Field(default=1, description="+1: a target right of the aim point needs a larger pan angle")
    tilt_direction: Literal[1, -1] = Field(default=1, description="+1: a target below the aim point needs a larger tilt angle")
    end_margin: float = Field(default=5.0, ge=0.0, le=45.0, description="Servo degrees the head keeps clear of each end stop")
    pan_range: tuple[float, float] = Field(
        default=SERVO_RANGE, description="Servo degrees the pan may use: where its linkage turns the head steadily",
    )
    tilt_range: tuple[float, float] = Field(
        default=SERVO_RANGE, description="Servo degrees the tilt may use: short of a linkage's over-centre point",
    )
    smoothing: float = Field(default=0.5, gt=0.0, le=1.0, description="Share of each new estimate taken into the goal")
    deadband: float = Field(default=1.5, ge=0.0, le=10.0, description="A still head turns only when its goal is further than this")
    max_speed: float = Field(default=40.0, gt=0.0, le=600.0, description="Top turning speed of the view, degrees per second")
    servo_speed: float = Field(
        default=150.0, gt=0.0, description="Top servo speed, servo degrees per second, slow enough that the servos keep up",
    )
    max_acceleration: float = Field(default=300.0, gt=0.0, description="Degrees per second squared, speeding up and slowing down")
    jump_degrees: float = Field(default=8.0, gt=0.0, description="A goal further than this from the current one is a jump")
    jump_samples: int = Field(default=4, ge=1, description="Samples in a row a jump must show up in before the head turns to it")
    reach: float = Field(
        default=0.65, gt=0.0, le=1.0,
        description="Share of a target's offset each frame aims for: a scale error under 1/reach - 1 cannot overshoot, "
                    "and later frames close the rest",
    )
    servo_lag: float = Field(
        default=0.08, ge=0.0, le=0.5,
        description="How far the servos trail their commands: the robot's picture moved about 0.15 s after a command, less the camera's 0.055",
    )
    assumed_latency: float = Field(
        default=0.06, ge=0.0, le=0.5, description="Camera-to-target delay assumed for targets that do not say when they were captured",
    )


class HeadAim:
    """Feed `observe` each fresh target and call `update` every control tick for the servo commands.

    `body` is how far the body has turned (degrees, in pan's sense) as the caller estimates
    it; only its changes over the last second matter.
    """

    def __init__(self, params: HeadAimParams, pan: float = MIDDLE, tilt: float = MIDDLE) -> None:
        self.params = params
        self.servos = [pan, tilt]  # The servo angles last commanded
        self.view = [self._view(0, pan), self._view(1, tilt)]  # The view they turn the camera to, from straight ahead
        self.rates = [0.0, 0.0]  # How fast the view turns, degrees per second
        self.turning = [False, False]
        self.pushing = [0, 0]  # Which way each servo last pushed its linkage: +1, -1, or 0 not yet
        self.extreme = list(self.view)  # The furthest view reached since the last reversal
        self.body = 0.0  # The body's turn at the last update
        self.goal: tuple[float, float] | None = None  # Pan relative to the base (body + pan), and tilt
        self.candidate: tuple[float, float] | None = None  # A jump waiting to be believed
        self.candidate_count = 0
        self.history: deque[tuple[float, float, float, float]] = deque()  # (time, body, pan, tilt view) commanded
        self.last_update: float | None = None

    @property
    def pan(self) -> float:
        return self.servos[0]

    @property
    def tilt(self) -> float:
        return self.servos[1]

    def _ratio(self, axis: int) -> float:
        return (self.params.pan_ratio, self.params.tilt_ratio)[axis]

    def _view(self, axis: int, servo: float) -> float:
        return self._ratio(axis) * (servo - MIDDLE)

    def _range(self, axis: int) -> tuple[float, float]:
        return (self.params.pan_range, self.params.tilt_range)[axis]

    def _slack(self, axis: int) -> float:
        return (self.params.pan_slack, self.params.tilt_slack)[axis]

    def span(self, axis: int) -> tuple[float, float]:
        """The views an axis can be commanded to (0 pan, 1 tilt): its servo, pushing past the view by half
        the slack, stays end_margin clear of its end stops."""
        ratio, margin = self._ratio(axis), self.params.end_margin + self._slack(axis) / 2
        low, high = self._range(axis)
        return ratio * (low + margin - MIDDLE), ratio * (high - margin - MIDDLE)

    def place(self, pan: float, tilt: float) -> None:
        """Something else moved the head here (a motor test): hold it here and forget the old goal."""
        self.servos, self.view = [pan, tilt], [self._view(0, pan), self._view(1, tilt)]
        self.rates, self.turning = [0.0, 0.0], [False, False]
        self.pushing, self.extreme = [0, 0], list(self.view)
        self.goal, self.candidate, self.candidate_count = None, None, 0
        self.history.clear()

    def bearing(self) -> float | None:
        """How far the target is from the body's forward direction, in pan's sense, or None without a goal."""
        return None if self.goal is None else self.goal[0] - self.body

    def _recorded(self, when: float) -> tuple[float, float, float]:
        """The body's turn and the commanded pan and tilt views at time `when`, from the recent commands."""
        chosen = None
        for entry in self.history:
            if entry[0] > when:
                break
            chosen = entry
        if chosen is None:
            chosen = self.history[0] if self.history else (when, self.body, self.view[0], self.view[1])
        return chosen[1], chosen[2], chosen[3]

    def pointing_at(self, captured: float) -> tuple[float, float]:
        """Where the camera pointed when a frame was captured (body + pan, and tilt): as commanded a servo lag earlier.

        While the body turns, that reads the body's turn a servo lag early too, which puts the estimate
        behind the person by the body's turn over the lag; the head's counter-rotation reaches the
        camera that same lag late, ahead of the person, and the two cancel."""
        body, pan, tilt = self._recorded(captured - self.params.servo_lag)
        return body + pan, tilt

    def observe(self, *, now: float, error_x: float, error_y: float, captured: float | None = None) -> None:
        """One detection: the target's offset in pixels from the aim point, in a frame captured at `captured`."""
        p = self.params
        pan_then, tilt_then = self.pointing_at(captured if captured is not None else now - p.assumed_latency)
        # Aim short of the target: an offset measured from afar carries the scale's error and the
        # parallax of a camera mounted ahead of the pan axis, both in proportion to the offset.
        focal = p.px_per_degree * 180.0 / math.pi  # The pinhole's focal length in pixels
        low, high = self.span(1)
        estimate = (
            pan_then + p.reach * p.pan_direction * math.degrees(math.atan(error_x / focal)),
            min(max(tilt_then + p.reach * p.tilt_direction * math.degrees(math.atan(error_y / focal)), low), high),
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

    def _command(self, axis: int) -> None:
        """The servo angle that puts the linkage at this axis's view: past it by half the slack, the way it moves.

        A linkage with play follows its servo only once the servo pushes it from one side. After a
        reversal the servo first crosses the play while the head stands still; pushing half the slack
        ahead of the view takes the play up at once. A turn back must exceed SLACK_TURN to count, so
        the servo never chatters across the play."""
        moved = self.view[axis] - self.extreme[axis]
        if (self.pushing[axis] >= 0 and moved < -SLACK_TURN) or (self.pushing[axis] <= 0 and moved > SLACK_TURN):
            self.pushing[axis], self.extreme[axis] = (1 if moved > 0 else -1), self.view[axis]
        elif self.pushing[axis] * moved > 0:
            self.extreme[axis] = self.view[axis]  # Still going the same way
        servo = MIDDLE + self.view[axis] / self._ratio(axis) + self.pushing[axis] * self._slack(axis) / 2
        low, high = self._range(axis)
        self.servos[axis] = min(max(servo, low), high)

    def update(self, *, now: float, body: float = 0.0) -> tuple[float, float]:
        """This tick's pan and tilt servo commands, for a body turned `body` degrees."""
        p = self.params
        dt = 0.0 if self.last_update is None else min(max(now - self.last_update, 0.0), MAX_STEP)
        self.last_update = now
        body_turning = body != self.body
        self.body = body
        if self.goal is not None:
            for axis, aim in enumerate((self.goal[0] - body, self.goal[1])):
                low, high = self.span(axis)
                wanted = min(max(aim, low), high)
                if not self.turning[axis] and (abs(wanted - self.view[axis]) > p.deadband or (axis == 0 and body_turning)):
                    self.turning[axis] = True  # Counter-rotate against the body at once
                if self.turning[axis]:
                    speed = min(p.max_speed, p.servo_speed * self._ratio(axis))
                    self.view[axis], self.rates[axis] = s_curve_step(
                        self.view[axis], self.rates[axis], wanted, up=speed, down=speed, jerk=p.max_acceleration, dt=dt,
                    )
                    self._command(axis)
                    if self.view[axis] == wanted and self.rates[axis] == 0.0:
                        self.turning[axis] = False
        self.history.append((now, body, self.view[0], self.view[1]))
        while self.history and self.history[0][0] < now - HISTORY_SECONDS:
            self.history.popleft()
        return self.pan, self.tilt
