"""Level the followed face in the camera's picture with the head's roll servo.

The OAK's face detector gives each face's eyes. The line between them tilts in the picture by
the face's own tilt less the camera's roll. RollLevel works as HeadAim does (head_aim.py): from
each usable face it estimates the camera roll that would level the face (the roll when the frame
was captured, as commanded a servo lag earlier, plus the eye line's angle), aims `reach` of the
way there, smooths that goal, and rolls toward it no faster than `max_speed` (and its servo no
faster than `servo_speed`), only once it is more than `deadband` away, and never more than `span`
servo degrees from level. It commands the servo half the linkage's `slack` past the roll in the
way it turns, so a reversal takes the play up at once. Without a usable face for `lost_after`
seconds it eases back to level.

Angles are the picture's rotation in degrees, clockwise on screen (image y points down). On
2026-11-06 scripts/check_motor_directions.py found a larger roll angle turning the picture's
content clockwise, about 0.06 degrees per servo degree (fitted to the recorded steps), through about
12 servo degrees of slack: within its span the roll turns the picture only about 1.3 degrees either way.
"""
import math
from collections import deque
from typing import Literal, Sequence

from pydantic import Field

from ultraskelly.core.bot.base_abcs import NodeParams
from ultraskelly.core.bot.motor.waist_drive import MAX_STEP, s_curve_step
from ultraskelly.core.pubsub.bot_topics import FaceObservation

LEVEL = 90.0  # Roll servo degrees at which the head stands upright
HISTORY_SECONDS = 1.0  # How far back the roll remembers how the camera was turned
SLACK_TURN = 1.5  # Servo degrees the roll must turn back before its servo takes up the slack the other way


class RollLevelParams(NodeParams):
    """How the head's roll levels the face it follows. Angles are the picture's rotation unless they say servo degrees."""

    enabled: bool = Field(default=True, description="Level the followed face with the roll servo; off, roll holds level")
    ratio: float = Field(
        default=0.065, gt=0.0, le=1.0,
        description="Degrees the picture turns per roll servo degree, once the slack is taken up: 0.063 fitted to the "
                    "robot's recorded steps (2026-11-06), which ranged 0.050-0.083",
    )
    direction: Literal[1, -1] = Field(
        default=-1, description="-1: a larger roll angle turns the picture's content clockwise, as measured on 2026-11-06",
    )
    slack: float = Field(
        default=10.0, ge=0.0, le=30.0,
        description="Servo degrees of play in the roll linkage, taken up at each reversal: 12 fitted to the recording; "
                    "a little under, as too much makes the head jump",
    )
    span: float = Field(
        default=25.0, gt=0.0, le=30.0,
        description="Servo degrees the roll may move either side of level: 5 short of its hard limits (ActuatorLimits)",
    )
    deadband: float = Field(default=1.0, ge=0.0, le=20.0, description="The roll turns only once its goal is further than this")
    reach: float = Field(
        default=0.8, gt=0.0, le=1.0,
        description="Share of the face's tilt in the picture each frame aims for: near 1, as the roll reaches only a degree or two",
    )
    smoothing: float = Field(default=0.1, gt=0.0, le=1.0, description="Share of each new estimate taken into the goal")
    max_speed: float = Field(default=15.0, gt=0.0, description="Top speed of the picture's rotation, degrees per second")
    servo_speed: float = Field(
        default=60.0, gt=0.0, description="Top roll servo speed, servo degrees per second: the servo itself stays calm",
    )
    max_acceleration: float = Field(default=60.0, gt=0.0, description="Degrees per second squared, speeding up and slowing down")
    servo_lag: float = Field(default=0.08, ge=0.0, le=0.5, description="How far the roll servo trails its commands")
    assumed_latency: float = Field(default=0.06, ge=0.0, le=0.5, description="Delay assumed for faces without a capture time")
    lost_after: float = Field(default=1.0, gt=0.0, description="Seconds without a usable face before the head eases back to level")
    min_score: float = Field(default=0.6, ge=0.0, le=1.0, description="Faces the detector is less sure of are not used")
    min_eye_distance: float = Field(
        default=16.0, ge=0.0, description="Pixels; eyes closer than this (a face beyond about 1.9 m) give too coarse an angle",
    )
    full_eye_distance: float = Field(
        default=30.0, gt=0.0,
        description="Pixels between the eyes (a face about a metre away) from which a frame counts fully; "
                    "smaller faces count for less, by the square, as their eye lines are noisier",
    )
    max_tilt: float = Field(default=40.0, gt=0.0, le=90.0, description="Eye lines steeper than this in the picture are not trusted")
    max_yaw: float = Field(default=40.0, gt=0.0, le=90.0, description="Faces turned further than this from the camera are not used")


def eye_line(face: FaceObservation) -> float:
    """The angle in the picture of the line from the face's right eye to its left (the person's own sides),
    degrees clockwise on screen: 0 for a level face."""
    right, left = face.landmarks[0], face.landmarks[1]
    return math.degrees(math.atan2(left[1] - right[1], left[0] - right[0]))


def choose_face(faces: Sequence[FaceObservation], target: tuple[float, float] | None) -> FaceObservation | None:
    """The face of the person the head follows: the one whose box holds the target point (the pose's nose),
    else the one whose nose is nearest it, within its box's size; None without a target or a face near it."""
    if target is None:
        return None
    def holds(face: FaceObservation) -> bool:
        x1, y1, x2, y2 = face.box
        return x1 <= target[0] <= x2 and y1 <= target[1] <= y2
    def near(face: FaceObservation) -> bool:
        x1, y1, x2, y2 = face.box
        return math.dist(face.landmarks[2], target) <= max(x2 - x1, y2 - y1)
    candidates = [face for face in faces if holds(face)] or [face for face in faces if near(face)]
    return min(candidates, key=lambda face: math.dist(face.landmarks[2], target)) if candidates else None


class RollLevel:
    """Feed `observe` each fresh face of the followed person and call `update` every control tick for the roll command."""

    def __init__(self, params: RollLevelParams, roll: float = LEVEL) -> None:
        self.params = params
        self.servo = roll  # The roll servo angle last commanded
        self.view = self._view(roll)  # The picture's rotation the linkage is meant to make, degrees
        self.rate = 0.0
        self.turning = False
        self.pushing = 0  # Which way the servo last pushed its linkage, in servo degrees: +1, -1, or 0 not yet
        self.extreme = roll  # The furthest servo angle (less the slack's push) reached since the last reversal
        self.goal = 0.0  # The rotation that levels the followed face; 0 keeps the head upright
        self.seen_at: float | None = None  # When a usable face was last observed
        self.tilt: float | None = None  # That face's eye line in the picture, degrees (measured)
        self.history: deque[tuple[float, float]] = deque()  # (time, rotation commanded)
        self.last_update: float | None = None

    def _view(self, servo: float) -> float:
        return self.params.direction * self.params.ratio * (servo - LEVEL)

    def span(self) -> float:
        """The largest rotation the roll may command either way: its servo, pushing half the slack past it,
        stays within `span` servo degrees of level."""
        return self.params.ratio * max(0.0, self.params.span - self.params.slack / 2)

    def usable(self, face: FaceObservation) -> float | None:
        """The face's eye line in the picture, or None if the face is too doubtful to level the head by."""
        p = self.params
        if face.score < p.min_score or math.dist(face.landmarks[0], face.landmarks[1]) < p.min_eye_distance:
            return None
        if face.head_pose is not None and abs(face.head_pose[0]) > p.max_yaw:
            return None  # Turned away: its eye line says little about its tilt
        angle = eye_line(face)
        return angle if abs(angle) <= p.max_tilt else None

    def rolled_at(self, when: float) -> float:
        """The picture's rotation commanded at time `when`, from the recent commands."""
        chosen = None
        for time, view in self.history:
            if time > when:
                break
            chosen = view
        if chosen is None:
            chosen = self.history[0][1] if self.history else self.view
        return chosen

    def level(self) -> bool:
        """Whether the followed face looks level enough in the picture for the roll to hold still."""
        return self.tilt is not None and abs(self.tilt) * self.params.reach <= self.params.deadband

    def observe(self, *, now: float, face: FaceObservation, captured: float | None = None) -> None:
        """One face of the followed person, in a frame captured at `captured`."""
        p = self.params
        angle = self.usable(face)
        if angle is None:
            return
        self.tilt, self.seen_at = angle, now
        seen = (captured if captured is not None else now - p.assumed_latency) - p.servo_lag
        reach = self.span()
        estimate = min(max(self.rolled_at(seen) + p.reach * angle, -reach), reach)
        eyes = math.dist(face.landmarks[0], face.landmarks[1])
        self.goal += p.smoothing * min(1.0, (eyes / p.full_eye_distance) ** 2) * (estimate - self.goal)

    def _command(self) -> None:
        """The servo angle that puts the linkage at the view: past it by half the slack, the way the servo moves.
        A turn back must exceed SLACK_TURN servo degrees to count, so the servo never chatters across the play."""
        p = self.params
        base = LEVEL + self.view / (p.direction * p.ratio)
        moved = base - self.extreme
        if (self.pushing >= 0 and moved < -SLACK_TURN) or (self.pushing <= 0 and moved > SLACK_TURN):
            self.pushing, self.extreme = (1 if moved > 0 else -1), base
        elif self.pushing * moved > 0:
            self.extreme = base  # Still going the same way
        self.servo = base + self.pushing * p.slack / 2

    def update(self, *, now: float) -> float:
        """This tick's roll servo command."""
        p = self.params
        dt = 0.0 if self.last_update is None else min(max(now - self.last_update, 0.0), MAX_STEP)
        self.last_update = now
        if self.seen_at is None or now - self.seen_at > p.lost_after:
            self.goal, self.tilt = 0.0, None  # No usable face lately: back to upright
        if not self.turning and abs(self.goal - self.view) > p.deadband:
            self.turning = True
        if self.turning:
            speed = min(p.max_speed, p.servo_speed * p.ratio)
            self.view, self.rate = s_curve_step(
                self.view, self.rate, self.goal, up=speed, down=speed, jerk=p.max_acceleration, dt=dt,
            )
            self._command()
            if self.view == self.goal and self.rate == 0.0:
                self.turning = False
        self.history.append((now, self.view))
        while self.history and self.history[0][0] < now - HISTORY_SECONDS:
            self.history.popleft()
        return self.servo
