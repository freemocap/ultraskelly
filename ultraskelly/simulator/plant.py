"""The robot's head servos and waist as measured on 2026-11-06, for the simulator and its fidelity test.

Each head servo is a chain fitted to tests/data/motor-response-20261106 (recorded with
scripts/record_motor_response.py, analysed by scripts/analyze_motor_response.py; fit in
tests/test_plant_fidelity.py):

    command --dead time--> servo (slew-limited) --slack--> linkage x gain --> head mode --> view

- dead time: the picture starts moving this long after a command;
- slew: the servo's top speed, servo degrees per second;
- slack: servo degrees of play the servo crosses after a reversal before the linkage follows;
- gain: degrees of view per servo degree once the slack is taken up;
- mode: the head's own ringing, a second-order system of natural frequency `frequency` (rad/s)
  and damping ratio `damping`: a step overshoots by exp(-damping pi / sqrt(1 - damping^2)).

Views are degrees from where the axis looks with its servo at 90. The waist is a first-order
motor with static friction and a different speed each way.
"""
import math
from collections import deque
from dataclasses import dataclass

MIDDLE = 90.0


def play(output: float, driver: float, slack: float) -> float:
    """Where a linkage with `slack` degrees of play sits: it moves only when the driver pushes it from one side."""
    return min(max(output, driver - slack / 2), driver + slack / 2)


@dataclass(frozen=True)
class AxisModel:
    gain: float  # Degrees of view per servo degree
    dead_time: float  # Seconds from a command to the picture starting to move
    slew: float  # Servo degrees per second at most
    slack: float  # Servo degrees of play
    frequency: float = 0.0  # Natural frequency of the head's mode, rad/s; 0: the view follows the linkage at once
    damping: float = 1.0  # Damping ratio of that mode
    friction: float = 0.0  # Dry friction on the head, degrees per second squared: damps small swings most
    damping_down: float | None = None  # Damping ratio while the view moves toward smaller views; None: `damping`


class Axis:
    """One head servo and what it does to the camera's view."""

    def __init__(self, model: AxisModel, start: float = MIDDLE) -> None:
        self.model = model
        self.commands: deque[tuple[float, float]] = deque([(-math.inf, start)])  # (time sent, servo degrees)
        self.servo = self.linkage = start
        self.view = model.gain * (start - MIDDLE)
        self.velocity = 0.0  # Of the view, degrees per second

    def command(self, time: float, value: float) -> None:
        """A servo command sent at `time`."""
        if value != self.commands[-1][1]:
            self.commands.append((time, value))

    def arriving(self, now: float) -> float:
        """The command the servo is acting on at `now`: the latest one sent a dead time earlier."""
        due = now - self.model.dead_time
        while len(self.commands) > 1 and self.commands[1][0] <= due:
            self.commands.popleft()
        return self.commands[0][1]

    def advance(self, now: float, dt: float) -> float:
        """Move the axis on by `dt` to time `now`; returns the view."""
        m = self.model
        target = self.arriving(now)
        step = m.slew * dt
        self.servo += min(max(target - self.servo, -step), step)
        self.linkage = play(self.linkage, self.servo, m.slack)
        goal = m.gain * (self.linkage - MIDDLE)
        if m.frequency <= 0.0:
            self.view, self.velocity = goal, 0.0
        elif m.friction > 0.0 or m.damping_down is not None:
            self.view, self.velocity = with_friction(self.view - goal, self.velocity, m, dt)
            self.view += goal
        else:
            self.view, self.velocity = second_order(self.view - goal, self.velocity, m.frequency, m.damping, dt)
            self.view += goal
        return self.view


def with_friction(offset: float, velocity: float, m: AxisModel, dt: float, step: float = 0.001) -> tuple[float, float]:
    """The head's mode with dry friction, and damping that may differ by the way it moves, in steps of at
    most `step` seconds. A still head stays put while the spring's pull is within the friction."""
    steps = max(1, math.ceil(dt / step))
    h = dt / steps
    spring = m.frequency ** 2
    up, down = 2.0 * m.damping * m.frequency, 2.0 * (m.damping if m.damping_down is None else m.damping_down) * m.frequency
    for _ in range(steps):
        pull = -spring * offset - (up if velocity >= 0.0 else down) * velocity
        if velocity == 0.0:
            if abs(pull) <= m.friction:
                continue  # Stuck
            pull -= math.copysign(m.friction, pull)
        else:
            pull -= math.copysign(m.friction, velocity)
        moving = velocity + pull * h
        velocity = 0.0 if velocity != 0.0 and moving * velocity < 0.0 else moving  # Friction stops it, never reverses it
        offset += velocity * h
    return offset, velocity


def second_order(offset: float, velocity: float, frequency: float, damping: float, dt: float) -> tuple[float, float]:
    """Exactly where an underdamped mass-spring settling to 0 is `dt` later: (offset, velocity)."""
    if damping >= 1.0:  # Critically or over damped: small Euler steps are plenty
        steps = max(1, math.ceil(dt * frequency * 20))
        h = dt / steps
        for _ in range(steps):
            velocity += (-frequency ** 2 * offset - 2 * damping * frequency * velocity) * h
            offset += velocity * h
        return offset, velocity
    decay, ringing = damping * frequency, frequency * math.sqrt(1.0 - damping ** 2)
    fade, c, s = math.exp(-decay * dt), math.cos(ringing * dt), math.sin(ringing * dt)
    return (fade * (offset * c + (velocity + decay * offset) / ringing * s),
            fade * (velocity * c - (decay * velocity + frequency ** 2 * offset) / ringing * s))


@dataclass(frozen=True)
class WaistModel:
    breakaway: float  # Drive a still waist needs to start turning
    stall: float  # A turning waist stops below this drive
    speed_negative: float  # Degrees per second at full negative drive (it turns the view right)
    speed_positive: float  # At full positive drive, the other way
    time_constant: float  # Seconds to reach its speed
    brake_time: float = 0.03  # Seconds to stop when braked (drive 0)
    coast_time: float = 0.3  # And when released to coast (drive None)


class Waist:
    """The torso's turn, degrees, positive the way negative drive turns it."""

    def __init__(self, model: WaistModel) -> None:
        self.model = model
        self.angle = 0.0
        self.speed = 0.0
        self.moving = False

    def advance(self, drive: float | None, dt: float) -> float:
        m = self.model
        strength = abs(drive or 0.0)
        self.moving = strength > m.stall if self.moving else strength >= m.breakaway
        if self.moving and drive:
            wanted, settling = -drive * (m.speed_negative if drive < 0 else m.speed_positive), m.time_constant
        else:
            wanted, settling = 0.0, m.coast_time if drive is None else m.brake_time
        self.speed += (wanted - self.speed) * min(1.0, dt / settling)
        self.angle += self.speed * dt
        return self.angle


# The robot as measured on 2026-11-06 (tests/data/motor-response-20261106), fitted by least squares on
# each labelled segment's view and the big steps' overshoot, settling and dead time (tests/test_plant_fidelity.py).
PAN = AxisModel(gain=0.2607, dead_time=0.044, slew=576.0, slack=2.48, frequency=11.84, damping=0.407,
                friction=19.1, damping_down=0.281)
TILT = AxisModel(gain=0.0809, dead_time=0.040, slew=830.0, slack=1.80, frequency=37.97, damping=0.641,
                 friction=21.7, damping_down=0.368)
ROLL = AxisModel(gain=0.0634, dead_time=0.043, slew=375.0, slack=11.99, frequency=82.07, damping=0.296, friction=7.14)
# Six pulses: 0.6 drive did not move it in 0.5 s; 0.7 for 0.5 s turned it 13.3 degrees (top 36 deg/s) with
# negative drive and 5.7 (top 15) with positive. The stall drive of a turning waist was not measured.
WAIST = WaistModel(breakaway=0.65, stall=0.45, speed_negative=50.4, speed_positive=21.5, time_constant=0.16)
