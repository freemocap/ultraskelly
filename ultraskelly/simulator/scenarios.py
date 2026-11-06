"""Closed-loop scenarios that catch a head or waist that swings, before any motor runs.

Each scenario puts people in front of the simulated robot and runs the production
controllers through the simulator's camera and servos (see engine.py). Its verdict fails on:

- overshoot: the camera swinging past a person it turned to by more than OVERSHOOT_DEGREES;
- oscillation: the commanded pan or tilt reversing more often than the scenario allows;
- runaway: a servo command reaching its end stop, or losing a person who stays in view;
- aim: still off a standing person by more than AIM_DEGREES at the end;
- waist hunting: the waist turning one way and then back while the person stands still.

Bearings are degrees from the robot's base: positive yaw to the robot's left as the
simulator draws it, positive pitch up.
"""
import asyncio
from dataclasses import dataclass, field
from typing import Callable

from ultraskelly.simulator.engine import Person, Settings, Simulation

OVERSHOOT_DEGREES = 2.0
AIM_DEGREES = 2.5
REVERSAL_DEGREES = 0.5  # A turn back counts once the command has come back this far
LIMIT_MARGIN = 2.0  # Degrees from a servo's 0 or 180 that count as reaching its end stop


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    seconds: float
    people: Callable[[float], list[Person]]
    settings: dict = field(default_factory=dict)  # Changes to the simulator's Settings
    step: bool = True  # People stand still after appearing: overshoot and final aim apply
    max_reversals: int = 1  # Per servo
    max_waist_moves: int = 1


@dataclass
class Trace:
    time: list[float] = field(default_factory=list)
    pan_command: list[float] = field(default_factory=list)
    tilt_command: list[float] = field(default_factory=list)
    camera_yaw: list[float] = field(default_factory=list)
    camera_pitch: list[float] = field(default_factory=list)
    person_yaw: list[float | None] = field(default_factory=list)  # The person the camera follows, if any
    person_pitch: list[float | None] = field(default_factory=list)
    waist: list[float] = field(default_factory=list)
    waist_drive: list[float | None] = field(default_factory=list)
    half_fov: float = 34.5  # Degrees from the image centre to its side edge


@dataclass
class Verdict:
    overshoot: float = 0.0
    pan_reversals: int = 0
    tilt_reversals: int = 0
    runaway: str | None = None
    final_error: float | None = None
    waist_moves: int = 0
    waist_reversals: int = 0
    failures: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.failures


def still(yaw: float, pitch: float = 0.0, appear: float = 0.0) -> Callable[[float], list[Person]]:
    return lambda t: [Person(yaw, pitch)] if t >= appear else []


def walking(start: float, end: float, speed: float, pitch: float = 0.0, pause: float = 0.5) -> Callable[[float], list[Person]]:
    """Someone standing at `start` who walks to `end` at `speed` degrees a second after `pause`, then stops."""
    def people(t: float) -> list[Person]:
        travelled = max(0.0, t - pause) * speed
        yaw = start + min(travelled, abs(end - start)) * (1 if end > start else -1)
        return [Person(yaw, pitch)]
    return people


def two_people(a: float, b: float, swap_period: float = 0.1) -> Callable[[float], list[Person]]:
    """Two people at yaw a and b; who scores higher swaps every `swap_period` seconds."""
    def people(t: float) -> list[Person]:
        a_first = int(t / swap_period) % 2 == 0
        return [Person(a, 0.0, 0.92 if a_first else 0.85), Person(b, 0.0, 0.85 if a_first else 0.92)]
    return people


def comes_and_goes(first: float, second: float, gone: tuple[float, float]) -> Callable[[float], list[Person]]:
    """Someone at `first`, gone during `gone`, then someone else at `second`."""
    return lambda t: [] if gone[0] <= t < gone[1] else [Person(first if t < gone[0] else second, 0.0)]


SCENARIOS = [
    Scenario("left", "A person steps in 20 degrees to the robot's left", 4.0, still(20.0)),
    Scenario("right", "A person steps in 20 degrees to the robot's right", 4.0, still(-20.0)),
    Scenario("up", "A person 12 degrees up", 4.0, still(0.0, 12.0)),
    Scenario("down", "A person 12 degrees down", 4.0, still(0.0, -12.0)),
    Scenario("far", "A person 27 degrees off: the head turns and the waist brings it back", 8.0, still(27.0)),
    # Turning to someone, following them back and counter-rotating while the waist recentres
    # are three purposeful phases: two reversals, not a swing.
    Scenario("walk", "A person walks across, 30 to -30 degrees at 12 deg/s", 8.0, walking(30.0, -30.0, 12.0),
             step=False, max_reversals=2, max_waist_moves=2),
    Scenario("walk-away", "A person walks from 10 to 100 degrees at 15 deg/s, out of the head's reach", 9.0,
             walking(10.0, 100.0, 15.0), step=False, max_reversals=2, max_waist_moves=2),
    Scenario("two-people", "Two people 24 degrees apart whose scores keep swapping", 6.0, two_people(10.0, -14.0),
             step=False, max_reversals=1),
    Scenario("noisy", "A person 10 degrees off with 4 px of detection jitter and a third of detections dropped", 6.0,
             still(10.0), settings={"noise_px": 4.0, "dropout": 0.33}, max_reversals=1),
    Scenario("lost", "A person at 15 degrees leaves for 2 s; another appears at -15", 8.0,
             comes_and_goes(15.0, -15.0, (3.0, 5.0)), step=False, max_reversals=2, max_waist_moves=2),
]
BY_NAME = {scenario.name: scenario for scenario in SCENARIOS}


def reversals(values: list[float], threshold: float = REVERSAL_DEGREES) -> int:
    """How often a series turns back by at least `threshold` from its last extreme."""
    if not values:
        return 0
    count, direction, extreme = 0, 0, values[0]  # direction: +1 rising, -1 falling, 0 not yet moved
    for value in values[1:]:
        if direction == 0:
            if abs(value - extreme) >= threshold:
                direction, extreme = (1 if value > extreme else -1), value
        elif direction * (value - extreme) > 0:
            extreme = value  # Still going the same way
        elif abs(value - extreme) >= threshold:
            count, direction, extreme = count + 1, -direction, value
    return count


def followed(sim: Simulation) -> Person | None:
    """The person nearest where the camera points: the one it follows."""
    people = sim.present()
    if not people:
        return None
    yaw, pitch = sim.camera_pose()
    return min(people, key=lambda person: (person.yaw - yaw) ** 2 + (person.pitch - pitch) ** 2)


async def record(scenario: Scenario, prepare: Callable[[Simulation], None] | None = None) -> Trace:
    """Run a scenario through the simulator; `prepare` may change the controllers before it starts."""
    sim = Simulation(Settings(scenario="manual", **scenario.settings), people=scenario.people)
    if prepare is not None:
        prepare(sim)
    trace = Trace(half_fov=sim.settings.camera_hfov / 2)
    try:
        for _ in range(int(round(scenario.seconds / sim.dt))):
            await sim.step()
            yaw, pitch = sim.camera_pose()
            person = followed(sim)
            trace.time.append(sim.now())
            trace.pan_command.append(sim.head.pan_angle)
            trace.tilt_command.append(sim.head.tilt_angle)
            trace.camera_yaw.append(yaw)
            trace.camera_pitch.append(pitch)
            trace.person_yaw.append(None if person is None else person.yaw)
            trace.person_pitch.append(None if person is None else person.pitch)
            trace.waist.append(sim.waist)
            trace.waist_drive.append(sim.waist_drive())
    finally:
        sim.close()
    return trace


def judge(scenario: Scenario, trace: Trace) -> Verdict:
    verdict = Verdict()
    verdict.pan_reversals = reversals(trace.pan_command)
    verdict.tilt_reversals = reversals(trace.tilt_command)
    for name, count in (("pan", verdict.pan_reversals), ("tilt", verdict.tilt_reversals)):
        if count > scenario.max_reversals:
            verdict.failures.append(f"oscillation: {name} reversed {count} times (allowed {scenario.max_reversals})")

    for name, series in (("pan", trace.pan_command), ("tilt", trace.tilt_command)):
        if min(series) <= LIMIT_MARGIN or max(series) >= 180 - LIMIT_MARGIN:
            verdict.runaway = f"{name} reached its end stop"
    # A person who stays put in the frame's view must stay in view.
    half_fov = trace.half_fov
    errors = [
        None if py is None else ((py - cy) ** 2 + (pp - cp) ** 2) ** 0.5
        for py, pp, cy, cp in zip(trace.person_yaw, trace.person_pitch, trace.camera_yaw, trace.camera_pitch)
    ]
    if scenario.step and any(error is not None and error > half_fov for error in errors[len(errors) // 4:]):
        verdict.runaway = verdict.runaway or "lost the person it had turned to"
    if verdict.runaway:
        verdict.failures.append(f"runaway: {verdict.runaway}")

    if scenario.step:
        # Overshoot: how far the camera went past the person, on each axis, after first reaching them.
        for camera, person in ((trace.camera_yaw, trace.person_yaw), (trace.camera_pitch, trace.person_pitch)):
            pairs = [(c, p) for c, p in zip(camera, person) if p is not None]
            if not pairs or abs(pairs[0][1] - pairs[0][0]) < OVERSHOOT_DEGREES:
                continue  # Already on target on this axis
            start_side = pairs[0][1] - pairs[0][0]
            past = [-(p - c) * (1 if start_side > 0 else -1) for c, p in pairs]
            verdict.overshoot = max(verdict.overshoot, max(0.0, max(past)))
        if verdict.overshoot > OVERSHOOT_DEGREES:
            verdict.failures.append(f"overshoot: {verdict.overshoot:.1f} degrees past the person")
        final = [error for error in errors[-10:] if error is not None]
        verdict.final_error = max(final) if final else None
        if verdict.final_error is None or verdict.final_error > AIM_DEGREES:
            verdict.failures.append(f"aim: {verdict.final_error} degrees off at the end")

    moves, signs = 0, []
    driving = False
    for drive in trace.waist_drive:
        now_driving = bool(drive)
        if now_driving and not driving:
            moves += 1
            signs.append(1 if drive > 0 else -1)
        driving = now_driving
    verdict.waist_moves = moves
    verdict.waist_reversals = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    if moves > scenario.max_waist_moves:
        verdict.failures.append(f"waist hunting: {moves} moves (allowed {scenario.max_waist_moves})")
    elif scenario.step and verdict.waist_reversals:
        verdict.failures.append(f"waist hunting: turned back {verdict.waist_reversals} times")
    return verdict


def run(scenario: Scenario, prepare: Callable[[Simulation], None] | None = None) -> tuple[Trace, Verdict]:
    trace = asyncio.run(record(scenario, prepare))
    return trace, judge(scenario, trace)
