"""Tune the head and the waist on the simulated robot with CMA-ES, and check a tuning before it is released.

The plant (plant.py) is the robot as recorded on 2026-11-06 and checked by tests/test_plant_fidelity.py.
A tuning is judged in two parts, in this order (docs/control-design.md, "Tuning, in the simulator"):

- **Constraints.** Every verdict of scenarios.judge (overshoot, oscillation, runaway, aim, chasing false
  detections, waist hunting, body overshoot) must pass. Each failure costs FAILURE, far more than any
  score, so the search finds feasible settings first and then improves them.
- **Score**, per run: the camera's error integral over the run (degrees x seconds, over the step's size for
  a step), the time to settle within SETTLE_DEGREES of a still person, the head's reversals (smoothness),
  servo travel and waist moves (effort), and how far false detections moved the camera.

Each candidate runs its stage's battery on the measured plant, on the four corner plants (gearing and
pixels per degree 15 % either way, robustness.CORNERS), on two stress plants and on fresh random plants
drawn each generation within the measured uncertainty (robustness.draw), the same draws for every
candidate of a generation. A scenario's cost is its mean over those plants plus half its worst.

Stages (`python -m ultraskelly.simulator.tuning STAGE`):

- `calm`: the head alone (waist braked), its acceleration at most 25 degrees of view per second squared;
- `responsive`: the head alone, the least acceleration that centres a 20-degree step within about 1 s on
  the measured plant (with the shaper on), the rest as for calm;
- `waist`: the whole robot with the head frozen at a tuning: when and how the waist moves.

`python -m ultraskelly.simulator.tuning check SETTING.json` runs the release check: the batteries and the
frozen held-out sets (scenarios.HELD_OUT_*) on the measured plant, the corners and 59 random plants, which
must all pass (a failure rate under 5 % with 95 % confidence).

The search is pycma's CMA-ES (Hansen's defaults) on parameters scaled to [0, 10], started from the
controller's defaults with a step of 3, for a fixed number of generations; every generation writes the best
setting so far to the output file. Each worker reuses one event loop for all its runs (scenarios.run).
"""
import argparse
import json
import logging
import math
import multiprocessing
import random
import statistics
import sys
import time
import warnings
from dataclasses import replace
from pathlib import Path

from ultraskelly.core.bot.motor.head_aim import HeadAimParams
from ultraskelly.core.bot.motor.waist_drive import WaistDriveParams
from ultraskelly.simulator import plant, robustness
from ultraskelly.simulator import scenarios as S

FAILURE = 100.0  # Cost of one failed verdict
STRESS = {  # Plants beyond the random draws' range that every candidate also runs on
    "pan gearing x1.3": {"pan_ratio": plant.PAN.gain * 1.3},
    "slower (1.5 x dead time, 0.1 s camera)": {"delay_scale": 1.5, "camera_latency": 0.1},
}
RANDOM_PLANTS = 2  # Fresh random plants per generation
BATTERIES = {  # Stage: (scenarios, whether the head runs alone)
    "calm": ([*S.HEAD_BATTERY, *S.TUNING_HEAD], True),
    "responsive": ([*S.HEAD_BATTERY, *S.TUNING_HEAD], True),
    "waist": ([scenario for scenario in S.SCENARIOS if not scenario.name.startswith("tilt")], False),
}
CHECKS = {  # Release check: (scenarios, whether the head runs alone)
    "head battery": (S.HEAD_BATTERY, True),
    "head held-out": (S.HELD_OUT_HEAD, True),
    "full scenarios": (S.SCENARIOS, False),
    "full held-out": (S.HELD_OUT_FULL, False),
}
HEAD_SPACE = {  # HeadAimParams field: (low, high)
    "pan_gain": (1.5, 5.0),
    "tilt_gain": (1.0, 5.0),
    "pan_speed": (20.0, 150.0),
    "tilt_speed": (6.0, 40.0),
    "pan_acceleration": (10.0, 25.0),  # The calm stage's range; the responsive stage's is RESPONSIVE_ACCELERATION
    "tilt_acceleration": (10.0, 25.0),
    "reach": (0.0, 1.0),
    "tracking_alpha": (0.15, 0.9),
    "tracking_beta": (0.0, 0.2),
    "prediction_gain": (0.0, 1.0),
    "prediction_lead": (0.0, 0.25),
    "min_target_speed": (3.0, 12.0),
    "pan_settle": (0.1, 0.5),
    "pan_resume": (0.5, 1.2),
    "tilt_settle": (0.1, 0.5),
    "tilt_resume": (0.4, 1.2),
}
RESPONSIVE_ACCELERATION = (25.0, 300.0)
WAIST_SPACE = {  # WaistDriveParams field: (low, high)
    "start_offset": (6.0, 16.0),
    "start_hold": (0.1, 0.8),
    "start_now": (14.0, 21.0),
    "leave": (0.0, 6.0),
    "min_move": (2.0, 8.0),
    "rest": (0.3, 2.0),
    "kick_seconds": (0.03, 0.12),
    "run_throttle": (0.52, 0.7),
    "closing_gain": (0.5, 6.0),
    "lead_time": (0.0, 1.5),
}
COUPLING_SPACE = {"counter_rotation": (0.5, 1.2)}  # HeadAimParams fields the waist stage tunes too
CENTRE_WITHIN = 1.0  # Seconds in which the responsive head must centre a 20-degree step on the measured plant


def apply(sim, setting: dict, head_only: bool) -> None:
    """Set a tuning, {"aim": {...}, "waist": {...}}, on a simulation's controllers in place."""
    for group in ("aim", "waist"):
        target = getattr(sim.head.params, group)
        for key, value in setting.get(group, {}).items():
            setattr(target, key, tuple(value) if isinstance(value, list) else value)
    if head_only:
        S.head_alone(sim)


def error_integral(scenario: S.Scenario, trace: S.Trace) -> float:
    """The camera's error integral while a person is in view (degrees x seconds), over the step's size for a
    step and over 10 degrees otherwise."""
    total, first = 0.0, None
    for camera_yaw, camera_pitch, yaw, pitch in zip(trace.camera_yaw, trace.camera_pitch, trace.person_yaw, trace.person_pitch):
        if yaw is None:
            continue
        error = math.hypot(yaw - camera_yaw, pitch - camera_pitch)
        first = error if first is None else first
        total += error * 0.01
    size = max(first or 0.0, 2.0) if scenario.step else 10.0
    return total / size


def run_cost(scenario: S.Scenario, trace: S.Trace, verdict: S.Verdict) -> float:
    """One run's cost (see the module's docstring)."""
    cost = FAILURE * len(verdict.failures)
    cost += 10.0 * max(0.0, verdict.overshoot - 0.5) + 3.0 * max(0.0, verdict.body_overshoot - 1.0)
    cost += 0.5 * (verdict.pan_reversals + verdict.tilt_reversals)
    if scenario.step:  # Beyond reach (no aim asked), settling counts only where it happens
        cost += verdict.settle if verdict.settle is not None else (scenario.seconds if scenario.aim else 0.0)
    cost += error_integral(scenario, trace)
    cost += 0.002 * verdict.servo_travel + 0.3 * verdict.waist_moves
    if scenario.still_within is not None:
        cost += 5.0 * verdict.wandered
    return cost


def _job(job):
    """One run in a worker: (candidate, stage, scenario name, plant name, plant settings, setting)."""
    logging.disable(logging.WARNING)
    candidate, battery_name, name, plant_name, settings, setting = job
    battery, head_only = (BATTERIES if battery_name in BATTERIES else CHECKS)[battery_name]
    scenario = {s.name: s for s in battery}[name]
    if settings:
        scenario = replace(scenario, settings={**scenario.settings, **settings})
    trace, verdict = S.run(scenario, lambda sim: apply(sim, setting, head_only))
    return candidate, name, plant_name, run_cost(scenario, trace, verdict), verdict


def plants_for(generation: int, seed: int) -> dict:
    """The plants a generation's candidates all run on: measured, corners, stress, and fresh random draws."""
    rng = random.Random(seed * 1000 + generation)
    draws = {f"random {generation}.{index}": robustness.draw(rng) for index in range(RANDOM_PLANTS)}
    return {"measured": {}, **robustness.CORNERS, **STRESS, **draws}


def evaluate(pool, stage: str, settings: list[dict], plants: dict) -> list[tuple[float, dict]]:
    """Each setting's total cost, and its runs' verdicts by scenario and plant."""
    battery, _ = BATTERIES[stage]
    jobs = [(index, stage, scenario.name, plant_name, plant_settings, setting)
            for index, setting in enumerate(settings) for scenario in battery for plant_name, plant_settings in plants.items()]
    costs = [{} for _ in settings]
    verdicts = [{} for _ in settings]
    for index, name, plant_name, cost, verdict in pool.imap_unordered(_job, jobs, chunksize=4):
        costs[index].setdefault(name, []).append(cost)
        verdicts[index][(name, plant_name)] = verdict
    results = []
    for index, setting in enumerate(settings):
        total = sum(statistics.fmean(values) + 0.5 * max(values) for values in costs[index].values())
        if stage == "responsive":  # The owner's target, on the measured plant, at the least acceleration that meets it
            for name in ("step-20", "step-20-right"):
                settle = verdicts[index][(name, "measured")].settle
                total += 100.0 * max(0.0, (settle if settle is not None else 4.0) - CENTRE_WITHIN)
            total += setting["aim"]["pan_acceleration"] / 25.0
        results.append((total, verdicts[index]))
    return results


def space_for(stage: str) -> dict:
    """{(group, field): (low, high)} for a stage."""
    if stage == "waist":
        return {**{("waist", key): span for key, span in WAIST_SPACE.items()},
                **{("aim", key): span for key, span in COUPLING_SPACE.items()}}
    space = {("aim", key): span for key, span in HEAD_SPACE.items()}
    if stage == "responsive":
        space[("aim", "pan_acceleration")] = space[("aim", "tilt_acceleration")] = RESPONSIVE_ACCELERATION
    return space


def decode(space: dict, x, base: dict) -> dict:
    """A setting from CMA-ES's point in [0, 10] per parameter, on top of `base`."""
    setting = {"aim": dict(base.get("aim", {})), "waist": dict(base.get("waist", {}))}
    for (group, key), (low, high), value in zip(space, space.values(), x):
        setting[group][key] = round(low + (high - low) * min(max(value, 0.0), 10.0) / 10.0, 4)
    aim = setting["aim"]
    aim["pan_settle"] = min(aim.get("pan_settle", 0.3), aim.get("pan_resume", 0.6))
    aim["tilt_settle"] = min(aim.get("tilt_settle", 0.25), aim.get("tilt_resume", 0.5))
    return setting


def encode(space: dict, setting: dict) -> list[float]:
    defaults = {"aim": HeadAimParams().model_dump(), "waist": WaistDriveParams().model_dump()}
    point = []
    for (group, key), (low, high) in space.items():
        value = setting.get(group, {}).get(key, defaults[group][key])
        point.append(min(max(10.0 * (value - low) / (high - low), 0.0), 10.0))
    return point


def tune(stage: str, generations: int, base: dict, out: Path, seed: int = 0, workers: int = 15) -> dict:
    """CMA-ES over the stage's space, from `base` (and the defaults); returns the best setting found."""
    warnings.filterwarnings("ignore", module="cma")
    import cma

    space = space_for(stage)
    strategy = cma.CMAEvolutionStrategy(encode(space, base), 3.0, {"bounds": [0, 10], "seed": seed + 1, "verbose": -9})
    best, best_cost = None, math.inf
    started = time.time()
    with multiprocessing.get_context("spawn").Pool(workers) as pool:
        for generation in range(generations):
            points = strategy.ask()
            settings = [decode(space, point, base) for point in points]
            plants = plants_for(generation, seed)
            results = evaluate(pool, stage, settings, plants)
            strategy.tell(points, [cost for cost, _ in results])
            for setting, (cost, verdicts) in zip(settings, results):
                if cost < best_cost:
                    failing = sorted({f"{name} on {plant_name}" for (name, plant_name), v in verdicts.items() if not v.passed})
                    best, best_cost = setting, cost
                    out.write_text(json.dumps({"stage": stage, "cost": round(cost, 2), "generation": generation,
                                               "setting": best, "failing": failing}, indent=1))
            costs = sorted(cost for cost, _ in results)
            print(f"generation {generation}: best {costs[0]:.1f}, median {statistics.median(costs):.1f}; "
                  f"best so far {best_cost:.1f} ({time.time() - started:.0f} s)", flush=True)
    return best


def check(setting: dict, count: int = 59, seed: int = 2026, workers: int = 15) -> dict:
    """The release check: every check battery on the measured plant, the corners and `count` random plants.
    Returns, per battery and plant group, the runs, failures and plants passing everything."""
    plants = {"measured": {}, **{f"corner: {name}": value for name, value in robustness.CORNERS.items()},
              **{f"random {index}": value for index, value in enumerate(robustness.plants(count, seed))}}
    jobs = [(0, battery, scenario.name, plant_name, plant_settings, setting)
            for battery, (scenarios, _) in CHECKS.items() for scenario in scenarios for plant_name, plant_settings in plants.items()]
    with multiprocessing.get_context("spawn").Pool(workers) as pool:
        results = list(pool.imap_unordered(_check_job, jobs, chunksize=4))
    report = {}
    for battery, name, plant_name, verdict in results:
        group = "measured" if plant_name == "measured" else plant_name.split(" ")[0].rstrip(":")
        entry = report.setdefault(battery, {}).setdefault(group, {"runs": 0, "failed": [], "plants": {}, "settles": []})
        entry["runs"] += 1
        entry["plants"].setdefault(plant_name, True)
        if not verdict.passed:
            entry["failed"].append(f"{name} on {plant_name}: {verdict.failures}")
            entry["plants"][plant_name] = False
        if verdict.settle is not None:
            entry["settles"].append(verdict.settle)
    return report


def _check_job(job):
    candidate, battery, name, plant_name, settings, setting = job
    logging.disable(logging.WARNING)
    scenarios, head_only = CHECKS[battery]
    scenario = {s.name: s for s in scenarios}[name]
    if settings:
        scenario = replace(scenario, settings={**scenario.settings, **settings})
    trace, verdict = S.run(scenario, lambda sim: apply(sim, setting, head_only))
    return battery, name, plant_name, verdict


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stage", choices=[*BATTERIES, "check"])
    parser.add_argument("setting", nargs="?", type=Path, help="A JSON tuning to start from, or to check")
    parser.add_argument("--generations", type=int, default=30)
    parser.add_argument("--out", type=Path, default=None, help="Where the best setting is kept (default: STAGE.json)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--plants", type=int, default=59, help="Random plants for the release check")
    args = parser.parse_args()
    base = {}
    if args.setting is not None:
        loaded = json.loads(args.setting.read_text())
        base = loaded.get("setting", loaded)
    if args.stage == "check":
        report = check(base, args.plants)
        for battery, groups in report.items():
            for group, entry in groups.items():
                plants = entry["plants"]
                settles = entry["settles"]
                print(f"{battery} / {group}: {entry['runs'] - len(entry['failed'])}/{entry['runs']} runs pass, "
                      f"{sum(plants.values())}/{len(plants)} plants pass everything"
                      + (f"; settle median {statistics.median(settles):.2f} s, worst {max(settles):.2f} s" if settles else ""))
                for failure in entry["failed"][:8]:
                    print(f"    {failure}")
        return
    out = args.out or Path(f"{args.stage}.json")
    best = tune(args.stage, args.generations, base, out, seed=args.seed)
    print(json.dumps(best, indent=1))
    sys.stdout.flush()


if __name__ == "__main__":
    main()
