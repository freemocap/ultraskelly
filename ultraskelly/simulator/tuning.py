"""Tune the head's aim on the measured plant against a stated cost, over the head battery.

The plant (plant.py) is the robot as recorded on 2026-11-06 and checked by tests/test_plant_fidelity.py;
the battery (scenarios.HEAD_BATTERY) is the robot's present setup, the head alone. Each candidate
setting of HeadAimParams runs every battery scenario on the measured plant and on six stressed ones,
and costs, per run:

    5 x overshoot past the person (degrees)
  + 3 x reversals beyond what the scenario allows
  + 1 x seconds to settle within 1.5 degrees of a still person (the whole run if it never does)
  + 2 x final aim error on a still person within reach (degrees)
  + 0.002 x servo travel (pan and tilt command degrees: wear and jitter)
  + 5 x degrees the camera wanders with only false detections about
  + 30 x each hard failure (runaway, a person lost, chasing false detections)

summed with weight 1 on the measured plant and 1/2 on each stressed one: pan gearing 15 % either way,
1.5 and 2 times, a head that rings at 45 % instead of 22-32 %, and 1.5 times the dead time. The search samples the space
at random, then refines the best setting one parameter at a time. A candidate takes about 4 s on one
worker; 120 samples and the refinement took 4 minutes on eight (2026-11-06).

    python -m ultraskelly.simulator.tuning [samples] [checkpoint.json]

The checkpoint keeps the best setting and the refinement's step after every round; run again with the
same checkpoint to carry on from it.
"""
import json
import logging
import math
import multiprocessing
import random
import sys
import time
from dataclasses import replace
from pathlib import Path

from ultraskelly.simulator import plant
from ultraskelly.simulator.scenarios import HEAD_BATTERY, Scenario, head_alone, run

PLANTS = {  # Name: (simulator settings, weight)
    "measured": ({}, 1.0),
    "pan gearing -15 %": ({"pan_ratio": plant.PAN.gain * 0.85}, 0.5),
    "pan gearing +15 %": ({"pan_ratio": plant.PAN.gain * 1.15}, 0.5),
    "less damped (45 % overshoot)": ({"pan_damping": 0.25, "pan_damping_down": 0.18}, 0.5),
    "slower (1.5 x dead time)": ({"delay_scale": 1.5}, 0.5),
    # A tracking run (20261106-194104) once looked like twice the fitted pan gearing on large moves
    # (447 px for 85 servo degrees); the replayed recording says otherwise, but swinging is the costliest
    # mistake, so the tuner weighs it too.
    "pan gearing x1.5": ({"pan_ratio": plant.PAN.gain * 1.5}, 0.5),
    "pan gearing x2": ({"pan_ratio": plant.PAN.gain * 2.0}, 0.5),
}
SPACE = {  # HeadAimParams: (low, high) for numbers, or the choices
    # The best reach sits on 0.3; along its ridge, reach x smoothing near 0.21, the cost rises below it
    # (0.25 at 0.83: 70.6, 0.21 at 1.0: 80.6, against 64.4)
    "reach": (0.3, 0.95),
    "smoothing": (0.15, 0.8),
    "max_speed": (10.0, 60.0),
    "max_acceleration": (5.0, 400.0),  # From 15 the best setting sat on that bound, so it reaches lower
    "deadband": (0.3, 2.5),
    "servo_lag": (0.04, 0.18),
    "jump_samples": [2, 3, 4, 5],
    "shaper": [True, False],
}
HARD = ("runaway", "chased", "waist hunting", "aim")  # Failures that count as hard, beyond their measured parts


def run_cost(scenario: Scenario, verdict) -> float:
    """One run's cost (see the module's docstring)."""
    cost = 5.0 * verdict.overshoot
    allowed = scenario.max_reversals
    cost += 3.0 * (max(0, verdict.pan_reversals - allowed) + max(0, verdict.tilt_reversals - allowed))
    if scenario.step and scenario.aim:
        cost += verdict.settle if verdict.settle is not None else scenario.seconds
        cost += 2.0 * (verdict.final_error if verdict.final_error is not None else 10.0)
    cost += 0.002 * verdict.servo_travel
    if scenario.still_within is not None:
        cost += 5.0 * verdict.wandered
    cost += 30.0 * sum(1 for failure in verdict.failures if failure.startswith(HARD) or "lost" in failure)
    return cost


def evaluate(setting: dict) -> tuple[float, dict]:
    """A HeadAimParams setting's total cost, and each plant's runs: {plant: {scenario: (cost, verdict numbers)}}."""
    logging.disable(logging.WARNING)

    def prepare(sim):
        head_alone(sim)
        for key, value in setting.items():
            setattr(sim.head.params.aim, key, value)

    total, details = 0.0, {}
    for name, (settings, weight) in PLANTS.items():
        runs = {}
        for scenario in HEAD_BATTERY:
            trace, verdict = run(replace(scenario, settings={**scenario.settings, **settings}), prepare)
            cost = run_cost(scenario, verdict)
            total += weight * cost
            runs[scenario.name] = (round(cost, 3), {
                "overshoot": round(verdict.overshoot, 2), "reversals": verdict.pan_reversals + verdict.tilt_reversals,
                "settle": None if verdict.settle is None else round(verdict.settle, 2),
                "final_error": None if verdict.final_error is None else round(verdict.final_error, 2),
                "travel": round(verdict.servo_travel), "failures": verdict.failures,
            })
        details[name] = runs
    return total, details


def sample(rng: random.Random) -> dict:
    return {key: rng.choice(span) if isinstance(span, list) else rng.uniform(*span) for key, span in SPACE.items()}


def neighbours(setting: dict, scale: float) -> list[dict]:
    """Settings one parameter away from `setting`: numbers moved by `scale` of their range either way."""
    found = []
    for key, span in SPACE.items():
        if isinstance(span, list):
            found += [{**setting, key: choice} for choice in span if choice != setting[key]]
        else:
            low, high = span
            for sign in (-1, 1):
                moved = min(max(setting[key] + sign * scale * (high - low), low), high)
                if moved != setting[key]:
                    found.append({**setting, key: moved})
    return found


def tune(samples: int = 160, seed: int = 0, workers: int = 8, checkpoint: Path | None = None) -> tuple[dict, float, dict]:
    """Random search, then refinement one parameter at a time, the step halving from 15 % of each range.
    After each round the checkpoint holds the best setting and the step; given a checkpoint that holds
    them, the run carries on from there."""
    saved = json.loads(checkpoint.read_text()) if checkpoint is not None and checkpoint.exists() else {}

    def keep(best: dict, cost: float, scale: float) -> None:
        if checkpoint is not None:
            checkpoint.write_text(json.dumps({"best": best, "cost": cost, "scale": scale}, indent=1))

    with multiprocessing.get_context("spawn").Pool(workers) as pool:
        def costs(settings: list[dict]) -> list[tuple[float, dict]]:
            # A candidate takes about 4 s on one worker. Pool.map would wait forever for a result a worker
            # lost; this stops with the checkpoint kept instead.
            limit = 120 + 60 * math.ceil(len(settings) / workers)
            return pool.map_async(evaluate, settings, chunksize=1).get(timeout=limit)

        if "scale" in saved:
            best, scale = saved["best"], saved["scale"]
            cost, details = costs([best])[0]
            print(f"carried on from the checkpoint: cost {cost:.2f}, step {scale:.4f}", flush=True)
        else:
            rng = random.Random(seed)
            candidates = [sample(rng) for _ in range(samples)]
            best, (cost, details) = min(zip(candidates, costs(candidates)), key=lambda pair: pair[1][0])
            scale = 0.15
            print(f"random search: best cost {cost:.2f} of {samples}", flush=True)
            keep(best, cost, scale)
        while scale > 0.01:
            trials = neighbours(best, scale)
            better = min(zip(trials, costs(trials)), key=lambda pair: pair[1][0])
            if better[1][0] < cost - 1e-6:
                best, (cost, details) = better[0], better[1]
                print(f"  scale {scale:.4f}: improved to {cost:.2f}", flush=True)
            else:
                scale /= 2
                print(f"  scale {scale * 2:.4f}: no better neighbour ({better[1][0]:.2f}); halving", flush=True)
            keep(best, cost, scale)
    return best, cost, details


def main() -> None:
    samples = int(sys.argv[1]) if len(sys.argv) > 1 else 160
    checkpoint = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    started = time.time()
    best, cost, details = tune(samples, checkpoint=checkpoint)
    rounded = {key: (round(value, 3) if isinstance(value, float) else value) for key, value in best.items()}
    print(json.dumps({"best": rounded, "cost": round(cost, 2), "seconds": round(time.time() - started)}, indent=1))
    for name, runs in details.items():
        print(f"{name}:")
        for scenario, (run_cost_, numbers) in runs.items():
            print(f"   {scenario:18s} cost {run_cost_:6.2f} {numbers}")


if __name__ == "__main__":
    main()
