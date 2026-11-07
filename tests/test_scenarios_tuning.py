"""The robot's head tuning on a plant like the one recorded on 2026-11-06: no swinging past people.

Run 20261106-194104 showed about 0.65 degrees of view per pan servo degree on large moves (the
controller assumes 0.33) and a picture that moved 0.1-0.2 s after a command (it assumes 0.08 s).
"""
import dataclasses

import pytest

from ultraskelly.__main__bot import launch_config
from ultraskelly.simulator import scenarios as S

RECORDED_PLANT = {"pan_ratio": 0.65, "servo_lag": 0.15}


@pytest.mark.parametrize("name", ["left", "right", "noisy", "lost"])
def test_the_launched_head_does_not_swing_past_people_on_the_recorded_plant(name):
    aim, scenario = launch_config().orientation.aim, next(s for s in S.SCENARIOS if s.name == name)

    def prepare(sim):
        sim.head.params.waist.enabled = sim.head.params.roll.enabled = False
        for key, value in aim.model_dump().items():  # In place: the head's aim already holds this object
            setattr(sim.head.params.aim, key, value)

    trace, verdict = S.run(dataclasses.replace(scenario, settings={**scenario.settings, **RECORDED_PLANT}), prepare)
    assert verdict.overshoot <= 0.5, verdict.failures
    assert not [f for f in verdict.failures if f.startswith(("overshoot", "oscillation", "runaway"))]


def test_the_old_gain_does_swing_on_that_plant():
    scenario = next(s for s in S.SCENARIOS if s.name == "left")

    def prepare(sim):
        sim.head.params.waist.enabled = sim.head.params.roll.enabled = False

    trace, verdict = S.run(dataclasses.replace(scenario, settings={**scenario.settings, **RECORDED_PLANT}), prepare)
    assert verdict.overshoot > 2.0  # The swing the robot showed: the test can see it
