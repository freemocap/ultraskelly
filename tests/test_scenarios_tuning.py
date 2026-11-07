"""The robot's head tuning on a plant like the one recorded on 2026-11-06: no swinging past people.

Full servo sweeps on the robot (scripts/check_pan_travel.py, 20:09-20:11) measured 0.27 degrees of view
per pan servo degree and 0.12 per tilt degree where the linkages turn steadily, and run 20261106-194104
showed the picture moving 0.1-0.2 s after a command. Outside the servo windows the robot's linkages jam
or go over centre, which the simulator does not model; the launched head never commands there
(tests/test_controllers.py). STRESS_PLANT is twice the measured pan gearing.
"""
import dataclasses

import pytest

from ultraskelly.__main__bot import launch_config
from ultraskelly.simulator import scenarios as S

RECORDED_PLANT = {"pan_ratio": 0.27, "tilt_ratio": 0.12, "servo_lag": 0.15}
STRESS_PLANT = {"pan_ratio": 0.54, "tilt_ratio": 0.12, "servo_lag": 0.15}


@pytest.mark.parametrize("plant,most", [(RECORDED_PLANT, 0.5), (STRESS_PLANT, S.OVERSHOOT_DEGREES)], ids=["measured", "stress"])
@pytest.mark.parametrize("name", ["left", "right", "noisy", "lost", "walk", "two-people"])
def test_the_launched_head_does_not_swing_past_people_on_the_recorded_plant(name, plant, most):
    aim, scenario = launch_config().orientation.aim, next(s for s in S.SCENARIOS if s.name == name)

    def prepare(sim):
        sim.head.params.waist.enabled = sim.head.params.roll.enabled = False
        for key, value in aim.model_dump().items():  # In place: the head's aim already holds this object
            setattr(sim.head.params.aim, key, value)

    trace, verdict = S.run(dataclasses.replace(scenario, settings={**scenario.settings, **plant}), prepare)
    assert verdict.overshoot <= most, verdict.failures
    assert not [f for f in verdict.failures if f.startswith(("overshoot", "oscillation", "runaway"))]


def test_the_old_gain_does_swing_on_the_stress_plant():
    scenario = next(s for s in S.SCENARIOS if s.name == "left")

    def prepare(sim):
        sim.head.params.waist.enabled = sim.head.params.roll.enabled = False

    trace, verdict = S.run(dataclasses.replace(scenario, settings={**scenario.settings, **STRESS_PLANT}), prepare)
    assert verdict.overshoot > 2.0  # The old gain swings where the new one does not: the test can see it
