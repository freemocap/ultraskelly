"""The whole robot in the simulator, head, waist and roll together, on the plant measured on 2026-11-06
(simulator/plant.py). The robot runs its head alone for now; that setup and its tuning are in
test_scenarios_tuning.py."""
from dataclasses import replace

import pytest

from ultraskelly.simulator.scenarios import BY_NAME, SCENARIOS, Scenario, reversals, run, still

# The waist and roll are not tuned on the measured plant yet, and both are off on the robot: the head
# first, then the waist. Until they are tuned these fail; strictly, so that a pass shows. The waist starts
# turning only above 0.65 of full drive, at 50 deg/s one way and 21.5 the other, and a person 20 degrees
# off is beyond the head's reach inside its servo window (about 16.6 degrees). The roll reaches about
# 1.3 degrees of picture and stops within its 1-degree deadband of where it aims, so a noisy face can end
# just short of the degree nearer level the judge asks for.
WAIST_UNTUNED = "beyond the head's reach, and the waist is not tuned on the measured plant yet"
FAILING_NOW = {"left": WAIST_UNTUNED, "right": WAIST_UNTUNED, "far": WAIST_UNTUNED}


def marked(scenario: Scenario):
    if scenario.name in FAILING_NOW:
        return pytest.param(scenario, marks=pytest.mark.xfail(strict=True, reason=FAILING_NOW[scenario.name]))
    return scenario


@pytest.mark.parametrize("scenario", [marked(scenario) for scenario in SCENARIOS], ids=lambda scenario: scenario.name)
def test_the_head_and_waist_follow_people_without_swinging(scenario):
    trace, verdict = run(scenario)
    assert verdict.passed, f"{scenario.description}: {verdict.failures}"


# The roll's fitted ratio, 0.063, varied from 0.050 to 0.083 between moves, with about 12 degrees of slack;
# its nod (0.18 degrees of tilt per servo degree) is a rough estimate. At either end the roll must stay calm,
# never turning back and forth or a face past level, and stand upright again once the face has gone. How far
# it levels a face depends on how far it reaches: under a degree at 0.050 or with 16 degrees of slack. (A nod
# of 0.3 can tip the camera further than tilt reaches inside its window; the head battery checks the aim.)
ROLL_CALM = ("roll oscillation", "roll runaway", "roll overshoot", "roll: the head not upright")
ROLL_PLANTS = [{"roll_ratio": 0.050}, {"roll_ratio": 0.083}, {"roll_backlash": 8.0}, {"roll_backlash": 16.0},
               {"roll_nod": 0.0}, {"roll_nod": 0.3}]
ROLL_SCENARIOS = [scenario for scenario in SCENARIOS
                  if (scenario.roll or scenario.upright_at_end) and scenario.name not in FAILING_NOW]


@pytest.mark.parametrize("plant", ROLL_PLANTS, ids=lambda plant: ",".join(f"{key}={value}" for key, value in plant.items()))
def test_the_roll_stays_calm_on_a_roll_that_differs_from_its_fit(plant):
    failing = []
    for scenario in ROLL_SCENARIOS:
        trace, verdict = run(replace(scenario, settings={**scenario.settings, **plant}))
        failing += [f"{scenario.name}: {failure}" for failure in verdict.failures if failure.startswith(ROLL_CALM)]
    assert not failing


def test_a_reversed_pan_servo_is_caught_before_any_motor_runs():
    reversed_pan = Scenario("reversed-pan", "The pan servo turns the camera the wrong way", 4.0, still(20.0),
                            settings={"pan_sign": -1})
    trace, verdict = run(reversed_pan)
    assert verdict.runaway and not verdict.passed


def test_a_reversed_roll_servo_is_caught_before_any_motor_runs():
    trace, verdict = run(replace(BY_NAME["tilt"], settings={"roll_sign": 1}))  # The robot's turns the other way
    assert not verdict.passed and verdict.final_tilt > 7.0  # Rolled the face further off level


def test_reversals_count_turns_back_but_not_wobble():
    assert reversals([0, 1, 2, 3, 2.4, 2.6]) == 1
    assert reversals([0, 3, 0, 3, 0]) == 3
    assert reversals([0, 0.3, 0.1, 0.4, 0.2]) == 0
