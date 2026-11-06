from dataclasses import replace

import pytest

from ultraskelly.simulator.scenarios import SCENARIOS, Scenario, reversals, run, still


@pytest.mark.parametrize("scenario", SCENARIOS, ids=lambda scenario: scenario.name)
def test_the_head_and_waist_follow_people_without_swinging(scenario):
    trace, verdict = run(scenario)
    assert verdict.passed, f"{scenario.description}: {verdict.failures}"


# The robot may differ from what the controller assumes. The camera's delay was measured at
# 50-70 ms; the servos' lag and the OAK's field of view (69 degrees on its datasheet, so
# 8.1 px per degree) are unmeasured; the waist was timed at 18-23 deg/s at 0.75 drive, and
# the head assumes 27 deg/s at full drive. Every scenario must pass at the corners of that range.
PLANTS = [
    {"camera_latency": latency, "servo_lag": lag, "camera_hfov": hfov}
    for latency in (0.04, 0.10) for lag in (0.02, 0.08) for hfov in (64.0, 74.0)
] + [{"waist_speed": 24.0}, {"waist_speed": 33.0}]


@pytest.mark.parametrize("plant", PLANTS, ids=lambda plant: ",".join(f"{key}={value}" for key, value in plant.items()))
def test_the_head_stays_calm_on_a_robot_that_differs_from_its_estimates(plant):
    failing = []
    for scenario in SCENARIOS:
        trace, verdict = run(replace(scenario, settings={**scenario.settings, **plant}))
        if not verdict.passed:
            failing.append(f"{scenario.name}: {verdict.failures}")
    assert not failing


def test_a_reversed_pan_servo_is_caught_before_any_motor_runs():
    reversed_pan = Scenario("reversed-pan", "The pan servo turns the camera the wrong way", 4.0, still(20.0),
                            settings={"pan_sign": -1})
    trace, verdict = run(reversed_pan)
    assert verdict.runaway and not verdict.passed


def test_reversals_count_turns_back_but_not_wobble():
    assert reversals([0, 1, 2, 3, 2.4, 2.6]) == 1
    assert reversals([0, 3, 0, 3, 0]) == 3
    assert reversals([0, 0.3, 0.1, 0.4, 0.2]) == 0
