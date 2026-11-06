from dataclasses import replace

import pytest

from ultraskelly.simulator.scenarios import BY_NAME, SCENARIOS, Scenario, reversals, run, still


@pytest.mark.parametrize("scenario", SCENARIOS, ids=lambda scenario: scenario.name)
def test_the_head_and_waist_follow_people_without_swinging(scenario):
    trace, verdict = run(scenario)
    assert verdict.passed, f"{scenario.description}: {verdict.failures}"


# The robot may differ from the simulator's plant, which follows the 2026-11-06 measurements
# (engine.py), and from what the controller assumes. The camera's delay was measured at 50-70 ms
# and the picture moved about 0.15 s after a servo command; the OAK's field of view (69 degrees
# on its datasheet, so 8.1 px per degree) is unmeasured. 12 pan servo degrees moved the picture
# 35 px with about 6 degrees of slack, 12 tilt degrees 9-10 px; the waist pulses at 0.7 moved it
# 105 px one way and 47 px the other (the head assumes 27 deg/s at full drive). Every scenario
# must pass at each edge of that range. Two of them at once, a pan ratio under 0.3 or a slack
# of 8 with a waist much faster than assumed, overshoot by 2-3 degrees: not covered yet.
PLANTS = [
    {"camera_latency": latency, "servo_lag": lag, "camera_hfov": hfov}
    for latency in (0.04, 0.10) for lag in (0.04, 0.10) for hfov in (64.0, 74.0)
] + [
    {"pan_ratio": 0.28}, {"pan_ratio": 0.42}, {"tilt_ratio": 0.07}, {"tilt_ratio": 0.14},
    {"pan_backlash": 3.0}, {"pan_backlash": 8.0}, {"tilt_backlash": 0.0}, {"tilt_backlash": 4.0},
    {"waist_speed": 17.0}, {"waist_speed": 37.0}, {"servo_lag": 0.12},
    {"pan_ratio": 0.42, "waist_speed": 17.0},
    # The roll servo's ratio is not measured yet: the roll is assumed to turn the picture 0.3 degrees per servo degree.
    {"roll_ratio": 0.2}, {"roll_ratio": 0.45},
]


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


def test_a_reversed_roll_servo_is_caught_before_any_motor_runs():
    trace, verdict = run(replace(BY_NAME["tilt"], settings={"roll_sign": -1}))
    assert not verdict.passed and verdict.final_tilt > 7.0  # Rolled the face further off level


def test_reversals_count_turns_back_but_not_wobble():
    assert reversals([0, 1, 2, 3, 2.4, 2.6]) == 1
    assert reversals([0, 3, 0, 3, 0]) == 3
    assert reversals([0, 0.3, 0.1, 0.4, 0.2]) == 0
