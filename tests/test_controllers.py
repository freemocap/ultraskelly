import asyncio

import pytest

from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.pubsub.bot_topics import ServoStateTopic, TargetLocationMessage, TargetLocationTopic
from ultraskelly.replay import create_controllers, replay


async def follow(bus, node, x, y, seconds):
    """Step the head on a fake clock for `seconds`, the target at pixel (x, y) in a fresh frame every 30 ms."""
    now = [0.0]
    node.clock = lambda: now[0]
    for tick in range(round(seconds * 100)):
        if tick % 3 == 0:
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=x, y=y, captured=now[0]))
        await node.step()
        now[0] += 0.01


def test_a_centred_target_locks_without_moving_the_head():
    async def scenario():
        bus, node, _ = create_controllers()
        states = bus.get_subscription(ServoStateTopic)
        await follow(bus, node, 320, 240, 1.0)
        state = await states.get()
        assert state.is_locked_x and state.is_locked_y
        assert node.pan_angle == node.tilt_angle == 90
        bus.close()
    asyncio.run(scenario())


def test_a_target_held_at_the_frame_corner_turns_the_head_to_the_edge_of_its_range_and_no_further():
    async def scenario():
        bus, node, _ = create_controllers()
        aim = node.params.aim  # The head keeps end_margin inside its servo windows
        await follow(bus, node, 640, 0, 6.0)
        assert node.pan_angle == pytest.approx(aim.pan_range[1] - aim.end_margin)
        assert node.tilt_angle == pytest.approx(aim.tilt_range[0] + aim.end_margin, abs=1e-3)  # The goal halves its way down
        assert node.servo_kit.servo[node.params.pan_channel].angle == pytest.approx(aim.pan_range[1] - aim.end_margin)
        bus.close()
    asyncio.run(scenario())


def test_waist_drives_continuously_while_the_head_is_turned():
    async def scenario():
        bus, node, _ = create_controllers()
        node.params.waist.degrees_per_second = 0.0  # The head holds still instead of counter-rotating
        now = [100.0]
        node.clock = lambda: now[0]
        node.pan_angle = 152  # 16 degrees of view, inside pan's window: the target ahead is past where the waist starts
        commands = []
        for tick in range(300):  # Three seconds; the old controller braked after half of one
            if tick % 10 == 0:
                await bus.publish(TargetLocationTopic, TargetLocationMessage(x=320, y=240))
            await node.step()
            commands.append(node.waist_motor.throttle)
            now[0] += 0.01
        moving = [index for index, command in enumerate(commands) if command]
        assert moving[0] <= 10 * node.params.aim.jump_samples  # Once the target has shown up that many times in a row
        move = commands[moving[0]:]
        assert all(-0.8 <= command < 0 for command in move)  # One continuous move
        assert move[0] == -node.params.waist.start_throttle
        # The head never comes back here, so the drive rises to its breakaway ceiling.
        assert move[-1] == pytest.approx(-node.params.waist.breakaway_throttle)
        bus.close()
    asyncio.run(scenario())


def test_cancellation_stops_both_motors():
    async def scenario():
        bus, orientation, mouth = create_controllers()
        tasks = [asyncio.create_task(node.run()) for node in (orientation, mouth)]
        await asyncio.sleep(0.02)
        orientation.waist_motor.throttle = 0.8
        mouth.mouth_motor.throttle = -1
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        assert orientation.waist_motor.throttle == mouth.mouth_motor.throttle == 0
        bus.close()
    asyncio.run(scenario())


def test_target_replay_exercises_production_coroutines():
    async def scenario():
        states = [state async for state in replay(40, 0.05)]
        assert len(states) == 40
        assert min(s["pan_degrees"] for s in states) < 90
        assert all(0 <= s["pan_degrees"] <= 180 for s in states)
        assert all(abs(s["waist_throttle"] or 0.0) <= 0.8 for s in states)
        assert states[-1]["target_x"] is None
        assert states[-1]["waist_throttle"] in (None, 0.0)  # Ramped down; coasting or braked
    asyncio.run(scenario())


def test_head_servo_channels_match_the_wiring_and_roll_holds_level():
    # Found 2026-11-06 by wiggling one channel at a time with the user watching.
    bus, node, _ = create_controllers()
    assert node.params.servo_names == {3: "pan", 11: "tilt", 7: "roll"}
    assert node.servo_kit.servo[7].angle == 90.0
    asyncio.run(follow(bus, node, 640, 0, 6.0))
    aim = node.params.aim
    angles = (node.servo_kit.servo[3].angle, node.servo_kit.servo[11].angle, node.servo_kit.servo[7].angle)
    assert angles == pytest.approx((aim.pan_range[1] - aim.end_margin, aim.tilt_range[0] + aim.end_margin, 90.0), abs=1e-3)
    bus.close()


def test_without_follow_the_head_and_waist_hold_still_for_a_target():
    async def scenario():
        bus, node, _ = create_controllers()
        node.set_following(False)
        for _ in range(20):
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=640, y=0))
            await node.step()
        assert (node.pan_angle, node.tilt_angle) == (90.0, 90.0)
        assert node.waist_motor.throttle in (None, 0.0)
        bus.close()
    asyncio.run(scenario())


def test_following_switched_off_holds_the_head_brakes_the_waist_and_comes_back_on():
    async def scenario():
        bus, node, _ = create_controllers()
        now = [0.0]
        node.clock = lambda: now[0]

        async def person_at(x, seconds):
            for tick in range(round(seconds * 100)):
                if tick % 3 == 0:
                    await bus.publish(TargetLocationTopic, TargetLocationMessage(x=x, y=240, captured=now[0]))
                await node.step()
                now[0] += 0.01

        await person_at(600, 1.0)  # Off to one side: the head turns toward them
        turned = (node.pan_angle, node.tilt_angle)
        assert turned[0] != 90.0
        node.waist_motor.throttle = 0.3  # As if the waist were turning
        node.set_following(False)  # The control panel's switch
        assert node.waist_motor.throttle == 0.0  # Braked at once, not at the next step
        await person_at(40, 1.0)  # Someone on the other side: nothing follows
        assert (node.pan_angle, node.tilt_angle) == turned and node.waist_motor.throttle == 0.0
        node.set_following(True)
        await person_at(40, 1.0)
        assert node.pan_angle != turned[0]  # Following again
        bus.close()
    asyncio.run(scenario())


def test_the_robot_launches_with_the_calmer_tuning():
    # 2026-11-06, the user: "turn down the aggressiveness". Gain 0.05 overshot 84 px on the robot.
    from ultraskelly.__main__bot import launch_config

    head = launch_config().orientation
    assert head.follow
    assert head.waist.enabled and head.roll.enabled  # Both back on (the owner, 2026-11-07)
    launched, defaults = head.model_dump(), OrientationNodeParams().model_dump()
    # The owner, 2026-11-07: "decrease max pan servo speed" and "tighten that range" (the centring dead zone). The
    # head battery checks these launched values on the measured plant (tests/test_scenarios_tuning.py).
    assert (head.aim.pan_servo_speed, head.aim.deadband) == (30.0, 0.12)
    for key in ("pan_servo_speed", "deadband"):
        launched["aim"][key] = defaults["aim"][key]
    assert launched == defaults  # The defaults the closed-loop scenarios check
    # Tuned on the measured plant (simulator/tuning.py, 2026-11-06) to reach 0.30 and 15 deg/s, inside these
    # bounds: the head battery passes calm with pan geared 1.5 and 2 times the fit (worst overshoot 0.51 degrees).
    assert head.aim.reach <= 0.35 and head.aim.max_speed <= 15.0  # 1 - reach * g >= 0 for g up to 1 / 0.35 = 2.86
    # Following uses the servos' whole measured travel (scripts/check_servo_travel.py, 2026-11-06), as the owner asked
    # on 2026-11-07; the hard limits are the servos' own 0-180, so motor tests reach every angle.
    assert head.aim.pan_range == (10.0, 170.0) and head.aim.tilt_range == (10.0, 180.0)  # Tilt all the way down
    limits = launch_config().limits
    assert (limits.pan_min_degrees, limits.pan_max_degrees, limits.tilt_min_degrees, limits.tilt_max_degrees) == (
        0.0, 180.0, 0.0, 180.0)
    assert head.waist.max_throttle <= 0.6 and head.waist.breakaway_throttle <= 0.7


def test_with_the_waist_off_following_never_drives_it():
    async def scenario():
        bus, node, _ = create_controllers()
        node.params.waist.enabled = False
        node.pan_angle = 160  # Far round: with the waist on this starts a move
        for tick in range(100):
            if tick % 10 == 0:
                await bus.publish(TargetLocationTopic, TargetLocationMessage(x=600, y=240))
            await node.step()
            assert node.waist_motor.throttle == 0.0
        bus.close()
    asyncio.run(scenario())


def test_the_head_never_commands_outside_its_servo_ranges():
    from ultraskelly.core.bot.motor.head_aim import HeadAim, HeadAimParams

    aim = HeadAim(HeadAimParams(pan_range=(20.0, 160.0), tilt_range=(20.0, 130.0)), 90.0, 90.0)
    commands = []
    for tick in range(600):  # 6 s of a person far right and far below, then far left and far above
        now = tick * 0.01
        error = (600.0, 400.0) if tick < 300 else (-600.0, -400.0)
        if tick % 3 == 0:
            aim.observe(now=now, error_x=error[0], error_y=error[1], captured=now)
        commands.append(aim.update(now=now))
    pans, tilts = [c[0] for c in commands], [c[1] for c in commands]
    assert 20.0 <= min(pans) and max(pans) <= 160.0 and 20.0 <= min(tilts) and max(tilts) <= 130.0
    assert max(tilts) > 120.0 and min(pans) < 30.0  # It does use its range


def test_a_jump_must_hold_over_consecutive_frames_not_add_up_from_bursts():
    from ultraskelly.core.bot.motor.head_aim import HeadAim, HeadAimParams

    def believed(seen: list[float]) -> bool:
        """Whether the head takes up a target by the picture's edge, seen at these times."""
        aim = HeadAim(HeadAimParams(jump_samples=5), 90.0, 90.0)
        for now in seen:
            aim.observe(now=now, error_x=300.0, error_y=0.0, captured=now)
        return aim.goal is not None

    frame = 1 / 30
    bursts = [start + n * frame for start in (0.0, 0.7, 1.4, 2.1) for n in range(3)]
    assert not believed(bursts)  # 12 sightings, 3 at a time: the robot's false detections (run 20261106-194104)
    assert believed([n * frame for n in range(12)])  # As many back to back: someone is there
    assert believed([n * frame for n in range(12) if n % 3 != 2])  # Someone whose every third detection drops


def test_following_tilts_all_the_way_down_to_the_servos_own_end():
    # 2026-11-07, the owner: "the face track doesnt go ALL the way down for tilt ... i click the 'all the way down
    # tilt' button, it goes down a bit more, then snaps up again". Following stopped at 135: a range to 140 less a
    # 5 degree margin. The servo's own end needs no margin.
    async def scenario():
        bus, node, _ = create_controllers()
        assert node.params.aim.tilt_range[1] == 180.0
        await follow(bus, node, 320, 470, 12.0)  # A face at the bottom of the picture, for as long as it takes
        assert node.tilt_angle == pytest.approx(180.0, abs=0.5)
        assert node.servo_kit.servo[node.params.tilt_channel].angle == pytest.approx(180.0, abs=0.5)
        bus.close()
    asyncio.run(scenario())


def test_an_axis_held_at_its_end_while_following_is_reported_as_unable_to_reach():
    from ultraskelly.core.bot.health import Status, head_finding

    async def scenario():
        bus, node, _ = create_controllers()
        await follow(bus, node, 320, 470, 8.0)  # A face at the bottom of the picture: tilt runs to its end and stays
        pinned = node.pinned(node.clock())
        assert pinned.get("tilt", 0.0) > 5.0 and "pan" not in pinned
        finding = head_finding(follow=True, pinned=pinned)
        assert finding.status == Status.DEGRADED and "Tilt" in finding.message
        await follow(bus, node, 320, 240, 1.0)  # Centred, though the head still sits at the end: it reached them
        assert node.tilt_angle >= node.params.aim.tilt_range[1] - node.params.aim.end_margin - 0.5
        assert node.pinned(node.clock()) == {}
        bus.close()

    asyncio.run(scenario())
    assert head_finding(follow=False, pinned={}).status == Status.OFF
    assert head_finding(follow=True, pinned={"pan": 2.0}).status == Status.OK  # Briefly at the end is fine
