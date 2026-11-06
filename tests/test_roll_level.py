import asyncio
import math

import numpy as np
import pytest

from ultraskelly.core.bot.motor.roll_level import LEVEL, RollLevel, RollLevelParams, choose_face, eye_line
from ultraskelly.core.pubsub.bot_topics import (
    FaceObservation,
    OakResultsMessage,
    OakResultsTopic,
    ServoStateTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.replay import create_controllers


def face(tilt: float = 0.0, centre=(320.0, 240.0), eyes: float = 40.0, score: float = 0.9, yaw: float | None = None):
    """A face whose eye line turns `tilt` degrees clockwise on screen, the person's right eye on the picture's left."""
    across = np.array([math.cos(math.radians(tilt)), math.sin(math.radians(tilt))]) * eyes / 2
    middle = np.array(centre)
    landmarks = np.array([middle - across, middle + across, middle + [0, 20], middle + [-15, 40], middle + [15, 40]])
    box = (centre[0] - 50, centre[1] - 60, centre[0] + 50, centre[1] + 70)
    return FaceObservation(box=box, score=score, landmarks=landmarks,
                           head_pose=None if yaw is None else (yaw, 0.0, 0.0))


def test_the_eye_line_is_level_for_an_upright_face_and_clockwise_positive():
    assert eye_line(face(0.0)) == pytest.approx(0.0)
    assert eye_line(face(10.0)) == pytest.approx(10.0)  # Left eye lower on screen: turned clockwise
    assert eye_line(face(-25.0)) == pytest.approx(-25.0)


def test_the_followed_face_is_the_one_around_the_target():
    near, far = face(centre=(200, 200)), face(centre=(450, 220))
    assert choose_face([near, far], (452, 230)) is far
    assert choose_face([near, far], (205, 190)) is near
    assert choose_face([near, far], None) is None
    assert choose_face([near], (600, 460)) is None  # Nobody's face near the person followed


def test_doubtful_faces_are_not_used():
    level = RollLevel(RollLevelParams())
    assert level.usable(face(10.0)) == pytest.approx(10.0)
    assert level.usable(face(10.0, score=0.3)) is None
    assert level.usable(face(10.0, eyes=6.0)) is None  # Too small to measure an angle on
    assert level.usable(face(60.0)) is None  # Steeper than any head tilts: a bad detection
    assert level.usable(face(10.0, yaw=70.0)) is None  # Turned away from the camera


def drive(level: RollLevel, tilt_of, seconds: float, ratio: float = 0.3, delay: float = 0.08):
    """Run the roll against a face tilted `tilt_of(t)` in the world, seen through a camera rolled by the commands
    `delay` seconds old at `ratio` degrees per servo degree; returns the face's tilt in the picture over time."""
    commands, seen = [], []
    for tick in range(round(seconds * 100)):
        now = tick * 0.01
        commands.append((now, level.servo))
        rolled = next((ratio * (servo - LEVEL) for at, servo in reversed(commands) if at <= now - delay), 0.0)
        picture = tilt_of(now) - rolled
        if tick % 3 == 0 and tilt_of(now) is not None:
            level.observe(now=now, face=face(picture), captured=now)
        level.update(now=now)
        seen.append(picture)
    return seen


def test_a_tilted_face_is_levelled_without_overshoot():
    level = RollLevel(RollLevelParams())
    seen = drive(level, lambda t: 7.0, 4.0)  # Within the roll's reach: 0.3 x 25 degrees
    assert level.servo > LEVEL  # Clockwise in the picture: a larger roll angle turns it back
    assert abs(seen[-1]) <= level.params.deadband / level.params.reach + 0.5
    assert min(seen) > -1.0  # Never turned past level


def test_a_slight_tilt_is_left_alone():
    level = RollLevel(RollLevelParams())
    drive(level, lambda t: 2.0, 2.0)
    assert level.servo == LEVEL


def test_without_a_face_the_head_eases_back_upright():
    level = RollLevel(RollLevelParams())
    drive(level, lambda t: 12.0, 3.0)
    assert level.servo > LEVEL + 5
    for tick in range(300):  # No faces for 3 s
        level.update(now=3.0 + tick * 0.01)
    assert level.servo == LEVEL and level.tilt is None


def test_the_roll_stays_within_its_span():
    level = RollLevel(RollLevelParams())
    drive(level, lambda t: 35.0, 6.0)  # More than the roll can take up
    assert LEVEL + level.params.span - 1.0 < level.servo <= LEVEL + level.params.span


def faces_message(tilt: float, x: float = 320, y: float = 240) -> OakResultsMessage:
    return OakResultsMessage(faces=[face(tilt, centre=(x, y))], image_size=(640, 480), timestamp=None)


def test_the_orientation_node_rolls_toward_the_followed_face_and_reports_the_command():
    async def scenario():
        bus, node, _ = create_controllers()
        states = bus.get_subscription(ServoStateTopic)
        now = [0.0]
        node.clock = lambda: now[0]
        for tick in range(200):
            if tick % 3 == 0:
                await bus.publish(TargetLocationTopic, TargetLocationMessage(x=320, y=240, captured=now[0]))
                await bus.publish(OakResultsTopic, faces_message(15.0))
            await node.step()
            now[0] += 0.01
        state = await states.get()
        assert node.roll_angle > LEVEL
        assert node.servo_kit.servo[node.params.roll_channel].angle == node.roll_angle == state.roll_angle
        bus.close()
    asyncio.run(scenario())


@pytest.mark.parametrize("hold", ["manual", "follow"])
def test_motor_tests_and_no_follow_leave_the_roll_alone(hold):
    async def scenario():
        bus, node, _ = create_controllers()
        if hold == "manual":
            node.manual = True
        else:
            node.params.follow = False
        now = [0.0]
        node.clock = lambda: now[0]
        for tick in range(100):
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=320, y=240, captured=now[0]))
            await bus.publish(OakResultsTopic, faces_message(20.0))
            await node.step()
            now[0] += 0.01
        assert node.roll_angle == LEVEL
        bus.close()
    asyncio.run(scenario())


def test_switched_off_the_roll_holds_upright():
    async def scenario():
        bus, node, _ = create_controllers()
        node.params.roll.enabled = False
        now = [0.0]
        node.clock = lambda: now[0]
        for tick in range(100):
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=320, y=240, captured=now[0]))
            await bus.publish(OakResultsTopic, faces_message(20.0))
            await node.step()
            now[0] += 0.01
        assert node.roll_angle == LEVEL
        bus.close()
    asyncio.run(scenario())


def test_a_target_seen_through_a_rolled_camera_is_turned_back_upright():
    bus, node, _ = create_controllers()
    node.roll_level.history.append((0.0, 90.0))  # The picture's content turned a quarter anticlockwise
    x, y = node._upright(0.0, -100.0, captured=0.2, now=0.3)  # Seen straight above the middle...
    assert (x, y) == pytest.approx((100.0, 0.0), abs=1e-9)  # ...it was to the right of it
    bus.close()
