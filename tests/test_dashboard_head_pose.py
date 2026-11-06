"""The dashboard's 3D head pose (ultraskelly/ui/dashboard/headpose.js), run under node: the ZED's depth stays rigid in
the head, drawn out of the skull's face at every commanded angle and never as height; the head turns the way and as
far as measured on the robot (scripts/check_motor_directions.py, 2026-11-06); roll turns it about its nose; commands
at their limits are named. World axes are the scene's: +y up, the skull facing +z at rest with its left at +x."""
import json
import math
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
HEADPOSE = ROOT / "ultraskelly" / "ui" / "dashboard" / "headpose.js"
SCRIPT = ROOT / "tests" / "js" / "head_pose.mjs"
NODE = shutil.which("node")
pytestmark = pytest.mark.skipif(NODE is None, reason="needs node to run the dashboard's JavaScript")


def run(*commands: dict) -> dict:
    done = subprocess.run([NODE, str(SCRIPT), str(HEADPOSE), json.dumps(list(commands))],
                          capture_output=True, encoding="utf-8", check=True, timeout=60)  # node writes UTF-8
    return json.loads(done.stdout)


def drawn(*commands: dict) -> list[dict]:
    """Where the scene draws the ZED's axes and the face for each command."""
    return run(*commands)["cases"]


def close(vector, expected):
    assert vector == pytest.approx(expected, abs=1e-9)


def facing(left_degrees: float, up_degrees: float) -> list[float]:
    """The face's direction turned this far to the skull's left and up."""
    yaw, pitch = math.radians(left_degrees), math.radians(up_degrees)
    return [math.sin(yaw) * math.cos(pitch), math.sin(pitch), math.cos(yaw) * math.cos(pitch)]


def test_a_level_head_draws_depth_straight_out_of_the_face_not_as_height():
    [level] = drawn({"pan": 90, "tilt": 90, "roll": 90})
    close(level["lens"], [0, 0, 1])          # Depth runs out of the face, horizontally
    close(level["imageDown"], [0, -1, 0])    # The image's down is the room's down
    close(level["imageRight"], [-1, 0, 0])   # and its right the skull's right
    close(level["face"], [0, 0, 1])


@pytest.mark.parametrize("pan", [0, 30, 90, 150, 180])
@pytest.mark.parametrize("tilt", [0, 60, 90, 120, 180])
def test_depth_stays_rigid_in_front_of_the_face_at_every_commanded_angle(pan, tilt):
    [pose] = drawn({"pan": pan, "tilt": tilt, "roll": 90})
    close(pose["lens"], pose["face"])
    close(pose["imageDown"], [-value for value in pose["top"]])


@pytest.mark.parametrize("pan, tilt, left, up", [
    (150, 90, -12, 0),  # More pan turns the head to its own right, 0.2 degrees per servo degree
    (30, 90, 12, 0),
    (180, 90, -18, 0),  # Even the end of the servo's range turns the head only 18 degrees
    (90, 120, 0, -3),   # More tilt nods it down, 0.1 degrees per servo degree
    (90, 60, 0, 3),
    (90, 180, 0, -9),   # A wound-up tilt draws a 9 degree nod, not the head pointing into the floor
])
def test_the_head_turns_the_way_and_as_far_as_measured_on_the_robot(pan, tilt, left, up):
    [pose] = drawn({"pan": pan, "tilt": tilt, "roll": 90})
    close(pose["face"], facing(left, up))


def test_roll_turns_the_head_about_its_nose():
    [rolled] = drawn({"pan": 90, "tilt": 90, "roll": 120})
    close(rolled["face"], [0, 0, 1])
    close(rolled["lens"], [0, 0, 1])
    close(rolled["top"], [0.5, math.sqrt(3) / 2, 0])  # Toward its left shoulder
    close(rolled["imageDown"], [-0.5, -math.sqrt(3) / 2, 0])


def test_negative_waist_drive_turns_the_body_to_its_right():
    right, left, still = drawn({"waist": -0.7}, {"waist": 0.7}, {"waist": 0.0})
    assert (right["waist"], left["waist"], still["waist"]) == (-1, 1, 0)


def test_head_commands_at_their_limits_are_named():
    ahead, wound, both = drawn({"pan": 90, "tilt": 90}, {"pan": 90, "tilt": 180}, {"pan": 0, "tilt": 0})
    assert ahead["note"] is None
    assert wound["note"] == "Tilt commanded to its 180° limit"
    assert both["note"] == "Pan commanded to its 0° limit · Tilt commanded to its 0° limit"


def test_the_scene_says_its_head_is_approximate():
    assert run()["headNote"] == "Head approximate: 0.2° of turn and 0.1° of nod per servo degree, as measured"
