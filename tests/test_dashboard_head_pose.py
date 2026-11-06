"""The dashboard's 3D head pose (ultraskelly/ui/dashboard/headpose.js), run under node: the ZED's depth is drawn out of
the skull's face at every commanded angle and never as height, pan and tilt follow the simulator's convention, roll
turns the head about its nose, and commands at their limits are named. World axes are the scene's: +y up, the skull
facing +z at rest with its left at +x."""
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


def drawn(*commands: dict) -> list[dict]:
    """Where the scene draws the ZED's axes and the face for each command."""
    done = subprocess.run([NODE, str(SCRIPT), str(HEADPOSE), json.dumps(list(commands))],
                          capture_output=True, encoding="utf-8", check=True, timeout=60)  # node writes UTF-8
    return json.loads(done.stdout)


def close(vector, expected):
    assert vector == pytest.approx(expected, abs=1e-9)


def test_a_level_head_draws_depth_straight_out_of_the_face_not_as_height():
    [level] = drawn({"pan": 90, "tilt": 90, "roll": 90})
    close(level["lens"], [0, 0, 1])          # Depth runs out of the face, horizontally
    close(level["imageDown"], [0, -1, 0])    # The image's down is the room's down
    close(level["imageRight"], [-1, 0, 0])   # and its right the skull's right
    close(level["face"], [0, 0, 1])


@pytest.mark.parametrize("pan", [30, 90, 150])
@pytest.mark.parametrize("tilt", [60, 90, 120])
def test_depth_stays_in_front_of_the_face_at_every_commanded_angle(pan, tilt):
    [pose] = drawn({"pan": pan, "tilt": tilt, "roll": 90})
    close(pose["lens"], pose["face"])
    close(pose["imageDown"], [-value for value in pose["top"]])


@pytest.mark.parametrize("pan", [0, 30, 90, 150, 180])
@pytest.mark.parametrize("tilt", [0, 60, 90, 120, 180])
def test_the_face_points_where_the_simulator_points_its_camera(pan, tilt):
    # ultraskelly/simulator/engine.py, Simulation.camera: yaw = pan - 90 (waist 0), pitch = 90 - tilt
    yaw, pitch = math.radians(pan - 90), math.radians(90 - tilt)
    [pose] = drawn({"pan": pan, "tilt": tilt, "roll": 90})
    close(pose["face"], [math.sin(yaw) * math.cos(pitch), math.sin(pitch), math.cos(yaw) * math.cos(pitch)])


def test_pan_above_90_turns_the_head_to_its_left_and_tilt_above_90_nods_it_down():
    left, right, down, up = drawn({"pan": 150, "tilt": 90}, {"pan": 30, "tilt": 90}, {"pan": 90, "tilt": 120},
                                  {"pan": 90, "tilt": 60})
    assert left["face"][0] > 0.8 and right["face"][0] < -0.8  # +x is the skull's left at rest
    assert down["face"][1] == pytest.approx(-0.5) and up["face"][1] == pytest.approx(0.5)


def test_roll_turns_the_head_about_its_nose():
    [rolled] = drawn({"pan": 90, "tilt": 90, "roll": 120})
    close(rolled["face"], [0, 0, 1])
    close(rolled["lens"], [0, 0, 1])
    close(rolled["top"], [0.5, math.sqrt(3) / 2, 0])  # Toward its left shoulder
    close(rolled["imageDown"], [-0.5, -math.sqrt(3) / 2, 0])


def test_commands_at_their_limits_are_named():
    ahead, nodded, both = drawn({"pan": 90, "tilt": 90}, {"pan": 90, "tilt": 180}, {"pan": 0, "tilt": 0})
    assert ahead["note"] is None
    assert nodded["note"] == "Tilt commanded to its 180° limit: drawn nodded fully down"
    assert both["note"] == ("Pan commanded to its 0° limit: drawn turned fully to its right · "
                            "Tilt commanded to its 0° limit: drawn tipped fully up")
