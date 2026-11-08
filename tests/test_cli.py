"""The command line's safe default and its option checks; none of these start the hardware."""
import subprocess
import sys


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-m", "ultraskelly", *args], capture_output=True, text=True, timeout=30)


def test_without_options_it_prints_help_and_starts_nothing():
    result = run()
    assert result.returncode == 0 and "--hardware" in result.stdout


def test_hardware_options_go_with_hardware():
    result = run("--simulate-motors")
    assert result.returncode == 2 and "--hardware" in result.stderr


def test_follow_and_no_follow_go_with_hardware():
    for option in ("--follow", "--no-follow"):
        result = run(option)
        assert result.returncode == 2 and "--follow or --no-follow" in result.stderr


def test_follow_axes_go_with_hardware_and_name_known_joints():
    from ultraskelly.__main__ import follow_axes

    result = run("--follow-axes=pan,waist")
    assert result.returncode == 2 and "--follow-axes" in result.stderr and "--hardware" in result.stderr
    result = run("--hardware", "--follow-axes=pan,elbow")
    assert result.returncode == 2 and "name joints from pan, tilt, roll, waist" in result.stderr
    assert follow_axes("pan,tilt") == ("pan", "tilt") and follow_axes("none") == ()
