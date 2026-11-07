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
