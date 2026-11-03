"""The command line's window options; none of these start the hardware."""
import subprocess
import sys


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-m", "ultraskelly", *args], capture_output=True, text=True, timeout=30)


def test_the_window_is_opt_in_and_goes_with_hardware():
    result = run("--window")
    assert result.returncode == 2 and "--window" in result.stderr


def test_no_window_is_still_accepted_for_older_commands_and_hidden_from_help():
    result = run("--no-window")
    assert result.returncode == 0 and "--window" in result.stdout and "--no-window" not in result.stdout


def test_window_and_no_window_together_are_refused_before_anything_starts():
    result = run("--hardware", "--window", "--no-window")
    assert result.returncode == 2 and "contradict" in result.stderr
