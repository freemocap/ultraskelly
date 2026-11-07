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
