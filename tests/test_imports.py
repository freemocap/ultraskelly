import subprocess
import sys


def test_controller_imports_do_not_load_drivers_or_configure_logging():
    code = """
import logging, sys
before = list(logging.getLogger().handlers)
from ultraskelly.replay import create_controllers
assert logging.getLogger().handlers == before
assert not any(name.startswith(('adafruit_', 'picamera2', 'libcamera')) for name in sys.modules)
"""
    subprocess.run([sys.executable, "-c", code], check=True, timeout=10)
