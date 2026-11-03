"""The process's one Hailo-8 VDevice, shared by every model that runs on the chip.

HailoRT lets a process open the device once: a second VDevice fails with
HAILO_OUT_OF_PHYSICAL_DEVICES while the first is open. So the pose detector and
StereoNet share this one, and its round-robin scheduler switches the chip between
their models. The device opens with its first user and is released after its last,
always after the users' own models, the order HailoRT 4.23 needs.
"""
import contextlib
import threading
from collections.abc import Iterator

_lock = threading.Lock()
_stack: contextlib.ExitStack | None = None
_device = None
_users = 0


@contextlib.contextmanager
def shared_vdevice() -> Iterator[object]:
    """Use the process's VDevice; enter it before, and so leave it after, the models made on it."""
    global _stack, _device, _users
    with _lock:
        if _device is None:
            import hailo_platform as hp

            params = hp.VDevice.create_params()
            params.scheduling_algorithm = hp.HailoSchedulingAlgorithm.ROUND_ROBIN
            _stack = contextlib.ExitStack()
            _device = _stack.enter_context(hp.VDevice(params))
        _users += 1
        device = _device
    try:
        yield device
    finally:
        with _lock:
            _users -= 1
            if _users == 0:
                stack, _stack, _device = _stack, None, None
                stack.close()


def users() -> int:
    """How many users hold the shared device now."""
    return _users
