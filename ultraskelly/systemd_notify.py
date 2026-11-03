"""Tell systemd when the bot is ready and that its event loop is still running.

When the bot runs as the boot service (`Type=notify` with `WatchdogSec=`), systemd
waits for READY=1 and then expects WATCHDOG=1 within every watchdog timeout. A task on
the bot's event loop sends those pings, so they stop whenever the loop stops: a
coroutine that blocks, a driver call that never returns, a deadlock. systemd then
kills the bot with SIGUSR1 (`WatchdogSignal=` in the unit), which first prints every
thread's Python traceback to the journal, and starts it again. Not SIGABRT: depthai
installs its own handler for that, which prints only native frames. The pings also
stop once the bot starts shutting down, so a shutdown stuck on a thread meets the same
end instead of hanging; no STOPPING=1 is sent, as it would switch the watchdog off.
Manual runs have no NOTIFY_SOCKET, so nothing is sent.
"""
import asyncio
import faulthandler
import logging
import os
import signal
import socket
from collections.abc import Awaitable, Callable
from typing import TypeVar

logger = logging.getLogger(__name__)
T = TypeVar("T")


def notify(state: str) -> bool:
    """Send systemd one notification such as "READY=1"; False when there is no service manager to tell."""
    address = os.environ.get("NOTIFY_SOCKET")
    if not address:
        return False
    if address.startswith("@"):  # Linux abstract socket name
        address = "\0" + address[1:]
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.sendto(state.encode(), address)
    except OSError as error:
        logger.warning(f"Could not notify systemd of {state}: {error}")
        return False
    return True


def watchdog_seconds() -> float | None:
    """The watchdog timeout systemd expects this process to keep, or None when it has none."""
    usec = os.environ.get("WATCHDOG_USEC")
    pid = os.environ.get("WATCHDOG_PID")
    if not usec or (pid and int(pid) != os.getpid()):
        return None
    return int(usec) / 1e6


async def _heartbeat(healthy: Callable[[], bool]) -> None:
    if not notify("READY=1"):
        return
    timeout = watchdog_seconds()
    if timeout is None:
        return
    if hasattr(signal, "SIGUSR1"):  # On the watchdog's signal, print all threads' tracebacks, then die of it
        faulthandler.register(signal.SIGUSR1, all_threads=True, chain=True)
    logger.info(f"systemd watchdog: {timeout:g} s; pinging every {timeout / 2:g} s")
    while healthy():
        notify("WATCHDOG=1")
        await asyncio.sleep(timeout / 2)
    logger.info(f"Watchdog pings stopped for the shutdown: systemd ends the bot if it lasts over {timeout:g} s")


async def run_with_heartbeat(work: Awaitable[T], *, healthy: Callable[[], bool] = lambda: True) -> T:
    """Await `work`, reporting ready once it first lets the event loop run, then alive while
    the loop runs and `healthy()` holds; the bot's goes false once it starts shutting down."""
    heartbeat = asyncio.create_task(_heartbeat(healthy))
    try:
        return await work
    finally:
        heartbeat.cancel()
