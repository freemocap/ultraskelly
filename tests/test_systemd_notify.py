import asyncio
import faulthandler
import os
import signal
import socket
import subprocess
import sys

import pytest

from ultraskelly import systemd_notify

needs_unix_datagrams = pytest.mark.skipif(not hasattr(socket, "AF_UNIX"), reason="needs Unix datagram sockets")
needs_sigusr1 = pytest.mark.skipif(not hasattr(signal, "SIGUSR1"), reason="needs POSIX signals")


def test_manual_runs_send_nothing(monkeypatch):
    monkeypatch.delenv("NOTIFY_SOCKET", raising=False)
    assert systemd_notify.notify("READY=1") is False
    assert asyncio.run(systemd_notify.run_with_heartbeat(asyncio.sleep(0, result="done"))) == "done"


def test_only_the_main_process_keeps_the_watchdog(monkeypatch):
    monkeypatch.setenv("WATCHDOG_USEC", "30000000")
    monkeypatch.setenv("WATCHDOG_PID", str(os.getpid()))
    assert systemd_notify.watchdog_seconds() == 30
    monkeypatch.setenv("WATCHDOG_PID", str(os.getpid() + 1))  # Inherited by a child process
    assert systemd_notify.watchdog_seconds() is None
    monkeypatch.delenv("WATCHDOG_USEC")
    assert systemd_notify.watchdog_seconds() is None


def test_ready_follows_start_up_then_pings_continue_until_the_work_ends(monkeypatch):
    sent = []
    monkeypatch.setattr(systemd_notify, "notify", lambda state: sent.append(state) or True)
    monkeypatch.setenv("WATCHDOG_USEC", "100000")  # A 0.1 s timeout: a ping every 0.05 s
    monkeypatch.setenv("WATCHDOG_PID", str(os.getpid()))

    async def bot():
        sent.append("nodes built")  # Like the bot, which opens its hardware before its first await
        await asyncio.sleep(0.3)
        return "stopped"

    try:
        assert asyncio.run(systemd_notify.run_with_heartbeat(bot())) == "stopped"
    finally:
        if hasattr(signal, "SIGUSR1"):
            faulthandler.unregister(signal.SIGUSR1)  # Registered for the watchdog; keep pytest's default
    assert sent[:2] == ["nodes built", "READY=1"]
    assert set(sent[2:]) == {"WATCHDOG=1"}  # No STOPPING=1: it would switch the watchdog off
    assert len(sent[2:]) >= 3


def test_pings_stop_once_the_bot_starts_shutting_down(monkeypatch):
    sent = []
    monkeypatch.setattr(systemd_notify, "notify", lambda state: sent.append(state) or True)
    monkeypatch.setenv("WATCHDOG_USEC", "100000")  # A ping every 0.05 s
    monkeypatch.setenv("WATCHDOG_PID", str(os.getpid()))
    shutting_down = False

    async def bot():
        nonlocal shutting_down
        await asyncio.sleep(0.2)
        shutting_down = True  # As BotLauncher.healthy() turns false when its shutdown starts
        pings = sent.count("WATCHDOG=1")
        await asyncio.sleep(0.3)  # A slow shutdown
        return pings

    try:
        pings_before_shutdown = asyncio.run(
            systemd_notify.run_with_heartbeat(bot(), healthy=lambda: not shutting_down)
        )
    finally:
        if hasattr(signal, "SIGUSR1"):
            faulthandler.unregister(signal.SIGUSR1)
    assert pings_before_shutdown >= 2
    assert sent.count("WATCHDOG=1") <= pings_before_shutdown + 1  # At most one already under way


@needs_sigusr1
def test_the_watchdog_signal_prints_the_frozen_coroutine_and_ends_the_process():
    code = """
import asyncio, os, time
from ultraskelly import systemd_notify
systemd_notify.notify = lambda state: True  # Stand in for systemd
os.environ.update(WATCHDOG_USEC="60000000", WATCHDOG_PID=str(os.getpid()))

async def frozen_bot():
    await asyncio.sleep(0.1)  # The heartbeat starts
    print("frozen", flush=True)
    time.sleep(30)  # A call that blocks the event loop

asyncio.run(systemd_notify.run_with_heartbeat(frozen_bot()))
"""
    bot = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert bot.stdout.readline().strip() == "frozen"
        bot.send_signal(signal.SIGUSR1)  # What systemd sends when the pings stop
        _, stderr = bot.communicate(timeout=10)
    finally:
        bot.kill()
    assert bot.returncode == -signal.SIGUSR1
    assert "in frozen_bot" in stderr


@needs_unix_datagrams
@pytest.mark.parametrize("abstract", [False, True])
def test_notifications_reach_the_service_manager(monkeypatch, tmp_path, abstract):
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as manager:
        if abstract:
            name = f"ultraskelly-test-{os.getpid()}"
            manager.bind("\0" + name)
            monkeypatch.setenv("NOTIFY_SOCKET", "@" + name)
        else:
            manager.bind(str(tmp_path / "notify"))
            monkeypatch.setenv("NOTIFY_SOCKET", str(tmp_path / "notify"))
        assert systemd_notify.notify("READY=1")
        assert manager.recv(64) == b"READY=1"


@needs_unix_datagrams
def test_a_missing_service_manager_is_reported_not_raised(monkeypatch, tmp_path):
    monkeypatch.setenv("NOTIFY_SOCKET", str(tmp_path / "gone"))
    assert systemd_notify.notify("WATCHDOG=1") is False
