import asyncio
import os
import socket

import pytest

from ultraskelly import systemd_notify

needs_unix_datagrams = pytest.mark.skipif(not hasattr(socket, "AF_UNIX"), reason="needs Unix datagram sockets")


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

    assert asyncio.run(systemd_notify.run_with_heartbeat(bot())) == "stopped"
    assert sent[:2] == ["nodes built", "READY=1"]
    assert sent[-1] == "STOPPING=1"
    assert set(sent[2:-1]) == {"WATCHDOG=1"}
    assert len(sent[2:-1]) >= 3


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
