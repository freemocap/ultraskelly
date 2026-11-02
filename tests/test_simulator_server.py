import json
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from ultraskelly.simulator.__main__ import Runtime, make_server


@pytest.fixture
def local_server():
    runtime = Runtime()
    server = make_server(runtime, 0)
    runtime.thread.start()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f'http://127.0.0.1:{server.server_port}', runtime
    server.shutdown()
    server.server_close()
    thread.join(timeout=3)
    runtime.close()


def call(base, payload=None, origin=None):
    req = Request(base + ('/api/control' if payload is not None else '/api/state'))
    if payload is not None:
        req.data = json.dumps(payload).encode()
        req.add_header('Content-Type', 'application/json')
    if origin:
        req.add_header('Origin', origin)
    with urlopen(req, timeout=3) as response:
        return json.load(response)


def test_http_controls_pause_step_reset_and_tuning(local_server):
    base, _ = local_server
    assert call(base)['tick'] == 0
    state = call(base, {'action': 'step'})
    assert state['tick'] == 1 and not state['running']
    assert call(base, {'action': 'configure', 'settings': {'head_gain': 0.08}})['settings']['head_gain'] == 0.08
    call(base, {'action': 'run'})
    deadline = time.monotonic() + 3
    while call(base)['tick'] < 5 and time.monotonic() < deadline:
        time.sleep(.02)
    paused = call(base, {'action': 'pause'})
    assert paused['tick'] >= 5
    time.sleep(.05)
    assert call(base)['tick'] == paused['tick']
    reset = call(base, {'action': 'reset'})
    assert reset['tick'] == 0 and reset['settings']['head_gain'] == .08


def test_http_rejects_invalid_and_cross_origin_commands(local_server):
    base, _ = local_server
    for payload in [{'action': 'oops'}, {'action': 'configure', 'settings': {'sensor_hz': 0}}, {'action': 'step', 'extra': 1}]:
        with pytest.raises(HTTPError) as exc:
            call(base, payload)
        assert exc.value.code == 400
    with pytest.raises(HTTPError) as exc:
        call(base, {'action': 'run'}, origin='https://example.com')
    assert exc.value.code == 403
    assert call(base)['tick'] == 0


def test_all_viewer_assets_are_served_locally(local_server):
    base, _ = local_server
    for path in ['/', '/style.css', '/viewer.js', '/vendor/three.module.js', '/vendor/three.core.js', '/vendor/OrbitControls.js']:
        with urlopen(base + path, timeout=3) as response:
            assert response.status == 200
            assert len(response.read()) > 100
    with pytest.raises(HTTPError) as exc:
        urlopen(base + '/AGENTS.md', timeout=3)
    assert exc.value.code == 404


@pytest.mark.parametrize('action', ['run', 'step'])
def test_controller_failure_pauses_and_zeroes_outputs(local_server, action):
    base, runtime = local_server
    async def fail():
        runtime.sim.head.waist_motor.throttle = .8
        raise RuntimeError('injected controller fault')
    object.__setattr__(runtime.sim.head, 'step', fail)
    call(base, {'action': action})
    deadline = time.monotonic() + 3
    state = call(base)
    while not state['error'] and time.monotonic() < deadline:
        time.sleep(.02)
        state = call(base)
    assert state['error'] == 'injected controller fault'
    assert not state['running']
    assert state['state']['waist_throttle'] == state['state']['mouth_throttle'] == 0
    assert call(base, {'action': 'reset'})['error'] is None
