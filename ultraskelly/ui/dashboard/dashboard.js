// Live dashboard, laid out as docs/dashboard.md describes. One FreeMoCap stream from /stream drives
// it: `robot_state` messages carry the bot's snapshot (motor commands, target, rates, node health);
// FreeMoCap `frame` messages carry camera JPEGs with the poses found in that very image.
import { TransportService } from '/vendor/freemocap-client.js';
import { MotorTestPanel } from '/motor-test.js';
import { createScene } from '/scene.js';
import { CameraTiles } from '/tiles.js';
import { DepthViews } from '/depth.js';

const $ = id => document.getElementById(id);
const error = message => { $('error').textContent = message; $('error').hidden = !message; };
const fixed = (value, digits, unit = '') => value == null ? '—' : `${value.toFixed(digits)}${unit}`;
const ago = seconds => seconds == null ? '—' : seconds < 1 ? `${Math.round(seconds * 1000)} ms` : `${seconds.toFixed(1)} s`;
const signed = value => `${value >= 0 ? '+' : '−'}${Math.abs(value).toFixed(2)}`;
const hasTarget = state => state?.target && state.target.x != null && state.target.y != null;

// 3D scene. Without WebGL the readouts and video still work.
let scene = null;
try { scene = createScene($('viewport')); }
catch (problem) { console.error(problem); error('The 3D view needs WebGL2. Enable browser graphics acceleration and reload.'); }
const viewButtons = [...document.querySelectorAll('.views button')];
for (const button of viewButtons) button.onclick = () => {
  for (const other of viewButtons) other.setAttribute('aria-pressed', String(other === button));
  scene?.setView(button.dataset.view);
};
for (const button of document.querySelectorAll('.layers button')) button.onclick = () => {
  const on = button.getAttribute('aria-pressed') !== 'true', layer = button.dataset.layer;
  button.setAttribute('aria-pressed', String(on));
  if (layer === 'gaze') scene?.gaze.setVisible(on);
  else scene?.depth.setVisible({[layer]: on});
};

// Camera tiles; any tile enlarges over the 3D scene, which stops rendering while covered.
const tiles = new CameraTiles(document, {element: $('enlarged'), canvas: $('enlarged').querySelector('canvas'), label: $('enlarged-label')});
function enlarge(cameraId) {
  tiles.enlarge(cameraId);
  scene?.setPaused(tiles.enlargedId != null);
}
for (const figure of document.querySelectorAll('.tile[data-camera]')) {
  figure.onclick = () => enlarge(tiles.enlargedId === figure.dataset.camera ? null : figure.dataset.camera);
}
$('enlarged').querySelector('canvas').onclick = () => enlarge(null);
$('enlarged-close').onclick = () => enlarge(null);

// Motor tests: W1's panel opens in a drawer beside the skeleton; the card's Stop works while it is closed.
const motorTest = new MotorTestPanel($('motor-test'));
function openTests(open) {
  $('motor-drawer').hidden = !open;
  $('tests-open').textContent = open ? 'Close' : 'Open';
}
$('tests-open').onclick = () => openTests($('motor-drawer').hidden);
$('motor-drawer-close').onclick = () => openTests(false);
addEventListener('keydown', event => {
  if (event.key !== 'Escape') return;
  if (tiles.enlargedId) enlarge(null);
  else openTests(false);
});
function showTestStatus(text, kind = '') {
  $('tests-status').textContent = text;
  $('tests-status').className = `tests-status ${kind}`;
}
let stopNotice = null;  // The Stop button's answer, shown for a few seconds over the test status
$('tests-stop').onclick = async () => {
  const notice = (text, kind = '') => { stopNotice = {text, kind, until: performance.now() + 3000}; showTestStatus(text, kind); };
  try {
    const reply = await fetch('/api/motor-test/stop', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: '{}'});
    const body = await reply.json();
    if (!reply.ok) notice(`Stop refused: ${body.error}`, 'refused');
    else notice(body.stopped ? 'Stopped' : 'Nothing was running', body.stopped ? 'refused' : '');
  } catch (problem) {
    notice(`No answer from the robot: ${problem.message}`, 'refused');
  }
};
function showTests(state) {
  const test = state.motor_test;
  $('tests-open').hidden = $('tests-stop').hidden = !test;
  if (!test) { $('tests-travel').textContent = ''; showTestStatus('Not available in this run'); return; }
  $('tests-travel').textContent = `waist ${signed(test.waist_travel)}`;
  if (stopNotice && performance.now() < stopNotice.until && !test.running) showTestStatus(stopNotice.text, stopNotice.kind);
  else if (!test.enabled) showTestStatus('Turned off in the launch configuration');
  else if (test.running) showTestStatus(`Running: ${test.running.description}`);
  else if (test.last) {
    const outcome = test.last.outcome;
    showTestStatus(`${outcome[0].toUpperCase()}${outcome.slice(1)}: ${test.last.description}`, outcome === 'done' ? 'done' : 'refused');
  } else showTestStatus('Ready');
}

// Readouts.
function showCommands(state) {
  for (const row of document.querySelectorAll('.motors tr[data-motor]')) {
    const name = row.dataset.motor, command = state.motors?.commands?.[name], value = command?.value ?? null;
    const servo = name === 'pan' || name === 'tilt';
    row.querySelector('.value').textContent = fixed(value, servo ? 1 : 2, servo ? '°' : '');
    row.querySelector('.age').textContent = ago(command?.age);
    const meter = row.querySelector('.meter i');
    meter.style.setProperty('--shown', value == null ? 'none' : 'block');
    if (value == null) continue;
    if (servo) {  // Servo angle across 0 to 180 degrees; the tick marks the 90 degree center
      const at = Math.max(0, Math.min(100, value / 180 * 100));
      meter.style.setProperty('--left', `calc(${at}% - 1.5px)`); meter.style.setProperty('--width', '3px');
    } else {      // Throttle from the center tick toward -1 or +1
      const at = 50 + Math.max(-1, Math.min(1, value)) * 50;
      meter.style.setProperty('--left', `${Math.min(50, at)}%`); meter.style.setProperty('--width', `${Math.abs(at - 50)}%`);
    }
  }
  const mode = $('motor-mode');
  mode.textContent = state.motors?.simulated ? 'Simulated motors' : 'Real motors';
  mode.classList.toggle('real', !state.motors?.simulated);
}

function showTarget(state) {
  const seen = hasTarget(state), target = state.target, servo = state.servo_state;
  $('target-state').textContent = seen ? 'Tracking' : 'No target';
  $('target-state').classList.toggle('tracking', seen);
  $('target-pixel').textContent = seen ? `${target.x}, ${target.y} px` : '—';
  $('target-angle').textContent = seen && target.angle != null ? `body ${target.angle.toFixed(0)}°` : '';
  // The head reports the target inside its jaw zone on both axes: the condition for opening the jaw.
  $('target-lock').textContent = seen && servo ? (servo.locked_x && servo.locked_y ? 'in jaw zone' : 'outside jaw zone') : '';
  $('target-age').textContent = target?.age != null ? ago(target.age) : '';
}

function definitionList(list, rows) {
  list.replaceChildren(...rows.flatMap(([term, value, stale, detail]) => {
    const dt = document.createElement('dt'), dd = document.createElement('dd');
    dt.textContent = term; dd.textContent = value; dd.title = detail ?? value; dd.classList.toggle('stale', Boolean(stale));
    return [dt, dd];
  }));
}

const cameraName = name => ({oak: 'OAK', zed_left: 'ZED L', zed_right: 'ZED R'}[name] ?? name);
function showHealth(state) {
  const labels = {servo_state: 'Servo', target: 'Target', pose: 'Pose'};
  const rows = Object.entries(state.rates ?? {}).map(([name, hz]) => [labels[name] || name, `${hz.toFixed(1)} Hz`]);
  for (const [name, camera] of Object.entries(state.cameras ?? {})) {
    // A camera shows its rate; once its newest frame is over a second old, it shows that age instead.
    const stale = !camera.width || camera.age > 1;
    const detail = camera.width ? `${camera.width}×${camera.height}, ${camera.fps.toFixed(1)} fps, newest frame ${ago(camera.age)} old` : 'no frames yet';
    rows.push([cameraName(name), !camera.width ? 'no frames' : stale ? `${ago(camera.age)} old` : `${camera.fps.toFixed(1)} fps`, stale, detail]);
  }
  definitionList($('rates'), rows);
  $('nodes').replaceChildren(...(state.nodes ?? []).map(node => {
    const card = document.createElement('div'), name = document.createElement('b'), status = document.createElement('span');
    card.className = `node ${node.state}`; card.title = node.error || node.name;
    name.textContent = node.name; status.textContent = node.state;
    card.append(name, status);
    return card;
  }));
  const running = (state.nodes ?? []).filter(node => node.state === 'running').length;
  $('node-summary').textContent = `${running}/${state.nodes?.length ?? 0} running`;
}

// The stream: the SDK reconnects, answers heartbeats and validates every message.
const RobotState = {parse(message) {
  if (typeof message?.motors !== 'object' || !Array.isArray(message?.nodes)) throw new Error('robot_state without motors or nodes');
  return message;
}};
const DepthFrame = {parse(message) {  // docs/depth-stream.md
  if (message?.depth?.encoding !== 'uint16le' || !(message.depth.data instanceof Uint8Array) || typeof message?.camera?.fx !== 'number') {
    throw new Error('depth_frame without uint16le depth or intrinsics');
  }
  return message;
}};
const stream = new TransportService({url: `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/stream`,
                                     kinds: {robot_state: RobotState, depth_frame: DepthFrame}});
const depth = new DepthViews({tiles, layer: scene?.depth ?? null,
                              legend: {near: $('depth-near'), ramp: $('depth-ramp'), far: $('depth-far'), info: $('depth-info')}});
stream.subscribeToKind('depth_frame', frame => depth.handle(frame));
let latestState = null, lastSnapshotAt = 0, snapshotsThisSecond = 0;
stream.subscribeToKind('robot_state', state => {
  latestState = state; lastSnapshotAt = performance.now(); snapshotsThisSecond += 1;
  // The tracking tile shows the camera the head steers by: the OAK, or replay's synthetic camera.
  const tracking = Object.keys(state.overlays ?? {})[0] ?? 'oak';
  if (tracking !== tiles.trackingId) { tiles.trackingId = tracking; $('tracking-name').textContent = cameraName(tracking); }
  scene?.update(state);
  tiles.showTargetWithoutVideo(state);
  showCommands(state); showTarget(state); showHealth(state); showTests(state);
  motorTest.update(state);
});
// The SDK delivers each frame message's channels, then its overlays, then its image, so what arrives
// between a channels call and the image belongs to that image. Values are copied out of the decoder's
// storage, which a later frame may reuse.
const noFrame = {frameNumber: -1, overlays: [], target: null};
let frame = noFrame;
stream.subscribeToChannels(views => {
  frame = views ? {frameNumber: views.frameNumber, overlays: [], target: null} : noFrame;
  for (const view of views?.channels ?? []) {
    if (view.descriptor.kind !== 'ultraskelly.target' || view.rowCount < 1) continue;
    const column = name => view.data[view.descriptor.columns.indexOf(name)];
    const [width, height] = view.descriptor.image_size ?? [640, 480];
    frame.target = {cameraId: view.descriptor.camera_id, x: column('x'), y: column('y'), bodyAngle: column('body_angle_deg'), width, height};
  }
  // Each tracking-camera frame moves the gaze ray to its target (or hides it: the head follows nobody).
  if (views?.cameras?.some(camera => camera.id === tiles.trackingId)) showGaze(frame.target);
});
function showGaze(target) {
  const distance = scene?.gaze.update(target, depth.latest);
  $('target-face').textContent = distance == null ? '' : `face ${distance.toFixed(2)} m (est.)`;
}
stream.subscribeToOverlay(overlay => {
  if (overlay.frameNumber === frame.frameNumber) frame.overlays.push(overlay);
});
stream.subscribeToImages((buffer, frameNumber) => {
  tiles.handle(buffer, frame.frameNumber === frameNumber ? frame : noFrame);
});
stream.subscribeToProtocolErrors(problem => { console.error('Stream protocol error', problem); error(`Stream error: ${problem?.message ?? problem}`); });
stream.on('open', () => error(''));
stream.on('close', () => { $('connection').textContent = '○ Reconnecting…'; });
stream.connect();
window.ultraskelly = {scene, tiles, depth, stream, motorTest, get state() { return latestState; }};  // For DevTools

// Once a second: connection freshness and frame rates.
const rateText = (drawn, camera) => camera?.width ? `${drawn} drawn · ${camera.fps.toFixed(1)} fps` : drawn ? `${drawn} drawn` : '—';
setInterval(() => {
  const age = (performance.now() - lastSnapshotAt) / 1000, stale = !lastSnapshotAt || age > 1.5;
  if (lastSnapshotAt) $('connection').textContent = stale ? '○ No data' : '● Live';
  $('state-rate').textContent = `${snapshotsThisSecond}/s`; snapshotsThisSecond = 0;
  $('snapshot-age').textContent = lastSnapshotAt ? `last ${ago(age)}` : '—';
  $('snapshot-age').classList.toggle('stale', stale);
  const drawn = tiles.tick(), cameras = latestState?.cameras ?? {};
  $('oak-rate').textContent = rateText(drawn.oak, cameras[tiles.trackingId]);
  const depthShown = depth.tick();
  scene?.depth.tick();  // Hides depth that stopped arriving
  $('zed-rate').textContent = rateText(drawn.zed_left, cameras.zed_left) + (depthShown ? ` · depth ${depthShown}/s` : '');
}, 1000);
