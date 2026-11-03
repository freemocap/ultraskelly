// Live dashboard, laid out as docs/dashboard.md describes. One FreeMoCap stream from /stream drives
// it: `robot_state` messages carry the bot's snapshot (motor commands, target, rates, node health);
// FreeMoCap `frame` messages carry camera JPEGs with the poses found in that very image.
import { TransportService } from '/vendor/freemocap-client.js';
import { MotorTestPanel } from '/motor-test.js';
import { createScene } from '/scene.js';
import { CameraTiles } from '/tiles.js';
import { DepthViews } from '/depth.js';
import { TimeSeries } from '/timeseries.js';
import { placeKeyed, setProperty, setText } from '/dom.js';

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
  else { scene?.depth.setVisible({[layer]: on}); sendViews(); }
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
  setText($('tests-status'), text);
  setProperty($('tests-status'), 'className', `tests-status ${kind}`);
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
  setProperty($('tests-open'), 'hidden', !test); setProperty($('tests-stop'), 'hidden', !test);
  if (!test) { setText($('tests-travel'), ''); showTestStatus('Not available in this run'); return; }
  setText($('tests-travel'), `waist ${signed(test.waist_travel)}`);
  if (stopNotice && performance.now() < stopNotice.until && !test.running) showTestStatus(stopNotice.text, stopNotice.kind);
  else if (!test.enabled) showTestStatus('Turned off in the launch configuration');
  else if (test.running) showTestStatus(`Running: ${test.running.description}`);
  else if (test.last) {
    const outcome = test.last.outcome;
    showTestStatus(`${outcome[0].toUpperCase()}${outcome.slice(1)}: ${test.last.description}`, outcome === 'done' ? 'done' : 'refused');
  } else showTestStatus('Ready');
}

// Readouts, written only where their text changes (dom.js). Motor values are commands; the time series keeps
// the last 30 s of them, of the controllers' input and of the face distance, on the Pi's clock.
const series = new TimeSeries($('series'));
function showCommands(state) {
  const command = name => state.motors?.commands?.[name]?.value ?? null, t = state.time;
  series.push('head', t, [command('pan'), command('tilt')]);
  series.push('drive', t, [command('waist'), command('mouth')]);
  const aim = state.overlays?.[tiles.trackingId]?.aim;
  series.push('target', t, hasTarget(state) && aim ? [state.target.x - aim[0], state.target.y - aim[1]] : [NaN, NaN]);
  const face = state.face;
  series.push('face', t, [face?.distance != null && face.age < FACE_FRESH ? face.distance : NaN]);
  const mode = $('motor-mode');
  setText(mode, state.motors?.simulated ? 'Simulated motors' : 'Real motors');
  mode.classList.toggle('real', !state.motors?.simulated);
}

// The tracked face in 3D, as the Pi's face locator places it (robot_state.face): the OAK's target pixel
// found in the ZED's depth. The dashboard draws it; it does not compute it again.
const FACE_FRESH = 0.5;  // Seconds; an older face is no longer drawn
let gazeTarget = null, facePoint = null;
const metres = value => `${value >= 0 ? '' : '−'}${Math.abs(value).toFixed(2)} m`;
function showFace(state) {
  const face = state.face, fresh = face?.position_head && face.age < FACE_FRESH;
  facePoint = fresh ? face.position_head : null;
  scene?.gaze.update(gazeTarget, facePoint);
  const nominal = face?.alignment === 'nominal' ? ' (nominal)' : '';
  setText($('target-face'), fresh ? `face ${face.distance.toFixed(2)} m${nominal}` : '');
  setText($('face-distance'), fresh ? `${face.distance.toFixed(2)} m` : '—');
  $('face-distance').classList.toggle('none', !fresh);
  setText($('face-alignment'), face?.alignment ?? '');
  const [x, y, z] = facePoint ?? [];
  setText($('face-position'), fresh ? `${metres(x)} fwd · ${metres(y)} left · ${metres(z)} up`
    : !face ? 'No face locator in this run' : 'No face located');
  setText($('face-foot'), fresh && face.pixel ? `from OAK pixel ${face.pixel.map(Math.round).join(', ')} · ${ago(face.age)} old` : '');
}

function showTarget(state) {
  const seen = hasTarget(state), target = state.target, servo = state.servo_state;
  setText($('target-state'), seen ? 'Tracking' : 'No target');
  $('target-state').classList.toggle('tracking', seen);
  setText($('target-pixel'), seen ? `${target.x}, ${target.y} px` : '—');
  setText($('target-angle'), seen && target.angle != null ? `body ${target.angle.toFixed(0)}°` : '');
  // The head reports the target inside its jaw zone on both axes: the condition for opening the jaw.
  setText($('target-lock'), seen && servo ? (servo.locked_x && servo.locked_y ? 'in jaw zone' : 'outside jaw zone') : '');
  setText($('target-age'), target?.age != null ? ago(target.age) : '');
}

// Rates as dt/dd pairs and nodes as chips, each made once per name and changed in place.
const rateRows = new Map(), nodeChips = new Map();
function definitionList(list, rows) {
  placeKeyed(list, rateRows, rows.map(([term]) => term), () => [document.createElement('dt'), document.createElement('dd')]);
  for (const [term, value, stale, detail] of rows) {
    const [dt, dd] = rateRows.get(term);
    setText(dt, term); setText(dd, value); setProperty(dd, 'title', detail ?? value);
    dd.classList.toggle('stale', Boolean(stale));
  }
}

const cameraName = name => ({oak: 'OAK', zed_left: 'ZED L', zed_right: 'ZED R'}[name] ?? name);
// Node health and message rates, from the newest robot_state once a second.
function showHealth(state) {
  const labels = {servo_state: 'Servo', target: 'Target', pose: 'Pose'};
  const rows = Object.entries(state.rates ?? {}).map(([name, hz]) => [labels[name] || name, `${hz.toFixed(1)} Hz`]);
  for (const [name, camera] of Object.entries(state.cameras ?? {})) {
    // Camera rates show in the camera sections; here a camera appears only once its frames stop.
    if (camera.width && camera.age <= 1) continue;
    const detail = camera.width ? `${camera.width}×${camera.height}, newest frame ${ago(camera.age)} old` : 'no frames yet';
    rows.push([cameraName(name), camera.width ? `${ago(camera.age)} old` : 'no frames', true, detail]);
  }
  definitionList($('rates'), rows);
  const nodes = state.nodes ?? [];
  placeKeyed($('nodes'), nodeChips, nodes.map(node => node.name), () => {
    const card = document.createElement('div');
    card.append(document.createElement('b'), document.createElement('span'));
    return [card];
  });
  for (const node of nodes) {
    const [card] = nodeChips.get(node.name), [name, status] = card.children;
    setProperty(card, 'className', `node ${node.state}`); setProperty(card, 'title', node.error || node.name);
    setText(name, node.name); setText(status, node.state);
  }
  setText($('node-summary'), `${nodes.filter(node => node.state === 'running').length}/${nodes.length} running`);
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
  if (tracking !== tiles.trackingId) { tiles.trackingId = tracking; $('tracking-name').textContent = cameraName(tracking); sendViews(); }
  scene?.update(state);
  tiles.showTargetWithoutVideo(state);
  showCommands(state); showTarget(state); showFace(state); showTests(state);
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
  if (views?.cameras?.some(camera => camera.id === tiles.trackingId)) {
    gazeTarget = frame.target;
    scene?.gaze.update(gazeTarget, facePoint);
  }
});
stream.subscribeToOverlay(overlay => {
  if (overlay.frameNumber === frame.frameNumber) frame.overlays.push(overlay);
});
stream.subscribeToImages((buffer, frameNumber) => {
  tiles.handle(buffer, frame.frameNumber === frameNumber ? frame : noFrame);
});
stream.subscribeToProtocolErrors(problem => { console.error('Stream protocol error', problem); error(`Stream error: ${problem?.message ?? problem}`); });
stream.on('open', () => { error(''); sentViews = ''; sendViews(); });  // Each connection starts wanting everything

// Tell the server which views this page shows (docs/stream.md), so the Pi encodes only feeds someone draws:
// each shown camera, with its drawn size, and depth_frame while the heat map or the 3D depth is shown.
let sentViews = '', viewsTimer = null;
function sendViews() {
  clearTimeout(viewsTimer);
  viewsTimer = setTimeout(() => {
    const shown = scene?.depth.shown ?? {}, displayImageSizes = tiles.displayImageSizes();
    const depthShown = !document.hidden && (tiles.visible.has('depth_heat') || tiles.enlargedId === 'depth_heat' || shown.cloud || shown.mesh);
    const views = [...Object.keys(displayImageSizes), ...(depthShown ? ['depth_frame'] : [])];
    const message = {kind: 'views', views, displayImageSizes}, text = JSON.stringify(message);
    if (text === sentViews || stream.getState() !== 'CONNECTED') return;  // Sent again on the next connect
    sentViews = text;
    stream.send(message);
  }, 250);
}
tiles.onChange = sendViews;
document.addEventListener('visibilitychange', sendViews);
addEventListener('resize', sendViews);
stream.on('close', () => { $('connection').textContent = '○ Reconnecting…'; });
stream.connect();
window.ultraskelly = {scene, tiles, depth, series, stream, motorTest, get state() { return latestState; }};  // For DevTools

// Once a second: connection freshness, frame rates, node health and message rates.
const rateText = (drawn, camera) => camera?.width ? `${drawn} drawn · ${camera.fps.toFixed(1)} fps` : drawn ? `${drawn} drawn` : '—';
setInterval(() => {
  const age = (performance.now() - lastSnapshotAt) / 1000, stale = !lastSnapshotAt || age > 1.5;
  if (lastSnapshotAt) setText($('connection'), stale ? '○ No data' : '● Live');
  setText($('state-rate'), `${snapshotsThisSecond}/s`); snapshotsThisSecond = 0;
  setProperty($('connection'), 'title', lastSnapshotAt ? `Newest robot_state ${ago(age)} ago` : '');
  const drawn = tiles.tick(), cameras = latestState?.cameras ?? {};
  setText($('oak-rate'), rateText(drawn.oak, cameras[tiles.trackingId]));
  scene?.setFrameRate(cameras[tiles.trackingId]?.fps);  // The 3D scene draws at most once per tracking frame
  const depthShown = depth.tick();
  scene?.depth.tick();  // Hides depth that stopped arriving
  setText($('zed-rate'), rateText(drawn.zed_left, cameras.zed_left) + (depthShown ? ` · depth ${depthShown}/s` : ''));
  if (latestState) showHealth(latestState);
}, 1000);
