// Live dashboard, laid out as docs/dashboard.md describes. One FreeMoCap stream from /stream drives
// it: `robot_state` messages carry the bot's snapshot (motor commands, target, rates, node health);
// FreeMoCap `frame` messages carry camera JPEGs with the poses found in that very image. Everything the
// page shows changes in one frame per tracking-camera frame (frameclock.js).
import { TransportService } from '/vendor/freemocap-client.js';
import { MotorTestPanel } from '/motor-test.js';
import { createScene } from '/scene.js';
import { CameraTiles } from '/tiles.js';
import { DepthViews } from '/depth.js';
import { TimeSeries } from '/timeseries.js';
import { FrameClock } from '/frameclock.js';
import { OakFinds, measuresOf } from '/faces.js';
import { HealthView, STALE_S, healthOf, readingsStale, reasonOf } from '/health.js';
import { headNote, limitNote } from '/headpose.js';
import { placeKeyed, setProperty, setText } from '/dom.js';
import { ControlPanel, panelSettings, streamUrl } from '/panel.js';
import { resizable } from '/resize.js';

// The panels at the sizes they were left at, each handle between them working (resize.js).
resizable(document);

// A panel's notes open from its i button (index.html: a popover, which a click elsewhere or Esc closes), just under the
// button and inside the window: placed by the button as they open, then moved in once their size is known.
for (const note of document.querySelectorAll('.info-note')) {
  const button = document.querySelector(`[popovertarget="${note.id}"]`);
  const place = () => {
    const at = button.getBoundingClientRect(), box = note.getBoundingClientRect();
    const above = at.top - 4 - box.height, below = at.bottom + 4;
    note.style.left = `${Math.round(Math.max(8, Math.min(at.left, innerWidth - box.width - 8)))}px`;
    note.style.top = `${Math.round(below + box.height > innerHeight - 8 && above >= 8 ? above : below)}px`;
  };
  note.addEventListener('beforetoggle', event => { if (event.newState === 'open') place(); });
  note.addEventListener('toggle', event => { if (event.newState === 'open') place(); });
}

const $ = id => document.getElementById(id);
const error = message => { $('error').textContent = message; $('error').hidden = !message; };
const fixed = (value, digits, unit = '') => value == null ? '—' : `${value.toFixed(digits)}${unit}`;
const ago = seconds => seconds == null ? '—' : seconds < 1 ? `${Math.round(seconds * 1000)} ms` : `${seconds.toFixed(1)} s`;
const hasTarget = state => state?.target && state.target.x != null && state.target.y != null;

const clock = new FrameClock();
// Served by the control panel's service (ultraskelly/control), the page also turns the bot on and off, and reads the
// bot's stream from the bot's own port; served by the bot, it is the dashboard alone.
const settings = await panelSettings();

// 3D scene. Without WebGL the readouts and video still work.
let scene = null;
try { scene = createScene($('viewport'), clock); }
catch (problem) { console.error(problem); error('The 3D view needs WebGL2. Enable browser graphics acceleration and reload.'); }
const viewButtons = [...document.querySelectorAll('.views button')];
for (const button of viewButtons) button.onclick = () => {
  for (const other of viewButtons) other.setAttribute('aria-pressed', String(other === button));
  scene?.setView(button.dataset.view);
};
for (const button of document.querySelectorAll('.layers button[data-layer]')) button.onclick = () => {
  const on = button.getAttribute('aria-pressed') !== 'true', layer = button.dataset.layer;
  button.setAttribute('aria-pressed', String(on));
  if (layer === 'gaze') scene?.gaze.setVisible(on);
  else { scene?.depth.setVisible({[layer]: on}); sendViews(); }
};

// Align: once, the OAK's turn from the ZED fitted to the person in front of the robot (POST /api/align, the face
// locator's align). The button says how it went for a few seconds, the whole answer on hover.
const alignButton = $('align-cameras'), ALIGN_TITLE = alignButton.title;
alignButton.onclick = async () => {
  alignButton.disabled = true;
  setText(alignButton, 'Aligning…');
  let answer;
  try {
    const reply = await fetch('/api/align', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: '{}'});
    answer = await reply.json().catch(() => ({ok: false, message: `Refused (HTTP ${reply.status})`}));
  } catch (problem) { answer = {ok: false, message: `No answer: ${problem.message ?? problem}`}; }
  const done = answer.ok && answer.changed !== undefined;
  setText(alignButton, done ? (answer.changed ? `Aligned ${answer.yaw >= 0 ? '+' : ''}${answer.yaw.toFixed(1)}°` : 'Aligned ✓') : 'Not aligned');
  alignButton.classList.toggle('refused', !done);
  alignButton.title = answer.message ?? answer.error ?? ALIGN_TITLE;
  setTimeout(() => {
    setText(alignButton, 'Align'); alignButton.classList.remove('refused'); alignButton.disabled = false;
  }, 6000);
};

// Camera tiles; any tile enlarges over the 3D scene, which stops rendering while covered.
const tiles = new CameraTiles(document, {element: $('enlarged'), canvas: $('enlarged').querySelector('canvas'), label: $('enlarged-label')}, clock);
function enlarge(cameraId) {
  tiles.enlarge(cameraId);
  scene?.setPaused(tiles.enlargedId != null);
}
for (const figure of document.querySelectorAll('.tile[data-camera]')) {
  figure.onclick = () => enlarge(tiles.enlargedId === figure.dataset.camera ? null : figure.dataset.camera);
}
$('enlarged').querySelector('canvas').onclick = () => enlarge(null);
$('enlarged-close').onclick = () => enlarge(null);

// Motor tests: buttons right in the strip's card (motor-test.js); the control column loads them once the bot runs.
const motorTest = new MotorTestPanel($('motor-test'), {travel: $('tests-travel'), load: !settings});
addEventListener('keydown', event => {
  if (event.key === 'Escape' && tiles.enlargedId) enlarge(null);
});

// Readouts, written in the page's frame and only where their text changes (dom.js). Motor values are commands;
// the time series keeps the last 30 s of them, of the controllers' input, of the face distance and of the
// ZED's IMU, on the Pi's clock.
const series = new TimeSeries($('series'), clock);
function pushSeries(state) {
  const command = name => state.motors?.commands?.[name]?.value ?? null, t = state.time;
  series.push('head', t, [command('pan'), command('tilt')]);
  series.push('drive', t, [command('waist'), command('mouth')]);
  const aim = state.overlays?.[tiles.trackingId]?.aim;
  series.push('target', t, hasTarget(state) && aim ? [state.target.x - aim[0], state.target.y - aim[1]] : [NaN, NaN]);
  const face = state.face;
  series.push('face', t, [face?.distance != null && face.age < FACE_FRESH ? face.distance : NaN]);
}
function showMode(state) {
  const mode = $('motor-mode');
  setText(mode, state.motors?.simulated ? 'Simulated motors' : 'Real motors');
  mode.classList.toggle('real', !state.motors?.simulated);
}

// The tracked face in 3D, as the Pi's face locator places it (robot_state.face): the OAK's target pixel
// found in the ZED's depth. The dashboard draws it; it does not compute it again.
const FACE_FRESH = 0.5;  // Seconds; an older face is no longer drawn
let gazeTarget = null, facePoint = null;
const metres = value => `${value >= 0 ? '' : '−'}${Math.abs(value).toFixed(2)} m`;
const faceFresh = face => Boolean(face?.position_head && face.age < FACE_FRESH);
function placeFace(state) {
  facePoint = faceFresh(state.face) ? state.face.position_head : null;
  scene?.gaze.update(gazeTarget, facePoint);
  const recent = state.face && state.face.age < FACE_FRESH;  // The figure needs no depth at the nose, only some joints'
  scene?.person.update(recent ? state.face.skeleton_head ?? null : null);
}
// Every view that waits on a part says why it is empty and what to do, from the bot's checks (health.js); with
// the part well, the view's own words.
let reasons = {};
function explainParts(model, state) {
  const why = id => reasonOf(model, id), zed = why('zed_camera'), imu = why('zed_imu');
  // The bot's depth check speaks for itself; without one, the ZED camera's problem is the depth's too.
  const found = why('depth'), depth = found ?? (zed && {message: `No depth: ${zed.message}`, fix: zed.fix});
  reasons = {oak: why('oak_camera'), zed, depth, hailo: why('hailo')};
  tiles.explain('oak', reasons.oak, hasTarget(state) ? 'No video: target position only' : null);
  tiles.explain('zed_left', zed); tiles.explain('zed_right', zed);
  tiles.explain('depth_heat', depth);
  for (const lane of ['accel', 'gyro']) series.setWaiting(lane, imu?.message ?? null);
  setText($('stage-note'), depth?.message ?? ''); setProperty($('stage-note'), 'hidden', !depth);
  // While a board is down the controllers still run and record their commands, but nothing reaches it.
  series.setNote('head', why('servo_board') ? 'not sent' : null);
  series.setNote('drive', why('motor_board') ? 'not sent' : null);
}

// How the scene draws the head (headpose.js), and a head command at the end of its range: there the tracking
// controller has wound up while the real head barely moves.
$('stage-head').textContent = headNote();
function showLimits(state) {
  const commanded = name => state.motors?.commands?.[name]?.value;
  const note = limitNote({pan: commanded('pan'), tilt: commanded('tilt')});
  setText($('stage-limits'), note ?? ''); setProperty($('stage-limits'), 'hidden', !note);
}

// The face card: its distance and place, or the part that keeps it from being found; the pixel and age on hover.
function showFace(state) {
  const face = state.face, fresh = faceFresh(face);
  setText($('face-distance'), fresh ? `${face.distance.toFixed(2)} m` : '—');
  $('face-distance').classList.toggle('none', !fresh);
  setText($('face-alignment'), face?.alignment ?? '—');
  const [x, y, z] = facePoint ?? [];
  setText($('face-position'), fresh ? `${metres(x)} fwd · ${metres(y)} left · ${metres(z)} up`
    : reasons.depth ? reasons.depth.message : reasons.oak ? `No OAK target: ${reasons.oak.message}` : '—');
  setProperty($('face-position'), 'title', fresh && face.pixel ? `From OAK pixel ${face.pixel.map(Math.round).join(', ')}, ${ago(face.age)} old` : '');
}

function showTarget(state) {
  const seen = hasTarget(state), target = state.target, servo = state.servo_state;
  setText($('target-state'), seen ? 'Tracking' : 'No target');
  $('target-state').classList.toggle('tracking', seen);
  // Without the AI HAT+ there are no poses, so no target: say so.
  setText($('target-pixel'), seen ? `${target.x}, ${target.y} px` : reasons.hailo ? `no poses: ${reasons.hailo.message}` : '—');
  setText($('target-angle'), seen && target.angle != null ? `body ${target.angle.toFixed(0)}°` : '');
  // The head reports the target inside its jaw zone on both axes: the condition for opening the jaw.
  setText($('target-lock'), seen && servo ? (servo.locked_x && servo.locked_y ? 'in jaw zone' : 'outside jaw zone') : '');
  setText($('target-age'), target?.age != null ? ago(target.age) : '');
}

// Rates as dt/dd pairs and nodes as chips, each made once per name and changed in place. A wide row takes the
// list's whole width.
const rateRows = new Map(), nodeChips = new Map();
function definitionList(list, rows) {
  placeKeyed(list, rateRows, rows.map(([term]) => term), () => [document.createElement('dt'), document.createElement('dd')]);
  for (const [term, value, stale, detail, wide] of rows) {
    const [dt, dd] = rateRows.get(term);
    setText(dt, term); setText(dd, value); setProperty(dd, 'title', detail ?? value);
    dd.classList.toggle('stale', Boolean(stale));
    dt.classList.toggle('wide', Boolean(wide)); dd.classList.toggle('wide', Boolean(wide));
  }
}

const cameraName = name => ({oak: 'OAK', zed_left: 'ZED L', zed_right: 'ZED R'}[name] ?? name);
// The OAK chip's own rates and load (robot_state.oak): its frames, what its models found or measured each second,
// its two CPUs (LeonOS and LeonRT), memory and temperature. Null without the OAK.
function oakRows(oak) {
  if (!oak) return [];
  const hz = name => oak.rates?.[name] == null ? '—' : `${oak.rates[name].toFixed(1)} Hz`, chip = oak.chip ?? {};
  const percent = value => value == null ? '—' : `${Math.round(value * 100)}%`;
  const missing = Object.entries(oak.missing ?? {}).filter(([, count]) => count).map(([what, count]) => `${what} ${count}`).join(', ');
  return [
    ['OAK frames', hz('frames')],
    ['Faces', hz('face_detection'), false, `faces found ${hz('face_detection')}`],
    ['Head pose', hz('head_pose'), false, `head pose ${hz('head_pose')}, gaze ${hz('gaze')}`],
    ['Face mesh', hz('face_landmarks')],
    ['Hands', hz('hand_landmarks'), false, `palms looked for ${hz('palm_detection')}, hands measured ${hz('hand_landmarks')}`],
    ['OAK chip', `${percent(chip.leon_os_cpu)} · ${percent(chip.leon_rt_cpu)} · ${chip.temperature_c?.toFixed(0) ?? '—'} °C`, false,
     `CPU LeonOS ${percent(chip.leon_os_cpu)}, LeonRT ${percent(chip.leon_rt_cpu)}; memory ${chip.ddr_used_mib?.toFixed(0) ?? '—'} of ` +
     `${chip.ddr_total_mib?.toFixed(0) ?? '—'} MiB; ${chip.temperature_c?.toFixed(1) ?? '—'} °C` +
     (oak.faces_incomplete ? `; ${oak.faces_incomplete} frames sent before all their measurements came (${missing})` : ''), true],
  ];
}

// Node health and message rates, from the newest robot_state once a second.
function showHealth(state) {
  const labels = {servo_state: 'Servo', target: 'Target', pose: 'Body pose', face: 'Face locator', imu: 'IMU batches', results: 'OAK results'};
  const rows = Object.entries(state.rates ?? {}).map(([name, hz]) => [labels[name] || name, `${hz.toFixed(1)} Hz`]);
  rows.push(...oakRows(state.oak));
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
  setText($('node-summary'), `${nodes.filter(node => node.state === 'running').length}/${nodes.length}`);
  setProperty($('node-summary'), 'title', 'Nodes running');
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
const Imu = {parse(message) {  // docs/stream.md
  const samples = message?.samples;
  if (typeof message?.start_time !== 'number' || !(samples?.data instanceof Uint8Array) || !Array.isArray(samples?.columns)) {
    throw new Error('imu without start_time or samples');
  }
  return message;
}};
const Health = {parse(message) {  // docs/stream.md: the bot's hardware checks
  if (!Array.isArray(message?.components)) throw new Error('health without components');
  return message;
}};
const stream = new TransportService({url: streamUrl(settings), reconnectDelay: settings ? 2000 : 5000,
                                     kinds: {robot_state: RobotState, depth_frame: DepthFrame, imu: Imu, health: Health}});
// The bot's newest hardware checks: on every change, every 2 s, and to each viewer as it connects.
let latestHealth = null, healthAt = 0;
stream.subscribeToKind('health', report => { latestHealth = report; healthAt = performance.now(); clock.changed(); });
const depth = new DepthViews({tiles, layer: scene?.depth ?? null, clock,
                              legend: {near: $('depth-near'), ramp: $('depth-ramp'), far: $('depth-far'), info: $('depth-info')}});
stream.subscribeToKind('depth_frame', frame => depth.handle(frame));
let latestState = null, lastSnapshotAt = 0, snapshotsThisSecond = 0;
stream.subscribeToKind('robot_state', state => {
  latestState = state; lastSnapshotAt = performance.now(); snapshotsThisSecond += 1;
  // The tracking tile shows the camera the head steers by: the OAK, or replay's synthetic camera.
  const tracking = Object.keys(state.overlays ?? {})[0] ?? 'oak';
  if (tracking !== tiles.trackingId) { tiles.trackingId = tracking; $('tracking-name').textContent = cameraName(tracking); sendViews(); }
  tiles.state = state;  // The aim point and jaw zone drawn over the video
  scene?.update(state);
  placeFace(state);
  pushSeries(state);
  clock.changed();
});
// The readouts' part of the page's frame: the newest robot_state's text, once per state.
// What works and what does not (health.js): the banner in the header, the panel over the left of the 3D scene,
// the nodes card's lights.
const health = new HealthView({banner: $('problems'), count: $('problems-count'), summary: $('problems-summary'),
                               toggle: $('problems-toggle'), panel: $('health'), close: $('health-close'), rows: $('health-rows'),
                               lights: $('health-lights'), diagnostics: $('health-diagnostics'), source: $('health-source'),
                               readingsAge: $('health-readings-age')});
// The control column, with the controls in front and the live views beside it; it connects the stream while the bot runs.
const control = settings ? new ControlPanel(settings, {stream, health, motorTest}) : null;
let stateShown = null;
clock.add(() => {
  const state = latestState;
  if (!state || state === stateShown) return false;
  stateShown = state;
  tiles.showTargetWithoutVideo(state);
  health.update(healthOf(state, latestHealth));
  explainParts(health.model, state);
  showMode(state); showTarget(state); showFace(state); showLimits(state);
  motorTest.update(state, health.model);
  control?.update(state, health.model);
  return false;
});
// The ZED IMU's batches (docs/stream.md), in its sensor axes. The bytes are copied out of the decoder's
// storage before they are read as rows of float32: time after start_time, ax, ay, az (m/s²), gx, gy, gz (°/s).
let imuNext = null;  // The sample number the next batch starts at, unless batches were missed
stream.subscribeToKind('imu', batch => {
  const {columns} = batch.samples, width = columns.length, bytes = batch.samples.data.slice();
  const rows = new Float32Array(bytes.buffer, 0, Math.floor(bytes.byteLength / 4)), n = Math.floor(rows.length / width);
  if (!n || columns.indexOf('time') < 0) return;
  const column = name => {
    const c = columns.indexOf(name), out = new Float64Array(n);
    for (let k = 0; k < n; k++) out[k] = c < 0 ? NaN : rows[k * width + c];
    return out;
  };
  const times = column('time');
  for (let k = 0; k < n; k++) times[k] += batch.start_time;
  const gap = imuNext != null && batch.first_sample !== imuNext;
  imuNext = batch.first_sample + n;
  series.pushBatch('accel', times, [column('ax'), column('ay'), column('az')], gap);
  series.pushBatch('gyro', times, [column('gx'), column('gy'), column('gz')], gap);
});
// The SDK delivers each frame message's channels, then its overlays, then its image, so what arrives
// between a channels call and the image belongs to that image. Values are copied out of the decoder's
// storage, which a later frame may reuse.
const noFrame = {frameNumber: -1, overlays: [], target: null, finds: null};
const oakFinds = new OakFinds();  // The OAK's faces and hands, each frame's with what earlier frames measured
let frame = noFrame;
stream.subscribeToChannels(views => {
  frame = views ? {frameNumber: views.frameNumber, overlays: [], target: null, finds: null, measures: measuresOf(views),
                   tracking: Boolean(views.cameras?.some(camera => camera.id === tiles.trackingId))} : noFrame;
  for (const view of views?.channels ?? []) {
    if (view.descriptor.kind !== 'ultraskelly.target' || view.rowCount < 1) continue;
    const column = name => view.data[view.descriptor.columns.indexOf(name)];
    const [width, height] = view.descriptor.image_size ?? [640, 480];
    frame.target = {cameraId: view.descriptor.camera_id, x: column('x'), y: column('y'), bodyAngle: column('body_angle_deg'), width, height};
  }
  // Each tracking-camera frame moves the gaze ray to its target (or hides it: the head follows nobody).
  if (frame.tracking) {
    gazeTarget = frame.target;
    scene?.gaze.update(gazeTarget, facePoint);
  }
});
stream.subscribeToOverlay(overlay => {
  if (overlay.frameNumber === frame.frameNumber) frame.overlays.push(overlay);
});
stream.subscribeToImages((buffer, frameNumber) => {
  if (frame.frameNumber !== frameNumber) { tiles.handle(buffer, noFrame); return; }
  // The faces and hands of every tracking frame, decoded or not, so measurements carry over between them.
  if (frame.tracking) frame.finds = oakFinds.update(frame.overlays, frame.measures);
  tiles.handle(buffer, frame);
});
stream.subscribeToProtocolErrors(problem => { console.error('Stream protocol error', problem); error(`Stream error: ${problem?.message ?? problem}`); });
stream.on('open', () => { error(''); sentViews = ''; sendViews(); });  // Each connection starts wanting everything

// Tell the server what this page draws (docs/stream.md), so the Pi encodes only the feeds someone shows:
// each shown camera with its drawn size, depth while the heat map or the 3D depth is shown, and the IMU while
// the page is, since the time series keeps its last 30 s even when scrolled out of view.
let sentViews = '', viewsTimer = null;
function sendViews() {
  clearTimeout(viewsTimer);
  viewsTimer = setTimeout(() => {
    const shown = scene?.depth.shown ?? {}, displayImageSizes = tiles.displayImageSizes();
    const depthShown = !document.hidden && (tiles.visible.has('depth_heat') || tiles.enlargedId === 'depth_heat' || shown.cloud || shown.mesh);
    const kinds = [...(depthShown ? ['depth_frame'] : []), ...(document.hidden ? [] : ['imu'])];
    const key = JSON.stringify([displayImageSizes, kinds]);
    if (key === sentViews || stream.getState() !== 'CONNECTED') return;  // Sent again on the next connect
    sentViews = key;
    stream.send({frameNumber: tiles.lastFrameNumber, displayImageSizes, kinds});
  }, 250);
}
tiles.onChange = sendViews;
document.addEventListener('visibilitychange', sendViews);
addEventListener('resize', sendViews);
stream.on('close', () => {
  $('connection').textContent = '○ Reconnecting…';
  health.setOwn('link', {text: 'Lost the connection to the bot, reconnecting', status: 'fail'});
});
if (!control) stream.connect();  // The control column connects it once the bot is up
window.ultraskelly = {clock, scene, tiles, depth, series, stream, motorTest, oakFinds, health, control, get state() { return latestState; }};  // For DevTools

// Once a second: connection freshness, frame rates, node health and message rates. A camera's rate is its own; the
// frames this page drew in the last second are the tooltip.
const rateText = camera => camera?.width ? `${camera.fps.toFixed(1)} fps` : '—';
const drawnText = drawn => `${drawn ?? 0} frames drawn here in the last second`;
setInterval(() => {
  const age = (performance.now() - lastSnapshotAt) / 1000, stale = !lastSnapshotAt || age > 1.5;
  if (control?.down) setText($('connection'), control.down);
  else if (lastSnapshotAt) setText($('connection'), stale ? '○ No data' : '● Live');
  // The dashboard's own check: whether the bot is sending anything at all.
  if (stream.getState() === 'CONNECTED') {
    health.setOwn('link', lastSnapshotAt && stale ? {text: `No data from the bot for ${ago(age)}`, status: 'fail'} : null);
    // Checks that stop coming while robot_state still does are stale: say so rather than show old ones as now.
    const checksAge = (performance.now() - healthAt) / 1000;
    health.setOwn('checks', latestHealth && !stale && checksAge > STALE_S ? {text: `No hardware checks from the bot for ${Math.round(checksAge)} s`, status: 'warn'} : null);
    // Checks that still come, but whose live readings stopped: the bot keeps sending its last ones (diagnostics_time).
    health.setOwn('readings', latestHealth && !stale && checksAge <= STALE_S && readingsStale(health.model)
      ? {text: `The bot's live checks stopped: readings ${Math.round(health.model.diagnosticsAge)} s old`, status: 'warn'} : null);
  }
  health.renderSince();
  setText($('state-rate'), `${snapshotsThisSecond}/s`); snapshotsThisSecond = 0;
  setProperty($('connection'), 'title', lastSnapshotAt ? `Newest robot_state ${ago(age)} ago` : '');
  const drawn = tiles.tick(), cameras = latestState?.cameras ?? {};
  setText($('oak-rate'), rateText(cameras[tiles.trackingId])); setProperty($('oak-rate'), 'title', drawnText(drawn.oak));
  clock.setFrameRate(cameras[tiles.trackingId]?.fps);  // The page draws at most once per tracking frame
  const depthShown = depth.tick();
  scene?.depth.tick();  // Hides depth that stopped arriving
  setText($('zed-rate'), rateText(cameras.zed_left) + (depthShown ? ` · depth ${depthShown}/s` : ''));
  setProperty($('zed-rate'), 'title', drawnText(drawn.zed_left));
  if (latestState) showHealth(latestState);
}, 1000);
