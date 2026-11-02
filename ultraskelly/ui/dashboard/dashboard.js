// Live dashboard: one FreeMoCap stream from /stream drives the skeleton, readouts and video.
// `robot_state` messages carry the bot's snapshot; FreeMoCap `frame` messages carry each
// camera's JPEG with the poses detected in that very image, drawn here over the video.
import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { TransportService, readImagePayload, decodeImageBitmaps } from '/vendor/freemocap-client.js';
import { buildSkeleton } from '/skeleton.js';

const $ = id => document.getElementById(id);
const deg = THREE.MathUtils.degToRad;
const error = message => { $('error').textContent = message; $('error').hidden = !message; };
const JAW_OPEN_DEGREES = 28;   // Same jaw travel as the simulator's model
const JAW_SPEED = 90;          // Degrees per second, for display easing only
const KEYPOINT_VISIBLE = 0.3;  // Keypoints below this confidence are not drawn
// Same colors the Pi used to draw on its video.
const COLORS = {bone: 'rgb(0,255,255)', head: 'rgb(255,0,0)', arm: 'rgb(0,255,0)', leg: 'rgb(0,0,255)',
                outline: '#fff', aim: 'rgb(141,227,194)', jaw: 'rgb(255,178,105)', target: 'rgb(255,178,105)'};

// 3D view, framed like the simulator.
const scene = new THREE.Scene();
scene.background = new THREE.Color('#111b21');
scene.fog = new THREE.Fog('#111b21', 9, 22);
const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 60);
let renderer;
try { renderer = new THREE.WebGLRenderer({antialias: true}); }
catch (e) { error('The 3D view needs WebGL2. Enable browser graphics acceleration and reload.'); throw e; }
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
$('viewport').appendChild(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true; orbit.minDistance = 2; orbit.maxDistance = 12;
function resetView() { camera.position.set(3.8, 2.6, 5.3); orbit.target.set(0, 1.35, 0.25); orbit.update(); }
resetView(); $('view-reset').onclick = resetView;
new ResizeObserver(() => {
  const box = $('viewport');
  camera.aspect = box.clientWidth / box.clientHeight; camera.updateProjectionMatrix();
  renderer.setSize(box.clientWidth, box.clientHeight);
}).observe($('viewport'));

const skeleton = buildSkeleton(scene);
// The robot reports a target pixel, not a world position, so the 3D target is hidden.
skeleton.target.visible = false; skeleton.targetLine.visible = false; skeleton.jawZone.visible = false;
// No waist angle sensor exists: show the commanded drive as a solid arrow in front of the
// pelvis, pointing where the front of the waist is being turned.
const driveMaterial = new THREE.MeshStandardMaterial({color: 0xffb269, emissive: 0xd76a23, emissiveIntensity: .6});
const waistArrow = new THREE.Group();
const shaft = new THREE.Mesh(new THREE.CylinderGeometry(.022, .022, .26, 12), driveMaterial);
shaft.rotation.z = -Math.PI / 2; shaft.position.x = .13;
const tip = new THREE.Mesh(new THREE.ConeGeometry(.06, .13, 16), driveMaterial);
tip.rotation.z = -Math.PI / 2; tip.position.x = .32;
waistArrow.add(shaft, tip); waistArrow.position.set(0, .05, .3); waistArrow.visible = false;
skeleton.waist.add(waistArrow);
let jawShown = 0, jawGoal = 0, lastFrame = performance.now();
renderer.setAnimationLoop(now => {
  const dt = Math.min((now - lastFrame) / 1000, .1); lastFrame = now;
  jawShown += Math.max(-JAW_SPEED * dt, Math.min(JAW_SPEED * dt, jawGoal - jawShown));
  skeleton.jaw.rotation.x = deg(jawShown);
  orbit.update(); renderer.render(scene, camera);
});

const commandOf = (state, name) => state.motors.commands[name]?.value ?? null;
const fixed = (value, digits, unit = '') => value == null ? '—' : `${value.toFixed(digits)}${unit}`;
const ago = seconds => seconds == null ? '—' : seconds < 1 ? `${Math.round(seconds * 1000)} ms` : `${seconds.toFixed(1)} s`;

function showSkeleton(state) {
  const pan = commandOf(state, 'pan'), tilt = commandOf(state, 'tilt');
  const waist = commandOf(state, 'waist'), mouth = commandOf(state, 'mouth');
  if (pan != null) skeleton.pan.rotation.y = deg(pan - 90);
  if (tilt != null) skeleton.tilt.rotation.x = deg(tilt - 90);
  jawGoal = mouth != null && mouth < 0 ? JAW_OPEN_DEGREES : 0;
  waistArrow.visible = waist != null && Math.abs(waist) > .01;
  if (waistArrow.visible) {
    // Negative waist drive turns the skeleton toward +yaw, which moves its front toward +x.
    const size = .7 + .5 * Math.abs(waist);
    waistArrow.scale.set(waist < 0 ? size : -size, size, size);
  }
}

function showCommands(state) {
  const units = {pan: ['°', 1], tilt: ['°', 1], waist: ['', 2], mouth: ['', 2]};
  for (const [name, [unit, digits]] of Object.entries(units)) {
    const command = state.motors.commands[name];
    $(name).textContent = fixed(command?.value ?? null, digits, unit);
    $(`${name}-age`).textContent = ago(command?.age);
  }
  const mode = $('motor-mode');
  mode.textContent = state.motors.simulated ? 'SIMULATED MOTORS' : 'REAL MOTORS';
  mode.classList.toggle('real', !state.motors.simulated);
}

const hasTarget = state => state?.target && state.target.x != null && state.target.y != null;

function showTarget(state) {
  const frame = state.cameras && Object.values(state.cameras)[0];
  const width = frame?.width || 640, height = frame?.height || 480;
  const canvas = $('camera'), ctx = canvas.getContext('2d');
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  ctx.fillStyle = '#0b1318'; ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = '#304953'; ctx.lineWidth = 2;
  for (let x = 0; x <= width; x += 80) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke(); }
  for (let y = 0; y <= height; y += 80) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
  ctx.strokeStyle = '#8de3c2'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(width / 2 - 20, height / 2); ctx.lineTo(width / 2 + 20, height / 2);
  ctx.moveTo(width / 2, height / 2 - 20); ctx.lineTo(width / 2, height / 2 + 20); ctx.stroke();
  const seen = hasTarget(state), target = state.target;
  if (seen) {
    ctx.strokeStyle = '#ffb269'; ctx.lineWidth = 4;
    ctx.beginPath(); ctx.arc(target.x, target.y, 14, 0, Math.PI * 2); ctx.stroke();
  }
  $('target-state').textContent = seen ? 'TRACKING' : 'NO TARGET';
  $('target-pixel').textContent = seen ? `${target.x}, ${target.y} px` : 'no target';
  $('target-angle').textContent = seen && target.angle != null ? `body ${target.angle.toFixed(0)}°` : '';
}

function definitionList(list, rows) {
  list.replaceChildren(...rows.flatMap(([term, value]) => {
    const dt = document.createElement('dt'), dd = document.createElement('dd');
    dt.textContent = term; dd.textContent = value;
    return [dt, dd];
  }));
}

function showHealth(state) {
  const labels = {servo_state: 'Servo state', target: 'Target', pose: 'Pose'};
  definitionList($('rates'), Object.entries(state.rates).map(([name, hz]) => [labels[name] || name, `${hz.toFixed(1)} Hz`]));
  const cameras = Object.entries(state.cameras);
  definitionList($('cameras'), cameras.length ? cameras.flatMap(([name, camera]) => [
    [`${name.toUpperCase()} rate`, `${camera.fps.toFixed(1)} fps`],
    [`${name.toUpperCase()} frame`, camera.width ? `${camera.width}×${camera.height}, ${ago(camera.age)} old` : 'no frames yet'],
  ]) : [['Cameras', 'none in this run']]);
  $('nodes').replaceChildren(...state.nodes.map(node => {
    const card = document.createElement('div'), name = document.createElement('b'), status = document.createElement('span');
    card.className = `node ${node.state}`; card.title = node.error || '';
    name.textContent = node.name; status.textContent = node.state;
    card.append(name, status);
    return card;
  }));
  const running = state.nodes.filter(node => node.state === 'running').length;
  $('node-summary').textContent = `${running}/${state.nodes.length} RUNNING`;
  for (const [name, camera] of cameras) videoPanel(name).cameraRate = camera.fps;
}

// Video: one canvas per camera, created when its first image arrives.
const panels = new Map();
function videoPanel(cameraId) {
  if (!panels.has(cameraId)) {
    const grid = $('video-grid');
    if (!panels.size) grid.replaceChildren();
    const figure = document.createElement('figure'), canvas = document.createElement('canvas');
    const caption = document.createElement('figcaption'), label = document.createElement('span'), rate = document.createElement('span');
    figure.className = 'video'; canvas.setAttribute('aria-label', `${cameraId} camera with the detected poses`);
    label.textContent = cameraId.toUpperCase();
    caption.append(label, rate); figure.append(canvas, caption); grid.append(figure);
    panels.set(cameraId, {canvas, rate, drawn: 0, cameraRate: null});
  }
  return panels.get(cameraId);
}

function drawPerson(ctx, overlay) {
  const column = name => overlay.columns.indexOf(name), width = overlay.columns.length;
  const [x, y, visibility] = [column('x'), column('y'), column('visibility')];
  const point = row => {
    const values = overlay.data.subarray(row * width, row * width + width);
    const seen = visibility < 0 || values[visibility] >= KEYPOINT_VISIBLE;
    return seen && Number.isFinite(values[x]) && Number.isFinite(values[y]) ? [values[x], values[y]] : null;
  };
  const rowOf = new Map(overlay.names.map((name, row) => [name, row]));
  ctx.strokeStyle = COLORS.bone; ctx.lineWidth = 2;
  for (const group of overlay.connectionGroups ?? []) {
    for (const [from, to] of group.pairs) {
      const start = point(rowOf.get(from)), end = point(rowOf.get(to));
      if (start && end) { ctx.beginPath(); ctx.moveTo(...start); ctx.lineTo(...end); ctx.stroke(); }
    }
  }
  overlay.names.forEach((_, row) => {
    const at = point(row);
    if (!at) return;
    // COCO order: five face points, six arm points, then hips and legs.
    ctx.fillStyle = row <= 4 ? COLORS.head : row <= 10 ? COLORS.arm : COLORS.leg;
    ctx.beginPath(); ctx.arc(at[0], at[1], 5, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = COLORS.outline; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(at[0], at[1], 6, 0, Math.PI * 2); ctx.stroke();
  });
}

function drawFrame(image, overlays) {
  const panel = videoPanel(image.cameraId), ctx = panel.canvas.getContext('2d');
  if (panel.canvas.width !== image.width || panel.canvas.height !== image.height) {
    panel.canvas.width = image.width; panel.canvas.height = image.height;
  }
  ctx.drawImage(image.bitmap, 0, 0);
  const guides = latestState?.overlays?.[image.cameraId];
  if (guides) {
    const [aimX, aimY] = guides.aim, zone = guides.jaw_zone;
    ctx.strokeStyle = COLORS.jaw; ctx.lineWidth = 1; ctx.strokeRect(aimX - zone, aimY - zone, 2 * zone, 2 * zone);
    ctx.strokeStyle = COLORS.aim; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(aimX - 20, aimY); ctx.lineTo(aimX + 20, aimY); ctx.moveTo(aimX, aimY - 20); ctx.lineTo(aimX, aimY + 20); ctx.stroke();
  }
  for (const overlay of overlays) if (overlay.cameraId === image.cameraId) drawPerson(ctx, overlay);
  if (guides && hasTarget(latestState)) {
    ctx.strokeStyle = COLORS.target; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(latestState.target.x, latestState.target.y, 20, 0, Math.PI * 2); ctx.stroke();
  }
  panel.drawn += 1;
}

// Decode newest-wins: while one frame decodes, only the newest arrival waits behind it.
let decoding = false, waiting = null;
async function showFrames(frame) {
  if (decoding) { waiting = frame; return; }
  decoding = true;
  try {
    for (let next = frame; next; next = waiting, waiting = null) {
      const images = await decodeImageBitmaps(readImagePayload(next.buffer));
      try { for (const image of images) drawFrame(image, next.overlays); }
      finally { for (const image of images) image.bitmap.close(); }
    }
  } catch (problem) {
    console.error('Could not show a camera frame', problem);
  } finally {
    decoding = false;
  }
}

// The stream: the SDK reconnects, answers heartbeats and validates every message.
const RobotState = {parse(message) {
  if (typeof message?.motors !== 'object' || !Array.isArray(message?.nodes)) throw new Error('robot_state without motors or nodes');
  return message;
}};
const stream = new TransportService({url: `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/stream`, kinds: {robot_state: RobotState}});
let latestState = null, lastSnapshotAt = 0, snapshotsThisSecond = 0;
stream.subscribeToKind('robot_state', state => {
  latestState = state; lastSnapshotAt = performance.now(); snapshotsThisSecond += 1;
  showSkeleton(state); showCommands(state); showTarget(state); showHealth(state);
});
let frameOverlays = {frameNumber: -1, overlays: []};
stream.subscribeToOverlay(overlay => {  // Delivered just before the image of the same frame
  if (overlay.frameNumber !== frameOverlays.frameNumber) frameOverlays = {frameNumber: overlay.frameNumber, overlays: []};
  frameOverlays.overlays.push(overlay);
});
stream.subscribeToImages((buffer, frameNumber) => {
  showFrames({buffer, overlays: frameOverlays.frameNumber === frameNumber ? frameOverlays.overlays : []});
});
stream.subscribeToProtocolErrors(problem => { console.error('Stream protocol error', problem); error(`Stream error: ${problem?.message ?? problem}`); });
stream.on('open', () => error(''));
stream.on('close', () => { $('connection').textContent = '○ Reconnecting…'; });
stream.connect();
window.ultraskelly = {skeleton, waistArrow, stream, panels};  // For inspection from DevTools

setInterval(() => {
  const age = (performance.now() - lastSnapshotAt) / 1000, stale = !lastSnapshotAt || age > 1.5;
  if (lastSnapshotAt) $('connection').textContent = stale ? '○ No data' : '● Live';
  $('snapshot-age').textContent = lastSnapshotAt ? ago(age) : '—';
  $('snapshot-age').classList.toggle('stale', stale);
  $('snapshot-rate').textContent = `${snapshotsThisSecond}/s`; snapshotsThisSecond = 0;
  for (const panel of panels.values()) {
    const camera = panel.cameraRate == null ? '' : ` · ${panel.cameraRate.toFixed(1)} fps from the camera`;
    panel.rate.textContent = `${panel.drawn} fps drawn${camera}`; panel.drawn = 0;
  }
}, 1000);
