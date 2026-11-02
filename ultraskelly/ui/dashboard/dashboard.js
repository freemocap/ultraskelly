// Live dashboard: renders the runtime's JSON snapshots from /ws onto the shared skeleton.
import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { buildSkeleton } from '/skeleton.js';

const $ = id => document.getElementById(id);
const deg = THREE.MathUtils.degToRad;
const error = message => { $('error').textContent = message; $('error').hidden = !message; };
const JAW_OPEN_DEGREES = 28;   // Same jaw travel as the simulator's model
const JAW_SPEED = 90;          // Degrees per second, for display easing only

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
window.ultraskelly = {skeleton, waistArrow};  // For inspection from the browser console
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
  const target = state.target;
  const seen = target && target.x != null && target.y != null;
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
}

// One MJPEG panel per camera the runtime reports. A stream ends when the bot stops, so
// restart it after errors and whenever the WebSocket reconnects. Streams start only after
// the page's load event: an MJPEG image never finishes, so it would hold the page "loading".
const videos = new Map();
function restartStream(name) {
  const video = videos.get(name);
  if (!video) return;
  const start = () => { video.img.src = `/video/${encodeURIComponent(name)}.mjpg?start=${Date.now()}`; };
  if (document.readyState === 'complete') start();
  else window.addEventListener('load', start, {once: true});
}
function showVideos(state) {
  const grid = $('video-grid');
  for (const [name, camera] of Object.entries(state.cameras)) {
    if (!videos.has(name)) {
      if (!videos.size) grid.replaceChildren();
      const figure = document.createElement('figure'), img = document.createElement('img');
      const caption = document.createElement('figcaption'), label = document.createElement('span'), rate = document.createElement('span');
      figure.className = 'video'; img.alt = `${name} camera`;
      img.onerror = () => setTimeout(() => restartStream(name), 1000);
      label.textContent = name.toUpperCase();
      caption.append(label, rate); figure.append(img, caption); grid.append(figure);
      videos.set(name, {img, rate});
      restartStream(name);
    }
    videos.get(name).rate.textContent = `${camera.fps.toFixed(1)} fps from the camera`;
  }
}

// Connection: reconnect with backoff; flag snapshots that stop arriving.
let lastSnapshotAt = 0, snapshotsThisSecond = 0, retryDelay = 500;
function connect() {
  const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  socket.onopen = () => { retryDelay = 500; error(''); for (const name of videos.keys()) restartStream(name); };
  socket.onmessage = event => {
    const state = JSON.parse(event.data);
    lastSnapshotAt = performance.now(); snapshotsThisSecond += 1;
    showSkeleton(state); showCommands(state); showTarget(state); showHealth(state); showVideos(state);
  };
  socket.onclose = () => {
    $('connection').textContent = '○ Reconnecting…';
    setTimeout(connect, retryDelay); retryDelay = Math.min(retryDelay * 2, 5000);
  };
}
connect();
setInterval(() => {
  const age = (performance.now() - lastSnapshotAt) / 1000, stale = !lastSnapshotAt || age > 1.5;
  if (lastSnapshotAt) $('connection').textContent = stale ? '○ No data' : '● Live';
  $('snapshot-age').textContent = lastSnapshotAt ? ago(age) : '—';
  $('snapshot-age').classList.toggle('stale', stale);
  $('snapshot-rate').textContent = `${snapshotsThisSecond}/s`; snapshotsThisSecond = 0;
}, 1000);
