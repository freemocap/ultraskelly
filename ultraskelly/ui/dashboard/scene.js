// The dashboard's 3D scene, in metres: the simulator's schematic skeleton driven by the live motor
// commands, with the ZED's depth on its head (depth3d.js). Motor values are the commands sent to the
// boards, not measured positions; the waist has no angle sensor, so its drive shows as an arrow. The
// head is drawn from its commands at the ratios measured on the robot (headpose.js), so it is approximate.
// The scene draws in the page's frame (frameclock.js), only when something in it changed, so the Pi's GPU
// idles between updates.
import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { buildSkeleton } from '/skeleton.js';
import { createDepthLayer } from '/depth3d.js';
import { createGazeLayer } from '/gaze.js';
import { createPersonLayer } from '/person3d.js';
import { headAngles, waistTurn } from '/headpose.js';

const deg = THREE.MathUtils.degToRad;
const JAW_OPEN_DEGREES = 28;   // Same jaw travel as the simulator's model
const JAW_SPEED = 90;          // Degrees per second, for display easing only
// The schematic's eyes are 2.15 units up; the robot's eye sockets are an estimated 1.88 m up
// (skellycad's UltraSkelly mock-up), so the skeleton is drawn at that scale and the scene is in metres.
const SKELETON_SCALE = 1.88 / 2.15;
const EYES = new THREE.Vector3(0, .23, .15);  // Midway between the eye sockets, in the head group's units
const VIEWS = {                // Camera position and the point it looks at, metres
  orbit: {position: [3.6, 2.7, -2.2], target: [0, 1.3, 1.2]},  // Over the shoulder, toward what the head faces
  side: {position: [5.2, 1.7, 1.4], target: [0, 1.35, 1.4]},
  top: {position: [0, 7.4, 2.2], target: [0, 1.0, 1.6]},
};
// "Behind head" rides with the commanded head: these points are in the head's frame, +z out of the face.
const BEHIND_EYE = new THREE.Vector3(.3, .9, -1.7), BEHIND_LOOK = new THREE.Vector3(0, 0, 3);

// Multisampling costs the Pi's GPU (VideoCore, which reports itself as V3D) far more than a desktop GPU,
// and a software renderer more still, so those draw without it. ?antialias=1 or ?antialias=0 decides instead.
function antialiasWanted() {
  const asked = new URLSearchParams(location.search).get('antialias');
  if (asked === '0' || asked === '1') return asked === '1';
  const gl = document.createElement('canvas').getContext('webgl2');
  if (!gl) return true;
  const info = gl.getExtension('WEBGL_debug_renderer_info');
  const name = String(gl.getParameter(info ? info.UNMASKED_RENDERER_WEBGL : gl.RENDERER));
  gl.getExtension('WEBGL_lose_context')?.loseContext();
  return !/V3D|VideoCore|llvmpipe|SwiftShader/i.test(name);
}

// The skeleton as one mesh per moving part and material. buildSkeleton makes about 60 meshes, each a draw call
// every render, which costs the Pi's GPU driver and Three.js far more than their few triangles. Each merged
// mesh stays in the group that moves it, so the waist, pan, tilt and jaw turn exactly as before.
function mergeMeshes(group) {
  const byMaterial = new Map();
  for (const child of [...group.children]) {
    if (!child.visible) continue;
    if (!child.isMesh || child.children.length) { mergeMeshes(child); continue; }
    if (!child.geometry.index) continue;
    if (!byMaterial.has(child.material)) byMaterial.set(child.material, []);
    byMaterial.get(child.material).push(child);
  }
  for (const [material, meshes] of byMaterial) {
    if (meshes.length < 2) continue;
    const parts = meshes.map(mesh => { mesh.updateMatrix(); return mesh.geometry.clone().applyMatrix4(mesh.matrix); });
    const vertices = parts.reduce((count, part) => count + part.attributes.position.count, 0);
    const position = new Float32Array(vertices * 3), normal = new Float32Array(vertices * 3), index = [];
    let offset = 0;
    for (const part of parts) {
      position.set(part.attributes.position.array, offset * 3); normal.set(part.attributes.normal.array, offset * 3);
      for (const i of part.index.array) index.push(i + offset);
      offset += part.attributes.position.count;
      part.dispose();
    }
    const merged = new THREE.BufferGeometry();
    merged.setAttribute('position', new THREE.BufferAttribute(position, 3));
    merged.setAttribute('normal', new THREE.BufferAttribute(normal, 3));
    merged.setIndex(index);
    for (const mesh of meshes) { group.remove(mesh); mesh.geometry.dispose(); }
    group.add(new THREE.Mesh(merged, material));
  }
}

// clock: the page's FrameClock; the scene draws as one of its parts.
export function createScene(viewport, clock) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#111b21');
  scene.fog = new THREE.Fog('#111b21', 9, 22);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 60);
  const renderer = new THREE.WebGLRenderer({antialias: antialiasWanted()});  // Throws without WebGL
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  viewport.appendChild(renderer.domElement);

  // Whatever changes the picture calls invalidate(): the gaze with each tracking frame, the pose with robot_state,
  // the depth, the view. The scene then draws in the page's next frame, all of them together.
  let dirty = true, paused = false, inFrame = false;
  function invalidate() {
    dirty = true;
    if (!inFrame && !paused) clock.changed();
  }
  const orbit = new OrbitControls(camera, renderer.domElement);
  orbit.enableDamping = true; orbit.minDistance = 0.5; orbit.maxDistance = 14;
  // OrbitControls damps by a share per update, so the share follows the time since the last one: the camera
  // coasts to rest in the same time as at 60 updates a second, however often the scene draws.
  const DAMPING = orbit.dampingFactor, DISPLAY_MS = 1000 / 60;
  let orbitAt = 0;
  orbit.addEventListener('change', invalidate);
  new ResizeObserver(() => {
    const {clientWidth: width, clientHeight: height} = viewport;
    if (!width || !height) return;
    camera.aspect = width / height; camera.updateProjectionMatrix();
    renderer.setSize(width, height);  // Clears the canvas, so draw again before it is shown
    if (!paused) renderer.render(scene, camera);
  }).observe(viewport);

  const skeleton = buildSkeleton(scene);
  skeleton.root.scale.setScalar(SKELETON_SCALE);
  // The robot reports a target pixel, not a world position, so the simulator's 3D target is hidden;
  // its 120-degree camera cone and fixed gaze arrow are too: the depth and gaze layers draw the ZED's and
  // the OAK's own.
  for (const part of [skeleton.target, skeleton.targetLine, skeleton.jawZone, skeleton.frustum, skeleton.gaze]) part.visible = false;
  mergeMeshes(skeleton.root);
  // Commanded waist drive: a solid arrow in front of the pelvis, pointing where its front is being turned.
  const driveMaterial = new THREE.MeshStandardMaterial({color: 0xffb269, emissive: 0xd76a23, emissiveIntensity: .6});
  const waistArrow = new THREE.Group();
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(.022, .022, .26, 12), driveMaterial);
  shaft.rotation.z = -Math.PI / 2; shaft.position.x = .13;
  const tip = new THREE.Mesh(new THREE.ConeGeometry(.06, .13, 16), driveMaterial);
  tip.rotation.z = -Math.PI / 2; tip.position.x = .32;
  waistArrow.add(shaft, tip); waistArrow.position.set(0, .05, .3); waistArrow.visible = false;
  skeleton.waist.add(waistArrow);
  const depth = createDepthLayer({head: skeleton.tilt, origin: EYES, scale: SKELETON_SCALE, invalidate});
  const gaze = createGazeLayer({group: depth.group, invalidate});
  const person = createPersonLayer({group: gaze.root, invalidate});  // The followed person's stick figure, with the gaze

  let view = 'orbit', jawShown = 0, jawGoal = 0, jawAt = 0;
  const eye = new THREE.Vector3(), look = new THREE.Vector3();
  function placeBehindHead() {
    skeleton.tilt.updateWorldMatrix(true, false);
    camera.position.copy(eye.copy(BEHIND_EYE).applyMatrix4(skeleton.tilt.matrixWorld));
    camera.lookAt(look.copy(BEHIND_LOOK).applyMatrix4(skeleton.tilt.matrixWorld));
  }
  function setView(name) {
    view = name;
    orbit.enabled = name !== 'behind';
    const preset = VIEWS[name];
    if (preset) { camera.position.set(...preset.position); orbit.target.set(...preset.target); orbit.update(); }
    else if (name === 'behind') placeBehindHead();
    invalidate();
  }
  setView('orbit');

  // One step of the jaw's display easing; true while it has further to go.
  function easeJaw(now) {
    if (jawShown === jawGoal) return false;
    const step = JAW_SPEED * Math.min(Math.max(now - jawAt, 0), 100) / 1000;
    jawAt = now;
    jawShown += Math.max(-step, Math.min(step, jawGoal - jawShown));
    skeleton.jaw.rotation.x = deg(jawShown);
    dirty = true;
    return jawShown !== jawGoal;
  }

  // The scene's part of the page's frame; true while the jaw eases or the camera settles.
  function draw(now) {
    if (paused) return false;  // Covered by an enlarged camera view
    inFrame = true;
    let again = easeJaw(now);
    if (view === 'behind') { if (dirty) placeBehindHead(); }
    else {
      orbit.dampingFactor = 1 - (1 - DAMPING) ** (Math.min(Math.max(now - orbitAt, DISPLAY_MS), 100) / DISPLAY_MS);
      orbitAt = now;
      if (orbit.update()) again = true;  // The camera is still moving or settling
    }
    inFrame = false;
    if (dirty) { dirty = false; renderer.render(scene, camera); }
    return again;
  }
  clock.add(draw);

  // Pose the skeleton from one robot_state message; draw again only when a command changed.
  let posed = '';
  function update(state) {
    const command = name => state.motors?.commands?.[name]?.value ?? null;
    const pan = command('pan'), tilt = command('tilt'), waist = command('waist'), mouth = command('mouth');
    const roll = command('roll') ?? state.servo_state?.roll ?? null;  // Held level (90) while no controller turns it
    const pose = `${pan} ${tilt} ${roll} ${waist} ${mouth}`;
    if (pose === posed) return;
    posed = pose;
    const head = headAngles({pan, tilt, roll});  // The depth on the head turns with all three
    if (head.yaw != null) skeleton.pan.rotation.y = head.yaw;
    if (head.pitch != null) skeleton.tilt.rotation.x = head.pitch;
    if (head.roll != null) skeleton.tilt.rotation.z = head.roll;
    const goal = mouth != null && mouth < 0 ? JAW_OPEN_DEGREES : 0;  // Negative jaw drive opens it
    if (goal !== jawGoal) { if (jawShown === jawGoal) jawAt = performance.now(); jawGoal = goal; }
    const turn = waistTurn(waist);  // The arrow points +x, the skeleton's own left, and is mirrored for its right
    waistArrow.visible = turn !== 0;
    if (turn) {
      const size = .7 + .5 * Math.abs(waist);
      waistArrow.scale.set(turn * size, size, size);
    }
    invalidate();
  }

  return {scene, camera, renderer, skeleton, waistArrow, depth, gaze, person, update, setView, invalidate,
          antialias: renderer.getContextAttributes().antialias,
          setPaused: value => { paused = value; if (!value) invalidate(); }};
}
