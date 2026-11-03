// The dashboard's 3D scene: the simulator's schematic skeleton driven by the live motor commands.
// These are the commands sent to the boards, not measured positions; the waist has no angle
// sensor, so its drive shows as an arrow rather than a turn.
import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { buildSkeleton } from '/skeleton.js';

const deg = THREE.MathUtils.degToRad;
const JAW_OPEN_DEGREES = 28;   // Same jaw travel as the simulator's model
const JAW_SPEED = 90;          // Degrees per second, for display easing only
const VIEWS = {                // Camera position and the point it looks at
  orbit: {position: [3.8, 2.6, 5.3], target: [0, 1.35, 0.25]},
  side: {position: [5.8, 1.9, 0.9], target: [0, 1.45, 0.9]},
  top: {position: [0, 8.5, 1.6], target: [0, 1.2, 1.0]},
};
// "Behind head" rides with the commanded head: these points are in the head's frame, +z out of the face.
const BEHIND_EYE = new THREE.Vector3(.3, .9, -1.7), BEHIND_LOOK = new THREE.Vector3(0, 0, 3);

export function createScene(viewport) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#111b21');
  scene.fog = new THREE.Fog('#111b21', 9, 22);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 60);
  const renderer = new THREE.WebGLRenderer({antialias: true});  // Throws without WebGL
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  viewport.appendChild(renderer.domElement);
  const orbit = new OrbitControls(camera, renderer.domElement);
  orbit.enableDamping = true; orbit.minDistance = 1; orbit.maxDistance = 14;
  new ResizeObserver(() => {
    const {clientWidth: width, clientHeight: height} = viewport;
    if (!width || !height) return;
    camera.aspect = width / height; camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  }).observe(viewport);

  const skeleton = buildSkeleton(scene);
  // The robot reports a target pixel, not a world position, so the simulator's 3D target is hidden.
  skeleton.target.visible = false; skeleton.targetLine.visible = false; skeleton.jawZone.visible = false;
  // Commanded waist drive: a solid arrow in front of the pelvis, pointing where its front is being turned.
  const driveMaterial = new THREE.MeshStandardMaterial({color: 0xffb269, emissive: 0xd76a23, emissiveIntensity: .6});
  const waistArrow = new THREE.Group();
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(.022, .022, .26, 12), driveMaterial);
  shaft.rotation.z = -Math.PI / 2; shaft.position.x = .13;
  const tip = new THREE.Mesh(new THREE.ConeGeometry(.06, .13, 16), driveMaterial);
  tip.rotation.z = -Math.PI / 2; tip.position.x = .32;
  waistArrow.add(shaft, tip); waistArrow.position.set(0, .05, .3); waistArrow.visible = false;
  skeleton.waist.add(waistArrow);

  let view = 'orbit', paused = false, jawShown = 0, jawGoal = 0, last = performance.now();
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
  }
  setView('orbit');

  renderer.setAnimationLoop(now => {
    const dt = Math.min((now - last) / 1000, .1); last = now;
    jawShown += Math.max(-JAW_SPEED * dt, Math.min(JAW_SPEED * dt, jawGoal - jawShown));
    skeleton.jaw.rotation.x = deg(jawShown);
    if (paused) return;  // Covered by an enlarged camera view
    if (view === 'behind') placeBehindHead();
    else orbit.update();
    renderer.render(scene, camera);
  });

  // Pose the skeleton from one robot_state message.
  function update(state) {
    const command = name => state.motors?.commands?.[name]?.value ?? null;
    const pan = command('pan'), tilt = command('tilt'), waist = command('waist'), mouth = command('mouth');
    if (pan != null) skeleton.pan.rotation.y = deg(pan - 90);
    if (tilt != null) skeleton.tilt.rotation.x = deg(tilt - 90);
    jawGoal = mouth != null && mouth < 0 ? JAW_OPEN_DEGREES : 0;  // Negative jaw drive opens it
    waistArrow.visible = waist != null && Math.abs(waist) > .01;
    if (waistArrow.visible) {
      // Negative waist drive turns the skeleton toward +yaw, which moves its front toward +x.
      const size = .7 + .5 * Math.abs(waist);
      waistArrow.scale.set(waist < 0 ? size : -size, size, size);
    }
  }

  return {scene, camera, skeleton, waistArrow, update, setView, setPaused: value => { paused = value; }};
}
