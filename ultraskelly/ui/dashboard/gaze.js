// The gaze in the 3D scene: the OAK's view cone from the skull's nose hole, and a ray from the OAK through
// the followed person's target pixel. Where the ray meets the ZED's depth it ends at an estimated face
// point. Both cameras' poses in the head frame are best guesses until the camera bracket gives them, and
// the waist angle is not measured, so where the ray points in the room is an estimate.
import * as THREE from 'three';

// The OAK in the head frame H (metres; +x forward, +y the skull's left, +z up): its lens in the nose hole,
// about 4 cm below the eye sockets (skellycad's UltraSkelly mock-up), looking forward, with image x to
// the skull's right and image y down. Its colour camera sees about 69 degrees across the image.
const OAK_LENS = new THREE.Vector3(0.03, 0, -0.04);
const OAK_ROTATION = new THREE.Matrix3().set(0, 0, 1, -1, 0, 0, 0, -1, 0);  // Camera axes into H
const OAK_HFOV_DEG = 69;
const CONE_DEPTH = 1.5;   // Metres the drawn view cone reaches
const RAY_LENGTH = 3;     // Metres of ray drawn when no depth gives its end
const MARCH_STEP = 0.01;  // Metres between depth lookups along the ray
const NEAREST = 0.3, FURTHEST = 8;  // Metres along the ray that are searched
const IN_FRONT = 0.03, BEHIND = 0.15;  // The ray meets a surface when within these of its depth
const ZED_NOMINAL = {rotation: [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], translation_m: [0, 0.0315, 0]};

function intrinsics(width, height) {
  const fx = width / 2 / Math.tan(THREE.MathUtils.degToRad(OAK_HFOV_DEG / 2));
  return {fx, fy: fx, cx: width / 2, cy: height / 2, width, height};
}

// group: the head frame H in the scene (the depth layer's group); invalidate: asks the scene to draw again.
export function createGazeLayer({group, invalidate}) {
  const root = new THREE.Group();
  group.add(root);
  const cone = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color: 0x8de3c2, transparent: true, opacity: .35}));
  const solid = new THREE.LineBasicMaterial({color: 0x8de3c2});
  const dashed = new THREE.LineDashedMaterial({color: 0x8de3c2, dashSize: .08, gapSize: .06});
  const ray = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]), dashed);
  const face = new THREE.Mesh(new THREE.SphereGeometry(.035, 16, 12),
                              new THREE.MeshStandardMaterial({color: 0xffb269, emissive: 0xd76a23, emissiveIntensity: .7}));
  ray.visible = face.visible = false;
  root.add(cone, ray, face);
  let camera = null, lastKey = null, lastDepth = null, distance = null;
  const direction = new THREE.Vector3(), end = new THREE.Vector3(), point = new THREE.Vector3();

  function placeCone(width, height) {
    camera = intrinsics(width, height);
    const corner = (x, y) => new THREE.Vector3((x - camera.cx) / camera.fx, (y - camera.cy) / camera.fy, 1)
      .multiplyScalar(CONE_DEPTH).applyMatrix3(OAK_ROTATION).add(OAK_LENS);
    const corners = [corner(0, 0), corner(width, 0), corner(width, height), corner(0, height)];
    cone.geometry.setFromPoints(corners.flatMap((c, i) => [OAK_LENS, c, c, corners[(i + 1) % 4]]));
  }
  placeCone(640, 480);

  // Metres along the ray from the OAK lens to the first surface the ZED sees on it, or null. The ray meets
  // a surface where it crosses that pixel's depth; a point just in front of it counts when the ray grazes.
  function meetDepth(depth) {
    const pose = depth.cameraToHead ?? ZED_NOMINAL, R = pose.rotation, t = pose.translation_m, c = depth.camera;
    let approaching = null;
    for (let s = NEAREST; s <= FURTHEST; s += MARCH_STEP) {
      point.copy(direction).multiplyScalar(s).add(OAK_LENS);
      const q0 = point.x - t[0], q1 = point.y - t[1], q2 = point.z - t[2];  // Into the ZED's frame: R^T (p - t)
      const x = R[0][0] * q0 + R[1][0] * q1 + R[2][0] * q2, y = R[0][1] * q0 + R[1][1] * q1 + R[2][1] * q2;
      const z = R[0][2] * q0 + R[1][2] * q1 + R[2][2] * q2;
      if (z < NEAREST) continue;
      const u = Math.round((c.fx * x / z + c.cx) / depth.step), v = Math.round((c.fy * y / z + c.cy) / depth.step);
      if (u < 0 || v < 0 || u >= depth.width || v >= depth.height) continue;
      const d = depth.mm[v * depth.width + u] / 1000;
      if (!d || z > d + BEHIND) { approaching = null; continue; }  // Unknown, or hidden behind something nearer
      if (z >= d) return s;
      if (z >= d - IN_FRONT) approaching ??= s;
    }
    return approaching;
  }

  // target: {x, y, width, height}, the target pixel in the OAK image, or null; depth: the newest decoded
  // depth (depth.js), or null. Returns the face point's distance from between the eyes, in metres, or null.
  function update(target, depth) {
    const key = target ? `${target.x} ${target.y} ${target.width} ${target.height}` : 'none';
    if (key === lastKey && depth === lastDepth) return distance;  // Nothing new to draw
    lastKey = key; lastDepth = depth;
    if (!target) { ray.visible = face.visible = false; invalidate(); return distance = null; }
    if (!camera || camera.width !== target.width || camera.height !== target.height) placeCone(target.width, target.height);
    direction.set((target.x - camera.cx) / camera.fx, (target.y - camera.cy) / camera.fy, 1).applyMatrix3(OAK_ROTATION).normalize();
    const reach = depth?.mm ? meetDepth(depth) : null;
    end.copy(direction).multiplyScalar(reach ?? RAY_LENGTH).add(OAK_LENS);
    ray.geometry.setFromPoints([OAK_LENS, end]);
    ray.material = reach == null ? dashed : solid;
    if (reach == null) ray.computeLineDistances();
    ray.visible = true;
    face.visible = reach != null;
    face.position.copy(end);
    invalidate();
    return distance = reach == null ? null : end.length();
  }

  return {root, update, setVisible(on) { root.visible = on; invalidate(); }};
}
