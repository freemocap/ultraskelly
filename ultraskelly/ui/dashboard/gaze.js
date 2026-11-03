// The gaze in the 3D scene: the OAK's view cone from the skull's nose hole, and a ray from the OAK toward
// the followed person. The face point is the Pi's (robot_state.face, from its face locator: the OAK's
// target pixel found in the ZED's depth); without one the ray is a dashed direction through the target
// pixel. Camera poses are nominal until the camera bracket gives them, and the waist angle is not
// measured, so where the ray points in the room is an estimate.
import * as THREE from 'three';

// The OAK in the head frame H (metres; +x forward, +y the skull's left, +z up): its lens in the nose hole,
// about 4 cm below the eye sockets (skellycad's UltraSkelly mock-up), looking forward, with image x to
// the skull's right and image y down. Its colour camera sees about 69 degrees across the image.
const OAK_LENS = new THREE.Vector3(0.03, 0, -0.04);
const OAK_ROTATION = new THREE.Matrix3().set(0, 0, 1, -1, 0, 0, 0, -1, 0);  // Camera axes into H
const OAK_HFOV_DEG = 69;
const CONE_DEPTH = 1.5;   // Metres the drawn view cone reaches
const RAY_LENGTH = 3;     // Metres of dashed ray drawn when no face point is known

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
  let camera = null, lastKey = null;
  const end = new THREE.Vector3();

  function placeCone(width, height) {
    camera = intrinsics(width, height);
    const corner = (x, y) => new THREE.Vector3((x - camera.cx) / camera.fx, (y - camera.cy) / camera.fy, 1)
      .multiplyScalar(CONE_DEPTH).applyMatrix3(OAK_ROTATION).add(OAK_LENS);
    const corners = [corner(0, 0), corner(width, 0), corner(width, height), corner(0, height)];
    cone.geometry.setFromPoints(corners.flatMap((c, i) => [OAK_LENS, c, c, corners[(i + 1) % 4]]));
  }
  placeCone(640, 480);

  // target: {x, y, width, height}, the target pixel in the OAK image, or null (the head follows nobody);
  // facePoint: [x, y, z] in H from robot_state.face, or null.
  function update(target, facePoint) {
    const key = `${target ? `${target.x} ${target.y} ${target.width} ${target.height}` : '-'} ${facePoint ?? '-'}`;
    if (key === lastKey) return;  // Nothing new to draw
    lastKey = key;
    if (facePoint) {
      end.fromArray(facePoint);
      ray.material = solid;
    } else if (target) {
      if (!camera || camera.width !== target.width || camera.height !== target.height) placeCone(target.width, target.height);
      end.set((target.x - camera.cx) / camera.fx, (target.y - camera.cy) / camera.fy, 1)
        .applyMatrix3(OAK_ROTATION).normalize().multiplyScalar(RAY_LENGTH).add(OAK_LENS);
      ray.material = dashed;
    }
    ray.visible = Boolean(facePoint || target);
    face.visible = Boolean(facePoint);
    if (ray.visible) {
      ray.geometry.setFromPoints([OAK_LENS, end]);
      if (!facePoint) ray.computeLineDistances();
      face.position.copy(end);
    }
    invalidate();
  }

  return {root, update, setVisible(on) { root.visible = on; invalidate(); }};
}
