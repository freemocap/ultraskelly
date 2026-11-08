// The person the head follows, in 3D: a glowing stick figure of their 17 COCO keypoints as the Pi's face locator
// places them on the ZED's depth (robot_state.face.skeleton_head: the head frame H, metres; +x forward, +y the
// skull's left, +z up). It lives in the gaze layer, in the point cloud's frame, and shows and hides with it. Camera
// poses are nominal, so where it stands in the room is an estimate.
import * as THREE from 'three';

// COCO: 0 nose, 1-2 eyes, 3-4 ears, 5-6 shoulders, 7-8 elbows, 9-10 wrists, 11-12 hips, 13-14 knees, 15-16 ankles
const BONES = [[0, 1], [0, 2], [1, 3], [2, 4], [5, 6], [5, 7], [7, 9], [6, 8], [8, 10], [5, 11], [6, 12], [11, 12],
               [11, 13], [13, 15], [12, 14], [14, 16]];
const COLOR = 0x6ff2ff;
const BONE_RADIUS = 0.012, JOINT_RADIUS = 0.022, HALO = 2.6;  // Metres; the halo's size against the core's
const LEAST_JOINTS = 3;  // Fewer known keypoints draw nothing

// group: the gaze layer's root, in H; invalidate: asks the scene to draw again.
export function createPersonLayer({group, invalidate}) {
  const root = new THREE.Group();
  root.visible = false;
  group.add(root);
  // A bright core and a faint wide halo, both adding light to whatever is behind them: a glow on the dark scene
  const glow = opacity => new THREE.MeshBasicMaterial({color: COLOR, transparent: true, opacity,
                                                         blending: THREE.AdditiveBlending, depthWrite: false});
  const core = glow(0.9), halo = glow(0.22);
  const tube = new THREE.CylinderGeometry(1, 1, 1, 8, 1, true), ball = new THREE.SphereGeometry(1, 12, 8);
  function part(geometry, radius, length) {  // A core inside its halo, scaled to `radius` (and `length` for a bone)
    const holder = new THREE.Group();
    for (const [material, size] of [[core, radius], [halo, radius * HALO]]) {
      const mesh = new THREE.Mesh(geometry, material);
      mesh.scale.set(size, length ?? size, size);
      holder.add(mesh);
    }
    root.add(holder);
    return holder;
  }
  const bones = BONES.map(() => part(tube, BONE_RADIUS, 1));  // Unit length: each bone's holder stretches it
  const joints = Array.from({length: 17}, () => part(ball, JOINT_RADIUS));
  const up = new THREE.Vector3(0, 1, 0), from = new THREE.Vector3(), to = new THREE.Vector3(), along = new THREE.Vector3();
  let lastKey = null;

  // points: 17 [x, y, z] in H, each null where unknown; or null while following nobody.
  function update(points) {
    const key = points ? JSON.stringify(points) : '-';
    if (key === lastKey) return;  // Nothing new to draw
    lastKey = key;
    root.visible = Boolean(points) && points.filter(Boolean).length >= LEAST_JOINTS;
    if (root.visible) {
      joints.forEach((joint, index) => {
        joint.visible = Boolean(points[index]);
        if (joint.visible) joint.position.fromArray(points[index]);
      });
      bones.forEach((bone, index) => {
        const [a, b] = BONES[index];
        bone.visible = Boolean(points[a] && points[b]);
        if (!bone.visible) return;
        from.fromArray(points[a]); to.fromArray(points[b]);
        along.subVectors(to, from);
        bone.position.addVectors(from, to).multiplyScalar(0.5);
        bone.scale.set(1, along.length(), 1);
        bone.quaternion.setFromUnitVectors(up, along.normalize());
      });
    }
    invalidate();
  }

  return {root, update};
}
