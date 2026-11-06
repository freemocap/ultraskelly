// Where the dashboard's 3D scene draws the ZED's axes and the skull's face for given head commands, computed with the
// dashboard's own headpose.js and the scene graph's rotations (the pan group's yaw, then the tilt group's nod and
// roll in Three.js's XYZ order). Run by tests/test_dashboard_head_pose.py:
//   node head_pose.mjs <path to headpose.js> '[{"pan": 90, "tilt": 90, "roll": 90}, ...]'
// Prints, per case, world directions: the lens axis, image right and image down, the face and the top of the head,
// and limitNote's words.
import { pathToFileURL } from 'node:url';

const { headAngles, H_AXES, OPTICAL_TO_HEAD, limitNote } = await import(pathToFileURL(process.argv[2]).href);
const cases = JSON.parse(process.argv[3]);

const apply = (matrix, v) => matrix.map(row => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
const product = (a, b) => a.map(row => [0, 1, 2].map(j => row[0] * b[0][j] + row[1] * b[1][j] + row[2] * b[2][j]));
const rotX = a => [[1, 0, 0], [0, Math.cos(a), -Math.sin(a)], [0, Math.sin(a), Math.cos(a)]];
const rotY = a => [[Math.cos(a), 0, Math.sin(a)], [0, 1, 0], [-Math.sin(a), 0, Math.cos(a)]];
const rotZ = a => [[Math.cos(a), -Math.sin(a), 0], [Math.sin(a), Math.cos(a), 0], [0, 0, 1]];
const inGroup = h => [0, 1, 2].map(i => H_AXES[0][i] * h[0] + H_AXES[1][i] * h[1] + H_AXES[2][i] * h[2]);  // H to head group

const results = cases.map(command => {
  const {yaw, pitch, roll} = headAngles(command);
  const head = product(rotY(yaw ?? 0), product(rotX(pitch ?? 0), rotZ(roll ?? 0)));
  const fromCamera = v => apply(head, inGroup(apply(OPTICAL_TO_HEAD, v)));
  return {
    lens: fromCamera([0, 0, 1]), imageRight: fromCamera([1, 0, 0]), imageDown: fromCamera([0, 1, 0]),
    face: apply(head, [0, 0, 1]), top: apply(head, [0, 1, 0]), note: limitNote(command),
  };
});
console.log(JSON.stringify(results));
