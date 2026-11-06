// Where the dashboard's 3D scene draws the ZED's axes and the skull's face for given head commands, computed with the
// dashboard's own headpose.js and the scene graph's rotations (the pan group's yaw, then the tilt group's nod and
// roll in Three.js's XYZ order). Run by tests/test_dashboard_head_pose.py:
//   node head_pose.mjs <path to headpose.js> '[{"pan": 90, "tilt": 90, "roll": 90, "waist": 0}, ...]'
// Prints {cases, headNote}: per case, world directions of the lens axis, image right and image down, the face and the
// top of the head, limitNote's words and which way the waist turns.
import { pathToFileURL } from 'node:url';

const pose = await import(pathToFileURL(process.argv[2]).href);
const { headAngles, H_AXES, OPTICAL_TO_HEAD, limitNote, waistTurn, headNote } = pose;

const apply = (matrix, v) => matrix.map(row => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
const product = (a, b) => a.map(row => [0, 1, 2].map(j => row[0] * b[0][j] + row[1] * b[1][j] + row[2] * b[2][j]));
const rotX = a => [[1, 0, 0], [0, Math.cos(a), -Math.sin(a)], [0, Math.sin(a), Math.cos(a)]];
const rotY = a => [[Math.cos(a), 0, Math.sin(a)], [0, 1, 0], [-Math.sin(a), 0, Math.cos(a)]];
const rotZ = a => [[Math.cos(a), -Math.sin(a), 0], [Math.sin(a), Math.cos(a), 0], [0, 0, 1]];
const inGroup = h => [0, 1, 2].map(i => H_AXES[0][i] * h[0] + H_AXES[1][i] * h[1] + H_AXES[2][i] * h[2]);  // H to head group

const cases = JSON.parse(process.argv[3]).map(command => {
  const {yaw, pitch, roll} = headAngles(command);
  const head = product(rotY(yaw ?? 0), product(rotX(pitch ?? 0), rotZ(roll ?? 0)));
  const fromCamera = v => apply(head, inGroup(apply(OPTICAL_TO_HEAD, v)));
  return {
    lens: fromCamera([0, 0, 1]), imageRight: fromCamera([1, 0, 0]), imageDown: fromCamera([0, 1, 0]),
    face: apply(head, [0, 0, 1]), top: apply(head, [0, 1, 0]), note: limitNote(command), waist: waistTurn(command.waist),
  };
});
console.log(JSON.stringify({cases, headNote: headNote()}));
