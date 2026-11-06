// How the commanded head angles pose the 3D scene's skull, and where the head frame H sits on it (docs/dashboard.md,
// "3D scene"). One place for the convention, so the scene, its depth and tests/test_dashboard_head_pose.py agree.
//
// The head is drawn from the servo commands at the ratios measured on the robot, so it is approximate: on 2026-11-06
// scripts/check_motor_directions.py (OAK image upright, about 9.3 pixels per degree) saw 12 servo degrees move the
// picture 11-35 px for pan and 9-10 px for tilt. More pan turns the head to its own right, more tilt nods it down,
// and negative waist drive turns the body to its right. Roll is held at 90 and its direction and ratio are unchecked.
// The ZED's IMU cannot pose the head yet: its axes have no measured place in the head (docs/stream.md, "imu").

export const LEVEL = 90;  // Servo degrees the head is commanded to rest at
const LOW = 0, HIGH = 180;  // The commands' full range (ActuatorLimits)
const rad = degrees => degrees * Math.PI / 180;

export const HEAD_PER_SERVO = {pan: 0.2, tilt: 0.1, roll: 1};  // Degrees of head per servo degree; roll not measured
// Each servo's turn onto the scene's joints: the pan group's +y turns the head to its own left, the tilt group's +x
// nods it down and its +z tips the top of the head to its right.
const TURN = {pan: -1, tilt: 1, roll: -1};
// Where the real head points at 90/90, degrees up and to its own left. It looks up at the ceiling there (2026-11-06);
// how far is not measured yet, so the scene draws it level.
export const HEAD_AT_LEVEL = {up: 0, left: 0};

// Joint angles in radians, null where the command is unknown. The skeleton's pan group turns about its up axis (y);
// the tilt group nods about the head's left axis (x) and rolls about the face's axis (z), in Three.js's XYZ order:
// the roll turns the head about its own nose, then the nod tips both.
export function headAngles({pan = null, tilt = null, roll = null} = {}) {
  const turned = (name, value) => TURN[name] * rad(HEAD_PER_SERVO[name] * (value - LEVEL));
  return {yaw: pan == null ? null : rad(HEAD_AT_LEVEL.left) + turned('pan', pan),
          pitch: tilt == null ? null : -rad(HEAD_AT_LEVEL.up) + turned('tilt', tilt),
          roll: roll == null ? null : turned('roll', roll)};
}

// Which way a waist drive turns the body: 1 to its own left, -1 to its right, 0 for none.
export const waistTurn = drive => drive == null || Math.abs(drive) <= 0.01 ? 0 : drive < 0 ? -1 : 1;

// The scene's own words for how its head is drawn.
export const headNote = () =>
  `Head approximate: ${HEAD_PER_SERVO.pan}° of turn and ${HEAD_PER_SERVO.tilt}° of nod per servo degree, as measured`;

// The head frame H (docs/depth-stream.md: origin midway between the eye sockets, x forward out of the face, y to the
// skull's left, z up) in the skeleton head group's axes (+x the skull's left, +y up, +z out of the face): H's x, y
// and z axes as group vectors, the columns of the basis that places depth on the head. The depth stays rigid in H.
export const H_AXES = [[0, 0, 1], [1, 0, 0], [0, 1, 0]];

// The ZED looking straight out of the face, as the bot's nominal camera_to_head (head_frame.py, OPTICAL_TO_HEAD):
// rows taking camera axes (x image right, y image down, z out of the lens) into H. Depth along the lens is H's x,
// out of the face, never height.
export const OPTICAL_TO_HEAD = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]];

// A head command at the end of its range, or null: the tracking controller winds up there while the head barely
// moves, which drew the skull into the floor when servo degrees were drawn as head degrees.
export function limitNote(commands) {
  const notes = [];
  for (const name of ['pan', 'tilt']) {
    const value = commands?.[name];
    if (value == null) continue;
    const end = value <= LOW + 0.5 ? LOW : value >= HIGH - 0.5 ? HIGH : null;
    if (end != null) notes.push(`${name[0].toUpperCase()}${name.slice(1)} commanded to its ${end}° limit`);
  }
  return notes.length ? notes.join(' · ') : null;
}
