// How the commanded head angles pose the 3D scene's skull, and where the head frame H sits on it (docs/dashboard.md,
// "3D scene"). One place for the convention, so the scene, its depth and tests/test_dashboard_head_pose.py agree.
//
// Servo degrees, 90 level and straight ahead, one servo degree drawn as one degree of the head. Pan above 90 turns
// the head to its own left, tilt above 90 nods it down, roll above 90 tips it toward its left shoulder. Pan and tilt
// are the simulator's convention (ultraskelly/simulator/engine.py), the one the head controller converges with; the
// real servos' directions are not yet checked (docs/hardware.md). Commands, not measured positions.

export const LEVEL = 90;
const LOW = 0, HIGH = 180;  // The commands' full range (ActuatorLimits); the controller can drive the head to either
const rad = degrees => degrees * Math.PI / 180;

// Joint angles in radians, null where the command is unknown. The skeleton's pan group turns about its up axis (y);
// the tilt group nods about the head's left axis (x) and rolls about the face's axis (z), in Three.js's XYZ order:
// the roll turns the head about its own nose, then the nod tips both.
export function headAngles({pan = null, tilt = null, roll = null} = {}) {
  return {yaw: pan == null ? null : rad(pan - LEVEL), pitch: tilt == null ? null : rad(tilt - LEVEL),
          roll: roll == null ? null : -rad(roll - LEVEL)};
}

// The head frame H (docs/depth-stream.md: origin midway between the eye sockets, x forward out of the face, y to the
// skull's left, z up) in the skeleton head group's axes (+x the skull's left, +y up, +z out of the face): H's x, y
// and z axes as group vectors, the columns of the basis that places depth on the head.
export const H_AXES = [[0, 0, 1], [1, 0, 0], [0, 1, 0]];

// The ZED looking straight out of the face, as the bot's nominal camera_to_head (head_frame.py, OPTICAL_TO_HEAD):
// rows taking camera axes (x image right, y image down, z out of the lens) into H. Depth along the lens is H's x,
// out of the face, never height.
export const OPTICAL_TO_HEAD = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]];

// A command at the end of its range, in words, or null: what the scene draws there, so a skull nodded into the
// floor reads as the command it is.
const ENDS = {pan: ['turned fully to its right', 'turned fully to its left'], tilt: ['tipped fully up', 'nodded fully down']};
export function limitNote(commands) {
  const notes = [];
  for (const [name, [low, high]] of Object.entries(ENDS)) {
    const value = commands?.[name];
    if (value == null) continue;
    if (value <= LOW + 0.5) notes.push(`${name[0].toUpperCase()}${name.slice(1)} commanded to its ${LOW}° limit: drawn ${low}`);
    else if (value >= HIGH - 0.5) notes.push(`${name[0].toUpperCase()}${name.slice(1)} commanded to its ${HIGH}° limit: drawn ${high}`);
  }
  return notes.length ? notes.join(' · ') : null;
}
