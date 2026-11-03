// Faces and hands found by the OAK's own chip (docs/stream.md, "Faces" and "Hands, found by the OAK's own
// chip"), drawn over the tracking tile in the pixels the stream gives them in. Every frame carries its faces'
// boxes and five landmarks; head pose and gaze come every second frame, the 468-point mesh every fourth and hands
// every sixth, absent in between. A face ID names a face in one frame only, so each face keeps the measurements
// of the previous frame's face its box overlaps most, until they are too old to show; hands stay until the next
// ones come or a quarter of a second passes.
const HOLD_MS = {headPose: 300, gaze: 300, mesh: 500};
const HAND_HOLD_MS = 250;
const MEASURES = {'ultraskelly.head_pose': 'headPose', 'ultraskelly.gaze': 'gaze', 'ultraskelly.face_mesh': 'mesh',
                  'ultraskelly.handedness': 'handedness'};
const COLORS = {face: '#4fc3f7', hand: '#ffb74d', mesh: 'rgba(200,235,255,.55)', axes: ['#ff6e6e', '#7ee787', '#6ea8ff'],
                gaze: '#ffe066'};

// The measurement channels of one frame message, copied out of the decoder's storage, which a later frame may
// reuse: trackerId -> {headPose, gaze, mesh, handedness}, each a Float32Array.
export function measuresOf(views) {
  const measures = new Map();
  for (const view of views?.channels ?? []) {
    const name = MEASURES[view.descriptor.kind], id = view.owner?.trackerId;
    if (!name || !id) continue;
    if (!measures.has(id)) measures.set(id, {});
    measures.get(id)[name] = view.data.slice();
  }
  return measures;
}

// An overlay's points, copied for drawing later.
const copyOf = overlay => overlay && {names: overlay.names, columns: overlay.columns, data: overlay.data.slice(),
                                      imageSize: overlay.imageSize, connectionGroups: overlay.connectionGroups};

function boxOf(overlay) {
  const at = name => overlay.data[overlay.columns.indexOf(name)];
  return {x1: at('x1'), y1: at('y1'), x2: at('x2'), y2: at('y2'), confidence: at('confidence'), imageSize: overlay.imageSize};
}

function overlap(a, b) {  // Intersection over union of two boxes
  const w = Math.min(a.x2, b.x2) - Math.max(a.x1, b.x1), h = Math.min(a.y2, b.y2) - Math.max(a.y1, b.y1);
  if (w <= 0 || h <= 0) return 0;
  const area = box => (box.x2 - box.x1) * (box.y2 - box.y1);
  return w * h / (area(a) + area(b) - w * h);
}

export class OakFinds {
  constructor() { this.faces = []; this.hands = []; this.handsAt = -Infinity; }

  // One OAK frame message: its overlays (people, faces and hands; boxes and points) and its measurements.
  // Returns {faces, hands} to draw over that frame's image.
  update(overlays, measures, now = performance.now()) {
    const trackers = new Map();
    for (const overlay of overlays) {
      if (!/^(face|hand)-/.test(overlay.trackerId ?? '')) continue;
      if (!trackers.has(overlay.trackerId)) trackers.set(overlay.trackerId, {});
      trackers.get(overlay.trackerId)[overlay.layer === 2 ? 'box' : 'points'] = overlay;
    }
    const faces = [], hands = [];
    for (const [id, {box, points}] of trackers) {
      if (!box) continue;
      const found = {box: boxOf(box), points: copyOf(points), ...measures.get(id)};
      if (id.startsWith('hand-')) { hands.push(found); continue; }
      found.at = {};
      let previous = null, best = 0.3;  // At least this much overlap to be the same face
      for (const face of this.faces) { const o = overlap(face.box, found.box); if (o > best) { best = o; previous = face; } }
      for (const name of Object.keys(HOLD_MS)) {
        if (found[name]) found.at[name] = now;
        else if (previous?.[name] && now - previous.at[name] < HOLD_MS[name]) { found[name] = previous[name]; found.at[name] = previous.at[name]; }
      }
      faces.push(found);
    }
    this.faces = faces;
    if (hands.length) { this.hands = hands; this.handsAt = now; }
    else if (now - this.handsAt > HAND_HOLD_MS) this.hands = [];
    return {faces: this.faces, hands: this.hands};
  }
}

// From a channel's image_size to the canvas's pixels.
function scaleOf(ctx, size) {
  return size?.[0] > 0 && size?.[1] > 0 ? [ctx.canvas.width / size[0], ctx.canvas.height / size[1]] : [1, 1];
}

function pointsOf(overlay, sx, sy) {  // name -> [x, y] in canvas pixels, for the points seen
  const columns = overlay.columns, width = columns.length, [x, y] = [columns.indexOf('x'), columns.indexOf('y')];
  const at = new Map();
  overlay.names.forEach((name, row) => {
    const px = overlay.data[row * width + x], py = overlay.data[row * width + y];
    if (Number.isFinite(px) && Number.isFinite(py)) at.set(name, [px * sx, py * sy]);
  });
  return at;
}

function drawBones(ctx, overlay, at, color, width) {
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
  for (const group of overlay.connectionGroups ?? []) {
    for (const [from, to] of group.pairs) {
      const start = at.get(from), end = at.get(to);
      if (start && end) { ctx.moveTo(...start); ctx.lineTo(...end); }
    }
  }
  ctx.stroke();
}

function drawDots(ctx, at, color, radius) {
  ctx.fillStyle = color; ctx.beginPath();
  for (const [x, y] of at.values()) { ctx.moveTo(x + radius, y); ctx.arc(x, y, radius, 0, Math.PI * 2); }
  ctx.fill();
}

// Head-pose axes as OpenVINO's head-pose demos draw them: R = Rz(roll) Ry(yaw) Rx(pitch) applied to the head's
// x (red), up (green) and forward (blue) axes, seen straight on from the camera.
function drawHeadPose(ctx, [yawDeg, pitchDeg, rollDeg], cx, cy, length) {
  const [y, p, r] = [yawDeg, pitchDeg, rollDeg].map(degrees => degrees * Math.PI / 180);
  const [cyw, syw, cp, sp, cr, sr] = [Math.cos(y), Math.sin(y), Math.cos(p), Math.sin(p), Math.cos(r), Math.sin(r)];
  const rx = [[1, 0, 0], [0, cp, -sp], [0, sp, cp]], ry = [[cyw, 0, -syw], [0, 1, 0], [syw, 0, cyw]], rz = [[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]];
  const times = (a, b) => a.map(row => b[0].map((_, j) => row.reduce((sum, value, k) => sum + value * b[k][j], 0)));
  const rotation = times(times(rz, ry), rx);
  [[1, 0, 0], [0, -1, 0], [0, 0, -1]].forEach((axis, i) => {
    const v = rotation.map(row => row[0] * axis[0] + row[1] * axis[1] + row[2] * axis[2]);
    ctx.strokeStyle = COLORS.axes[i]; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + length * v[0], cy + length * v[1]); ctx.stroke();
  });
}

function drawArrow(ctx, x, y, dx, dy, color) {
  const length = Math.hypot(dx, dy);
  if (length < 1) return;
  const [ux, uy] = [dx / length, dy / length], head = Math.min(8, length / 2);
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(x, y); ctx.lineTo(x + dx, y + dy);
  ctx.moveTo(x + dx, y + dy); ctx.lineTo(x + dx - head * (ux - uy * .5), y + dy - head * (uy + ux * .5));
  ctx.moveTo(x + dx, y + dy); ctx.lineTo(x + dx - head * (ux + uy * .5), y + dy - head * (uy - ux * .5));
  ctx.stroke();
}

// Every face and hand of one frame over its image: ctx is the tile's canvas, in the image's pixels.
export function drawFinds(ctx, {faces, hands}) {
  for (const face of faces) {
    const {box} = face, [sx, sy] = scaleOf(ctx, box.imageSize);
    const x1 = box.x1 * sx, y1 = box.y1 * sy, width = (box.x2 - box.x1) * sx, height = (box.y2 - box.y1) * sy;
    if (face.mesh) {  // MediaPipe's 468 points: x, y in pixels of the same image
      ctx.fillStyle = COLORS.mesh; ctx.beginPath();
      for (let i = 0; i + 2 < face.mesh.length; i += 3) ctx.rect(face.mesh[i] * sx - .75, face.mesh[i + 1] * sy - .75, 1.5, 1.5);
      ctx.fill();
    }
    ctx.strokeStyle = COLORS.face; ctx.lineWidth = 1.5; ctx.strokeRect(x1, y1, width, height);
    const at = face.points ? pointsOf(face.points, sx, sy) : new Map();
    if (face.points) { drawBones(ctx, face.points, at, COLORS.face, 1); drawDots(ctx, at, COLORS.face, 2.5); }
    const nose = at.get('nose') ?? [x1 + width / 2, y1 + height / 2];
    if (face.headPose) drawHeadPose(ctx, face.headPose, nose[0], nose[1], width * .5);
    const [right, left] = [at.get('right_eye'), at.get('left_eye')];
    if (face.gaze && right && left) {  // From between the eyes; the gaze's y is up, the image's down
      const length = .4 * width;
      drawArrow(ctx, (right[0] + left[0]) / 2, (right[1] + left[1]) / 2, face.gaze[0] * length, -face.gaze[1] * length, COLORS.gaze);
    }
  }
  for (const hand of hands) {
    const {box} = hand, [sx, sy] = scaleOf(ctx, box.imageSize);
    ctx.strokeStyle = 'rgba(255,183,77,.6)'; ctx.lineWidth = 1; ctx.setLineDash([4, 3]);
    ctx.strokeRect(box.x1 * sx, box.y1 * sy, (box.x2 - box.x1) * sx, (box.y2 - box.y1) * sy); ctx.setLineDash([]);
    if (!hand.points) continue;
    const at = pointsOf(hand.points, sx, sy);
    drawBones(ctx, hand.points, at, COLORS.hand, 2); drawDots(ctx, at, '#fff3e0', 2);
    const wrist = at.get('wrist');
    if (wrist && hand.handedness) {  // The landmarker's score, not yet checked on this unmirrored camera
      const right = hand.handedness[0];
      ctx.font = '12px system-ui, sans-serif'; ctx.fillStyle = COLORS.hand;
      ctx.fillText(`${right > .5 ? 'right' : 'left'} ${(right > .5 ? right : 1 - right).toFixed(2)}`, wrist[0] + 6, wrist[1] + 14);
    }
  }
}
