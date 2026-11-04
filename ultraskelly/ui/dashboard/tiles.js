// Camera tiles: each camera ID draws into the tile marked with it (data-camera); the tracking tile
// (data-camera="oak") shows whichever camera the head steers by. Each image source, the set of camera
// IDs one frame message carries, has its own newest-wins decode slot: while one of its frames decodes,
// only its newest arrival waits, and no source holds up another. Frames whose tiles cannot be seen are
// not decoded at all. A decoded picture waits for the page's frame (frameclock.js), which a tracking-camera
// picture asks for at once. People are drawn by tracker ID (docs/stream.md): `target` is the person the head
// follows, and its ring comes from that frame's `ultraskelly.target` channel.
import { readImagePayload, decodeImageBitmaps } from '/vendor/freemocap-client.js';
import { drawFinds } from '/faces.js';
import { setText } from '/dom.js';

const KEYPOINT_VISIBLE = 0.3;  // Keypoints below this confidence are not drawn
const STALE_MS = 3000;         // A tile with no frame for this long is dimmed
// Same colors the Pi used to draw on its video.
const COLORS = {bone: 'rgb(0,255,255)', head: 'rgb(255,0,0)', arm: 'rgb(0,255,0)', leg: 'rgb(0,0,255)',
                outline: '#fff', aim: 'rgb(141,227,194)', jaw: 'rgb(255,178,105)', target: 'rgb(255,178,105)',
                other: 'rgba(170,196,206,.75)'};
const IGNORED = new Set(['zed_rectified', 'zed_depth']);  // ZED previews: depth_frame carries both

const hasTarget = state => state?.target && state.target.x != null && state.target.y != null;

// JPEG decoding. WebCodecs' ImageDecoder, where the page may use it (a secure context such as localhost on the
// Pi), decodes the bytes as they are, in about half the time of the SDK's createImageBitmap on a Blob, which also
// sends them through the browser process; elsewhere the SDK decodes, and so does every frame after one WebCodecs
// could not. Either way each picture has the size its header gives and is closed once drawn or replaced.
let webCodecs = typeof ImageDecoder === 'function';
async function decodePictures(payload) {
  if (webCodecs) {
    try { return await decodeWithWebCodecs(payload); }
    catch (problem) { webCodecs = false; console.warn('WebCodecs could not decode a camera frame; using createImageBitmap from now on', problem); }
  }
  return (await decodeImageBitmaps(payload)).map(image => ({...image, picture: image.bitmap}));
}
async function decodeWithWebCodecs(payload) {
  const decoded = await Promise.allSettled(payload.images.map(async image => {
    const decoder = new ImageDecoder({data: image.jpeg, type: 'image/jpeg'});
    try { return (await decoder.decode()).image; } finally { decoder.close(); }
  }));
  const wrong = decoded.findIndex(({status, value}, i) =>
    status === 'rejected' || value.displayWidth !== payload.images[i].width || value.displayHeight !== payload.images[i].height);
  if (wrong >= 0) {
    for (const {value} of decoded) value?.close();
    throw decoded[wrong].reason ?? new Error(`JPEG dimensions disagree with the image header for ${payload.images[wrong].cameraId}`);
  }
  return payload.images.map((image, i) => ({...image, picture: decoded[i].value}));
}

// A tile that just drew: counted for its rate, live and not stale. A forced toggle leaves an unchanged class
// alone, where add and remove would write it again with every frame.
function counted(tile) {
  tile.drawn += 1; tile.lastAt = performance.now();
  tile.figure.classList.toggle('live', true); tile.figure.classList.toggle('stale', false);
}

function sizeCanvas(canvas, width, height) {
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  return canvas.getContext('2d');
}

// From coordinates in an image of `size` [width, height] to the canvas's pixels; no size, no scaling.
function scaleFrom(ctx, size) {
  return size?.[0] > 0 && size?.[1] > 0 ? [ctx.canvas.width / size[0], ctx.canvas.height / size[1]] : [1, 1];
}

// One person's keypoints, in the pixels of their channel's image_size. The followed person (`target`) is drawn
// bold; everyone else thin and muted.
function drawPerson(ctx, overlay, followed) {
  const column = name => overlay.columns.indexOf(name), width = overlay.columns.length;
  const [x, y, visibility] = [column('x'), column('y'), column('visibility')], [sx, sy] = scaleFrom(ctx, overlay.imageSize);
  const point = row => {
    if (row == null) return null;
    const values = overlay.data.subarray(row * width, row * width + width);
    const seen = visibility < 0 || values[visibility] >= KEYPOINT_VISIBLE;
    return seen && Number.isFinite(values[x]) && Number.isFinite(values[y]) ? [values[x] * sx, values[y] * sy] : null;
  };
  const rowOf = new Map(overlay.names.map((name, row) => [name, row]));
  ctx.strokeStyle = followed ? COLORS.bone : COLORS.other; ctx.lineWidth = followed ? 3 : 1.5;
  for (const group of overlay.connectionGroups ?? []) {
    for (const [from, to] of group.pairs) {
      const start = point(rowOf.get(from)), end = point(rowOf.get(to));
      if (start && end) { ctx.beginPath(); ctx.moveTo(...start); ctx.lineTo(...end); ctx.stroke(); }
    }
  }
  overlay.names.forEach((_, row) => {
    const at = point(row);
    if (!at) return;
    // COCO order: five face points, six arm points, then hips and legs.
    ctx.fillStyle = !followed ? COLORS.other : row <= 4 ? COLORS.head : row <= 10 ? COLORS.arm : COLORS.leg;
    ctx.beginPath(); ctx.arc(at[0], at[1], followed ? 5 : 3, 0, Math.PI * 2); ctx.fill();
    if (!followed) return;
    ctx.strokeStyle = COLORS.outline; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(at[0], at[1], 6, 0, Math.PI * 2); ctx.stroke();
  });
}

export class CameraTiles {
  // root holds the tiles; enlarged = {element, canvas, label} shows one camera large; clock: the page's FrameClock.
  constructor(root, enlarged, clock) {
    this.tiles = new Map();
    for (const figure of root.querySelectorAll('.tile[data-camera]')) {
      this.tiles.set(figure.dataset.camera, {figure, canvas: figure.querySelector('canvas'), label: figure.dataset.label, drawn: 0, lastAt: 0});
    }
    this.enlarged = enlarged;
    this.enlargedId = null;    // Tile key of the enlarged view
    this.trackingId = 'oak';   // Camera the head steers by: the one robot_state.overlays describes
    this.state = null;         // Latest robot_state: aim point, jaw zone and target
    this.slots = new Map();    // Source key -> {busy, waiting}
    this.pending = new Map();  // Tile key -> {image, frame}: decoded, waiting for the page's frame
    this.clock = clock;
    clock.add(() => this.present());
    this.unknown = new Set();
    this.visible = new Set();
    this.onChange = null;      // Called when what the page shows changes (wanted views, docs/stream.md)
    this.lastFrameNumber = 0;
    const watch = new IntersectionObserver(entries => {
      for (const entry of entries) this.visible[entry.isIntersecting ? 'add' : 'delete'](entry.target.dataset.camera);
      this.onChange?.();
    });
    for (const {figure} of this.tiles.values()) watch.observe(figure);
  }

  // The tile key for a camera ID, or null when no tile shows it.
  keyOf(cameraId) {
    if (this.tiles.has(cameraId)) return cameraId;
    return cameraId === this.trackingId ? 'oak' : null;
  }

  shows(cameraId) {
    const key = this.keyOf(cameraId);
    return key != null && (this.visible.has(key) || this.enlargedId === key);
  }

  // One image buffer from the stream, with what came with it in the same frame message: frame = {overlays,
  // target, finds}, the keypoints and boxes, the target channel's row, and the OAK's faces and hands (faces.js).
  handle(buffer, frame) {
    let payload;
    try { payload = readImagePayload(buffer); }
    catch (problem) { console.error('Unreadable image payload', problem); return; }
    const ids = payload.images.map(image => image.cameraId);
    this.lastFrameNumber = payload.frameNumber;
    for (const id of ids) {
      if (this.keyOf(id) == null && !IGNORED.has(id) && !this.unknown.has(id)) { this.unknown.add(id); console.warn(`No dashboard tile for camera ${id}`); }
    }
    if (document.hidden || !ids.some(id => this.shows(id))) return;
    const key = ids.join('+');
    let slot = this.slots.get(key);
    if (!slot) this.slots.set(key, slot = {busy: false, waiting: null});
    const job = {payload, frame};
    if (slot.busy) { slot.waiting = job; return; }
    this.decode(slot, job);
  }

  async decode(slot, job) {
    slot.busy = true;
    try {
      for (let next = job; next; next = slot.waiting, slot.waiting = null) {
        for (const image of await decodePictures(next.payload)) this.hold(image, next.frame);
      }
    } catch (problem) {
      console.error('Could not show a camera frame', problem);
    } finally {
      slot.busy = false;
    }
  }

  canvasesFor(key) {
    const tile = this.tiles.get(key);
    return this.enlargedId === key ? [tile.canvas, this.enlarged.canvas] : [tile.canvas];
  }

  // A decoded picture waits for the page's next frame. When a newer one for the same tile comes first, the
  // older is drawn at once rather than lost.
  hold(image, frame) {
    const key = this.keyOf(image.cameraId);
    if (key == null) { image.picture.close(); return; }
    const older = this.pending.get(key);
    if (older) this.show(older);
    this.pending.set(key, {image, frame});
    if (image.cameraId === this.trackingId) this.clock.tracked();
    else this.clock.changed();
  }

  show({image, frame}) {
    try { this.draw(image, frame); }
    finally { image.picture.close(); }
  }

  // The tiles' part of the page's frame: every picture that arrived since the last one.
  present() {
    for (const held of this.pending.values()) this.show(held);
    this.pending.clear();
    return false;
  }

  draw(image, frame) {
    const key = this.keyOf(image.cameraId), tile = this.tiles.get(key);
    if (!tile) return;
    for (const canvas of this.canvasesFor(key)) {
      const ctx = sizeCanvas(canvas, image.width, image.height);
      ctx.drawImage(image.picture, 0, 0);
      this.drawGuides(ctx, image.cameraId, frame);
    }
    counted(tile);
  }

  // A tile drawn elsewhere (the heat map, on its own WebGL canvas): count it, and copy it into the enlarged
  // view when that shows this tile.
  markDrawn(key) {
    const tile = this.tiles.get(key);
    if (!tile) return;
    if (this.enlargedId === key) sizeCanvas(this.enlarged.canvas, tile.canvas.width, tile.canvas.height).drawImage(tile.canvas, 0, 0);
    counted(tile);
  }

  // Aim point, jaw zone, people and the target ring over this camera's image. The ring is the target channel's row
  // from the same frame; without one the head follows nobody. Each comes in its own pixels: the aim point and jaw
  // zone in the camera's (robot_state.cameras), the keypoints and the ring in their channel's image_size. Each is
  // scaled to the image as it arrived, which the server may send smaller than the camera's (docs/stream.md).
  drawGuides(ctx, cameraId, frame, guides = this.state?.overlays?.[cameraId]) {
    if (guides) {
      const camera = this.state?.cameras?.[cameraId], [sx, sy] = scaleFrom(ctx, camera && [camera.width, camera.height]);
      const aimX = guides.aim[0] * sx, aimY = guides.aim[1] * sy, zoneX = guides.jaw_zone * sx, zoneY = guides.jaw_zone * sy;
      ctx.strokeStyle = COLORS.jaw; ctx.lineWidth = 1; ctx.strokeRect(aimX - zoneX, aimY - zoneY, 2 * zoneX, 2 * zoneY);
      ctx.strokeStyle = COLORS.aim; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(aimX - 20, aimY); ctx.lineTo(aimX + 20, aimY); ctx.moveTo(aimX, aimY - 20); ctx.lineTo(aimX, aimY + 20); ctx.stroke();
    }
    // People are the body keypoints; faces' and hands' points and every box draw with the OAK's finds.
    const people = frame.overlays.filter(overlay => overlay.cameraId === cameraId && overlay.layer === 0 && !/^(face|hand)-/.test(overlay.trackerId ?? ''));
    for (const overlay of people) if (overlay.trackerId !== 'target') drawPerson(ctx, overlay, false);
    for (const overlay of people) if (overlay.trackerId === 'target') drawPerson(ctx, overlay, true);  // On top
    if (frame.finds && cameraId === this.trackingId) drawFinds(ctx, frame.finds);
    const target = frame.target;
    if (target && target.cameraId === cameraId && Number.isFinite(target.x) && Number.isFinite(target.y)) {
      const [sx, sy] = scaleFrom(ctx, [target.width, target.height]);
      ctx.strokeStyle = COLORS.target; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.arc(target.x * sx, target.y * sy, 20, 0, Math.PI * 2); ctx.stroke();
    }
  }

  // Why a tile has no picture: its part's {message, fix} from the bot's checks (health.js), or null for the tile's
  // own words, `fallback` or its first ones ("No ZED video yet"), and once it has drawn, that it stopped. Shown
  // over the tile while it has no frames and over its dimmed last frame once they stop.
  explain(key, reason, fallback = null) {
    const tile = this.tiles.get(key), empty = tile?.figure.querySelector('.empty');
    if (!empty) return;
    tile.firstWords ??= empty.firstElementChild.textContent;
    const words = reason?.message ?? fallback ?? (tile.lastAt ? `${tile.label} stopped` : tile.firstWords);
    setText(empty.firstElementChild, words); setText(empty.lastElementChild, reason?.fix ? `Fix: ${reason.fix}` : '');
    tile.figure.classList.toggle('down', Boolean(reason));
  }

  // Without OAK video (a run without cameras, or a camera that stopped), show the target on a grid; the tile's
  // overlay says why (explain).
  showTargetWithoutVideo(state) {
    this.state = state;
    const tile = this.tiles.get('oak');
    if (!tile || (tile.lastAt && performance.now() - tile.lastAt < STALE_MS)) return;
    const id = this.trackingId, camera = state.cameras?.[id], width = camera?.width || 640, height = camera?.height || 480;
    // No video, so only robot_state's unpaired target is left to draw.
    const target = hasTarget(state) ? {cameraId: id, x: state.target.x, y: state.target.y} : null;
    for (const canvas of this.canvasesFor('oak')) {
      const ctx = sizeCanvas(canvas, width, height);
      ctx.fillStyle = '#0b1318'; ctx.fillRect(0, 0, width, height);
      ctx.strokeStyle = '#243840'; ctx.lineWidth = 2;
      for (let x = 0; x <= width; x += 80) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke(); }
      for (let y = 0; y <= height; y += 80) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
      // A run without cameras reports no aim point: use the image center and the default jaw zone.
      this.drawGuides(ctx, id, {overlays: [], target}, state.overlays?.[id] ?? {aim: [width / 2, height / 2], jaw_zone: 100});
    }
  }

  // Each camera this page shows, with the size it is drawn at in device pixels: what the server need encode.
  // Tiles out of view and a hidden page show nothing; the depth tile is not a camera.
  displayImageSizes() {
    const sizes = {}, scale = devicePixelRatio || 1;
    if (document.hidden) return sizes;
    for (const [key, tile] of this.tiles) {
      if (key === 'depth_heat' || !(this.visible.has(key) || this.enlargedId === key)) continue;
      const canvas = this.enlargedId === key ? this.enlarged.canvas : tile.canvas;
      sizes[key === 'oak' ? this.trackingId : key] = {width: Math.round(canvas.clientWidth * scale), height: Math.round(canvas.clientHeight * scale)};
    }
    return sizes;
  }

  // Show one camera large (null closes the view). The tile keeps drawing as well.
  enlarge(cameraId) {
    this.enlargedId = cameraId && this.tiles.has(cameraId) ? cameraId : null;
    this.enlarged.element.hidden = !this.enlargedId;
    this.onChange?.();
    if (!this.enlargedId) return;
    const tile = this.tiles.get(this.enlargedId);
    this.enlarged.label.textContent = tile.label || this.enlargedId;
    const ctx = sizeCanvas(this.enlarged.canvas, tile.canvas.width || 640, tile.canvas.height || 360);
    ctx.clearRect(0, 0, this.enlarged.canvas.width, this.enlarged.canvas.height);
    ctx.drawImage(tile.canvas, 0, 0);
  }

  // Once a second: frames drawn per camera since the last call. Dims tiles whose frames stopped;
  // the OAK tile switches to the target grid instead.
  tick() {
    const drawn = {}, now = performance.now();
    for (const [id, tile] of this.tiles) {
      drawn[id] = tile.drawn; tile.drawn = 0;
      tile.figure.classList.toggle('stale', id !== 'oak' && tile.lastAt > 0 && now - tile.lastAt > STALE_MS);
    }
    return drawn;
  }
}
