// Camera tiles: each camera ID draws into the tile marked with it (data-camera). Each image source,
// the set of camera IDs one frame message carries, has its own newest-wins decode slot: while one of
// its frames decodes, only its newest arrival waits, and no source holds up another. Frames whose
// tiles cannot be seen are not decoded at all.
import { readImagePayload, decodeImageBitmaps } from '/vendor/freemocap-client.js';

const KEYPOINT_VISIBLE = 0.3;  // Keypoints below this confidence are not drawn
const STALE_MS = 3000;         // A tile with no frame for this long is dimmed
// Same colors the Pi used to draw on its video.
const COLORS = {bone: 'rgb(0,255,255)', head: 'rgb(255,0,0)', arm: 'rgb(0,255,0)', leg: 'rgb(0,0,255)',
                outline: '#fff', aim: 'rgb(141,227,194)', jaw: 'rgb(255,178,105)', target: 'rgb(255,178,105)'};

const hasTarget = state => state?.target && state.target.x != null && state.target.y != null;

function sizeCanvas(canvas, width, height) {
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  return canvas.getContext('2d');
}

function drawPerson(ctx, overlay) {
  const column = name => overlay.columns.indexOf(name), width = overlay.columns.length;
  const [x, y, visibility] = [column('x'), column('y'), column('visibility')];
  const point = row => {
    if (row == null) return null;
    const values = overlay.data.subarray(row * width, row * width + width);
    const seen = visibility < 0 || values[visibility] >= KEYPOINT_VISIBLE;
    return seen && Number.isFinite(values[x]) && Number.isFinite(values[y]) ? [values[x], values[y]] : null;
  };
  const rowOf = new Map(overlay.names.map((name, row) => [name, row]));
  ctx.strokeStyle = COLORS.bone; ctx.lineWidth = 2;
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
    ctx.fillStyle = row <= 4 ? COLORS.head : row <= 10 ? COLORS.arm : COLORS.leg;
    ctx.beginPath(); ctx.arc(at[0], at[1], 5, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = COLORS.outline; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(at[0], at[1], 6, 0, Math.PI * 2); ctx.stroke();
  });
}

export class CameraTiles {
  // root holds the tiles; enlarged = {element, canvas, label} shows one camera large.
  constructor(root, enlarged) {
    this.tiles = new Map();
    for (const figure of root.querySelectorAll('.tile[data-camera]')) {
      this.tiles.set(figure.dataset.camera, {figure, canvas: figure.querySelector('canvas'), label: figure.dataset.label, drawn: 0, lastAt: 0});
    }
    this.enlarged = enlarged;
    this.enlargedId = null;
    this.state = null;         // Latest robot_state: aim point, jaw zone and target
    this.slots = new Map();    // Source key -> {busy, waiting}
    this.unknown = new Set();
    this.visible = new Set();
    const watch = new IntersectionObserver(entries => {
      for (const entry of entries) this.visible[entry.isIntersecting ? 'add' : 'delete'](entry.target.dataset.camera);
    });
    for (const {figure} of this.tiles.values()) watch.observe(figure);
  }

  shows(cameraId) {
    return this.tiles.has(cameraId) && (this.visible.has(cameraId) || this.enlargedId === cameraId);
  }

  // One image buffer from the stream, with the overlays (poses) of the same frame.
  handle(buffer, overlays) {
    let payload;
    try { payload = readImagePayload(buffer); }
    catch (problem) { console.error('Unreadable image payload', problem); return; }
    const ids = payload.images.map(image => image.cameraId);
    for (const id of ids) {
      if (!this.tiles.has(id) && !this.unknown.has(id)) { this.unknown.add(id); console.warn(`No dashboard tile for camera ${id}`); }
    }
    if (document.hidden || !ids.some(id => this.shows(id))) return;
    const key = ids.join('+');
    let slot = this.slots.get(key);
    if (!slot) this.slots.set(key, slot = {busy: false, waiting: null});
    const job = {payload, overlays};
    if (slot.busy) { slot.waiting = job; return; }
    this.decode(slot, job);
  }

  async decode(slot, job) {
    slot.busy = true;
    try {
      for (let next = job; next; next = slot.waiting, slot.waiting = null) {
        const images = await decodeImageBitmaps(next.payload);
        try { for (const image of images) this.draw(image, next.overlays); }
        finally { for (const image of images) image.bitmap.close(); }
      }
    } catch (problem) {
      console.error('Could not show a camera frame', problem);
    } finally {
      slot.busy = false;
    }
  }

  canvasesFor(cameraId) {
    const tile = this.tiles.get(cameraId);
    return this.enlargedId === cameraId ? [tile.canvas, this.enlarged.canvas] : [tile.canvas];
  }

  draw(image, overlays) {
    const tile = this.tiles.get(image.cameraId);
    if (!tile) return;
    for (const canvas of this.canvasesFor(image.cameraId)) {
      const ctx = sizeCanvas(canvas, image.width, image.height);
      ctx.drawImage(image.bitmap, 0, 0);
      this.drawGuides(ctx, image.cameraId, overlays);
    }
    tile.drawn += 1; tile.lastAt = performance.now();
    tile.figure.classList.add('live'); tile.figure.classList.remove('stale');
  }

  // Aim point, jaw zone, poses and the target ring, in the pixels of this camera's image.
  drawGuides(ctx, cameraId, overlays, guides = this.state?.overlays?.[cameraId]) {
    if (guides) {
      const [aimX, aimY] = guides.aim, zone = guides.jaw_zone;
      ctx.strokeStyle = COLORS.jaw; ctx.lineWidth = 1; ctx.strokeRect(aimX - zone, aimY - zone, 2 * zone, 2 * zone);
      ctx.strokeStyle = COLORS.aim; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(aimX - 20, aimY); ctx.lineTo(aimX + 20, aimY); ctx.moveTo(aimX, aimY - 20); ctx.lineTo(aimX, aimY + 20); ctx.stroke();
    }
    for (const overlay of overlays) if (overlay.cameraId === cameraId) drawPerson(ctx, overlay);
    if (guides && hasTarget(this.state)) {
      ctx.strokeStyle = COLORS.target; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.arc(this.state.target.x, this.state.target.y, 20, 0, Math.PI * 2); ctx.stroke();
    }
  }

  // Without OAK video (a run without cameras, or a camera that stopped), show the target on a grid.
  showTargetWithoutVideo(state) {
    this.state = state;
    const tile = this.tiles.get('oak');
    if (!tile || (tile.lastAt && performance.now() - tile.lastAt < STALE_MS)) return;
    const camera = state.cameras?.oak, width = camera?.width || 640, height = camera?.height || 480;
    for (const canvas of this.canvasesFor('oak')) {
      const ctx = sizeCanvas(canvas, width, height);
      ctx.fillStyle = '#0b1318'; ctx.fillRect(0, 0, width, height);
      ctx.strokeStyle = '#243840'; ctx.lineWidth = 2;
      for (let x = 0; x <= width; x += 80) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke(); }
      for (let y = 0; y <= height; y += 80) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
      // A run without cameras reports no aim point: use the image center and the default jaw zone.
      this.drawGuides(ctx, 'oak', [], state.overlays?.oak ?? {aim: [width / 2, height / 2], jaw_zone: 100});
      ctx.fillStyle = '#91a2a8'; ctx.font = '16px system-ui, sans-serif';
      ctx.fillText(hasTarget(state) ? 'No video: target position only' : 'No video and no target', 16, height - 18);
    }
  }

  // Show one camera large (null closes the view). The tile keeps drawing as well.
  enlarge(cameraId) {
    this.enlargedId = cameraId && this.tiles.has(cameraId) ? cameraId : null;
    this.enlarged.element.hidden = !this.enlargedId;
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
