// Strip charts of the last 30 seconds on one canvas. Motor values are the commands sent: the servos have
// no position feedback and the DC motors no encoders, so no motor position exists to plot. The target's
// offset is the controllers' input, the face distance comes from the Pi's face locator, and the ZED's
// accelerometer and gyroscope are measured, in the IMU's own axes, which are not the head's. Times are the
// Pi's clock (robot_state.time, the IMU's start_time), and the window ends at the newest robot_state time:
// a batch whose clock ran ahead (the IMU's did, 2026-11-06, and blanked every lane) cannot move the window,
// and samples outside it are not drawn. The canvas redraws in the page's frame (frameclock.js),
// at most 4 times a second and only when something new arrived, at one point per pixel column (the first,
// lowest, highest and last value that falls in it), so a long window costs little. The IMU's 794 samples a
// second come in batches of about 53, each kept as its lowest and highest value per trace.
const WINDOW_S = 30;
const DRAW_MS = 250;
const CAPACITY = 4096;  // Samples kept per trace: 30 s at up to about 130 a second

// Lane titles are as short as they can be; the card's i explains them. "cmd" stays on the commanded lanes, which
// are commands and not measured positions (AGENTS.md: label them apart from what is measured).
export const LANES = [
  {id: 'head', title: 'Head cmd', unit: '°', digits: 1, traces: [['pan', '#8de3c2'], ['tilt', '#b8b0f0']], span: 20},
  {id: 'drive', title: 'Drive cmd', unit: '', digits: 2, traces: [['waist', '#ffb269'], ['jaw', '#f08fb8']], fixed: [-1, 1]},
  {id: 'target', title: 'Target', unit: ' px', digits: 0, traces: [['x', '#8de3c2'], ['y', '#b8b0f0']], symmetric: 60},
  {id: 'face', title: 'Face', unit: ' m', digits: 2, traces: [['distance', '#ffb269']], fromZero: 2},
  {id: 'accel', title: 'Accel m/s²', unit: '', digits: 2, traces: [['x', '#ff8a80'], ['y', '#8ee59b'], ['z', '#82aaff']], symmetric: 2},
  {id: 'gyro', title: 'Gyro °/s', unit: '', digits: 1, traces: [['x', '#ff8a80'], ['y', '#8ee59b'], ['z', '#82aaff']], symmetric: 10},
];

const GAP = 8;  // px between lanes
// Lanes in columns: two of three, unless the card is too narrow for two lanes side by side (then one column of six)
// or too short for three rows (then three of two). A lane's header needs about 180 px, its plot 25 px under it.
export function laneColumns(width, height) {
  const laneW = columns => (width - GAP * (columns - 1)) / columns;
  const laneH = columns => { const rows = Math.ceil(LANES.length / columns); return (height - GAP * (rows - 1)) / rows; };
  if (laneW(2) < 180 && laneH(1) >= 40) return 1;
  if (laneH(2) < 40 && laneW(3) >= 180) return 3;
  return 2;
}

class Ring {
  constructor() { this.t = new Float64Array(CAPACITY); this.v = new Float32Array(CAPACITY); this.n = 0; this.next = 0; this.newest = NaN; }
  push(t, v, newest = v) {
    this.t[this.next] = t; this.v[this.next] = v; this.newest = newest;
    this.next = (this.next + 1) % CAPACITY; this.n = Math.min(this.n + 1, CAPACITY);
  }
  // Oldest first, the samples from `since` to `until`.
  forEach(since, until, visit) {
    for (let k = 0, i = (this.next - this.n + CAPACITY) % CAPACITY; k < this.n; k++, i = (i + 1) % CAPACITY) {
      if (this.t[i] >= since && this.t[i] <= until) visit(this.t[i], this.v[i]);
    }
  }
  last() { return this.newest; }  // The newest value, for the header
}

export class TimeSeries {
  // canvas: the strip's canvas; clock: the page's FrameClock.
  constructor(canvas, clock) {
    this.canvas = canvas; this.clock = clock;
    this.rings = new Map(LANES.map(lane => [lane.id, lane.traces.map(() => new Ring())]));
    this.now = null;   // Newest time seen, seconds on the Pi's clock
    this.dirty = true; this.drawnAt = -Infinity; this.timer = 0;
    this.waiting = new Map();  // Lane id -> why it has no data, in place of the lane's own "Waiting for ..."
    this.notes = new Map();    // Lane id -> a word after its title, such as "not sent" while its board is down
    new ResizeObserver(() => this.changed()).observe(canvas);
    clock.add(now => this.frame(now));
  }

  changed() {
    this.dirty = true;
    this.clock.changed();
  }

  // A word after a lane's title (the bot's checks, health.js), or null for none.
  setNote(laneId, text) {
    if ((this.notes.get(laneId) ?? null) === (text ?? null)) return;
    if (text) this.notes.set(laneId, text); else this.notes.delete(laneId);
    this.changed();
  }

  // Why a lane has no data (the bot's checks, health.js), or null for its own words.
  setWaiting(laneId, text) {
    if ((this.waiting.get(laneId) ?? null) === (text ?? null)) return;
    if (text) this.waiting.set(laneId, text); else this.waiting.delete(laneId);
    this.changed();
  }

  // values: one per trace of the lane, null or NaN for a gap.
  push(laneId, t, values) {
    const rings = this.rings.get(laneId);
    if (!rings || !Number.isFinite(t)) return;
    values.forEach((value, i) => rings[i].push(t, value ?? NaN));
    this.now = Math.max(this.now ?? t, t);
    this.changed();
  }

  // A batch of samples for one lane: times, and one array of values per trace. Each trace keeps the batch's
  // lowest and highest value, in the order they came, so a pixel column still spans every sample in it; the
  // header shows the newest. gap: the batch does not follow the previous one, so the line breaks before it.
  pushBatch(laneId, times, traces, gap = false) {
    const rings = this.rings.get(laneId), n = times.length;
    if (!rings || !n) return;
    traces.forEach((values, i) => {
      let low = 0, high = 0;
      for (let k = 1; k < n; k++) { if (values[k] < values[low]) low = k; if (values[k] > values[high]) high = k; }
      if (gap) rings[i].push(times[0], NaN);
      for (const k of low === high ? [low] : low < high ? [low, high] : [high, low]) rings[i].push(times[k], values[k], values[n - 1]);
    });
    this.changed();  // The window still ends at the robot's state time: a batch's own clock never moves it
  }

  // The strip's part of the page's frame: at most every DRAW_MS, and only when something new arrived.
  frame(now) {
    if (!this.dirty || document.hidden) return false;
    const wait = this.drawnAt + DRAW_MS - now;
    if (wait > 0) {
      this.timer ||= setTimeout(() => { this.timer = 0; this.clock.changed(); }, wait);
      return false;
    }
    this.drawnAt = now;
    this.draw();
    return false;
  }

  draw() {
    this.dirty = false;
    const canvas = this.canvas, scale = devicePixelRatio || 1;
    const width = Math.round(canvas.clientWidth * scale), height = Math.round(canvas.clientHeight * scale);
    if (!width || !height) return;
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(scale, 0, 0, scale, 0, 0);
    ctx.clearRect(0, 0, width, height);
    const w = canvas.clientWidth, h = canvas.clientHeight, columns = laneColumns(w, h), rows = Math.ceil(LANES.length / columns);
    const laneW = (w - GAP * (columns - 1)) / columns, laneH = (h - GAP * (rows - 1)) / rows;
    LANES.forEach((lane, i) => {
      const x = Math.floor(i / rows) * (laneW + GAP), y = (i % rows) * (laneH + GAP);
      ctx.save(); ctx.beginPath(); ctx.rect(x, y, laneW, laneH); ctx.clip();  // A long title stops at its lane's edge
      this.drawLane(ctx, lane, x, y, laneW, laneH);
      ctx.restore();
    });
  }

  drawLane(ctx, lane, x, y, w, h) {
    const rings = this.rings.get(lane.id), now = this.now ?? 0, since = now - WINDOW_S;
    const top = y, plotH = Math.max(4, h);  // The plot fills the lane; its label sits over the upper-left corner
    ctx.fillStyle = '#0b1318'; ctx.fillRect(x, top, w, plotH);
    // The value range: fixed, or the window's own with a minimum span.
    let lo = Infinity, hi = -Infinity, any = false;
    rings.forEach(ring => ring.forEach(since, now, (t, v) => { if (Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); any = true; } }));
    if (lane.fixed) [lo, hi] = lane.fixed;
    else if (!any) [lo, hi] = lane.fromZero ? [0, lane.fromZero] : [-(lane.symmetric ?? 1), lane.symmetric ?? 1];
    else if (lane.symmetric) { const m = Math.max(Math.abs(lo), Math.abs(hi), lane.symmetric); lo = -m; hi = m; }
    else if (lane.fromZero) { lo = 0; hi = Math.max(hi * 1.1, lane.fromZero); }
    else { const mid = (lo + hi) / 2, half = Math.max(hi - lo, lane.span) / 2 * 1.1; lo = mid - half; hi = mid + half; }
    const yOf = v => top + plotH - (v - lo) / (hi - lo) * plotH;
    // Grid: every 10 s, and zero when it is in range.
    ctx.strokeStyle = '#1d2b31'; ctx.lineWidth = 1; ctx.beginPath();
    for (let s = 10; s < WINDOW_S; s += 10) { const gx = x + w - s / WINDOW_S * w; ctx.moveTo(gx, top); ctx.lineTo(gx, top + plotH); }
    if (lo < 0 && hi > 0) { ctx.moveTo(x, yOf(0)); ctx.lineTo(x + w, yOf(0)); }
    ctx.stroke();
    const waiting = this.waiting.get(lane.id) ?? lane.waiting;
    if (!any && waiting) {
      ctx.fillStyle = '#5d6f76'; ctx.font = '10px ui-monospace, monospace'; ctx.textBaseline = 'middle';
      ctx.fillText(waiting, x + w / 2 - ctx.measureText(waiting).width / 2, top + plotH / 2);
    }
    // Traces, decimated to the pixel columns; a NaN leaves a gap.
    if (any) lane.traces.forEach(([, color], i) => {
      ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.beginPath();
      let column = null, first = 0, low = 0, high = 0, last = 0, pen = false;
      const flush = () => {
        if (column == null) return;
        const cx = x + column;
        if (!pen) { ctx.moveTo(cx, yOf(first)); pen = true; } else ctx.lineTo(cx, yOf(first));
        if (low !== high) { ctx.lineTo(cx, yOf(low)); ctx.lineTo(cx, yOf(high)); }
        ctx.lineTo(cx, yOf(last));
      };
      rings[i].forEach(since, now, (t, v) => {
        if (!Number.isFinite(v)) { flush(); column = null; pen = false; return; }
        const c = Math.round((t - since) / WINDOW_S * (w - 1));
        if (c !== column) { flush(); column = c; first = low = high = last = v; }
        else { low = Math.min(low, v); high = Math.max(high, v); last = v; }
      });
      flush();
      ctx.stroke();
    });
    const label = this.drawLabel(ctx, lane, rings, x, y, w, h);
    // The scale's ends, small: the top one under the label, the bottom one in the lower-left corner.
    ctx.fillStyle = '#5d6f76'; ctx.font = '10px ui-monospace, monospace'; ctx.textBaseline = 'middle';
    if (plotH > label + 30) { ctx.fillText(hi.toFixed(lane.digits), x + 6, top + label + 12); ctx.fillText(lo.toFixed(lane.digits), x + 6, top + plotH - 7); }
  }

  // The lane's name and each trace's newest value in its colour, large, over the plot's upper-left corner: about a
  // quarter to a third of the lane's height (the owner, 2026-11-07), smaller where the lane is too narrow for the line.
  // Outlined in the plot's colour, so it stays readable over the traces. Returns the text's height.
  drawLabel(ctx, lane, rings, x, y, w, h) {
    const note = this.notes.get(lane.id);
    const parts = [[lane.title, '#c3cdcf'], ...(note ? [[note, '#ff8f7f']] : []), ...lane.traces.map(([name, color], i) => {
      const value = rings[i].last();
      return [`${name} ${Number.isFinite(value) ? value.toFixed(lane.digits) + lane.unit : '—'}`, color];
    })];
    const font = px => `600 ${px}px ui-monospace, monospace`;
    let size = Math.max(12, Math.min(36, Math.round(h * 0.28)));
    ctx.font = font(size);
    const width = parts.reduce((sum, [text]) => sum + ctx.measureText(text).width, 0) + size * 0.6 * (parts.length - 1);
    if (width > w - 12) { size = Math.max(9, Math.floor(size * (w - 12) / width)); ctx.font = font(size); }
    ctx.textBaseline = 'top'; ctx.lineJoin = 'round'; ctx.lineWidth = Math.max(2, size / 5); ctx.strokeStyle = 'rgba(11,19,24,0.85)';
    let at = x + 6;
    for (const [text, color] of parts) {
      ctx.strokeText(text, at, y + 5);
      ctx.fillStyle = color; ctx.fillText(text, at, y + 5);
      at += ctx.measureText(text).width + size * 0.6;
    }
    return size;
  }
}
