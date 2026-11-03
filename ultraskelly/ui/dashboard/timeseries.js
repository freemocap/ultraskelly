// Strip charts of the last 30 seconds on one canvas. Motor values are the commands sent: the servos have
// no position feedback and the DC motors no encoders, so no motor position exists to plot. The target's
// offset is the controllers' input, the face distance comes from the Pi's face locator, and the ZED's
// accelerometer and gyroscope are measured. Times are the Pi's (robot_state.time). The canvas redraws at
// most 10 times a second and only when something new arrived, at one point per pixel column (the first,
// lowest, highest and last value that falls in it), so a long window or a fast sensor costs little.
const WINDOW_S = 30;
const DRAW_MS = 100;
const CAPACITY = 4096;  // Samples kept per trace: 30 s at up to about 130 a second

export const LANES = [
  {id: 'head', title: 'Head, commanded', unit: '°', digits: 1, traces: [['pan', '#8de3c2'], ['tilt', '#b8b0f0']], span: 20},
  {id: 'drive', title: 'Waist and jaw drive, commanded', unit: '', digits: 2, traces: [['waist', '#ffb269'], ['jaw', '#f08fb8']], fixed: [-1, 1]},
  {id: 'target', title: 'Target from the aim point', unit: ' px', digits: 0, traces: [['x', '#8de3c2'], ['y', '#b8b0f0']], symmetric: 60},
  {id: 'face', title: 'Face distance, from depth', unit: ' m', digits: 2, traces: [['distance', '#ffb269']], fromZero: 2},
  {id: 'accel', title: 'ZED accelerometer, measured', unit: ' m/s²', digits: 2, traces: [['x', '#ff8a80'], ['y', '#8ee59b'], ['z', '#82aaff']], symmetric: 2,
   waiting: 'Waiting for the ZED IMU'},
  {id: 'gyro', title: 'ZED gyroscope, measured', unit: ' °/s', digits: 1, traces: [['x', '#ff8a80'], ['y', '#8ee59b'], ['z', '#82aaff']], symmetric: 10,
   waiting: 'Waiting for the ZED IMU'},
];

class Ring {
  constructor() { this.t = new Float64Array(CAPACITY); this.v = new Float32Array(CAPACITY); this.n = 0; this.next = 0; }
  push(t, v) {
    this.t[this.next] = t; this.v[this.next] = v;
    this.next = (this.next + 1) % CAPACITY; this.n = Math.min(this.n + 1, CAPACITY);
  }
  // Oldest first, from the first sample at or after `since`.
  forEach(since, visit) {
    for (let k = 0, i = (this.next - this.n + CAPACITY) % CAPACITY; k < this.n; k++, i = (i + 1) % CAPACITY) {
      if (this.t[i] >= since) visit(this.t[i], this.v[i]);
    }
  }
  last() { return this.n ? this.v[(this.next - 1 + CAPACITY) % CAPACITY] : NaN; }
}

export class TimeSeries {
  constructor(canvas) {
    this.canvas = canvas;
    this.rings = new Map(LANES.map(lane => [lane.id, lane.traces.map(() => new Ring())]));
    this.now = null;   // Newest time seen, seconds on the Pi's clock
    this.dirty = true;
    new ResizeObserver(() => { this.dirty = true; }).observe(canvas);
    setInterval(() => { if (this.dirty && !document.hidden) this.draw(); }, DRAW_MS);
  }

  // values: one per trace of the lane, null or NaN for a gap.
  push(laneId, t, values) {
    const rings = this.rings.get(laneId);
    if (!rings || !Number.isFinite(t)) return;
    values.forEach((value, i) => rings[i].push(t, value ?? NaN));
    this.now = Math.max(this.now ?? t, t);
    this.dirty = true;
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
    const w = canvas.clientWidth, h = canvas.clientHeight, columns = 2, rows = Math.ceil(LANES.length / columns), gap = 8;
    const laneW = (w - gap * (columns - 1)) / columns, laneH = (h - gap * (rows - 1)) / rows;
    LANES.forEach((lane, i) => this.drawLane(ctx, lane, Math.floor(i / rows) * (laneW + gap), (i % rows) * (laneH + gap), laneW, laneH));
  }

  drawLane(ctx, lane, x, y, w, h) {
    const rings = this.rings.get(lane.id), now = this.now ?? 0, since = now - WINDOW_S;
    const header = Math.min(15, h * .4), top = y + header, plotH = Math.max(4, h - header);
    // Header: the lane's name, then each trace's newest value in its colour.
    ctx.font = '11px system-ui, sans-serif'; ctx.textBaseline = 'middle';
    ctx.fillStyle = '#91a2a8'; ctx.fillText(lane.title, x, y + header / 2);
    let at = x + ctx.measureText(lane.title).width + 8;
    ctx.font = '11px ui-monospace, monospace';
    lane.traces.forEach(([name, color], i) => {
      const value = rings[i].last(), text = `${name} ${Number.isFinite(value) ? value.toFixed(lane.digits) + lane.unit : '—'}`;
      if (at + ctx.measureText(text).width > x + w) return;
      ctx.fillStyle = color; ctx.fillText(text, at, y + header / 2);
      at += ctx.measureText(text).width + 8;
    });
    ctx.fillStyle = '#0b1318'; ctx.fillRect(x, top, w, plotH);
    // The value range: fixed, or the window's own with a minimum span.
    let lo = Infinity, hi = -Infinity, any = false;
    rings.forEach(ring => ring.forEach(since, (t, v) => { if (Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); any = true; } }));
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
    ctx.fillStyle = '#5d6f76'; ctx.font = '10px ui-monospace, monospace';
    if (plotH > 24) { ctx.fillText(hi.toFixed(lane.digits), x + 3, top + 7); ctx.fillText(lo.toFixed(lane.digits), x + 3, top + plotH - 6); }
    if (!any && lane.waiting) { ctx.fillText(lane.waiting, x + w / 2 - ctx.measureText(lane.waiting).width / 2, top + plotH / 2); return; }
    // Traces, decimated to the pixel columns; a NaN leaves a gap.
    lane.traces.forEach(([, color], i) => {
      ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.beginPath();
      let column = null, first = 0, low = 0, high = 0, last = 0, pen = false;
      const flush = () => {
        if (column == null) return;
        const cx = x + column;
        if (!pen) { ctx.moveTo(cx, yOf(first)); pen = true; } else ctx.lineTo(cx, yOf(first));
        if (low !== high) { ctx.lineTo(cx, yOf(low)); ctx.lineTo(cx, yOf(high)); }
        ctx.lineTo(cx, yOf(last));
      };
      rings[i].forEach(since, (t, v) => {
        if (!Number.isFinite(v)) { flush(); column = null; pen = false; return; }
        const c = Math.round((t - since) / WINDOW_S * (w - 1));
        if (c !== column) { flush(); column = c; first = low = high = last = v; }
        else { low = Math.min(low, v); high = Math.max(high, v); last = v; }
      });
      flush();
      ctx.stroke();
    });
  }
}
