// Depth views (docs/depth-stream.md). Each depth_frame is copied out of the stream decoder, whose
// storage a later message may reuse, and decoded in depth-worker.js. Its results draw the heat-map
// tile and its metre scale, answer "how far is that?" under the cursor, and feed the 3D point cloud
// and mesh. Newest wins: while the worker is busy, only the newest frame waits. The heat map and its
// legend draw in the page's frame (frameclock.js).
import { turboGradient } from '/colormap.js';
import { HeatMap } from '/heatmap.js';
import { setProperty, setText } from '/dom.js';

const metres = mm => `${(mm / 1000).toFixed(2)} m`;

export class DepthViews {
  // tiles: CameraTiles; layer: the 3D depth layer or null; legend: {near, ramp, far, info} elements;
  // clock: the page's FrameClock.
  constructor({tiles, layer, legend, clock}) {
    this.tiles = tiles; this.layer = layer; this.legend = legend; this.clock = clock;
    this.pending = null;  // The newest result whose heat map and legend wait for the page's frame
    clock.add(() => this.present());
    this.worker = new Worker('/depth-worker.js', {type: 'module'});
    this.worker.onmessage = ({data}) => this.done(data);
    this.worker.onerror = event => console.error('Depth worker failed:', event.message);
    this.inFlight = null; this.waiting = null; this.sent = 0; this.shown = 0; this.timing = null;
    this.latest = null;  // The newest depth drawn: millimetres, grid size, step, intrinsics and pose
    try { this.heat = new HeatMap(tiles.tiles.get('depth_heat').canvas); }
    catch (problem) { this.heat = null; console.error('No heat map:', problem); }
    legend.ramp.style.background = turboGradient();
    this.readout(tiles.tiles.get('depth_heat'));
  }

  // One depth_frame message from the stream.
  handle(frame) {
    const {depth, color} = frame;
    const job = {
      id: ++this.sent, width: depth.width, height: depth.height, compression: depth.compression,
      bytes: depth.data.slice(),  // Copies, so the worker owns what it is given
      color: color?.encoding === 'jpeg' ? color.data.slice() : null,
      meta: {camera: frame.camera, step: depth.step, method: frame.method, cameraToHead: frame.camera_to_head},
    };
    if (this.inFlight) { this.waiting = job; return; }
    this.send(job);
  }

  send(job) {
    const {meta, ...message} = job;
    this.inFlight = meta;
    this.worker.postMessage(message, [message.bytes.buffer, ...(message.color ? [message.color.buffer] : [])]);
  }

  done(result) {
    const meta = this.inFlight;
    this.inFlight = null;
    if (this.waiting) { const next = this.waiting; this.waiting = null; this.send(next); }
    if (result.error) { console.error('Could not show a depth frame:', result.error); return; }
    this.shown += 1;
    this.latest = {mm: result.mm, width: result.width, height: result.height, step: meta.step,
                   camera: meta.camera, cameraToHead: meta.cameraToHead};
    this.timing = result.timing;
    this.pending = {result, method: meta.method};
    this.clock.changed();
    if (this.layer) this.layer.update(result, meta);  // The layer keeps and later closes the colour image
    else result.color?.close();
  }

  // The heat map's and the legend's part of the page's frame.
  present() {
    if (!this.pending) return false;
    const {result, method} = this.pending;
    this.pending = null;
    if (this.heat && this.tiles.shows('depth_heat')) {  // Like the camera tiles, a heat map out of view is not drawn
      this.heat.draw(result.mm, result.width, result.height, result.near, result.far);
      this.tiles.markDrawn('depth_heat');  // Also copies it into the enlarged view when that is open
    }
    const {near, ramp, far, info} = this.legend;
    setText(near, metres(result.near)); setText(far, metres(result.far));
    setProperty(ramp, 'title', `Heat map: ${metres(result.near)} (red) to ${metres(result.far)} (blue), from this frame's 2nd to 98th percentile`);
    setText(info, `${Math.round(100 * result.valid / result.mm.length)}% known · ${method}`);
    return false;
  }

  // The depth under the cursor, in the tile's caption. Tiles letterbox, so map through object-fit.
  readout(tile) {
    const caption = tile.figure.querySelector('figcaption'), label = caption.textContent;
    tile.figure.addEventListener('mousemove', event => {
      const depth = this.latest;
      if (!depth) return;
      const box = tile.canvas.getBoundingClientRect(), scale = Math.min(box.width / depth.width, box.height / depth.height);
      const u = Math.floor((event.clientX - box.left - (box.width - depth.width * scale) / 2) / scale);
      const v = Math.floor((event.clientY - box.top - (box.height - depth.height * scale) / 2) / scale);
      const inside = u >= 0 && v >= 0 && u < depth.width && v < depth.height, mm = inside ? depth.mm[v * depth.width + u] : 0;
      caption.textContent = !inside ? label : [label, mm ? metres(mm) : 'unknown'].filter(Boolean).join(' · ');
    });
    tile.figure.addEventListener('mouseleave', () => { caption.textContent = label; });
  }

  // Once a second: depth frames drawn since the last call. The legend's tooltip gives the worker's time.
  tick() {
    const shown = this.shown, t = this.timing;
    this.shown = 0;
    if (t) setProperty(this.legend.info, 'title', `Worker ${t.total.toFixed(1)} ms a frame: inflate ${t.inflate.toFixed(1)}, ` +
                                                  `range ${t.percentiles.toFixed(1)}, colour image ${t.colorJpeg.toFixed(1)}`);
    return shown;
  }
}
