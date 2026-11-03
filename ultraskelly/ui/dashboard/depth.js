// Depth views (docs/depth-stream.md). Each depth_frame is copied out of the stream decoder, whose
// storage a later message may reuse, and decoded in depth-worker.js. Its results draw the heat-map
// tile and its metre scale, answer "how far is that?" under the cursor, and feed the 3D point cloud
// and mesh. Newest wins: while the worker is busy, only the newest frame waits.
import { turboGradient } from '/colormap.js';

const metres = mm => `${(mm / 1000).toFixed(2)} m`;

export class DepthViews {
  // tiles: CameraTiles; layer: the 3D depth layer or null; legend: {near, ramp, far, info} elements.
  constructor({tiles, layer, legend}) {
    this.tiles = tiles; this.layer = layer; this.legend = legend;
    this.worker = new Worker('/depth-worker.js', {type: 'module'});
    this.worker.onmessage = ({data}) => this.done(data);
    this.worker.onerror = event => console.error('Depth worker failed:', event.message);
    this.inFlight = null; this.waiting = null; this.sent = 0; this.shown = 0;
    this.latest = null;  // The newest depth drawn: millimetres, grid size, step, intrinsics and pose
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
    this.tiles.showBitmap('depth_heat', result.heat);
    result.heat.close();
    this.latest = {mm: result.mm, width: result.width, height: result.height, step: meta.step,
                   camera: meta.camera, cameraToHead: meta.cameraToHead};
    const {near, ramp, far, info} = this.legend;
    near.textContent = metres(result.near); far.textContent = metres(result.far);
    ramp.title = `Heat map: ${metres(result.near)} (red) to ${metres(result.far)} (blue), from this frame's 2nd to 98th percentile`;
    info.textContent = `${Math.round(100 * result.valid / result.mm.length)}% known · ${meta.method}`;
    if (this.layer) this.layer.update(result, meta);  // The layer keeps and later closes the colour image
    else result.color?.close();
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
      caption.textContent = !inside ? label : `${label} · ${mm ? metres(mm) : 'unknown'}`;
    });
    tile.figure.addEventListener('mouseleave', () => { caption.textContent = label; });
  }

  // Once a second: depth frames drawn since the last call.
  tick() {
    const shown = this.shown;
    this.shown = 0;
    return shown;
  }
}
