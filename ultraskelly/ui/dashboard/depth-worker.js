// Depth frames off the main thread (docs/depth-stream.md). For each depth_frame: inflate the zlib
// depth, read the little-endian millimetres, colour a heat map (auto range, red near, blue far) and a
// raw view (fixed range, near bright), and decode the rectified colour JPEG. Everything goes back to
// the dashboard as transferred bitmaps and one Uint16Array; nothing is shared with the stream decoder.
import { TURBO } from '/colormap.js';

const RAW_NEAR = 360, RAW_FAR = 8000;  // Millimetres: the depth's whole range, for the raw view

let range = null;  // The heat map's range, eased between frames so it does not flicker

// The 2nd and 98th percentiles of the valid depths, from a 50 mm histogram.
function percentiles(mm) {
  const bins = new Uint32Array(Math.ceil(RAW_FAR / 50) + 1);
  let valid = 0;
  for (let i = 0; i < mm.length; i++) if (mm[i]) { bins[Math.min(bins.length - 1, mm[i] / 50 | 0)]++; valid++; }
  if (!valid) return null;
  const at = share => { let seen = 0; for (let b = 0; b < bins.length; b++) { seen += bins[b]; if (seen >= share * valid) return b * 50 + 25; } return RAW_FAR; };
  return {near: at(0.02), far: Math.max(at(0.98), at(0.02) + 200), valid};
}

async function inflate(bytes) {
  const stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

self.onmessage = async ({data: job}) => {
  try {
    const {width, height} = job;
    const bytes = job.compression === 'zlib' ? await inflate(job.bytes) : job.bytes;
    if (bytes.byteLength < width * height * 2) throw new Error(`depth has ${bytes.byteLength} bytes for ${width}x${height}`);
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const mm = new Uint16Array(width * height);
    for (let i = 0; i < mm.length; i++) mm[i] = view.getUint16(2 * i, true);
    const found = percentiles(mm);
    if (found) {
      range = range ? {near: range.near + .3 * (found.near - range.near), far: range.far + .3 * (found.far - range.far)} : found;
    }
    const heat = new ImageData(width, height), raw = new ImageData(width, height);
    const near = range?.near ?? RAW_NEAR, span = Math.max(1, (range?.far ?? RAW_FAR) - near);
    for (let i = 0, o = 0; i < mm.length; i++, o += 4) {
      const d = mm[i];
      heat.data[o + 3] = raw.data[o + 3] = 255;
      if (!d) continue;  // Unknown stays black in both
      const c = 255 - Math.max(0, Math.min(255, Math.round((d - near) / span * 255)));
      heat.data[o] = TURBO[c * 3]; heat.data[o + 1] = TURBO[c * 3 + 1]; heat.data[o + 2] = TURBO[c * 3 + 2];
      const g = 255 - Math.max(0, Math.min(255, Math.round((d - RAW_NEAR) / (RAW_FAR - RAW_NEAR) * 255)));
      raw.data[o] = raw.data[o + 1] = raw.data[o + 2] = Math.max(g, 24);  // The farthest depth stays visible over unknown
    }
    const [heatBitmap, rawBitmap, color] = await Promise.all([
      createImageBitmap(heat), createImageBitmap(raw),
      job.color ? createImageBitmap(new Blob([job.color], {type: 'image/jpeg'})) : null,
    ]);
    const transfer = [mm.buffer, heatBitmap, rawBitmap, ...(color ? [color] : [])];
    self.postMessage({id: job.id, mm, width, height, near, far: near + span, valid: found?.valid ?? 0,
                      heat: heatBitmap, raw: rawBitmap, color}, transfer);
  } catch (problem) {
    self.postMessage({id: job.id, error: String(problem?.message ?? problem)});
  }
};
