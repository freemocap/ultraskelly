// The page's drawing frame, at most once per tracking-camera frame (docs/dashboard.md). A decoded tracking frame
// asks for a frame at once; every other change (another camera, the depth, robot_state, the 3D view) waits for
// the next tracking frame, so sources that arrive out of step share one frame where they would each have made
// their own. Without tracking frames, changes draw at most once per the tracking camera's last frame period.
// Nothing runs while nothing changes.
const EARLY_MS = 4;  // A frame may come this much before its time, to land on the nearest display frame
const WAKE_MS = 12;  // A wait's timer ends this long before then; the display frame after it draws

export class FrameClock {
  constructor() {
    this.parts = [];            // part(now) draws what changed; true asks for the next frame as well
    this.frameMs = 1000 / 30;   // The tracking camera's frame period: the OAK's 30 a second until it reports
    this.trackedAt = -Infinity; this.drawnAt = -Infinity;
    this.wanted = false; this.tracking = false; this.request = 0; this.timer = 0;
  }

  add(part) { this.parts.push(part); }

  setFrameRate(fps) { this.frameMs = 1000 / Math.min(60, Math.max(10, fps || 30)); }

  // A tracking-camera frame is ready to draw.
  tracked() {
    this.trackedAt = performance.now(); this.wanted = this.tracking = true;
    clearTimeout(this.timer); this.timer = 0;
    this.request ||= requestAnimationFrame(now => this.frame(now));
  }

  // Anything else changed.
  changed() {
    this.wanted = true;
    if (this.request || this.timer) return;
    const now = performance.now(), flowing = now - this.trackedAt < 2 * this.frameMs;
    // While tracking frames flow, the next one draws this; the timer only covers their stopping.
    const wait = (flowing ? this.trackedAt + 2 * this.frameMs : this.drawnAt + this.frameMs - WAKE_MS) - now;
    if (wait > 1) this.timer = setTimeout(() => { this.timer = 0; this.request ||= requestAnimationFrame(t => this.frame(t)); }, wait);
    else this.request = requestAnimationFrame(t => this.frame(t));
  }

  frame(now) {
    this.request = 0;
    if (!this.wanted) return;
    if (!this.tracking && now - this.drawnAt < this.frameMs - EARLY_MS) {  // Too soon after the last frame
      this.request = requestAnimationFrame(t => this.frame(t));
      return;
    }
    this.wanted = this.tracking = false; this.drawnAt = now;
    let again = false;
    for (const part of this.parts) if (part(now)) again = true;
    if (again) this.changed();
  }
}
