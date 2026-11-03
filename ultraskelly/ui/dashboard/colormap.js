// Turbo colormap (a polynomial approximation of Google's Turbo), shared by the depth worker's heat map
// and the legend under it. Index 255 is red, 0 is blue; the dashboard draws near depths red.
export const TURBO = (() => {
  const lut = new Uint8ClampedArray(256 * 3);
  for (let i = 0; i < 256; i++) {
    const t = i / 255;
    const r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
    const g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
    const b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
    lut.set([r * 255, g * 255, b * 255], i * 3);
  }
  return lut;
})();

// A CSS gradient running from near (red) on the left to far (blue) on the right.
export function turboGradient(stops = 9) {
  const colors = [];
  for (let s = 0; s < stops; s++) {
    const i = Math.round(255 - s / (stops - 1) * 255);
    colors.push(`rgb(${TURBO[i * 3]},${TURBO[i * 3 + 1]},${TURBO[i * 3 + 2]})`);
  }
  return `linear-gradient(to right, ${colors.join(', ')})`;
}
