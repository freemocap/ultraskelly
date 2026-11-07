// Handles between the dashboard's panels (docs/dashboard.md, "Sizing"). Each handle is the 8 px between two panels:
// drag it to resize them, double-click it for the usual size. The browser keeps the sizes (localStorage), so the Pi's
// window opens as it was left, across reloads and reboots. A size reaches the page as a CSS property on <body>, which
// dashboard.css and panel.css read with the usual size as the fallback; a dragged size carries its limits as a CSS
// clamp(), so a smaller window narrows it rather than overflow. The phone layout has no handles.
const KEY = 'ultraskelly.sizes';  // localStorage: {control, sensors, strip, oak, tests, nodes}, each in px

// The sizes a handle sets: its CSS property, the element whose size it is, which way a drag grows it (x, y) and its
// limits. The 3D scene keeps 480 px of width and 220 px of height (8 px padding and 8 px handles around it); the
// control column keeps 260 px, where the motor tests in it keep whole labels.
export const SIZES = {
  control: {property: '--control-size', element: '#control', side: 'width', grow: [1, 0], min: 260, max: '35vw'},
  sensors: {property: '--sensor-size', element: '.sensors', side: 'width', grow: [-1, 0], min: 260, max: '100vw - var(--control-w, 0px) - 512px'},
  strip: {property: '--strip-size', element: '.strip', side: 'height', grow: [0, -1], min: 90, max: '100vh - var(--header-h) - 244px'},
  // The tracking camera's video height: dashboard.css keeps it 4:3 within the column and leaves the ZED's tiles room.
  oak: {property: '--oak-size', element: '.oak .tile', side: 'height', grow: [0, 1], min: 1, max: null},
  // The motor tests' and the nodes' cards beside the time series in the strip, which takes the rest: each at most half
  // of what leaves it 200 px. The motor tests' buttons keep whole labels down to 236 px. With the control panel both
  // cards are in its column and these handles are not shown.
  tests: {property: '--tests-size', element: '.strip > .card.tests', side: 'width', grow: [-1, 0], min: 236, max: '(100% - 216px) / 2'},
  nodes: {property: '--nodes-size', element: '.strip > .card.nodes', side: 'width', grow: [-1, 0], min: 160, max: '(100% - 216px) / 2'},
};

const positive = value => typeof value === 'number' && Number.isFinite(value) && value > 0;

// A size as the page sets it: a dragged px value within its limits, which the browser applies again as the window
// changes size.
export function expression(name, px) {
  const {min, max} = SIZES[name], size = `${Math.round(px)}px`;
  return max ? `clamp(${min}px, ${size}, ${max})` : size;
}

// The stored sizes this page knows; anything else is the usual size.
export function parseSizes(text) {
  let data = null;
  try { data = JSON.parse(text); } catch { /* None kept */ }
  if (!data || typeof data !== 'object') return {};
  return Object.fromEntries(Object.keys(SIZES).filter(name => positive(data[name])).map(name => [name, data[name]]));
}

// A panel's size after a drag by (dx, dy) px from `start` px.
export function dragged(name, start, dx, dy) {
  const [x, y] = SIZES[name].grow;
  return start + x * dx + y * dy;
}

// The sizes after a double-click on a handle: what it sets, back to the usual.
export function reset(sizes, grip) {
  const next = {...sizes};
  delete next[grip];
  return next;
}

// localStorage, which a locked-down browser may refuse; the panels then open at their usual sizes.
function stored() { try { return parseSizes(localStorage.getItem(KEY)); } catch { return {}; } }
function store(sizes) {
  try {
    if (Object.keys(sizes).length) localStorage.setItem(KEY, JSON.stringify(sizes));
    else localStorage.removeItem(KEY);
  } catch { /* Not kept */ }
}

// Set the sizes on the page; one not given is the usual, from the stylesheets.
function apply(sizes) {
  const style = document.body.style;
  for (const [name, {property}] of Object.entries(SIZES)) {
    if (sizes[name]) style.setProperty(property, expression(name, sizes[name]));
    else style.removeProperty(property);
  }
}

// Make each handle on the page work, with the sizes kept from before.
export function resizable(root = document) {
  let sizes = stored();
  apply(sizes);
  const size = name => root.querySelector(SIZES[name].element)?.getBoundingClientRect()[SIZES[name].side] ?? 0;
  // The views beside the panels take up their new sizes; the page tells the bot what it now draws (dashboard.js).
  const changed = () => dispatchEvent(new Event('resize'));
  for (const grip of root.querySelectorAll('[data-grip]')) {
    const name = grip.dataset.grip;
    if (!(name in SIZES)) continue;
    grip.title = 'Drag to resize; double-click for the usual size';
    grip.addEventListener('pointerdown', event => {
      if (event.button !== 0) return;
      event.preventDefault();
      const x0 = event.clientX, y0 = event.clientY, start = size(name);
      let last = event, moved = false, frame = 0;
      const show = () => {
        frame = 0;
        const dx = last.clientX - x0, dy = last.clientY - y0;
        moved ||= Math.abs(dx) + Math.abs(dy) > 2;  // A click or a double-click changes nothing
        if (moved) apply({...sizes, [name]: Math.max(1, dragged(name, start, dx, dy))});
      };
      grip.setPointerCapture(event.pointerId);
      grip.classList.add('dragging');
      document.body.classList.add('resizing', grip.classList.contains('row') ? 'resizing-row' : 'resizing-col');
      const move = moving => { last = moving; frame ||= requestAnimationFrame(show); };
      const end = () => {
        cancelAnimationFrame(frame);
        show();
        grip.removeEventListener('pointermove', move);
        grip.removeEventListener('lostpointercapture', end);
        grip.classList.remove('dragging');
        document.body.classList.remove('resizing', 'resizing-row', 'resizing-col');
        if (!moved) return;
        // Kept as the panel came out: a drag past a limit keeps the limit, not where the pointer went.
        sizes = {...sizes, [name]: Math.round(size(name))};
        apply(sizes);
        store(sizes);
        changed();
      };
      grip.addEventListener('pointermove', move);
      grip.addEventListener('lostpointercapture', end);  // After pointerup or pointercancel
    });
    grip.addEventListener('dblclick', () => {
      sizes = reset(sizes, name);
      apply(sizes);
      store(sizes);
      changed();
    });
  }
}
