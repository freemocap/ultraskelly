// The Tune window, opened from the control column (panel.js): every setting of the bot's controllers, drawn from each
// group's schema (GET /api/tuning; ultraskelly/core/bot/motor/tuning.py), so a controller's new settings appear by
// themselves. A number is a slider with its box, within the schema's bounds; a switch is a switch; a choice of a few
// values (a direction's +1 or -1) is a pair of buttons; a range is two boxes. A change goes to the bot as it is let go
// (a slider released, a number entered, a switch flipped) and applies live. A setting that differs from its default is
// marked, with ↺ to put it back; one not yet saved has a dot. Save keeps what differs from the defaults for the bot's
// next start; Reset puts every setting back, live. Each setting's description, default and unit are its tooltip.
// No imports: tests/js/dashboard_calls.mjs runs the parts that need no page under node.

const SLIDER_STEPS = 500;  // A slider's span in steps, at least: its step is the power of ten at or under span / 500

const decade = x => 10 ** Math.floor(Math.log10(x));
// The round number (1, 2, 2.5 or 5 times a power of ten) at or above x.
function roundUp(x) {
  const unit = decade(x);
  return [1, 2, 2.5, 5, 10].map(m => m * unit).find(value => value >= x * (1 - 1e-9));
}
const plural = (n, word) => `${n} ${word}${n === 1 ? '' : 's'}`;
const two = n => String(n).padStart(2, '0');
const localDay = (now = new Date()) => `${now.getFullYear()}-${two(now.getMonth() + 1)}-${two(now.getDate())}`;  // As saved_at

// A setting's label: its name in words ("max_speed", or pydantic's title made from it, "Max Speed": "Max speed"); a
// title set by hand stays as it is.
export function labelOf(name, title) {
  const words = name.replace(/_/g, ' ');
  if (!title || title.toLowerCase() === words.toLowerCase()) return words.charAt(0).toUpperCase() + words.slice(1);
  return title;
}

// A number's slider: the schema's bounds, a step inside an exclusive one. An open side runs to 0, or from 0 to a round
// number three times the default (or half again the value, if that is more); servo degrees to the servo's own 0-180.
export function sliderOf(prop, value) {
  const integer = prop.type === 'integer';
  const typical = Math.abs(typeof prop.default === 'number' ? prop.default : typeof value === 'number' ? value : 1) || 1;
  const servo = prop.unit === 'servo°';
  let min = prop.minimum ?? prop.exclusiveMinimum ?? (typeof prop.default === 'number' && prop.default < 0 ? -roundUp(typical * 3) : 0);
  let max = prop.maximum ?? prop.exclusiveMaximum ?? (servo ? 180 : roundUp(Math.max(typical * 3, Math.abs(value ?? 0) * 1.5)));
  const step = integer ? 1 : decade((max - min) / SLIDER_STEPS);
  if (prop.minimum == null && prop.exclusiveMinimum != null) min += step;
  if (prop.maximum == null && prop.exclusiveMaximum != null) max -= step;
  return {min, max, step};
}

// How a setting is drawn: a switch, a choice of its few values, a slider with a number, numbers side by side, or, for a
// kind the window does not know, its value as text.
export function kindOf(prop, value) {
  if (prop.type === 'boolean') return {kind: 'switch'};
  if (Array.isArray(prop.enum) && prop.enum.length <= 4) return {kind: 'choice', options: prop.enum};
  if (prop.type === 'number' || prop.type === 'integer') return {kind: 'number', integer: prop.type === 'integer', ...sliderOf(prop, value)};
  const items = prop.prefixItems;
  if (prop.type === 'array' && Array.isArray(items) && items.length && items.every(item => item.type === 'number' || item.type === 'integer'))
    return {kind: 'numbers', integer: items.map(item => item.type === 'integer')};
  return {kind: 'fixed'};
}

// A group's settings in its schema's order, each with how it is drawn and whether it differs from its default or from
// what is saved; nested settings follow a heading, their key dotted ("inner.gain").
export function settings(group) {
  const changed = new Set(group.changed ?? []), unsaved = new Set(group.unsaved ?? []), entries = [];
  const walk = (schema, values, path) => {
    for (const [name, prop] of Object.entries(schema.properties ?? {})) {
      const at = [...path, name], key = at.join('.'), value = values?.[name];
      if (prop.type === 'object' && prop.properties) {
        entries.push({kind: 'heading', key, path: at, label: labelOf(name, prop.title), description: prop.description ?? ''});
        walk(prop, value, at);
        continue;
      }
      entries.push({...kindOf(prop, value), key, path: at, label: labelOf(name, prop.title), description: prop.description ?? '',
        unit: prop.unit ?? '', value, fallback: prop.default, changed: changed.has(key), unsaved: unsaved.has(key),
        start: prop.tuning === 'start'});
    }
  };
  walk(group.schema, group.values, []);
  return entries;
}

// A value as the window shows it.
export function shown(value) {
  if (typeof value === 'boolean') return value ? 'On' : 'Off';
  if (Array.isArray(value)) return value.map(shown).join(' – ');
  if (typeof value === 'number' && value < 0) return `−${-value}`;
  return String(value);
}

// A setting's tooltip: what it does, its default, and when it applies if not at once.
export function tipOf(entry) {
  return [entry.description, `Default ${shown(entry.fallback)}${entry.unit ? ` ${entry.unit}` : ''}`,
          entry.start ? 'Applies when the bot next starts' : ''].filter(Boolean).join('\n');
}

// The window's status line, from the bot's state and the last refusal, and whether Save and Reset can be pressed.
export function statusOf(state, note, today = localDay()) {
  const unsaved = state.groups.reduce((sum, group) => sum + group.unsaved.length, 0);
  const parts = [];
  if (note) parts.push(note);
  if (state.problems.length) parts.push(`${plural(state.problems.length, 'saved setting')} not applied`);
  if (!state.file) parts.push('Live only: nowhere to save');
  else if (unsaved) parts.push(`${plural(unsaved, 'change')} not saved`);
  else if (state.saved_at) parts.push(`Saved ${state.saved_at.startsWith(today) ? state.saved_at.slice(11, 16) : state.saved_at.replace('T', ' ').slice(0, 16)}`);
  return {
    text: parts.join(' · '), kind: note || state.problems.length ? 'refused' : unsaved ? 'unsaved' : '',
    title: [...state.problems, state.file ? `Saved in ${state.file}` : ''].filter(Boolean).join('\n'),
    save: Boolean(state.file) && unsaved > 0, reset: state.groups.some(group => group.changed.length > 0),
  };
}

// {"inner": {"gain": value}} for the path ["inner", "gain"].
const nest = (path, value) => path.reduceRight((inner, name) => ({[name]: inner}), value);

function element(tag, className = '', text = '') {
  const made = document.createElement(tag);
  if (className) made.className = className;
  if (text) made.textContent = text;
  return made;
}

function button(className, text, title = '') {
  const made = element('button', className, text);
  made.type = 'button';
  if (title) made.title = title;
  return made;
}

export class Tune {
  constructor(dialog) {
    this.dialog = dialog;
    this.state = null;
    this.note = null;  // The last refusal, until a change goes through
    this.shape = '';  // The schemas the window's rows were made for
    this.rows = new Map();  // "group.setting" -> its row's update(entry)
    this.resets = new Map();  // group -> its section's Reset button
    this.queue = Promise.resolve();  // Changes go one at a time, in order
    this.groups = dialog.querySelector('.tune-groups');
    this.status = dialog.querySelector('.tune-status');
    this.saveButton = dialog.querySelector('.tune-save');
    this.resetButton = dialog.querySelector('.tune-reset');
    this.saveButton.onclick = () => this.post('/api/tuning/save', {});
    this.resetButton.onclick = () => this.post('/api/tuning/reset', {});
    dialog.querySelector('.tune-close').onclick = () => dialog.close();
    // A click on the dimmed page closes it; not a slider drag or a selection that ends there, which the browser also
    // reports as a click on the window.
    let pressedOutside = false;
    dialog.addEventListener('pointerdown', event => { pressedOutside = event.target === dialog; });
    dialog.addEventListener('click', event => { if (event.target === dialog && pressedOutside) dialog.close(); });
  }

  async open() {
    if (!this.dialog.open) this.dialog.showModal();
    this.note = null;
    await this.load();
  }

  async load() {
    try {
      const reply = await fetch('/api/tuning', {cache: 'no-store'});
      const answer = await reply.json().catch(() => ({}));
      if (!answer.groups) throw new Error(answer.error ?? (reply.status === 404 ? 'This bot has no tuning' : `HTTP ${reply.status}`));
      this.note = null;
      this.state = answer;
    } catch (problem) {
      this.note = problem.message ?? String(problem);
    }
    this.render();
  }

  post(path, body) {
    this.queue = this.queue.then(() => this.send(path, body));
    return this.queue;
  }

  async send(path, body) {
    try {
      const reply = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
      const answer = await reply.json().catch(() => ({}));
      this.note = reply.ok ? null : answer.error ?? `Refused (HTTP ${reply.status})`;
      if (answer.groups) this.state = answer;  // A refusal carries the settings as they still are: the controls go back
    } catch (problem) {
      this.note = `No answer: ${problem.message ?? problem}`;
    }
    this.render();
  }

  render() {
    const state = this.state, status = state ? statusOf(state, this.note) : {text: this.note ?? 'Asking the bot…',
      kind: this.note ? 'refused' : '', title: '', save: false, reset: false};
    this.status.textContent = status.text;
    this.status.className = `tune-status ${status.kind}`;
    this.status.title = status.title;
    this.saveButton.disabled = !status.save;
    this.resetButton.disabled = !status.reset;
    this.saveButton.title = state?.file ? `Keep the changed settings for the bot's next start, in ${state.file}` : 'This bot keeps no tuning';
    if (!state) return;
    const shape = JSON.stringify(state.groups.map(group => [group.key, group.title, group.schema]));
    if (shape !== this.shape) this.build(state, shape);
    for (const group of state.groups) {
      this.resets.get(group.key).disabled = !group.changed.length;
      for (const entry of settings(group)) this.rows.get(`${group.key}.${entry.key}`)?.(entry);
    }
  }

  build(state, shape) {
    this.shape = shape;
    this.rows.clear();
    this.resets.clear();
    this.groups.replaceChildren(...state.groups.map(group => {
      const section = element('section', 'tune-group'), head = element('header'), title = element('h3', '', group.title);
      title.title = group.schema.description ?? '';
      const reset = button('tune-group-reset', 'Reset', `Every ${group.title.toLowerCase()} setting back to its default`);
      reset.onclick = () => this.post('/api/tuning/reset', {group: group.key});
      this.resets.set(group.key, reset);
      head.append(title, reset);
      section.append(head, ...settings(group).map(entry => this.row(group, entry)));
      return section;
    }));
  }

  row(group, entry) {
    if (entry.kind === 'heading') {
      const heading = element('h4', 'tune-heading', entry.label);
      heading.title = entry.description;
      return heading;
    }
    const id = `tune-${group.key}-${entry.path.join('-')}`, row = element('div', `tune-row ${entry.kind}`);
    const label = element('label', '', entry.label), unit = element('span', 'tune-unit', entry.unit);
    const undo = button('tune-undo', '↺');
    undo.onclick = () => this.post('/api/tuning/reset', {group: group.key, setting: entry.key});
    const send = value => this.post('/api/tuning', {[group.key]: nest(entry.path, value)});
    let current = entry, controls, show;
    const typed = (box, integer) => {  // A box's number, or null when it holds none
      const value = Number(box.value);
      if (box.value.trim() === '' || !Number.isFinite(value)) return null;
      return integer ? Math.round(value) : value;
    };
    // A box shows the bot's value unless something is being typed in it and not yet entered: so a refused or rounded
    // value goes back even while the box keeps the focus after Enter.
    const box = () => {
      const made = element('input');
      made.type = 'number';
      made.step = 'any';
      made.oninput = () => { made.dataset.typing = '1'; };
      made.addEventListener('change', () => { delete made.dataset.typing; }, {capture: true});
      return made;
    };
    const fill = (input, value) => { if (!input.dataset.typing) input.value = String(value ?? ''); };
    if (entry.kind === 'number') {
      const slider = element('input'), number = box();
      slider.type = 'range';
      slider.setAttribute('aria-label', entry.label);
      number.id = label.htmlFor = id;
      slider.oninput = () => { number.value = slider.value; };
      slider.onchange = () => send(entry.integer ? Math.round(Number(slider.value)) : Number(slider.value));
      number.onchange = () => {
        const value = typed(number, entry.integer);
        if (value === null) fill(number, current.value);
        else send(value);
      };
      controls = [slider, number];
      show = next => {
        slider.min = next.min; slider.max = next.max; slider.step = next.step; slider.value = next.value;
        fill(number, next.value);
      };
    } else if (entry.kind === 'switch') {
      const toggle = button('tune-switch'), word = element('b', 'tune-switch-word');
      toggle.setAttribute('role', 'switch');
      toggle.id = label.htmlFor = id;
      toggle.append(element('span', 'tune-switch-track'), word);
      toggle.onclick = () => send(!current.value);
      controls = [toggle];
      show = next => { toggle.setAttribute('aria-checked', String(Boolean(next.value))); word.textContent = shown(Boolean(next.value)); };
    } else if (entry.kind === 'choice') {
      const choice = element('div', 'tune-choice'), options = entry.options.map(option => {
        const pick = button('', typeof option === 'number' && option > 0 ? `+${option}` : shown(option));
        pick.onclick = () => send(option);
        return pick;
      });
      choice.setAttribute('role', 'group');
      choice.setAttribute('aria-label', entry.label);
      choice.append(...options);
      controls = [choice];
      show = next => options.forEach((pick, index) => pick.setAttribute('aria-pressed', String(entry.options[index] === next.value)));
    } else if (entry.kind === 'numbers') {
      const boxes = element('div', 'tune-numbers'), inputs = entry.integer.map((integer, index) => {
        const input = box();
        input.setAttribute('aria-label', `${entry.label} ${index + 1}`);
        input.onchange = () => {
          const values = inputs.map((each, at) => typed(each, entry.integer[at]));
          if (values.includes(null)) show(current);
          else send(values);
        };
        return input;
      });
      inputs[0].id = label.htmlFor = id;
      boxes.append(...inputs);
      controls = [boxes];
      show = next => inputs.forEach((input, index) => fill(input, next.value?.[index]));
    } else {
      const fixed = element('code', 'tune-fixed');
      controls = [fixed];
      show = next => { fixed.textContent = JSON.stringify(next.value); };
    }
    row.append(label, ...controls, unit, undo);
    const update = next => {
      current = next;
      row.classList.toggle('changed', next.changed);
      row.classList.toggle('unsaved', next.unsaved);
      row.title = tipOf(next);
      undo.title = `Back to the default, ${shown(next.fallback)}`;
      undo.disabled = !next.changed;
      show(next);
    };
    this.rows.set(`${group.key}.${entry.key}`, update);
    update(entry);
    return row;
  }
}
