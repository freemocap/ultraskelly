// Motor test panel: moves one actuator at a time through /api/motor-test. The bot applies the
// test limits and pauses tracking during each test; every test is answered once it has
// finished, been stopped or been refused, and the answer is shown here. From the bot's hardware
// checks (the `health` message, as health.js reads it) it shows whether each board answers and,
// after a servo test, the command next to what the servo board reads back: angles are commands,
// not measured positions, and are labelled "not sent" while their board is down.
import { setProperty, setText } from '/dom.js';
import { readingsStale } from '/health.js';

const CENTER = 90;      // Degrees; the head's neutral angle
const SERVO_STEP = 10;  // Degrees per nudge

const element = (tag, properties = {}, ...children) => {
  const made = Object.assign(document.createElement(tag), properties);
  made.append(...children);
  return made;
};
const signed = value => `${value >= 0 ? '+' : '−'}${Math.abs(value).toFixed(2)}`;
const BOARDS = [['servo_board', 'Servo board 0x40'], ['motor_board', 'Motor HAT 0x60']];
// A read-back pulse this close to the one the commanded angle needs is the command: about 2.4 degrees. The bot's own
// tolerance (diagnostics.py, PULSE_TOLERANCE_US), so this panel and the servo board's status never disagree.
const SAME_US = 20;
export const down = (model, id) => { const part = model?.components?.find(component => component.id === id); return part && (part.status === 'fail' || part.status === 'warn') ? part : null; };

// What the servo board says about the last servo test's channel, from the bot's checks (health.js's model): its
// diagnostics.pca9685, pulse widths in microseconds. [words, hint, kind], or null when there is nothing to say.
// A board that does not answer has nothing to read back: the bot refuses the test with its message and hint.
function readbackOf(model, name) {
  const part = model?.components?.find(component => component.id === 'servo_board');
  const chip = model?.diagnostics?.pca9685, channel = chip?.channels?.[name];
  if (part?.status === 'off') return [part.message || 'Simulated: commands are recorded, not sent', part.fix || '', ''];
  if (channel?.pulse_us == null) return null;
  if (readingsStale(model)) {  // An old register value is not this test's result
    return ['No fresh read-back: the bot\'s live checks stopped', 'See why in the bot\'s log: journalctl -u ultraskelly-bot', 'refused'];
  }
  const angle = channel.commanded == null ? 'the command' : `${Math.round(channel.commanded)}°`;
  const said = `${name[0].toUpperCase()}${name.slice(1)} commanded ${angle}; the chip reads ${Math.round(channel.pulse_us)} µs` +
               (channel.expected_us == null ? '' : `, ${angle} needs ${Math.round(channel.expected_us)} µs`);
  if (chip.awake === false) return [`${said}, but the chip is asleep, so it sends no pulses`, 'The bot wakes it when it starts the servos', 'refused'];
  // The bot's own advice for a board that has the command (its hint), or the usual three places to look.
  const moved = (part?.status === 'ok' && part.fix) || 'If it did not move: the OE wire (it must be low), V+ servo power on the board\'s terminal, the plug\'s orientation';
  const same = channel.matches ?? (channel.expected_us == null ? null : Math.abs(channel.pulse_us - channel.expected_us) <= SAME_US);  // The bot's verdict first
  if (same === null) return [said, moved, ''];
  if (!same) {
    return [`${said}: the command did not reach the chip`, 'Check the I2C wires to the servo board (SDA, SCL, GND)', 'refused'];
  }
  const rate = chip.frequency_hz != null && Math.abs(chip.frequency_hz - 50) > 5 ? ` at ${chip.frequency_hz.toFixed(0)} Hz, where servos want about 50` : '';
  return [`${said}${rate}: the chip has the command`, moved, rate ? 'refused' : 'done'];
}

export class MotorTestPanel {
  // `load`: ask the bot for its tests now; the control panel loads it later, once the bot runs.
  constructor(root, {load = true} = {}) {
    this.root = root;
    this.limits = null;
    this.running = false;
    this.own = null;  // `started` of the last test this page ran: a robot_state sent before it ended is not another screen's
    this.foreign = null;  // `started` of a test another screen is running, to show how it ended
    this.commands = {};
    this.lastServo = null;  // The actuator of the last servo test, whose read-back is shown
    if (load) this.load();
    else root.hidden = true;
  }

  // Asks the bot for its tests and builds the panel; again later if the bot was off (the control panel calls it once the
  // bot runs). Built once.
  async load() {
    if (this.buttons) { this.root.hidden = false; return; }
    let reply = null;
    try { reply = await fetch('/api/motor-test', {cache: 'no-store'}); } catch { /* No server: no panel */ }
    if (!reply?.ok) { this.root.hidden = true; return; }
    const state = await reply.json();
    this.limits = state.limits;
    this.build(state.enabled);
  }

  build(enabled) {
    const limits = this.limits, low = CENTER - limits.servo_range, high = CENTER + limits.servo_range;
    this.buttons = [];
    const button = (label, title, action) => {
      const made = element('button', {textContent: label, title, onclick: action});
      this.buttons.push(made);
      return made;
    };
    const number = (value, max, step, label) => element('input', {type: 'number', value, min: step, max, step, title: label, ariaLabel: label});
    const servoRow = (name, label) => {
      const shown = element('span', {className: 'motor-test-value', textContent: '—', title: 'Commanded angle, not a measured position'});
      const move = angle => this.send({actuator: name, angle: Math.min(high, Math.max(low, angle))});
      const current = () => this.commands[name] ?? CENTER;
      return [element('tr', {},
        element('td', {textContent: label}),
        element('td', {className: 'motor-test-buttons'},
          button(`−${SERVO_STEP}°`, `${label}: ${SERVO_STEP} degrees less`, () => move(current() - SERVO_STEP)),
          button(`${CENTER}°`, `${label}: back to center`, () => move(CENTER)),
          button(`+${SERVO_STEP}°`, `${label}: ${SERVO_STEP} degrees more`, () => move(current() + SERVO_STEP))),
        element('td', {}, shown)), shown];
    };
    const [panRow, panShown] = servoRow('pan', 'Head pan');
    const [tiltRow, tiltShown] = servoRow('tilt', 'Head tilt');
    this.waistDrive = number(Math.min(.5, limits.waist_max_throttle), limits.waist_max_throttle, .05, 'Waist drive');
    this.waistSeconds = number(Math.min(.3, limits.waist_max_seconds), limits.waist_max_seconds, .05, 'Waist seconds');
    this.jawDrive = number(limits.mouth_max_throttle, limits.mouth_max_throttle, .05, 'Jaw drive');
    this.jawSeconds = number(Math.min(.2, limits.mouth_max_seconds), limits.mouth_max_seconds, .05, 'Jaw seconds');
    const pulse = (actuator, sign, drive, seconds) => () =>
      this.send({actuator, throttle: sign * Number(drive.value), seconds: Number(seconds.value)});
    this.travel = element('span', {className: 'motor-test-value', textContent: '—', title: 'Net waist test drive, throttle-seconds'});
    this.shown = {pan: panShown, tilt: tiltShown};
    const table = element('table', {className: 'motor-test'},
      element('tbody', {}, panRow, tiltRow,
        element('tr', {},
          element('td', {textContent: 'Waist'}),
          element('td', {className: 'motor-test-buttons'},
            button('− drive', 'Drive the waist with negative throttle', pulse('waist', -1, this.waistDrive, this.waistSeconds)),
            button('+ drive', 'Drive the waist with positive throttle', pulse('waist', 1, this.waistDrive, this.waistSeconds))),
          element('td', {}, this.travel)),
        element('tr', {},
          element('td', {}),
          element('td', {className: 'motor-test-inputs', colSpan: 2}, this.waistDrive, element('span', {textContent: 'for'}), this.waistSeconds, element('span', {textContent: 's'}))),
        element('tr', {},
          element('td', {textContent: 'Jaw'}),
          element('td', {className: 'motor-test-buttons'},
            button('Open', 'Drive the jaw open (negative throttle); its spring closes it', pulse('mouth', -1, this.jawDrive, this.jawSeconds))),
          element('td', {})),
        element('tr', {},
          element('td', {}),
          element('td', {className: 'motor-test-inputs', colSpan: 2}, this.jawDrive, element('span', {textContent: 'for'}), this.jawSeconds, element('span', {textContent: 's'})))));
    this.stopButton = element('button', {textContent: '■ Stop', className: 'motor-test-stop', onclick: () => this.stop()});
    this.result = element('span', {className: 'motor-test-result', textContent: enabled ? 'Ready' : 'Turned off in the launch configuration', role: 'status'});
    this.boards = element('div', {className: 'motor-test-boards', hidden: true});
    this.readback = element('div', {className: 'motor-test-readback', hidden: true}, element('span'), element('span', {className: 'advice'}));
    this.root.replaceChildren(
      element('div', {className: 'section-title'}, element('h2', {textContent: 'Motor test'}), element('span', {textContent: 'ONE AT A TIME'})),
      this.boards,
      table,
      element('div', {className: 'motor-test-foot'}, this.stopButton, this.result),
      this.readback,
      element('p', {className: 'note', textContent:
        `Tracking pauses during each test. Servos ${low}–${high}°; waist up to ${limits.waist_max_throttle.toFixed(2)} for ` +
        `${limits.waist_max_seconds.toFixed(2)} s, at most ${limits.waist_max_travel.toFixed(2)} throttle-seconds one way; ` +
        `jaw up to ${limits.mouth_max_throttle.toFixed(2)} for ${limits.mouth_max_seconds.toFixed(2)} s. ` +
        `Angles are commands; the servo board's read-back is its own register, so it shows a command arrived, not where a servo is.`}));
    this.enabled = enabled;
    this.setRunning(false);
  }

  setRunning(running) {
    this.running = running;
    for (const made of this.buttons) made.disabled = running || !this.enabled;
    this.stopButton.disabled = !this.enabled;
  }

  show(text, kind = '') {
    setText(this.result, text);
    setProperty(this.result, 'className', `motor-test-result ${kind}`);
  }

  async post(path, body) {
    const reply = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    return [reply, await reply.json()];
  }

  async send(test) {
    if (this.running) return;
    if (test.angle != null) this.lastServo = test.actuator;
    this.setRunning(true);
    this.show(test.angle != null ? `Moving ${test.actuator} to ${test.angle}°…` : `Driving ${test.actuator} ${signed(test.throttle)} for ${test.seconds} s…`);
    try {
      const [reply, body] = await this.post('/api/motor-test', test);
      if (!reply.ok) this.show(`Refused: ${body.error}`, 'refused');
      else {
        this.own = body.test.started;
        this.show(`${body.test.outcome[0].toUpperCase()}${body.test.outcome.slice(1)}: ${body.test.description}`, body.ok ? 'done' : 'refused');
      }
    } catch (problem) {
      this.show(`No answer from the robot: ${problem.message}`, 'refused');
    } finally {
      this.setRunning(false);
    }
  }

  async stop() {
    try {
      const [, body] = await this.post('/api/motor-test/stop', {});
      if (!body.stopped && !this.running) this.show('Nothing was running');
    } catch (problem) {
      this.show(`No answer from the robot: ${problem.message}`, 'refused');
    }
  }

  // From each robot_state and the bot's checks (health.js's model): the commanded angles, net waist travel, tests run
  // from elsewhere, each board's presence and the last servo test's read-back.
  update(state, model) {
    if (!this.limits) return;
    const servosDown = down(model, 'servo_board'), motorsDown = down(model, 'motor_board');
    for (const name of ['pan', 'tilt']) {
      const value = state.motors?.commands?.[name]?.value;
      this.commands[name] = value ?? undefined;
      setText(this.shown[name], value == null ? '—' : `${value.toFixed(0)}°${servosDown ? ' not sent' : ''}`);
    }
    const test = state.motor_test;
    if (test?.last?.angle != null && !this.running) this.lastServo = test.last.actuator;  // Also a test from another screen
    if (!this.root.closest('[hidden]')) { this.showBoards(model); this.showReadback(model); }  // Only while open
    if (!test) return;
    setText(this.travel, `${signed(test.waist_travel)}${motorsDown ? ' not sent' : ''}`);
    if (test.running && !this.running && test.running.started !== this.own) {
      this.foreign = test.running.started;
      this.show(`Running from another screen: ${test.running.description}`);
    } else if (!test.running && this.foreign != null && test.last?.started === this.foreign) {
      this.foreign = null;  // It ended: say how, once
      const outcome = test.last.outcome;
      this.show(`${outcome[0].toUpperCase()}${outcome.slice(1)} from another screen: ${test.last.description}`, outcome === 'done' ? 'done' : 'refused');
    }
  }

  // Whether each board answers, from the bot's checks; nothing without them.
  showBoards(model) {
    const parts = BOARDS.map(([id, name]) => [model?.checked ? model.components.find(part => part.id === id) : null, name]).filter(([part]) => part);
    setProperty(this.boards, 'hidden', !parts.length);
    if (this.boards.childElementCount !== parts.length) {
      this.boards.replaceChildren(...parts.map(() => element('span', {className: 'lamp'}, element('i'), document.createTextNode(''))));
    }
    parts.forEach(([part, name], i) => {
      const lamp = this.boards.children[i];
      setProperty(lamp, 'className', `lamp ${part.status}`);
      setProperty(lamp, 'title', part.fix ? `${part.message}. Fix: ${part.fix}` : part.message);
      setText(lamp.lastChild, part.status === 'ok' ? `${name} answers` : part.message || `${name}: ${part.status}`);
    });
  }

  // After a servo test, the command next to the chip's read-back, live, and where to look if the servo did not move.
  showReadback(model) {
    const said = this.lastServo ? readbackOf(model, this.lastServo) : null;
    setProperty(this.readback, 'hidden', !said);
    if (!said) return;
    const [words, hint, kind] = said;
    setText(this.readback.firstChild, words); setText(this.readback.lastChild, hint);
    setProperty(this.readback, 'className', `motor-test-readback ${kind}`);
  }
}
