// Motor test panel: moves one actuator at a time through /api/motor-test. The bot applies the
// test limits and pauses tracking during each test; every test is answered once it has
// finished, been stopped or been refused, and the answer is shown here. From the bot's hardware
// checks (robot_state.health) it shows whether each board answers and, after a servo test, the
// command next to what the servo board reads back: angles are commands, not measured positions.
import { setProperty, setText } from '/dom.js';

const CENTER = 90;      // Degrees; the head's neutral angle
const SERVO_STEP = 10;  // Degrees per nudge

const element = (tag, properties = {}, ...children) => {
  const made = Object.assign(document.createElement(tag), properties);
  made.append(...children);
  return made;
};
const signed = value => `${value >= 0 ? '+' : '−'}${Math.abs(value).toFixed(2)}`;
const percent = duty => `${(duty * 100).toFixed(2)}%`;
const BOARDS = [['servo_board', 'Servo board 0x40'], ['motor_hat', 'Motor HAT 0x60']];
const SAME_DUTY = 0.0015;  // Read-back this close to the commanded angle's duty is the command: about 1.4 degrees

// What the servo board says about the last servo test's channel, from the bot's checks: [words, hint, kind], or null.
function readbackOf(health, name) {
  const board = health?.components?.find(part => part.id === 'servo_board'), channel = health?.servo_board?.channels?.[name];
  if (board && (board.status === 'fail' || board.status === 'warn') && !channel) {
    return [`${board.message || 'The servo board does not answer'}: the command went nowhere`, board.fix || '', 'refused'];
  }
  if (!channel || channel.duty == null || channel.expected_duty == null) return null;
  const servo = health.servo_board, angle = channel.commanded_deg == null ? 'the command' : `${Math.round(channel.commanded_deg)}°`;
  const said = `${name[0].toUpperCase()}${name.slice(1)} commanded ${angle}; the chip reads ${percent(channel.duty)}, ${angle} needs ${percent(channel.expected_duty)}`;
  if (servo.awake === false) return [`${said}, but the chip is asleep, so it sends no pulses`, 'The bot wakes it when it starts the servos', 'refused'];
  if (Math.abs(channel.duty - channel.expected_duty) > SAME_DUTY) {
    return [`${said}: the command did not reach the chip`, 'Check the I2C wires to the servo board (SDA, SCL, GND)', 'refused'];
  }
  const rate = servo.frequency_hz != null && Math.abs(servo.frequency_hz - 50) > 5 ? ` at ${servo.frequency_hz.toFixed(0)} Hz, where servos want about 50` : '';
  return [`${said}${rate}: the chip has the command`,
          'If it did not move: the OE wire (it must be low), V+ servo power on the board\'s terminal, the plug\'s orientation', rate ? 'refused' : 'done'];
}

export class MotorTestPanel {
  constructor(root) {
    this.root = root;
    this.limits = null;
    this.running = false;
    this.commands = {};
    this.lastServo = null;  // The actuator of the last servo test, whose read-back is shown
    this.load();
  }

  async load() {
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
    this.result.textContent = text;
    this.result.className = `motor-test-result ${kind}`;
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
      else this.show(`${body.test.outcome[0].toUpperCase()}${body.test.outcome.slice(1)}: ${body.test.description}`, body.ok ? 'done' : 'refused');
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

  // From each robot_state: the commanded angles, net waist travel, and tests run from elsewhere.
  update(state) {
    if (!this.limits) return;
    for (const name of ['pan', 'tilt']) {
      const value = state.motors?.commands?.[name]?.value;
      this.commands[name] = value ?? undefined;
      setText(this.shown[name], value == null ? '—' : `${value.toFixed(0)}°`);
    }
    const test = state.motor_test;
    if (test?.last?.angle != null && !this.running) this.lastServo = test.last.actuator;  // Also a test from another screen
    if (!this.root.closest('[hidden]')) { this.showBoards(state.health); this.showReadback(state.health); }  // Only while open
    if (!test) return;
    setText(this.travel, signed(test.waist_travel));
    if (test.running && !this.running) this.show(`Running from another screen: ${test.running.description}`);
  }

  // Whether each board answers, from the bot's checks; nothing without them.
  showBoards(health) {
    const parts = BOARDS.map(([id, name]) => [health?.components?.find(part => part.id === id), name]).filter(([part]) => part);
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
  showReadback(health) {
    const said = this.lastServo ? readbackOf(health, this.lastServo) : null;
    setProperty(this.readback, 'hidden', !said);
    if (!said) return;
    const [words, hint, kind] = said;
    setText(this.readback.firstChild, words); setText(this.readback.lastChild, hint);
    setProperty(this.readback, 'className', `motor-test-readback ${kind}`);
  }
}
