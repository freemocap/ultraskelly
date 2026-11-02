"""Drive the waist through short throttle profiles while recording its sound and motion (MOVES REAL MOTORS).

Run on the Pi with the bot stopped (this needs the OAK and the motor board), a USB
microphone near the waist, and the robot ready for motion:

    .venv/bin/python -m scripts.waist_sound_test                    # every profile
    .venv/bin/python -m scripts.waist_sound_test current steady     # just these, in order

Each profile turns the waist one way, rests, then turns it back the same way. The
microphone records the sound; the OAK measures how far the torso turned from the
horizontal image shift while the head holds still. Afterwards the waist turns back by
the angle the camera measured, so it ends roughly where it started. A table prints at
the end; the WAV, the camera video and every measurement are saved under
~/ultraskelly-recordings/. Levels are relative (an uncalibrated microphone at a fixed
gain), so compare rows, not absolute values. Medians over time keep intermittent sounds
such as speech from dominating them. DC motors are always stopped on exit, including
on Ctrl+C.
"""
import argparse
import csv
import json
import math
import threading
import time
import wave
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from ultraskelly.core.bot.motor.actuator_limits import ActuatorLimits
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder, RecordingMotor
from ultraskelly.core.bot.recording.microphone import RATE as AUDIO_RATE, Microphone, find_microphone

MIC_GAIN = 256  # Of 496 steps on the first microphone (16 dB): a fixed gain with headroom
CONTROL_PERIOD = 0.01  # The orientation controller's loop period
IDLE_SECONDS = 2.0
REST_SECONDS = 1.2
SOUND_TAIL = 0.4  # Sound after the drive ends (the stop) still belongs to the move
MOTION_TAIL = 0.8  # So does the torso's swing after it
RETURN_THROTTLE = 0.7
RETURN_TOLERANCE = 3.0  # Degrees
RETURN_SECONDS = 4.0
FRAME_SIZE = (640, 480)
HFOV_DEGREES = 69.0  # OAK-1 colour camera over the full 4:3 sensor
FOCAL_PX = FRAME_SIZE[0] / 2 / math.tan(math.radians(HFOV_DEGREES / 2))
PCA9685_CLOCK = 25e6  # Nominal; the internal oscillator often runs a few percent fast
SPECTRUM_FRAME = 2048
CLICK_FRAME = 512


@dataclass(frozen=True)
class Profile:
    """One way to drive a single waist move. Throttles are magnitudes; each move applies a direction."""

    description: str
    cruise: float = 0.75
    seconds: float = 1.0
    kick: float = 0.0  # Throttle for the first kick_seconds, to break static friction
    kick_seconds: float = 0.0
    ramp_up: float = 0.0  # Seconds to ramp from 0 to cruise
    ramp_down: float = 0.0  # Seconds to ramp from cruise to 0 at the end
    pulse_on: float = 0.0  # Drive for pulse_on seconds, brake for pulse_off seconds, repeat
    pulse_off: float = 0.0
    coast: bool = False  # Stop by releasing the motor instead of braking it
    slow_decay: bool = False  # Brake instead of coast between PWM pulses
    pwm_hz: float = 1600.0

    def throttle(self, t: float) -> float | None:
        """Throttle magnitude t seconds into the move; None releases the motor."""
        if t >= self.seconds:
            return None if self.coast else 0.0
        if self.pulse_on and t % (self.pulse_on + self.pulse_off) >= self.pulse_on:
            return 0.0  # Like the controller's cooldown, which brakes
        if t < self.kick_seconds:
            return self.kick
        level = self.cruise
        if self.ramp_up:
            level = min(level, self.cruise * t / self.ramp_up)
        if self.ramp_down:
            level = min(level, self.cruise * (self.seconds - t) / self.ramp_down)
        return level


SLOW = dict(seconds=1.2, kick=0.75, kick_seconds=0.12, ramp_down=0.2, coast=True)
PROFILES = {
    "current": Profile(
        "the controller today: 0.75 for 0.5 s, braked 0.4 s, repeated", seconds=1.4, pulse_on=0.5, pulse_off=0.4
    ),
    "steady": Profile("0.75 without pauses, braked at the end"),
    "steady-coast": Profile("0.75 without pauses, released at the end", coast=True),
    "probe": Profile(
        "ramp 0 to 0.8 over 1.5 s and back: where the waist breaks loose and where it stalls",
        cruise=0.8, seconds=3.0, ramp_up=1.5, ramp_down=1.5, coast=True,
    ),
    "ramp": Profile("ramp to 0.75 over 0.3 s, back down over 0.3 s, released", ramp_up=0.3, ramp_down=0.3, coast=True),
    "slow-55": Profile("kick 0.75 for 0.12 s, then 0.55, ramp down, released", cruise=0.55, **SLOW),
    "slow-45": Profile("kick 0.75 for 0.12 s, then 0.45, ramp down, released", cruise=0.45, **SLOW),
    "slow-55-slowdecay": Profile("slow-55 with slow decay", cruise=0.55, slow_decay=True, **SLOW),
    "slow-55-pwm500": Profile("slow-55 at 500 Hz PWM", cruise=0.55, pwm_hz=500.0, **SLOW),
    "slow-55-pwm200": Profile("slow-55 at 200 Hz PWM", cruise=0.55, pwm_hz=200.0, **SLOW),
}


def pwm_output_hz(requested: float) -> float:
    """The PCA9685's nominal PWM frequency for a requested one (adafruit_pca9685's prescale rounding)."""
    prescale = int(PCA9685_CLOCK / 4096.0 / requested + 0.5) - 1
    return PCA9685_CLOCK / 4096.0 / (prescale + 1)


class CameraMotion:
    """Saves the OAK's frames to a video and measures the horizontal image shift between them."""

    def __init__(self, video_path: Path, flip: bool) -> None:
        import depthai as dai

        self.flip = flip
        self.pipeline = dai.Pipeline()  # Connects to the first OAK found
        camera = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        self.queue = camera.requestOutput(FRAME_SIZE, type=dai.ImgFrame.Type.RGB888i, fps=30).createOutputQueue(
            maxSize=8, blocking=False
        )
        self.writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter.fourcc(*"MJPG"), 30, FRAME_SIZE)
        self.shifts: list[tuple[float, float, float, float]] = []  # (frame time, dx px, dy px, peak strength)
        self.started = threading.Event()
        self.stopping = threading.Event()
        self.pipeline.start()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        small_size = (160, 120)
        scale = FRAME_SIZE[0] / small_size[0]
        window = cv2.createHanningWindow(small_size, cv2.CV_32F)
        previous = None
        while not self.stopping.is_set():
            message = self.queue.tryGet()
            if message is None:
                time.sleep(0.002)
                continue
            frame = message.getFrame()
            if self.flip:
                frame = np.ascontiguousarray(frame[::-1, ::-1])
            self.writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            small = cv2.resize(gray, small_size, interpolation=cv2.INTER_AREA).astype(np.float32)
            if previous is not None:
                (dx, dy), strength = cv2.phaseCorrelate(previous, small, window)
                self.shifts.append((message.getTimestamp().total_seconds(), dx * scale, dy * scale, strength))
            previous = small
            self.started.set()

    def turned_since(self, start: float) -> float:
        """Degrees the camera has turned since `start`, from the shifts measured so far."""
        return sum(math.degrees(math.atan(dx / FOCAL_PX)) for t, dx, _, _ in list(self.shifts) if t >= start)

    def stop(self) -> None:
        self.stopping.set()
        self.thread.join(timeout=5)
        self.writer.release()
        self.pipeline.stop()
        self.pipeline.getDefaultDevice().close()  # Free the USB device for the bot


def drive(waist: RecordingMotor, profile: Profile, direction: int) -> tuple[float, float]:
    """Run one move at the controller's rate; return when its drive started and ended."""
    start = time.monotonic()
    while (t := time.monotonic() - start) < profile.seconds:
        value = profile.throttle(t)
        waist.throttle = None if value is None else direction * value
        time.sleep(CONTROL_PERIOD)
    waist.throttle = profile.throttle(profile.seconds)
    return start, time.monotonic()


def return_waist(waist: RecordingMotor, camera: CameraMotion, since: float, offset: float) -> None:
    """Turn the waist back by the angle measured since `since`, plus `offset` from earlier runs.

    A positive throttle turns the camera's view positive, so the return drives against the sign.
    """
    error = camera.turned_since(since) + offset
    if abs(error) < RETURN_TOLERANCE:
        return
    print(f"Returning the waist {-error:+.0f} degrees", flush=True)
    start = time.monotonic()
    waist.throttle = -math.copysign(RETURN_THROTTLE, error)
    while time.monotonic() - start < RETURN_SECONDS:
        remaining = camera.turned_since(since) + offset
        if remaining * error <= 0 or abs(remaining) < RETURN_TOLERANCE:
            break
        time.sleep(CONTROL_PERIOD)
    waist.throttle = None
    time.sleep(REST_SECONDS)
    print(f"Waist is {camera.turned_since(since) + offset:+.0f} degrees from where it started", flush=True)


def a_weighting(frequencies: np.ndarray) -> np.ndarray:
    """IEC 61672 A-weighting as a power gain, 1 at 1 kHz."""
    f2 = np.square(frequencies)
    ra = 12194.0**2 * f2**2 / ((f2 + 20.6**2) * np.sqrt((f2 + 107.7**2) * (f2 + 737.9**2)) * (f2 + 12194.0**2))
    return np.square(ra * 10 ** (2.0 / 20))


def spectra(samples: np.ndarray, size: int = SPECTRUM_FRAME) -> np.ndarray:
    """Power per bin of half-overlapping Hann frames; a band's sum reads 1 for a full-scale sine."""
    hop = size // 2
    count = (len(samples) - size) // hop + 1
    if count < 1:
        return np.zeros((0, size // 2 + 1))
    window = np.hanning(size)
    frames = np.stack([samples[i * hop : i * hop + size] for i in range(count)])
    power = np.square(np.abs(np.fft.rfft(frames * window, axis=1)))
    return power * 4 / (size * np.sum(np.square(window)))


def decibels(power: float) -> float:
    return round(10 * math.log10(power + 1e-20), 1)


def sound_metrics(samples: np.ndarray, audio_start: float, t0: float, t1: float, pwm_hz: float) -> dict:
    """Typical A-weighted level, the PWM whine's part of it, and click peaks in a stretch of audio."""
    first, last = (max(0, int((t - audio_start) * AUDIO_RATE)) for t in (t0, t1))
    stretch = samples[first:last]
    power = spectra(stretch)
    if not len(power):
        return {}
    frequencies = np.fft.rfftfreq(SPECTRUM_FRAME, 1 / AUDIO_RATE)
    weighted = power * a_weighting(frequencies)
    typical = np.median(weighted, axis=0)  # The steady sound; speech comes and goes
    whine = np.zeros(frequencies.shape, dtype=bool)
    expected = pwm_output_hz(pwm_hz)
    near = np.flatnonzero(np.abs(frequencies - expected) <= 0.1 * expected)
    fundamental = float(frequencies[near[np.argmax(typical[near])]])
    for harmonic in range(1, 5):
        peak = int(round(harmonic * fundamental / frequencies[1]))
        whine[max(0, peak - 2) : peak + 3] = True
    click_power = spectra(stretch, CLICK_FRAME)
    click_frequencies = np.fft.rfftfreq(CLICK_FRAME, 1 / AUDIO_RATE)
    high = (click_frequencies >= 4000) & (click_frequencies <= 12000)
    click_levels = 10 * np.log10((click_power[:, high] * a_weighting(click_frequencies[high])).sum(axis=1) + 1e-20)
    return {
        "level_db": decibels(typical.sum()),
        "whine_db": decibels(typical[whine].sum()),
        "rest_db": decibels(typical[~whine].sum()),
        "click_db": round(float(click_levels.max() - np.median(click_levels)), 1) if len(click_levels) else None,
        "whine_hz": fundamental,
        "clipped": bool(np.abs(stretch).max(initial=0) >= 0.999),
    }


def motion_metrics(shifts: list, t0: float, drive_end: float, t1: float) -> dict:
    """How far and how smoothly the camera turned over a move, from the horizontal image shift."""
    turns = np.array([(t, math.degrees(math.atan(dx / FOCAL_PX))) for t, dx, _, _ in shifts if t0 <= t <= t1])
    if not len(turns):
        return {}
    times, rates = turns[:, 0], turns[:, 1] * 30
    sign = 1.0 if turns[:, 1].sum() >= 0 else -1.0
    driving = times <= drive_end
    settled = driving & (times >= (t0 + drive_end) / 2)
    steady = float(np.median(np.abs(rates[settled]))) if settled.any() else 0.0
    return {
        "turned_deg": round(float(turns[:, 1].sum()), 1),
        "speed_dps": round(float(turns[driving, 1].sum()) / max(drive_end - t0, 1e-3), 1),
        "steady_dps": round(steady, 1),
        "peak_dps": round(float(np.abs(rates[driving]).max(initial=0)), 1),
        # Reverse swing while driving (pulses) and after stopping: the jerk you see and hear
        "swing_dps": round(float(max(0.0, (-sign * rates).max(initial=0))), 1),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive the waist through throttle profiles while recording sound and motion (moves real motors).")
    parser.add_argument("profiles", nargs="*", help=f"Profiles to run, in order (default: all): {', '.join(PROFILES)}")
    parser.add_argument("--mic", default="USB PnP Audio Device", help="Part of the microphone's ALSA card name")
    parser.add_argument("--gain", type=int, default=None, help="Set the microphone's 'Mic' capture control to this many steps")
    parser.add_argument("--offset", type=float, default=0.0, help="Degrees the waist was already turned by earlier runs, to undo")
    parser.add_argument("--no-flip", action="store_true", help="The OAK is mounted upright (the bot flips it by default)")
    args = parser.parse_args()
    selected = args.profiles or list(PROFILES)
    unknown = [name for name in selected if name not in PROFILES]
    if unknown:
        parser.error(f"unknown profiles {unknown}; choose from {list(PROFILES)}")

    try:
        card = find_microphone(args.mic)
    except ValueError as error:
        raise SystemExit(str(error))
    folder = Path.home() / "ultraskelly-recordings" / f"waist-sound-{datetime.now():%Y%m%d-%H%M%S}"
    folder.mkdir(parents=True)
    orientation, limits, recorder = OrientationNodeParams(), ActuatorLimits(), CommandRecorder()

    from adafruit_motor import motor as dc_motor
    from adafruit_motorkit import MotorKit

    camera = CameraMotion(folder / "camera.avi", flip=not args.no_flip)
    if not camera.started.wait(timeout=20):
        camera.stop()
        raise SystemExit("No frames from the OAK; is the bot still running?")
    kit = MotorKit(address=0x60, pwm_frequency=1600)
    motors = {number: getattr(kit, f"motor{number}") for number in (1, 2, 3, 4)}
    raw_waist = motors[orientation.waist_motor_channel]
    waist = RecordingMotor(motor=raw_waist, name="waist", recorder=recorder, max_throttle=limits.waist_max_throttle)
    audio = Microphone(card, folder / "audio.wav", gain=args.gain)
    segments = []
    try:
        time.sleep(0.5)  # arecord's first chunks
        began = time.monotonic()
        time.sleep(IDLE_SECONDS)
        segments.append(dict(profile="idle", direction=0, start=began, drive_end=time.monotonic(), end=time.monotonic()))
        for name in selected:
            profile = PROFILES[name]
            kit._pca.frequency = profile.pwm_hz
            raw_waist.decay_mode = dc_motor.SLOW_DECAY if profile.slow_decay else dc_motor.FAST_DECAY
            for direction in (+1, -1):
                print(f"{name} {direction:+d}: {profile.description}", flush=True)
                start, drive_end = drive(waist, profile, direction)
                time.sleep(REST_SECONDS)
                segments.append(dict(profile=name, direction=direction, start=start, drive_end=drive_end, end=time.monotonic()))
        kit._pca.frequency = 1600
        raw_waist.decay_mode = dc_motor.FAST_DECAY
        return_waist(waist, camera, began, args.offset)
    finally:
        for motor in motors.values():
            motor.throttle = 0.0
        raw_waist.decay_mode = dc_motor.FAST_DECAY
        kit._pca.frequency = 1600
        print("All DC motors stopped", flush=True)
        audio_start = audio.stop()
        camera.stop()

    # Save the raw measurements first, so a failure in the analysis below loses nothing.
    with open(folder / "motion.csv", "w", newline="") as output:
        csv.writer(output).writerows([("time", "dx_px", "dy_px", "strength"), *camera.shifts])
    with open(folder / "commands.csv", "w", newline="") as output:
        csv.writer(output).writerows([("time", "actuator", "throttle"), *((c.at, c.actuator, c.value) for c in recorder.history)])
    results = {
        "audio_start": audio_start,
        "mic_card": card,
        "mic": args.mic,
        "mic_gain_steps": args.gain,
        "profiles": {name: asdict(PROFILES[name]) for name in selected},
        "segments": segments,
    }
    (folder / "results.json").write_text(json.dumps(results, indent=2))

    with wave.open(str(folder / "audio.wav")) as recording:
        samples = np.frombuffer(recording.readframes(recording.getnframes()), dtype="<i2") / 32768.0
    for segment in segments:
        profile = PROFILES.get(segment["profile"])
        sound_end = segment["end"] if profile is None else segment["drive_end"] + SOUND_TAIL
        motion_end = segment["end"] if profile is None else segment["drive_end"] + MOTION_TAIL
        segment.update(sound_metrics(samples, audio_start, segment["start"], sound_end, profile.pwm_hz if profile else 1600.0))
        segment.update(motion_metrics(camera.shifts, segment["start"], segment["drive_end"], motion_end))
    (folder / "results.json").write_text(json.dumps(results, indent=2))

    print(f"\n{'profile':<19}{'dir':>4}{'turned':>8}{'deg/s':>7}{'steady':>7}{'peak':>6}{'swing':>7}"
          f"{'level':>7}{'whine':>7}{'rest':>7}{'click':>7}{'pwm Hz':>8}")
    for s in segments:
        def show(key: str, width: int, digits: int = 1) -> str:
            value = s.get(key)
            return f"{'-':>{width}}" if value is None else f"{value:>{width}.{digits}f}"
        print(f"{s['profile']:<19}{s['direction']:>+4d}{show('turned_deg', 8)}{show('speed_dps', 7)}{show('steady_dps', 7)}"
              f"{show('peak_dps', 6, 0)}{show('swing_dps', 7)}{show('level_db', 7)}{show('whine_db', 7)}{show('rest_db', 7)}"
              f"{show('click_db', 7)}{show('whine_hz', 8, 0)}{'  CLIPPED' if s.get('clipped') else ''}")
    print("\nSpeeds are camera degrees per second; swing is the reverse speed when the torso bounces back.")
    print(f"Levels are median dB re full scale, A-weighted; click is the loudest 11 ms above 4 kHz over the median. Saved to {folder}")


if __name__ == "__main__":
    main()
