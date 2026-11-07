"""Summarize a recorded run: whether everything was kept, how the waist moved, and how loud it was.

    python -m scripts.analyze_run ~/ultraskelly-recordings/run-20261102-160000

Prints the summary and saves it as analysis.json in the run's folder. Waist moves come from
the commanded throttle in commands.csv, not a measured angle. Sound levels are relative (an
uncalibrated microphone at a fixed gain), so compare runs recorded the same way. Without
OpenCV the video is not decoded.
"""
import argparse
import csv
import json
import wave
from pathlib import Path

import numpy as np

from scripts.waist_sound_test import a_weighting, decibels, spectra

FRAME = 2048  # Samples per level measurement (43 ms at 48 kHz, half overlapping)
CLICK_FRAME = 512  # Samples per click measurement (11 ms)
SOUND_TAIL = 0.3  # Seconds after a move's drive ends whose sound still belongs to it (the stop)
EVENT_WINDOW = 0.1  # Seconds either side of a start or stop searched for its click
LOUD_CLICK_DB = 10.0  # A click this far above the still background counts as loud


def read_rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path, newline="") as file:
        return list(csv.DictReader(file))


def number(text: str) -> float | None:
    return None if text in ("", "None") else float(text)


def video_frame_count(path: Path) -> int | None:
    """Frames OpenCV can decode from the run's video; None without OpenCV or a video."""
    try:
        import cv2
    except ImportError:
        return None
    if not path.exists():
        return None
    video = cv2.VideoCapture(str(path))
    count = 0
    while video.read()[0]:
        count += 1
    video.release()
    return count


def video_frames(folder: Path, camera: str) -> int | None:
    """Frames in the run's video: <camera>.avi, and <camera>-2.avi, -3.avi, ... after each reached 4 GiB."""
    counts = [video_frame_count(path) for path in [folder / f"{camera}.avi", *folder.glob(f"{camera}-*.avi")]]
    return None if any(count is None for count in counts) else sum(counts)


def recording_summary(folder: Path, run: dict) -> dict:
    """What the recorder kept: counts, drops, frame gaps, and whether the video and audio cover the run."""
    camera = run.get("camera", "camera")
    frames = read_rows(folder / f"{camera}_frames.csv")
    numbers = [int(row["frame_number"]) for row in frames if number(row["frame_number"]) is not None]
    captured = np.array([float(row["captured"]) for row in frames if number(row["captured"]) is not None])
    gaps = np.diff(captured)
    summary = {
        "seconds": round(run["stopped_monotonic"] - run["started_monotonic"], 2),
        "written": run.get("written", {}),
        "dropped": run.get("dropped", {}),
        "frames_missed": run.get("frames_missed"),
        "failure": run.get("failure"),
        "frame_numbers_skipped": int(sum(max(0, b - a - 1) for a, b in zip(numbers, numbers[1:]))),
        "camera_fps": round(1 / float(np.median(gaps)), 2) if len(gaps) else None,
        "longest_frame_gap_s": round(float(gaps.max()), 3) if len(gaps) else None,
        "video_frames": video_frames(folder, camera),
        "audio_seconds": None,
        "audio_starts_after_run_s": None,
    }
    audio = folder / "audio.wav"
    if audio.exists():
        with wave.open(str(audio)) as recording:
            summary["audio_seconds"] = round(recording.getnframes() / recording.getframerate(), 2)
        first = run.get("audio_first_sample_monotonic")
        summary["audio_starts_after_run_s"] = None if first is None else round(first - run["started_monotonic"], 3)
    summary["complete"] = (
        not any(summary["dropped"].values()) and not summary["frames_missed"] and summary["failure"] is None
        and summary["video_frames"] in (None, summary["written"].get("frame", 0))
    )
    return summary


def waist_moves(commands: list[tuple[float, float | None]], end: float) -> tuple[list[tuple[float, float]], dict]:
    """Intervals of nonzero waist drive, and how they ended: braked while driving, released, or still going."""
    moves, endings, began = [], {"braked": 0, "released": 0, "unfinished": 0}, None
    for at, value in commands:
        driving = value is not None and value != 0.0
        if driving and began is None:
            began = at
        elif not driving and began is not None:
            moves.append((began, at))
            endings["released" if value is None else "braked"] += 1
            began = None
    if began is not None:
        moves.append((began, end))
        endings["unfinished"] += 1
    return moves, endings


def waist_summary(commands: list[tuple[float, float | None]], moves: list, endings: dict, seconds: float) -> dict:
    lengths = sorted(b - a for a, b in moves)
    drives = [abs(value) for _, value in commands if value is not None]
    return {
        "moves": len(moves),
        "moves_per_minute": round(len(moves) / (seconds / 60), 1) if seconds else None,
        "braked_while_driving": endings["braked"],
        "released": endings["released"],
        "median_move_s": round(lengths[len(lengths) // 2], 2) if lengths else None,
        "longest_move_s": round(lengths[-1], 2) if lengths else None,
        "driven_fraction": round(sum(lengths) / seconds, 3) if seconds else None,
        "largest_drive": max(drives, default=None),
    }


def sound_summary(folder: Path, run: dict, moves: list) -> dict | None:
    """Median A-weighted level while the waist moves and while it is still, its PWM whine, and its clicks."""
    audio, first = folder / "audio.wav", run.get("audio_first_sample_monotonic")
    if not audio.exists() or first is None:
        return None
    with wave.open(str(audio)) as recording:
        rate = recording.getframerate()
        samples = np.frombuffer(recording.readframes(recording.getnframes()), dtype="<i2") / 32768.0
    start, stop = run["started_monotonic"], run["stopped_monotonic"]

    def frame_times(size: int, count: int) -> np.ndarray:
        return first + (np.arange(count) * (size // 2) + size / 2) / rate

    def moving_mask(times: np.ndarray) -> np.ndarray:
        mask = np.zeros(len(times), dtype=bool)
        for began, ended in moves:
            mask |= (times >= began) & (times <= ended + SOUND_TAIL)
        return mask

    power = spectra(samples, FRAME)
    if not len(power):
        return None
    times = frame_times(FRAME, len(power))
    frequencies = np.fft.rfftfreq(FRAME, 1 / rate)
    weighted = power * a_weighting(frequencies)
    in_run = (times >= start) & (times <= stop)
    moving = moving_mask(times) & in_run
    still = in_run & ~moving_mask(times)
    summary = {"clipped": bool(np.abs(samples).max(initial=0) >= 0.999)}
    level = lambda mask: decibels(float(np.median(weighted[mask].sum(axis=1)))) if mask.any() else None
    summary["level_moving_db"], summary["level_still_db"] = level(moving), level(still)
    if summary["level_moving_db"] is not None and summary["level_still_db"] is not None:
        summary["moving_over_still_db"] = round(summary["level_moving_db"] - summary["level_still_db"], 1)
    if moving.any():
        typical = np.median(weighted[moving], axis=0)
        near = np.flatnonzero((frequencies >= 1300) & (frequencies <= 1800))  # The motor board's PWM, about 1.5-1.65 kHz
        fundamental = float(frequencies[near[np.argmax(typical[near])]])
        whine = np.zeros(frequencies.shape, dtype=bool)
        for harmonic in range(1, 5):
            peak = int(round(harmonic * fundamental / frequencies[1]))
            whine[max(0, peak - 2):peak + 3] = True
        summary["whine_hz"], summary["whine_moving_db"] = fundamental, decibels(float(typical[whine].sum()))

    click_power = spectra(samples, CLICK_FRAME)
    click_times = frame_times(CLICK_FRAME, len(click_power))
    click_frequencies = np.fft.rfftfreq(CLICK_FRAME, 1 / rate)
    high = (click_frequencies >= 4000) & (click_frequencies <= 12000)
    highs = 10 * np.log10((click_power[:, high] * a_weighting(click_frequencies[high])).sum(axis=1) + 1e-20)
    quiet = (click_times >= start) & (click_times <= stop) & ~moving_mask(click_times)
    if quiet.any():
        background = float(np.median(highs[quiet]))

        def click(at: float) -> float | None:
            near = np.abs(click_times - at) <= EVENT_WINDOW
            return round(float(highs[near].max()) - background, 1) if near.any() else None

        starts = [c for c in (click(began) for began, _ in moves) if c is not None]
        stops = [c for c in (click(ended) for _, ended in moves) if c is not None]
        summary["start_click_db"] = round(float(np.median(starts)), 1) if starts else None
        summary["stop_click_db"] = round(float(np.median(stops)), 1) if stops else None
        summary["loud_clicks"] = sum(c > LOUD_CLICK_DB for c in starts + stops)
    return summary


def analyze(folder: Path) -> dict:
    run = json.loads((folder / "run.json").read_text())
    commands = [
        (float(row["time"]), number(row["value"]))
        for row in read_rows(folder / "commands.csv") if row["actuator"] == "waist"
    ]
    commands.sort(key=lambda command: command[0])
    recording = recording_summary(folder, run)
    moves, endings = waist_moves(commands, run["stopped_monotonic"])
    return {
        "recording": recording,
        "waist": waist_summary(commands, moves, endings, recording["seconds"]),
        "sound": sound_summary(folder, run, moves),
        "targets_seen": sum(number(row["x"]) is not None for row in read_rows(folder / "targets.csv")),
    }


def report(result: dict) -> str:
    r, w, s = result["recording"], result["waist"], result["sound"]
    def show(value, digits=1, unit=""):
        return "-" if value is None else f"{value:.{digits}f}{unit}"
    lines = [
        f"Recording: {r['seconds']:.1f} s, {r['written'].get('frame', 0)} frames at {show(r['camera_fps'])} fps "
        f"(dropped {sum(r['dropped'].values())}, missed {r['frames_missed']}, longest gap {show(r['longest_frame_gap_s'], 3, ' s')}), "
        f"video decodes {r['video_frames'] if r['video_frames'] is not None else '(not checked)'}, "
        f"audio {show(r['audio_seconds'], 1, ' s')} -> {'complete' if r['complete'] else 'INCOMPLETE'}",
        f"Targets seen in {result['targets_seen']} detections",
        f"Waist: {w['moves']} moves ({show(w['moves_per_minute'])}/min), median {show(w['median_move_s'], 2, ' s')}, "
        f"longest {show(w['longest_move_s'], 2, ' s')}, driven {show(None if w['driven_fraction'] is None else 100 * w['driven_fraction'], 0, '%')} "
        f"of the time; {w['braked_while_driving']} braked while driving, {w['released']} released; largest drive {show(w['largest_drive'], 2)}",
    ]
    if s:
        lines.append(
            f"Sound: {show(s.get('level_moving_db'))} dB(A) moving vs {show(s.get('level_still_db'))} still "
            f"({show(s.get('moving_over_still_db'))} dB); whine {show(s.get('whine_moving_db'))} dB at {show(s.get('whine_hz'), 0, ' Hz')}; "
            f"clicks over background: starts {show(s.get('start_click_db'))} dB, stops {show(s.get('stop_click_db'))} dB, "
            f"{s.get('loud_clicks', 0)} over {LOUD_CLICK_DB:.0f} dB{'; CLIPPED' if s.get('clipped') else ''}"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize a recorded run.")
    parser.add_argument("folder", type=Path, help="A run folder under ~/ultraskelly-recordings")
    args = parser.parse_args()
    result = analyze(args.folder)
    (args.folder / "analysis.json").write_text(json.dumps(result, indent=2))
    print(report(result))


if __name__ == "__main__":
    main()
