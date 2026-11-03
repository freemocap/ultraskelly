import csv
import json
import wave

import numpy as np

from scripts.analyze_run import analyze, report

RATE = 48_000
START, STOP = 100.0, 110.0


def write_run(folder, *, dropped=None):
    """A ten-second run: the waist drives 101-103 s (released) and 105-106 s (braked while driving)."""
    (folder / "run.json").write_text(json.dumps({
        "started_monotonic": START, "stopped_monotonic": STOP, "camera": "oak",
        "written": {"frame": 3, "command": 6}, "dropped": dropped or {}, "frames_missed": 0, "failure": None,
        "audio_first_sample_monotonic": START,
    }))
    with open(folder / "oak_frames.csv", "w", newline="") as file:
        csv.writer(file).writerows([("index", "frame_number", "captured", "received"),
                                    (0, 10, 100.0, 100.01), (1, 11, 100.0333, 100.04), (2, 12, 100.0667, 100.07)])
    with open(folder / "commands.csv", "w", newline="") as file:
        csv.writer(file).writerows([("time", "actuator", "value"), (100.0, "pan", 90.0), (100.0, "waist", 0.0),
                                    (101.0, "waist", 0.45), (101.2, "waist", 0.7), (103.0, "waist", None),
                                    (105.0, "waist", -0.7), (106.0, "waist", 0.0)])
    with open(folder / "targets.csv", "w", newline="") as file:
        csv.writer(file).writerows([("received", "x", "y", "angle"), (100.5, 320, 240, ""), (100.6, "", "", "")])
    rng = np.random.default_rng(1)
    t = np.arange(int((STOP - START) * RATE)) / RATE + START
    sound = 0.001 * rng.standard_normal(len(t))
    moving = ((t >= 101) & (t < 103)) | ((t >= 105) & (t < 106))
    sound[moving] += 0.05 * np.sin(2 * np.pi * 1550 * t[moving])  # The PWM whine
    click = (t >= 106.0) & (t < 106.003)
    sound[click] += 0.5 * rng.standard_normal(click.sum())  # The brake's click
    with wave.open(str(folder / "audio.wav"), "wb") as recording:
        recording.setnchannels(1)
        recording.setsampwidth(2)
        recording.setframerate(RATE)
        recording.writeframes((np.clip(sound, -1, 1) * 32767).astype("<i2").tobytes())


def test_a_run_is_summarized(tmp_path):
    write_run(tmp_path)
    result = analyze(tmp_path)
    recording, waist, sound = result["recording"], result["waist"], result["sound"]
    assert recording["complete"] and abs(recording["camera_fps"] - 30) < 0.1 and recording["audio_seconds"] == 10.0
    assert (waist["moves"], waist["released"], waist["braked_while_driving"]) == (2, 1, 1)
    assert waist["longest_move_s"] == 2.0 and waist["largest_drive"] == 0.7
    assert sound["moving_over_still_db"] > 20
    assert abs(sound["whine_hz"] - 1550) < 30
    assert sound["stop_click_db"] > sound["start_click_db"] and sound["loud_clicks"] >= 1
    assert result["targets_seen"] == 1
    assert "complete" in report(result)


def test_drops_make_a_run_incomplete(tmp_path):
    write_run(tmp_path, dropped={"frame": 2})
    assert not analyze(tmp_path)["recording"]["complete"]
