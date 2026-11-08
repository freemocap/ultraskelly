"""Measure the camera's turn in every frame of a record_motor_response session, and each servo's step response.

`measure` registers every frame against a fixed reference (simulator/camera_rotation.py): the median
of the still part's middle frames, the head at its middle. The head's parts are all within that one
reference's view; the waist turns out of it, so its frames register against keyframes laid along the
turn. Before measuring it finds what of the robot itself is in the picture (`self_mask`), takes the
camera's intrinsics from the device's calibration turned as the bot turns the image (or, without one,
from the frames: `calibrate`), and times each frame at its middle row's mid-exposure. It writes beside
the session:

- views.npz: per frame its time (s, at mid-exposure), DepthAI stamp, yaw, pitch and roll (degrees: the
  view turning right, down, and the picture's content turning clockwise are positive) and the quality
  of its registration (feature matches and inliers, ECC correlation, how far ECC moved it, keyframe);
- camera.json: the intrinsics used, the self-calibration beside them, and the still part's noise floor;
- self-mask.png: what was masked as the robot.

`analyze_servo` measures each servo's gain each way, slack at reversals, and the dead time, rise,
overshoot and settling of steps, from a measured view and the commands.

    python -m scripts.analyze_motor_response ~/ultraskelly-recordings/motor-response-20261107-193000
"""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

SETTLED = 0.4  # Seconds at the end of a hold whose views are averaged as the settled view
# Seconds the IMX378 takes to read out a frame in each sensor mode (source size), from Luxonis
# (depthai issue 795). An unknown mode is taken as 1080p's, and camera.json says so.
READOUT = {(4056, 3040): 0.03304, (3840, 2160): 0.02358, (1920, 1080): 0.01652}
DEFAULT_READOUT = 0.01652
REFERENCE_FRAMES = 15  # Still frames whose median is the reference
WAIST_KEYFRAMES = 12.0  # Degrees of turn from the nearest keyframe that make a waist frame a new one


def load_session(folder: Path) -> dict:
    """The session's frame rows, commands (time, actuator, value; None for a released motor), parts and settings."""
    with open(folder / "oak_frames.csv", newline="") as file:
        rows = list(csv.DictReader(file))
    frames = {name: np.array([float(row[name]) for row in rows]) for name in rows[0]}
    with open(folder / "commands.csv", newline="") as file:
        commands = [(float(r["time"]), r["actuator"], None if r["value"] == "" else float(r["value"]))
                    for r in csv.DictReader(file)]
    return {"frames": frames, "commands": commands, "segments": json.loads((folder / "segments.json").read_text()),
            "info": json.loads((folder / "session.json").read_text())}


def readout_seconds(info: dict) -> tuple[float, str]:
    source = tuple(info.get("frame_transformation", {}).get("source_size", ()))
    if source in READOUT:
        return READOUT[source], f"sensor mode {source[0]}x{source[1]}"
    return DEFAULT_READOUT, f"sensor mode {source or 'unknown'}: taken as 1080p's"


def turned_half_way(matrix: np.ndarray, distortion: np.ndarray, size: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """Intrinsics for the image turned 180 degrees: the bot reads the OAK-1's sensor out the other way up
    from the board's default, which the device's calibration (and the frame's transformation) describe."""
    width, height = size
    turned = matrix.copy()
    turned[0, 2], turned[1, 2] = width - 1 - matrix[0, 2], height - 1 - matrix[1, 2]
    flipped = distortion.copy()
    for index in (2, 3, 8, 9, 10, 11, 12, 13):  # Tangential, thin-prism and tilt terms change sign
        if index < len(flipped):
            flipped[index] = -flipped[index]
    return turned, flipped


def device_camera(info: dict):
    """The camera as the device's calibration describes it, turned as the session turned the image; None without one."""
    from ultraskelly.simulator.camera_rotation import Camera

    size = tuple(info["size"])
    calibration = info.get("device_calibration", {})
    if "matrix" not in calibration:
        return None
    matrix, distortion = turned_half_way(np.array(calibration["matrix"]), np.array(calibration["distortion"]), size)
    return Camera(matrix, size, distortion)


def parts_named(segments: list[dict], prefix: str) -> list[dict]:
    return [s for s in segments if s["name"].startswith(prefix)]


def measure(folder: Path, workers: int = 14) -> dict:
    """Measure every frame of the session in `folder`; writes views.npz, camera.json and self-mask.png."""
    import cv2

    from ultraskelly.simulator.camera_rotation import (
        AviFrames,
        Camera,
        RotationTracker,
        calibrate,
        exposure_middle,
        measure_frames,
        self_mask,
    )

    session = load_session(folder)
    frames, segments, info = session["frames"], session["segments"], session["info"]
    readout, readout_note = readout_seconds(info)
    stamps = frames["stamp"]
    times = np.array([exposure_middle(s, e, readout) for s, e in zip(stamps, frames["exposure"])])
    source = AviFrames(folder / "oak.avi")
    still = parts_named(segments, "still")[0]
    inside = np.flatnonzero((times >= still["start"]) & (times <= still["end"]))
    middle = inside[len(inside) // 2 - REFERENCE_FRAMES // 2:][:REFERENCE_FRAMES]
    reference = np.median(np.stack([source.read(int(i)) for i in middle]), axis=0).astype(np.uint8)

    size = tuple(info["size"])
    device = device_camera(info)
    rough_camera = device or Camera.from_field_of_view(size, 67.0)
    waist_parts = parts_named(segments, "waist")
    head_end = (waist_parts[0]["start"] - 1.0) if waist_parts else times[-1] + 1.0
    head = np.flatnonzero(times < head_end)
    rough = measure_frames(source, rough_camera, reference, head[::6].tolist(), refine=False, workers=workers)
    away = rough["ok"] & ((np.abs(rough["yaw"]) > 3) | (np.abs(rough["pitch"]) > 1.5) | (np.abs(rough["roll"]) > 1.5))
    turned = rough["index"][away]
    mask = self_mask(np.stack([source.read(int(i)) for i in turned[:: max(1, len(turned) // 300)]]))
    cv2.imwrite(str(folder / "self-mask.png"), mask)

    # The frames' own intrinsics, beside the device's: rotation-only bundle adjustment over still frames
    seen = rough["index"]
    speed = np.abs(np.gradient(rough["yaw"], times[seen])) + np.abs(np.gradient(rough["pitch"], times[seen]))
    calm = rough["ok"] & (speed < 0.5)
    order = np.argsort(rough["yaw"][calm])
    picks = seen[calm][order][np.linspace(0, calm.sum() - 1, min(60, int(calm.sum()))).astype(int)]
    found, report = calibrate(rough_camera, reference, [source.read(int(i)) for i in picks], mask=mask, radial_terms=0)
    camera = device or found
    camera_note = "device calibration, turned 180 degrees as the session reads the sensor" if device else "self-calibrated"

    table = measure_frames(source, camera, reference, head.tolist(), self_mask=mask, workers=workers)
    keyframes = 1
    if waist_parts:
        waist = np.flatnonzero(times >= head_end)
        tracker = RotationTracker(camera, reference, self_mask=mask, refine=False, new_keyframe_beyond=WAIST_KEYFRAMES)
        walked = {}
        for i in waist[::3]:
            found_turn = tracker.measure(source.read(int(i)))
            if found_turn.ok:
                walked[int(i)] = found_turn.orientation
        known = np.array(sorted(walked))
        guesses = [walked[int(known[np.argmin(np.abs(known - i))])] for i in waist]
        waist_table = measure_frames(source, camera, reference, waist.tolist(), self_mask=mask,
                                     keyframes=tracker.keyframes, guesses=guesses, workers=workers)
        table = {name: np.concatenate([table[name], waist_table[name]]) for name in table}
        keyframes = len(tracker.keyframes)
    order = np.argsort(table["index"])
    table = {name: column[order] for name, column in table.items()}
    np.savez_compressed(folder / "views.npz", times=times[table["index"]], stamps=stamps[table["index"]], **table)

    clear = np.flatnonzero((times >= still["start"] + 0.5) & (times <= still["end"] - 0.5))  # Away from the parts beside it
    quiet = np.isin(table["index"], np.setdiff1d(clear, middle)) & table["ok"]
    noise = {axis: float(np.std(table[axis][quiet])) for axis in ("yaw", "pitch", "roll")}
    summary = {
        "camera": {"used": camera_note, "matrix": camera.matrix.tolist(), "distortion": camera.distortion.tolist(),
                   "horizontal_degrees": math.degrees(2 * math.atan(size[0] / 2 / camera.matrix[0, 0]))},
        "self_calibration": report, "readout": readout, "readout_note": readout_note,
        "noise_floor_degrees": noise, "noise_frames": int(quiet.sum()), "registered": float(table["ok"].mean()),
        "waist_keyframes": keyframes,
        "ecc_moved_99th_percentile_degrees": float(np.nanpercentile(table["refinement"], 99)),
    }
    (folder / "camera.json").write_text(json.dumps(summary, indent=1))
    return summary


def command_series(commands: list[tuple[float, str, float]], name: str, times: np.ndarray, start: float = 90.0) -> np.ndarray:
    """The latest command to `name` at each time (a step function)."""
    stamps = np.array([c[0] for c in commands if c[1] == name])
    values = np.array([c[2] for c in commands if c[1] == name])
    if len(stamps) == 0:
        return np.full(len(times), start)
    index = np.searchsorted(stamps, times, side="right") - 1
    return np.where(index >= 0, values[np.clip(index, 0, None)], start)


def settled(times: np.ndarray, view: np.ndarray, end: float) -> float:
    pick = (times > end - SETTLED) & (times <= end)
    return float(np.nanmean(view[pick])) if pick.any() else float("nan")


def analyze_servo(name: str, axis: np.ndarray, times: np.ndarray, commands, segments) -> dict:
    """Gain each way, slack, step dynamics and ramp behaviour for one servo from its view axis.

    Staircases give gain and slack; big steps and ladder steps also their dead time, rise, overshoot and
    settling; other steps (one way, the 2026-11-06 steps) only their gain. Validation parts are skipped."""
    cmd = command_series(commands, name, times)
    result = {"steps": [], "stairs": [], "big_steps": [], "ramps": []}
    stamps = [c for c in commands if c[1] == name]
    for segment in segments:
        label = segment["name"]
        if not label.startswith(name + " ") or "validation" in label:
            continue
        inside = [c for c in stamps if segment["start"] <= c[0] <= segment["end"]]
        if any(kind in label for kind in (" step ", "staircase", "big steps", "ladder", "one way")):
            # Each command is a step held until the next command (or the segment's end).
            ends = [c[0] for c in inside[1:]] + [segment["end"]]
            previous_angle = command_series(commands, name, np.array([inside[0][0] - 1e-3]))[0] if inside else 90.0
            previous_view = settled(times, axis, inside[0][0]) if inside else float("nan")
            dynamic = "big steps" in label or "ladder" in label
            for (stamp, _, angle), end in zip(inside, ends):
                view_end = settled(times, axis, end)
                record = {"from": previous_angle, "to": angle, "view_change": view_end - previous_view, "at": stamp,
                          "gain": (view_end - previous_view) / (angle - previous_angle) if angle != previous_angle else None}
                if dynamic and angle != previous_angle:
                    record.update(step_dynamics(times, axis, stamp, end, previous_view, view_end))
                key = "big_steps" if dynamic else "stairs" if "staircase" in label else "steps"
                result[key].append(record)
                previous_angle, previous_view = angle, view_end
        elif " ramp " in label:
            pick = (times >= segment["start"]) & (times <= segment["end"])
            t, v, c = times[pick], axis[pick], cmd[pick]
            if len(t) > 10:
                speed = np.gradient(v, t)
                kernel = np.ones(5) / 5
                speed = np.convolve(speed, kernel, mode="same")
                lags = np.arange(0.0, 0.6, 0.01)
                gains = [g for g in [r["gain"] for r in result["stairs"]] if g] or [0.27]
                gain = float(np.median(np.abs(gains))) * np.sign(np.median(gains))
                errors = [np.mean(np.abs(v - gain * (np.interp(t - lag, times, cmd) - 90.0) - (v[0] - gain * (c[0] - 90.0))))
                          for lag in lags]
                result["ramps"].append({"segment": label, "top_view_speed": float(np.max(np.abs(speed))),
                                        "trails_command_by": float(lags[int(np.argmin(errors))])})
    gains_up = [r["gain"] for r in result["stairs"] + result["steps"] if r["gain"] is not None and r["to"] > r["from"]]
    gains_down = [r["gain"] for r in result["stairs"] + result["steps"] if r["gain"] is not None and r["to"] < r["from"]]
    stairs = result["stairs"]
    slack = []
    for previous, current in zip(stairs, stairs[1:]):
        if None in (previous["gain"], current["gain"]):
            continue
        reversed_ = np.sign(current["to"] - current["from"]) != np.sign(previous["to"] - previous["from"])
        if reversed_:
            typical = np.median([abs(g) for g in gains_up + gains_down])
            lost = abs(current["to"] - current["from"]) - abs(current["view_change"]) / typical
            slack.append(float(lost))
    result["summary"] = {
        "gain_up_median": float(np.median(gains_up)) if gains_up else None,
        "gain_down_median": float(np.median(gains_down)) if gains_down else None,
        "slack_servo_degrees": slack,
        "dead_time_median": float(np.median([r["dead_time"] for r in result["big_steps"] if r.get("dead_time") is not None]))
        if any(r.get("dead_time") is not None for r in result["big_steps"]) else None,
    }
    return result


def step_dynamics(times, axis, start, end, before, after) -> dict:
    pick = (times >= start) & (times <= end)
    t, v = times[pick] - start, axis[pick]
    change = after - before
    if len(t) < 5 or abs(change) < 0.3:
        return {}
    progress = (v - before) / change

    def first(level):
        hit = np.flatnonzero(progress >= level)
        return float(t[hit[0]]) if len(hit) else None

    dead, ten, ninety = first(0.02), first(0.1), first(0.9)
    outside = np.flatnonzero(np.abs(progress - 1.0) > 0.05)
    return {"dead_time": dead, "rise_10_90": (ninety - ten) if None not in (ten, ninety) else None,
            "overshoot_percent": float(max(0.0, (np.nanmax(progress) - 1.0) * 100.0)),
            "settle_5_percent": float(t[outside[-1]]) if len(outside) else 0.0}


def main() -> None:
    folder = Path(sys.argv[1])
    summary = measure(folder, workers=int(sys.argv[2]) if len(sys.argv) > 2 else 14)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
