"""Turn a record_motor_response run into each motor's measured response: the numbers a simulated head needs.

From frames.npz it chains each frame's motion against the one before (ORB features and a similarity
fit, phase correlation where features are few) into the camera's view over time: yaw (degrees, the
view turning right is positive), pitch (down is positive) and roll (the picture's content turning
clockwise is positive). Lined up with commands.csv and segments.json it measures, per servo:

- gain each way: degrees of view per servo degree from the settled ends of steps and stairs;
- slack: servo degrees lost on the first stair after a reversal;
- dead time, 10-90 % rise time, overshoot and 5 % settling time of the big steps;
- the view's top speed and how far it trails the command on ramps;

and for the waist, each pulse's turn and top speed. Writes response.json and response.png beside the run.

    python -m scripts.analyze_motor_response ~/ultraskelly-recordings/motor-response-20261106-201500
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np

PX_PER_DEGREE = 8.1 / 2  # The OAK's 8.1 px per degree near the middle, on the recording's half-size frames
SETTLED = 0.4  # Seconds at the end of a hold whose views are averaged as the settled view


def frame_motions(cv2, frames: np.ndarray) -> np.ndarray:
    """(N, 3): each frame's dx, dy (pixels, the content's motion) and rotation (degrees, clockwise) from the one before."""
    orb = cv2.ORB_create(nfeatures=600)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    window = cv2.createHanningWindow((frames.shape[2], frames.shape[1]), cv2.CV_32F)
    motions = np.zeros((len(frames), 3))
    previous = orb.detectAndCompute(frames[0], None)
    for i in range(1, len(frames)):
        current = orb.detectAndCompute(frames[i], None)
        found = None
        if previous[1] is not None and current[1] is not None:
            matches = matcher.match(previous[1], current[1])
            if len(matches) >= 12:
                a = np.float32([previous[0][m.queryIdx].pt for m in matches])
                b = np.float32([current[0][m.trainIdx].pt for m in matches])
                fit, inliers = cv2.estimateAffinePartial2D(a, b, method=cv2.RANSAC, ransacReprojThreshold=2.0)
                if fit is not None and inliers is not None and inliers.sum() >= 10:
                    angle = np.degrees(np.arctan2(fit[1, 0], fit[0, 0]))
                    centre = np.array([frames.shape[2] / 2, frames.shape[1] / 2])
                    moved = fit[:, :2] @ centre + fit[:, 2] - centre  # How the picture's middle moved
                    found = (moved[0], moved[1], angle)
        if found is None:
            (dx, dy), _ = cv2.phaseCorrelate(frames[i - 1].astype(np.float32), frames[i].astype(np.float32), window)
            found = (dx, dy, 0.0)
        motions[i] = found
        previous = current
    return motions


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
    return float(np.mean(view[pick])) if pick.any() else float("nan")


def analyze_servo(name: str, axis: np.ndarray, times: np.ndarray, commands, segments) -> dict:
    """Gain each way, slack, step dynamics and ramp behaviour for one servo from its view axis."""
    cmd = command_series(commands, name, times)
    result = {"steps": [], "stairs": [], "big_steps": [], "ramps": []}
    stamps = [c for c in commands if c[1] == name]
    for segment in segments:
        if not segment["name"].startswith(name + " "):
            continue
        inside = [c for c in stamps if segment["start"] <= c[0] <= segment["end"]]
        if " step " in segment["name"] or "staircase" in segment["name"] or "big steps" in segment["name"]:
            # Each command is a step held until the next command (or the segment's end).
            ends = [c[0] for c in inside[1:]] + [segment["end"]]
            previous_angle = command_series(commands, name, np.array([inside[0][0] - 1e-3]))[0] if inside else 90.0
            previous_view = settled(times, axis, inside[0][0]) if inside else float("nan")
            for (stamp, _, angle), end in zip(inside, ends):
                view_end = settled(times, axis, end)
                record = {"from": previous_angle, "to": angle, "view_change": view_end - previous_view,
                          "gain": (view_end - previous_view) / (angle - previous_angle) if angle != previous_angle else None}
                if "big steps" in segment["name"] and angle != previous_angle:
                    record.update(step_dynamics(times, axis, stamp, end, previous_view, view_end))
                key = "big_steps" if "big steps" in segment["name"] else "stairs" if "staircase" in segment["name"] else "steps"
                result[key].append(record)
                previous_angle, previous_view = angle, view_end
        elif " ramp " in segment["name"]:
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
                result["ramps"].append({"segment": segment["name"], "top_view_speed": float(np.max(np.abs(speed))),
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
        "dead_time_median": float(np.median([r["dead_time"] for r in result["big_steps"] if "dead_time" in r]))
        if any("dead_time" in r for r in result["big_steps"]) else None,
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
            "overshoot_percent": float(max(0.0, (np.max(progress) - 1.0) * 100.0)),
            "settle_5_percent": float(t[outside[-1]]) if len(outside) else 0.0}


def main() -> None:
    import cv2

    folder = Path(sys.argv[1])
    data = np.load(folder / "frames.npz")
    frames, times = data["frames"], data["captured"]
    commands = [(float(r["time"]), r["actuator"], float(r["value"])) for r in csv.DictReader(open(folder / "commands.csv"))]
    segments = json.loads((folder / "segments.json").read_text())
    motions = frame_motions(cv2, frames)
    yaw = -np.cumsum(motions[:, 0]) / PX_PER_DEGREE  # Content moving left: the view turned right
    pitch = -np.cumsum(motions[:, 1]) / PX_PER_DEGREE  # Content moving up: the view turned down
    roll = np.cumsum(motions[:, 2])  # The content's clockwise turn
    np.savez_compressed(folder / "views.npz", times=times, yaw=yaw, pitch=pitch, roll=roll, motions=motions)
    report = {"frames": int(len(frames)), "seconds": float(times[-1] - times[0]),
              "pan": analyze_servo("pan", yaw, times, commands, segments),
              "tilt": analyze_servo("tilt", pitch, times, commands, segments),
              "roll": analyze_servo("roll", roll, times, commands, segments)}
    waist = []
    for segment in segments:
        if segment["name"].startswith("waist"):
            pick = (times >= segment["start"]) & (times <= segment["end"])
            if pick.sum() > 5:
                t, v = times[pick], yaw[pick]
                speed = np.convolve(np.gradient(v, t), np.ones(5) / 5, mode="same")
                waist.append({"segment": segment["name"], "turn_degrees": float(v[-1] - v[0]),
                              "top_speed": float(np.max(np.abs(speed)))})
    report["waist"] = waist
    (folder / "response.json").write_text(json.dumps(report, indent=1, default=float))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
        t0 = times[0]
        for ax, name, view in zip(axes[:3], ("pan", "tilt", "roll"), (yaw, pitch, roll)):
            command = command_series(commands, name, times)
            ax.plot(times - t0, command - 90.0, label=f"{name} command - 90 (servo deg)", color="tab:gray")
            ax.plot(times - t0, view, label=f"measured view {name} (deg)", color="tab:blue")
            ax.legend(loc="upper right"); ax.grid(alpha=0.3)
        axes[3].plot(times - t0, command_series(commands, "waist", times, 0.0), label="waist drive", color="tab:gray")
        axes[3].plot(times - t0, yaw / 20.0, label="view yaw / 20 (deg)", color="tab:blue")
        axes[3].legend(loc="upper right"); axes[3].grid(alpha=0.3); axes[3].set_xlabel("seconds")
        for segment in segments:
            for ax in axes:
                ax.axvline(segment["start"] - t0, color="tab:orange", alpha=0.15)
        figure.tight_layout(); figure.savefig(folder / "response.png", dpi=90)
    except ImportError:
        pass
    for name in ("pan", "tilt", "roll"):
        print(name, json.dumps(report[name]["summary"]), "| ramps:",
              [(r["segment"].split()[-2], round(r["top_view_speed"], 1), r["trails_command_by"]) for r in report[name]["ramps"]])
        for r in report[name]["big_steps"]:
            print("   big step", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    print("waist", [(w["segment"], round(w["turn_degrees"], 1), round(w["top_speed"], 1)) for w in waist])


if __name__ == "__main__":
    main()
