"""Replay a recorded following run in the simulator, and compare what the simulated robot did with the real one.

The run's own video says where the camera pointed (simulator/camera_rotation.py, each stretch of video
against its own first frame, the detected people masked). Each detection's pixel, through the camera's
orientation when its frame was captured, gives the person's direction; the spacing of their eyes in the
pose detector's keypoints gives their distance. Placed in the simulator's world, the person is then
seen by the simulated camera wherever its plant points it, and the production controller
(OrientationNode.step, with the run's own settings) answers as it did on the robot, or does not.

`compare` sets the simulated run beside the recorded one: where the camera pointed (measured from the
video against the plant's view), where the controller pointed the head (commands), the person's place in
the picture, and the waist's moves. Recorded logs are evidence of the closed loop, never fitted to.
"""
import asyncio
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ultraskelly.simulator.camera_rotation import Camera, RotationTracker, box_mask, orientation

EYE_SPACING = 0.063  # Metres between an adult's eyes
WINDOW_GAP = 1  # Clip frames apart that start a new stretch of video
NEAREST = 0.5  # Metres: no one is placed closer than this
HOLD = 0.15  # Seconds a detection stands for the person in the replay, without a newer one


@dataclass
class Run:
    folder: Path
    frames: dict[str, np.ndarray]  # Per video frame: index, frame_number, captured, received
    commands: list[tuple[float, str, float | None]]
    detections: list[dict]  # captured, received, frame_number, x, y (None: nobody), eyes (pixels apart or None), boxes
    configuration: dict  # run.json's


def load_run(folder: Path) -> Run:
    with open(folder / "oak_frames.csv", newline="") as file:
        rows = list(csv.DictReader(file))
    frames = {name: np.array([float(r[name]) for r in rows]) for name in ("index", "frame_number", "captured", "received")}
    with open(folder / "commands.csv", newline="") as file:
        commands = [(float(r["time"]), r["actuator"], None if r["value"] in ("", "None") else float(r["value"]))
                    for r in csv.DictReader(file)]
    with open(folder / "targets.csv", newline="") as file:
        targets = list(csv.DictReader(file))
    detections = []
    with open(folder / "poses.jsonl") as file:
        for target, line in zip(targets, file):  # One target per pose result, in order
            pose = json.loads(line)
            seen = target["x"] not in ("", "None")
            eyes, boxes = None, [p["box"] for p in pose["people"] if p.get("box")]
            if seen and pose["people"]:
                x, y = float(target["x"]), float(target["y"])
                person = min(pose["people"], key=lambda p: math.dist(p["keypoints"][0][:2], (x, y)))
                left, right = person["keypoints"][1], person["keypoints"][2]
                if min(left[2], right[2]) > 0.3:
                    eyes = math.dist(left[:2], right[:2])
            detections.append({"captured": pose["captured"], "received": pose["received"],
                               "frame_number": pose["frame_number"], "x": float(target["x"]) if seen else None,
                               "y": float(target["y"]) if seen else None, "eyes": eyes, "boxes": boxes})
    configuration = json.loads((folder / "run.json").read_text())["configuration"]
    return Run(folder, frames, commands, detections, configuration)


def stretches(indices: list[int]) -> list[list[int]]:
    """Runs of consecutive frame indices."""
    out = []
    for index in indices:
        if out and index - out[-1][-1] <= WINDOW_GAP:
            out[-1].append(index)
        else:
            out.append([index])
    return out


def _measure_stretch(job) -> list[tuple]:
    clip, positions, run_indices, camera, mask, boxes = job
    rows, tracker = [], None
    for position, index in zip(positions, run_indices):
        picture = clip.read(position)
        frame_mask = box_mask(camera.size, boxes.get(index, [])) if boxes.get(index) else None
        if mask is not None:
            frame_mask = mask if frame_mask is None else np.minimum(frame_mask, mask)
        if tracker is None:
            tracker = RotationTracker(camera, picture, frame_mask, refine=False, new_keyframe_beyond=12.0)
            rows.append((index, 0.0, 0.0, 0.0, True))
            continue
        found = tracker.measure(picture, frame_mask)
        rows.append((index, found.yaw, found.pitch, found.roll, found.ok))
    return rows


def measure_run(run: Run, clip, clip_indices: list[int], camera: Camera, self_mask: np.ndarray | None = None,
                workers: int = 14) -> dict[str, np.ndarray]:
    """The camera's turn in each clipped frame (`clip` holds the run's frames `clip_indices`, in order), each
    stretch against its own first frame, the detected people masked. Returns index, stretch, yaw, pitch,
    roll and ok per frame (degrees; yaw right, pitch down, the content's roll clockwise positive)."""
    by_number = {int(n): int(i) for i, n in zip(run.frames["index"], run.frames["frame_number"])}
    boxes = {}
    for detection in run.detections:
        index = by_number.get(int(detection["frame_number"])) if detection["frame_number"] is not None else None
        if index is not None and detection["boxes"]:
            boxes[index] = detection["boxes"]
    position_of = {index: k for k, index in enumerate(clip_indices)}
    jobs = []
    for stretch in stretches(clip_indices):
        jobs.append((clip, [position_of[i] for i in stretch], stretch, camera, self_mask,
                     {i: boxes[i] for i in stretch if i in boxes}))
    if workers <= 1:
        results = [_measure_stretch(job) for job in jobs]
    else:
        import multiprocessing

        with multiprocessing.get_context("spawn").Pool(workers) as pool:
            results = pool.map(_measure_stretch, jobs, chunksize=1)
    table = {"index": [], "stretch": [], "yaw": [], "pitch": [], "roll": [], "ok": []}
    for number, rows in enumerate(results):
        for index, yaw, pitch, roll, ok in rows:
            for name, value in zip(("index", "stretch", "yaw", "pitch", "roll", "ok"), (index, number, yaw, pitch, roll, ok)):
                table[name].append(value)
    return {name: np.array(values) for name, values in table.items()}


def camera_axes(yaw: float, pitch: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The simulator's camera axes (right, up, forward) in its world (x right, y up, z ahead) for a camera
    pose (yaw right, pitch up, degrees), as Simulation.sighting builds them."""
    y, p = math.radians(yaw), math.radians(pitch)
    right = np.array([math.cos(y), 0.0, -math.sin(y)])
    up = np.array([-math.sin(y) * math.sin(p), math.cos(p), -math.cos(y) * math.sin(p)])
    forward = np.array([math.sin(y) * math.cos(p), math.sin(p), math.cos(y) * math.cos(p)])
    return right, up, forward


def place(direction: np.ndarray, origin: np.ndarray, distance: float):
    """The simulator's Person at `distance` metres from the camera's `origin` along `direction` (world)."""
    from ultraskelly.simulator.engine import Person

    x, y, z = origin + max(distance, NEAREST) * direction
    across = math.hypot(x, z)
    return Person(yaw=math.degrees(math.atan2(x, z)), pitch=math.degrees(math.atan2(y - 2.10, across)),
                  distance=math.hypot(across, y - 2.10))


def people_track(run: Run, views: dict[str, np.ndarray], camera: Camera, start_pose: tuple[float, float],
                 stretch: int, default_distance: float = 2.0) -> list[tuple[float, object | None]]:
    """(capture time, Person or None) for each detection in one stretch of measured video: where the person was
    in the simulator's world, with the camera at `start_pose` (yaw, pitch up) as the stretch began."""
    pick = views["stretch"] == stretch
    by_number = dict(zip(run.frames["frame_number"].astype(int), run.frames["index"].astype(int)))
    measured = {int(i): (yaw, pitch, roll) for i, yaw, pitch, roll, ok in
                zip(views["index"][pick], views["yaw"][pick], views["pitch"][pick], views["roll"][pick], views["ok"][pick]) if ok}
    if not measured:
        return []
    first, last = min(measured), max(measured)
    right0, up0, forward0 = camera_axes(*start_pose)
    to_world = np.column_stack([right0, -up0, forward0])  # The stretch's first camera axes (x right, y down, z ahead)
    inverse = np.linalg.inv(camera.matrix)
    spacing = [d["eyes"] for d in run.detections if d["eyes"]]
    typical = (camera.matrix[0, 0] * EYE_SPACING / float(np.median(spacing))) if spacing else default_distance
    track = []
    for detection in run.detections:
        if not run.frames["captured"][first] <= detection["captured"] <= run.frames["captured"][last]:
            continue
        index = by_number.get(int(detection["frame_number"])) if detection["frame_number"] is not None else None
        if index is None or detection["x"] is None or index not in measured:
            track.append((detection["captured"], None))
            continue
        turn = orientation(*measured[index])  # This frame's camera in the stretch's first camera's axes
        ray = inverse @ np.array([detection["x"], detection["y"], 1.0])
        direction = to_world @ (turn @ (ray / np.linalg.norm(ray)))
        right, up, forward = to_world @ turn[:, 0], -(to_world @ turn[:, 1]), to_world @ turn[:, 2]
        origin = 0.18 * up + 0.20 * forward + np.array([0.0, 1.92, 0.0])  # As Simulation.sighting places the camera
        distance = camera.matrix[0, 0] * EYE_SPACING / detection["eyes"] if detection["eyes"] else typical
        track.append((detection["captured"], place(direction, origin, distance)))
    return track


def command_at(commands, actuator: str, time: float, default: float | None = None) -> float | None:
    value = default
    for at, name, sent in commands:
        if at > time:
            break
        if name == actuator:
            value = sent
    return value


def replay(run: Run, track: list[tuple[float, object | None]], *, settings: dict | None = None) -> dict[str, np.ndarray]:
    """Run the production controller, with the run's settings, on the simulated robot for the stretch `track`
    covers, the person where the track puts them (held between detections). Returns the simulated traces."""
    from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
    from ultraskelly.simulator.engine import Settings, Simulation

    times = np.array([t for t, _ in track])
    begin, end = float(times[0]), float(times[-1])

    def people(now: float):
        k = int(np.searchsorted(times, begin + now, side="right")) - 1
        if k < 0 or begin + now - times[k] > HOLD:
            return []  # No detection lately: nobody, as the robot saw it
        person = track[k][1]
        return [] if person is None else [person]

    from ultraskelly.core.bot.motor.head_aim import HeadAim
    from ultraskelly.core.bot.motor.roll_level import RollLevel
    from ultraskelly.core.bot.motor.waist_drive import WaistDrive

    sim = Simulation(Settings(scenario="manual", **(settings or {})), people=people)
    params = OrientationNodeParams.model_validate(run.configuration["orientation"])
    head = sim.head
    head.params = params
    for name in ("pan", "tilt", "roll"):
        angle = command_at(run.commands, name, begin, 90.0)
        setattr(head, f"{name}_angle", angle)
        axis = sim.axes[name]
        axis.servo = axis.linkage = angle
        axis.commands[0] = (-math.inf, angle)
        axis.view = axis.model.view_of(angle)
    # The node built these from its default settings when it was made: rebuild them from the run's
    head.head_aim = HeadAim(params.aim, head.pan_angle, head.tilt_angle)
    head.waist_drive = WaistDrive(params.waist)
    head.roll_level = RollLevel(params.roll, head.roll_angle)
    head.following = params.follow
    traces = {key: [] for key in ("time", "pan", "tilt", "roll", "waist_drive", "waist", "camera_yaw", "camera_pitch", "x", "y")}

    async def run_it():
        try:
            for _ in range(int((end - begin) / sim.dt)):
                await sim.step()
                yaw, pitch = sim.camera_pose()
                for key, value in (("time", begin + sim.now()), ("pan", sim.head.pan_angle), ("tilt", sim.head.tilt_angle),
                                   ("roll", sim.head.roll_angle), ("waist_drive", sim.waist_drive()), ("waist", sim.waist),
                                   ("camera_yaw", yaw), ("camera_pitch", pitch), ("x", sim.last_camera["x"]),
                                   ("y", sim.last_camera["y"])):
                    traces[key].append(np.nan if value is None else value)
        finally:
            sim.close()

    asyncio.run(run_it())
    return {key: np.array(values, dtype=float) for key, values in traces.items()}


def start_pose(run: Run, time: float) -> tuple[float, float]:
    """The simulated camera's pose (yaw right, pitch up, degrees) with the head at rest where the run
    commanded it at `time`, the body turned to 0, as Simulation.camera_pose gives it."""
    from ultraskelly.simulator import plant

    pan, tilt = command_at(run.commands, "pan", time, 90.0), command_at(run.commands, "tilt", time, 90.0)
    return plant.PAN.gain * (pan - 90.0), -plant.TILT.gain * (tilt - 90.0)


def measured_pose(views: dict[str, np.ndarray], stretch: int, pose: tuple[float, float]) -> dict[int, tuple[float, float]]:
    """Per frame index of the stretch, where the camera pointed in the simulator's world (yaw right, pitch up)."""
    right0, up0, forward0 = camera_axes(*pose)
    to_world = np.column_stack([right0, -up0, forward0])
    out = {}
    pick = (views["stretch"] == stretch) & views["ok"].astype(bool)
    for index, yaw, pitch, roll in zip(views["index"][pick], views["yaw"][pick], views["pitch"][pick], views["roll"][pick]):
        forward = to_world @ orientation(yaw, pitch, roll)[:, 2]
        out[int(index)] = (math.degrees(math.atan2(forward[0], forward[2])), math.degrees(math.asin(np.clip(forward[1], -1, 1))))
    return out


def moves(times: np.ndarray, drive: np.ndarray) -> list[float]:
    """When each waist move began: the drive leaving zero."""
    active = np.nan_to_num(np.abs(drive)) > 0
    return [float(t) for t, now, before in zip(times[1:], active[1:], active[:-1]) if now and not before]


def compare(run: Run, views: dict[str, np.ndarray], stretch: int, pose: tuple[float, float],
            simulated: dict[str, np.ndarray]) -> dict:
    """The simulated stretch beside the recorded one."""
    times = simulated["time"]
    measured = measured_pose(views, stretch, pose)
    by_index = dict(zip(run.frames["index"].astype(int), run.frames["captured"]))
    seen_times = np.array([by_index[i] for i in sorted(measured)])
    seen_yaw = np.array([measured[i][0] for i in sorted(measured)])
    inside = (times >= seen_times[0]) & (times <= seen_times[-1])
    camera_yaw = np.interp(times[inside], seen_times, seen_yaw)
    sent = [(t, v) for t, name, v in run.commands if name == "pan" and v is not None]
    stamps, values = np.array([t for t, _ in sent]), np.array([v for _, v in sent])
    recorded_pan = values[np.clip(np.searchsorted(stamps, times, side="right") - 1, 0, None)]
    drive = [(t, v) for t, name, v in run.commands if name == "waist"]
    drive_times = np.array([t for t, _ in drive])
    drive_values = np.array([np.nan if v is None else v for _, v in drive])
    recorded_drive = drive_values[np.clip(np.searchsorted(drive_times, times, side="right") - 1, 0, None)]
    span = (times[0], times[-1])
    recorded_x = np.array([d["x"] for d in run.detections if d["x"] is not None and span[0] <= d["captured"] <= span[1]])
    simulated_x = simulated["x"][np.isfinite(simulated["x"])]

    def spread(x):
        off = np.abs(np.asarray(x) - 320.0)
        return {"median_px": float(np.median(off)) if len(off) else None, "p90_px": float(np.percentile(off, 90)) if len(off) else None}

    return {
        "seconds": float(times[-1] - times[0]),
        "camera_yaw_rms_degrees": float(np.sqrt(np.mean((simulated["camera_yaw"][inside] - camera_yaw) ** 2))),
        "camera_yaw_travel_degrees": {"recorded": float(np.sum(np.abs(np.diff(camera_yaw)))),
                                      "simulated": float(np.sum(np.abs(np.diff(simulated["camera_yaw"][inside]))))},
        "pan_command_rms_degrees": float(np.sqrt(np.mean((simulated["pan"] - recorded_pan) ** 2))),
        "waist_moves": {"recorded": len(moves(times, recorded_drive)), "simulated": len(moves(times, simulated["waist_drive"]))},
        "person_off_centre": {"recorded": spread(recorded_x), "simulated": spread(simulated_x)},
    }


def _series(times: np.ndarray, commands, actuator: str, default: float | None) -> np.ndarray:
    sent = [(t, v) for t, name, v in commands if name == actuator]
    if not sent:
        return np.full(len(times), np.nan if default is None else default)
    stamps = np.array([t for t, _ in sent])
    values = np.array([np.nan if v is None else v for _, v in sent])
    index = np.searchsorted(stamps, times, side="right") - 1
    return np.where(index >= 0, values[np.clip(index, 0, None)], np.nan if default is None else default)


def idle_spans(run: Run, begin: float, end: float, quiet: float = 1.5, shortest: float = 4.0) -> list[tuple[float, float]]:
    """Spans of [begin, end] in which the waist was not driven, and had not been for `quiet` seconds."""
    driven = sorted(t for t, name, v in run.commands if name == "waist" and v not in (None, 0.0) and begin - quiet <= t <= end)
    stops = [t for t, name, v in run.commands if name == "waist" and v in (None, 0.0)]
    busy = []
    for t in driven:
        after = [s for s in stops if s > t]
        busy.append((t, (after[0] if after else end) + quiet))
    spans, at = [], begin
    for a, b in sorted(busy):
        if a > at and a - at >= shortest:
            spans.append((at, min(a, end)))
        at = max(at, b)
    if end - at >= shortest:
        spans.append((at, end))
    return spans


def open_loop_head(run: Run, views: dict[str, np.ndarray], stretch: int, model, *, align: float = 0.3) -> dict:
    """The pan plant driven by the run's recorded pan commands against the camera's measured yaw, in each span
    of the stretch where the waist was idle: each span free-run from the command at its start, its view
    aligned to the measurement over the span's first `align` seconds. Returns the RMS per span and over all."""
    from ultraskelly.simulator.plant import Axis

    pick = (views["stretch"] == stretch) & views["ok"].astype(bool)
    captured = run.frames["captured"][views["index"][pick].astype(int)]
    yaw = views["yaw"][pick]
    spans, errors = [], []
    for a, b in idle_spans(run, captured[0], captured[-1]):
        inside = (captured >= a) & (captured <= b)
        if inside.sum() < 30:
            continue
        times, measured = captured[inside], yaw[inside]
        commands = [(t, v) for t, name, v in run.commands if name == "pan" and v is not None and t <= b]
        start = [v for t, v in commands if t <= a][-1] if any(t <= a for t, _ in commands) else 90.0
        axis = Axis(model, start=start)
        for t, v in commands:
            if t > a:
                axis.command(t, v)
        grid = np.arange(a, b + 0.002, 0.002)
        predicted = np.interp(times, grid, [axis.advance(now, 0.002) for now in grid])
        head = times <= a + align
        predicted += np.median(measured[head]) - np.median(predicted[head])
        error = predicted - measured
        spans.append({"seconds": float(b - a), "rms_degrees": float(np.sqrt(np.mean(error ** 2))),
                      "yaw_range": float(np.ptp(measured))})
        errors.append(error)
    joined = np.concatenate(errors) if errors else np.array([np.nan])
    return {"spans": spans, "rms_degrees": float(np.sqrt(np.mean(joined ** 2))), "frames": int(len(joined))}


def open_loop_waist(run: Run, views: dict[str, np.ndarray], stretch: int, waist_model, pan_model, *,
                    settle: float = 1.5) -> list[dict]:
    """Each recorded waist move in the stretch: the turn the camera measured over it (to `settle` seconds after
    the drive stopped) less what the pan plant says the head turned, beside the waist model's turn from the
    recorded drive."""
    from ultraskelly.simulator.plant import Axis, Waist

    pick = (views["stretch"] == stretch) & views["ok"].astype(bool)
    captured = run.frames["captured"][views["index"][pick].astype(int)]
    yaw = views["yaw"][pick]
    drive = [(t, v) for t, name, v in run.commands if name == "waist"]
    moves_out = []
    k = 0
    while k < len(drive):
        t, v = drive[k]
        if v in (None, 0.0) or not captured[0] <= t <= captured[-1]:
            k += 1
            continue
        start = t
        while k < len(drive) and drive[k][1] not in (None, 0.0):
            k += 1
        stop = drive[k][0] if k < len(drive) else captured[-1]
        end = stop + settle
        if end > captured[-1]:
            break
        seen = np.interp([start - 0.05, end], captured, yaw)
        commands = [(at, value) for at, name, value in run.commands if name == "pan" and value is not None and at <= end]
        pan_start = [value for at, value in commands if at <= start][-1] if any(at <= start for at, _ in commands) else 90.0
        axis = Axis(pan_model, start=pan_start)
        for at, value in commands:
            if at > start:
                axis.command(at, value)
        grid = np.arange(start, end + 0.002, 0.002)
        head = [axis.advance(now, 0.002) for now in grid]
        waist = Waist(waist_model)
        sent = [(at, value) for at, value in drive if start - 1e-9 <= at <= end]
        position, angle = 0, 0.0
        for now in np.arange(start, end, 0.01):
            while position + 1 < len(sent) and sent[position + 1][0] <= now:
                position += 1
            angle = waist.advance(sent[position][1], 0.01)
        moves_out.append({"start": float(start), "seconds_driven": float(stop - start),
                          "peak_drive": float(max(abs(value) for _, value in sent if value is not None)),
                          "measured_waist_turn": float(seen[1] - seen[0] - (head[-1] - head[0])),
                          "model_waist_turn": float(angle)})
    return moves_out
