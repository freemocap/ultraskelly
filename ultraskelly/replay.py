"""Hardware-free target replay through the production controller coroutines.

This observes commanded state; it is not a physical model of the robot. With `--serve
--people N`, a synthetic camera shows N people, and the faces the OAK's chip would find on
them, to the dashboard instead of scripted targets.
"""
import argparse
import asyncio
import itertools
import json
import time
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.motor.mouth_node import MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder, create_simulated_motor_nodes
from ultraskelly.core.bot.sensory.pose_common import CocoKeypoint, locate_target
from ultraskelly.core.pubsub.bot_topics import (
    FaceObservation,
    FacesMessage,
    FacesTopic,
    FrameMessage,
    FrameTopic,
    PoseDataMessage,
    PoseDataTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

REPLAY_INTERVAL = 0.05
SERVE_INTERVAL = 0.1  # Slower, so the dashboard's motion is easy to follow

SYNTHETIC_CAMERA = "synthetic"
SYNTHETIC_JPEG = Path(__file__).with_name("synthetic_camera.jpg")  # 640x480 background, made once with OpenCV
SYNTHETIC_FPS = 30.0
# One person standing face on: COCO-17 keypoint offsets in pixels from the middle of the hips.
PERSON = np.array([
    (0, -150), (8, -160), (-8, -160), (18, -154), (-18, -154),  # Nose, eyes, ears
    (40, -110), (-40, -110), (52, -55), (-52, -55), (56, -5), (-56, -5),  # Shoulders, elbows, wrists
    (24, 0), (-24, 0), (26, 70), (-26, 70), (28, 140), (-28, 140),  # Hips, knees, ankles
], np.float32)


def create_controllers(recorder: CommandRecorder | None = None):
    """Inject memory-only actuators without calling either hardware factory."""
    bus = PubSubTopicManager.create()
    orientation, mouth = create_simulated_motor_nodes(
        pubsub=bus,
        orientation_params=OrientationNodeParams(),
        mouth_params=MouthNodeParams(),
        recorder=recorder or CommandRecorder(),
    )
    return bus, orientation, mouth


def scripted_target(step: int) -> TargetLocationMessage:
    """Center, left, right, and missing-target phases of ten steps each, repeating."""
    x = (320, 0, 640, None)[(step // 10) % 4]
    return TargetLocationMessage(x=x, y=240 if x is not None else None)


async def replay(steps: int, interval: float):
    bus, orientation, mouth = create_controllers()
    tasks = [asyncio.create_task(node.run()) for node in (orientation, mouth)]
    try:
        for step in range(steps):
            target = scripted_target(step)
            await bus.publish(TargetLocationTopic, target)
            await asyncio.sleep(interval)
            for task in tasks:
                if task.done():
                    task.result()
                    raise RuntimeError("Controller exited before replay completed")
            yield {
                "step": step, "target_x": target.x,
                "pan_degrees": orientation.pan_angle, "tilt_degrees": orientation.tilt_angle,
                "waist_throttle": orientation.waist_motor.throttle,
                "mouth_throttle": mouth.mouth_motor.throttle,
            }
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        bus.close()


def synthetic_people(count: int, seconds: float, order: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """`count` people side by side in a 640x480 image, each swaying sideways, with their rows in
    `order`. The middle person scores highest, so the head follows them wherever their row is.

    Returns keypoints (count, 17, 3) as [x, y, visibility] and scores (count,).
    """
    keypoints = np.empty((count, 17, 3), np.float32)
    scores = np.empty(count, np.float32)
    for person in range(count):
        x = (person + 1) * 640 / (count + 1) + 40 * np.sin(2 * np.pi * seconds / 6 + person)
        keypoints[person, :, :2] = PERSON + (x, 300)
        keypoints[person, :, 2] = 0.9
        scores[person] = 0.9 if person == count // 2 else 0.6
    return keypoints[order], scores[order]


MESH = np.stack(np.meshgrid(np.linspace(-0.4, 0.4, 26), np.linspace(-0.45, 0.45, 18)), axis=-1).reshape(-1, 2)  # 468


def synthetic_faces(keypoints: np.ndarray, scores: np.ndarray, number: int, seconds: float) -> FacesMessage:
    """The faces the OAK would find on the synthetic people, best score first. As in the bot, head
    pose and gaze are measured in every second frame and the largest face's landmarks too."""
    faces = []
    for row in np.argsort(-scores):
        person = keypoints[row]
        nose, left_eye, right_eye = person[CocoKeypoint.NOSE, :2], person[CocoKeypoint.LEFT_EYE, :2], person[CocoKeypoint.RIGHT_EYE, :2]
        width = 2.5 * float(np.linalg.norm(left_eye - right_eye))
        box = (float(nose[0] - width / 2), float(nose[1] - 0.85 * width), float(nose[0] + width / 2), float(nose[1] + 0.45 * width))
        landmarks = np.array([right_eye, left_eye, nose, nose + (-6, 12), nose + (6, 12)], np.float32)
        measured = number % 2 == 0
        yaw, pitch = 25 * np.sin(2 * np.pi * seconds / 6 + row), 8 * np.sin(2 * np.pi * seconds / 4)
        gaze = np.array([np.sin(np.radians(yaw)), np.sin(np.radians(pitch)), np.cos(np.radians(yaw))])
        centre = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        mesh = np.column_stack([centre + MESH * (width, 1.3 * width), np.zeros(len(MESH))]) if measured and not faces else None
        faces.append(FaceObservation(
            box=box, score=float(scores[row]), landmarks=landmarks,
            head_pose=(float(yaw), float(pitch), 0.0) if measured else None,
            gaze=tuple(float(value) for value in gaze / np.linalg.norm(gaze)) if measured else None,
            mesh=None if mesh is None else mesh.astype(np.float32),
        ))
    return FacesMessage(faces=faces, image_size=(640, 480), frame_number=number)


async def synthetic_camera(bus: PubSubTopicManager, people: int, faces: dict[int, FacesMessage] | None = None) -> None:
    """Publish frames and the people found in them 30 times a second, as the OAK and pose nodes do,
    with the people's rows in a new order each frame, as a detector's can be. With `faces`, also
    their faces, kept there by frame number as the OAK node keeps them."""
    frame = np.zeros((480, 640, 3), np.uint8)  # Only its size travels; the stream sends SYNTHETIC_JPEG
    shuffle = np.random.default_rng(0)
    start = time.monotonic()
    for number in itertools.count():
        now = time.monotonic()
        camera_frame = FrameMessage(frame=frame, frame_number=number, timestamp=now)
        keypoints, scores = synthetic_people(people, now - start, shuffle.permutation(people))
        if faces is not None:
            found = synthetic_faces(keypoints, scores, number, now - start)
            faces[number] = found
            while len(faces) > 32:
                del faces[min(faces)]
            await bus.publish(FacesTopic, found)
        row, (x, y, angle) = locate_target(
            keypoints=keypoints, scores=scores, target_keypoint=CocoKeypoint.NOSE, keypoint_threshold=0.5
        )
        target = TargetLocationMessage(x=x, y=y, angle=angle)
        await bus.publish(FrameTopic, camera_frame)
        await bus.publish(TargetLocationTopic, target)
        await bus.publish(PoseDataTopic, PoseDataMessage(
            keypoints=keypoints, scores=scores, frame=camera_frame, target=target, target_person=row))
        await asyncio.sleep(max(0.0, start + (number + 1) / SYNTHETIC_FPS - time.monotonic()))


async def serve(port: int, interval: float, people: int = 0, host: str = "127.0.0.1") -> None:
    """Loop the scripted targets through the controllers and serve the dashboard on `host`.
    With `people`, a synthetic camera's people drive the controllers instead."""
    from ultraskelly.core.bot.motor.motor_test import MotorTester, MotorTestLimits
    from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams, TrackingOverlay

    recorder = CommandRecorder()
    bus, orientation, mouth = create_controllers(recorder)
    tasks: dict[str, asyncio.Task] = {}

    def node_status() -> list[dict]:
        return [
            {"name": name, "state": "running" if not task.done() else "stopped"}
            for name, task in tasks.items()
        ]

    camera = {}
    faces: dict[int, FacesMessage] = {}
    if people:
        jpeg = SYNTHETIC_JPEG.read_bytes()
        camera = dict(cameras={SYNTHETIC_CAMERA: FrameTopic}, overlays={SYNTHETIC_CAMERA: TrackingOverlay()},
                      pose_camera=SYNTHETIC_CAMERA, encode_jpeg=lambda frame, quality: jpeg,
                      faces_camera=SYNTHETIC_CAMERA, faces_for=faces.get)
    server = RuntimeServerNode.create(
        pubsub=bus,
        params=RuntimeServerParams(host=host, port=port),
        recorder=recorder,
        motors_simulated=True,
        node_status=node_status,
        motor_tests=MotorTester(orientation=orientation, mouth=mouth, limits=MotorTestLimits()),
        **(camera or dict(cameras={})),
    )
    for node in (orientation, mouth, server):
        tasks[type(node).__name__] = asyncio.create_task(node.run())
    if people:
        tasks["SyntheticCamera"] = asyncio.create_task(synthetic_camera(bus, people, faces))
    source = f"a synthetic camera with {people} people" if people else "scripted targets"
    print(f"Replay dashboard: http://{host}:{port} ({source}, simulated motor tests, no hardware). "
          "Ctrl+C to stop.", flush=True)
    try:
        step = 0
        while True:
            if not people:
                await bus.publish(TargetLocationTopic, scripted_target(step))
            step += 1
            await asyncio.sleep(interval)
            for task in tasks.values():
                if task.done():
                    task.result()
                    raise RuntimeError("A node exited during the replay")
    finally:
        for task in tasks.values():
            task.cancel()
        await asyncio.gather(*tasks.values(), return_exceptions=True)
        bus.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--interval", type=float, default=None,
                        help=f"Seconds per target (default {REPLAY_INTERVAL}, or {SERVE_INTERVAL} with --serve)")
    parser.add_argument("--serve", action="store_true",
                        help="Loop the targets and serve the live dashboard instead of printing JSON lines")
    parser.add_argument("--port", type=int, default=8081, help="Dashboard port with --serve")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Interface to serve on with --serve; 0.0.0.0 lets other computers connect")
    parser.add_argument("--people", type=int, default=0,
                        help="With --serve: a synthetic camera shows this many people (1-8), not scripted targets")
    args = parser.parse_args()
    interval = args.interval if args.interval is not None else (SERVE_INTERVAL if args.serve else REPLAY_INTERVAL)
    if args.steps <= 0 or interval <= 0:
        parser.error("steps and interval must be positive")
    if not 0 <= args.people <= 8 or (args.people and not args.serve):
        parser.error("--people takes 1 to 8, together with --serve")

    if args.serve:
        try:
            asyncio.run(serve(args.port, interval, people=args.people, host=args.host))
        except KeyboardInterrupt:
            pass
        return

    async def run():
        async for state in replay(args.steps, interval):
            print(json.dumps(state))

    asyncio.run(run())


if __name__ == "__main__":
    main()
