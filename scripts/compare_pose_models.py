"""Compare the OAK's own YOLOv8n pose with the Hailo-8's YOLOv8s pose on the same OAK frames.

Uses the OAK and the Hailo-8, so stop the bot and run it under the robot lock, with people in
front of the nose camera:

    flock -w 60 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.compare_pose_models --seconds 60 --save ~/pose-compare

The OAK runs YOLOv8n pose on its chip (Luxonis Model Zoo, 2 threads x 6 SHAVEs, letterboxed to
512x288) and sends each frame's RGB image; the Pi runs the bot's YOLOv8s pose on the Hailo-8
on that image, so both look at the same frames, paired by frame number. People are matched by
box overlap. Prints a JSON summary: frames with people per model, people found by one model
and not the other, and for matched people the keypoint distances in pixels. With `--save`,
writes some frames with both skeletons drawn (OAK orange, Hailo green).
"""
import argparse
import contextlib
import json
import time
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.oak_models import MODEL_CACHE, PLATFORM, without_telemetry
from ultraskelly.core.bot.sensory.pose_common import SKELETON_CONNECTIONS
from ultraskelly.core.bot.sensory.yolov8_pose import Letterbox, PoseDetections, decode_yolov8_pose

OAK_MODEL = "luxonis/yolov8-nano-pose-estimation:coco-512x288"
OAK_INPUT = (512, 288)
HEF = "/usr/share/hailo-models/yolov8s_pose_h8.hef"
SCORE, KEYPOINT_SCORE, MATCH_IOU = 0.5, 0.4, 0.5


def oak_people(detections, letterbox: Letterbox) -> PoseDetections:
    """The OAK's ImgDetections (normalized to its letterboxed input) as frame-pixel people."""
    width, height = OAK_INPUT
    boxes, keypoints, scores = [], [], []
    for detection in detections.detections:
        if detection.confidence < SCORE:
            continue
        points = np.array([[point.imageCoordinates.x, point.imageCoordinates.y, point.confidence]
                           for point in detection.getKeypoints()], np.float32)
        points[:, 0] = (points[:, 0] * width - letterbox.pad_x) / letterbox.scale
        points[:, 1] = (points[:, 1] * height - letterbox.pad_y) / letterbox.scale
        box = np.array([detection.xmin * width, detection.ymin * height, detection.xmax * width, detection.ymax * height])
        boxes.append((box - np.tile([letterbox.pad_x, letterbox.pad_y], 2)) / letterbox.scale)
        keypoints.append(points)
        scores.append(detection.confidence)
    if not scores:
        return PoseDetections(np.zeros((0, 17, 3), np.float32), np.zeros(0, np.float32), np.zeros((0, 4), np.float32))
    return PoseDetections(np.stack(keypoints), np.array(scores, np.float32), np.stack(boxes).astype(np.float32))


def iou(a: np.ndarray, b: np.ndarray) -> float:
    width = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    height = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - width * height
    return float(width * height / union) if union > 0 else 0.0


def match(oak: PoseDetections, hailo: PoseDetections) -> list[tuple[int, int]]:
    """Pairs (oak row, hailo row) by best box overlap above MATCH_IOU, greedily."""
    pairs, used = [], set()
    candidates = sorted(((iou(oak.boxes[i], hailo.boxes[j]), i, j) for i in range(oak.count) for j in range(hailo.count)), reverse=True)
    for overlap, i, j in candidates:
        if overlap >= MATCH_IOU and i not in {p[0] for p in pairs} and j not in used:
            pairs.append((i, j))
            used.add(j)
    return pairs


def draw(frame: np.ndarray, people: PoseDetections, colour: tuple[int, int, int]) -> None:
    import cv2

    for person in people.keypoints:
        for start, end in SKELETON_CONNECTIONS:
            if person[start, 2] >= KEYPOINT_SCORE and person[end, 2] >= KEYPOINT_SCORE:
                cv2.line(frame, tuple(int(v) for v in person[start, :2]), tuple(int(v) for v in person[end, :2]), colour, 2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare the OAK's YOLOv8n pose with the Hailo's YOLOv8s on the same frames.")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--save", type=Path, default=None, help="Folder for frames with both skeletons drawn")
    args = parser.parse_args()
    without_telemetry()
    import cv2
    import depthai as dai
    import hailo_platform as hp

    from ultraskelly.core.bot.sensory.hailo_device import shared_vdevice

    # 640x480 letterboxed into the OAK model's 512x288: scaled to 384x288, 64 pixels either side.
    letterbox_oak = Letterbox(scale=0.6, pad_x=64, pad_y=0, resized_width=384, resized_height=288)
    totals = {"frames": 0, "oak_frames_with_people": 0, "hailo_frames_with_people": 0, "oak_people": 0, "hailo_people": 0,
              "matched": 0, "hailo_only": 0, "oak_only": 0}
    distances, nose_distances = [], []
    with contextlib.ExitStack() as stack:
        device = stack.enter_context(shared_vdevice())
        model = device.create_infer_model(HEF)
        model.set_batch_size(1)
        model.input().set_format_type(hp.FormatType.UINT8)
        for output in model.outputs:
            output.set_format_type(hp.FormatType.UINT8)
            output.set_format_order(hp.FormatOrder.NHWC)
        configured = stack.enter_context(model.configure())
        image = np.full(model.input().shape, 114, np.uint8)
        outputs = {name: np.empty(model.output(name).shape, np.uint8) for name in model.output_names}
        quantization = [(model.output(name).quant_infos[0].qp_scale, model.output(name).quant_infos[0].qp_zp) for name in outputs]
        bindings = configured.create_bindings()
        bindings.input().set_buffer(image)
        for name, buffer in outputs.items():
            bindings.output(name).set_buffer(buffer)
        letterbox_hailo = Letterbox.fit(width=640, height=480, size=image.shape[0])

        pipeline = stack.enter_context(dai.Pipeline())
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        camera.setImageOrientation(dai.CameraImageOrientation.ROTATE_180_DEG)
        rgb = camera.requestOutput((640, 480), dai.ImgFrame.Type.RGB888i, fps=30).createOutputQueue(maxSize=30, blocking=False)
        network = pipeline.create(dai.node.DetectionNetwork)
        archive = dai.NNArchive(str(dai.getModelFromZoo(dai.NNModelDescription(OAK_MODEL, platform=PLATFORM), useCached=True,
                                                        cacheDirectory=str(MODEL_CACHE))))
        network.setNNArchive(archive, 6)
        network.setNumInferenceThreads(2)
        camera.requestOutput(OAK_INPUT, dai.ImgFrame.Type.BGR888p, dai.ImgResizeMode.LETTERBOX, fps=30).link(network.input)
        poses = network.out.createOutputQueue(maxSize=30, blocking=False)
        pipeline.start()
        frames, saved, end = {}, 0, time.monotonic() + args.seconds
        if args.save:
            args.save.mkdir(parents=True, exist_ok=True)
        while time.monotonic() < end:
            while (frame := rgb.tryGet()) is not None:
                frames[frame.getSequenceNum()] = frame.getFrame()
                for old in [number for number in frames if number < frame.getSequenceNum() - 30]:
                    del frames[old]
            detections = poses.tryGet()
            if detections is None or detections.getSequenceNum() not in frames:
                time.sleep(0.002)
                continue
            pixels = frames.pop(detections.getSequenceNum())
            image[letterbox_hailo.pad_y:letterbox_hailo.pad_y + 480, letterbox_hailo.pad_x:letterbox_hailo.pad_x + 640] = pixels
            configured.run([bindings], 1000)
            hailo = decode_yolov8_pose(list(outputs.values()), input_size=image.shape[0], letterbox=letterbox_hailo,
                                       score_threshold=SCORE, iou_threshold=0.65, max_people=10, quantization=quantization)
            oak = oak_people(detections, letterbox_oak)
            pairs = match(oak, hailo)
            totals["frames"] += 1
            totals["oak_frames_with_people"] += oak.count > 0
            totals["hailo_frames_with_people"] += hailo.count > 0
            totals["oak_people"] += oak.count
            totals["hailo_people"] += hailo.count
            totals["matched"] += len(pairs)
            totals["hailo_only"] += hailo.count - len(pairs)
            totals["oak_only"] += oak.count - len(pairs)
            for i, j in pairs:
                both = (oak.keypoints[i, :, 2] >= KEYPOINT_SCORE) & (hailo.keypoints[j, :, 2] >= KEYPOINT_SCORE)
                distances.extend(np.linalg.norm(oak.keypoints[i, both, :2] - hailo.keypoints[j, both, :2], axis=1).tolist())
                if both[0]:
                    nose_distances.append(float(np.linalg.norm(oak.keypoints[i, 0, :2] - hailo.keypoints[j, 0, :2])))
            if args.save and (oak.count or hailo.count) and saved < 20 and totals["frames"] % 15 == 0:
                drawn = cv2.cvtColor(np.ascontiguousarray(pixels), cv2.COLOR_RGB2BGR)
                draw(drawn, hailo, (0, 200, 0))
                draw(drawn, oak, (0, 140, 255))
                cv2.imwrite(str(args.save / f"frame-{detections.getSequenceNum()}.jpg"), drawn)
                saved += 1

    def stats(values):
        return {"median": round(float(np.median(values)), 1), "p90": round(float(np.percentile(values, 90)), 1),
                "count": len(values)} if values else None

    print(json.dumps({**totals, "keypoint_distance_px": stats(distances), "nose_distance_px": stats(nose_distances),
                      "fps": round(totals["frames"] / args.seconds, 1)}))


if __name__ == "__main__":
    main()
