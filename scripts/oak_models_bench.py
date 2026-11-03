"""Measure Luxonis Model Zoo models on the OAK's own chip, alone and together.

Uses the OAK, so stop the bot and run it under the robot lock:

    flock -w 60 /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.oak_models_bench \
        --run yunet --run yunet,headpose,pose8n --seconds 15

Each `--run` is one pipeline: the camera at `--fps` and the listed parts (`Bench.add`), each
optionally as `part@<threads>x<shaves>:<fps>`, for example `pose8n@2x6` or `hand@1x2:15`. The
second-stage models (head pose, gaze, face and hand landmarks, emotion) get fixed crops, as
if one face and one hand were in every frame, so their load does not depend on who is in
view. Prints one JSON line per run with each part's output rate and latency (capture to the
host), the OAK's CPU loads, memory and chip temperature (SystemLogger) and this process's
CPU. Models come from the zoo once and are then read from `--cache`.
"""
import argparse
import json
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path

import depthai as dai
import numpy as np

CACHE = Path.home() / "ultraskelly-models" / "oak-zoo"
MODELS = {
    "yunet": "luxonis/yunet:320x240",
    "yunet640": "luxonis/yunet:640x480",
    "headpose": "luxonis/head-pose-estimation:60x60",
    "gaze": "luxonis/gaze-estimation-adas:60x60",
    "facemesh": "luxonis/mediapipe-face-landmarker:192x192",
    "palm": "luxonis/mediapipe-palm-detection:192x192",
    "palm128": "luxonis/mediapipe-palm-detection:128x128",
    "hand": "luxonis/mediapipe-hand-landmarker:224x224",
    "pose8n": "luxonis/yolov8-nano-pose-estimation:coco-512x288",
    "pose8nraw": "luxonis/yolov8-nano-pose-estimation:coco-512x288",  # Raw tensors to the host, no on-device decoding
    "pose26n": "luxonis/yolo26-nano-pose-estimation:coco-512x288",
    "pose26n43": "luxonis/yolo26-nano-pose-estimation:coco-512x384",
    "emotion": "luxonis/emotion-recognition:gray-64x64",
    "selfie": "luxonis/mediapipe-selfie-segmentation:256x144",
}
# Fixed crops in normalized coordinates of the 4:3 frame: (x, y, width, height).
FACE = (0.42, 0.25, 0.16, 0.21)
LEFT_EYE, RIGHT_EYE = (0.44, 0.3, 0.05, 0.067), (0.51, 0.3, 0.05, 0.067)
HAND = (0.2, 0.5, 0.2, 0.27)
FRAME_TYPES = {"bgr": dai.ImgFrame.Type.BGR888p, "nv12": dai.ImgFrame.Type.NV12}
BACKENDS = {"cpu": dai.node.ImageManip.Backend.CPU, "hw": dai.node.ImageManip.Backend.HW, "auto": dai.node.ImageManip.Backend.AUTO}


def archive(slug: str, cache: Path) -> dai.NNArchive:
    path = dai.getModelFromZoo(dai.NNModelDescription(slug, platform="RVC2"), useCached=True, cacheDirectory=str(cache))
    return dai.NNArchive(str(path))


@dataclass
class Output:
    """One host queue whose messages are counted."""

    queue: object
    arrivals: list[float] = field(default_factory=list)
    latencies: list[float] = field(default_factory=list)
    detections: int = 0


@dataclass
class Bench:
    pipeline: dai.Pipeline
    camera: object
    args: argparse.Namespace
    outputs: dict[str, Output] = field(default_factory=dict)
    crop_sources: dict[float, object] = field(default_factory=dict)  # Camera outputs the crops are cut from, by rate
    rate: float = 30.0  # Frames per second of the part being added
    senders: list = field(default_factory=list)  # (input queue, message) sent once at the start
    resources: dict[str, tuple[int, int]] = field(default_factory=dict)  # Part: (threads, SHAVEs per thread)

    def model(self, part: str) -> dai.NNArchive:
        return archive(MODELS[part], self.args.cache)

    def watch(self, name: str, output) -> None:
        self.outputs[name] = Output(output.createOutputQueue(maxSize=8, blocking=False))

    def create_network(self, part: str, kind=dai.node.NeuralNetwork, source=None, inputs=None):
        """A network for `part`, with threads and SHAVEs from `part@<threads>x<shaves>` or the command
        line. The SHAVE count picks the blob out of the superblob, so it is given with the archive.
        Unless --queue-frames, its image inputs keep only the newest frame."""
        network = self.pipeline.create(kind)
        threads, shaves = self.resources.get(part, (self.args.threads, self.args.shaves))
        if shaves:
            network.setNNArchive(self.model(part), shaves)
        else:
            network.setNNArchive(self.model(part))
        if threads:
            network.setNumInferenceThreads(threads)
        if source is not None:
            source.link(network.input)
        if not self.args.queue_frames:
            for port in inputs if inputs is not None else [network.input]:
                port.setBlocking(False)
                port.setMaxSize(1)
        self.watch(part, network.out)
        return network

    def crop(self, box: tuple[float, float, float, float], size: tuple[int, int], frame_type=dai.ImgFrame.Type.BGR888p):
        """An ImageManip cropping `box` out of the shared crop source (--crop-source, --crop-type) and resizing it to `size`."""
        fps = self.rate
        if fps not in self.crop_sources:
            self.crop_sources[fps] = self.camera.requestOutput(self.args.crop_source, FRAME_TYPES[self.args.crop_type], fps=fps)
        manip = self.pipeline.create(dai.node.ImageManip)
        manip.setBackend(BACKENDS[self.args.manip_backend])
        width, height = self.args.crop_source
        x, y, w, h = box
        manip.initialConfig.addCrop(int(x * width), int(y * height), int(w * width), int(h * height))
        manip.initialConfig.setOutputSize(*size)
        manip.initialConfig.setFrameType(frame_type)
        manip.setMaxOutputFrameSize(size[0] * size[1] * 3)
        self.crop_sources[fps].link(manip.inputImage)
        return manip.out

    def network(self, part: str, source) -> object:
        return self.create_network(part, source=source)

    def add(self, part: str, rate: float | None = None) -> None:
        camera = self.camera
        fps = self.rate = rate or self.args.fps
        if part == "rgb":  # What the bot reads today
            self.watch(part, camera.requestOutput((640, 480), dai.ImgFrame.Type.RGB888i, fps=fps))
        elif part == "mjpeg":
            nv12 = camera.requestOutput((640, 480), dai.ImgFrame.Type.NV12, fps=fps)
            encoder = self.pipeline.create(dai.node.VideoEncoder).build(
                nv12, frameRate=fps, profile=dai.VideoEncoderProperties.Profile.MJPEG, quality=80
            )
            self.watch(part, encoder.out)
        elif part in ("yunet", "yunet640"):
            size = (320, 240) if part == "yunet" else (640, 480)
            self.network(part, camera.requestOutput(size, dai.ImgFrame.Type.BGR888p, fps=rate or self.args.face_fps))
        elif part == "headpose":
            self.network(part, self.crop(FACE, (60, 60)))
        elif part == "facemesh":
            self.network(part, self.crop(FACE, (192, 192)))
        elif part == "emotion":
            self.network(part, self.crop(FACE, (64, 64), dai.ImgFrame.Type.GRAY8))
        elif part == "hand":
            self.network(part, self.crop(HAND, (224, 224)))
        elif part in ("palm", "palm128"):
            size = (192, 192) if part == "palm" else (128, 128)
            source = camera.requestOutput(size, dai.ImgFrame.Type.BGR888p, dai.ImgResizeMode.LETTERBOX, fps=fps)
            self.network(part, source)
        elif part == "selfie":
            source = camera.requestOutput((256, 144), dai.ImgFrame.Type.BGR888p, dai.ImgResizeMode.LETTERBOX, fps=fps)
            self.network(part, source)
        elif part == "gaze":
            network = self.create_network(part, inputs=[])
            eyes = [network.inputs["left_eye_image"], network.inputs["right_eye_image"]]
            for port in eyes if not self.args.queue_frames else []:
                port.setBlocking(False)
                port.setMaxSize(1)
            self.crop(LEFT_EYE, (60, 60)).link(eyes[0])
            self.crop(RIGHT_EYE, (60, 60)).link(eyes[1])
            angles = network.inputs["head_pose_angles_yaw_pitch_roll"]
            angles.setReusePreviousMessage(True)  # Stands in for head pose's output, sent once
            message = dai.NNData()
            message.addTensor("head_pose_angles_yaw_pitch_roll", np.zeros((1, 3), dtype=np.float32), dai.TensorInfo.StorageOrder.NC)
            self.senders.append((angles.createInputQueue(), message))
        elif part in ("pose8n", "pose8nraw", "pose26n", "pose26n43"):
            size = (512, 288) if part != "pose26n43" else (512, 384)
            mode = dai.ImgResizeMode.LETTERBOX if self.args.pose_resize == "letterbox" else dai.ImgResizeMode.CROP
            source = camera.requestOutput(size, dai.ImgFrame.Type.BGR888p, mode, fps=rate or self.args.pose_fps or fps)
            kind = dai.node.NeuralNetwork if part == "pose8nraw" else dai.node.DetectionNetwork
            self.create_network(part, kind, source)
        else:
            raise SystemExit(f"Unknown part {part!r}; choose from rgb, mjpeg, {', '.join(MODELS)}")


def summarize(values: list[float]) -> dict:
    if not values:
        return {}
    ordered = sorted(values)
    return {"median": round(1000 * statistics.median(ordered), 1), "p90": round(1000 * ordered[int(0.9 * (len(ordered) - 1))], 1)}


def run(parts: list[str], args: argparse.Namespace) -> dict:
    with dai.Pipeline() as pipeline:
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        camera.setImageOrientation(dai.CameraImageOrientation.ROTATE_180_DEG)
        bench = Bench(pipeline, camera, args)
        for spec in parts:
            spec, _, rate = spec.partition(":")
            part, _, resources = spec.partition("@")
            if resources:
                threads, shaves = resources.split("x")
                bench.resources[part] = (int(threads), int(shaves))
            bench.add(part, float(rate) if rate else None)
        logger = pipeline.create(dai.node.SystemLogger)
        logger.setRate(2)
        system = logger.out.createOutputQueue(maxSize=16, blocking=False)
        pipeline.start()
        for queue, message in bench.senders:
            queue.send(message)
        started = time.monotonic()
        measure_from, measure_to = started + args.warmup, started + args.warmup + args.seconds
        samples, cpu_start = [], None
        while (now := time.monotonic()) < measure_to:
            if not pipeline.isRunning():
                raise RuntimeError("The OAK pipeline stopped")
            if cpu_start is None and now >= measure_from:
                cpu_start = (time.process_time(), now)
            for output in bench.outputs.values():
                while (message := output.queue.tryGet()) is not None:
                    if now < measure_from:
                        continue
                    output.arrivals.append(now)
                    output.latencies.append(now - message.getTimestamp().total_seconds())
                    if isinstance(message, dai.ImgDetections):
                        output.detections += len(message.detections)
            while (info := system.tryGet()) is not None:
                if now >= measure_from:
                    samples.append(info)
            time.sleep(0.001)
        process_cpu = (time.process_time() - cpu_start[0]) / (time.monotonic() - cpu_start[1])
    result = {
        "parts": parts,
        "fps": args.fps,
        "outputs": {
            name: {
                "rate": round(len(output.arrivals) / args.seconds, 1),
                "latency_ms": summarize(output.latencies),
                **({"detections_per_frame": round(output.detections / len(output.arrivals), 2)} if output.detections else {}),
            }
            for name, output in bench.outputs.items()
        },
        "host_process_cpu": round(process_cpu, 3),
    }
    if samples:
        result["oak"] = {
            "leon_css_cpu": round(statistics.mean(s.leonCssCpuUsage.average for s in samples), 3),
            "leon_mss_cpu": round(statistics.mean(s.leonMssCpuUsage.average for s in samples), 3),
            "ddr_used_mib": round(max(s.ddrMemoryUsage.used for s in samples) / 2**20, 1),
            "ddr_total_mib": round(samples[0].ddrMemoryUsage.total / 2**20, 1),
            "css_heap_used_mib": round(max(s.leonCssMemoryUsage.used for s in samples) / 2**20, 1),
            "mss_heap_used_mib": round(max(s.leonMssMemoryUsage.used for s in samples) / 2**20, 1),
            "chip_temp_c": round(samples[-1].chipTemperature.average, 1),
        }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure Model Zoo models on the OAK's chip.")
    parser.add_argument("--run", action="append", required=True, help="Comma-separated parts in one pipeline; repeat for more runs")
    parser.add_argument("--seconds", type=float, default=15.0, help="Measuring window per run, after the warm-up")
    parser.add_argument("--warmup", type=float, default=6.0)
    parser.add_argument("--fps", type=float, default=30.0, help="Camera rate for the full-frame parts")
    parser.add_argument("--face-fps", type=float, default=None, help="Rate of the face detector's frames (default --fps)")
    parser.add_argument("--pose-fps", type=float, default=None, help="Rate of the pose model's frames (default --fps)")
    parser.add_argument("--pose-resize", choices=["letterbox", "crop"], default="letterbox")
    parser.add_argument("--threads", type=int, default=0, help="Inference threads per network; 0 leaves depthai's default")
    parser.add_argument("--shaves", type=int, default=0, help="SHAVEs per inference thread; 0 leaves depthai's default")
    parser.add_argument("--crop-source", type=lambda text: tuple(int(v) for v in text.split("x")), default=(640, 480),
                        help="Size of the camera output the fixed crops are cut from, WxH")
    parser.add_argument("--crop-type", choices=FRAME_TYPES, default="bgr", help="Pixel format of the crop source")
    parser.add_argument("--manip-backend", choices=BACKENDS, default="cpu", help="ImageManip backend for the crops")
    parser.add_argument("--queue-frames", action="store_true", help="Leave networks' input queues as depthai sets them (blocking)")
    parser.add_argument("--cache", type=Path, default=CACHE)
    args = parser.parse_args()
    args.face_fps = args.face_fps or args.fps
    for parts in args.run:
        print(json.dumps(run(parts.split(","), args)), flush=True)


if __name__ == "__main__":
    main()
