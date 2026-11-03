"""Luxonis OAK USB camera (such as the OAK-1): frames for the pose detector, and the models on the OAK's own chip.

The OAK does its own image work: its ISP turns the image the right way up, its hardware
encoder makes each frame's JPEG for the dashboard and the recorder, and a Sync node pairs
the two on the chip, so the Pi neither flips nor encodes. With `faces`, the OAK's neural
engines also run Luxonis Model Zoo models: YuNet face detection on every frame, then head
pose, gaze and MediaPipe face landmarks on the faces it found, from crops the OAK cuts out
itself (oak_script.py). The Pi only decodes YuNet's boxes and sends back which crops to cut
(oak_faces.py). The skeleton stays on the Hailo-8, which runs a larger pose model than
this chip can hold at camera rate beside the face models (docs/audit-2026.md, "The OAK's
own chip").

One thread reads every OAK output, waiting on whichever sends next instead of polling, and
hands frames and faces to the event loop, where they are published.
"""
import asyncio
import logging
import threading
import time
from collections import deque
from datetime import timedelta
from pathlib import Path

from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.sensory.oak_faces import Crop, FaceAssembler
from ultraskelly.core.bot.sensory.oak_models import MODEL_CACHE, load_archive
from ultraskelly.core.bot.sensory.oak_script import GAZE_ANGLES_INPUT, RESULT_LAYER, SCRIPT
from ultraskelly.core.bot.sensory.yunet import decode_yunet
from ultraskelly.core.pubsub.bot_topics import FacesMessage, FacesTopic, FrameMessage, FrameTopic
from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

YUNET_INPUT = (320, 240)
RATE_WINDOW = 2.0  # Seconds of arrivals each reported rate counts
RECENT_FACES = 32  # Frames whose faces faces_for still finds
CROP_SIZES = {"head_pose": 60, "left_eye": 60, "right_eye": 60, "mesh": 192}  # Each crop's network input side
# Inference threads and SHAVEs per thread for each network. depthai's defaults starve the
# chain; these held every network at 30 fps together on the robot's OAK-1 (audit).
RESOURCES = {"face_detection": (1, 4), "head_pose": (1, 1), "gaze": (1, 1), "face_landmarks": (1, 2)}


class OakFaceParams(NodeParams):
    """The face chain on the OAK's chip."""

    fps: float = Field(default=30.0, gt=0.0, le=60.0, description="Frames per second YuNet looks for faces in")
    score_threshold: float = Field(default=0.6, ge=0.0, le=1.0, description="Least YuNet score kept as a face")
    max_faces: int = Field(default=3, ge=0, le=8, description="Most faces per frame measured for head pose and gaze")
    measure_every: int = Field(default=2, ge=1, description="Head pose and gaze in one face detection of this many")
    mesh_every: int | None = Field(
        default=2, ge=1,
        description="MediaPipe's 468 face landmarks, on the largest face, in one detection of this many; None: never",
    )
    resources: dict[str, tuple[int, int]] = Field(
        default_factory=lambda: dict(RESOURCES), description="Inference threads and SHAVEs per thread, by network"
    )
    model_cache: Path = Field(default=MODEL_CACHE, description="Where the Model Zoo archives are kept")


class OakCameraNodeParams(NodeParams):
    """Parameters for OakCameraNode."""

    width: int = Field(default=640, ge=160, le=1920)
    height: int = Field(default=480, ge=120, le=1080)
    fps: float = Field(default=30.0, gt=0.0, le=60.0)
    flip_camera: bool = Field(
        default=False,
        description="Rotate camera image 180 degrees (for upside-down mounting); the OAK's ISP does it",
    )
    jpeg_quality: int | None = Field(
        default=80, ge=10, le=100,
        description="Quality of the JPEG the OAK's hardware encoder makes of each frame; None sends frames without one",
    )
    faces: OakFaceParams | None = Field(default=None, description="Run the face chain on the OAK's chip; None runs none")
    frames_pool: int = Field(
        default=10, ge=4, le=16,
        description="Frames in each camera output's pool. Frames waiting in queues (the Script keeps 3 for their "
                    "crop requests) hold pool frames, and a camera whose pool runs out stops every output",
    )
    system_log_hz: float = Field(default=1.0, gt=0.0, le=10.0, description="OAK CPU, memory and temperature reports per second")


class OakCameraNode(Node):
    """Streams the OAK's frames, each with its JPEG, and the faces its own chip finds in them."""

    params: OakCameraNodeParams
    pipeline: SkipValidation[object] = Field(default=None, exclude=True)
    queues: SkipValidation[dict] = Field(default_factory=dict, exclude=True)  # Output name: depthai MessageQueue
    crop_requests: SkipValidation[object] = Field(default=None, exclude=True)  # Input queue of the Script node
    faces: SkipValidation[FaceAssembler | None] = Field(default=None, exclude=True)
    outbox: SkipValidation[asyncio.Queue | None] = Field(default=None, exclude=True)
    reader_stop: SkipValidation[threading.Event] = Field(default_factory=threading.Event, exclude=True)
    # Recent arrival times of each OAK output, and of the faces published with each measurement.
    arrivals: SkipValidation[dict[str, deque]] = Field(default_factory=dict, exclude=True)
    arrivals_lock: SkipValidation[threading.Lock] = Field(default_factory=threading.Lock, exclude=True)
    chip: dict = Field(default_factory=dict, exclude=True)  # The newest SystemLogger report
    # The faces of recent frames by frame number, for consumers that need every frame's (faces_for).
    recent_faces: SkipValidation[dict[int, FacesMessage]] = Field(default_factory=dict, exclude=True)
    dropped: int = Field(default=0, exclude=True)  # Messages the event loop was too busy to take

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: OakCameraNodeParams) -> "OakCameraNode":
        """Factory method: connects to the first OAK found, loads its models and builds its pipeline."""
        import depthai as dai

        node = cls(pubsub=pubsub, params=params)
        node.pipeline = dai.Pipeline()  # Connects to and boots the first available OAK
        try:
            node._build(dai)
        except BaseException:
            node.pipeline.getDefaultDevice().close()
            raise
        return node

    def _build(self, dai) -> None:
        params, pipeline = self.params, self.pipeline
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        camera.setOutputsNumFramesPool(params.frames_pool)
        if params.flip_camera:
            camera.setImageOrientation(dai.CameraImageOrientation.ROTATE_180_DEG)
        size = (params.width, params.height)
        rgb = camera.requestOutput(size, dai.ImgFrame.Type.RGB888i, fps=params.fps)
        frames = rgb
        if params.jpeg_quality is not None:
            nv12 = camera.requestOutput(size, dai.ImgFrame.Type.NV12, fps=params.fps)
            encoder = pipeline.create(dai.node.VideoEncoder).build(
                nv12, frameRate=params.fps, profile=dai.VideoEncoderProperties.Profile.MJPEG, quality=params.jpeg_quality,
            )
            sync = pipeline.create(dai.node.Sync)  # Each frame with its own JPEG: they share the ISP frame's time
            sync.setSyncThreshold(timedelta(milliseconds=10))
            rgb.link(sync.inputs["rgb"])
            encoder.out.link(sync.inputs["jpeg"])
            frames = sync.out
        self.queues["frames"] = frames.createOutputQueue(maxSize=4, blocking=False)
        if params.faces is not None:
            self._build_faces(dai, camera, params.faces)
        system = pipeline.create(dai.node.SystemLogger)
        system.setRate(params.system_log_hz)
        self.queues["system"] = system.out.createOutputQueue(maxSize=2, blocking=False)

    def _build_faces(self, dai, camera, faces: OakFaceParams) -> None:
        """YuNet on its own small camera output; the Script node cuts the crops out of full-size frames,
        planar BGR like the networks take, so cropping converts nothing."""
        pipeline = self.pipeline

        def network(name: str, image_inputs: list | None = None):
            threads, shaves = faces.resources[name]
            node = pipeline.create(dai.node.NeuralNetwork)
            node.setNNArchive(load_archive(name, faces.model_cache), shaves)  # The SHAVE count picks the blob
            node.setNumInferenceThreads(threads)
            for port in image_inputs if image_inputs is not None else [node.input]:
                port.setBlocking(True)  # Inside the chain nothing is dropped, so answers stay in step
                port.setMaxSize(4)
            return node

        detector = network("face_detection", image_inputs=[])
        camera.requestOutput(YUNET_INPUT, dai.ImgFrame.Type.BGR888p, fps=faces.fps).link(detector.input)
        detector.input.setBlocking(False)  # The detector takes the newest frame
        detector.input.setMaxSize(1)
        self.queues["face_detection"] = detector.out.createOutputQueue(maxSize=2, blocking=False)

        script = pipeline.create(dai.node.Script)
        script.setScript(SCRIPT, "face-chain")
        size = (self.params.width, self.params.height)
        camera.requestOutput(size, dai.ImgFrame.Type.BGR888p, fps=faces.fps).link(script.inputs["frame"])
        script.inputs["frame"].setBlocking(False)  # Recent frames wait here for their crop request
        script.inputs["frame"].setMaxSize(3)
        script.inputs["crops"].setBlocking(False)
        script.inputs["crops"].setMaxSize(2)
        self.crop_requests = script.inputs["crops"].createInputQueue()
        manips = {}
        for crop, side in CROP_SIZES.items():
            if crop == "mesh" and faces.mesh_every is None:
                continue
            manip = pipeline.create(dai.node.ImageManip)
            manip.inputConfig.setWaitForMessage(True)
            manip.setMaxOutputFrameSize(side * side * 3)
            for port in (manip.inputConfig, manip.inputImage):
                port.setBlocking(True)  # Short queues: each waiting crop holds one of the camera's frames
                port.setMaxSize(1)
            script.outputs[f"{crop}_config"].link(manip.inputConfig)
            script.outputs[f"{crop}_image"].link(manip.inputImage)
            manips[crop] = manip

        head_pose = network("head_pose")
        manips["head_pose"].out.link(head_pose.input)
        head_pose.out.link(script.inputs["head_pose"])
        gaze_inputs = ["left_eye_image", "right_eye_image", GAZE_ANGLES_INPUT]
        gaze = network("gaze", image_inputs=[])
        for name in gaze_inputs:
            gaze.inputs[name].setBlocking(True)
            gaze.inputs[name].setMaxSize(4)
        manips["left_eye"].out.link(gaze.inputs["left_eye_image"])
        manips["right_eye"].out.link(gaze.inputs["right_eye_image"])
        script.outputs["angles"].link(gaze.inputs[GAZE_ANGLES_INPUT])
        gaze.out.link(script.inputs["gaze"])
        for name in ("head_pose", "gaze"):
            script.inputs[name].setBlocking(True)
            script.inputs[name].setMaxSize(8)
        self.queues["face_results"] = script.outputs["results"].createOutputQueue(maxSize=4, blocking=False)
        if faces.mesh_every is not None:
            mesh = network("face_landmarks")
            manips["mesh"].out.link(mesh.input)
            self.queues["face_landmarks"] = mesh.out.createOutputQueue(maxSize=4, blocking=False)
        self.faces = FaceAssembler(
            (self.params.width, self.params.height), max_faces=faces.max_faces, every=faces.measure_every,
            mesh_every=faces.mesh_every,
        )

    def faces_for(self, frame_number: int) -> FacesMessage | None:
        """The faces found in a recent frame, once its measurements are in; None before (or long after).
        FacesTopic keeps only the newest message, and two can be published together."""
        with self.arrivals_lock:
            return self.recent_faces.get(frame_number)

    def status(self) -> dict:
        """For the dashboard: per second over the last RATE_WINDOW, frames, face detections and the
        faces published with each measurement; and the chip's load and temperature."""
        now = time.monotonic()
        with self.arrivals_lock:
            rates = {name: round(sum(now - at <= RATE_WINDOW for at in times) / RATE_WINDOW, 1)
                     for name, times in self.arrivals.items()}
        return {
            "rates": rates,
            "chip": dict(self.chip),
            "faces_incomplete": 0 if self.faces is None else self.faces.incomplete,
        }

    # The reader thread: everything below runs there, except _deliver, on the event loop.

    def _read(self, loop: asyncio.AbstractEventLoop) -> None:
        import depthai as dai

        try:
            while not self.reader_stop.is_set():
                messages = dai.MessageQueue.getAny(self.queues, timedelta(seconds=0.25))
                now = time.monotonic()
                for name, message in messages.items():
                    if name in ("frames", "face_detection"):
                        self._count(name, now)
                    getattr(self, f"_on_{name}")(dai, message, now, loop)
                if self.faces is not None:
                    for faces in self.faces.finished(now):
                        self._post(loop, FacesTopic, faces)
                if not messages and not self.reader_stop.is_set() and not self.pipeline.isRunning():
                    raise RuntimeError("OAK pipeline stopped unexpectedly (camera unplugged?)")
        except Exception as error:
            if not self.reader_stop.is_set():
                self._post(loop, None, error)  # run() raises it on the event loop

    def _count(self, name: str, now: float, count: int = 1) -> None:
        with self.arrivals_lock:
            times = self.arrivals.setdefault(name, deque())
            times.extend([now] * count)
            while times and now - times[0] > RATE_WINDOW:
                times.popleft()

    def _post(self, loop: asyncio.AbstractEventLoop, topic: type[PubSubTopicABC] | None, message: object) -> None:
        if isinstance(message, FacesMessage):
            now = time.monotonic()
            with self.arrivals_lock:
                self.recent_faces[message.frame_number] = message
                while len(self.recent_faces) > RECENT_FACES:
                    del self.recent_faces[min(self.recent_faces)]
            for measurement, count in (
                ("head_pose", sum(face.head_pose is not None for face in message.faces)),
                ("gaze", sum(face.gaze is not None for face in message.faces)),
                ("face_landmarks", sum(face.mesh is not None for face in message.faces)),
            ):
                self._count(measurement, now, count)
        loop.call_soon_threadsafe(self._deliver, topic, message)

    def _deliver(self, topic: type[PubSubTopicABC] | None, message: object) -> None:
        if self.outbox.full():
            self.outbox.get_nowait()
            self.dropped += 1
        self.outbox.put_nowait((topic, message))

    def _on_frames(self, dai, message, now: float, loop: asyncio.AbstractEventLoop) -> None:
        image, jpeg = (message["rgb"], bytes(message["jpeg"].getData())) if isinstance(message, dai.MessageGroup) else (message, None)
        self._post(loop, FrameTopic, FrameMessage(
            frame=image.getFrame(),
            jpeg=jpeg,
            frame_number=image.getSequenceNum(),
            # depthai stamps frames on the host's steady clock: time.monotonic()'s on Linux.
            timestamp=image.getTimestamp().total_seconds(),
        ))

    def _on_face_detection(self, dai, message, now: float, loop: asyncio.AbstractEventLoop) -> None:
        faces = decode_yunet(
            message.getTensor("loc"), message.getTensor("conf"), message.getTensor("iou"),
            input_size=YUNET_INPUT, score_threshold=self.params.faces.score_threshold,
        )
        sequence = message.getSequenceNum()
        crops, finished = self.faces.detected(sequence, message.getTimestamp().total_seconds(), faces, now)
        if crops:
            self.crop_requests.send(crop_request(dai, crops, sequence))
        for done in finished:
            self._post(loop, FacesTopic, done)

    def _on_face_results(self, dai, message, now: float, loop: asyncio.AbstractEventLoop) -> None:
        for done in self.faces.answered(message.getSequenceNum(), now, results=message.getTensor(RESULT_LAYER)):
            self._post(loop, FacesTopic, done)

    def _on_face_landmarks(self, dai, message, now: float, loop: asyncio.AbstractEventLoop) -> None:
        for done in self.faces.answered(message.getSequenceNum(), now, mesh=message.getFirstTensor()):
            self._post(loop, FacesTopic, done)

    def _on_system(self, dai, message, now: float, loop: asyncio.AbstractEventLoop) -> None:
        self.chip = {
            "leon_os_cpu": round(message.leonCssCpuUsage.average, 3),
            "leon_rt_cpu": round(message.leonMssCpuUsage.average, 3),
            "ddr_used_mib": round(message.ddrMemoryUsage.used / 2**20, 1),
            "ddr_total_mib": round(message.ddrMemoryUsage.total / 2**20, 1),
            "temperature_c": round(message.chipTemperature.average, 1),
        }

    async def run(self) -> None:
        """Start the OAK and publish what it sends until stopped."""
        faces = self.params.faces
        logger.info(
            f"Starting OakCameraNode [{self.params.width}x{self.params.height} @ {self.params.fps:g} fps, "
            f"flip_camera={self.params.flip_camera}, jpeg_quality={self.params.jpeg_quality}, "
            f"faces={'off' if faces is None else f'{faces.fps:g} fps, head pose 1 in {faces.measure_every}, mesh 1 in {faces.mesh_every}'}]"
        )
        loop = asyncio.get_running_loop()
        self.outbox = asyncio.Queue(maxsize=16)
        self.pipeline.start()
        reader = threading.Thread(target=self._read, args=(loop,), name="oak-reader", daemon=True)
        reader.start()
        try:
            while not self.stop_event.is_set():
                try:
                    topic, message = await asyncio.wait_for(self.outbox.get(), timeout=0.5)
                except asyncio.TimeoutError:
                    continue
                if topic is None:
                    raise message
                await self.pubsub.topics[topic].publish(message)
        finally:
            self.reader_stop.set()
            await asyncio.to_thread(reader.join, 2.0)
            self.pipeline.stop()
            # Release the USB device so the next run can connect to it.
            self.pipeline.getDefaultDevice().close()
            logger.info(f"OakCameraNode stopped ({self.dropped} messages dropped)")


def crop_request(dai, crops: list[Crop], sequence: int):
    """The Script node's request for one frame: a detection per crop, its label the crop's kind and its
    confidence the face's row, with the frame's sequence number."""
    request = dai.ImgDetections()
    detections = []
    for crop in crops:
        detection = dai.ImgDetection()
        detection.xmin, detection.ymin, detection.xmax, detection.ymax = crop.box
        detection.label = int(crop.kind)
        detection.confidence = float(crop.face)
        detections.append(detection)
    request.detections = detections
    request.setSequenceNum(sequence)
    return request
