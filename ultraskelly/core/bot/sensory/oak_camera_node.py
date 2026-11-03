"""Luxonis OAK USB camera (such as the OAK-1): frames for the pose detector, and the models on the OAK's own chip.

The OAK does its own image work: its ISP turns the image the right way up, its hardware
encoder makes each frame's JPEG for the dashboard and the recorder, and a Sync node pairs
the two on the chip, so the Pi neither flips nor encodes. With `models`, the OAK's neural
engines also run Luxonis Model Zoo models: YuNet face detection on every frame, then head
pose, gaze and MediaPipe face landmarks on the faces it found, and on some frames MediaPipe's
palm detector and hand landmarker, from crops the OAK cuts out itself (oak_script.py). The Pi
only decodes YuNet's boxes and the palms and sends back which crops to cut (oak_faces.py,
oak_results.py). The skeleton stays on the Hailo-8, which runs a larger pose model than this
chip can hold at camera rate beside the face models (docs/audit-2026.md, "The OAK's own chip").

One thread reads every OAK output, waiting on whichever sends next instead of polling, and
hands frames and results to the event loop, where they are published.
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
from ultraskelly.core.bot.sensory.oak_faces import Crop, CropKind
from ultraskelly.core.bot.sensory.oak_models import MODEL_CACHE, load_archive
from ultraskelly.core.bot.sensory.oak_results import ResultsAssembler
from ultraskelly.core.bot.sensory.oak_script import GAZE_ANGLES_INPUT, KEEP_FRAMES, RESULT_LAYER, SCRIPT
from ultraskelly.core.bot.sensory.yunet import decode_yunet
from ultraskelly.core.pubsub.bot_topics import OakResultsMessage, OakResultsTopic, FrameMessage, FrameTopic
from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

YUNET_INPUT = (320, 240)
RATE_WINDOW = 2.0  # Seconds of arrivals each reported rate counts
RECENT_RESULTS = 32  # Frames whose results results_for still finds
# Each crop's network input side
CROP_SIZES = {"head_pose": 60, "left_eye": 60, "right_eye": 60, "mesh": 192, "palm": 128, "hand": 224}
# Inference threads and SHAVEs per thread for each network. depthai's defaults starve the
# chain; the face networks held 30 fps together with these on the robot's OAK-1 (audit).
RESOURCES = {"face_detection": (1, 4), "head_pose": (1, 1), "gaze": (1, 1), "face_landmarks": (1, 2),
             "palm_detection": (1, 3), "hand_landmarks": (1, 2)}


class OakModelParams(NodeParams):
    """The models on the OAK's chip: faces on every frame, and what is measured on them and on hands."""

    fps: float = Field(default=30.0, gt=0.0, le=60.0, description="Frames per second YuNet looks for faces in")
    score_threshold: float = Field(default=0.6, ge=0.0, le=1.0, description="Least YuNet score kept as a face")
    # Defaults measured on the robot's OAK-1 with one face in view: with hands every sixth frame they
    # keep its real-time CPU near 75%, leaving room for a second face (audit).
    max_faces: int = Field(default=2, ge=0, le=8, description="Most faces per frame measured for head pose and gaze")
    measure_every: int = Field(default=2, ge=1, description="Head pose and gaze in one face detection of this many")
    mesh_every: int | None = Field(
        default=4, ge=1,
        description="MediaPipe's 468 face landmarks, on the largest face, in one detection of this many; None: never",
    )
    hands_every: int | None = Field(
        default=6, ge=1, description="Look for hands in one face detection of this many (5 fps at 30); None: never",
    )
    max_hands: int = Field(default=2, ge=1, le=4, description="Most hands per frame measured")
    hand_threshold: float = Field(default=0.5, ge=0.0, le=1.0, description="Least hand landmarker presence kept as a hand")
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
    models: OakModelParams | None = Field(default=None, description="Run models on the OAK's chip; None runs none")
    frames_pool: int = Field(
        default=14, ge=4, le=16,
        description="Frames in each camera output's pool. Frames waiting in queues and the Script's own "
                    f"{KEEP_FRAMES} hold pool frames, and a camera whose pool runs out stops every output",
    )
    system_log_hz: float = Field(default=1.0, gt=0.0, le=10.0, description="OAK CPU, memory and temperature reports per second")


class OakCameraNode(Node):
    """Streams the OAK's frames, each with its JPEG, and the faces and hands its own chip finds in them."""

    params: OakCameraNodeParams
    pipeline: SkipValidation[object] = Field(default=None, exclude=True)
    queues: SkipValidation[dict] = Field(default_factory=dict, exclude=True)  # Output name: depthai MessageQueue
    crop_requests: SkipValidation[object] = Field(default=None, exclude=True)  # Input queue of the Script node
    assembler: SkipValidation[ResultsAssembler | None] = Field(default=None, exclude=True)
    outbox: SkipValidation[asyncio.Queue | None] = Field(default=None, exclude=True)
    reader_stop: SkipValidation[threading.Event] = Field(default_factory=threading.Event, exclude=True)
    # Recent arrival times of each OAK output, and of the faces published with each measurement.
    arrivals: SkipValidation[dict[str, deque]] = Field(default_factory=dict, exclude=True)
    arrivals_lock: SkipValidation[threading.Lock] = Field(default_factory=threading.Lock, exclude=True)
    chip: dict = Field(default_factory=dict, exclude=True)  # The newest SystemLogger report
    # The results of recent frames by frame number, for consumers that need every frame's (results_for).
    recent_results: SkipValidation[dict[int, OakResultsMessage]] = Field(default_factory=dict, exclude=True)
    dropped: int = Field(default=0, exclude=True)  # Messages the event loop was too busy to take
    models_error: str | None = Field(default=None, exclude=True)  # Why the OAK runs without its models

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
        if params.models is not None:
            self._build_models(dai, camera, params.models)
        system = pipeline.create(dai.node.SystemLogger)
        system.setRate(params.system_log_hz)
        self.queues["system"] = system.out.createOutputQueue(maxSize=2, blocking=False)

    def _build_models(self, dai, camera, models: OakModelParams) -> None:
        """YuNet on its own small camera output; the Script node cuts the crops out of full-size frames,
        planar BGR like the networks take, so cropping converts nothing. Every model is loaded
        first: with one missing, the OAK runs none and the bot tracks with the Hailo alone."""
        pipeline = self.pipeline
        hands = models.hands_every is not None
        names = ["face_detection", "head_pose", "gaze"]
        names += ["face_landmarks"] if models.mesh_every is not None else []
        names += ["palm_detection", "hand_landmarks"] if hands else []
        try:
            archives = {name: load_archive(name, models.model_cache) for name in names}
        except RuntimeError as error:
            self.models_error = str(error)
            logger.error(f"Running the OAK without its models: {error}")
            return

        def network(name: str, image_inputs: list | None = None):
            threads, shaves = models.resources[name]
            node = pipeline.create(dai.node.NeuralNetwork)
            node.setNNArchive(archives[name], shaves)  # The SHAVE count picks the blob
            node.setNumInferenceThreads(threads)
            for port in image_inputs if image_inputs is not None else [node.input]:
                port.setBlocking(True)  # Inside the chain nothing is dropped, so answers stay in step
                port.setMaxSize(4)
            return node

        detector = network("face_detection", image_inputs=[])
        camera.requestOutput(YUNET_INPUT, dai.ImgFrame.Type.BGR888p, fps=models.fps).link(detector.input)
        detector.input.setBlocking(False)  # The detector takes the newest frame
        detector.input.setMaxSize(1)
        self.queues["face_detection"] = detector.out.createOutputQueue(maxSize=2, blocking=False)

        script = pipeline.create(dai.node.Script)
        script.setScript(SCRIPT, "crops")
        size = (self.params.width, self.params.height)
        camera.requestOutput(size, dai.ImgFrame.Type.BGR888p, fps=models.fps).link(script.inputs["frame"])
        script.inputs["frame"].setBlocking(False)  # Frames wait here, then among the Script's own, for crop requests
        script.inputs["frame"].setMaxSize(2)
        script.inputs["crops"].setBlocking(False)
        script.inputs["crops"].setMaxSize(2)
        self.crop_requests = script.inputs["crops"].createInputQueue()
        manips = {}
        for crop, side in CROP_SIZES.items():
            if (crop == "mesh" and models.mesh_every is None) or (crop in ("palm", "hand") and not hands):
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
        if models.mesh_every is not None:
            mesh = network("face_landmarks")
            manips["mesh"].out.link(mesh.input)
            self.queues["face_landmarks"] = mesh.out.createOutputQueue(maxSize=4, blocking=False)
        if hands:
            palms = network("palm_detection")
            manips["palm"].out.link(palms.input)
            self.queues["palm_detection"] = palms.out.createOutputQueue(maxSize=4, blocking=False)
            hand = network("hand_landmarks")
            manips["hand"].out.link(hand.input)
            self.queues["hand_landmarks"] = hand.out.createOutputQueue(maxSize=8, blocking=False)
        self.assembler = ResultsAssembler(
            (self.params.width, self.params.height), max_faces=models.max_faces, every=models.measure_every,
            mesh_every=models.mesh_every, hands_every=models.hands_every, max_hands=models.max_hands,
            hand_threshold=models.hand_threshold,
        )

    def results_for(self, frame_number: int) -> OakResultsMessage | None:
        """What the chip found in a recent frame, once all of it is in; None before (or long after).
        OakResultsTopic keeps only the newest message, and two can be published together."""
        with self.arrivals_lock:
            return self.recent_results.get(frame_number)

    def status(self) -> dict:
        """For the dashboard: per second over the last RATE_WINDOW, frames, face detections, palm
        detections, and the faces and hands published with each measurement; and the chip's load
        and temperature."""
        now = time.monotonic()
        with self.arrivals_lock:
            rates = {name: round(sum(now - at <= RATE_WINDOW for at in times) / RATE_WINDOW, 1)
                     for name, times in self.arrivals.items()}
        return {
            "rates": rates,
            "chip": dict(self.chip),
            "models_error": self.models_error,
            "faces_incomplete": 0 if self.assembler is None else self.assembler.incomplete,
            "missing": {} if self.assembler is None else dict(self.assembler.missing),
        }

    # The reader thread: everything below runs there, except _deliver, on the event loop.

    def _read(self, loop: asyncio.AbstractEventLoop) -> None:
        import depthai as dai

        try:
            while not self.reader_stop.is_set():
                messages = dai.MessageQueue.getAny(self.queues, timedelta(seconds=0.25))
                now = time.monotonic()
                for name, message in messages.items():
                    if name in ("frames", "face_detection", "palm_detection"):
                        self._count(name, now)
                    getattr(self, f"_on_{name}")(dai, message, now, loop)
                if self.assembler is not None:
                    for results in self.assembler.finished(now):
                        self._post(loop, OakResultsTopic, results)
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
        if isinstance(message, OakResultsMessage):
            now = time.monotonic()
            with self.arrivals_lock:
                self.recent_results[message.frame_number] = message
                while len(self.recent_results) > RECENT_RESULTS:
                    del self.recent_results[min(self.recent_results)]
            for measurement, count in (
                ("head_pose", sum(face.head_pose is not None for face in message.faces)),
                ("gaze", sum(face.gaze is not None for face in message.faces)),
                ("face_landmarks", sum(face.mesh is not None for face in message.faces)),
                ("hand_landmarks", len(message.hands or [])),
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
            input_size=YUNET_INPUT, score_threshold=self.params.models.score_threshold,
        )
        sequence = message.getSequenceNum()
        crops, finished = self.assembler.detected(sequence, message.getTimestamp().total_seconds(), faces, now)
        if crops:
            self.crop_requests.send(crop_request(dai, crops, sequence, self.params.width, self.params.height))
        for done in finished:
            self._post(loop, OakResultsTopic, done)

    def _on_face_results(self, dai, message, now: float, loop: asyncio.AbstractEventLoop) -> None:
        for done in self.assembler.answered(message.getSequenceNum(), now, results=message.getTensor(RESULT_LAYER)):
            self._post(loop, OakResultsTopic, done)

    def _on_face_landmarks(self, dai, message, now: float, loop: asyncio.AbstractEventLoop) -> None:
        for done in self.assembler.answered(message.getSequenceNum(), now, mesh=message.getFirstTensor()):
            self._post(loop, OakResultsTopic, done)

    def _on_palm_detection(self, dai, message, now: float, loop: asyncio.AbstractEventLoop) -> None:
        sequence = message.getSequenceNum()
        crops, finished = self.assembler.palms(
            sequence, message.getTensor("regressors"), message.getTensor("classificators"), now,
        )
        if crops:  # A second request for the same frame: the Script still keeps it
            self.crop_requests.send(crop_request(dai, crops, sequence, self.params.width, self.params.height))
        for done in finished:
            self._post(loop, OakResultsTopic, done)

    def _on_hand_landmarks(self, dai, message, now: float, loop: asyncio.AbstractEventLoop) -> None:
        presence, handedness = float(message.getTensor("score").ravel()[0]), float(message.getTensor("handedness").ravel()[0])
        for done in self.assembler.hand(message.getSequenceNum(), message.getTensor("keypoints"), presence, handedness, now):
            self._post(loop, OakResultsTopic, done)

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
        models = self.params.models
        logger.info(
            f"Starting OakCameraNode [{self.params.width}x{self.params.height} @ {self.params.fps:g} fps, "
            f"flip_camera={self.params.flip_camera}, jpeg_quality={self.params.jpeg_quality}, models="
            + ("off" if models is None else f"faces at {models.fps:g} fps, head pose 1 in {models.measure_every}, "
               f"landmarks 1 in {models.mesh_every}, hands 1 in {models.hands_every}") + "]"
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


def crop_request(dai, crops: list[Crop], sequence: int, width: int, height: int):
    """The Script node's request for one frame: a detection per crop, its label the crop's kind and its
    confidence the face's or hand's row, with the frame's sequence number. A hand's turned square
    travels as the detection's rotated box, normalized to the `width` x `height` frame."""
    request = dai.ImgDetections()
    detections = []
    for crop in crops:
        detection = dai.ImgDetection()
        if crop.kind == CropKind.HAND:
            hand = crop.hand
            detection.setBoundingBox(dai.RotatedRect(
                dai.Point2f(hand.center[0] / width, hand.center[1] / height),
                dai.Size2f(hand.side / width, hand.side / height), hand.angle))
        else:
            detection.xmin, detection.ymin, detection.xmax, detection.ymax = crop.box
        detection.label = int(crop.kind)
        detection.confidence = float(crop.row)
        detections.append(detection)
    request.detections = detections
    request.setSequenceNum(sequence)
    return request
