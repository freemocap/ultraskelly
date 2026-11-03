"""The program the OAK's Script node runs for its second-stage models, on the OAK's own CPU.

For each crop request from the Pi (an ImgDetections with the frame's sequence number: one
detection per crop, its label the `oak_faces.CropKind`, its confidence the face's or hand's
row), it takes that frame from the newest few it keeps, sends each crop and the frame to the
ImageManip in front of the crop's network, passes every face's head pose to the gaze network
with that face's eyes, and answers each request with faces in it with one NNData: per face,
its row, yaw, pitch, roll and gaze x, y, z. Face landmark, palm and hand crops go to their
networks, whose outputs go straight to the Pi; a frame's hand crops come in a later request,
once the Pi has found its palms. Everything keeps the frame's sequence number, so an answer
that comes too late is recognized and dropped instead of being taken for the next frame's.
Within a frame the answers look alike, so if one does not come in time the whole frame goes
unmeasured (NaN) rather than misplaced.

Crops are cut in pixels: the OAK's ImageManip took normalized rotated rectangles in a way
that did not match their documentation (docs/audit-2026.md, "The OAK's own chip").

No hardware imports: tests run the program against stand-ins (tests/test_oak_script.py).
"""

GAZE_ANGLES_INPUT = "head_pose_angles_yaw_pitch_roll"
RESULT_LAYER = "faces"
RESULT_FIELDS = 7  # Row, yaw, pitch, roll, gaze x, y, z
KEEP_FRAMES = 4  # The newest frames the Script keeps for requests, including a frame's later hand crops

SCRIPT = r"""
import time

# CropKind values (oak_faces.py): which ImageManip each crop goes to, and its network's input size.
OUTPUTS = {0: "head_pose", 1: "left_eye", 2: "right_eye", 3: "mesh", 4: "palm", 5: "hand"}
SIZES = {0: (60, 60), 1: (60, 60), 2: (60, 60), 3: (192, 192), 4: (128, 128), 5: (224, 224)}
WAIT = 0.25  # Seconds to wait for a network's answer before giving up on it
KEEP = 4  # Frames kept for requests (oak_script.KEEP_FRAMES)

held = {}  # An answer for a later frame, read while waiting for an earlier one's
recent = []  # The newest frames, oldest first


def answer(name, sequence):
    # The next output of `name` for this frame; None after WAIT. Older frames' outputs are dropped.
    deadline = time.time() + WAIT
    while time.time() < deadline:
        message = held.pop(name, None) or node.inputs[name].tryGet()
        if message is None:
            time.sleep(0.001)
        elif message.getSequenceNum() == sequence:
            return message
        elif message.getSequenceNum() > sequence:
            held[name] = message  # This frame's answer was lost; keep the later one for its own frame
            return None
    return None


def frame_for(sequence):
    # The frame with this number, reading frames until it or a later one comes; None once it is gone.
    while not recent or recent[-1].getSequenceNum() < sequence:
        recent.append(node.inputs["frame"].get())
        if len(recent) > KEEP:
            recent.pop(0)
    for frame in recent:
        if frame.getSequenceNum() == sequence:
            return frame
    return None


def configure(crop, width, height):
    config = ImageManipConfig()
    size = SIZES[crop.label]
    if crop.label == 4:  # The whole frame, letterboxed
        config.setOutputSize(size[0], size[1], ImageManipConfig.ResizeMode.LETTERBOX)
    elif crop.label == 5:  # A turned square, sent normalized and cut in pixels
        rect = crop.getBoundingBox()
        config.addCropRotatedRect(RotatedRect(
            Point2f(rect.center.x * width, rect.center.y * height),
            Size2f(rect.size.width * width, rect.size.height * height), rect.angle), False)
        config.setOutputSize(size[0], size[1], ImageManipConfig.ResizeMode.STRETCH)
    else:
        x, y = int(round(crop.xmin * width)), int(round(crop.ymin * height))
        w, h = max(2, int(round((crop.xmax - crop.xmin) * width))), max(2, int(round((crop.ymax - crop.ymin) * height)))
        config.addCrop(x, y, w, h)
        config.setOutputSize(size[0], size[1], ImageManipConfig.ResizeMode.STRETCH)
    config.setFrameType(ImgFrame.Type.BGR888p)
    return config


while True:
    request = node.inputs["crops"].get()
    sequence = request.getSequenceNum()
    frame = frame_for(sequence)
    if frame is None:
        continue  # That frame is gone; the Pi sees no answer and moves on
    width, height = frame.getWidth(), frame.getHeight()
    faces = []
    for crop in request.detections:
        node.outputs[OUTPUTS[crop.label] + "_config"].send(configure(crop, width, height))
        node.outputs[OUTPUTS[crop.label] + "_image"].send(frame)
        if crop.label == 0:
            faces.append(int(crop.confidence))
    # The answers for one frame look alike, so with one missing none can be placed: the frame
    # then goes unmeasured. Every face's eyes still reach the gaze network, so every face
    # still sends it angles, which keeps the gaze network's three inputs in step.
    poses = []
    for face in faces:
        pose = answer("head_pose", sequence)
        poses.append(None if pose is None else [pose.getLayerFp16(name)[0] for name in ("tf.identity", "tf.identity_1", "tf.identity_2")])
    if None in poses:
        poses = [None] * len(faces)
    for found in poses:
        message = NNData(64)
        message.setLayer("head_pose_angles_yaw_pitch_roll", found if found is not None else [0.0, 0.0, 0.0])
        message.setSequenceNum(sequence)
        node.outputs["angles"].send(message)
    gazes = [answer("gaze", sequence) for face in faces]
    if None in gazes or None in poses:
        gazes = [None] * len(faces)
    values = []
    nan = float("nan")
    for face, found, gaze in zip(faces, poses, gazes):
        direction = gaze.getLayerFp16("Identity0")[:3] if gaze is not None else [nan, nan, nan]
        values += [face] + list(found if found is not None else [nan, nan, nan]) + list(direction)
    if faces:  # A request with no head pose crops has nothing to report here
        results = NNData(64 + 8 * len(values))
        results.setLayer("faces", values)
        results.setSequenceNum(sequence)
        results.setTimestamp(frame.getTimestamp())
        node.outputs["results"].send(results)
"""
