import numpy as np
import pytest

from ultraskelly.core.bot.sensory.face_locator_node import (
    FaceLocatorNode,
    FaceLocatorParams,
    locate_along_ray,
    nominal_alignment,
)
from ultraskelly.core.bot.sensory.oak_zed_alignment import (
    OakZedAlignment,
    intrinsics_from_field_of_view,
    points_from_depth,
)
from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification
from ultraskelly.core.pubsub.bot_topics import (
    DepthFrameMessage,
    FrameMessage,
    PoseDataMessage,
    TargetLocationMessage,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

HALF_VGA = StereoRectification(width=336, height=188, focal_length=169.9, cx=165.3, cy=96.45, baseline=0.063,
                               left_rotation=((1, 0, 0), (0, 1, 0), (0, 0, 1)))


def scene(wall: float, ball: tuple[np.ndarray, float] | None = None) -> np.ndarray:
    """uint16 millimetre depth of a wall facing the camera, with an optional ball (centre, radius) before it."""
    columns, rows = np.meshgrid(np.arange(HALF_VGA.width), np.arange(HALF_VGA.height))
    rays = np.stack([(columns - HALF_VGA.cx) / HALF_VGA.focal_length,
                     (rows - HALF_VGA.cy) / HALF_VGA.focal_length, np.ones(columns.shape)], axis=-1)
    depth = np.full(columns.shape, wall)
    if ball is not None:
        centre, radius = ball
        a, b = (rays ** 2).sum(-1), -2 * (rays @ centre)
        c = centre @ centre - radius ** 2
        hit = b ** 2 - 4 * a * c >= 0
        t = np.where(hit, (-b - np.sqrt(np.maximum(b ** 2 - 4 * a * c, 0))) / (2 * a), np.inf)
        depth = np.where(hit & (t < depth), t, depth)
    return np.rint(depth * 1000).astype(np.uint16)


def test_oak_intrinsics_follow_from_its_field_of_view():
    matrix = intrinsics_from_field_of_view((640, 480), (69.0, 55.0))
    assert matrix[0, 0] == pytest.approx(465.6, abs=0.2) and matrix[1, 1] == pytest.approx(461.0, abs=0.2)
    assert (matrix[0, 2], matrix[1, 2]) == (319.5, 239.5)


def test_an_oak_pixel_becomes_a_ray_in_the_zed_frame_and_the_alignment_round_trips(tmp_path):
    alignment = nominal_alignment()
    origin, direction = alignment.ray((319.5, 239.5))
    assert origin == pytest.approx([0.0315, 0.035, 0.0]) and direction == pytest.approx([0, 0, 1])
    path = tmp_path / "alignment.json"
    alignment.save(path)
    again = OakZedAlignment.load(path)
    assert again.ray((100, 50))[1] == pytest.approx(alignment.ray((100, 50))[1])


def test_depth_pixels_become_points_in_the_zed_frame():
    depth = scene(2.0)
    points, known = points_from_depth(np.array([[HALF_VGA.cx, HALF_VGA.cy], [HALF_VGA.cx + 169.9, HALF_VGA.cy]]),
                                      depth, HALF_VGA)
    assert known.all()
    assert points[0] == pytest.approx([0, 0, 2.0], abs=1e-3)
    assert points[1] == pytest.approx([2.0, 0, 2.0], abs=2e-3)  # One focal length right of centre: x = z


def test_the_ray_stops_at_the_first_surface_it_passes_behind():
    origin, forward = np.array([0.0315, 0.035, 0.0]), np.array([0.0, 0.0, 1.0])
    wall_hit = locate_along_ray(origin, forward, scene(1.5), HALF_VGA)
    assert wall_hit == pytest.approx([0.0315, 0.035, 1.5], abs=0.01)
    # A ball (a head) 1 m ahead, in front of a wall 3 m away: the ray meets the ball's near side.
    ball_hit = locate_along_ray(origin, forward, scene(3.0, (np.array([0.0315, 0.035, 1.0]), 0.09)), HALF_VGA)
    assert ball_hit[2] == pytest.approx(0.91, abs=0.02)
    assert locate_along_ray(origin, forward, np.zeros((188, 336), np.uint16), HALF_VGA) is None  # No depth anywhere
    # A ray that never crosses the depth image (2026-11-06 it stopped the bot): no point, no error.
    assert locate_along_ray(origin, np.array([1.0, 0.0, 0.0]), scene(1.5), HALF_VGA) is None


def test_the_oak_pose_is_recovered_from_points_and_their_oak_pixels():
    cv2 = pytest.importorskip("cv2")
    from ultraskelly.core.bot.sensory.oak_zed_alignment import fit_alignment

    rng = np.random.default_rng(1)
    matrix = intrinsics_from_field_of_view((640, 480), (69.0, 55.0))
    rotation, _ = cv2.Rodrigues(np.array([0.02, -0.03, 0.01]))  # OAK to ZED
    translation = np.array([0.03, 0.04, -0.01])
    points = np.stack([rng.uniform(-1, 1, 200), rng.uniform(-0.6, 0.6, 200), rng.uniform(0.8, 3.0, 200)], axis=1)
    in_oak = (points - translation) @ rotation  # rotation.T @ (point - translation), row by row
    pixels = (in_oak @ matrix.T)[:, :2] / in_oak[:, 2:]
    pixels += rng.normal(0, 0.5, pixels.shape)
    pixels[:20] += rng.uniform(-80, 80, (20, 2))  # Some mismatches

    alignment = fit_alignment(points, pixels, matrix, (640, 480))

    assert alignment.translation == pytest.approx(translation, abs=0.01)
    assert alignment.rotation == pytest.approx(rotation, abs=0.005)
    assert alignment.error_px < 1.0 and alignment.points >= 170


def test_the_node_places_the_face_in_the_head_frame_from_recent_depth_only():
    bus = PubSubTopicManager.create()
    node = FaceLocatorNode.create(pubsub=bus, params=FaceLocatorParams(alignment=None))
    node.alignment, node.alignment_source = nominal_alignment(), "nominal"
    node.latest_depth = DepthFrameMessage(depth=scene(1.2), color=np.zeros((188, 336, 3), np.uint8),
                                          rectification=HALF_VGA, frame_number=5, timestamp=10.0)
    frame = FrameMessage(frame=np.zeros((480, 640, 3), np.uint8), frame_number=9, timestamp=10.05)
    pose = PoseDataMessage(frame=frame, target=TargetLocationMessage(x=320, y=240))

    face = node.locate(pose)
    stale = node.locate(PoseDataMessage(frame=frame.model_copy(update={"timestamp": 11.0}), target=pose.target))
    nobody = node.locate(PoseDataMessage(frame=frame, target=TargetLocationMessage()))
    untimed = node.locate(PoseDataMessage(frame=frame.model_copy(update={"timestamp": None}), target=pose.target))
    no_frame = node.locate(PoseDataMessage(target=pose.target))  # A detector that doesn't say which image
    other_size = node.locate(PoseDataMessage(
        frame=frame.model_copy(update={"frame": np.zeros((720, 1280, 3), np.uint8)}), target=pose.target,
    ))
    bus.close()

    assert face.position_camera[2] == pytest.approx(1.2, abs=0.01)
    forward, left, up = face.position_head
    assert forward == pytest.approx(1.2, abs=0.01)  # Ahead of the face
    assert left == pytest.approx(0.0315 - 0.0315, abs=0.01)  # The OAK sits between the eyes
    assert up == pytest.approx(-0.035, abs=0.01)  # And a little below them
    assert face.distance == pytest.approx(np.linalg.norm(face.position_head))
    assert (face.pixel, face.pose_timestamp, face.depth_timestamp, face.alignment) == ((320, 240), 10.05, 10.0, "nominal")
    assert stale.distance is None and nobody.distance is None
    assert untimed.distance is None and no_frame.distance is None and other_size.distance is None
    assert node.size_warned


def test_robot_state_reports_the_newest_face():
    import asyncio

    from ultraskelly.core.pubsub.bot_topics import FaceLocationMessage, FaceLocationTopic
    from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams

    async def scenario():
        bus = PubSubTopicManager.create()
        server = RuntimeServerNode.create(pubsub=bus, params=RuntimeServerParams(), cameras={}, recorder=None,
                                          motors_simulated=True, node_status=lambda: [])
        pump = asyncio.create_task(server.pump())
        await bus.publish(FaceLocationTopic, FaceLocationMessage(
            position_camera=(0.03, 0.0, 1.2), position_head=(1.2, 0.0, -0.035), distance=1.2005,
            pixel=(320, 240), pose_timestamp=1.0, depth_timestamp=0.95, alignment="nominal"))
        await asyncio.sleep(0.05)
        state = server.robot_state()
        server.stop()
        await pump
        bus.close()
        return state

    face = asyncio.run(scenario())["face"]
    assert face["position_head"] == [1.2, 0.0, -0.035] and face["distance"] == 1.2005
    assert (face["pixel"], face["alignment"]) == ([320, 240], "nominal") and face["age"] >= 0
