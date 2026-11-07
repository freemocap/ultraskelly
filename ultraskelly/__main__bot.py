"""
Target Tracker with Pose Estimation Integration

ROS2-inspired architecture: independent nodes + parameter system + launch config.
Tracks people with an OAK USB camera and YOLOv8 pose on the Hailo-8 AI HAT+.
"""
import asyncio
import logging
from ultraskelly.bot_launcher import LaunchConfig, BotLauncher
from ultraskelly.core.bot.motor.head_aim import HeadAimParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.recording.run_recorder_node import RunRecorderParams
from ultraskelly.core.bot.sensory.hailo_pose_node import HailoPoseDetectorParams
from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNodeParams, OakModelParams
from ultraskelly.core.bot.sensory.pose_common import CocoKeypoint
from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNodeParams
from ultraskelly.core.bot.sensory.zed_camera_node import ZedCameraNodeParams
from ultraskelly.core.bot.sensory.zed_imu_node import ZedImuNodeParams
logger = logging.getLogger(__name__)


def launch_config(
    *, simulate_motors: bool = False, record: bool = False, microphone: str | None = None,
    zed: bool = False, zed_only: bool = False, depth: str | None = None, depth_previews: bool = False,
    port: int = 8080, oak_models: bool = True, follow: bool = True,
) -> LaunchConfig:
    """The robot's declarative launch configuration.

    Args:
        simulate_motors: Record motor commands instead of driving the motor boards.
        record: Save the camera video, detections and motor commands to disk.
        microphone: With `record`, also record audio from the sound card whose name contains this.
        zed: Also capture the ZED Mini stereo camera and stream its two eyes and its IMU.
        zed_only: Run only the ZED Mini and the dashboard, without tracking or motors.
        depth: With the ZED, also compute depth from its pair with this matcher (sgbm or stereonet).
        depth_previews: With depth, also stream it as colour images, besides the depth_frame messages.
        port: The dashboard's port.
        oak_models: Run the face and hand models on the OAK's own chip (faces, head pose, gaze, face landmarks, hands).
        follow: Turn the head and waist toward the target; without, they move only for motor tests.
    """

    # Pose tracking (nose): OAK camera + Hailo-8
    config = LaunchConfig(
        oak_camera=OakCameraNodeParams(
            flip_camera=True,  # The OAK sits upside down; its sensor reads the image out turned
            models=OakModelParams() if oak_models else None,
        ),
        hailo_pose_detector=HailoPoseDetectorParams(
            target_keypoint=CocoKeypoint.NOSE,
            detection_threshold=0.5,
            keypoint_threshold=0.4,
        ),
    )

    config.simulate_motors = simulate_motors
    config.recording = RunRecorderParams(microphone=microphone) if record else None
    config.zed_camera = ZedCameraNodeParams() if zed or zed_only else None
    config.zed_imu = ZedImuNodeParams() if zed or zed_only else None
    config.stereo_depth = StereoDepthNodeParams(matcher=depth) if depth else None
    config.runtime_server.depth_previews = depth_previews
    config.tracking = not zed_only
    config.runtime_server.port = port
    # The head runs its defaults: its aim tuned on the simulated robot fitted to the motors recorded on
    # 2026-11-06 (simulator/tuning.py), and checked by the closed-loop scenarios (tests/test_scenarios_tuning.py),
    # which run what launches here. It follows over the servos' whole measured travel (pan 10-170, tilt 10-140;
    # the owner, 2026-11-07: "head servos should move their FULL range of motion"), and the hard limits are the
    # servos' own 0-180, so motor tests reach every angle.
    # Two changes from the tuned head (the owner, 2026-11-07): a gentler acceleration, 25 degrees a second squared
    # instead of 34, at full speed ("I just need a less aggressive acceleration"; a 30 degree a second cap on pan's
    # servo was "way too slow"), and a deadband of 0.12, so a still head moves again for a face about 0.4 degrees off
    # (deadband / reach), 3 px, where 0.39 waited for 1.3, 10 px ("tighten that range"). The head battery passes
    # with both, no new reversals, median settle 1.67 s against 1.53 (tests/test_scenarios_tuning.py).
    # The waist follows too, untuned with this head (the owner, 2026-11-07: "Turn on as is"): in the full scenarios
    # it swings 14-20 degrees past someone far to the side, and it starts a move only once the head has held still
    # for 0.3 s, which the tighter deadband makes rarer. The roll levels the followed face (the owner, 2026-11-07:
    # "start the face leveling head roll support work"): on the simulated robot it straightens a tilted face only
    # about 1.5 degrees, all its linkage gives within `span` (tests/test_scenarios.py, the "tilt" scenarios).
    config.orientation = OrientationNodeParams(follow=follow, aim=HeadAimParams())
    return config


async def main(
    *, simulate_motors: bool = False, record: bool = False, microphone: str | None = None,
    zed: bool = False, zed_only: bool = False, depth: str | None = None, depth_previews: bool = False,
    port: int = 8080, oak_models: bool = True, follow: bool = True,
) -> None:
    """Launch with the configuration above; the options are `launch_config`'s."""
    launcher = BotLauncher.from_config(launch_config(
        simulate_motors=simulate_motors, record=record, microphone=microphone,
        zed=zed, zed_only=zed_only, depth=depth, depth_previews=depth_previews, port=port, oak_models=oak_models,
        follow=follow,
    ))
    await launcher.run()


if __name__ == "__main__":
    asyncio.run(main())