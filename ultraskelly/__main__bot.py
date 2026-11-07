"""
Target Tracker with Pose Estimation Integration

ROS2-inspired architecture: independent nodes + parameter system + launch config.
Tracks people with an OAK USB camera and YOLOv8 pose on the Hailo-8 AI HAT+.
"""
import asyncio
import logging
from ultraskelly.bot_launcher import LaunchConfig, BotLauncher
from ultraskelly.core.bot.motor.actuator_limits import ActuatorLimits
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.motor.roll_level import RollLevelParams
from ultraskelly.core.bot.motor.waist_drive import WaistDriveParams
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
    # 2026-11-06 (simulator/tuning.py) inside the servo windows measured that evening (pan 20-160, tilt
    # 20-130), and checked by the closed-loop scenarios (tests/test_scenarios_tuning.py), which run what
    # launches here. Head first, then the waist (the user, 2026-11-06): the waist stays braked and the roll
    # upright, so only pan and tilt follow.
    config.orientation = OrientationNodeParams(
        follow=follow, roll=RollLevelParams(enabled=False), waist=WaistDriveParams(enabled=False),
    )
    config.limits = ActuatorLimits(  # The same windows as hard limits on every command, motor tests included
        pan_min_degrees=20.0, pan_max_degrees=160.0, tilt_min_degrees=20.0, tilt_max_degrees=130.0,
    )
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