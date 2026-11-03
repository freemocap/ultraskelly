"""
Target Tracker with Pose Estimation Integration

ROS2-inspired architecture: independent nodes + parameter system + launch config.
Tracks people with an OAK USB camera and YOLOv8 pose on the Hailo-8 AI HAT+.
"""
import asyncio
import logging
from ultraskelly.core.bot.base_abcs import DetectorType
from ultraskelly.bot_launcher import LaunchConfig, BotLauncher
from ultraskelly.core.bot.recording.run_recorder_node import RunRecorderParams
from ultraskelly.core.bot.sensory.hailo_pose_node import HailoPoseDetectorParams
from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNodeParams
from ultraskelly.core.bot.sensory.pose_detection_node import PoseDetectorParams, CocoKeypoint
from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNodeParams
from ultraskelly.core.bot.sensory.zed_camera_node import ZedCameraNodeParams
from ultraskelly.core.bot.sensory.zed_imu_node import ZedImuNodeParams
logger = logging.getLogger(__name__)


def launch_config(
    *, simulate_motors: bool = False, show_window: bool = True, record: bool = False, microphone: str | None = None,
    zed: bool = False, zed_only: bool = False, depth: str | None = None, depth_previews: bool = False,
    port: int = 8080,
) -> LaunchConfig:
    """The robot's declarative launch configuration.

    Args:
        simulate_motors: Record motor commands instead of driving the motor boards.
        show_window: Show the OpenCV camera window (needs a desktop session).
        record: Save the camera video, detections and motor commands to disk.
        microphone: With `record`, also record audio from the sound card whose name contains this.
        zed: Also capture the ZED Mini stereo camera and stream its two eyes and its IMU.
        zed_only: Run only the ZED Mini and the dashboard, without tracking or motors.
        depth: With the ZED, also compute depth from its pair with this matcher (sgbm or stereonet).
        depth_previews: With depth, also stream it as colour images, besides the depth_frame messages.
        port: The dashboard's port.
    """

    # Example 1: Default pose tracking (nose), OAK camera + Hailo-8
    config = LaunchConfig(
        detector_type=DetectorType.HAILO_POSE,
        oak_camera=OakCameraNodeParams(flip_camera=True),  # The OAK's image arrives upside down
        hailo_pose_detector=HailoPoseDetectorParams(
            target_keypoint=CocoKeypoint.NOSE,
            detection_threshold=0.5,
            keypoint_threshold=0.4,
        ),
    )

    # Example : IMX500 AI camera on the Pi's camera connector (the original hardware)
    # config = LaunchConfig(
    #     detector_type=DetectorType.POSE,
    #     pose_detector=PoseDetectorParams(
    #         target_keypoint=CocoKeypoint.NOSE,
    #         detection_threshold=0.4,
    #         keypoint_threshold=0.4,
    #     ),
    # )

    # Example : Track left wrist
    # config = LaunchConfig(
    #     detector_type=DetectorType.HAILO_POSE,
    #     hailo_pose_detector=HailoPoseDetectorParams(
    #         target_keypoint=CocoKeypoint.LEFT_WRIST,
    #     ),
    # )

    # Example : Brightness detector
    # config = LaunchConfig(
    #     detector_type=DetectorType.BRIGHTNESS,
    #     brightness_detector=BrightnessDetectorParams(blur_size=21, threshold=150),
    #     motor=HeadNodeParams(gain=0.08, deadzone=20),
    # )

    config.simulate_motors = simulate_motors
    config.show_window = show_window
    config.recording = RunRecorderParams(microphone=microphone) if record else None
    config.zed_camera = ZedCameraNodeParams() if zed or zed_only else None
    config.zed_imu = ZedImuNodeParams() if zed or zed_only else None
    config.stereo_depth = StereoDepthNodeParams(matcher=depth) if depth else None
    config.runtime_server.depth_previews = depth_previews
    config.tracking = not zed_only
    config.runtime_server.port = port
    return config


async def main(
    *, simulate_motors: bool = False, show_window: bool = True, record: bool = False, microphone: str | None = None,
    zed: bool = False, zed_only: bool = False, depth: str | None = None, depth_previews: bool = False,
    port: int = 8080,
) -> None:
    """Launch with the configuration above; the options are `launch_config`'s."""
    launcher = BotLauncher.from_config(launch_config(
        simulate_motors=simulate_motors, show_window=show_window, record=record, microphone=microphone,
        zed=zed, zed_only=zed_only, depth=depth, depth_previews=depth_previews, port=port,
    ))
    await launcher.run()


if __name__ == "__main__":
    asyncio.run(main())