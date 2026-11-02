"""
Target Tracker with Pose Estimation Integration

ROS2-inspired architecture: independent nodes + parameter system + launch config.
Tracks people with an OAK USB camera and YOLOv8 pose on the Hailo-8 AI HAT+.
"""
import asyncio
import logging
from ultraskelly.core.bot.base_abcs import DetectorType
from ultraskelly.bot_launcher import LaunchConfig, BotLauncher
from ultraskelly.core.bot.sensory.hailo_pose_node import HailoPoseDetectorParams
from ultraskelly.core.bot.sensory.pose_detection_node import PoseDetectorParams, CocoKeypoint
logger = logging.getLogger(__name__)


async def main(*, simulate_motors: bool = False) -> None:
    """Launch with declarative config.

    Args:
        simulate_motors: Record motor commands instead of driving the motor boards.
    """

    # Example 1: Default pose tracking (nose), OAK camera + Hailo-8
    config = LaunchConfig(
        detector_type=DetectorType.HAILO_POSE,
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
    launcher = BotLauncher.from_config(config)
    await launcher.run()


if __name__ == "__main__":
    asyncio.run(main())