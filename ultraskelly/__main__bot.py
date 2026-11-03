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
from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNodeParams, OakFaceParams
from ultraskelly.core.bot.sensory.pose_detection_node import PoseDetectorParams, CocoKeypoint
from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNodeParams
from ultraskelly.core.bot.sensory.zed_camera_node import ZedCameraNodeParams
logger = logging.getLogger(__name__)


def launch_config(
    *, simulate_motors: bool = False, show_window: bool = True, record: bool = False, microphone: str | None = None,
    zed: bool = False, zed_only: bool = False, depth: str | None = None, port: int = 8080, oak_models: bool = True,
) -> LaunchConfig:
    """The robot's declarative launch configuration.

    Args:
        simulate_motors: Record motor commands instead of driving the motor boards.
        show_window: Show the OpenCV camera window (needs a desktop session).
        record: Save the camera video, detections and motor commands to disk.
        microphone: With `record`, also record audio from the sound card whose name contains this.
        zed: Also capture the ZED Mini stereo camera and stream its two eyes.
        zed_only: Run only the ZED Mini and the dashboard, without tracking or motors.
        depth: With the ZED, also compute depth from its pair with this matcher (sgbm or stereonet).
        port: The dashboard's port.
        oak_models: Run the face models on the OAK's own chip (face detection, head pose, gaze, face landmarks).
    """

    # Example 1: Default pose tracking (nose), OAK camera + Hailo-8
    config = LaunchConfig(
        detector_type=DetectorType.HAILO_POSE,
        oak_camera=OakCameraNodeParams(
            flip_camera=True,  # The OAK's image arrives upside down; its ISP turns it
            faces=OakFaceParams() if oak_models else None,
        ),
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
    config.stereo_depth = StereoDepthNodeParams(matcher=depth) if depth else None
    config.tracking = not zed_only
    config.runtime_server.port = port
    return config


async def main(
    *, simulate_motors: bool = False, show_window: bool = True, record: bool = False, microphone: str | None = None,
    zed: bool = False, zed_only: bool = False, depth: str | None = None, port: int = 8080, oak_models: bool = True,
) -> None:
    """Launch with the configuration above; the options are `launch_config`'s."""
    launcher = BotLauncher.from_config(launch_config(
        simulate_motors=simulate_motors, show_window=show_window, record=record, microphone=microphone,
        zed=zed, zed_only=zed_only, depth=depth, port=port, oak_models=oak_models,
    ))
    await launcher.run()


if __name__ == "__main__":
    asyncio.run(main())