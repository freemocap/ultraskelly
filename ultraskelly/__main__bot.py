"""
Target Tracker with Pose Estimation Integration

ROS2-inspired architecture: independent nodes + parameter system + launch config.
Tracks people with an OAK USB camera and YOLOv8 pose on the Hailo-8 AI HAT+.
"""
import asyncio
import logging
from ultraskelly.core.bot.base_abcs import DetectorType
from ultraskelly.bot_launcher import LaunchConfig, BotLauncher
from ultraskelly.core.bot.motor.actuator_limits import ActuatorLimits
from ultraskelly.core.bot.motor.head_aim import HeadAimParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.motor.roll_level import RollLevelParams
from ultraskelly.core.bot.motor.waist_drive import WaistDriveParams
from ultraskelly.core.bot.recording.run_recorder_node import RunRecorderParams
from ultraskelly.core.bot.sensory.hailo_pose_node import HailoPoseDetectorParams
from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNodeParams, OakModelParams
from ultraskelly.core.bot.sensory.pose_detection_node import PoseDetectorParams, CocoKeypoint
from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNodeParams
from ultraskelly.core.bot.sensory.zed_camera_node import ZedCameraNodeParams
from ultraskelly.core.bot.sensory.zed_imu_node import ZedImuNodeParams
logger = logging.getLogger(__name__)


def launch_config(
    *, simulate_motors: bool = False, show_window: bool = False, record: bool = False, microphone: str | None = None,
    zed: bool = False, zed_only: bool = False, depth: str | None = None, depth_previews: bool = False,
    port: int = 8080, oak_models: bool = True, follow: bool = True,
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
        oak_models: Run the face and hand models on the OAK's own chip (faces, head pose, gaze, face landmarks, hands).
        follow: Turn the head and waist toward the target; without, they move only for motor tests.
    """

    # Example 1: Default pose tracking (nose), OAK camera + Hailo-8
    config = LaunchConfig(
        detector_type=DetectorType.HAILO_POSE,
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
    # The head, roll and waist run their defaults: fitted to the gearing, slack and direction measured on the
    # robot on 2026-11-06 and checked by the closed-loop scenarios (tests/test_scenarios.py), which run the
    # defaults, so nothing is overridden here.
    # Head-only tuning, 2026-11-06 (the user: disable the waist and tune the head's face centering first, then
    # work the waist back in): the waist stays braked and the roll upright, so only pan and tilt follow.
    config.orientation = OrientationNodeParams(
        follow=follow, roll=RollLevelParams(enabled=False), waist=WaistDriveParams(enabled=False),
        # Lower head gain (2026-11-06, the user: "Swings past me", "gain tooo high"). Recorded run
        # 20261106-194104: one approach went from -297 px to +150 px past the middle; 447 px of image for
        # 85 servo degrees is about 0.65 degrees of view per servo degree on large moves, twice pan_ratio,
        # so g = 2 and each sample's error factor 1 - reach * g = -0.3 (an overshoot every sample). With
        # reach 0.35 it stays >= 0 up to g = 2.86; the simulator then shows no overshoot even with 0.15 s of
        # servo lag (tests/test_scenarios_tuning.py), at the cost of slower aiming.
        # Servo windows from full sweeps on the robot (scripts/check_pan_travel.py, 2026-11-06 20:09-20:11):
        # pan turns the view steadily from 10 to 170 at 0.27 degrees per servo degree, but jammed for 70
        # degrees after being held at 5, then snapped; tilt turns it steadily from 10 to ~140 (only 3.4
        # degrees below rest), and past ~140 its linkage goes over centre: further servo turns the view
        # back, then not at all. So pan keeps to 20-160 and tilt to 20-130.
        aim=HeadAimParams(
            reach=0.35, smoothing=0.3, max_speed=15.0, pan_ratio=0.27, pan_range=(20.0, 160.0), tilt_range=(20.0, 130.0),
        ),
    )
    config.limits = ActuatorLimits(  # The same windows as hard limits on every command, motor tests included
        pan_min_degrees=20.0, pan_max_degrees=160.0, tilt_min_degrees=20.0, tilt_max_degrees=130.0,
    )
    return config


async def main(
    *, simulate_motors: bool = False, show_window: bool = False, record: bool = False, microphone: str | None = None,
    zed: bool = False, zed_only: bool = False, depth: str | None = None, depth_previews: bool = False,
    port: int = 8080, oak_models: bool = True, follow: bool = True,
) -> None:
    """Launch with the configuration above; the options are `launch_config`'s."""
    launcher = BotLauncher.from_config(launch_config(
        simulate_motors=simulate_motors, show_window=show_window, record=record, microphone=microphone,
        zed=zed, zed_only=zed_only, depth=depth, depth_previews=depth_previews, port=port, oak_models=oak_models,
        follow=follow,
    ))
    await launcher.run()


if __name__ == "__main__":
    asyncio.run(main())