import asyncio
import logging
import time

import cv2
import numpy as np
from pydantic import Field, SkipValidation

from ultraskelly.core.pubsub.bot_topics import (
    FrameMessage,
    FrameTopic,
    PoseDataMessage,
    PoseDataTopic,
    ServoStateMessage,
    ServoStateTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_abcs import TopicSubscriptionQueue

logger = logging.getLogger(__name__)
from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.sensory.pose_common import SKELETON_CONNECTIONS
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


class UINodeParams(NodeParams):
    """Parameters for UINode."""

    deadzone: int = Field(default=30, ge=0)
    window_name: str = Field(default="Async Target Tracker")


class UINode(Node):
    """Displays camera feed with tracking visualization."""

    params: UINodeParams

    # Queue subscriptions
    frame_queue: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)
    target_queue: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)
    servo_state_queue: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)
    pose_data_queue: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)

    # Latest state from queues
    latest_target: TargetLocationMessage | None = Field(default=None, exclude=True)
    latest_servo_state: ServoStateMessage | None = Field(default=None, exclude=True)
    latest_pose_data: PoseDataMessage | None = Field(default=None, exclude=True)

    # FPS tracking
    frame_count: int = Field(default=0, exclude=True)
    last_fps_time: float = Field(default_factory=time.time, exclude=True)
    fps: float = Field(default=0.0, exclude=True)

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: UINodeParams) -> "UINode":
        """Factory method to create and initialize UINode."""
        node = cls(pubsub=pubsub, params=params)

        # Subscribe to all necessary topics
        node.frame_queue = pubsub.topics[FrameTopic].get_subscription()
        node.target_queue = pubsub.topics[TargetLocationTopic].get_subscription()
        node.servo_state_queue = pubsub.topics[ServoStateTopic].get_subscription()
        node.pose_data_queue = pubsub.topics[PoseDataTopic].get_subscription()

        return node

    def _draw_skeleton(
        self, *, frame: np.ndarray, keypoints: np.ndarray, confidence_threshold: float = 0.3
    ) -> None:
        """Draw skeleton on frame for a single person."""
        # Draw bones
        for kp1, kp2 in SKELETON_CONNECTIONS:
            pt1 = keypoints[kp1]
            pt2 = keypoints[kp2]

            # Only draw if both keypoints are confident
            if pt1[2] > confidence_threshold and pt2[2] > confidence_threshold:
                x1, y1 = int(pt1[0]), int(pt1[1])
                x2, y2 = int(pt2[0]), int(pt2[1])
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

        # Draw keypoints on top of bones
        for i, kp in enumerate(keypoints):
            if kp[2] > confidence_threshold:
                x, y = int(kp[0]), int(kp[1])
                # Different color for different body parts
                if i <= 4:  # Head
                    color = (255, 0, 0)  # Blue
                elif i <= 10:  # Arms
                    color = (0, 255, 0)  # Green
                else:  # Legs
                    color = (0, 0, 255)  # Red
                cv2.circle(frame, (x, y), 5, color, -1)
                cv2.circle(frame, (x, y), 6, (255, 255, 255), 1)

    def _draw_visualization(self, *, frame: np.ndarray) -> np.ndarray:
        """Draw tracking visualization on frame."""
        h, w = frame.shape[:2]
        center_x = w // 2
        center_y = h // 2

        # Draw all detected skeletons
        if self.latest_pose_data and self.latest_pose_data.keypoints is not None:
            for person_keypoints in self.latest_pose_data.keypoints:
                self._draw_skeleton(
                    frame=frame, keypoints=person_keypoints, confidence_threshold=0.3
                )

        # Center crosshair
        cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 2)
        cv2.line(frame, (0, center_y), (w, center_y), (255, 255, 255), 2)

        # Deadzone box
        cv2.rectangle(
            frame,
            (center_x - self.params.deadzone, center_y - self.params.deadzone),
            (center_x + self.params.deadzone, center_y + self.params.deadzone),
            (128, 128, 128),
            1,
        )

        # Target point
        if self.latest_target and self.latest_target.x is not None:
            x = self.latest_target.x
            y = self.latest_target.y

            is_locked = (
                self.latest_servo_state.is_locked_x
                and self.latest_servo_state.is_locked_y
                and self.latest_servo_state.is_locked_roll
                if self.latest_servo_state
                else False
            )

            color = (0, 255, 0) if is_locked else (255, 0, 0)
            cv2.circle(frame, (x, y), 20, color, 3)

            # Draw orientation line if angle is available
            if self.latest_target.angle is not None:
                angle_rad = np.radians(self.latest_target.angle)
                line_length = 40
                end_x = int(x + line_length * np.sin(angle_rad))
                end_y = int(y - line_length * np.cos(angle_rad))

                roll_color = (
                    (0, 255, 0)
                    if (self.latest_servo_state and self.latest_servo_state.is_locked_roll)
                    else (255, 0, 255)
                )
                cv2.line(frame, (x, y), (end_x, end_y), roll_color, 3)

            if self.latest_servo_state:
                line_x_color = (
                    (0, 255, 0) if self.latest_servo_state.is_locked_x else (255, 255, 0)
                )
                line_y_color = (
                    (0, 255, 0) if self.latest_servo_state.is_locked_y else (255, 255, 0)
                )
                cv2.line(frame, (center_x, y), (x, y), line_x_color, 2)
                cv2.line(frame, (x, center_y), (x, y), line_y_color, 2)

        # Status overlay
        if self.latest_servo_state:
            status = (
                "LOCKED"
                if (
                    self.latest_servo_state.is_locked_x
                    and self.latest_servo_state.is_locked_y
                    and self.latest_servo_state.is_locked_roll
                )
                else "TRACKING"
            )
            status_color = (0, 255, 0) if status == "LOCKED" else (255, 255, 0)

            cv2.putText(
                frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2
            )
            cv2.putText(
                frame,
                f"Pan: {self.latest_servo_state.pan_angle:.1f}° - x-locked: {self.latest_servo_state.is_locked_x}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                frame,
                f"Tilt: {self.latest_servo_state.tilt_angle:.1f}° - y-locked: {self.latest_servo_state.is_locked_y}",
                (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                frame,
                f"Roll: {self.latest_servo_state.roll_angle:.1f}°",
                (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
            )

        # Target angle display
        if self.latest_target and self.latest_target.angle is not None:
            cv2.putText(
                frame,
                f"Body Angle: {self.latest_target.angle:.1f}°",
                (10, 135),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 255),
                1,
            )

        cv2.putText(
            frame,
            f"FPS: {self.fps:.1f}",
            (10, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

        return frame

    async def _update_state(self) -> None:
        """Background task to consume queue updates."""
        while not self.stop_event.is_set():
            try:
                # Drain queues to get latest values
                while not self.target_queue.empty():
                    try:
                        self.latest_target = self.target_queue.get_nowait()
                    except asyncio.QueueEmpty:
                        break

                while not self.servo_state_queue.empty():
                    try:
                        self.latest_servo_state = self.servo_state_queue.get_nowait()
                    except asyncio.QueueEmpty:
                        break

                while not self.pose_data_queue.empty():
                    try:
                        self.latest_pose_data = self.pose_data_queue.get_nowait()
                    except asyncio.QueueEmpty:
                        break

                await asyncio.sleep(0.001)
            except Exception as e:
                logger.error(f"Error in UI update task: {e}")
                raise

    async def run(self) -> None:
        """Main UI loop."""
        logger.info(f"Starting UINode [window='{self.params.window_name}']")

        # Start update task
        update_task = asyncio.create_task(self._update_state())

        try:
            while not self.stop_event.is_set():
                try:
                    if not self.frame_queue.empty():
                        frame_msg: FrameMessage = await  self.frame_queue.get()
                        vis_frame = self._draw_visualization(frame=frame_msg.frame)

                        self.frame_count += 1
                        if time.time() - self.last_fps_time >= 1.0:
                            self.fps = self.frame_count / (time.time() - self.last_fps_time)
                            self.frame_count = 0
                            self.last_fps_time = time.time()

                        cv2.imshow(
                            self.params.window_name, cv2.cvtColor(vis_frame, cv2.COLOR_RGB2BGR)
                        )

                    # Non-blocking key check
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.stop_event.set()
                        break

                    await asyncio.sleep(0.001)
                except asyncio.TimeoutError:
                    continue
        finally:
            self.stop_event.set()
            update_task.cancel()
            try:
                await update_task
            except asyncio.CancelledError:
                pass
            cv2.destroyAllWindows()
            logger.info("UINode stopped")