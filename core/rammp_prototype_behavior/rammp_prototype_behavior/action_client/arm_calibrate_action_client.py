from __future__ import annotations
import asyncio

from rclpy.node import Node
from arm_interfaces.action import Calibrate
from .action_client_wrapper import ActionClientWrapper


class ArmCalibrateActionClient(ActionClientWrapper):
    def __init__(
        self,
        node: Node,
    ):
        super().__init__(
            "/arm/calibrate",
            Calibrate,
            self.goal_callback,
            self.result_callback,
            self.cancel_callback,
            node,
            feedback_callback=self.feedback_callback,
        )

    def feedback_callback(self, feedback: Calibrate.Feedback):
        # publisher = getattr(self._node, "_calibrate_progress_publisher", None)
        # if publisher is not None:
        #     progress_msg = Float32()
        #     progress_msg.data = feedback.progress
        #     publisher.publish(progress_msg)
        self._node.get_logger().debug(f"Calibration feedback: {feedback}")

    def goal_callback(self, success: bool):
        if success:
            self._node.get_logger().info(
                "Goal Calibration accepted by the action server."
            )
        else:
            self._node.get_logger().warn(
                "Goal Calibration rejected by the action server."
            )
            self._node.calibrationFailed()

    def result_callback(self, success: bool):
        if success:
            self._node.get_logger().info("Successfully completed Calibration.")
            self._node.calibrationComplete()
            if hasattr(self._node, "_arm_calibrated"):
                self._node._arm_calibrated = True
        else:
            self._node.get_logger().warn("Failed to complete Calibration.")
            self._node.calibrationFailed()

    def cancel_callback(self, success: bool):
        if success:
            self._node.get_logger().info(
                "Goal cancellation accepted by the action server."
            )
            self._node.calibrationFailed()
        else:
            self._node.get_logger().warn(
                "Goal cancellation rejected by the action server."
            )
            self._node.calibrationFailed()

    def send_goal(self):
        goal = Calibrate.Goal()
        asyncio.run_coroutine_threadsafe(super().send_goal(goal), self._node._loop)

    def cancel(self):
        if not self.is_action_running():
            # do nothing if no action is currently running
            return
        asyncio.run_coroutine_threadsafe(super().cancel_goal(), self._node._loop)
