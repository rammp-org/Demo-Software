from __future__ import annotations
from typing import Optional
import asyncio
import enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from arm_interfaces.action import ReachPreset


class ArmPreset(enum.IntEnum):
    HOME = ReachPreset.Goal.PRESET_HOME
    RETRACT = ReachPreset.Goal.PRESET_RETRACT
    ZERO = ReachPreset.Goal.PRESET_ZERO
    CUP_STABILIZE = ReachPreset.Goal.PRESET_CUP_STABILIZE


class ArmPresetActionClient:
    def __init__(self, node: Node):
        self._node = node
        self._action_name = "/arm/reach_preset"
        self._client: ActionClient = ActionClient(node, ReachPreset, self._action_name)
        self._action_running = False
        self.gh = None

    async def wait_for_server(self, timeout: Optional[float] = None) -> bool:
        """
        Wait for the action server to be available.

        :param timeout: Optional timeout in seconds. If None, wait indefinitely.
        :return: True if the server is available, False if timed out.
        """
        try:
            await self._client.wait_for_server(timeout_sec=timeout)
            self._node.get_logger().info(
                f"Action server '{self._action_name}' is available."
            )
            return True
        except asyncio.TimeoutError:
            self._node.get_logger().warn(
                f"Timed out waiting for action server '{self._action_name}'."
            )
            return False

    def is_server_ready(self) -> bool:
        return self._client.server_is_ready()

    async def send_goal(self, arm_preset: ArmPreset):
        if not self.is_server_ready():
            raise RuntimeError(f"Action server '{self._action_name}' is not ready.")
        if self.gh is not None:
            raise RuntimeError(
                "A goal is already active. Please wait for it to complete or cancel it before sending a new one."
            )

        self._action_running = True
        self.gh = None
        goal_msg = ReachPreset.Goal()
        goal_msg.preset = arm_preset.value

        send_future: Future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._send_goal_done_callback)

    def _send_goal_done_callback(self, future: Future):
        try:
            goal_handle: ClientGoalHandle = future.result()
            if not goal_handle.accepted:
                self._node.get_logger().warn("Goal was rejected by the action server.")
                self._action_running = False
                return
            self._node.get_logger().info("Goal accepted by the action server.")
            self.gh = goal_handle

            res_future = self.gh.get_result_async()
            res_future.add_done_callback(self._on_result)

        except Exception as e:
            self._node.get_logger().error(f"Exception while sending goal: {e}")
            self._action_running = False

    def _on_result(self, future: Future):
        try:
            result = future.result().result
            self._node.get_logger().info(f"Action completed with result: {result}")
            if result.success:
                self._node.get_logger().info("Arm reached preset successfully.")
            else:
                self._node.get_logger().warn("Arm failed to reach preset.")
        except Exception as e:
            self._node.get_logger().error(f"Exception while getting result: {e}")
        finally:
            self._action_running = False
            self.gh = None

    def _feedback_callback(self, feedback_msg: ReachPreset.Feedback):
        feedback = feedback_msg.feedback
        self._node.get_logger().info(
            f"Received feedback from '{self._action_name}': {feedback.status_message}"
        )

    def cancel_goal(self, goal_handle: ClientGoalHandle):
        if not self.is_server_ready():
            raise RuntimeError(f"Action server '{self._action_name}' is not ready.")

        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_goal_done_callback)

    def _cancel_goal_done_callback(self, future: Future):
        try:
            cancel_result = future.result()
        except Exception as e:
            self._node.get_logger().error(f"Exception while canceling goal: {e}")
            return False

        if cancel_result.return_code == rclpy.action.GoalResponse.ACCEPTED:
            self._node.get_logger().info(
                "Goal cancellation accepted by the action server."
            )
        else:
            self._node.get_logger().warn(
                "Goal cancellation rejected by the action server."
            )
