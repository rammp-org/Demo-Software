from __future__ import annotations
from typing import Any, Optional, Callable, Type
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future

ActionClientCallback = Callable[[bool], None]  # (success, result) -> None


def is_action_type(t: Type[Any]) -> bool:
    return all(hasattr(t, a) for a in ("Goal", "Result", "Feedback"))


class ActionClientWrapper:
    def __init__(
        self,
        name: str,
        goal_type: Type[Any],
        goal_callback: ActionClientCallback,
        result_callback: ActionClientCallback,
        cancel_callback: ActionClientCallback,
        node: Node,
    ):
        assert is_action_type(
            goal_type
        ), "goal_type must be an action type with Goal, Result, and Feedback attributes"
        self._action_running = False
        self._node = node
        self._action_name = name
        self._action_type = goal_type
        self._action_cb_group = self._node._cb_group
        self._qos_profile_services_goal = rclpy.qos.QoSProfile(depth=10)
        self._qos_profile_services_goal.reliability = (
            rclpy.qos.ReliabilityPolicy.RELIABLE
        )

        self._client: ActionClient = ActionClient(
            self._node,
            goal_type,
            self._action_name,
            callback_group=self._action_cb_group,
            goal_service_qos_profile=self._qos_profile_services_goal,
        )
        self._goal_callback = goal_callback
        self._result_callback = result_callback
        self._cancel_callback = cancel_callback
        self.gh = None

    async def wait_for_server(self, timeout: Optional[float] = None) -> bool:
        """
        Wait for the action server to be available.

        :param timeout: Optional timeout in seconds. If None, wait indefinitely.
        :return: True if the server is available, False if timed out.
        """
        try:
            await self._client.wait_for_server(timeout_sec=timeout)
            self._node.get_logger().debug(
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

    def is_action_running(self) -> bool:
        return self._action_running

    async def send_goal(self, goal: Type[Any]):
        if self._action_running:
            self._goal_callback(False)
            return
        if not isinstance(goal, self._action_type.Goal):
            self._node.get_logger().error("Invalid goal type.")
            self._goal_callback(False)
            return
        self._action_running = True
        send_goal_future = self._client.send_goal_async(
            goal
        )  # no feedback callback here.
        send_goal_future.add_done_callback(self._send_goal_done_callback)
        self._node.get_logger().debug(
            "Goal sent to action server, waiting for response..."
        )

    def _send_goal_done_callback(self, future: Future):
        try:
            goal_handle: ClientGoalHandle = future.result()
            if not goal_handle.accepted:
                self._node.get_logger().warn("Goal was rejected by the action server.")
                self._action_running = False
                self._goal_callback(False)
                return
            self._node.get_logger().debug("Goal accepted by the action server.")
            self.gh = goal_handle

            res_future = self.gh.get_result_async()
            res_future.add_done_callback(self._on_result)
            self._goal_callback(True)

        except Exception as e:
            self._node.get_logger().error(f"Exception while sending goal: {e}")
            self._action_running = False
            self._goal_callback(False)

    def _on_result(self, future: Future):
        try:
            result = future.result().result
            if result.success:
                self._node.get_logger().debug("Arm reached preset successfully.")
                self._result_callback(True)
            else:
                self._node.get_logger().warn("Arm failed to reach preset.")
                self._result_callback(False)
        except Exception as e:
            self._node.get_logger().error(f"Exception while getting result: {e}")
            self._result_callback(False)
        finally:
            self._action_running = False
            self.gh = None

    async def cancel_goal(self):
        if not self.is_server_ready() or self.gh is None:
            self._cancel_callback(False)
            return
        cancel_future = self.gh.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_goal_done_callback)

    def _cancel_goal_done_callback(self, future: Future):
        try:
            cancel_result = future.result()
        except Exception as e:
            self._node.get_logger().error(f"Exception while canceling goal: {e}")
            self._cancel_callback(False)
            return False

        if cancel_result.return_code == rclpy.action.GoalResponse.ACCEPTED:
            self._node.get_logger().debug(
                "Goal cancellation accepted by the action server."
            )
            self._cancel_callback(True)
        else:
            self._node.get_logger().warn(
                "Goal cancellation rejected by the action server."
            )
            self._cancel_callback(False)
