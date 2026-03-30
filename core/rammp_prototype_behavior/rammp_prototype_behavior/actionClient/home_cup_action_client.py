from __future__ import annotations
import asyncio

from rclpy.node import Node
from cornell_feeding_interfaces.action import CornellActionsPlaceHolder
from .action_client_wrapper import ActionClientWrapper


class HomeCupActionClient(ActionClientWrapper):
    def __init__(
        self,
        node: Node,
    ):
        super().__init__(
            "/arm/drink/HomeCup",
            CornellActionsPlaceHolder,
            self.goal_callback,
            self.result_callback,
            self.cancel_callback,
            node,
        )

    def goal_callback(self, success: bool):
        if success:
            self._node.get_logger().info("Goal HomeCup accepted by the action server.")
        else:
            self._node.get_logger().warn("Goal HomeCup rejected by the action server.")
            self._node.reqArmActionGoalFailed()

    def result_callback(self, success: bool):
        if success:
            self._node.get_logger().info("Successfully homed cup.")
            self._node.homedCup()  # should enter homed state after homing cup
            self._node.set_arm_mode_idle()  # set arm to idle after homing cup
            self._node.finish_mock_task()  # for testing, will remove after testing
        else:
            self._node.get_logger().warn("Failed to home cup.")
            self._node.ArmActionFailed()

    def cancel_callback(self, success: bool):
        if success:
            self._node.get_logger().info(
                "Goal cancellation accepted by the action server."
            )
            self._node.reqArmActionCancelSuccess()
        else:
            self._node.get_logger().warn(
                "Goal cancellation rejected by the action server."
            )
            self._node.reqArmActionCancelFailed()

    def send_goal(self):
        goal = CornellActionsPlaceHolder.Goal()
        asyncio.run_coroutine_threadsafe(super().send_goal(goal), self._node._loop)

    def cancel(self):
        if not self.is_action_running():
            # do nothing if no action is currently running
            return
        asyncio.run_coroutine_threadsafe(super().cancel_goal(), self._node._loop)
