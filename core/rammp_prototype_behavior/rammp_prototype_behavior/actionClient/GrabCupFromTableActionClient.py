from __future__ import annotations
import asyncio

from rclpy.node import Node
from cornell_feeding_interfaces.action import CornellActionsPlaceHolder
from .ActionClientWrapper import ActionClientWrapper


class GrabCupFromTableActionClient(ActionClientWrapper):
    def __init__(
        self,
        node: Node,
    ):
        super().__init__(
            "/arm/drink/GrabCupFromTable",
            CornellActionsPlaceHolder,
            self.goal_callback,
            self.result_callback,
            self.cancel_callback,
            node,
        )

    def goal_callback(self, success: bool):
        if success:
            self._node.get_logger().info("Goal accepted by the action server.")
        else:
            self._node.get_logger().warn("Goal rejected by the action server.")
            self._node.reqArmActionGoalFailed()

    def result_callback(self, success: bool):
        if success:
            self._node.get_logger().info("Successfully grabbed cup from table.")
        else:
            self._node.get_logger().warn("Failed to grab cup from table.")
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
        asyncio.run_coroutine_threadsafe(self.send_goal(goal), self._node._loop)

    def cancel(self):
        asyncio.run_coroutine_threadsafe(self.cancel_goal(), self._node._loop)
