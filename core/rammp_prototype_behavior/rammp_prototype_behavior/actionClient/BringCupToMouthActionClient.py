from __future__ import annotations
import asyncio

from rclpy.node import Node
from cornell_feeding_interfaces.action import CornellActionsPlaceHolder
from .ActionClientWrapper import ActionClientWrapper


class BringCupToMouthActionClient(ActionClientWrapper):
    def __init__(
        self,
        node: Node,
    ):
        super().__init__(
            "/arm/drink/BringCupToMouth",
            CornellActionsPlaceHolder,
            self.goal_callback,
            self.result_callback,
            self.cancel_callback,
            node,
        )

    def goal_callback(self, success: bool):
        if success:
            self._node.get_logger().info(
                "Goal BringCupToMouth accepted by the action server."
            )
        else:
            self._node.get_logger().warn(
                "Goal BringCupToMouth rejected by the action server."
            )
            self._node.reqArmActionGoalFailed()

    def result_callback(self, success: bool):
        if success:
            self._node.get_logger().info("Successfully brought cup to mouth.")
            self._node.readyForDrink()  # should enter readyForDrink state after bringing cup to mouth
        else:
            self._node.get_logger().warn("Failed to bring cup to mouth.")
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
        asyncio.run_coroutine_threadsafe(super().cancel_goal(), self._node._loop)
