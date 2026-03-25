from __future__ import annotations
import asyncio
import enum

from rclpy.node import Node
from rammp_prototype_interfaces.action import CurbTraverse
from .ActionClientWrapper import ActionClientWrapper


class CurbTraverseDirection(enum.IntEnum):
    ASCEND = CurbTraverse.Goal.ASCEND
    DESCEND = CurbTraverse.Goal.DESCEND


class ChairCurbTraverseActionClient(ActionClientWrapper):
    def __init__(
        self,
        node: Node,
    ):
        super().__init__(
            "/base/curb_traverse",
            CurbTraverse,
            self.goal_callback,
            self.result_callback,
            self.cancel_callback,
            node,
        )

    def goal_callback(self, success: bool):
        if success:
            self._node.get_logger().info(
                "Goal CurbTraverse accepted by the action server."
            )
        else:
            self._node.get_logger().warn(
                "Goal CurbTraverse rejected by the action server."
            )
            self._node.reqArmActionGoalFailed()

    def result_callback(self, success: bool):
        if success:
            self._node.get_logger().info("Successfully completed CurbTraverse.")
            self._node.homedCup()  # should enter homed state after homing cup
            self._node.set_arm_mode_idle()  # set arm to idle after homing cup
            self._node.finish_mock_task()  # for testing, will remove after testing
        else:
            self._node.get_logger().warn("Failed to complete CurbTraverse.")
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

    def send_goal(self, direction: CurbTraverseDirection):
        goal = CurbTraverse.Goal()
        goal.direction = direction.value
        asyncio.run_coroutine_threadsafe(super().send_goal(goal), self._node._loop)

    def cancel(self):
        asyncio.run_coroutine_threadsafe(super().cancel_goal(), self._node._loop)
