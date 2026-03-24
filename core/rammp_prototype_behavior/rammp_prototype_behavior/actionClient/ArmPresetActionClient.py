from __future__ import annotations
import asyncio
import enum

from rclpy.node import Node
from arm_interfaces.action import ReachPreset
from .ActionClientWrapper import ActionClientWrapper


class ArmPreset(enum.IntEnum):
    HOME = ReachPreset.Goal.PRESET_HOME
    RETRACT = ReachPreset.Goal.PRESET_RETRACT
    ZERO = ReachPreset.Goal.PRESET_ZERO
    CUP_STABILIZE = ReachPreset.Goal.PRESET_CUP_STABILIZE


class ArmPresetActionClient(ActionClientWrapper):
    def __init__(
        self,
        node: Node,
    ):
        super().__init__(
            "/arm/reach_preset",
            ReachPreset,
            self.goal_callback,
            self.result_callback,
            self.cancel_callback,
            node,
        )
        self._current_preset = None

    def goal_callback(self, success: bool):
        if success:
            self._node.get_logger().info(
                f"Goal accepted by the action server for preset {self._current_preset.name}."
            )
        else:
            self._node.get_logger().warn(
                f"Goal rejected by the action server for preset {self._current_preset.name}."
            )
            self._node.reqArmActionGoalFailed()

    def result_callback(self, success: bool):
        if success:
            self._node.get_logger().info(
                f"Arm successfully reached preset {self._current_preset.name}."
            )
            if self._current_preset == ArmPreset.HOME:
                if (
                    self._node.state == "Arm_OrderDrink_releasingCup"
                    or self._node.state == "Arm_cupStabilize_homing"
                ):  # for testing, will remove after testing
                    self._node.finish_mock_task()

                self._node.homed()
            elif self._current_preset == ArmPreset.CUP_STABILIZE:
                self._node.cupStable()  # should enter cup stabilized state after reaching cup stabilize preset
        else:
            self._node.get_logger().warn(
                f"Arm failed to reach preset {self._current_preset.name}."
            )
            self._node.ArmActionFailed()
        self._current_preset = None

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

    def set_preset(self, arm_preset: ArmPreset):
        self._current_preset = arm_preset
        self._node.get_logger().info(f"Sending goal to reach preset: {arm_preset.name}")
        goal = ReachPreset.Goal()
        goal.preset = arm_preset.value
        asyncio.run_coroutine_threadsafe(self.send_goal(goal), self._node._loop)

    def cancel(self):
        self._current_preset = None
        asyncio.run_coroutine_threadsafe(self.cancel_goal(), self._node._loop)
