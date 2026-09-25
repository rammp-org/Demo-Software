"""
Regression test for the gripper direction in BaseAction.

The cup is held by an INSIDE grasp: the closed fingers enter the handle loop and
opening them presses outward against the handle. So grasp_tool must send
OpenGripperCommand and ungrasp_tool must send CloseGripperCommand (the arm
driver maps open -> Kortex finger 0, close -> finger 1, no inversion).
pickup_and_order closes the gripper before entering the handle for this reason.
"""

from rammp.actions.base import BaseAction
from rammp.control.robot_controller.command_interface import (
    CloseGripperCommand,
    OpenGripperCommand,
)


class _SimStub:
    def grasp_object(self, tool):
        pass

    def ungrasp_object(self):
        pass


class _Spy:
    """Minimal stand-in for BaseAction's `self` capturing the emitted command."""

    def __init__(self):
        self.sim = _SimStub()
        self.robot_interface = object()  # not None -> hardware command path
        self.sent = []

    def _check_cancel(self):
        pass

    def execute_robot_command(self, command, tool_update=None):
        self.sent.append(command)


def test_grasp_tool_opens_gripper_inside_handle():
    spy = _Spy()
    BaseAction.grasp_tool(spy, "drink")
    assert len(spy.sent) == 1
    assert isinstance(spy.sent[0], OpenGripperCommand)


def test_ungrasp_tool_closes_gripper():
    spy = _Spy()
    BaseAction.ungrasp_tool(spy, "drink")
    assert len(spy.sent) == 1
    assert isinstance(spy.sent[0], CloseGripperCommand)
