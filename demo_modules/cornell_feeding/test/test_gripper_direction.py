"""
Regression test for the gripper direction in BaseAction.

grasp_tool must physically CLOSE the gripper (CloseGripperCommand) and ungrasp_tool
must OPEN it (OpenGripperCommand). The arm driver does not invert open/close, so an
inverted mapping here would release the cup at the grasp moment. This guards the fix.
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


def test_grasp_tool_closes_gripper():
    spy = _Spy()
    BaseAction.grasp_tool(spy, "drink")
    assert len(spy.sent) == 1
    assert isinstance(spy.sent[0], CloseGripperCommand)


def test_ungrasp_tool_opens_gripper():
    spy = _Spy()
    BaseAction.ungrasp_tool(spy, "drink")
    assert len(spy.sent) == 1
    assert isinstance(spy.sent[0], OpenGripperCommand)
