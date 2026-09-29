"""Unit tests for the /arm/set_gripper_position service (issue #235).

Adds continuous partial-position gripper control. These tests mock KinovaArm
entirely — no hardware required.

Run with:
    cd /path/to/Demo-Software
    python -m pytest hardware/arm_driver/test/test_gripper_position.py -v
"""

import pytest
import rclpy
from arm_interfaces.srv import SetGripperPosition
from unittest.mock import MagicMock, patch


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node():
    """Create ArmDriverNode with KinovaArm, xacro and CollisionChecker mocked.

    Returns (node, mock_arm, ArmState).
    """
    with (
        patch("arm_driver.arm_driver.KinovaArm") as MockArm,
        patch("arm_driver.arm_driver.subprocess.run") as mock_run,
        patch("arm_driver.arm_driver.CollisionChecker") as MockChecker,
    ):
        mock_arm = MagicMock()
        mock_arm.actuator_count = 7
        mock_arm.get_fault_state.return_value = ("ARMSTATE_SERVOING_READY", False)
        MockArm.return_value = mock_arm

        mock_run.return_value = MagicMock(stdout="<robot/>")

        mock_checker = MagicMock()
        mock_checker.check.return_value = False
        MockChecker.return_value = mock_checker

        from arm_driver.arm_driver import ArmDriverNode, ArmState

        node = ArmDriverNode()
        # Set self._arm to the mock while the patches are still active.
        node._try_connect_arm()
        return node, mock_arm, ArmState


def _call(node, position):
    req = SetGripperPosition.Request()
    req.position = float(position)
    resp = SetGripperPosition.Response()
    return node._on_set_gripper_position(req, resp)


class TestSetGripperPosition:
    def test_partial_position_forwarded_to_arm(self):
        """A valid in-range position is passed straight through to the arm."""
        node, mock_arm, ArmState = _make_node()
        try:
            node._state = ArmState.DRINKING  # gripper-allowed (non IDLE/ERROR) state
            resp = _call(node, 0.5)
            assert resp.success
            mock_arm.set_gripper_position.assert_called_once_with(0.5)
        finally:
            node.destroy_node()

    def test_value_above_one_is_clamped(self):
        node, mock_arm, ArmState = _make_node()
        try:
            node._state = ArmState.MANUAL
            resp = _call(node, 1.5)
            assert resp.success
            mock_arm.set_gripper_position.assert_called_once_with(1.0)
            assert "clamp" in resp.message.lower()
        finally:
            node.destroy_node()

    def test_value_below_zero_is_clamped(self):
        node, mock_arm, ArmState = _make_node()
        try:
            node._state = ArmState.MANUAL
            resp = _call(node, -0.2)
            assert resp.success
            mock_arm.set_gripper_position.assert_called_once_with(0.0)
            assert "clamp" in resp.message.lower()
        finally:
            node.destroy_node()

    def test_rejected_in_idle(self):
        """Mirrors open/close gating: gripper commands are refused in IDLE."""
        node, mock_arm, ArmState = _make_node()
        try:
            assert node._state == ArmState.IDLE
            resp = _call(node, 0.5)
            assert not resp.success
            mock_arm.set_gripper_position.assert_not_called()
        finally:
            node.destroy_node()

    def test_rejected_in_error(self):
        node, mock_arm, ArmState = _make_node()
        try:
            node._state = ArmState.ERROR
            resp = _call(node, 0.5)
            assert not resp.success
            mock_arm.set_gripper_position.assert_not_called()
        finally:
            node.destroy_node()

    def test_rejected_when_arm_not_connected(self):
        node, mock_arm, ArmState = _make_node()
        try:
            node._state = ArmState.DRINKING
            node._arm = None
            resp = _call(node, 0.5)
            assert not resp.success
            assert "not connected" in resp.message.lower()
        finally:
            node.destroy_node()
