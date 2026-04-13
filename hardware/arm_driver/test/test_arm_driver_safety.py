"""Unit tests for arm_driver safety behaviours.

These tests mock KinovaArm entirely — no hardware required.

Run with:
    cd /path/to/Demo-Software
    python -m pytest hardware/arm_driver/test/test_arm_driver_safety.py -v
"""

import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from unittest.mock import MagicMock, patch


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node():
    """Create ArmDriverNode with KinovaArm and xacro/CollisionChecker mocked.

    Returns (node, mock_arm_instance, ArmState).
    """
    with (
        patch("arm_driver.arm_driver.KinovaArm") as MockArm,
        patch("arm_driver.arm_driver.subprocess.run") as mock_run,
        patch("arm_driver.arm_driver.CollisionChecker") as MockChecker,
    ):
        mock_arm = MagicMock()
        mock_arm.actuator_count = 7
        MockArm.return_value = mock_arm

        # Simulate xacro producing a URDF string
        mock_run.return_value = MagicMock(stdout="<robot/>")

        # CollisionChecker construction succeeds but check() returns False by default
        mock_checker = MagicMock()
        mock_checker.check.return_value = False
        MockChecker.return_value = mock_checker

        from arm_driver.arm_driver import ArmDriverNode, ArmState

        node = ArmDriverNode()
        return node, mock_arm, ArmState


def _good_state():
    """Return a realistic get_state() dict for use in feedback-loop tests."""
    return {
        "position": np.zeros(7),
        "velocity": np.zeros(7),
        "effort": np.zeros(7),
        "ee_pos": np.zeros(7),
        "ee_vel": np.zeros(6),
        "ee_force": np.zeros(3),
        "gripper_pos": 0.0,
    }


# ---------------------------------------------------------------------------
# Task 1: E-stop
# ---------------------------------------------------------------------------


class TestEstop:
    def test_estop_true_calls_arm_stop(self):
        node, mock_arm, ArmState = _make_node()
        try:
            msg = Bool()
            msg.data = True
            node._on_estop(msg)
            mock_arm.stop.assert_called()
            assert node._state == ArmState.ERROR
        finally:
            node.destroy_node()

    def test_estop_false_does_nothing(self):
        node, mock_arm, ArmState = _make_node()
        try:
            msg = Bool()
            msg.data = False
            node._on_estop(msg)
            mock_arm.stop.assert_not_called()
            assert node._state == ArmState.IDLE
        finally:
            node.destroy_node()

    def test_estop_stop_exception_still_transitions_to_error(self):
        """A stop() failure must not silently swallow the e-stop."""
        node, mock_arm, ArmState = _make_node()
        try:
            mock_arm.stop.side_effect = RuntimeError("comm error")
            msg = Bool()
            msg.data = True
            node._on_estop(msg)  # must not raise
            assert node._state == ArmState.ERROR
        finally:
            node.destroy_node()


# ---------------------------------------------------------------------------
# Task 2: Arm communication health
# ---------------------------------------------------------------------------


class TestCommsHealth:
    def test_timeout_triggers_error(self):
        node, mock_arm, ArmState = _make_node()
        try:
            mock_arm.get_state.side_effect = TimeoutError("no response")
            from arm_driver.arm_driver import COMMS_TIMEOUT_S

            node._last_feedback_time = time.monotonic() - COMMS_TIMEOUT_S - 0.1
            node._publish_joint_states()
            assert node._state == ArmState.ERROR
            assert "communication" in node._error_reason.lower()
        finally:
            node.destroy_node()

    def test_timeout_resets_on_successful_feedback(self):
        node, mock_arm, ArmState = _make_node()
        try:
            from arm_driver.arm_driver import COMMS_TIMEOUT_S

            node._last_feedback_time = time.monotonic() - COMMS_TIMEOUT_S + 0.2
            mock_arm.get_state.return_value = _good_state()
            node._publish_joint_states()
            assert node._state == ArmState.IDLE
        finally:
            node.destroy_node()

    def test_timeout_not_triggered_while_fresh(self):
        node, mock_arm, ArmState = _make_node()
        try:
            mock_arm.get_state.side_effect = TimeoutError("no response")
            node._last_feedback_time = time.monotonic()  # just now
            node._publish_joint_states()
            assert node._state == ArmState.IDLE
        finally:
            node.destroy_node()


# ---------------------------------------------------------------------------
# Task 3: Stale twist watchdog
# ---------------------------------------------------------------------------


class TestTwistWatchdog:
    def test_stale_twist_stops_arm_in_manual(self):
        node, mock_arm, ArmState = _make_node()
        try:
            from arm_driver.arm_driver import TWIST_TIMEOUT_S

            node._state = ArmState.MANUAL
            node._last_twist_time = time.monotonic() - TWIST_TIMEOUT_S - 0.1
            node._check_twist_timeout()
            mock_arm.stop.assert_called_once()
            assert node._state == ArmState.MANUAL  # not ERROR — recoverable
        finally:
            node.destroy_node()

    def test_stale_twist_stops_arm_in_cup_stabilize(self):
        node, mock_arm, ArmState = _make_node()
        try:
            from arm_driver.arm_driver import TWIST_TIMEOUT_S

            node._state = ArmState.CUP_STABILIZE
            node._last_twist_time = time.monotonic() - TWIST_TIMEOUT_S - 0.1
            node._check_twist_timeout()
            mock_arm.stop.assert_called_once()
        finally:
            node.destroy_node()

    def test_fresh_twist_does_not_stop_arm(self):
        node, mock_arm, ArmState = _make_node()
        try:
            node._state = ArmState.MANUAL
            node._last_twist_time = time.monotonic() - 0.1  # recent
            node._check_twist_timeout()
            mock_arm.stop.assert_not_called()
        finally:
            node.destroy_node()

    def test_watchdog_ignores_non_twist_states(self):
        node, mock_arm, ArmState = _make_node()
        try:
            from arm_driver.arm_driver import TWIST_TIMEOUT_S

            node._state = ArmState.IDLE
            node._last_twist_time = time.monotonic() - TWIST_TIMEOUT_S - 1.0
            node._check_twist_timeout()
            mock_arm.stop.assert_not_called()
        finally:
            node.destroy_node()

    def test_watchdog_no_op_before_first_twist(self):
        """_last_twist_time is None at startup — must not stop or crash."""
        node, mock_arm, ArmState = _make_node()
        try:
            node._state = ArmState.MANUAL
            node._last_twist_time = None
            node._check_twist_timeout()
            mock_arm.stop.assert_not_called()
        finally:
            node.destroy_node()

    def test_handle_twist_updates_timestamp(self):
        node, mock_arm, ArmState = _make_node()
        try:
            node._state = ArmState.MANUAL
            before = time.monotonic()
            msg = Twist()
            node._handle_twist(msg)
            assert node._last_twist_time is not None
            assert node._last_twist_time >= before
        finally:
            node.destroy_node()


# ---------------------------------------------------------------------------
# Task 6: Collision detection integration
# ---------------------------------------------------------------------------


class TestCollisionIntegration:
    def test_collision_transitions_to_error(self):
        node, mock_arm, ArmState = _make_node()
        try:
            mock_checker = MagicMock()
            mock_checker.check.return_value = True
            node._collision_checker = mock_checker
            mock_arm.get_state.return_value = _good_state()
            node._last_feedback_time = time.monotonic()
            node._publish_joint_states()
            assert node._state == ArmState.ERROR
            assert "collision" in node._error_reason.lower()
        finally:
            node.destroy_node()

    def test_no_collision_stays_idle(self):
        node, mock_arm, ArmState = _make_node()
        try:
            mock_checker = MagicMock()
            mock_checker.check.return_value = False
            node._collision_checker = mock_checker
            mock_arm.get_state.return_value = _good_state()
            node._last_feedback_time = time.monotonic()
            node._publish_joint_states()
            assert node._state == ArmState.IDLE
        finally:
            node.destroy_node()


# ---------------------------------------------------------------------------
# ERROR state: clear_error service and action/command rejection
# ---------------------------------------------------------------------------


def _make_node_in_error():
    """Return (node, mock_arm, ArmState) with the node already in ERROR state."""
    node, mock_arm, ArmState = _make_node()
    node._error_reason = "test-induced error"
    node._state = ArmState.ERROR
    return node, mock_arm, ArmState


class TestErrorState:
    # ── clear_error service ───────────────────────────────────────────────

    def test_clear_error_transitions_to_idle(self):
        node, mock_arm, ArmState = _make_node_in_error()
        try:
            req = Trigger.Request()
            resp = Trigger.Response()
            resp = node._on_clear_error(req, resp)
            assert resp.success
            assert node._state == ArmState.IDLE
            assert node._error_reason == ""
        finally:
            node.destroy_node()

    def test_clear_error_rejects_when_not_in_error(self):
        node, mock_arm, ArmState = _make_node()
        try:
            assert node._state == ArmState.IDLE
            req = Trigger.Request()
            resp = Trigger.Response()
            resp = node._on_clear_error(req, resp)
            assert not resp.success
            assert node._state == ArmState.IDLE  # unchanged
        finally:
            node.destroy_node()

    # ── set_mode rejected from ERROR ─────────────────────────────────────

    def test_set_mode_rejected_from_error(self):
        node, mock_arm, ArmState = _make_node_in_error()
        try:
            from arm_interfaces.srv import SetMode

            req = SetMode.Request()
            req.mode = SetMode.Request.MODE_IDLE
            resp = SetMode.Response()
            resp = node._on_set_mode(req, resp)
            assert not resp.success
            assert node._state == ArmState.ERROR  # unchanged
        finally:
            node.destroy_node()

    # ── reach_preset action rejected from ERROR ───────────────────────────

    def test_reach_preset_aborts_from_error(self):
        node, mock_arm, ArmState = _make_node_in_error()
        try:
            from arm_interfaces.action import ReachPreset

            goal_handle = MagicMock()
            goal_handle.request.preset = ReachPreset.Goal.PRESET_HOME
            result = node._on_reach_preset(goal_handle)
            goal_handle.abort.assert_called_once()
            assert not result.success
            assert "ERROR" in result.message
            # Must not have transitioned away from ERROR
            assert node._state == ArmState.ERROR
        finally:
            node.destroy_node()

    # ── twist commands ignored in ERROR ──────────────────────────────────

    def test_twist_ignored_in_error(self):
        """_on_command gates by authorized source; ERROR has no authorized source
        so twist messages must be silently dropped."""
        node, mock_arm, ArmState = _make_node_in_error()
        try:
            msg = Twist()
            msg.linear.x = 0.5
            # Route through _on_command exactly as the subscriber would
            node._on_command("xbox", node._handle_twist, msg)
            mock_arm.send_twist.assert_not_called()
        finally:
            node.destroy_node()
