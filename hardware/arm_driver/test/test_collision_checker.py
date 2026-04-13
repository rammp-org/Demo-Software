"""Unit tests for CollisionChecker.

Pinocchio is mocked — no URDF or hardware required.

Run with:
    cd /path/to/Demo-Software
    python -m pytest hardware/arm_driver/test/test_collision_checker.py -v
"""

import math

import numpy as np
import pytest
from unittest.mock import MagicMock, patch


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_checker(thresholds=None):
    """Build a CollisionChecker with Pinocchio fully mocked."""
    if thresholds is None:
        thresholds = {"DEFAULT": 100.0, "OPEN_DOOR": 500.0}

    with patch("arm_driver.collision_checker.pin") as mock_pin:
        mock_model = MagicMock()
        mock_model.nv = 7
        mock_pin.buildModelFromUrdf.return_value = mock_model
        mock_model.createData.return_value = MagicMock()

        from arm_driver.collision_checker import CollisionChecker

        checker = CollisionChecker("/fake/path.urdf", thresholds)
        # Store the data mock so tests can set .tau on it
        checker._model_data.tau = np.zeros(7)
        return checker


def _zero_inputs():
    return np.zeros(7), np.zeros(7)


# ---------------------------------------------------------------------------
# Tests: threshold logic
# ---------------------------------------------------------------------------


class TestThresholds:
    def test_no_collision_when_residual_below_default(self):
        checker = _make_checker()
        q, dq = _zero_inputs()
        tau_measured = np.array([50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        checker._model_data.tau = np.zeros(7)  # model predicts 0 → residual = 50

        with patch("arm_driver.collision_checker.pin") as mock_pin:
            mock_pin.rnea.return_value = None
            checker._pin = mock_pin
            result = checker.check(q, dq, tau_measured, "IDLE")

        assert result is False

    def test_collision_when_residual_exceeds_default(self):
        checker = _make_checker()
        q, dq = _zero_inputs()
        tau_measured = np.array([200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        checker._model_data.tau = np.zeros(7)  # residual = 200 > 100

        with patch("arm_driver.collision_checker.pin") as mock_pin:
            mock_pin.rnea.return_value = None
            checker._pin = mock_pin
            result = checker.check(q, dq, tau_measured, "IDLE")

        assert result is True

    def test_open_door_uses_higher_threshold(self):
        """200 Nm residual: exceeds DEFAULT (100) but not OPEN_DOOR (500)."""
        checker = _make_checker(thresholds={"DEFAULT": 100.0, "OPEN_DOOR": 500.0})
        q, dq = _zero_inputs()
        tau_measured = np.array([200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        checker._model_data.tau = np.zeros(7)

        with patch("arm_driver.collision_checker.pin") as mock_pin:
            mock_pin.rnea.return_value = None
            checker._pin = mock_pin
            result_normal = checker.check(q, dq, tau_measured, "IDLE")
            result_door = checker.check(q, dq, tau_measured, "OPEN_DOOR")

        assert result_normal is True
        assert result_door is False

    def test_unknown_state_falls_back_to_default(self):
        checker = _make_checker(thresholds={"DEFAULT": 100.0})
        q, dq = _zero_inputs()
        tau_measured = np.array([200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        checker._model_data.tau = np.zeros(7)

        with patch("arm_driver.collision_checker.pin") as mock_pin:
            mock_pin.rnea.return_value = None
            checker._pin = mock_pin
            result = checker.check(q, dq, tau_measured, "SOME_FUTURE_STATE")

        assert result is True

    def test_requires_default_key(self):
        with patch("arm_driver.collision_checker.pin"):
            from arm_driver.collision_checker import CollisionChecker

            with pytest.raises(ValueError, match="DEFAULT"):
                CollisionChecker("/fake.urdf", {"OPEN_DOOR": 500.0})


# ---------------------------------------------------------------------------
# Tests: q_pin encoding
# ---------------------------------------------------------------------------


class TestQPinEncoding:
    def test_continuous_joints_use_cos_sin(self):
        """Joints at indices 0, 2, 4, 6 must be encoded as [cos, sin]."""
        from arm_driver.collision_checker import CollisionChecker

        q = np.array(
            [math.pi / 4, 0.5, math.pi / 3, 1.0, math.pi / 6, 0.8, math.pi / 2]
        )
        expected = np.array(
            [
                math.cos(q[0]),
                math.sin(q[0]),
                q[1],
                math.cos(q[2]),
                math.sin(q[2]),
                q[3],
                math.cos(q[4]),
                math.sin(q[4]),
                q[5],
                math.cos(q[6]),
                math.sin(q[6]),
            ]
        )
        result = CollisionChecker._to_q_pin(q)
        np.testing.assert_array_almost_equal(result, expected)

    def test_output_length_is_11(self):
        from arm_driver.collision_checker import CollisionChecker

        result = CollisionChecker._to_q_pin(np.zeros(7))
        assert len(result) == 11

    def test_zero_angles_give_cos1_sin0(self):
        """cos(0)=1, sin(0)=0 for all continuous joints at q=0."""
        from arm_driver.collision_checker import CollisionChecker

        result = CollisionChecker._to_q_pin(np.zeros(7))
        # Continuous joint slots: indices 0,1  |  3,4  |  6,7  |  9,10
        for cos_idx, sin_idx in [(0, 1), (3, 4), (6, 7), (9, 10)]:
            assert result[cos_idx] == pytest.approx(1.0)
            assert result[sin_idx] == pytest.approx(0.0)
