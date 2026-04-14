"""Unit tests for CupStabilizer — no ROS, no hardware required.

Run with:
    cd /path/to/Demo-Software
    python -m pytest hardware/arm_driver/test/test_cup_stabilizer.py -v
"""

import numpy as np


def _make_imu(accel=(0.0, 0.0, -9.81), gyro=(0.0, 0.0, 0.0), euler_deg=(0.0, 0.0, 0.0)):
    """Build a minimal imu_data dict matching arm_interface.get_imu_data() output."""
    return {
        "accel": np.array(accel, dtype=float),
        "gyro": np.array(gyro, dtype=float),
        "ee_euler_deg": list(euler_deg),
    }


class TestCupStabilizerCalibration:
    def test_feed_returns_none_before_calibration(self):
        from arm_driver.cup_stabilizer import CupStabilizer

        cs = CupStabilizer(hz=40.0, kp=8.0, kd=1.0)
        assert cs.feed(_make_imu()) is None

    def test_calibrate_accepts_gyro_samples_and_computes_mean(self):
        from arm_driver.cup_stabilizer import CupStabilizer

        cs = CupStabilizer(hz=40.0, kp=8.0, kd=1.0)
        samples = [np.array([1.0, 2.0, 3.0]), np.array([3.0, 4.0, 5.0])]
        cs.calibrate(samples)
        np.testing.assert_array_almost_equal(cs._gyro_offset, [2.0, 3.0, 4.0])

    def test_feed_returns_tuple_after_calibration(self):
        from arm_driver.cup_stabilizer import CupStabilizer

        cs = CupStabilizer(hz=40.0, kp=8.0, kd=1.0)
        cs.calibrate([np.zeros(3)])
        result = cs.feed(_make_imu())
        assert result is not None
        linear, angular = result
        assert len(linear) == 3
        assert len(angular) == 3

    def test_feed_linear_is_always_zero(self):
        """Cup stabilizer only rotates — no translation."""
        from arm_driver.cup_stabilizer import CupStabilizer

        cs = CupStabilizer(hz=40.0, kp=8.0, kd=1.0)
        cs.calibrate([np.zeros(3)])
        linear, _ = cs.feed(_make_imu())
        assert linear == [0.0, 0.0, 0.0]


class TestCupStabilizerControl:
    def test_zero_error_produces_zero_angular_command(self):
        """When tool_y already points up (gravity down = -Y in base frame → up = +Y)
        and identity rotation has tool_y = [0,1,0], error = 0."""
        from arm_driver.cup_stabilizer import CupStabilizer

        cs = CupStabilizer(hz=40.0, kp=8.0, kd=1.0)
        cs.calibrate([np.zeros(3)])

        # Gravity pointing -Y in base frame → up = +Y → tool_y should be [0,1,0]
        # Identity rotation has tool_y = [0,1,0], so error = 0
        imu = _make_imu(
            accel=(0.0, -9.81, 0.0),  # gravity in -Y → up = +Y
            gyro=(0.0, 0.0, 0.0),
            euler_deg=(0.0, 0.0, 0.0),  # identity rotation → tool_y = [0,1,0]
        )
        _, angular = cs.feed(imu)
        assert abs(angular[0]) < 1e-6
        assert abs(angular[1]) < 1e-6
        assert abs(angular[2]) < 1e-6

    def test_gyro_bias_is_subtracted(self):
        """Gyro bias must be removed before applying derivative gain."""
        from arm_driver.cup_stabilizer import CupStabilizer

        # cs calibrated with a known bias
        cs = CupStabilizer(hz=40.0, kp=8.0, kd=1.0)
        bias = np.array([10.0, 5.0, 0.0])
        cs.calibrate([bias])

        # cs2 calibrated with zero bias
        cs2 = CupStabilizer(hz=40.0, kp=8.0, kd=1.0)
        cs2.calibrate([np.zeros(3)])

        # Feeding cs raw gyro=bias should produce same result as feeding cs2 raw gyro=zeros
        # because cs subtracts bias → effective gyro = 0 in both cases
        imu_with_bias = _make_imu(
            accel=(0.0, -9.81, 0.0),
            gyro=tuple(bias),
            euler_deg=(0.0, 0.0, 0.0),
        )
        imu_zero_gyro = _make_imu(
            accel=(0.0, -9.81, 0.0),
            gyro=(0.0, 0.0, 0.0),
            euler_deg=(0.0, 0.0, 0.0),
        )
        _, angular_biased = cs.feed(imu_with_bias)
        _, angular_no_bias = cs2.feed(imu_zero_gyro)
        np.testing.assert_array_almost_equal(angular_biased, angular_no_bias)

    def test_near_zero_accel_norm_returns_zero_command(self):
        """Degenerate accel (sensor disconnected etc.) must not crash or produce garbage."""
        from arm_driver.cup_stabilizer import CupStabilizer

        cs = CupStabilizer(hz=40.0, kp=8.0, kd=1.0)
        cs.calibrate([np.zeros(3)])
        imu = _make_imu(accel=(0.0, 0.0, 0.0))
        result = cs.feed(imu)
        assert result is not None
        _, angular = result
        np.testing.assert_array_equal(angular, [0.0, 0.0, 0.0])

    def test_nonzero_gyro_contributes_to_angular_command(self):
        """Gyro rate must affect output with correct sign independent of orientation error."""
        from arm_driver.cup_stabilizer import CupStabilizer

        cs = CupStabilizer(hz=40.0, kp=8.0, kd=1.0)
        cs.calibrate([np.zeros(3)])

        # Zero-error pose (tool_y = up), non-zero gyro rate in x only
        # With zero orientation error, only the kd term contributes
        # omega_x = kp*0 + kd*gyro_rads[0]  → positive gyro_x → positive omega_x
        # omega_y = -kp*0 - kd*gyro_rads[1] → zero gyro_y → zero omega_y
        gyro_x_dps = 10.0  # 10 deg/s around x
        imu = _make_imu(
            accel=(0.0, -9.81, 0.0),  # gravity -Y → up = +Y
            gyro=(gyro_x_dps, 0.0, 0.0),  # spinning around x only
            euler_deg=(0.0, 0.0, 0.0),  # identity → tool_y = [0,1,0] = up
        )
        _, angular = cs.feed(imu)

        expected_omega_x = 1.0 * np.deg2rad(gyro_x_dps)  # kd=1.0, gyro_rads[0]
        assert (
            abs(angular[0] - expected_omega_x) < 1e-6
        ), f"omega_x expected {expected_omega_x:.6f}, got {angular[0]:.6f}"
        assert abs(angular[1]) < 1e-6  # no y-gyro → no y contribution
        assert abs(angular[2]) < 1e-6
