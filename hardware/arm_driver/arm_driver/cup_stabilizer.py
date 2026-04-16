"""Cup stabilization PD controller.

Pure algorithm — no ROS, no Kortex dependency.  Receives IMU snapshots from
arm_driver and returns Cartesian twist commands (base frame) to counteract cup tilt.

The controller aligns the end-effector tool-Y axis with the gravity direction
inferred from the raw accelerometer.  Using raw (unfiltered) accel intentionally
preserves cup-vibration dynamics so the controller acts as active vibration
compensation, not just static leveling.
"""

import numpy as np
from scipy.spatial.transform import Rotation


class CupStabilizer:
    """PD controller for cup stabilization.

    Args:
        hz: Control loop rate in Hz (informational only — timing is owned by arm_driver).
        kp: Proportional gain (rad/s per unit orientation error).
        kd: Derivative gain (rad/s per rad/s gyro rate).
    """

    def __init__(self, hz: float, kp: float, kd: float):
        # hz is accepted for API symmetry with arm_driver parameters but not stored;
        # the caller (arm_driver) owns the timer rate.
        self._kp = kp
        self._kd = kd
        self._gyro_offset: np.ndarray | None = None

    @property
    def gyro_offset(self) -> np.ndarray | None:
        """The calibrated gyro bias offset, or None if not yet calibrated."""
        return self._gyro_offset

    def calibrate(self, gyro_samples: list[np.ndarray]) -> None:
        """Compute gyro bias from collected samples.

        Args:
            gyro_samples: List of raw gyro readings (each shape (3,), deg/s)
                collected while the arm was stationary.
        """
        self._gyro_offset = np.mean(gyro_samples, axis=0)

    def feed(self, imu_data: dict) -> tuple[list[float], list[float]] | None:
        """Compute a twist command from one IMU snapshot.

        Args:
            imu_data: Dict with keys:
                ``accel``       – np.ndarray [ax, ay, az], raw accelerometer (m/s²).
                ``gyro``        – np.ndarray [gx, gy, gz], raw gyro (deg/s).
                ``ee_euler_deg``– list [rx, ry, rz], end-effector Euler angles (deg).

        Returns:
            (linear_xyz, angular_xyz) tuple ready for ``send_twist_base_frame``,
            or None if not yet calibrated.
        """
        if self._gyro_offset is None:
            return None

        accel: np.ndarray = imu_data["accel"]
        gyro: np.ndarray = imu_data["gyro"]
        ee_euler_deg: list[float] = imu_data["ee_euler_deg"]

        norm = float(np.linalg.norm(accel))
        if norm < 1e-6:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        up = -(accel / norm)
        gyro_rads = np.deg2rad(gyro - self._gyro_offset)

        rot = Rotation.from_euler("xyz", np.deg2rad(ee_euler_deg))
        tool_y = rot.as_matrix()[:, 1]

        error = tool_y - up
        # small-angle approximation: ||tool_y - up|| ≈ 2·sin(θ/2) ≈ θ for small θ
        # error[0]/[1] (X/Y components in base frame) map to omega_y/omega_x respectively
        # due to the cross-product geometry: a Y-axis tilt is corrected by an X rotation.
        omega_x = self._kp * error[1] + self._kd * gyro_rads[0]
        omega_y = -self._kp * error[0] - self._kd * gyro_rads[1]

        return [0.0, 0.0, 0.0], [float(omega_x), float(omega_y), 0.0]
