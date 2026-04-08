"""Async reachability checker that delegates to the arm driver's Kortex IK solver.

Checks the full push trajectory — from the approach pose (button surface) to
the push pose (button + PUSH_MAX overshoot) — by solving IK at both endpoints,
interpolating in joint space, and verifying that no joint limit is crossed
along the path.

Both IK requests fire in cycle *N* and responses are picked up in cycle *N+1*
(one-frame latency, ~200 ms at 5 Hz — negligible).
"""

import numpy as np
from scipy.spatial.transform import Rotation

from arm_interfaces.srv import CheckReachability

# Must match button_push_controller.py
PUSH_MAX = 0.05  # metres — max push overshoot past button surface
TOOL_OFFSET = np.array([0.033, 0.0, 0.091])  # must match button_push_controller.py

# Gen3 7-DOF joint limits (radians) from the URDF.
# Joints 1,3,5,7 are continuous (±2π); joints 2,4,6 have tighter limits.
JOINT_LOWER = np.array(
    [-2 * np.pi, -2.24, -2 * np.pi, -2.57, -2 * np.pi, -2.09, -2 * np.pi]
)
JOINT_UPPER = np.array([2 * np.pi, 2.24, 2 * np.pi, 2.57, 2 * np.pi, 2.09, 2 * np.pi])

# Margin from the hard limit to consider "safe" (radians, ~3°).
JOINT_LIMIT_MARGIN = 0.05

# Number of interpolation steps between approach and push IK solutions.
INTERP_STEPS = 20


class ReachabilityChecker:
    """Non-blocking IK reachability check via ``/arm/check_reachability``.

    Solves IK at the approach and push poses, then interpolates the joint-space
    path and verifies every sample stays within joint limits.

    Parameters
    ----------
    node : rclpy.node.Node
        The owning ROS node (used to create the service client and for logging).
    """

    def __init__(self, node):
        self._node = node
        self._client = node.create_client(CheckReachability, "/arm/check_reachability")
        self._approach_future = None
        self._push_future = None
        self._reachable = False
        self._service_warned = False

    # ------------------------------------------------------------------
    def check_async(self, approach_xyz, approach_quat):
        """Fire async reachability checks for approach and push poses.

        Parameters
        ----------
        approach_xyz : array-like, shape (3,)
            Button surface position [x, y, z] in the robot base frame (metres).
        approach_quat : array-like, shape (4,)
            Button surface orientation as quaternion [x, y, z, w].
        """
        self._poll()

        if not self._client.service_is_ready():
            if not self._service_warned:
                self._node.get_logger().warn(
                    "Reachability service /arm/check_reachability not available "
                    "— is_pressable will be False until the arm driver is running"
                )
                self._service_warned = True
            self._reachable = False
            return

        if self._service_warned:
            self._node.get_logger().info("Reachability service now available")
            self._service_warned = False

        # Don't fire new requests while any are in-flight.
        approach_busy = (
            self._approach_future is not None and not self._approach_future.done()
        )
        push_busy = self._push_future is not None and not self._push_future.done()
        if approach_busy or push_busy:
            return

        # Apply the same lateral offset as ButtonPushController so we
        # check reachability for the actual poses the arm will command.
        rot = Rotation.from_quat(approach_quat)
        corrected_xyz = np.asarray(approach_xyz, dtype=np.float64) + rot.apply(
            TOOL_OFFSET
        )

        # 1) Approach pose — button surface (with lateral correction)
        req_approach = CheckReachability.Request()
        req_approach.target_pose.position.x = float(corrected_xyz[0])
        req_approach.target_pose.position.y = float(corrected_xyz[1])
        req_approach.target_pose.position.z = float(corrected_xyz[2])
        req_approach.target_pose.orientation.x = float(approach_quat[0])
        req_approach.target_pose.orientation.y = float(approach_quat[1])
        req_approach.target_pose.orientation.z = float(approach_quat[2])
        req_approach.target_pose.orientation.w = float(approach_quat[3])
        self._approach_future = self._client.call_async(req_approach)

        # 2) Push pose — corrected approach + PUSH_MAX along tool z-axis
        push_dir = rot.as_matrix()[:, 2]
        push_xyz = corrected_xyz + push_dir * PUSH_MAX

        req_push = CheckReachability.Request()
        req_push.target_pose.position.x = float(push_xyz[0])
        req_push.target_pose.position.y = float(push_xyz[1])
        req_push.target_pose.position.z = float(push_xyz[2])
        req_push.target_pose.orientation.x = float(approach_quat[0])
        req_push.target_pose.orientation.y = float(approach_quat[1])
        req_push.target_pose.orientation.z = float(approach_quat[2])
        req_push.target_pose.orientation.w = float(approach_quat[3])
        self._push_future = self._client.call_async(req_push)

    # ------------------------------------------------------------------
    @property
    def is_reachable(self) -> bool:
        """Return True only if the full push trajectory is within joint limits."""
        self._poll()
        return self._reachable

    # ------------------------------------------------------------------
    def _poll(self):
        """Check if in-flight futures have completed and validate the path."""
        # Need both futures to be done before we can validate.
        if self._approach_future is None or self._push_future is None:
            return
        if not self._approach_future.done() or not self._push_future.done():
            return

        approach_ok = False
        push_ok = False
        q_approach = None
        q_push = None

        try:
            result = self._approach_future.result()
            approach_ok = result.reachable if result else False
            if approach_ok and result.joint_angles:
                q_approach = np.array(result.joint_angles[:7])
            if not approach_ok and result and result.message:
                self._node.get_logger().debug(
                    f"Reachability (approach): {result.message}"
                )
        except Exception as e:
            self._node.get_logger().debug(f"Reachability (approach) failed: {e}")
        self._approach_future = None

        try:
            result = self._push_future.result()
            push_ok = result.reachable if result else False
            if push_ok and result.joint_angles:
                q_push = np.array(result.joint_angles[:7])
            if not push_ok and result and result.message:
                self._node.get_logger().debug(f"Reachability (push): {result.message}")
        except Exception as e:
            self._node.get_logger().debug(f"Reachability (push) failed: {e}")
        self._push_future = None

        if not approach_ok or not push_ok or q_approach is None or q_push is None:
            self._reachable = False
            return

        # Interpolate joint-space path and check limits at every step.
        self._reachable = self._path_within_limits(q_approach, q_push)

    # ------------------------------------------------------------------
    @staticmethod
    def _path_within_limits(q_start, q_end):
        """Check that a linear joint-space interpolation stays within limits."""
        lo = JOINT_LOWER + JOINT_LIMIT_MARGIN
        hi = JOINT_UPPER - JOINT_LIMIT_MARGIN

        for t in np.linspace(0.0, 1.0, INTERP_STEPS):
            q = q_start + t * (q_end - q_start)
            if np.any(q < lo) or np.any(q > hi):
                return False
        return True
