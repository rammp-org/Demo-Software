"""Async reachability checker that delegates to the arm driver's Kortex IK solver.

Checks both stages of the button push sequence — the approach pose (button
surface) and the push pose (button + PUSH_MAX overshoot along the push
direction) — matching what the ButtonPushController actually commands.

Both requests fire in cycle *N* and responses are picked up in cycle *N+1*
(one-frame latency, ~200 ms at 5 Hz — negligible).
"""

import numpy as np
from scipy.spatial.transform import Rotation

from arm_interfaces.srv import CheckReachability

# Must match button_push_controller.py
PUSH_MAX = 0.10  # metres — max push overshoot past button surface


class ReachabilityChecker:
    """Non-blocking IK reachability check via ``/arm/check_reachability``.

    Checks two poses per cycle (approach + push) to match the actual
    motion the arm will execute.

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
        self._approach_ok = False
        self._push_ok = False
        self._service_warned = False

    # ------------------------------------------------------------------
    def check_async(self, approach_xyz, approach_quat):
        """Fire async reachability checks for both the approach and push poses.

        The approach pose is the button surface. The push pose is computed by
        offsetting PUSH_MAX along the tool z-axis (push direction), exactly as
        ButtonPushController does.

        Safe to call every detection cycle.  If previous requests are still
        in-flight they are left alone (not duplicated).

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
            self._approach_ok = False
            self._push_ok = False
            return

        if self._service_warned:
            self._node.get_logger().info("Reachability service now available")
            self._service_warned = False

        # Don't fire new requests while any are in-flight.
        approach_busy = self._approach_future is not None and not self._approach_future.done()
        push_busy = self._push_future is not None and not self._push_future.done()
        if approach_busy or push_busy:
            return

        # 1) Approach pose — button surface
        req_approach = CheckReachability.Request()
        req_approach.target_pose.position.x = float(approach_xyz[0])
        req_approach.target_pose.position.y = float(approach_xyz[1])
        req_approach.target_pose.position.z = float(approach_xyz[2])
        req_approach.target_pose.orientation.x = float(approach_quat[0])
        req_approach.target_pose.orientation.y = float(approach_quat[1])
        req_approach.target_pose.orientation.z = float(approach_quat[2])
        req_approach.target_pose.orientation.w = float(approach_quat[3])
        self._approach_future = self._client.call_async(req_approach)

        # 2) Push pose — button surface + PUSH_MAX along tool z-axis
        rot = Rotation.from_quat(approach_quat)
        push_dir = rot.as_matrix()[:, 2]
        push_xyz = np.asarray(approach_xyz, dtype=np.float64) + push_dir * PUSH_MAX

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
        """Return True only if both approach and push poses are reachable."""
        self._poll()
        return self._approach_ok and self._push_ok

    # ------------------------------------------------------------------
    def _poll(self):
        """Check if in-flight futures have completed and update the cache."""
        if self._approach_future is not None and self._approach_future.done():
            try:
                result = self._approach_future.result()
                self._approach_ok = result.reachable if result else False
                if not self._approach_ok and result and result.message:
                    self._node.get_logger().debug(
                        f"Reachability (approach): {result.message}"
                    )
            except Exception as e:
                self._node.get_logger().debug(f"Reachability (approach) failed: {e}")
                self._approach_ok = False
            self._approach_future = None

        if self._push_future is not None and self._push_future.done():
            try:
                result = self._push_future.result()
                self._push_ok = result.reachable if result else False
                if not self._push_ok and result and result.message:
                    self._node.get_logger().debug(
                        f"Reachability (push): {result.message}"
                    )
            except Exception as e:
                self._node.get_logger().debug(f"Reachability (push) failed: {e}")
                self._push_ok = False
            self._push_future = None
