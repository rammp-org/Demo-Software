"""Async reachability checker that delegates to the arm driver's Kortex IK solver.

Sends a ``CheckReachability`` service request each detection cycle and caches
the latest result.  Designed to run inside a single-threaded ROS 2 executor
without blocking: the request fires in cycle *N* and the response is picked up
in cycle *N+1* (one-frame latency, ~200 ms at 5 Hz — negligible).
"""

from arm_interfaces.srv import CheckReachability


class ReachabilityChecker:
    """Non-blocking IK reachability check via ``/arm/check_reachability``.

    Parameters
    ----------
    node : rclpy.node.Node
        The owning ROS node (used to create the service client and for logging).
    """

    def __init__(self, node):
        self._node = node
        self._client = node.create_client(CheckReachability, "/arm/check_reachability")
        self._future = None
        self._reachable = False
        self._service_warned = False

    # ------------------------------------------------------------------
    def check_async(self, target_xyz, target_quat):
        """Fire an async reachability check for the given pose.

        Safe to call every detection cycle.  If a previous request is still
        in-flight it is left alone (not duplicated).

        Parameters
        ----------
        target_xyz : array-like, shape (3,)
            Target position [x, y, z] in the robot base frame (metres).
        target_quat : array-like, shape (4,)
            Target orientation as quaternion [x, y, z, w].
        """
        # Harvest any completed future first.
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

        # Don't fire a new request while one is in-flight.
        if self._future is not None and not self._future.done():
            return

        req = CheckReachability.Request()
        req.target_pose.position.x = float(target_xyz[0])
        req.target_pose.position.y = float(target_xyz[1])
        req.target_pose.position.z = float(target_xyz[2])
        req.target_pose.orientation.x = float(target_quat[0])
        req.target_pose.orientation.y = float(target_quat[1])
        req.target_pose.orientation.z = float(target_quat[2])
        req.target_pose.orientation.w = float(target_quat[3])
        self._future = self._client.call_async(req)

    # ------------------------------------------------------------------
    @property
    def is_reachable(self) -> bool:
        """Return the latest cached reachability result."""
        self._poll()
        return self._reachable

    # ------------------------------------------------------------------
    def _poll(self):
        """Check if the in-flight future has completed and update the cache."""
        if self._future is not None and self._future.done():
            try:
                result = self._future.result()
                self._reachable = result.reachable if result else False
                if not self._reachable and result and result.message:
                    self._node.get_logger().debug(
                        f"Reachability check: {result.message}"
                    )
            except Exception as e:
                self._node.get_logger().debug(f"Reachability service call failed: {e}")
                self._reachable = False
            self._future = None
