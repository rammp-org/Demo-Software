"""Minimal LUCI stand-in for stripped-down demos.

Registers a node named ``interface`` (ROS graph name ``/interface``) so that
``MEBot_control_node.check_luci_node`` reports the LUCI node as active, and
serves the four ``/luci/*_remote_input`` services (``std_srvs/srv/Empty``)
that ``base_control_node`` calls on startup/shutdown, so those async calls
resolve cleanly instead of logging failures.

It intentionally does NOT drive the chair: there is no consumer of
``/luci/remote_joystick`` and nothing is published to ``/luci/joystick_position``.
This node is scaffolding to pass the system_control readiness gate, not a
functional LUCI replacement.
"""

import rclpy
import rclpy.node
from std_srvs.srv import Empty

# Service names created by MEBot_control_node (verified MEBot_control_node.py:305-315).
LUCI_REMOTE_INPUT_SERVICES = [
    "/luci/set_auto_remote_input",
    "/luci/remove_auto_remote_input",
    "/luci/set_shared_remote_input",
    "/luci/remove_shared_remote_input",
]


class LuciMock(rclpy.node.Node):
    """A no-op node named ``/interface`` that satisfies the LUCI diagnostic."""

    def __init__(self):
        super().__init__("interface")
        self._services = [
            self.create_service(Empty, name, self._noop)
            for name in LUCI_REMOTE_INPUT_SERVICES
        ]
        self.get_logger().info(
            "LUCI mock running as /interface; serving "
            f"{len(LUCI_REMOTE_INPUT_SERVICES)} /luci/*_remote_input services (no chair drive)"
        )

    def _noop(self, request, response):
        """Accept any remote-input request and return an empty success response."""
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LuciMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
