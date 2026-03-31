import rclpy
import rclpy.action
import rclpy.node

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool


class CupStabilizerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("cup_stabilizer_node")
        self.get_logger().info("Mock Cup Stabilizer Node has been started.")
        self._callback_group = ReentrantCallbackGroup()

        self.create_service(
            SetBool,
            "/arm/drink/stabilize/enable",
            self._srv_stabilize_enable,
            callback_group=self._callback_group,
        )

    def _srv_stabilize_enable(self, request, response):
        if request.data:
            self.get_logger().info("Received request to enable cup stabilization.")
            # Simulate enabling cup stabilization (e.g., by starting a thread or setting a flag)
            response.success = True
            response.message = "Cup stabilization enabled."
        else:
            self.get_logger().info("Received request to disable cup stabilization.")
            # Simulate disabling cup stabilization (e.g., by stopping a thread or clearing a flag)
            response.success = True
            response.message = "Cup stabilization disabled."
        return response


def main(args=None):
    """Entry point for the mock cup stabilizer node."""
    rclpy.init(args=args)
    node = CupStabilizerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
