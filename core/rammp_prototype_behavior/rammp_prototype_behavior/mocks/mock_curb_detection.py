import rclpy
import rclpy.action
import rclpy.node

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool


class PerceptionCurbDetectionNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("perception_curb_detection_node")
        self.get_logger().info("Mock Perception Curb Detection Node has been started.")
        self._mock_action_reject = False  # default to accept all goals
        self._mock_action_result = True  # default to successful execution
        self._action_running = False
        self._action_counter = 0
        self.cb_group = ReentrantCallbackGroup()
        self.create_service(
            SetBool,
            "/nav/curb/detect",
            self._srv_detection_enable,
            callback_group=self.cb_group,
        )

    def _srv_detection_enable(self, request, response):
        if request.data:
            self.get_logger().info("Curb detection enabled.")
        else:
            self.get_logger().info("Curb detection disabled.")
        response.success = True
        return response


def main(args=None):
    """Entry point for the mock curb detection node."""
    rclpy.init(args=args)
    node = PerceptionCurbDetectionNode()
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
