import rclpy
from rammp_prototype_interfaces.action import CurbTraverse

# custom msgs/srvs
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Int64
import time


class TestActionServer(Node):
    def __init__(self):
        super().__init__("test_action_server")

        self.CA_flag = 0
        self.curb_traverse_action = ActionServer(
            self, CurbTraverse, "curb_traverse", self.curb_traverse_action_callback
        )

        self.dummy_sub = self.create_subscription(
            Int64, "dummy_ca_flag", self.dummy_sub_callback, 10
        )

    def dummy_sub_callback(self, msg):
        self.CA_flag = msg.data
        self.get_logger().info(msg.data)

    def curb_traverse_action_callback(self, goal):
        if goal.request.direction == 1:
            self.get_logger().info("Curb Ascending enabled")
        if goal.request.direction == 0:
            self.get_logger().info("Curb Descending enabled")

        feedback_msg = CurbTraverse.Feedback()
        result = CurbTraverse.Result()

        # Poll CA_flag until the final step is reached
        while self.CA_flag != 6:
            if goal.is_cancel_requested:
                goal.canceled()
                self.get_logger().info("Action cancelled")
                result.success = False
                return result

            feedback_msg.ca_flag = self.CA_flag
            goal.publish_feedback(feedback_msg)

            time.sleep(0.05)

        goal.succeed()
        result.success = True
        self.get_logger().info("Action successful")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TestActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
