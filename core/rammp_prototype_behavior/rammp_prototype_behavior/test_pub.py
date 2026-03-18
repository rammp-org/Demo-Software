#!/usr/bin/env python3
from std_msgs.msg import String
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node


class TestPub(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("test_pub")  # MODIFY NAME
        self.user_input_pub = self.create_publisher(String, "user_input", 10)
        self.timer = self.create_timer(1.0, self.publish_user_input)

        self.estop_pub = self.create_publisher(Bool, "estop", 10)
        self.estop_timer = self.create_timer(1.0, self.publish_estop)

    def publish_user_input(self):
        msg = String()
        msg.data = "arm enabled"  # Change this to test different inputs
        self.user_input_pub.publish(msg)

    def publish_estop(self):
        msg = Bool()
        msg.data = False  # Change this to test different inputs
        self.estop_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestPub()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
