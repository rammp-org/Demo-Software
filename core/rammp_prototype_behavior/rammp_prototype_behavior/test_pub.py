#!/usr/bin/env python3
from std_msgs.msg import String
import rclpy
from rclpy.node import Node


class TestPub(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("test_pub")  # MODIFY NAME
        self.user_input_pub = self.create_publisher(String, "user_input", 10)
        self.timer = self.create_timer(1.0, self.publish_user_input)

    def publish_user_input(self):
        msg = String()
        msg.data = "self level off"  # Change this to test different inputs
        self.user_input_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestPub()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
