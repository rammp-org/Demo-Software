#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class testNode1(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("test_node1")  # MODIFY NAME
        self.get_logger().info("Test node1 running...")


def main(args=None):
    rclpy.init(args=args)
    node = testNode1()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
