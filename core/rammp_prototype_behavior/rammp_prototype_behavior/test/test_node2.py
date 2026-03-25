#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class testNode2(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("test_node2")  # MODIFY NAME
        self.get_logger().info("Test node2 running...")


def main(args=None):
    rclpy.init(args=args)
    node = testNode2()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
