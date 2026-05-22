#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
