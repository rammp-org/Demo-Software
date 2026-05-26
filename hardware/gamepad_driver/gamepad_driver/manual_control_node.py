#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")
        # joy node
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.buttons_array = []

    def joy_callback(self, msg):
        self.buttons_array = msg.buttons
        direction = msg.axes[5]
        self.buttons_array.insert(0, direction)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
