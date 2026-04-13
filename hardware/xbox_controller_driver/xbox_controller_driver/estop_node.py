#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import keyboard
from rclpy.qos import QoSProfile, DurabilityPolicy


class EstopNode(Node):
    def __init__(self):
        super().__init__("estop_node")

        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.estop_publisher = self.create_publisher(Bool, "estop", qos_profile)
        self.estop_pressed = False

        # upon init publish estop
        msg = Bool()
        msg.data = self.estop_pressed
        self.estop_publisher.publish(msg)

        keyboard.add_hotkey("q", self._on_press)

    def _on_press(self):
        self.estop_pressed = not self.estop_pressed
        msg = Bool()
        msg.data = self.estop_pressed
        self.estop_publisher.publish(msg)


def main(args=None):
    rclpy.init()
    node = EstopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
