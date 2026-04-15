#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pynput import keyboard
from rclpy.qos import QoSProfile, DurabilityPolicy


class EstopNode(Node):
    def __init__(self):
        super().__init__("estop_node")

        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.estop_publisher = self.create_publisher(Bool, "estop", qos_profile)
        self.estop_pressed = False

        # publish initial state
        self.publish_estop()

        # start keyboard listener (non-blocking)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def publish_estop(self):
        msg = Bool()
        msg.data = self.estop_pressed
        self.estop_publisher.publish(msg)

    def on_press(self, key):
        if key == keyboard.Key.enter:
            self.estop_pressed = not self.estop_pressed
            self.publish_estop()


def main(args=None):
    rclpy.init()
    node = EstopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
