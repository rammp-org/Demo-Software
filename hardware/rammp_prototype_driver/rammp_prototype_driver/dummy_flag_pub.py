import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import time


class DummyCAFlagNode(Node):
    def __init__(self):
        super().__init__("dummy_ca_flag_node")
        self.dummy_pub = self.create_publisher(Int64, "dummy_ca_flag", 10)
        self.dummy_timer = self.create_timer(1.0, self.dummy_flag_pub)

    def dummy_flag_pub(self):
        msg = Int64()
        msg.data = 1
        self.dummy_pub.publish(msg)
        time.sleep(5)
        msg.data = 6
        self.dummy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyCAFlagNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
