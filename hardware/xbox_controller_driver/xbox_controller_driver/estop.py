import rclpy
from rclpy.node import Node
from std_msgs.msgs import Bool


class EstopNode(Node):
    def __init__(self):
        super().__init__("estop_node")
        self.estop_publisher = self.create_publisher(Bool, "estop", 10)
        self.estop_timer = self.create_timer(1.0, self.estop_pub)

    def estop_pub(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = EstopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
