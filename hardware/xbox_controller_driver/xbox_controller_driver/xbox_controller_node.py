import rclpy
from rclpy.node import Node
from std_msgs.msgs import Bool


class xboxControllerNode(Node):
    def __init__(self):
        super().__init__("xbox_controller_node")
        self.estop_publisher = self.create_publisher(Bool, "estop", 10)
        self.estop_timer = self.create_timer(1.0, self.estop_pub)

    def estop_pub(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = xboxControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
