import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class gamepadNode(Node):
    def __init__(self):
        super().__init__("gamepad_node")

        self.estop_publisher = self.create_publisher(Bool, "estop", 10)
        self.estop_timer = self.create_timer(1.0, self.estop_pub)

        self.twist_publisher = self.create_publisher(Twist, "twist", 10)
        self.twist_timer = self.create_timer(1.0, self.twist_pub)

    def estop_pub(self):
        # msg = Bool()
        pass

    def twist_pub(self):
        msg = Twist()
        msg.linear = None
        msg.angular = None
        self.twist_publisher.publish(msg)
        pass


def main(args=None):
    rclpy.init(args=args)
    node = gamepadNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
