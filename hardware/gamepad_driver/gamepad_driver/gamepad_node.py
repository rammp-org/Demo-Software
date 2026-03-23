import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class gamepadNode(Node):
    def __init__(self):
        super().__init__("gamepad_node")

        self.buttons = None
        self.axes = None

        self.estop_publisher = self.create_publisher(Bool, "estop", 10)
        self.estop_timer = self.create_timer(1.0, self.estop_pub)

        self.twist_publisher = self.create_publisher(Twist, "twist", 10)
        self.twist_timer = self.create_timer(1.0, self.twist_pub)

        self.joy_pub = self.create_publisher(Float64MultiArray, "gamepad_inputs", 10)
        self.joy_timer = self.create_timer(1.0, self.joy_pub_callback)

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_sub_callback, 10)

    def estop_pub(self):
        # msg = Bool()
        pass

    def twist_pub(self):
        # msg = Twist()
        # msg.linear = None
        # msg.angular = None
        # self.twist_publisher.publish(msg)
        pass

    def joy_pub_callback(self):
        if self.buttons is None:
            return
        if self.axes is None:
            return
        msg = Float64MultiArray()
        msg.data = [float(b) for b in self.buttons]
        # msg.data = [float(b) for b in self.buttons]
        self.joy_pub.publish(msg)

    def joy_sub_callback(self, msg):
        self.buttons = msg.buttons
        self.axes = msg.axes


def main(args=None):
    rclpy.init(args=args)
    node = gamepadNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
