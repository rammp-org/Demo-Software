from rclpy.executors import MultiThreadedExecutor
import rclpy
from luci_messages.msg import LuciJoystick

from rclpy.node import Node
from std_msgs.msg import Int32


# LUCI STUFF
JS_FRONT = 0
JS_FRONT_LEFT = 1
JS_FRONT_RIGHT = 2
JS_LEFT = 3
JS_RIGHT = 4
JS_BACK_LEFT = 5
JS_BACK_RIGHT = 6
JS_BACK = 7
JS_ORIGIN = 8

INPUT_REMOTE = 1

JOYSTICK_TOPIC = "luci/remote_joystick"
JOYSTICK_MSG_TYPE = "luci_messages/msg/LuciJoystick"
SET_AUTO_SERVICE = "/luci/set_auto_remote_input"
REMOVE_AUTO_SERVICE = "/luci/remove_auto_remote_input"


def _compute_zone(fb: int, lr: int) -> int:
    if fb == 0 and lr == 0:
        return JS_ORIGIN
    if fb > 0 and lr == 0:
        return JS_FRONT
    if fb < 0 and lr == 0:
        return JS_BACK
    if fb == 0 and lr > 0:
        return JS_RIGHT
    if fb == 0 and lr < 0:
        return JS_LEFT
    if fb > 0 and lr > 0:
        return JS_FRONT_RIGHT
    if fb > 0 and lr < 0:
        return JS_FRONT_LEFT
    if fb < 0 and lr > 0:
        return JS_BACK_RIGHT
    return JS_BACK_LEFT


class LuciHeartBeat(Node):
    def __init__(self):
        super().__init__("luci_heartbeat")

        self.test_pwm = 0

        #### Init all ROS interfaces
        self._init_subscribers()
        self._init_publishers()

    def _init_subscribers(self):
        # subscriptions
        self.manual_seat_control_subscription = self.create_subscription(
            Int32, "dummy_pwm", self.dummy_pwm_callback, 10
        )

    def _init_publishers(self):
        self.luci_js_publisher = self.create_publisher(LuciJoystick, JOYSTICK_TOPIC, 10)

        self.luci_heartbeat_timer = self.create_timer(
            0.005, self.send_joy
        )

    def dummy_pwm_callback(self, msg):
        self.test_pwm = msg.data

    def send_joy(self):
        msg = LuciJoystick()
        msg.forward_back = self.test_pwm
        msg.left_right = 0
        msg.joystick_zone = _compute_zone(msg.forward_back, 0)
        msg.input_source = INPUT_REMOTE
        self.luci_js_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LuciHeartBeat()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
