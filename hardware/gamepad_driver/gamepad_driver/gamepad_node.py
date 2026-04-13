import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
import tf2_ros
from gui_interfaces.srv import UserInputs
from rclpy.callback_groups import ReentrantCallbackGroup


class gamepadNode(Node):
    def __init__(self):
        super().__init__("gamepad_node")

        self.button_press_time = {}  # tracks when preset buttons for homing positions were first pressed
        self.hold_duration = 2.0  # min time required for user to hold down buttons to go to preset locations

        # joy node
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.last_button_state = [0] * 12

        self.declare_parameter("home_button_index", 0)
        self.home_button_index = (
            self.get_parameter("home_button_index").get_parameter_value().integer_value
        )
        self.declare_parameter("manual_control_button_index", 1)
        self.manual_control_button_index = (
            self.get_parameter("manual_control_button_index")
            .get_parameter_value()
            .integer_value
        )
        self.declare_parameter("open_gripper_button_index", 4)
        self.open_gripper_button_index = (
            self.get_parameter("open_gripper_button_index")
            .get_parameter_value()
            .integer_value
        )
        self.declare_parameter("close_gripper_button_index", 5)
        self.close_gripper_button_index = (
            self.get_parameter("close_gripper_button_index")
            .get_parameter_value()
            .integer_value
        )

        # make sure the button indices are valid < 12 and not the same
        button_indices = [
            self.home_button_index,
            self.manual_control_button_index,
            self.open_gripper_button_index,
            self.close_gripper_button_index,
        ]
        if any(index < 0 or index >= 12 for index in button_indices):
            self.get_logger().error("Button indices must be between 0 and 11.")
            raise ValueError("Button indices must be between 0 and 11.")
        if len(set(button_indices)) != len(button_indices):
            self.get_logger().error(
                "Button indices for home, manual control, open gripper, and close gripper must be unique."
            )
            raise ValueError(
                "Button indices for home, manual control, open gripper, and close gripper must be unique."
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._cb_group = ReentrantCallbackGroup()
        # Arm Velocity Publisher
        self.twist_pub = self.create_publisher(
            Twist, "/arm/xbox/twist", 10, callback_group=self._cb_group
        )

        # UserInputs Service Client
        self.user_input_service_client = self.create_client(
            UserInputs, "/GuiBridge/user_input", callback_group=self._cb_group
        )
        self.get_logger().info("gamepad Node initialized...")

    def openGripper(self):
        self.get_logger().info("Requesting gripper to open")
        self.send_user_input(UserInputs.Request.ARM_OPEN_GRIPPER)

    def closeGripper(self):
        self.get_logger().info("Requesting gripper to close")
        self.send_user_input(UserInputs.Request.ARM_CLOSE_GRIPPER)

    def send_user_input(self, input: str):
        self.get_logger().info(f"Sending user input to ROS: {input}")
        if self.user_input_service_client.wait_for_service(timeout_sec=1.0):
            request = UserInputs.Request()
            request.input = input
            future = self.user_input_service_client.call_async(request)
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            event.wait(timeout=5.0)
            if not future.done():
                self.get_logger().error("User input service call timed out.")
                return False
            if future.result() is not None:
                self.get_logger().info(f"User input '{input}' sent successfully.")
                return future.result().success
            else:
                self.get_logger().error("User input service call failed.")
                return False
        else:
            self.get_logger().error("User input service is not available.")
            return False

    def send_manual_control_request(self):
        self.get_logger().info("Requesting manual arm control on/off")
        self.send_user_input(UserInputs.Request.ARM_MANUAL_ON)

    def request_home(self):
        self.get_logger().info("Requesting arm to move to home position")
        self.send_user_input(UserInputs.Request.ARM_HOME)

    def joy_callback(self, msg):  # includes twist publishing
        try:
            # --- Arm Control (Twist) ---
            scale = 0.2  # Max linear speed (m/s)
            ang_scale = 20.0  # rad/s

            sensitivity_level = 0.8

            # lessen sensitivity of left joystick x and y
            if abs(msg.axes[0]) - abs(msg.axes[1]) > sensitivity_level:
                msg.axes[1] = 0.0
            elif abs(msg.axes[1]) - abs(msg.axes[0]) > sensitivity_level:
                msg.axes[0] = 0.0

            # lessen sensitivity of right joystick x and y
            if abs(msg.axes[2]) - abs(msg.axes[3]) > sensitivity_level:
                msg.axes[3] = 0.0
            elif abs(msg.axes[3]) - abs(msg.axes[2]) > sensitivity_level:
                msg.axes[2] = 0.0

            final_twist = Twist()

            # Map Angular Input (Directly to Tool Frame)
            tool_angular = Vector3()
            tool_angular.x = msg.axes[5] * -ang_scale  # Pitch
            tool_angular.y = msg.axes[2] * -ang_scale  # Yaw
            tool_angular.z = msg.axes[4] * -ang_scale  # Roll

            # Map linear input
            final_twist.linear.x = msg.axes[3] * scale
            final_twist.linear.y = msg.axes[0] * scale
            final_twist.linear.z = msg.axes[1] * scale

            final_twist.angular = tool_angular
            self.twist_pub.publish(final_twist)

            # --- HOME BUTTON LOGIC ---
            current_time = (
                self.get_clock().now().nanoseconds / 1e9
            )  # convert to seconds

            for button_index in enumerate(
                [self.home_button_index, self.manual_control_button_index]
            ):
                if msg.buttons[button_index] == 1:
                    if self.last_button_state[button_index] == 0:
                        # Button just pressed — record the time
                        self.button_press_time[button_index] = current_time
                    else:
                        # Button still held — check if held long enough
                        press_duration = current_time - self.button_press_time.get(
                            button_index, current_time
                        )
                        if press_duration >= self.hold_duration:
                            if button_index == self.home_button_index:
                                self.request_home()
                            elif button_index == self.manual_control_button_index:
                                self.send_manual_control_request()
                            self.button_press_time.pop(
                                button_index
                            )  # reset so it doesn't fire repeatedly
                else:
                    # Button released — clear the timer
                    self.button_press_time.pop(button_index, None)

            # --- Gripper Control (Buttons) ---
            if (
                msg.buttons[self.close_gripper_button_index] == 1
                and self.last_button_state[self.close_gripper_button_index] == 0
            ):
                self.closeGripper()
            elif (
                msg.buttons[self.open_gripper_button_index] == 1
                and self.last_button_state[self.open_gripper_button_index] == 0
            ):
                self.openGripper()

            self.last_button_state = list(msg.buttons)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warn(f"Waiting for TF: {e}", throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(f"Unexpected Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = gamepadNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
