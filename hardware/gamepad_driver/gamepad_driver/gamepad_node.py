import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
import tf2_ros
from gui_interfaces.srv import UserInputs


class gamepadNode(Node):
    def __init__(self):
        super().__init__("gamepad_node")

        self.button_press_time = {}  # tracks when preset buttons for homing positions were first pressed
        self.hold_duration = 2.0  # min time required for user to hold down buttons to go to preset locations

        # joy node
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.last_button_state = [0] * 12

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Arm Velocity Publisher
        self.twist_pub = self.create_publisher(Twist, "/arm/xbox/twist", 10)

        # UserInputs Service Client
        self.user_input_service_client = self.create_client(
            UserInputs, "/GuiBridge/user_input", callback_group=self._cb_group
        )
        # self.home_pub = self.create_publisher(
        #     JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        # )

        # self.open_gripper_client = self.create_client(Trigger, "/arm/open_gripper")
        # self.close_gripper_client = self.create_client(Trigger, "/arm/close_gripper")

        # self.preset_client = ActionClient(self, ReachPreset, "/arm/reach_preset")
        # self.in_preset_mode = False

        # self.manual_client = self.create_client(SetMode, "/arm/set_mode")

    # def send_preset(self, value):
    #     if self.in_preset_mode:
    #         return
    #     self.in_preset_mode = True

    #     self.preset_client.wait_for_server()

    #     goal_msg = ReachPreset.Goal()
    #     goal_msg.preset = value

    #     self.send_goal_future = self.preset_client.send_goal_async(
    #         goal_msg, feedback_callback=self.feedback_callback
    #     )

    #     self.send_goal_future.add_done_callback(self.goal_response_callback)

    # def goal_response_callback(self, future):
    #     goal_handle = future.result()

    #     if not goal_handle.accepted:
    #         self.get_logger().info("Goal rejected")
    #         self.in_preset_mode = False
    #         self.send_manual_control_request()
    #         return

    #     self.get_logger().info("Goal accepeted")

    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.result_callback)

    # def feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger().info(f"Feedback: {feedback.joint_states}")

    # def result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f"Result: {result.success}")
    #     self.in_preset_mode = False
    #     self.send_manual_control_request()

    def openGripper(self):
        # self.open_gripper_client.wait_for_service()

        # request = Trigger.Request()
        # future = self.open_gripper_client.call_async(request)
        # future.add_done_callback(self.handle_service_response)
        return

    def closeGripper(self):
        return

    def send_manual_control_request(self):
        return
        # # Wait until service is available
        # while not self.manual_client.wait_for_service():
        #     self.get_logger().info("Service not available, waiting...")

        # request = SetMode.Request()
        # request.mode = 5
        # future = self.manual_client.call_async(request)
        # future.add_done_callback(self.handle_service_response)

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

            for button_index, preset_index in enumerate([0, 1, 2, 3]):
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
                            self.send_preset(preset_index)
                            self.button_press_time.pop(
                                button_index
                            )  # reset so it doesn't fire repeatedly
                else:
                    # Button released — clear the timer
                    self.button_press_time.pop(button_index, None)

            # --- Gripper Control (Buttons) ---
            if msg.buttons[4] == 1 and self.last_button_state[4] == 0:
                self.closeGripper()
            elif msg.buttons[5] == 1 and self.last_button_state[5] == 0:
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
