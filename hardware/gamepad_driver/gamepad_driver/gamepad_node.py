import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Vector3Stamped
import tf2_ros
from tf2_geometry_msgs import do_transform_vector3
from trajectory_msgs.msg import JointTrajectory
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from arm_interfaces.srv import SetMode
from arm_interfaces.action import ReachPreset


class gamepadNode(Node):
    def __init__(self):
        super().__init__("gamepad_node")

        # estop
        self.estop_publisher = self.create_publisher(Bool, "estop", 10)
        self.estop_timer = self.create_timer(1.0, self.estop_pub)

        # joy node
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.last_button_state = [0] * 12  # Adjust based on your controller

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Controller Switcher Client
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        # Arm Velocity Publisher
        self.twist_pub = self.create_publisher(Twist, "/arm/xbox/twist", 10)
        self.home_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        # Gripper Action Client
        self.gripper_client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )

        self.homing_client = ActionClient(self, ReachPreset, "/arm/reach_preset")
        self.client = self.create_client(SetMode, "/arm/set_mode")

        self.send_manual_control_request()  # upon init, be in manual mode

    def estop_pub(self):
        # msg = Bool()
        pass

    def go_home(self):
        self.homing_client.wait_for_server()

        goal_msg = ReachPreset.Goal()
        goal_msg.preset = 0

        self.send_goal_future = self.homing_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepeted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback.joint_states}")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.success}")

    def send_manual_control_request(self):
        # Wait until service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = SetMode.Request()
        request.mode = 5
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):  # can be use by any service client
        response = future.result()
        if response.success:
            self.get_logger().info("Service call success")
        else:
            self.get_logger().info("Service call failed")

    def reactivate_twist(self):
        req = SwitchController.Request()
        req.activate_controllers = ["twist_controller"]
        req.deactivate_controllers = ["joint_trajectory_controller"]
        self.switch_client.call_async(req)
        self.get_logger().info("Joystick Control Reactivated.")

    def joy_callback(self, msg):  # includes twist publishing
        # --- PRINTING AXES ---
        # axes_str = " | ".join(
        #     [f"Axis {i}: {val:.2f}" for i, val in enumerate(msg.axes)]
        # )
        # self.get_logger().info(f"Readings: {axes_str}")
        try:
            # --- Arm Control (Twist) ---
            # twist = Twist() Not used anywhere??
            scale = 0.2  # Max linear speed (m/s)
            ang_scale = 20.0  # rad/s

            # 1. Look up transform from World to End Effector
            transform = self.tf_buffer.lookup_transform(
                "end_effector_link",  # may need to change  --> actual frame name for kinova gen3, may need to change
                "world",  # may need to change  --> actual frame name for kinova gen3, may need to change
                rclpy.time.Time(),
            )

            # only one will work
            if abs(msg.axes[3]) - abs(msg.axes[4]) > 0.5:
                msg.axes[4] = 0.0
            elif abs(msg.axes[4]) - abs(msg.axes[3]) > 0.5:
                msg.axes[3] = 0.0

            # 2. Create a Vector3Stamped for the Linear Joystick Input
            # do_transform_vector3 REQUIREs the .vector attribute found in Stamped messages
            world_linear_stamped = Vector3Stamped()
            world_linear_stamped.header.frame_id = (
                "world"  # actual frame name for kinova gen3, may need to change
            )
            world_linear_stamped.vector.x = (
                msg.axes[1] * scale
            )  # Map to your specific axis
            world_linear_stamped.vector.y = msg.axes[0] * scale
            world_linear_stamped.vector.z = msg.axes[3] * scale

            # 3. Transform the Stamped Vector
            tool_linear_stamped = do_transform_vector3(world_linear_stamped, transform)

            # 4. Map Angular Input (Directly to Tool Frame)
            tool_angular = Vector3()
            tool_angular.x = msg.axes[5] * ang_scale  # Roll
            tool_angular.y = msg.axes[2] * -ang_scale  # Pitch
            tool_angular.z = msg.axes[4] * -ang_scale  # Yaw

            # 5. Build and Publish the Twist
            final_twist = Twist()
            # Extract the raw vector from the transformed stamped message
            final_twist.linear = tool_linear_stamped.vector
            final_twist.angular = tool_angular

            self.twist_pub.publish(final_twist)

            # --- HOME BUTTON LOGIC ---
            # msg.buttons[3] is typically X or Square
            if msg.buttons[3] == 1 and self.last_home_button_state == 0:
                self.go_home()
            self.last_home_button_state = msg.buttons[3]

            # # --- Gripper Control (Buttons) ---
            # # msg.buttons[6] = A (Close), msg.buttons[7] = B (Open)
            # if msg.buttons[4] == 1 and self.last_button_state[4] == 0:
            #     self.send_gripper_goal(0.8)  # 0.8 = Fully Closed
            # elif msg.buttons[5] == 1 and self.last_button_state[5] == 0:
            #     self.send_gripper_goal(0.0)  # 0.0 = Fully Open

            # self.last_button_state = msg.buttons

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warn(f"Waiting for TF: {e}", throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(f"Unexpected Error: {e}")

    def send_gripper_goal(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server not available")
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 100.0
        self.gripper_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = gamepadNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
