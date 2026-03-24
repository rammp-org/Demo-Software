import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Vector3Stamped
import tf2_ros
from tf2_geometry_msgs import do_transform_vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


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
        self.twist_pub = self.create_publisher(Twist, "/twist_controller/commands", 10)
        self.home_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        # Gripper Action Client
        self.gripper_client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )

    def estop_pub(self):
        # msg = Bool()
        pass

    def go_home(self):
        # A. Switch to Trajectory Controller
        req = SwitchController.Request()
        req.activate_controllers = ["joint_trajectory_controller"]
        req.deactivate_controllers = ["twist_controller"]
        req.strictness = 1  # STRICT
        self.switch_client.call_async(req)

        # B. Send Home Trajectory
        msg = JointTrajectory()
        msg.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            0.0,
            0.26,
            2.26,
            0.0,
            0.95,
            0.0,
        ]  # Gen3 Home    --> may need to be adjusted for 7 dof arm
        point.time_from_start = Duration(sec=3, nanosec=0)
        msg.points.append(point)

        self.get_logger().info("Homing... Waiting 4s.")
        self.home_pub.publish(msg)

        # C. Switch back to Twist after move (Simplified: call again after delay)
        # In a real app, you'd wait for trajectory completion.
        self.create_timer(
            4.0, self.reactivate_twist
        )  # is this timer/callback meant to run indef every 4s? Why?

    def reactivate_twist(self):
        req = SwitchController.Request()
        req.activate_controllers = ["twist_controller"]
        req.deactivate_controllers = ["joint_trajectory_controller"]
        self.switch_client.call_async(req)
        self.get_logger().info("Joystick Control Reactivated.")

    def joy_callback(self, msg):  # includes twist publishing
        # --- PRINTING AXES ---
        axes_str = " | ".join(
            [f"Axis {i}: {val:.2f}" for i, val in enumerate(msg.axes)]
        )
        self.get_logger().info(f"Readings: {axes_str}")
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

            # --- Gripper Control (Buttons) ---
            # msg.buttons[6] = A (Close), msg.buttons[7] = B (Open)
            if msg.buttons[4] == 1 and self.last_button_state[4] == 0:
                self.send_gripper_goal(0.8)  # 0.8 = Fully Closed
            elif msg.buttons[5] == 1 and self.last_button_state[5] == 0:
                self.send_gripper_goal(0.0)  # 0.0 = Fully Open

            self.last_button_state = msg.buttons

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
