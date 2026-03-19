"""Drinking node that exposes action servers for cup manipulation.

This node provides action servers matching the Drinking Node spec:
  - /arm/drink/grab_cup_from_table
  - /arm/drink/bring_cup_to_mouth
  - /arm/drink/home_cup
  - /arm/drink/put_cup_back_to_holder
  - /arm/drink/pickup_and_order

Each action server sends joint/cartesian move commands to the arm
via ArmInterfaceClient, or runs them in PyBullet simulation.
"""

import traceback
from pathlib import Path

import rclpy
import rclpy.node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pybullet_helpers.geometry import Pose

from cornell_feeding_interfaces.action import (
    BringCupToMouth,
    GrabCupFromTable,
    HomeCup,
    PickupAndOrder,
    PutCupBackToHolder,
)

from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.control.robot_controller.command_interface import (
    CartesianCommand,
    CloseGripperCommand,
    JointCommand,
    OpenGripperCommand,
)
from rammp.simulation.scene_description import create_scene_description_from_config
from rammp.simulation.simulator import FeedingDeploymentPyBulletSimulator


class DrinkingNode(rclpy.node.Node):
    """ROS 2 node that exposes action servers for cup manipulation."""

    def __init__(self):
        super().__init__("drinking_node")
        self.get_logger().info("Drinking Node starting up...")

        self.declare_parameter("scene_config", "wheelchair")
        self.declare_parameter("run_on_robot", True)
        self.declare_parameter("use_gui", False)

        scene_config = self.get_parameter("scene_config").value
        self._run_on_robot = self.get_parameter("run_on_robot").value
        use_gui = self.get_parameter("use_gui").value

        # Initialize scene description and simulator.
        scene_config_path = (
            Path(__file__).resolve().parents[3]
            / "src" / "rammp" / "simulation" / "configs"
            / f"{scene_config}.yaml"
        )
        self._scene_description = create_scene_description_from_config(str(scene_config_path))
        self._sim = FeedingDeploymentPyBulletSimulator(
            self._scene_description, use_gui=use_gui, ignore_user=True
        )

        # Initialize robot interface.
        if self._run_on_robot:
            self._arm = ArmInterfaceClient(node=self)
        else:
            self._arm = None

        # Set up action servers.
        self._action_group = ReentrantCallbackGroup()

        self._grab_cup_action = ActionServer(
            self,
            GrabCupFromTable,
            "/arm/drink/grab_cup_from_table",
            self._on_grab_cup,
            callback_group=self._action_group,
        )

        self._bring_cup_action = ActionServer(
            self,
            BringCupToMouth,
            "/arm/drink/bring_cup_to_mouth",
            self._on_bring_cup_to_mouth,
            callback_group=self._action_group,
        )

        self._home_cup_action = ActionServer(
            self,
            HomeCup,
            "/arm/drink/home_cup",
            self._on_home_cup,
            callback_group=self._action_group,
        )

        self._put_cup_back_action = ActionServer(
            self,
            PutCupBackToHolder,
            "/arm/drink/put_cup_back_to_holder",
            self._on_put_cup_back,
            callback_group=self._action_group,
        )

        self._pickup_and_order_action = ActionServer(
            self,
            PickupAndOrder,
            "/arm/drink/pickup_and_order",
            self._on_pickup_and_order,
            callback_group=self._action_group,
        )

        self.get_logger().info("Drinking Node ready.")

    # -- helpers --

    def _publish_feedback(self, goal_handle, action_type, status_msg: str):
        feedback = action_type.Feedback()
        feedback.status = status_msg
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"Feedback: {status_msg}")

    def _move_joints(self, joint_positions: list[float]):
        """Move to joint positions on robot or in sim."""
        self.get_logger().info(f"Moving joints: {[round(v, 3) for v in joint_positions]}")
        if self._arm is not None:
            self._arm.execute_command(JointCommand(pos=joint_positions))
        else:
            plan = self._sim.plan_to_joint_positions(joint_positions)
            self._sim.visualize_plan(plan)

    def _move_cartesian(self, pose_values: list[float]):
        """Move to cartesian pose on robot or in sim. pose_values = [x,y,z, qx,qy,qz,qw]."""
        pos = tuple(pose_values[:3])
        quat = tuple(pose_values[3:])
        self.get_logger().info(f"Moving cartesian: pos={[round(v, 3) for v in pos]}")
        if self._arm is not None:
            self._arm.execute_command(CartesianCommand(pos=list(pos), quat=list(quat)))
        else:
            plan = self._sim.plan_to_ee_pose(Pose(pos, quat))
            self._sim.visualize_plan(plan)

    def _open_gripper(self):
        self.get_logger().info("Opening gripper")
        if self._arm is not None:
            self._arm.execute_command(OpenGripperCommand())
        else:
            self._sim.robot.open_fingers()

    def _close_gripper(self):
        self.get_logger().info("Closing gripper")
        if self._arm is not None:
            self._arm.execute_command(CloseGripperCommand())
        else:
            self._sim.robot.close_fingers()

    def _set_speed(self, speed: str):
        if self._arm is not None:
            self._arm.set_speed(speed)

    # -- action callbacks --

    def _on_grab_cup(self, goal_handle):
        """Pick up the cup from the table."""
        self.get_logger().info("[GrabCupFromTable] Received goal")
        try:
            self._set_speed("medium")
            self._publish_feedback(goal_handle, GrabCupFromTable, "Moving to retract")
            self._move_joints(self._scene_description.retract_pos)
            self._close_gripper()

            self._publish_feedback(goal_handle, GrabCupFromTable, "Moving to drink gaze")
            self._move_joints(self._scene_description.drink_gaze_pos)

            # Move to staging, open gripper, then home
            self._publish_feedback(goal_handle, GrabCupFromTable, "Grasping cup")
            self._open_gripper()
            self._move_joints(self._scene_description.home_pos)

            if self._arm is not None:
                self._arm.start_maintain_home_orientation()

            goal_handle.succeed()
            result = GrabCupFromTable.Result()
            result.success = True
            result.message = "Cup grabbed from table"
        except Exception as e:
            self.get_logger().error(f"[GrabCupFromTable] Failed: {traceback.format_exc()}")
            goal_handle.abort()
            result = GrabCupFromTable.Result()
            result.success = False
            result.message = str(e)

        self.get_logger().info(f"[GrabCupFromTable] Completed: {result.message}")
        return result

    def _on_bring_cup_to_mouth(self, goal_handle):
        """Transfer the cup toward the user's mouth."""
        self.get_logger().info("[BringCupToMouth] Received goal")
        try:
            self._set_speed("medium")

            if self._arm is not None:
                self._arm.stop_maintain_home_orientation()

            self._publish_feedback(goal_handle, BringCupToMouth, "Moving to transfer waypoint")
            self._move_joints(self._scene_description.drink_transfer_waypoint_pos)

            self._publish_feedback(goal_handle, BringCupToMouth, "Moving to before-transfer position")
            self._move_joints(self._scene_description.drink_before_transfer_pos)

            self._publish_feedback(goal_handle, BringCupToMouth, "Returning to waypoint")
            self._move_joints(self._scene_description.drink_transfer_waypoint_pos)

            self._publish_feedback(goal_handle, BringCupToMouth, "Returning home")
            self._move_joints(self._scene_description.home_pos)

            if self._arm is not None:
                self._arm.start_maintain_home_orientation()

            goal_handle.succeed()
            result = BringCupToMouth.Result()
            result.success = True
            result.message = "Cup brought to mouth"
        except Exception as e:
            self.get_logger().error(f"[BringCupToMouth] Failed: {traceback.format_exc()}")
            goal_handle.abort()
            result = BringCupToMouth.Result()
            result.success = False
            result.message = str(e)

        self.get_logger().info(f"[BringCupToMouth] Completed: {result.message}")
        return result

    def _on_home_cup(self, goal_handle):
        """Return the arm to the home position."""
        self.get_logger().info("[HomeCup] Received goal")
        try:
            self._publish_feedback(goal_handle, HomeCup, "Moving to home position")
            self._move_joints(self._scene_description.home_pos)

            goal_handle.succeed()
            result = HomeCup.Result()
            result.success = True
            result.message = "Arm returned to home position"
        except Exception as e:
            self.get_logger().error(f"[HomeCup] Failed: {traceback.format_exc()}")
            goal_handle.abort()
            result = HomeCup.Result()
            result.success = False
            result.message = str(e)

        self.get_logger().info(f"[HomeCup] Completed: {result.message}")
        return result

    def _on_pickup_and_order(self, goal_handle):
        """Pick up the cup from the wheelchair holder."""
        self.get_logger().info("[PickupAndOrder] Received goal")
        try:
            self._set_speed("medium")

            self._publish_feedback(goal_handle, PickupAndOrder, "Moving home")
            self._move_joints(self._scene_description.home_pos)

            self._publish_feedback(goal_handle, PickupAndOrder, "Moving outside handle")
            self._move_joints(self._scene_description.outside_drink_handle_pos)
            self._close_gripper()

            self._publish_feedback(goal_handle, PickupAndOrder, "Moving below handle")
            self._move_cartesian(self._scene_description.below_drink_handle_pose)

            self._publish_feedback(goal_handle, PickupAndOrder, "Moving inside handle")
            self._move_cartesian(self._scene_description.inside_drink_handle_pose)

            self._publish_feedback(goal_handle, PickupAndOrder, "Grasping cup")
            self._open_gripper()

            self._publish_feedback(goal_handle, PickupAndOrder, "Lifting cup")
            self._move_cartesian(self._scene_description.above_drink_handle_pose)

            self._publish_feedback(goal_handle, PickupAndOrder, "Moving home")
            self._move_joints(self._scene_description.home_pos)

            goal_handle.succeed()
            result = PickupAndOrder.Result()
            result.success = True
            result.message = "Pickup and order completed"
        except Exception as e:
            self.get_logger().error(f"[PickupAndOrder] Failed: {traceback.format_exc()}")
            goal_handle.abort()
            result = PickupAndOrder.Result()
            result.success = False
            result.message = str(e)

        self.get_logger().info(f"[PickupAndOrder] Completed: {result.message}")
        return result

    def _on_put_cup_back(self, goal_handle):
        """Place the cup back to the wheelchair holder."""
        self.get_logger().info("[PutCupBackToHolder] Received goal")
        try:
            self._set_speed("medium")

            if self._arm is not None:
                self._arm.stop_maintain_home_orientation()

            self._publish_feedback(goal_handle, PutCupBackToHolder, "Moving home")
            self._move_joints(self._scene_description.home_pos)

            self._publish_feedback(goal_handle, PutCupBackToHolder, "Moving above handle")
            self._move_joints(self._scene_description.above_drink_handle_pos)

            self._publish_feedback(goal_handle, PutCupBackToHolder, "Placing inside handle")
            self._move_cartesian(self._scene_description.inside_drink_handle_pose)

            self._publish_feedback(goal_handle, PutCupBackToHolder, "Releasing cup")
            self._close_gripper()

            self._publish_feedback(goal_handle, PutCupBackToHolder, "Moving below handle")
            self._move_cartesian(self._scene_description.below_drink_handle_pose)

            self._publish_feedback(goal_handle, PutCupBackToHolder, "Moving outside handle")
            self._move_cartesian(self._scene_description.outside_drink_handle_pose)

            self._publish_feedback(goal_handle, PutCupBackToHolder, "Moving home")
            self._move_joints(self._scene_description.home_pos)

            goal_handle.succeed()
            result = PutCupBackToHolder.Result()
            result.success = True
            result.message = "Cup placed back at wheelchair holder"
        except Exception as e:
            self.get_logger().error(f"[PutCupBackToHolder] Failed: {traceback.format_exc()}")
            goal_handle.abort()
            result = PutCupBackToHolder.Result()
            result.success = False
            result.message = str(e)

        self.get_logger().info(f"[PutCupBackToHolder] Completed: {result.message}")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DrinkingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
