"""Drinking node that exposes action servers for cup manipulation.

This node provides action servers matching the Drinking Node spec:
  - /arm/drink/grab_cup_from_table
  - /arm/drink/bring_cup_to_mouth
  - /arm/drink/home_cup
  - /arm/drink/put_cup_back_to_holder
  - /arm/drink/pickup_and_order

Each action server calls the actual RAMMP HLA implementations for
pick, transfer, and stow.
"""

import shutil
import traceback
from pathlib import Path

import rclpy
import rclpy.node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from cornell_feeding_interfaces.action import (
    BringCupToMouth,
    GrabCupFromTable,
    HomeCup,
    PickupAndOrder,
    PutCupBackToHolder,
)

from rammp.actions.pick_tool import PickToolHLA
from rammp.actions.stow_tool import StowToolHLA
from rammp.actions.transfer_tool import TransferToolHLA
from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.interfaces.perception_interface import PerceptionInterface
from rammp.interfaces.rviz_interface import RVizInterface
from rammp.simulation.scene_description import create_scene_description_from_config
from rammp.simulation.simulator import FeedingDeploymentPyBulletSimulator


class DrinkingNode(rclpy.node.Node):
    """ROS 2 node that exposes action servers backed by RAMMP HLAs."""

    def __init__(self):
        super().__init__("drinking_node")
        self.get_logger().info("Drinking Node starting up...")

        # Declare ROS parameters for configuration.
        self.declare_parameter("scene_config", "wheelchair")
        self.declare_parameter("run_on_robot", True)
        self.declare_parameter("no_waits", True)
        self.declare_parameter("simulate_head_perception", False)
        self.declare_parameter("max_motion_planning_time", 10.0)

        scene_config = self.get_parameter("scene_config").value
        run_on_robot = self.get_parameter("run_on_robot").value
        no_waits = self.get_parameter("no_waits").value
        simulate_head_perception = self.get_parameter("simulate_head_perception").value
        max_motion_planning_time = self.get_parameter("max_motion_planning_time").value

        # Set up log and behavior tree directories.
        self._log_dir = Path(__file__).resolve().parent / "log" / "drinking_node"
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._execution_log = self._log_dir / "execution_log.txt"
        self._execution_log.write_text("")

        self._behavior_tree_dir = self._log_dir / "behavior_trees"
        self._behavior_tree_dir.mkdir(exist_ok=True)
        original_bt_dir = Path(__file__).resolve().parents[3] / "src" / "rammp" / "actions" / "behavior_trees"
        if original_bt_dir.exists():
            for bt_file in original_bt_dir.glob("*.yaml"):
                shutil.copy(bt_file, self._behavior_tree_dir)

        # Initialize robot interface.
        if run_on_robot:
            self._robot_interface = ArmInterfaceClient(node=self)
        else:
            self._robot_interface = None

        # Initialize perception interface.
        self._perception_interface = PerceptionInterface(
            node=self,
            robot_interface=self._robot_interface,
            simulate_head_perception=simulate_head_perception,
            log_dir=self._log_dir,
        )

        # Initialize scene description and simulator.
        scene_config_path = Path(__file__).resolve().parents[3] / "src" / "rammp" / "simulation" / "configs" / f"{scene_config}.yaml"
        self._scene_description = create_scene_description_from_config(str(scene_config_path))
        self._sim = FeedingDeploymentPyBulletSimulator(self._scene_description, use_gui=False, ignore_user=True)

        # Initialize RViz interface.
        if run_on_robot:
            self._rviz_interface = RVizInterface(self, self._scene_description)
        else:
            self._rviz_interface = None

        # Common HLA constructor arguments.
        hla_hyperparams = {"max_motion_planning_time": max_motion_planning_time}
        hla_args = (
            self._sim,
            self._robot_interface,
            self._perception_interface,
            self._rviz_interface,
            None,  # web_interface
            hla_hyperparams,
            no_waits,
            self._log_dir,
            self._behavior_tree_dir,
            self._execution_log,
        )

        # Create HLA instances.
        self._pick_tool_hla = PickToolHLA(*hla_args)
        self._stow_tool_hla = StowToolHLA(*hla_args)
        self._transfer_tool_hla = TransferToolHLA(*hla_args)

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

    def _publish_feedback(self, goal_handle, action_type, status_msg: str):
        """Helper to publish a feedback message."""
        feedback = action_type.Feedback()
        feedback.status = status_msg
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"Feedback: {status_msg}")

    def _on_grab_cup(self, goal_handle):
        """Pick up the cup from the table using PickToolHLA."""
        self.get_logger().info("[GrabCupFromTable] Received goal")

        try:
            self._publish_feedback(goal_handle, GrabCupFromTable, "Picking up cup from table")
            self._pick_tool_hla.drink_location = "table"
            self._pick_tool_hla.pick_drink("medium")

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
        """Transfer the cup to the user's mouth using TransferToolHLA."""
        self.get_logger().info("[BringCupToMouth] Received goal")

        try:
            self._publish_feedback(goal_handle, BringCupToMouth, "Transferring cup to mouth")
            self._transfer_tool_hla.transfer_drink(
                "medium",                # speed
                "voice",                 # ready_to_initiate_mode
                "open_mouth",            # initiate_transfer_mode
                "voice",                 # ready_to_transfer_mode
                "sense",                 # transfer_complete_mode
                0.1,                     # outside_mouth_distance
                0,                       # ask_confirmation (no web interface)
                10.0,                    # drink_autocontinue_time
            )

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
        """Return the cup to the home position using base HLA movement."""
        self.get_logger().info("[HomeCup] Received goal")

        try:
            self._publish_feedback(goal_handle, HomeCup, "Moving to home position")
            self._pick_tool_hla.move_to_joint_positions(
                self._sim.scene_description.home_pos
            )

            goal_handle.succeed()
            result = HomeCup.Result()
            result.success = True
            result.message = "Cup returned to home position"
        except Exception as e:
            self.get_logger().error(f"[HomeCup] Failed: {traceback.format_exc()}")
            goal_handle.abort()
            result = HomeCup.Result()
            result.success = False
            result.message = str(e)

        self.get_logger().info(f"[HomeCup] Completed: {result.message}")
        return result

    def _on_pickup_and_order(self, goal_handle):
        """Pick up the cup from the wheelchair holder using PickToolHLA."""
        self.get_logger().info("[PickupAndOrder] Received goal")

        try:
            self._publish_feedback(goal_handle, PickupAndOrder, "Picking up cup from wheelchair holder")
            self._pick_tool_hla.drink_location = "wheelchair_handle"
            self._pick_tool_hla.pick_drink("medium")

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
        """Place the cup back to the wheelchair holder using StowToolHLA."""
        self.get_logger().info("[PutCupBackToHolder] Received goal")

        try:
            self._publish_feedback(goal_handle, PutCupBackToHolder, "Placing cup back to wheelchair holder")
            self._stow_tool_hla.drink_location = "wheelchair_handle"
            self._stow_tool_hla.stow_drink("medium")

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
