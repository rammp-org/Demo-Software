"""Dummy drinking node that exposes action servers for cup manipulation.

This node provides action servers matching the Drinking Node spec:
  - /arm/drink/grab_cup_from_table
  - /arm/drink/bring_cup_to_mouth
  - /arm/drink/home_cup
  - /arm/drink/put_cup_back_to_holder
  - /arm/drink/pickup_and_order

Each action server is a dummy that sleeps briefly and returns success.
"""

import time

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


DUMMY_STEP_DURATION = 1.0  # seconds per dummy step


class DrinkingNode(rclpy.node.Node):
    """ROS 2 node that exposes dummy action servers for the feeding demo."""

    def __init__(self):
        super().__init__("drinking_node")
        self.get_logger().info("Drinking Node starting up...")

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
        self.get_logger().info("[GrabCupFromTable] Received goal")

        self._publish_feedback(goal_handle, GrabCupFromTable, "Moving to table")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, GrabCupFromTable, "Closing gripper on cup")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, GrabCupFromTable, "Lifting cup")
        time.sleep(DUMMY_STEP_DURATION)

        goal_handle.succeed()
        result = GrabCupFromTable.Result()
        result.success = True
        result.message = "Cup grabbed from table"
        self.get_logger().info(f"[GrabCupFromTable] Completed: {result.message}")
        return result

    def _on_bring_cup_to_mouth(self, goal_handle):
        self.get_logger().info("[BringCupToMouth] Received goal")

        self._publish_feedback(goal_handle, BringCupToMouth, "Moving cup toward mouth")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, BringCupToMouth, "Holding at mouth")
        time.sleep(DUMMY_STEP_DURATION)

        goal_handle.succeed()
        result = BringCupToMouth.Result()
        result.success = True
        result.message = "Cup brought to mouth"
        self.get_logger().info(f"[BringCupToMouth] Completed: {result.message}")
        return result

    def _on_home_cup(self, goal_handle):
        self.get_logger().info("[HomeCup] Received goal")

        self._publish_feedback(goal_handle, HomeCup, "Retracting cup from mouth")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, HomeCup, "Moving to home position")
        time.sleep(DUMMY_STEP_DURATION)

        goal_handle.succeed()
        result = HomeCup.Result()
        result.success = True
        result.message = "Cup returned to home position"
        self.get_logger().info(f"[HomeCup] Completed: {result.message}")
        return result

    def _on_pickup_and_order(self, goal_handle):
        self.get_logger().info("[PickupAndOrder] Received goal")

        self._publish_feedback(goal_handle, PickupAndOrder, "Picking up cup")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, PickupAndOrder, "Ordering drink")
        time.sleep(DUMMY_STEP_DURATION)

        goal_handle.succeed()
        result = PickupAndOrder.Result()
        result.success = True
        result.message = "Pickup and order completed"
        self.get_logger().info(f"[PickupAndOrder] Completed: {result.message}")
        return result

    def _on_put_cup_back(self, goal_handle):
        self.get_logger().info("[PutCupBackToHolder] Received goal")

        self._publish_feedback(goal_handle, PutCupBackToHolder, "Moving to wheelchair holder")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, PutCupBackToHolder, "Placing cup down")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, PutCupBackToHolder, "Opening gripper")
        time.sleep(DUMMY_STEP_DURATION)

        goal_handle.succeed()
        result = PutCupBackToHolder.Result()
        result.success = True
        result.message = "Cup placed back at wheelchair holder"
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
