"""Dummy drinking node that exposes action servers for cup manipulation.

This node provides action servers matching the Drinking Node spec:
  - /arm/drink/grab_cup_from_table
  - /arm/drink/bring_cup_to_mouth
  - /arm/drink/home_cup
  - /arm/drink/put_cup_back_to_holder

Each action server is a dummy that sleeps briefly and returns success.
"""

import time

import rclpy
import rclpy.node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from cornell_feeding_interfaces.action import (
    GrabCup,
    BringCupToMouth,
    HomeCup,
    PutCupBack,
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
            GrabCup,
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
            PutCupBack,
            "/arm/drink/put_cup_back_to_holder",
            self._on_put_cup_back,
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
        source = goal_handle.request.source
        self.get_logger().info(f"[GrabCup] Received goal: source={source}")

        self._publish_feedback(goal_handle, GrabCup, f"Moving to {source}")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, GrabCup, "Closing gripper on cup")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, GrabCup, "Lifting cup")
        time.sleep(DUMMY_STEP_DURATION)

        goal_handle.succeed()
        result = GrabCup.Result()
        result.success = True
        result.message = f"Cup grabbed from {source}"
        self.get_logger().info(f"[GrabCup] Completed: {result.message}")
        return result

    def _on_bring_cup_to_mouth(self, goal_handle):
        distance = goal_handle.request.outside_mouth_distance
        self.get_logger().info(
            f"[BringCupToMouth] Received goal: outside_mouth_distance={distance}"
        )

        self._publish_feedback(goal_handle, BringCupToMouth, "Moving cup toward mouth")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(
            goal_handle, BringCupToMouth, f"Holding at {distance}m from mouth"
        )
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

    def _on_put_cup_back(self, goal_handle):
        destination = goal_handle.request.destination
        self.get_logger().info(f"[PutCupBack] Received goal: destination={destination}")

        self._publish_feedback(goal_handle, PutCupBack, f"Moving to {destination}")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, PutCupBack, "Placing cup down")
        time.sleep(DUMMY_STEP_DURATION)

        self._publish_feedback(goal_handle, PutCupBack, "Opening gripper")
        time.sleep(DUMMY_STEP_DURATION)

        goal_handle.succeed()
        result = PutCupBack.Result()
        result.success = True
        result.message = f"Cup placed back at {destination}"
        self.get_logger().info(f"[PutCupBack] Completed: {result.message}")
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
