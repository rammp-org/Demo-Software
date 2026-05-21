#!/usr/bin/env python3
"""
Manual test script: watch /arm/door/button_info and fire /arm/door/open
once a button has been in frame continuously for 5 seconds.

Pressability check is intentionally skipped — is_pressable is not yet
implemented in the detector. Trigger condition is confidence > 0 held
for REQUIRED_STABLE_S seconds.

Usage:
    python3 test_button_press_trigger.py

Prerequisites:
    - button_detector running and detection enabled
    - button_push_controller running
    - arm in OPEN_DOOR mode  (ros2 service call /arm/set_mode ...)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from cmu_door_opener_interfaces.msg import ButtonInfo
from cmu_door_opener_interfaces.action import DoorOpen

REQUIRED_STABLE_S = 5.0  # continuous in-frame detections needed before firing


class ButtonPressTrigger(Node):
    def __init__(self):
        super().__init__("button_press_trigger_test")

        self._stable_since: float | None = None  # wall time when streak started
        self._fired = False

        self.create_subscription(
            ButtonInfo, "/arm/door/button_info", self._cb_button_info, 10
        )
        self._action_client = ActionClient(self, DoorOpen, "/arm/door/open")

        self.get_logger().info(
            f"Waiting for button in frame for {REQUIRED_STABLE_S}s ..."
        )

    def _cb_button_info(self, msg: ButtonInfo):
        if self._fired:
            return

        in_frame = msg.confidence > 0.0

        if in_frame:
            if self._stable_since is None:
                self._stable_since = time.time()
                self.get_logger().info(
                    f"Button in frame — starting stability timer (conf={msg.confidence:.2f})"
                )

            elapsed = time.time() - self._stable_since
            self.get_logger().info(
                f"  stable for {elapsed:.1f}s / {REQUIRED_STABLE_S}s  "
                f"(conf={msg.confidence:.2f}  "
                f"xyz=[{msg.pose_xyzrpy[0]:.3f}, {msg.pose_xyzrpy[1]:.3f}, {msg.pose_xyzrpy[2]:.3f}])"
            )

            if elapsed >= REQUIRED_STABLE_S:
                self._fire_action()
        else:
            if self._stable_since is not None:
                self.get_logger().warn("Button left frame — resetting stability timer")
            self._stable_since = None

    def _fire_action(self):
        self._fired = True
        self.get_logger().info(
            f"Button in frame for {REQUIRED_STABLE_S}s — sending /arm/door/open goal"
        )

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/arm/door/open action server not available")
            return

        goal = DoorOpen.Goal()
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._cb_goal_accepted)

    def _cb_goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return
        self.get_logger().info("Goal accepted — waiting for result ...")
        handle.get_result_async().add_done_callback(self._cb_result)

    def _cb_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"SUCCESS: {result.message}")
        else:
            self.get_logger().error(f"FAILED: {result.message}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ButtonPressTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
