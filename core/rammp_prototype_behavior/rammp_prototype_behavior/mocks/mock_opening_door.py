import time

import rclpy
import rclpy.action
import rclpy.node

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from cmu_door_opener_interfaces.action import DoorOpen


class ButtonDetector(rclpy.node.Node):
    def __init__(self):
        super().__init__("button_detector")
        self.get_logger().info("Mock Button Detector Node has been started.")
        self.cb_group = ReentrantCallbackGroup()

        self._srv_detection_enable = self.create_service(
            SetBool,
            "/arm/door/detection/enable",
            self._srv_detection_enable,
            callback_group=self.cb_group,
        )

    def _srv_detection_enable(self, request, response):
        if request.data:
            self.get_logger().info("Button detection enabled.")
        else:
            self.get_logger().info("Button detection disabled.")
        response.success = True
        return response


class OpenDoor(rclpy.node.Node):
    def __init__(self):
        super().__init__("button_push_controller")
        self.get_logger().info("Mock Opening Door Node has been started.")
        self._mock_action_reject = False  # default to accept all goals
        self._mock_action_result = True  # default to successful execution
        self._action_running = False
        self._action_counter = 0
        self.cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            DoorOpen,
            "/arm/door/open",
            self._execute_callback,
            callback_group=self.cb_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

    def publish_feedback(self, goal_handle) -> bool:
        feedback = DoorOpen.Feedback()
        while self._action_counter > 0:
            self.get_logger().info("action counter left: " + str(self._action_counter))
            feedback.status = str(self._action_counter)
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            self._action_counter -= 1
            time.sleep(0.1)  # sleep 0.1s
        return True

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Received an action cancel request.")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self.get_logger().info("Received a  action goal.")
        if self._action_running or self._mock_action_reject:
            self.get_logger().warn(
                "Another action is already running. Rejecting new goal."
            )
            goal_handle.reject()
            return DoorOpen.Result()
        self._action_running = True
        self._action_counter = 20  # 2s action time.
        if not self.publish_feedback(goal_handle):  # canceled during action process
            result = DoorOpen.Result()
            self._action_running = False
            return result
        self.get_logger().info("Action execution finished.")
        # mock result
        result = DoorOpen.Result()
        self._action_running = False

        if self._mock_action_result:
            goal_handle.succeed()
            result.success = True
        else:
            goal_handle.abort()
            result.success = False
        return result

    def _goal_callback(self, goal_request):
        # goal_callback receives the Goal request message, not a goal handle.
        # Accept/reject by returning GoalResponse.
        self.get_logger().info("Received an action goal request.")
        return GoalResponse.ACCEPT


def main(args=None):
    """Entry point for the mock opening door node."""
    rclpy.init(args=args)
    node = OpenDoor()
    button_detector = ButtonDetector()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(button_detector)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        button_detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
