import time

import rclpy
import rclpy.action
import rclpy.node

from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cornell_feeding_interfaces.action import DrinkAction
from std_srvs.srv import SetBool


class DrinkingNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("drink_action_server")
        self.get_logger().info("Mock Drinking Node has been started.")

        self._action_group = ReentrantCallbackGroup()
        self.create_service(
            SetBool,
            "/arm/drink/detection/enable",
            self._srv_detection_enable,
            callback_group=self._action_group,
        )
        # Action Server for PickUpAndOrder
        self._pickup_and_order_action_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/pickup_and_order",
            self.execute_action_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )
        self._grab_cup_from_table_action_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/grab_cup_from_table",
            self.execute_action_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )
        self._bring_cup_to_mouth_action_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/bring_cup_to_mouth",
            self.execute_action_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )
        self._home_cup_action_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/home_cup",
            self.execute_action_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )
        self._put_cup_back_to_holder_action_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/put_cup_back_to_holder",
            self.execute_action_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )

        self._action_running = False
        self._action_counter = 0
        self._mock_action_result = True  # default to True
        self._mock_action_reject = False  # default to accept action request

    def _srv_detection_enable(self, request, response):
        if request.data:
            self.get_logger().info("Drink detection enabled.")
        else:
            self.get_logger().info("Drink detection disabled.")
        response.success = True
        return response

    def mock_action_result(self, result: bool):
        self._mock_action_result = result

    def mock_action_reject(self, reject: bool):
        self._mock_action_reject = reject

    def publish_feedback(self, goal_handle) -> bool:
        feedback = DrinkAction.Feedback()
        while self._action_counter > 0:
            feedback.state = str(self._action_counter)
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            self._action_counter -= 1
            self.get_logger().info(f"action counter left: {self._action_counter}")
            time.sleep(0.1)  # sleep 0.1s
        return True

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Received an action cancel request.")
        return CancelResponse.ACCEPT

    def execute_action_callback(self, goal_handle):
        self.get_logger().info("Received a  action goal.")
        if self._action_running or self._mock_action_reject:
            self.get_logger().warn(
                "Another action is already running. Rejecting new goal."
            )
            goal_handle.reject()
            return DrinkAction.Result()
        self._action_running = True
        self._action_counter = 20  # 2s action time.
        if not self.publish_feedback(goal_handle):  # canceled during action process
            result = DrinkAction.Result()
            self._action_running = False
            return result

        # mock result
        result = DrinkAction.Result()
        self._action_running = False

        if self._mock_action_result:
            goal_handle.succeed()
            result.success = True
        else:
            goal_handle.abort()
            result.success = False
        return result


def main(args=None):
    """Entry point for the mock drinking node."""
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
