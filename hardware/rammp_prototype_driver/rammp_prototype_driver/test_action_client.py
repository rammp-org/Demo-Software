import rclpy
from rammp_prototype_interfaces.action import CurbTraverse

# custom msgs/srvs
from rclpy.action import ActionClient
from rclpy.node import Node


class TestActionClient(Node):
    def __init__(self):
        super().__init__("test_action_client")
        self.dummy_client = ActionClient(self, CurbTraverse, "curb_traverse")

    def send_goal(self, direction):
        goal = CurbTraverse.Goal()
        goal.direction = direction

        self.dummy_client.wait_for_server()
        self.send_goal_future = self.dummy_client.send_goal_async(
            goal, feedback_callback=self.feedback_callabck
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal = future.result()
        if not goal.accepted():
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")

        self.get_result_future = goal.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(str(result))
        rclpy.shutdown()

    def feedback_callabck(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Feedback: {0}".format(feedback))
        pass


def main(args=None):
    rclpy.init(args=args)
    action_client = TestActionClient()
    # future = action_client.send_goal(1)
    action_client.send_goal(1)
    # rclpy.spin_until_future_complete(action_client, future)
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
