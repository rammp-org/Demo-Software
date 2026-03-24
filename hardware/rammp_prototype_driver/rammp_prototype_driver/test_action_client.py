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
        return self.dummy_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    action_client = TestActionClient()
    future = action_client.send_goal(1)
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == "__main__":
    main()
