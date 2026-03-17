import py_trees
from rclpy.node import Node
from std_msgs.msg import String


class subscribers(Node):
    def __init__(self):
        super().__init__("subscribers")
        self.user_input_sub = self.create_subscription(
            String, "user_input", self.user_input_callback, 10
        )
        self.arm_status_sub = self.create_subscription(
            String, "arm status", self.arm_status_callback, 10
        )
        self.estop_sub = self.create_subscription(
            bool, "estop", self.estop_status_callback, 10
        )
        self.doorVis_sub = self.create_subscription(
            bool, "doorVis", self.doorVis_status_callback, 10
        )
        self.user_connection_sub = self.create_subscription(
            bool, "user_connection", self.user_connection_callback, 10
        )

    def user_input_callback(self, msg):
        self.blackboard.user_input = msg.data

    def arm_status_callback(self, msg):
        self.blackboard.arm_status = msg.data

    def estop_status_callback(self, msg):
        self.blackboard.estop_status = msg.data

    def doorVis_status_callback(self, msg):
        self.blackboard.doorVis_status = msg.data

    def user_connection_callback(self, msg):
        self.blackboard.userConnection_status = msg.data


class chair_control(py_trees.behaviours.Behaviour):
    def __init__(self, name="chair control"):
        super().__init__(name)


def create_tree():
    client = py_trees.blackboard.Client(name="init")
    client.register_key("arm_status", access=py_trees.common.Access.READ)
