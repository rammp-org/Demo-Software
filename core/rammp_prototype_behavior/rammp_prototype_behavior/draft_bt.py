import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool


class subscribers(Node):
    def __init__(self):
        super().__init__("subscribers")

        self.blackboard = py_trees.blackboard.Client(name="subscribers")
        self.blackboard.register_key("user_input", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("arm_status", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "estop_status", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "doorVisible_status", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "userConnection_status", access=py_trees.common.Access.WRITE
        )

        self.user_input_sub = self.create_subscription(
            String, "user_input", self.user_input_callback, 10
        )
        self.arm_status_sub = self.create_subscription(
            String, "arm_status", self.arm_status_callback, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, "estop", self.estop_status_callback, 10
        )
        self.doorVis_sub = self.create_subscription(
            Bool, "doorVisible", self.doorVisible_status_callback, 10
        )
        self.user_connection_sub = self.create_subscription(
            Bool, "user_connection", self.user_connection_callback, 10
        )

    def user_input_callback(self, msg):
        self.blackboard.user_input = msg.data

    def arm_status_callback(self, msg):
        self.blackboard.arm_status = msg.data

    def estop_status_callback(self, msg):
        self.blackboard.estop_status = msg.data

    def doorVisible_status_callback(self, msg):
        self.blackboard.doorVisible_status = msg.data

    def user_connection_callback(self, msg):
        self.blackboard.userConnection_status = msg.data


class chair_control(py_trees.behaviour.Behaviour):
    def __init__(self, name="chair control"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("user_input", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.user_input in ("self level on", "self level off"):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class self_level_on(py_trees.behaviour.Behaviour):  # service client
    def __init__(self, name="self level on"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("user_input", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.user_input == "self level on":
            # send service request here
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class self_level_off(py_trees.behaviour.Behaviour):  # service client
    def __init__(self, name="self level off"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("user_input", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.user_input == "self level off":
            # send service request here
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


def create_tree():
    client = py_trees.blackboard.Client(name="init")
    client.register_key("user_input", access=py_trees.common.Access.WRITE)
    client.register_key("arm_status", access=py_trees.common.Access.WRITE)
    client.register_key("estop_status", access=py_trees.common.Access.WRITE)
    client.register_key("doorVisible_status", access=py_trees.common.Access.WRITE)
    client.register_key("userConnection_status", access=py_trees.common.Access.WRITE)
    client.user_input = ""
    client.arm_status = ""
    client.estop_status = False
    client.doorVisible_status = False
    client.userConnection_status = False

    chair_control_selector = py_trees.composites.Selector(
        name="chair control selector", memory=True
    )
    chair_control_selector.add_children([self_level_on(), self_level_off()])

    root = py_trees.composites.Selector(name="root selector", memory=True)
    root.add_children(
        [
            chair_control(),  # check user input first
            chair_control_selector,  # then execute the right action
        ]
    )

    return py_trees.trees.BehaviourTree(root)


def main():
    rclpy.init()
    tree = create_tree()
    subscribers_node = subscribers()

    # Print tree structure once at start
    print(py_trees.display.unicode_tree(tree.root))

    try:
        while rclpy.ok():
            rclpy.spin_once(
                subscribers_node, timeout_sec=0.1
            )  # process incoming ROS2 messages

            # Print raw blackboard values before ticking
            print(py_trees.display.unicode_blackboard())

            tree.tick()  # tick the tree

            # Print tree status every tick
            print(py_trees.display.unicode_tree(tree.root, show_status=True))

            # Print feedback from each node
            for node in tree.root.iterate():
                if node.feedback_message:
                    print(f"  [{node.name}] {node.feedback_message}")
    finally:
        tree.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
