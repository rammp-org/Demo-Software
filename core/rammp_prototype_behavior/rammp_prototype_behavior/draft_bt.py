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


class estop_check(py_trees.behaviour.Behaviour):
    def __init__(self, name="estop check"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("estop_status", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.estop_status:
            # some function here to request mebot and arm stop all movement
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


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


class navigation(py_trees.behaviour.Behaviour):
    def __init__(self, name="navigation"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("user_input", access=py_trees.common.Access.READ)

    def update(self):
        if (
            self.blackboard.user_input == "CA" or self.blackboard.user_input == "CD"
        ):  # curb ascending or descending
            # maybe split into sequence

            # send service request: mebot/drive/enable = true
            # send action request: /nav/curb/navigate
            # send service request: mebot/drive/enable = false after navigation completes
            # send action request: mebot/curb/traverse
            # go back to default/idle state?
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class arm_movement(py_trees.behaviour.Behaviour):
    def __init__(self, name="arm movement"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("user_input", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.user_input == "arm enabled":
            # could be broken into multiple behaviors for a sequence

            # send service request: mebot/drive/enable= false
            # send service request: /set_mode
            # select arm mode to send action request?
            # send action request: /arm/retract
            # go back to default/idle state?

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
        name="chair control selector", memory=False
    )
    chair_control_selector.add_children([self_level_on(), self_level_off()])

    chair_control_sequence = py_trees.composites.Sequence(
        name="chair control sequence", memory=True
    )
    chair_control_sequence.add_children([chair_control(), chair_control_selector])

    navigation_sequence = py_trees.composites.Sequence(
        name="navigation sequence", memory=True
    )
    navigation_sequence.add_children([navigation()])

    arm_movement_sequence = py_trees.composites.Sequence(
        name="arm movement sequence", memory=True
    )
    arm_movement_sequence.add_children([arm_movement()])

    check_state = py_trees.composites.Selector(name="check state selector", memory=True)
    check_state.add_children(
        [chair_control_sequence, navigation_sequence, arm_movement_sequence]
    )

    root = py_trees.composites.Sequence(name="root sequence", memory=False)
    root.add_children([estop_check(), check_state])

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
