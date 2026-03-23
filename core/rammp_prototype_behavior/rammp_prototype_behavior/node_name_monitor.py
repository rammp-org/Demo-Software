import json
from rclpy.callback_groups import ReentrantCallbackGroup
# json_path = Path(
#     "/Demo-Software/core/rammp_prototype_behavior/rammp_prototype_behavior/nodes.json"
# )


class NodeNameMonitor:
    def __init__(self, ros_node, json_path, callback):
        self.ros_node = ros_node
        self.callback = callback
        self.json_path = json_path
        cb_group = ReentrantCallbackGroup()
        self.nodes_ready_timer = ros_node.create_timer(
            1.0, self.NodesReady, callback_group=cb_group
        )
        self.nodes_was_missing = True  # to track the previous state of node readiness and only call the callback when there is a change in state

    def NodesReady(self):
        names_and_ns = self.ros_node.get_node_names_and_namespaces()

        actual_nodes = [
            f"{ns}/{name}" if ns != "/" else f"/{name}"
            for name, ns in names_and_ns
            if not name.startswith("_")
        ]

        with open(self.json_path) as f:
            data = json.load(f)

        expected_nodes = [
            n["name"] for n in data["nodes"]
        ]  # convert json dict format into list to compare with the actual nodes

        actual_nodes = set(
            actual_nodes
        )  # make actual_nodes set instead of list to get O(n) time

        missing = [node for node in expected_nodes if node not in actual_nodes]

        if missing and not self.nodes_was_missing:
            self.callback(False)
        elif not missing and self.nodes_was_missing:
            self.callback(True)
        self.nodes_was_missing = missing
