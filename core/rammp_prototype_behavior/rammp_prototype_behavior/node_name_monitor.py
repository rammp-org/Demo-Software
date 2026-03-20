import json
from pathlib import Path

json_path = Path(
    "/Demo-Software/core/rammp_prototype_behavior/rammp_prototype_behavior/nodes.json"
)


class NodeNameMonitor:
    def __init__(self, ros_node, callback):
        self.ros_node = ros_node
        self.callback = callback

    def NodesReady(self):
        actual_nodes = [
            name
            for name in self.get_fully_qualified_node_names()
            if not name.startswith(
                "_"
            )  # to remove built-in ros tools that show up by default when getting a nodes list
        ]

        with open(json_path) as f:
            data = json.load(f)

        expected_nodes = [
            n["name"] for n in data["nodes"]
        ]  # convert json dict format into list to compare with the actual nodes

        actual_nodes = set(
            actual_nodes
        )  # make actual_nodes set instead of list to get O(n) time

        missing = [node for node in expected_nodes if node not in actual_nodes]

        if missing:
            self.callback(False)
