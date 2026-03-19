from ament_index_python.packages import get_package_share_directory
import os
import json


pkg_path = get_package_share_directory("rammp_pkg")  # needs to be changed
json_path = os.path.join(pkg_path, "config", "nodes.json")  # needs to be changed


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
        else:
            self.callback(True)

        return
