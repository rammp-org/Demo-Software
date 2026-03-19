#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import os
import json


pkg_path = get_package_share_directory("rammp_pkg")  # needs to be changed
json_path = os.path.join(pkg_path, "config", "nodes.json")  # needs to be changed


class NodeNameMonitor:
    def __init__(self, ros_node, callback):
        self.ros_node = ros_node
        self.callback = callback
        self.ready = True
        self.find_nodes_timer = self.ros_node.create_timer(
            1.0, self.find_nodes_callback
        )

    def monitorNodes(self):
        actual_nodes = [
            name
            for name in self.get_fully_qualified_node_names()
            if not name.startswith("_")
        ]

        with open(json_path) as f:
            data = json.load(f)

        expected_nodes = [n["name"] for n in data["nodes"]]

        actual_nodes = set(
            actual_nodes
        )  # make actual_nodes set instead of list to get O(n)
        missing = [node for node in expected_nodes if node not in actual_nodes]

        if missing:
            self.callback(False)
        else:
            self.callback(True)

        return
