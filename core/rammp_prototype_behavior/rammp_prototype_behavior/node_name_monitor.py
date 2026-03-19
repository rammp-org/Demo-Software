#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import json


pkg_path = get_package_share_directory("rammp_pkg")
json_path = os.path.join(pkg_path, "config", "nodes.json")


class FindNodes(Node):  # MODIFY NAME
    def __init__(self, callback):
        super().__init__("find_nodes")  # MODIFY NAME

        self.find_nodes_timer = self.create_timer(1.0, self.find_nodes_callback)

    def find_nodes_callback(self):
        actual_nodes = [
            (name, ns)
            for name, ns in self.get_node_names_and_namespaces()
            if not name.startswith("_")
        ]

        # for name, ns in actual_nodes:
        #     self.get_logger().info(f"{name}, {ns}")

        with open(json_path) as f:
            data = json.load(f)

        expected_nodes = [(n["name"], n["namespace"]) for n in data["nodes"]]

        for node in expected_nodes:
            if node not in actual_nodes:
                self.error()

        # if all expected node exist:
        #  callback(True)

        self.get_logger().info("SUCCESS")

    def error(self):
        self.get_logger().info("ERROR")

    def is_ready() -> bool:
        pass
        # return self.is_ready


def main(args=None):
    def notice(ready: bool):
        print("resutl: " + ready)

    rclpy.init(args=args)
    node = FindNodes(callback=notice)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
