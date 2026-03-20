import rclpy
from rclpy.node import Node
from pathlib import Path
from node_name_monitor import NodeNameMonitor

json_path = Path(
    "/Demo-Software/core/rammp_prototype_behavior/rammp_prototype_behavior/nodes.json"
)


class testNNMonitor(Node):
    def __init__(self):
        super().__init__("test_nn_monitor")
        self.nodes_check = NodeNameMonitor(self, json_path, self.ready())
        self.node_check_timer = self.create_timer(1.0, self.nodes_check)

    def ready(self, isReady):
        if not isReady:
            self.get_logger().info("ERROR: Node missing!")


def main(args=None):
    rclpy.init(args=args)
    node = testNNMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
