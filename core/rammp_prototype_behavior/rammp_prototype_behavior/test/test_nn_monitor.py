import rclpy
from rclpy.node import Node
from pathlib import Path
from rammp_prototype_behavior.node_name_monitor import NodeNameMonitor

json_path = Path(
    "/home/herl/rammp_ws/src/Demo-Software/core/rammp_prototype_behavior/rammp_prototype_behavior/test/nodes.json"
)


class testNNMonitor(Node):
    def __init__(self):
        super().__init__("test_nn_monitor")
        self.nodes_check = NodeNameMonitor(self, json_path, self.ready)

    def ready(self, isReady):
        if not isReady:
            self.get_logger().info("Node missing")
        else:
            self.get_logger().info("Node ready")


def main(args=None):
    rclpy.init(args=args)
    node = testNNMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
