#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rosbag2_py
from rclpy.serialization import serialize_message
from rammp_prototype_interfaces.msg import RAMMPPrototypeState


class rosBagNode(Node):
    def __init__(self):
        super().__init__("ros_bag_node")

        self.rosbag_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri="rammp_prototype_state_bag", storage_id="sqlite3"
        )
        converter_options = rosbag2_py.ConverterOptions("", "")
        self.rosbag_writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            name="rammp_prototype_state",
            type="rammp_prototype_interfaces/msg/RAMMPPrototypeState",
            serialization_format="cdr",
        )
        self.rosbag_writer.create_topic(topic_info)

        # init rammp_prototype_state subscriber
        self.rammp_prototype_state_subscription = self.create_subscription(
            RAMMPPrototypeState,
            "rammp_prototype_state",
            self.rammp_prototype_state_callback,
            10,
        )

    def rammp_prototype_state_callback(self, msg: RAMMPPrototypeState):
        self.rosbag_writer.write(
            "rammp_prototype_state",
            serialize_message(msg),
            self.get_clock().now().nanoseconds,
        )


def main(args=None):
    rclpy.init(args=args)
    node = rosBagNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
