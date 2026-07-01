#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rosbag2_py
from rclpy.serialization import serialize_message
from rammp_prototype_interfaces.msg import RAMMPPrototypeState
from gui_interfaces.msg import SystemState

NAV_ASCEND_DETECTING = "Nav_ascendDetecting"
NAV_DESCEND_DETECTING = "Nav_descendDetecting"
NAV_ASCENDING = "Nav_ascending"
NAV_DESCENDING = "Nav_descending"
NAV_SL_ON = "Nav_SLOn"
NAV_CANCELING = "Nav_canceling"

BAG_START_STATES = {
    NAV_ASCEND_DETECTING: "ascend_bag",
    NAV_DESCEND_DETECTING: "descend_bag",
    NAV_SL_ON: "sl_on_bag",
}

RECORDING_WRITE_STATES = frozenset(
    {
        NAV_ASCENDING,
        NAV_DESCENDING,
        NAV_SL_ON,
        NAV_CANCELING,
    }
)

ACTIVE_RECORDING_STATES = frozenset(BAG_START_STATES) | RECORDING_WRITE_STATES


class RosBagNode(Node):
    def __init__(self):
        super().__init__("ros_bag_node")

        self.rammp_prototype_state_subscription = self.create_subscription(
            RAMMPPrototypeState,
            "rammp_prototype_state",
            self.rammp_prototype_state_callback,
            10,
        )

        self.writer = None
        self.bag_id = 0
        self.current_state = None

        self.system_state_subscription = self.create_subscription(
            SystemState,
            "/system/state",
            self.system_state_callback,
            10,
        )

    def start_recording(self, bag_prefix: str):
        if self.writer is not None:
            return

        self.bag_id += 1
        bag_uri = f"{bag_prefix}_{self.bag_id}"

        try:
            self.writer = rosbag2_py.SequentialWriter()
            self.writer.open(
                rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""),
            )

            topic_info = rosbag2_py.TopicMetadata(
                name="rammp_prototype_state",
                type="rammp_prototype_interfaces/msg/RAMMPPrototypeState",
                serialization_format="cdr",
            )
            self.writer.create_topic(topic_info)
        except Exception as e:
            self.get_logger().error(f"Error starting recording: {e}")
            return
        self.get_logger().info(f"Started recording bag: {bag_uri}")

    def stop_recording(self):
        if self.writer is None:
            return

        del self.writer
        self.writer = None
        self.get_logger().info("Stopped recording bag")

    def system_state_callback(self, msg: SystemState):
        previous_state = self.current_state
        new_state = msg.state
        self.current_state = new_state

        if new_state == previous_state:
            return

        if new_state in BAG_START_STATES:
            self.stop_recording()
            self.start_recording(BAG_START_STATES[new_state])
        elif (
            previous_state in ACTIVE_RECORDING_STATES
            and new_state not in ACTIVE_RECORDING_STATES
        ):
            self.stop_recording()

    def rammp_prototype_state_callback(self, msg: RAMMPPrototypeState):
        if self.writer is None:
            return
        if self.current_state not in RECORDING_WRITE_STATES:
            return

        self.writer.write(
            "rammp_prototype_state",
            serialize_message(msg),
            self.get_clock().now().nanoseconds,
        )


def main(args=None):
    rclpy.init(args=args)
    node = RosBagNode()
    rclpy.spin(node)
    node.stop_recording()  # add this — ensures del writer is called cleanly
    rclpy.shutdown()


if __name__ == "__main__":
    main()
