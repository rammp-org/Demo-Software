import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32
import time
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.get_logger().info("Test Node has been started.")
        self._reentrant_cb_group1 = ReentrantCallbackGroup()
        self._reentrant_cb_group2 = ReentrantCallbackGroup()

        self._mutually_exclusive_cb_group_pub = MutuallyExclusiveCallbackGroup()
        self._mutually_exclusive_cb_group_sub = MutuallyExclusiveCallbackGroup()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub1 = self.create_publisher(Int32, "topic1", 10)
        self.pub2 = self.create_publisher(Int32, "topic2", 0)
        self.pub1_counter = 0
        self.pub2_counter = 0

        # self.timer1 = self.create_timer(
        #     0.1, self.timer1_callback, callback_group=self._reentrant_cb_group1
        # )
        # self.timer2 = self.create_timer(
        #     0.1,
        #     self.timer2_callback,
        #     callback_group=self._mutually_exclusive_cb_group_pub,
        # )
        # self.timer3 = self.create_timer(
        #     0.1, self.timer1_callback, callback_group=self._reentrant_cb_group2
        # )

        self.sub1 = self.create_subscription(
            Int32,
            "topic1",
            self.sub1_callback,
            qos_profile,
            callback_group=self._reentrant_cb_group1,
        )

        self.sub2 = self.create_subscription(
            Int32,
            "topic2",
            self.sub2_callback,
            qos_profile,
            callback_group=self._mutually_exclusive_cb_group_sub,
        )

        self.sub3 = self.create_subscription(
            Int32,
            "topic1",
            self.sub1_callback,
            qos_profile,
            callback_group=self._reentrant_cb_group2,
        )

    def sub1_callback(self, msg):
        self.get_logger().info(f"processing message {msg.data} from topic1...")
        # add one second delay to block the callback group and see if the other callback group can still process the message
        time.sleep(5)
        self.get_logger().info(f"finished processing message {msg.data} from topic1...")

    def sub2_callback(self, msg):
        self.get_logger().info(f"processing message {msg.data} from topic2...")
        time.sleep(3)
        self.get_logger().info(f"finished processing message {msg.data} from topic2...")

    def timer1_callback(self):
        msg = Int32()
        msg.data = self.pub1_counter
        self.pub1.publish(msg)
        self.get_logger().info(
            f"Published message from topic1, id: {self.pub1_counter}"
        )
        self.pub1_counter += 1

    def timer2_callback(self):
        msg = Int32()
        msg.data = self.pub2_counter
        self.pub2.publish(msg)
        self.get_logger().info(
            f"Published message from topic2, id: {self.pub2_counter}"
        )
        self.pub2_counter += 1


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(test_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()
