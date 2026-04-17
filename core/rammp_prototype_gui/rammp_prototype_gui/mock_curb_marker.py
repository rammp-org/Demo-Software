import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from neu_navigation_interfaces.msg import CurbInfo
import numpy as np


class CurbMarkerPublisher(Node):
    def __init__(self):
        super().__init__("curb_marker_publisher")

        self.publisher_marker = self.create_publisher(
            Marker, "/perception/curb_marker", 10
        )
        self.publisher_curbInfo = self.create_publisher(CurbInfo, "/nav/curb/info", 10)
        self.timer = self.create_timer(1.0, self.publish_curb_marker_and_info)

    def publish_curb_marker_and_info(self):
        distance = 1.5

        height = 0.4
        orientation = 15.0
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "camera_frame"

        marker.ns = "curb_plane"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(distance)  # Example x position
        marker.pose.position.y = float(2.0)  # Example y position
        marker.pose.position.z = float(0.0)  # Example z position

        angle = (
            orientation / 180.0 * 3.14159
        )  # Example rotation around Y-axis (15 degrees)
        marker.pose.orientation.z = float(np.sin(angle / 2.0))
        marker.pose.orientation.w = float(np.cos(angle / 2.0))

        marker.scale.x = 2.0
        marker.scale.y = 0.05
        marker.scale.z = float(height)  # Example height

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 0.6

        self.publisher_marker.publish(marker)
        self.get_logger().info(
            f"Published Marker id={marker.id} to /perception/curb_marker"
        )

        curb_info = CurbInfo()
        curb_info.success = True
        curb_info.distance = distance
        curb_info.orientation = orientation
        curb_info.height = height
        self.publisher_curbInfo.publish(curb_info)
        self.get_logger().info("Published CurbInfo to /nav/curb/info")


def main(args=None):
    rclpy.init(args=args)
    node = CurbMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
