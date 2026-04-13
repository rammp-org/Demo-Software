import rclpy
from rclpy.node import Node
from cmu_door_opener_interfaces.msg import ButtonInfo
from sensor_msgs.msg import Image


class ButtonInfoPublisher(Node):
    def __init__(self):
        super().__init__("button_info_publisher")

        self.declare_parameter("format", "mono8")  # Default test mono8
        self.test_format = (
            self.get_parameter("format").get_parameter_value().string_value
        )
        self.publisher_ = self.create_publisher(ButtonInfo, "/arm/door/button_info", 10)
        self.timer = self.create_timer(1.0, self.publish_button_info)

    def publish_button_info(self):
        msg = ButtonInfo()
        msg.id = 1

        mask = Image()
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.header.frame_id = "camera_frame"
        mask.height = 480
        mask.width = 640

        if self.test_format == "rgb8":
            mask.encoding = "rgb8"
            mask.step = 640 * 3
        else:
            if self.test_format != "mono8":
                self.get_logger().warn(
                    f"Unsupported format '{self.test_format}', falling back to mono8"
                )
            mask.encoding = "mono8"
            mask.step = 640

        msg.bounding_box = [10, 150, 200, 250]  # [x_min, y_min, x_max, y_max]
        # Create a dummy segmentation mask that corresponds to the bounding box (for demonstration purposes)
        if mask.encoding == "rgb8":
            mask.data = [0] * (640 * 480 * 3)
            for y in range(150, 250):
                for x in range(10, 200):
                    base = (y * 640 + x) * 3
                    mask.data[base] = 255
                    mask.data[base + 1] = 255
                    mask.data[base + 2] = 255
        else:
            mask.data = [0] * (640 * 480)
            for y in range(150, 250):
                for x in range(10, 200):
                    mask.data[y * 640 + x] = 255  # Mark the button area in the mask
        msg.segmentation_mask = mask

        msg.confidence = 0.92
        msg.pose_xyzrpy = [0.5, 0.1, 1.2, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
        msg.is_pressable = False

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published ButtonInfo id={msg.id}, confidence={msg.confidence}, format={mask.encoding}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ButtonInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
