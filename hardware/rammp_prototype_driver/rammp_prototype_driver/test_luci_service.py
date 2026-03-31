import rclpy

# custom msgs/srvs
from rclpy.node import Node
from std_srvs.srv import SetBool


class TestLuciService(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("test_luci_service")  # MODIFY NAME
        self.luci_req_client = self.create_client(SetBool, "/base/drive_enable")
        self.send_req()

    def send_req(self):
        self.get_logger().info("I am in send_req function")
        req = SetBool.Request()
        req.data = True
        self.luci_req_client.wait_for_service()
        self.luci_req_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = TestLuciService()  # MODIFY NAME
    rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
