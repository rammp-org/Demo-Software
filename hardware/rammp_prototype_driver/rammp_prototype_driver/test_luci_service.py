import rclpy

# custom msgs/srvs
from rclpy.node import Node
from std_srvs.srv import SetBool


class TestLuciService(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("test_luci_service")  # MODIFY NAME
        self.luci_req_client = self.create_client(SetBool, "drive_enable")

    def send_req(self):
        req = SetBool.request()
        req.data = True
        self.set_auto_remote_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = TestLuciService()  # MODIFY NAME
    rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
