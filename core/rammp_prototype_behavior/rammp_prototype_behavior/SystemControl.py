import rclpy
import rclpy.action
import rclpy.node
from .ArmPresetActionClient import ArmPresetActionClient


class SystemControl(rclpy.node.Node):
    def __init__(self):
        super().__init__("system_control")
        self.get_logger().info("System Control Node has been started.")

    def init_subscribers(self):
        pass

    def init_services_clients(self):
        pass

    def init_actions_clients(self):
        self.arm_preset_client = ArmPresetActionClient(self)


def main():
    rclpy.init()
    node = SystemControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
