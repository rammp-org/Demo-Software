import rclpy
from rclpy.node import Node
from gui_interfaces.srv import UserInputs
from pynput import keyboard


class KeypadNode(Node):
    def __init__(self):
        super().__init__("button_input_node")
        self.client = self.create_client(UserInputs, "/GuiBridge/user_input")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for GuiBridge service...")

        self.get_logger().info("Button input node ready")

        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        try:
            if key.char == "e":
                self.send_command("chair/curb/ascend")
            elif key.char == "r":
                self.send_command("chair/curb/descend")
        except AttributeError:
            pass

    def send_command(self, command):
        request = UserInputs.Request()
        request.input = command
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Response: success={f.result().success}, message={f.result().message}"
            )
        )
        self.get_logger().info(f"Sent: {command}")


def main():
    rclpy.init()
    node = KeypadNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
