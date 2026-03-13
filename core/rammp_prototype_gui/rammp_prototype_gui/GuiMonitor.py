import posix_ipc
import mmap
import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from .unreal_remote_websocket import UnrealRemoteWebsocket


class GuiMonitor(Node):
    instance = None

    def __init__(self, shm_size=1024 * 1024):
        super().__init__("GuiMonitor")
        GuiMonitor.instance = self
        print("GuiMonitor node has been started.")

        self.ue = UnrealRemoteWebsocket(host="192.168.68.51", preset="RCPS")
        # ue = UnrealRemote(host="192.168.68.63",http_port=30010)
        # if not ue.ping():
        #     print("Connection failed!")
        #     sys.exit(1)
        # print("Connected!\n")
        # Create shared memory and map it
        self.shm = posix_ipc.SharedMemory(
            "/ros_ue_shm",
            posix_ipc.O_CREAT,
            size=shm_size,  # default size is 1MB
        )
        # Map it
        self.mapfile = mmap.mmap(self.shm.fd, shm_size)
        self.shm.close_fd()

        # make publisher for user connected or not, message should be boolen
        self.connection_publisher = self.create_publisher(Bool, "user_connection", 1)
        self.ethernet_gui_connected = False
        self.wifi_gui_connected = False
        self.connection_publisher_timer = self.create_timer(
            1.0, self.publish_connection_status
        )
        self.test_ue_counter = 0
        self.test_ue_timer = self.create_timer(1.0, self.test_ue)

        # make publisher for user input, message should be string
        self.input_publisher = self.create_publisher(String, "user_input", 10)

        # make publisher for manual set control, message should be string
        self.manual_control_publisher = self.create_publisher(
            String, "mebot/seat/manual_control", 10
        )

        # make subscriber for system state, message should be string
        self.system_state_subscriber = self.create_subscription(
            String, "state", self.system_state_callback, 10
        )

    def test_ue(self):
        if self.ue.is_connected():
            self.test_ue_counter += 1
            if self.test_ue_counter == 3:
                print("UE connection test successful, calling Mebot function...")
                self.ue.call_function("Mebot", {})
            if self.test_ue_counter == 5:
                print("get UE preset functions and properties...")
                self.ue.get_preset_functions_porperties()

        else:
            self.test_ue_counter = 0

    def publish_connection_status(self):
        msg = Bool()
        msg.data = self.ue.is_connected()
        self.connection_publisher.publish(msg)

    def destroy_node(self):
        # Signal websocket handlers to stop
        self.ue.shutdown()
        # Stop the event loop
        self.loop.call_soon_threadsafe(self.loop.stop)
        # Wait for the event loop thread to finish
        timeout = 5  # seconds
        start = time.time()
        while self.loop.is_running() and (time.time() - start) < timeout:
            time.sleep(0.1)
        self.mapfile.close()
        self.shm.unlink()
        super().destroy_node()

    def system_state_callback(self, msg):
        state = msg.data
        print(f"Received system state: {state}")

    def write_data_to_shm(self, index, data):
        # Write data to shared memory
        self.mapfile.seek(index)
        self.mapfile.write(data.encode("utf-8"))
        self.mapfile.flush()


def main():
    rclpy.init()
    node = GuiMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
