import posix_ipc
import mmap
import asyncio
import websockets
import rclpy
import threading
import time
import json

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool


class GuiMonitor(Node):
    instance = None

    def __init__(self, shm_size=1024 * 1024):
        super().__init__("GuiMonitor")
        GuiMonitor.instance = self
        print("GuiMonitor node has been started.")

        self.ws_client_ethernet = None
        self.ws_client_wifi = None
        self.ws_shutdown = False  # Flag to signal websocket handlers to stop
        self.loop = asyncio.new_event_loop()
        t = threading.Thread(target=self.start_async_loop, daemon=True)
        t.start()
        asyncio.run_coroutine_threadsafe(self.ws_client_ethernet_handler(), self.loop)
        # asyncio.run_coroutine_threadsafe(self.ws_client_wifi_handler(), self.loop)
        asyncio.run_coroutine_threadsafe(self.toggle_gripper(), self.loop)

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
        self.gripper_opened = False
        self.msg_id = 0

    def start_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def ws_client_ethernet_handler(self):
        uri = "ws://192.168.68.63:30020"
        while not self.ws_shutdown:
            try:
                async with websockets.connect(uri) as ws:
                    self.ws_client_ethernet = ws
                    print("Ethernet GUI connected.")
                    async for message in ws:
                        command = json.loads(message)
                        print(f"Received command from Ethernet GUI: {command}")
                        if self.ws_shutdown:
                            break
                        # Publish the message to ROS topic
                        # ros_msg = String()
                        # ros_msg.data = message
                        # self.input_publisher.publish(ros_msg)
            except Exception as e:
                if not self.ws_shutdown:
                    print(f"Ethernet GUI connection error: {e}")
            finally:
                self.ws_client_ethernet = None
                if not self.ws_shutdown:
                    print("Ethernet GUI disconnected.")
                    await asyncio.sleep(3)  # Wait before trying to reconnect

    async def ws_client_wifi_handler(self):
        uri = "ws://127.0.0.1:5678"
        while not self.ws_shutdown:
            try:
                async with websockets.connect(uri) as ws:
                    self.ws_client_wifi = ws
                    print("WiFi GUI connected.")
                    async for message in ws:
                        if self.ws_shutdown:
                            break
                        print(f"Received from WiFi GUI: {message}")
                        # should not receive message from wifi gui. just print it for debug
            except Exception as e:
                if not self.ws_shutdown:
                    print(f"WiFi GUI connection error: {e}")
            finally:
                self.ws_client_wifi = None
                if not self.ws_shutdown:
                    print("WiFi GUI disconnected.")
                    await asyncio.sleep(5)  # Wait before trying to reconnect

    def publish_connection_status(self):
        self.ethernet_gui_connected = self.ws_client_ethernet is not None
        self.wifi_gui_connected = self.ws_client_wifi is not None
        msg = Bool()
        msg.data = self.ethernet_gui_connected
        self.connection_publisher.publish(msg)

    def destroy_node(self):
        # Signal websocket handlers to stop
        self.ws_shutdown = True
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

    async def toggle_gripper(self):
        while not self.ws_shutdown:
            self.gripper_opened = not self.gripper_opened
            if self.ws_client_ethernet is not None:
                self.msg_id += 1
                print(
                    f"Toggling gripper to {'close' if self.gripper_opened else 'open'}"
                )
                if self.gripper_opened:
                    command = {
                        "MessageName": "http",
                        "Parameters": {
                            "Url": "/remote/object/call",
                            "Verb": "PUT",
                            "Body": {
                                "ObjectPath": "/Game/VehicleTemplate/Maps/UEDPIE_0_VehicleBasic.VehicleBasic:PersistentLevel.BP_Mebot_Ramms_C_0.GripperController",
                                "functionName": "Close",
                                "parameters": {},
                            },
                        },
                        "Id": str(self.msg_id),
                    }
                    await self.ws_client_ethernet.send(json.dumps(command))
                else:
                    command = {
                        "MessageName": "http",
                        "Parameters": {
                            "Url": "/remote/object/call",
                            "Verb": "PUT",
                            "Body": {
                                "ObjectPath": "/Game/VehicleTemplate/Maps/UEDPIE_0_VehicleBasic.VehicleBasic:PersistentLevel.BP_Mebot_Ramms_C_0.GripperController",
                                "functionName": "Open",
                                "parameters": {},
                            },
                        },
                        "Id": str(self.msg_id),
                    }
                    await self.ws_client_ethernet.send(json.dumps(command))
            await asyncio.sleep(3)  # Wait before trying to reconnect


def main():
    rclpy.init()
    node = GuiMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
