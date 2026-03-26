import posix_ipc
import mmap
import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from .unreal_remote_websocket import UnrealRemoteWebsocket
from .streaming.sender import StreamSender
from gui_interfaces.srv import UserInputs
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup


class GuiBridge(Node):
    instance = None

    def __init__(
        self,
    ):
        super().__init__("Gui_bridge")
        print("Gui_bridge node has been started.")
        GuiBridge.instance = self

        self.declare_parameter("ue_host", "192.168.1.21")
        self.host = self.get_parameter("ue_host").get_parameter_value().string_value

        self.declare_parameter("use_shared_memory", False)
        self.use_shared_memory = (
            self.get_parameter("use_shared_memory").get_parameter_value().bool_value
        )

        self.declare_parameter("shm_size", 1024 * 1024)  # 1MB default size
        self.shm_size = (
            self.get_parameter("shm_size").get_parameter_value().integer_value
        )

        self.declare_parameter("ue_preset", "RCPS")
        self.ue_preset = (
            self.get_parameter("ue_preset").get_parameter_value().string_value
        )
        print(f"Using UE host: {self.host}, UE preset: {self.ue_preset}")

        self.ue = UnrealRemoteWebsocket(host=self.host, preset=self.ue_preset)
        self.stream_sender = StreamSender(host=self.host, port=30030)
        self.stream_timer = self.create_timer(0.1, self.check_streamer_connection)

        self._cb_group = ReentrantCallbackGroup()
        self.arm_joints = None
        self.base_joints = None

        if self.use_shared_memory:
            # Create shared memory and map it
            self.shm = posix_ipc.SharedMemory(
                "/ros_ue_shm",
                posix_ipc.O_CREAT,
                size=self.shm_size,  # default size is 1MB
            )
            # Map it
            self.mapfile = mmap.mmap(self.shm.fd, self.shm_size)
            self.shm.close_fd()

        self.init_publisher()
        self.init_subscriber()
        self.init_service()

        self.test_ue_counter = 0
        self.test_ue_timer = self.create_timer(0.1, self.test_ue)

    def init_service(self):
        # make service client for user input, request should be string
        self.user_input_service_client = self.create_client(
            UserInputs, "/GuiBridge/receive_input", callback_group=self._cb_group
        )

    def init_publisher(self):
        # make publisher for user input, message should be string
        self.connection_publisher = self.create_publisher(
            Bool, "/GuiBridge/gui_connection", 10
        )
        self.connection_publisher_timer = self.create_timer(
            1.0, self.publish_connection_status
        )

    def init_subscriber(self):
        # make subscriber for system state, message should be string
        self.system_state_subscriber = self.create_subscription(
            String,
            "/system/state",
            self.system_state_callback,
            10,
            callback_group=self._cb_group,
        )
        # make subscriber for arm joint state,
        self.arm_joint_state_subscriber = self.create_subscription(
            JointState,
            "/arm/joint_states",
            self.arm_joint_state_callback,
            10,
            callback_group=self._cb_group,
        )
        self.base_joint_state_subscriber = self.create_subscription(
            JointState,
            "/base/joint_states",
            self.base_joint_state_callback,
            10,
            callback_group=self._cb_group,
        )

    def check_streamer_connection(self):
        # check UE connection and connect/disconnect StreamSender accordingly
        if self.ue.is_connected() and not self.stream_sender.is_connected():
            try:
                self.stream_sender.connect()
            except Exception as e:
                self.get_logger().error(f"Failed to connect StreamSender: {e}")
        elif not self.ue.is_connected() and self.stream_sender.is_connected():
            self.stream_sender.disconnect()

        # if self.stream_sender.is_connected():
        #     print ("StreamSender is connected, sending stream data...")

    def arm_joint_state_callback(self, msg):
        self.arm_joints = msg

    def base_joint_state_callback(self, msg):
        self.base_joints = msg

    def test_ue(self):
        if self.ue.is_connected():
            self.test_ue_counter += 1
            # if self.test_ue_counter == 5:
            #     print("UE connection test successful, calling Mebot function...")
            #     self.ue.call_function("setJoints", {'Values': [10, 10, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10]})
            # if self.test_ue_counter == 3:
            #     print("get UE preset functions and properties...")
            #     self.ue.get_preset_functions_porperties()
            self.set_ui_joints()
        else:
            self.test_ue_counter = 0

    def set_ui_joints(self):
        arr = [0.0] * 53
        arm_joint_index_offset = 46  # index for link 0
        arm_gripper_joint_left_index = 38  # index for left gripper joint
        arm_gripper_joint_right_index = 40  # index for right gripper joint
        chair_fc_joint_index = 16  # index for chair foot control joint
        chair_rc_joint_index = 25  # index for chair rotation control joint
        chair_ml_joint_index = 1  # index for chair movement control joint
        chair_mr_joint_index = 8  # index for chair movement control joint
        chair_ml_carriage_index = 0  # index for chair movement carriage joint
        chair_mr_carriage_index = 4  # index for chair movement carriage joint
        chair_ml_wheel_index = 2  # index for chair movement wheel joint
        chair_mr_wheel_index = 9  # index for chair movement wheel joint
        if len(self.base_joints.position) >= 8:
            for i in range(7):  # Only take the first 7 joints for the arm joint
                arr[arm_joint_index_offset + i] = (
                    self.arm_joints.position[i] * 180.0 / 3.14159
                )  # Convert radians to degrees
            arr[arm_joint_index_offset + 0] = -arr[arm_joint_index_offset + 0]
            arr[arm_joint_index_offset + 4] = -arr[arm_joint_index_offset + 4]
            arr[arm_joint_index_offset + 5] = -arr[arm_joint_index_offset + 5]
            arr[arm_joint_index_offset + 6] = -arr[arm_joint_index_offset + 6]
            # the 8th joint is the gripper, we can set it to 0 for now
            # set gripper joint angle to 0 or 45 for close and open
            arr[arm_gripper_joint_left_index] = (
                0.0 if self.arm_joints.position[7] < 0.01 else -45.0
            )
            arr[arm_gripper_joint_right_index] = (
                0.0 if self.arm_joints.position[7] < 0.01 else 45.0
            )
        if len(self.base_joints.position) >= 8:
            arr[chair_fc_joint_index] = self.base_joints.position[0] * 180.0 / 3.14159
            arr[chair_rc_joint_index] = self.base_joints.position[1] * 180.0 / 3.14159
            arr[chair_mr_joint_index] = self.base_joints.position[2] * 180.0 / 3.14159
            arr[chair_ml_joint_index] = self.base_joints.position[3] * 180.0 / 3.14159
            arr[chair_ml_carriage_index] = self.base_joints.position[4] * 100.0  # to cm
            arr[chair_mr_carriage_index] = self.base_joints.position[5] * 100.0  # to cm
            arr[chair_ml_wheel_index] = self.base_joints.position[6] * 180.0 / 3.14159
            arr[chair_mr_wheel_index] = self.base_joints.position[7] * 180.0 / 3.14159

        self.ue.call_function("setJoints", {"Values": arr})

    def set_system_state(self):
        self.ue.call_function("setSystemState", {"Values": self._system_state})

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
        if self.use_shared_memory:
            self.mapfile.close()
            self.shm.unlink()
        super().destroy_node()

    def system_state_callback(self, msg):
        state = msg.data
        print(f"Received system state: {state}")
        self._system_state = msg.data

    def write_data_to_shm(self, index, data):
        if self.use_shared_memory:
            # Write data to shared memory
            self.mapfile.seek(index)
            self.mapfile.write(data.encode("utf-8"))
            self.mapfile.flush()


def main():
    rclpy.init()
    node = GuiBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
