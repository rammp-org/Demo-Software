import posix_ipc
import mmap
import rclpy
import time
import enum
import threading

from dataclasses import dataclass
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from .unreal_remote_websocket import UnrealRemoteWebsocket
from .streaming.sender import StreamSender
from gui_interfaces.srv import UserInputs
from sensor_msgs.msg import CameraInfo, JointState, Image
from rclpy.callback_groups import ReentrantCallbackGroup
from realsense2_camera_msgs.msg import Extrinsics
from neu_navigation_interfaces.msg import CurbInfo
from cmu_door_opener_interfaces.msg import ButtonInfo
from scipy.spatial.transform import Rotation as R


@dataclass
class Vector:
    X: float
    Y: float
    Z: float


@dataclass
class Quaternion:
    X: float
    Y: float
    Z: float
    W: float


@dataclass
class Transform:
    Translation: Vector
    Rotation: Quaternion
    Scale3D: Vector


@dataclass
class CupInfo:
    Pose: Transform
    Success: bool
    Message: str


# @dataclass
# class ButtonInfo:
#     BoundingBox: list[float]  # [x_min, y_min, x_max, y_max] in pixel coordinates
#     Confidence: float
#     Pose: Transform
#     CanPress: bool


class UserInputString(enum.Enum):
    # get all userinput from UserInputs.srv file as enum
    CHAIR_CONTROL_MAIN = UserInputs.Request.CHAIR_CONTROL_MAIN
    CHAIR_SELFLEVELING_ON = UserInputs.Request.CHAIR_SELFLEVELING_ON
    CHAIR_SELFLEVELING_OFF = UserInputs.Request.CHAIR_SELFLEVELING_OFF
    CHAIR_SEAT_ELEVATE_UP = UserInputs.Request.CHAIR_SEAT_ELEVATE_UP
    CHAIR_SEAT_ELEVATE_DOWN = UserInputs.Request.CHAIR_SEAT_ELEVATE_DOWN
    CHAIR_SEAT_ELEVATE_HOME = UserInputs.Request.CHAIR_SEAT_ELEVATE_HOME
    CHAIR_SEAT_RECLINE_FORWARD = UserInputs.Request.CHAIR_SEAT_RECLINE_FORWARD
    CHAIR_SEAT_RECLINE_BACK = UserInputs.Request.CHAIR_SEAT_RECLINE_BACK
    CHAIR_SEAT_RECLINE_HOME = UserInputs.Request.CHAIR_SEAT_RECLINE_HOME
    CHAIR_SEAT_LTILT_LEFT = UserInputs.Request.CHAIR_SEAT_LTILT_LEFT
    CHAIR_SEAT_LTILT_RIGHT = UserInputs.Request.CHAIR_SEAT_LTILT_RIGHT
    CHAIR_SEAT_LTILT_HOME = UserInputs.Request.CHAIR_SEAT_LTILT_HOME
    CHAIR_SEAT_HOME = UserInputs.Request.CHAIR_SEAT_HOME
    CHAIR_CURB_NAVIGATION = UserInputs.Request.CHAIR_CURB_NAVIGATION
    CHAIR_CURB_ASCEND = UserInputs.Request.CHAIR_CURB_ASCEND
    CHAIR_CURB_DESCEND = UserInputs.Request.CHAIR_CURB_DESCEND
    CHAIR_CURB_CANCEL = UserInputs.Request.CHAIR_CURB_CANCEL
    ARM_CONTROL_MAIN = UserInputs.Request.ARM_CONTROL_MAIN
    ARM_RETRACT = UserInputs.Request.ARM_RETRACT
    ARM_HOME = UserInputs.Request.ARM_HOME
    ARM_MANUAL_ON = UserInputs.Request.ARM_MANUAL_ON
    ARM_MANUAL_OFF = UserInputs.Request.ARM_MANUAL_OFF
    ARM_OPEN_DOOR = UserInputs.Request.ARM_OPEN_DOOR
    ARM_OPEN_DOOR_CONFIRM = UserInputs.Request.ARM_OPEN_DOOR_CONFIRM
    ARM_ORDER_DRINK = UserInputs.Request.ARM_ORDER_DRINK
    ARM_ORDER_DRINK_RELEASE_CUP = UserInputs.Request.ARM_ORDER_DRINK_RELEASE_CUP
    ARM_ORDER_DRINK_RECEIVE = UserInputs.Request.ARM_ORDER_DRINK_RECEIVE
    ARM_ORDER_DRINK_RECEIVE_CONFIRM = UserInputs.Request.ARM_ORDER_DRINK_RECEIVE_CONFIRM
    ARM_CUP_STABLE_ON = UserInputs.Request.ARM_CUP_STABLE_ON
    ARM_CUP_STABLE_OFF = UserInputs.Request.ARM_CUP_STABLE_OFF
    ARM_DRINKING_START = UserInputs.Request.ARM_DRINKING_START
    ARM_DRINKING_FINISH = UserInputs.Request.ARM_DRINKING_FINISH
    ARM_CUP_BACK = UserInputs.Request.ARM_CUP_BACK
    ARM_CANCEL = UserInputs.Request.ARM_CANCEL
    RESET = UserInputs.Request.RESET
    ESTOP = UserInputs.Request.ESTOP
    CONFIRM = UserInputs.Request.CONFIRM
    CANCEL = UserInputs.Request.CANCEL


def rotation_matrix_to_euler_zyx(R):
    sy = -R[2, 0]
    epsilon = 1e-6

    if abs(sy) < 1 - epsilon:
        pitch = np.arcsin(sy)
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock
        pitch = np.pi / 2 if sy < 0 else -np.pi / 2
        roll = 0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)  # degrees


class GuiBridge(Node):
    instance = None

    def __init__(
        self,
    ):
        super().__init__("Gui_bridge_node")
        print("Gui_bridge node has been started.")
        GuiBridge.instance = self

        self.declare_parameter("ue_host", "192.168.68.65")
        self.host = self.get_parameter("ue_host").get_parameter_value().string_value

        self.declare_parameter("use_shared_memory", False)
        self.use_shared_memory = (
            self.get_parameter("use_shared_memory").get_parameter_value().bool_value
        )

        self.declare_parameter("shm_size", "1000000")  # 1MB default size
        self.shm_size = int(
            self.get_parameter("shm_size").get_parameter_value().string_value
        )

        self.declare_parameter("ue_preset", "RCPS")
        self.ue_preset = (
            self.get_parameter("ue_preset").get_parameter_value().string_value
        )
        print(f"Using UE host: {self.host}, UE preset: {self.ue_preset}")

        self.declare_parameter("wrist_camera_namespace", "/camera/wrist")
        self.declare_parameter("nav_camera_namespace_1", "/camera/nav1")
        self.declare_parameter("nav_camera_namespace_2", "/camera/nav2")
        self.declare_parameter("rear_camera_namespace", "/camera/rear")
        self.wrist_camera_namespace = (
            self.get_parameter("wrist_camera_namespace")
            .get_parameter_value()
            .string_value
        )
        self.nav_camera_namespace_1 = (
            self.get_parameter("nav_camera_namespace_1")
            .get_parameter_value()
            .string_value
        )
        self.nav_camera_namespace_2 = (
            self.get_parameter("nav_camera_namespace_2")
            .get_parameter_value()
            .string_value
        )
        self.rear_camera_namespace = (
            self.get_parameter("rear_camera_namespace")
            .get_parameter_value()
            .string_value
        )
        self.declare_parameter("image_channel", 0)
        self.image_channel = (
            self.get_parameter("image_channel").get_parameter_value().integer_value
        )
        print(f"Image channel: {self.image_channel}")
        self.declare_parameter("depth_channel", 100)
        self.depth_channel = (
            self.get_parameter("depth_channel").get_parameter_value().integer_value
        )
        print(f"Depth channel: {self.depth_channel}")
        self.declare_parameter("mask_channel", 200)
        self.mask_channel = (
            self.get_parameter("mask_channel").get_parameter_value().integer_value
        )
        print(f"Mask channel: {self.mask_channel}")

        self.ue = UnrealRemoteWebsocket(
            host=self.host,
            preset=self.ue_preset,
            user_input_callback=self.user_input_callback,
        )
        self.stream_sender = StreamSender(host=self.host, port=30030)
        self.stream_check_timer = self.create_timer(1.0, self.check_streamer_connection)
        self.update_ue_timer = self.create_timer(0.1, self.ue_update)

        self._cb_group = ReentrantCallbackGroup()
        self.arm_joints = None
        self.base_joints = None
        self._system_state = None

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

        # self.test_ue_counter = 0
        # self.test_ue_timer = self.create_timer(1.0, self.test_ue)

    def init_service(self):
        # make service client for user input, request should be string
        self.user_input_service_client = self.create_client(
            UserInputs, "/GuiBridge/user_input", callback_group=self._cb_group
        )

    def send_user_input(self, input: str):
        self.get_logger().info(f"Sending user input to ROS: {input}")
        if self.user_input_service_client.wait_for_service(timeout_sec=1.0):
            request = UserInputs.Request()
            request.input = input
            future = self.user_input_service_client.call_async(request)
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            event.wait(timeout=5.0)
            if not future.done():
                self.get_logger().error("User input service call timed out.")
                return False
            if future.result() is not None:
                self.get_logger().info(f"User input '{input}' sent successfully.")
                return future.result().success
            else:
                self.get_logger().error("User input service call failed.")
                return False
        else:
            self.get_logger().error("User input service is not available.")
            return False

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

        # init camera subscribers
        # init camera msg to None first.
        self.wrist_camera_image_info = None
        self.wrist_camera_depth_info = None
        self.wrist_camera_image = None
        self.wrist_camera_depth = None
        self.wrist_camera_extrinsics = None
        self.nav_camera_1_image_info = None
        self.nav_camera_1_depth_info = None
        self.nav_camera_1_image = None
        self.nav_camera_1_depth = None
        self.nav_camera_1_extrinsics = None
        self.nav_camera_2_image_info = None
        self.nav_camera_2_depth_info = None
        self.nav_camera_2_image = None
        self.nav_camera_2_depth = None
        self.nav_camera_2_extrinsics = None
        self.rear_camera_image_info = None
        self.rear_camera_depth_info = None
        self.rear_camera_image = None
        self.rear_camera_depth = None
        self.rear_camera_extrinsics = None
        # wrist camera:
        self.wrist_camera_image_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.wrist_camera_namespace}/color/camera_info",
            self.wrist_camera_image_info_callback,
            10,
            callback_group=self._cb_group,
        )
        self.wrist_camera_depth_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.wrist_camera_namespace}/depth/camera_info",
            self.wrist_camera_depth_info_callback,
            10,
            callback_group=self._cb_group,
        )
        self.wrist_camera_image_sub = self.create_subscription(
            Image,
            f"{self.wrist_camera_namespace}/color/image_raw",
            self.wrist_camera_image_callback,
            10,
            callback_group=self._cb_group,
        )
        self.wrist_camera_depth_sub = self.create_subscription(
            Image,
            f"{self.wrist_camera_namespace}/depth/image_rect_raw",
            self.wrist_camera_depth_callback,
            10,
            callback_group=self._cb_group,
        )
        self.wrist_camera_extrinsics_sub = self.create_subscription(
            Extrinsics,
            f"{self.wrist_camera_namespace}/extrinsics/depth_to_color",
            self.wrist_camera_extrinsics_callback,
            10,
            callback_group=self._cb_group,
        )

        # nav camera 1
        self.nav_camera_1_image_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.nav_camera_namespace_1}/color/camera_info",
            self.nav_camera_1_image_info_callback,
            10,
            callback_group=self._cb_group,
        )
        self.nav_camera_1_depth_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.nav_camera_namespace_1}/depth/camera_info",
            self.nav_camera_1_depth_info_callback,
            10,
            callback_group=self._cb_group,
        )
        self.nav_camera_1_image_sub = self.create_subscription(
            Image,
            f"{self.nav_camera_namespace_1}/color/image_raw",
            self.nav_camera_1_image_callback,
            10,
            callback_group=self._cb_group,
        )
        self.nav_camera_1_depth_sub = self.create_subscription(
            Image,
            f"{self.nav_camera_namespace_1}/depth/image_rect_raw",
            self.nav_camera_1_depth_callback,
            10,
            callback_group=self._cb_group,
        )
        self.nav_camera_1_extrinsics_sub = self.create_subscription(
            Extrinsics,
            f"{self.nav_camera_namespace_1}/extrinsics/depth_to_color",
            self.nav_camera_1_extrinsics_callback,
            10,
            callback_group=self._cb_group,
        )

        # nav camera 2
        self.nav_camera_2_image_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.nav_camera_namespace_2}/color/camera_info",
            self.nav_camera_2_image_info_callback,
            10,
            callback_group=self._cb_group,
        )
        self.nav_camera_2_depth_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.nav_camera_namespace_2}/depth/camera_info",
            self.nav_camera_2_depth_info_callback,
            10,
            callback_group=self._cb_group,
        )
        self.nav_camera_2_image_sub = self.create_subscription(
            Image,
            f"{self.nav_camera_namespace_2}/color/image_raw",
            self.nav_camera_2_image_callback,
            10,
            callback_group=self._cb_group,
        )
        self.nav_camera_2_depth_sub = self.create_subscription(
            Image,
            f"{self.nav_camera_namespace_2}/depth/image_rect_raw",
            self.nav_camera_2_depth_callback,
            10,
            callback_group=self._cb_group,
        )
        self.nav_camera_2_extrinsics_sub = self.create_subscription(
            Extrinsics,
            f"{self.nav_camera_namespace_2}/extrinsics/depth_to_color",
            self.nav_camera_2_extrinsics_callback,
            10,
            callback_group=self._cb_group,
        )
        # rear camera
        self.rear_camera_image_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.rear_camera_namespace}/color/camera_info",
            self.rear_camera_image_info_callback,
            10,
            callback_group=self._cb_group,
        )
        self.rear_camera_depth_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.rear_camera_namespace}/depth/camera_info",
            self.rear_camera_depth_info_callback,
            10,
            callback_group=self._cb_group,
        )
        self.rear_camera_image_sub = self.create_subscription(
            Image,
            f"{self.rear_camera_namespace}/color/image_raw",
            self.rear_camera_image_callback,
            10,
            callback_group=self._cb_group,
        )
        self.rear_camera_depth_sub = self.create_subscription(
            Image,
            f"{self.rear_camera_namespace}/depth/image_rect_raw",
            self.rear_camera_depth_callback,
            10,
            callback_group=self._cb_group,
        )
        self.rear_camera_extrinsics_sub = self.create_subscription(
            Extrinsics,
            f"{self.rear_camera_namespace}/extrinsics/depth_to_color",
            self.rear_camera_extrinsics_callback,
            10,
            callback_group=self._cb_group,
        )

        # curb info
        self.curb_info_sub = self.create_subscription(
            CurbInfo,
            "/nav/curb/info",
            self.curb_info_callback,
            10,
            callback_group=self._cb_group,
        )
        # curb mask
        self.curb_mask_sub = self.create_subscription(
            Image,
            "/perception/curb_mask",
            self.curb_mask_callback,
            10,
            callback_group=self._cb_group,
        )

        # door button info
        self.door_button_info_sub = self.create_subscription(
            ButtonInfo,
            "/arm/door/button_info",
            self.door_button_info_callback,
            10,
            callback_group=self._cb_group,
        )

    def door_button_info_callback(self, msg: ButtonInfo):
        self.update_button_info(msg)
        self.send_mask(msg.segmentation_mask, "door_button")

    def curb_mask_callback(self, msg: Image):
        self.send_mask(msg, "nav")

    def curb_info_callback(self, msg: CurbInfo):
        self.update_curb_info(msg)

    # camera subscription callbacks
    def wrist_camera_image_info_callback(self, msg: CameraInfo):
        self.wrist_camera_image_info = msg

    def wrist_camera_depth_info_callback(self, msg: CameraInfo):
        self.wrist_camera_depth_info = msg

    def wrist_camera_image_callback(self, msg: Image):
        self.wrist_camera_image = msg
        self.send_wrist_camera_image()

    def wrist_camera_depth_callback(self, msg: Image):
        self.wrist_camera_depth = msg
        self.send_wrist_camera_depth()

    def wrist_camera_extrinsics_callback(self, msg: Extrinsics):
        self.wrist_camera_extrinsics = msg

    def nav_camera_1_image_info_callback(self, msg: CameraInfo):
        self.nav_camera_1_image_info = msg

    def nav_camera_1_depth_info_callback(self, msg: CameraInfo):
        self.nav_camera_1_depth_info = msg

    def nav_camera_1_image_callback(self, msg: Image):
        self.nav_camera_1_image = msg
        self.send_nav_camera_1_image()

    def nav_camera_1_depth_callback(self, msg: Image):
        self.nav_camera_1_depth = msg
        self.send_nav_camera_1_depth()

    def nav_camera_1_extrinsics_callback(self, msg: Extrinsics):
        self.nav_camera_1_extrinsics = msg

    def nav_camera_2_image_info_callback(self, msg: CameraInfo):
        self.nav_camera_2_image_info = msg

    def nav_camera_2_depth_info_callback(self, msg: CameraInfo):
        self.nav_camera_2_depth_info = msg

    def nav_camera_2_image_callback(self, msg: Image):
        self.nav_camera_2_image = msg
        self.send_nav_camera_2_image()

    def nav_camera_2_depth_callback(self, msg: Image):
        self.nav_camera_2_depth = msg
        self.send_nav_camera_2_depth()

    def nav_camera_2_extrinsics_callback(self, msg: Extrinsics):
        self.nav_camera_2_extrinsics = msg

    def rear_camera_image_info_callback(self, msg: CameraInfo):
        self.rear_camera_image_info = msg

    def rear_camera_depth_info_callback(self, msg: CameraInfo):
        self.rear_camera_depth_info = msg

    def rear_camera_image_callback(self, msg: Image):
        self.rear_camera_image = msg
        self.send_rear_camera_image()

    def rear_camera_depth_callback(self, msg: Image):
        self.rear_camera_depth = msg
        self.send_rear_camera_depth()

    def rear_camera_extrinsics_callback(self, msg: Extrinsics):
        self.rear_camera_extrinsics = msg

    def send_image(self, image: Image, image_info: CameraInfo, source: str):
        if self.stream_sender.is_connected():
            if image is not None and image_info is not None:
                try:
                    width = image.width
                    height = image.height
                    meta = {
                        "w": width,
                        "h": height,
                        "source": source,
                        "fmt": image.encoding,
                    }
                    self.stream_sender.send_image(
                        channel=self.image_channel,
                        image_bytes=image.data.tobytes(),
                        width=width,
                        height=height,
                        metadata=meta,
                    )
                except Exception as e:
                    self.get_logger().warn(f"Failed to send {source} image: {e}")

    def send_depth(
        self, depth: Image, depth_info: CameraInfo, extrinsics: Extrinsics, source: str
    ):
        if self.stream_sender.is_connected():
            if depth is not None and depth_info is not None:
                try:
                    width = depth.width
                    height = depth.height
                    fx = depth_info.k[0]
                    fy = depth_info.k[4]
                    cx = depth_info.k[2]
                    cy = depth_info.k[5]

                    meta = {
                        "w": width,
                        "h": height,
                        "source": source,
                        "fmt": depth.encoding,
                    }
                    meta["intrinsics"] = {
                        "fx": fx,
                        "fy": fy,
                        "cx": cx,
                        "cy": cy,
                    }
                    if extrinsics is not None:
                        # from rotation matrix to get Euler angles
                        roll, pitch, yaw = rotation_matrix_to_euler_zyx(
                            np.array(extrinsics.rotation).reshape(3, 3)
                        )
                        meta["transform"] = {
                            "x": extrinsics.translation.x,
                            "y": extrinsics.translation.y,
                            "z": extrinsics.translation.z,
                            "pitch": pitch,
                            "roll": roll,
                            "yaw": yaw,
                        }
                        meta["transform_space"] = "relative"
                    self.stream_sender.send_image(
                        channel=self.depth_channel,
                        image_bytes=depth.data.tobytes(),
                        width=width,
                        height=height,
                        metadata=meta,
                    )
                except Exception as e:
                    self.get_logger().warn(f"Failed to send {source} depth image: {e}")

    def send_mask(self, mask: Image, source: str):
        if self.stream_sender.is_connected():
            if mask is not None:
                try:
                    width = mask.width
                    height = mask.height
                    meta = {
                        "w": width,
                        "h": height,
                        "source": source,
                        "fmt": mask.encoding,
                    }
                    self.stream_sender.send_image(
                        channel=self.mask_channel,
                        image_bytes=mask.data.tobytes(),
                        width=width,
                        height=height,
                        metadata=meta,
                    )
                except Exception as e:
                    self.get_logger().warn(f"Failed to send {source} mask image: {e}")

    def send_wrist_camera_image(self):
        self.send_image(
            image=self.wrist_camera_image,
            image_info=self.wrist_camera_image_info,
            source="wrist",
        )

    def send_wrist_camera_depth(self):
        self.send_depth(
            depth=self.wrist_camera_depth,
            depth_info=self.wrist_camera_depth_info,
            extrinsics=self.wrist_camera_extrinsics,
            source="wrist",
        )

    def send_nav_camera_1_image(self):
        self.send_image(
            image=self.nav_camera_1_image,
            image_info=self.nav_camera_1_image_info,
            source="nav_1",
        )

    def send_nav_camera_1_depth(self):
        self.send_depth(
            depth=self.nav_camera_1_depth,
            depth_info=self.nav_camera_1_depth_info,
            extrinsics=self.nav_camera_1_extrinsics,
            source="nav_1",
        )

    def send_nav_camera_2_image(self):
        self.send_image(
            image=self.nav_camera_2_image,
            image_info=self.nav_camera_2_image_info,
            source="nav_2",
        )

    def send_nav_camera_2_depth(self):
        self.send_depth(
            depth=self.nav_camera_2_depth,
            depth_info=self.nav_camera_2_depth_info,
            extrinsics=self.nav_camera_2_extrinsics,
            source="nav_2",
        )

    def send_rear_camera_image(self):
        self.send_image(
            image=self.rear_camera_image,
            image_info=self.rear_camera_image_info,
            source="rear",
        )

    def send_rear_camera_depth(self):
        self.send_depth(
            depth=self.rear_camera_depth,
            depth_info=self.rear_camera_depth_info,
            extrinsics=self.rear_camera_extrinsics,
            source="rear",
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
            if self.test_ue_counter == 3:
                print("get UE preset functions and properties...")
                self.ue.get_preset_functions_porperties()
            # self.set_ui_joints()
            if self.test_ue_counter == 4:
                self._system_state = "TestState"
            if self.test_ue_counter == 5:
                self._system_state = None
                print("Testing sending curb info to UE...")
                curbInfo = CurbInfo(
                    Distance=0.5,
                    Height=0.2,
                    Pose=Transform(
                        Translation=Vector(X=1.0, Y=2.0, Z=3.0),
                        Rotation=Quaternion(X=0.0, Y=0.0, Z=0.0, W=1.0),
                        Scale3D=Vector(X=1.0, Y=1.0, Z=1.0),
                    ),
                    Success=True,
                    Message="Curb info updated successfully",
                )
                # print("CurbInfo to send:", asdict(curbInfo))

                self.update_curb_info(curbInfo)
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
        if self.base_joints is not None and len(self.base_joints.position) >= 8:
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
        if self.base_joints is not None and len(self.base_joints.position) >= 8:
            arr[chair_fc_joint_index] = self.base_joints.position[0] * 180.0 / 3.14159
            arr[chair_rc_joint_index] = self.base_joints.position[1] * 180.0 / 3.14159
            arr[chair_mr_joint_index] = self.base_joints.position[2] * 180.0 / 3.14159
            arr[chair_ml_joint_index] = self.base_joints.position[3] * 180.0 / 3.14159
            arr[chair_ml_carriage_index] = self.base_joints.position[4] * 100.0  # to cm
            arr[chair_mr_carriage_index] = self.base_joints.position[5] * 100.0  # to cm
            arr[chair_ml_wheel_index] = self.base_joints.position[6] * 180.0 / 3.14159
            arr[chair_mr_wheel_index] = self.base_joints.position[7] * 180.0 / 3.14159

        if self.base_joints is not None or self.arm_joints is not None:
            self.ue.call_function("setJoints", {"Values": arr})

    def send_system_state_to_ue(self):
        if self.ue.is_connected() and self._system_state is not None:
            self.ue.call_function(
                "UpdateSystemState", {"SystemState": str(self._system_state)}
            )

    def update_curb_info(self, curb_info: CurbInfo):
        if self.ue.is_connected():
            curbInfoDict = {
                "Message": curb_info.Message,
                "Success": curb_info.Success,
                "Pose": {
                    "Translation": {
                        "X": curb_info.Pose.Translation.X,
                        "Y": curb_info.Pose.Translation.Y,
                        "Z": curb_info.Pose.Translation.Z,
                    },
                    "Rotation": {
                        "X": curb_info.Pose.Rotation.X,
                        "Y": curb_info.Pose.Rotation.Y,
                        "Z": curb_info.Pose.Rotation.Z,
                        "W": curb_info.Pose.Rotation.W,
                    },
                    "Scale3D": {
                        "X": curb_info.Pose.Scale3D.X,
                        "Y": curb_info.Pose.Scale3D.Y,
                        "Z": curb_info.Pose.Scale3D.Z,
                    },
                },
                "Distance": curb_info.Distance,
                "Height": curb_info.Height,
            }
            self.ue.call_function("UpdateCurbInfo", curbInfoDict)

    def update_cup_info(self, cup_info: CupInfo):
        if self.ue.is_connected():
            cupInfoDict = {
                "Message": cup_info.Message,
                "Success": cup_info.Success,
                "Pose": {
                    "Translation": {
                        "X": cup_info.Pose.Translation.X,
                        "Y": cup_info.Pose.Translation.Y,
                        "Z": cup_info.Pose.Translation.Z,
                    },
                    "Rotation": {
                        "X": cup_info.Pose.Rotation.X,
                        "Y": cup_info.Pose.Rotation.Y,
                        "Z": cup_info.Pose.Rotation.Z,
                        "W": cup_info.Pose.Rotation.W,
                    },
                    "Scale3D": {
                        "X": cup_info.Pose.Scale3D.X,
                        "Y": cup_info.Pose.Scale3D.Y,
                        "Z": cup_info.Pose.Scale3D.Z,
                    },
                },
            }
            self.ue.call_function("UpdateCupInfo", cupInfoDict)

    def update_button_info(self, button_info: ButtonInfo):
        if self.ue.is_connected():
            float_bounding_box = [
                float(x) for x in button_info.BoundingBox
            ]  # Convert BoundingBox to list of floats
            r = R.from_euler(
                "xyz",
                [button_info.Pose[3], button_info.Pose[4], button_info.Pose[5]],
                degrees=False,
            )
            qx, qy, qz, qw = r.as_quat()  # Convert Euler angles to quaternion
            buttonInfoDict = {
                "BoundingBox": float_bounding_box,  # Use the converted list of floats
                "Confidence": button_info.Confidence,
                "CanPress": button_info.CanPress,
                "Pose": {
                    "Translation": {
                        "X": button_info.Pose[0],
                        "Y": button_info.Pose[1],
                        "Z": button_info.Pose[2],
                    },
                    "Rotation": {
                        "X": qx,
                        "Y": qy,
                        "Z": qz,
                        "W": qw,
                    },
                    "Scale3D": {
                        "X": 1.0,
                        "Y": 1.0,
                        "Z": 1.0,
                    },
                },
            }
            self.ue.call_function("UpdateButtonInfo", buttonInfoDict)

    def ue_update(self):
        if self.ue.is_connected():
            self.send_system_state_to_ue()
            self.set_ui_joints()

    def publish_connection_status(self):
        msg = Bool()
        msg.data = self.ue.is_connected()
        self.connection_publisher.publish(msg)

    def system_state_callback(self, msg):
        if self._system_state is None or self._system_state != msg.data:
            self.get_logger().debug("Received new system state: " + msg.data)
            self._system_state = msg.data
            self.send_system_state_to_ue()
        self._system_state = msg.data

    def write_data_to_shm(self, index, data):
        if self.use_shared_memory:
            # Write data to shared memory
            self.mapfile.seek(index)
            self.mapfile.write(data.encode("utf-8"))
            self.mapfile.flush()

    def user_input_callback(self, input: str):
        self.get_logger().debug(f"Received user input: {input}")
        # Here you can process the user input and send it to UE if needed
        # For example, you can call a UE function with the user input as parameter
        if input in (item.value for item in UserInputString):
            self.send_user_input(input)

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


def main():
    rclpy.init()
    node = GuiBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
