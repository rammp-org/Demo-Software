import enum
import mmap
import threading
import time
from dataclasses import dataclass

import numpy as np
import posix_ipc
import rclpy
from cmu_door_opener_interfaces.msg import ButtonInfo
from cornell_feeding_interfaces.msg import CupInfo
from gui_interfaces.msg import SystemState
from gui_interfaces.srv import UserInputs

# from realsense2_camera_msgs.msg import Extrinsics
from neu_navigation_interfaces.msg import CurbInfo
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import tf2_ros
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float32

from .streaming.sender import StreamSender
from .unreal_remote_websocket import UnrealRemoteWebsocket
from visualization_msgs.msg import Marker


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
class Extrinsics:
    location: Vector
    rotation: Vector
    scale: Vector


# @dataclass
# class CupInfo:
#     Pose: Transform
#     Success: bool
#     Message: str


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


def _tf_to_ue_extrinsics(transform_stamped) -> "Extrinsics":
    """Convert a geometry_msgs TransformStamped to a UE-compatible Extrinsics.

    Position is converted from meters to centimeters.
    Rotation is converted from quaternion to Euler angles (roll, pitch, yaw) in degrees.
    Uses direct conversion — no axis flips applied.

    # ROS→UE coordinate adjustment:
    # UE Y-axis is flipped from ROS, so negate y_cm.
    """
    t = transform_stamped.transform.translation
    r = transform_stamped.transform.rotation

    rot = R.from_quat([r.x, r.y, r.z, r.w])
    roll_deg, pitch_deg, yaw_deg = rotation_matrix_to_euler_zyx(rot.as_matrix())

    x_cm = t.x * 100.0
    y_cm = -t.y * 100.0
    z_cm = t.z * 100.0

    # Mirror rotation to match Y-axis flip
    pitch_deg = -pitch_deg
    yaw_deg = -yaw_deg

    return Extrinsics(
        location=Vector(x_cm, y_cm, z_cm),
        rotation=Vector(roll_deg, pitch_deg, yaw_deg),
        scale=Vector(1.0, 1.0, 1.0),
    )


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
        # TF frame names for camera extrinsics lookup
        self.declare_parameter("tf_base_frame", "mebot")
        self.tf_base_frame = (
            self.get_parameter("tf_base_frame").get_parameter_value().string_value
        )
        self.declare_parameter("wrist_camera_tf_frame", "wrist_color_frame")
        self.wrist_camera_tf_frame = (
            self.get_parameter("wrist_camera_tf_frame")
            .get_parameter_value()
            .string_value
        )
        self.declare_parameter("nav_camera_1_tf_frame", "nav1_link")
        self.nav_camera_1_tf_frame = (
            self.get_parameter("nav_camera_1_tf_frame")
            .get_parameter_value()
            .string_value
        )
        self.declare_parameter("nav_camera_2_tf_frame", "nav2_link")
        self.nav_camera_2_tf_frame = (
            self.get_parameter("nav_camera_2_tf_frame")
            .get_parameter_value()
            .string_value
        )

        self._trim_prefixes = {
            self.wrist_camera_tf_frame: "wrist",
            self.nav_camera_1_tf_frame: "nav1",
            self.nav_camera_2_tf_frame: "nav2",
        }
        # Per-camera extrinsic trim offsets (cm for position, degrees for rotation).
        # Values captured from TrimTUI calibration session.
        _trim_defaults = {
            # wrist
            ("wrist", "x"): 10.0,
            ("wrist", "y"): -3.0,
            ("wrist", "z"): 2.0,
            # nav1 (mounted sideways → roll -90°)
            ("nav1", "roll"): -90.0,
            ("nav1", "z"): -15.24,
            # nav2
            ("nav2", "x"): -1.0,
            ("nav2", "y"): 1.0,
            ("nav1", "z"): -15.24,
            ("nav2", "pitch"): -2.0,
        }
        for cam in ["wrist", "nav1", "nav2"]:
            for axis in ["x", "y", "z", "roll", "pitch", "yaw"]:
                self.declare_parameter(
                    f"{cam}_trim_{axis}",
                    _trim_defaults.get((cam, axis), 0.0),
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

        self.stream_lock = threading.Lock()
        self.ue = UnrealRemoteWebsocket(
            host=self.host,
            preset=self.ue_preset,
            user_input_callback=self.user_input_callback,
        )
        self.stream_sender = StreamSender(host=self.host, port=30030, queue_size=10)
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=True
        )
        self._last_extrinsics: dict[str, "Extrinsics"] = {}

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
        # make subscriber for system state, message should be SystemState
        self.system_state_subscriber = self.create_subscription(
            SystemState,
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
        self.curb_traverse_progress_subscriber = self.create_subscription(
            Float32,
            "/nav/curb_traverse_progress",
            self.curb_traverse_progress_callback,
            10,
            callback_group=self._cb_group,
        )
        self.cup_info_subscriber = self.create_subscription(
            CupInfo,
            "/arm/drink/cup_info",
            self.cup_info_callback,
            10,
            callback_group=self._cb_group,
        )

        # init camera subscribers
        # init camera msg to None first.
        self.wrist_camera_image_info = None
        self.wrist_camera_depth_info = None
        self.wrist_camera_image = None
        self.wrist_camera_depth = None
        self.wrist_camera_extrinsics = Extrinsics(
            location=Vector(0, 0, 0), rotation=Vector(0, 0, 0), scale=Vector(1, 1, 1)
        )
        self.nav_camera_1_image_info = None
        self.nav_camera_1_depth_info = None
        self.nav_camera_1_image = None
        self.nav_camera_1_depth = None
        self.nav_camera_1_extrinsics = Extrinsics(
            location=Vector(0, 0, 0), rotation=Vector(0, 0, 0), scale=Vector(1, 1, 1)
        )
        self.nav_camera_2_image_info = None
        self.nav_camera_2_depth_info = None
        self.nav_camera_2_image = None
        self.nav_camera_2_depth = None
        self.nav_camera_2_extrinsics = Extrinsics(
            location=Vector(0, 0, 0), rotation=Vector(0, 0, 0), scale=Vector(1, 1, 1)
        )
        self.rear_camera_image_info = None
        self.rear_camera_depth_info = None
        self.rear_camera_image = None
        self.rear_camera_depth = None
        self.rear_camera_extrinsics = Extrinsics(
            location=Vector(0, 0, 0), rotation=Vector(0, 0, 0), scale=Vector(1, 1, 1)
        )
        # wrist camera:
        self.wrist_camera_image_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.wrist_camera_namespace}/color/camera_info",
            self.wrist_camera_image_info_callback,
            0,
            callback_group=self._cb_group,
        )
        self.wrist_camera_depth_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.wrist_camera_namespace}/depth/camera_info",
            self.wrist_camera_depth_info_callback,
            0,
            callback_group=self._cb_group,
        )
        self.wrist_camera_image_sub = self.create_subscription(
            Image,
            f"{self.wrist_camera_namespace}/color/image_raw",
            self.wrist_camera_image_callback,
            0,
            callback_group=self._cb_group,
        )
        self.wrist_camera_depth_sub = self.create_subscription(
            Image,
            f"{self.wrist_camera_namespace}/depth/image_rect_raw",
            self.wrist_camera_depth_callback,
            0,
            callback_group=self._cb_group,
        )
        # nav camera 1
        self.nav_camera_1_image_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.nav_camera_namespace_1}/color/camera_info_rotated",
            self.nav_camera_1_image_info_callback,
            0,
            callback_group=self._cb_group,
        )
        self.nav_camera_1_depth_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.nav_camera_namespace_1}/depth/camera_info_rotated",
            self.nav_camera_1_depth_info_callback,
            0,
            callback_group=self._cb_group,
        )
        self.nav_camera_1_image_sub = self.create_subscription(
            Image,
            f"{self.nav_camera_namespace_1}/color/image_rotated",
            self.nav_camera_1_image_callback,
            0,
            callback_group=self._cb_group,
        )
        self.nav_camera_1_depth_sub = self.create_subscription(
            Image,
            f"{self.nav_camera_namespace_1}/depth/image_rotated",
            self.nav_camera_1_depth_callback,
            0,
            callback_group=self._cb_group,
        )
        # nav camera 2
        self.nav_camera_2_image_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.nav_camera_namespace_2}/color/camera_info",
            self.nav_camera_2_image_info_callback,
            0,
            callback_group=self._cb_group,
        )
        self.nav_camera_2_depth_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.nav_camera_namespace_2}/depth/camera_info",
            self.nav_camera_2_depth_info_callback,
            0,
            callback_group=self._cb_group,
        )
        self.nav_camera_2_image_sub = self.create_subscription(
            Image,
            f"{self.nav_camera_namespace_2}/color/image_raw",
            self.nav_camera_2_image_callback,
            0,
            callback_group=self._cb_group,
        )
        self.nav_camera_2_depth_sub = self.create_subscription(
            Image,
            f"{self.nav_camera_namespace_2}/depth/image_raw",
            self.nav_camera_2_depth_callback,
            0,
            callback_group=self._cb_group,
        )
        # rear camera
        self.rear_camera_image_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.rear_camera_namespace}/color/camera_info",
            self.rear_camera_image_info_callback,
            0,
            callback_group=self._cb_group,
        )
        self.rear_camera_depth_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.rear_camera_namespace}/depth/camera_info",
            self.rear_camera_depth_info_callback,
            0,
            callback_group=self._cb_group,
        )
        self.rear_camera_image_sub = self.create_subscription(
            Image,
            f"{self.rear_camera_namespace}/color/image_raw",
            self.rear_camera_image_callback,
            0,
            callback_group=self._cb_group,
        )
        self.rear_camera_depth_sub = self.create_subscription(
            Image,
            f"{self.rear_camera_namespace}/depth/image_raw",
            self.rear_camera_depth_callback,
            0,
            callback_group=self._cb_group,
        )
        # curb info
        self.curb_info_sub = self.create_subscription(
            CurbInfo,
            "/nav/curb/info",
            self.curb_info_callback,
            0,
            callback_group=self._cb_group,
        )
        # curb mask
        self.curb_mask_sub = self.create_subscription(
            Image,
            "/perception/curb_mask",
            self.curb_mask_callback,
            0,
            callback_group=self._cb_group,
        )
        # curb marker
        self._curb_marker = None
        self.curb_marker_sub = self.create_subscription(
            Marker,
            "/perception/curb_visual",
            self.curb_marker_callback,
            0,
            callback_group=self._cb_group,
        )

        # door button info
        self.door_button_info_sub = self.create_subscription(
            ButtonInfo,
            "/arm/door/button_info",
            self.door_button_info_callback,
            0,
            callback_group=self._cb_group,
        )

    def curb_marker_callback(self, msg: Marker):
        self._curb_marker = msg

    def curb_traverse_progress_callback(self, msg: Float32):
        self.get_logger().info(f"Curb traverse progress: {msg.data:.2f}%")
        self.send_curb_traverse_progress_to_ue(msg.data)

    def cup_info_callback(self, msg: CupInfo):
        self.update_cup_info(msg)
        self.send_mask(
            msg.segmentation_mask,
            self.wrist_camera_namespace,
            channel=self.mask_channel,
            num_seg_ids=3.0,
        )  # for now just send the segmentation mask of the cup, can also send cup pose to UE if needed

    def door_button_info_callback(self, msg: ButtonInfo):
        self.update_button_info(msg)
        self.send_mask(
            msg.segmentation_mask,
            self.wrist_camera_namespace,
            channel=self.mask_channel,
            num_seg_ids=2.0,
        )

    def curb_mask_callback(self, msg: Image):
        self.send_mask(
            msg,
            self.nav_camera_namespace_1,
            channel=self.mask_channel + 1,
            num_seg_ids=4.0,
        )

    def curb_info_callback(self, msg: CurbInfo):
        self.update_curb_info(msg, marker=self._curb_marker)

    # camera subscription callbacks
    def wrist_camera_image_info_callback(self, msg: CameraInfo):
        self.wrist_camera_image_info = msg

    def wrist_camera_depth_info_callback(self, msg: CameraInfo):
        self.wrist_camera_depth_info = msg

    def wrist_camera_image_callback(self, msg: Image):
        self.wrist_camera_image = msg
        self.wrist_camera_extrinsics = self._lookup_camera_extrinsics(
            self.wrist_camera_tf_frame
        )
        self.send_wrist_camera_image()

    def wrist_camera_depth_callback(self, msg: Image):
        self.wrist_camera_depth = msg
        self.wrist_camera_extrinsics = self._lookup_camera_extrinsics(
            self.wrist_camera_tf_frame
        )
        self.send_wrist_camera_depth()

    def nav_camera_1_image_info_callback(self, msg: CameraInfo):
        self.nav_camera_1_image_info = msg

    def nav_camera_1_depth_info_callback(self, msg: CameraInfo):
        self.nav_camera_1_depth_info = msg

    def nav_camera_1_image_callback(self, msg: Image):
        self.nav_camera_1_image = msg
        self.nav_camera_1_extrinsics = self._lookup_camera_extrinsics(
            self.nav_camera_1_tf_frame
        )
        self.send_nav_camera_1_image()

    def nav_camera_1_depth_callback(self, msg: Image):
        self.nav_camera_1_depth = msg
        self.nav_camera_1_extrinsics = self._lookup_camera_extrinsics(
            self.nav_camera_1_tf_frame
        )
        self.send_nav_camera_1_depth()

    def nav_camera_2_image_info_callback(self, msg: CameraInfo):
        self.nav_camera_2_image_info = msg

    def nav_camera_2_depth_info_callback(self, msg: CameraInfo):
        self.nav_camera_2_depth_info = msg

    def nav_camera_2_image_callback(self, msg: Image):
        self.nav_camera_2_image = msg
        self.nav_camera_2_extrinsics = self._lookup_camera_extrinsics(
            self.nav_camera_2_tf_frame
        )
        self.send_nav_camera_2_image()

    def nav_camera_2_depth_callback(self, msg: Image):
        self.nav_camera_2_depth = msg
        self.nav_camera_2_extrinsics = self._lookup_camera_extrinsics(
            self.nav_camera_2_tf_frame
        )
        self.send_nav_camera_2_depth()

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

    def _lookup_camera_extrinsics(self, camera_tf_frame: str) -> Extrinsics:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_base_frame,
                camera_tf_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            extrinsics = _tf_to_ue_extrinsics(tf)
            self._last_extrinsics[camera_tf_frame] = extrinsics
        except Exception as e:
            self.get_logger().warn(
                f"TF2 lookup failed for {self.tf_base_frame} → {camera_tf_frame}: {e}",
                throttle_duration_sec=5.0,
            )
            extrinsics = self._last_extrinsics.get(
                camera_tf_frame,
                Extrinsics(
                    location=Vector(0, 0, 0),
                    rotation=Vector(0, 0, 0),
                    scale=Vector(1, 1, 1),
                ),
            )

        prefix = self._trim_prefixes.get(camera_tf_frame)
        if prefix:
            extrinsics = Extrinsics(
                location=Vector(
                    extrinsics.location.X
                    + self.get_parameter(f"{prefix}_trim_x").value,
                    extrinsics.location.Y
                    + self.get_parameter(f"{prefix}_trim_y").value,
                    extrinsics.location.Z
                    + self.get_parameter(f"{prefix}_trim_z").value,
                ),
                rotation=Vector(
                    extrinsics.rotation.X
                    + self.get_parameter(f"{prefix}_trim_roll").value,
                    extrinsics.rotation.Y
                    + self.get_parameter(f"{prefix}_trim_pitch").value,
                    extrinsics.rotation.Z
                    + self.get_parameter(f"{prefix}_trim_yaw").value,
                ),
                scale=Vector(1.0, 1.0, 1.0),
            )

        self.get_logger().debug(
            f"TF2 [{camera_tf_frame}]: "
            f"pos=({extrinsics.location.X:.1f}, {extrinsics.location.Y:.1f}, {extrinsics.location.Z:.1f})cm "
            f"rot=({extrinsics.rotation.X:.1f}, {extrinsics.rotation.Y:.1f}, {extrinsics.rotation.Z:.1f})deg",
            throttle_duration_sec=1.0,
        )
        return extrinsics

    def send_image(
        self,
        image: Image,
        image_info: CameraInfo,
        source: str,
        extrinsics: Extrinsics = None,
        channel: int = 0,
    ):
        if self.stream_sender.is_connected():
            if image is not None and image_info is not None:
                try:
                    width = image.width
                    height = image.height
                    fx = image_info.k[0]
                    fy = image_info.k[4]
                    cx = image_info.k[2]
                    cy = image_info.k[5]
                    meta = {
                        "w": width,
                        "h": height,
                        "fmt": image.encoding,
                        "group": source,
                        "source": source,
                        "role": "color",
                        "stream_id": f"{source}/color",
                    }
                    meta["intrinsics"] = {
                        "fx": fx,
                        "fy": fy,
                        "cx": cx,
                        "cy": cy,
                    }
                    if extrinsics is not None:
                        meta["transform"] = {
                            "x": extrinsics.location.X,
                            "y": extrinsics.location.Y,
                            "z": extrinsics.location.Z,
                            "pitch": extrinsics.rotation.Y,
                            "roll": extrinsics.rotation.X,
                            "yaw": extrinsics.rotation.Z,
                        }
                        meta["transform_space"] = "relative"
                    with self.stream_lock:
                        self.stream_sender.send_image(
                            channel=channel,
                            image_bytes=image.data.tobytes(),
                            width=width,
                            height=height,
                            metadata=meta,
                        )
                except Exception as e:
                    self.get_logger().warn(f"Failed to send {source} image: {e}")

    def send_depth(
        self,
        depth: Image,
        depth_info: CameraInfo,
        source: str,
        extrinsics: Extrinsics = None,
        channel: int = 100,
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
                        "group": source,
                        "role": "depth",
                        "stream_id": f"{source}/depth",
                    }
                    meta["intrinsics"] = {
                        "fx": fx,
                        "fy": fy,
                        "cx": cx,
                        "cy": cy,
                    }
                    if extrinsics is not None:
                        meta["transform"] = {
                            "x": extrinsics.location.X,
                            "y": extrinsics.location.Y,
                            "z": extrinsics.location.Z,
                            "pitch": extrinsics.rotation.Y,
                            "roll": extrinsics.rotation.X,
                            "yaw": extrinsics.rotation.Z,
                        }
                        meta["transform_space"] = "relative"
                    with self.stream_lock:
                        self.stream_sender.send_depth_uint16(
                            channel=channel,
                            depth_bytes=depth.data.tobytes(),
                            width=width,
                            height=height,
                            metadata=meta,
                        )
                except Exception as e:
                    self.get_logger().warn(f"Failed to send {source} depth image: {e}")

    def send_mask(
        self, mask: Image, source: str, channel: int = 200, num_seg_ids: float = 4.0
    ):
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
                        "group": source,
                        "role": "mask",
                        "stream_id": f"{source}/mask",
                    }
                    material_params = {
                        "NumSegmentIDs": num_seg_ids,
                        "MaskScale": 255.0,
                    }
                    with self.stream_lock:
                        self.stream_sender.send_mask(
                            channel=channel,
                            mask_bytes=mask.data.tobytes(),
                            width=width,
                            height=height,
                            metadata=meta,
                            material_params=material_params,
                        )
                except Exception as e:
                    self.get_logger().warn(f"Failed to send {source} mask image: {e}")

    def send_wrist_camera_image(self):
        self.send_image(
            image=self.wrist_camera_image,
            image_info=self.wrist_camera_image_info,
            source=self.wrist_camera_namespace,
            extrinsics=self.wrist_camera_extrinsics,
            channel=self.image_channel,
        )

    def send_wrist_camera_depth(self):
        self.send_depth(
            depth=self.wrist_camera_depth,
            depth_info=self.wrist_camera_depth_info,
            extrinsics=self.wrist_camera_extrinsics,
            source=self.wrist_camera_namespace,
            channel=self.depth_channel,
        )

    def send_nav_camera_1_image(self):
        self.send_image(
            image=self.nav_camera_1_image,
            image_info=self.nav_camera_1_image_info,
            source=self.nav_camera_namespace_1,
            extrinsics=self.nav_camera_1_extrinsics,
            channel=self.image_channel + 1,
        )

    def send_nav_camera_1_depth(self):
        self.send_depth(
            depth=self.nav_camera_1_depth,
            depth_info=self.nav_camera_1_depth_info,
            extrinsics=self.nav_camera_1_extrinsics,
            source=self.nav_camera_namespace_1,
            channel=self.depth_channel + 1,
        )

    def send_nav_camera_2_image(self):
        self.send_image(
            image=self.nav_camera_2_image,
            image_info=self.nav_camera_2_image_info,
            source=self.nav_camera_namespace_2,
            extrinsics=self.nav_camera_2_extrinsics,
            channel=self.image_channel + 2,
        )

    def send_nav_camera_2_depth(self):
        self.send_depth(
            depth=self.nav_camera_2_depth,
            depth_info=self.nav_camera_2_depth_info,
            extrinsics=self.nav_camera_2_extrinsics,
            source=self.nav_camera_namespace_2,
            channel=self.depth_channel + 2,
        )

    def send_rear_camera_image(self):
        self.send_image(
            image=self.rear_camera_image,
            image_info=self.rear_camera_image_info,
            source=self.rear_camera_namespace,
            extrinsics=self.rear_camera_extrinsics,
            channel=self.image_channel + 3,
        )

    def send_rear_camera_depth(self):
        self.send_depth(
            depth=self.rear_camera_depth,
            depth_info=self.rear_camera_depth_info,
            extrinsics=self.rear_camera_extrinsics,
            source=self.rear_camera_namespace,
            channel=self.depth_channel + 3,
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
                # print("Testing sending curb info to UE...")
                # curbInfo = CurbInfo(
                #     Distance=0.5,
                #     Height=0.2,
                #     Pose=Transform(
                #         Translation=Vector(X=1.0, Y=2.0, Z=3.0),
                #         Rotation=Quaternion(X=0.0, Y=0.0, Z=0.0, W=1.0),
                #         Scale3D=Vector(X=1.0, Y=1.0, Z=1.0),
                #     ),
                #     Success=True,
                #     Message="Curb info updated successfully",
                # )
                # # print("CurbInfo to send:", asdict(curbInfo))

                # self.update_curb_info(curbInfo)
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
        arm_joints = self.arm_joints
        base_joints = self.base_joints
        if arm_joints is not None and len(arm_joints.position) >= 8:
            for i in range(7):  # Only take the first 7 joints for the arm joint
                arr[arm_joint_index_offset + i] = (
                    arm_joints.position[i] * 180.0 / 3.14159
                )  # Convert radians to degrees
            arr[arm_joint_index_offset + 0] = -arr[arm_joint_index_offset + 0]
            arr[arm_joint_index_offset + 2] = -arr[arm_joint_index_offset + 2]
            arr[arm_joint_index_offset + 4] = -arr[arm_joint_index_offset + 4]
            arr[arm_joint_index_offset + 5] = -arr[arm_joint_index_offset + 5]
            arr[arm_joint_index_offset + 6] = -arr[arm_joint_index_offset + 6]
            # the 8th joint is the gripper, 1 is close and 0 is open
            # set gripper joint angle to 0 or 45 for close and open
            arr[arm_gripper_joint_left_index] = (
                0.0 if arm_joints.position[7] > 0.9 else -45.0
            )
            arr[arm_gripper_joint_right_index] = (
                0.0 if arm_joints.position[7] > 0.9 else 45.0
            )
        if base_joints is not None and len(base_joints.position) >= 8:
            arr[chair_fc_joint_index] = base_joints.position[0] * 180.0 / 3.14159
            arr[chair_rc_joint_index] = base_joints.position[1] * 180.0 / 3.14159
            arr[chair_mr_joint_index] = base_joints.position[2] * 180.0 / 3.14159
            arr[chair_ml_joint_index] = base_joints.position[3] * 180.0 / 3.14159
            arr[chair_ml_carriage_index] = base_joints.position[4] * 100.0  # to cm
            arr[chair_mr_carriage_index] = base_joints.position[5] * 100.0  # to cm
            arr[chair_ml_wheel_index] = base_joints.position[6] * 180.0 / 3.14159
            arr[chair_mr_wheel_index] = base_joints.position[7] * 180.0 / 3.14159

        if base_joints is not None or arm_joints is not None:
            self.ue.call_function("setJoints", {"Values": arr})

    def send_system_state_to_ue(self):
        if self.ue.is_connected() and self._system_state is not None:
            self.ue.call_function(
                "UpdateSystemState", {"SystemState": str(self._system_state.state)}
            )
            self.ue.call_function(
                "UpdateAllowedCommands",
                {"CommandList": self._system_state.supported_user_inputs},
            )

    def send_curb_traverse_progress_to_ue(self, progress: float):
        if self.ue.is_connected():
            self.ue.call_function("UpdateCurbTraversalProgress", {"Progress": progress})

    def update_curb_info(self, curb_info: CurbInfo, marker: Marker = None):
        if self.ue.is_connected():
            success = curb_info.success if marker is not None else False
            curbInfoDict = {
                "Success": success,
                "Orientation": curb_info.orientation
                * 180.0
                / 3.14159,  # convert to degree
                "Distance": curb_info.distance * 100.0,  # convert meter to cm
                "Height": curb_info.height * 100.0,  # convert meter to cm
                "NumSegmentIDs": 4,
            }
            if marker is not None:
                curbInfoDict["Pose"] = {
                    "Translation": {
                        "X": marker.pose.position.x * 100.0,  # convert meter to cm
                        "Y": -marker.pose.position.y
                        * 100.0,  # convert meter to cm and change to UE coordinate
                        "Z": marker.pose.position.z * 100.0,  # convert meter to cm
                    },
                    "Rotation": {
                        "X": marker.pose.orientation.x,
                        "Y": marker.pose.orientation.y,
                        "Z": marker.pose.orientation.z,
                        "W": marker.pose.orientation.w,
                    },
                    "Scale3D": {
                        "X": marker.scale.y,  # flip x and y because of the difference between ROS and UE coordinate system
                        "Y": marker.scale.x,
                        "Z": marker.scale.z,
                    },
                }
            self.ue.call_function("UpdateCurbInfo", curbInfoDict)

    def update_cup_info(self, cup_info: CupInfo):
        if self.ue.is_connected():
            width = (
                cup_info.segmentation_mask.width
                if cup_info.segmentation_mask.width > 0
                else 640
            )
            height = (
                cup_info.segmentation_mask.height
                if cup_info.segmentation_mask.height > 0
                else 480
            )
            float_bounding_box = [
                float(x) for x in cup_info.bounding_box
            ]  # Convert BoundingBox to list of floats
            float_bounding_box[0] /= width  # Normalize x
            float_bounding_box[1] /= height  # Normalize y
            float_bounding_box[2] /= width  # Normalize width
            float_bounding_box[3] /= height  # Normalize height
            cupInfoDict = {
                "BoundingBox": float_bounding_box,  # Use the converted list of floats
                "Success": cup_info.success,
                "Pose": {
                    "Translation": {
                        "X": cup_info.pose[0],
                        "Y": cup_info.pose[1],
                        "Z": cup_info.pose[2],
                    },
                    "Rotation": {
                        "X": cup_info.pose[3],
                        "Y": cup_info.pose[4],
                        "Z": cup_info.pose[5],
                        "W": cup_info.pose[6],
                    },
                    "Scale3D": {
                        "X": 1.0,
                        "Y": 1.0,
                        "Z": 1.0,
                    },
                },
                "NumSegmentIDs": 3,
            }
            self.ue.call_function("UpdateCupInfo", cupInfoDict)

    def update_button_info(self, button_info: ButtonInfo):
        if self.ue.is_connected():
            width = (
                button_info.segmentation_mask.width
                if button_info.segmentation_mask.width > 0
                else 640
            )
            height = (
                button_info.segmentation_mask.height
                if button_info.segmentation_mask.height > 0
                else 480
            )
            float_bounding_box = [
                float(x) for x in button_info.bounding_box
            ]  # Convert BoundingBox to list of floats
            float_bounding_box[0] /= width  # Normalize x
            float_bounding_box[1] /= height  # Normalize y
            float_bounding_box[2] /= width  # Normalize width
            float_bounding_box[3] /= height  # Normalize height
            r = R.from_euler(
                "xyz",
                [
                    button_info.pose_xyzrpy[3],
                    button_info.pose_xyzrpy[4],
                    button_info.pose_xyzrpy[5],
                ],
                degrees=False,
            )
            qx, qy, qz, qw = r.as_quat()  # Convert Euler angles to quaternion
            buttonInfoDict = {
                "BoundingBox": float_bounding_box,  # Use the converted list of floats
                "Confidence": button_info.confidence,
                "CanPress?": button_info.is_pressable,
                "Pose": {
                    "Translation": {
                        "X": button_info.pose_xyzrpy[0],
                        "Y": button_info.pose_xyzrpy[1],
                        "Z": button_info.pose_xyzrpy[2],
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
                "NumSegmentIDs": 2,
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
        if self._system_state is None or self._system_state.state != msg.state:
            self.get_logger().info(
                "Received new system state: "
                + (msg.state)
                + ", supported user input: "
                + str(msg.supported_user_inputs)
            )
            self._system_state = msg
            self.send_system_state_to_ue()
        self._system_state = msg

    def write_data_to_shm(self, index, data):
        if self.use_shared_memory:
            # Write data to shared memory
            self.mapfile.seek(index)
            self.mapfile.write(data.encode("utf-8"))
            self.mapfile.flush()

    def user_input_callback(self, input: str):
        self.get_logger().info(f"Received user input: {input}")
        # Here you can process the user input and send it to UE if needed
        # For example, you can call a UE function with the user input as parameter
        if input in (item.value for item in UserInputString):
            self.send_user_input(input)

    def destroy_node(self):
        # Signal websocket handlers to stop
        self.ue.shutdown()
        # Stop the event loop
        ue_loop = getattr(self.ue, "loop", None)
        if ue_loop is not None:
            ue_loop.call_soon_threadsafe(ue_loop.stop)
            # Wait for the event loop thread to finish
            timeout = 5  # seconds
            start = time.time()
            while ue_loop.is_running() and (time.time() - start) < timeout:
                time.sleep(0.1)
        if self.use_shared_memory:
            self.mapfile.close()
            self.shm.unlink()
        super().destroy_node()


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = GuiBridge()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
