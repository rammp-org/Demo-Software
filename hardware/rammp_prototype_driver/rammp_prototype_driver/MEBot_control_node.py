import math
import time
from enum import IntEnum
from std_srvs.srv import Empty
from rclpy.executors import MultiThreadedExecutor
import json
from ament_index_python.packages import get_package_share_directory
import rclpy
import serial
from rammp_prototype_interfaces.action import CurbTraverse
from rammp_prototype_interfaces.action import Calibration
from rammp_prototype_interfaces.msg import SeatCommand
from luci_messages.msg import LuciJoystick

from rammp_prototype_interfaces.msg import RAMMPPrototypeState
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

from .keyframe import Keyframe, NUM_MOTORS
from .protocol import ProtocolEncoder

# LUCI STUFF
JS_FRONT = 0
JS_FRONT_LEFT = 1
JS_FRONT_RIGHT = 2
JS_LEFT = 3
JS_RIGHT = 4
JS_BACK_LEFT = 5
JS_BACK_RIGHT = 6
JS_BACK = 7
JS_ORIGIN = 8

INPUT_REMOTE = 1

JOYSTICK_TOPIC = "/luci/remote_joystick"
JOYSTICK_MSG_TYPE = "/luci_messages/msg/LuciJoystick"
SET_AUTO_SERVICE = "/luci/set_auto_remote_input"
REMOVE_AUTO_SERVICE = "/luci/remove_auto_remote_input"


def _compute_zone(fb: int, lr: int) -> int:
    if fb == 0 and lr == 0:
        return JS_ORIGIN
    if fb > 0 and lr == 0:
        return JS_FRONT
    if fb < 0 and lr == 0:
        return JS_BACK
    if fb == 0 and lr > 0:
        return JS_RIGHT
    if fb == 0 and lr < 0:
        return JS_LEFT
    if fb > 0 and lr > 0:
        return JS_FRONT_RIGHT
    if fb > 0 and lr < 0:
        return JS_FRONT_LEFT
    if fb < 0 and lr > 0:
        return JS_BACK_RIGHT
    return JS_BACK_LEFT


# Keyframe stuff

SEAT_MOVE_DURATION_MS = 1000
CALIBRATION_PWM = -0.2

SEAT_DELTAS: dict[int, list[float]] = {
    SeatCommand.RAISE: [70.0, 0.0, 40.0, 40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.LOWER: [-70.0, 0.0, -40.0, -40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.TILT_FWD: [0.0, 0.0, -40.0, -40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.TILT_BACK: [0.0, 0.0, 40.0, 40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.LATERAL_LEFT: [0.0, 0.0, -30.0, 30.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.LATERAL_RIGHT: [0.0, 0.0, 30.0, -30.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.RESET: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}


def _load_keyframes_from_json(json_path: str) -> list[Keyframe]:
    with open(json_path, "r") as f:
        data = json.load(f)
    keyframes_data = data.get("keyframes", data if isinstance(data, list) else [])
    return [Keyframe.from_dict(d) for d in keyframes_data]


def _build_seat_keyframe(deltas: list[float], duration_ms: int, command) -> Keyframe:
    kf = Keyframe()
    kf.targets = list(deltas)
    kf.duration_ms = duration_ms

    if command == SeatCommand.RESET:
        kf.relative = [False] * NUM_MOTORS
        kf.relative[6] = True
        kf.relative[7] = True
    else:
        kf.relative = [True] * NUM_MOTORS

    return kf


class SerialField(IntEnum):
    """Named indices for the serial data list sent by the Teensy.

    The Teensy sends sensor data as a Python list string (e.g. '[0.5, 1.2, ...]'),
    which is parsed with ast.literal_eval into a list. Each enum member's integer
    value is the index of that field in the list, matching the order the Teensy
    populates its output array in Base.ino.

    Usage:
        data = ast.literal_eval(raw)
        pitch = data[SerialField.IMU_PITCH]  # instead of data[0]

    If the Teensy serial protocol changes field order, update the values here.
    """

    #  Indices here should match the order of data sent from the Teensy in Base.ino's serial output array
    IMU_PITCH = 37
    IMU_ROLL = 38
    IMU_YAW = 39
    ACCEL_X = 40
    ACCEL_Y = 41
    ACCEL_Z = 42
    IMU_QW = 43
    IMU_QX = 44
    IMU_QY = 45
    IMU_QZ = 46
    FC_POS = 4
    RC_POS = 3
    MR_POS = 6
    ML_POS = 5
    ML_CARRIAGE_POS = 7
    MR_CARRIAGE_POS = 8
    # ML_WHEEL_POS = 0      #TODO: add this index
    # MR_WHEEL_POS = 0      #TODO: add this index
    FC_LOADCELL = 53
    RC_LOADCELL = 52
    MR_LOADCELL = 55
    ML_LOADCELL = 54
    # APP_TIME = 0
    # SPEED_ML = 0      #TODO: add this index
    # SPEED_MR = 0      #TODO: add this index
    STATE = 2
    FB_PWM = 66


class MEBotControlNode(Node):
    def __init__(self):
        super().__init__("MEBot_control_node")

        # serial init
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )

        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=115200,
                timeout=1,
            )
        except serial.SerialException:
            self.ser = None

        # diagnostics updater init
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("MEBot")
        self.updater.add("LUCI status", self.check_luci_node)
        self.updater.add("Teensy status", self.check_teensy_connection)
        self.updater.add("Teensy status", self.check_teensy_state)

        # Data transfer rates
        # Rate to read data from serial
        self.serial_rate = 1 / 1000.0
        # Rate to publish joint states
        self.joint_state_rate = 1 / 100.0
        # Rate to publish RAMMPPrototypeState
        self.state_publish_rate = 1 / 100.0
        # Diagnostic publish rate
        self.diagnostic_publish_rate = 1 / 1.0

        # timer for serial data reading
        self.serial_timer = self.create_timer(self.serial_rate, self.read_serial_data)

        # heartbeat to send serial message from jetson ->teensy to prevent teensy from timing out
        self.serial_timer = self.create_timer(
            self.serial_rate, self.send_serial_heartbeat
        )

        # Fields to store sequence player data
        self.current_seq = 0
        self.seq_length = 0
        self.seq_mode = 0  # 1= interpolating, 2 = settling, 0 = idle/complete

        ### Fields to store incoming data from serial for publishing in ROS messages
        # IMU
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_yaw = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.imu_qw = 0.0
        self.imu_qx = 0.0
        self.imu_qy = 0.0
        self.imu_qz = 0.0

        # Encoders
        self.FC_pos = 0.0
        self.RC_pos = 0.0
        self.MR_pos = 0.0
        self.ML_pos = 0.0
        self.ML_carriage_pos = 0.0
        self.MR_carriage_pos = 0.0
        self.ML_wheel_pos = 0.0
        self.MR_wheel_pos = 0.0

        # Loadcells
        self.RC_loadcell = 0.0
        self.FC_loadcell = 0.0
        self.MR_loadcell = 0.0
        self.ML_loadcell = 0.0

        # state
        self.state = RAMMPPrototypeState.STATE_IDLE

        # app time
        self.app_time = 0.0

        # velocity and acceleration
        self.prev_speed_ML = 0.0
        self.current_speed_ML = 0.0

        self.prev_speed_MR = 0.0
        self.current_speed_MR = 0.0

        self.cal_joints_done = 0
        self.cal_complete = False

        # drive wheel velocities
        self.fb_pwm = 0
        self.test_pwm = 0

        #### Init all ROS interfaces
        self._init_services()
        self._init_actions()
        self._init_subscribers()
        self._init_publishers()

    def _init_services(self):
        # services
        self.drive_enable_service = self.create_service(
            SetBool, "drive_enable", self.drive_enable_callback
        )

        self.self_level_enable_service = self.create_service(
            SetBool, "self_level_enable", self.self_level_enable_callback
        )

        # LUCI service clients
        self.set_auto_remote_client = self.create_client(
            Empty, "/luci/set_auto_remote_input"
        )
        self.remove_auto_remote_client = self.create_client(
            Empty, "/luci/remove_auto_remote_input"
        )

    def _init_actions(self):
        # actions
        self.curb_traverse_action = ActionServer(
            self, CurbTraverse, "curb_traverse", self.curb_traverse_action_callback
        )

        self.calibrate_action = ActionServer(
            self, Calibration, "calibrate", self.calibrate_motors_callback
        )

    def _init_subscribers(self):
        # subscriptions
        self.manual_seat_control_subscription = self.create_subscription(
            SeatCommand, "manual_seat_control", self.manual_seat_control_callback, 10
        )

        self.estop_subscription = self.create_subscription(
            Bool, "estop", self.estop_callback, 10
        )

    def _init_publishers(self):
        # joint state publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        self.joint_state_timer = self.create_timer(
            self.joint_state_rate, self.publish_joint_states
        )

        # state publisher
        self.RAMMPPrototypeState_publisher = self.create_publisher(
            RAMMPPrototypeState, "rammp_prototype_state", 10
        )
        self.RAMMPPrototypeState_timer = self.create_timer(
            self.state_publish_rate, self.publish_RAMMPPrototypeState
        )

        self.luci_js_publisher = self.create_publisher(LuciJoystick, JOYSTICK_TOPIC, 10)

        self.luci_heartbeat_timer = self.create_timer(0.005, self._send_joystick)

        # self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        # self.imu_timer = self.create_timer(self.publish_rate, self.publish_imu_data)

    # reading incoming serial data from teensy

    # TODO: Add checks for serial corruption
    def read_serial_data(self):
        if self.ser is None:
            return
        if self.ser.in_waiting > 0:
            line = self.ser.readline()
            if line:
                raw_data = line.decode("utf-8", errors="replace").strip()
                if raw_data.startswith("TELEMETRY"):
                    # self.get_logger().info(raw_data)
                    data = raw_data.split(",")  # All values are str
                    # self.get_logger().info(str(data))
                    self.update_data(data)  # Update variables with new data
                if raw_data.startswith(
                    "SEQ_STATUS"
                ):  # parsing and storing information about sequence player if running
                    split_data = raw_data.split(",")
                    self.current_seq = int(split_data[1])
                    self.seq_length = int(split_data[2])
                    self.seq_mode = int(split_data[3].strip())
                if raw_data.startswith("CAL: Homed"):
                    self.cal_joints_done += 1
                elif raw_data == "CAL_DONE":
                    self.cal_complete = True

    def write_serial_data(self, data):
        if self.ser is None:
            return
        if isinstance(data, bytes):
            self.ser.write(data)
        else:
            self.ser.write(data.encode("utf-8"))

    def send_sequence(self, keyframes: list[Keyframe], auto_run: bool = True):
        self.write_serial_data(ProtocolEncoder.enter_sequence_mode(True))
        for idx, kf in enumerate(keyframes):
            targets = [t if t is not None else 0.0 for t in kf.targets]
            active = [t is not None for t in kf.targets]
            durations = [
                kf.motor_durations[i]
                if kf.motor_durations[i] is not None
                else kf.duration_ms
                for i in range(NUM_MOTORS)
            ]
            self.write_serial_data(
                ProtocolEncoder.send_keyframe(
                    idx, targets, active, durations, kf.relative
                )
            )
        if auto_run:
            self.write_serial_data(ProtocolEncoder.seq_auto_run(True))
        self.write_serial_data(ProtocolEncoder.seq_step_forward())

    def send_serial_heartbeat(self):
        self.ser.write("\n")

    # update variables to be published
    def update_data(self, data):
        # IMU
        self.imu_pitch = float(data[SerialField.IMU_PITCH])
        self.imu_roll = float(data[SerialField.IMU_ROLL])
        self.imu_yaw = float(data[SerialField.IMU_YAW])
        self.accel_x = float(data[SerialField.ACCEL_X])
        self.accel_y = float(data[SerialField.ACCEL_Y])
        self.accel_z = float(data[SerialField.ACCEL_Z])
        self.imu_qw = float(data[SerialField.IMU_QW])
        self.imu_qx = float(data[SerialField.IMU_QX])
        self.imu_qy = float(data[SerialField.IMU_QY])
        self.imu_qz = float(data[SerialField.IMU_QZ])

        # Encoders — convert cm to meters
        self.FC_pos = float(data[SerialField.FC_POS]) / 100.0
        self.RC_pos = float(data[SerialField.RC_POS]) / 100.0
        self.MR_pos = float(data[SerialField.MR_POS]) / 100.0
        self.ML_pos = float(data[SerialField.ML_POS]) / 100.0
        self.ML_carriage_pos = float(data[SerialField.ML_CARRIAGE_POS]) / 100.0
        self.MR_carriage_pos = float(data[SerialField.MR_CARRIAGE_POS]) / 100.0
        # TODO: ML/MR wheel joints are revolute — position should be in radians.
        # Convert from distance traveled (m) to radians using wheel radius when known.
        # self.ML_wheel_pos = float(data[SerialField.ML_WHEEL_POS]) / 100.0
        # self.MR_wheel_pos = float(data[SerialField.MR_WHEEL_POS]) / 100.0

        # Loadcells
        self.RC_loadcell = float(data[SerialField.RC_LOADCELL])
        self.FC_loadcell = float(data[SerialField.FC_LOADCELL])
        self.MR_loadcell = float(data[SerialField.MR_LOADCELL])
        self.ML_loadcell = float(data[SerialField.ML_LOADCELL])

        # app_time
        # self.app_time = float(data[SerialField.APP_TIME])

        # Velocity — convert cm/s to m/s
        # TODO: Fix wheel velocities once they are sent by the Teensy (currently sending 0 in SerialField.SPEED_ML and SPEED_MR)
        # self.prev_speed_ML = self.current_speed_ML
        # self.current_speed_ML = float(data[SerialField.SPEED_ML]) / 100.0
        # self.prev_speed_MR = self.current_speed_MR
        # self.current_speed_MR = float(data[SerialField.SPEED_MR]) / 100.0

        # State
        self.state = int(data[SerialField.STATE])

        self.fb_pwm = int(100.0 * float(SerialField.FB_PWM))

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "FC_joint",
            "RC_joint",
            "MR_joint",
            "ML_joint",
            "ML_carriage_joint",
            "MR_carriage_joint",
            "ML_wheel_joint",
            "MR_wheel_joint",
        ]
        msg.position = [
            self.FC_pos,
            self.RC_pos,
            self.MR_pos,
            self.ML_pos,
            self.ML_carriage_pos,
            self.MR_carriage_pos,
            self.ML_wheel_pos,
            self.MR_wheel_pos,
        ]  # joint positions from encoders
        self.joint_state_publisher.publish(msg)

    def publish_RAMMPPrototypeState(self):
        msg = RAMMPPrototypeState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # TODO: populate orientation, linear_acceleration, angular_velocity, tilt
        # once Teensy sends quaternion data directly over serial

        # Encoders
        msg.fc_pos = self.FC_pos
        msg.rc_pos = self.RC_pos
        msg.mr_pos = self.MR_pos
        msg.ml_pos = self.ML_pos
        msg.ml_carriage_pos = self.ML_carriage_pos
        msg.mr_carriage_pos = self.MR_carriage_pos
        msg.ml_wheel_pos = self.ML_wheel_pos
        msg.mr_wheel_pos = self.MR_wheel_pos

        # loadcells
        msg.rc_loadcell = self.RC_loadcell
        msg.fc_loadcell = self.FC_loadcell
        msg.mr_loadcell = self.MR_loadcell
        msg.ml_loadcell = self.ML_loadcell

        # IMU
        msg.orientation.x = self.imu_qx
        msg.orientation.y = self.imu_qy
        msg.orientation.z = self.imu_qz
        msg.orientation.w = self.imu_qw

        msg.linear_acceleration.x = self.accel_x
        msg.linear_acceleration.y = self.accel_y
        msg.linear_acceleration.z = self.accel_z

        # TODO: add angular velocity
        try:
            msg.tilt = math.acos(
                max(-1.0, min(1.0, math.cos(self.imu_pitch) * math.cos(self.imu_roll)))
            )
        except ValueError:
            msg.tilt = 0.0

        # state
        msg.state = self.state

        # app time
        msg.app_time = float(self.app_time)

        # velocities
        msg.ml_vel = float(self.current_speed_ML)
        msg.mr_vel = float(self.current_speed_MR)
        # TODO: Add the remaining velocities once they are sent by the Teensy

        self.RAMMPPrototypeState_publisher.publish(msg)

    def publish_imu_data(self):
        msg = Imu()
        # populate Imu message fields with appropriate data
        msg.linear_acceleration.x = self.accel_x
        msg.linear_acceleration.y = self.accel_y
        msg.linear_acceleration.z = self.accel_z

        msg.orientation.x = self.imu_qx
        msg.orientation.y = self.imu_qy
        msg.orientation.z = self.imu_qz
        msg.orientation.w = self.imu_qw

        self.imu_publisher.publish(msg)

    def check_luci_node(self, stat):
        active_nodes = self.get_node_names_and_namespaces()
        # Build full paths
        active_nodes = [ns.rstrip("/") + "/" + name for name, ns in active_nodes]

        # TODO: test this
        if "/interfaces" not in active_nodes:
            stat.add("node_status", "dead")
            stat.summary(DiagnosticStatus.ERROR, "Luci node not active")
        else:
            stat.add("node_status", "active")
            stat.summary(DiagnosticStatus.OK, "Luci node running")

        return stat

    def check_teensy_connection(self, stat):
        if self.ser is None:
            stat.add("connection_status", "not found")
            stat.summary(DiagnosticStatus.ERROR, "Serial port unavailable")
        elif self.ser.is_open:
            stat.add("connection_status", "connected")
            stat.summary(DiagnosticStatus.OK, "Teensy is connected")
        else:
            stat.add("connection_status", "disconnected")
            stat.summary(DiagnosticStatus.ERROR, "Teensy is disconnected")
        return stat

    def manual_seat_control_callback(self, msg: SeatCommand):
        self.get_logger().info("Seat command callback has been entered")
        deltas = SEAT_DELTAS.get(msg.command)
        if deltas is None:
            self.get_logger().warn(
                f"SeatCommand: unknown command {msg.command}, ignoring"
            )
            return

        kf = _build_seat_keyframe(deltas, SEAT_MOVE_DURATION_MS, msg.command)
        self.send_sequence([kf], auto_run=False)

        self.get_logger().info(
            f"SeatCommand {msg.command}: keyframe uploaded and triggered "
            f"(duration={SEAT_MOVE_DURATION_MS}ms)"
        )

    def estop_callback(self, msg):
        if msg.data:
            self.send_remove_luci()  # may be redundent, ensure user has manual control
            self.write_serial_data(
                "z\n"
            )  # triggers MotorController function NO_MOVEMENT
            self.write_serial_data("K0\n")

    def send_set_luci(self):
        request = Empty.Request()
        future = self.set_auto_remote_client.call_async(request)
        future.add_done_callback(self.luci_req_done)
        return future

    def send_remove_luci(self):
        request = Empty.Request()
        future = self.remove_auto_remote_client.call_async(request)
        future.add_done_callback(self.luci_req_done)
        return future

    def luci_req_done(self, future):
        result = future.result()
        if result:
            self.get_logger().info("Service call completed")
            return
        self.get_logger().error("Service call failed")

    def calibrate_motors_callback(self, goal):
        result = Calibration.Result()

        if not goal.request.enable:
            goal.succeed()
            result.success = False
            result.message = "Calibration not enabled"
            return result

        self.cal_joints_done = 0
        self.cal_complete = False
        self.write_serial_data(f"W0:{CALIBRATION_PWM}\n")
        self.get_logger().info("Calibration started via firmware W0 command")

        feedback_msg = Calibration.Feedback()

        while not self.cal_complete:
            if goal.is_cancel_requested:
                self.write_serial_data("W0:0\n")
                goal.canceled()
                result.success = False
                result.message = "Calibration cancelled"
                return result

            feedback_msg.joints_calibrated = self.cal_joints_done
            goal.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal.succeed()
        result.success = True
        result.message = f"Calibrated {self.cal_joints_done}/6 joints"
        return result

    def _send_joystick(self):
        msg = LuciJoystick()
        msg.forward_back = self.fb_pwm
        msg.left_right = 0
        msg.joystick_zone = _compute_zone(self.fb_pwm, 0)
        msg.input_source = INPUT_REMOTE
        self.luci_js_publisher.publish(msg)

    def curb_traverse_action_callback(self, goal):
        self.send_set_luci()  # enable LUCI control over js

        feedback_msg = CurbTraverse.Feedback()
        result = CurbTraverse.Result()

        if goal.request.direction == 1:
            json_path = (
                get_package_share_directory("rammp_prototype_driver")
                + "/config/dry_run_seq.json"
            )
        else:
            json_path = (
                get_package_share_directory("rammp_prototype_driver")
                + "/config/dry_run_seq_2.json"
            )

        keyframes = _load_keyframes_from_json(json_path)
        self.get_logger().info(f"Loaded {len(keyframes)} keyframes from {json_path}")

        self.send_sequence(keyframes, auto_run=True)

        while self.seq_mode == 0:
            time.sleep(0.01)

        while self.current_seq != self.seq_length and self.seq_mode != 0:
            if goal.is_cancel_requested:
                goal.canceled()
                result.success = False
                return result

            feedback_msg.progress = (
                self.current_seq * 100.0 / float(self.seq_length)
                if self.seq_length > 0
                else 0.0
            )
            goal.publish_feedback(feedback_msg)

            time.sleep(0.05)

        goal.succeed()
        result.success = True

        self.current_seq = 0
        self.seq_length = 0
        self.seq_mode = 0

        self.send_remove_luci()
        return result

    def drive_enable_callback(self, request, response):
        if request.data:
            self.send_set_luci()
        else:
            self.send_remove_luci()

        response.success = True  # just acknowledges request recieved and sent
        return response

    def self_level_enable_callback(self, request, response):
        if request.data:
            self.write_serial_data("L1:1\n")
        else:
            self.write_serial_data("L1:0\n")

        response.success = True  # just acknowledges request recieved and sent
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MEBotControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
