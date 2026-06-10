import json
import math
import threading
import time
from enum import IntEnum

import diagnostic_updater
import rclpy
import serial
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus
from luci_messages.msg import LuciJoystick
from rammp_prototype_interfaces.action import Calibration, CurbTraverse
from rammp_prototype_interfaces.msg import RAMMPPrototypeState, SeatCommand
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty, SetBool

from .joint_converter import BASE_JOINT_ORDER, JOINT_CONVERSIONS
from .keyframe import NUM_MOTORS, Keyframe
from .protocol import ProtocolEncoder, ProtocolParser, SeqGuardTrigData

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

INPUT_REMOTE = 5

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

# Per-motor deltas for seat commands (motors 0-7 only; ODrive slots 8-9 inactive).
# Order: RC, FC, ML, MR, ML_Carriage, MR_Carriage, Drive_FB, Drive_LR, OD_R, OD_L
SEAT_DELTAS: dict[int, list[float]] = {
    SeatCommand.RAISE: [70.0, 0.0, 40.0, 40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.LOWER: [-70.0, 0.0, -40.0, -40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.TILT_FWD: [0.0, 0.0, -40.0, -40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.TILT_BACK: [0.0, 0.0, 40.0, 40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.LATERAL_LEFT: [0.0, 0.0, -30.0, 30.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.LATERAL_RIGHT: [0.0, 0.0, 30.0, -30.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.RESET: [300.0, 25.0, 132.0, 112.0, 100.0, 100.0, 0.0, 0.0],
}


def _load_keyframes_from_json(json_path: str) -> list[Keyframe]:
    with open(json_path, "r") as f:
        data = json.load(f)
    keyframes_data = data.get("keyframes", data if isinstance(data, list) else [])
    return [Keyframe.from_dict(d) for d in keyframes_data]


def _build_seat_keyframe(deltas: list[float], duration_ms: int, command) -> Keyframe:
    kf = Keyframe()
    kf.duration_ms = duration_ms
    # Always NUM_MOTORS slots; ODrive indices 8-9 stay None (inactive).
    for i, d in enumerate(deltas[:NUM_MOTORS]):
        kf.targets[i] = float(d)

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

    FC_LOADCELL = 53
    RC_LOADCELL = 52
    MR_LOADCELL = 55
    ML_LOADCELL = 54
    ML_WHEEL_POS = 70
    MR_WHEEL_POS = 71
    ML_WHEEL_VEL = 72
    MR_WHEEL_VEL = 73
    ODRIVE_L_POS = 78
    ODRIVE_R_POS = 79
    ODRIVE_L_TORQUE_NM = 80
    ODRIVE_R_TORQUE_NM = 81
    STATE = 2
    FB_PWM = 66


class SystemState(IntEnum):
    INIT = 0
    IDLE = 1
    TUNER_MODE = 2
    ESTOP = 3
    SELF_LEVELING = 4
    CONFIGURATION = 5
    AUTO_CURB_CLIMBING = 6
    CALIBRATING = 7
    UNCALIBRATED = 8


class MEBotControlNode(Node):
    def __init__(self):
        super().__init__("base_control_node")

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

        self.lock = threading.Lock()

        # diagnostics updater init
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("MEBot")
        self.updater.add("LUCI status", self.check_luci_node)
        self.updater.add("Teensy connection", self.check_teensy_connection)
        self.updater.add("Teensy state", self.check_teensy_state)

        # Data transfer rates
        # Rate to read data from serial
        self.serial_rate = 1 / 1000.0
        # Rate to publish joint states
        self.joint_state_rate = 1 / 100.0
        # Rate to publish RAMMPPrototypeState
        self.state_publish_rate = 1 / 100.0
        # Diagnostic publish rate
        self.diagnostic_publish_rate = 1 / 1.0
        # heartbeat timer
        self.heartbeat_rate = 0.5

        # timer for serial data reading
        self.serial_timer = self.create_timer(self.serial_rate, self.read_serial_data)

        # heartbeat to send serial message from jetson ->teensy to prevent teensy from timing out
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_rate, self.send_serial_heartbeat
        )

        self.estop = False
        self.user_fb = 0
        self.user_lr = 0
        self.user_control_enabled = True

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
        self.ML_wheel_vel = 0.0
        self.MR_wheel_vel = 0.0
        self.odrive_l_pos = 0.0
        self.odrive_r_pos = 0.0
        # self.odrive_l_torque_nm = 0.0
        # self.odrive_r_torque_nm = 0.0

        # Loadcells
        self.RC_loadcell = 0.0
        self.FC_loadcell = 0.0
        self.MR_loadcell = 0.0
        self.ML_loadcell = 0.0

        # state
        self.state = RAMMPPrototypeState.STATE_IDLE

        # app time
        self.app_time = 0.0

        self.current_speed_ML = 0.0
        self.current_speed_MR = 0.0

        self.cal_joints_done = 0
        self.cal_complete = False

        # drive wheel velocities
        self.fb_pwm = 0
        self.test_pwm = 0

        # Debug: track previous fb_pwm to detect transitions
        self._prev_fb_pwm = 0
        # Throttle joystick warnings to avoid flooding logs
        self._js_warn_count = 0
        self._js_warn_interval = 200  # log every Nth unexpected non-zero publish

        #### Init all ROS interfaces
        self._init_services()
        self._init_actions()
        self._init_subscribers()
        self._init_publishers()

        self.enable_remote_input()

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
        self.set_remote_input = self.create_client(
            Empty, "/luci/set_shared_remote_input"
        )
        self.remove_remote_input = self.create_client(
            Empty, "/luci/remove_shared_remote_input"
        )

    def _init_actions(self):
        # actions
        self.curb_traverse_action = ActionServer(
            self,
            CurbTraverse,
            "curb_traverse",
            self.curb_traverse_action_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
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

        self.user_joystick_subscription = self.create_subscription(
            LuciJoystick, "luci/joystick_position", self.user_joystick_callback, 10
        )

    def _init_publishers(self):
        # FC loadcell publisher
        self.fc_loadcell_pub = self.create_publisher(Float32, "fc_loadcell", 10)
        self.fc_loadcell_timer = self.create_timer(1.0, self.pub_fc_loadcell)
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
        self.luci_heartbeat_timer.cancel()  # start with heartbeat disabled until LUCI control is enabled

        # self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        # self.imu_timer = self.create_timer(self.publish_rate, self.publish_imu_data)

    def pub_fc_loadcell(self):
        msg = Float32()
        msg.data = self.FC_loadcell
        self.fc_loadcell_pub.publish(msg)

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
                parsed = ProtocolParser.parse_line(raw_data)
                if isinstance(parsed, SeqGuardTrigData):
                    self.get_logger().info(
                        f"Guard triggered: motor {parsed.motor_index}, load={parsed.load_value:.1f}"
                    )
                if raw_data.startswith("CAL: Homed"):
                    self.cal_joints_done += 1
                elif raw_data == "CAL_DONE":
                    self.cal_complete = True

    def write_serial_data(self, data):
        if self.ser is None:
            return
        if isinstance(data, bytes):
            with self.lock:
                self.ser.write(data)
        else:
            with self.lock:
                self.ser.write(data.encode("utf-8"))

    def send_sequence(self, keyframes: list[Keyframe], auto_run: bool = True):
        self.write_serial_data(ProtocolEncoder.enter_sequence_mode(True))
        for idx, kf in enumerate(keyframes):
            # Always emit NUM_MOTORS fields (firmware rejects 8-motor / 32-value frames).
            targets = [0.0] * NUM_MOTORS
            active = [False] * NUM_MOTORS
            for i in range(NUM_MOTORS):
                t = kf.targets[i] if i < len(kf.targets) else None
                if t is not None:
                    targets[i] = float(t)
                    active[i] = True
            relative = list(kf.relative[:NUM_MOTORS])
            while len(relative) < NUM_MOTORS:
                relative.append(False)
            durations = [
                kf.motor_durations[i]
                if i < len(kf.motor_durations) and kf.motor_durations[i] is not None
                else kf.duration_ms
                for i in range(NUM_MOTORS)
            ]
            line = ProtocolEncoder.send_keyframe(
                idx,
                targets,
                active,
                durations,
                relative,
                guard_threshold=kf.guard_threshold,
                guard_condition=kf.guard_condition,
            )
            payload = line.decode().strip().split(":", 1)[1]
            n_fields = len(payload.split(","))
            self.get_logger().info(
                f"Keyframe {idx}: sending {n_fields} fields "
                f"(standard={NUM_MOTORS * 4}, guarded={NUM_MOTORS * 6})"
            )
            if n_fields not in (NUM_MOTORS * 4, NUM_MOTORS * 6, NUM_MOTORS * 2 + 1):
                self.get_logger().error(
                    f"Keyframe {idx} field count {n_fields} does not match "
                    f"NUM_MOTORS={NUM_MOTORS}; Teensy will reject with SEQ_ERR"
                )
            self.write_serial_data(line)
        if auto_run:
            self.write_serial_data(ProtocolEncoder.seq_auto_run(True))
        self.write_serial_data(ProtocolEncoder.seq_step_forward())

    def send_serial_heartbeat(self):
        if not self.estop:
            self.write_serial_data("c\n")
        else:
            self.write_serial_data("z\n")

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

        # Encoder positions — raw ticks from firmware
        self.RC_pos = float(data[SerialField.RC_POS])
        self.FC_pos = float(data[SerialField.FC_POS])
        self.ML_pos = float(data[SerialField.ML_POS])
        self.MR_pos = float(data[SerialField.MR_POS])
        self.ML_carriage_pos = float(data[SerialField.ML_CARRIAGE_POS])
        self.MR_carriage_pos = float(data[SerialField.MR_CARRIAGE_POS])
        self.ML_wheel_pos = float(data[SerialField.ML_WHEEL_POS])
        self.MR_wheel_pos = float(data[SerialField.MR_WHEEL_POS])

        # Wheel velocities — raw ticks/sec from firmware
        self.ML_wheel_vel = float(data[SerialField.ML_WHEEL_VEL])
        self.MR_wheel_vel = float(data[SerialField.MR_WHEEL_VEL])

        # Loadcells
        self.RC_loadcell = float(data[SerialField.RC_LOADCELL])
        self.FC_loadcell = float(data[SerialField.FC_LOADCELL])
        self.MR_loadcell = float(data[SerialField.MR_LOADCELL])
        self.ML_loadcell = float(data[SerialField.ML_LOADCELL])

        # State
        self.state = int(data[SerialField.STATE])

        new_fb_pwm = int(100.0 * float(data[SerialField.FB_PWM]))

        # Log transitions: zero→non-zero and non-zero→zero
        if new_fb_pwm != 0 and self._prev_fb_pwm == 0:
            self.get_logger().warn(
                f"JoystickDebug: fb_pwm went non-zero: {new_fb_pwm} "
                f"(raw={data[SerialField.FB_PWM]}, state={self.state})"
            )
        elif new_fb_pwm == 0 and self._prev_fb_pwm != 0:
            self.get_logger().info(
                f"JoystickDebug: fb_pwm returned to zero (was {self._prev_fb_pwm}, state={self.state})"
            )
        self._prev_fb_pwm = new_fb_pwm
        self.fb_pwm = new_fb_pwm

        if len(data) > SerialField.ODRIVE_R_POS:
            self.odrive_l_pos = float(data[SerialField.ODRIVE_L_POS])
            self.odrive_r_pos = float(data[SerialField.ODRIVE_R_POS])
        # if len(data) > SerialField.ODRIVE_R_TORQUE_NM:
        #     self.odrive_l_torque_nm = float(data[SerialField.ODRIVE_L_TORQUE_NM])
        #     self.odrive_r_torque_nm = float(data[SerialField.ODRIVE_R_TORQUE_NM])

    def publish_joint_states(self):
        conv = JOINT_CONVERSIONS
        raw_ticks = {
            "front_caster_swing_arm": self.FC_pos,
            "rear_caster_swing_arm": self.RC_pos,
            "motor_swing_arm_r": self.MR_pos,
            "motor_swing_arm_l": self.ML_pos,
            "dw_main_plate_l": self.ML_carriage_pos,
            "dw_main_plate_r": self.MR_carriage_pos,
            "drive_wheel_l": self.ML_wheel_pos,
            "drive_wheel_r": self.MR_wheel_pos,
        }
        raw_vels = {
            "drive_wheel_l": self.ML_wheel_vel,
            "drive_wheel_r": self.MR_wheel_vel,
        }

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(BASE_JOINT_ORDER)
        msg.position = [conv[n].position(raw_ticks[n]) for n in BASE_JOINT_ORDER]
        msg.velocity = [
            conv[n].velocity(raw_vels[n]) if n in raw_vels else 0.0
            for n in BASE_JOINT_ORDER
        ]
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

        # msg.odrive_l_torque_nm = float(self.odrive_l_torque_nm)
        # msg.odrive_r_torque_nm = float(self.odrive_r_torque_nm)

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
        if "/interface" not in active_nodes:
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

    def check_teensy_state(self, stat):
        if self.state == SystemState.ESTOP:
            stat.add("state_status", "ESTOP state")
            stat.summary(DiagnosticStatus.ERROR, "Teensy in estop state")
        elif self.state == SystemState.INIT:
            stat.add("state_status", "Initialization state")
            stat.summary(DiagnosticStatus.OK, "Teensy in initialization state")
        elif self.state == SystemState.IDLE:
            stat.add("state_status", "Idle state")
            stat.summary(DiagnosticStatus.OK, "Teensy in idle state")
        elif self.state == SystemState.TUNER_MODE:
            stat.add("state_status", "Tuner mode")
            stat.summary(DiagnosticStatus.OK, "Teensy in tuner mode")
        elif self.state == SystemState.SELF_LEVELING:
            stat.add("state_status", "Self-leveling mode")
            stat.summary(DiagnosticStatus.OK, "Teensy in self-leveling mode")
        elif self.state == SystemState.CONFIGURATION:
            stat.add("state_status", "Configuration mode")
            stat.summary(DiagnosticStatus.OK, "Teensy in configuration mode")
        elif self.state == SystemState.AUTO_CURB_CLIMBING:
            stat.add("state_status", "Auto curb climbing mode")
            stat.summary(DiagnosticStatus.OK, "Teensy in auto curb climbing mode")
        elif self.state == SystemState.CALIBRATING:
            stat.add("state_status", "Calibrating")
            stat.summary(DiagnosticStatus.OK, "Teensy is calibrating")
        elif self.state == SystemState.UNCALIBRATED:
            stat.add("state_status", "Uncalibrated")
            stat.summary(
                DiagnosticStatus.WARN,
                "Teensy is uncalibrated — calibration required before operation",
            )
        return stat

    def user_joystick_callback(self, msg: LuciJoystick):
        self.user_fb = msg.forward_back
        self.user_lr = msg.left_right

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
        self.estop = msg.data
        if msg.data:
            self.user_control_enabled = True
            # self.send_remove_luci()  # may be redundent, ensure user has manual control
            self.write_serial_data(
                "z\n"
            )  # triggers MotorController function NO_MOVEMENT
            self.write_serial_data("K0\n")

    def enable_remote_input(self):
        request = Empty.Request()
        future = self.set_remote_input.call_async(request)
        future.add_done_callback(self.luci_req_done)
        self.get_logger().info("Remote input enabled")
        return future

    def disable_remote_input(self):
        request = Empty.Request()
        future = self.remove_remote_input.call_async(request)
        future.add_done_callback(self.remote_input_done)
        self.get_logger().info("Remote input disabled")
        return future

    def send_set_luci(self):
        self.get_logger().info(
            f"JoystickDebug: setting LUCI auto remote input (state={self.state}, fb_pwm={self.fb_pwm})"
        )
        request = Empty.Request()
        future = self.set_auto_remote_client.call_async(request)
        future.add_done_callback(self.luci_req_done)

        self.luci_heartbeat_timer.reset()
        return future

    def send_remove_luci(self):
        self.get_logger().info(
            f"JoystickDebug: removing LUCI auto remote input (state={self.state}, fb_pwm={self.fb_pwm})"
        )
        request = Empty.Request()
        future = self.remove_auto_remote_client.call_async(request)
        future.add_done_callback(self.luci_req_done)

        self.luci_heartbeat_timer.cancel()
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

    def _send_joystick(self, fb_pwm=None):
        msg = LuciJoystick()
        if self.user_control_enabled:
            msg.forward_back = self.user_fb
            msg.left_right = self.user_lr
        # elif self.user_control_enabled and fb_pwm is not None:
        #     msg.forward_back = fb_pwm
        #     msg.left_right = 0
        else:
            msg.forward_back = self.fb_pwm
            msg.left_right = 0

        msg.joystick_zone = _compute_zone(msg.forward_back, msg.left_right)
        msg.input_source = INPUT_REMOTE
        self.luci_js_publisher.publish(msg)

        # Warn when publishing non-zero joystick data outside of active drive states
        if self.fb_pwm != 0 and self.state != SystemState.AUTO_CURB_CLIMBING:
            self._js_warn_count += 1
            if (
                self._js_warn_count == 1
                or self._js_warn_count % self._js_warn_interval == 0
            ):
                self.get_logger().warn(
                    f"JoystickDebug: non-zero joystick published in state {self.state}: "
                    f"fb_pwm={self.fb_pwm} (occurrence #{self._js_warn_count})"
                )
        else:
            if self._js_warn_count > 0:
                self.get_logger().info(
                    f"JoystickDebug: normalized after {self._js_warn_count} unexpected publishes"
                )
            self._js_warn_count = 0

    def curb_traverse_action_callback(self, goal):
        # self.send_set_luci()  # enable LUCI control over js

        # feedback_msg = CurbTraverse.Feedback()
        result = CurbTraverse.Result()

        # call the calibration function before going down curb
        # self.cal_joints_done = 0
        # self.cal_complete = False
        # self.write_serial_data(f"W0:{CALIBRATION_PWM}\n")
        # # delay while calibration runs
        # time.sleep(6)

        # self.self_level_enable_callback(True)
        # time.sleep(3)

        if goal.request.direction == 1:
            # send first kf to get chair at height to detect curb
            json_path = (
                get_package_share_directory("rammp_prototype_driver")
                + "/config/ascend_approach.json"
            )
            keyframes = _load_keyframes_from_json(json_path)
            self.send_sequence(keyframes, auto_run=True)

            # waiting for user to hit front caster on curb
            while self.FC_loadcell > 150:
                if goal.is_cancel_requested:
                    goal.canceled()
                    result.success = False
                    self.write_serial_data(ProtocolEncoder.enter_sequence_mode(False))
                    self.write_serial_data("z\n")
                    self.write_serial_data("c\n")
                    return result
                time.sleep(0.01)

            # immediately remove user joystick control and stop drive wheels
            self.send_set_luci()
            self._send_joystick(0)
            time.sleep(3.0)
            json_path = (
                get_package_share_directory("rammp_prototype_driver")
                + "/config/curb_ascending.json"
            )
        else:
            # send first kf to get chair at height to detect ground
            json_path = (
                get_package_share_directory("rammp_prototype_driver")
                + "/config/descend_approach.json"
            )
            keyframes = _load_keyframes_from_json(json_path)
            self.send_sequence(keyframes, auto_run=True)

            # waiting for user to get front caster off curb
            while self.FC_loadcell < 150:
                if goal.is_cancel_requested:
                    goal.canceled()
                    result.success = False
                    self.write_serial_data(ProtocolEncoder.enter_sequence_mode(False))
                    self.write_serial_data("z\n")
                    self.write_serial_data("c\n")
                    return result
                time.sleep(0.01)

            # immediately remove user joystick control and stop drive wheels
            self.send_set_luci()
            self._send_joystick(0)
            time.sleep(3.0)
            json_path = (
                get_package_share_directory("rammp_prototype_driver")
                + "/config/curb_descending.json"
            )

        keyframes = _load_keyframes_from_json(json_path)
        self.get_logger().info(f"Loaded {len(keyframes)} keyframes from {json_path}")

        self.send_sequence(keyframes, auto_run=True)

        while self.seq_mode == 0:
            if goal.is_cancel_requested:
                goal.canceled()
                result.success = False
                self.send_remove_luci()
                self.write_serial_data(ProtocolEncoder.enter_sequence_mode(False))
                self.write_serial_data("z\n")
                self.write_serial_data("c\n")
                return result
            time.sleep(0.01)

        while self.current_seq != self.seq_length and self.seq_mode != 0:
            self.get_logger().info(f"Current sequence: {self.current_seq}")

            if goal.is_cancel_requested:
                goal.canceled()
                result.success = False
                self.send_remove_luci()
                self.write_serial_data(ProtocolEncoder.enter_sequence_mode(False))
                self.write_serial_data("z\n")
                self.write_serial_data("c\n")
                return result

            # feedback_msg.progress = (
            #     self.current_seq * 100.0 / float(self.seq_length)
            #     if self.seq_length > 0
            #     else 0.0
            # )
            # goal.publish_feedback(feedback_msg)

            time.sleep(0.05)

        goal.succeed()
        result.success = True

        self.current_seq = 0
        self.seq_length = 0
        self.seq_mode = 0

        self.send_remove_luci()
        self.write_serial_data(ProtocolEncoder.enter_sequence_mode(False))
        return result

    def drive_enable_callback(self, request, response):
        if request.data:
            self.user_control_enabled = True
        else:
            self.user_control_enabled = False

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
