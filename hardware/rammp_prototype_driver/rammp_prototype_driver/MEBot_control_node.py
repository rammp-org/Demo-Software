import math
from enum import IntEnum
from std_srvs.srv import Empty
from rclpy.executors import MultiThreadedExecutor
import time
import json

import rclpy
import serial
from rammp_prototype_interfaces.action import CurbTraverse
from rammp_prototype_interfaces.msg import SeatCommand

# custom msgs/srvs
from rammp_prototype_interfaces.msg import RAMMPPrototypeState
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus


# Sequence player motor order (must match SEQ_NUM_MOTORS order in Teensy):
#   0=RC, 1=FC, 2=ML, 3=MR, 4=ML_CARRIAGE, 5=MR_CARRIAGE, 6=DRIVE_FB, 7=DRIVE_LR
SEQ_NUM_MOTORS = 8

# Duration (ms) for each seat command's interpolation.
# Increase for slower/smoother motion, decrease for snappier response.
# *** TUNE to your preference. ***
SEAT_MOVE_DURATION_MS = 1000

# Per-command relative deltas (encoder ticks) for each motor.
# Order: [RC, FC, ML, MR, ML_CARRIAGE, MR_CARRIAGE, DRIVE_FB, DRIVE_LR]
# 0.0 = motor inactive for this command (active flag will be False).
# *** TUNE these deltas once you know your geometry. ***
SEAT_DELTAS: dict[int, list[float]] = {
    #                           RC      FC      ML      MR    ML_C   MR_C    DFB   DLR
    SeatCommand.RAISE: [70.0, 0.0, 40.0, 40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.LOWER: [-70.0, 0.0, -40.0, -40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.TILT_FWD: [0.0, 0.0, -40.0, -40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.TILT_BACK: [0.0, 0.0, 40.0, 40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.LATERAL_LEFT: [0.0, 0.0, -40.0, 40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.LATERAL_RIGHT: [0.0, 0.0, 40.0, -40.0, 0.0, 0.0, 0.0, 0.0],
    SeatCommand.RESET: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

# for testing curb traversal action callback
move_sequence = [
    SEAT_DELTAS[SeatCommand.RAISE],
    SEAT_DELTAS[SeatCommand.LOWER],
    SEAT_DELTAS[SeatCommand.RAISE],
    SEAT_DELTAS[SeatCommand.LOWER],
]


def _build_keyframe_payload(deltas: list[float], duration_ms: int, command) -> str:
    """Build a J0 keyframe payload string in the 32-value format expected by
    parseKeyframePayload():
        targets(x6), active(x6), relative(x6), duration_ms(x6)

    Motors with a delta of 0.0 are marked inactive (active=0) so the
    sequence player leaves them at their current position.
    """

    if command == SeatCommand.RESET:  # reset command
        active = [1] * SEQ_NUM_MOTORS
        relative = [0] * SEQ_NUM_MOTORS  # always absolute for seat reset
        relative[6] = 1
        relative[7] = 1
        durations = [duration_ms] * SEQ_NUM_MOTORS

        parts = (
            [f"{d:.2f}" for d in deltas]
            + [str(a) for a in active]
            + [str(r) for r in relative]
            + [str(t) for t in durations]
        )
        return ",".join(parts)

    active = [1 if d != 0.0 else 0 for d in deltas]
    relative = [1] * SEQ_NUM_MOTORS  # always relative for seat moves
    durations = [duration_ms] * SEQ_NUM_MOTORS

    parts = (
        [f"{d:.2f}" for d in deltas]
        + [str(a) for a in active]
        + [str(r) for r in relative]
        + [str(t) for t in durations]
    )
    return ",".join(parts)


def _build_array_of_keyframes(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)

    sequence = []

    for kf in data["keyframes"]:
        targets = (f"{t:.2f}" for t in kf["targets"])
        active = (1 if t != 0.0 else 0 for t in kf["targets"])
        duration_ms = kf["duration_ms"]
        relative = [int(r) for r in kf["relative"]]

        durations = [duration_ms if md is None else md for md in kf["motor_durations"]]

        parts = (
            [str(t) for t in targets]
            + [str(a) for a in active]
            + [str(r) for r in relative]
            + [str(t) for t in durations]
        )
        row = ",".join(parts)
        sequence.append(row)

    return sequence


# def _build_reset_keyframe_payload(deltas: list[float], duration_ms: int) -> str:
#     """Build a J0 keyframe payload string in the 32-value format expected by
#     parseKeyframePayload():
#         targets(x6), active(x6), relative(x6), duration_ms(x6)

#     Motors with a delta of 0.0 are marked inactive (active=0) so the
#     sequence player leaves them at their current position.
#     """
#     active = [1] * SEQ_NUM_MOTORS
#     relative = [0] * SEQ_NUM_MOTORS  # always absolute for seat reset
#     relative[6] = 1
#     relative[7] = 1
#     durations = [duration_ms] * SEQ_NUM_MOTORS

#     parts = (
#         [f"{d:.2f}" for d in deltas]
#         + [str(a) for a in active]
#         + [str(r) for r in relative]
#         + [str(t) for t in durations]
#     )
#     return ",".join(parts)


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
    # ML_WHEEL_POS = 0
    # MR_WHEEL_POS = 0
    FC_LOADCELL = 53
    RC_LOADCELL = 52
    MR_LOADCELL = 55
    ML_LOADCELL = 54
    # APP_TIME = 0
    # SPEED_ML = 0
    # SPEED_MR = 0
    STATE = 2


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

    def write_serial_data(self, data):
        if self.ser is None:
            return
        self.ser.write(data.encode("utf-8"))

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

        # TODO: get correct LUCI node name and namespace
        if "/luci/node" not in active_nodes:
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

        # Upload keyframe 0 with relative deltas
        payload = _build_keyframe_payload(deltas, SEAT_MOVE_DURATION_MS, msg.command)
        self.write_serial_data("B1:1\n")
        self.write_serial_data(f"J0:{payload}\n")

        # Trigger execution (CMD_SEQ_STEP_FWD)
        self.write_serial_data(">\n")

        self.get_logger().info(
            f"SeatCommand {msg.command}: keyframe uploaded and triggered "
            f"(duration={SEAT_MOVE_DURATION_MS}ms)"
            f" for J0:{payload})"
        )

    def estop_callback(self, msg):
        if msg.data:
            self.send_remove_luci()  # may be redundent, ensure user has manual control
            self.write_serial_data(
                "z\n"
            )  # triggers MotorController function NO_MOVEMENT

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

    def curb_traverse_action_callback(self, goal):
        # TODO: add checkpoint here checking if sequence flag is at starting/default state, curb traversal should not be called if MEBot already in curb traversal (default flag is 0)
        # TODO: add descending flag
        if goal.request.direction == 1:
            self.write_serial_data("c\n")
        if goal.request.direction == 0:
            self.write_serial_data("d\n")

        feedback_msg = CurbTraverse.Feedback()
        result = CurbTraverse.Result()

        json_path = "curb_climbing_wip.json"
        sequence = _build_array_of_keyframes(json_path)

        self.write_serial_data("B1:1\n")  # enter seq mode
        self.write_serial_data("B2:1\n")  # enable auto run

        for i, payload in enumerate(sequence):
            self.write_serial_data(f"J{i}:{payload}\n")

        self.write_serial_data(">\n")

        # Poll sequence step until the final step is reached
        while self.current_seq != self.seq_length and self.seq_mode != 0:
            if goal.is_cancel_requested:
                goal.canceled()
                result.success = False
                return result

            feedback_msg.current_seq = self.current_seq
            goal.publish_feedback(feedback_msg)

            time.sleep(0.05)

        # TODO: Make success true or false depending on information given by teensy that states whether or not curb traversal succeeded
        goal.succeed()
        result.success = True

        # reset sequence player data
        self.current_seq = 0
        self.seq_length = 0
        self.seq_mode = 0
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
            self.write_serial_data("s\n")
            pass
        else:
            self.write_serial_data("r\n")
            pass

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
