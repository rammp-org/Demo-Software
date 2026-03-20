import ast
import math
from enum import IntEnum

import rclpy
import serial
from rammp_prototype_interfaces.action import CurbTraverse

# custom msgs/srvs
from rammp_prototype_interfaces.msg import RAMMPPrototypeState
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


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

    IMU_PITCH = 0
    IMU_ROLL = 1
    ACCEL_X = 2
    ACCEL_Y = 3
    ACCEL_Z = 4
    FC_POS = 5
    RC_POS = 6
    MR_POS = 7
    ML_POS = 8
    ML_CARRIAGE_POS = 9
    MR_CARRIAGE_POS = 10
    ML_WHEEL_POS = 11
    MR_WHEEL_POS = 12
    FC_LOADCELL = 13
    MR_LOADCELL = 14
    ML_LOADCELL = 15
    CA_FLAG = 16
    APP_TIME = 17
    SPEED_ML = 18
    SPEED_MR = 19


class MEBotControlNode(Node):
    def __init__(self):
        super().__init__("MEBot_control_node")

        # serial init
        self.ser = serial.Serial(
            port="/dev/ttyACM0",  # USB connection
            baudrate=115200,
            timeout=1,
        )

        # Data transfer rates
        # Rate to read data from serial
        self.serial_rate = 1 / 1000.0
        # Rate to publish joint states
        self.joint_state_rate = 1 / 100.0
        # Rate to publish RAMMPPrototypeState
        self.state_publish_rate = 1 / 100.0
        # Diagnostic publish rate
        self.publish_rate = 1 / 1.0

        # timer for serial data reading
        self.serial_timer = self.create_timer(self.serial_rate, self.read_serial_data)

        # IMU
        self.IMU_pitch = 0.0
        self.IMU_roll = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

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
        self.FC_loadcell = 0.0
        self.MR_loadcell = 0.0
        self.ML_loadcell = 0.0

        # CA_flag and action
        self.CA_flag = 0.0
        self.action = (
            "z"  # variable to store most recent action command Base.ino recieved
        )

        # app time
        self.appTime = 0.0

        # velocity and acceleration
        self.prev_speed_ML = 0.0
        self.current_speed_ML = 0.0

        self.prev_speed_MR = 0.0
        self.current_speed_MR = 0.0

        self.acceleration_ML = 0.0
        self.acceleration_MR = 0.0

        # tilt and measure height
        self.tilt = 0.0
        self.measure_height = 0.0

        # Init all ROS interfaces
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

    def _init_actions(self):
        # actions
        self.curb_traverse_action = ActionServer(
            self, CurbTraverse, "curb_traverse", self.curb_traverse_action_callback
        )

    def _init_subscribers(self):
        # subscriptions
        self.manual_seat_control_subscription = self.create_subscription(
            Bool, "manual_seat_control", self.manual_seat_control_callback, 10
        )  # message type is placeholder

        self.curb_ascend_subscription = self.create_subscription(
            Bool, "curb_ascend", self.curb_ascend_callback, 10
        )

        self.curb_descend_subscription = self.create_subscription(
            Bool, "curb_descend", self.curb_descend_callback, 10
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
            self.publish_rate, self.publish_RAMMPPrototypeState
        )

        self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        self.imu_timer = self.create_timer(self.publish_rate, self.publish_imu_data)

    # reading incoming serial data from teensy
    def read_serial_data(self):
        line = self.ser.readline()
        if line:
            raw_data = line.decode("utf-8", errors="replace").strip()
            if raw_data.startswith("[") and raw_data.endswith(
                "]"
            ):  # check if data is in expected list format
                data = ast.literal_eval(raw_data)
                self.update_data(data)  # Update variables with new data
                self.publish_tf_data()  # update tf data based on updated teensy encoder data
            if raw_data.startswith(
                "Action:"
            ):  # check if data is an action command from Base.ino
                self.action = raw_data.split("Action:")[1].strip()[0]

    def write_serial_data(self, data):
        self.ser.write(data.encode("utf-8"))

    # update variables to be published
    def update_data(self, data):
        F = SerialField

        # IMU
        self.IMU_pitch = data[F.IMU_PITCH]
        self.IMU_roll = data[F.IMU_ROLL]
        self.accel_x = data[F.ACCEL_X]
        self.accel_y = data[F.ACCEL_Y]
        self.accel_z = data[F.ACCEL_Z]

        # Encoders
        self.FC_pos = data[F.FC_POS]
        self.RC_pos = data[F.RC_POS]
        self.MR_pos = data[F.MR_POS]
        self.ML_pos = data[F.ML_POS]
        self.ML_carriage_pos = data[F.ML_CARRIAGE_POS]
        self.MR_carriage_pos = data[F.MR_CARRIAGE_POS]
        self.ML_wheel_pos = data[F.ML_WHEEL_POS]
        self.MR_wheel_pos = data[F.MR_WHEEL_POS]

        # Loadcells
        self.FC_loadcell = data[F.FC_LOADCELL]
        self.MR_loadcell = data[F.MR_LOADCELL]
        self.ML_loadcell = data[F.ML_LOADCELL]

        # CA_flag
        self.CA_flag = data[F.CA_FLAG]

        # Apptime
        self.appTime = data[F.APP_TIME]

        # Velocity
        self.prev_speed_ML = self.current_speed_ML
        self.current_speed_ML = data[F.SPEED_ML]
        self.prev_speed_MR = self.current_speed_MR
        self.current_speed_MR = data[F.SPEED_MR]

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

    def publish_tf_data(self):
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = None
        # t.child_frame_id = None
        # self.tf_broadcaster.sendTransform(t)
        pass

    def publish_RAMMPPrototypeState(self):
        msg = RAMMPPrototypeState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pitch = self.IMU_pitch
        msg.roll = self.IMU_roll
        msg.ax = self.accel_x
        msg.ay = self.accel_y
        msg.az = self.accel_z
        msg.tilt = math.acos(math.cos(self.IMU_pitch) * math.cos(self.IMU_roll)) * (
            180 / math.pi
        )  # calculate tilt in degrees using pitch and roll

        # Encoders
        msg.fc_enc = self.FC_pos
        msg.fr_enc = self.RC_pos
        msg.mr_enc = self.MR_pos
        msg.ml_enc = self.ML_pos
        msg.ml_carr_enc = self.ML_carriage_pos
        msg.mr_carr_enc = self.MR_carriage_pos
        msg.ml_wheel_enc = self.ML_wheel_pos
        msg.mr_wheel_enc = self.MR_wheel_pos

        # loadcells
        msg.fc_lc = self.FC_loadcell
        msg.mr_lc = self.MR_loadcell
        msg.ml_lc = self.ML_loadcell

        # CA_flag and action
        msg.ca_flag = int(self.CA_flag)
        msg.action = str(self.action)

        # app time
        msg.app_time = float(self.appTime)

        # velocity and acceleration
        msg.ml_vel = float(self.current_speed_ML)
        msg.mr_vel = float(self.current_speed_MR)
        msg.ml_acc = float(
            self.current_speed_ML - self.prev_speed_ML
        )  # calculate acceleration using change in speed over time (0.1s between serial data updates)
        msg.mr_acc = float(
            self.current_speed_MR - self.prev_speed_MR
        )  # calculate acceleration using change in speed over time (0.1s between serial data updates)

        # measure height
        msg.measure_height = float(self.measure_height)

        self.RAMMPPrototypeState_publisher.publish(msg)

    def publish_imu_data(self):
        msg = Imu()
        # populate Imu message fields with appropriate data
        msg.linear_acceleration.x = self.accel_x
        msg.linear_acceleration.y = self.accel_y
        msg.linear_acceleration.z = self.accel_z

        # convert IMU angles from degrees to radians for orientation fields
        pitch = math.radians(self.IMU_pitch)
        roll = math.radians(self.IMU_roll)
        yaw = 0.0  # assuming yaw is 0 since it is not measured by the IMU

        # populate orientation fields using Euler angles (assuming yaw is 0)
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        self.imu_publisher.publish(msg)

    def manual_seat_control_callback(self, msg):
        if msg.data:
            # content
            pass

    def curb_ascend_callback(self, msg):
        if msg.data:
            # content
            pass

    def curb_descend_callback(self, msg):
        if msg.data:
            # content
            pass

    def estop_callback(self, msg):
        if msg.data:
            # content
            pass
        else:
            # content
            pass

    def curb_traverse_action_callback(self, goal, response):
        # content
        return response

    def drive_enable_callback(self, request, response):
        if request.data:
            # content
            pass
        else:
            # content
            pass

        return response

    def self_level_enable_callback(self, request, response):
        if request.data:
            self.write_serial_data("s\n")
            pass
        else:
            self.write_serial_data("r\n")
            pass

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MEBotControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
