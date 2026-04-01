import ast
import math
from enum import IntEnum
from std_srvs.srv import Empty
from rclpy.executors import MultiThreadedExecutor
import time

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

    #  Indices here should match the order of data sent from the Teensy in Base.ino's serial output array
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
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self.ser = serial.Serial(
            port=serial_port,
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
        self.diagnostic_publish_rate = 1 / 1.0

        # timer for serial data reading
        self.serial_timer = self.create_timer(self.serial_rate, self.read_serial_data)

        ### Fields to store incoming data from serial for publishing in ROS messages
        # IMU
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
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

        # CA_flag
        self.CA_flag = 0

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
            Bool, "manual_seat_control", self.manual_seat_control_callback, 10
        )  # message type is placeholder

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
    def read_serial_data(self):
        line = self.ser.readline()
        if line:
            raw_data = line.decode("utf-8", errors="replace").strip()
            if raw_data.startswith("[") and raw_data.endswith(
                "]"
            ):  # check if data is in expected list format
                data = ast.literal_eval(raw_data)
                self.update_data(data)  # Update variables with new data

    def write_serial_data(self, data):
        self.ser.write(data.encode("utf-8"))

    # update variables to be published
    def update_data(self, data):
        # IMU
        self.imu_pitch = data[SerialField.IMU_PITCH]
        self.imu_roll = data[SerialField.IMU_ROLL]
        self.accel_x = data[SerialField.ACCEL_X]
        self.accel_y = data[SerialField.ACCEL_Y]
        self.accel_z = data[SerialField.ACCEL_Z]

        # Encoders — convert cm to meters
        self.FC_pos = data[SerialField.FC_POS] / 100.0
        self.RC_pos = data[SerialField.RC_POS] / 100.0
        self.MR_pos = data[SerialField.MR_POS] / 100.0
        self.ML_pos = data[SerialField.ML_POS] / 100.0
        self.ML_carriage_pos = data[SerialField.ML_CARRIAGE_POS] / 100.0
        self.MR_carriage_pos = data[SerialField.MR_CARRIAGE_POS] / 100.0
        # TODO: ML/MR wheel joints are revolute — position should be in radians.
        # Convert from distance traveled (m) to radians using wheel radius when known.
        self.ML_wheel_pos = data[SerialField.ML_WHEEL_POS] / 100.0
        self.MR_wheel_pos = data[SerialField.MR_WHEEL_POS] / 100.0

        # Loadcells
        self.FC_loadcell = data[SerialField.FC_LOADCELL]
        self.MR_loadcell = data[SerialField.MR_LOADCELL]
        self.ML_loadcell = data[SerialField.ML_LOADCELL]

        # CA_flag
        self.CA_flag = int(data[SerialField.CA_FLAG])

        # app_time
        self.app_time = data[SerialField.APP_TIME]

        # Velocity — convert cm/s to m/s
        # TODO:
        self.prev_speed_ML = self.current_speed_ML
        self.current_speed_ML = data[SerialField.SPEED_ML] / 100.0
        self.prev_speed_MR = self.current_speed_MR
        self.current_speed_MR = data[SerialField.SPEED_MR] / 100.0

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
        msg.fc_loadcell = self.FC_loadcell
        msg.mr_loadcell = self.MR_loadcell
        msg.ml_loadcell = self.ML_loadcell

        # CA_flag
        msg.ca_flag = self.CA_flag

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

        # convert IMU angles from degrees to radians for orientation fields
        pitch = math.radians(self.imu_pitch)
        roll = math.radians(self.imu_roll)
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

    def estop_callback(self, msg):
        if msg.data:
            # content
            pass
        else:
            # content
            pass

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
        if goal.request.direction == 1:
            self.write_serial_data("c\n")
        if goal.request.direction == 0:
            self.write_serial_data("d\n")

        feedback_msg = CurbTraverse.Feedback()
        result = CurbTraverse.Result()

        # Poll CA_flag until the final step is reached
        while self.CA_flag != 6:
            if goal.is_cancel_requested:
                goal.canceled()
                result.success = False
                return result

            feedback_msg.ca_flag = self.CA_flag
            goal.publish_feedback(feedback_msg)

            time.sleep(0.05)

        goal.succeed()
        result.success = True
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

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MEBotControlNode()
    executor = MultiThreadedExecutor()  # for action server
    executor.add_node(node)

    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
