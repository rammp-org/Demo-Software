import rclpy
from rclpy.node import Node
import serial
import ast
from sensor_msgs.msg import Imu
import math
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from interfaces.rammp_prototype_interfaces.msg import RAMMPPrototypeState


class MEBotControlNode(Node):
    def __init__(self):
        super().__init__("MEBot_control_node")

        # serial init
        self.ser = serial.Serial(
            port="/dev/ttyACM0",  # USB connection
            baudrate=115200,
            timeout=1,
        )

        # timer for serial data reading
        self.serial_timer = self.create_timer(0.001, self.read_serial_data)

        # publishing rate for all topics
        self.publish_rate = 0.001

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

        # services
        # self.drive_enable_service = self.create_service(
        #     SetBool, "drive_enable", self.drive_enable_callback
        # )

        # self.self_level_enable_service = self.create_service(
        #     SetBool, "self_level_enable", self.self_level_enable_callback
        # )

        # subscriptions
        # self.manual_seat_control_subscription = self.create_subscription(
        #     bool, "manual_seat_control", self.manual_seat_control_callback, 10
        # )  # message type is placeholder
        # self.curb_ascend_subscription = self.create_subscription(
        #     bool, "curb_ascend", self.curb_ascend_callback, 10
        # )
        # self.curb_descend_subscription = self.create_subscription(
        #     bool, "curb_descend", self.curb_descend_callback, 10
        # )
        # self.estop_subscription = self.create_subscription(
        #     bool, "estop", self.estop_callback, 10
        # )

        # joint state publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        self.joint_state_timer = self.create_timer(
            self.publish_rate, self.publish_joint_states
        )

        # tf publisher
        self.tf_pubslisher = self.create_publisher(TFMessage, "tf_data", 10)
        self.tf_timer = self.create_timer(self.publish_rate, self.publish_tf_data)

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
            if raw_data.startswith(
                "Action:"
            ):  # check if data is an action command from Base.ino
                self.action = raw_data.split("Action:")[1].strip()[0]

    # update variables to be published
    def update_data(self, data):
        # IMU
        self.IMU_pitch = data[0]
        self.IMU_roll = data[1]
        self.accel_x = data[2]
        self.accel_y = data[3]
        self.accel_z = data[4]

        # Encoders
        self.FC_pos = data[5]
        self.RC_pos = data[6]
        self.MR_pos = data[7]
        self.ML_pos = data[8]
        self.ML_carriage_pos = data[9]
        self.MR_carriage_pos = data[10]
        self.ML_wheel_pos = data[11]
        self.MR_wheel_pos = data[12]

        # loadcells
        self.FC_loadcell = data[13]
        self.MR_loadcell = data[14]
        self.ML_loadcell = data[15]

        # CA_flag
        self.CA_flag = data[16]

        # Apptime
        self.appTime = data[17]

        # velocity
        self.prev_speed_ML = self.current_speed_ML
        self.current_speed_ML = data[18]
        self.prev_speed_MR = self.current_speed_MR
        self.current_speed_MR = data[19]

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
        pass
        # msg = TFMessage()
        # transform = TransformStamped()

    def publish_RAMMPPrototypeState(self):
        msg = RAMMPPrototypeState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.IMU_pitch = self.IMU_pitch
        msg.IMU_roll = self.IMU_roll
        msg.accel_x = self.accel_x
        msg.accel_y = self.accel_y
        msg.accel_z = self.accel_z
        msg.tilt = math.acos(math.cos(self.IMU_pitch) * math.cos(self.IMU_roll)) * (
            180 / math.pi
        )  # calculate tilt in degrees using pitch and roll

        # Encoders
        msg.FC_enc = self.FC_pos
        msg.FR_enc = self.RC_pos
        msg.MR_enc = self.MR_pos
        msg.ML_enc = self.ML_pos
        msg.ML_carr_enc = self.ML_carriage_pos
        msg.MR_carr_enc = self.MR_carriage_pos
        msg.ML_wheel_enc = self.ML_wheel_pos
        msg.MR_wheel_enc = self.MR_wheel_pos

        # loadcells
        msg.FC_lc = self.FC_loadcell
        msg.MR_lc = self.MR_loadcell
        msg.ML_lc = self.ML_loadcell

        # CA_flag and action
        msg.ca_flag = int(self.CA_flag)
        msg.action = str(self.action)

        # app time
        msg.app_time = float(self.appTime)

        # velocity and acceleration
        msg.ML_vel = float(self.current_speed_ML)
        msg.MR_vel = float(self.current_speed_MR)
        msg.ML_acc = float(
            self.current_speed_ML - self.prev_speed_ML
        )  # calculate acceleration using change in speed over time (0.1s between serial data updates)
        msg.MR_acc = float(
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

    # def manual_seat_control_callback(self, msg):
    #     if msg.data:
    #         # content
    #         pass

    # def curb_ascend_callback(self, msg):
    #     if msg.data:
    #         # content
    #         pass

    # def curb_descend_callback(self, msg):
    #     if msg.data:
    #         # content
    #         pass

    # def drive_enable_callback(self, request, response):
    #     if request.data:
    #         # content
    #         pass
    #     else:
    #         # content
    #         pass

    #     return response

    # def self_level_enable_callback(self, request, response):
    #     if request.data:
    #         # content
    #         pass
    #     else:
    #         # content
    #         pass

    #     return response

    # def estop_callback(self, msg):
    #     if msg.data:
    #         # content
    #         pass
    #     else:
    #         # content
    #         pass


def main(args=None):
    rclpy.init(args=args)
    node = MEBotControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
