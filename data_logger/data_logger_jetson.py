import rclpy
from rclpy.node import Node
import csv
import os.path
import serial
import ast
from datetime import datetime
from std_msgs.msg import Float64


class dataLoggerPub(Node):
    def __init__(self):
        super().__init__("dataLogger_pub")

        # serial init
        self.ser = serial.Serial(
            port="/dev/ttyACM0",  # USB connection
            baudrate=115200,
            timeout=1,
        )

        # timer for serial data reading
        self.serial_timer = self.create_timer(1.0, self.read_serial_data)

        # variables
        self.appTime = 0.0
        self.speed = 0.0
        self.acceleration = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.seat_angle_pitch = 0.0
        self.seat_angle_roll = 0.0
        self.tilt = 0.0
        self.measure_height = 0.0

        # publishers and timers
        self.appTime_publisher = self.create_publisher(Float64, "app_time", 10)
        self.appTime_timer = self.create_timer(1.0, self.publish_appTime)

        self.speed_publisher = self.create_publisher(Float64, "chair_speed", 10)
        self.speed_timer = self.create_timer(1.0, self.publish_speed)

        self.acceleration_publisher = self.create_publisher(
            Float64, "chair_acceleration", 10
        )
        self.acceleration_timer = self.create_timer(1.0, self.publish_acceleration)

        self.accel_x_publisher = self.create_publisher(Float64, "accelerometer_x", 10)
        self.accel_x_timer = self.create_timer(1.0, self.publish_accel_x)

        self.accel_y_publisher = self.create_publisher(Float64, "accelerometer_y", 10)
        self.accel_y_timer = self.create_timer(1.0, self.publish_accel_y)

        self.accel_z_publisher = self.create_publisher(Float64, "accelerometer_z", 10)
        self.accel_z_timer = self.create_timer(1.0, self.publish_accel_z)

        self.seat_angle_pitch_publisher = self.create_publisher(
            Float64, "seat_angle_pitch", 10
        )
        self.seat_angle_pitch_timer = self.create_timer(
            1.0, self.publish_seat_angle_pitch
        )

        self.seat_angle_roll_publisher = self.create_publisher(
            Float64, "seat_angle_roll", 10
        )
        self.seat_angle_roll_timer = self.create_timer(
            1.0, self.publish_seat_angle_roll
        )

        self.tilt_publisher = self.create_publisher(Float64, "tilt", 10)
        self.tilt_timer = self.create_timer(1.0, self.publish_tilt)

        self.measure_height_publisher = self.create_publisher(
            Float64, "measure_height", 10
        )
        self.measure_height_timer = self.create_timer(1.0, self.publish_measure_height)

    # write data to csv
    def write_data(self, data):
        file_exists = os.path.isfile("fake_logging_records.csv")
        headers = [
            "Timestamp",
            "Time to complete command (s)",  # maybe default to 0 if not used, if used, first number is which command being timed
            "Chair speed (m/s)",
            "Chair acceleration (m/s^2)",
            "Accelerometer x (m/s^2)",
            "Accelerometer y (m/s^2)",
            "Accelerometer z (m/s^2)",
            "Seat angle pitch (degrees)",
            "Seat angle roll (degrees)",
            "Tilt (degrees)",
            "Measure height (m)",
        ]

        with open("fake_logging_records.csv", "a", newline="") as csvfile:
            writer = csv.writer(csvfile)

            if not file_exists:
                writer.writerow(headers)

            writer.writerow(data)

    # reading incoming serial data from teensy
    def read_serial_data(self):
        line = self.ser.readline()
        if line:
            raw_data = line.decode("utf-8", errors="replace").strip()
            data = ast.literal_eval(raw_data)
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S %p")
            data.insert(0, timestamp)
            # may include another insert for type of command being timed
            self.update_data(data[1:])  # Exclude timestamp for updating variables
            self.write_data(data)

    # update variables to be published
    def update_data(self, data):
        self.appTime = data[0]
        self.speed = data[1]
        self.acceleration = data[2]
        self.accel_x = data[3]
        self.accel_y = data[4]
        self.accel_z = data[5]
        self.seat_angle_pitch = data[6]
        self.seat_angle_roll = data[7]
        self.tilt = data[8]
        self.measure_height = data[9]

    def publish_appTime(self):
        msg = Float64()
        msg.data = float(self.appTime)
        self.appTime_publisher.publish(msg)

    def publish_speed(self):
        msg = Float64()
        msg.data = float(self.speed)
        self.speed_publisher.publish(msg)

    def publish_acceleration(self):
        msg = Float64()
        msg.data = float(self.acceleration)
        self.acceleration_publisher.publish(msg)

    def publish_accel_x(self):
        msg = Float64()
        msg.data = float(self.accel_x)
        self.accel_x_publisher.publish(msg)

    def publish_accel_y(self):
        msg = Float64()
        msg.data = float(self.accel_y)
        self.accel_y_publisher.publish(msg)

    def publish_accel_z(self):
        msg = Float64()
        msg.data = float(self.accel_z)
        self.accel_z_publisher.publish(msg)

    def publish_seat_angle_pitch(self):
        msg = Float64()
        msg.data = float(self.seat_angle_pitch)
        self.seat_angle_pitch_publisher.publish(msg)

    def publish_seat_angle_roll(self):
        msg = Float64()
        msg.data = float(self.seat_angle_roll)
        self.seat_angle_roll_publisher.publish(msg)

    def publish_tilt(self):
        msg = Float64()
        msg.data = float(self.tilt)
        self.tilt_publisher.publish(msg)

    def publish_measure_height(self):
        msg = Float64()
        msg.data = float(self.measure_height)
        self.measure_height_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = dataLoggerPub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
