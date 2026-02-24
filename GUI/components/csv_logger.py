from datetime import datetime
import rclpy
from rclpy.node import Node
import os.path
import csv
from std_msgs.msg import Float64

class csvLoggerNode(Node):
    def __init__(self):
        super().__init__("csv_logger_node")
        
        self.logging = False
        self.filename = "sensor_dataLogging.csv"
        self.writer = None
        self.csvfile = None
        self.headers = [ 'Timestamp',
                    'Time to complete command (s)', #maybe default to 0 if not used, if used, first number is which command being timed
                    'Chair speed ML (m/s)',
                    'Chair speed MR (m/s)',
                    'Chair acceleration ML (m/s^2)',
                    'Chair acceleration MR (m/s^2)',
                    'Accelerometer x (m/s^2)',
                    'Accelerometer y (m/s^2)',
                    'Accelerometer z (m/s^2)',
                    'Seat angle pitch (degrees)',
                    'Seat angle roll (degrees)',
                    'Tilt (degrees)',
                    'Measure height (m)']

        #subscriptions to sensor data topics
        self.appTime_sub = self.create_subscription(Float64, 'app_time', self.AppTime_callback, 10)
        self.speed_ML_sub = self.create_subscription(Float64, 'chair_speed_ML', self.speed_ML_callback, 10)
        self.speed_MR_sub = self.create_subscription(Float64, 'chair_speed_MR', self.speed_MR_callback, 10)
        self.acceleration_ML_sub = self.create_subscription(Float64, 'chair_acceleration_ML', self.acceleration_ML_callback, 10)
        self.acceleration_MR_sub = self.create_subscription(Float64, 'chair_acceleration_MR', self.acceleration_MR_callback, 10)
        self.accel_x_sub = self.create_subscription(Float64, 'accelerometer_x', self.accel_x_callback, 10)
        self.accel_y_sub = self.create_subscription(Float64, 'accelerometer_y', self.accel_y_callback, 10)
        self.accel_z_sub = self.create_subscription(Float64, 'accelerometer_z', self.accel_z_callback, 10)
        self.seat_angle_pitch_sub = self.create_subscription(Float64, 'seat_angle_pitch', self.seat_angle_pitch_callback, 10)
        self.seat_angle_roll_sub = self.create_subscription(Float64, 'seat_angle_roll', self.seat_angle_roll_callback, 10)
        self.tilt_sub = self.create_subscription(Float64, 'tilt', self.tilt_callback, 10)
        self.measure_height_sub = self.create_subscription(Float64, 'measure_height', self.measure_height_callback, 10)

        #variables to store data for logging, updated by callbacks
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

    #user window functions 
    def start(self):
        if self.logging:
            return  # Already logging, do nothing
        
        file_exists = os.path.isfile(self.filename)

        # open file
        self.csvfile = open(self.filename, 'a', newline='', buffering=1)  # Open in append mode with line buffering
        self.writer = csv.writer(self.csvfile)

        if not file_exists:
            self.writer.writerow(self.headers)
            self.csvfile.flush()

        self.logging = True

    def stop(self):
        self.logging = False
        if self.csvfile and not self.csvfile.closed:
            self.csvfile.flush()
            self.csvfile.close()
        self.writer = None
        self.csvfile = None
    
    def log(self):
        if not self.logging or self.writer is None:
            return
        
        timestamp = datetime.now().strftime("%Y-%m-%d %I:%M:%S %p")
        # timestamp = self.get_clock().now().nanoseconds / 1e9 #don't know yet if I have to use ros time or can just use datetime, but this is how to get ros time in seconds
        data = [timestamp, self.appTime, self.speed_ML, self.speed_MR, self.acceleration_ML, self.acceleration_MR, self.accel_x, self.accel_y, self.accel_z, self.seat_angle_pitch, self.seat_angle_roll, self.tilt, self.measure_height]
        self.writer.writerow(data)

    def AppTime_callback(self, msg):
        self.appTime = msg.data
    def speed_ML_callback(self, msg):
        self.speed_ML = msg.data
    def speed_MR_callback(self, msg):
        self.speed_MR = msg.data
    def acceleration_ML_callback(self, msg):
        self.acceleration_ML = msg.data
    def acceleration_MR_callback(self, msg):
        self.acceleration_MR = msg.data
    def accel_x_callback(self, msg):
        self.accel_x = msg.data
    def accel_y_callback(self, msg):
        self.accel_y = msg.data
    def accel_z_callback(self, msg):
        self.accel_z = msg.data
    def seat_angle_pitch_callback(self, msg):
        self.seat_angle_pitch = msg.data
    def seat_angle_roll_callback(self, msg):
        self.seat_angle_roll = msg.data 
    def tilt_callback(self, msg):
        self.tilt = msg.data
    def measure_height_callback(self, msg): 
        self.measure_height = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = csvLoggerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()