import rclpy
from rclpy.node import Node
import csv
import os.path
import serial
import time 
import ast
from datetime import datetime
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int64
import csv_logger
import MeBotState_pub
import math

class dataPub(Node): 
    def __init__(self, MeBot_logger, State_pub):
        super().__init__("data_pub") 

        #serial init
        self.ser =  serial.Serial(
                    port ='/dev/ttyACM0',  #USB connection
                    baudrate=115200,
                    timeout=1)
        
        #timer for serial data reading 
        self.serial_timer = self.create_timer(1.0, self.read_serial_data)

        #variables for publishing and csv logging
        self.appTime = 0.0
        self.prev_speed = 0.0
        self.current_speed = 0.0
        self.acceleration = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.seat_angle_pitch = 0.0
        self.seat_angle_roll = 0.0
        self.tilt = 0.0
        self.measure_height = 0.0

        #variables for only publishing 
        self.IMU_pitch = 0.0
        self.IMU_roll = 0.0

        self.FC_pos = 0.0
        self.RC_pos = 0.0
        self.MR_pos = 0.0
        self.ML_pos = 0.0

        self.FC_loadcell = 0.0
        self.MR_loadcell = 0.0
        self.ML_loadcell = 0.0

        self.ML_wheel_pos = 0.0
        self.MR_wheel_pos = 0.0

        self.CA_flag = 0.0

        #publishers and timers
        self.appTime_publisher = self.create_publisher(Float64, 'app_time', 10)
        self.appTime_timer = self.create_timer(1.0, self.publish_appTime)

        self.speed_publisher = self.create_publisher(Float64, 'chair_speed', 10)
        self.speed_timer = self.create_timer(1.0, self.publish_speed)

        self.acceleration_publisher = self.create_publisher(Float64, 'chair_acceleration', 10)
        self.acceleration_timer = self.create_timer(1.0, self.publish_acceleration)

        self.accel_x_publisher = self.create_publisher(Float64, 'accelerometer_x', 10)
        self.accel_x_timer = self.create_timer(1.0, self.publish_accel_x)

        self.accel_y_publisher = self.create_publisher(Float64, 'accelerometer_y', 10)
        self.accel_y_timer = self.create_timer(1.0, self.publish_accel_y)

        self.accel_z_publisher = self.create_publisher(Float64, 'accelerometer_z', 10)
        self.accel_z_timer = self.create_timer(1.0, self.publish_accel_z)

        self.seat_angle_pitch_publisher = self.create_publisher(Float64, 'seat_angle_pitch', 10)
        self.seat_angle_pitch_timer = self.create_timer(1.0, self.publish_seat_angle_pitch)

        self.seat_angle_roll_publisher = self.create_publisher(Float64, 'seat_angle_roll', 10)
        self.seat_angle_roll_timer = self.create_timer(1.0, self.publish_seat_angle_roll)

        self.tilt_publisher = self.create_publisher(Float64, 'tilt', 10)
        self.tilt_timer = self.create_timer(1.0, self.publish_tilt)

        self.measure_height_publisher = self.create_publisher(Float64, 'measure_height', 10)
        self.measure_height_timer = self.create_timer(1.0, self.publish_measure_height)

        self.IMU_pitch_publisher = self.create_publisher(Float64, 'IMU_pitch', 10)
        self.IMU_pitch_timer = self.create_timer(1.0, self.publish_IMU_pitch)

        self.IMU_roll_publisher = self.create_publisher(Float64, 'IMU_roll', 10)
        self.IMU_roll_timer = self.create_timer(1.0, self.publish_IMU_roll)

        self.FC_pos_publisher = self.create_publisher(Float64, 'FC_pos', 10)
        self.FC_pos_timer = self.create_timer(1.0, self.publish_FC_pos)

        self.RC_pos_publisher = self.create_publisher(Float64, 'RC_pos', 10)
        self.RC_pos_timer = self.create_timer(1.0, self.publish_RC_pos)

        self.MR_pos_publisher = self.create_publisher(Float64, 'MR_pos', 10)
        self.MR_pos_timer = self.create_timer(1.0, self.publish_MR_pos)

        self.ML_pos_publisher = self.create_publisher(Float64, 'ML_pos', 10)
        self.ML_pos_timer = self.create_timer(1.0, self.publish_ML_pos)

        self.FC_loadcell_publisher = self.create_publisher(Float64, 'FC_loadcell', 10)  
        self.FC_loadcell_timer = self.create_timer(1.0, self.publish_FC_loadcell)   

        self.MR_loadcell_publisher = self.create_publisher(Float64, 'MR_loadcell', 10)      
        self.MR_loadcell_timer = self.create_timer(1.0, self.publish_MR_loadcell)   

        self.ML_loadcell_publisher = self.create_publisher(Float64, 'ML_loadcell', 10)  
        self.ML_loadcell_timer = self.create_timer(1.0, self.publish_ML_loadcell)   

        self.ML_wheel_pos_publisher = self.create_publisher(Float64, 'ML_wheel_pos', 10)
        self.ML_wheel_pos_timer = self.create_timer(1.0, self.publish_ML_wheel_pos)

        self.MR_wheel_pos_publisher = self.create_publisher(Float64, 'MR_wheel_pos', 10)
        self.MR_wheel_pos_timer = self.create_timer(1.0, self.publish_MR_wheel_pos)

        self.CA_flag_publisher = self.create_publisher(String, 'CA_flag', 10)
        self.CA_flag_timer = self.create_timer(1.0, self.publish_CA_flag)

    #reading incoming serial data from teensy
    def read_serial_data(self):
        line = self.ser.readline()
        if line:
            raw_data = line.decode("utf-8", errors="replace").strip()
            data = ast.literal_eval(raw_data)
            self.update_data(data)  # Update variables with new data
                

    #update variables to be published        
    def update_data(self, data):
        #IMU
        self.seat_angle_pitch = data[0]
        self.seat_angle_roll = data[1]
        self.accel_x = data[2]
        self.accel_y = data[3]
        self.accel_z = data[4]

        #Encoders
        self.FC_pos = data[5]
        self.RC_pos = data[6]
        self.MR_pos = data[7]
        self.ML_pos = data[8]


        #loadcells 
        self.FC_loadcell = data[9]
        self.MR_loadcell = data[10]
        self.ML_loadcell = data[11]

        #wheel positions
        self.ML_wheel_pos = data[12]
        self.MR_wheel_pos = data[13]

        #CA_flag
        self.CA_flag = data[14]

        #Apptime
        self.appTime = data[15]

        #velocity
        self.prev_speed = self.current_speed
        self.current_speed = data[16]
         
    def publish_appTime(self):
        msg = Float64()
        msg.data = float(self.appTime) 
        self.appTime_publisher.publish(msg)    

    def publish_speed(self):
        msg = Float64()
        msg.data = float(self.current_speed) 
        self.speed_publisher.publish(msg)

    def publish_acceleration(self):
        msg = Float64()
        msg.data = float(self.current_speed - self.prev_speed)  #calculate acceleration using change in speed over time (0.1s between serial data updates)
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
        self.tilt =math.acos(math.cos(self.seat_angle_pitch)*math.cos(self.seat_angle_roll)) * (180/math.pi)  #calculate tilt in degrees using pitch and roll
        msg.data = float(self.tilt) 
        self.tilt_publisher.publish(msg)
    
    def publish_measure_height(self):
        msg = Float64()
        msg.data = float(self.measure_height)    
        self.measure_height_publisher.publish(msg)

    def publish_IMU_pitch(self):
        msg = Float64()
        msg.data = float(self.IMU_pitch) 
        self.IMU_pitch_publisher.publish(msg)

    def publish_IMU_roll(self):
        msg = Float64()
        msg.data = float(self.IMU_roll) 
        self.IMU_roll_publisher.publish(msg)

    def publish_FC_pos(self):
        msg = Float64()
        msg.data = float(self.FC_pos) 
        self.FC_pos_publisher.publish(msg)

    def publish_RC_pos(self):
        msg = Float64()
        msg.data = float(self.RC_pos) 
        self.RC_pos_publisher.publish(msg)      

    def publish_MR_pos(self):
        msg = Float64()
        msg.data = float(self.MR_pos) 
        self.MR_pos_publisher.publish(msg)  

    def publish_ML_pos(self):   
        msg = Float64()
        msg.data = float(self.ML_pos) 
        self.ML_pos_publisher.publish(msg)
    
    def publish_FC_loadcell(self):      
        msg = Float64()
        msg.data = float(self.FC_loadcell) 
        self.FC_loadcell_publisher.publish(msg)

    def publish_MR_loadcell(self):
        msg = Float64()
        msg.data = float(self.MR_loadcell) 
        self.MR_loadcell_publisher.publish(msg)

    def publish_ML_loadcell(self):
        msg = Float64()
        msg.data = float(self.ML_loadcell) 
        self.ML_loadcell_publisher.publish(msg)
    
    def publish_ML_wheel_pos(self):
        msg = Float64()
        msg.data = float(self.ML_wheel_pos) 
        self.ML_wheel_pos_publisher.publish(msg)

    def publish_MR_wheel_pos(self):
        msg = Float64()
        msg.data = float(self.MR_wheel_pos) 
        self.MR_wheel_pos_publisher.publish(msg)

    def publish_CA_flag(self):
        msg = Int64()
        msg.data = int(self.CA_flag) 
        self.CA_flag_publisher.publish(msg)
   

def main(args=None):
    rclpy.init(args=args)
    node = dataPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
