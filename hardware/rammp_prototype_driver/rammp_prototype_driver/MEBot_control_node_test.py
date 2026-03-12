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
from sensor_msgs.msg import Imu
import math
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

class MEBotControlNodeTest(Node): 
    def __init__(self):
        super().__init__("MEBot_control_node_test") 

        #serial init
        self.ser =  serial.Serial(
                    port ='/dev/ttyACM0',  #USB connection
                    baudrate=115200,
                    timeout=1)
        
        #timer for serial data reading 
        self.serial_timer = self.create_timer(.001, self.read_serial_data)

        #publishing rate for all topics
        self.publish_rate = .001  

        #IMU
        self.IMU_pitch = 0.0
        self.IMU_roll = 0.0       
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        #Encoders
        self.FC_pos = 0.0
        self.RC_pos = 0.0
        self.MR_pos = 0.0
        self.ML_pos = 0.0
        self.ML_carriage_pos = 0.0
        self.MR_carriage_pos = 0.0
        self.ML_wheel_pos = 0.0
        self.MR_wheel_pos = 0.0

        #Loadcells
        self.FC_loadcell = 0.0
        self.MR_loadcell = 0.0
        self.ML_loadcell = 0.0        
        
        #CA_flag and action
        self.CA_flag = 0.0
        self.action = "z"  #variable to store most recent action command Base.ino recieved

        #app time
        self.appTime = 0.0

        #velocity and acceleration
        self.prev_speed_ML = 0.0
        self.current_speed_ML = 0.0

        self.prev_speed_MR = 0.0
        self.current_speed_MR = 0.0

        self.acceleration_ML = 0.0
        self.acceleration_MR = 0.0

        #tilt and measure height
        self.tilt = 0.0
        self.measure_height = 0.0

    #reading incoming serial data from teensy
    def read_serial_data(self):
        line = self.ser.readline()
        if line:
            raw_data = line.decode("utf-8", errors="replace").strip()
            if (raw_data.startswith('[') and raw_data.endswith(']')):  #check if data is in expected list format
                data = ast.literal_eval(raw_data)
                self.update_data(data)  # Update variables with new data
            if (raw_data.startswith('Action:')):  #check if data is an action command from Base.ino
                self.action = raw_data.split('Action:')[1].strip()[0]
    
    #update variables to be published        
    def update_data(self, data):
        #IMU
        self.IMU_pitch = data[0]
        self.IMU_roll = data[1]
        self.accel_x = data[2]
        self.accel_y = data[3]
        self.accel_z = data[4]

        #Encoders
        self.FC_pos = data[5]
        self.RC_pos = data[6]
        self.MR_pos = data[7]
        self.ML_pos = data[8]
        self.ML_carriage_pos = data[9]
        self.MR_carriage_pos = data[10]
        self.ML_wheel_pos = data[11]
        self.MR_wheel_pos = data[12]

        #loadcells 
        self.FC_loadcell = data[13]
        self.MR_loadcell = data[14]
        self.ML_loadcell = data[15]

        #CA_flag
        self.CA_flag = data[16]

        #Apptime
        self.appTime = data[17]

        #velocity
        self.prev_speed_ML = self.current_speed_ML
        self.current_speed_ML = data[18]
        self.prev_speed_MR = self.current_speed_MR
        self.current_speed_MR = data[19]

def main(args=None):
    rclpy.init(args=args)
    node = MEBotControlNodeTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()