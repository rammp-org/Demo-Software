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
        #publishers and timers for sensor and state data
        self.IMU_pitch_publisher = self.create_publisher(Float64, 'IMU_pitch', 10)
        self.IMU_pitch_timer = self.create_timer(self.publish_rate, self.publish_IMU_pitch)

        self.IMU_roll_publisher = self.create_publisher(Float64, 'IMU_roll', 10)
        self.IMU_roll_timer = self.create_timer(self.publish_rate, self.publish_IMU_roll)        
        
        self.accel_x_publisher = self.create_publisher(Float64, 'accelerometer_x', 10)
        self.accel_x_timer = self.create_timer(self.publish_rate, self.publish_accel_x)

        self.accel_y_publisher = self.create_publisher(Float64, 'accelerometer_y', 10)
        self.accel_y_timer = self.create_timer(self.publish_rate, self.publish_accel_y)

        self.accel_z_publisher = self.create_publisher(Float64, 'accelerometer_z', 10)
        self.accel_z_timer = self.create_timer(self.publish_rate, self.publish_accel_z)        
        
        self.FC_pos_publisher = self.create_publisher(Float64, 'FC_pos', 10)
        self.FC_pos_timer = self.create_timer(self.publish_rate, self.publish_FC_pos)

        self.RC_pos_publisher = self.create_publisher(Float64, 'RC_pos', 10)
        self.RC_pos_timer = self.create_timer(self.publish_rate, self.publish_RC_pos)

        self.MR_pos_publisher = self.create_publisher(Float64, 'MR_pos', 10)
        self.MR_pos_timer = self.create_timer(self.publish_rate, self.publish_MR_pos)

        self.ML_pos_publisher = self.create_publisher(Float64, 'ML_pos', 10)
        self.ML_pos_timer = self.create_timer(self.publish_rate, self.publish_ML_pos)
        
        self.ML_carriage_pos_publisher = self.create_publisher(Float64, 'ML_carriage_pos', 10)
        self.ML_carriage_pos_timer = self.create_timer(self.publish_rate, self.publish_ML_carriage_pos)

        self.MR_carriage_pos_publisher = self.create_publisher(Float64, 'MR_carriage_pos', 10)
        self.MR_carriage_pos_timer = self.create_timer(self.publish_rate, self.publish_MR_carriage_pos)        
        
        self.FC_loadcell_publisher = self.create_publisher(Float64, 'FC_loadcell', 10)  
        self.FC_loadcell_timer = self.create_timer(self.publish_rate, self.publish_FC_loadcell)   

        self.MR_loadcell_publisher = self.create_publisher(Float64, 'MR_loadcell', 10)      
        self.MR_loadcell_timer = self.create_timer(self.publish_rate, self.publish_MR_loadcell)   

        self.ML_loadcell_publisher = self.create_publisher(Float64, 'ML_loadcell', 10)  
        self.ML_loadcell_timer = self.create_timer(self.publish_rate, self.publish_ML_loadcell)  

        self.ML_wheel_pos_publisher = self.create_publisher(Float64, 'ML_wheel_pos', 10)
        self.ML_wheel_pos_timer = self.create_timer(self.publish_rate, self.publish_ML_wheel_pos)

        self.MR_wheel_pos_publisher = self.create_publisher(Float64, 'MR_wheel_pos', 10)
        self.MR_wheel_pos_timer = self.create_timer(self.publish_rate, self.publish_MR_wheel_pos) 

        self.CA_flag_publisher = self.create_publisher(Int64, 'CA_flag', 10)
        self.CA_flag_timer = self.create_timer(self.publish_rate, self.publish_CA_flag)

        self.action_publisher = self.create_publisher(String, 'action', 10)
        self.action_timer = self.create_timer(self.publish_rate, self.publish_action)    

        self.appTime_publisher = self.create_publisher(Float64, 'app_time', 10)
        self.appTime_timer = self.create_timer(self.publish_rate, self.publish_appTime)

        self.speed_ML_publisher = self.create_publisher(Float64, 'chair_speed_ML', 10)
        self.speed_ML_timer = self.create_timer(self.publish_rate, self.publish_speed_ML)

        self.speed_MR_publisher = self.create_publisher(Float64, 'chair_speed_MR', 10)
        self.speed_MR_timer = self.create_timer(self.publish_rate, self.publish_speed_MR)

        self.acceleration_ML_publisher = self.create_publisher(Float64, 'chair_acceleration_ML', 10)
        self.acceleration_ML_timer = self.create_timer(self.publish_rate, self.publish_acceleration_ML)

        self.acceleration_MR_publisher = self.create_publisher(Float64, 'chair_acceleration_MR', 10)
        self.acceleration_MR_timer = self.create_timer(self.publish_rate, self.publish_acceleration_MR)

        self.tilt_publisher = self.create_publisher(Float64, 'tilt', 10)
        self.tilt_timer = self.create_timer(self.publish_rate, self.publish_tilt)

        self.measure_height_publisher = self.create_publisher(Float64, 'measure_height', 10)
        self.measure_height_timer = self.create_timer(self.publish_rate, self.publish_measure_height)

        self.imu_publisher = self.create_publisher(Imu, 'imu_data', 10)
        self.imu_timer = self.create_timer(self.publish_rate, self.publish_imu_data)


    #reading incoming serial data from teensy
    def read_serial_data(self):
        line = self.ser.readline()
        if line:
            raw_data = line.decode("utf-8", errors="replace").strip()
            if (raw_data.startswith('[') and raw_data.endswith(']')):  #check if data is in expected list format
                data = ast.literal_eval(raw_data)

                self.logger.info(f"Received data: {data}")  # Log the received data

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
        
    def publish_appTime(self):
        msg = Float64()
        msg.data = float(self.appTime) 
        self.appTime_publisher.publish(msg)    

    def publish_speed_ML(self):
        msg = Float64()
        msg.data = float(self.current_speed_ML) 
        self.speed_ML_publisher.publish(msg)

    def publish_speed_MR(self):
        msg = Float64()
        msg.data = float(self.current_speed_MR) 
        self.speed_MR_publisher.publish(msg)

    def publish_acceleration_ML(self):
        msg = Float64()
        msg.data = float(self.current_speed_ML - self.prev_speed_ML)  #calculate acceleration using change in speed over time (0.1s between serial data updates)
        self.acceleration_ML_publisher.publish(msg)

    def publish_acceleration_MR(self):
        msg = Float64()
        msg.data = float(self.current_speed_MR - self.prev_speed_MR)  #calculate acceleration using change in speed over time (0.1s between serial data updates)
        self.acceleration_MR_publisher.publish(msg)

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

    def publish_tilt(self):
        msg = Float64()
        self.tilt =math.acos(math.cos(self.IMU_pitch)*math.cos(self.IMU_roll)) * (180/math.pi)  #calculate tilt in degrees using pitch and roll
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
    
    def publish_ML_carriage_pos(self):
        msg = Float64()
        msg.data = float(self.ML_carriage_pos) 
        self.ML_carriage_pos_publisher.publish(msg)
    
    def publish_MR_carriage_pos(self):
        msg = Float64()
        msg.data = float(self.MR_carriage_pos) 
        self.MR_carriage_pos_publisher.publish(msg)

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
    
    def publish_action(self):
        msg = String()
        msg.data = str(self.action) 
        self.action_publisher.publish(msg)

    def publish_imu_data(self):
        msg = Imu()
        #populate Imu message fields with appropriate data
        msg.linear_acceleration.x = self.accel_x
        msg.linear_acceleration.y = self.accel_y
        msg.linear_acceleration.z = self.accel_z

        #convert IMU angles from degrees to radians for orientation fields
        pitch = math.radians(self.IMU_pitch)
        roll = math.radians(self.IMU_roll)
        yaw = 0.0  #assuming yaw is 0 since it is not measured by the IMU

        #populate orientation fields using Euler angles (assuming yaw is 0)
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        self.imu_publisher.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = MEBotControlNodeTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()