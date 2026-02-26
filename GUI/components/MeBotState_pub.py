from re import match
from unittest import case
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

class MeBotStateNode(Node):
    def __init__(self,UI):
        super().__init__("MeBotState_pub") 
    
        self.UI = UI    #to access mode and action and carriage state values in UserWindow.py 

        #variables
        self.mode = "neutral"
        self.action = "none"
        self.carriage_state = "Front-Wheel Drive"
        self.CA_flag = 1

        #pubs/subs/timers
        self.mode_pub = self.create_publisher(String, 'MeBot_mode', 10) 
        self.mode_timer =self.create_timer(1.0,self.publish_mode)

        self.action_pub = self.create_publisher(String, 'MeBot_action', 10)
        self.action_timer =self.create_timer(1.0,self.publish_action)   

        # self.carriage_state_pub = self.create_publisher(String, 'MeBot_carriage_state', 10)
        # self.carriage_state_timer =self.create_timer(1.0,self.publish_carriage_state)   

        self.CA_sub = self.create_subscription(Int64, 'CA_flag', self.update_CA_flag, 10) #subscribe to CA_flag topic published by sensor_data_pub
        self.CA_pub = self.create_publisher(String, 'MeBot_CA_state', 10)
        self.CA_timer =self.create_timer(1.0,self.publish_CA_state)
        
    def update_CA_flag(self, msg):  #called from sensor_data_pub to update CA_flag value
        self.CA_flag = msg.data

    def publish_CA_state(self):
        msg = String()
        match (self.CA_flag):
            case 0:
                msg.data = "Reseting/Complete" 
                self.CA_pub.publish(msg)
            case 1:
                msg.data = "Initial/Ready" 
                self.CA_pub.publish(msg)
            case 2:         
                msg.data = "Driving forward until curb hit" #default state
                self.CA_pub.publish(msg)    
            case 3:
                msg.data = "Pushing carriage onto curb" 
                self.CA_pub.publish(msg)
            case 4:
                msg.data = "Raising main wheels" 
                self.CA_pub.publish(msg)
            case 5:
                msg.data = "Moving driving wheels and carriage onto curb" 
                self.CA_pub.publish(msg)
            case 6:
                msg.data = "Moving carriage back to edge of curb"
                self.CA_pub.publish(msg)
            case 7:
                msg.data = "Raising rear casters"  
                self.CA_pub.publish(msg)
            case 8:
                msg.data = "Moving both carriages back to original position" 
                self.CA_pub.publish(msg)
            case 9:
                msg.data = "Raising front casters and stopping all movement" 
                self.CA_pub.publish(msg)

    def publish_mode(self):
        msg = String()
        msg.data = self.UI.mode.value
        self.mode_pub.publish(msg)
    
    def publish_action(self):
        msg = String()
        msg.data = self.UI.action.value
        self.action_pub.publish(msg)

    # def publish_carriage_state(self):
    #     msg = String()
    #     msg.data = self.UI.carriage_state.value
    #     self.carriage_state_pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MeBotStateNode() 
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()