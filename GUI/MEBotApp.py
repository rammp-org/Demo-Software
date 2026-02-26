import sys
import serial
from GUI.components.MeBotState_pub import MeBotStateNode
from GUI.components.csv_logger import csvLoggerNode
from PyQt5.QtWidgets import QMainWindow, QStackedWidget, QApplication
from components.TeensyController import TeensyController
from components.DataLogger import DataLogger
from windows.UserWindow import UserWindow
from windows.DevWindow import DevWindow
from MeBotState_pub import MeBotStateNode
import rclpy
            

class MEBotApp(QMainWindow):
    def __init__(self):

        super().__init__()

        # initialize ROS
        rclpy.init()
       
        # Serial communication setup
        # /dev/rfcomm0 should be used to initialize serial if want to use wireless mode (Bluetooth)
        # /dev/ttyACM0 should be used to initialize serial if want to use wired mode (USB)
        try:
            # self.ser = serial.Serial('/dev/rfcomm0', 115200) # Would also try 38400
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200)
        except:
            sys.exit("\n \
                    Please check which port path HC-05 is connected to. \n \
                    And make sure it matches the port path in MEBotApp.py file.")
        
        # Create QStackedWidget
        self.stacked_widget = QStackedWidget()
        self.teensy_controller = TeensyController(self.serial_port)
        # self.data_logger = DataLogger(self.serial_port)
        self.data_logger = csvLoggerNode()  # Initialize the ROS2 node for CSV logging
        self.MeBotStateNode = MeBotStateNode(self.teensy_controller)  # Initialize the ROS2 node for publishing MeBot state

        # Create and add screens
        self.screen0 = UserWindow(self.stacked_widget, self.teensy_controller, self.data_logger)
        self.screen1 = DevWindow(self.stacked_widget, self.teensy_controller)
        
        self.stacked_widget.addWidget(self.screen0)
        self.stacked_widget.addWidget(self.screen1)
        
        # Set the initial screen
        self.stacked_widget.setCurrentIndex(0)
        
        self.setCentralWidget(self.stacked_widget)
        self.setWindowTitle("MEBot App")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MEBotApp()
    window.showFullScreen()
    # window.show()

    # spin ROS node
    rclpy.spin(window.MeBotStateNode)

    # cleanup
    window.MeBotStateNode.destroy_node()
    rclpy.shutdown()

    sys.exit(app.exec_())