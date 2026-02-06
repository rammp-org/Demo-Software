import Jetson.GPIO as GPIO
from PyQt5.QtCore import pyqtSignal, QThread
import time
import traceback

class JoystickThread(QThread):
    forward_signal = pyqtSignal()
    reverse_signal = pyqtSignal()
    left_signal = pyqtSignal()
    right_signal = pyqtSignal()
    neutral_signal = pyqtSignal()
    
    def __init__(self, parent=None):
        super(JoystickThread, self).__init__(parent)
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        self.joystick_pins = {
            'Forward': 20,
            'Reverse': 17,
            'Left': 27,
            'Right': 18
        }

        GPIO.setup(list(self.joystick_pins.values()), GPIO.IN)

        # Initialize variables to track joystick state
        self.prev_joystick_state = {
            'Forward': GPIO.input(self.joystick_pins['Forward']),
            'Reverse': GPIO.input(self.joystick_pins['Reverse']),
            'Left': GPIO.input(self.joystick_pins['Left']),
            'Right': GPIO.input(self.joystick_pins['Right'])
        }

    def run(self):
        try:
            while not self.isInterruptionRequested():
                forward_state = GPIO.input(self.joystick_pins['Forward'])
                reverse_state = GPIO.input(self.joystick_pins['Reverse'])
                left_state = GPIO.input(self.joystick_pins['Left'])
                right_state = GPIO.input(self.joystick_pins['Right'])

                # Check if joystick state has changed
                if (forward_state != self.prev_joystick_state['Forward'] or
                    reverse_state != self.prev_joystick_state['Reverse'] or
                    left_state != self.prev_joystick_state['Left'] or
                    right_state != self.prev_joystick_state['Right']):

                    # Update previous joystick state
                    self.prev_joystick_state['Forward'] = forward_state
                    self.prev_joystick_state['Reverse'] = reverse_state
                    self.prev_joystick_state['Left'] = left_state
                    self.prev_joystick_state['Right'] = right_state
                
                    if forward_state == GPIO.HIGH:
                        self.forward_signal.emit()
                    elif reverse_state == GPIO.HIGH:
                        self.reverse_signal.emit()
                    elif left_state == GPIO.HIGH:
                        print("Emitting Left Signal")
                        self.left_signal.emit()
                    elif right_state == GPIO.HIGH:
                        print("Emitting Right Signal")
                        self.right_signal.emit()
                    else:
                        self.neutral_signal.emit() # Emit signal for neutral state

                # Sleep to avoid high CPU usage
                time.sleep(0.1)
        except KeyboardInterrupt:
            # Handle Keyboard interrupt (Ctrl+C)
            self.interrupt()
        except Exception as e:
            # Handle other exceptions
            print("Error in JoystickThread:", e)
            traceback.print_exc()
        finally:
            # Clean up GPIO resources
            GPIO.cleanup()