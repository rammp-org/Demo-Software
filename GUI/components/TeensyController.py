import traceback
from enum import Enum


class Modes(Enum):
    MANUAL = "1"
    NORMAL = "2"
    SELF_LEVELING = "3"
    CARRIAGE = "5"
    BUTTON = "6"


class Actions(Enum):
    NEUTRAL = "z"
    RAISE_UP = "q"
    LOWER_DOWN = "a"
    TILT_FORWARD = "s"
    TILT_BACKWARD = "w"
    TILT_LEFT = "r"
    TILT_RIGHT = "f"
    CARRIAGE_FORWARD = "u"
    CARRIAGE_BACKWARD = "j"
    RESET = "r"
    CURB_ASCEND = "c"
    CURB_DESCEND = "d"


class TeensyController:
    def __init__(self, serial_port):
        self.ser = serial_port
        self.mode = Modes.NORMAL
        self.action = Actions.NEUTRAL

    def send_mode_action(self):
        # send the mode to Teensy
        self.ser.write((f"{self.mode.value}\n").encode())
        # send the action to Teensy
        self.ser.write((f"{self.action.value}\n").encode())

    def send_neutral_signal_to_teensy(self):
        # Function to serial write 'z' to the Teensy
        try:
            print("Neutral position detected. Sending 'z'to Teensy")
            self.ser.write(f"{Actions.NEUTRAL.value}\n".encode())
        except Exception as e:
            print("Error sending 'z' to Teensy:", e)
            traceback.print_exc()

    def raise_up_pressed(self):
        try:
            self.mode = Modes.NORMAL
            self.action = Actions.RAISE_UP

            print("Raise Up Pressed")

            self.send_mode_action()

        except Exception as e:
            print("Error in raise_up_pressed:", e)
            traceback.print_exc()

    def lower_down_pressed(self):
        try:
            self.mode = Modes.NORMAL
            self.action = Actions.LOWER_DOWN

            print("Lower Down Pressed")

            self.send_mode_action()

        except Exception as e:
            print("Error lower_down_pressed:", e)
            traceback.print_exc()

    def tilt_forward_pressed(self):
        try:
            self.mode = Modes.NORMAL
            self.action = Actions.TILT_FORWARD

            print("Tilt Forward Pressed")

            self.send_mode_action()

        except Exception as e:
            print("Error in tilt_forward_pressed:", e)
            traceback.print_exc()

    def tilt_backward_pressed(self):
        try:
            self.mode = Modes.NORMAL
            self.action = Actions.TILT_BACKWARD

            print("Tilt Backward Pressed")

            self.send_mode_action()

        except Exception as e:
            print("Error in tilt_backward_pressed:", e)
            traceback.print_exc()

    def tilt_left_pressed(self):
        try:
            self.mode = Modes.NORMAL
            self.action = Actions.TILT_LEFT

            print("Tilt Left Pressed")

            self.send_mode_action()

        except Exception as e:
            print("Error in tilt_left_pressed:", e)
            traceback.print_exc()

    def tilt_right_pressed(self):
        try:
            self.mode = Modes.NORMAL
            self.action = Actions.TILT_RIGHT

            print("Tilt Right Pressed")

            self.send_mode_action()

        except Exception as e:
            print("Error in tilt_right_pressed:", e)
            traceback.print_exc()

    def carriage_forward_pressed(self):
        try:
            self.mode = Modes.CARRIAGE
            self.action = Actions.CARRIAGE_FORWARD
            print("Carriage Forward Pressed")

            self.send_mode_action()

            # NOTE: Temporary Solution
            # need to switch to a different mode so motors stop running
            # Look into a different solution but for now this works
            # time.sleep(7)
            # self.raise_up_pressed()
            # self.send_neutral_signal_to_teensy

        except Exception as e:
            print("Error in carriage_forward_pressed:", e)
            traceback.print_exc()

    def carriage_backward_pressed(self):
        try:
            self.mode = Modes.CARRIAGE
            self.action = Actions.CARRIAGE_BACKWARD
            print("Carriage Backward Pressed")

            self.send_mode_action()

            # NOTE: Temporary Solution
            # need to switch to a different mode so motors stop running
            # Look into a different solution but for now this works
            # time.sleep(7)
            # self.raise_up_pressed()
            # self.send_neutral_signal_to_teensy

        except Exception as e:
            print("Error in carriage_backward_pressed:", e)
            traceback.print_exc()

    def self_leveling_pressed(self):
        try:
            self.mode = Modes.SELF_LEVELING
            self.action = Actions.TILT_FORWARD

            print("Self Leveling Pressed")

            self.send_mode_action()
            self.send_neutral_signal_to_teensy()

        except Exception as e:
            print("Error in self_leveling_pressed:", e)
            traceback.print_exc()

    def curb_climb_automation(self):
        try:
            self.mode = Modes.SELF_LEVELING
            self.action = Actions.CURB_ASCEND

            self.send_mode_action()
            self.send_neutral_signal_to_teensy()

        except Exception as e:
            print("Error in curb_climb_automation:", e)
            traceback.print_exc()

    def curb_descend_automation(self):
        try:
            self.mode = Modes.SELF_LEVELING
            self.action = Actions.CURB_DESCEND

            self.send_mode_action()
            self.send_neutral_signal_to_teensy()

        except Exception as e:
            print("Error in curb_descend_automation:", e)
            traceback.print_exc()

    def reset(self):
        try:
            self.mode = Modes.SELF_LEVELING
            self.action = Actions.RESET

            # self.send_mode_action()

            # time.sleep(3)
            # self.send_neutral_signal_to_teensy()

            # NOTE: Temporary Solution
            # need to switch to a different mode so the motors stop
            # self.raise_up_pressed()
            self.send_neutral_signal_to_teensy()

            # Mimic Manual Reset (lower everything)
            # self.FC_UP_pressed()
            # time.sleep(2)
            # self.RC_UP_pressed()
            # time.sleep(2)
            # self.ML_UP_pressed()
            # time.sleep(2)
            # self.MR_UP_pressed()

        except Exception as e:
            print("Error in reset:", e)
            traceback.print_exc()

    def FC_UP_pressed(self):
        print("FC UP pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("r" + "\n").encode())

    def FC_DOWN_pressed(self):
        print("FC DOWN pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("f" + "\n").encode())

    def RC_UP_pressed(self):
        print("RC UP pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("w" + "\n").encode())

    def RC_DOWN_pressed(self):
        print("RC DOWN pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("s" + "\n").encode())

    def ML_UP_pressed(self):
        print("ML UP pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("q" + "\n").encode())

    def ML_DOWN_pressed(self):
        print("ML DOWN pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("a" + "\n").encode())

    def MR_UP_pressed(self):
        print("MR UP pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("e" + "\n").encode())

    def MR_DOWN_pressed(self):
        print("MR DOWN pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("d" + "\n").encode())

    def L_CARRIAGE_FORWARD_pressed(self):
        print("L CARRIAGE FORWARD pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("t" + "\n").encode())

    def L_CARRIAGE_BACKWARD_pressed(self):
        print("L CARRIAGE BACKWARD pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("g" + "\n").encode())

    def R_CARRIAGE_FORWARD_pressed(self):
        print("R CARRIAGE FORWARD pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("y" + "\n").encode())

    def R_CARRIAGE_BACKWARD_pressed(self):
        print("R CARRIAGE BACKWARD pressed")
        self.ser.write(("1" + "\n").encode())
        self.ser.write(("h" + "\n").encode())
