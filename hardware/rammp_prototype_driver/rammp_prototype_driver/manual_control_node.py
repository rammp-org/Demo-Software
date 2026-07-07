#!/usr/bin/env python3
from __future__ import annotations
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

FC_MOTOR_JS_THRESHOLD = 0.5
AXIS_NEUTRAL_THRESHOLD = 0.15


class ManualControlNode(Node):
    """
    Minimal /joy -> Teensy serial forwarder.

    Forwards gamepad input for joint motors and front casters over serial.
    Drive wheels are not controlled here.
    """

    STATE_IDLE = 1
    STATE_TUNER_MODE = 2

    def __init__(self, serial_handler, data_store):
        super().__init__("manual_control_node")
        self._serial_handler = serial_handler
        self._data_store = data_store

        self.cal_test_pub = self.create_publisher(String, "cal_test", 10)
        self.state = self.STATE_IDLE
        self.prev_start_pressed = False
        self._prev_estop_pressed = False
        self._prev_cal_pressed = False
        self._calibrating = False
        self.fc_motors_active = False
        self.last_pwm_array = [0, 0, 0, 0, 0, 0]
        self.axes_centered = False

        self.status_pub = self.create_publisher(String, "gamepad_status", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        if hasattr(self._serial_handler, "raw_lines_received"):
            try:
                self._serial_handler.raw_lines_received.connect(self._on_serial_lines)
            except Exception:
                pass

    def _on_serial_lines(self, lines) -> None:
        try:
            for line in lines:
                s = str(line).strip()
                if s == "CAL_DONE":
                    self._calibrating = False
                    if self.state == self.STATE_TUNER_MODE:
                        self.write_serial_data("M1:0\nM2:0\nM3:0\nM4:0\nM5:0\nM6:0\n")
                    self.cal_test_pub.publish(String(data="CAL_DONE"))
                elif s.startswith("CAL: Aborted"):
                    self._calibrating = False
        except Exception:
            return

    def write_serial_data(self, s: str) -> None:
        self._serial_handler.send_command(s.encode("ascii"))

    def _fc_axis_neutral(self, axes_array: list[float]) -> bool:
        axis3 = axes_array[3] if len(axes_array) > 3 else 0.0
        return abs(axis3) <= AXIS_NEUTRAL_THRESHOLD

    def _stop_fc_motors(self) -> None:
        if self._data_store.uses_odrive:
            self.write_serial_data("s:0.0000\n")
        else:
            self.write_serial_data("T9:0.00\nT10:0.00\n")

    def _start_fc_motors(self, direction: int) -> None:
        if self._data_store.uses_odrive:
            self.write_serial_data(f"s:{direction * 2:.4f}\n")
        else:
            pwm = 0.2 * direction
            self.write_serial_data(f"M9:0\nM10:0\nT9:{pwm:.2f}\nT10:{pwm:.2f}\n")

    def _trigger_estop(self) -> None:
        self.state = self.STATE_IDLE
        self.fc_motors_active = False
        self.last_pwm_array = [0, 0, 0, 0, 0, 0]
        self._calibrating = False
        self.write_serial_data(
            "s:0.0000\nT9:0.00\nT10:0.00\nT1:0\nT2:0\nT3:0\nT4:0\nT5:0\nT6:0\n"
        )
        self._serial_handler.disable_motors()

    def joy_callback(self, msg):
        estop_pressed = len(msg.buttons) > 1 and msg.buttons[1] == 1
        if estop_pressed and not self._prev_estop_pressed:
            self._trigger_estop()
            self._prev_estop_pressed = estop_pressed
            self.prev_start_pressed = len(msg.buttons) > 9 and msg.buttons[9] == 1
            return
        self._prev_estop_pressed = estop_pressed

        start_pressed = msg.buttons[9] == 1

        entered_manual = False
        if start_pressed and not self.prev_start_pressed:
            if self.state == self.STATE_IDLE:
                axes_array = list(msg.axes)
                if not self._fc_axis_neutral(axes_array):
                    self.axes_centered = False
                    self.status_pub.publish(
                        String(
                            data=("Center FC stick (axis 3), then press Start again")
                        )
                    )
                else:
                    self.axes_centered = True
                    self.state = self.STATE_TUNER_MODE
                    self.fc_motors_active = False
                    self.write_serial_data(
                        "M1:0\nM2:0\nM3:0\nM4:0\nM5:0\nM6:0\n"
                        "s:0.0000\nT9:0.00\nT10:0.00\n"
                    )
                    entered_manual = True
            else:
                self.state = self.STATE_IDLE
                self.axes_centered = False
                self.write_serial_data("T1:0\nT2:0\nT3:0\nT4:0\nT5:0\nT6:0\n")

        self.prev_start_pressed = start_pressed

        if entered_manual:
            return

        if self.state == self.STATE_TUNER_MODE:
            raw_direction = msg.axes[5]

            cal_pressed = msg.buttons[2] == 1
            if cal_pressed and not self._prev_cal_pressed:
                self._calibrating = True
                self.write_serial_data("W0:-0.20\n")
            self._prev_cal_pressed = cal_pressed

            if (abs(raw_direction)) < 0.15:
                direction = 0
            else:
                direction = 1 if raw_direction > 0 else -1
            buttons_array = list(msg.buttons)
            axes_array = list(msg.axes)
            buttons_all_zeros = not any(buttons_array)
            axes_all_zeros = not any(abs(axis) > 0.15 for axis in axes_array)
            if buttons_all_zeros and axes_all_zeros:
                if self.fc_motors_active:
                    self.fc_motors_active = False
                    self._stop_fc_motors()
                return

            if abs(axes_array[3]) > FC_MOTOR_JS_THRESHOLD and not self.fc_motors_active:
                self.fc_motors_active = True
                fc_direction = 1 if axes_array[3] > 0 else -1
                self._start_fc_motors(fc_direction)
            elif abs(axes_array[3]) < FC_MOTOR_JS_THRESHOLD and self.fc_motors_active:
                self.fc_motors_active = False
                self._stop_fc_motors()

            if self._calibrating:
                return

            del buttons_array[8 : len(buttons_array)]
            del buttons_array[1:3]
            buttons_array[2], buttons_array[4] = buttons_array[4], buttons_array[2]
            buttons_array[3], buttons_array[5] = buttons_array[5], buttons_array[3]

            pwm_scale = 0.30
            lines = []
            for i in range(len(buttons_array)):
                id = i + 1
                if i == 4 or i == 5:
                    pwm = -1 * (
                        (pwm_scale * direction)
                        if (buttons_array[i] == 1 and direction != 0.0)
                        else 0.0
                    )
                else:
                    pwm = (
                        (pwm_scale * direction)
                        if (buttons_array[i] == 1 and direction != 0.0)
                        else 0.0
                    )

                if pwm != self.last_pwm_array[i]:
                    self.last_pwm_array[i] = pwm
                    lines.append(f"T{id}:{pwm:.2f}\n")

            self.write_serial_data("".join(lines))


def main(args=None):
    raise SystemExit(
        "This node must be started from inside the PID tuner GUI process so it can reuse SerialHandler."
    )
