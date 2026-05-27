#!/usr/bin/env python3
from __future__ import annotations

from rclpy.node import Node
from sensor_msgs.msg import Joy


class ManualControlNode(Node):
    """
    Minimal /joy -> Teensy serial forwarder that uses pid_tuner's SerialHandler.

    This must run in the same process as the PID tuner GUI so SerialHandler owns
    the port and we only enqueue commands via SerialHandler.send_command().
    """

    STATE_IDLE = 1
    STATE_TUNER_MODE = 2

    def __init__(self, serial_handler):
        super().__init__("manual_control_node")
        self._serial_handler = serial_handler

        # Local toggle state (do not depend on Teensy telemetry here)
        self.state = self.STATE_IDLE
        self.prev_start_pressed = False

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

    def write_serial_data(self, s: str) -> None:
        self._serial_handler.send_command(s.encode("ascii"))

    def joy_callback(self, msg):
        start_pressed = msg.buttons[9] == 1

        # Rising edge: toggle manual control
        if start_pressed and not self.prev_start_pressed:
            if self.state == self.STATE_TUNER_MODE:
                self.state = self.STATE_IDLE
                self.write_serial_data("T1:0\nT2:0\nT3:0\nT4:0\nT5:0\nT6:0\n")
            else:
                self.state = self.STATE_TUNER_MODE
                self.write_serial_data("M1:0\nM2:0\nM3:0\nM4:0\nM5:0\nM6:0\n")

        self.prev_start_pressed = start_pressed

        if self.state == self.STATE_TUNER_MODE:
            raw_direction = msg.axes[5]
            if (abs(raw_direction)) < 0.15:
                direction = 0
            else:
                direction = 1 if raw_direction > 0 else -1
            buttons_array = list(msg.buttons)
            axes_array = list(msg.axes)
            is_all_zeros = not any(buttons_array) and not any(axes_array)
            if is_all_zeros:
                return

            # Check for odrive velocity
            if abs(axes_array[3]) > 0.15:
                direction = 1 if axes_array[3] > 0 else -1
                self.write_serial_data(f"s:{direction * 3:.4f}\n")

            del buttons_array[8 : len(buttons_array)]
            del buttons_array[1:3]
            buttons_array[2], buttons_array[4] = buttons_array[4], buttons_array[2]
            buttons_array[3], buttons_array[5] = buttons_array[5], buttons_array[3]

            pwm_scale = 0.20
            lines = []
            for i in range(len(buttons_array)):
                id = i + 1
                pwm = (
                    (pwm_scale * direction)
                    if (buttons_array[i] == 1 and direction != 0.0)
                    else 0.0
                )
                lines.append(f"T{id}:{pwm:.2f}\n")
            self.write_serial_data("".join(lines))


def main(args=None):
    raise SystemExit(
        "This node must be started from inside the PID tuner GUI process so it can reuse SerialHandler."
    )
