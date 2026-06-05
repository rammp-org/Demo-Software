#!/usr/bin/env python3
from __future__ import annotations
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from pid_tuner.ros_bridge.luci_client import LuciClient

DRIVE_WHEEL_JS_THRESHOLD = 2.0


class ManualControlNode(Node):
    """
    Minimal /joy -> Teensy serial forwarder that uses pid_tuner's SerialHandler.

    This must run in the same process as the PID tuner GUI so SerialHandler owns
    the port and we only enqueue commands via SerialHandler.send_command().
    """

    STATE_IDLE = 1
    STATE_TUNER_MODE = 2

    def __init__(self, serial_handler, luci_client: LuciClient):
        super().__init__("manual_control_node")
        self._serial_handler = serial_handler
        self._luci_client = luci_client

        self.cal_test_pub = self.create_publisher(String, "cal_test", 10)
        # Local toggle state (do not depend on Teensy telemetry here)
        self.state = self.STATE_IDLE
        self.prev_start_pressed = False
        self._prev_estop_pressed = False
        self._prev_cal_pressed = False
        self._calibrating = False
        self.odrives_active = False
        self.last_pwm_array = [0, 0, 0, 0, 0, 0]
        self.drive_wheel_active = False
        self._manual_enter_time: float = 0.0
        self.SETTLE_DURATION = 0.5  # seconds

        self.status_pub = self.create_publisher(String, "gamepad_status", 10)
        # self.status_timer = self.create_timer(1.0, self.pub_gamepad)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Listen for firmware status lines (e.g. CAL_DONE) coming from the same
        # SerialHandler used by the GUI. This signal is emitted in the GUI thread.
        if hasattr(self._serial_handler, "raw_lines_received"):
            try:
                self._serial_handler.raw_lines_received.connect(self._on_serial_lines)
            except Exception:
                pass

    # def pub_gamepad(self, message):
    #     msg = String
    #     msg = String(message)
    #     self.status_pub.publish(msg)

    def _on_serial_lines(self, lines) -> None:
        # Unlock joint commands when calibration completes.
        # Teensy prints "CAL_DONE" on completion (see Base.ino).
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

    def _trigger_estop(self) -> None:
        self.state = self.STATE_IDLE
        self.odrives_active = False
        self.drive_wheel_active = False
        self.last_pwm_array = [0, 0, 0, 0, 0, 0]
        self._calibrating = False
        self.write_serial_data("s:0.0000\nT1:0\nT2:0\nT3:0\nT4:0\nT5:0\nT6:0\n")
        self._luci_client.request_stop_drive()
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

        # Rising edge: toggle manual control
        entered_manual = False
        if start_pressed and not self.prev_start_pressed:
            if self.state == self.STATE_IDLE:
                self.status_pub.publish(String(data=msg.axes[0]))
                self.status_pub.publish(String(data=msg.axes[1]))
                self.state = self.STATE_TUNER_MODE
                self.odrives_active = False
                self.drive_wheel_active = False
                self._manual_enter_time = time.monotonic()  # record entry time
                self.write_serial_data("M1:0\nM2:0\nM3:0\nM4:0\nM5:0\nM6:0\ns:0.0000\n")
                self._luci_client.request_stop_drive()
                entered_manual = True
            else:
                self.state = self.STATE_IDLE
                self.write_serial_data("T1:0\nT2:0\nT3:0\nT4:0\nT5:0\nT6:0\n")

        self.prev_start_pressed = start_pressed

        if entered_manual:
            return
        if (time.monotonic() - self._manual_enter_time) < self.SETTLE_DURATION:
            return

        if self.state == self.STATE_TUNER_MODE:
            raw_direction = msg.axes[5]

            # Calibration hotkey (button index 2, if present).
            # Important: send only on rising edge to avoid flooding / restarting.
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
                if self.odrives_active:
                    self.odrives_active = False
                    self.write_serial_data("s:0.0000\n")
                if self.drive_wheel_active:
                    self.drive_wheel_active = False
                    self._luci_client.request_gamepad_drive(0, 0)
                return

            # Check for odrive velocity
            odrive_js_threshold = 2.0
            if abs(axes_array[3]) > odrive_js_threshold and not self.odrives_active:
                self.odrives_active = True
                direction = 1 if axes_array[3] > 0 else -1
                self.write_serial_data(f"s:{direction * 2:.4f}\n")
            elif abs(axes_array[3]) < odrive_js_threshold and self.odrives_active:
                self.odrives_active = False
                self.write_serial_data("s:0.0000\n")

            # Drive wheels via LUCI (axes 0 = fb, 1 = lr; scaled to -100..100)
            axis0 = axes_array[0] if len(axes_array) > 0 else 0.0
            axis1 = axes_array[1] if len(axes_array) > 1 else 0.0
            above = (
                abs(axis0) > DRIVE_WHEEL_JS_THRESHOLD
                or abs(axis1) > DRIVE_WHEEL_JS_THRESHOLD
            )
            if above:
                fb = (
                    int(max(-100, min(100, axis1 * 100)))
                    if abs(axis1) > DRIVE_WHEEL_JS_THRESHOLD
                    else 0
                )
                lr = -1 * (
                    int(max(-100, min(100, axis0 * 100)))
                    if abs(axis0) > DRIVE_WHEEL_JS_THRESHOLD
                    else 0
                )
                self.drive_wheel_active = True
                self._luci_client.request_gamepad_drive(fb, lr)
            elif self.drive_wheel_active:
                self.drive_wheel_active = False
                self._luci_client.request_gamepad_drive(0, 0)

            # Lock out joint button commands while calibration is running.
            # Firmware enters CALIBRATING and ignores normal motor dispatch until
            # it prints CAL_DONE, so suppressing button spam keeps the serial
            # queue clean and prevents confusing partial control.
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
