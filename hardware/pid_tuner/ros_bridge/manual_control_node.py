#!/usr/bin/env python3
from __future__ import annotations

from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
from std_msgs.msg import String

try:
    # Optional: only available when luci_messages is built/sourced.
    from luci_messages.msg import LuciJoystick  # type: ignore
except Exception:  # pragma: no cover
    LuciJoystick = None  # type: ignore

INPUT_REMOTE = 1
DRIVE_WHEEL_JS_THRESHOLD = 0.25


def _compute_joystick_zone(fb: int, lr: int) -> int:
    if fb == 0 and lr == 0:
        return 8  # origin
    if fb > 0 and lr == 0:
        return 0
    if fb < 0 and lr == 0:
        return 7
    if fb == 0 and lr > 0:
        return 4
    if fb == 0 and lr < 0:
        return 3
    if fb > 0 and lr > 0:
        return 2
    if fb > 0 and lr < 0:
        return 1
    if fb < 0 and lr > 0:
        return 6
    return 5


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

        self.cal_test_pub = self.create_publisher(String, "cal_test", 10)
        # Local toggle state (do not depend on Teensy telemetry here)
        self.state = self.STATE_IDLE
        self.prev_start_pressed = False
        self._prev_cal_pressed = False
        self._calibrating = False
        self.odrives_active = False
        self.last_pwm_array = [0, 0, 0, 0, 0, 0]
        self.drive_wheel_active = False
        self._luci_available = LuciJoystick is not None
        self._luci_enabled = False
        self._luci_pub = None
        self._luci_set_auto = None

        if self._luci_available:
            self._luci_pub = self.create_publisher(
                LuciJoystick, "luci/remote_joystick", 10
            )
            self._luci_set_auto = self.create_client(
                Empty, "/luci/set_auto_remote_input"
            )

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Listen for firmware status lines (e.g. CAL_DONE) coming from the same
        # SerialHandler used by the GUI. This signal is emitted in the GUI thread.
        if hasattr(self._serial_handler, "raw_lines_received"):
            try:
                self._serial_handler.raw_lines_received.connect(self._on_serial_lines)
            except Exception:
                pass

        if not self._luci_available:
            self.get_logger().warn(
                "luci_messages not available — drive wheels via LUCI disabled. "
                "Source the workspace that builds luci_messages."
            )

    def _on_serial_lines(self, lines) -> None:
        # Unlock joint commands when calibration completes.
        # Teensy prints "CAL_DONE" on completion (see Base.ino).
        try:
            for line in lines:
                s = str(line).strip()
                if s == "CAL_DONE":
                    self._calibrating = False
                    self.cal_test_pub.publish(String(data="CAL_DONE"))
                elif s.startswith("CAL: Aborted"):
                    self._calibrating = False
        except Exception:
            return

    def write_serial_data(self, s: str) -> None:
        self._serial_handler.send_command(s.encode("ascii"))

    def _luci_enable_auto_input(self) -> None:
        if not self._luci_available or self._luci_enabled:
            return
        if self._luci_set_auto is None or not self._luci_set_auto.service_is_ready():
            return
        try:
            self._luci_set_auto.call_async(Empty.Request())
            self._luci_enabled = True
        except Exception:
            return

    def _luci_publish(self, fb: int, lr: int) -> None:
        if not self._luci_available or self._luci_pub is None:
            return
        self._luci_enable_auto_input()
        fb = int(max(-100, min(100, fb)))
        lr = int(max(-100, min(100, lr)))
        msg = LuciJoystick()
        msg.forward_back = fb
        msg.left_right = lr
        msg.joystick_zone = _compute_joystick_zone(fb, lr)
        if hasattr(msg, "input_source"):
            msg.input_source = INPUT_REMOTE
        self._luci_pub.publish(msg)

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
                # self.write_serial_data("T1:0\nT2:0\nT3:0\nT4:0\nT5:0\nT6:0\n")

        self.prev_start_pressed = start_pressed

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
                    self._luci_publish(0, 0)
                return

            # Check for odrive velocity
            odrive_js_threshold = 0.5
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
                self._luci_publish(fb, lr)
            elif self.drive_wheel_active:
                self.drive_wheel_active = False
                self._luci_publish(0, 0)

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
