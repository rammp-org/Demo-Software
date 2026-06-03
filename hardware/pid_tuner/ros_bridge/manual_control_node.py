#!/usr/bin/env python3
from __future__ import annotations

from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from pid_tuner.ros_bridge.luci_drive import (
    DRIVE_WHEEL_JS_THRESHOLD,
    ManualDriveInput,
)


class ManualControlNode(Node):
    """
    Minimal /joy -> Teensy serial forwarder that uses pid_tuner's SerialHandler.

    Drive wheels use the shared LuciClient (Connect LUCI in the GUI). This node
    only updates ManualDriveInput for stick commands; main_window syncs LUCI from
    telemetry + manual sticks + sequence carriage_return.
    """

    STATE_IDLE = 1
    STATE_TUNER_MODE = 2
    STATE_AUTO_CURB_CLIMBING = 6

    def __init__(
        self,
        serial_handler,
        data_store=None,
        manual_drive: ManualDriveInput | None = None,
        luci_client=None,
    ):
        super().__init__("manual_control_node")
        self._serial_handler = serial_handler
        self._data_store = data_store
        self._manual_drive = (
            manual_drive if manual_drive is not None else ManualDriveInput()
        )
        self._luci_client = luci_client

        self.cal_test_pub = self.create_publisher(String, "cal_test", 10)
        self.state = self.STATE_IDLE
        self.prev_start_pressed = False
        self._prev_estop_pressed = False
        self._prev_cal_pressed = False
        self._calibrating = False
        self.odrives_active = False
        self.last_pwm_array = [0, 0, 0, 0, 0, 0]
        self.drive_wheel_active = False

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

    def _luci_stop(self) -> None:
        self._manual_drive.clear_sticks()
        if self._luci_client is not None and self._luci_client.is_connected:
            self._luci_client.request_drive(0, 0)

    def _trigger_estop(self) -> None:
        """Same as PID tuner GUI ESTOP — works in any Teensy state (including sequence)."""
        self._serial_handler.disable_motors()
        self.odrives_active = False
        self.drive_wheel_active = False
        self._calibrating = False
        self._luci_stop()

    def _in_sequence_mode(self) -> bool:
        return (
            self._data_store is not None
            and self._data_store.current_state == self.STATE_AUTO_CURB_CLIMBING
        )

    def joy_callback(self, msg):
        # Button 1: ESTOP (rising edge). Always handled, even outside tuner mode /
        # while AUTO_CURB_CLIMBING sequence is running on the Teensy.
        if len(msg.buttons) > 1:
            estop_pressed = msg.buttons[1] == 1
            if estop_pressed and not self._prev_estop_pressed:
                self._trigger_estop()
            self._prev_estop_pressed = estop_pressed

        start_pressed = len(msg.buttons) > 9 and msg.buttons[9] == 1

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

            cal_pressed = len(msg.buttons) > 2 and msg.buttons[2] == 1
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
                    if not self._in_sequence_mode():
                        self._manual_drive.clear_sticks()
                return

            odrive_js_threshold = 0.5
            if abs(axes_array[3]) > odrive_js_threshold and not self.odrives_active:
                self.odrives_active = True
                direction = 1 if axes_array[3] > 0 else -1
                self.write_serial_data(f"s:{direction * 2:.4f}\n")
            elif abs(axes_array[3]) < odrive_js_threshold and self.odrives_active:
                self.odrives_active = False
                self.write_serial_data("s:0.0000\n")

            # Drive wheels: update shared state (LUCI published from main_window when connected).
            if not self._in_sequence_mode():
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
                    self._manual_drive.set_sticks(True, fb, lr)
                elif self.drive_wheel_active:
                    self.drive_wheel_active = False
                    self._manual_drive.clear_sticks()

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
